"""Read-only ROS 2 capability discovery and profile interface checks."""
from __future__ import annotations

import re
from typing import Any

from blacknode.node import Bool, Dict, List, Text, node


_CATEGORY = "Robot"


def _ros_names(values: Any) -> set[str]:
    """Normalize `ros2 * list -t` rows to graph names."""
    if not isinstance(values, list):
        return set()
    names: set[str] = set()
    for value in values:
        name = str(value or "").strip().split(maxsplit=1)[0]
        if name:
            names.add(name)
    return names


def _ros_topic_entries(values: Any) -> list[dict[str, Any]]:
    """Normalize `ros2 topic list -t` rows and future structured inventories."""
    if not isinstance(values, list):
        return []
    entries: list[dict[str, Any]] = []
    for value in values:
        if isinstance(value, dict):
            name = str(value.get("name") or value.get("topic") or "").strip()
            raw_types = (
                value.get("types")
                if isinstance(value.get("types"), list)
                else [value.get("message_type") or value.get("type")]
            )
            message_types = [
                str(item or "").strip()
                for item in raw_types
                if str(item or "").strip()
            ]
        else:
            row = str(value or "").strip()
            if not row:
                continue
            name = row.split(maxsplit=1)[0]
            groups = re.findall(r"\[([^\]]*)\]", row)
            message_types = [
                item.strip()
                for group in groups
                for item in group.split(",")
                if item.strip()
            ]
        if name:
            entries.append({
                "name": name,
                "message_types": sorted(set(message_types)),
            })
    return entries


def _topic_capability_evidence(
    name: str,
    message_type: str,
) -> list[dict[str, Any]]:
    """Classify standard ROS interfaces without assuming a robot vendor."""
    topic = name.lower()
    msg = message_type.lower().strip()
    evidence: list[dict[str, Any]] = []

    def add(capability: str, role: str, score: int, reason: str) -> None:
        evidence.append({
            "capability": capability,
            "kind": "topic",
            "name": name,
            "message_type": message_type,
            "role": role,
            "score": score,
            "reason": reason,
        })

    if msg.endswith("/laserscan"):
        add("lidar", "state", 100, "standard LaserScan telemetry")
    elif msg.endswith("/pointcloud2"):
        score = 85 if any(token in topic for token in ("lidar", "laser", "scan")) else 70
        add(
            "lidar",
            "state",
            score,
            "point-cloud telemetry; sensor source needs confirmation",
        )
    elif msg.endswith("/imu"):
        add("imu", "state", 100, "standard IMU telemetry")
    elif msg.endswith("/batterystate"):
        add("battery", "state", 100, "standard battery telemetry")
    elif msg.endswith("/navsatfix"):
        add("gps", "state", 100, "standard GNSS telemetry")
    elif msg.endswith("/odometry"):
        add("mobile_base", "state", 85, "standard base odometry telemetry")
    elif msg.endswith("/twist") and "cmd_vel" in topic:
        add(
            "mobile_base",
            "command",
            85,
            "velocity command interface; direction and safety need confirmation",
        )
    elif msg.endswith("/jointstate"):
        add("joint_state", "state", 100, "standard joint feedback telemetry")
    elif msg.endswith("/compressedimage"):
        capability = "depth_camera" if "depth" in topic else "camera"
        add(capability, "state", 95, "standard compressed image telemetry")
    elif msg.endswith("/image"):
        capability = "depth_camera" if "depth" in topic else "camera"
        add(capability, "state", 100, "standard image telemetry")
    elif msg.endswith("/camerainfo"):
        capability = "depth_camera" if "depth" in topic else "camera"
        add(capability, "metadata", 65, "camera calibration metadata")
    elif msg.endswith("/audiodata") or msg.endswith("/audio"):
        add("microphone", "state", 90, "audio telemetry")
    return evidence


def _confidence(score: int) -> str:
    if score >= 90:
        return "high"
    if score >= 65:
        return "medium"
    return "low"


@node(
    name="RobotROSCapabilityDiscover",
    component="capabilities",
    category=_CATEGORY,
    description=(
        "Infer generic robot capability candidates from a live ROS 2 graph. "
        "Read-only: reports evidence and never binds or commands hardware."
    ),
    inputs={
        "topics": List(default=[]),
        "nodes": List(default=[]),
        "services": List(default=[]),
    },
    outputs={
        "found": Bool,
        "capabilities": List,
        "unclassified": List,
        "inventory": Dict,
        "report": Text,
    },
)
def robot_ros_capability_discover(ctx: dict) -> dict:
    topic_entries = _ros_topic_entries(ctx.get("topics"))
    nodes = sorted(_ros_names(ctx.get("nodes")))
    services = sorted(_ros_names(ctx.get("services")))
    grouped: dict[str, list[dict[str, Any]]] = {}
    classified_topics: set[str] = set()
    for entry in topic_entries:
        name = entry["name"]
        for message_type in entry["message_types"]:
            for evidence in _topic_capability_evidence(name, message_type):
                grouped.setdefault(evidence["capability"], []).append(evidence)
                classified_topics.add(name)

    candidates: list[dict[str, Any]] = []
    for capability, evidence in grouped.items():
        roles = {str(item["role"]) for item in evidence}
        score = max(int(item["score"]) for item in evidence)
        if capability == "mobile_base" and {"state", "command"} <= roles:
            score = max(score, 95)
        state_topics = sorted({
            str(item["name"])
            for item in evidence
            if item["role"] in {"state", "metadata"}
        })
        command_topics = sorted({
            str(item["name"])
            for item in evidence
            if item["role"] == "command"
        })
        candidates.append({
            "kind": "blacknode.robot-capability-candidate",
            "schema_version": 1,
            "capability": capability,
            "confidence": _confidence(score),
            "score": score,
            "state_topics": state_topics,
            "command_topics": command_topics,
            "safe_to_read": bool(state_topics),
            "requires_confirmation": True,
            "evidence": sorted(
                evidence,
                key=lambda item: (-int(item["score"]), str(item["name"])),
            ),
        })
    candidates.sort(key=lambda item: (-int(item["score"]), str(item["capability"])))
    unclassified = [
        entry
        for entry in topic_entries
        if entry["name"] not in classified_topics
    ]

    lines = [
        "Generic ROS 2 capability discovery",
        (
            f"observed: {len(topic_entries)} topic(s), "
            f"{len(nodes)} node(s), {len(services)} service(s)"
        ),
        "",
    ]
    if candidates:
        for candidate in candidates:
            evidence_topics = candidate["state_topics"] + candidate["command_topics"]
            lines.append(
                f"[{candidate['confidence'].upper()}] {candidate['capability']}: "
                f"{', '.join(evidence_topics)}"
            )
    else:
        lines.append("[NONE] No capability candidates were identified.")
    lines.extend([
        "",
        (
            "Discovery is read-only. Confirm each candidate before creating a "
            "provider binding; command topics were not published."
        ),
    ])
    return {
        "found": bool(candidates),
        "capabilities": candidates,
        "unclassified": unclassified,
        "inventory": {
            "topics": topic_entries,
            "nodes": nodes,
            "services": services,
            "classified_topic_count": len(classified_topics),
            "unclassified_topic_count": len(unclassified),
        },
        "report": "\n".join(lines),
    }


def _profile_bindings(profile: dict[str, Any]) -> list[dict[str, Any]]:
    raw = profile.get("capability_bindings")
    values = raw.values() if isinstance(raw, dict) else raw if isinstance(raw, list) else []
    return [dict(value) for value in values if isinstance(value, dict)]


def _interface_specs(binding: dict[str, Any]) -> list[dict[str, Any]]:
    configuration = (
        binding.get("configuration")
        if isinstance(binding.get("configuration"), dict)
        else {}
    )
    raw = configuration.get("ros2_interfaces")
    if not isinstance(raw, list):
        raw = configuration.get("interfaces")
    if isinstance(raw, list):
        return [dict(value) for value in raw if isinstance(value, dict)]
    kind = str(configuration.get("interface_kind") or "").strip().lower()
    candidates = configuration.get("candidates")
    contains = configuration.get("contains")
    if kind or isinstance(candidates, list) or isinstance(contains, list):
        return [{
            "kind": kind or "topic",
            "candidates": list(candidates or []),
            "contains": list(contains or []),
            "required": True,
        }]
    return []


def _observed_for_kind(
    kind: str,
    *,
    topics: set[str],
    nodes: set[str],
    services: set[str],
) -> set[str]:
    if kind == "node":
        return nodes
    if kind == "service":
        return services
    return topics


def _match_interface(
    spec: dict[str, Any],
    *,
    topics: set[str],
    nodes: set[str],
    services: set[str],
) -> dict[str, Any]:
    kind = str(spec.get("kind") or "topic").strip().lower()
    observed = _observed_for_kind(
        kind,
        topics=topics,
        nodes=nodes,
        services=services,
    )
    candidates = [
        str(value or "").strip()
        for value in (spec.get("candidates") or [])
        if str(value or "").strip()
    ]
    contains = [
        str(value or "").strip().lower()
        for value in (spec.get("contains") or [])
        if str(value or "").strip()
    ]
    exact = next((name for name in candidates if name in observed), "")
    fuzzy = next(
        (
            name
            for name in sorted(observed)
            if contains and any(token in name.lower() for token in contains)
        ),
        "",
    )
    matched = exact or fuzzy
    return {
        "kind": kind,
        "available": bool(matched),
        "matched": matched,
        "candidates": candidates,
        "contains": contains,
        "required": bool(spec.get("required", True)),
        "label": str(spec.get("label") or "").strip(),
        "note": str(spec.get("note") or "").strip(),
    }


@node(
    name="RobotROSInterfaceCheck",
    component="capabilities",
    category=_CATEGORY,
    description=(
        "Match a live ROS 2 graph against the interfaces declared by a robot "
        "profile. Read-only: discovers bindings and never publishes commands."
    ),
    inputs={
        "profile": Dict,
        "topics": List(default=[]),
        "nodes": List(default=[]),
        "services": List(default=[]),
    },
    outputs={
        "ready": Bool,
        "capabilities": List,
        "missing": List,
        "bindings": Dict,
        "report": Text,
    },
)
def robot_ros_interface_check(ctx: dict) -> dict:
    profile = dict(ctx.get("profile") or {}) if isinstance(ctx.get("profile"), dict) else {}
    topics = _ros_names(ctx.get("topics"))
    nodes = _ros_names(ctx.get("nodes"))
    services = _ros_names(ctx.get("services"))
    declared = _profile_bindings(profile)
    capability_states: dict[str, dict[str, Any]] = {}
    for binding in declared:
        capability = str(binding.get("capability") or "").strip()
        if not capability:
            continue
        specs = _interface_specs(binding)
        checked = [
            _match_interface(
                spec,
                topics=topics,
                nodes=nodes,
                services=services,
            )
            for spec in specs
        ]
        required_checks = [value for value in checked if value["required"]]
        matched = [value["matched"] for value in checked if value["matched"]]
        available = bool(checked) and (
            all(value["available"] for value in required_checks)
            if required_checks
            else bool(matched)
        )
        missing_required = [
            value
            for value in required_checks
            if not value["available"]
        ]
        capability_states[capability] = {
            "label": str(binding.get("label") or capability.replace("_", " ").title()),
            "available": available,
            "matched": matched[0] if len(matched) == 1 else "",
            "matches": matched,
            "required": bool(binding.get("required", True)),
            "interfaces": checked,
            "note": (
                "profile does not declare ROS 2 interfaces"
                if not specs
                else "; ".join(
                    value["note"]
                    for value in missing_required
                    if value["note"]
                )
            ),
        }
    available_capabilities = [
        name for name, value in capability_states.items() if value["available"]
    ]
    missing = [
        name for name, value in capability_states.items() if not value["available"]
    ]
    missing_required = [
        name
        for name, value in capability_states.items()
        if value["required"] and not value["available"]
    ]
    ready = bool(capability_states) and not missing_required
    lines = [
        "Generic ROS 2 robot interface readiness",
        f"profile: {profile.get('display_name') or profile.get('id') or 'not connected'}",
        f"observed: {len(topics)} topic(s), {len(nodes)} node(s), {len(services)} service(s)",
        "",
    ]
    if not capability_states:
        lines.append("[UNCONFIGURED] Connect a robot profile with capability bindings.")
    for capability, value in capability_states.items():
        if value["available"]:
            state = "AVAILABLE"
            detail = ", ".join(value["matches"])
        elif not value["interfaces"]:
            state = "UNCONFIGURED"
            detail = value["note"]
        else:
            state = "UNAVAILABLE"
            expected = []
            for interface in value["interfaces"]:
                expected.extend(interface["candidates"])
                expected.extend(interface["contains"])
            detail = ", ".join(expected) or "declared ROS 2 interface"
        lines.append(f"[{state}] {value['label']}: {detail}")
    lines.extend([
        "",
        (
            "READY: every required profile-declared ROS 2 interface was discovered."
            if ready
            else (
                "NEXT: connect a configured robot profile, then start its declared "
                "providers and run this check again. No motion command was sent."
            )
        ),
    ])
    return {
        "ready": ready,
        "capabilities": available_capabilities,
        "missing": missing,
        "bindings": {
            "profile_id": str(profile.get("id") or profile.get("profile_id") or ""),
            "topics": sorted(topics),
            "nodes": sorted(nodes),
            "services": sorted(services),
            "capabilities": capability_states,
        },
        "report": "\n".join(lines),
    }
