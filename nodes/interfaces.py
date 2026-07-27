"""Read-only ROS 2 interface checks driven entirely by robot profiles."""
from __future__ import annotations

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
