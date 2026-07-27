"""Profile-bound robot capability contracts and read-only inspection."""
from __future__ import annotations

import copy
import re
from typing import Any

from blacknode.contracts import robot_capability_binding, robot_capability_status
from blacknode.node import Bool, Dict, Int, List, Text, node


_CATEGORY = "Robot"
_CAPABILITY_ID = re.compile(r"^[a-z][a-z0-9_]{0,63}$")


def _identifier(value: Any, fallback: str = "") -> str:
    text = re.sub(r"[^a-z0-9]+", "_", str(value or "").strip().lower()).strip("_")
    if not text:
        return fallback
    if not text[0].isalpha():
        text = f"capability_{text}"
    return text[:64]


def _provider_name(value: Any) -> str:
    return re.sub(
        r"[^a-z0-9_.-]+",
        "-",
        str(value or "").strip().lower(),
    ).strip("-")[:96]


def _hardware_identity(value: Any = None, explicit_id: Any = "") -> dict[str, Any]:
    hardware = value if isinstance(value, dict) else {}
    recommended = hardware.get("recommended") if isinstance(hardware.get("recommended"), dict) else {}
    source = recommended or hardware
    identity = {
        "id": str(
            explicit_id
            or source.get("serial")
            or source.get("serial_number")
            or hardware.get("serial")
            or hardware.get("serial_number")
            or source.get("path")
            or hardware.get("path")
            or ""
        ).strip(),
        "serial": str(source.get("serial") or source.get("serial_number") or "").strip(),
        "vendor_id": str(source.get("vendor_id") or "").strip().lower().removeprefix("0x"),
        "product_id": str(source.get("product_id") or "").strip().lower().removeprefix("0x"),
        "path": str(source.get("path") or "").strip(),
    }
    return {key: item for key, item in identity.items() if item}


def _provider_ref(provider: dict[str, Any]) -> str:
    package = str(provider.get("package") or "").strip()
    component = str(provider.get("component") or "").strip()
    adapter = str(provider.get("adapter") or "").strip()
    reference = f"{package}/{component}" if package and component else package or component
    return f"{reference}@{adapter}" if reference and adapter else reference


def _binding_values(profile: dict[str, Any]) -> list[dict[str, Any]]:
    raw = profile.get("capability_bindings")
    if isinstance(raw, dict):
        values = raw.values()
    elif isinstance(raw, list):
        values = raw
    else:
        values = []
    return [copy.deepcopy(value) for value in values if isinstance(value, dict)]


def _normalized_provider_states(value: Any) -> dict[str, Any]:
    states = value if isinstance(value, dict) else {}
    if isinstance(states.get("bindings"), dict):
        states = states["bindings"]
    if isinstance(states.get("capabilities"), dict):
        states = states["capabilities"]
    return dict(states)


def _installed_refs(values: Any) -> set[str]:
    if not isinstance(values, list):
        return set()
    return {str(value or "").strip() for value in values if str(value or "").strip()}


def _provider_is_installed(provider: dict[str, Any], installed: set[str]) -> bool:
    if not installed:
        return True
    reference = _provider_ref(provider)
    base = reference.split("@", 1)[0]
    package = str(provider.get("package") or "").strip()
    return reference in installed or base in installed or package in installed


def _observed_state(binding: dict[str, Any], states: dict[str, Any]) -> Any:
    capability = str(binding.get("capability") or "")
    provider = binding.get("provider") if isinstance(binding.get("provider"), dict) else {}
    configuration = (
        binding.get("configuration")
        if isinstance(binding.get("configuration"), dict)
        else {}
    )
    keys = [
        str(configuration.get("state_key") or "").strip(),
        capability,
        _provider_ref(provider),
        _provider_ref(provider).split("@", 1)[0],
    ]
    for key in keys:
        if key and key in states:
            return states[key]
    return None


def _classify_observed_state(value: Any) -> tuple[str, str]:
    if isinstance(value, bool):
        return ("available", "provider reports ready") if value else (
            "unavailable", "provider reports unavailable",
        )
    if isinstance(value, str):
        state = value.strip().lower()
        if state in {"available", "ready", "running", "connected", "healthy"}:
            return "available", f"provider reports {state}"
        if state in {"unhealthy", "failed", "error", "stale", "degraded"}:
            return "unhealthy", f"provider reports {state}"
        return "unavailable", f"provider reports {state or 'no state'}"
    if isinstance(value, dict):
        state = str(value.get("state") or value.get("status") or "").strip().lower()
        reason = str(
            value.get("reason")
            or value.get("error")
            or value.get("note")
            or value.get("matched")
            or ""
        ).strip()
        if value.get("healthy") is False or state in {
            "unhealthy", "failed", "error", "stale", "degraded",
        }:
            return "unhealthy", reason or f"provider reports {state or 'unhealthy'}"
        if value.get("available") is True or value.get("ready") is True or state in {
            "available", "ready", "running", "connected", "healthy",
        }:
            return "available", reason or "provider reports ready"
        if value.get("available") is False or value.get("ready") is False or state:
            return "unavailable", reason or f"provider reports {state or 'unavailable'}"
    return "unavailable", "provider readiness was not reported"


@node(
    name="RobotCapabilityBinding",
    component="capabilities",
    category=_CATEGORY,
    description=(
        "Bind one semantic robot capability to a replaceable package component "
        "and optional adapter. This only declares the provider; it never starts motion."
    ),
    inputs={
        "capability": Text(default="camera"),
        "provider_package": Text(default="blacknode-perception"),
        "provider_component": Text(default="camera"),
        "provider_adapter": Text(default=""),
        "configuration": Dict,
        "hardware_id": Text(default=""),
        "hardware": Dict,
        "required": Bool(default=True),
    },
    outputs={
        "valid": Bool,
        "binding": Dict,
        "capability": Text,
        "provider_ref": Text,
        "report": Text,
    },
)
def robot_capability_binding_node(ctx: dict) -> dict:
    capability = _identifier(ctx.get("capability"))
    provider_package = str(ctx.get("provider_package") or "").strip()
    provider_component = _provider_name(ctx.get("provider_component"))
    provider_adapter = _provider_name(ctx.get("provider_adapter"))
    identity = _hardware_identity(ctx.get("hardware"), ctx.get("hardware_id"))
    binding = robot_capability_binding(
        capability,
        provider_package=provider_package,
        provider_component=provider_component,
        provider_adapter=provider_adapter,
        configuration=dict(ctx.get("configuration") or {}),
        hardware_identity=identity,
        required=bool(ctx.get("required", True)),
    )
    provider_ref = _provider_ref(binding["provider"])
    errors = []
    if not _CAPABILITY_ID.fullmatch(capability):
        errors.append("capability needs a lowercase stable id")
    if not provider_package:
        errors.append("provider package is required")
    if not provider_component:
        errors.append("provider component is required")
    return {
        "valid": not errors,
        "binding": binding,
        "capability": capability,
        "provider_ref": provider_ref,
        "report": (
            f"{capability or 'invalid capability'} -> {provider_ref or 'provider not configured'}"
            + (f" · hardware {identity['id']}" if identity.get("id") else "")
            + ("\nINVALID: " + "; ".join(errors) if errors else "\nDECLARED: no provider was started")
        ),
    }


@node(
    name="RobotCapabilityList",
    component="capabilities",
    category=_CATEGORY,
    description="Collect capability bindings for a robot profile.",
    inputs={"binding_1": Dict},
    outputs={"bindings": List, "capabilities": List, "count": Int, "report": Text},
    variadic_input=Dict,
    variadic_prefix="binding",
)
def robot_capability_list(ctx: dict) -> dict:
    def sort_key(name: str) -> tuple[int, str]:
        suffix = name.rsplit("_", 1)[-1]
        return (int(suffix), name) if suffix.isdigit() else (999_999, name)

    names = sorted((name for name in ctx if name.startswith("binding_")), key=sort_key)
    bindings = [
        copy.deepcopy(ctx[name])
        for name in names
        if isinstance(ctx.get(name), dict) and ctx[name]
    ]
    capabilities = [str(value.get("capability") or "") for value in bindings]
    return {
        "bindings": bindings,
        "capabilities": capabilities,
        "count": len(bindings),
        "report": f"assembled {len(bindings)} capability binding(s)",
    }


@node(
    name="RobotCapabilityProfile",
    component="capabilities",
    category=_CATEGORY,
    description=(
        "Attach replaceable capability providers and stable hardware identity "
        "to a robot profile."
    ),
    inputs={
        "profile": Dict,
        "profile_id": Text(default="my_robot"),
        "display_name": Text(default="My Robot"),
        "bindings": List,
        "hardware_id": Text(default=""),
        "hardware": Dict,
    },
    outputs={
        "valid": Bool,
        "profile": Dict,
        "capabilities": List,
        "hardware_identity": Dict,
        "report": Text,
    },
)
def robot_capability_profile(ctx: dict) -> dict:
    profile = copy.deepcopy(ctx.get("profile") or {}) if isinstance(ctx.get("profile"), dict) else {}
    profile_id = _identifier(profile.get("id") or ctx.get("profile_id"), "my_robot")
    display_name = str(
        profile.get("display_name")
        or ctx.get("display_name")
        or profile_id
    ).strip()
    values = [
        copy.deepcopy(value)
        for value in (ctx.get("bindings") or [])
        if isinstance(value, dict)
    ]
    binding_map: dict[str, dict[str, Any]] = {}
    errors: list[str] = []
    for value in values:
        capability = _identifier(value.get("capability"))
        provider = value.get("provider") if isinstance(value.get("provider"), dict) else {}
        if not capability or not provider.get("package") or not provider.get("component"):
            errors.append("every binding needs a capability, provider package, and provider component")
            continue
        if capability in binding_map:
            errors.append(f"capability '{capability}' is bound more than once")
            continue
        value["capability"] = capability
        binding_map[capability] = value
    identity = _hardware_identity(ctx.get("hardware"), ctx.get("hardware_id"))
    if not identity:
        identity = copy.deepcopy(profile.get("hardware_identity") or {})
    profile.update({
        "kind": "blacknode.robot-profile",
        "schema_version": 1,
        "id": profile_id,
        "profile_id": profile_id,
        "display_name": display_name,
        "capabilities": list(binding_map),
        "capability_bindings": binding_map,
        "hardware_identity": identity,
    })
    if not binding_map:
        errors.append("add at least one capability binding")
    return {
        "valid": not errors,
        "profile": profile,
        "capabilities": list(binding_map),
        "hardware_identity": identity,
        "report": (
            f"capability profile: {display_name} ({profile_id}) · "
            f"{len(binding_map)} capability binding(s)"
            + (f" · hardware {identity['id']}" if identity.get("id") else " · reusable hardware identity")
            + ("\nINVALID:\n- " + "\n- ".join(errors) if errors else "")
        ),
    }


@node(
    name="RobotCapabilityInspect",
    component="capabilities",
    category=_CATEGORY,
    description=(
        "Resolve a robot profile against installed components and live provider "
        "reports. Read-only: unavailable or unhealthy providers never break discovery."
    ),
    inputs={
        "profile": Dict,
        "provider_states": Dict,
        "installed_components": List(default=[]),
        "hardware_id": Text(default=""),
        "hardware": Dict,
    },
    outputs={
        "ready": Bool,
        "capabilities": List,
        "available": List,
        "unavailable": List,
        "unhealthy": List,
        "summary": Dict,
        "report": Text,
    },
)
def robot_capability_inspect(ctx: dict) -> dict:
    profile = copy.deepcopy(ctx.get("profile") or {}) if isinstance(ctx.get("profile"), dict) else {}
    bindings = _binding_values(profile)
    states = _normalized_provider_states(ctx.get("provider_states"))
    installed = _installed_refs(ctx.get("installed_components"))
    hardware_report = ctx.get("hardware") if isinstance(ctx.get("hardware"), dict) else {}
    current_identity = _hardware_identity(ctx.get("hardware"), ctx.get("hardware_id"))
    if not current_identity:
        current_identity = copy.deepcopy(profile.get("hardware_identity") or {})
    hardware_state = (
        current_identity.get("id")
        or (
            "connected"
            if bool(hardware_report.get("ready"))
            else "disconnected"
            if hardware_report
            else "not configured"
        )
    )
    statuses: list[dict[str, Any]] = []
    lines = [
        f"Robot capability readiness · {profile.get('display_name') or profile.get('id') or 'unnamed robot'}",
        f"hardware: {hardware_state}",
        "",
    ]
    if not bindings:
        lines.append(
            "[UNCONFIGURED] Add capability bindings to the robot profile. "
            "The profile determines the robot's shape and providers."
        )
    for binding in bindings:
        capability = _identifier(binding.get("capability"), "invalid")
        provider = binding.get("provider") if isinstance(binding.get("provider"), dict) else {}
        required = bool(binding.get("required", True))
        expected_identity = (
            binding.get("hardware_identity")
            if isinstance(binding.get("hardware_identity"), dict)
            else {}
        )
        expected_id = str(expected_identity.get("id") or "").strip()
        current_id = str(current_identity.get("id") or "").strip()
        if expected_id and not current_id:
            state, reason = "unavailable", f"requires physical hardware {expected_id}"
        elif expected_id and expected_id != current_id:
            state, reason = "unhealthy", f"bound to {expected_id}, but {current_id} is connected"
        elif not _provider_is_installed(provider, installed):
            state, reason = "unavailable", f"provider component {_provider_ref(provider)} is not installed"
        else:
            state, reason = _classify_observed_state(_observed_state(binding, states))
        status = robot_capability_status(
            capability,
            state=state,
            provider=provider,
            required=required,
            reason=reason,
            hardware_identity=expected_identity or current_identity,
        )
        statuses.append(status)
        lines.append(
            f"[{state.upper()}] {capability} -> {_provider_ref(provider) or 'provider not configured'}"
            f"{' (required)' if required else ' (optional)'}: {reason}"
        )
    available = [item["capability"] for item in statuses if item["state"] == "available"]
    unavailable = [item["capability"] for item in statuses if item["state"] == "unavailable"]
    unhealthy = [item["capability"] for item in statuses if item["state"] == "unhealthy"]
    blocking = [
        item["capability"]
        for item in statuses
        if item["required"] and item["state"] != "available"
    ]
    ready = bool(statuses) and not blocking
    lines.extend([
        "",
        (
            "READY: every required provider reports available."
            if ready
            else (
                "UNCONFIGURED: connect a profile that declares this robot's capabilities."
                if not statuses
                else "ATTENTION: required providers need setup or a healthy live report."
            )
        ),
        "Capability inspection is read-only. No motion was authorized or commanded.",
    ])
    return {
        "ready": ready,
        "capabilities": statuses,
        "available": available,
        "unavailable": unavailable,
        "unhealthy": unhealthy,
        "summary": {
            "profile_id": str(profile.get("id") or profile.get("profile_id") or ""),
            "hardware_id": str(current_identity.get("id") or ""),
            "total": len(statuses),
            "available": len(available),
            "unavailable": len(unavailable),
            "unhealthy": len(unhealthy),
            "blocking": blocking,
        },
        "report": "\n".join(lines),
    }
