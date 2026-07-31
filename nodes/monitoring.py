"""Read-only robot monitoring targets and provider-neutral raw discovery."""

from __future__ import annotations

from collections.abc import Mapping
from typing import Any

from blacknode.node import Bool, Dict, Int, List, Text, node
from blacknode.node import _NODE_REGISTRY


_RAW_MONITOR_PROVIDER_ATTRIBUTE = "_bn_robot_raw_monitor_provider"


def _raw_monitor_providers() -> list[dict[str, Any]]:
    providers: list[dict[str, Any]] = []
    for fn in _NODE_REGISTRY.values():
        value = getattr(fn, _RAW_MONITOR_PROVIDER_ATTRIBUTE, None)
        if not isinstance(value, Mapping) or not callable(value.get("sample")):
            continue
        providers.append(dict(value))
    return providers


def _raw_provider_score(
    provider: Mapping[str, Any],
    hardware: Mapping[str, Any],
) -> int:
    selected = (
        hardware.get("raw_monitor_provider")
        if isinstance(hardware.get("raw_monitor_provider"), Mapping)
        else {}
    )
    if selected:
        if (
            str(selected.get("package") or "") == str(provider.get("package") or "")
            and str(selected.get("component") or "")
            == str(provider.get("component") or "")
        ):
            return 10_000
        return 0
    matcher = provider.get("match_hardware")
    if not callable(matcher):
        return 0
    try:
        value = matcher(hardware)
    except Exception:
        return 0
    if isinstance(value, bool):
        return 1 if value else 0
    try:
        return max(0, int(value))
    except (TypeError, ValueError):
        return 0


def _select_raw_monitor_provider(
    hardware: Mapping[str, Any],
) -> dict[str, Any]:
    ranked = [
        (_raw_provider_score(provider, hardware), provider)
        for provider in _raw_monitor_providers()
    ]
    best_score = max((score for score, _provider in ranked), default=0)
    best = [provider for score, provider in ranked if score == best_score and score > 0]
    if len(best) == 1:
        return best[0]
    if len(best) > 1:
        names = ", ".join(
            sorted(
                f"{provider.get('package')}/{provider.get('component')}"
                for provider in best
            )
        )
        raise RuntimeError(
            "raw monitoring found multiple equally suitable read-only providers "
            f"({names}); select a robot profile to identify the protocol"
        )
    available = ", ".join(
        sorted(
            f"{provider.get('package')}/{provider.get('component')}"
            for provider in _raw_monitor_providers()
        )
    )
    suffix = f"; installed providers: {available}" if available else ""
    raise RuntimeError(
        "no installed read-only provider recognizes this hardware"
        f"{suffix}"
    )


@node(
    name="RobotMonitor",
    component="capabilities",
    category="Robot",
    inputs={
        "robot_id": Text(default=""),
        "robot_name": Text(default=""),
        "profile_id": Text(default="auto"),
    },
    outputs={"robot": Dict},
    primary_inputs=[],
    primary_outputs=["robot"],
    description="Show a registered robot's live state, telemetry, and streams on the canvas.",
)
def robot_monitor(
    robot_id: str = "",
    robot_name: str = "",
    profile_id: str = "auto",
) -> dict:
    """Describe the stable, read-only target used by the live canvas monitor."""
    target_id = str(robot_id or "").strip()
    target_name = str(robot_name or "").strip()
    return {
        "robot": {
            "kind": "blacknode.robot-monitor-target",
            "schema_version": 1,
            "robot_id": target_id,
            "robot_name": target_name,
            "profile_id": str(profile_id or "auto").strip() or "auto",
            "configured": bool(target_id),
        },
    }


@node(
    name="RobotRawMonitor",
    component="capabilities",
    category="Robot",
    hidden=True,
    inputs={
        "hardware": Dict,
        "max_servo_id": Int(default=32),
        "provider_config": Dict,
    },
    outputs={
        "available": Bool,
        "joints": List,
        "position_unit": Text,
        "velocity_unit": Text,
        "torque_enabled": Bool,
        "warnings": List,
        "errors": List,
        "diagnostics": Dict,
        "provider": Dict,
        "report": Text,
    },
    description=(
        "Resolve an installed hardware provider and discover raw servo telemetry "
        "using read-only operations only."
    ),
)
def robot_raw_monitor(ctx: dict) -> dict:
    hardware = ctx.get("hardware") if isinstance(ctx.get("hardware"), Mapping) else {}
    try:
        provider = _select_raw_monitor_provider(hardware)
        provider_ctx = dict(ctx)
        provider_ctx["hardware"] = dict(hardware)
        configured = (
            ctx.get("provider_config")
            if isinstance(ctx.get("provider_config"), Mapping)
            else {}
        )
        provider_ctx["provider_config"] = {
            **dict(provider.get("configuration") or {}),
            **dict(configured),
            "max_servo_id": max(
                1,
                min(
                    253,
                    int(
                        configured.get("max_servo_id")
                        or ctx.get("max_servo_id")
                        or 32
                    ),
                ),
            ),
        }
        sample = provider["sample"](provider_ctx)
        if not isinstance(sample, Mapping):
            raise TypeError("raw-monitor provider returned a non-object sample")
        joints = [
            dict(value)
            for value in (sample.get("joints") or [])
            if isinstance(value, Mapping)
        ]
        errors = [str(value) for value in (sample.get("errors") or []) if value]
        warnings = [
            str(value) for value in (sample.get("warnings") or []) if value
        ]
        descriptor = {
            "package": str(provider.get("package") or ""),
            "component": str(provider.get("component") or ""),
            "capability": str(
                provider.get("capability") or "raw_position_feedback"
            ),
        }
        return {
            "available": bool(joints),
            "joints": joints,
            "position_unit": str(sample.get("position_unit") or "ticks"),
            "velocity_unit": str(sample.get("velocity_unit") or "ticks/s"),
            "torque_enabled": sample.get("torque_enabled"),
            "warnings": warnings,
            "errors": errors,
            "diagnostics": dict(sample.get("diagnostics") or {}),
            "provider": descriptor,
            "report": str(
                sample.get("report")
                or (
                    f"Discovered {len(joints)} responding servo(s) with "
                    f"{descriptor['package']}/{descriptor['component']}."
                )
            ),
        }
    except Exception as exc:
        message = f"{type(exc).__name__}: {exc}"
        return {
            "available": False,
            "joints": [],
            "position_unit": "ticks",
            "velocity_unit": "ticks/s",
            "torque_enabled": None,
            "warnings": [],
            "errors": [message],
            "diagnostics": {},
            "provider": {},
            "report": message,
        }


def _mock_raw_monitor_score(hardware: Mapping[str, Any]) -> int:
    selected = hardware.get("raw_monitor_provider")
    return 100 if isinstance(selected, Mapping) else 0


def _mock_raw_monitor_sample(ctx: Mapping[str, Any]) -> dict[str, Any]:
    configured = (
        ctx.get("provider_config")
        if isinstance(ctx.get("provider_config"), Mapping)
        else {}
    )
    joints = (
        configured.get("joints")
        if isinstance(configured.get("joints"), list)
        else []
    )
    return {
        "joints": [dict(value) for value in joints if isinstance(value, Mapping)],
        "position_unit": "ticks",
        "velocity_unit": "ticks/s",
        "torque_enabled": None,
        "warnings": [],
        "errors": [],
        "diagnostics": {"mock": True},
        "report": "Raw monitor mock sample.",
    }


@node(
    name="RobotRawMonitorMockProvider",
    component="capabilities",
    category="Robot",
    hidden=True,
    inputs={"hardware": Dict, "provider_config": Dict},
    outputs={"available": Bool, "provider": Dict, "report": Text},
    description="Hardware-free provider for the generic raw-monitor contract.",
)
def robot_raw_monitor_mock_provider(ctx: dict) -> dict:
    provider = {
        "package": "blacknode-robot",
        "component": "capabilities",
        "capability": "raw_position_feedback",
    }
    return {
        "available": True,
        "provider": provider,
        "report": "Robot raw-monitor mock provider is available.",
    }


robot_raw_monitor_mock_provider._bn_robot_raw_monitor_provider = {
    "package": "blacknode-robot",
    "component": "capabilities",
    "capability": "raw_position_feedback",
    "match_hardware": _mock_raw_monitor_score,
    "sample": _mock_raw_monitor_sample,
}
