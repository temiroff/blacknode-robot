"""Generic, profile-bound control for hand-guided robot calibration."""
from __future__ import annotations

import threading
import time
from typing import Any, Mapping

from blacknode.node import Any as AnyPort
from blacknode.node import Bool, Dict, Enum, Float, List, Text, node
from blacknode.node import _NODE_REGISTRY


_CATEGORY = "Robot"
_PROVIDER_ATTRIBUTE = "_bn_robot_calibration_provider"
_session_lock = threading.Lock()
_sessions: dict[str, dict[str, Any]] = {}


def _profile_from_ctx(ctx: Mapping[str, Any]) -> dict[str, Any]:
    profile = ctx.get("profile") if isinstance(ctx.get("profile"), Mapping) else {}
    robot = ctx.get("robot") if isinstance(ctx.get("robot"), Mapping) else {}
    driver = robot.get("driver") if isinstance(robot.get("driver"), Mapping) else {}
    if not profile and isinstance(driver.get("profile"), Mapping):
        profile = driver["profile"]
    return dict(profile)


def _provider_binding(profile: Mapping[str, Any]) -> dict[str, Any]:
    bindings = (
        profile.get("capability_bindings")
        if isinstance(profile.get("capability_bindings"), Mapping)
        else {}
    )
    for capability in ("calibration_control", "position_feedback", "joint_group"):
        binding = bindings.get(capability)
        if not isinstance(binding, Mapping):
            continue
        provider = (
            binding.get("provider")
            if isinstance(binding.get("provider"), Mapping)
            else {}
        )
        if provider.get("package") and provider.get("component"):
            return {
                "capability": capability,
                "package": str(provider["package"]),
                "component": str(provider["component"]),
                "configuration": dict(binding.get("configuration") or {}),
            }
    raise ValueError(
        "robot profile does not bind calibration_control, position_feedback, "
        "or joint_group to a provider"
    )


def _provider_specs() -> list[dict[str, Any]]:
    providers: list[dict[str, Any]] = []
    for fn in _NODE_REGISTRY.values():
        value = getattr(fn, _PROVIDER_ATTRIBUTE, None)
        if not isinstance(value, Mapping):
            continue
        open_session = value.get("open_session")
        if not callable(open_session):
            continue
        providers.append(dict(value))
    return providers


def _open_provider_session(
    ctx: Mapping[str, Any],
) -> tuple[Any, dict[str, Any]]:
    binding = _provider_binding(_profile_from_ctx(ctx))
    for provider in _provider_specs():
        if (
            str(provider.get("package") or "") == binding["package"]
            and str(provider.get("component") or "") == binding["component"]
        ):
            provider_ctx = dict(ctx)
            configured = (
                ctx.get("provider_config")
                if isinstance(ctx.get("provider_config"), Mapping)
                else {}
            )
            provider_ctx["provider_config"] = {
                **binding["configuration"],
                **configured,
            }
            session = provider["open_session"](provider_ctx)
            descriptor = {
                "package": binding["package"],
                "component": binding["component"],
                "capability": str(
                    provider.get("capability") or "calibration_control"
                ),
                "bound_via": binding["capability"],
            }
            return session, descriptor
    available = ", ".join(
        sorted(
            f"{value.get('package')}/{value.get('component')}"
            for value in _provider_specs()
        )
    )
    suffix = f"; available providers: {available}" if available else ""
    raise RuntimeError(
        "calibration-control provider is unavailable for "
        f"{binding['package']}/{binding['component']}{suffix}"
    )


def _outputs(
    action: str,
    sample: Mapping[str, Any],
    *,
    live: bool,
    provider: Mapping[str, Any] | None = None,
    error: str = "",
) -> dict[str, Any]:
    pose = dict(sample.get("pose") or {})
    torque = sample.get("torque_enabled")
    errors = [str(value) for value in (sample.get("errors") or []) if value]
    warnings = [str(value) for value in (sample.get("warnings") or []) if value]
    if error:
        errors.insert(0, error)
    mode = (
        "released"
        if torque is False
        else "hold"
        if torque is True
        else "unknown"
    )
    report = (
        "Move the supported robot by hand; live joint positions update here."
        if torque is False and not errors
        else "Holding the current pose."
        if torque is True and not errors
        else "Robot calibration control needs attention."
    )
    if errors:
        report += "\n" + "\n".join(f"- {value}" for value in errors)
    if warnings:
        report += "\nWarnings:\n" + "\n".join(
            f"- {value}" for value in warnings
        )
    return {
        "action": action,
        "available": not errors,
        "live": live,
        "data_ready": bool(pose),
        "mode": mode,
        "torque_enabled": torque,
        "command_ok": not errors,
        "pose": pose,
        "joints": list(pose),
        "warnings": warnings,
        "servos": dict(sample.get("servos") or {}),
        "diagnostics": dict(sample.get("diagnostics") or {}),
        "provider": dict(provider or {}),
        "updated_at": time.strftime("updated %H:%M:%S"),
        "report": report,
    }


def _downstream_outputs(
    ctx: Mapping[str, Any],
    pose: Mapping[str, float],
    torque_enabled: bool | None,
) -> tuple[dict[str, dict[str, Any]], dict[str, str]]:
    graph = ctx.get("__graph__")
    node_id = str(ctx.get("__node_id__") or "")
    graph_nodes = getattr(graph, "_nodes", {}) if graph is not None else {}
    graph_edges = list(getattr(graph, "_edges", []) or []) if graph is not None else []
    cache = (getattr(graph, "_cache", {}) or {}) if graph is not None else {}
    targets = {
        str(edge.get("to") or "")
        for edge in graph_edges
        if edge.get("from") == node_id
        and edge.get("from_port") in {"pose", "torque_enabled"}
    }
    outputs: dict[str, dict[str, Any]] = {}
    node_types: dict[str, str] = {}
    for target_id in targets:
        target = graph_nodes.get(target_id) or {}
        target_type = str(target.get("type") or "")
        if target_type != "RobotCalibrationRecorder":
            continue
        target_fn = _NODE_REGISTRY.get(target_type)
        if target_fn is None:
            continue
        target_ctx = dict(target.get("params") or {})
        for edge in graph_edges:
            if edge.get("to") != target_id:
                continue
            port = str(edge.get("to_port") or "")
            if edge.get("from") == node_id and edge.get("from_port") == "pose":
                target_ctx[port] = dict(pose)
            elif (
                edge.get("from") == node_id
                and edge.get("from_port") == "torque_enabled"
            ):
                target_ctx[port] = torque_enabled
            else:
                cache_key = (edge.get("from"), edge.get("from_port"))
                if cache_key in cache:
                    target_ctx[port] = cache[cache_key]
        target_ctx.update({"action": "_sample", "__live_pose__": True})
        try:
            outputs[target_id] = dict(target_fn(target_ctx))
            node_types[target_id] = target_type
        except Exception:
            continue
    return outputs, node_types


def _close_session(item: Mapping[str, Any]) -> None:
    item["stop"].set()
    with item["provider_lock"]:
        item["provider_session"].close()


def _stop_session(run_id: str) -> bool:
    with _session_lock:
        item = _sessions.pop(run_id, None)
    if item is None:
        return False
    _close_session(item)
    return True


def _worker(run_id: str, item: dict[str, Any]) -> None:
    interval = 1.0 / max(1.0, min(float(item["sample_hz"]), 60.0))
    while not item["stop"].wait(interval):
        try:
            with item["provider_lock"]:
                sample = item["provider_session"].sample()
            outputs = _outputs(
                str(item["action"]),
                sample,
                live=True,
                provider=item["provider"],
            )
            if outputs["data_ready"]:
                downstream, downstream_types = _downstream_outputs(
                    item["ctx"],
                    outputs["pose"],
                    outputs["torque_enabled"],
                )
            else:
                downstream, downstream_types = {}, {}
            with _session_lock:
                if _sessions.get(run_id) is not item:
                    return
                item["outputs"] = outputs
                item["downstream_outputs"] = downstream
                item["downstream_types"] = downstream_types
                item["updated_at"] = time.time()
                item["error"] = ""
        except Exception as exc:
            with _session_lock:
                if _sessions.get(run_id) is not item:
                    return
                item["error"] = f"{type(exc).__name__}: {exc}"
                item["outputs"] = _outputs(
                    str(item["action"]),
                    {},
                    live=True,
                    provider=item["provider"],
                    error=item["error"],
                )


def _start_session(
    run_id: str,
    ctx: Mapping[str, Any],
    action: str,
) -> dict[str, Any]:
    _stop_session(run_id)
    provider_session, provider = _open_provider_session(ctx)
    try:
        sample = (
            provider_session.release()
            if action == "release"
            else provider_session.hold()
            if action == "hold"
            else provider_session.sample()
        )
    except Exception:
        provider_session.close()
        raise
    outputs = _outputs(
        action,
        sample,
        live=True,
        provider=provider,
    )
    item: dict[str, Any] = {
        "ctx": dict(ctx),
        "action": action,
        "sample_hz": float(ctx.get("sample_hz") or 15.0),
        "provider": provider,
        "provider_session": provider_session,
        "provider_lock": threading.Lock(),
        "outputs": outputs,
        "downstream_outputs": {},
        "downstream_types": {},
        "updated_at": time.time(),
        "error": "",
        "stop": threading.Event(),
    }
    with _session_lock:
        _sessions[run_id] = item
    thread = threading.Thread(
        target=_worker,
        args=(run_id, item),
        name=f"blacknode-robot-calibration-{run_id}",
        daemon=True,
    )
    item["thread"] = thread
    thread.start()
    return outputs


class _MockCalibrationSession:
    def __init__(self, ctx: Mapping[str, Any]) -> None:
        profile = _profile_from_ctx(ctx)
        config = (
            ctx.get("provider_config")
            if isinstance(ctx.get("provider_config"), Mapping)
            else {}
        )
        configured_pose = (
            config.get("pose")
            if isinstance(config.get("pose"), Mapping)
            else {}
        )
        joints = profile.get("joints") if isinstance(profile.get("joints"), list) else []
        self.pose = {
            str(joint.get("id")): float(
                configured_pose.get(str(joint.get("id")), 0.0)
            )
            for joint in joints
            if isinstance(joint, Mapping) and joint.get("id")
        }
        self.torque_enabled = bool(config.get("torque_enabled", False))

    def sample(self) -> dict[str, Any]:
        return {
            "pose": dict(self.pose),
            "torque_enabled": self.torque_enabled,
            "errors": [],
        }

    def release(self) -> dict[str, Any]:
        self.torque_enabled = False
        return self.sample()

    def hold(self) -> dict[str, Any]:
        self.torque_enabled = True
        return self.sample()

    def command(
        self,
        positions_deg: Mapping[str, float],
        *,
        deadline: float,
    ) -> dict[str, Any]:
        if not self.torque_enabled:
            raise PermissionError("joint motion is disarmed")
        if time.monotonic() > float(deadline):
            raise TimeoutError("joint command is stale")
        for name, value in positions_deg.items():
            if name not in self.pose:
                raise ValueError(f"unknown joint: {name}")
            self.pose[name] = float(value)
        return self.sample()

    def close(self) -> None:
        self.torque_enabled = False


@node(
    name="RobotCalibrationMockProvider",
    component="calibration",
    category=_CATEGORY,
    hidden=True,
    description=(
        "Hardware-free provider for testing the generic robot calibration "
        "control contract."
    ),
    inputs={"profile": Dict, "provider_config": Dict},
    outputs={"available": Bool, "provider": Dict, "report": Text},
)
def robot_calibration_mock_provider(ctx: dict) -> dict:
    provider = {
        "package": "blacknode-robot",
        "component": "calibration",
        "capability": "calibration_control",
    }
    return {
        "available": True,
        "provider": provider,
        "report": "Robot calibration mock provider is available.",
    }


robot_calibration_mock_provider._bn_robot_calibration_provider = {
    "package": "blacknode-robot",
    "component": "calibration",
    "capability": "calibration_control",
    "open_session": _MockCalibrationSession,
}

robot_calibration_mock_provider._bn_robot_joint_motion_provider = {
    "package": "blacknode-robot",
    "component": "calibration",
    "capability": "joint_group",
    "open_session": _MockCalibrationSession,
}


@node(
    name="RobotCalibrationControl",
    component="calibration",
    category=_CATEGORY,
    live=True,
    description=(
        "Release or hold a profile-bound robot safely and stream normalized "
        "joint feedback through its selected calibration provider."
    ),
    primary_inputs=["trigger", "profile", "hardware"],
    primary_outputs=["pose", "torque_enabled", "report"],
    inputs={
        "trigger": AnyPort,
        "run_id": Text(default="robot_calibration_control"),
        "action": Enum(
            ["check", "release", "hold", "stop"],
            default="check",
        ),
        "profile": Dict,
        "hardware": Dict,
        "robot": Dict,
        "provider_config": Dict,
        "sample_hz": Float(default=15.0),
    },
    outputs={
        "action": Text,
        "available": Bool,
        "live": Bool,
        "data_ready": Bool,
        "mode": Text,
        "torque_enabled": Bool,
        "command_ok": Bool,
        "pose": Dict,
        "joints": List,
        "warnings": List,
        "servos": Dict,
        "diagnostics": Dict,
        "provider": Dict,
        "updated_at": Text,
        "report": Text,
    },
)
def robot_calibration_control(ctx: dict) -> dict:
    run_id = str(ctx.get("run_id") or "robot_calibration_control").strip()
    action = str(ctx.get("action") or "check").strip().lower()
    action = {"monitor": "check", "enter": "release", "exit": "hold"}.get(
        action,
        action,
    )
    if action == "stop":
        stopped = _stop_session(run_id)
        return _outputs(
            action,
            {},
            live=False,
            error="" if stopped else "calibration session was not running",
        )
    try:
        run_mode = "once" if ctx.get("__run_mode__") == "once" else "live"
        if run_mode == "once" and action == "hold":
            return _outputs(
                action,
                {},
                live=False,
                error=(
                    "Hold requires a live calibration session so Blacknode can "
                    "retain provider ownership and supervise shutdown"
                ),
            )
        if run_mode == "live":
            return _start_session(run_id, ctx, action)
        provider_session, provider = _open_provider_session(ctx)
        try:
            sample = (
                provider_session.release()
                if action == "release"
                else provider_session.sample()
            )
            return _outputs(
                action,
                sample,
                live=False,
                provider=provider,
            )
        finally:
            provider_session.close()
    except Exception as exc:
        return _outputs(
            action,
            {},
            live=False,
            error=f"{type(exc).__name__}: {exc}",
        )


def runtime_status() -> dict[str, Any]:
    with _session_lock:
        node_outputs: list[dict[str, Any]] = []
        managed_runs: list[dict[str, Any]] = []
        for run_id, item in _sessions.items():
            node_id = str(item.get("ctx", {}).get("__node_id__") or "")
            node_outputs.append({
                "run_id": run_id,
                "node_id": node_id,
                "node_type": "RobotCalibrationControl",
                "outputs": dict(item.get("outputs") or {}),
                "updated_at": item.get("updated_at"),
                "error": str(item.get("error") or ""),
            })
            for target_id, outputs in dict(
                item.get("downstream_outputs") or {}
            ).items():
                node_outputs.append({
                    "run_id": run_id,
                    "node_id": target_id,
                    "node_type": str(
                        item.get("downstream_types", {}).get(target_id) or ""
                    ),
                    "outputs": dict(outputs),
                    "updated_at": item.get("updated_at"),
                    "error": "",
                })
            managed_runs.append({
                "run_id": run_id,
                "node_id": node_id,
                "kind": "robot_calibration_control",
                "provider": dict(item.get("provider") or {}),
                "active": not item["stop"].is_set(),
                "updated_at": item.get("updated_at"),
                "error": str(item.get("error") or ""),
            })
    return {
        "ok": True,
        "active": bool(managed_runs),
        "managed_runs": managed_runs,
        "node_outputs": node_outputs,
    }


def stop_runtime_services() -> dict[str, Any]:
    with _session_lock:
        items = list(_sessions.values())
        _sessions.clear()
    for item in items:
        _close_session(item)
    return {
        "ok": True,
        "stopped": {"managed_runs": len(items)},
        "report": f"stopped {len(items)} robot calibration session(s)",
    }
