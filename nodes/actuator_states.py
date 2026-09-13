"""Hardware-bound actuator calibration points, named poses, and leased motion tests."""
from __future__ import annotations

import atexit
import copy
import hashlib
import importlib
import math
import threading
import time

from . import profiles

_lock = threading.RLock()
_drafts = {}
_tests = {}
_stopped = {}
_LEASE_SECONDS = 3.0
ACTIONS = {"state-status", "read-position", "capture-state", "save-states", "arm-test", "test-target", "test-status", "stop-test"}


def _motion():
    return importlib.import_module("blacknode.pkg.blacknode_motion.arm.servo_control")


def _identity(ctx):
    from . import actuator_setup as setup
    _, provider, config = setup._resolve(ctx)
    servo_id = ctx.get("servo_id")
    if isinstance(servo_id, bool) or not isinstance(servo_id, int) or not 0 <= servo_id <= 253:
        raise ValueError("Select a discovered servo card first")
    hardware = setup._NODE_REGISTRY["RobotUSBDiscovery"]({"port_filter": config["port"], "probe_open": False})
    hardware = hardware.get("hardware") or hardware
    if (hardware.get("recommended") or {}).get("path") != config["port"]:
        raise ValueError("Selected USB adapter is disconnected")
    hardware_id = profiles._hardware_id({"hardware": hardware})
    if not hardware_id:
        raise ValueError("Physical USB identity is required")
    config["hardware_id"] = hardware_id
    key = (hardware_id, provider["package"], provider["component"], servo_id)
    return key, provider, config


def _path(key):
    digest = hashlib.sha256("\0".join(map(str, key[:3])).encode()).hexdigest()
    return profiles._profile_root() / "actuator_setups" / digest / f"servo_{key[3]}.json"


def _state(key):
    if key not in _drafts:
        path = _path(key)
        draft = path.with_suffix(".draft.json")
        if draft.exists():
            path = draft
        data = profiles._read_json(path) if path.exists() else {}
        if data and data.get("identity") != list(key):
            raise ValueError("Saved setup belongs to different hardware")
        _drafts[key] = data or {"schema_version": 1, "units": "ticks", "identity": list(key), "servo_id": key[3],
                               "points": {}, "poses": {}, "safety_margin_ticks": 0}
        # Setup captures are the operator's command limits. Retire the old
        # automatic inset when loading existing saved states or drafts.
        _drafts[key]["safety_margin_ticks"] = 0
    return _drafts[key]


def _limits(state):
    points = state.get("points") or {}
    if not all(name in points for name in ("min", "max")):
        raise ValueError("Capture both Min and Max with torque released before testing")
    low, high = sorted(int(points[name]) for name in ("min", "max"))
    margin = int(state.get("safety_margin_ticks", 0))
    if margin < 0 or high - low <= 2 * margin + 1:
        raise ValueError("Capture distinct Min and Max positions with room for an interior test origin")
    # A test origin converts raw ticks to the motion contract's joint angle.
    # It is not a measured Home capture and does not rewrite captured points.
    home = int(points.get("home", (low + high) // 2))
    if not low + margin < home < high - margin:
        home = (low + high) // 2
    return low + margin, home, high - margin


def _snapshot(key, report=""):
    state = copy.deepcopy(_state(key))
    try:
        low, home, high = _limits(state)
        limits = {"min": low, "home": home, "max": high}
    except ValueError:
        limits = {}
    return {"ok": True, "states": state, "test_limits": limits,
            "saved": bool(state.get("saved_at")) and not state.get("dirty", False),
            "path": str(_path(key)), "report": report}


def ensure_idle(port):
    with _lock:
        if any(item["port"] == port for item in _tests.values()):
            raise ValueError("Stop the active servo test before scanning, changing IDs or capturing released calibration")


def retire_hardware(hardware_id):
    """Retire per-actuator setup when an ID write may change assembly identity."""
    root = profiles._profile_root() / "actuator_setups"
    retired = []
    with _lock:
        for path in root.glob("*/servo_*.json"):
            state = profiles._read_json(path)
            if (state.get("identity") or [None])[0] == hardware_id:
                target = path.parent / "retired" / f"{time.time_ns()}-{path.name}"
                target.parent.mkdir(parents=True, exist_ok=True)
                path.rename(target)
                retired.append(str(target))
        for key in list(_drafts):
            if key[0] == hardware_id:
                _drafts.pop(key)
    return retired


def _read_row(provider, config, servo_id, *, strict=False, live=False):
    reader = provider.get("read_position")
    rows = ([reader(config, servo_id)] if live and not strict and callable(reader)
            else provider["scan"](config).get("actuators") or [])
    if strict and any(row.get("discovery_status") == "unreadable" for row in rows):
        raise ValueError("Resolve unreadable addresses before motion testing")
    row = next((row for row in rows if row.get("servo_id") == servo_id), None)
    if not row or row.get("discovery_status") == "unreadable":
        raise ValueError("This servo is not readable; resolve its ID and scan again")
    if row.get("errors") or row.get("hardware_error_flags") or not row.get("assignment_supported"):
        raise ValueError("Complete, supported, warning-free servo feedback is required")
    if row.get("reported_id", servo_id) != servo_id:
        raise ValueError("Servo ID readback does not match this card")
    ticks = row.get("raw_position")
    if isinstance(ticks, bool) or not isinstance(ticks, int):
        raise ValueError("Current position is unavailable")
    return row


def _stop(key):
    item = _tests.pop(key, None)
    if item is None:
        return {"ok": True, "armed": False, "report": "Test stopped"}
    try:
        result = _motion().disarm_servo_motion(item["run_id"])
    except Exception as exc:
        result = {"ok": False, "armed": False, "report": f"Torque release needs attention: {exc}"}
    _stopped[key] = result
    item["stop"].set()
    return result


def _sample(item, *, cached=False):
    motion = _motion()
    status = motion.servo_motion_status(item["run_id"])
    if not status.get("armed"):
        raise ValueError("Test is disarmed")
    if cached and time.monotonic() - item.get("sample_at", 0) <= 0.25:
        return dict(item["sample"])
    sample = motion.sample_servo_motion_for_robot(item["run_id"])
    if not sample or not sample.get("command_ok") or sample.get("torque_enabled") is not True:
        raise ValueError((sample or {}).get("report") or "Fresh test feedback is unavailable")
    row = (sample.get("servos") or {}).get("actuator") or {}
    ticks = row.get("ticks")
    if ticks is None and isinstance((sample.get("pose") or {}).get("actuator"), (int, float)):
        ticks = round(item["limits"][1] + sample["pose"]["actuator"] / item["context"]["degrees_per_tick"])
    if isinstance(ticks, bool) or not isinstance(ticks, int):
        raise ValueError("Fresh test position is unavailable")
    result = {"armed": True, "raw_position": ticks, "torque_enabled": True,
              "report": "Test armed. Slider moves this servo within saved limits."}
    item["sample"], item["sample_at"] = result, time.monotonic()
    return dict(result)


def _watch(key, item):
    while not item["stop"].is_set():
        item["wake"].wait(0.02)
        item["wake"].clear()
        with _lock:
            if _tests.get(key) is not item:
                return
            try:
                if time.monotonic() - item["heartbeat"] > _LEASE_SECONDS:
                    raise TimeoutError("Test controls disconnected")
                sample = _sample(item)
                desired = item.get("desired")
                if desired is not None:
                    direct = item.get("position_target_mode") is True
                    if direct and desired == item.get("last_sent_target"):
                        continue
                    scale = item["context"]["degrees_per_tick"]
                    current = sample["raw_position"]
                    # Small, bounded increments prevent a delayed UI request
                    # from becoming one large position jump after idle time.
                    elapsed = time.monotonic() - item["last_step_at"]
                    step = int(item["speed"] * min(0.05, elapsed) / scale)
                    if not direct and step < 1:
                        continue
                    target = desired if direct else max(current - step, min(current + step, desired))
                    low, _, high = item["limits"]
                    if not low <= target <= high:
                        # Hold is allowed at a captured endpoint. Enter the
                        # inset command range only after enough time for this
                        # first small step; do not let driver clamping create
                        # a faster-than-configured jump across the margin.
                        target = max(low, min(high, target))
                        elapsed = time.monotonic() - item["last_step_at"]
                        if abs(target - current) * scale > item["speed"] * elapsed:
                            continue
                    if target != current:
                        result = _motion().command_servo_motion(item["run_id"], {
                            "kind": "blacknode.joint-command-request", "schema_version": 1,
                            "joint_name": "actuator", "servo_id": key[3],
                            "position_rad": math.radians((target - item["limits"][1]) * scale),
                            "issued_at": time.time(), "requires_motion_authorization": True,
                        })
                        if not result.get("ok"):
                            raise ValueError(result.get("report") or "Test command failed")
                        item["last_step_at"] = time.monotonic()
                        if direct:
                            item["last_sent_target"] = target
            except Exception as exc:
                stopped = _stop(key)
                report = str(exc)
                if not stopped.get("ok"):
                    report += "; " + stopped["report"]
                _stopped[key] = {"ok": False, "armed": False, "report": report}
                return


def control(ctx, action, payload):
    key = None
    try:
        # Stop must also work after the adapter has been disconnected.
        if action == "stop-test":
            with _lock:
                matches = [key for key, item in _tests.items()
                           if item["port"] == ctx.get("serial_port") and key[3] == ctx.get("servo_id")]
                results = [_stop(key) for key in matches]
                return next((result for result in results if not result.get("ok")),
                            {"ok": True, "armed": False, "report": "Test stopped; torque released" if matches
                             else "No active test session"})
        key, provider, config = _identity(ctx)
        with _lock:
            state = _state(key)
            if action == "state-status":
                return _snapshot(key)
            if action == "read-position":
                if payload.get("confirm_read_only") is not True:
                    raise ValueError("Confirm read-only position feedback")
                ensure_idle(config["port"])
                reader = provider.get("read_position")
                if not callable(reader):
                    raise ValueError("This provider does not support live position feedback")
                row = reader(config, key[3])
                ticks = row.get("raw_position")
                sampled = row.get("sampled_at", 0)
                if (row.get("reported_id") != key[3] or not row.get("assignment_supported")
                        or row.get("discovery_status") == "unreadable" or row.get("errors")
                        or isinstance(ticks, bool) or not isinstance(ticks, int)
                        or not 0 <= time.time() - sampled <= 1.0):
                    raise ValueError("Fresh position from the selected actuator is unavailable")
                return {"ok": True, "raw_position": ticks, "sampled_at": sampled,
                        "position_range": row.get("position_range"),
                        "torque_enabled": row.get("torque_enabled"),
                        "hardware_error_flags": row.get("hardware_error_flags", 0),
                        "report": "; ".join(row.get("hardware_errors") or [])}
            if action == "capture-state":
                kind = str(payload.get("point") or "pose")
                if kind not in {"min", "home", "max", "pose"}:
                    raise ValueError("Choose Min, Home, Max or a named pose")
                if kind == "pose" and key in _tests:
                    ticks = _sample(_tests[key])["raw_position"]
                else:
                    ensure_idle(config["port"])
                    row = _read_row(provider, config, key[3], live=True)
                    if row.get("torque_enabled") is not False:
                        raise ValueError("Support this servo and release torque before capturing calibration")
                    if state.get("model_number") not in (None, row["model_number"]):
                        raise ValueError("Actuator model changed; discard the previous setup before calibrating")
                    state["model_number"] = row["model_number"]
                    ticks = row["raw_position"]
                if isinstance(ticks, bool) or not isinstance(ticks, int):
                    raise ValueError("Fresh position is unavailable")
                if kind == "pose":
                    name = str(payload.get("name") or "").strip()
                    if not name or len(name) > 64:
                        raise ValueError("Give this pose a name of 1–64 characters")
                    state["poses"][name] = ticks
                else:
                    state["points"][kind] = ticks
                state["dirty"] = True
                profiles._write_json(_path(key).with_suffix(".draft.json"), state)
                return {**_snapshot(key, f"Captured {kind} at {ticks} ticks. Press Save states to keep it."), "raw_position": ticks}
            if action == "save-states":
                ensure_idle(config["port"])
                # Named poses can be saved before calibration is complete.
                if all(name in state["points"] for name in ("min", "max")):
                    _limits(state)
                if not state["points"] and not state["poses"]:
                    raise ValueError("Capture a calibration point or named pose first")
                saved = {**state, "saved_at": time.time(), "dirty": False}
                profiles._write_json(_path(key), saved)
                _path(key).with_suffix(".draft.json").unlink(missing_ok=True)
                state.update(saved)
                return _snapshot(key, "Servo states saved for this USB hardware and servo ID")
            if action == "arm-test":
                if payload.get("confirm_test") is not True and payload.get("operator_action") != "arm-test":
                    raise ValueError("Confirm the calibrated actuator has a unique ID and the robot is supported")
                ensure_idle(config["port"])
                low, home, high = _limits(state)
                margin = int(state.get("safety_margin_ticks", 0))
                accept_draft = payload.get("operator_action") == "arm-test" and payload.get("save_calibration") is True
                if (state.get("dirty") or not state.get("saved_at")) and not accept_draft:
                    raise ValueError("Save calibration before arming the test")
                row = _read_row(provider, config, key[3])
                if row.get("model_number") != state.get("model_number"):
                    raise ValueError("Calibration model does not match this actuator")
                if row.get("torque_enabled") is not False:
                    raise ValueError("Release torque before arming this test")
                if not low - margin <= row["raw_position"] <= high + margin:
                    raise ValueError("Current position is outside the captured endpoints; update the captured range before testing")
                builder = provider.get("build_test_context")
                if not callable(builder):
                    raise ValueError("This provider does not support calibrated servo testing")
                test_state = {**state, "points": {"min": low - margin, "home": home, "max": high + margin}}
                motion_ctx = builder(config, test_state, row)
                speeds = [float(joint.get("velocity_limit") or 0) for joint in motion_ctx["profile"]["joints"]]
                speed = min([180.0 if motion_ctx.get("position_target_mode") else 60.0, *speeds])
                if not math.isfinite(speed) or speed <= 0:
                    raise ValueError("A bounded test speed is required")
                if accept_draft and (state.get("dirty") or not state.get("saved_at")):
                    saved = {**state, "saved_at": time.time(), "dirty": False}
                    profiles._write_json(_path(key), saved)
                    _path(key).with_suffix(".draft.json").unlink(missing_ok=True)
                    state.update(saved)
                run_id = "actuator-test:" + hashlib.sha256(str(key).encode()).hexdigest()
                motion_ctx["robot_id"] = run_id
                result = _motion().arm_servo_motion(run_id, motion_ctx)
                if not result.get("ok") or not result.get("armed"):
                    _motion().disarm_servo_motion(run_id)
                    raise ValueError(result.get("report") or "Test could not arm")
                item = {"run_id": run_id, "port": config["port"], "heartbeat": time.monotonic(),
                        "last_step_at": time.monotonic(),
                        "wake": threading.Event(), "speed": speed,
                        "position_target_mode": motion_ctx.get("position_target_mode") is True,
                        "stop": threading.Event(), "context": motion_ctx, "limits": (low, home, high)}
                _tests[key] = item
                _stopped.pop(key, None)
                threading.Thread(target=_watch, args=(key, item), daemon=True, name="actuator-test").start()
                return {**_snapshot(key), **_sample(item), "target_ticks": row["raw_position"]}
            if action in {"test-status", "test-target"}:
                item = _tests.get(key)
                if item is None:
                    return _stopped.get(key, {"ok": True, "armed": False, "report": "Test is stopped"})
                if time.monotonic() - item["heartbeat"] > _LEASE_SECONDS:
                    _stop(key)
                    raise ValueError("Test connection expired; arm again")
                item["heartbeat"] = time.monotonic()
                if action == "test-target":
                    target = payload.get("ticks")
                    issued = float(payload.get("issued_at") or 0)
                    if isinstance(target, bool) or not isinstance(target, int) or not 0 <= time.time() - issued <= 0.75:
                        raise ValueError("Slider command is invalid or stale")
                    low, home, high = item["limits"]
                    if not low <= target <= high:
                        raise ValueError("Slider target is outside the saved safe range")
                    item["desired"] = target
                    item["wake"].set()
                    return {"ok": True, **_sample(item, cached=True), "target_ticks": target}
                return {"ok": True, **_sample(item, cached=True)}
            raise ValueError("Unsupported actuator state action")
    except Exception as exc:
        report = str(exc)
        if key is not None and action in {"arm-test", "test-target", "test-status"}:
            with _lock:
                stopped = _stop(key)
                if not stopped.get("ok"):
                    report += "; " + stopped["report"]
        return {"ok": False, "armed": False, "report": report}


@atexit.register
def stop_runtime_services():
    with _lock:
        results = [_stop(key) for key in list(_tests)]
    return {"ok": all(result.get("ok") for result in results), "stopped": {"managed_runs": len(results)}}
