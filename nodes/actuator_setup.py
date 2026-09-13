"""Operator-controlled USB actuator setup over replaceable provider contracts."""
from __future__ import annotations

import secrets
import copy
import threading
import time
from collections.abc import Mapping

from blacknode.node import Bool, Dict, Int, List, Text, node, _NODE_REGISTRY
from .calibration_control import _provider_binding
from . import profiles

_tickets = {}
_lock = threading.Lock()
_TTL = 120.0


def _resolve(ctx):
    profile_id = str(ctx.get("profile_id") or "").strip()
    port = str(ctx.get("serial_port") or "").strip()
    if not port:
        raise ValueError("Pick a USB port, then press Scan")
    providers = [value for fn in _NODE_REGISTRY.values()
                 if isinstance(value := getattr(fn, "_bn_robot_actuator_setup_provider", None), Mapping)]
    profile = {}
    if profile_id and profile_id not in {"auto", "none"}:
        profile, _ = profiles.load_profile(profile_id)
        if not profile:
            raise ValueError("Selected robot profile is unavailable")
        errors = profiles._validate_profile(profile)
        if errors:
            raise ValueError("Invalid robot profile: " + "; ".join(errors))
        profile = profiles._profile_with_default_capabilities(profile, profile.get("driver") or {})
        binding = _provider_binding(profile)
        candidates = [p for p in providers if p.get("package") == binding["package"]
                      and p.get("component") == binding["component"]]
    else:
        ranked = [(int(p["match_hardware"]({"port": port})), p) for p in providers
                  if callable(p.get("match_hardware"))]
        best = max((score for score, _ in ranked), default=0)
        candidates = [p for score, p in ranked if score == best and score > 0]
    if len(candidates) != 1:
        raise ValueError("Actuator setup provider unavailable or ambiguous for this USB adapter")
    provider = candidates[0]
    if not provider or not callable(provider.get("scan")) or not callable(provider.get("assign")):
        raise ValueError("Actuator setup provider unavailable")
    return profile, provider, {"port": port, "baudrate": int(ctx.get("baudrate") or 1000000)}


def _retire_calibrations(hardware_id):
    """Preserve old files for review while preventing automatic reuse on this assembly."""
    retired = []
    for item in profiles.list_profiles():
        while (path := profiles._find_calibration_path(item["id"], hardware_id)) is not None:
            destination = path.parent / "retired" / f"{time.time_ns()}-{path.name}"
            destination.parent.mkdir(parents=True, exist_ok=True)
            path.rename(destination)
            retired.append(str(destination))
    from . import actuator_states
    return retired + actuator_states.retire_hardware(hardware_id)


def control_actuator_setup(ctx, action, payload=None):
    payload = payload or {}
    from . import actuator_states
    if action in actuator_states.ACTIONS:
        return actuator_states.control(ctx, action, payload)
    base = {"ok": False, "assigned": False, "actuators": [], "scan_token": "",
            "profile": {}, "bus": {}, "report": "", "retired_calibrations": []}
    try:
        profile, provider, config = _resolve(ctx)
        base["profile"] = profile
        base["bus"] = dict(config)
        if action == "inspect":
            return {**base, "ok": True, "bus": dict(config), "report": "Choose Scan bus to discover IDs 0–253. "
                    "Stop monitoring, calibration and motion sessions that own this port first."}
        if action not in {"scan", "assign", "release"}:
            raise ValueError("Actuator Setup supports inspect, scan, release, or assign")
        actuator_states.ensure_idle(config["port"])
        if payload.get("confirm_read_only") is not True:
            raise ValueError("Confirm the selected port, power and wiring before scanning")
        # USB discovery is transport-neutral and does not open the servo bus.
        discovery = _NODE_REGISTRY["RobotUSBDiscovery"]({"port_filter": config["port"], "probe_open": False})
        hardware = discovery.get("hardware") or discovery
        recommended = hardware.get("recommended") or {}
        if str(recommended.get("path") or "") != config["port"]:
            raise ValueError("Selected USB port is no longer connected; refresh the port selection")
        hardware_id = profiles._hardware_id({"hardware": hardware})
        if not hardware_id:
            raise ValueError("A physical USB hardware identity is required")
        base["bus"]["hardware_id"] = hardware_id
        identity = (profile.get("id", ""), config["port"], config["baudrate"], hardware_id)
        if action == "scan":
            result = dict(provider["scan"](config))
            rows = result.get("actuators") or []
            token = secrets.token_urlsafe(24)
            with _lock:
                now = time.monotonic()
                for key, ticket in list(_tickets.items()):
                    if now - ticket["at"] > _TTL or ticket["identity"] == identity:
                        _tickets.pop(key, None)
                _tickets[token] = {"at": now, "identity": identity, "rows": rows}
            unreadable = [row["servo_id"] for row in rows if row.get("discovery_status") == "unreadable"]
            reported = {row["servo_id"] for row in rows if row.get("discovery_status") != "unreadable"}
            missing = [j["servo_id"] for j in profile.get("joints", []) if j["servo_id"] not in reported]
            report = (f"Scan complete on {config['port']}: {len(reported)} responding IDs. "
                      "Open each card to inspect and set up its servo.")
            if unreadable:
                report += (f" Unreadable replies at IDs {', '.join(map(str, unreadable))}; "
                           "these addresses have separate unresolved cards. "
                           "Servos sharing an ID must be connected one at a time to identify them separately.")
            if not rows:
                report = "No servos responded. Check power and wiring; try another baud rate under Advanced."
            return {**base, "ok": bool(rows), "actuators": rows, "scan_token": token,
                    "scanned_at": time.time(), "bus": {**config, "hardware_id": hardware_id},
                    "missing_ids": missing, "report": report}
        # The dedicated button is the operator action. Its adjacent instructions
        # require an isolated, supported actuator and explain calibration reset.
        # Older clients retain their explicit confirmation/token contract.
        button_action = payload.get("operator_action") == action and action in {"assign", "release"}
        if button_action and ctx.get("servo_id") is None:
            raise ValueError("Use Set ID on a discovered servo card")
        if not button_action and payload.get("confirm_isolated") is not True:
            raise ValueError("Confirm one isolated actuator supported against gravity")
        if action == "assign" and not button_action and payload.get("confirm_recalibrate") is not True:
            raise ValueError("Confirm recalibration of the changed assembly")
        if button_action:
            rows = dict(provider["scan"](config)).get("actuators") or []
            ticket = {"at": time.monotonic(), "identity": identity, "rows": rows}
        else:
            with _lock:
                ticket = _tickets.pop(str(payload.get("scan_token") or ""), None)
        if not ticket or ticket["identity"] != identity or time.monotonic() - ticket["at"] > _TTL:
            raise ValueError("Discovery expired or selection changed; scan the isolated actuator again")
        if len(ticket["rows"]) != 1:
            raise ValueError("Connect exactly one actuator and scan again before assigning an ID")
        if ctx.get("servo_id") is not None and ctx["servo_id"] != ticket["rows"][0]["servo_id"]:
            raise ValueError("This servo card does not match the isolated actuator; scan again")
        if ticket["rows"][0].get("discovery_status") == "unreadable":
            raise ValueError("This address is unresolved. Connect one actuator and scan until its ID and model can be read")
        if action == "release":
            release = provider.get("release")
            if not callable(release):
                raise ValueError("This provider does not support isolated torque release")
            result = dict(release(config, ticket["rows"][0]))
            return {**base, **result, "ok": result.get("released") is True}
        joint_id = str(payload.get("joint_id") or "")
        joint = next((j for j in profile.get("joints", []) if j.get("id") == joint_id), None)
        new_id = payload.get("new_id")
        if new_id is None and joint:
            new_id = joint["servo_id"]
        if isinstance(new_id, bool) or not isinstance(new_id, int) or not 1 <= new_id <= 253:
            raise ValueError("Enter a new servo ID from 1 to 253")
        row = ticket["rows"][0]
        if not row.get("assignment_supported") or row.get("errors") or row.get("hardware_error_flags"):
            raise ValueError("ID programming requires a supported actuator with complete, warning-free feedback")
        if row.get("torque_enabled") is not False:
            raise ValueError("Support this actuator and release torque before setting its ID")
        # Retire calibration before a possibly successful physical write. A failed
        # acknowledgement must not leave a changed assembly using old limits.
        if new_id != ticket["rows"][0]["servo_id"]:
            base["retired_calibrations"] = _retire_calibrations(hardware_id)
        result = dict(provider["assign"](config, ticket["rows"][0], new_id))
        return {**base, **result, "ok": bool(result.get("assigned")), "profile": profile,
                "joint_id": joint_id, "hardware_id": hardware_id}
    except Exception as exc:
        return {**base, "report": str(exc)}


@node(name="ActuatorSetup", component="capabilities", category="Robot",
      description="Pick a USB port and scan to create a separate setup card for every responding servo.",
      inputs={"profile_id": Text(default=""), "serial_port": Text(default=""), "baudrate": Int(default=1000000)},
      outputs={"ok": Bool, "assigned": Bool, "actuators": List, "profile": Dict, "bus": Dict, "report": Text},
      primary_inputs=[], primary_outputs=["bus", "report"])
def actuator_setup(ctx):
    # Cooking or replaying a saved workflow never scans, writes EEPROM or moves.
    return control_actuator_setup(ctx, "inspect")


actuator_setup._bn_actuator_setup_control = control_actuator_setup


@node(name="ActuatorServoSetup", component="capabilities", category="Robot",
      description="Set up one discovered servo: inspect settings, change its ID, release torque, and calibrate.",
      inputs={"bus": Dict, "servo_id": Int(default=1), "profile_id": Text(default="")},
      outputs={"report": Text}, primary_inputs=["bus"], primary_outputs=["report"])
def actuator_servo_setup(ctx):
    return {"report": "Press Scan on the USB node to refresh this servo's settings"}


_mock_rows = {}


def _mock_scan(config):
    with _lock:
        rows = _mock_rows.setdefault(config["port"], [{
            "servo_id": 1, "reported_id": 1, "model_number": 777, "model": "Mock actuator",
            "assignment_supported": True,
            "torque_enabled": False, "raw_position": 2048, "voltage_v": 12.0,
            "temperature_c": 25, "hardware_error_flags": 0, "hardware_errors": [], "errors": [],
        }])
        return {"actuators": copy.deepcopy(rows)}


def _mock_assign(config, expected, new_id):
    rows = _mock_scan(config)["actuators"]
    if len(rows) != 1 or rows[0] != expected:
        raise ValueError("Mock actuator changed; scan again")
    if not 1 <= new_id <= 253 or rows[0]["torque_enabled"] or rows[0]["hardware_error_flags"]:
        raise ValueError("Mock assignment blocked by actuator state or invalid ID")
    old_id = rows[0]["servo_id"]
    rows[0].update(servo_id=new_id, reported_id=new_id)
    with _lock:
        _mock_rows[config["port"]] = rows
    return {"assigned": True, "old_id": old_id, "new_id": new_id, "actuators": rows,
            "report": f"Mock ID {old_id} → {new_id} verified; physical hardware was not accessed"}


def _mock_release(config, expected):
    rows = _mock_scan(config)["actuators"]
    if len(rows) != 1 or rows[0]["servo_id"] != expected["servo_id"]:
        raise ValueError("Mock actuator changed; scan again")
    rows[0]["torque_enabled"] = False
    with _lock:
        _mock_rows[config["port"]] = rows
    return {"released": True, "actuators": rows, "report": "Mock torque released; scan again before assignment"}


def _mock_read_position(config, servo_id):
    row = next((row for row in _mock_scan(config)["actuators"] if row["servo_id"] == servo_id), None)
    if row is None:
        raise ValueError("Servo did not respond")
    return {**row, "sampled_at": time.time(), "position_range": {"min": 0, "max": 4095}}


def _mock_test_context(config, state, row):
    from .actuator_states import _limits
    low, home, high = _limits(state)
    scale = 360.0 / 4096
    joint = {"id": "actuator", "servo_id": state["servo_id"], "home_ticks": home,
             "safe_min_deg": (low - home) * scale, "safe_max_deg": (high - home) * scale,
             "velocity_limit": 180.0}
    profile = {"id": "mock_actuator_setup", "joints": [joint], "capability_bindings": {
        "joint_group": {"provider": {"package": "blacknode-robot", "component": "calibration"}}}}
    row = next(row for row in _mock_scan(config)["actuators"] if row["servo_id"] == state["servo_id"])
    return {"profile": profile, "hardware_id": config["hardware_id"], "position_target_mode": True,
            "calibration": {"profile_id": profile["id"], "hardware_id": config["hardware_id"],
                            "joints": {"actuator": joint}},
            "provider_config": {"pose": {"actuator": (row["raw_position"] - home) * scale}},
            "degrees_per_tick": scale}


@node(name="RobotActuatorSetupMockProvider", component="capabilities", category="Robot", hidden=True,
      inputs={}, outputs={"available": Bool, "report": Text})
def robot_actuator_setup_mock_provider(ctx):
    return {"available": True, "report": "In-memory actuator setup provider for hardware-free development"}


robot_actuator_setup_mock_provider._bn_robot_actuator_setup_provider = {
    "package": "blacknode-robot", "component": "capabilities", "scan": _mock_scan, "assign": _mock_assign,
    "release": _mock_release,
    "read_position": _mock_read_position,
    "build_test_context": _mock_test_context,
}
