"""Visual robot definitions, persistent profiles, and guided calibration.

Profiles describe a robot model. Calibrations are stored separately per
physical USB serial so two otherwise-identical arms can keep different zeros
and safe ranges. Calibration only observes released joints; it never commands
motion or treats a mechanical hard stop as an automatically safe limit.
"""
from __future__ import annotations

import base64
import copy
import html
import importlib.util
import json
import os
import re
import threading
import time
from pathlib import Path
from typing import Any

from blacknode.node import Bool, Dict, Enum, Float, Image, Int, List, Text, node

_CATEGORY = "Robot"
_PROFILE_SCHEMA = 1
_TICKS_PER_REV = 4095
_DEFAULT_HOME_TICKS = 2048
_DRIVERS_DIR = Path(__file__).resolve().parents[1] / "drivers"
_ID_PATTERN = re.compile(r"^[a-z][a-z0-9_]{0,63}$")
_calibration_lock = threading.Lock()
_calibration_sessions: dict[str, dict[str, Any]] = {}


def _slug(value: Any, fallback: str = "robot") -> str:
    text = re.sub(r"[^a-z0-9]+", "_", str(value or "").strip().lower()).strip("_")
    if not text:
        text = fallback
    if not text[0].isalpha():
        text = f"robot_{text}"
    return text[:64]


def _profile_root() -> Path:
    configured = str(os.environ.get("BLACKNODE_ROBOTS_DIR") or "").strip()
    return Path(configured).expanduser().resolve() if configured else (Path.cwd() / "robots").resolve()


def _profile_dir(profile_id: str) -> Path:
    return _profile_root() / _slug(profile_id)


def _profile_path(profile_id: str) -> Path:
    return _profile_dir(profile_id) / "profile.json"


def _calibration_path(profile_id: str, hardware_id: str) -> Path:
    return _profile_dir(profile_id) / "calibrations" / f"{_slug(hardware_id, 'device')}.json"


def _write_json(path: Path, payload: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    os.replace(temporary, path)


def _read_json(path: Path) -> dict[str, Any]:
    value = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise ValueError(f"expected a JSON object in {path}")
    return value


def _joint_id(value: Any, fallback: str = "joint") -> str:
    return _slug(value, fallback)


def _joint_list(profile: dict[str, Any]) -> list[dict[str, Any]]:
    joints = profile.get("joints") if isinstance(profile.get("joints"), list) else []
    return [dict(joint) for joint in joints if isinstance(joint, dict)]


def _validate_profile(profile: dict[str, Any]) -> list[str]:
    errors: list[str] = []
    profile_id = str(profile.get("id") or "")
    if not _ID_PATTERN.fullmatch(profile_id):
        errors.append("profile id must be lowercase snake_case, begin with a letter, and be at most 64 characters")
    joints = _joint_list(profile)
    if not joints:
        errors.append("add at least one RobotJointDefinition")
    names: set[str] = set()
    servo_ids: set[int] = set()
    for index, joint in enumerate(joints, start=1):
        name = str(joint.get("id") or "")
        if not _ID_PATTERN.fullmatch(name):
            errors.append(f"joint {index} has invalid id '{name}'")
        if name in names:
            errors.append(f"joint id '{name}' is duplicated")
        names.add(name)
        try:
            servo_id = int(joint.get("servo_id"))
        except (TypeError, ValueError):
            errors.append(f"joint '{name or index}' needs an integer servo id")
            continue
        if servo_id in servo_ids:
            errors.append(f"servo id {servo_id} is duplicated")
        servo_ids.add(servo_id)
        lo = float(joint.get("safe_min_deg", joint.get("min_deg", 0.0)))
        hi = float(joint.get("safe_max_deg", joint.get("max_deg", 0.0)))
        if lo >= hi:
            errors.append(f"joint '{name}' minimum must be below maximum")
    return errors


def _so_arm101_profile() -> dict[str, Any]:
    specs = [
        ("shoulder_pan", "Shoulder pan", 1, -100.0, 100.0),
        ("shoulder_lift", "Shoulder lift", 2, -100.0, 100.0),
        ("elbow_flex", "Elbow flex", 3, -100.0, 100.0),
        ("wrist_flex", "Wrist flex", 4, -100.0, 100.0),
        ("wrist_roll", "Wrist roll", 5, -150.0, 150.0),
        ("gripper", "Gripper", 6, -10.0, 90.0),
    ]
    return {
        "schema_version": _PROFILE_SCHEMA,
        "id": "so_arm101",
        "display_name": "SO-ARM101 (Feetech STS3215 x6)",
        "protocol": "feetech",
        "driver": {
            "script": "feetech_bus_driver.py",
            "baudrate": 1_000_000,
            "transport": "auto",
            "host": "127.0.0.1",
            "port": 9090,
            "rate_hz": 15.0,
            "state_topic": "/joint_states",
            "command_topic": "/joint_commands",
            "config_topic": "/joint_config",
            "control_topic": "/robot_control",
            "units": "degrees",
        },
        "match": {"vendor_id": "", "product_id": ""},
        "joints": [
            {
                "id": name,
                "display_name": label,
                "servo_id": servo_id,
                "min_deg": lo,
                "max_deg": hi,
                "safe_min_deg": lo,
                "safe_max_deg": hi,
                "home_ticks": _DEFAULT_HOME_TICKS,
                "invert": False,
            }
            for name, label, servo_id, lo, hi in specs
        ],
    }


_BUILTINS = {"so_arm101": _so_arm101_profile}


def builtin_profile(profile_id: str) -> dict[str, Any] | None:
    factory = _BUILTINS.get(str(profile_id or "").strip())
    return copy.deepcopy(factory()) if factory else None


def list_profiles() -> list[dict[str, Any]]:
    profiles = [
        {"id": profile_id, "display_name": factory().get("display_name", profile_id), "builtin": True, "path": ""}
        for profile_id, factory in sorted(_BUILTINS.items())
    ]
    root = _profile_root()
    if root.exists():
        for path in sorted(root.glob("*/profile.json")):
            try:
                profile = _read_json(path)
            except Exception:
                continue
            profiles = [entry for entry in profiles if entry["id"] != profile.get("id")]
            profiles.append({
                "id": str(profile.get("id") or path.parent.name),
                "display_name": str(profile.get("display_name") or profile.get("id") or path.parent.name),
                "builtin": False,
                "path": str(path),
            })
    return sorted(profiles, key=lambda item: str(item["id"]))


def load_profile(profile_id: str) -> tuple[dict[str, Any] | None, Path | None]:
    path = _profile_path(profile_id)
    if path.exists():
        return _read_json(path), path
    return builtin_profile(profile_id), None


def _hardware_id(ctx: dict[str, Any]) -> str:
    explicit = str(ctx.get("hardware_id") or "").strip()
    hardware = ctx.get("hardware") if isinstance(ctx.get("hardware"), dict) else {}
    recommended = hardware.get("recommended") if isinstance(hardware.get("recommended"), dict) else {}
    return explicit or str(
        hardware.get("serial")
        or hardware.get("serial_number")
        or recommended.get("serial")
        or hardware.get("path")
        or recommended.get("path")
        or ""
    ).strip()


def _apply_calibration(profile: dict[str, Any], calibration: dict[str, Any] | None) -> dict[str, Any]:
    result = copy.deepcopy(profile)
    if not calibration:
        return result
    overrides = calibration.get("joints") if isinstance(calibration.get("joints"), dict) else {}
    joints = _joint_list(result)
    for joint in joints:
        values = overrides.get(str(joint.get("id")))
        if isinstance(values, dict):
            joint.update(values)
    result["joints"] = joints
    result["calibration"] = copy.deepcopy(calibration)
    return result


def _driver_from_profile(profile: dict[str, Any], hardware_id: str = "") -> dict[str, Any]:
    profile_id = str(profile.get("id") or "robot")
    calibration: dict[str, Any] = {}
    calibration_path: Path | None = None
    if hardware_id:
        candidate = _calibration_path(profile_id, hardware_id)
        if candidate.exists():
            calibration = _read_json(candidate)
            calibration_path = candidate
    effective = _apply_calibration(profile, calibration)
    driver_cfg = effective.get("driver") if isinstance(effective.get("driver"), dict) else {}
    joints = _joint_list(effective)
    protocol = str(effective.get("protocol") or "custom")
    script_value = str(driver_cfg.get("script") or "")
    script = Path(script_value)
    if script_value and not script.is_absolute():
        script = _DRIVERS_DIR / script
    requested_transport = str(driver_cfg.get("transport") or "auto")
    transport = (
        "native" if importlib.util.find_spec("rclpy") is not None else "rosbridge"
    ) if requested_transport == "auto" else requested_transport
    host = str(driver_cfg.get("host") or "127.0.0.1")
    port = int(driver_cfg.get("port") or 9090)
    rate_hz = float(driver_cfg.get("rate_hz") or 15.0)
    state_topic = str(driver_cfg.get("state_topic") or "/joint_states")
    command_topic = str(driver_cfg.get("command_topic") or "/joint_commands")
    config_topic = str(driver_cfg.get("config_topic") or "/joint_config")
    control_topic = str(driver_cfg.get("control_topic") or "/robot_control")
    command_template = str(driver_cfg.get("command_template") or "").strip()
    if protocol == "feetech":
        joint_arg = ",".join(
            f"{joint['id']}:{int(joint['servo_id'])}:{float(joint.get('safe_min_deg', joint.get('min_deg', -180))):g}:"
            f"{float(joint.get('safe_max_deg', joint.get('max_deg', 180))):g}"
            for joint in joints
        )
        home_arg = ",".join(f"{joint['id']}:{int(joint.get('home_ticks', _DEFAULT_HOME_TICKS))}" for joint in joints)
        inverted = ",".join(str(joint["id"]) for joint in joints if bool(joint.get("invert")))
        command_template = (
            f'"{{python}}" "{script}" --port "{{serial_port}}" --baudrate {int(driver_cfg.get("baudrate") or 1_000_000)} '
            f'--joints "{joint_arg}" --home-ticks "{home_arg}" --invert "{inverted}" '
            f'--state-topic {{state_topic}} --command-topic {{command_topic}} --config-topic {{config_topic}} '
            f'--control-topic {{control_topic}} --rate-hz {rate_hz:g} --transport {transport} '
            f'--host "{host}" --rosbridge-port {port}'
        )
    return {
        "id": profile_id,
        "profile_id": profile_id,
        "name": str(effective.get("display_name") or profile_id),
        "command_template": command_template,
        "transport": transport,
        "requested_transport": requested_transport,
        "host": host,
        "port": port,
        "state_topic": state_topic,
        "command_topic": command_topic,
        "config_topic": config_topic,
        "control_topic": control_topic,
        "units": str(driver_cfg.get("units") or "degrees"),
        "match": dict(effective.get("match") or {}),
        "joints": joints,
        "profile": effective,
        "hardware_id": hardware_id,
        "calibration_path": str(calibration_path or ""),
    }


@node(
    name="RobotJointDefinition",
    category=_CATEGORY,
    description="Define one stable robot joint: its user-facing label, bus id, safe limits, zero, and direction.",
    inputs={
        "joint_id": Text(default="joint"),
        "display_name": Text(default="Joint"),
        "servo_id": Int(default=1),
        "min_deg": Float(default=-90.0),
        "max_deg": Float(default=90.0),
        "home_ticks": Int(default=_DEFAULT_HOME_TICKS),
        "invert": Bool(default=False),
        "velocity_limit": Float(default=0.0),
        "torque_limit": Float(default=0.0),
    },
    outputs={"joint": Dict, "report": Text},
)
def robot_joint_definition(ctx: dict) -> dict:
    requested = str(ctx.get("joint_id") or "joint")
    joint_id = _joint_id(requested)
    lo = float(ctx.get("min_deg") if ctx.get("min_deg") is not None else -90.0)
    hi = float(ctx.get("max_deg") if ctx.get("max_deg") is not None else 90.0)
    joint = {
        "id": joint_id,
        "display_name": str(ctx.get("display_name") or joint_id),
        "servo_id": int(ctx.get("servo_id") or 1),
        "min_deg": lo,
        "max_deg": hi,
        "safe_min_deg": lo,
        "safe_max_deg": hi,
        "home_ticks": int(ctx.get("home_ticks") or _DEFAULT_HOME_TICKS),
        "invert": bool(ctx.get("invert")),
        "velocity_limit": max(0.0, float(ctx.get("velocity_limit") or 0.0)),
        "torque_limit": max(0.0, float(ctx.get("torque_limit") or 0.0)),
    }
    notes = []
    if requested != joint_id:
        notes.append(f"normalized id '{requested}' -> '{joint_id}'")
    if lo >= hi:
        notes.append("INVALID: minimum must be below maximum")
    report = f"joint {joint_id}: servo {joint['servo_id']}, safe range {lo:g}..{hi:g} degrees"
    if notes:
        report += "\n" + "\n".join(notes)
    return {"joint": joint, "report": report}


_JOINT_INPUTS = {f"joint_{index}": Dict for index in range(1, 17)}


@node(
    name="RobotJointList",
    category=_CATEGORY,
    description="Collect up to 16 RobotJointDefinition outputs into one ordered joint list.",
    inputs=_JOINT_INPUTS,
    outputs={"joints": List, "count": Int, "report": Text},
)
def robot_joint_list(ctx: dict) -> dict:
    joints = [dict(ctx[name]) for name in _JOINT_INPUTS if isinstance(ctx.get(name), dict) and ctx.get(name)]
    return {"joints": joints, "count": len(joints), "report": f"assembled {len(joints)} joint definition(s)"}


@node(
    name="RobotDefinition",
    category=_CATEGORY,
    description="Assemble an editable robot model and executable driver descriptor from ordinary graph inputs.",
    inputs={
        "profile_id": Text(default="my_robot"),
        "display_name": Text(default="My Robot"),
        "protocol": Enum(["feetech", "custom"], default="feetech"),
        "driver_script": Text(default="feetech_bus_driver.py"),
        "command_template": Text(default=""),
        "baudrate": Int(default=1_000_000),
        "joints": List,
        "vendor_id": Text(default=""),
        "product_id": Text(default=""),
        "transport": Enum(["auto", "native", "rosbridge"], default="auto"),
        "host": Text(default="127.0.0.1"),
        "port": Int(default=9090),
        "state_topic": Text(default="/joint_states"),
        "command_topic": Text(default="/joint_commands"),
        "config_topic": Text(default="/joint_config"),
        "control_topic": Text(default="/robot_control"),
        "rate_hz": Float(default=15.0),
        "units": Enum(["degrees", "radians"], default="degrees"),
    },
    outputs={"valid": Bool, "profile": Dict, "driver": Dict, "report": Text},
)
def robot_definition(ctx: dict) -> dict:
    requested_id = str(ctx.get("profile_id") or "my_robot")
    profile_id = _slug(requested_id, "my_robot")
    profile = {
        "schema_version": _PROFILE_SCHEMA,
        "id": profile_id,
        "display_name": str(ctx.get("display_name") or profile_id),
        "protocol": str(ctx.get("protocol") or "feetech"),
        "driver": {
            "script": str(ctx.get("driver_script") or ""),
            "command_template": str(ctx.get("command_template") or ""),
            "baudrate": int(ctx.get("baudrate") or 1_000_000),
            "transport": str(ctx.get("transport") or "auto"),
            "host": str(ctx.get("host") or "127.0.0.1"),
            "port": int(ctx.get("port") or 9090),
            "state_topic": str(ctx.get("state_topic") or "/joint_states"),
            "command_topic": str(ctx.get("command_topic") or "/joint_commands"),
            "config_topic": str(ctx.get("config_topic") or "/joint_config"),
            "control_topic": str(ctx.get("control_topic") or "/robot_control"),
            "rate_hz": float(ctx.get("rate_hz") or 15.0),
            "units": str(ctx.get("units") or "degrees"),
        },
        "match": {
            "vendor_id": str(ctx.get("vendor_id") or "").strip().lower(),
            "product_id": str(ctx.get("product_id") or "").strip().lower(),
        },
        "joints": [dict(value) for value in (ctx.get("joints") or []) if isinstance(value, dict)],
    }
    errors = _validate_profile(profile)
    notes = [] if requested_id == profile_id else [f"normalized profile id '{requested_id}' -> '{profile_id}'"]
    report = f"robot definition: {profile['display_name']} ({profile_id}), {len(profile['joints'])} joint(s)"
    if notes:
        report += "\n" + "\n".join(notes)
    if errors:
        report += "\nINVALID:\n- " + "\n- ".join(errors)
    return {"valid": not errors, "profile": profile, "driver": _driver_from_profile(profile), "report": report}


@node(
    name="RobotProfileSave",
    category=_CATEGORY,
    description="Save an editable robot definition to robots/<profile_id>/profile.json for reuse.",
    inputs={"profile": Dict, "overwrite": Bool(default=False)},
    outputs={"saved": Bool, "profile": Dict, "driver": Dict, "path": Text, "report": Text},
)
def robot_profile_save(ctx: dict) -> dict:
    profile = copy.deepcopy(ctx.get("profile") if isinstance(ctx.get("profile"), dict) else {})
    errors = _validate_profile(profile)
    path = _profile_path(str(profile.get("id") or "robot"))
    if errors:
        return {"saved": False, "profile": profile, "driver": {}, "path": str(path), "report": "profile not saved:\n- " + "\n- ".join(errors)}
    if path.exists() and not bool(ctx.get("overwrite")):
        return {
            "saved": False,
            "profile": profile,
            "driver": _driver_from_profile(profile),
            "path": str(path),
            "report": f"profile already exists: {path}\nSet overwrite=true after reviewing the definition.",
        }
    payload = copy.deepcopy(profile)
    payload["schema_version"] = _PROFILE_SCHEMA
    payload["saved_at"] = time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())
    _write_json(path, payload)
    return {
        "saved": True,
        "profile": payload,
        "driver": _driver_from_profile(payload),
        "path": str(path),
        "report": f"saved robot profile '{payload['id']}'\n{path}",
    }


@node(
    name="RobotProfileLoad",
    category=_CATEGORY,
    description="Load a reusable robot profile and automatically apply calibration for a physical USB serial when supplied.",
    inputs={"profile_id": Text(default="so_arm101"), "hardware_id": Text(default=""), "hardware": Dict},
    outputs={"found": Bool, "profile": Dict, "driver": Dict, "calibration": Dict, "path": Text, "report": Text},
)
def robot_profile_load(ctx: dict) -> dict:
    profile_id = _slug(ctx.get("profile_id") or "so_arm101")
    profile, path = load_profile(profile_id)
    if profile is None:
        known = ", ".join(item["id"] for item in list_profiles()) or "none"
        return {"found": False, "profile": {}, "driver": {}, "calibration": {}, "path": "", "report": f"robot profile '{profile_id}' not found (available: {known})"}
    hardware_id = _hardware_id(ctx)
    driver = _driver_from_profile(profile, hardware_id)
    effective = dict(driver.get("profile") or profile)
    return {
        "found": True,
        "profile": effective,
        "driver": driver,
        "calibration": dict(effective.get("calibration") or {}),
        "path": str(path or "builtin"),
        "report": (
            f"loaded robot profile '{profile_id}' ({len(_joint_list(effective))} joint(s))"
            + (f"\ncalibration: {driver['calibration_path']}" if driver.get("calibration_path") else "\ncalibration: none")
        ),
    }


@node(
    name="RobotProfileList",
    category=_CATEGORY,
    description="List built-in and locally saved robot profiles.",
    inputs={"refresh": Text(default="")},
    outputs={"profiles": List, "count": Int, "root": Text, "report": Text},
)
def robot_profile_list(ctx: dict) -> dict:
    del ctx
    profiles = list_profiles()
    return {
        "profiles": profiles,
        "count": len(profiles),
        "root": str(_profile_root()),
        "report": "robot profiles:\n" + "\n".join(f"- {item['id']}: {item['display_name']}{' (built-in)' if item['builtin'] else ''}" for item in profiles),
    }


@node(
    name="RobotProfileDuplicate",
    category=_CATEGORY,
    description="Duplicate a built-in or saved profile under a new editable lowercase id.",
    inputs={
        "source_profile_id": Text(default="so_arm101"),
        "new_profile_id": Text(default="my_robot"),
        "display_name": Text(default="My Robot"),
        "overwrite": Bool(default=False),
    },
    outputs={"saved": Bool, "profile": Dict, "driver": Dict, "path": Text, "report": Text},
)
def robot_profile_duplicate(ctx: dict) -> dict:
    source, _path = load_profile(str(ctx.get("source_profile_id") or "so_arm101"))
    if source is None:
        return {"saved": False, "profile": {}, "driver": {}, "path": "", "report": "source robot profile not found"}
    profile = copy.deepcopy(source)
    profile["id"] = _slug(ctx.get("new_profile_id") or "my_robot")
    profile["display_name"] = str(ctx.get("display_name") or profile["id"])
    profile.pop("calibration", None)
    return robot_profile_save({"profile": profile, "overwrite": bool(ctx.get("overwrite"))})


def _sample_session(session: dict[str, Any], pose: dict[str, Any]) -> int:
    accepted = 0
    allowed = {str(joint.get("id")) for joint in _joint_list(session["profile"])}
    for name, raw in pose.items():
        if name not in allowed or not isinstance(raw, (int, float)):
            continue
        value = float(raw)
        bounds = session["observed"].setdefault(name, {"min_deg": value, "max_deg": value})
        bounds["min_deg"] = min(float(bounds["min_deg"]), value)
        bounds["max_deg"] = max(float(bounds["max_deg"]), value)
        accepted += 1
    if accepted:
        session["samples"] += 1
        session["last_pose"] = {str(k): float(v) for k, v in pose.items() if isinstance(v, (int, float))}
        session["updated_at"] = time.time()
    return accepted


def _calibration_dashboard(session: dict[str, Any] | None, report: str) -> str:
    active = bool(session and session.get("active"))
    accent = "#f59e0b" if active else "#22c55e"
    rows = []
    observed = dict(session.get("observed") or {}) if session else {}
    home = dict(session.get("home") or {}) if session else {}
    for index, name in enumerate(sorted(observed)[:8]):
        bounds = observed[name]
        y = 178 + index * 44
        rows.append(
            f'<text x="46" y="{y}" fill="#f8fafc" font-family="monospace" font-size="15">{html.escape(name)}</text>'
            f'<text x="410" y="{y}" text-anchor="end" fill="#93a4b8" font-family="monospace" font-size="15">{float(bounds["min_deg"]):.2f} .. {float(bounds["max_deg"]):.2f}</text>'
            f'<text x="620" y="{y}" text-anchor="end" fill="{accent}" font-family="monospace" font-size="15">{float(home[name]):.2f}</text>'
            if name in home else
            f'<text x="46" y="{y}" fill="#f8fafc" font-family="monospace" font-size="15">{html.escape(name)}</text>'
            f'<text x="410" y="{y}" text-anchor="end" fill="#93a4b8" font-family="monospace" font-size="15">{float(bounds["min_deg"]):.2f} .. {float(bounds["max_deg"]):.2f}</text>'
            f'<text x="620" y="{y}" text-anchor="end" fill="#64748b" font-family="monospace" font-size="15">-</text>'
        )
    if not rows:
        rows.append('<text x="340" y="250" text-anchor="middle" fill="#93a4b8" font-family="Arial" font-size="17">Start recording, then move each released joint slowly.</text>')
    state = "RECORDING" if active else "IDLE / SAVED"
    samples = int(session.get("samples") or 0) if session else 0
    safe_report = html.escape(report[:100])
    svg = f'''<svg xmlns="http://www.w3.org/2000/svg" width="680" height="590" viewBox="0 0 680 590">
<rect width="680" height="590" rx="22" fill="#0b1020"/>
<rect x="22" y="22" width="636" height="100" rx="16" fill="#172033" stroke="{accent}" stroke-width="2"/>
<text x="44" y="58" fill="#f8fafc" font-family="Arial" font-size="23" font-weight="800">ROBOT CALIBRATION</text>
<text x="44" y="91" fill="{accent}" font-family="Arial" font-size="16" font-weight="800">{state} · {samples} SAMPLES</text>
<text x="46" y="146" fill="#93a4b8" font-family="Arial" font-size="12">JOINT</text>
<text x="410" y="146" text-anchor="end" fill="#93a4b8" font-family="Arial" font-size="12">OBSERVED RANGE</text>
<text x="620" y="146" text-anchor="end" fill="#93a4b8" font-family="Arial" font-size="12">HOME</text>
{''.join(rows)}
<text x="40" y="552" fill="#93a4b8" font-family="Arial" font-size="13">{safe_report}</text>
</svg>'''
    return "data:image/svg+xml;base64," + base64.b64encode(svg.encode("utf-8")).decode("ascii")


def _session_outputs(session: dict[str, Any] | None, report: str, *, saved: bool = False, path: str = "") -> dict[str, Any]:
    profile = copy.deepcopy(session.get("effective_profile") or session.get("profile") or {}) if session else {}
    calibration = copy.deepcopy(session.get("calibration") or {}) if session else {}
    hardware_id = str(session.get("hardware_id") or "") if session else ""
    return {
        "active": bool(session and session.get("active")),
        "data_ready": bool(session and session.get("observed")),
        "samples": int(session.get("samples") or 0) if session else 0,
        "observed": copy.deepcopy(session.get("observed") or {}) if session else {},
        "home": copy.deepcopy(session.get("home") or {}) if session else {},
        "calibration": calibration,
        "profile": profile,
        "driver": _driver_from_profile(profile, hardware_id) if profile else {},
        "saved": saved,
        "path": path,
        "dashboard": _calibration_dashboard(session, report),
        "report": report,
    }


@node(
    name="RobotCalibrationRecorder",
    category=_CATEGORY,
    live=True,
    description="Record released-joint extrema and home pose, review a safety margin, and save calibration per physical robot.",
    inputs={
        "action": Enum(["check", "start", "capture_home", "finish", "cancel"], default="check"),
        "run_id": Text(default="robot_calibration"),
        "profile": Dict,
        "hardware_id": Text(default=""),
        "hardware": Dict,
        "pose": Dict,
        "torque_enabled": Bool(default=True),
        "require_released": Bool(default=True),
        "safety_margin_deg": Float(default=3.0),
    },
    outputs={
        "active": Bool,
        "data_ready": Bool,
        "samples": Int,
        "observed": Dict,
        "home": Dict,
        "calibration": Dict,
        "profile": Dict,
        "driver": Dict,
        "saved": Bool,
        "path": Text,
        "dashboard": Image,
        "report": Text,
    },
)
def robot_calibration_recorder(ctx: dict) -> dict:
    action = str(ctx.get("action") or "check").strip().lower()
    run_id = str(ctx.get("run_id") or "robot_calibration").strip() or "robot_calibration"
    pose = dict(ctx.get("pose") or {})
    torque_enabled = bool(ctx.get("torque_enabled", True))
    require_released = bool(ctx.get("require_released", True))
    with _calibration_lock:
        session = _calibration_sessions.get(run_id)
        if action == "start":
            profile = copy.deepcopy(ctx.get("profile") if isinstance(ctx.get("profile"), dict) else {})
            errors = _validate_profile(profile)
            hardware_id = _hardware_id(ctx)
            if errors:
                return _session_outputs(None, "calibration blocked: invalid robot profile: " + "; ".join(errors))
            if not hardware_id:
                return _session_outputs(None, "calibration blocked: connect hardware or set hardware_id so results belong to one physical robot")
            if require_released and torque_enabled:
                return _session_outputs(None, "calibration blocked: torque is on. Support the arm and use Release + live pose first.")
            session = {
                "run_id": run_id,
                "profile": profile,
                "hardware_id": hardware_id,
                "observed": {},
                "home": {},
                "samples": 0,
                "active": True,
                "margin": max(0.0, float(ctx.get("safety_margin_deg") or 0.0)),
                "started_at": time.time(),
                "updated_at": time.time(),
            }
            _calibration_sessions[run_id] = session
            _sample_session(session, pose)
            return _session_outputs(session, "RECORDING: torque is off. Support the arm and slowly sweep each joint through the intended usable range.")
        if session is None:
            return _session_outputs(None, "calibration is idle. Release torque, then press Start recording.")
        if action in {"_sample", "sample"}:
            if require_released and torque_enabled:
                session["active"] = False
                return _session_outputs(session, "calibration paused: torque became enabled; release it before recording more samples")
            if session.get("active"):
                _sample_session(session, pose)
            return _session_outputs(session, "RECORDING: move every released joint slowly; capture Home when the robot is in its neutral pose.")
        if action == "capture_home":
            if require_released and torque_enabled:
                return _session_outputs(session, "home not captured: torque is on")
            _sample_session(session, pose)
            allowed = {str(joint.get("id")) for joint in _joint_list(session["profile"])}
            session["home"] = {name: float(value) for name, value in pose.items() if name in allowed and isinstance(value, (int, float))}
            return _session_outputs(session, f"captured neutral Home for {len(session['home'])} joint(s); continue sweeping or press Save calibration")
        if action == "cancel":
            session["active"] = False
            _calibration_sessions.pop(run_id, None)
            return _session_outputs(session, "calibration cancelled; no files were changed")
        if action == "finish":
            profile = copy.deepcopy(session["profile"])
            joints = _joint_list(profile)
            missing_observed = [str(joint.get("id")) for joint in joints if str(joint.get("id")) not in session["observed"]]
            missing_home = [str(joint.get("id")) for joint in joints if str(joint.get("id")) not in session["home"]]
            if missing_observed or missing_home:
                details = []
                if missing_observed:
                    details.append("not observed: " + ", ".join(missing_observed))
                if missing_home:
                    details.append("home missing: " + ", ".join(missing_home))
                return _session_outputs(session, "calibration not saved: " + "; ".join(details))
            margin = float(session.get("margin") or 0.0)
            overrides: dict[str, Any] = {}
            invalid: list[str] = []
            for joint in joints:
                name = str(joint["id"])
                observed = session["observed"][name]
                absolute_lo = float(observed["min_deg"])
                absolute_hi = float(observed["max_deg"])
                home_deg = float(session["home"][name])
                safe_lo = absolute_lo + margin
                safe_hi = absolute_hi - margin
                if safe_lo >= safe_hi:
                    invalid.append(f"{name} moved only {absolute_hi - absolute_lo:.2f}°, smaller than the {margin:g}° margin on both sides")
                    continue
                base_ticks = int(joint.get("home_ticks", _DEFAULT_HOME_TICKS))
                direction = -1.0 if bool(joint.get("invert")) else 1.0
                calibrated_ticks = max(0, min(_TICKS_PER_REV - 1, round(base_ticks + direction * home_deg * _TICKS_PER_REV / 360.0)))
                overrides[name] = {
                    "home_ticks": calibrated_ticks,
                    "home_offset_deg": home_deg,
                    "observed_min_deg": absolute_lo - home_deg,
                    "observed_max_deg": absolute_hi - home_deg,
                    "safe_min_deg": safe_lo - home_deg,
                    "safe_max_deg": safe_hi - home_deg,
                }
            if invalid:
                return _session_outputs(session, "calibration not saved:\n- " + "\n- ".join(invalid))
            calibration = {
                "schema_version": _PROFILE_SCHEMA,
                "profile_id": str(profile["id"]),
                "hardware_id": str(session["hardware_id"]),
                "recorded_at": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
                "units": "degrees",
                "safety_margin_deg": margin,
                "samples": int(session["samples"]),
                "joints": overrides,
            }
            profile_path = _profile_path(str(profile["id"]))
            if not profile_path.exists():
                _write_json(profile_path, profile)
            path = _calibration_path(str(profile["id"]), str(session["hardware_id"]))
            _write_json(path, calibration)
            session["calibration"] = calibration
            session["effective_profile"] = _apply_calibration(profile, calibration)
            session["active"] = False
            return _session_outputs(session, f"saved calibration for {session['hardware_id']}\n{path}", saved=True, path=str(path))
        return _session_outputs(session, "calibration recording is active" if session.get("active") else "calibration recording is complete")


def calibration_runtime_status() -> dict[str, Any]:
    with _calibration_lock:
        sessions = [
            {
                "run_id": run_id,
                "kind": "robot_calibration",
                "active": bool(session.get("active")),
                "samples": int(session.get("samples") or 0),
                "hardware_id": str(session.get("hardware_id") or ""),
                "profile_id": str(session.get("profile", {}).get("id") or ""),
                "updated_at": session.get("updated_at"),
            }
            for run_id, session in _calibration_sessions.items()
        ]
    return {"sessions": sessions, "active": any(item["active"] for item in sessions)}


def stop_calibration_services() -> int:
    with _calibration_lock:
        count = len(_calibration_sessions)
        _calibration_sessions.clear()
    return count
