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

from blacknode.node import Any as AnyPort
from blacknode.node import Bool, Dict, Enum, Float, Image, Int, List, Text, node

_CATEGORY = "Robot"
_PROFILE_SCHEMA = 1
_TICKS_PER_REV = 4095
_DEFAULT_HOME_TICKS = 2048
_DRIVERS_DIR = Path(__file__).resolve().parents[1] / "drivers"
_ID_PATTERN = re.compile(r"^[a-z][a-z0-9_]{0,63}$")
_USB_ID_PATTERN = re.compile(r"^[0-9a-f]{4}$")
_calibration_lock = threading.Lock()
_calibration_sessions: dict[str, dict[str, Any]] = {}


def _available_driver_scripts() -> list[str]:
    """Return installed driver entry points for editor dropdown metadata."""
    if not _DRIVERS_DIR.exists():
        return []
    return sorted(
        path.name
        for path in _DRIVERS_DIR.iterdir()
        if path.is_file() and not path.name.startswith("_") and path.name.endswith("_driver.py")
    )


_DRIVER_SCRIPT_CHOICES = _available_driver_scripts()
_DEFAULT_DRIVER_SCRIPT = (
    "feetech_bus_driver.py"
    if "feetech_bus_driver.py" in _DRIVER_SCRIPT_CHOICES
    else (_DRIVER_SCRIPT_CHOICES[0] if _DRIVER_SCRIPT_CHOICES else "")
)


def _slug(value: Any, fallback: str = "robot") -> str:
    text = re.sub(r"[^a-z0-9]+", "_", str(value or "").strip().lower()).strip("_")
    if not text:
        text = fallback
    if not text[0].isalpha():
        text = f"robot_{text}"
    return text[:64]


def _profile_root() -> Path:
    configured = str(os.environ.get("BLACKNODE_ROBOTS_DIR") or "").strip()
    return (
        Path(configured).expanduser().resolve()
        if configured
        else (Path.home() / ".blacknode" / "robots").resolve()
    )


def _legacy_profile_root() -> Path | None:
    """Return the pre-0.5.6 working-directory store when it is distinct."""
    if str(os.environ.get("BLACKNODE_ROBOTS_DIR") or "").strip():
        return None
    legacy = (Path.cwd() / "robots").resolve()
    return legacy if legacy != _profile_root() else None


def _profile_roots() -> list[Path]:
    roots = [_profile_root()]
    legacy = _legacy_profile_root()
    if legacy is not None:
        roots.append(legacy)
    return roots


def _profile_dir(profile_id: str) -> Path:
    return _profile_root() / _slug(profile_id)


def _profile_path(profile_id: str) -> Path:
    return _profile_dir(profile_id) / "profile.json"


def _calibration_path(profile_id: str, hardware_id: str) -> Path:
    return _profile_dir(profile_id) / "calibrations" / f"{_slug(hardware_id, 'device')}.json"


def _find_calibration_path(profile_id: str, hardware_id: str) -> Path | None:
    relative = Path(_slug(profile_id)) / "calibrations" / f"{_slug(hardware_id, 'device')}.json"
    return next((root / relative for root in _profile_roots() if (root / relative).exists()), None)


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
    capability_bindings = (
        profile.get("capability_bindings")
        if isinstance(profile.get("capability_bindings"), dict)
        else {}
    )
    if not joints and not capability_bindings:
        errors.append("add at least one RobotJointDefinition or robot capability binding")
    for capability, binding in capability_bindings.items():
        if not _ID_PATTERN.fullmatch(str(capability or "")):
            errors.append(f"capability '{capability}' needs a lowercase stable id")
            continue
        provider = binding.get("provider") if isinstance(binding, dict) and isinstance(binding.get("provider"), dict) else {}
        if not str(provider.get("package") or "").strip() or not str(provider.get("component") or "").strip():
            errors.append(f"capability '{capability}' needs a provider package and component")
    attachment_ids: set[str] = set()
    attachments = (
        profile.get("attachments")
        if isinstance(profile.get("attachments"), list)
        else []
    )
    for index, attachment in enumerate(attachments, start=1):
        if not isinstance(attachment, dict):
            errors.append(f"attachment {index} must be an object")
            continue
        attachment_id = str(attachment.get("id") or "")
        if not _ID_PATTERN.fullmatch(attachment_id):
            errors.append(f"attachment {index} has invalid id '{attachment_id}'")
        elif attachment_id in attachment_ids:
            errors.append(f"attachment id '{attachment_id}' is duplicated")
        attachment_ids.add(attachment_id)
        if not str(attachment.get("frame_id") or "").strip():
            errors.append(f"attachment '{attachment_id or index}' needs a frame id")
        interfaces = (
            attachment.get("interfaces")
            if isinstance(attachment.get("interfaces"), list)
            else []
        )
        if not interfaces:
            errors.append(f"attachment '{attachment_id or index}' needs at least one interface")
        for interface in interfaces:
            if not isinstance(interface, dict):
                errors.append(f"attachment '{attachment_id or index}' has an invalid interface")
                continue
            if str(interface.get("kind") or "") == "topic":
                topic = str(interface.get("topic") or "").strip()
                candidates = interface.get("candidates")
                if not topic and not (
                    isinstance(candidates, list)
                    and any(str(value or "").strip() for value in candidates)
                ):
                    errors.append(f"attachment '{attachment_id or index}' needs a ROS 2 topic")
                if not str(interface.get("message_type") or "").strip():
                    errors.append(
                        f"attachment '{attachment_id or index}' needs a ROS 2 message type"
                    )
    match = profile.get("match") if isinstance(profile.get("match"), dict) else {}
    for key, label in (("vendor_id", "vendor id"), ("product_id", "product id")):
        value = str(match.get(key) or "")
        if value and not _USB_ID_PATTERN.fullmatch(value):
            errors.append(f"USB {label} must be four hexadecimal characters, for example 1a86")
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
            "rate_hz": 60.0,
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
    for root in reversed(_profile_roots()):
        if not root.exists():
            continue
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
    relative = Path(_slug(profile_id)) / "profile.json"
    for root in _profile_roots():
        path = root / relative
        if path.exists():
            return _read_json(path), path
    return builtin_profile(profile_id), None


def _available_profile_ids() -> list[str]:
    ids = [str(item["id"]) for item in list_profiles()]
    return ids or ["so_arm101"]


def _available_robot_profile_ids() -> list[str]:
    return ["auto", *_available_profile_ids()]


def _hardware_id(ctx: dict[str, Any]) -> str:
    explicit = str(ctx.get("hardware_id") or "").strip()
    hardware = ctx.get("hardware") if isinstance(ctx.get("hardware"), dict) else {}
    recommended = hardware.get("recommended") if isinstance(hardware.get("recommended"), dict) else {}
    return explicit or str(
        recommended.get("serial")
        or recommended.get("serial_number")
        or hardware.get("serial")
        or hardware.get("serial_number")
        or recommended.get("path")
        or hardware.get("path")
        or ""
    ).strip()


def _normalized_hardware_id(value: Any) -> str:
    return str(value or "").strip().replace("\\", "/").casefold()


def _hardware_matches_id(hardware: dict[str, Any], hardware_id: str) -> bool:
    expected = _normalized_hardware_id(hardware_id)
    if not expected:
        return False
    return any(
        _normalized_hardware_id(hardware.get(key)) == expected
        for key in ("serial", "serial_number", "path", "hardware_id")
    )


def _hardware_details(ctx: dict[str, Any]) -> dict[str, Any]:
    hardware = ctx.get("hardware") if isinstance(ctx.get("hardware"), dict) else {}
    recommended = hardware.get("recommended") if isinstance(hardware.get("recommended"), dict) else {}
    return recommended or hardware


def _usb_id(value: Any) -> str:
    text = str(value or "").strip().lower()
    return text[2:] if text.startswith("0x") else text


def _profile_hardware_identity(profile: dict[str, Any]) -> str:
    identity = (
        profile.get("hardware_identity")
        if isinstance(profile.get("hardware_identity"), dict)
        else {}
    )
    return str(
        identity.get("id")
        or identity.get("serial")
        or identity.get("serial_number")
        or identity.get("path")
        or ""
    ).strip()


def _auto_profile_for_hardware(
    hardware: dict[str, Any],
) -> tuple[dict[str, Any] | None, Path | None, str]:
    candidates: list[tuple[dict[str, Any], Path | None]] = []
    for item in list_profiles():
        profile, path = load_profile(str(item["id"]))
        if profile is not None:
            candidates.append((profile, path))
    if not candidates:
        return None, None, "no robot profiles are installed"

    recommended = (
        hardware.get("recommended")
        if isinstance(hardware.get("recommended"), dict)
        else {}
    )
    hardware_id = _hardware_id({"hardware": hardware})
    serial = str(
        recommended.get("serial")
        or recommended.get("serial_number")
        or hardware.get("serial")
        or hardware.get("serial_number")
        or ""
    ).strip()
    path_value = str(recommended.get("path") or hardware.get("path") or "").strip()
    vendor_id = _usb_id(recommended.get("vendor_id") or hardware.get("vendor_id"))
    product_id = _usb_id(recommended.get("product_id") or hardware.get("product_id"))

    ranked: list[tuple[int, dict[str, Any], Path | None, str]] = []
    for profile, path in candidates:
        profile_id = str(profile.get("id") or "")
        match = profile.get("match") if isinstance(profile.get("match"), dict) else {}
        expected_hardware = _profile_hardware_identity(profile)
        expected_serial = str(
            match.get("serial")
            or match.get("serial_number")
            or ""
        ).strip()
        expected_path = str(match.get("path") or "").strip()
        expected_vendor = _usb_id(match.get("vendor_id"))
        expected_product = _usb_id(match.get("product_id"))
        score = 0
        reason = ""
        if hardware_id and _find_calibration_path(profile_id, hardware_id) is not None:
            score, reason = 400, "saved calibration for this physical device"
        elif expected_hardware and expected_hardware in {hardware_id, serial, path_value}:
            score, reason = 300, "saved physical hardware identity"
        elif expected_serial and serial and expected_serial == serial:
            score, reason = 250, "USB serial"
        elif expected_path and path_value and expected_path == path_value:
            score, reason = 225, "saved device path"
        elif (
            expected_vendor
            and expected_product
            and expected_vendor == vendor_id
            and expected_product == product_id
        ):
            score, reason = 200, "USB vendor/product"
        ranked.append((score, profile, path, reason))

    best_score = max(item[0] for item in ranked)
    best = [item for item in ranked if item[0] == best_score]
    if best_score > 0 and len(best) == 1:
        _score, profile, path, reason = best[0]
        return profile, path, reason
    if len(candidates) == 1:
        profile, path = candidates[0]
        return profile, path, "only installed robot profile"
    names = ", ".join(str(profile.get("display_name") or profile.get("id")) for profile, _ in candidates)
    return (
        None,
        None,
        f"multiple profiles could match ({names}); select one profile once to bind this device",
    )


def _profile_with_default_capabilities(
    profile: dict[str, Any],
    driver: dict[str, Any],
) -> dict[str, Any]:
    result = copy.deepcopy(profile)
    existing = result.get("capability_bindings")
    if isinstance(existing, dict) and existing:
        result["capabilities"] = list(existing)
        return result
    if not _joint_list(result):
        return result

    protocol = _slug(result.get("protocol") or driver.get("id") or "joint_driver")
    provider = {
        "package": "blacknode-drivers",
        "component": protocol,
        "adapter": "ros2",
    }
    state_topic = str(driver.get("state_topic") or "/joint_states")
    command_topic = str(driver.get("command_topic") or "/joint_commands")
    bindings = {
        "calibration_control": {
            "kind": "blacknode.robot-capability-binding",
            "schema_version": 1,
            "capability": "calibration_control",
            "label": "Calibration control",
            "provider": {
                "package": "blacknode-drivers",
                "component": protocol,
            },
            "configuration": {},
            "hardware_identity": {},
            "required": True,
        },
        "position_feedback": {
            "kind": "blacknode.robot-capability-binding",
            "schema_version": 1,
            "capability": "position_feedback",
            "label": "Position feedback",
            "provider": copy.deepcopy(provider),
            "configuration": {
                "ros2_interfaces": [{
                    "kind": "topic",
                    "candidates": [state_topic],
                    "required": True,
                    "label": "Joint state",
                    "note": "Start the selected robot driver.",
                }],
            },
            "hardware_identity": {},
            "required": True,
        },
        "joint_group": {
            "kind": "blacknode.robot-capability-binding",
            "schema_version": 1,
            "capability": "joint_group",
            "label": "Joint group",
            "provider": copy.deepcopy(provider),
            "configuration": {
                "ros2_interfaces": [{
                    "kind": "topic",
                    "candidates": [command_topic],
                    "required": True,
                    "label": "Joint command",
                    "note": "Start the selected robot driver.",
                }],
            },
            "hardware_identity": {},
            "required": True,
        },
    }
    result["capabilities"] = list(bindings)
    result["capability_bindings"] = bindings
    return result


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


def _external_motor_calibration(
    profile: dict[str, Any],
    hardware_id: str,
    source: dict[str, Any],
    *,
    source_name: str = "calibration.json",
    safety_margin_deg: float = 3.0,
) -> dict[str, Any]:
    """Convert a six-motor range file into Blacknode's native calibration.

    Older SO-ARM setup tools store the Feetech homing offset and the observed
    position range per motor. The offset is already reflected in the motor's
    Present Position register, where the calibration pose is centred at 2048.
    Blacknode retains the original register values for audit/recovery and uses
    native degree limits at runtime; no external package is imported or needed.
    """
    profile_id = str(profile.get("id") or "").strip()
    hardware_id = str(hardware_id or "").strip()
    if not profile_id:
        raise ValueError("calibration import needs a Robot profile")
    if not hardware_id:
        raise ValueError("calibration import needs a discovered physical hardware ID")
    if not isinstance(source, dict):
        raise ValueError("calibration file must contain a JSON object")

    joints = _joint_list(profile)
    expected = {str(joint.get("id") or ""): joint for joint in joints}
    if set(source) != set(expected):
        missing = sorted(set(expected) - set(source))
        extra = sorted(set(source) - set(expected))
        details = []
        if missing:
            details.append("missing " + ", ".join(missing))
        if extra:
            details.append("unexpected " + ", ".join(extra))
        raise ValueError("calibration joints do not match the Robot profile: " + "; ".join(details))

    margin = max(0.0, float(safety_margin_deg))
    overrides: dict[str, Any] = {}
    retained: dict[str, Any] = {}
    for name, joint in expected.items():
        raw = source.get(name)
        if not isinstance(raw, dict):
            raise ValueError(f"calibration for {name} must be an object")
        required = ("id", "drive_mode", "homing_offset", "range_min", "range_max")
        if any(key not in raw for key in required):
            raise ValueError(f"calibration for {name} is missing motor range fields")
        try:
            servo_id = int(raw["id"])
            drive_mode = int(raw["drive_mode"])
            homing_offset = int(raw["homing_offset"])
            range_min = int(raw["range_min"])
            range_max = int(raw["range_max"])
        except (TypeError, ValueError) as exc:
            raise ValueError(f"calibration for {name} has non-integer motor values") from exc
        if servo_id != int(joint.get("servo_id")):
            raise ValueError(f"calibration servo ID for {name} does not match the Robot profile")
        if drive_mode not in (0, 1):
            raise ValueError(f"calibration drive mode for {name} must be 0 or 1")
        if not -_TICKS_PER_REV <= homing_offset <= _TICKS_PER_REV:
            raise ValueError(f"calibration homing offset for {name} is outside one revolution")
        if not 0 <= range_min < range_max <= _TICKS_PER_REV:
            raise ValueError(f"calibration range for {name} must be within 0..{_TICKS_PER_REV}")

        direction = -1.0 if drive_mode else 1.0
        endpoint_a = direction * (range_min - _DEFAULT_HOME_TICKS) * 360.0 / _TICKS_PER_REV
        endpoint_b = direction * (range_max - _DEFAULT_HOME_TICKS) * 360.0 / _TICKS_PER_REV
        observed_lo, observed_hi = sorted((endpoint_a, endpoint_b))
        base_lo = float(joint.get("safe_min_deg", joint.get("min_deg", observed_lo)))
        base_hi = float(joint.get("safe_max_deg", joint.get("max_deg", observed_hi)))
        safe_lo = max(observed_lo + margin, base_lo)
        safe_hi = min(observed_hi - margin, base_hi)
        if safe_lo >= safe_hi:
            raise ValueError(f"calibration range for {name} is too small after applying safety limits")
        overrides[name] = {
            "home_ticks": _DEFAULT_HOME_TICKS,
            "home_offset_deg": 0.0,
            "observed_min_deg": round(observed_lo, 6),
            "observed_max_deg": round(observed_hi, 6),
            "safe_min_deg": round(safe_lo, 6),
            "safe_max_deg": round(safe_hi, 6),
            "invert": bool(drive_mode),
        }
        retained[name] = {
            "servo_id": servo_id,
            "drive_mode": drive_mode,
            "homing_offset": homing_offset,
            "range_min": range_min,
            "range_max": range_max,
        }

    return {
        "schema_version": _PROFILE_SCHEMA,
        "name": f"Imported {Path(source_name).name}"[:96],
        "profile_id": profile_id,
        "hardware_id": hardware_id,
        "recorded_at": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
        "units": "degrees",
        "safety_margin_deg": margin,
        "samples": 0,
        "source_format": "feetech_motor_ranges_v2",
        "source_name": Path(source_name).name,
        "source_motor_calibration": retained,
        "joints": overrides,
    }


def import_motor_calibration(
    profile: dict[str, Any],
    hardware_id: str,
    source: dict[str, Any],
    *,
    source_name: str = "calibration.json",
    safety_margin_deg: float = 3.0,
) -> tuple[dict[str, Any], Path]:
    """Validate, hardware-bind, and persist an external motor calibration copy."""
    calibration = _external_motor_calibration(
        profile,
        hardware_id,
        source,
        source_name=source_name,
        safety_margin_deg=safety_margin_deg,
    )
    profile_id = str(profile["id"])
    profile_path = _profile_path(profile_id)
    if not profile_path.exists():
        _write_json(profile_path, profile)
    path = _calibration_path(profile_id, hardware_id)
    if path.exists():
        try:
            existing = _read_json(path)
        except (OSError, ValueError, json.JSONDecodeError):
            existing = {}
        if (
            existing.get("profile_id") == calibration.get("profile_id")
            and existing.get("hardware_id") == calibration.get("hardware_id")
            and existing.get("source_motor_calibration")
            == calibration.get("source_motor_calibration")
        ):
            return existing, path
    _write_json(path, calibration)
    return calibration, path


def _normalize_supplied_calibration(
    payload: dict[str, Any],
    profile: dict[str, Any],
    hardware_id: str,
) -> tuple[dict[str, Any], Path | None, bool]:
    """Return native calibration, optional saved path, and whether it was imported."""
    if payload.get("kind") != "blacknode.calibration-import":
        return copy.deepcopy(payload), None, False
    source = payload.get("calibration")
    if not isinstance(source, dict) or not source:
        raise ValueError("selected calibration file does not contain a JSON object")
    if "profile_id" in source or "joints" in source:
        return copy.deepcopy(source), None, False
    calibration, path = import_motor_calibration(
        profile,
        hardware_id,
        source,
        source_name=str(payload.get("source_name") or "calibration.json"),
    )
    return calibration, path, True


def _driver_from_profile(
    profile: dict[str, Any],
    hardware_id: str = "",
    topic_prefix: str = "",
    read_only: bool = False,
) -> dict[str, Any]:
    profile_id = str(profile.get("id") or "robot")
    calibration: dict[str, Any] = {}
    calibration_path: Path | None = None
    if hardware_id:
        candidate = _find_calibration_path(profile_id, hardware_id)
        if candidate is not None:
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
    clean_prefix = str(topic_prefix or "").strip().strip("/")
    prefix = f"/{clean_prefix}" if clean_prefix else ""
    if prefix:
        state_topic = f"{prefix}/joint_states"
        command_topic = f"{prefix}/joint_commands"
        config_topic = f"{prefix}/joint_config"
        control_topic = f"{prefix}/robot_control"
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
            + (" --read-only" if read_only else "")
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
        "topic_prefix": prefix,
        "read_only": bool(read_only),
        "safe_shutdown_watchdog": protocol == "feetech",
        "calibration_path": str(calibration_path or ""),
    }


@node(
    name="RobotJointDefinition", component="contracts",
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


_JOINT_INPUTS = {"joint_1": Dict}


@node(
    name="RobotJointList", component="contracts",
    category=_CATEGORY,
    description="Collect any number of joint definitions; the editor adds another joint socket as the list fills.",
    inputs=_JOINT_INPUTS,
    outputs={"joints": List, "count": Int, "report": Text},
)
def robot_joint_list(ctx: dict) -> dict:
    def sort_key(name: str) -> tuple[int, str]:
        suffix = name.rsplit("_", 1)[-1]
        return (int(suffix), name) if suffix.isdigit() else (999_999, name)

    inputs = sorted((name for name in ctx if name.startswith("joint_")), key=sort_key)
    joints = [dict(ctx[name]) for name in inputs if isinstance(ctx.get(name), dict) and ctx.get(name)]
    return {"joints": joints, "count": len(joints), "report": f"assembled {len(joints)} joint definition(s)"}


@node(
    name="RobotDefinition", component="contracts",
    category=_CATEGORY,
    description="Assemble an editable robot model and executable driver descriptor from ordinary graph inputs.",
    inputs={
        "profile_id": Text(default="my_robot"),
        "display_name": Text(default="My Robot"),
        "protocol": Enum(["feetech", "custom"], default="feetech"),
        "driver_script": Enum(_DRIVER_SCRIPT_CHOICES, default=_DEFAULT_DRIVER_SCRIPT),
        "command_template": Text(default=""),
        "baudrate": Int(default=1_000_000),
        "joints": List,
        "hardware": Dict,
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
    hardware = _hardware_details(ctx)
    manual_vendor_id = _usb_id(ctx.get("vendor_id"))
    manual_product_id = _usb_id(ctx.get("product_id"))
    vendor_id = manual_vendor_id or _usb_id(hardware.get("vendor_id"))
    product_id = manual_product_id or _usb_id(hardware.get("product_id"))
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
            "vendor_id": vendor_id,
            "product_id": product_id,
        },
        "joints": [dict(value) for value in (ctx.get("joints") or []) if isinstance(value, dict)],
    }
    errors = _validate_profile(profile)
    notes = [] if requested_id == profile_id else [f"normalized profile id '{requested_id}' -> '{profile_id}'"]
    report = f"robot definition: {profile['display_name']} ({profile_id}), {len(profile['joints'])} joint(s)"
    if vendor_id and product_id:
        source = "manual override" if manual_vendor_id or manual_product_id else "USB discovery"
        report += f"\nUSB match: {vendor_id}:{product_id} ({source})"
    else:
        report += "\nUSB match: not set; connect RobotUSBDiscovery.recommended to hardware or enter an advanced override"
    report += f"\ndriver: {profile['driver']['script'] or 'custom command template'}"
    if notes:
        report += "\n" + "\n".join(notes)
    if errors:
        report += "\nINVALID:\n- " + "\n- ".join(errors)
    return {"valid": not errors, "profile": profile, "driver": _driver_from_profile(profile), "report": report}


@node(
    name="RobotProfileSave", component="profiles",
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
    name="Robot", component="models",
    category=_CATEGORY,
    description="One easy robot node: select a robot, find its connection, apply calibration, and optionally start its driver.",
    primary_inputs=["trigger"],
    primary_outputs=["robot", "report"],
    inputs={
        "trigger": AnyPort,
        "profile_id": Enum(_available_robot_profile_ids(), default="auto"),
        "profile": Dict,
        "calibration": Dict,
        "calibration_hardware_id": Text(default=""),
        "selection": Int(default=0),
        "hardware": Dict,
        "usb": Dict,
        "driver": Dict,
        "hardware_selection": Int(default=0),
        "hardware_filter": Text(default=""),
        "port_filter": Text(default=""),
        "match_vendor_id": Text(default=""),
        "match_product_id": Text(default=""),
        "probe_open": Bool(default=False),
        "auto_discover": Bool(default=True),
        "action": Enum(["check", "start", "stop"], default="check"),
        "run_id": Text(default="robot_driver"),
        "require_hardware": Bool(default=True),
        "require_usb": Bool(default=True),
        "serial_port": Text(default=""),
        "host": Text(default="127.0.0.1"),
        "port": Int(default=9090),
        "state_topic": Text(default="/joint_states"),
        "command_topic": Text(default="/joint_commands"),
        "config_topic": Text(default="/joint_config"),
        "control_topic": Text(default="/robot_control"),
        "units": Enum(["radians", "degrees"], default="degrees"),
        "topic_prefix": Text(default=""),
        "read_only": Bool(default=False),
        "rate_hz": Float(default=0.0),
    },
    outputs={
        "found": Bool, "ready": Bool, "usb_ready": Bool, "driver_running": Bool,
        "port": Text, "serial": Text, "hardware_id": Text,
        "profile": Dict, "driver": Dict, "robot": Dict,
        "hardware": Dict, "usb": Dict, "devices": List, "recommended": Dict,
        "permissions": Dict, "calibration": Dict, "path": Text, "report": Text,
    },
)
def robot_profile_load(ctx: dict) -> dict:
    supplied_profile = (
        copy.deepcopy(ctx.get("profile"))
        if isinstance(ctx.get("profile"), dict) and ctx.get("profile")
        else None
    )
    supplied_hardware = ctx.get("hardware") if isinstance(ctx.get("hardware"), dict) else ctx.get("usb")
    hardware = dict(supplied_hardware or {}) if isinstance(supplied_hardware, dict) else {}
    calibration_hardware_id = str(ctx.get("calibration_hardware_id") or "").strip()
    discovery_report = ""
    devices: list[dict[str, Any]] = []
    if not hardware and bool(ctx.get("auto_discover", True)):
        from .robot import robot_usb_discovery

        discovered = robot_usb_discovery({
            "port_filter": ctx.get("hardware_filter") or ctx.get("port_filter", ""),
            "match_vendor_id": ctx.get("match_vendor_id", ""),
            "match_product_id": ctx.get("match_product_id", ""),
            "probe_open": bool(ctx.get("probe_open", False)),
        })
        devices = [dict(item) for item in discovered.get("devices", []) if isinstance(item, dict)]
        legacy_selection = ctx.get("hardware_selection")
        selection = int(legacy_selection if legacy_selection not in (None, "", 0) else ctx.get("selection") or 0)
        selection_source = "device index"
        if calibration_hardware_id:
            matching_indexes = [
                index
                for index, device in enumerate(devices)
                if _hardware_matches_id(device, calibration_hardware_id)
            ]
            selection = matching_indexes[0] if len(matching_indexes) == 1 else -1
            selection_source = "calibration hardware ID"
        recommended = devices[selection] if 0 <= selection < len(devices) else {}
        selected_report = (
            f"selected_index: {selection}\n"
            f"selection_source: {selection_source}\n"
            f"selected_port: {recommended.get('path') or 'not available'}\n"
            f"selected_serial: {recommended.get('serial') or 'not available'}"
        )
        discovery_report = str(discovered.get("report") or "") + "\n" + selected_report
        hardware = {
            **discovered,
            "port": str(recommended.get("path") or ""),
            "serial": str(recommended.get("serial") or ""),
            "recommended": recommended,
            "report": discovery_report,
        }
    elif hardware:
        devices = [dict(item) for item in hardware.get("devices", []) if isinstance(item, dict)]
        matching_devices = [
            device
            for device in devices
            if calibration_hardware_id and _hardware_matches_id(device, calibration_hardware_id)
        ]
        if len(matching_devices) == 1:
            recommended = matching_devices[0]
            hardware = {
                **hardware,
                "port": str(recommended.get("path") or ""),
                "serial": str(
                    recommended.get("serial")
                    or recommended.get("serial_number")
                    or ""
                ),
                "recommended": recommended,
            }

    requested_profile = str(
        (supplied_profile or {}).get("id")
        or ctx.get("profile_id")
        or "auto"
    ).strip()
    auto_reason = ""
    if supplied_profile is not None:
        profile = supplied_profile
        profile_id = _slug(profile.get("id") or "robot")
        path = None
    elif requested_profile.lower() == "auto":
        profile, path, auto_reason = _auto_profile_for_hardware(hardware)
        profile_id = _slug((profile or {}).get("id") or "auto")
    else:
        profile_id = _slug(requested_profile)
        profile, path = load_profile(profile_id)
    if profile is None:
        known = ", ".join(item["id"] for item in list_profiles()) or "none"
        reason = auto_reason or f"profile '{profile_id}' was not found"
        return {
            "found": False, "ready": False, "usb_ready": bool(hardware.get("ready")),
            "driver_running": False, "profile": {}, "driver": {}, "robot": {},
            "hardware": hardware, "usb": hardware, "devices": devices,
            "recommended": dict(hardware.get("recommended") or {}),
            "permissions": dict(hardware.get("permissions") or {}),
            "calibration": {}, "path": "",
            "report": (
                f"robot profile selection needs attention: {reason}"
                f"\navailable profiles: {known}"
                + (f"\n{discovery_report}" if discovery_report else "")
            ),
        }

    effective_ctx = {**ctx, "hardware": hardware}
    hardware_id = _hardware_id(effective_ctx)
    if (
        calibration_hardware_id
        and _normalized_hardware_id(calibration_hardware_id)
        != _normalized_hardware_id(hardware_id)
    ):
        return {
            "found": False, "ready": False, "usb_ready": bool(hardware.get("ready")),
            "driver_running": False, "profile": {}, "driver": {}, "robot": {},
            "hardware": hardware, "usb": hardware, "devices": devices,
            "recommended": dict(hardware.get("recommended") or {}),
            "permissions": dict(hardware.get("permissions") or {}),
            "calibration": {}, "path": str(path or "builtin"),
            "report": (
                f"selected calibration belongs to {calibration_hardware_id}, "
                f"but discovery selected {hardware_id or 'no hardware'}\n"
                "Choose the calibration for this physical Robot instance or "
                "select the matching connected robot."
            ),
        }
    supplied_calibration = (
        copy.deepcopy(ctx.get("calibration"))
        if isinstance(ctx.get("calibration"), dict) and ctx.get("calibration")
        else None
    )
    calibration_import_requested = bool(
        supplied_calibration
        and supplied_calibration.get("kind") == "blacknode.calibration-import"
    )
    imported_calibration_path: Path | None = None
    calibration_was_imported = False
    if supplied_calibration is not None:
        try:
            (
                supplied_calibration,
                imported_calibration_path,
                calibration_was_imported,
            ) = _normalize_supplied_calibration(
                supplied_calibration,
                profile,
                hardware_id,
            )
        except (TypeError, ValueError) as exc:
            return {
                "found": False, "ready": False, "profile": {}, "driver": {},
                "robot": {}, "hardware": hardware, "devices": devices,
                "calibration": {}, "path": "",
                "report": f"calibration import blocked: {exc}",
            }
        calibration_profile_id = str(
            supplied_calibration.get("profile_id") or ""
        ).strip()
        calibration_hardware_id = str(
            supplied_calibration.get("hardware_id") or ""
        ).strip()
        profile_joint_ids = {
            str(joint.get("id") or "").strip()
            for joint in _joint_list(profile)
        }
        calibration_joint_ids = set(
            (supplied_calibration.get("joints") or {}).keys()
            if isinstance(supplied_calibration.get("joints"), dict)
            else []
        )
        if calibration_profile_id != str(profile.get("id") or ""):
            return {
                "found": False, "ready": False, "profile": {}, "driver": {},
                "robot": {}, "hardware": hardware, "devices": devices,
                "calibration": {}, "path": "",
                "report": "embedded calibration profile does not match the Robot profile",
            }
        if not hardware_id or calibration_hardware_id != hardware_id:
            return {
                "found": False, "ready": False, "profile": {}, "driver": {},
                "robot": {}, "hardware": hardware, "devices": devices,
                "calibration": {}, "path": "",
                "report": (
                    "embedded calibration is bound to "
                    f"{calibration_hardware_id or 'an unknown device'}, "
                    f"but discovery selected {hardware_id or 'no hardware'}"
                ),
            }
        if profile_joint_ids != calibration_joint_ids:
            return {
                "found": False, "ready": False, "profile": {}, "driver": {},
                "robot": {}, "hardware": hardware, "devices": devices,
                "calibration": {}, "path": "",
                "report": "embedded calibration joints do not match the Robot profile",
            }
        if calibration_import_requested and imported_calibration_path is None:
            profile_path = _profile_path(str(profile["id"]))
            if not profile_path.exists():
                _write_json(profile_path, profile)
            imported_calibration_path = _calibration_path(
                str(profile["id"]), hardware_id
            )
            _write_json(imported_calibration_path, supplied_calibration)
            calibration_was_imported = True
        effective_profile = _apply_calibration(profile, supplied_calibration)
    else:
        effective_profile = copy.deepcopy(profile)
    rate_override = float(ctx.get("rate_hz") or 0.0)
    if rate_override > 0:
        effective_profile.setdefault("driver", {})["rate_hz"] = rate_override
    supplied_driver = ctx.get("driver") if isinstance(ctx.get("driver"), dict) else {}
    if supplied_driver:
        driver = dict(supplied_driver)
        if supplied_calibration is not None:
            driver["profile"] = copy.deepcopy(effective_profile)
            driver["joints"] = _joint_list(effective_profile)
            driver["hardware_id"] = hardware_id
            driver["calibration_path"] = str(imported_calibration_path or "")
    else:
        driver = _driver_from_profile(
            effective_profile,
            "" if supplied_calibration is not None else hardware_id,
            str(ctx.get("topic_prefix") or ""),
            bool(ctx.get("read_only", False)),
        )
        if supplied_calibration is not None:
            # A deployment embeds the selected calibration because the target
            # does not share the editor's local calibration files. Preserve
            # the separately verified physical identity on the driver
            # contract so downstream safety gates can bind commands to this
            # exact robot.
            driver["hardware_id"] = hardware_id
            driver["calibration_path"] = str(imported_calibration_path or "")
    effective_profile = copy.deepcopy(driver.get("profile") or effective_profile)
    effective_profile = _profile_with_default_capabilities(effective_profile, driver)
    driver["profile"] = copy.deepcopy(effective_profile)
    effective = dict(driver.get("profile") or profile)
    from .robot import robot_discovery

    connection = robot_discovery({
        "driver": driver,
        "usb": hardware,
        "action": ctx.get("action", "check"),
        "run_id": ctx.get("run_id", "robot_driver"),
        "require_usb": bool(ctx.get("require_hardware") if "require_hardware" in ctx else ctx.get("require_usb", True)),
        "serial_port": ctx.get("serial_port", ""),
        "host": ctx.get("host", "127.0.0.1"),
        "port": ctx.get("port", 9090),
        "state_topic": ctx.get("state_topic", "/joint_states"),
        "command_topic": ctx.get("command_topic", "/joint_commands"),
        "config_topic": ctx.get("config_topic", "/joint_config"),
        "control_topic": ctx.get("control_topic", "/robot_control"),
        "units": ctx.get("units", "degrees"),
    })
    recommended = hardware.get("recommended") if isinstance(hardware.get("recommended"), dict) else {}
    return {
        "found": True,
        "ready": bool(connection.get("ready")),
        "usb_ready": bool(connection.get("usb_ready")),
        "driver_running": bool(connection.get("driver_running")),
        "port": str(recommended.get("path") or ""),
        "serial": str(recommended.get("serial") or ""),
        "hardware_id": hardware_id,
        "profile": effective,
        "driver": driver,
        "robot": dict(connection.get("robot") or {}),
        "hardware": hardware,
        "usb": hardware,
        "devices": devices,
        "recommended": recommended,
        "permissions": dict(hardware.get("permissions") or {}),
        "calibration": dict(effective.get("calibration") or {}),
        "path": str(path or "builtin"),
        "report": (
            f"loaded robot profile '{profile_id}' ({len(_joint_list(effective))} joint(s))"
            + (f"\nautomatic profile match: {auto_reason}" if auto_reason else "")
            + (
                f"\ncalibration: imported and saved to {imported_calibration_path}"
                if calibration_was_imported and imported_calibration_path is not None
                else "\ncalibration: embedded deployment calibration"
                if supplied_calibration is not None
                else f"\ncalibration: {driver['calibration_path']}"
                if driver.get("calibration_path")
                else "\ncalibration: none"
            )
            + (f"\nhardware: {hardware_id}" if hardware_id else "\nhardware: not connected")
            + (f"\n{discovery_report}" if discovery_report else "")
            + f"\n{connection.get('report', '')}"
        ),
    }


@node(
    name="RobotProfileLoad", component="profiles",
    category=_CATEGORY,
    hidden=True,
    description="Compatibility alias for Robot. New workflows should use the generic Robot node.",
    inputs={
        "profile_id": Enum(_available_profile_ids(), default="so_arm101"),
        "hardware_id": Text(default=""),
        "hardware": Dict,
        "topic_prefix": Text(default=""),
        "rate_hz": Float(default=0.0),
    },
    outputs={"found": Bool, "profile": Dict, "driver": Dict, "calibration": Dict, "path": Text, "report": Text},
)
def robot_profile_load_compat(ctx: dict) -> dict:
    return robot_profile_load(ctx)


@node(
    name="RobotProfileList", component="profiles",
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
    name="RobotProfileDuplicate", component="profiles",
    category=_CATEGORY,
    description="Duplicate a built-in or saved profile under a new editable lowercase id.",
    inputs={
        "source_profile_id": Enum(_available_profile_ids(), default="so_arm101"),
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
    previous_pose = dict(session.get("last_pose") or {})
    now = time.time()
    movement: list[tuple[float, str]] = []
    for name, raw in pose.items():
        if name not in allowed or not isinstance(raw, (int, float)):
            continue
        value = float(raw)
        previous_value = previous_pose.get(name)
        if isinstance(previous_value, (int, float)):
            movement.append((abs(value - float(previous_value)), name))
        is_new = name not in session["observed"]
        bounds = session["observed"].setdefault(name, {"min_deg": value, "max_deg": value})
        old_min = float(bounds["min_deg"])
        old_max = float(bounds["max_deg"])
        bounds["min_deg"] = min(float(bounds["min_deg"]), value)
        bounds["max_deg"] = max(float(bounds["max_deg"]), value)
        update_kind = "both" if is_new else "min" if value < old_min else "max" if value > old_max else ""
        if update_kind:
            session.setdefault("range_updates", {})[name] = {
                "kind": update_kind,
                "value": value,
                "at": now,
            }
        accepted += 1
    if accepted:
        session["samples"] += 1
        session["last_pose"] = {str(k): float(v) for k, v in pose.items() if k in allowed and isinstance(v, (int, float))}
        if movement:
            delta, moved_joint = max(movement)
            if delta >= 0.05:
                session["capturing_joint"] = moved_joint
                session["capturing_at"] = now
        session["updated_at"] = now
    return accepted


def _calibration_context_session(ctx: dict[str, Any]) -> dict[str, Any] | None:
    profile = (
        copy.deepcopy(ctx.get("profile"))
        if isinstance(ctx.get("profile"), dict) and ctx.get("profile")
        else {}
    )
    if not profile:
        return None
    allowed = {str(joint.get("id")) for joint in _joint_list(profile)}
    pose = {
        str(name): float(value)
        for name, value in dict(ctx.get("pose") or {}).items()
        if name in allowed and isinstance(value, (int, float))
    }
    return {
        "profile": profile,
        "hardware_id": _hardware_id(ctx),
        "last_pose": pose,
        "observed": {},
        "home": {},
        "range_updates": {},
        "samples": 0,
        "active": False,
        "paused": False,
        "monitoring": False,
    }


def _calibration_dashboard(session: dict[str, Any] | None, report: str) -> str:
    active = bool(session and session.get("active"))
    paused = bool(session and session.get("paused"))
    accent = "#f59e0b" if active else "#60a5fa" if paused else "#22c55e"
    rows = []
    observed = dict(session.get("observed") or {}) if session else {}
    home = dict(session.get("home") or {}) if session else {}
    current = dict(session.get("last_pose") or {}) if session else {}
    range_updates = dict(session.get("range_updates") or {}) if session else {}
    now = time.time()
    capturing_joint = str(session.get("capturing_joint") or "") if session else ""
    capturing_recent = bool(
        active
        and capturing_joint
        and now - float(session.get("capturing_at") or 0.0) <= 1.25
    ) if session else False
    profile = dict(session.get("profile") or {}) if session else {}
    profile_joints = [
        str(joint.get("id"))
        for joint in _joint_list(profile)
        if joint.get("id")
    ]
    joint_names = list(dict.fromkeys([*profile_joints, *sorted(observed)]))
    for index, name in enumerate(joint_names):
        bounds = observed.get(name)
        y = 178 + index * 44
        update = range_updates.get(name) if isinstance(range_updates.get(name), dict) else {}
        update_recent = bool(update and now - float(update.get("at") or 0.0) <= 1.5)
        moving = capturing_recent and name == capturing_joint
        row_fill = "#78350f" if update_recent else "#172554" if moving else "transparent"
        current_fill = "#60a5fa" if moving else "#f8fafc"
        range_fill = "#fbbf24" if update_recent else "#93a4b8" if bounds else "#64748b"
        update_kind = str(update.get("kind") or "").upper()
        update_badge = "RANGE" if update_kind == "BOTH" else f"{update_kind} {'↓' if update_kind == 'MIN' else '↑'}" if update_kind else ""
        row_prefix = f'<rect x="28" y="{y - 25}" width="624" height="34" rx="7" fill="{row_fill}" opacity="0.72"/>'
        current_text = (
            f"{float(current[name]):.2f}"
            if isinstance(current.get(name), (int, float))
            else "-"
        )
        range_text = (
            f"{float(bounds['min_deg']):.2f} .. {float(bounds['max_deg']):.2f}"
            if bounds
            else "not observed"
        )
        home_text = (
            f"{float(home[name]):.2f}"
            if isinstance(home.get(name), (int, float))
            else "-"
        )
        home_fill = accent if name in home else "#64748b"
        rows.append(
            row_prefix
            + f'<text x="46" y="{y}" fill="#f8fafc" font-family="monospace" font-size="15">{html.escape(name)}</text>'
            f'<text x="270" y="{y}" text-anchor="end" fill="{current_fill}" font-family="monospace" font-size="15">{current_text}</text>'
            f'<text x="500" y="{y}" text-anchor="end" fill="{range_fill}" font-family="monospace" font-size="15">{range_text}</text>'
            f'<text x="510" y="{y}" fill="{range_fill}" font-family="Arial" font-size="10" font-weight="800">{update_badge if update_recent else ""}</text>'
            f'<text x="640" y="{y}" text-anchor="end" fill="{home_fill}" font-family="monospace" font-size="15">{home_text}</text>'
        )
    if not rows:
        rows.append('<text x="340" y="220" text-anchor="middle" fill="#93a4b8" font-family="Arial" font-size="17">Connect a robot profile to list its joints.</text>')
    state = "RECORDING" if active else "PAUSED" if paused else "IDLE / SAVED"
    samples = int(session.get("samples") or 0) if session else 0
    safe_report = html.escape(report[:100])
    capture_label = html.escape(f"CAPTURING {capturing_joint}" if capturing_recent else "")
    footer_y = max(300, 178 + max(0, len(joint_names) - 1) * 44 + 70)
    height = footer_y + 38
    hardware_id = html.escape(str(session.get("hardware_id") or "not connected")) if session else "not connected"
    svg = f'''<svg xmlns="http://www.w3.org/2000/svg" width="680" height="{height}" viewBox="0 0 680 {height}">
<rect width="680" height="{height}" rx="22" fill="#0b1020"/>
<rect x="22" y="22" width="636" height="100" rx="16" fill="#172033" stroke="{accent}" stroke-width="2"/>
<text x="44" y="58" fill="#f8fafc" font-family="Arial" font-size="23" font-weight="800">ROBOT CALIBRATION</text>
<text x="44" y="91" fill="{accent}" font-family="Arial" font-size="16" font-weight="800">{state} · {samples} SAMPLES</text>
<text x="636" y="58" text-anchor="end" fill="#93a4b8" font-family="monospace" font-size="11">{hardware_id}</text>
<text x="636" y="91" text-anchor="end" fill="#60a5fa" font-family="Arial" font-size="13" font-weight="800">{capture_label}</text>
<text x="46" y="146" fill="#93a4b8" font-family="Arial" font-size="12">JOINT</text>
<text x="270" y="146" text-anchor="end" fill="#93a4b8" font-family="Arial" font-size="12">CURRENT</text>
<text x="500" y="146" text-anchor="end" fill="#93a4b8" font-family="Arial" font-size="12">OBSERVED RANGE</text>
<text x="640" y="146" text-anchor="end" fill="#93a4b8" font-family="Arial" font-size="12">HOME</text>
{''.join(rows)}
<text x="40" y="{footer_y}" fill="#93a4b8" font-family="Arial" font-size="13">{safe_report}</text>
</svg>'''
    return "data:image/svg+xml;base64," + base64.b64encode(svg.encode("utf-8")).decode("ascii")


def _session_outputs(session: dict[str, Any] | None, report: str, *, saved: bool = False, path: str = "") -> dict[str, Any]:
    profile = copy.deepcopy(session.get("effective_profile") or session.get("profile") or {}) if session else {}
    calibration = copy.deepcopy(session.get("calibration") or {}) if session else {}
    hardware_id = str(session.get("hardware_id") or "") if session else ""
    active = bool(session and session.get("active"))
    paused = bool(session and session.get("paused"))
    state = "recording" if active else "paused" if paused else "saved" if calibration else "idle"
    saved_path = path or (str(session.get("path") or "") if session else "")
    capturing_joint = ""
    if session and active and time.time() - float(session.get("capturing_at") or 0.0) <= 1.25:
        capturing_joint = str(session.get("capturing_joint") or "")
    return {
        "live": bool(session and session.get("monitoring")),
        "state": state,
        "active": active,
        "data_ready": bool(session and session.get("observed")),
        "samples": int(session.get("samples") or 0) if session else 0,
        "pose": copy.deepcopy(session.get("last_pose") or {}) if session else {},
        "capturing_joint": capturing_joint,
        "range_updates": copy.deepcopy(session.get("range_updates") or {}) if session else {},
        "observed": copy.deepcopy(session.get("observed") or {}) if session else {},
        "home": copy.deepcopy(session.get("home") or {}) if session else {},
        "calibration": calibration,
        "profile": profile,
        "driver": _driver_from_profile(profile, hardware_id) if profile else {},
        "hardware_id": hardware_id,
        "saved": saved or bool(calibration),
        "path": saved_path,
        "dashboard": _calibration_dashboard(session, report),
        "report": report,
    }


@node(
    name="RobotCalibrationRecorder", component="calibration",
    category=_CATEGORY,
    live=True,
    description="Record released-joint extrema and home pose, review a safety margin, and save calibration per physical robot.",
    inputs={
        "action": Enum(["check", "start", "pause", "capture_home", "finish", "cancel"], default="check"),
        "run_id": Text(default="robot_calibration"),
        "calibration_name": Text(default=""),
        "profile": Dict,
        "hardware_id": Text(default=""),
        "hardware": Dict,
        "pose": Dict,
        "capturing_joint": Text,
        "range_updates": Dict,
        "torque_enabled": Bool(default=True),
        "require_released": Bool(default=True),
        "safety_margin_deg": Float(default=3.0),
    },
    outputs={
        "live": Bool,
        "state": Text,
        "active": Bool,
        "data_ready": Bool,
        "samples": Int,
        "pose": Dict,
        "observed": Dict,
        "home": Dict,
        "calibration": Dict,
        "profile": Dict,
        "driver": Dict,
        "hardware_id": Text,
        "saved": Bool,
        "path": Text,
        "dashboard": Image,
        "report": Text,
    },
)
def robot_calibration_recorder(ctx: dict) -> dict:
    action = str(ctx.get("action") or "check").strip().lower()
    run_id = str(ctx.get("run_id") or "robot_calibration").strip() or "robot_calibration"
    requested_name = " ".join(str(ctx.get("calibration_name") or "").split())[:96]
    pose = dict(ctx.get("pose") or {})
    raw_torque = ctx.get("torque_enabled", True)
    torque_enabled = raw_torque if isinstance(raw_torque, bool) else None
    require_released = bool(ctx.get("require_released", True))
    with _calibration_lock:
        session = _calibration_sessions.get(run_id)
        if session is not None and requested_name:
            session["calibration_name"] = requested_name
        if action == "start":
            profile = copy.deepcopy(ctx.get("profile") if isinstance(ctx.get("profile"), dict) else {})
            preview = _calibration_context_session(ctx)
            errors = _validate_profile(profile)
            hardware_id = _hardware_id(ctx)
            if errors:
                return _session_outputs(preview, "calibration blocked: invalid robot profile: " + "; ".join(errors))
            if not hardware_id:
                return _session_outputs(
                    preview,
                    (
                        "calibration blocked: no physical hardware identity was "
                        "received. Re-run Robot discovery and select a connected "
                        "device; calibration cannot use a generic shared ID"
                    ),
                )
            if require_released and torque_enabled is not False:
                return _session_outputs(preview, "calibration blocked: torque is on. Support the arm and use Release + live pose first.")
            if (
                session is not None
                and session.get("paused")
                and not session.get("calibration")
                and str(session.get("hardware_id")) == hardware_id
                and str(session.get("profile", {}).get("id")) == str(profile.get("id"))
            ):
                session["active"] = True
                session["paused"] = False
                session["margin"] = max(0.0, float(ctx.get("safety_margin_deg") or session.get("margin") or 0.0))
                _sample_session(session, pose)
                return _session_outputs(session, "RECORDING RESUMED: continue moving each released joint through its intended usable range.")
            session = {
                "run_id": run_id,
                "profile": profile,
                "hardware_id": hardware_id,
                "calibration_name": (
                    requested_name
                    or f"{str(profile.get('display_name') or profile.get('id') or 'Robot')} · {hardware_id}"
                ),
                "observed": {},
                "home": {},
                "samples": 0,
                "active": True,
                "paused": False,
                "monitoring": True,
                "margin": max(0.0, float(ctx.get("safety_margin_deg") or 0.0)),
                "started_at": time.time(),
                "updated_at": time.time(),
            }
            _calibration_sessions[run_id] = session
            _sample_session(session, pose)
            return _session_outputs(session, "RECORDING: torque is off. Support the arm and slowly sweep each joint through the intended usable range.")
        if session is None:
            return _session_outputs(
                _calibration_context_session(ctx),
                "calibration is idle. Release torque, then press Start recording.",
            )
        if action in {"_sample", "sample"}:
            if require_released and torque_enabled is not False:
                session["active"] = False
                session["paused"] = True
                allowed = {str(joint.get("id")) for joint in _joint_list(session["profile"])}
                current = {
                    str(name): float(value)
                    for name, value in pose.items()
                    if name in allowed and isinstance(value, (int, float))
                }
                if current:
                    session["last_pose"] = current
                    session["updated_at"] = time.time()
                return _session_outputs(session, "calibration paused: torque is not verified released; restore complete feedback before recording more samples")
            if session.get("active"):
                _sample_session(session, pose)
                return _session_outputs(session, "RECORDING: live ranges are updating; capture Home when the robot is in its neutral pose.")
            allowed = {str(joint.get("id")) for joint in _joint_list(session["profile"])}
            current = {str(name): float(value) for name, value in pose.items() if name in allowed and isinstance(value, (int, float))}
            if current:
                session["last_pose"] = current
                session["updated_at"] = time.time()
            if session.get("calibration"):
                return _session_outputs(session, "SAVED: calibration is stored; current pose remains live until the graph stops.")
            return _session_outputs(session, "PAUSED: live pose is still visible, but range samples are not being recorded.")
        if action == "pause":
            session["active"] = False
            session["paused"] = True
            return _session_outputs(session, "PAUSED: recording stopped without discarding samples. Press Resume recording or save when ready.")
        if action == "capture_home":
            if require_released and torque_enabled is not False:
                return _session_outputs(session, "home not captured: torque is not verified released")
            _sample_session(session, pose)
            allowed = {str(joint.get("id")) for joint in _joint_list(session["profile"])}
            session["home"] = {name: float(value) for name, value in pose.items() if name in allowed and isinstance(value, (int, float))}
            return _session_outputs(session, f"captured neutral Home for {len(session['home'])} joint(s); continue sweeping or press Save calibration")
        if action == "cancel":
            session["active"] = False
            session["paused"] = False
            session["monitoring"] = False
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
                "name": str(session.get("calibration_name") or session["hardware_id"]),
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
            session["path"] = str(path)
            session["active"] = False
            session["paused"] = False
            return _session_outputs(
                session,
                (
                    f"saved calibration '{calibration['name']}' "
                    f"for {session['hardware_id']}\n{path}"
                ),
                saved=True,
                path=str(path),
            )
        if session.get("active"):
            report = "calibration recording is active and receiving live joint samples"
        elif session.get("paused"):
            report = "calibration recording is paused; live pose remains connected"
        else:
            report = "calibration recording is complete"
        return _session_outputs(session, report)


def calibration_runtime_status() -> dict[str, Any]:
    with _calibration_lock:
        sessions = [
            {
                "run_id": run_id,
                "kind": "robot_calibration",
                "active": bool(session.get("monitoring")),
                "recording": bool(session.get("active")),
                "state": "recording" if session.get("active") else "paused" if session.get("paused") else "saved" if session.get("calibration") else "idle",
                "samples": int(session.get("samples") or 0),
                "hardware_id": str(session.get("hardware_id") or ""),
                "calibration_name": str(session.get("calibration_name") or ""),
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
