"""Connected-device inspection nodes."""

from __future__ import annotations

import math
import re
import time
from typing import Any as TypingAny

from blacknode.node import Any, Bool, Dict, Enum, Float, Int, List, Text, node


_CATEGORY = "Robot"


@node(
    name="HardwareCapabilities",
    component="devices",
    category=_CATEGORY,
    description="Report the connected-device capabilities requested by a robot profile.",
    inputs={
        "device_id": Text(default="device"),
        "capabilities": List(default=["mobile_base"]),
        "refresh": Any,
    },
    outputs={"available": Bool, "device_id": Text, "capabilities": List, "state": Dict, "report": Text},
)
def hardware_capabilities(ctx: dict) -> dict:
    device_id = str(ctx.get("device_id") or "device")
    capabilities = [str(value) for value in (ctx.get("capabilities") or [])]
    state = {
        "device_id": device_id,
        "connected": False,
        "armed": False,
        "capabilities": capabilities,
        "provider": "unconfigured",
    }
    return {
        "available": False,
        "device_id": device_id,
        "capabilities": capabilities,
        "state": state,
        "report": "no hardware adapter configured; select an adapter in the device profile",
    }


def _servo_id_from_name(name: str) -> int | None:
    match = re.fullmatch(r"(?:servo|joint)_(\d+)", str(name or "").strip())
    return int(match.group(1)) if match else None


def _profile_joint(robot: dict[str, TypingAny], servo_id: int) -> dict[str, TypingAny]:
    driver = robot.get("driver") if isinstance(robot.get("driver"), dict) else {}
    profile = robot.get("profile") if isinstance(robot.get("profile"), dict) else {}
    if not profile and isinstance(driver.get("profile"), dict):
        profile = driver["profile"]
    for joint in profile.get("joints") or driver.get("joints") or []:
        if (
            isinstance(joint, dict)
            and int(joint.get("servo_id") or 0) == servo_id
        ):
            return dict(joint)
    return {}


def _display_value(value: TypingAny, source_units: str, units: str) -> float | None:
    try:
        number = float(value)
    except (TypeError, ValueError):
        return None
    if not math.isfinite(number):
        return None
    source = str(source_units or "degrees").lower()
    target = str(units or "degrees").lower()
    if source.startswith("radian") and target.startswith("degree"):
        return math.degrees(number)
    if source.startswith("degree") and target.startswith("radian"):
        return math.radians(number)
    return number


def _servo_payload(
    state: dict[str, TypingAny],
    robot: dict[str, TypingAny],
    servo_id: int,
    requested_joint: str,
    units: str,
) -> dict[str, TypingAny]:
    sample = state if isinstance(state, dict) else {}
    payload = (
        sample.get("payload")
        if isinstance(sample.get("payload"), dict)
        else sample
    )
    payload = payload if isinstance(payload, dict) else {}
    profile_joint = _profile_joint(robot, servo_id)
    profile_name = str(profile_joint.get("id") or "").strip()
    preferred_name = requested_joint or profile_name or f"servo_{servo_id}"
    source_units = str(payload.get("position_unit") or "degrees")
    velocity_units = str(payload.get("velocity_unit") or f"{source_units}/s")

    joints = [
        item for item in (payload.get("joints") or [])
        if isinstance(item, dict)
    ]
    selected = next(
        (
            item for item in joints
            if int(item.get("servo_id") or 0) == servo_id
            or str(item.get("name") or "") == preferred_name
            or _servo_id_from_name(str(item.get("name") or "")) == servo_id
        ),
        None,
    )

    if selected is None and payload.get("kind") == "blacknode.device-state":
        joint_state = (
            payload.get("joint_state")
            if isinstance(payload.get("joint_state"), dict)
            else {}
        )
        positions = dict(joint_state.get("positions") or {})
        values = (
            payload.get("values")
            if isinstance(payload.get("values"), dict)
            else {}
        )
        servo_ids = dict(values.get("servo_ids") or {})
        raw_positions = dict(values.get("raw_positions") or {})
        name = next(
            (
                candidate for candidate in positions
                if candidate == preferred_name
                or servo_ids.get(candidate) == servo_id
                or _servo_id_from_name(candidate) == servo_id
            ),
            "",
        )
        if name:
            raw_limits = dict(joint_state.get("limits") or {}).get(name)
            selected = {
                "name": name,
                "servo_id": servo_id,
                "position": positions[name],
                "velocity": dict(joint_state.get("velocities") or {}).get(name, 0.0),
                "effort": dict(joint_state.get("efforts") or {}).get(name),
                "raw_position": raw_positions.get(name),
                "lower_limit": (
                    raw_limits.get("lower")
                    if isinstance(raw_limits, dict)
                    else None
                ),
                "upper_limit": (
                    raw_limits.get("upper")
                    if isinstance(raw_limits, dict)
                    else None
                ),
            }
            source_units = str(joint_state.get("position_unit") or "radian")
            velocity_units = str(joint_state.get("velocity_unit") or "radian/s")

    selected = dict(selected or {})
    joint_name = str(selected.get("name") or preferred_name)
    position = _display_value(selected.get("position"), source_units, units)
    velocity = _display_value(selected.get("velocity", 0.0), velocity_units, f"{units}/s")
    lower = _display_value(selected.get("lower_limit"), source_units, units)
    upper = _display_value(selected.get("upper_limit"), source_units, units)
    if lower is None:
        lower = _display_value(
            profile_joint.get("safe_min_deg", profile_joint.get("min_deg")),
            "degrees",
            units,
        )
    if upper is None:
        upper = _display_value(
            profile_joint.get("safe_max_deg", profile_joint.get("max_deg")),
            "degrees",
            units,
        )

    temperatures = dict(payload.get("temperatures_c") or {})
    temperature = temperatures.get(joint_name)
    if temperature is None:
        temperature = temperatures.get(f"servo_{servo_id}")
    faults = [
        dict(item) for item in (payload.get("faults") or [])
        if isinstance(item, dict)
    ]
    calibration = (
        payload.get("calibration")
        if isinstance(payload.get("calibration"), dict)
        else robot.get("calibration")
        if isinstance(robot.get("calibration"), dict)
        else {}
    )
    calibrated = bool(
        payload.get("calibrated")
        if payload.get("calibrated") is not None
        else calibration
    )
    return {
        "available": selected != {},
        "joint_name": joint_name,
        "servo_id": servo_id,
        "position": position,
        "velocity": velocity,
        "effort": selected.get("effort"),
        "raw_position": selected.get("raw_position"),
        "lower_limit": lower,
        "upper_limit": upper,
        "temperature_c": temperature,
        "voltage_v": payload.get("voltage_v"),
        "faults": faults,
        "connected": bool(payload.get("connected")),
        "armed": bool(payload.get("armed")),
        "torque_enabled": payload.get("torque_enabled"),
        "calibrated": calibrated,
        "calibration": dict(calibration),
        "stale": bool(sample.get("stale", payload.get("stale", False))),
        "units": units,
        "source": str(sample.get("source") or payload.get("source") or ""),
        "updated_at": sample.get("received_at") or payload.get("updated_at"),
    }


@node(
    name="RobotServo",
    component="devices",
    category=_CATEGORY,
    description=(
        "Inspect one servo from a Robot monitor target. Connect several Servo "
        "nodes to one Robot for live debugging. Its slider creates a canonical "
        "command request; connect joint and target_position to a motion node to execute it."
    ),
    primary_inputs=["robot", "state"],
    primary_outputs=["servo", "joint", "target_position", "command"],
    inputs={
        "robot": Dict(default={}),
        "state": Dict(default={}),
        "servo_id": Int(default=1),
        "joint_name": Text(default=""),
        "target_position": Float(default=0.0),
        "follow_feedback": Bool(default=True),
        "units": Enum(["degrees", "radians"], default="degrees"),
    },
    outputs={
        "servo": Dict,
        "available": Bool,
        "joint": Text,
        "servo_id": Int,
        "position": Float,
        "velocity": Float,
        "raw_position": Int,
        "limits": Dict,
        "calibrated": Bool,
        "temperature_c": Float,
        "voltage_v": Float,
        "faults": List,
        "target_position": Float,
        "command": Dict,
        "report": Text,
    },
)
def robot_servo(ctx: dict) -> dict:
    robot = ctx.get("robot") if isinstance(ctx.get("robot"), dict) else {}
    state = ctx.get("state") if isinstance(ctx.get("state"), dict) else {}
    servo_id = max(1, min(253, int(ctx.get("servo_id") or 1)))
    units = str(ctx.get("units") or "degrees")
    servo = _servo_payload(
        state,
        robot,
        servo_id,
        str(ctx.get("joint_name") or "").strip(),
        units,
    )
    position = (
        float(servo["position"])
        if servo.get("position") is not None
        else 0.0
    )
    target = (
        position
        if bool(ctx.get("follow_feedback", True)) and servo["available"]
        else float(ctx.get("target_position") or 0.0)
    )
    lower = servo.get("lower_limit")
    upper = servo.get("upper_limit")
    if lower is not None and upper is not None:
        target = min(float(upper), max(float(lower), target))
    target_rad = math.radians(target) if units == "degrees" else target
    command = {
        "kind": "blacknode.joint-command-request",
        "schema_version": 1,
        "joint_name": servo["joint_name"],
        "servo_id": servo_id,
        "position_rad": target_rad,
        "issued_at": time.time(),
        "source": "RobotServo",
        "requires_motion_authorization": True,
    }
    servo["target_position"] = target
    servo["command"] = command
    report = (
        (
            f"servo {servo_id} ({servo['joint_name']}): "
            f"{position:.3f} {units}, target preview {target:.3f} {units}"
        )
        if servo["available"]
        else (
            f"servo {servo_id} is not present in the supplied state; connect a "
            "Robot monitor target and live state, or select another servo ID"
        )
    )
    if lower is not None and upper is not None:
        report += f"\nlimits: {float(lower):.3f} .. {float(upper):.3f} {units}"
    report += (
        "\ncalibration: active"
        if servo["calibrated"]
        else "\ncalibration: not active"
    )
    report += "\npreview only: connect this node to blacknode-motion to execute"
    return {
        "servo": servo,
        "available": bool(servo["available"]),
        "joint": str(servo["joint_name"]),
        "servo_id": servo_id,
        "position": position,
        "velocity": float(servo.get("velocity") or 0.0),
        "raw_position": int(servo.get("raw_position") or 0),
        "limits": {
            "lower": lower,
            "upper": upper,
            "units": units,
        },
        "calibrated": bool(servo["calibrated"]),
        "temperature_c": float(servo.get("temperature_c") or 0.0),
        "voltage_v": float(servo.get("voltage_v") or 0.0),
        "faults": list(servo["faults"]),
        "target_position": target,
        "command": command,
        "report": report,
    }
