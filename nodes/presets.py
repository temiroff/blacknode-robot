"""Curated robot driver presets: a known, tested robot -> a filled driver descriptor.

Deliberately separate from robot.py, which stays 100% robot-agnostic. Adding
a new supported robot is one dict entry in _PRESETS (plus a new
<protocol>_bus_driver.py in ../drivers/ only if it's a new wire protocol,
never a new node type).
"""
from __future__ import annotations

from pathlib import Path
from typing import Any

from blacknode.node import Dict, Enum, Float, Int, Text, node

_CATEGORY = "Robot"
_DRIVERS_DIR = Path(__file__).resolve().parents[1] / "drivers"

# name -> (servo_id, min_deg, max_deg). Degrees are relative to each servo's
# raw center tick (2048 of 4095) by default -- see feetech_bus_driver.py's
# --home-ticks / --invert if a joint's true mechanical zero or sign differs
# on the physical arm. This sweep is a starting placeholder: confirm it
# against the real SO-ARM101 (see packages/blacknode-robot/README.md Safety
# section) before setting armed=true.
_SO_ARM101_JOINTS: dict[str, tuple[int, float, float]] = {
    "shoulder_pan": (1, -100.0, 100.0),
    "shoulder_lift": (2, -100.0, 100.0),
    "elbow_flex": (3, -100.0, 100.0),
    "wrist_flex": (4, -100.0, 100.0),
    "wrist_roll": (5, -150.0, 150.0),
    "gripper": (6, -10.0, 90.0),
}

_PRESETS: dict[str, dict[str, Any]] = {
    "so_arm101": {
        "name": "SO-ARM101 (Feetech STS3215 x6)",
        "script": "feetech_bus_driver.py",
        "baudrate": 1_000_000,
        "joints": _SO_ARM101_JOINTS,
    },
}


def _joints_arg(joints: dict[str, tuple[int, float, float]]) -> str:
    return ",".join(f"{name}:{servo_id}:{lo:g}:{hi:g}" for name, (servo_id, lo, hi) in joints.items())


def _joint_table(joints: dict[str, tuple[int, float, float]]) -> str:
    name_width = max(len(name) for name in joints)
    lines = [f"joint map ({len(joints)} joints):"]
    for name, (servo_id, lo, hi) in joints.items():
        lines.append(f"  {name.ljust(name_width)}  servo id {servo_id}   range {lo:g}..{hi:g} deg")
    return "\n".join(lines)


@node(
    name="RobotDriverPreset",
    category=_CATEGORY,
    description="Fill in a driver descriptor for a known, tested robot. Drop-in ahead of RobotDriverLauncher/RobotDiscovery, same output shape as RobotDriverDescriptor.",
    inputs={
        "preset": Enum(sorted(_PRESETS), default="so_arm101"),
        "state_topic": Text(default="/joint_states"),
        "command_topic": Text(default="/joint_commands"),
        "config_topic": Text(default="/joint_config"),
        "rate_hz": Float(default=15.0),
        "transport": Enum(["native", "rosbridge"], default="native"),
        "host": Text(default="127.0.0.1"),
        "port": Int(default=9090),
    },
    outputs={"driver": Dict, "report": Text},
)
def robot_driver_preset(ctx: dict) -> dict:
    preset_id = str(ctx.get("preset") or "so_arm101").strip() or "so_arm101"
    preset = _PRESETS.get(preset_id)
    if preset is None:
        known = ", ".join(sorted(_PRESETS))
        return {"driver": {}, "report": f"unknown robot preset: {preset_id} (known: {known})"}

    script = _DRIVERS_DIR / str(preset["script"])
    state_topic = str(ctx.get("state_topic") or "/joint_states")
    command_topic = str(ctx.get("command_topic") or "/joint_commands")
    config_topic = str(ctx.get("config_topic") or "/joint_config")
    rate_hz = float(ctx.get("rate_hz") or 15.0)
    transport = str(ctx.get("transport") or "native")
    host = str(ctx.get("host") or "127.0.0.1")
    port = int(ctx.get("port") or 9090)

    command_template = (
        f'"{{python}}" "{script}" --port "{{serial_port}}" --baudrate {preset["baudrate"]} '
        f'--joints "{_joints_arg(preset["joints"])}" '
        f'--state-topic {{state_topic}} --command-topic {{command_topic}} --config-topic {{config_topic}} '
        f'--rate-hz {rate_hz:g} --transport {transport} --host "{host}" --rosbridge-port {port}'
    )
    driver = {
        "id": preset_id,
        "name": preset["name"],
        "command_template": command_template,
        "transport": transport,
        "host": host,
        "port": port,
        "state_topic": state_topic,
        "command_topic": command_topic,
        "config_topic": config_topic,
        "units": "degrees",
        "match": {"vendor_id": "", "product_id": ""},
    }
    report = (
        f"robot driver preset: {preset['name']} ({preset_id})\n"
        f"{_joint_table(preset['joints'])}\n"
        f"launch template: {command_template}"
    )
    if not script.exists():
        report += f"\nWARNING: driver script not found at {script}"
    return {"driver": driver, "report": report}
