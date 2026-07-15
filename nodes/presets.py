"""Curated robot driver presets: a known, tested robot -> a filled driver descriptor.

Deliberately separate from robot.py, which stays 100% robot-agnostic. Adding
a new supported robot is one dict entry in _PRESETS (plus a new
<protocol>_bus_driver.py in ../drivers/ only if it's a new wire protocol,
never a new node type).
"""
from __future__ import annotations

from blacknode.node import Dict, Enum, Float, Int, Text, node

from .profiles import _driver_from_profile, builtin_profile

_CATEGORY = "Robot"

_PRESET_IDS = ["so_arm101"]


@node(
    name="RobotDriverPreset",
    category=_CATEGORY,
    hidden=True,
    description="Fill in a driver descriptor for a known, tested robot. Drop-in ahead of RobotDriverLauncher/RobotDiscovery, same output shape as RobotDriverDescriptor.",
    inputs={
        "preset": Enum(_PRESET_IDS, default="so_arm101"),
        "state_topic": Text(default="/joint_states"),
        "command_topic": Text(default="/joint_commands"),
        "config_topic": Text(default="/joint_config"),
        "control_topic": Text(default="/robot_control"),
        "rate_hz": Float(default=60.0),
        "transport": Enum(["auto", "native", "rosbridge"], default="auto"),
        "host": Text(default="127.0.0.1"),
        "port": Int(default=9090),
    },
    outputs={"driver": Dict, "report": Text},
)
def robot_driver_preset(ctx: dict) -> dict:
    preset_id = str(ctx.get("preset") or "so_arm101").strip() or "so_arm101"
    profile = builtin_profile(preset_id)
    if profile is None:
        known = ", ".join(_PRESET_IDS)
        return {"driver": {}, "report": f"unknown robot preset: {preset_id} (known: {known})"}
    config = profile["driver"]
    config.update({
        "state_topic": str(ctx.get("state_topic") or "/joint_states"),
        "command_topic": str(ctx.get("command_topic") or "/joint_commands"),
        "config_topic": str(ctx.get("config_topic") or "/joint_config"),
        "control_topic": str(ctx.get("control_topic") or "/robot_control"),
        "rate_hz": float(ctx.get("rate_hz") or 60.0),
        "transport": str(ctx.get("transport") or "auto").strip().lower(),
        "host": str(ctx.get("host") or "127.0.0.1"),
        "port": int(ctx.get("port") or 9090),
    })
    driver = _driver_from_profile(profile)
    transport_note = " (auto-selected)" if driver["requested_transport"] == "auto" else ""
    joint_lines = [f"joint map ({len(profile['joints'])} joints):"]
    for joint in profile["joints"]:
        joint_lines.append(
            f"  {joint['id']}  servo id {joint['servo_id']}   range {joint['safe_min_deg']:g}..{joint['safe_max_deg']:g} deg"
        )
    report = (
        f"robot driver preset: {profile['display_name']} ({preset_id})\n"
        f"transport: {driver['transport']}{transport_note}\n"
        f"{'\n'.join(joint_lines)}\n"
        f"launch template: {driver['command_template']}"
    )
    return {"driver": driver, "report": report}
