"""Read-only capability binding checks for supported robot ROS interfaces."""
from __future__ import annotations

from typing import Any

from blacknode.node import Bool, Dict, Enum, List, Text, node


_CATEGORY = "Robot"
_PRESETS = ["hiwonder_rosorin_pro"]


def _ros_names(values: Any) -> set[str]:
    """Normalize `ros2 * list -t` rows to graph names."""
    if not isinstance(values, list):
        return set()
    names: set[str] = set()
    for value in values:
        name = str(value or "").strip().split(maxsplit=1)[0]
        if name:
            names.add(name)
    return names


def _binding(
    *,
    label: str,
    kind: str,
    candidates: list[str],
    observed: set[str],
    required: bool,
    contains: tuple[str, ...] = (),
    note: str = "",
) -> dict[str, Any]:
    exact = next((name for name in candidates if name in observed), "")
    fuzzy = next(
        (
            name
            for name in sorted(observed)
            if contains and any(token in name.lower() for token in contains)
        ),
        "",
    )
    matched = exact or fuzzy
    return {
        "label": label,
        "kind": kind,
        "available": bool(matched),
        "matched": matched,
        "candidates": list(candidates),
        "required": required,
        "note": note,
    }


def _rosorin_pro_bindings(
    topics: set[str],
    nodes: set[str],
    services: set[str],
) -> dict[str, dict[str, Any]]:
    """Bindings documented by Hiwonder, including known camera-name variants."""
    return {
        "mobile_base": _binding(
            label="Mecanum base velocity",
            kind="topic",
            candidates=["/controller/cmd_vel"],
            observed=topics,
            required=True,
        ),
        "odometry": _binding(
            label="Base odometry",
            kind="topic",
            candidates=["/odom", "/odom_raw"],
            observed=topics,
            required=True,
        ),
        "lidar": _binding(
            label="TOF LiDAR scan",
            kind="topic",
            candidates=["/scan"],
            observed=topics,
            required=True,
        ),
        "imu": _binding(
            label="Chassis IMU",
            kind="topic",
            candidates=["/ros_robot_controller/imu_raw"],
            observed=topics,
            required=True,
        ),
        "rgb_camera": _binding(
            label="Depth-camera RGB stream",
            kind="topic",
            candidates=["/depth_cam/rgb/image_raw", "/depth_cam/rgb0/image_raw"],
            observed=topics,
            required=True,
            note="The vendor image uses either the rgb or rgb0 namespace.",
        ),
        "depth_camera": _binding(
            label="Depth image stream",
            kind="topic",
            candidates=["/depth_cam/depth/image_raw", "/depth_cam/depth0/image_raw"],
            observed=topics,
            required=True,
            note="The vendor image uses either the depth or depth0 namespace.",
        ),
        "camera_info": _binding(
            label="Depth-camera calibration",
            kind="topic",
            candidates=["/depth_cam/depth/camera_info", "/depth_cam/depth0/camera_info"],
            observed=topics,
            required=True,
        ),
        "arm_command": _binding(
            label="6DOF arm servo command",
            kind="topic",
            candidates=["/servo_controller", "/servo_controller11"],
            observed=topics,
            required=True,
            note="Blacknode keeps arm motion disarmed until live feedback and limits are confirmed.",
        ),
        "arm_kinematics": _binding(
            label="Arm inverse kinematics",
            kind="service",
            candidates=[
                "/kinematics/set_pose_target",
                "/kinematics/set_joint_value_target",
            ],
            observed=services,
            required=False,
            note="These services appear when the vendor kinematics stack is running.",
        ),
        "depth_control": _binding(
            label="Depth-camera emitter control",
            kind="service",
            candidates=["/depth_cam/set_ldp_enable"],
            observed=services,
            required=False,
        ),
        "navigation": _binding(
            label="Navigation stack",
            kind="node",
            candidates=[],
            observed=nodes,
            required=False,
            contains=("nav2", "navigate", "planner_server", "controller_server"),
            note="Navigation nodes are normally launched only for a navigation mission.",
        ),
        "slam": _binding(
            label="SLAM mapping",
            kind="node",
            candidates=[],
            observed=nodes | topics,
            required=False,
            contains=("slam", "gmapping", "/map"),
            note="Mapping nodes and /map appear only while the mapping stack is active.",
        ),
        "voice": _binding(
            label="Voice interaction",
            kind="node",
            candidates=[],
            observed=nodes | topics | services,
            required=False,
            contains=("voice", "speech", "audio", "microphone", "wonder_echo", "asr", "tts"),
            note="Voice interfaces vary by kit and vendor image; live discovery selects the binding.",
        ),
        "buzzer": _binding(
            label="Controller buzzer",
            kind="topic",
            candidates=["/ros_robot_controller/set_buzzer"],
            observed=topics,
            required=False,
        ),
    }


@node(
    name="RobotROSInterfaceCheck",
    component="capabilities",
    category=_CATEGORY,
    description=(
        "Match a live ROS graph against a supported robot interface profile. "
        "Read-only: discovers capability bindings and never publishes commands."
    ),
    inputs={
        "preset": Enum(_PRESETS, default="hiwonder_rosorin_pro"),
        "topics": List(default=[]),
        "nodes": List(default=[]),
        "services": List(default=[]),
    },
    outputs={
        "ready": Bool,
        "capabilities": List,
        "missing": List,
        "bindings": Dict,
        "report": Text,
    },
)
def robot_ros_interface_check(ctx: dict) -> dict:
    preset = str(ctx.get("preset") or "hiwonder_rosorin_pro").strip()
    topics = _ros_names(ctx.get("topics"))
    nodes = _ros_names(ctx.get("nodes"))
    services = _ros_names(ctx.get("services"))
    if preset != "hiwonder_rosorin_pro":
        return {
            "ready": False,
            "capabilities": [],
            "missing": [],
            "bindings": {},
            "report": f"unknown robot ROS interface preset: {preset}",
        }

    bindings = _rosorin_pro_bindings(topics, nodes, services)
    capabilities = [name for name, value in bindings.items() if value["available"]]
    missing = [name for name, value in bindings.items() if not value["available"]]
    required = [name for name, value in bindings.items() if value["required"]]
    missing_required = [name for name in required if not bindings[name]["available"]]
    ready = not missing_required

    lines = [
        "Hiwonder ROSOrin Pro ROS interface readiness",
        f"observed: {len(topics)} topic(s), {len(nodes)} node(s), {len(services)} service(s)",
        f"required data-plane bindings: {len(required) - len(missing_required)}/{len(required)}",
        "",
    ]
    for name, value in bindings.items():
        if value["available"]:
            state = "READY"
            detail = value["matched"]
        elif value["required"]:
            state = "MISSING"
            detail = " or ".join(value["candidates"]) or "matching live interface"
        else:
            state = "ON DEMAND"
            detail = "not active in this graph"
        lines.append(f"[{state}] {value['label']}: {detail}")
    lines.extend([
        "",
        (
            "READY: the read-only base, sensor, camera, and arm-command interfaces were discovered. "
            "Keep motion disarmed until feedback, calibration, limits, and stop behavior are verified."
            if ready
            else "NEXT: start the vendor base/camera/arm services, then run this check again. No motion command was sent."
        ),
    ])
    return {
        "ready": ready,
        "capabilities": capabilities,
        "missing": missing,
        "bindings": {
            "preset": preset,
            "topics": sorted(topics),
            "nodes": sorted(nodes),
            "services": sorted(services),
            "capabilities": bindings,
        },
        "report": "\n".join(lines),
    }
