from __future__ import annotations

import json
from collections import defaultdict
from pathlib import Path


_TEMPLATE_DIR = Path(__file__).resolve().parents[1] / "templates"
_EXPECTED_ROBOTS = {
    "complete-robot-bringup.json": {"robot": 0},
    "compute-device-inspection.json": {},
    "editable-so-arm101-profile.json": {},
    "generic-ros-capability-discovery.json": {},
    "robot-guided-calibration.json": {"robot": 0},
    "robot-sensor-attachments.json": {},
    "servo-debug-monitor.json": {},
    "so-arm101-motion-test.json": {"robot": 0},
}


def test_editable_profile_joints_follow_servo_id_order_on_canvas() -> None:
    path = _TEMPLATE_DIR / "editable-so-arm101-profile.json"
    workflow = json.loads(path.read_text(encoding="utf-8"))
    joint_nodes = [
        node
        for node in workflow["node_meta"].values()
        if node.get("type") == "RobotJointDefinition"
    ]
    visual_order = sorted(
        joint_nodes,
        key=lambda node: (node["pos"][1], node["pos"][0]),
    )

    assert [node["params"]["servo_id"] for node in visual_order] == [1, 2, 3, 4, 5, 6]
    assert all(
        node.get("type") != "Robot"
        for node in workflow["node_meta"].values()
    )


def test_template_adapters_are_not_declared_as_components() -> None:
    for path in sorted(_TEMPLATE_DIR.glob("*.json")):
        workflow = json.loads(path.read_text(encoding="utf-8"))
        required_components = (workflow.get("metadata") or {}).get(
            "required_components",
            [],
        )

        assert all(
            "@" not in requirement
            for requirement in required_components
            if isinstance(requirement, str)
        ), f"{path.name}: adapters belong in metadata.required_adapters"


def test_guided_calibration_uses_profile_bound_generic_control() -> None:
    path = _TEMPLATE_DIR / "robot-guided-calibration.json"
    workflow = json.loads(path.read_text(encoding="utf-8"))
    node_types = {
        node["type"]
        for node in workflow["node_meta"].values()
    }
    metadata = workflow["metadata"]

    assert node_types == {
        "Robot",
        "RobotCalibrationControl",
        "RobotCalibrationRecorder",
        "Output",
    }
    assert workflow["node_meta"]["robot"]["params"]["action"] == "check"
    assert workflow["node_meta"]["robot"]["params"]["profile_id"] == "auto"
    assert metadata["required_packages"] == ["blacknode-robot"]
    assert metadata["required_capabilities"] == [
        "calibration_control",
        "position_feedback",
    ]
    assert metadata.get("required_adapters") in (None, [])
    assert all(
        "feetech" not in requirement.lower()
        for requirement in metadata["required_components"]
    )
    assert all(not node_type.startswith("ROS2") for node_type in node_types)
    assert {
        (
            edge["from"],
            edge["from_port"],
            edge["to"],
            edge["to_port"],
        )
        for edge in workflow["edges"]
    } >= {
        ("robot", "hardware_id", "calibration", "hardware_id"),
    }


def test_physical_device_selectors_use_unique_indexes() -> None:
    for path in sorted(_TEMPLATE_DIR.glob("*.json")):
        workflow = json.loads(path.read_text(encoding="utf-8"))
        nodes = workflow.get("node_meta") or {}
        actual_robots = {
            node_id: (node.get("params") or {}).get("selection", 0)
            for node_id, node in nodes.items()
            if node.get("type") == "Robot"
        }
        assert actual_robots == _EXPECTED_ROBOTS[path.name]
        incoming: dict[str, set[str]] = defaultdict(set)
        for edge in workflow.get("edges", []):
            incoming[str(edge.get("to") or "")].add(str(edge.get("to_port") or ""))

        used: dict[str, dict[int, str]] = defaultdict(dict)
        for node_id, node in nodes.items():
            device_type = node.get("type")
            if device_type not in {"Robot", "Camera"}:
                continue

            chained_robot_ports = incoming[node_id] & {"hardware", "usb", "driver"}
            assert device_type != "Robot" or not chained_robot_ports, (
                f"{path.name}: {node_id} rebuilds a low-level Robot setup chain through "
                f"{sorted(chained_robot_ports)} instead of using one Robot facade"
            )
            supplied_ports = set() if device_type == "Robot" else {"camera"}
            if incoming[node_id] & supplied_ports:
                continue

            selection = (node.get("params") or {}).get("selection", 0)
            assert isinstance(selection, int) and selection >= 0, (
                f"{path.name}: {node_id} has invalid {device_type} selection {selection!r}"
            )
            assert selection not in used[device_type], (
                f"{path.name}: {node_id} and {used[device_type][selection]} independently "
                f"select {device_type} index {selection}"
            )
            used[device_type][selection] = node_id
