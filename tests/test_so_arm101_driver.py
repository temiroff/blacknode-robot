import importlib.util
import math
import sys
from pathlib import Path

import blacknode  # noqa: F401
from blacknode.node import _NODE_REGISTRY
from blacknode.pkg.blacknode_robot import presets as presets_module

_DRIVER_PATH = Path(presets_module.__file__).resolve().parents[1] / "drivers" / "feetech_bus_driver.py"


def _load_driver_module():
    # Only works because feetech_bus_driver.py defers its rclpy/scservo_sdk
    # imports into _hardware_imports(): the module itself has no hard
    # dependency on either, so this exercises the real file with no ROS 2
    # sourced and no servo SDK installed.
    spec = importlib.util.spec_from_file_location("feetech_bus_driver", _DRIVER_PATH)
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module  # dataclass field resolution needs this in sys.modules
    spec.loader.exec_module(module)
    return module


def test_robot_driver_preset_registered():
    assert "RobotDriverPreset" in _NODE_REGISTRY
    assert _NODE_REGISTRY["RobotDriverPreset"]._bn_category == "Robot"
    assert _NODE_REGISTRY["RobotDriverPreset"]._bn_package == "blacknode-robot"


def test_robot_driver_preset_so_arm101_shape():
    result = _NODE_REGISTRY["RobotDriverPreset"]({"preset": "so_arm101"})
    driver = result["driver"]

    assert driver["id"] == "so_arm101"
    assert driver["transport"] == "native"
    assert driver["state_topic"] == "/joint_states"
    assert driver["command_topic"] == "/joint_commands"
    assert driver["config_topic"] == "/joint_config"
    for name in ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"]:
        assert name in driver["command_template"]
    assert "--baudrate 1000000" in driver["command_template"]
    assert "{serial_port}" in driver["command_template"]
    assert "{python}" in driver["command_template"]
    assert _DRIVER_PATH.exists()
    assert "WARNING" not in result["report"]


def test_robot_driver_preset_unknown_id_does_not_raise():
    result = _NODE_REGISTRY["RobotDriverPreset"]({"preset": "not_a_real_robot"})

    assert result["driver"] == {}
    assert "unknown robot preset" in result["report"]


def test_robot_driver_preset_rosbridge_transport():
    driver = _NODE_REGISTRY["RobotDriverPreset"]({
        "preset": "so_arm101",
        "transport": "rosbridge",
        "host": "127.0.0.1",
        "port": 9090,
    })["driver"]

    assert driver["transport"] == "rosbridge"
    assert driver["host"] == "127.0.0.1"
    assert driver["port"] == 9090
    assert "--transport rosbridge" in driver["command_template"]
    assert '--host "127.0.0.1" --rosbridge-port 9090' in driver["command_template"]


def test_driver_joint_map_parsing_and_degree_math():
    mod = _load_driver_module()

    joints = mod.parse_joint_map("shoulder_pan:1:-100:100,gripper:6:-10:90", {}, set())
    assert joints["shoulder_pan"].servo_id == 1
    assert joints["gripper"].max_deg == 90.0

    # protocol center tick (2048) is exactly zero degrees by default
    assert mod.ticks_to_degrees(2048, joints["shoulder_pan"]) == 0.0
    # a quarter turn (90 degrees) is 1024 of 4096 ticks off center
    assert mod.degrees_to_ticks(90.0, joints["shoulder_pan"]) == 2048 + 1024
    # round trip
    assert mod.ticks_to_degrees(mod.degrees_to_ticks(45.0, joints["shoulder_pan"]), joints["shoulder_pan"]) == 45.0

    assert mod.clamp_degrees(999.0, joints["shoulder_pan"]) == 100.0
    assert mod.clamp_degrees(-999.0, joints["shoulder_pan"]) == -100.0


def test_driver_rosbridge_payload_contains_discovered_joint_names():
    mod = _load_driver_module()
    joints = mod.parse_joint_map("shoulder_pan:1:-100:100,gripper:6:-10:90", {}, set())

    payload = mod._joint_state_payload({"shoulder_pan": 2048, "gripper": 2048}, joints)

    assert payload["name"] == ["shoulder_pan", "gripper"]
    assert payload["position"] == [0.0, 0.0]
    assert mod._config_payload(joints)["commands_allowed"] is True


def test_driver_home_ticks_and_invert_overrides():
    mod = _load_driver_module()

    home_overrides = mod.parse_int_map("shoulder_pan:2100")
    joints = mod.parse_joint_map("shoulder_pan:1:-100:100", home_overrides, {"shoulder_pan"})
    joint = joints["shoulder_pan"]

    assert joint.home_ticks == 2100
    assert joint.invert is True
    # inverted: a positive-degree command should move ticks below home, not above
    assert mod.degrees_to_ticks(10.0, joint) < joint.home_ticks
    assert mod.ticks_to_degrees(joint.home_ticks, joint) == 0.0


def test_driver_ticks_to_degrees_clamped_to_servo_range():
    mod = _load_driver_module()
    joints = mod.parse_joint_map("j:1:-360:360", {}, set())
    # degrees_to_ticks must never escape the servo's raw 0-4095 range even
    # for an out-of-hardware-range request
    assert 0 <= mod.degrees_to_ticks(10_000.0, joints["j"]) <= 4095
    assert 0 <= mod.degrees_to_ticks(-10_000.0, joints["j"]) <= 4095


def test_driver_launcher_accepts_preset_command_template_shape():
    # Reuses the fake-long-running-subprocess pattern from test_robot_nodes.py
    # to prove RobotDriverLauncher accepts whatever RobotDriverPreset hands
    # it, without touching real hardware.
    driver = dict(_NODE_REGISTRY["RobotDriverPreset"]({"preset": "so_arm101"})["driver"])
    driver["command_template"] = f'{sys.executable} -c "import time; time.sleep(30)"'

    try:
        started = _NODE_REGISTRY["RobotDriverLauncher"]({
            "action": "start",
            "run_id": "test_so_arm101_driver",
            "driver": driver,
            "serial_port": "/dev/ttyACM0",
        })
        assert started["running"] is True
    finally:
        stopped = _NODE_REGISTRY["RobotDriverLauncher"]({
            "action": "stop",
            "run_id": "test_so_arm101_driver",
            "driver": driver,
        })
        assert stopped["running"] is False
