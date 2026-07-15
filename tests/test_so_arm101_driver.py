import importlib.util
import math
import sys
from types import SimpleNamespace
from pathlib import Path

import blacknode  # noqa: F401
from blacknode.node import _NODE_REGISTRY
from blacknode.pkg.blacknode_robot import presets as presets_module
from blacknode.pkg.blacknode_robot import profiles as profiles_module

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


def test_robot_driver_preset_so_arm101_shape(monkeypatch):
    monkeypatch.setattr(profiles_module.importlib.util, "find_spec", lambda name: object() if name == "rclpy" else None)
    result = _NODE_REGISTRY["RobotDriverPreset"]({"preset": "so_arm101"})
    driver = result["driver"]

    assert driver["id"] == "so_arm101"
    assert driver["transport"] == "native"
    assert driver["requested_transport"] == "auto"
    assert driver["state_topic"] == "/joint_states"
    assert driver["command_topic"] == "/joint_commands"
    assert driver["config_topic"] == "/joint_config"
    assert driver["control_topic"] == "/robot_control"
    assert "--control-topic {control_topic}" in driver["command_template"]
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


def test_robot_driver_preset_auto_falls_back_to_rosbridge_without_rclpy(monkeypatch):
    monkeypatch.setattr(profiles_module.importlib.util, "find_spec", lambda name: None)

    result = _NODE_REGISTRY["RobotDriverPreset"]({"preset": "so_arm101"})

    assert result["driver"]["transport"] == "rosbridge"
    assert result["driver"]["requested_transport"] == "auto"
    assert "transport: rosbridge (auto-selected)" in result["report"]


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
    assert mod._config_payload(joints, torque_enabled=False)["teach_mode"] is True
    assert mod._config_payload(joints, torque_enabled=False)["commands_allowed"] is False


def test_driver_reenables_torque_only_after_reading_and_seeding_every_joint():
    mod = _load_driver_module()
    joints = mod.parse_joint_map("shoulder_pan:1:-100:100,gripper:6:-10:90", {}, set())
    calls = []

    class Packet:
        def read2ByteTxRx(self, _port, servo_id, _address):
            calls.append(("read", servo_id))
            return 2000 + servo_id, 0, 0

        def write2ByteTxRx(self, _port, servo_id, _address, ticks):
            calls.append(("goal", servo_id, ticks))
            return 0, 0

        def write1ByteTxRx(self, _port, servo_id, _address, enabled):
            calls.append(("torque", servo_id, enabled))
            return 0, 0

    ok, positions, error = mod._enable_all_torque_at_current_pose(
        SimpleNamespace(COMM_SUCCESS=0), Packet(), object(), joints
    )

    assert ok is True
    assert error == ""
    assert positions == {"shoulder_pan": 2001, "gripper": 2006}
    assert calls == [
        ("read", 1), ("read", 6),
        ("goal", 1, 2001), ("goal", 6, 2006),
        ("torque", 1, 1), ("torque", 6, 1),
    ]


def test_driver_failed_teach_exit_returns_every_joint_to_torque_off():
    mod = _load_driver_module()
    joints = mod.parse_joint_map("shoulder_pan:1:-100:100,gripper:6:-10:90", {}, set())
    torque_writes = []

    class Packet:
        def read2ByteTxRx(self, _port, servo_id, _address):
            return 2048, 0, 0

        def write2ByteTxRx(self, _port, servo_id, _address, _ticks):
            return (1, 0) if servo_id == 6 else (0, 0)

        def write1ByteTxRx(self, _port, servo_id, _address, enabled):
            torque_writes.append((servo_id, enabled))
            return 0, 0

    ok, _positions, error = mod._enable_all_torque_at_current_pose(
        SimpleNamespace(COMM_SUCCESS=0), Packet(), object(), joints
    )

    assert ok is False
    assert "could not seed Goal_Position for gripper" in error
    assert torque_writes == [(1, 0), (6, 0)]


def test_driver_transient_short_packet_does_not_crash_pose_polling():
    mod = _load_driver_module()
    sdk = SimpleNamespace(COMM_SUCCESS=0)

    def short_packet(*_args):
        raise IndexError("list index out of range")

    packet = SimpleNamespace(read2ByteTxRx=short_packet)

    assert mod._read_position_or_none(sdk, packet, object(), 1) is None


def test_repeated_torque_control_is_idempotent_but_errors_retry():
    mod = _load_driver_module()

    assert mod._control_already_applied(
        "enter_teach", {"torque_enabled": False, "last_error": ""}
    ) is True
    assert mod._control_already_applied(
        "exit_teach", {"torque_enabled": True, "last_error": ""}
    ) is True
    assert mod._control_already_applied(
        "enter_teach", {"torque_enabled": True, "last_error": ""}
    ) is False
    assert mod._control_already_applied(
        "exit_teach", {"torque_enabled": False, "last_error": ""}
    ) is False
    assert mod._control_already_applied(
        "enter_teach", {"torque_enabled": False, "last_error": "one servo failed"}
    ) is False


def test_driver_command_falls_back_to_confirmed_writes_for_older_sdk():
    mod = _load_driver_module()
    joints = mod.parse_joint_map("shoulder_pan:1:-100:100", {}, set())
    writes = []
    packet = SimpleNamespace(
        write2ByteTxRx=lambda _port, servo_id, address, ticks: writes.append(
            (servo_id, address, ticks)
        ) or (0, 0),
        write2ByteTxOnly=lambda *_args: (_ for _ in ()).throw(
            AssertionError("command writes must consume the servo response")
        ),
    )

    mod._apply_command(
        {"name": ["shoulder_pan"], "position": [math.radians(5.0)]},
        joints,
        SimpleNamespace(COMM_SUCCESS=0),
        packet,
        object(),
    )

    assert len(writes) == 1
    assert writes[0][0] == 1


def test_driver_command_uses_one_group_sync_write_for_all_joints():
    mod = _load_driver_module()
    joints = mod.parse_joint_map(
        "shoulder_pan:1:-100:100,elbow_flex:3:-100:100", {}, set()
    )
    groups = []

    class FakeGroupSyncWrite:
        def __init__(self, _port, _packet, address, width):
            self.address = address
            self.width = width
            self.params = []
            self.tx_count = 0
            groups.append(self)

        def addParam(self, servo_id, data):
            self.params.append((servo_id, data))
            return True

        def txPacket(self):
            self.tx_count += 1
            return 0

    sdk = SimpleNamespace(
        COMM_SUCCESS=0,
        GroupSyncWrite=FakeGroupSyncWrite,
        SCS_LOBYTE=lambda value: value & 0xFF,
        SCS_HIBYTE=lambda value: (value >> 8) & 0xFF,
    )
    mod._apply_command(
        {
            "name": ["shoulder_pan", "elbow_flex"],
            "position": [math.radians(5.0), math.radians(-8.0)],
        },
        joints,
        sdk,
        object(),
        object(),
    )

    assert len(groups) == 1
    assert groups[0].tx_count == 1
    assert [servo_id for servo_id, _data in groups[0].params] == [1, 3]
    assert all(len(data) == 2 for _servo_id, data in groups[0].params)


def test_driver_reads_all_joint_positions_with_one_group_transaction():
    mod = _load_driver_module()
    joints = mod.parse_joint_map(
        "shoulder_pan:1:-100:100,elbow_flex:3:-100:100", {}, set()
    )
    groups = []

    class FakeGroupSyncRead:
        def __init__(self, _port, _packet, address, width):
            self.address = address
            self.width = width
            self.ids = []
            self.tx_count = 0
            groups.append(self)

        def addParam(self, servo_id):
            self.ids.append(servo_id)
            return True

        def txRxPacket(self):
            self.tx_count += 1
            return 0

        def isAvailable(self, servo_id, _address, _width):
            return servo_id in self.ids

        def getData(self, servo_id, _address, _width):
            return {1: 2100, 3: 1900}[servo_id]

    result = mod._sync_read_positions(
        SimpleNamespace(COMM_SUCCESS=0, GroupSyncRead=FakeGroupSyncRead),
        object(),
        object(),
        joints,
    )

    assert result == {"shoulder_pan": 2100, "elbow_flex": 1900}
    assert len(groups) == 1
    assert groups[0].tx_count == 1


def test_driver_uses_ros2_rosbridge_message_type_names():
    source = _DRIVER_PATH.read_text(encoding="utf-8")

    assert '"sensor_msgs/msg/JointState"' in source
    assert '"std_msgs/msg/String"' in source
    assert '"sensor_msgs/JointState"' not in source


def test_driver_rosbridge_reconnects_and_resumes_state_stream():
    mod = _load_driver_module()
    joints = mod.parse_joint_map("shoulder_pan:1:-100:100", {}, set())

    class FakeRos:
        def __init__(self, **_kwargs):
            self.is_connected = False
            self.terminated = False

        def run(self, timeout):
            self.is_connected = True

        def terminate(self):
            self.terminated = True
            self.is_connected = False

    topics = {}

    class FakeTopic:
        def __init__(self, ros, name, message_type, **_kwargs):
            self.ros = ros
            self.name = name
            self.message_type = message_type
            self.messages = []
            topics[name] = self

        def advertise(self):
            pass

        def unadvertise(self):
            pass

        def subscribe(self, callback):
            self.callback = callback

        def unsubscribe(self):
            pass

        def publish(self, message):
            self.messages.append(message)

    fake_roslibpy = SimpleNamespace(
        Ros=FakeRos,
        Topic=FakeTopic,
        Message=lambda value: value,
    )
    ros_box = {}
    original_ros = fake_roslibpy.Ros

    def make_ros(**kwargs):
        ros_box["ros"] = original_ros(**kwargs)
        return ros_box["ros"]

    fake_roslibpy.Ros = make_ros

    class ReconnectStopEvent:
        def __init__(self):
            self.period_waits = 0

        def is_set(self):
            return False

        def wait(self, timeout):
            if timeout == 1.1:
                ros_box["ros"].is_connected = True
                return False
            self.period_waits += 1
            if self.period_waits == 1:
                ros_box["ros"].is_connected = False
                return False
            if self.period_waits == 2:
                ros_box["ros"].is_connected = True
                return False
            return True

    args = SimpleNamespace(
        host="127.0.0.1",
        rosbridge_port=9090,
        connect_timeout=1.0,
        state_topic="/joint_states",
        config_topic="/joint_config",
        command_topic="/joint_commands",
        control_topic="/robot_control",
        rate_hz=10.0,
    )
    sdk = SimpleNamespace(COMM_SUCCESS=0)
    packet = SimpleNamespace(read2ByteTxRx=lambda *_args: (2048, 0, 0))

    mod._run_rosbridge(
        args,
        {"roslibpy": fake_roslibpy},
        joints,
        sdk,
        packet,
        object(),
        {"shoulder_pan": 2048},
        ReconnectStopEvent(),
    )

    assert len(topics["/joint_config"].messages) == 2
    assert len(topics["/joint_states"].messages) >= 3
    assert ros_box["ros"].terminated is True


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
        first_pid = _NODE_REGISTRY["RobotDriverLauncher"].__globals__["_managed_drivers"]["test_so_arm101_driver"].pid
        repeated = _NODE_REGISTRY["RobotDriverLauncher"]({
            "action": "start",
            "run_id": "test_so_arm101_driver",
            "driver": driver,
            "serial_port": "/dev/ttyACM0",
        })
        second_pid = _NODE_REGISTRY["RobotDriverLauncher"].__globals__["_managed_drivers"]["test_so_arm101_driver"].pid
        assert repeated["running"] is True
        assert "already running" in repeated["report"]
        assert second_pid == first_pid
    finally:
        stopped = _NODE_REGISTRY["RobotDriverLauncher"]({
            "action": "stop",
            "run_id": "test_so_arm101_driver",
            "driver": driver,
        })
        assert stopped["running"] is False
