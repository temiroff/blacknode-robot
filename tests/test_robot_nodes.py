import base64
import io
import json
import sys
from pathlib import Path
from types import SimpleNamespace

import blacknode  # noqa: F401
from blacknode.node import _NODE_REGISTRY
from blacknode.workflow import validate_workflow
from blacknode.pkg.blacknode_robot import robot as robot_nodes
from blacknode.pkg.blacknode_robot import profiles as profile_nodes


EXPECTED_NODES = [
    "RobotUSBDiscovery",
    "RobotDriverDescriptor",
    "RobotDriverLauncher",
    "RobotDiscovery",
    "RobotDriverPreset",
    "Robot",
    "RobotConnectionDashboard",
    "RobotJointDefinition",
    "RobotJointList",
    "RobotDefinition",
    "RobotProfileSave",
    "RobotProfileLoad",
    "RobotProfileList",
    "RobotProfileDuplicate",
    "RobotCalibrationRecorder",
]


def test_robot_nodes_registered_with_category():
    for name in EXPECTED_NODES:
        assert name in _NODE_REGISTRY, name
        assert _NODE_REGISTRY[name]._bn_category == "Robot"
        assert _NODE_REGISTRY[name]._bn_package == "blacknode-robot"


def test_usb_discovery_reports_no_devices(monkeypatch):
    monkeypatch.setattr(robot_nodes, "_serial_candidate_paths", lambda: [])
    result = _NODE_REGISTRY["RobotUSBDiscovery"]({})

    assert result["found"] is False
    assert result["ready"] is False
    assert result["port"] == ""
    assert result["serial"] == ""
    assert result["devices"] == []
    assert "no USB serial ports detected" in result["report"]


def test_usb_discovery_reports_permission_fix(monkeypatch):
    monkeypatch.setattr(robot_nodes, "_serial_candidate_paths", lambda: ["/dev/serial/by-id/usb-Test_Robot"])
    monkeypatch.setattr(robot_nodes, "_serial_device_info", lambda path, probe_open=False: {
        "path": path,
        "real_path": "/dev/ttyACM0",
        "name": "ttyACM0",
        "by_id": path,
        "manufacturer": "Test",
        "product": "Robot",
        "serial": "abc",
        "vendor_id": "1234",
        "product_id": "5678",
        "group": "dialout",
        "mode": "0o660",
        "readable": False,
        "writable": False,
        "accessible": False,
        "fixes": ["sudo usermod -aG dialout alex", "log out and back in, or run a new shell with: newgrp dialout"],
    })
    monkeypatch.setattr(robot_nodes, "_current_username", lambda: "alex")
    monkeypatch.setattr(robot_nodes, "_user_group_names", lambda: ["alex"])

    result = _NODE_REGISTRY["RobotUSBDiscovery"]({})

    assert result["found"] is True
    assert result["ready"] is False
    assert result["recommended"]["path"] == "/dev/serial/by-id/usb-Test_Robot"
    assert "access blocked" in result["report"]
    assert "sudo usermod -aG dialout alex" in result["report"]


def test_usb_discovery_uses_pyserial_com_ports(monkeypatch):
    fake_port = SimpleNamespace(
        device="COM7",
        description="USB Serial Device",
        manufacturer="Feetech",
        product="SO-ARM101 Servo Bus",
        serial_number="abc123",
        vid=0x1A86,
        pid=0x7523,
        hwid="USB VID:PID=1A86:7523 SER=abc123",
    )
    monkeypatch.setattr(robot_nodes, "serial_list_ports", SimpleNamespace(comports=lambda: [fake_port]))

    result = _NODE_REGISTRY["RobotUSBDiscovery"]({})

    assert result["found"] is True
    assert result["ready"] is True
    assert result["recommended"]["path"] == "COM7"
    assert result["port"] == "COM7"
    assert result["serial"] == "abc123"
    assert result["recommended"]["vendor_id"] == "1a86"
    assert result["recommended"]["product_id"] == "7523"
    assert result["usb"]["recommended"]["serial"] == "abc123"
    assert "COM7" in result["report"]
    assert "SO-ARM101 Servo Bus" in result["report"]
    assert "OS detected" in result["report"]
    assert "discovery confirms the adapter, not robot power" in result["report"]


def test_usb_discovery_filters_using_saved_vid_pid(monkeypatch):
    monkeypatch.setattr(robot_nodes, "_serial_candidate_paths", lambda: ["COM3", "COM9"])
    devices = {
        "COM3": {"path": "COM3", "vendor_id": "1a86", "product_id": "55d3", "accessible": True, "fixes": []},
        "COM9": {"path": "COM9", "vendor_id": "1234", "product_id": "abcd", "accessible": True, "fixes": []},
    }
    monkeypatch.setattr(robot_nodes, "_serial_device_info", lambda path, probe_open=False: devices[path])

    result = _NODE_REGISTRY["RobotUSBDiscovery"]({
        "match_vendor_id": "0x1A86",
        "match_product_id": "55D3",
    })

    assert [device["path"] for device in result["devices"]] == ["COM3"]
    assert result["recommended"]["path"] == "COM3"


def test_driver_descriptor_builds_generic_contract():
    result = _NODE_REGISTRY["RobotDriverDescriptor"]({
        "driver_id": "acme",
        "name": "Acme Arm",
        "command_template": "{python} driver.py --port {serial_port}",
        "state_topic": "/acme/joint_states",
        "command_topic": "/acme/joint_commands",
    })

    driver = result["driver"]
    assert driver["id"] == "acme"
    assert driver["command_template"] == "{python} driver.py --port {serial_port}"
    assert driver["state_topic"] == "/acme/joint_states"
    assert "Acme Arm" in result["report"]


def test_driver_launcher_start_check_stop():
    descriptor = _NODE_REGISTRY["RobotDriverDescriptor"]({
        "command_template": f"{sys.executable} -c \"import time; time.sleep(30)\"",
    })["driver"]
    try:
        started = _NODE_REGISTRY["RobotDriverLauncher"]({
            "action": "start",
            "run_id": "test_robot_driver",
            "driver": descriptor,
        })
        assert started["running"] is True

        checked = _NODE_REGISTRY["RobotDriverLauncher"]({
            "action": "check",
            "run_id": "test_robot_driver",
            "driver": descriptor,
        })
        assert checked["running"] is True
    finally:
        stopped = _NODE_REGISTRY["RobotDriverLauncher"]({
            "action": "stop",
            "run_id": "test_robot_driver",
            "driver": descriptor,
        })
        assert stopped["running"] is False


def test_driver_launcher_restarts_when_profile_command_changes():
    run_id = "test_robot_profile_restart"
    first = _NODE_REGISTRY["RobotDriverDescriptor"]({
        "name": "First profile",
        "command_template": f'{sys.executable} -c "import time; time.sleep(30)"',
    })["driver"]
    second = _NODE_REGISTRY["RobotDriverDescriptor"]({
        "name": "Second profile",
        "command_template": f'{sys.executable} -c "import time; time.sleep(31)"',
    })["driver"]
    try:
        started = _NODE_REGISTRY["RobotDriverLauncher"]({
            "action": "start", "run_id": run_id, "driver": first,
        })
        first_pid = robot_nodes._managed_drivers[run_id].pid

        restarted = _NODE_REGISTRY["RobotDriverLauncher"]({
            "action": "start", "run_id": run_id, "driver": second,
        })

        assert started["running"] is True
        assert restarted["running"] is True
        assert robot_nodes._managed_drivers[run_id].pid != first_pid
        assert robot_nodes._managed_driver_commands[run_id] == restarted["command"]
        assert "restarted with updated profile" in restarted["report"]
    finally:
        _NODE_REGISTRY["RobotDriverLauncher"]({
            "action": "stop", "run_id": run_id, "driver": second,
        })


def test_identify_robot_wiggles_a_joint_and_returns(monkeypatch):
    from blacknode.pkg.blacknode_ros2 import rosbridge_runtime as rb
    moves = []
    monkeypatch.setattr(rb, "available", lambda: (True, ""))
    monkeypatch.setattr(rb, "read_pose", lambda host, port, topic, timeout: {
        "shoulder_pan": 0.1, "gripper": 0.0,
    })
    monkeypatch.setattr(rb, "stream_motion",
                        lambda host, port, cmd, names, start, target, **k:
                        moves.append((start["shoulder_pan"], target["shoulder_pan"])) or {"ok": True})

    result = robot_nodes.identify_robot({
        "host": "192.168.1.9", "port": 9090, "state_topic": "/joint_states",
        "command_topic": "/joint_commands", "units": "degrees",
    })

    assert result["moved"] is True
    assert result["joint"] == "shoulder_pan"
    # Two out-and-back cycles, each ending exactly at the start pose.
    assert len(moves) == 4
    assert moves[0][0] == 0.1 and moves[0][1] != 0.1  # nudged away
    assert moves[1] == (moves[0][1], 0.1)             # returned to start


def test_driver_stop_releases_torque_over_rosbridge(monkeypatch):
    # Windows hard-kills the driver, skipping its SIGTERM torque-off handler, so a
    # stop must actively publish the driver's 'enter_teach' disarm first.
    run_id = "test_torque_release"
    published = []
    monkeypatch.setattr(robot_nodes, "_terminate_process", lambda proc: True)

    from blacknode.pkg.blacknode_ros2 import rosbridge_runtime as rb
    monkeypatch.setattr(rb, "publish_string", lambda host, port, topic, value, timeout=2.0:
                        published.append((host, port, topic, json.loads(value))) or {"ok": True})

    robot_nodes._managed_drivers[run_id] = SimpleNamespace(poll=lambda: None, pid=4321)
    robot_nodes._managed_driver_meta[run_id] = {
        "transport": "rosbridge", "host": "192.168.1.7", "port": 9090,
        "control_topic": "/robot_control",
    }
    try:
        # Explicit stop → releases torque; internal restart (default) would not.
        robot_nodes._stop_driver(run_id, release_torque=True)
        assert published == [("192.168.1.7", 9090, "/robot_control", {"action": "enter_teach"})]
        assert run_id not in robot_nodes._managed_driver_meta
    finally:
        robot_nodes._managed_drivers.pop(run_id, None)
        robot_nodes._managed_driver_meta.pop(run_id, None)


def test_driver_launcher_preserves_late_exit_error():
    run_id = "test_late_driver_exit"
    proc = SimpleNamespace(
        poll=lambda: 7,
        returncode=7,
        stderr=io.StringIO("serial transport failed after startup"),
    )
    robot_nodes._managed_drivers[run_id] = proc
    robot_nodes._last_driver_exits.pop(run_id, None)
    try:
        result = _NODE_REGISTRY["RobotDriverLauncher"]({
            "action": "check",
            "run_id": run_id,
            "driver": {},
        })
        assert result["running"] is False
        assert "last exit code 7" in result["report"]
        assert "serial transport failed after startup" in result["report"]
        assert any(
            item["run_id"] == run_id and item["returncode"] == 7
            for item in robot_nodes.runtime_status()["recent_exits"]
        )
    finally:
        robot_nodes._managed_drivers.pop(run_id, None)
        robot_nodes._last_driver_exits.pop(run_id, None)


def test_stop_runtime_services_stops_managed_drivers():
    descriptor = _NODE_REGISTRY["RobotDriverDescriptor"]({
        "command_template": f"{sys.executable} -c \"import time; time.sleep(30)\"",
    })["driver"]
    try:
        started = _NODE_REGISTRY["RobotDriverLauncher"]({
            "action": "start",
            "run_id": "test_stop_all_driver",
            "driver": descriptor,
        })
        assert started["running"] is True
        assert robot_nodes.runtime_status()["active"] is True

        result = robot_nodes.stop_runtime_services()
        assert result["ok"] is True
        assert result["stopped"]["managed_runs"] >= 1
        assert robot_nodes.runtime_status()["active"] is False
    finally:
        _NODE_REGISTRY["RobotDriverLauncher"]({
            "action": "stop",
            "run_id": "test_stop_all_driver",
            "driver": descriptor,
        })


def test_robot_discovery_is_generic_and_driver_first(monkeypatch):
    monkeypatch.setattr(robot_nodes, "robot_usb_discovery", lambda ctx: {
        "found": True,
        "ready": True,
        "devices": [{"path": "/dev/serial/by-id/robot", "accessible": True}],
        "recommended": {"path": "/dev/serial/by-id/robot"},
        "permissions": {"fixes": []},
        "report": "USB robot discovery\n=> READY",
    })
    monkeypatch.setattr(robot_nodes, "robot_driver_launcher", lambda ctx: {
        "running": False,
        "run_id": ctx["run_id"],
        "driver": ctx["driver"],
        "command": "driver --port /dev/serial/by-id/robot",
        "report": "robot driver not running: robot_driver",
    })
    driver = _NODE_REGISTRY["RobotDriverDescriptor"]({
        "command_template": "driver --port {serial_port}",
        "state_topic": "/joints",
        "command_topic": "/commands",
    })["driver"]

    result = _NODE_REGISTRY["RobotDiscovery"]({
        "driver": driver,
        "require_usb": True,
    })

    assert result["usb_ready"] is True
    assert result["driver_running"] is False
    assert result["ready"] is False
    assert result["robot"]["usb"]["recommended"]["path"] == "/dev/serial/by-id/robot"
    assert result["robot"]["state_topic"] == "/joints"
    assert "=> NEXT: start the robot driver" in result["report"]


def test_connection_dashboard_summarizes_ready_robot():
    result = _NODE_REGISTRY["RobotConnectionDashboard"]({
        "robot": {
            "state_topic": "/joint_states",
            "command_topic": "/joint_commands",
            "usb": {"ready": True, "recommended": {"path": "COM7"}},
            "driver": {"running": True, "name": "SO-ARM101", "transport": "native"},
            "interface": {"kind": "native"},
        },
        "connected": True,
        "interface_ready": True,
        "pose": {"shoulder_pan": 12.5, "elbow_flex": -3.0},
    })

    assert result["ready"] is True
    assert result["summary"]["joint_count"] == 2
    assert result["summary"]["serial_port"] == "COM7"
    assert result["dashboard"].startswith("data:image/svg+xml;base64,")
    assert "READY" in result["report"]


def test_connection_dashboard_stays_safe_until_live_pose_arrives():
    result = _NODE_REGISTRY["RobotConnectionDashboard"]({
        "robot": {
            "usb": {"ready": True, "recommended": {"path": "/dev/ttyUSB0"}},
            "driver": {"running": True, "name": "SO-ARM101"},
        },
        "connected": True,
        "interface_ready": True,
        "pose": {},
    })

    assert result["ready"] is False
    assert result["summary"]["live_pose"] is False
    assert "WAITING" in result["report"]


def test_connection_dashboard_shows_calibrated_home_and_safe_limits():
    result = _NODE_REGISTRY["RobotConnectionDashboard"]({
        "robot": {
            "state_topic": "/joint_states",
            "command_topic": "/joint_commands",
            "usb": {
                "ready": True,
                "recommended": {"path": "COM7", "serial_number": "SERIAL-42"},
            },
            "driver": {
                "running": True,
                "name": "SO-ARM101",
                "hardware_id": "SERIAL-42",
                "calibration_path": "robots/so_arm101/calibrations/serial-42.json",
                "profile": {"calibration": {"hardware_id": "SERIAL-42"}},
                "joints": [{
                    "id": "shoulder_pan",
                    "home_ticks": 2200,
                    "home_offset_deg": 13.36,
                    "safe_min_deg": -82.4,
                    "safe_max_deg": 91.6,
                }],
            },
            "interface": {"kind": "rosbridge"},
        },
        "connected": True,
        "interface_ready": True,
        "pose": {"shoulder_pan": 12.5},
    })

    detail = result["summary"]["joints"][0]
    assert result["summary"]["calibrated"] is True
    assert result["summary"]["calibration_status"] == "saved calibration"
    assert result["summary"]["hardware_id"] == "SERIAL-42"
    assert detail["home_deg"] == 0.0
    assert detail["home_ticks"] == 2200
    assert detail["safe_min_deg"] == -82.4
    assert detail["safe_max_deg"] == 91.6

    svg = base64.b64decode(result["dashboard"].split(",", 1)[1]).decode("utf-8")
    assert "SAVED CALIBRATION" in svg
    assert "shoulder_pan" in svg
    assert "0.00° · 2200t" in svg
    assert "-82.40° .. 91.60°" in svg


def test_connection_dashboard_labels_uncalibrated_profile_defaults():
    result = _NODE_REGISTRY["RobotConnectionDashboard"]({
        "robot": {
            "usb": {"ready": True, "recommended": {"path": "COM3"}},
            "driver": {
                "running": True,
                "name": "Custom arm",
                "joints": [{
                    "id": "joint_1",
                    "home_ticks": 2048,
                    "safe_min_deg": -90.0,
                    "safe_max_deg": 90.0,
                }],
            },
        },
        "connected": True,
        "interface_ready": True,
        "pose": {"joint_1": 5.0},
    })

    assert result["summary"]["calibrated"] is False
    assert result["summary"]["calibration_status"] == "profile defaults"
    assert result["summary"]["profile_id"] == "not selected"
    svg = base64.b64decode(result["dashboard"].split(",", 1)[1]).decode("utf-8")
    assert "PROFILE DEFAULTS" in svg
    assert "No saved calibration file" in svg
    assert "Run calibration, then Save" in svg
    assert "0.00° · 2048t" in svg
    assert "-90.00° .. 90.00°" in svg


def test_visual_robot_definition_saves_and_loads_named_profile(monkeypatch, tmp_path):
    monkeypatch.setenv("BLACKNODE_ROBOTS_DIR", str(tmp_path / "robots"))
    shoulder = _NODE_REGISTRY["RobotJointDefinition"]({
        "joint_id": "Shoulder Pan",
        "display_name": "Shoulder pan",
        "servo_id": 1,
        "min_deg": -90.0,
        "max_deg": 90.0,
    })["joint"]
    gripper = _NODE_REGISTRY["RobotJointDefinition"]({
        "joint_id": "gripper",
        "display_name": "Gripper",
        "servo_id": 6,
        "min_deg": -10.0,
        "max_deg": 60.0,
        "invert": True,
    })["joint"]
    joints = _NODE_REGISTRY["RobotJointList"]({"joint_1": shoulder, "joint_2": gripper})["joints"]
    definition = _NODE_REGISTRY["RobotDefinition"]({
        "profile_id": "My Custom Arm",
        "display_name": "My Custom Arm",
        "joints": joints,
        "transport": "rosbridge",
    })

    assert definition["valid"] is True
    assert definition["profile"]["id"] == "my_custom_arm"
    assert [joint["id"] for joint in definition["profile"]["joints"]] == ["shoulder_pan", "gripper"]
    assert "--invert \"gripper\"" in definition["driver"]["command_template"]

    saved = _NODE_REGISTRY["RobotProfileSave"]({"profile": definition["profile"]})
    assert saved["saved"] is True
    assert (tmp_path / "robots" / "my_custom_arm" / "profile.json").exists()

    loaded = _NODE_REGISTRY["RobotProfileLoad"]({
        "profile_id": "my_custom_arm",
        "topic_prefix": "/leader",
        "rate_hz": 60.0,
    })
    assert loaded["found"] is True
    assert loaded["profile"]["display_name"] == "My Custom Arm"
    assert len(loaded["driver"]["joints"]) == 2
    assert loaded["driver"]["topic_prefix"] == "/leader"
    assert loaded["driver"]["state_topic"] == "/leader/joint_states"
    assert loaded["driver"]["command_topic"] == "/leader/joint_commands"
    assert "--rate-hz 60" in loaded["driver"]["command_template"]
    assert "--state-topic /leader/joint_states" in robot_nodes._driver_command(loaded["driver"], "COM7")

    listed = _NODE_REGISTRY["RobotProfileList"]({})
    assert {item["id"] for item in listed["profiles"]} == {"so_arm101", "my_custom_arm"}


def test_robot_definition_discovers_driver_choices_and_usb_identity():
    definition_fn = _NODE_REGISTRY["RobotDefinition"]
    assert "feetech_bus_driver.py" in definition_fn._bn_input_choices["driver_script"]

    joint = _NODE_REGISTRY["RobotJointDefinition"]({"joint_id": "base", "servo_id": 1})["joint"]
    result = definition_fn({
        "profile_id": "usb_arm",
        "joints": [joint],
        "hardware": {"vendor_id": "1A86", "product_id": "55D3", "serial": "ABC"},
    })

    assert result["valid"] is True
    assert result["profile"]["match"] == {"vendor_id": "1a86", "product_id": "55d3"}
    assert "USB discovery" in result["report"]


def test_joint_list_accepts_more_than_sixteen_numbered_inputs():
    joint_2 = {"id": "second", "servo_id": 2}
    joint_25 = {"id": "twenty_fifth", "servo_id": 25}
    result = _NODE_REGISTRY["RobotJointList"]({"joint_25": joint_25, "joint_2": joint_2})

    assert result["count"] == 2
    assert [joint["id"] for joint in result["joints"]] == ["second", "twenty_fifth"]


def test_robot_profile_selector_is_a_dropdown():
    assert "so_arm101" in _NODE_REGISTRY["Robot"]._bn_input_choices["profile_id"]
    assert _NODE_REGISTRY["Robot"]._bn_primary_inputs == ["trigger"]
    assert _NODE_REGISTRY["Robot"]._bn_primary_outputs == ["robot", "report"]
    assert _NODE_REGISTRY["RobotUSBDiscovery"]._bn_hidden is True
    assert _NODE_REGISTRY["RobotProfileLoad"]._bn_hidden is True
    assert _NODE_REGISTRY["RobotDriverPreset"]._bn_hidden is True
    assert _NODE_REGISTRY["RobotDiscovery"]._bn_hidden is True


def test_robot_automatically_selects_hardware_and_checks_connection(monkeypatch):
    devices = [
        {"path": "COM3", "serial": "LEADER", "accessible": True},
        {"path": "COM7", "serial": "FOLLOWER", "accessible": True},
    ]
    monkeypatch.setattr(robot_nodes, "robot_usb_discovery", lambda _ctx: {
        "found": True, "ready": True, "port": "COM3", "serial": "LEADER",
        "devices": devices, "recommended": devices[0], "report": "found 2",
    })
    seen = {}

    def fake_connection(ctx):
        seen.update(ctx)
        return {"ready": False, "robot": {"ready": False}, "report": "driver checked"}

    monkeypatch.setattr(robot_nodes, "robot_discovery", fake_connection)

    result = _NODE_REGISTRY["Robot"]({"profile_id": "so_arm101", "selection": 1})

    assert result["hardware"]["recommended"]["serial"] == "FOLLOWER"
    assert result["hardware"]["serial"] == "FOLLOWER"
    assert result["driver"]["hardware_id"] == "FOLLOWER"
    assert seen["usb"]["recommended"]["path"] == "COM7"
    assert "selected_index: 1" in result["report"]
    assert "selected_port: COM7" in result["report"]
    assert seen["action"] == "check"


def test_robot_applies_embedded_calibration_only_to_matching_hardware(monkeypatch):
    profile = profile_nodes.builtin_profile("so_arm101")
    calibration = {
        "schema_version": 1,
        "profile_id": profile["id"],
        "hardware_id": "SERIAL-42",
        "joints": {
            joint["id"]: {
                "home_ticks": 2000 + index,
                "safe_min_deg": -50.0,
                "safe_max_deg": 50.0,
            }
            for index, joint in enumerate(profile["joints"])
        },
    }
    hardware = {
        "found": True,
        "ready": True,
        "port": "COM7",
        "serial": "SERIAL-42",
        "devices": [{"path": "COM7", "serial": "SERIAL-42", "accessible": True}],
        "recommended": {"path": "COM7", "serial": "SERIAL-42", "accessible": True},
        "report": "found",
    }
    monkeypatch.setattr(robot_nodes, "robot_usb_discovery", lambda _ctx: hardware)
    monkeypatch.setattr(robot_nodes, "robot_discovery", lambda ctx: {
        "ready": False,
        "usb_ready": True,
        "driver_running": False,
        "robot": {"ready": False, "driver": ctx["driver"]},
        "report": "driver checked",
    })

    result = _NODE_REGISTRY["Robot"]({
        "profile": profile,
        "calibration": calibration,
        "driver": {"driver": "blacknode_drivers.feetech", "run_id": "embedded-test"},
        "action": "check",
    })

    assert result["found"] is True
    assert result["calibration"]["hardware_id"] == "SERIAL-42"
    assert result["profile"]["joints"][0]["home_ticks"] == 2000
    assert result["profile"]["joints"][0]["safe_min_deg"] == -50.0
    assert result["driver"]["run_id"] == "embedded-test"
    assert result["driver"]["joints"][0]["home_ticks"] == 2000
    assert "embedded deployment calibration" in result["report"]

    rejected_calibration = dict(calibration)
    rejected_calibration["hardware_id"] = "OTHER-SERIAL"
    rejected = _NODE_REGISTRY["Robot"]({
        "profile": profile,
        "calibration": rejected_calibration,
        "action": "check",
    })

    assert rejected["found"] is False
    assert "discovery selected SERIAL-42" in rejected["report"]


def test_profile_duplicate_turns_builtin_into_editable_local_robot(monkeypatch, tmp_path):
    monkeypatch.setenv("BLACKNODE_ROBOTS_DIR", str(tmp_path / "robots"))

    result = _NODE_REGISTRY["RobotProfileDuplicate"]({
        "source_profile_id": "so_arm101",
        "new_profile_id": "workbench arm",
        "display_name": "Workbench Arm",
    })

    assert result["saved"] is True
    assert result["profile"]["id"] == "workbench_arm"
    assert result["profile"]["display_name"] == "Workbench Arm"
    assert len(result["profile"]["joints"]) == 6
    assert (tmp_path / "robots" / "workbench_arm" / "profile.json").exists()


def test_profile_load_uses_nested_discovery_hardware_identity(monkeypatch, tmp_path):
    monkeypatch.setenv("BLACKNODE_ROBOTS_DIR", str(tmp_path / "robots"))
    profile = profile_nodes.builtin_profile("so_arm101")
    profile["id"] = "nested_hardware_arm"
    _NODE_REGISTRY["RobotProfileSave"]({"profile": profile})
    calibration_path = (
        tmp_path / "robots" / "nested_hardware_arm" / "calibrations" / "serial_42.json"
    )
    calibration_path.parent.mkdir(parents=True)
    calibration_path.write_text(json.dumps({
        "profile_id": "nested_hardware_arm",
        "hardware_id": "SERIAL-42",
        "joints": {"shoulder_pan": {"home_ticks": 2200}},
    }), encoding="utf-8")

    loaded = _NODE_REGISTRY["RobotProfileLoad"]({
        "profile_id": "nested_hardware_arm",
        "hardware": {"recommended": {"serial": "SERIAL-42", "path": "COM3"}},
    })

    assert loaded["driver"]["hardware_id"] == "SERIAL-42"
    assert loaded["driver"]["joints"][0]["home_ticks"] == 2200


def test_calibration_records_extrema_home_margin_and_device_file(monkeypatch, tmp_path):
    monkeypatch.setenv("BLACKNODE_ROBOTS_DIR", str(tmp_path / "robots"))
    profile = profile_nodes.builtin_profile("so_arm101")
    profile["id"] = "calibration_arm"
    profile["joints"] = profile["joints"][:2]
    run_id = "test_calibration"
    profile_nodes.stop_calibration_services()

    started = _NODE_REGISTRY["RobotCalibrationRecorder"]({
        "action": "start",
        "run_id": run_id,
        "calibration_name": "Workbench left arm",
        "profile": profile,
        "hardware_id": "USB ABC-123",
        "pose": {"shoulder_pan": 0.0, "shoulder_lift": 0.0},
        "torque_enabled": False,
        "safety_margin_deg": 2.0,
    })
    assert started["active"] is True
    assert started["live"] is True
    assert started["state"] == "recording"

    for pose in (
        {"shoulder_pan": -20.0, "shoulder_lift": -30.0},
        {"shoulder_pan": 40.0, "shoulder_lift": 50.0},
    ):
        _NODE_REGISTRY["RobotCalibrationRecorder"]({
            "action": "_sample",
            "run_id": run_id,
            "pose": pose,
            "torque_enabled": False,
        })
    home = _NODE_REGISTRY["RobotCalibrationRecorder"]({
        "action": "capture_home",
        "run_id": run_id,
        "pose": {"shoulder_pan": 5.0, "shoulder_lift": 10.0},
        "torque_enabled": False,
    })
    assert home["home"] == {"shoulder_pan": 5.0, "shoulder_lift": 10.0}

    finished = _NODE_REGISTRY["RobotCalibrationRecorder"]({"action": "finish", "run_id": run_id})
    assert finished["saved"] is True
    assert finished["active"] is False
    assert finished["live"] is True
    assert finished["state"] == "saved"
    assert finished["calibration"]["name"] == "Workbench left arm"
    after_save = _NODE_REGISTRY["RobotCalibrationRecorder"]({
        "action": "_sample",
        "run_id": run_id,
        "pose": {"shoulder_pan": 6.0, "shoulder_lift": 11.0},
        "torque_enabled": False,
    })
    assert after_save["state"] == "saved"
    assert after_save["saved"] is True
    assert after_save["path"] == finished["path"]
    assert after_save["pose"] == {"shoulder_pan": 6.0, "shoulder_lift": 11.0}
    path = tmp_path / "robots" / "calibration_arm" / "calibrations" / "usb_abc_123.json"
    assert path.exists()
    assert json.loads(path.read_text(encoding="utf-8"))["name"] == "Workbench left arm"
    shoulder = finished["calibration"]["joints"]["shoulder_pan"]
    assert shoulder["observed_min_deg"] == -25.0
    assert shoulder["observed_max_deg"] == 35.0
    assert shoulder["safe_min_deg"] == -23.0
    assert shoulder["safe_max_deg"] == 33.0
    assert shoulder["home_ticks"] > 2048

    loaded = _NODE_REGISTRY["RobotProfileLoad"]({
        "profile_id": "calibration_arm",
        "hardware_id": "USB ABC-123",
    })
    assert loaded["found"] is True
    assert loaded["calibration"]["hardware_id"] == "USB ABC-123"
    assert loaded["driver"]["joints"][0]["safe_min_deg"] == -23.0


def test_calibration_refuses_to_record_with_torque_enabled(monkeypatch, tmp_path):
    monkeypatch.setenv("BLACKNODE_ROBOTS_DIR", str(tmp_path / "robots"))
    profile_nodes.stop_calibration_services()

    result = _NODE_REGISTRY["RobotCalibrationRecorder"]({
        "action": "start",
        "profile": profile_nodes.builtin_profile("so_arm101"),
        "hardware_id": "robot-1",
        "torque_enabled": True,
    })

    assert result["active"] is False
    assert "torque is on" in result["report"]


def test_calibration_pause_resume_preserves_samples_and_live_pose(monkeypatch, tmp_path):
    monkeypatch.setenv("BLACKNODE_ROBOTS_DIR", str(tmp_path / "robots"))
    profile_nodes.stop_calibration_services()
    profile = profile_nodes.builtin_profile("so_arm101")
    profile["joints"] = profile["joints"][:1]
    run_id = "pause_resume_calibration"

    started = _NODE_REGISTRY["RobotCalibrationRecorder"]({
        "action": "start",
        "run_id": run_id,
        "profile": profile,
        "hardware_id": "PAUSE-1",
        "pose": {"shoulder_pan": 1.0},
        "torque_enabled": False,
    })
    paused = _NODE_REGISTRY["RobotCalibrationRecorder"]({"action": "pause", "run_id": run_id})
    assert paused["state"] == "paused"
    assert paused["live"] is True
    paused_samples = paused["samples"]

    while_paused = _NODE_REGISTRY["RobotCalibrationRecorder"]({
        "action": "_sample",
        "run_id": run_id,
        "pose": {"shoulder_pan": 9.0},
        "torque_enabled": False,
    })
    assert while_paused["samples"] == paused_samples
    assert while_paused["pose"] == {"shoulder_pan": 9.0}

    resumed = _NODE_REGISTRY["RobotCalibrationRecorder"]({
        "action": "start",
        "run_id": run_id,
        "profile": profile,
        "hardware_id": "PAUSE-1",
        "pose": {"shoulder_pan": 9.0},
        "torque_enabled": False,
    })
    assert resumed["state"] == "recording"
    assert resumed["samples"] == paused_samples + 1
    assert resumed["pose"] == {"shoulder_pan": 9.0}


def test_calibration_highlights_moving_joint_and_extended_range(monkeypatch, tmp_path):
    monkeypatch.setenv("BLACKNODE_ROBOTS_DIR", str(tmp_path / "robots"))
    profile_nodes.stop_calibration_services()
    profile = profile_nodes.builtin_profile("so_arm101")
    profile["joints"] = profile["joints"][:1]
    run_id = "highlight_calibration"
    _NODE_REGISTRY["RobotCalibrationRecorder"]({
        "action": "start",
        "run_id": run_id,
        "profile": profile,
        "hardware_id": "HIGHLIGHT-1",
        "pose": {"shoulder_pan": 0.0},
        "torque_enabled": False,
    })

    result = _NODE_REGISTRY["RobotCalibrationRecorder"]({
        "action": "_sample",
        "run_id": run_id,
        "pose": {"shoulder_pan": -7.0},
        "torque_enabled": False,
    })

    assert result["capturing_joint"] == "shoulder_pan"
    assert result["range_updates"]["shoulder_pan"]["kind"] == "min"
    svg = base64.b64decode(result["dashboard"].split(",", 1)[1]).decode("utf-8")
    assert "CAPTURING shoulder_pan" in svg
    assert "MIN ↓" in svg


def test_custom_robot_templates_validate():
    templates = Path(__file__).resolve().parents[1] / "templates"
    for name in (
        "editable-so-arm101-profile.json",
        "robot-guided-calibration.json",
        "so-arm101-motion-test.json",
    ):
        workflow = json.loads((templates / name).read_text(encoding="utf-8"))
        report = validate_workflow(workflow)
        assert report.ok, (name, [issue.message for issue in report.issues])
