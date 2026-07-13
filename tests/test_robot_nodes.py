import sys
from types import SimpleNamespace

import blacknode  # noqa: F401
from blacknode.node import _NODE_REGISTRY
from blacknode.pkg.blacknode_robot import robot as robot_nodes


EXPECTED_NODES = [
    "RobotUSBDiscovery",
    "RobotDriverDescriptor",
    "RobotDriverLauncher",
    "RobotDiscovery",
    "RobotDriverPreset",
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
    assert result["recommended"]["vendor_id"] == "1a86"
    assert result["recommended"]["product_id"] == "7523"
    assert "COM7" in result["report"]
    assert "SO-ARM101 Servo Bus" in result["report"]


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
