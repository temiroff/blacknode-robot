from __future__ import annotations

import pytest
import os
from pathlib import Path
import subprocess
import sys
import threading
from types import SimpleNamespace
from urllib.error import HTTPError
from urllib.request import Request, urlopen
import json

from blacknode_robot.devices import (
    DeviceState,
    FaultState,
    I2CMecanumBase,
    JointState,
    SerialJointConfig,
    SerialJointMonitor,
    SerialJointSpec,
    JointGroupCommand,
    JointGroupState,
    MobileBaseCommand,
    SafetyGate,
    SafetyLimits,
)
from blacknode_robot.devices.service import HardwareRuntime
from blacknode_robot.devices.service.server import create_server
from blacknode_robot.telemetry import RobotStateTelemetrySampler, TelemetryBus
from blacknode_robot.telemetry.adapters.mqtt import (
    MqttTelemetryPublisher,
    mqtt_config_from_env,
)
from blacknode_robot.devices.version import service_version
from blacknode_robot.devices.calibration import CalibrationError, CalibrationStore
from blacknode_robot.devices.device_config import load_device_config
from blacknode_robot.devices.auth import (
    authorization_matches,
    load_auth_token,
    save_auth_token,
    token_fingerprint,
)
from scripts.render_systemd_unit import render_unit, unit_quote, working_directory_value
from scripts.configure_devices import (
    assign_device_names,
    build_fleet_entries,
    deduplicate_serial_ports,
    device_key,
    discover_occupied_service_ports,
    hardware_unit_name,
    normalize_stack_instance,
)


def calibration_fixture() -> tuple[dict, dict]:
    profile = {
        "schema_version": 1,
        "id": "arm_profile",
        "joints": [
            {"id": "shoulder", "servo_id": 1},
            {"id": "elbow", "servo_id": 2},
        ],
    }
    calibration = {
        "schema_version": 1,
        "name": "Workshop left arm",
        "profile_id": "arm_profile",
        "hardware_id": "USB-SERIAL-42",
        "joints": {
            "shoulder": {
                "home_ticks": 2048,
                "safe_min_deg": -80.0,
                "safe_max_deg": 80.0,
            },
            "elbow": {
                "home_ticks": 2100,
                "safe_min_deg": -70.0,
                "safe_max_deg": 65.0,
            },
        },
    }
    return profile, calibration


class ConnectedProvider:
    capabilities = ("joint_group", "servo_bus", "position_feedback")

    def refresh(self):
        return JointGroupState(
            device_id="arm-device",
            connected=True,
            armed=False,
            joint_names=["servo_1", "servo_2"],
        )


def test_safety_limits_block_excess_speed():
    gate = SafetyGate(SafetyLimits(max_linear_speed=0.1))
    gate.arm()
    with pytest.raises(ValueError):
        gate.validate(MobileBaseCommand(linear_x=0.2))


def test_safety_gate_blocks_stale_commands():
    gate = SafetyGate()
    gate.arm()
    command = MobileBaseCommand(expires_at=10.0)
    with pytest.raises(TimeoutError):
        gate.validate(command, now=11.0)


def test_i2c_kinematics_can_be_checked_without_hardware():
    adapter = I2CMecanumBase()
    assert adapter._wheel_commands(MobileBaseCommand(linear_x=0.1)) == (-40, 40, -40, 40)


def test_joint_command_tracks_freshness():
    command = JointGroupCommand({"joint_1": 0.2}, expires_at=10.0)
    assert command.is_fresh(now=9.0)
    assert not command.is_fresh(now=11.0)


def test_telemetry_bus_versions_sequences_and_isolates_sink_errors():
    received = []

    class RecordingSink:
        name = "recording"

        def publish(self, envelope):
            received.append(envelope)

        def status(self):
            return {"name": self.name, "connected": True}

        def close(self):
            return None

    class BrokenSink:
        name = "broken"

        def publish(self, _envelope):
            raise RuntimeError("broker offline")

        def status(self):
            return {"name": self.name, "connected": False}

        def close(self):
            return None

    bus = TelemetryBus("arm-01", sinks=[RecordingSink(), BrokenSink()])
    first = bus.publish("robot-state", {"connected": True}, source_time=10.0)
    second = bus.publish("robot-state", {"connected": False}, source_time=11.0)

    assert first.sequence == 1
    assert second.sequence == 2
    assert second.as_dict()["kind"] == "blacknode.telemetry"
    assert second.as_dict()["schema_version"] == 1
    assert len(received) == 2
    assert bus.latest("robot-state")["payload"]["connected"] is False
    broken = next(
        sink for sink in bus.status()["sinks"] if sink["name"] == "broken"
    )
    assert broken["error"] == "broker offline"


def test_robot_state_telemetry_derives_joint_velocity():
    states = iter([
        {
            "connected": True,
            "armed": False,
            "joint_names": ["joint_1"],
            "positions": {"joint_1": 10.0},
            "updated_at": 100.0,
        },
        {
            "connected": True,
            "armed": False,
            "joint_names": ["joint_1"],
            "positions": {"joint_1": 14.0},
            "updated_at": 102.0,
        },
    ])

    class StatusSource:
        def status(self):
            return next(states)

    bus = TelemetryBus("arm-01")
    sampler = RobotStateTelemetrySampler(StatusSource(), bus)
    sampler.sample_once()
    second = sampler.sample_once()

    assert second.payload["kind"] == "blacknode.device-state"
    joint_state = second.payload["joint_state"]
    assert joint_state["kind"] == "blacknode.joint-state"
    assert joint_state["positions"]["joint_1"] == pytest.approx(
        14.0 * 3.141592653589793 / 180.0
    )
    assert joint_state["velocities"]["joint_1"] == pytest.approx(
        2.0 * 3.141592653589793 / 180.0
    )
    assert joint_state["position_unit"] == "radian"
    assert joint_state["velocity_unit"] == "radian/s"


def test_canonical_robot_state_is_versioned_and_transport_neutral():
    joint_state = JointState(
        positions={"shoulder": 0.25},
        velocities={"shoulder": 0.1},
        limits={"shoulder": (-1.0, 1.0)},
        source_time=10.0,
        receive_time=10.1,
    )
    fault = FaultState(
        code="over-temperature",
        message="servo temperature exceeded the configured limit",
        severity="critical",
        source_time=10.0,
        vendor_code="FEETECH-7",
    )
    state = DeviceState(
        device_id="arm-01",
        connected=True,
        armed=False,
        torque_enabled=False,
        capabilities=["joint_group"],
        joint_state=joint_state,
        faults=[fault],
        temperatures_c={"shoulder": 72.0},
        voltage_v=12.1,
        updated_at=10.1,
    )

    payload = state.as_dict()

    assert payload["kind"] == "blacknode.device-state"
    assert payload["schema_version"] == 1
    assert payload["joint_state"] == joint_state.as_dict()
    assert payload["faults"] == [fault.as_dict()]
    assert "topic" not in json.dumps(payload).lower()
    assert "sensor_msgs" not in json.dumps(payload)


def test_mqtt_telemetry_is_optional_and_uses_non_retained_versioned_topics(tmp_path: Path):
    assert mqtt_config_from_env("arm-01", {}) is None
    password_path = tmp_path / "mqtt.password"
    password_path.write_text("private-password\n", encoding="utf-8")
    config = mqtt_config_from_env(
        "arm/01",
        {
            "BLACKNODE_MQTT_URL": "mqtts://broker.example:8883",
            "BLACKNODE_MQTT_USERNAME": "robot",
            "BLACKNODE_MQTT_PASSWORD_FILE": str(password_path),
            "BLACKNODE_MQTT_TOPIC_PREFIX": "factory/blacknode",
            "BLACKNODE_MQTT_QOS": "1",
        },
    )
    assert config is not None

    class PublishResult:
        rc = 0

    class RecordingClient:
        def __init__(self):
            self.published = []
            self.credentials = None
            self.tls = False
            self.connected_to = None
            self.looping = False
            self.on_connect = None
            self.on_disconnect = None

        def username_pw_set(self, username, password):
            self.credentials = (username, password)

        def tls_set(self):
            self.tls = True

        def connect_async(self, host, port, keepalive):
            self.connected_to = (host, port, keepalive)

        def loop_start(self):
            self.looping = True

        def publish(self, topic, payload, qos, retain):
            self.published.append((topic, json.loads(payload), qos, retain))
            return PublishResult()

        def disconnect(self):
            return None

        def loop_stop(self):
            self.looping = False

    client = RecordingClient()
    publisher = MqttTelemetryPublisher(config, client_factory=lambda: client)
    bus = TelemetryBus("arm/01", sinks=[publisher])
    envelope = bus.publish("robot-state", {"connected": True})

    assert client.credentials == ("robot", "private-password")
    assert client.tls is True
    assert client.connected_to == ("broker.example", 8883, 30)
    topic, payload, qos, retain = client.published[0]
    assert topic == "factory/blacknode/arm%2F01/telemetry/robot-state"
    assert payload == envelope.as_dict()
    assert qos == 1
    assert retain is False
    assert "private-password" not in json.dumps(publisher.status())
    runtime = HardwareRuntime(device_id="arm/01", telemetry_bus=bus)
    assert "telemetry_v1" in runtime.service_features()
    assert "mqtt_telemetry_v1" in runtime.service_features()
    assert runtime.status()["telemetry"]["enabled"] is True
    runtime.close()
    assert client.looping is False


def test_joint_state_serializes_without_hardware():
    state = JointGroupState(device_id="arm-0", joint_names=["joint_1"])
    payload = state.as_dict()
    assert payload["device_id"] == "arm-0"
    assert payload["connected"] is False


def test_serial_joint_position_conversion_is_bounded_and_reversible():
    joint = SerialJointSpec("joint_1", servo_id=1, home_ticks=2048)
    assert round((joint.home_ticks + 512 - joint.home_ticks) * 360 / 4096, 5) == 45.0
    from blacknode_robot.devices.adapters.serial_joint import degrees_to_ticks, ticks_to_degrees
    assert degrees_to_ticks(45.0, joint) == 2560
    assert ticks_to_degrees(2560, joint) == 45.0


def test_serial_monitor_exposes_no_motion_or_torque_enable_methods():
    monitor = SerialJointMonitor(
        SerialJointConfig(port="/dev/not-opened", joints=(SerialJointSpec("servo_1", 1),))
    )
    assert not hasattr(monitor, "arm")
    assert not hasattr(monitor, "command")
    assert not hasattr(monitor, "stop")
    assert hasattr(monitor, "release_torque")


def test_serial_monitor_reads_physical_torque_and_warns_when_servos_differ(monkeypatch):
    class FakeSdk:
        COMM_SUCCESS = 0

    class FakePacket:
        def read2ByteTxRx(self, _port, servo_id, _address):
            return 2048 + servo_id, 0, 0

        def read1ByteTxRx(self, _port, servo_id, _address):
            return (1 if servo_id == 2 else 0), 0, 0

    class FakePort:
        def closePort(self):
            return None

    from blacknode_robot.devices.adapters import serial_joint

    monkeypatch.setattr(serial_joint, "load_sdk", lambda: FakeSdk())
    monkeypatch.setattr(
        serial_joint,
        "_open",
        lambda _sdk, _config: (FakePort(), FakePacket()),
    )
    monitor = SerialJointMonitor(
        SerialJointConfig(
            port="/dev/serial/by-id/test-arm",
            joints=(
                SerialJointSpec("servo_1", 1),
                SerialJointSpec("servo_2", 2),
            ),
        ),
        device_id="test-arm",
    )

    state = monitor.refresh()

    assert state.connected is True
    assert state.torque_enabled is True
    assert "mixed physical torque state" in state.error
    assert "servo_2" in state.error


def test_serial_monitor_explains_when_physical_torque_cannot_be_read(monkeypatch):
    class FakeSdk:
        COMM_SUCCESS = 0

    class FakePacket:
        def read2ByteTxRx(self, _port, servo_id, _address):
            return 2048 + servo_id, 0, 0

        def read1ByteTxRx(self, _port, servo_id, _address):
            if servo_id == 2:
                return 0, 1, 0
            return 1, 0, 0

    class FakePort:
        def closePort(self):
            return None

    from blacknode_robot.devices.adapters import serial_joint

    monkeypatch.setattr(serial_joint, "load_sdk", lambda: FakeSdk())
    monkeypatch.setattr(
        serial_joint,
        "_open",
        lambda _sdk, _config: (FakePort(), FakePacket()),
    )
    monitor = SerialJointMonitor(
        SerialJointConfig(
            port="/dev/serial/by-id/test-arm",
            joints=(
                SerialJointSpec("servo_1", 1),
                SerialJointSpec("servo_2", 2),
            ),
        ),
        device_id="test-arm",
    )

    state = monitor.refresh()

    assert state.connected is True
    assert state.torque_enabled is None
    assert state.torque_report_error == (
        "Could not read the physical torque-enable register for "
        "servo_2 (servo 2)."
    )
    assert state.as_dict()["torque_report_error"] == state.torque_report_error


def test_serial_monitor_explicitly_releases_torque_without_sending_positions(monkeypatch):
    writes = []

    class FakeSdk:
        COMM_SUCCESS = 0

    class FakePacket:
        def write1ByteTxRx(self, _port, servo_id, address, value):
            writes.append((servo_id, address, value))
            return 0, 0

    class FakePort:
        def closePort(self):
            return None

    from blacknode_robot.devices.adapters import serial_joint

    monkeypatch.setattr(serial_joint, "load_sdk", lambda: FakeSdk())
    monkeypatch.setattr(
        serial_joint,
        "_open",
        lambda _sdk, _config: (FakePort(), FakePacket()),
    )
    monitor = SerialJointMonitor(
        SerialJointConfig(
            port="/dev/serial/by-id/test-arm",
            joints=(
                SerialJointSpec("servo_1", 1),
                SerialJointSpec("servo_2", 2),
            ),
        ),
        device_id="test-arm",
    )

    state = monitor.release_torque()

    assert writes == [
        (1, serial_joint.ADDR_TORQUE_ENABLE, 0),
        (2, serial_joint.ADDR_TORQUE_ENABLE, 0),
    ]
    assert state.armed is False
    assert state.torque_enabled is False


def test_calibration_activation_is_bound_to_device_and_exact_servo_topology(tmp_path: Path):
    path = tmp_path / "active-calibration.json"
    profile, calibration = calibration_fixture()
    store = CalibrationStore(
        path,
        device_id="arm-device",
        servos=[{"id": 1, "name": "servo_1"}, {"id": 2, "name": "servo_2"}],
    )

    active = store.activate(profile, calibration)

    assert active["active"] is True
    assert active["profile_id"] == "arm_profile"
    assert active["hardware_id"] == "USB-SERIAL-42"
    reloaded = CalibrationStore(
        path,
        device_id="arm-device",
        servos=[{"id": 1, "name": "servo_1"}, {"id": 2, "name": "servo_2"}],
    )
    assert reloaded.status()["digest"] == active["digest"]

    wrong_device = CalibrationStore(
        path,
        device_id="another-device",
        servos=[{"id": 1, "name": "servo_1"}, {"id": 2, "name": "servo_2"}],
    )
    assert wrong_device.status()["active"] is False
    assert "different device" in wrong_device.status()["error"]


def test_calibration_activation_rejects_partial_or_different_servo_topology(tmp_path: Path):
    profile, calibration = calibration_fixture()
    store = CalibrationStore(
        tmp_path / "active-calibration.json",
        device_id="arm-device",
        servos=[{"id": 1, "name": "servo_1"}],
    )

    with pytest.raises(CalibrationError, match="servo IDs"):
        store.activate(profile, calibration)


def test_configuration_can_be_replaced_and_preserves_unspecified_settings(tmp_path: Path):
    config_path = tmp_path / "device.json"
    repo_dir = Path(__file__).parents[1]
    first = subprocess.run(
        [
            sys.executable,
            "-m",
            "scripts.configure_device",
            "--config",
            str(config_path),
            "--port",
            "/dev/serial/by-id/example",
            "--servos",
            "6",
            "--name",
            "Workshop left arm",
            "--device-id",
            "arm-01",
        ],
        check=False,
        capture_output=True,
        text=True,
        cwd=repo_dir,
    )
    assert first.returncode == 0, first.stderr

    second = subprocess.run(
        [
            sys.executable,
            "-m",
            "scripts.configure_device",
            "--config",
            str(config_path),
            "--servos",
            "7",
        ],
        check=False,
        capture_output=True,
        text=True,
        cwd=repo_dir,
    )
    assert second.returncode == 0, second.stderr
    config = load_device_config(config_path)
    assert config["device_id"] == "arm-01"
    assert config["name"] == "Workshop left arm"
    assert config["port"] == "/dev/serial/by-id/example"
    assert len(config["servos"]) == 7


def test_multi_robot_discovery_deduplicates_serial_aliases(monkeypatch):
    resolved = {
        "/dev/serial/by-id/robot-a": "/dev/ttyACM0",
        "/dev/ttyACM0": "/dev/ttyACM0",
        "/dev/serial/by-id/robot-b": "/dev/ttyACM1",
        "/dev/ttyACM1": "/dev/ttyACM1",
    }
    monkeypatch.setattr(os.path, "realpath", lambda value: resolved[value])

    ports = deduplicate_serial_ports(resolved)

    assert ports == [
        "/dev/serial/by-id/robot-a",
        "/dev/serial/by-id/robot-b",
    ]


def test_occupied_service_port_discovery_uses_read_only_socket_binds():
    occupied = discover_occupied_service_ports(8765, 8764)
    assert occupied == set()


def test_multi_robot_configuration_keeps_identity_and_service_ports_stable(tmp_path: Path):
    root = tmp_path / ".blacknode-hardware"
    robot_a = "/dev/serial/by-id/robot-a"
    robot_b = "/dev/serial/by-id/robot-b"
    first = build_fleet_entries(
        [(robot_a, [1, 2, 3, 4, 5, 6])],
        root=root,
        hostname="workshop",
        base_port=8765,
    )
    assert first[0]["device_id"].startswith("workshop-")
    assert first[0]["name"] == "Robot 1"
    assert first[0]["service_port"] == 8765
    assert first[0]["key"] == device_key(robot_a)
    assert first[0]["config"].endswith("device.json")

    second = build_fleet_entries(
        [(robot_b, [1, 2]), (robot_a, [1, 2, 3, 4, 5, 6])],
        root=root,
        hostname="workshop",
        base_port=8765,
        previous=first,
    )
    by_serial = {entry["serial_port"]: entry for entry in second}
    assert by_serial[robot_a]["device_id"] == first[0]["device_id"]
    assert by_serial[robot_a]["service_port"] == 8765
    assert by_serial[robot_b]["service_port"] == 8767
    assert by_serial[robot_a]["unit"] != by_serial[robot_b]["unit"]


def test_isolated_stack_namespaces_hardware_units_and_avoids_reserved_ports(tmp_path: Path):
    entries = build_fleet_entries(
        [("/dev/serial/by-id/robot-c", [1, 2])],
        root=tmp_path / ".blacknode-hardware",
        hostname="workshop",
        base_port=8765,
        reserved_ports={8765, 8766, 8767, 8768},
        stack_instance="instance-2",
    )

    assert entries[0]["service_port"] == 8769
    assert entries[0]["unit"].startswith("blacknode-hardware-instance-2-")
    assert entries[0]["unit"].endswith(".service")
    assert hardware_unit_name("arm-1234", "instance-2") == (
        "blacknode-hardware-instance-2-arm-1234.service"
    )
    assert normalize_stack_instance("") == ""
    with pytest.raises(ValueError, match="lowercase"):
        normalize_stack_instance("instance-2;unsafe")


def test_multi_robot_configuration_migrates_runtime_port_collision(tmp_path: Path):
    root = tmp_path / ".blacknode-hardware"
    robot_a = "/dev/serial/by-id/robot-a"
    robot_b = "/dev/serial/by-id/robot-b"
    previous = build_fleet_entries(
        [(robot_a, [1, 2]), (robot_b, [1, 2])],
        root=root,
        hostname="workshop",
        base_port=8765,
        reserved_ports=set(),
    )
    assert [entry["service_port"] for entry in previous] == [8765, 8766]

    migrated = build_fleet_entries(
        [(robot_a, [1, 2]), (robot_b, [1, 2])],
        root=root,
        hostname="workshop",
        base_port=8765,
        previous=previous,
        reserved_ports={8766},
    )

    assert [entry["service_port"] for entry in migrated] == [8765, 8767]


def test_multi_robot_names_can_be_assigned_and_are_preserved(tmp_path: Path):
    root = tmp_path / ".blacknode-hardware"
    robot_a = "/dev/serial/by-id/robot-a"
    robot_b = "/dev/serial/by-id/robot-b"
    entries = build_fleet_entries(
        [(robot_a, [1, 2]), (robot_b, [1, 2])],
        root=root,
        hostname="workshop",
        base_port=8765,
    )

    assign_device_names(entries, ["Left arm", "Packing arm"], prompt=False)

    assert [entry["name"] for entry in entries] == ["Left arm", "Packing arm"]
    rescanned = build_fleet_entries(
        [(robot_b, [1, 2]), (robot_a, [1, 2])],
        root=root,
        hostname="workshop",
        base_port=8765,
        previous=entries,
    )
    assert {
        entry["serial_port"]: entry["name"] for entry in rescanned
    } == {
        robot_a: "Left arm",
        robot_b: "Packing arm",
    }


@pytest.mark.skipif(os.name == "nt", reason="systemd units use POSIX paths")
def test_systemd_unit_uses_validated_config_and_failure_restart(tmp_path: Path):
    repo_dir = tmp_path / "blacknode-hardware"
    config_path = repo_dir / ".blacknode-hardware" / "device.json"
    token_path = repo_dir / ".blacknode-hardware" / "auth.token"
    unit = render_unit(
        repo=repo_dir,
        user="alex",
        host="0.0.0.0",
        port=8765,
        config=config_path,
        auth_token_file=token_path,
    )
    assert "User=alex" in unit
    assert "WorkingDirectory=" in unit
    assert 'WorkingDirectory="' not in unit
    assert "ExecStartPre=" in unit
    assert f"--config {unit_quote(str(config_path.resolve()))} --show" in unit
    assert f"--auth-token-file {unit_quote(str(token_path.resolve()))}" in unit
    assert "--require-auth" in unit
    assert (
        f"EnvironmentFile=-{working_directory_value(str(config_path.parent / 'mqtt.env'))}"
        in unit
    )
    assert "Restart=on-failure" in unit
    assert "WantedBy=multi-user.target" in unit


def test_systemd_working_directory_is_unquoted_absolute_path():
    value = working_directory_value("/home/alex/blacknode-hardware")
    assert value == "/home/alex/blacknode-hardware"
    assert not value.startswith('"')


def test_pairing_token_is_private_and_timing_safe(tmp_path: Path):
    token_path, token = save_auth_token(tmp_path / "auth.token")
    assert load_auth_token(token_path) == token
    assert len(token) >= 32
    assert len(token_fingerprint(token)) == 12
    assert authorization_matches(f"Bearer {token}", token)
    assert authorization_matches(f"bearer {token}", token)
    assert not authorization_matches("Bearer wrong", token)
    assert not authorization_matches(None, token)
    if os.name != "nt":
        assert token_path.stat().st_mode & 0o777 == 0o600


def test_service_reports_unconfigured_hardware_honestly():
    runtime = HardwareRuntime(device_id="pi-device")
    assert runtime.status() == {
        "device_id": "pi-device",
        "software_version": service_version(),
        "service_features": ["torque_release_v1"],
        "connected": False,
        "armed": False,
        "capabilities": [],
        "error": "no hardware adapter configured",
    }
    assert runtime.capabilities()["connected"] is False


def test_service_reports_the_exact_serial_port_for_deployment_binding():
    monitor = SerialJointMonitor(
        SerialJointConfig(
            port="/dev/serial/by-id/leader-arm",
            joints=(SerialJointSpec("servo_1", 1),),
        ),
        device_id="leader-arm",
    )
    runtime = HardwareRuntime(monitor, device_id="leader-arm")

    assert runtime.status()["connection"] == {
        "transport": "serial",
        "port": "/dev/serial/by-id/leader-arm",
    }


def test_hardware_monitor_releases_and_resumes_for_a_deployment(monkeypatch):
    monitor = SerialJointMonitor(
        SerialJointConfig(
            port="/dev/serial/by-id/follower-arm",
            joints=(SerialJointSpec("servo_1", 1),),
        ),
        device_id="follower-arm",
    )
    refreshed = []
    closed = []
    monkeypatch.setattr(
        monitor,
        "refresh",
        lambda: refreshed.append(True) or monitor.state(),
    )
    monkeypatch.setattr(monitor, "close", lambda: closed.append(True))
    runtime = HardwareRuntime(monitor, device_id="follower-arm")

    released = runtime.call("release")
    assert released["status"]["leased_to_deployment"] is True
    assert released["status"]["connected"] is False
    assert released["status"]["torque_enabled"] is None
    assert "running deployment owns" in released["status"]["torque_report_error"]
    assert closed == [True]
    assert refreshed == []

    resumed = runtime.call("resume")
    assert resumed["status"]["leased_to_deployment"] is False
    assert refreshed == [True, True]


def test_leased_hardware_reports_serial_path_presence_without_opening_port(tmp_path: Path):
    if os.name == "nt":
        pytest.skip("POSIX serial device paths are verified on POSIX hosts")
    serial_path = tmp_path / "robot-serial"
    serial_path.touch()

    class Provider:
        capabilities = ("servo_bus",)
        config = SimpleNamespace(port=str(serial_path))

        def close(self):
            return None

        def disarm(self):
            return None

    runtime = HardwareRuntime(Provider(), device_id="follower")
    connected = runtime.call("release")["status"]

    assert connected["connected"] is True
    assert connected["connection_present"] is True
    assert connected["connection_reported"] is True
    assert connected["connection_source"] == "device_path"

    serial_path.unlink()
    disconnected = runtime.status()
    assert disconnected["connected"] is False
    assert disconnected["connection_present"] is False


def test_hardware_runtime_requires_explicit_torque_release_and_rejects_active_lease():
    class Provider:
        capabilities = ("joint_group",)

        def __init__(self):
            self.releases = 0

        def state(self):
            return {
                "connected": True,
                "armed": False,
                "torque_enabled": True,
            }

        def refresh(self):
            return self.state()

        def release_torque(self):
            self.releases += 1
            return self.state()

    provider = Provider()
    runtime = HardwareRuntime(provider, device_id="leader")

    assert runtime.call("disable_torque")["ok"] is True
    assert provider.releases == 1
    runtime._leased_to_deployment = True
    with pytest.raises(RuntimeError, match="stop the active deployment"):
        runtime.call("disable_torque")
    assert provider.releases == 1


def test_resume_reconnects_writable_provider_and_keeps_it_disarmed():
    class State:
        connected = False
        armed = True

        def as_dict(self):
            return {
                "device_id": "arm",
                "connected": self.connected,
                "armed": self.armed,
            }

    class Provider:
        capabilities = ("joint_group",)

        def __init__(self):
            self.value = State()
            self.calls = []

        def state(self):
            return self.value

        def connect(self):
            self.calls.append("connect")
            self.value.connected = True
            return self.value

        def disarm(self):
            self.calls.append("disarm")
            self.value.armed = False
            return self.value

        def refresh(self):
            self.calls.append("refresh")
            return self.value

    provider = Provider()
    runtime = HardwareRuntime(provider, device_id="arm")

    resumed = runtime.call("resume")

    assert resumed["ok"] is True
    assert resumed["status"]["connected"] is True
    assert resumed["status"]["armed"] is False
    assert provider.calls == ["connect", "disarm", "refresh"]


def test_service_health_and_status_endpoints():
    server = create_server(HardwareRuntime(device_id="test-device"), port=0)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    base = f"http://127.0.0.1:{server.server_port}"
    try:
        with urlopen(f"{base}/health") as response:
            health = json.loads(response.read())
        assert health["ok"] is True
        assert health["software_version"] == service_version()
        assert "torque_release_v1" in health["features"]
        with urlopen(f"{base}/status") as response:
            status = json.loads(response.read())
        assert status["device_id"] == "test-device"
        assert status["software_version"] == service_version()
        assert "torque_release_v1" in status["service_features"]
    finally:
        server.shutdown()
        server.server_close()


def test_paired_service_keeps_health_public_and_protects_device_data():
    token = "a" * 43
    server = create_server(HardwareRuntime(device_id="paired-device"), port=0, auth_token=token)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    base = f"http://127.0.0.1:{server.server_port}"
    try:
        with urlopen(f"{base}/health") as response:
            health = json.loads(response.read())
        assert health["auth_required"] is True
        assert "calibration_activation" in health["features"]

        with pytest.raises(HTTPError) as missing:
            urlopen(f"{base}/status")
        assert missing.value.code == 401

        request = Request(
            f"{base}/status",
            headers={"Authorization": f"Bearer {token}"},
        )
        with urlopen(request) as response:
            status = json.loads(response.read())
        assert status["device_id"] == "paired-device"
    finally:
        server.shutdown()
        server.server_close()


def test_paired_service_activates_calibration_and_reports_it_in_status(tmp_path: Path):
    token = "a" * 43
    profile, calibration = calibration_fixture()
    store = CalibrationStore(
        tmp_path / "active-calibration.json",
        device_id="arm-device",
        servos=[{"id": 1, "name": "servo_1"}, {"id": 2, "name": "servo_2"}],
    )
    runtime = HardwareRuntime(
        ConnectedProvider(),
        device_id="arm-device",
        calibration_store=store,
    )
    server = create_server(runtime, port=0, auth_token=token)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    base = f"http://127.0.0.1:{server.server_port}"
    try:
        unauthenticated = Request(
            f"{base}/calibration",
            data=json.dumps({"profile": profile, "calibration": calibration}).encode(),
            method="POST",
            headers={"Content-Type": "application/json"},
        )
        with pytest.raises(HTTPError) as missing:
            urlopen(unauthenticated)
        assert missing.value.code == 401

        activate = Request(
            f"{base}/calibration",
            data=json.dumps({"profile": profile, "calibration": calibration}).encode(),
            method="POST",
            headers={
                "Authorization": f"Bearer {token}",
                "Content-Type": "application/json",
            },
        )
        with urlopen(activate) as response:
            result = json.loads(response.read())
        assert result["calibration"]["active"] is True
        assert result["calibration"]["name"] == "Workshop left arm"

        status_request = Request(
            f"{base}/status",
            headers={"Authorization": f"Bearer {token}"},
        )
        with urlopen(status_request) as response:
            status = json.loads(response.read())
        assert status["calibrated"] is True
        assert status["calibration"]["profile_id"] == "arm_profile"
        assert status["calibration"]["hardware_id"] == "USB-SERIAL-42"
    finally:
        server.shutdown()
        server.server_close()


def test_service_check_distinguishes_service_health_from_hardware_readiness():
    server = create_server(HardwareRuntime(device_id="test-device"), port=0)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    repo_dir = Path(__file__).parents[1]
    command = [
        sys.executable,
        "-m",
        "scripts.service_check",
        "--url",
        f"http://127.0.0.1:{server.server_port}",
    ]
    try:
        service_only = subprocess.run(
            command,
            check=False,
            capture_output=True,
            text=True,
            cwd=repo_dir,
        )
        hardware_required = subprocess.run(
            [*command, "--require-hardware"],
            check=False,
            capture_output=True,
            text=True,
            cwd=repo_dir,
        )
        assert service_only.returncode == 0
        assert "[WARN] Hardware: not connected" in service_only.stdout
        assert hardware_required.returncode == 2
    finally:
        server.shutdown()
        server.server_close()


def test_service_check_uses_saved_pairing_token(tmp_path: Path):
    token_path, token = save_auth_token(tmp_path / "auth.token")
    server = create_server(
        HardwareRuntime(device_id="paired-device"),
        port=0,
        auth_token=token,
    )
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    repo_dir = Path(__file__).parents[1]
    base_command = [
        sys.executable,
        "-m",
        "scripts.service_check",
        "--url",
        f"http://127.0.0.1:{server.server_port}",
    ]
    try:
        missing = subprocess.run(
            base_command,
            check=False,
            capture_output=True,
            text=True,
            cwd=repo_dir,
        )
        paired = subprocess.run(
            [*base_command, "--token-file", str(token_path)],
            check=False,
            capture_output=True,
            text=True,
            cwd=repo_dir,
        )
        assert missing.returncode == 1
        assert "pairing token required" in missing.stdout
        assert paired.returncode == 0
        assert "[OK] Authentication: pairing token accepted" in paired.stdout
    finally:
        server.shutdown()
        server.server_close()
