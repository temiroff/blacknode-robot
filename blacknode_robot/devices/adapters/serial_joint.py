"""Real serial bus adapter for position-controlled joint groups."""

from __future__ import annotations

from dataclasses import dataclass, field
import threading
import time
from typing import Any

from ..joint_group import JointGroupCommand, JointGroupState


TICKS_PER_REV = 4096
ADDR_TORQUE_ENABLE = 40
ADDR_GOAL_POSITION = 42
ADDR_PRESENT_POSITION = 56


@dataclass(frozen=True)
class SerialJointSpec:
    name: str
    servo_id: int
    min_deg: float = -180.0
    max_deg: float = 180.0
    home_ticks: int = 2048
    invert: bool = False

    def __post_init__(self) -> None:
        if not 1 <= self.servo_id <= 253:
            raise ValueError(f"servo id must be between 1 and 253: {self.servo_id}")
        if self.min_deg >= self.max_deg:
            raise ValueError(f"minimum must be below maximum for {self.name}")
        if not 0 <= self.home_ticks < TICKS_PER_REV:
            raise ValueError(f"home ticks must be between 0 and {TICKS_PER_REV - 1}")


@dataclass(frozen=True)
class SerialJointConfig:
    port: str
    baudrate: int = 1_000_000
    joints: tuple[SerialJointSpec, ...] = ()


def ticks_to_degrees(ticks: int, joint: SerialJointSpec) -> float:
    value = (int(ticks) - joint.home_ticks) * 360.0 / TICKS_PER_REV
    return -value if joint.invert else value


def degrees_to_ticks(degrees: float, joint: SerialJointSpec) -> int:
    value = -float(degrees) if joint.invert else float(degrees)
    ticks = joint.home_ticks + round(value * TICKS_PER_REV / 360.0)
    return max(0, min(TICKS_PER_REV - 1, ticks))


def load_sdk() -> Any:
    try:
        import scservo_sdk
    except Exception as exc:
        raise RuntimeError("install feetech-servo-sdk to use the serial joint adapter") from exc
    return scservo_sdk


def _open(sdk: Any, config: SerialJointConfig) -> tuple[Any, Any]:
    if not config.port:
        raise ValueError("serial port is required")
    port = sdk.PortHandler(config.port)
    if not port.openPort() or not port.setBaudRate(config.baudrate):
        try:
            port.closePort()
        except Exception:
            pass
        raise RuntimeError(f"could not open {config.port} at {config.baudrate} baud")
    return port, sdk.PacketHandler(0)


def read_position(sdk: Any, packet: Any, port: Any, servo_id: int) -> int | None:
    try:
        ticks, comm_result, servo_error = packet.read2ByteTxRx(port, servo_id, ADDR_PRESENT_POSITION)
    except Exception:
        return None
    if comm_result != sdk.COMM_SUCCESS or servo_error != 0:
        return None
    return int(ticks)


def read_torque_enabled(
    sdk: Any,
    packet: Any,
    port: Any,
    servo_id: int,
) -> bool | None:
    """Read the physical torque-enable register without sending actuator writes."""
    try:
        value, comm_result, servo_error = packet.read1ByteTxRx(
            port,
            servo_id,
            ADDR_TORQUE_ENABLE,
        )
    except Exception:
        return None
    if comm_result != sdk.COMM_SUCCESS or servo_error != 0:
        return None
    return bool(value)

def write_torque_enabled(
    sdk: Any,
    packet: Any,
    port: Any,
    servo_id: int,
    enabled: bool,
) -> bool:
    """Write only the physical torque-enable register for one servo."""
    try:
        comm_result, servo_error = packet.write1ByteTxRx(
            port,
            servo_id,
            ADDR_TORQUE_ENABLE,
            1 if enabled else 0,
        )
    except Exception:
        return False
    return comm_result == sdk.COMM_SUCCESS and servo_error == 0


def probe_serial(config: SerialJointConfig) -> dict[str, Any]:
    """Read present positions only. This function performs no writes."""
    sdk = load_sdk()
    port, packet = _open(sdk, config)
    readings: dict[str, dict[str, Any]] = {}
    errors: list[str] = []
    try:
        for joint in config.joints:
            ticks = read_position(sdk, packet, port, joint.servo_id)
            if ticks is None:
                errors.append(f"no position response from servo {joint.servo_id} ({joint.name})")
                continue
            readings[joint.name] = {
                "servo_id": joint.servo_id,
                "ticks": ticks,
                "degrees": ticks_to_degrees(ticks, joint),
            }
    finally:
        port.closePort()
    return {"ok": bool(readings), "readings": readings, "errors": errors}


class SerialJointMonitor:
    """Position monitor with one explicit torque-release safety operation."""

    capabilities = ("joint_group", "servo_bus", "position_feedback")

    def __init__(self, config: SerialJointConfig, device_id: str = "device") -> None:
        if not config.joints:
            raise ValueError("at least one joint is required")
        self.config = config
        self._sdk: Any | None = None
        self._port: Any | None = None
        self._packet: Any | None = None
        self._lock = threading.Lock()
        self._state = JointGroupState(
            device_id=device_id,
            connected=False,
            armed=False,
            joint_names=[joint.name for joint in config.joints],
            positions={},
            raw_positions={},
            limits={joint.name: {"min": joint.min_deg, "max": joint.max_deg} for joint in config.joints},
            calibrated=False,
        )

    def connect(self) -> JointGroupState:
        return self.refresh()

    def refresh(self) -> JointGroupState:
        with self._lock:
            self._state.updated_at = time.time()
            if not self._port:
                try:
                    self._sdk = load_sdk()
                    self._port, self._packet = _open(self._sdk, self.config)
                except Exception as exc:
                    self._close_port()
                    self._state.connected = False
                    self._state.error = str(exc)
                    return self._state

            positions: dict[str, float] = {}
            raw_positions: dict[str, int] = {}
            torque_states: dict[str, bool] = {}
            torque_unavailable: list[str] = []
            errors: list[str] = []
            for joint in self.config.joints:
                ticks = read_position(self._sdk, self._packet, self._port, joint.servo_id)
                if ticks is None:
                    errors.append(f"no response from servo {joint.servo_id} ({joint.name})")
                    continue
                raw_positions[joint.name] = ticks
                positions[joint.name] = ticks_to_degrees(ticks, joint)
                torque_enabled = read_torque_enabled(
                    self._sdk,
                    self._packet,
                    self._port,
                    joint.servo_id,
                )
                if torque_enabled is not None:
                    torque_states[joint.name] = torque_enabled
                else:
                    torque_unavailable.append(
                        f"{joint.name} (servo {joint.servo_id})"
                    )

            self._state.positions = positions
            self._state.raw_positions = raw_positions
            self._state.connected = len(positions) == len(self.config.joints)
            self._state.torque_enabled = (
                any(torque_states.values())
                if len(torque_states) == len(self.config.joints)
                else None
            )
            self._state.torque_report_error = (
                "Could not read the physical torque-enable register for "
                + ", ".join(torque_unavailable)
                + "."
                if torque_unavailable
                else ""
            )
            if (
                len(torque_states) == len(self.config.joints)
                and len(set(torque_states.values())) > 1
            ):
                enabled = ", ".join(
                    name for name, value in torque_states.items() if value
                )
                errors.append(
                    f"mixed physical torque state; torque is enabled for: {enabled}"
                )
            self._state.error = "; ".join(errors)
            if not positions:
                self._close_port()
            return self._state

    def state(self) -> JointGroupState:
        return self._state

    def release_torque(self) -> JointGroupState:
        """Disable holding torque without sending positions or enabling motion."""
        with self._lock:
            if not self._port:
                self._sdk = load_sdk()
                self._port, self._packet = _open(self._sdk, self.config)
            failed = [
                joint
                for joint in self.config.joints
                if not write_torque_enabled(
                    self._sdk,
                    self._packet,
                    self._port,
                    joint.servo_id,
                    False,
                )
            ]
            if failed:
                names = ", ".join(
                    f"{joint.name} (servo {joint.servo_id})" for joint in failed
                )
                raise RuntimeError(f"could not disable torque for {names}")
            self._state.armed = False
            self._state.torque_enabled = False
            self._state.torque_report_error = ""
            self._state.updated_at = time.time()
            return self._state

    def close(self) -> None:
        with self._lock:
            self._close_port()
            self._state.connected = False
            self._state.updated_at = time.time()

    def _close_port(self) -> None:
        if self._port:
            try:
                self._port.closePort()
            except Exception:
                pass
        self._port = None
        self._packet = None
        self._sdk = None


class SerialJointGroup:
    """Real serial actuator provider with torque-safe lifecycle behavior."""

    def __init__(self, config: SerialJointConfig) -> None:
        if not config.joints:
            raise ValueError("at least one joint is required")
        self.config = config
        self._sdk: Any | None = None
        self._port: Any | None = None
        self._packet: Any | None = None
        self._state = JointGroupState(
            device_id=config.port,
            connected=False,
            armed=False,
            joint_names=[joint.name for joint in config.joints],
            positions={},
            limits={joint.name: {"min": joint.min_deg, "max": joint.max_deg} for joint in config.joints},
        )

    def connect(self) -> JointGroupState:
        self._sdk = load_sdk()
        self._port, self._packet = _open(self._sdk, self.config)
        self._state.connected = True
        self.refresh()
        return self._state

    def refresh(self) -> JointGroupState:
        if not self._sdk or not self._port or not self._packet:
            raise ConnectionError("serial joint adapter is not connected")
        readings: dict[str, float] = {}
        for joint in self.config.joints:
            ticks = read_position(self._sdk, self._packet, self._port, joint.servo_id)
            if ticks is None:
                raise RuntimeError(f"could not read position for {joint.name} (servo {joint.servo_id})")
            readings[joint.name] = ticks_to_degrees(ticks, joint)
        self._state.positions = readings
        self._state.updated_at = time.time()
        return self._state

    def state(self) -> JointGroupState:
        return self._state

    def arm(self) -> JointGroupState:
        if not self._sdk or not self._port or not self._packet:
            raise ConnectionError("serial joint adapter is not connected")
        current_ticks: dict[int, int] = {}
        for joint in self.config.joints:
            ticks = read_position(self._sdk, self._packet, self._port, joint.servo_id)
            if ticks is None:
                self.disarm()
                raise RuntimeError(f"could not read current position for {joint.name}")
            current_ticks[joint.servo_id] = ticks
        try:
            for joint in self.config.joints:
                self._write_goal(joint.servo_id, current_ticks[joint.servo_id])
            for joint in self.config.joints:
                self._set_torque(joint.servo_id, True)
        except Exception:
            self.disarm()
            raise
        self._state.armed = True
        self._state.torque_enabled = True
        self._state.torque_report_error = ""
        self.refresh()
        return self._state

    def disarm(self) -> JointGroupState:
        if self._sdk and self._port and self._packet:
            for joint in self.config.joints:
                try:
                    self._set_torque(joint.servo_id, False)
                except Exception:
                    pass
        self._state.armed = False
        self._state.torque_enabled = False
        self._state.torque_report_error = ""
        self._state.updated_at = time.time()
        return self._state

    def stop(self) -> JointGroupState:
        return self.disarm()

    def command(self, command: JointGroupCommand) -> JointGroupState:
        if not self._state.armed:
            raise PermissionError("joint motion is disarmed")
        if not command.is_fresh():
            raise TimeoutError("joint command is stale")
        if not self._sdk or not self._port or not self._packet:
            raise ConnectionError("serial joint adapter is not connected")
        joints = {joint.name: joint for joint in self.config.joints}
        for name, degrees in command.positions.items():
            joint = joints.get(name)
            if joint is None:
                raise ValueError(f"unknown joint: {name}")
            if not joint.min_deg <= degrees <= joint.max_deg:
                raise ValueError(f"{name} is outside its configured limits")
            self._write_goal(joint.servo_id, degrees_to_ticks(degrees, joint))
        self.refresh()
        return self._state

    def close(self) -> None:
        self.disarm()
        if self._port:
            self._port.closePort()
        self._port = None
        self._packet = None
        self._sdk = None
        self._state.connected = False

    def _write_goal(self, servo_id: int, ticks: int) -> None:
        result, error = self._packet.write2ByteTxRx(self._port, servo_id, ADDR_GOAL_POSITION, ticks)
        if result != self._sdk.COMM_SUCCESS or error != 0:
            raise RuntimeError(f"goal write failed for servo {servo_id}")

    def _set_torque(self, servo_id: int, enabled: bool) -> None:
        if not write_torque_enabled(
            self._sdk,
            self._packet,
            self._port,
            servo_id,
            enabled,
        ):
            raise RuntimeError(f"torque {'enable' if enabled else 'disable'} failed for servo {servo_id}")
