"""Configurable I2C four-wheel mecanum-base adapter."""

from __future__ import annotations

from dataclasses import dataclass
import time
from typing import Any

from ..contracts import DeviceState, MobileBaseCommand
from ..safety import SafetyGate, SafetyLimits


@dataclass(frozen=True)
class I2CMecanumConfig:
    bus: int = 1
    address: int = 0x7A
    motor_register_start: int = 31
    motor_polarity: tuple[int, int, int, int] = (-1, 1, -1, 1)
    max_linear_speed: float = 0.25
    max_angular_speed: float = 0.75
    command_timeout: float = 0.5


class I2CMecanumBase:
    """Drive a four-wheel mecanum base through an SMBus-compatible device.

    ``bus`` can be injected for tests. ``smbus2`` is imported only when
    ``connect`` is called, so package discovery works without I2C dependencies.
    """

    def __init__(
        self,
        device_id: str = "i2c-mecanum-base",
        config: I2CMecanumConfig | None = None,
        *,
        bus: Any | None = None,
    ) -> None:
        self.device_id = device_id
        self.config = config or I2CMecanumConfig()
        self._bus = bus
        self._owns_bus = False
        self._last_command_at = 0.0
        self._state = DeviceState(
            device_id=device_id,
            connected=bus is not None,
            armed=False,
            capabilities=["mobile_base"],
        )
        self.gate = SafetyGate(SafetyLimits(
            max_linear_speed=self.config.max_linear_speed,
            max_angular_speed=self.config.max_angular_speed,
            command_timeout=self.config.command_timeout,
        ))

    def connect(self) -> DeviceState:
        if self._bus is None:
            try:
                from smbus2 import SMBus
            except ImportError as exc:
                raise RuntimeError("install smbus2 to use the I2C hardware adapter") from exc
            self._bus = SMBus(self.config.bus)
            self._owns_bus = True
        self._state.connected = True
        self._state.error = ""
        self._state.updated_at = time.time()
        return self._state

    def close(self) -> DeviceState:
        self.stop()
        self.disarm()
        if self._owns_bus and self._bus is not None and hasattr(self._bus, "close"):
            self._bus.close()
        self._bus = None
        self._owns_bus = False
        self._state.connected = False
        self._state.updated_at = time.time()
        return self._state

    def state(self) -> DeviceState:
        return self._state

    def arm(self) -> DeviceState:
        if not self._state.connected:
            raise ConnectionError("hardware device is not connected")
        self.gate.arm()
        self._state.armed = True
        self._state.updated_at = time.time()
        return self._state

    def disarm(self) -> DeviceState:
        self.stop()
        self.gate.disarm()
        self._state.armed = False
        self._state.updated_at = time.time()
        return self._state

    def command(self, command: MobileBaseCommand) -> DeviceState:
        self.gate.validate(command)
        if self._bus is None or not self._state.connected:
            raise ConnectionError("hardware device is not connected")
        wheels = self._wheel_commands(command)
        try:
            for index, value in enumerate(wheels):
                self._write_motor(index, value)
        except Exception as exc:
            self._safe_stop_after_error()
            self._state.error = f"motor write failed: {exc}"
            raise
        self._last_command_at = time.monotonic()
        self._state.values["mobile_base_command"] = {
            "linear_x": command.linear_x,
            "linear_y": command.linear_y,
            "angular_z": command.angular_z,
            "wheels": list(wheels),
        }
        self._state.updated_at = time.time()
        return self._state

    def stop(self) -> DeviceState:
        if self._bus is not None and self._state.connected:
            for index in range(4):
                try:
                    self._write_motor(index, 0)
                except Exception as exc:
                    self._state.error = f"stop write failed: {exc}"
        self._state.values["mobile_base_command"] = {
            "linear_x": 0.0,
            "linear_y": 0.0,
            "angular_z": 0.0,
            "wheels": [0, 0, 0, 0],
        }
        self._state.updated_at = time.time()
        return self._state

    def watchdog_tick(self, now: float | None = None) -> DeviceState:
        """Stop the base when its command heartbeat becomes stale."""
        if self._last_command_at <= 0.0:
            return self._state
        current = time.monotonic() if now is None else now
        if current - self._last_command_at > self.config.command_timeout:
            self.stop()
        return self._state

    def _wheel_commands(self, command: MobileBaseCommand) -> tuple[int, int, int, int]:
        # linear_x is forward, linear_y is lateral, angular_z is rotation.
        x = command.linear_x / self.config.max_linear_speed * 100.0
        y = command.linear_y / self.config.max_linear_speed * 100.0
        rotation = command.angular_z / self.config.max_angular_speed * 100.0
        raw = (x + y - rotation, x - y + rotation, x - y - rotation, x + y + rotation)
        peak = max(100.0, *(abs(value) for value in raw))
        normalized = tuple(int(round(value * 100.0 / peak)) for value in raw)
        return tuple(
            max(-100, min(100, value * self.config.motor_polarity[index]))
            for index, value in enumerate(normalized)
        )

    def _write_motor(self, index: int, speed: int) -> None:
        if self._bus is None:
            raise ConnectionError("hardware device is not connected")
        register = self.config.motor_register_start + index
        encoded = speed & 0xFF
        if not hasattr(self._bus, "write_i2c_block_data"):
            raise TypeError("bus provider must implement write_i2c_block_data")
        self._bus.write_i2c_block_data(self.config.address, register, [encoded])

    def _safe_stop_after_error(self) -> None:
        for index in range(4):
            try:
                self._write_motor(index, 0)
            except Exception:
                pass
