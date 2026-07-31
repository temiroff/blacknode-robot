"""Generic joint-group contracts for real actuator adapters."""

from __future__ import annotations

from dataclasses import dataclass, field
import time
from typing import Any, Protocol


@dataclass(frozen=True)
class JointGroupCommand:
    positions: dict[str, float]
    issued_at: float = field(default_factory=time.monotonic)
    expires_at: float = 0.0

    def is_fresh(self, now: float | None = None) -> bool:
        current = time.monotonic() if now is None else now
        return self.expires_at <= 0.0 or current <= self.expires_at


@dataclass
class JointGroupState:
    device_id: str
    connected: bool = False
    armed: bool = False
    torque_enabled: bool | None = None
    torque_report_error: str = ""
    joint_names: list[str] = field(default_factory=list)
    positions: dict[str, float] = field(default_factory=dict)
    raw_positions: dict[str, int] = field(default_factory=dict)
    limits: dict[str, dict[str, float]] = field(default_factory=dict)
    temperatures_c: dict[str, float] = field(default_factory=dict)
    voltages_v: dict[str, float] = field(default_factory=dict)
    voltage_v: float | None = None
    hardware_error_flags: dict[str, int] = field(default_factory=dict)
    hardware_errors: dict[str, list[str]] = field(default_factory=dict)
    servo_status: dict[str, int] = field(default_factory=dict)
    bus: dict[str, Any] = field(default_factory=dict)
    calibrated: bool = False
    error: str = ""
    updated_at: float = field(default_factory=time.time)

    def as_dict(self) -> dict[str, Any]:
        return {
            "device_id": self.device_id,
            "connected": self.connected,
            "armed": self.armed,
            "torque_enabled": self.torque_enabled,
            "torque_report_error": self.torque_report_error,
            "joint_names": list(self.joint_names),
            "positions": dict(self.positions),
            "raw_positions": dict(self.raw_positions),
            "limits": {name: dict(values) for name, values in self.limits.items()},
            "temperatures_c": dict(self.temperatures_c),
            "voltages_v": dict(self.voltages_v),
            "voltage_v": self.voltage_v,
            "hardware_error_flags": dict(self.hardware_error_flags),
            "hardware_errors": {
                name: list(values)
                for name, values in self.hardware_errors.items()
            },
            "servo_status": dict(self.servo_status),
            "bus": dict(self.bus),
            "calibrated": self.calibrated,
            "error": self.error,
            "updated_at": self.updated_at,
        }


class JointGroupProvider(Protocol):
    def state(self) -> JointGroupState: ...

    def arm(self) -> JointGroupState: ...

    def disarm(self) -> JointGroupState: ...

    def command(self, command: JointGroupCommand) -> JointGroupState: ...

    def stop(self) -> JointGroupState: ...
