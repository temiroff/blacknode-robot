"""Stable, hardware-neutral contracts used by hardware providers."""

from __future__ import annotations

from dataclasses import dataclass, field
import math
from typing import Any, Protocol
import time


@dataclass(frozen=True)
class JointState:
    """Canonical Blacknode joint feedback in SI units.

    ROS ``sensor_msgs/JointState`` and vendor register maps are adapter
    representations. Providers normalize them into this contract before state
    reaches workflows, telemetry, or motion safety.
    """

    positions: dict[str, float] = field(default_factory=dict)
    velocities: dict[str, float] = field(default_factory=dict)
    efforts: dict[str, float] = field(default_factory=dict)
    limits: dict[str, tuple[float, float]] = field(default_factory=dict)
    source_time: float = field(default_factory=time.time)
    receive_time: float = field(default_factory=time.time)
    frame_id: str = ""

    def __post_init__(self) -> None:
        names = set(self.positions)
        if not set(self.velocities).issubset(names):
            raise ValueError("joint velocities must refer to known positions")
        if not set(self.efforts).issubset(names):
            raise ValueError("joint efforts must refer to known positions")
        if not set(self.limits).issubset(names):
            raise ValueError("joint limits must refer to known positions")
        values = [
            *self.positions.values(),
            *self.velocities.values(),
            *self.efforts.values(),
            *(value for limits in self.limits.values() for value in limits),
            self.source_time,
            self.receive_time,
        ]
        if not all(
            isinstance(value, (int, float))
            and not isinstance(value, bool)
            and math.isfinite(float(value))
            for value in values
        ):
            raise ValueError("joint state values and timestamps must be finite")
        for lower, upper in self.limits.values():
            if float(lower) >= float(upper):
                raise ValueError("joint lower limits must be less than upper limits")

    def is_fresh(self, max_age: float, now: float | None = None) -> bool:
        current = time.time() if now is None else float(now)
        return current - self.source_time <= max(0.0, float(max_age))

    def as_dict(self) -> dict[str, Any]:
        return {
            "kind": "blacknode.joint-state",
            "schema_version": 1,
            "source_time": self.source_time,
            "receive_time": self.receive_time,
            "frame_id": self.frame_id,
            "position_unit": "radian",
            "velocity_unit": "radian/s",
            "effort_unit": "newton-metre",
            "positions": dict(self.positions),
            "velocities": dict(self.velocities),
            "efforts": dict(self.efforts),
            "limits": {
                name: {"lower": lower, "upper": upper}
                for name, (lower, upper) in self.limits.items()
            },
        }


@dataclass(frozen=True)
class FaultState:
    """Canonical active or historical device fault."""

    code: str
    message: str
    severity: str = "error"
    active: bool = True
    recoverable: bool = False
    source_time: float = field(default_factory=time.time)
    vendor_code: str = ""
    details: dict[str, Any] = field(default_factory=dict)

    def __post_init__(self) -> None:
        if not self.code.strip():
            raise ValueError("fault code must not be empty")
        if self.severity not in {"info", "warning", "error", "critical"}:
            raise ValueError(f"unsupported fault severity: {self.severity}")
        if not math.isfinite(float(self.source_time)):
            raise ValueError("fault source_time must be finite")

    def as_dict(self) -> dict[str, Any]:
        return {
            "kind": "blacknode.fault-state",
            "schema_version": 1,
            "code": self.code,
            "message": self.message,
            "severity": self.severity,
            "active": self.active,
            "recoverable": self.recoverable,
            "source_time": self.source_time,
            "vendor_code": self.vendor_code,
            "details": dict(self.details),
        }


@dataclass(frozen=True)
class MobileBaseCommand:
    """A bounded velocity command in metres/second and radians/second."""

    linear_x: float = 0.0
    linear_y: float = 0.0
    angular_z: float = 0.0
    issued_at: float = field(default_factory=time.monotonic)
    expires_at: float = 0.0

    def is_fresh(self, now: float | None = None) -> bool:
        current = time.monotonic() if now is None else now
        return self.expires_at <= 0.0 or current <= self.expires_at


@dataclass
class DeviceState:
    """Canonical state shared by drivers, workflows, telemetry, and safety."""

    device_id: str
    connected: bool = False
    armed: bool = False
    torque_enabled: bool | None = None
    capabilities: list[str] = field(default_factory=list)
    joint_state: JointState | None = None
    faults: list[FaultState] = field(default_factory=list)
    temperatures_c: dict[str, float] = field(default_factory=dict)
    voltage_v: float | None = None
    values: dict[str, Any] = field(default_factory=dict)
    error: str = ""
    updated_at: float = field(default_factory=time.time)

    def as_dict(self) -> dict[str, Any]:
        return {
            "kind": "blacknode.device-state",
            "schema_version": 1,
            "device_id": self.device_id,
            "connected": self.connected,
            "armed": self.armed,
            "torque_enabled": self.torque_enabled,
            "capabilities": list(self.capabilities),
            "joint_state": self.joint_state.as_dict() if self.joint_state else None,
            "faults": [fault.as_dict() for fault in self.faults],
            "temperatures_c": {
                name: float(value)
                for name, value in self.temperatures_c.items()
            },
            "voltage_v": self.voltage_v,
            "values": dict(self.values),
            "error": self.error,
            "updated_at": self.updated_at,
        }


class MobileBaseProvider(Protocol):
    """Provider contract implemented by local and remote mobile bases."""

    def state(self) -> DeviceState: ...

    def arm(self) -> DeviceState: ...

    def disarm(self) -> DeviceState: ...

    def command(self, command: MobileBaseCommand) -> DeviceState: ...

    def stop(self) -> DeviceState: ...
