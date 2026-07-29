"""Stable, hardware-neutral contracts used by hardware providers."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Protocol
import time


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
    """Normalized state shared by providers and the Blacknode runtime."""

    device_id: str
    connected: bool = False
    armed: bool = False
    capabilities: list[str] = field(default_factory=list)
    values: dict[str, Any] = field(default_factory=dict)
    error: str = ""
    updated_at: float = field(default_factory=time.time)

    def as_dict(self) -> dict[str, Any]:
        return {
            "device_id": self.device_id,
            "connected": self.connected,
            "armed": self.armed,
            "capabilities": list(self.capabilities),
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
