"""Safety checks shared by every motion provider."""

from __future__ import annotations

from dataclasses import dataclass

from .contracts import MobileBaseCommand


@dataclass(frozen=True)
class SafetyLimits:
    max_linear_speed: float = 0.25
    max_angular_speed: float = 0.75
    command_timeout: float = 0.5


class SafetyGate:
    """Authorization and bounded-command gate; disarmed until explicitly armed."""

    def __init__(self, limits: SafetyLimits | None = None) -> None:
        self.limits = limits or SafetyLimits()
        self._armed = False

    @property
    def armed(self) -> bool:
        return self._armed

    def arm(self) -> None:
        self._armed = True

    def disarm(self) -> None:
        self._armed = False

    def validate(self, command: MobileBaseCommand, *, now: float | None = None) -> None:
        if not self._armed:
            raise PermissionError("motion is disarmed")
        if not command.is_fresh(now):
            raise TimeoutError("motion command is stale")
        if abs(command.linear_x) > self.limits.max_linear_speed:
            raise ValueError("linear_x exceeds the configured safety limit")
        if abs(command.linear_y) > self.limits.max_linear_speed:
            raise ValueError("linear_y exceeds the configured safety limit")
        if abs(command.angular_z) > self.limits.max_angular_speed:
            raise ValueError("angular_z exceeds the configured safety limit")
