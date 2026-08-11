"""Read-only provider for a robot already running its own ROS 2 stack."""

from __future__ import annotations

from dataclasses import dataclass
import time
from typing import Any, Callable

from ..contracts import DeviceState


@dataclass(frozen=True)
class ExistingRos2Config:
    """Connection and observed-interface contract for an existing ROS robot."""

    host: str = "127.0.0.1"
    port: int = 9090
    required_topics: tuple[str, ...] = ()
    capabilities: tuple[str, ...] = ()
    connect_timeout: float = 5.0


def load_roslibpy() -> Any:
    try:
        import roslibpy
    except Exception as exc:
        raise RuntimeError(
            "install roslibpy to use the existing ROS 2 adapter"
        ) from exc
    return roslibpy


class ExistingRos2Monitor:
    """Observe an existing ROS graph through rosbridge without publishing.

    This provider deliberately exposes no arm, command, or torque methods. The
    vendor robot stack retains ownership of actuators and startup services.
    """

    exclusive_connection = False

    def __init__(
        self,
        config: ExistingRos2Config,
        *,
        device_id: str = "device",
        client_factory: Callable[[str, int], Any] | None = None,
    ) -> None:
        if not str(config.host).strip():
            raise ValueError("ROSBridge host is required")
        if not 1 <= int(config.port) <= 65535:
            raise ValueError("ROSBridge port must be from 1 to 65535")
        if not config.required_topics:
            raise ValueError("at least one observed ROS topic is required")
        self.config = config
        self.capabilities = tuple(config.capabilities)
        self.device_id = device_id
        self._client_factory = client_factory
        self._client: Any | None = None
        self._state = DeviceState(
            device_id=device_id,
            connected=False,
            armed=False,
            capabilities=list(self.capabilities),
            values={
                "transport": "rosbridge",
                "host": config.host,
                "port": config.port,
                "required_topics": list(config.required_topics),
            },
        )

    def _new_client(self) -> Any:
        if self._client_factory is not None:
            return self._client_factory(self.config.host, self.config.port)
        roslibpy = load_roslibpy()
        return roslibpy.Ros(host=self.config.host, port=self.config.port)

    def connect(self) -> DeviceState:
        return self.refresh()

    def refresh(self) -> DeviceState:
        self._state.updated_at = time.time()
        try:
            if self._client is None:
                self._client = self._new_client()
            if not bool(getattr(self._client, "is_connected", False)):
                self._client.run(timeout=float(self.config.connect_timeout))
            if not bool(getattr(self._client, "is_connected", False)):
                raise ConnectionError(
                    f"ROSBridge did not connect at {self.config.host}:{self.config.port}"
                )
            topics = sorted(
                {
                    str(topic).strip()
                    for topic in (self._client.get_topics() or [])
                    if str(topic).strip()
                }
            )
            topic_set = set(topics)
            missing = [
                topic for topic in self.config.required_topics if topic not in topic_set
            ]
            self._state.connected = not missing
            self._state.armed = False
            self._state.capabilities = list(self.capabilities)
            self._state.values = {
                "transport": "rosbridge",
                "host": self.config.host,
                "port": self.config.port,
                "required_topics": list(self.config.required_topics),
                "observed_topics": topics,
                "missing_topics": missing,
                "read_only": True,
                "vendor_stack_preserved": True,
            }
            self._state.error = (
                "Required ROS topics are unavailable: " + ", ".join(missing)
                if missing
                else ""
            )
        except Exception as exc:
            self._state.connected = False
            self._state.armed = False
            self._state.error = str(exc)
            self._discard_client()
        return self._state

    def state(self) -> DeviceState:
        return self._state

    def close(self) -> None:
        self._discard_client()
        self._state.connected = False
        self._state.armed = False
        self._state.updated_at = time.time()

    def _discard_client(self) -> None:
        if self._client is not None:
            try:
                self._client.terminate()
            except Exception:
                pass
        self._client = None
