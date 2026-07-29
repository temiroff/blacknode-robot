"""Optional MQTT transport for normalized Robot Hardware telemetry."""

from __future__ import annotations

from dataclasses import dataclass, field
import os
from pathlib import Path
import re
import threading
import time
from typing import Any, Callable
from urllib.parse import quote, urlsplit

from ..core import TelemetryEnvelope


_TOPIC_PREFIX = re.compile(r"^[^#+\x00]+$")


@dataclass(frozen=True)
class MqttTelemetryConfig:
    broker_url: str
    device_id: str
    topic_prefix: str = "blacknode"
    client_id: str = ""
    username: str = ""
    password: str = field(default="", repr=False)
    qos: int = 0
    keepalive: int = 30

    def __post_init__(self) -> None:
        parsed = urlsplit(self.broker_url)
        if parsed.scheme not in {"mqtt", "mqtts"}:
            raise ValueError("MQTT broker URL must start with mqtt:// or mqtts://")
        if not parsed.hostname:
            raise ValueError("MQTT broker URL must include a hostname")
        if parsed.username or parsed.password:
            raise ValueError(
                "MQTT credentials must use environment variables or a password file"
            )
        if parsed.path not in {"", "/"} or parsed.query or parsed.fragment:
            raise ValueError("MQTT broker URL must not include a path, query, or fragment")
        prefix = self.topic_prefix.strip().strip("/")
        if not prefix or not _TOPIC_PREFIX.fullmatch(prefix):
            raise ValueError("MQTT topic prefix must not contain wildcards")
        if not self.device_id.strip():
            raise ValueError("MQTT device_id must not be empty")
        if self.qos not in {0, 1, 2}:
            raise ValueError("MQTT QoS must be 0, 1, or 2")
        if not 5 <= self.keepalive <= 65535:
            raise ValueError("MQTT keepalive must be from 5 to 65535 seconds")
        if self.password and not self.username:
            raise ValueError("MQTT username is required when a password is configured")

    @property
    def host(self) -> str:
        return str(urlsplit(self.broker_url).hostname)

    @property
    def port(self) -> int:
        parsed = urlsplit(self.broker_url)
        return int(parsed.port or (8883 if parsed.scheme == "mqtts" else 1883))

    @property
    def use_tls(self) -> bool:
        return urlsplit(self.broker_url).scheme == "mqtts"

    @property
    def normalized_topic_prefix(self) -> str:
        return self.topic_prefix.strip().strip("/")


def mqtt_config_from_env(
    device_id: str,
    environ: dict[str, str] | None = None,
) -> MqttTelemetryConfig | None:
    values = os.environ if environ is None else environ
    broker_url = str(values.get("BLACKNODE_MQTT_URL") or "").strip()
    if not broker_url:
        return None
    password = ""
    password_file = str(values.get("BLACKNODE_MQTT_PASSWORD_FILE") or "").strip()
    if password_file:
        path = Path(password_file)
        password = path.read_text(encoding="utf-8").strip()
        if not password:
            raise ValueError(f"MQTT password file is empty: {path}")
    elif values.get("BLACKNODE_MQTT_PASSWORD"):
        password = str(values["BLACKNODE_MQTT_PASSWORD"])
    qos_text = str(values.get("BLACKNODE_MQTT_QOS") or "0").strip()
    keepalive_text = str(values.get("BLACKNODE_MQTT_KEEPALIVE") or "30").strip()
    try:
        qos = int(qos_text)
        keepalive = int(keepalive_text)
    except ValueError as exc:
        raise ValueError("MQTT QoS and keepalive must be whole numbers") from exc
    safe_device = quote(str(device_id).strip(), safe="-_.~")
    return MqttTelemetryConfig(
        broker_url=broker_url,
        device_id=device_id,
        topic_prefix=str(
            values.get("BLACKNODE_MQTT_TOPIC_PREFIX") or "blacknode"
        ),
        client_id=str(
            values.get("BLACKNODE_MQTT_CLIENT_ID")
            or f"blacknode-hardware-{safe_device}"
        ),
        username=str(values.get("BLACKNODE_MQTT_USERNAME") or ""),
        password=password,
        qos=qos,
        keepalive=keepalive,
    )


class MqttTelemetryPublisher:
    """Publish telemetry only; physical control is deliberately out of scope."""

    name = "mqtt"

    def __init__(
        self,
        config: MqttTelemetryConfig,
        *,
        client_factory: Callable[[], Any] | None = None,
    ) -> None:
        self.config = config
        self._lock = threading.Lock()
        self._connected = False
        self._published = 0
        self._last_published_at = 0.0
        self._last_error = ""
        self._mqtt_success = 0
        if client_factory is None:
            try:
                import paho.mqtt.client as mqtt
            except ModuleNotFoundError as exc:
                raise RuntimeError(
                    "MQTT telemetry requires the optional dependency; install "
                    "blacknode-robot[mqtt]"
                ) from exc
            self._mqtt_success = int(mqtt.MQTT_ERR_SUCCESS)
            self._client = mqtt.Client(
                mqtt.CallbackAPIVersion.VERSION2,
                client_id=config.client_id,
                protocol=mqtt.MQTTv311,
            )
        else:
            self._client = client_factory()
        if hasattr(self._client, "max_queued_messages_set"):
            self._client.max_queued_messages_set(1000)
        if hasattr(self._client, "reconnect_delay_set"):
            self._client.reconnect_delay_set(min_delay=1, max_delay=30)
        self._client.on_connect = self._on_connect
        self._client.on_disconnect = self._on_disconnect
        if config.username:
            self._client.username_pw_set(config.username, config.password or None)
        if config.use_tls:
            self._client.tls_set()
        self._client.connect_async(config.host, config.port, config.keepalive)
        self._client.loop_start()

    def _on_connect(
        self,
        _client: Any,
        _userdata: Any,
        _flags: Any,
        reason_code: Any,
        _properties: Any = None,
    ) -> None:
        try:
            connected = int(reason_code) == 0
        except (TypeError, ValueError):
            connected = not bool(reason_code)
        with self._lock:
            self._connected = connected
            self._last_error = "" if connected else f"broker rejected connection: {reason_code}"

    def _on_disconnect(
        self,
        _client: Any,
        _userdata: Any,
        *_args: Any,
    ) -> None:
        with self._lock:
            self._connected = False

    def topic_for(self, stream: str) -> str:
        device = quote(self.config.device_id.strip(), safe="-_.~")
        return (
            f"{self.config.normalized_topic_prefix}/{device}/telemetry/"
            f"{quote(stream, safe='-_.~')}"
        )

    def publish(self, envelope: TelemetryEnvelope) -> None:
        result = self._client.publish(
            self.topic_for(envelope.stream),
            envelope.to_json(),
            qos=self.config.qos,
            retain=False,
        )
        result_code = int(getattr(result, "rc", self._mqtt_success))
        if result_code != self._mqtt_success:
            raise RuntimeError(f"MQTT publish failed with code {result_code}")
        with self._lock:
            self._published += 1
            self._last_published_at = time.time()
            self._last_error = ""

    def status(self) -> dict[str, Any]:
        with self._lock:
            return {
                "name": self.name,
                "configured": True,
                "connected": self._connected,
                "broker": f"{self.config.host}:{self.config.port}",
                "tls": self.config.use_tls,
                "topic_prefix": self.config.normalized_topic_prefix,
                "qos": self.config.qos,
                "published": self._published,
                "last_published_at": self._last_published_at or None,
                "error": self._last_error,
            }

    def close(self) -> None:
        try:
            self._client.disconnect()
        finally:
            self._client.loop_stop()
