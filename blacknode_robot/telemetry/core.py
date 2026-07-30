"""Transport-neutral, timestamped telemetry for Robot Hardware."""

from __future__ import annotations

from dataclasses import dataclass
import json
import math
import re
import threading
import time
from typing import Any, Protocol

from blacknode_robot.devices.contracts import DeviceState, FaultState, JointState


_STREAM_NAME = re.compile(r"^[a-z0-9][a-z0-9._-]{0,63}$")


@dataclass(frozen=True)
class TelemetryEnvelope:
    """One versioned telemetry sample suitable for local or remote transports."""

    device_id: str
    stream: str
    sequence: int
    source_time: float
    receive_time: float
    payload: dict[str, Any]
    schema_version: int = 1

    def as_dict(self) -> dict[str, Any]:
        return {
            "kind": "blacknode.telemetry",
            "schema_version": self.schema_version,
            "device_id": self.device_id,
            "stream": self.stream,
            "sequence": self.sequence,
            "source_time": self.source_time,
            "receive_time": self.receive_time,
            "payload": dict(self.payload),
        }

    def to_json(self) -> str:
        return json.dumps(
            self.as_dict(),
            ensure_ascii=False,
            separators=(",", ":"),
            sort_keys=True,
        )


class TelemetrySink(Protocol):
    """Replaceable output transport such as MQTT or a future local WebSocket."""

    name: str

    def publish(self, envelope: TelemetryEnvelope) -> None: ...

    def status(self) -> dict[str, Any]: ...

    def close(self) -> None: ...


class TelemetryBus:
    """Fan telemetry out to optional sinks while retaining the latest sample."""

    def __init__(
        self,
        device_id: str,
        sinks: list[TelemetrySink] | None = None,
    ) -> None:
        self.device_id = str(device_id).strip()
        if not self.device_id:
            raise ValueError("telemetry device_id must not be empty")
        self._sinks = list(sinks or [])
        self._sequence: dict[str, int] = {}
        self._latest: dict[str, TelemetryEnvelope] = {}
        self._sink_errors: dict[str, str] = {}
        self._lock = threading.Lock()

    def add_sink(self, sink: TelemetrySink) -> None:
        with self._lock:
            self._sinks.append(sink)

    def publish(
        self,
        stream: str,
        payload: dict[str, Any],
        *,
        source_time: float | None = None,
    ) -> TelemetryEnvelope:
        normalized_stream = str(stream).strip().lower()
        if not _STREAM_NAME.fullmatch(normalized_stream):
            raise ValueError(
                "telemetry stream must contain lowercase letters, numbers, "
                "dots, underscores, or hyphens"
            )
        if not isinstance(payload, dict):
            raise TypeError("telemetry payload must be an object")
        received = time.time()
        with self._lock:
            sequence = self._sequence.get(normalized_stream, 0) + 1
            self._sequence[normalized_stream] = sequence
            envelope = TelemetryEnvelope(
                device_id=self.device_id,
                stream=normalized_stream,
                sequence=sequence,
                source_time=float(source_time if source_time is not None else received),
                receive_time=received,
                payload=dict(payload),
            )
            self._latest[normalized_stream] = envelope
            sinks = list(self._sinks)
        for sink in sinks:
            try:
                sink.publish(envelope)
            except Exception as exc:
                with self._lock:
                    self._sink_errors[sink.name] = str(exc)
            else:
                with self._lock:
                    self._sink_errors.pop(sink.name, None)
        return envelope

    def latest(self, stream: str | None = None) -> dict[str, Any]:
        with self._lock:
            if stream is not None:
                envelope = self._latest.get(str(stream).strip().lower())
                return envelope.as_dict() if envelope else {}
            return {
                name: envelope.as_dict()
                for name, envelope in self._latest.items()
            }

    def status(self) -> dict[str, Any]:
        with self._lock:
            sinks = list(self._sinks)
            errors = dict(self._sink_errors)
            streams = sorted(self._latest)
        return {
            "enabled": bool(sinks),
            "streams": streams,
            "sinks": [
                {
                    **sink.status(),
                    **({"error": errors[sink.name]} if sink.name in errors else {}),
                }
                for sink in sinks
            ],
        }

    def close(self) -> None:
        with self._lock:
            sinks = list(self._sinks)
            self._sinks.clear()
        for sink in sinks:
            try:
                sink.close()
            except Exception:
                pass


class RobotStateTelemetrySampler:
    """Publish normalized robot state and derived joint velocity at a fixed rate."""

    def __init__(
        self,
        runtime: Any,
        bus: TelemetryBus,
        *,
        interval: float = 0.1,
    ) -> None:
        if interval < 0.02:
            raise ValueError("telemetry interval must be at least 0.02 seconds")
        self.runtime = runtime
        self.bus = bus
        self.interval = float(interval)
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._previous_positions: dict[str, float] = {}
        self._previous_time: float | None = None

    def sample_once(self) -> TelemetryEnvelope:
        status = self.runtime.status()
        source_time = float(status.get("updated_at") or time.time())
        positions_deg = {
            str(name): float(value)
            for name, value in dict(status.get("positions") or {}).items()
            if isinstance(value, (int, float)) and not isinstance(value, bool)
        }
        velocities_deg: dict[str, float] = {}
        if self._previous_time is not None and source_time > self._previous_time:
            elapsed = source_time - self._previous_time
            velocities_deg = {
                name: (value - self._previous_positions[name]) / elapsed
                for name, value in positions_deg.items()
                if name in self._previous_positions
            }
        self._previous_positions = positions_deg
        self._previous_time = source_time
        limits = {}
        active_calibration = (
            status.get("calibration")
            if isinstance(status.get("calibration"), dict)
            else {}
        )
        calibration_joints = dict(active_calibration.get("joints") or {})
        calibration_topology = dict(active_calibration.get("topology") or {})
        for name, raw in dict(status.get("limits") or {}).items():
            if name not in positions_deg or not isinstance(raw, dict):
                continue
            try:
                lower = raw["lower"] if "lower" in raw else raw["min"]
                upper = raw["upper"] if "upper" in raw else raw["max"]
                limits[str(name)] = (
                    math.radians(float(lower)),
                    math.radians(float(upper)),
                )
            except (KeyError, TypeError, ValueError):
                continue
        for servo_id, semantic_name in calibration_topology.items():
            hardware_name = f"servo_{servo_id}"
            raw = calibration_joints.get(str(semantic_name))
            if hardware_name not in positions_deg or not isinstance(raw, dict):
                continue
            try:
                limits[hardware_name] = (
                    math.radians(float(raw["safe_min_deg"])),
                    math.radians(float(raw["safe_max_deg"])),
                )
            except (KeyError, TypeError, ValueError):
                continue
        error = str(status.get("error") or "")
        faults = (
            [FaultState(code="device-error", message=error)]
            if error
            else []
        )
        state = DeviceState(
            device_id=self.bus.device_id,
            connected=bool(status.get("connected")),
            armed=bool(status.get("armed")),
            torque_enabled=status.get("torque_enabled"),
            capabilities=list(status.get("capabilities") or []),
            joint_state=JointState(
                positions={
                    name: math.radians(value)
                    for name, value in positions_deg.items()
                },
                velocities={
                    name: math.radians(value)
                    for name, value in velocities_deg.items()
                },
                limits=limits,
                source_time=source_time,
            ),
            faults=faults,
            temperatures_c={
                str(name): float(value)
                for name, value in dict(status.get("temperatures_c") or {}).items()
                if isinstance(value, (int, float)) and not isinstance(value, bool)
            },
            voltage_v=(
                float(status["voltage_v"])
                if isinstance(status.get("voltage_v"), (int, float))
                and not isinstance(status.get("voltage_v"), bool)
                else None
            ),
            values={
                "calibrated": status.get("calibrated"),
                "calibration": active_calibration,
                "raw_positions": {
                    str(name): int(value)
                    for name, value in dict(status.get("raw_positions") or {}).items()
                    if isinstance(value, int) and not isinstance(value, bool)
                },
                "leased_to_deployment": bool(status.get("leased_to_deployment")),
                "torque_report_error": str(status.get("torque_report_error") or ""),
            },
            error=error,
            updated_at=source_time,
        )
        return self.bus.publish(
            "robot-state",
            state.as_dict(),
            source_time=source_time,
        )

    def start(self) -> None:
        if self._thread is not None and self._thread.is_alive():
            return
        self._stop.clear()
        self._thread = threading.Thread(
            target=self._run,
            name=f"blacknode-telemetry-{self.bus.device_id}",
            daemon=True,
        )
        self._thread.start()

    def _run(self) -> None:
        next_sample = time.monotonic()
        while not self._stop.is_set():
            try:
                self.sample_once()
            except Exception as exc:
                self.bus.publish(
                    "robot-state",
                    {
                        "connected": False,
                        "armed": False,
                        "error": f"telemetry sample failed: {exc}",
                    },
                )
            next_sample += self.interval
            delay = max(0.0, next_sample - time.monotonic())
            if self._stop.wait(delay):
                break

    def stop(self) -> None:
        self._stop.set()
        thread = self._thread
        if thread is not None and thread is not threading.current_thread():
            thread.join(timeout=max(1.0, self.interval * 3))
        self._thread = None
