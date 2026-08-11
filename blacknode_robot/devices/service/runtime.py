"""Hardware service runtime with honest disconnected-device status."""

from __future__ import annotations

import threading
from pathlib import Path
from typing import Any

from ...telemetry import RobotStateTelemetrySampler, TelemetryBus
from ..version import service_version


def _serial_connection_present(port: str) -> bool | None:
    """Check a POSIX device path without opening or claiming the serial port."""
    clean_port = str(port or "").strip()
    if not clean_port.startswith("/"):
        return None
    return Path(clean_port).exists()


class HardwareRuntime:
    def __init__(
        self,
        provider: Any | None = None,
        device_id: str = "device",
        calibration_store: Any | None = None,
        telemetry_bus: TelemetryBus | None = None,
    ) -> None:
        self.provider = provider
        self.device_id = device_id
        self.calibration_store = calibration_store
        self.telemetry_bus = telemetry_bus
        self._telemetry_sampler: RobotStateTelemetrySampler | None = None
        self._lease_lock = threading.Lock()
        self._leased_to_deployment = False

    def service_features(self) -> list[str]:
        features = ["torque_release_v1"]
        if self.telemetry_bus is not None:
            features.append("telemetry_v1")
            if any(
                sink.get("name") == "mqtt"
                for sink in self.telemetry_bus.status().get("sinks", [])
            ):
                features.append("mqtt_telemetry_v1")
        return features

    def telemetry_status(self) -> dict[str, Any]:
        if self.telemetry_bus is None:
            return {"enabled": False, "streams": [], "sinks": []}
        return self.telemetry_bus.status()

    def start_telemetry(self, *, interval: float = 0.1) -> None:
        if self.telemetry_bus is None:
            raise RuntimeError("no telemetry bus is configured")
        if self._telemetry_sampler is None:
            self._telemetry_sampler = RobotStateTelemetrySampler(
                self,
                self.telemetry_bus,
                interval=interval,
            )
        self._telemetry_sampler.start()

    def status(self) -> dict[str, Any]:
        if self.provider is None:
            payload = {
                "device_id": self.device_id,
                "software_version": service_version(),
                "service_features": self.service_features(),
                "connected": False,
                "armed": False,
                "capabilities": [],
                "error": "no hardware adapter configured",
            }
            if self.telemetry_bus is not None:
                payload["telemetry"] = self.telemetry_status()
            return payload
        with self._lease_lock:
            leased = self._leased_to_deployment
        provider_config = getattr(self.provider, "config", None)
        serial_port = str(getattr(provider_config, "port", "") or "").strip()
        connection_present = _serial_connection_present(serial_port)
        if leased:
            payload = {
                "device_id": self.device_id,
                "connected": bool(connection_present),
                "armed": False,
                "torque_enabled": None,
                "torque_report_error": (
                    "A running deployment owns the robot connection, so Robot "
                    "Hardware cannot read the physical torque-enable registers."
                ),
                "leased_to_deployment": True,
                "error": "serial hardware is leased to a Blacknode deployment",
            }
            if connection_present is not None:
                payload["connection_present"] = connection_present
                payload["connection_reported"] = True
                payload["connection_source"] = "device_path"
        else:
            try:
                state = self.provider.refresh() if hasattr(self.provider, "refresh") else self.provider.state()
                payload = state.as_dict() if hasattr(state, "as_dict") else dict(state)
            except Exception as exc:
                payload = {
                    "device_id": self.device_id,
                    "connected": False,
                    "armed": False,
                    "error": str(exc),
                }
        payload["device_id"] = self.device_id
        payload["software_version"] = service_version()
        payload["service_features"] = self.service_features()
        if self.telemetry_bus is not None:
            payload["telemetry"] = self.telemetry_status()
        payload.setdefault("armed", False)
        payload["leased_to_deployment"] = leased
        payload["capabilities"] = list(getattr(self.provider, "capabilities", ()))
        if serial_port:
            payload["connection"] = {
                "transport": "serial",
                "port": serial_port,
            }
            if connection_present is not None:
                payload["connection_present"] = connection_present
        if self.calibration_store is not None:
            calibration = self.calibration_store.status()
            payload["calibrated"] = calibration.get("active") is True
            payload["calibration"] = (
                {key: value for key, value in calibration.items() if key not in {"active", "error"}}
                if calibration.get("active")
                else None
            )
            if calibration.get("error"):
                payload["calibration_error"] = str(calibration["error"])
        return payload

    def activate_calibration(
        self,
        profile: dict[str, Any],
        calibration: dict[str, Any],
    ) -> dict[str, Any]:
        if self.calibration_store is None:
            raise RuntimeError("calibration storage is not configured")
        status = self.status()
        if not status.get("connected"):
            raise RuntimeError("hardware must be connected before calibration activation")
        if status.get("armed"):
            raise RuntimeError("hardware must be disarmed before calibration activation")
        return self.calibration_store.activate(profile, calibration)

    def deactivate_calibration(self) -> dict[str, Any]:
        if self.calibration_store is None:
            raise RuntimeError("calibration storage is not configured")
        return self.calibration_store.deactivate()

    def calibration_status(self) -> dict[str, Any]:
        if self.calibration_store is None:
            return {"active": False, "error": "calibration storage is not configured"}
        return self.calibration_store.status()

    def capabilities(self) -> dict[str, Any]:
        status = self.status()
        return {
            "device_id": status.get("device_id", self.device_id),
            "connected": bool(status.get("connected", False)),
            "capabilities": list(status.get("capabilities", [])),
        }

    def stop(self) -> dict[str, Any]:
        return self.release()

    def release(self) -> dict[str, Any]:
        if self.provider is None:
            return {"ok": False, "error": "no hardware adapter configured"}
        if getattr(self.provider, "exclusive_connection", True) is False:
            return {"ok": True, "status": self.status()}
        if hasattr(self.provider, "stop"):
            self.provider.stop()
        if hasattr(self.provider, "disarm"):
            self.provider.disarm()
        if hasattr(self.provider, "close"):
            self.provider.close()
        with self._lease_lock:
            self._leased_to_deployment = True
        return {"ok": True, "status": self.status()}

    def resume(self) -> dict[str, Any]:
        if self.provider is None:
            return {"ok": False, "error": "no hardware adapter configured"}
        try:
            state = self.provider.state() if hasattr(self.provider, "state") else {}
            state_payload = (
                state.as_dict()
                if hasattr(state, "as_dict")
                else dict(state) if isinstance(state, dict) else {}
            )
            if not state_payload.get("connected") and hasattr(self.provider, "connect"):
                self.provider.connect()
            # Reconnection must never imply motion authorization or holding
            # torque. Providers that can control torque return to disarmed.
            if hasattr(self.provider, "disarm"):
                self.provider.disarm()
        except Exception as exc:
            return {"ok": False, "error": f"could not reconnect hardware safely: {exc}"}
        with self._lease_lock:
            self._leased_to_deployment = False
        return {"ok": True, "status": self.status()}

    def disable_torque(self) -> dict[str, Any]:
        """Explicitly release physical holding torque while motion stays disarmed."""
        with self._lease_lock:
            if self._leased_to_deployment:
                return {
                    "ok": False,
                    "error": "stop the active deployment before releasing torque",
                }
        if self.provider is None or not hasattr(self.provider, "release_torque"):
            return {
                "ok": False,
                "error": "this hardware provider cannot release torque",
            }
        try:
            self.provider.release_torque()
        except Exception as exc:
            return {"ok": False, "error": str(exc)}
        return {"ok": True, "status": self.status()}

    def call(self, method: str, params: dict[str, Any] | None = None) -> Any:
        del params
        if method == "get_status":
            return self.status()
        if method == "get_capabilities":
            return self.capabilities()
        if method == "stop":
            result = self.stop()
            if not result["ok"]:
                raise RuntimeError(result["error"])
            return result
        if method == "release":
            result = self.release()
            if not result["ok"]:
                raise RuntimeError(result["error"])
            return result
        if method == "resume":
            result = self.resume()
            if not result["ok"]:
                raise RuntimeError(result["error"])
            return result
        if method == "disable_torque":
            result = self.disable_torque()
            if not result["ok"]:
                raise RuntimeError(result["error"])
            return result
        raise ValueError(f"unknown method: {method}")

    def close(self) -> None:
        if self._telemetry_sampler is not None:
            self._telemetry_sampler.stop()
            self._telemetry_sampler = None
        if self.provider is not None and hasattr(self.provider, "close"):
            self.provider.close()
        if self.telemetry_bus is not None:
            self.telemetry_bus.close()
