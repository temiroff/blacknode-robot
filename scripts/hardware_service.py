"""Start the device-side service with an optional local hardware configuration."""

from __future__ import annotations

import argparse
import os
from pathlib import Path

from blacknode_robot.telemetry.adapters.mqtt import (
    MqttTelemetryPublisher,
    mqtt_config_from_env,
)
from blacknode_robot.devices.auth import load_auth_token, token_fingerprint
from blacknode_robot.devices.calibration import CalibrationStore
from blacknode_robot.devices.device_config import load_device_config, serial_monitor_from_config
from blacknode_robot.devices.service import HardwareRuntime
from blacknode_robot.devices.service.server import serve
from blacknode_robot.telemetry import TelemetryBus


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8765)
    parser.add_argument("--device-id")
    parser.add_argument("--config")
    parser.add_argument("--auth-token-file")
    parser.add_argument("--require-auth", action="store_true")
    args = parser.parse_args()
    provider = None
    calibration_store = None
    config_device_id = "device"
    config = None
    if args.config:
        config = load_device_config(args.config)
        config_device_id = config["device_id"]
        provider = serial_monitor_from_config(config)
        provider.connect()
    auth_token = None
    token_path = Path(args.auth_token_file) if args.auth_token_file else None
    if token_path is None and args.config:
        token_path = Path(args.config).parent / "auth.token"
    if token_path is not None and token_path.exists():
        auth_token = load_auth_token(token_path)
    elif args.require_auth:
        parser.error(f"pairing token not found: {token_path}")
    device_id = args.device_id or config_device_id
    if config is not None and args.config:
        calibration_store = CalibrationStore(
            Path(args.config).parent / "active-calibration.json",
            device_id=device_id,
            servos=config["servos"],
        )
    if auth_token:
        print(f"pairing authentication enabled ({token_fingerprint(auth_token)})")
    else:
        print("pairing authentication is not configured; read-only trusted-LAN mode")
    telemetry_bus = None
    mqtt_config = mqtt_config_from_env(device_id)
    if mqtt_config is not None:
        telemetry_bus = TelemetryBus(
            device_id,
            sinks=[MqttTelemetryPublisher(mqtt_config)],
        )
        print(
            "MQTT telemetry enabled at "
            f"{mqtt_config.host}:{mqtt_config.port} under "
            f"{mqtt_config.normalized_topic_prefix}/{device_id}/telemetry"
        )
    runtime = HardwareRuntime(
        provider=provider,
        device_id=device_id,
        calibration_store=calibration_store,
        telemetry_bus=telemetry_bus,
    )
    if telemetry_bus is not None:
        interval_text = os.environ.get("BLACKNODE_TELEMETRY_INTERVAL", "0.1")
        try:
            interval = float(interval_text)
        except ValueError:
            parser.error("BLACKNODE_TELEMETRY_INTERVAL must be a number")
        runtime.start_telemetry(interval=interval)
    serve(runtime, args.host, args.port, auth_token=auth_token)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
