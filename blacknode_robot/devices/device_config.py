"""Validated local configuration for a Blacknode hardware device."""

from __future__ import annotations

import json
import os
from pathlib import Path
import tempfile
from typing import Any

from .adapters.serial_joint import (
    SerialJointConfig,
    SerialJointMonitor,
    SerialJointSpec,
)


CONFIG_VERSION = 1
DEFAULT_CONFIG_PATH = Path(".blacknode-hardware/device.json")


def normalize_device_name(value: Any, *, fallback: str = "") -> str:
    name = str(value or fallback).strip()
    if not name:
        raise ValueError("name must be a non-empty string")
    if len(name) > 80:
        raise ValueError("name must be 80 characters or fewer")
    if any(ord(character) < 32 or ord(character) == 127 for character in name):
        raise ValueError("name cannot contain control characters")
    return name


def validate_device_config(value: dict[str, Any]) -> dict[str, Any]:
    """Validate and normalize a serial read-only device configuration."""
    if value.get("version") != CONFIG_VERSION:
        raise ValueError(f"configuration version must be {CONFIG_VERSION}")
    if value.get("adapter") != "serial_joint":
        raise ValueError("adapter must be serial_joint")
    if value.get("mode") != "read_only":
        raise ValueError("mode must be read_only")

    device_id = value.get("device_id")
    port = value.get("port")
    baudrate = value.get("baudrate")
    servos = value.get("servos")
    if not isinstance(device_id, str) or not device_id.strip():
        raise ValueError("device_id must be a non-empty string")
    name = normalize_device_name(value.get("name"), fallback=device_id)
    if not isinstance(port, str) or not port.strip():
        raise ValueError("port must be a non-empty string")
    if isinstance(baudrate, bool) or not isinstance(baudrate, int) or baudrate <= 0:
        raise ValueError("baudrate must be a positive whole number")
    if not isinstance(servos, list) or not servos:
        raise ValueError("servos must contain at least one servo")

    normalized_servos: list[dict[str, Any]] = []
    seen_ids: set[int] = set()
    seen_names: set[str] = set()
    for servo in servos:
        if not isinstance(servo, dict):
            raise ValueError("each servo must be an object")
        servo_id = servo.get("id")
        servo_name = servo.get("name")
        if isinstance(servo_id, bool) or not isinstance(servo_id, int) or not 1 <= servo_id <= 253:
            raise ValueError("servo id must be a whole number from 1 to 253")
        if not isinstance(servo_name, str) or not servo_name.strip():
            raise ValueError("servo name must be a non-empty string")
        if servo_id in seen_ids:
            raise ValueError(f"duplicate servo id: {servo_id}")
        if servo_name in seen_names:
            raise ValueError(f"duplicate servo name: {servo_name}")
        seen_ids.add(servo_id)
        seen_names.add(servo_name)
        normalized_servos.append({"id": servo_id, "name": servo_name.strip()})

    return {
        "version": CONFIG_VERSION,
        "device_id": device_id.strip(),
        "name": name,
        "adapter": "serial_joint",
        "mode": "read_only",
        "port": port.strip(),
        "baudrate": baudrate,
        "servos": normalized_servos,
    }


def load_device_config(path: str | Path = DEFAULT_CONFIG_PATH) -> dict[str, Any]:
    config_path = Path(path)
    try:
        value = json.loads(config_path.read_text(encoding="utf-8"))
    except FileNotFoundError as exc:
        raise FileNotFoundError(f"device configuration not found: {config_path}") from exc
    except json.JSONDecodeError as exc:
        raise ValueError(f"invalid JSON in device configuration: {config_path}") from exc
    if not isinstance(value, dict):
        raise ValueError("device configuration must be a JSON object")
    return validate_device_config(value)


def save_device_config(value: dict[str, Any], path: str | Path = DEFAULT_CONFIG_PATH) -> Path:
    """Atomically create or replace the local device configuration."""
    normalized = validate_device_config(value)
    config_path = Path(path)
    config_path.parent.mkdir(parents=True, exist_ok=True)
    temporary_path: Path | None = None
    try:
        with tempfile.NamedTemporaryFile(
            "w",
            encoding="utf-8",
            dir=config_path.parent,
            prefix=f".{config_path.name}.",
            suffix=".tmp",
            delete=False,
        ) as temporary:
            json.dump(normalized, temporary, indent=2)
            temporary.write("\n")
            temporary.flush()
            os.fsync(temporary.fileno())
            temporary_path = Path(temporary.name)
        os.replace(temporary_path, config_path)
    finally:
        if temporary_path is not None and temporary_path.exists():
            temporary_path.unlink()
    return config_path


def serial_monitor_from_config(value: dict[str, Any]) -> SerialJointMonitor:
    config = validate_device_config(value)
    joints = tuple(
        SerialJointSpec(name=servo["name"], servo_id=servo["id"])
        for servo in config["servos"]
    )
    serial_config = SerialJointConfig(
        port=config["port"],
        baudrate=config["baudrate"],
        joints=joints,
    )
    return SerialJointMonitor(serial_config, device_id=config["device_id"])
