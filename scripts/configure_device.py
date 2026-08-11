"""Create or replace the local read-only hardware service configuration."""

from __future__ import annotations

import argparse
from glob import glob
import os
from pathlib import Path
import socket
import sys
from typing import Any

from blacknode_robot.devices.device_config import (
    DEFAULT_CONFIG_PATH,
    load_device_config,
    save_device_config,
)


def detect_serial_port() -> str:
    stable_ports = sorted(glob("/dev/serial/by-id/*"))
    if len(stable_ports) == 1:
        return stable_ports[0]
    if len(stable_ports) > 1:
        choices = "\n  ".join(stable_ports)
        raise ValueError(f"multiple serial devices found; choose one with --port:\n  {choices}")

    fallback_ports = sorted(set(glob("/dev/ttyACM*") + glob("/dev/ttyUSB*")))
    if len(fallback_ports) == 1:
        return fallback_ports[0]
    if len(fallback_ports) > 1:
        choices = "\n  ".join(fallback_ports)
        raise ValueError(f"multiple serial devices found; choose one with --port:\n  {choices}")
    raise ValueError("no serial device found; connect it and run ./discover.sh")


def existing_config(path: Path) -> dict[str, Any] | None:
    if not path.exists():
        return None
    return load_device_config(path)


def print_config(config: dict[str, Any], path: Path) -> None:
    print("Blacknode Hardware configuration")
    print("================================")
    print(f"File: {path}")
    print(f"Name: {config['name']}")
    print(f"Device ID: {config['device_id']}")
    print(f"Mode: {config['mode']}")
    print(f"Adapter: {config['adapter']}")
    if config["adapter"] == "existing_ros2":
        print(f"ROSBridge: {config['host']}:{config['rosbridge_port']}")
        print(f"Observed topics: {', '.join(config['required_topics'])}")
        print(f"Capabilities: {', '.join(config['capabilities'])}")
    else:
        print(f"Port: {config['port']}")
        print(f"Baudrate: {config['baudrate']}")
        print(f"Servos: {', '.join(str(servo['id']) for servo in config['servos'])}")


def main() -> int:
    default_path = Path(os.environ.get("BLACKNODE_HARDWARE_CONFIG", DEFAULT_CONFIG_PATH))
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--servos", "-s", type=int, help="configure sequential servo IDs starting at 1")
    parser.add_argument("--port", help="serial path; use 'auto' to detect it again")
    parser.add_argument("--baudrate", type=int)
    parser.add_argument("--name", help="friendly robot name shown during pairing")
    parser.add_argument("--device-id")
    parser.add_argument("--config", type=Path, default=default_path)
    parser.add_argument("--show", action="store_true", help="show the saved configuration without changing it")
    args = parser.parse_args()

    if args.show:
        print_config(load_device_config(args.config), args.config)
        return 0

    previous = existing_config(args.config)
    servo_count = args.servos if args.servos is not None else (
        len(previous["servos"]) if previous else None
    )
    if servo_count is None:
        parser.error("--servos is required for the first configuration")
    if not 1 <= servo_count <= 253:
        parser.error("--servos must be from 1 to 253")

    requested_port = args.port or os.environ.get("BLACKNODE_SERIAL_PORT")
    if requested_port == "auto":
        port = detect_serial_port()
    else:
        port = requested_port or (previous["port"] if previous else None) or detect_serial_port()
    baudrate = args.baudrate if args.baudrate is not None else (
        previous["baudrate"] if previous else int(os.environ.get("BLACKNODE_SERIAL_BAUDRATE", "1000000"))
    )
    device_id = args.device_id or (previous["device_id"] if previous else socket.gethostname())
    name = args.name or (previous["name"] if previous else device_id)
    servos = (
        previous["servos"]
        if previous is not None and args.servos is None
        else [
            {"id": servo_id, "name": f"servo_{servo_id}"}
            for servo_id in range(1, servo_count + 1)
        ]
    )

    config = {
        "version": 1,
        "device_id": device_id,
        "name": name,
        "adapter": "serial_joint",
        "mode": "read_only",
        "port": port,
        "baudrate": baudrate,
        "servos": servos,
    }
    saved_path = save_device_config(config, args.config)
    print_config(config, saved_path)
    print()
    print("Configuration saved. Re-run ./configure.sh anytime to change it.")
    print("No actuator commands or torque writes were sent.")
    print("Next: ./start.sh")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (FileNotFoundError, ValueError) as exc:
        print(f"Configuration failed: {exc}", file=sys.stderr)
        raise SystemExit(1) from None
