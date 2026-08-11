"""Discover and configure robots through safe, read-only hardware providers."""

from __future__ import annotations

import argparse
from glob import glob
import hashlib
import json
import os
from pathlib import Path
import re
import socket
import sys
import tempfile
from typing import Any, Iterable

from blacknode_robot.devices.adapters.serial_joint import (
    SerialJointConfig,
    SerialJointSpec,
    probe_serial,
)
from blacknode_robot.devices.adapters.existing_ros2 import (
    ExistingRos2Config,
    ExistingRos2Monitor,
)
from blacknode_robot.devices.device_config import normalize_device_name, save_device_config


FLEET_VERSION = 1
DEFAULT_ROOT = Path(".blacknode-hardware")
DEFAULT_RUNTIME_PORT = 8766
STACK_INSTANCE_RE = re.compile(r"[a-z0-9][a-z0-9-]{0,31}")


def normalize_stack_instance(value: str | None) -> str:
    instance = str(value or "").strip()
    if instance and not STACK_INSTANCE_RE.fullmatch(instance):
        raise ValueError(
            "hardware stack instance must contain lowercase letters, numbers, or hyphens"
        )
    return instance


def hardware_unit_name(device_key_value: str, stack_instance: str = "") -> str:
    instance = normalize_stack_instance(stack_instance)
    prefix = f"blacknode-hardware-{instance}" if instance else "blacknode-hardware"
    return f"{prefix}-{device_key_value}.service"


def deduplicate_serial_ports(paths: Iterable[str]) -> list[str]:
    """Keep one stable-looking path for each underlying serial device."""
    unique: dict[str, str] = {}
    for raw_path in sorted(set(str(path) for path in paths)):
        resolved = os.path.realpath(raw_path)
        current = unique.get(resolved)
        if current is None or (
            "/dev/serial/by-id/" in raw_path
            and "/dev/serial/by-id/" not in current
        ):
            unique[resolved] = raw_path
    return sorted(unique.values())


def discover_serial_ports() -> list[str]:
    stable = glob("/dev/serial/by-id/*")
    fallback = glob("/dev/ttyACM*") + glob("/dev/ttyUSB*")
    return deduplicate_serial_ports([*stable, *fallback])


def discover_occupied_service_ports(
    start: int = 8765,
    end: int = 8865,
) -> set[int]:
    occupied: set[int] = set()
    for port in range(start, end + 1):
        probe = socket.socket()
        try:
            probe.bind(("0.0.0.0", port))
        except OSError:
            occupied.add(port)
        finally:
            probe.close()
    return occupied


def device_key(port: str) -> str:
    label = Path(port).name.lower()
    label = re.sub(r"[^a-z0-9]+", "-", label).strip("-") or "serial"
    digest = hashlib.sha256(port.encode("utf-8")).hexdigest()[:8]
    return f"{label[:40].rstrip('-')}-{digest}"


def existing_ros2_key(host: str, port: int) -> str:
    identity = f"{host}:{port}"
    digest = hashlib.sha256(identity.encode("utf-8")).hexdigest()[:8]
    return f"existing-ros2-{digest}"


def load_manifest(path: Path) -> dict[str, Any]:
    if not path.exists():
        return {"version": FLEET_VERSION, "devices": []}
    value = json.loads(path.read_text(encoding="utf-8"))
    if (
        not isinstance(value, dict)
        or value.get("version") != FLEET_VERSION
        or not isinstance(value.get("devices"), list)
    ):
        raise ValueError(f"invalid multi-robot manifest: {path}")
    return value


def save_manifest(path: Path, value: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary_path: Path | None = None
    try:
        with tempfile.NamedTemporaryFile(
            "w",
            encoding="utf-8",
            dir=path.parent,
            prefix=f".{path.name}.",
            suffix=".tmp",
            delete=False,
        ) as temporary:
            json.dump(value, temporary, indent=2)
            temporary.write("\n")
            temporary.flush()
            os.fsync(temporary.fileno())
            temporary_path = Path(temporary.name)
        os.replace(temporary_path, path)
    finally:
        if temporary_path is not None and temporary_path.exists():
            temporary_path.unlink()


def build_fleet_entries(
    detected: list[tuple[str, list[int]]],
    *,
    root: Path,
    hostname: str,
    base_port: int,
    previous: list[dict[str, Any]] | None = None,
    reserved_ports: set[int] | None = None,
    stack_instance: str = "",
) -> list[dict[str, Any]]:
    stack_instance = normalize_stack_instance(stack_instance)
    reserved_ports = set(
        {DEFAULT_RUNTIME_PORT} if reserved_ports is None else reserved_ports
    )
    previous_by_serial = {
        str(item.get("serial_port")): item
        for item in (previous or [])
        if isinstance(item, dict) and item.get("serial_port")
    }
    reserved_service_ports = {
        int(item["service_port"])
        for item in (previous or [])
        if (
            isinstance(item, dict)
            and isinstance(item.get("service_port"), int)
            and not isinstance(item.get("service_port"), bool)
            and 1 <= int(item["service_port"]) <= 65535
        )
    }
    used_service_ports: set[int] = set()
    entries: list[dict[str, Any]] = []
    next_service_port = base_port
    for serial_port, servo_ids in detected:
        index = len(entries) + 1
        old = previous_by_serial.get(serial_port, {})
        key = str(old.get("key") or device_key(serial_port))
        service_port = old.get("service_port")
        if (
            isinstance(service_port, bool)
            or not isinstance(service_port, int)
            or not 1 <= service_port <= 65535
            or service_port in reserved_ports
            or service_port in used_service_ports
        ):
            while (
                next_service_port in used_service_ports
                or next_service_port in reserved_service_ports
                or next_service_port in reserved_ports
            ):
                next_service_port += 1
            service_port = next_service_port
        if service_port > 65535:
            raise ValueError("not enough HTTP ports remain for every detected robot")
        used_service_ports.add(service_port)
        next_service_port = max(next_service_port, service_port + 1)
        device_id = str(old.get("device_id") or f"{hostname}-{key}")
        name = normalize_device_name(old.get("name"), fallback=f"Robot {index}")
        device_dir = root / "devices" / key
        entries.append(
            {
                "key": key,
                "name": name,
                "device_id": device_id,
                "serial_port": serial_port,
                "service_port": service_port,
                "servo_ids": sorted(set(servo_ids)),
                "config": str(device_dir / "device.json"),
                "token_file": str(device_dir / "auth.token"),
                "unit": hardware_unit_name(key, stack_instance),
            }
        )
    return entries


def assign_device_names(
    entries: list[dict[str, Any]],
    requested_names: list[str],
    *,
    prompt: bool,
) -> None:
    if len(requested_names) > len(entries):
        raise ValueError("more --name values were provided than detected robots")
    if entries and prompt:
        print()
        print("Name the detected robots")
        print("========================")
        print("These labels are shown again with each editor pairing token.")
    for index, entry in enumerate(entries):
        if index < len(requested_names):
            entry["name"] = normalize_device_name(requested_names[index])
            continue
        if not prompt:
            continue
        print()
        print(f"Robot {index + 1}")
        print(f"  Serial: {entry['serial_port']}")
        print(f"  Servos: {', '.join(str(value) for value in entry['servo_ids'])}")
        entered = input(f"  Friendly name [{entry['name']}]: ").strip()
        if entered:
            entry["name"] = normalize_device_name(entered)


def print_manifest(manifest: dict[str, Any], path: Path) -> None:
    devices = manifest["devices"]
    print("Blacknode multi-robot configuration")
    print("===================================")
    print(f"Manifest: {path}")
    print(f"Robots: {len(devices)}")
    for index, device in enumerate(devices, start=1):
        print()
        print(f"{index}. {device.get('name') or device['device_id']}")
        print(f"   Device ID: {device['device_id']}")
        if device.get("adapter") == "existing_ros2":
            print(
                "   ROSBridge: "
                f"{device.get('rosbridge_host')}:{device.get('rosbridge_port')}"
            )
            print(f"   Topics: {', '.join(device.get('required_topics') or [])}")
        else:
            servo_ids = ", ".join(str(value) for value in device["servo_ids"])
            print(f"   Serial: {device['serial_port']}")
            print(f"   Servos: {servo_ids}")
        print(f"   Service: http://DEVICE_IP:{device['service_port']}")


def list_manifest_rows(manifest: dict[str, Any]) -> None:
    for device in manifest["devices"]:
        print(
            "\t".join(
                [
                    str(device["key"]),
                    str(device.get("name") or device["device_id"]),
                    str(device["device_id"]),
                    str(device["service_port"]),
                    str(device["config"]),
                    str(device["token_file"]),
                    str(device["unit"]),
                ]
            )
        )


def main() -> int:
    parser = argparse.ArgumentParser(
        description=__doc__,
        epilog=(
            "Use ./configure.sh --all --install to configure, pair, install, "
            "and start every detected robot."
        ),
    )
    parser.add_argument(
        "--servos",
        "-s",
        type=int,
        default=20,
        help="scan servo IDs 1 through COUNT on every serial bus (default: 20)",
    )
    parser.add_argument(
        "--existing-ros2",
        action="store_true",
        help="configure the robot already exposed through ROSBridge instead of scanning serial servos",
    )
    parser.add_argument("--rosbridge-host", default="127.0.0.1")
    parser.add_argument("--rosbridge-port", type=int, default=9090)
    parser.add_argument(
        "--required-topic",
        action="append",
        default=[],
        help="observed ROS topic required for a healthy robot; repeat as needed",
    )
    parser.add_argument(
        "--capability",
        action="append",
        default=[],
        help="capability confirmed from the live ROS graph; repeat as needed",
    )
    parser.add_argument("--baudrate", type=int, default=1_000_000)
    parser.add_argument("--base-port", type=int, default=8765)
    parser.add_argument(
        "--runtime-port",
        type=int,
        default=int(os.environ.get("BLACKNODE_RUNTIME_PORT", str(DEFAULT_RUNTIME_PORT))),
        help="reserve the Blacknode runtime service port (default: 8766)",
    )
    parser.add_argument(
        "--instance",
        default=os.environ.get("BLACKNODE_HARDWARE_INSTANCE", ""),
        help="isolated stack identity used to namespace Hardware services",
    )
    parser.add_argument(
        "--reserved-port",
        action="append",
        type=int,
        default=[],
        help="additional occupied service port to avoid; repeat as needed",
    )
    parser.add_argument(
        "--serial-port",
        action="append",
        default=[],
        help="probe only this newly connected serial path; repeat as needed",
    )
    parser.add_argument(
        "--name",
        action="append",
        default=[],
        help="friendly robot name in discovery order; repeat for multiple robots",
    )
    parser.add_argument(
        "--no-prompt",
        action="store_true",
        help="keep saved/default names instead of asking interactively",
    )
    parser.add_argument("--root", type=Path, default=DEFAULT_ROOT)
    parser.add_argument("--show", action="store_true")
    parser.add_argument("--list", action="store_true", help=argparse.SUPPRESS)
    args = parser.parse_args()

    manifest_path = args.root / "devices.json"
    if args.show or args.list:
        manifest = load_manifest(manifest_path)
        if args.list:
            list_manifest_rows(manifest)
        else:
            print_manifest(manifest, manifest_path)
        return 0
    if not 1 <= args.servos <= 253:
        parser.error("--servos must be from 1 to 253")
    if not 1 <= args.base_port <= 65535:
        parser.error("--base-port must be from 1 to 65535")
    if not 1 <= args.runtime_port <= 65535:
        parser.error("--runtime-port must be from 1 to 65535")
    try:
        stack_instance = normalize_stack_instance(args.instance)
    except ValueError as exc:
        parser.error(str(exc))
    if any(not 1 <= value <= 65535 for value in args.reserved_port):
        parser.error("--reserved-port must be from 1 to 65535")

    if args.existing_ros2:
        if not 1 <= args.rosbridge_port <= 65535:
            parser.error("--rosbridge-port must be from 1 to 65535")
        if not args.required_topic:
            parser.error("--required-topic is required with --existing-ros2")
        if not args.capability:
            parser.error("--capability is required with --existing-ros2")
        previous = load_manifest(manifest_path)
        key = existing_ros2_key(args.rosbridge_host, args.rosbridge_port)
        old = next(
            (
                item for item in previous["devices"]
                if isinstance(item, dict) and item.get("key") == key
            ),
            {},
        )
        occupied_ports = discover_occupied_service_ports()
        service_port = old.get("service_port")
        if (
            isinstance(service_port, bool)
            or not isinstance(service_port, int)
            or not 1 <= service_port <= 65535
        ):
            service_port = args.base_port
            while service_port in {args.runtime_port, *args.reserved_port, *occupied_ports}:
                service_port += 1
        if service_port > 65535:
            raise ValueError("no HTTP port remains for the ROS robot service")
        device_id = str(old.get("device_id") or f"{socket.gethostname()}-{key}")
        name = normalize_device_name(
            args.name[0] if args.name else old.get("name"),
            fallback="ROS 2 Robot",
        )
        device_dir = args.root / "devices" / key
        config_value = {
            "version": 1,
            "device_id": device_id,
            "name": name,
            "adapter": "existing_ros2",
            "mode": "read_only",
            "host": args.rosbridge_host,
            "rosbridge_port": args.rosbridge_port,
            "required_topics": args.required_topic,
            "capabilities": args.capability,
        }
        provider = ExistingRos2Monitor(
            ExistingRos2Config(
                host=args.rosbridge_host,
                port=args.rosbridge_port,
                required_topics=tuple(args.required_topic),
                capabilities=tuple(args.capability),
            ),
            device_id=device_id,
        )
        state = provider.connect()
        connected = state.connected
        connection_error = state.error
        provider.close()
        if not connected:
            raise ValueError(
                "the existing ROS 2 robot was not healthy through ROSBridge: "
                + (connection_error or "required topics were unavailable")
            )
        config_path = device_dir / "device.json"
        save_device_config(config_value, config_path)
        entry = {
            "key": key,
            "name": name,
            "device_id": device_id,
            "adapter": "existing_ros2",
            "rosbridge_host": args.rosbridge_host,
            "rosbridge_port": args.rosbridge_port,
            "required_topics": list(args.required_topic),
            "service_port": service_port,
            "config": str(config_path),
            "token_file": str(device_dir / "auth.token"),
            "unit": hardware_unit_name(key, stack_instance),
        }
        manifest = {"version": FLEET_VERSION, "devices": [entry]}
        save_manifest(manifest_path, manifest)
        print()
        print_manifest(manifest, manifest_path)
        print()
        print("The existing ROS 2 robot was configured read-only. No ROS messages were published.")
        print("Next: ./pair.sh --all")
        return 0

    candidates = (
        deduplicate_serial_ports(args.serial_port)
        if args.serial_port
        else discover_serial_ports()
    )
    if not candidates:
        raise ValueError("no serial devices found; connect the robots and run ./discover.sh")

    detected: list[tuple[str, list[int]]] = []
    print(f"Checking {len(candidates)} serial device(s) using position reads only...")
    for port in candidates:
        print(f"  {port}")
        joints = tuple(
            SerialJointSpec(name=f"servo_{servo_id}", servo_id=servo_id)
            for servo_id in range(1, args.servos + 1)
        )
        try:
            result = probe_serial(
                SerialJointConfig(port=port, baudrate=args.baudrate, joints=joints)
            )
        except Exception as exc:
            print(f"    skipped: {exc}")
            continue
        servo_ids = sorted(
            int(reading["servo_id"])
            for reading in result["readings"].values()
        )
        if not servo_ids:
            print("    skipped: no servos responded")
            continue
        detected.append((port, servo_ids))
        print(f"    robot found: {len(servo_ids)} servo(s)")

    if not detected:
        raise ValueError(
            "serial devices were present, but no robot servos responded; "
            "check robot power, baudrate, and the --servos scan range"
        )

    previous = load_manifest(manifest_path)
    occupied_ports = discover_occupied_service_ports()
    previous_ports = {
        int(item["service_port"])
        for item in previous["devices"]
        if (
            isinstance(item, dict)
            and isinstance(item.get("service_port"), int)
            and not isinstance(item.get("service_port"), bool)
        )
    }
    entries = build_fleet_entries(
        detected,
        root=args.root,
        hostname=socket.gethostname(),
        base_port=args.base_port,
        previous=previous["devices"],
        reserved_ports={
            args.runtime_port,
            *args.reserved_port,
            *(occupied_ports - previous_ports),
        },
        stack_instance=stack_instance,
    )
    assign_device_names(
        entries,
        args.name,
        prompt=not args.no_prompt and sys.stdin.isatty(),
    )
    for entry in entries:
        save_device_config(
            {
                "version": 1,
                "device_id": entry["device_id"],
                "name": entry["name"],
                "adapter": "serial_joint",
                "mode": "read_only",
                "port": entry["serial_port"],
                "baudrate": args.baudrate,
                "servos": [
                    {"id": servo_id, "name": f"servo_{servo_id}"}
                    for servo_id in entry["servo_ids"]
                ],
            },
            entry["config"],
        )
    manifest = {"version": FLEET_VERSION, "devices": entries}
    save_manifest(manifest_path, manifest)
    print()
    print_manifest(manifest, manifest_path)
    print()
    print("All responding robots were configured. No torque or motion writes were sent.")
    print("Next: ./pair.sh --all")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (OSError, ValueError, json.JSONDecodeError) as exc:
        print(f"Automatic configuration failed: {exc}", file=sys.stderr)
        raise SystemExit(1) from None
