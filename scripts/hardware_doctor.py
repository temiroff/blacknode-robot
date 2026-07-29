"""Check host readiness for the hardware service without commanding motion."""

from __future__ import annotations

import argparse
import importlib.util
import os
from pathlib import Path
import platform
import shutil
import subprocess
import sys


_USE_COLOR = sys.stdout.isatty() and "NO_COLOR" not in os.environ
_GREEN = "\u001b[32m"
_RED = "\u001b[31m"
_YELLOW = "\u001b[33m"
_RESET = "\u001b[0m"


def paint(value: str, color: str) -> str:
    return f"{color}{value}{_RESET}" if _USE_COLOR else value


def check(label: str, ok: bool, detail: str) -> bool:
    status = paint("[OK]", _GREEN) if ok else paint("[FAIL]", _RED)
    print(f"{status} {label}: {detail}")
    return ok


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bus", type=int, default=1)
    parser.add_argument("--address", type=lambda value: int(value, 0), default=0x7A)
    parser.add_argument("--probe-address", action="store_true", help="perform an optional read-only address probe")
    args = parser.parse_args()

    results: list[bool] = []
    results.append(check("Python", sys.version_info >= (3, 11), platform.python_version()))
    results.append(check("Linux", platform.system() == "Linux", platform.platform()))
    smbus_ready = importlib.util.find_spec("smbus2") is not None
    results.append(check("smbus2", smbus_ready, "installed" if smbus_ready else "install with: pip install smbus2"))
    serial_ready = importlib.util.find_spec("serial") is not None
    results.append(check("pyserial", serial_ready, "installed" if serial_ready else "install with: pip install pyserial"))
    i2cdetect = shutil.which("i2cdetect")
    results.append(check("i2c-tools", i2cdetect is not None, "i2cdetect found" if i2cdetect else "install with: sudo apt install i2c-tools"))

    bus_path = Path(f"/dev/i2c-{args.bus}")
    results.append(check("I2C bus", bus_path.exists(), str(bus_path)))
    access = bus_path.exists() and os.access(bus_path, os.R_OK | os.W_OK)
    results.append(check("I2C access", access, "read/write access" if access else "check i2c group membership and reconnect"))

    address_ok: bool | None = None
    if args.probe_address:
        try:
            if i2cdetect is None:
                raise RuntimeError("i2cdetect is not installed")
            scan = subprocess.run(
                [i2cdetect, "-y", "-a", str(args.bus), hex(args.address), hex(args.address)],
                capture_output=True,
                text=True,
                check=False,
            )
            token = f"{args.address:02x}"
            responded = token in scan.stdout.lower().split()
            detail = (
                f"0x{args.address:02x} detected on bus {args.bus}"
                if responded
                else f"0x{args.address:02x} not detected; scan output: {scan.stdout.strip() or scan.stderr.strip()}"
            )
            address_ok = responded
            results.append(check("I2C address", responded, detail))
        except Exception as exc:
            address_ok = False
            results.append(check("I2C address", False, f"0x{args.address:02x} did not respond: {exc}"))
    else:
        print("[SKIP] I2C address: use --probe-address for an optional read-only probe")

    try:
        import blacknode_robot.devices  # noqa: F401

        results.append(check("blacknode-robot/devices", True, "package import succeeded"))
    except Exception as exc:
        results.append(check("blacknode-robot/devices", False, str(exc)))

    passed = sum(results)
    if address_ok is False:
        print()
        print(paint("CHECK I2C CONNECTION", _RED))
        print(paint("  Verify controller power, SDA, SCL, and GND wiring.", _YELLOW))
        print(paint(f"  Expected device address: 0x{args.address:02x} on bus {args.bus}.", _YELLOW))
    print(f"\n{passed}/{len(results)} required checks passed")
    return 0 if all(results) else 1


if __name__ == "__main__":
    raise SystemExit(main())
