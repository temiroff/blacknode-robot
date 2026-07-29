"""Read-only serial actuator probe."""

from __future__ import annotations

import argparse

from blacknode_robot.devices.adapters.serial_joint import SerialJointConfig, SerialJointSpec, probe_serial


def parse_ids(value: str) -> list[int]:
    ids: list[int] = []
    for part in value.split(","):
        part = part.strip()
        if not part:
            continue
        if "-" in part:
            start, end = (int(item) for item in part.split("-", 1))
            ids.extend(range(start, end + 1))
        else:
            ids.append(int(part))
    return sorted(set(ids))


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", required=True)
    parser.add_argument("--baudrate", type=int, default=1_000_000)
    parser.add_argument("--ids", default="1-20")
    args = parser.parse_args()
    joints = tuple(SerialJointSpec(name=f"servo_{servo_id}", servo_id=servo_id) for servo_id in parse_ids(args.ids))
    result = probe_serial(SerialJointConfig(port=args.port, baudrate=args.baudrate, joints=joints))
    for name, reading in result["readings"].items():
        print(f"[OK] {name}: {reading['ticks']} ticks, {reading['degrees']:.2f} degrees")
    for error in result["errors"]:
        print(f"[SKIP] {error}")
    if not result["readings"]:
        print("No actuator responses. Check power, bus wiring, baudrate, and servo IDs.")
        return 1
    print(f"Read-only probe found {len(result['readings'])} actuator(s). No writes were sent.")
    return 0 if result["ok"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
