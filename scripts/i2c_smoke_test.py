"""Safe local smoke test for an I2C mobile-base adapter.

Run with the wheels lifted. The default action only connects and stops. Motion
requires the explicit ``--forward`` option and always ends with stop/disarm.
"""

from __future__ import annotations

import argparse
import time

from blacknode_robot.devices import I2CMecanumBase, I2CMecanumConfig, MobileBaseCommand


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--bus", type=int, default=1)
    parser.add_argument("--address", type=lambda value: int(value, 0), default=0x7A)
    parser.add_argument("--forward", type=float, default=0.0, help="seconds of low-speed forward motion")
    args = parser.parse_args()

    base = I2CMecanumBase(config=I2CMecanumConfig(bus=args.bus, address=args.address))
    try:
        base.connect()
        base.stop()
        print("connected:", base.state().as_dict())
        if args.forward <= 0:
            print("stop-only smoke test passed")
            return 0

        base.arm()
        deadline = time.monotonic() + args.forward
        command = MobileBaseCommand(
            linear_x=0.05,
            expires_at=time.monotonic() + 0.5,
        )
        base.command(command)
        while time.monotonic() < deadline:
            base.watchdog_tick()
            time.sleep(0.05)
        print("motion smoke test completed")
        return 0
    finally:
        base.close()


if __name__ == "__main__":
    raise SystemExit(main())
