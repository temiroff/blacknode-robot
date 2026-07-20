#!/usr/bin/env python3
"""Compatibility entrypoint for the Feetech driver now owned by blacknode-drivers.

Existing robot profiles and saved driver descriptors may continue launching
this path. New integrations should enable blacknode-drivers/feetech-ros2.
"""
from __future__ import annotations

import os
import runpy
import sys
from pathlib import Path


def _runtime_path() -> Path:
    configured = str(os.environ.get("BLACKNODE_FEETECH_DRIVER") or "").strip()
    candidates = []
    if configured:
        candidates.append(Path(configured).expanduser())
    candidates.append(
        Path(__file__).resolve().parents[2]
        / "blacknode-drivers"
        / "components"
        / "feetech-ros2"
        / "runtime"
        / "feetech_bus_driver.py"
    )
    for candidate in candidates:
        resolved = candidate.resolve()
        if resolved.is_file() and resolved != Path(__file__).resolve():
            return resolved
    searched = ", ".join(str(path) for path in candidates)
    raise FileNotFoundError(
        "Feetech runtime is provided by blacknode-drivers/feetech-ros2; "
        f"enable or install that component. Searched: {searched}"
    )


def _load_compatibility_symbols() -> None:
    try:
        exported = runpy.run_path(str(_runtime_path()), run_name="blacknode_feetech_runtime")
    except FileNotFoundError:
        return
    globals().update({name: value for name, value in exported.items() if not name.startswith("__")})


if __name__ == "__main__":
    try:
        runpy.run_path(str(_runtime_path()), run_name="__main__")
    except FileNotFoundError as exc:
        print(f"UNAVAILABLE: {exc}", file=sys.stderr)
        raise SystemExit(2) from exc
else:
    _load_compatibility_symbols()
