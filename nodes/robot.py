"""Generic robot setup nodes.

This package owns the product-level robot UX: USB discovery, permissions,
driver process launch, and a standard robot profile. Robot-specific packages
provide driver descriptors; transport packages such as blacknode-ros2 provide
topic adapters and motion nodes.
"""
from __future__ import annotations

import glob
import grp
import os
import pwd
import shlex
import signal
import stat
import subprocess
import sys
import time
from collections import defaultdict
from typing import Any

from blacknode.node import Any as AnyPort
from blacknode.node import Bool, Dict, Enum, Float, Int, List, Text, node

_CATEGORY = "Robot"
_SERIAL_GLOBS = ("/dev/serial/by-id/*", "/dev/ttyACM*", "/dev/ttyUSB*")
_COMMON_SERIAL_GROUPS = {"dialout", "uucp", "plugdev", "tty"}
_managed_drivers: dict[str, subprocess.Popen] = {}


def _current_username() -> str:
    try:
        return pwd.getpwuid(os.geteuid()).pw_name
    except Exception:  # noqa: BLE001
        return os.environ.get("USER", "")


def _group_name(gid: int) -> str:
    try:
        return grp.getgrgid(gid).gr_name
    except Exception:  # noqa: BLE001
        return str(gid)


def _user_group_names() -> list[str]:
    gids = {os.getegid(), *os.getgroups()}
    names = {_group_name(gid) for gid in gids}
    return sorted(name for name in names if name)


def _read_small_file(path: str) -> str:
    try:
        with open(path, encoding="utf-8") as handle:
            return handle.read().strip()
    except OSError:
        return ""


def _serial_candidate_paths() -> list[str]:
    paths: list[str] = []
    for pattern in _SERIAL_GLOBS:
        paths.extend(sorted(glob.glob(pattern)))

    result: list[str] = []
    seen_realpaths: set[str] = set()
    for path in paths:
        real = os.path.realpath(path)
        if not os.path.exists(real) or real in seen_realpaths:
            continue
        seen_realpaths.add(real)
        result.append(path)
    return result


def _usb_attrs_for_serial(path: str) -> dict[str, str]:
    tty_name = os.path.basename(os.path.realpath(path))
    current = os.path.join("/sys/class/tty", tty_name, "device")
    attrs: dict[str, str] = {}
    for _ in range(8):
        for key in ("idVendor", "idProduct", "manufacturer", "product", "serial"):
            value = _read_small_file(os.path.join(current, key))
            if value:
                attrs[key] = value
        if attrs:
            return attrs
        parent = os.path.dirname(current)
        if not parent or parent == current:
            break
        current = parent
    return attrs


def _serial_permission_fixes(device: dict[str, Any]) -> list[str]:
    if device.get("accessible"):
        return []

    path = str(device.get("path") or device.get("real_path") or "the device")
    group = str(device.get("group") or "")
    user = _current_username() or "$USER"
    user_groups = set(_user_group_names())
    fixes: list[str] = []

    if group and group not in user_groups:
        fixes.append(f"sudo usermod -aG {group} {user}")
        fixes.append("log out and back in, or run a new shell with: newgrp " + group)
    elif group in _COMMON_SERIAL_GROUPS:
        fixes.append(f"check udev permissions for {path}; expected write access for group {group}")
    else:
        fixes.append(f"grant stable serial access for {path} with a udev rule")

    vendor_id = str(device.get("vendor_id") or "")
    product_id = str(device.get("product_id") or "")
    if vendor_id and product_id:
        rule_group = group if group in _COMMON_SERIAL_GROUPS else "dialout"
        rule = (
            f'SUBSYSTEM=="tty", ATTRS{{idVendor}}=="{vendor_id}", '
            f'ATTRS{{idProduct}}=="{product_id}", MODE="0660", GROUP="{rule_group}"'
        )
        fixes.append(f"udev rule: {rule}")

    fixes.append(f"temporary test only: sudo chmod a+rw {path}")
    return fixes


def _serial_device_info(path: str, probe_open: bool = False) -> dict[str, Any]:
    real = os.path.realpath(path)
    attrs = _usb_attrs_for_serial(path)
    info: dict[str, Any] = {
        "path": path,
        "real_path": real,
        "name": os.path.basename(real),
        "by_id": path if path.startswith("/dev/serial/by-id/") else "",
        "vendor_id": attrs.get("idVendor", ""),
        "product_id": attrs.get("idProduct", ""),
        "manufacturer": attrs.get("manufacturer", ""),
        "product": attrs.get("product", ""),
        "serial": attrs.get("serial", ""),
        "exists": os.path.exists(real),
        "readable": False,
        "writable": False,
        "accessible": False,
        "owner": "",
        "group": "",
        "mode": "",
        "probe": "",
        "fixes": [],
    }
    if not info["exists"]:
        info["fixes"] = [f"device disappeared: {path}"]
        return info

    try:
        st = os.stat(real)
        info["owner"] = pwd.getpwuid(st.st_uid).pw_name
        info["group"] = _group_name(st.st_gid)
        info["mode"] = oct(stat.S_IMODE(st.st_mode))
    except Exception:  # noqa: BLE001
        pass

    info["readable"] = os.access(real, os.R_OK)
    info["writable"] = os.access(real, os.W_OK)
    info["accessible"] = bool(info["readable"] and info["writable"])

    if probe_open:
        flags = os.O_RDWR | os.O_NONBLOCK
        if hasattr(os, "O_NOCTTY"):
            flags |= os.O_NOCTTY
        try:
            fd = os.open(real, flags)
        except PermissionError as exc:
            info["probe"] = f"permission_denied: {exc}"
            info["accessible"] = False
        except OSError as exc:
            info["probe"] = f"open_failed: {exc}"
        else:
            os.close(fd)
            info["probe"] = "open_ok"
            info["accessible"] = True

    info["fixes"] = _serial_permission_fixes(info)
    return info


def _serial_device_matches_filter(device: dict[str, Any], text_filter: str) -> bool:
    if not text_filter:
        return True
    haystack = " ".join(
        str(device.get(key) or "")
        for key in ("path", "real_path", "manufacturer", "product", "serial", "vendor_id", "product_id")
    ).lower()
    return text_filter.lower() in haystack


def _recommended_serial_device(devices: list[dict[str, Any]]) -> dict[str, Any]:
    for prefer_stable in (True, False):
        for device in devices:
            if device.get("accessible") and (not prefer_stable or device.get("by_id")):
                return device
    for prefer_stable in (True, False):
        for device in devices:
            if not prefer_stable or device.get("by_id"):
                return device
    return {}


def _format_serial_device(device: dict[str, Any]) -> str:
    path = str(device.get("path") or "")
    real = str(device.get("real_path") or "")
    label_parts = [
        str(device.get("manufacturer") or "").strip(),
        str(device.get("product") or "").strip(),
    ]
    label = " ".join(part for part in label_parts if part) or str(device.get("name") or path)
    access = "access OK" if device.get("accessible") else "access blocked"
    suffix = f" -> {real}" if real and real != path else ""
    return f"{path}{suffix}: {label} ({access}, group={device.get('group') or '?'}, mode={device.get('mode') or '?'})"


def _driver_from_ctx(ctx: dict) -> dict[str, Any]:
    driver = ctx.get("driver") if isinstance(ctx.get("driver"), dict) else {}
    if driver:
        return dict(driver)
    return robot_driver_descriptor(ctx)["driver"]


def _driver_command(driver: dict[str, Any], serial_port: str) -> str:
    template = str(driver.get("command_template") or "").strip()
    if not template:
        return ""
    values = defaultdict(str)
    values.update({
        "serial_port": serial_port,
        "python": sys.executable,
        "host": str(driver.get("host") or "127.0.0.1"),
        "port": str(driver.get("port") or 9090),
        "state_topic": str(driver.get("state_topic") or "/joint_states"),
        "command_topic": str(driver.get("command_topic") or "/joint_commands"),
        "config_topic": str(driver.get("config_topic") or ""),
    })
    return template.format_map(values)


def _driver_running(run_id: str) -> bool:
    proc = _managed_drivers.get(run_id)
    if proc is None:
        return False
    if proc.poll() is None:
        return True
    _managed_drivers.pop(run_id, None)
    return False


def _terminate_process(proc: subprocess.Popen) -> bool:
    if proc.poll() is not None:
        return False
    try:
        os.killpg(proc.pid, signal.SIGTERM)
    except ProcessLookupError:
        return False
    except Exception:
        proc.terminate()
    try:
        proc.wait(timeout=3)
    except subprocess.TimeoutExpired:
        try:
            os.killpg(proc.pid, signal.SIGKILL)
        except Exception:
            proc.kill()
        proc.wait(timeout=3)
    return True


def _stop_driver(run_id: str) -> int:
    proc = _managed_drivers.pop(run_id, None)
    if proc is None:
        return 0
    return 1 if _terminate_process(proc) else 0


def runtime_status() -> dict[str, Any]:
    """Return Blacknode-started robot driver processes still known to this process."""
    live_runs: list[dict[str, Any]] = []
    for run_id, proc in list(_managed_drivers.items()):
        if proc.poll() is None:
            live_runs.append({"run_id": run_id, "pid": proc.pid})
        else:
            _managed_drivers.pop(run_id, None)
    return {
        "ok": True,
        "managed_runs": live_runs,
        "detached_count": 0,
        "active": bool(live_runs),
    }


def stop_runtime_services() -> dict[str, Any]:
    """Stop every robot driver process this Blacknode process started.

    Terminating with SIGTERM (via _terminate_process) is what lets each
    driver's own shutdown handler run -- for feetech_bus_driver.py that
    means disabling torque on every servo before it exits, so this is the
    call that must be wired into the editor's "Stop all" action, not just
    a housekeeping cleanup.
    """
    status_before = runtime_status()
    stopped = 0
    for run_id in list(_managed_drivers):
        stopped += _stop_driver(run_id)
    return {
        "ok": True,
        "active_before": status_before,
        "stopped": {"managed_runs": stopped},
        "report": f"stopped {stopped} robot driver process(es)",
    }


@node(
    name="RobotUSBDiscovery",
    category=_CATEGORY,
    description="Discover USB serial robot ports and report access/permission fixes.",
    inputs={
        "refresh": AnyPort,
        "port_filter": Text(default=""),
        "probe_open": Bool(default=False),
    },
    outputs={"found": Bool, "ready": Bool, "devices": List, "recommended": Dict, "permissions": Dict, "report": Text},
)
def robot_usb_discovery(ctx: dict) -> dict:
    text_filter = str(ctx.get("port_filter") or "").strip()
    probe_open = bool(ctx.get("probe_open", False))
    devices = [
        _serial_device_info(path, probe_open=probe_open)
        for path in _serial_candidate_paths()
    ]
    devices = [device for device in devices if _serial_device_matches_filter(device, text_filter)]
    recommended = _recommended_serial_device(devices)
    found = bool(devices)
    ready = bool(recommended.get("accessible"))

    fixes: list[str] = []
    seen_fixes: set[str] = set()
    for device in devices:
        for fix in device.get("fixes") or []:
            if fix not in seen_fixes:
                seen_fixes.add(str(fix))
                fixes.append(str(fix))

    permissions = {
        "user": _current_username(),
        "groups": _user_group_names(),
        "fixes": fixes,
    }

    lines = ["USB robot discovery"]
    if not devices:
        if text_filter:
            lines.append(f"no USB serial ports matched filter: {text_filter}")
        else:
            lines.append("no USB serial ports detected (/dev/serial/by-id/*, /dev/ttyACM*, /dev/ttyUSB*)")
        lines.append("plug in the robot with a USB data cable, power it on, then cook this node again")
    else:
        lines.append(f"found {len(devices)} USB serial candidate(s)")
        for device in devices[:8]:
            lines.append("- " + _format_serial_device(device))
        if len(devices) > 8:
            lines.append(f"- ... plus {len(devices) - 8} more")
        if recommended:
            lines.append(f"recommended_port: {recommended.get('path')}")
        if ready:
            lines.append("=> READY: at least one robot USB serial port is readable and writable")
        else:
            lines.append("=> NOT READY: USB serial device exists but Blacknode does not have read/write access")
            for fix in fixes[:6]:
                lines.append("FIX: " + fix)

    return {
        "found": found,
        "ready": ready,
        "devices": devices,
        "recommended": recommended,
        "permissions": permissions,
        "report": "\n".join(lines),
    }


@node(
    name="RobotDriverDescriptor",
    category=_CATEGORY,
    description="Declare how a robot driver should be launched from a discovered serial port.",
    inputs={
        "driver_id": Text(default="generic"),
        "name": Text(default="Generic Robot Driver"),
        "command_template": Text(default=""),
        "transport": Enum(["none", "native", "rosbridge"], default="rosbridge"),
        "host": Text(default="127.0.0.1"),
        "port": Int(default=9090),
        "state_topic": Text(default="/joint_states"),
        "command_topic": Text(default="/joint_commands"),
        "config_topic": Text(default="/joint_config"),
        "units": Enum(["radians", "degrees"], default="degrees"),
        "match_vendor_id": Text(default=""),
        "match_product_id": Text(default=""),
    },
    outputs={"driver": Dict, "report": Text},
)
def robot_driver_descriptor(ctx: dict) -> dict:
    driver = {
        "id": str(ctx.get("driver_id") or "generic").strip() or "generic",
        "name": str(ctx.get("name") or "Generic Robot Driver").strip() or "Generic Robot Driver",
        "command_template": str(ctx.get("command_template") or "").strip(),
        "transport": str(ctx.get("transport") or "rosbridge"),
        "host": str(ctx.get("host") or "127.0.0.1"),
        "port": int(ctx.get("port") or 9090),
        "state_topic": str(ctx.get("state_topic") or "/joint_states"),
        "command_topic": str(ctx.get("command_topic") or "/joint_commands"),
        "config_topic": str(ctx.get("config_topic") or "/joint_config"),
        "units": str(ctx.get("units") or "degrees"),
        "match": {
            "vendor_id": str(ctx.get("match_vendor_id") or "").strip().lower(),
            "product_id": str(ctx.get("match_product_id") or "").strip().lower(),
        },
    }
    report = f"driver descriptor: {driver['name']} ({driver['id']})"
    if driver["command_template"]:
        report += "\nlaunch template: " + driver["command_template"]
    else:
        report += "\nno launch command yet; connect a robot-specific driver descriptor"
    return {"driver": driver, "report": report}


@node(
    name="RobotDriverLauncher",
    category=_CATEGORY,
    description="Start or stop a generic robot driver process using a descriptor command template.",
    inputs={
        "trigger": AnyPort,
        "action": Enum(["check", "start", "stop"], default="check"),
        "run_id": Text(default="robot_driver"),
        "driver": Dict,
        "usb": Dict,
        "serial_port": Text(default=""),
        "wait_seconds": Float(default=0.0),
    },
    outputs={"running": Bool, "run_id": Text, "driver": Dict, "command": Text, "report": Text},
)
def robot_driver_launcher(ctx: dict) -> dict:
    run_id = str(ctx.get("run_id") or "robot_driver").strip() or "robot_driver"
    action = str(ctx.get("action") or "check").strip().lower()
    driver = _driver_from_ctx(ctx)
    usb = ctx.get("usb") if isinstance(ctx.get("usb"), dict) else {}
    recommended = usb.get("recommended") if isinstance(usb.get("recommended"), dict) else {}
    serial_port = str(ctx.get("serial_port") or recommended.get("path") or "").strip()
    command = _driver_command(driver, serial_port)

    if action == "stop":
        stopped = _stop_driver(run_id)
        return {
            "running": False,
            "run_id": run_id,
            "driver": driver,
            "command": command,
            "report": f"stopped {stopped} robot driver process(es)",
        }

    if action == "check":
        running = _driver_running(run_id)
        return {
            "running": running,
            "run_id": run_id,
            "driver": driver,
            "command": command,
            "report": f"robot driver {'running' if running else 'not running'}: {run_id}",
        }

    if not command:
        return {
            "running": False,
            "run_id": run_id,
            "driver": driver,
            "command": "",
            "report": "robot driver start BLOCKED: no command_template in driver descriptor",
        }
    if "{serial_port}" in str(driver.get("command_template") or "") and not serial_port:
        return {
            "running": False,
            "run_id": run_id,
            "driver": driver,
            "command": command,
            "report": "robot driver start BLOCKED: no serial port available from USB discovery",
        }

    _stop_driver(run_id)
    try:
        args = shlex.split(command)
        proc = subprocess.Popen(args, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, start_new_session=True)
    except Exception as exc:  # noqa: BLE001
        return {
            "running": False,
            "run_id": run_id,
            "driver": driver,
            "command": command,
            "report": f"robot driver start FAILED: {exc}",
        }
    _managed_drivers[run_id] = proc

    wait_seconds = max(0.0, float(ctx.get("wait_seconds") or 0.0))
    if wait_seconds > 0:
        deadline = time.time() + wait_seconds
        while time.time() < deadline:
            if proc.poll() is not None:
                _managed_drivers.pop(run_id, None)
                return {
                    "running": False,
                    "run_id": run_id,
                    "driver": driver,
                    "command": command,
                    "report": f"robot driver exited during startup with code {proc.returncode}",
                }
            time.sleep(0.1)

    return {
        "running": True,
        "run_id": run_id,
        "driver": driver,
        "command": command,
        "report": f"robot driver running: {driver.get('name') or run_id} (pid {proc.pid})",
    }


@node(
    name="RobotDiscovery",
    category=_CATEGORY,
    description="One generic robot setup node: discover USB, optionally launch a driver, and output a standard robot profile.",
    inputs={
        "trigger": AnyPort,
        "driver": Dict,
        "action": Enum(["check", "start", "stop"], default="check"),
        "run_id": Text(default="robot_driver"),
        "port_filter": Text(default=""),
        "probe_open": Bool(default=False),
        "require_usb": Bool(default=False),
        "serial_port": Text(default=""),
        "host": Text(default="127.0.0.1"),
        "port": Int(default=9090),
        "state_topic": Text(default="/joint_states"),
        "command_topic": Text(default="/joint_commands"),
        "config_topic": Text(default="/joint_config"),
        "units": Enum(["radians", "degrees"], default="degrees"),
    },
    outputs={
        "found": Bool,
        "ready": Bool,
        "usb_ready": Bool,
        "driver_running": Bool,
        "usb": Dict,
        "driver": Dict,
        "robot": Dict,
        "report": Text,
    },
)
def robot_discovery(ctx: dict) -> dict:
    usb = robot_usb_discovery({
        "port_filter": ctx.get("port_filter", ""),
        "probe_open": bool(ctx.get("probe_open", False)),
    })
    driver = _driver_from_ctx(ctx)
    serial_port = str(ctx.get("serial_port") or (usb.get("recommended") or {}).get("path") or "").strip()
    driver_result = robot_driver_launcher({
        "action": ctx.get("action", "check"),
        "run_id": ctx.get("run_id", "robot_driver"),
        "driver": driver,
        "usb": usb,
        "serial_port": serial_port,
        "wait_seconds": 0.25,
    })

    usb_ready = bool(usb.get("ready"))
    driver_running = bool(driver_result.get("running"))
    require_usb = bool(ctx.get("require_usb", False))
    has_driver_command = bool(driver.get("command_template"))
    ready = bool(driver_running and (usb_ready or not require_usb))

    host = str(driver.get("host") or ctx.get("host") or "127.0.0.1")
    port = int(driver.get("port") or ctx.get("port") or 9090)
    state_topic = str(driver.get("state_topic") or ctx.get("state_topic") or "/joint_states")
    command_topic = str(driver.get("command_topic") or ctx.get("command_topic") or "/joint_commands")
    config_topic = str(driver.get("config_topic") or ctx.get("config_topic") or "/joint_config")
    units = str(driver.get("units") or ctx.get("units") or "degrees")
    robot = {
        "host": host,
        "port": port,
        "state_topic": state_topic,
        "command_topic": command_topic,
        "config_topic": config_topic,
        "units": units,
        "connected": False,
        "ready": ready,
        "joints": [],
        "pose": {},
        "limits": {},
        "commands_allowed": None,
        "usb": {
            "found": bool(usb.get("found")),
            "ready": usb_ready,
            "recommended": usb.get("recommended") or {},
            "devices": usb.get("devices") or [],
            "permissions": usb.get("permissions") or {},
        },
        "driver": {
            **driver,
            "running": driver_running,
            "run_id": str(ctx.get("run_id") or "robot_driver"),
            "command": driver_result.get("command") or "",
        },
        "interface": {
            "kind": driver.get("transport") or "none",
            "verified": False,
        },
        "error": "",
    }
    if require_usb and not usb_ready:
        robot["error"] = "USB robot device is not ready"
    elif not has_driver_command:
        robot["error"] = "no robot driver descriptor connected"
    elif not driver_running:
        robot["error"] = "robot driver is not running"

    lines = ["Robot discovery"]
    lines.append("")
    lines.append("[USB]")
    lines.append(str(usb.get("report") or "").strip())
    lines.append("")
    lines.append("[Driver]")
    lines.append(str(driver_result.get("report") or "").strip())
    lines.append("")
    if ready:
        lines.append("=> READY: robot driver is running; verify the interface with a transport package node")
    elif require_usb and not usb_ready:
        lines.append("=> NEXT: fix USB device access, then start the driver")
    elif not has_driver_command:
        lines.append("=> NEXT: connect a robot-specific driver descriptor")
    elif usb_ready and not driver_running:
        lines.append("=> NEXT: start the robot driver")
    else:
        lines.append("=> NEXT: configure and start the robot driver")

    return {
        "found": bool(usb.get("found")),
        "ready": ready,
        "usb_ready": usb_ready,
        "driver_running": driver_running,
        "usb": {
            "found": bool(usb.get("found")),
            "ready": usb_ready,
            "devices": usb.get("devices") or [],
            "recommended": usb.get("recommended") or {},
            "permissions": usb.get("permissions") or {},
        },
        "driver": driver,
        "robot": robot,
        "report": "\n".join(lines),
    }
