"""Generic robot setup nodes.

This package owns the product-level robot UX: USB discovery, permissions,
driver process launch, and a standard robot profile. Robot-specific packages
provide driver descriptors; transport packages such as blacknode-ros2 provide
topic adapters and motion nodes.
"""
from __future__ import annotations

import glob
import base64
import html
import json
import os
import re
import shlex
import signal
import stat
import subprocess
import sys
import time
from collections import defaultdict
from typing import Any

from blacknode.node import Any as AnyPort
from blacknode.node import Bool, Dict, Enum, Float, Image, Int, List, Text, node

try:
    import grp
except ImportError:  # Windows does not provide Unix group database APIs.
    grp = None

try:
    import pwd
except ImportError:  # Windows does not provide Unix password database APIs.
    pwd = None

try:
    import serial
    from serial.tools import list_ports as serial_list_ports
except ImportError:  # pyserial is optional until robot setup is installed.
    serial = None
    serial_list_ports = None

_CATEGORY = "Robot"
_SERIAL_GLOBS = ("/dev/serial/by-id/*", "/dev/ttyACM*", "/dev/ttyUSB*")
_COMMON_SERIAL_GROUPS = {"dialout", "uucp", "plugdev", "tty"}
_managed_drivers: dict[str, subprocess.Popen] = {}
_managed_driver_commands: dict[str, str] = {}
# Connection facts kept per running driver so a stop can disarm it (release
# torque) over rosbridge before killing the process — see _release_torque_best_effort.
_managed_driver_meta: dict[str, dict[str, Any]] = {}
_last_driver_exits: dict[str, dict[str, Any]] = {}


def _current_username() -> str:
    if pwd is not None and hasattr(os, "geteuid"):
        try:
            return pwd.getpwuid(os.geteuid()).pw_name
        except Exception:  # noqa: BLE001
            pass
    return os.environ.get("USER") or os.environ.get("USERNAME", "")


def _group_name(gid: int) -> str:
    if grp is not None:
        try:
            return grp.getgrgid(gid).gr_name
        except Exception:  # noqa: BLE001
            pass
    return str(gid)


def _user_group_names() -> list[str]:
    gids: set[int] = set()
    if hasattr(os, "getegid"):
        gids.add(os.getegid())
    if hasattr(os, "getgroups"):
        gids.update(os.getgroups())
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
    paths.extend(_pyserial_candidate_paths())

    result: list[str] = []
    seen_realpaths: set[str] = set()
    for path in paths:
        real = os.path.realpath(path)
        key = real.lower() if os.name == "nt" else real
        if not _serial_path_exists(path, real) or key in seen_realpaths:
            continue
        seen_realpaths.add(key)
        result.append(path)
    return result


def _pyserial_candidate_paths() -> list[str]:
    if serial_list_ports is None:
        return []
    try:
        devices = [str(port.device) for port in serial_list_ports.comports() if str(port.device or "").strip()]
    except Exception:  # noqa: BLE001
        return []
    # pyserial returns ports in registry/enumeration order, which is not stable
    # on Windows — so 'selection: 0' could point at either arm and shift on
    # replug. Sort so a numbered device (COM3 before COM4, ttyACM0 before
    # ttyACM1) always maps to the same index. Two identical arms are still best
    # pinned by serial via hardware_filter, but this makes plain index selection
    # deterministic.
    def _natural_key(device: str) -> tuple[str, int, str]:
        match = re.search(r"(\d+)\D*$", device)
        prefix = device[: match.start(1)] if match else device
        number = int(match.group(1)) if match else -1
        return (prefix.lower(), number, device.lower())

    return sorted(devices, key=_natural_key)


def _serial_path_exists(path: str, real: str) -> bool:
    if os.path.exists(real):
        return True
    return path in _pyserial_port_info_by_device()


def _pyserial_port_info_by_device() -> dict[str, Any]:
    if serial_list_ports is None:
        return {}
    try:
        return {str(port.device): port for port in serial_list_ports.comports() if str(port.device or "").strip()}
    except Exception:  # noqa: BLE001
        return {}


def _hex_id(value: Any) -> str:
    if value is None:
        return ""
    try:
        return f"{int(value):04x}"
    except Exception:  # noqa: BLE001
        return str(value)


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

    if device.get("source") == "pyserial":
        path = str(device.get("path") or "the serial port")
        if os.name == "nt":
            return [
                f"close any app already using {path}",
                "check Device Manager > Ports (COM & LPT) and install the USB-serial driver if needed",
                "unplug/replug the robot USB data cable, then cook this node again",
            ]
        return [
            f"close any app already using {path}",
            f"grant read/write access to {path}, then cook this node again",
        ]

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


def _pyserial_device_info(path: str, probe_open: bool = False) -> dict[str, Any] | None:
    port = _pyserial_port_info_by_device().get(path)
    if port is None:
        return None

    description = str(getattr(port, "description", "") or "")
    manufacturer = str(getattr(port, "manufacturer", "") or "")
    product = str(getattr(port, "product", "") or "") or description
    info: dict[str, Any] = {
        "path": path,
        "real_path": path,
        "name": path,
        "by_id": "",
        "vendor_id": _hex_id(getattr(port, "vid", None)),
        "product_id": _hex_id(getattr(port, "pid", None)),
        "manufacturer": manufacturer,
        "product": product,
        "serial": str(getattr(port, "serial_number", "") or ""),
        "description": description,
        "hwid": str(getattr(port, "hwid", "") or ""),
        "exists": True,
        "readable": True,
        "writable": True,
        "accessible": True,
        "owner": "",
        "group": "",
        "mode": "serial",
        "probe": "",
        "fixes": [],
        "source": "pyserial",
    }

    if probe_open:
        if serial is None:
            info["probe"] = "pyserial_not_installed"
            info["accessible"] = False
        else:
            try:
                handle = serial.Serial(path, baudrate=115200, timeout=0.1, write_timeout=0.1)
            except Exception as exc:  # noqa: BLE001
                info["probe"] = f"open_failed: {exc}"
                info["accessible"] = False
            else:
                handle.close()
                info["probe"] = "open_ok"
                info["accessible"] = True

    info["fixes"] = _serial_permission_fixes(info)
    return info


def _serial_device_info(path: str, probe_open: bool = False) -> dict[str, Any]:
    pyserial_info = _pyserial_device_info(path, probe_open=probe_open)
    if pyserial_info is not None and (os.name == "nt" or not os.path.exists(os.path.realpath(path))):
        return pyserial_info

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
        "source": "devfs",
    }
    if not info["exists"]:
        info["fixes"] = [f"device disappeared: {path}"]
        return info

    try:
        st = os.stat(real)
        info["owner"] = pwd.getpwuid(st.st_uid).pw_name if pwd is not None else str(st.st_uid)
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
    if device.get("source") == "pyserial":
        probe = str(device.get("probe") or "")
        if probe == "open_ok":
            access = "open OK"
        elif probe:
            access = "open failed"
        else:
            access = "OS detected"
        bits = [access]
        if device.get("vendor_id") and device.get("product_id"):
            bits.append(f"vid:pid={device['vendor_id']}:{device['product_id']}")
        if device.get("serial"):
            bits.append(f"serial={device['serial']}")
        return f"{path}{suffix}: {label} ({', '.join(bits)})"
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
        "control_topic": str(driver.get("control_topic") or "/robot_control"),
    })
    return template.format_map(values)


def _split_command(command: str) -> list[str]:
    if os.name != "nt":
        return shlex.split(command)

    import ctypes  # local import keeps Unix path lightweight

    argc = ctypes.c_int()
    shell32 = ctypes.windll.shell32
    kernel32 = ctypes.windll.kernel32
    shell32.CommandLineToArgvW.argtypes = [ctypes.c_wchar_p, ctypes.POINTER(ctypes.c_int)]
    shell32.CommandLineToArgvW.restype = ctypes.POINTER(ctypes.c_wchar_p)
    kernel32.LocalFree.argtypes = [ctypes.c_void_p]
    kernel32.LocalFree.restype = ctypes.c_void_p

    argv = shell32.CommandLineToArgvW(command, ctypes.byref(argc))
    if not argv:
        raise ValueError(f"could not parse command: {command}")
    try:
        return [argv[i] for i in range(argc.value)]
    finally:
        kernel32.LocalFree(ctypes.cast(argv, ctypes.c_void_p))


def _driver_running(run_id: str) -> bool:
    proc = _managed_drivers.get(run_id)
    if proc is None:
        return False
    if proc.poll() is None:
        return True
    _managed_drivers.pop(run_id, None)
    _managed_driver_commands.pop(run_id, None)
    stderr = (proc.stderr.read() if proc.stderr else "").strip()
    _last_driver_exits[run_id] = {
        "run_id": run_id,
        "returncode": proc.returncode,
        "error": stderr[-4000:],
    }
    return False


def _last_driver_exit_report(run_id: str) -> str:
    item = _last_driver_exits.get(run_id)
    if not item:
        return ""
    detail = str(item.get("error") or "").strip()
    suffix = f": {detail}" if detail else ""
    return f"; last exit code {item.get('returncode')}{suffix}"


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


def _release_torque_best_effort(run_id: str) -> None:
    """Tell the driver to drop torque before we kill its process.

    On Linux the driver's SIGTERM handler already disables torque on exit, but
    on Windows subprocess termination is a hard kill that never runs the handler,
    so the arm would stay stiff. Publishing the driver's own 'enter_teach'
    control message over rosbridge releases torque regardless of platform. Purely
    best-effort: if rosbridge isn't reachable or the transport is native, skip.
    """
    meta = _managed_driver_meta.get(run_id) or {}
    if str(meta.get("transport") or "").lower() != "rosbridge":
        return
    control_topic = str(meta.get("control_topic") or "").strip()
    if not control_topic:
        return
    try:
        from blacknode.pkg.blacknode_ros2 import rosbridge_runtime as rb

        rb.publish_string(
            str(meta.get("host") or "127.0.0.1"),
            int(meta.get("port") or 9090),
            control_topic,
            json.dumps({"action": "enter_teach"}),
            timeout=2.0,
        )
    except Exception:
        # A stop must never fail because the arm couldn't be reached to disarm.
        pass


def _stop_driver(run_id: str, release_torque: bool = False) -> int:
    if release_torque:
        _release_torque_best_effort(run_id)
    proc = _managed_drivers.pop(run_id, None)
    _managed_driver_commands.pop(run_id, None)
    _managed_driver_meta.pop(run_id, None)
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
            _managed_driver_commands.pop(run_id, None)
    try:
        from .profiles import calibration_runtime_status

        calibration = calibration_runtime_status()
    except Exception:
        calibration = {"sessions": [], "active": False}
    calibration_runs = [
        {"run_id": item["run_id"], "kind": "robot_calibration"}
        for item in calibration.get("sessions", [])
        if item.get("active")
    ]
    return {
        "ok": True,
        "managed_runs": live_runs + calibration_runs,
        "recent_exits": list(_last_driver_exits.values()),
        "calibrations": calibration.get("sessions", []),
        "detached_count": 0,
        "active": bool(live_runs or calibration_runs),
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
        stopped += _stop_driver(run_id, release_torque=True)
    try:
        from .profiles import stop_calibration_services

        stopped_calibrations = stop_calibration_services()
    except Exception:
        stopped_calibrations = 0
    return {
        "ok": True,
        "active_before": status_before,
        "stopped": {"managed_runs": stopped, "calibrations": stopped_calibrations},
        "report": f"stopped {stopped} robot driver process(es) and {stopped_calibrations} calibration session(s)",
    }


@node(
    name="RobotUSBDiscovery", component="capabilities",
    category=_CATEGORY,
    hidden=True,
    description="Discover USB serial robot ports and report access/permission fixes.",
    inputs={
        "refresh": AnyPort,
        "port_filter": Text(default=""),
        "match_vendor_id": Text(default=""),
        "match_product_id": Text(default=""),
        "probe_open": Bool(default=False),
    },
    outputs={
        "found": Bool,
        "ready": Bool,
        "port": Text,
        "serial": Text,
        "devices": List,
        "recommended": Dict,
        "usb": Dict,
        "permissions": Dict,
        "report": Text,
    },
)
def robot_usb_discovery(ctx: dict) -> dict:
    text_filter = str(ctx.get("port_filter") or "").strip()
    vendor_filter = str(ctx.get("match_vendor_id") or "").strip().lower().removeprefix("0x")
    product_filter = str(ctx.get("match_product_id") or "").strip().lower().removeprefix("0x")
    probe_open = bool(ctx.get("probe_open", False))
    devices = [
        _serial_device_info(path, probe_open=probe_open)
        for path in _serial_candidate_paths()
    ]
    devices = [device for device in devices if _serial_device_matches_filter(device, text_filter)]
    if vendor_filter:
        devices = [device for device in devices if str(device.get("vendor_id") or "").lower() == vendor_filter]
    if product_filter:
        devices = [device for device in devices if str(device.get("product_id") or "").lower() == product_filter]
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
        if vendor_filter or product_filter:
            expected = f"{vendor_filter or '*'}:{product_filter or '*'}"
            lines.append(f"no USB serial ports matched saved vid:pid {expected}")
        elif text_filter:
            lines.append(f"no USB serial ports matched filter: {text_filter}")
        else:
            lines.append("no USB serial ports detected (COM* on Windows; /dev/serial/by-id/*, /dev/ttyACM*, /dev/ttyUSB* on Linux)")
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
            lines.append("=> ADAPTER READY: the operating system exposes at least one USB serial adapter")
            if not probe_open:
                lines.append("note: discovery confirms the adapter, not robot power or servo communication; enable probe_open to test opening the port")
        else:
            lines.append("=> NOT READY: USB serial device exists but Blacknode does not have read/write access")
            for fix in fixes[:6]:
                lines.append("FIX: " + fix)

    usb = {
        "found": found,
        "ready": ready,
        "devices": devices,
        "recommended": recommended,
        "permissions": permissions,
        "report": "\n".join(lines),
    }
    return {
        "found": found,
        "ready": ready,
        "port": str(recommended.get("path") or ""),
        "serial": str(recommended.get("serial") or ""),
        "devices": devices,
        "recommended": recommended,
        "usb": usb,
        "permissions": permissions,
        "report": "\n".join(lines),
    }


@node(
    name="RobotDriverDescriptor", component="models",
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
        "control_topic": Text(default="/robot_control"),
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
        "control_topic": str(ctx.get("control_topic") or "/robot_control"),
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
    name="RobotDriverLauncher", component="models",
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
        stopped = _stop_driver(run_id, release_torque=True)
        return {
            "running": False,
            "run_id": run_id,
            "driver": driver,
            "command": command,
            "report": f"stopped {stopped} robot driver process(es); torque released",
        }

    if action == "check":
        running = _driver_running(run_id)
        return {
            "running": running,
            "run_id": run_id,
            "driver": driver,
            "command": command,
            "report": f"robot driver {'running' if running else 'not running'}: {run_id}{'' if running else _last_driver_exit_report(run_id)}",
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

    restarted = False
    if _driver_running(run_id) and _managed_driver_commands.get(run_id) == command:
        proc = _managed_drivers[run_id]
        return {
            "running": True,
            "run_id": run_id,
            "driver": driver,
            "command": command,
            "report": f"robot driver already running: {driver.get('name') or run_id} (pid {proc.pid})",
        }

    if _driver_running(run_id):
        # A profile change can alter joint ids, home ticks, limits, or the bus
        # command. Never claim the new descriptor is active while retaining a
        # process launched from the old command. An explicit Start/Run safely
        # replaces it through the normal shutdown path.
        _stop_driver(run_id)
        restarted = True

    _stop_driver(run_id)
    _last_driver_exits.pop(run_id, None)
    try:
        args = _split_command(command)
        proc = subprocess.Popen(
            args,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.PIPE,
            text=True,
            start_new_session=True,
        )
    except Exception as exc:  # noqa: BLE001
        return {
            "running": False,
            "run_id": run_id,
            "driver": driver,
            "command": command,
            "report": f"robot driver start FAILED: {exc}",
        }
    _managed_drivers[run_id] = proc
    _managed_driver_commands[run_id] = command
    _managed_driver_meta[run_id] = {
        "transport": str(driver.get("transport") or "rosbridge"),
        "host": str(driver.get("host") or "127.0.0.1"),
        "port": int(driver.get("port") or 9090),
        "control_topic": str(driver.get("control_topic") or "/robot_control"),
    }

    wait_seconds = max(0.0, float(ctx.get("wait_seconds") or 0.0))
    if wait_seconds > 0:
        deadline = time.time() + wait_seconds
        while time.time() < deadline:
            if proc.poll() is not None:
                _driver_running(run_id)
                return {
                    "running": False,
                    "run_id": run_id,
                    "driver": driver,
                    "command": command,
                    "report": f"robot driver exited during startup{_last_driver_exit_report(run_id)}",
                }
            time.sleep(0.1)

    return {
        "running": True,
        "run_id": run_id,
        "driver": driver,
        "command": command,
        "report": f"robot driver {'restarted with updated profile' if restarted else 'running'}: {driver.get('name') or run_id} (pid {proc.pid})",
    }


@node(
    name="RobotDiscovery", component="capabilities",
    category=_CATEGORY,
    hidden=True,
    description="Advanced compatibility node for connection discovery and driver launch; new workflows should use Robot.",
    inputs={
        "trigger": AnyPort,
        "driver": Dict,
        "usb": Dict,
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
        "control_topic": Text(default="/robot_control"),
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
    driver = _driver_from_ctx(ctx)
    match = driver.get("match") if isinstance(driver.get("match"), dict) else {}
    supplied_usb = ctx.get("usb") if isinstance(ctx.get("usb"), dict) else {}
    usb = supplied_usb or robot_usb_discovery({
        "port_filter": ctx.get("port_filter", ""),
        "match_vendor_id": match.get("vendor_id", ""),
        "match_product_id": match.get("product_id", ""),
        "probe_open": bool(ctx.get("probe_open", False)),
    })
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
    control_topic = str(driver.get("control_topic") or ctx.get("control_topic") or "/robot_control")
    units = str(driver.get("units") or ctx.get("units") or "degrees")
    robot = {
        "host": host,
        "port": port,
        "state_topic": state_topic,
        "command_topic": command_topic,
        "config_topic": config_topic,
        "control_topic": control_topic,
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
        driver_report = str(driver_result.get("report") or "")
        if "last exit code" in driver_report or "FAILED" in driver_report or "exited" in driver_report:
            lines.append("=> NEXT: fix the driver startup error above, then start it again")
        else:
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


def _svg_text(value: Any, limit: int = 64) -> str:
    text = str(value or "").strip()
    if len(text) > limit:
        text = text[: max(0, limit - 1)] + "…"
    return html.escape(text)


def _svg_data(svg: str) -> str:
    encoded = base64.b64encode(svg.encode("utf-8")).decode("ascii")
    return f"data:image/svg+xml;base64,{encoded}"


@node(
    name="RobotConnectionDashboard", component="capabilities",
    category=_CATEGORY,
    description="Render one clear USB, driver, ROS interface, and live-pose readiness screen for a robot demo.",
    live=True,
    inputs={
        "robot": Dict,
        "connected": Bool(default=False),
        "interface_ready": Bool(default=False),
        "pose": Dict,
        "status_report": Text(default=""),
    },
    outputs={"dashboard": Image, "ready": Bool, "live": Bool, "summary": Dict, "report": Text},
)
def robot_connection_dashboard(ctx: dict) -> dict:
    live_pose = bool(ctx.get("__live_pose__"))
    robot = dict(ctx.get("robot") or {})
    usb = dict(robot.get("usb") or {})
    driver = dict(robot.get("driver") or {})
    pose = dict(ctx.get("pose") or {})
    connected = bool(ctx.get("connected", False))
    interface_ready = bool(ctx.get("interface_ready", False))
    usb_ready = bool(usb.get("ready"))
    driver_running = bool(driver.get("running"))
    pose_ready = bool(pose)
    ready = bool(usb_ready and driver_running and connected and interface_ready and pose_ready)

    recommended = dict(usb.get("recommended") or {})
    serial_port = str(recommended.get("path") or "not detected")
    driver_name = str(driver.get("name") or driver.get("id") or "not selected")
    profile_id = str(driver.get("profile_id") or driver.get("id") or "not selected")
    transport = str((robot.get("interface") or {}).get("kind") or driver.get("transport") or "none")
    configured_joints = driver.get("joints") if isinstance(driver.get("joints"), list) else []
    joint_configs = {
        str(joint.get("id")): dict(joint)
        for joint in configured_joints
        if isinstance(joint, dict) and str(joint.get("id") or "").strip()
    }
    joints = list(joint_configs)
    joints.extend(name for name in sorted(pose) if name not in joint_configs)
    effective_profile = driver.get("profile") if isinstance(driver.get("profile"), dict) else {}
    calibration = effective_profile.get("calibration") if isinstance(effective_profile.get("calibration"), dict) else {}
    calibration_path = str(driver.get("calibration_path") or "").strip()
    calibrated = bool(calibration_path or calibration)
    hardware_id = str(
        driver.get("hardware_id")
        or calibration.get("hardware_id")
        or recommended.get("serial_number")
        or recommended.get("serial")
        or serial_port
    )

    joint_details = []
    for name in joints:
        config = joint_configs.get(name, {})
        current = pose.get(name)
        safe_min = config.get("safe_min_deg", config.get("min_deg"))
        safe_max = config.get("safe_max_deg", config.get("max_deg"))
        joint_details.append({
            "joint": name,
            "current_deg": current,
            "home_deg": 0.0 if config else None,
            "home_ticks": config.get("home_ticks"),
            "home_offset_deg": config.get("home_offset_deg"),
            "safe_min_deg": safe_min,
            "safe_max_deg": safe_max,
            "calibrated": calibrated,
        })
    summary = {
        "ready": ready,
        "usb_ready": usb_ready,
        "driver_running": driver_running,
        "connected": connected,
        "interface_ready": interface_ready,
        "live_pose": pose_ready,
        "serial_port": serial_port,
        "driver": driver_name,
        "profile_id": profile_id,
        "transport": transport,
        "joint_count": len(joints),
        "pose": pose,
        "calibrated": calibrated,
        "calibration_status": "saved calibration" if calibrated else "profile defaults",
        "calibration_path": calibration_path,
        "hardware_id": hardware_id,
        "joints": joint_details,
    }

    stages = [
        ("USB", usb_ready, serial_port),
        ("DRIVER", driver_running, driver_name),
        ("ROS 2", connected and interface_ready, transport),
        ("LIVE STATE", pose_ready, f"{len(joints)} joints" if joints else "waiting"),
    ]
    cards = []
    for index, (label, ok, detail) in enumerate(stages):
        x = 36 + index * 260
        color = "#22c55e" if ok else "#f59e0b"
        verdict = "READY" if ok else "WAITING"
        cards.append(
            f'<rect x="{x}" y="150" width="236" height="132" rx="16" fill="#172033" stroke="{color}" stroke-width="2"/>'
            f'<circle cx="{x + 28}" cy="181" r="8" fill="{color}"/>'
            f'<text x="{x + 48}" y="187" fill="#f8fafc" font-family="Arial,sans-serif" font-size="15" font-weight="700">{label}</text>'
            f'<text x="{x + 20}" y="226" fill="{color}" font-family="Arial,sans-serif" font-size="18" font-weight="800">{verdict}</text>'
            f'<text x="{x + 20}" y="258" fill="#93a4b8" font-family="monospace" font-size="12">{_svg_text(detail, 27)}</text>'
        )

    pose_rows = []
    for index, detail in enumerate(joint_details[:6]):
        value = detail["current_deg"]
        value_text = f"{value:.2f}°" if isinstance(value, (int, float)) else "—"
        home_ticks = detail["home_ticks"]
        home_text = f"0.00° · {home_ticks}t" if isinstance(home_ticks, (int, float)) else "—"
        safe_min = detail["safe_min_deg"]
        safe_max = detail["safe_max_deg"]
        range_text = (
            f"{float(safe_min):.2f}° .. {float(safe_max):.2f}°"
            if isinstance(safe_min, (int, float)) and isinstance(safe_max, (int, float))
            else "—"
        )
        y = 388 + index * 38
        pose_rows.append(
            f'<text x="64" y="{y}" fill="#cbd5e1" font-family="monospace" font-size="14">{_svg_text(detail["joint"], 20)}</text>'
            f'<text x="310" y="{y}" text-anchor="end" fill="#f8fafc" font-family="monospace" font-size="14" font-weight="700">{_svg_text(value_text, 15)}</text>'
            f'<text x="510" y="{y}" text-anchor="end" fill="#f8fafc" font-family="monospace" font-size="14">{_svg_text(home_text, 19)}</text>'
            f'<text x="730" y="{y}" text-anchor="end" fill="#f8fafc" font-family="monospace" font-size="14">{_svg_text(range_text, 25)}</text>'
        )

    accent = "#22c55e" if ready else "#f59e0b"
    verdict = "READY TO ARM" if ready else "SAFE / NOT ARMED"
    next_step_lines = (
        ("Live state verified.", "Clear workspace before arming.")
        if ready
        else ("Complete readiness stages.", "Motion remains blocked.")
    )
    calibration_color = "#22c55e" if calibrated else "#f59e0b"
    calibration_label = "SAVED CALIBRATION" if calibrated else "PROFILE DEFAULTS"
    calibration_detail_1 = "Hardware-specific limits" if calibrated else "No saved calibration file"
    calibration_detail_2 = "and home are active" if calibrated else "Run calibration, then Save"
    svg = f'''<svg xmlns="http://www.w3.org/2000/svg" width="1120" height="680" viewBox="0 0 1120 680">
<rect width="1120" height="680" rx="28" fill="#0b1020"/>
<rect x="24" y="24" width="1072" height="92" rx="18" fill="#172033" stroke="#14b8a6" stroke-width="2"/>
<text x="56" y="64" fill="#f8fafc" font-family="Arial,sans-serif" font-size="26" font-weight="800">ROBOT CONNECTION</text>
<text x="56" y="91" fill="#93a4b8" font-family="Arial,sans-serif" font-size="15">Plug in → robot → driver → live state</text>
<rect x="856" y="44" width="210" height="52" rx="26" fill="{accent}"/>
<text x="961" y="77" text-anchor="middle" fill="#07111f" font-family="Arial,sans-serif" font-size="17" font-weight="900">{verdict}</text>
{''.join(cards)}
<rect x="36" y="316" width="720" height="328" rx="16" fill="#172033"/>
<text x="64" y="344" fill="#93a4b8" font-family="Arial,sans-serif" font-size="13" font-weight="700">JOINT</text>
<text x="310" y="344" text-anchor="end" fill="#93a4b8" font-family="Arial,sans-serif" font-size="13" font-weight="700">LIVE</text>
<text x="510" y="344" text-anchor="end" fill="#93a4b8" font-family="Arial,sans-serif" font-size="13" font-weight="700">HOME</text>
<text x="730" y="344" text-anchor="end" fill="#93a4b8" font-family="Arial,sans-serif" font-size="13" font-weight="700">SAFE RANGE</text>
<line x1="64" y1="356" x2="730" y2="356" stroke="#334155"/>
{''.join(pose_rows) if pose_rows else '<text x="64" y="394" fill="#f59e0b" font-family="Arial,sans-serif" font-size="16">Waiting for robot joint configuration and /joint_states…</text>'}
<text x="64" y="624" fill="#64748b" font-family="Arial,sans-serif" font-size="12">Home is 0° in robot coordinates; “t” is the saved raw servo tick.</text>
<rect x="780" y="316" width="304" height="328" rx="16" fill="#172033"/>
<text x="808" y="346" fill="#93a4b8" font-family="Arial,sans-serif" font-size="13" font-weight="700">CALIBRATION</text>
<rect x="808" y="362" width="248" height="38" rx="19" fill="{calibration_color}"/>
<text x="932" y="387" text-anchor="middle" fill="#07111f" font-family="Arial,sans-serif" font-size="12" font-weight="900">{calibration_label}</text>
<text x="808" y="426" fill="#cbd5e1" font-family="Arial,sans-serif" font-size="13">{calibration_detail_1}</text>
<text x="808" y="447" fill="#cbd5e1" font-family="Arial,sans-serif" font-size="13">{calibration_detail_2}</text>
<text x="808" y="476" fill="#93a4b8" font-family="monospace" font-size="12">profile: {_svg_text(profile_id, 25)}</text>
<text x="808" y="497" fill="#93a4b8" font-family="monospace" font-size="12">device: {_svg_text(hardware_id, 26)}</text>
<text x="808" y="532" fill="#93a4b8" font-family="Arial,sans-serif" font-size="13" font-weight="700">NEXT SAFE ACTION</text>
<text x="808" y="557" fill="#f8fafc" font-family="Arial,sans-serif" font-size="13" font-weight="700">{next_step_lines[0]}</text>
<text x="808" y="577" fill="#f8fafc" font-family="Arial,sans-serif" font-size="13" font-weight="700">{next_step_lines[1]}</text>
<text x="808" y="598" fill="#93a4b8" font-family="monospace" font-size="12">state: {_svg_text(robot.get('state_topic'), 27)}</text>
<text x="808" y="616" fill="#93a4b8" font-family="monospace" font-size="12">command: {_svg_text(robot.get('command_topic'), 25)}</text>
<text x="808" y="636" fill="{accent}" font-family="Arial,sans-serif" font-size="12" font-weight="700">Stop all uses safe shutdown.</text>
</svg>'''
    report = (
        f"robot connection dashboard {'READY' if ready else 'WAITING'}: "
        f"USB={'ok' if usb_ready else 'wait'}, driver={'ok' if driver_running else 'wait'}, "
        f"interface={'ok' if connected and interface_ready else 'wait'}, live_pose={'ok' if pose_ready else 'wait'}"
    )
    return {"dashboard": _svg_data(svg), "ready": ready, "live": live_pose, "summary": summary, "report": report}
