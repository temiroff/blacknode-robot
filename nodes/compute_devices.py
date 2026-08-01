"""Portable compute-device targets and credential-free live state."""

from __future__ import annotations

from typing import Any as TypingAny

from blacknode.node import Bool, Dict, List, Text, node


_PRIVATE_KEYS = {
    "password",
    "token",
    "runtime_token",
    "pairing_token",
    "secret",
    "authorization",
}


def _public_value(value: TypingAny) -> TypingAny:
    """Copy device state while excluding credential-shaped fields."""
    if isinstance(value, dict):
        return {
            str(key): _public_value(item)
            for key, item in value.items()
            if str(key).strip().lower() not in _PRIVATE_KEYS
            and not str(key).strip().lower().endswith("_password")
            and not str(key).strip().lower().endswith("_token")
            and not str(key).strip().lower().endswith("_secret")
        }
    if isinstance(value, list):
        return [_public_value(item) for item in value]
    return value


@node(
    name="ComputeDevice",
    component="capabilities",
    category="Robot",
    description=(
        "Select a registered compute device by stable identity and expose "
        "current credential-free state supplied by its paired Runtime."
    ),
    inputs={
        "device_id": Text(default=""),
        "device_name": Text(default=""),
        "inspection": Dict,
    },
    outputs={
        "configured": Bool,
        "inspection_available": Bool,
        "device": Dict,
        "inspection": Dict,
        "report": Text,
    },
    primary_inputs=[],
    primary_outputs=["device", "inspection"],
)
def compute_device(ctx: dict) -> dict:
    device_id = str(ctx.get("device_id") or "").strip()
    device_name = str(ctx.get("device_name") or "").strip()
    inspection = _public_value(
        ctx.get("inspection") if isinstance(ctx.get("inspection"), dict) else {}
    )
    configured = bool(device_id)
    inspection_available = bool(inspection.get("ok"))
    live = bool(inspection_available and inspection.get("live"))
    device = {
        "kind": "blacknode.compute-device-target",
        "schema_version": 1,
        "device_id": device_id,
        "device_name": device_name,
        "configured": configured,
        "inspection_available": inspection_available,
        "live": live,
        "read_only": True,
    }
    if not configured:
        report = "Choose a compute device in the node."
    elif live:
        checked_at = str(inspection.get("checked_at") or "").strip()
        report = (
            f"{device_name or device_id}: paired Runtime is live"
            + (f"; ROS state checked {checked_at}." if checked_at else ".")
        )
    else:
        report = (
            f"{device_name or device_id}: selected, but its paired Runtime did "
            "not return current ROS state. Start or install the Runtime from Devices."
        )
    return {
        "configured": configured,
        "inspection_available": inspection_available,
        "device": device,
        "inspection": inspection,
        "report": report,
    }


@node(
    name="DeviceInspect",
    component="capabilities",
    category="Robot",
    description=(
        "Read sanitized live device state. This node never runs "
        "commands, starts services, publishes ROS messages, or arms motion."
    ),
    inputs={
        "device": Dict,
        "inspection": Dict,
    },
    outputs={
        "available": Bool,
        "read_only": Bool,
        "environment": Dict,
        "ros2_graph": Dict,
        "capabilities": List,
        "unclassified": List,
        "inventory": Dict,
        "report": Text,
    },
)
def device_inspect(ctx: dict) -> dict:
    device = _public_value(
        ctx.get("device") if isinstance(ctx.get("device"), dict) else {}
    )
    inspection = _public_value(
        ctx.get("inspection") if isinstance(ctx.get("inspection"), dict) else {}
    )
    environment = (
        inspection.get("environment")
        if isinstance(inspection.get("environment"), dict)
        else {}
    )
    ros2_graph = (
        inspection.get("ros2_graph")
        if isinstance(inspection.get("ros2_graph"), dict)
        else {}
    )
    capabilities = (
        ros2_graph.get("capabilities")
        if isinstance(ros2_graph.get("capabilities"), list)
        else []
    )
    unclassified = (
        ros2_graph.get("unclassified")
        if isinstance(ros2_graph.get("unclassified"), list)
        else []
    )
    inventory = (
        ros2_graph.get("inventory")
        if isinstance(ros2_graph.get("inventory"), dict)
        else {
            "topics": list(ros2_graph.get("topics") or []),
            "nodes": list(ros2_graph.get("nodes") or []),
            "services": list(ros2_graph.get("services") or []),
        }
    )
    configured = bool(device.get("configured") or device.get("device_id"))
    available = bool(
        configured and inspection.get("ok") and inspection.get("live")
    )
    read_only = bool(
        inspection.get("read_only", True)
        and ros2_graph.get("read_only", True)
        and not ros2_graph.get("daemon_used", False)
    )
    if not configured:
        report = "No compute device is connected."
    elif not inspection.get("ok") or not inspection.get("live"):
        report = "The paired Runtime did not return current device state."
    else:
        graph_report = str(ros2_graph.get("report") or "").strip()
        report = graph_report or (
            "Live read-only device state loaded: "
            f"{len(inventory.get('topics') or [])} ROS 2 topics, "
            f"{len(inventory.get('nodes') or [])} nodes, and "
            f"{len(capabilities)} capability candidates."
        )
    return {
        "available": available,
        "read_only": read_only,
        "environment": environment,
        "ros2_graph": ros2_graph,
        "capabilities": capabilities,
        "unclassified": unclassified,
        "inventory": inventory,
        "report": report,
    }
