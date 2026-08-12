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
        report = f"{device_name or device_id}: paired Runtime is live"
        report += f"; ROS state checked {checked_at}." if checked_at else "."
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


def _public_list(value: TypingAny) -> list[dict]:
    if not isinstance(value, list):
        return []
    return [dict(item) for item in _public_value(value) if isinstance(item, dict)]


@node(
    name="PhysicalRobot",
    component="capabilities",
    category="Robot",
    description="Select one physical robot registered under a compute device.",
    inputs={
        "device": Dict,
        "inspection": Dict,
        "robot_id": Text(default=""),
        "robot_name": Text(default=""),
    },
    outputs={
        "configured": Bool,
        "robot": Dict,
        "inspection": Dict,
        "report": Text,
    },
    primary_inputs=["device", "inspection"],
    primary_outputs=["robot", "inspection"],
)
def physical_robot(ctx: dict) -> dict:
    device = _public_value(
        ctx.get("device") if isinstance(ctx.get("device"), dict) else {}
    )
    inspection = _public_value(
        ctx.get("inspection") if isinstance(ctx.get("inspection"), dict) else {}
    )
    robot_id = str(ctx.get("robot_id") or "").strip()
    robot_name = str(ctx.get("robot_name") or "").strip()
    robots = _public_list(inspection.get("robots"))
    selected = next(
        (item for item in robots if str(item.get("id") or "") == robot_id),
        None,
    )
    if selected is None and not robot_id and len(robots) == 1:
        selected = robots[0]
    selected_id = str((selected or {}).get("id") or robot_id).strip()
    selected_name = str((selected or {}).get("name") or robot_name).strip()
    live = bool(inspection.get("ok") and inspection.get("live"))
    robot = {
        "kind": "blacknode.physical-robot-target",
        "schema_version": 1,
        "device_id": str(device.get("device_id") or ""),
        "device_name": str(device.get("device_name") or ""),
        "robot_id": selected_id,
        "robot_name": selected_name,
        "configured": bool(selected_id),
        "live": live,
        "read_only": True,
    }
    if selected_id:
        report = f"Selected physical robot {selected_name or selected_id}."
    elif len(robots) > 1:
        report = "Choose one physical robot in Properties."
    elif not device.get("device_id"):
        report = "Connect a Compute Device node first."
    else:
        report = "No physical robot is registered under this compute device."
    return {
        "configured": bool(selected_id),
        "robot": robot,
        "inspection": inspection,
        "report": report,
    }


@node(
    name="RobotDeployment",
    component="capabilities",
    category="Robot",
    description="Select one deployment that belongs to a physical robot.",
    inputs={
        "robot": Dict,
        "inspection": Dict,
        "deployment_id": Text(default=""),
        "deployment_name": Text(default=""),
    },
    outputs={
        "selected": Bool,
        "deployment": Dict,
        "inspection": Dict,
        "report": Text,
    },
    primary_inputs=["robot", "inspection"],
    primary_outputs=["deployment", "inspection"],
)
def robot_deployment(ctx: dict) -> dict:
    robot = _public_value(
        ctx.get("robot") if isinstance(ctx.get("robot"), dict) else {}
    )
    inspection = _public_value(
        ctx.get("inspection") if isinstance(ctx.get("inspection"), dict) else {}
    )
    deployment_id = str(ctx.get("deployment_id") or "").strip()
    deployment_name = str(ctx.get("deployment_name") or "").strip()
    robot_id = str(robot.get("robot_id") or "").strip()
    deployments = [
        item for item in _public_list(inspection.get("deployments"))
        if not robot_id or str(item.get("target_device_id") or "") in {"", robot_id}
    ]
    selected = next(
        (item for item in deployments if str(item.get("id") or "") == deployment_id),
        None,
    )
    if selected is None and not deployment_id and len(deployments) == 1:
        selected = deployments[0]
    deployment = dict(selected) if selected else {
        "kind": "blacknode.robot-deployment",
        "schema_version": 1,
        "id": deployment_id,
        "name": deployment_name,
        "target_device_id": robot_id,
        "state": "unavailable",
        "available": False,
    }
    if selected:
        report = f"Selected deployment {selected.get('name') or selected.get('id')}."
    elif len(deployments) > 1:
        report = "Choose one robot deployment in Properties."
    elif not robot_id:
        report = "Connect a Physical Robot node first."
    else:
        report = "No deployment is available for this physical robot."
    return {
        "selected": bool(selected),
        "deployment": deployment,
        "inspection": inspection,
        "report": report,
    }


@node(
    name="RobotStream",
    component="capabilities",
    category="Robot",
    description="Select one deployed robot capability stream for downstream nodes.",
    inputs={
        "robot": Dict,
        "deployment": Dict,
        "inspection": Dict,
        "capability": Text(default=""),
        "topic": Text(default=""),
        "message_type": Text(default=""),
    },
    outputs={
        "available": Bool,
        "stream": Dict,
        "topic": Text,
        "message_type": Text,
        "report": Text,
    },
    primary_inputs=["robot", "deployment", "inspection"],
    primary_outputs=["stream", "topic", "message_type"],
)
def robot_stream(ctx: dict) -> dict:
    robot = _public_value(
        ctx.get("robot") if isinstance(ctx.get("robot"), dict) else {}
    )
    deployment = _public_value(
        ctx.get("deployment") if isinstance(ctx.get("deployment"), dict) else {}
    )
    inspection = _public_value(
        ctx.get("inspection") if isinstance(ctx.get("inspection"), dict) else {}
    )
    capability = str(ctx.get("capability") or "").strip()
    topic = str(ctx.get("topic") or "").strip()
    message_type = str(ctx.get("message_type") or "").strip()
    robot_id = str(robot.get("robot_id") or "").strip()
    deployment_id = str(deployment.get("id") or "").strip()
    streams = [
        item for item in _public_list(inspection.get("streams"))
        if (not robot_id or str(item.get("robot_id") or "") in {"", robot_id})
        and (
            not deployment_id
            or str(item.get("deployment_id") or "") in {"", deployment_id}
        )
        and (not capability or str(item.get("capability") or "") == capability)
    ]
    selected = next(
        (
            item for item in streams
            if str(item.get("topic") or "") == topic
            and (
                not message_type
                or str(item.get("message_type") or "") == message_type
            )
        ),
        None,
    )
    if selected is None and not topic and len(streams) == 1:
        selected = streams[0]
    stream = dict(selected) if selected else {
        "kind": "blacknode.deployed-stream",
        "schema_version": 1,
        "source": "saved_selection",
        "capability": capability,
        "device_id": str(robot.get("device_id") or ""),
        "robot_id": robot_id,
        "deployment_id": deployment_id,
        "state": "unavailable",
        "available": False,
        "topic": topic,
        "message_type": message_type,
    }
    if selected:
        report = f"Selected {selected.get('capability') or 'robot'} stream {selected.get('topic')}."
    elif len(streams) > 1:
        report = "Choose one capability stream in Properties."
    elif not robot_id:
        report = "Connect a Physical Robot node first."
    else:
        report = "No matching deployed stream is available."
    return {
        "available": bool(stream.get("available")),
        "stream": stream,
        "topic": str(stream.get("topic") or topic),
        "message_type": str(stream.get("message_type") or message_type),
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
