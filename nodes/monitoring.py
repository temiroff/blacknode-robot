"""Read-only robot monitoring targets for the Blacknode canvas."""

from blacknode.node import Dict, Text, node


@node(
    name="RobotMonitor",
    component="capabilities",
    category="Robot",
    inputs={
        "robot_id": Text(default=""),
        "robot_name": Text(default=""),
    },
    outputs={"robot": Dict},
    primary_inputs=[],
    primary_outputs=["robot"],
    description="Show a registered robot's live state, telemetry, and streams on the canvas.",
)
def robot_monitor(robot_id: str = "", robot_name: str = "") -> dict:
    """Describe the stable, read-only target used by the live canvas monitor."""
    target_id = str(robot_id or "").strip()
    target_name = str(robot_name or "").strip()
    return {
        "robot": {
            "kind": "blacknode.robot-monitor-target",
            "schema_version": 1,
            "robot_id": target_id,
            "robot_name": target_name,
            "configured": bool(target_id),
        },
    }
