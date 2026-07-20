from __future__ import annotations

import json
from collections import defaultdict
from pathlib import Path


_TEMPLATE_DIR = Path(__file__).resolve().parents[1] / "templates"
_EXPECTED_ROBOTS = {
    "editable-so-arm101-profile.json": {"robot": 0},
    "robot-guided-calibration.json": {"robot": 0},
    "so-arm101-motion-test.json": {"robot": 0},
}


def test_physical_device_selectors_use_unique_indexes() -> None:
    for path in sorted(_TEMPLATE_DIR.glob("*.json")):
        workflow = json.loads(path.read_text(encoding="utf-8"))
        nodes = workflow.get("node_meta") or {}
        actual_robots = {
            node_id: (node.get("params") or {}).get("selection", 0)
            for node_id, node in nodes.items()
            if node.get("type") == "Robot"
        }
        assert actual_robots == _EXPECTED_ROBOTS[path.name]
        incoming: dict[str, set[str]] = defaultdict(set)
        for edge in workflow.get("edges", []):
            incoming[str(edge.get("to") or "")].add(str(edge.get("to_port") or ""))

        used: dict[str, dict[int, str]] = defaultdict(dict)
        for node_id, node in nodes.items():
            device_type = node.get("type")
            if device_type not in {"Robot", "Camera"}:
                continue

            chained_robot_ports = incoming[node_id] & {"hardware", "usb", "driver"}
            assert device_type != "Robot" or not chained_robot_ports, (
                f"{path.name}: {node_id} rebuilds a low-level Robot setup chain through "
                f"{sorted(chained_robot_ports)} instead of using one Robot facade"
            )
            supplied_ports = set() if device_type == "Robot" else {"camera"}
            if incoming[node_id] & supplied_ports:
                continue

            selection = (node.get("params") or {}).get("selection", 0)
            assert isinstance(selection, int) and selection >= 0, (
                f"{path.name}: {node_id} has invalid {device_type} selection {selection!r}"
            )
            assert selection not in used[device_type], (
                f"{path.name}: {node_id} and {used[device_type][selection]} independently "
                f"select {device_type} index {selection}"
            )
            used[device_type][selection] = node_id
