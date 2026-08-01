import json
from pathlib import Path

import blacknode  # noqa: F401
from blacknode.node import _NODE_REGISTRY
from blacknode.workflow import validate_workflow


def _inspection():
    return {
        "ok": True,
        "live": True,
        "checked_at": "2026-07-31T09:00:00+00:00",
        "password": "must-not-leak",
        "environment": {
            "os": {"name": "Ubuntu", "version": "22.04"},
            "nested": {"runtime_token": "must-not-leak"},
        },
        "ros2_graph": {
            "available": True,
            "read_only": True,
            "daemon_used": False,
            "distribution": "humble",
            "capabilities": [{"capability": "mobile_base"}],
            "unclassified": [{"name": "/vendor/status"}],
            "inventory": {
                "topics": ["/odom", "/cmd_vel"],
                "nodes": ["/controller"],
                "services": [],
            },
            "report": "Generic ROS 2 capability discovery",
        },
    }


def test_compute_device_nodes_are_registered():
    assert _NODE_REGISTRY["ComputeDevice"]._bn_package == "blacknode-robot"
    assert _NODE_REGISTRY["DeviceInspect"]._bn_package == "blacknode-robot"
    assert _NODE_REGISTRY["ComputeDevice"]._bn_primary_outputs == [
        "device",
        "inspection",
    ]


def test_compute_device_emits_only_a_stable_public_handle():
    result = _NODE_REGISTRY["ComputeDevice"]({
        "device_id": "jetson-01",
        "device_name": "Workshop Jetson",
        "inspection": _inspection(),
    })

    assert result["configured"] is True
    assert result["inspection_available"] is True
    assert result["device"] == {
        "kind": "blacknode.compute-device-target",
        "schema_version": 1,
        "device_id": "jetson-01",
        "device_name": "Workshop Jetson",
        "configured": True,
        "inspection_available": True,
        "live": True,
        "read_only": True,
    }
    serialized = json.dumps(result)
    assert "must-not-leak" not in serialized
    assert "password" not in serialized
    assert "runtime_token" not in serialized


def test_device_inspect_exposes_read_only_inventory_and_candidates():
    selected = _NODE_REGISTRY["ComputeDevice"]({
        "device_id": "jetson-01",
        "device_name": "Workshop Jetson",
        "inspection": _inspection(),
    })
    result = _NODE_REGISTRY["DeviceInspect"]({
        "device": selected["device"],
        "inspection": selected["inspection"],
    })

    assert result["available"] is True
    assert result["read_only"] is True
    assert result["environment"]["os"]["name"] == "Ubuntu"
    assert result["capabilities"] == [{"capability": "mobile_base"}]
    assert result["inventory"]["topics"] == ["/odom", "/cmd_vel"]
    assert result["unclassified"] == [{"name": "/vendor/status"}]


def test_device_inspect_degrades_cleanly_without_live_runtime_state():
    result = _NODE_REGISTRY["DeviceInspect"]({
        "device": {
            "kind": "blacknode.compute-device-target",
            "schema_version": 1,
            "device_id": "jetson-01",
            "configured": True,
        },
        "inspection": {},
    })

    assert result["available"] is False
    assert result["environment"] == {}
    assert result["capabilities"] == []
    assert "did not return current device state" in result["report"]


def test_compute_device_inspection_template_validates():
    path = (
        Path(__file__).resolve().parents[1]
        / "templates"
        / "compute-device-inspection.json"
    )
    report = validate_workflow(json.loads(path.read_text(encoding="utf-8")))
    assert report.ok, [issue.message for issue in report.errors]
