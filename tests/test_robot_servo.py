from __future__ import annotations

import math

from blacknode_robot.devices.nodes import robot_servo


def test_robot_servo_projects_live_feedback_limits_and_diagnostics():
    result = robot_servo({
        "robot": {
            "calibration": {
                "profile_id": "so_arm101",
                "hardware_id": "SERIAL-42",
            },
            "profile": {
                "joints": [{
                    "id": "shoulder_lift",
                    "servo_id": 2,
                    "safe_min_deg": -75.0,
                    "safe_max_deg": 82.0,
                }],
            },
        },
        "state": {
            "type": "robot_telemetry",
            "source": "hardware",
            "stale": False,
            "payload": {
                "connected": True,
                "armed": False,
                "torque_enabled": False,
                "position_unit": "degree",
                "velocity_unit": "degree/s",
                "calibrated": True,
                "voltage_v": 12.2,
                "temperatures_c": {"servo_2": 41.5},
                "faults": [],
                "joints": [{
                    "name": "servo_2",
                    "semantic_name": "shoulder_lift",
                    "servo_id": 2,
                    "position": 12.5,
                    "velocity": -3.0,
                    "raw_position": 2190,
                    "lower_limit": -70.0,
                    "upper_limit": 80.0,
                }],
            },
        },
        "servo_id": 2,
        "target_position": 100.0,
        "follow_feedback": False,
        "units": "degrees",
    })

    assert result["available"] is True
    assert result["joint"] == "servo_2"
    assert result["position"] == 12.5
    assert result["velocity"] == -3.0
    assert result["raw_position"] == 2190
    assert result["limits"] == {
        "lower": -70.0,
        "upper": 80.0,
        "units": "degrees",
    }
    assert result["calibrated"] is True
    assert result["temperature_c"] == 41.5
    assert result["voltage_v"] == 12.2
    assert result["target_position"] == 80.0
    assert math.isclose(result["command"]["position_rad"], math.radians(80.0))
    assert result["command"]["requires_motion_authorization"] is True
    assert "preview only" in result["report"]


def test_robot_servo_can_follow_feedback_and_report_unavailable_ids():
    state = {
        "payload": {
            "connected": True,
            "position_unit": "degree",
            "joints": [{
                "name": "servo_1",
                "servo_id": 1,
                "position": 9.25,
                "velocity": 0.0,
            }],
        },
    }
    following = robot_servo({
        "robot": {},
        "state": state,
        "servo_id": 1,
        "follow_feedback": True,
        "units": "degrees",
    })
    missing = robot_servo({
        "robot": {},
        "state": state,
        "servo_id": 6,
        "follow_feedback": True,
        "units": "degrees",
    })

    assert following["target_position"] == 9.25
    assert following["available"] is True
    assert missing["available"] is False
    assert missing["joint"] == "servo_6"
    assert "not present" in missing["report"]
