import json
from pathlib import Path

import blacknode  # noqa: F401
from blacknode.node import _NODE_REGISTRY
from blacknode.workflow import validate_workflow


def _discover(topics):
    return _NODE_REGISTRY["RobotROSCapabilityDiscover"]({
        "topics": topics,
        "nodes": ["/controller", "/lidar"],
        "services": ["/controller/get_parameters"],
    })


def test_generic_ros_capability_discovery_is_registered():
    discover = _NODE_REGISTRY["RobotROSCapabilityDiscover"]

    assert discover._bn_category == "Robot"
    assert discover._bn_package == "blacknode-robot"


def test_generic_ros_capability_discovery_template_validates():
    path = (
        Path(__file__).resolve().parents[1]
        / "templates"
        / "generic-ros-capability-discovery.json"
    )
    report = validate_workflow(json.loads(path.read_text(encoding="utf-8")))

    assert report.ok, [issue.message for issue in report.errors]


def test_generic_ros_capability_discovery_uses_standard_message_types():
    result = _discover([
        "/front/scan [sensor_msgs/msg/LaserScan]",
        "/sensors/imu_raw [sensor_msgs/msg/Imu]",
        "/stereo/depth/image_raw [sensor_msgs/msg/Image]",
        "/camera/color/image_raw [sensor_msgs/msg/Image]",
        "/base/odom_raw [nav_msgs/msg/Odometry]",
        "/controller/cmd_vel [geometry_msgs/msg/Twist]",
        "/power [sensor_msgs/msg/BatteryState]",
    ])

    assert result["found"] is True
    candidates = {
        candidate["capability"]: candidate
        for candidate in result["capabilities"]
    }
    assert set(candidates) == {
        "battery",
        "camera",
        "depth_camera",
        "imu",
        "lidar",
        "mobile_base",
    }
    assert candidates["mobile_base"]["confidence"] == "high"
    assert candidates["mobile_base"]["state_topics"] == ["/base/odom_raw"]
    assert candidates["mobile_base"]["command_topics"] == ["/controller/cmd_vel"]
    assert candidates["mobile_base"]["requires_confirmation"] is True
    assert all(candidate["requires_confirmation"] for candidate in candidates.values())
    assert "read-only" in result["report"].lower()


def test_discovery_does_not_treat_command_only_servo_topic_as_feedback():
    result = _discover([
        (
            "/controller/pwm_servo/set_state "
            "[example_interfaces/msg/SetPWMServoState]"
        ),
        "/parameter_events [rcl_interfaces/msg/ParameterEvent]",
    ])

    assert result["found"] is False
    assert result["capabilities"] == []
    assert result["inventory"]["unclassified_topic_count"] == 2
    assert {
        entry["name"]
        for entry in result["unclassified"]
    } == {
        "/controller/pwm_servo/set_state",
        "/parameter_events",
    }


def test_discovery_accepts_structured_topic_inventory():
    result = _discover([
        {
            "name": "/robot/joint_states",
            "message_type": "sensor_msgs/msg/JointState",
        },
        {
            "topic": "/points",
            "types": ["sensor_msgs/msg/PointCloud2"],
        },
    ])

    candidates = {
        candidate["capability"]: candidate
        for candidate in result["capabilities"]
    }
    assert candidates["joint_state"]["confidence"] == "high"
    assert candidates["lidar"]["confidence"] == "medium"
    assert candidates["lidar"]["requires_confirmation"] is True
