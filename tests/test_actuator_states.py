import time

import blacknode  # noqa: F401
import pytest
from blacknode.pkg.blacknode_robot import actuator_setup as setup
from blacknode.pkg.blacknode_robot import actuator_states as states


@pytest.fixture
def rig(monkeypatch, tmp_path):
    states.stop_runtime_services()
    states._drafts.clear()
    states._stopped.clear()
    setup._mock_rows.clear()
    setup._mock_scan({"port": "fake"})
    key = ("physical-123", "blacknode-robot", "capabilities", 1)
    provider = setup.robot_actuator_setup_mock_provider._bn_robot_actuator_setup_provider
    config = {"port": "fake", "baudrate": 1000000, "hardware_id": key[0]}
    monkeypatch.setattr(states, "_identity", lambda _: (key, provider, config))
    monkeypatch.setattr(states.profiles, "_profile_root", lambda: tmp_path)
    ctx = {"serial_port": "fake", "servo_id": 1}
    yield ctx, key, setup._mock_rows["fake"][0]
    states.stop_runtime_services()
    states._drafts.clear()


def call(rig, action, **payload):
    return states.control(rig[0], action, payload)


def calibrated(rig):
    for point, ticks in (("min", 1000), ("home", 2000), ("max", 3000)):
        rig[2]["raw_position"] = ticks
        assert call(rig, "capture-state", point=point)["ok"]
    rig[2]["raw_position"] = 2000
    assert call(rig, "save-states")["saved"]


def test_capture_and_save_reload_hardware_bound_calibration_and_named_pose(rig):
    calibrated(rig)
    result = call(rig, "capture-state", point="pose", name="Ready")
    assert result["states"]["poses"] == {"Ready": 2000} and not result["saved"]
    result = call(rig, "save-states")
    assert result["test_limits"] == {"min": 1000, "home": 2000, "max": 3000}
    states._drafts.clear()
    loaded = call(rig, "state-status")
    assert loaded["saved"] and loaded["states"]["poses"] == {"Ready": 2000}
    assert loaded["states"]["identity"] == list(rig[1])
    assert rig[2]["torque_enabled"] is False


@pytest.mark.parametrize("draft", [False, True])
def test_legacy_automatic_margin_loads_exact_endpoints_and_saves_them(rig, draft):
    calibrated(rig)
    path = states._path(rig[1])
    legacy = states.profiles._read_json(path)
    legacy["safety_margin_ticks"] = 20
    if draft:
        path = path.with_suffix(".draft.json")
        legacy["dirty"] = True
    states.profiles._write_json(path, legacy)
    states._drafts.clear()
    loaded = call(rig, "state-status")
    assert loaded["test_limits"] == {"min": 1000, "home": 2000, "max": 3000}
    assert loaded["states"]["points"] == legacy["points"]
    assert loaded["states"]["safety_margin_ticks"] == 0
    assert call(rig, "save-states")["saved"]
    assert states.profiles._read_json(states._path(rig[1]))["safety_margin_ticks"] == 0


def test_partial_capture_survives_reload_as_draft_until_explicit_save(rig):
    result = call(rig, "capture-state", point="home")
    states._drafts.clear()
    loaded = call(rig, "state-status")
    assert loaded["states"]["points"] == result["states"]["points"]
    assert loaded["states"]["dirty"] and not loaded["saved"]
    assert not call(rig, "arm-test", operator_action="arm-test")["ok"]
    assert call(rig, "save-states")["saved"]
    assert not states._path(rig[1]).with_suffix(".draft.json").exists()


def test_live_hand_position_before_calibration_never_arms_or_captures(rig):
    for ticks in (500, 3500):
        rig[2]["raw_position"] = ticks
        result = call(rig, "read-position", confirm_read_only=True)
        assert result["ok"] and result["raw_position"] == ticks
        assert result["torque_enabled"] is False
        assert result["position_range"] == {"min": 0, "max": 4095}
        assert 0 <= time.time() - result["sampled_at"] < 1
    assert not states._tests
    assert not call(rig, "state-status")["states"]["points"]
    assert not call(rig, "read-position")["ok"]


def test_live_position_does_not_open_another_connection_during_motion(rig):
    calibrated(rig)
    assert call(rig, "arm-test", operator_action="arm-test")["armed"]
    assert not call(rig, "read-position", confirm_read_only=True)["ok"]
    assert states._tests


def test_partial_points_can_be_saved_but_cannot_arm(rig):
    assert call(rig, "capture-state", point="home")["ok"]
    assert call(rig, "save-states")["saved"]
    assert not call(rig, "arm-test", confirm_test=True)["ok"]


@pytest.mark.parametrize("change", ["torque", "warning", "unreadable", "id_mismatch"])
def test_invalid_feedback_does_not_capture_calibration(rig, change):
    if change == "torque": rig[2]["torque_enabled"] = True
    if change == "warning": rig[2]["hardware_error_flags"] = 32
    if change == "unreadable": rig[2]["discovery_status"] = "unreadable"
    if change == "id_mismatch": rig[2]["reported_id"] = 2
    assert not call(rig, "capture-state", point="home")["ok"]
    assert not call(rig, "state-status")["states"]["points"]


def test_invalid_range_does_not_save_or_arm(rig):
    for point in ("min", "home", "max"):
        assert call(rig, "capture-state", point=point)["ok"]
    assert not call(rig, "save-states")["ok"]
    assert not call(rig, "arm-test", confirm_test=True)["ok"]


def test_arm_requires_explicit_confirmation_and_saved_matching_model(rig):
    calibrated(rig)
    assert not call(rig, "arm-test")["ok"]
    rig[2]["model_number"] = 123
    assert not call(rig, "arm-test", confirm_test=True)["ok"]


def test_arm_button_is_explicit_authorization_without_an_extra_checkbox(rig):
    calibrated(rig)
    result = call(rig, "arm-test", operator_action="arm-test")
    assert result["ok"] and result["armed"]
    assert call(rig, "stop-test")["ok"]


@pytest.mark.parametrize("home", [None, 4038, 3300])
def test_arm_accepts_reversed_endpoints_and_derives_test_origin_without_rewriting_captures(rig, home):
    points = {"min": 4038, "max": 2607}
    if home is not None:
        points["home"] = home
    for point, ticks in points.items():
        rig[2]["raw_position"] = ticks
        assert call(rig, "capture-state", point=point)["ok"]
    rig[2]["raw_position"] = 3000
    result = call(rig, "arm-test", operator_action="arm-test", save_calibration=True)
    assert result["ok"] and result["armed"] and result["saved"], result
    assert result["test_limits"] == {"min": 2607, "home": 3300 if home == 3300 else 3322, "max": 4038}
    assert result["states"]["points"] == points
    assert result["target_ticks"] == result["raw_position"] == 3000
    assert states.profiles._read_json(states._path(rig[1]))["points"] == points


@pytest.mark.parametrize("start,target", [(1000, 3000), (3000, 1000)])
def test_arm_holds_endpoint_then_sends_direct_target_once(rig, monkeypatch, start, target):
    calibrated(rig)
    rig[2]["raw_position"] = start
    commands = []
    original = states._motion().command_servo_motion
    def command(run_id, payload):
        commands.append((time.monotonic(), payload))
        return original(run_id, payload)
    monkeypatch.setattr(states._motion(), "command_servo_motion", command)
    result = call(rig, "arm-test", operator_action="arm-test", save_calibration=True)
    assert result["ok"] and result["armed"], result
    assert result["raw_position"] == result["target_ticks"] == start
    time.sleep(0.12)
    assert not commands
    result = call(rig, "test-target", ticks=target, issued_at=time.time())
    assert result["ok"]
    deadline = time.monotonic() + 2
    while result["raw_position"] != target and time.monotonic() < deadline:
        time.sleep(0.02)
        result = call(rig, "test-status")
    assert result["raw_position"] == target and result["armed"], result
    assert len(commands) == 1


def test_direct_slider_target_does_not_walk_through_intermediate_positions(rig, monkeypatch):
    calibrated(rig)
    sent = []
    original = states._motion().command_servo_motion
    def command(run_id, payload):
        sent.append(payload["position_rad"])
        return original(run_id, payload)
    monkeypatch.setattr(states._motion(), "command_servo_motion", command)
    assert call(rig, "arm-test", operator_action="arm-test")["armed"]
    assert call(rig, "test-target", ticks=2900, issued_at=time.time())["ok"]
    deadline = time.monotonic() + 1
    while time.monotonic() < deadline:
        result = call(rig, "test-status")
        if result.get("raw_position") == 2900: break
        time.sleep(0.01)
    assert result["raw_position"] == 2900 and result["armed"]
    assert len(sent) == 1


def test_slider_ack_and_status_reuse_fresh_feedback_without_extra_bus_reads(rig, monkeypatch):
    calibrated(rig)
    assert call(rig, "arm-test", operator_action="arm-test")["armed"]
    # Hold the worker lock so only request-path I/O is measured here.
    with states._lock:
        def unexpected(*args): raise AssertionError("Redundant serial feedback read")
        monkeypatch.setattr(states._motion(), "sample_servo_motion_for_robot", unexpected)
        assert call(rig, "test-target", ticks=2010, issued_at=time.time())["ok"]
        assert call(rig, "test-target", ticks=2020, issued_at=time.time())["ok"]
        assert call(rig, "test-status")["ok"]
        item = states._tests[rig[1]]
        assert item["desired"] == 2020 and item["wake"].is_set()
        # Old feedback must trigger a real read and fail closed, never be reused.
        item["sample_at"] -= 1
        assert not call(rig, "test-status")["ok"]
        assert not states._tests


def test_arm_accept_draft_keeps_captured_limits_and_does_not_save_on_failed_preflight(rig):
    for point, ticks in (("min", 1000), ("max", 3000)):
        rig[2]["raw_position"] = ticks
        assert call(rig, "capture-state", point=point)["ok"]
    rig[2]["raw_position"] = 3001
    result = call(rig, "arm-test", operator_action="arm-test", save_calibration=True)
    assert not result["ok"] and not result["armed"] and not states._tests
    assert "outside the captured endpoints" in result["report"]
    assert not call(rig, "state-status")["saved"]


def test_arm_accept_draft_never_arms_if_save_fails(rig, monkeypatch):
    for point, ticks in (("min", 1000), ("max", 3000)):
        rig[2]["raw_position"] = ticks
        assert call(rig, "capture-state", point=point)["ok"]
    rig[2]["raw_position"] = 2000
    def fail(*args): raise OSError("Disk full")
    monkeypatch.setattr(states.profiles, "_write_json", fail)
    result = call(rig, "arm-test", operator_action="arm-test", save_calibration=True)
    assert not result["ok"] and not result["armed"] and not states._tests
    assert not call(rig, "state-status")["saved"]


def test_arm_button_cannot_bypass_missing_calibration(rig):
    result = call(rig, "arm-test", operator_action="arm-test", save_calibration=True)
    assert not result["ok"] and not result["armed"] and not states._tests


def test_mock_motion_moves_only_selected_joint_and_captures_measured_pose(rig):
    pytest.importorskip("blacknode.pkg.blacknode_motion.arm.servo_control")
    calibrated(rig)
    result = call(rig, "arm-test", confirm_test=True)
    assert result["ok"] and result["armed"], result
    assert result["target_ticks"] == 2000
    result = call(rig, "test-target", ticks=2001, issued_at=time.time())
    assert result["ok"] and result["armed"] and isinstance(result["raw_position"], int), result
    deadline = time.monotonic() + 2
    while result["raw_position"] != 2001 and time.monotonic() < deadline:
        time.sleep(0.05)
        result = call(rig, "test-status")
    assert result["raw_position"] == 2001
    captured = call(rig, "capture-state", point="pose", name="Tested")
    assert captured["ok"] and captured["states"]["poses"]["Tested"] == result["raw_position"]
    assert not call(rig, "capture-state", point="min")["ok"]
    assert not call(rig, "save-states")["ok"]
    assert call(rig, "stop-test")["ok"]
    assert not states._tests
    assert call(rig, "save-states")["saved"]


@pytest.mark.parametrize("target,issued_delta", [(999, 0), (3001, 0), (True, 0), (2000, -5)])
def test_bad_slider_command_disarms_and_closes_session(rig, target, issued_delta):
    calibrated(rig)
    assert call(rig, "arm-test", confirm_test=True)["armed"]
    result = call(rig, "test-target", ticks=target, issued_at=time.time() + issued_delta)
    assert not result["ok"] and not result["armed"] and not states._tests


def test_deadman_releases_when_controls_disconnect(rig, monkeypatch):
    calibrated(rig)
    monkeypatch.setattr(states, "_LEASE_SECONDS", 0.02)
    assert call(rig, "arm-test", confirm_test=True)["armed"]
    item = states._tests[rig[1]]
    assert item["stop"].wait(2), "watchdog must stop the test"
    assert not states._tests
    assert not states._motion().servo_motion_status(item["run_id"])["armed"]


def test_id_change_retires_saved_points_and_named_poses(rig):
    calibrated(rig)
    path = states._path(rig[1])
    assert path.exists()
    assert states.retire_hardware(rig[1][0])
    assert not path.exists() and not states._drafts
    assert not call(rig, "state-status")["states"]["points"]


def test_failed_disk_save_keeps_draft_unsaved(rig, monkeypatch):
    call(rig, "capture-state", point="home")
    def fail(*args): raise OSError("Disk full")
    monkeypatch.setattr(states.profiles, "_write_json", fail)
    assert not call(rig, "save-states")["ok"]
    assert not call(rig, "state-status")["saved"]


def test_unreadable_other_address_does_not_hide_healthy_calibrated_servo(rig):
    calibrated(rig)
    setup._mock_rows["fake"].append({"servo_id": 99, "discovery_status": "unreadable"})
    assert call(rig, "arm-test", confirm_test=True)["armed"]


def test_watchdog_preserves_torque_release_failure_report(rig, monkeypatch):
    calibrated(rig)
    assert call(rig, "arm-test", confirm_test=True)["armed"]
    original = states._motion().disarm_servo_motion
    def release_failed(run_id):
        original(run_id)
        return {"ok": False, "armed": False, "report": "Torque release was not verified"}
    monkeypatch.setattr(states._motion(), "disarm_servo_motion", release_failed)
    states._tests[rig[1]]["heartbeat"] -= 10
    item = states._tests[rig[1]]
    assert item["stop"].wait(2)
    assert "not verified" in call(rig, "test-status")["report"]
