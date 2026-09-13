import copy
import json

import pytest
import blacknode  # noqa: F401
from blacknode.node import _NODE_REGISTRY
from blacknode.pkg.blacknode_robot import actuator_setup as setup


@pytest.fixture
def context(monkeypatch, tmp_path):
    profile = setup.profiles.builtin_profile("so_arm101")
    profile["capability_bindings"] = {"joint_group": {"provider": {
        "package": "blacknode-robot", "component": "capabilities"}}}
    monkeypatch.setattr(setup.profiles, "load_profile", lambda name: (copy.deepcopy(profile), None))
    monkeypatch.setattr(setup.profiles, "_profile_roots", lambda: [tmp_path])
    monkeypatch.setattr(setup.profiles, "list_profiles", lambda: [{"id": "so_arm101"}])
    monkeypatch.setitem(_NODE_REGISTRY, "RobotUSBDiscovery", lambda ctx: {
        "recommended": {"path": "fake", "serial": "physical-123"}})
    setup._tickets.clear()
    setup._mock_rows.clear()
    return {"profile_id": "so_arm101", "serial_port": "fake", "baudrate": 1000000}


def discover(ctx):
    return setup.control_actuator_setup(ctx, "scan", {"confirm_read_only": True})


def confirmation(result):
    return {"scan_token": result["scan_token"], "joint_id": "gripper", "confirm_read_only": True,
            "confirm_isolated": True, "confirm_recalibrate": True}


def test_saved_workflow_cook_is_inert_even_with_injected_action(context, monkeypatch):
    monkeypatch.setattr(setup, "_mock_scan", lambda _: pytest.fail("must not scan"))
    result = setup.actuator_setup({**context, "action": "assign", "confirm_isolated": True})
    assert result["ok"] and not result["assigned"] and not setup._tickets


def test_mock_provider_normalized_contract_and_single_use_authorization(context, tmp_path):
    path = tmp_path / "so_arm101/calibrations/physical_123.json"
    path.parent.mkdir(parents=True)
    path.write_text(json.dumps({"hardware_id": "physical-123", "joints": {}}))
    scan = discover(context)
    assert scan["ok"] and scan["actuators"][0]["servo_id"] == 1
    payload = confirmation(scan)
    result = setup.control_actuator_setup(context, "assign", payload)
    assert result["ok"] and result["assigned"] and result["new_id"] == 6
    assert result["actuators"][0]["torque_enabled"] is False
    assert not path.exists() and len(result["retired_calibrations"]) == 1
    assert not setup.control_actuator_setup(context, "assign", payload)["ok"]
    assert discover(context)["actuators"][0]["servo_id"] == 6


@pytest.mark.parametrize("field", ["confirm_isolated", "confirm_recalibrate", "confirm_read_only"])
def test_missing_confirmation_never_assigns(context, field):
    payload = confirmation(discover(context))
    payload[field] = False
    result = setup.control_actuator_setup(context, "assign", payload)
    assert not result["ok"] and setup._mock_scan({"port": "fake"})["actuators"][0]["servo_id"] == 1


def test_expired_ticket_blocks_assignment(context, monkeypatch):
    payload = confirmation(discover(context))
    now = setup.time.monotonic()
    monkeypatch.setattr(setup.time, "monotonic", lambda: now + 121)
    result = setup.control_actuator_setup(context, "assign", payload)
    assert not result["ok"] and "expired" in result["report"]


def test_changed_port_baud_or_hardware_invalidates_scan(context):
    payload = confirmation(discover(context))
    result = setup.control_actuator_setup({**context, "baudrate": 115200}, "assign", payload)
    assert not result["ok"] and "selection changed" in result["report"]


def test_provider_absence_is_structured_and_cook_opens_no_hardware(context, monkeypatch):
    monkeypatch.delitem(_NODE_REGISTRY, "RobotActuatorSetupMockProvider")
    result = setup.actuator_setup(context)
    assert not result["ok"] and "provider unavailable" in result["report"]


def test_scan_invalidates_older_confirmation_for_same_hardware(context):
    first = discover(context)
    second = discover(context)
    assert first["scan_token"] != second["scan_token"]
    assert not setup.control_actuator_setup(context, "assign", confirmation(first))["ok"]


def test_multiple_actuators_and_invalid_joint_block_assignment(context):
    setup._mock_rows["fake"] = [{"servo_id": 1}, {"servo_id": 2}]
    result = setup.control_actuator_setup(context, "assign", confirmation(discover(context)))
    assert not result["ok"] and "exactly one" in result["report"]
    setup._mock_rows.clear()
    payload = {**confirmation(discover(context)), "joint_id": "unknown"}
    assert not setup.control_actuator_setup(context, "assign", payload)["ok"]


def test_profile_defaults_resolve_feetech_without_sdk_access(monkeypatch):
    monkeypatch.setattr(setup.profiles, "load_profile", lambda name: (setup.profiles.builtin_profile(name), None))
    if "FeetechActuatorSetupProvider" not in _NODE_REGISTRY:
        pytest.skip("driver package not installed")
    assert setup.actuator_setup({"profile_id": "so_arm101", "serial_port": "fake"})["ok"]


def test_release_requires_isolation_and_consumes_scan(context):
    rows = setup._mock_scan({"port": "fake"})["actuators"]
    rows[0]["torque_enabled"] = True
    setup._mock_rows["fake"] = rows
    payload = confirmation(discover(context))
    assert not setup.control_actuator_setup(context, "release", {**payload, "confirm_isolated": False})["ok"]
    result = setup.control_actuator_setup(context, "release", {**payload, "confirm_recalibrate": False})
    assert result["ok"] and result["released"] and not result["assigned"]
    assert not setup.control_actuator_setup(context, "assign", payload)["ok"]


def test_usb_only_scan_and_numeric_id_need_no_profile(context, monkeypatch):
    provider = dict(setup.robot_actuator_setup_mock_provider._bn_robot_actuator_setup_provider)
    provider["match_hardware"] = lambda hardware: 100 if hardware["port"] == "fake" else 0
    monkeypatch.setattr(setup.robot_actuator_setup_mock_provider, "_bn_robot_actuator_setup_provider", provider)
    monkeypatch.setattr(setup.profiles, "load_profile", lambda _: pytest.fail("USB discovery must not need a profile"))
    monkeypatch.setattr(setup.profiles, "list_profiles", lambda: [])
    ctx = {"serial_port": "fake", "servo_id": 1}
    scan = discover(ctx)
    assert scan["ok"] and scan["profile"] == {} and scan["bus"]["port"] == "fake"
    result = setup.control_actuator_setup(ctx, "assign", {**confirmation(scan), "new_id": 7})
    assert result["ok"] and result["new_id"] == 7


def test_servo_card_cannot_program_another_isolated_servo(context):
    scan = discover(context)
    result = setup.control_actuator_setup({**context, "servo_id": 2}, "assign", {
        **confirmation(scan), "new_id": 7})
    assert not result["ok"] and "does not match" in result["report"]
    assert discover(context)["actuators"][0]["servo_id"] == 1


@pytest.mark.parametrize("new_id", [0, 254, True, "6", 1.5])
def test_direct_id_rejects_invalid_values(context, new_id):
    result = setup.control_actuator_setup(context, "assign", {
        **confirmation(discover(context)), "new_id": new_id})
    assert not result["ok"] and "1 to 253" in result["report"]


def test_scan_keeps_unresolved_addresses_separate_from_confirmed_ids(context):
    setup._mock_rows["fake"] = [{"servo_id": 1, "discovery_status": "unreadable"},
                                {"servo_id": 2, "model": "Mock"}]
    result = discover(context)
    assert result["ok"] and len(result["actuators"]) == 2
    assert "1 responding IDs" in result["report"] and "Unreadable replies at IDs 1" in result["report"]
    assert 1 in result["missing_ids"] and 2 not in result["missing_ids"]


@pytest.mark.parametrize("action", ["assign", "release"])
def test_unresolved_address_cannot_authorize_writes(context, action):
    setup._mock_rows["fake"] = [{"servo_id": 1, "discovery_status": "unreadable"}]
    result = setup.control_actuator_setup(context, action, confirmation(discover(context)))
    assert not result["ok"] and "address is unresolved" in result["report"]


def test_set_id_button_scans_and_assigns_without_checkboxes_or_stale_tokens(context):
    result = setup.control_actuator_setup({**context, "servo_id": 1}, "assign", {
        "operator_action": "assign", "new_id": 6, "confirm_read_only": True})
    assert result["ok"] and result["assigned"] and result["new_id"] == 6
    assert not setup._tickets


@pytest.mark.parametrize("problem", ["multiple", "unreadable", "different_id", "torque", "warning"])
def test_set_id_button_checks_actual_bus_before_writing(context, problem):
    setup._mock_scan({"port": "fake"})
    rows = setup._mock_rows["fake"]
    if problem == "multiple": rows.append({"servo_id": 2})
    if problem == "unreadable": rows[0]["discovery_status"] = "unreadable"
    if problem == "different_id": rows[0]["servo_id"] = 2
    if problem == "torque": rows[0]["torque_enabled"] = True
    if problem == "warning": rows[0]["hardware_error_flags"] = 32
    result = setup.control_actuator_setup({**context, "servo_id": 1}, "assign", {
        "operator_action": "assign", "new_id": 6, "confirm_read_only": True})
    assert not result["ok"] and not result["assigned"]
    assert setup._mock_rows["fake"][0]["servo_id"] != 6


def test_setting_current_id_keeps_existing_calibration(context, monkeypatch):
    monkeypatch.setattr(setup, "_retire_calibrations", lambda _: pytest.fail("unchanged ID must preserve calibration"))
    result = setup.control_actuator_setup({**context, "servo_id": 1}, "assign", {
        "operator_action": "assign", "new_id": 1, "confirm_read_only": True})
    assert result["ok"] and not result["retired_calibrations"]
