from gradient_os.telemetry import drive_faults
from gradient_os.telemetry.drive_faults import (
    build_drive_fault_snapshot,
    build_startup_fault_reset_plan,
)


def test_build_startup_fault_reset_plan_targets_disarmed_faulted_axes():
    plan = build_startup_fault_reset_plan(
        {
            "physical_state": "FAULTED",
            "driver_state": "DISARMED",
            "rtcore_state": "UP",
            "armed": 0,
            "axis_enable_mask": 0,
            "op_enabled_axes": 0,
            "axes": [
                {"axis": 0, "logical_joint": 1, "ds402_state": "SwitchOnDisabled", "error_code": 0},
                {
                    "axis": 2,
                    "logical_joint": 3,
                    "ds402_state": "Fault",
                    "error_code": 0x8700,
                    "error_code_hex": "0x8700",
                    "fault": {"code": "ErC2.0", "name": "SYNC signal loss", "resettable": True},
                },
            ],
        }
    )

    assert plan["should_auto_reset"] is True
    assert plan["blocks_startup"] is False
    assert plan["faulted_axis_count"] == 1
    assert plan["faulted_axis_mask"] == 0x4
    assert plan["faulted_axis_mask_hex"] == "0x4"
    assert "J3/axis2" in plan["faulted_summary"]
    assert "0x8700" in plan["faulted_summary"]


def test_build_startup_fault_reset_plan_blocks_when_fault_is_not_disarmed():
    plan = build_startup_fault_reset_plan(
        {
            "physical_state": "FAULTED",
            "driver_state": "ACTIVE",
            "rtcore_state": "UP",
            "armed": 1,
            "axis_enable_mask": 0x4,
            "op_enabled_axes": 1,
            "axes": [
                {
                    "axis": 2,
                    "logical_joint": 3,
                    "ds402_state": "Fault",
                    "error_code": 0x8700,
                },
            ],
        }
    )

    assert plan["should_auto_reset"] is False
    assert plan["blocks_startup"] is True
    assert "hardware_not_disarmed" in plan["reason"]


def test_build_startup_fault_reset_plan_ignores_clean_bus_state():
    plan = build_startup_fault_reset_plan(
        {
            "physical_state": "BUS_UP_DISARMED",
            "driver_state": "DISARMED",
            "rtcore_state": "UP",
            "armed": 0,
            "axis_enable_mask": 0,
            "op_enabled_axes": 0,
            "axes": [
                {"axis": 0, "logical_joint": 1, "ds402_state": "SwitchOnDisabled", "error_code": 0},
                {"axis": 1, "logical_joint": 2, "ds402_state": "SwitchOnDisabled", "error_code": 0},
            ],
        }
    )

    assert plan["should_auto_reset"] is False
    assert plan["blocks_startup"] is False
    assert plan["faulted_axis_mask"] == 0
    assert plan["reason"] == "no_faulted_axes"


def test_build_drive_fault_snapshot_uses_feedback_for_driver_state(monkeypatch):
    monkeypatch.setattr(
        drive_faults.backend_registry,
        "resolve_drive_profile_for_backend",
        lambda *args, **kwargs: "a6ec_ds402",
    )
    monkeypatch.setattr(
        drive_faults.backend_registry,
        "resolve_fieldbus_profile_for_backend",
        lambda *args, **kwargs: "test_fieldbus",
    )
    monkeypatch.setattr(
        drive_faults.backend_registry,
        "get_drive_fault_reference_metadata_for_backend",
        lambda *args, **kwargs: {"available": False},
    )
    monkeypatch.setattr(
        drive_faults.backend_registry,
        "decode_fieldbus_state_for_backend",
        lambda *args, **kwargs: {"name": "OP"},
    )
    monkeypatch.setattr(
        drive_faults.backend_registry,
        "decode_drive_statusword_for_backend",
        lambda *args, **kwargs: {"state": "NotReady"},
    )
    monkeypatch.setattr(
        drive_faults.backend_registry,
        "extract_drive_startup_config_axis_for_backend",
        lambda *args, **kwargs: None,
    )
    monkeypatch.setattr(
        drive_faults.backend_registry,
        "describe_drive_fault_code_for_backend",
        lambda *args, **kwargs: None,
    )
    monkeypatch.setattr(
        drive_faults.backend_registry,
        "describe_drive_manufacturer_fault_code_for_backend",
        lambda *args, **kwargs: None,
    )
    monkeypatch.setattr(
        drive_faults.backend_registry,
        "describe_fieldbus_master_state_for_backend",
        lambda *args, **kwargs: "OP",
    )

    snapshot = build_drive_fault_snapshot(
        metrics={
            "num_axes": 2,
            "armed": 1,
            "axis_enable_mask": 0x3,
            "link_up": 1,
            "responding_slaves": 2,
            "online_slaves": 2,
            "operational_slaves": 2,
            "startup_ready": 1,
            "wkc_actual": 4,
            "wkc_expected": 4,
            "master_al_states": 0x08,
            "axes": [
                {
                    "statusword": 0,
                    "error_code": 0,
                    "manufacturer_error_code": 0,
                    "slave_online": 1,
                    "slave_operational": 1,
                    "slave_al_state": 0x08,
                    "pos_counts": 0,
                },
                {
                    "statusword": 0,
                    "error_code": 0,
                    "manufacturer_error_code": 0,
                    "slave_online": 1,
                    "slave_operational": 1,
                    "slave_al_state": 0x08,
                    "pos_counts": 0,
                },
            ],
        },
        servo_backend="ethercat_rtcore",
        axis_to_joint=[0, 1],
        socket_present=True,
    )

    assert snapshot["driver_state"] == "DISARMED"
    assert snapshot["physical_state"] == "BUS_UP_DISARMED"
    assert snapshot["enable_requested"] is True
    assert snapshot["requested_axes"] == 2
    assert snapshot["op_enabled_axes"] == 0
    assert snapshot["statusword_feedback_axes"] == 0
    assert snapshot["slave_online_axes"] == 2
    assert snapshot["slave_operational_axes"] == 2


def test_build_drive_fault_snapshot_carries_native_home_status(monkeypatch):
    monkeypatch.setattr(
        drive_faults.backend_registry,
        "resolve_drive_profile_for_backend",
        lambda *args, **kwargs: "test_drive",
    )
    monkeypatch.setattr(
        drive_faults.backend_registry,
        "resolve_fieldbus_profile_for_backend",
        lambda *args, **kwargs: "test_fieldbus",
    )
    monkeypatch.setattr(
        drive_faults.backend_registry,
        "get_drive_fault_reference_metadata_for_backend",
        lambda *args, **kwargs: {"available": False},
    )
    monkeypatch.setattr(
        drive_faults.backend_registry,
        "decode_fieldbus_state_for_backend",
        lambda *args, **kwargs: {"name": "OP"},
    )
    monkeypatch.setattr(
        drive_faults.backend_registry,
        "decode_drive_statusword_for_backend",
        lambda *args, **kwargs: {"state": "SwitchOnDisabled"},
    )
    monkeypatch.setattr(
        drive_faults.backend_registry,
        "extract_drive_startup_config_axis_for_backend",
        lambda *args, **kwargs: None,
    )
    monkeypatch.setattr(
        drive_faults.backend_registry,
        "describe_drive_fault_code_for_backend",
        lambda *args, **kwargs: None,
    )
    monkeypatch.setattr(
        drive_faults.backend_registry,
        "describe_drive_manufacturer_fault_code_for_backend",
        lambda *args, **kwargs: None,
    )
    monkeypatch.setattr(
        drive_faults.backend_registry,
        "describe_fieldbus_master_state_for_backend",
        lambda *args, **kwargs: "OP",
    )

    snapshot = build_drive_fault_snapshot(
        metrics={
            "num_axes": 1,
            "armed": 1,
            "axis_enable_mask": 0x0,
            "link_up": 1,
            "responding_slaves": 1,
            "online_slaves": 1,
            "operational_slaves": 1,
            "startup_ready": 1,
            "wkc_actual": 2,
            "wkc_expected": 2,
            "master_al_states": 0x08,
            "axes": [
                {
                    "statusword": 0x1638,
                    "error_code": 0,
                    "manufacturer_error_code": 0,
                    "slave_online": 1,
                    "slave_operational": 1,
                    "slave_al_state": 0x08,
                    "pos_counts": 122784,
                    "native_home_state": 2,
                    "native_home_position_offset": -244354,
                    "native_home_last_abort_code": 0,
                },
            ],
        },
        servo_backend="ethercat_rtcore",
        axis_to_joint=[1],
        socket_present=True,
    )

    axis = snapshot["axes"][0]
    assert axis["logical_joint"] == 2
    assert axis["native_home_state"] == 2
    assert axis["native_home_state_name"] == "succeeded"
    assert axis["native_home_active"] is False
    assert axis["native_home_position_offset"] == -244354
    assert axis["native_home_last_abort_code_hex"] == "0x00000000"


def test_build_drive_fault_snapshot_prefers_live_statusword_for_native_home_success(monkeypatch):
    monkeypatch.setattr(
        drive_faults.backend_registry,
        "resolve_drive_profile_for_backend",
        lambda *args, **kwargs: "test_drive",
    )
    monkeypatch.setattr(
        drive_faults.backend_registry,
        "resolve_fieldbus_profile_for_backend",
        lambda *args, **kwargs: "test_fieldbus",
    )
    monkeypatch.setattr(
        drive_faults.backend_registry,
        "get_drive_fault_reference_metadata_for_backend",
        lambda *args, **kwargs: {"available": False},
    )
    monkeypatch.setattr(
        drive_faults.backend_registry,
        "decode_fieldbus_state_for_backend",
        lambda *args, **kwargs: {"name": "OP"},
    )
    monkeypatch.setattr(
        drive_faults.backend_registry,
        "decode_drive_statusword_for_backend",
        lambda *args, **kwargs: {"state": "SwitchOnDisabled"},
    )
    monkeypatch.setattr(
        drive_faults.backend_registry,
        "extract_drive_startup_config_axis_for_backend",
        lambda *args, **kwargs: None,
    )
    monkeypatch.setattr(
        drive_faults.backend_registry,
        "describe_drive_fault_code_for_backend",
        lambda *args, **kwargs: None,
    )
    monkeypatch.setattr(
        drive_faults.backend_registry,
        "describe_drive_manufacturer_fault_code_for_backend",
        lambda *args, **kwargs: None,
    )
    monkeypatch.setattr(
        drive_faults.backend_registry,
        "describe_fieldbus_master_state_for_backend",
        lambda *args, **kwargs: "OP",
    )

    snapshot = build_drive_fault_snapshot(
        metrics={
            "num_axes": 1,
            "armed": 0,
            "axis_enable_mask": 0x0,
            "link_up": 1,
            "responding_slaves": 1,
            "online_slaves": 1,
            "operational_slaves": 1,
            "startup_ready": 1,
            "wkc_actual": 2,
            "wkc_expected": 2,
            "master_al_states": 0x08,
            "axes": [
                {
                    "statusword": 0x9650,
                    "error_code": 0,
                    "manufacturer_error_code": 0,
                    "slave_online": 1,
                    "slave_operational": 1,
                    "slave_al_state": 0x08,
                    "pos_counts": 131060,
                    "native_home_state": 3,
                    "native_home_position_offset": 0,
                    "native_home_last_abort_code": 0x06010002,
                },
            ],
        },
        servo_backend="ethercat_rtcore",
        axis_to_joint=[2],
        socket_present=True,
    )

    axis = snapshot["axes"][0]
    assert axis["native_home_state"] == 2
    assert axis["native_home_state_name"] == "succeeded"
    assert axis["native_home_last_abort_code"] == 0
    assert axis["native_home_last_abort_code_hex"] == "0x00000000"
    assert axis["native_home_state_reported"] == 3
    assert axis["native_home_state_reported_name"] == "failed"
    assert axis["native_home_last_abort_code_reported"] == 0x06010002
    assert axis["native_home_verification_source"] == "statusword_bit15"


def test_build_drive_fault_snapshot_normalizes_absolute_feedback(monkeypatch):
    monkeypatch.setattr(
        drive_faults.backend_registry,
        "resolve_drive_profile_for_backend",
        lambda *args, **kwargs: "a6ec_ds402",
    )
    monkeypatch.setattr(
        drive_faults.backend_registry,
        "resolve_fieldbus_profile_for_backend",
        lambda *args, **kwargs: "test_fieldbus",
    )
    monkeypatch.setattr(
        drive_faults.backend_registry,
        "get_drive_fault_reference_metadata_for_backend",
        lambda *args, **kwargs: {"available": False},
    )
    monkeypatch.setattr(
        drive_faults.backend_registry,
        "decode_fieldbus_state_for_backend",
        lambda *args, **kwargs: {"name": "OP"},
    )
    monkeypatch.setattr(
        drive_faults.backend_registry,
        "decode_drive_statusword_for_backend",
        lambda *args, **kwargs: {"state": "SwitchOnDisabled"},
    )
    monkeypatch.setattr(
        drive_faults.backend_registry,
        "extract_drive_startup_config_axis_for_backend",
        lambda *args, **kwargs: None,
    )
    monkeypatch.setattr(
        drive_faults.backend_registry,
        "describe_drive_fault_code_for_backend",
        lambda *args, **kwargs: None,
    )
    monkeypatch.setattr(
        drive_faults.backend_registry,
        "describe_drive_manufacturer_fault_code_for_backend",
        lambda *args, **kwargs: None,
    )
    monkeypatch.setattr(
        drive_faults.backend_registry,
        "describe_fieldbus_master_state_for_backend",
        lambda *args, **kwargs: "OP",
    )

    snapshot = build_drive_fault_snapshot(
        metrics={
            "num_axes": 1,
            "armed": 0,
            "axis_enable_mask": 0x0,
            "link_up": 1,
            "responding_slaves": 1,
            "online_slaves": 1,
            "operational_slaves": 1,
            "startup_ready": 1,
            "wkc_actual": 2,
            "wkc_expected": 2,
            "master_al_states": 0x08,
            "axes": [
                {
                    "statusword": 0x1650,
                    "error_code": 0,
                    "manufacturer_error_code": 0,
                    "slave_online": 1,
                    "slave_operational": 1,
                    "slave_al_state": 0x08,
                    "pos_counts": 25,
                    "absolute_feedback": {
                        "encoder_multi_turn_low": {"valid": 1, "value": 1234},
                        "encoder_multi_turn_high": {"valid": 1, "value": 0},
                        "rotation_mode_encoder_low": {"valid": 1, "value": 5678},
                        "rotation_mode_encoder_high": {"valid": 1, "value": 0},
                    },
                },
            ],
        },
        servo_backend="ethercat_rtcore",
        axis_to_joint=[0],
        socket_present=True,
    )

    axis = snapshot["axes"][0]
    assert axis["absolute_feedback"] is not None
    assert axis["absolute_feedback"]["encoder_multi_turn_low"]["valid"] is True
    assert axis["absolute_feedback"]["encoder_multi_turn_low"]["value"] == 1234
    assert axis["absolute_feedback"]["encoder_multi_turn_counts"]["value"] == 1234
    assert axis["absolute_feedback"]["rotation_mode_encoder_counts"]["value"] == 5678


def test_build_drive_fault_snapshot_carries_native_home_active_mask(monkeypatch):
    monkeypatch.setattr(
        drive_faults.backend_registry,
        "resolve_drive_profile_for_backend",
        lambda *args, **kwargs: "test_drive",
    )
    monkeypatch.setattr(
        drive_faults.backend_registry,
        "resolve_fieldbus_profile_for_backend",
        lambda *args, **kwargs: "test_fieldbus",
    )
    monkeypatch.setattr(
        drive_faults.backend_registry,
        "get_drive_fault_reference_metadata_for_backend",
        lambda *args, **kwargs: {"available": False},
    )
    monkeypatch.setattr(
        drive_faults.backend_registry,
        "decode_fieldbus_state_for_backend",
        lambda *args, **kwargs: {"name": "OP"},
    )
    monkeypatch.setattr(
        drive_faults.backend_registry,
        "decode_drive_statusword_for_backend",
        lambda *args, **kwargs: {"state": "SwitchOnDisabled"},
    )
    monkeypatch.setattr(
        drive_faults.backend_registry,
        "extract_drive_startup_config_axis_for_backend",
        lambda *args, **kwargs: None,
    )
    monkeypatch.setattr(
        drive_faults.backend_registry,
        "describe_drive_fault_code_for_backend",
        lambda *args, **kwargs: None,
    )
    monkeypatch.setattr(
        drive_faults.backend_registry,
        "describe_drive_manufacturer_fault_code_for_backend",
        lambda *args, **kwargs: None,
    )
    monkeypatch.setattr(
        drive_faults.backend_registry,
        "describe_fieldbus_master_state_for_backend",
        lambda *args, **kwargs: "OP",
    )

    snapshot = build_drive_fault_snapshot(
        metrics={
            "num_axes": 2,
            "armed": 0,
            "axis_enable_mask": 0x0,
            "native_home_active_axis_mask": 0x2,
            "link_up": 1,
            "responding_slaves": 2,
            "online_slaves": 2,
            "operational_slaves": 2,
            "startup_ready": 1,
            "wkc_actual": 4,
            "wkc_expected": 4,
            "master_al_states": 0x08,
            "axes": [
                {
                    "statusword": 0x1650,
                    "error_code": 0,
                    "manufacturer_error_code": 0,
                    "slave_online": 1,
                    "slave_operational": 1,
                    "slave_al_state": 0x08,
                    "pos_counts": 0,
                    "native_home_state": 0,
                    "native_home_last_abort_code": 0,
                },
                {
                    "statusword": 0x1650,
                    "error_code": 0,
                    "manufacturer_error_code": 0,
                    "slave_online": 1,
                    "slave_operational": 1,
                    "slave_al_state": 0x08,
                    "pos_counts": 0,
                    "native_home_state": 0,
                    "native_home_last_abort_code": 0,
                },
            ],
        },
        servo_backend="ethercat_rtcore",
        axis_to_joint=[0, 1],
        socket_present=True,
    )

    assert snapshot["native_home_active_axis_mask"] == 0x2
    assert snapshot["native_home_active_axis_mask_hex"] == "0x2"
    assert snapshot["axes"][0]["native_home_active"] is False
    assert snapshot["axes"][1]["native_home_active"] is True
