from gradient_os.telemetry import drive_faults
from gradient_os.telemetry.drive_faults import (
    build_drive_fault_snapshot,
    build_startup_fault_reset_plan,
)
from gradient_os.telemetry.native_home_status import statusword_indicates_valid_native_home_reference


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
    # Plain resettable DS402 faults route through the pulse path only.
    assert plan["ds402_pulse_required"] is True
    assert plan["ds402_pulse_axis_mask"] == 0x4
    assert plan["ds402_pulse_axis_mask_hex"] == "0x4"
    assert plan["encoder_reset_required"] is False
    assert plan["encoder_reset_axis_mask"] == 0x0
    assert plan["encoder_reset_logical_joints"] == []
    assert plan["unresettable_axis_mask"] == 0x0
    assert plan["faulted_axes"][0]["reset_action"] == "ds402_fault_pulse"


def test_build_startup_fault_reset_plan_routes_encoder_retention_to_f31_10():
    """A6-EC encoder-retention fault (Er20.8 = encoder battery failure
    after the cable was disconnected) must be classified into the
    encoder-reset bucket and the affected logical joint surfaced for
    anchor invalidation. DS402-pulse-only axes on the same bus stay
    in the pulse bucket so the two recovery paths can run
    side-by-side.
    """
    plan = build_startup_fault_reset_plan(
        {
            "physical_state": "FAULTED",
            "driver_state": "FAULTED",
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
                    "error_code": 0x0208,
                    "error_code_hex": "0x0208",
                    "fault": {"code": "Er20.8", "name": "Encoder battery failure", "resettable": False},
                    "encoder_retention_fault_present": True,
                    "encoder_retention_fault": {
                        "present": True,
                        "codes": ["Er20.8"],
                        "names": ["Encoder battery failure"],
                        "matched_sources": ["manufacturer_error_code"],
                    },
                },
                {
                    "axis": 5,
                    "logical_joint": 6,
                    "ds402_state": "Fault",
                    "error_code": 0x8700,
                    "error_code_hex": "0x8700",
                    "fault": {"code": "ErC2.0", "name": "SYNC signal loss", "resettable": True},
                    "encoder_retention_fault_present": False,
                },
            ],
        }
    )

    assert plan["should_auto_reset"] is True
    assert plan["blocks_startup"] is False
    assert plan["faulted_axis_mask"] == 0x24
    # Encoder-retention axis (J3) goes through F31.10 + anchor invalidation.
    assert plan["encoder_reset_required"] is True
    assert plan["encoder_reset_axis_mask"] == 0x4
    assert plan["encoder_reset_axis_mask_hex"] == "0x4"
    # Anchor store is 0-indexed while the probe uses 1-indexed logical
    # joints; the plan must translate so the caller can pass the list
    # straight into ``invalidate_absolute_encoder_anchors``.
    assert plan["encoder_reset_logical_joints"] == [2]
    # DS402 pulse mask MUST include the encoder-retention axis too -
    # F31.10 clears the encoder-internal fault but the DS402 state
    # machine stays latched in Fault until a controlword 0x80 pulse
    # re-runs the fault-reset edge. Without this the preflight waits
    # forever for BUS_UP_DISARMED even though the SDO write succeeded
    # (observed live 2026-04-23).
    assert plan["ds402_pulse_required"] is True
    assert plan["ds402_pulse_axis_mask"] == 0x24  # encoder-reset (0x4) + SYNC-loss (0x20)
    assert plan["ds402_pulse_axis_mask_hex"] == "0x24"
    assert plan["unresettable_axis_mask"] == 0x0
    assert plan["reason"] == "encoder_retention_and_ds402_resets_required"
    # Summary marks the retention axis so operators can see it in logs.
    assert "[encoder-retention]" in plan["faulted_summary"]
    # Per-axis reset_action mirrors the bucketing so downstream code
    # can drive the right tool per axis.
    actions = {entry["axis"]: entry["reset_action"] for entry in plan["faulted_axes"]}
    assert actions == {2: "encoder_data_reset_then_ds402_pulse", 5: "ds402_fault_pulse"}


def test_build_startup_fault_reset_plan_encoder_retention_only_reason():
    """When EVERY faulted axis is encoder-retention (the typical case
    after disconnecting all encoder cables), the plan reason narrows
    to ``encoder_retention_reset_required`` so the preflight message
    is precise instead of the generic "ready for reset" label.
    """
    plan = build_startup_fault_reset_plan(
        {
            "physical_state": "FAULTED",
            "driver_state": "FAULTED",
            "rtcore_state": "UP",
            "armed": 0,
            "axis_enable_mask": 0,
            "op_enabled_axes": 0,
            "axes": [
                {
                    "axis": axis_i,
                    "logical_joint": axis_i + 1,
                    "ds402_state": "Fault",
                    "error_code": 0x0208,
                    "error_code_hex": "0x0208",
                    "fault": {"code": "Er20.8", "name": "Encoder battery failure", "resettable": False},
                    "encoder_retention_fault_present": True,
                }
                for axis_i in (2, 3, 4, 5)
            ],
        }
    )

    assert plan["should_auto_reset"] is True
    assert plan["encoder_reset_required"] is True
    assert plan["encoder_reset_axis_mask"] == 0x3C
    # DS402 pulse mask tracks the same axes because encoder-reset
    # requires the follow-up pulse to leave DS402 Fault state. This
    # is the fix for the "F31.10 sent but drives stayed in Fault"
    # regression found live on 2026-04-23.
    assert plan["ds402_pulse_required"] is True
    assert plan["ds402_pulse_axis_mask"] == 0x3C
    assert plan["reason"] == "encoder_retention_reset_required"
    assert plan["encoder_reset_logical_joints"] == [2, 3, 4, 5]


def test_drive_faults_and_ethercat_backend_import_without_circular_dependency():
    from gradient_os.arm_controller.backends.ethercat_rtcore.backend import EthercatRTCoreBackend

    assert callable(build_startup_fault_reset_plan)
    assert EthercatRTCoreBackend is not None


def test_statusword_indicates_valid_native_home_reference_requires_vendor_success_bits():
    assert statusword_indicates_valid_native_home_reference(0x9650) is True
    assert statusword_indicates_valid_native_home_reference(0x8650) is False
    assert statusword_indicates_valid_native_home_reference(0xB650) is False


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
    assert axis["native_home_verification_source"] == "statusword_bits12_15_clear13"


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


def test_a6ec_gear_ratio_u16_pair_recovers_100_over_11_from_ieee754_float():
    """A6-EC C10.18 / C10.19 gear-ratio SDO writes must fit in u16
    registers. The robot config stores ``actuator_gear_ratios`` as a
    list of floats; non-trivial mechanical ratios such as ``100/11``
    round-trip in Python as the float ``9.090909090909091``. The naive
    ``Fraction(str(float))`` path parses that decimal string into
    ``9090909090909091 / 1000000000000000`` which overflows the u16
    limit and would break startup. Verify the profile helper recovers
    the exact ``(100, 11)`` pair via
    ``Fraction(raw_float).limit_denominator(0xFFFF)``.
    """
    from gradient_os.arm_controller.profiles.drive.a6ec_ds402 import (
        _ratio_u16_pair_for_axis,
    )

    robot_cfg = {
        "actuator_gear_ratios": [100.0, 100.0, 100.0, 18.0, 9.090909090909091, 10.0],
    }
    numerator, denominator = _ratio_u16_pair_for_axis(4, robot_config=robot_cfg)
    assert (numerator, denominator) == (100, 11)
    # Exact-representable ratios (ints, power-of-two denominators) must
    # round-trip without limit_denominator touching them.
    for axis_i, expected in [
        (0, (100, 1)),   # 100.0
        (3, (18, 1)),    # 18.0
        (5, (10, 1)),    # 10.0
    ]:
        assert _ratio_u16_pair_for_axis(axis_i, robot_config=robot_cfg) == expected


def test_a6ec_gear_ratio_u16_pair_accepts_exact_fraction_string():
    """Operators can supply exact rationals as strings when the
    intended value is not IEEE754-exact (``"100/11"`` is nicer than
    ``9.090909090909091`` in a human review). Both paths must land
    on the same u16 pair.
    """
    from gradient_os.arm_controller.profiles.drive.a6ec_ds402 import (
        _ratio_u16_pair_for_axis,
    )

    robot_cfg = {"actuator_gear_ratios": [1.0, 1.0, 1.0, 1.0, "100/11", 1.0]}
    assert _ratio_u16_pair_for_axis(4, robot_config=robot_cfg) == (100, 11)


def test_a6ec_gear_ratio_u16_pair_rejects_ratio_that_cannot_be_approximated():
    """If the input float is genuinely far from any u16/u16 rational
    (within 1 ppm relative tolerance), the helper must raise instead
    of silently snapping to a wildly different value. Pick a ratio
    with a prime denominator larger than 65535 so limit_denominator
    is forced to deviate beyond the tolerance.
    """
    import math

    from gradient_os.arm_controller.profiles.drive.a6ec_ds402 import (
        _ratio_u16_pair_for_axis,
    )

    robot_cfg = {"actuator_gear_ratios": [1.0, 1.0, 1.0, 1.0, math.pi, 1.0]}
    try:
        _ratio_u16_pair_for_axis(4, robot_config=robot_cfg)
    except ValueError as exc:
        assert "1 ppm" in str(exc) or "u16" in str(exc)
    else:
        # math.pi has no u16/u16 approximation within 1 ppm; the helper
        # must refuse it loudly.
        raise AssertionError("Expected ValueError for math.pi gear ratio")


def test_rtcore_env_emits_gear_ratio_with_full_precision():
    """The ``GRADIENT_RT_GEAR_RATIO`` env var must preserve enough
    float precision that RTCore's host-side ``counts_per_unit`` math
    stays in lockstep with exact drive SDO pairs. The previous ``:g``
    format truncated to 6 sig figs, which could silently drift host
    vs drive for non-trivial ratios.
    Lock in ``repr(float)`` (shortest-round-trip) so the C++ parser
    reconstructs the exact IEEE754 double the profile used.
    """
    from gradient_os.arm_controller.robots import get_robot_config
    from gradient_os.arm_controller.backends.ethercat_rtcore.runtime import (
        render_rtcore_systemd_env,
    )

    robot = get_robot_config("gradient05").get_config_dict()
    env = render_rtcore_systemd_env(
        robot_config=robot, drive_profile="a6ec_ds402", max_rpm=6000
    )
    gear_line = next(
        (line for line in env.splitlines() if line.startswith("GRADIENT_RT_GEAR_RATIO=")),
        None,
    )
    assert gear_line is not None, "GRADIENT_RT_GEAR_RATIO missing from env"
    # Strip KEY= and surrounding quotes so downstream parsing can
    # split on comma without worrying about shell quoting.
    value = gear_line.split("=", 1)[1].strip().strip('"').strip("'")
    tokens = value.split(",")
    assert len(tokens) == 6
    # Each token must parse back into the exact same IEEE754 double
    # that the profile saw (which is what ``repr(float)`` guarantees).
    expected = robot["actuator_gear_ratios"][:6]
    for token, raw in zip(tokens, expected):
        parsed = float(token)
        assert parsed == float(raw), (
            f"gear ratio token {token!r} lost precision: re-parsed as {parsed!r} "
            f"but robot config has {raw!r}"
        )


def test_describe_encoder_retention_fault_falls_back_to_bus_code_when_203f_is_zero():
    """When the drive does not PDO-map 0x203F (observed live on
    A6-EC), the manufacturer code reads as zero but the firmware
    publishes the same numeric Er20.8 value (0x0208) on the 0x603F
    bus code path. The retention classifier must still fire so the
    startup preflight can run the F31.10 encoder reset instead of
    walking into the useless DS402-pulse-only recovery.
    """
    from gradient_os.arm_controller.profiles.drive.a6ec_ds402 import (
        describe_encoder_retention_fault,
    )

    result = describe_encoder_retention_fault(
        manufacturer_error_code=0,
        error_code=0x0208,
    )
    assert result["present"] is True
    assert "Er20.8" in result["codes"]
    assert "Encoder battery failure" in result["names"]
    assert "error_code_matches_manufacturer_code" in result["matched_sources"]
    # bus_fault_name falls back to the vendor name when the DS402 bus
    # table has no entry for that code - which is the production case
    # for 0x0208 today.
    assert result["bus_fault_name"] == "Encoder battery failure"


def test_describe_encoder_retention_fault_falls_back_to_shared_bus_class_7305():
    """Bus code 0x7305 is the DS402 ``Encoder`` class and is shared by
    every Er20.x / ErA0.1 / ALF9.0 on A6-EC. When the preflight only
    has the bus code (0x203F not PDO-mapped and no specific
    ``fault_code_203f`` numeric match), the retention classifier must
    still fire so the affected axis ends up on the F31.10 recovery
    path instead of the useless DS402-pulse-only path.
    """
    from gradient_os.arm_controller.profiles.drive.a6ec_ds402 import (
        describe_encoder_retention_fault,
    )

    result = describe_encoder_retention_fault(
        manufacturer_error_code=0,
        error_code=0x7305,
    )
    assert result["present"] is True
    # First match in codebook iteration order is Er20.1 (encoder-family
    # retention). The specific subcode is ambiguous without 0x203F but
    # the retention verdict is correct.
    assert result["codes"] == ["Er20.1"]
    assert "error_code_bus_class_retention_match" in result["matched_sources"]
    assert "manufacturer_error_code" not in result["matched_sources"]
    assert "error_code_matches_manufacturer_code" not in result["matched_sources"]


def test_describe_encoder_retention_fault_prefers_manufacturer_side_when_available():
    """When 0x203F IS populated (the vendor-standard path), the
    retention match source must still be reported as
    ``manufacturer_error_code`` so operators can tell the two cases
    apart in logs / monitor output.
    """
    from gradient_os.arm_controller.profiles.drive.a6ec_ds402 import (
        describe_encoder_retention_fault,
    )

    result = describe_encoder_retention_fault(
        manufacturer_error_code=0x00000208,
        error_code=0x7305,
    )
    assert result["present"] is True
    assert result["codes"] == ["Er20.8"]
    assert "manufacturer_error_code" in result["matched_sources"]
    assert "error_code_matches_manufacturer_code" not in result["matched_sources"]


def test_build_drive_fault_snapshot_carries_encoder_retention_fault(monkeypatch):
    # When the live manufacturer_error_code matches the A6-EC
    # encoder-retention family (e.g. Er20.9 = 0x209, "Encoder multi-turn
    # error"), the per-axis payload in build_drive_fault_snapshot must
    # carry an encoder_retention_fault dict with the matched vendor code
    # and name, and the truth-validity reason must upgrade to
    # encoder_retention_fault_present ahead of the generic
    # manufacturer_fault_present branch.
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
        lambda *args, **kwargs: {"state": "Fault"},
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
    # Leave describe_drive_encoder_retention_fault_for_backend unpatched
    # so the real a6ec_ds402 decoder runs; that's exactly the production
    # path we want to cover.

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
                    "manufacturer_error_code": 0x209,
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
        axis_to_joint=[5],
        socket_present=True,
    )

    axis = snapshot["axes"][0]
    assert axis["encoder_retention_fault_present"] is True
    retention_detail = axis["encoder_retention_fault"]
    assert isinstance(retention_detail, dict)
    assert "Er20.9" in retention_detail.get("codes", [])
    assert "Encoder multi-turn error" in retention_detail.get("names", [])
    assert axis["drive_native_truth_reason"] == "encoder_retention_fault_present"
    assert axis["coordinate_system_valid"] is False
    assert axis["drive_native_truth_valid"] is False
