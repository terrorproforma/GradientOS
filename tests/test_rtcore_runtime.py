import pytest
from fractions import Fraction

from gradient_os.arm_controller.backends.ethercat_rtcore.runtime import (
    DEFAULT_RT_MAX_RPM,
    RTCORE_EXEC_STATE_COMPLETED,
    RTCORE_MOTION_MODE_LEGACY_SETPOINT,
    build_rtcore_drive_startup_config,
    build_rtcore_axis_scaling,
    render_rtcore_systemd_env,
    rtcore_drive_profile_id_to_name,
    rtcore_execution_state_id_to_name,
    rtcore_motion_mode_id_to_name,
    rtcore_drive_profile_name_to_id,
)
from gradient_os.arm_controller.robots import get_robot_config
from gradient_os.telemetry.drive_faults import build_drive_fault_snapshot


def _expected_a6ec_ratio_pairs(robot_cfg: dict[str, object]) -> tuple[list[int], list[int]]:
    numerators: list[int] = []
    denominators: list[int] = []
    for raw_value in list(robot_cfg.get("actuator_gear_ratios", []))[:6]:
        ratio = Fraction(str(raw_value))
        numerators.append(int(ratio.numerator))
        denominators.append(int(ratio.denominator))
    return numerators, denominators


def _a6ec_startup_drive_configs_for_ratio(
    raw_ratio: object,
    *,
    mode_value: int = 4,
) -> list[dict[str, int | str]]:
    ratio = Fraction(str(raw_ratio))
    numerator = int(ratio.numerator)
    denominator = int(ratio.denominator)
    return [
        {
            "setting_key": "a6ec_encoder_position_tracking_mode",
            "configured": 1,
            "commanded": mode_value,
            "readback_valid": 1,
            "readback": mode_value,
            "verified": 1,
        },
        {
            "setting_key": "a6ec_rotation_mode_gear_ratio_numerator",
            "configured": 1,
            "commanded": numerator,
            "readback_valid": 1,
            "readback": numerator,
            "verified": 1,
        },
        {
            "setting_key": "a6ec_rotation_mode_gear_ratio_denominator",
            "configured": 1,
            "commanded": denominator,
            "readback_valid": 1,
            "readback": denominator,
            "verified": 1,
        },
    ]


def test_rtcore_drive_profile_ids_round_trip():
    profile_code = rtcore_drive_profile_name_to_id("a6ec_ds402")
    assert profile_code != 0
    assert rtcore_drive_profile_id_to_name(profile_code) == "a6ec_ds402"
    assert rtcore_drive_profile_id_to_name(0) is None


def test_rtcore_motion_state_name_maps_round_trip():
    assert rtcore_motion_mode_id_to_name(RTCORE_MOTION_MODE_LEGACY_SETPOINT) == "legacy_setpoint"
    assert rtcore_execution_state_id_to_name(RTCORE_EXEC_STATE_COMPLETED) == "completed"
    assert rtcore_motion_mode_id_to_name(999) is None
    assert rtcore_execution_state_id_to_name(999) is None


def test_build_rtcore_axis_scaling_uses_drive_native_ratio_for_a6ec():
    robot = get_robot_config("gradient05")
    robot_cfg = robot.get_config_dict()
    scaling = build_rtcore_axis_scaling(robot_cfg, drive_profile="a6ec_ds402")
    assert scaling["num_axes"] == 6
    assert scaling["counts_per_rev"] == [131072] * 6
    assert scaling["gear_ratio"] == [float(value) for value in robot_cfg["actuator_gear_ratios"][:6]]
    assert scaling["drive_native_ratio_enabled"] is True
    assert len(scaling["sign"]) == 6


def test_build_rtcore_axis_scaling_keeps_robot_gear_ratio_for_non_native_profile():
    robot = get_robot_config("gradient05")
    robot_cfg = robot.get_config_dict()
    scaling = build_rtcore_axis_scaling(robot_cfg, drive_profile="cia402")
    assert scaling["num_axes"] == 6
    assert scaling["counts_per_rev"] == [131072] * 6
    assert scaling["gear_ratio"] == [float(value) for value in robot_cfg["actuator_gear_ratios"][:6]]
    assert scaling["drive_native_ratio_enabled"] is False
    assert len(scaling["sign"]) == 6


def test_render_rtcore_systemd_env_contains_scaling_and_profile():
    robot = get_robot_config("gradient05")
    robot_cfg = robot.get_config_dict()
    expected_gear_ratio = ",".join(f"{float(value):g}" for value in robot_cfg["actuator_gear_ratios"][:6])
    expected_ratio_numerators, expected_ratio_denominators = _expected_a6ec_ratio_pairs(robot_cfg)
    expected_startup_sdo = (
        "a6ec_encoder_position_tracking_mode|u16|0x2000|0x08|4,4,4,4,4,4;"
        "a6ec_rotation_mode_gear_ratio_numerator|u16|0x2010|0x19|"
        + ",".join(str(value) for value in expected_ratio_numerators)
        + ";a6ec_rotation_mode_gear_ratio_denominator|u16|0x2010|0x1A|"
        + ",".join(str(value) for value in expected_ratio_denominators)
    )
    expected_native_home = (
        "steady_state_mode|8;commissioning_mode|6;truth_source|0x607C|0x00|i32;"
        "op|set_mode|6;op|write_sdo|0x60E6|0x00|u8|0;op|write_sdo|0x607C|0x00|i32|0;"
        "op|write_sdo|0x6098|0x00|i8|35;op|controlword_sequence|6,7,15;"
        "op|wait_statusword|0x0227|0x2048;op|controlword_sequence|31;"
        "op|wait_statusword|0x9000|0x2000;op|refresh_truth;op|restore_mode|8;"
        "op|release_service_override;"
        "op|write_sdo|0x2031|0x11|u16|1;op|wait_sdo|0x2031|0x11|u16|0;"
        "op|write_sdo|0x2031|0x11|u16|2;op|wait_sdo|0x2031|0x11|u16|0"
    )
    expected_absolute_feedback = (
        "absolute_position_reference|0x2040|0x17|i32;"
        "encoder_single_turn_data|0x2040|0x1D|i32;"
        "encoder_multi_turn_position|0x2040|0x1F|u16;"
        "encoder_multi_turn_low|0x2040|0x21|i32;"
        "encoder_multi_turn_high|0x2040|0x23|i32;"
        "rotation_mode_position_reference|0x2040|0x29|i32;"
        "rotation_mode_encoder_low|0x2040|0x2B|i32;"
        "rotation_mode_encoder_high|0x2040|0x2D|i32"
    )
    rendered = render_rtcore_systemd_env(
        robot_config=robot_cfg,
        drive_profile="a6ec_ds402",
    )
    assert 'GRADIENT_RT_NUM_AXES="6"' in rendered
    assert 'GRADIENT_RT_COUNTS_PER_REV="131072,131072,131072,131072,131072,131072"' in rendered
    assert f'GRADIENT_RT_GEAR_RATIO="{expected_gear_ratio}"' in rendered
    assert 'GRADIENT_RT_DRIVE_PROFILE="a6ec_ds402"' in rendered
    assert f'GRADIENT_RT_MAX_RPM="{DEFAULT_RT_MAX_RPM:g}"' in rendered
    assert 'GRADIENT_RT_DRIVE_VENDOR_ID="0x00400000"' in rendered
    assert 'GRADIENT_RT_DRIVE_PRODUCT_CODE="0x00000715"' in rendered
    assert 'GRADIENT_RT_DRIVE_RX_SYNC_INDEX="2"' in rendered
    assert 'GRADIENT_RT_DRIVE_TX_SYNC_INDEX="3"' in rendered
    assert 'GRADIENT_RT_DRIVE_DC_CYCLE_MULTIPLE_NS="250000"' in rendered
    assert 'GRADIENT_RT_DRIVE_RX_PDO="0x1702"' in rendered
    assert 'GRADIENT_RT_DRIVE_TX_PDO="0x1B02"' in rendered
    assert 'GRADIENT_RT_DRIVE_RX_PDO_LAYOUT="cw|0x6040|0x00|16;target_pos|0x607A|0x00|32;' in rendered
    assert 'GRADIENT_RT_DRIVE_TX_PDO_LAYOUT="err|0x603F|0x00|16;sw|0x6041|0x00|16;pos|0x6064|0x00|32;' in rendered
    assert f'GRADIENT_RT_DRIVE_STARTUP_SDO_CONFIG="{expected_startup_sdo}"' in rendered
    assert f'GRADIENT_RT_ABSOLUTE_FEEDBACK_CONFIG="{expected_absolute_feedback}"' in rendered
    assert f'GRADIENT_RT_NATIVE_HOME_CONFIG="{expected_native_home}"' in rendered
    assert 'GRADIENT_RT_FEEDBACK_WRAP_AXIS_MASK="0x3f"' in rendered


def test_build_rtcore_drive_startup_config_uses_drive_profile_defaults_when_robot_has_no_override():
    robot = get_robot_config("gradient05")
    assert robot.get_config_dict()["ethercat_drive_startup_config"] == []
    expected_ratio_numerators, expected_ratio_denominators = _expected_a6ec_ratio_pairs(
        robot.get_config_dict()
    )
    startup = build_rtcore_drive_startup_config(
        robot.get_config_dict(),
        drive_profile="a6ec_ds402",
    )
    assert startup["settings"] == {
        "a6ec_encoder_position_tracking_mode": [4, 4, 4, 4, 4, 4],
        "a6ec_rotation_mode_gear_ratio_numerator": expected_ratio_numerators,
        "a6ec_rotation_mode_gear_ratio_denominator": expected_ratio_denominators,
    }


def test_build_rtcore_drive_startup_config_uses_drive_profile_module():
    robot = get_robot_config("gradient05")
    expected_ratio_numerators, expected_ratio_denominators = _expected_a6ec_ratio_pairs(
        robot.get_config_dict()
    )
    startup = build_rtcore_drive_startup_config(
        robot.get_config_dict(),
        drive_profile="a6ec_ds402",
    )
    assert startup["profile_id"] == "a6ec_ds402"
    assert startup["settings"] == {
        "a6ec_encoder_position_tracking_mode": [4, 4, 4, 4, 4, 4],
        "a6ec_rotation_mode_gear_ratio_numerator": expected_ratio_numerators,
        "a6ec_rotation_mode_gear_ratio_denominator": expected_ratio_denominators,
    }
    assert startup["env"]["GRADIENT_RT_DRIVE_STARTUP_SDO_CONFIG"] == (
        "a6ec_encoder_position_tracking_mode|u16|0x2000|0x08|4,4,4,4,4,4;"
        "a6ec_rotation_mode_gear_ratio_numerator|u16|0x2010|0x19|"
        + ",".join(str(value) for value in expected_ratio_numerators)
        + ";a6ec_rotation_mode_gear_ratio_denominator|u16|0x2010|0x1A|"
        + ",".join(str(value) for value in expected_ratio_denominators)
    )


def test_render_rtcore_systemd_env_rejects_drive_profile_without_ethercat_catalog_entry():
    robot = get_robot_config("gradient05")
    with pytest.raises(ValueError, match="does not define an EtherCAT drive catalog entry"):
        render_rtcore_systemd_env(
            robot_config=robot.get_config_dict(),
            drive_profile="cia402",
        )


def test_drive_fault_snapshot_decodes_axis_fault_and_master_state():
    snapshot = build_drive_fault_snapshot(
        metrics={
            "num_axes": 6,
            "armed": 0,
            "axis_enable_mask": 0,
            "link_up": 1,
            "responding_slaves": 6,
            "online_slaves": 6,
            "operational_slaves": 6,
            "startup_ready": 1,
            "wkc_actual": 18,
            "wkc_expected": 12,
            "master_al_states": 8,
            "axes": [
                {"statusword": 0x1650, "error_code": 0, "slave_online": 1, "slave_operational": 1, "slave_al_state": 8, "pos_counts": 0},
                {"statusword": 0x1650, "error_code": 0, "slave_online": 1, "slave_operational": 1, "slave_al_state": 8, "pos_counts": 0},
                {
                    "statusword": 0x1618,
                    "error_code": 0x7305,
                    "manufacturer_error_code": 0x208,
                    "startup_drive_config": {
                        "setting_key": "a6ec_encoder_position_tracking_mode",
                        "configured": 1,
                        "commanded": 1,
                        "commanded_value_label": "Absolute position linear mode",
                        "readback_valid": 1,
                        "readback": 1,
                        "readback_value_label": "Absolute position linear mode",
                        "verified": 1,
                    },
                    "slave_online": 1,
                    "slave_operational": 1,
                    "slave_al_state": 8,
                    "pos_counts": 0,
                },
                {"statusword": 0x1650, "error_code": 0, "slave_online": 1, "slave_operational": 1, "slave_al_state": 8, "pos_counts": 0},
                {"statusword": 0x1650, "error_code": 0, "slave_online": 1, "slave_operational": 1, "slave_al_state": 8, "pos_counts": 0},
                {"statusword": 0x1650, "error_code": 0, "slave_online": 1, "slave_operational": 1, "slave_al_state": 8, "pos_counts": 0},
            ],
        },
        servo_backend="ethercat_rtcore",
        drive_profile="a6ec_ds402",
        axis_to_joint=[0, 1, 2, 3, 4, 5],
        socket_present=True,
    )
    assert snapshot["ethercat_master_state"] == "OP"
    assert snapshot["axes"][2]["logical_joint"] == 3
    assert snapshot["axes"][2]["fault"]["decoded"] is True
    assert snapshot["axes"][2]["fault"]["error_code_hex"] == "0x7305"
    assert snapshot["axes"][2]["fault"]["bus_fault_name"] == "Encoder error"
    assert "encoder" in snapshot["axes"][2]["fault"]["name"].lower()
    assert snapshot["axes"][2]["manufacturer_error_code"] == 0x208
    assert snapshot["axes"][2]["manufacturer_fault"]["decoded"] is True
    assert snapshot["axes"][2]["manufacturer_fault"]["error_code_hex"] == "0x00000208"
    assert snapshot["axes"][2]["manufacturer_fault"]["code"] == "Er20.8"
    assert snapshot["axes"][2]["manufacturer_fault"]["name"] == "Encoder battery failure"
    assert snapshot["drive_native_ratio_enabled"] is True
    assert snapshot["position_semantics_source"] == "drive_output_shaft"
    assert snapshot["drive_native_startup_valid"] is False
    assert snapshot["drive_native_startup_valid_axes"] == 0
    assert snapshot["drive_native_startup_invalid_axes"] == 6
    assert snapshot["drive_native_truth_valid"] is False
    assert snapshot["axes"][0]["drive_native_startup_valid"] is False
    assert snapshot["axes"][0]["drive_native_startup_reason"] == "startup_drive_config_missing"
    assert snapshot["axes"][0]["drive_native_truth_reason"] == "coordinate_system_invalid"
    assert snapshot["axes"][2]["drive_native_startup_valid"] is False
    assert snapshot["axes"][2]["drive_native_startup_reason"] == "startup_drive_config_missing_required_settings"
    assert snapshot["axes"][2]["drive_native_truth_reason"] == "fault_present"
    assert snapshot["axes"][2]["startup_drive_config"] == {
        "profile_id": "a6ec_ds402",
        "setting_key": "a6ec_encoder_position_tracking_mode",
        "setting_label": "A6-EC encoder position tracking mode",
        "object": "C00.07 / 0x2000:08",
        "configured": False,
        "commanded": 1,
        "commanded_value_label": "Absolute position linear mode",
        "readback_valid": False,
        "readback": 1,
        "readback_value_label": "Absolute position linear mode",
        "verified": False,
        "required_setting_keys": [
            "a6ec_encoder_position_tracking_mode",
            "a6ec_rotation_mode_gear_ratio_numerator",
            "a6ec_rotation_mode_gear_ratio_denominator",
        ],
        "present_setting_keys": ["a6ec_encoder_position_tracking_mode"],
        "missing_setting_keys": [
            "a6ec_rotation_mode_gear_ratio_numerator",
            "a6ec_rotation_mode_gear_ratio_denominator",
        ],
        "unconfigured_setting_keys": [],
        "unverified_setting_keys": [],
        "mismatched_setting_keys": [],
        "settings": {
            "a6ec_encoder_position_tracking_mode": {
                "setting_key": "a6ec_encoder_position_tracking_mode",
                "setting_label": "A6-EC encoder position tracking mode",
                "object": "C00.07 / 0x2000:08",
                "configured": True,
                "commanded": 1,
                "commanded_value_label": "Absolute position linear mode",
                "readback_valid": True,
                "readback": 1,
                "readback_value_label": "Absolute position linear mode",
                "verified": True,
            }
        },
    }
    assert snapshot["startup_drive_config_verified_axes"] == 0
    assert snapshot["startup_drive_config_mismatch_axes"] == 0


def test_drive_fault_snapshot_marks_drive_native_truth_valid_when_startup_and_status_are_valid():
    robot_cfg = get_robot_config("gradient05").get_config_dict()
    snapshot = build_drive_fault_snapshot(
        metrics={
            "num_axes": 1,
            "armed": 0,
            "axis_enable_mask": 0,
            "link_up": 1,
            "responding_slaves": 1,
            "online_slaves": 1,
            "operational_slaves": 1,
            "startup_ready": 1,
            "wkc_actual": 3,
            "wkc_expected": 3,
            "master_al_states": 8,
            "axes": [
                {
                    "statusword": 0x9650,
                    "error_code": 0,
                    "manufacturer_error_code": 0,
                    "startup_drive_config": _a6ec_startup_drive_configs_for_ratio(
                        robot_cfg["actuator_gear_ratios"][0]
                    )[0],
                    "startup_drive_configs": _a6ec_startup_drive_configs_for_ratio(
                        robot_cfg["actuator_gear_ratios"][0]
                    ),
                    "slave_online": 1,
                    "slave_operational": 1,
                    "slave_al_state": 8,
                    "pos_counts": 0,
                }
            ],
        },
        servo_backend="ethercat_rtcore",
        drive_profile="a6ec_ds402",
        axis_to_joint=[0],
        socket_present=True,
    )
    assert snapshot["drive_native_ratio_enabled"] is True
    assert snapshot["drive_native_startup_valid"] is True
    assert snapshot["drive_native_truth_valid"] is True
    assert snapshot["coordinate_system_valid_axes"] == 1
    assert snapshot["axes"][0]["drive_native_startup_valid"] is True
    assert snapshot["axes"][0]["drive_native_truth_valid"] is True
    assert snapshot["axes"][0]["drive_native_truth_reason"] == "valid"


def test_drive_fault_snapshot_accepts_a6ec_bit15_only_restart_truth():
    robot_cfg = get_robot_config("gradient05").get_config_dict()
    snapshot = build_drive_fault_snapshot(
        metrics={
            "num_axes": 1,
            "armed": 0,
            "axis_enable_mask": 0,
            "link_up": 1,
            "responding_slaves": 1,
            "online_slaves": 1,
            "operational_slaves": 1,
            "startup_ready": 1,
            "wkc_actual": 3,
            "wkc_expected": 3,
            "master_al_states": 8,
            "axes": [
                {
                    "statusword": 0x8650,
                    "error_code": 0,
                    "manufacturer_error_code": 0,
                    "startup_drive_config": _a6ec_startup_drive_configs_for_ratio(
                        robot_cfg["actuator_gear_ratios"][0]
                    )[0],
                    "startup_drive_configs": _a6ec_startup_drive_configs_for_ratio(
                        robot_cfg["actuator_gear_ratios"][0]
                    ),
                    "slave_online": 1,
                    "slave_operational": 1,
                    "slave_al_state": 8,
                    "pos_counts": 0,
                }
            ],
        },
        servo_backend="ethercat_rtcore",
        drive_profile="a6ec_ds402",
        axis_to_joint=[0],
        socket_present=True,
    )
    assert snapshot["drive_native_ratio_enabled"] is True
    assert snapshot["drive_native_startup_valid"] is True
    assert snapshot["drive_native_truth_valid"] is True
    assert snapshot["coordinate_system_valid_axes"] == 1
    assert snapshot["axes"][0]["drive_native_truth_valid"] is True
    assert snapshot["axes"][0]["drive_native_truth_signature_valid"] is False
    assert snapshot["axes"][0]["coordinate_system_valid"] is True
    assert snapshot["axes"][0]["drive_native_truth_verification_source"] == "statusword_bit15"
