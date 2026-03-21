from gradient_os.arm_controller.backends.ethercat_rtcore.runtime import (
    DEFAULT_RT_MAX_RPM,
    RTCORE_EXEC_STATE_COMPLETED,
    RTCORE_MOTION_MODE_LEGACY_SETPOINT,
    RTCORE_DRIVE_PROFILE_A6EC_DS402,
    build_rtcore_axis_scaling,
    render_rtcore_systemd_env,
    rtcore_drive_profile_id_to_name,
    rtcore_execution_state_id_to_name,
    rtcore_motion_mode_id_to_name,
    rtcore_drive_profile_name_to_id,
)
from gradient_os.arm_controller.robots import get_robot_config
from gradient_os.telemetry.drive_faults import build_drive_fault_snapshot


def test_rtcore_drive_profile_ids_round_trip():
    assert rtcore_drive_profile_name_to_id("a6ec_ds402") == RTCORE_DRIVE_PROFILE_A6EC_DS402
    assert rtcore_drive_profile_id_to_name(RTCORE_DRIVE_PROFILE_A6EC_DS402) == "a6ec_ds402"
    assert rtcore_drive_profile_id_to_name(0) is None


def test_rtcore_motion_state_name_maps_round_trip():
    assert rtcore_motion_mode_id_to_name(RTCORE_MOTION_MODE_LEGACY_SETPOINT) == "legacy_setpoint"
    assert rtcore_execution_state_id_to_name(RTCORE_EXEC_STATE_COMPLETED) == "completed"
    assert rtcore_motion_mode_id_to_name(999) is None
    assert rtcore_execution_state_id_to_name(999) is None


def test_build_rtcore_axis_scaling_uses_robot_config_values():
    robot = get_robot_config("gradient05")
    robot_cfg = robot.get_config_dict()
    scaling = build_rtcore_axis_scaling(robot_cfg)
    assert scaling["num_axes"] == 6
    assert scaling["counts_per_rev"] == [131072] * 6
    assert scaling["gear_ratio"] == [float(value) for value in robot_cfg["actuator_gear_ratios"][:6]]
    assert len(scaling["sign"]) == 6


def test_render_rtcore_systemd_env_contains_scaling_and_profile():
    robot = get_robot_config("gradient05")
    robot_cfg = robot.get_config_dict()
    expected_gear_ratio = ",".join(f"{float(value):g}" for value in robot_cfg["actuator_gear_ratios"][:6])
    rendered = render_rtcore_systemd_env(
        robot_config=robot_cfg,
        drive_profile="a6ec_ds402",
    )
    assert 'GRADIENT_RT_NUM_AXES="6"' in rendered
    assert 'GRADIENT_RT_COUNTS_PER_REV="131072,131072,131072,131072,131072,131072"' in rendered
    assert f'GRADIENT_RT_GEAR_RATIO="{expected_gear_ratio}"' in rendered
    assert 'GRADIENT_RT_DRIVE_PROFILE="a6ec_ds402"' in rendered
    assert f'GRADIENT_RT_MAX_RPM="{DEFAULT_RT_MAX_RPM:g}"' in rendered


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
                {"statusword": 0x1618, "error_code": 0x8700, "slave_online": 1, "slave_operational": 1, "slave_al_state": 8, "pos_counts": 0},
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
    assert snapshot["axes"][2]["fault"]["error_code_hex"] == "0x8700"
    assert snapshot["axes"][2]["fault"]["bus_fault_name"] == "Synchronization controller"
    assert "sync" in snapshot["axes"][2]["fault"]["name"].lower()
