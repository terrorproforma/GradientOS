from gradient_os.telemetry.encoder_retention import compare_retention_snapshots


def test_compare_retention_snapshots_reports_mismatch_faults_and_mode_verification():
    comparison = compare_retention_snapshots(
        before_snapshot={
            "captured_at": "2026-04-05T00:00:00+00:00",
            "joint_state": {
                "axis_counts": [100, 200, 300],
                "arm_rad": [0.1, 0.2, 0.3],
            },
        },
        after_snapshot={
            "captured_at": "2026-04-05T00:10:00+00:00",
            "joint_state": {
                "axis_counts": [100, 201, 300],
                "arm_rad": [0.1, 0.21, 0.3],
            },
            "drive_faults": {
                "axes": [
                    {
                        "axis": 1,
                        "logical_joint": 2,
                        "startup_drive_config": {
                            "setting_key": "a6ec_encoder_position_tracking_mode",
                            "setting_label": "A6-EC encoder position tracking mode",
                            "configured": True,
                            "commanded": 1,
                            "commanded_value_label": "Absolute position linear mode",
                            "readback_valid": True,
                            "readback": 0,
                            "readback_value_label": "Incremental encoder mode",
                            "verified": False,
                        },
                        "manufacturer_error_code": 0x208,
                        "manufacturer_error_code_hex": "0x00000208",
                        "manufacturer_fault": {
                            "code": "Er20.8",
                            "name": "Encoder battery failure",
                        },
                    }
                ]
            },
        },
    )

    assert comparison["before_timestamp"] == "2026-04-05T00:00:00+00:00"
    assert comparison["after_timestamp"] == "2026-04-05T00:10:00+00:00"
    assert comparison["raw_encoder_mismatch"] is True
    assert comparison["logical_angle_mismatch"] is True
    assert comparison["startup_drive_config_mismatch"] is True
    assert comparison["axis_counts"][1]["delta"] == 1
    assert comparison["logical_joints_rad"][1]["joint"] == 2
    assert comparison["startup_drive_config_mismatch_axes"] == [
        {
            "axis": 1,
            "logical_joint": 2,
            "setting_key": "a6ec_encoder_position_tracking_mode",
            "setting_label": "A6-EC encoder position tracking mode",
            "commanded": 1,
            "commanded_value_label": "Absolute position linear mode",
            "readback": 0,
            "readback_value_label": "Incremental encoder mode",
            "readback_valid": True,
        }
    ]
    assert comparison["active_battery_or_multiturn_faults"] == [
        {
            "axis": 1,
            "logical_joint": 2,
            "manufacturer_error_code": 0x208,
            "manufacturer_error_code_hex": "0x00000208",
            "code": "Er20.8",
            "name": "Encoder battery failure",
        }
    ]
