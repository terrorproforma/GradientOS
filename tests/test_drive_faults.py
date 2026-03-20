from gradient_os.telemetry.drive_faults import build_startup_fault_reset_plan


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
