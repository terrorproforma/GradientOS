from gradient_os.telemetry.startup_recovery import build_rtcore_startup_recovery_plan


def test_startup_recovery_plan_is_healthy_when_bus_is_ready():
    plan = build_rtcore_startup_recovery_plan(
        {
            "physical_state": "BUS_UP_DISARMED",
            "driver_state": "DISARMED",
            "ethercat_master_state": "OP",
            "rtcore_state": "UP",
            "startup_ready": 1,
            "responding": 6,
            "online": 6,
            "operational": 6,
            "num_axes": 6,
        }
    )
    assert plan["healthy"] is True
    assert plan["should_recover"] is False
    assert plan["reboot_required"] is False


def test_startup_recovery_plan_requests_one_recycle_when_rtcore_is_up_but_master_is_down():
    plan = build_rtcore_startup_recovery_plan(
        {
            "physical_state": "INACTIVE",
            "driver_state": "INACTIVE",
            "ethercat_master_state": "DOWN",
            "rtcore_state": "UP",
            "startup_ready": 0,
            "responding": 0,
            "online": 0,
            "operational": 0,
            "num_axes": 6,
        }
    )
    assert plan["healthy"] is False
    assert plan["should_recover"] is True
    assert plan["reboot_required"] is False
    assert plan["reason"] == "rtcore_up_master_down"


def test_startup_recovery_plan_escalates_to_reboot_after_busy_master_survives_recovery():
    plan = build_rtcore_startup_recovery_plan(
        {
            "physical_state": "INACTIVE",
            "driver_state": "INACTIVE",
            "ethercat_master_state": "DOWN",
            "rtcore_state": "UP",
            "startup_ready": 0,
            "responding": 0,
            "online": 0,
            "operational": 0,
            "num_axes": 6,
        },
        recent_log="""
        gradient-rt-motion.service: Found left-over process 1766 (gradient-rt-mot) in control group while starting unit. Ignoring.
        Failed to reserve master: Device or resource busy
        [gradient-rt-motion] ERROR: ecrt_request_master(0) failed
        """,
        recovery_attempted=True,
    )
    assert plan["should_recover"] is False
    assert plan["reboot_required"] is True
    assert "master_device_busy" in plan["journal_signatures"]
    assert "leftover_process" in plan["journal_signatures"]


def test_startup_recovery_plan_does_not_recycle_normal_bus_convergence():
    plan = build_rtcore_startup_recovery_plan(
        {
            "physical_state": "BUS_UP_DISARMED",
            "driver_state": "DISARMED",
            "ethercat_master_state": "PREOP",
            "rtcore_state": "UP",
            "startup_ready": 0,
            "responding": 6,
            "online": 6,
            "operational": 0,
            "num_axes": 6,
        }
    )
    assert plan["healthy"] is False
    assert plan["should_recover"] is False
    assert plan["reboot_required"] is False
