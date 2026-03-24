from gradient_os import run_controller


def test_attach_drive_faults_to_telemetry_message_uses_servo_backend(monkeypatch):
    sync_calls: list[object] = []
    build_calls: list[dict[str, object]] = []

    monkeypatch.setattr(
        run_controller,
        "_sync_live_rtcore_drive_profile",
        lambda active_runtime_config, backend_instance: sync_calls.append(
            (active_runtime_config, backend_instance)
        ),
    )

    def fake_build(
        servo_backend: str,
        *,
        drive_profile=None,
        configured_drive_profile=None,
        live_drive_profile=None,
        fieldbus_profile=None,
        backend_instance=None,
    ):
        build_calls.append(
            {
                "servo_backend": servo_backend,
                "drive_profile": drive_profile,
                "configured_drive_profile": configured_drive_profile,
                "live_drive_profile": live_drive_profile,
                "backend_instance": backend_instance,
            }
        )
        return {"driver_state": "ACTIVE"}

    monkeypatch.setattr(run_controller, "_build_drive_fault_snapshot", fake_build)

    msg: dict[str, object] = {}
    active_runtime_config = {
        "drive_profile": {
            "effective_profile": "a6ec_ds402",
            "configured_profile": "a6ec_ds402",
            "live_profile": "a6ec_ds402",
        }
    }
    backend = object()

    run_controller._attach_drive_faults_to_telemetry_message(
        msg,
        active_runtime_config=active_runtime_config,
        servo_backend="ethercat_rtcore",
        backend_instance=backend,
    )

    assert sync_calls == [(active_runtime_config, backend)]
    assert build_calls == [
        {
            "servo_backend": "ethercat_rtcore",
            "drive_profile": "a6ec_ds402",
            "configured_drive_profile": "a6ec_ds402",
            "live_drive_profile": "a6ec_ds402",
            "backend_instance": backend,
        }
    ]
    assert msg["drive_faults"] == {"driver_state": "ACTIVE"}


def test_build_monitor_motion_status_payload_returns_controller_snapshot(monkeypatch):
    payload = {
        "accepted": True,
        "state": "executing",
        "trajectory_id": 9,
        "execution": {"state_name": "queued"},
    }
    monkeypatch.setattr(
        run_controller.command_api,
        "get_motion_execution_status",
        lambda: payload,
    )

    result = run_controller._build_monitor_motion_status_payload()

    assert result == payload


def test_build_monitor_motion_status_payload_fails_closed(monkeypatch):
    monkeypatch.setattr(
        run_controller.command_api,
        "get_motion_execution_status",
        lambda: "not-a-dict",
    )
    assert run_controller._build_monitor_motion_status_payload() is None

    def _boom():
        raise RuntimeError("monitor unavailable")

    monkeypatch.setattr(run_controller.command_api, "get_motion_execution_status", _boom)
    assert run_controller._build_monitor_motion_status_payload() is None
