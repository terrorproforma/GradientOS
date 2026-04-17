import pytest

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


def test_attach_monitor_joint_feedback_uses_display_snapshot_when_available():
    msg: dict[str, object] = {}

    run_controller._attach_monitor_joint_feedback(
        msg,
        canonical_joint_positions_rad=[-628.0, -113.0, -62.0],
        display_snapshot={
            "truth_available": True,
            "joint_positions_rad": [0.01, 0.02, 0.03],
            "axis_absolute_feedback": [{"logical_joint": 1, "truth_available": True}],
        },
    )

    assert msg["joints"] == [-628.0, -113.0, -62.0]
    assert msg["display_joints"] == [0.01, 0.02, 0.03]
    assert msg["axis_absolute_feedback"] == [{"logical_joint": 1, "truth_available": True}]


def test_attach_monitor_joint_feedback_omits_display_when_truth_unavailable():
    msg: dict[str, object] = {}

    run_controller._attach_monitor_joint_feedback(
        msg,
        canonical_joint_positions_rad=[-628.0, -113.0, -62.0],
        display_snapshot={
            "truth_available": False,
            "joint_positions_rad": [0.01, 0.02, 0.03],
            "joint_positions_rad_partial": [0.01, None, 0.03],
            "axis_absolute_feedback": [{"logical_joint": 1, "truth_available": False}],
        },
    )

    assert msg["joints"] == [-628.0, -113.0, -62.0]
    assert "display_joints" not in msg
    assert msg["axis_absolute_feedback"] == [{"logical_joint": 1, "truth_available": False}]


def test_build_joint_state_snapshot_uses_backend_display_feedback(monkeypatch):
    display_positions = [0.11, 0.21, 0.31, 0.41, 0.51, 0.61]
    canonical_positions = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6]
    backend = object()

    monkeypatch.setattr(
        run_controller.servo_driver,
        "get_current_arm_state_rad",
        lambda verbose=False: canonical_positions,
    )
    monkeypatch.setattr(run_controller.backend_registry, "get_active_backend", lambda: backend)
    monkeypatch.setattr(run_controller.backend_registry, "get_active_backend_name", lambda: "ethercat_rtcore")
    monkeypatch.setattr(run_controller, "_sync_backend_raw_positions", lambda _backend: {0: 0})
    monkeypatch.setattr(
        run_controller,
        "_backend_display_feedback_snapshot",
        lambda _backend, _raw_positions: {
            "truth_available": True,
            "joint_positions_rad": display_positions,
            "truth_unavailable_axes": [],
            "truth_unavailable_joints": [],
            "axis_absolute_feedback": [],
        },
    )
    monkeypatch.setattr(run_controller.utils, "gripper_present", False, raising=False)

    snapshot = run_controller._build_joint_state_snapshot()

    assert snapshot["arm_rad"] == canonical_positions
    assert snapshot["arm_display_rad"] == display_positions
    assert snapshot["arm_display_deg"] == pytest.approx(
        [float((value * 180.0) / 3.141592653589793) for value in display_positions]
    )


def test_build_joint_state_snapshot_does_not_fallback_display_feedback(monkeypatch):
    canonical_positions = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6]
    backend = object()

    monkeypatch.setattr(
        run_controller.servo_driver,
        "get_current_arm_state_rad",
        lambda verbose=False: canonical_positions,
    )
    monkeypatch.setattr(run_controller.backend_registry, "get_active_backend", lambda: backend)
    monkeypatch.setattr(run_controller.backend_registry, "get_active_backend_name", lambda: "ethercat_rtcore")
    monkeypatch.setattr(run_controller, "_sync_backend_raw_positions", lambda _backend: {0: 0})
    monkeypatch.setattr(
        run_controller,
        "_backend_display_feedback_snapshot",
        lambda _backend, _raw_positions: {
            "truth_available": False,
            "joint_positions_rad": [0.11, 0.21, 0.31, 0.41, 0.51, 0.61],
            "truth_unavailable_axes": [0],
            "truth_unavailable_joints": [1],
            "axis_absolute_feedback": [
                {
                    "axis": 0,
                    "logical_joint": 1,
                    "truth_available": False,
                    "truth_reason": "command_frame_roundtrip_mismatch",
                }
            ],
        },
    )
    monkeypatch.setattr(run_controller.utils, "gripper_present", False, raising=False)

    snapshot = run_controller._build_joint_state_snapshot()

    assert snapshot["arm_rad"] == canonical_positions
    assert snapshot["arm_display_rad"] == []
    assert snapshot["arm_display_deg"] == []
    assert snapshot["raw_canonical_joint_truth_available"] is True
    assert snapshot["display_joint_truth_available"] is False
    assert snapshot["canonical_joint_truth_available"] is True


def test_build_joint_state_snapshot_preserves_partial_display_feedback(monkeypatch):
    canonical_positions = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6]
    partial_display_positions = [0.11, None, 0.31, None, 0.51, 0.61]
    backend = object()

    monkeypatch.setattr(
        run_controller.servo_driver,
        "get_current_arm_state_rad",
        lambda verbose=False: canonical_positions,
    )
    monkeypatch.setattr(run_controller.backend_registry, "get_active_backend", lambda: backend)
    monkeypatch.setattr(run_controller.backend_registry, "get_active_backend_name", lambda: "ethercat_rtcore")
    monkeypatch.setattr(run_controller, "_sync_backend_raw_positions", lambda _backend: {0: 0})
    monkeypatch.setattr(
        run_controller,
        "_backend_display_feedback_snapshot",
        lambda _backend, _raw_positions: {
            "truth_available": False,
            "joint_positions_rad": [9.0, 9.0, 9.0, 9.0, 9.0, 9.0],
            "joint_positions_rad_partial": partial_display_positions,
            "truth_unavailable_axes": [1, 3],
            "truth_unavailable_joints": [2, 4],
            "axis_absolute_feedback": [
                {
                    "axis": 0,
                    "logical_joint": 1,
                    "truth_available": True,
                    "canonical_rad": 0.11,
                },
                {
                    "axis": 1,
                    "logical_joint": 2,
                    "truth_available": False,
                    "truth_reason": "absolute_home_anchor_stale",
                },
                {
                    "axis": 2,
                    "logical_joint": 3,
                    "truth_available": True,
                    "canonical_rad": 0.31,
                },
                {
                    "axis": 3,
                    "logical_joint": 4,
                    "truth_available": False,
                    "truth_reason": "command_frame_roundtrip_mismatch",
                },
                {
                    "axis": 4,
                    "logical_joint": 5,
                    "truth_available": True,
                    "canonical_rad": 0.51,
                },
                {
                    "axis": 5,
                    "logical_joint": 6,
                    "truth_available": True,
                    "canonical_rad": 0.61,
                },
            ],
        },
    )
    monkeypatch.setattr(run_controller.utils, "gripper_present", False, raising=False)

    snapshot = run_controller._build_joint_state_snapshot()

    assert snapshot["arm_rad"] == canonical_positions
    assert snapshot["arm_display_rad"] == partial_display_positions
    display_deg = snapshot["arm_display_deg"]
    assert isinstance(display_deg, list)
    assert display_deg[0] == pytest.approx(float((0.11 * 180.0) / 3.141592653589793))
    assert display_deg[1] is None
    assert display_deg[2] == pytest.approx(float((0.31 * 180.0) / 3.141592653589793))
    assert display_deg[3] is None
    assert display_deg[4] == pytest.approx(float((0.51 * 180.0) / 3.141592653589793))
    assert display_deg[5] == pytest.approx(float((0.61 * 180.0) / 3.141592653589793))
    assert snapshot["raw_canonical_joint_truth_available"] is True
    assert snapshot["display_joint_truth_available"] is False
    assert snapshot["display_joint_truth_unavailable_axes"] == [1, 3]
    assert snapshot["display_joint_truth_unavailable_joints"] == [2, 4]
    assert snapshot["canonical_joint_truth_available"] is True


def test_build_joint_state_snapshot_keeps_raw_blocker_details_when_display_truth_is_available(monkeypatch):
    display_positions = [0.11, 0.21, 0.31, 0.41, 0.51, 0.61]
    backend = object()

    def _boom(verbose=False):
        raise RuntimeError(
            "Canonical joint truth unavailable (axes=[2, 3, 5], joints=[3, 4, 6])"
        )

    monkeypatch.setattr(
        run_controller.servo_driver,
        "get_current_arm_state_rad",
        _boom,
    )
    monkeypatch.setattr(run_controller.backend_registry, "get_active_backend", lambda: backend)
    monkeypatch.setattr(run_controller.backend_registry, "get_active_backend_name", lambda: "ethercat_rtcore")
    monkeypatch.setattr(run_controller, "_sync_backend_raw_positions", lambda _backend: {0: 0})
    monkeypatch.setattr(
        run_controller,
        "_backend_display_feedback_snapshot",
        lambda _backend, _raw_positions: {
            "truth_available": True,
            "joint_positions_rad": display_positions,
            "truth_unavailable_axes": [],
            "truth_unavailable_joints": [],
            "axis_absolute_feedback": [],
        },
    )
    monkeypatch.setattr(run_controller.utils, "gripper_present", False, raising=False)

    snapshot = run_controller._build_joint_state_snapshot()

    assert snapshot["read_source"] == "unavailable"
    assert snapshot["raw_canonical_joint_truth_available"] is False
    assert snapshot["display_joint_truth_available"] is True
    assert snapshot["canonical_joint_truth_available"] is False
    assert snapshot["canonical_joint_truth_unavailable_axes"] == [2, 3, 5]
    assert snapshot["canonical_joint_truth_unavailable_joints"] == [3, 4, 6]
    assert snapshot["display_joint_truth_unavailable_axes"] == []
    assert snapshot["display_joint_truth_unavailable_joints"] == []
    assert snapshot["arm_display_rad"] == display_positions
