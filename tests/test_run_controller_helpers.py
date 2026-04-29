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


def _reset_canonical_truth_monitor_state() -> None:
    with run_controller._CANONICAL_TRUTH_MONITOR_LOCK:
        run_controller._CANONICAL_TRUTH_MONITOR_STATE.update(
            {
                "available": None,
                "reason": None,
                "axes": (),
                "joints": (),
                "read_source": None,
            }
        )
        run_controller._CANONICAL_TRUTH_ADVISORY_PENDING.update(
            {
                "state": None,
                "count": 0,
            }
        )


def test_canonical_truth_monitor_debounces_advisory_unavailable_during_jog(monkeypatch, capsys):
    _reset_canonical_truth_monitor_state()
    monkeypatch.setattr(
        run_controller.command_api,
        "handle_get_jog_session_state",
        lambda: {"session_active": True},
    )
    snapshot = {
        "canonical_joint_truth_available": False,
        "canonical_joint_truth_reason": "canonical_truth_unavailable",
        "canonical_joint_truth_unavailable_axes": [0],
        "canonical_joint_truth_unavailable_joints": [1],
        "read_source": "unavailable",
    }

    run_controller._emit_canonical_truth_monitor_transition(snapshot)
    run_controller._emit_canonical_truth_monitor_transition(snapshot)
    assert capsys.readouterr().out == ""

    run_controller._emit_canonical_truth_monitor_transition(snapshot)
    assert "Canonical joint truth monitor: UNAVAILABLE" in capsys.readouterr().out


def test_canonical_truth_monitor_does_not_debounce_hard_unavailable_reason(monkeypatch, capsys):
    _reset_canonical_truth_monitor_state()
    monkeypatch.setattr(
        run_controller.command_api,
        "handle_get_jog_session_state",
        lambda: {"session_active": True},
    )
    snapshot = {
        "canonical_joint_truth_available": False,
        "canonical_joint_truth_reason": "absolute_home_anchor_missing",
        "canonical_joint_truth_unavailable_axes": [0],
        "canonical_joint_truth_unavailable_joints": [1],
        "read_source": "unavailable",
    }

    run_controller._emit_canonical_truth_monitor_transition(snapshot)

    assert "reason=absolute_home_anchor_missing" in capsys.readouterr().out


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


def test_attach_monitor_joint_feedback_can_omit_heavy_axis_feedback():
    msg: dict[str, object] = {}

    run_controller._attach_monitor_joint_feedback(
        msg,
        canonical_joint_positions_rad=[-628.0, -113.0, -62.0],
        display_snapshot={
            "truth_available": True,
            "joint_positions_rad": [0.01, 0.02, 0.03],
            "axis_absolute_feedback": [{"logical_joint": 1, "truth_available": True}],
        },
        include_axis_absolute_feedback=False,
    )

    assert msg["joints"] == [-628.0, -113.0, -62.0]
    assert msg["display_joints"] == [0.01, 0.02, 0.03]
    assert "axis_absolute_feedback" not in msg


def test_read_monitor_joint_feedback_uses_control_feedback(monkeypatch):
    monkeypatch.setattr(
        run_controller.servo_driver,
        "get_control_arm_state_rad",
        lambda verbose=False: [1, 2, 3],
    )
    monkeypatch.setattr(
        run_controller.servo_driver,
        "get_current_arm_state_rad",
        lambda verbose=False: (_ for _ in ()).throw(AssertionError("strict canonical read should not be used")),
    )

    q, available, stale, error, last_good, last_ts = run_controller._read_monitor_joint_feedback(
        None,
        0.0,
        now_s=123.0,
    )

    assert q == [1.0, 2.0, 3.0]
    assert available is True
    assert stale is False
    assert error is None
    assert last_good == [1.0, 2.0, 3.0]
    assert last_ts == 123.0


def test_read_monitor_joint_feedback_falls_back_to_recent_sample(monkeypatch):
    monkeypatch.setattr(
        run_controller.servo_driver,
        "get_control_arm_state_rad",
        lambda verbose=False: (_ for _ in ()).throw(RuntimeError("control unavailable")),
    )

    q, available, stale, error, last_good, last_ts = run_controller._read_monitor_joint_feedback(
        [0.1, 0.2, 0.3],
        10.0,
        now_s=11.5,
    )

    assert q == [0.1, 0.2, 0.3]
    assert available is False
    assert stale is True
    assert error == "control unavailable"
    assert last_good == [0.1, 0.2, 0.3]
    assert last_ts == 10.0


class _FakeExtendedBackend:
    """Minimal backend stub exposing the Phase 1 extended-telemetry
    accessors. Used by the Phase 2 regression tests below to verify
    that ``_build_joint_state_snapshot`` enriches the per-axis entries
    and emits the new top-level arrays."""

    def __init__(self) -> None:
        # Simulate extended telemetry received for axes 0 and 2 only;
        # axes 1, 3, 4, 5 have never been sampled (updated_ns == 0).
        self._axis_extended_updated_ns = [
            1_700_000_000,  # axis 0
            0,              # axis 1
            1_700_000_000,  # axis 2
            0,              # axis 3
            0,              # axis 4
            0,              # axis 5
        ]
        # Raw PDO fields populate both the enrichment path and the
        # top-level array path.
        self._axis_bus_voltage_raw = [4800, 0, 4810, 0, 0, 0]
        self._axis_load_rate_raw = [235, 0, 189, 0, 0, 0]
        self._axis_igbt_temp_raw = [42, 0, 47, 0, 0, 0]
        self._axis_motor_temp_raw = [55, 0, 58, 0, 0, 0]
        self._axis_position_error_counts = [12, 0, -7, 0, 0, 0]
        self._axis_drive_not_ready_bits = [0x02, 0, 0x00, 0, 0, 0]
        self._axis_motor_not_rotating_code = [3, 0, 0, 0, 0, 0]

    def _axis_bus_voltage_v(self, axis_i: int) -> float | None:
        if self._axis_extended_updated_ns[axis_i] == 0:
            return None
        return float(self._axis_bus_voltage_raw[axis_i]) * 0.1

    def _axis_load_rate_pct(self, axis_i: int) -> float | None:
        if self._axis_extended_updated_ns[axis_i] == 0:
            return None
        return float(self._axis_load_rate_raw[axis_i]) * 0.1

    def _axis_igbt_temp_c(self, axis_i: int) -> int | None:
        if self._axis_extended_updated_ns[axis_i] == 0:
            return None
        return int(self._axis_igbt_temp_raw[axis_i])

    def _axis_motor_temp_c(self, axis_i: int) -> int | None:
        if self._axis_extended_updated_ns[axis_i] == 0:
            return None
        return int(self._axis_motor_temp_raw[axis_i])

    def _axis_position_error_counts_or_none(self, axis_i: int) -> int | None:
        if self._axis_extended_updated_ns[axis_i] == 0:
            return None
        return int(self._axis_position_error_counts[axis_i])

    def _axis_drive_not_ready_bits_or_none(self, axis_i: int) -> int | None:
        if self._axis_extended_updated_ns[axis_i] == 0:
            return None
        return int(self._axis_drive_not_ready_bits[axis_i])

    def _axis_motor_not_rotating_code_or_none(self, axis_i: int) -> int | None:
        if self._axis_extended_updated_ns[axis_i] == 0:
            return None
        return int(self._axis_motor_not_rotating_code[axis_i])


def test_build_joint_state_snapshot_enriches_axis_absolute_feedback_with_extended_pdo(monkeypatch):
    """Phase 2 (2026-04-20) — ``_build_joint_state_snapshot`` must fold
    the Phase 1 extended A6-EC 0x2040 PDO diagnostics onto each
    ``axis_absolute_feedback`` entry. Only axes with a non-zero
    ``_axis_extended_updated_ns`` get enriched; the rest stay minimal so
    the UI can tell "drive rejected extended PDO mapping" apart from
    "drive reports 0.0 V"."""
    canonical_positions = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6]
    backend = _FakeExtendedBackend()

    monkeypatch.setattr(
        run_controller.servo_driver,
        "get_current_arm_state_rad",
        lambda verbose=False: canonical_positions,
    )
    monkeypatch.setattr(run_controller.backend_registry, "get_active_backend", lambda: backend)
    monkeypatch.setattr(
        run_controller.backend_registry, "get_active_backend_name", lambda: "ethercat_rtcore"
    )
    monkeypatch.setattr(run_controller, "_sync_backend_raw_positions", lambda _backend: {0: 0})
    monkeypatch.setattr(
        run_controller,
        "_backend_display_feedback_snapshot",
        lambda _backend, _raw_positions: {
            "truth_available": True,
            "joint_positions_rad": canonical_positions,
            "truth_unavailable_axes": [],
            "truth_unavailable_joints": [],
            "axis_absolute_feedback": [
                {"axis": idx, "logical_joint": idx + 1, "truth_available": True}
                for idx in range(6)
            ],
        },
    )
    monkeypatch.setattr(run_controller.utils, "gripper_present", False, raising=False)

    snapshot = run_controller._build_joint_state_snapshot()

    entries = snapshot["axis_absolute_feedback"]
    # Axis 0: fresh sample — every extended key present with A6-EC scaling.
    axis0 = entries[0]
    assert axis0["bus_voltage_v"] == pytest.approx(480.0)
    assert axis0["load_rate_pct"] == pytest.approx(23.5)
    assert axis0["igbt_temp_c"] == 42
    assert axis0["motor_temp_c"] == 55
    assert axis0["position_error_counts"] == 12
    assert axis0["drive_not_ready_bits"] == 0x02
    # 0x02 is the "servo_off" bit in the seed skeleton — verify the text
    # decoder runs rather than asserting the specific label (the label
    # map is documented as a work in progress pending manual review).
    assert isinstance(axis0["drive_not_ready_text"], str)
    assert axis0["drive_not_ready_text"] != "ready"
    assert axis0["motor_not_rotating_code"] == 3
    assert isinstance(axis0["motor_not_rotating_text"], str)

    # Axis 1: never sampled — no extended keys added. "truth_available"
    # stays from the display snapshot.
    axis1 = entries[1]
    assert "bus_voltage_v" not in axis1
    assert "igbt_temp_c" not in axis1
    assert "motor_not_rotating_text" not in axis1

    # Axis 2: fresh sample.
    assert entries[2]["bus_voltage_v"] == pytest.approx(481.0)
    assert entries[2]["drive_not_ready_text"] == "ready"  # bits=0 => ready
    assert entries[2]["motor_not_rotating_text"] == "ok"  # code=0 => ok

    # Top-level arrays: present, with None for never-sampled axes so
    # the UI can index them directly.
    assert snapshot["axis_bus_voltage_v"] == [
        pytest.approx(480.0),
        None,
        pytest.approx(481.0),
        None,
        None,
        None,
    ]
    assert snapshot["axis_igbt_temp_c"] == [42, None, 47, None, None, None]
    assert snapshot["axis_motor_temp_c"] == [55, None, 58, None, None, None]
    assert snapshot["axis_position_error_counts"] == [12, None, -7, None, None, None]
    assert snapshot["axis_drive_not_ready_bits"] == [0x02, None, 0x00, None, None, None]
    assert snapshot["axis_drive_not_ready_text"][0] != "ready"
    assert snapshot["axis_drive_not_ready_text"][1] is None
    assert snapshot["axis_drive_not_ready_text"][2] == "ready"
    assert snapshot["axis_motor_not_rotating_code"] == [3, None, 0, None, None, None]
    assert snapshot["axis_motor_not_rotating_text"][2] == "ok"


def test_build_joint_state_snapshot_skips_enrichment_for_bare_backend(monkeypatch):
    """If the active backend does not expose the Phase 1 extended
    accessors (e.g. simulation backend), ``_build_joint_state_snapshot``
    must NOT crash and must NOT emit any top-level extended arrays."""
    canonical_positions = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6]
    backend = object()  # no extended accessors at all

    monkeypatch.setattr(
        run_controller.servo_driver,
        "get_current_arm_state_rad",
        lambda verbose=False: canonical_positions,
    )
    monkeypatch.setattr(run_controller.backend_registry, "get_active_backend", lambda: backend)
    monkeypatch.setattr(
        run_controller.backend_registry, "get_active_backend_name", lambda: "simulation"
    )
    monkeypatch.setattr(run_controller, "_sync_backend_raw_positions", lambda _backend: {0: 0})
    monkeypatch.setattr(
        run_controller,
        "_backend_display_feedback_snapshot",
        lambda _backend, _raw_positions: {
            "truth_available": True,
            "joint_positions_rad": canonical_positions,
            "truth_unavailable_axes": [],
            "truth_unavailable_joints": [],
            "axis_absolute_feedback": [{"axis": 0, "logical_joint": 1, "truth_available": True}],
        },
    )
    monkeypatch.setattr(run_controller.utils, "gripper_present", False, raising=False)

    snapshot = run_controller._build_joint_state_snapshot()

    # No extended fields on any axis entry.
    assert "bus_voltage_v" not in snapshot["axis_absolute_feedback"][0]
    # No top-level extended arrays either.
    assert "axis_bus_voltage_v" not in snapshot
    assert "axis_igbt_temp_c" not in snapshot
    assert "axis_motor_not_rotating_text" not in snapshot


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
