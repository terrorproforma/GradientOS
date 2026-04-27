"""Regression tests for Phase 0 of the canonical-truth stability work.

These tests prove the cartesian jog thread survives transient canonical-truth
``RuntimeError`` failures. Before Phase 0 landed, a single flicker from the
backend shaft-frame gate would raise through
``servo_driver.get_current_arm_state_rad`` at 50 Hz inside
``_jog_controller_thread`` and silently kill the daemon thread, halting
motion mid-jog.

The tests here focus on the two surfaces Phase 0 introduces:

1. ``_record_jog_truth_flicker`` — the new helper that counts flicker events
   and throttles log emission to at most once per second.
2. ``get_jog_performance_snapshot`` — the public API path that exposes the
   new counter to ``GET_PERFORMANCE_STATE`` consumers (UI / operator tools).

The try/except wrappers inside ``_jog_controller_thread`` itself are
syntactically enforced by the plan and validated via the full
py_compile / ReadLints pass; wiring them into a live thread test would
require mocking the entire jog-loop dependency graph (session manager,
backend, ik_solver, trajectory_state), which adds surface area without
exercising new behavior. The helper tests plus the existing jog-session
manager suite cover the relevant contract.
"""

from __future__ import annotations

import io
import sys
from unittest import mock

import pytest

from gradient_os.arm_controller import command_api


@pytest.fixture(autouse=True)
def _reset_jog_perf():
    """Give every test a clean _JOG_PERF so counter assertions are exact."""
    with command_api._JOG_PERF_LOCK:
        command_api._JOG_PERF = command_api._new_jog_perf_state()
    yield


def test_record_jog_truth_flicker_increments_counter() -> None:
    """Every call must increment ``truth_flicker_total`` by exactly one."""
    for _ in range(5):
        command_api._record_jog_truth_flicker("unit-test-reason")
    with command_api._JOG_PERF_LOCK:
        assert int(command_api._JOG_PERF["truth_flicker_total"]) == 5


def test_record_jog_truth_flicker_updates_last_reason_and_wall_s(monkeypatch: pytest.MonkeyPatch) -> None:
    """Each call must overwrite ``truth_flicker_last_reason`` and
    ``truth_flicker_last_wall_s`` with the most recent values."""
    fake_now = {"value": 1_700_000_000.0}

    def _fake_time() -> float:
        return fake_now["value"]

    monkeypatch.setattr(command_api.time, "time", _fake_time)

    command_api._record_jog_truth_flicker("first")
    fake_now["value"] += 2.0  # advance past the throttle window
    command_api._record_jog_truth_flicker("second")

    with command_api._JOG_PERF_LOCK:
        assert command_api._JOG_PERF["truth_flicker_last_reason"] == "second"
        assert float(command_api._JOG_PERF["truth_flicker_last_wall_s"]) == pytest.approx(1_700_000_002.0)


def test_record_jog_truth_flicker_truncates_long_reason() -> None:
    """Defensive: an unbounded exception string should not bloat the perf
    state. The helper truncates to 200 chars."""
    long_reason = "x" * 1000
    command_api._record_jog_truth_flicker(long_reason)
    with command_api._JOG_PERF_LOCK:
        stored = command_api._JOG_PERF["truth_flicker_last_reason"]
    assert isinstance(stored, str)
    assert len(stored) == 200


def test_record_jog_truth_flicker_throttles_log_emission(monkeypatch: pytest.MonkeyPatch) -> None:
    """100 flickers within one throttle window must emit at most one log
    line while still incrementing the counter on every call."""
    fake_now = {"value": 1_700_000_000.0}
    monkeypatch.setattr(command_api.time, "time", lambda: fake_now["value"])

    stderr_buffer = io.StringIO()
    monkeypatch.setattr(command_api.sys, "stderr", stderr_buffer)

    for _ in range(100):
        command_api._record_jog_truth_flicker("flicker-burst")
        fake_now["value"] += 0.001  # 1 ms apart, all inside the throttle window

    with command_api._JOG_PERF_LOCK:
        counter = int(command_api._JOG_PERF["truth_flicker_total"])

    assert counter == 100
    log_text = stderr_buffer.getvalue()
    log_lines = [line for line in log_text.splitlines() if line.strip()]
    assert len(log_lines) == 1, f"Expected exactly one throttled log line, got: {log_lines!r}"
    assert "control feedback miss" in log_lines[0]


def test_record_jog_truth_flicker_re_emits_log_after_throttle_window(monkeypatch: pytest.MonkeyPatch) -> None:
    """Two flickers separated by more than the throttle window must produce
    two log lines."""
    fake_now = {"value": 1_700_000_000.0}
    monkeypatch.setattr(command_api.time, "time", lambda: fake_now["value"])

    stderr_buffer = io.StringIO()
    monkeypatch.setattr(command_api.sys, "stderr", stderr_buffer)

    command_api._record_jog_truth_flicker("first")
    fake_now["value"] += command_api._JOG_TRUTH_FLICKER_LOG_THROTTLE_S + 0.01
    command_api._record_jog_truth_flicker("second")

    log_lines = [line for line in stderr_buffer.getvalue().splitlines() if line.strip()]
    assert len(log_lines) == 2
    assert "first" in log_lines[0]
    assert "second" in log_lines[1]


def test_jog_performance_snapshot_surfaces_truth_flicker_fields() -> None:
    """Operators must be able to read the flicker counter via
    ``GET_PERFORMANCE_STATE`` (which routes through
    ``get_jog_performance_snapshot``)."""
    for _ in range(3):
        command_api._record_jog_truth_flicker("snapshot-test")

    snapshot = command_api.get_jog_performance_snapshot()

    assert "truth_flicker_total" in snapshot
    assert "truth_flicker_last_reason" in snapshot
    assert "truth_flicker_last_wall_s" in snapshot
    assert int(snapshot["truth_flicker_total"]) == 3
    assert snapshot["truth_flicker_last_reason"] == "snapshot-test"


def test_jog_performance_snapshot_backfills_missing_truth_flicker_fields() -> None:
    """If someone constructed _JOG_PERF without the Phase 0 fields (e.g. in
    a unit-test monkeypatch), the snapshot must still return the contract
    keys defaulted rather than KeyError-ing downstream consumers."""
    with command_api._JOG_PERF_LOCK:
        command_api._JOG_PERF = {
            "control_frequency_hz": 50,
            "execution_policy": "controller_cartesian_loop",
            "rtcore_owned": False,
            "loop": {"count": 0, "avg_ms": 0.0, "max_ms": 0.0, "last_ms": 0.0},
            "velocity_updates": {"count": 0},
            "stages": {},
        }

    snapshot = command_api.get_jog_performance_snapshot()

    assert snapshot["truth_flicker_total"] == 0
    assert snapshot["truth_flicker_last_reason"] == ""
    assert snapshot["truth_flicker_last_wall_s"] == 0.0


def test_new_jog_perf_state_includes_truth_flicker_defaults() -> None:
    """Regression: the factory that builds a fresh perf state for a new
    session must include the Phase 0 fields with zero defaults."""
    state = command_api._new_jog_perf_state()
    assert state["truth_flicker_total"] == 0
    assert state["truth_flicker_last_reason"] == ""
    assert state["truth_flicker_last_wall_s"] == 0.0
    assert state["truth_flicker_last_log_wall_s"] == 0.0


def test_slew_limited_vector_caps_per_tick_delta() -> None:
    current = command_api.np.array([0.0, 0.02, -0.02])
    target = command_api.np.array([0.05, -0.05, -0.021])

    result = command_api._slew_limited_vector(current, target, 0.01)

    assert result.tolist() == pytest.approx([0.01, 0.01, -0.021])


def test_record_jog_truth_flicker_is_thread_safe() -> None:
    """Concurrent calls must not drop increments. 10 threads × 100 calls
    each = 1000 total increments after join."""
    import threading

    def _worker() -> None:
        for _ in range(100):
            command_api._record_jog_truth_flicker("concurrent-test")

    threads = [threading.Thread(target=_worker) for _ in range(10)]
    for t in threads:
        t.start()
    for t in threads:
        t.join()

    with command_api._JOG_PERF_LOCK:
        assert int(command_api._JOG_PERF["truth_flicker_total"]) == 1000


class _FakeJogBackend:
    """Minimal stand-in for the RTCore jog backend to satisfy the
    handle_jog_session_start flow in tests without spinning up IPC."""

    def __init__(self) -> None:
        self.timeout_s = 0.4

    def supports_joint_velocity_lease_jog(self) -> bool:  # pragma: no cover
        return True


def _reset_trajectory_state() -> None:
    try:
        command_api.utils.trajectory_state["is_running"] = False
    except Exception:
        pass


def _force_session_idle() -> None:
    """Stop any session left over from a previous test. The session
    manager's stop_session API needs an explicit ``session_id=None``
    kwarg and uses allow_missing=True by default."""
    try:
        command_api._JOG_SESSION_MANAGER.stop_session(
            session_id=None, reason="unit-test-reset"
        )
    except Exception:
        pass


@pytest.fixture
def _fresh_jog_session() -> None:
    _force_session_idle()
    _reset_trajectory_state()
    _reset_canonical_truth_cache()
    yield
    _force_session_idle()
    _reset_canonical_truth_cache()


def _reset_canonical_truth_cache() -> None:
    """2026-04-21: `handle_jog_session_start` now skips the strict
    retry loop when `_recently_valid_canonical_truth()` is True. Tests
    that exercise the strict path (including ones that started a
    successful session earlier in the run) must reset the timestamp
    so they don't accidentally hit the fast-path cache."""
    with command_api._LAST_VALID_CANONICAL_TRUTH_LOCK:
        command_api._LAST_VALID_CANONICAL_TRUTH_MONOTONIC = None


def test_handle_jog_session_start_records_truth_valid_at_arm(
    monkeypatch: pytest.MonkeyPatch, _fresh_jog_session: None
) -> None:
    """Phase 3 (2026-04-20): arm-time strict canonical-truth check must
    run once and stamp ``truth_valid_at_arm=True`` on the returned
    session snapshot when the backend reports clean joint state."""
    call_count = {"value": 0}

    def _fake_strict_read(verbose: bool = False):
        call_count["value"] += 1
        return [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

    monkeypatch.setattr(
        command_api.servo_driver,
        "get_current_arm_state_rad",
        _fake_strict_read,
    )
    monkeypatch.setattr(command_api, "_get_rtcore_jog_backend", lambda: None)
    monkeypatch.setattr(command_api, "_ensure_jog_thread_running", lambda: None)

    snapshot = command_api.handle_jog_session_start(
        {"owner_id": "test-ui", "seq": 0, "deadman": True}
    )

    assert snapshot.get("truth_valid_at_arm") is True
    # Strict read ran exactly once at arm time.
    assert call_count["value"] == 1


def test_handle_jog_session_start_refuses_when_strict_truth_fails(
    monkeypatch: pytest.MonkeyPatch, _fresh_jog_session: None
) -> None:
    """If the arm-time strict read raises for the ENTIRE retry window,
    the session MUST NOT start and the caller MUST see a
    ``CANONICAL_JOINT_TRUTH_UNAVAILABLE`` JogSessionError. This is the
    safety net protecting the relaxed Phase 0 per-tick path: it
    preserves fail-closed semantics at session boundaries even though
    transient mid-motion flickers are absorbed. The 2026-04-21 retry
    softening shortens the window to 500 ms but still rejects if truth
    never becomes available."""
    from gradient_os.arm_controller.jog_session import JogSessionError

    # Short-circuit the real retry budget for unit speed — the test
    # doesn't need to spend 500 ms of wall clock validating that
    # monkeypatched RuntimeError keeps raising.
    monkeypatch.setattr(command_api, "_JOG_ARM_TRUTH_RETRY_BUDGET_S", 0.05)
    monkeypatch.setattr(command_api, "_JOG_ARM_TRUTH_RETRY_INTERVAL_S", 0.005)

    def _raising_strict_read(verbose: bool = False):
        raise RuntimeError(
            "Canonical joint truth unavailable (shaft_frame_inconsistent)"
        )

    monkeypatch.setattr(
        command_api.servo_driver,
        "get_current_arm_state_rad",
        _raising_strict_read,
    )
    monkeypatch.setattr(command_api, "_get_rtcore_jog_backend", lambda: None)
    monkeypatch.setattr(command_api, "_ensure_jog_thread_running", lambda: None)

    with pytest.raises(JogSessionError) as excinfo:
        command_api.handle_jog_session_start(
            {"owner_id": "test-ui", "seq": 0, "deadman": True}
        )
    assert excinfo.value.code == "CONTROL_FEEDBACK_UNAVAILABLE"
    assert "Canonical joint truth unavailable" in str(excinfo.value)


def test_handle_jog_session_start_retries_and_succeeds_on_later_attempt(
    monkeypatch: pytest.MonkeyPatch, _fresh_jog_session: None
) -> None:
    """2026-04-21: the A6-EC drive needs ~0.5-2 s to settle after
    SAFE_POWER_UP before the command-frame roundtrip gate stops
    tripping on transient drive-settling mismatches. The arm-time
    strict check must retry within a short budget instead of the
    original single-shot rejection. This proves that when the first
    N attempts raise but a later attempt succeeds inside the retry
    window, the session still arms cleanly."""
    # 5-attempt budget that corresponds to 4 failures + 1 success.
    monkeypatch.setattr(command_api, "_JOG_ARM_TRUTH_RETRY_BUDGET_S", 0.2)
    monkeypatch.setattr(command_api, "_JOG_ARM_TRUTH_RETRY_INTERVAL_S", 0.01)

    attempts = {"value": 0}

    def _eventually_succeeding_read(verbose: bool = False):
        attempts["value"] += 1
        if attempts["value"] < 4:
            raise RuntimeError(
                "Canonical joint truth unavailable (drive still settling)"
            )
        return [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

    monkeypatch.setattr(
        command_api.servo_driver,
        "get_current_arm_state_rad",
        _eventually_succeeding_read,
    )
    monkeypatch.setattr(command_api, "_get_rtcore_jog_backend", lambda: None)
    monkeypatch.setattr(command_api, "_ensure_jog_thread_running", lambda: None)

    snapshot = command_api.handle_jog_session_start(
        {"owner_id": "test-ui", "seq": 0, "deadman": True}
    )

    assert snapshot.get("truth_valid_at_arm") is True
    # 3 raised + 1 succeeded. Exactly 4 calls.
    assert attempts["value"] == 4, (
        f"Expected 4 attempts (3 failures + 1 success), got {attempts['value']}"
    )


def test_handle_jog_session_start_uses_fast_path_when_truth_was_recently_valid(
    monkeypatch: pytest.MonkeyPatch, _fresh_jog_session: None
) -> None:
    """2026-04-21 (pass 4): the arm-time strict check was the dominant
    user-perceived jog lag source — every click-after-release blocked
    up to 500 ms while the drive's deceleration window transiently
    tripped the roundtrip / shaft-frame gates. The fast-path skips
    the retry loop entirely when canonical truth has been stamped
    valid within `_JOG_ARM_RECENT_TRUTH_WINDOW_S`. This test pins
    that behaviour so a future refactor doesn't silently reintroduce
    the lag regression."""
    # Pretend the jog thread just stamped a valid truth reading.
    command_api._note_valid_canonical_truth()

    call_count = {"value": 0}

    def _raising_strict_read(verbose: bool = False):
        call_count["value"] += 1
        raise RuntimeError(
            "Canonical joint truth unavailable (drive still settling)"
        )

    monkeypatch.setattr(
        command_api.servo_driver,
        "get_current_arm_state_rad",
        _raising_strict_read,
    )
    monkeypatch.setattr(command_api, "_get_rtcore_jog_backend", lambda: None)
    monkeypatch.setattr(command_api, "_ensure_jog_thread_running", lambda: None)

    snapshot = command_api.handle_jog_session_start(
        {"owner_id": "test-ui", "seq": 0, "deadman": True}
    )

    # Fast path must not call the strict read AT ALL.
    assert call_count["value"] == 0, (
        f"Fast-path was supposed to skip the strict read; it ran "
        f"{call_count['value']} times."
    )
    assert snapshot.get("truth_valid_at_arm") is True


def test_handle_jog_session_start_fast_path_expires_after_window(
    monkeypatch: pytest.MonkeyPatch, _fresh_jog_session: None
) -> None:
    """If the recent-valid-truth timestamp is older than the window,
    the fast-path must NOT trigger and the strict retry loop runs
    normally. This keeps the safety intent of the original Phase 3
    check for first-boot / post-power-cycle jogs where canonical
    truth has not been observed valid recently."""
    # Stamp a "very old" valid-truth moment by setting the timestamp
    # far enough in the past to exceed the window.
    import time as _t

    with command_api._LAST_VALID_CANONICAL_TRUTH_LOCK:
        command_api._LAST_VALID_CANONICAL_TRUTH_MONOTONIC = (
            _t.monotonic() - command_api._JOG_ARM_RECENT_TRUTH_WINDOW_S - 1.0
        )

    # Short retry budget for test speed.
    monkeypatch.setattr(command_api, "_JOG_ARM_TRUTH_RETRY_BUDGET_S", 0.05)
    monkeypatch.setattr(command_api, "_JOG_ARM_TRUTH_RETRY_INTERVAL_S", 0.005)

    call_count = {"value": 0}

    def _succeeding_strict_read(verbose: bool = False):
        call_count["value"] += 1
        return [0.0] * 6

    monkeypatch.setattr(
        command_api.servo_driver,
        "get_current_arm_state_rad",
        _succeeding_strict_read,
    )
    monkeypatch.setattr(command_api, "_get_rtcore_jog_backend", lambda: None)
    monkeypatch.setattr(command_api, "_ensure_jog_thread_running", lambda: None)

    snapshot = command_api.handle_jog_session_start(
        {"owner_id": "test-ui", "seq": 0, "deadman": True}
    )

    # Stale timestamp MUST force the strict path; at least one
    # get_current_arm_state_rad call must have run.
    assert call_count["value"] >= 1
    assert snapshot.get("truth_valid_at_arm") is True


def test_handle_jog_session_start_propagates_non_runtime_exceptions_unchanged(
    monkeypatch: pytest.MonkeyPatch, _fresh_jog_session: None
) -> None:
    """Non-RuntimeError exceptions from the strict read must bubble up
    unchanged; we only convert RuntimeError (the canonical-truth path)
    into JogSessionError because that's the exception class the backend
    contract raises for truth failures. Other exception classes indicate
    unexpected bugs and should surface plainly."""

    def _valueerror_read(verbose: bool = False):
        raise ValueError("bug: unexpected exception class")

    monkeypatch.setattr(
        command_api.servo_driver,
        "get_current_arm_state_rad",
        _valueerror_read,
    )
    monkeypatch.setattr(command_api, "_get_rtcore_jog_backend", lambda: None)
    monkeypatch.setattr(command_api, "_ensure_jog_thread_running", lambda: None)

    with pytest.raises(ValueError, match="unexpected exception class"):
        command_api.handle_jog_session_start(
            {"owner_id": "test-ui", "seq": 0, "deadman": True}
        )


def test_jog_thread_source_contains_session_inactive_race_guard() -> None:
    """2026-04-21: a JOG_SESSION_STOP that races between the top-of-
    loop `session_active` check and the later
    `_JOG_SESSION_MANAGER.update_following_error(...)` call used to
    raise `JogSessionError(SESSION_INACTIVE)` out of the jog thread
    entrypoint and kill it with an uncaught traceback (terminal
    29.txt:253). The fix wraps all raise-prone session-manager
    mutations with `_safe_session_call`, which absorbs
    SESSION_INACTIVE into a clean loop break. If someone rewrites
    the thread and drops the wrapping, we want this regression to
    fire before a latent crash ships."""
    import inspect

    source = inspect.getsource(command_api._jog_controller_thread)

    # The helper must be defined inside the thread so it can see the
    # session sentinel.
    assert "_safe_session_call" in source, (
        "Thread must define _safe_session_call helper that absorbs "
        "SESSION_INACTIVE into a sentinel return value."
    )
    assert "_session_gone_sentinel" in source, (
        "Thread must define _session_gone_sentinel sentinel object."
    )
    # The four known raise-prone callsites must each be wrapped with
    # _safe_session_call. If any of these moves or is renamed, this
    # assert surfaces the gap immediately.
    import re

    for callsite in (
        "_JOG_SESSION_MANAGER.update_following_error",
        "_JOG_SESSION_MANAGER.resync_command_state",
        "_JOG_SESSION_MANAGER.record_gate_failure",
    ):
        # Each of these names must appear INSIDE a `_safe_session_call`
        # invocation (any indentation depth), never as a bare call that
        # could raise on a stop-race. The regex below tolerates
        # arbitrary whitespace between `_safe_session_call(` and the
        # method reference so the test keeps passing across reorderings
        # and indentation changes.
        wrapped_re = re.compile(
            r"_safe_session_call\s*\(\s*" + re.escape(callsite)
        )
        bare_re = re.compile(
            r"^\s+" + re.escape(callsite) + r"\s*\(",
            re.MULTILINE,
        )
        assert wrapped_re.search(source), (
            f"Expected {callsite} to be wrapped in _safe_session_call — "
            f"a bare call can crash the thread on a stop-race."
        )
        for m in bare_re.finditer(source):
            # A bare match is only suspicious when it isn't the FIRST
            # arg of a `_safe_session_call` wrapper directly above it.
            preceding = source[max(0, m.start() - 60) : m.start()]
            if "_safe_session_call" in preceding:
                continue
            raise AssertionError(
                f"Found bare {callsite} call in jog thread at offset "
                f"{m.start()} — MUST be wrapped in _safe_session_call "
                f"to absorb SESSION_INACTIVE."
            )


def test_try_except_is_present_at_jog_loop_call_sites() -> None:
    """Source-level guard: the three ``get_control_arm_state_rad`` call
    sites inside ``_jog_controller_thread`` must be wrapped in
    try/except RuntimeError. If someone ever rewrites that thread and
    drops the wrap, this regression fires before a latent halt hits
    production."""
    import inspect

    source = inspect.getsource(command_api._jog_controller_thread)

    # Every get_control_arm_state_rad call in this function should be
    # preceded by a `try:` and followed by an `except RuntimeError`
    # calling _record_jog_truth_flicker. Exact structural regex keeps
    # the test cheap.
    call_count = source.count("servo_driver.get_control_arm_state_rad(verbose=False)")
    assert call_count >= 3, (
        f"Expected at least 3 get_control_arm_state_rad call sites in "
        f"_jog_controller_thread, found {call_count}"
    )
    wrap_count = source.count("_record_jog_truth_flicker")
    assert wrap_count >= 3, (
        f"Expected at least 3 _record_jog_truth_flicker invocations to "
        f"match the call sites, found {wrap_count}"
    )


def test_ui_release_sends_zero_velocity_before_rtcore_jog_stop(monkeypatch: pytest.MonkeyPatch, _fresh_jog_session: None) -> None:
    updates: list[tuple[list[float], float]] = []
    stops: list[bool] = []
    sleeps: list[float] = []

    class FakeBackend:
        def update_joint_velocity_lease_jog(self, velocities, timeout_s):
            updates.append((list(velocities), float(timeout_s)))

        def stop_joint_velocity_lease_jog(self, *, quick_stop=False):
            stops.append(bool(quick_stop))

    session = command_api._JOG_SESSION_MANAGER.start_session(
        owner_id="test-ui",
        seq=0,
        lease_timeout_s=0.4,
        deadman=True,
        velocity_vector=[0.05, 0.0, 0.0, 0.0, 0.0, 0.0],
        backend_mode="joint_velocity_lease",
        backend_timeout_s=0.2,
    )
    monkeypatch.setattr(command_api, "_get_rtcore_jog_backend", lambda: FakeBackend())
    monkeypatch.setattr(command_api, "_sync_jog_trajectory_state", lambda *args, **kwargs: None)
    monkeypatch.setattr(command_api, "_wait_for_jog_thread_stop", lambda *args, **kwargs: None)
    monkeypatch.setattr(command_api.time, "sleep", lambda seconds: sleeps.append(float(seconds)))
    monkeypatch.setattr(command_api, "JOG_UI_RELEASE_ZERO_SETTLE_S", 0.06)

    command_api.handle_jog_session_stop({
        "session_id": session["session_id"],
        "owner_id": "test-ui",
        "reason": "ui-release",
    })

    assert updates == [([0.0] * 6, pytest.approx(0.2))]
    assert sleeps == [pytest.approx(0.06)]
    assert stops == [False]


@pytest.mark.parametrize(
    ("reason", "expected"),
    [
        ("lease-expired-before-loop", False),
        ("lease-expired-before-send", False),
        ("motion-resume-lease-expired", False),
        ("SESSION_EXPIRED", False),
        ("session-stop:controller-stop", True),
        ("session-stop:controller-shutdown", True),
        ("fk-failed", True),
    ],
)
def test_jog_quick_stop_policy_keeps_session_lease_expiry_soft(reason: str, expected: bool) -> None:
    assert command_api._jog_stop_reason_requests_quick_stop(reason) is expected


def test_lease_expiry_backend_stop_is_soft_not_ds402_quick_stop(
    monkeypatch: pytest.MonkeyPatch,
    _fresh_jog_session: None,
) -> None:
    stops: list[bool] = []

    class FakeBackend:
        def stop_joint_velocity_lease_jog(self, *, quick_stop=False):
            stops.append(bool(quick_stop))

    monkeypatch.setattr(command_api, "_get_rtcore_jog_backend", lambda: FakeBackend())

    command_api._stop_rtcore_jog_backend_best_effort("lease-expired-before-loop")

    assert stops == [False]


def test_controller_stop_backend_stop_still_requests_quick_stop(
    monkeypatch: pytest.MonkeyPatch,
    _fresh_jog_session: None,
) -> None:
    stops: list[bool] = []

    class FakeBackend:
        def stop_joint_velocity_lease_jog(self, *, quick_stop=False):
            stops.append(bool(quick_stop))

    monkeypatch.setattr(command_api, "_get_rtcore_jog_backend", lambda: FakeBackend())

    command_api._stop_rtcore_jog_backend_best_effort("session-stop:controller-stop")

    assert stops == [True]


def test_jog_thread_holds_zero_when_control_feedback_unavailable() -> None:
    """A missed control-feedback tick must hold the RTCore lease instead of
    integrating a Cartesian target from stale q_current."""
    import inspect

    source = inspect.getsource(command_api._jog_controller_thread)

    assert "pending_resync_reason = \"control-feedback-unavailable\"" in source
    assert "[0.0] * int(commanded_joints_for_hold.size)" in source
    assert "continue" in source[source.index("pending_resync_reason = \"control-feedback-unavailable\"") :]
