"""Regression tests for the Phase 4 collision watchdog.

The watchdog is designed as a safety overlay over the Phase 1 extended
PDO telemetry. These tests exercise it without spinning up the jog
thread / RTCore / IPC stack — we use a minimal ``FakeBackend`` exposing
the per-axis arrays the watchdog reads.

Coverage:

* Sustained above-threshold torque triggers the callback once per
  burst (edge-triggered).
* Sub-threshold samples never trigger.
* Single-sample spikes below ``sustained_samples`` do not trigger.
* Recovery below threshold resets the sustained counter so the next
  trip requires a fresh burst.
* Axes with ``_axis_extended_updated_ns == 0`` (never received a live
  extended snapshot) are skipped so bring-up defaults cannot trip it.
* Position-error excursions fire the callback with the correct
  reason label.
* Callback exceptions do not propagate out of the watchdog (they are
  logged but the watchdog keeps running).
"""

from __future__ import annotations

import threading
import time
from dataclasses import dataclass
from typing import Callable

import pytest

from gradient_os.arm_controller.collision_watchdog import (
    CollisionEvent,
    CollisionThresholds,
    CollisionWatchdog,
)


@dataclass
class _FakeBackend:
    """Minimal backend stub exposing only the fields the watchdog reads."""

    _axis_torque_raw: list[int]
    _axis_position_error_counts: list[int]
    _axis_extended_updated_ns: list[int]
    _status_lock: threading.Lock

    @classmethod
    def make(cls, num_axes: int = 6) -> "_FakeBackend":
        return cls(
            _axis_torque_raw=[0] * num_axes,
            _axis_position_error_counts=[0] * num_axes,
            _axis_extended_updated_ns=[1_000_000_000] * num_axes,  # mark all axes fresh
            _status_lock=threading.Lock(),
        )


def _default_thresholds(num_axes: int = 6, sustained: int = 3) -> list[CollisionThresholds]:
    return [
        CollisionThresholds(
            torque_abs_max_raw=1000,
            position_error_counts_max=5000,
            sustained_samples=sustained,
            sample_period_s=0.01,
        )
        for _ in range(num_axes)
    ]


def test_watchdog_triggers_on_sustained_torque_spike():
    """Three consecutive above-threshold ticks fire the callback exactly once."""
    backend = _FakeBackend.make()
    events: list[CollisionEvent] = []
    watchdog = CollisionWatchdog(
        backend=backend,
        thresholds=_default_thresholds(sustained=3),
        on_collision=events.append,
    )

    # Below threshold — nothing happens.
    backend._axis_torque_raw[2] = 500
    watchdog.check_once()
    assert events == []

    # Above threshold for sustained_samples - 1 ticks — still nothing.
    backend._axis_torque_raw[2] = 1500
    watchdog.check_once()
    watchdog.check_once()
    assert events == []

    # One more sustained tick triggers the callback.
    watchdog.check_once()
    assert len(events) == 1
    event = events[0]
    assert event.axis_i == 2
    assert event.reason == "torque_spike"
    assert event.observed_value == 1500
    assert event.threshold == 1000


def test_watchdog_never_triggers_when_all_samples_are_below_threshold():
    """High noise floor but below threshold must not trip."""
    backend = _FakeBackend.make()
    events: list[CollisionEvent] = []
    watchdog = CollisionWatchdog(
        backend=backend,
        thresholds=_default_thresholds(sustained=2),
        on_collision=events.append,
    )
    backend._axis_torque_raw[0] = 999  # just under the threshold
    for _ in range(20):
        watchdog.check_once()
    assert events == []


def test_watchdog_ignores_single_sample_spike_below_sustained_count():
    """A lone spike shorter than ``sustained_samples`` must not trigger."""
    backend = _FakeBackend.make()
    events: list[CollisionEvent] = []
    watchdog = CollisionWatchdog(
        backend=backend,
        thresholds=_default_thresholds(sustained=3),
        on_collision=events.append,
    )
    # One spike flanked by sub-threshold samples.
    backend._axis_torque_raw[0] = 2000  # well over
    watchdog.check_once()  # count = 1
    backend._axis_torque_raw[0] = 0  # recover
    watchdog.check_once()
    watchdog.check_once()
    assert events == []


def test_watchdog_resets_sustained_counter_on_recovery():
    """Two above, one below, two above — must NOT trigger because the
    recovery sample resets the counter."""
    backend = _FakeBackend.make()
    events: list[CollisionEvent] = []
    watchdog = CollisionWatchdog(
        backend=backend,
        thresholds=_default_thresholds(sustained=3),
        on_collision=events.append,
    )
    backend._axis_torque_raw[0] = 1500
    watchdog.check_once()
    watchdog.check_once()
    backend._axis_torque_raw[0] = 0  # recovery resets counter
    watchdog.check_once()
    backend._axis_torque_raw[0] = 1500
    watchdog.check_once()
    watchdog.check_once()
    assert events == []  # 2 + 2 consecutive, never reached 3


def test_watchdog_triggers_on_position_error_excursion():
    """Position-error above threshold fires with reason=position_error_excursion."""
    backend = _FakeBackend.make()
    events: list[CollisionEvent] = []
    watchdog = CollisionWatchdog(
        backend=backend,
        thresholds=_default_thresholds(sustained=2),
        on_collision=events.append,
    )
    backend._axis_position_error_counts[3] = -6000  # over 5000 threshold
    watchdog.check_once()
    watchdog.check_once()
    assert len(events) == 1
    event = events[0]
    assert event.axis_i == 3
    assert event.reason == "position_error_excursion"
    # Observed value is always abs(sample).
    assert event.observed_value == 6000


def test_watchdog_skips_axes_without_extended_data():
    """Axis with ``_axis_extended_updated_ns == 0`` must be skipped; the
    drive may not have accepted the extended PDO mapping, so zero raw
    values there are meaningless and must not be interpreted as a
    zero-torque or zero-PE valid reading."""
    backend = _FakeBackend.make()
    events: list[CollisionEvent] = []
    watchdog = CollisionWatchdog(
        backend=backend,
        thresholds=_default_thresholds(sustained=1),
        on_collision=events.append,
    )
    backend._axis_torque_raw[4] = 9999  # would trip
    backend._axis_extended_updated_ns[4] = 0  # but never received live data
    for _ in range(10):
        watchdog.check_once()
    assert events == []

    # Confirm the complementary: once the axis DOES receive data, a fresh
    # burst does trip.
    backend._axis_extended_updated_ns[4] = 1_234_567_890
    watchdog.check_once()
    assert len(events) == 1
    assert events[0].axis_i == 4


def test_watchdog_edge_triggers_per_sustained_burst():
    """After a trigger the counter resets. A held-above-threshold signal
    must fire again only when the next fresh sustained burst lands."""
    backend = _FakeBackend.make()
    events: list[CollisionEvent] = []
    watchdog = CollisionWatchdog(
        backend=backend,
        thresholds=_default_thresholds(sustained=2),
        on_collision=events.append,
    )
    backend._axis_torque_raw[0] = 1500
    watchdog.check_once()  # count = 1
    watchdog.check_once()  # count = 2 -> trigger; counter resets to 0
    assert len(events) == 1
    watchdog.check_once()  # count = 1 again
    watchdog.check_once()  # count = 2 -> second trigger
    assert len(events) == 2


def test_watchdog_swallows_callback_exceptions():
    """If the callback raises, the watchdog must continue running and
    process subsequent trips; the collision detection path must never
    bring down the controller."""
    backend = _FakeBackend.make()
    call_count = {"value": 0}

    def _exploding_callback(_event: CollisionEvent) -> None:
        call_count["value"] += 1
        raise RuntimeError("operator handler failed")

    watchdog = CollisionWatchdog(
        backend=backend,
        thresholds=_default_thresholds(sustained=1),
        on_collision=_exploding_callback,
    )
    backend._axis_torque_raw[0] = 2000
    # Must not raise.
    watchdog.check_once()
    backend._axis_torque_raw[0] = 0
    watchdog.check_once()
    backend._axis_torque_raw[0] = 2000
    watchdog.check_once()
    # Both bursts attempted to call the callback.
    assert call_count["value"] == 2


def test_watchdog_tolerates_backend_without_extended_state_arrays():
    """If the backend does not expose the Phase 1 arrays (e.g. bare
    object() or a simulation backend pre-Phase-1), the watchdog must
    no-op gracefully rather than AttributeError the controller."""
    backend = object()  # no extended-PDO arrays at all
    events: list[CollisionEvent] = []
    watchdog = CollisionWatchdog(
        backend=backend,
        thresholds=_default_thresholds(sustained=1),
        on_collision=events.append,
    )
    # Must not raise, must not trip.
    watchdog.check_once()
    assert events == []


def test_watchdog_thread_start_and_stop_is_idempotent():
    """start() and stop() must be safe to call any number of times
    without leaking threads or raising."""
    backend = _FakeBackend.make()
    events: list[CollisionEvent] = []
    watchdog = CollisionWatchdog(
        backend=backend,
        thresholds=_default_thresholds(sustained=1),
        on_collision=events.append,
    )
    watchdog.start()
    assert watchdog.is_running
    # Idempotent start.
    watchdog.start()
    assert watchdog.is_running
    watchdog.stop()
    assert not watchdog.is_running
    # Idempotent stop.
    watchdog.stop()
    assert not watchdog.is_running


def test_watchdog_thread_actually_fires_callback():
    """Integration smoke test: start the watchdog thread, flip an axis
    above threshold, verify the callback fires within the loop period.
    The watchdog's default period is derived from thresholds[0], so we
    use a short period to keep the test fast."""
    backend = _FakeBackend.make()
    events: list[CollisionEvent] = []
    cond = threading.Condition()

    def _record(event: CollisionEvent) -> None:
        with cond:
            events.append(event)
            cond.notify_all()

    watchdog = CollisionWatchdog(
        backend=backend,
        thresholds=[
            CollisionThresholds(
                torque_abs_max_raw=1000,
                position_error_counts_max=5000,
                sustained_samples=1,  # fire on first above-threshold tick
                sample_period_s=0.005,
            )
            for _ in range(6)
        ],
        on_collision=_record,
    )
    watchdog.start()
    try:
        backend._axis_torque_raw[1] = 1500
        with cond:
            # Wait up to 500 ms for the watchdog to fire.
            fired = cond.wait_for(lambda: len(events) > 0, timeout=0.5)
        assert fired, "watchdog did not fire within 500 ms"
        assert events[0].axis_i == 1
        assert events[0].reason == "torque_spike"
    finally:
        watchdog.stop()


def test_gradient05_config_provides_six_axis_thresholds():
    """Regression: the robot config must expose thresholds for every
    axis the controller plans to watch; a missing threshold for any
    axis would silently disable collision detection on that joint."""
    from gradient_os.arm_controller.robots.gradient05.config import Gradient05Config

    thresholds = Gradient05Config().collision_watchdog_thresholds
    assert len(thresholds) == 6
    for t in thresholds:
        assert t.torque_abs_max_raw > 0
        assert t.position_error_counts_max > 0
        assert t.sustained_samples >= 1
        assert t.sample_period_s > 0.0


def test_watchdog_can_be_driven_concurrently_from_check_once():
    """Multi-threaded safety: check_once must not crash when called
    from multiple threads. We don't care about strict once-per-burst
    accuracy under contention (the watchdog is single-threaded in
    practice), only that nothing explodes."""
    backend = _FakeBackend.make()
    watchdog = CollisionWatchdog(
        backend=backend,
        thresholds=_default_thresholds(sustained=1),
        on_collision=lambda _e: None,
    )
    backend._axis_torque_raw[0] = 1500

    def _worker():
        for _ in range(50):
            watchdog.check_once()

    threads = [threading.Thread(target=_worker) for _ in range(4)]
    for t in threads:
        t.start()
    for t in threads:
        t.join()
