"""Collision-detection watchdog for the live EtherCAT backend.

Phase 4 (2026-04-20) foundation layer: monitor the Phase 1 extended
PDO telemetry (torque, position error) against per-axis configurable
thresholds and request a safe power-down with ``quick_stop=True`` when
a sustained excursion is seen. The watchdog is deliberately simple and
layered: it reads already-published backend state (no SDO traffic, no
RTCore IPC), runs at a modest cadence (default 100 Hz), and edge-
triggers on sustained conditions to avoid nuisance trips from
single-sample transients.

Thresholds should be calibrated on hardware by observing peak torque
and peak position error during known-safe motion and setting the
watchdog bound to ~1.5x the peak. See
``gradient05/config.py::collision_watchdog_thresholds`` for the
production thresholds and the Phase 4.5 calibration notes.

Contract with the jog thread and the RTCore loop:
* The watchdog never touches setpoint state directly. It only requests
  a safe power-down via the callback supplied at construction. The
  caller is responsible for propagating the event (telemetry, UI).
* Triggers are edge-triggered: one callback invocation per sustained
  window, then the counter resets so the same excursion doesn't
  re-fire every cycle.
* Axes that have never received a live extended-PDO snapshot
  (``_axis_extended_updated_ns == 0``) are skipped. This is critical
  during bring-up when the drive may not yet have accepted the
  extended mapping; the watchdog must not trip on default zeros.
"""

from __future__ import annotations

import threading
import time
from dataclasses import dataclass
from typing import Callable, Sequence


@dataclass(slots=True, frozen=True)
class CollisionThresholds:
    """Per-axis collision-detection bounds.

    ``torque_abs_max_raw``: absolute value of ``0x6077`` raw counts
        (0.1 % of rated torque per the A6-EC manual) above which a
        torque spike is flagged. Example: 2000 ~= 200 % of rated.

    ``position_error_counts_max``: absolute value of ``0x2040:0x11``
        in drive counts above which a position-error excursion is
        flagged. Example: 5000 counts on a 17-bit encoder is a
        ~3.8-degree tracking error on the motor shaft.

    ``sustained_samples``: number of consecutive above-threshold
        samples required before firing the callback. Filters out
        single-sample spikes.

    ``sample_period_s``: loop period between checks. 0.01 (100 Hz) is
        a reasonable default; lower values raise watchdog CPU cost
        without adding sensitivity beyond the PDO update rate.
    """

    torque_abs_max_raw: int
    position_error_counts_max: int
    sustained_samples: int
    sample_period_s: float


@dataclass(slots=True, frozen=True)
class CollisionEvent:
    """Edge-triggered event describing a detected collision.

    ``axis_i``: axis that tripped. Use the backend's ``_axis_to_joint``
        map to resolve to a logical joint number if needed.

    ``reason``: ``"torque_spike"`` (motor fighting an obstacle) or
        ``"position_error_excursion"`` (drive could not track within
        the internal following-error window).

    ``observed_value``: absolute value of the signal at the moment
        the threshold was crossed for the ``sustained_samples``-th
        consecutive time.

    ``threshold``: configured bound the signal exceeded.

    ``wall_s``: ``time.time()`` wall-clock timestamp at detection.
    """

    axis_i: int
    reason: str
    observed_value: int
    threshold: int
    wall_s: float


class CollisionWatchdog:
    """Background watchdog thread. Construct with the backend to
    monitor and a callback invoked on each sustained trip.

    The watchdog is deliberately decoupled from the rest of the
    motion stack: it never calls RTCore directly and never touches
    the jog session manager. On trigger it simply invokes the
    ``on_collision`` callback; the caller is responsible for
    requesting a safe power-down, emitting a telemetry event, or
    whatever else the operator workflow demands.
    """

    def __init__(
        self,
        *,
        backend: object,
        thresholds: Sequence[CollisionThresholds],
        on_collision: Callable[[CollisionEvent], None],
    ) -> None:
        self._backend = backend
        self._thresholds: list[CollisionThresholds] = list(thresholds)
        self._on_collision = on_collision
        self._thread: threading.Thread | None = None
        self._stop_event = threading.Event()
        self._sustained_counts: dict[tuple[int, str], int] = {}

    @property
    def is_running(self) -> bool:
        return self._thread is not None and self._thread.is_alive()

    def start(self) -> None:
        if self._thread is not None and self._thread.is_alive():
            return
        self._stop_event.clear()
        self._thread = threading.Thread(
            target=self._run, daemon=True, name="collision-watchdog"
        )
        self._thread.start()

    def stop(self, *, join_timeout_s: float = 1.0) -> None:
        self._stop_event.set()
        thread = self._thread
        if thread is not None:
            thread.join(timeout=join_timeout_s)
            self._thread = None

    def _run(self) -> None:
        while not self._stop_event.is_set():
            loop_start = time.monotonic()
            try:
                self.check_once()
            except Exception as exc:  # pragma: no cover - defensive
                print(
                    f"[Collision] WARNING: watchdog tick raised: {exc}",
                    flush=True,
                )
            elapsed = time.monotonic() - loop_start
            period = self._default_period_s()
            sleep = max(0.0, period - elapsed)
            # Using wait() rather than sleep() so stop() returns promptly.
            if self._stop_event.wait(sleep):
                break

    def _default_period_s(self) -> float:
        if not self._thresholds:
            return 0.01
        # Use the first axis's period as the loop rate. All axes share
        # the watchdog thread; the finest period wins if thresholds
        # disagree.
        return min(float(t.sample_period_s) for t in self._thresholds if t.sample_period_s > 0.0)

    def check_once(self) -> None:
        """Synchronous single pass over all configured axes. Exposed
        so tests can drive the watchdog deterministically without
        spawning the thread."""
        backend = self._backend
        status_lock = getattr(backend, "_status_lock", None)
        torque_arr = getattr(backend, "_axis_torque_raw", None)
        pe_arr = getattr(backend, "_axis_position_error_counts", None)
        updated_arr = getattr(backend, "_axis_extended_updated_ns", None)
        if torque_arr is None or pe_arr is None or updated_arr is None:
            return

        for axis_i, thresholds in enumerate(self._thresholds):
            if status_lock is not None:
                with status_lock:
                    (
                        torque,
                        position_error,
                        updated_ns,
                    ) = self._sample_axis_locked(
                        axis_i, torque_arr, pe_arr, updated_arr
                    )
            else:
                torque, position_error, updated_ns = self._sample_axis_locked(
                    axis_i, torque_arr, pe_arr, updated_arr
                )

            if updated_ns is None or int(updated_ns) == 0:
                # Axis has never received a live extended-PDO snapshot;
                # skip it so bring-up defaults cannot trip the watchdog.
                continue

            self._evaluate(
                axis_i,
                "torque_spike",
                abs(int(torque)),
                int(thresholds.torque_abs_max_raw),
                int(thresholds.sustained_samples),
            )
            self._evaluate(
                axis_i,
                "position_error_excursion",
                abs(int(position_error)),
                int(thresholds.position_error_counts_max),
                int(thresholds.sustained_samples),
            )

    def _sample_axis_locked(
        self,
        axis_i: int,
        torque_arr: Sequence[int],
        pe_arr: Sequence[int],
        updated_arr: Sequence[int],
    ) -> tuple[int, int, int | None]:
        torque = 0
        pe = 0
        updated_ns: int | None = None
        if 0 <= axis_i < len(torque_arr):
            torque = int(torque_arr[axis_i])
        if 0 <= axis_i < len(pe_arr):
            pe = int(pe_arr[axis_i])
        if 0 <= axis_i < len(updated_arr):
            updated_ns = int(updated_arr[axis_i])
        return torque, pe, updated_ns

    def _evaluate(
        self,
        axis_i: int,
        reason: str,
        observed: int,
        threshold: int,
        sustained_required: int,
    ) -> None:
        if sustained_required <= 0:
            sustained_required = 1
        key = (axis_i, reason)
        if observed > threshold:
            count = self._sustained_counts.get(key, 0) + 1
            self._sustained_counts[key] = count
            if count >= sustained_required:
                # Edge-triggered: reset the counter so the same
                # excursion does not re-fire on every tick. The next
                # trip requires another fresh sustained burst.
                self._sustained_counts[key] = 0
                event = CollisionEvent(
                    axis_i=axis_i,
                    reason=reason,
                    observed_value=observed,
                    threshold=threshold,
                    wall_s=time.time(),
                )
                try:
                    self._on_collision(event)
                except Exception as exc:  # pragma: no cover - defensive
                    print(
                        f"[Collision] WARNING: on_collision callback raised: {exc}",
                        flush=True,
                    )
        else:
            # Recovery below threshold resets the sustained counter.
            self._sustained_counts.pop(key, None)
