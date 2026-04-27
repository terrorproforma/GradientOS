# backends/fairino/state_stream.py
#
# Background poller that asks the Fairino RPC client for joint positions and
# status at a fixed rate, caches the latest snapshot in a thread-safe slot,
# and lets callers read it without blocking.
#
# Why polling instead of a true push stream from port 20004:
#   - The Fairino SDK we're targeting wraps both ports 20003 (commands) and
#     20004 (state) behind one client object. The natural API is `client.
#     GetActualJointPosDegree()` — a polled read. Until we verify whether
#     the SDK exposes the raw 20004 stream, polling at a fixed rate is the
#     simplest correct approach and is plenty for Milestone 1 (UI display).
#   - At 50 Hz the latency is 20 ms — fine for a human watching joint
#     readouts in a browser. Milestone 4+ may require lifting this to 125 Hz
#     for closed-loop work, at which point we revisit.

from __future__ import annotations

import math
import threading
import time
from dataclasses import dataclass, field
from typing import Optional

from .config import FR10_NUM_JOINTS
from .rpc_client import FairinoRPCClient, FairinoStatusFlags


@dataclass
class StateSnapshot:
    """One frozen reading of the FR10's reported state.

    `timestamp_monotonic_s == 0.0` means "no reading received yet" — the
    snapshot is at its initial sentinel value. Consumers should check
    `has_reading` rather than inspecting the timestamp directly.
    """

    joint_positions_rad: list[float] = field(
        default_factory=lambda: [0.0] * FR10_NUM_JOINTS
    )
    status: FairinoStatusFlags = field(default_factory=FairinoStatusFlags)
    timestamp_monotonic_s: float = 0.0
    sequence: int = 0

    @property
    def has_reading(self) -> bool:
        return self.sequence > 0

    def age_s(self, now_s: Optional[float] = None) -> float:
        if not self.has_reading:
            return math.inf
        return (now_s if now_s is not None else time.monotonic()) - self.timestamp_monotonic_s


class FairinoStateStream:
    """Daemon-thread state poller backed by a `FairinoRPCClient`.

    Lifecycle:
      `start()` — spawn the poller thread (idempotent).
      `stop()`  — signal the poller to exit and join (idempotent, never
                  raises). Always called from `FairinoBackend.shutdown()`.
      `latest()` — return a copy of the most recent snapshot, never blocks.

    The poller swallows transient read errors and just stops updating —
    callers detect staleness via `latest().age_s()`. Bubbling up exceptions
    from the daemon thread would be invisible to the main thread anyway.
    """

    def __init__(self, rpc: FairinoRPCClient, poll_hz: float) -> None:
        if poll_hz <= 0.0:
            raise ValueError(f"poll_hz must be > 0, got {poll_hz}")
        self._rpc = rpc
        self._period_s = 1.0 / float(poll_hz)
        self._stop_evt = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self._snapshot_lock = threading.Lock()
        self._snapshot = StateSnapshot()
        self._consecutive_errors = 0
        self._last_error_log_s = 0.0

    def start(self) -> None:
        if self._thread is not None and self._thread.is_alive():
            return
        self._stop_evt.clear()
        self._thread = threading.Thread(
            target=self._run,
            name="fairino-state-stream",
            daemon=True,
        )
        self._thread.start()

    def stop(self) -> None:
        self._stop_evt.set()
        thread = self._thread
        if thread is not None and thread.is_alive():
            thread.join(timeout=2.0)
        self._thread = None

    def latest(self) -> StateSnapshot:
        """Return a snapshot copy. Never blocks on the poller."""
        with self._snapshot_lock:
            snap = self._snapshot
            return StateSnapshot(
                joint_positions_rad=list(snap.joint_positions_rad),
                status=FairinoStatusFlags(
                    in_collision=snap.status.in_collision,
                    estopped=snap.status.estopped,
                    faulted=snap.status.faulted,
                    program_state=snap.status.program_state,
                    last_error_code=snap.status.last_error_code,
                ),
                timestamp_monotonic_s=snap.timestamp_monotonic_s,
                sequence=snap.sequence,
            )

    def _run(self) -> None:
        next_tick = time.monotonic()
        while not self._stop_evt.is_set():
            try:
                joints = self._rpc.read_joint_positions_rad()
                status = self._rpc.read_status_flags()
                self._publish(joints, status)
                self._consecutive_errors = 0
            except Exception as e:
                self._consecutive_errors += 1
                # Log first error and every ~5s thereafter, not every cycle.
                now = time.monotonic()
                if self._consecutive_errors == 1 or (now - self._last_error_log_s) > 5.0:
                    print(
                        f"[Fairino state] poller error "
                        f"(consec={self._consecutive_errors}): {e}"
                    )
                    self._last_error_log_s = now

            next_tick += self._period_s
            sleep_s = next_tick - time.monotonic()
            if sleep_s > 0:
                # Use the stop event for sleep so stop() returns promptly.
                self._stop_evt.wait(timeout=sleep_s)
            else:
                # We fell behind — reset the cadence rather than burst-catching.
                next_tick = time.monotonic()

    def _publish(self, joints_rad: list[float], status: FairinoStatusFlags) -> None:
        with self._snapshot_lock:
            self._snapshot = StateSnapshot(
                joint_positions_rad=list(joints_rad),
                status=status,
                timestamp_monotonic_s=time.monotonic(),
                sequence=self._snapshot.sequence + 1,
            )
