#!/usr/bin/env python3
"""In-process timing of the jog-thread feedback read path.

The API read of /info/joints-detailed measures 10-40 ms per call, but
that includes FastAPI + JSON serialisation. The jog thread calls
`servo_driver.get_current_arm_state_rad()` directly via the Python
backend — no HTTP. This script times each layer of that direct call
to identify which one is the actual bottleneck.
"""
from __future__ import annotations

import statistics
import sys
import time


def _time(fn, iters: int = 100) -> dict[str, float]:
    samples = []
    for _ in range(iters):
        t0 = time.monotonic()
        fn()
        samples.append((time.monotonic() - t0) * 1000.0)
    s = sorted(samples)
    pick = lambda f: s[min(int(f * len(s)), len(s) - 1)]
    return {
        "n": len(s),
        "min": min(s),
        "p50": statistics.median(s),
        "p90": pick(0.9),
        "p99": pick(0.99),
        "max": max(s),
        "mean": statistics.mean(s),
    }


def _fmt(label: str, stats: dict[str, float]) -> str:
    return (
        f"{label:<48s} n={stats['n']:3d} "
        f"min={stats['min']:6.2f}ms p50={stats['p50']:6.2f}ms "
        f"p90={stats['p90']:6.2f}ms p99={stats['p99']:6.2f}ms "
        f"max={stats['max']:6.2f}ms mean={stats['mean']:6.2f}ms"
    )


def main() -> int:
    # Connect to the running controller via the same API surface the
    # jog thread uses — the EtherCAT backend in-process.
    from gradient_os.arm_controller.backends.registry import get_active_backend_instance
    from gradient_os.arm_controller import servo_driver
    backend = get_active_backend_instance()
    if backend is None or getattr(backend, "__class__", None) is None:
        print("No active backend; run this script while the stack is up.")
        return 1
    print(f"backend={backend.__class__.__name__} connected={getattr(backend, '_connected', '?')}")

    # Warm up any caches first.
    for _ in range(10):
        backend.sync_read_positions()
        backend.get_joint_positions(verbose=False)

    print()
    print("== Direct backend calls (no HTTP) ==")
    print(_fmt("sync_read_positions", _time(lambda: backend.sync_read_positions(), 200)))
    print(_fmt("get_joint_positions", _time(lambda: backend.get_joint_positions(verbose=False), 200)))
    print(_fmt("servo_driver.get_current_arm_state_rad", _time(lambda: servo_driver.get_current_arm_state_rad(verbose=False), 200)))
    print(_fmt("_load_rtcore_metrics_snapshot", _time(lambda: backend._load_rtcore_metrics_snapshot(), 200)))
    print(_fmt("_canonical_joint_positions_from_raw_feedback", _time(
        lambda: backend._canonical_joint_positions_from_raw_feedback(
            backend.sync_read_positions(), reference_mode="raw"
        ), 200
    )))
    return 0


if __name__ == "__main__":
    sys.path.insert(0, "/home/pi/GradientOS/src")
    sys.exit(main())
