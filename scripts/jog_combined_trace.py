#!/usr/bin/env python3
"""Combined jog-time trace: gate diagnostics + jog-loop stage timings.

Polls two endpoints concurrently:
- /info/joints-detailed (canonical-truth gate state per axis)
- /debug/performance    (jog loop + stage timings, when jog is active)

Logs any gate anomaly immediately, and a single stage-timing snapshot
every time the jog loop counter increments (= one jog session active).
"""
from __future__ import annotations

import argparse
import json
import sys
import time
import urllib.request


def _fetch(url: str, timeout: float = 0.4) -> dict | None:
    try:
        with urllib.request.urlopen(url, timeout=timeout) as r:
            return json.load(r)
    except Exception:
        return None


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=120.0)
    parser.add_argument("--hz", type=float, default=20.0)
    args = parser.parse_args()

    start = time.monotonic()
    period = 1.0 / args.hz
    sample_idx = 0
    prev_gate_state: dict[int, tuple[bool | None, bool | None, str | None, str | None]] = {}
    prev_truth: bool | None = None
    prev_jog_count = 0
    max_feedback_ms = 0.0
    max_loop_ms = 0.0

    def _fmt(val, spec: str = "") -> str:
        if val is None:
            return "None"
        try:
            return format(val, spec) if spec else str(val)
        except Exception:
            return str(val)

    print(
        f"# jog_combined_trace start @ {time.strftime('%H:%M:%S')} "
        f"duration={args.duration}s hz={args.hz}",
        flush=True,
    )

    while time.monotonic() - start < args.duration:
        tick_start = time.monotonic()
        sample_idx += 1
        t_s = tick_start - start

        gates = _fetch("http://127.0.0.1:4400/info/joints-detailed")
        perf = _fetch("http://127.0.0.1:4400/debug/performance")

        if gates is not None:
            truth = bool(gates.get("canonical_joint_truth_available", False))
            if prev_truth is not None and prev_truth != truth:
                print(
                    f"!TRUTH t={t_s:.3f}s {prev_truth}→{truth} "
                    f"unavailable_joints={gates.get('display_joint_truth_unavailable_joints')}",
                    flush=True,
                )
            prev_truth = truth
            for ax in gates.get("axis_absolute_feedback", []):
                ai = ax.get("axis")
                if ai is None or ai >= 6:
                    continue
                sf_src = ax.get("shaft_frame_reference_source")
                sf_c = ax.get("shaft_frame_consistent")
                sf_d = ax.get("shaft_frame_mod_rm_delta_counts")
                sf_tol = ax.get("shaft_frame_tolerance_counts")
                sf_vel = ax.get("shaft_frame_velocity_counts_per_s")
                rt_src = ax.get("command_roundtrip_reference_source")
                rt_c = ax.get("command_roundtrip_consistent")
                rt_err = ax.get("command_roundtrip_reference_error_rad")
                rt_vel = ax.get("command_roundtrip_velocity_counts_per_s")
                truth_reason = ax.get("truth_reason") or ""
                prev = prev_gate_state.get(ai)
                if prev is not None:
                    psf_c, prt_c, psf_src, prt_src = prev
                    if psf_c is not None and psf_c != sf_c:
                        print(
                            f"!SHAFT ax{ai} t={t_s:.3f}s {psf_c}→{sf_c} "
                            f"d={_fmt(sf_d, '.1f')}/tol={_fmt(sf_tol, '.0f')} "
                            f"vel={_fmt(sf_vel, '.0f')} src={sf_src} rsn={truth_reason}",
                            flush=True,
                        )
                    if prt_c is not None and prt_c != rt_c:
                        print(
                            f"!RT    ax{ai} t={t_s:.3f}s {prt_c}→{rt_c} "
                            f"err={_fmt(rt_err, '.6f')} vel={_fmt(rt_vel, '.0f')} "
                            f"src={rt_src} rsn={truth_reason}",
                            flush=True,
                        )
                    if psf_src is not None and psf_src != sf_src:
                        print(f"!SFSRC ax{ai} t={t_s:.3f}s {psf_src}→{sf_src}", flush=True)
                    if prt_src is not None and prt_src != rt_src:
                        print(f"!RTSRC ax{ai} t={t_s:.3f}s {prt_src}→{rt_src}", flush=True)
                prev_gate_state[ai] = (sf_c, rt_c, sf_src, rt_src)

        if perf is not None:
            controller = perf.get("controller") or {}
            jog = controller.get("jog") or {}
            loop = jog.get("loop") or {}
            stages = jog.get("stages") or {}
            loop_count = int(loop.get("count") or 0)
            if loop_count > prev_jog_count:
                fb = stages.get("feedback_read_ms") or {}
                ik = stages.get("ik_solve_ms") or {}
                cs = stages.get("command_send_ms") or {}
                if float(fb.get("max_ms") or 0) > max_feedback_ms:
                    max_feedback_ms = float(fb.get("max_ms") or 0)
                if float(loop.get("max_ms") or 0) > max_loop_ms:
                    max_loop_ms = float(loop.get("max_ms") or 0)
                if sample_idx % 10 == 0 or loop_count - prev_jog_count > 1:
                    print(
                        f"JOG t={t_s:6.2f}s loop#{loop_count} loop={_fmt(loop.get('last_ms'), '.1f')}ms "
                        f"(avg={_fmt(loop.get('avg_ms'), '.1f')} max={_fmt(loop.get('max_ms'), '.1f')} "
                        f"ovr={loop.get('overrun_count')}) "
                        f"fb={_fmt(fb.get('last_ms'), '.1f')}/{_fmt(fb.get('avg_ms'), '.1f')}/{_fmt(fb.get('max_ms'), '.1f')} "
                        f"ik={_fmt(ik.get('last_ms'), '.1f')}/{_fmt(ik.get('avg_ms'), '.1f')}/{_fmt(ik.get('max_ms'), '.1f')} "
                        f"cs={_fmt(cs.get('last_ms'), '.1f')}/{_fmt(cs.get('avg_ms'), '.1f')}/{_fmt(cs.get('max_ms'), '.1f')}",
                        flush=True,
                    )
            prev_jog_count = loop_count

        elapsed = time.monotonic() - tick_start
        sleep = period - elapsed
        if sleep > 0:
            time.sleep(sleep)

    print("# === SUMMARY ===", flush=True)
    print(f"# max_feedback_ms across session: {max_feedback_ms:.2f} ms", flush=True)
    print(f"# max_loop_ms across session: {max_loop_ms:.2f} ms", flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
