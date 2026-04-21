#!/usr/bin/env python3
"""Jog-time canonical-truth gate tracer.

Polls /info/joints-detailed at high rate and logs every axis-level
gate diagnostic (shaft-frame + command-roundtrip) to stderr. Flags
transitions (available→unavailable, source switches, etc.) so the
anomaly window is easy to find in a long capture.

Usage:
    python3 scripts/jog_gate_trace.py [--duration 120] [--hz 20]
"""
from __future__ import annotations

import argparse
import json
import sys
import time
import urllib.request


def _fetch(url: str) -> dict | None:
    try:
        with urllib.request.urlopen(url, timeout=0.25) as r:
            return json.load(r)
    except Exception:
        return None


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=120.0)
    parser.add_argument("--hz", type=float, default=20.0)
    parser.add_argument(
        "--url",
        default="http://127.0.0.1:4400/info/joints-detailed",
    )
    args = parser.parse_args()

    period = 1.0 / args.hz
    start = time.monotonic()
    sample_idx = 0
    prev_truth: bool | None = None
    prev_per_axis_consistent: dict[int, bool] = {}
    prev_per_axis_rt_consistent: dict[int, bool] = {}
    prev_per_axis_source: dict[int, str] = {}
    prev_rt_source: dict[int, str] = {}
    flicker_count = 0
    max_abs_shaft_delta = [0.0] * 6
    max_abs_rt_err_rad = [0.0] * 6
    max_abs_velocity = [0.0] * 6

    print(
        f"# jog_gate_trace start @ {time.strftime('%H:%M:%S')} "
        f"duration={args.duration}s hz={args.hz} url={args.url}",
        flush=True,
    )
    print(
        "# columns: t_s sample ax truth_avail rsn sf_src sf_c sf_delta sf_tol sf_vel "
        "rt_src rt_c rt_err rt_tol rt_vel raw",
        flush=True,
    )

    while time.monotonic() - start < args.duration:
        tick_start = time.monotonic()
        sample_idx += 1
        t_s = tick_start - start
        data = _fetch(args.url)
        if data is None:
            time.sleep(period)
            continue
        truth_available = bool(data.get("canonical_joint_truth_available", False))
        unavailable_joints = data.get("display_joint_truth_unavailable_joints") or []
        if prev_truth is not None and prev_truth != truth_available:
            flicker_count += 1
            print(
                f"!FLIP t={t_s:.3f}s truth_available {prev_truth}→{truth_available} "
                f"unavailable_joints={unavailable_joints}",
                flush=True,
            )
        prev_truth = truth_available

        axes = data.get("axis_absolute_feedback") or []
        for ax in axes:
            ai = ax.get("axis")
            if ai is None or ai >= 6:
                continue
            sf_src = ax.get("shaft_frame_reference_source")
            sf_c = ax.get("shaft_frame_consistent")
            sf_delta = ax.get("shaft_frame_mod_rm_delta_counts")
            sf_tol = ax.get("shaft_frame_tolerance_counts")
            sf_tol_motion = ax.get("shaft_frame_tolerance_motion_widen_counts")
            sf_vel = ax.get("shaft_frame_velocity_counts_per_s")
            rt_src = ax.get("command_roundtrip_reference_source")
            rt_c = ax.get("command_roundtrip_consistent")
            rt_err = ax.get("command_roundtrip_reference_error_rad")
            rt_tol = ax.get("command_roundtrip_tolerance_rad")
            rt_vel = ax.get("command_roundtrip_velocity_counts_per_s")
            raw_counts = ax.get("raw_counts")
            truth_reason = ax.get("truth_reason") or ""

            if sf_delta is not None and abs(float(sf_delta)) > max_abs_shaft_delta[ai]:
                max_abs_shaft_delta[ai] = abs(float(sf_delta))
            if rt_err is not None and abs(float(rt_err)) > max_abs_rt_err_rad[ai]:
                max_abs_rt_err_rad[ai] = abs(float(rt_err))
            if sf_vel is not None and abs(float(sf_vel)) > max_abs_velocity[ai]:
                max_abs_velocity[ai] = abs(float(sf_vel))

            def _fmt(val, spec: str = "") -> str:
                if val is None:
                    return "None"
                try:
                    return format(val, spec) if spec else str(val)
                except Exception:
                    return str(val)

            prev_sf_c = prev_per_axis_consistent.get(ai)
            prev_rt_c = prev_per_axis_rt_consistent.get(ai)
            prev_sfs = prev_per_axis_source.get(ai)
            prev_rts = prev_rt_source.get(ai)
            anomaly = False
            if prev_sf_c is not None and prev_sf_c != sf_c:
                anomaly = True
                print(
                    f"!SHAFT_FLIP t={t_s:.3f}s ax{ai} {prev_sf_c}→{sf_c} "
                    f"delta={sf_delta} tol={sf_tol} (base+mot={sf_tol_motion}) vel={sf_vel} "
                    f"reason={truth_reason}",
                    flush=True,
                )
            if prev_rt_c is not None and prev_rt_c != rt_c:
                anomaly = True
                print(
                    f"!RT_FLIP t={t_s:.3f}s ax{ai} {prev_rt_c}→{rt_c} "
                    f"err={rt_err} tol={rt_tol} vel={rt_vel} reason={truth_reason}",
                    flush=True,
                )
            if prev_sfs is not None and prev_sfs != sf_src:
                anomaly = True
                print(f"!SFSRC t={t_s:.3f}s ax{ai} shaft_src {prev_sfs}→{sf_src}", flush=True)
            if prev_rts is not None and prev_rts != rt_src:
                anomaly = True
                print(f"!RTSRC t={t_s:.3f}s ax{ai} rt_src {prev_rts}→{rt_src}", flush=True)
            # Always print when anomaly, or every 40 samples per axis as a baseline.
            if anomaly or (sample_idx % 40 == 0 and ai == 0):
                print(
                    f"t={t_s:7.3f} s={sample_idx:4d} ax{ai} "
                    f"tr={truth_available!s:>5} rsn={truth_reason} "
                    f"sf[{sf_src} c={sf_c} d={_fmt(sf_delta)} tol={_fmt(sf_tol)} vel={_fmt(sf_vel, '.0f')}] "
                    f"rt[{rt_src} c={rt_c} err={_fmt(rt_err, '.6f')} tol={_fmt(rt_tol, '.6f')} vel={_fmt(rt_vel, '.0f')}] "
                    f"raw={raw_counts}",
                    flush=True,
                )
            prev_per_axis_consistent[ai] = sf_c
            prev_per_axis_rt_consistent[ai] = rt_c
            prev_per_axis_source[ai] = sf_src
            prev_rt_source[ai] = rt_src

        elapsed = time.monotonic() - tick_start
        sleep_s = period - elapsed
        if sleep_s > 0:
            time.sleep(sleep_s)

    print("# === SUMMARY ===", flush=True)
    print(f"# total_samples={sample_idx}", flush=True)
    print(f"# top_level_truth_flips={flicker_count}", flush=True)
    for i in range(6):
        print(
            f"# ax{i} max|shaft_delta|={max_abs_shaft_delta[i]:.1f} counts "
            f"max|rt_err|={max_abs_rt_err_rad[i]:.6f} rad "
            f"max|velocity|={max_abs_velocity[i]:.0f} counts/s",
            flush=True,
        )
    return 0


if __name__ == "__main__":
    sys.exit(main())
