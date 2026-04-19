#!/usr/bin/env python3
"""High-frequency multi-turn capture + analysis for J6 seam-crossing verification.

Why this exists:
  Earlier seam-crossing PASS verdicts were based on endpoint-only U40.20/U40.22
  reads; a full 360 deg whip (forward +360 then back -350, net +10) is
  indistinguishable from a clean +10 deg move when only the endpoints are
  sampled. This tool provides dense (1 kHz-capable) capture of the 0x6064
  single-turn feedback plus 0x607A target plus U40.20/U40.22 multi-turn
  register across a single trajectory, so the whole motion path can be judged.

Primary path (preferred) - `analyze-rtcore`:
  RTCore has a built-in `fast_trace_thread` (see `src/gradient_rt_motion/main.cpp`
  around line 4383) that writes per-cycle feedback to a JSONL at up to the
  RTCore cycle rate (1 kHz on this stack) WITHOUT any userspace SDO traffic.
  Enable it via a systemd drop-in:
    /etc/systemd/system/gradient-rt-motion.service.d/fast-trace.conf:
      [Service]
      Environment=GRADIENT_RT_FAST_TRACE_PATH=/run/gradient-rt-motion/j6-fast-trace.jsonl
      Environment=GRADIENT_RT_FAST_TRACE_HZ=1000
      Environment=GRADIENT_RT_FAST_TRACE_AXIS_MASK=0x20
    sudo systemctl daemon-reload
  Start the stack; RTCore will stream samples at 1 kHz. Copy the file out
  before `./start-stack.sh stop` (the runtime dir is wiped on service stop).
  Then run:
    python scripts/j6_multiturn_fast_capture.py analyze-rtcore <file.jsonl>

Fallback path (deprecated) - `capture` + `analyze`:
  If RTCore fast_trace is unavailable (e.g. old binary), a userspace SDO loop
  via `sudo -n ethercat upload` can sample at ~5-15 Hz depending on the number
  of registers. This is almost always too slow to detect a sub-200 ms whip at
  reasonable motion speeds; use only for static bus-state snapshots.

Commands:
  analyze-rtcore  - post-process a RTCore fast_trace JSONL (preferred)
  capture         - userspace SDO sample loop (deprecated, ~5-15 Hz ceiling)
  analyze         - post-process a capture JSONL from this script's own format

See /home/pi/.cursor/plans/j6_seam_whip_verification_b8c230f3.plan.md Phase 2.
"""

from __future__ import annotations

import argparse
import dataclasses
import datetime as dt
import json
import os
import signal
import subprocess
import sys
import threading
import time
from pathlib import Path
from typing import Any, Iterable

COUNTS_PER_MOTOR_REV = 131072  # 17-bit encoder, per Chapter 5
DEFAULT_J6_AXIS_INDEX = 5       # zero-based slave position
DEFAULT_J6_GEAR_RATIO = 10.0    # A6-EC J6 C10.18/C10.19 numerator/denominator
DEFAULT_LOG_DIR = Path("logs/j6-multiturn-fast")

# 0x6064 on A6-EC is in rotation-mode reference frame AFTER the vendor
# gear ratio (C10.18/C10.19), wrapping at `counts_per_motor_rev * gear`.
# For J6 that is 131072 * 10 = 1,310,720.
DEFAULT_J6_RM_COUNTS = int(COUNTS_PER_MOTOR_REV * DEFAULT_J6_GEAR_RATIO)

# Per-sample SDO read descriptors. Keep this small; each entry is one
# subprocess per sample. See docs/ethercat/a6ec-manufacturer-notes-2026-04-15.md
# for vendor semantics on each object.
#
# Tuple: (field_key, object_index, subindex, data_type, width_bytes,
#         signed, is_primary_truth)
SDO_DESCRIPTORS: tuple[tuple[str, str, str, str, int, bool, bool], ...] = (
    ("u40_20_i32_low",  "0x2040", "0x21", "int32",  4, True,  True),
    ("u40_22_i32_high", "0x2040", "0x23", "int32",  4, True,  True),
    ("c6064",           "0x6064", "0x00", "int32",  4, True,  False),
    ("c607A",           "0x607A", "0x00", "int32",  4, True,  False),
    ("c603F",           "0x603F", "0x00", "uint16", 2, False, False),
    ("c6041",           "0x6041", "0x00", "uint16", 2, False, False),
)

# Minimal-truth fallback: drop 6064/607A to cut per-sample cost in half.
# Kept narrow: we still need 603F/6041 to halt on fault even in minimal mode.
MINIMAL_DESCRIPTORS: tuple[tuple[str, str, str, str, int, bool, bool], ...] = (
    ("u40_20_i32_low",  "0x2040", "0x21", "int32",  4, True,  True),
    ("u40_22_i32_high", "0x2040", "0x23", "int32",  4, True,  True),
    ("c603F",           "0x603F", "0x00", "uint16", 2, False, False),
    ("c6041",           "0x6041", "0x00", "uint16", 2, False, False),
)


# =========================================================================
# Helpers
# =========================================================================

def _utc_now() -> dt.datetime:
    return dt.datetime.now(dt.timezone.utc)


def _iso_token(moment: dt.datetime | None = None) -> str:
    current = moment or _utc_now()
    return current.strftime("%Y%m%dT%H%M%SZ")


def _normalize_label(value: str) -> str:
    token = "".join(
        ch if ch.isalnum() or ch in {"-", "_", "."} else "-" for ch in value.strip()
    )
    token = token.strip("-._")
    if not token:
        raise ValueError("label must contain at least one safe filename character")
    return token


def _combine_signed_i64(low_i32: int | None, high_i32: int | None) -> int | None:
    if low_i32 is None or high_i32 is None:
        return None
    low = int(low_i32) & 0xFFFFFFFF
    high = int(high_i32) & 0xFFFFFFFF
    combined = (high << 32) | low
    if high & 0x80000000:
        combined -= 1 << 64
    return combined


def _statusword_is_fault(statusword_u16: int | None) -> bool:
    if statusword_u16 is None:
        return False
    # DS402 Fault state pattern: (SW & 0x004F) == 0x0008
    return (int(statusword_u16) & 0x004F) == 0x0008


# =========================================================================
# Fast SDO reader
# =========================================================================

@dataclasses.dataclass
class SdoRead:
    key: str
    value: int | None
    ok: bool
    raw: str
    error: str | None


def _parse_ethercat_value(raw: str, signed: bool, width_bytes: int) -> int | None:
    """Parse `ethercat upload` output. Tolerant of the two output shapes
    observed in practice:
      - `0x00001234 4660`  (hex + decimal)
      - `4660`              (decimal only)
    """
    tokens = raw.strip().split()
    if not tokens:
        return None
    # Prefer the last token (always decimal in both shapes)
    try:
        value = int(tokens[-1], 0)
    except ValueError:
        return None
    # Sign-extend if the CLI returned an unsigned interpretation.
    if signed and width_bytes == 4 and value >= (1 << 31):
        value -= 1 << 32
    elif signed and width_bytes == 2 and value >= (1 << 15):
        value -= 1 << 16
    return value


def _spawn_sdo_read(
    axis_index: int,
    index: str,
    subindex: str,
    data_type: str,
) -> subprocess.Popen[bytes]:
    return subprocess.Popen(
        [
            "sudo",
            "-n",
            "ethercat",
            "upload",
            "-p",
            str(axis_index),
            "-t",
            data_type,
            index,
            subindex,
        ],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
    )


def _read_sdo_batch_parallel(
    axis_index: int,
    descriptors: Iterable[tuple[str, str, str, str, int, bool, bool]],
) -> dict[str, SdoRead]:
    """Issue all SDO reads for a single sample concurrently, wait for all,
    return {key: SdoRead}.
    """
    pending: list[tuple[str, subprocess.Popen[bytes], str, int, bool]] = []
    for key, index, sub, dtype, width, signed, _primary in descriptors:
        proc = _spawn_sdo_read(axis_index, index, sub, dtype)
        pending.append((key, proc, dtype, width, signed))

    out: dict[str, SdoRead] = {}
    for key, proc, _dtype, width, signed in pending:
        try:
            stdout, stderr = proc.communicate(timeout=2.0)
        except subprocess.TimeoutExpired:
            proc.kill()
            stdout, stderr = b"", b"ethercat upload timed out"
        raw = (stdout or b"").decode("utf-8", "replace").strip()
        err = (stderr or b"").decode("utf-8", "replace").strip()
        if proc.returncode != 0:
            out[key] = SdoRead(key=key, value=None, ok=False, raw=raw, error=err or f"rc={proc.returncode}")
            continue
        value = _parse_ethercat_value(raw, signed=signed, width_bytes=width)
        out[key] = SdoRead(
            key=key,
            value=value,
            ok=value is not None,
            raw=raw,
            error=None if value is not None else "could not parse ethercat upload output",
        )
    return out


# =========================================================================
# Capture loop
# =========================================================================

@dataclasses.dataclass
class CaptureOptions:
    label: str
    axis_index: int
    duration_s: float
    log_dir: Path
    descriptors: tuple[tuple[str, str, str, str, int, bool, bool], ...]
    halt_on_fault: bool
    note: str | None


def _run_capture(options: CaptureOptions) -> dict[str, Any]:
    options.log_dir.mkdir(parents=True, exist_ok=True)
    iso = _iso_token()
    base = f"{_normalize_label(options.label)}-{iso}"
    jsonl_path = options.log_dir / f"{base}.jsonl"
    meta_path = options.log_dir / f"{base}.meta.json"

    stop_flag = threading.Event()

    def _on_sig(_signum: int, _frame: Any) -> None:
        stop_flag.set()

    signal.signal(signal.SIGINT, _on_sig)
    signal.signal(signal.SIGTERM, _on_sig)

    start_monotonic_ns = time.monotonic_ns()
    start_wall = _utc_now().isoformat()
    deadline_ns = start_monotonic_ns + int(options.duration_s * 1e9)

    sample_count = 0
    fault_halt_reason: str | None = None
    first_mt: int | None = None
    last_mt: int | None = None

    print(
        f"[capture] label={options.label} axis={options.axis_index} "
        f"duration={options.duration_s:.1f}s jsonl={jsonl_path}",
        flush=True,
    )

    with jsonl_path.open("w", encoding="utf-8") as fh:
        while not stop_flag.is_set():
            now_ns = time.monotonic_ns()
            if now_ns >= deadline_ns:
                break

            reads = _read_sdo_batch_parallel(options.axis_index, options.descriptors)
            # Primary truth: combine U40.20 + U40.22 as signed i64.
            low = reads.get("u40_20_i32_low")
            high = reads.get("u40_22_i32_high")
            mt_i64 = _combine_signed_i64(
                low.value if low and low.ok else None,
                high.value if high and high.ok else None,
            )
            c603F = reads.get("c603F")
            c6041 = reads.get("c6041")
            c6064 = reads.get("c6064")
            c607A = reads.get("c607A")

            if first_mt is None and mt_i64 is not None:
                first_mt = mt_i64
            if mt_i64 is not None:
                last_mt = mt_i64

            sample = {
                "t_mono_ns": now_ns - start_monotonic_ns,
                "mt_i64": mt_i64,
                "c6064": c6064.value if c6064 and c6064.ok else None,
                "c607A": c607A.value if c607A and c607A.ok else None,
                "c603F": c603F.value if c603F and c603F.ok else None,
                "c6041": c6041.value if c6041 and c6041.ok else None,
                "reads": {k: dataclasses.asdict(v) for k, v in reads.items()},
            }
            fh.write(json.dumps(sample, separators=(",", ":")))
            fh.write("\n")
            sample_count += 1

            if options.halt_on_fault:
                code = c603F.value if c603F and c603F.ok else None
                if code is not None and int(code) != 0:
                    fault_halt_reason = f"603F=0x{int(code):04X} (non-zero)"
                    print(f"[capture] HALT {fault_halt_reason}", flush=True)
                    break
                sw = c6041.value if c6041 and c6041.ok else None
                if _statusword_is_fault(sw):
                    fault_halt_reason = f"6041=0x{int(sw):04X} (DS402 Fault)"
                    print(f"[capture] HALT {fault_halt_reason}", flush=True)
                    break

    end_monotonic_ns = time.monotonic_ns()
    end_wall = _utc_now().isoformat()
    elapsed_ns = end_monotonic_ns - start_monotonic_ns
    elapsed_s = elapsed_ns / 1e9
    effective_hz = (sample_count / elapsed_s) if elapsed_s > 0 else 0.0

    meta: dict[str, Any] = {
        "schema_version": 1,
        "tool": "j6_multiturn_fast_capture",
        "label": options.label,
        "axis_index": options.axis_index,
        "start_wall_utc": start_wall,
        "end_wall_utc": end_wall,
        "start_monotonic_ns": start_monotonic_ns,
        "end_monotonic_ns": end_monotonic_ns,
        "duration_cap_s": options.duration_s,
        "elapsed_s": elapsed_s,
        "sample_count": sample_count,
        "effective_hz": effective_hz,
        "fault_halt_reason": fault_halt_reason,
        "descriptor_keys": [d[0] for d in options.descriptors],
        "jsonl_path": str(jsonl_path),
        "first_mt_i64": first_mt,
        "last_mt_i64": last_mt,
        "net_mt_delta_counts": (last_mt - first_mt) if (first_mt is not None and last_mt is not None) else None,
        "note": options.note,
    }
    meta_path.write_text(json.dumps(meta, indent=2))
    print(
        f"[capture] done samples={sample_count} "
        f"elapsed={elapsed_s:.2f}s hz={effective_hz:.1f} "
        f"meta={meta_path}",
        flush=True,
    )
    return meta


# =========================================================================
# Post-processor
# =========================================================================

@dataclasses.dataclass
class AnalysisResult:
    jsonl_path: Path
    sample_count: int
    elapsed_s: float
    effective_hz: float
    first_mt: int | None
    last_mt: int | None
    net_mt_delta: int | None
    cumulative_travel: int
    max_sample_abs_delta: int
    ratio_cumulative_over_abs_net: float | None
    monotonic_within_budget: bool
    overshoot_budget_counts: int
    fault_seen: bool
    fault_sample_index: int | None
    fault_detail: str | None


def _analyze_jsonl(
    jsonl_path: Path,
    *,
    overshoot_budget_counts: int = 500,
) -> AnalysisResult:
    samples: list[dict[str, Any]] = []
    with jsonl_path.open("r", encoding="utf-8") as fh:
        for line in fh:
            line = line.strip()
            if not line:
                continue
            try:
                samples.append(json.loads(line))
            except json.JSONDecodeError:
                continue
    if not samples:
        raise ValueError(f"no samples in {jsonl_path}")

    mts: list[int] = [s["mt_i64"] for s in samples if s.get("mt_i64") is not None]
    if not mts:
        raise ValueError(f"no valid mt_i64 samples in {jsonl_path}")

    first_mt = mts[0]
    last_mt = mts[-1]
    net = last_mt - first_mt

    cumulative = 0
    max_abs_delta = 0
    monotonic = True
    direction = 0  # +1 / -1 once determined
    for a, b in zip(mts[:-1], mts[1:]):
        delta = b - a
        cumulative += abs(delta)
        max_abs_delta = max(max_abs_delta, abs(delta))
        if direction == 0 and abs(delta) > overshoot_budget_counts:
            direction = 1 if delta > 0 else -1
        elif direction != 0 and (delta * direction) < -overshoot_budget_counts:
            monotonic = False

    elapsed_s = samples[-1]["t_mono_ns"] / 1e9 if samples else 0.0
    hz = (len(samples) / elapsed_s) if elapsed_s > 0 else 0.0

    # Find first fault (603F != 0) if present.
    fault_seen = False
    fault_sample_index: int | None = None
    fault_detail: str | None = None
    for idx, s in enumerate(samples):
        code = s.get("c603F")
        sw = s.get("c6041")
        if code is not None and int(code) != 0:
            fault_seen = True
            fault_sample_index = idx
            fault_detail = f"603F=0x{int(code):04X} at sample {idx}"
            break
        if _statusword_is_fault(sw):
            fault_seen = True
            fault_sample_index = idx
            fault_detail = f"6041=0x{int(sw):04X} (DS402 Fault) at sample {idx}"
            break

    ratio: float | None = None
    if abs(net) > 0:
        ratio = cumulative / abs(net)

    return AnalysisResult(
        jsonl_path=jsonl_path,
        sample_count=len(samples),
        elapsed_s=elapsed_s,
        effective_hz=hz,
        first_mt=first_mt,
        last_mt=last_mt,
        net_mt_delta=net,
        cumulative_travel=cumulative,
        max_sample_abs_delta=max_abs_delta,
        ratio_cumulative_over_abs_net=ratio,
        monotonic_within_budget=monotonic,
        overshoot_budget_counts=overshoot_budget_counts,
        fault_seen=fault_seen,
        fault_sample_index=fault_sample_index,
        fault_detail=fault_detail,
    )


def _counts_to_motor_revs(counts: int) -> float:
    return float(counts) / float(COUNTS_PER_MOTOR_REV)


def _counts_to_output_deg(counts: int, *, gear_ratio: float = 10.0) -> float:
    """Convert motor counts to joint output angle in degrees.

    Plan: J6 gear ratio is 10. `COUNTS_PER_MOTOR_REV = 131072`. Output deg is
    counts / (COUNTS_PER_MOTOR_REV * gear_ratio) * 360.
    """
    return (float(counts) / (float(COUNTS_PER_MOTOR_REV) * float(gear_ratio))) * 360.0


def _plot_trace_png(
    jsonl_path: Path,
    result: AnalysisResult,
    *,
    gear_ratio: float,
) -> Path | None:
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt  # type: ignore
    except Exception:
        return None

    samples: list[dict[str, Any]] = []
    with jsonl_path.open("r", encoding="utf-8") as fh:
        for line in fh:
            line = line.strip()
            if not line:
                continue
            try:
                samples.append(json.loads(line))
            except json.JSONDecodeError:
                continue
    if not samples:
        return None
    ts = [s["t_mono_ns"] / 1e9 for s in samples if s.get("mt_i64") is not None]
    mts = [s["mt_i64"] for s in samples if s.get("mt_i64") is not None]
    if not ts:
        return None

    first_mt = mts[0]
    rel_deg = [_counts_to_output_deg(mt - first_mt, gear_ratio=gear_ratio) for mt in mts]

    fig, ax = plt.subplots(figsize=(10, 5))
    ax.plot(ts, rel_deg, linewidth=1.5, color="#1f77b4")
    ax.set_xlabel("time since capture start (s)")
    ax.set_ylabel("J6 multi-turn displacement (deg output, relative to first sample)")
    title = (
        f"{jsonl_path.stem}\n"
        f"samples={result.sample_count} hz={result.effective_hz:.1f} "
        f"net={_counts_to_output_deg(result.net_mt_delta or 0, gear_ratio=gear_ratio):+.2f} deg "
        f"cum/abs(net)={result.ratio_cumulative_over_abs_net:.2f}"
        if result.ratio_cumulative_over_abs_net is not None
        else f"{jsonl_path.stem}\nsamples={result.sample_count} hz={result.effective_hz:.1f}"
    )
    ax.set_title(title)
    ax.grid(True, alpha=0.3)
    ax.axhline(0.0, linestyle=":", color="gray", alpha=0.6)
    if result.fault_seen and result.fault_sample_index is not None:
        fault_t = samples[result.fault_sample_index]["t_mono_ns"] / 1e9
        ax.axvline(fault_t, color="red", alpha=0.6, linewidth=1.0, label="fault")
        ax.legend(loc="upper left")

    out_path = jsonl_path.with_suffix(".png")
    fig.tight_layout()
    fig.savefig(out_path, dpi=120)
    plt.close(fig)
    return out_path


def _print_analysis_summary(result: AnalysisResult, *, gear_ratio: float) -> None:
    net = result.net_mt_delta or 0
    cum = result.cumulative_travel
    net_deg = _counts_to_output_deg(net, gear_ratio=gear_ratio)
    cum_deg = _counts_to_output_deg(cum, gear_ratio=gear_ratio)
    max_delta_deg = _counts_to_output_deg(result.max_sample_abs_delta, gear_ratio=gear_ratio)
    print(f"=== Analysis: {result.jsonl_path.name} ===")
    print(f"  samples            : {result.sample_count}")
    print(f"  elapsed_s          : {result.elapsed_s:.3f}")
    print(f"  effective_hz       : {result.effective_hz:.2f}")
    print(f"  first_mt_i64       : {result.first_mt}")
    print(f"  last_mt_i64        : {result.last_mt}")
    print(f"  net_mt_delta       : {net} counts ({net_deg:+.3f} deg output)")
    print(f"  cumulative_travel  : {cum} counts ({cum_deg:.3f} deg output)")
    print(f"  max_sample_delta   : {result.max_sample_abs_delta} counts ({max_delta_deg:.3f} deg output)")
    if result.ratio_cumulative_over_abs_net is not None:
        print(f"  cum/|net| ratio    : {result.ratio_cumulative_over_abs_net:.3f}")
        if result.ratio_cumulative_over_abs_net > 1.2:
            print("  VERDICT            : WHIP (cumulative travel > 1.2 x |net|)")
        else:
            print("  VERDICT            : CLEAN (cumulative travel <= 1.2 x |net|)")
    else:
        print("  cum/|net| ratio    : undefined (net==0)")
    print(f"  monotonic          : {result.monotonic_within_budget} "
          f"(budget={result.overshoot_budget_counts} counts)")
    if result.fault_seen:
        print(f"  fault_seen         : True -- {result.fault_detail}")
    else:
        print("  fault_seen         : False")


# =========================================================================
# RTCore fast_trace JSONL analyzer
# =========================================================================

# Each line of the RTCore fast_trace JSONL looks like:
#   {"t_ns":165648317449778,"seq":185818,
#    "ax":[{"i":5,"p":655401,"tp":655401,"sw":38480,"er":0,"mfr":0,
#           "af":[{"k":"absolute_position_reference","v":...,"ok":1},
#                 {"k":"encoder_single_turn_data","v":...,"ok":1},
#                 {"k":"encoder_multi_turn_position","v":...,"ok":1},
#                 {"k":"encoder_multi_turn_low","v":...,"ok":1},
#                 {"k":"encoder_multi_turn_high","v":...,"ok":1},
#                 {"k":"rotation_mode_position_reference","v":...,"ok":1},
#                 {"k":"rotation_mode_encoder_low","v":...,"ok":1},
#                 {"k":"rotation_mode_encoder_high","v":...,"ok":1}]}]}
#
# PDO fields (p, tp, sw, er, mfr) update every 1 ms cycle.
# `af` (absolute_feedback) fields are refreshed by the metrics-thread
# SDO poll at kAbsoluteFeedbackPollIntervalNs (default 200 ms = 5 Hz), so
# consecutive fast_trace lines will show identical af values until the
# next poll.

def _unwrap_wire_delta(delta: int, rm_counts: int) -> int:
    """Convert a raw wire-frame 6064 delta into the shortest-periodic delta.

    If the raw delta is larger than RM/2 in either direction, the drive has
    wrapped around [0, RM): correct it back into [-RM/2, RM/2].
    """
    if rm_counts <= 0:
        return int(delta)
    half = rm_counts // 2
    d = int(delta)
    if d > half:
        d -= rm_counts
    elif d < -half:
        d += rm_counts
    return d


def _extract_axis_sample(
    record: dict,
    axis_index: int,
) -> dict | None:
    """Given one RTCore fast_trace JSONL record, return the extracted per-axis
    fields for `axis_index`, or None if that axis is not in the record."""
    ax_list = record.get("ax")
    if not isinstance(ax_list, list):
        return None
    axis_entry: dict | None = None
    for entry in ax_list:
        if isinstance(entry, dict) and int(entry.get("i", -1)) == int(axis_index):
            axis_entry = entry
            break
    if axis_entry is None:
        return None
    af = axis_entry.get("af") or []
    mt_low = mt_high = None
    mt_low_ok = mt_high_ok = False
    for field in af:
        if not isinstance(field, dict):
            continue
        k = field.get("k")
        v = field.get("v")
        ok = int(field.get("ok", 0))
        if k == "encoder_multi_turn_low":
            mt_low = int(v) if v is not None else None
            mt_low_ok = bool(ok)
        elif k == "encoder_multi_turn_high":
            mt_high = int(v) if v is not None else None
            mt_high_ok = bool(ok)
    mt_i64 = _combine_signed_i64(mt_low, mt_high) if (mt_low_ok and mt_high_ok) else None
    return {
        "t_ns": int(record.get("t_ns", 0)),
        "seq": int(record.get("seq", 0)),
        "p": int(axis_entry.get("p", 0)),
        "tp": int(axis_entry.get("tp", 0)),
        "sw": int(axis_entry.get("sw", 0)),
        "er": int(axis_entry.get("er", 0)),
        "mfr": int(axis_entry.get("mfr", 0)),
        "mt_i64": mt_i64,
    }


@dataclasses.dataclass
class FastTraceAnalysis:
    jsonl_path: Path
    axis_index: int
    sample_count: int
    elapsed_s: float
    effective_hz: float
    first_p: int | None
    last_p: int | None
    first_mt_i64: int | None
    last_mt_i64: int | None
    cumulative_travel_wire_counts: int
    net_displacement_wire_counts: int
    net_displacement_shortest_wrap_counts: int
    long_path_excess_counts: int
    max_abs_wire_step_counts: int
    ratio_cumulative_over_abs_net_wire: float | None
    wire_monotonic_within_budget: bool
    overshoot_budget_counts: int
    mt_sample_count: int
    mt_distinct_samples: int
    mt_net_delta: int | None
    fault_seen: bool
    fault_sample_index: int | None
    fault_detail: str | None
    rm_counts: int


def _analyze_rtcore_jsonl(
    jsonl_path: Path,
    *,
    axis_index: int = DEFAULT_J6_AXIS_INDEX,
    rm_counts: int = DEFAULT_J6_RM_COUNTS,
    overshoot_budget_counts: int = 500,
) -> FastTraceAnalysis:
    samples: list[dict] = []
    with jsonl_path.open("r", encoding="utf-8") as fh:
        for line in fh:
            line = line.strip()
            if not line:
                continue
            try:
                record = json.loads(line)
            except json.JSONDecodeError:
                continue
            extracted = _extract_axis_sample(record, axis_index=axis_index)
            if extracted is not None:
                samples.append(extracted)
    if not samples:
        raise ValueError(f"no samples for axis {axis_index} in {jsonl_path}")

    # Drop leading zeros (RTCore writes a few samples before PDO latches).
    first_live = 0
    for i, s in enumerate(samples):
        if s["p"] != 0 or s["sw"] != 0:
            first_live = i
            break
    samples = samples[first_live:]
    if not samples:
        raise ValueError(f"no non-zero samples for axis {axis_index} in {jsonl_path}")

    t0 = samples[0]["t_ns"]
    elapsed_ns = samples[-1]["t_ns"] - t0
    elapsed_s = elapsed_ns / 1e9 if elapsed_ns > 0 else 0.0
    hz = (len(samples) / elapsed_s) if elapsed_s > 0 else 0.0

    # Wire-frame (0x6064) per-cycle deltas, wrap-corrected.
    ps = [s["p"] for s in samples]
    cumulative_wire = 0
    max_abs_wire_step = 0
    monotonic = True
    direction = 0
    for a, b in zip(ps[:-1], ps[1:]):
        d = _unwrap_wire_delta(b - a, rm_counts)
        cumulative_wire += abs(d)
        max_abs_wire_step = max(max_abs_wire_step, abs(d))
        if direction == 0 and abs(d) > overshoot_budget_counts:
            direction = 1 if d > 0 else -1
        elif direction != 0 and (d * direction) < -overshoot_budget_counts:
            monotonic = False

    # Net displacement in wire frame via wrap-aware accumulation (path length
    # net of direction). Each per-sample delta is unwrapped to its shortest
    # equivalent, so this sum captures the true amount and direction of motion
    # even across the 0/RM seam.
    net_wire = 0
    for a, b in zip(ps[:-1], ps[1:]):
        net_wire += _unwrap_wire_delta(b - a, rm_counts)

    # "Long-path detour" check. For a clean rotation-mode seam crossing, net
    # stays within [-RM/2, +RM/2]: that's the shortest path. If net exceeds
    # RM/2 in magnitude, the motor took at least one extra full revolution
    # beyond what the shortest path would require. That's the signature of
    # the 2026-04-19 UI-driven whip where the host's nearest-turn fold
    # flipped the turn count and emitted a long-path target, and the drive
    # faithfully followed it ~360 deg in the wrong direction.
    #
    # Important: this is NOT the same as wire_monotonic. A whip CAN be
    # monotonic (drive goes straight around the long way without flipping
    # direction). That's why wire_monotonic + cum/|net| ≈ 1 is NOT a
    # sufficient CLEAN verdict; we also need the long-path check.
    shortest_net = int(net_wire)
    if rm_counts > 0:
        half = rm_counts // 2
        while shortest_net > half:
            shortest_net -= rm_counts
        while shortest_net < -half:
            shortest_net += rm_counts
    long_path_excess = int(abs(int(net_wire) - shortest_net))

    ratio_wire: float | None = None
    if abs(net_wire) > 0:
        ratio_wire = cumulative_wire / abs(net_wire)

    # Multi-turn cross-check (U40.20/.22 combined signed i64, refreshed at
    # ~5 Hz by the RTCore metrics poll).
    mt_values = [s["mt_i64"] for s in samples if s["mt_i64"] is not None]
    mt_distinct = sorted({v for v in mt_values})
    first_mt = mt_values[0] if mt_values else None
    last_mt = mt_values[-1] if mt_values else None
    mt_net = (last_mt - first_mt) if (first_mt is not None and last_mt is not None) else None

    # First fault sample (by 603F or DS402 Fault statusword).
    fault_seen = False
    fault_sample_index: int | None = None
    fault_detail: str | None = None
    for idx, s in enumerate(samples):
        if s["er"] != 0:
            fault_seen = True
            fault_sample_index = idx
            fault_detail = f"603F=0x{int(s['er']):04X} at sample {idx}"
            break
        if _statusword_is_fault(s["sw"]):
            fault_seen = True
            fault_sample_index = idx
            fault_detail = f"6041=0x{int(s['sw']):04X} (DS402 Fault) at sample {idx}"
            break

    return FastTraceAnalysis(
        jsonl_path=jsonl_path,
        axis_index=axis_index,
        sample_count=len(samples),
        elapsed_s=elapsed_s,
        effective_hz=hz,
        first_p=ps[0] if ps else None,
        last_p=ps[-1] if ps else None,
        first_mt_i64=first_mt,
        last_mt_i64=last_mt,
        cumulative_travel_wire_counts=cumulative_wire,
        net_displacement_wire_counts=net_wire,
        net_displacement_shortest_wrap_counts=shortest_net,
        long_path_excess_counts=long_path_excess,
        max_abs_wire_step_counts=max_abs_wire_step,
        ratio_cumulative_over_abs_net_wire=ratio_wire,
        wire_monotonic_within_budget=monotonic,
        overshoot_budget_counts=overshoot_budget_counts,
        mt_sample_count=len(mt_values),
        mt_distinct_samples=len(mt_distinct),
        mt_net_delta=mt_net,
        fault_seen=fault_seen,
        fault_sample_index=fault_sample_index,
        fault_detail=fault_detail,
        rm_counts=rm_counts,
    )


def _plot_rtcore_trace_png(
    jsonl_path: Path,
    result: FastTraceAnalysis,
    *,
    gear_ratio: float,
) -> Path | None:
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt  # type: ignore
    except Exception:
        return None

    ts: list[float] = []
    ps: list[int] = []
    mts: list[int | None] = []
    t0: int | None = None
    with jsonl_path.open("r", encoding="utf-8") as fh:
        for line in fh:
            line = line.strip()
            if not line:
                continue
            try:
                record = json.loads(line)
            except json.JSONDecodeError:
                continue
            extracted = _extract_axis_sample(record, axis_index=result.axis_index)
            if extracted is None:
                continue
            if t0 is None:
                t0 = extracted["t_ns"]
            ts.append((extracted["t_ns"] - t0) / 1e9)
            ps.append(extracted["p"])
            mts.append(extracted["mt_i64"])
    if not ts:
        return None

    # Reconstruct unwrapped wire cumulative from per-sample deltas.
    unwrapped = [0]
    for a, b in zip(ps[:-1], ps[1:]):
        unwrapped.append(unwrapped[-1] + _unwrap_wire_delta(b - a, result.rm_counts))
    unwrapped_deg = [_counts_to_output_deg(c, gear_ratio=gear_ratio) for c in unwrapped]

    fig, axes = plt.subplots(2, 1, figsize=(12, 8), sharex=True)

    axes[0].plot(ts, unwrapped_deg, color="#1f77b4", linewidth=1.2,
                 label="0x6064 unwrapped (deg output)")
    mt_ts = [t for t, m in zip(ts, mts) if m is not None]
    if mt_ts:
        first_mt = next((m for m in mts if m is not None), None)
        if first_mt is not None:
            mt_series = [
                _counts_to_output_deg(m - first_mt, gear_ratio=gear_ratio)
                for m in mts if m is not None
            ]
            axes[0].plot(mt_ts, mt_series, color="#d62728", linewidth=1.0,
                         linestyle="--", alpha=0.8,
                         label="U40.20/.22 multi-turn (deg output)")
    axes[0].set_ylabel("displacement (deg output)")
    title = (
        f"{jsonl_path.name}  axis={result.axis_index}  "
        f"samples={result.sample_count}  hz={result.effective_hz:.1f}  "
        f"cum={_counts_to_output_deg(result.cumulative_travel_wire_counts, gear_ratio=gear_ratio):.2f}deg  "
        f"net={_counts_to_output_deg(result.net_displacement_wire_counts, gear_ratio=gear_ratio):+.2f}deg"
    )
    if result.ratio_cumulative_over_abs_net_wire is not None:
        title += f"  cum/|net|={result.ratio_cumulative_over_abs_net_wire:.2f}"
    axes[0].set_title(title)
    axes[0].grid(True, alpha=0.3)
    axes[0].axhline(0.0, linestyle=":", color="gray", alpha=0.5)
    axes[0].legend(loc="upper left")

    axes[1].plot(ts, ps, color="#2ca02c", linewidth=1.0, label="0x6064 (wire, motor counts)")
    axes[1].set_ylabel("0x6064 (motor counts)")
    axes[1].set_xlabel("time since first sample (s)")
    axes[1].grid(True, alpha=0.3)
    if result.rm_counts > 0:
        axes[1].axhline(result.rm_counts, color="gray", linestyle=":", alpha=0.4,
                        label=f"RM={result.rm_counts}")
        axes[1].axhline(0, color="gray", linestyle=":", alpha=0.4)
    axes[1].legend(loc="upper left")

    if result.fault_seen and result.fault_sample_index is not None:
        # samples[] was post-filtered to drop leading zeros; fault index is
        # into that filtered list. For the plot we just annotate by time.
        fault_t = ts[min(result.fault_sample_index, len(ts) - 1)]
        for ax in axes:
            ax.axvline(fault_t, color="red", alpha=0.7, linewidth=1.2)
        axes[0].legend(loc="upper left")

    out_path = jsonl_path.with_suffix(".png")
    fig.tight_layout()
    fig.savefig(out_path, dpi=120)
    plt.close(fig)
    return out_path


def _print_rtcore_analysis(
    result: FastTraceAnalysis,
    *,
    gear_ratio: float,
) -> None:
    cum_deg = _counts_to_output_deg(result.cumulative_travel_wire_counts, gear_ratio=gear_ratio)
    net_deg = _counts_to_output_deg(result.net_displacement_wire_counts, gear_ratio=gear_ratio)
    max_step_deg = _counts_to_output_deg(result.max_abs_wire_step_counts, gear_ratio=gear_ratio)
    mt_net_deg: float | None = None
    if result.mt_net_delta is not None:
        mt_net_deg = _counts_to_output_deg(result.mt_net_delta, gear_ratio=gear_ratio)

    print(f"=== RTCore fast_trace analysis: {result.jsonl_path.name} ===")
    print(f"  axis_index               : {result.axis_index}")
    print(f"  rm_counts                : {result.rm_counts}")
    print(f"  samples                  : {result.sample_count}")
    print(f"  elapsed_s                : {result.elapsed_s:.3f}")
    print(f"  effective_hz             : {result.effective_hz:.2f}")
    print(f"  first_p (0x6064)         : {result.first_p}")
    print(f"  last_p  (0x6064)         : {result.last_p}")
    print(f"  cumulative_travel        : {result.cumulative_travel_wire_counts} counts "
          f"({cum_deg:.3f} deg output)")
    print(f"  net_displacement         : {result.net_displacement_wire_counts:+d} counts "
          f"({net_deg:+.3f} deg output)")
    shortest_deg = _counts_to_output_deg(result.net_displacement_shortest_wrap_counts,
                                          gear_ratio=gear_ratio)
    long_excess_deg = _counts_to_output_deg(result.long_path_excess_counts, gear_ratio=gear_ratio)
    print(f"  net_shortest_path        : {result.net_displacement_shortest_wrap_counts:+d} counts "
          f"({shortest_deg:+.3f} deg output)")
    print(f"  long_path_excess         : {result.long_path_excess_counts} counts "
          f"({long_excess_deg:.3f} deg output) -- # of RM-revs the motor took beyond shortest path")
    print(f"  max_abs_wire_step        : {result.max_abs_wire_step_counts} counts "
          f"({max_step_deg:.3f} deg output) -- per-1ms-cycle cap")
    net_abs = abs(result.net_displacement_wire_counts)
    # Noise floor: on an idle/disarmed axis the wire ticks by at most a few
    # counts per cycle. Apply the motion-based WHIP verdict only when the net
    # displacement is meaningfully above that floor.
    noise_floor_counts = max(10 * max(result.max_abs_wire_step_counts, 1), 1000)

    # Primary WHIP gate: long-path excess >= RM/2 means the motor took at
    # least one extra full revolution beyond the shortest path. This fires
    # even on perfectly monotonic motion that happens to go the long way
    # around (the 2026-04-19 UI jog sequence discovered this class of bug:
    # host's nearest-turn fold flipped turn count at the seam, commanded a
    # long-path target, drive faithfully followed). Threshold is RM/2
    # because anything less would be within the drive's Nearest (C10.16=0)
    # short-path region.
    half_rm = result.rm_counts // 2 if result.rm_counts > 0 else 0
    long_path_is_whip = half_rm > 0 and result.long_path_excess_counts >= half_rm

    r = result.ratio_cumulative_over_abs_net_wire
    if r is not None:
        print(f"  cum / |net|              : {r:.3f}")
    else:
        print("  cum / |net|              : undefined (net = 0)")

    if long_path_is_whip:
        turns = result.long_path_excess_counts / max(result.rm_counts, 1)
        print(f"  VERDICT                  : WHIP (long-path detour = "
              f"{turns:.2f} full revolutions beyond shortest path)")
    elif r is None or net_abs < noise_floor_counts:
        print(f"  VERDICT                  : STATIC (|net|={net_abs} counts < noise floor "
              f"{noise_floor_counts})")
    elif r > 1.2 or not result.wire_monotonic_within_budget:
        reasons: list[str] = []
        if r > 1.2:
            reasons.append(f"cum/|net|={r:.2f} > 1.2")
        if not result.wire_monotonic_within_budget:
            reasons.append("direction flipped beyond overshoot budget")
        print(f"  VERDICT                  : WHIP ({'; '.join(reasons)})")
    else:
        print("  VERDICT                  : CLEAN")
    print(f"  wire_monotonic           : {result.wire_monotonic_within_budget} "
          f"(budget={result.overshoot_budget_counts} counts)")
    print(f"  U40.20/.22 samples       : {result.mt_sample_count} ok  "
          f"(distinct values = {result.mt_distinct_samples})")
    if mt_net_deg is not None:
        print(f"  U40.20/.22 net_delta     : {result.mt_net_delta:+d} motor counts "
              f"({mt_net_deg:+.3f} deg output) -- cross-check")
    if result.fault_seen:
        print(f"  fault_seen               : True -- {result.fault_detail}")
    else:
        print("  fault_seen               : False")


# =========================================================================
# save: preserve the RTCore fast_trace JSONL before the runtime dir is wiped
# =========================================================================

DEFAULT_RTCORE_TRACE_PATH = Path("/run/gradient-rt-motion/j6-fast-trace.jsonl")


@dataclasses.dataclass
class SaveResult:
    dest_jsonl: Path | None
    dest_meta: Path | None
    copied: bool
    skipped_reason: str | None
    source_size_bytes: int
    source_mtime_ns: int | None


def _estimate_trace_stats(path: Path) -> dict[str, Any]:
    """Cheap summary of a fast_trace JSONL: line count, first+last timestamps,
    first+last axis samples (the axis whose mask=0x20 position was configured).

    This avoids loading the full JSONL into memory: we read the head and tail
    only for the representative samples, and stream once for the line count.
    """
    line_count = 0
    with path.open("rb") as fh:
        for _ in fh:
            line_count += 1

    first_line = ""
    last_line = ""
    try:
        with path.open("r", encoding="utf-8", errors="replace") as fh:
            first_line = fh.readline().strip()
    except OSError:
        pass
    # Tail a small chunk (up to 64 KB) for the last complete line.
    try:
        with path.open("rb") as fh:
            fh.seek(0, os.SEEK_END)
            end = fh.tell()
            chunk = 65536 if end > 65536 else end
            fh.seek(end - chunk)
            tail = fh.read(chunk).decode("utf-8", errors="replace")
            for candidate in reversed(tail.splitlines()):
                if candidate.strip():
                    last_line = candidate.strip()
                    break
    except OSError:
        pass

    def _parse_t(line: str) -> int | None:
        if not line:
            return None
        try:
            record = json.loads(line)
        except json.JSONDecodeError:
            return None
        t = record.get("t_ns")
        return int(t) if isinstance(t, int) else None

    t_first = _parse_t(first_line)
    t_last = _parse_t(last_line)
    elapsed_s = (t_last - t_first) / 1e9 if (t_first is not None and t_last is not None) else None
    hz = (line_count / elapsed_s) if (elapsed_s is not None and elapsed_s > 0) else None
    return {
        "line_count": line_count,
        "first_t_ns": t_first,
        "last_t_ns": t_last,
        "elapsed_s": elapsed_s,
        "effective_hz": hz,
    }


def _run_subproc(argv: list[str]) -> tuple[int, str, str]:
    try:
        proc = subprocess.run(argv, capture_output=True, text=True, check=False)
    except FileNotFoundError as exc:
        return 127, "", str(exc)
    return proc.returncode, proc.stdout, proc.stderr


def _save_rtcore_trace(
    *,
    source_path: Path,
    dest_dir: Path,
    label: str,
    if_exists: bool,
    note: str | None,
) -> SaveResult:
    """Copy the active RTCore fast_trace JSONL to `dest_dir/<label>-<iso>.jsonl`
    plus a `.meta.json` sibling. Uses `sudo -n cp` because the runtime file is
    owned by root:pi but systemd may have it open. Handles the "source does
    not exist" case gracefully when `if_exists` is True (for the auto-save
    hook).
    """
    if not source_path.exists():
        reason = f"source {source_path} does not exist"
        if if_exists:
            return SaveResult(
                dest_jsonl=None, dest_meta=None, copied=False,
                skipped_reason=reason, source_size_bytes=0, source_mtime_ns=None,
            )
        raise FileNotFoundError(reason)

    # Bytes + mtime up-front for the meta file.
    stat_rc, stat_out, stat_err = _run_subproc(["sudo", "-n", "stat", "-c", "%s %Y",
                                                str(source_path)])
    if stat_rc != 0:
        raise RuntimeError(f"stat({source_path}) failed rc={stat_rc}: {stat_err.strip()}")
    parts = stat_out.strip().split()
    size_bytes = int(parts[0]) if parts else 0
    mtime_s = int(parts[1]) if len(parts) > 1 else 0
    if if_exists and size_bytes == 0:
        return SaveResult(
            dest_jsonl=None, dest_meta=None, copied=False,
            skipped_reason=f"source {source_path} is empty (size 0)",
            source_size_bytes=0, source_mtime_ns=None,
        )

    dest_dir.mkdir(parents=True, exist_ok=True)
    base = f"{_normalize_label(label)}-{_iso_token()}"
    dest_jsonl = dest_dir / f"{base}.jsonl"
    dest_meta = dest_dir / f"{base}.meta.json"

    # Atomic-ish copy: cp then chown. Done via sudo because the source file
    # may not be world-readable depending on the runtime dir mode.
    cp_rc, _cp_out, cp_err = _run_subproc(["sudo", "-n", "cp", "--preserve=timestamps",
                                            str(source_path), str(dest_jsonl)])
    if cp_rc != 0:
        raise RuntimeError(f"sudo cp failed rc={cp_rc}: {cp_err.strip()}")
    # Chown to the invoking user so the analyze path does not need sudo.
    user = os.environ.get("SUDO_USER") or os.environ.get("USER") or "pi"
    try:
        import pwd
        import grp
        pw = pwd.getpwnam(user)
        gid = grp.getgrnam(user).gr_gid if user == "pi" else pw.pw_gid
    except Exception:
        pw = None
        gid = None
    if pw is not None and gid is not None:
        chown_rc, _out, chown_err = _run_subproc(
            ["sudo", "-n", "chown", f"{pw.pw_uid}:{gid}", str(dest_jsonl)]
        )
        if chown_rc != 0:
            # Not fatal; the file still exists and root can read it.
            print(f"[save] warn: chown failed: {chown_err.strip()}", flush=True)

    # Compute summary stats from the copied-out JSONL (no sudo needed now).
    try:
        stats = _estimate_trace_stats(dest_jsonl)
    except Exception as exc:
        stats = {"line_count": None, "error": str(exc)}

    meta = {
        "schema_version": 1,
        "tool": "j6_multiturn_fast_capture save",
        "label": label,
        "source_path": str(source_path),
        "dest_jsonl": str(dest_jsonl),
        "saved_at_wall_utc": _utc_now().isoformat(),
        "source_size_bytes": size_bytes,
        "source_mtime_unix_s": mtime_s,
        "stats": stats,
        "note": note,
    }
    dest_meta.write_text(json.dumps(meta, indent=2))

    return SaveResult(
        dest_jsonl=dest_jsonl,
        dest_meta=dest_meta,
        copied=True,
        skipped_reason=None,
        source_size_bytes=size_bytes,
        source_mtime_ns=mtime_s * 1_000_000_000,
    )


# =========================================================================
# CLI
# =========================================================================

def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "J6 high-frequency multi-turn capture + post-processor. See "
            "Phase 2 of the j6_seam_whip_verification plan."
        )
    )
    sub = parser.add_subparsers(dest="command", required=True)

    cap = sub.add_parser("capture", help="Run the live capture loop")
    cap.add_argument("--label", required=True, help="Human label for this capture run")
    cap.add_argument("--axis-index", type=int, default=DEFAULT_J6_AXIS_INDEX,
                     help="Zero-based slave position (J6 = 5)")
    cap.add_argument("--duration-s", type=float, required=True,
                     help="Maximum capture duration in seconds")
    cap.add_argument("--log-dir", type=Path, default=DEFAULT_LOG_DIR,
                     help="Output directory (default logs/j6-multiturn-fast)")
    cap.add_argument("--minimal", action="store_true",
                     help="Minimal descriptor set (U40.20/U40.22 + 603F/6041 only)")
    cap.add_argument("--no-fault-halt", action="store_true",
                     help="Continue capture even if 603F or 6041 signals fault")
    cap.add_argument("--note", default=None,
                     help="Optional operator note appended to the meta file")

    ana = sub.add_parser("analyze",
                         help="Post-process a capture JSONL from this script's own format (deprecated)")
    ana.add_argument("jsonl", type=Path, help="Path to capture JSONL")
    ana.add_argument("--overshoot-budget-counts", type=int, default=500,
                     help="Tolerable small-reversal per-sample, motor counts (default 500 = ~0.14 deg)")
    ana.add_argument("--gear-ratio", type=float, default=10.0,
                     help="Gear ratio for output-deg conversion (J6 = 10)")
    ana.add_argument("--no-plot", action="store_true",
                     help="Skip PNG plot even if matplotlib is available")

    sav = sub.add_parser("save",
                         help="Copy the live RTCore fast_trace JSONL to logs/j6-multiturn-fast/ before "
                              "/run/gradient-rt-motion/ is wiped")
    sav.add_argument("--source", type=Path, default=DEFAULT_RTCORE_TRACE_PATH,
                     help="Source JSONL path (default /run/gradient-rt-motion/j6-fast-trace.jsonl)")
    sav.add_argument("--log-dir", type=Path, default=DEFAULT_LOG_DIR,
                     help="Destination directory (default logs/j6-multiturn-fast)")
    sav.add_argument("--label", default="autosave",
                     help="Human label for this snapshot (default 'autosave')")
    sav.add_argument("--if-exists", action="store_true",
                     help="Silent no-op if source is missing or empty (for hooks)")
    sav.add_argument("--note", default=None,
                     help="Optional operator note appended to the meta file")

    rt = sub.add_parser("analyze-rtcore",
                        help="Post-process a RTCore fast_trace JSONL (preferred)")
    rt.add_argument("jsonl", type=Path, help="Path to RTCore fast_trace JSONL")
    rt.add_argument("--axis-index", type=int, default=DEFAULT_J6_AXIS_INDEX,
                    help="Zero-based slave position (J6 = 5)")
    rt.add_argument("--gear-ratio", type=float, default=DEFAULT_J6_GEAR_RATIO,
                    help="Drive gear ratio for output-deg conversion (J6 = 10)")
    rt.add_argument("--counts-per-motor-rev", type=int, default=COUNTS_PER_MOTOR_REV,
                    help="Counts per motor revolution (17-bit encoder = 131072)")
    rt.add_argument("--rm-counts", type=int, default=0,
                    help="Wire rotation-mode period in counts. 0 = counts_per_motor_rev * gear_ratio")
    rt.add_argument("--overshoot-budget-counts", type=int, default=500,
                    help="Tolerable small-reversal per-sample, wire-frame motor counts (default 500)")
    rt.add_argument("--no-plot", action="store_true",
                    help="Skip PNG plot even if matplotlib is available")

    return parser.parse_args()


def main() -> int:
    args = _parse_args()
    if args.command == "capture":
        descriptors = MINIMAL_DESCRIPTORS if args.minimal else SDO_DESCRIPTORS
        options = CaptureOptions(
            label=args.label,
            axis_index=int(args.axis_index),
            duration_s=float(args.duration_s),
            log_dir=Path(args.log_dir),
            descriptors=descriptors,
            halt_on_fault=not bool(args.no_fault_halt),
            note=args.note,
        )
        _run_capture(options)
        return 0

    if args.command == "analyze":
        jsonl = Path(args.jsonl)
        if not jsonl.is_file():
            print(f"ERROR: not a file: {jsonl}", file=sys.stderr)
            return 1
        result = _analyze_jsonl(jsonl, overshoot_budget_counts=int(args.overshoot_budget_counts))
        _print_analysis_summary(result, gear_ratio=float(args.gear_ratio))
        if not args.no_plot:
            png = _plot_trace_png(jsonl, result, gear_ratio=float(args.gear_ratio))
            if png is not None:
                print(f"  plot               : {png}")
            else:
                print("  plot               : skipped (matplotlib unavailable)")
        return 0

    if args.command == "save":
        try:
            result = _save_rtcore_trace(
                source_path=Path(args.source),
                dest_dir=Path(args.log_dir),
                label=str(args.label),
                if_exists=bool(args.if_exists),
                note=args.note,
            )
        except FileNotFoundError as exc:
            print(f"[save] ERROR: {exc}", file=sys.stderr)
            return 1
        except RuntimeError as exc:
            print(f"[save] ERROR: {exc}", file=sys.stderr)
            return 2
        if not result.copied:
            print(f"[save] skipped: {result.skipped_reason}", flush=True)
            return 0
        print(
            f"[save] copied {result.source_size_bytes} bytes -> {result.dest_jsonl}\n"
            f"[save] meta -> {result.dest_meta}",
            flush=True,
        )
        return 0

    if args.command == "analyze-rtcore":
        jsonl = Path(args.jsonl)
        if not jsonl.is_file():
            print(f"ERROR: not a file: {jsonl}", file=sys.stderr)
            return 1
        rm = int(args.rm_counts) if int(args.rm_counts) > 0 else int(
            int(args.counts_per_motor_rev) * float(args.gear_ratio)
        )
        result = _analyze_rtcore_jsonl(
            jsonl,
            axis_index=int(args.axis_index),
            rm_counts=rm,
            overshoot_budget_counts=int(args.overshoot_budget_counts),
        )
        _print_rtcore_analysis(result, gear_ratio=float(args.gear_ratio))
        if not args.no_plot:
            png = _plot_rtcore_trace_png(jsonl, result, gear_ratio=float(args.gear_ratio))
            if png is not None:
                print(f"  plot                     : {png}")
            else:
                print("  plot                     : skipped (matplotlib unavailable)")
        return 0

    return 2


if __name__ == "__main__":
    raise SystemExit(main())
