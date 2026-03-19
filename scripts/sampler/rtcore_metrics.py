#!/usr/bin/env python3
"""
Tiny helper that reads RTCore metrics from:
  /run/gradient-rt-motion/metrics.json

Designed for Sampler dashboards (numeric outputs for charts/gauges).
"""

from __future__ import annotations

import argparse
import json
import os
import sys
from typing import Any


def _load_metrics(path: str) -> tuple[dict[str, Any], str | None]:
    try:
        with open(path, "r", encoding="utf-8") as f:
            data = json.load(f)
        if isinstance(data, dict):
            if not data:
                return {}, "metrics JSON object is empty"
            return data, None
        return {}, "metrics file did not contain a JSON object"
    except FileNotFoundError:
        return {}, "metrics file does not exist"
    except PermissionError:
        return {}, "metrics file is not readable (permission denied)"
    except json.JSONDecodeError:
        return {}, "metrics file contains invalid JSON"
    except Exception as exc:
        return {}, f"failed to load metrics ({exc.__class__.__name__})"


def _fnum(x: Any, default: float = 0.0) -> float:
    try:
        return float(x)
    except Exception:
        return float(default)


def _inum(x: Any, default: int = 0) -> int:
    try:
        return int(x)
    except Exception:
        return int(default)


def _al_state_name(x: Any) -> str:
    state = _inum(x, 0)
    if state == 0x01:
        return "INIT"
    if state == 0x02:
        return "PREOP"
    if state == 0x04:
        return "SAFEOP"
    if state == 0x08:
        return "OP"
    return "UNKNOWN"


def _hex16(x: Any) -> str:
    return f"0x{_inum(x, 0) & 0xFFFF:04x}"


def _get_axis(m: dict[str, Any], axis: int) -> dict[str, Any]:
    axes = m.get("axes", [])
    if isinstance(axes, list) and 0 <= axis < len(axes):
        ent = axes[axis]
        if isinstance(ent, dict):
            return ent
    return {}


def _get_slave_positions(m: dict[str, Any]) -> str:
    vals = m.get("slave_positions", [])
    if not isinstance(vals, list):
        return "[]"
    out: list[str] = []
    for ent in vals:
        out.append(str(_inum(ent, 0)))
    return "[" + ",".join(out) + "]"


def main() -> int:
    ap = argparse.ArgumentParser(description="Read RTCore metrics (for Sampler)")
    ap.add_argument(
        "--path",
        default=os.environ.get("GRADIENT_RTCORE_METRICS", "/run/gradient-rt-motion/metrics.json"),
        help="Path to RTCore metrics.json",
    )
    ap.add_argument(
        "--axis",
        type=int,
        default=0,
        help="Axis index for axis_* metrics (default: 0)",
    )
    ap.add_argument(
        "metric",
        help=(
            "Metric name. Examples: rt_hz, rt_cycles, rt_last_jitter_us, rt_max_jitter_us, "
            "wkc_actual, wkc_expected, wkc_ratio, master_state, armed, axis0_error_code, summary"
        ),
    )
    args = ap.parse_args()

    m, load_error = _load_metrics(str(args.path))
    metric = str(args.metric).strip()
    axis = int(args.axis)

    if metric == "rt_hz":
        print(f"{_fnum(m.get('rt_hz'), 0.0):.1f}")
        return 0

    if metric == "rt_cycles":
        print(_inum(m.get("rt_cycle_counter"), 0))
        return 0

    if metric == "rt_last_jitter_ns":
        print(_inum(m.get("rt_last_jitter_ns"), 0))
        return 0

    if metric == "rt_max_abs_jitter_ns":
        print(_inum(m.get("rt_max_abs_jitter_ns"), 0))
        return 0

    if metric == "rt_last_jitter_us":
        v = abs(_inum(m.get("rt_last_jitter_ns"), 0))
        print(f"{v / 1000.0:.1f}")
        return 0

    if metric == "rt_max_jitter_us":
        v = _inum(m.get("rt_max_abs_jitter_ns"), 0)
        print(f"{v / 1000.0:.1f}")
        return 0

    if metric == "wkc_actual":
        print(_inum(m.get("wkc_actual"), 0))
        return 0

    if metric == "wkc_expected":
        print(_inum(m.get("wkc_expected"), 0))
        return 0

    if metric == "wkc_ratio":
        a = _fnum(m.get("wkc_actual"), 0.0)
        e = _fnum(m.get("wkc_expected"), 0.0)
        if e <= 0:
            print("0")
            return 0
        print(f"{a / e:.3f}")
        return 0

    if metric == "master_state":
        print(_inum(m.get("master_state"), 0))
        return 0

    if metric == "armed":
        print(1 if _inum(m.get("armed"), 0) else 0)
        return 0

    if metric == "axis_error_code":
        a = _get_axis(m, axis)
        print(_inum(a.get("error_code"), 0))
        return 0

    if metric == "axis_statusword":
        a = _get_axis(m, axis)
        print(_inum(a.get("statusword"), 0))
        return 0

    if metric == "summary":
        if not m:
            # Keep summary mode user-readable so preflight failures are obvious.
            print(f"RTCore metrics unavailable ({args.path})")
            if load_error:
                print(f"  reason={load_error}")
            print("  expected producer=/usr/local/bin/gradient-rt-motion")
            print("  hint=start RTCore and retry")
            return 0

        # Multi-line, human-readable for Sampler textbox.
        num_axes = _inum(m.get("num_axes"), 0)
        cycle_ns = _inum(m.get("cycle_ns"), 0)
        pdo_profile = str(m.get("pdo_profile") or "unknown")
        print(f"RTCore metrics ({args.path})")
        print(f"  cycle_ns={cycle_ns} num_axes={num_axes}")
        print(
            f"  profile={pdo_profile} rx_pdo={_hex16(m.get('rx_pdo'))} "
            f"tx_pdo={_hex16(m.get('tx_pdo'))} dc_enabled={_inum(m.get('dc_enabled'), 0)} "
            f"output_watchdog_enabled={_inum(m.get('output_watchdog_enabled'), 0)} "
            f"split_domains_per_axis={_inum(m.get('split_domains_per_axis'), 0)} "
            f"queue_split_domains_round_robin={_inum(m.get('queue_split_domains_round_robin'), 0)} "
            f"explicit_pdo_config={_inum(m.get('explicit_pdo_config'), 0)} "
            f"wait_before_safeop_ms={_inum(m.get('wait_before_safeop_ms'), 0)} "
            f"preop_safeop_timeout_ms={_inum(m.get('preop_to_safeop_timeout_ms'), 0)} "
            f"safeop_op_timeout_ms={_inum(m.get('safeop_to_op_timeout_ms'), 0)} "
            f"startup_passive_ms={_inum(m.get('startup_passive_ms'), 0)} "
            f"startup_passive_active={_inum(m.get('startup_passive_active'), 0)} "
            f"startup_skip_domain_queue_ms={_inum(m.get('startup_skip_domain_queue_ms'), 0)} "
            f"startup_skip_domain_queue_active={_inum(m.get('startup_skip_domain_queue_active'), 0)}"
        )
        print(f"  slave_positions={_get_slave_positions(m)}")
        print(f"  rt_hz={_fnum(m.get('rt_hz'), 0.0):.1f} rt_cycles={_inum(m.get('rt_cycle_counter'), 0)}")
        print(
            f"  rt_jitter_us(last/max)={abs(_inum(m.get('rt_last_jitter_ns'), 0))/1000.0:.1f}/"
            f"{_inum(m.get('rt_max_abs_jitter_ns'), 0)/1000.0:.1f}"
        )
        print(
            f"  wkc={_inum(m.get('wkc_actual'), 0)}/{_inum(m.get('wkc_expected'), 0)} "
            f"master_state={_inum(m.get('master_state'), 0)} armed={_inum(m.get('armed'), 0)} "
            f"enable_mask=0x{_inum(m.get('axis_enable_mask'), 0):x}"
        )
        print(
            f"  bus: link_up={_inum(m.get('link_up'), 0)} "
            f"responding={_inum(m.get('responding_slaves'), 0)}/{num_axes} "
            f"online={_inum(m.get('online_slaves'), 0)}/{num_axes} "
            f"operational={_inum(m.get('operational_slaves'), 0)}/{num_axes} "
            f"master_al=0x{_inum(m.get('master_al_states'), 0):x} "
            f"domain_wc={_inum(m.get('domain_wc_state'), 0)} "
            f"startup_ready={_inum(m.get('startup_ready'), 0)} "
            f"startup_elapsed_ms={_inum(m.get('startup_elapsed_ms'), 0)} "
            f"startup_resets={_inum(m.get('startup_reset_count'), 0)}"
        )
        axes = m.get("axes", [])
        if isinstance(axes, list):
            for i, ent in enumerate(axes[:num_axes]):
                if not isinstance(ent, dict):
                    continue
                err = _inum(ent.get("error_code"), 0)
                sw = _inum(ent.get("statusword"), 0)
                pos = _inum(ent.get("pos_counts"), 0)
                slave_online = _inum(ent.get("slave_online"), 0)
                slave_operational = _inum(ent.get("slave_operational"), 0)
                slave_al_state = _inum(ent.get("slave_al_state"), 0)
                print(
                    f"  axis{i}: err=0x{err:04x} sw=0x{sw:04x} pos_counts={pos} "
                    f"slave_online={slave_online} slave_operational={slave_operational} "
                    f"slave_al={_al_state_name(slave_al_state)}(0x{slave_al_state:x})"
                )
        return 0

    # Unknown metric: keep Sampler stable (numeric default).
    print("0")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

