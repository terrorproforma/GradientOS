#!/usr/bin/env python3
"""Capture and analyze Cartesian held-jog diagnostics."""

from __future__ import annotations

import argparse
import datetime as _dt
import json
import os
from pathlib import Path
import shutil
import signal
import subprocess
import sys
import time
from typing import Any
from urllib import request


DEFAULT_API_BASE = "http://127.0.0.1:4400"
DEFAULT_OUTPUT_DIR = Path("logs/cartesian-jog-diagnostic")
DEFAULT_FAST_TRACE_PATH = Path("/run/gradient-rt-motion/cartesian-jog-fast-trace.jsonl")
FAST_TRACE_DROPIN = Path("/etc/systemd/system/gradient-rt-motion.service.d/fast-trace.conf")


def _timestamp() -> str:
    return _dt.datetime.now(_dt.timezone.utc).strftime("%Y%m%d-%H%M%S")


def _json_request(method: str, url: str, payload: dict[str, Any] | None = None, timeout_s: float = 2.0) -> Any:
    body = None
    headers = {}
    if payload is not None:
        body = json.dumps(payload, separators=(",", ":")).encode("utf-8")
        headers["Content-Type"] = "application/json"
    req = request.Request(url, data=body, headers=headers, method=method)
    with request.urlopen(req, timeout=timeout_s) as response:  # noqa: S310 - local operator tool
        return json.loads(response.read().decode("utf-8"))


def _set_ab_compare(api_base: str, enabled: bool) -> None:
    _json_request("POST", f"{api_base.rstrip('/')}/control/jog/debug", {"ab_compare": bool(enabled)})


def _performance_snapshot(api_base: str) -> dict[str, Any]:
    payload = _json_request("GET", f"{api_base.rstrip('/')}/debug/performance")
    if not isinstance(payload, dict):
        raise RuntimeError("Performance endpoint did not return a JSON object.")
    return payload


def _extract_jog_payload(snapshot: dict[str, Any]) -> dict[str, Any]:
    """Return controller jog payload from /debug/performance response."""
    jog = snapshot.get("jog")
    if isinstance(jog, dict):
        return jog
    controller = snapshot.get("controller")
    if isinstance(controller, dict) and isinstance(controller.get("jog"), dict):
        return controller["jog"]
    return {}


def _run_systemctl_restart() -> None:
    subprocess.run(["sudo", "systemctl", "daemon-reload"], check=True)
    subprocess.run(["sudo", "systemctl", "restart", "gradient-rt-motion.service"], check=True)


def enable_fast_trace(_args: argparse.Namespace) -> int:
    FAST_TRACE_DROPIN.parent.mkdir(parents=True, exist_ok=True)
    content = "\n".join(
        [
            "[Service]",
            "Environment=GRADIENT_RT_FAST_TRACE_HZ=1000",
            "Environment=GRADIENT_RT_FAST_TRACE_AXIS_MASK=0x3F",
            f"Environment=GRADIENT_RT_FAST_TRACE_PATH={DEFAULT_FAST_TRACE_PATH}",
            "",
        ]
    )
    FAST_TRACE_DROPIN.write_text(content, encoding="utf-8")
    _run_systemctl_restart()
    print(f"Enabled RTCore fast trace at {DEFAULT_FAST_TRACE_PATH}. Disable after capture.")
    return 0


def disable_fast_trace(_args: argparse.Namespace) -> int:
    if FAST_TRACE_DROPIN.exists():
        FAST_TRACE_DROPIN.write_text(
            "[Service]\nEnvironment=GRADIENT_RT_FAST_TRACE_HZ=0\n",
            encoding="utf-8",
        )
    _run_systemctl_restart()
    print("Disabled RTCore fast trace. If /run is low on space, inspect/truncate old trace files.")
    return 0


def capture(args: argparse.Namespace) -> int:
    output_dir = Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    out_path = output_dir / f"python-jog-{_timestamp()}.jsonl"
    stop = {"requested": False}

    def _handle_signal(_signum, _frame):
        stop["requested"] = True

    signal.signal(signal.SIGINT, _handle_signal)
    signal.signal(signal.SIGTERM, _handle_signal)

    period_s = 1.0 / float(args.hz)
    _set_ab_compare(args.api_base, True)
    print(f"Capturing jog diagnostics to {out_path}. Press Ctrl-C to stop.")
    try:
        with out_path.open("w", encoding="utf-8") as f:
            while not stop["requested"]:
                started = time.monotonic()
                snapshot = _performance_snapshot(args.api_base)
                jog = _extract_jog_payload(snapshot)
                record = {
                    "captured_monotonic_s": started,
                    "captured_wall_utc": _dt.datetime.now(_dt.timezone.utc).isoformat(),
                    "ik_debug": jog.get("ik_debug") if isinstance(jog, dict) else None,
                    "jog": jog,
                }
                f.write(json.dumps(record, separators=(",", ":"), ensure_ascii=True) + "\n")
                f.flush()
                elapsed = time.monotonic() - started
                time.sleep(max(0.0, period_s - elapsed))
    finally:
        try:
            _set_ab_compare(args.api_base, False)
        except Exception as exc:  # pragma: no cover - best-effort operator cleanup
            print(f"WARNING: failed to disable A/B compare: {exc}", file=sys.stderr)

    if DEFAULT_FAST_TRACE_PATH.exists():
        dest = out_path.with_name(f"rtcore-fast-trace-{out_path.stem.removeprefix('python-jog-')}.jsonl")
        shutil.copy2(DEFAULT_FAST_TRACE_PATH, dest)
        print(f"Copied RTCore fast trace to {dest}")
    print("IMPORTANT: run `cartesian_jog_diagnostic_capture.py disable-fast-trace` now if fast trace is enabled.")
    return 0


def load_jsonl(path: str | Path) -> list[dict[str, Any]]:
    records: list[dict[str, Any]] = []
    with Path(path).open("r", encoding="utf-8") as f:
        for line_no, line in enumerate(f, start=1):
            line = line.strip()
            if not line:
                continue
            try:
                payload = json.loads(line)
            except json.JSONDecodeError as exc:
                raise ValueError(f"{path}:{line_no}: invalid JSONL: {exc}") from exc
            if isinstance(payload, dict):
                records.append(payload)
    return records


def _ik_debug(record: dict[str, Any]) -> dict[str, Any]:
    if isinstance(record.get("ik_debug"), dict):
        return record["ik_debug"]
    jog = record.get("jog")
    if isinstance(jog, dict) and isinstance(jog.get("ik_debug"), dict):
        return jog["ik_debug"]
    controller = record.get("controller")
    if isinstance(controller, dict):
        jog = controller.get("jog")
        if isinstance(jog, dict) and isinstance(jog.get("ik_debug"), dict):
            return jog["ik_debug"]
    return {}


def analyze_python_records(records: list[dict[str, Any]], *, branch_threshold_rad: float = 0.5) -> dict[str, Any]:
    branch_flips: list[dict[str, Any]] = []
    singularity_spans: list[dict[str, Any]] = []
    drift_values: list[float] = []
    gate_rejections: dict[str, int] = {}
    in_singularity: dict[str, Any] | None = None

    for idx, record in enumerate(records):
        debug = _ik_debug(record)
        ab_compare = debug.get("ab_compare")
        if isinstance(ab_compare, dict):
            delta = ab_compare.get("ik_minus_jacobian_max_abs_rad")
            if isinstance(delta, (int, float)) and float(delta) > branch_threshold_rad:
                branch_flips.append({"index": idx, "delta_rad": float(delta), "record": record})

        jacobian = debug.get("jacobian_diagnostics")
        near = False
        sigma_min = None
        if isinstance(jacobian, dict):
            near = bool(jacobian.get("jacobian_near_singular", False))
            raw_sigma = jacobian.get("jacobian_sigma_min")
            if isinstance(raw_sigma, (int, float)):
                sigma_min = float(raw_sigma)
        if near and in_singularity is None:
            in_singularity = {"start_index": idx, "start_sigma_min": sigma_min}
        if not near and in_singularity is not None:
            in_singularity["end_index"] = idx - 1
            singularity_spans.append(in_singularity)
            in_singularity = None

        drift = debug.get("command_drift_norm_rad")
        if isinstance(drift, (int, float)):
            drift_values.append(float(drift))
        reason = debug.get("gate_reason")
        result = debug.get("gate_result")
        if result == "rejected" and isinstance(reason, str) and reason:
            gate_rejections[reason] = gate_rejections.get(reason, 0) + 1

    if in_singularity is not None:
        in_singularity["end_index"] = len(records) - 1
        singularity_spans.append(in_singularity)

    monotonic_drift = (
        len(drift_values) >= 2
        and all(next_value >= value for value, next_value in zip(drift_values, drift_values[1:]))
    )
    return {
        "record_count": len(records),
        "branch_flips": branch_flips,
        "singularity_spans": singularity_spans,
        "max_command_drift_rad": max(drift_values) if drift_values else None,
        "monotonic_command_drift": monotonic_drift,
        "gate_rejections": gate_rejections,
    }


def analyze(args: argparse.Namespace) -> int:
    records = load_jsonl(args.python_jsonl)
    summary = analyze_python_records(records, branch_threshold_rad=float(args.branch_threshold_rad))
    print(f"Records: {summary['record_count']}")
    print(f"Branch flips detected: {len(summary['branch_flips'])}")
    print(f"Singularity events: {len(summary['singularity_spans'])}")
    print(f"Max command drift rad: {summary['max_command_drift_rad']}")
    print(f"Monotonic command drift: {summary['monotonic_command_drift']}")
    print(f"Gate rejections: {summary['gate_rejections']}")
    if args.rtcore_jsonl:
        print(f"RTCore trace provided: {args.rtcore_jsonl} (wire-seam analysis TODO: inspect pos_counts crossings).")
    return 0


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="command", required=True)
    sub.add_parser("enable-fast-trace").set_defaults(func=enable_fast_trace)
    sub.add_parser("disable-fast-trace").set_defaults(func=disable_fast_trace)
    capture_parser = sub.add_parser("capture")
    capture_parser.add_argument("--api-base", default=DEFAULT_API_BASE)
    capture_parser.add_argument("--hz", type=float, default=50.0)
    capture_parser.add_argument("--output-dir", default=str(DEFAULT_OUTPUT_DIR))
    capture_parser.set_defaults(func=capture)
    analyze_parser = sub.add_parser("analyze")
    analyze_parser.add_argument("python_jsonl")
    analyze_parser.add_argument("rtcore_jsonl", nargs="?")
    analyze_parser.add_argument("--branch-threshold-rad", type=float, default=0.5)
    analyze_parser.set_defaults(func=analyze)
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    return int(args.func(args))


if __name__ == "__main__":
    raise SystemExit(main())
