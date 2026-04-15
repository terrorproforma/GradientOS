#!/usr/bin/env python3
"""
Read-only A6-EC Chapter 5 / Section 11 probe harness.

This script captures the raw encoder-unit objects, the reference-unit motion
objects, and the manual-derived bridge formulas so we can test the current
frame model on live hardware without moving the robot.

Typical use:

  python scripts/a6ec_chapter5_probe.py snapshot --label boot --axes J3 J6
  python scripts/a6ec_chapter5_probe.py snapshot --label post-home --axes J3 J6 --experiment-id <id>
  python scripts/a6ec_chapter5_probe.py snapshot --label post-restart --axes J3 J6 --experiment-id <id>

Artifacts are written under:
  logs/encoder-retention/<experiment-id>/
"""

from __future__ import annotations

import argparse
import datetime as dt
import json
import subprocess
import sys
import time
import urllib.error
import urllib.request
from pathlib import Path
from typing import Any


COUNTS_PER_MOTOR_REV = 131072  # 17-bit encoder, per Chapter 5
DEFAULT_API_URL = "http://127.0.0.1:4000"
AXIS_NAME_TO_INDEX = {f"J{joint}": joint - 1 for joint in range(1, 7)}
STANDARD_WANDER_COUNTS = 2.0
MEDIUM_WANDER_COUNTS = 6.0
LARGE_WANDER_COUNTS = 10.0
EXTREME_WANDER_COUNTS = 100.0

SDO_OBJECTS: list[tuple[str, str, str, str]] = [
    ("C00.07", "0x2000", "0x08", "uint16"),
    ("C10.16", "0x2010", "0x17", "uint16"),
    ("C10.18", "0x2010", "0x19", "uint16"),
    ("C10.19", "0x2010", "0x1A", "uint16"),
    ("C10.1A", "0x2010", "0x1B", "uint32"),
    ("C10.1C", "0x2010", "0x1D", "uint32"),
    ("6041", "0x6041", "0x00", "uint16"),
    ("6060", "0x6060", "0x00", "int8"),
    ("6061", "0x6061", "0x00", "int8"),
    ("6062", "0x6062", "0x00", "int32"),
    ("6063", "0x6063", "0x00", "int32"),
    ("6064", "0x6064", "0x00", "int32"),
    ("607A", "0x607A", "0x00", "int32"),
    ("607C", "0x607C", "0x00", "int32"),
    ("6091.01", "0x6091", "0x01", "uint32"),
    ("6091.02", "0x6091", "0x02", "uint32"),
    ("60E6", "0x60E6", "0x00", "uint8"),
    ("60FC", "0x60FC", "0x00", "int32"),
    ("U40.16", "0x2040", "0x17", "int32"),
    ("U40.1C", "0x2040", "0x1D", "int32"),
    ("U40.1E", "0x2040", "0x1F", "uint16"),
    ("U40.20", "0x2040", "0x21", "int32"),
    ("U40.22", "0x2040", "0x23", "int32"),
    ("U40.24", "0x2040", "0x25", "int32"),
    ("U40.26", "0x2040", "0x27", "int32"),
    ("U40.28", "0x2040", "0x29", "int32"),
    ("U40.2A", "0x2040", "0x2B", "int32"),
    ("U40.2C", "0x2040", "0x2D", "int32"),
]


def _repo_root() -> Path:
    return Path(__file__).resolve().parents[1]


def _log_root() -> Path:
    return _repo_root() / "logs" / "encoder-retention"


def _utc_now() -> dt.datetime:
    return dt.datetime.now(dt.timezone.utc)


def _timestamp_token(moment: dt.datetime | None = None) -> str:
    current = moment or _utc_now()
    return current.strftime("%Y%m%d-%H%M%S")


def _normalize_label(value: str) -> str:
    token = "".join(ch if ch.isalnum() or ch in {"-", "_", "."} else "-" for ch in value.strip())
    token = token.strip("-._")
    if not token:
        raise ValueError("label must contain at least one safe filename character")
    return token


def _normalize_experiment_id(value: str | None) -> str:
    if value is None or not value.strip():
        return f"{_timestamp_token()}-a6ec-ch5-probe"
    return _normalize_label(value)


def _sign_extend_u16(value: int) -> int:
    return int(value) if int(value) < 0x8000 else int(value) - 0x10000


def _combine_signed_i64(low_i32: int, high_i32: int) -> int:
    low = int(low_i32) & 0xFFFFFFFF
    high = int(high_i32) & 0xFFFFFFFF
    combined = (high << 32) | low
    if high & 0x80000000:
        combined -= 1 << 64
    return combined


def _combine_unsigned_u64(low_u32: int, high_u32: int) -> int:
    low = int(low_u32) & 0xFFFFFFFF
    high = int(high_u32) & 0xFFFFFFFF
    return (high << 32) | low


def _statusword_bits(statusword: int) -> dict[str, bool]:
    value = int(statusword)
    bit12 = bool(value & (1 << 12))
    bit13 = bool(value & (1 << 13))
    bit15 = bool(value & (1 << 15))
    return {
        "bit10_target_reached": bool(value & (1 << 10)),
        "bit12_homing_attained": bit12,
        "bit13_homing_error": bit13,
        "bit15_reference_attained": bit15,
        "bit15_homing_completed": bit15,
        "vendor_hm_success_signature": bit12 and bit15 and not bit13,
    }


def _read_sdo(axis_index: int, object_index: str, subindex: str, data_type: str) -> dict[str, Any]:
    command = [
        "sudo",
        "ethercat",
        "upload",
        "-p",
        str(axis_index),
        "-t",
        data_type,
        object_index,
        subindex,
    ]
    try:
        raw = subprocess.check_output(command, text=True, stderr=subprocess.STDOUT).strip()
    except subprocess.CalledProcessError as exc:
        return {
            "ok": False,
            "type": data_type,
            "index": object_index,
            "subindex": subindex,
            "command": command,
            "error": (exc.output or str(exc)).strip(),
        }
    tokens = raw.split()
    try:
        value = int(tokens[-1], 0)
    except Exception:
        return {
            "ok": False,
            "type": data_type,
            "index": object_index,
            "subindex": subindex,
            "command": command,
            "raw": raw,
            "error": "could not parse ethercat upload output",
        }
    return {
        "ok": True,
        "type": data_type,
        "index": object_index,
        "subindex": subindex,
        "command": command,
        "raw": raw,
        "value": value,
    }


def _fetch_json(url: str) -> dict[str, Any] | None:
    try:
        with urllib.request.urlopen(url, timeout=2.0) as response:
            return json.load(response)
    except (urllib.error.URLError, urllib.error.HTTPError, TimeoutError, json.JSONDecodeError):
        return None


def _safe_ratio(numerator: int | None, denominator: int | None) -> float | None:
    if numerator is None or denominator is None or int(denominator) == 0:
        return None
    return float(numerator) / float(denominator)


def _value(row: dict[str, Any], key: str) -> int | None:
    entry = row.get(key)
    if not isinstance(entry, dict) or not bool(entry.get("ok")):
        return None
    try:
        return int(entry.get("value"))
    except Exception:
        return None


def _within_one_count(left: float | int | None, right: float | int | None) -> bool | None:
    if left is None or right is None:
        return None
    return abs(float(left) - float(right)) <= 1.0


def _classify_count_delta(delta: float | int | None) -> str | None:
    if delta is None:
        return None
    magnitude = abs(float(delta))
    if magnitude <= STANDARD_WANDER_COUNTS:
        return "standard"
    if magnitude <= MEDIUM_WANDER_COUNTS:
        return "medium"
    if magnitude <= LARGE_WANDER_COUNTS:
        return "large"
    if magnitude <= EXTREME_WANDER_COUNTS:
        return "excessive"
    return "extreme"


def _delta_summary(delta: float | int | None) -> dict[str, float | str | None]:
    if delta is None:
        return {"delta": None, "abs_delta": None, "category": None}
    delta_float = float(delta)
    return {
        "delta": delta_float,
        "abs_delta": abs(delta_float),
        "category": _classify_count_delta(delta_float),
    }


def _collect_axis_snapshot(axis_name: str, axis_index: int) -> dict[str, Any]:
    row: dict[str, Any] = {
        "axis_name": axis_name,
        "axis_index": axis_index,
        "reads": {},
    }
    for label, object_index, subindex, data_type in SDO_OBJECTS:
        row["reads"][label] = _read_sdo(axis_index, object_index, subindex, data_type)

    reads = row["reads"]
    statusword = _value(reads, "6041")
    if statusword is not None:
        row["statusword_bits"] = _statusword_bits(statusword)

    u401c = _value(reads, "U40.1C")
    u401e = _value(reads, "U40.1E")
    u4020 = _value(reads, "U40.20")
    u4022 = _value(reads, "U40.22")
    u4024 = _value(reads, "U40.24")
    u4026 = _value(reads, "U40.26")
    u4028 = _value(reads, "U40.28")
    u402a = _value(reads, "U40.2A")
    u402c = _value(reads, "U40.2C")
    v6062 = _value(reads, "6062")
    v6063 = _value(reads, "6063")
    v6064 = _value(reads, "6064")
    v60fc = _value(reads, "60FC")
    v6091_01 = _value(reads, "6091.01")
    v6091_02 = _value(reads, "6091.02")
    c10_18 = _value(reads, "C10.18")
    c10_19 = _value(reads, "C10.19")
    c10_1a = _value(reads, "C10.1A")
    c10_1c = _value(reads, "C10.1C")

    signed_multiturn_revs = _sign_extend_u16(u401e) if u401e is not None else None
    single_turn_mod = (int(u401c) % COUNTS_PER_MOTOR_REV) if u401c is not None else None
    reconstructed_abs_counts = (
        (int(signed_multiturn_revs) * COUNTS_PER_MOTOR_REV) + int(single_turn_mod)
        if signed_multiturn_revs is not None and single_turn_mod is not None
        else None
    )
    combined_u4020_22 = _combine_signed_i64(u4020, u4022) if u4020 is not None and u4022 is not None else None
    combined_u4024_26 = _combine_signed_i64(u4024, u4026) if u4024 is not None and u4026 is not None else None
    combined_u402a_2c = _combine_signed_i64(u402a, u402c) if u402a is not None and u402c is not None else None
    upper_limit_rotation_counts = (
        _combine_unsigned_u64(c10_1a, c10_1c) if c10_1a is not None and c10_1c is not None else None
    )

    ratio_6091 = _safe_ratio(v6091_01, v6091_02)
    ratio_c10_rotation = _safe_ratio(c10_18, c10_19)

    expected_6063_from_6064 = (float(v6064) * float(ratio_6091)) if v6064 is not None and ratio_6091 is not None else None
    expected_60fc_from_6062 = (float(v6062) * float(ratio_6091)) if v6062 is not None and ratio_6091 is not None else None
    expected_u4024_26_from_u4016 = (
        float(_value(reads, "U40.16")) * float(ratio_6091)
        if _value(reads, "U40.16") is not None and ratio_6091 is not None
        else None
    )
    expected_u402a_2c_from_u4028 = (
        float(u4028) * float(ratio_c10_rotation)
        if u4028 is not None and ratio_c10_rotation is not None
        else None
    )

    delta_u4020_22_vs_reconstructed = (
        None if combined_u4020_22 is None or reconstructed_abs_counts is None else combined_u4020_22 - reconstructed_abs_counts
    )
    delta_6063_vs_6064_times_6091 = (
        None if v6063 is None or expected_6063_from_6064 is None else float(v6063) - float(expected_6063_from_6064)
    )
    delta_60fc_vs_6062_times_6091 = (
        None if v60fc is None or expected_60fc_from_6062 is None else float(v60fc) - float(expected_60fc_from_6062)
    )
    delta_u4024_26_vs_u4016_times_6091 = (
        None
        if combined_u4024_26 is None or expected_u4024_26_from_u4016 is None
        else float(combined_u4024_26) - float(expected_u4024_26_from_u4016)
    )
    delta_u402a_2c_vs_u4028_times_c10_ratio = (
        None
        if combined_u402a_2c is None or expected_u402a_2c_from_u4028 is None
        else float(combined_u402a_2c) - float(expected_u402a_2c_from_u4028)
    )

    row["derived"] = {
        "counts_per_motor_rev": COUNTS_PER_MOTOR_REV,
        "signed_multiturn_revolutions_candidate": signed_multiturn_revs,
        "single_turn_mod_counts": single_turn_mod,
        "reconstructed_abs_counts_from_u401c_u401e": reconstructed_abs_counts,
        "combined_u4020_22_signed_counts": combined_u4020_22,
        "combined_u4024_26_signed_counts": combined_u4024_26,
        "combined_u402a_2c_signed_counts": combined_u402a_2c,
        "upper_limit_rotation_counts": upper_limit_rotation_counts,
        "ratio_6091_motor_over_shaft": ratio_6091,
        "ratio_c10_rotation_mode": ratio_c10_rotation,
        "expected_6063_from_6064_times_6091": expected_6063_from_6064,
        "expected_60fc_from_6062_times_6091": expected_60fc_from_6062,
        "expected_u4024_26_from_u4016_times_6091": expected_u4024_26_from_u4016,
        "expected_u402a_2c_from_u4028_times_c10_ratio": expected_u402a_2c_from_u4028,
        "delta_u4020_22_vs_reconstructed": delta_u4020_22_vs_reconstructed,
        "delta_u4020_22_vs_reconstructed_abs": _delta_summary(delta_u4020_22_vs_reconstructed)["abs_delta"],
        "delta_u4020_22_vs_reconstructed_category": _delta_summary(delta_u4020_22_vs_reconstructed)["category"],
        "delta_6063_vs_6064_times_6091": delta_6063_vs_6064_times_6091,
        "delta_6063_vs_6064_times_6091_abs": _delta_summary(delta_6063_vs_6064_times_6091)["abs_delta"],
        "delta_6063_vs_6064_times_6091_category": _delta_summary(delta_6063_vs_6064_times_6091)["category"],
        "delta_60fc_vs_6062_times_6091": delta_60fc_vs_6062_times_6091,
        "delta_60fc_vs_6062_times_6091_abs": _delta_summary(delta_60fc_vs_6062_times_6091)["abs_delta"],
        "delta_60fc_vs_6062_times_6091_category": _delta_summary(delta_60fc_vs_6062_times_6091)["category"],
        "delta_u4024_26_vs_u4016_times_6091": delta_u4024_26_vs_u4016_times_6091,
        "delta_u4024_26_vs_u4016_times_6091_abs": _delta_summary(delta_u4024_26_vs_u4016_times_6091)["abs_delta"],
        "delta_u4024_26_vs_u4016_times_6091_category": _delta_summary(delta_u4024_26_vs_u4016_times_6091)["category"],
        "delta_u402a_2c_vs_u4028_times_c10_ratio": delta_u402a_2c_vs_u4028_times_c10_ratio,
        "delta_u402a_2c_vs_u4028_times_c10_ratio_abs": _delta_summary(delta_u402a_2c_vs_u4028_times_c10_ratio)["abs_delta"],
        "delta_u402a_2c_vs_u4028_times_c10_ratio_category": _delta_summary(delta_u402a_2c_vs_u4028_times_c10_ratio)["category"],
        "matches_u4020_22_formula_within_one_count": _within_one_count(combined_u4020_22, reconstructed_abs_counts),
        "matches_6063_bridge_within_one_count": _within_one_count(v6063, expected_6063_from_6064),
        "matches_60fc_bridge_within_one_count": _within_one_count(v60fc, expected_60fc_from_6062),
        "matches_u4024_26_bridge_within_one_count": _within_one_count(combined_u4024_26, expected_u4024_26_from_u4016),
        "matches_u402a_2c_bridge_within_one_count": _within_one_count(combined_u402a_2c, expected_u402a_2c_from_u4028),
    }
    return row


def _render_markdown(snapshot: dict[str, Any]) -> str:
    lines = [
        "# A6-EC Chapter 5 Probe",
        "",
        f"- Experiment: `{snapshot['experiment_id']}`",
        f"- Label: `{snapshot['label']}`",
        f"- Captured at: `{snapshot['captured_at']}`",
        "",
        "## Manual hypotheses",
        "",
        "- Raw motor absolute counts should match `sign_extend16(U40.1E) * 131072 + (U40.1C mod 131072)`.",
        "- `6063 ~= 6064 * 6091` when the drive is exposing encoder-unit/reference-unit feedback consistently.",
        "- `60FC ~= 6062 * 6091` when the encoder-unit reference bridge is active.",
        "- `U40.2A/.2C ~= U40.28 * (C10.18 / C10.19)` for rotation-mode encoder/reference feedback.",
        "",
    ]
    api_snapshot = snapshot.get("api_joints_detailed")
    if isinstance(api_snapshot, dict):
        lines.extend(
            [
                "## API snapshot",
                "",
                f"- Global truth available: `{api_snapshot.get('canonical_joint_truth_available')}`",
                f"- Unavailable joints: `{api_snapshot.get('canonical_joint_truth_unavailable_joints')}`",
                "",
            ]
        )
    for axis_name, axis_snapshot in snapshot["axes"].items():
        derived = axis_snapshot.get("derived", {})
        statusword_bits = axis_snapshot.get("statusword_bits", {})
        lines.extend(
            [
                f"## {axis_name}",
                "",
                f"- Axis index: `{axis_snapshot.get('axis_index')}`",
                f"- Vendor HM success bits valid: `{statusword_bits.get('vendor_hm_success_signature') if isinstance(statusword_bits, dict) else None}`",
                f"- Statusword bits: `{axis_snapshot.get('statusword_bits')}`",
                f"- `6091 = {derived.get('ratio_6091_motor_over_shaft')}`",
                f"- `C10.18/C10.19 = {derived.get('ratio_c10_rotation_mode')}`",
                f"- Raw formula match: `{derived.get('matches_u4020_22_formula_within_one_count')}`"
                f" delta=`{derived.get('delta_u4020_22_vs_reconstructed')}`"
                f" category=`{derived.get('delta_u4020_22_vs_reconstructed_category')}`",
                f"- `6063 ~= 6064*6091`: `{derived.get('matches_6063_bridge_within_one_count')}`"
                f" delta=`{derived.get('delta_6063_vs_6064_times_6091')}`"
                f" category=`{derived.get('delta_6063_vs_6064_times_6091_category')}`",
                f"- `60FC ~= 6062*6091`: `{derived.get('matches_60fc_bridge_within_one_count')}`"
                f" delta=`{derived.get('delta_60fc_vs_6062_times_6091')}`"
                f" category=`{derived.get('delta_60fc_vs_6062_times_6091_category')}`",
                f"- `U40.2A/.2C ~= U40.28*C10`: `{derived.get('matches_u402a_2c_bridge_within_one_count')}`"
                f" delta=`{derived.get('delta_u402a_2c_vs_u4028_times_c10_ratio')}`"
                f" category=`{derived.get('delta_u402a_2c_vs_u4028_times_c10_ratio_category')}`",
                "",
            ]
        )
    return "\n".join(lines)


def _select_api_axis_details(api_payload: dict[str, Any] | None, axis_names: list[str]) -> dict[str, Any] | None:
    if not isinstance(api_payload, dict):
        return None
    details = api_payload.get("axis_absolute_feedback")
    if not isinstance(details, list):
        return {
            "canonical_joint_truth_available": api_payload.get("canonical_joint_truth_available"),
            "canonical_joint_truth_unavailable_joints": api_payload.get("canonical_joint_truth_unavailable_joints"),
        }
    selected: dict[str, Any] = {}
    for axis_name in axis_names:
        axis_index = AXIS_NAME_TO_INDEX[axis_name]
        if 0 <= axis_index < len(details):
            selected[axis_name] = details[axis_index]
    return {
        "canonical_joint_truth_available": api_payload.get("canonical_joint_truth_available"),
        "canonical_joint_truth_unavailable_joints": api_payload.get("canonical_joint_truth_unavailable_joints"),
        "selected_axis_absolute_feedback": selected,
    }


def _capture_snapshot(*, label: str, experiment_id: str, axis_names: list[str], api_url: str) -> dict[str, Any]:
    axis_snapshots = {axis_name: _collect_axis_snapshot(axis_name, AXIS_NAME_TO_INDEX[axis_name]) for axis_name in axis_names}
    api_payload = _fetch_json(f"{api_url.rstrip('/')}/info/joints-detailed")
    return {
        "experiment_id": experiment_id,
        "label": label,
        "captured_at": _utc_now().isoformat(timespec="seconds"),
        "manual_notes": {
            "counts_per_motor_rev": COUNTS_PER_MOTOR_REV,
            "chapter5_absolute_encoder": "17-bit single turn + 16-bit multi-turn revolutions",
            "section11_motion_units": "6062/6064/607A are reference-unit objects; 6063/60FC are encoder-unit bridge objects",
        },
        "axes": axis_snapshots,
        "api_joints_detailed": _select_api_axis_details(api_payload, axis_names),
    }


def _build_watch_axis_sample(
    axis_name: str,
    axis_snapshot: dict[str, Any],
    api_payload: dict[str, Any] | None,
) -> dict[str, Any]:
    reads = axis_snapshot.get("reads", {})
    derived = axis_snapshot.get("derived", {})
    statusword = _value(reads, "6041")
    axis_index = int(axis_snapshot.get("axis_index", AXIS_NAME_TO_INDEX[axis_name]))
    statusword_bits = axis_snapshot.get("statusword_bits", {})

    result: dict[str, Any] = {
        "axis_index": axis_index,
        "statusword": statusword,
        "statusword_hex": f"0x{int(statusword) & 0xFFFF:04X}" if statusword is not None else None,
        "statusword_bits": statusword_bits,
        "vendor_hm_success_signature": (
            bool(statusword_bits.get("vendor_hm_success_signature"))
            if isinstance(statusword_bits, dict)
            else None
        ),
        "6064": _value(reads, "6064"),
        "607C": _value(reads, "607C"),
        "U40.16": _value(reads, "U40.16"),
        "U40.28": _value(reads, "U40.28"),
        "6063": _value(reads, "6063"),
        "60FC": _value(reads, "60FC"),
        "combined_u4020_22_signed_counts": derived.get("combined_u4020_22_signed_counts"),
        "combined_u402a_2c_signed_counts": derived.get("combined_u402a_2c_signed_counts"),
    }

    if not isinstance(api_payload, dict):
        return result

    result["api_read_source"] = api_payload.get("read_source")
    result["api_canonical_joint_truth_available"] = api_payload.get("canonical_joint_truth_available")
    result["api_raw_canonical_joint_truth_available"] = api_payload.get("raw_canonical_joint_truth_available")
    result["api_display_joint_truth_available"] = api_payload.get("display_joint_truth_available")

    for key in ("arm_deg", "arm_display_deg", "arm_rad", "arm_display_rad"):
        values = api_payload.get(key)
        if isinstance(values, list) and 0 <= axis_index < len(values):
            result[f"api_{key}"] = values[axis_index]

    axis_absolute_feedback = api_payload.get("axis_absolute_feedback")
    if isinstance(axis_absolute_feedback, list) and 0 <= axis_index < len(axis_absolute_feedback):
        entry = axis_absolute_feedback[axis_index]
        if isinstance(entry, dict):
            for key in (
                "truth_available",
                "truth_status",
                "truth_reason",
                "display_source",
                "absolute_source",
                "canonical_rad",
                "display_rad",
                "absolute_counts",
                "raw_counts",
                "reference_mode",
            ):
                if key in entry:
                    result[f"api_{key}"] = entry.get(key)
    return result


def _capture_watch_sample(*, axis_names: list[str], api_url: str) -> dict[str, Any]:
    api_payload = _fetch_json(f"{api_url.rstrip('/')}/info/joints-detailed")
    axes: dict[str, Any] = {}
    for axis_name in axis_names:
        axis_snapshot = _collect_axis_snapshot(axis_name, AXIS_NAME_TO_INDEX[axis_name])
        axes[axis_name] = _build_watch_axis_sample(axis_name, axis_snapshot, api_payload)
    return {
        "captured_at": _utc_now().isoformat(timespec="milliseconds"),
        "api_read_source": api_payload.get("read_source") if isinstance(api_payload, dict) else None,
        "api_canonical_joint_truth_available": (
            api_payload.get("canonical_joint_truth_available") if isinstance(api_payload, dict) else None
        ),
        "axes": axes,
    }


def _format_watch_line(sample: dict[str, Any]) -> str:
    timestamp = str(sample.get("captured_at", "")).strip() or "unknown-time"
    axis_parts: list[str] = []
    axes = sample.get("axes", {})
    if isinstance(axes, dict):
        for axis_name, axis in axes.items():
            if not isinstance(axis, dict):
                continue
            axis_parts.append(
                " ".join(
                    [
                        str(axis_name),
                        f"6064={axis.get('6064')}",
                        f"607C={axis.get('607C')}",
                        f"U40.16={axis.get('U40.16')}",
                        f"abs={axis.get('combined_u4020_22_signed_counts')}",
                        f"rot={axis.get('combined_u402a_2c_signed_counts')}",
                        f"api_deg={axis.get('api_arm_deg')}",
                        f"api_display_deg={axis.get('api_arm_display_deg')}",
                        f"hm_ok={axis.get('vendor_hm_success_signature')}",
                    ]
                )
            )
    return f"{timestamp} | {' | '.join(axis_parts)}"


def _parse_axes(tokens: list[str]) -> list[str]:
    result: list[str] = []
    for token in tokens:
        for part in token.split(","):
            axis_name = part.strip().upper()
            if not axis_name:
                continue
            if axis_name not in AXIS_NAME_TO_INDEX:
                raise ValueError(f"Unsupported axis name: {axis_name}")
            if axis_name not in result:
                result.append(axis_name)
    if not result:
        raise ValueError("At least one axis must be supplied")
    return result


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    subparsers = parser.add_subparsers(dest="command", required=True)

    snapshot_parser = subparsers.add_parser("snapshot", help="capture a Chapter 5 probe snapshot")
    snapshot_parser.add_argument("--label", required=True, help="phase label, for example boot or post-home")
    snapshot_parser.add_argument(
        "--axes",
        nargs="+",
        default=["J3", "J6"],
        help="axis names such as J3 J6 or J3,J6",
    )
    snapshot_parser.add_argument("--experiment-id", default=None, help="reuse an existing experiment directory")
    snapshot_parser.add_argument("--api-url", default=DEFAULT_API_URL, help="API base URL for optional joint snapshot")

    watch_parser = subparsers.add_parser("watch", help="stream live Chapter 5 probe samples to JSONL")
    watch_parser.add_argument("--label", required=True, help="stream label, for example j6-hand-rotate")
    watch_parser.add_argument(
        "--axes",
        nargs="+",
        default=["J6"],
        help="axis names such as J6 or J3,J6",
    )
    watch_parser.add_argument("--experiment-id", default=None, help="reuse an existing experiment directory")
    watch_parser.add_argument("--api-url", default=DEFAULT_API_URL, help="API base URL for optional joint snapshot")
    watch_parser.add_argument(
        "--interval-s",
        type=float,
        default=0.25,
        help="target sample interval in seconds (default: 0.25)",
    )
    watch_parser.add_argument(
        "--duration-s",
        type=float,
        default=None,
        help="optional max capture duration in seconds; omit to run until Ctrl-C",
    )
    watch_parser.add_argument(
        "--samples",
        type=int,
        default=None,
        help="optional max sample count; omit to run until Ctrl-C",
    )
    watch_parser.add_argument(
        "--quiet",
        action="store_true",
        help="write JSONL samples without printing condensed live lines",
    )

    args = parser.parse_args(argv)
    label = _normalize_label(args.label)
    experiment_id = _normalize_experiment_id(args.experiment_id)
    axis_names = _parse_axes(list(args.axes))
    experiment_dir = _log_root() / experiment_id
    experiment_dir.mkdir(parents=True, exist_ok=True)

    if args.command == "snapshot":
        snapshot = _capture_snapshot(
            label=label,
            experiment_id=experiment_id,
            axis_names=axis_names,
            api_url=str(args.api_url),
        )

        json_path = experiment_dir / f"{label}.json"
        md_path = experiment_dir / f"{label}.md"
        json_path.write_text(json.dumps(snapshot, indent=2) + "\n", encoding="utf-8")
        md_path.write_text(_render_markdown(snapshot), encoding="utf-8")

        condensed = {
            "experiment_id": experiment_id,
            "label": label,
            "captured_at": snapshot["captured_at"],
            "json_path": str(json_path),
            "markdown_path": str(md_path),
            "axes": {},
        }
        for axis_name in axis_names:
            derived = snapshot["axes"][axis_name]["derived"]
            condensed["axes"][axis_name] = {
                "ratio_6091_motor_over_shaft": derived.get("ratio_6091_motor_over_shaft"),
                "ratio_c10_rotation_mode": derived.get("ratio_c10_rotation_mode"),
                "raw_formula_match": derived.get("matches_u4020_22_formula_within_one_count"),
                "raw_formula_delta_counts": derived.get("delta_u4020_22_vs_reconstructed"),
                "raw_formula_delta_category": derived.get("delta_u4020_22_vs_reconstructed_category"),
                "bridge_6063_from_6064_match": derived.get("matches_6063_bridge_within_one_count"),
                "bridge_6063_from_6064_delta": derived.get("delta_6063_vs_6064_times_6091"),
                "bridge_6063_from_6064_delta_category": derived.get("delta_6063_vs_6064_times_6091_category"),
                "bridge_60fc_from_6062_match": derived.get("matches_60fc_bridge_within_one_count"),
                "bridge_60fc_from_6062_delta": derived.get("delta_60fc_vs_6062_times_6091"),
                "bridge_60fc_from_6062_delta_category": derived.get("delta_60fc_vs_6062_times_6091_category"),
                "bridge_u402a_from_u4028_match": derived.get("matches_u402a_2c_bridge_within_one_count"),
                "bridge_u402a_from_u4028_delta": derived.get("delta_u402a_2c_vs_u4028_times_c10_ratio"),
                "bridge_u402a_from_u4028_delta_category": derived.get("delta_u402a_2c_vs_u4028_times_c10_ratio_category"),
            }
        print(json.dumps(condensed, indent=2))
        return 0

    if args.command != "watch":
        parser.error("unsupported command")

    interval_s = max(0.05, float(args.interval_s))
    duration_s = float(args.duration_s) if args.duration_s is not None else None
    max_samples = int(args.samples) if args.samples is not None else None
    jsonl_path = experiment_dir / f"{label}.watch.jsonl"
    meta_path = experiment_dir / f"{label}.watch-meta.json"
    meta = {
        "experiment_id": experiment_id,
        "label": label,
        "axes": axis_names,
        "interval_s": interval_s,
        "duration_s": duration_s,
        "samples": max_samples,
        "api_url": str(args.api_url),
        "jsonl_path": str(jsonl_path),
        "started_at": _utc_now().isoformat(timespec="seconds"),
    }
    meta_path.write_text(json.dumps(meta, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(meta, indent=2))

    started_monotonic = time.monotonic()
    captured = 0
    try:
        with jsonl_path.open("a", encoding="utf-8") as handle:
            while True:
                loop_started = time.monotonic()
                sample = _capture_watch_sample(axis_names=axis_names, api_url=str(args.api_url))
                handle.write(json.dumps(sample) + "\n")
                handle.flush()
                captured += 1
                if not args.quiet:
                    print(_format_watch_line(sample), flush=True)
                if max_samples is not None and captured >= max_samples:
                    break
                if duration_s is not None and (time.monotonic() - started_monotonic) >= duration_s:
                    break
                sleep_s = interval_s - (time.monotonic() - loop_started)
                if sleep_s > 0:
                    time.sleep(sleep_s)
    except KeyboardInterrupt:
        return 130
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
