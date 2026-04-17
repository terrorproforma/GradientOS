#!/usr/bin/env python3
"""Build the committed A6-EC J6 replay fixture from a full watch capture.

Source: `scripts/a6ec_chapter5_probe.py watch ... --axes J6` writes a JSONL
stream at ~1 Hz containing every raw SDO object, every API truth-view
field, and the RTCore metrics snapshot. The full capture is huge (the
reference sweep is ~205 MB over 1189 samples) and, more importantly,
`logs/` is gitignored, so raw captures do not round-trip through CI.

This script trims any watch capture to a small, high-signal JSONL that:

- covers the interesting motion band (seam crossings, multi-turn
  traversal, return-to-zero), skipping the long stationary tail;
- keeps only the fields the Python-side replay tests actually need:
  `6064`, `statusword`, `combined_u4020_22_signed_counts`,
  `U40.1C`, `U40.1E` (for optional cross-checks), plus an
  `expected` sub-object with `canonical_rad`, `raw_counts`,
  `absolute_counts` pulled from the same sample's controller API
  view so Layer B tests have an external ground-truth reference
  without re-deriving it.

The output fixture is deterministic: given the same input file and the
same trim parameters the script regenerates byte-for-byte.

Usage:

  python scripts/build_a6ec_j6_replay_fixture.py \\
      --in logs/encoder-retention/j6-manual-rotate-20260416-053435/j6-manual-rotate-live.watch.jsonl \\
      --out tests/fixtures/a6ec_j6_seam_rotation.jsonl

The defaults match the reference capture and the committed fixture path.
"""

from __future__ import annotations

import argparse
import json
import sys
from collections.abc import Iterable
from pathlib import Path

_DEFAULT_INPUT = Path(
    "logs/encoder-retention/j6-manual-rotate-20260416-053435/j6-manual-rotate-live.watch.jsonl"
)
_DEFAULT_OUTPUT = Path("tests/fixtures/a6ec_j6_seam_rotation.jsonl")

# Reference sweep contained all of its seam crossings within indices 0-120
# and then sat at a stationary value for ~27 minutes. We therefore keep
# every sample through the motion window and then drop to a thin
# stride through the stationary plateau so regression tests still prove
# truth stays stable under encoder wander without paying ~1000 samples of
# redundant capture.
_MOTION_WINDOW_LAST_INDEX = 125
_TAIL_STRIDE = 400  # keep about three tail samples out of ~1060 stationary ones
_TAIL_MAX_SAMPLES = 4


def _iter_source_records(path: Path) -> Iterable[dict[str, object]]:
    with path.open("r", encoding="utf-8") as handle:
        for line in handle:
            line = line.strip()
            if not line:
                continue
            try:
                yield json.loads(line)
            except json.JSONDecodeError as exc:
                raise RuntimeError(
                    f"Failed to parse JSON in {path}: {exc}"
                ) from exc


def _axis_detail_for_j6(record: dict[str, object]) -> dict[str, object] | None:
    controller = record.get("controller")
    if not isinstance(controller, dict):
        return None
    joint_state = controller.get("joint_state")
    if not isinstance(joint_state, dict):
        return None
    payload = joint_state.get("json")
    if not isinstance(payload, dict):
        return None
    axis_absolute_feedback = payload.get("axis_absolute_feedback")
    if not isinstance(axis_absolute_feedback, list):
        return None
    for detail in axis_absolute_feedback:
        if isinstance(detail, dict) and int(detail.get("logical_joint", 0) or 0) == 6:
            return detail
    return None


def _extract_j6_inputs(record: dict[str, object]) -> dict[str, object] | None:
    axes = record.get("axes")
    if not isinstance(axes, dict):
        return None
    j6 = axes.get("J6")
    if not isinstance(j6, dict):
        return None
    counts_6064 = j6.get("6064")
    statusword = j6.get("statusword")
    combined_u40 = j6.get("combined_u4020_22_signed_counts")
    if counts_6064 is None or statusword is None or combined_u40 is None:
        return None
    u40_1c = j6.get("U40.1C")
    u40_1e = j6.get("U40.1E")
    detail = _axis_detail_for_j6(record)
    if detail is None:
        return None
    return {
        "captured_at": record.get("captured_at"),
        "counts_6064": int(counts_6064),
        "statusword": int(statusword),
        "combined_u4020_22_signed_counts": int(combined_u40),
        "u40_1c": None if u40_1c is None else int(u40_1c),
        "u40_1e": None if u40_1e is None else int(u40_1e),
        "expected": {
            "canonical_rad": float(detail.get("canonical_rad", 0.0)),
            "raw_counts": int(detail.get("raw_counts", 0)),
            "absolute_counts": int(detail.get("absolute_counts", 0)),
            "absolute_source": str(detail.get("absolute_source", "")),
            "statusword_hex": str(detail.get("statusword_hex", "")),
        },
    }


def _select_indices(total_samples: int) -> list[int]:
    """Pick the indices to keep from the full capture.

    Keep the whole motion window (indices 0..MOTION_WINDOW_LAST_INDEX
    inclusive), then thin the stationary tail so the fixture stays
    CI-friendly.
    """
    motion_end = min(_MOTION_WINDOW_LAST_INDEX, total_samples - 1)
    indices = list(range(0, motion_end + 1))
    tail_start = motion_end + 1
    if tail_start < total_samples:
        tail = list(range(tail_start, total_samples, _TAIL_STRIDE))[:_TAIL_MAX_SAMPLES]
        indices.extend(tail)
    return indices


def build_fixture(
    source_path: Path,
    output_path: Path,
) -> int:
    """Write the trimmed fixture. Returns the number of replay samples emitted."""
    if not source_path.exists():
        raise FileNotFoundError(f"input capture not found: {source_path}")
    records = list(_iter_source_records(source_path))
    if not records:
        raise RuntimeError(f"{source_path} is empty or has no JSON records")

    selected_indices = _select_indices(len(records))
    emitted: list[dict[str, object]] = []
    for idx in selected_indices:
        rec = records[idx]
        payload = _extract_j6_inputs(rec)
        if payload is None:
            continue
        payload["source_index"] = int(idx)
        emitted.append(payload)
    if not emitted:
        raise RuntimeError(
            f"did not extract any usable J6 samples from {source_path}"
        )

    header = {
        "__kind__": "fixture_meta",
        "source_path": str(source_path),
        "source_total_records": len(records),
        "emitted_sample_count": len(emitted),
        "motion_window_last_index": _MOTION_WINDOW_LAST_INDEX,
        "tail_stride": _TAIL_STRIDE,
        "tail_max_samples": _TAIL_MAX_SAMPLES,
        "schema_version": 1,
    }

    output_path.parent.mkdir(parents=True, exist_ok=True)
    # Stable formatting: sort keys and use compact separators so the
    # regeneration check is byte-for-byte deterministic.
    with output_path.open("w", encoding="utf-8") as out:
        out.write(json.dumps(header, sort_keys=True, separators=(",", ":")))
        out.write("\n")
        for payload in emitted:
            out.write(json.dumps(payload, sort_keys=True, separators=(",", ":")))
            out.write("\n")

    return len(emitted)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description="Trim an A6-EC J6 watch capture into a small replay fixture."
    )
    parser.add_argument(
        "--in",
        dest="input_path",
        type=Path,
        default=_DEFAULT_INPUT,
        help="Path to the full J6 watch JSONL capture (default: reference sweep under logs/).",
    )
    parser.add_argument(
        "--out",
        dest="output_path",
        type=Path,
        default=_DEFAULT_OUTPUT,
        help="Destination fixture path (default: tests/fixtures/a6ec_j6_seam_rotation.jsonl).",
    )
    args = parser.parse_args(argv)
    try:
        emitted = build_fixture(args.input_path, args.output_path)
    except (FileNotFoundError, RuntimeError) as exc:
        print(f"[build_a6ec_j6_replay_fixture] {exc}", file=sys.stderr)
        return 1
    print(
        f"[build_a6ec_j6_replay_fixture] wrote {emitted} samples to {args.output_path}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
