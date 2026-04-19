"""Unit tests for scripts/j6_multiturn_fast_capture.py.

These tests exercise the pure-Python helpers:
  - i32+i32 -> signed i64 combination
  - ethercat-upload output parsing, both observed shapes
  - DS402 statusword Fault pattern
  - the whip vs clean verdict from `_analyze_jsonl` against synthetic JSONL

The capture loop itself is not tested here because it spawns `sudo ethercat`
subprocesses and requires the live EtherCAT master; live smoke tests happen
separately per Phase 2 of the plan.
"""

from __future__ import annotations

import importlib.util
import json
import sys
from pathlib import Path

import pytest


def _load_capture_module():
    repo_root = Path(__file__).resolve().parents[1]
    module_path = repo_root / "scripts" / "j6_multiturn_fast_capture.py"
    spec = importlib.util.spec_from_file_location("j6_multiturn_fast_capture", module_path)
    assert spec is not None
    assert spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    # Register in sys.modules before exec so @dataclasses.dataclass can find
    # its parent module via cls.__module__.
    sys.modules.setdefault("j6_multiturn_fast_capture", module)
    spec.loader.exec_module(module)
    return module


cap = _load_capture_module()


# =========================================================================
# _combine_signed_i64
# =========================================================================

def test_combine_signed_i64_zero():
    assert cap._combine_signed_i64(0, 0) == 0


def test_combine_signed_i64_low_only_positive():
    assert cap._combine_signed_i64(12345, 0) == 12345


def test_combine_signed_i64_low_appears_negative_when_high_is_zero():
    """Low i32 bit pattern 0xFFFFB8E4 (-18,204 as signed i32) with high=0 must
    sign-extend based on the HIGH word, not the low word. Combined value is
    +4,294,949,092 (positive) because high=0 means not negative."""
    low = 0xFFFFB8E4  # unsigned
    assert cap._combine_signed_i64(low, 0) == 0x00000000FFFFB8E4


def test_combine_signed_i64_negative_with_high_sign_bit():
    """High word has 0x80000000 bit set -> combined is negative."""
    low = 0xFFFFB8E4
    high = 0xFFFFFFFF  # all ones
    combined = cap._combine_signed_i64(low, high)
    assert combined == -((1 << 64) - ((high << 32) | low))
    assert combined < 0


def test_combine_signed_i64_handles_live_a6ec_sample():
    """From DEVLOG 2026-04-19 05:12: live wire read for Move B+:
       U40.20 = 0xfff6bf68 (unsigned), U40.22 = 0xffffffff (unsigned)
       i.e. -595,624 combined as int64 = -606,360.
    """
    low = 0xFFF6BF68
    high = 0xFFFFFFFF
    combined = cap._combine_signed_i64(low, high)
    assert combined == -606_360


def test_combine_signed_i64_none_inputs():
    assert cap._combine_signed_i64(None, 0) is None
    assert cap._combine_signed_i64(0, None) is None
    assert cap._combine_signed_i64(None, None) is None


# =========================================================================
# _parse_ethercat_value
# =========================================================================

def test_parse_ethercat_value_hex_decimal_shape():
    assert cap._parse_ethercat_value("0x00000BEEF 48879", signed=False, width_bytes=4) == 48879


def test_parse_ethercat_value_decimal_only_shape():
    assert cap._parse_ethercat_value("48879", signed=False, width_bytes=4) == 48879


def test_parse_ethercat_value_signed_i32_negative():
    # 0xFFFFFFFD as unsigned is 4,294,967,293; as signed i32 is -3
    assert cap._parse_ethercat_value("0xfffffffd 4294967293", signed=True, width_bytes=4) == -3


def test_parse_ethercat_value_signed_i32_positive_high_bit_clear():
    assert cap._parse_ethercat_value("0x7fffffff 2147483647", signed=True, width_bytes=4) == 2147483647


def test_parse_ethercat_value_signed_i16_negative():
    # 0xFF00 as signed i16 = -256
    assert cap._parse_ethercat_value("0xff00 65280", signed=True, width_bytes=2) == -256


def test_parse_ethercat_value_unsigned_u16_no_sign_extend():
    # 603F manufacturer error is uint16; 0xFF00 stays 65280
    assert cap._parse_ethercat_value("0xff00 65280", signed=False, width_bytes=2) == 65280


def test_parse_ethercat_value_empty_returns_none():
    assert cap._parse_ethercat_value("", signed=False, width_bytes=4) is None
    assert cap._parse_ethercat_value("   ", signed=False, width_bytes=4) is None


def test_parse_ethercat_value_malformed_returns_none():
    assert cap._parse_ethercat_value("not a number", signed=False, width_bytes=4) is None


# =========================================================================
# _statusword_is_fault
# =========================================================================

def test_statusword_none_is_not_fault():
    assert cap._statusword_is_fault(None) is False


def test_statusword_operation_enabled_is_not_fault():
    # 0x9637 was the live operation-enabled statusword for J6 per DEVLOG
    assert cap._statusword_is_fault(0x9637) is False


def test_statusword_fault_ds402_pattern():
    # DS402 Fault: (SW & 0x004F) == 0x0008 exactly.
    # 0x9638 is the live "faulted" statusword we saw after Move B Er87.1.
    assert cap._statusword_is_fault(0x9638) is True
    assert cap._statusword_is_fault(0x0008) is True
    # 0x0048 = Fault Reaction Active in DS402 (bit 6 + fault bit) - not the
    # pure Fault state.
    assert cap._statusword_is_fault(0x0048) is False


def test_statusword_switch_on_disabled_is_not_fault():
    # Switched-on disabled pattern: 0x40 - does not match fault mask
    assert cap._statusword_is_fault(0x0250) is False


# =========================================================================
# _analyze_jsonl
# =========================================================================

def _write_jsonl(path: Path, samples: list[dict]) -> None:
    path.write_text("\n".join(json.dumps(s) for s in samples) + "\n")


def test_analyze_jsonl_clean_short_path_motion(tmp_path: Path):
    """Clean monotonic motion: cumulative == |net|, ratio = 1.0, no whip."""
    jsonl = tmp_path / "clean.jsonl"
    samples = [
        {"t_mono_ns": i * 10_000_000, "mt_i64": 0 + i * 1_000,
         "c6064": 100, "c607A": 100, "c603F": 0, "c6041": 0x9637}
        for i in range(51)  # 50 steps of +1000 = +50,000 net
    ]
    _write_jsonl(jsonl, samples)
    result = cap._analyze_jsonl(jsonl)
    assert result.sample_count == 51
    assert result.first_mt == 0
    assert result.last_mt == 50_000
    assert result.net_mt_delta == 50_000
    assert result.cumulative_travel == 50_000
    assert result.ratio_cumulative_over_abs_net == pytest.approx(1.0)
    assert result.monotonic_within_budget is True
    assert result.fault_seen is False


def test_analyze_jsonl_detects_whip(tmp_path: Path):
    """Whip motion: motor goes +50k forward, then -40k back, net +10k.
    Cumulative = 90k, |net| = 10k, ratio = 9.0. Verdict = WHIP.
    """
    jsonl = tmp_path / "whip.jsonl"
    samples: list[dict] = []
    t = 0
    for i in range(51):  # +50k in 50 steps
        samples.append({"t_mono_ns": t, "mt_i64": i * 1000,
                        "c6064": 100, "c607A": 100, "c603F": 0, "c6041": 0x9637})
        t += 10_000_000
    for i in range(41):  # -40k in 40 steps, starting from +50k
        samples.append({"t_mono_ns": t, "mt_i64": 50_000 - i * 1000,
                        "c6064": 100, "c607A": 100, "c603F": 0, "c6041": 0x9637})
        t += 10_000_000
    _write_jsonl(jsonl, samples)
    result = cap._analyze_jsonl(jsonl)
    assert result.first_mt == 0
    assert result.last_mt == 10_000
    assert result.net_mt_delta == 10_000
    assert result.cumulative_travel == 90_000
    assert result.ratio_cumulative_over_abs_net == pytest.approx(9.0)
    # Not monotonic: direction flipped mid-trace.
    assert result.monotonic_within_budget is False


def test_analyze_jsonl_detects_fault_sample(tmp_path: Path):
    """Fault shows up on sample 5 via 603F=0xFF00."""
    jsonl = tmp_path / "fault.jsonl"
    samples = [
        {"t_mono_ns": i * 10_000_000, "mt_i64": i * 1_000,
         "c6064": 100, "c607A": 100,
         "c603F": 0x0000 if i < 5 else 0xFF00,
         "c6041": 0x9637 if i < 5 else 0x9638}
        for i in range(10)
    ]
    _write_jsonl(jsonl, samples)
    result = cap._analyze_jsonl(jsonl)
    assert result.fault_seen is True
    assert result.fault_sample_index == 5
    assert "0xFF00" in (result.fault_detail or "")


def test_analyze_jsonl_small_reversals_within_budget_are_not_whip(tmp_path: Path):
    """Small dither < overshoot_budget is allowed and does not break monotonic."""
    jsonl = tmp_path / "dither.jsonl"
    # Overall +50k, but each step is +1000 with occasional -100 jitter (< budget 500)
    samples = []
    for i in range(51):
        jitter = -100 if (i % 5 == 0 and i > 0) else 1000
        samples.append({
            "t_mono_ns": i * 10_000_000,
            "mt_i64": sum(1000 if j != 0 and j % 5 != 0 else (0 if j == 0 else -100)
                          for j in range(i + 1)) if i > 0 else 0,
            "c6064": 100, "c607A": 100, "c603F": 0, "c6041": 0x9637,
        })
    # Simpler: build the cumulative sum directly.
    mts = [0]
    for i in range(1, 51):
        step = -100 if (i % 5 == 0) else 1000
        mts.append(mts[-1] + step)
    samples = [
        {"t_mono_ns": i * 10_000_000, "mt_i64": mts[i],
         "c6064": 100, "c607A": 100, "c603F": 0, "c6041": 0x9637}
        for i in range(len(mts))
    ]
    _write_jsonl(jsonl, samples)
    result = cap._analyze_jsonl(jsonl, overshoot_budget_counts=500)
    assert result.monotonic_within_budget is True
    assert result.fault_seen is False


def test_analyze_jsonl_empty_file_raises(tmp_path: Path):
    jsonl = tmp_path / "empty.jsonl"
    jsonl.write_text("")
    with pytest.raises(ValueError):
        cap._analyze_jsonl(jsonl)


# =========================================================================
# Output-deg conversion
# =========================================================================

def test_counts_to_output_deg_one_motor_rev_at_ratio_10():
    """131072 motor counts == 1 motor rev. With gear ratio 10, output is 36 deg."""
    deg = cap._counts_to_output_deg(131072, gear_ratio=10.0)
    assert deg == pytest.approx(36.0)


def test_counts_to_output_deg_full_output_turn_at_ratio_10():
    """10 motor revs == 1 full output turn. 131072 * 10 counts -> 360 deg output."""
    deg = cap._counts_to_output_deg(131072 * 10, gear_ratio=10.0)
    assert deg == pytest.approx(360.0)


def test_counts_to_output_deg_ten_deg_output_signed():
    """A +10 deg output move at gear 10 is ~36,408.88... motor counts. Signed."""
    counts = int(round(131072 * 10 * 10 / 360))  # ~36,409
    deg = cap._counts_to_output_deg(counts, gear_ratio=10.0)
    assert deg == pytest.approx(10.0, abs=0.01)


# =========================================================================
# RTCore fast_trace analyzer: _unwrap_wire_delta
# =========================================================================

RM = 131072 * 10  # J6 rotation-mode period on A6-EC


def test_unwrap_wire_delta_small_delta_passthrough():
    assert cap._unwrap_wire_delta(1000, RM) == 1000
    assert cap._unwrap_wire_delta(-1000, RM) == -1000
    assert cap._unwrap_wire_delta(0, RM) == 0


def test_unwrap_wire_delta_positive_wrap():
    """Raw delta RM-10 is really -10 (short path backward through 0)."""
    assert cap._unwrap_wire_delta(RM - 10, RM) == -10


def test_unwrap_wire_delta_negative_wrap():
    """Raw delta -(RM-10) is really +10 (short path forward through 0)."""
    assert cap._unwrap_wire_delta(-(RM - 10), RM) == 10


def test_unwrap_wire_delta_rm_zero_passthrough():
    assert cap._unwrap_wire_delta(500000, 0) == 500000


# =========================================================================
# RTCore fast_trace analyzer: _extract_axis_sample
# =========================================================================

def _fast_trace_record(
    *,
    t_ns: int,
    seq: int,
    axis_index: int,
    p: int,
    tp: int,
    sw: int,
    er: int,
    mt_low: int,
    mt_high: int,
    mt_ok: bool = True,
) -> dict:
    return {
        "t_ns": t_ns,
        "seq": seq,
        "ax": [
            {
                "i": axis_index,
                "p": p,
                "tp": tp,
                "sw": sw,
                "er": er,
                "mfr": 0,
                "af": [
                    {"k": "encoder_multi_turn_low", "v": mt_low, "ok": int(mt_ok)},
                    {"k": "encoder_multi_turn_high", "v": mt_high, "ok": int(mt_ok)},
                ],
            }
        ],
    }


def test_extract_axis_sample_j6_present():
    record = _fast_trace_record(
        t_ns=100, seq=1, axis_index=5, p=655360, tp=655361, sw=0x9637, er=0,
        mt_low=67239, mt_high=0,
    )
    s = cap._extract_axis_sample(record, axis_index=5)
    assert s is not None
    assert s["t_ns"] == 100
    assert s["p"] == 655360
    assert s["tp"] == 655361
    assert s["sw"] == 0x9637
    assert s["er"] == 0
    assert s["mt_i64"] == 67239


def test_extract_axis_sample_missing_axis_returns_none():
    record = _fast_trace_record(
        t_ns=0, seq=0, axis_index=5, p=0, tp=0, sw=0, er=0, mt_low=0, mt_high=0
    )
    assert cap._extract_axis_sample(record, axis_index=3) is None


def test_extract_axis_sample_ok_false_means_no_mt():
    record = _fast_trace_record(
        t_ns=0, seq=0, axis_index=5, p=100, tp=100, sw=0x9637, er=0,
        mt_low=67239, mt_high=0, mt_ok=False,
    )
    s = cap._extract_axis_sample(record, axis_index=5)
    assert s is not None
    assert s["mt_i64"] is None


# =========================================================================
# RTCore fast_trace analyzer: _analyze_rtcore_jsonl
# =========================================================================

def _write_fast_trace(path: Path, records: list[dict]) -> None:
    path.write_text("\n".join(json.dumps(r) for r in records) + "\n")


def test_analyze_rtcore_clean_short_motion(tmp_path: Path):
    """J6 moves +36,408 motor counts over 100 samples at 1 kHz (= +10 deg
    output at gear 10). Non-wrapping, monotonic, no fault."""
    records = []
    for i in range(101):
        p = 655360 + i * 364  # ~36,400 total increment over 100 steps
        records.append(_fast_trace_record(
            t_ns=1_000_000_000 + i * 1_000_000,
            seq=100 + i,
            axis_index=5,
            p=p, tp=p + 1, sw=0x9637, er=0,
            mt_low=p, mt_high=0,
        ))
    jsonl = tmp_path / "clean.jsonl"
    _write_fast_trace(jsonl, records)
    result = cap._analyze_rtcore_jsonl(jsonl, axis_index=5, rm_counts=RM)
    assert result.sample_count == 101
    assert result.cumulative_travel_wire_counts == 100 * 364
    assert result.net_displacement_wire_counts == 100 * 364
    assert result.ratio_cumulative_over_abs_net_wire == pytest.approx(1.0)
    assert result.wire_monotonic_within_budget is True
    assert result.fault_seen is False


def test_analyze_rtcore_whip_detected(tmp_path: Path):
    """Synthetic whip: motor moves forward +50,000 counts (50 samples), then
    backward -40,000 counts (40 samples). 90 samples, 89 transitions. Net
    wire delta = +9,000 (last_p - first_p). Cumulative = 89,000. The non-
    monotonic direction flip is what flags the whip here regardless of the
    exact ratio.
    """
    records = []
    t = 1_000_000_000
    p = 655360
    for i in range(50):
        p += 1000
        records.append(_fast_trace_record(
            t_ns=t, seq=i + 1, axis_index=5,
            p=p, tp=p, sw=0x9637, er=0, mt_low=p, mt_high=0,
        ))
        t += 1_000_000
    for i in range(40):
        p -= 1000
        records.append(_fast_trace_record(
            t_ns=t, seq=50 + i + 1, axis_index=5,
            p=p, tp=p, sw=0x9637, er=0, mt_low=p, mt_high=0,
        ))
        t += 1_000_000
    jsonl = tmp_path / "whip.jsonl"
    _write_fast_trace(jsonl, records)
    result = cap._analyze_rtcore_jsonl(jsonl, axis_index=5, rm_counts=RM,
                                       overshoot_budget_counts=500)
    # 89 transitions * 1000 counts each.
    assert result.cumulative_travel_wire_counts == 89_000
    # Net = last_p - first_p across the 90 samples (starting +1000 before the
    # first record was appended).
    assert result.net_displacement_wire_counts == 9_000
    # Ratio: 89/9 ~ 9.89, well above the 1.2 whip threshold.
    assert result.ratio_cumulative_over_abs_net_wire == pytest.approx(89 / 9)
    assert result.ratio_cumulative_over_abs_net_wire > 1.2
    # Direction flip well outside the 500-count budget -> not monotonic.
    assert result.wire_monotonic_within_budget is False


def test_analyze_rtcore_handles_seam_wrap(tmp_path: Path):
    """Motion crosses the 0/RM seam forward. Raw wire delta from (RM-100) to
    +100 is NOT a 1.31M-count jump: the unwrap delta is +200.
    """
    records = []
    p = RM - 100
    for i in range(10):
        p += 20  # +200 total across samples; last 5 wrap through 0
        p = p % RM
        records.append(_fast_trace_record(
            t_ns=1000 + i * 1_000_000, seq=i + 1, axis_index=5,
            p=p, tp=p, sw=0x9637, er=0, mt_low=p, mt_high=0,
        ))
    jsonl = tmp_path / "seam.jsonl"
    _write_fast_trace(jsonl, records)
    result = cap._analyze_rtcore_jsonl(jsonl, axis_index=5, rm_counts=RM)
    assert result.cumulative_travel_wire_counts == 9 * 20
    assert result.net_displacement_wire_counts == 9 * 20
    assert result.ratio_cumulative_over_abs_net_wire == pytest.approx(1.0)
    assert result.wire_monotonic_within_budget is True


def test_analyze_rtcore_drops_leading_zeros(tmp_path: Path):
    """RTCore writes a few samples with p=0/sw=0 before PDO latches.
    Analyzer should skip those and use the real stream for first_p etc."""
    records = []
    # 3 leading zero samples.
    for i in range(3):
        records.append(_fast_trace_record(
            t_ns=i * 1_000_000, seq=i, axis_index=5,
            p=0, tp=0, sw=0, er=0, mt_low=0, mt_high=0,
        ))
    # Real samples: p grows from 655360 by +100/sample.
    for i in range(10):
        records.append(_fast_trace_record(
            t_ns=(3 + i) * 1_000_000, seq=3 + i, axis_index=5,
            p=655360 + i * 100, tp=655360 + i * 100, sw=0x9637, er=0,
            mt_low=67239 + i * 100, mt_high=0,
        ))
    jsonl = tmp_path / "leading_zeros.jsonl"
    _write_fast_trace(jsonl, records)
    result = cap._analyze_rtcore_jsonl(jsonl, axis_index=5, rm_counts=RM)
    assert result.sample_count == 10
    assert result.first_p == 655360
    assert result.last_p == 655360 + 9 * 100


def test_analyze_rtcore_detects_fault_by_er(tmp_path: Path):
    """When 603F flips non-zero, analyzer reports fault with the sample idx."""
    records = []
    for i in range(5):
        records.append(_fast_trace_record(
            t_ns=i * 1_000_000, seq=i, axis_index=5,
            p=655360 + i * 10, tp=655360 + i * 10, sw=0x9637, er=0,
            mt_low=67239 + i, mt_high=0,
        ))
    records.append(_fast_trace_record(
        t_ns=5 * 1_000_000, seq=5, axis_index=5,
        p=655360 + 5 * 10, tp=655360 + 5 * 10, sw=0x9638, er=0xFF00,
        mt_low=67239 + 5, mt_high=0,
    ))
    jsonl = tmp_path / "fault.jsonl"
    _write_fast_trace(jsonl, records)
    result = cap._analyze_rtcore_jsonl(jsonl, axis_index=5, rm_counts=RM)
    assert result.fault_seen is True
    assert result.fault_sample_index == 5
    assert "0xFF00" in (result.fault_detail or "")


# =========================================================================
# save: _estimate_trace_stats + _save_rtcore_trace
# =========================================================================

def test_estimate_trace_stats_empty_file(tmp_path: Path):
    jsonl = tmp_path / "empty.jsonl"
    jsonl.write_text("")
    stats = cap._estimate_trace_stats(jsonl)
    assert stats["line_count"] == 0
    assert stats["first_t_ns"] is None
    assert stats["last_t_ns"] is None
    assert stats["effective_hz"] is None


def test_estimate_trace_stats_computes_hz_from_head_and_tail(tmp_path: Path):
    """Synthetic 1001-line JSONL with t_ns stepping 1 ms/line over 1 second.
    Head + tail parse gives effective_hz ~= 1000."""
    jsonl = tmp_path / "trace.jsonl"
    with jsonl.open("w", encoding="utf-8") as fh:
        for i in range(1001):
            fh.write(json.dumps({"t_ns": i * 1_000_000, "seq": i, "ax": [{"i": 5}]}))
            fh.write("\n")
    stats = cap._estimate_trace_stats(jsonl)
    assert stats["line_count"] == 1001
    assert stats["first_t_ns"] == 0
    assert stats["last_t_ns"] == 1_000_000_000
    assert stats["elapsed_s"] == pytest.approx(1.0)
    assert stats["effective_hz"] == pytest.approx(1001.0)


def test_estimate_trace_stats_ignores_trailing_blank_lines(tmp_path: Path):
    jsonl = tmp_path / "trace.jsonl"
    with jsonl.open("w", encoding="utf-8") as fh:
        fh.write(json.dumps({"t_ns": 0, "seq": 0, "ax": [{"i": 5}]}))
        fh.write("\n")
        fh.write(json.dumps({"t_ns": 500_000_000, "seq": 1, "ax": [{"i": 5}]}))
        fh.write("\n\n\n")  # blank trailing lines
    stats = cap._estimate_trace_stats(jsonl)
    assert stats["first_t_ns"] == 0
    assert stats["last_t_ns"] == 500_000_000


def test_save_rtcore_trace_if_exists_missing_source_is_noop(tmp_path: Path):
    """--if-exists mode on a missing source returns SaveResult(copied=False)."""
    result = cap._save_rtcore_trace(
        source_path=tmp_path / "does-not-exist.jsonl",
        dest_dir=tmp_path / "logs",
        label="autosave",
        if_exists=True,
        note=None,
    )
    assert result.copied is False
    assert result.skipped_reason is not None
    assert "does not exist" in result.skipped_reason
    # No destination dir should be spuriously created either.
    assert not (tmp_path / "logs").exists() or not any((tmp_path / "logs").iterdir())


def test_save_rtcore_trace_without_if_exists_raises_on_missing(tmp_path: Path):
    with pytest.raises(FileNotFoundError):
        cap._save_rtcore_trace(
            source_path=tmp_path / "does-not-exist.jsonl",
            dest_dir=tmp_path / "logs",
            label="manual",
            if_exists=False,
            note=None,
        )


# =========================================================================
# analyze-rtcore continued
# =========================================================================

def test_analyze_rtcore_detects_long_path_whip_even_when_monotonic(tmp_path: Path):
    """The 2026-04-19 UI-driven whip: motor moved ~360 deg the LONG way
    around in a single monotonic sweep. Cumulative travel ~= |net| ~= 360 deg,
    ratio ~= 1, wire_monotonic=True. Earlier verdict would call this CLEAN,
    but long_path_excess >= RM/2 must flag it as WHIP.
    """
    # Simulate the drive taking the long path: wire value increases by
    # ~RM counts across ~100 samples (e.g., 13,107 counts/cycle ramp at
    # 6000 motor RPM). No direction flip; cum/|net| ~= 1.
    step = 13107
    samples = []
    p = 1_310_700  # start near the seam
    for i in range(100):
        # Each step accumulates 13,107 counts of forward motion. Eventually
        # p wraps through 0, but the UNWRAPPED sum tracks the full +RM motion.
        p = (p + step) % RM
        samples.append(_fast_trace_record(
            t_ns=i * 1_000_000, seq=i, axis_index=5,
            p=p, tp=p, sw=0x9637, er=0, mt_low=i * step, mt_high=0,
        ))
    jsonl = tmp_path / "long_path_whip.jsonl"
    _write_jsonl(jsonl, samples)
    result = cap._analyze_rtcore_jsonl(jsonl, axis_index=5, rm_counts=RM)
    # 100 samples = 99 transitions; cumulative ~= 99 * 13,107 = 1,297,593.
    # That's about 99.0% of RM, so net is +1,297,593 and shortest-wrap is
    # -13,127 (= net - RM). long_path_excess is |net - shortest| = RM.
    assert abs(result.cumulative_travel_wire_counts - 99 * step) < 100
    assert abs(result.net_displacement_wire_counts - 99 * step) < 100
    assert result.wire_monotonic_within_budget is True
    # long_path_excess should be at least RM/2 (the gate).
    assert result.long_path_excess_counts >= RM // 2
    # Shortest-path wrap of net: should be in [-RM/2, +RM/2].
    assert -RM // 2 <= result.net_displacement_shortest_wrap_counts <= RM // 2


def test_analyze_rtcore_clean_short_cross_stays_clean(tmp_path: Path):
    """Sanity check: a clean 10 deg cross has long_path_excess = 0. The
    improved verdict MUST NOT turn the four prior clean runs into WHIP."""
    records = []
    # 10 deg = ~36,400 motor counts. Simulate a smooth 100-sample ramp.
    p_start = 18_200
    step = -364
    samples = []
    p = p_start
    for i in range(101):
        samples.append(_fast_trace_record(
            t_ns=i * 1_000_000, seq=i, axis_index=5,
            p=p % RM, tp=p % RM, sw=0x9637, er=0,
            mt_low=p, mt_high=(0 if p >= 0 else 0xFFFFFFFF),
        ))
        p += step
    jsonl = tmp_path / "clean_cross.jsonl"
    _write_jsonl(jsonl, samples)
    result = cap._analyze_rtcore_jsonl(jsonl, axis_index=5, rm_counts=RM)
    assert result.wire_monotonic_within_budget is True
    assert result.long_path_excess_counts == 0  # shortest path taken
    assert abs(result.net_displacement_wire_counts - 100 * step) < 100


def test_analyze_rtcore_mt_cross_check(tmp_path: Path):
    """U40.20/.22 refreshes every Nth sample (5 Hz on 1 kHz fast_trace =
    every ~200 samples). Analyzer counts distinct mt values separately."""
    records = []
    p = 655360
    mt = 67239
    for i in range(1000):
        p += 10
        # Multi-turn only updates every 200 samples to simulate 5 Hz poll.
        if i % 200 == 0:
            mt += 2000
        records.append(_fast_trace_record(
            t_ns=i * 1_000_000, seq=i, axis_index=5,
            p=p, tp=p, sw=0x9637, er=0, mt_low=mt, mt_high=0,
        ))
    jsonl = tmp_path / "mt_refresh.jsonl"
    _write_fast_trace(jsonl, records)
    result = cap._analyze_rtcore_jsonl(jsonl, axis_index=5, rm_counts=RM)
    assert result.mt_sample_count == 1000
    # Distinct mt values: one increment per 200 samples, 5 total distinct.
    assert result.mt_distinct_samples == 5
