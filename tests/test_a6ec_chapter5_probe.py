from __future__ import annotations

import importlib.util
from pathlib import Path


def _load_probe_module():
    repo_root = Path(__file__).resolve().parents[1]
    module_path = repo_root / "scripts" / "a6ec_chapter5_probe.py"
    spec = importlib.util.spec_from_file_location("a6ec_chapter5_probe", module_path)
    assert spec is not None
    assert spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


probe = _load_probe_module()


def test_classify_count_delta_boundaries():
    assert probe._classify_count_delta(None) is None

    assert probe._classify_count_delta(0) == "standard"
    assert probe._classify_count_delta(2) == "standard"
    assert probe._classify_count_delta(-2) == "standard"

    assert probe._classify_count_delta(3) == "medium"
    assert probe._classify_count_delta(6) == "medium"
    assert probe._classify_count_delta(-6) == "medium"

    assert probe._classify_count_delta(7) == "large"
    assert probe._classify_count_delta(10) == "large"
    assert probe._classify_count_delta(-10) == "large"

    assert probe._classify_count_delta(11) == "excessive"
    assert probe._classify_count_delta(100) == "excessive"
    assert probe._classify_count_delta(-100) == "excessive"

    assert probe._classify_count_delta(101) == "extreme"
    assert probe._classify_count_delta(-101) == "extreme"


def test_delta_summary_reports_signed_and_absolute_values():
    assert probe._delta_summary(None) == {"delta": None, "abs_delta": None, "category": None}
    assert probe._delta_summary(-4) == {"delta": -4.0, "abs_delta": 4.0, "category": "medium"}


def test_statusword_bits_report_vendor_hm_success_signature():
    success_bits = probe._statusword_bits(0x9650)
    assert success_bits["bit12_homing_attained"] is True
    assert success_bits["bit13_homing_error"] is False
    assert success_bits["bit15_reference_attained"] is True
    assert success_bits["bit15_homing_completed"] is True
    assert success_bits["vendor_hm_success_signature"] is True

    incomplete_bits = probe._statusword_bits(0x8650)
    assert incomplete_bits["bit15_reference_attained"] is True
    assert incomplete_bits["bit12_homing_attained"] is False
    assert incomplete_bits["vendor_hm_success_signature"] is False


def test_build_watch_axis_sample_merges_sdo_and_api_views():
    axis_snapshot = {
        "axis_index": 5,
        "reads": {
            "6041": {"ok": True, "value": 0x9650},
            "6064": {"ok": True, "value": 21},
            "607C": {"ok": True, "value": 0},
            "U40.16": {"ok": True, "value": -14},
            "U40.28": {"ok": True, "value": 131058},
            "6063": {"ok": True, "value": 21},
            "60FC": {"ok": True, "value": 131057},
        },
        "statusword_bits": {
            "bit12_homing_attained": True,
            "bit13_homing_error": False,
            "bit15_reference_attained": True,
            "vendor_hm_success_signature": True,
        },
        "derived": {
            "combined_u4020_22_signed_counts": 56113,
            "combined_u402a_2c_signed_counts": 131057,
        },
    }
    api_payload = {
        "read_source": "live_feedback",
        "canonical_joint_truth_available": True,
        "raw_canonical_joint_truth_available": True,
        "display_joint_truth_available": True,
        "arm_deg": [0, 0, 0, 0, 0, 0.0047],
        "arm_display_deg": [0, 0, 0, 0, 0, 0.0047],
        "arm_rad": [0, 0, 0, 0, 0, 0.00008],
        "arm_display_rad": [0, 0, 0, 0, 0, 0.00008],
        "axis_absolute_feedback": [
            {},
            {},
            {},
            {},
            {},
            {
                "truth_available": True,
                "truth_status": "available",
                "truth_reason": None,
                "display_source": "absolute_encoder_anchor",
                "absolute_source": "encoder_multi_turn_counts",
                "canonical_rad": 0.00008,
                "display_rad": 0.00008,
                "absolute_counts": 56113,
                "raw_counts": 131059,
                "reference_mode": "display",
            },
        ],
    }

    sample = probe._build_watch_axis_sample("J6", axis_snapshot, api_payload)

    assert sample["statusword_hex"] == "0x9650"
    assert sample["vendor_hm_success_signature"] is True
    assert sample["6064"] == 21
    assert sample["607C"] == 0
    assert sample["U40.16"] == -14
    assert sample["combined_u4020_22_signed_counts"] == 56113
    assert sample["combined_u402a_2c_signed_counts"] == 131057
    assert sample["api_arm_deg"] == 0.0047
    assert sample["api_display_source"] == "absolute_encoder_anchor"
    assert sample["api_truth_available"] is True


def test_format_watch_line_includes_core_live_fields():
    line = probe._format_watch_line(
        {
            "captured_at": "2026-04-15T09:30:00.000+00:00",
            "axes": {
                "J6": {
                    "6064": 21,
                    "607C": 0,
                    "U40.16": -14,
                    "combined_u4020_22_signed_counts": 56113,
                    "combined_u402a_2c_signed_counts": 131057,
                    "api_arm_deg": 0.0047,
                    "api_arm_display_deg": 0.0047,
                    "vendor_hm_success_signature": True,
                }
            },
        }
    )

    assert "J6" in line
    assert "6064=21" in line
    assert "607C=0" in line
    assert "U40.16=-14" in line
    assert "abs=56113" in line
    assert "rot=131057" in line
    assert "api_deg=0.0047" in line
    assert "hm_ok=True" in line


def test_render_markdown_includes_delta_categories():
    snapshot = {
        "experiment_id": "exp",
        "label": "stationary-1",
        "captured_at": "2026-04-14T23:08:53+00:00",
        "axes": {
            "J1": {
                "axis_index": 0,
                "statusword_bits": {"bit15_homing_completed": False},
                "derived": {
                    "ratio_6091_motor_over_shaft": 1.0,
                    "ratio_c10_rotation_mode": 1.0,
                    "matches_u4020_22_formula_within_one_count": False,
                    "delta_u4020_22_vs_reconstructed": 4.0,
                    "delta_u4020_22_vs_reconstructed_category": "medium",
                    "matches_6063_bridge_within_one_count": True,
                    "delta_6063_vs_6064_times_6091": 1.0,
                    "delta_6063_vs_6064_times_6091_category": "standard",
                    "matches_60fc_bridge_within_one_count": True,
                    "delta_60fc_vs_6062_times_6091": 8.0,
                    "delta_60fc_vs_6062_times_6091_category": "large",
                    "matches_u402a_2c_bridge_within_one_count": False,
                    "delta_u402a_2c_vs_u4028_times_c10_ratio": 120.0,
                    "delta_u402a_2c_vs_u4028_times_c10_ratio_category": "extreme",
                },
            }
        },
    }

    markdown = probe._render_markdown(snapshot)

    assert "category=`medium`" in markdown
    assert "category=`standard`" in markdown
    assert "category=`large`" in markdown
    assert "category=`extreme`" in markdown
