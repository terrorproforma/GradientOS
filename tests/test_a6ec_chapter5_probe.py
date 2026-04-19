from __future__ import annotations

import base64
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


def test_resolve_watch_interval_uses_standard_and_fast_proof_floors():
    assert probe._resolve_watch_interval(0.25, fast_proof=False) == 0.25
    assert probe._resolve_watch_interval(0.02, fast_proof=False) == probe.WATCH_INTERVAL_FLOOR_S
    assert probe._resolve_watch_interval(0.02, fast_proof=True) == 0.02
    assert probe._resolve_watch_interval(0.005, fast_proof=True) == probe.FAST_PROOF_INTERVAL_FLOOR_S


def test_select_watch_sdo_objects_fast_proof_keeps_phase1_core_fields():
    labels = [label for label, _index, _subindex, _type in probe._select_watch_sdo_objects(fast_proof=True)]

    assert set(labels) == set(probe.FAST_PROOF_SDO_LABELS)
    assert "603F" in labels
    assert "607A" in labels
    assert "U40.20" in labels
    assert "U40.1C" not in labels
    assert "C10.16" not in labels


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


def test_parse_motion_status_response_decodes_base64_payload():
    token = base64.urlsafe_b64encode(b'{"state":"idle","queue_depth":0}').decode("ascii")

    payload = probe._parse_motion_status_response(f"MOTION_STATUS,{token}")

    assert payload == {"state": "idle", "queue_depth": 0}


def test_parse_monitor_event_lines_extracts_json_payload():
    parsed = probe._parse_monitor_event_lines(
        [
            "event: telemetry",
            'data: {"display_joints":[0.1],"joints":[0.2]}',
            "",
        ]
    )

    assert parsed["event"] == "telemetry"
    assert parsed["data"] == '{"display_joints":[0.1],"joints":[0.2]}'
    assert parsed["json"] == {"display_joints": [0.1], "joints": [0.2]}


def test_build_watch_axis_sample_merges_sdo_and_api_views():
    axis_snapshot = {
        "axis_index": 5,
        "reads": {
            "203F": {"ok": True, "value": 0x0871},
            "603F": {"ok": True, "value": 0xFF00},
            "6041": {"ok": True, "value": 0x9650},
            "6062": {"ok": True, "value": 21},
            "6064": {"ok": True, "value": 21},
            "607A": {"ok": True, "value": 22},
            "607C": {"ok": True, "value": 0},
            "60B0": {"ok": True, "value": -17},
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
    controller_joint_state = {
        "read_source": "live_feedback",
        "canonical_joint_truth_available": True,
        "arm_deg": [0, 0, 0, 0, 0, 0.0047],
        "arm_display_deg": [0, 0, 0, 0, 0, 0.0047],
        "arm_rad": [0, 0, 0, 0, 0, 0.00008],
        "arm_display_rad": [0, 0, 0, 0, 0, 0.00008],
        "axis_counts": [0, 0, 0, 0, 0, 131059],
    }
    controller_motion_status = {
        "state": "executing",
        "trajectory_id": 17,
        "execution": {
            "controller_motion_state": "executing",
            "active_mode_name": "trajectory",
            "state_name": "executing",
            "active_traj_id": 17,
            "queue_depth": 12,
            "current_point_index": 3,
            "last_event_code": 288,
            "active_command_seq": 44,
            "motion_done": False,
            "stale_command": False,
        },
    }
    frontend_payload = {
        "read_source": "live_feedback",
        "canonical_joint_truth_available": True,
        "arm_deg": [0, 0, 0, 0, 0, 0.0047],
        "arm_display_deg": [0, 0, 0, 0, 0, 0.0047],
        "arm_rad": [0, 0, 0, 0, 0, 0.00008],
        "arm_display_rad": [0, 0, 0, 0, 0, 0.00008],
        "axis_counts": [0, 0, 0, 0, 0, 131059],
    }
    monitor_payload = {
        "joint_feedback_available": True,
        "motion_status": {"state": "idle"},
        "joints": [0, 0, 0, 0, 0, 0.00008],
        "display_joints": [0, 0, 0, 0, 0, 0.00008],
    }
    metrics_payload = {
        "axes": [
            {},
            {},
            {},
            {},
            {},
            {
                "statusword": 0x9650,
                "error_code": 0,
                "manufacturer_error_code": 0,
                "native_home_position_offset": 0,
                "absolute_feedback": {"encoder_multi_turn_low": {"valid": 1, "value": 56113}},
            },
        ]
    }

    sample = probe._build_watch_axis_sample(
        "J6",
        axis_snapshot,
        api_payload,
        controller_joint_state=controller_joint_state,
        controller_motion_status=controller_motion_status,
        frontend_payload=frontend_payload,
        monitor_payload=monitor_payload,
        metrics_payload=metrics_payload,
    )

    assert sample["statusword_hex"] == "0x9650"
    assert sample["vendor_hm_success_signature"] is True
    assert sample["203F"] == 0x0871
    assert sample["203F_hex"] == "0x00000871"
    assert sample["603F"] == 0xFF00
    assert sample["603F_hex"] == "0xFF00"
    assert sample["6062"] == 21
    assert sample["6064"] == 21
    assert sample["607A"] == 22
    assert sample["607C"] == 0
    assert sample["60B0"] == -17
    assert sample["U40.16"] == -14
    assert sample["combined_u4020_22_signed_counts"] == 56113
    assert sample["combined_u402a_2c_signed_counts"] == 131057
    assert sample["api_arm_deg"] == 0.0047
    assert sample["api_display_source"] == "absolute_encoder_anchor"
    assert sample["api_truth_available"] is True
    assert sample["controller_arm_deg"] == 0.0047
    assert sample["controller_axis_counts"] == 131059
    assert sample["controller_motion_status_state"] == "executing"
    assert sample["controller_motion_state_name"] == "executing"
    assert sample["controller_motion_active_traj_id"] == 17
    assert sample["controller_motion_last_event_code"] == 288
    assert sample["controller_motion_active_command_seq"] == 44
    assert sample["frontend_arm_display_deg"] == 0.0047
    assert sample["monitor_display_joints_rad"] == 0.00008
    assert sample["monitor_motion_state"] == "idle"
    assert sample["metrics_statusword"] == 0x9650
    assert sample["metrics_absolute_feedback"] == {"encoder_multi_turn_low": {"valid": 1, "value": 56113}}


def test_capture_watch_sample_fast_proof_uses_reduced_capture_profile(monkeypatch):
    seen: dict[str, object] = {}

    def fake_capture_controller_views(**kwargs):
        seen["controller_kwargs"] = kwargs
        return {
            "joint_state": {"ok": False, "skipped": True},
            "motion_status": {
                "ok": True,
                "json": {
                    "state": "executing",
                    "trajectory_id": 17,
                    "execution": {
                        "controller_motion_state": "executing",
                        "active_mode_name": "trajectory",
                        "state_name": "executing",
                        "active_traj_id": 17,
                        "queue_depth": 12,
                        "current_point_index": 3,
                        "last_event_code": 288,
                        "active_command_seq": 44,
                        "motion_done": False,
                        "stale_command": False,
                    },
                },
            },
        }

    def fake_capture_api_views(**kwargs):
        seen["api_kwargs"] = kwargs
        return {
            "joints": {"ok": False, "skipped": True},
            "joints_detailed": {"ok": False, "skipped": True},
            "motion_status": {"ok": False, "skipped": True},
            "monitor_event": {"ok": False, "skipped": True},
        }

    def fake_collect_axis_snapshot(axis_name, axis_index, *, sdo_objects=None):
        assert axis_name == "J6"
        assert axis_index == 5
        assert sdo_objects is not None
        seen["sdo_labels"] = [label for label, _index, _subindex, _type in sdo_objects]
        return {
            "axis_name": axis_name,
            "axis_index": axis_index,
            "reads": {
                "203F": {"ok": True, "value": 0},
                "603F": {"ok": True, "value": 0},
                "6041": {"ok": True, "value": 0x9650},
                "6062": {"ok": True, "value": 21},
                "6064": {"ok": True, "value": 21},
                "607A": {"ok": True, "value": 22},
                "607C": {"ok": True, "value": 655360},
                "60B0": {"ok": True, "value": 0},
                "U40.20": {"ok": True, "value": 56113},
                "U40.22": {"ok": True, "value": 0},
            },
            "statusword_bits": {
                "bit12_homing_attained": True,
                "bit13_homing_error": False,
                "bit15_reference_attained": True,
                "vendor_hm_success_signature": True,
            },
            "derived": {
                "combined_u4020_22_signed_counts": 56113,
            },
        }

    monkeypatch.setattr(probe, "_capture_controller_views", fake_capture_controller_views)
    monkeypatch.setattr(probe, "_capture_api_views", fake_capture_api_views)
    monkeypatch.setattr(probe, "_collect_axis_snapshot", fake_collect_axis_snapshot)
    monkeypatch.setattr(
        probe,
        "_load_rtcore_metrics_capture",
        lambda: {
            "ok": True,
            "json": {
                "axes": [{}, {}, {}, {}, {}, {"statusword": 0x9650, "error_code": 0, "manufacturer_error_code": 0}]
            },
        },
    )

    sample = probe._capture_watch_sample(
        axis_names=["J6"],
        api_url="http://127.0.0.1:4400",
        fast_proof=True,
    )

    assert seen["controller_kwargs"] == {
        "controller_host": None,
        "controller_port": None,
        "include_joint_state": False,
        "include_motion_status": True,
    }
    assert seen["api_kwargs"] == {
        "api_url": "http://127.0.0.1:4400",
        "monitor_timeout_s": probe.DEFAULT_MONITOR_TIMEOUT_S,
        "include_joints": False,
        "include_joints_detailed": False,
        "include_motion_status": False,
        "include_monitor_event": False,
    }
    assert set(seen["sdo_labels"]) == set(probe.FAST_PROOF_SDO_LABELS)
    axis = sample["axes"]["J6"]
    assert axis["controller_motion_state_name"] == "executing"
    assert axis["controller_motion_active_traj_id"] == 17
    assert axis.get("controller_arm_deg") is None
    assert axis.get("monitor_display_joints_rad") is None


def test_format_watch_line_includes_core_live_fields():
    line = probe._format_watch_line(
        {
            "captured_at": "2026-04-15T09:30:00.000+00:00",
            "axes": {
                "J6": {
                    "203F_hex": "0x00000871",
                    "603F_hex": "0xFF00",
                    "6062": 21,
                    "6064": 21,
                    "607A": 22,
                    "607C": 0,
                    "60B0": -17,
                    "U40.16": -14,
                    "combined_u4020_22_signed_counts": 56113,
                    "combined_u402a_2c_signed_counts": 131057,
                    "api_arm_deg": 0.0047,
                    "api_arm_display_deg": 0.0047,
                    "controller_motion_state_name": "executing",
                    "controller_motion_active_traj_id": 17,
                    "controller_motion_last_event_code": 288,
                    "controller_motion_active_command_seq": 44,
                    "vendor_hm_success_signature": True,
                }
            },
        }
    )

    assert "J6" in line
    assert "603F=0xFF00" in line
    assert "203F=0x00000871" in line
    assert "6062=21" in line
    assert "6064=21" in line
    assert "607A=22" in line
    assert "607C=0" in line
    assert "60B0=-17" in line
    assert "U40.16=-14" in line
    assert "abs=56113" in line
    assert "rot=131057" in line
    assert "api_deg=0.0047" in line
    assert "rt_state=executing" in line
    assert "rt_traj=17" in line
    assert "rt_evt=288" in line
    assert "rt_seq=44" in line
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


def test_capture_snapshot_records_controller_api_and_metrics_views(monkeypatch):
    monkeypatch.setattr(
        probe,
        "_collect_axis_snapshot",
        lambda axis_name, axis_index: {
            "axis_name": axis_name,
            "axis_index": axis_index,
            "reads": {},
            "derived": {},
        },
    )
    monkeypatch.setattr(
        probe,
        "_capture_controller_views",
        lambda **_kwargs: {
            "endpoint": {"host": "127.0.0.1", "port": 3000},
            "joint_state": {
                "ok": True,
                "json": {
                    "read_source": "live_feedback",
                    "canonical_joint_truth_available": True,
                    "arm_deg": [0, 0, 0, 0, 0, 6.0],
                    "arm_display_deg": [0, 0, 0, 0, 0, 6.5],
                    "arm_rad": [0, 0, 0, 0, 0, 0.1],
                    "arm_display_rad": [0, 0, 0, 0, 0, 0.11],
                    "axis_counts": [0, 0, 0, 0, 0, 12345],
                },
            },
            "motion_status": {
                "ok": True,
                "json": {
                    "state": "idle",
                    "trajectory_id": 7,
                    "execution": {
                        "controller_motion_state": "idle",
                        "active_mode_name": "idle",
                        "state_name": "idle",
                        "active_traj_id": 0,
                        "queue_depth": 0,
                        "current_point_index": None,
                        "last_event_code": 0,
                        "active_command_seq": 0,
                        "motion_done": True,
                        "stale_command": False,
                    },
                },
            },
        },
    )
    monkeypatch.setattr(
        probe,
        "_capture_api_views",
        lambda **_kwargs: {
            "joints": {
                "ok": True,
                "json": {
                    "read_source": "live_feedback",
                    "canonical_joint_truth_available": True,
                    "arm_deg": [0, 0, 0, 0, 0, 6.0],
                    "arm_display_deg": [0, 0, 0, 0, 0, 6.5],
                    "arm_rad": [0, 0, 0, 0, 0, 0.1],
                    "arm_display_rad": [0, 0, 0, 0, 0, 0.11],
                },
            },
            "joints_detailed": {
                "ok": True,
                "json": {
                    "canonical_joint_truth_available": True,
                    "canonical_joint_truth_unavailable_joints": [],
                    "axis_absolute_feedback": [{}, {}, {}, {}, {}, {"truth_available": True}],
                },
            },
            "motion_status": {"ok": True, "json": {"state": "idle"}},
            "monitor_event": {
                "ok": True,
                "json": {
                    "joint_feedback_available": True,
                    "display_joints": [0, 0, 0, 0, 0, 0.11],
                    "joints": [0, 0, 0, 0, 0, 0.1],
                    "motion_status": {"state": "idle"},
                },
            },
        },
    )
    monkeypatch.setattr(
        probe,
        "_load_rtcore_metrics_capture",
        lambda: {"ok": True, "json": {"axes": [{}, {}, {}, {}, {}, {"statusword": 0x9650}]}},
    )

    snapshot = probe._capture_snapshot(
        label="manual-j6-rotate",
        experiment_id="exp",
        axis_names=["J6"],
        api_url="http://127.0.0.1:4400",
    )

    assert snapshot["controller"]["joint_state"]["json"]["axis_counts"][5] == 12345
    assert snapshot["api"]["joints"]["json"]["arm_display_deg"][5] == 6.5
    assert snapshot["rtcore_metrics"]["json"]["axes"][5]["statusword"] == 0x9650
    assert snapshot["controller_joint_state_selected"]["selected_axes"]["J6"]["axis_counts"] == 12345
    assert snapshot["api_joints_selected"]["selected_axes"]["J6"]["arm_display_deg"] == 6.5
    assert snapshot["monitor_selected"]["selected_axes"]["J6"]["display_joints_rad"] == 0.11
