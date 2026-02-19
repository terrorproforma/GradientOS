import numpy as np
import pytest

from gradient_os.arm_controller import command_api
from gradient_os.arm_controller import utils


def test_weld_orientation_fallback_warning_includes_tool_id(monkeypatch, tmp_path):
    monkeypatch.setattr(command_api, "RECORDED_TRAJ_DIR", str(tmp_path / "recorded"))
    monkeypatch.setattr(utils, "TRAJECTORY_CACHE_DIR", str(tmp_path / "cache"))
    monkeypatch.setattr(
        utils,
        "current_logical_joint_angles_rad",
        [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        raising=False,
    )
    monkeypatch.setattr(command_api.ik_solver, "get_fk", lambda _q: [0.0, 0.0, 0.0])
    monkeypatch.setattr(command_api.ik_solver, "get_fk_matrix", lambda _q: np.eye(4, dtype=float))
    monkeypatch.setattr(
        command_api.trajectory_execution,
        "_plan_linear_move",
        lambda *args, **kwargs: [[0.0] * 6, [0.01] * 6],
    )

    call_count = {"n": 0}

    def _fake_hifi(*args, **kwargs):
        call_count["n"] += 1
        if call_count["n"] == 1:
            return None
        return [[0.01] * 6, [0.02] * 6]

    monkeypatch.setattr(command_api.trajectory_execution, "_plan_high_fidelity_trajectory", _fake_hifi)
    monkeypatch.setattr(
        command_api.kinematics_runtime,
        "get_runtime_state_snapshot",
        lambda: {
            "active_tool": {
                "active_tool_id": "tig-torch-65deg",
            },
            "offsets": {
                "tool_effective": {
                    "position_m": {"x": 0.0, "y": 0.0, "z": 0.0},
                    "rotation_deg": {"x": 0.0, "y": 0.0, "z": 0.0},
                }
            },
        },
    )

    payload = command_api.plan_preview_trajectory_points(
        points=[[0.0, 0.0, 0.0], [0.03, 0.0, 0.0]],
        preview_name="__tool_semantics_test__",
        weld_metadata={
            "type": "fillet",
            "options": {
                "post_action": "none",
            },
        },
    )
    warnings = payload.get("planning_warnings", [])
    assert any("tool=tig-torch-65deg" in item for item in warnings)


def test_tangent_roll_changes_weld_orientations(monkeypatch, tmp_path):
    monkeypatch.setattr(command_api, "RECORDED_TRAJ_DIR", str(tmp_path / "recorded"))
    monkeypatch.setattr(utils, "TRAJECTORY_CACHE_DIR", str(tmp_path / "cache"))
    monkeypatch.setattr(
        utils,
        "current_logical_joint_angles_rad",
        [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        raising=False,
    )
    monkeypatch.setattr(command_api.ik_solver, "get_fk", lambda _q: [0.0, 0.0, 0.0])
    monkeypatch.setattr(command_api.ik_solver, "get_fk_matrix", lambda _q: np.eye(4, dtype=float))
    monkeypatch.setattr(
        command_api.trajectory_execution,
        "_plan_linear_move",
        lambda *args, **kwargs: [[0.0] * 6, [0.01] * 6],
    )
    monkeypatch.setattr(
        command_api.kinematics_runtime,
        "get_runtime_state_snapshot",
        lambda: {
            "active_tool": {"active_tool_id": "identity"},
            "offsets": {
                "tool_effective": {
                    "position_m": {"x": 0.0, "y": 0.0, "z": 0.0},
                    "rotation_deg": {"x": 0.0, "y": 0.0, "z": 0.0},
                }
            },
        },
    )

    def _run_for_roll(roll_deg: float):
        captured: dict[str, list[np.ndarray] | None] = {"orientations": None}

        def _fake_hifi(*args, **kwargs):
            captured["orientations"] = kwargs.get("orientations_list")
            return [[0.01] * 6, [0.02] * 6]

        monkeypatch.setattr(command_api.trajectory_execution, "_plan_high_fidelity_trajectory", _fake_hifi)
        payload = command_api.plan_preview_trajectory_points(
            points=[[0.0, 0.0, 0.0], [0.04, 0.0, 0.0], [0.08, 0.02, 0.0]],
            preview_name=f"__tool_roll_{roll_deg:.0f}__",
            weld_metadata={
                "type": "fillet",
                "options": {
                    "post_action": "none",
                    "interior_speed_m_s": 5.0,
                    "work_angle_deg": 35.0,
                    "travel_angle_deg": 10.0,
                    "tangent_roll_deg": roll_deg,
                },
            },
        )
        assert captured["orientations"] is not None
        assert len(captured["orientations"]) > 0
        return np.array(captured["orientations"][0], dtype=float), payload

    roll0_orientation, roll0_payload = _run_for_roll(0.0)
    roll45_orientation, roll45_payload = _run_for_roll(45.0)

    assert not np.allclose(roll0_orientation, roll45_orientation)
    assert roll0_payload["weld"]["options"]["tangent_roll_deg"] == pytest.approx(0.0)
    assert roll45_payload["weld"]["options"]["tangent_roll_deg"] == pytest.approx(45.0)


def test_tangent_roll_defaults_to_zero_when_missing(monkeypatch, tmp_path):
    monkeypatch.setattr(command_api, "RECORDED_TRAJ_DIR", str(tmp_path / "recorded"))
    monkeypatch.setattr(utils, "TRAJECTORY_CACHE_DIR", str(tmp_path / "cache"))
    monkeypatch.setattr(
        utils,
        "current_logical_joint_angles_rad",
        [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        raising=False,
    )
    monkeypatch.setattr(command_api.ik_solver, "get_fk", lambda _q: [0.0, 0.0, 0.0])
    monkeypatch.setattr(command_api.ik_solver, "get_fk_matrix", lambda _q: np.eye(4, dtype=float))
    monkeypatch.setattr(
        command_api.trajectory_execution,
        "_plan_linear_move",
        lambda *args, **kwargs: [[0.0] * 6, [0.01] * 6],
    )
    monkeypatch.setattr(
        command_api.trajectory_execution,
        "_plan_high_fidelity_trajectory",
        lambda *args, **kwargs: [[0.01] * 6, [0.02] * 6],
    )

    payload = command_api.plan_preview_trajectory_points(
        points=[[0.0, 0.0, 0.0], [0.04, 0.0, 0.0]],
        preview_name="__tool_roll_default__",
        weld_metadata={
            "type": "fillet",
            "options": {
                "post_action": "none",
            },
        },
    )
    assert payload["weld"]["options"]["tangent_roll_deg"] == pytest.approx(0.0)
