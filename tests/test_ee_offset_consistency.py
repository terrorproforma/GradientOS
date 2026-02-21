import numpy as np
from scipy.spatial.transform import Rotation as R

from gradient_os import ik_solver
from gradient_os.kinematics import runtime as kinematics_runtime


def _frontend_like_tool_offset_matrix(position_mm: dict[str, float], rotation_deg: dict[str, float]) -> np.ndarray:
    rx = np.deg2rad(float(rotation_deg.get("x", 0.0)))
    ry = np.deg2rad(float(rotation_deg.get("y", 0.0)))
    rz = np.deg2rad(float(rotation_deg.get("z", 0.0)))
    rot = (
        R.from_euler("z", rz).as_matrix()
        @ R.from_euler("y", ry).as_matrix()
        @ R.from_euler("x", rx).as_matrix()
    )
    out = np.eye(4, dtype=float)
    out[:3, :3] = rot
    out[:3, 3] = np.array(
        [
            float(position_mm.get("x", 0.0)) / 1000.0,
            float(position_mm.get("y", 0.0)) / 1000.0,
            float(position_mm.get("z", 0.0)) / 1000.0,
        ],
        dtype=float,
    )
    return out


def test_frontend_tool_offset_matrix_matches_backend_runtime_offset_matrix():
    tool_offset = {
        "position_mm": {"x": 12.5, "y": -37.0, "z": 347.773},
        "rotation_deg": {"x": 31.0, "y": -47.5, "z": 102.25},
    }
    backend_runtime_offset = kinematics_runtime._tool_offset_to_runtime_offset({"offset": tool_offset})  # type: ignore[attr-defined]
    backend_matrix = kinematics_runtime._matrix_from_offset(backend_runtime_offset)  # type: ignore[attr-defined]
    frontend_matrix = _frontend_like_tool_offset_matrix(
        tool_offset["position_mm"],
        tool_offset["rotation_deg"],
    )
    assert np.allclose(frontend_matrix, backend_matrix, atol=1e-12)


def test_fk_matrix_tool_offset_composition_matches_expected(monkeypatch):
    kinematics_runtime.reset_runtime_offsets(expected_revision=None, motion_state="IDLE")
    tool_payload = {
        "active_tool_id": "test-offset-tool",
        "display_name": "Test Offset Tool",
        "offset": {
            "position_mm": {"x": 0.0, "y": 37.5, "z": 347.773},
            "rotation_deg": {"x": 0.0, "y": 65.0, "z": 0.0},
        },
        "mesh": None,
    }
    kinematics_runtime.set_active_tool_definition(
        tool_payload,
        expected_revision=None,
        motion_state="IDLE",
        reset_runtime_trim=True,
    )

    wrist_fk = np.eye(4, dtype=float)
    wrist_fk[:3, :3] = R.from_euler("xyz", [15.0, -20.0, 30.0], degrees=True).as_matrix()
    wrist_fk[:3, 3] = np.array([0.42, -0.11, 0.28], dtype=float)
    monkeypatch.setattr(ik_solver, "_fk_matrix_impl", lambda _q: wrist_fk.copy())

    actual_fk = ik_solver.get_fk_matrix([0.0] * 6)
    assert actual_fk is not None

    expected_tool = _frontend_like_tool_offset_matrix(
        tool_payload["offset"]["position_mm"],
        tool_payload["offset"]["rotation_deg"],
    )
    expected_fk = wrist_fk.dot(expected_tool)
    assert np.allclose(actual_fk, expected_fk, atol=1e-12)
