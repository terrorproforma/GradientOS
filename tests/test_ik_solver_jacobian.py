import numpy as np
from scipy.spatial.transform import Rotation as R

from gradient_os import ik_solver
from gradient_os.kinematics import runtime as kinematics_runtime


def test_finite_difference_jacobian_uses_spatial_angular_frame(monkeypatch):
    def _fk(joints):
        q = np.asarray(joints, dtype=float).reshape(-1)
        pose = np.eye(4, dtype=float)
        c = float(np.cos(q[0]))
        s = float(np.sin(q[0]))
        pose[:3, :3] = np.array(
            [
                [c, -s, 0.0],
                [s, c, 0.0],
                [0.0, 0.0, 1.0],
            ],
            dtype=float,
        )
        return pose

    monkeypatch.setattr(ik_solver, "get_fk_matrix", _fk)

    J = ik_solver._finite_difference_jacobian_via_fk_runtime(np.zeros(6, dtype=float))

    np.testing.assert_allclose(J[:3, 0], [0.0, 0.0, 0.0], atol=1e-8)
    np.testing.assert_allclose(J[3:, 0], [0.0, 0.0, 1.0], atol=1e-5)


def test_compute_jacobian_falls_back_to_runtime_fd(monkeypatch):
    sentinel = np.arange(36, dtype=float).reshape(6, 6)
    monkeypatch.setattr(ik_solver, "_JACOBIAN_AVAILABLE", True)
    monkeypatch.setattr(ik_solver.kinematics_runtime, "runtime_offsets_are_identity", lambda: False)
    monkeypatch.setattr(ik_solver, "_finite_difference_jacobian_via_fk_runtime", lambda _q: sentinel)

    J = ik_solver.compute_jacobian(np.zeros(6, dtype=float))

    assert J is sentinel


def test_compute_jacobian_uses_analytical_under_identity_runtime(monkeypatch):
    sentinel = np.eye(6, dtype=float) * 2.0

    class _Robot:
        dof = 6

        def jacobian(self, _q):
            return sentinel

    monkeypatch.setattr(ik_solver, "_JACOBIAN_AVAILABLE", True)
    monkeypatch.setattr(ik_solver.kinematics_runtime, "runtime_offsets_are_identity", lambda: True)
    monkeypatch.setattr(ik_solver, "_analytical_jacobian_matches_active_fk", lambda: True)
    monkeypatch.setattr(ik_solver, "_ensure_jacobian_robot", lambda: _Robot())

    J = ik_solver.compute_jacobian(np.zeros(6, dtype=float))

    assert J is sentinel


def test_analytical_jacobian_mismatch_disables_analytical_path(monkeypatch):
    class _Robot:
        dof = 6

        def jacobian(self, _q):
            return np.zeros((6, 6), dtype=float)

    monkeypatch.setattr(ik_solver, "_JACOBIAN_AVAILABLE", True)
    monkeypatch.setattr(ik_solver, "_ROBOT_ID", "test-robot")
    monkeypatch.setattr(ik_solver, "_BACKEND_NAME", "ikfast")
    monkeypatch.setattr(ik_solver, "_ensure_jacobian_robot", lambda: _Robot())
    monkeypatch.setattr(ik_solver.kinematics_runtime, "runtime_offsets_are_identity", lambda: True)
    monkeypatch.setattr(ik_solver, "_finite_difference_jacobian_via_fk_runtime", lambda _q: np.eye(6, dtype=float))
    ik_solver._JACOBIAN_FK_COMPAT_CACHE.clear()

    assert ik_solver._analytical_jacobian_matches_active_fk() is False


def _twist_from_pose_delta(T0: np.ndarray, T1: np.ndarray, dt: float) -> np.ndarray:
    linear = (T1[:3, 3] - T0[:3, 3]) / dt
    dR = T1[:3, :3] @ T0[:3, :3].T
    angular = R.from_matrix(dR).as_rotvec() / dt
    return np.concatenate([linear, angular])


def _assert_jacobian_matches_runtime_fk(monkeypatch, patch_payload):
    kinematics_runtime.reset_runtime_offsets(expected_revision=None, motion_state="IDLE")
    try:
        kinematics_runtime.patch_runtime_offsets(
            patch_payload,
            expected_revision=None,
            motion_state="IDLE",
        )
        monkeypatch.setattr(ik_solver, "_JACOBIAN_AVAILABLE", True)
        q = np.array([0.21, -0.32, 0.18, 0.09, -0.14, 0.26], dtype=float)
        q_dot = np.array([0.04, -0.03, 0.02, 0.01, -0.015, 0.025], dtype=float)
        dt = 1e-4

        J = ik_solver.compute_jacobian(q)
        T0 = ik_solver.get_fk_matrix(q)
        T1 = ik_solver.get_fk_matrix(q + q_dot * dt)
        actual_twist = _twist_from_pose_delta(T0, T1, dt)

        np.testing.assert_allclose(J @ q_dot, actual_twist, atol=2e-3, rtol=2e-2)
    finally:
        kinematics_runtime.reset_runtime_offsets(expected_revision=None, motion_state="IDLE")


def test_compute_jacobian_matches_get_fk_under_non_identity_tool_offset(monkeypatch):
    _assert_jacobian_matches_runtime_fk(
        monkeypatch,
        {
            "tool": {
                "position_m": {"x": 0.0, "y": 0.0, "z": 0.10},
                "rotation_deg": {"x": 0.0, "y": 0.0, "z": 0.0},
            }
        },
    )


def test_compute_jacobian_matches_get_fk_under_non_identity_base_offset(monkeypatch):
    _assert_jacobian_matches_runtime_fk(
        monkeypatch,
        {
            "base": {
                "position_m": {"x": 0.01, "y": -0.02, "z": 0.03},
                "rotation_deg": {"x": 0.0, "y": 0.0, "z": 15.0},
            }
        },
    )


def test_jacobian_status_reports_unavailable_reason(monkeypatch):
    monkeypatch.setattr(ik_solver, "_JACOBIAN_AVAILABLE", False)
    monkeypatch.setattr(ik_solver, "_JACOBIAN_UNAVAILABLE_REASON", "missing binding")

    status = ik_solver.get_jacobian_status()

    assert status == {"available": False, "unavailable_reason": "missing binding"}
