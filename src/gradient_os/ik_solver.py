"""ik_solver.py – High-level kinematics API

This module exposes **solver-agnostic** helper functions:
    solve_ik(...)        – inverse kinematics (position + orientation)
    get_fk_matrix(...)   – full 4×4 pose of the tool tip
    get_fk(...)          – convenience: only tool-tip XYZ

Internally we can plug in several different solver back-ends. Selection is
done **at import time** via the environment variable ``MINI_ARM_SOLVER``.

    "ikfast"   – the original IKFast C++ wrapper (default)
    "numeric"  – the QuIK numeric solver (via ``numeric_wrapper.py``)
    "trac"     – placeholder for future TRAC-IK integration

The goal: callers’ code never changes – you can switch the backend via env-var
and update robot assets at runtime via ``set_robot_id()``.
"""

from __future__ import annotations

import os
import numpy as np
from scipy.spatial.transform import Rotation as R
from .arm_controller import utils  # for diagnostics folder/session info
from . import robot_assets
from .kinematics import runtime as kinematics_runtime

# -----------------------------------------------------------
# END-EFFECTOR OFFSET (tool-tip with respect to wrist frame)
# -----------------------------------------------------------
# This vector is expressed **in the world frame when the robot is at its
# zero-angle pose**.  In that pose the numeric (QuIK) solver’s positive Z-axis
# aligns with +X_world, so an offset of +x means +z in the EE frame.
# Keep everything in metres.
END_EFFECTOR_OFFSET = np.array([0.180, 0.0, 0.0], dtype=float)

# NOTE: the old constant above has been replaced – see top-of-file comment.

# Placeholder; real solver object (IKFastSolver instance or numeric IKSolver)
# is assigned inside the backend-specific initialisers so that legacy code
# that still refers to ``IK_SOLVER`` continues to work.
IK_SOLVER = None  # type: ignore
_KINEMATICS_PROFILE = None  # type: ignore

def _find_closest_solution(solutions, current_joint_angles):
    """
    Finds the IK solution that is closest to the current joint configuration.
    This helps ensure smooth transitions between movements.

    Args:
        solutions (np.array): An array of joint angle solutions from the solver.
        current_joint_angles (np.array): The current joint angles of the robot.

    Returns:
        np.array: The solution that requires the smallest change in joint angles.
    """
    if solutions is None or solutions.size == 0:
        return None

    # If only one solution is returned (already the closest), just return it
    if solutions.ndim == 1:
        return solutions

    # Before computing distances make every candidate continuous w.r.t current
    continuous_solutions = np.array([
        _wrap_to_prev(current_joint_angles, sol) for sol in solutions
    ])

    # Euclidean distance in joint space (all differences already shortest)
    distances = np.linalg.norm(continuous_solutions - current_joint_angles, axis=1)
    best_solution_idx = np.argmin(distances)
            
    # Return the wrapped version so the caller already gets a continuous vector
    return continuous_solutions[best_solution_idx]


def _coerce_rotation_matrix(target_orientation_matrix):
    """Normalize orientation input to a 3x3 rotation matrix."""
    if target_orientation_matrix is None:
        return np.identity(3, dtype=float)
    return np.asarray(target_orientation_matrix, dtype=float).reshape(3, 3)


def _prepare_backend_target_pose(target_position, target_orientation_matrix, backend_name: str):
    """
    Convert a tool-frame target pose into the backend-expected target pose.

    Backend semantics:
      - IKFast expects wrist-frame target position.
      - Numeric (QuIK) expects tool-frame target position directly.
    """
    target_rotation = _coerce_rotation_matrix(target_orientation_matrix)
    target_pos = np.asarray(target_position, dtype=float).reshape(3)
    target_pos, target_rotation = kinematics_runtime.compensate_target_pose_for_runtime(
        target_pos, target_rotation
    )

    if backend_name == "ikfast":
        rotated_offset = target_rotation.dot(END_EFFECTOR_OFFSET)
        target_pos = target_pos - rotated_offset
    elif backend_name == "numeric":
        # QuIK path uses tool-frame pose directly.
        pass
    else:
        raise ValueError(f"Unsupported backend '{backend_name}' in pose adapter.")

    return target_pos, target_rotation


def _normalize_joint_path_output(joint_solutions):
    """Return batch IK results as a plain list-of-lists for downstream callers."""
    if joint_solutions is None:
        return None

    joint_solutions_np = np.asarray(joint_solutions, dtype=np.float64)
    if joint_solutions_np.ndim == 1:
        return [joint_solutions_np.tolist()]
    if joint_solutions_np.ndim != 2:
        raise ValueError(
            "Batch IK solver returned an unexpected shape: "
            f"{joint_solutions_np.shape}"
        )
    return joint_solutions_np.tolist()

# -----------------------------------------------------------
# Backend selection helper – import the chosen solver only once
# -----------------------------------------------------------

_BACKEND_NAME: str = os.getenv("MINI_ARM_SOLVER", "ikfast").lower()  # set ikfast as default
# _BACKEND_NAME: str = os.getenv("MINI_ARM_SOLVER", "numeric").lower()   # set numeric as default
_ROBOT_ID: str = robot_assets.get_active_robot_id()

# These module-level globals will be filled by the selected backend loader.
_solve_ik_impl = None  # type: ignore
_fk_matrix_impl = None  # type: ignore
NUM_JOINTS = 6  # default until backend sets the real value
_JACOBIAN_ROBOT_CACHE: dict[str, object] = {}
_JACOBIAN_FK_COMPAT_CACHE: dict[tuple[str, str, int | str], bool] = {}
_JACOBIAN_AVAILABLE = False
_JACOBIAN_UNAVAILABLE_REASON = "jacobian-not-initialized"


def _normalize_backend_name(raw: str) -> str:
    backend = str(raw).strip().lower()
    if backend not in {"ikfast", "numeric", "trac"}:
        raise ValueError(f"Unknown MINI_ARM_SOLVER backend '{backend}'")
    return backend


def _ensure_jacobian_robot():
    """Lazy-load the pyquik Robot used by held-jog Jacobian math."""
    cached = _JACOBIAN_ROBOT_CACHE.get(_ROBOT_ID)
    if cached is not None:
        return cached
    from numeric_solver.numeric_wrapper import _ensure_loaded as _ensure_numeric_loaded

    robot, _ = _ensure_numeric_loaded(_ROBOT_ID)
    _JACOBIAN_ROBOT_CACHE[_ROBOT_ID] = robot
    return robot


def _refresh_jacobian_availability() -> None:
    """Warm the pyquik Jacobian binding and record a clear fallback reason."""
    global _JACOBIAN_AVAILABLE, _JACOBIAN_UNAVAILABLE_REASON
    try:
        robot = _ensure_jacobian_robot()
        jacobian_fn = getattr(robot, "jacobian", None)
        if not callable(jacobian_fn):
            raise RuntimeError("pyquik Robot does not expose jacobian()")
        dof = int(getattr(robot, "dof", NUM_JOINTS) or NUM_JOINTS)
        q = np.zeros(dof, dtype=float)
        J = np.asarray(jacobian_fn(q), dtype=float)
        if J.ndim != 2 or J.shape[0] != 6 or J.shape[1] != dof:
            raise RuntimeError(f"pyquik Robot.jacobian returned unexpected shape {J.shape}")
    except Exception as exc:
        _JACOBIAN_AVAILABLE = False
        _JACOBIAN_UNAVAILABLE_REASON = str(exc)[:240] or exc.__class__.__name__
        print(f"[IK Solver] WARNING: held-jog Jacobian unavailable: {_JACOBIAN_UNAVAILABLE_REASON}")
        return
    _JACOBIAN_AVAILABLE = True
    _JACOBIAN_UNAVAILABLE_REASON = ""


def is_jacobian_available() -> bool:
    return bool(_JACOBIAN_AVAILABLE)


def get_jacobian_status() -> dict[str, object]:
    return {
        "available": bool(_JACOBIAN_AVAILABLE),
        "unavailable_reason": _JACOBIAN_UNAVAILABLE_REASON,
    }


def _init_ikfast_backend():
    """Initialise IKFast C++ back-end (original behaviour)."""
    global _solve_ik_impl, _fk_matrix_impl, NUM_JOINTS

    from ikfast_solver.ikfast_wrapper import IKFastSolver  # local import: heavy .so

    try:
        profile = robot_assets.load_kinematics_profile(_ROBOT_ID, backend_name="ikfast")
        globals()["_KINEMATICS_PROFILE"] = profile
        solver = IKFastSolver()
        NUM_JOINTS = solver.num_joints
        globals()["IK_SOLVER"] = solver  # expose for legacy helpers
        print(f"[IK Solver] IKFast back-end initialised for {NUM_JOINTS} joints.")
    except Exception as e:
        print("--- [IK Solver] FATAL ERROR – IKFast backend ---")
        print(f"Failed to load IKFast: {e}")
        solver = None
        globals()["IK_SOLVER"] = None

    def _ikfast_solve_ik(target_position, target_orientation_matrix, initial_joint_angles):
        if solver is None:
            return None

        wrist_position, target_rotation = _prepare_backend_target_pose(
            target_position,
            target_orientation_matrix,
            "ikfast",
        )

        pose_rot_flat = target_rotation.flatten()
        sol = solver.solve_ik(wrist_position, pose_rot_flat, initial_joint_angles)

        # If multiple IK solutions are available, pick the one closest to the
        # current joint configuration to preserve continuity (critical for jog).
        if sol is None:
            return None
        if initial_joint_angles is not None:
            try:
                cur = np.asarray(initial_joint_angles, dtype=float)
                sol_arr = np.asarray(sol)
                best = _find_closest_solution(sol_arr, cur)
                return best
            except Exception:
                # Fallback to raw solver output if anything goes wrong
                return sol
        return sol

    def _ikfast_fk_matrix(joint_angles):
        if solver is None:
            return None

        wrist_t, wrist_r = solver.compute_fk(joint_angles)
        wrist_matrix = np.eye(4)
        wrist_matrix[:3, :3] = wrist_r.reshape(3, 3)
        wrist_matrix[:3, 3] = wrist_t

        offset_matrix = np.eye(4)
        offset_matrix[:3, 3] = END_EFFECTOR_OFFSET
        return wrist_matrix.dot(offset_matrix)

    _solve_ik_impl = _ikfast_solve_ik
    _fk_matrix_impl = _ikfast_fk_matrix
    return solver is not None


def _init_numeric_backend():
    """Initialise QuIK numeric back-end (python/pybind)."""
    global _solve_ik_impl, _fk_matrix_impl, NUM_JOINTS

    from numeric_solver.numeric_wrapper import (
        numeric_fk,
        numeric_ik,
        init_numeric_solver,
        set_robot_id as set_numeric_robot_id,
    )

    try:
        profile = robot_assets.load_kinematics_profile(_ROBOT_ID, backend_name="numeric")
        globals()["_KINEMATICS_PROFILE"] = profile
        set_numeric_robot_id(_ROBOT_ID)
        kin, solver = init_numeric_solver(_ROBOT_ID)
        NUM_JOINTS = kin.num_joints if hasattr(kin, "num_joints") else 6
        globals()["IK_SOLVER"] = solver  # expose numeric IKSolver for future use
        dh_csv_path = robot_assets.get_dh_csv_path(_ROBOT_ID)
        print(
            "[IK Solver] Numeric (QuIK) back-end initialised for "
            f"{NUM_JOINTS} joints (robot_id={_ROBOT_ID}, dh={dh_csv_path})."
        )
    except Exception as e:
        print("--- [IK Solver] FATAL ERROR – Numeric backend ---")
        print(f"Failed to load numeric solver: {e}")
        kin = solver = None
        globals()["IK_SOLVER"] = None

    def _numeric_solve_ik(target_position, target_orientation_matrix, initial_joint_angles):
        if solver is None:
            return None

        numeric_position, target_rotation = _prepare_backend_target_pose(
            target_position,
            target_orientation_matrix,
            "numeric",
        )
        quat = R.from_matrix(target_rotation).as_quat()
        sol, _, _, _ = numeric_ik(quat, np.asarray(numeric_position, dtype=float), initial_joint_angles)

        if sol is None:
            return None
        if initial_joint_angles is not None:
            try:
                cur = np.asarray(initial_joint_angles, dtype=float)
                sol_arr = np.asarray(sol)
                best = _find_closest_solution(sol_arr, cur)
                return best
            except Exception:
                return sol
        return sol

    def _numeric_fk_matrix(joint_angles):
        if kin is None:
            return None
        return numeric_fk(joint_angles)

    _solve_ik_impl = _numeric_solve_ik
    _fk_matrix_impl = _numeric_fk_matrix
    return kin is not None and solver is not None


def _init_backend():
    global _BACKEND_NAME
    requested_backend = _BACKEND_NAME
    if requested_backend == "ikfast":
        if _init_ikfast_backend():
            return
    elif requested_backend == "numeric":
        if _init_numeric_backend():
            return
    elif _BACKEND_NAME == "trac":
        raise NotImplementedError("TRAC-IK backend not yet integrated.")
    else:
        raise ValueError(f"Unknown MINI_ARM_SOLVER backend '{_BACKEND_NAME}'")

    fallback_order = [name for name in ("ikfast", "numeric") if name != requested_backend]
    for fallback_backend in fallback_order:
        print(
            f"[IK Solver] Falling back from '{requested_backend}' to '{fallback_backend}'."
        )
        ok = _init_ikfast_backend() if fallback_backend == "ikfast" else _init_numeric_backend()
        if ok:
            _BACKEND_NAME = fallback_backend
            os.environ["MINI_ARM_SOLVER"] = _BACKEND_NAME
            return
    print(
        f"[IK Solver] WARNING: No usable kinematics backend is available "
        f"(requested={requested_backend})."
    )


def configure(*, robot_id: str | None = None, backend_name: str | None = None) -> dict[str, str]:
    """
    Explicitly configure active robot + backend and reinitialize solver internals.

    This function is the runtime-safe path used by controller startup. It keeps
    environment variables synchronized for compatibility with legacy call paths.
    """
    global _ROBOT_ID, _BACKEND_NAME

    next_robot_id = _ROBOT_ID if robot_id is None else str(robot_id).strip()
    if not next_robot_id:
        raise ValueError("robot_id cannot be empty.")
    robot_assets.get_robot_manifest(next_robot_id)

    next_backend = _BACKEND_NAME if backend_name is None else _normalize_backend_name(backend_name)
    _ROBOT_ID = next_robot_id
    _BACKEND_NAME = next_backend
    os.environ["GRADIENT_ROBOT_ID"] = _ROBOT_ID
    os.environ["MINI_ARM_SOLVER"] = _BACKEND_NAME
    _init_backend()
    _refresh_jacobian_availability()
    return {
        "robot_id": _ROBOT_ID,
        "backend_name": _BACKEND_NAME,
    }


def set_robot_id(robot_id: str) -> None:
    """Backward-compatible helper that preserves current backend selection."""
    configure(robot_id=robot_id, backend_name=_BACKEND_NAME)


def get_backend_name() -> str:
    return _BACKEND_NAME


def get_robot_id() -> str:
    return _ROBOT_ID


# Initialise the chosen backend immediately so NUM_JOINTS is correct.
configure(robot_id=_ROBOT_ID, backend_name=_BACKEND_NAME)

# -----------------------------------------------------------
# Public, solver-independent API
# -----------------------------------------------------------


def solve_ik(*, target_position, target_orientation_matrix=None, initial_joint_angles=None):
    """Solver-agnostic wrapper – dispatches to selected backend and returns joint angles or None."""
    return _solve_ik_impl(target_position, target_orientation_matrix, initial_joint_angles)


def get_fk_matrix(active_joint_angles):
    """Return 4×4 tool-tip pose (world frame) for *active_joint_angles* or None on failure."""
    fk_matrix = _fk_matrix_impl(active_joint_angles)
    if fk_matrix is None:
        return None
    return kinematics_runtime.apply_runtime_to_fk_matrix(fk_matrix)


def get_fk(active_joint_angles):
    """Convenience: return just XYZ position from ``get_fk_matrix``."""
    fk_matrix = get_fk_matrix(active_joint_angles)
    if fk_matrix is not None:
        return fk_matrix[:3, 3]
    return None


def _finite_difference_jacobian_via_fk_runtime(joint_angles, eps: float = 1e-5) -> np.ndarray:
    """Return spatial/world-frame finite-difference Jacobian via get_fk_matrix()."""
    q = np.asarray(joint_angles, dtype=float).reshape(-1)
    joint_count = q.size
    J = np.zeros((6, joint_count), dtype=float)
    for idx in range(joint_count):
        q_plus = q.copy()
        q_minus = q.copy()
        q_plus[idx] += eps
        q_minus[idx] -= eps
        T_plus = get_fk_matrix(q_plus)
        T_minus = get_fk_matrix(q_minus)
        if T_plus is None or T_minus is None:
            raise RuntimeError("FK failed while computing finite-difference Jacobian.")
        T_plus = np.asarray(T_plus, dtype=float).reshape(4, 4)
        T_minus = np.asarray(T_minus, dtype=float).reshape(4, 4)
        J[:3, idx] = (T_plus[:3, 3] - T_minus[:3, 3]) / (2.0 * eps)

        # Spatial/world-frame angular velocity. Cartesian jog applies angular
        # deltas as R_delta @ R_commanded, so body-frame log would disagree.
        dR = T_plus[:3, :3] @ T_minus[:3, :3].T
        cos_theta = np.clip((np.trace(dR) - 1.0) / 2.0, -1.0, 1.0)
        theta = float(np.arccos(cos_theta))
        if abs(theta) < 1e-9:
            J[3:, idx] = 0.0
        else:
            scale = theta / (2.0 * np.sin(theta))
            log_R = scale * (dR - dR.T)
            J[3:, idx] = np.array([log_R[2, 1], log_R[0, 2], log_R[1, 0]]) / (2.0 * eps)
    return J


def _analytical_jacobian_matches_active_fk() -> bool:
    revision_key: int | str
    if kinematics_runtime.runtime_offsets_are_identity():
        revision_key = "identity-runtime"
    else:
        revision_key = kinematics_runtime.get_revision()
    cache_key = (_ROBOT_ID, _BACKEND_NAME, revision_key)
    cached = _JACOBIAN_FK_COMPAT_CACHE.get(cache_key)
    if cached is not None:
        return bool(cached)
    if not _JACOBIAN_AVAILABLE:
        _JACOBIAN_FK_COMPAT_CACHE[cache_key] = False
        return False
    try:
        robot = _ensure_jacobian_robot()
        dof = int(getattr(robot, "dof", NUM_JOINTS) or NUM_JOINTS)
        reference_vectors = [
            np.linspace(0.11, 0.11 * dof, dof, dtype=float),
            np.linspace(-0.17, 0.19, dof, dtype=float),
        ]
        for q in reference_vectors:
            J_analytic = np.asarray(robot.jacobian(q), dtype=float)
            J_fd = _finite_difference_jacobian_via_fk_runtime(q)
            if not np.allclose(J_analytic, J_fd, atol=2e-3, rtol=2e-2):
                _JACOBIAN_FK_COMPAT_CACHE[cache_key] = False
                return False
    except Exception:
        _JACOBIAN_FK_COMPAT_CACHE[cache_key] = False
        return False
    _JACOBIAN_FK_COMPAT_CACHE[cache_key] = True
    return True


def compute_jacobian(joint_angles) -> np.ndarray:
    """Return a 6xN spatial Jacobian aligned with get_fk_matrix() semantics."""
    if not _JACOBIAN_AVAILABLE:
        raise RuntimeError(_JACOBIAN_UNAVAILABLE_REASON or "held-jog Jacobian unavailable")
    if kinematics_runtime.runtime_offsets_are_identity() and _analytical_jacobian_matches_active_fk():
        robot = _ensure_jacobian_robot()
        q = np.asarray(joint_angles, dtype=float).reshape(-1)
        return np.asarray(robot.jacobian(q), dtype=float)
    return _finite_difference_jacobian_via_fk_runtime(joint_angles)

# ----------------------------------------------------------------------------------
# Path IK (Sequential) helper – now supports a `verbose` flag to silence prints
# ----------------------------------------------------------------------------------

def solve_ik_path_sequential(path_points, initial_joint_angles=None, target_orientations=None, *, verbose=True):
    """
    Solves IK for a sequence of points, using the previous solution as the
    initial guess for the next. This is ideal for smooth, connected paths.

    Args:
        path_points (list): A sequence of [x, y, z] target positions.
        initial_joint_angles (list, optional): Starting joint angles for the first point.
        target_orientations (list, optional): A list of 3x3 rotation matrices.
                                             Can be flat (9,) or (3,3).
        verbose (bool, optional): Whether to print debug information.

    Returns:
        list or None: A list of joint angle solutions for the path.
    """
    if _solve_ik_impl is None:
        return None

    if target_orientations and len(target_orientations) != len(path_points):
        raise ValueError("The number of target orientations must match the number of path points.")

    current_joint_angles = np.array(initial_joint_angles if initial_joint_angles is not None else [0.0] * NUM_JOINTS, dtype=np.float64)
    
    all_solutions = []
    for i, position in enumerate(path_points):
        if verbose:
            print(f"\n--- Path Point {i+1}/{len(path_points)} ---")
        
        orientation = np.array(target_orientations[i]).flatten() if target_orientations else None
        
        solution = solve_ik(
            target_position=position,
            target_orientation_matrix=orientation,
            initial_joint_angles=current_joint_angles
        )

        if solution is None:
            if verbose:
                print(f"IK solution not found for point {i} at position {position}. Aborting path calculation.")
            return None
        
        all_solutions.append(solution)
        current_joint_angles[:] = solution

    return all_solutions

def solve_ik_path_batch(path_points, initial_joint_angles=None, target_orientations=None):
    """
    Solves IK for a sequence of points using a single, efficient C++ batch call.

    Args:
        path_points (list): A sequence of [x, y, z] target positions.
        initial_joint_angles (list, optional): Starting joint angles for the first point.
        target_orientations (list, optional): A list of 3x3 rotation matrices.

    Returns:
        list or None: A list of joint angle solutions for the path.
    """
    if _solve_ik_impl is None:
        return None

    num_poses = len(path_points)
    if target_orientations and len(target_orientations) != num_poses:
        raise ValueError("The number of target orientations must match the number of path points.")

    start_angles_np = np.array(initial_joint_angles if initial_joint_angles is not None else [0.0] * NUM_JOINTS, dtype=np.float64)
    
    # --- Prepare the batch data for the C++ function ---
    poses_batch = np.zeros((num_poses, 12), dtype=np.float64)
    
    default_rotation = np.identity(3, dtype=float)
    
    for i in range(num_poses):
        position = path_points[i]
        
        if target_orientations:
            target_rotation = np.array(target_orientations[i]).reshape(3, 3)
        else:
            target_rotation = default_rotation

        backend_position, backend_rotation = _prepare_backend_target_pose(
            position,
            target_rotation,
            _BACKEND_NAME,
        )

        poses_batch[i, :3] = backend_position
        poses_batch[i, 3:] = backend_rotation.flatten()

    # Call the new batch solver in the wrapper
    joint_solutions = _normalize_joint_path_output(
        IK_SOLVER.solve_ik_path(poses_batch, start_angles_np)
    )

    # --- Optional Diagnostics Logging ---
    if joint_solutions is not None and os.environ.get("MINI_ARM_IK_LOG", "0") == "1":
        try:
            import csv, datetime
            from pathlib import Path

            session_id = utils.trajectory_state.get('diagnostics_session_id')
            folder_type = utils.trajectory_state.get('diagnostics_folder_type', 'ik_plans') # fallback

            if session_id:
                out_dir = Path(f"diagnostics/{folder_type}/{session_id}")
                csv_file = out_dir / "ik_plan.csv"
            else:
                session_id = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
                out_dir = Path(f"diagnostics/{folder_type}")
                csv_file = out_dir / f"ik_plan_{session_id}.csv"

            out_dir.mkdir(parents=True, exist_ok=True)
            with open(csv_file, "w", newline="") as fp:
                writer = csv.writer(fp)
                header = ["idx", "target_x", "target_y", "target_z", *[f"J{i+1}_rad" for i in range(len(joint_solutions[0]))]]
                writer.writerow(header)
                for idx, (pt, q) in enumerate(zip(path_points, joint_solutions)):
                    writer.writerow([idx, *pt, *q])
            print(f"[IK Solver] Diagnostics CSV saved -> {csv_file}")
        except Exception as e:
            print(f"[IK Solver] WARNING: Failed to write diagnostics CSV: {e}")

    return joint_solutions

# Note: A parallel implementation is not provided as the C++ IKFast solver
# is extremely fast, making the overhead of process creation often slower
# than sequential execution.

# ---------------------------------------------------------------
# Angle-wrapping helpers (for revolute/continuous joints)
# ---------------------------------------------------------------

_TWO_PI = 2.0 * np.pi


def _shortest_angular_distance(a, b):
    """Return the signed smallest angular difference a→b (both rad)."""
    diff = (b - a + np.pi) % _TWO_PI - np.pi
    return diff


def _wrap_to_prev(prev, angles):
    """Shift each element of *angles* by ±2π so it is <π from prev."""
    wrapped = angles.copy()
    deltas = _shortest_angular_distance(prev, wrapped)
    wrapped = prev + deltas
    return wrapped

if __name__ == '__main__':
    # Example usage and test of the new IK solver
    if IK_SOLVER:
        # print("\n--- Testing IK Solver ---")
        
        # Start with all joints at 0 radians
        zero_angles = [0.0] * NUM_JOINTS
        
        # --- Get Initial Pose via FK ---
        # We need this for the performance test's starting position.
        fk_matrix_initial = get_fk_matrix(zero_angles)
        
        if fk_matrix_initial is not None:
            initial_pos = fk_matrix_initial[:3, 3]
            # Get the 3x3 rotation matrix for the path orientations
            initial_orient_matrix = fk_matrix_initial[:3, :3]
        else:
            print("FK calculation FAILED. Using fallback values for performance test.")
            # Use fallback values to prevent crashes in later tests. Manually calculated from URDF.
            initial_pos = np.array([0.489, 0, 0.3701]) 
            initial_orient_matrix = np.identity(3)
        
        # Note: The single-point IK and sequential path IK tests have been removed for brevity,
        # allowing direct execution of the performance test below.

        # ------------------------------------------------------------------
        # Performance test: 10,000-point straight-line path, silent mode
        # ------------------------------------------------------------------
        import time

        print("\n--- Performance Test: 10000-point path (silent) ---")

        # Generate 10000 points descending 0.05 m in z (tool-tip frame)
        points = 100000
        perf_path = [
            initial_pos + np.array([0.0, 0.0, -0.05 * (i / (points - 1))])
            for i in range(points)
        ]

        # Path orientations are all the same for this test
        path_orientations = [initial_orient_matrix] * points

        t0 = time.perf_counter()
        perf_solutions = solve_ik_path_batch(
            perf_path,
            initial_joint_angles=zero_angles,
            target_orientations=path_orientations,
        )
        t1 = time.perf_counter()

        if perf_solutions is not None:
            total_time_s = t1 - t0
            avg_time_us = (total_time_s / points) * 1_000_000
            print(f"Solved {points} poses in {total_time_s:.3f} s  (avg {avg_time_us:.2f} µs per pose)")
        else:
            print("Performance path IK failed (no solution for at least one pose)") 