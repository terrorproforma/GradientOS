"""ik_solver.py – High-level kinematics API

This module exposes **solver-agnostic** helper functions:
    solve_ik(...)        – inverse kinematics (position + orientation)
    get_fk_matrix(...)   – full 4×4 pose of the tool tip
    get_fk(...)          – convenience: only tool-tip XYZ

Internally we can plug in several different solver back-ends.  Selection is
done **at import time** via the environment variable ``MINI_ARM_SOLVER``.

    "ikfast"   – the original IKFast C++ wrapper (default)
    "numeric"  – the QuIK numeric solver (via ``numeric_wrapper.py``)
    "trac"     – placeholder for future TRAC-IK integration

The goal: callers’ code never changes – you just switch the env-var or call
``set_backend()`` at runtime *before* the first IK/FK call.
"""

from __future__ import annotations

import os
import numpy as np
from scipy.spatial.transform import Rotation as R

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

# -----------------------------------------------------------
# Backend selection helper – import the chosen solver only once
# -----------------------------------------------------------

_BACKEND_NAME: str = os.getenv("MINI_ARM_SOLVER", "ikfast").lower()  # set ikfast as default
# _BACKEND_NAME: str = os.getenv("MINI_ARM_SOLVER", "numeric").lower()   # set numeric as default

# These module-level globals will be filled by the selected backend loader.
_solve_ik_impl = None  # type: ignore
_fk_matrix_impl = None  # type: ignore
NUM_JOINTS = 6  # default until backend sets the real value


def _init_ikfast_backend():
    """Initialise IKFast C++ back-end (original behaviour)."""
    global _solve_ik_impl, _fk_matrix_impl, NUM_JOINTS

    from ikfast_solver.ikfast_wrapper import IKFastSolver  # local import: heavy .so

    try:
        solver = IKFastSolver()
        NUM_JOINTS = solver.num_joints
        globals()["IK_SOLVER"] = solver  # expose for legacy helpers
        print(f"[IK Solver] IKFast back-end initialised for {NUM_JOINTS} joints.")
    except (RuntimeError, OSError) as e:
        print("--- [IK Solver] FATAL ERROR – IKFast backend ---")
        print(f"Failed to load IKFast: {e}")
        solver = None
        globals()["IK_SOLVER"] = None

    def _ikfast_solve_ik(target_position, target_orientation_matrix, initial_joint_angles):
        if solver is None:
            return None

        # fall back to identity if orientation omitted
        target_rotation = (
            np.identity(3)
            if target_orientation_matrix is None
            else np.array(target_orientation_matrix).reshape(3, 3)
        )

        rotated_offset = target_rotation.dot(END_EFFECTOR_OFFSET)
        wrist_position = np.array(target_position) - rotated_offset

        pose_rot_flat = target_rotation.flatten()
        sol = solver.solve_ik(wrist_position, pose_rot_flat, initial_joint_angles)
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


def _init_numeric_backend():
    """Initialise QuIK numeric back-end (python/pybind)."""
    global _solve_ik_impl, _fk_matrix_impl, NUM_JOINTS

    from numeric_solver.numeric_wrapper import (
        numeric_fk,
        numeric_ik,
        init_numeric_solver,
    )

    # Lazy initialisation – path to DH CSV via env var or default location
    dh_path = os.getenv("MINI_ARM_DH_CSV", "mini-6dof-arm/dh_params.csv")
    try:
        kin, solver = init_numeric_solver(dh_path)
        NUM_JOINTS = kin.num_joints if hasattr(kin, "num_joints") else 6
        globals()["IK_SOLVER"] = solver  # expose numeric IKSolver for future use
        print(f"[IK Solver] Numeric (QuIK) back-end initialised for {NUM_JOINTS} joints.")
    except Exception as e:
        print("--- [IK Solver] FATAL ERROR – Numeric backend ---")
        print(f"Failed to load numeric solver: {e}")
        kin = solver = None
        globals()["IK_SOLVER"] = None

    def _numeric_solve_ik(target_position, target_orientation_matrix, initial_joint_angles):
        if solver is None:
            return None

        # Expect orientation as 3×3 matrix or flat 9.
        target_rotation = (
            np.identity(3)
            if target_orientation_matrix is None
            else np.array(target_orientation_matrix).reshape(3, 3)
        )

        # QuIK solver already accepts tool-frame pose directly – **do not** subtract offset.
        quat = R.from_matrix(target_rotation).as_quat()
        sol, _, _, _ = numeric_ik(quat, np.asarray(target_position, dtype=float), initial_joint_angles)
        return sol

    def _numeric_fk_matrix(joint_angles):
        if kin is None:
            return None
        return numeric_fk(joint_angles)

    _solve_ik_impl = _numeric_solve_ik
    _fk_matrix_impl = _numeric_fk_matrix


def _init_backend():
    if _BACKEND_NAME == "ikfast":
        _init_ikfast_backend()
    elif _BACKEND_NAME == "numeric":
        _init_numeric_backend()
    elif _BACKEND_NAME == "trac":
        raise NotImplementedError("TRAC-IK backend not yet integrated.")
    else:
        raise ValueError(f"Unknown MINI_ARM_SOLVER backend '{_BACKEND_NAME}'")


# Initialise the chosen backend immediately so NUM_JOINTS is correct.
_init_backend()

# -----------------------------------------------------------
# Public, solver-independent API
# -----------------------------------------------------------


def solve_ik(*, target_position, target_orientation_matrix=None, initial_joint_angles=None):
    """Solver-agnostic wrapper – dispatches to selected backend and returns joint angles or None."""
    return _solve_ik_impl(target_position, target_orientation_matrix, initial_joint_angles)


def get_fk_matrix(active_joint_angles):
    """Return 4×4 tool-tip pose (world frame) for *active_joint_angles* or None on failure."""
    return _fk_matrix_impl(active_joint_angles)


def get_fk(active_joint_angles):
    """Convenience: return just XYZ position from ``get_fk_matrix``."""
    fk_matrix = get_fk_matrix(active_joint_angles)
    if fk_matrix is not None:
        return fk_matrix[:3, 3]
    return None

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
    
    default_rotation = np.identity(3)
    
    for i in range(num_poses):
        position = path_points[i]
        
        if target_orientations:
            target_rotation = np.array(target_orientations[i]).reshape(3, 3)
        else:
            target_rotation = default_rotation
            
        rotated_offset = target_rotation.dot(END_EFFECTOR_OFFSET)
        wrist_position = np.array(position) - rotated_offset
        
        poses_batch[i, :3] = wrist_position
        poses_batch[i, 3:] = target_rotation.flatten()

    # Call the new batch solver in the wrapper
    joint_solutions = IK_SOLVER.solve_ik_path(poses_batch, start_angles_np)

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