"""numeric_k.py
A high-level façade around the underlying C++ QuIK solver (exposed via
`pyquik`).  The public API intentionally mirrors the expectations of the
rest of the Mini-Arm codebase: you feed it a URDF file (or directly a DH
table) and then call `fk()` / `ik()`.

This keeps 100 % of the heavy lifting inside the highly-optimised C++
code while presenting a clean, dependency-free (no ROS) Python surface.
"""

import numpy as np
from scipy.spatial.transform import Rotation

# ---------------------------------------------------------------------------
# PyQuIK binding helper – build Robot + IKSolver exactly once
# ---------------------------------------------------------------------------

# The real C++ pybind module is typically installed as `pyquik`, but if the
# import fails we fall back to a *mock* that raises clear runtime errors.
try:
    import pyquik  # type: ignore
except ImportError as e:  # pragma: no cover – dev machines may lack the .so
    # Attempt package-relative import (numeric_solver.pyquik.pyquik)
    try:
        import importlib
        pyquik = importlib.import_module("numeric_solver.pyquik.pyquik")  # type: ignore
    except ModuleNotFoundError:
        class _PyQuikMock:
            def __getattr__(self, name):
                raise RuntimeError(
                    "pyquik C++ extension not found. Build / install it or set "
                    "MINI_ARM_SOLVER=ikfast to keep working with IKFast."
                )

        pyquik = _PyQuikMock()  # type: ignore


_NUMERIC_ROBOT = None   # pyquik.Robot instance (cached)
_NUMERIC_IKSOLVER = None  # pyquik.IKSolver instance (cached)


def _ensure_loaded(dh_csv_path: str):
    """Internal helper – lazily load Robot & IKSolver (idempotent)."""
    global _NUMERIC_ROBOT, _NUMERIC_IKSOLVER

    if _NUMERIC_ROBOT is not None:
        return _NUMERIC_ROBOT, _NUMERIC_IKSOLVER

    # ------------------------------------------------------------------
    # 1. Load DH parameters from CSV (identical format to test_numeric.py)
    # ------------------------------------------------------------------
    # Resolve relative path from project root if needed
    from pathlib import Path
    dh_path_obj = Path(dh_csv_path)
    if not dh_path_obj.is_file():
        dh_path_obj = Path(__file__).resolve().parents[2] / dh_csv_path
    if not dh_path_obj.is_file():
        raise FileNotFoundError(f"DH CSV not found at {dh_csv_path} or {dh_path_obj}")

    DH = np.genfromtxt(str(dh_path_obj), delimiter=",", skip_header=1, usecols=(1, 2, 3, 4), dtype=float)

    # ------------------------------------------------------------------
    # 2. Construct Robot & IKSolver via pyquik API
    # ------------------------------------------------------------------
    # pyquik bindings expose Robot.__init__(DH, link_types, q_sign, Tbase, Ttool)
    link_types = [0] * DH.shape[0]  # 0 = revolute for all joints by default
    q_sign = np.ones(DH.shape[0])
    from numpy import identity
    _NUMERIC_ROBOT = pyquik.Robot(
        DH,
        link_types,
        q_sign,
        identity(4),
        identity(4),
    )

    # Empirically, IKSolver expects (robot, max_iterations, ...); we pass
    # defaults identical to previous Python bindings (200 iters etc.).
    try:
        _NUMERIC_IKSOLVER = pyquik.IKSolver(_NUMERIC_ROBOT)
    except Exception as e:
        raise RuntimeError(f"Failed to create pyquik.IKSolver: {e}")

    # Attach a Python-level convenience for batched IK if not present.
    # 2a. Provide single-pose IK alias expected by numeric_wrapper/ik_solver.
    if not hasattr(_NUMERIC_IKSOLVER, "ik"):
        def _ik(quat: np.ndarray, pos: np.ndarray, seed: np.ndarray):
            """Thin wrapper around the C++ .solve method to mimic original API."""
            q_vec, e_vec, iters, br = _NUMERIC_IKSOLVER.solve(quat, pos, seed)
            return np.asarray(q_vec, dtype=float), np.asarray(e_vec, dtype=float), iters, br

        _NUMERIC_IKSOLVER.ik = _ik  # type: ignore

    # 2b. Provide batch path solver if missing
    if not hasattr(_NUMERIC_IKSOLVER, "solve_ik_path"):

        def _solve_ik_path(poses_batch: np.ndarray, initial_joint_angles: np.ndarray):
            """Fallback: loop over poses if C++ batch API unavailable."""
            out = []
            q_seed = initial_joint_angles
            for pose in poses_batch:
                px, py_, pz, *rot_flat = pose
                rot_m = np.array(rot_flat, dtype=float).reshape(3, 3)
                quat = Rotation.from_matrix(rot_m).as_quat()
                q, _, _, _ = _NUMERIC_IKSOLVER.ik(quat, np.array([px, py_, pz]), q_seed)
                if q is None:
                    return None
                out.append(q)
                q_seed = q
            return np.vstack(out)

        _NUMERIC_IKSOLVER.solve_ik_path = _solve_ik_path  # type: ignore

    return _NUMERIC_ROBOT, _NUMERIC_IKSOLVER


def init_numeric_solver(dh_csv_path: str):
    """External entry‐point used by ik_solver – returns (robot, iksolver)."""
    return _ensure_loaded(dh_csv_path)


# ---------------------------------------------------------------------------
# Light functional API requested by ik_solver numeric backend
# ---------------------------------------------------------------------------


def numeric_fk(joint_angles: np.ndarray) -> np.ndarray:
    """Forward kinematics: ndarray(6,) → 4×4 H-matrix (numpy)."""
    robot, _ = _ensure_loaded("mini-6dof-arm/dh_params.csv")
    return robot.fk(joint_angles)  # Assumes pyquik Robot.fk returns np.ndarray


def numeric_ik(quat: np.ndarray, pos: np.ndarray, seed: np.ndarray | None = None):
    """Inverse kinematics wrapper with signature expected by ik_solver.

    Returns (q_star, e_star, iters, reason) just like IKFast’s Python wrapper.
    """
    _, solver = _ensure_loaded("mini-6dof-arm/dh_params.csv")
    if seed is None:
        seed = np.zeros(solver.R.dof if hasattr(solver, "R") else 6, dtype=float)

    return solver.ik(quat, pos, seed)


# Keep orientation helper functions below (already present) unchanged.


def compute_ref_rot(kin) -> np.ndarray:
    """
    Computes the reference rotation matrix for the robot at zero joint angles.

    This function calculates the 3x3 rotation matrix of the end effector when all joint angles are zero,
    using the forward kinematics method of the provided kinematics instance. This reference is essential
    for orientation conversions to handle the specific alignment where the end effector's z-axis points
    along the world's positive x-axis, avoiding gimbal lock in Euler representations.

    Detailed step-by-step explanation:
    1. Assume 'kin' is an instance with a working 'fk' method that takes a numpy array of joint angles
       and returns a 4x4 transformation matrix.
    2. Create a numpy array of zero joint angles with length equal to the number of joints (kin.num_joints),
       using dtype=float for precision.
    3. Call kin.fk(zeros) to get the full 4x4 transformation matrix at zero pose.
    4. Extract the top-left 3x3 submatrix, which is the rotation matrix.
    5. Return this 3x3 matrix as a numpy array with dtype=float.

    Parameters:
    kin (object): A kinematics instance with 'fk' method and 'num_joints' attribute.

    Returns:
    np.ndarray: 3x3 reference rotation matrix with dtype=float.

    Note: If 'kin' does not have 'num_joints', this may need adjustment (e.g., pass num_joints separately).
          All operations use explicit dtype=float to prevent datatype issues.
    """
    if not hasattr(kin, 'fk') or not callable(kin.fk):
        raise ValueError("'kin' must have a callable 'fk' method.")
    num_joints = kin.num_joints if hasattr(kin, 'num_joints') else len(kin)  # Adjust based on actual attribute
    zeros = np.zeros(num_joints, dtype=float)
    T = kin.fk(zeros)
    ref_rot = T[:3, :3]
    return np.asarray(ref_rot, dtype=float)

def user_to_ee_quat(user_euler_deg: np.ndarray, ref_rot: np.ndarray) -> np.ndarray:
    """
    Converts a user-specified world orientation, given as Euler angles (roll, pitch, yaw) in degrees, 
    into the corresponding quaternion for the end effector frame to be used in inverse kinematics.

    This function provides a user-friendly way to specify orientations where (0, 0, 0) corresponds to 
    the zero-angle pose of the arm, with the end effector's z-axis aligned with the world's positive x-axis. 
    It avoids gimbal lock issues present in direct Euler conversions at this pose by composing the user 
    rotation with a reference rotation matrix computed from the zero pose.

    Detailed step-by-step explanation:
    1. The input is taken as a numpy array of three floats representing roll, pitch, and yaw in degrees.
       No abbreviations are used; roll is rotation around the x-axis, pitch around the y-axis, yaw around the z-axis.
    2. Convert the Euler angles to a rotation matrix using the 'xyz' sequence. This sequence means first rotate 
       around x (roll), then y (pitch), then z (yaw). Degrees are explicitly specified to avoid any confusion with radians.
    3. Compose this user rotation matrix with the pre-computed reference rotation matrix (self.ref_rot) by matrix 
       multiplication: R_des = R_user @ self.ref_rot. This ensures that when user inputs (0,0,0), the result is exactly 
       the reference orientation without gimbal lock.
    4. Convert the resulting rotation matrix to a quaternion in (x, y, z, w) order.
    5. Return the quaternion as a numpy array with dtype=float.

    Parameters:
    user_euler_deg (np.ndarray): Array of shape (3,) containing roll, pitch, yaw in degrees, with dtype=float.

    Returns:
    np.ndarray: Quaternion [x, y, z, w] with dtype=float, representing the end effector orientation.

    Note: This function assumes the 'xyz' Euler convention. If a different convention is needed, the code would need adjustment.
          All operations use explicit dtype=float to prevent any datatype issues.
    """
    if user_euler_deg.shape != (3,):
        raise ValueError("user_euler_deg must be a numpy array of shape (3,) with dtype=float.")
    if ref_rot.shape != (3, 3):
        raise ValueError("ref_rot must be a numpy array of shape (3, 3) with dtype=float.")
    user_euler_deg = np.asarray(user_euler_deg, dtype=float)
    ref_rot = np.asarray(ref_rot, dtype=float)
    R_user = Rotation.from_euler('xyz', user_euler_deg, degrees=True).as_matrix()
    R_des = np.matmul(R_user, ref_rot, dtype=float)
    quat = Rotation.from_matrix(R_des).as_quat()  # returns [x,y,z,w]
    return np.asarray(quat, dtype=float)

def ee_to_user_euler(ee_rot_matrix: np.ndarray, ref_rot: np.ndarray) -> np.ndarray:
    """
    Converts an end effector rotation matrix into user world orientation Euler angles (roll, pitch, yaw) in degrees.

    This is the inverse operation of user_to_ee_quat. It takes the rotation matrix of the end effector (as obtained 
    from forward kinematics) and computes the equivalent user-friendly Euler angles where (0, 0, 0) corresponds to 
    the zero-angle pose.

    Detailed step-by-step explanation:
    1. The input is a 3x3 numpy array representing the rotation matrix of the end effector, with dtype=float.
    2. Compute the user rotation matrix by multiplying the input with the reference: R_user = ee_rot_matrix @ self.ref_rot.
       Since the reference matrix is its own inverse, this recovers the relative rotation.
    3. Convert this user rotation matrix to Euler angles using the 'xyz' sequence, in degrees.
    4. Return the Euler angles as a numpy array with dtype=float.

    Parameters:
    ee_rot_matrix (np.ndarray): 3x3 rotation matrix with dtype=float.

    Returns:
    np.ndarray: Array of shape (3,) containing roll, pitch, yaw in degrees, with dtype=float.

    Note: This function assumes the 'xyz' Euler convention matching user_to_ee_quat. All operations use explicit dtype=float.
          If the matrix is not a valid rotation matrix, the behavior is undefined (as per scipy Rotation).
    """
    if ee_rot_matrix.shape != (3, 3):
        raise ValueError("ee_rot_matrix must be a numpy array of shape (3, 3) with dtype=float.")
    if ref_rot.shape != (3, 3):
        raise ValueError("ref_rot must be a numpy array of shape (3, 3) with dtype=float.")
    ee_rot_matrix = np.asarray(ee_rot_matrix, dtype=float)
    ref_rot = np.asarray(ref_rot, dtype=float)
    R_user = np.matmul(ee_rot_matrix, ref_rot, dtype=float)  # Note: ref_rot is its own inverse
    euler_deg = Rotation.from_matrix(R_user).as_euler('xyz', degrees=True)
    return np.asarray(euler_deg, dtype=float)
