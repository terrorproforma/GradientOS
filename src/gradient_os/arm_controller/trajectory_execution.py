# Contains the logic for planning and executing timed trajectories,
# including the high-fidelity planner and the closed-loop executor thread. 
import time
import math
import statistics
import datetime
import numpy as np
from scipy.signal import savgol_filter
from scipy.spatial.transform import Rotation as R
from typing import Sequence, Union
from pathlib import Path
import os
import csv
import threading

from typing import Optional

try:
    from .. import ik_solver
    from .. import trajectory_planner
except ImportError:
    print("ERROR: Missing 'ik_solver' or 'trajectory_planner'. Ensure they are in the python path.")
    ik_solver = None
    trajectory_planner = None

from . import utils
from . import servo_driver
from . import servo_protocol
from . import robot_config
from .backends import registry as backend_registry
from .actuator_interface import ActuatorBackend
from ..kinematics import runtime as kinematics_runtime


# =============================================================================
# Backend Accessor Functions
# =============================================================================

def _get_backend() -> Optional[ActuatorBackend]:
    """Returns the active ActuatorBackend instance, or None if not set."""
    try:
        return backend_registry.get_active_backend()
    except backend_registry.BackendInstanceNotSetError:
        return None


def _use_backend() -> bool:
    """Returns True if an ActuatorBackend instance is active and initialized."""
    backend = _get_backend()
    return backend is not None and backend.is_initialized


def _build_primary_feedback_ids() -> list[int]:
    """
    Build a list of primary servo IDs for feedback (one per logical joint).
    Uses the first servo ID from each logical joint's actuator list.
    """
    primary_ids = []
    active_robot = robot_config.get_active_robot()
    logical_map = active_robot.logical_joint_to_actuator_ids
    num_logical_joints = active_robot.num_logical_joints
    # Sort by logical joint number (1-based in the map)
    for joint_num in sorted(logical_map.keys()):
        actuator_ids = logical_map[joint_num]
        if actuator_ids and joint_num <= num_logical_joints:  # Skip gripper (joint 7)
            primary_ids.append(actuator_ids[0])
    return primary_ids


def _build_logical_to_physical_index_map() -> dict[int, list[int]]:
    """
    Build a mapping from logical joint index (0-based) to physical servo indices in SERVO_IDS.
    """
    active_robot = robot_config.get_active_robot()
    logical_map = active_robot.logical_joint_to_actuator_ids
    num_logical_joints = active_robot.num_logical_joints
    servo_ids = robot_config.SERVO_IDS
    result = {}
    for joint_num in sorted(logical_map.keys()):
        if joint_num > num_logical_joints:  # Skip gripper
            continue
        actuator_ids = logical_map[joint_num]
        indices = []
        for aid in actuator_ids:
            if aid in servo_ids:
                indices.append(servo_ids.index(aid))
        result[joint_num - 1] = indices  # Convert to 0-based
    return result


def _get_twin_motor_pairs() -> list[tuple[int, int]]:
    """
    Get the twin motor pairs (primary_id, secondary_id) from robot config.
    Returns a list of tuples like [(20, 21), (30, 31)].
    """
    twin_map = robot_config.get_active_robot().twin_motor_actuator_ids
    pairs = []
    # twin_map is like {1: 21, 2: 31} mapping logical joint to secondary servo ID
    logical_map = robot_config.get_active_robot().logical_joint_to_actuator_ids
    for logical_joint, secondary_id in twin_map.items():
        # The logical_joint here is 1-based index in the physical config
        # We need to find the primary ID for this joint
        actuator_ids = logical_map.get(logical_joint + 1, [])  # +1 because logical_joint in twin_map is 0-based offset
        if len(actuator_ids) >= 2:
            primary_id = actuator_ids[0]
            pairs.append((primary_id, secondary_id))
    return pairs

JointTraj = Union[Sequence[Sequence[float]], np.ndarray]

MAX_JOINT_STEP_RAD = float(os.environ.get("MINI_ARM_MAX_JOINT_STEP_RAD", "0.75"))
JOINT_LIMIT_MARGIN_RAD = float(os.environ.get("MINI_ARM_JOINT_LIMIT_MARGIN_RAD", "0.03"))
MAX_CART_RESIDUAL_M = float(os.environ.get("MINI_ARM_MAX_CART_RESIDUAL_M", "0.012"))
MAX_ORIENT_RESIDUAL_DEG = float(os.environ.get("MINI_ARM_MAX_ORIENT_RESIDUAL_DEG", "6.0"))


def _set_planner_diagnostics(payload: dict) -> None:
    utils.trajectory_state_set("last_planner_diagnostics", payload)


def _json_safe_float(value: float) -> float | None:
    if not np.isfinite(value):
        return None
    return float(value)


def _rotation_residual_deg(actual: np.ndarray, target: np.ndarray) -> float:
    try:
        delta = R.from_matrix(target.T @ actual)
        return float(np.rad2deg(delta.magnitude()))
    except Exception:
        return float("inf")


def _densify_cartesian_path(
    cartesian_points: list,
    orientations_list: list[np.ndarray],
    *,
    max_segment_m: float = 0.01,
) -> tuple[list[list[float]], list[np.ndarray]]:
    if len(cartesian_points) <= 1:
        return [list(map(float, p)) for p in cartesian_points], list(orientations_list)

    dense_points: list[list[float]] = [list(map(float, cartesian_points[0]))]
    dense_orientations: list[np.ndarray] = [np.asarray(orientations_list[0], dtype=float).reshape(3, 3)]
    for idx in range(1, len(cartesian_points)):
        p0 = np.asarray(cartesian_points[idx - 1], dtype=float)
        p1 = np.asarray(cartesian_points[idx], dtype=float)
        seg_len = float(np.linalg.norm(p1 - p0))
        steps = max(1, int(math.ceil(seg_len / max_segment_m)))
        o1 = np.asarray(orientations_list[idx], dtype=float).reshape(3, 3)
        for step in range(1, steps + 1):
            t = step / steps
            pi = (1.0 - t) * p0 + t * p1
            dense_points.append(pi.tolist())
            # Keep deterministic orientation interpolation by selecting end orientation for each dense sample.
            dense_orientations.append(o1)
    return dense_points, dense_orientations


def _validate_joint_trajectory_gates(
    joint_trajectory: list[list[float]],
    cartesian_points: list,
    orientations_list: list[np.ndarray],
) -> tuple[bool, str, dict[str, float | None]]:
    if not joint_trajectory:
        return False, "IK_NO_SOLUTION", {}

    num_joints = len(joint_trajectory[0])
    max_joint_step = 0.0
    min_limit_margin = float("inf")
    max_cart_residual = 0.0
    max_orient_residual = 0.0

    limits = utils.LOGICAL_JOINT_LIMITS_RAD or [(-math.inf, math.inf)] * num_joints

    for i in range(len(joint_trajectory)):
        q = np.asarray(joint_trajectory[i], dtype=float)
        if i > 0:
            prev = np.asarray(joint_trajectory[i - 1], dtype=float)
            step = float(np.max(np.abs(q - prev)))
            max_joint_step = max(max_joint_step, step)
            if step > MAX_JOINT_STEP_RAD:
                return False, "IK_JUMP_REJECTED", {
                    "max_joint_step_rad": _json_safe_float(max_joint_step),
                    "joint_limit_margin_rad": _json_safe_float(min_limit_margin),
                    "cartesian_residual_m": _json_safe_float(max_cart_residual),
                    "orientation_residual_deg": _json_safe_float(max_orient_residual),
                }

        for j in range(min(num_joints, len(limits))):
            lo, hi = limits[j]
            margin = min(float(q[j] - lo), float(hi - q[j]))
            min_limit_margin = min(min_limit_margin, margin)
            if margin < JOINT_LIMIT_MARGIN_RAD:
                return False, "LIMIT_VIOLATION", {
                    "max_joint_step_rad": _json_safe_float(max_joint_step),
                    "joint_limit_margin_rad": _json_safe_float(min_limit_margin),
                    "cartesian_residual_m": _json_safe_float(max_cart_residual),
                    "orientation_residual_deg": _json_safe_float(max_orient_residual),
                }

        if i < len(cartesian_points):
            fk_matrix = ik_solver.get_fk_matrix(q)
            if fk_matrix is None:
                return False, "FK_VALIDATION_FAILED", {
                    "max_joint_step_rad": _json_safe_float(max_joint_step),
                    "joint_limit_margin_rad": _json_safe_float(min_limit_margin),
                    "cartesian_residual_m": _json_safe_float(max_cart_residual),
                    "orientation_residual_deg": _json_safe_float(max_orient_residual),
                }
            fk_pos = np.asarray(fk_matrix[:3, 3], dtype=float)
            target_pos = np.asarray(cartesian_points[i], dtype=float).reshape(3)
            cart_residual = float(np.linalg.norm(fk_pos - target_pos))
            max_cart_residual = max(max_cart_residual, cart_residual)
            if cart_residual > MAX_CART_RESIDUAL_M:
                return False, "CARTESIAN_RESIDUAL_EXCEEDED", {
                    "max_joint_step_rad": _json_safe_float(max_joint_step),
                    "joint_limit_margin_rad": _json_safe_float(min_limit_margin),
                    "cartesian_residual_m": _json_safe_float(max_cart_residual),
                    "orientation_residual_deg": _json_safe_float(max_orient_residual),
                }

            if i < len(orientations_list):
                target_rot = np.asarray(orientations_list[i], dtype=float).reshape(3, 3)
                orient_residual = _rotation_residual_deg(np.asarray(fk_matrix[:3, :3], dtype=float), target_rot)
                max_orient_residual = max(max_orient_residual, orient_residual)
                if orient_residual > MAX_ORIENT_RESIDUAL_DEG:
                    return False, "ORIENTATION_RESIDUAL_EXCEEDED", {
                        "max_joint_step_rad": _json_safe_float(max_joint_step),
                        "joint_limit_margin_rad": _json_safe_float(min_limit_margin),
                        "cartesian_residual_m": _json_safe_float(max_cart_residual),
                        "orientation_residual_deg": _json_safe_float(max_orient_residual),
                    }

    if min_limit_margin == float("inf"):
        min_limit_margin = float("inf")

    return True, "OK", {
        "max_joint_step_rad": _json_safe_float(max_joint_step),
        "joint_limit_margin_rad": _json_safe_float(min_limit_margin),
        "cartesian_residual_m": _json_safe_float(max_cart_residual),
        "orientation_residual_deg": _json_safe_float(max_orient_residual),
    }

def _save_executor_diagnostics_charts(
    mode: str,
    session_id: str | None,
    time_step: float | None,
    loop_durations: list[float],
    write_durations: list[float],
    abs_errors_per_joint: list[list[float]] | None = None,
    target_angles_per_joint: list[list[float]] | None = None,
    actual_angles_per_joint: list[list[float]] | None = None,
    read_durations: list[float] | None = None,
    compute_durations: list[float] | None = None,
    sync_profiles: list[tuple[float, float, float]] | None = None,
) -> Path | None:
    """Render and save diagnostics charts for both executors in one place.

    Args:
        mode: 'open_loop' or 'closed_loop'.
        session_id: optional session identifier used for output folder.
        time_step: control period in seconds; used to plot deadline line if provided.
        loop_durations: per-cycle total durations (s).
        write_durations: write durations (s).
        abs_errors_per_joint: per-joint |error| arrays (rad).
        target_angles_per_joint: per-joint target angle arrays (rad).
        actual_angles_per_joint: per-joint actual angle arrays (rad).
        read_durations: read durations (s), if available (closed-loop).
        compute_durations: compute durations (s), if available (closed-loop).
        sync_profiles: list of (write, read, parse) triplets in seconds, optional.

    Returns:
        The output directory Path, or None on failure.
    """
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except Exception as e:
        print(f"[Pi Diag] WARNING: Matplotlib unavailable for diagnostics: {e}")
        return None

    try:
        # Resolve output directory
        if session_id:
            out_dir = Path(f"diagnostics/{mode}/{session_id}")
        else:
            ts = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
            out_dir = Path(f"diagnostics/{mode}/{ts}")
        out_dir.mkdir(exist_ok=True, parents=True)

        # 0) Save angle telemetry as CSV for PID tuning
        try:
            has_target = bool(target_angles_per_joint) and any(target_angles_per_joint)
            has_actual = bool(actual_angles_per_joint) and any(actual_angles_per_joint)
            has_error = bool(abs_errors_per_joint) and any(abs_errors_per_joint)

            if has_target or has_actual or has_error:
                max_len = 0
                for arrs in (target_angles_per_joint or []):
                    max_len = max(max_len, len(arrs))
                for arrs in (actual_angles_per_joint or []):
                    max_len = max(max_len, len(arrs))
                for arrs in (abs_errors_per_joint or []):
                    max_len = max(max_len, len(arrs))

                header = ["step"]
                if time_step is not None:
                    header.append("time_s")
                # Target columns
                for j in range(utils.NUM_LOGICAL_JOINTS):
                    header.append(f"T_J{j+1}_rad")
                # Actual columns
                for j in range(utils.NUM_LOGICAL_JOINTS):
                    header.append(f"A_J{j+1}_rad")
                # Error columns (abs)
                for j in range(utils.NUM_LOGICAL_JOINTS):
                    header.append(f"Eabs_J{j+1}_rad")

                csv_path = out_dir / "angles.csv"
                with open(csv_path, "w", newline="") as fp:
                    writer = csv.writer(fp)
                    writer.writerow(header)
                    for idx in range(max_len):
                        row = [idx]
                        if time_step is not None:
                            row.append(idx * time_step)
                        # Targets
                        for j in range(utils.NUM_LOGICAL_JOINTS):
                            val = ""
                            if has_target and target_angles_per_joint and j < len(target_angles_per_joint):
                                ta = target_angles_per_joint[j]
                                if idx < len(ta):
                                    val = ta[idx]
                            row.append(val)
                        # Actuals
                        for j in range(utils.NUM_LOGICAL_JOINTS):
                            val = ""
                            if has_actual and actual_angles_per_joint and j < len(actual_angles_per_joint):
                                aa = actual_angles_per_joint[j]
                                if idx < len(aa):
                                    val = aa[idx]
                            row.append(val)
                        # Errors (abs)
                        for j in range(utils.NUM_LOGICAL_JOINTS):
                            val = ""
                            if has_error and abs_errors_per_joint and j < len(abs_errors_per_joint):
                                ea = abs_errors_per_joint[j]
                                if idx < len(ea):
                                    val = ea[idx]
                            row.append(val)
                        writer.writerow(row)
        except Exception as _e:
            pass

        # 1) Timing chart
        try:
            fig, ax = plt.subplots(1, 1, figsize=(10, 4))
            ax.plot([d * 1000 for d in loop_durations], label="total")
            if read_durations is not None:
                ax.plot([d * 1000 for d in read_durations], label="read")
            if compute_durations is not None:
                ax.plot([d * 1000 for d in compute_durations], label="compute")
            ax.plot([d * 1000 for d in write_durations], label="write")
            if time_step is not None:
                ax.axhline(time_step * 1000, color="red", linestyle="--", label="deadline")
            ax.set_xlabel("Cycle index")
            ax.set_ylabel("Duration (ms)")
            ax.set_title(("Closed" if mode == "closed_loop" else "Open") + "-loop cycle timing")
            ax.legend()
            timing_path = out_dir / "timing.png"
            fig.tight_layout()
            fig.savefig(timing_path)
            plt.close(fig)
        except Exception as _e:
            pass

        # 2) Error chart per joint
        if abs_errors_per_joint is not None:
            try:
                fig, ax = plt.subplots(1, 1, figsize=(10, 4))
                for j_idx, errs in enumerate(abs_errors_per_joint):
                    if errs:
                        ax.plot([math.degrees(e) for e in errs], label=f"J{j_idx+1}")
                ax.set_xlabel("Cycle index")
                ax.set_ylabel("Abs error (deg)")
                ax.set_title("Tracking error per joint")
                ax.legend(ncol=3)
                error_path = out_dir / "error.png"
                fig.tight_layout()
                fig.savefig(error_path)
                plt.close(fig)
            except Exception as _e:
                pass

        # 3) Target vs Actual joint angles
        if target_angles_per_joint is not None or actual_angles_per_joint is not None:
            try:
                fig, axes = plt.subplots(2, 3, figsize=(12, 6), sharex=True)
                for j_idx in range(utils.NUM_LOGICAL_JOINTS):
                    r, c = divmod(j_idx, 3)
                    ax = axes[r][c]
                    if target_angles_per_joint and j_idx < len(target_angles_per_joint) and target_angles_per_joint[j_idx]:
                        ax.plot([math.degrees(v) for v in target_angles_per_joint[j_idx]], label="target")
                    if actual_angles_per_joint and j_idx < len(actual_angles_per_joint) and actual_angles_per_joint[j_idx]:
                        ax.plot([math.degrees(v) for v in actual_angles_per_joint[j_idx]], label="actual")
                    ax.set_title(f"J{j_idx+1}")
                    ax.set_ylabel("deg")
                    ax.grid(True, alpha=0.3)
                axes[1][0].set_xlabel("Cycle index")
                axes[1][1].set_xlabel("Cycle index")
                axes[1][2].set_xlabel("Cycle index")
                handles, labels = axes[0][0].get_legend_handles_labels()
                if handles:
                    fig.legend(handles, labels, loc="upper center", ncol=2)
                fig.suptitle(("Closed" if mode == "closed_loop" else "Open") + "-loop Target vs Actual Joint Angles")
                angles_path = out_dir / "angles.png"
                fig.tight_layout(rect=[0, 0, 1, 0.95])
                fig.savefig(angles_path)
                plt.close(fig)
            except Exception as _e:
                pass

        # 4) Sync Read internals (closed loop specific)
        if sync_profiles:
            try:
                w_list, r_list, p_list = zip(*sync_profiles)
                fig, ax = plt.subplots(1, 1, figsize=(10, 4))
                ax.plot([d * 1000 for d in w_list], label="write")
                ax.plot([d * 1000 for d in r_list], label="read")
                ax.plot([d * 1000 for d in p_list], label="parse")
                ax.set_xlabel("Cycle index")
                ax.set_ylabel("Duration (ms)")
                ax.set_title("Sync Read internal timing")
                ax.legend()
                sync_path = out_dir / "sync.png"
                fig.tight_layout()
                fig.savefig(sync_path)
                plt.close(fig)
            except Exception as _e:
                pass

        print(f"[Pi Diag] Charts saved → {out_dir}")
        return out_dir
    except Exception as e:
        print(f"[Pi Diag] WARNING: Failed to generate diagnostics charts: {e}")
        return None

def _plan_smooth_move(start_q: list[float], target_pos: np.ndarray, velocity: float, acceleration: float, frequency: int, use_smoothing: bool) -> list | None:
    """
    DEPRECATED: This function is a legacy wrapper. Use _plan_linear_move instead.
    Plans a smooth, orientation-locked joint-space trajectory from a starting
    joint configuration to a target Cartesian position.
    """
    t_start_plan = time.monotonic()
    
    # 1. Get start pose from the provided start_q for path generation and orientation lock
    start_pos_from_q = ik_solver.get_fk(start_q)
    if start_pos_from_q is None:
        print("[Pi Plan] ERROR: Cannot start planning, failed to get start pose from initial_q.")
        return None
    # Optional diagnostics: compare FK position sources and log any mismatch
    try:
        if os.environ.get("MINI_ARM_IK_LOG", "0") == "1":
            mx = ik_solver.get_fk_matrix(start_q)
            if mx is not None:
                pos_from_mx = mx[:3, 3]
                delta = np.linalg.norm(start_pos_from_q - pos_from_mx)
                if delta > 1e-3:
                    print(f"[Pi Plan] WARNING: FK position mismatch at start (||get_fk - get_fk_matrix|| = {delta*1000:.2f} mm)")
    except Exception as _e:
        pass
    
    # 2. Generate the ideal Cartesian trajectory
    ideal_cartesian_points = trajectory_planner.generate_trapezoidal_profile(
        start_pos_from_q, target_pos, velocity, acceleration, frequency
    )
    if not ideal_cartesian_points:
        print("[Pi Plan] ERROR: Trajectory planner failed to generate a path.")
        return None

    # 3. Plan the high-fidelity joint-space path from the Cartesian points.
    #    By default, this locks orientation to the starting pose since forced_orientation is None.
    final_joint_trajectory = _plan_high_fidelity_trajectory(
        cartesian_points=ideal_cartesian_points,
        start_q=start_q,
        use_smoothing=use_smoothing
    )
        
    t_end_plan = time.monotonic()
    
    if final_joint_trajectory:
        print(f"[Pi Plan] Planning complete for move. Took {(t_end_plan - t_start_plan) * 1000:.2f} ms")
        # Optional diagnostics: verify endpoint FK against target
        try:
            if os.environ.get("MINI_ARM_IK_LOG", "0") == "1":
                fk_end = ik_solver.get_fk(final_joint_trajectory[-1])
                if fk_end is not None:
                    err = np.linalg.norm(fk_end - target_pos)
                    print(f"[Pi Plan] Endpoint FK error vs target: {err*1000:.2f} mm")
        except Exception as _e:
            pass

    return final_joint_trajectory


def _unwrap_joint_trajectory(joint_trajectory: JointTraj) -> list[list[float]]:
    """
    Post-processes a joint trajectory to correct for 2*pi jumps on revolute joints.
    This is critical for ensuring the robot takes the shortest path between two
    points in a trajectory and doesn't unwind its wrist joints unnecessarily.
    It respects the hard joint limits defined in `utils.py`.

    Args:
        trajectory: A list of joint angle solutions from the IK solver.

    Returns:
        The unwrapped, continuous joint trajectory.
    """
    # trajectory might be a numpy array; checking its truth value directly is ambiguous.
    if joint_trajectory is None or len(joint_trajectory) == 0:
        return []

    unwrapped_trajectory = [list(joint_trajectory[0])]  # Start with the first solution as a mutable list
    for i in range(1, len(joint_trajectory)):
        last_angles = unwrapped_trajectory[-1]
        current_angles = list(joint_trajectory[i]) # Make a mutable copy

        for j in range(len(current_angles)):
            # Find the closest equivalent angle to the previous one by checking all rotations
            diff = current_angles[j] - last_angles[j]
            
            # If the jump is bigger than 180 degrees, it's likely a wrap-around
            if abs(diff) > math.pi:
                # Propose a corrected angle by "unwrapping" the jump
                num_wraps = round(diff / (2 * math.pi))
                potential_new_angle = current_angles[j] - num_wraps * 2 * math.pi

                # Get the limits for the current logical joint
                min_limit, max_limit = utils.LOGICAL_JOINT_LIMITS_RAD[j]
                
                # Check if the proposed angle is within the joint's physical limits.
                if min_limit <= potential_new_angle <= max_limit:
                    # If it is, accept the unwrapped angle for a smoother path.
                    current_angles[j] = potential_new_angle
                else:
                    # If not, the IK solver likely chose the "long way around" on purpose
                    # to avoid a joint limit. We respect this decision and do not unwrap.
                    pass
        
        unwrapped_trajectory.append(current_angles)

    return unwrapped_trajectory


def _plan_linear_move(start_q: list[float],
                      target_pos: np.ndarray,
                      velocity: float,
                      acceleration: float,
                      frequency: int,
                      use_smoothing: bool,
                      forced_orientation: np.ndarray = None,
                      interpolate_orientation: bool = True) -> list | None:
    """
    Plans a smooth, orientation-locked, straight-line joint-space trajectory.
    This function generates a linear Cartesian path and then calls the high-fidelity
    planner to generate the corresponding joint-space path.

    Args:
        start_q: The starting joint angles of the robot.
        target_pos: The target Cartesian position (as a numpy array).
        velocity: The maximum velocity for the move.
        acceleration: The acceleration for the move.
        frequency: The frequency of the generated path points.
        use_smoothing: Whether to apply a Savitzky-Golay filter to the final path.
        forced_orientation: A 3x3 rotation matrix to lock the orientation to. 
                              If None, orientation is locked to the start pose.
        interpolate_orientation: Whether to interpolate the orientation along the path.

    Returns:
        A list of lists representing the dense joint-space trajectory, or None on failure.
    """
    # 1. Get start pose from the provided start_q for path generation
    start_pos_from_q = ik_solver.get_fk(start_q)
    if start_pos_from_q is None:
        print("[Pi Plan] ERROR: Cannot start planning, failed to get start position from initial_q.")
        return None
    # Optional diagnostics: compare FK position sources and log any mismatch
    try:
        if os.environ.get("MINI_ARM_IK_LOG", "0") == "1":
            mx = ik_solver.get_fk_matrix(start_q)
            if mx is not None:
                pos_from_mx = mx[:3, 3]
                delta = np.linalg.norm(start_pos_from_q - pos_from_mx)
                if delta > 1e-3:
                    print(f"[Pi Plan] WARNING: FK position mismatch at start (||get_fk - get_fk_matrix|| = {delta*1000:.2f} mm)")
    except Exception as _e:
        pass

    # 2. Generate the ideal linear Cartesian trajectory
    ideal_cartesian_points = trajectory_planner.generate_trapezoidal_profile(
        start_pos_from_q, target_pos, velocity, acceleration, frequency
    )
    if not ideal_cartesian_points:
        print("[Pi Plan] ERROR: Trajectory planner failed to generate a path.")
        return None

    # --- Orientation Handling ---
    orientations_list = None
    if forced_orientation is not None and interpolate_orientation:
        # Build SLERP from start orientation (derived from start_q) to forced_orientation
        initial_pose_mx = ik_solver.get_fk_matrix(start_q)
        if initial_pose_mx is not None:
            start_orientation_mx = initial_pose_mx[:3, :3]
            try:
                from scipy.spatial.transform import Rotation as _R, Slerp as _Slerp
                key_rots = _R.concatenate([_R.from_matrix(start_orientation_mx), _R.from_matrix(forced_orientation)])
                key_times = [0, 1]
                slerp = _Slerp(key_times, key_rots)
                times = np.linspace(0, 1, len(ideal_cartesian_points))
                interpolated_rots = slerp(times)
                orientations_list = [r.as_matrix() for r in interpolated_rots]
            except Exception as e:
                print(f"[Pi Plan] WARNING: Failed to interpolate orientation: {e}. Reverting to fixed orientation.")
                orientations_list = [forced_orientation] * len(ideal_cartesian_points)
        else:
            orientations_list = [forced_orientation] * len(ideal_cartesian_points)

    final_joint_trajectory = _plan_high_fidelity_trajectory(
        cartesian_points=ideal_cartesian_points,
        start_q=start_q,
        use_smoothing=use_smoothing,
        forced_orientation=forced_orientation if orientations_list is None else None,
        orientations_list=orientations_list,
    )

    # Optional diagnostics: verify endpoint FK against target
    try:
        if final_joint_trajectory is not None and os.environ.get("MINI_ARM_IK_LOG", "0") == "1":
            fk_end = ik_solver.get_fk(final_joint_trajectory[-1])
            if fk_end is not None:
                err = np.linalg.norm(fk_end - target_pos)
                print(f"[Pi Plan] Endpoint FK error vs target: {err*1000:.2f} mm")
    except Exception as _e:
        pass

    return final_joint_trajectory


def _plan_high_fidelity_trajectory(cartesian_points: list,
                                   start_q: list[float],
                                   use_smoothing: bool = True,
                                   forced_orientation: np.ndarray = None,
                                   orientations_list: list[np.ndarray] | None = None) -> list | None:
    """
    Takes a list of Cartesian points and plans a complete, smoothed joint-space trajectory.
    This is the core planning function, which solves IK for every point on the path.

    Args:
        cartesian_points: The list of [x,y,z] points for the tool tip.
        start_q: The starting joint angles, used for orientation lock and as the initial seed.
        use_smoothing: Whether to apply a Savitzky-Golay filter to the final path.
        forced_orientation: A 3x3 rotation matrix to lock the orientation to. 
                              If None, orientation is locked to the start pose.
        orientations_list: A list of 3x3 rotation matrices to use for orientation interpolation.
                           If None, orientation is locked to the start pose.

    Returns:
        The final, dense list of joint angle solutions, or None on failure.
    """
    try:
        utils.set_motion_state("PLANNING")
    except Exception:
        pass
    print(f"[Pi Plan HF] Planning high-fidelity trajectory for {len(cartesian_points)} points.")
    # Optional diagnostics: compare FK position sources at start
    try:
        if os.environ.get("MINI_ARM_IK_LOG", "0") == "1":
            pos_fk = ik_solver.get_fk(start_q)
            mx = ik_solver.get_fk_matrix(start_q)
            if pos_fk is not None and mx is not None:
                pos_from_mx = mx[:3, 3]
                delta = np.linalg.norm(pos_fk - pos_from_mx)
                if delta > 1e-3:
                    print(f"[Pi Plan HF] WARNING: FK position mismatch at start (||get_fk - get_fk_matrix|| = {delta*1000:.2f} mm)")
    except Exception as _e:
        pass
    
    # Determine orientations for each path point
    if orientations_list is None:
        if forced_orientation is not None:
            target_orientation = forced_orientation
        else:
            initial_pose_matrix = ik_solver.get_fk_matrix(start_q)
            if initial_pose_matrix is None:
                print("[Pi Plan HF] ERROR: FK failed on start_q. Cannot determine orientation lock.")
                try:
                    if utils.get_motion_state() == "PLANNING":
                        utils.set_motion_state("IDLE")
                except Exception:
                    pass
                return None
            target_orientation = initial_pose_matrix[:3, :3]
        orientations_list = [target_orientation] * len(cartesian_points)

    profile_revision = None
    try:
        profile_revision = int(kinematics_runtime.get_runtime_state_snapshot()["revision"])
    except Exception:
        profile_revision = None

    planner_diag = {
        "reason_code": "IK_NO_SOLUTION",
        "seed_used": "start_q",
        "fallback_level": 0,
        "residuals": {},
        "profile_revision": profile_revision,
        "smoothing_applied": False,
    }

    dense_points, dense_orientations = _densify_cartesian_path(
        cartesian_points,
        orientations_list,
        max_segment_m=0.01,
    )
    attempts = [
        ("batch", cartesian_points, orientations_list, 0),
        ("sequential", cartesian_points, orientations_list, 1),
        ("dense_batch", dense_points, dense_orientations, 2),
        ("dense_sequential", dense_points, dense_orientations, 3),
    ]

    selected_unwrapped: list[list[float]] | None = None
    selected_points: list = cartesian_points
    selected_orientations: list[np.ndarray] = orientations_list
    selected_residuals: dict[str, float | None] = {}
    selected_attempt = "batch"

    for attempt_name, attempt_points, attempt_orientations, fallback_level in attempts:
        t_start_ik = time.monotonic()
        if attempt_name.endswith("batch"):
            joint_trajectory = ik_solver.solve_ik_path_batch(
                path_points=attempt_points,
                initial_joint_angles=start_q,
                target_orientations=attempt_orientations,
            )
        else:
            joint_trajectory = ik_solver.solve_ik_path_sequential(
                path_points=attempt_points,
                initial_joint_angles=start_q,
                target_orientations=attempt_orientations,
            )
        t_end_ik = time.monotonic()
        print(
            f"[Pi Plan HF] IK attempt '{attempt_name}' finished in "
            f"{(t_end_ik - t_start_ik) * 1000:.2f} ms"
        )

        if joint_trajectory is None:
            planner_diag["reason_code"] = "IK_NO_SOLUTION"
            planner_diag["fallback_level"] = fallback_level
            continue

        unwrapped_joint_trajectory = _unwrap_joint_trajectory(joint_trajectory)
        is_valid, reason_code, residuals = _validate_joint_trajectory_gates(
            unwrapped_joint_trajectory,
            attempt_points,
            attempt_orientations,
        )
        if is_valid:
            selected_unwrapped = unwrapped_joint_trajectory
            selected_points = attempt_points
            selected_orientations = attempt_orientations
            selected_residuals = residuals
            selected_attempt = attempt_name
            planner_diag["reason_code"] = "OK"
            planner_diag["fallback_level"] = fallback_level
            planner_diag["seed_used"] = (
                "prev_valid_seed" if "sequential" in attempt_name else "start_q"
            )
            break

        planner_diag["reason_code"] = reason_code
        planner_diag["fallback_level"] = fallback_level
        planner_diag["residuals"] = residuals

    if selected_unwrapped is None:
        _set_planner_diagnostics(planner_diag)
        print(
            f"[Pi Plan HF] ERROR: IK solve rejected. reason_code={planner_diag['reason_code']} "
            f"fallback_level={planner_diag['fallback_level']}"
        )
        try:
            if utils.get_motion_state() == "PLANNING":
                utils.set_motion_state("IDLE")
        except Exception:
            pass
        return None

    # 3. Post-process the trajectory (smooth + safety re-validation).
    final_joint_trajectory = selected_unwrapped
    if use_smoothing:
        raw_joint_trajectory_np = np.array(selected_unwrapped).T
        window_length = 15
        polyorder = 3

        if window_length > len(selected_unwrapped):
            window_length = max(polyorder + 1, len(selected_unwrapped) - 1)
            if window_length % 2 == 0:
                window_length -= 1

        if window_length > polyorder:
            smoothed_joint_trajectory_np = savgol_filter(
                raw_joint_trajectory_np, window_length, polyorder, axis=1
            )
            smoothed_candidate = smoothed_joint_trajectory_np.T.tolist()
            is_valid_smooth, smooth_reason, smooth_residuals = _validate_joint_trajectory_gates(
                smoothed_candidate,
                selected_points,
                selected_orientations,
            )
            if is_valid_smooth:
                final_joint_trajectory = smoothed_candidate
                selected_residuals = smooth_residuals
                planner_diag["smoothing_applied"] = True
            else:
                # Safety-first fallback: keep unsmoothed segment if filter violates gates.
                planner_diag["reason_code"] = smooth_reason
                planner_diag["smoothing_disabled_reason"] = smooth_reason
                planner_diag["smoothing_residuals"] = smooth_residuals
                final_joint_trajectory = selected_unwrapped
        
    print("[Pi Plan HF] Path post-processing complete.")
    # Optional diagnostics: verify endpoint FK against last Cartesian target
    try:
        if os.environ.get("MINI_ARM_IK_LOG", "0") == "1" and cartesian_points:
            fk_end = ik_solver.get_fk(final_joint_trajectory[-1]) if final_joint_trajectory else None
            if fk_end is not None:
                err = np.linalg.norm(fk_end - np.array(cartesian_points[-1]))
                print(f"[Pi Plan HF] Endpoint FK error vs last path point: {err*1000:.2f} mm")
    except Exception as _e:
        pass

    # ------------------------------
    # Diagnostics: save smoothed joint path
    # ------------------------------
    if os.environ.get("MINI_ARM_IK_LOG", "0") == "1":
        try:
            import csv, datetime
            from pathlib import Path

            session_id = utils.trajectory_state.get('diagnostics_session_id')
            folder_type = utils.trajectory_state.get('diagnostics_folder_type', 'open_loop')

            if session_id:
                out_dir = Path(f"diagnostics/{folder_type}/{session_id}")
            else:
                ts = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
                out_dir = Path(f"diagnostics/{folder_type}/{ts}")
                out_dir.mkdir(parents=True, exist_ok=True)
            csv_file = out_dir / "joint_path.csv"

            with open(csv_file, "w", newline="") as fp:
                writer = csv.writer(fp)
                header = [f"J{i+1}_rad" for i in range(len(final_joint_trajectory[0]))]
                writer.writerow(header)
                writer.writerows(final_joint_trajectory)
            print(f"[Pi Plan HF] Smoothed joint path CSV saved -> {csv_file}")

            # Auto-generate comparison plots using the existing utility
            try:
                from diagnostics import plot_ik_plan as plot_util

                ik_csv = out_dir / "ik_plan.csv"
                csv_to_plot = ik_csv if ik_csv.exists() else csv_file  # Fall back to smoothed path only

                plot_util.main(csv_to_plot)
            except Exception as e:
                print(f"[Pi Plan HF] WARNING: Failed to auto-generate plots: {e}")
        except Exception as e:
            print(f"[Pi Plan HF] WARNING: Failed to write joint_path.csv: {e}")

    planner_diag["reason_code"] = "OK"
    planner_diag["residuals"] = selected_residuals
    planner_diag["attempt"] = selected_attempt
    _set_planner_diagnostics(planner_diag)
    try:
        if utils.get_motion_state() == "PLANNING":
            utils.set_motion_state("IDLE")
    except Exception:
        pass
    return final_joint_trajectory


def _derive_program_terminal_outcome(exc: BaseException | None = None) -> tuple[str, str]:
    if bool(utils.trajectory_state_get("should_stop", False)):
        reason = str(utils.trajectory_state_get("stop_request_reason", None) or "operator_abort").strip().lower()
        return "aborted", reason or "operator_abort"

    backend = _get_backend()
    getter = getattr(backend, "get_execution_status", None) if backend is not None else None
    if callable(getter):
        try:
            execution_status = getter()
            state_name = str(getattr(execution_status, "state_name", "")).strip().lower()
        except Exception:
            state_name = ""
        if state_name in {"faulted", "underrun"}:
            return "faulted", "rtcore_fault"
        if state_name == "aborted":
            return "aborted", str(
                utils.trajectory_state_get("stop_request_reason", None) or "compatibility_interruption"
            ).strip().lower() or "compatibility_interruption"

    message = str(exc).strip().lower() if exc is not None else ""
    if "timeout" in message or "timed out" in message:
        return "faulted", "timeout"
    return "faulted", "compatibility_interruption"


def _trajectory_executor_thread(planned_steps: list[dict], should_loop: bool):
    """
    The target function for the trajectory execution thread. This function
    iterates through a list of pre-planned steps (moves, pauses, etc.) and
    can be stopped gracefully via the global `trajectory_state` flag.

    Args:
        planned_steps: A list of dictionaries, where each dict describes a step
                       (e.g., {'type': 'move', 'path': [...]}).
        should_loop (bool): Whether to repeat the entire sequence upon completion.
    """
    terminal_state = "completed"
    terminal_reason = "completed"
    failing_step_index: int | None = None
    loop_iteration = 0
    completed_step_count = int(utils.program_status_snapshot().get("completed_step_count", 0) or 0)
    completed_loop_count = int(utils.program_status_snapshot().get("completed_loop_count", 0) or 0)

    try:
        utils.trajectory_state_set("weld_active", False)
        utils.program_status_update(
            active=True,
            state="executing",
            terminal_reason=None,
            failing_step_index=None,
            current_step_index=None,
            current_step_type=None,
            loop_iteration=0,
        )
        execution_loop_active = True
        while execution_loop_active and not bool(utils.trajectory_state_get("should_stop", False)):
            utils.trajectory_state_update(active_program_loop_iteration=loop_iteration)
            utils.program_status_update(
                active=True,
                state="executing",
                current_step_index=None,
                current_step_type=None,
                loop_iteration=loop_iteration,
            )
            for i, step in enumerate(planned_steps):
                if bool(utils.trajectory_state_get("should_stop", False)):
                    print("[Pi Trajectory] Stop detected, halting execution.")
                    terminal_state, terminal_reason = _derive_program_terminal_outcome()
                    failing_step_index = i
                    execution_loop_active = False
                    break

                step_type = str(step.get("type", ""))
                print(f"[Pi Execute] Executing Step {i+1}/{len(planned_steps)} ({step_type})...")
                utils.trajectory_state_update(
                    active_program_step_index=i,
                    active_program_step_type=step_type,
                )
                utils.program_status_update(
                    active=True,
                    state="executing",
                    terminal_reason=None,
                    failing_step_index=None,
                    current_step_index=i,
                    current_step_type=step_type,
                    loop_iteration=loop_iteration,
                )
                utils.trajectory_state_set("weld_active", bool(step.get("weld_active", False)))
                if step_type == "move":
                    _execute_joint_path(step["path"], step["freq"])
                elif step_type == "joint_move":
                    print(f"[Pi Execute] Moving joints to target configuration and waiting {step['duration']}s.")
                    servo_driver.set_servo_positions(step["target_q"], step["speed"], 0)
                    utils.current_logical_joint_angles_rad = step["target_q"]
                    end_time = time.monotonic() + step["duration"]
                    while not bool(utils.trajectory_state_get("should_stop", False)) and time.monotonic() < end_time:
                        time.sleep(0.01)
                elif step_type == "pause":
                    utils.trajectory_state_set("weld_active", False)
                    print(f"[Pi Execute] Pausing for {step['duration']} seconds.")
                    end_time = time.monotonic() + step["duration"]
                    while not bool(utils.trajectory_state_get("should_stop", False)) and time.monotonic() < end_time:
                        time.sleep(0.01)

                if bool(utils.trajectory_state_get("should_stop", False)):
                    print("[Pi Trajectory] Stop detected during step execution.")
                    terminal_state, terminal_reason = _derive_program_terminal_outcome()
                    failing_step_index = i
                    execution_loop_active = False
                    break

                completed_step_count += 1
                utils.program_status_update(completed_step_count=completed_step_count)

            if not execution_loop_active:
                break

            if not should_loop:
                execution_loop_active = False
            elif not bool(utils.trajectory_state_get("should_stop", False)):
                completed_loop_count += 1
                utils.program_status_update(
                    completed_loop_count=completed_loop_count,
                    current_step_index=None,
                    current_step_type=None,
                    loop_iteration=loop_iteration + 1,
                )
                print("\n[Pi Trajectory] Loop enabled. Restarting sequence...")
                loop_iteration += 1
                time.sleep(1)

        if bool(utils.trajectory_state_get("should_stop", False)) and terminal_reason == "completed":
            terminal_state, terminal_reason = _derive_program_terminal_outcome()
            current_step_index = utils.program_status_snapshot().get("current_step_index")
            if current_step_index is not None:
                try:
                    failing_step_index = int(current_step_index)
                except Exception:
                    failing_step_index = failing_step_index

    except Exception as exc:
        terminal_state, terminal_reason = _derive_program_terminal_outcome(exc)
        current_step_index = utils.program_status_snapshot().get("current_step_index")
        if current_step_index is not None:
            try:
                failing_step_index = int(current_step_index)
            except Exception:
                pass
        print(f"[Pi Trajectory] ERROR: Executor thread failed: {exc}")

    finally:
        print("[Pi Trajectory] Executor thread finished.")
        utils.program_status_update(
            active=False,
            state=terminal_state,
            terminal_reason=terminal_reason,
            failing_step_index=failing_step_index,
            completed_step_count=completed_step_count,
            completed_loop_count=completed_loop_count,
            current_step_index=None,
            current_step_type=None,
            loop_iteration=loop_iteration,
        )
        # Clean up global state, but DO NOT reset the should_stop flag.
        # The stop flag should persist until a new motion command clears it.
        utils.trajectory_state_update(
            weld_active=False,
            current_weld_type=None,
            is_running=False,
            thread=None,
            active_program_name=None,
            active_program_loop_enabled=False,
            active_program_use_cache=False,
            active_program_step_count=0,
            active_program_move_steps=0,
            active_program_pause_steps=0,
            active_program_joint_move_steps=0,
            active_program_rtcore_segments=False,
            active_program_segment_execution_policy="",
            active_program_step_index=None,
            active_program_step_type=None,
            active_program_loop_iteration=0,
        )
        utils.trajectory_state_set("stop_request_reason", None)
        try:
            utils.set_motion_state("IDLE")
        except Exception:
            pass


def _open_loop_executor_thread(
    joint_path: list[list[float]],
    frequency: int,
    diagnostics: bool = True,
    return_telemetry: bool = False,
    owns_trajectory_state: bool = True,
):
    """High-speed *open-loop* executor.

    For generic backends, this pre-computes batch writes and emits them from
    Python at the requested cadence. For the EtherCAT RTCore backend, this now
    offloads the full joint path to RTCore as a queued trajectory so Python is
    no longer the sample clock for scheduled motion.
    """

    # Honor global diagnostics toggle as well as explicit argument
    diagnostics_enabled = diagnostics or utils.trajectory_state.get("diagnostics_enabled", False)

    if diagnostics_enabled and frequency > 400:
        print(f"[Pi OL] WARNING: Diagnostics enabled. Capping frequency from {frequency} Hz to 400 Hz due to feedback overhead.")
        frequency = 400

    n_steps = len(joint_path)
    print(f"[Pi OL] Starting Open-Loop Executor at {frequency} Hz ({n_steps} steps).")

    # ----------------------------------------------
    # Pre-allocate Sync-Write command buffers
    # ----------------------------------------------
    backend = _get_backend()
    use_backend = backend is not None and backend.is_initialized
    try:
        executor_speed = max(0, int(utils.ENCODER_RESOLUTION))
    except Exception:
        executor_speed = 4095

    if use_backend and hasattr(backend, "execute_joint_trajectory"):
        telemetry_result = None
        offload_t0 = time.perf_counter()
        try:
            status = backend.execute_joint_trajectory(joint_path, frequency)  # type: ignore[attr-defined]
            state_name = str(getattr(status, "state_name", "unknown"))
            if state_name not in ("completed", "idle"):
                raise RuntimeError(f"RTCore trajectory execution ended in state '{state_name}'")
            utils.current_logical_joint_angles_rad = joint_path[-1]
            elapsed_s = time.perf_counter() - offload_t0
            print(
                "[Pi OL] RTCore trajectory execution finished:"
                f" state={state_name}"
                f" traj_id={getattr(status, 'active_traj_id', 0)}"
                f" elapsed={elapsed_s:.3f}s"
            )
            if return_telemetry:
                telemetry_result = {
                    "frequency": frequency,
                    "backend_mode": "rtcore_trajectory",
                    "execution_state": getattr(status, "state_name", "unknown"),
                    "trajectory_id": int(getattr(status, "active_traj_id", 0)),
                    "elapsed_s": float(elapsed_s),
                }
        finally:
            if owns_trajectory_state and utils.trajectory_state.get("thread") is threading.current_thread():
                utils.trajectory_state.update({"is_running": False, "should_stop": False, "thread": None})
                utils.trajectory_state.pop('diagnostics_session_id', None)
                utils.trajectory_state.pop('diagnostics_folder_type', None)
                try:
                    utils.set_motion_state("IDLE")
                except Exception:
                    pass
            print("[Pi OL] Open-Loop Executor finished.")
        if return_telemetry:
            return telemetry_result
        return

    if use_backend:
        precomputed_cmds: list[list[tuple]] = [
            backend.prepare_sync_write_commands(list(q), speed=executor_speed, accel=0)
            for q in joint_path
        ]
    else:
        precomputed_cmds = [
            servo_driver.logical_q_to_syncwrite_tuple(q, utils.ENCODER_RESOLUTION, 0) for q in joint_path
        ]

    # ----------------------------------------------
    #  Timing buffers
    # ----------------------------------------------
    _loop_durations: list[float] = []
    _write_durations: list[float] = []
    _abs_errors_per_joint: list[list[float]] = [[] for _ in range(utils.NUM_LOGICAL_JOINTS)]
    # Angle telemetry (target vs actual) for plotting
    _actual_angles_per_joint: list[list[float]] = [[] for _ in range(utils.NUM_LOGICAL_JOINTS)]
    _target_angles_per_joint: list[list[float]] = [[] for _ in range(utils.NUM_LOGICAL_JOINTS)]

    time_step = 1.0 / frequency
    start_time = time.monotonic()

    try:
        for i, cmd in enumerate(precomputed_cmds):
            deadline = start_time + (i + 1) * time_step

            loop_start = time.perf_counter()

            # --- Actuation (WRITE) ---
            w_t0 = time.perf_counter()
            if use_backend:
                backend.sync_write(cmd)
            else:
                servo_protocol.sync_write_goal_pos_speed_accel(cmd)
            _write_durations.append(time.perf_counter() - w_t0)

            # --- Timing / sleep ---
            iter_elapsed = time.perf_counter() - loop_start
            _loop_durations.append(iter_elapsed)

            sleep_t = deadline - time.monotonic()
            if sleep_t > 0:
                time.sleep(sleep_t)
            # No per-cycle printouts; we'll summarise at the end.

            if diagnostics_enabled:
                # For diagnostics, read back the position to calculate tracking error.
                # This adds overhead and is NOT part of a true open-loop system.
                if use_backend:
                    actual_q = backend.get_joint_positions(verbose=False)
                else:
                    actual_q = servo_driver.get_current_arm_state_rad(verbose=False)
                target_q = joint_path[i]
                for j_idx in range(utils.NUM_LOGICAL_JOINTS):
                    error = target_q[j_idx] - actual_q[j_idx]
                    _abs_errors_per_joint[j_idx].append(abs(error))
                    _target_angles_per_joint[j_idx].append(target_q[j_idx])
                    _actual_angles_per_joint[j_idx].append(actual_q[j_idx])

            if bool(utils.trajectory_state_get("should_stop", False)):
                print("[Pi OL] Stop signal received, halting execution.")
                break

        # Update global logical joint state
        utils.current_logical_joint_angles_rad = joint_path[min(i, n_steps-1)]

    finally:
        # ----------------------------
        #  Statistics & Diagnostics
        # ----------------------------
        if _loop_durations:
            import statistics

            avg_ms = statistics.mean(_loop_durations) * 1000.0
            max_ms = max(_loop_durations) * 1000.0
            write_avg = statistics.mean(_write_durations) * 1000.0
            write_max = max(_write_durations) * 1000.0

            overruns = [d for d in _loop_durations if d > time_step]
            overrun_pct = len(overruns) / len(_loop_durations) * 100.0

            print(
                f"[Pi OL] Timing summary: avg {avg_ms:.2f} ms (max {max_ms:.2f}), "
                f"write avg {write_avg:.2f} (max {write_max:.2f}), "
                f"overruns {len(overruns)}/{len(_loop_durations)} ({overrun_pct:.1f} %)"
            )

            if diagnostics_enabled:
                session_id = utils.trajectory_state.get('diagnostics_session_id')
                _save_executor_diagnostics_charts(
                    mode="open_loop",
                    session_id=session_id,
                    time_step=time_step,
                    loop_durations=_loop_durations,
                    write_durations=_write_durations,
                    abs_errors_per_joint=_abs_errors_per_joint,
                    target_angles_per_joint=_target_angles_per_joint,
                    actual_angles_per_joint=_actual_angles_per_joint,
                )

        telemetry_result = None
        if return_telemetry:
            # Build telemetry dict in radians
            telemetry_result = {
                "target_angles_per_joint": _target_angles_per_joint,
                "actual_angles_per_joint": _actual_angles_per_joint,
                "abs_errors_per_joint": _abs_errors_per_joint,
                "loop_durations": _loop_durations,
                "write_durations": _write_durations,
                "frequency": frequency,
            }

        # Only clear global run-state when this executor owns lifecycle management.
        # Nested calls from the higher-level trajectory step executor must not
        # toggle is_running/thread between weld sub-steps.
        if owns_trajectory_state and utils.trajectory_state.get("thread") is threading.current_thread():
            utils.trajectory_state.update({"is_running": False, "should_stop": False, "thread": None})
            # Clean up session keys
            utils.trajectory_state.pop('diagnostics_session_id', None)
            utils.trajectory_state.pop('diagnostics_folder_type', None)
            try:
                utils.set_motion_state("IDLE")
            except Exception:
                pass
        print("[Pi OL] Open-Loop Executor finished.")

        if return_telemetry:
            return telemetry_result


def _closed_loop_executor_thread(
    joint_path: list[list[float]],
    frequency: int,
    diagnostics: bool = True,
    return_telemetry: bool = False,
    owns_trajectory_state: bool = True,
):
    """
    Executes a pre-planned joint-space trajectory using a real-time, closed-loop
    proportional controller to ensure path accuracy. This is the primary executor
    for high-precision moves.

    Args:
        joint_path: The dense list of target joint angle configurations.
        frequency: The target execution frequency for the control loop (default 50 Hz).
        diagnostics: Whether to generate and save timing and error charts.
    """
    # Honor global diagnostics toggle
    diagnostics_enabled = diagnostics or utils.trajectory_state.get("diagnostics_enabled", False)
    print(f"[Pi CLC] Starting Closed-Loop Executor at {frequency} Hz for a path with {len(joint_path)} steps.")
    
    try:
        time_step = 1.0 / frequency
        start_time = time.monotonic()
        
        # -----------------------------
        #   Telemetry Buffers
        # -----------------------------
        _loop_durations: list[float] = []          # total cycle duration
        _read_durations: list[float] = []          # SYNC-READ latency
        _compute_durations: list[float] = []       # control law & conversions
        _write_durations: list[float] = []         # SYNC-WRITE latency

        # Per-joint error accumulators (abs radians per cycle)
        _abs_errors_per_joint: list[list[float]] = [[] for _ in range(utils.NUM_LOGICAL_JOINTS)]
        # Angle telemetry buffers (radians)
        _target_angles_per_joint: list[list[float]] = [[] for _ in range(utils.NUM_LOGICAL_JOINTS)]
        _actual_angles_per_joint: list[list[float]] = [[] for _ in range(utils.NUM_LOGICAL_JOINTS)]

        backend = _get_backend()
        use_backend = backend is not None and backend.is_initialized
        logical_to_physical_map: dict[int, list[int]] = {}
        PRIMARY_FB_IDS: list[int] = []
        twin_motor_pairs: list[tuple[int, int]] = []
        if not use_backend:
            # Legacy servo backends still use physical-servo feedback and sync-write tuples.
            logical_to_physical_map = _build_logical_to_physical_index_map()
            PRIMARY_FB_IDS = _build_primary_feedback_ids()
            twin_motor_pairs = _get_twin_motor_pairs()

        try:
            executor_speed = max(0, int(utils.ENCODER_RESOLUTION))
        except Exception:
            executor_speed = 4095

        # Integral accumulators for steady-state error compensation (gravity, bias)
        integral_error_rad = [0.0 for _ in range(utils.NUM_LOGICAL_JOINTS)]

        for i, target_q_step in enumerate(joint_path):
            deadline = start_time + (i + 1) * time_step
            
            # Capture start for this iteration
            iter_start = time.perf_counter()

            # --- Stop Check ---
            if bool(utils.trajectory_state_get("should_stop", False)):
                print("[Pi CLC] Stop signal received, halting execution.")
                break

            # --- Feedback (Read) ---
            read_t0 = time.perf_counter()

            # Use a fixed but small timeout to ensure full packets arrive; setting
            # too low leads to intermittent Sync Read failures.
            per_cycle_timeout = max(0.01, time_step * 0.8)
            if use_backend:
                raw_positions = backend.sync_read_positions(timeout_s=per_cycle_timeout)
            else:
                raw_positions = servo_protocol.sync_read_positions(
                    PRIMARY_FB_IDS,
                    timeout_s=per_cycle_timeout,
                    poll_delay_s=0.0,
                )
            _read_durations.append(time.perf_counter() - read_t0)

            actual_joint_positions: list[float] = []
            if use_backend:
                try:
                    actual_joint_positions = list(backend.raw_to_joint_positions(raw_positions))
                except Exception as exc:
                    print(f"[Pi CLC] WARNING: Backend joint feedback conversion failed: {exc}")
                    try:
                        actual_joint_positions = list(backend.get_joint_positions(verbose=False))
                    except Exception as fallback_exc:
                        print(f"[Pi CLC] WARNING: Backend joint feedback fallback failed: {fallback_exc}")
                        actual_joint_positions = []

            # ------------------------------------------------------------
            #  Feedback synthesis for twin-motor joints
            #  -----------------------------------------------------------
            #  We only query one servo per twin-motor joint (20 & 30) to
            #  save wire time.  The partner motors (21 & 31) rotate in the
            #  opposite RAW direction, so we re-create a plausible raw
            #  reading for them by:
            #    1. converting the received RAW value → physical angle
            #    2. converting that angle back → RAW using the partner's
            #       mapping rules (direct vs inverted)
            #  This keeps the downstream control law unchanged.

            def _angle_to_raw(angle_rad: float, physical_idx: int) -> int:
                """Convert a physical angle into a raw servo value using the
                mapping rules held in utils.  Clamp to valid range."""
                min_map_rad, max_map_rad = utils.EFFECTIVE_MAPPING_RANGES[physical_idx]
                angle_clamped = max(min_map_rad, min(max_map_rad, angle_rad))
                norm_val = (angle_clamped - min_map_rad) / (max_map_rad - min_map_rad)
                encoder_max = utils.ENCODER_RESOLUTION

                if utils._is_servo_direct_mapping(physical_idx):
                    raw = norm_val * encoder_max
                else:
                    raw = (1.0 - norm_val) * encoder_max

                return int(round(max(0, min(encoder_max, raw))))

            # --- Control Law (Calculate Error and Correction) ---
            commands_for_sync_write = []
            
            compute_t0 = time.perf_counter()

            if use_backend:
                for logical_joint_index, target_angle_rad in enumerate(target_q_step):
                    _target_angles_per_joint[logical_joint_index].append(target_angle_rad)

                    fallback_angle = (
                        _actual_angles_per_joint[logical_joint_index][-1]
                        if _actual_angles_per_joint[logical_joint_index]
                        else 0.0
                    )
                    try:
                        actual_angle = float(actual_joint_positions[logical_joint_index])
                        if not math.isfinite(actual_angle):
                            actual_angle = fallback_angle
                    except Exception:
                        actual_angle = fallback_angle

                    _actual_angles_per_joint[logical_joint_index].append(actual_angle)

                    error_rad = target_angle_rad - actual_angle
                    _abs_errors_per_joint[logical_joint_index].append(abs(error_rad))
                    integral_error_rad[logical_joint_index] += error_rad * time_step
                    if integral_error_rad[logical_joint_index] > utils.CORRECTION_INTEGRAL_CLAMP_RAD:
                        integral_error_rad[logical_joint_index] = utils.CORRECTION_INTEGRAL_CLAMP_RAD
                    elif integral_error_rad[logical_joint_index] < -utils.CORRECTION_INTEGRAL_CLAMP_RAD:
                        integral_error_rad[logical_joint_index] = -utils.CORRECTION_INTEGRAL_CLAMP_RAD

                commands_for_sync_write = backend.prepare_sync_write_commands(
                    list(target_q_step),
                    speed=executor_speed,
                    accel=0,
                )
            else:
                # --- Twin motor mirroring (dynamically from robot config) ---
                for primary_id, secondary_id in twin_motor_pairs:
                    if primary_id in raw_positions and secondary_id not in raw_positions:
                        primary_idx = utils.SERVO_IDS.index(primary_id)
                        secondary_idx = utils.SERVO_IDS.index(secondary_id)
                        angle_rad = servo_driver.servo_value_to_radians(raw_positions[primary_id], primary_idx)
                        raw_positions[secondary_id] = _angle_to_raw(angle_rad, secondary_idx)

                # Record target vs actual angles per logical joint using primary IDs
                try:
                    # Build primary_ids dict from the PRIMARY_FB_IDS list
                    primary_ids = {i: PRIMARY_FB_IDS[i] for i in range(len(PRIMARY_FB_IDS))}
                    for logical_joint_index in range(utils.NUM_LOGICAL_JOINTS):
                        target_angle_rad = target_q_step[logical_joint_index]
                        _target_angles_per_joint[logical_joint_index].append(target_angle_rad)
                        primary_id = primary_ids.get(logical_joint_index)
                        if primary_id and primary_id in raw_positions:
                            config_index = utils.SERVO_IDS.index(primary_id)
                            actual_angle = servo_driver.servo_value_to_radians(raw_positions[primary_id], config_index)
                            # Undo master offset to get logical angle
                            actual_angle -= utils.LOGICAL_JOINT_MASTER_OFFSETS_RAD[logical_joint_index]
                            _actual_angles_per_joint[logical_joint_index].append(actual_angle)
                        else:
                            # If missing, repeat last or 0.0
                            last = _actual_angles_per_joint[logical_joint_index][-1] if _actual_angles_per_joint[logical_joint_index] else 0.0
                            _actual_angles_per_joint[logical_joint_index].append(last)
                except Exception:
                    pass

                for logical_joint_index, target_angle_rad in enumerate(target_q_step):
                    # This logic mirrors set_servo_positions to find the target physical angle.
                    angle_with_master_offset = target_angle_rad + utils.LOGICAL_JOINT_MASTER_OFFSETS_RAD[logical_joint_index]
                    target_physical_angle_rad = angle_with_master_offset

                    # NOTE: Previous versions applied a *2 scaling here to compensate for a
                    # 2:1 belt gear ratio on the J1 (base) joint. The current hardware is
                    # direct-drive (1:1), so this extra scaling would cause the base to
                    # rotate twice as far as commanded during closed-loop execution,
                    # leading to exaggerated Y-axis motion.  The scaling has therefore
                    # been removed.

                    # For each physical servo associated with this logical joint...
                    for physical_servo_config_index in logical_to_physical_map[logical_joint_index]:
                        servo_id = utils.SERVO_IDS[physical_servo_config_index]
                        
                        # 1. Get the actual angle of this specific servo from the feedback data
                        actual_raw_pos = raw_positions.get(servo_id)
                        if actual_raw_pos is None:
                            print(f"[Pi CLC] WARNING: Missing feedback for servo {servo_id}. Skipping correction.")
                            continue # Skip this servo if feedback failed
                        
                        actual_physical_angle_rad = servo_driver.servo_value_to_radians(actual_raw_pos, physical_servo_config_index)

                        # 2. Calculate the error
                        error_rad = target_physical_angle_rad - actual_physical_angle_rad
                        
                        # Collect telemetry: absolute error per logical joint
                        _abs_errors_per_joint[logical_joint_index].append(abs(error_rad))
                        
                        # 3. Calculate the corrected command
                        # Update integral term (anti-windup clamped)
                        integral_error_rad[logical_joint_index] += error_rad * time_step
                        if integral_error_rad[logical_joint_index] > utils.CORRECTION_INTEGRAL_CLAMP_RAD:
                            integral_error_rad[logical_joint_index] = utils.CORRECTION_INTEGRAL_CLAMP_RAD
                        elif integral_error_rad[logical_joint_index] < -utils.CORRECTION_INTEGRAL_CLAMP_RAD:
                            integral_error_rad[logical_joint_index] = -utils.CORRECTION_INTEGRAL_CLAMP_RAD

                        # Commanded Angle = Target + Kp * Error + Ki * ∫Error dt
                        # Software PID correction intentionally disabled. We command the planned target
                        # to let inner servo PID be the only stabilizing loop during tuning.
                        commanded_physical_angle_rad = target_physical_angle_rad
                        
                        # 4. Convert the final commanded angle to a raw servo value
                        # This block is the same as in set_servo_positions
                        min_urdf_rad, max_urdf_rad = utils.URDF_JOINT_LIMITS[physical_servo_config_index]
                        angle_clamped_to_urdf = max(min_urdf_rad, min(max_urdf_rad, commanded_physical_angle_rad))
                        
                        min_map_rad, max_map_rad = utils.EFFECTIVE_MAPPING_RANGES[physical_servo_config_index]
                        angle_for_norm = max(min_map_rad, min(max_map_rad, angle_clamped_to_urdf))
                        normalized_value = (angle_for_norm - min_map_rad) / (max_map_rad - min_map_rad)
                        encoder_max = utils.ENCODER_RESOLUTION
                        
                        raw_servo_value = (normalized_value * encoder_max) if utils._is_servo_direct_mapping(physical_servo_config_index) else ((1.0 - normalized_value) * encoder_max)
                        
                        final_servo_pos_value = int(round(raw_servo_value))
                        
                        # Add to the command list. Speed is max and Accel is none (0)
                        # because the path is timed by the closed-loop executor itself.
                        commands_for_sync_write.append((servo_id, final_servo_pos_value, encoder_max, 0))

            _compute_durations.append(time.perf_counter() - compute_t0)

            # --- Actuation (Write) ---
            if commands_for_sync_write:
                write_t0 = time.perf_counter()
                if use_backend:
                    backend.sync_write(commands_for_sync_write)
                else:
                    servo_protocol.sync_write_goal_pos_speed_accel(commands_for_sync_write)
                _write_durations.append(time.perf_counter() - write_t0)
            else:
                _write_durations.append(0.0)

            # --- Timing ---
            iter_elapsed = time.perf_counter() - iter_start  # Actual compute + IO time for this cycle
            _loop_durations.append(iter_elapsed)

            sleep_time = deadline - time.monotonic()
            if sleep_time > 0:
                time.sleep(sleep_time)
            else:
                overrun_ms = -sleep_time * 1000
                if overrun_ms > 1.0: # Only warn for significant overruns
                    print(f"[Pi CLC] WARNING: Loop overrun by {overrun_ms:.2f} ms at step {i}.")

        # After the loop, update the global state to the last commanded position
        utils.current_logical_joint_angles_rad = joint_path[-1]

    finally:
        print("[Pi CLC] Closed-Loop Executor thread finished a segment.")

        # Print timing statistics if we collected any samples
        if '_loop_durations' in locals() and _loop_durations:
            fmt = lambda xs: (statistics.mean(xs) * 1000.0, max(xs) * 1000.0)

            avg_ms, max_ms = fmt(_loop_durations)
            read_avg, read_max = fmt(_read_durations)
            comp_avg, comp_max = fmt(_compute_durations)
            write_avg, write_max = fmt(_write_durations)

            overruns = [d for d in _loop_durations if d > time_step]
            overrun_pct = (len(overruns) / len(_loop_durations)) * 100.0

            print(
                f"[Pi CLC] Timing summary: total avg={avg_ms:.2f} ms (max {max_ms:.2f}), "
                f"read avg={read_avg:.2f} (max {read_max:.2f}), "
                f"compute avg={comp_avg:.2f} (max {comp_max:.2f}), "
                f"write avg={write_avg:.2f} (max {write_max:.2f}), "
                f"overruns {len(overruns)}/{len(_loop_durations)} ({overrun_pct:.1f}%)"
            )

            # --- Error statistics ---
            joint_stats = []
            for j_idx, errs in enumerate(_abs_errors_per_joint):
                if errs:
                    mean_err = statistics.mean(errs)
                    max_err = max(errs)
                    joint_stats.append(f"J{j_idx+1}: mean {math.degrees(mean_err):.2f}°, max {math.degrees(max_err):.2f}°")
            if joint_stats:
                print("[Pi CLC] Tracking error summary → " + "; ".join(joint_stats))

            # ------------------
            # Optional Charts
            # ------------------
            if diagnostics_enabled:
                session_id = utils.trajectory_state.get('diagnostics_session_id')
                # Get sync profiles from backend if available, else from servo_protocol
                backend = _get_backend()
                if backend and _use_backend() and hasattr(backend, 'get_sync_profiles'):
                    sync_profiles = backend.get_sync_profiles()
                else:
                    sync_profiles = servo_protocol.get_sync_profiles()
                _save_executor_diagnostics_charts(
                    mode="closed_loop",
                    session_id=session_id,
                    time_step=time_step,
                    loop_durations=_loop_durations,
                    write_durations=_write_durations,
                    abs_errors_per_joint=_abs_errors_per_joint,
                    target_angles_per_joint=_target_angles_per_joint,
                    actual_angles_per_joint=_actual_angles_per_joint,
                    read_durations=_read_durations,
                    compute_durations=_compute_durations,
                    sync_profiles=sync_profiles,
                )

        # If this executor thread is the one registered in trajectory_state, clear it
        telemetry_result = None
        if return_telemetry:
            telemetry_result = {
                "target_angles_per_joint": _target_angles_per_joint if '_target_angles_per_joint' in locals() else None,
                "actual_angles_per_joint": _actual_angles_per_joint if '_actual_angles_per_joint' in locals() else None,
                "abs_errors_per_joint": _abs_errors_per_joint if '_abs_errors_per_joint' in locals() else None,
                "loop_durations": _loop_durations if '_loop_durations' in locals() else None,
                "read_durations": _read_durations if '_read_durations' in locals() else None,
                "compute_durations": _compute_durations if '_compute_durations' in locals() else None,
                "write_durations": _write_durations if '_write_durations' in locals() else None,
                "frequency": frequency,
            }

        if owns_trajectory_state and utils.trajectory_state.get("thread") is threading.current_thread():
            utils.trajectory_state.update({"is_running": False, "should_stop": False, "thread": None})
            utils.trajectory_state.pop('diagnostics_session_id', None)
            utils.trajectory_state.pop('diagnostics_folder_type', None)
            try:
                utils.set_motion_state("IDLE")
            except Exception:
                pass

        if return_telemetry:
            return telemetry_result


def _execute_joint_path(joint_path: list[list[float]], frequency: int):
    """Executes a pre-computed joint path synchronously (blocking).

    Current implementation chooses the **open-loop** executor for speed.
    It runs in the *current* thread so the caller remains blocking, which
    is appropriate for the step-by-step `_trajectory_executor_thread`.

    Parameters
    ----------
    joint_path : list[list[float]]
        Dense list of joint configurations.
    frequency : int
        Execution frequency in Hz.
    """
    diagnostics_enabled = utils.trajectory_state.get("diagnostics_enabled", False)
    _open_loop_executor_thread(
        joint_path,
        frequency,
        diagnostics=diagnostics_enabled,
        owns_trajectory_state=False,
    )

