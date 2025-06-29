# Contains the logic for planning and executing timed trajectories,
# including the high-fidelity planner and the closed-loop executor thread. 
import time
import math
import statistics
import datetime
import numpy as np
from scipy.signal import savgol_filter
from typing import Sequence, Union
from pathlib import Path
import os
import csv

try:
    import ik_solver
    import trajectory_planner
except ImportError:
    print("ERROR: Missing 'ik_solver' or 'trajectory_planner'. Ensure they are in the python path.")
    ik_solver = None
    trajectory_planner = None

from . import utils
from . import servo_driver
from . import servo_protocol

JointTraj = Union[Sequence[Sequence[float]], np.ndarray]

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


def _plan_linear_move(start_q: list[float], target_pos: np.ndarray, velocity: float, acceleration: float, frequency: int, use_smoothing: bool, forced_orientation: np.ndarray =None) -> list | None:
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

    Returns:
        A list of lists representing the dense joint-space trajectory, or None on failure.
    """
    # 1. Get start pose from the provided start_q for path generation
    start_pos_from_q = ik_solver.get_fk(start_q)
    if start_pos_from_q is None:
        print("[Pi Plan] ERROR: Cannot start planning, failed to get start position from initial_q.")
        return None

    # 2. Generate the ideal linear Cartesian trajectory
    ideal_cartesian_points = trajectory_planner.generate_trapezoidal_profile(
        start_pos_from_q, target_pos, velocity, acceleration, frequency
    )
    if not ideal_cartesian_points:
        print("[Pi Plan] ERROR: Trajectory planner failed to generate a path.")
        return None

    # 3. Plan the joint-space path from the Cartesian points using the new high-fidelity planner
    final_joint_trajectory = _plan_high_fidelity_trajectory(
        cartesian_points=ideal_cartesian_points, 
        start_q=start_q, 
        use_smoothing=use_smoothing,
        forced_orientation=forced_orientation
    )

    return final_joint_trajectory


def _plan_high_fidelity_trajectory(cartesian_points: list, start_q: list[float], use_smoothing: bool = True, forced_orientation: np.ndarray = None) -> list | None:
    """
    Takes a list of Cartesian points and plans a complete, smoothed joint-space trajectory.
    This is the core planning function, which solves IK for every point on the path.

    Args:
        cartesian_points: The list of [x,y,z] points for the tool tip.
        start_q: The starting joint angles, used for orientation lock and as the initial seed.
        use_smoothing: Whether to apply a Savitzky-Golay filter to the final path.
        forced_orientation: A 3x3 rotation matrix to lock the orientation to. 
                              If None, orientation is locked to the start pose.

    Returns:
        The final, dense list of joint angle solutions, or None on failure.
    """
    print(f"[Pi Plan HF] Planning high-fidelity trajectory for {len(cartesian_points)} points.")
    
    # 1. Determine target orientation for the entire path.
    target_orientation = None
    if forced_orientation is not None:
        target_orientation = forced_orientation
    else:
        # Fallback to locking orientation to the start of the move.
        initial_pose_matrix = ik_solver.get_fk_matrix(start_q)
        if initial_pose_matrix is None:
            print("[Pi Plan HF] ERROR: FK failed on start_q. Cannot determine orientation lock.")
            return None
        target_orientation = initial_pose_matrix[:3, :3]

    # Create a list of orientations for the batch solver. This is required by the current
    # `solve_ik_path_batch` function signature.
    orientations_list = [target_orientation] * len(cartesian_points)

    # 2. Solve IK for the entire path in one batch call for maximum performance.
    t_start_ik = time.monotonic()
    
    joint_trajectory = ik_solver.solve_ik_path_batch(
        path_points=cartesian_points,
        initial_joint_angles=start_q,
        target_orientations=orientations_list
    )
    
    t_end_ik = time.monotonic()

    if joint_trajectory is None:
        print("[Pi Plan HF] ERROR: Batch IK solver failed to find a solution for the path.")
        return None
    
    print(f"[Pi Plan HF] Batch IK solving complete. Took {(t_end_ik - t_start_ik) * 1000:.2f} ms")

    # 3. Post-process the trajectory (unwrap and smooth).
    unwrapped_joint_trajectory = _unwrap_joint_trajectory(joint_trajectory)
    
    if use_smoothing:
        # Using the same smoothing parameters as the old function for consistency.
        raw_joint_trajectory_np = np.array(unwrapped_joint_trajectory).T
        window_length = 15 
        polyorder = 3

        # The window for the filter must be odd and smaller than the number of points.
        if window_length > len(unwrapped_joint_trajectory):
            window_length = max(polyorder + 1, len(unwrapped_joint_trajectory) - 1)
            if window_length % 2 == 0: window_length -= 1
        
        if window_length > polyorder:
            smoothed_joint_trajectory_np = savgol_filter(
                raw_joint_trajectory_np, window_length, polyorder, axis=1
            )
            final_joint_trajectory = smoothed_joint_trajectory_np.T.tolist()
        else:
            # Not enough points to smooth, return the unwrapped path.
            final_joint_trajectory = unwrapped_joint_trajectory
    else:
        final_joint_trajectory = unwrapped_joint_trajectory
        
    print("[Pi Plan HF] Path post-processing complete.")

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

    return final_joint_trajectory


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
    try:
        execution_loop_active = True
        while execution_loop_active and not utils.trajectory_state["should_stop"]:
            for i, step in enumerate(planned_steps):
                # Before each step, check if a stop has been requested.
                if utils.trajectory_state["should_stop"]:
                    print("[Pi Trajectory] Stop detected, halting execution.")
                    execution_loop_active = False
                    break

                print(f"[Pi Execute] Executing Step {i+1}/{len(planned_steps)} ({step['type']})...")
                if step['type'] == 'move':
                    _execute_joint_path(step['path'], step['freq'])
                elif step['type'] == 'joint_move':
                    print(f"[Pi Execute] Moving joints to target configuration and waiting {step['duration']}s.")
                    servo_driver.set_servo_positions(step['target_q'], step['speed'], 0)
                    # Update global state immediately
                    utils.current_logical_joint_angles_rad = step['target_q']
                    time.sleep(step['duration'])
                elif step['type'] == 'pause':
                    print(f"[Pi Execute] Pausing for {step['duration']} seconds.")
                    time.sleep(step['duration'])
            
            if not should_loop:
                execution_loop_active = False
            elif not utils.trajectory_state["should_stop"]:
                print("\n[Pi Trajectory] Loop enabled. Restarting sequence...")
                time.sleep(1)

    finally:
        print("[Pi Trajectory] Executor thread finished.")
        # Clean up global state
        utils.trajectory_state["is_running"] = False
        utils.trajectory_state["should_stop"] = False
        utils.trajectory_state["thread"] = None


def _open_loop_executor_thread(
    joint_path: list[list[float]],
    frequency: int,
    diagnostics: bool = True,
):
    """High-speed *open-loop* executor.

    • Pre-computes the raw Sync-Write payload for every step so the realtime
      loop only needs to send the bytes.
    • Collects basic timing statistics analogous to the closed-loop executor
      (loop + write duration). Optionally saves a timing chart.
    """

    if diagnostics and frequency > 400:
        print(f"[Pi OL] WARNING: Diagnostics enabled. Capping frequency from {frequency} Hz to 400 Hz due to feedback overhead.")
        frequency = 400

    n_steps = len(joint_path)
    print(f"[Pi OL] Starting Open-Loop Executor at {frequency} Hz ({n_steps} steps).")

    # ----------------------------------------------
    # Pre-allocate Sync-Write command buffers
    # ----------------------------------------------
    precomputed_cmds: list[list[tuple[int,int,int,int]]] = [
        servo_driver.logical_q_to_syncwrite_tuple(q, 4095, 0) for q in joint_path
    ]

    # ----------------------------------------------
    #  Timing buffers
    # ----------------------------------------------
    _loop_durations: list[float] = []
    _write_durations: list[float] = []
    _abs_errors_per_joint: list[list[float]] = [[] for _ in range(utils.NUM_LOGICAL_JOINTS)]

    time_step = 1.0 / frequency
    start_time = time.monotonic()

    try:
        for i, cmd in enumerate(precomputed_cmds):
            deadline = start_time + (i + 1) * time_step

            loop_start = time.perf_counter()

            # --- Actuation (WRITE) ---
            w_t0 = time.perf_counter()
            servo_protocol.sync_write_goal_pos_speed_accel(cmd)
            _write_durations.append(time.perf_counter() - w_t0)

            # --- Timing / sleep ---
            iter_elapsed = time.perf_counter() - loop_start
            _loop_durations.append(iter_elapsed)

            sleep_t = deadline - time.monotonic()
            if sleep_t > 0:
                time.sleep(sleep_t)
            # No per-cycle printouts; we'll summarise at the end.

            if diagnostics:
                # For diagnostics, read back the position to calculate tracking error.
                # This adds overhead and is NOT part of a true open-loop system.
                actual_q = servo_driver.get_current_arm_state_rad(verbose=False)
                target_q = joint_path[i]
                for j_idx in range(utils.NUM_LOGICAL_JOINTS):
                    error = target_q[j_idx] - actual_q[j_idx]
                    _abs_errors_per_joint[j_idx].append(abs(error))

            if utils.trajectory_state["should_stop"]:
                print("[Pi OL] Stop signal received, halting execution.")
                break

        # Update global logical joint state
        utils.current_logical_joint_angles_rad = joint_path[min(i, n_steps-1)]

    finally:
        # ----------------------------
        #  Statistics & Diagnostics
        # ----------------------------
        if _loop_durations:
            import statistics, math, datetime, matplotlib
            matplotlib.use("Agg")
            import matplotlib.pyplot as plt

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

            if diagnostics:
                try:
                    session_id = utils.trajectory_state.get('diagnostics_session_id')
                    
                    if session_id:
                        # A session is active; save charts into the session folder.
                        out_dir = Path(f"diagnostics/open_loop/{session_id}")
                    else:
                        # No session; save with a unique timestamp in the parent folder.
                        session_id = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
                        out_dir = Path("diagnostics/open_loop")
                    out_dir.mkdir(exist_ok=True, parents=True)

                    # --- Joint Error Chart ---
                    fig, ax = plt.subplots(1, 1, figsize=(10, 4))
                    for j_idx, errs in enumerate(_abs_errors_per_joint):
                        if errs:
                            ax.plot([math.degrees(e) for e in errs], label=f"J{j_idx+1}")
                    ax.set_xlabel("Cycle index")
                    ax.set_ylabel("Abs error (deg)")
                    ax.set_title("Open-Loop Tracking Error per Joint")
                    ax.legend(ncol=3)
                    error_path = out_dir / "error.png"
                    fig.tight_layout()
                    fig.savefig(error_path)
                    plt.close(fig)

                    # --- Timing Chart ---
                    fig, ax = plt.subplots(1, 1, figsize=(10, 4))
                    ax.plot([d * 1000 for d in _loop_durations], label="total loop")
                    ax.plot([d * 1000 for d in _write_durations], label="write")
                    ax.axhline(time_step * 1000, color="red", linestyle="--", label="deadline")
                    ax.set_xlabel("Cycle index")
                    ax.set_ylabel("Duration (ms)")
                    ax.set_title("Open-loop cycle timing")
                    ax.legend()
                    chart_path = out_dir / "timing.png"
                    fig.tight_layout()
                    fig.savefig(chart_path)
                    plt.close(fig)

                    print(f"[Pi OL] Charts saved → {out_dir}")
                except Exception as e:
                    print(f"[Pi OL] WARNING: Failed to generate diagnostics chart: {e}")

        # Reset controller state flags
        utils.trajectory_state.update({"is_running": False, "should_stop": False, "thread": None})

        print("[Pi OL] Open-Loop Executor finished.")
        # Clean up session keys if they exist
        if 'diagnostics_session_id' in utils.trajectory_state:
            del utils.trajectory_state['diagnostics_session_id']
        if 'diagnostics_folder_type' in utils.trajectory_state:
            del utils.trajectory_state['diagnostics_folder_type']


def _closed_loop_executor_thread(
    joint_path: list[list[float]],
    frequency: int,
    diagnostics: bool = True,
):
    """
    Executes a pre-planned joint-space trajectory using a real-time, closed-loop
    proportional controller to ensure path accuracy. This is the primary executor
    for high-precision moves.

    Args:
        joint_path: The dense list of target joint angle configurations.
        frequency: The target execution frequency for the control loop (e.g., 200 Hz).
        diagnostics: Whether to generate and save timing and error charts.
    """
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

        # We need a mapping from logical joint index back to the physical servos to command.
        # This is the reverse of the logic in set_servo_positions.
        # This could be pre-calculated, but is clear here for now.
        logical_to_physical_map = {
            0: [0],     # Logical J1 -> Physical Servo ID 10
            1: [1, 2],  # Logical J2 -> Physical Servo IDs 20, 21
            2: [3, 4],  # Logical J3 -> Physical Servo IDs 30, 31
            3: [5],     # Logical J4 -> Physical Servo ID 40
            4: [6],     # Logical J5 -> Physical Servo ID 50
            5: [7]      # Logical J6 -> Physical Servo ID 60
        }
        all_physical_servo_ids = [utils.SERVO_IDS[i] for i in range(utils.NUM_PHYSICAL_SERVOS)]
        PRIMARY_FB_IDS = [10, 20, 30, 40, 50, 60]

        for i, target_q_step in enumerate(joint_path):
            deadline = start_time + (i + 1) * time_step
            
            # Capture start for this iteration
            iter_start = time.perf_counter()

            # --- Stop Check ---
            if utils.trajectory_state["should_stop"]:
                print("[Pi CLC] Stop signal received, halting execution.")
                break

            # --- Feedback (Read) ---
            read_t0 = time.perf_counter()

            # Use a fixed but small timeout to ensure full packets arrive; setting
            # too low leads to intermittent Sync Read failures.
            per_cycle_timeout = max(0.01, time_step * 0.8)
            raw_positions = servo_protocol.sync_read_positions(
                PRIMARY_FB_IDS,
                timeout_s=per_cycle_timeout,
                poll_delay_s=0.0,
            )
            _read_durations.append(time.perf_counter() - read_t0)

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
                """Convert a physical angle into a raw servo value (0-4095) using the
                mapping rules held in utils.  Clamp to valid range."""
                min_map_rad, max_map_rad = utils.EFFECTIVE_MAPPING_RANGES[physical_idx]
                angle_clamped = max(min_map_rad, min(max_map_rad, angle_rad))
                norm_val = (angle_clamped - min_map_rad) / (max_map_rad - min_map_rad)

                if utils._is_servo_direct_mapping(physical_idx):
                    raw = norm_val * 4095.0
                else:
                    raw = (1.0 - norm_val) * 4095.0

                return int(round(max(0, min(4095, raw))))

            # --- Joint 2 mirror (IDs 20 & 21) ---
            if 20 in raw_positions and 21 not in raw_positions:
                angle_rad_20 = servo_driver.servo_value_to_radians(raw_positions[20], 1)  # index of 20
                raw_positions[21] = _angle_to_raw(angle_rad_20, 2)  # index of 21

            # --- Joint 3 mirror (IDs 30 & 31) ---
            if 30 in raw_positions and 31 not in raw_positions:
                angle_rad_30 = servo_driver.servo_value_to_radians(raw_positions[30], 3)  # index of 30
                raw_positions[31] = _angle_to_raw(angle_rad_30, 4)  # index of 31

            # --- Control Law (Calculate Error and Correction) ---
            commands_for_sync_write = []
            
            compute_t0 = time.perf_counter()

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
                    # Commanded Angle = Target + Kp * Error
                    commanded_physical_angle_rad = target_physical_angle_rad + (utils.CORRECTION_KP_GAIN * error_rad)
                    
                    # 4. Convert the final commanded angle to a raw servo value (0-4095)
                    # This block is the same as in set_servo_positions
                    min_urdf_rad, max_urdf_rad = utils.URDF_JOINT_LIMITS[physical_servo_config_index]
                    angle_clamped_to_urdf = max(min_urdf_rad, min(max_urdf_rad, commanded_physical_angle_rad))
                    
                    min_map_rad, max_map_rad = utils.EFFECTIVE_MAPPING_RANGES[physical_servo_config_index]
                    angle_for_norm = max(min_map_rad, min(max_map_rad, angle_clamped_to_urdf))
                    normalized_value = (angle_for_norm - min_map_rad) / (max_map_rad - min_map_rad)
                    
                    raw_servo_value = (normalized_value * 4095.0) if utils._is_servo_direct_mapping(physical_servo_config_index) else ((1.0 - normalized_value) * 4095.0)
                    
                    final_servo_pos_value = int(round(raw_servo_value))
                    
                    # Add to the command list. Speed is max (4095) and Accel is none (0)
                    # because the path is timed by the closed-loop executor itself.
                    commands_for_sync_write.append((servo_id, final_servo_pos_value, 4095, 0))

            _compute_durations.append(time.perf_counter() - compute_t0)

            # --- Actuation (Write) ---
            if commands_for_sync_write:
                write_t0 = time.perf_counter()
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
        print("[Pi CLC] Closed-Loop Executor thread finished.")

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
            if diagnostics:
                try:
                    import matplotlib
                    matplotlib.use("Agg")  # headless backend
                    import matplotlib.pyplot as plt

                    session_id = utils.trajectory_state.get('diagnostics_session_id')
                    
                    if session_id:
                        # A session is active; save charts into the session folder.
                        out_dir = Path(f"diagnostics/closed_loop/{session_id}")
                    else:
                        # No session; save with a unique timestamp in the parent folder.
                        session_id = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
                        out_dir = Path("diagnostics/closed_loop")
                    out_dir.mkdir(exist_ok=True, parents=True)

                    # 1. Timing chart
                    fig, ax = plt.subplots(1, 1, figsize=(10, 4))
                    ax.plot([d * 1000 for d in _loop_durations], label="total")
                    ax.plot([d * 1000 for d in _read_durations], label="read")
                    ax.plot([d * 1000 for d in _compute_durations], label="compute")
                    ax.plot([d * 1000 for d in _write_durations], label="write")
                    ax.set_xlabel("Cycle index")
                    ax.set_ylabel("Duration (ms)")
                    ax.set_title("Closed-loop cycle timing")
                    ax.legend()
                    timing_path = out_dir / "timing.png"
                    fig.tight_layout()
                    fig.savefig(timing_path)
                    plt.close(fig)

                    # 2. Error chart per joint
                    fig, ax = plt.subplots(1, 1, figsize=(10, 4))
                    for j_idx, errs in enumerate(_abs_errors_per_joint):
                        if errs:
                            ax.plot([math.degrees(e) for e in errs], label=f"J{j_idx+1}")
                    ax.set_xlabel("Cycle index")
                    ax.set_ylabel("Abs error (deg)")
                    ax.set_title("Tracking error per joint")
                    ax.legend()
                    error_path = out_dir / "error.png"
                    fig.tight_layout()
                    fig.savefig(error_path)
                    plt.close(fig)

                    # Additional Sync Read breakdown if available
                    from arm_controller.servo_protocol import get_sync_profiles
                    sync_profiles = get_sync_profiles()
                    if sync_profiles:
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
                        print(f"[Pi CLC] Diagnostics charts saved to {out_dir}")
                    else:
                        print(f"[Pi CLC] Diagnostics charts saved to {timing_path} and {error_path}")
                except Exception as e:
                    print(f"[Pi CLC] WARNING: Failed to generate diagnostics charts: {e}")

        # Clean up global state
        utils.trajectory_state.update({"is_running": False, "should_stop": False, "thread": None})
        # Clean up session keys if they exist
        if 'diagnostics_session_id' in utils.trajectory_state:
            del utils.trajectory_state['diagnostics_session_id']
        if 'diagnostics_folder_type' in utils.trajectory_state:
            del utils.trajectory_state['diagnostics_folder_type']

