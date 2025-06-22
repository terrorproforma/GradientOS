# Contains the logic for planning and executing timed trajectories,
# including the high-fidelity planner and the closed-loop executor thread. 
import time
import math
import numpy as np
from scipy.signal import savgol_filter
from typing import Sequence, Union

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


def _closed_loop_executor_thread(joint_path: list[list[float]], frequency: int):
    """
    Executes a pre-planned joint-space trajectory using a real-time, closed-loop
    proportional controller to ensure path accuracy. This is the primary executor
    for high-precision moves.

    Args:
        joint_path: The dense list of target joint angle configurations.
        frequency: The target execution frequency for the control loop (e.g., 200 Hz).
    """
    print(f"[Pi CLC] Starting Closed-Loop Executor at {frequency} Hz for a path with {len(joint_path)} steps.")
    
    try:
        time_step = 1.0 / frequency
        start_time = time.monotonic()
        
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

        for i, target_q_step in enumerate(joint_path):
            deadline = start_time + (i + 1) * time_step
            
            # --- Stop Check ---
            if utils.trajectory_state["should_stop"]:
                print("[Pi CLC] Stop signal received, halting execution.")
                break

            # --- Feedback (Read) ---
            # Use a fixed but small timeout to ensure full packets arrive; setting
            # too low leads to intermittent Sync Read failures.
            per_cycle_timeout = max(0.01, time_step * 0.8)
            raw_positions = servo_protocol.sync_read_positions(
                all_physical_servo_ids,
                timeout_s=per_cycle_timeout,
                poll_delay_s=0.0,
            )
            if raw_positions is None:
                print("[Pi CLC] WARNING: Failed to read servo feedback. Stopping path.")
                break

            # --- Control Law (Calculate Error and Correction) ---
            commands_for_sync_write = []
            
            for logical_joint_index, target_angle_rad in enumerate(target_q_step):
                # This logic mirrors set_servo_positions to find the target physical angle.
                angle_with_master_offset = target_angle_rad + utils.LOGICAL_JOINT_MASTER_OFFSETS_RAD[logical_joint_index]
                target_physical_angle_rad = angle_with_master_offset
                if logical_joint_index == 0:
                    target_physical_angle_rad *= 2.0

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

            # --- Actuation (Write) ---
            if commands_for_sync_write:
                servo_protocol.sync_write_goal_pos_speed_accel(commands_for_sync_write)

            # --- Timing ---
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
        # Clean up global state
        utils.trajectory_state["is_running"] = False
        utils.trajectory_state["should_stop"] = False
        utils.trajectory_state["thread"] = None

