# Contains the high-level command handlers that parse and react to UDP messages. 
import os
import json
import time
import numpy as np
from scipy.spatial.transform import Rotation as R
import threading
import datetime

try:
    import ik_solver
except ImportError:
    print("ERROR: Missing 'ik_solver'. Ensure it is in the python path.")
    ik_solver = None

from . import utils
from . import servo_driver
from . import trajectory_execution

def handle_translate_command(dx: float, dy: float, dz: float):
    """
    Handles the 'TRANSLATE' command.
    Performs a simple, blocking, single-point IK move relative to the current
    pose while keeping orientation locked.
    """
    print(f"[Pi IK] Received TRANSLATE command: dx={dx}, dy={dy}, dz={dz}")

    # 1. Get current logical joint angles from our global state
    initial_angles = utils.current_logical_joint_angles_rad
    print(f"[Pi IK] Initial logical joint angles (rad): {np.round(initial_angles, 3)}")

    # 2. Use Forward Kinematics (FK) to find the current full pose (position and orientation)
    current_pose_matrix = ik_solver.get_fk_matrix(initial_angles)
    if current_pose_matrix is None:
        print("[Pi IK] ERROR: Failed to calculate current pose using FK.")
        return
    
    current_pos_xyz = current_pose_matrix[:3, 3]
    # This is the key: Lock the orientation to the current one.
    target_orientation_matrix = current_pose_matrix[:3, :3]
    print(f"[Pi IK] Current EE position (m): {np.round(current_pos_xyz, 4)}")
    print(f"[Pi IK] Locking orientation during translation.")

    # 3. Calculate the target position by adding the deltas
    target_pos_xyz = current_pos_xyz + np.array([dx, dy, dz])
    print(f"[Pi IK] Target EE position (m):  {np.round(target_pos_xyz, 4)}")

    # 4. Use Inverse Kinematics (IK) to find the required joint angles for the target position
    #    We provide the current angles as the starting point and the locked orientation.
    new_logical_joint_angles = ik_solver.solve_ik(
        target_position=target_pos_xyz,
        target_orientation_matrix=target_orientation_matrix,
        initial_joint_angles=initial_angles
    )

    if new_logical_joint_angles is None:
        print("[Pi IK] ERROR: IK solver failed to find a solution.")
        return

    print(f"[Pi IK] IK Solution Found (rad): {np.round(new_logical_joint_angles, 3)}")
    print(f"[Pi IK] IK Solution Found (deg): {np.round(np.rad2deg(new_logical_joint_angles), 2)}")

    # 5. Command the servos to the new angles
    #    Using default speed and acceleration for now. This could be made adjustable.
    servo_driver.set_servo_positions(new_logical_joint_angles, utils.DEFAULT_SERVO_SPEED, utils.DEFAULT_SERVO_ACCELERATION_DEG_S2)
    print("[Pi IK] Sent new positions to servos.")

    # 6. Get and print the final position for verification
    final_pos_xyz = ik_solver.get_fk(new_logical_joint_angles)
    if final_pos_xyz is not None:
        print(f"[Pi IK] Verification -> Target: {np.round(target_pos_xyz, 4)}, Final FK: {np.round(final_pos_xyz, 4)}")
        print(f"[Pi IK] Distance from target: {np.linalg.norm(final_pos_xyz - target_pos_xyz):.6f} m")

def handle_rotate_command(axis: str, angle_deg: float):
    """
    Handles the 'ROTATE' command.
    Performs a simple, blocking, single-point IK move to rotate the end effector
    around a specified axis of the base frame.
    """
    print(f"[Pi IK] Received ROTATE command: axis={axis}, angle={angle_deg} degrees")

    # 1. Get current state
    initial_angles = utils.current_logical_joint_angles_rad
    print(f"[Pi IK] Initial logical joint angles (rad): {np.round(initial_angles, 3)}")

    # 2. Get the full current pose (position and orientation) using FK
    current_pose_matrix = ik_solver.get_fk_matrix(initial_angles)
    if current_pose_matrix is None:
        print("[Pi IK] ERROR: Failed to calculate current pose using FK.")
        return

    current_position = current_pose_matrix[:3, 3]
    current_orientation = current_pose_matrix[:3, :3]
    print(f"[Pi IK] Current EE position (m): {np.round(current_position, 4)}")

    # 3. Create the rotation matrix for the new rotation
    try:
        # The rotation is created around the specified axis (x, y, or z)
        # This new rotation is then pre-multiplied with the current orientation matrix.
        # This results in a rotation being applied in the context of the base frame.
        rotation = R.from_euler(axis, angle_deg, degrees=True).as_matrix()
        target_orientation = rotation @ current_orientation
    except Exception as e:
        print(f"[Pi IK] ERROR: Failed to create rotation matrix: {e}")
        return

    # 4. The target position is the current position (we only want to rotate)
    target_position = current_position
    print(f"[Pi IK] Target EE orientation matrix:\n{np.round(target_orientation, 2)}")

    # 5. Use IK to find the new joint angles
    new_logical_joint_angles = ik_solver.solve_ik(
        target_position=target_position,
        target_orientation_matrix=target_orientation,
        initial_joint_angles=initial_angles
    )

    if new_logical_joint_angles is None:
        print("[Pi IK] ERROR: IK solver failed to find a solution for rotation.")
        return

    print(f"[Pi IK] IK Solution Found (deg): {np.round(np.rad2deg(new_logical_joint_angles), 2)}")

    # 6. Command servos
    servo_driver.set_servo_positions(new_logical_joint_angles, utils.DEFAULT_SERVO_SPEED, utils.DEFAULT_SERVO_ACCELERATION_DEG_S2)
    print("[Pi IK] Sent new positions to servos for rotation.")

    # 7. Get and print the final pose for verification
    final_pose_matrix = ik_solver.get_fk_matrix(new_logical_joint_angles)
    if final_pose_matrix is not None:
        final_position = final_pose_matrix[:3, 3]
        print(f"[Pi IK] Verification -> Target Pos: {np.round(target_position, 4)}, Final FK Pos: {np.round(final_position, 4)}")
        print(f"[Pi IK] Positional distance from target: {np.linalg.norm(final_position - target_position):.6f} m")


def handle_set_orientation_command(
    roll: float,
    pitch: float,
    yaw: float,
    *,
    closed_loop: bool = False,
    duration_s: float = 2.0,
    diagnostics: bool = False,
):
    """
    Handles the `SET_ORIENTATION` command.

    This command **smoothly re-orients** the tool tip to the specified absolute
    Euler angles **while keeping its Cartesian position fixed**.  Internally it:

    1.  Interpolates between the current and target orientations with a SLERP
        curve (density chosen from `duration_s` × execution frequency).
    2.  Solves IK in a single batched call for every intermediate pose, so the
        position constraint is enforced at all times.
    3.  Executes the resulting joint path either:
        • **Open-loop** at 1300 Hz  (default, high-speed)
        • **Closed-loop** at 400 Hz (`closed_loop=True`, high-precision)

    Because the path is pre-planned, the function is *blocking*: it only
    returns after the motion (≈ `duration_s`) has finished.

    Parameters
    ----------
    roll, pitch, yaw : float
        Absolute tool orientation in degrees, XYZ intrinsic Euler order.
    closed_loop : bool, optional
        Run the high-precision closed-loop executor at 400 Hz instead of the
        high-speed open-loop executor.  Default `False`.
    duration_s : float, optional
        Desired motion duration (≥ 0.1 s).  Controls the smoothness/speed by
        scaling the number of interpolation steps.  Default `1.0`.
    """
    print(f"[Pi IK] Received SET_ORIENTATION command: Roll={roll}, Pitch={pitch}, Yaw={yaw} degrees")

    # 1. Get current state (joint angles and full pose).
    initial_angles = utils.current_logical_joint_angles_rad
    current_pose_matrix = ik_solver.get_fk_matrix(initial_angles)
    if current_pose_matrix is None:
        print("[Pi IK] ERROR: Failed to calculate current pose using FK.")
        return

    current_position = current_pose_matrix[:3, 3]
    current_orientation = current_pose_matrix[:3, :3]

    # 2. Build the target orientation matrix from Euler angles (XYZ intrinsic).
    try:
        target_orientation = R.from_euler('xyz', [roll, pitch, yaw], degrees=True).as_matrix()
    except Exception as e:
        print(f"[Pi IK] ERROR: Failed to create orientation matrix from Euler angles: {e}")
        return

    print(f"[Pi IK] Target EE Orientation Matrix:\n{np.round(target_orientation, 2)}")
    print(f"[Pi IK] Maintaining EE Position at: {np.round(current_position, 4)}")

    # --- 3. Set up diagnostics session if enabled ---
    session_id = None
    if os.environ.get("MINI_ARM_IK_LOG", "0") == "1" or diagnostics:
        # Use a consistent timestamp for all diagnostics in this session
        session_id = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        utils.trajectory_state['diagnostics_session_id'] = session_id
        # The IK logger will use this to place the plan in the correct subfolder
        utils.trajectory_state['diagnostics_folder_type'] = "closed_loop" if closed_loop else "open_loop"

    # ------------------------------------------------------------------
    # 3. Generate an orientation-only path that keeps the tool-tip fixed.
    # ------------------------------------------------------------------
    # ------------------------------------------------------------
    #   3. Determine execution parameters
    # ------------------------------------------------------------
    frequency_hz = 400 if closed_loop else 1300  # Align with other commands

    # Use caller-provided duration (default 1 s) to scale interpolation density.
    duration_s = max(0.1, duration_s)  # clamp to sane minimum
    NUM_STEPS = max(2, int(duration_s * frequency_hz))

    try:
        # Use spherical linear interpolation (SLERP) between current and target orientation
        from scipy.spatial.transform import Slerp  # local import to avoid polluting global namespace

        rot_start = R.from_matrix(current_orientation)
        rot_end = R.from_matrix(target_orientation)

        key_rots = R.concatenate([rot_start, rot_end])
        key_times = [0, 1]
        slerp = Slerp(key_times, key_rots)
        times = np.linspace(0, 1, NUM_STEPS)
        interpolated_rots = slerp(times)
        orientation_matrices = [r.as_matrix() for r in interpolated_rots]
    except Exception as e:
        print(f"[Pi IK] ERROR: Failed to build SLERP interpolation: {e}")
        return

    # Build constant-position list matching orientation list
    path_positions = [current_position] * NUM_STEPS

    # ------------------------------------------------------------------
    # 4. Solve IK for the entire path in one batched call.
    # ------------------------------------------------------------------
    joint_path = ik_solver.solve_ik_path_batch(
        path_points=path_positions,
        initial_joint_angles=initial_angles,
        target_orientations=orientation_matrices,
    )

    if joint_path is None:
        print("[Pi IK] ERROR: IK solver failed to find a solution for the orientation path.")
        return

    # ------------------------------------------------------------------
    # 5. Execute the joint path (blocking) using the selected executor.
    # ------------------------------------------------------------------
    if closed_loop:
        target_func = trajectory_execution._closed_loop_executor_thread
    else:
        target_func = trajectory_execution._open_loop_executor_thread
    
    executor_thread = threading.Thread(
        target=target_func,
        kwargs={'joint_path': joint_path, 'frequency': frequency_hz, 'diagnostics': diagnostics}
    )

    executor_thread.start()
    executor_thread.join() # Block until the move is finished

    # ------------------------------------------------------------------
    # 6. Final verification (optional, quick FK check).
    # ------------------------------------------------------------------
    final_pose_matrix = ik_solver.get_fk_matrix(joint_path[-1])
    if final_pose_matrix is not None:
        final_position = final_pose_matrix[:3, 3]
        final_orientation = final_pose_matrix[:3, :3]

        # Compare orientation error
        orient_error_matrix = np.transpose(target_orientation) @ final_orientation
        angle_rad, _, _ = R.from_matrix(orient_error_matrix).as_rotvec()

        print(f"[Pi IK] Verification -> Final Pos: {np.round(final_position, 4)}")
        print(f"[Pi IK] Positional error: {np.linalg.norm(final_position - current_position):.6f} m")
        print(f"[Pi IK] Orientational error: {np.rad2deg(np.linalg.norm(angle_rad)):.3f} degrees")

    # --- Clean up diagnostics session ---
    if session_id:
        del utils.trajectory_state['diagnostics_session_id']
        del utils.trajectory_state['diagnostics_folder_type']


def handle_move_profiled(target_x: float, 
                         target_y: float, 
                         target_z: float, 
                         velocity: float, 
                         acceleration: float, 
                         frequency: int = 400, 
                         use_smoothing: bool = True, 
                         closed_loop: bool = False,
                         diagnostics: bool = False
                         ):
    """
    Handles the 'MOVE_PROFILED' command. This is the core handler for all
    high-precision, profiled, non-blocking linear moves. It plans the full path,
    then starts the closed-loop executor in a background thread.
    """
    print(f"[Pi Smooth] Received MOVE_PROFILED command to [{target_x}, {target_y}, {target_z}]")
    
    if utils.trajectory_state["is_running"]:
        print("[Pi Smooth] ERROR: Cannot start move, another task is running.")
        return

    # 1. Get current state from the physical robot to start the plan
    initial_q = servo_driver.get_current_arm_state_rad(verbose=False)
    target_pos = np.array([target_x, target_y, target_z])

    diagnostics_enabled = os.environ.get("MINI_ARM_IK_LOG", "0") == "1" or diagnostics

    # --- Set up diagnostics session if enabled ---
    if diagnostics_enabled:
        session_id = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        utils.trajectory_state['diagnostics_session_id'] = session_id
        utils.trajectory_state['diagnostics_folder_type'] = "closed_loop" if closed_loop else "open_loop"

    if closed_loop:
        frequency = 400
    else:
        # With diagnostics, the open-loop executor reads feedback and is much slower.
        # We must plan the trajectory at the slower rate to ensure the move duration is correct.
        frequency = 400 if diagnostics_enabled else 1300

    # 2. Plan the entire move.
    joint_path = trajectory_execution._plan_smooth_move(
        start_q=initial_q,
        target_pos=target_pos,
        velocity=velocity,
        acceleration=acceleration,
        frequency=frequency,
        use_smoothing=use_smoothing
    )
    
    # 3. If planning was successful, choose executor
    if joint_path:
        executor_fn = (trajectory_execution._closed_loop_executor_thread
                       if closed_loop
                       else trajectory_execution._open_loop_executor_thread)

        executor_thread = threading.Thread(
            target=executor_fn,
            # Pass diagnostics flag to the executor thread
            kwargs={'joint_path': joint_path, 'frequency': frequency, 'diagnostics': diagnostics_enabled},
            daemon=True,
        )
        utils.trajectory_state["thread"] = executor_thread
        utils.trajectory_state["is_running"] = True
        utils.trajectory_state["should_stop"] = False
        executor_thread.start()
        print("[Pi Smooth] Trajectory started "
              f"({'closed' if closed_loop else 'open'} loop, background).")
    else:
        print("[Pi Smooth] ERROR: Move failed because path planning was unsuccessful.")


def handle_move_profiled_relative(dx: float, dy: float, dz: float, speed: float = 1.0, use_smoothing: bool = True):
    """
    Handles the 'MOVE_PROFILED_RELATIVE' command. Calculates the absolute
    target position and then calls the main `handle_move_profiled` handler.
    """
    print(f"[Pi Smooth] Received MOVE_PROFILED_RELATIVE command: dX={dx}, dY={dy}, dZ={dz}, SpeedMultiplier={speed}")

    # 1. Get current position
    current_q = servo_driver.get_current_arm_state_rad(verbose=False)
    start_pos = ik_solver.get_fk(current_q)
    if start_pos is None:
        print("[Pi Smooth] ERROR: Cannot start relative move, failed to get current position.")
        return
        
    # 2. Calculate absolute target position
    target_pos = start_pos + np.array([dx, dy, dz])
    
    # 3. Calculate profiled move parameters
    target_velocity = utils.DEFAULT_PROFILE_VELOCITY * speed
    target_acceleration = utils.DEFAULT_PROFILE_ACCELERATION
    
    # 4. Call the absolute profiled move handler to perform the action
    print(f"[Pi Smooth] Calculated absolute target: {np.round(target_pos, 4)}. Executing profiled move.")
    handle_move_profiled(target_pos[0], target_pos[1], target_pos[2], target_velocity, target_acceleration, use_smoothing=use_smoothing)


def handle_run_trajectory(trajectory_name: str, use_cache: bool = False):
    """
    Handles the 'RUN_TRAJECTORY' command. It loads a trajectory definition
    from a JSON file, plans all the constituent moves, and then starts the
    trajectory executor thread to run the full sequence.
    """
    if utils.trajectory_state["is_running"]:
        print("[Pi Trajectory] ERROR: Cannot start new trajectory, one is already running. Send 'STOP' first.")
        return

    print(f"[Pi Trajectory] Received RUN_TRAJECTORY for '{trajectory_name}' (Use Cache: {use_cache})")

    # --- 1. Load Trajectory Definition ---
    trajectory = _load_trajectory_by_name(trajectory_name)

    if trajectory is None:
        print(f"[Pi Trajectory] ERROR: Trajectory '{trajectory_name}' not found.")
        return

    moves = trajectory.get("moves", [])
    should_loop = trajectory.get("loop", False)
    # NEW: Parse orientation lock from trajectory file
    orientation_lock_euler = trajectory.get("orientation_euler_angles_deg")
    target_orientation_matrix = None
    if orientation_lock_euler:
        try:
            # Note: scipy is imported at the top of pi_controller
            target_orientation_matrix = R.from_euler('xyz', orientation_lock_euler, degrees=True).as_matrix()
            print(f"[Pi Trajectory] Orientation will be locked to Euler (deg): {orientation_lock_euler}")
        except Exception as e:
            print(f"[Pi Trajectory] WARNING: Invalid Euler angles in trajectory file: {e}. Ignoring orientation lock.")

    print(f"[Pi Trajectory] Found '{trajectory_name}': {trajectory.get('description', 'No description')}")
    print(f"[Pi Trajectory] This trajectory will loop: {should_loop}")

    # --- 2. Get Planned Path (from Cache or by Planning) ---
    cache_file_path = os.path.join(utils.TRAJECTORY_CACHE_DIR, f"{trajectory_name}.json")
    planned_steps = None

    if use_cache:
        if os.path.exists(cache_file_path):
            print(f"[Pi Trajectory] Loading pre-computed path from cache: {cache_file_path}")
            try:
                with open(cache_file_path, 'r') as f:
                    planned_steps = json.load(f)
            except Exception as e:
                print(f"[Pi Trajectory] WARNING: Failed to load or parse cached file: {e}. Re-planning.")
                planned_steps = None # Ensure we re-plan on failure
        else:
            print(f"[Pi Trajectory] WARNING: Cache requested but not found. Planning trajectory...")

    # If we don't have a plan yet (either not requested or failed to load), plan it.
    if planned_steps is None:
        print("\n--- Starting Trajectory Planning Phase (Pre-computation) ---")
        planned_steps = []
        
        # Start planning from the robot's current known state, not a hardcoded home position.
        current_q = np.array(utils.current_logical_joint_angles_rad)
        print(f"[Pi Trajectory] Planning will start from current state (rad): {np.round(current_q, 3)}")
        
        planning_succeeded = True
        for i, move_cmd in enumerate(moves):
            command = move_cmd.get("command")
            print(f"[Pi Plan] Planning Command {i+1}/{len(moves)}: {command}...")

            if command == "home":
                home_q_list = [0.0] * utils.NUM_LOGICAL_JOINTS
                speed = move_cmd.get("speed", utils.DEFAULT_SERVO_SPEED)
                duration = move_cmd.get("duration", 2.5)
                planned_steps.append({
                    'type': 'joint_move', 'target_q': home_q_list, 'speed': speed, 'duration': duration
                })
                current_q = np.array(home_q_list)
            
            elif command == "move_relative":
                t_start_plan = time.monotonic()
                vector = np.array(move_cmd.get("vector", [0,0,0]))
                speed_mult = move_cmd.get("speed_multiplier", 1.0)

                # Per-move orientation override
                move_orient_euler = move_cmd.get("orientation_euler_deg")
                per_move_orientation_matrix = None
                if move_orient_euler is not None:
                    try:
                        per_move_orientation_matrix = R.from_euler('xyz', move_orient_euler, degrees=True).as_matrix()
                    except Exception as e:
                        print(f"[Pi Plan] WARNING: Invalid per-move Euler orientation: {e}. Ignoring.")

                start_pos = ik_solver.get_fk(current_q)
                if start_pos is None:
                    print(f"[Pi Trajectory] ERROR: Could not get start position for relative move. Aborting plan.")
                    planning_succeeded = False
                    break
                
                target_pos = start_pos + vector
                forced_orient = per_move_orientation_matrix if per_move_orientation_matrix is not None else target_orientation_matrix

                joint_path = trajectory_execution._plan_linear_move(
                    current_q, target_pos, utils.DEFAULT_PROFILE_VELOCITY * speed_mult, utils.DEFAULT_PROFILE_ACCELERATION, 100, True,
                    forced_orientation=forced_orient
                )
                
                if joint_path:
                    t_end_plan = time.monotonic()
                    print(f"[Pi Plan] Planning complete for move. Took {(t_end_plan - t_start_plan) * 1000:.2f} ms")
                    planned_steps.append({'type': 'move', 'path': joint_path, 'freq': 100})
                    current_q = np.array(joint_path[-1])
                else:
                    planning_succeeded = False
                    break

            elif command == "move_absolute":
                t_start_plan = time.monotonic()
                target_pos = np.array(move_cmd.get("vector", [0,0,0]))
                speed_mult = move_cmd.get("speed_multiplier", 1.0)

                move_orient_euler = move_cmd.get("orientation_euler_deg")
                per_move_orientation_matrix = None
                if move_orient_euler is not None:
                    try:
                        per_move_orientation_matrix = R.from_euler('xyz', move_orient_euler, degrees=True).as_matrix()
                    except Exception as e:
                        print(f"[Pi Plan] WARNING: Invalid per-move Euler orientation: {e}. Ignoring.")

                forced_orient = per_move_orientation_matrix if per_move_orientation_matrix is not None else target_orientation_matrix

                joint_path = trajectory_execution._plan_linear_move(
                    current_q, target_pos, utils.DEFAULT_PROFILE_VELOCITY * speed_mult, utils.DEFAULT_PROFILE_ACCELERATION, 100, True,
                    forced_orientation=forced_orient
                )
                    
                if joint_path:
                    t_end_plan = time.monotonic()
                    print(f"[Pi Plan] Planning complete for move. Took {(t_end_plan - t_start_plan) * 1000:.2f} ms")
                    planned_steps.append({'type': 'move', 'path': joint_path, 'freq': 100})
                    current_q = np.array(joint_path[-1])
                else:
                    planning_succeeded = False
                    break
                    
            elif command == "move_arc":
                t_start_plan = time.monotonic()
                end_pos = np.array(move_cmd.get("end_point", [0,0,0]))
                center_pos = np.array(move_cmd.get("center_point", [0,0,0]))
                speed_mult = move_cmd.get("speed_multiplier", 1.0)
                
                start_pos = ik_solver.get_fk(current_q)
                if start_pos is None:
                    print(f"[Pi Trajectory] ERROR: Could not get start position for arc move. Aborting plan.")
                    planning_succeeded = False
                    break
                
                # 1. Generate the Cartesian path for the arc
                cartesian_path = trajectory_planner.generate_arc_trajectory(
                    start_pos, end_pos, center_pos, utils.DEFAULT_PROFILE_VELOCITY * speed_mult, utils.DEFAULT_PROFILE_ACCELERATION, 100)
                
                if not cartesian_path:
                    print(f"[Pi Trajectory] ERROR: Could not generate arc trajectory. Aborting plan.")
                    planning_succeeded = False
                    break

                # 2. Plan the joint space path from the Cartesian points
                joint_path = trajectory_execution._plan_high_fidelity_trajectory(
                    cartesian_points=cartesian_path,
                    start_q=current_q,
                    use_smoothing=True,
                    forced_orientation=target_orientation_matrix
                )

                if joint_path:
                    t_end_plan = time.monotonic()
                    print(f"[Pi Plan] Planning complete for move. Took {(t_end_plan - t_start_plan) * 1000:.2f} ms")
                    planned_steps.append({'type': 'move', 'path': joint_path, 'freq': 100})
                    current_q = np.array(joint_path[-1])
                else:
                    planning_succeeded = False
                    break

            elif command == "pause":
                duration = move_cmd.get("duration", 1.0)
                planned_steps.append({'type': 'pause', 'duration': duration})
                
            else:
                print(f"[Pi Trajectory] WARNING: Unknown command '{command}' in trajectory. Skipping.")

        if not planning_succeeded:
            print("[Pi Trajectory] FATAL: Planning failed for one of the moves. Aborting execution.")
            return

        # After successful planning, save the result to cache
        try:
            os.makedirs(utils.TRAJECTORY_CACHE_DIR, exist_ok=True)
            # We need to convert numpy arrays to lists before saving
            serializable_planned_steps = utils._convert_numpy_to_list(planned_steps)
            with open(cache_file_path, 'w') as f:
                json.dump(serializable_planned_steps, f, indent=2)
            print(f"[Pi Trajectory] Successfully saved planned path to cache: {cache_file_path}")
        except Exception as e:
            print(f"[Pi Trajectory] WARNING: Failed to save planned path to cache: {e}")
        
    # --- 2. Execution Phase ---
    print("\n--- Trajectory Ready. Starting Execution in a background thread ---")
    
    utils.trajectory_state["is_running"] = True
    utils.trajectory_state["should_stop"] = False
    
    executor_thread = threading.Thread(
        target=trajectory_execution._trajectory_executor_thread,
        args=(planned_steps, should_loop)
    )
    utils.trajectory_state["thread"] = executor_thread
    executor_thread.start()
    
    print("[Pi Trajectory] Trajectory thread started. Main loop is responsive.")


def handle_stop_command():
    """
    Handles the 'STOP' command by setting the global 'should_stop' flag.
    The running motion thread will detect this flag and exit cleanly.
    """
    if utils.trajectory_state["is_running"]:
        print("[Pi Control] STOP command received. Signaling trajectory to stop...")
        utils.trajectory_state["should_stop"] = True
    else:
        print("[Pi Control] STOP command received, but no trajectory is running.")


def handle_move_to_position_absolute(x: float, y: float, z: float):
    """
    Handles the 'MOVE' command (legacy).
    Performs a simple, blocking, single-point IK move to an absolute position
    with no orientation constraint.
    """
    print(f"[Pi IK] Received MOVE command: x={x}, y={y}, z={z}")

    # 1. Get current logical joint angles from our global state
    initial_angles = utils.current_logical_joint_angles_rad
    print(f"[Pi IK] Initial logical joint angles (rad): {np.round(initial_angles, 3)}")
    print(f"[Pi IK] Orientation is UNLOCKED for this move.")

    # 2. Set the target position from the absolute coordinates
    target_pos_xyz = np.array([x, y, z])
    print(f"[Pi IK] Target EE position (m):  {np.round(target_pos_xyz, 4)}")

    # 3. Use Inverse Kinematics (IK) to find the required joint angles for the target position.
    #    We provide the current angles as the starting point. Orientation is not constrained.
    new_logical_joint_angles = ik_solver.solve_ik(
        target_position=target_pos_xyz,
        initial_joint_angles=initial_angles
    )

    if new_logical_joint_angles is None:
        print("[Pi IK] ERROR: IK solver failed to find a solution.")
        return

    print(f"[Pi IK] IK Solution Found (rad): {np.round(new_logical_joint_angles, 3)}")
    print(f"[Pi IK] IK Solution Found (deg): {np.round(np.rad2deg(new_logical_joint_angles), 2)}")

    # 4. Command the servos to the new angles
    #    Using default speed and acceleration for now. This could be made adjustable.
    servo_driver.set_servo_positions(new_logical_joint_angles, utils.DEFAULT_SERVO_SPEED, utils.DEFAULT_SERVO_ACCELERATION_DEG_S2)
    print("[Pi IK] Sent new positions to servos.")

    # 5. Get and print the final position for verification
    final_pos_xyz = ik_solver.get_fk(new_logical_joint_angles)
    if final_pos_xyz is not None:
        print(f"[Pi IK] Verification -> Target: {np.round(target_pos_xyz, 4)}, Final FK: {np.round(final_pos_xyz, 4)}")
        print(f"[Pi IK] Distance from target: {np.linalg.norm(final_pos_xyz - target_pos_xyz):.6f} m")


def handle_get_position(sock: 'socket.socket', addr: tuple):
    """
    Handles the 'GET_POSITION' command.
    Calculates the current end-effector pose (position and joint angles) using FK 
    and sends it back to the requester.
    """
    print(f"[Pi] Received GET_POSITION from {addr}.")

    # Fetch the latest joint angles directly from the physical servos
    current_angles = servo_driver.get_current_arm_state_rad(verbose=False)
    
    # Get the current position using Forward Kinematics
    current_pos_xyz = ik_solver.get_fk(current_angles)

    if current_pos_xyz is not None:
        # Round cartesian coordinates to 3 decimal places as requested
        pos_rounded = [round(p, 3) for p in current_pos_xyz]
        
        # Round joint angles for cleaner display
        angles_rounded = [round(a, 4) for a in current_angles]

        pos_str = ",".join(map(str, pos_rounded))
        angles_str = ",".join(map(str, angles_rounded))
        
        print(f"[Pi] Sending coordinates pose: {pos_str}")
        print(f"[Pi] Sending joint angles: {angles_str}")

        reply_msg = f"CURRENT_POSE,{pos_str},{angles_str}"
        
        try:
            sock.sendto(reply_msg.encode("utf-8"), addr)
        except Exception as e:
            print(f"[Pi] Error sending CURRENT_POSE to {addr}: {e}")
    else:
        print("[Pi] ERROR: Could not calculate current position (FK failed).")
        try:
            sock.sendto("ERROR,FK_FAILED".encode("utf-8"), addr)
        except Exception as e:
            print(f"[Pi] Error sending FK_FAILED error to {addr}: {e}")


def handle_move_line(target_x: float, target_y: float, target_z: float, velocity: float, acceleration: float, closed_loop: bool = True):
    """Convenience wrapper that calls the main profiled move handler, defaulting to closed-loop."""
    handle_move_profiled(
        target_x, target_y, target_z, velocity, acceleration,
        closed_loop=closed_loop,
        use_smoothing=True,
        diagnostics=False
    )

def handle_move_line_relative(dx: float, dy: float, dz: float, speed: float = 1.0, closed_loop: bool = True):
    """Convenience wrapper that calls the main profiled move handler, defaulting to closed-loop."""
    current_q = servo_driver.get_current_arm_state_rad(verbose=False)
    if current_q is None:
        print("[Pi Smooth] ERROR: Cannot start relative move, failed to get current position.")
        return
        
    start_pos = ik_solver.get_fk(current_q)
    if start_pos is None:
        print("[Pi Smooth] ERROR: Cannot start relative move, failed to get start position.")
        return
        
    target_pos = start_pos + np.array([dx, dy, dz])
    
    handle_move_profiled(
        target_pos[0], target_pos[1], target_pos[2],
        velocity=utils.DEFAULT_PROFILE_VELOCITY * speed,
        acceleration=utils.DEFAULT_PROFILE_ACCELERATION,
        closed_loop=closed_loop,
        use_smoothing=True,
        diagnostics=False
    )


def handle_wait_for_idle():
    """
    This is a blocking call that waits until the currently running trajectory is finished.
    """
    # Check if a trajectory thread exists and is running
    if utils.trajectory_state.get("is_running") and utils.trajectory_state.get("thread"):
        print("[Controller] Waiting for current move to complete...")
        thread = utils.trajectory_state.get("thread")
        if thread:
            thread.join() # Wait for the thread to finish
        print("[Controller] Move complete. Resuming.")
    else:
        print("[Controller] No move is currently running.")

# -----------------------------------------------------------------------------
# Recording subsystem: PLAN_TRAJECTORY / REC_POS / END_TRAJECTORY
# -----------------------------------------------------------------------------

# Holds intermediate state while a user is interactively recording a trajectory.
_recording_state = {
    "is_recording": False,
    "points": [],  # list of dicts: {"position": [...], "orientation_euler_deg": [...]}
    "start_time": None,
}

RECORDED_TRAJ_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", "recorded_trajectories"))


def _ensure_record_dir_exists():
    """Create the recorded_trajectories directory if it does not already exist."""
    try:
        os.makedirs(RECORDED_TRAJ_DIR, exist_ok=True)
    except Exception as e:
        print(f"[Recorder] ERROR: Could not create directory {RECORDED_TRAJ_DIR}: {e}")


def handle_plan_trajectory_start():
    """Initiate a new recording session."""
    if _recording_state["is_recording"]:
        print("[Recorder] WARNING: Already recording. Send END_TRAJECTORY first if you want to start a new one.")
        return

    _recording_state["is_recording"] = True
    _recording_state["points"].clear()
    _recording_state["start_time"] = datetime.datetime.now()
    print("[Recorder] *** Recording mode ENABLED. Use REC_POS to add way-points, END_TRAJECTORY,<name> to finish. ***")


def handle_record_position():
    """Record the current end-effector pose (position + orientation)."""
    if not _recording_state["is_recording"]:
        print("[Recorder] ERROR: Not currently recording. Send PLAN_TRAJECTORY first.")
        return

    # Query current joint angles and FK
    current_q = servo_driver.get_current_arm_state_rad(verbose=False)
    pose_matrix = ik_solver.get_fk_matrix(current_q)
    if pose_matrix is None:
        print("[Recorder] ERROR: FK failed – cannot record point.")
        return

    position = pose_matrix[:3, 3].tolist()
    # Convert orientation matrix → XYZ intrinsic Euler degrees for human-friendly storage
    orientation_euler_deg = R.from_matrix(pose_matrix[:3, :3]).as_euler('xyz', degrees=True).tolist()

    _recording_state["points"].append({
        "position": [round(p, 4) for p in position],
        "orientation_euler_deg": [round(o, 2) for o in orientation_euler_deg],
    })
    print(f"[Recorder] Way-point #{len(_recording_state['points'])} recorded: Pos={position}, EulerDeg={orientation_euler_deg}")


def handle_end_trajectory(traj_name: str):
    """Finalize the recording and dump to JSON file under recorded_trajectories/."""
    if not _recording_state["is_recording"]:
        print("[Recorder] ERROR: Not currently recording – nothing to end.")
        return

    if not traj_name:
        print("[Recorder] ERROR: Trajectory name required. Use END_TRAJECTORY,<name>.")
        return

    if len(_recording_state["points"]) == 0:
        print("[Recorder] WARNING: No points recorded – nothing will be saved.")
        _recording_state["is_recording"] = False
        return

    _ensure_record_dir_exists()

    file_path = os.path.join(RECORDED_TRAJ_DIR, f"{traj_name}.json")
    if os.path.exists(file_path):
        print(f"[Recorder] WARNING: File {file_path} already exists – it will be overwritten.")

    # Build moves list – for now we store as move_absolute steps with 1 s pauses between
    moves = []
    for i, p in enumerate(_recording_state["points"]):
        moves.append({
            "command": "move_absolute",
            "vector": p["position"],
            "orientation_euler_deg": p["orientation_euler_deg"],
        })
        if i < len(_recording_state["points"]) - 1:
            moves.append({"command": "pause", "duration": 1.0})

    traj_dict = {
        "description": f"Recorded on {_recording_state['start_time'].strftime('%Y-%m-%d %H:%M:%S')}",
        "loop": False,
        "orientation_euler_angles_deg": None,  # kept for future use
        "moves": moves,
    }

    try:
        with open(file_path, "w") as f:
            json.dump(traj_dict, f, indent=2)
        print(f"[Recorder] Trajectory saved to {file_path} (total moves: {len(moves)})")
    except Exception as e:
        print(f"[Recorder] ERROR: Failed to write file {file_path}: {e}")

    # Reset state
    _recording_state["is_recording"] = False
    _recording_state["points"].clear()

# -----------------------------------------------------------------------------
# Utility: load trajectory file (default + recorded)
# -----------------------------------------------------------------------------

def _load_trajectory_by_name(name: str):
    """Return trajectory dict by checking recorded_trajectories first, then trajectories.json."""
    # 1) Recorded folder
    recorded_path = os.path.join(RECORDED_TRAJ_DIR, f"{name}.json")
    if os.path.exists(recorded_path):
        try:
            with open(recorded_path, "r") as f:
                print(f"[Pi Trajectory] Loading recorded trajectory: {recorded_path}")
                return json.load(f)
        except Exception as e:
            print(f"[Pi Trajectory] ERROR: Could not load recorded trajectory {recorded_path}: {e}")

    # 2) Built-in trajectories.json
    fallback_path = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", "trajectories.json"))
    try:
        with open(fallback_path, "r") as f:
            all_traj = json.load(f)
            return all_traj.get(name)
    except Exception as e:
        print(f"[Pi Trajectory] ERROR: Could not load fallback trajectories.json: {e}")

    return None

