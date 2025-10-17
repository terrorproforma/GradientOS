# Contains the high-level command handlers that parse and react to UDP messages.
import os
import json
import time
from scipy.spatial.transform import Slerp  # needed for reset/initial path slerp interpolation
import numpy as np
from scipy.spatial.transform import Rotation as R
import threading
import datetime

# --- Global State for Motion Control ---
# The motion_stopped Event has been removed to use the native trajectory_state flag.

try:
    from .. import ik_solver
except ImportError:
    print("ERROR: Missing 'ik_solver'. Ensure it is in the python path.")
    ik_solver = None
    trajectory_planner = None

from . import utils
from . import servo_driver
from . import servo_protocol
from . import trajectory_execution
from . import pid_tuner

def handle_translate_command(dx: float, dy: float, dz: float):
    """
    Handles the 'TRANSLATE' command.
    Performs a simple, blocking, single-point IK move relative to the current
    pose while keeping orientation locked.
    """
    utils.trajectory_state["should_stop"] = False # Reset stop flag on new move
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
    utils.trajectory_state["should_stop"] = False # Reset stop flag on new move
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
    closed_loop: bool = True,
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
        • **Closed-loop** at 50 Hz (default)
        • **Open-loop** at 100 Hz (`closed_loop=False`)

    Because the path is pre-planned, the function is *blocking*: it only
    returns after the motion (≈ `duration_s`) has finished.

    Parameters
    ----------
    roll, pitch, yaw : float
        Absolute tool orientation in degrees, XYZ intrinsic Euler order.
    closed_loop : bool, optional
        Executes closed-loop at 50 Hz; open-loop at 100 Hz when `False`.
        Default `True` for closed-loop.
    duration_s : float, optional
        Desired motion duration (≥ 0.1 s).  Controls the smoothness/speed by
        scaling the number of interpolation steps.  Default `1.0`.
    """
    utils.trajectory_state["should_stop"] = False # Reset stop flag on new move
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
    diagnostics_enabled = (
        os.environ.get("MINI_ARM_IK_LOG", "0") == "1"
        or diagnostics
        or utils.trajectory_state.get("diagnostics_enabled", False)
    )
    if diagnostics_enabled:
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
    frequency_hz = 50 if closed_loop else 100

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
        kwargs={'joint_path': joint_path, 'frequency': frequency_hz, 'diagnostics': diagnostics_enabled}
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
                         frequency: int = 100, 
                         use_smoothing: bool = True, 
                         closed_loop: bool = True,
                         diagnostics: bool = False
                         ):
    """
    Handles the 'MOVE_PROFILED' command. This is the core handler for all
    high-precision, profiled, non-blocking linear moves. It plans the full path,
    then starts the closed-loop executor in a background thread.
    """
    utils.trajectory_state["should_stop"] = False # Reset stop flag on new move
    print(f"[Pi Smooth] Received MOVE_PROFILED command to [{target_x}, {target_y}, {target_z}]")
    
    if utils.trajectory_state["is_running"]:
        print("[Pi Smooth] ERROR: Cannot start move, another task is running.")
        return

    # 1. Get current state from the physical robot to start the plan
    initial_q = servo_driver.get_current_arm_state_rad(verbose=False)
    target_pos = np.array([target_x, target_y, target_z])

    diagnostics_enabled = (
        os.environ.get("MINI_ARM_IK_LOG", "0") == "1"
        or diagnostics
        or utils.trajectory_state.get("diagnostics_enabled", False)
    )

    # --- Set up diagnostics session if enabled ---
    if diagnostics_enabled:
        session_id = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        utils.trajectory_state['diagnostics_session_id'] = session_id
        utils.trajectory_state['diagnostics_folder_type'] = "closed_loop" if closed_loop else "open_loop"

    if closed_loop:
        frequency = 50
    else:
        # Standardize default open-loop planning/execution to 100 Hz as well
        frequency = 100

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
    utils.trajectory_state["should_stop"] = False # Reset stop flag on new move
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

def handle_run_trajectory(trajectory_name: str, use_cache: bool = False, loop_override: bool | None = None):
    """
    Handles the 'RUN_TRAJECTORY' command. It loads a trajectory definition
    from a JSON file, plans all the constituent moves, and then starts the
    trajectory executor thread to run the full sequence.
    The `loop_override` parameter from the UI takes precedence over the setting in the file.
    """
    utils.trajectory_state["should_stop"] = False # Reset stop flag for a new trajectory run
    print(f"[Pi Trajectory] Received RUN_TRAJECTORY for '{trajectory_name}' (Use Cache: {use_cache}, Loop Override: {loop_override})")

    # --- 1. Load Trajectory Definition ---
    trajectory = _load_trajectory_by_name(trajectory_name)

    if trajectory is None:
        print(f"[Pi Trajectory] ERROR: Trajectory '{trajectory_name}' not found.")
        return

    moves = trajectory.get("moves", [])
    
    # Determine looping behavior: UI override > file setting > default false
    if loop_override is not None:
        should_loop = loop_override
    else:
        should_loop = trajectory.get("loop", False)

    # NEW: Parse orientation lock from trajectory file (unchanged from original)
    orientation_lock_euler = trajectory.get("orientation_euler_angles_deg")
    target_orientation_matrix = None
    if orientation_lock_euler:
        try:
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
            if utils.trajectory_state["should_stop"]:
                print("[Pi Plan] Stop detected – aborting trajectory planning.")
                planning_succeeded = False
                break
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
        
    # NEW: Now that we have planned_steps (from planning or cache), plan the reset move for looping if needed.
    # Explanation: This is the key fix. We calculate a smooth path from the last waypoint back to the first.
    # We use linear interpolation for position and SLERP for orientation. This prevents jerking back to the random start.
    # If planning fails, we disable looping to avoid the bug.
    # This works for both cached and non-cached plans.
    first_pose = None  # We'll set this if looping is possible
    if should_loop:
        if len(planned_steps) < 2:
            print("[Pi Trajectory] Trajectory has less than 2 steps, disabling loop.")
            should_loop = False
        else:
            # Get joints at end of first step (first waypoint)
            if planned_steps[0]['type'] != 'move':
                print("[Pi Trajectory] First step is not a move, disabling loop.")
                should_loop = False
            else:
                first_end_q = np.array(planned_steps[0]['path'][-1])
                # Get joints at end of last move step (last waypoint)
                last_end_q = None
                for step in reversed(planned_steps):
                    if step['type'] == 'move' or step['type'] == 'joint_move':
                        last_end_q = np.array(step['path'][-1])
                        break
                if last_end_q is None:
                    print("[Pi Trajectory] Could not find last move step, disabling loop.")
                    should_loop = False
                else:
                    first_pose = ik_solver.get_fk_matrix(first_end_q)
                    last_pose = ik_solver.get_fk_matrix(last_end_q)
                    if first_pose is None or last_pose is None:
                        print("[Pi Trajectory] Failed to calculate poses for loop reset, disabling loop.")
                        should_loop = False
                    else:
                        first_pos = first_pose[:3, 3]
                        first_orient = first_pose[:3, :3]
                        last_pos = last_pose[:3, 3]
                        last_orient = last_pose[:3, :3]
                        # Calculate duration based on distance and default velocity (for smooth speed)
                        dist = np.linalg.norm(first_pos - last_pos)
                        velocity = utils.DEFAULT_PROFILE_VELOCITY
                        duration_s = max(0.5, dist / velocity)
                        frequency_hz = 100  # Matches frequency used in other moves
                        num_steps = max(2, int(duration_s * frequency_hz))
                        # Linear interpolation for positions (straight line path)
                        t = np.linspace(0, 1, num_steps)
                        path_positions = [last_pos + ti * (first_pos - last_pos) for ti in t]
                        # SLERP for orientations (smooth rotation)
                        key_times = [0, 1]
                        key_rots = R.concatenate([R.from_matrix(last_orient), R.from_matrix(first_orient)])
                        slerp = Slerp(key_times, key_rots)
                        interp_rots = slerp(t)
                        orientation_matrices = [r.as_matrix() for r in interp_rots]
                        # Solve IK for the entire path (batch for efficiency)
                        joint_path_reset = ik_solver.solve_ik_path_batch(
                            path_points=path_positions,
                            initial_joint_angles=last_end_q,
                            target_orientations=orientation_matrices,
                        )
                        if joint_path_reset is None:
                            print("[Pi Trajectory] Failed to plan reset path from last to first, disabling loop to avoid jerk.")
                            should_loop = False
                        else:
                            reset_move = {'type': 'move', 'path': joint_path_reset, 'freq': frequency_hz}

    # --- 2. Execution Phase ---
    print("\n--- Trajectory Ready. Starting Execution in a background thread ---")
    
    utils.trajectory_state["is_running"] = True
    utils.trajectory_state["should_stop"] = False

    # NEW: For looping, first move to the trajectory's start point from current position.
    # Explanation: We re-plan this move every time (even for cache) to avoid jerk if starting from a different position.
    # Uses similar interpolation as the reset for smoothness.
    if should_loop and len(planned_steps) > 0:
        print("[Pi Trajectory] Looping enabled. Moving to trajectory start point first.")
        # Plan initial move from current actual position to first waypoint
        current_q = np.array(utils.current_logical_joint_angles_rad)
        current_pose = ik_solver.get_fk_matrix(current_q)
        if current_pose is None or first_pose is None:
            print("[Pi Trajectory] Failed to get current or first pose, aborting trajectory.")
            utils.trajectory_state["is_running"] = False
            return
        first_pos = first_pose[:3, 3]
        first_orient = first_pose[:3, :3]
        current_pos = current_pose[:3, 3]
        current_orient = current_pose[:3, :3]
        dist = np.linalg.norm(first_pos - current_pos)
        velocity = utils.DEFAULT_PROFILE_VELOCITY
        duration_s = max(0.5, dist / velocity)
        frequency_hz = 100
        num_steps = max(2, int(duration_s * frequency_hz))
        t = np.linspace(0, 1, num_steps)
        path_positions = [current_pos + ti * (first_pos - current_pos) for ti in t]
        key_times = [0, 1]
        key_rots = R.concatenate([R.from_matrix(current_orient), R.from_matrix(first_orient)])
        slerp = Slerp(key_times, key_rots)
        interp_rots = slerp(t)
        orientation_matrices = [r.as_matrix() for r in interp_rots]
        joint_path_initial = ik_solver.solve_ik_path_batch(
            path_points=path_positions,
            initial_joint_angles=current_q,
            target_orientations=orientation_matrices,
        )
        if joint_path_initial is None:
            print("[Pi Trajectory] Failed to plan initial move to first waypoint, aborting trajectory.")
            utils.trajectory_state["is_running"] = False
            return
        initial_freq = frequency_hz
        initial_thread = threading.Thread(
            target=trajectory_execution._open_loop_executor_thread,
            kwargs={'joint_path': joint_path_initial, 'frequency': initial_freq, 'diagnostics': False},
            daemon=True
        )
        initial_thread.start()
        # Timed join with stop check to allow interruption
        while initial_thread.is_alive():
            if utils.trajectory_state["should_stop"]:
                print("[Pi Trajectory] Stop detected during initial move – aborting.")
                break
            initial_thread.join(timeout=0.1)  # Check every 100ms

        # NEW: For looping, create a modified steps list that starts from the second step
        loop_steps = planned_steps[1:]
        # Add the newly planned reset move back to the first waypoint
        loop_steps.append(reset_move)  # Uses the smooth reset we planned, not the old initial path

        executor_thread = threading.Thread(
            target=trajectory_execution._trajectory_executor_thread,
            args=(loop_steps, should_loop)
        )
    else:
        # Non-looping case (unchanged)
        executor_thread = threading.Thread(
            target=trajectory_execution._trajectory_executor_thread,
            args=(planned_steps, should_loop)
        )
    utils.trajectory_state["thread"] = executor_thread
    executor_thread.start()
    
    print("[Pi Trajectory] Trajectory thread started. Main loop is responsive.")

def handle_stop_command():
    """
    Stops any currently running motion by setting a global flag and sending
    an immediate brake command to the servos.
    """
    print("[Controller] Received STOP command. Halting all motion.")
    # Set the flag to stop any high-level trajectory loops
    utils.trajectory_state["should_stop"] = True

    # Also send an immediate brake command to the physical servos
    # by commanding them to their current position with zero speed.
    current_angles = servo_driver.get_current_arm_state_rad(verbose=False)
    if current_angles:
        print(f"[Controller] Sending immediate brake command to current position: {np.round(current_angles, 2)}")
        # Use speed 0 and max acceleration to act as a hard stop
        servo_driver.set_servo_positions(current_angles, 0, 100)
    else:
        print("[Controller] WARNING: Could not get current position to send brake command.")


def handle_move_to_position_absolute(x: float, y: float, z: float):
    """
    Handles the 'MOVE' command (legacy).
    Performs a simple, blocking, single-point IK move to an absolute position
    with no orientation constraint.
    """
    utils.trajectory_state["should_stop"] = False # Reset stop flag on new move
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
    
    # Get the current full pose using Forward Kinematics (matrix)
    pose_mx = ik_solver.get_fk_matrix(current_angles)

    if pose_mx is not None:
        # Position rounded to 3 decimals
        pos_xyz = pose_mx[:3, 3]
        pos_rounded = [round(float(p), 3) for p in pos_xyz]
        # Orientation as Euler XYZ degrees, rounded
        euler_deg = R.from_matrix(pose_mx[:3, :3]).as_euler('xyz', degrees=True)
        euler_rounded = [round(float(e), 2) for e in euler_deg]
        
        # Round joint angles for cleaner display
        angles_rounded = [round(float(a), 4) for a in current_angles]

        pos_str = ",".join(map(str, pos_rounded))
        euler_str = ",".join(map(str, euler_rounded))
        angles_str = ",".join(map(str, angles_rounded))
        
        print(f"[Pi] Sending pose: pos={pos_str} eulerXYZdeg={euler_str}")
        print(f"[Pi] Sending joint angles: {angles_str}")

        # Extended format: CURRENT_POSE,x,y,z,roll,pitch,yaw,<angles...>
        reply_msg = f"CURRENT_POSE,{pos_str},{euler_str},{angles_str}"
        
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

def handle_get_orientation(sock: 'socket.socket', addr: tuple):
    """
    Handles the 'GET_ORIENTATION' command.
    Calculates the current end-effector orientation (as a rotation matrix) using FK
    and sends it back to the requester.
    """
    print(f"[Pi] Received GET_ORIENTATION from {addr}.")

    current_angles = utils.current_logical_joint_angles_rad

    # Get the current orientation using Forward Kinematics
    current_pose_matrix = ik_solver.get_fk_matrix(current_angles)

    if current_pose_matrix is not None:
        current_orientation = current_pose_matrix[:3, :3]
        
        # Round the orientation matrix for cleaner display
        orientation_rounded = np.round(current_orientation, 4)

        orientation_str = ",".join(map(str, orientation_rounded.flatten()))

        print(f"[Pi] Sending orientation: {orientation_str}")

        reply_msg = f"CURRENT_ORIENTATION,{orientation_str}"

        try:
            sock.sendto(reply_msg.encode("utf-8"), addr)
        except Exception as e:
            print(f"[Pi] Error sending CURRENT_ORIENTATION to {addr}: {e}")
    else:
        print("[Pi] ERROR: Could not calculate current orientation (FK failed).")
        try:
            sock.sendto("ERROR,FK_FAILED".encode("utf-8"), addr)
        except Exception as e:
            print(f"[Pi] Error sending FK_FAILED error to {addr}: {e}")

def handle_move_line(target_x: float, target_y: float, target_z: float, velocity: float, acceleration: float, closed_loop: bool = True):
    """Convenience wrapper that calls the main profiled move handler, defaulting to closed-loop."""
    utils.trajectory_state["should_stop"] = False # Reset stop flag on new move
    handle_move_profiled(
        target_x, target_y, target_z, velocity, acceleration,
        closed_loop=closed_loop,
        use_smoothing=True,
        diagnostics=False
    )

def handle_move_line_relative(dx: float, dy: float, dz: float, speed: float = 1.0, closed_loop: bool = True):
    """Convenience wrapper that calls the main profiled move handler, defaulting to closed-loop."""
    utils.trajectory_state["should_stop"] = False # Reset stop flag on new move
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
# PID Tuning API
# -----------------------------------------------------------------------------

def handle_tune_pid_joint(joint_index: int, amplitude_deg: float = 5.0, frequency_hz: int = 100, duration_s: float = 3.0, move_to_zero_first: bool = True):
    """Runs the internal PID tuner for a single logical joint (blocking)."""
    if utils.trajectory_state.get("is_running"):
        print("[PID Tune] ERROR: Motion already active. Stop current move before tuning.")
        return
    try:
        print(f"[PID Tune] Starting tuning for joint J{joint_index+1}...")
        pid_tuner.tune_internal_pid_for_joint(
            logical_joint_index=joint_index,
            amplitude_deg=amplitude_deg,
            frequency_hz=frequency_hz,
            duration_s=duration_s,
            move_to_zero_first=move_to_zero_first,
        )
        print(f"[PID Tune] Tuning complete for joint J{joint_index+1}.")
    except Exception as e:
        print(f"[PID Tune] ERROR: {e}")


def handle_tune_pid_all(amplitude_deg: float = 5.0, frequency_hz: int = 50, duration_s: float = 3.0, move_to_zero_first_each: bool = True):
    """Tunes all logical joints sequentially (blocking)."""
    for j in range(utils.NUM_LOGICAL_JOINTS):
        if utils.trajectory_state.get("is_running"):
            print("[PID Tune] Motion became active mid-run; aborting all-joint tuning.")
            return
        handle_tune_pid_joint(j, amplitude_deg, frequency_hz, duration_s, move_to_zero_first_each)

# -----------------------------------------------------------------------------
# Gripper Control
# -----------------------------------------------------------------------------
def handle_set_gripper_state(angle_deg: float, speed: int = 50, accel: int = 0):
    """
    Handles the 'SET_GRIPPER' command. Commands the gripper to a specific angle.
    
    Args:
        angle_deg (float): The target angle for the gripper in degrees.
        speed (int): The speed for the movement.
        accel (int): The acceleration for the movement.
    """
    if not utils.gripper_present:
        print("[Controller] Cannot set gripper state: Gripper is not present.")
        return

    print(f"[Controller] Setting gripper to {angle_deg} degrees.")
    
    # Convert degrees to radians for internal use and validation
    angle_rad = np.deg2rad(angle_deg)

    # Validate against gripper limits
    min_rad, max_rad = utils.GRIPPER_LIMITS_RAD
    if not (min_rad <= angle_rad <= max_rad):
        print(f"[Controller] ERROR: Gripper angle {angle_deg}° is outside limits "
              f"({np.rad2deg(min_rad):.1f}° to {np.rad2deg(max_rad):.1f}°).")
        return

    # Use the existing single-servo write function
    servo_driver.set_single_servo_position_rads(
        servo_id=utils.SERVO_ID_GRIPPER,
        position_rad=angle_rad,
        speed=speed,
        accel=accel
    )
    # Update global state
    utils.current_gripper_angle_rad = angle_rad


def handle_get_gripper_state(sock: 'socket.socket', addr: tuple):
    """
    Handles the 'GET_GRIPPER_STATE' command. Reads the gripper's current
    angle and sends it back to the client.
    """
    if not utils.gripper_present:
        print("[Controller] Cannot get gripper state: Gripper is not present.")
        try:
            sock.sendto("ERROR,GRIPPER_NOT_PRESENT".encode("utf-8"), addr)
        except Exception as e:
            print(f"[Controller] Error sending GRIPPER_NOT_PRESENT error to {addr}: {e}")
        return

    print(f"[Controller] Received GET_GRIPPER_STATE from {addr}.")
    
    # Read the raw position from the servo
    raw_pos = servo_protocol.read_servo_position(utils.SERVO_ID_GRIPPER)
    
    if raw_pos is not None:
        # Convert raw position to angle in degrees
        # This requires finding the correct config index for the gripper
        try:
            gripper_config_index = utils.SERVO_IDS.index(utils.SERVO_ID_GRIPPER)
            angle_rad = servo_driver.raw_to_angle_rad(raw_pos, gripper_config_index)
            angle_deg = np.rad2deg(angle_rad)
            
            # Update global state as well
            utils.current_gripper_angle_rad = angle_rad

            reply = f"GRIPPER_STATE,{angle_deg:.2f},{raw_pos}"
            print(f"[Controller] Sending gripper state: {reply}")
            sock.sendto(reply.encode("utf-8"), addr)
        except ValueError:
            print("[Controller] ERROR: Gripper servo ID not found in SERVO_IDS list.")
            sock.sendto("ERROR,GRIPPER_ID_NOT_CONFIGURED".encode("utf-8"), addr)
        except Exception as e:
            print(f"[Controller] ERROR: Could not convert raw position to angle: {e}")
            sock.sendto(f"ERROR,CONVERSION_FAILED".encode("utf-8"), addr)
    else:
        print("[Controller] ERROR: Failed to read gripper position.")
        sock.sendto("ERROR,READ_FAILED".encode("utf-8"), addr)


# -----------------------------------------------------------------------------
# Real-time Cartesian Jogging
# -----------------------------------------------------------------------------

JOG_CONTROL_FREQUENCY_HZ = 25
JOG_VELOCITY_TIMEOUT_S = 0.5  # If no command received in this time, stop
MAX_JOG_LINEAR_M_S = 0.2      # Safety cap per-axis
MAX_JOG_ANGULAR_DEG_S = 180.0 # Safety cap per-axis
MAX_GRIPPER_JOG_DEG_S = 90.0 # Safety cap for gripper rotation rate

def _jog_controller_thread():
    """
    This is the heart of the real-time jogging feature. It runs in a tight loop,
    continuously calculating and commanding small IK movements based on the
    latest velocity commands stored in the global trajectory_state.
    """
    print("[Jog] Jog controller thread started.")
    
    # Get initial state
    q_current = servo_driver.get_current_arm_state_rad(verbose=False)
    
    last_loop_time = time.monotonic()
    
    last_status_log_time = time.monotonic()
    timeout_zero_logged = False

    while utils.trajectory_state.get("is_jogging"):
        loop_start_time = time.monotonic()
        dt = loop_start_time - last_loop_time
        last_loop_time = loop_start_time

        # --- Safety Timeout ---
        # If we haven't received a velocity command recently, set velocities to zero.
        time_since_last_cmd = time.monotonic() - utils.trajectory_state["last_jog_command_time"]
        if time_since_last_cmd > JOG_VELOCITY_TIMEOUT_S:
            utils.trajectory_state["jog_velocities"] = np.zeros(6, dtype=float)
            if not timeout_zero_logged:
                print(f"[Jog] Timeout {time_since_last_cmd:.3f}s > {JOG_VELOCITY_TIMEOUT_S:.2f}s; zeroing jog velocities.")
                timeout_zero_logged = True
        else:
            timeout_zero_logged = False

        # 1. Get current pose from FK
        current_pose_matrix = ik_solver.get_fk_matrix(q_current)
        if current_pose_matrix is None:
            print("[Jog] ERROR: FK failed during jog loop. Stopping.")
            break
        
        current_position = current_pose_matrix[:3, 3]
        current_orientation = current_pose_matrix[:3, :3]
        
        # 2. Get target velocities from global state (respect deadman gate)
        velocities = utils.trajectory_state["jog_velocities"]
        if not utils.trajectory_state.get("jog_deadman", False):
            # If deadman not held, force zero velocities (gripper included)
            if utils.trajectory_state.get("jog_debug", False):
                print("[Jog] Deadman not held → zeroing velocities.")
            velocities = np.zeros(6, dtype=float)
            utils.trajectory_state["jog_gripper_velocity_deg_s"] = 0.0
        # Apply backend safety caps component-wise
        linear_vel = np.clip(velocities[:3], -MAX_JOG_LINEAR_M_S, MAX_JOG_LINEAR_M_S)
        angular_deg_s = np.clip(velocities[3:], -MAX_JOG_ANGULAR_DEG_S, MAX_JOG_ANGULAR_DEG_S)
        angular_vel_rad_s = np.deg2rad(angular_deg_s) # Convert RPY rates to radians
        
        # 3. Calculate target pose for this time step
        # Integrate linear velocity to get new position
        target_position = current_position + linear_vel * dt
        
        # Integrate angular velocity to get new orientation
        # Create a small rotation vector from angular velocity and time step
        rotation_vector = angular_vel_rad_s * dt
        # Convert the small rotation vector to a rotation matrix
        delta_rotation = R.from_rotvec(rotation_vector).as_matrix()
        # Apply the small rotation to the current orientation
        target_orientation = delta_rotation @ current_orientation

        # 4. Solve IK for the new target pose
        q_target = ik_solver.solve_ik(
            target_position=target_position,
            target_orientation_matrix=target_orientation,
            initial_joint_angles=q_current
        )
        
        if q_target is not None:
            # 5. Enforce logical joint limits before commanding
            try:
                q_arr = np.array(q_target, dtype=float)
                limits = np.array(utils.LOGICAL_JOINT_LIMITS_RAD, dtype=float)
                mins = limits[:, 0]
                maxs = limits[:, 1]
                q_clamped = np.clip(q_arr, mins, maxs)
                if not np.allclose(q_arr, q_clamped, atol=1e-6):
                    clamped_idx = np.where(np.abs(q_arr - q_clamped) > 1e-6)[0].tolist()
                    print(f"[Jog] NOTE: IK target clamped at joints: {clamped_idx}")
                # 6. Command servos to the clamped angles. High speed, zero accel for responsiveness.
                servo_driver.set_servo_positions(q_clamped, 800, 0)
                q_current = q_clamped # Update our state for the next iteration's IK
            except Exception as e:
                print(f"[Jog] WARNING: Failed to clamp/apply joint limits: {e}")
        else:
            # If IK fails, we don't command anything and just try again next cycle.
            # This can happen if the target is unreachable.
            print("[Jog] WARNING: IK solution not found for step.")

        # --- Maintain loop frequency ---
        loop_duration = time.monotonic() - loop_start_time
        sleep_time = (1.0 / JOG_CONTROL_FREQUENCY_HZ) - loop_duration
        if sleep_time > 0:
            time.sleep(sleep_time)

        # 6. Update gripper if present, integrating jog velocity with safety caps
        try:
            if utils.gripper_present:
                rate_deg_s = float(utils.trajectory_state.get("jog_gripper_velocity_deg_s", 0.0))
                # Apply backend cap
                rate_deg_s = float(np.clip(rate_deg_s, -MAX_GRIPPER_JOG_DEG_S, MAX_GRIPPER_JOG_DEG_S))
                if abs(rate_deg_s) > 1e-3:
                    current_deg = float(np.rad2deg(utils.current_gripper_angle_rad))
                    target_deg = current_deg + rate_deg_s * dt
                    # Clamp to physical limits
                    min_rad, max_rad = utils.GRIPPER_LIMITS_RAD
                    target_rad_unclamped = float(np.deg2rad(target_deg))
                    target_rad = float(np.clip(target_rad_unclamped, min_rad, max_rad))
                    if abs(target_rad - target_rad_unclamped) > 1e-6:
                        print("[Jog] NOTE: Gripper target clamped to limits.")
                    # Choose a reasonable speed scaling from requested rate
                    speed_scaled = max(100, min(800, int(abs(rate_deg_s) * 4 + 100)))
                    servo_driver.set_single_servo_position_rads(
                        servo_id=utils.SERVO_ID_GRIPPER,
                        position_rad=target_rad,
                        speed=speed_scaled,
                        accel=0,
                    )
                    utils.current_gripper_angle_rad = target_rad
        except Exception as e:
            print(f"[Jog] WARNING: Gripper jog update failed: {e}")

        # Periodic status log (every ~0.5 s)
        now = time.monotonic()
        if now - last_status_log_time > 0.5:
            if utils.trajectory_state.get("jog_debug", False):
                print(f"[Jog] dt={dt*1000:.1f}ms, v_lin={np.round(linear_vel,3)}, v_ang(deg/s)={np.round(angular_deg_s,1)}")
            last_status_log_time = now

    print("[Jog] Jog controller thread stopped.")
    # Ensure the thread reference is cleared upon exit
    if utils.trajectory_state.get("thread") is threading.current_thread():
        utils.trajectory_state["thread"] = None


def handle_jog_start():
    """Starts the real-time jogging mode."""
    if utils.trajectory_state.get("is_running") or utils.trajectory_state.get("is_jogging"):
        print("[Jog] ERROR: Another motion is already active. Cannot start jog mode.")
        return

    print("[Jog] Starting jog mode...")
    utils.trajectory_state["is_jogging"] = True
    utils.trajectory_state["last_jog_command_time"] = time.monotonic()
    utils.trajectory_state["jog_velocities"] = np.zeros(6, dtype=float)
    utils.trajectory_state["jog_gripper_velocity_deg_s"] = 0.0

    jog_thread = threading.Thread(target=_jog_controller_thread, daemon=True)
    utils.trajectory_state["thread"] = jog_thread
    jog_thread.start()


def handle_jog_stop():
    """Stops the real-time jogging mode."""
    print("[Jog] Stopping jog mode...")
    utils.trajectory_state["is_jogging"] = False # Signal the thread to exit
    utils.trajectory_state["jog_gripper_velocity_deg_s"] = 0.0

    # Give the thread a moment to stop
    thread = utils.trajectory_state.get("thread")
    if thread and thread.is_alive():
        thread.join(timeout=0.5)

    # Hard stop the servos as a final safety measure
    current_angles = servo_driver.get_current_arm_state_rad(verbose=False)
    if current_angles:
        servo_driver.set_servo_positions(current_angles, 0, 100)
    
    print("[Jog] Jog mode stopped.")

def handle_set_jog_velocity(vx, vy, vz, v_roll, v_pitch, v_yaw):
    """
    Updates the target velocities for the active jogging session.
    This is expected to be called at a high frequency by the client.
    """
    if not utils.trajectory_state.get("is_jogging"):
        return # Ignore if not in jog mode

    utils.trajectory_state["jog_velocities"] = np.array([vx, vy, vz, v_roll, v_pitch, v_yaw], dtype=float)
    utils.trajectory_state["last_jog_command_time"] = time.monotonic()
    try:
        # Lightweight log when non-zero or on significant change
        v = utils.trajectory_state["jog_velocities"]
        if utils.trajectory_state.get("jog_debug", False) and np.any(np.abs(v) > 1e-6):
            print(f"[Jog] Vel update: lin(m/s)={np.round(v[:3],3)}, ang(deg/s)={np.round(v[3:],1)}")
    except Exception:
        pass


def handle_set_jog_deadman(enabled: bool):
    """Sets the jog deadman gate. When False, all jog rates are forced to zero."""
    utils.trajectory_state["jog_deadman"] = bool(enabled)
    # Touch the timestamp so timeout doesn't immediately zero after engage
    utils.trajectory_state["last_jog_command_time"] = time.monotonic()
    if not enabled:
        utils.trajectory_state["jog_velocities"] = np.zeros(6, dtype=float)
        utils.trajectory_state["jog_gripper_velocity_deg_s"] = 0.0
    if utils.trajectory_state.get("jog_debug", False):
        print(f"[Jog] Deadman set to {enabled}")


def handle_set_jog_debug(enabled: bool):
    """Enables/disables verbose jog logging."""
    utils.trajectory_state["jog_debug"] = bool(enabled)
    print(f"[Jog] Debug logging set to {enabled}")


def handle_set_gripper_jog_velocity(rate_deg_s: float):
    """
    Sets the gripper jogging angular rate in degrees/second. Takes effect when
    jog mode is active. Backend safety caps apply.
    """
    if not utils.trajectory_state.get("is_jogging"):
        return
    try:
        utils.trajectory_state["jog_gripper_velocity_deg_s"] = float(rate_deg_s)
        utils.trajectory_state["last_jog_command_time"] = time.monotonic()
    except Exception:
        pass


# -----------------------------------------------------------------------------
# Recording subsystem: PLAN_TRAJECTORY / REC_POS / END_TRAJECTORY
# -----------------------------------------------------------------------------

# Holds intermediate state while a user is interactively recording a trajectory.
_recording_state = {
    "is_recording": False,
    "points": [],  # list of dicts: {"position": [...], "orientation_euler_deg": [...]}
    "start_time": None,
}

# Project root (three levels up from this file): .../GradientOS
_PROJECT_ROOT_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", ".."))
# Recorded trajectories live at the project root
RECORDED_TRAJ_DIR = os.path.join(_PROJECT_ROOT_DIR, "recorded_trajectories")

# Well-known filename for the most recent PLAN_TRAJECTORY_POINTS preview
PLANNED_PREVIEW_NAME = "__planner_preview__"
PLANNED_PREVIEW_FILENAME = f"{PLANNED_PREVIEW_NAME}.json"


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


def handle_plan_trajectory_points(points, sock, addr):
    """
    Plan a Cartesian trajectory for a list of way-points and return the joint-space path
    without executing it. The resulting trajectory is written to the recorded_trajectories
    directory under a well-known name so it can be executed with RUN_TRAJECTORY.
    """
    if len(points) == 0:
        print("[Pi Trajectory] ERROR: PLAN_TRAJECTORY_POINTS requires at least one waypoint.")
        try:
            sock.sendto("ERROR,PLAN_TRAJECTORY_POINTS,NO_POINTS".encode("utf-8"), addr)
        except Exception as e:
            print(f"[Pi Trajectory] WARNING: Failed to send PLAN_TRAJECTORY_POINTS error response: {e}")
        return

    current_q = np.array(utils.current_logical_joint_angles_rad, dtype=float)
    planned_steps = []
    waypoint_results = []
    cartesian_samples = []
    planning_succeeded = True
    sample_stride = 5  # Down-sample cartesian samples to keep UDP payload modest

    print(f"[Pi Trajectory] Planning {len(points)} waypoint(s) from current state via PLAN_TRAJECTORY_POINTS.")

    for idx, waypoint in enumerate(points, start=1):
        target_pos = np.array(waypoint, dtype=float)
        t_start = time.monotonic()
        joint_path = trajectory_execution._plan_linear_move(
            current_q,
            target_pos,
            utils.DEFAULT_PROFILE_VELOCITY,
            utils.DEFAULT_PROFILE_ACCELERATION,
            100,
            True,
            forced_orientation=None,
        )
        if not joint_path:
            print(f"[Pi Trajectory] ERROR: Failed to plan waypoint #{idx} -> {np.round(target_pos, 4)}.")
            planning_succeeded = False
            break

        planned_steps.append({
            "type": "move",
            "path": joint_path,
            "freq": 100,
        })

        # Gather cartesian samples along the path for visualization.
        for sample_index, joint_sample in enumerate(joint_path):
            if sample_index % sample_stride != 0 and sample_index != len(joint_path) - 1:
                continue
            pose_matrix = ik_solver.get_fk_matrix(np.array(joint_sample))
            if pose_matrix is None:
                continue
            position = pose_matrix[:3, 3]
            cartesian_samples.append([round(float(position[0]), 4), round(float(position[1]), 4), round(float(position[2]), 4)])

        final_pose = ik_solver.get_fk_matrix(np.array(joint_path[-1]))
        if final_pose is not None:
            position = final_pose[:3, 3].tolist()
            orient_deg = R.from_matrix(final_pose[:3, :3]).as_euler('xyz', degrees=True).tolist()
            waypoint_results.append({
                "position": [round(p, 4) for p in position],
                "orientation_euler_deg": [round(o, 2) for o in orient_deg],
            })
        else:
            waypoint_results.append({
                "position": [round(float(val), 4) for val in waypoint],
                "orientation_euler_deg": None,
            })

        current_q = np.array(joint_path[-1])
        t_end = time.monotonic()
        print(f"[Pi Trajectory] Planned waypoint #{idx} -> {np.round(target_pos, 4)} in {(t_end - t_start) * 1000:.2f} ms")

    if not planning_succeeded or len(planned_steps) == 0:
        try:
            sock.sendto("ERROR,PLAN_TRAJECTORY_POINTS,PLANNING_FAILED".encode("utf-8"), addr)
        except Exception as e:
            print(f"[Pi Trajectory] WARNING: Failed to send PLAN_TRAJECTORY_POINTS failure response: {e}")
        return

    # Build recorded trajectory representation mirroring handle_end_trajectory().
    moves = []
    for i, waypoint_data in enumerate(waypoint_results):
        move = {
            "command": "move_absolute",
            "vector": waypoint_data["position"],
        }
        if waypoint_data["orientation_euler_deg"]:
            move["orientation_euler_deg"] = waypoint_data["orientation_euler_deg"]
        moves.append(move)
        if i < len(waypoint_results) - 1:
            moves.append({"command": "pause", "duration": 1.0})

    traj_dict = {
        "description": f"Planned on {datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S')} via PLAN_TRAJECTORY_POINTS",
        "loop": False,
        "orientation_euler_angles_deg": None,
        "moves": moves,
    }

    _ensure_record_dir_exists()
    preview_path = os.path.join(RECORDED_TRAJ_DIR, PLANNED_PREVIEW_FILENAME)
    try:
        with open(preview_path, "w") as f:
            json.dump(traj_dict, f, indent=2)
        print(f"[Pi Trajectory] Preview trajectory saved to {preview_path}")
    except Exception as e:
        print(f"[Pi Trajectory] WARNING: Failed to persist planner preview to {preview_path}: {e}")

    payload = {
        "name": PLANNED_PREVIEW_NAME,
        "steps": utils._convert_numpy_to_list(planned_steps),
        "trajectory": traj_dict,
        "cartesian_path": cartesian_samples,
        "waypoints": [item["position"] for item in waypoint_results],
        "file_path": preview_path,
    }
    message = "PLANNED_TRAJECTORY_POINTS," + json.dumps(payload)
    try:
        sock.sendto(message.encode("utf-8"), addr)
    except Exception as e:
        print(f"[Pi Trajectory] WARNING: Failed to send PLAN_TRAJECTORY_POINTS result: {e}")


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
