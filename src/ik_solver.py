import numpy as np
from scipy.spatial.transform import Rotation as R
import os
from ikfast_solver.ikfast_wrapper import IKFastSolver

# --- Configuration ---
# Define the translation offset from the final joint (wrist) to the tool tip.
# This is a 3-element vector [x, y, z] in meters.
# For example, if your tool extends 12cm along the wrist's Z-axis:
END_EFFECTOR_OFFSET = np.array([0.180, 0.0, 0.0]) 

# --- Global IKFast Solver Initialization ---
# This creates a single, reusable instance of the solver.
# The C++ library is built only once when this module is first imported.
try:
    IK_SOLVER = IKFastSolver()
    NUM_JOINTS = IK_SOLVER.num_joints
    print(f"[IK Solver] IKFast solver initialized successfully for {NUM_JOINTS} joints.")
except (RuntimeError, OSError) as e:
    print(f"--- [IK Solver] FATAL ERROR ---")
    print(f"Failed to initialize the IKFast solver: {e}")
    IK_SOLVER = None
    NUM_JOINTS = 6 # Default fallback

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

    # Calculate distances without a loop for performance
    distances = np.linalg.norm(solutions - current_joint_angles, axis=1)
    best_solution_idx = np.argmin(distances)
            
    return solutions[best_solution_idx]

def solve_ik(target_position, target_orientation_matrix=None, initial_joint_angles=None):
    """
    Solves inverse kinematics for a given tool-tip pose using the C++ IKFast solver.
    This function compensates for the end-effector offset.

    Args:
        target_position (list or np.array): The target [x, y, z] position of the TOOL TIP.
        target_orientation_matrix (list or np.array, optional): A 9-element list or 3x3 array for the tool's
                                                              rotation matrix. If None, identity is used.
        initial_joint_angles (list or np.array, optional): The current joint angles, used to select the closest solution.

    Returns:
        list or None: The best joint angle solution in radians, or None if no solution is found.
    """
    if IK_SOLVER is None:
        print("[IK Solver] Error: Solver is not initialized.")
        return None

    if target_orientation_matrix is None:
        # Default to an identity matrix if no orientation is provided
        target_rotation = np.identity(3)
    else:
        target_rotation = np.array(target_orientation_matrix).reshape(3, 3)

    # To find the wrist pose, we transform the offset vector by the target rotation
    # and subtract it from the target tool tip position.
    rotated_offset = target_rotation.dot(END_EFFECTOR_OFFSET)
    target_wrist_position = np.array(target_position) - rotated_offset
    
    # The new wrapper takes numpy arrays directly
    target_wrist_orientation_flat = target_rotation.flatten()
    
    solutions = IK_SOLVER.solve_ik(target_wrist_position, target_wrist_orientation_flat, initial_joint_angles)

    return solutions

def get_fk_matrix(active_joint_angles):
    """
    Calculates the forward kinematics to find the 4x4 transformation matrix of the TOOL TIP.
    
    Args:
        active_joint_angles (list or np.array): The current active joint angles in radians.

    Returns:
        np.array: The 4x4 transformation matrix, or None on failure.
    """
    if IK_SOLVER is None:
        print("[IK Solver] Error: Solver is not initialized.")
        return None

    # Get the FK of the wrist from the C++ solver
    wrist_translation, wrist_rotation = IK_SOLVER.compute_fk(active_joint_angles)
    
    # Assemble the 4x4 transformation matrix for the wrist
    wrist_matrix = np.eye(4)
    wrist_matrix[:3, :3] = wrist_rotation.reshape(3, 3)
    wrist_matrix[:3, 3] = wrist_translation
    
    # Create a transformation matrix for the end-effector offset
    offset_matrix = np.eye(4)
    offset_matrix[:3, 3] = END_EFFECTOR_OFFSET
    
    # Combine the wrist matrix and the offset matrix to get the final tool tip pose
    tool_tip_matrix = wrist_matrix.dot(offset_matrix)
    
    return tool_tip_matrix

def get_fk(active_joint_angles):
    """
    Calculates the forward kinematics to find just the end-effector position.

    Args:
        active_joint_angles (list or np.array): The current active joint angles.

    Returns:
        np.array: The [x, y, z] position, or None on failure.
    """
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
    if IK_SOLVER is None:
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
    if IK_SOLVER is None:
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
    solutions = IK_SOLVER.solve_ik_path(poses_batch, start_angles_np)

    return solutions

# Note: A parallel implementation is not provided as the C++ IKFast solver
# is extremely fast, making the overhead of process creation often slower
# than sequential execution.

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