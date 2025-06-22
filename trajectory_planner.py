import numpy as np
from scipy.spatial.transform import Rotation as R

def generate_trapezoidal_profile(start_pos, end_pos, max_velocity, max_acceleration, frequency):
    """
    Generates a trapezoidal velocity profile for a straight-line move.

    This function calculates a series of points in 3D space that form a
    straight line from start_pos to end_pos. The spacing of these points
    is calculated to create a velocity profile that accelerates to a maximum
    velocity, cruises, and then decelerates to a stop.

    Args:
        start_pos (np.ndarray): The starting [x, y, z] position.
        end_pos (np.ndarray): The ending [x, y, z] position.
        max_velocity (float): The maximum velocity during the cruise phase (m/s).
        max_acceleration (float): The acceleration and deceleration rate (m/s^2).
        frequency (int): The number of points to generate per second (Hz).

    Returns:
        list[list[float]]: A list of [x, y, z] points representing the path.
    """
    if max_velocity <= 0 or max_acceleration <= 0 or frequency <= 0:
        print("[Planner] ERROR: Velocity, acceleration, and frequency must be positive.")
        return None

    time_step = 1.0 / frequency
    direction_vector = end_pos - start_pos
    total_distance = np.linalg.norm(direction_vector)

    if total_distance == 0:
        return [start_pos]

    # Normalize the direction vector
    unit_direction = direction_vector / total_distance

    # --- Calculate the properties of the motion profile ---

    # Time to accelerate to max velocity
    t_accel = max_velocity / max_acceleration
    # Distance covered during acceleration
    d_accel = 0.5 * max_acceleration * t_accel**2

    # Check if the move is long enough for a cruise phase (trapezoidal) or not (triangular)
    if total_distance < 2 * d_accel:
        # Triangular profile (accelerate, then immediately decelerate)
        is_trapezoidal = False
        t_accel = np.sqrt(total_distance / max_acceleration)
        t_cruise = 0
        t_decel = t_accel
        cruise_velocity = max_acceleration * t_accel
    else:
        # Trapezoidal profile
        is_trapezoidal = True
        d_cruise = total_distance - 2 * d_accel
        t_cruise = d_cruise / max_velocity
        t_decel = t_accel
        cruise_velocity = max_velocity

    total_time = t_accel + t_cruise + t_decel
    
    # --- Generate the trajectory points ---
    trajectory = []
    current_time = 0
    while current_time < total_time:
        if current_time <= t_accel:
            # Acceleration phase
            current_dist = 0.5 * max_acceleration * current_time**2
        elif current_time <= t_accel + t_cruise:
            # Cruise phase
            time_in_cruise = current_time - t_accel
            current_dist = d_accel + cruise_velocity * time_in_cruise
        else:
            # Deceleration phase
            time_in_decel = current_time - (t_accel + t_cruise)
            current_dist = total_distance - (0.5 * max_acceleration * (t_decel - time_in_decel)**2)
        
        # Calculate the position vector for the current point in time
        current_pos = start_pos + current_dist * unit_direction
        trajectory.append(current_pos)
        current_time += time_step
        
    # Always ensure the final point is exactly the target position
    trajectory.append(end_pos)
    
    print(f"[Planner] Trajectory generated. Profile: {'Trapezoidal' if is_trapezoidal else 'Triangular'}. "
          f"Total time: {total_time:.2f}s. Steps: {len(trajectory)}.")
          
    return trajectory

def generate_arc_trajectory(start_pos, end_pos, center_pos, max_velocity, max_acceleration, frequency):
    """
    Generates a constant-velocity trajectory along a circular arc.
    Note: This implementation does not have a trapezoidal profile; it attempts
    to maintain a constant speed throughout the arc.

    Args:
        start_pos (np.ndarray): The starting [x, y, z] position on the arc.
        end_pos (np.ndarray): The ending [x, y, z] position on the arc.
        center_pos (np.ndarray): The center point of the circle defining the arc.
        max_velocity (float): The target velocity for the move (m/s).
        max_acceleration (float): The acceleration rate (m/s^2) - currently used to calculate duration.
        frequency (int): The number of points to generate per second (Hz).

    Returns:
        list[list[float]]: A list of [x, y, z] points representing the arc path.
    """
    if max_velocity <= 0 or max_acceleration <= 0 or frequency <= 0:
        print("[Planner] ERROR: Velocity, acceleration, and frequency must be positive.")
        return None

    v_start = start_pos - center_pos
    v_end_input = end_pos - center_pos
    radius = np.linalg.norm(v_start)
    
    if np.isclose(radius, 0):
        print("[Planner] ERROR: Radius is zero because start point is the same as center.")
        return None

    # To handle floating point inaccuracies from previous moves, we ensure the
    # end point lies on the same sphere as the start point, relative to the center.
    radius_end_input = np.linalg.norm(v_end_input)
    if np.isclose(radius_end_input, 0):
        print("[Planner] ERROR: End point cannot be the same as the center point.")
        return None

    v_end = v_end_input
    if not np.isclose(radius, radius_end_input):
        print(f"[Planner] WARNING: Start/end points not equidistant from center (Start: {radius:.5f}, End: {radius_end_input:.5f}). Projecting end point onto arc.")
        v_end = v_end_input * (radius / radius_end_input)

    # Calculate the total angle of the arc
    # Use clip to avoid domain errors with acos due to floating point inaccuracies
    dot_product = np.dot(v_start, v_end) / (radius * radius)
    total_angle = np.arccos(np.clip(dot_product, -1.0, 1.0))
    
    if np.isclose(total_angle, 0):
        return [start_pos]

    total_arc_length = radius * total_angle
    
    # Use the linear trapezoidal planner on the 1D arc length
    arc_distances = generate_trapezoidal_profile(np.array([0,0,0]), np.array([total_arc_length, 0, 0]), max_velocity, max_acceleration, frequency)
    if not arc_distances:
        return None
    
    arc_distances_1d = [d[0] for d in arc_distances] # Extract the 1D distance values

    # Convert distances along the arc back to 3D points
    arc_normal = np.cross(v_start, v_end)
    if np.linalg.norm(arc_normal) == 0:
        # This case happens if start, end, and center are collinear, which is ambiguous.
        # We need an arbitrary normal perpendicular to v_start.
        if np.allclose(v_start, np.array([0, 0, radius])) or np.allclose(v_start, np.array([0, 0, -radius])):
             arc_normal = np.array([0, 1, 0]) # If start is along Z, use Y as normal
        else:
             arc_normal = np.cross(v_start, np.array([0, 0, 1])) # Otherwise, use Z-axis cross product
    
    arc_normal = arc_normal / np.linalg.norm(arc_normal)
    
    trajectory = []
    for dist in arc_distances_1d:
        angle = dist / radius
        rotation = R.from_rotvec(angle * arc_normal)
        rotated_v = rotation.apply(v_start)
        point = center_pos + rotated_v
        trajectory.append(point)

    # The print from generate_trapezoidal_profile is sufficient
    print(f"[Planner] Arc trajectory generated. Angle: {np.rad2deg(total_angle):.2f} degrees.")
    return trajectory


if __name__ == '__main__':
    # --- Test the planner ---
    start = np.array([0.0, 0.0, 0.0])
    end = np.array([0.5, 0.0, 0.0]) # Move 0.5m in X
    vel = 0.1 # m/s
    accel = 0.1 # m/s^2
    hz = 100 # 100Hz loop

    print("--- Testing Trapezoidal Profile ---")
    points = generate_trapezoidal_profile(start, end, vel, accel, hz)
    if points:
        # Optional: Print first and last few points
        print(f"Start point: {points[0]}")
        print(f"End point: {points[-1]}")
        print("...")

    print("\n--- Testing Triangular Profile (short move) ---")
    end_short = np.array([0.05, 0.0, 0.0]) # Move 5cm in X
    points_short = generate_trapezoidal_profile(start, end_short, vel, accel, hz)
    if points_short:
        print(f"Start point: {points_short[0]}")
        print(f"End point: {points_short[-1]}")
        print("...") 

    print("\n--- Testing Arc Profile ---")
    arc_start = np.array([0.3, 0.0, 0.2])
    arc_end = np.array([0.0, 0.3, 0.2])
    arc_center = np.array([0.0, 0.0, 0.2]) # 90 degree arc in the X-Y plane
    arc_points = generate_arc_trajectory(arc_start, arc_end, arc_center, vel, accel, hz)
    if arc_points:
        print(f"Arc start: {arc_points[0]}")
        print(f"Arc end: {arc_points[-1]}")
        print("...") 