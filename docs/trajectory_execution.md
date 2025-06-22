## `trajectory_execution.py` - Path Planning & Execution

**Primary Responsibility:** To translate high-level motion goals into fully planned, time-parameterized, and executable joint-space trajectories, and to run the real-time control loop that executes these trajectories accurately.

### File Description

This module is the heart of the motion control system. It embodies the "Plan then Execute" strategy. It is responsible for all the complex calculations that turn a simple command like "move here" into a smooth, precise, and error-corrected physical motion. It does not deal with any hardware-specific communication, but rather orchestrates the `ik_solver` and the `servo_driver` to achieve its goals.

---

### Core Concepts

#### 1. Path Planning (`_plan_...` functions)

The planning phase is responsible for creating a complete, point-by-point recipe for a move before the robot moves at all.

*   **`_plan_high_fidelity_trajectory`**: This is the primary planning function.
    1.  It receives a dense list of Cartesian points from a function like `trajectory_planner.generate_trapezoidal_profile`.
    2.  It calls `ik_solver.solve_ik_path_batch` to solve the Inverse Kinematics for **every single point** in the path in a single, highly efficient operation. This eliminates the path deviation errors seen in older, interpolated methods.
    3.  The resulting dense joint-space path is then passed to `_unwrap_joint_trajectory` to correct for any `2*pi` discontinuities (e.g., a joint flipping from +180 to -180 degrees).
    4.  Finally, a Savitzky-Golay filter (`savgol_filter`) is applied to smooth out any small jitters in the path, resulting in a mechanically smooth motion.

*   **`_plan_linear_move` / `_plan_smooth_move`**: These are helper functions that generate the initial Cartesian path for a simple straight-line move and then pass it to `_plan_high_fidelity_trajectory` to do the actual joint-space planning.

#### 2. Path Execution (`_closed_loop_executor_thread`)

Once a path is planned, it is handed to the executor thread. This thread is responsible for making the robot follow the planned path accurately in real-time. It runs a high-frequency control loop (e.g., 200 Hz) that is independent of the main command loop.

The logic for each cycle of the control loop is as follows:

```mermaid
graph TD
    A[Start Loop Cycle] --> B{Calculate Target<br/>Where should the joints be at this exact millisecond?};
    B --> C{Read Actual<br/>Get all current servo positions via fast Sync Read};
    C --> D{Calculate Error<br/>Error = Target Position - Actual Position};
    D --> E{Calculate Correction<br/>Command = Target Position + (Kp * Error)};
    E --> F{Actuate<br/>Send new commanded positions via Sync Write};
    F --> G{Time<br/>Sleep precisely to maintain loop frequency};
    G --> A;
```

*   **Calculate Target:** Based on the loop frequency and the number of points in the plan, the executor determines which set of joint angles is the target for the current time step.
*   **Read Actual:** It uses `servo_protocol.sync_read_positions` to get the real-world position of all servos in one efficient command.
*   **Calculate Error:** It compares the target angles to the actual angles to find the tracking error for each joint.
*   **Calculate Correction:** It uses a Proportional (P) control law. The new commanded position is the original target, nudged slightly by the error multiplied by a gain (`CORRECTION_KP_GAIN`). If the arm is lagging, the command will be slightly ahead of the path; if it's overshooting, the command will be slightly behind. This forces the arm to stay on the planned trajectory.
*   **Actuate:** The newly calculated position commands are sent to all servos simultaneously using `servo_protocol.sync_write_goal_pos_speed_accel`.
*   **Time:** The loop sleeps for the exact amount of time needed to maintain the target frequency, ensuring the move happens at the correct speed.

#### 3. Trajectory Executor (`_trajectory_executor_thread`)

This is a higher-level executor designed to run complex sequences of moves defined in `trajectories.json`. It works by:
1.  Taking a list of pre-planned steps (where each step might be a joint path, a pause, etc.).
2.  Iterating through the steps.
3.  Calling the appropriate underlying executor for each step (e.g., the `_closed_loop_executor_thread` for a 'move' step, or `time.sleep` for a 'pause' step).
4.  Optionally looping the entire sequence. 