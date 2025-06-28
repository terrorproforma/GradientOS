## `command_api.py` - UDP Command Reference

**Primary Responsibility:** To provide a clear and robust API for controlling the robot arm via UDP commands. This module acts as the bridge between the network-facing `run_controller.py` and the internal motion logic in other modules.

### File Description

This module contains a `handle_...` function for every high-level action the robot can perform. These handlers are responsible for:
1.  Validating and parsing the parameters for their respective commands.
2.  Orchestrating calls to other modules (`trajectory_execution`, `servo_driver`, `ik_solver`) to execute the command.
3.  Managing the robot's state via the `utils.py` module (e.g., starting and stopping trajectory threads).

---

## Complete Command List

### Motion Commands

#### `MOVE_LINE`
-   **Syntax:** `MOVE_LINE,x,y,z,[velocity],[acceleration]`
-   **Description:** Plans and executes a high-precision, closed-loop, straight-line move to an absolute Cartesian position. The tool's orientation is locked to match its orientation at the start of the move. This command is **non-blocking**.
-   **Parameters:**
    -   `x, y, z`: (float, required) The absolute target coordinates in meters.
    -   `velocity`: (float, optional) The maximum velocity for the move in meters/sec. Defaults to `DEFAULT_PROFILE_VELOCITY`.
    -   `acceleration`: (float, optional) The acceleration and deceleration for the move in meters/sec^2. Defaults to `DEFAULT_PROFILE_ACCELERATION`.

#### `MOVE_LINE_RELATIVE`
-   **Syntax:** `MOVE_LINE_RELATIVE,dx,dy,dz,[speed_multiplier]`
-   **Description:** Plans and executes a high-precision, closed-loop, straight-line move relative to the robot's current position. The tool's orientation is locked. This command is **non-blocking**.
-   **Parameters:**
    -   `dx, dy, dz`: (float, required) The relative distance to move in meters.
    -   `speed_multiplier`: (float, optional) A multiplier for the default velocity and acceleration. Defaults to `1.0`.

#### `RUN_TRAJECTORY`
-   **Syntax:** `RUN_TRAJECTORY,name,[use_cache]`
-   **Description:** Loads a pre-defined sequence of moves from `trajectories.json` and executes it. This command is **non-blocking**.
-   **Parameters:**
    -   `name`: (string, required) The name of the trajectory to run (must be a key in `trajectories.json`).
    -   `use_cache`: (boolean, optional) If `true`, loads a pre-planned path from the `trajectory_cache` directory. If `false` or omitted, plans the trajectory from scratch.

#### `TRANSLATE`
-   **Syntax:** `TRANSLATE,dx,dy,dz`
-   **Description:** A simple, un-profiled relative move that keeps the tool's orientation locked. This is a **blocking**, single-point IK move and is less smooth than `MOVE_LINE_RELATIVE`.
-   **Parameters:**
    -   `dx, dy, dz`: (float, required) The relative distance to move in meters.

#### `ROTATE`
-   **Syntax:** `ROTATE,axis,degrees`
-   **Description:** A simple, un-profiled rotation around one of the base frame's axes, keeping the tool's position constant. This is a **blocking**, single-point IK move.
-   **Parameters:**
    -   `axis`: (char, required) The axis to rotate around. Must be `'x'`, `'y'`, or `'z'`.
    -   `degrees`: (float, required) The angle to rotate in degrees.

#### `SET_ORIENTATION`
-   **Syntax (basic):** `SET_ORIENTATION,roll,pitch,yaw`
-   **Syntax (advanced):** `SET_ORIENTATION,roll,pitch,yaw,[duration_s],[closed_loop]`
-   **Description:** Smoothly re-orients the tool tip to the given absolute Euler angles **while strictly maintaining its Cartesian position** throughout the motion.  Internally the controller interpolates the rotation, solves IK for every step (to enforce the position lock), and plays the trajectory at high frequency.
-   **Parameters:**
    -   `roll, pitch, yaw` (float, required): Target orientation in degrees, XYZ Euler order.
    -   `duration_s` (float, optional): Desired motion duration in seconds (≥ 0.1 s). Defaults to `1.0` for a gentle re-orientation.
    -   `closed_loop` (bool, optional): If `true`, executes in 400 Hz closed-loop mode for maximum accuracy; otherwise runs open-loop at 1300 Hz (default).
-   **Blocking:** Yes — the command returns only after the orientation move has finished.

##### Effect of `duration_s`

| duration_s | Execution Mode | Frequency | Interpolation Steps | Feel of Motion | Typical Use-Case |
|------------|----------------|-----------|---------------------|----------------|------------------|
| **0.25**   | Open-loop      | 1300 Hz   | 325                 | Very brisk     | Small touch-ups, dynamic demos |
| **0.50**   | Open-loop      | 1300 Hz   | 650                 | Fast           | Quick pick-and-place |
| **1.00**   | Open-loop      | 1300 Hz   | 1300                | Smooth         | Default, safe around people |
| **2.00**   | Open-loop      | 1300 Hz   | 2600                | Slow & gentle  | Precision assembly, filming |
| **0.25**   | Closed-loop    | 400 Hz    | 100                 | Quick, precise | Rapid calibration |
| **0.50**   | Closed-loop    | 400 Hz    | 200                 | Moderate       | High-accuracy pick-and-place |
| **1.00**   | Closed-loop    | 400 Hz    | 400                 | Very smooth    | Fine alignment tasks |
| **2.00**   | Closed-loop    | 400 Hz    | 800                 | Ultra smooth   | Slow-mo camera shots, teaching |

##### Command Examples

```text
# Default (1 s, open-loop)
SET_ORIENTATION,0,30,0

# Twice as slow (2 s, open-loop)
SET_ORIENTATION,0,30,0,2

# Very fast (0.25 s, open-loop)
SET_ORIENTATION,0,30,0,0.25

# Smooth and high-accuracy (1.5 s, closed-loop)
SET_ORIENTATION,0,30,0,1.5,true
```

### State & Utility Commands

#### `STOP`
-   **Syntax:** `STOP`
-   **Description:** Immediately and safely halts any running motion (`MOVE_LINE`, `RUN_TRAJECTORY`, etc.). This is the highest priority command.

#### `WAIT_FOR_IDLE`
-   **Syntax:** `WAIT_FOR_IDLE`
-   **Description:** This is a **blocking** command that does nothing until the currently running move or trajectory has finished. It is used for sequencing moves in client-side scripts.

#### `GET_POSITION`
-   **Syntax:** `GET_POSITION`
-   **Description:** Requests the current Cartesian position of the tool tip. The controller will reply with a UDP packet in the format `"CURRENT_POSITION,x,y,z"`.

### Calibration Commands

#### `CALIBRATE`
-   **Syntax:** `CALIBRATE,id`
-   **Description:** Enters calibration mode for a specific servo. While active, the controller will continuously stream the raw position of that servo back to the client in the format `"CALIB_DATA,id,position"`. Sending any other command will exit calibration mode.
-   **Parameters:**
    -   `id`: (int, required) The physical ID of the servo to calibrate (e.g., 10, 20, 21).

#### `SET_ZERO`
-   **Syntax:** `SET_ZERO,id`
-   **Description:** Sets the current physical position of a specific servo as its new permanent hardware zero point. This is a destructive operation written to the servo's EEPROM.
-   **Parameters:**
    -   `id`: (int, required) The physical ID of the servo to zero.

#### `GET_ALL_POSITIONS`
-   **Syntax:** `GET_ALL_POSITIONS`
-   **Description:** Requests the current raw position (0-4095) of all physical servos. The controller replies with a UDP packet in the format `"ALL_POS_DATA,id1,pos1,id2,pos2,..."`.

#### `RESET_OFFSETS`
-   **Syntax:** `RESET_OFFSETS`
-   **Description:** A utility command to reset the hardware `Position Correction` register on **all servos** back to `0`. This is useful for clearing out old calibration data and starting from a clean slate. This is a destructive operation written to the servos' EEPROM. 