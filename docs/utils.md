## `utils.py` - Shared Constants and Helpers

**Primary Responsibility:** To serve as a centralized repository for configuration constants, shared run-time state, and miscellaneous helper functions.

### File Description

This module doesn't perform any single high-level function. Instead, it solves a common software architecture problem: it provides a single, unambiguous source for configuration and state that all other modules in the `arm_controller` package can import and use. This prevents circular dependencies (e.g., module A needing a variable from B, while B needs a variable from A) and makes configuration changes easy, as they only need to be made in this one file.

---

### Configuration Constants

This file contains numerous blocks of constants that define the robot's physical properties and behavior.

*   **UDP Configuration:** Defines the IP, Port, and buffer size for the main command server.
*   **Servo Configuration:** Contains critical hardware numbers, such as the number of logical joints vs. physical servos and the mapping of their hardware IDs. It also defines default motion parameters like `DEFAULT_PROFILE_VELOCITY` and the `CORRECTION_KP_GAIN` for the closed-loop controller.
*   **Serial Port Configuration:** Defines the device path (`/dev/ttyAMA0`) and baud rate for the serial connection to the servos.
*   **Calibration Inputs (`LOGICAL_JOINT_MASTER_OFFSETS_RAD`):** Provides a high-level software offset for each joint to fine-tune the arm's zero position.
*   **Joint Limits (`LOGICAL_JOINT_LIMITS_RAD`, `URDF_JOINT_LIMITS`):** Defines the safe range of motion for each joint. These values are used to program the hardware limits on the servos themselves.
*   **Effective Mapping Ranges:** Defines the expected range of motion in radians for each servo, which is used in the radian-to-raw-value conversion.
*   **Servo Protocol Constants:** Defines various constant values (register addresses, default PID values, etc.) taken from the Feetech servo datasheet.
*   **Trajectory Planning & Caching:** Defines constants related to the motion planners and the location of the trajectory cache directory.

### Global State Variables

To allow different modules to share information about the robot's current state at runtime, a few global variables are managed here.

*   **`ser`**: The global `pyserial` object representing the connection to the servos. It is initialized in `servo_driver.initialize_servos()` and used by `servo_protocol.py` for all read/write operations.
*   **`trajectory_state`**: A dictionary that holds the state of the motion system, including whether a trajectory is running (`is_running`), if a stop has been requested (`should_stop`), and a reference to the running motion `thread`. This is the core of the non-blocking architecture.
*   **`current_logical_joint_angles_rad`**: A list of the six most recently commanded logical joint angles. This is used as the starting point for many IK calculations.

### Helper Functions

*   **`_is_servo_direct_mapping(...)`**: A utility that returns `True` or `False` depending on whether a specific servo is mounted in a way that its raw value increases with its angle (direct) or decreases (inverted).
*   **`_convert_numpy_to_list(...)`**: A recursive helper to convert data structures containing NumPy arrays into pure Python lists, which is necessary before saving them to a JSON file. 