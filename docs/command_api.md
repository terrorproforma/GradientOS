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
-   **Syntax:** `MOVE_LINE,x,y,z,[velocity],[acceleration],[closed_loop]`
-   **Description:** Plans and executes a straight-line move to an absolute Cartesian position while locking the tool orientation to its starting orientation. This command is **non-blocking**.
-   **Parameters:**
    -   `x, y, z`: (float, required) The absolute target coordinates in meters.
    -   `velocity`: (float, optional) The maximum velocity for the move in meters/sec. Defaults to `DEFAULT_PROFILE_VELOCITY`.
    -   `acceleration`: (float, optional) The acceleration and deceleration for the move in meters/sec^2. Defaults to `DEFAULT_PROFILE_ACCELERATION`.
    -   `closed_loop`: (bool, optional) Compatibility flag. On non-RT backends it still selects the Python closed-loop executor. On EtherCAT RTCore scheduled motion, requests are coerced onto the RTCore queued path instead of reactivating Python-timed closed-loop execution.

#### `MOVE_LINE_RELATIVE`
-   **Syntax:** `MOVE_LINE_RELATIVE,dx,dy,dz,[speed_multiplier],[closed_loop]`
-   **Description:** Plans and executes a straight-line move relative to the robot's current position while locking the tool orientation. This command is **non-blocking**.
-   **Parameters:**
    -   `dx, dy, dz`: (float, required) The relative distance to move in meters.
    -   `speed_multiplier`: (float, optional) A multiplier for the default velocity and acceleration. Defaults to `1.0`.
    -   `closed_loop`: (bool, optional) Compatibility flag. On non-RT backends it still selects the Python closed-loop executor. On EtherCAT RTCore scheduled motion, requests are coerced onto the RTCore queued path instead of reactivating Python-timed closed-loop execution.

#### `RUN_TRAJECTORY`
-   **Syntax:** `RUN_TRAJECTORY,name,[use_cache]`
-   **Description:** Loads a pre-defined sequence of moves from `trajectories.json` and executes it. This command is **non-blocking**.
-   **Parameters:**
    -   `name`: (string, required) The name of the trajectory to run (must be a key in `trajectories.json`).
    -   `use_cache`: (boolean, optional) If `true`, loads a pre-planned path from the `trajectory_cache` directory. If `false` or omitted, plans the trajectory from scratch.
-   **Reply behavior:** Returns a structured acknowledgement (ACK) payload immediately after controller acceptance. For multi-step programs, the payload now includes shared program-lifecycle metadata such as `program_state`, `program_terminal_reason`, `program_failing_step_index`, `program_completed_step_count`, `program_completed_loop_count`, and a nested `program` object. This controller-program state is distinct from RTCore segment state.

#### `PLAN_TRAJECTORY_POINTS`
-   **Syntax:** `PLAN_TRAJECTORY_POINTS,x1,y1,z1[,x2,y2,z2,...]`
-   **Description:** Plans a joint-space path through one or more Cartesian way-points and returns the serialized steps without executing them. The response is a UDP message starting with `PLANNED_TRAJECTORY_POINTS,` followed by a JSON payload containing the joint path.
-   **Notes:** Every point is treated as a `move_absolute` step using the current end-effector pose as the starting state. The computed trajectory is also written to `recorded_trajectories/__planner_preview__.json`, allowing immediate playback via `RUN_TRAJECTORY,__planner_preview__`. Each invocation overwrites the previous preview file. The command is **non-blocking**.

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
    -   `closed_loop` (bool, optional): Compatibility flag. On non-RT backends it can still request the Python closed-loop executor. On EtherCAT RTCore scheduled motion, requests are coerced onto the RTCore queued path instead of reactivating Python-timed closed-loop execution.
-   **Blocking:** Yes — the command returns only after the orientation move has finished.

##### Effect of `duration_s`

`duration_s` changes how densely the orientation path is planned and how long the move should take. On EtherCAT RTCore scheduled motion, the resulting queued sample timing is snapped onto RTCore-cycle-aligned intervals instead of using arbitrary off-cycle timestamps.

##### Command Examples

```text
# Default (1 s, open-loop)
SET_ORIENTATION,0,30,0

# Twice as slow (2 s, open-loop)
SET_ORIENTATION,0,30,0,2

# Very fast (0.25 s, open-loop)
SET_ORIENTATION,0,30,0,0.25

# Compatibility request for closed-loop semantics. On EtherCAT RTCore scheduled motion
# this is coerced back onto the RTCore queued path instead of reactivating Python timing.
SET_ORIENTATION,0,30,0,1.5,true
```

#### `JOG_SESSION_START`
-   **Syntax:** `JOG_SESSION_START,<payload_b64>`
-   **Description:** Starts a controller-owned jog session. This is the primary jog entrypoint for new clients. The payload is base64url-encoded JSON containing `owner_id`, `seq`, `deadman`, and the six Cartesian velocity fields (`vx`, `vy`, `vz`, `v_roll`, `v_pitch`, `v_yaw`). Optional fields include `gripper_velocity_deg_s`, `lease_timeout_s`, and `session_id` (normally controller-generated).
-   **Reply behavior:** Returns `ACK,JOG_SESSION_START,<payload_b64>` on success, where the payload is the controller session snapshot. Validation and ownership failures return `ERROR,JOG_SESSION_START,<payload_b64>` with structured error fields such as `code`, `message`, and optional details.

#### `JOG_SESSION_UPDATE`
-   **Syntax:** `JOG_SESSION_UPDATE,<payload_b64>`
-   **Description:** Renews and updates an active controller-owned jog session. The payload must include `session_id`, `owner_id`, `seq`, `deadman`, and the current velocity vector. Sequence numbers must increase monotonically per session.
-   **Reply behavior:** Returns a session snapshot on success. Stale sequence numbers, wrong owners, expired leases, and wrong session ids are rejected with structured controller errors.

#### `JOG_SESSION_STOP`
-   **Syntax:** `JOG_SESSION_STOP,<payload_b64>`
-   **Description:** Stops a controller-owned jog session. The payload must include `session_id`; `owner_id` and `reason` are optional but recommended so conflicts and stop reasons stay diagnosable.
-   **Reply behavior:** Returns the terminal session snapshot after stop processing.

#### `GET_JOG_SESSION_STATE`
-   **Syntax:** `GET_JOG_SESSION_STATE`
-   **Description:** Returns the current controller-owned jog session snapshot for diagnostics, including owner/session identity, lease state, sequence tracking, pause state, backend mode, and rejection counters.

#### `SET_JOG_DEBUG`
-   **Syntax:** `SET_JOG_DEBUG,flag`
-   **Description:** Turns verbose session-jog logging on/off. When enabled, the controller logs accepted session updates and periodic jog-loop status lines.

### State & Utility Commands

#### `STOP`
-   **Syntax:** `STOP`
-   **Description:** Immediately and safely halts any running motion (`MOVE_LINE`, `RUN_TRAJECTORY`, etc.). This is the highest priority command.
-   **RTCore note:** On EtherCAT RTCore, `STOP` aborts queued RTCore trajectory ownership and active jog ownership first. It intentionally does **not** send the legacy "hold current position" write through `servo_driver`, because that write becomes a one-point RTCore trajectory and can falsely relatch motion state during safe power-down.

#### `WAIT_FOR_IDLE`
-   **Syntax:** `WAIT_FOR_IDLE[,timeout_s]`
-   **Description:** This is a **blocking** compatibility helper used for sequencing moves in client-side scripts. Instead of only joining one controller thread, it polls the composite motion execution state until controller-managed motion and RTCore-backed queued execution are both quiescent.
-   **Parameters:**
    -   `timeout_s`: (float, optional) Maximum time to wait before returning a `timeout` result. Defaults to `30.0`.
-   **Reply behavior:** Returns a structured acknowledgement (ACK) payload with an explicit terminal `state` such as `completed`, `idle`, `aborted`, `faulted`, `underrun`, or `timeout`, plus wait metadata like `waited_for_motion` and `wait_timeout_s`. If the waited motion was a multi-step program, the same shared `program` / `program_*` metadata remains available so callers can see whether the terminal outcome came from operator abort, planner failure, RTCore fault, or normal completion.

#### `GET_MOTION_STATUS`
-   **Syntax:** `GET_MOTION_STATUS`
-   **Description:** Returns the controller's composite motion-execution snapshot. This includes RTCore queued/active segment state when present and, for multi-step recorded programs, a shared controller-program contract that persists terminal truth after the worker thread exits.
-   **Reply behavior:** The payload includes top-level motion fields such as `state`, `completion_scope`, `trajectory_id`, and `source_of_truth`, plus:
    -   `execution`: low-level RTCore/controller execution detail (`state_name`, `active_traj_id`, `queue_depth`, etc.)
    -   `program` and mirrored `program_*` fields: controller-program lifecycle detail (`program_state`, `program_terminal_reason`, `program_failing_step_index`, `program_completed_step_count`, `program_completed_loop_count`, active step/type, and loop iteration)
-   **Power-transition fields:** The payload also carries:
    -   `safe_for_power_transition`: derived neutral/disarmed-safe gate
    -   `power_transition_blockers`: short blocker codes
    -   `power_transition_blocker_details`: structured blocker metadata suitable for UI/operator display
    -   mirrored copies of those fields under `execution`

#### `SAFE_POWER_UP`
-   **Syntax:** `SAFE_POWER_UP`
-   **Description:** Explicitly arms/enables the active actuator backend only after verifying the runtime is neutral, fault-free, and synchronized to live feedback.
-   **RTCore note:** On EtherCAT RTCore this is the required enable path. Startup is expected to land `BUS_UP_DISARMED`, not auto-armed.
-   **Reply behavior:** Returns a structured ACK payload with fields such as `code`, `message`, `safe_for_power_transition`, and blocker details when the request is refused.

#### `SAFE_POWER_DOWN`
-   **Syntax:** `SAFE_POWER_DOWN[,wait]`
-   **Description:** Explicitly de-energizes the active actuator backend using the safe stop/disarm flow.
-   **RTCore note:** On EtherCAT RTCore this means stop jog -> abort trajectory ownership -> optionally wait for neutral -> disable axes -> disarm.
-   **Reply behavior:** Returns a structured ACK payload including `waited_for_idle`, `code`, and the current power-transition safety fields.

#### `RESET_FAULTS`
-   **Syntax:** `RESET_FAULTS[,joint]`
-   **Description:** Requests DS402/drive fault reset. On RTCore-backed EtherCAT this first neutralizes motion/disarm intent before pulsing fault reset.
-   **RTCore note:** Fault reset must leave the system `BUS_UP_DISARMED`; a separate `SAFE_POWER_UP` is still required afterward.
-   **Reply behavior:** Returns a structured ACK payload including `disarmed_after_reset=true` when the reset request is accepted.

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
-   **Description:** Requests the current raw position (0-4095) of all physical servos. The controller replies with a UDP packet in the format `
