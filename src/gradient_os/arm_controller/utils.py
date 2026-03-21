# Contains utility functions and constants shared across the arm_controller package. 
#
# NOTE: This file imports configuration from:
#   - robot_config.py for robot-specific settings
#   - backends/registry.py for servo-backend-specific constants
#
# The backend registry MUST be configured before accessing servo-specific values.
# This is done in run_controller.py at startup via backend_registry.set_active_backend().
#
# For new code, prefer importing directly from:
#   - robot_config.py for robot-specific settings
#   - backends/registry.py for servo-backend constants

import math
import os
import threading
import time
import uuid
from typing import Any
import numpy as np

# Import robot configuration for backward compatibility
from . import robot_config

# Import backend registry for servo-specific constants (dynamically configured at runtime)
from .backends import registry as backend_registry

# =============================================================================
# Re-export robot configuration for backward compatibility
# =============================================================================

# --- UDP Configuration ---
PI_IP = robot_config.PI_IP
UDP_PORT = robot_config.UDP_PORT
BUFFER_SIZE = robot_config.BUFFER_SIZE

# --- Kinematics & Planning ---
NUM_LOGICAL_JOINTS = robot_config.NUM_LOGICAL_JOINTS

# --- Servo Configuration ---
NUM_PHYSICAL_SERVOS = robot_config.NUM_PHYSICAL_SERVOS
SERVO_IDS = robot_config.SERVO_IDS

# IDs for the second motor on multi-servo joints
SERVO_ID_JOINT_2_SECOND = robot_config.SERVO_ID_JOINT_2_SECOND
SERVO_ID_JOINT_3_SECOND = robot_config.SERVO_ID_JOINT_3_SECOND

# ID for the gripper servo
SERVO_ID_GRIPPER = robot_config.SERVO_ID_GRIPPER

# --- Default Motion Parameters ---
DEFAULT_SERVO_SPEED = robot_config.DEFAULT_SERVO_SPEED
DEFAULT_PROFILE_VELOCITY = robot_config.DEFAULT_PROFILE_VELOCITY
DEFAULT_PROFILE_ACCELERATION = robot_config.DEFAULT_PROFILE_ACCELERATION
CORRECTION_KP_GAIN = robot_config.CORRECTION_KP_GAIN
CORRECTION_KI_GAIN = robot_config.CORRECTION_KI_GAIN
CORRECTION_INTEGRAL_CLAMP_RAD = robot_config.CORRECTION_INTEGRAL_CLAMP_RAD

# --- Serial Port Configuration ---
SERIAL_PORT = robot_config.SERIAL_PORT
BAUD_RATE = robot_config.BAUD_RATE

# --- Master Calibration Offsets ---
LOGICAL_JOINT_MASTER_OFFSETS_RAD = robot_config.LOGICAL_JOINT_MASTER_OFFSETS_RAD

# --- Joint Limits ---
LOGICAL_JOINT_LIMITS_RAD = robot_config.LOGICAL_JOINT_LIMITS_RAD
GRIPPER_LIMITS_RAD = robot_config.GRIPPER_LIMITS_RAD
URDF_JOINT_LIMITS = robot_config.URDF_JOINT_LIMITS
EFFECTIVE_MAPPING_RANGES = robot_config.EFFECTIVE_MAPPING_RANGES

# --- PID Configuration ---
DEFAULT_KP = robot_config.DEFAULT_KP
DEFAULT_KI = robot_config.DEFAULT_KI
DEFAULT_KD = robot_config.DEFAULT_KD
J1_PID_GAINS = robot_config.J1_PID_GAINS
J2_PID_GAINS = robot_config.J2_PID_GAINS
J3_PID_GAINS = robot_config.J3_PID_GAINS
J4_PID_GAINS = robot_config.J4_PID_GAINS
J5_PID_GAINS = robot_config.J5_PID_GAINS
J6_PID_GAINS = robot_config.J6_PID_GAINS
Gripper_PID_GAINS = robot_config.GRIPPER_PID_GAINS
DEFAULT_PID_GAINS = robot_config.DEFAULT_PID_GAINS

# =============================================================================
# Servo Protocol Constants (from active backend via registry)
# These are initialized to None and populated when the backend is configured.
# =============================================================================

# Encoder resolution (actuator-specific) - populated from backend registry
ENCODER_RESOLUTION = None
ENCODER_CENTER = None

# Protocol constants - populated from backend registry
SERVO_HEADER = None
SERVO_INSTRUCTION_WRITE = None
SERVO_INSTRUCTION_READ = None
SERIAL_READ_TIMEOUT = None
SERVO_ADDR_TARGET_POSITION = None
SERVO_ADDR_PRESENT_POSITION = None
SERVO_ADDR_TARGET_ACCELERATION = None
SERVO_ADDR_POSITION_CORRECTION = None
DEFAULT_SERVO_ACCELERATION_DEG_S2 = None
ACCELERATION_SCALE_FACTOR = None
SERVO_ADDR_POS_KP = None
SERVO_ADDR_POS_KI = None
SERVO_ADDR_POS_KD = None
SERVO_INSTRUCTION_PING = None
SERVO_INSTRUCTION_RESET = None
SERVO_INSTRUCTION_RESTART = None
SERVO_INSTRUCTION_CALIBRATE_MIDDLE = None
SERVO_ADDR_WRITE_LOCK = None
SERVO_ADDR_MIN_ANGLE_LIMIT = None
SERVO_ADDR_MAX_ANGLE_LIMIT = None

# Sync Write/Read Constants
SERVO_INSTRUCTION_SYNC_WRITE = None
SERVO_BROADCAST_ID = None
SYNC_WRITE_START_ADDRESS = None
SYNC_WRITE_DATA_LEN_PER_SERVO = None
SERVO_INSTRUCTION_SYNC_READ = None


def _populate_servo_constants():
    """
    Populate servo-specific constants from the active servo backend.
    Called after backend_registry.set_active_backend() in run_controller.py.
    
    These are protocol/hardware constants for the specific servo type (e.g., Feetech),
    distinct from robot-specific constants which are populated by _populate_robot_constants().
    """
    global ENCODER_RESOLUTION, ENCODER_CENTER, BAUD_RATE
    global SERVO_HEADER, SERVO_INSTRUCTION_WRITE, SERVO_INSTRUCTION_READ
    global SERIAL_READ_TIMEOUT, SERVO_ADDR_TARGET_POSITION, SERVO_ADDR_PRESENT_POSITION
    global SERVO_ADDR_TARGET_ACCELERATION, SERVO_ADDR_POSITION_CORRECTION
    global DEFAULT_SERVO_ACCELERATION_DEG_S2, ACCELERATION_SCALE_FACTOR
    global SERVO_ADDR_POS_KP, SERVO_ADDR_POS_KI, SERVO_ADDR_POS_KD
    global SERVO_INSTRUCTION_PING, SERVO_INSTRUCTION_RESET, SERVO_INSTRUCTION_RESTART
    global SERVO_INSTRUCTION_CALIBRATE_MIDDLE
    global SERVO_ADDR_WRITE_LOCK, SERVO_ADDR_MIN_ANGLE_LIMIT, SERVO_ADDR_MAX_ANGLE_LIMIT
    global SERVO_INSTRUCTION_SYNC_WRITE, SERVO_BROADCAST_ID
    global SYNC_WRITE_START_ADDRESS, SYNC_WRITE_DATA_LEN_PER_SERVO, SERVO_INSTRUCTION_SYNC_READ
    
    servo_config = backend_registry.get_config()

    # EtherCAT/RTCore backend is not a serial servo protocol. Skip populating
    # the legacy "servo protocol constants" to avoid implying they're valid.
    # (They remain None and any accidental serial-path use will fail loudly.)
    if not getattr(servo_config, "SERVO_PROTOCOL_SUPPORTED", True):
        try:
            backend_name = backend_registry.get_active_backend_name()
        except Exception:
            backend_name = "unknown"
        # Reset protocol constants to a known "unsupported" state. This prevents
        # accidental use of the legacy serial paths after switching backends.
        for _name in (
            "ENCODER_RESOLUTION",
            "ENCODER_CENTER",
            "SERVO_HEADER",
            "SERVO_INSTRUCTION_WRITE",
            "SERVO_INSTRUCTION_READ",
            "SERIAL_READ_TIMEOUT",
            "SERVO_ADDR_TARGET_POSITION",
            "SERVO_ADDR_PRESENT_POSITION",
            "SERVO_ADDR_TARGET_ACCELERATION",
            "SERVO_ADDR_POSITION_CORRECTION",
            "DEFAULT_SERVO_ACCELERATION_DEG_S2",
            "ACCELERATION_SCALE_FACTOR",
            "SERVO_ADDR_POS_KP",
            "SERVO_ADDR_POS_KI",
            "SERVO_ADDR_POS_KD",
            "SERVO_INSTRUCTION_PING",
            "SERVO_INSTRUCTION_RESET",
            "SERVO_INSTRUCTION_RESTART",
            "SERVO_INSTRUCTION_CALIBRATE_MIDDLE",
            "SERVO_ADDR_WRITE_LOCK",
            "SERVO_ADDR_MIN_ANGLE_LIMIT",
            "SERVO_ADDR_MAX_ANGLE_LIMIT",
            "SERVO_INSTRUCTION_SYNC_WRITE",
            "SERVO_BROADCAST_ID",
            "SYNC_WRITE_START_ADDRESS",
            "SYNC_WRITE_DATA_LEN_PER_SERVO",
            "SERVO_INSTRUCTION_SYNC_READ",
        ):
            globals()[_name] = None
        # Keep BAUD_RATE in a defined state for callers that still read it.
        # (It is not used by ethercat_rtcore.)
        try:
            BAUD_RATE = int(getattr(servo_config, "DEFAULT_BAUD_RATE", 0))
        except Exception:
            BAUD_RATE = 0
        print(f"[Utils] Backend '{backend_name}' does not use serial servo protocol; skipping servo constants.")
        return
    
    ENCODER_RESOLUTION = servo_config.SERVO_VALUE_MAX
    ENCODER_CENTER = servo_config.SERVO_VALUE_CENTER
    BAUD_RATE = servo_config.DEFAULT_BAUD_RATE
    SERVO_HEADER = servo_config.SERVO_HEADER
    SERVO_INSTRUCTION_WRITE = servo_config.SERVO_INSTRUCTION_WRITE
    SERVO_INSTRUCTION_READ = servo_config.SERVO_INSTRUCTION_READ
    SERIAL_READ_TIMEOUT = servo_config.SERIAL_READ_TIMEOUT
    SERVO_ADDR_TARGET_POSITION = servo_config.SERVO_ADDR_TARGET_POSITION
    SERVO_ADDR_PRESENT_POSITION = servo_config.SERVO_ADDR_PRESENT_POSITION
    SERVO_ADDR_TARGET_ACCELERATION = servo_config.SERVO_ADDR_TARGET_ACCELERATION
    SERVO_ADDR_POSITION_CORRECTION = servo_config.SERVO_ADDR_POSITION_CORRECTION
    DEFAULT_SERVO_ACCELERATION_DEG_S2 = servo_config.DEFAULT_SERVO_ACCELERATION_DEG_S2
    ACCELERATION_SCALE_FACTOR = servo_config.ACCELERATION_SCALE_FACTOR
    SERVO_ADDR_POS_KP = servo_config.SERVO_ADDR_POS_KP
    SERVO_ADDR_POS_KI = servo_config.SERVO_ADDR_POS_KI
    SERVO_ADDR_POS_KD = servo_config.SERVO_ADDR_POS_KD
    SERVO_INSTRUCTION_PING = servo_config.SERVO_INSTRUCTION_PING
    SERVO_INSTRUCTION_RESET = servo_config.SERVO_INSTRUCTION_RESET
    SERVO_INSTRUCTION_RESTART = servo_config.SERVO_INSTRUCTION_RESTART
    SERVO_INSTRUCTION_CALIBRATE_MIDDLE = servo_config.SERVO_INSTRUCTION_CALIBRATE_MIDDLE
    SERVO_ADDR_WRITE_LOCK = servo_config.SERVO_ADDR_WRITE_LOCK
    SERVO_ADDR_MIN_ANGLE_LIMIT = servo_config.SERVO_ADDR_MIN_ANGLE_LIMIT
    SERVO_ADDR_MAX_ANGLE_LIMIT = servo_config.SERVO_ADDR_MAX_ANGLE_LIMIT
    SERVO_INSTRUCTION_SYNC_WRITE = servo_config.SERVO_INSTRUCTION_SYNC_WRITE
    SERVO_BROADCAST_ID = servo_config.SERVO_BROADCAST_ID
    SYNC_WRITE_START_ADDRESS = servo_config.SYNC_WRITE_START_ADDRESS
    SYNC_WRITE_DATA_LEN_PER_SERVO = servo_config.SYNC_WRITE_DATA_LEN_PER_SERVO
    SERVO_INSTRUCTION_SYNC_READ = servo_config.SERVO_INSTRUCTION_SYNC_READ


# Keep old name as alias for backward compatibility during migration
_populate_backend_constants = _populate_servo_constants

# =============================================================================
# Trajectory Planning Configuration
# =============================================================================

# IK planning frequency (Hz) - solve IK at this rate, interpolate for execution
IK_PLANNING_FREQUENCY = 100

# Trajectory Caching
TRAJECTORY_CACHE_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", "trajectory_cache"))

# =============================================================================
# Global Runtime State
# =============================================================================

# Global state for trajectory execution
def _default_program_status() -> dict[str, Any]:
    return {
        "name": None,
        "active": False,
        "state": "idle",
        "terminal_reason": None,
        "failing_step_index": None,
        "completed_step_count": 0,
        "completed_loop_count": 0,
        "loop_enabled": False,
        "use_cache": False,
        "step_count": 0,
        "move_steps": 0,
        "pause_steps": 0,
        "joint_move_steps": 0,
        "rtcore_segments": False,
        "segment_execution_policy": "",
        "current_step_index": None,
        "current_step_type": None,
        "loop_iteration": 0,
    }


trajectory_state = {
    "is_running": False,
    "should_stop": False,
    "thread": None,
    "stop_request_reason": None,
    # Weld execution state for telemetry/UI overlays.
    "weld_active": False,
    "current_weld_type": None,
    # Diagnostics toggle (runtime), used to enable IK/executor logging and charts
    "diagnostics_enabled": False,
    # --- Real-time jogging state ---
    "is_jogging": False,
    # Separate thread handle for jogging (do not reuse "thread", which is reserved for trajectories)
    "jog_thread": None,
    # 6D velocity vector: [vx, vy, vz, v_roll, v_pitch, v_yaw]
    "jog_velocities": np.zeros(6, dtype=float),
    "last_jog_command_time": 0.0,
    # Deadman gate for jog (must be True to apply non-zero velocities)
    "jog_deadman": False,
    # Jog debug logging verbosity flag
    "jog_debug": False,
    # Explicit controller motion state
    "motion_state": "IDLE",
    # Correlation ID for command/audit tracing
    "last_correlation_id": None,
    # Persistent multi-step program lifecycle state.
    "program_status": _default_program_status(),
    # Rolling audit trail (small in-memory ring)
    "audit_log": [],
}

trajectory_state_lock = threading.RLock()

MOTION_STATE_TRANSITIONS: dict[str, set[str]] = {
    "IDLE": {"PLANNING", "EXECUTING", "FAULT", "E_STOP"},
    "PLANNING": {"IDLE", "EXECUTING", "FAULT", "E_STOP"},
    "EXECUTING": {"PAUSED", "IDLE", "FAULT", "E_STOP"},
    "PAUSED": {"EXECUTING", "IDLE", "FAULT", "E_STOP"},
    "FAULT": {"IDLE", "E_STOP"},
    "E_STOP": {"IDLE"},
}


def trajectory_state_get(key: str, default: Any = None) -> Any:
    with trajectory_state_lock:
        return trajectory_state.get(key, default)


def trajectory_state_set(key: str, value: Any) -> None:
    with trajectory_state_lock:
        trajectory_state[key] = value


def trajectory_state_update(**kwargs: Any) -> None:
    with trajectory_state_lock:
        trajectory_state.update(kwargs)


def trajectory_state_snapshot() -> dict[str, Any]:
    with trajectory_state_lock:
        return dict(trajectory_state)


def program_status_snapshot() -> dict[str, Any]:
    with trajectory_state_lock:
        current = trajectory_state.get("program_status")
        if isinstance(current, dict):
            merged = _default_program_status()
            merged.update(current)
            return merged
        return _default_program_status()


def program_status_update(**kwargs: Any) -> None:
    with trajectory_state_lock:
        current = trajectory_state.get("program_status")
        merged = _default_program_status()
        if isinstance(current, dict):
            merged.update(current)
        merged.update(kwargs)
        trajectory_state["program_status"] = merged


def program_status_reset(**overrides: Any) -> None:
    with trajectory_state_lock:
        next_status = _default_program_status()
        next_status.update(overrides)
        trajectory_state["program_status"] = next_status


def get_motion_state() -> str:
    with trajectory_state_lock:
        state = trajectory_state.get("motion_state", "IDLE")
        return str(state) if state else "IDLE"


def set_motion_state(next_state: str) -> None:
    next_state_norm = str(next_state).strip().upper()
    if next_state_norm not in MOTION_STATE_TRANSITIONS:
        raise ValueError(f"Invalid motion state '{next_state}'.")
    with trajectory_state_lock:
        current = str(trajectory_state.get("motion_state", "IDLE")).upper()
        allowed = MOTION_STATE_TRANSITIONS.get(current, set())
        if next_state_norm not in allowed and next_state_norm != current:
            raise ValueError(
                f"Illegal motion state transition {current} -> {next_state_norm}."
            )
        trajectory_state["motion_state"] = next_state_norm


def begin_correlation(command_name: str) -> str:
    correlation_id = f"{int(time.time() * 1000)}-{uuid.uuid4().hex[:8]}"
    with trajectory_state_lock:
        trajectory_state["last_correlation_id"] = correlation_id
        log = trajectory_state.setdefault("audit_log", [])
        if isinstance(log, list):
            log.append(
                {
                    "ts": time.time(),
                    "event": "command_received",
                    "command": str(command_name),
                    "correlation_id": correlation_id,
                }
            )
            if len(log) > 200:
                del log[:-200]
    return correlation_id


def append_audit(event: str, **fields: Any) -> None:
    with trajectory_state_lock:
        log = trajectory_state.setdefault("audit_log", [])
        if not isinstance(log, list):
            return
        row = {"ts": time.time(), "event": event}
        row.update(fields)
        log.append(row)
        if len(log) > 200:
            del log[:-200]

# =============================================================================
# Global Serial & State Objects
# =============================================================================

# Global serial object (managed by servo_driver.initialize_servos())
ser: 'serial.Serial | None' = None

# Global state for the arm's last known logical joint angles (in radians)
# NOTE: This is initialized as empty and populated by run_controller.py after
# the robot configuration is set. Code should check len() before accessing.
current_logical_joint_angles_rad: list[float] = []

# Global state for the gripper's last known angle (in radians)
current_gripper_angle_rad = 0.0

# Flag to indicate if the gripper servo was detected on startup
gripper_present = False


def _populate_robot_constants() -> None:
    """
    Populate robot-specific constants from the active robot configuration.
    
    Called by robot_config.set_active_robot() to ensure utils module globals
    are refreshed when the active robot changes.
    
    These are robot-specific constants (joint IDs, limits, mappings),
    distinct from servo-specific constants which are populated by _populate_servo_constants().
    """
    global current_logical_joint_angles_rad
    global NUM_LOGICAL_JOINTS, NUM_PHYSICAL_SERVOS, SERVO_IDS
    global SERVO_ID_GRIPPER, SERVO_ID_JOINT_2_SECOND, SERVO_ID_JOINT_3_SECOND
    global URDF_JOINT_LIMITS, EFFECTIVE_MAPPING_RANGES, INVERTED_SERVO_IDS
    global LOGICAL_JOINT_MASTER_OFFSETS_RAD, LOGICAL_JOINT_LIMITS_RAD, GRIPPER_LIMITS_RAD
    global PI_IP, UDP_PORT, BUFFER_SIZE, SERIAL_PORT
    global DEFAULT_SERVO_SPEED, DEFAULT_PROFILE_VELOCITY, DEFAULT_PROFILE_ACCELERATION
    global CORRECTION_KP_GAIN, CORRECTION_KI_GAIN, CORRECTION_INTEGRAL_CLAMP_RAD
    global DEFAULT_KP, DEFAULT_KI, DEFAULT_KD, DEFAULT_PID_GAINS
    global J1_PID_GAINS, J2_PID_GAINS, J3_PID_GAINS, J4_PID_GAINS, J5_PID_GAINS, J6_PID_GAINS
    global Gripper_PID_GAINS
    
    # Re-read from robot_config (which was just updated by set_active_robot)
    NUM_LOGICAL_JOINTS = robot_config.NUM_LOGICAL_JOINTS
    NUM_PHYSICAL_SERVOS = robot_config.NUM_PHYSICAL_SERVOS
    SERVO_IDS = robot_config.SERVO_IDS
    SERVO_ID_GRIPPER = robot_config.SERVO_ID_GRIPPER
    SERVO_ID_JOINT_2_SECOND = robot_config.SERVO_ID_JOINT_2_SECOND
    SERVO_ID_JOINT_3_SECOND = robot_config.SERVO_ID_JOINT_3_SECOND
    URDF_JOINT_LIMITS = robot_config.URDF_JOINT_LIMITS
    EFFECTIVE_MAPPING_RANGES = robot_config.EFFECTIVE_MAPPING_RANGES
    INVERTED_SERVO_IDS = robot_config.INVERTED_SERVO_IDS
    LOGICAL_JOINT_MASTER_OFFSETS_RAD = robot_config.LOGICAL_JOINT_MASTER_OFFSETS_RAD
    LOGICAL_JOINT_LIMITS_RAD = robot_config.LOGICAL_JOINT_LIMITS_RAD
    GRIPPER_LIMITS_RAD = robot_config.GRIPPER_LIMITS_RAD
    DEFAULT_SERVO_SPEED = robot_config.DEFAULT_SERVO_SPEED
    DEFAULT_PROFILE_VELOCITY = robot_config.DEFAULT_PROFILE_VELOCITY
    DEFAULT_PROFILE_ACCELERATION = robot_config.DEFAULT_PROFILE_ACCELERATION
    CORRECTION_KP_GAIN = robot_config.CORRECTION_KP_GAIN
    CORRECTION_KI_GAIN = robot_config.CORRECTION_KI_GAIN
    CORRECTION_INTEGRAL_CLAMP_RAD = robot_config.CORRECTION_INTEGRAL_CLAMP_RAD
    DEFAULT_KP = robot_config.DEFAULT_KP
    DEFAULT_KI = robot_config.DEFAULT_KI
    DEFAULT_KD = robot_config.DEFAULT_KD
    DEFAULT_PID_GAINS = robot_config.DEFAULT_PID_GAINS
    J1_PID_GAINS = robot_config.J1_PID_GAINS
    J2_PID_GAINS = robot_config.J2_PID_GAINS
    J3_PID_GAINS = robot_config.J3_PID_GAINS
    J4_PID_GAINS = robot_config.J4_PID_GAINS
    J5_PID_GAINS = robot_config.J5_PID_GAINS
    J6_PID_GAINS = robot_config.J6_PID_GAINS
    Gripper_PID_GAINS = robot_config.GRIPPER_PID_GAINS
    
    # Communication settings
    PI_IP = robot_config.PI_IP
    UDP_PORT = robot_config.UDP_PORT
    BUFFER_SIZE = robot_config.BUFFER_SIZE
    SERIAL_PORT = robot_config.SERIAL_PORT
    
    n_joints = NUM_LOGICAL_JOINTS
    if n_joints is not None and len(current_logical_joint_angles_rad) != n_joints:
        current_logical_joint_angles_rad = [0.0] * n_joints


# Keep old name as alias for backward compatibility during migration
_reinitialize_state = _populate_robot_constants

# =============================================================================
# Servo Orientation Mapping
# =============================================================================

# Import from robot_config for backward compatibility
INVERTED_SERVO_IDS = robot_config.INVERTED_SERVO_IDS


def _is_servo_direct_mapping(physical_servo_config_index: int) -> bool:
    """
    Determines if a physical servo's raw value increases with its angle (direct)
    or decreases (inverted). This is necessary for correct angle-to-raw-value conversion.

    Args:
        physical_servo_config_index: The 0-based index of the servo in the SERVO_IDS list.

    Returns:
        bool: True if the mapping is direct, False if inverted.
    """
    return robot_config._is_servo_direct_mapping(physical_servo_config_index)


# =============================================================================
# Utility Functions
# =============================================================================

def _convert_numpy_to_list(obj):
    """
    Recursively converts numpy arrays within a dictionary or list into native Python lists.
    This is required before saving data structures to JSON files.
    
    Args:
        obj: The object (dict, list, or other) to convert.

    Returns:
        The converted object with all numpy arrays turned into lists.
    """
    if isinstance(obj, np.ndarray):
        return obj.tolist()
    if isinstance(obj, dict):
        return {k: _convert_numpy_to_list(v) for k, v in obj.items()}
    if isinstance(obj, list):
        return [_convert_numpy_to_list(elem) for elem in obj]
    return obj


def get_actuator_backend():
    """
    Get the currently active actuator backend instance.
    
    This function provides access to the actuator backend for advanced usage.
    For most cases, use the functions in servo_driver.py instead.
    
    Returns:
        ActuatorBackend: The active backend, or None if not initialized.
    """
    # Import here to avoid circular dependency
    from . import servo_driver
    return getattr(servo_driver, '_actuator_backend', None)
