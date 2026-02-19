# robot_config.py
#
# Robot configuration module for GradientOS.
#
# This module provides backward-compatible access to robot configuration values.
# It imports from the new modular robot configuration system (robots/) and
# re-exports values as module-level constants for existing code.
#
# IMPORTANT: set_active_robot() MUST be called before accessing any values.
# This is done in run_controller.py at startup.
#
# For new code, prefer using the RobotConfig class directly:
#   from gradient_os.arm_controller.robots import get_robot_config
#   robot = get_robot_config("gradient0")

import math
import numpy as np
from typing import Optional

# Import the robot configuration class
from .robots import RobotConfig, get_robot_config

# The active robot instance - MUST be set by set_active_robot() before use
_robot: Optional[RobotConfig] = None


def _ensure_robot_configured():
    """Raise an error if no robot has been configured."""
    if _robot is None:
        raise RuntimeError(
            "No robot configured. Call robot_config.set_active_robot() first. "
            "This is normally done in run_controller.py at startup."
        )

# =============================================================================
# Module-level constants for backward compatibility
# These are initialized to None and populated by set_active_robot()
# =============================================================================

# --- Robot Identity ---
ROBOT_ID = None
ROBOT_NAME = None
ROBOT_VERSION = None

# --- Kinematic Configuration ---
NUM_LOGICAL_JOINTS = None
NUM_PHYSICAL_SERVOS = None

# --- Servo Configuration ---
SERVO_IDS = None
SERVO_ID_GRIPPER = None
SERVO_ID_JOINT_2_SECOND = None
SERVO_ID_JOINT_3_SECOND = None

# --- Joint Mapping ---
LOGICAL_TO_PHYSICAL_MAP = None

# --- Joint Limits ---
LOGICAL_JOINT_LIMITS_RAD = None
GRIPPER_LIMITS_RAD = None
URDF_JOINT_LIMITS = None

# --- Angle Mapping ---
EFFECTIVE_MAPPING_RANGES = None
INVERTED_SERVO_IDS = None

# --- Calibration ---
LOGICAL_JOINT_MASTER_OFFSETS_RAD = None

# --- Motion Parameters ---
DEFAULT_SERVO_SPEED = None
DEFAULT_SERVO_ACCELERATION_DEG_S2 = None
ACCELERATION_SCALE_FACTOR = None

# --- Trapezoidal Profile ---
DEFAULT_PROFILE_VELOCITY = None
DEFAULT_PROFILE_ACCELERATION = None

# --- Closed-Loop Control ---
CORRECTION_KP_GAIN = None
CORRECTION_KI_GAIN = None
CORRECTION_INTEGRAL_CLAMP_RAD = None

# --- Communication ---
SERIAL_PORT = None
PI_IP = None
UDP_PORT = None
BUFFER_SIZE = None

# --- PID Gains ---
DEFAULT_KP = None
DEFAULT_KI = None
DEFAULT_KD = None
DEFAULT_PID_GAINS = None
J1_PID_GAINS = None
J2_PID_GAINS = None
J3_PID_GAINS = None
J4_PID_GAINS = None
J5_PID_GAINS = None
J6_PID_GAINS = None
GRIPPER_PID_GAINS = None

# --- Servo-specific (from backend) ---
BAUD_RATE = None

# =============================================================================
# Helper Functions (Backward Compatibility)
# =============================================================================

def _is_servo_direct_mapping(physical_servo_config_index: int) -> bool:
    """
    Check if a servo uses direct mapping (raw increases with angle).
    
    Args:
        physical_servo_config_index: Index in SERVO_IDS list.
    
    Returns:
        bool: True if direct, False if inverted.
    """
    _ensure_robot_configured()
    return _robot.is_actuator_direct_mapping(physical_servo_config_index)


def get_robot_config_dict() -> dict:
    """
    Get the robot configuration as a dictionary.
    
    This is useful for passing configuration to ActuatorBackend or serialization.
    
    Returns:
        dict: Configuration dictionary.
    """
    _ensure_robot_configured()
    return _robot.get_config_dict()


def get_active_robot() -> RobotConfig:
    """
    Get the currently active robot configuration instance.
    
    Returns:
        RobotConfig: The active robot configuration.
    
    Raises:
        RuntimeError: If no robot has been configured.
    """
    _ensure_robot_configured()
    return _robot


def set_active_robot(robot: RobotConfig) -> None:
    """
    Set the active robot configuration.
    
    This updates ALL module-level constants to match the new robot.
    MUST be called before any other code accesses robot configuration.
    
    Args:
        robot: The new robot configuration to use.
    """
    global _robot
    global ROBOT_ID, ROBOT_NAME, ROBOT_VERSION
    global NUM_LOGICAL_JOINTS, NUM_PHYSICAL_SERVOS, SERVO_IDS
    global SERVO_ID_GRIPPER, SERVO_ID_JOINT_2_SECOND, SERVO_ID_JOINT_3_SECOND
    global LOGICAL_TO_PHYSICAL_MAP, LOGICAL_JOINT_LIMITS_RAD, GRIPPER_LIMITS_RAD
    global URDF_JOINT_LIMITS, EFFECTIVE_MAPPING_RANGES, INVERTED_SERVO_IDS
    global LOGICAL_JOINT_MASTER_OFFSETS_RAD, SERIAL_PORT, PI_IP, UDP_PORT, BUFFER_SIZE
    global DEFAULT_SERVO_SPEED, DEFAULT_SERVO_ACCELERATION_DEG_S2, ACCELERATION_SCALE_FACTOR
    global DEFAULT_PROFILE_VELOCITY, DEFAULT_PROFILE_ACCELERATION
    global CORRECTION_KP_GAIN, CORRECTION_KI_GAIN, CORRECTION_INTEGRAL_CLAMP_RAD
    global DEFAULT_KP, DEFAULT_KI, DEFAULT_KD, DEFAULT_PID_GAINS
    global J1_PID_GAINS, J2_PID_GAINS, J3_PID_GAINS, J4_PID_GAINS, J5_PID_GAINS, J6_PID_GAINS
    global GRIPPER_PID_GAINS, BAUD_RATE
    
    _robot = robot
    
    # --- Robot Identity ---
    ROBOT_ID = robot.robot_id
    ROBOT_NAME = robot.name
    ROBOT_VERSION = robot.version
    
    # --- Kinematic Configuration ---
    NUM_LOGICAL_JOINTS = robot.num_logical_joints
    NUM_PHYSICAL_SERVOS = robot.num_physical_actuators
    
    # --- Servo Configuration ---
    SERVO_IDS = robot.actuator_ids
    SERVO_ID_GRIPPER = robot.gripper_actuator_id
    _twin = robot.twin_motor_actuator_ids
    SERVO_ID_JOINT_2_SECOND = _twin.get(1)
    SERVO_ID_JOINT_3_SECOND = _twin.get(2)
    
    # --- Joint Mapping ---
    LOGICAL_TO_PHYSICAL_MAP = robot.logical_to_physical_map
    
    # --- Joint Limits ---
    LOGICAL_JOINT_LIMITS_RAD = robot.logical_joint_limits_rad
    GRIPPER_LIMITS_RAD = list(robot.gripper_limits_rad) if robot.gripper_limits_rad else [0, math.pi]
    URDF_JOINT_LIMITS = robot.actuator_limits_rad
    
    # --- Angle Mapping ---
    EFFECTIVE_MAPPING_RANGES = robot.actuator_mapping_ranges_rad
    INVERTED_SERVO_IDS = robot.inverted_actuator_ids
    
    # --- Calibration ---
    LOGICAL_JOINT_MASTER_OFFSETS_RAD = robot.logical_joint_master_offsets_rad
    
    # --- Motion Parameters ---
    DEFAULT_SERVO_SPEED = robot.default_speed
    DEFAULT_SERVO_ACCELERATION_DEG_S2 = robot.default_acceleration_deg_s2
    ACCELERATION_SCALE_FACTOR = robot.acceleration_scale_factor
    
    # --- Trapezoidal Profile ---
    DEFAULT_PROFILE_VELOCITY = robot.default_profile_velocity
    DEFAULT_PROFILE_ACCELERATION = robot.default_profile_acceleration
    
    # --- Closed-Loop Control ---
    CORRECTION_KP_GAIN = robot.correction_kp_gain
    CORRECTION_KI_GAIN = robot.correction_ki_gain
    CORRECTION_INTEGRAL_CLAMP_RAD = robot.correction_integral_clamp_rad
    
    # --- Communication ---
    SERIAL_PORT = robot.default_serial_port
    PI_IP = robot.udp_listen_ip
    UDP_PORT = robot.udp_port
    BUFFER_SIZE = robot.udp_buffer_size
    
    # --- PID Gains ---
    _dpid = robot.default_pid_gains
    DEFAULT_KP = _dpid[0]
    DEFAULT_KI = _dpid[1]
    DEFAULT_KD = _dpid[2]
    DEFAULT_PID_GAINS = robot.actuator_pid_gains
    
    # Per-joint gains (using actuator IDs from this robot's config)
    _pid = robot.actuator_pid_gains
    _actuator_ids = robot.actuator_ids
    if len(_actuator_ids) >= 9:  # Standard 6DOF + gripper robot
        J1_PID_GAINS = _pid.get(_actuator_ids[0], _dpid)
        J2_PID_GAINS = _pid.get(_actuator_ids[1], _dpid)
        J3_PID_GAINS = _pid.get(_actuator_ids[3], _dpid)
        J4_PID_GAINS = _pid.get(_actuator_ids[5], _dpid)
        J5_PID_GAINS = _pid.get(_actuator_ids[6], _dpid)
        J6_PID_GAINS = _pid.get(_actuator_ids[7], _dpid)
        GRIPPER_PID_GAINS = _pid.get(_actuator_ids[8], _dpid) if robot.gripper_actuator_id else _dpid
    
    # --- Servo-specific (from backend registry) ---
    from .backends import registry as backend_registry
    if backend_registry.is_configured():
        BAUD_RATE = backend_registry.get_default_baud_rate()
    
    print(f"[RobotConfig] Active robot set to: {robot.name} v{robot.version}")
    
    # Reinitialize utils module state (e.g., current_logical_joint_angles_rad)
    # Populate utils with robot-specific constants from the active robot config.
    # This must be called AFTER all the globals are set above.
    from . import utils
    utils._populate_robot_constants()
