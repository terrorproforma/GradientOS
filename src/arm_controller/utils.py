# Contains utility functions and constants shared across the arm_controller package. 
import math
import os
import numpy as np

# --- UDP Configuration ---
PI_IP = "0.0.0.0"  # Listen on all available interfaces
UDP_PORT = 3000    # Same port as in follow_target_rmp_UDPsend.py
BUFFER_SIZE = 1024 # Size of the UDP receive buffer in bytes

# --- Kinematics & Planning ---
NUM_LOGICAL_JOINTS = 6 # Number of controllable joints in the kinematic model

# --- Servo Configuration ---
NUM_PHYSICAL_SERVOS = 9 # Total number of physical servos (J1 is no longer doubled up)
SERVO_IDS = [10, 20, 21, 30, 31, 40, 50, 60, 100] # Hardware ID of each servo, 100 is gripper

# IDs for the second motor on multi-servo joints
SERVO_ID_JOINT_2_SECOND = 21
SERVO_ID_JOINT_3_SECOND = 31

# ID for the gripper servo
SERVO_ID_GRIPPER = 100

# --- Default Motion Parameters ---
DEFAULT_SERVO_SPEED = 200 # Default speed for servos if not specified (0-4095)
DEFAULT_PROFILE_VELOCITY = 0.1 # m/s, for trapezoidal profiles
DEFAULT_PROFILE_ACCELERATION = 0.05 # m/s^2, for trapezoidal profiles
CORRECTION_KP_GAIN = 0.5 # Proportional gain for the closed-loop executor

# --- Serial Port Configuration ---
# Raspberry Pi GPIO 14 (TXD) and GPIO 15 (RXD) often map to /dev/ttyS0 or /dev/serial0.
# You may need to configure your Pi to enable the hardware serial port and disable the
# serial console. On a Raspberry Pi, run 'sudo raspi-config' and go to:
#  1. Interface Options -> Serial Port
#  2. Answer 'No' to 'Would you like a login shell to be accessible over serial?'
#  3. Answer 'Yes' to 'Would you like the serial port hardware to be enabled?'
#  4. Reboot your Pi.
SERIAL_PORT = "/dev/ttyAMA0"  # Default serial port for Raspberry Pi GPIO
BAUD_RATE = 1000000 # Communication speed for the Feetech servos

# --- Master Calibration Offsets ---
LOGICAL_JOINT_MASTER_OFFSETS_RAD = [
    0.0,  # Logical Joint 1 (IDs 10, 11)
    0.0,  # Logical Joint 2 (IDs 20, 21)
    0.0,  # Logical Joint 3 (IDs 30, 31)
    0.0,  # Logical Joint 4 (ID 40)
    0.0,  # Logical Joint 5 (ID 50)
    0.0   # Logical Joint 6 (ID 60)
]

# The `SERVO_ENCODER_ZERO_POSITIONS` list is now obsolete and has been removed.
# Calibration is now handled by the servo's internal `POSITION_CORRECTION` register,
# which is set using the `SET_ZERO,ID` UDP command.

# --- END CALIBRATION INPUTS ---

# Logical Joint Limits (radians) [min_limit, max_limit]
# These are the effective limits for the 6 logical joints of the arm.
LOGICAL_JOINT_LIMITS_RAD = [
    [-3.1416, 3.1416], # Logical J1 (Base)
    [-1.5708, 1.5708], # Logical J2 (Shoulder)
    [-1.5708, 1.5708], # Logical J3 (Elbow)
    [-3.1416, 3.1416], # Logical J4 (Wrist Roll)
    [-1.8326, 2.0944], # Logical J5 (Wrist Pitch)
    [-3.1416, 3.1416]  # Logical J6 (Wrist Yaw)
]

# Gripper servo limits (radians)
GRIPPER_LIMITS_RAD = [0, 3.1416]  # 0° (closed) to 90° (open)

# URDF Joint Limits (radians) [min_limit, max_limit] - These are for the LOGICAL joints
# but need to be consistent with the physical servo capabilities after all offsets.
# The config for physical servos (config indices 0-8) must match up.
# E.g. URDF_JOINT_LIMITS[0] and [1] are for Servos ID 10 and 11 (J1).
URDF_JOINT_LIMITS = [ # Per PHYSICAL servo config index
    LOGICAL_JOINT_LIMITS_RAD[0], # Servo ID 10 (J1)
    LOGICAL_JOINT_LIMITS_RAD[1], # Servo ID 20 (Logical J2, Servo 1)
    LOGICAL_JOINT_LIMITS_RAD[1], # Servo ID 21 (Logical J2, Servo 2) - Must match Servo ID 20
    LOGICAL_JOINT_LIMITS_RAD[2], # Servo ID 30 (Logical J3, Servo 1)
    LOGICAL_JOINT_LIMITS_RAD[2], # Servo ID 31 (Logical J3, Servo 2) - Must match Servo ID 30
    LOGICAL_JOINT_LIMITS_RAD[3], # Servo ID 40 (Logical J4)
    LOGICAL_JOINT_LIMITS_RAD[4], # Servo ID 50 (Logical J5)
    LOGICAL_JOINT_LIMITS_RAD[5], # Servo ID 60 (Logical J6)
    GRIPPER_LIMITS_RAD           # Servo ID 100 (Gripper)
]

# Effective Mapping Ranges: Assuming 0-4095 on servo maps to +/- PI radians from servo zero.
EFFECTIVE_MAPPING_RANGES = [] # Per PHYSICAL servo config index
for _ in range(NUM_PHYSICAL_SERVOS): 
    EFFECTIVE_MAPPING_RANGES.append([-math.pi, math.pi])

# --- Servo Protocol Constants ---
"""Constants that define the Feetech servo protocol, such as instruction bytes and register addresses."""
SERVO_HEADER = 0xFF
SERVO_INSTRUCTION_WRITE = 0x03
SERVO_INSTRUCTION_READ = 0x02
SERIAL_READ_TIMEOUT = 0.05 

SERVO_ADDR_TARGET_POSITION = 0x2A 
SERVO_ADDR_PRESENT_POSITION = 0x38 
SERVO_ADDR_TARGET_ACCELERATION = 0x29 
SERVO_ADDR_POSITION_CORRECTION = 31 # Register 0x1F, for storing hardware zero offset

# Default values for the servo's internal controller and motion profiles
DEFAULT_SERVO_ACCELERATION_DEG_S2 = 500
ACCELERATION_SCALE_FACTOR = 100

SERVO_ADDR_POS_KP = 0x15
SERVO_ADDR_POS_KI = 0x17 
SERVO_ADDR_POS_KD = 0x16

DEFAULT_KP = 32  
DEFAULT_KI = 0   
DEFAULT_KD = 0

# Trajectory Planning Optimization
IK_PLANNING_FREQUENCY = 100 # Hz. We solve IK at this rate, then interpolate for smoother, faster execution.

# Trajectory Caching
# Note: The path is adjusted to navigate from `src/arm_controller` up to the project root.
TRAJECTORY_CACHE_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", "trajectory_cache"))

# Global state for trajectory execution
trajectory_state = {
    "is_running": False,
    "should_stop": False,
    "thread": None,
    # --- New state for real-time jogging ---
    "is_jogging": False,
    # 6D vector: [vx, vy, vz, v_roll, v_pitch, v_yaw]
    "jog_velocities": np.zeros(6, dtype=float),
    "last_jog_command_time": 0.0,
}


# Servo Sync Write Constants
SERVO_INSTRUCTION_SYNC_WRITE = 0x83
SERVO_BROADCAST_ID = 0xFE # Typically 0xFE for Feetech Sync Write
# Start address for Sync Write block (Accel, Pos, Time, Speed)
SYNC_WRITE_START_ADDRESS = SERVO_ADDR_TARGET_ACCELERATION # 0x29
# Length of data block per servo in Sync Write: Accel (1) + Pos (2) + Time (2) + Speed (2) = 7 bytes
SYNC_WRITE_DATA_LEN_PER_SERVO = 7
# Servo Sync Read Constants
SERVO_INSTRUCTION_SYNC_READ = 0x82

# Global serial object
ser: 'serial.Serial | None' = None

# Global state for the arm's last known logical joint angles (in radians)
current_logical_joint_angles_rad = [0.0] * NUM_LOGICAL_JOINTS
# NEW: Global state for the gripper's last known angle (in radians)
current_gripper_angle_rad = 0.0
# NEW: Flag to indicate if the gripper servo (ID 100) was detected on startup
gripper_present = False

# -----------------------------------------------------------------------------
# Servo Orientation Mapping (RAW ↔︎ Angle)
# -----------------------------------------------------------------------------
# For each physical servo we have to know whether a positive physical rotation of
# the joint corresponds to an increase ("direct") or a decrease ("inverted") of
# the raw encoder/command value.  This depends on how the servo is mounted.
#
# Mapping for the current robot (verified by physical testing):
#   Joint 1 : ID 10 → inverted
#   Joint 2 : ID 20 → inverted| ID 21 → direct
#   Joint 3 : ID 30 → inverted| ID 31 → direct
#   Joint 4 : ID 40 → inverted
#   Joint 5 : ID 50 → inverted
#   Joint 6 : ID 60 → direct
#
# If you commission a new robot and the mounting differs, simply edit the
# `INVERTED_SERVO_IDS` set below so the control-system math stays correct.

# IDs whose raw value DECREASES when the joint rotates in the positive logical
# direction.
INVERTED_SERVO_IDS: set[int] = {10, 20, 30, 40, 50, 60, 100} # Assuming gripper (100) is inverse mapping

def _is_servo_direct_mapping(physical_servo_config_index: int) -> bool:
    """
    Determines if a physical servo's raw value increases with its angle (direct)
    or decreases (inverted). This is necessary for correct angle-to-raw-value conversion.

    Args:
        physical_servo_config_index (int): The 0-based index of the servo in the SERVO_IDS list.

    Returns:
        bool: True if the mapping is direct, False if inverted.
    """
    current_physical_servo_id = SERVO_IDS[physical_servo_config_index]

    # Return False (inverted) if the ID is in the inverted set; True otherwise.
    return current_physical_servo_id not in INVERTED_SERVO_IDS


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

