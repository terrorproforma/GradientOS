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
NUM_PHYSICAL_SERVOS = 9 # Total number of physical servos (J1, J2, J3 are doubled up)
SERVO_IDS = [10, 11, 20, 21, 30, 31, 40, 50, 60] # Hardware ID of each servo

# IDs for the second motor on multi-servo joints
SERVO_ID_JOINT_1_SECOND = 11
SERVO_ID_JOINT_2_SECOND = 21
SERVO_ID_JOINT_3_SECOND = 31

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
    [-1.5708, 1.5708], # Logical J1 (Base)
    [-1.5708, 1.5708], # Logical J2 (Shoulder)
    [-1.5708, 1.5708], # Logical J3 (Elbow)
    [-3.1416, 3.1416], # Logical J4 (Wrist Roll)
    [-1.8326, 2.0944], # Logical J5 (Wrist Pitch)
    [-3.1416, 3.1416]  # Logical J6 (Wrist Yaw)
]

# URDF Joint Limits (radians) [min_limit, max_limit] - These are for the LOGICAL joints
# but need to be consistent with the physical servo capabilities after all offsets.
# The config for physical servos (config indices 0-8) must match up.
# E.g. URDF_JOINT_LIMITS[0] and [1] are for Servos ID 10 and 11 (J1).
URDF_JOINT_LIMITS = [ # Per PHYSICAL servo config index
    [lim * 2.0 for lim in LOGICAL_JOINT_LIMITS_RAD[0]], # Servo ID 10 (J1) -> Physical range is 2x logical due to gearing
    [lim * 2.0 for lim in LOGICAL_JOINT_LIMITS_RAD[0]], # Servo ID 11 (J1) -> Physical range is 2x logical due to gearing
    LOGICAL_JOINT_LIMITS_RAD[1], # Servo ID 20 (Logical J2, Servo 1)
    LOGICAL_JOINT_LIMITS_RAD[1], # Servo ID 21 (Logical J2, Servo 2) - Must match Servo ID 20
    LOGICAL_JOINT_LIMITS_RAD[2], # Servo ID 30 (Logical J3, Servo 1)
    LOGICAL_JOINT_LIMITS_RAD[2], # Servo ID 31 (Logical J3, Servo 2) - Must match Servo ID 30
    LOGICAL_JOINT_LIMITS_RAD[3], # Servo ID 40 (Logical J4)
    LOGICAL_JOINT_LIMITS_RAD[4], # Servo ID 50 (Logical J5)
    LOGICAL_JOINT_LIMITS_RAD[5]  # Servo ID 60 (Logical J6)
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
    "thread": None
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

# -----------------------------------------------------------------------------
# Servo Orientation Mapping (RAW ↔︎ Angle)
# -----------------------------------------------------------------------------
# For each physical servo we have to know whether a positive physical rotation of
# the joint corresponds to an increase ("direct") or a decrease ("inverted") of
# the raw encoder/command value.  This depends on how the servo is mounted.
#
# Mapping for the current robot (verified by physical testing):
#   Joint 1 : ID 10 → direct  | ID 11 → direct
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
INVERTED_SERVO_IDS: set[int] = {20, 30, 40, 50, 60}



# Contains low-level functions for communicating with Feetech servos,
# such as packet creation and checksum calculation. 
from . import utils


# Servo Protocol Constants
SERVO_HEADER = 0xFF
SERVO_INSTRUCTION_WRITE = 0x03
SERVO_ADDR_TARGET_POSITION = 0x2A 
SERVO_INSTRUCTION_READ = 0x02
SERVO_ADDR_PRESENT_POSITION = 0x38 
SERIAL_READ_TIMEOUT = 0.05 
SERVO_ADDR_TARGET_ACCELERATION = 0x29 
DEFAULT_SERVO_ACCELERATION_DEG_S2 = 500 
ACCELERATION_SCALE_FACTOR = 100 
SERVO_ADDR_POS_KP = 0x15
SERVO_ADDR_POS_KI = 0x17 
SERVO_ADDR_POS_KD = 0x16
DEFAULT_KP = 32  
DEFAULT_KI = 0   
DEFAULT_KD = 0  
SERVO_INSTRUCTION_CALIBRATE_MIDDLE = 0x0B
SERVO_INSTRUCTION_RESET = 0x06
SERVO_INSTRUCTION_RESTART = 0x08


# Servo Sync Write Constants
SERVO_INSTRUCTION_SYNC_WRITE = 0x83
SERVO_BROADCAST_ID = 0xFE # Typically 0xFE for Feetech Sync Write
# Start address for Sync Write block (Accel, Pos, Time, Speed)
SYNC_WRITE_START_ADDRESS = SERVO_ADDR_TARGET_ACCELERATION # 0x29
# Length of data block per servo in Sync Write: Accel (1) + Pos (2) + Time (2) + Speed (2) = 7 bytes
SYNC_WRITE_DATA_LEN_PER_SERVO = 7
# Servo Sync Read Constants
SERVO_INSTRUCTION_SYNC_READ = 0x82


# Contains high-level functions for controlling the servos, acting as a
# hardware abstraction layer. Imports from servo_protocol.py. 
import time
import serial
import numpy as np

from . import utils
from . import servo_protocol



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



def calculate_checksum(packet_data_for_sum: bytearray) -> int:
    """
    Calculates the Feetech checksum for a given packet.
    The checksum is the bitwise inverse of the sum of all bytes in the packet
    (excluding the initial 0xFF headers).

    Args:
        packet_data_for_sum (bytearray): The packet data (from Servo ID to last parameter) to sum.

    Returns:
        int: The calculated checksum byte.
    """
    current_sum = sum(packet_data_for_sum)
    return (~current_sum) & 0xFF


def send_servo_command(servo_id: int, position_value: int, speed_value: int = None):
    """
    Sends a command to a specific servo to move to a target position with a given speed.
    Command Packet: [0xFF, 0xFF, ID, PacketLen=9, Instr=0x03, Addr=0x2A, Pos_L, Pos_H, Pad1=0, Pad2=0, Spd_L, Spd_H, Checksum]
    Addr (Target Position): 0x2A (42)
    Addr (Target Speed): 0x2C (44) - Note: The actual command sends speed along with position.
    The specific command 0x03 (WRITE) to address 0x2A (Goal Position) seems to take 6 bytes of parameters
    (Pos_L, Pos_H, Time_L, Time_H, Speed_L, Speed_H) according to some Feetech docs,
    or (Pos_L, Pos_H, 0, 0, Speed_L, Speed_H) for STS series.
    The provided Arduino code suggests: Addr=0x2A, Pos_L, Pos_H, Pad1=0, Pad2=0, Spd_L, Spd_H
    This implies instruction 0x03 (WRITE) writes multiple registers starting from 0x2A.
    0x2A, 0x2B for Goal Position (2 bytes)
    0x2C, 0x2D for Goal Time (2 bytes) - We use 0,0 for this (Pad1, Pad2) implying minimal time / use speed
    0x2E, 0x2F for Goal Speed (2 bytes)
    So PacketLen should be NumParams + 2 = 6 + 2 = 8, if Addr is start.
    Ah, PacketLen is the length of the part from `Instr` to the last data byte of the parameter.
    So, `Instr (1) + StartAddr (1) + Number of data bytes to write (6 in this case: Pos_L, Pos_H, Pad, Pad, Spd_L, Spd_H) = 8`.
    The Arduino code uses `DataLen = 9`. This is `ID, Len, Instr, Addr, P1, P2, P3, P4, P5, P6, Checksum`.
    No, `PacketLen` is the length of the part from `Instr` to the last data byte of the parameter.
    So, `Instr (1) + StartAddr (1) + DataBytes (6 for Pos_L,Pos_H,Pad,Pad,Spd_L,Spd_H) = 8`.
    Let's re-check the Arduino code: `writeBuf[3] = 9;` This is `LEN`.
    `writeBuf[4] = scs_inst_write; //INST`
    `writeBuf[5] = reg; //PARAM_0 (Address)`
    `writeBuf[6] = data; //PARAM_1 (Data for that address - if only one byte)`
    `writeBuf[7] = data1; //PARAM_2`
    ...
    For `WritePosEx` (position and speed): `ADDR_STS_GOAL_POSITION = 42 (0x2A)`.
    `writeReg(ID, ADDR_STS_GOAL_POSITION, (unsigned char*)&Position, 2);` // Writes 2 bytes (Pos_L, Pos_H)
    `writeReg(ID, ADDR_STS_GOAL_SPEED,  (unsigned char*)&Speed, 2);`   // Writes 2 bytes (Spd_L, Spd_H)
    This implies two separate write commands if we follow that pattern.

    However, the `syncWrite` or single command for pos+speed often writes to a starting address (e.g., Goal Position 0x2A)
    and the subsequent bytes are for speed, effectively writing to 0x2A, 0x2B, 0x2C, 0x2D (or skipping some like time).
    The confirmed working packet was:
    `[0xFF, 0xFF, ID, PacketLen=9, Instr=0x03, Addr=0x2A, Pos_L, Pos_H, Pad1=0, Pad2=0, Spd_L, Spd_H, Checksum]`
    Here, `PacketLen=9` means:
    `Instr (1) + Addr (1) + Pos_L (1) + Pos_H (1) + Pad1 (1) + Pad2 (1) + Spd_L (1) + Spd_H (1) + ??? (1 extra byte in count?)`
    Let's re-evaluate the packet length definition for Feetech.
    "Packet Length: Length of Data Packets = Number of Parameters N + 2 (Instruction and Checksum are not included)"
    If Parameters = [Addr, Data1, Data2, ..., DataM], then N = M+1. PacketLength = M+1+2 = M+3.
    For Instr=0x03 (WRITE), Parameters are: Start_Address, Value1, Value2...
    Here, Start_Address=0x2A. Values are Pos_L, Pos_H, Pad1, Pad2, Spd_L, Spd_H. So 6 values.
    So N (number of parameters for instruction) = 1 (Address) + 6 (Data bytes) = 7.
    PacketLength = N + 2 = 7 + 2 = 9. This matches the `PacketLen=9`. Perfect.

    Parameters for instruction 0x03 (WRITE):
    1. Address (0x2A)
    2. Data for 0x2A (Pos_L)
    3. Data for 0x2B (Pos_H)
    4. Data for 0x2C (Pad1, for Time_L)
    5. Data for 0x2D (Pad2, for Time_H)
    6. Data for 0x2E (Spd_L)
    7. Data for 0x2F (Spd_H)
    Total 7 parameters.

    Args:
        servo_id (int): The hardware ID of the target servo.
        position_value (int): The target position, in raw servo values (0-4095).
        speed_value (int, optional): The target speed (0-4095). Defaults to `DEFAULT_SERVO_SPEED`.
    """
    if speed_value is None:
        speed_value = utils.DEFAULT_SERVO_SPEED

    if utils.ser is None:
        print("[Pi] Serial port not initialized.")
        return

    # Clamp position and speed to their valid ranges
    pos_val_clamped = int(max(0, min(4095, position_value)))
    spd_val_clamped = int(max(0, min(4095, speed_value)))

    # Goal Position (Address 0x2A), 2 bytes for position, 2 bytes for "time" (set to 0), 2 bytes for speed
    # This means we are writing 6 bytes starting at address 0x2A.
    # Parameters = [Addr=0x2A, Pos_L, Pos_H, Time_L=0, Time_H=0, Spd_L, Spd_H]
    # PacketLen = NumParams (Addr + 6 data bytes = 7) + 2 = 9. This is correct.
    # Instruction = 0x03 (WRITE)
    # Start Address = SERVO_ADDR_TARGET_POSITION (0x2A)

    packet = bytearray(13)
    packet[0] = SERVO_HEADER # 0xFF
    packet[1] = SERVO_HEADER # 0xFF
    packet[2] = servo_id     # Servo ID
    packet[3] = 9            # Packet Length = (Num Instruction Parameters) + 2. Here: Addr + 6 data bytes = 7 params. 7+2=9.
    packet[4] = SERVO_INSTRUCTION_WRITE # 0x03
    packet[5] = SERVO_ADDR_TARGET_POSITION # 0x2A (Start address for Goal Position)
    packet[6] = pos_val_clamped & 0xFF # Position Low Byte
    packet[7] = (pos_val_clamped >> 8) & 0xFF # Position High Byte
    packet[8] = 0 # Padding / Time_L (set to 0)
    packet[9] = 0 # Padding / Time_H (set to 0)
    packet[10] = spd_val_clamped & 0xFF # Speed Low Byte
    packet[11] = (spd_val_clamped >> 8) & 0xFF # Speed High Byte
    
    # Calculate checksum on ID, PacketLen, Instruction, Address, and all data bytes
    packet_data_for_sum = packet[2 : 12] # From ID up to Spd_H
    packet[12] = calculate_checksum(packet_data_for_sum)

    try:
        print(f"[Pi PROTOCOL] send_servo_command to ID {servo_id}: {list(packet)}")
        utils.ser.write(packet)
        # print(f"[Pi] Servo {servo_id} command: {list(packet)}") # Verbose
    except Exception as e:
        print(f"[Pi] Error writing to serial for servo {servo_id}: {e}")


def set_servo_acceleration(servo_id: int, acceleration_value_deg_s2: float):
    """
    Constructs and sends a 'WRITE' packet to set a single servo's acceleration register.

    Args:
        servo_id (int): The hardware ID of the target servo.
        acceleration_value_deg_s2 (float): Target acceleration in degrees per second squared.
                                           A value of 0 means max physical acceleration.
    """
    if utils.ser is None:
        print("[Pi] Serial port not initialized for acceleration command.")
        return

    servo_register_value = 0
    if acceleration_value_deg_s2 > 0:
        # Convert deg/s^2 to servo register scale (0-254)
        servo_register_value = int(round(acceleration_value_deg_s2 / utils.ACCELERATION_SCALE_FACTOR))
        # Clamp to servo's valid register range (1-254 if > 0, or 0 for max accel)
        servo_register_value = max(1, min(254, servo_register_value)) # if >0, use 1-254
    else:
        servo_register_value = 0 # Explicitly set to 0 for max physical acceleration

    # Packet: [0xFF, 0xFF, ID, PacketLen=4, Instr=0x03, Addr=0x29, Value, Checksum]
    # PacketLen = NumParams (Addr + Value = 2) + 2 = 4.
    # Checksum on ID, PacketLen, Instr, Addr, Value
    packet = bytearray(8)
    packet[0] = SERVO_HEADER
    packet[1] = SERVO_HEADER
    packet[2] = servo_id
    packet[3] = 4  # Packet Length
    packet[4] = SERVO_INSTRUCTION_WRITE # 0x03 (WRITE)
    packet[5] = SERVO_ADDR_TARGET_ACCELERATION # 0x29
    packet[6] = servo_register_value # Acceleration value (0-254)

    packet_data_for_sum = packet[2:7] # ID, Len, Instr, Addr, Value
    packet[7] = calculate_checksum(packet_data_for_sum)

    try:
        utils.ser.write(packet)
        # print(f"[Pi] Servo {servo_id} acceleration set to: {servo_register_value} (reg value), from {acceleration_value_deg_s2} deg/s^2")
    except Exception as e:
        print(f"[Pi] Error writing acceleration for servo {servo_id}: {e}")



def read_servo_position(servo_id: int) -> int | None:
    """
    Constructs and sends a 'READ' packet to request the current position of a single servo.

    Args:
        servo_id (int): The hardware ID of the target servo.

    Returns:
        int | None: The servo's current raw position (0-4095), or None on failure.
    """
    if utils.ser is None or not utils.ser.is_open:
        print(f"[Pi ReadPos] Servo {servo_id}: Serial port not open.")
        return None

    # Command Packet: [0xFF, 0xFF, ID, Length=4, Instr=0x02, Addr=0x38, BytesToRead=2, Checksum]
    # Length = (Num Instruction Parameters) + 2. Here: Addr + BytesToRead = 2 params. 2+2=4.
    read_command = bytearray(8)
    read_command[0] = SERVO_HEADER
    read_command[1] = SERVO_HEADER
    read_command[2] = servo_id
    read_command[3] = 4  # Length
    read_command[4] = SERVO_INSTRUCTION_READ
    read_command[5] = SERVO_ADDR_PRESENT_POSITION
    read_command[6] = 2  # Number of bytes to read (for position)
    
    # Checksum for read command (ID, Length, Instr, Addr, BytesToRead)
    read_command[7] = calculate_checksum(read_command[2:7])

    try:
        utils.ser.reset_input_buffer() # Clear any old data
        utils.ser.write(read_command)
        # print(f"[Pi ReadPos] Servo {servo_id}: Sent read cmd: {list(read_command)}")
        
        # Expected response: [0xFF, 0xFF, ID, Length=5, Error, Pos_L, Pos_H, Checksum]
        # Total 8 bytes. Length field (index 3) should be 5.
        response = utils.ser.read(8) 
        # print(f"[Pi ReadPos] Servo {servo_id}: Raw response: {list(response)}")

        if not response or len(response) < 8:
            # print(f"[Pi ReadPos] Servo {servo_id}: No/incomplete response (got {len(response)} bytes).")
            return None

        if response[0] != SERVO_HEADER or response[1] != SERVO_HEADER:
            print(f"[Pi ReadPos] Servo {servo_id}: Invalid response header: {list(response[:2])}")
            return None
        
        if response[2] != servo_id:
            print(f"[Pi ReadPos] Servo {servo_id}: Response ID mismatch (expected {servo_id}, got {response[2]}).")
            return None

        response_len_field = response[3]
        # For STATUS packet: Length = (Number of returned data parameters, e.g. Pos_L, Pos_H) + 2
        # Returned data parameters = 2 (Pos_L, Pos_H). So, Length field should be 2 + 2 = 4.
        if response_len_field != 4: 
            print(f"[Pi ReadPos] Servo {servo_id}: Incorrect response length field (expected 4, got {response_len_field}). Packet: {list(response)}")
            return None

        # Validate checksum of response (ID, Length, Error, Pos_L, Pos_H)
        expected_checksum = calculate_checksum(response[2:7])
        actual_checksum = response[7]
        if expected_checksum != actual_checksum:
            print(f"[Pi ReadPos] Servo {servo_id}: Response checksum mismatch (expected {expected_checksum}, got {actual_checksum}). Packet: {list(response)}")
            return None

        error_byte = response[4]
        if error_byte != 0:
            print(f"[Pi ReadPos] Servo {servo_id}: Servo reported error: {error_byte}")
            return None # Or handle specific errors

        # Per the datasheet, Current Position can be a signed value in multi-turn mode.
        # It's safer to always read it as a signed 16-bit integer.
        position = int.from_bytes(response[5:7], byteorder='little', signed=True)
        # print(f"[Pi ReadPos] Servo {servo_id}: Successfully read raw position: {position}")
        return position

    except serial.SerialTimeoutException:
        print(f"[Pi ReadPos] Servo {servo_id}: Serial timeout during read.")
        return None
    except Exception as e:
        print(f"[Pi ReadPos] Servo {servo_id}: Error during read_servo_position: {e}")
        return None


def read_servo_register_signed_word(servo_id: int, register_address: int) -> int | None:
    """
    Reads a signed 16-bit word (2 bytes) from an arbitrary servo register.

    Args:
        servo_id (int): The hardware ID of the target servo.
        register_address (int): The address of the register to read from.

    Returns:
        int | None: The signed 16-bit value from the register, or None on failure.
    """
    if utils.ser is None or not utils.ser.is_open:
        print(f"[Pi ReadWord] Servo {servo_id}: Serial port not open.")
        return None

    # Command to read 2 bytes from the specified address
    read_command = bytearray(8)
    read_command[0] = SERVO_HEADER
    read_command[1] = SERVO_HEADER
    read_command[2] = servo_id
    read_command[3] = 4  # Length
    read_command[4] = SERVO_INSTRUCTION_READ
    read_command[5] = register_address
    read_command[6] = 2  # Number of bytes to read
    read_command[7] = calculate_checksum(read_command[2:7])

    try:
        utils.ser.reset_input_buffer()
        utils.ser.write(read_command)
        
        # Expected response: [0xFF, 0xFF, ID, Length=4, Error, Val_L, Val_H, Checksum]
        response = utils.ser.read(8) 

        if not response or len(response) < 8:
            print(f"[Pi ReadWord] Servo {servo_id}: No/incomplete response.")
            return None

        # Basic validation
        if response[0] != SERVO_HEADER or response[1] != SERVO_HEADER or response[2] != servo_id:
            print(f"[Pi ReadWord] Servo {servo_id}: Invalid response header or ID mismatch.")
            return None
        
        # Checksum validation
        expected_checksum = calculate_checksum(response[2:7])
        if expected_checksum != response[7]:
            print(f"[Pi ReadWord] Servo {servo_id}: Response checksum mismatch.")
            return None
        
        # Error byte validation
        if response[4] != 0:
            print(f"[Pi ReadWord] Servo {servo_id}: Servo reported error: {response[4]}")
            return None

        # Convert the 2 data bytes into a signed 16-bit integer (little-endian)
        value = int.from_bytes(response[5:7], byteorder='little', signed=True)
        return value

    except Exception as e:
        print(f"[Pi ReadWord] Servo {servo_id}: Error during read: {e}")
        return None


def write_servo_register_word(servo_id: int, register_address: int, value: int) -> bool:
    """
    Helper function to write a 2-byte word to a servo register.

    Args:
        servo_id (int): The hardware ID of the target servo.
        register_address (int): The address of the register to write to.
        value (int): The 16-bit value to write.

    Returns:
        bool: True on success, False on failure.
    """
    if utils.ser is None:
        print(f"[Pi] Serial port not initialized for writing register {hex(register_address)}.")
        return False

    # The value for position correction can be signed (-2047 to 2047).
    # Other word values (like limits) are unsigned (0-4095).
    # The servo protocol handles signed values using standard two's complement representation
    # within the 16-bit space, so we can just send the low and high bytes.
    val_clamped = int(value)

    packet = bytearray(9)
    packet[0] = SERVO_HEADER
    packet[1] = SERVO_HEADER
    packet[2] = servo_id
    packet[3] = 5  # Packet Length: Instr(1) + Addr(1) + Data(2) + 2 = 5
    packet[4] = SERVO_INSTRUCTION_WRITE
    packet[5] = register_address
    packet[6] = val_clamped & 0xFF      # Low byte
    packet[7] = (val_clamped >> 8) & 0xFF # High byte
    
    packet_data_for_sum = packet[2:8]
    packet[8] = calculate_checksum(packet_data_for_sum)

    try:
        utils.ser.write(packet)
        # print(f"[Pi] Servo {servo_id} register {hex(register_address)} set to {val_clamped}")
        return True
    except Exception as e:
        print(f"[Pi] Error writing word to servo {servo_id} register {hex(register_address)}: {e}")
        return False


def write_servo_register_byte(servo_id: int, register_address: int, value: int) -> bool:
    """
    Helper function to write a single byte to a servo register.

    Args:
        servo_id (int): The hardware ID of the target servo.
        register_address (int): The address of the register to write to.
        value (int): The 8-bit value to write.

    Returns:
        bool: True on success, False on failure.
    """
    if utils.ser is None:
        print(f"[Pi] Serial port not initialized for writing register {hex(register_address)}.")
        return False

    # Clamp value to byte range
    val_clamped = int(max(0, min(255, value)))

    packet = bytearray(8)
    packet[0] = SERVO_HEADER
    packet[1] = SERVO_HEADER
    packet[2] = servo_id
    packet[3] = 4  # Packet Length (Instr + Addr + Value + Checksum = 2 + 1 + 1)
    packet[4] = SERVO_INSTRUCTION_WRITE
    packet[5] = register_address
    packet[6] = val_clamped
    packet_data_for_sum = packet[2:7]
    packet[7] = calculate_checksum(packet_data_for_sum)

    try:
        utils.ser.write(packet)
        # print(f"[Pi] Servo {servo_id} register {hex(register_address)} set to {val_clamped}")
        return True
    except Exception as e:
        print(f"[Pi] Error writing to servo {servo_id} register {hex(register_address)}: {e}")
        return False


def sync_write_goal_pos_speed_accel(servo_data_list: list[tuple[int, int, int, int]]):
    """
    Constructs and sends a 'SYNC WRITE' packet to command multiple servos simultaneously.
    This is highly efficient as it bundles all commands into a single serial transmission.

    Args:
        servo_data_list: A list of tuples, where each tuple contains:
                         (servo_id, position_value, speed_value, acceleration_register_value)
    """
    if utils.ser is None or not utils.ser.is_open:
        print("[Pi SyncWrite] Serial port not initialized.")
        return

    num_servos = len(servo_data_list)
    if num_servos == 0:
        # print("[Pi SyncWrite] No servo data to send.")
        return

    packet_len_field_value = num_servos * (1 + SYNC_WRITE_DATA_LEN_PER_SERVO) + 4

    # Total packet size = Header(2) + BroadcastID(1) + PacketLenField(1) + ContentDescribedByPacketLenField + Checksum(1)
    total_packet_bytes = 4 + packet_len_field_value + 1

    packet = bytearray(total_packet_bytes)
    packet[0] = SERVO_HEADER
    packet[1] = SERVO_HEADER
    packet[2] = SERVO_BROADCAST_ID
    packet[3] = packet_len_field_value

    packet[4] = SERVO_INSTRUCTION_SYNC_WRITE
    packet[5] = SYNC_WRITE_START_ADDRESS
    packet[6] = SYNC_WRITE_DATA_LEN_PER_SERVO

    current_byte_index = 7
    for servo_id, pos_val, speed_val, accel_reg_val in servo_data_list:
        packet[current_byte_index] = servo_id
        current_byte_index += 1

        # Data order: Accel (1), Pos_L (1), Pos_H (1), Time_L (1), Time_H (1), Spd_L (1), Spd_H (1)
        packet[current_byte_index] = accel_reg_val # Accel (for SYNC_WRITE_START_ADDRESS, which is Accel address)
        current_byte_index += 1

        packet[current_byte_index] = pos_val & 0xFF # Pos_L (for ...START_ADDRESS + 1)
        current_byte_index += 1
        packet[current_byte_index] = (pos_val >> 8) & 0xFF # Pos_H (for ...START_ADDRESS + 2)
        current_byte_index += 1

        packet[current_byte_index] = 0 # Time_L (for ...START_ADDRESS + 3) - always 0
        current_byte_index += 1
        packet[current_byte_index] = 0 # Time_H (for ...START_ADDRESS + 4) - always 0
        current_byte_index += 1

        packet[current_byte_index] = speed_val & 0xFF # Speed_L (for ...START_ADDRESS + 5)
        current_byte_index += 1
        packet[current_byte_index] = (speed_val >> 8) & 0xFF # Speed_H (for ...START_ADDRESS + 6)
        current_byte_index += 1

    # Checksum is calculated from Broadcast_ID (packet[2]) up to the last data byte written into the packet.
    checksum_data = packet[2:current_byte_index]
    packet[current_byte_index] = calculate_checksum(checksum_data)

    final_packet_to_send = packet[0 : current_byte_index + 1]

    try:
        utils.ser.write(final_packet_to_send)
        # print(f"[Pi SyncWrite] Sent {len(final_packet_to_send)} bytes: {list(final_packet_to_send)}")
    except Exception as e:
        print(f"[Pi SyncWrite] Error: {e}")


def calibrate_servo_middle_position(servo_id: int) -> bool:
    """
    Sends a parameter-less 'Calibrate Middle Position' (0x0B) command to a servo.
    This instructs the servo to treat its current physical position as the new
    center point (raw value 2048).

    Args:
        servo_id (int): The hardware ID of the target servo.
    
    Returns:
        bool: True on success, False on failure.
    """
    if utils.ser is None:
        print(f"[Pi] Serial port not initialized for calibration command.")
        return False

    # Packet: [0xFF, 0xFF, ID, Length=2, Instr=0x0B, Checksum]
    packet = bytearray(6)
    packet[0] = SERVO_HEADER
    packet[1] = SERVO_HEADER
    packet[2] = servo_id
    packet[3] = 2 # Packet Length = NumParams(0) + 2
    packet[4] = SERVO_INSTRUCTION_CALIBRATE_MIDDLE
    
    packet_data_for_sum = packet[2:5] # ID, Len, Instr
    packet[5] = calculate_checksum(packet_data_for_sum)

    try:
        utils.ser.write(packet)
        return True
    except Exception as e:
        print(f"[Pi] Error writing calibration command to servo {servo_id}: {e}")
        return False


def factory_reset_servo(servo_id: int) -> bool:
    """
    Sends a 'Factory Reset' (0x06) command to a servo.
    This resets all EEPROM values (like PID, offsets, limits) to their 
    factory defaults, EXCEPT for the servo's ID.

    Args:
        servo_id (int): The hardware ID of the target servo.
    
    Returns:
        bool: True on success, False on failure.
    """
    if utils.ser is None:
        print(f"[Pi] Serial port not initialized for factory reset command.")
        return False

    # Packet: [0xFF, 0xFF, ID, Length=2, Instr=0x06, Checksum]
    packet = bytearray(6)
    packet[0] = SERVO_HEADER
    packet[1] = SERVO_HEADER
    packet[2] = servo_id
    packet[3] = 2  # Packet Length = NumParams(0) + 2
    packet[4] = SERVO_INSTRUCTION_RESET
    
    packet_data_for_sum = packet[2:5]  # ID, Len, Instr
    packet[5] = calculate_checksum(packet_data_for_sum)

    try:
        utils.ser.write(packet)
        # We don't get a response for a reset command.
        return True
    except Exception as e:
        print(f"[Pi] Error writing factory reset command to servo {servo_id}: {e}")
        return False


def restart_servo(servo_id: int) -> bool:
    """
    Sends a 'Restart' (0x08) command to a servo.
    This is equivalent to a power cycle.

    Args:
        servo_id (int): The hardware ID of the target servo.

    Returns:
        bool: True on success, False on failure.
    """
    if utils.ser is None:
        print(f"[Pi] Serial port not initialized for restart command.")
        return False

    # Packet: [0xFF, 0xFF, ID, Length=2, Instr=0x08, Checksum]
    packet = bytearray(6)
    packet[0] = SERVO_HEADER
    packet[1] = SERVO_HEADER
    packet[2] = servo_id
    packet[3] = 2  # Packet Length = NumParams(0) + 2
    packet[4] = SERVO_INSTRUCTION_RESTART

    packet_data_for_sum = packet[2:5]  # ID, Len, Instr
    packet[5] = calculate_checksum(packet_data_for_sum)

    try:
        utils.ser.write(packet)
        # No response for this command either.
        return True
    except Exception as e:
        print(f"[Pi] Error writing restart command to servo {servo_id}: {e}")
        return False


def sync_read_positions(servo_ids: list[int]) -> dict[int, int] | None:
    """
    Reads the present position of multiple servos using a single 'SYNC READ' command.
    This is significantly faster than reading one by one, enabling high-frequency feedback.

    Args:
        servo_ids (list): A list of servo IDs to read from.

    Returns:
        dict[int, int] | None: A dictionary mapping servo_id to its raw position,
                                or None if the read fails at any stage.
    """
    if utils.ser is None or not utils.ser.is_open:
        print("[Pi SyncRead] Serial port not initialized.")
        return None

    num_servos = len(servo_ids)
    if num_servos == 0:
        return {}

    # --- Construct the Sync Read Packet ---
    # Packet: [0xFF, 0xFF, Broadcast_ID, Len, Instr, StartAddr, DataLen, ID1, ID2, ..., Checksum]
    packet_len_field_value = num_servos + 4
    
    packet = bytearray(7 + num_servos + 1) # Header(2) + BcastID(1) + Len(1) + Instr(1) + Addr(1) + DataLen(1) + IDs(N) + Checksum(1)
    packet[0] = SERVO_HEADER
    packet[1] = SERVO_HEADER
    packet[2] = SERVO_BROADCAST_ID
    packet[3] = packet_len_field_value
    packet[4] = SERVO_INSTRUCTION_SYNC_READ
    packet[5] = SERVO_ADDR_PRESENT_POSITION # Start Address to read from (0x38)
    packet[6] = 2 # Length of data to read per servo (Pos_L, Pos_H)

    # Add all the servo IDs to the packet
    for i, servo_id in enumerate(servo_ids):
        packet[7 + i] = servo_id

    # Calculate checksum on the instruction parameters
    checksum_data = packet[2:-1] # From Broadcast_ID to last servo ID
    packet[-1] = calculate_checksum(checksum_data)

    # --- Send Command and Process Responses ---
    try:
        utils.ser.reset_input_buffer()
        utils.ser.write(packet)

        positions = {}
        expected_ids = set(servo_ids)

        for i in range(num_servos):
            # Read one 8-byte status packet at a time.
            # The serial port's read timeout (set during initialization) will handle cases
            # where a servo doesn't respond.
            response = utils.ser.read(8)

            if len(response) < 8:
                responded_ids = set(positions.keys())
                missing_ids = expected_ids - responded_ids
                print(f"[Pi SyncRead] Timed out waiting for packet {i+1}/{num_servos}. "
                      f"Received {len(response)} bytes. Got responses for {list(responded_ids)}. "
                      f"Still waiting on {list(missing_ids)}.")
                return None

            # Basic validation
            if response[0] != SERVO_HEADER or response[1] != SERVO_HEADER:
                print(f"[Pi SyncRead] Invalid header in response packet {i+1}. Got: {list(response)}")
                continue # Try to read the next packet

            response_id = response[2]
            if response_id not in expected_ids:
                print(f"[Pi SyncRead] Received response for unexpected servo ID {response_id} in packet {i+1}.")
                continue

            # Error byte validation
            if response[4] != 0:
                print(f"[Pi SyncRead] Servo {response_id} reported error: {response[4]}")
                return None # Fail on first error

            # Checksum validation
            expected_checksum = calculate_checksum(response[2:7])
            if expected_checksum != response[7]:
                print(f"[Pi SyncRead] Checksum mismatch for servo {response_id}.")
                return None # Fail on checksum error

            position = int.from_bytes(response[5:7], byteorder='little', signed=True)
            positions[response_id] = position

        # Final check to see if we got a valid response for every servo
        if len(positions) != num_servos:
            responded_ids = set(positions.keys())
            missing_ids = expected_ids - responded_ids
            print(f"[Pi SyncRead] Finished read loop, but missing data for IDs: {list(missing_ids)}")
            return None

        return positions

    except Exception as e:
        print(f"[Pi SyncRead] Error during Sync Read: {e}")
        return None



def initialize_servos():
    """
    Initializes the serial connection to the servos and sets their default PID gains.
    This function must be called once at application startup.
    """
    print("[Pi] Initializing servos...")
    try:
        utils.ser = serial.Serial(utils.SERIAL_PORT, utils.BAUD_RATE, timeout=0.1)
        print(f"[Pi] Serial port {utils.SERIAL_PORT} opened successfully at {utils.BAUD_RATE} baud.")
        
        # On startup, immediately read and display the hardware zero offsets for verification.
        print("[Pi] Reading stored hardware zero offsets from all servos...")
        offsets = get_servo_hardware_zero_offsets()
        for i, offset in enumerate(offsets):
            print(f"[Pi]   - Servo {utils.SERVO_IDS[i]}: Offset = {offset}")

        # Set default PID gains for all servos upon initialization
        # These are EEPROM values, so they will persist.
        # Ensure EEPROM is writable (typically it is for PID unless explicitly locked by other means).
        # Register 0x37 (WRITE_LOCK) is 0 by default (writable).
        print("[Pi] Setting default PID gains for all servos...")
        all_pid_set_successfully = True
        for s_id in utils.SERVO_IDS:
            if not set_servo_pid_gains(s_id, utils.DEFAULT_KP, utils.DEFAULT_KI, utils.DEFAULT_KD):
                all_pid_set_successfully = False
            time.sleep(0.05) # Give a bit more time after each servo's PID set
        
        if all_pid_set_successfully:
            print("[Pi] Default PID gains set for all servos.")
        else:
            print("[Pi] WARNING: Failed to set PID gains for one or more servos.")
            print("[Pi] Check servo connections and power.")

    except serial.SerialException as e:
        print(f"[Pi] Error opening serial port {utils.SERIAL_PORT}: {e}")
        print("Please ensure the serial port is correct, available, and you have permissions.")
        print("You might need to run 'sudo raspi-config' and:")
        print("  1. Go to 'Interface Options'")
        print("  2. Go to 'Serial Port'")
        print("  3. Answer 'No' to 'Would you like a login shell to be accessible over serial?'")
        print("  4. Answer 'Yes' to 'Would you like the serial port hardware to be enabled?'")
        print("  5. Reboot your Pi.")
        exit() # Exit if we can't open the serial port
    print("[Pi] Servos initialized.")


def set_servo_positions(logical_joint_angles_rad: list[float], speed_value: int, acceleration_value_deg_s2: float):
    """
    Translates a list of 6 logical joint angles into the corresponding commands
    for the 9 physical servos and sends them in a single SYNC WRITE packet.

    This function handles:
    - Applying master calibration offsets.
    - Applying the 1:2 gear ratio for the base joint.
    - Converting radian angles to raw servo values (0-4095), handling servo inversion.
    - Updating the global state `current_logical_joint_angles_rad`.

    Args:
        logical_joint_angles_rad (list[float]): A list of 6 joint angles in radians.
        speed_value (int): The speed for the move (0-4095).
        acceleration_value_deg_s2 (float): The acceleration for the move in deg/s^2.
    """
    if utils.ser is None:
        print("[Pi] Serial port not initialized, cannot set positions.")
        return

    # Get the latest hardware zero offsets. This is important in case they
    # were changed dynamically (e.g., via a 'set_zero' command).
    servo_hardware_zero_offsets_raw = get_servo_hardware_zero_offsets()

    # The global `_calculated_servo_radian_offsets` is no longer used.
    if len(logical_joint_angles_rad) != utils.NUM_LOGICAL_JOINTS:
        print(f"[Pi] Error: Expected {utils.NUM_LOGICAL_JOINTS} logical joint angles, got {len(logical_joint_angles_rad)}")
        return

    # --- Update the global state of the arm's logical joint angles ---
    # We store the commanded angles before any clamping or offsets are applied,
    # as this represents the "ideal" state for the IK solver.
    utils.current_logical_joint_angles_rad = list(logical_joint_angles_rad)
    # ---

    commands_for_sync_write = []

    # Calculate the single acceleration register value to be used for all servos in this command cycle
    accel_reg_val_for_cycle = 0
    if acceleration_value_deg_s2 > 0:
        accel_reg_val_for_cycle = int(round(acceleration_value_deg_s2 / utils.ACCELERATION_SCALE_FACTOR))
        accel_reg_val_for_cycle = max(1, min(254, accel_reg_val_for_cycle)) # if >0, use 1-254
    else:
        accel_reg_val_for_cycle = 0 # Explicitly set to 0 for max physical acceleration

    # Clamp the global speed value once
    clamped_speed_value_for_cycle = int(max(0, min(4095, speed_value)))

    logical_to_physical_map = {
        0: [0, 1],  # Logical J1 -> Physical Servo IDs 10, 11
        1: [2, 3],  # Logical J2 -> Physical Servo IDs 20, 21
        2: [4, 5],  # Logical J3 -> Physical Servo IDs 30, 31
        3: [6],     # Logical J4 -> Physical Servo ID 40
        4: [7],     # Logical J5 -> Physical Servo ID 50
        5: [8],     # Logical J6 -> Physical Servo ID 60
    }

    for logical_joint_index in range(utils.NUM_LOGICAL_JOINTS):
        # Master offset is still useful for high-level adjustments
        angle_to_be_processed = logical_joint_angles_rad[logical_joint_index] + utils.LOGICAL_JOINT_MASTER_OFFSETS_RAD[logical_joint_index]
        
        target_physical_angle_rad = angle_to_be_processed

        if logical_joint_index == 0:
            target_physical_angle_rad *= 2.0
            
        for physical_servo_config_index in logical_to_physical_map[logical_joint_index]:
            current_physical_servo_id = utils.SERVO_IDS[physical_servo_config_index]

            # No extra sign inversion here—orientation differences are fully
            # captured by the direct/inverted mapping in utils._is_servo_direct_mapping.
            final_target_physical_angle_rad = target_physical_angle_rad

            # --- Convert Target Angle (Radians) to Raw Servo Value (0-4095) ---
            
            # 1. Apply the servo's individual hardware offset. This is the value stored
            #    on the servo's EEPROM, which defines its "zero" position.
            current_servo_hardware_offset_raw = servo_hardware_zero_offsets_raw[physical_servo_config_index]

            # 2. Normalize the angle to a 0-1 range based on its mapping
            #    Most servos are direct (angle increases with raw value), but some are inverted.
            min_map_rad, max_map_rad = utils.EFFECTIVE_MAPPING_RANGES[physical_servo_config_index]

            # Clamp the angle to the effective mapping range before normalization
            angle_for_norm = max(min_map_rad, min(max_map_rad, final_target_physical_angle_rad))
            
            normalized_value = (angle_for_norm - min_map_rad) / (max_map_rad - min_map_rad)

            # Apply mapping direction and add the hardware zero offset
            if utils._is_servo_direct_mapping(physical_servo_config_index):
                raw_servo_value = normalized_value * 4095.0
            else: 
                raw_servo_value = (1.0 - normalized_value) * 4095.0
            
            # This is the CRITICAL step. The hardware offset defines the servo's zero point.
            # We add it to the calculated raw value to get the final, corrected target position.
            final_servo_pos_value = int(round(raw_servo_value + current_servo_hardware_offset_raw))
            final_servo_pos_value = max(0, min(4095, final_servo_pos_value))

            # Add command data for this servo to the list for Sync Write
            commands_for_sync_write.append((
                current_physical_servo_id, 
                final_servo_pos_value, 
                clamped_speed_value_for_cycle, 
                accel_reg_val_for_cycle
            ))
            
            # Disabling this high-frequency print to improve loop performance
            print(f"[Pi PrepSync] Servo {current_physical_servo_id} (Log.J{logical_joint_index+1}): TargetPos={final_servo_pos_value} Spd={clamped_speed_value_for_cycle} AccelReg={accel_reg_val_for_cycle} (from {acceleration_value_deg_s2:.1f} deg/s^2)")

    # After collecting all commands, send them in a single Sync Write packet
    if commands_for_sync_write:
        servo_protocol.sync_write_goal_pos_speed_accel(commands_for_sync_write)
    else:
        # print("[Pi] No servo commands to send in set_servo_positions (SyncWrite)." )
        pass


def servo_value_to_radians(servo_value: int, physical_servo_config_index: int) -> float:
    """
    Converts a raw servo value (0-4095) back into a physical angle in radians.
    This is the inverse of the conversion logic in `set_servo_positions`.

    IMPORTANT: This function assumes the incoming `servo_value` has already had
    the hardware zero offset subtracted from it. It operates on a value
    relative to the theoretical center.

    Args:
        servo_value (int): The **offset-corrected** raw position value from the servo.
        physical_servo_config_index (int): The 0-based index of the servo in the `SERVO_IDS` list.

    Returns:
        float: The calculated physical angle of the servo in radians.
    """
    if not (0 <= physical_servo_config_index < utils.NUM_PHYSICAL_SERVOS): # NUM_PHYSICAL_SERVOS is now 9
        print(f"[Pi ConvertRad] Invalid physical_servo_config_index {physical_servo_config_index}")
        return 0.0

    if servo_value is None:
        return 0.0 # Or some indicator of failure

    # Ensure servo_value is within expected bounds for safety in calculation, though it should be.
    servo_value = max(0, min(4095, servo_value))

    min_map_rad, max_map_rad = utils.EFFECTIVE_MAPPING_RANGES[physical_servo_config_index]

    # 1. Normalize the servo value from [0, 4095] to [0, 1]
    # Undo inversion based on the physical_servo_config_index
    is_direct_mapping = utils._is_servo_direct_mapping(physical_servo_config_index)

    if is_direct_mapping:
        normalized_direct = servo_value / 4095.0
    else: # Standard inverted mapping
        normalized_inverted = servo_value / 4095.0
        normalized_direct = 1.0 - normalized_inverted
    
    # 3. De-normalize this direct value back to the radian range for that joint.
    angle_rad = normalized_direct * (max_map_rad - min_map_rad) + min_map_rad
    
    return angle_rad


def set_servo_pid_gains(servo_id: int, kp: int, ki: int, kd: int) -> bool:
    """
    Sets the Kp, Ki, and Kd PID gains for a specific servo.
    These values are written to the servo's EEPROM and will persist.

    Args:
        servo_id (int): The hardware ID of the target servo.
        kp (int): The Proportional gain value.
        ki (int): The Integral gain value.
        kd (int): The Derivative gain value.

    Returns:
        bool: True on success, False on failure.
    """
    print(f"[Pi] Setting PID for Servo {servo_id}: Kp={kp}, Ki={ki}, Kd={kd}")
    # Note: PID registers are in EEPROM. Need to ensure EEPROM is unlocked if necessary.
    # The STSServoDriver.cpp unlocks EEPROM (reg 0x37) before writing to ID (0x05) or PosCorrection (0x1F).
    # For standard PID registers, direct write might be okay if servo isn't locked by default for these.
    # Let's assume direct writes are fine for PID for now, as they are common tuning parameters.

    success = True
    if not servo_protocol.write_servo_register_byte(servo_id, servo_protocol.SERVO_ADDR_POS_KP, kp):
        success = False
        print(f"[Pi] Failed to set Kp for servo {servo_id}")
    time.sleep(0.01) # Small delay between writes

    if not servo_protocol.write_servo_register_byte(servo_id, servo_protocol.SERVO_ADDR_POS_KI, ki):
        success = False
        print(f"[Pi] Failed to set Ki for servo {servo_id}")
    time.sleep(0.01)

    if not servo_protocol.write_servo_register_byte(servo_id, servo_protocol.SERVO_ADDR_POS_KD, kd):
        success = False
        print(f"[Pi] Failed to set Kd for servo {servo_id}")
    time.sleep(0.01)

    if success:
        print(f"[Pi] Successfully set PID for Servo {servo_id}")
    else:
        print(f"[Pi] PID setting partially/fully FAILED for Servo {servo_id}")
    return success


def set_servo_angle_limits_from_urdf():
    """
    Sets the min/max angle limits in each servo's EEPROM based on `URDF_JOINT_LIMITS`.
    This is a crucial safety feature to prevent the arm from damaging itself.
    This operation requires unlocking the servo EEPROM for writing.
    """
    print("[Pi] Setting hardware angle limits for all servos from URDF config...")
    
    # Define register addresses from documentation
    SERVO_ADDR_MIN_ANGLE_LIMIT = 0x09
    SERVO_ADDR_MAX_ANGLE_LIMIT = 0x0B
    SERVO_ADDR_WRITE_LOCK = 0x37

    all_limits_set_successfully = True
    for i in range(utils.NUM_PHYSICAL_SERVOS):
        servo_id = utils.SERVO_IDS[i]
        min_urdf_rad, max_urdf_rad = utils.URDF_JOINT_LIMITS[i]

        # Convert URDF radian limits to raw servo values (0-4095)
        # This conversion must be the inverse of `servo_value_to_radians`
        min_map_rad, max_map_rad = utils.EFFECTIVE_MAPPING_RANGES[i]
        
        # --- Min Angle Conversion ---
        norm_min = (min_urdf_rad - min_map_rad) / (max_map_rad - min_map_rad)
        if utils._is_servo_direct_mapping(i):
            raw_min = norm_min * 4095.0
        else:
            raw_min = (1.0 - norm_min) * 4095.0
        
        # --- Max Angle Conversion ---
        norm_max = (max_urdf_rad - min_map_rad) / (max_map_rad - min_map_rad)
        if utils._is_servo_direct_mapping(i):
            raw_max = norm_max * 4095.0
        else:
            raw_max = (1.0 - norm_max) * 4095.0

        # The raw values might be inverted (e.g., min > max), so we must order them correctly.
        final_min_raw = int(round(min(raw_min, raw_max)))
        final_max_raw = int(round(max(raw_min, raw_max)))

        # Clamp to the servo's absolute possible range
        final_min_raw = max(0, min(4095, final_min_raw))
        final_max_raw = max(0, min(4095, final_max_raw))

        print(f"[Pi] Servo {servo_id}: Setting HW limits. URDF(rad): [{min_urdf_rad:.2f}, {max_urdf_rad:.2f}] -> RAW: [{final_min_raw}, {final_max_raw}]")

        # --- Write to Servo ---
        # 1. Unlock EEPROM
        if not servo_protocol.write_servo_register_byte(servo_id, SERVO_ADDR_WRITE_LOCK, 0):
            print(f"[Pi] Failed to unlock EEPROM for servo {servo_id}. Skipping limit set.")
            all_limits_set_successfully = False
            continue
        time.sleep(0.01)

        # 2. Write Min Angle Limit
        if not servo_protocol.write_servo_register_word(servo_id, SERVO_ADDR_MIN_ANGLE_LIMIT, final_min_raw):
            print(f"[Pi] Failed to set MIN angle limit for servo {servo_id}")
            all_limits_set_successfully = False
        time.sleep(0.01)

        # 3. Write Max Angle Limit
        if not servo_protocol.write_servo_register_word(servo_id, SERVO_ADDR_MAX_ANGLE_LIMIT, final_max_raw):
            print(f"[Pi] Failed to set MAX angle limit for servo {servo_id}")
            all_limits_set_successfully = False
        time.sleep(0.01)

        # 4. Lock EEPROM
        if not servo_protocol.write_servo_register_byte(servo_id, SERVO_ADDR_WRITE_LOCK, 1):
            print(f"[Pi] WARNING: Failed to re-lock EEPROM for servo {servo_id}.")
        time.sleep(0.02) # Extra delay after lock

    if all_limits_set_successfully:
        print("[Pi] Hardware angle limits set for all servos.")
    else:
        print("[Pi] WARNING: Failed to set hardware angle limits for one or more servos.")


def get_current_arm_state_rad(verbose: bool = True) -> list[float]:
    """
    Reads the current position of all logical joints by polling the physical servos.

    This function uses the efficient `sync_read_positions` protocol command
    to get feedback from all servos in a single transaction, making it suitable
    for high-frequency state updates.

    Args:
        verbose (bool, optional): Whether to print debug information. Defaults to True.

    Returns:
        list[float]: A list of the 6 current logical joint angles in radians.
    """
    if verbose:
        print("[Pi] Reading current arm state from servos...")

    # ------------------------------------------------------------------
    # Bulk feedback using Sync Read for better speed and determinism
    # ------------------------------------------------------------------
    all_servo_ids = utils.SERVO_IDS
    raw_positions_dict = servo_protocol.sync_read_positions(all_servo_ids)
    
    # Get the hardware zero offsets to correctly interpret the raw position values.
    # This is the critical inverse of the logic in `set_servo_positions`.
    servo_hardware_zero_offsets_raw = get_servo_hardware_zero_offsets()

    current_angles_rad = list(utils.current_logical_joint_angles_rad)  # Start with last known good state

    # Mapping: which physical index (in utils.SERVO_IDS) is the authoritative
    # feedback source for each logical joint.
    logical_to_primary_physical_map = {
        0: 0,  # J1 -> ID 10
        1: 2,  # J2 -> ID 20
        2: 4,  # J3 -> ID 30
        3: 6,  # J4 -> ID 40
        4: 7,  # J5 -> ID 50
        5: 8,  # J6 -> ID 60
    }

    for logical_idx, physical_idx in logical_to_primary_physical_map.items():
        servo_id = utils.SERVO_IDS[physical_idx]
        raw_pos = None if raw_positions_dict is None else raw_positions_dict.get(servo_id)

        if raw_pos is not None:
            # Apply the inverse of the command logic: subtract the offset before converting.
            offset = servo_hardware_zero_offsets_raw[physical_idx]
            corrected_raw_pos = raw_pos - offset
            
            physical_angle_rad = servo_value_to_radians(corrected_raw_pos, physical_idx)

            logical_angle_rad = physical_angle_rad

            # Compensate for base gear ratio 2:1
            if logical_idx == 0:
                logical_angle_rad /= 2.0

            current_angles_rad[logical_idx] = logical_angle_rad
            if verbose:
                print(
                    f"[Pi State] Logical J{logical_idx+1} (Servo {servo_id}): Raw={raw_pos} (Offset={offset}, Corrected={corrected_raw_pos}), Angle={logical_angle_rad:.3f} rad"
                )
        else:
            if verbose:
                print(
                    f"[Pi State] FAILED to read feedback for servo {servo_id}. Using previous value for J{logical_idx+1}."
                )

    # Update global state with the newly read values
    utils.current_logical_joint_angles_rad = current_angles_rad
    
    if verbose:
        print(f"[Pi] Arm state read. Angles (rad): {np.round(current_angles_rad, 3)}")

    return current_angles_rad


def set_current_position_as_hardware_zero(servo_id: int):
    """
    Sets the servo's current physical position as its new hardware zero point.
    This is the definitive, jump-free, torque-on procedure for calibration.
    It uses the native 'Calibrate Middle' (0x0B) servo command.

    NOTE: This EEPROM write will cause a brief, unavoidable torque release.
    The arm is commanded to hold its position after the write to re-engage torque.

    Args:
        servo_id (int): The hardware ID of the servo to zero.
    """
    print(f"--- Starting Definitive Zero-Set for Servo {servo_id} ---")
    
    # 1. Send the native calibration command.
    print(f"[Pi] Servo {servo_id}: Sending native 'Calibrate Middle' (0x0B) command...")
    if not servo_protocol.calibrate_servo_middle_position(servo_id):
        print(f"[Pi] SET_ZERO FAILED: Could not send calibration command to servo {servo_id}.")
        return
    
    print(f"[Pi] Calibration command sent to servo {servo_id}.")
    
    # Give the servo time to process the EEPROM write.
    time.sleep(0.1)

    # 2. CRITICAL: Re-engage torque to prevent falling.
    # We command the servo to its *new* theoretical center (2048).
    # Since the 0x0B command just made the current position the center,
    # this command should result in the servo holding its current position.
    print(f"[Pi] Servo {servo_id}: Sending 'hold position' command to re-engage torque.")
    servo_protocol.send_servo_command(servo_id, 2048, 100)
    time.sleep(0.05)

    # 3. Verify the result by reading the offset back.
    print(f"[Pi] Servo {servo_id}: Verifying new offset stored in servo...")
    verified_offset = None
    for i in range(3): # Retry loop
        time.sleep(0.1)
        verified_offset = servo_protocol.read_servo_register_signed_word(servo_id, utils.SERVO_ADDR_POSITION_CORRECTION)
        if verified_offset is not None:
            break
        print(f"[Pi] Servo {servo_id}: Verification read attempt {i+1} failed. Retrying...")

    if verified_offset is not None:
         print(f"[Pi] SET_ZERO SUCCESS: Servo {servo_id} confirmed new offset written: {verified_offset}.")
    else:
        print(f"[Pi] SET_ZERO WARNING: Wrote calibration, but could not verify offset read-back.")

    print(f"--- Finished Definitive Zero-Set for Servo {servo_id} ---")


def reset_all_servo_offsets_to_zero():
    """
    Resets the 'Position Correction' EEPROM register to 0 for all servos.
    This provides a clean slate for calibration.
    """
    print("--- Resetting ALL servo hardware offsets to 0 ---")
    SERVO_ADDR_WRITE_LOCK = 0x37
    all_reset_ok = True

    for servo_id in utils.SERVO_IDS:
        print(f"[Pi] Resetting offset for servo {servo_id}...")
        
        # 1. Unlock EEPROM
        if not servo_protocol.write_servo_register_byte(servo_id, SERVO_ADDR_WRITE_LOCK, 0):
            print(f"[Pi] FAILED to unlock EEPROM for servo {servo_id}.")
            all_reset_ok = False
            continue
        time.sleep(0.01)

        # 2. Write 0 to the Position Correction register
        if not servo_protocol.write_servo_register_word(servo_id, utils.SERVO_ADDR_POSITION_CORRECTION, 0):
            print(f"[Pi] FAILED to write offset for servo {servo_id}.")
            all_reset_ok = False
        time.sleep(0.01)

        # 3. Re-lock EEPROM
        if not servo_protocol.write_servo_register_byte(servo_id, SERVO_ADDR_WRITE_LOCK, 1):
            print(f"[Pi] WARNING: Failed to re-lock EEPROM for servo {servo_id}.")
        
        # 4. Verify the write, retrying on failure to handle busy servos.
        verified_offset = None
        for i in range(3):
            time.sleep(0.1) # Increased delay to allow EEPROM write to settle
            verified_offset = servo_protocol.read_servo_register_signed_word(servo_id, utils.SERVO_ADDR_POSITION_CORRECTION)
            if verified_offset is not None:
                break # Success
            print(f"[Pi] Servo {servo_id}: Verification read attempt {i+1} failed. Retrying...")

        if verified_offset == 0:
            print(f"[Pi] Servo {servo_id} offset successfully reset to 0.")
        else:
            print(f"[Pi] FAILED to verify reset for servo {servo_id}! Read back {verified_offset}.")
            all_reset_ok = False
        
        time.sleep(0.02)

    if all_reset_ok:
        print("--- All servo offsets have been reset to 0. ---")
    else:
        print("--- WARNING: Failed to reset one or more servo offsets. ---")


def reinitialize_servo(servo_id: int):
    """
    Re-applies critical settings (PID gains, angle limits) to a single servo.
    This is useful after a factory reset to bring a servo back to a known-good state.

    Args:
        servo_id (int): The hardware ID of the servo to re-initialize.
    """
    print(f"[Pi] Re-initializing servo {servo_id} with application defaults...")
    
    try:
        # Find the configuration index for this servo ID
        physical_servo_config_index = utils.SERVO_IDS.index(servo_id)
    except ValueError:
        print(f"[Pi] ERROR: Cannot re-initialize servo. ID {servo_id} not found in SERVO_IDS list.")
        return

    # 1. Set default PID gains
    print(f"[Pi]   - Setting PID gains for servo {servo_id}...")
    pid_success = set_servo_pid_gains(servo_id, utils.DEFAULT_KP, utils.DEFAULT_KI, utils.DEFAULT_KD)
    if not pid_success:
        print(f"[Pi]   - WARNING: Failed to set PID gains for servo {servo_id}.")
        # We can continue, but the servo might not be stable.

    time.sleep(0.05) # Small delay

    # 2. Set Angle Limits
    print(f"[Pi]   - Setting angle limits for servo {servo_id}...")
    SERVO_ADDR_MIN_ANGLE_LIMIT = 0x09
    SERVO_ADDR_MAX_ANGLE_LIMIT = 0x0B
    SERVO_ADDR_WRITE_LOCK = 0x37

    min_urdf_rad, max_urdf_rad = utils.URDF_JOINT_LIMITS[physical_servo_config_index]

    min_map_rad, max_map_rad = utils.EFFECTIVE_MAPPING_RANGES[physical_servo_config_index]
    
    norm_min = (min_urdf_rad - min_map_rad) / (max_map_rad - min_map_rad)
    if utils._is_servo_direct_mapping(physical_servo_config_index):
        raw_min = norm_min * 4095.0
    else:
        raw_min = (1.0 - norm_min) * 4095.0
    
    norm_max = (max_urdf_rad - min_map_rad) / (max_map_rad - min_map_rad)
    if utils._is_servo_direct_mapping(physical_servo_config_index):
        raw_max = norm_max * 4095.0
    else:
        raw_max = (1.0 - norm_max) * 4095.0

    final_min_raw = int(round(min(raw_min, raw_max)))
    final_max_raw = int(round(max(raw_min, raw_max)))
    final_min_raw = max(0, min(4095, final_min_raw))
    final_max_raw = max(0, min(4095, final_max_raw))

    print(f"[Pi]   - Servo {servo_id} HW limits RAW: [{final_min_raw}, {final_max_raw}]")

    if not servo_protocol.write_servo_register_byte(servo_id, SERVO_ADDR_WRITE_LOCK, 0):
        print(f"[Pi]   - FAILED to unlock EEPROM for servo {servo_id}. Aborting limit set.")
        return # Can't proceed if EEPROM is locked

    time.sleep(0.01)
    servo_protocol.write_servo_register_word(servo_id, SERVO_ADDR_MIN_ANGLE_LIMIT, final_min_raw)
    time.sleep(0.01)
    servo_protocol.write_servo_register_word(servo_id, SERVO_ADDR_MAX_ANGLE_LIMIT, final_max_raw)
    time.sleep(0.01)
    if not servo_protocol.write_servo_register_byte(servo_id, SERVO_ADDR_WRITE_LOCK, 1):
        print(f"[Pi]   - WARNING: Failed to re-lock EEPROM for servo {servo_id}.")
    
    print(f"[Pi] Re-initialization for servo {servo_id} complete.")


def get_servo_hardware_zero_offsets() -> list[int]:
    """
    Reads the hardware 'zero' offset from the EEPROM of each physical servo.
    This value is what's set by the 'SET_ZERO' command. It can be positive or
    negative and is added to the calculated position before sending it to the servo.

    Returns:
        A list of 9 integer offset values, one for each physical servo.
    """
    offsets = [0] * utils.NUM_PHYSICAL_SERVOS
    for i, servo_id in enumerate(utils.SERVO_IDS):
        # The position correction is stored as a signed word (2 bytes)
        offset_val = servo_protocol.read_servo_register_signed_word(servo_id, utils.SERVO_ADDR_POSITION_CORRECTION)
        
        if offset_val is not None:
            offsets[i] = offset_val
        else:
            print(f"[Pi] WARNING: Failed to read hardware zero offset for servo {servo_id}. Using 0.")
            offsets[i] = 0 # Default to 0 on failure
            
    return offsets
