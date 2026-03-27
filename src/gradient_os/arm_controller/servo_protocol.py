# =============================================================================
# servo_protocol.py - DEPRECATED
# =============================================================================
#
# WARNING: This module is deprecated and will be removed in a future release.
# For new code, use the ActuatorBackend interface directly:
#
#     from gradient_os.arm_controller.backends import registry
#     backend = registry.get_active_backend()
#     backend.sync_read_positions(servo_ids)
#     backend.sync_write(commands)
#
# This module now acts as a thin dispatcher that routes calls to the active
# backend when available, falling back to direct serial communication for
# legacy compatibility.
#
# =============================================================================

from . import utils
from ..telemetry import alerts
import time
import threading
import warnings
from typing import Optional

# Import backend registry for servo-specific constants and backend access
from .backends import registry as backend_registry


def _get_backend():
    """Get the active backend instance, or None if not set."""
    try:
        return backend_registry.get_active_backend()
    except backend_registry.BackendInstanceNotSetError:
        return None


def _use_backend() -> bool:
    """Returns True if an ActuatorBackend is active and initialized."""
    backend = _get_backend()
    return backend is not None and backend.is_initialized


def _warn_deprecated(func_name: str):
    """Emit a deprecation warning for servo_protocol usage."""
    warnings.warn(
        f"servo_protocol.{func_name}() is deprecated. "
        f"Use backends.registry.get_active_backend() methods instead.",
        DeprecationWarning,
        stacklevel=3
    )


def _get_backend_config():
    """Get the active backend config. Raises if not configured."""
    return backend_registry.get_config()


def _serial_protocol_supported() -> bool:
    """Return whether the active backend config supports legacy serial packets."""
    try:
        cfg = _get_backend_config()
    except backend_registry.BackendNotConfiguredError:
        return True
    return bool(getattr(cfg, "SERVO_PROTOCOL_SUPPORTED", True))


# These are accessed as module-level constants but delegate to the backend.
# They're populated when first accessed after backend is configured.
def __getattr__(name):
    """Lazy attribute access for backend constants."""
    cfg = _get_backend_config()
    
    _BACKEND_ATTRS = {
        'SERVO_HEADER': 'SERVO_HEADER',
        'SERVO_INSTRUCTION_WRITE': 'SERVO_INSTRUCTION_WRITE',
        'SERVO_ADDR_TARGET_POSITION': 'SERVO_ADDR_TARGET_POSITION',
        'SERVO_INSTRUCTION_READ': 'SERVO_INSTRUCTION_READ',
        'SERVO_ADDR_PRESENT_POSITION': 'SERVO_ADDR_PRESENT_POSITION',
        'SERIAL_READ_TIMEOUT': 'SERIAL_READ_TIMEOUT',
        'SERVO_ADDR_TARGET_ACCELERATION': 'SERVO_ADDR_TARGET_ACCELERATION',
        'DEFAULT_SERVO_ACCELERATION_DEG_S2': 'DEFAULT_SERVO_ACCELERATION_DEG_S2',
        'ACCELERATION_SCALE_FACTOR': 'ACCELERATION_SCALE_FACTOR',
        'SERVO_ADDR_POS_KP': 'SERVO_ADDR_POS_KP',
        'SERVO_ADDR_POS_KI': 'SERVO_ADDR_POS_KI',
        'SERVO_ADDR_POS_KD': 'SERVO_ADDR_POS_KD',
        'DEFAULT_KP': 'DEFAULT_KP',
        'DEFAULT_KI': 'DEFAULT_KI',
        'DEFAULT_KD': 'DEFAULT_KD',
        'SERVO_INSTRUCTION_CALIBRATE_MIDDLE': 'SERVO_INSTRUCTION_CALIBRATE_MIDDLE',
        'SERVO_INSTRUCTION_RESET': 'SERVO_INSTRUCTION_RESET',
        'SERVO_INSTRUCTION_RESTART': 'SERVO_INSTRUCTION_RESTART',
        'SERVO_INSTRUCTION_PING': 'SERVO_INSTRUCTION_PING',
        'SERVO_ADDR_WRITE_LOCK': 'SERVO_ADDR_WRITE_LOCK',
        'SERVO_ADDR_MIN_ANGLE_LIMIT': 'SERVO_ADDR_MIN_ANGLE_LIMIT',
        'SERVO_ADDR_MAX_ANGLE_LIMIT': 'SERVO_ADDR_MAX_ANGLE_LIMIT',
        'SERVO_INSTRUCTION_SYNC_WRITE': 'SERVO_INSTRUCTION_SYNC_WRITE',
        'SERVO_BROADCAST_ID': 'SERVO_BROADCAST_ID',
        'SYNC_WRITE_START_ADDRESS': 'SYNC_WRITE_START_ADDRESS',
        'SYNC_WRITE_DATA_LEN_PER_SERVO': 'SYNC_WRITE_DATA_LEN_PER_SERVO',
        'SERVO_INSTRUCTION_SYNC_READ': 'SERVO_INSTRUCTION_SYNC_READ',
    }
    
    if name in _BACKEND_ATTRS:
        return getattr(cfg, _BACKEND_ATTRS[name])
    
    raise AttributeError(f"module '{__name__}' has no attribute '{name}'")


# A simple in-memory cache for which servos were detected at startup
_present_servo_ids: set[int] = set()

def get_present_servo_ids() -> set[int]:
    """Returns the set of servo IDs that responded to the initial ping."""
    return _present_servo_ids


# ---------------------------------------------------------------------------
# Telemetry buffer for Sync Read profiling (write, read, parse durations in s)
# ---------------------------------------------------------------------------

_sync_profiles: list[tuple[float, float, float]] = []  # populated when diagnostics enabled

# Global re-entrant lock to serialize all serial I/O across threads
_SERIAL_LOCK = threading.RLock()


def get_sync_profiles() -> list[tuple[float, float, float]]:
    """Return and clear the collected Sync-Read timing tuples (seconds)."""
    global _sync_profiles
    out = _sync_profiles[:]
    _sync_profiles.clear()
    return out


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


def ping(servo_id: int) -> bool:
    """
    Sends a PING instruction to a servo to check for its presence.

    Args:
        servo_id (int): The hardware ID of the servo to ping.

    Returns:
        bool: True if a valid status packet is received, False otherwise.
        
    .. deprecated::
        Use ``backend.ping_actuator(servo_id)`` instead.
    """
    # Dispatch to backend if available
    backend = _get_backend()
    if backend and hasattr(backend, 'ping_actuator'):
        result = backend.ping_actuator(servo_id)
        if result:
            _present_servo_ids.add(servo_id)
        return result
    
    # Fallback to direct serial communication
    if utils.ser is None or not utils.ser.is_open:
        return False

    # PING Packet: [0xFF, 0xFF, ID, Length=2, Instr=0x01, Checksum]
    # Length = (Num Instruction Parameters) + 2. Here: 0 params. 0+2=2.
    ping_command = bytearray(6)
    ping_command[0] = utils.SERVO_HEADER
    ping_command[1] = utils.SERVO_HEADER
    ping_command[2] = servo_id
    ping_command[3] = 2  # Length
    ping_command[4] = utils.SERVO_INSTRUCTION_PING
    
    # Checksum for ping command (ID, Length, Instr)
    ping_command[5] = calculate_checksum(ping_command[2:5])

    try:
        with _SERIAL_LOCK:
            utils.ser.reset_input_buffer()
            utils.ser.write(ping_command)
            
            # A successful ping should receive a status packet in response
            # Status Packet: [0xFF, 0xFF, ID, Length=2, Error=0, Checksum]
            # We'll just check if we get *any* valid response for the correct ID
            response = utils.ser.read(6) # Read the expected status packet length

        if len(response) == 6 and response[0] == 0xFF and response[1] == 0xFF and response[2] == servo_id:
            # We have a response from the correct servo. Add to cache.
            _present_servo_ids.add(servo_id)
            return True
            
        return False

    except Exception as e:
        print(f"[Pi PING] Error during ping for servo {servo_id}: {e}")
        return False


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

    # Clamp position and speed to their valid ranges (encoder resolution from backend)
    encoder_max = utils.ENCODER_RESOLUTION
    pos_val_clamped = int(max(0, min(encoder_max, position_value)))
    spd_val_clamped = int(max(0, min(encoder_max, speed_value)))

    # Goal Position (Address 0x2A), 2 bytes for position, 2 bytes for "time" (set to 0), 2 bytes for speed
    # This means we are writing 6 bytes starting at address 0x2A.
    # Parameters = [Addr=0x2A, Pos_L, Pos_H, Time_L=0, Time_H=0, Spd_L, Spd_H]
    # PacketLen = NumParams (Addr + 6 data bytes = 7) + 2 = 9. This is correct.
    # Instruction = 0x03 (WRITE)
    # Start Address = SERVO_ADDR_TARGET_POSITION (0x2A)

    packet = bytearray(13)
    packet[0] = utils.SERVO_HEADER # 0xFF
    packet[1] = utils.SERVO_HEADER # 0xFF
    packet[2] = servo_id     # Servo ID
    packet[3] = 9            # Packet Length = (Num Instruction Parameters) + 2. Here: Addr + 6 data bytes = 7 params. 7+2=9.
    packet[4] = utils.SERVO_INSTRUCTION_WRITE # 0x03
    packet[5] = utils.SERVO_ADDR_TARGET_POSITION # 0x2A (Start address for Goal Position)
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
        with _SERIAL_LOCK:
            utils.ser.write(packet)
        # print(f"[Pi] Servo {servo_id} command: {list(packet)}") # Verbose
    except Exception as e:
        print(f"[Pi] Error writing to serial for servo {servo_id}: {e}")


def write_servo_angle_limits(servo_id: int, min_limit_raw: int, max_limit_raw: int) -> bool:
    """
    Writes the minimum and maximum angle limits to a servo's EEPROM.
    This requires unlocking the EEPROM, writing the values, and re-locking it.

    Args:
        servo_id (int): The hardware ID of the target servo.
        min_limit_raw (int): The raw minimum angle limit (0-4095).
        max_limit_raw (int): The raw maximum angle limit (0-4095).

    Returns:
        bool: True if all steps were successful, False otherwise.
    """
    # Get constants from backend config
    cfg = _get_backend_config()
    ADDR_WRITE_LOCK = cfg.SERVO_ADDR_WRITE_LOCK
    ADDR_MIN_ANGLE = cfg.SERVO_ADDR_MIN_ANGLE_LIMIT
    ADDR_MAX_ANGLE = cfg.SERVO_ADDR_MAX_ANGLE_LIMIT
    
    # 1. Unlock EEPROM
    if not write_servo_register_byte(servo_id, ADDR_WRITE_LOCK, 0):
        print(f"[Pi LimitSet] FAILED to unlock EEPROM for servo {servo_id}.")
        return False
    time.sleep(0.01)

    # 2. Write Min and Max Angle Limits
    min_ok = write_servo_register_word(servo_id, ADDR_MIN_ANGLE, min_limit_raw)
    time.sleep(0.01)
    max_ok = write_servo_register_word(servo_id, ADDR_MAX_ANGLE, max_limit_raw)
    time.sleep(0.01)

    if not (min_ok and max_ok):
        print(f"[Pi LimitSet] FAILED to write angle limits for servo {servo_id}.")
        # Attempt to re-lock EEPROM even on failure for safety
        write_servo_register_byte(servo_id, ADDR_WRITE_LOCK, 1)
        return False

    # 3. Re-lock EEPROM
    if not write_servo_register_byte(servo_id, ADDR_WRITE_LOCK, 1):
        # This is not a critical failure, but should be noted.
        print(f"[Pi LimitSet] WARNING: Failed to re-lock EEPROM for servo {servo_id}.")

    print(f"[Pi LimitSet] Successfully set angle limits for servo {servo_id} to [{min_limit_raw}, {max_limit_raw}]")
    return True


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
    packet[0] = utils.SERVO_HEADER
    packet[1] = utils.SERVO_HEADER
    packet[2] = servo_id
    packet[3] = 4  # Packet Length
    packet[4] = utils.SERVO_INSTRUCTION_WRITE # 0x03 (WRITE)
    packet[5] = utils.SERVO_ADDR_TARGET_ACCELERATION # 0x29
    packet[6] = servo_register_value # Acceleration value (0-254)

    packet_data_for_sum = packet[2:7] # ID, Len, Instr, Addr, Value
    packet[7] = calculate_checksum(packet_data_for_sum)

    try:
        with _SERIAL_LOCK:
            utils.ser.write(packet)
        # print(f"[Pi] Servo {servo_id} acceleration set to: {servo_register_value} (reg value), from {acceleration_value_deg_s2} deg/s^2")
    except Exception as e:
        print(f"[Pi] Error writing acceleration for servo {servo_id}: {e}")



def read_servo_register_word(servo_id: int, register_address: int) -> int | None:
    """
    Constructs and sends a 'READ' packet to request 2 bytes (a word) from a
    specific register address of a single servo.

    Args:
        servo_id (int): The hardware ID of the target servo.
        register_address (int): The starting address of the register to read from.

    Returns:
        int | None: The 16-bit value read from the register, or None on failure.
    """
    if utils.ser is None or not utils.ser.is_open:
        # This case is logged by the caller, so no print here to avoid noise.
        return None

    # Command Packet: [0xFF, 0xFF, ID, Length=4, Instr=0x02, Addr, BytesToRead=2, Checksum]
    read_command = bytearray(8)
    read_command[0] = utils.SERVO_HEADER
    read_command[1] = utils.SERVO_HEADER
    read_command[2] = servo_id
    read_command[3] = 4  # Length
    read_command[4] = utils.SERVO_INSTRUCTION_READ
    read_command[5] = register_address
    read_command[6] = 2  # Number of bytes to read (word)
    
    read_command[7] = calculate_checksum(read_command[2:7])

    try:
        with _SERIAL_LOCK:
            utils.ser.reset_input_buffer()
            utils.ser.write(read_command)

        # Expected response (Status Packet): [0xFF, 0xFF, ID, Length=4, Error, Param1(LSB), Param2(MSB), Checksum]
        # Total response length = 8 bytes.
        response = utils.ser.read(8)

        if len(response) < 8:
            return None # Timeout or short response
            
        # Verify header and ID
        if not (response[0] == 0xFF and response[1] == 0xFF and response[2] == servo_id):
            return None
        
        # Verify checksum
        if response[7] != calculate_checksum(response[2:7]):
            return None

        # Check for hardware errors
        if response[4] != 0:
            return None

        # Extract the 16-bit value (little-endian)
        value = response[5] | (response[6] << 8)
        return value

    except Exception:
        return None


def read_servo_position(servo_id: int) -> int | None:
    """
    Constructs and sends a 'READ' packet to request the current position of a single servo.

    Args:
        servo_id (int): The hardware ID of the target servo.

    Returns:
        int | None: The servo's current raw position (0-4095), or None on failure.
        
    .. deprecated::
        Use ``backend.read_single_actuator_position(servo_id)`` instead.
    """
    # Dispatch to backend if available
    backend = _get_backend()
    if backend and hasattr(backend, 'read_single_actuator_position'):
        return backend.read_single_actuator_position(servo_id)
    
    # Fallback to direct serial communication
    if utils.ser is None or not utils.ser.is_open:
        print(f"[Pi ReadPos] Servo {servo_id}: Serial port not open.")
        return None

    # Command Packet: [0xFF, 0xFF, ID, Length=4, Instr=0x02, Addr=0x38, BytesToRead=2, Checksum]
    # Length = (Num Instruction Parameters) + 2. Here: Addr + BytesToRead = 2 params. 2+2=4.
    read_command = bytearray(8)
    read_command[0] = utils.SERVO_HEADER
    read_command[1] = utils.SERVO_HEADER
    read_command[2] = servo_id
    read_command[3] = 4  # Length
    read_command[4] = utils.SERVO_INSTRUCTION_READ
    read_command[5] = utils.SERVO_ADDR_PRESENT_POSITION
    read_command[6] = 2  # Number of bytes to read (for position)
    
    # Checksum for read command (ID, Length, Instr, Addr, BytesToRead)
    read_command[7] = calculate_checksum(read_command[2:7])

    try:
        with _SERIAL_LOCK:
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

        if response[0] != utils.SERVO_HEADER or response[1] != utils.SERVO_HEADER:
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
            names = alerts.names_for_status_bits(int(error_byte))
            print(f"[Pi ReadPos] Servo {servo_id}: Error {int(error_byte)} ({', '.join(names)})")
            alerts.push_alert(
                level="error",
                kind="SERVO_STATUS",
                message=f"Servo {int(servo_id)} reported: {', '.join(names) or 'Unknown error'}",
                servo_ids=[int(servo_id)],
                details={"status_byte": int(error_byte)},
                key=f"SERVO_STATUS:{int(servo_id)}:{int(error_byte)}",
            )
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
    read_command[0] = utils.SERVO_HEADER
    read_command[1] = utils.SERVO_HEADER
    read_command[2] = servo_id
    read_command[3] = 4  # Length
    read_command[4] = utils.SERVO_INSTRUCTION_READ
    read_command[5] = register_address
    read_command[6] = 2  # Number of bytes to read
    read_command[7] = calculate_checksum(read_command[2:7])

    try:
        with _SERIAL_LOCK:
            utils.ser.reset_input_buffer()
            utils.ser.write(read_command)
        
        # Expected response: [0xFF, 0xFF, ID, Length=4, Error, Val_L, Val_H, Checksum]
        response = utils.ser.read(8) 

        if not response or len(response) < 8:
            print(f"[Pi ReadWord] Servo {servo_id}: No/incomplete response.")
            return None

        # Basic validation
        if response[0] != utils.SERVO_HEADER or response[1] != utils.SERVO_HEADER or response[2] != servo_id:
            print(f"[Pi ReadWord] Servo {servo_id}: Invalid response header or ID mismatch.")
            return None
        
        # Checksum validation
        expected_checksum = calculate_checksum(response[2:7])
        if expected_checksum != response[7]:
            print(f"[Pi ReadWord] Servo {servo_id}: Response checksum mismatch.")
            return None
        
        # Error byte validation
        if response[4] != 0:
            names = alerts.names_for_status_bits(int(response[4]))
            print(f"[Pi ReadWord] Servo {servo_id}: Error {int(response[4])} ({', '.join(names)})")
            alerts.push_alert(
                level="error",
                kind="SERVO_STATUS",
                message=f"Servo {int(servo_id)} reported: {', '.join(names) or 'Unknown error'}",
                servo_ids=[int(servo_id)],
                details={"status_byte": int(response[4]), "register": int(register_address)},
                key=f"SERVO_STATUS:{int(servo_id)}:{int(response[4])}",
            )
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
    packet[0] = utils.SERVO_HEADER
    packet[1] = utils.SERVO_HEADER
    packet[2] = servo_id
    packet[3] = 5  # Packet Length: Instr(1) + Addr(1) + Data(2) + 2 = 5
    packet[4] = utils.SERVO_INSTRUCTION_WRITE
    packet[5] = register_address
    packet[6] = val_clamped & 0xFF      # Low byte
    packet[7] = (val_clamped >> 8) & 0xFF # High byte
    
    packet_data_for_sum = packet[2:8]
    packet[8] = calculate_checksum(packet_data_for_sum)

    try:
        with _SERIAL_LOCK:
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
    packet[0] = utils.SERVO_HEADER
    packet[1] = utils.SERVO_HEADER
    packet[2] = servo_id
    packet[3] = 4  # Packet Length (Instr + Addr + Value + Checksum = 2 + 1 + 1)
    packet[4] = utils.SERVO_INSTRUCTION_WRITE
    packet[5] = register_address
    packet[6] = val_clamped
    packet_data_for_sum = packet[2:7]
    packet[7] = calculate_checksum(packet_data_for_sum)

    try:
        with _SERIAL_LOCK:
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
                         
    .. deprecated::
        Use ``backend.sync_write(commands)`` instead.
    """
    # Dispatch to backend if available
    backend = _get_backend()
    if backend and hasattr(backend, 'sync_write'):
        backend.sync_write(servo_data_list)
        return
    
    # Fallback to direct serial communication
    if utils.ser is None or not utils.ser.is_open:
        print("[Pi SyncWrite] Serial port not initialized.")
        return

    num_servos = len(servo_data_list)
    if num_servos == 0:
        # print("[Pi SyncWrite] No servo data to send.")
        return

    packet_len_field_value = num_servos * (1 + utils.SYNC_WRITE_DATA_LEN_PER_SERVO) + 4

    # Total packet size = Header(2) + BroadcastID(1) + PacketLenField(1) + ContentDescribedByPacketLenField + Checksum(1)
    total_packet_bytes = 4 + packet_len_field_value + 1

    packet = bytearray(total_packet_bytes)
    packet[0] = utils.SERVO_HEADER
    packet[1] = utils.SERVO_HEADER
    packet[2] = utils.SERVO_BROADCAST_ID
    packet[3] = packet_len_field_value

    packet[4] = utils.SERVO_INSTRUCTION_SYNC_WRITE
    packet[5] = utils.SYNC_WRITE_START_ADDRESS
    packet[6] = utils.SYNC_WRITE_DATA_LEN_PER_SERVO

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
        with _SERIAL_LOCK:
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
    packet[0] = utils.SERVO_HEADER
    packet[1] = utils.SERVO_HEADER
    packet[2] = servo_id
    packet[3] = 2 # Packet Length = NumParams(0) + 2
    packet[4] = utils.SERVO_INSTRUCTION_CALIBRATE_MIDDLE
    
    packet_data_for_sum = packet[2:5] # ID, Len, Instr
    packet[5] = calculate_checksum(packet_data_for_sum)

    try:
        with _SERIAL_LOCK:
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
        
    .. deprecated::
        Use ``backend.factory_reset_actuator(servo_id)`` instead.
    """
    # Dispatch to backend if available
    backend = _get_backend()
    if backend and hasattr(backend, 'factory_reset_actuator'):
        return backend.factory_reset_actuator(servo_id)
    
    # Fallback to direct serial communication
    if utils.ser is None:
        print(f"[Pi] Serial port not initialized for factory reset command.")
        return False

    # Packet: [0xFF, 0xFF, ID, Length=2, Instr=0x06, Checksum]
    packet = bytearray(6)
    packet[0] = utils.SERVO_HEADER
    packet[1] = utils.SERVO_HEADER
    packet[2] = servo_id
    packet[3] = 2  # Packet Length = NumParams(0) + 2
    packet[4] = utils.SERVO_INSTRUCTION_RESET
    
    packet_data_for_sum = packet[2:5]  # ID, Len, Instr
    packet[5] = calculate_checksum(packet_data_for_sum)

    try:
        with _SERIAL_LOCK:
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
        
    .. deprecated::
        Use ``backend.restart_actuator(servo_id)`` instead.
    """
    # Dispatch to backend if available
    backend = _get_backend()
    if backend and hasattr(backend, 'restart_actuator'):
        return backend.restart_actuator(servo_id)
    
    # Fallback to direct serial communication
    if utils.ser is None:
        print(f"[Pi] Serial port not initialized for restart command.")
        return False

    # Packet: [0xFF, 0xFF, ID, Length=2, Instr=0x08, Checksum]
    packet = bytearray(6)
    packet[0] = utils.SERVO_HEADER
    packet[1] = utils.SERVO_HEADER
    packet[2] = servo_id
    packet[3] = 2  # Packet Length = NumParams(0) + 2
    packet[4] = utils.SERVO_INSTRUCTION_RESTART

    packet_data_for_sum = packet[2:5]  # ID, Len, Instr
    packet[5] = calculate_checksum(packet_data_for_sum)

    try:
        with _SERIAL_LOCK:
            utils.ser.write(packet)
        # No response for this command either.
        return True
    except Exception as e:
        print(f"[Pi] Error writing restart command to servo {servo_id}: {e}")
        return False


def sync_read_positions(
    servo_ids: list[int],
    timeout_s: float | None = None,
    poll_delay_s: float = 0.0,
    diagnostics: bool = True,
) -> dict[int, int] | None:
    """
    Reads the present position of multiple servos using a single 'SYNC READ' command.
    This is significantly faster than reading one by one, enabling high-frequency feedback.
    This implementation is designed to be robust against timeouts from individual servos.

    Args:
        servo_ids (list): A list of servo IDs to read from.
        timeout_s (float | None): Optional per-call serial read timeout override in seconds. If None,
                                  the existing serial timeout is preserved.
        poll_delay_s (float): Optional delay inserted after issuing the SYNC READ before attempting
                              to read the responses. This can sometimes improve reliability on slow
                              links. Defaults to 0 (no delay).
        diagnostics (bool): Whether to collect timing data for diagnostics.

    Returns:
        dict[int, int]: A dictionary mapping servo_id to its raw position. This may be a partial
                        result if some servos did not respond.
                        
    .. deprecated::
        Use ``backend.sync_read_positions(servo_ids)`` instead.
    """
    # Dispatch to backend if available
    # Note: The ActuatorBackend.sync_read_positions() doesn't take servo_ids as an argument.
    # The backend reads from all configured arm servos internally.
    backend = _get_backend()
    if backend and hasattr(backend, 'sync_read_positions'):
        return backend.sync_read_positions(timeout_s=timeout_s)
    
    # Fallback to direct serial communication
    if not _serial_protocol_supported():
        return {}
    if utils.ser is None or not utils.ser.is_open:
        print("[Pi SyncRead] Serial port not initialized.")
        return {}

    num_servos = len(servo_ids)
    if num_servos == 0:
        return {}

    # --- Construct the Sync Read Packet ---
    # Packet: [0xFF, 0xFF, Broadcast_ID, Len, Instr, StartAddr, DataLen, ID1, ID2, ..., Checksum]
    packet_len_field_value = num_servos + 4
    
    packet = bytearray(7 + num_servos + 1) # Header(2) + BcastID(1) + Len(1) + Instr(1) + Addr(1) + DataLen(1) + IDs(N) + Checksum(1)
    packet[0] = utils.SERVO_HEADER
    packet[1] = utils.SERVO_HEADER
    packet[2] = utils.SERVO_BROADCAST_ID
    packet[3] = packet_len_field_value
    packet[4] = utils.SERVO_INSTRUCTION_SYNC_READ
    packet[5] = utils.SERVO_ADDR_PRESENT_POSITION # Start Address to read from (0x38)
    packet[6] = 2 # Length of data to read per servo (Pos_L, Pos_H)

    # Add all the servo IDs to the packet
    for i, servo_id in enumerate(servo_ids):
        packet[7 + i] = servo_id

    # Calculate checksum on the instruction parameters
    checksum_data = packet[2:-1] # From Broadcast_ID to last servo ID
    packet[-1] = calculate_checksum(checksum_data)


    # --- Send Command and Process Responses ---
    try:
        # Optionally override the serial timeout for this call
        original_timeout = None
        if timeout_s is not None:
            original_timeout = utils.ser.timeout
            utils.ser.timeout = timeout_s

        # --- WRITE ---
        write_start = time.perf_counter()
        with _SERIAL_LOCK:
            utils.ser.reset_input_buffer()
            utils.ser.write(packet)
        write_dur = time.perf_counter() - write_start

        # Allow a brief delay for servos to respond if requested
        if poll_delay_s > 0.0:
            time.sleep(poll_delay_s)

        positions = {}
        expected_ids = set(servo_ids)
        
        # --- READ ---
        bytes_to_read = num_servos * 8  # Each servo sends an 8-byte status packet
        read_start = time.perf_counter()
        with _SERIAL_LOCK:
            response_data = utils.ser.read(bytes_to_read)
        read_dur = time.perf_counter() - read_start

        if len(response_data) < bytes_to_read:
            print(f"[Pi SyncRead] WARNING: Timed out. Expected {bytes_to_read} bytes, but got {len(response_data)}. Parsing partial data.")

        # --- PARSE ---
        parse_start = time.perf_counter()
        for i in range(0, len(response_data) - 7): # Iterate with a sliding window
            # Look for the header
            if response_data[i] == utils.SERVO_HEADER and response_data[i+1] == utils.SERVO_HEADER:
                # Potential packet found, extract it
                packet_candidate = response_data[i : i+8]
                
                response_id = packet_candidate[2]
                if response_id not in expected_ids:
                    # This could be a stray packet, just log it and continue searching
                    # print(f"[Pi SyncRead] Found packet for unexpected ID {response_id}. Ignoring.")
                    continue

                # Error byte validation
                if packet_candidate[4] != 0:
                    names = alerts.names_for_status_bits(int(packet_candidate[4]))
                    print(f"[Pi SyncRead] Servo {response_id} reported error: {int(packet_candidate[4])} ({', '.join(names)})")
                    alerts.push_alert(
                        level="error",
                        kind="SERVO_STATUS",
                        message=f"Servo {int(response_id)} reported: {', '.join(names) or 'Unknown error'}",
                        servo_ids=[int(response_id)],
                        details={"status_byte": int(packet_candidate[4])},
                        key=f"SERVO_STATUS:{int(response_id)}:{int(packet_candidate[4])}",
                    )
                    continue # Skip this packet

                # Checksum validation
                expected_checksum = calculate_checksum(packet_candidate[2:7])
                if expected_checksum != packet_candidate[7]:
                    print(f"[Pi SyncRead] Checksum mismatch for servo {response_id}. Ignoring packet.")
                    continue # Skip this packet
                
                # If we're here, the packet is valid
                position = int.from_bytes(packet_candidate[5:7], byteorder='little', signed=True)
                positions[response_id] = position
                # Remove the ID from the set of servos we are still waiting for
                expected_ids.discard(response_id)


        # After parsing all we could, report which servos didn't respond
        if len(expected_ids) > 0:
            missing = list(expected_ids)
            print(f"[Pi SyncRead] Did not receive valid responses for IDs: {missing}")
            alerts.push_alert(
                level="warning",
                kind="SYNCREAD_TIMEOUT",
                message=f"No feedback from servos {', '.join(str(x) for x in missing)} (SyncRead). Check power/wiring/baud.",
                servo_ids=[int(x) for x in missing if isinstance(x, int)],
                details={"operation": "sync_read_positions"},
                key=f"SYNCREAD_TIMEOUT:{','.join(str(x) for x in missing)}",
            )

        parse_dur = time.perf_counter() - parse_start

        if diagnostics:
            # Telemetry: store timings for diagnostics (in seconds)
            _sync_profiles.append((write_dur, read_dur, parse_dur))

        return positions

    except Exception as e:
        print(f"[Pi SyncRead] Error during Sync Read: {e}")
        return {}
    finally:
        # Restore the previous serial timeout if we modified it
        if timeout_s is not None and original_timeout is not None:
            utils.ser.timeout = original_timeout

def fast_sync_read_positions(
    servo_ids: list[int],
    timeout_s: float | None = None,
    poll_delay_s: float = 0.0,
    diagnostics: bool = True,
) -> dict[int, int] | None:
    """
    Reads the present position of multiple servos using a single 'SYNC READ' command.
    This is significantly faster than reading one by one, enabling high-frequency feedback.
    This implementation is designed to be robust against timeouts from individual servos.

    Args:
        servo_ids (list): A list of servo IDs to read from.
        timeout_s (float | None): Optional per-call serial read timeout override in seconds. If None,
                                  the existing serial timeout is preserved.
        poll_delay_s (float): Optional delay inserted after issuing the SYNC READ before attempting
                              to read the responses. This can sometimes improve reliability on slow
                              links. Defaults to 0 (no delay).
        diagnostics (bool): Whether to collect timing data for diagnostics.

    Returns:
        dict[int, int]: A dictionary mapping servo_id to its raw position. This may be a partial
                        result if some servos did not respond.
    """
    if not _serial_protocol_supported():
        return {}
    if utils.ser is None or not utils.ser.is_open:
        print("[Pi SyncRead] Serial port not initialized.")
        return {}

    num_servos = len(servo_ids)
    if num_servos == 0:
        return {}

    # --- Construct the Sync Read Packet ---
    # Packet: [0xFF, 0xFF, Broadcast_ID, Len, Instr, StartAddr, DataLen, ID1, ID2, ..., Checksum]
    packet_len_field_value = num_servos + 4
    
    packet = bytearray(7 + num_servos + 1) # Header(2) + BcastID(1) + Len(1) + Instr(1) + Addr(1) + DataLen(1) + IDs(N) + Checksum(1)
    packet[0] = utils.SERVO_HEADER
    packet[1] = utils.SERVO_HEADER
    packet[2] = utils.SERVO_BROADCAST_ID
    packet[3] = packet_len_field_value
    packet[4] = utils.SERVO_INSTRUCTION_SYNC_READ
    packet[5] = utils.SERVO_ADDR_PRESENT_POSITION # Start Address to read from (0x38)
    packet[6] = 2 # Length of data to read per servo (Pos_L, Pos_H)

    # Add all the servo IDs to the packet
    for i, servo_id in enumerate(servo_ids):
        packet[7 + i] = servo_id

    # Calculate checksum on the instruction parameters
    checksum_data = packet[2:-1] # From Broadcast_ID to last servo ID
    packet[-1] = calculate_checksum(checksum_data)


    # --- Send Command and Process Responses ---
    try:
        # Optionally override the serial timeout for this call
        original_timeout = None
        if timeout_s is not None:
            original_timeout = utils.ser.timeout
            utils.ser.timeout = timeout_s

        # --- WRITE ---
        write_start = time.perf_counter()
        with _SERIAL_LOCK:
            utils.ser.reset_input_buffer()
            utils.ser.write(packet)
        write_dur = time.perf_counter() - write_start

        # Allow a brief delay for servos to respond if requested
        if poll_delay_s > 0.0:
            time.sleep(poll_delay_s)

        positions = {}
        expected_ids = set(servo_ids)
        
        # --- READ ---
        bytes_to_read = num_servos * 8  # Each servo sends an 8-byte status packet
        read_start = time.perf_counter()
        with _SERIAL_LOCK:
            response_data = utils.ser.read(bytes_to_read)
        read_dur = time.perf_counter() - read_start

        if len(response_data) < bytes_to_read:
            print(f"[Pi SyncRead] WARNING: Timed out. Expected {bytes_to_read} bytes, but got {len(response_data)}. Parsing partial data.")

        # --- PARSE ---
        parse_start = time.perf_counter()
        for i in range(0, len(response_data) - 7): # Iterate with a sliding window
            # Look for the header
            if response_data[i] == utils.SERVO_HEADER and response_data[i+1] == utils.SERVO_HEADER:
                # Potential packet found, extract it
                packet_candidate = response_data[i : i+8]
                
                response_id = packet_candidate[2]
                if response_id not in expected_ids:
                    # This could be a stray packet, just log it and continue searching
                    # print(f"[Pi SyncRead] Found packet for unexpected ID {response_id}. Ignoring.")
                    continue

                # Error byte validation
                if packet_candidate[4] != 0:
                    names = alerts.names_for_status_bits(int(packet_candidate[4]))
                    print(f"[Pi SyncRead] Servo {response_id} reported error: {int(packet_candidate[4])} ({', '.join(names)})")
                    alerts.push_alert(
                        level="error",
                        kind="SERVO_STATUS",
                        message=f"Servo {int(response_id)} reported: {', '.join(names) or 'Unknown error'}",
                        servo_ids=[int(response_id)],
                        details={"status_byte": int(packet_candidate[4])},
                        key=f"SERVO_STATUS:{int(response_id)}:{int(packet_candidate[4])}",
                    )
                    continue # Skip this packet

                # Checksum validation
                expected_checksum = calculate_checksum(packet_candidate[2:7])
                if expected_checksum != packet_candidate[7]:
                    print(f"[Pi SyncRead] Checksum mismatch for servo {response_id}. Ignoring packet.")
                    continue # Skip this packet
                
                # If we're here, the packet is valid
                position = int.from_bytes(packet_candidate[5:7], byteorder='little', signed=True)
                positions[response_id] = position
                # Remove the ID from the set of servos we are still waiting for
                expected_ids.discard(response_id)


        # After parsing all we could, report which servos didn't respond
        if len(expected_ids) > 0:
            missing = list(expected_ids)
            print(f"[Pi SyncRead] Did not receive valid responses for IDs: {missing}")
            alerts.push_alert(
                level="warning",
                kind="SYNCREAD_TIMEOUT",
                message=f"No feedback from servos {', '.join(str(x) for x in missing)} (SyncRead). Check power/wiring/baud.",
                servo_ids=[int(x) for x in missing if isinstance(x, int)],
                details={"operation": "fast_sync_read_positions"},
                key=f"SYNCREAD_TIMEOUT:{','.join(str(x) for x in missing)}",
            )

        parse_dur = time.perf_counter() - parse_start

        if diagnostics:
            # Telemetry: store timings for diagnostics (in seconds)
            _sync_profiles.append((write_dur, read_dur, parse_dur))

        return positions

    except Exception as e:
        print(f"[Pi SyncRead] Error during Sync Read: {e}")
        return {}
    finally:
        # Restore the previous serial timeout if we modified it
        if timeout_s is not None and original_timeout is not None:
            utils.ser.timeout = original_timeout


def sync_read_block(
    servo_ids: list[int],
    start_address: int,
    data_len: int,
    timeout_s: float | None = None,
    poll_delay_s: float = 0.0,
    diagnostics: bool = False,
) -> dict[int, bytes]:
    """
    Perform a generic SYNC READ for a contiguous block of registers starting at `start_address`
    with length `data_len` for each servo in `servo_ids`.

    Returns a mapping of servo_id -> raw bytes (length == data_len) for all servos that
    responded with a valid status packet. Missing IDs indicate no valid response was parsed.
    
    .. deprecated::
        Use ``backend.sync_read_block(servo_ids, start_address, data_len)`` instead.
    """
    # Dispatch to backend if available
    backend = _get_backend()
    if backend and hasattr(backend, 'sync_read_block'):
        return backend.sync_read_block(servo_ids, start_address=start_address, data_len=data_len)
    
    # Fallback to direct serial communication
    if not _serial_protocol_supported():
        return {}
    if utils.ser is None or not utils.ser.is_open:
        print("[Pi SyncReadBlk] Serial port not initialized.")
        return {}

    num_servos = len(servo_ids)
    if num_servos == 0 or data_len <= 0:
        return {}

    packet_len_field_value = num_servos + 4
    packet = bytearray(7 + num_servos + 1)
    packet[0] = utils.SERVO_HEADER
    packet[1] = utils.SERVO_HEADER
    packet[2] = utils.SERVO_BROADCAST_ID
    packet[3] = packet_len_field_value
    packet[4] = utils.SERVO_INSTRUCTION_SYNC_READ
    packet[5] = start_address
    packet[6] = data_len
    for i, servo_id in enumerate(servo_ids):
        packet[7 + i] = servo_id
    packet[-1] = calculate_checksum(packet[2:-1])

    try:
        original_timeout = None
        if timeout_s is not None:
            original_timeout = utils.ser.timeout
            utils.ser.timeout = timeout_s

        write_start = time.perf_counter()
        with _SERIAL_LOCK:
            utils.ser.reset_input_buffer()
            utils.ser.write(packet)
        write_dur = time.perf_counter() - write_start

        if poll_delay_s > 0.0:
            time.sleep(poll_delay_s)

        # Each status packet size = 2 (hdr) + 1 (id) + 1 (len) + 1 (err) + data_len + 1 (ck)
        per_packet = 6 + data_len
        bytes_to_read = num_servos * per_packet
        read_start = time.perf_counter()
        with _SERIAL_LOCK:
            response_data = utils.ser.read(bytes_to_read)
        read_dur = time.perf_counter() - read_start

        if len(response_data) < per_packet:
            # Not enough to parse even one packet
            return {}

        results: dict[int, bytes] = {}
        expected_ids = set(servo_ids)

        parse_start = time.perf_counter()
        # Sliding window parse for variable-length status packets
        i = 0
        end = len(response_data) - per_packet + 1
        while i < end:
            if response_data[i] == utils.SERVO_HEADER and response_data[i + 1] == utils.SERVO_HEADER:
                pkt = response_data[i : i + per_packet]
                sid = pkt[2]
                if sid in expected_ids:
                    # Validate error == 0 and checksum
                    if pkt[4] == 0:
                        if calculate_checksum(pkt[2: (2 + 1 + 1 + 1 + data_len)]) == pkt[-1]:
                            results[sid] = bytes(pkt[5 : 5 + data_len])
                            expected_ids.discard(sid)
                            i += per_packet
                            continue
                    else:
                        names = alerts.names_for_status_bits(int(pkt[4]))
                        print(f"[Pi SyncReadBlk] Servo {sid} reported error: {int(pkt[4])} ({', '.join(names)})")
                        alerts.push_alert(
                            level="error",
                            kind="SERVO_STATUS",
                            message=f"Servo {int(sid)} reported: {', '.join(names) or 'Unknown error'}",
                            servo_ids=[int(sid)],
                            details={"status_byte": int(pkt[4]), "start_address": int(start_address), "len": int(data_len)},
                            key=f"SERVO_STATUS:{int(sid)}:{int(pkt[4])}",
                        )
                # If not valid, advance by 1 to resync
                i += 1
            else:
                i += 1
        parse_dur = time.perf_counter() - parse_start

        if diagnostics:
            _sync_profiles.append((write_dur, read_dur, parse_dur))

        return results
    except Exception as e:
        print(f"[Pi SyncReadBlk] Error: {e}")
        return {}
    finally:
        if timeout_s is not None and original_timeout is not None:
            utils.ser.timeout = original_timeout
