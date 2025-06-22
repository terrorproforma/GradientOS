# Contains low-level functions for communicating with Feetech servos,
# such as packet creation and checksum calculation. 
from . import utils
import time


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


def sync_read_positions(
    servo_ids: list[int],
    timeout_s: float | None = None,
    poll_delay_s: float = 0.0,
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

    Returns:
        dict[int, int]: A dictionary mapping servo_id to its raw position. This may be a partial
                        result if some servos did not respond.
    """
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
        # Optionally override the serial timeout for this call
        original_timeout = None
        if timeout_s is not None:
            original_timeout = utils.ser.timeout
            utils.ser.timeout = timeout_s

        utils.ser.reset_input_buffer()
        utils.ser.write(packet)

        # Allow a brief delay for servos to respond if requested
        if poll_delay_s > 0.0:
            time.sleep(poll_delay_s)

        positions = {}
        expected_ids = set(servo_ids)
        
        # Each servo sends an 8-byte status packet
        bytes_to_read = num_servos * 8
        response_data = utils.ser.read(bytes_to_read)

        if len(response_data) < bytes_to_read:
            print(f"[Pi SyncRead] WARNING: Timed out. Expected {bytes_to_read} bytes, but got {len(response_data)}. Parsing partial data.")

        # Parse the buffer for valid packets
        for i in range(0, len(response_data) - 7): # Iterate with a sliding window
            # Look for the header
            if response_data[i] == SERVO_HEADER and response_data[i+1] == SERVO_HEADER:
                # Potential packet found, extract it
                packet_candidate = response_data[i : i+8]
                
                response_id = packet_candidate[2]
                if response_id not in expected_ids:
                    # This could be a stray packet, just log it and continue searching
                    # print(f"[Pi SyncRead] Found packet for unexpected ID {response_id}. Ignoring.")
                    continue

                # Error byte validation
                if packet_candidate[4] != 0:
                    print(f"[Pi SyncRead] Servo {response_id} reported error: {packet_candidate[4]}")
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
            print(f"[Pi SyncRead] Did not receive valid responses for IDs: {list(expected_ids)}")

        return positions

    except Exception as e:
        print(f"[Pi SyncRead] Error during Sync Read: {e}")
        return {}
    finally:
        # Restore the previous serial timeout if we modified it
        if timeout_s is not None and original_timeout is not None:
            utils.ser.timeout = original_timeout

