# Contains high-level functions for controlling the servos, acting as a
# hardware abstraction layer. Imports from servo_protocol.py. 
import time
import serial
import numpy as np

from . import utils
from . import servo_protocol

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

    # The hardware offset is no longer needed as calibration is handled on the servo itself.

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
        0: [0],     # Logical J1 -> Physical Servo ID 10
        1: [1, 2],  # Logical J2 -> Physical Servo IDs 20, 21
        2: [3, 4],  # Logical J3 -> Physical Servo IDs 30, 31
        3: [5],     # Logical J4 -> Physical Servo ID 40
        4: [6],     # Logical J5 -> Physical Servo ID 50
        5: [7],     # Logical J6 -> Physical Servo ID 60
    }

    for logical_joint_index in range(utils.NUM_LOGICAL_JOINTS):
        # Master offset is still useful for high-level adjustments
        angle_to_be_processed = logical_joint_angles_rad[logical_joint_index] + utils.LOGICAL_JOINT_MASTER_OFFSETS_RAD[logical_joint_index]
        
        target_physical_angle_rad = angle_to_be_processed

        for physical_servo_config_index in logical_to_physical_map[logical_joint_index]:
            current_physical_servo_id = utils.SERVO_IDS[physical_servo_config_index]

            # No extra sign inversion here—orientation differences are fully
            # captured by the direct/inverted mapping in utils._is_servo_direct_mapping.
            final_target_physical_angle_rad = target_physical_angle_rad

            # --- Convert Target Angle (Radians) to Raw Servo Value (0-4095) ---
            
            # The servo's individual hardware offset is no longer needed.
            # Calibration is now handled directly on the servo via the 0x0B command,
            # which sets the current position to be the center (2048).

            # 2. Normalize the angle to a 0-1 range based on its mapping
            #    Most servos are direct (angle increases with raw value), but some are inverted.
            min_map_rad, max_map_rad = utils.EFFECTIVE_MAPPING_RANGES[physical_servo_config_index]

            # Clamp the angle to the effective mapping range before normalization
            angle_for_norm = max(min_map_rad, min(max_map_rad, final_target_physical_angle_rad))
            
            normalized_value = (angle_for_norm - min_map_rad) / (max_map_rad - min_map_rad)

            # Apply mapping direction
            if utils._is_servo_direct_mapping(physical_servo_config_index):
                raw_servo_value = normalized_value * 4095.0
            else: 
                raw_servo_value = (1.0 - normalized_value) * 4095.0
            
            # The calculation is now simpler as we don't add a software offset.
            final_servo_pos_value = int(round(raw_servo_value))
            final_servo_pos_value = max(0, min(4095, final_servo_pos_value))

            # Add command data for this servo to the list for Sync Write
            commands_for_sync_write.append((
                current_physical_servo_id, 
                final_servo_pos_value, 
                clamped_speed_value_for_cycle, 
                accel_reg_val_for_cycle
            ))
            
            # Disabling this high-frequency print to improve loop performance
            # print(f"[Pi PrepSync] Servo {current_physical_servo_id} (Log.J{logical_joint_index+1}): TargetPos={final_servo_pos_value} Spd={clamped_speed_value_for_cycle} AccelReg={accel_reg_val_for_cycle} (from {acceleration_value_deg_s2:.1f} deg/s^2)")

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
    It assumes the servo has been correctly calibrated so that its center point
    (e.g., 2048) corresponds to the zero-radian angle for the control software.

    Args:
        servo_value (int): The raw position value from the servo.
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
    
    # The hardware zero offset is no longer needed from the software side.
    # Calibration is now handled directly on the servo hardware.

    current_angles_rad = list(utils.current_logical_joint_angles_rad)  # Start with last known good state

    # Mapping: which physical index (in utils.SERVO_IDS) is the authoritative
    # feedback source for each logical joint.
    logical_to_primary_physical_map = {
        0: 0,  # J1 -> ID 10
        1: 1,  # J2 -> ID 20
        2: 3,  # J3 -> ID 30
        3: 5,  # J4 -> ID 40
        4: 6,  # J5 -> ID 50
        5: 7,  # J6 -> ID 60
    }

    for logical_idx, physical_idx in logical_to_primary_physical_map.items():
        servo_id = utils.SERVO_IDS[physical_idx]
        raw_pos = None if raw_positions_dict is None else raw_positions_dict.get(servo_id)

        if raw_pos is not None:
            # The raw position is now used directly, as software offsets are removed.
            corrected_raw_pos = raw_pos
            
            physical_angle_rad = servo_value_to_radians(corrected_raw_pos, physical_idx)

            logical_angle_rad = physical_angle_rad

            current_angles_rad[logical_idx] = logical_angle_rad
            if verbose:
                print(
                    f"[Pi State] Logical J{logical_idx+1} (Servo {servo_id}): Raw={raw_pos}, Angle={logical_angle_rad:.3f} rad"
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
    Uses the servo's built-in 0x0B "Calibrate Middle" instruction to make the
    *current* position the new centre (raw value ≈ 2048).  No software offsets
    are stored or needed after this call.

    Args
    ----
    servo_id : int
        Hardware ID of the servo to calibrate.
    """
    THEORETICAL_CENTER = 2048

    print(f"--- Calibrating Servo {servo_id} (Calibrate-Middle 0x0B) ---")

    # 1. Send the parameter-less 0x0B command.
    if not servo_protocol.calibrate_servo_middle_position(servo_id):
        print(f"[Pi] SET_ZERO FAILED: could not send Calibrate-Middle to {servo_id}.")
        return
    print(f"[Pi] Servo {servo_id}: 0x0B command sent.")

    # 2. Give EEPROM time to write, then re-enable torque by commanding 2048.
    time.sleep(0.2)                      # allow EEPROM write to finish
    servo_protocol.send_servo_command(servo_id, THEORETICAL_CENTER, 100)
    time.sleep(0.1)                      # small pause after torque re-engage

    # 3. Verify: the present-position read-back should now be ≈ 2048.
    verified_pos = None
    for attempt in range(3):
        time.sleep(0.1)
        verified_pos = servo_protocol.read_servo_position(servo_id)
        if verified_pos is not None:
            break
        print(f"[Pi] Servo {servo_id}: read-back attempt {attempt+1} failed.")

    if verified_pos is not None and abs(verified_pos - THEORETICAL_CENTER) < 50:
        print(f"[Pi] SET_ZERO SUCCESS: Servo {servo_id} now reports {verified_pos}.")
    else:
        print(f"[Pi] SET_ZERO WARNING: expected ≈2048, read {verified_pos}.")

    print(f"--- Calibration complete for Servo {servo_id} ---")


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
