# Contains high-level functions for controlling the servos, acting as a
# hardware abstraction layer. Imports from servo_protocol.py. 
import time
import serial
import numpy as np

from . import utils
from . import servo_protocol

def initialize_servos():
    """
    Initializes the serial connection to the servos, checks for their presence,
    and sets their default PID gains. This function must be called once at application startup.
    """
    print("[Pi] Initializing servos...")
    try:
        utils.ser = serial.Serial(utils.SERIAL_PORT, utils.BAUD_RATE, timeout=0.1)
        print(f"[Pi] Serial port {utils.SERIAL_PORT} opened successfully at {utils.BAUD_RATE} baud.")
        
        # --- Check for presence of each servo, including the gripper ---
        print("[Pi] Pinging all configured servos...")
        present_servo_ids = []
        for s_id in utils.SERVO_IDS:
            if servo_protocol.ping(s_id):
                print(f"[Pi]   - Servo {s_id}: PRESENT")
                present_servo_ids.append(s_id)
            else:
                print(f"[Pi]   - Servo {s_id}: ABSENT")
        
        # Check specifically for the gripper and set the global flag
        if utils.SERVO_ID_GRIPPER in present_servo_ids:
            utils.gripper_present = True
            print(f"[Pi] Gripper (ID {utils.SERVO_ID_GRIPPER}) is present.")
        else:
            utils.gripper_present = False
            print(f"[Pi] Gripper (ID {utils.SERVO_ID_GRIPPER}) is ABSENT.")
        
        # On startup, immediately read and display the hardware zero offsets for verification.
        print("[Pi] Reading stored hardware zero offsets from present servos...")
        # Only read from servos that are actually connected
        offsets = get_servo_hardware_zero_offsets(servo_ids_to_check=present_servo_ids)
        for i, offset in enumerate(offsets):
            # The index i corresponds to the index in present_servo_ids
            print(f"[Pi]   - Servo {present_servo_ids[i]}: Offset = {offset}")

        # Set default PID gains for all PRESENT servos upon initialization
        print("[Pi] Setting default PID gains for present servos...")
        all_pid_set_successfully = True
        # Only configure servos that responded to ping
        for s_id in present_servo_ids:
            if not set_servo_pid_gains(s_id, utils.DEFAULT_KP, utils.DEFAULT_KI, utils.DEFAULT_KD):
                all_pid_set_successfully = False
            time.sleep(0.05) # Give a bit more time after each servo's PID set
        
        if all_pid_set_successfully:
            print("[Pi] Default PID gains set for all present servos.")
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


def set_single_servo_position_rads(servo_id: int, position_rad: float, speed: int, accel: int):
    """
    Commands a single servo to a specified position in radians using the
    efficient SYNC_WRITE protocol, consistent with how the main arm is controlled.
    """
    if utils.ser is None:
        print("[Pi] Serial port not initialized, cannot set single servo position.")
        return

    try:
        config_index = utils.SERVO_IDS.index(servo_id)
    except ValueError:
        print(f"[Pi] ERROR: Servo ID {servo_id} not found in configuration.")
        return

    # Convert the high-level acceleration value to the 1-byte register value.
    accel_reg_val = 0
    if accel > 0:
        # Note: The 'accel' param here is the register value (0-254)
        accel_reg_val = int(round(accel / utils.ACCELERATION_SCALE_FACTOR))
        accel_reg_val = max(1, min(254, accel_reg_val))

    # Convert the desired angle into a raw servo value.
    raw_pos_value = angle_to_raw(position_rad, config_index)

    # Clamp the speed value.
    clamped_speed = int(max(0, min(4095, speed)))

    # Build the command tuple in the format the sync write function expects.
    command_tuple = (servo_id, raw_pos_value, clamped_speed, accel_reg_val)

    # Call the sync write function with a list containing just our single command.
    servo_protocol.sync_write_goal_pos_speed_accel([command_tuple])
    
    print(f"[Pi] Commanded single servo {servo_id} to {position_rad:.2f} rad ({raw_pos_value}) "
          f"with Speed={clamped_speed}, AccelReg={accel_reg_val}")


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

            # --- IMPORTANT: Only command servos that are present ---
            if current_physical_servo_id not in servo_protocol.get_present_servo_ids():
                continue # Skip this servo if it wasn't detected at startup

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


def angle_to_raw(angle_rad: float, physical_servo_config_index: int) -> int:
    """
    Converts a physical angle in radians to a raw servo value (0-4095).
    This is the core conversion logic used by both single and sync servo writes.
    """
    min_map_rad, max_map_rad = utils.EFFECTIVE_MAPPING_RANGES[physical_servo_config_index]

    # Clamp the angle to the effective mapping range before normalization
    angle_for_norm = max(min_map_rad, min(max_map_rad, angle_rad))
    
    normalized_value = (angle_for_norm - min_map_rad) / (max_map_rad - min_map_rad)

    # Apply mapping direction
    if utils._is_servo_direct_mapping(physical_servo_config_index):
        raw_servo_value = normalized_value * 4095.0
    else: 
        raw_servo_value = (1.0 - normalized_value) * 4095.0
    
    final_servo_pos_value = int(round(raw_servo_value))
    return max(0, min(4095, final_servo_pos_value))


def raw_to_angle_rad(raw_value: int, physical_servo_config_index: int) -> float:
    """
    Converts a raw servo value (0-4095) to a physical angle in radians.
    This is the inverse of angle_to_raw.
    """
    if raw_value is None:
        return 0.0

    servo_value = max(0, min(4095, raw_value))
    min_map_rad, max_map_rad = utils.EFFECTIVE_MAPPING_RANGES[physical_servo_config_index]
    
    is_direct = utils._is_servo_direct_mapping(physical_servo_config_index)
    
    if is_direct:
        normalized = servo_value / 4095.0
    else:
        normalized = 1.0 - (servo_value / 4095.0)
        
    angle = normalized * (max_map_rad - min_map_rad) + min_map_rad
    return angle


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
    Sets the angle limits (min and max) for each physical servo based on the
    values defined in `utils.URDF_JOINT_LIMITS`. This ensures the servos
    respect the motion boundaries defined for the kinematic model.
    """
    print("[Pi] Setting servo angle limits from URDF configuration...")
    if utils.ser is None or not utils.ser.is_open:
        print("[Pi] Error: Serial port not available for setting angle limits.")
        return

    all_limits_set = True
    # Iterate through all configured servos.
    for config_index, servo_id in enumerate(utils.SERVO_IDS):
        # --- IMPORTANT: Only configure servos that are present ---
        if servo_id not in servo_protocol.get_present_servo_ids():
            print(f"[Pi] Skipping angle limits for absent servo {servo_id}")
            continue

        min_limit_rad, max_limit_rad = utils.URDF_JOINT_LIMITS[config_index]

        # Convert the radian limits to raw servo values (0-4095)
        min_raw_limit = angle_to_raw(min_limit_rad, config_index)
        max_raw_limit = angle_to_raw(max_limit_rad, config_index)

        # The registers expect the smaller value first, but our mapping might invert this.
        # e.g., for an inverted servo, a positive angle (max_limit_rad) corresponds to a
        # smaller raw value.
        final_min_raw = min(min_raw_limit, max_raw_limit)
        final_max_raw = max(min_raw_limit, max_raw_limit)

        print(f"[Pi] -> Servo {servo_id}: Rad Limits [{min_limit_rad:.2f}, {max_limit_rad:.2f}] "
              f"-> Raw Limits [{final_min_raw}, {final_max_raw}]")

        # Write the limits to the servo's EEPROM registers
        if not servo_protocol.write_servo_angle_limits(servo_id, final_min_raw, final_max_raw):
            all_limits_set = False
            print(f"[Pi] Error: Failed to set angle limits for servo {servo_id}")
        time.sleep(0.02) # Small delay after writing to a servo's EEPROM

    if all_limits_set:
        print("[Pi] All servo angle limits set successfully.")
    else:
        print("[Pi] WARNING: Failed to set angle limits for one or more servos.")


def get_current_arm_state_rad(verbose: bool = True) -> list[float]:
    """
    Reads the current position of each of the 6 logical joints by querying the
    physical servos and averaging/mapping them as required.

    This is a "read-only" operation that does not change any state but provides
    the current snapshot of the arm's configuration.

    Args:
        verbose (bool): If True, prints detailed debug information.

    Returns:
        list[float]: A list of 6 joint angles in radians.
    """
    if verbose:
        print("[Pi] Getting current arm state...")

    # Use a single SYNC READ for efficiency, targeting only the arm servos (not gripper)
    arm_servo_ids = [sid for sid in utils.SERVO_IDS if sid != utils.SERVO_ID_GRIPPER]
    
    # --- IMPORTANT: Only read from servos that are present ---
    present_arm_servo_ids = [sid for sid in arm_servo_ids if sid in servo_protocol.get_present_servo_ids()]
    
    raw_positions = servo_protocol.sync_read_positions(present_arm_servo_ids)

    if raw_positions is None:
        print("[Pi] Sync Read failed. Falling back to individual reads.")
        raw_positions = {}
        for s_id in present_arm_servo_ids:
            pos = servo_protocol.read_servo_position(s_id)
            if pos is not None:
                raw_positions[s_id] = pos
            time.sleep(0.01)

    # Convert raw servo values to logical joint angles in radians
    current_logical_angles_rad = [0.0] * utils.NUM_LOGICAL_JOINTS
    
    # Define the mapping from logical joints to their corresponding physical servo IDs
    logical_to_physical_map = {
        0: [10],
        1: [20, 21],
        2: [30, 31],
        3: [40],
        4: [50],
        5: [60],
    }

    for logical_joint_index, physical_ids in logical_to_physical_map.items():
        angles_for_this_joint = []
        for servo_id in physical_ids:
            if servo_id in raw_positions:
                raw_pos = raw_positions[servo_id]
                try:
                    config_index = utils.SERVO_IDS.index(servo_id)
                    angle_rad = raw_to_angle_rad(raw_pos, config_index)
                    angles_for_this_joint.append(angle_rad)
                except ValueError:
                    if verbose:
                        print(f"[Pi] Warning: Servo ID {servo_id} from logical map not found in main config.")
            elif verbose:
                # This case handles when a servo in the logical map wasn't in the sync read result
                print(f"[Pi] Warning: No position data for servo {servo_id} (logical joint {logical_joint_index + 1})")

        if angles_for_this_joint:
            # Average the angles for joints with multiple servos
            logical_angle = np.mean(angles_for_this_joint)
            
            # Apply the master calibration offset in reverse to get the "true" logical angle
            logical_angle -= utils.LOGICAL_JOINT_MASTER_OFFSETS_RAD[logical_joint_index]
            
            current_logical_angles_rad[logical_joint_index] = logical_angle
        
        elif verbose:
            print(f"[Pi] Warning: Could not determine angle for logical joint {logical_joint_index + 1} as no associated servos responded.")

    # Update the global state with the newly read values
    utils.current_logical_joint_angles_rad = current_logical_angles_rad
    
    if verbose:
        angles_deg = np.rad2deg(current_logical_angles_rad)
        print(f"[Pi] Current logical angles (deg): {np.round(angles_deg, 2)}")

    return current_logical_angles_rad


def set_current_position_as_hardware_zero(servo_id: int):
    """
    Commands a servo to set its current physical position as its new zero point.
    This value is written to the servo's EEPROM (Register 0x1F).

    Args:
        servo_id (int): The ID of the servo to calibrate.
    """
    if utils.ser is None or not utils.ser.is_open:
        print(f"[Pi] Serial port not open. Cannot set zero for servo {servo_id}.")
        return

    # --- IMPORTANT: Only act on servos that are present ---
    if servo_id not in servo_protocol.get_present_servo_ids():
        print(f"[Pi] Cannot set zero for absent servo {servo_id}.")
        return

    print(f"[Pi] Reading current position of servo {servo_id} to use as zero offset...")
    current_pos_raw = servo_protocol.read_servo_position(servo_id)

    if current_pos_raw is not None:
        # Step 1: Calculate the offset value for logging purposes only.
        # The offset is the difference between the servo's current raw position reading
        # and the ideal center value of 2048. This calculation is done here in the software
        # so we can print it out for debugging and verification. However, the servo itself
        # will perform a similar calculation internally when it receives the SET_ZERO command.
        # We do not send this calculated offset to the servo; we just log it.
        offset_value = current_pos_raw - 2048
        
        # Step 2: Clamp the offset value to a safe range for logging purposes only.
        # The safe range is from -512 to 511, which is a signed 10-bit range. This is because
        # the servo's POSITION_CORRECTION register (0x1F) is a 16-bit signed value, but typically
        # only the lower 10 bits are used for fine-tuning the zero point. Clamping prevents
        # extreme values that could indicate a problem (like a bad position read). Again, this
        # clamping is just for our logging; the servo handles its own clamping internally.
        clamped_offset = max(-512, min(511, offset_value))

        # Step 3: Print the calculated offset for verification.
        # This helps you see what the software thinks the offset should be, even though
        # the servo will compute and store its own version when it receives the command.
        print(f"[Pi] Current raw position is {current_pos_raw}. Calculated offset from center is {offset_value}.")

        # Step 4: Send the SET_ZERO command to the servo.
        # This command tells the servo to measure its own current position, calculate the offset
        # from 2048, and store that offset in its EEPROM (non-volatile memory that remembers the value
        # even after power is turned off). The servo will then use this offset for all future position
        # calculations, treating the current physical position as 'zero'.
        print(f"[Pi] Sending SET_ZERO command to servo {servo_id}...")

        if servo_protocol.calibrate_servo_middle_position(servo_id):
            # Step 5: Confirm success and wait for EEPROM write.
            # If the command succeeds, the servo has updated its zero point. We wait 0.1 seconds
            # to give the servo time to finish writing the new offset value to its EEPROM memory.
            # This delay is important because EEPROM writes take a small amount of time, and trying
            # to read or write to the servo too soon could cause errors.
            print(f"[Pi] Servo {servo_id} has set its current position as the new zero point.")
            time.sleep(0.1)
        else:
            print(f"[Pi] Failed to send SET_ZERO command to servo {servo_id}.")
    else:
        # Step 6: Handle the case where reading the current position fails.
        # If we cannot read the current position, we still try to send the SET_ZERO command.
        # Many servos can handle the zeroing internally without needing the software to provide
        # the offset value. This is a fallback to make the function more robust.
        print(f"[Pi] Could not read current position of servo {servo_id}. Attempting direct Calibrate-Middle.")
        if servo_protocol.calibrate_servo_middle_position(servo_id):
            print(f"[Pi] Servo {servo_id} set zero successfully with Calibrate-Middle.")
            time.sleep(0.1)
        else:
            print(f"[Pi] Failed to send Calibrate-Middle to servo {servo_id}.")

    # Step 7: Refresh the angle limits after zeroing.
    # After changing the zero point, the servo's understanding of its position range changes.
    # We call this function to re-apply the angle limits from the URDF configuration file,
    # ensuring the servo knows its new safe movement range. This is especially important for
    # the gripper (servo ID 100) because its limits are different from the arm joints.
    if servo_id == utils.SERVO_ID_GRIPPER:
        print(f"[Pi] Refreshing gripper limits after zeroing...")
        set_servo_angle_limits_from_urdf()  # This will re-apply limits for all, but that's fine


def reinitialize_servo(servo_id: int):
    """
    After a factory reset, a servo needs its essential operational parameters
    (PID gains, angle limits) to be set again. This function does that for a
    single servo.

    Args:
        servo_id (int): The ID of the servo to re-initialize.
    """
    print(f"\n[Pi] --- Re-initializing Servo {servo_id} post-reset ---")
    if utils.ser is None or not utils.ser.is_open:
        print(f"[Pi] Error: Serial port not available for re-initialization.")
        return

    # --- IMPORTANT: Only act on servos that are present ---
    if servo_id not in servo_protocol.get_present_servo_ids():
        print(f"[Pi] Cannot re-initialize absent servo {servo_id}.")
        return

    # 1. Set PID Gains
    print(f"[Pi] Setting default PID gains for servo {servo_id}...")
    if not set_servo_pid_gains(servo_id, utils.DEFAULT_KP, utils.DEFAULT_KI, utils.DEFAULT_KD):
        print(f"[Pi] WARNING: Failed to set PID gains for servo {servo_id}.")
    else:
        print(f"[Pi] PID gains set for servo {servo_id}.")
    time.sleep(0.1)

    # 2. Set Angle Limits
    try:
        config_index = utils.SERVO_IDS.index(servo_id)
        min_limit_rad, max_limit_rad = utils.URDF_JOINT_LIMITS[config_index]

        min_raw = angle_to_raw(min_limit_rad, config_index)
        max_raw = angle_to_raw(max_limit_rad, config_index)

        final_min = min(min_raw, max_raw)
        final_max = max(min_raw, max_raw)

        print(f"[Pi] Setting angle limits for servo {servo_id} to [{final_min}, {final_max}]...")
        if not servo_protocol.write_servo_angle_limits(servo_id, final_min, final_max):
            print(f"[Pi] WARNING: Failed to set angle limits for servo {servo_id}.")
        else:
            print(f"[Pi] Angle limits set for servo {servo_id}.")

    except ValueError:
        print(f"[Pi] ERROR: Servo ID {servo_id} not found in configuration. Cannot set angle limits.")
    except Exception as e:
        print(f"[Pi] An unexpected error occurred while setting angle limits for servo {servo_id}: {e}")
    
    print(f"[Pi] --- Re-initialization for Servo {servo_id} complete ---")


def get_servo_hardware_zero_offsets(servo_ids_to_check: list[int] | None = None) -> list[int]:
    """
    Reads the hardware zero offset value (Register 0x1F) from each servo.

    Args:
        servo_ids_to_check (list[int] | None): A specific list of servo IDs to query.
            If None, queries all servos listed in `utils.SERVO_IDS`.

    Returns:
        list[int]: A list of the offset values read from the servos. Returns
                   -1 for any servo that fails to respond.
    """
    target_ids = servo_ids_to_check if servo_ids_to_check is not None else utils.SERVO_IDS
    offsets = []
    for s_id in target_ids:
        offset = servo_protocol.read_servo_register_word(s_id, utils.SERVO_ADDR_POSITION_CORRECTION)
        if offset is not None:
            # The register is a signed 16-bit value, so we may need to handle negative numbers
            if offset > 32767:
                 offset -= 65536
            offsets.append(offset)
        else:
            offsets.append(-1) # Indicate failure
        time.sleep(0.01)
    return offsets


# ============================================================================
# Helper for high-frequency executors
# ============================================================================

def logical_q_to_syncwrite_tuple(logical_joint_angles_rad: list[float],
                                 speed: int = 4095,
                                 accel: int = 0) -> list[tuple[int,int,int,int]]:
    """
    A pure-python utility that converts a set of logical joint angles into the
    tuple format required by the `servo_protocol.sync_write_goal_pos_speed_accel`
    function. This is used by the closed-loop executor for direct hardware control.

    Args:
        logical_joint_angles_rad: A list of 6 joint angles in radians.
        speed: The speed for the move (0-4095).
        accel: The acceleration register value (0-254).

    Returns:
        A list of tuples, where each tuple is (servo_id, position, speed, accel).
    """
    # This map needs to be kept in sync with the physical robot structure.
    logical_to_physical_map = {
        0: [0], 1: [1, 2], 2: [3, 4], 3: [5], 4: [6], 5: [7],
    }

    commands = []
    for logical_idx, physical_indices in logical_to_physical_map.items():
        # Apply the master offset for this logical joint
        angle_with_offset = logical_joint_angles_rad[logical_idx] + utils.LOGICAL_JOINT_MASTER_OFFSETS_RAD[logical_idx]

        for physical_idx in physical_indices:
            servo_id = utils.SERVO_IDS[physical_idx]

            # --- IMPORTANT: Only command servos that are present ---
            if servo_id not in servo_protocol.get_present_servo_ids():
                continue

            raw_pos = angle_to_raw(angle_with_offset, physical_idx)
            commands.append((servo_id, raw_pos, speed, accel))

    return commands
