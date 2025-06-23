# This script will be the main entry point for running the robot controller.
# It will import logic from the 'src/arm_controller' package and start the UDP server.
import socket
import time
import traceback
import sys
import os

# Add the 'src' directory to the Python path to allow importing the arm_controller package
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), 'src')))

try:
    from arm_controller import (
        command_api, 
        servo_driver, 
        servo_protocol,
        utils
    )
except ImportError as e:
    print(f"Error importing arm_controller package: {e}")
    print("Please ensure the script is run from the project root directory and 'src' is in the Python path.")
    sys.exit(1)


def main():
    """
    Main entry point for the robot controller.

    This function performs the following steps:
    1. Initializes the hardware (serial port, servos, PID gains, angle limits).
    2. Performs an initial read of servo positions to synchronize the internal state.
    3. Enters an infinite loop to listen for UDP commands.
    4. Parses incoming commands and dispatches them to the appropriate handler
       in the `command_api` module.
    5. Manages a simple calibration mode for streaming servo data.
    6. Ensures a graceful shutdown of the serial port on exit.
    """
    # Initialize the hardware
    servo_driver.initialize_servos()
    servo_driver.set_servo_angle_limits_from_urdf()
    
    # Homing Routine: Read servo positions to synchronize our internal state.
    # This prevents dangerous movements if the arm isn't at zero when the script starts.
    utils.current_logical_joint_angles_rad = servo_driver.get_current_arm_state_rad()
    
    # --- UDP Server Setup ---
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        sock.bind((utils.PI_IP, utils.UDP_PORT))
        print(f"[Controller] Listening for UDP packets on {utils.PI_IP}:{utils.UDP_PORT}")

        in_calibration_mode = False
        calibrating_servo_id = None
        calibration_client_addr = None

        while True:
            try:
                sock.settimeout(0.1) # 100ms timeout for non-blocking checks
                data, addr = sock.recvfrom(utils.BUFFER_SIZE)
                message = data.decode("utf-8").strip()
                print(f"[Controller] Received: '{message}' from {addr}")

                # --- High-Priority Commands ---
                if message.upper() == "STOP":
                    command_api.handle_stop_command()
                    continue

                # --- Command Parsing ---
                parts = message.split(',')
                command = parts[0].upper()

                # --- Mode-Based Handling (Calibration) ---
                if in_calibration_mode:
                    if command != "CALIBRATE": # Any other command exits calibration
                        print(f"[Controller] *** Exiting CALIBRATION mode for Servo ID: {calibrating_servo_id} ***")
                        in_calibration_mode = False
                        calibrating_servo_id = None
                        calibration_client_addr = None
                    else: # Continue streaming calibration data
                        raw_pos = servo_protocol.read_servo_position(calibrating_servo_id)
                        if raw_pos is not None:
                            reply = f"CALIB_DATA,{calibrating_servo_id},{raw_pos}"
                            sock.sendto(reply.encode("utf-8"), calibration_client_addr)
                        continue

                # --- Standard Command Handling ---
                if command == "CALIBRATE":
                    try:
                        servo_id_to_calibrate = int(parts[1])
                        if servo_id_to_calibrate in utils.SERVO_IDS:
                            calibrating_servo_id = servo_id_to_calibrate
                            in_calibration_mode = True
                            calibration_client_addr = addr
                            print(f"[Controller] *** Entered CALIBRATION mode for Servo ID: {calibrating_servo_id} ***")
                        else:
                            print(f"[Controller] Error: Servo ID {servo_id_to_calibrate} not in SERVO_IDS.")
                    except (ValueError, IndexError):
                        print("[Controller] Error: Invalid CALIBRATE command. Use 'CALIBRATE,ID'.")

                elif command == "SET_ZERO":
                    try:
                        joint_num = int(parts[1])  # Expect 1-6 now

                        if not (1 <= joint_num <= utils.NUM_LOGICAL_JOINTS):
                            print("[Controller] Error: Joint number must be 1-6.")
                            continue

                        # Map logical joint numbers to their physical servo IDs
                        joint_to_servo_ids = {
                            1: [10],      # Base
                            2: [20, 21],  # Shoulder
                            3: [30, 31],  # Elbow
                            4: [40],      # Wrist roll
                            5: [50],      # Wrist pitch
                            6: [60],      # Wrist yaw
                        }

                        servos_to_zero = joint_to_servo_ids[joint_num]
                        print(f"[Controller] SET_ZERO (Joint {joint_num}) will calibrate servos: {servos_to_zero}")

                        for sid in servos_to_zero:
                            servo_driver.set_current_position_as_hardware_zero(sid)

                    except (ValueError, IndexError):
                        print("[Controller] Error: Invalid SET_ZERO command. Use 'SET_ZERO,JointNum'.")

                elif command == "FACTORY_RESET":
                    try:
                        servo_id_to_reset = int(parts[1])
                        print(f"[Controller] WARNING: Received FACTORY_RESET for Servo ID: {servo_id_to_reset}.")
                        print("[Controller] This will reset all EEPROM values (PID, offsets, limits) to factory defaults, except for the ID.")
                        
                        if servo_protocol.factory_reset_servo(servo_id_to_reset):
                            print(f"[Controller] Factory reset command sent to servo {servo_id_to_reset}.")
                            # Add a longer delay for the servo to process the EEPROM write before restarting.
                            print("[Controller] Waiting 1 second for servo to process reset...")
                            time.sleep(1.0) 
                            
                            print(f"[Controller] Now sending RESTART command to servo ID {servo_id_to_reset}.")
                            if servo_protocol.restart_servo(servo_id_to_reset):
                                print(f"[Controller] Servo {servo_id_to_reset} has been reset and restarted.")
                                # CRITICAL: Re-initialize the servo with our application's settings
                                time.sleep(1.0) # Wait for servo to be fully online after restart
                                servo_driver.reinitialize_servo(servo_id_to_reset)
                            else:
                                print(f"[Controller] Failed to send restart command. Please power cycle the servo manually.")
                        else:
                            print(f"[Controller] Failed to send factory reset command.")
                    except (ValueError, IndexError):
                        print("[Controller] Error: Invalid FACTORY_RESET command. Use 'FACTORY_RESET,ID'.")

                elif command == "GET_ALL_POSITIONS":
                    # Use a single SYNC READ command for faster bulk feedback
                    positions_dict = servo_protocol.sync_read_positions(utils.SERVO_IDS)

                    # If the sync read failed, fall back to the slower per-servo read to maintain functionality
                    if positions_dict is None:
                        positions_dict = {}
                        for s_id in utils.SERVO_IDS:
                            raw_pos = servo_protocol.read_servo_position(s_id)
                            positions_dict[s_id] = raw_pos
                            time.sleep(0.01)  # brief spacing to avoid overwhelming the bus

                    # Build the reply in the format: ALL_POS_DATA,ID1,Pos1,ID2,Pos2,...
                    all_positions_data = []
                    for s_id in utils.SERVO_IDS:
                        raw_pos = positions_dict.get(s_id)
                        all_positions_data.append(str(s_id))
                        all_positions_data.append(str(raw_pos if raw_pos is not None else 'FAIL'))

                    reply = "ALL_POS_DATA," + ",".join(all_positions_data)

                    # Debug print each servo ID with its corresponding position
                    for s_id in utils.SERVO_IDS:
                        raw_pos = positions_dict.get(s_id)
                        print(f"[Controller] Servo {s_id} position: {raw_pos if raw_pos is not None else 'FAIL'}")

                    sock.sendto(reply.encode("utf-8"), addr)

                elif command == "GET_POSITION":
                    command_api.handle_get_position(sock, addr)
                
                elif command == "WAIT_FOR_IDLE":
                    command_api.handle_wait_for_idle()

                elif command == "TRANSLATE":
                    try:
                        dx, dy, dz = map(float, parts[1:4])
                        command_api.handle_translate_command(dx, dy, dz)
                    except (ValueError, IndexError):
                        print("[Controller] Error: Invalid TRANSLATE command. Use 'TRANSLATE,dx,dy,dz'.")

                elif command == "ROTATE":
                    try:
                        axis = parts[1].lower()
                        angle_deg = float(parts[2])
                        command_api.handle_rotate_command(axis, angle_deg)
                    except (ValueError, IndexError, KeyError):
                         print("[Controller] Error: Invalid ROTATE command. Use 'ROTATE,axis,degrees'.")

                elif command == "SET_ORIENTATION":
                    try:
                        roll, pitch, yaw = map(float, parts[1:4])
                        command_api.handle_set_orientation_command(roll, pitch, yaw)
                    except (ValueError, IndexError):
                        print("[Controller] Error: Invalid SET_ORIENTATION command. Use 'SET_ORIENTATION,roll,pitch,yaw'.")

                elif command == "MOVE_LINE":
                    try:
                        x, y, z = map(float, parts[1:4])
                        v = float(parts[4]) if len(parts) > 4 and parts[4].strip() != "" else utils.DEFAULT_PROFILE_VELOCITY
                        a = float(parts[5]) if len(parts) > 5 and parts[5].strip() != "" else utils.DEFAULT_PROFILE_ACCELERATION
                        f = int(parts[6]) if len(parts) > 6 and parts[6].strip() != "" else 500
                        # closed-loop flag: interpret "false|0|no" (case-insensitive) as False, everything else → True
                        closed_loop = True
                        if len(parts) > 7 and parts[7].strip() != "":
                            closed_loop = parts[7].strip().lower() not in {"false", "0", "no"}

                        command_api.handle_move_line(x, y, z, v, a, frequency=f, closed_loop=closed_loop)
                    except (ValueError, IndexError):
                        print("[Controller] Error: Invalid MOVE_LINE command. Use 'MOVE_LINE,x,y,z,[v],[a],[freq],[closed_loop]'.")

                elif command == "MOVE_LINE_RELATIVE":
                    if len(parts) < 4:
                        print("[Controller] Error: Invalid MOVE_LINE_RELATIVE command. Use '...,dx,dy,dz,[speed_multiplier]'.")
                        continue

                    # Parse numeric arguments safely
                    try:
                        dx, dy, dz = map(float, parts[1:4])
                    except ValueError:
                        print("[Controller] Error: dx,dy,dz must be numeric.")
                        continue

                    # Optional speed multiplier
                    speed_multiplier = 1.0
                    if len(parts) > 4 and parts[4].strip() != "":
                        try:
                            speed_multiplier = float(parts[4])
                        except ValueError:
                            print("[Controller] Error: speed_multiplier must be numeric.")
                            continue

                    # Call the handler outside the parsing try-block so that any
                    # runtime errors inside the motion planner don't get caught
                    # and mis-reported as a command-format error.
                    command_api.handle_move_line_relative(dx, dy, dz, speed_multiplier)

                elif command == "MOVE_PROFILED":
                    print("[Controller] WARNING: The 'MOVE_PROFILED' command is deprecated. Please use 'MOVE_LINE' for clearer intent.")
                    try:
                        x, y, z = map(float, parts[1:4])
                        v = float(parts[4]) if len(parts) > 4 else utils.DEFAULT_PROFILE_VELOCITY
                        a = float(parts[5]) if len(parts) > 5 else utils.DEFAULT_PROFILE_ACCELERATION
                        command_api.handle_move_profiled(x, y, z, v, a)
                    except (ValueError, IndexError):
                        print("[Controller] Error: Invalid MOVE_PROFILED command. Use 'MOVE_PROFILED,x,y,z,[v],[a]'.")

                elif command == "MOVE_PROFILED_RELATIVE":
                    print("[Controller] WARNING: The 'MOVE_PROFILED_RELATIVE' command is deprecated. Please use 'MOVE_LINE_RELATIVE' for clearer intent.")
                    try:
                        dx, dy, dz = map(float, parts[1:4])
                        speed = float(parts[4]) if len(parts) > 4 else 1.0
                        command_api.handle_move_profiled_relative(dx, dy, dz, speed)
                    except (ValueError, IndexError):
                        print("[Controller] Error: Invalid MOVE_PROFILED_RELATIVE command. Use '...,dx,dy,dz,[speed]'.")
                
                elif command == "RUN_TRAJECTORY":
                    try:
                        name = parts[1].lower().strip()
                        cache = parts[2].lower().strip() in ['true', '1', 'yes'] if len(parts) > 2 else False
                        command_api.handle_run_trajectory(name, use_cache=cache)
                    except IndexError:
                        print("[Controller] Error: Invalid RUN_TRAJECTORY command. Use 'RUN_TRAJECTORY,name,[use_cache]'.")
                
                # Default case for raw joint angles
                else:
                    try:
                        if len(parts) >= utils.NUM_LOGICAL_JOINTS:
                            joint_angles = [float(p) for p in parts[:utils.NUM_LOGICAL_JOINTS]]
                            speed = int(float(parts[utils.NUM_LOGICAL_JOINTS])) if len(parts) > utils.NUM_LOGICAL_JOINTS else utils.DEFAULT_SERVO_SPEED
                            accel = float(parts[utils.NUM_LOGICAL_JOINTS + 1]) if len(parts) > utils.NUM_LOGICAL_JOINTS + 1 else utils.DEFAULT_SERVO_ACCELERATION_DEG_S2
                            servo_driver.set_servo_positions(joint_angles, speed, accel)
                        else:
                            print(f"[Controller] Error: Unknown command '{message}'")
                    except ValueError:
                        print(f"[Controller] Error: Could not parse joint angle command '{message}'")


            except socket.timeout:
                if in_calibration_mode and calibrating_servo_id is not None:
                    # Keep streaming calibration data if no new command arrives
                    raw_pos = servo_protocol.read_servo_position(calibrating_servo_id)
                    if raw_pos is not None:
                        reply = f"CALIB_DATA,{calibrating_servo_id},{raw_pos}"
                        sock.sendto(reply.encode("utf-8"), calibration_client_addr)
                    time.sleep(0.2) # Avoid flooding the network
                continue

            except Exception:
                print("[Controller] An unexpected error occurred in the main loop:")
                traceback.print_exc()


    except socket.error as e:
        print(f"[Controller] Error binding UDP socket: {e}")
    except KeyboardInterrupt:
        print("\n[Controller] Shutdown requested.")
    finally:
        print("[Controller] Shutting down.")
        sock.close()
        if utils.ser and utils.ser.is_open:
            utils.ser.close()
            print("[Controller] Serial port closed.")

if __name__ == "__main__":
    main()

