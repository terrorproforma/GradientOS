# This script will be the main entry point for running the robot controller.
# It will import logic from the 'src/arm_controller' package and start the UDP server.
import socket
import time
import traceback
import sys
import os
import numpy as np # Added for gripper angle conversion
import argparse
import threading
import subprocess
import json

try:
    from .arm_controller import (
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
    parser = argparse.ArgumentParser(description="Robot Arm Controller")
    parser.add_argument(
        "--serial-port",
        type=str,
        default="/dev/ttyUSB0",
        help="The serial port to connect to the robot arm.",
    )
    parser.add_argument(
        "--sim",
        action="store_true",
        help="Run the controller against an in-memory servo simulator instead of hardware.",
    )
    args = parser.parse_args()

    # Update the serial port from the command line argument
    if args.serial_port:
        utils.SERIAL_PORT = args.serial_port

    if getattr(args, "sim", False):
        from .arm_controller import sim_backend

        sim_backend.activate()

    # Initialize the hardware
    servo_driver.initialize_servos()
    servo_driver.set_servo_angle_limits_from_urdf()

    # Homing Routine: Read servo positions to synchronize our internal state.
    # This prevents dangerous movements if the arm isn't at zero when the script starts.
    utils.current_logical_joint_angles_rad = servo_driver.get_current_arm_state_rad()
    # If gripper is present, also get its initial state
    if utils.gripper_present:
        # Use the generic and robust word-reading function
        raw_pos = servo_protocol.read_servo_register_word(
            utils.SERVO_ID_GRIPPER, 
            utils.SERVO_ADDR_PRESENT_POSITION
        )
        if raw_pos is not None:
            try:
                gripper_config_index = utils.SERVO_IDS.index(utils.SERVO_ID_GRIPPER)
                utils.current_gripper_angle_rad = servo_driver.raw_to_angle_rad(raw_pos, gripper_config_index)
                print(f"[Controller] Initial gripper angle: {np.rad2deg(utils.current_gripper_angle_rad):.1f} degrees")
            except (ValueError, IndexError):
                print("[Controller] WARNING: Could not determine initial gripper angle.")

    # --- UDP Server Setup ---
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    # Allow quick restarts without TIME_WAIT issues; ignore errors on platforms lacking the option
    try:
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    except Exception:
        pass
    try:
        # REUSEPORT lets multiple sockets bind the same UDP port; not required, but can ease restarts
        sock.setsockopt(socket.SOL_SOCKET, getattr(socket, 'SO_REUSEPORT', 15), 1)
    except Exception:
        pass
    try:
        sock.bind((utils.PI_IP, utils.UDP_PORT))
        print(f"[Controller] Listening for UDP packets on {utils.PI_IP}:{utils.UDP_PORT}")

        in_calibration_mode = False
        calibrating_servo_id = None
        calibration_client_addr = None

        # --- Telemetry publisher state ---
        telemetry_thread = None
        telemetry_stop_event = threading.Event()
        telemetry_target = None  # (ip, port)
        telemetry_hz = 10

        def _telemetry_loop():
            period = 1.0 / max(1, int(telemetry_hz))
            udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            while not telemetry_stop_event.is_set():
                try:
                    q = servo_driver.get_current_arm_state_rad(verbose=False)
                    g = utils.current_gripper_angle_rad if utils.gripper_present else None
                    msg = {"t": time.time(), "joints": [float(x) for x in q]}
                    if g is not None:
                        msg["gripper"] = float(g)
                    if telemetry_target is not None:
                        udp.sendto(json.dumps(msg).encode("utf-8"), telemetry_target)
                except Exception:
                    pass
                time.sleep(period)

        # --- Episode recorder process state ---
        recorder_proc = None
        camera_proc = None

        while True:
            try:
                sock.settimeout(0.1) # 100ms timeout for non-blocking checks
                data, addr = sock.recvfrom(utils.BUFFER_SIZE)
                message = data.decode("utf-8").strip()
                print(f"[Controller] Received: '{message}' from {addr}")

                # --- High-Priority Commands ---
                if message.upper() == "STOP":
                    command_api.handle_stop_command()
                    try:
                        sock.sendto("ACK,STOP".encode("utf-8"), addr)
                    except Exception:
                        pass
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

                        if not (1 <= joint_num <= utils.NUM_LOGICAL_JOINTS or joint_num == 7): # 7 is for gripper
                            print("[Controller] Error: Joint number must be 1-6 for arm, or 7 for gripper.")
                            continue

                        # Map logical joint numbers to their physical servo IDs
                        joint_to_servo_ids = {
                            1: [10],      # Base
                            2: [20, 21],  # Shoulder
                            3: [30, 31],  # Elbow
                            4: [40],      # Wrist roll
                            5: [50],      # Wrist pitch
                            6: [60],      # Wrist yaw
                            7: [utils.SERVO_ID_GRIPPER], # Gripper
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

                elif command == "GET_ORIENTATION":
                    command_api.handle_get_orientation(sock, addr)

                elif command == "GET_STATUS":
                    reply = f"STATUS,gripper_present,{utils.gripper_present}"
                    sock.sendto(reply.encode("utf-8"), addr)

                elif command == "DIAGNOSTICS":
                    # Toggle runtime diagnostics without restart
                    try:
                        mode = parts[1].strip().lower()
                        enable = mode in {"on", "true", "1", "yes"}
                        utils.trajectory_state["diagnostics_enabled"] = enable
                        # Also reflect in environment for planning logs that check env
                        os.environ["MINI_ARM_IK_LOG"] = "1" if enable else "0"
                        sock.sendto(f"ACK,DIAGNOSTICS,{mode}".encode("utf-8"), addr)
                        print(f"[Controller] Diagnostics set to {enable}")
                    except Exception as e:
                        print(f"[Controller] Error parsing DIAGNOSTICS command: {e}")
                        sock.sendto("ERROR,DIAGNOSTICS".encode("utf-8"), addr)

                # ------------------------------------------------------------------
                # NEW: Real-time Cartesian Jogging
                # ------------------------------------------------------------------
                elif command == "JOG_START":
                    try:
                        command_api.handle_jog_start()
                        try:
                            sock.sendto("ACK,JOG_START".encode("utf-8"), addr)
                        except Exception:
                            pass
                    except Exception as e:
                        print(f"[Controller] Error starting jog: {e}")

                elif command == "JOG_STOP":
                    try:
                        command_api.handle_jog_stop()
                        try:
                            sock.sendto("ACK,JOG_STOP".encode("utf-8"), addr)
                        except Exception:
                            pass
                    except Exception as e:
                        print(f"[Controller] Error stopping jog: {e}")

                elif command == "SET_JOG_VELOCITY":
                    try:
                        # Expect 6 numeric values: vx, vy, vz (m/s), v_roll, v_pitch, v_yaw (deg/s)
                        if len(parts) < 7:
                            print("[Controller] Error: SET_JOG_VELOCITY requires 6 values.")
                        else:
                            vx, vy, vz, v_roll, v_pitch, v_yaw = map(float, parts[1:7])
                            command_api.handle_set_jog_velocity(vx, vy, vz, v_roll, v_pitch, v_yaw)
                    except ValueError:
                        print("[Controller] Error: Non-numeric value in SET_JOG_VELOCITY.")

                elif command == "SET_GRIPPER_JOG_VELOCITY":
                    try:
                        rate = float(parts[1]) if len(parts) > 1 else 0.0
                        command_api.handle_set_gripper_jog_velocity(rate)
                    except ValueError:
                        print("[Controller] Error: Non-numeric value in SET_GRIPPER_JOG_VELOCITY.")

                elif command == "SET_JOG_DEADMAN":
                    try:
                        flag = parts[1].strip().lower() in {"true","1","yes","on","hold"}
                        command_api.handle_set_jog_deadman(flag)
                    except Exception:
                        print("[Controller] Error parsing SET_JOG_DEADMAN.")

                elif command == "SET_JOG_DEBUG":
                    try:
                        flag = parts[1].strip().lower() in {"true","1","yes","on"}
                        command_api.handle_set_jog_debug(flag)
                    except Exception:
                        print("[Controller] Error parsing SET_JOG_DEBUG.")

                elif command == "GET_JOINT_ANGLES":
                    arm_deg = np.rad2deg(utils.current_logical_joint_angles_rad)
                    reply = "JOINT_ANGLES," + ",".join(f"{deg:.2f}" for deg in arm_deg)
                    if utils.gripper_present:
                        gripper_deg = np.rad2deg(utils.current_gripper_angle_rad)
                        reply += f",{gripper_deg:.2f}"
                    sock.sendto(reply.encode("utf-8"), addr)

                elif command == "REFRESH_LIMITS":
                    servo_driver.set_servo_angle_limits_from_urdf()

                elif command == "WAIT_FOR_IDLE":
                    command_api.handle_wait_for_idle()
                    try:
                        sock.sendto("ACK,WAIT_FOR_IDLE".encode("utf-8"), addr)
                    except Exception:
                        pass

                # ------------------------------------------------------------------
                # NEW: Gripper Commands
                # ------------------------------------------------------------------
                elif command == "SET_GRIPPER":
                    try:
                        angle_deg = float(parts[1])
                        speed = int(parts[2]) if len(parts) > 2 else 100
                        accel = int(parts[3]) if len(parts) > 3 else 0
                        command_api.handle_set_gripper_state(angle_deg, speed, accel)
                    except (ValueError, IndexError):
                        print("[Controller] Error: Invalid SET_GRIPPER command. Use 'SET_GRIPPER,angle_deg,[speed],[accel]'.")

                elif command == "GET_GRIPPER_STATE":
                    command_api.handle_get_gripper_state(sock, addr)

                # ------------------------------------------------------------------
                # NEW: PID tuning commands (advanced)
                # ------------------------------------------------------------------
                elif command == "TUNE_PID_JOINT":
                    try:
                        j = int(parts[1]) - 1  # UI sends 1-6
                        amp = float(parts[2]) if len(parts) > 2 else 5.0
                        freq = int(float(parts[3])) if len(parts) > 3 else 200
                        dur = float(parts[4]) if len(parts) > 4 else 3.0
                        move_zero = (parts[5].strip().lower() in {"true","1","yes","on"}) if len(parts) > 5 else True
                        command_api.handle_tune_pid_joint(j, amplitude_deg=amp, frequency_hz=freq, duration_s=dur, move_to_zero_first=move_zero)
                        sock.sendto("ACK,TUNE_PID_JOINT".encode("utf-8"), addr)
                    except Exception as e:
                        print(f"[Controller] Error: TUNE_PID_JOINT malformed: {e}")
                        sock.sendto("ERROR,TUNE_PID_JOINT".encode("utf-8"), addr)

                elif command == "TUNE_PID_ALL":
                    try:
                        amp = float(parts[1]) if len(parts) > 1 else 5.0
                        freq = int(float(parts[2])) if len(parts) > 2 else 200
                        dur = float(parts[3]) if len(parts) > 3 else 3.0
                        move_zero_each = (parts[4].strip().lower() in {"true","1","yes","on"}) if len(parts) > 4 else True
                        command_api.handle_tune_pid_all(amplitude_deg=amp, frequency_hz=freq, duration_s=dur, move_to_zero_first_each=move_zero_each)
                        sock.sendto("ACK,TUNE_PID_ALL".encode("utf-8"), addr)
                    except Exception as e:
                        print(f"[Controller] Error: TUNE_PID_ALL malformed: {e}")
                        sock.sendto("ERROR,TUNE_PID_ALL".encode("utf-8"), addr)

                # ------------------------------------------------------------------
                # NEW: Recording commands (trajectory recorder)
                # ------------------------------------------------------------------
                elif command == "PLAN_TRAJECTORY":
                    command_api.handle_plan_trajectory_start()

                elif command == "PLAN_TRAJECTORY_POINTS":
                    try:
                        raw_tokens = [item for item in parts[1:] if item.strip() != ""]
                        coords = [float(item) for item in raw_tokens]
                        if len(coords) == 0 or len(coords) % 3 != 0:
                            raise ValueError("Coordinates must be supplied as x,y,z triples.")
                        points = [
                            (coords[i], coords[i + 1], coords[i + 2])
                            for i in range(0, len(coords), 3)
                        ]
                        command_api.handle_plan_trajectory_points(points, sock, addr)
                    except ValueError as e:
                        print(f"[Controller] Error: Invalid PLAN_TRAJECTORY_POINTS command: {e}")
                        try:
                            sock.sendto("ERROR,PLAN_TRAJECTORY_POINTS,BAD_ARGS".encode("utf-8"), addr)
                        except Exception:
                            pass
                    except Exception as e:
                        print(f"[Controller] Error while handling PLAN_TRAJECTORY_POINTS: {e}")

                elif command == "REC_POS":
                    command_api.handle_record_position()

                elif command == "END_TRAJECTORY":
                    try:
                        traj_name = parts[1].strip()
                        command_api.handle_end_trajectory(traj_name)
                    except IndexError:
                        print("[Controller] Error: Invalid END_TRAJECTORY command. Use 'END_TRAJECTORY,name'.")

                elif command == "GET_TRAJECTORIES":
                    try:
                        # Use the canonical recorded trajectories dir from command_api (root-level)
                        traj_dir = command_api.RECORDED_TRAJ_DIR

                        if not os.path.isdir(traj_dir):
                            print(f"[Controller] Trajectory directory not found: {traj_dir}")
                            sock.sendto("TRAJECTORIES,".encode("utf-8"), addr)
                            continue

                        # Get all .json files, remove extension
                        traj_files = [f.replace('.json', '') for f in os.listdir(traj_dir) if f.endswith('.json')]

                        reply = "TRAJECTORIES," + ",".join(traj_files)
                        print(f"[Controller] Sending trajectory list: {reply}")
                        sock.sendto(reply.encode("utf-8"), addr)
                    except Exception as e:
                        print(f"[Controller] Error getting trajectories: {e}")
                        sock.sendto("ERROR,TRAJECTORY_LIST_FAILED".encode("utf-8"), addr)

                # ------------------------------------------------------------------
                # Standard Command Handling continues
                # ------------------------------------------------------------------
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
                        v = float(parts[4]) if len(parts) > 4 else utils.DEFAULT_PROFILE_VELOCITY
                        a = float(parts[5]) if len(parts) > 5 else utils.DEFAULT_PROFILE_ACCELERATION
                        # Default to OPEN loop unless the user specifies 'true', 'closed', 'yes', etc.
                        closed_loop = False
                        if len(parts) > 6 and parts[6].strip() != "":
                            closed_loop = parts[6].strip().lower() in {"true", "1", "yes", "closed", "on"}
                        command_api.handle_move_line(x, y, z, v, a, closed_loop)
                    except (ValueError, IndexError):
                        print("[Controller] Error: Invalid MOVE_LINE command. Use 'MOVE_LINE,x,y,z,[v],[a],[closed_loop]'.")

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
                    # Default to OPEN loop unless the user specifies 'true', 'closed', 'yes', etc.
                    closed_loop = False
                    if len(parts) > 5 and parts[5].strip() != "":
                        closed_loop = parts[5].strip().lower() in {"true", "1", "yes", "closed", "on"}
                    command_api.handle_move_line_relative(dx, dy, dz, speed_multiplier, closed_loop)

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
                        loop_override = parts[3].lower().strip() in ['true', '1', 'yes'] if len(parts) > 3 else None
                        command_api.handle_run_trajectory(name, use_cache=cache, loop_override=loop_override)
                    except IndexError:
                        print("[Controller] Error: Invalid RUN_TRAJECTORY command. Use 'RUN_TRAJECTORY,name,[use_cache],[loop_override]'.")

                # ------------------------------------------------------------------
                # Telemetry control and episode recorder
                # ------------------------------------------------------------------
                elif command == "START_TELEMETRY":
                    try:
                        target = parts[1]
                        hz = int(parts[2]) if len(parts) > 2 and parts[2].strip() != "" else 10
                        host, port = target.rsplit(":", 1)
                        telemetry_hz = max(1, hz)
                        telemetry_target = (host, int(port))
                        if telemetry_thread and telemetry_thread.is_alive():
                            telemetry_stop_event.set()
                            telemetry_thread.join(timeout=0.5)
                            telemetry_stop_event.clear()
                        telemetry_thread = threading.Thread(target=_telemetry_loop, daemon=True)
                        telemetry_thread.start()
                        try:
                            sock.sendto(f"ACK,START_TELEMETRY,{target},{telemetry_hz}".encode("utf-8"), addr)
                        except Exception:
                            pass
                        print(f"[Controller] Telemetry started to {telemetry_target} at {telemetry_hz} Hz")
                    except Exception as e:
                        print(f"[Controller] Error starting telemetry: {e}")

                elif command == "STOP_TELEMETRY":
                    try:
                        if telemetry_thread and telemetry_thread.is_alive():
                            telemetry_stop_event.set()
                            telemetry_thread.join(timeout=0.5)
                        telemetry_thread = None
                        telemetry_stop_event.clear()
                        try:
                            sock.sendto("ACK,STOP_TELEMETRY".encode("utf-8"), addr)
                        except Exception:
                            pass
                        print("[Controller] Telemetry stopped")
                    except Exception:
                        pass

                elif command == "START_RECORDER":
                    # START_RECORDER,episodes_dir,prompt,base_cam,wrist_cam,fps,resize[,state_udp[,action_udp]]
                    try:
                        episodes_dir = parts[1]
                        prompt = parts[2] if len(parts) > 2 else ""
                        base_cam = parts[3] if len(parts) > 3 and parts[3] != "" else None
                        wrist_cam = parts[4] if len(parts) > 4 and parts[4] != "" else None
                        # Honor explicit camera disable sentinels
                        disable_vals = {"off", "none", "disabled"}
                        base_disabled = isinstance(base_cam, str) and base_cam.lower() in disable_vals
                        wrist_disabled = isinstance(wrist_cam, str) and wrist_cam.lower() in disable_vals
                        if base_disabled:
                            base_cam = None
                        if wrist_disabled:
                            wrist_cam = None
                        fps = int(parts[5]) if len(parts) > 5 and parts[5] != "" else 10
                        resize = int(parts[6]) if len(parts) > 6 and parts[6] != "" else 256
                        state_udp = parts[7] if len(parts) > 7 and parts[7] != "" else "0.0.0.0:5555"
                        action_udp = parts[8] if len(parts) > 8 and parts[8] != "" else None

                        # If camera URLs are not provided or set to 'auto', start internal MJPEG server(s)
                        need_auto_cams = (base_cam in (None, "", "auto")) and (wrist_cam in (None, "", "auto")) and not (base_disabled and wrist_disabled)
                        if need_auto_cams:
                            try:
                                cam_cmd = [
                                    sys.executable, "-m", "gradient_os.vision", "mjpeg",
                                    "--host", "0.0.0.0",
                                    "--port", "8080",
                                    "--both",
                                    # Prefer training-friendly 2:1 output to avoid post-resize
                                    "--width", "640", "--height", "320",
                                    "--fps", "30",
                                    "--vflip",
                                    "--hflip",
                                    "--no-overlay",
                                ]
                                print(f"[Controller] Auto-starting camera streamer (vision.mjpeg): {' '.join(cam_cmd)}")
                                # Stop existing camera proc if any
                                if camera_proc and camera_proc.poll() is None:
                                    try:
                                        camera_proc.terminate(); camera_proc.wait(timeout=1.0)
                                    except Exception:
                                        pass
                                camera_proc = subprocess.Popen(cam_cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
                                # Assign local URLs for recorder
                                base_cam = "http://127.0.0.1:8080/cam0.mjpg"
                                wrist_cam = "http://127.0.0.1:8080/cam1.mjpg"
                                print(f"[Controller] Cameras will be read from {base_cam} and {wrist_cam}")
                            except Exception as e:
                                print(f"[Controller] WARNING: Failed to auto-start cameras: {e}")

                        # Stop existing recorder if running
                        if recorder_proc and recorder_proc.poll() is None:
                            print("[Controller] Recorder already running; restarting with new settings...")
                            try:
                                recorder_proc.terminate()
                                recorder_proc.wait(timeout=1.0)
                            except Exception:
                                pass

                        cmd = [
                            sys.executable, "-m", "gradient_os.telemetry.record_episode",
                            "--episodes-dir", episodes_dir,
                            "--prompt", prompt,
                            "--fps", str(fps),
                            "--state-udp", state_udp,
                            "--no-mjpeg-autostart",
                        ]
                        if resize and int(resize) > 0:
                            cmd += ["--resize", str(resize)]
                        if base_disabled and wrist_disabled:
                            cmd += ["--no-cameras"]
                        if base_cam: cmd += ["--base-cam", base_cam]
                        if wrist_cam: cmd += ["--wrist-cam", wrist_cam]
                        if action_udp: cmd += ["--action-udp", action_udp]

                        print(f"[Controller] Starting recorder: {' '.join(cmd)}")
                        recorder_proc = subprocess.Popen(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

                        # Start internal telemetry publisher to localhost if state_udp is localhost-bound
                        try:
                            host, port = state_udp.rsplit(":", 1)
                            # If binding to 0.0.0.0, send to 127.0.0.1
                            send_host = "127.0.0.1" if host in ("0.0.0.0", "::", "") else host
                            telemetry_hz = max(1, fps)
                            telemetry_target = (send_host, int(port))
                            if telemetry_thread and telemetry_thread.is_alive():
                                telemetry_stop_event.set()
                                telemetry_thread.join(timeout=0.5)
                                telemetry_stop_event.clear()
                            telemetry_thread = threading.Thread(target=_telemetry_loop, daemon=True)
                            telemetry_thread.start()
                        except Exception:
                            pass

                        try:
                            sock.sendto("ACK,START_RECORDER".encode("utf-8"), addr)
                        except Exception:
                            pass
                    except Exception as e:
                        print(f"[Controller] Error starting recorder: {e}")
                        try:
                            sock.sendto("ERROR,START_RECORDER".encode("utf-8"), addr)
                        except Exception:
                            pass

                elif command == "STOP_RECORDER":
                    try:
                        if recorder_proc and recorder_proc.poll() is None:
                            recorder_proc.terminate()
                            try:
                                recorder_proc.wait(timeout=1.5)
                            except Exception:
                                pass
                        recorder_proc = None
                        # Stop auto-started camera streamer if we launched it
                        if camera_proc and camera_proc.poll() is None:
                            try:
                                camera_proc.terminate(); camera_proc.wait(timeout=1.5)
                            except Exception:
                                pass
                        camera_proc = None
                        # Optionally stop telemetry if we started it for recorder
                        if telemetry_thread and telemetry_thread.is_alive():
                            telemetry_stop_event.set()
                            telemetry_thread.join(timeout=0.5)
                            telemetry_thread = None
                            telemetry_stop_event.clear()
                        try:
                            sock.sendto("ACK,STOP_RECORDER".encode("utf-8"), addr)
                        except Exception:
                            pass
                        print("[Controller] Recorder stopped")
                    except Exception as e:
                        print(f"[Controller] Error stopping recorder: {e}")

                # Default case for raw joint angles
                else:
                    try:
                        num_angles = len(parts)
                        if num_angles < 6:
                            print(f"[Controller] Error: Too few angles in command '{message}'")
                        else:
                            angles = [float(p) for p in parts[:min(num_angles, 7)]]
                            arm_angles = angles[:6]
                            gripper_rad = angles[6] if len(angles) == 7 else None

                            speed_index = min(num_angles, 7)
                            speed = int(float(parts[speed_index])) if len(parts) > speed_index else utils.DEFAULT_SERVO_SPEED
                            accel_index = speed_index + 1
                            accel = float(parts[accel_index]) if len(parts) > accel_index else utils.DEFAULT_SERVO_ACCELERATION_DEG_S2

                            servo_driver.set_servo_positions(arm_angles, speed, accel)
                            if gripper_rad is not None:
                                command_api.handle_set_gripper_state(np.rad2deg(gripper_rad), speed, accel)
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
