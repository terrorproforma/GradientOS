import socket
import time
import math

# =============================================================================
#                              CONFIGURATION
# =============================================================================
# This should be the hostname or IP address of your Raspberry Pi.
# Using .local hostnames requires Avahi/Bonjour/Zeroconf networking.
# If this does not work, use the direct IP address of the Pi.
PI_IP = "ai-pi.local"
UDP_PORT = 3000

# =============================================================================
#                         NETWORKING AND COMMANDS
# =============================================================================
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
PI_RESOLVED_IP = ""

try:
    PI_RESOLVED_IP = socket.gethostbyname(PI_IP)
    print(f"UDP Client configured for Pi at {PI_IP} ({PI_RESOLVED_IP}):{UDP_PORT}")
except socket.gaierror:
    print(f"CRITICAL: Could not resolve hostname '{PI_IP}'.")
    print("Please check your network connection, DNS, or use the Pi's direct IP address.")
    exit()

def send_command(command_str: str):
    """Sends a formatted command string to the robot controller."""
    sock.sendto(command_str.encode("utf-8"), (PI_IP, UDP_PORT))
    print(f"Sent: '{command_str}'")

def receive_data(timeout_seconds=2.0) -> str | None:
    """Waits for and returns a single response from the robot."""
    sock.settimeout(timeout_seconds)
    try:
        data, server_addr = sock.recvfrom(1024)
        if server_addr[0] == PI_RESOLVED_IP:
            response = data.decode("utf-8").strip()
            return response
        else:
            print(f"Warning: Received data from unexpected source: {server_addr}")
            return None
    except socket.timeout:
        print("Warning: Timeout waiting for response from Pi.")
        return None
    finally:
        sock.settimeout(None) # Disable timeout for future operations

# =============================================================================
#                         PRE-DEFINED POSES & SEQUENCES
# =============================================================================
# Predefined positions in logical joint angles (radians)
POS_ZERO = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
POS_HOME = [0.0, -0.785, 0.785, 0.0, -0.785, 0.0] # A more useful home pose
POS_REST = [0.0, -1.4, 1.5, 0.0, 0.0, 0.0]

def run_test_square_sequence():
    """
    Executes a square movement using the new high-fidelity linear move commands.
    This demonstrates the new 'plan-then-execute' and closed-loop control model.
    """
    print("\n--- Running High-Fidelity Square Test ---")
    print("This will use MOVE_LINE and WAIT_FOR_IDLE to trace a square.")
    
    # 1. Define the square corners in Cartesian space (x, y, z)
    #    These coordinates are relative to the robot's base frame.
    try:
        # First, get the current position to start the square from there.
        print("\n1. Getting current position...")
        send_command("GET_POSITION")
        response = receive_data()
        if not response or not response.startswith("CURRENT_POSITION,"):
            print("ERROR: Could not get starting position. Aborting square test.")
            return

        parts = response.split(',')
        start_pos = np.array([float(parts[1]), float(parts[2]), float(parts[3])])
        print(f"Starting from: {np.round(start_pos, 3)}")

        # Define the square corners relative to the starting position
        p1 = start_pos
        p2 = p1 + np.array([0.0, 0.1, 0.0])   # Move +10cm in Y
        p3 = p2 + np.array([0.1, 0.0, 0.0])   # Move +10cm in X
        p4 = p3 + np.array([0.0, -0.1, 0.0])  # Move -10cm in Y
        
        square_path = [p1, p2, p3, p4, p1] # Return to start
        
        # 2. Execute the sequence
        for i, point in enumerate(square_path):
            print(f"\n2.{i+1} Moving to corner {i+1}: {np.round(point, 3)}...")
            cmd = f"MOVE_LINE,{point[0]},{point[1]},{point[2]}"
            send_command(cmd)
            
            # This is crucial: wait for the move to complete before sending the next one.
            send_command("WAIT_FOR_IDLE") 
            # We don't need a response, the controller will just block until ready.
            # We add a small local delay just for visual separation in the logs.
            time.sleep(0.2) 

        print("\n--- Square Test Complete ---")

    except (ValueError, IndexError):
        print("ERROR: Failed to parse position response. Aborting square test.")
    except Exception as e:
        print(f"An unexpected error occurred during the square test: {e}")

# =============================================================================
#                            INTERACTIVE SHELL
# =============================================================================

def print_help():
    """Prints the available commands."""
    print("\n--- Interactive UDP Client for Robot Arm ---")
    print("Commands:")
    print("  --- Joint Control (Low-Level) ---")
    print("  <j1,j2,j3,j4,j5,j6>             : Send 6 joint angles (rad) with default speed/accel.")
    print("  <j1,j2,j3,j4,j5,j6,speed>       : Send 6 angles (rad) and speed (0-4095).")
    print("  <j1,j2,j3,j4,j5,j6,speed,accel> : Send 6 angles, speed (0-4095), accel (deg/s^2).")
    print("  zero                            : Send all joints to [0,0,0,0,0,0].")
    print("  home                            : Send to a predefined 'home' pose.")
    print("  rest                            : Send to a predefined 'rest' pose.")
    print("\n  --- Modern Cartesian Control ---")
    print("  move_line,x,y,z[,v,a,closed]        : Move tool tip to an absolute position.")
    print("  move_line_rel,dx,dy,dz[,s,closed]   : Move tool tip by a relative amount (s=speed multiplier).")
    print("  set_orientation,r,p,y[,d,closed]    : Smoothly set tool orientation (d=duration in sec).")
    print("    (For all above, default is precise closed-loop. Set closed to 'false', 'open', or 'off' for high-speed open-loop).")
    print("\n  --- Legacy Cartesian Commands ---")
    print("  translate,dx,dy,dz              : (Legacy) Simple blocking relative translation.")
    print("  rotate,axis,angle               : (Legacy) Simple blocking rotation (axis='x,y,z', angle=deg).")
    print("\n  --- Trajectory & State Control ---")
    print("  run_trajectory,name[,yes]       : Execute a named trajectory from trajectories.json or recorded folder.")
    print("  plan_trajectory                 : Start recording mode (teach-by-demo).")
    print("  rec_pos                         : Record current pose (FK) as way-point during recording.")
    print("  end_trajectory,name             : Finalise recording and save to recorded_trajectories/name.json.")
    print("  test_square                     : Executes a high-fidelity square movement test.")
    print("  stop                            : Gracefully stops the current high-level move or trajectory.")
    print("  wait_for_idle                   : Blocks until the current move is finished.")
    print("\n  --- System & Calibration ---")
    print("  get_position                    : Get the current calculated tool tip position (X,Y,Z).")
    print("  get_all_positions               : Get the raw position values of all 9 physical servos.")
    print("  calibrate,id                    : Start streaming live position data for a specific servo ID.")
    print("  set_zero,joint#                 : Zero all servos for a logical joint (1-6).")
    print("  factory_reset,id                : Resets a servo's EEPROM (PID, offsets, etc.) to defaults, keeping its ID.")
    print("  help                            : Show this command list.")
    print("  quit, exit                      : Exit the client.")

def main():
    """Main interactive loop."""
    print_help()

    while True:
        try:
            user_input = input("\nEnter command: ").strip().lower()

            if user_input in ["quit", "exit"]:
                break
            
            elif user_input == "help":
                print_help()
                continue
            
            elif user_input == "zero":
                send_command("0,0,0,0,0,0")
                continue
            
            elif user_input == "home":
                cmd_str = ",".join(map(str, POS_HOME))
                send_command(cmd_str)
                continue

            elif user_input == "rest":
                cmd_str = ",".join(map(str, POS_REST))
                send_command(cmd_str)
                continue

            elif user_input == "test_square":
                run_test_square_sequence()
                continue

            elif user_input.startswith("calibrate,"):
                send_command(user_input.upper())
                print(f"Request sent. Check the controller's console output for the data stream.")
                print("(Press Ctrl+C on the controller to stop its stream).")
                continue
            
            elif user_input.startswith("factory_reset,"):
                print("\n--- Factory Reset ---")
                print("This command will reset the specified servo to its factory default settings.")
                print("The servo's ID will NOT be changed.")
                print("The servo will be automatically restarted after the reset.")
                confirm = input("Are you sure you want to proceed? Type 'yes' to send the command: ").strip().lower()
                if confirm == 'yes':
                    send_command(user_input.upper())
                    print("Factory reset and restart command sent.")
                else:
                    print("Factory reset aborted by user.")
                continue
            
            elif user_input.startswith(("get_position", "get_all_positions")):
                send_command(user_input.upper())
                print("Waiting for response...")
                response = receive_data()
                if response:
                    print(f"Response from Pi: {response}")
                continue

            # For commands that are directly passed through (like MOVE_LINE, SET_ZERO, etc.)
            # No special client-side logic is needed.
            elif "," in user_input or user_input in {"stop", "wait_for_idle"}:
                # Provide alias mapping for convenience
                alias_map = {
                    "move_line_rel": "MOVE_LINE_RELATIVE",
                    "move_line": "MOVE_LINE",  # ensure proper case
                }
                # Split only on the first comma to separate the command from its arguments
                parts = user_input.split(',', 1)
                base_command = parts[0].strip()
                
                # Check if the base command is an alias
                aliased_command = alias_map.get(base_command, base_command.upper())

                if len(parts) > 1:
                    # Re-join the aliased command with its original arguments
                    full_command = f"{aliased_command},{parts[1]}"
                else:
                    # The command had no arguments (e.g., 'stop')
                    full_command = aliased_command
                
                send_command(full_command)
                
            elif user_input.startswith(("plan_trajectory", "rec_pos", "end_trajectory")):
                # Recorder commands are passthrough but need correct casing
                # Special case: END_TRAJECTORY may include a name arg; preserve after comma
                parts = user_input.split(',', 1)
                base = parts[0].upper()
                if len(parts) > 1:
                    cmd_str = f"{base},{parts[1]}"
                else:
                    cmd_str = base
                send_command(cmd_str)
                continue

            else:
                try:
                    # Check if it's a raw joint command with no command name
                    # e.g., "0.1, 0.2, 0.3, 0.4, 0.5, 0.6"
                    values = [float(p.strip()) for p in user_input.split(',')]
                    if 6 <= len(values) <= 8:
                        send_command(user_input) # Pass it straight through
                    else:
                        print("Error: Unknown command. Type 'help' for options.")
                except ValueError:
                    print("Error: Unknown command. Type 'help' for options.")

        except KeyboardInterrupt:
            print("\nExiting...")
            break
        except Exception as e:
            print(f"An unexpected error occurred: {e}")
            import traceback
            traceback.print_exc()
            break

    sock.close()
    print("UDP client closed.")

if __name__ == "__main__":
    # Import numpy for the square test, but don't make it a hard requirement
    # for the whole script.
    try:
        import numpy as np
    except ImportError:
        print("\nWARNING: numpy is not installed. The 'test_square' command will not be available.")
        print("Install it with: pip install numpy")
        def run_test_square_sequence():
            print("ERROR: Cannot run 'test_square' because numpy is not installed.")

    main() 