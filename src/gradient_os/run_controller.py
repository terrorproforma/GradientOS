# This script will be the main entry point for running the robot controller.
# It will import logic from the 'src/arm_controller' package and start the UDP server.
#
# Configuration:
# --------------
# The robot and servo backend are configured via command-line arguments:
#   --robot: Which robot configuration to use (e.g., "gradient0")
#   --servo-backend: Which servo backend to use (e.g., "feetech")
#
# This allows the frontend to expose robot/servo selection and makes the
# configuration explicit at startup rather than buried in code.

import socket
import time
import traceback
import sys
import os
import numpy as np
import argparse
import threading
import subprocess
import json
import base64

try:
    # Import the backend registry FIRST - it must be configured before other modules
    from .arm_controller.backends import registry as backend_registry
    
    from .arm_controller import (
        command_api,
        servo_driver,
        servo_protocol,
        utils,
        robot_config,
    )
    from .arm_controller.robots import (
        get_robot_config,
        get_robot_name_by_id,
        list_available_robots,
        RobotConfig,
    )
    from . import ik_solver
    from . import robot_assets
    from . import runtime_config
    from . import tool_library
    from .telemetry import alerts as _alerts
    from .kinematics import runtime as kinematics_runtime
except ImportError as e:
    print(f"Error importing arm_controller package: {e}")
    print("Please ensure the script is run from the project root directory and 'src' is in the Python path.")
    sys.exit(1)

# Get available servo backends from the registry
AVAILABLE_SERVO_BACKENDS = backend_registry.list_available_backends()
IK_BACKEND_CHOICES = ["ikfast", "numeric"]


def _resolve_default_robot_name() -> str:
    available = list_available_robots()
    if not available:
        raise RuntimeError("No available robot configurations were found.")
    default_robot_id = robot_assets.get_default_robot_id()
    mapped = get_robot_name_by_id(default_robot_id)
    if mapped in available:
        return mapped
    return available[0]


def main():
    """
    Main entry point for the robot controller.

    This function performs the following steps:
    1. Parses command-line arguments for robot and servo configuration.
    2. Initializes the robot configuration and servo backend.
    3. Initializes the hardware (serial port, servos, PID gains, angle limits).
    4. Performs an initial read of servo positions to synchronize the internal state.
    5. Enters an infinite loop to listen for UDP commands.
    6. Parses incoming commands and dispatches them to the appropriate handler
       in the `command_api` module.
    7. Manages a simple calibration mode for streaming servo data.
    8. Ensures a graceful shutdown of the serial port on exit.
    """
    exit_code = 0
    restart_requested = False
    restart_reason = ""
    active_runtime_config: dict[str, object] | None = None

    # Get list of available robots for help text
    available_robots = list_available_robots()
    default_robot_name = _resolve_default_robot_name()
    
    parser = argparse.ArgumentParser(
        description="Robot Arm Controller",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=f"""
Available robots: {', '.join(available_robots)}
Available servo backends: {', '.join(AVAILABLE_SERVO_BACKENDS)}

Examples:
  python -m gradient_os.run_controller --robot gradient05 --sim
  python -m gradient_os.run_controller --robot gradient05 --allow-unsafe-overrides --servo-backend simulation
        """
    )
    parser.add_argument(
        "--robot",
        type=str,
        default=None,
        choices=available_robots,
        help=(
            f"Robot configuration to use. Available: {', '.join(available_robots)} "
            f"(default: runtime policy -> {default_robot_name})"
        ),
    )
    parser.add_argument(
        "--servo-backend",
        type=str,
        default=None,
        choices=AVAILABLE_SERVO_BACKENDS,
        help=(
            f"Development override: servo backend ({', '.join(AVAILABLE_SERVO_BACKENDS)}). "
            "Requires --allow-unsafe-overrides."
        ),
    )
    parser.add_argument(
        "--ik-solver-backend",
        type=str,
        default=None,
        choices=IK_BACKEND_CHOICES,
        help=(
            "Development override: IK backend (ikfast|numeric). "
            "Requires --allow-unsafe-overrides."
        ),
    )
    parser.add_argument(
        "--allow-unsafe-overrides",
        action="store_true",
        help=(
            "Enable development-only override paths (solver/backend). "
            "Without this flag, robot policy is authoritative."
        ),
    )
    parser.add_argument(
        "--serial-port",
        type=str,
        default=None,
        help="Override the serial port (default: from robot config).",
    )
    parser.add_argument(
        "--sim",
        action="store_true",
        help="Run the controller against an in-memory servo simulator instead of hardware.",
    )
    parser.add_argument(
        "--list-robots",
        action="store_true",
        help="List available robot configurations and exit.",
    )
    args = parser.parse_args()

    # Handle --list-robots
    if args.list_robots:
        print(f"Default robot policy: {default_robot_name}")
        print("Available robot configurations:")
        for robot_name in available_robots:
            try:
                cfg = get_robot_config(robot_name)
                print(
                    f"  - {robot_name}: {cfg.name} v{cfg.version} "
                    f"({cfg.num_logical_joints} joints, {cfg.num_physical_actuators} actuators, "
                    f"ik={cfg.default_ik_solver_backend}, servo={cfg.default_servo_backend})"
                )
            except Exception as e:
                print(f"  - {robot_name}: (error loading: {e})")
        sys.exit(0)

    desired_runtime = runtime_config.load_runtime_config()
    desired_obj = desired_runtime.get("desired", {}) if isinstance(desired_runtime, dict) else {}
    desired_robot = str(desired_obj.get("robot", "")).strip()
    desired_active_tool_id = desired_obj.get("active_tool_id")
    desired_allow_unsafe = bool(desired_obj.get("allow_unsafe_overrides", False))
    desired_overrides = desired_obj.get("overrides", {})
    desired_overrides = desired_overrides if isinstance(desired_overrides, dict) else {}
    allow_unsafe_overrides = runtime_config.resolve_allow_unsafe_overrides(
        cli_flag=bool(args.allow_unsafe_overrides),
        desired_flag=desired_allow_unsafe,
    )

    requested_robot_name = args.robot or desired_robot or default_robot_name
    if requested_robot_name not in available_robots:
        print(
            f"[Controller] WARNING: Requested robot '{requested_robot_name}' is unavailable. "
            f"Falling back to policy default '{default_robot_name}'."
        )
        requested_robot_name = default_robot_name

    # ==========================================================================
    # Initialize Robot Configuration
    # ==========================================================================
    print(f"[Controller] Loading robot configuration: {requested_robot_name}")
    try:
        selected_robot = get_robot_config(requested_robot_name)
        print(f"[Controller] Robot: {selected_robot.name} v{selected_robot.version}")
        print(f"[Controller]   - {selected_robot.num_logical_joints} logical joints")
        print(f"[Controller]   - {selected_robot.num_physical_actuators} physical actuators")
        print(f"[Controller]   - Gripper: {'Yes' if selected_robot.has_gripper else 'No'}")

        # Update the global robot configuration
        robot_config.set_active_robot(selected_robot)

        requested_ik_backend = None
        if allow_unsafe_overrides:
            requested_ik_backend = (
                args.ik_solver_backend
                or desired_overrides.get("ik_solver_backend")
                or os.environ.get("MINI_ARM_SOLVER", "").strip().lower()
                or None
            )
        elif args.ik_solver_backend or os.environ.get("MINI_ARM_SOLVER", "").strip():
            print(
                "[Controller] WARNING: IK override requested but unsafe overrides are disabled. "
                "Using robot policy backend."
            )

        requested_servo_backend = None
        if allow_unsafe_overrides:
            requested_servo_backend = (
                args.servo_backend
                or desired_overrides.get("servo_backend")
                or None
            )
        elif args.servo_backend:
            print(
                "[Controller] WARNING: Servo backend override requested but unsafe overrides are disabled. "
                "Using robot policy backend."
            )

        active_runtime_config = runtime_config.resolve_effective_runtime(
            robot_name=requested_robot_name,
            sim_mode=bool(args.sim),
            requested_ik_solver_backend=requested_ik_backend,
            requested_servo_backend=requested_servo_backend,
            requested_active_tool_id=str(desired_active_tool_id) if desired_active_tool_id is not None else None,
            allow_unsafe_overrides=allow_unsafe_overrides,
        )

        # Keep asset resolution deterministic for the selected robot.
        os.environ["GRADIENT_ROBOT_ID"] = selected_robot.robot_id
        ik_solver.configure(
            robot_id=selected_robot.robot_id,
            backend_name=str(
                active_runtime_config.get("ik_solver", {}).get("effective_backend", "")
            ),
        )
        print(
            "[Controller] IK backend: "
            f"{active_runtime_config['ik_solver']['effective_backend']} "
            f"(source={active_runtime_config['ik_solver']['source']})"
        )
        tool_block = active_runtime_config.get("tool", {})
        if isinstance(tool_block, dict):
            try:
                kinematics_runtime.set_active_tool_definition(
                    tool_block,
                    expected_revision=None,
                    motion_state="IDLE",
                    reset_runtime_trim=True,
                )
            except Exception as tool_error:
                print(f"[Controller] WARNING: Failed to apply active tool definition: {tool_error}")
            print(
                "[Controller] Active tool: "
                f"{tool_block.get('display_name', tool_block.get('active_tool_id', 'unknown'))} "
                f"(id={tool_block.get('active_tool_id', 'identity')}, source={tool_block.get('source', 'unknown')})"
            )
    except Exception as e:
        print(f"[Controller] Error loading robot configuration '{requested_robot_name}': {e}")
        sys.exit(1)

    # ==========================================================================
    # Configure Serial Port
    # ==========================================================================
    # Priority: command-line arg > robot config default
    if args.serial_port:
        utils.SERIAL_PORT = args.serial_port
        print(f"[Controller] Serial port (from command line): {utils.SERIAL_PORT}")
    else:
        utils.SERIAL_PORT = selected_robot.default_serial_port
        print(f"[Controller] Serial port (from robot config): {utils.SERIAL_PORT}")

    # ==========================================================================
    # Configure Servo Backend (MUST be done before using any servo-dependent modules)
    # ==========================================================================
    if bool(active_runtime_config and active_runtime_config.get("mode", {}).get("sim")):
        print("[Controller] Running in SIMULATION mode (no hardware)")
    servo_backend = str(
        active_runtime_config.get("servo_backend", {}).get("effective_backend", selected_robot.default_servo_backend)
    )
    print(
        f"[Controller] Servo backend: {servo_backend} "
        f"(source={active_runtime_config['servo_backend']['source']})"
    )
    
    # Set active backend CONFIG (loads constants like encoder resolution, register addresses).
    # Simulation now has its own config module (compatible constants/parsers) so
    # runtime state/logs correctly reflect "simulation".
    backend_registry.set_active_backend(servo_backend)
    
    # Populate utils with servo-specific constants from the active backend
    utils._populate_servo_constants()
    
    # Now that backend is configured, update robot_config's BAUD_RATE
    robot_config.BAUD_RATE = backend_registry.get_default_baud_rate()
    
    # ==========================================================================
    # Create and Initialize Backend Instance
    # ==========================================================================
    # This creates the actual ActuatorBackend object that performs I/O operations.
    # The backend is created from the robot config and handles all hardware communication.
    print(f"[Controller] Creating {servo_backend} backend instance...")
    robot_config_dict = selected_robot.get_config_dict()
    
    active_backend = None
    backend_ready = False
    try:
        active_backend = backend_registry.create_backend(
            backend_name=servo_backend,
            robot_config=robot_config_dict,
            serial_port=utils.SERIAL_PORT,
        )
        
        # Set as active backend BEFORE initialization (in case other modules query it)
        backend_registry.set_active_backend_instance(active_backend)
        
        # Initialize the backend (opens serial port, pings servos, sets PID gains)
        backend_ready = bool(active_backend.initialize())
        if not backend_ready:
            print("[Controller] WARNING: Backend initialization returned False")
        
    except Exception as e:
        print(f"[Controller] Error creating backend: {e}")
        backend_ready = False
        if servo_backend == "ethercat_rtcore":
            # Critical migration rule: do NOT fall back to serial initialization when EtherCAT RTCore
            # backend is selected. Keep the controller alive but treat motion as unavailable.
            print("[Controller] EtherCAT RTCore backend selected; skipping legacy serial init (motion unavailable).")
        else:
            print("[Controller] Falling back to legacy initialization...")
            # For backward compatibility during migration, if backend creation fails,
            # we continue with legacy initialization

    # ==========================================================================
    # Legacy Initialization (to be migrated in Phase 2)
    # ==========================================================================
    # For now, we still use servo_driver which uses servo_protocol directly.
    # Once Phase 2 is complete, this will be replaced with:
    #   active_backend.initialize()
    #   active_backend.apply_joint_limits()
    if servo_backend == "ethercat_rtcore":
        print("[Controller] EtherCAT RTCore backend active; skipping legacy serial servo initialization.")
        # Best-effort state sync (no serial). If RTCore is connected, servo_driver will read via backend.
        if backend_ready and active_backend is not None and active_backend.is_initialized:
            try:
                utils.current_logical_joint_angles_rad = servo_driver.get_current_arm_state_rad(verbose=False)
            except Exception:
                utils.current_logical_joint_angles_rad = [0.0] * selected_robot.num_logical_joints
        else:
            utils.current_logical_joint_angles_rad = [0.0] * selected_robot.num_logical_joints
        # Tool/gripper I/O is handled via EtherCAT I/O later (not serial gripper servo).
        utils.gripper_present = False
        utils.current_gripper_angle_rad = 0.0
    else:
        if args.sim:
            from .arm_controller import sim_backend
            sim_backend.activate()

        # Initialize the hardware using legacy servo_driver
        servo_driver.initialize_servos()
        # Angle limit writes are serial-servo specific (EEPROM registers). Skip for non-serial backends.
        if servo_backend == "feetech":
            servo_driver.set_servo_angle_limits_from_urdf()
        else:
            print(f"[Controller] Skipping URDF angle limit writes for backend: {servo_backend}")

        # Homing Routine: Read servo positions to synchronize our internal state.
        # This prevents dangerous movements if the arm isn't at zero when the script starts.
        utils.current_logical_joint_angles_rad = servo_driver.get_current_arm_state_rad()
        # If gripper is present, also get its initial state
        if utils.gripper_present:
            # Read gripper position via servo_driver (uses backend if available)
            raw_pos = servo_driver.read_single_servo_position(utils.SERVO_ID_GRIPPER)
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
        def _env_float(name: str, default: float) -> float:
            raw = os.environ.get(name, str(default))
            try:
                return float(raw)
            except Exception:
                return float(default)

        command_silence_warn_s = max(
            1.0, _env_float("GRADIENT_COMMAND_SILENCE_WARN_S", 10.0)
        )
        udp_reset_log_throttle_s = max(
            1.0, _env_float("GRADIENT_UDP_RESET_LOG_THROTTLE_S", 5.0)
        )

        def _telemetry_loop():
            period = 1.0 / max(1, int(telemetry_hz))
            udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            last_extra_ts = 0.0  # throttle extended servo telemetry to ~2 Hz
            
            # Get telemetry block configuration from the active backend
            telemetry_blocks = backend_registry.get_telemetry_blocks()
            
            while not telemetry_stop_event.is_set():
                try:
                    q = servo_driver.get_current_arm_state_rad(verbose=False)
                    g = utils.current_gripper_angle_rad if utils.gripper_present else None
                    msg: dict[str, object] = {"t": time.time(), "joints": [float(x) for x in q]}
                    correlation_id = utils.trajectory_state_get("last_correlation_id")
                    if isinstance(correlation_id, str) and correlation_id:
                        msg["correlation_id"] = correlation_id
                    if g is not None:
                        msg["gripper"] = float(g)
                    msg["weld_active"] = bool(utils.trajectory_state_get("weld_active", False))
                    weld_type = utils.trajectory_state_get("current_weld_type")
                    if isinstance(weld_type, str) and weld_type:
                        msg["weld_type"] = weld_type
                    planner_diag = utils.trajectory_state.get("last_planner_diagnostics")
                    if isinstance(planner_diag, dict):
                        msg["planner_diagnostics"] = planner_diag
                    # Communication health (useful for diagnosing transport issues).
                    comms = {
                        "command_link_stale": bool(
                            utils.trajectory_state_get("command_link_stale", False)
                        ),
                        "last_command_age_s": float(
                            utils.trajectory_state_get("last_command_age_s", 0.0)
                        ),
                        "recent_udp_reset_count": int(
                            utils.trajectory_state_get("recent_udp_reset_count", 0)
                        ),
                    }
                    last_peer = utils.trajectory_state_get("last_command_peer", "")
                    if isinstance(last_peer, str) and last_peer:
                        comms["last_command_peer"] = last_peer
                    msg["comms"] = comms
                    try:
                        k_snapshot = kinematics_runtime.get_runtime_state_snapshot()
                        msg["kinematics"] = {
                            "revision": int(k_snapshot.get("revision", 0)),
                            "profile_id": k_snapshot.get("profile", {}).get("profile_id"),
                            "offsets": k_snapshot.get("offsets", {}),
                        }
                    except Exception:
                        pass
                    
                    # --- Servo telemetry (voltage/temp/current/torque + alarms) ---
                    now = time.time()
                    if now - last_extra_ts >= 0.5:
                        last_extra_ts = now
                        try:
                            # Get present servo IDs from backend or use configured IDs
                            backend = backend_registry.get_active_backend()
                            if backend and hasattr(backend, 'present_servo_ids'):
                                present_ids = list(backend.present_servo_ids)
                            else:
                                present_ids = list(utils.SERVO_IDS)
                            
                            if present_ids:
                                # Read telemetry blocks using backend-defined addresses
                                block_data = []
                                for addr, length in telemetry_blocks:
                                    if backend and hasattr(backend, 'sync_read_block'):
                                        block_data.append(backend.sync_read_block(
                                            present_ids, start_address=addr, data_len=length
                                        ))
                                    else:
                                        block_data.append(servo_protocol.sync_read_block(
                                            present_ids, start_address=addr, data_len=length,
                                            timeout_s=0.05, diagnostics=False
                                        ))
                                
                                servos: dict[str, dict[str, object]] = {}
                                for sid in present_ids:
                                    sample: dict[str, object] = {}
                                    
                                    # Parse each block using backend-specific parsing
                                    for block_idx, raw_block in enumerate(block_data):
                                        data = raw_block.get(sid)
                                        if data:
                                            parsed = backend_registry.parse_telemetry_block(block_idx, data)
                                            sample.update(parsed)
                                    
                                    if sample:
                                        servos[str(sid)] = sample
                                
                                if servos:
                                    msg["servos"] = servos
                        except Exception:
                            # Do not let telemetry extras break the main joints stream
                            pass
                    
                    # Drain any alerts collected by lower layers and attach them
                    try:
                        drained = _alerts.drain_alerts(max_items=25)
                        if drained:
                            # Keep payload small: convert ts to ms for UI display
                            for a in drained:
                                # nothing to mutate; just ensure JSON-serializable
                                a["ts"] = float(a.get("ts", time.time()))
                            msg["alerts"] = drained
                    except Exception:
                            pass
                    
                    if telemetry_target is not None:
                        udp.sendto(json.dumps(msg).encode("utf-8"), telemetry_target)
                except Exception:
                    pass
                time.sleep(period)

        # --- Episode recorder process state ---
        recorder_proc = None
        camera_proc = None
        last_command_rx_monotonic = time.monotonic()
        last_comms_warning_monotonic = 0.0
        last_udp_reset_warning_monotonic = 0.0
        was_command_link_stale = False
        recent_udp_reset_count = 0
        utils.trajectory_state_update(
            command_link_stale=False,
            last_command_age_s=0.0,
            last_command_rx_ts=time.time(),
            last_command_peer="",
            recent_udp_reset_count=0,
        )

        while True:
            try:
                sock.settimeout(0.1) # 100ms timeout for non-blocking checks
                try:
                    data, addr = sock.recvfrom(utils.BUFFER_SIZE)
                except ConnectionResetError as exc:
                    # Windows UDP can emit WSAECONNRESET (10054) when a previous
                    # sendto() hit an unreachable peer. This is transport noise
                    # for datagram sockets, not a controller crash condition.
                    if getattr(exc, "winerror", None) == 10054:
                        recent_udp_reset_count += 1
                        utils.trajectory_state_set("recent_udp_reset_count", recent_udp_reset_count)
                        now = time.monotonic()
                        if now - last_udp_reset_warning_monotonic >= udp_reset_log_throttle_s:
                            print(
                                "[Controller] UDP peer reset (WinError 10054). "
                                "Ignoring and continuing."
                            )
                            last_udp_reset_warning_monotonic = now
                        continue
                    raise
                now_monotonic = time.monotonic()
                link_was_stale = was_command_link_stale
                last_command_rx_monotonic = now_monotonic
                was_command_link_stale = False
                recent_udp_reset_count = 0
                utils.trajectory_state_update(
                    command_link_stale=False,
                    last_command_age_s=0.0,
                    last_command_rx_ts=time.time(),
                    last_command_peer=f"{addr[0]}:{addr[1]}",
                    recent_udp_reset_count=0,
                )
                if link_was_stale:
                    print("[Controller] UDP command link restored.")
                message = data.decode("utf-8").strip()
                print(f"[Controller] Received: '{message}' from {addr}")

                # --- High-Priority Commands ---
                if message.upper() == "STOP":
                    corr_id = utils.begin_correlation("STOP")
                    command_api.handle_stop_command()
                    try:
                        utils.append_audit("command_completed", correlation_id=corr_id, command="STOP", result="OK")
                    except Exception:
                        pass
                    try:
                        sock.sendto("ACK,STOP".encode("utf-8"), addr)
                    except Exception:
                        pass
                    continue

                # --- Command Parsing ---
                parts = message.split(',')
                command = parts[0].upper()
                corr_id = utils.begin_correlation(command)
                try:
                    profile_revision = kinematics_runtime.get_runtime_state_snapshot().get("revision")
                except Exception:
                    profile_revision = None
                try:
                    utils.append_audit(
                        "command_dispatch",
                        correlation_id=corr_id,
                        command=command,
                        profile_revision=profile_revision,
                    )
                except Exception:
                    pass

                # --- Mode-Based Handling (Calibration) ---
                if in_calibration_mode:
                    if command != "CALIBRATE": # Any other command exits calibration
                        print(f"[Controller] *** Exiting CALIBRATION mode for Servo ID: {calibrating_servo_id} ***")
                        in_calibration_mode = False
                        calibrating_servo_id = None
                        calibration_client_addr = None
                    else: # Continue streaming calibration data
                        raw_pos = servo_driver.read_single_servo_position(calibrating_servo_id)
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
                        joint_num = int(parts[1])  # Expect 1-6 for arm, 7 for gripper
                        
                        # Get joint-to-servo mapping from robot config
                        joint_to_servo_ids = selected_robot.logical_joint_to_actuator_ids
                        max_joint = max(joint_to_servo_ids.keys())
                        
                        if joint_num not in joint_to_servo_ids:
                            print(f"[Controller] Error: Joint number must be 1-{selected_robot.num_logical_joints} for arm, or {max_joint} for gripper.")
                            continue

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

                        # Use backend if available, otherwise fall back to servo_protocol
                        backend = backend_registry.get_active_backend()
                        reset_success = False
                        if backend and hasattr(backend, 'factory_reset_actuator'):
                            reset_success = backend.factory_reset_actuator(servo_id_to_reset)
                        else:
                            reset_success = servo_protocol.factory_reset_servo(servo_id_to_reset)
                        
                        if reset_success:
                            print(f"[Controller] Factory reset command sent to servo {servo_id_to_reset}.")
                            # Add a longer delay for the servo to process the EEPROM write before restarting.
                            print("[Controller] Waiting 1 second for servo to process reset...")
                            time.sleep(1.0)

                            print(f"[Controller] Now sending RESTART command to servo ID {servo_id_to_reset}.")
                            restart_success = False
                            if backend and hasattr(backend, 'restart_actuator'):
                                restart_success = backend.restart_actuator(servo_id_to_reset)
                            else:
                                restart_success = servo_protocol.restart_servo(servo_id_to_reset)
                            
                            if restart_success:
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
                    backend = backend_registry.get_active_backend()
                    if backend and hasattr(backend, 'sync_read_positions'):
                        positions_dict = backend.sync_read_positions()
                    else:
                        positions_dict = servo_protocol.sync_read_positions(utils.SERVO_IDS)

                    # If the sync read failed, fall back to the slower per-servo read to maintain functionality
                    if positions_dict is None:
                        positions_dict = {}
                        for s_id in utils.SERVO_IDS:
                            raw_pos = servo_driver.read_single_servo_position(s_id)
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

                elif command == "GET_RUNTIME_CONFIG":
                    try:
                        if active_runtime_config is None:
                            raise RuntimeError("Runtime config is unavailable.")
                        payload = dict(active_runtime_config)
                        payload["controller"] = {
                            "pid": os.getpid(),
                        }
                        reply = "RUNTIME_CONFIG," + json.dumps(payload, separators=(",", ":"))
                        sock.sendto(reply.encode("utf-8"), addr)
                    except Exception as e:
                        msg = str(e).replace(",", ";")
                        sock.sendto(f"ERROR,RUNTIME_CONFIG,{msg}".encode("utf-8"), addr)

                elif command == "SET_ACTIVE_TOOL":
                    requested_tool_id_raw = parts[1].strip() if len(parts) > 1 else ""
                    requested_tool_id = requested_tool_id_raw if requested_tool_id_raw else None
                    try:
                        # Block until motion completes so kinematics tool transform can be
                        # applied without changing frame assumptions mid-trajectory.
                        command_api.handle_wait_for_idle()
                        selected_tool = tool_library.resolve_active_tool(
                            robot_id=selected_robot.robot_id,
                            requested_tool_id=requested_tool_id,
                        )
                        kinematics_runtime.set_active_tool_definition(
                            selected_tool,
                            expected_revision=None,
                            motion_state="IDLE",
                            reset_runtime_trim=True,
                        )
                        if isinstance(active_runtime_config, dict):
                            active_runtime_config["tool"] = {
                                "active_tool_id": selected_tool.get("tool_id"),
                                "display_name": selected_tool.get("display_name"),
                                "tool_type": selected_tool.get("tool_type"),
                                "source": selected_tool.get("selection_source", "live_update"),
                                "offset": selected_tool.get("offset"),
                                "mesh": selected_tool.get("mesh"),
                                "compatible_robot_ids": selected_tool.get("compatible_robot_ids"),
                                "weld": selected_tool.get("weld"),
                            }
                        active_tool_id = str(selected_tool.get("tool_id", "identity"))
                        print(
                            "[Controller] Active tool updated live: "
                            f"{selected_tool.get('display_name', active_tool_id)} "
                            f"(id={active_tool_id})"
                        )
                        sock.sendto(f"ACK,SET_ACTIVE_TOOL,{active_tool_id}".encode("utf-8"), addr)
                    except Exception as e:
                        msg = str(e).replace(",", ";")
                        sock.sendto(f"ERROR,SET_ACTIVE_TOOL,{msg}".encode("utf-8"), addr)

                elif command == "REQUEST_RESTART":
                    restart_reason = parts[1].strip() if len(parts) > 1 else "api-request"
                    print(f"[Controller] Restart requested: {restart_reason}")
                    try:
                        utils.append_audit(
                            "controller_restart_requested",
                            reason=restart_reason,
                            source="udp",
                        )
                    except Exception:
                        pass
                    try:
                        sock.sendto(f"ACK,REQUEST_RESTART,{restart_reason}".encode("utf-8"), addr)
                    except Exception:
                        pass
                    try:
                        command_api.handle_stop_command()
                        command_api.handle_wait_for_idle()
                    except Exception:
                        pass
                    restart_requested = True
                    break

                elif command == "GET_KINEMATICS_PROFILE":
                    try:
                        payload = command_api.handle_get_kinematics_profile()
                        reply = "KINEMATICS_PROFILE," + json.dumps(payload, separators=(",", ":"))
                        sock.sendto(reply.encode("utf-8"), addr)
                    except Exception as e:
                        msg = str(e).replace(",", ";")
                        sock.sendto(f"ERROR,KINEMATICS,INTERNAL,{msg}".encode("utf-8"), addr)

                elif command == "PATCH_RUNTIME_OFFSETS":
                    try:
                        rev_token = parts[1].strip() if len(parts) > 1 else ""
                        expected_revision = int(rev_token) if rev_token != "" else None
                        payload_b64 = parts[2].strip() if len(parts) > 2 else ""
                        if not payload_b64:
                            raise ValueError("Missing base64 payload")
                        payload_json = base64.urlsafe_b64decode(payload_b64.encode("ascii")).decode("utf-8")
                        payload = json.loads(payload_json)
                        snapshot = command_api.handle_patch_runtime_offsets(payload, expected_revision)
                        reply = "KINEMATICS_OK," + json.dumps(snapshot, separators=(",", ":"))
                        sock.sendto(reply.encode("utf-8"), addr)
                    except Exception as e:
                        code = getattr(e, "code", "INVALID_PAYLOAD")
                        msg = str(e).replace(",", ";")
                        sock.sendto(f"ERROR,KINEMATICS,{code},{msg}".encode("utf-8"), addr)

                elif command == "RESET_RUNTIME_OFFSETS":
                    try:
                        rev_token = parts[1].strip() if len(parts) > 1 else ""
                        expected_revision = int(rev_token) if rev_token != "" else None
                        snapshot = command_api.handle_reset_runtime_offsets(expected_revision)
                        reply = "KINEMATICS_OK," + json.dumps(snapshot, separators=(",", ":"))
                        sock.sendto(reply.encode("utf-8"), addr)
                    except Exception as e:
                        code = getattr(e, "code", "INVALID_PAYLOAD")
                        msg = str(e).replace(",", ";")
                        sock.sendto(f"ERROR,KINEMATICS,{code},{msg}".encode("utf-8"), addr)

                elif command == "APPLY_KINEMATICS_PROFILE":
                    try:
                        rev_token = parts[1].strip() if len(parts) > 1 else ""
                        expected_revision = int(rev_token) if rev_token != "" else None
                        payload_b64 = parts[2].strip() if len(parts) > 2 else ""
                        if not payload_b64:
                            raise ValueError("Missing base64 payload")
                        payload_json = base64.urlsafe_b64decode(payload_b64.encode("ascii")).decode("utf-8")
                        payload = json.loads(payload_json)
                        snapshot = command_api.handle_apply_kinematics_profile(payload, expected_revision)
                        reply = "KINEMATICS_OK," + json.dumps(snapshot, separators=(",", ":"))
                        sock.sendto(reply.encode("utf-8"), addr)
                    except Exception as e:
                        code = getattr(e, "code", "INVALID_PAYLOAD")
                        msg = str(e).replace(",", ";")
                        sock.sendto(f"ERROR,KINEMATICS,{code},{msg}".encode("utf-8"), addr)

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
                now_monotonic = time.monotonic()
                idle_s = now_monotonic - last_command_rx_monotonic
                utils.trajectory_state_set("last_command_age_s", float(idle_s))
                if idle_s >= command_silence_warn_s:
                    was_command_link_stale = True
                    utils.trajectory_state_set("command_link_stale", True)
                    if now_monotonic - last_comms_warning_monotonic >= command_silence_warn_s:
                        print(
                            "[Controller] WARNING: No UDP commands received for "
                            f"{idle_s:.1f}s."
                        )
                        last_comms_warning_monotonic = now_monotonic
                if in_calibration_mode and calibrating_servo_id is not None:
                    # Keep streaming calibration data if no new command arrives
                    raw_pos = servo_driver.read_single_servo_position(calibrating_servo_id)
                    if raw_pos is not None:
                        reply = f"CALIB_DATA,{calibrating_servo_id},{raw_pos}"
                        sock.sendto(reply.encode("utf-8"), calibration_client_addr)
                    time.sleep(0.2) # Avoid flooding the network
                continue

            except Exception:
                print("[Controller] An unexpected error occurred in the main loop:")
                traceback.print_exc()

        if restart_requested:
            exit_code = 75

    except socket.error as e:
        print(f"[Controller] Error binding UDP socket: {e}")
        exit_code = 1
    except KeyboardInterrupt:
        print("\n[Controller] Shutdown requested.")
        exit_code = 0
    finally:
        print("[Controller] Shutting down.")
        sock.close()
        
        # Shutdown the backend instance (closes serial port, releases resources)
        try:
            backend_registry.shutdown_backend()
        except Exception as e:
            print(f"[Controller] Backend shutdown error: {e}")
        
        # Legacy: Also close the global serial port if open
        if utils.ser and utils.ser.is_open:
            utils.ser.close()
            print("[Controller] Serial port closed.")

    if restart_requested:
        print(f"[Controller] Exiting for supervisor restart (reason={restart_reason}).")
    return exit_code

if __name__ == "__main__":
    sys.exit(main())
