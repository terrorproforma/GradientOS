# Contains high-level functions for controlling the servos, acting as a
# hardware abstraction layer. Imports from servo_protocol.py.
#
# This module provides backward-compatible functions that work with the existing
# codebase while also supporting the new ActuatorBackend abstraction layer.
#
# Migration Status (Phase 2):
# ---------------------------
# Functions in this module are being migrated to use the ActuatorBackend interface.
# Each function checks if a backend is available and uses it when possible,
# falling back to legacy servo_protocol calls otherwise.
#
# For new code, prefer using the backend directly:
#   from gradient_os.arm_controller.backends import registry
#   backend = registry.get_active_backend()
#   backend.set_joint_positions(angles, speed, accel)

import time
import serial
import numpy as np
import os
import json
import glob
from typing import Iterable, Optional, TYPE_CHECKING

from . import utils
from . import servo_protocol
from . import robot_config

if TYPE_CHECKING:
    from .actuator_interface import ActuatorBackend


# =============================================================================
# Backend Accessor
# =============================================================================

def _get_backend() -> Optional['ActuatorBackend']:
    """
    Get the active ActuatorBackend instance from the registry.
    
    Returns None if no backend is configured (allows fallback to legacy code).
    This enables gradual migration - functions can check for a backend and
    use it if available, otherwise fall back to servo_protocol.
    
    Returns:
        Optional[ActuatorBackend]: The active backend, or None.
    """
    try:
        from .backends import registry
        if registry.is_instance_set():
            return registry.get_active_backend()
    except Exception:
        pass
    return None


def _use_backend() -> bool:
    """
    Check if we should use the new backend system.
    
    Returns True if a backend instance is available and initialized.
    Functions can use this for quick checks before deciding which path to take.
    """
    backend = _get_backend()
    return backend is not None and backend.is_initialized

def _env_truthy(var_name: str, default: bool = False) -> bool:
    """Return True if the given env var is set to a truthy value."""
    val = os.environ.get(var_name)
    if val is None:
        return default
    return val.strip().lower() in {"1", "true", "yes", "on"}

_ALL_CANDIDATE_PATTERNS: tuple[str, ...] = (
    "/dev/serial/by-id/*",
    "/dev/serial/by-path/*",
    "/dev/ttyUSB*",
    "/dev/ttyACM*",
    "/dev/ttyAMA*",   # Raspberry Pi PL011 / mini-UART (legacy)
    "/dev/ttyTHS*",   # NVIDIA Jetson hardware UART
    "/dev/ttyS*",     # Generic on-board UARTs (includes ttyS0 on many SBCs)
)

_USB_ONLY_PATTERNS: tuple[str, ...] = (
    # Prefer stable, USB-backed symlinks
    "/dev/serial/by-id/usb-*",
    # Only entries behind a USB hop on the host bus
    "/dev/serial/by-path/*-usb-*/**",
    # Classic USB serial drivers
    "/dev/ttyACM*",
    "/dev/ttyUSB*",
)

def _get_candidate_patterns() -> tuple[str, ...]:
    """
    Default to scanning only USB serial devices. To also scan on-board UARTs,
    set the environment variable SERIAL_SCAN_INCLUDE_UART to a truthy value.
    """
    include_uart = _env_truthy("SERIAL_SCAN_INCLUDE_UART", default=False)
    if include_uart:
        return _ALL_CANDIDATE_PATTERNS
    return _USB_ONLY_PATTERNS


def _is_serial_response_valid(response: object, servo_id: int) -> bool:
    """Check whether the raw bytes returned from a probe look like a servo status packet."""
    if not isinstance(response, (bytes, bytearray)):
        return False
    if len(response) != 6:
        return False
    return response[0] == 0xFF and response[1] == 0xFF and response[2] == servo_id


def _ping_over_handle(handle: serial.Serial, servo_id: int) -> bool:
    """Issue a single servo ping over an already-open serial handle."""
    ping_command = bytearray(6)
    ping_command[0] = servo_protocol.SERVO_HEADER
    ping_command[1] = servo_protocol.SERVO_HEADER
    ping_command[2] = servo_id
    ping_command[3] = 2  # Length
    ping_command[4] = servo_protocol.SERVO_INSTRUCTION_PING
    ping_command[5] = servo_protocol.calculate_checksum(ping_command[2:5])

    try:
        handle.reset_input_buffer()
    except Exception:
        # Some drivers may not support reset_input_buffer; proceed anyway
        pass

    handle.write(ping_command)
    response = handle.read(6)
    return _is_serial_response_valid(response, servo_id)


def _candidate_serial_devices() -> list[str]:
    """
    Enumerate potential serial device paths, preferring persistent /dev/serial/by-id names
    and deduplicating symlink targets.
    """
    candidates: list[str] = []
    seen_realpaths: set[str] = set()
    debug = _env_truthy("SERIAL_SCAN_DEBUG", default=False)
    if debug:
        print("[Pi] SERIAL_SCAN_DEBUG=1: starting candidate enumeration...")

    # 1) Prefer pyudev to enumerate only USB-backed TTYs and select stable symlinks
    try:
        use_udev_default = True
        env_use_udev = os.environ.get("SERIAL_SCAN_USE_UDEV", "")
        if env_use_udev.strip():
            truthy = {"1", "true", "yes", "on"}
            use_udev_default = env_use_udev.strip().lower() in truthy
        if use_udev_default:
            import importlib
            pyudev_spec = importlib.util.find_spec("pyudev")
            if debug:
                print(f"[Pi] pyudev present: {pyudev_spec is not None}")
            if pyudev_spec is not None:
                pyudev = importlib.import_module("pyudev")
                ctx = pyudev.Context()
                for dev in ctx.list_devices(subsystem="tty"):
                    try:
                        # Filter strictly to USB-backed TTYs
                        is_usb = dev.properties.get("ID_BUS") == "usb" or any(p.subsystem == "usb" for p in dev.ancestors)
                        if not is_usb:
                            continue
                        node = dev.device_node
                        if not node:
                            continue
                        # Prefer stable symlinks under by-id/by-path if present
                        devlinks = dev.properties.get("DEVLINKS", "")
                        symlinks = devlinks.split() if devlinks else []
                        preferred = None
                        for s in symlinks:
                            if s.startswith("/dev/serial/by-id/"):
                                preferred = s
                                break
                        if preferred is None:
                            for s in symlinks:
                                if s.startswith("/dev/serial/by-path/"):
                                    preferred = s
                                    break
                        candidate_path = preferred if preferred else node
                        try:
                            realpath = os.path.realpath(candidate_path)
                        except OSError:
                            continue
                        if realpath in seen_realpaths:
                            continue
                        if not os.path.exists(realpath):
                            continue
                        seen_realpaths.add(realpath)
                        candidates.append(candidate_path)
                        if debug:
                            print(f"[Pi] udev candidate: {candidate_path} -> {realpath}")
                    except Exception:
                        continue
    except Exception:
        # If any issue occurs with udev scanning, silently fall back to glob patterns
        if debug:
            print("[Pi] udev enumeration failed; falling back to glob patterns")

    # 2) Fallback to glob patterns
    patterns = _get_candidate_patterns()
    if debug:
        print(f"[Pi] glob patterns in use: {patterns}")
    for pattern in patterns:
        for path in sorted(glob.glob(pattern, recursive=True)):
            try:
                realpath = os.path.realpath(path)
            except OSError:
                continue
            if realpath in seen_realpaths:
                continue
            if not os.path.exists(realpath):
                continue
            seen_realpaths.add(realpath)
            candidates.append(path)
            if debug:
                print(f"[Pi] glob candidate: {path} -> {realpath}")

    # 3) If list_ports reports additional USB serial devices we missed, append them
    try:
        from serial.tools import list_ports as _list_ports
        for port in _list_ports.comports():
            if not port.device:
                continue
            try:
                realpath = os.path.realpath(port.device)
            except OSError:
                continue
            if realpath in seen_realpaths:
                continue
            if not os.path.exists(realpath):
                continue
            seen_realpaths.add(realpath)
            candidates.append(port.device)
            if debug:
                print(f"[Pi] list_ports candidate: {port.device} -> {realpath}")
    except Exception:
        # pyserial list_ports can fail on some minimal installations; ignore
        if debug:
            print("[Pi] list_ports enumeration failed or unavailable")

    return candidates


def _probe_serial_device(path: str, servo_ids: Iterable[int]) -> bool:
    """
    Attempt to open the given serial device and confirm a connected servo responds to ping.
    Only overrides the configured serial port if a unique responsive device is found.
    """
    probe_ids = list(servo_ids)
    if len(probe_ids) > 4:
        probe_ids = probe_ids[:4]
    if utils.SERVO_ID_GRIPPER not in probe_ids:
        probe_ids.append(utils.SERVO_ID_GRIPPER)

    try:
        with serial.Serial(path, utils.BAUD_RATE, timeout=0.1) as handle:
            # Allow adapter/TTL bridges a brief moment to initialize
            time.sleep(0.05)
            for _ in range(2):  # attempt each servo twice before giving up
                for servo_id in probe_ids:
                    if _ping_over_handle(handle, servo_id):
                        return True
            return False
    except serial.SerialException as exc:
        print(f"[Pi] Serial auto-detect skipped '{path}': {exc}")
    except Exception as exc:
        print(f"[Pi] Serial auto-detect encountered an unexpected error on '{path}': {exc}")
    return False


def _resolve_serial_port() -> Optional[str]:
    """
    Determine the most appropriate serial port to use for the servo bus.
    Priority:
      1. Explicit SERIAL_PORT environment variable (if the device exists)
      2. Configured default (utils.SERIAL_PORT) if it exists
      3. Auto-detected, responsive device (unique match)
    """
    env_override = os.environ.get("SERIAL_PORT")
    if env_override:
        if os.path.exists(env_override):
            print(f"[Pi] Using serial port from environment override: {env_override}")
            return env_override
        else:
            print(f"[Pi] Warning: SERIAL_PORT override '{env_override}' does not exist.")

    if utils.SERIAL_PORT and os.path.exists(utils.SERIAL_PORT):
        print(f"[Pi] Using configured serial port: {utils.SERIAL_PORT}")
        return utils.SERIAL_PORT

    candidates = _candidate_serial_devices()
    if not candidates:
        print("[Pi] Serial auto-detect found no candidate devices. Falling back to configured path.")
        return utils.SERIAL_PORT

    responsive = [path for path in candidates if _probe_serial_device(path, utils.SERVO_IDS)]

    if len(responsive) == 1:
        detected = responsive[0]
        print(f"[Pi] Auto-detected servo serial port: {detected}")
        return detected

    if len(responsive) > 1:
        print("[Pi] Warning: Multiple serial devices responded to servo pings:")
        for path in responsive:
            print(f"  - {path}")
        print("[Pi] Please set SERIAL_PORT to the desired device. Falling back to configured path.")
        return utils.SERIAL_PORT

    print("[Pi] Serial auto-detect did not find a responsive device. Falling back to configured path.")
    return utils.SERIAL_PORT

def _is_jetson_platform() -> bool:
    """Best-effort detection of NVIDIA Jetson platforms via device-tree model."""
    model_paths = (
        "/proc/device-tree/model",
        "/sys/firmware/devicetree/base/model",
    )
    for path in model_paths:
        try:
            with open(path, "r") as fp:
                model = fp.read().strip().lower()
            if "jetson" in model or "nvidia" in model:
                return True
        except Exception:
            pass
    return False

def _print_serial_port_help() -> None:
    """
    Print platform-aware instructions to resolve serial access issues.
    Avoids Raspberry Pi–specific guidance on non-Pi systems (e.g., Jetson).
    """
    if _is_jetson_platform():
        print("Jetson hints:")
        print("  - Ensure your user is in the 'dialout' (and possibly 'tty') group:")
        print("    sudo usermod -aG dialout $USER && newgrp dialout")
        print("  - If using the on-board UART (e.g., /dev/ttyTHS1), free it from getty:")
        print("    sudo systemctl disable --now nvgetty.service || true")
        print("    sudo systemctl disable --now serial-getty@ttyS0.service || true")
        print("  - Prefer stable paths under /dev/serial/by-id for USB-TTL adapters.")
        print("  - Verify device presence and permissions: ls -l /dev/ttyUSB* /dev/ttyACM* /dev/ttyTHS* 2>/dev/null")
        print("  - You can override detection with the SERIAL_PORT env var or --serial-port flag.")
    else:
        print("Linux hints:")
        print("  - Ensure your user is in the 'dialout' (and possibly 'tty') group:")
        print("    sudo usermod -aG dialout $USER && newgrp dialout")
        print("  - Prefer stable paths under /dev/serial/by-id for USB-TTL adapters.")
        print("  - Verify device presence and permissions: ls -l /dev/ttyUSB* /dev/ttyACM* 2>/dev/null")
        print("  - You can override detection with the SERIAL_PORT env var or --serial-port flag.")

def initialize_servos():
    """
    Initializes the serial connection to the servos, checks for their presence,
    and sets their default PID gains. This function must be called once at application startup.
    
    Migration Note:
    ---------------
    If an ActuatorBackend is available and initialized, this function will use it
    to populate the present servo IDs and gripper status. The backend handles
    serial port management, pinging, and PID configuration internally.
    
    For legacy operation (no backend), this function opens the serial port directly
    and performs all initialization via servo_protocol.
    """
    print("[Pi] Initializing servos...")
    
    # Check if we have a backend available
    backend = _get_backend()
    if backend is not None and backend.is_initialized:
        # --- Backend-based initialization ---
        print("[Pi] Using ActuatorBackend for initialization...")
        
        # Get present servo IDs from the backend
        present_servo_ids = list(backend.get_present_actuator_ids())
        
        # Update the servo_protocol cache for compatibility with legacy code
        for s_id in present_servo_ids:
            servo_protocol._present_servo_ids.add(s_id)
        
        # Check for gripper
        gripper_id = backend.gripper_actuator_id
        if gripper_id and gripper_id in present_servo_ids:
            utils.gripper_present = True
            print(f"[Pi] Gripper (ID {gripper_id}) is present.")
        else:
            utils.gripper_present = False
            print(f"[Pi] Gripper is ABSENT.")
        
        # The backend's serial port should be accessible for legacy code
        if hasattr(backend, 'serial_port') and backend.serial_port:
            utils.ser = backend.serial_port
        
        print("[Pi] Servos initialized via backend.")
        return
    
    # --- Legacy initialization (no backend) ---
    resolved_port = _resolve_serial_port()
    if resolved_port:
        utils.SERIAL_PORT = resolved_port

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
        # Load persisted PID overrides if available
        overrides_path = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", "config", "pid_gains.json"))
        pid_overrides: dict[str, list[int]] | None = None
        try:
            if os.path.isfile(overrides_path):
                with open(overrides_path, "r") as fp:
                    pid_overrides = json.load(fp)
                print(f"[Pi] Loaded PID overrides from {overrides_path}")
        except Exception as e:
            print(f"[Pi] WARNING: Failed to load PID overrides: {e}")
        all_pid_set_successfully = True
        # Only configure servos that responded to ping
        for s_id in present_servo_ids:
            if pid_overrides and str(s_id) in pid_overrides:
                try:
                    kp, ki, kd = pid_overrides[str(s_id)]
                except Exception:
                    kp, ki, kd = utils.DEFAULT_PID_GAINS.get(s_id, (utils.DEFAULT_KP, utils.DEFAULT_KI, utils.DEFAULT_KD))
            else:
                kp, ki, kd = utils.DEFAULT_PID_GAINS.get(s_id, (utils.DEFAULT_KP, utils.DEFAULT_KI, utils.DEFAULT_KD))
            if not set_servo_pid_gains(s_id, kp, ki, kd):
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
        _print_serial_port_help()
        exit() # Exit if we can't open the serial port
    print("[Pi] Servos initialized.")


def set_single_servo_position_rads(servo_id: int, position_rad: float, speed: int, accel: int):
    """
    Commands a single servo to a specified position in radians using the
    efficient SYNC_WRITE protocol, consistent with how the main arm is controlled.
    
    Migration Note:
    ---------------
    If an ActuatorBackend is available, uses backend.sync_write() which handles
    the conversion internally. Otherwise falls back to legacy servo_protocol.
    """
    backend = _get_backend()
    if backend and _use_backend():
        actuator_id = int(servo_id)
        command_speed = max(0, int(speed))
        command_accel = max(0, int(accel))
        backend.set_single_actuator_position(
            actuator_id=actuator_id,
            position_rad=float(position_rad),
            speed=command_speed,
            accel=command_accel,
        )
        print(
            f"[Pi] Commanded single actuator {actuator_id} to {position_rad:.2f} rad "
            f"with Speed={command_speed}, Accel={command_accel} (via backend)"
        )
        return
    
    # --- Fallback to legacy servo_protocol ---
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
    clamped_speed = int(max(0, min(utils.ENCODER_RESOLUTION, speed)))

    # Build the command tuple in the format the sync write function expects.
    command_tuple = (servo_id, raw_pos_value, clamped_speed, accel_reg_val)

    # Call the sync write function with a list containing just our single command.
    servo_protocol.sync_write_goal_pos_speed_accel([command_tuple])
    
    print(f"[Pi] Commanded single servo {servo_id} to {position_rad:.2f} rad ({raw_pos_value}) "
          f"with Speed={clamped_speed}, AccelReg={accel_reg_val}")


def read_single_servo_position(servo_id: int) -> Optional[int]:
    """
    Read the raw position of a single servo.
    
    Args:
        servo_id: The hardware ID of the servo.
        
    Returns:
        The raw encoder value (0-4095 typically), or None if read failed.
        
    Migration Note:
    ---------------
    If an ActuatorBackend is available, uses backend.read_single_actuator_position().
    Otherwise falls back to legacy servo_protocol.
    """
    backend = _get_backend()
    if backend and _use_backend():
        return backend.read_single_actuator_position(servo_id)
    
    # Fallback to legacy servo_protocol
    return servo_protocol.read_servo_position(servo_id)


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
        
    Migration Note:
    ---------------
    If an ActuatorBackend is available, uses backend.set_joint_positions() which
    handles all the mapping and conversion internally. Otherwise falls back to
    legacy servo_protocol-based writing.
    """
    # Check if we can use the backend
    backend = _get_backend()
    if backend is not None and backend.is_initialized:
        # --- Backend-based position setting ---
        if len(logical_joint_angles_rad) != backend.num_joints:
            print(f"[Pi] Error: Expected {backend.num_joints} logical joint angles, got {len(logical_joint_angles_rad)}")
            return
        
        # Update the global state
        utils.current_logical_joint_angles_rad = list(logical_joint_angles_rad)
        
        # Use the backend
        backend.set_joint_positions(
            positions_rad=logical_joint_angles_rad,
            speed=float(speed_value),
            acceleration=acceleration_value_deg_s2,
        )
        return
    
    # --- Legacy servo_protocol-based writing ---
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
    clamped_speed_value_for_cycle = int(max(0, min(utils.ENCODER_RESOLUTION, speed_value)))

    # Use robot configuration for logical-to-physical mapping
    logical_to_physical_map = robot_config.LOGICAL_TO_PHYSICAL_MAP

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
                raw_servo_value = normalized_value * utils.ENCODER_RESOLUTION
            else: 
                raw_servo_value = (1.0 - normalized_value) * utils.ENCODER_RESOLUTION
            
            # The calculation is now simpler as we don't add a software offset.
            final_servo_pos_value = int(round(raw_servo_value))
            final_servo_pos_value = max(0, min(utils.ENCODER_RESOLUTION, final_servo_pos_value))

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
    servo_value = max(0, min(utils.ENCODER_RESOLUTION, servo_value))

    min_map_rad, max_map_rad = utils.EFFECTIVE_MAPPING_RANGES[physical_servo_config_index]

    # 1. Normalize the servo value from [0, ENCODER_RESOLUTION] to [0, 1]
    # Undo inversion based on the physical_servo_config_index
    is_direct_mapping = utils._is_servo_direct_mapping(physical_servo_config_index)

    if is_direct_mapping:
        normalized_direct = servo_value / utils.ENCODER_RESOLUTION
    else: # Standard inverted mapping
        normalized_inverted = servo_value / utils.ENCODER_RESOLUTION
        normalized_direct = 1.0 - normalized_inverted
    
    # 3. De-normalize this direct value back to the radian range for that joint.
    angle_rad = normalized_direct * (max_map_rad - min_map_rad) + min_map_rad
    
    return angle_rad


def angle_to_raw(angle_rad: float, physical_servo_config_index: int) -> int:
    """
    Converts a physical angle in radians to a raw servo value (0-ENCODER_RESOLUTION).
    This is the core conversion logic used by both single and sync servo writes.
    """
    min_map_rad, max_map_rad = utils.EFFECTIVE_MAPPING_RANGES[physical_servo_config_index]

    # Clamp the angle to the effective mapping range before normalization
    angle_for_norm = max(min_map_rad, min(max_map_rad, angle_rad))
    
    normalized_value = (angle_for_norm - min_map_rad) / (max_map_rad - min_map_rad)

    # Apply mapping direction
    if utils._is_servo_direct_mapping(physical_servo_config_index):
        raw_servo_value = normalized_value * utils.ENCODER_RESOLUTION
    else: 
        raw_servo_value = (1.0 - normalized_value) * utils.ENCODER_RESOLUTION
    
    final_servo_pos_value = int(round(raw_servo_value))
    return max(0, min(utils.ENCODER_RESOLUTION, final_servo_pos_value))


def raw_to_angle_rad(raw_value: int, physical_servo_config_index: int) -> float:
    """
    Converts a raw servo value (0-ENCODER_RESOLUTION) to a physical angle in radians.
    This is the inverse of angle_to_raw.
    """
    if raw_value is None:
        return 0.0

    servo_value = max(0, min(utils.ENCODER_RESOLUTION, raw_value))
    min_map_rad, max_map_rad = utils.EFFECTIVE_MAPPING_RANGES[physical_servo_config_index]
    
    is_direct = utils._is_servo_direct_mapping(physical_servo_config_index)
    
    if is_direct:
        normalized = servo_value / utils.ENCODER_RESOLUTION
    else:
        normalized = 1.0 - (servo_value / utils.ENCODER_RESOLUTION)
        
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
        
    Migration Note:
    ---------------
    If an ActuatorBackend is available, uses backend.set_pid_gains() which
    handles the register writes internally.
    """
    def _clamp_byte(value: int) -> int:
        try:
            iv = int(value)
        except Exception:
            iv = 0
        if iv < 0:
            return 0
        if iv > 254:
            return 254
        return iv

    orig = (kp, ki, kd)
    kp = _clamp_byte(kp)
    ki = _clamp_byte(ki)
    kd = _clamp_byte(kd)
    if orig != (kp, ki, kd):
        print(f"[Pi] PID inputs clamped to 0-254 for Servo {servo_id}: Kp={kp}, Ki={ki}, Kd={kd}")
    else:
        print(f"[Pi] Setting PID for Servo {servo_id}: Kp={kp}, Ki={ki}, Kd={kd}")

    # Check if we can use the backend
    backend = _get_backend()
    if backend is not None and backend.is_initialized:
        # --- Backend-based PID setting ---
        success = backend.set_pid_gains(servo_id, kp, ki, kd)
        if success:
            print(f"[Pi] Successfully set PID for Servo {servo_id} via backend")
        else:
            print(f"[Pi] PID setting FAILED for Servo {servo_id} via backend")
        return success
    
    # --- Legacy servo_protocol-based PID setting ---
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
        
    Migration Note:
    ---------------
    If an ActuatorBackend is available, uses backend.get_joint_positions() which
    handles all the servo reading and mapping internally. Otherwise falls back to
    legacy servo_protocol-based reading.
    """
    if verbose:
        print("[Pi] Getting current arm state...")

    # Check if we can use the backend
    backend = _get_backend()
    if backend is not None and backend.is_initialized:
        # --- Backend-based reading ---
        current_logical_angles_rad = backend.get_joint_positions(verbose=verbose)
        
        # Update the global state with the newly read values
        utils.current_logical_joint_angles_rad = current_logical_angles_rad
        
        if verbose:
            angles_deg = np.rad2deg(current_logical_angles_rad)
            print(f"[Pi] Current logical angles (deg): {np.round(angles_deg, 2)}")
        
        return current_logical_angles_rad

    # --- Legacy servo_protocol-based reading ---
    
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
    
    # Get logical-to-physical mapping from robot config (not hardcoded)
    # Build a map from logical joint index to servo IDs
    logical_to_physical_id_map = _build_logical_to_servo_id_map()

    for logical_joint_index, physical_ids in logical_to_physical_id_map.items():
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


def get_control_arm_state_rad(verbose: bool = False) -> list[float]:
    """Read joint state for motion control.

    Backends may expose a relaxed live-control feedback path that hard-fails
    only on fault/offline/unsafe states. Legacy and non-supporting backends
    continue to use the strict public state read.
    """
    backend = _get_backend()
    if backend is not None and backend.is_initialized:
        getter = getattr(backend, "get_control_joint_positions", None)
        if callable(getter):
            positions = getter(verbose=verbose)
            utils.current_logical_joint_angles_rad = positions
            return positions
    return get_current_arm_state_rad(verbose=verbose)


def _build_logical_to_servo_id_map() -> dict[int, list[int]]:
    """
    Build a map from logical joint index (0-based) to physical servo IDs.
    
    This uses the robot configuration instead of hardcoded values.
    
    Returns:
        dict[int, list[int]]: Maps logical joint index to list of servo IDs.
    """
    # Get the logical_to_physical_map from robot config
    # This maps logical joint index -> list of config indices
    # NOTE: Must access robot_config directly (not utils) to get latest values
    logical_to_physical = robot_config.LOGICAL_TO_PHYSICAL_MAP
    servo_ids = robot_config.SERVO_IDS
    
    if logical_to_physical is None or servo_ids is None:
        # Fallback if robot config not yet set
        print("[Pi] Warning: Robot config not set, cannot build logical-to-servo map")
        return {}
    
    result = {}
    for logical_idx, physical_indices in logical_to_physical.items():
        # Convert config indices to servo IDs
        result[logical_idx] = [servo_ids[idx] for idx in physical_indices]
    
    return result


def set_current_position_as_hardware_zero(servo_id: int):
    """
    Commands a servo to set its current physical position as its new zero point.
    This value is written to the servo's EEPROM (Register 0x1F).

    Args:
        servo_id (int): The ID of the servo to calibrate.
        
    Migration Note:
    ---------------
    If an ActuatorBackend is available, uses backend.set_current_position_as_zero()
    which handles the calibration command internally.
    """
    # Check if we can use the backend
    backend = _get_backend()
    if backend is not None and backend.is_initialized:
        # --- Backend-based calibration ---
        present_ids = backend.get_present_actuator_ids()
        if servo_id not in present_ids:
            print(f"[Pi] Cannot set zero for absent servo {servo_id}.")
            return
        
        print(f"[Pi] Setting zero for servo {servo_id} via backend...")
        if backend.set_current_position_as_zero(servo_id):
            print(f"[Pi] Servo {servo_id} has set its current position as the new zero point.")
            # Refresh limits for gripper
            if servo_id == utils.SERVO_ID_GRIPPER:
                print(f"[Pi] Refreshing gripper limits after zeroing...")
                set_servo_angle_limits_from_urdf()
        else:
            print(f"[Pi] Failed to set zero for servo {servo_id}.")
        return
    
    # --- Legacy servo_protocol-based calibration ---
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
    # Use robot configuration for logical-to-physical mapping
    logical_to_physical_map = robot_config.LOGICAL_TO_PHYSICAL_MAP

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
