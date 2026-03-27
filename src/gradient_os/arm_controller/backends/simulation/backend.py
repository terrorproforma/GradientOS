# backends/simulation/backend.py
#
# Simulation backend for GradientOS.
# Provides an in-memory simulation of the actuator system for development and testing.

from typing import Optional, TYPE_CHECKING
import math
import threading
import time
import numpy as np

from ..registry import get_encoder_resolution
from ...actuator_interface import ActuatorBackend

if TYPE_CHECKING:
    from ...robots.base import RobotConfig


class SimulationBackend(ActuatorBackend):
    """
    In-memory simulation backend for development and testing.
    
    This backend simulates the behavior of physical actuators without requiring
    actual hardware. It's useful for:
    - Development and debugging of motion planning algorithms
    - Testing trajectory execution logic
    - Running the controller UI without a physical robot
    
    The simulation maintains joint positions in radians internally and converts
    to/from raw encoder values as needed to match the interface expected by
    other parts of the system.
    """
    
    def __init__(
        self,
        robot_config: Optional[dict] = None,
        num_joints: Optional[int] = None,
        has_gripper: Optional[bool] = None,
        encoder_resolution: Optional[int] = None,
    ):
        """
        Initialize the simulation backend.
        
        Can be initialized either with a robot_config dict (preferred) or with
        explicit parameters. If robot_config is provided, its values are used
        as defaults, but explicit parameters will override them.
        
        Args:
            robot_config: Optional dict from RobotConfig.get_config_dict() to get parameters from.
            num_joints: Number of logical joints to simulate (default: 6 or from robot_config).
            has_gripper: Whether to simulate a gripper (default: False or from robot_config).
            encoder_resolution: Encoder resolution to simulate (default: 4095 for 12-bit).
        """
        # Get defaults from robot_config if provided
        if robot_config is not None:
            if isinstance(robot_config, dict):
                default_num_joints = robot_config.get('num_logical_joints', 6)
                default_has_gripper = robot_config.get('has_gripper', False)
            else:
                # Assume it's a RobotConfig object with attributes
                default_num_joints = robot_config.num_logical_joints
                default_has_gripper = robot_config.has_gripper
        else:
            # No robot config provided - require explicit parameters
            if num_joints is None:
                raise ValueError("SimulationBackend requires either robot_config or explicit num_joints parameter")
            default_num_joints = num_joints
            default_has_gripper = False
        
        # For encoder resolution, get from the active servo backend via registry
        try:
            default_encoder_resolution = get_encoder_resolution()
        except Exception:
            default_encoder_resolution = 4095  # Fallback to common 12-bit encoder
        
        # Apply overrides
        self._num_joints = num_joints if num_joints is not None else default_num_joints
        self._has_gripper = has_gripper if has_gripper is not None else default_has_gripper
        self._encoder_resolution = encoder_resolution if encoder_resolution is not None else default_encoder_resolution
        self._encoder_center = self._encoder_resolution // 2
        
        self._robot_config = robot_config
        self._positions = [0.0] * self._num_joints
        self._gripper_position = 0.0
        self._raw_positions: dict[int, int] = {}
        self._state_lock = threading.RLock()
        self._lease_jog_active = False
        self._lease_jog_timeout_s = 0.0
        self._lease_jog_vector = [0.0] * self._num_joints
        self._lease_jog_deadline_monotonic: float | None = None
        self._lease_jog_last_tick_monotonic: float | None = None

        if self._robot_config:
            self._actuator_ids = list(self._robot_config.get("actuator_ids", []))
            raw_l2p = dict(self._robot_config.get("logical_to_physical_map", {}))
            self._logical_to_physical = {
                int(logical_idx): [int(physical_idx) for physical_idx in physical_indices]
                for logical_idx, physical_indices in raw_l2p.items()
            }
            self._mapping_ranges = list(self._robot_config.get("actuator_mapping_ranges_rad", []))
            self._inverted_actuator_ids = set(self._robot_config.get("inverted_actuator_ids", set()))
            self._master_offsets = list(
                self._robot_config.get(
                    "logical_joint_master_offsets_rad",
                    [0.0] * self._num_joints,
                )
            )
            self._gripper_id = self._robot_config.get("gripper_actuator_id")
        else:
            self._actuator_ids = list(range(self._num_joints))
            self._logical_to_physical = {idx: [idx] for idx in range(self._num_joints)}
            self._mapping_ranges = [(-math.pi, math.pi)] * len(self._actuator_ids)
            self._inverted_actuator_ids = set()
            self._master_offsets = [0.0] * self._num_joints
            self._gripper_id = 100 if self._has_gripper else None

        self._initialize_raw_cache()
        self._initialized = False
    
    def initialize(self) -> bool:
        self._initialized = True
        print(f"[Sim] Simulation backend initialized with {self._num_joints} joints")
        return True
    
    def shutdown(self) -> None:
        self._initialized = False
        print("[Sim] Simulation backend shut down")
    
    @property
    def num_joints(self) -> int:
        return self._num_joints
    
    @property
    def is_initialized(self) -> bool:
        return self._initialized
    
    @property
    def encoder_resolution(self) -> int:
        """Returns the configured encoder resolution."""
        return self._encoder_resolution
    
    def set_joint_positions(
        self,
        positions_rad: list[float],
        speed: float,
        acceleration: float,
    ) -> None:
        if len(positions_rad) != self._num_joints:
            raise ValueError(f"Expected {self._num_joints} positions, got {len(positions_rad)}")
        with self._state_lock:
            self._advance_lease_jog_state_locked()
            self._positions = list(positions_rad)
            self._update_raw_from_logical()
    
    def get_joint_positions(self, verbose: bool = False) -> list[float]:
        with self._state_lock:
            self._advance_lease_jog_state_locked()
            positions = list(self._positions)
        if verbose:
            print(f"[Sim] Current positions: {np.round(positions, 3)}")
        return positions
    
    def prepare_sync_write_commands(
        self,
        positions_rad: list[float],
        speed: int = 4095,
        accel: int = 0,
    ) -> list[tuple]:
        if len(positions_rad) != self._num_joints:
            raise ValueError(f"Expected {self._num_joints} positions, got {len(positions_rad)}")

        commands: list[tuple] = []
        for logical_idx, logical_angle in enumerate(positions_rad):
            physical_indices = self._logical_to_physical.get(logical_idx, [logical_idx])
            physical_angle = float(logical_angle) + self._master_offset_for_joint(logical_idx)
            for phys_idx in physical_indices:
                if 0 <= int(phys_idx) < len(self._actuator_ids):
                    servo_id = self._actuator_ids[int(phys_idx)]
                    raw_pos = self._angle_to_raw(physical_angle, servo_id)
                    commands.append((servo_id, raw_pos, speed, accel))
        return commands
    
    def sync_write(self, commands: list[tuple]) -> None:
        # Update positions from the commands
        # Commands are in format [(servo_id, raw_pos, speed, accel), ...]
        # We keep raw actuator values exactly as commanded, then derive logical
        # joint angles from those raw values using robot mapping metadata.
        with self._state_lock:
            self._advance_lease_jog_state_locked()
            for servo_id_or_idx, raw_pos, _, _ in commands:
                servo_id = int(servo_id_or_idx)
                clamped_raw = max(0, min(self._encoder_resolution, int(raw_pos)))

                # Normal case: caller sends actuator IDs directly.
                if servo_id in self._actuator_ids:
                    self._raw_positions[servo_id] = clamped_raw
                    continue

                # Backward-compat: allow actuator index addressing.
                if 0 <= servo_id < len(self._actuator_ids):
                    mapped_servo_id = self._actuator_ids[servo_id]
                    self._raw_positions[mapped_servo_id] = clamped_raw
                    continue

                # Unknown actuator ID; keep a best-effort cache.
                self._raw_positions[servo_id] = clamped_raw

            self._update_logical_positions_from_raw()
    
    def sync_read_positions(self, servo_ids: list[int] = None, timeout_s: Optional[float] = None) -> dict[int, int]:
        # Return cached raw values for requested actuator IDs.
        with self._state_lock:
            self._advance_lease_jog_state_locked()
            result = {}
            target_ids = servo_ids if servo_ids else list(self._actuator_ids)
            for servo_id in target_ids:
                sid = int(servo_id)
                if sid in self._raw_positions:
                    result[sid] = self._raw_positions[sid]
                elif 0 <= sid < len(self._actuator_ids):
                    mapped = self._actuator_ids[sid]
                    result[sid] = self._raw_positions.get(mapped, self._encoder_center)
                else:
                    result[sid] = self._encoder_center
            return result

    def sync_read_block(
        self,
        servo_ids: list[int],
        start_address: int,
        data_len: int,
        timeout_s: Optional[float] = None,
        poll_delay_s: float = 0.0,
        diagnostics: bool = False,
    ) -> dict[int, bytes]:
        """
        Simulate a block read by returning zeroed bytes for each servo.
        
        This keeps telemetry consumers running in simulation mode without
        requiring real hardware feedback.
        """
        actuator_ids = self._robot_config.get('actuator_ids', []) if self._robot_config else []
        target_ids = servo_ids if servo_ids else list(actuator_ids)
        if not target_ids or data_len <= 0:
            return {}
        payload = bytes([0] * data_len)
        return {sid: payload for sid in target_ids}
    
    def raw_to_joint_positions(self, raw_positions: dict[int, int]) -> list[float]:
        if not raw_positions:
            return [0.0] * self._num_joints
        with self._state_lock:
            self._advance_lease_jog_state_locked()
            for sid, raw in raw_positions.items():
                sid_int = int(sid)
                if sid_int in self._actuator_ids:
                    self._raw_positions[sid_int] = int(raw)
                elif 0 <= sid_int < len(self._actuator_ids):
                    self._raw_positions[self._actuator_ids[sid_int]] = int(raw)
            self._update_logical_positions_from_raw()
            return list(self._positions)
    
    def set_single_actuator_position(
        self,
        actuator_id: int,
        position_rad: float,
        speed: int,
        accel: int,
    ) -> None:
        with self._state_lock:
            self._advance_lease_jog_state_locked()
            if actuator_id == self._gripper_id and self._has_gripper:
                self._gripper_position = position_rad
                self._raw_positions[actuator_id] = self._angle_to_raw(position_rad, actuator_id)
                return
            
            # Map servo ID to joint index
            joint_idx = self._servo_id_to_joint_index(actuator_id)
            if joint_idx is not None and 0 <= joint_idx < self._num_joints:
                self._positions[joint_idx] = position_rad
                physical_angle = position_rad + self._master_offset_for_joint(joint_idx)
                self._raw_positions[actuator_id] = self._angle_to_raw(physical_angle, actuator_id)
    
    def read_single_actuator_position(self, actuator_id: int) -> Optional[int]:
        with self._state_lock:
            self._advance_lease_jog_state_locked()
            if actuator_id is None:
                return None
            if actuator_id in self._raw_positions:
                return self._raw_positions[actuator_id]
            if 0 <= actuator_id < len(self._actuator_ids):
                mapped = self._actuator_ids[actuator_id]
                return self._raw_positions.get(mapped, self._encoder_center)
            return self._encoder_center
    
    def _servo_id_to_joint_index(self, servo_id: int) -> Optional[int]:
        """Map a servo ID to its logical joint index."""
        if not self._robot_config:
            return servo_id if 0 <= servo_id < self._num_joints else None
        
        actuator_ids = self._robot_config.get('actuator_ids', [])
        logical_to_physical = self._robot_config.get('logical_to_physical_map', {})
        
        for logical_idx, physical_indices in logical_to_physical.items():
            for phys_idx in physical_indices:
                if phys_idx < len(actuator_ids) and actuator_ids[phys_idx] == servo_id:
                    return logical_idx
        return None
    
    def set_current_position_as_zero(self, actuator_id: int) -> bool:
        print(f"[Sim] Set zero for actuator {actuator_id}")
        return True
    
    def set_pid_gains(self, actuator_id: int, kp: int, ki: int, kd: int) -> bool:
        print(f"[Sim] Set PID for actuator {actuator_id}: Kp={kp}, Ki={ki}, Kd={kd}")
        return True
    
    def apply_joint_limits(self) -> bool:
        print("[Sim] Applied joint limits (simulated)")
        return True
    
    @property
    def has_gripper(self) -> bool:
        return self._has_gripper
    
    @property
    def gripper_actuator_id(self) -> Optional[int]:
        return 100 if self._has_gripper else None
    
    def get_gripper_position(self) -> Optional[float]:
        return self._gripper_position if self._has_gripper else None

    def supports_joint_velocity_lease_jog(self) -> bool:
        return True

    def get_jog_capabilities(self) -> dict[str, object]:
        return {
            "joint_velocity_lease": True,
            "realtime_jog_compat": False,
            "backend": "simulation",
            "watchdog_source": "controller_simulated",
        }

    def start_joint_velocity_lease_jog(self, timeout_s: float) -> None:
        with self._state_lock:
            self._advance_lease_jog_state_locked()
            self._set_lease_jog_command_locked([0.0] * self._num_joints, timeout_s)

    def update_joint_velocity_lease_jog(
        self,
        joint_velocities_rad_s: list[float],
        timeout_s: float,
    ) -> None:
        if len(joint_velocities_rad_s) != self._num_joints:
            raise ValueError(
                f"Expected {self._num_joints} joint velocities, got {len(joint_velocities_rad_s)}"
            )
        with self._state_lock:
            self._advance_lease_jog_state_locked()
            self._set_lease_jog_command_locked(joint_velocities_rad_s, timeout_s)

    def stop_joint_velocity_lease_jog(self, *, quick_stop: bool = False) -> None:
        _ = quick_stop
        with self._state_lock:
            self._advance_lease_jog_state_locked()
            self._clear_lease_jog_locked(now=time.monotonic())
    
    def get_present_actuator_ids(self) -> set[int]:
        ids = set(self._actuator_ids)
        if self._has_gripper and self._gripper_id is not None:
            ids.add(self._gripper_id)
        return ids
    
    def ping_actuator(self, actuator_id: int) -> bool:
        return actuator_id in self.get_present_actuator_ids()

    def _initialize_raw_cache(self) -> None:
        self._raw_positions.clear()
        for servo_id in self._actuator_ids:
            self._raw_positions[int(servo_id)] = self._encoder_center
        if self._has_gripper and self._gripper_id is not None:
            self._raw_positions[int(self._gripper_id)] = self._encoder_center

    def _set_lease_jog_command_locked(self, joint_velocities_rad_s: list[float], timeout_s: float) -> None:
        now = time.monotonic()
        self._lease_jog_active = True
        self._lease_jog_timeout_s = max(0.0, float(timeout_s))
        self._lease_jog_vector = [float(value) for value in joint_velocities_rad_s]
        self._lease_jog_deadline_monotonic = (
            now + self._lease_jog_timeout_s if self._lease_jog_timeout_s > 0.0 else now
        )
        self._lease_jog_last_tick_monotonic = now

    def _clear_lease_jog_locked(self, *, now: float | None = None) -> None:
        now_value = time.monotonic() if now is None else float(now)
        self._lease_jog_active = False
        self._lease_jog_timeout_s = 0.0
        self._lease_jog_vector = [0.0] * self._num_joints
        self._lease_jog_deadline_monotonic = None
        self._lease_jog_last_tick_monotonic = now_value

    def _advance_lease_jog_state_locked(self, now: float | None = None) -> None:
        if not self._lease_jog_active:
            return

        now_value = time.monotonic() if now is None else float(now)
        last_tick = float(self._lease_jog_last_tick_monotonic or now_value)
        deadline = self._lease_jog_deadline_monotonic
        advance_until = now_value
        if isinstance(deadline, (int, float)) and deadline > 0.0:
            advance_until = min(now_value, float(deadline))

        dt = max(0.0, advance_until - last_tick)
        if dt > 0.0:
            self._positions = [
                float(position) + float(velocity) * dt
                for position, velocity in zip(self._positions, self._lease_jog_vector)
            ]
            self._update_raw_from_logical()
            self._lease_jog_last_tick_monotonic = advance_until

        if isinstance(deadline, (int, float)) and now_value >= float(deadline):
            self._clear_lease_jog_locked(now=now_value)

    def _master_offset_for_joint(self, logical_joint_idx: int) -> float:
        if 0 <= logical_joint_idx < len(self._master_offsets):
            return float(self._master_offsets[logical_joint_idx])
        return 0.0

    def _servo_id_to_config_index(self, servo_id: int) -> Optional[int]:
        try:
            return self._actuator_ids.index(int(servo_id))
        except ValueError:
            return None

    def _is_direct_mapping(self, servo_id: int) -> bool:
        return int(servo_id) not in self._inverted_actuator_ids

    def _mapping_range_for_servo(self, servo_id: int) -> tuple[float, float]:
        cfg_idx = self._servo_id_to_config_index(servo_id)
        if cfg_idx is not None and cfg_idx < len(self._mapping_ranges):
            lo, hi = self._mapping_ranges[cfg_idx]
            return float(lo), float(hi)
        return -math.pi, math.pi

    def _raw_to_angle(self, raw_value: int, servo_id: int) -> float:
        lo, hi = self._mapping_range_for_servo(servo_id)
        span = hi - lo
        if abs(span) < 1e-12:
            return lo
        normalized = float(raw_value) / float(self._encoder_resolution)
        if self._is_direct_mapping(servo_id):
            return lo + normalized * span
        return hi - normalized * span

    def _angle_to_raw(self, angle_rad: float, servo_id: int) -> int:
        lo, hi = self._mapping_range_for_servo(servo_id)
        span = hi - lo
        if abs(span) < 1e-12:
            return self._encoder_center
        clamped = max(lo, min(hi, float(angle_rad)))
        normalized = (clamped - lo) / span
        if not self._is_direct_mapping(servo_id):
            normalized = 1.0 - normalized
        raw = int(round(normalized * float(self._encoder_resolution)))
        return max(0, min(self._encoder_resolution, raw))

    def _update_logical_positions_from_raw(self) -> None:
        logical_positions = [0.0] * self._num_joints
        for logical_idx in range(self._num_joints):
            physical_indices = self._logical_to_physical.get(logical_idx, [logical_idx])
            physical_angles: list[float] = []
            for phys_idx in physical_indices:
                if 0 <= int(phys_idx) < len(self._actuator_ids):
                    servo_id = self._actuator_ids[int(phys_idx)]
                    raw = self._raw_positions.get(servo_id, self._encoder_center)
                    physical_angles.append(self._raw_to_angle(raw, servo_id))
            if physical_angles:
                mean_physical = float(sum(physical_angles)) / float(len(physical_angles))
                logical_positions[logical_idx] = mean_physical - self._master_offset_for_joint(logical_idx)
        self._positions = logical_positions

        if self._has_gripper and self._gripper_id is not None:
            raw_gripper = self._raw_positions.get(int(self._gripper_id), self._encoder_center)
            self._gripper_position = self._raw_to_angle(raw_gripper, int(self._gripper_id))

    def _update_raw_from_logical(self) -> None:
        for logical_idx, logical_angle in enumerate(self._positions):
            physical_indices = self._logical_to_physical.get(logical_idx, [logical_idx])
            physical_angle = float(logical_angle) + self._master_offset_for_joint(logical_idx)
            for phys_idx in physical_indices:
                if 0 <= int(phys_idx) < len(self._actuator_ids):
                    servo_id = self._actuator_ids[int(phys_idx)]
                    self._raw_positions[servo_id] = self._angle_to_raw(physical_angle, servo_id)

        if self._has_gripper and self._gripper_id is not None:
            self._raw_positions[int(self._gripper_id)] = self._angle_to_raw(
                self._gripper_position,
                int(self._gripper_id),
            )

