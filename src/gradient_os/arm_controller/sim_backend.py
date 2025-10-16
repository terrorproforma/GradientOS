"""
Lightweight in-process simulator for the Gradient controller.

When activated, this module monkey patches the low-level `servo_driver` and
`servo_protocol` helpers with in-memory implementations so the existing
controller loop can run without physical hardware.  The goal is to provide the
minimal surface area needed for development and testing, while keeping the
overrides modular so a richer simulator can replace them later.
"""
from __future__ import annotations

import threading
from typing import Dict, Iterable, Tuple

import numpy as np

from . import servo_driver, servo_protocol, utils


class _DummySerial:
    """Stub serial handle that satisfies the attributes the controller expects."""

    def __init__(self) -> None:
        self.is_open = True
        self.timeout = 0.1

    def close(self) -> None:
        self.is_open = False

    def reset_input_buffer(self) -> None:  # pragma: no cover - no-op stub
        pass

    def write(self, _data: bytes) -> int:  # pragma: no cover - no-op stub
        return 0

    def read(self, _size: int = 0) -> bytes:  # pragma: no cover - no-op stub
        return b""


class _SimState:
    """Shared state for the simulated servos."""

    def __init__(self) -> None:
        self._lock = threading.RLock()
        self.present_servo_ids = set(utils.SERVO_IDS)
        self.raw_positions: Dict[int, int] = {}
        self.angle_limits_raw: Dict[int, Tuple[int, int]] = {}
        self.registers_word: Dict[Tuple[int, int], int] = {}
        self.registers_byte: Dict[Tuple[int, int], int] = {}
        self.reset()

    def reset(self) -> None:
        """Reset all servo state to nominal defaults."""
        with self._lock:
            self.raw_positions.clear()
            self.registers_word.clear()
            self.registers_byte.clear()
            for index, servo_id in enumerate(utils.SERVO_IDS):
                self.raw_positions[servo_id] = servo_driver.angle_to_raw(0.0, index)
                min_rad, max_rad = utils.URDF_JOINT_LIMITS[index]
                min_raw = servo_driver.angle_to_raw(min_rad, index)
                max_raw = servo_driver.angle_to_raw(max_rad, index)
                self.angle_limits_raw[servo_id] = (min(min_raw, max_raw), max(min_raw, max_raw))
            self._refresh_globals()

    def set_raw(self, servo_id: int, raw_value: int) -> None:
        with self._lock:
            clamped = max(0, min(4095, int(raw_value)))
            self.raw_positions[servo_id] = clamped
            self._refresh_globals()

    def get_raw(self, servo_id: int) -> int:
        with self._lock:
            return self.raw_positions.get(servo_id, 2048)

    def set_angle_limits(self, servo_id: int, min_raw: int, max_raw: int) -> None:
        with self._lock:
            lo = max(0, min(4095, int(min_raw)))
            hi = max(0, min(4095, int(max_raw)))
            self.angle_limits_raw[servo_id] = (min(lo, hi), max(lo, hi))

    def set_register_word(self, servo_id: int, register: int, value: int) -> None:
        with self._lock:
            self.registers_word[(servo_id, register)] = int(value) & 0xFFFF

    def get_register_word(self, servo_id: int, register: int) -> int | None:
        with self._lock:
            return self.registers_word.get((servo_id, register))

    def set_register_byte(self, servo_id: int, register: int, value: int) -> None:
        with self._lock:
            self.registers_byte[(servo_id, register)] = int(value) & 0xFF

    def get_register_byte(self, servo_id: int, register: int) -> int | None:
        with self._lock:
            return self.registers_byte.get((servo_id, register))

    def _refresh_globals(self) -> None:
        """Update the global joint caches to reflect the simulated state."""
        logical_angles = [0.0] * utils.NUM_LOGICAL_JOINTS
        logical_to_physical: Dict[int, Iterable[int]] = {
            0: [10],
            1: [20, 21],
            2: [30, 31],
            3: [40],
            4: [50],
            5: [60],
        }

        for logical_index, servo_ids in logical_to_physical.items():
            angles = []
            for servo_id in servo_ids:
                try:
                    servo_idx = utils.SERVO_IDS.index(servo_id)
                except ValueError:
                    continue
                raw = self.raw_positions.get(servo_id, 2048)
                angles.append(servo_driver.raw_to_angle_rad(raw, servo_idx))
            if angles:
                mean_angle = float(np.mean(angles))
                mean_angle -= utils.LOGICAL_JOINT_MASTER_OFFSETS_RAD[logical_index]
                logical_angles[logical_index] = mean_angle

        utils.current_logical_joint_angles_rad = logical_angles

        # Update gripper cache when configured
        if utils.SERVO_ID_GRIPPER in self.raw_positions:
            try:
                grip_idx = utils.SERVO_IDS.index(utils.SERVO_ID_GRIPPER)
            except ValueError:
                grip_idx = len(utils.SERVO_IDS) - 1
            utils.current_gripper_angle_rad = servo_driver.raw_to_angle_rad(
                self.raw_positions[utils.SERVO_ID_GRIPPER],
                grip_idx,
            )


_SIM_STATE = _SimState()


def _sim_initialize_servos() -> None:
    print("[Sim] Initializing simulated servos...")
    _SIM_STATE.reset()
    utils.ser = _DummySerial()
    utils.gripper_present = True


def _sim_set_servo_angle_limits_from_urdf() -> None:
    print("[Sim] Pretending to set servo angle limits from URDF configuration.")
    for index, servo_id in enumerate(utils.SERVO_IDS):
        min_rad, max_rad = utils.URDF_JOINT_LIMITS[index]
        min_raw = servo_driver.angle_to_raw(min_rad, index)
        max_raw = servo_driver.angle_to_raw(max_rad, index)
        _SIM_STATE.set_angle_limits(servo_id, min_raw, max_raw)


def _sim_sync_write_goal_pos_speed_accel(
    servo_data_list: list[tuple[int, int, int, int]],
) -> None:
    for servo_id, raw_pos, _speed, _accel in servo_data_list:
        _SIM_STATE.set_raw(servo_id, raw_pos)


def _sim_sync_read_positions(servo_ids: Iterable[int]) -> Dict[int, int]:
    return {servo_id: _SIM_STATE.get_raw(servo_id) for servo_id in servo_ids}


def _sim_read_servo_position(servo_id: int) -> int:
    return _SIM_STATE.get_raw(servo_id)


def _sim_write_servo_angle_limits(servo_id: int, min_raw: int, max_raw: int) -> bool:
    _SIM_STATE.set_angle_limits(servo_id, min_raw, max_raw)
    return True


def _sim_write_servo_register_word(servo_id: int, register: int, value: int) -> bool:
    _SIM_STATE.set_register_word(servo_id, register, value)
    return True


def _sim_write_servo_register_byte(servo_id: int, register: int, value: int) -> bool:
    _SIM_STATE.set_register_byte(servo_id, register, value)
    return True


def _sim_read_servo_register_word(servo_id: int, register: int) -> int | None:
    return _SIM_STATE.get_register_word(servo_id, register)


def _sim_read_servo_register_signed_word(servo_id: int, register: int) -> int | None:
    value = _SIM_STATE.get_register_word(servo_id, register)
    if value is None:
        return None
    if value > 0x7FFF:
        value -= 0x10000
    return value


def _sim_calibrate_servo_middle_position(servo_id: int) -> bool:
    index = utils.SERVO_IDS.index(servo_id) if servo_id in utils.SERVO_IDS else 0
    _SIM_STATE.set_raw(servo_id, servo_driver.angle_to_raw(0.0, index))
    return True


def _sim_factory_reset_servo(servo_id: int) -> bool:
    print(f"[Sim] Factory reset servo {servo_id}")
    _sim_calibrate_servo_middle_position(servo_id)
    return True


def _sim_restart_servo(servo_id: int) -> bool:
    print(f"[Sim] Restart servo {servo_id}")
    return True


def _sim_ping(servo_id: int) -> bool:
    _SIM_STATE.present_servo_ids.add(servo_id)
    return True


def activate() -> None:
    """
    Enable simulator mode by patching servo helpers with in-memory versions.
    Safe to call multiple times; only the first invocation applies overrides.
    """
    if getattr(activate, "_activated", False):
        return

    print("[Sim] Activating Gradient controller simulator backend.")

    # Patch servo_driver high-level entrypoints
    servo_driver.initialize_servos = _sim_initialize_servos  # type: ignore[assignment]
    servo_driver.set_servo_angle_limits_from_urdf = _sim_set_servo_angle_limits_from_urdf  # type: ignore[assignment]

    # Patch servo_protocol low-level operations used by the controller stack
    servo_protocol._present_servo_ids = _SIM_STATE.present_servo_ids  # type: ignore[attr-defined]
    servo_protocol.get_present_servo_ids = lambda: _SIM_STATE.present_servo_ids  # type: ignore[assignment]
    servo_protocol.ping = _sim_ping  # type: ignore[assignment]
    servo_protocol.sync_write_goal_pos_speed_accel = _sim_sync_write_goal_pos_speed_accel  # type: ignore[assignment]
    servo_protocol.sync_read_positions = _sim_sync_read_positions  # type: ignore[assignment]
    servo_protocol.read_servo_position = _sim_read_servo_position  # type: ignore[assignment]
    servo_protocol.write_servo_angle_limits = _sim_write_servo_angle_limits  # type: ignore[assignment]
    servo_protocol.write_servo_register_word = _sim_write_servo_register_word  # type: ignore[assignment]
    servo_protocol.write_servo_register_byte = _sim_write_servo_register_byte  # type: ignore[assignment]
    servo_protocol.read_servo_register_word = _sim_read_servo_register_word  # type: ignore[assignment]
    servo_protocol.read_servo_register_signed_word = _sim_read_servo_register_signed_word  # type: ignore[assignment]
    servo_protocol.calibrate_servo_middle_position = _sim_calibrate_servo_middle_position  # type: ignore[assignment]
    servo_protocol.factory_reset_servo = _sim_factory_reset_servo  # type: ignore[assignment]
    servo_protocol.restart_servo = _sim_restart_servo  # type: ignore[assignment]

    activate._activated = True

