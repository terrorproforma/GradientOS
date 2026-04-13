from __future__ import annotations

import array
import json
import mmap
import os
import select
import socket
import struct
import threading
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Optional

from ....absolute_encoder_anchors import (
    load_absolute_encoder_anchors,
    save_absolute_encoder_anchor,
)
from ...actuator_interface import ActuatorBackend
from ...profiles import registry as drive_profile_registry
from .runtime import (
    RTCORE_EXEC_STATE_ABORTED,
    RTCORE_EXEC_STATE_COMPLETED,
    RTCORE_EXEC_STATE_FAULTED,
    RTCORE_EXEC_STATE_IDLE,
    RTCORE_JOG_STOP_REASON_NONE,
    RTCORE_MOTION_CAP_JOG_COMMAND,
    RTCORE_EXEC_STATE_UNDERRUN,
    RTCORE_MOTION_MODE_IDLE,
    rtcore_drive_profile_id_to_name,
    rtcore_execution_state_id_to_name,
    rtcore_jog_stop_reason_id_to_name,
    rtcore_motion_mode_id_to_name,
)
from ....joint_zero_offsets import load_joint_zero_offsets, save_joint_zero_offsets


def _fourcc(a: str, b: str, c: str, d: str) -> int:
    return (
        (ord(a) & 0xFF) << 0
        | (ord(b) & 0xFF) << 8
        | (ord(c) & 0xFF) << 16
        | (ord(d) & 0xFF) << 24
    )


_MAGIC_GIPC = _fourcc("G", "I", "P", "C")
_MAGIC_GSHM = _fourcc("G", "S", "H", "M")
_MAGIC_RING = _fourcc("R", "I", "N", "G")

_VER_MAJOR = 1
_VER_MINOR = 0

_ROLE_CONTROLLER = 1

_GRADIENT_MAX_AXES = 16

_MSG_STATUS_HELLO = 0x0201
_MSG_STATUS_SNAPSHOT = 0x0202
_MSG_STATUS_AXIS_CONFIG = 0x0203
_MSG_STATUS_MOTION_STATE = 0x0204
_MSG_STATUS_JOG_DEBUG = 0x0205

# Command ring message types (v1)
_MSG_CMD_ARM = 0x0101
_MSG_CMD_AXIS_ENABLE = 0x0102
_MSG_CMD_AXIS_DISABLE = 0x0103
_MSG_CMD_FAULT_RESET = 0x0104
_MSG_CMD_SET_MODE = 0x0106
_MSG_CMD_NATIVE_HOME = 0x0108
_MSG_CMD_SERVICE_SDO_WRITE = 0x0109
_MSG_CMD_TRAJECTORY_BEGIN = 0x0120
_MSG_CMD_TRAJECTORY_POINT = 0x0121
_MSG_CMD_TRAJECTORY_COMMIT = 0x0122
_MSG_CMD_TRAJECTORY_ABORT = 0x0123
_MSG_CMD_JOG = 0x0130

_MODE_CSP = 8
_TRAJ_POINTF_HAS_VELOCITY = 1 << 0
_TRAJ_POINTF_LAST_POINT = 1 << 1
_JOG_FLAG_ACTIVE = 1 << 0
_JOG_FLAG_STOP = 1 << 1
_JOG_FLAG_QUICK_STOP = 1 << 2
_SERVICE_SDO_VALUE_U16 = 1

_POWER_TRANSITION_WAIT_POLL_INTERVAL_S = 0.01
_POWER_TRANSITION_DEFAULT_TIMEOUT_S = 1.0
_NATIVE_HOME_WAIT_POLL_INTERVAL_S = 0.05
_NATIVE_HOME_WAIT_TIMEOUT_S = 20.0
_RTCORE_METRICS_PATH = Path("/run/gradient-rt-motion/metrics.json")

_HELLO_STRUCT = struct.Struct("<IHHIIQ4Q")  # 56 bytes
_WELCOME_STRUCT = struct.Struct("<IHHI II 4x QQ IIII Q 4Q")  # 96 bytes (includes padding after reserved0)

_SHM_HEADER_STRUCT = struct.Struct("<IHHIIIIQQIIIII4x8Q")  # 128 bytes (includes padding before reserved2)
_RING_HEADER_STRUCT = struct.Struct("<7I")  # 28 bytes
_MSG_HEADER_STRUCT = struct.Struct("<HHIQQ")  # 24 bytes

_CMD_ARM_STRUCT = struct.Struct("<II")
_CMD_AXIS_MASK_STRUCT = struct.Struct("<II")
_CMD_SERVICE_SDO_WRITE_STRUCT = struct.Struct("<I H B B I I")
_CMD_SET_MODE_STRUCT = struct.Struct("<II")
_CMD_TRAJECTORY_BEGIN_STRUCT = struct.Struct("<QIIII")
_TRAJECTORY_POINT_STRUCT = struct.Struct("<QIIQ16d16dII")
_CMD_TRAJECTORY_CONTROL_STRUCT = struct.Struct("<QII")
_CMD_JOG_STRUCT = struct.Struct("<IIQ16d")
_STATUS_HELLO_STRUCT = struct.Struct("<QQQIIII")
_AXIS_CONFIG_STRUCT = struct.Struct("<II16I16d16i16B16x16d")  # 424 bytes
_STATUS_SNAPSHOT_HEADER_STRUCT = struct.Struct("<IIIIqqQ")
_STATUS_MOTION_STATE_STRUCT = struct.Struct("<IIQIIIIIIIIQQ")  # 64 bytes
_STATUS_JOG_DEBUG_STRUCT = struct.Struct("<12I7Q16i16i16i16i")  # 360 bytes

_TRAJECTORY_WAIT_SETTLE_MARGIN_S = 5.0
_CMD_RING_WRITE_WAIT_S = 0.5
_CMD_RING_WRITE_RETRY_S = 0.001


def _align_up(value: int, alignment: int) -> int:
    return ((value + alignment - 1) // alignment) * alignment


def _now_monotonic_ns() -> int:
    # Use monotonic clock to match RTCore.
    return time.monotonic_ns()


def _native_home_state_name(value: object) -> str:
    try:
        state = int(value)
    except Exception:
        state = 0
    labels = {
        0: "idle",
        1: "requested",
        2: "succeeded",
        3: "failed",
    }
    return labels.get(state, f"unknown:{state}")


def _estimate_joint_path_velocities(
    joint_path: list[list[float]],
    *,
    step_s: float,
) -> list[list[float]]:
    """Estimate per-sample joint velocities for scheduled RTCore motion."""
    if step_s <= 0.0:
        raise ValueError("step_s must be positive")
    if not joint_path:
        return []

    point_count = len(joint_path)
    if point_count == 1:
        return [[0.0] * len(joint_path[0])]

    velocities: list[list[float]] = []
    for idx, point in enumerate(joint_path):
        joint_count = len(point)
        if idx == 0:
            prev_point = point
            next_point = joint_path[idx + 1]
            denom_s = step_s
        elif idx == point_count - 1:
            prev_point = joint_path[idx - 1]
            next_point = point
            denom_s = step_s
        else:
            prev_point = joint_path[idx - 1]
            next_point = joint_path[idx + 1]
            denom_s = step_s * 2.0

        if len(prev_point) != joint_count or len(next_point) != joint_count:
            raise ValueError("All joint trajectory points must have the same length")

        velocities.append(
            [
                (float(next_point[joint_i]) - float(prev_point[joint_i])) / denom_s
                for joint_i in range(joint_count)
            ]
        )
    return velocities


@dataclass(frozen=True)
class _ShmHeader:
    kind: int
    num_axes: int
    cycle_ns: int
    topology_hash: int
    ring_offset: int
    ring_capacity: int
    ring_msg_bytes: int
    setpoint_offset: int


@dataclass(frozen=True)
class _AxisConfig:
    num_axes: int
    counts_per_unit: list[float]
    sign: list[int]
    counts_per_rev: list[int] = field(default_factory=lambda: [0] * _GRADIENT_MAX_AXES)


@dataclass(frozen=True)
class _AbsoluteFeedbackField:
    valid: bool = False
    value: int = 0

    def to_dict(self) -> dict[str, object]:
        return {
            "valid": bool(self.valid),
            "value": int(self.value),
        }


def _absolute_feedback_field_from_mapping(raw: object) -> _AbsoluteFeedbackField:
    if not isinstance(raw, dict):
        return _AbsoluteFeedbackField()
    try:
        valid = bool(int(raw.get("valid", 0)))
    except Exception:
        valid = False
    try:
        value = int(raw.get("value", 0))
    except Exception:
        value = 0
    return _AbsoluteFeedbackField(valid=valid, value=value)


@dataclass(frozen=True)
class _AbsoluteFeedbackAxisMetrics:
    fields: dict[str, _AbsoluteFeedbackField] = field(default_factory=dict)

    @classmethod
    def from_mapping(cls, raw: object) -> "_AbsoluteFeedbackAxisMetrics":
        if not isinstance(raw, dict):
            return cls()
        fields: dict[str, _AbsoluteFeedbackField] = {}
        for key, value in raw.items():
            if not isinstance(key, str):
                continue
            fields[str(key)] = _absolute_feedback_field_from_mapping(value)
        return cls(fields=fields)

    def to_dict(self) -> dict[str, object]:
        return {
            str(key): field.to_dict()
            for key, field in self.fields.items()
        }

    def has_any_valid(self) -> bool:
        return any(field.valid for field in self.fields.values())


@dataclass(frozen=True)
class RTCoreExecutionStatus:
    active_mode: int
    active_mode_name: str
    state: int
    state_name: str
    active_traj_id: int
    current_point_index: Optional[int]
    queue_depth: int
    queue_capacity: int
    last_event_code: int
    underrun_count: int
    stale_command: bool
    motion_done: bool
    capability_flags: int
    active_command_seq: int
    last_update_ns: int


@dataclass(frozen=True)
class RTCoreJogDebugStatus:
    num_axes: int
    active_jog: bool
    active_jog_axis_mask: int
    command_sp_mask: int
    have_hold_mask: int
    have_jog_target_mask: int
    snap_hold_mask: int
    stop_arrest_mask: int
    latest_cmd_axis_mask: int
    latest_cmd_flags: int
    latest_cmd_timeout_ns: int
    sample_time_ns: int
    active_jog_cmd_seq: int
    latest_jog_seq_seen: int
    active_jog_deadline_ns: int
    last_stop_reason: int
    last_stop_reason_name: str
    last_stop_axis_mask: int
    last_stop_time_ns: int
    last_stop_cmd_seq: int
    feedback_pos_counts: list[int]
    hold_target_counts: list[int]
    output_target_counts: list[int]
    output_target_velocity_counts_per_s: list[int]


class EthercatRTCoreBackend(ActuatorBackend):
    """
    ActuatorBackend proxy to the RTCore daemon (`gradient-rt-motion`).

    This class intentionally does *not* perform any EtherCAT I/O itself.
    It only performs:
    - IPC handshake (UDS + SCM_RIGHTS)
    - trajectory command-ring writes
    - status ring reads (best-effort, non-RT)
    """

    def __init__(
        self,
        robot_config: dict,
        socket_path: str = "/run/gradient-rt-motion/ipc.sock",
    ) -> None:
        self._robot_config = robot_config
        self._socket_path = socket_path

        self._robot_id = str(robot_config.get("robot_id", "unknown")).strip() or "unknown"
        self._num_joints = int(robot_config.get("num_logical_joints", 6))
        self._master_offsets_rad = load_joint_zero_offsets(
            self._robot_id,
            num_joints=self._num_joints,
            defaults=robot_config.get("logical_joint_master_offsets_rad", [0.0] * self._num_joints),
        )

        self._initialized = False
        self._connected = False
        self._rt_num_axes: int = 0

        # Mapping: RTCore axis index -> GradientOS logical joint index (0-based).
        # Default policy is direct ordering axis0->J1, axis1->J2, ... up to min(num_axes, num_joints).
        # For hardware bring-up/custom wiring, override via GRADIENT_RTCORE_CONTROL_JOINTS.
        self._axis_to_joint: list[int] = []

        # Command ring sequencing (producer-owned).
        self._cmd_seq = 1
        self._next_traj_id = 1
        self._last_submitted_traj_id = 0
        self._last_trajectory_timing: dict[str, int] = {}

        # Auto-arm on successful IPC connect (mirrors how serial servos become usable after init).
        self._auto_arm = os.environ.get("GRADIENT_RTCORE_AUTO_ARM", "0").lower() not in ("0", "false", "no")
        # Optional: restrict which axes are armed/enabled on connect.
        # Examples:
        #   GRADIENT_RTCORE_AUTO_ARM_MASK=0x1   (only axis 0, i.e. slave position 0)
        #   GRADIENT_RTCORE_AUTO_ARM_MASK=0x3   (axes 0 and 1)
        raw_mask = os.environ.get("GRADIENT_RTCORE_AUTO_ARM_MASK", "").strip()
        self._auto_arm_mask: Optional[int] = None
        if raw_mask:
            try:
                # base=0 accepts 0x.. hex or decimal.
                self._auto_arm_mask = int(raw_mask, 0)
            except ValueError:
                print(f"[EtherCAT RTCore] WARNING: invalid GRADIENT_RTCORE_AUTO_ARM_MASK='{raw_mask}'")

        self._sock: Optional[socket.socket] = None

        self._cmd_shm_fd: Optional[int] = None
        self._status_shm_fd: Optional[int] = None
        self._cmd_eventfd: Optional[int] = None
        self._status_eventfd: Optional[int] = None

        self._cmd_shm: Optional[mmap.mmap] = None
        self._status_shm: Optional[mmap.mmap] = None

        self._cmd_hdr: Optional[_ShmHeader] = None
        self._status_hdr: Optional[_ShmHeader] = None
        self._robot_axis_config = self._build_axis_config_from_robot_config(robot_config)
        self._runtime_axis_config: Optional[_AxisConfig] = None
        self._axis_config: Optional[_AxisConfig] = self._robot_axis_config
        self._axis_config_mismatch_logged = False
        self._axis_config_event = threading.Event()
        self._status_snapshot_event = threading.Event()
        self._status_lock = threading.Lock()
        if self._axis_config is not None:
            self._axis_config_event.set()

        # Latest known axis counts from STATUS_SNAPSHOT (pos_counts per axis).
        self._axis_counts: list[int] = [0] * _GRADIENT_MAX_AXES
        self._axis_torque_raw: list[int] = [0] * _GRADIENT_MAX_AXES
        self._axis_statusword: list[int] = [0] * _GRADIENT_MAX_AXES
        self._axis_error_code: list[int] = [0] * _GRADIENT_MAX_AXES
        self._axis_manufacturer_error_code: list[int] = [0] * _GRADIENT_MAX_AXES
        self._axis_mode_display: list[int] = [0] * _GRADIENT_MAX_AXES
        self._axis_ds402_state: list[int] = [0] * _GRADIENT_MAX_AXES
        self._axis_di_bits: list[int] = [0] * _GRADIENT_MAX_AXES
        self._axis_fault_flags: list[int] = [0] * _GRADIENT_MAX_AXES
        self._axis_brake_state: list[int] = [0] * _GRADIENT_MAX_AXES
        self._native_home_offset_counts: list[int] = [0] * _GRADIENT_MAX_AXES
        self._absolute_feedback_by_axis: list[_AbsoluteFeedbackAxisMetrics] = [
            _AbsoluteFeedbackAxisMetrics() for _ in range(_GRADIENT_MAX_AXES)
        ]
        self._absolute_encoder_home_anchors: list[dict[str, Any] | None] = load_absolute_encoder_anchors(
            self._robot_id,
            num_joints=self._num_joints,
        )
        self._native_home_metrics_mtime_ns = -1
        self._feedback_unwrapped_counts: list[int] = [0] * _GRADIENT_MAX_AXES
        self._feedback_unwrapped_valid: list[bool] = [False] * _GRADIENT_MAX_AXES
        self._rt_drive_profile_code = 0
        self._rt_drive_profile_id: Optional[str] = None
        self._last_wkc_expected = 0
        self._last_wkc_actual = 0
        self._last_master_state = 0
        self._last_status_snapshot_monotonic_s = 0.0
        self._execution_status = RTCoreExecutionStatus(
            active_mode=RTCORE_MOTION_MODE_IDLE,
            active_mode_name=rtcore_motion_mode_id_to_name(RTCORE_MOTION_MODE_IDLE) or "idle",
            state=RTCORE_EXEC_STATE_IDLE,
            state_name=rtcore_execution_state_id_to_name(RTCORE_EXEC_STATE_IDLE) or "idle",
            active_traj_id=0,
            current_point_index=None,
            queue_depth=0,
            queue_capacity=0,
            last_event_code=0,
            underrun_count=0,
            stale_command=False,
            motion_done=True,
            capability_flags=0,
            active_command_seq=0,
            last_update_ns=0,
        )
        self._jog_debug_status = RTCoreJogDebugStatus(
            num_axes=0,
            active_jog=False,
            active_jog_axis_mask=0,
            command_sp_mask=0,
            have_hold_mask=0,
            have_jog_target_mask=0,
            snap_hold_mask=0,
            stop_arrest_mask=0,
            latest_cmd_axis_mask=0,
            latest_cmd_flags=0,
            latest_cmd_timeout_ns=0,
            sample_time_ns=0,
            active_jog_cmd_seq=0,
            latest_jog_seq_seen=0,
            active_jog_deadline_ns=0,
            last_stop_reason=RTCORE_JOG_STOP_REASON_NONE,
            last_stop_reason_name=rtcore_jog_stop_reason_id_to_name(RTCORE_JOG_STOP_REASON_NONE) or "none",
            last_stop_axis_mask=0,
            last_stop_time_ns=0,
            last_stop_cmd_seq=0,
            feedback_pos_counts=[0] * _GRADIENT_MAX_AXES,
            hold_target_counts=[0] * _GRADIENT_MAX_AXES,
            output_target_counts=[0] * _GRADIENT_MAX_AXES,
            output_target_velocity_counts_per_s=[0] * _GRADIENT_MAX_AXES,
        )

        # Latest commanded joint positions (radians) as a safe fallback for getters.
        self._last_joint_setpoint_rad: list[float] = [0.0] * self._num_joints

        self._status_thread: Optional[threading.Thread] = None
        self._status_stop = threading.Event()

    # -------------------------------------------------------------------------
    # ActuatorBackend required API
    # -------------------------------------------------------------------------

    def initialize(self) -> bool:
        try:
            ok = self._connect_ipc()
            self._initialized = bool(ok)
            return bool(ok)
        except Exception as e:
            # Keep the controller alive, but report as not connected.
            print(f"[EtherCAT RTCore] WARNING: IPC init failed: {e}")
            self._connected = False
            self._initialized = False
            return False

    def shutdown(self) -> None:
        self._best_effort_safe_power_down()
        self._status_stop.set()
        self._axis_config_event.clear()
        self._status_snapshot_event.clear()
        if self._status_thread and self._status_thread.is_alive():
            self._status_thread.join(timeout=1.0)

        self._status_thread = None

        if self._cmd_shm is not None:
            try:
                self._cmd_shm.close()
            except Exception:
                pass
        if self._status_shm is not None:
            try:
                self._status_shm.close()
            except Exception:
                pass

        self._cmd_shm = None
        self._status_shm = None

        for fd_name in ("_cmd_shm_fd", "_status_shm_fd", "_cmd_eventfd", "_status_eventfd"):
            fd = getattr(self, fd_name)
            if isinstance(fd, int):
                try:
                    os.close(fd)
                except Exception:
                    pass
            setattr(self, fd_name, None)

        if self._sock is not None:
            try:
                self._sock.close()
            except Exception:
                pass
        self._sock = None

        self._connected = False
        self._initialized = False
        with self._status_lock:
            self._rt_drive_profile_code = 0
            self._rt_drive_profile_id = None
            self._feedback_unwrapped_counts = [0] * _GRADIENT_MAX_AXES
            self._feedback_unwrapped_valid = [False] * _GRADIENT_MAX_AXES
            self._last_submitted_traj_id = 0
            self._execution_status = RTCoreExecutionStatus(
                active_mode=RTCORE_MOTION_MODE_IDLE,
                active_mode_name=rtcore_motion_mode_id_to_name(RTCORE_MOTION_MODE_IDLE) or "idle",
                state=RTCORE_EXEC_STATE_IDLE,
                state_name=rtcore_execution_state_id_to_name(RTCORE_EXEC_STATE_IDLE) or "idle",
                active_traj_id=0,
                current_point_index=None,
                queue_depth=0,
                queue_capacity=0,
                last_event_code=0,
                underrun_count=0,
                stale_command=False,
                motion_done=True,
                capability_flags=0,
                active_command_seq=0,
                last_update_ns=0,
            )
            self._jog_debug_status = RTCoreJogDebugStatus(
                num_axes=0,
                active_jog=False,
                active_jog_axis_mask=0,
                command_sp_mask=0,
                have_hold_mask=0,
                have_jog_target_mask=0,
                snap_hold_mask=0,
                stop_arrest_mask=0,
                latest_cmd_axis_mask=0,
                latest_cmd_flags=0,
                latest_cmd_timeout_ns=0,
                sample_time_ns=0,
                active_jog_cmd_seq=0,
                latest_jog_seq_seen=0,
                active_jog_deadline_ns=0,
                last_stop_reason=RTCORE_JOG_STOP_REASON_NONE,
                last_stop_reason_name=rtcore_jog_stop_reason_id_to_name(RTCORE_JOG_STOP_REASON_NONE) or "none",
                last_stop_axis_mask=0,
                last_stop_time_ns=0,
                last_stop_cmd_seq=0,
                feedback_pos_counts=[0] * _GRADIENT_MAX_AXES,
                hold_target_counts=[0] * _GRADIENT_MAX_AXES,
                output_target_counts=[0] * _GRADIENT_MAX_AXES,
                output_target_velocity_counts_per_s=[0] * _GRADIENT_MAX_AXES,
            )

    def safe_power_down(self, *, wait_for_idle: bool = False, timeout_s: float | None = None) -> bool:
        self._best_effort_safe_power_down(wait_for_idle=wait_for_idle, timeout_s=timeout_s)
        return True

    def safe_power_up(self) -> bool:
        self._best_effort_safe_power_up()
        return True

    def get_live_drive_profile_id(self) -> Optional[str]:
        with self._status_lock:
            return self._rt_drive_profile_id

    def get_execution_status(self) -> RTCoreExecutionStatus:
        with self._status_lock:
            return self._execution_status

    def get_jog_debug_status(self) -> RTCoreJogDebugStatus:
        with self._status_lock:
            return self._jog_debug_status

    def get_last_submitted_trajectory_id(self) -> int:
        with self._status_lock:
            return int(self._last_submitted_traj_id)

    def get_last_trajectory_timing(self) -> dict[str, int]:
        with self._status_lock:
            return dict(self._last_trajectory_timing)

    def _all_axis_mask(self) -> int:
        return (1 << self._rt_num_axes) - 1 if self._rt_num_axes > 0 else 0

    def get_power_transition_snapshot(self) -> dict[str, Any]:
        with self._status_lock:
            execution_status = self._execution_status
            jog_status = self._jog_debug_status
            axis_error_code = list(self._axis_error_code[: self._rt_num_axes])
            axis_fault_flags = list(self._axis_fault_flags[: self._rt_num_axes])
            axis_counts = list(self._axis_counts[: self._rt_num_axes])
            axis_config = self._axis_config
            feedback_ready = self._status_snapshot_event.is_set()

        terminal_states = {"idle", "completed", "aborted", "faulted", "underrun"}
        active_mode_name = str(getattr(execution_status, "active_mode_name", "idle") or "idle").strip().lower() or "idle"
        state_name = str(getattr(execution_status, "state_name", "idle") or "idle").strip().lower() or "idle"
        active_traj_id = int(getattr(execution_status, "active_traj_id", 0) or 0)
        queue_depth = int(getattr(execution_status, "queue_depth", 0) or 0)
        stale_command = bool(getattr(execution_status, "stale_command", False))
        motion_done = bool(getattr(execution_status, "motion_done", False))
        active_jog = bool(getattr(jog_status, "active_jog", False))
        faulted_axis_indices = [
            axis_i
            for axis_i, (error_code, fault_flag) in enumerate(zip(axis_error_code, axis_fault_flags, strict=False))
            if int(error_code) != 0 or int(fault_flag) != 0
        ]

        live_feedback_joint_positions: list[float] = []
        if axis_config is not None and self._connected and self._rt_num_axes > 0:
            try:
                live_feedback_joint_positions = self.raw_to_joint_positions(
                    {axis_i: axis_counts[axis_i] for axis_i in range(min(len(axis_counts), self._rt_num_axes))}
                )
            except Exception:
                live_feedback_joint_positions = []

        motion_active = False
        if active_traj_id > 0 or queue_depth > 0 or active_jog:
            motion_active = True
        elif state_name in {"accepted", "queued", "executing"}:
            motion_active = True
        elif active_mode_name != "idle" and not motion_done and state_name not in terminal_states:
            motion_active = True

        feedback_synchronized = (
            feedback_ready
            and len(live_feedback_joint_positions) == self._num_joints
        )

        return {
            "connected": bool(self._connected),
            "feedback_ready": bool(feedback_ready),
            "feedback_synchronized": bool(feedback_synchronized),
            "live_feedback_joint_positions_rad": list(live_feedback_joint_positions),
            "active_mode_name": active_mode_name,
            "state_name": state_name,
            "active_traj_id": active_traj_id,
            "queue_depth": queue_depth,
            "stale_command": bool(stale_command),
            "motion_done": bool(motion_done),
            "active_jog": bool(active_jog),
            "active_jog_axis_mask": int(getattr(jog_status, "active_jog_axis_mask", 0) or 0),
            "faulted_axis_count": len(faulted_axis_indices),
            "faulted_axis_indices": faulted_axis_indices,
            "motion_active": bool(motion_active),
            "motion_intent_cleared": not motion_active,
            "power_up_ready": bool(not motion_active and not stale_command and not faulted_axis_indices and feedback_synchronized),
        }

    def wait_for_power_transition_neutral(self, *, timeout_s: float | None = None) -> dict[str, Any]:
        latest = self.get_power_transition_snapshot()
        resolved_timeout_s = (
            _POWER_TRANSITION_DEFAULT_TIMEOUT_S
            if timeout_s is None
            else max(0.0, float(timeout_s))
        )
        if resolved_timeout_s <= 0.0 or bool(latest.get("motion_intent_cleared", False)):
            latest["wait_timed_out"] = False
            return latest

        deadline = time.monotonic() + resolved_timeout_s
        while time.monotonic() <= deadline:
            time.sleep(_POWER_TRANSITION_WAIT_POLL_INTERVAL_S)
            latest = self.get_power_transition_snapshot()
            if bool(latest.get("motion_intent_cleared", False)):
                latest["wait_timed_out"] = False
                return latest

        latest["wait_timed_out"] = True
        return latest

    def prepare_for_power_transition(
        self,
        *,
        wait_for_idle: bool = False,
        timeout_s: float | None = None,
        quick_stop: bool = True,
    ) -> dict[str, Any]:
        if not self._connected:
            snapshot = self.get_power_transition_snapshot()
            snapshot["waited_for_idle"] = bool(wait_for_idle)
            snapshot["wait_timed_out"] = False
            return snapshot

        try:
            self.abort_trajectory()
        except Exception as exc:
            print(f"[EtherCAT RTCore] WARNING: trajectory abort before power transition failed: {exc}")
        try:
            self.stop_joint_velocity_lease_jog(quick_stop=quick_stop)
        except Exception as exc:
            print(f"[EtherCAT RTCore] WARNING: jog stop before power transition failed: {exc}")

        if wait_for_idle:
            snapshot = self.wait_for_power_transition_neutral(timeout_s=timeout_s)
        else:
            time.sleep(0.02)
            snapshot = self.get_power_transition_snapshot()
            snapshot["wait_timed_out"] = False
        snapshot["waited_for_idle"] = bool(wait_for_idle)
        return snapshot

    def synchronize_command_targets_to_feedback(self) -> dict[str, Any]:
        snapshot = self.get_power_transition_snapshot()
        joint_positions = snapshot.get("live_feedback_joint_positions_rad")
        if not isinstance(joint_positions, list) or len(joint_positions) != self._num_joints:
            return {
                "synchronized": False,
                "reason": "feedback_unavailable",
                "joint_positions_rad": [],
            }
        with self._status_lock:
            self._last_joint_setpoint_rad = [float(value) for value in joint_positions]
        return {
            "synchronized": True,
            "reason": "ok",
            "joint_positions_rad": [float(value) for value in joint_positions],
        }

    def get_rtcore_cycle_ns(self) -> int:
        hdr = self._cmd_hdr or self._status_hdr
        cycle_ns = int(hdr.cycle_ns) if hdr is not None else 0
        return cycle_ns if cycle_ns > 0 else 1_000_000

    def resolve_trajectory_frequency(self, requested_hz: int) -> dict[str, int]:
        requested_hz = max(1, int(requested_hz))
        cycle_ns = max(1, int(self.get_rtcore_cycle_ns()))
        requested_step_ns = max(1, (1_000_000_000 + requested_hz - 1) // requested_hz)
        cycles_per_point = max(1, (requested_step_ns + cycle_ns - 1) // cycle_ns)
        step_ns = cycles_per_point * cycle_ns
        effective_hz = max(1, 1_000_000_000 // step_ns)
        return {
            "requested_frequency_hz": requested_hz,
            "effective_frequency_hz": int(effective_hz),
            "cycle_ns": cycle_ns,
            "step_ns": int(step_ns),
            "cycles_per_point": int(cycles_per_point),
        }

    def begin_trajectory(
        self,
        *,
        traj_id: Optional[int] = None,
        axis_mask: Optional[int] = None,
        expected_points: int = 0,
    ) -> int:
        if not self._connected:
            raise RuntimeError("RTCore not connected (cannot begin trajectory)")
        if self._rt_num_axes <= 0:
            raise RuntimeError("RTCore did not report a valid num_axes")
        if traj_id is None:
            traj_id = self._allocate_traj_id()
        resolved_axis_mask = (
            int(axis_mask)
            if axis_mask is not None
            else ((1 << self._rt_num_axes) - 1)
        )
        self._cmd_ring_write(
            _MSG_CMD_TRAJECTORY_BEGIN,
            _CMD_TRAJECTORY_BEGIN_STRUCT.pack(
                int(traj_id),
                int(resolved_axis_mask),
                0,
                max(0, int(expected_points)),
                0,
            ),
        )
        return int(traj_id)

    def enqueue_trajectory_points(self, traj_id: int, points: list[dict[str, object]]) -> None:
        if not self._connected:
            raise RuntimeError("RTCore not connected (cannot enqueue trajectory points)")
        if self._rt_num_axes <= 0:
            raise RuntimeError("RTCore did not report a valid num_axes")
        total = len(points)
        for idx, point in enumerate(points):
            t_from_start_ns = int(point.get("t_from_start_ns", 0))
            axis_mask = int(point.get("axis_mask", (1 << self._rt_num_axes) - 1))
            flags = int(point.get("flags", 0))
            if idx == total - 1:
                flags |= _TRAJ_POINTF_LAST_POINT

            axis_q_obj = point.get("axis_q")
            if axis_q_obj is not None:
                axis_q = [float(v) for v in list(axis_q_obj)]
            else:
                positions_obj = point.get("positions_rad")
                if positions_obj is None:
                    raise ValueError("Trajectory point requires positions_rad or axis_q")
                axis_q = self._axis_q_from_joint_positions([float(v) for v in list(positions_obj)])
            if len(axis_q) != self._rt_num_axes:
                raise ValueError(
                    f"Expected {self._rt_num_axes} RT axis values, got {len(axis_q)}"
                )

            qd = point.get("qd")
            qd_values = [0.0] * _GRADIENT_MAX_AXES
            if qd is not None:
                qd_list = [float(v) for v in list(qd)]
                for vel_i, vel in enumerate(qd_list[:_GRADIENT_MAX_AXES]):
                    qd_values[vel_i] = vel

            q_values = [0.0] * _GRADIENT_MAX_AXES
            for axis_i, value in enumerate(axis_q[:_GRADIENT_MAX_AXES]):
                q_values[axis_i] = float(value)

            self._cmd_ring_write(
                _MSG_CMD_TRAJECTORY_POINT,
                _TRAJECTORY_POINT_STRUCT.pack(
                    int(traj_id),
                    idx,
                    int(flags),
                    max(0, t_from_start_ns),
                    *q_values,
                    *qd_values,
                    int(axis_mask),
                    0,
                ),
            )

    def commit_trajectory(self, traj_id: int) -> int:
        cmd_seq = self._cmd_ring_write(
            _MSG_CMD_TRAJECTORY_COMMIT,
            _CMD_TRAJECTORY_CONTROL_STRUCT.pack(int(traj_id), 0, 0),
        )
        with self._status_lock:
            self._last_submitted_traj_id = int(traj_id)
        return int(cmd_seq)

    def abort_trajectory(self, traj_id: Optional[int] = None) -> None:
        self._cmd_ring_write(
            _MSG_CMD_TRAJECTORY_ABORT,
            _CMD_TRAJECTORY_CONTROL_STRUCT.pack(0 if traj_id is None else int(traj_id), 0, 0),
        )

    def execute_joint_trajectory(
        self,
        joint_path: list[list[float]],
        frequency: int,
        *,
        timeout_s: Optional[float] = None,
    ) -> RTCoreExecutionStatus:
        if joint_path is None or len(joint_path) == 0:
            raise ValueError("joint_path must not be empty")
        timing = self.resolve_trajectory_frequency(frequency)
        requested_frequency_hz = int(timing["requested_frequency_hz"])
        frequency_hz = int(timing["effective_frequency_hz"])
        step_ns = int(timing["step_ns"])
        with self._status_lock:
            self._last_trajectory_timing = dict(timing)
        if frequency_hz != requested_frequency_hz:
            print(
                "[EtherCAT RTCore] Quantized trajectory frequency:"
                f" requested={requested_frequency_hz}Hz"
                f" effective={frequency_hz}Hz"
                f" cycle_ns={int(timing['cycle_ns'])}"
                f" cycles_per_point={int(timing['cycles_per_point'])}"
            )
        step_s = step_ns / 1e9
        joint_velocities = _estimate_joint_path_velocities(joint_path, step_s=step_s)
        traj_id = self.begin_trajectory(expected_points=len(joint_path))
        points = []
        for idx, (q, qd) in enumerate(zip(joint_path, joint_velocities, strict=True)):
            points.append(
                {
                    "positions_rad": list(q),
                    "qd": self._axis_qd_from_joint_velocities(list(qd)),
                    "flags": _TRAJ_POINTF_HAS_VELOCITY,
                    "t_from_start_ns": idx * step_ns,
                }
            )
        self.enqueue_trajectory_points(traj_id, points)
        submitted_command_seq = self.commit_trajectory(traj_id)

        wait_timeout_s = timeout_s
        if wait_timeout_s is None:
            duration_s = len(joint_path) / float(frequency_hz)
            wait_timeout_s = max(
                _TRAJECTORY_WAIT_SETTLE_MARGIN_S,
                duration_s + _TRAJECTORY_WAIT_SETTLE_MARGIN_S,
            )
        return self.wait_for_trajectory_complete(
            traj_id,
            timeout_s=wait_timeout_s,
            submitted_command_seq=submitted_command_seq,
        )

    def wait_for_trajectory_complete(
        self,
        traj_id: int,
        *,
        timeout_s: float,
        submitted_command_seq: Optional[int] = None,
    ) -> RTCoreExecutionStatus:
        deadline = time.monotonic() + max(0.0, float(timeout_s))
        saw_target_trajectory = False
        terminal_state_names = {"idle", "completed", "aborted", "faulted", "underrun"}
        last_status: Optional[RTCoreExecutionStatus] = None

        def _is_terminal_idle_or_complete(status: RTCoreExecutionStatus) -> bool:
            state_name = str(getattr(status, "state_name", "idle") or "idle").strip().lower() or "idle"
            queue_depth = int(getattr(status, "queue_depth", 0) or 0)
            return bool(status.motion_done and queue_depth == 0 and state_name in terminal_state_names)

        while time.monotonic() <= deadline:
            status = self.get_execution_status()
            last_status = status
            state_name = str(getattr(status, "state_name", "idle") or "idle").strip().lower() or "idle"
            queue_depth = int(getattr(status, "queue_depth", 0) or 0)
            active_command_seq = int(getattr(status, "active_command_seq", 0) or 0)
            if status.active_traj_id == int(traj_id):
                saw_target_trajectory = True
                if status.state in (
                    RTCORE_EXEC_STATE_COMPLETED,
                    RTCORE_EXEC_STATE_ABORTED,
                    RTCORE_EXEC_STATE_FAULTED,
                    RTCORE_EXEC_STATE_UNDERRUN,
                ):
                    return status
            elif saw_target_trajectory and status.motion_done and status.last_update_ns > 0:
                # A newer trajectory or stop/abort command superseded this one.
                return status
            elif (
                not saw_target_trajectory
                and submitted_command_seq is not None
                and active_command_seq >= int(submitted_command_seq)
                and _is_terminal_idle_or_complete(status)
            ):
                # Very short trajectories can fully execute between polls, leaving no
                # observable window where active_traj_id == traj_id. In live commissioning
                # it is not safe to treat a bare idle snapshot as completion unless the
                # commit sequence has actually become visible in RTCore status.
                return status
            time.sleep(0.01)
        final_status = self.get_execution_status()
        if final_status.active_traj_id == int(traj_id) and final_status.state in (
            RTCORE_EXEC_STATE_COMPLETED,
            RTCORE_EXEC_STATE_ABORTED,
            RTCORE_EXEC_STATE_FAULTED,
            RTCORE_EXEC_STATE_UNDERRUN,
        ):
            return final_status
        if saw_target_trajectory and final_status.motion_done and final_status.last_update_ns > 0:
            return final_status
        if (
            not saw_target_trajectory
            and submitted_command_seq is not None
            and int(getattr(final_status, "active_command_seq", 0) or 0) >= int(submitted_command_seq)
            and _is_terminal_idle_or_complete(final_status)
        ):
            return final_status
        if (
            last_status is not None
            and not saw_target_trajectory
            and submitted_command_seq is not None
            and int(getattr(last_status, "active_command_seq", 0) or 0) >= int(submitted_command_seq)
            and _is_terminal_idle_or_complete(last_status)
        ):
            return last_status
        raise TimeoutError(f"Timed out waiting for RTCore trajectory {traj_id} to complete")

    def reset_faults(self, logical_joint_index: Optional[int] = None) -> bool:
        if not self._connected:
            print("[EtherCAT RTCore] WARNING: cannot reset faults while RTCore is disconnected")
            return False

        if self._rt_num_axes <= 0:
            print("[EtherCAT RTCore] WARNING: cannot reset faults without any configured axes")
            return False

        if logical_joint_index is None:
            axis_mask = (1 << self._rt_num_axes) - 1
            label = "all axes"
        else:
            joint_i = int(logical_joint_index)
            if joint_i < 0 or joint_i >= self._num_joints:
                print(f"[EtherCAT RTCore] WARNING: joint index out of range for fault reset: {joint_i}")
                return False
            axis_mask = 0
            for axis_i, mapped_joint in enumerate(self._axis_to_joint):
                if mapped_joint == joint_i:
                    axis_mask |= (1 << axis_i)
            if axis_mask == 0:
                print(
                    "[EtherCAT RTCore] WARNING: cannot reset faults for joint"
                    f" {joint_i + 1}; no mapped RTCore axes"
                )
                return False
            label = f"joint {joint_i + 1}"

        try:
            self.prepare_for_power_transition(wait_for_idle=True, timeout_s=_POWER_TRANSITION_DEFAULT_TIMEOUT_S)
            axis_mask_all = self._all_axis_mask()
            if axis_mask_all:
                self._send_cmd_axis_disable(axis_mask=axis_mask_all)
            self._send_cmd_arm(False)
            time.sleep(0.05)
        except Exception as exc:
            print(f"[EtherCAT RTCore] WARNING: pre-reset neutralization failed: {exc}")

        self._send_cmd_fault_reset(axis_mask=axis_mask)
        print(
            "[EtherCAT RTCore] Fault reset requested:"
            f" target={label} axis_mask=0x{axis_mask:x}"
        )
        return True

    def reset_encoder_data(self, logical_joint_index: Optional[int] = None) -> bool:
        if not self._connected:
            print("[EtherCAT RTCore] WARNING: cannot reset encoder data while RTCore is disconnected")
            return False

        if self._rt_num_axes <= 0:
            print("[EtherCAT RTCore] WARNING: cannot reset encoder data without any configured axes")
            return False

        live_profile_id = self.get_live_drive_profile_id()
        if not live_profile_id and self._rt_drive_profile_code:
            live_profile_id = rtcore_drive_profile_id_to_name(self._rt_drive_profile_code)
        operation = drive_profile_registry.get_drive_encoder_data_reset_operation(live_profile_id)
        if not isinstance(operation, dict):
            print(
                "[EtherCAT RTCore] WARNING: active drive profile does not define an encoder-data reset operation:"
                f" profile={live_profile_id!r}"
            )
            return False

        if str(operation.get("type", "")).strip().lower() != "u16":
            print(
                "[EtherCAT RTCore] WARNING: unsupported encoder-data reset operation type:"
                f" {operation.get('type')!r}"
            )
            return False

        if logical_joint_index is None:
            axis_mask = (1 << self._rt_num_axes) - 1
            label = "all axes"
        else:
            joint_i = int(logical_joint_index)
            if joint_i < 0 or joint_i >= self._num_joints:
                print(f"[EtherCAT RTCore] WARNING: joint index out of range for encoder reset: {joint_i}")
                return False
            axis_mask = 0
            for axis_i, mapped_joint in enumerate(self._axis_to_joint):
                if mapped_joint == joint_i:
                    axis_mask |= (1 << axis_i)
            if axis_mask == 0:
                print(
                    "[EtherCAT RTCore] WARNING: cannot reset encoder data for joint"
                    f" {joint_i + 1}; no mapped RTCore axes"
                )
                return False
            label = f"joint {joint_i + 1}"

        try:
            self.prepare_for_power_transition(wait_for_idle=True, timeout_s=_POWER_TRANSITION_DEFAULT_TIMEOUT_S)
            axis_mask_all = self._all_axis_mask()
            if axis_mask_all:
                self._send_cmd_axis_disable(axis_mask=axis_mask_all)
            self._send_cmd_arm(False)
            time.sleep(0.05)
        except Exception as exc:
            print(f"[EtherCAT RTCore] WARNING: pre-encoder-reset neutralization failed: {exc}")

        self._send_cmd_service_sdo_write(
            axis_mask=axis_mask,
            index=int(operation.get("index", 0)),
            subindex=int(operation.get("subindex", 0)),
            value_type=_SERVICE_SDO_VALUE_U16,
            value_u32=int(operation.get("value", 0)),
        )
        print(
            "[EtherCAT RTCore] Encoder data reset requested:"
            f" target={label} axis_mask=0x{axis_mask:x}"
            f" profile={live_profile_id!r} parameter={operation.get('parameter')}"
            f" index=0x{int(operation.get('index', 0)) & 0xFFFF:04x}"
            f" sub=0x{int(operation.get('subindex', 0)) & 0xFF:02x}"
            f" value={int(operation.get('value', 0))}"
        )
        return True

    def native_home_joint(self, logical_joint_index: int) -> dict[str, object]:
        def _result(
            *,
            accepted: bool,
            verified: bool,
            code: str,
            message: str,
            joint: int,
            axis_mask: int,
            timed_out: bool = False,
            terminal_state: str = "idle",
            native_home_state: int = 0,
            native_home_last_abort_code: int = 0,
            metrics_time_ns: int = 0,
        ) -> dict[str, object]:
            abort_code = int(native_home_last_abort_code)
            return {
                "accepted": bool(accepted),
                "verified": bool(verified),
                "timed_out": bool(timed_out),
                "code": str(code),
                "message": str(message),
                "joint": int(joint),
                "axis_mask": int(axis_mask),
                "terminal_state": str(terminal_state),
                "native_home_state": int(native_home_state),
                "native_home_state_name": _native_home_state_name(native_home_state),
                "native_home_last_abort_code": abort_code,
                "native_home_last_abort_code_hex": f"0x{abort_code & 0xFFFFFFFF:08X}",
                "disarmed_after_home": True,
                "metrics_time_ns": int(metrics_time_ns),
            }

        if not self._connected:
            print("[EtherCAT RTCore] WARNING: cannot drive-home a joint while RTCore is disconnected")
            return _result(
                accepted=False,
                verified=False,
                code="NATIVE_HOME_UNAVAILABLE",
                message="RTCore is disconnected; cannot run drive-native home.",
                joint=int(logical_joint_index) + 1,
                axis_mask=0,
            )
        if self._rt_num_axes <= 0:
            print("[EtherCAT RTCore] WARNING: cannot drive-home without any configured axes")
            return _result(
                accepted=False,
                verified=False,
                code="NATIVE_HOME_UNAVAILABLE",
                message="RTCore did not report any configured axes for native home.",
                joint=int(logical_joint_index) + 1,
                axis_mask=0,
            )

        joint_i = int(logical_joint_index)
        if joint_i < 0 or joint_i >= self._num_joints:
            print(f"[EtherCAT RTCore] WARNING: joint index out of range for native homing: {joint_i}")
            return _result(
                accepted=False,
                verified=False,
                code="NATIVE_HOME_INVALID_JOINT",
                message=f"Joint index {joint_i + 1} is out of range for native home.",
                joint=joint_i + 1,
                axis_mask=0,
            )

        axis_mask = 0
        for axis_i, mapped_joint in enumerate(self._axis_to_joint):
            if mapped_joint == joint_i:
                axis_mask |= 1 << axis_i
        if axis_mask == 0:
            print(
                "[EtherCAT RTCore] WARNING: cannot drive-home joint"
                f" {joint_i + 1}; no mapped RTCore axes"
            )
            return _result(
                accepted=False,
                verified=False,
                code="NATIVE_HOME_UNMAPPED",
                message=f"Joint {joint_i + 1} has no mapped RTCore axis for native home.",
                joint=joint_i + 1,
                axis_mask=0,
            )

        try:
            self.prepare_for_power_transition(wait_for_idle=True, timeout_s=_POWER_TRANSITION_DEFAULT_TIMEOUT_S)
        except Exception as exc:
            print(f"[EtherCAT RTCore] WARNING: pre-native-home neutralization failed: {exc}")
            return _result(
                accepted=False,
                verified=False,
                code="NATIVE_HOME_PRECONDITION_FAILED",
                message=f"Could not neutralize motion before drive-native home: {exc}",
                joint=joint_i + 1,
                axis_mask=axis_mask,
            )

        baseline_snapshot = self._load_rtcore_metrics_snapshot()
        self._send_cmd_native_home(axis_mask=axis_mask)
        wait_timeout_s = self._env_float(
            "GRADIENT_RTCORE_NATIVE_HOME_WAIT_TIMEOUT_S",
            _NATIVE_HOME_WAIT_TIMEOUT_S,
        )
        wait_result = self._wait_for_native_home_result(
            axis_mask,
            timeout_s=wait_timeout_s,
            min_metrics_time_ns=(
                int(baseline_snapshot.get("time_ns", 0))
                if isinstance(baseline_snapshot, dict)
                else 0
            ),
            min_metrics_mtime_ns=(
                int(baseline_snapshot.get("_mtime_ns", 0))
                if isinstance(baseline_snapshot, dict)
                else 0
            ),
        )
        if bool(wait_result.get("verified", False)):
            try:
                self._capture_absolute_home_anchor_for_joint(
                    joint_i,
                    actor=f"ethercat_rtcore:joint{joint_i + 1}:native_home",
                )
            except Exception:
                pass
            print(
                "[EtherCAT RTCore] Native drive-home verified:"
                f" joint={joint_i + 1} axis_mask=0x{axis_mask:x}"
                " leaving the homed axis disabled until an explicit safe power-up"
            )
            return _result(
                accepted=True,
                verified=True,
                code="NATIVE_HOME_VERIFIED",
                message=(
                    "Drive-native commissioning home verified. "
                    "The homed axis remains disabled until an explicit safe power-up."
                ),
                joint=joint_i + 1,
                axis_mask=axis_mask,
                terminal_state=str(wait_result.get("terminal_state", "succeeded")),
                native_home_state=int(wait_result.get("native_home_state", 2)),
                native_home_last_abort_code=int(wait_result.get("native_home_last_abort_code", 0)),
                metrics_time_ns=int(wait_result.get("metrics_time_ns", 0)),
            )

        if bool(wait_result.get("timed_out", False)):
            print(
                "[EtherCAT RTCore] WARNING: native drive-home is still awaiting verified terminal state:"
                f" joint={joint_i + 1} axis_mask=0x{axis_mask:x}"
                f" timeout_s={wait_timeout_s:.2f}"
            )
            return _result(
                accepted=True,
                verified=False,
                timed_out=True,
                code="NATIVE_HOME_PENDING_VERIFICATION",
                message=(
                    "Drive-native commissioning home was requested, but RTCore did not observe a "
                    "verified terminal state before the wait window expired. Keep the axis disabled "
                    "and confirm the live drive-home status before powering it back up."
                ),
                joint=joint_i + 1,
                axis_mask=axis_mask,
                terminal_state=str(wait_result.get("terminal_state", "pending")),
                native_home_state=int(wait_result.get("native_home_state", 1)),
                native_home_last_abort_code=int(wait_result.get("native_home_last_abort_code", 0)),
                metrics_time_ns=int(wait_result.get("metrics_time_ns", 0)),
            )

        abort_code = int(wait_result.get("native_home_last_abort_code", 0))
        print(
            "[EtherCAT RTCore] WARNING: native drive-home failed verification:"
            f" joint={joint_i + 1} axis_mask=0x{axis_mask:x}"
            f" abort=0x{abort_code & 0xFFFFFFFF:08X}"
        )
        return _result(
            accepted=False,
            verified=False,
            code="NATIVE_HOME_FAILED",
            message=(
                "Drive-native commissioning home failed verification."
                + (
                    f" Abort code 0x{abort_code & 0xFFFFFFFF:08X}."
                    if abort_code != 0
                    else ""
                )
            ),
            joint=joint_i + 1,
            axis_mask=axis_mask,
            terminal_state=str(wait_result.get("terminal_state", "failed")),
            native_home_state=int(wait_result.get("native_home_state", 3)),
            native_home_last_abort_code=abort_code,
            metrics_time_ns=int(wait_result.get("metrics_time_ns", 0)),
        )

    def _best_effort_safe_power_down(
        self,
        *,
        wait_for_idle: bool = False,
        timeout_s: float | None = None,
    ) -> None:
        if not self._connected:
            return

        try:
            self.prepare_for_power_transition(wait_for_idle=wait_for_idle, timeout_s=timeout_s)
            axis_mask = self._all_axis_mask()
            if axis_mask:
                self._send_cmd_axis_disable(axis_mask=axis_mask)
            self._send_cmd_arm(False)
            # Give the RTCore thread a brief window to consume the disable/disarm command
            # before we tear down IPC resources on the Python side.
            time.sleep(0.05)
        except Exception as e:
            print(f"[EtherCAT RTCore] WARNING: safe power-down failed during shutdown: {e}")

    def _best_effort_safe_power_up(self) -> None:
        if not self._connected:
            return
        try:
            sync_result = self.synchronize_command_targets_to_feedback()
            if not bool(sync_result.get("synchronized", False)):
                print(
                    "[EtherCAT RTCore] WARNING: power-up target synchronization failed:"
                    f" {sync_result.get('reason', 'unknown')}"
                )
            axis_mask_all = self._all_axis_mask()
            axis_mask = axis_mask_all
            if self._auto_arm_mask is not None:
                axis_mask = int(self._auto_arm_mask) & int(axis_mask_all)
            if not axis_mask:
                print("[EtherCAT RTCore] WARNING: safe power-up skipped; no axes selected")
                return
            self._send_cmd_arm(True)
            self._send_cmd_set_mode(axis_mask=axis_mask, mode=_MODE_CSP)
            self._send_cmd_axis_enable(axis_mask=axis_mask)
        except Exception as e:
            print(f"[EtherCAT RTCore] WARNING: safe power-up failed: {e}")

    @property
    def num_joints(self) -> int:
        return self._num_joints

    @property
    def is_initialized(self) -> bool:
        return self._initialized and self._connected

    @property
    def encoder_resolution(self) -> int:
        # EtherCAT RTCore does not use "servo encoder resolution" values here.
        # Returning 0 discourages use in legacy clamp logic.
        return 0

    def set_joint_positions(
        self,
        positions_rad: list[float],
        speed: float,
        acceleration: float,
    ) -> None:
        if len(positions_rad) != self._num_joints:
            raise ValueError(f"Expected {self._num_joints} joint positions, got {len(positions_rad)}")

        if not self._connected:
            raise RuntimeError("RTCore not connected (cannot send setpoints)")

        if self._rt_num_axes <= 0:
            raise RuntimeError("RTCore did not report a valid num_axes")

        traj_id = self.begin_trajectory(expected_points=1)
        self.enqueue_trajectory_points(
            traj_id,
            [
                {
                    "positions_rad": list(positions_rad),
                    "t_from_start_ns": 0,
                    "axis_mask": (1 << self._rt_num_axes) - 1,
                }
            ],
        )
        self.commit_trajectory(traj_id)
        self._store_last_joint_setpoint_rad(positions_rad)

    def get_joint_positions(self, verbose: bool = False) -> list[float]:
        if self._connected and self._axis_config is None:
            # Give the status thread a brief chance to receive scaling data.
            self._wait_for_feedback_ready(timeout_s=0.25, require_live_wkc=False)
        if self._connected and self._axis_config is not None:
            positions = self.raw_to_joint_positions(self.sync_read_positions())
            if verbose:
                print("[EtherCAT RTCore] get_joint_positions() (connected) -> live feedback")
            return positions
        if verbose:
            state = "connected" if self._connected else "disconnected"
            print(f"[EtherCAT RTCore] get_joint_positions() ({state}) -> last setpoint")
        return self._copy_last_joint_setpoint_rad()

    def prepare_sync_write_commands(
        self,
        positions_rad: list[float],
        speed: int = 4095,
        accel: int = 0,
    ) -> list[tuple]:
        # This backend is trajectory-only. Return a backend-private point payload
        # so sync_write() can wrap it as a one-point trajectory when callers still
        # use the generic sync-write executor interface.
        return [("trajectory_point_rad", list(positions_rad))]

    def sync_write(self, commands: list[tuple]) -> None:
        # Accept our private command format only.
        if not commands:
            return

        kind = commands[0][0] if isinstance(commands[0], tuple) and commands[0] else None
        if kind != "trajectory_point_rad":
            raise NotImplementedError(
                "ethercat_rtcore requires trajectory-point command tuples; "
                "use set_joint_positions(), execute_joint_trajectory(), or prepare_sync_write_commands()."
            )

        positions_rad = commands[0][1]
        if not isinstance(positions_rad, list):
            raise ValueError("Invalid setpoint command format")
        self.set_joint_positions(positions_rad, speed=0.0, acceleration=0.0)

    def sync_read_positions(self, timeout_s: Optional[float] = None) -> dict[int, int]:
        # Return last known raw counts for each exposed axis (index -> pos_counts).
        if not self._connected:
            return {}

        return {i: int(self._axis_counts[i]) for i in range(self._rt_num_axes)}

    def _canonical_joint_positions_from_raw_feedback(
        self,
        raw_positions: dict[int, int],
    ) -> dict[str, object]:
        positions = self._copy_last_joint_setpoint_rad()
        axis_truth_details: list[dict[str, object]] = []
        unavailable_axes: list[int] = []
        unavailable_joints: list[int] = []
        for axis_i, joint_i in enumerate(self._axis_to_joint):
            raw = raw_positions.get(axis_i)
            logical_joint = joint_i + 1 if 0 <= joint_i < self._num_joints else None
            metrics = self._absolute_feedback_metrics_for_axis(axis_i)
            normalized_absolute_feedback = self._normalize_absolute_feedback_metrics(metrics)
            absolute_result = self._absolute_axis_q_from_metrics(axis_i, metrics)
            anchor_entry = (
                self._absolute_home_anchor_for_joint(joint_i)
                if 0 <= joint_i < self._num_joints
                else None
            )
            detail: dict[str, object] = {
                "axis": int(axis_i),
                "logical_joint": logical_joint,
                "absolute_feedback": normalized_absolute_feedback,
            }
            if raw is not None:
                detail["raw_counts"] = int(raw)
                reference_q = self._reference_q_before_master_offset_for_axis(axis_i, int(raw))
                if reference_q is not None:
                    detail["reference_pre_zero_rad"] = float(reference_q)
            if anchor_entry is not None:
                detail["absolute_home_anchor_rad"] = float(anchor_entry["home_anchor_rad"])
                if anchor_entry.get("source") is not None:
                    detail["absolute_home_anchor_source"] = anchor_entry["source"]
            if absolute_result is not None:
                absolute_axis_q, absolute_source, absolute_counts = absolute_result
                detail["absolute_counts"] = int(absolute_counts)
                detail["absolute_source"] = str(absolute_source)
                detail["absolute_axis_q_rad"] = float(absolute_axis_q)
            else:
                absolute_axis_q = None
                absolute_source = None

            truth_reason = None
            if raw is None:
                truth_reason = "raw_feedback_missing"
            elif joint_i < 0 or joint_i >= self._num_joints:
                truth_reason = "logical_joint_unmapped"
            elif absolute_axis_q is None:
                truth_reason = "absolute_feedback_unavailable"
            elif anchor_entry is None:
                truth_reason = "absolute_home_anchor_missing"

            if truth_reason is not None:
                detail["truth_available"] = False
                detail["truth_status"] = "unavailable"
                detail["truth_reason"] = truth_reason
                detail["display_source"] = "truth_unavailable"
                unavailable_axes.append(int(axis_i))
                if logical_joint is not None:
                    unavailable_joints.append(int(logical_joint))
                axis_truth_details.append(detail)
                continue

            canonical_q = (
                float(absolute_axis_q)
                - float(anchor_entry["home_anchor_rad"])
                - self._master_offset_for_joint(joint_i)
            )
            positions[joint_i] = float(canonical_q)
            detail["truth_available"] = True
            detail["truth_status"] = "available"
            detail["truth_source"] = "absolute_encoder_anchor"
            detail["canonical_rad"] = float(canonical_q)
            # Keep the legacy display fields as diagnostics/compatibility aliases
            # while the rest of the stack is migrated onto the canonical fields.
            detail["display_source"] = "absolute_encoder_anchor"
            detail["display_rad"] = float(canonical_q)
            axis_truth_details.append(detail)

        unavailable_axes = sorted(set(unavailable_axes))
        unavailable_joints = sorted(set(unavailable_joints))
        return {
            "joint_positions_rad": [float(value) for value in positions],
            "axis_absolute_feedback": axis_truth_details,
            "truth_available": len(unavailable_axes) == 0,
            "truth_unavailable_axes": unavailable_axes,
            "truth_unavailable_joints": unavailable_joints,
        }

    def _canonical_joint_positions_or_raise(self, raw_positions: dict[int, int]) -> list[float]:
        snapshot = self._canonical_joint_positions_from_raw_feedback(raw_positions)
        if bool(snapshot.get("truth_available")):
            positions = snapshot.get("joint_positions_rad")
            if isinstance(positions, list) and positions:
                return [float(value) for value in positions]
        unavailable_axes = snapshot.get("truth_unavailable_axes")
        unavailable_joints = snapshot.get("truth_unavailable_joints")
        raise RuntimeError(
            "Canonical joint truth unavailable"
            f" (axes={list(unavailable_axes) if isinstance(unavailable_axes, list) else []},"
            f" joints={list(unavailable_joints) if isinstance(unavailable_joints, list) else []})"
        )

    def raw_to_joint_positions(self, raw_positions: dict[int, int]) -> list[float]:
        # Canonical controller truth now comes from anchored multi-turn absolute
        # feedback. RTCore's raw 0x6064/0x607A frame remains a derived transport
        # encoding used only when translating queued targets back into wire counts.
        return self._canonical_joint_positions_or_raise(raw_positions)

    def get_display_feedback_snapshot(
        self,
        raw_positions: dict[int, int] | None = None,
    ) -> dict[str, object] | None:
        if raw_positions is None:
            raw_positions = self.sync_read_positions()
        if not isinstance(raw_positions, dict) or not raw_positions:
            return None
        return self._canonical_joint_positions_from_raw_feedback(raw_positions)

    def raw_to_display_joint_positions(self, raw_positions: dict[int, int]) -> list[float]:
        # Legacy compatibility alias: operator display truth is now the same
        # canonical logical joint truth that the controller uses everywhere else.
        return self._canonical_joint_positions_or_raise(raw_positions)

    def set_single_actuator_position(
        self,
        actuator_id: int,
        position_rad: float,
        speed: int,
        accel: int,
    ) -> None:
        # In EtherCAT/RTCore backend, actuator_id == RTCore axis index.
        if actuator_id < 0 or actuator_id >= self._rt_num_axes:
            raise ValueError(f"Axis index out of range: {actuator_id}")

        if not self._connected:
            raise RuntimeError("RTCore not connected (cannot send setpoints)")

        axis_q = [0.0] * self._rt_num_axes
        axis_q[actuator_id] = float(position_rad)
        axis_mask = 1 << actuator_id
        traj_id = self.begin_trajectory(axis_mask=axis_mask, expected_points=1)
        self.enqueue_trajectory_points(
            traj_id,
            [
                {
                    "axis_q": axis_q,
                    "t_from_start_ns": 0,
                    "axis_mask": axis_mask,
                }
            ],
        )
        self.commit_trajectory(traj_id)
        self._store_last_axis_target_q(actuator_id, float(position_rad))

    def read_single_actuator_position(self, actuator_id: int) -> Optional[int]:
        if not self._connected:
            return None
        if actuator_id < 0 or actuator_id >= self._rt_num_axes:
            return None
        return int(self._axis_counts[actuator_id])

    def set_current_position_as_zero(self, actuator_id: int) -> bool:
        axis_i = int(actuator_id)
        if axis_i < 0 or axis_i >= len(self._axis_to_joint):
            return False
        joint_i = self._axis_to_joint[axis_i]
        if joint_i < 0 or joint_i >= self._num_joints:
            return False
        return self.set_logical_joint_current_position_as_zero(joint_i)

    def set_logical_joint_current_position_as_zero(self, logical_joint_index: int) -> bool:
        joint_i = int(logical_joint_index)
        if joint_i < 0 or joint_i >= self._num_joints:
            print(f"[EtherCAT RTCore] WARNING: joint index out of range for zeroing: {joint_i}")
            return False
        if not self._connected:
            print("[EtherCAT RTCore] WARNING: cannot zero joint while RTCore is disconnected")
            return False

        physical_samples: list[float] = []
        for axis_i, mapped_joint in enumerate(self._axis_to_joint):
            if mapped_joint != joint_i:
                continue
            physical_q = self._display_axis_q_from_raw_feedback_counts(
                axis_i,
                int(self._axis_counts[axis_i]),
            )
            if physical_q is not None:
                physical_samples.append(physical_q)

        if not physical_samples:
            print(
                "[EtherCAT RTCore] WARNING: cannot zero joint"
                f" {joint_i + 1}; no live scaled feedback available yet"
            )
            return False

        native_home_samples = [
            physical_q + self._native_home_offset_q_for_axis(axis_i)
            for axis_i, physical_q in zip(
                [axis_i for axis_i, mapped_joint in enumerate(self._axis_to_joint) if mapped_joint == joint_i],
                physical_samples,
                strict=False,
            )
        ]
        new_offset = float(sum(native_home_samples) / len(native_home_samples))
        self._master_offsets_rad[joint_i] = new_offset
        self._store_last_joint_index_setpoint(joint_i, 0.0)
        save_joint_zero_offsets(
            self._robot_id,
            self._master_offsets_rad,
            actor=f"ethercat_rtcore:joint{joint_i + 1}",
        )
        try:
            self._capture_absolute_home_anchor_for_joint(
                joint_i,
                actor=f"ethercat_rtcore:joint{joint_i + 1}:software_zero",
            )
        except Exception:
            pass
        print(
            "[EtherCAT RTCore] Captured logical zero:"
            f" joint={joint_i + 1} physical_q={new_offset:.6f} rad"
        )
        return True

    def set_pid_gains(self, actuator_id: int, kp: int, ki: int, kd: int) -> bool:
        # Not supported here (should be RTCore commissioning / SDO templates).
        return False

    def apply_joint_limits(self) -> bool:
        # Limits are enforced in RTCore using axis config + soft limit commands.
        return False

    def get_present_actuator_ids(self) -> set[int]:
        if not self._connected:
            return set()
        return set(range(self._rt_num_axes))

    def ping_actuator(self, actuator_id: int) -> bool:
        return actuator_id in self.get_present_actuator_ids()

    # -------------------------------------------------------------------------
    # IPC internals
    # -------------------------------------------------------------------------

    def _connect_ipc(self) -> bool:
        if self._connected:
            return True

        sock = socket.socket(socket.AF_UNIX, socket.SOCK_SEQPACKET)
        try:
            sock.connect(self._socket_path)
        except FileNotFoundError:
            print(f"[EtherCAT RTCore] RTCore socket not found: {self._socket_path}")
            sock.close()
            return False
        except Exception as e:
            print(f"[EtherCAT RTCore] Failed to connect to RTCore socket: {e}")
            sock.close()
            return False

        # Send HELLO.
        hello = _HELLO_STRUCT.pack(
            _MAGIC_GIPC,
            _VER_MAJOR,
            _VER_MINOR,
            _HELLO_STRUCT.size,
            _ROLE_CONTROLLER,
            os.getpid(),
            0,
            0,
            0,
            0,
        )
        sock.sendall(hello)

        # Receive WELCOME + FDs.
        fd_size = array.array("i").itemsize
        data, ancdata, _flags, _addr = sock.recvmsg(
            _WELCOME_STRUCT.size,
            socket.CMSG_SPACE(fd_size * 4),
        )
        if len(data) != _WELCOME_STRUCT.size:
            sock.close()
            raise RuntimeError(f"WELCOME size mismatch (got {len(data)} bytes)")

        fds: list[int] = []
        for level, ctype, cmsg_data in ancdata:
            if level == socket.SOL_SOCKET and ctype == socket.SCM_RIGHTS:
                arr = array.array("i")
                arr.frombytes(cmsg_data[: len(cmsg_data) - (len(cmsg_data) % fd_size)])
                fds.extend(arr.tolist())

        if len(fds) < 4:
            sock.close()
            raise RuntimeError(f"Expected 4 SCM_RIGHTS fds, got {len(fds)}")

        (
            magic,
            vmaj,
            vmin,
            bytes_len,
            num_axes,
            _reserved0,
            cycle_ns,
            topology_hash,
            cmd_ring_capacity,
            cmd_msg_bytes,
            status_ring_capacity,
            status_msg_bytes,
            build_id_hash,
            *_rest,
        ) = _WELCOME_STRUCT.unpack(data)

        if magic != _MAGIC_GIPC or vmaj != _VER_MAJOR or vmin != _VER_MINOR or bytes_len != _WELCOME_STRUCT.size:
            sock.close()
            raise RuntimeError("WELCOME validation failed (magic/ver/bytes)")

        self._sock = sock
        self._rt_num_axes = int(num_axes)
        self._cmd_shm_fd, self._status_shm_fd, self._cmd_eventfd, self._status_eventfd = fds[:4]

        # Map shared memory.
        self._cmd_shm = self._map_fd(self._cmd_shm_fd)
        self._status_shm = self._map_fd(self._status_shm_fd)

        self._cmd_hdr = self._parse_shm_header(self._cmd_shm)
        self._status_hdr = self._parse_shm_header(self._status_shm)

        # Basic sanity checks.
        if self._cmd_hdr.kind != 1 or self._status_hdr.kind != 2:
            raise RuntimeError("SHM kind mismatch (cmd/status)")
        if self._cmd_hdr.num_axes != self._status_hdr.num_axes:
            raise RuntimeError("SHM num_axes mismatch")

        self._connected = True
        self._initialized = True

        self._axis_to_joint = self._resolve_axis_to_joint_map(self._rt_num_axes, self._num_joints)

        print(
            "[EtherCAT RTCore] Connected:"
            f" num_axes={num_axes} cycle_ns={cycle_ns} topo_hash=0x{topology_hash:016x}"
            f" build=0x{build_id_hash:016x}"
            f" cmd_ring={cmd_ring_capacity}x{cmd_msg_bytes} status_ring={status_ring_capacity}x{status_msg_bytes}"
        )

        self._start_status_thread()

        feedback_timeout_s = self._env_float("GRADIENT_RTCORE_FEEDBACK_READY_TIMEOUT_S", 2.0)
        if feedback_timeout_s > 0.0:
            if self._wait_for_feedback_ready(timeout_s=feedback_timeout_s, require_live_wkc=False):
                print(
                    "[EtherCAT RTCore] Feedback ready:"
                    f" axis_config=1 snapshot=1 wkc={self._last_wkc_actual}/{self._last_wkc_expected}"
                    f" master_state={self._last_master_state}"
                )
            else:
                print(
                    "[EtherCAT RTCore] WARNING: Timed out waiting for RTCore status feedback"
                    f" ({self._feedback_ready_summary()})"
                )

        # Default safety policy: connect disarmed unless the environment opts back
        # into the legacy auto-arm behavior.
        if self._auto_arm:
            try:
                self._best_effort_safe_power_up()
            except Exception as e:
                print(f"[EtherCAT RTCore] WARNING: auto-arm failed: {e}")

        return True

    def _resolve_axis_to_joint_map(self, num_axes: int, num_joints: int) -> list[int]:
        """
        Determine how GradientOS joint vectors map to RTCore axes.

        Env override (1-based joint numbers):
          GRADIENT_RTCORE_CONTROL_JOINTS="3,4"
        """
        raw = os.environ.get("GRADIENT_RTCORE_CONTROL_JOINTS", "").strip()
        if raw:
            parts = [p.strip() for p in raw.split(",") if p.strip()]
            joints: list[int] = []
            for p in parts:
                try:
                    j1 = int(p)
                except ValueError:
                    continue
                joints.append(j1 - 1)  # convert to 0-based
            if len(joints) == num_axes and all(0 <= j < num_joints for j in joints):
                return joints
            print(
                f"[EtherCAT RTCore] WARNING: GRADIENT_RTCORE_CONTROL_JOINTS='{raw}' "
                f"does not match num_axes={num_axes}; using defaults."
            )

        # Default mapping policy: axis0..axisN maps to joint0..jointN in order.
        return list(range(min(num_axes, num_joints)))

    def _env_float(self, name: str, default: float) -> float:
        raw = os.environ.get(name, str(default)).strip()
        try:
            return float(raw)
        except Exception:
            return float(default)

    def _feedback_ready_summary(self) -> str:
        with self._status_lock:
            return (
                f"axis_config={1 if self._axis_config is not None else 0} "
                f"snapshot={1 if self._status_snapshot_event.is_set() else 0} "
                f"wkc={self._last_wkc_actual}/{self._last_wkc_expected} "
                f"master_state={self._last_master_state}"
            )

    def _wait_for_feedback_ready(self, timeout_s: float, *, require_live_wkc: bool) -> bool:
        deadline = time.monotonic() + max(0.0, float(timeout_s))
        while time.monotonic() <= deadline:
            with self._status_lock:
                have_axis_cfg = self._axis_config is not None
                have_snapshot = self._status_snapshot_event.is_set()
                live_wkc = self._last_wkc_actual > 0
            if have_axis_cfg and have_snapshot and (live_wkc or not require_live_wkc):
                return True
            time.sleep(0.01)
        return False

    def _cmd_ring_offsets(self) -> tuple[int, int]:
        assert self._cmd_hdr is not None
        ring_hdr_off = self._cmd_hdr.ring_offset
        ring_entries_off = ring_hdr_off + _align_up(_RING_HEADER_STRUCT.size, 8)
        return ring_hdr_off, ring_entries_off

    def _cmd_ring_write(self, msg_type: int, payload: bytes) -> int:
        if self._cmd_shm is None or self._cmd_hdr is None:
            raise RuntimeError("cmd_shm not mapped")

        ring_hdr_off, ring_entries_off = self._cmd_ring_offsets()
        deadline = time.monotonic() + _CMD_RING_WRITE_WAIT_S
        while True:
            hdr_bytes = self._cmd_shm[ring_hdr_off : ring_hdr_off + _RING_HEADER_STRUCT.size]
            magic, capacity, msg_bytes, write_idx, read_idx, dropped, reserved0 = _RING_HEADER_STRUCT.unpack(hdr_bytes)
            if magic != _MAGIC_RING:
                raise RuntimeError("cmd ring header magic mismatch")
            if capacity == 0 or msg_bytes == 0:
                raise RuntimeError("cmd ring has invalid sizing")
            if (write_idx - read_idx) < capacity:
                break
            if time.monotonic() >= deadline:
                # Producer increments dropped on overflow.
                dropped += 1
                self._cmd_shm[ring_hdr_off + 20 : ring_hdr_off + 24] = struct.pack("<I", dropped)
                raise RuntimeError("cmd ring overflow")
            time.sleep(_CMD_RING_WRITE_RETRY_S)

        slot = write_idx % capacity
        off = ring_entries_off + (slot * msg_bytes)

        # Construct message: header + payload + zero padding to msg_bytes.
        time_ns = _now_monotonic_ns()
        msg_seq = int(self._cmd_seq)
        header = _MSG_HEADER_STRUCT.pack(
            int(msg_type) & 0xFFFF,
            0,
            _MSG_HEADER_STRUCT.size + len(payload),
            msg_seq,
            int(time_ns),
        )
        self._cmd_seq += 1

        blob = header + payload
        if len(blob) > msg_bytes:
            raise RuntimeError("cmd payload exceeds ring slot size")
        blob = blob.ljust(msg_bytes, b"\x00")
        self._cmd_shm[off : off + msg_bytes] = blob

        # Publish new write_idx (producer-owned).
        write_idx += 1
        self._cmd_shm[ring_hdr_off + 12 : ring_hdr_off + 16] = struct.pack("<I", write_idx)

        # Wake RTCore helper thread.
        if self._cmd_eventfd is not None:
            try:
                os.write(self._cmd_eventfd, struct.pack("<Q", 1))
            except Exception:
                pass
        return msg_seq

    def _send_cmd_arm(self, arm: bool) -> None:
        self._cmd_ring_write(_MSG_CMD_ARM, _CMD_ARM_STRUCT.pack(1 if arm else 0, 0))

    def _send_cmd_axis_enable(self, axis_mask: int) -> None:
        self._cmd_ring_write(_MSG_CMD_AXIS_ENABLE, _CMD_AXIS_MASK_STRUCT.pack(int(axis_mask), 0))

    def _send_cmd_axis_disable(self, axis_mask: int) -> None:
        self._cmd_ring_write(_MSG_CMD_AXIS_DISABLE, _CMD_AXIS_MASK_STRUCT.pack(int(axis_mask), 0))

    def _send_cmd_fault_reset(self, axis_mask: int) -> None:
        self._cmd_ring_write(_MSG_CMD_FAULT_RESET, _CMD_AXIS_MASK_STRUCT.pack(int(axis_mask), 0))

    def _send_cmd_set_mode(self, axis_mask: int, mode: int) -> None:
        self._cmd_ring_write(_MSG_CMD_SET_MODE, _CMD_SET_MODE_STRUCT.pack(int(axis_mask), int(mode)))

    def _send_cmd_native_home(self, axis_mask: int) -> None:
        self._cmd_ring_write(_MSG_CMD_NATIVE_HOME, _CMD_AXIS_MASK_STRUCT.pack(int(axis_mask), 0))

    def _send_cmd_service_sdo_write(
        self,
        *,
        axis_mask: int,
        index: int,
        subindex: int,
        value_type: int,
        value_u32: int,
        flags: int = 0,
    ) -> None:
        self._cmd_ring_write(
            _MSG_CMD_SERVICE_SDO_WRITE,
            _CMD_SERVICE_SDO_WRITE_STRUCT.pack(
                int(axis_mask),
                int(index) & 0xFFFF,
                int(subindex) & 0xFF,
                int(value_type) & 0xFF,
                int(value_u32) & 0xFFFFFFFF,
                int(flags) & 0xFFFFFFFF,
            ),
        )

    def _send_cmd_jog(
        self,
        *,
        axis_mask: int,
        flags: int,
        timeout_ns: int,
        axis_qd: list[float],
    ) -> None:
        payload_qd = [0.0] * _GRADIENT_MAX_AXES
        for idx, value in enumerate(axis_qd[:_GRADIENT_MAX_AXES]):
            payload_qd[idx] = float(value)
        self._cmd_ring_write(
            _MSG_CMD_JOG,
            _CMD_JOG_STRUCT.pack(
                int(axis_mask),
                int(flags),
                int(timeout_ns),
                *payload_qd,
            ),
        )

    def _map_fd(self, fd: int) -> mmap.mmap:
        st = os.fstat(fd)
        if st.st_size <= 0:
            raise RuntimeError("Shared memory fd has zero size")
        return mmap.mmap(fd, st.st_size, flags=mmap.MAP_SHARED, prot=mmap.PROT_READ | mmap.PROT_WRITE)

    def _parse_shm_header(self, mm: mmap.mmap) -> _ShmHeader:
        data = mm[: _SHM_HEADER_STRUCT.size]
        (
            magic,
            vmaj,
            vmin,
            bytes_len,
            kind,
            num_axes,
            _reserved0,
            cycle_ns,
            topology_hash,
            ring_offset,
            ring_capacity,
            ring_msg_bytes,
            setpoint_offset,
            _reserved1,
            *_rest,
        ) = _SHM_HEADER_STRUCT.unpack(data)

        if magic != _MAGIC_GSHM or vmaj != _VER_MAJOR or vmin != _VER_MINOR or bytes_len != _SHM_HEADER_STRUCT.size:
            raise RuntimeError("SHM header validation failed (magic/ver/bytes)")

        return _ShmHeader(
            kind=int(kind),
            num_axes=int(num_axes),
            cycle_ns=int(cycle_ns),
            topology_hash=int(topology_hash),
            ring_offset=int(ring_offset),
            ring_capacity=int(ring_capacity),
            ring_msg_bytes=int(ring_msg_bytes),
            setpoint_offset=int(setpoint_offset),
        )

    def _parse_axis_config(self, payload: bytes) -> _AxisConfig:
        (
            num_axes,
            _reserved0,
            *rest,
        ) = _AXIS_CONFIG_STRUCT.unpack_from(payload, 0)
        idx = 0
        counts_per_rev = [int(x) for x in rest[idx : idx + _GRADIENT_MAX_AXES]]
        idx += _GRADIENT_MAX_AXES  # counts_per_rev
        idx += _GRADIENT_MAX_AXES  # gear_ratio
        sign = [int(x) for x in rest[idx : idx + _GRADIENT_MAX_AXES]]
        idx += _GRADIENT_MAX_AXES
        idx += _GRADIENT_MAX_AXES  # axis_type
        counts_per_unit = [float(x) for x in rest[idx : idx + _GRADIENT_MAX_AXES]]
        return _AxisConfig(
            num_axes=int(num_axes),
            counts_per_unit=counts_per_unit,
            sign=sign,
            counts_per_rev=counts_per_rev,
        )

    def _parse_status_hello(self, payload: bytes) -> tuple[Optional[str], int, int]:
        (
            _build_id_hash,
            _topology_hash,
            _cycle_ns,
            _num_axes,
            drive_profile_code,
            wkc_expected,
            _reserved0,
        ) = _STATUS_HELLO_STRUCT.unpack_from(payload, 0)
        return (
            rtcore_drive_profile_id_to_name(drive_profile_code),
            int(drive_profile_code),
            int(wkc_expected),
        )

    def _parse_motion_state(self, payload: bytes) -> RTCoreExecutionStatus:
        (
            active_mode,
            state,
            active_traj_id,
            current_point_index,
            queue_depth,
            queue_capacity,
            last_event_code,
            underrun_count,
            stale_command_flag,
            motion_done,
            capability_flags,
            active_command_seq,
            last_update_ns,
        ) = _STATUS_MOTION_STATE_STRUCT.unpack_from(payload, 0)
        return RTCoreExecutionStatus(
            active_mode=int(active_mode),
            active_mode_name=rtcore_motion_mode_id_to_name(active_mode) or f"unknown:{int(active_mode)}",
            state=int(state),
            state_name=rtcore_execution_state_id_to_name(state) or f"unknown:{int(state)}",
            active_traj_id=int(active_traj_id),
            current_point_index=None
            if int(current_point_index) == 0xFFFFFFFF
            else int(current_point_index),
            queue_depth=int(queue_depth),
            queue_capacity=int(queue_capacity),
            last_event_code=int(last_event_code),
            underrun_count=int(underrun_count),
            stale_command=bool(stale_command_flag),
            motion_done=bool(motion_done),
            capability_flags=int(capability_flags),
            active_command_seq=int(active_command_seq),
            last_update_ns=int(last_update_ns),
        )

    def _parse_jog_debug_state(self, payload: bytes) -> RTCoreJogDebugStatus:
        unpacked = _STATUS_JOG_DEBUG_STRUCT.unpack_from(payload, 0)
        num_axes = max(0, min(int(unpacked[0]), _GRADIENT_MAX_AXES))
        active_jog = bool(unpacked[1])
        active_jog_axis_mask = int(unpacked[2])
        command_sp_mask = int(unpacked[3])
        have_hold_mask = int(unpacked[4])
        have_jog_target_mask = int(unpacked[5])
        snap_hold_mask = int(unpacked[6])
        latest_cmd_axis_mask = int(unpacked[7])
        latest_cmd_flags = int(unpacked[8])
        last_stop_reason = int(unpacked[9])
        last_stop_axis_mask = int(unpacked[10])
        stop_arrest_mask = int(unpacked[11])
        sample_time_ns = int(unpacked[12])
        active_jog_cmd_seq = int(unpacked[13])
        latest_jog_seq_seen = int(unpacked[14])
        active_jog_deadline_ns = int(unpacked[15])
        latest_cmd_timeout_ns = int(unpacked[16])
        last_stop_time_ns = int(unpacked[17])
        last_stop_cmd_seq = int(unpacked[18])
        axis_start = 19
        feedback_pos_counts = [int(value) for value in unpacked[axis_start : axis_start + _GRADIENT_MAX_AXES]]
        hold_target_counts = [
            int(value) for value in unpacked[axis_start + _GRADIENT_MAX_AXES : axis_start + (_GRADIENT_MAX_AXES * 2)]
        ]
        output_target_counts = [
            int(value) for value in unpacked[axis_start + (_GRADIENT_MAX_AXES * 2) : axis_start + (_GRADIENT_MAX_AXES * 3)]
        ]
        output_target_velocity_counts_per_s = [
            int(value) for value in unpacked[axis_start + (_GRADIENT_MAX_AXES * 3) : axis_start + (_GRADIENT_MAX_AXES * 4)]
        ]
        return RTCoreJogDebugStatus(
            num_axes=num_axes,
            active_jog=active_jog,
            active_jog_axis_mask=active_jog_axis_mask,
            command_sp_mask=command_sp_mask,
            have_hold_mask=have_hold_mask,
            have_jog_target_mask=have_jog_target_mask,
            snap_hold_mask=snap_hold_mask,
            stop_arrest_mask=stop_arrest_mask,
            latest_cmd_axis_mask=latest_cmd_axis_mask,
            latest_cmd_flags=latest_cmd_flags,
            latest_cmd_timeout_ns=latest_cmd_timeout_ns,
            sample_time_ns=sample_time_ns,
            active_jog_cmd_seq=active_jog_cmd_seq,
            latest_jog_seq_seen=latest_jog_seq_seen,
            active_jog_deadline_ns=active_jog_deadline_ns,
            last_stop_reason=last_stop_reason,
            last_stop_reason_name=rtcore_jog_stop_reason_id_to_name(last_stop_reason) or f"unknown:{last_stop_reason}",
            last_stop_axis_mask=last_stop_axis_mask,
            last_stop_time_ns=last_stop_time_ns,
            last_stop_cmd_seq=last_stop_cmd_seq,
            feedback_pos_counts=feedback_pos_counts,
            hold_target_counts=hold_target_counts,
            output_target_counts=output_target_counts,
            output_target_velocity_counts_per_s=output_target_velocity_counts_per_s,
        )

    def _build_axis_config_from_robot_config(self, robot_config: dict) -> Optional[_AxisConfig]:
        counts_per_radian = list(robot_config.get("actuator_counts_per_radian", []))
        counts_per_rev = list(robot_config.get("actuator_encoder_counts_per_rev", []))
        if not counts_per_radian:
            gear_ratios = list(robot_config.get("actuator_gear_ratios", []))
            counts_per_radian = []
            for idx in range(max(len(counts_per_rev), len(gear_ratios))):
                cpr = int(counts_per_rev[idx]) if idx < len(counts_per_rev) else 0
                ratio = float(gear_ratios[idx]) if idx < len(gear_ratios) else 1.0
                if cpr <= 0 or ratio <= 0.0:
                    counts_per_radian.append(0.0)
                else:
                    counts_per_radian.append((float(cpr) * ratio) / (2.0 * 3.141592653589793))

        if not counts_per_radian:
            return None

        signs_raw = list(robot_config.get("actuator_position_signs", []))
        if not signs_raw:
            inverted_ids = set(robot_config.get("inverted_actuator_ids", set()))
            actuator_ids = list(robot_config.get("actuator_ids", []))
            signs_raw = [-1 if actuator_id in inverted_ids else 1 for actuator_id in actuator_ids]

        num_axes = min(
            int(robot_config.get("num_physical_actuators", len(counts_per_radian))),
            len(counts_per_radian),
        )
        if num_axes <= 0:
            return None

        counts_per_unit = [0.0] * _GRADIENT_MAX_AXES
        signs = [0] * _GRADIENT_MAX_AXES
        runtime_counts_per_rev = [0] * _GRADIENT_MAX_AXES
        for idx in range(min(num_axes, _GRADIENT_MAX_AXES)):
            counts_per_unit[idx] = float(counts_per_radian[idx])
            raw_sign = int(signs_raw[idx]) if idx < len(signs_raw) else 1
            signs[idx] = 1 if raw_sign >= 0 else -1
            runtime_counts_per_rev[idx] = int(counts_per_rev[idx]) if idx < len(counts_per_rev) else 0

        return _AxisConfig(
            num_axes=min(num_axes, _GRADIENT_MAX_AXES),
            counts_per_unit=counts_per_unit,
            sign=signs,
            counts_per_rev=runtime_counts_per_rev,
        )

    def _effective_drive_profile_id(self) -> Optional[str]:
        live_profile_id = self.get_live_drive_profile_id()
        if not live_profile_id and self._rt_drive_profile_code:
            live_profile_id = rtcore_drive_profile_id_to_name(self._rt_drive_profile_code)
        return live_profile_id

    def _normalize_feedback_counts_for_axis(self, axis_i: int, raw_counts: int) -> int:
        cfg = self._axis_config
        if cfg is None or axis_i < 0 or axis_i >= int(cfg.num_axes):
            return int(raw_counts)
        if self._effective_drive_profile_id() != "a6ec_ds402":
            return int(raw_counts)
        counts_per_rev = int(cfg.counts_per_rev[axis_i]) if axis_i < len(cfg.counts_per_rev) else 0
        if counts_per_rev <= 0:
            return int(raw_counts)
        half_turn = counts_per_rev // 2
        if half_turn <= 0:
            return int(raw_counts)
        return ((int(raw_counts) + half_turn) % counts_per_rev) - half_turn

    def _display_feedback_counts_for_axis(self, axis_i: int, raw_counts: int) -> int:
        normalized_counts = self._normalize_feedback_counts_for_axis(axis_i, raw_counts)
        cfg = self._axis_config
        if cfg is None or axis_i < 0 or axis_i >= int(cfg.num_axes):
            return int(normalized_counts)
        if self._effective_drive_profile_id() != "a6ec_ds402":
            return int(normalized_counts)
        counts_per_rev = int(cfg.counts_per_rev[axis_i]) if axis_i < len(cfg.counts_per_rev) else 0
        if counts_per_rev <= 0:
            return int(normalized_counts)
        with self._status_lock:
            if axis_i >= len(self._feedback_unwrapped_counts) or axis_i >= len(self._feedback_unwrapped_valid):
                return int(normalized_counts)
            if not self._feedback_unwrapped_valid[axis_i]:
                self._feedback_unwrapped_counts[axis_i] = int(normalized_counts)
                self._feedback_unwrapped_valid[axis_i] = True
                return int(normalized_counts)
            previous = int(self._feedback_unwrapped_counts[axis_i])
            turns = round((float(previous) - float(normalized_counts)) / float(counts_per_rev))
            unwrapped = int(normalized_counts) + (int(turns) * counts_per_rev)
            self._feedback_unwrapped_counts[axis_i] = int(unwrapped)
            return int(unwrapped)

    def _maybe_warn_runtime_axis_config_mismatch(self, runtime_cfg: _AxisConfig) -> None:
        if self._robot_axis_config is None or self._axis_config_mismatch_logged:
            return
        robot_cfg = self._robot_axis_config
        compare_axes = min(robot_cfg.num_axes, runtime_cfg.num_axes, _GRADIENT_MAX_AXES)
        for axis_i in range(compare_axes):
            robot_cpu = float(robot_cfg.counts_per_unit[axis_i])
            runtime_cpu = float(runtime_cfg.counts_per_unit[axis_i])
            robot_sign = int(robot_cfg.sign[axis_i])
            runtime_sign = int(runtime_cfg.sign[axis_i])
            cpu_mismatch = abs(robot_cpu - runtime_cpu) > max(1.0, abs(robot_cpu) * 1e-3)
            sign_mismatch = robot_sign != runtime_sign
            if cpu_mismatch or sign_mismatch:
                print(
                    "[EtherCAT RTCore] WARNING: RTCore axis scaling differs from robot config; "
                    f"keeping robot-config scaling for Python feedback conversion "
                    f"(axis={axis_i} robot_cpu={robot_cpu:.6f} runtime_cpu={runtime_cpu:.6f} "
                    f"robot_sign={robot_sign} runtime_sign={runtime_sign})"
                )
                self._axis_config_mismatch_logged = True
                return

    def _allocate_traj_id(self) -> int:
        traj_id = int(self._next_traj_id)
        self._next_traj_id += 1
        return traj_id

    def _master_offset_for_joint(self, logical_joint_idx: int) -> float:
        if 0 <= logical_joint_idx < len(self._master_offsets_rad):
            return float(self._master_offsets_rad[logical_joint_idx])
        return 0.0

    def _copy_last_joint_setpoint_rad(self) -> list[float]:
        with self._status_lock:
            return [float(value) for value in self._last_joint_setpoint_rad]

    def _store_last_joint_setpoint_rad(self, positions_rad: list[float]) -> None:
        with self._status_lock:
            self._last_joint_setpoint_rad = [float(value) for value in positions_rad]

    def _store_last_joint_index_setpoint(self, joint_i: int, position_rad: float) -> None:
        if joint_i < 0 or joint_i >= self._num_joints:
            return
        with self._status_lock:
            updated = list(self._last_joint_setpoint_rad)
            updated[joint_i] = float(position_rad)
            self._last_joint_setpoint_rad = updated

    def _store_last_axis_target_q(self, axis_i: int, target_axis_q: float) -> None:
        if axis_i < 0 or axis_i >= len(self._axis_to_joint):
            return
        joint_i = self._axis_to_joint[axis_i]
        if joint_i < 0 or joint_i >= self._num_joints:
            return
        logical_q = float(target_axis_q) - self._master_offset_for_joint(joint_i)
        self._store_last_joint_index_setpoint(joint_i, logical_q)

    def _refresh_native_home_offsets_from_metrics(self) -> None:
        try:
            stat = _RTCORE_METRICS_PATH.stat()
        except OSError:
            return
        if stat.st_mtime_ns == self._native_home_metrics_mtime_ns:
            return
        try:
            payload = json.loads(_RTCORE_METRICS_PATH.read_text())
        except Exception:
            return
        axes = payload.get("axes")
        if not isinstance(axes, list):
            return
        offsets = [0] * _GRADIENT_MAX_AXES
        absolute_feedback = [_AbsoluteFeedbackAxisMetrics() for _ in range(_GRADIENT_MAX_AXES)]
        for axis_i, axis in enumerate(axes[:_GRADIENT_MAX_AXES]):
            if not isinstance(axis, dict):
                continue
            try:
                offsets[axis_i] = int(axis.get("native_home_position_offset", 0))
            except Exception:
                offsets[axis_i] = 0
            absolute_feedback[axis_i] = _AbsoluteFeedbackAxisMetrics.from_mapping(
                axis.get("absolute_feedback")
            )
        with self._status_lock:
            self._native_home_offset_counts = offsets
            self._absolute_feedback_by_axis = absolute_feedback
            self._native_home_metrics_mtime_ns = stat.st_mtime_ns

    def _load_rtcore_metrics_snapshot(self) -> dict[str, object] | None:
        try:
            stat = _RTCORE_METRICS_PATH.stat()
            payload = json.loads(_RTCORE_METRICS_PATH.read_text())
        except Exception:
            return None
        if not isinstance(payload, dict):
            return None
        payload["_mtime_ns"] = int(stat.st_mtime_ns)
        return payload

    def _load_rtcore_metrics_axes(self) -> list[dict[str, object]]:
        snapshot = self._load_rtcore_metrics_snapshot()
        if not isinstance(snapshot, dict):
            return []
        axes = snapshot.get("axes")
        if not isinstance(axes, list):
            return []
        return [axis for axis in axes if isinstance(axis, dict)]

    def _absolute_feedback_metrics_for_axis(self, axis_i: int) -> _AbsoluteFeedbackAxisMetrics:
        self._refresh_native_home_offsets_from_metrics()
        with self._status_lock:
            if axis_i < 0 or axis_i >= len(self._absolute_feedback_by_axis):
                return _AbsoluteFeedbackAxisMetrics()
            return self._absolute_feedback_by_axis[axis_i]

    def _normalize_absolute_feedback_metrics(
        self,
        metrics: _AbsoluteFeedbackAxisMetrics,
    ) -> dict[str, object]:
        raw_feedback = metrics.to_dict()
        normalized = drive_profile_registry.normalize_drive_absolute_feedback(
            self._effective_drive_profile_id(),
            raw_feedback,
        )
        return normalized if isinstance(normalized, dict) else raw_feedback

    def _absolute_axis_counts_from_metrics(
        self,
        axis_i: int,
        metrics: _AbsoluteFeedbackAxisMetrics,
    ) -> tuple[int, str] | None:
        del axis_i
        payload = drive_profile_registry.resolve_drive_absolute_feedback_counts(
            self._effective_drive_profile_id(),
            metrics.to_dict(),
        )
        if not isinstance(payload, dict):
            return None
        try:
            counts = int(payload.get("counts"))
        except Exception:
            return None
        source_key = str(payload.get("source_key", "")).strip()
        if not source_key:
            return None
        return int(counts), source_key

    def _absolute_axis_q_from_metrics(
        self,
        axis_i: int,
        metrics: _AbsoluteFeedbackAxisMetrics,
    ) -> tuple[float, str, int] | None:
        counts_and_source = self._absolute_axis_counts_from_metrics(axis_i, metrics)
        if counts_and_source is None:
            return None
        absolute_counts, source = counts_and_source
        axis_q = self._axis_q_from_counts(axis_i, int(absolute_counts))
        if axis_q is None:
            return None
        return float(axis_q), str(source), int(absolute_counts)

    def _absolute_home_anchor_for_joint(self, logical_joint_idx: int) -> dict[str, Any] | None:
        with self._status_lock:
            if logical_joint_idx < 0 or logical_joint_idx >= len(self._absolute_encoder_home_anchors):
                return None
            raw_entry = self._absolute_encoder_home_anchors[logical_joint_idx]
        if not isinstance(raw_entry, dict):
            return None
        try:
            home_anchor_rad = float(raw_entry.get("home_anchor_rad"))
        except Exception:
            return None
        axis_indices_raw = raw_entry.get("axis_indices")
        axis_indices: list[int] = []
        if isinstance(axis_indices_raw, list):
            for value in axis_indices_raw:
                try:
                    axis_indices.append(int(value))
                except Exception:
                    continue
        source_raw = raw_entry.get("source")
        updated_at_raw = raw_entry.get("updated_at")
        updated_by_raw = raw_entry.get("updated_by")
        return {
            "home_anchor_rad": float(home_anchor_rad),
            "source": str(source_raw).strip() if source_raw is not None else None,
            "axis_indices": axis_indices,
            "updated_at": str(updated_at_raw).strip() if updated_at_raw is not None else None,
            "updated_by": str(updated_by_raw).strip() if updated_by_raw is not None else None,
        }

    def _reference_q_before_master_offset_for_axis(
        self,
        axis_i: int,
        raw_counts: int,
    ) -> float | None:
        physical_q = self._axis_q_from_counts(axis_i, int(raw_counts))
        if physical_q is None:
            return None
        return float(physical_q) + self._native_home_offset_q_for_axis(axis_i)

    def _capture_absolute_home_anchor_for_joint(
        self,
        logical_joint_index: int,
        *,
        raw_positions: dict[int, int] | None = None,
        actor: str = "unknown",
    ) -> dict[str, Any] | None:
        joint_i = int(logical_joint_index)
        if joint_i < 0 or joint_i >= self._num_joints:
            return None
        if raw_positions is None:
            with self._status_lock:
                raw_positions = {
                    axis_i: int(self._axis_counts[axis_i])
                    for axis_i in range(min(self._rt_num_axes, len(self._axis_counts)))
                }
        anchor_samples: list[float] = []
        axis_indices: list[int] = []
        source_labels: list[str] = []
        for axis_i, mapped_joint in enumerate(self._axis_to_joint):
            if mapped_joint != joint_i:
                continue
            raw_counts = raw_positions.get(axis_i)
            if raw_counts is None:
                continue
            metrics = self._absolute_feedback_metrics_for_axis(axis_i)
            absolute_result = self._absolute_axis_q_from_metrics(axis_i, metrics)
            reference_q = self._reference_q_before_master_offset_for_axis(axis_i, int(raw_counts))
            if absolute_result is None or reference_q is None:
                continue
            absolute_axis_q, source, _absolute_counts = absolute_result
            anchor_samples.append(float(absolute_axis_q) - float(reference_q))
            axis_indices.append(int(axis_i))
            source_labels.append(str(source))
        if not anchor_samples:
            return None
        source = source_labels[0] if source_labels and len(set(source_labels)) == 1 else "mixed"
        home_anchor_rad = float(sum(anchor_samples) / len(anchor_samples))
        saved_entry = save_absolute_encoder_anchor(
            self._robot_id,
            num_joints=self._num_joints,
            logical_joint_index=joint_i,
            home_anchor_rad=home_anchor_rad,
            source=source,
            axis_indices=axis_indices,
            actor=actor,
        )
        normalized_entry = (
            {
                "home_anchor_rad": float(saved_entry.get("home_anchor_rad", home_anchor_rad)),
                "source": str(saved_entry.get("source", source)).strip() or source,
                "axis_indices": [int(value) for value in list(saved_entry.get("axis_indices", axis_indices))],
                "updated_at": str(saved_entry.get("updated_at", "")).strip() or None,
                "updated_by": str(saved_entry.get("updated_by", actor)).strip() or actor,
            }
            if isinstance(saved_entry, dict)
            else {
                "home_anchor_rad": float(home_anchor_rad),
                "source": source,
                "axis_indices": [int(value) for value in axis_indices],
                "updated_at": None,
                "updated_by": actor,
            }
        )
        if normalized_entry is not None:
            with self._status_lock:
                if 0 <= joint_i < len(self._absolute_encoder_home_anchors):
                    self._absolute_encoder_home_anchors[joint_i] = normalized_entry
        return normalized_entry

    def _native_home_metrics_result(
        self,
        target_axes: list[int],
        *,
        snapshot: dict[str, object],
        allow_statusword_fallback: bool = True,
    ) -> dict[str, object]:
        axes = snapshot.get("axes")
        axis_results: list[dict[str, object]] = []
        any_failed = False
        all_succeeded = True
        if not isinstance(axes, list):
            axes = []
        for axis_i in target_axes:
            axis_payload = axes[axis_i] if axis_i < len(axes) and isinstance(axes[axis_i], dict) else {}
            try:
                state = int(axis_payload.get("native_home_state", 0))
            except Exception:
                state = 0
            try:
                abort_code = int(axis_payload.get("native_home_last_abort_code", 0))
            except Exception:
                abort_code = 0
            try:
                statusword = int(axis_payload.get("statusword", 0))
            except Exception:
                statusword = 0
            try:
                error_code = int(axis_payload.get("error_code", 0))
            except Exception:
                error_code = 0
            try:
                manufacturer_error_code = int(axis_payload.get("manufacturer_error_code", 0))
            except Exception:
                manufacturer_error_code = 0
            effective_state = int(state)
            effective_abort_code = int(abort_code)
            verification_source = "native_home_state"
            if (
                allow_statusword_fallback
                and effective_state not in {1, 2}
                and error_code == 0
                and manufacturer_error_code == 0
                and (statusword & 0x8000) != 0
            ):
                # Keep the command-result semantics aligned with the live driveFaults
                # view: once RTCore's native-home tail is done, a clean live wire-state
                # with HM bit 15 set should override stale last-operation failure fields.
                effective_state = 2
                effective_abort_code = 0
                verification_source = "statusword_bit15"
            axis_results.append(
                {
                    "axis": int(axis_i),
                    "native_home_state": int(effective_state),
                    "native_home_state_name": _native_home_state_name(effective_state),
                    "native_home_state_reported": int(state),
                    "native_home_state_reported_name": _native_home_state_name(state),
                    "native_home_last_abort_code": int(effective_abort_code),
                    "native_home_last_abort_code_hex": f"0x{effective_abort_code & 0xFFFFFFFF:08X}",
                    "native_home_last_abort_code_reported": int(abort_code),
                    "native_home_last_abort_code_reported_hex": f"0x{abort_code & 0xFFFFFFFF:08X}",
                    "statusword": int(statusword),
                    "statusword_hex": f"0x{statusword & 0xFFFF:04X}",
                    "verification_source": verification_source,
                }
            )
            if effective_state == 3:
                any_failed = True
            elif effective_state != 2:
                all_succeeded = False

        terminal_state = "pending"
        if any_failed:
            terminal_state = "failed"
        elif axis_results and all_succeeded:
            terminal_state = "succeeded"

        primary = axis_results[0] if axis_results else {}
        return {
            "verified": terminal_state == "succeeded",
            "timed_out": False,
            "terminal_state": terminal_state,
            "native_home_state": int(primary.get("native_home_state", 0) or 0),
            "native_home_state_name": str(primary.get("native_home_state_name", "idle") or "idle"),
            "native_home_last_abort_code": int(primary.get("native_home_last_abort_code", 0) or 0),
            "native_home_last_abort_code_hex": str(
                primary.get("native_home_last_abort_code_hex", "0x00000000") or "0x00000000"
            ),
            "metrics_time_ns": int(snapshot.get("time_ns", 0) or 0),
            "axis_results": axis_results,
        }

    def _wait_for_native_home_result(
        self,
        axis_mask: int,
        *,
        timeout_s: float,
        min_metrics_time_ns: int = 0,
        min_metrics_mtime_ns: int = 0,
    ) -> dict[str, object]:
        target_axes = [
            axis_i
            for axis_i in range(self._rt_num_axes)
            if (int(axis_mask) & (1 << axis_i)) != 0
        ]
        if not target_axes:
            return {
                "verified": False,
                "timed_out": False,
                "terminal_state": "invalid",
                "native_home_state": 0,
                "native_home_state_name": "idle",
                "native_home_last_abort_code": 0,
                "native_home_last_abort_code_hex": "0x00000000",
                "metrics_time_ns": 0,
                "axis_results": [],
            }

        deadline = time.monotonic() + max(0.0, float(timeout_s))
        last_result: dict[str, object] | None = None
        saw_fresh_snapshot = False
        saw_active_mask = False
        while time.monotonic() <= deadline:
            snapshot = self._load_rtcore_metrics_snapshot()
            if isinstance(snapshot, dict):
                snapshot_time_ns = int(snapshot.get("time_ns", 0) or 0)
                snapshot_mtime_ns = int(snapshot.get("_mtime_ns", 0) or 0)
                if snapshot_time_ns > int(min_metrics_time_ns) or snapshot_mtime_ns > int(min_metrics_mtime_ns):
                    saw_fresh_snapshot = True
                    try:
                        active_mask = int(snapshot.get("native_home_active_axis_mask", 0) or 0)
                    except Exception:
                        active_mask = 0
                    active_for_target = (active_mask & int(axis_mask)) != 0
                    if active_for_target:
                        saw_active_mask = True
                    last_result = self._native_home_metrics_result(
                        target_axes,
                        snapshot=snapshot,
                        allow_statusword_fallback=saw_active_mask and not active_for_target,
                    )
                    if (
                        not active_for_target
                        and str(last_result.get("terminal_state", "pending")) in {"failed", "succeeded"}
                    ):
                        return last_result
            time.sleep(_NATIVE_HOME_WAIT_POLL_INTERVAL_S)
        if not saw_fresh_snapshot:
            return {
                "verified": False,
                "timed_out": True,
                "terminal_state": "pending",
                "native_home_state": 0,
                "native_home_state_name": "idle",
                "native_home_last_abort_code": 0,
                "native_home_last_abort_code_hex": "0x00000000",
                "metrics_time_ns": 0,
                "axis_results": [],
            }
        if last_result is None:
            last_result = {
                "verified": False,
                "timed_out": True,
                "terminal_state": "pending",
                "native_home_state": 0,
                "native_home_state_name": "idle",
                "native_home_last_abort_code": 0,
                "native_home_last_abort_code_hex": "0x00000000",
                "metrics_time_ns": 0,
                "axis_results": [],
            }
        last_result = dict(last_result)
        last_result["timed_out"] = str(last_result.get("terminal_state", "pending")) not in {"failed", "succeeded"}
        return last_result

    def _native_home_offset_q_for_axis(self, axis_i: int) -> float:
        self._refresh_native_home_offsets_from_metrics()
        with self._status_lock:
            if axis_i < 0 or axis_i >= len(self._native_home_offset_counts):
                return 0.0
            native_home_offset_counts = int(self._native_home_offset_counts[axis_i])
        native_home_offset_q = self._axis_q_from_counts(axis_i, native_home_offset_counts)
        return float(native_home_offset_q) if native_home_offset_q is not None else 0.0

    def _axis_q_from_joint_positions(self, positions_rad: list[float]) -> list[float]:
        if len(positions_rad) != self._num_joints:
            raise ValueError(f"Expected {self._num_joints} joint positions, got {len(positions_rad)}")
        axis_q: list[float] = [0.0] * self._rt_num_axes
        for axis_i, joint_i in enumerate(self._axis_to_joint):
            if 0 <= joint_i < len(positions_rad):
                # Queue trajectory points in the same controller/logical frame that
                # `raw_to_joint_positions()` publishes. RTCore converts those queued
                # targets once into raw CSP wire counts (the same 0x6064/0x607A
                # frame the drive uses) by subtracting the persisted native-home
                # offset during trajectory-point latch, so Python must not subtract
                # the drive-home offset here.
                axis_q[axis_i] = float(positions_rad[joint_i]) + self._master_offset_for_joint(joint_i)
        return axis_q

    def _axis_qd_from_joint_velocities(self, joint_velocities_rad_s: list[float]) -> list[float]:
        if len(joint_velocities_rad_s) != self._num_joints:
            raise ValueError(
                f"Expected {self._num_joints} joint velocities, got {len(joint_velocities_rad_s)}"
            )
        axis_qd: list[float] = [0.0] * self._rt_num_axes
        for axis_i, joint_i in enumerate(self._axis_to_joint):
            if 0 <= joint_i < len(joint_velocities_rad_s):
                axis_qd[axis_i] = float(joint_velocities_rad_s[joint_i])
        return axis_qd

    def _display_axis_q_from_raw_feedback_counts(self, axis_i: int, raw_counts: int) -> Optional[float]:
        display_counts = self._display_feedback_counts_for_axis(axis_i, raw_counts)
        return self._axis_q_from_counts(axis_i, int(display_counts))

    def _axis_q_from_counts(self, axis_i: int, raw_counts: int) -> Optional[float]:
        # Convert raw RTCore/drive counts into axis-space radians without
        # display normalization. This preserves the motion frame that RTCore
        # expects when queued targets are converted back into 0x607A counts.
        cfg = self._axis_config
        if cfg is None or axis_i < 0 or axis_i >= int(cfg.num_axes):
            return None
        sign = int(cfg.sign[axis_i])
        counts_per_unit = float(cfg.counts_per_unit[axis_i])
        if sign not in (-1, 1) or counts_per_unit <= 0.0:
            return None
        return float(raw_counts) / (float(sign) * counts_per_unit)

    def _start_status_thread(self) -> None:
        if self._status_thread and self._status_thread.is_alive():
            return

        self._status_stop.clear()
        self._status_thread = threading.Thread(target=self._status_loop, daemon=True)
        self._status_thread.start()

    def _status_loop(self) -> None:
        if self._status_eventfd is None or self._status_shm is None or self._status_hdr is None:
            return

        # Pre-compute ring offsets.
        ring_hdr_offset = self._status_hdr.ring_offset
        ring_entries_offset = ring_hdr_offset + _align_up(_RING_HEADER_STRUCT.size, 8)

        while not self._status_stop.is_set():
            # Wait for status_eventfd (or poll at low rate).
            r, _w, _x = select.select([self._status_eventfd], [], [], 0.25)
            if r:
                try:
                    os.read(self._status_eventfd, 8)  # drain counter
                except Exception:
                    pass

            try:
                self._drain_status_ring(ring_hdr_offset, ring_entries_offset)
            except Exception:
                # Keep best-effort reader alive.
                pass

    def _drain_status_ring(self, ring_hdr_offset: int, ring_entries_offset: int) -> None:
        assert self._status_shm is not None
        assert self._status_hdr is not None

        hdr_bytes = self._status_shm[ring_hdr_offset : ring_hdr_offset + _RING_HEADER_STRUCT.size]
        magic, capacity, msg_bytes, write_idx, read_idx, _dropped, _reserved0 = _RING_HEADER_STRUCT.unpack(hdr_bytes)
        if magic != _MAGIC_RING:
            return
        if capacity == 0 or msg_bytes == 0:
            return

        # Consume available entries.
        while read_idx < write_idx:
            slot = read_idx % capacity
            off = ring_entries_offset + (slot * msg_bytes)
            blob = self._status_shm[off : off + msg_bytes]

            mtype, _mflags, mbytes, _seq, _t_ns = _MSG_HEADER_STRUCT.unpack_from(blob, 0)
            payload = blob[_MSG_HEADER_STRUCT.size : min(msg_bytes, mbytes)]

            if mtype == _MSG_STATUS_HELLO and len(payload) >= _STATUS_HELLO_STRUCT.size:
                try:
                    live_profile_id, drive_profile_code, wkc_expected = self._parse_status_hello(payload)
                    with self._status_lock:
                        self._rt_drive_profile_code = int(drive_profile_code)
                        self._rt_drive_profile_id = live_profile_id
                        self._last_wkc_expected = int(wkc_expected)
                except Exception:
                    pass

            if mtype == _MSG_STATUS_AXIS_CONFIG and len(payload) >= _AXIS_CONFIG_STRUCT.size:
                try:
                    parsed_cfg = self._parse_axis_config(payload)
                    self._runtime_axis_config = parsed_cfg
                    if self._robot_axis_config is None:
                        self._axis_config = parsed_cfg
                    else:
                        self._maybe_warn_runtime_axis_config_mismatch(parsed_cfg)
                    self._axis_config_event.set()
                except Exception:
                    pass

            if mtype == _MSG_STATUS_SNAPSHOT and len(payload) >= _STATUS_SNAPSHOT_HEADER_STRUCT.size:
                # Snapshot layout:
                #   0..39: header
                #   40.. : axes[16] where each axis is 28 bytes; pos_counts at offset 0 in each axis
                (
                    _num_axes,
                    wkc_expected,
                    wkc_actual,
                    master_state,
                    _dc_offset_ns,
                    _cycle_jitter_ns,
                    _topology_hash,
                ) = _STATUS_SNAPSHOT_HEADER_STRUCT.unpack_from(payload, 0)
                with self._status_lock:
                    self._last_wkc_expected = int(wkc_expected)
                    self._last_wkc_actual = int(wkc_actual)
                    self._last_master_state = int(master_state)
                    self._last_status_snapshot_monotonic_s = time.monotonic()
                for axis_i in range(min(self._rt_num_axes, _GRADIENT_MAX_AXES)):
                    axis_off = 40 + axis_i * 28
                    if axis_off + 28 <= len(payload):
                        (
                            pos_counts,
                            torque_raw,
                            statusword,
                            error_code,
                            mode_display,
                            ds402_state,
                            manufacturer_error_code,
                            di_bits,
                            axis_fault_flags,
                            brake_state,
                        ) = struct.unpack_from(
                            "<ihHHBBIIII",
                            payload,
                            axis_off,
                        )
                        self._axis_counts[axis_i] = int(pos_counts)
                        self._axis_torque_raw[axis_i] = int(torque_raw)
                        self._axis_statusword[axis_i] = int(statusword)
                        self._axis_error_code[axis_i] = int(error_code)
                        self._axis_manufacturer_error_code[axis_i] = int(manufacturer_error_code)
                        self._axis_mode_display[axis_i] = int(mode_display)
                        self._axis_ds402_state[axis_i] = int(ds402_state)
                        self._axis_di_bits[axis_i] = int(di_bits)
                        self._axis_fault_flags[axis_i] = int(axis_fault_flags)
                        self._axis_brake_state[axis_i] = int(brake_state)
                self._status_snapshot_event.set()

            if mtype == _MSG_STATUS_MOTION_STATE and len(payload) >= _STATUS_MOTION_STATE_STRUCT.size:
                try:
                    parsed_state = self._parse_motion_state(payload)
                    with self._status_lock:
                        self._execution_status = parsed_state
                except Exception:
                    pass

            if mtype == _MSG_STATUS_JOG_DEBUG and len(payload) >= _STATUS_JOG_DEBUG_STRUCT.size:
                try:
                    parsed_state = self._parse_jog_debug_state(payload)
                    with self._status_lock:
                        self._jog_debug_status = parsed_state
                except Exception:
                    pass

            read_idx += 1

        # Publish new read_idx (consumer-owned).
        # RingHeader layout (u32):
        #   magic(0), capacity(4), msg_bytes(8), write_idx(12), read_idx(16), dropped(20), reserved0(24)
        self._status_shm[ring_hdr_offset + 16 : ring_hdr_offset + 20] = struct.pack("<I", read_idx)

    def supports_realtime_jog(self) -> bool:
        if not self.is_initialized or self._rt_num_axes <= 0:
            return False
        capability_flags = int(getattr(self._execution_status, "capability_flags", 0) or 0)
        return capability_flags == 0 or bool(capability_flags & RTCORE_MOTION_CAP_JOG_COMMAND)

    def supports_joint_velocity_lease_jog(self) -> bool:
        return self.supports_realtime_jog()

    def get_jog_capabilities(self) -> dict[str, object]:
        return {
            "joint_velocity_lease": bool(self.supports_joint_velocity_lease_jog()),
            "realtime_jog_compat": bool(self.supports_realtime_jog()),
            "backend": "ethercat_rtcore",
            "watchdog_source": "rtcore_motor_side",
        }

    def start_joint_velocity_lease_jog(self, timeout_s: float) -> None:
        self.send_realtime_jog_command([0.0] * self._num_joints, timeout_s=timeout_s)

    def update_joint_velocity_lease_jog(
        self,
        joint_velocities_rad_s: list[float],
        timeout_s: float,
    ) -> None:
        self.send_realtime_jog_command(
            joint_velocities_rad_s,
            timeout_s=timeout_s,
        )

    def stop_joint_velocity_lease_jog(self, *, quick_stop: bool = False) -> None:
        if not self._connected:
            return
        self._send_cmd_jog(
            axis_mask=0,
            flags=_JOG_FLAG_STOP | (_JOG_FLAG_QUICK_STOP if quick_stop else 0),
            timeout_ns=0,
            axis_qd=[0.0] * self._rt_num_axes,
        )

    def start_realtime_jog(self, timeout_s: float) -> None:
        self.start_joint_velocity_lease_jog(timeout_s=timeout_s)

    def send_realtime_jog_command(
        self,
        joint_velocities_rad_s: list[float],
        *,
        timeout_s: float,
    ) -> None:
        if not self._connected:
            raise RuntimeError("RTCore not connected (cannot send jog command)")
        if self._rt_num_axes <= 0:
            raise RuntimeError("RTCore did not report a valid num_axes")
        if float(timeout_s) <= 0.0:
            raise ValueError("Realtime jog timeout must be > 0.")

        axis_mask = (1 << self._rt_num_axes) - 1 if self._rt_num_axes > 0 else 0
        timeout_ns = int(max(1, round(float(timeout_s) * 1e9)))
        axis_qd = self._axis_qd_from_joint_velocities(joint_velocities_rad_s)
        self._send_cmd_jog(
            axis_mask=axis_mask,
            flags=_JOG_FLAG_ACTIVE,
            timeout_ns=timeout_ns,
            axis_qd=axis_qd,
        )

    def stop_realtime_jog(self) -> None:
        self.stop_joint_velocity_lease_jog()

    def _write_setpoint(self, positions_rad: list[float], axis_mask: int) -> None:
        raise RuntimeError(
            "Legacy RTCore setpoint-slot writes are disabled. Use trajectory upload/commit paths instead."
        )

