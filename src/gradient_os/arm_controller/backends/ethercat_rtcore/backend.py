from __future__ import annotations

import array
import json
import math
import mmap
import os
import select
import socket
import struct
import threading
import time
from collections.abc import Mapping, Sequence
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Optional

from ....absolute_encoder_anchors import (
    load_absolute_encoder_anchors,
    save_absolute_encoder_anchor,
    save_last_seen_absolute_counts,
)
from ....telemetry.native_home_status import (
    derive_drive_native_truth_validity,
    derive_effective_native_home_status,
    statusword_indicates_valid_native_home_reference,
)
from ...actuator_interface import ActuatorBackend
from ...profiles import registry as drive_profile_registry
from .config import DEFAULT_DRIVE_PROFILE_ID
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

# W1: rate-limit interval for persisting the optional last-seen
# U40.20/.22 sidecar on the anchor file. 5s is short enough that an
# operator-induced drive power cycle is likely to preserve at least one
# fresh sample, and long enough that routine feedback cycles do not
# hammer the disk.
_LAST_SEEN_PERSIST_INTERVAL_S = 5.0

# Standard profile-contract key for the drive's unambiguous multi-turn
# encoder counter, emitted as a signed i64 in MOTOR-frame counts (i.e.,
# counts_per_motor_rev per motor revolution, never wrapping at the
# rotation-mode period). A drive profile opts into the multi-turn-aware
# nearest-turn fold by emitting a mapping under this key from its
# `normalize_absolute_feedback()` with `{"valid": bool, "value": int}`.
#
# Only the A6-EC profile emits this today (by combining U40.20 +
# U40.22 via `signed_i64_pair`; see `ABSOLUTE_FEEDBACK_SOURCES` in
# `profiles/drive/a6ec_ds402.py`), but the contract is profile-agnostic.
# Controllers MUST NOT assume A6-EC-specific semantics from the value
# beyond "signed i64 motor-frame counts, monotonic with physical motor
# rotation, seam-free"; any profile that exposes an equivalent multi-
# turn register gets the multi-turn fold for free.
_PROFILE_MULTI_TURN_COUNTS_KEY = "encoder_multi_turn_counts"

# A6-EC motor-encoder multi-turn counter is 32767 turns wide; anything
# past that without commanded motion during the off-window is considered
# physically impossible and therefore evidence of lost retention rather
# than legitimate drift (see vendor Q3 and commissioning-safety.md).
_MAX_OFF_MOTOR_REVOLUTIONS = 32_767

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
_POWER_TRANSITION_DRIVE_DISARMED_TIMEOUT_S = 1.0
_NATIVE_HOME_WAIT_POLL_INTERVAL_S = 0.05
_NATIVE_HOME_WAIT_TIMEOUT_S = 20.0
_NATIVE_HOME_POST_SETTLE_TIMEOUT_S = 3.0
_NATIVE_HOME_FAILED_STABILIZATION_SNAPSHOTS = 2
_COMMAND_ROUNDTRIP_TOLERANCE_COUNTS = 15.0
_ABSOLUTE_HOME_ANCHOR_STALE_TOLERANCE_COUNTS = 8.0
# Per the plan: a live 6064 disagreement with anchored canonical truth,
# taken modulo RM, above this tolerance fails closed with the explicit
# reason `multi_turn_anchor_inconsistent_with_live_6064`. The A6-EC probe
# work established that stationary reads wander by a few counts, and
# post-restart live soak runs showed occasional 7-9 count spikes even on
# a physically stationary joint, so keep this gate comfortably above the
# observed jitter band while still catching real sub-shaft-turn drift.
_SHAFT_FRAME_CONSISTENCY_TOLERANCE_COUNTS = 16.0
# Per-trajectory safety cage (2026-04-17 J6 incident hardening).
#
# Background: the write path folds each queued point's canonical_q to the
# nearest shaft turn of live 6064, so the wire-frame delta between
# consecutive points should be small and the FIRST point should land
# essentially on top of live 6064. Any violation of those invariants is
# a turn-selection bug (host fold flip, drive-side rotation-direction
# config, or stale live counts) that would otherwise show up as a full
# shaft revolution of physical motion from a tiny operator command.
#
# Both bounds are expressed in joint-space radians so the threshold means
# the same operator-facing motion envelope on every axis regardless of
# gear ratio or counts-per-unit.
#
# FIRST-point deviation from live 6064: tight, because the upload must
# start from where the drive actually is. 0.35 rad ~= 20 deg of joint
# space is wider than any legitimate read-vs-live skew we have ever
# observed while allowing a tiny "commanded hold offset" at commit time
# if the controller nudged the setpoint forward between snapshot and
# commit. A full shaft revolution is 6.28 rad, so this bound rejects the
# pathological case by a factor of ~18.
_TRAJECTORY_MAX_FIRST_POINT_DEVIATION_FROM_LIVE_RAD = 0.35
# Per-point step between consecutive uploaded points, joint-space rad.
# Long bounded moves are planned from many small steps (the bounded
# path generator already caps step size via max_motor_rpm and 100 Hz
# quantization), so each per-point delta sits well under this bound.
# A mid-trajectory fold flip would produce a step of roughly a full
# shaft revolution (6.28 rad), which this gate rejects by ~18x.
_TRAJECTORY_MAX_PER_POINT_STEP_RAD = 0.35
# Synthesized RTCore abort code for "HM35 precondition never saw a drive-
# confirmed disarmed statusword". Must match
# NATIVE_HOME_ABORT_DISARM_PRECONDITION_TIMEOUT in ipc_v1.hpp.
_NATIVE_HOME_ABORT_DISARM_PRECONDITION_TIMEOUT = 0xF1000001
# DS402 "state name" cells we accept as "drive is not actively driving the
# motor" for the Vendor-Q2 "stationary and inactive" precondition. Both host
# and RTCore use the same decode, but we keep the list local so the Python
# contract is explicit.
_DS402_OP_ENABLED_STATES = {5, 6}  # OperationEnabled, QuickStopActive
_TWO_PI = 2.0 * 3.141592653589793
_RTCORE_METRICS_PATH = Path("/run/gradient-rt-motion/metrics.json")

_POST_HOME_TRUTH_RETRYABLE_REASONS = {
    "raw_feedback_missing",
    "truth_snapshot_unavailable",
    "drive_native_statusword_unavailable",
    "drive_native_fault_present",
    "drive_native_manufacturer_fault_present",
    "drive_native_slave_offline",
    "drive_native_slave_not_operational",
    "drive_native_native_home_active",
}

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


def _post_home_truth_reason_is_retryable(reason: object) -> bool:
    normalized = str(reason or "").strip().lower()
    return normalized in _POST_HOME_TRUTH_RETRYABLE_REASONS


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
        # Continuous 607A command emission is now driven by the active
        # drive profile's `MOTION_FEEDBACK_CONFIG["command_counts_wrap"]`
        # flag (False => continuous, True => wrapped) and plumbed into
        # RTCore via `GRADIENT_RT_COMMAND_WRAP_AXIS_MASK`. The historical
        # `GRADIENT_RTCORE_EXPERIMENTAL_CONTINUOUS_607A_JOINTS` env var
        # only affected the Python-side fold, not RTCore's wire output,
        # so it was a no-op on actual motion and has been retired.

        self._sock: Optional[socket.socket] = None

        self._cmd_shm_fd: Optional[int] = None
        self._status_shm_fd: Optional[int] = None
        self._cmd_eventfd: Optional[int] = None
        self._status_eventfd: Optional[int] = None

        self._cmd_shm: Optional[mmap.mmap] = None
        self._status_shm: Optional[mmap.mmap] = None

        self._cmd_hdr: Optional[_ShmHeader] = None
        self._status_hdr: Optional[_ShmHeader] = None
        configured_drive_profile = str(robot_config.get("configured_drive_profile_id", "") or "").strip().lower()
        self._configured_drive_profile_id: Optional[str] = configured_drive_profile or None
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
        # Rate-limit last-seen sidecar persistence so we do not hammer
        # the anchor file on every feedback cycle. Keyed by logical
        # joint index; value is the `time.time()` wall-clock timestamp
        # of the last successful persist.
        self._last_seen_persist_last_wall_s: dict[int, float] = {}
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

        # Latest commanded joint positions (radians) for command bookkeeping and
        # diagnostics. These values must never masquerade as live encoder truth.
        self._last_joint_setpoint_rad: list[float] = [0.0] * self._num_joints

        self._status_thread: Optional[threading.Thread] = None
        self._status_stop = threading.Event()

    # -------------------------------------------------------------------------
    # ActuatorBackend required API
    # -------------------------------------------------------------------------

    def initialize(self) -> bool:
        try:
            ok = self._connect_ipc()
            if ok and self._absolute_home_anchor_required():
                try:
                    self._bootstrap_missing_absolute_home_anchors(
                        actor="ethercat_rtcore:startup_alignment",
                    )
                except Exception as exc:
                    print(f"[EtherCAT RTCore] WARNING: absolute-home anchor bootstrap failed: {exc}")
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
        status_lock = getattr(self, "_status_lock", None)
        if status_lock is None:
            return getattr(self, "_rt_drive_profile_id", None)
        with status_lock:
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

    def logical_joint_indices_to_axis_mask(self, logical_joint_indices: Sequence[int]) -> int:
        if self._rt_num_axes <= 0:
            return 0
        requested = {int(joint_i) for joint_i in logical_joint_indices}
        if not requested:
            return 0
        axis_mask = 0
        for axis_i, mapped_joint in enumerate(self._axis_to_joint):
            if mapped_joint in requested:
                axis_mask |= 1 << axis_i
        return int(axis_mask)

    def get_power_transition_snapshot(self) -> dict[str, Any]:
        with self._status_lock:
            execution_status = self._execution_status
            jog_status = self._jog_debug_status
            axis_error_code = list(self._axis_error_code[: self._rt_num_axes])
            axis_fault_flags = list(self._axis_fault_flags[: self._rt_num_axes])
            axis_counts = list(self._axis_counts[: self._rt_num_axes])
            axis_ds402_state = list(self._axis_ds402_state[: self._rt_num_axes])
            axis_statusword = list(self._axis_statusword[: self._rt_num_axes])
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
        feedback_truth_available = False
        feedback_truth_unavailable_axes: list[int] = []
        feedback_truth_unavailable_joints: list[int] = []
        feedback_truth_reasons: list[str] = []
        feedback_truth_statuswords: list[str] = []
        if axis_config is not None and self._connected and self._rt_num_axes > 0:
            raw_feedback = {
                axis_i: axis_counts[axis_i]
                for axis_i in range(min(len(axis_counts), self._rt_num_axes))
            }
            truth_snapshot = self._canonical_joint_positions_from_raw_feedback(
                raw_feedback,
            )
            feedback_truth_available = bool(truth_snapshot.get("truth_available", False))
            if feedback_truth_available:
                positions = truth_snapshot.get("joint_positions_rad")
                if isinstance(positions, list):
                    live_feedback_joint_positions = [float(value) for value in positions]
            feedback_truth_unavailable_axes = [
                int(value)
                for value in list(truth_snapshot.get("truth_unavailable_axes", []))
                if isinstance(value, (int, float))
            ]
            feedback_truth_unavailable_joints = [
                int(value)
                for value in list(truth_snapshot.get("truth_unavailable_joints", []))
                if isinstance(value, (int, float))
            ]
            axis_truth_details = truth_snapshot.get("axis_absolute_feedback")
            if isinstance(axis_truth_details, list):
                for detail in axis_truth_details:
                    if not isinstance(detail, dict) or bool(detail.get("truth_available", False)):
                        continue
                    reason = str(detail.get("truth_reason", "")).strip()
                    if reason:
                        feedback_truth_reasons.append(reason)
                    statusword_hex = str(detail.get("statusword_hex", "")).strip()
                    if statusword_hex:
                        feedback_truth_statuswords.append(statusword_hex)
            feedback_truth_reasons = sorted(set(feedback_truth_reasons))
            feedback_truth_statuswords = sorted(set(feedback_truth_statuswords))

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

        per_axis_drive_disarmed: list[bool] = [
            int(state) not in _DS402_OP_ENABLED_STATES for state in axis_ds402_state
        ]
        per_axis_statusword_hex: list[str] = [
            f"0x{int(sw) & 0xFFFF:04x}" for sw in axis_statusword
        ]
        drive_disarmed_all = bool(
            per_axis_drive_disarmed and all(per_axis_drive_disarmed)
        )
        drive_op_enabled_axes = [
            axis_i
            for axis_i, disarmed in enumerate(per_axis_drive_disarmed)
            if not disarmed
        ]
        return {
            "connected": bool(self._connected),
            "feedback_ready": bool(feedback_ready),
            "feedback_synchronized": bool(feedback_synchronized),
            "live_feedback_joint_positions_rad": list(live_feedback_joint_positions),
            "feedback_truth_available": bool(feedback_truth_available),
            "feedback_truth_unavailable_axes": feedback_truth_unavailable_axes,
            "feedback_truth_unavailable_joints": feedback_truth_unavailable_joints,
            "feedback_truth_reasons": feedback_truth_reasons,
            "feedback_truth_statuswords": feedback_truth_statuswords,
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
            "per_axis_drive_disarmed": per_axis_drive_disarmed,
            "per_axis_ds402_state": [int(state) for state in axis_ds402_state],
            "per_axis_statusword_hex": per_axis_statusword_hex,
            "drive_disarmed_all": drive_disarmed_all,
            "drive_op_enabled_axes": drive_op_enabled_axes,
            "power_up_ready": bool(not motion_active and not stale_command and not faulted_axis_indices and feedback_synchronized),
        }

    def _power_transition_neutrality_satisfied(
        self,
        snapshot: dict[str, Any],
        *,
        require_drive_disarmed: bool,
        require_drive_disarmed_axis_mask: int,
    ) -> bool:
        if not bool(snapshot.get("motion_intent_cleared", False)):
            return False
        if not require_drive_disarmed:
            return True
        per_axis = snapshot.get("per_axis_drive_disarmed")
        if not isinstance(per_axis, list):
            return False
        if require_drive_disarmed_axis_mask > 0:
            for axis_i in range(len(per_axis)):
                if (require_drive_disarmed_axis_mask & (1 << axis_i)) == 0:
                    continue
                if not bool(per_axis[axis_i]):
                    return False
            return True
        return bool(snapshot.get("drive_disarmed_all", False))

    def wait_for_power_transition_neutral(
        self,
        *,
        timeout_s: float | None = None,
        require_drive_disarmed: bool = False,
        require_drive_disarmed_axis_mask: int = 0,
    ) -> dict[str, Any]:
        latest = self.get_power_transition_snapshot()
        resolved_timeout_s = (
            _POWER_TRANSITION_DEFAULT_TIMEOUT_S
            if timeout_s is None
            else max(0.0, float(timeout_s))
        )
        latest["drive_disarmed_required"] = bool(require_drive_disarmed)
        latest["drive_disarmed_required_axis_mask"] = int(require_drive_disarmed_axis_mask)
        if resolved_timeout_s <= 0.0 or self._power_transition_neutrality_satisfied(
            latest,
            require_drive_disarmed=require_drive_disarmed,
            require_drive_disarmed_axis_mask=int(require_drive_disarmed_axis_mask),
        ):
            latest["wait_timed_out"] = False
            return latest

        deadline = time.monotonic() + resolved_timeout_s
        while time.monotonic() <= deadline:
            time.sleep(_POWER_TRANSITION_WAIT_POLL_INTERVAL_S)
            latest = self.get_power_transition_snapshot()
            latest["drive_disarmed_required"] = bool(require_drive_disarmed)
            latest["drive_disarmed_required_axis_mask"] = int(require_drive_disarmed_axis_mask)
            if self._power_transition_neutrality_satisfied(
                latest,
                require_drive_disarmed=require_drive_disarmed,
                require_drive_disarmed_axis_mask=int(require_drive_disarmed_axis_mask),
            ):
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
        require_drive_disarmed: bool = False,
        require_drive_disarmed_axis_mask: int = 0,
    ) -> dict[str, Any]:
        if not self._connected:
            snapshot = self.get_power_transition_snapshot()
            snapshot["waited_for_idle"] = bool(wait_for_idle)
            snapshot["wait_timed_out"] = False
            snapshot["drive_disarmed_required"] = bool(require_drive_disarmed)
            snapshot["drive_disarmed_required_axis_mask"] = int(
                require_drive_disarmed_axis_mask
            )
            return snapshot

        try:
            self.abort_trajectory()
        except Exception as exc:
            print(f"[EtherCAT RTCore] WARNING: trajectory abort before power transition failed: {exc}")
        try:
            self.stop_joint_velocity_lease_jog(quick_stop=quick_stop)
        except Exception as exc:
            print(f"[EtherCAT RTCore] WARNING: jog stop before power transition failed: {exc}")

        # If caller requires drive-confirmed disarm, also send explicit RTCore
        # disable commands for the targeted axes so the cyclic loop starts
        # driving the DS402 state machine away from OperationEnabled. Without
        # this step the host would only be "waiting", not "asking".
        if require_drive_disarmed and require_drive_disarmed_axis_mask > 0:
            try:
                self._send_cmd_axis_disable(
                    axis_mask=int(require_drive_disarmed_axis_mask)
                )
            except Exception as exc:
                print(
                    "[EtherCAT RTCore] WARNING: axis disable before drive-disarm wait"
                    f" failed: {exc}"
                )

        if wait_for_idle:
            snapshot = self.wait_for_power_transition_neutral(
                timeout_s=timeout_s,
                require_drive_disarmed=require_drive_disarmed,
                require_drive_disarmed_axis_mask=int(require_drive_disarmed_axis_mask),
            )
        else:
            time.sleep(0.02)
            snapshot = self.get_power_transition_snapshot()
            snapshot["wait_timed_out"] = False
            snapshot["drive_disarmed_required"] = bool(require_drive_disarmed)
            snapshot["drive_disarmed_required_axis_mask"] = int(
                require_drive_disarmed_axis_mask
            )
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
        # Pre-commit safety cage (2026-04-17 J6 incident). For every axis the
        # queued 607A wire-frame target must (a) stay close to the drive's
        # live 6064 at upload time, and (b) not step by more than a small
        # per-point joint-space bound between consecutive points. See
        # _enforce_trajectory_wire_frame_safety for the rationale and the
        # explicit bound values.
        previous_axis_counts: list[int | None] = [None] * self._rt_num_axes
        initial_live_counts: list[int | None] = [None] * self._rt_num_axes
        for axis_i in range(self._rt_num_axes):
            initial_live_counts[axis_i] = self._live_reference_counts_for_axis(axis_i)
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
            self._enforce_trajectory_wire_frame_safety(
                axis_q=axis_q,
                axis_mask=int(axis_mask),
                previous_axis_counts=previous_axis_counts,
                initial_live_counts=initial_live_counts,
                point_index=int(idx),
                traj_id=int(traj_id),
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
        axis_mask: Optional[int] = None,
    ) -> RTCoreExecutionStatus:
        if joint_path is None or len(joint_path) == 0:
            raise ValueError("joint_path must not be empty")
        timing = self.resolve_trajectory_frequency(frequency)
        requested_frequency_hz = int(timing["requested_frequency_hz"])
        frequency_hz = int(timing["effective_frequency_hz"])
        step_ns = int(timing["step_ns"])
        resolved_axis_mask = self._all_axis_mask() if axis_mask is None else (int(axis_mask) & self._all_axis_mask())
        if resolved_axis_mask == 0:
            raise ValueError("RTCore trajectory requires at least one mapped axis")
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
        traj_id = self.begin_trajectory(axis_mask=resolved_axis_mask, expected_points=len(joint_path))
        points = []
        for idx, (q, qd) in enumerate(zip(joint_path, joint_velocities, strict=True)):
            points.append(
                {
                    "positions_rad": list(q),
                    "qd": self._axis_qd_from_joint_velocities(list(qd)),
                    "axis_mask": resolved_axis_mask,
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
        timeout_status = final_status if final_status is not None else last_status
        raise TimeoutError(
            "Timed out waiting for RTCore trajectory"
            f" {traj_id} to complete"
            f" (saw_target={saw_target_trajectory}"
            f" state={getattr(timeout_status, 'state_name', None)}"
            f" active_traj_id={getattr(timeout_status, 'active_traj_id', None)}"
            f" queue_depth={getattr(timeout_status, 'queue_depth', None)}"
            f" motion_done={getattr(timeout_status, 'motion_done', None)}"
            f" active_command_seq={getattr(timeout_status, 'active_command_seq', None)}"
            f" submitted_command_seq={submitted_command_seq})"
        )

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
            extra: dict[str, object] | None = None,
        ) -> dict[str, object]:
            abort_code = int(native_home_last_abort_code)
            payload = {
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
            if isinstance(extra, dict):
                payload.update(extra)
            return payload

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
            disarm_timeout_s = self._env_float(
                "GRADIENT_RTCORE_NATIVE_HOME_DISARM_TIMEOUT_S",
                _POWER_TRANSITION_DRIVE_DISARMED_TIMEOUT_S,
            )
            precondition_snapshot = self.prepare_for_power_transition(
                wait_for_idle=True,
                timeout_s=disarm_timeout_s,
                require_drive_disarmed=True,
                require_drive_disarmed_axis_mask=int(axis_mask),
            )
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

        precondition_snapshot_mapping: dict[str, Any] = (
            precondition_snapshot if isinstance(precondition_snapshot, dict) else {}
        )
        if bool(precondition_snapshot_mapping.get("wait_timed_out", False)):
            # Drive never left OperationEnabled within the disarm window; do not
            # start HM35 at all, per vendor Q2 "stationary and inactive".
            op_enabled_axes = [
                int(axis_i)
                for axis_i in list(precondition_snapshot_mapping.get("drive_op_enabled_axes", []))
                if (int(axis_i) & 0xFFFF) < 32 and (axis_mask & (1 << int(axis_i))) != 0
            ]
            statuswords = list(precondition_snapshot_mapping.get("per_axis_statusword_hex", []))
            op_statuswords = [
                statuswords[axis_i]
                for axis_i in op_enabled_axes
                if 0 <= axis_i < len(statuswords)
            ]
            print(
                "[EtherCAT RTCore] WARNING: native-home disarm precondition timeout"
                f" joint={joint_i + 1} axis_mask=0x{axis_mask:x}"
                f" op_enabled_axes={op_enabled_axes}"
                f" statuswords={op_statuswords}"
            )
            return _result(
                accepted=False,
                verified=False,
                code="NATIVE_HOME_DISARM_PRECONDITION_TIMEOUT",
                message=(
                    "Drive did not leave OperationEnabled within the disarm"
                    f" precondition window; refusing to start HM35."
                    f" op_enabled_axes={op_enabled_axes} statuswords={op_statuswords}"
                ),
                joint=joint_i + 1,
                axis_mask=axis_mask,
                native_home_last_abort_code=_NATIVE_HOME_ABORT_DISARM_PRECONDITION_TIMEOUT,
                extra={
                    "disarm_precondition_timed_out": True,
                    "drive_op_enabled_axes": op_enabled_axes,
                    "drive_op_enabled_statuswords": op_statuswords,
                },
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
        def _complete_verified_native_home(
            verified_result: dict[str, object],
            *,
            verification_retry_after_timeout: bool = False,
        ) -> dict[str, object]:
            anchor_required = self._absolute_home_anchor_required()
            post_home_detail: dict[str, object] = {
                "absolute_home_anchor_required": bool(anchor_required),
                "absolute_home_anchor_capture_succeeded": False,
                "absolute_home_anchor_refresh_ok": False,
                "post_home_verification_retry_after_timeout": bool(verification_retry_after_timeout),
            }
            try:
                settle_timeout_s = self._env_float(
                    "GRADIENT_RTCORE_NATIVE_HOME_POST_SETTLE_TIMEOUT_S",
                    _NATIVE_HOME_POST_SETTLE_TIMEOUT_S,
                )
                def _record_post_home_settle_result(settle_result: dict[str, object]) -> str:
                    post_home_detail["post_home_settle_ok"] = bool(settle_result.get("ok", False))
                    post_home_detail["post_home_settle_timed_out"] = bool(settle_result.get("timed_out", False))
                    post_home_detail["post_home_settle_timeout_s"] = float(settle_timeout_s)
                    post_home_detail["post_home_settle_metrics_time_ns"] = int(
                        settle_result.get("metrics_time_ns", 0) or 0
                    )
                    post_home_detail["post_home_settle_hard_failure"] = bool(
                        settle_result.get("hard_failure", False)
                    )
                    settle_reason = str(settle_result.get("failure_reason", "") or "").strip()
                    if settle_reason:
                        post_home_detail["post_home_settle_reason"] = settle_reason
                    else:
                        post_home_detail.pop("post_home_settle_reason", None)
                    settle_axis_results = settle_result.get("axis_results")
                    if isinstance(settle_axis_results, list) and settle_axis_results:
                        primary_settle = settle_axis_results[0]
                        for key in (
                            "axis",
                            "native_home_state",
                            "native_home_state_name",
                            "native_home_last_abort_code",
                            "native_home_last_abort_code_hex",
                            "statusword",
                            "statusword_hex",
                            "statusword_fault",
                            "error_code",
                            "error_code_hex",
                            "manufacturer_error_code",
                            "manufacturer_error_code_hex",
                            "slave_online",
                            "slave_operational",
                            "native_home_active",
                            "failure_reason",
                            "clean",
                        ):
                            value = primary_settle.get(key)
                            if value is not None:
                                post_home_detail[f"post_home_settle_{key}"] = value
                    return settle_reason

                def _raise_for_post_home_settle_result(
                    settle_result: dict[str, object],
                    settle_reason: str,
                ) -> None:
                    if bool(settle_result.get("ok", False)):
                        return
                    if bool(settle_result.get("hard_failure", False)):
                        raise RuntimeError(
                            "Post-home settle window detected a drive fault or unavailable axis"
                            + (f" ({settle_reason})." if settle_reason else ".")
                        )
                    raise TimeoutError(
                        "Post-home settle window did not stay clean long enough"
                        + (f" ({settle_reason})." if settle_reason else ".")
                    )

                def _refresh_post_home_validation() -> tuple[dict[str, object], str]:
                    self._refresh_native_home_offsets_from_metrics()
                    raw_positions = self.sync_read_positions()
                    if not raw_positions:
                        raise RuntimeError("Live raw feedback was unavailable after native home verification.")
                    post_home_detail.pop("absolute_home_anchor_capture_skipped", None)
                    post_home_detail["absolute_home_anchor_capture_succeeded"] = False
                    if anchor_required:
                        captured_anchor = self._capture_absolute_home_anchor_for_joint(
                            joint_i,
                            raw_positions=raw_positions,
                            actor=f"ethercat_rtcore:joint{joint_i + 1}:native_home",
                            reference_mode="raw",
                        )
                        if captured_anchor is None:
                            raise RuntimeError(
                                "Could not capture a post-home absolute encoder anchor from live feedback."
                            )
                        post_home_detail["absolute_home_anchor_capture_succeeded"] = True
                        post_home_detail["absolute_home_anchor_rad"] = float(
                            captured_anchor.get("home_anchor_rad", 0.0)
                        )
                        if captured_anchor.get("source") is not None:
                            post_home_detail["absolute_home_anchor_source"] = str(
                                captured_anchor.get("source")
                            )
                    else:
                        post_home_detail["absolute_home_anchor_capture_skipped"] = True
                    validation = self._absolute_home_anchor_validation_for_joint(
                        joint_i,
                        raw_positions=raw_positions,
                        reference_mode="raw",
                    )
                    post_home_detail["absolute_home_anchor_refresh_ok"] = (
                        bool(validation.get("ok", False))
                        if anchor_required
                        else True
                    )
                    post_home_detail["post_home_truth_available"] = bool(
                        validation.get("truth_available", False)
                    )
                    truth_reason = str(validation.get("truth_reason", "") or "").strip()
                    if truth_reason:
                        post_home_detail["post_home_truth_reason"] = truth_reason
                    else:
                        post_home_detail.pop("post_home_truth_reason", None)
                    post_home_detail["post_home_truth_source"] = str(
                        self._position_semantics_source()
                        if self._drive_native_ratio_enabled()
                        else "absolute_encoder_anchor"
                    )
                    for key in (
                        "post_home_axis",
                        "post_home_logical_joint",
                        "post_home_command_roundtrip_reference_error_counts",
                        "post_home_command_roundtrip_reference_error_rad",
                        "post_home_shaft_frame_consistent",
                        "post_home_shaft_frame_mod_rm_delta_counts",
                        "post_home_shaft_frame_mod_rm_delta_rad",
                        "post_home_shaft_frame_tolerance_counts",
                        "post_home_shaft_frame_tolerance_rad",
                        "post_home_shaft_frame_period_counts",
                        "post_home_shaft_frame_wrap_turns",
                    ):
                        post_home_detail.pop(key, None)
                    for key in (
                        "axis",
                        "logical_joint",
                        "command_roundtrip_reference_error_counts",
                        "command_roundtrip_reference_error_rad",
                        "shaft_frame_consistent",
                        "shaft_frame_mod_rm_delta_counts",
                        "shaft_frame_mod_rm_delta_rad",
                        "shaft_frame_tolerance_counts",
                        "shaft_frame_tolerance_rad",
                        "shaft_frame_period_counts",
                        "shaft_frame_wrap_turns",
                    ):
                        value = validation.get(key)
                        if value is not None:
                            post_home_detail[f"post_home_{key}"] = value
                    return validation, truth_reason

                validation, truth_reason = _refresh_post_home_validation()
                settle_result: dict[str, object] | None = None
                if (
                    not bool(validation.get("ok", False))
                    and _post_home_truth_reason_is_retryable(truth_reason)
                ):
                    post_home_detail["post_home_truth_retry_after_settle"] = True
                    settle_result = self._wait_for_native_home_post_settle_result(
                        axis_mask,
                        timeout_s=settle_timeout_s,
                        min_metrics_time_ns=int(wait_result.get("metrics_time_ns", 0) or 0),
                    )
                    settle_reason = _record_post_home_settle_result(settle_result)
                    _raise_for_post_home_settle_result(settle_result, settle_reason)
                    validation, truth_reason = _refresh_post_home_validation()
                if not bool(validation.get("ok", False)):
                    reason_suffix = (
                        f" ({truth_reason})"
                        if truth_reason
                        else ""
                    )
                    if anchor_required:
                        raise RuntimeError(
                            "Refreshed absolute-home anchor did not validate against the live command frame"
                            f"{reason_suffix}."
                        )
                    raise RuntimeError(
                        "Drive-native truth did not validate against the live command frame"
                        f"{reason_suffix}."
                    )
                if settle_result is None:
                    settle_result = self._wait_for_native_home_post_settle_result(
                        axis_mask,
                        timeout_s=settle_timeout_s,
                        min_metrics_time_ns=int(wait_result.get("metrics_time_ns", 0) or 0),
                    )
                    settle_reason = _record_post_home_settle_result(settle_result)
                    _raise_for_post_home_settle_result(settle_result, settle_reason)
            except Exception as exc:
                failure_text = str(exc).strip() or "unknown post-home anchor refresh failure"
                if isinstance(exc, TimeoutError):
                    post_home_detail["post_home_settle_error"] = failure_text
                    print(
                        "[EtherCAT RTCore] WARNING: native drive-home reached a verified terminal state,"
                        f" but the post-home settle window did not complete cleanly for joint={joint_i + 1}: {failure_text}"
                    )
                    return _result(
                        accepted=True,
                        verified=False,
                        timed_out=True,
                        code="NATIVE_HOME_POST_HOME_SETTLE_PENDING",
                        message=(
                            "Drive-native commissioning home reached a verified terminal state and refreshed a "
                            "coherent absolute-home anchor, but RTCore did not observe a clean post-home settle "
                            "window before the verification deadline. Keep the axis disabled and confirm the live "
                            "drive state before powering it back up."
                        ),
                        joint=joint_i + 1,
                        axis_mask=axis_mask,
                        terminal_state=str(wait_result.get("terminal_state", "succeeded")),
                        native_home_state=int(wait_result.get("native_home_state", 2)),
                        native_home_last_abort_code=int(wait_result.get("native_home_last_abort_code", 0)),
                        metrics_time_ns=int(
                            post_home_detail.get(
                                "post_home_settle_metrics_time_ns",
                                wait_result.get("metrics_time_ns", 0),
                            )
                        ),
                        extra=post_home_detail,
                    )
                if post_home_detail.get("post_home_settle_ok") is False:
                    post_home_detail["post_home_settle_error"] = failure_text
                    print(
                        "[EtherCAT RTCore] WARNING: native drive-home reached a verified terminal state,"
                        f" but the post-home settle window faulted for joint={joint_i + 1}: {failure_text}"
                    )
                    return _result(
                        accepted=True,
                        verified=False,
                        code="NATIVE_HOME_POST_HOME_SETTLE_FAILED",
                        message=(
                            "Drive-native commissioning home reached a verified terminal state and refreshed a "
                            "coherent absolute-home anchor, but the axis did not stay clean during the post-home "
                            "settle window. Keep the axis disabled and inspect the live drive fault before "
                            "powering it back up."
                        ),
                        joint=joint_i + 1,
                        axis_mask=axis_mask,
                        terminal_state=str(wait_result.get("terminal_state", "succeeded")),
                        native_home_state=int(wait_result.get("native_home_state", 2)),
                        native_home_last_abort_code=int(wait_result.get("native_home_last_abort_code", 0)),
                        metrics_time_ns=int(
                            post_home_detail.get(
                                "post_home_settle_metrics_time_ns",
                                wait_result.get("metrics_time_ns", 0),
                            )
                        ),
                        extra=post_home_detail,
                    )
                post_home_detail["post_home_anchor_refresh_error"] = failure_text
                print(
                    "[EtherCAT RTCore] WARNING: native drive-home reached a verified terminal state,"
                    f" but post-home anchor refresh failed for joint={joint_i + 1}: {failure_text}"
                )
                return _result(
                    accepted=True,
                    verified=False,
                    code="NATIVE_HOME_ANCHOR_REFRESH_FAILED",
                    message=(
                        "Drive-native commissioning home reached a verified terminal state, but GradientOS "
                        "could not refresh a coherent absolute-home anchor from live feedback. Keep the "
                        "axis disabled and inspect the live pose before powering it back up."
                    ),
                    joint=joint_i + 1,
                    axis_mask=axis_mask,
                    terminal_state=str(wait_result.get("terminal_state", "succeeded")),
                    native_home_state=int(wait_result.get("native_home_state", 2)),
                    native_home_last_abort_code=int(wait_result.get("native_home_last_abort_code", 0)),
                    metrics_time_ns=int(wait_result.get("metrics_time_ns", 0)),
                    extra=post_home_detail,
                )
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
                    terminal_state=str(verified_result.get("terminal_state", "succeeded")),
                    native_home_state=int(verified_result.get("native_home_state", 2)),
                    native_home_last_abort_code=int(
                        verified_result.get("native_home_last_abort_code", 0)
                    ),
                    metrics_time_ns=int(verified_result.get("metrics_time_ns", 0)),
                extra=post_home_detail,
            )
        if bool(wait_result.get("verified", False)):
            return _complete_verified_native_home(wait_result)

        if bool(wait_result.get("timed_out", False)):
            late_snapshot = self._load_rtcore_metrics_snapshot()
            if isinstance(late_snapshot, dict):
                target_axes = [
                    axis_i
                    for axis_i, mapped_joint in enumerate(self._axis_to_joint)
                    if mapped_joint == joint_i
                ]
                late_result = self._native_home_metrics_result(
                    target_axes,
                    snapshot=late_snapshot,
                )
                if bool(late_result.get("verified", False)):
                    print(
                        "[EtherCAT RTCore] Native drive-home verified after timeout via late metrics retry:"
                        f" joint={joint_i + 1} axis_mask=0x{axis_mask:x}"
                    )
                    return _complete_verified_native_home(
                        late_result,
                        verification_retry_after_timeout=True,
                    )
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
        abort_is_disarm_timeout = (
            (abort_code & 0xFFFFFFFF) == _NATIVE_HOME_ABORT_DISARM_PRECONDITION_TIMEOUT
        )
        print(
            "[EtherCAT RTCore] WARNING: native drive-home failed verification:"
            f" joint={joint_i + 1} axis_mask=0x{axis_mask:x}"
            f" abort=0x{abort_code & 0xFFFFFFFF:08X}"
            + (" (RTCore disarm precondition timeout)" if abort_is_disarm_timeout else "")
        )
        return _result(
            accepted=False,
            verified=False,
            code=(
                "NATIVE_HOME_DISARM_PRECONDITION_TIMEOUT"
                if abort_is_disarm_timeout
                else "NATIVE_HOME_FAILED"
            ),
            message=(
                (
                    "RTCore refused to start HM35 because the drive never left"
                    " OperationEnabled within the disarm precondition window."
                )
                if abort_is_disarm_timeout
                else (
                    "Drive-native commissioning home failed verification."
                    + (
                        f" Abort code 0x{abort_code & 0xFFFFFFFF:08X}."
                        if abort_code != 0
                        else ""
                    )
                )
            ),
            joint=joint_i + 1,
            axis_mask=axis_mask,
            terminal_state=str(wait_result.get("terminal_state", "failed")),
            native_home_state=int(wait_result.get("native_home_state", 3)),
            native_home_last_abort_code=abort_code,
            metrics_time_ns=int(wait_result.get("metrics_time_ns", 0)),
            extra=(
                {"disarm_precondition_timed_out": True}
                if abort_is_disarm_timeout
                else None
            ),
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
        if not self._connected:
            raise RuntimeError("Canonical joint truth unavailable (rtcore_disconnected)")
        if self._axis_config is None:
            raise RuntimeError("Canonical joint truth unavailable (rtcore_feedback_not_ready)")

        positions = self.raw_to_joint_positions(self.sync_read_positions())
        if verbose:
            print("[EtherCAT RTCore] get_joint_positions() (connected) -> live feedback")
        return positions

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
        *,
        reference_mode: str = "raw",
    ) -> dict[str, object]:
        positions = self._copy_last_joint_setpoint_rad()
        partial_position_samples: list[list[float]] = [[] for _ in range(self._num_joints)]
        axis_truth_details: list[dict[str, object]] = []
        unavailable_axes: list[int] = []
        unavailable_joints: list[int] = []
        drive_native_enabled_axes: list[int] = []
        drive_native_startup_invalid_axes: list[int] = []
        drive_native_unavailable_axes: list[int] = []
        position_semantics_sources: list[str] = []
        metrics_snapshot = self._load_rtcore_metrics_snapshot()
        metrics_axes = metrics_snapshot.get("axes") if isinstance(metrics_snapshot, dict) else None
        normalized_reference_mode = str(reference_mode).strip().lower()
        for axis_i, joint_i in enumerate(self._axis_to_joint):
            raw = raw_positions.get(axis_i)
            logical_joint = joint_i + 1 if 0 <= joint_i < self._num_joints else None
            metrics = self._absolute_feedback_metrics_for_axis(axis_i)
            axis_snapshot = (
                metrics_axes[axis_i]
                if isinstance(metrics_axes, list)
                and axis_i < len(metrics_axes)
                and isinstance(metrics_axes[axis_i], dict)
                else None
            )
            normalized_absolute_feedback = self._normalize_absolute_feedback_metrics(metrics)
            absolute_result = self._absolute_axis_q_from_metrics(axis_i, metrics)
            reference_q: float | None = None
            raw_reference_q: float | None = None
            axis_payload = axis_snapshot if isinstance(axis_snapshot, dict) else {}
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
            # Ask the active drive profile whether the live fault/alarm
            # codes belong to the encoder-retention family (e.g. Er20.9,
            # ALF9.0 on A6-EC). When present this must outrank the generic
            # fault_present / manufacturer_fault_present branches and
            # block the persisted-home-anchor restart-trust path, because
            # multi-turn retention is exactly the precondition that path
            # depends on.
            encoder_retention_fault_detail = drive_profile_registry.describe_drive_encoder_retention_fault(
                self._profile_id_for_axis_semantics(),
                manufacturer_error_code=manufacturer_error_code,
                error_code=error_code,
            )
            encoder_retention_fault_present = bool(
                isinstance(encoder_retention_fault_detail, dict)
                and encoder_retention_fault_detail.get("present", False)
            )
            native_home_status = derive_effective_native_home_status(
                axis_payload,
                statusword=statusword,
                error_code=error_code,
                manufacturer_error_code=manufacturer_error_code,
            )
            drive_native_ratio_enabled = self._drive_native_ratio_enabled()
            absolute_home_anchor_required = self._absolute_home_anchor_required()
            configured_canonical_truth_source = (
                self._canonical_truth_source() if drive_native_ratio_enabled else "absolute_encoder_anchor"
            )
            canonical_truth_uses_absolute_feedback = (
                not drive_native_ratio_enabled
                or self._drive_native_canonical_truth_uses_absolute_feedback()
            )
            startup_truth_requires_hm_success_signature = (
                self._startup_truth_requires_hm_success_signature()
            )
            accept_persisted_home_anchor_as_restart_trust = (
                self._accept_persisted_home_anchor_as_restart_trust()
            )
            drive_native_startup = self._drive_native_startup_validity(axis_payload)
            drive_native_startup_valid = bool(drive_native_startup.get("drive_native_startup_valid", False))
            configured_position_semantics_source = (
                self._position_semantics_source()
                if drive_native_ratio_enabled
                else "absolute_encoder_anchor"
            )
            position_semantics_source = (
                configured_position_semantics_source
                if drive_native_ratio_enabled
                else "absolute_encoder_anchor"
            )
            # Pre-compute the anchor lookup and the shaft-frame consistency
            # gate so the validity helper has enough signal to accept a
            # missing bit 15 as still-trusted after a drive power cycle, per
            # the per-profile `accept_persisted_home_anchor_as_restart_trust`
            # flag. The gate itself is re-used later in place of the old
            # Workstream 3 short-circuit.
            anchor_required_for_truth = bool(
                absolute_home_anchor_required or canonical_truth_uses_absolute_feedback
            )
            anchor_entry = (
                self._absolute_home_anchor_for_joint(joint_i)
                if anchor_required_for_truth and 0 <= joint_i < self._num_joints
                else None
            )
            persisted_home_anchor_present = anchor_entry is not None
            multi_turn_feedback_valid = False
            if isinstance(normalized_absolute_feedback, Mapping):
                mt_source_payload = normalized_absolute_feedback.get(
                    _PROFILE_MULTI_TURN_COUNTS_KEY
                )
                if isinstance(mt_source_payload, Mapping):
                    multi_turn_feedback_valid = bool(mt_source_payload.get("valid"))
            # W1 last-seen U40.20/.22 sidecar: compute a pre-gate
            # delta between the in-memory absolute counts and the
            # sidecar value stored on the anchor. The sidecar is
            # diagnostic only; it is passed into the validity helper
            # so that when the shaft-frame gate fails AND the delta is
            # physically impossible (>32767 motor revs for A6-EC), the
            # rejection reason upgrades to the more specific
            # "multi_turn_feedback_lost_across_power_cycle" label.
            last_seen_sidecar: dict[str, Any] | None = None
            last_seen_delta_counts: int | None = None
            last_seen_delta_physically_possible: bool | None = None
            max_off_motor_delta_counts: int | None = None
            if isinstance(anchor_entry, dict):
                candidate_sidecar = anchor_entry.get("last_seen")
                if isinstance(candidate_sidecar, dict):
                    try:
                        stored_absolute_counts = int(candidate_sidecar.get("absolute_counts"))
                    except Exception:
                        stored_absolute_counts = None
                    if (
                        stored_absolute_counts is not None
                        and absolute_result is not None
                    ):
                        last_seen_sidecar = dict(candidate_sidecar)
                        last_seen_delta_counts = int(
                            int(absolute_result[2]) - int(stored_absolute_counts)
                        )
                        counts_per_rev_axis = self._encoder_counts_per_rev_for_axis(axis_i)
                        if counts_per_rev_axis > 0:
                            max_off_motor_delta_counts = int(
                                _MAX_OFF_MOTOR_REVOLUTIONS * counts_per_rev_axis
                            )
                            last_seen_delta_physically_possible = bool(
                                abs(int(last_seen_delta_counts)) <= int(max_off_motor_delta_counts)
                            )
            persisted_home_anchor_consistent: bool | None = None
            anchor_consistency_detail: dict[str, object] | None = None
            if (
                canonical_truth_uses_absolute_feedback
                and raw is not None
                and anchor_entry is not None
                and absolute_result is not None
            ):
                # master_offset cancels inside the gate (expected_reference_q
                # re-adds it before comparing to live 6064). Use the full
                # canonical_q expression so the diagnostic fields land with
                # the same meaning they have later in the block.
                trial_canonical_q = (
                    float(absolute_result[0])
                    - float(anchor_entry["home_anchor_rad"])
                    - self._master_offset_for_joint(joint_i)
                )
                anchor_consistency_detail = self._shaft_frame_consistency_detail(
                    axis_i=axis_i,
                    canonical_q=float(trial_canonical_q),
                    logical_joint_idx=joint_i,
                    live_reference_counts=int(raw),
                )
                if isinstance(anchor_consistency_detail, dict):
                    persisted_home_anchor_consistent = bool(
                        anchor_consistency_detail.get("shaft_frame_consistent", False)
                    )
            drive_native_truth = derive_drive_native_truth_validity(
                axis_payload,
                statusword=statusword,
                error_code=error_code,
                manufacturer_error_code=manufacturer_error_code,
                require_hm_success_signature=startup_truth_requires_hm_success_signature,
                accept_persisted_home_anchor_as_restart_trust=accept_persisted_home_anchor_as_restart_trust,
                persisted_home_anchor_present=persisted_home_anchor_present,
                persisted_home_anchor_consistent=persisted_home_anchor_consistent,
                multi_turn_feedback_valid=multi_turn_feedback_valid,
                encoder_retention_fault_present=encoder_retention_fault_present,
                last_seen_present=bool(last_seen_sidecar is not None),
                last_seen_delta_physically_possible=last_seen_delta_physically_possible,
            )
            # Expose the persisted-anchor signals so operator tools and
            # regressions can inspect exactly why a given axis was (or was
            # not) upgraded to trust via the persisted-home-anchor path.
            if accept_persisted_home_anchor_as_restart_trust:
                axis_payload_restart_trust_context = {
                    "persisted_home_anchor_present": bool(persisted_home_anchor_present),
                    "multi_turn_feedback_valid": bool(multi_turn_feedback_valid),
                }
                if persisted_home_anchor_consistent is not None:
                    axis_payload_restart_trust_context[
                        "persisted_home_anchor_consistent"
                    ] = bool(persisted_home_anchor_consistent)
            else:
                axis_payload_restart_trust_context = None
            detail: dict[str, object] = {
                "axis": int(axis_i),
                "logical_joint": logical_joint,
                "absolute_feedback": normalized_absolute_feedback,
                "reference_mode": normalized_reference_mode,
                "configured_position_semantics_source": configured_position_semantics_source,
                "configured_canonical_truth_source": configured_canonical_truth_source,
                "canonical_truth_uses_absolute_feedback": bool(canonical_truth_uses_absolute_feedback),
                "position_semantics_source": position_semantics_source,
                "drive_native_ratio_enabled": bool(drive_native_ratio_enabled),
                "drive_native_startup_valid": bool(drive_native_startup_valid),
                "drive_native_startup_reason": str(
                    drive_native_startup.get("drive_native_startup_reason", "unknown")
                ),
                "drive_native_truth_valid": bool(drive_native_truth.get("drive_native_truth_valid", False)),
                "drive_native_truth_reason": str(drive_native_truth.get("drive_native_truth_reason", "unknown")),
                "drive_native_truth_signature_valid": bool(
                    drive_native_truth.get("drive_native_truth_signature_valid", False)
                ),
                "coordinate_system_valid": bool(drive_native_truth.get("coordinate_system_valid", False)),
                "drive_native_truth_verification_source": str(
                    drive_native_truth.get("drive_native_truth_verification_source", "unverified")
                ),
                "native_home_state": int(native_home_status.get("native_home_state", 0)),
                "native_home_state_name": str(native_home_status.get("native_home_state_name", "idle")),
                "native_home_last_abort_code": int(native_home_status.get("native_home_last_abort_code", 0)),
                "native_home_verification_source": str(
                    native_home_status.get("native_home_verification_source", "native_home_state")
                ),
                "statusword": int(statusword),
                "statusword_hex": f"0x{statusword & 0xFFFF:04X}",
                "error_code": int(error_code),
                "manufacturer_error_code": int(manufacturer_error_code),
            }
            # Populate the shaft-frame / anchor-trust context early so it
            # rides along with detail even when a later short-circuit sets
            # truth_available=False.
            if isinstance(anchor_consistency_detail, dict):
                detail.update(anchor_consistency_detail)
            if axis_payload_restart_trust_context is not None:
                detail.update(axis_payload_restart_trust_context)
            # Surface the profile-decoded encoder-retention fault detail
            # when present so operator tools see matched vendor codes/names
            # alongside the derived truth reason.
            if encoder_retention_fault_present and isinstance(
                encoder_retention_fault_detail, dict
            ):
                detail["encoder_retention_fault"] = dict(encoder_retention_fault_detail)
                detail["encoder_retention_fault_present"] = True
            # W1: surface the optional last-seen U40.20/.22 sidecar so
            # /info/joints-detailed can expose it. The sidecar is
            # diagnostic only - consumers that care about trust should
            # branch on `drive_native_truth_*` fields, not on these.
            if last_seen_sidecar is not None:
                detail["last_seen_absolute_counts"] = int(
                    last_seen_sidecar.get("absolute_counts", 0)
                )
                if "reference_counts" in last_seen_sidecar:
                    detail["last_seen_reference_counts"] = int(
                        last_seen_sidecar.get("reference_counts", 0)
                    )
                if "observed_at" in last_seen_sidecar:
                    detail["last_seen_observed_at"] = str(
                        last_seen_sidecar.get("observed_at", "")
                    )
                if "observed_by" in last_seen_sidecar:
                    detail["last_seen_observed_by"] = str(
                        last_seen_sidecar.get("observed_by", "")
                    )
            if last_seen_delta_counts is not None:
                detail["last_seen_delta_counts"] = int(last_seen_delta_counts)
            if last_seen_delta_physically_possible is not None:
                detail["last_seen_delta_physically_possible"] = bool(
                    last_seen_delta_physically_possible
                )
            if max_off_motor_delta_counts is not None:
                detail["last_seen_delta_budget_counts"] = int(max_off_motor_delta_counts)
            position_semantics_sources.append(position_semantics_source)
            if drive_native_ratio_enabled:
                drive_native_enabled_axes.append(int(axis_i))
                if not drive_native_startup_valid:
                    drive_native_startup_invalid_axes.append(int(axis_i))
            display_reference_q: float | None = None
            if raw is not None:
                detail["raw_counts"] = int(raw)
                display_reference_q = self._reference_q_before_master_offset_for_axis(
                    axis_i,
                    int(raw),
                    reference_mode="display",
                )
                raw_reference_q = self._reference_q_before_master_offset_for_axis(
                    axis_i,
                    int(raw),
                    reference_mode="raw",
                )
                reference_q = self._reference_q_before_master_offset_for_axis(
                    axis_i,
                    int(raw),
                    reference_mode=normalized_reference_mode,
                )
                if reference_q is not None:
                    detail["reference_pre_zero_rad"] = float(reference_q)
                if display_reference_q is not None and normalized_reference_mode != "display":
                    detail["display_reference_pre_zero_rad"] = float(display_reference_q)
                if raw_reference_q is not None:
                    detail["raw_reference_pre_zero_rad"] = float(raw_reference_q)
            if anchor_entry is not None:
                detail["absolute_home_anchor_rad"] = float(anchor_entry["home_anchor_rad"])
                if anchor_entry.get("source") is not None:
                    detail["absolute_home_anchor_source"] = anchor_entry["source"]
            if absolute_result is not None:
                absolute_axis_q, absolute_source, absolute_counts = absolute_result
                detail["absolute_counts"] = int(absolute_counts)
                detail["absolute_source"] = str(absolute_source)
                detail["absolute_axis_q_rad"] = float(absolute_axis_q)
                if canonical_truth_uses_absolute_feedback:
                    detail["canonical_truth_counts_source"] = str(absolute_source)
            else:
                absolute_axis_q = None
                absolute_source = None

            truth_reason = None
            if raw is None:
                truth_reason = "raw_feedback_missing"
            elif joint_i < 0 or joint_i >= self._num_joints:
                truth_reason = "logical_joint_unmapped"
            elif reference_q is None:
                truth_reason = "reference_frame_unavailable"
            elif drive_native_ratio_enabled and not drive_native_startup_valid:
                truth_reason = (
                    f"drive_native_{drive_native_startup.get('drive_native_startup_reason', 'startup_invalid')}"
                )
            elif drive_native_ratio_enabled and not bool(drive_native_truth.get("drive_native_truth_valid", False)):
                truth_reason = f"drive_native_{drive_native_truth.get('drive_native_truth_reason', 'invalid')}"
            elif canonical_truth_uses_absolute_feedback and absolute_axis_q is None:
                truth_reason = (
                    "drive_native_absolute_feedback_unavailable"
                    if drive_native_ratio_enabled
                    else "absolute_feedback_unavailable"
                )
            elif canonical_truth_uses_absolute_feedback and anchor_entry is None:
                truth_reason = (
                    "drive_native_absolute_home_anchor_missing"
                    if drive_native_ratio_enabled
                    else "absolute_home_anchor_missing"
                )

            if truth_reason is not None:
                detail["truth_available"] = False
                detail["truth_status"] = "unavailable"
                detail["truth_reason"] = truth_reason
                detail["display_source"] = "truth_unavailable"
                unavailable_axes.append(int(axis_i))
                if logical_joint is not None:
                    unavailable_joints.append(int(logical_joint))
                if drive_native_ratio_enabled:
                    drive_native_unavailable_axes.append(int(axis_i))
                axis_truth_details.append(detail)
                continue

            if canonical_truth_uses_absolute_feedback:
                detail["canonical_truth_source"] = configured_canonical_truth_source
                canonical_q = (
                    float(absolute_axis_q)
                    - float(anchor_entry["home_anchor_rad"])
                    - self._master_offset_for_joint(joint_i)
                )
            elif drive_native_ratio_enabled:
                detail["canonical_truth_source"] = "drive_reference_frame"
                canonical_q = float(reference_q) - self._master_offset_for_joint(joint_i)
            else:
                detail["canonical_truth_source"] = "absolute_encoder_anchor"
                canonical_q = (
                    float(absolute_axis_q)
                    - float(anchor_entry["home_anchor_rad"])
                    - self._master_offset_for_joint(joint_i)
                )

            # Vendor Q4/Q10: 6064 is authoritative within shaft space. When
            # our multi-turn canonical truth is rooted in anchored
            # U40.20/.22, we must prove that truth still agrees with live
            # 6064 modulo RM before publishing it. A whole-shaft-turn offset
            # in the anchor is legitimate (it only changes which turn we are
            # on, not where on the shaft); a sub-shaft-turn disagreement is
            # a frame bug and must fail closed. We also surface the
            # stale-anchor diagnostic fields when relevant so operators see
            # how much the stored anchor needs to move.
            #
            # We already computed this gate above to feed the truth-validity
            # helper (it also underpins the persisted-home-anchor restart
            # trust path) and merged it into `detail`. Re-use the cached
            # result here to decide whether to short-circuit with the
            # "multi_turn_anchor_inconsistent_with_live_6064" reason.
            if (
                canonical_truth_uses_absolute_feedback
                and raw is not None
                and anchor_entry is not None
            ):
                shaft_gate = anchor_consistency_detail
                if isinstance(shaft_gate, dict):
                    if not bool(shaft_gate.get("shaft_frame_consistent", True)):
                        if absolute_axis_q is not None:
                            diagnostic_reference_q = float(
                                display_reference_q
                                if display_reference_q is not None
                                else reference_q
                            )
                            stale_anchor_detail = self._absolute_home_anchor_diagnostic_for_axis(
                                axis_i=axis_i,
                                axis_snapshot=axis_snapshot,
                                absolute_axis_q=float(absolute_axis_q),
                                reference_q=diagnostic_reference_q,
                                anchor_entry=anchor_entry,
                            )
                            detail.update(stale_anchor_detail)
                        detail["truth_available"] = False
                        detail["truth_status"] = "unavailable"
                        detail["truth_reason"] = (
                            "multi_turn_anchor_inconsistent_with_live_6064"
                        )
                        detail["display_source"] = "truth_unavailable"
                        unavailable_axes.append(int(axis_i))
                        if logical_joint is not None:
                            unavailable_joints.append(int(logical_joint))
                        if drive_native_ratio_enabled:
                            drive_native_unavailable_axes.append(int(axis_i))
                        axis_truth_details.append(detail)
                        continue

            roundtrip_detail = self._command_roundtrip_detail_for_axis(
                axis_i=axis_i,
                logical_joint_idx=joint_i,
                canonical_q=float(canonical_q),
                reference_q=float(reference_q),
                reference_mode=normalized_reference_mode,
            )
            detail.update(roundtrip_detail)
            raw_roundtrip_detail: dict[str, object] | None = None
            if raw_reference_q is not None and normalized_reference_mode != "raw":
                raw_roundtrip_detail = self._command_roundtrip_detail_for_axis(
                    axis_i=axis_i,
                    logical_joint_idx=joint_i,
                    canonical_q=float(canonical_q),
                    reference_q=float(raw_reference_q),
                    reference_mode="raw",
                )
                detail.update(
                    {
                        f"raw_{key}": value
                        for key, value in raw_roundtrip_detail.items()
                    }
                )
            roundtrip_consistent = bool(roundtrip_detail.get("command_roundtrip_consistent", False))
            raw_roundtrip_consistent = bool(
                raw_roundtrip_detail.get("command_roundtrip_consistent", False)
            ) if raw_roundtrip_detail is not None else True
            if not roundtrip_consistent or not raw_roundtrip_consistent:
                if canonical_truth_uses_absolute_feedback and absolute_axis_q is not None and anchor_entry is not None:
                    diagnostic_reference_q = float(
                        display_reference_q
                        if display_reference_q is not None
                        else reference_q
                    )
                    stale_anchor_detail = self._absolute_home_anchor_diagnostic_for_axis(
                        axis_i=axis_i,
                        axis_snapshot=axis_snapshot,
                        absolute_axis_q=float(absolute_axis_q),
                        reference_q=diagnostic_reference_q,
                        anchor_entry=anchor_entry,
                    )
                    detail.update(stale_anchor_detail)
                detail["truth_available"] = False
                detail["truth_status"] = "unavailable"
                if canonical_truth_uses_absolute_feedback and bool(detail.get("absolute_home_anchor_stale", False)):
                    detail["truth_reason"] = "absolute_home_anchor_stale"
                elif drive_native_ratio_enabled:
                    detail["truth_reason"] = "drive_native_command_frame_roundtrip_mismatch"
                else:
                    detail["truth_reason"] = "command_frame_roundtrip_mismatch"
                detail["display_source"] = "truth_unavailable"
                unavailable_axes.append(int(axis_i))
                if logical_joint is not None:
                    unavailable_joints.append(int(logical_joint))
                if drive_native_ratio_enabled:
                    drive_native_unavailable_axes.append(int(axis_i))
                axis_truth_details.append(detail)
                continue

            positions[joint_i] = float(canonical_q)
            partial_position_samples[joint_i].append(float(canonical_q))
            detail["truth_available"] = True
            detail["truth_status"] = "available"
            detail["truth_source"] = position_semantics_source
            detail["canonical_rad"] = float(canonical_q)
            # Keep the display aliases aligned with the active truth source so the
            # operator view follows the same canonical frame.
            detail["display_source"] = position_semantics_source
            detail["display_rad"] = float(canonical_q)
            # W1: persist a rate-limited last-seen U40.20/.22 sidecar on
            # the runtime canonical-truth path (reference_mode="raw").
            # Skip in display-only mode (same spirit as the
            # mutate_command_wrap_bookkeeping=False guard): display
            # reads must stay observational and must not mutate anchor
            # state.
            if (
                normalized_reference_mode == "raw"
                and canonical_truth_uses_absolute_feedback
                and anchor_entry is not None
                and absolute_result is not None
                and raw is not None
                and multi_turn_feedback_valid
                and not encoder_retention_fault_present
                and 0 <= joint_i < self._num_joints
            ):
                self._persist_last_seen_absolute_counts_for_joint(
                    logical_joint_idx=int(joint_i),
                    absolute_counts=int(absolute_result[2]),
                    reference_counts=int(raw),
                    now_wall_s=time.time(),
                )
            axis_truth_details.append(detail)

        unavailable_axes = sorted(set(unavailable_axes))
        unavailable_joints = sorted(set(unavailable_joints))
        drive_native_enabled_axes = sorted(set(drive_native_enabled_axes))
        drive_native_startup_invalid_axes = sorted(set(drive_native_startup_invalid_axes))
        drive_native_unavailable_axes = sorted(set(drive_native_unavailable_axes))
        joint_positions_partial: list[float | None] = []
        for samples in partial_position_samples:
            if samples:
                joint_positions_partial.append(float(sum(samples) / len(samples)))
            else:
                joint_positions_partial.append(None)
        unique_semantics_sources = sorted(set(position_semantics_sources))
        semantics_source = "mixed" if len(unique_semantics_sources) > 1 else (
            unique_semantics_sources[0] if unique_semantics_sources else "unknown"
        )
        return {
            "joint_positions_rad": [float(value) for value in positions],
            # Publish an explicit per-joint truth view alongside the legacy
            # all-or-nothing list. When truth is unavailable for a joint we keep
            # it as null instead of leaking cached setpoints back into the
            # operator display path.
            "joint_positions_rad_partial": joint_positions_partial,
            "axis_absolute_feedback": axis_truth_details,
            "truth_available": len(unavailable_axes) == 0,
            "truth_unavailable_axes": unavailable_axes,
            "truth_unavailable_joints": unavailable_joints,
            "drive_native_ratio_enabled": bool(drive_native_enabled_axes),
            "drive_native_startup_valid": bool(drive_native_enabled_axes) and len(drive_native_startup_invalid_axes) == 0,
            "drive_native_startup_invalid_axes": drive_native_startup_invalid_axes,
            "drive_native_truth_available": (
                bool(drive_native_enabled_axes)
                and len(drive_native_startup_invalid_axes) == 0
                and len(drive_native_unavailable_axes) == 0
            ),
            "drive_native_truth_unavailable_axes": drive_native_unavailable_axes,
            "position_semantics_source": semantics_source,
        }

    def _canonical_joint_positions_or_raise(
        self,
        raw_positions: dict[int, int],
        *,
        reference_mode: str = "raw",
        error_label: str = "Canonical joint truth unavailable",
    ) -> list[float]:
        snapshot = self._canonical_joint_positions_from_raw_feedback(
            raw_positions,
            reference_mode=reference_mode,
        )
        if bool(snapshot.get("truth_available")):
            positions = snapshot.get("joint_positions_rad")
            if isinstance(positions, list) and positions:
                return [float(value) for value in positions]
        unavailable_axes = snapshot.get("truth_unavailable_axes")
        unavailable_joints = snapshot.get("truth_unavailable_joints")
        truth_reasons: list[str] = []
        statuswords: list[str] = []
        axis_truth_details = snapshot.get("axis_absolute_feedback")
        if isinstance(axis_truth_details, list):
            for detail in axis_truth_details:
                if not isinstance(detail, dict) or bool(detail.get("truth_available", False)):
                    continue
                reason = str(detail.get("truth_reason", "")).strip()
                if reason:
                    truth_reasons.append(reason)
                statusword_hex = str(detail.get("statusword_hex", "")).strip()
                if statusword_hex:
                    statuswords.append(statusword_hex)
        truth_reasons = sorted(set(truth_reasons))
        statuswords = sorted(set(statuswords))
        raise RuntimeError(
            f"{error_label}"
            f" (axes={list(unavailable_axes) if isinstance(unavailable_axes, list) else []},"
            f" joints={list(unavailable_joints) if isinstance(unavailable_joints, list) else []},"
            f" reasons={truth_reasons}, statuswords={statuswords})"
        )

    def raw_to_joint_positions(self, raw_positions: dict[int, int]) -> list[float]:
        # Controller/planner truth stays continuous in logical joint space.
        # Operator display uses the stricter display snapshot path below.
        # The 607A turn selection is now a stateless nearest-turn fold against
        # live 6064 applied at write time (see _command_axis_q_for_joint_value),
        # so no per-axis wrap-lift state needs to be mutated here.
        return self._canonical_joint_positions_or_raise(
            raw_positions,
            reference_mode="raw",
        )

    def get_display_feedback_snapshot(
        self,
        raw_positions: dict[int, int] | None = None,
    ) -> dict[str, object] | None:
        if raw_positions is None:
            raw_positions = self.sync_read_positions()
        if not isinstance(raw_positions, dict) or not raw_positions:
            return None
        return self._canonical_joint_positions_from_raw_feedback(
            raw_positions,
            reference_mode="display",
        )

    def raw_to_display_joint_positions(self, raw_positions: dict[int, int]) -> list[float]:
        # Operator-facing joint display follows the same canonical truth source as
        # the controller state rather than the raw 0x6064 transport frame.
        return self._canonical_joint_positions_or_raise(
            raw_positions,
            reference_mode="display",
            error_label="Operator display joint truth unavailable",
        )

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
        if self._absolute_home_anchor_required():
            try:
                self._capture_absolute_home_anchor_for_joint(
                    joint_i,
                    actor=f"ethercat_rtcore:joint{joint_i + 1}:software_zero",
                    reference_mode="display",
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

    def _command_counts_wrap_for_joint(self, logical_joint_idx: int) -> bool:
        """True when the active drive profile wraps 0x607A into the
        single-turn window for this joint (default), False for profiles
        that have opted into continuous 0x607A emission (A6-EC rotation
        mode per vendor Chapter 5 Figure 5-1).

        Reads the profile's `MOTION_FEEDBACK_CONFIG` via the drive-profile
        registry so the decision stays drive-family-specific. Defaults to
        True on unknown profiles to preserve the historical wrap
        behavior.
        """
        payload = None
        try:
            payload = drive_profile_registry.get_drive_motion_feedback_config(
                self._effective_drive_profile_id()
            )
        except Exception:
            payload = None
        if isinstance(payload, Mapping):
            if "command_counts_wrap" in payload:
                return bool(payload.get("command_counts_wrap"))
            command_axes = payload.get("command_wrap_axes")
            if isinstance(command_axes, list):
                return int(logical_joint_idx) in {int(x) for x in command_axes if isinstance(x, int)}
            if "feedback_counts_wrap" in payload:
                return bool(payload.get("feedback_counts_wrap"))
        return True

    def _resolve_experimental_continuous_607a_joint_indices(self, num_joints: int) -> set[int]:
        # Retired path kept for API compatibility with any test helper
        # that still constructs the set directly. The returned value is
        # always empty because the decision has moved to
        # `_command_counts_wrap_for_joint`.
        raw = os.environ.get("GRADIENT_RTCORE_EXPERIMENTAL_CONTINUOUS_607A_JOINTS", "").strip()
        if not raw:
            return set()
        joints: set[int] = set()
        invalid_tokens: list[str] = []
        for token in [part.strip() for part in raw.split(",") if part.strip()]:
            normalized = token.upper()
            if normalized.startswith("J"):
                normalized = normalized[1:]
            try:
                joint_1based = int(normalized)
            except ValueError:
                invalid_tokens.append(token)
                continue
            joint_i = joint_1based - 1
            if 0 <= joint_i < num_joints:
                joints.add(joint_i)
            else:
                invalid_tokens.append(token)
        if invalid_tokens:
            print(
                "[EtherCAT RTCore] WARNING: invalid "
                f"GRADIENT_RTCORE_EXPERIMENTAL_CONTINUOUS_607A_JOINTS='{raw}' "
                f"(ignored tokens: {', '.join(invalid_tokens)})"
            )
        return joints

    def _experimental_continuous_607a_enabled_for_joint(self, logical_joint_idx: int) -> bool:
        # Retained as a thin alias of `_command_counts_wrap_for_joint` so
        # existing callers keep their signatures. Returns True when the
        # profile emits continuous (unwrapped) 0x607A for this joint.
        return not self._command_counts_wrap_for_joint(logical_joint_idx)

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
        counts_per_rev = list(robot_config.get("actuator_encoder_counts_per_rev", []))
        counts_per_radian = [float(value) for value in list(robot_config.get("actuator_counts_per_radian", []))]
        if not counts_per_radian:
            gear_ratios = list(robot_config.get("actuator_gear_ratios", []))
            counts_per_radian = []
            for idx in range(max(len(counts_per_rev), len(gear_ratios))):
                cpr = int(counts_per_rev[idx]) if idx < len(counts_per_rev) else 0
                ratio = float(gear_ratios[idx]) if idx < len(gear_ratios) else 1.0
                if cpr <= 0 or ratio <= 0.0:
                    counts_per_radian.append(0.0)
                else:
                    counts_per_radian.append((float(cpr) * ratio) / _TWO_PI)

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
        rt_drive_profile_code = getattr(self, "_rt_drive_profile_code", 0)
        if not live_profile_id and rt_drive_profile_code:
            live_profile_id = rtcore_drive_profile_id_to_name(rt_drive_profile_code)
        return live_profile_id

    def _profile_id_for_axis_semantics(self) -> Optional[str]:
        return (
            self._effective_drive_profile_id()
            or getattr(self, "_configured_drive_profile_id", None)
            or DEFAULT_DRIVE_PROFILE_ID
        )

    def _drive_position_semantics_config(self, profile_id: Optional[str] = None) -> dict[str, object]:
        resolved_profile_id = str(profile_id).strip().lower() if profile_id else self._profile_id_for_axis_semantics()
        payload = drive_profile_registry.get_drive_position_semantics_config(resolved_profile_id)
        return dict(payload) if isinstance(payload, dict) else {}

    def _drive_native_ratio_enabled(self, profile_id: Optional[str] = None) -> bool:
        payload = self._drive_position_semantics_config(profile_id)
        return bool(payload.get("drive_native_ratio_enabled", False))

    def _absolute_home_anchor_required(self, profile_id: Optional[str] = None) -> bool:
        payload = self._drive_position_semantics_config(profile_id)
        if "absolute_home_anchor_required" in payload:
            return bool(payload.get("absolute_home_anchor_required"))
        return not self._drive_native_ratio_enabled(profile_id)

    def _position_semantics_source(self, profile_id: Optional[str] = None) -> str:
        payload = self._drive_position_semantics_config(profile_id)
        source = payload.get("position_semantics_source")
        if source is None:
            return "absolute_encoder_anchor"
        return str(source)

    def _canonical_truth_source(self, profile_id: Optional[str] = None) -> str:
        payload = self._drive_position_semantics_config(profile_id)
        source = payload.get("canonical_truth_source")
        if source is None:
            return "absolute_encoder_anchor"
        normalized = str(source).strip()
        return normalized or "absolute_encoder_anchor"

    def _drive_native_canonical_truth_uses_absolute_feedback(
        self,
        profile_id: Optional[str] = None,
    ) -> bool:
        if not self._drive_native_ratio_enabled(profile_id):
            return True
        return self._canonical_truth_source(profile_id).strip().lower() != "drive_reference_frame"

    def _startup_truth_requires_hm_success_signature(self, profile_id: Optional[str] = None) -> bool:
        payload = self._drive_position_semantics_config(profile_id)
        return bool(payload.get("startup_truth_requires_hm_success_signature", True))

    def _accept_persisted_home_anchor_as_restart_trust(self, profile_id: Optional[str] = None) -> bool:
        payload = self._drive_position_semantics_config(profile_id)
        return bool(payload.get("accept_persisted_home_anchor_as_restart_trust", False))

    def _command_frame_seam_crossing_unsafe(self, profile_id: Optional[str] = None) -> bool:
        payload = self._drive_position_semantics_config(profile_id)
        return bool(payload.get("command_frame_seam_crossing_unsafe", False))

    def _drive_native_startup_validity(
        self,
        axis_snapshot: Optional[dict[str, object]],
    ) -> dict[str, object]:
        if not self._drive_native_ratio_enabled():
            return {
                "drive_native_startup_valid": False,
                "drive_native_startup_reason": "profile_disabled",
            }
        startup_drive_config = None
        if isinstance(axis_snapshot, dict):
            extracted = drive_profile_registry.extract_drive_startup_config_axis(
                self._effective_drive_profile_id(),
                dict(axis_snapshot),
            )
            if isinstance(extracted, dict):
                startup_drive_config = extracted
            else:
                startup_drive_config = axis_snapshot.get("startup_drive_config")
        if not isinstance(startup_drive_config, dict):
            return {
                "drive_native_startup_valid": False,
                "drive_native_startup_reason": "startup_drive_config_missing",
            }
        missing_setting_keys = startup_drive_config.get("missing_setting_keys", [])
        configured = bool(startup_drive_config.get("configured", False))
        readback_valid = bool(startup_drive_config.get("readback_valid", False))
        verified = bool(startup_drive_config.get("verified", False))
        if isinstance(missing_setting_keys, list) and missing_setting_keys:
            reason = "startup_drive_config_missing_required_settings"
        elif not configured:
            reason = "startup_drive_config_unconfigured"
        elif not readback_valid:
            reason = "startup_drive_config_unverified"
        elif not verified:
            reason = "startup_drive_config_mismatch"
        else:
            reason = "verified"
        return {
            "drive_native_startup_valid": configured and readback_valid and verified,
            "drive_native_startup_reason": reason,
        }

    def _reference_wrap_period_counts_for_axis(self, axis_i: int) -> int:
        cfg = self._axis_config
        if cfg is None or axis_i < 0 or axis_i >= int(cfg.num_axes):
            return 0
        if self._effective_drive_profile_id() != "a6ec_ds402":
            return 0
        if self._drive_native_ratio_enabled():
            counts_per_unit = float(cfg.counts_per_unit[axis_i])
            if counts_per_unit > 0.0:
                period_counts = int(round(float(counts_per_unit) * _TWO_PI))
                if period_counts > 0:
                    return int(period_counts)
        counts_per_rev = int(cfg.counts_per_rev[axis_i]) if axis_i < len(cfg.counts_per_rev) else 0
        return counts_per_rev if counts_per_rev > 0 else 0

    def _normalize_feedback_counts_for_axis(self, axis_i: int, raw_counts: int) -> int:
        period_counts = self._reference_wrap_period_counts_for_axis(axis_i)
        if period_counts <= 0:
            return int(raw_counts)
        half_turn = period_counts // 2
        if half_turn <= 0:
            return int(raw_counts)
        return ((int(raw_counts) + half_turn) % period_counts) - half_turn

    def _display_feedback_counts_for_axis(self, axis_i: int, raw_counts: int) -> int:
        # Preferred path: derive the unwrapped display counts from the drive's
        # unambiguous multi-turn register (profile contract:
        # `encoder_multi_turn_counts`; A6-EC populates via U40.20 + U40.22).
        # This is the SAME ground truth the seam-aware fold and anchor-
        # consistency gates use, eliminating a class of drift bugs that the
        # host-accumulated unwrap below is susceptible to:
        #   * pre-OP PDO zero-reads at drive startup can seed the cache at 0,
        #     permanently offsetting by one turn when the drive later reports
        #     a near-seam position (round((0 - ~RM)/RM) = -1).
        #   * HM35 rewrites 0x607C which causes 0x6064 to jump by up to RM/2
        #     with zero physical motion; nearest-turn unwrap interprets that
        #     jump as "same turn" and propagates a stale pre-home turn count
        #     through the home procedure indefinitely.
        # Requires BOTH a valid multi-turn reading AND a captured home anchor;
        # falls back cleanly when either is missing. Only engaged for drives
        # running in native-ratio mode so the legacy single-motor-turn
        # semantics (counts_per_rev wrap) are not disturbed.
        normalized_counts = self._normalize_feedback_counts_for_axis(axis_i, raw_counts)
        period_counts = self._reference_wrap_period_counts_for_axis(axis_i)
        if period_counts <= 0:
            return int(normalized_counts)
        if self._drive_native_ratio_enabled():
            multi_turn_axis_q_counts = (
                self._multi_turn_reference_counts_for_axis_when_anchored(axis_i)
            )
            if multi_turn_axis_q_counts is not None:
                native_home_offset_counts = int(
                    self._native_home_offset_counts_for_axis(axis_i)
                )
                # multi_turn_axis_q is in the canonical axis-q frame (home
                # subtracted); the display path is contracted to return the
                # WIRE-frame unwrapped counts. Forward conversion is
                # wire + native_home_offset_counts = axis_q_counts, so
                # reverse is wire = axis_q_counts - native_home_offset_counts.
                wire_unwrapped_counts = int(multi_turn_axis_q_counts) - int(
                    native_home_offset_counts
                )
                with self._status_lock:
                    if (
                        axis_i < len(self._feedback_unwrapped_counts)
                        and axis_i < len(self._feedback_unwrapped_valid)
                    ):
                        # Keep the fallback cache in lockstep with ground truth
                        # so any transient loss of multi-turn data (first cycles
                        # after boot, SDO poll glitch) continues from a correct
                        # value, not a stale one.
                        self._feedback_unwrapped_counts[axis_i] = int(
                            wire_unwrapped_counts
                        )
                        self._feedback_unwrapped_valid[axis_i] = True
                return int(wire_unwrapped_counts)
        # Fallback: accumulated-turn unwrap. Used when the profile does not
        # expose a multi-turn register, the drive has not yet published a
        # valid multi-turn reading, no home anchor has been captured, or the
        # axis is running in legacy single-motor-turn mode.
        with self._status_lock:
            if axis_i >= len(self._feedback_unwrapped_counts) or axis_i >= len(self._feedback_unwrapped_valid):
                return int(normalized_counts)
            if not self._feedback_unwrapped_valid[axis_i]:
                # Guard against the pre-OP PDO zero-read pattern: during the
                # first few seconds after stack start the drive publishes
                # `pos_counts = 0, statusword = 0x0000` until PDO mapping
                # latches. Seeding the unwrap cache from that bogus zero
                # permanently offsets the cache by one turn when the drive
                # later reports a near-seam position. Treat "both the
                # normalized count and the live statusword are zero" as the
                # pre-OP signature and defer seeding until at least one of
                # them becomes non-zero.
                live_statusword = 0
                if 0 <= axis_i < len(self._axis_statusword):
                    live_statusword = int(self._axis_statusword[axis_i])
                if int(normalized_counts) == 0 and live_statusword == 0:
                    return int(normalized_counts)
                self._feedback_unwrapped_counts[axis_i] = int(normalized_counts)
                self._feedback_unwrapped_valid[axis_i] = True
                return int(normalized_counts)
            previous = int(self._feedback_unwrapped_counts[axis_i])
            turns = round((float(previous) - float(normalized_counts)) / float(period_counts))
            unwrapped = int(normalized_counts) + (int(turns) * period_counts)
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
        logical_q = self._canonical_joint_q_from_command_axis_q(axis_i, joint_i, float(target_axis_q))
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
            # HM35 writes a new 0x607C value, which the drive reports as an
            # updated native_home_position_offset. The live 0x6064 reading
            # jumps by up to RM/2 with NO physical motion the moment the new
            # 607C latches. The fallback accumulated-unwrap path treats that
            # jump like ordinary motion (nearest-turn unwrap) and propagates
            # any pre-home turn state through the home procedure. Invalidate
            # the affected axis's unwrap cache so the next display query
            # re-seeds from the post-HM35 wire value. Defense-in-depth: the
            # primary display path derives from the anchor-relative multi-
            # turn register which self-corrects via the re-captured anchor,
            # but the fallback path needs this hook.
            for invalidate_axis_i in range(
                min(_GRADIENT_MAX_AXES, len(self._native_home_offset_counts))
            ):
                if offsets[invalidate_axis_i] != int(
                    self._native_home_offset_counts[invalidate_axis_i]
                ):
                    if invalidate_axis_i < len(self._feedback_unwrapped_valid):
                        self._feedback_unwrapped_valid[invalidate_axis_i] = False
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
        entry: dict[str, Any] = {
            "home_anchor_rad": float(home_anchor_rad),
            "source": str(source_raw).strip() if source_raw is not None else None,
            "axis_indices": axis_indices,
            "updated_at": str(updated_at_raw).strip() if updated_at_raw is not None else None,
            "updated_by": str(updated_by_raw).strip() if updated_by_raw is not None else None,
        }
        last_seen = raw_entry.get("last_seen")
        if isinstance(last_seen, dict):
            entry["last_seen"] = dict(last_seen)
        return entry

    def _encoder_counts_per_rev_for_axis(self, axis_i: int) -> int:
        """Encoder counts per single motor revolution for the target
        axis. Used by the W1 last-seen sidecar to bound
        ``last_seen_delta_physically_possible`` at
        ``32_767 * counts_per_rev``.
        """
        config = getattr(self, "_axis_config", None)
        try:
            counts_per_rev = int(config.counts_per_rev[axis_i]) if config is not None else 0
        except Exception:
            counts_per_rev = 0
        if counts_per_rev > 0:
            return int(counts_per_rev)
        robot_config = getattr(self, "_robot_config", None)
        if isinstance(robot_config, Mapping):
            raw = robot_config.get("actuator_encoder_counts_per_rev")
            if isinstance(raw, list) and 0 <= axis_i < len(raw):
                try:
                    return int(raw[axis_i])
                except Exception:
                    return 0
        return 0

    def _persist_last_seen_absolute_counts_for_joint(
        self,
        *,
        logical_joint_idx: int,
        absolute_counts: int,
        reference_counts: int,
        now_wall_s: float,
    ) -> None:
        """Rate-limited persistence of the last-seen sidecar on the
        anchor file. No-op when the joint has no recorded anchor
        (``save_last_seen_absolute_counts`` returns ``None`` in that
        case) and when the last persist for this joint was within
        ``_LAST_SEEN_PERSIST_INTERVAL_S``. Failures are swallowed - the
        sidecar is diagnostic, not safety-critical.
        """
        if logical_joint_idx < 0 or logical_joint_idx >= self._num_joints:
            return
        last_persist = self._last_seen_persist_last_wall_s.get(int(logical_joint_idx))
        if (
            last_persist is not None
            and (now_wall_s - float(last_persist)) < _LAST_SEEN_PERSIST_INTERVAL_S
        ):
            return
        try:
            saved_entry = save_last_seen_absolute_counts(
                self._robot_id,
                num_joints=self._num_joints,
                logical_joint_index=int(logical_joint_idx),
                absolute_counts=int(absolute_counts),
                reference_counts=int(reference_counts),
                observed_at_monotonic_ns=int(time.monotonic_ns()),
                actor="ethercat_rtcore:restart-trust",
            )
        except Exception:
            return
        if not isinstance(saved_entry, dict):
            return
        # Mirror the persisted sidecar back into the in-memory anchor
        # list so the next feedback cycle sees it without reloading the
        # file from disk.
        with self._status_lock:
            if 0 <= logical_joint_idx < len(self._absolute_encoder_home_anchors):
                existing = self._absolute_encoder_home_anchors[int(logical_joint_idx)]
                if isinstance(existing, dict):
                    merged = dict(existing)
                    merged["last_seen"] = dict(saved_entry.get("last_seen", {}))
                    self._absolute_encoder_home_anchors[int(logical_joint_idx)] = merged
        self._last_seen_persist_last_wall_s[int(logical_joint_idx)] = float(now_wall_s)

    def _reference_q_before_master_offset_for_axis(
        self,
        axis_i: int,
        raw_counts: int,
        *,
        reference_mode: str = "raw",
    ) -> float | None:
        normalized_reference_mode = str(reference_mode).strip().lower()
        if normalized_reference_mode == "display":
            physical_q = self._display_axis_q_from_raw_feedback_counts(axis_i, int(raw_counts))
        else:
            physical_q = self._axis_q_from_counts(axis_i, int(raw_counts))
        if physical_q is None:
            return None
        return float(physical_q) + self._native_home_offset_q_for_axis(axis_i)

    def _live_reference_counts_for_axis(self, axis_i: int) -> int | None:
        # The live 6064 wire-frame reading that the stateless nearest-turn
        # selector uses as its anchor. Returning None means there is no live
        # reference to fold against and the caller must treat that as "no
        # turn selection possible" rather than synthesizing one.
        with self._status_lock:
            if axis_i < 0 or axis_i >= len(self._axis_counts):
                return None
            return int(self._axis_counts[axis_i])

    def _native_home_offset_counts_for_axis(self, axis_i: int) -> int:
        self._refresh_native_home_offsets_from_metrics()
        with self._status_lock:
            if axis_i < 0 or axis_i >= len(self._native_home_offset_counts):
                return 0
            return int(self._native_home_offset_counts[axis_i])

    def _logicalized_live_reference_counts_for_axis(
        self,
        axis_i: int,
        *,
        logical_joint_idx: int,
        live_reference_counts: int | None,
    ) -> int | None:
        if live_reference_counts is None:
            return None
        resolved_counts = int(live_reference_counts)
        if self._experimental_continuous_607a_enabled_for_joint(logical_joint_idx):
            resolved_counts += int(self._native_home_offset_counts_for_axis(axis_i))
        return int(resolved_counts)

    def _multi_turn_reference_counts_for_axis(self, axis_i: int) -> int | None:
        """Return the drive's live multi-turn position for `axis_i` as an
        unambiguous continuous counts value in the SAME frame as the
        fold's `base_counts` (axis-q-counts, canonical-relative).

        Uses the profile's standard `encoder_multi_turn_counts` entry
        (see `_PROFILE_MULTI_TURN_COUNTS_KEY`) which any drive profile
        can populate via its `normalize_absolute_feedback()`; the A6-EC
        profile does this today by combining U40.20 + U40.22 into a
        signed i64. No A6-EC-specific code runs on this path.

        Frame conversion note: the drive's multi-turn register counts
        in encoder-internal frame (motor counts since encoder-internal
        zero). To put that in the fold's canonical-axis-q-counts frame
        we subtract the home-anchor value, which is
        `motor_counts_at_home_time` recorded when the operator
        commissioned the joint. At canonical 0 (home), the result is 0;
        at canonical X, the result is `X * sign * counts_per_unit`,
        which is exactly what `base_counts = base_axis_q * sign *
        counts_per_unit` expects.

        The gear ratio that distinguishes motor-frame from reference-
        frame is ALREADY baked into `counts_per_unit = RM / (2π)`, so
        motor_counts minus anchor (both in motor frame) equals the
        axis-q-counts equivalent of the current canonical angle (see
        `_axis_q_from_counts` for the same identity).

        Returns None when:
          * no axis config is attached yet, or
          * the encoder's counts-per-motor-rev is not known (sanity
            guard; exists because tests occasionally build _AxisConfig
            without counts_per_rev), or
          * the drive has not yet reported a valid multi-turn value
            (e.g., first cycles after boot before the metrics-thread SDO
            poll has fired).

        When the home anchor is not available (e.g., fresh boot before
        any anchor was commissioned), a zero anchor is used and the
        returned value is motor-encoder-internal. That is OK for the
        fold's seam-disambiguation purpose because the disambiguation
        code only cares about the SHIFT (modulo period_counts), which
        is unaffected by an absolute offset. A zero anchor biases the
        shift by a fixed amount per axis that cancels out inside the
        `round(delta/period)` call at non-seam positions; at the seam
        the shift correctly picks the side matching current motor
        position regardless of the anchor's absolute value.

        Callers that receive None fall back to their legacy single-turn
        `0x6064` logic; this method never raises.
        """
        cfg = self._axis_config
        if cfg is None or axis_i < 0 or axis_i >= int(cfg.num_axes):
            return None
        counts_per_motor_rev = (
            int(cfg.counts_per_rev[axis_i])
            if axis_i < len(cfg.counts_per_rev)
            else 0
        )
        if counts_per_motor_rev <= 0:
            return None
        metrics = self._absolute_feedback_metrics_for_axis(axis_i)
        if not metrics.has_any_valid():
            return None
        normalized = self._normalize_absolute_feedback_metrics(metrics)
        if not isinstance(normalized, Mapping):
            return None
        mt_payload = normalized.get(_PROFILE_MULTI_TURN_COUNTS_KEY)
        if not isinstance(mt_payload, Mapping) or not bool(mt_payload.get("valid")):
            return None
        try:
            motor_counts = int(mt_payload.get("value"))
        except Exception:
            return None
        # Translate from encoder-internal frame to canonical-axis-q
        # frame by subtracting the home-anchor (= motor_counts at home
        # commissioning time). Anchor is stored as `home_anchor_rad`;
        # convert to counts via sign * counts_per_unit.
        sign = int(cfg.sign[axis_i]) if axis_i < len(cfg.sign) else 0
        counts_per_unit = (
            float(cfg.counts_per_unit[axis_i]) if axis_i < len(cfg.counts_per_unit) else 0.0
        )
        anchor_counts = 0
        if sign in (-1, 1) and counts_per_unit > 0.0:
            with self._status_lock:
                joint_i = (
                    int(self._axis_to_joint[axis_i])
                    if 0 <= axis_i < len(self._axis_to_joint)
                    else -1
                )
                anchor_entry = None
                if 0 <= joint_i < len(self._absolute_encoder_home_anchors):
                    anchor_entry = self._absolute_encoder_home_anchors[joint_i]
            if isinstance(anchor_entry, dict):
                try:
                    anchor_rad = float(anchor_entry.get("home_anchor_rad"))
                except Exception:
                    anchor_rad = 0.0
                anchor_counts = int(round(anchor_rad * float(sign) * float(counts_per_unit)))
        return int(motor_counts - anchor_counts)

    def _multi_turn_reference_counts_for_axis_when_anchored(
        self, axis_i: int
    ) -> int | None:
        """Stricter variant of `_multi_turn_reference_counts_for_axis` for the
        operator-facing display path.

        The permissive sibling substitutes a zero anchor when none has been
        commissioned, which is fine for the seam-disambiguation fold (it only
        cares about the modulo-period shift) but WRONG for the display path:
        with a zero anchor the returned value is in motor-encoder-internal
        frame (large absolute offset per axis) and leaks into the operator
        view as bogus positions. Gate on anchor presence so the display
        caller can fall back to the legacy accumulated-unwrap path when the
        axis has not yet been commissioned.
        """
        cfg = self._axis_config
        if cfg is None or axis_i < 0 or axis_i >= int(cfg.num_axes):
            return None
        with self._status_lock:
            joint_i = (
                int(self._axis_to_joint[axis_i])
                if 0 <= axis_i < len(self._axis_to_joint)
                else -1
            )
            anchor_entry = None
            if 0 <= joint_i < len(self._absolute_encoder_home_anchors):
                anchor_entry = self._absolute_encoder_home_anchors[joint_i]
        if not isinstance(anchor_entry, dict):
            return None
        return self._multi_turn_reference_counts_for_axis(axis_i)

    def _nearest_turn_fold_axis_q_for_axis(
        self,
        axis_i: int,
        base_axis_q: float,
        *,
        observed_reference_counts: int | None = None,
        observed_reference_q: float | None = None,
        observed_multi_turn_reference_counts: int | None = None,
        wrap_to_single_turn: bool = False,
    ) -> tuple[float, int]:
        # Stateless per-write nearest-turn fold. Takes the desired axis-space
        # target and the live 6064 reference and returns an axis-space
        # value in one of two output frames:
        #
        # * Default (`wrap_to_single_turn=False`): value is within RM/2 of
        #   the live reference in LINEAR counts. This is the historical
        #   "windowed" behavior used by the roundtrip diagnostic and the
        #   canonical-from-axis reverse map. Linear comparisons against
        #   the observed reference stay meaningful in this frame.
        # * Command mode (`wrap_to_single_turn=True`): value is further
        #   folded into the drive's [0, RM) single-turn presentation
        #   range. The A6-EC command path must use this because the drive
        #   misinterprets 607A values outside [0, RM) in rotation mode -
        #   see the 2026-04-17 J6 incident note below.
        #
        # 2026-04-17 incident note: the A6-EC misinterprets 607A values
        # outside [0, RM) in rotation mode. Even with C10.16=0 (Nearest),
        # commanding 607A = RM + 3,623 (= one turn + 1 deg above 6064)
        # while 6064 was at 1,310,694 (near the seam) caused J6 to rotate
        # the LONG way by ~RM counts while the drive reported no fault.
        # Wrapping the command-path output into [0, RM) gives the drive a
        # canonical target it can parse without ambiguity; the shortest-
        # path decision then rests on the drive's C10.16 setting, not on
        # whatever undefined behavior the A6-EC uses for out-of-range
        # commands. The non-command callers keep the linear-windowed
        # value because they compare against reference_q directly and a
        # seam-crossing wrap would produce false-positive "roundtrip
        # inconsistent" diagnostics.
        cfg = self._axis_config
        period_counts = self._reference_wrap_period_counts_for_axis(axis_i)
        if cfg is None or axis_i < 0 or axis_i >= int(cfg.num_axes) or period_counts <= 0:
            return float(base_axis_q), 0
        counts_per_unit = float(cfg.counts_per_unit[axis_i])
        sign = int(cfg.sign[axis_i])
        if counts_per_unit <= 0.0 or sign not in (-1, 1):
            return float(base_axis_q), 0
        base_counts = float(base_axis_q) * float(sign) * float(counts_per_unit)
        # observed_counts must live in the SAME frame as base_counts
        # (raw/axis-q) for the nearest-turn round to be meaningful.
        # `observed_reference_counts` is the live 6064 value which lives
        # in the drive's single-turn WIRE frame; the raw/axis-q frame
        # differs by native_home_position_offset (= -607C after HM35).
        # Without this correction the fold frame-mismatches by RM/2 when
        # home sits at the midpoint of the wire sawtooth, and a small
        # canonical move snaps the rounded turn up by 1, emitting a
        # target one full revolution above the intended wire position.
        # This was the 2026-04-19 Move A 350-deg long-way excursion: the
        # drive saw 0x607A = RM + 618,951 instead of 618,951 and took
        # the long way to the same single-turn position.
        # `observed_reference_q` is already in axis-q space and needs no
        # correction; the live-counts fallback below also comes from
        # `_live_reference_counts_for_axis` which returns raw wire
        # counts and therefore needs the same correction applied.
        #
        # 2026-04-19 UI-whip fix (SEAM-ONLY DISAMBIGUATION): even with
        # the frame correction above, live 0x6064 is still AMBIGUOUS
        # right on the seam boundary. When J6 sits at canonical +/-180
        # deg, the drive reports 0x6064 = 0 or RM-1 depending on sub-
        # count encoder noise; these two readings are ONE FULL TURN
        # APART in the continuous frame, which flips
        # `round(delta/period)` between 0 and +/-1 and emits a target
        # one full revolution off. The two-jog UI case (jog +5 deg to
        # +180, wait, jog +5 deg to +185) triggered the whip this way.
        #
        # The fix uses the drive's UNAMBIGUOUS multi-turn register
        # (see `_multi_turn_reference_counts_for_axis`) to pick the
        # RIGHT side of the seam. This disambiguation ONLY fires when
        # the live wire reading is within a small tolerance of the
        # seam (< RM/16 ~= 22 deg); outside that band, single-turn
        # observed is unambiguous and the fold runs its original math
        # exactly as before. Wire emission is therefore IDENTICAL to
        # the pre-fix behavior everywhere except at the seam.
        #
        # PREFERENCE ORDER for observed_counts:
        #   1. Explicit `observed_reference_counts` (single-turn 0x6064).
        #      If `observed_multi_turn_reference_counts` is ALSO
        #      provided AND the wire value is seam-adjacent, pick the
        #      turn of live_6064 that is closest to the multi-turn
        #      reference (resolves the +/-RM jump at the seam). This is
        #      the primary path for live motion writes.
        #   2. Explicit `observed_reference_q` (axis-q space; used by
        #      the roundtrip diagnostic only).
        #   3. Auto-fetched live 0x6064 + auto-fetched multi-turn via
        #      the helpers. Same seam-adjacent disambiguation applies.
        #   4. Final fallback: auto-fetched live 0x6064 alone.
        native_home_offset_counts = int(self._native_home_offset_counts_for_axis(axis_i))

        def _disambiguate_seam_with_multi_turn(
            single_turn_wire_counts: int,
            ref_counts_axis_q: float,
            multi_turn_axis_q: float,
            period: int,
        ) -> float:
            # Inner helper: given the SINGLE-TURN wire counts reading
            # (which is what the drive reports in 0x6064, wrapped into
            # [0, period)), the axis-q-frame equivalent of that reading
            # (ref_counts_axis_q = wire + native_home_offset), the
            # unambiguous multi-turn reference in axis-q frame, and the
            # wrap period:
            #
            #   * detect whether the wire reading sits within a narrow
            #     seam-adjacent band (< period/16 ~= 22 deg from the
            #     0/RM boundary)
            #   * if at the seam AND the multi-turn indicates the OTHER
            #     side, shift ref_counts_axis_q by +/-period to move it
            #     onto the correct side
            #   * if NOT at the seam, return ref_counts_axis_q unchanged
            #     (preserves the pre-fix wire-emission behavior exactly)
            #
            # Returns the (possibly shifted) axis-q counts to use as
            # observed_counts in the fold's round(delta/period).
            seam_tolerance = int(period) // 16
            if seam_tolerance <= 0:
                return ref_counts_axis_q
            wire_mod = int(single_turn_wire_counts) % int(period)
            if wire_mod < 0:
                wire_mod += int(period)
            # Distance from wire to the nearest seam boundary (0 or
            # period). The seam is a single physical point so both
            # halves count as "at the seam" -- encoder noise can
            # sample on either side.
            distance_to_seam = min(wire_mod, int(period) - wire_mod)
            if distance_to_seam >= seam_tolerance:
                return ref_counts_axis_q
            # At the seam: pick the turn-shifted ref value that is
            # closest to the multi-turn reference. Modulo-N closeness:
            # the shift we apply is the one that minimises the
            # absolute continuous distance.
            shift = int(round((multi_turn_axis_q - ref_counts_axis_q) / float(period)))
            return ref_counts_axis_q + float(shift) * float(period)

        if observed_reference_counts is not None:
            ref_axis_q = (
                float(observed_reference_counts) + float(native_home_offset_counts)
            )
            if observed_multi_turn_reference_counts is not None:
                mt_axis_q = float(observed_multi_turn_reference_counts)
                observed_counts = _disambiguate_seam_with_multi_turn(
                    int(observed_reference_counts),
                    ref_axis_q,
                    mt_axis_q,
                    int(period_counts),
                )
            else:
                observed_counts = ref_axis_q
        elif observed_reference_q is not None:
            observed_counts = float(observed_reference_q) * float(sign) * float(counts_per_unit)
        else:
            live_counts = self._live_reference_counts_for_axis(axis_i)
            if live_counts is None:
                return float(base_axis_q), 0
            ref_axis_q = float(live_counts) + float(native_home_offset_counts)
            auto_multi_turn = self._multi_turn_reference_counts_for_axis(axis_i)
            if auto_multi_turn is not None:
                observed_counts = _disambiguate_seam_with_multi_turn(
                    int(live_counts),
                    ref_axis_q,
                    float(auto_multi_turn),
                    int(period_counts),
                )
            else:
                observed_counts = ref_axis_q
        # Step 1: classical nearest-turn fold so adjusted_counts lands
        # within RM/2 of the live reference (the diagnostic "wrap_lift"
        # we still expose for downstream roundtrip telemetry).
        delta = float(observed_counts) - float(base_counts)
        wrap_turns = int(round(delta / float(period_counts)))
        wrap_lift_counts = int(wrap_turns * int(period_counts))
        adjusted_counts = float(base_counts) + float(wrap_lift_counts)
        # Step 2 (command mode only): wrap into the drive's [0, RM)
        # single-turn presentation range. Python's integer math on
        # `float % positive` returns a value in [0, period), which is
        # exactly what the A6-EC rotation mode expects for 607A.
        if wrap_to_single_turn:
            period_float = float(period_counts)
            adjusted_counts = adjusted_counts - period_float * math.floor(
                adjusted_counts / period_float
            )
            # Defensive clamp in case of IEEE-754 drift near the seam.
            if adjusted_counts < 0.0:
                adjusted_counts += period_float
            if adjusted_counts >= period_float:
                adjusted_counts -= period_float
        adjusted_axis_q = float(adjusted_counts) / (float(sign) * float(counts_per_unit))
        return float(adjusted_axis_q), int(wrap_lift_counts)

    def _base_command_axis_q_for_joint_value(self, logical_joint_idx: int, canonical_q: float) -> float:
        return float(canonical_q) + self._master_offset_for_joint(logical_joint_idx)

    def _command_axis_q_for_joint_value(
        self,
        axis_i: int,
        logical_joint_idx: int,
        canonical_q: float,
        *,
        live_reference_counts: int | None = None,
        live_multi_turn_reference_counts: int | None = None,
    ) -> float:
        base_axis_q = self._base_command_axis_q_for_joint_value(logical_joint_idx, canonical_q)
        wrap_to_single_turn = not self._experimental_continuous_607a_enabled_for_joint(
            logical_joint_idx
        )
        # 2026-04-20 direction-preserving command path: when the drive is in
        # continuous-607A rotation mode AND we have a valid anchored multi-turn
        # register reading, the caller's `canonical_q` is already multi-turn-
        # aware (see `_canonical_joint_positions_from_raw_feedback` which
        # builds canonical from `absolute_axis_q - anchor - master_offset` using
        # U40.20/.22 as the truth source). In that regime the fold's nearest-
        # turn `round(delta/period)` shift is actively HARMFUL: it anchors on
        # live 0x6064 (single-turn modular) and can flip the commanded turn
        # between adjacent trajectory waypoints, producing a non-monotonic
        # wire-frame target path that RTCore chases at MAX RPM. The Phase 5
        # (canonical +365° → +180°) whip on 2026-04-20 was exactly this mode:
        # an s-curve trajectory in canonical space crossed the round() boundary
        # mid-trajectory, the per-waypoint folds disagreed on which turn, and
        # the motor oscillated through a full revolution at 6000 RPM before
        # settling at the correct net position. The operator's principled
        # insight: "the move is positive or negative - must rotate in a
        # specific direction" is literally satisfied by emitting axis_q =
        # base_axis_q without any turn-shift — axis-q counts are linearly
        # proportional to canonical, so a signed canonical delta maps to a
        # signed wire delta of the same sign, preserving direction.
        #
        # Gate on the stricter `_when_anchored` variant so a joint without a
        # captured home anchor (fresh boot, legacy profile) still falls
        # through to the fold path as before. Legacy single-turn drives
        # (wrap_to_single_turn=True) continue to use the fold unconditionally
        # because they require the [0, RM) wrap for drive-parse safety.
        use_direction_preserving = False
        if not wrap_to_single_turn and live_multi_turn_reference_counts is None:
            anchored_multi_turn = (
                self._multi_turn_reference_counts_for_axis_when_anchored(axis_i)
            )
            use_direction_preserving = anchored_multi_turn is not None
        if use_direction_preserving:
            # Direction-preserving path: trust the multi-turn-aware canonical
            # input. Skip the fold's turn-shift (Step 1 of
            # `_nearest_turn_fold_axis_q_for_axis`). The
            # `command_frame_oversized_delta` safety gate below still fires
            # if the emitted target would be modularly more than half a
            # period from the live wire — that catches real frame bugs
            # without inducing the whip.
            adjusted_axis_q = float(base_axis_q)
            lift_counts = 0
        else:
            # Legacy fold path: used when no anchor is captured OR the drive
            # is in legacy single-turn-wrap mode. Keep the permissive multi-
            # turn reference so the seam-only disambiguation still fires at
            # the 0/RM boundary (2026-04-19 UI-whip regression coverage).
            resolved_multi_turn = live_multi_turn_reference_counts
            if resolved_multi_turn is None:
                resolved_multi_turn = self._multi_turn_reference_counts_for_axis(axis_i)
            adjusted_axis_q, lift_counts = self._nearest_turn_fold_axis_q_for_axis(
                axis_i,
                base_axis_q,
                observed_reference_counts=live_reference_counts,
                observed_multi_turn_reference_counts=resolved_multi_turn,
                wrap_to_single_turn=wrap_to_single_turn,
            )
        period_counts = self._reference_wrap_period_counts_for_axis(axis_i)
        if period_counts > 0:
            # After folding, the signed SHORTEST-ANGULAR delta against the
            # live reference must fit inside half a shaft revolution. The
            # fold now wraps its output into [0, RM) so the drive can
            # unambiguously parse 607A, which means the raw linear delta
            # between adjusted_counts and live_counts can legitimately be
            # up to RM (when they are on opposite sides of the seam). We
            # compare the angular (mod-RM) residual instead so the gate
            # fires only when the host really did pick the wrong shaft
            # turn, not when adjusted happens to land on the far side of
            # the wrap boundary.
            cfg = self._axis_config
            if cfg is not None and 0 <= axis_i < int(cfg.num_axes):
                counts_per_unit = float(cfg.counts_per_unit[axis_i])
                sign = int(cfg.sign[axis_i])
                if counts_per_unit > 0.0 and sign in (-1, 1):
                    adjusted_counts = float(adjusted_axis_q) * float(sign) * float(counts_per_unit)
                    resolved_live_counts: float
                    if live_reference_counts is not None:
                        resolved_live_counts = float(live_reference_counts)
                    else:
                        live_counts = self._live_reference_counts_for_axis(axis_i)
                        resolved_live_counts = (
                            float(live_counts)
                            if live_counts is not None
                            else float(adjusted_counts) - float(lift_counts)
                        )
                    linear_delta_counts = adjusted_counts - resolved_live_counts
                    period_float = float(period_counts)
                    half_period = 0.5 * period_float
                    # Fold linear delta into (-RM/2, +RM/2] for a true
                    # shortest-angular-distance measurement. This matches
                    # how the drive computes motion under C10.16=0.
                    angular_delta_counts = (
                        (linear_delta_counts + half_period) % period_float
                    ) - half_period
                    if abs(angular_delta_counts) > half_period:
                        raise RuntimeError(
                            "command_frame_oversized_delta:"
                            f" axis={axis_i} joint={logical_joint_idx + 1}"
                            f" angular_delta_counts={angular_delta_counts:.1f}"
                            f" linear_delta_counts={linear_delta_counts:.1f}"
                            f" period_counts={int(period_counts)}"
                            f" live_counts={resolved_live_counts:.1f}"
                        )
        return float(adjusted_axis_q)

    def _canonical_joint_q_from_command_axis_q(
        self,
        axis_i: int,
        logical_joint_idx: int,
        target_axis_q: float,
    ) -> float:
        # The forward command path now folds stateless per write, so the
        # reverse (used only to record what canonical_q a given axis_q target
        # represented) re-folds against the live reference to recover a
        # single-turn-equivalent logical q without carrying stale lift state.
        cfg = self._axis_config
        period_counts = self._reference_wrap_period_counts_for_axis(axis_i)
        base_target_axis_q = float(target_axis_q)
        if cfg is not None and 0 <= axis_i < int(cfg.num_axes) and period_counts > 0:
            counts_per_unit = float(cfg.counts_per_unit[axis_i])
            sign = int(cfg.sign[axis_i])
            if counts_per_unit > 0.0 and sign in (-1, 1):
                live_counts = self._live_reference_counts_for_axis(axis_i)
                if live_counts is not None:
                    target_counts = float(target_axis_q) * float(sign) * float(counts_per_unit)
                    delta_counts = target_counts - float(live_counts)
                    wrap_turns = int(round(delta_counts / float(period_counts)))
                    base_counts = target_counts - float(wrap_turns * int(period_counts))
                    base_target_axis_q = float(base_counts) / (float(sign) * float(counts_per_unit))
        return float(base_target_axis_q) - self._master_offset_for_joint(logical_joint_idx)

    def _counts_tolerance_rad_for_axis(self, axis_i: int, counts: float) -> float:
        cfg = self._axis_config
        if cfg is None or axis_i < 0 or axis_i >= int(cfg.num_axes):
            return 1e-9
        counts_per_unit = float(cfg.counts_per_unit[axis_i])
        if counts_per_unit <= 0.0:
            return 1e-9
        return (float(counts) / counts_per_unit) + 1e-9

    def _command_roundtrip_tolerance_rad_for_axis(self, axis_i: int) -> float:
        # Live raw/absolute reads are sequential rather than simultaneous, and the
        # A6-EC probe work established that stationary bridges can wander by several
        # counts without indicating a semantic frame shift. Post-restart live soak
        # runs have since shown J6 occasionally spiking into the 7-9 count range
        # while the robot remains physically stationary, so keep the command-frame
        # guard above that observed jitter band while still rejecting larger
        # mismatches.
        return self._counts_tolerance_rad_for_axis(axis_i, _COMMAND_ROUNDTRIP_TOLERANCE_COUNTS)

    def _command_roundtrip_detail_for_axis(
        self,
        *,
        axis_i: int,
        logical_joint_idx: int,
        canonical_q: float,
        reference_q: float,
        reference_mode: str = "raw",
    ) -> dict[str, object]:
        normalized_reference_mode = str(reference_mode).strip().lower()
        base_roundtrip_reference_q = self._base_command_axis_q_for_joint_value(logical_joint_idx, canonical_q)
        wrap_lift_counts = 0
        if normalized_reference_mode == "raw":
            # Diagnostic view: fold the base reference to the same shaft turn
            # the observed 6064 is currently in. Purely informational here;
            # does NOT mutate any per-axis state.
            roundtrip_reference_q, wrap_lift_counts = self._nearest_turn_fold_axis_q_for_axis(
                axis_i,
                base_roundtrip_reference_q,
                observed_reference_q=float(reference_q),
            )
        else:
            roundtrip_reference_q = float(base_roundtrip_reference_q)
        error_rad = float(roundtrip_reference_q) - float(reference_q)
        tolerance_rad = self._command_roundtrip_tolerance_rad_for_axis(axis_i)
        detail: dict[str, object] = {
            "command_roundtrip_reference_rad": float(roundtrip_reference_q),
            "command_roundtrip_reference_error_rad": float(error_rad),
            "command_roundtrip_tolerance_rad": float(tolerance_rad),
            "command_roundtrip_consistent": abs(float(error_rad)) <= float(tolerance_rad),
        }
        if normalized_reference_mode == "raw":
            period_counts = self._reference_wrap_period_counts_for_axis(axis_i)
            detail["command_roundtrip_reference_base_rad"] = float(base_roundtrip_reference_q)
            detail["command_roundtrip_reference_wrap_lift_counts"] = int(wrap_lift_counts)
            if period_counts > 0:
                detail["command_roundtrip_reference_wrap_lift_turns"] = (
                    float(wrap_lift_counts) / float(period_counts)
                )
        cfg = self._axis_config
        if cfg is not None and 0 <= axis_i < int(cfg.num_axes):
            counts_per_unit = float(cfg.counts_per_unit[axis_i])
            sign = int(cfg.sign[axis_i])
            if counts_per_unit > 0.0 and sign in (-1, 1):
                detail["command_roundtrip_reference_error_counts"] = float(
                    float(error_rad) * float(sign) * float(counts_per_unit)
                )
        return detail

    def _absolute_home_anchor_stale_tolerance_rad_for_axis(self, axis_i: int) -> float:
        return self._counts_tolerance_rad_for_axis(axis_i, _ABSOLUTE_HOME_ANCHOR_STALE_TOLERANCE_COUNTS)

    def _shaft_frame_consistency_detail(
        self,
        *,
        axis_i: int,
        canonical_q: float,
        logical_joint_idx: int,
        live_reference_counts: int,
    ) -> dict[str, object] | None:
        # Compute the mod-RM distance between the anchored canonical_q view
        # (expressed in 607A/6064 wire counts) and the live 6064 reading.
        # Whole-shaft-turn offsets between the two are legitimate (they just
        # encode which shaft turn the joint is currently on); only a
        # sub-shaft-turn disagreement is a frame bug.
        cfg = self._axis_config
        if cfg is None or axis_i < 0 or axis_i >= int(cfg.num_axes):
            return None
        period_counts = self._reference_wrap_period_counts_for_axis(axis_i)
        if period_counts <= 0:
            return None
        counts_per_unit = float(cfg.counts_per_unit[axis_i])
        sign = int(cfg.sign[axis_i])
        if counts_per_unit <= 0.0 or sign not in (-1, 1):
            return None
        # canonical_q is the planner-space value (after anchor and
        # master_offset subtracted); the expected reference frame is what
        # 6064 should read if canonical_q and the live multi-turn anchor
        # were fully consistent. Re-apply the master offset, convert to
        # counts in the wire frame, and compare modulo RM.
        expected_reference_q = float(canonical_q) + self._master_offset_for_joint(logical_joint_idx)
        expected_counts = float(expected_reference_q) * float(sign) * float(counts_per_unit)
        live_reference_logical_counts = float(live_reference_counts) + float(
            self._native_home_offset_counts_for_axis(axis_i)
        )
        delta_counts = float(expected_counts) - float(live_reference_logical_counts)
        # Fold to nearest shaft turn so the mod-RM distance is the
        # sub-shaft-turn residual.
        period = float(period_counts)
        wrap_turns = int(round(delta_counts / period))
        mod_rm_delta_counts = delta_counts - float(wrap_turns) * period
        mod_rm_delta_rad = mod_rm_delta_counts / (float(sign) * float(counts_per_unit))
        tolerance_rad = self._counts_tolerance_rad_for_axis(
            axis_i,
            _SHAFT_FRAME_CONSISTENCY_TOLERANCE_COUNTS,
        )
        consistent = abs(float(mod_rm_delta_rad)) <= float(tolerance_rad)
        return {
            "shaft_frame_consistent": bool(consistent),
            "shaft_frame_mod_rm_delta_counts": float(mod_rm_delta_counts),
            "shaft_frame_mod_rm_delta_rad": float(mod_rm_delta_rad),
            "shaft_frame_tolerance_counts": float(_SHAFT_FRAME_CONSISTENCY_TOLERANCE_COUNTS),
            "shaft_frame_tolerance_rad": float(tolerance_rad),
            "shaft_frame_period_counts": int(period_counts),
            "shaft_frame_wrap_turns": int(wrap_turns),
            "shaft_frame_expected_reference_counts": float(expected_counts),
            "shaft_frame_live_reference_counts": int(live_reference_counts),
            "shaft_frame_live_reference_logical_counts": float(live_reference_logical_counts),
        }

    def _absolute_home_anchor_diagnostic_for_axis(
        self,
        *,
        axis_i: int,
        axis_snapshot: dict[str, object] | None,
        absolute_axis_q: float,
        reference_q: float,
        anchor_entry: dict[str, Any],
    ) -> dict[str, object]:
        stored_anchor_rad = float(anchor_entry.get("home_anchor_rad", 0.0))
        implied_anchor_rad = float(absolute_axis_q) - float(reference_q)
        anchor_delta_rad = float(implied_anchor_rad) - float(stored_anchor_rad)
        stale_tolerance_rad = self._absolute_home_anchor_stale_tolerance_rad_for_axis(axis_i)
        detail: dict[str, object] = {
            "absolute_home_anchor_implied_rad": float(implied_anchor_rad),
            "absolute_home_anchor_delta_rad": float(anchor_delta_rad),
            "absolute_home_anchor_stale_tolerance_rad": float(stale_tolerance_rad),
        }
        cfg = self._axis_config
        if cfg is not None and 0 <= axis_i < int(cfg.num_axes):
            counts_per_unit = float(cfg.counts_per_unit[axis_i])
            sign = int(cfg.sign[axis_i])
            if counts_per_unit > 0.0 and sign in (-1, 1):
                detail["absolute_home_anchor_delta_counts"] = float(
                    float(anchor_delta_rad) * float(sign) * float(counts_per_unit)
                )
                detail["absolute_home_anchor_stale_tolerance_counts"] = float(
                    float(stale_tolerance_rad) * float(counts_per_unit)
                )

        axis_payload = axis_snapshot if isinstance(axis_snapshot, dict) else {}
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
        native_home_status = derive_effective_native_home_status(
            axis_payload,
            statusword=statusword,
            error_code=error_code,
            manufacturer_error_code=manufacturer_error_code,
        )
        native_home_active = bool(axis_payload.get("native_home_active", False))
        drive_clean = (
            statusword != 0
            and error_code == 0
            and manufacturer_error_code == 0
            and int(native_home_status.get("native_home_state", 0)) == 2
            and not native_home_active
        )
        if "slave_online" in axis_payload:
            drive_clean = drive_clean and bool(axis_payload.get("slave_online"))
        if "slave_operational" in axis_payload:
            drive_clean = drive_clean and bool(axis_payload.get("slave_operational"))
        detail.update(
            {
                "statusword": int(statusword),
                "statusword_hex": f"0x{statusword & 0xFFFF:04X}",
                "error_code": int(error_code),
                "error_code_hex": f"0x{error_code & 0xFFFF:04X}",
                "manufacturer_error_code": int(manufacturer_error_code),
                "manufacturer_error_code_hex": f"0x{manufacturer_error_code & 0xFFFFFFFF:08X}",
                "native_home_state": int(native_home_status.get("native_home_state", 0)),
                "native_home_state_name": str(native_home_status.get("native_home_state_name", "idle")),
                "native_home_last_abort_code": int(native_home_status.get("native_home_last_abort_code", 0)),
                "native_home_last_abort_code_hex": str(
                    native_home_status.get("native_home_last_abort_code_hex", "0x00000000")
                ),
                "native_home_verification_source": str(
                    native_home_status.get("native_home_verification_source", "reported")
                ),
                "native_home_active": bool(native_home_active),
                "absolute_home_anchor_drive_clean": bool(drive_clean),
                "absolute_home_anchor_stale": bool(
                    drive_clean and abs(float(anchor_delta_rad)) > float(stale_tolerance_rad)
                ),
            }
        )
        return detail

    def _absolute_home_anchor_validation_for_joint(
        self,
        logical_joint_index: int,
        *,
        raw_positions: dict[int, int] | None = None,
        reference_mode: str = "raw",
    ) -> dict[str, object]:
        joint_i = int(logical_joint_index)
        if joint_i < 0 or joint_i >= self._num_joints:
            return {
                "ok": False,
                "truth_available": False,
                "truth_reason": "logical_joint_unmapped",
            }
        if raw_positions is None:
            raw_positions = self.sync_read_positions()
        if not isinstance(raw_positions, dict) or not raw_positions:
            return {
                "ok": False,
                "truth_available": False,
                "truth_reason": "raw_feedback_missing",
            }
        target_axes = [axis_i for axis_i, mapped_joint in enumerate(self._axis_to_joint) if mapped_joint == joint_i]
        if not target_axes:
            return {
                "ok": False,
                "truth_available": False,
                "truth_reason": "logical_joint_unmapped",
            }
        snapshot = self._canonical_joint_positions_from_raw_feedback(
            raw_positions,
            reference_mode=reference_mode,
        )
        axis_truth_details = snapshot.get("axis_absolute_feedback")
        if not isinstance(axis_truth_details, list):
            return {
                "ok": False,
                "truth_available": False,
                "truth_reason": "truth_snapshot_unavailable",
            }
        matching_details = [
            detail
            for detail in axis_truth_details
            if isinstance(detail, dict) and int(detail.get("axis", -1)) in target_axes
        ]
        if not matching_details:
            return {
                "ok": False,
                "truth_available": False,
                "truth_reason": "truth_snapshot_unavailable",
            }
        failing_detail = next(
            (detail for detail in matching_details if not bool(detail.get("truth_available", False))),
            None,
        )
        if failing_detail is not None:
            result: dict[str, object] = {
                "ok": False,
                "truth_available": False,
                "truth_reason": str(failing_detail.get("truth_reason", "truth_unavailable") or "truth_unavailable"),
            }
            for key in (
                "axis",
                "logical_joint",
                "command_roundtrip_reference_error_counts",
                "command_roundtrip_reference_error_rad",
                "absolute_home_anchor_delta_counts",
                "absolute_home_anchor_delta_rad",
                "absolute_home_anchor_implied_rad",
                "shaft_frame_consistent",
                "shaft_frame_mod_rm_delta_counts",
                "shaft_frame_mod_rm_delta_rad",
                "shaft_frame_tolerance_counts",
                "shaft_frame_tolerance_rad",
                "shaft_frame_period_counts",
                "shaft_frame_wrap_turns",
            ):
                value = failing_detail.get(key)
                if value is not None:
                    result[key] = value
            return result
        primary_detail = matching_details[0]
        result = {
            "ok": True,
            "truth_available": True,
        }
        for key in ("axis", "logical_joint", "canonical_rad"):
            value = primary_detail.get(key)
            if value is not None:
                result[key] = value
        return result

    def _capture_absolute_home_anchor_for_joint(
        self,
        logical_joint_index: int,
        *,
        raw_positions: dict[int, int] | None = None,
        actor: str = "unknown",
        reference_mode: str = "raw",
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
            reference_q = self._reference_q_before_master_offset_for_axis(
                axis_i,
                int(raw_counts),
                reference_mode=reference_mode,
            )
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

    def _bootstrap_missing_absolute_home_anchors(self, *, actor: str) -> dict[str, object]:
        if not self._absolute_home_anchor_required():
            return {
                "created_joints": [],
                "missing_joints": [],
            }
        target_joint_indices = sorted({joint_i for joint_i in self._axis_to_joint if 0 <= joint_i < self._num_joints})
        if not target_joint_indices:
            target_joint_indices = list(range(self._num_joints))
        if not self._connected:
            return {
                "created_joints": [],
                "missing_joints": [joint_i + 1 for joint_i in target_joint_indices],
            }
        if self._axis_config is None and not self._wait_for_feedback_ready(timeout_s=0.25, require_live_wkc=False):
            return {
                "created_joints": [],
                "missing_joints": [joint_i + 1 for joint_i in target_joint_indices],
            }
        raw_positions = self.sync_read_positions()
        if not raw_positions:
            return {
                "created_joints": [],
                "missing_joints": [joint_i + 1 for joint_i in target_joint_indices],
            }

        created_joints: list[int] = []
        missing_joints: list[int] = []
        for joint_i in target_joint_indices:
            if self._absolute_home_anchor_for_joint(joint_i) is not None:
                continue
            captured = self._capture_absolute_home_anchor_for_joint(
                joint_i,
                raw_positions=raw_positions,
                actor=actor,
                reference_mode="display",
            )
            if captured is not None:
                created_joints.append(joint_i + 1)
            else:
                missing_joints.append(joint_i + 1)

        if created_joints:
            print(
                "[EtherCAT RTCore] Bootstrapped absolute-home anchors from live display/absolute alignment:"
                f" joints={created_joints} actor={actor}"
            )
        if missing_joints:
            print(
                "[EtherCAT RTCore] Absolute-home anchors still missing after bootstrap:"
                f" joints={missing_joints}"
            )
        return {
            "created_joints": created_joints,
            "missing_joints": missing_joints,
        }

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
                and statusword_indicates_valid_native_home_reference(statusword)
            ):
                # Keep the command-result semantics aligned with the live driveFaults
                # view: once RTCore's native-home tail is done, a clean live wire-state
                # with vendor-confirmed HM success bits should override stale
                # last-operation failure fields.
                effective_state = 2
                effective_abort_code = 0
                verification_source = "statusword_bits12_15_clear13"
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
        def _pending_from_last_result(result: dict[str, object]) -> dict[str, object]:
            return {
                "verified": False,
                "timed_out": True,
                "terminal_state": "pending",
                "native_home_state": 1,
                "native_home_state_name": "requested",
                "native_home_last_abort_code": 0,
                "native_home_last_abort_code_hex": "0x00000000",
                "metrics_time_ns": int(result.get("metrics_time_ns", 0) or 0),
                "axis_results": (
                    [dict(value) for value in result.get("axis_results", [])]
                    if isinstance(result.get("axis_results"), list)
                    else []
                ),
            }

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
        saw_clear_after_active = False
        failed_after_clear_snapshots = 0
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
                    allow_statusword_fallback = saw_active_mask and not active_for_target
                    if allow_statusword_fallback:
                        saw_clear_after_active = True
                    last_result = self._native_home_metrics_result(
                        target_axes,
                        snapshot=snapshot,
                        allow_statusword_fallback=allow_statusword_fallback,
                    )
                    terminal_state = str(last_result.get("terminal_state", "pending"))
                    if allow_statusword_fallback:
                        if terminal_state == "succeeded":
                            last_result["timed_out"] = False
                            return last_result
                        if terminal_state == "failed":
                            failed_after_clear_snapshots += 1
                            if failed_after_clear_snapshots >= _NATIVE_HOME_FAILED_STABILIZATION_SNAPSHOTS:
                                last_result["timed_out"] = False
                                return last_result
                        else:
                            failed_after_clear_snapshots = 0
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
        terminal_state = str(last_result.get("terminal_state", "pending"))
        if saw_active_mask:
            if terminal_state == "succeeded":
                last_result["timed_out"] = False
                return last_result
            if (
                saw_clear_after_active
                and terminal_state == "failed"
                and failed_after_clear_snapshots >= _NATIVE_HOME_FAILED_STABILIZATION_SNAPSHOTS
            ):
                last_result["timed_out"] = False
                return last_result
            return _pending_from_last_result(last_result)
        return _pending_from_last_result(last_result)

    def _native_home_post_settle_result(
        self,
        target_axes: list[int],
        *,
        snapshot: dict[str, object],
    ) -> dict[str, object]:
        axes = snapshot.get("axes")
        axis_results: list[dict[str, object]] = []
        if not isinstance(axes, list):
            axes = []
        try:
            active_mask = int(snapshot.get("native_home_active_axis_mask", 0) or 0)
        except Exception:
            active_mask = 0

        all_clean = bool(target_axes)
        first_nonclean_reason = ""
        hard_failure = False
        for axis_i in target_axes:
            axis_payload = axes[axis_i] if axis_i < len(axes) and isinstance(axes[axis_i], dict) else {}
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
            try:
                state = int(axis_payload.get("native_home_state", 0))
            except Exception:
                state = 0
            try:
                abort_code = int(axis_payload.get("native_home_last_abort_code", 0))
            except Exception:
                abort_code = 0

            reported_active = bool(axis_payload.get("native_home_active", False))
            active = reported_active or ((active_mask & (1 << axis_i)) != 0)
            statusword_fault = (statusword & 0x0008) != 0
            slave_online_raw = axis_payload.get("slave_online")
            slave_operational_raw = axis_payload.get("slave_operational")
            slave_online = True if slave_online_raw is None else bool(slave_online_raw)
            slave_operational = True if slave_operational_raw is None else bool(slave_operational_raw)

            failure_reason = ""
            if not slave_online:
                failure_reason = "slave_offline"
            elif not slave_operational:
                failure_reason = "slave_not_operational"
            elif error_code != 0 or manufacturer_error_code != 0 or statusword_fault:
                failure_reason = "drive_faulted"
            elif active:
                failure_reason = "native_home_active"

            axis_clean = not failure_reason
            if not axis_clean:
                all_clean = False
                if not first_nonclean_reason:
                    first_nonclean_reason = failure_reason or "post_home_unclean"
                if failure_reason in {"slave_offline", "slave_not_operational", "drive_faulted"}:
                    hard_failure = True

            axis_results.append(
                {
                    "axis": int(axis_i),
                    "native_home_state": int(state),
                    "native_home_state_name": _native_home_state_name(state),
                    "native_home_last_abort_code": int(abort_code),
                    "native_home_last_abort_code_hex": f"0x{abort_code & 0xFFFFFFFF:08X}",
                    "statusword": int(statusword),
                    "statusword_hex": f"0x{statusword & 0xFFFF:04X}",
                    "statusword_fault": bool(statusword_fault),
                    "error_code": int(error_code),
                    "error_code_hex": f"0x{error_code & 0xFFFF:04X}",
                    "manufacturer_error_code": int(manufacturer_error_code),
                    "manufacturer_error_code_hex": f"0x{manufacturer_error_code & 0xFFFFFFFF:08X}",
                    "slave_online": bool(slave_online),
                    "slave_operational": bool(slave_operational),
                    "native_home_active": bool(active),
                    "failure_reason": failure_reason or None,
                    "clean": bool(axis_clean),
                }
            )

        return {
            "ok": bool(axis_results) and bool(all_clean),
            "timed_out": False,
            "hard_failure": bool(hard_failure),
            "failure_reason": first_nonclean_reason or None,
            "metrics_time_ns": int(snapshot.get("time_ns", 0) or 0),
            "axis_results": axis_results,
        }

    def _wait_for_native_home_post_settle_result(
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
                "ok": False,
                "timed_out": False,
                "hard_failure": False,
                "failure_reason": "invalid_axis_mask",
                "metrics_time_ns": 0,
                "axis_results": [],
            }

        settle_timeout_s = max(0.0, float(timeout_s))
        if settle_timeout_s <= 0.0:
            return {
                "ok": True,
                "timed_out": False,
                "hard_failure": False,
                "failure_reason": None,
                "metrics_time_ns": int(min_metrics_time_ns),
                "axis_results": [],
            }

        deadline = time.monotonic() + settle_timeout_s
        last_result: dict[str, object] | None = None
        saw_fresh_snapshot = False
        saw_unclean_snapshot = False
        first_unclean_reason = ""
        newest_time_ns = int(min_metrics_time_ns)
        newest_mtime_ns = int(min_metrics_mtime_ns)

        while time.monotonic() <= deadline:
            snapshot = self._load_rtcore_metrics_snapshot()
            if isinstance(snapshot, dict):
                snapshot_time_ns = int(snapshot.get("time_ns", 0) or 0)
                snapshot_mtime_ns = int(snapshot.get("_mtime_ns", 0) or 0)
                if snapshot_time_ns > newest_time_ns or snapshot_mtime_ns > newest_mtime_ns:
                    newest_time_ns = max(newest_time_ns, snapshot_time_ns)
                    newest_mtime_ns = max(newest_mtime_ns, snapshot_mtime_ns)
                    saw_fresh_snapshot = True
                    last_result = self._native_home_post_settle_result(target_axes, snapshot=snapshot)
                    if not bool(last_result.get("ok", False)):
                        saw_unclean_snapshot = True
                        reason = str(last_result.get("failure_reason", "") or "").strip()
                        if reason and not first_unclean_reason:
                            first_unclean_reason = reason
                    if bool(last_result.get("hard_failure", False)):
                        return last_result
            time.sleep(_NATIVE_HOME_WAIT_POLL_INTERVAL_S)

        if not saw_fresh_snapshot:
            return {
                "ok": False,
                "timed_out": True,
                "hard_failure": False,
                "failure_reason": "post_home_settle_no_fresh_metrics",
                "metrics_time_ns": 0,
                "axis_results": [],
            }
        if last_result is None:
            return {
                "ok": False,
                "timed_out": True,
                "hard_failure": False,
                "failure_reason": "post_home_settle_unknown",
                "metrics_time_ns": int(newest_time_ns),
                "axis_results": [],
            }
        if bool(last_result.get("ok", False)) and not saw_unclean_snapshot:
            return last_result
        result = dict(last_result)
        result["ok"] = False
        result["timed_out"] = True
        result["hard_failure"] = bool(result.get("hard_failure", False))
        result["failure_reason"] = (
            first_unclean_reason
            or str(result.get("failure_reason", "") or "").strip()
            or "post_home_settle_unstable"
        )
        return result

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
                # The persisted absolute-home anchor is defined as:
                #   absolute_axis_q - reference_q
                # so canonical truth becomes:
                #   absolute_axis_q - absolute_home_anchor - software_zero
                # = reference_q - software_zero
                # The command path inverts back to the reference frame by
                # re-applying the software-zero offset; turn selection is
                # handled by a stateless nearest-turn fold against live 6064
                # inside _command_axis_q_for_joint_value, so there is no cached
                # per-axis wrap state that can go stale between writes.
                axis_q[axis_i] = self._command_axis_q_for_joint_value(
                    axis_i,
                    joint_i,
                    float(positions_rad[joint_i]),
                )
        return axis_q

    def _enforce_trajectory_wire_frame_safety(
        self,
        *,
        axis_q: list[float],
        axis_mask: int,
        previous_axis_counts: list[int | None],
        initial_live_counts: list[int | None],
        point_index: int,
        traj_id: int,
    ) -> None:
        # Per-point wire-frame safety cage (2026-04-17 J6 incident hardening).
        #
        # For every axis actually included in the trajectory:
        #   1. On point 0 specifically, the queued target must stay within
        #      a tight joint-space bound of the drive's live 6064 at upload
        #      time. The fold is supposed to land point 0 essentially on
        #      top of live 6064 (since its canonical_q is drawn from live
        #      feedback); any bigger deviation means the fold or the
        #      drive's rotation-mode config picked the wrong shaft turn,
        #      and executing that trajectory would teleport the joint by
        #      roughly a full shaft revolution.
        #   2. On every point, the step between this point and the previous
        #      point must stay inside a tight joint-space bound. This
        #      catches mid-trajectory fold flips (e.g. live 6064 drifted
        #      between upload calls) before RTCore executes them.
        #
        # Both bounds are in joint-space radians so the threshold means the
        # same operator-facing motion envelope on every axis regardless of
        # gear ratio. Long bounded moves are safe because they are planned
        # from many small per-point steps; only pathological wrap-turn
        # mis-selections ever violate these bounds, and when they do we
        # fail closed BEFORE the drive ever sees the upload.
        cfg = self._axis_config
        if cfg is None:
            return
        seam_crossing_unsafe = self._command_frame_seam_crossing_unsafe()
        num_axes = min(int(cfg.num_axes), self._rt_num_axes, len(axis_q))
        for axis_i in range(num_axes):
            if axis_mask != 0 and (axis_mask & (1 << axis_i)) == 0:
                continue
            logical_joint_idx = (
                self._axis_to_joint[axis_i] if 0 <= axis_i < len(self._axis_to_joint) else axis_i
            )
            seam_crossing_guard_enabled = (
                seam_crossing_unsafe
                and not self._experimental_continuous_607a_enabled_for_joint(logical_joint_idx)
            )
            counts_per_unit = float(cfg.counts_per_unit[axis_i])
            sign = int(cfg.sign[axis_i])
            if counts_per_unit <= 0.0 or sign not in (-1, 1):
                continue
            current_counts = int(
                round(float(axis_q[axis_i]) * float(sign) * float(counts_per_unit))
            )
            period_counts = self._reference_wrap_period_counts_for_axis(axis_i)
            max_step_counts = int(
                round(_TRAJECTORY_MAX_PER_POINT_STEP_RAD * float(counts_per_unit))
            )
            prev = previous_axis_counts[axis_i]
            if prev is not None and max_step_counts > 0:
                # After the 2026-04-17 fold wrap-to-[0, RM) change, two
                # legitimate consecutive points can sit on opposite sides
                # of the wrap boundary (e.g. 1,310,700 -> 50 is physically
                # a +70-count step, not a -1,310,650-count one). Compare
                # on the SHORTEST ANGULAR distance so we only trip on real
                # mid-trajectory wrap-turn mis-selections.
                linear_step = current_counts - int(prev)
                if period_counts > 0:
                    period_float = float(period_counts)
                    half_period = 0.5 * period_float
                    step_counts = int(round(
                        ((float(linear_step) + half_period) % period_float) - half_period
                    ))
                else:
                    step_counts = linear_step
                if abs(step_counts) > max_step_counts:
                    raise RuntimeError(
                        "command_frame_oversized_step:"
                        f" axis={axis_i} joint={axis_i + 1}"
                        f" point_index={point_index} traj_id={traj_id}"
                        f" step_counts={step_counts}"
                        f" linear_step_counts={linear_step}"
                        f" max_step_counts={max_step_counts}"
                        f" max_step_rad={_TRAJECTORY_MAX_PER_POINT_STEP_RAD:.3f}"
                    )
                if period_counts > 0 and seam_crossing_guard_enabled and abs(linear_step) > half_period:
                    # Live A6-EC verification showed that seam-straddling
                    # absolute 607A point sequences can still execute the long
                    # way by ~RM counts even when the shortest-angular step is
                    # tiny. Profiles that opt into this guard fail closed until
                    # a seam-biased wire-frame policy is validated on hardware.
                    raise RuntimeError(
                        "command_frame_seam_crossing_step_disallowed:"
                        f" axis={axis_i} joint={axis_i + 1}"
                        f" point_index={point_index} traj_id={traj_id}"
                        f" step_counts={step_counts}"
                        f" linear_step_counts={linear_step}"
                        f" period_counts={int(period_counts)}"
                    )
            # Only gate point 0 against live 6064; later points may legitimately
            # travel far from live by the end of a long bounded move.
            if prev is None:
                live_counts = initial_live_counts[axis_i]
                max_live_deviation_counts = int(
                    round(
                        _TRAJECTORY_MAX_FIRST_POINT_DEVIATION_FROM_LIVE_RAD
                        * float(counts_per_unit)
                    )
                )
                if live_counts is not None and max_live_deviation_counts > 0:
                    comparison_live_counts = self._logicalized_live_reference_counts_for_axis(
                        axis_i,
                        logical_joint_idx=logical_joint_idx,
                        live_reference_counts=live_counts,
                    )
                    if comparison_live_counts is None:
                        comparison_live_counts = int(live_counts)
                    # Same shortest-angular treatment as per-point step:
                    # after wrap-to-[0, RM), point 0's `current_counts`
                    # can be on the opposite side of the seam from
                    # live_6064 even when the physical delta is tiny.
                    linear_deviation = current_counts - int(comparison_live_counts)
                    if period_counts > 0:
                        period_float = float(period_counts)
                        half_period = 0.5 * period_float
                        deviation_counts = int(round(
                            ((float(linear_deviation) + half_period) % period_float) - half_period
                        ))
                    else:
                        deviation_counts = linear_deviation
                    if abs(deviation_counts) > max_live_deviation_counts:
                        raise RuntimeError(
                            "command_frame_live_deviation_out_of_range:"
                            f" axis={axis_i} joint={axis_i + 1}"
                            f" point_index={point_index} traj_id={traj_id}"
                            f" target_counts={current_counts}"
                            f" live_counts={int(comparison_live_counts)}"
                            f" deviation_counts={deviation_counts}"
                            f" linear_deviation_counts={linear_deviation}"
                            f" max_deviation_counts={max_live_deviation_counts}"
                            f" max_deviation_rad={_TRAJECTORY_MAX_FIRST_POINT_DEVIATION_FROM_LIVE_RAD:.3f}"
                        )
                    if period_counts > 0 and seam_crossing_guard_enabled and abs(linear_deviation) > half_period:
                        raise RuntimeError(
                            "command_frame_seam_crossing_first_point_disallowed:"
                            f" axis={axis_i} joint={axis_i + 1}"
                            f" point_index={point_index} traj_id={traj_id}"
                            f" target_counts={current_counts}"
                            f" live_counts={int(comparison_live_counts)}"
                            f" deviation_counts={deviation_counts}"
                            f" linear_deviation_counts={linear_deviation}"
                            f" period_counts={int(period_counts)}"
                        )
            previous_axis_counts[axis_i] = current_counts

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

