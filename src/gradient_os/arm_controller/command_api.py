# Contains the high-level command handlers that parse and react to UDP messages.
import os
import json
import sys
import time
from scipy.spatial.transform import Slerp  # needed for reset/initial path slerp interpolation
import numpy as np
from scipy.spatial.transform import Rotation as R
import threading
import datetime
from typing import Sequence

# --- Global State for Motion Control ---
# The motion_stopped Event has been removed to use the native trajectory_state flag.

try:
    from .. import ik_solver
except ImportError:
    print("ERROR: Missing 'ik_solver'. Ensure it is in the python path.")
    ik_solver = None
    trajectory_planner = None

from . import utils
from . import robot_config
from . import servo_driver
from . import trajectory_execution
from . import pid_tuner
from .jog_session import JogSessionError, JogSessionManager
from .backends import registry as backend_registry
from ..kinematics import runtime as kinematics_runtime
from ..telemetry import alerts as telemetry_alerts

_DIRECT_SETPOINT_FALLBACK_SPEED = 500
_DIRECT_SETPOINT_FALLBACK_ACCELERATION_DEG_S2 = 500.0
_SAFE_JOINT_MOVE_FREQUENCY_HZ = 100
_SAFE_JOINT_MOVE_MIN_DURATION_S = 0.25
_PROGRAM_JOINT_MOVE_MAX_MOTOR_RPM = 100.0
_WAIT_FOR_IDLE_DEFAULT_TIMEOUT_S = 30.0
_WAIT_FOR_IDLE_POLL_INTERVAL_S = 0.01
_RTCORE_ACK_REFRESH_TIMEOUT_S = 0.15
_RTCORE_ACK_REFRESH_POLL_INTERVAL_S = 0.005
_POWER_TRANSITION_DEFAULT_TIMEOUT_S = 1.0
_RTCORE_TERMINAL_EXECUTION_STATES = {"idle", "completed", "aborted", "faulted", "underrun"}
_PROGRAM_TERMINAL_STATES = {"completed", "aborted", "faulted", "timeout", "interrupted"}
_PROGRAM_ACTIVE_STATES = {"planning", "accepted", "executing"}
_DS402_OPERATION_ENABLED_STATE = 5
_JOG_PERF_LOCK = threading.Lock()

# Phase 0 (2026-04-20 canonical-truth stability): the 50 Hz jog feedback
# read used to be able to raise a RuntimeError (canonical joint truth
# unavailable) on any transient flicker from the shaft-frame gate, which
# would kill the daemon jog thread silently and halt motion. The handler
# below records the event so operators can measure the flicker rate and
# tells the loop to reuse cached q_current instead of crashing. The log
# emission is throttled to at most once per second so a pathological
# flicker burst cannot flood the controller log.
_JOG_TRUTH_FLICKER_LOG_THROTTLE_S = 1.0

# 2026-04-21 canonical-truth settling tolerance at jog arm time. The
# A6-EC's servo loop takes 0.5-2 s after SAFE_POWER_UP (or a fresh
# SAFE_POWER_DOWN→SAFE_POWER_UP cycle) before the command-frame
# roundtrip gate stops tripping on transient drive-settling mismatches.
# `_JOG_ARM_TRUTH_RETRY_BUDGET_S` is the hard ceiling for how long we
# will wait for canonical truth to become available before rejecting
# a jog session start. `_JOG_ARM_TRUTH_RETRY_INTERVAL_S` is the per-
# attempt sleep so we don't spin the CPU. A real encoder-retention
# fault will NOT clear inside this window, so the safety intent of
# the original strict check is preserved: we still refuse to arm if
# the drive legitimately cannot prove its position, but we don't
# reject the user's first jog attempt 200 ms after power-up just
# because the drive is still decelerating.
_JOG_ARM_TRUTH_RETRY_BUDGET_S = 0.5
_JOG_ARM_TRUTH_RETRY_INTERVAL_S = 0.05

# 2026-04-21 (pass 4): the retry budget above bounded the worst case
# but still imposed up to 500 ms of blocking on every click-after-
# release because the A6-EC's deceleration window after releasing a
# jog button repeatedly trips the command-frame-roundtrip gate while
# the servo loop settles. Operators reported this as the dominant
# source of jog lag: "release → click → wait almost a full second
# before motion starts". Fix: if canonical truth has been observed
# valid within this window, accept the jog immediately and skip the
# retry loop entirely. Phase 0's per-tick try/except already absorbs
# any transient flicker that occurs DURING the newly started session.
# A real encoder-retention fault would not have produced a recent
# valid reading so the strict fallback path still runs for first-boot
# and post-power-cycle scenarios.
_JOG_ARM_RECENT_TRUTH_WINDOW_S = 5.0
_LAST_VALID_CANONICAL_TRUTH_MONOTONIC: float | None = None
_LAST_VALID_CANONICAL_TRUTH_LOCK = threading.Lock()


def _note_valid_canonical_truth() -> None:
    """Stamp the last-moment canonical-truth was observed valid.
    Called from the hot path on every successful
    ``get_control_arm_state_rad`` read. The jog thread's feedback
    read (50 Hz) alone keeps this timestamp within ~20 ms of fresh
    during any active session, so `_recently_valid_canonical_truth`
    stays `True` continuously for at least
    ``_JOG_ARM_RECENT_TRUTH_WINDOW_S`` seconds after the user releases
    the jog button."""
    stamp = time.monotonic()
    with _LAST_VALID_CANONICAL_TRUTH_LOCK:
        global _LAST_VALID_CANONICAL_TRUTH_MONOTONIC
        _LAST_VALID_CANONICAL_TRUTH_MONOTONIC = stamp


def _recently_valid_canonical_truth(window_s: float | None = None) -> bool:
    if window_s is None:
        window_s = _JOG_ARM_RECENT_TRUTH_WINDOW_S
    with _LAST_VALID_CANONICAL_TRUTH_LOCK:
        ts = _LAST_VALID_CANONICAL_TRUTH_MONOTONIC
    if ts is None:
        return False
    return (time.monotonic() - ts) < float(window_s)


def _new_jog_perf_state() -> dict[str, object]:
    return {
        "control_frequency_hz": 0,
        "execution_policy": "",
        "rtcore_owned": False,
        "last_velocity_command_age_s": None,
        "velocity_updates": {
            "count": 0,
            "gap_count": 0,
            "avg_gap_ms": 0.0,
            "max_gap_ms": 0.0,
            "last_gap_ms": None,
            "last_update_monotonic_s": None,
            "zero_velocity_updates": 0,
            "nonzero_velocity_updates": 0,
        },
        "loop": {
            "count": 0,
            "avg_ms": 0.0,
            "max_ms": 0.0,
            "last_ms": 0.0,
            "overrun_count": 0,
            "max_overrun_ms": 0.0,
            "last_overrun_ms": 0.0,
        },
        "stages": {
            "feedback_read_ms": {"count": 0, "avg_ms": 0.0, "max_ms": 0.0, "last_ms": 0.0},
            "ik_solve_ms": {"count": 0, "avg_ms": 0.0, "max_ms": 0.0, "last_ms": 0.0},
            "jacobian_compute_ms": {"count": 0, "avg_ms": 0.0, "max_ms": 0.0, "last_ms": 0.0},
            "command_send_ms": {"count": 0, "avg_ms": 0.0, "max_ms": 0.0, "last_ms": 0.0},
        },
        "command_state_valid": False,
        "commanded_pose": None,
        "commanded_joints_deg": None,
        "last_accepted_target_pose": None,
        "measured_pose": None,
        "measured_joints_deg": None,
        "following_error": None,
        "last_resync_reason": None,
        "last_resync_age_s": None,
        "last_gate_failure_reason": None,
        "last_gate_failure_details": None,
        "ik_debug": None,
        "truth_flicker_total": 0,
        "truth_flicker_last_reason": "",
        "truth_flicker_last_wall_s": 0.0,
        "truth_flicker_last_log_wall_s": 0.0,
        "control_feedback_miss_total": 0,
        "control_feedback_miss_consecutive": 0,
        "control_feedback_last_reason": "",
    }


def _record_jog_truth_flicker(reason: str) -> None:
    """Record a transient canonical-truth failure in the cartesian jog loop.

    RuntimeError raised by the jog feedback read is counted and surfaced
    rather than killing the jog thread. After the control-feedback split this
    generally means hard control feedback is unavailable; advisory canonical
    truth flicker should stay on the diagnostics path.

    Log emission is throttled to at most one entry per
    ``_JOG_TRUTH_FLICKER_LOG_THROTTLE_S`` seconds so sustained flicker
    bursts cannot flood stderr.
    """
    now = time.time()
    reason_text = str(reason)[:200]
    with _JOG_PERF_LOCK:
        _JOG_PERF["truth_flicker_total"] = int(_JOG_PERF.get("truth_flicker_total", 0)) + 1
        _JOG_PERF["truth_flicker_last_reason"] = reason_text
        _JOG_PERF["truth_flicker_last_wall_s"] = now
        last_log = float(_JOG_PERF.get("truth_flicker_last_log_wall_s", 0.0))
        should_log = now - last_log >= _JOG_TRUTH_FLICKER_LOG_THROTTLE_S
        if should_log:
            _JOG_PERF["truth_flicker_last_log_wall_s"] = now
    if should_log:
        print(
            f"[Jog] control feedback miss ({reason_text[:120]})",
            file=sys.stderr,
            flush=True,
        )


def _record_control_feedback_miss(reason: str) -> None:
    with _JOG_PERF_LOCK:
        _JOG_PERF["control_feedback_miss_total"] = int(_JOG_PERF.get("control_feedback_miss_total", 0)) + 1
        _JOG_PERF["control_feedback_miss_consecutive"] = int(
            _JOG_PERF.get("control_feedback_miss_consecutive", 0)
        ) + 1
        _JOG_PERF["control_feedback_last_reason"] = str(reason)[:200]


def _record_control_feedback_ok() -> None:
    with _JOG_PERF_LOCK:
        _JOG_PERF["control_feedback_miss_consecutive"] = 0


_JOG_PERF: dict[str, object] = _new_jog_perf_state()


def _update_perf_metric(metric: dict[str, object], value_ms: float) -> None:
    count = int(metric.get("count", 0)) + 1
    avg_ms = float(metric.get("avg_ms", 0.0))
    metric["count"] = count
    metric["last_ms"] = float(value_ms)
    metric["avg_ms"] = avg_ms + ((float(value_ms) - avg_ms) / float(count))
    metric["max_ms"] = max(float(metric.get("max_ms", 0.0)), float(value_ms))


def _vector_to_float_list(values: np.ndarray | list[float] | tuple[float, ...]) -> list[float]:
    arr = np.asarray(values, dtype=float).reshape(-1)
    return [float(item) for item in arr.tolist()]


def _joint_angles_deg_list(values: np.ndarray | list[float] | tuple[float, ...]) -> list[float]:
    return [float(np.rad2deg(item)) for item in np.asarray(values, dtype=float).reshape(-1).tolist()]


def _pose_snapshot_from_components(
    position: np.ndarray | list[float] | tuple[float, ...],
    orientation_matrix: np.ndarray,
) -> dict[str, object] | None:
    try:
        pos = np.asarray(position, dtype=float).reshape(3)
        orient = np.asarray(orientation_matrix, dtype=float).reshape(3, 3)
        euler_deg = R.from_matrix(orient).as_euler("xyz", degrees=True)
    except Exception:
        return None
    return {
        "position_m": {
            "x": float(pos[0]),
            "y": float(pos[1]),
            "z": float(pos[2]),
        },
        "orientation_euler_deg": {
            "roll": float(euler_deg[0]),
            "pitch": float(euler_deg[1]),
            "yaw": float(euler_deg[2]),
        },
    }


def _pose_snapshot_from_matrix(matrix: np.ndarray | None) -> dict[str, object] | None:
    if matrix is None:
        return None
    try:
        pose_mx = np.asarray(matrix, dtype=float).reshape(4, 4)
    except Exception:
        return None
    return _pose_snapshot_from_components(pose_mx[:3, 3], pose_mx[:3, :3])


def _pose_error_snapshot(
    target_position: np.ndarray,
    target_orientation_matrix: np.ndarray,
    actual_pose_matrix: np.ndarray | None,
) -> dict[str, object] | None:
    if actual_pose_matrix is None:
        return None
    try:
        actual_pose = np.asarray(actual_pose_matrix, dtype=float).reshape(4, 4)
        actual_position = actual_pose[:3, 3]
        actual_orientation = actual_pose[:3, :3]
        delta_position = actual_position - np.asarray(target_position, dtype=float).reshape(3)
        orientation_delta = (
            R.from_matrix(np.asarray(target_orientation_matrix, dtype=float).reshape(3, 3)).inv()
            * R.from_matrix(actual_orientation)
        )
        orientation_error_deg = float(np.rad2deg(orientation_delta.magnitude()))
    except Exception:
        return None
    return {
        "delta_position_m": {
            "x": float(delta_position[0]),
            "y": float(delta_position[1]),
            "z": float(delta_position[2]),
        },
        "position_error_mm": float(np.linalg.norm(delta_position) * 1000.0),
        "orientation_error_deg": orientation_error_deg,
    }


def _joint_error_snapshot(
    commanded_joints: np.ndarray | list[float] | tuple[float, ...],
    measured_joints: np.ndarray | list[float] | tuple[float, ...],
) -> dict[str, object] | None:
    try:
        commanded = np.asarray(commanded_joints, dtype=float).reshape(-1)
        measured = np.asarray(measured_joints, dtype=float).reshape(-1)
        delta_deg = np.rad2deg(measured - commanded)
    except Exception:
        return None
    return {
        "delta_deg": [float(value) for value in delta_deg.tolist()],
        "max_abs_joint_error_deg": float(np.max(np.abs(delta_deg))) if delta_deg.size else 0.0,
    }


def _following_error_snapshot(
    *,
    commanded_position: np.ndarray,
    commanded_orientation_matrix: np.ndarray,
    commanded_joints: np.ndarray,
    measured_pose_matrix: np.ndarray | None,
    measured_joints: np.ndarray,
) -> dict[str, object] | None:
    pose_error = _pose_error_snapshot(commanded_position, commanded_orientation_matrix, measured_pose_matrix)
    joint_error = _joint_error_snapshot(commanded_joints, measured_joints)
    if pose_error is None and joint_error is None:
        return None
    return {
        "pose": pose_error,
        "joint": joint_error,
    }


def _commanded_pose_snapshot_from_control(control: dict[str, object], *, key_prefix: str = "commanded") -> dict[str, object] | None:
    position_key = f"{key_prefix}_position_m"
    orientation_key = f"{key_prefix}_orientation_matrix"
    position = control.get(position_key)
    orientation = control.get(orientation_key)
    if position is None or orientation is None:
        return None
    try:
        return _pose_snapshot_from_components(
            np.asarray(position, dtype=float).reshape(3),
            np.asarray(orientation, dtype=float).reshape(3, 3),
        )
    except Exception:
        return None


def _commanded_joint_vector_from_control(control: dict[str, object]) -> np.ndarray | None:
    joints = control.get("commanded_joint_vector")
    if joints is None:
        return None
    try:
        return np.asarray(joints, dtype=float).reshape(-1)
    except Exception:
        return None


def _build_jog_ik_debug_payload(
    *,
    control: dict[str, object],
    dt: float,
    linear_vel: "np.ndarray",
    angular_deg_s: "np.ndarray",
    current_pose_snapshot: "dict[str, object] | None",
    current_commanded_pose_snapshot: "dict[str, object] | None",
    target_pose_snapshot: "dict[str, object] | None",
    solved_pose_matrix: "np.ndarray",
    applied_pose_matrix: "np.ndarray",
    gate_ok: bool,
    target_position: "np.ndarray",
    target_orientation: "np.ndarray",
    measured_joints_deg: "list[float] | None",
    commanded_joints: "np.ndarray",
    q_arr: "np.ndarray",
    applied_joint_vector: "np.ndarray",
    following_error: "dict[str, object] | None",
    perf_fields_after_gate: "dict[str, object]",
    gate_reason: "str | None",
    gate_details: "dict[str, object] | None",
    jacobian_diag: "dict[str, object] | None" = None,
    command_drift_norm: "float | None" = None,
    command_drift_per_joint: "list[float] | None" = None,
    ab_compare: "dict[str, object] | None" = None,
    recovery_action: "str | None" = None,
    command_horizon_s: "float | None" = None,
    using_jacobian: bool = False,
    jacobian_unavailable_reason: "str | None" = None,
    accepted_commanded_pose_snapshot: "dict[str, object] | None" = None,
) -> dict[str, object]:
    """2026-04-21 extracted helper: used to be an inline dict literal
    inside `_jog_controller_thread`. Pulled out so both the hot path
    (success, runs AFTER command_send) and the gate-failure path
    (runs inline) build the same ik_debug payload without duplicating
    ~40 lines of snapshot construction. Pure function — no locks, no
    IO — so it is safe to call from either path without ordering
    side effects."""
    return {
        "captured_at": datetime.datetime.now(datetime.timezone.utc).isoformat(),
        "seq": int(control.get("last_seq_received", -1)),
        "dt_s": float(dt),
        "linear_velocity_m_s": _vector_to_float_list(linear_vel),
        "angular_velocity_deg_s": _vector_to_float_list(angular_deg_s),
        "current_pose": current_pose_snapshot,
        "measured_pose": current_pose_snapshot,
        "commanded_pose": current_commanded_pose_snapshot,
        "target_pose": target_pose_snapshot,
        "solved_pose": _pose_snapshot_from_matrix(solved_pose_matrix),
        "applied_pose": _pose_snapshot_from_matrix(applied_pose_matrix),
        "accepted_commanded_pose": (
            accepted_commanded_pose_snapshot
            if accepted_commanded_pose_snapshot is not None
            else (target_pose_snapshot if gate_ok else current_commanded_pose_snapshot)
        ),
        "target_vs_solved": _pose_error_snapshot(
            target_position,
            target_orientation,
            solved_pose_matrix,
        ),
        "target_vs_applied": _pose_error_snapshot(
            target_position,
            target_orientation,
            applied_pose_matrix,
        ),
        "current_joints_deg": measured_joints_deg,
        "measured_joints_deg": measured_joints_deg,
        "commanded_joints_deg": _joint_angles_deg_list(commanded_joints),
        "ik_seed_joints_deg": _joint_angles_deg_list(commanded_joints),
        "ik_solution_joints_deg": _joint_angles_deg_list(q_arr),
        "applied_joints_deg": _joint_angles_deg_list(applied_joint_vector),
        "accepted_commanded_joints_deg": _joint_angles_deg_list(applied_joint_vector),
        "clamped_joint_indices": [],
        "clamped": False,
        "solve_failed": False,
        "following_error": following_error,
        "last_resync_reason": perf_fields_after_gate.get("last_resync_reason"),
        "last_resync_age_s": perf_fields_after_gate.get("last_resync_age_s"),
        "gate_result": "accepted" if gate_ok else "rejected",
        "gate_reason": gate_reason,
        "gate_details": gate_details,
        "jacobian_diagnostics": jacobian_diag,
        "command_drift_norm_rad": (
            float(command_drift_norm) if command_drift_norm is not None else None
        ),
        "command_drift_per_joint_rad": command_drift_per_joint,
        "q_dot_rad_s": (
            jacobian_diag.get("q_dot_rad_s")
            if isinstance(jacobian_diag, dict)
            else None
        ),
        "twist": (
            jacobian_diag.get("twist")
            if isinstance(jacobian_diag, dict)
            else None
        ),
        "achieved_twist": (
            jacobian_diag.get("achieved_twist")
            if isinstance(jacobian_diag, dict)
            else None
        ),
        "twist_residual": (
            jacobian_diag.get("twist_residual")
            if isinstance(jacobian_diag, dict)
            else None
        ),
        "twist_residual_norm": (
            jacobian_diag.get("twist_residual_norm")
            if isinstance(jacobian_diag, dict)
            else None
        ),
        "twist_attenuation_ratio": (
            jacobian_diag.get("twist_attenuation_ratio")
            if isinstance(jacobian_diag, dict)
            else None
        ),
        "linear_attenuation_ratio": (
            jacobian_diag.get("linear_attenuation_ratio")
            if isinstance(jacobian_diag, dict)
            else None
        ),
        "angular_attenuation_ratio": (
            jacobian_diag.get("angular_attenuation_ratio")
            if isinstance(jacobian_diag, dict)
            else None
        ),
        "ab_compare": ab_compare,
        "recovery_action": recovery_action,
        "command_horizon_s": float(command_horizon_s) if command_horizon_s is not None else None,
        "using_jacobian": bool(using_jacobian),
        "jacobian_unavailable_reason": jacobian_unavailable_reason,
    }


def _build_jog_command_state_perf_fields(control: dict[str, object]) -> dict[str, object]:
    last_resync_monotonic = control.get("last_resync_monotonic")
    if isinstance(last_resync_monotonic, (int, float)):
        last_resync_age_s: float | None = max(0.0, time.monotonic() - float(last_resync_monotonic))
    else:
        last_resync_age_s = None

    commanded_joints = _commanded_joint_vector_from_control(control)
    last_target_position = control.get("last_accepted_target_position_m")
    last_target_orientation = control.get("last_accepted_target_orientation_matrix")
    last_accepted_target_pose = None
    if last_target_position is not None and last_target_orientation is not None:
        try:
            last_accepted_target_pose = _pose_snapshot_from_components(
                np.asarray(last_target_position, dtype=float).reshape(3),
                np.asarray(last_target_orientation, dtype=float).reshape(3, 3),
            )
        except Exception:
            last_accepted_target_pose = None

    return {
        "command_state_valid": bool(control.get("command_state_valid", False)),
        "commanded_pose": _commanded_pose_snapshot_from_control(control),
        "commanded_joints_deg": _joint_angles_deg_list(commanded_joints) if commanded_joints is not None else None,
        "last_accepted_target_pose": last_accepted_target_pose,
        "following_error": control.get("following_error_snapshot"),
        "last_resync_reason": control.get("last_resync_reason"),
        "last_resync_age_s": last_resync_age_s,
        "last_gate_failure_reason": control.get("last_gate_failure_reason"),
        "last_gate_failure_details": control.get("last_gate_failure_details"),
    }


def _emit_jog_gate_alert(reason_code: str, details: dict[str, object] | None = None) -> None:
    normalized_reason = str(reason_code or "JOG_GATE_REJECTED").strip().upper() or "JOG_GATE_REJECTED"
    message = f"Jog step rejected: {normalized_reason}."
    telemetry_alerts.push_alert(
        level="warning",
        kind="JOG_GATE_REJECTED",
        message=message,
        details={
            "reason_code": normalized_reason,
            **(details or {}),
        },
        key=f"JOG_GATE_REJECTED:{normalized_reason}",
    )


def _unwrap_jog_joint_target(
    seed_joint_angles: np.ndarray,
    candidate_joint_angles: np.ndarray | list[float] | tuple[float, ...],
) -> np.ndarray:
    candidate = np.asarray(candidate_joint_angles, dtype=float).reshape(-1)
    try:
        unwrapped = trajectory_execution._unwrap_joint_trajectory(
            [seed_joint_angles.tolist(), candidate.tolist()]
        )
        if len(unwrapped) >= 2:
            return np.asarray(unwrapped[-1], dtype=float)
    except Exception:
        pass
    return candidate


def _validate_jog_step_candidate(
    *,
    seed_joint_angles: np.ndarray,
    candidate_joint_angles: np.ndarray,
    target_position: np.ndarray,
    target_orientation: np.ndarray,
) -> tuple[bool, str, dict[str, object], np.ndarray | None]:
    candidate = np.asarray(candidate_joint_angles, dtype=float).reshape(-1)
    seed = np.asarray(seed_joint_angles, dtype=float).reshape(-1)
    max_joint_step = float(np.max(np.abs(candidate - seed))) if candidate.size else 0.0

    limits = utils.LOGICAL_JOINT_LIMITS_RAD or [(-float("inf"), float("inf"))] * len(candidate)
    min_limit_margin = float("inf")
    violating_joint_indices: list[int] = []
    for idx, (joint_value, joint_limits) in enumerate(zip(candidate.tolist(), limits)):
        lo, hi = joint_limits
        margin = min(float(joint_value - lo), float(hi - joint_value))
        min_limit_margin = min(min_limit_margin, margin)
        if margin < trajectory_execution.JOINT_LIMIT_MARGIN_RAD:
            violating_joint_indices.append(int(idx))

    solved_pose_matrix = ik_solver.get_fk_matrix(candidate)
    pose_error = _pose_error_snapshot(target_position, target_orientation, solved_pose_matrix)
    cartesian_residual_m = None
    orientation_residual_deg = None
    if pose_error is not None:
        cartesian_residual_m = float(pose_error.get("position_error_mm", 0.0)) / 1000.0
        orientation_residual_deg = float(pose_error.get("orientation_error_deg", 0.0))

    details: dict[str, object] = {
        "max_joint_step_rad": trajectory_execution._json_safe_float(max_joint_step),
        "joint_limit_margin_rad": trajectory_execution._json_safe_float(min_limit_margin),
        "cartesian_residual_m": trajectory_execution._json_safe_float(
            cartesian_residual_m if cartesian_residual_m is not None else float("inf")
        ),
        "orientation_residual_deg": trajectory_execution._json_safe_float(
            orientation_residual_deg if orientation_residual_deg is not None else float("inf")
        ),
        "violating_joint_indices": [int(idx) for idx in violating_joint_indices],
        "candidate_joints_deg": _joint_angles_deg_list(candidate),
    }

    if max_joint_step > trajectory_execution.MAX_JOINT_STEP_RAD:
        return False, "IK_JUMP_REJECTED", details, solved_pose_matrix
    if violating_joint_indices:
        return False, "LIMIT_VIOLATION", details, solved_pose_matrix
    if solved_pose_matrix is None:
        return False, "FK_VALIDATION_FAILED", details, None
    if cartesian_residual_m is None or cartesian_residual_m > trajectory_execution.MAX_CART_RESIDUAL_M:
        return False, "CARTESIAN_RESIDUAL_EXCEEDED", details, solved_pose_matrix
    if orientation_residual_deg is None or orientation_residual_deg > trajectory_execution.MAX_ORIENT_RESIDUAL_DEG:
        return False, "ORIENTATION_RESIDUAL_EXCEEDED", details, solved_pose_matrix
    return True, "OK", details, solved_pose_matrix


def _compute_jog_joint_velocity_via_jacobian(
    *,
    q_seed: np.ndarray,
    twist: np.ndarray,
    sigma_threshold: float | None = None,
    lambda_max: float | None = None,
) -> tuple[np.ndarray, dict[str, object]]:
    """Compute joint velocity from Cartesian twist using damped least squares."""
    sigma_threshold = float(
        JOG_JACOBIAN_SIGMA_MIN_THRESHOLD if sigma_threshold is None else sigma_threshold
    )
    lambda_max = float(JOG_JACOBIAN_DAMPING_LAMBDA_MAX if lambda_max is None else lambda_max)
    J = ik_solver.compute_jacobian(q_seed)
    sigma = np.linalg.svd(J, compute_uv=False)
    sigma_min = float(sigma[-1]) if sigma.size else 0.0
    sigma_max = float(sigma[0]) if sigma.size else 0.0
    condition_number = sigma_max / max(sigma_min, 1e-12)

    if sigma_min < sigma_threshold and sigma_threshold > 0.0:
        ratio = max(0.0, sigma_min / sigma_threshold)
        lambda_squared = (lambda_max ** 2) * ((1.0 - ratio) ** 2)
    else:
        lambda_squared = 0.0

    twist_arr = np.asarray(twist, dtype=float).reshape(6)
    A = J @ J.T + lambda_squared * np.eye(6)
    x = np.linalg.solve(A, twist_arr)
    q_dot = J.T @ x
    achieved_twist = J @ q_dot
    twist_residual = twist_arr - achieved_twist
    requested_twist_norm = float(np.linalg.norm(twist_arr))
    achieved_twist_norm = float(np.linalg.norm(achieved_twist))
    requested_linear_norm = float(np.linalg.norm(twist_arr[:3]))
    achieved_linear_norm = float(np.linalg.norm(achieved_twist[:3]))
    requested_angular_norm = float(np.linalg.norm(twist_arr[3:]))
    achieved_angular_norm = float(np.linalg.norm(achieved_twist[3:]))

    diag = {
        "jacobian_sigma_min": sigma_min,
        "jacobian_sigma_max": sigma_max,
        "jacobian_condition_number": condition_number,
        "jacobian_damping_lambda_squared": float(lambda_squared),
        "jacobian_near_singular": bool(sigma_min < sigma_threshold),
        "twist": [float(value) for value in twist_arr.tolist()],
        "achieved_twist": [float(value) for value in achieved_twist.tolist()],
        "twist_residual": [float(value) for value in twist_residual.tolist()],
        "twist_residual_norm": float(np.linalg.norm(twist_residual)),
        "twist_attenuation_ratio": (
            achieved_twist_norm / requested_twist_norm
            if requested_twist_norm > 1e-12
            else 1.0
        ),
        "linear_attenuation_ratio": (
            achieved_linear_norm / requested_linear_norm
            if requested_linear_norm > 1e-12
            else 1.0
        ),
        "angular_attenuation_ratio": (
            achieved_angular_norm / requested_angular_norm
            if requested_angular_norm > 1e-12
            else 1.0
        ),
        "q_dot_rad_s": [float(value) for value in q_dot.tolist()],
    }
    return q_dot, diag


def _emit_joint_limit_alert(
    clamped_idx: list[int],
    q_requested: np.ndarray,
    q_applied: np.ndarray,
    mins: np.ndarray,
    maxs: np.ndarray,
    *,
    source: str,
) -> None:
    if not clamped_idx:
        return
    joint_labels: list[str] = []
    limit_sides: list[str] = []
    requested_deg: list[float] = []
    applied_deg: list[float] = []
    limit_min_deg: list[float] = []
    limit_max_deg: list[float] = []

    for idx in clamped_idx:
        requested = float(q_requested[idx])
        applied = float(q_applied[idx])
        lower = float(mins[idx])
        upper = float(maxs[idx])
        if requested > upper + 1e-6:
            side = "upper"
        elif requested < lower - 1e-6:
            side = "lower"
        else:
            side = "clamped"
        joint_num = int(idx) + 1
        joint_labels.append(f"J{joint_num} {side}")
        limit_sides.append(side)
        requested_deg.append(float(np.rad2deg(requested)))
        applied_deg.append(float(np.rad2deg(applied)))
        limit_min_deg.append(float(np.rad2deg(lower)))
        limit_max_deg.append(float(np.rad2deg(upper)))

    message = f"Joint limit reached during {source}: {', '.join(joint_labels)}."
    telemetry_alerts.push_alert(
        level="warning",
        kind="JOINT_LIMIT",
        message=message,
        details={
            "source": source,
            "logical_joints": [int(idx) + 1 for idx in clamped_idx],
            "joint_labels": joint_labels,
            "limit_sides": limit_sides,
            "requested_deg": requested_deg,
            "applied_deg": applied_deg,
            "limit_min_deg": limit_min_deg,
            "limit_max_deg": limit_max_deg,
        },
        key=f"JOINT_LIMIT:{source}:{'|'.join(joint_labels)}",
    )


def _jog_perf_update(**kwargs: object) -> None:
    with _JOG_PERF_LOCK:
        _JOG_PERF.update(kwargs)


def _record_jog_stage_metric(stage_name: str, value_ms: float) -> None:
    with _JOG_PERF_LOCK:
        stages = _JOG_PERF.setdefault("stages", {})
        if not isinstance(stages, dict):
            return
        metric = stages.setdefault(stage_name, {"count": 0, "avg_ms": 0.0, "max_ms": 0.0, "last_ms": 0.0})
        if isinstance(metric, dict):
            _update_perf_metric(metric, value_ms)


def _record_jog_loop_metric(loop_ms: float, *, target_period_ms: float) -> None:
    with _JOG_PERF_LOCK:
        loop = _JOG_PERF.setdefault("loop", {})
        if not isinstance(loop, dict):
            return
        _update_perf_metric(loop, loop_ms)
        overrun_ms = max(0.0, float(loop_ms) - float(target_period_ms))
        loop["last_overrun_ms"] = overrun_ms
        if overrun_ms > 0.0:
            loop["overrun_count"] = int(loop.get("overrun_count", 0)) + 1
            loop["max_overrun_ms"] = max(float(loop.get("max_overrun_ms", 0.0)), overrun_ms)


def _record_jog_velocity_update(velocity_vector: np.ndarray) -> None:
    now = time.monotonic()
    with _JOG_PERF_LOCK:
        updates = _JOG_PERF.setdefault("velocity_updates", {})
        if not isinstance(updates, dict):
            return
        last_ts = updates.get("last_update_monotonic_s")
        if isinstance(last_ts, (int, float)):
            gap_ms = max(0.0, (now - float(last_ts)) * 1000.0)
            gap_count = int(updates.get("gap_count", 0)) + 1
            avg_gap = float(updates.get("avg_gap_ms", 0.0))
            updates["gap_count"] = gap_count
            updates["last_gap_ms"] = gap_ms
            updates["avg_gap_ms"] = avg_gap + ((gap_ms - avg_gap) / float(gap_count))
            updates["max_gap_ms"] = max(float(updates.get("max_gap_ms", 0.0)), gap_ms)
        updates["last_update_monotonic_s"] = now
        updates["count"] = int(updates.get("count", 0)) + 1
        if bool(np.any(np.abs(velocity_vector) > 1e-9)):
            updates["nonzero_velocity_updates"] = int(updates.get("nonzero_velocity_updates", 0)) + 1
        else:
            updates["zero_velocity_updates"] = int(updates.get("zero_velocity_updates", 0)) + 1
        _JOG_PERF["last_velocity_command_age_s"] = 0.0


def get_jog_performance_snapshot() -> dict[str, object]:
    with _JOG_PERF_LOCK:
        snapshot = json.loads(json.dumps(_JOG_PERF))
    # Back-fill the truth-flicker metrics so downstream consumers always see a
    # stable shape even if _JOG_PERF was constructed before these fields
    # existed (the live perf state normally carries them via
    # _new_jog_perf_state(), so this is defensive).
    snapshot.setdefault("truth_flicker_total", 0)
    snapshot.setdefault("truth_flicker_last_reason", "")
    snapshot.setdefault("truth_flicker_last_wall_s", 0.0)
    snapshot.setdefault("control_feedback_miss_total", 0)
    snapshot.setdefault("control_feedback_miss_consecutive", 0)
    snapshot.setdefault("control_feedback_last_reason", "")
    session_snapshot = {}
    control_state = {}
    try:
        session_snapshot = _JOG_SESSION_MANAGER.get_snapshot()
    except Exception:
        session_snapshot = {}
    try:
        control_state = _JOG_SESSION_MANAGER.get_control_state()
    except Exception:
        control_state = {}
    last_update_age_s = session_snapshot.get("last_update_age_s")
    if isinstance(last_update_age_s, (int, float)):
        snapshot["last_velocity_command_age_s"] = max(0.0, float(last_update_age_s))
    else:
        last_cmd = utils.trajectory_state_get("last_jog_command_time", 0.0)
        if isinstance(last_cmd, (int, float)) and last_cmd > 0.0:
            snapshot["last_velocity_command_age_s"] = max(0.0, time.monotonic() - float(last_cmd))
    snapshot["execution_policy"] = str(
        session_snapshot.get("backend_mode")
        or utils.trajectory_state.get("jog_execution_policy", "")
        or ""
    )
    snapshot.update(_build_jog_command_state_perf_fields(control_state))
    snapshot["rtcore_owned"] = bool(_get_rtcore_jog_backend() is not None)
    snapshot["control_frequency_hz"] = int(JOG_CONTROL_FREQUENCY_HZ) if "JOG_CONTROL_FREQUENCY_HZ" in globals() else 0
    rtcore_jog_debug = _build_rtcore_jog_debug_snapshot()
    if rtcore_jog_debug is not None:
        snapshot["rtcore_jog_debug"] = rtcore_jog_debug
    return snapshot


def _safe_int(value: object, default: int = 0) -> int:
    try:
        return int(value if value is not None else default)
    except Exception:
        return default


def _safe_optional_int(value: object) -> int | None:
    if value is None or value == "":
        return None
    try:
        return int(value)
    except Exception:
        return None


def _reset_program_status(**overrides: object) -> None:
    utils.program_status_reset(**overrides)


def _update_program_status(**kwargs: object) -> None:
    utils.program_status_update(**kwargs)


def _clear_motion_stop_latch() -> None:
    utils.trajectory_state_update(
        motion_stop_latched=False,
        motion_stop_latched_at=None,
        motion_stop_latched_reason=None,
        stop_request_reason=None,
        should_stop=False,
    )


def _require_motion_not_stop_latched(command_name: str) -> None:
    if not bool(utils.trajectory_state_get("motion_stop_latched", False)):
        return
    reason = str(utils.trajectory_state_get("motion_stop_latched_reason", None) or "operator_abort")
    raise RuntimeError(
        "MOTION_STOP_LATCHED: "
        f"STOP is latched from {reason}; request SAFE_POWER_UP before starting {command_name}."
    )


def _begin_non_program_motion() -> None:
    _require_motion_not_stop_latched("non-program motion")
    _reset_program_status()
    utils.trajectory_state_set("stop_request_reason", None)


def _build_bounded_joint_path(
    current_q: Sequence[float],
    target_q: Sequence[float],
    *,
    max_motor_rpm: float = _PROGRAM_JOINT_MOVE_MAX_MOTOR_RPM,
    max_joint_deg_s: float | None = None,
) -> tuple[list[list[float]], float]:
    current_q_arr = np.asarray(list(current_q), dtype=float)
    target_q_arr = np.asarray(list(target_q), dtype=float)
    if current_q_arr.shape != target_q_arr.shape:
        raise ValueError("Current and target joint vectors must have matching shapes.")

    try:
        active_robot = robot_config.get_active_robot()
        gear_ratios = list(active_robot.actuator_gear_ratios)
    except Exception:
        gear_ratios = []

    delta_q = target_q_arr - current_q_arr
    duration_s = _SAFE_JOINT_MOVE_MIN_DURATION_S
    if max_joint_deg_s is not None and np.isfinite(max_joint_deg_s) and float(max_joint_deg_s) > 0.0:
        max_joint_rad_s = float(np.deg2rad(max_joint_deg_s))
        for joint_idx in range(len(target_q_arr)):
            duration_s = max(duration_s, abs(float(delta_q[joint_idx])) / max_joint_rad_s)
    else:
        for joint_idx in range(len(target_q_arr)):
            ratio = float(gear_ratios[joint_idx]) if joint_idx < len(gear_ratios) else 1.0
            if not np.isfinite(ratio) or ratio <= 0.0:
                ratio = 1.0
            max_rate = (float(max_motor_rpm) / ratio) * (2.0 * np.pi / 60.0)
            if max_rate > 0.0:
                duration_s = max(duration_s, abs(float(delta_q[joint_idx])) / max_rate)

    num_steps = max(2, int(np.ceil(duration_s * _SAFE_JOINT_MOVE_FREQUENCY_HZ)))
    t = np.linspace(0.0, 1.0, num_steps)
    smooth = (3.0 * np.square(t)) - (2.0 * np.power(t, 3))
    joint_path = [
        (current_q_arr + (smooth_i * delta_q)).tolist()
        for smooth_i in smooth
    ]
    return joint_path, float(duration_s)


def _plan_joint_move_to_pose(
    current_q: Sequence[float],
    *,
    target_q: Sequence[float] | None = None,
    target_pos: Sequence[float] | None = None,
    target_orientation: np.ndarray | None = None,
    max_motor_rpm: float = _PROGRAM_JOINT_MOVE_MAX_MOTOR_RPM,
    max_joint_deg_s: float | None = None,
) -> tuple[list[list[float]] | None, list[float] | None]:
    current_q_list = list(map(float, current_q))
    solved_q: list[float] | None
    if target_q is not None:
        solved_q = list(map(float, target_q))
    else:
        if target_pos is None:
            raise ValueError("target_pos is required when target_q is not provided.")
        orientation_matrix = target_orientation
        if orientation_matrix is None:
            current_pose_matrix = ik_solver.get_fk_matrix(current_q_list)
            if current_pose_matrix is None:
                return None, None
            orientation_matrix = np.array(current_pose_matrix[:3, :3], dtype=float)
        solved = ik_solver.solve_ik(
            target_position=np.asarray(target_pos, dtype=float),
            target_orientation_matrix=orientation_matrix,
            initial_joint_angles=current_q_list,
        )
        if solved is None:
            return None, None
        solved_q = list(map(float, np.asarray(solved, dtype=float).tolist()))

    joint_path, _duration_s = _build_bounded_joint_path(
        current_q_list,
        solved_q,
        max_motor_rpm=max_motor_rpm,
        max_joint_deg_s=max_joint_deg_s,
    )
    return joint_path, solved_q


def _coerce_optional_positive_speed(value: object) -> float | None:
    if value is None or value == "":
        return None
    try:
        numeric = float(value)
    except Exception:
        return None
    if not np.isfinite(numeric) or numeric <= 0.0:
        return None
    return float(numeric)


def _extract_waypoint_linear_speed_m_s(raw_waypoint: dict | None) -> float | None:
    if not isinstance(raw_waypoint, dict):
        return None
    raw_value = raw_waypoint.get(
        "linear_speed_mm_s",
        raw_waypoint.get(
            "linearSpeedMmS",
            raw_waypoint.get("linear_speed_mm_per_s", raw_waypoint.get("linearSpeedMmPerSec")),
        ),
    )
    speed_mm_s = _coerce_optional_positive_speed(raw_value)
    return None if speed_mm_s is None else float(speed_mm_s / 1000.0)


def _extract_waypoint_linear_acceleration_m_s2(raw_waypoint: dict | None) -> float | None:
    if not isinstance(raw_waypoint, dict):
        return None
    acceleration_mm_s2 = _coerce_optional_positive_speed(
        raw_waypoint.get(
            "linear_acceleration_mm_s2",
            raw_waypoint.get(
                "linearAccelerationMmS2",
                raw_waypoint.get(
                    "linear_acceleration_mm_per_s2",
                    raw_waypoint.get("linearAccelerationMmPerSec2"),
                ),
            ),
        )
    )
    return None if acceleration_mm_s2 is None else float(acceleration_mm_s2 / 1000.0)


def _extract_waypoint_rotation_speed_deg_s(raw_waypoint: dict | None) -> float | None:
    if not isinstance(raw_waypoint, dict):
        return None
    return _coerce_optional_positive_speed(
        raw_waypoint.get(
            "rotation_speed_deg_s",
            raw_waypoint.get(
                "rotationSpeedDegS",
                raw_waypoint.get("rotation_speed_deg_per_s", raw_waypoint.get("rotationSpeedDegPerSec")),
            ),
        )
    )


def _extract_waypoint_pause_after_s(raw_waypoint: dict | None) -> float | None:
    if not isinstance(raw_waypoint, dict):
        return None
    return _coerce_optional_positive_speed(
        raw_waypoint.get(
            "pause_after_s",
            raw_waypoint.get(
                "pauseAfterS",
                raw_waypoint.get(
                    "pause_after_sec",
                    raw_waypoint.get(
                        "pauseAfterSec",
                        raw_waypoint.get(
                            "pause_after_seconds",
                            raw_waypoint.get(
                                "pauseAfterSeconds",
                                raw_waypoint.get(
                                    "pause_duration_s",
                                    raw_waypoint.get("pauseDurationS"),
                                ),
                            ),
                        ),
                    ),
                ),
            ),
        )
    )


def _extract_move_linear_speed_m_s(move_cmd: dict | None) -> float | None:
    if not isinstance(move_cmd, dict):
        return None
    speed_mm_s = _coerce_optional_positive_speed(
        move_cmd.get(
            "linear_speed_mm_s",
            move_cmd.get(
                "linearSpeedMmS",
                move_cmd.get("linear_speed_mm_per_s", move_cmd.get("linearSpeedMmPerSec")),
            ),
        )
    )
    return None if speed_mm_s is None else float(speed_mm_s / 1000.0)


def _extract_move_linear_acceleration_m_s2(move_cmd: dict | None) -> float | None:
    if not isinstance(move_cmd, dict):
        return None
    acceleration_mm_s2 = _coerce_optional_positive_speed(
        move_cmd.get(
            "linear_acceleration_mm_s2",
            move_cmd.get(
                "linearAccelerationMmS2",
                move_cmd.get(
                    "linear_acceleration_mm_per_s2",
                    move_cmd.get("linearAccelerationMmPerSec2"),
                ),
            ),
        )
    )
    return None if acceleration_mm_s2 is None else float(acceleration_mm_s2 / 1000.0)


def _extract_move_rotation_speed_deg_s(move_cmd: dict | None) -> float | None:
    if not isinstance(move_cmd, dict):
        return None
    return _coerce_optional_positive_speed(
        move_cmd.get(
            "rotation_speed_deg_s",
            move_cmd.get(
                "rotationSpeedDegS",
                move_cmd.get("rotation_speed_deg_per_s", move_cmd.get("rotationSpeedDegPerSec")),
            ),
        )
    )


def _collapse_runtime_move_pause_steps(
    planned_steps: list[dict],
    *,
    position_tolerance_rad: float = 1e-9,
) -> list[dict] | None:
    if not isinstance(planned_steps, list) or len(planned_steps) == 0:
        return None

    shared_freq: int | None = None
    combined_path: list[list[float]] = []
    logical_step_count = 0

    for step in planned_steps:
        if not isinstance(step, dict):
            return None
        step_type = str(step.get("type", "")).strip().lower()
        if step_type == "move":
            path = step.get("path")
            if not isinstance(path, list) or len(path) == 0:
                return None
            try:
                step_freq = int(step.get("freq"))
            except Exception:
                return None
            if step_freq <= 0:
                return None
            if shared_freq is None:
                shared_freq = step_freq
            elif step_freq != shared_freq:
                return None

            normalized_path = [
                [float(value) for value in sample]
                for sample in path
                if isinstance(sample, (list, tuple))
            ]
            if len(normalized_path) == 0:
                return None

            if not combined_path:
                combined_path.extend(normalized_path)
            else:
                first_sample = normalized_path[0]
                last_sample = combined_path[-1]
                same_start = (
                    len(first_sample) == len(last_sample)
                    and all(
                        abs(float(lhs) - float(rhs)) <= position_tolerance_rad
                        for lhs, rhs in zip(first_sample, last_sample)
                    )
                )
                combined_path.extend(normalized_path[1:] if same_start else normalized_path)
        elif step_type == "pause":
            if shared_freq is None or len(combined_path) == 0:
                return None
            duration_s = _coerce_optional_positive_speed(step.get("duration"))
            if duration_s is not None:
                hold_samples = max(1, int(round(float(duration_s) * float(shared_freq))))
                hold_pose = list(combined_path[-1])
                combined_path.extend([list(hold_pose) for _ in range(hold_samples)])
        else:
            return None
        logical_step_count += 1

    if shared_freq is None or len(combined_path) == 0:
        return None

    return [
        {
            "type": "move",
            "path": combined_path,
            "freq": shared_freq,
            "logical_step_count": logical_step_count,
        }
    ]


def _plan_orientation_only_move(
    current_q: Sequence[float],
    *,
    target_pos: Sequence[float],
    target_orientation: np.ndarray,
    angular_speed_deg_s: float,
    frequency_hz: int = 100,
) -> list[list[float]] | None:
    current_q_list = list(map(float, current_q))
    current_pose_matrix = ik_solver.get_fk_matrix(current_q_list)
    if current_pose_matrix is None:
        return None
    start_orientation = np.asarray(current_pose_matrix[:3, :3], dtype=float)
    target_orientation_matrix = np.asarray(target_orientation, dtype=float).reshape(3, 3)
    rotation_delta = R.from_matrix(start_orientation).inv() * R.from_matrix(target_orientation_matrix)
    angle_deg = abs(float(np.rad2deg(rotation_delta.magnitude())))
    if angle_deg <= 1e-6:
        return [current_q_list, current_q_list]
    speed_deg_s = max(0.1, float(angular_speed_deg_s))
    duration_s = max(_SAFE_JOINT_MOVE_MIN_DURATION_S, angle_deg / speed_deg_s)
    num_steps = max(2, int(np.ceil(duration_s * max(1, int(frequency_hz)))))
    t_values = np.linspace(0.0, 1.0, num_steps)
    slerp = Slerp(
        [0.0, 1.0],
        R.concatenate([R.from_matrix(start_orientation), R.from_matrix(target_orientation_matrix)]),
    )
    orientations = [rotation.as_matrix() for rotation in slerp(t_values)]
    positions = [np.asarray(target_pos, dtype=float).reshape(3) for _ in t_values]
    joint_path = ik_solver.solve_ik_path_batch(
        path_points=positions,
        initial_joint_angles=current_q_list,
        target_orientations=orientations,
    )
    if joint_path is None:
        return None
    return [list(map(float, np.asarray(sample, dtype=float).tolist())) for sample in joint_path]


def _estimate_joint_path_rotation_speed_deg_s(
    start_q: Sequence[float],
    joint_path: list[list[float]] | None,
    *,
    frequency_hz: int,
) -> float | None:
    if not joint_path:
        return None
    start_q_arr = np.asarray(list(start_q), dtype=float)
    end_q_arr = np.asarray(joint_path[-1], dtype=float)
    if start_q_arr.shape != end_q_arr.shape:
        return None
    duration_s = max(1.0 / max(1, int(frequency_hz)), (max(1, len(joint_path)) - 1) / max(1, int(frequency_hz)))
    if duration_s <= 0.0:
        return None
    delta_deg = np.abs(np.rad2deg(end_q_arr - start_q_arr))
    return float(np.max(delta_deg) / duration_s)


def _resolve_default_orientation_speed_deg_s(angle_deg: float) -> float:
    magnitude = abs(float(angle_deg))
    if magnitude <= 1e-6:
        return 90.0
    duration_s = max(0.12, min(0.75, magnitude / 90.0))
    return float(magnitude / duration_s)


def _program_status_from_snapshot(snapshot: dict[str, object]) -> dict[str, object]:
    raw = snapshot.get("program_status")
    merged: dict[str, object] = {
        "name": None,
        "active": False,
        "state": "idle",
        "terminal_reason": None,
        "failing_step_index": None,
        "completed_step_count": 0,
        "completed_loop_count": 0,
        "loop_enabled": False,
        "use_cache": False,
        "step_count": 0,
        "move_steps": 0,
        "pause_steps": 0,
        "joint_move_steps": 0,
        "rtcore_segments": False,
        "segment_execution_policy": "",
        "current_step_index": None,
        "current_step_type": None,
        "loop_iteration": 0,
    }
    if isinstance(raw, dict):
        merged.update(raw)

    active_program_name = snapshot.get("active_program_name")
    if isinstance(active_program_name, str) and active_program_name.strip():
        merged["name"] = active_program_name.strip()
        merged["active"] = True
        merged["loop_enabled"] = bool(snapshot.get("active_program_loop_enabled", merged["loop_enabled"]))
        merged["use_cache"] = bool(snapshot.get("active_program_use_cache", merged["use_cache"]))
        merged["step_count"] = _safe_int(snapshot.get("active_program_step_count"), _safe_int(merged.get("step_count")))
        merged["move_steps"] = _safe_int(snapshot.get("active_program_move_steps"), _safe_int(merged.get("move_steps")))
        merged["pause_steps"] = _safe_int(snapshot.get("active_program_pause_steps"), _safe_int(merged.get("pause_steps")))
        merged["joint_move_steps"] = _safe_int(
            snapshot.get("active_program_joint_move_steps"),
            _safe_int(merged.get("joint_move_steps")),
        )
        merged["rtcore_segments"] = bool(snapshot.get("active_program_rtcore_segments", merged["rtcore_segments"]))
        merged["segment_execution_policy"] = str(
            snapshot.get("active_program_segment_execution_policy", merged["segment_execution_policy"]) or ""
        )
        merged["current_step_index"] = snapshot.get("active_program_step_index")
        merged["current_step_type"] = snapshot.get("active_program_step_type")
        merged["loop_iteration"] = _safe_int(
            snapshot.get("active_program_loop_iteration"),
            _safe_int(merged.get("loop_iteration")),
        )

    name = str(merged.get("name") or "").strip()
    active = bool(merged.get("active", False))
    state = str(merged.get("state") or ("executing" if active else "idle")).strip().lower() or "idle"
    if active and state not in _PROGRAM_ACTIVE_STATES:
        state = "executing"

    if not name and not active and state == "idle":
        return {}

    terminal_reason_raw = merged.get("terminal_reason")
    terminal_reason = None
    if terminal_reason_raw is not None and str(terminal_reason_raw).strip():
        terminal_reason = str(terminal_reason_raw).strip().lower()

    return {
        "name": name or None,
        "active": active,
        "state": state,
        "terminal_reason": terminal_reason,
        "failing_step_index": _safe_optional_int(merged.get("failing_step_index")),
        "completed_step_count": _safe_int(merged.get("completed_step_count")),
        "completed_loop_count": _safe_int(merged.get("completed_loop_count")),
        "loop_enabled": bool(merged.get("loop_enabled", False)),
        "use_cache": bool(merged.get("use_cache", False)),
        "step_count": _safe_int(merged.get("step_count")),
        "move_steps": _safe_int(merged.get("move_steps")),
        "pause_steps": _safe_int(merged.get("pause_steps")),
        "joint_move_steps": _safe_int(merged.get("joint_move_steps")),
        "rtcore_segments": bool(merged.get("rtcore_segments", False)),
        "segment_execution_policy": str(merged.get("segment_execution_policy") or ""),
        "current_step_index": _safe_optional_int(merged.get("current_step_index")),
        "current_step_type": (
            str(merged.get("current_step_type")).strip()
            if merged.get("current_step_type") not in (None, "")
            else None
        ),
        "loop_iteration": _safe_int(merged.get("loop_iteration")),
    }


def _flatten_program_status(program: dict[str, object]) -> dict[str, object]:
    if not program:
        return {}
    return {
        "program_name": program.get("name"),
        "program_active": bool(program.get("active", False)),
        "program_state": str(program.get("state") or "idle"),
        "program_terminal_reason": program.get("terminal_reason"),
        "program_failing_step_index": program.get("failing_step_index"),
        "program_completed_step_count": _safe_int(program.get("completed_step_count")),
        "program_completed_loop_count": _safe_int(program.get("completed_loop_count")),
        "program_loop_enabled": bool(program.get("loop_enabled", False)),
        "program_use_cache": bool(program.get("use_cache", False)),
        "program_step_count": _safe_int(program.get("step_count")),
        "program_move_steps": _safe_int(program.get("move_steps")),
        "program_pause_steps": _safe_int(program.get("pause_steps")),
        "program_joint_move_steps": _safe_int(program.get("joint_move_steps")),
        "program_rtcore_segments": bool(program.get("rtcore_segments", False)),
        "program_segment_execution_policy": str(program.get("segment_execution_policy") or ""),
        "program_current_step_index": program.get("current_step_index"),
        "program_current_step_type": program.get("current_step_type"),
        "program_loop_iteration": _safe_int(program.get("loop_iteration")),
    }


def _attach_program_status(payload: dict[str, object], snapshot: dict[str, object]) -> None:
    program = _program_status_from_snapshot(snapshot)
    if not program:
        return

    payload["program"] = program
    payload.update(_flatten_program_status(program))
    payload["accepted"] = bool(payload.get("accepted", False) or program.get("state") != "idle")

    program_active = bool(program.get("active", False))
    program_state = str(program.get("state") or "idle").strip().lower() or "idle"
    execution = payload.get("execution")
    execution_payload = dict(execution) if isinstance(execution, dict) else {}
    rtcore_segment_active = bool(
        int(execution_payload.get("active_traj_id", 0) or 0) > 0
        or int(execution_payload.get("queue_depth", 0) or 0) > 0
    )
    payload["execution"] = execution_payload

    if program_active and not rtcore_segment_active:
        payload["completion_scope"] = "controller_program_thread"
        payload["source_of_truth"] = "controller_program_thread"
        if str(payload.get("state", "")).strip().lower() in {"", "idle", "accepted"}:
            payload["state"] = program_state
        return

    if (not program_active) and program_state in _PROGRAM_TERMINAL_STATES and not _motion_payload_is_active(payload):
        payload["completion_scope"] = "controller_program_thread"
        payload["source_of_truth"] = "controller_program_thread"
        payload["state"] = program_state


def _get_active_backend():
    try:
        return backend_registry.get_active_backend()
    except Exception:
        return None


def _clear_bounded_endpoint(reason: str) -> None:
    utils.trajectory_state_set(
        "last_bounded_endpoint",
        {
            "cleared": True,
            "reason": str(reason),
            "cleared_at_monotonic": time.monotonic(),
        },
    )


def _target_axis_mask_for_joint_indices(backend, target_joint_indices: Sequence[int] | None) -> int:
    if target_joint_indices is None:
        all_axis_mask_getter = getattr(backend, "_all_axis_mask", None)
        if callable(all_axis_mask_getter):
            try:
                return int(all_axis_mask_getter())
            except Exception:
                return 0
        return 0
    axis_mask_getter = getattr(backend, "logical_joint_indices_to_axis_mask", None)
    if not callable(axis_mask_getter):
        return 0
    return int(axis_mask_getter([int(value) for value in target_joint_indices]))


def _require_target_axes_motion_ready(target_joint_indices: Sequence[int] | None) -> None:
    backend = _get_active_backend()
    if backend is None:
        return
    snapshot_getter = getattr(backend, "describe_motion_snapshot", None)
    if not callable(snapshot_getter):
        return

    snapshot = snapshot_getter()
    states = list(snapshot.get("per_axis_ds402_state", []) or [])
    axis_mask = _target_axis_mask_for_joint_indices(backend, target_joint_indices)
    if axis_mask == 0 and target_joint_indices is None and states:
        axis_mask = (1 << len(states)) - 1
    if axis_mask == 0:
        return

    faulted_axes = set(int(value) for value in snapshot.get("faulted_axis_indices", []) or [])
    bad_axes: list[int] = []
    faulted_target_axes: list[int] = []
    for axis_i, state in enumerate(states):
        if (axis_mask & (1 << axis_i)) == 0:
            continue
        if axis_i in faulted_axes:
            faulted_target_axes.append(axis_i)
        if int(state) != _DS402_OPERATION_ENABLED_STATE:
            bad_axes.append(axis_i)

    target_joint_label = (
        [int(value) + 1 for value in target_joint_indices]
        if target_joint_indices is not None
        else "all"
    )
    if faulted_target_axes:
        raise RuntimeError(
            "DRIVE_FAULTED:"
            f" target_joints={target_joint_label}"
            f" target_axes={faulted_target_axes}"
            " are faulted; clear faults before jogging"
        )

    if bad_axes:
        raise RuntimeError(
            "DRIVE_NOT_OP_ENABLED:"
            f" target_joints={target_joint_label}"
            f" target_axes={bad_axes}"
            " are not OperationEnabled; run /control/safe-power-up before jogging"
        )


def _get_rtcore_execution_backend_and_status():
    backend = _get_active_backend()
    if backend is None:
        return None, None
    getter = getattr(backend, "get_execution_status", None)
    if not callable(getter):
        return backend, None
    try:
        return backend, getter()
    except Exception:
        return backend, None


def _get_rtcore_jog_debug_status():
    backend = _get_rtcore_jog_backend()
    if backend is None:
        return None
    getter = getattr(backend, "get_jog_debug_status", None)
    if not callable(getter):
        return None
    try:
        return getter()
    except Exception:
        return None


def _build_rtcore_jog_debug_snapshot() -> dict[str, object] | None:
    status = _get_rtcore_jog_debug_status()
    if status is None:
        return None
    num_axes = max(0, min(int(getattr(status, "num_axes", 0) or 0), 16))
    feedback = [int(value) for value in list(getattr(status, "feedback_pos_counts", []))[:num_axes]]
    hold = [int(value) for value in list(getattr(status, "hold_target_counts", []))[:num_axes]]
    output = [int(value) for value in list(getattr(status, "output_target_counts", []))[:num_axes]]
    output_velocity = [
        int(value)
        for value in list(getattr(status, "output_target_velocity_counts_per_s", []))[:num_axes]
    ]
    sample_time_ns = int(getattr(status, "sample_time_ns", 0) or 0)
    last_stop_time_ns = int(getattr(status, "last_stop_time_ns", 0) or 0)
    now_ns = time.monotonic_ns()
    snapshot: dict[str, object] = {
        "num_axes": num_axes,
        "active_jog": bool(getattr(status, "active_jog", False)),
        "active_jog_axis_mask": int(getattr(status, "active_jog_axis_mask", 0) or 0),
        "command_sp_mask": int(getattr(status, "command_sp_mask", 0) or 0),
        "have_hold_mask": int(getattr(status, "have_hold_mask", 0) or 0),
        "have_jog_target_mask": int(getattr(status, "have_jog_target_mask", 0) or 0),
        "snap_hold_mask": int(getattr(status, "snap_hold_mask", 0) or 0),
        "stop_arrest_mask": int(getattr(status, "stop_arrest_mask", 0) or 0),
        "latest_cmd_axis_mask": int(getattr(status, "latest_cmd_axis_mask", 0) or 0),
        "latest_cmd_flags": int(getattr(status, "latest_cmd_flags", 0) or 0),
        "latest_cmd_timeout_ns": int(getattr(status, "latest_cmd_timeout_ns", 0) or 0),
        "sample_time_ns": sample_time_ns,
        "sample_age_s": max(0.0, (now_ns - sample_time_ns) / 1e9) if sample_time_ns > 0 else None,
        "active_jog_cmd_seq": int(getattr(status, "active_jog_cmd_seq", 0) or 0),
        "latest_jog_seq_seen": int(getattr(status, "latest_jog_seq_seen", 0) or 0),
        "active_jog_deadline_ns": int(getattr(status, "active_jog_deadline_ns", 0) or 0),
        "last_stop_reason": int(getattr(status, "last_stop_reason", 0) or 0),
        "last_stop_reason_name": str(getattr(status, "last_stop_reason_name", "none") or "none"),
        "last_stop_axis_mask": int(getattr(status, "last_stop_axis_mask", 0) or 0),
        "last_stop_time_ns": last_stop_time_ns,
        "last_stop_age_s": max(0.0, (now_ns - last_stop_time_ns) / 1e9) if last_stop_time_ns > 0 else None,
        "last_stop_cmd_seq": int(getattr(status, "last_stop_cmd_seq", 0) or 0),
        "feedback_pos_counts": feedback,
        "hold_target_counts": hold,
        "output_target_counts": output,
        "output_target_velocity_counts_per_s": output_velocity,
        "hold_minus_feedback_counts": [h - f for h, f in zip(hold, feedback)],
        "output_minus_feedback_counts": [o - f for o, f in zip(output, feedback)],
    }
    return snapshot


def _get_backend_last_submitted_trajectory_id(backend) -> int:
    if backend is None:
        return 0
    getter = getattr(backend, "get_last_submitted_trajectory_id", None)
    if not callable(getter):
        return 0
    try:
        return int(getter() or 0)
    except Exception:
        return 0


def _rtcore_status_observes_new_submission(
    execution_status,
    *,
    baseline_active_traj_id: int,
    submitted_traj_id: int,
) -> bool:
    active_traj_id = int(getattr(execution_status, "active_traj_id", 0) or 0)
    queue_depth = int(getattr(execution_status, "queue_depth", 0) or 0)
    observed_state = str(getattr(execution_status, "state_name", "idle")).strip().lower() or "idle"
    motion_done = bool(getattr(execution_status, "motion_done", False))

    if active_traj_id > baseline_active_traj_id:
        return True
    if active_traj_id > 0 and queue_depth > 0:
        return True
    if submitted_traj_id > baseline_active_traj_id and (
        active_traj_id == submitted_traj_id
        or observed_state not in _RTCORE_TERMINAL_EXECUTION_STATES
        or not motion_done
    ):
        return True
    return False


def _refresh_rtcore_execution_status_for_new_submission(
    backend,
    execution_status,
    *,
    accepted: bool,
    controller_motion_state: str,
    controller_thread_running: bool,
):
    submitted_traj_id = _get_backend_last_submitted_trajectory_id(backend)
    if backend is None or execution_status is None:
        return execution_status, submitted_traj_id
    if not accepted or not (controller_thread_running or controller_motion_state == "executing"):
        return execution_status, submitted_traj_id

    baseline_active_traj_id = int(getattr(execution_status, "active_traj_id", 0) or 0)
    if _rtcore_status_observes_new_submission(
        execution_status,
        baseline_active_traj_id=baseline_active_traj_id,
        submitted_traj_id=submitted_traj_id,
    ):
        return execution_status, submitted_traj_id

    status_getter = getattr(backend, "get_execution_status", None)
    if not callable(status_getter):
        return execution_status, submitted_traj_id

    latest_status = execution_status
    latest_submitted_traj_id = submitted_traj_id
    deadline = time.monotonic() + _RTCORE_ACK_REFRESH_TIMEOUT_S
    while time.monotonic() <= deadline:
        time.sleep(_RTCORE_ACK_REFRESH_POLL_INTERVAL_S)
        latest_submitted_traj_id = _get_backend_last_submitted_trajectory_id(backend)
        try:
            latest_status = status_getter()
        except Exception:
            break
        if _rtcore_status_observes_new_submission(
            latest_status,
            baseline_active_traj_id=baseline_active_traj_id,
            submitted_traj_id=latest_submitted_traj_id,
        ):
            break

    return latest_status, latest_submitted_traj_id


def _backend_supports_rtcore_execution() -> bool:
    backend, status = _get_rtcore_execution_backend_and_status()
    return backend is not None and status is not None


def _get_rtcore_jog_backend():
    backend = _get_active_backend()
    if backend is None:
        return None
    supports_joint_velocity_lease_jog = getattr(backend, "supports_joint_velocity_lease_jog", None)
    if callable(supports_joint_velocity_lease_jog):
        try:
            if bool(supports_joint_velocity_lease_jog()):
                return backend
        except Exception:
            pass
    supports = getattr(backend, "supports_realtime_jog", None)
    if not callable(supports):
        return None
    try:
        return backend if bool(supports()) else None
    except Exception:
        return None


def _default_completion_scope(*, closed_loop: bool = False, prefer_rtcore: bool = False) -> str:
    if prefer_rtcore and _backend_supports_rtcore_execution():
        return "rtcore_execution"
    if closed_loop:
        return "controller_closed_loop"
    return "controller_trajectory_thread"


def _resolve_scheduled_motion_execution_policy(
    *,
    closed_loop_requested: bool,
    command_name: str,
) -> tuple[bool, dict[str, object]]:
    requested = bool(closed_loop_requested)
    rtcore_backed = _backend_supports_rtcore_execution()
    effective_closed_loop = requested
    execution_policy = "controller_closed_loop" if requested else "controller_open_loop"

    if rtcore_backed:
        if requested:
            print(
                f"[RTCore Policy] {command_name}: forcing scheduled motion onto the RTCore queued path "
                "instead of the Python-timed closed-loop executor."
            )
        effective_closed_loop = False
        execution_policy = "rtcore_queued"

    return effective_closed_loop, {
        "closed_loop_requested": requested,
        "closed_loop_effective": effective_closed_loop,
        "execution_policy": execution_policy,
        "rtcore_execution_preferred": rtcore_backed,
    }


def _motion_payload_is_active(payload: dict[str, object]) -> bool:
    program = payload.get("program")
    if isinstance(program, dict):
        if bool(program.get("active", False)):
            return True
        program_state = str(program.get("state", "")).strip().lower()
        if program_state in _PROGRAM_ACTIVE_STATES:
            return True

    execution = payload.get("execution")
    if not isinstance(execution, dict):
        return False
    if bool(execution.get("controller_thread_running", False)):
        return True
    if not bool(execution.get("rtcore_status_present", False)):
        return False

    state_name = str(execution.get("state_name", payload.get("state", "idle"))).strip().lower() or "idle"
    active_mode_name = str(execution.get("active_mode_name", "idle")).strip().lower() or "idle"
    active_traj_id = int(execution.get("active_traj_id", 0) or 0)
    queue_depth = int(execution.get("queue_depth", 0) or 0)
    motion_done = bool(execution.get("motion_done", False))

    if motion_done and queue_depth == 0 and state_name in _RTCORE_TERMINAL_EXECUTION_STATES:
        return False
    if active_traj_id > 0 or queue_depth > 0:
        return True
    if state_name in {"accepted", "queued", "executing"}:
        return True
    if active_mode_name != "idle" and not motion_done and state_name not in _RTCORE_TERMINAL_EXECUTION_STATES:
        return True
    return False


def _wait_for_idle_terminal_state(
    current_payload: dict[str, object],
    last_active_payload: dict[str, object] | None,
) -> str:
    for payload in (current_payload, last_active_payload):
        if not isinstance(payload, dict):
            continue
        program = payload.get("program")
        if isinstance(program, dict):
            program_state = str(program.get("state", "")).strip().lower()
            if program_state in _PROGRAM_TERMINAL_STATES:
                return program_state
        execution = payload.get("execution")
        if isinstance(execution, dict):
            state_name = str(execution.get("state_name", "")).strip().lower()
            if state_name in _RTCORE_TERMINAL_EXECUTION_STATES - {"idle"}:
                return state_name
        state = str(payload.get("state", "")).strip().lower()
        if state in _RTCORE_TERMINAL_EXECUTION_STATES - {"idle"}:
            return state
    if last_active_payload is not None:
        return "completed"
    return "idle"


def _finalize_wait_for_idle_payload(
    current_payload: dict[str, object],
    *,
    timeout_s: float,
    waited_for_motion: bool,
    last_active_payload: dict[str, object] | None,
    timed_out: bool,
) -> dict[str, object]:
    payload = dict(current_payload)
    execution = payload.get("execution")
    execution_payload = dict(execution) if isinstance(execution, dict) else {}

    if waited_for_motion and isinstance(last_active_payload, dict):
        payload["completion_scope"] = str(
            last_active_payload.get("completion_scope", payload.get("completion_scope", "controller_idle"))
        )
        payload["source_of_truth"] = str(
            last_active_payload.get("source_of_truth", payload.get("source_of_truth", "controller"))
        )

    terminal_state = "timeout" if timed_out else _wait_for_idle_terminal_state(
        payload,
        last_active_payload,
    )
    payload["accepted"] = bool(waited_for_motion or payload.get("accepted", False) or terminal_state != "idle")
    payload["state"] = terminal_state
    payload["waited_for_motion"] = bool(waited_for_motion)
    payload["wait_timeout_s"] = float(timeout_s)
    payload["wait_timed_out"] = bool(timed_out)
    execution_payload["wait_terminal_state"] = terminal_state
    if isinstance(last_active_payload, dict):
        execution_payload["wait_last_active_state"] = str(
            last_active_payload.get("state", "")
        ).strip().lower()
        last_active_execution = last_active_payload.get("execution")
        if isinstance(last_active_execution, dict):
            execution_payload["wait_last_active_state_name"] = str(
                last_active_execution.get("state_name", "")
            ).strip().lower()
    payload["execution"] = execution_payload
    return payload


def _get_backend_power_transition_snapshot(backend) -> dict[str, object]:
    if backend is None:
        return {}
    getter = getattr(backend, "get_power_transition_snapshot", None)
    if not callable(getter):
        return {}
    try:
        snapshot = getter()
    except Exception:
        return {}
    return dict(snapshot) if isinstance(snapshot, dict) else {}


def _feedback_synchronization_blocker_from_snapshot(
    backend_snapshot: dict[str, object],
) -> dict[str, object]:
    truth_reasons = sorted(
        {
            str(value).strip()
            for value in list(backend_snapshot.get("feedback_truth_reasons", []))
            if str(value).strip()
        }
    )
    unavailable_axes = [
        int(value)
        for value in list(backend_snapshot.get("feedback_truth_unavailable_axes", []))
        if isinstance(value, (int, float))
    ]
    unavailable_joints = [
        int(value)
        for value in list(backend_snapshot.get("feedback_truth_unavailable_joints", []))
        if isinstance(value, (int, float))
    ]
    statuswords = sorted(
        {
            str(value).strip()
            for value in list(backend_snapshot.get("feedback_truth_statuswords", []))
            if str(value).strip()
        }
    )
    if truth_reasons == ["drive_native_coordinate_system_invalid"]:
        return {
            "code": "coordinate_system_invalid",
            "message": "Drive-native coordinate system is invalid; run Drive Home before power-up.",
            "truth_reasons": truth_reasons,
            "truth_unavailable_axes": unavailable_axes,
            "truth_unavailable_joints": unavailable_joints,
            "statuswords": statuswords,
            "requires_native_home": True,
        }
    if truth_reasons:
        return {
            "code": "canonical_truth_unavailable",
            "message": "Live joint truth is unavailable; keep the drives disarmed until telemetry is valid.",
            "truth_reasons": truth_reasons,
            "truth_unavailable_axes": unavailable_axes,
            "truth_unavailable_joints": unavailable_joints,
            "statuswords": statuswords,
        }
    return {
        "code": "not_synchronized",
        "message": "Live feedback is not synchronized yet; keep the drives disarmed.",
    }


def _build_power_transition_guard(
    *,
    controller_motion_state: str,
    controller_thread_running: bool,
    backend,
    execution_status,
) -> dict[str, object]:
    blockers: list[dict[str, object]] = []

    if controller_thread_running:
        blockers.append(
            {
                "code": "controller_thread_running",
                "message": "A controller motion/program thread is still running.",
                "controller_motion_state": controller_motion_state,
            }
        )

    if execution_status is not None:
        active_traj_id = int(getattr(execution_status, "active_traj_id", 0) or 0)
        queue_depth = int(getattr(execution_status, "queue_depth", 0) or 0)
        state_name = str(getattr(execution_status, "state_name", "idle") or "idle").strip().lower() or "idle"
        active_mode_name = (
            str(getattr(execution_status, "active_mode_name", "idle") or "idle").strip().lower() or "idle"
        )
        stale_command = bool(getattr(execution_status, "stale_command", False))

        if active_traj_id > 0:
            blockers.append(
                {
                    "code": "active_trajectory",
                    "message": "An RTCore trajectory is still latched or active.",
                    "active_traj_id": active_traj_id,
                }
            )
        if queue_depth > 0:
            blockers.append(
                {
                    "code": "queued_motion",
                    "message": "Queued RTCore motion points are still pending.",
                    "queue_depth": queue_depth,
                }
            )
        if state_name in {"accepted", "queued", "executing"} and active_traj_id == 0 and queue_depth == 0:
            blockers.append(
                {
                    "code": "motion_active",
                    "message": "RTCore still reports active motion execution.",
                    "state_name": state_name,
                    "active_mode_name": active_mode_name,
                }
            )
        if stale_command:
            blockers.append(
                {
                    "code": "stale_command",
                    "message": "RTCore reports a stale command; resynchronization is required before enable.",
                }
            )

    backend_snapshot = _get_backend_power_transition_snapshot(backend)
    active_jog = bool(backend_snapshot.get("active_jog", False))
    faulted_axis_count = int(backend_snapshot.get("faulted_axis_count", 0) or 0)
    feedback_synchronized = bool(backend_snapshot.get("feedback_synchronized", False))
    if active_jog:
        blockers.append(
            {
                "code": "active_jog",
                "message": "A jog command is still active in RTCore.",
                "active_jog_axis_mask": int(backend_snapshot.get("active_jog_axis_mask", 0) or 0),
            }
        )
    if faulted_axis_count > 0:
        blockers.append(
            {
                "code": "fault_present",
                "message": "One or more drives are still faulted.",
                "faulted_axis_count": faulted_axis_count,
                "faulted_axis_indices": list(backend_snapshot.get("faulted_axis_indices", [])),
            }
        )
    if backend is not None and not feedback_synchronized:
        blockers.append(_feedback_synchronization_blocker_from_snapshot(backend_snapshot))

    blocker_codes = [str(item.get("code", "")).strip() for item in blockers if str(item.get("code", "")).strip()]
    return {
        "safe_for_power_transition": len(blockers) == 0,
        "power_transition_blockers": blocker_codes,
        "power_transition_blocker_details": blockers,
        "power_transition_feedback_synchronized": feedback_synchronized,
        "power_transition_faulted_axis_count": faulted_axis_count,
        "power_transition_active_jog": active_jog,
    }


def _build_motion_execution_metadata(
    *,
    accepted: bool,
    completion_scope: str,
    state: str | None = None,
    use_rtcore_status: bool = False,
    extra: dict[str, object] | None = None,
) -> dict[str, object]:
    snapshot = utils.trajectory_state_snapshot()
    controller_motion_state = str(snapshot.get("motion_state", "IDLE")).strip().lower() or "idle"
    controller_thread_running = bool(snapshot.get("is_running", False))
    backend = None
    execution_status = None
    if use_rtcore_status:
        backend, execution_status = _get_rtcore_execution_backend_and_status()

    trajectory_id = 0
    derived_state = str(state).strip().lower() if state is not None else ""
    source_of_truth = "controller"
    execution: dict[str, object] = {
        "controller_motion_state": controller_motion_state,
        "controller_thread_running": controller_thread_running,
        "last_correlation_id": snapshot.get("last_correlation_id"),
        "rtcore_status_present": execution_status is not None,
    }

    if execution_status is not None:
        source_of_truth = "rtcore"
        execution_status, submitted_traj_id = _refresh_rtcore_execution_status_for_new_submission(
            backend,
            execution_status,
            accepted=accepted,
            controller_motion_state=controller_motion_state,
            controller_thread_running=controller_thread_running,
        )
        active_traj_id = int(getattr(execution_status, "active_traj_id", 0) or 0)
        trajectory_timing: dict[str, object] = {}
        if backend is not None:
            timing_getter = getattr(backend, "get_last_trajectory_timing", None)
            if callable(timing_getter):
                try:
                    raw_timing = timing_getter()
                    if isinstance(raw_timing, dict):
                        trajectory_timing = {
                            "trajectory_requested_frequency_hz": int(raw_timing.get("requested_frequency_hz", 0) or 0),
                            "trajectory_effective_frequency_hz": int(raw_timing.get("effective_frequency_hz", 0) or 0),
                            "trajectory_cycle_ns": int(raw_timing.get("cycle_ns", 0) or 0),
                            "trajectory_step_ns": int(raw_timing.get("step_ns", 0) or 0),
                            "trajectory_cycles_per_point": int(raw_timing.get("cycles_per_point", 0) or 0),
                        }
                except Exception:
                    trajectory_timing = {}
        trajectory_id = submitted_traj_id if submitted_traj_id > active_traj_id else (active_traj_id or submitted_traj_id)
        observed_state = (
            str(getattr(execution_status, "state_name", "idle")).strip().lower() or "idle"
        )
        program_thread_active = controller_thread_running and bool(
            snapshot.get("active_program_name")
        )
        rtcore_segment_active = (
            active_traj_id > 0 or int(getattr(execution_status, "queue_depth", 0) or 0) > 0
        )
        if not derived_state:
            if program_thread_active and not rtcore_segment_active:
                source_of_truth = "controller_program_thread"
                derived_state = "executing"
            elif (
                observed_state in _RTCORE_TERMINAL_EXECUTION_STATES
                and bool(getattr(execution_status, "motion_done", False))
            ):
                derived_state = observed_state
            elif rtcore_segment_active:
                derived_state = observed_state
            elif trajectory_id > 0 and accepted:
                derived_state = "accepted"
            else:
                derived_state = observed_state
        execution.update(
            {
                "active_mode": int(getattr(execution_status, "active_mode", 0) or 0),
                "active_mode_name": str(getattr(execution_status, "active_mode_name", "idle")),
                "state_id": int(getattr(execution_status, "state", 0) or 0),
                "state_name": observed_state,
                "active_traj_id": active_traj_id,
                "current_point_index": getattr(execution_status, "current_point_index", None),
                "queue_depth": int(getattr(execution_status, "queue_depth", 0) or 0),
                "queue_capacity": int(getattr(execution_status, "queue_capacity", 0) or 0),
                "last_event_code": int(getattr(execution_status, "last_event_code", 0) or 0),
                "underrun_count": int(getattr(execution_status, "underrun_count", 0) or 0),
                "stale_command": bool(getattr(execution_status, "stale_command", False)),
                "motion_done": bool(getattr(execution_status, "motion_done", False)),
                "capability_flags": int(getattr(execution_status, "capability_flags", 0) or 0),
                "active_command_seq": int(getattr(execution_status, "active_command_seq", 0) or 0),
                "last_update_ns": int(getattr(execution_status, "last_update_ns", 0) or 0),
                "last_submitted_traj_id": submitted_traj_id,
                **trajectory_timing,
            }
        )

    if not derived_state:
        if controller_thread_running:
            derived_state = "executing" if controller_motion_state == "executing" else "accepted"
        else:
            derived_state = "accepted" if accepted else "idle"

    payload: dict[str, object] = {
        "accepted": bool(accepted),
        "state": derived_state,
        "completion_scope": str(completion_scope),
        "trajectory_id": int(trajectory_id),
        "source_of_truth": source_of_truth,
        "execution": execution,
    }
    power_transition_guard = _build_power_transition_guard(
        controller_motion_state=controller_motion_state,
        controller_thread_running=controller_thread_running,
        backend=backend,
        execution_status=execution_status,
    )
    execution.update(
        {
            "safe_for_power_transition": bool(power_transition_guard["safe_for_power_transition"]),
            "power_transition_blockers": list(power_transition_guard["power_transition_blockers"]),
            "power_transition_blocker_details": list(power_transition_guard["power_transition_blocker_details"]),
            "power_transition_feedback_synchronized": bool(
                power_transition_guard["power_transition_feedback_synchronized"]
            ),
            "power_transition_faulted_axis_count": int(
                power_transition_guard["power_transition_faulted_axis_count"]
            ),
            "power_transition_active_jog": bool(power_transition_guard["power_transition_active_jog"]),
        }
    )
    payload["safe_for_power_transition"] = bool(power_transition_guard["safe_for_power_transition"])
    payload["power_transition_blockers"] = list(power_transition_guard["power_transition_blockers"])
    payload["power_transition_blocker_details"] = list(power_transition_guard["power_transition_blocker_details"])
    if extra:
        payload.update(extra)
    _attach_program_status(payload, snapshot)
    return payload


def get_motion_execution_status() -> dict[str, object]:
    backend, execution_status = _get_rtcore_execution_backend_and_status()
    snapshot = utils.trajectory_state_snapshot()
    controller_thread_running = bool(snapshot.get("is_running", False))
    controller_motion_state = str(snapshot.get("motion_state", "IDLE")).strip().lower() or "idle"
    active_program = bool(snapshot.get("active_program_name"))
    if execution_status is not None:
        execution_status, submitted_traj_id = _refresh_rtcore_execution_status_for_new_submission(
            backend,
            execution_status,
            accepted=True,
            controller_motion_state=controller_motion_state,
            controller_thread_running=controller_thread_running,
        )
        accepted = bool(
            int(getattr(execution_status, "active_traj_id", 0) or 0) > 0
            or submitted_traj_id > 0
            or int(getattr(execution_status, "last_update_ns", 0) or 0) > 0
            or controller_thread_running
            or controller_motion_state not in {"idle", ""}
        )
        queue_depth = int(getattr(execution_status, "queue_depth", 0) or 0)
        active_traj_id = int(getattr(execution_status, "active_traj_id", 0) or 0)
        completion_scope = (
            "controller_program_thread"
            if controller_thread_running and active_program and active_traj_id == 0 and queue_depth == 0
            else "rtcore_execution"
        )
        return _build_motion_execution_metadata(
            accepted=accepted,
            completion_scope=completion_scope,
            use_rtcore_status=True,
        )

    controller_motion_state = str(snapshot.get("motion_state", "IDLE")).strip().lower() or "idle"
    completion_scope = (
        "controller_program_thread"
        if controller_thread_running and active_program
        else "controller_trajectory_thread" if controller_thread_running else "controller_idle"
    )
    state = controller_motion_state if controller_thread_running else "idle"
    accepted = controller_thread_running or controller_motion_state not in {"idle", ""}
    return _build_motion_execution_metadata(
        accepted=accepted,
        completion_scope=completion_scope,
        state=state,
        use_rtcore_status=False,
    )


def _coerce_direct_setpoint_speed(value: int | float | str | None) -> int:
    try:
        if value is None:
            raise TypeError("missing speed")
        return int(value)
    except Exception:
        return _DIRECT_SETPOINT_FALLBACK_SPEED


def _coerce_direct_setpoint_acceleration(value: float | int | str | None) -> float:
    try:
        if value is None:
            raise TypeError("missing acceleration")
        return float(value)
    except Exception:
        return _DIRECT_SETPOINT_FALLBACK_ACCELERATION_DEG_S2


def _resolve_profile_params_for_speed_multiplier(speed_multiplier: float | int | str | None) -> tuple[float, float, float]:
    """
    Normalize a speed multiplier and derive profile velocity/acceleration.

    We intentionally scale acceleration by speed^2 so that short, acceleration-
    limited moves still exhibit a clear and near-linear time scaling with the
    UI speed multiplier.

    Returns:
        tuple[float, float, float]: (normalized_multiplier, velocity_m_s, acceleration_m_s2)
    """
    try:
        multiplier = float(speed_multiplier) if speed_multiplier is not None else 1.0
    except (TypeError, ValueError):
        multiplier = 1.0
    if not np.isfinite(multiplier):
        multiplier = 1.0

    # Match UI slider semantics (0.1x .. 10x) while keeping backend-safe bounds.
    multiplier = float(np.clip(multiplier, 0.1, 10.0))

    base_velocity = (
        float(utils.DEFAULT_PROFILE_VELOCITY)
        if utils.DEFAULT_PROFILE_VELOCITY is not None
        and np.isfinite(utils.DEFAULT_PROFILE_VELOCITY)
        and float(utils.DEFAULT_PROFILE_VELOCITY) > 0.0
        else 0.08
    )
    base_acceleration = (
        float(utils.DEFAULT_PROFILE_ACCELERATION)
        if utils.DEFAULT_PROFILE_ACCELERATION is not None
        and np.isfinite(utils.DEFAULT_PROFILE_ACCELERATION)
        and float(utils.DEFAULT_PROFILE_ACCELERATION) > 0.0
        else 0.2
    )
    velocity = float(base_velocity * multiplier)
    acceleration = float(base_acceleration * (multiplier ** 2))
    return multiplier, velocity, acceleration


def _resolve_profile_params_for_linear_speed_m_s(
    requested_velocity_m_s: float | int | str | None,
    requested_acceleration_m_s2: float | int | str | None = None,
) -> tuple[float, float, float]:
    """
    Resolve an absolute linear speed in m/s plus optional acceleration in m/s^2.

    For authored trajectory segments we use direct physical motion semantics:
    - velocity comes from the requested line speed (or controller default)
    - acceleration comes from the requested line acceleration when provided
    - otherwise the controller defaults to reaching the commanded speed in
      about one second

    Returning the normalized multiplier as the first tuple element preserves the
    existing helper shape for callers and diagnostics.

    Returns:
        tuple[float, float, float]: (normalized_multiplier, velocity_m_s, acceleration_m_s2)
    """
    base_velocity = (
        float(utils.DEFAULT_PROFILE_VELOCITY)
        if utils.DEFAULT_PROFILE_VELOCITY is not None
        and np.isfinite(utils.DEFAULT_PROFILE_VELOCITY)
        and float(utils.DEFAULT_PROFILE_VELOCITY) > 0.0
        else 0.08
    )
    try:
        requested_velocity = float(requested_velocity_m_s) if requested_velocity_m_s is not None else base_velocity
    except (TypeError, ValueError):
        requested_velocity = base_velocity
    if not np.isfinite(requested_velocity) or requested_velocity <= 0.0:
        requested_velocity = base_velocity
    try:
        requested_acceleration = (
            float(requested_acceleration_m_s2)
            if requested_acceleration_m_s2 is not None
            else (requested_velocity / 1.0)
        )
    except (TypeError, ValueError):
        requested_acceleration = requested_velocity / 1.0
    if not np.isfinite(requested_acceleration) or requested_acceleration <= 0.0:
        requested_acceleration = requested_velocity / 1.0
    normalized_multiplier = float(requested_velocity / base_velocity) if base_velocity > 0.0 else 1.0
    return normalized_multiplier, requested_velocity, requested_acceleration

def handle_translate_command(dx: float, dy: float, dz: float):
    """
    Handles the 'TRANSLATE' command.
    Performs a simple, blocking, single-point IK move relative to the current
    pose while keeping orientation locked.
    """
    utils.trajectory_state["should_stop"] = False # Reset stop flag on new move
    print(f"[Pi IK] Received TRANSLATE command: dx={dx}, dy={dy}, dz={dz}")

    # 1. Get current logical joint angles from our global state
    initial_angles = utils.current_logical_joint_angles_rad
    print(f"[Pi IK] Initial logical joint angles (rad): {np.round(initial_angles, 3)}")

    # 2. Use Forward Kinematics (FK) to find the current full pose (position and orientation)
    current_pose_matrix = ik_solver.get_fk_matrix(initial_angles)
    if current_pose_matrix is None:
        print("[Pi IK] ERROR: Failed to calculate current pose using FK.")
        return
    
    current_pos_xyz = current_pose_matrix[:3, 3]
    # This is the key: Lock the orientation to the current one.
    target_orientation_matrix = current_pose_matrix[:3, :3]
    print(f"[Pi IK] Current EE position (m): {np.round(current_pos_xyz, 4)}")
    print(f"[Pi IK] Locking orientation during translation.")

    # 3. Calculate the target position by adding the deltas
    target_pos_xyz = current_pos_xyz + np.array([dx, dy, dz])
    print(f"[Pi IK] Target EE position (m):  {np.round(target_pos_xyz, 4)}")

    # 4. Use Inverse Kinematics (IK) to find the required joint angles for the target position
    #    We provide the current angles as the starting point and the locked orientation.
    new_logical_joint_angles = ik_solver.solve_ik(
        target_position=target_pos_xyz,
        target_orientation_matrix=target_orientation_matrix,
        initial_joint_angles=initial_angles
    )

    if new_logical_joint_angles is None:
        print("[Pi IK] ERROR: IK solver failed to find a solution.")
        return

    print(f"[Pi IK] IK Solution Found (rad): {np.round(new_logical_joint_angles, 3)}")
    print(f"[Pi IK] IK Solution Found (deg): {np.round(np.rad2deg(new_logical_joint_angles), 2)}")

    # 5. Command the servos to the new angles
    #    Using default speed and acceleration for now. This could be made adjustable.
    servo_driver.set_servo_positions(new_logical_joint_angles, utils.DEFAULT_SERVO_SPEED, utils.DEFAULT_SERVO_ACCELERATION_DEG_S2)
    print("[Pi IK] Sent new positions to servos.")

    # 6. Get and print the final position for verification
    final_pos_xyz = ik_solver.get_fk(new_logical_joint_angles)
    if final_pos_xyz is not None:
        print(f"[Pi IK] Verification -> Target: {np.round(target_pos_xyz, 4)}, Final FK: {np.round(final_pos_xyz, 4)}")
        print(f"[Pi IK] Distance from target: {np.linalg.norm(final_pos_xyz - target_pos_xyz):.6f} m")

def _get_live_pose_snapshot() -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Snapshot of the robot's current joint/pose state from live feedback."""
    initial_angles = servo_driver.get_current_arm_state_rad(verbose=False)
    if initial_angles is None:
        raise RuntimeError("Failed to read current arm state from hardware.")
    initial_angles_np = np.asarray(initial_angles, dtype=float)
    current_pose_matrix = ik_solver.get_fk_matrix(initial_angles_np)
    if current_pose_matrix is None:
        raise RuntimeError("Failed to calculate current pose using FK.")
    current_position = np.asarray(current_pose_matrix[:3, 3], dtype=float)
    current_orientation = np.asarray(current_pose_matrix[:3, :3], dtype=float)
    return initial_angles_np, current_position, current_orientation


def _get_control_pose_snapshot() -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Snapshot of the robot's current joint/pose state for motion planning."""
    initial_angles = servo_driver.get_control_arm_state_rad(verbose=False)
    if initial_angles is None:
        raise RuntimeError("Failed to read current control arm state from hardware.")
    initial_angles_np = np.asarray(initial_angles, dtype=float)
    current_pose_matrix = ik_solver.get_fk_matrix(initial_angles_np)
    if current_pose_matrix is None:
        raise RuntimeError("Failed to calculate current pose using FK.")
    current_position = np.asarray(current_pose_matrix[:3, 3], dtype=float)
    current_orientation = np.asarray(current_pose_matrix[:3, :3], dtype=float)
    return initial_angles_np, current_position, current_orientation


def _get_best_available_joint_state() -> np.ndarray:
    """Prefer trustworthy live feedback, but fall back to cached controller state."""
    cached_angles_np = np.asarray(utils.current_logical_joint_angles_rad, dtype=float).reshape(-1)
    num_joints = int(utils.NUM_LOGICAL_JOINTS or 0)
    if cached_angles_np.size == 0 and num_joints > 0:
        cached_angles_np = np.zeros(num_joints, dtype=float)

    should_attempt_live_feedback = False
    try:
        active_backend = backend_registry.get_active_backend()
        should_attempt_live_feedback = bool(
            active_backend is not None and getattr(active_backend, "is_initialized", False)
        )
    except Exception:
        should_attempt_live_feedback = False

    if not should_attempt_live_feedback:
        try:
            # In the API process there is usually no active actuator backend, and
            # legacy serial reads can collapse to a synthetic all-zero vector.
            # Only trust the legacy read path when servos were actually detected.
            should_attempt_live_feedback = len(servo_driver.servo_protocol.get_present_servo_ids()) > 0
        except Exception:
            should_attempt_live_feedback = False

    if should_attempt_live_feedback:
        live_angles = servo_driver.get_current_arm_state_rad(verbose=False)
        if live_angles is not None:
            live_angles_np = np.asarray(live_angles, dtype=float).reshape(-1)
            if live_angles_np.size > 0:
                return live_angles_np

    return cached_angles_np


def _execute_orientation_path(
    target_orientation: np.ndarray,
    *,
    command_name: str = "SET_ORIENTATION",
    closed_loop: bool = True,
    duration_s: float = 2.0,
    diagnostics: bool = False,
    initial_angles: np.ndarray | None = None,
    current_position: np.ndarray | None = None,
    current_orientation: np.ndarray | None = None,
) -> dict[str, object]:
    """Plan and execute a smooth orientation-only move from live robot state."""
    if initial_angles is None or current_position is None or current_orientation is None:
        initial_angles, current_position, current_orientation = _get_control_pose_snapshot()

    effective_closed_loop, execution_policy = _resolve_scheduled_motion_execution_policy(
        closed_loop_requested=closed_loop,
        command_name=command_name,
    )

    assert initial_angles is not None
    assert current_position is not None
    assert current_orientation is not None

    # --- Set up diagnostics session if enabled ---
    session_id = None
    diagnostics_enabled = (
        os.environ.get("MINI_ARM_IK_LOG", "0") == "1"
        or diagnostics
        or utils.trajectory_state.get("diagnostics_enabled", False)
    )
    if diagnostics_enabled:
        session_id = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        utils.trajectory_state['diagnostics_session_id'] = session_id
        utils.trajectory_state['diagnostics_folder_type'] = (
            "closed_loop" if effective_closed_loop else "open_loop"
        )

    frequency_hz = 50 if effective_closed_loop else 100
    duration_s = max(0.1, duration_s)
    num_steps = max(2, int(duration_s * frequency_hz))

    try:
        from scipy.spatial.transform import Slerp

        rot_start = R.from_matrix(current_orientation)
        rot_end = R.from_matrix(target_orientation)
        key_rots = R.concatenate([rot_start, rot_end])
        key_times = [0, 1]
        slerp = Slerp(key_times, key_rots)
        times = np.linspace(0, 1, num_steps)
        interpolated_rots = slerp(times)
        orientation_matrices = [r.as_matrix() for r in interpolated_rots]
    except Exception as e:
        if session_id:
            del utils.trajectory_state['diagnostics_session_id']
            del utils.trajectory_state['diagnostics_folder_type']
        raise RuntimeError(f"Failed to build SLERP interpolation: {e}") from e

    path_positions = [current_position] * num_steps
    joint_path = ik_solver.solve_ik_path_batch(
        path_points=path_positions,
        initial_joint_angles=initial_angles,
        target_orientations=orientation_matrices,
    )
    if joint_path is None:
        if session_id:
            del utils.trajectory_state['diagnostics_session_id']
            del utils.trajectory_state['diagnostics_folder_type']
        raise RuntimeError("IK solver failed to find a solution for the orientation path.")

    target_func = (
        trajectory_execution._closed_loop_executor_thread
        if effective_closed_loop
        else trajectory_execution._open_loop_executor_thread
    )
    executor_error: list[BaseException] = []

    def _run_executor() -> None:
        try:
            target_func(
                joint_path=joint_path,
                frequency=frequency_hz,
                diagnostics=diagnostics_enabled,
            )
        except BaseException as exc:  # pragma: no cover - re-raised after join
            executor_error.append(exc)

    executor_thread = threading.Thread(
        target=_run_executor,
        daemon=True,
    )

    utils.trajectory_state_update(thread=executor_thread, is_running=True, should_stop=False)
    try:
        utils.set_motion_state("EXECUTING")
    except Exception:
        pass

    try:
        executor_thread.start()
        executor_thread.join()
        if executor_error:
            raise RuntimeError(str(executor_error[0])) from executor_error[0]
    finally:
        utils.trajectory_state_update(thread=None, is_running=False)
        try:
            utils.set_motion_state("IDLE")
        except Exception:
            pass

    final_pose_matrix = ik_solver.get_fk_matrix(joint_path[-1])
    if final_pose_matrix is not None:
        final_position = final_pose_matrix[:3, 3]
        final_orientation = final_pose_matrix[:3, :3]
        orient_error_matrix = np.transpose(target_orientation) @ final_orientation
        orient_error_rotvec = R.from_matrix(orient_error_matrix).as_rotvec()
        print(f"[Pi IK] Verification -> Final Pos: {np.round(final_position, 4)}")
        print(f"[Pi IK] Positional error: {np.linalg.norm(final_position - current_position):.6f} m")
        print(f"[Pi IK] Orientational error: {np.rad2deg(np.linalg.norm(orient_error_rotvec)):.3f} degrees")

    if session_id:
        del utils.trajectory_state['diagnostics_session_id']
        del utils.trajectory_state['diagnostics_folder_type']

    return _build_motion_execution_metadata(
        accepted=True,
        completion_scope=_default_completion_scope(
            closed_loop=effective_closed_loop,
            prefer_rtcore=not effective_closed_loop,
        ),
        state="completed",
        use_rtcore_status=(not effective_closed_loop),
        extra={
            "duration_s": float(duration_s),
            "frequency_hz": int(frequency_hz),
            "closed_loop": bool(effective_closed_loop),
            **execution_policy,
        },
    )


def handle_rotate_command(axis: str, angle_deg: float, *, duration_s: float | None = None) -> dict[str, object]:
    """
    Handles the 'ROTATE' command.
    Performs a smooth relative rotation around a specified base-frame axis.
    """
    utils.trajectory_state["should_stop"] = False # Reset stop flag on new move
    _begin_non_program_motion()
    print(f"[Pi IK] Received ROTATE command: axis={axis}, angle={angle_deg} degrees")

    if utils.trajectory_state.get("is_running"):
        raise RuntimeError("Cannot start ROTATE, another task is running.")

    initial_angles, current_position, current_orientation = _get_control_pose_snapshot()
    print(f"[Pi IK] Initial logical joint angles (rad): {np.round(initial_angles, 3)}")
    print(f"[Pi IK] Current EE position (m): {np.round(current_position, 4)}")

    try:
        rotation = R.from_euler(axis, angle_deg, degrees=True).as_matrix()
        target_orientation = rotation @ current_orientation
    except Exception as e:
        raise RuntimeError(f"Failed to create rotation matrix: {e}") from e

    print(f"[Pi IK] Target EE orientation matrix:\n{np.round(target_orientation, 2)}")
    rotate_duration_s = (
        max(0.1, float(duration_s))
        if duration_s is not None
        else max(0.12, min(0.75, abs(float(angle_deg)) / 90.0))
    )
    payload = _execute_orientation_path(
        target_orientation,
        command_name="ROTATE",
        closed_loop=False,
        duration_s=rotate_duration_s,
        diagnostics=False,
        initial_angles=initial_angles,
        current_position=current_position,
        current_orientation=current_orientation,
    )
    payload.update(
        {
            "axis": str(axis),
            "angle_deg": float(angle_deg),
        }
    )
    return payload


def handle_set_orientation_command(
    roll: float,
    pitch: float,
    yaw: float,
    *,
    closed_loop: bool = True,
    duration_s: float = 2.0,
    diagnostics: bool = False,
) -> dict[str, object]:
    """
    Handles the `SET_ORIENTATION` command.

    This command **smoothly re-orients** the tool tip to the specified absolute
    Euler angles **while keeping its Cartesian position fixed**.  Internally it:

    1.  Interpolates between the current and target orientations with a SLERP
        curve (density chosen from `duration_s` × execution frequency).
    2.  Solves IK in a single batched call for every intermediate pose, so the
        position constraint is enforced at all times.
    3.  Executes the resulting joint path either:
        • **Closed-loop** at 50 Hz (default)
        • **Open-loop** at 100 Hz (`closed_loop=False`)

    Because the path is pre-planned, the function is *blocking*: it only
    returns after the motion (≈ `duration_s`) has finished.

    Parameters
    ----------
    roll, pitch, yaw : float
        Absolute tool orientation in degrees, XYZ intrinsic Euler order.
    closed_loop : bool, optional
        Executes closed-loop at 50 Hz; open-loop at 100 Hz when `False`.
        Default `True` for closed-loop.
    duration_s : float, optional
        Desired motion duration (≥ 0.1 s).  Controls the smoothness/speed by
        scaling the number of interpolation steps.  Default `1.0`.
    """
    utils.trajectory_state["should_stop"] = False # Reset stop flag on new move
    _begin_non_program_motion()
    print(f"[Pi IK] Received SET_ORIENTATION command: Roll={roll}, Pitch={pitch}, Yaw={yaw} degrees")

    if utils.trajectory_state.get("is_running"):
        raise RuntimeError("Cannot start SET_ORIENTATION, another task is running.")

    initial_angles, current_position, current_orientation = _get_control_pose_snapshot()

    # 2. Build the target orientation matrix from Euler angles (XYZ intrinsic).
    try:
        target_orientation = R.from_euler('xyz', [roll, pitch, yaw], degrees=True).as_matrix()
    except Exception as e:
        raise RuntimeError(f"Failed to create orientation matrix from Euler angles: {e}") from e

    print(f"[Pi IK] Target EE Orientation Matrix:\n{np.round(target_orientation, 2)}")
    print(f"[Pi IK] Maintaining EE Position at: {np.round(current_position, 4)}")

    payload = _execute_orientation_path(
        target_orientation,
        command_name="SET_ORIENTATION",
        closed_loop=closed_loop,
        duration_s=duration_s,
        diagnostics=diagnostics,
        initial_angles=initial_angles,
        current_position=current_position,
        current_orientation=current_orientation,
    )
    payload.update(
        {
            "roll_deg": float(roll),
            "pitch_deg": float(pitch),
            "yaw_deg": float(yaw),
        }
    )
    return payload


def handle_move_profiled(target_x: float,
                         target_y: float,
                         target_z: float,
                         velocity: float,
                         acceleration: float,
                         frequency: int = 100,
                         use_smoothing: bool = True,
                         closed_loop: bool = False,
                         diagnostics: bool = False,
                         command_name: str = "MOVE_LINE",
                         ) -> dict[str, object]:
    """
    Handles the 'MOVE_PROFILED' command. This is the core handler for all
    high-precision, profiled, non-blocking linear moves. It plans the full path,
    then starts the requested executor in a background thread.
    """
    utils.trajectory_state["should_stop"] = False # Reset stop flag on new move
    _begin_non_program_motion()
    print(f"[Pi Smooth] Received MOVE_PROFILED command to [{target_x}, {target_y}, {target_z}]")
    
    if bool(utils.trajectory_state_get("is_running", False)):
        raise RuntimeError("Cannot start move, another task is running.")

    _require_target_axes_motion_ready(None)

    # 1. Get current state from the physical robot to start the plan
    initial_q = servo_driver.get_control_arm_state_rad(verbose=False)
    target_pos = np.array([target_x, target_y, target_z])

    diagnostics_enabled = (
        os.environ.get("MINI_ARM_IK_LOG", "0") == "1"
        or diagnostics
        or utils.trajectory_state.get("diagnostics_enabled", False)
    )
    effective_closed_loop, execution_policy = _resolve_scheduled_motion_execution_policy(
        closed_loop_requested=closed_loop,
        command_name=command_name,
    )

    # --- Set up diagnostics session if enabled ---
    if diagnostics_enabled:
        session_id = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        utils.trajectory_state['diagnostics_session_id'] = session_id
        utils.trajectory_state['diagnostics_folder_type'] = (
            "closed_loop" if effective_closed_loop else "open_loop"
        )

    if effective_closed_loop:
        frequency = 50
    else:
        # Standardize default open-loop planning/execution to 100 Hz as well
        frequency = 100

    # 2. Plan the entire move.
    joint_path = trajectory_execution._plan_smooth_move(
        start_q=initial_q,
        target_pos=target_pos,
        velocity=velocity,
        acceleration=acceleration,
        frequency=frequency,
        use_smoothing=use_smoothing
    )
    
    # 3. If planning was successful, choose executor
    if joint_path:
        executor_fn = (trajectory_execution._closed_loop_executor_thread
                       if effective_closed_loop
                       else trajectory_execution._open_loop_executor_thread)

        executor_thread = threading.Thread(
            target=executor_fn,
            # Pass diagnostics flag to the executor thread
            kwargs={'joint_path': joint_path, 'frequency': frequency, 'diagnostics': diagnostics_enabled},
            daemon=True,
        )
        utils.trajectory_state_update(thread=executor_thread, is_running=True, should_stop=False)
        try:
            utils.set_motion_state("EXECUTING")
        except Exception:
            pass
        executor_thread.start()
        print("[Pi Smooth] Trajectory started "
              f"({'closed' if effective_closed_loop else 'open'} loop, background).")
        return _build_motion_execution_metadata(
            accepted=True,
            completion_scope=_default_completion_scope(
                closed_loop=effective_closed_loop,
                prefer_rtcore=not effective_closed_loop,
            ),
            state="accepted",
            use_rtcore_status=(not effective_closed_loop),
            extra={
                "velocity": float(velocity),
                "acceleration": float(acceleration),
                "frequency_hz": int(frequency),
                "closed_loop": bool(effective_closed_loop),
                **execution_policy,
            },
        )

    raise RuntimeError("Move failed because path planning was unsuccessful.")


def handle_move_profiled_relative(dx: float, dy: float, dz: float, speed: float = 1.0, use_smoothing: bool = True):
    """
    Handles the 'MOVE_PROFILED_RELATIVE' command. Calculates the absolute
    target position and then calls the main `handle_move_profiled` handler.
    """
    utils.trajectory_state["should_stop"] = False # Reset stop flag on new move
    print(f"[Pi Smooth] Received MOVE_PROFILED_RELATIVE command: dX={dx}, dY={dy}, dZ={dz}, SpeedMultiplier={speed}")

    _require_target_axes_motion_ready(None)

    # 1. Get current position
    current_q = servo_driver.get_control_arm_state_rad(verbose=False)
    start_pos = ik_solver.get_fk(current_q)
    if start_pos is None:
        print("[Pi Smooth] ERROR: Cannot start relative move, failed to get current position.")
        return
        
    # 2. Calculate absolute target position
    target_pos = start_pos + np.array([dx, dy, dz])
    
    # 3. Calculate profiled move parameters
    resolved_speed, target_velocity, target_acceleration = _resolve_profile_params_for_speed_multiplier(speed)
    
    # 4. Call the absolute profiled move handler to perform the action
    print(
        f"[Pi Smooth] Calculated absolute target: {np.round(target_pos, 4)}. "
        f"Executing profiled move at {resolved_speed:.2f}x."
    )
    handle_move_profiled(target_pos[0], target_pos[1], target_pos[2], target_velocity, target_acceleration, use_smoothing=use_smoothing)


def _finish_failed_program_run(*, state: str, terminal_reason: str, failing_step_index: int | None) -> None:
    """Finalize program status when a looped trajectory's preflight fails.

    Used by the background loop wrapper so a fault during the move-to-start
    preflight cannot leave `program_status` advertising an active program nor
    leave `is_running` stuck. Mirrors the cleanup that the synchronous loop
    branch used to perform inline before the preflight was moved off-thread.
    """

    _update_program_status(
        active=False,
        state=state,
        terminal_reason=terminal_reason,
        failing_step_index=failing_step_index,
        current_step_index=None,
        current_step_type=None,
    )
    utils.trajectory_state_update(
        is_running=False,
        thread=None,
        weld_active=False,
        current_weld_type=None,
        active_program_name=None,
        active_program_loop_enabled=False,
        active_program_use_cache=False,
        active_program_step_count=0,
        active_program_move_steps=0,
        active_program_pause_steps=0,
        active_program_joint_move_steps=0,
        active_program_rtcore_segments=False,
        active_program_segment_execution_policy="",
        active_program_step_index=None,
        active_program_step_type=None,
        active_program_loop_iteration=0,
    )
    target_motion_state = "FAULT" if state == "faulted" else "IDLE"
    try:
        utils.set_motion_state(target_motion_state)
    except Exception as exc:
        print(
            "[Pi Trajectory] WARNING: failed to set motion state after program failure: "
            f"target={target_motion_state} error={exc}"
        )


def _looping_trajectory_executor_thread(
    *,
    initial_joint_path: list[list[float]],
    initial_frequency_hz: int,
    loop_steps: list[dict[str, object]],
    loop_enabled: bool,
) -> None:
    """Background entry point for looped trajectories.

    Runs the move-to-start wrapper as a strict RTCore preflight BEFORE handing
    off to the regular trajectory executor for the loop body. RTCore must report
    completion, then live control feedback must confirm the endpoint; otherwise
    the program is marked faulted/aborted and the loop body never runs.
    """

    if utils.trajectory_state_get("should_stop", False):
        _finish_failed_program_run(
            state="aborted",
            terminal_reason=str(utils.trajectory_state_get("stop_request_reason", None) or "operator_abort"),
            failing_step_index=0,
        )
        return

    print("[Pi Trajectory] Executing loop move-to-start preflight.")
    try:
        trajectory_execution._open_loop_executor_thread(
            joint_path=initial_joint_path,
            frequency=initial_frequency_hz,
            diagnostics=False,
            owns_trajectory_state=False,
            require_completion=True,
        )
    except Exception as exc:
        print(f"[Pi Trajectory] ERROR: Loop move-to-start preflight failed: {exc}")
        _finish_failed_program_run(
            state="faulted",
            terminal_reason="loop_start_failed",
            failing_step_index=0,
        )
        return

    if utils.trajectory_state_get("should_stop", False):
        _finish_failed_program_run(
            state="aborted",
            terminal_reason=str(utils.trajectory_state_get("stop_request_reason", None) or "operator_abort"),
            failing_step_index=0,
        )
        return

    # Verify the live/control pose actually reached the wrapper endpoint
    # before the loop body starts. Tolerance is small enough to catch a
    # stale or in-flight wrapper completion but larger than normal servo
    # following noise.
    try:
        live_q = servo_driver.get_control_arm_state_rad(verbose=False)
        if live_q is None:
            raise RuntimeError("control_feedback_unavailable")
        target_q = np.asarray(initial_joint_path[-1], dtype=float)
        max_abs_err = float(np.max(np.abs(np.asarray(live_q, dtype=float) - target_q)))
        if max_abs_err > 0.05:
            raise RuntimeError(
                "loop_start_endpoint_mismatch: "
                f"max_abs_err_rad={max_abs_err:.6f} tolerance_rad=0.050000"
            )
    except Exception as exc:
        print(f"[Pi Trajectory] ERROR: Loop move-to-start endpoint verification failed: {exc}")
        _finish_failed_program_run(
            state="faulted",
            terminal_reason="loop_start_failed",
            failing_step_index=0,
        )
        return

    print("[Pi Trajectory] Loop move-to-start preflight complete. Starting loop body.")
    trajectory_execution._trajectory_executor_thread(loop_steps, loop_enabled)


def handle_run_trajectory(trajectory_name: str, use_cache: bool = False, loop_override: bool | None = None) -> dict[str, object]:
    """
    Handles the 'RUN_TRAJECTORY' command. It loads a trajectory definition
    from a JSON file, plans all the constituent moves, and then starts the
    trajectory executor thread to run the full sequence.
    The `loop_override` parameter from the UI takes precedence over the setting in the file.
    """
    _require_motion_not_stop_latched("RUN_TRAJECTORY")
    utils.trajectory_state["should_stop"] = False # Reset stop flag for a new trajectory run
    utils.trajectory_state_set("stop_request_reason", None)
    print(f"[Pi Trajectory] Received RUN_TRAJECTORY for '{trajectory_name}' (Use Cache: {use_cache}, Loop Override: {loop_override})")

    if utils.trajectory_state.get("is_running"):
        raise RuntimeError("Cannot start trajectory, another task is running.")

    _reset_program_status(
        name=str(trajectory_name),
        active=True,
        state="planning",
        terminal_reason=None,
        failing_step_index=None,
        completed_step_count=0,
        completed_loop_count=0,
        loop_enabled=False,
        use_cache=bool(use_cache),
    )

    # Jog can stay active in the UI and keep streaming velocity packets.
    # Force-stop it before trajectory execution so no jog loop can contend
    # with weld path playback.
    if utils.trajectory_state.get("is_jogging"):
        print("[Pi Trajectory] Jog mode is active; stopping jog before trajectory run.")
        try:
            stop_active_jog_session(reason="motion-start")
        except Exception as e:
            print(f"[Pi Trajectory] WARNING: Failed to stop jog mode cleanly: {e}")
        if utils.trajectory_state.get("is_jogging"):
            _update_program_status(
                active=False,
                state="interrupted",
                terminal_reason="compatibility_interruption",
            )
            raise RuntimeError("Jog mode is still active; aborting trajectory run.")

    # --- 1. Load Trajectory Definition ---
    trajectory = _load_trajectory_by_name(trajectory_name)

    if trajectory is None:
        _update_program_status(
            active=False,
            state="faulted",
            terminal_reason="planner_failure",
        )
        raise RuntimeError(f"Trajectory '{trajectory_name}' not found.")

    moves = trajectory.get("moves", [])
    command_tokens = [
        str(move_cmd.get("command", "")).strip().lower()
        for move_cmd in moves
        if isinstance(move_cmd, dict)
    ]
    declared_move_step_count = sum(
        1 for command in command_tokens if command in {"move_relative", "move_absolute", "move_arc"}
    )
    declared_pause_step_count = sum(1 for command in command_tokens if command == "pause")
    declared_joint_move_step_count = sum(
        1 for command in command_tokens if command in {"home", "move"}
    )
    weld_meta = trajectory.get("weld") if isinstance(trajectory.get("weld"), dict) else None
    if weld_meta:
        utils.trajectory_state["current_weld_type"] = weld_meta.get("type")
    else:
        utils.trajectory_state["current_weld_type"] = None
    utils.trajectory_state["weld_active"] = False
    
    # Determine looping behavior: UI override > file setting > default false
    if loop_override is not None:
        should_loop = loop_override
    else:
        should_loop = trajectory.get("loop", False)

    _update_program_status(
        loop_enabled=bool(should_loop),
        step_count=len(moves),
        move_steps=declared_move_step_count,
        pause_steps=declared_pause_step_count,
        joint_move_steps=declared_joint_move_step_count,
    )

    # NEW: Parse orientation lock from trajectory file (unchanged from original)
    orientation_lock_euler = trajectory.get("orientation_euler_angles_deg")
    target_orientation_matrix = None
    if orientation_lock_euler:
        try:
            target_orientation_matrix = R.from_euler('xyz', orientation_lock_euler, degrees=True).as_matrix()
            print(f"[Pi Trajectory] Orientation will be locked to Euler (deg): {orientation_lock_euler}")
        except Exception as e:
            print(f"[Pi Trajectory] WARNING: Invalid Euler angles in trajectory file: {e}. Ignoring orientation lock.")

    print(f"[Pi Trajectory] Found '{trajectory_name}': {trajectory.get('description', 'No description')}")
    print(f"[Pi Trajectory] This trajectory will loop: {should_loop}")

    # --- 2. Get Planned Path (from Cache or by Planning) ---
    cache_file_path = os.path.join(utils.TRAJECTORY_CACHE_DIR, f"{trajectory_name}.json")
    planned_steps = None

    if use_cache:
        if os.path.exists(cache_file_path):
            print(f"[Pi Trajectory] Loading pre-computed path from cache: {cache_file_path}")
            try:
                with open(cache_file_path, 'r') as f:
                    planned_steps = json.load(f)
            except Exception as e:
                print(f"[Pi Trajectory] WARNING: Failed to load or parse cached file: {e}. Re-planning.")
                planned_steps = None # Ensure we re-plan on failure
        else:
            print(f"[Pi Trajectory] WARNING: Cache requested but not found. Planning trajectory...")

    # If we don't have a plan yet (either not requested or failed to load), plan it.
    if planned_steps is None:
        print("\n--- Starting Trajectory Planning Phase (Pre-computation) ---")
        planned_steps = []
        
        # Start planning from the robot's current known state, not a hardcoded home position.
        current_q = _get_best_available_joint_state()
        print(f"[Pi Trajectory] Planning will start from current state (rad): {np.round(current_q, 3)}")
        
        planning_succeeded = True
        planning_failure_step_index: int | None = None
        planning_failure_reason = "planner_failure"
        for i, move_cmd in enumerate(moves):
            if utils.trajectory_state["should_stop"]:
                print("[Pi Plan] Stop detected – aborting trajectory planning.")
                planning_succeeded = False
                planning_failure_step_index = i
                planning_failure_reason = str(
                    utils.trajectory_state_get("stop_request_reason", None) or "operator_abort"
                )
                break
            command = move_cmd.get("command")
            print(f"[Pi Plan] Planning Command {i+1}/{len(moves)}: {command}...")

            if command == "home":
                move_rotation_speed_deg_s = _extract_move_rotation_speed_deg_s(move_cmd)
                home_q_list = [0.0] * utils.NUM_LOGICAL_JOINTS
                joint_path, solved_q = _plan_joint_move_to_pose(
                    current_q,
                    target_q=home_q_list,
                    max_joint_deg_s=move_rotation_speed_deg_s,
                )
                if not joint_path or solved_q is None:
                    print("[Pi Trajectory] ERROR: Failed to plan home joint move. Aborting plan.")
                    planning_succeeded = False
                    planning_failure_step_index = i
                    break
                planned_steps.append(
                    {
                        "type": "move",
                        "path": joint_path,
                        "freq": _SAFE_JOINT_MOVE_FREQUENCY_HZ,
                        "serialization_command": "home",
                            "rotation_speed_deg_s": move_rotation_speed_deg_s,
                    }
                )
                current_q = np.array(solved_q, dtype=float)

            elif command == "move":
                t_start_plan = time.monotonic()
                target_pos = np.array(move_cmd.get("vector", [0, 0, 0]), dtype=float)
                move_rotation_speed_deg_s = _extract_move_rotation_speed_deg_s(move_cmd)
                move_orient_euler = move_cmd.get("orientation_euler_deg")
                per_move_orientation_matrix = None
                if move_orient_euler is not None:
                    try:
                        per_move_orientation_matrix = R.from_euler(
                            "xyz",
                            move_orient_euler,
                            degrees=True,
                        ).as_matrix()
                    except Exception as e:
                        print(f"[Pi Plan] WARNING: Invalid joint-move Euler orientation: {e}. Ignoring.")

                forced_orient = (
                    per_move_orientation_matrix
                    if per_move_orientation_matrix is not None
                    else target_orientation_matrix
                )
                joint_path, solved_q = _plan_joint_move_to_pose(
                    current_q,
                    target_pos=target_pos,
                    target_orientation=forced_orient,
                    max_joint_deg_s=move_rotation_speed_deg_s,
                )
                if joint_path and solved_q is not None:
                    t_end_plan = time.monotonic()
                    print(f"[Pi Plan] Joint-move planning complete. Took {(t_end_plan - t_start_plan) * 1000:.2f} ms")
                    planned_steps.append(
                        {
                            "type": "move",
                            "path": joint_path,
                            "freq": _SAFE_JOINT_MOVE_FREQUENCY_HZ,
                            "weld_active": False,
                            "serialization_command": "move",
                            "rotation_speed_deg_s": move_rotation_speed_deg_s,
                        }
                    )
                    current_q = np.array(solved_q, dtype=float)
                else:
                    planning_succeeded = False
                    planning_failure_step_index = i
                    break
            
            elif command == "move_relative":
                t_start_plan = time.monotonic()
                vector = np.array(move_cmd.get("vector", [0,0,0]))
                speed_mult = move_cmd.get("speed_multiplier", 1.0)
                _, move_velocity, move_acceleration = _resolve_profile_params_for_speed_multiplier(speed_mult)
                is_weld_move = bool(move_cmd.get("is_weld", False))

                # Per-move orientation override
                move_orient_euler = move_cmd.get("orientation_euler_deg")
                per_move_orientation_matrix = None
                if move_orient_euler is not None:
                    try:
                        per_move_orientation_matrix = R.from_euler('xyz', move_orient_euler, degrees=True).as_matrix()
                    except Exception as e:
                        print(f"[Pi Plan] WARNING: Invalid per-move Euler orientation: {e}. Ignoring.")

                start_pos = ik_solver.get_fk(current_q)
                if start_pos is None:
                    print(f"[Pi Trajectory] ERROR: Could not get start position for relative move. Aborting plan.")
                    planning_succeeded = False
                    planning_failure_step_index = i
                    break
                
                target_pos = start_pos + vector
                forced_orient = per_move_orientation_matrix if per_move_orientation_matrix is not None else target_orientation_matrix

                joint_path = trajectory_execution._plan_linear_move(
                    current_q, target_pos, move_velocity, move_acceleration, 100, True,
                    forced_orientation=forced_orient
                )
                
                if joint_path:
                    t_end_plan = time.monotonic()
                    print(f"[Pi Plan] Planning complete for move. Took {(t_end_plan - t_start_plan) * 1000:.2f} ms")
                    planned_steps.append(
                        {
                            'type': 'move',
                            'path': joint_path,
                            'freq': 100,
                            'weld_active': is_weld_move,
                        }
                    )
                    current_q = np.array(joint_path[-1])
                else:
                    planning_succeeded = False
                    planning_failure_step_index = i
                    break

            elif command == "move_absolute":
                t_start_plan = time.monotonic()
                target_pos = np.array(move_cmd.get("vector", [0,0,0]))
                move_linear_speed_m_s = _extract_move_linear_speed_m_s(move_cmd)
                move_linear_acceleration_m_s2 = _extract_move_linear_acceleration_m_s2(move_cmd)
                move_rotation_speed_deg_s = _extract_move_rotation_speed_deg_s(move_cmd)
                speed_mult = move_cmd.get("speed_multiplier", 1.0)
                _, move_velocity, move_acceleration = _resolve_profile_params_for_speed_multiplier(speed_mult)
                if move_linear_speed_m_s is not None or move_linear_acceleration_m_s2 is not None:
                    _, move_velocity, move_acceleration = _resolve_profile_params_for_linear_speed_m_s(
                        move_linear_speed_m_s,
                        move_linear_acceleration_m_s2,
                    )
                is_weld_move = bool(move_cmd.get("is_weld", False))

                move_orient_euler = move_cmd.get("orientation_euler_deg")
                per_move_orientation_matrix = None
                if move_orient_euler is not None:
                    try:
                        per_move_orientation_matrix = R.from_euler('xyz', move_orient_euler, degrees=True).as_matrix()
                    except Exception as e:
                        print(f"[Pi Plan] WARNING: Invalid per-move Euler orientation: {e}. Ignoring.")

                forced_orient = per_move_orientation_matrix if per_move_orientation_matrix is not None else target_orientation_matrix
                current_pos = ik_solver.get_fk(current_q.tolist())
                is_pure_rotation = (
                    current_pos is not None
                    and forced_orient is not None
                    and np.linalg.norm(np.array(current_pos, dtype=float) - target_pos) <= 1e-5
                    and move_rotation_speed_deg_s is not None
                )
                if is_pure_rotation:
                    joint_path = _plan_orientation_only_move(
                        current_q,
                        target_pos=target_pos,
                        target_orientation=forced_orient,
                        angular_speed_deg_s=move_rotation_speed_deg_s,
                        frequency_hz=100,
                    )
                else:
                    joint_path = trajectory_execution._plan_linear_move(
                        current_q, target_pos, move_velocity, move_acceleration, 100, True,
                        forced_orientation=forced_orient
                    )
                    
                if joint_path:
                    t_end_plan = time.monotonic()
                    print(f"[Pi Plan] Planning complete for move. Took {(t_end_plan - t_start_plan) * 1000:.2f} ms")
                    planned_steps.append(
                        {
                            'type': 'move',
                            'path': joint_path,
                            'freq': 100,
                            'weld_active': is_weld_move,
                            'linear_speed_mm_s': (
                                round(float(move_velocity * 1000.0), 3)
                                if move_linear_speed_m_s is not None
                                else None
                            ),
                            'linear_acceleration_mm_s2': (
                                round(float(move_acceleration * 1000.0), 3)
                                if move_linear_speed_m_s is not None or move_linear_acceleration_m_s2 is not None
                                else None
                            ),
                            'rotation_speed_deg_s': move_rotation_speed_deg_s,
                        }
                    )
                    current_q = np.array(joint_path[-1])
                else:
                    planning_succeeded = False
                    planning_failure_step_index = i
                    break
                    
            elif command == "move_arc":
                t_start_plan = time.monotonic()
                end_pos = np.array(move_cmd.get("end_point", [0,0,0]))
                center_pos = np.array(move_cmd.get("center_point", [0,0,0]))
                speed_mult = move_cmd.get("speed_multiplier", 1.0)
                _, move_velocity, move_acceleration = _resolve_profile_params_for_speed_multiplier(speed_mult)
                is_weld_move = bool(move_cmd.get("is_weld", False))
                
                start_pos = ik_solver.get_fk(current_q)
                if start_pos is None:
                    print(f"[Pi Trajectory] ERROR: Could not get start position for arc move. Aborting plan.")
                    planning_succeeded = False
                    planning_failure_step_index = i
                    break
                
                # 1. Generate the Cartesian path for the arc
                cartesian_path = trajectory_planner.generate_arc_trajectory(
                    start_pos, end_pos, center_pos, move_velocity, move_acceleration, 100)
                
                if not cartesian_path:
                    print(f"[Pi Trajectory] ERROR: Could not generate arc trajectory. Aborting plan.")
                    planning_succeeded = False
                    planning_failure_step_index = i
                    break

                # 2. Plan the joint space path from the Cartesian points
                joint_path = trajectory_execution._plan_high_fidelity_trajectory(
                    cartesian_points=cartesian_path,
                    start_q=current_q,
                    use_smoothing=True,
                    forced_orientation=target_orientation_matrix
                )

                if joint_path:
                    t_end_plan = time.monotonic()
                    print(f"[Pi Plan] Planning complete for move. Took {(t_end_plan - t_start_plan) * 1000:.2f} ms")
                    planned_steps.append(
                        {
                            'type': 'move',
                            'path': joint_path,
                            'freq': 100,
                            'weld_active': is_weld_move,
                        }
                    )
                    current_q = np.array(joint_path[-1])
                else:
                    planning_succeeded = False
                    planning_failure_step_index = i
                    break

            elif command == "pause":
                duration = move_cmd.get("duration", 1.0)
                planned_steps.append({'type': 'pause', 'duration': duration})
                
            else:
                print(f"[Pi Trajectory] WARNING: Unknown command '{command}' in trajectory. Skipping.")

        if not planning_succeeded:
            print("[Pi Trajectory] FATAL: Planning failed for one of the moves. Aborting execution.")
            utils.trajectory_state["weld_active"] = False
            utils.trajectory_state["current_weld_type"] = None
            terminal_state = "aborted" if planning_failure_reason == "operator_abort" else "faulted"
            _update_program_status(
                active=False,
                state=terminal_state,
                terminal_reason=planning_failure_reason,
                failing_step_index=planning_failure_step_index,
                current_step_index=None,
                current_step_type=None,
            )
            if planning_failure_step_index is None:
                raise RuntimeError("Trajectory planning failed before execution started.")
            raise RuntimeError(
                f"Trajectory planning failed at step {planning_failure_step_index + 1}."
            )

        # After successful planning, save the result to cache
        try:
            os.makedirs(utils.TRAJECTORY_CACHE_DIR, exist_ok=True)
            # We need to convert numpy arrays to lists before saving
            serializable_planned_steps = utils._convert_numpy_to_list(planned_steps)
            with open(cache_file_path, 'w') as f:
                json.dump(serializable_planned_steps, f, indent=2)
            print(f"[Pi Trajectory] Successfully saved planned path to cache: {cache_file_path}")
        except Exception as e:
            print(f"[Pi Trajectory] WARNING: Failed to save planned path to cache: {e}")
        
    # NEW: Now that we have planned_steps (from planning or cache), plan the reset move for looping if needed.
    # Explanation: This is the key fix. We calculate a smooth path from the last waypoint back to the first.
    # We use linear interpolation for position and SLERP for orientation. This prevents jerking back to the random start.
    # If planning fails, we disable looping to avoid the bug.
    # This works for both cached and non-cached plans.
    first_pose = None  # We'll set this if looping is possible
    if should_loop:
        if len(planned_steps) < 2:
            print("[Pi Trajectory] Trajectory has less than 2 steps, disabling loop.")
            should_loop = False
        else:
            # Get joints at end of first step (first waypoint)
            if planned_steps[0]['type'] != 'move':
                print("[Pi Trajectory] First step is not a move, disabling loop.")
                should_loop = False
            else:
                first_end_q = np.array(planned_steps[0]['path'][-1])
                # Get joints at end of last move step (last waypoint)
                last_end_q = None
                for step in reversed(planned_steps):
                    if step['type'] == 'move' or step['type'] == 'joint_move':
                        last_end_q = np.array(step['path'][-1])
                        break
                if last_end_q is None:
                    print("[Pi Trajectory] Could not find last move step, disabling loop.")
                    should_loop = False
                else:
                    first_pose = ik_solver.get_fk_matrix(first_end_q)
                    last_pose = ik_solver.get_fk_matrix(last_end_q)
                    if first_pose is None or last_pose is None:
                        print("[Pi Trajectory] Failed to calculate poses for loop reset, disabling loop.")
                        should_loop = False
                    else:
                        first_pos = first_pose[:3, 3]
                        first_orient = first_pose[:3, :3]
                        last_pos = last_pose[:3, 3]
                        last_orient = last_pose[:3, :3]
                        # Calculate duration based on distance and default velocity (for smooth speed)
                        dist = np.linalg.norm(first_pos - last_pos)
                        velocity = utils.DEFAULT_PROFILE_VELOCITY
                        duration_s = max(0.5, dist / velocity)
                        frequency_hz = 100  # Matches frequency used in other moves
                        num_steps = max(2, int(duration_s * frequency_hz))
                        # Linear interpolation for positions (straight line path)
                        t = np.linspace(0, 1, num_steps)
                        path_positions = [last_pos + ti * (first_pos - last_pos) for ti in t]
                        # SLERP for orientations (smooth rotation)
                        key_times = [0, 1]
                        key_rots = R.concatenate([R.from_matrix(last_orient), R.from_matrix(first_orient)])
                        slerp = Slerp(key_times, key_rots)
                        interp_rots = slerp(t)
                        orientation_matrices = [r.as_matrix() for r in interp_rots]
                        # Solve IK for the entire path (batch for efficiency)
                        joint_path_reset = ik_solver.solve_ik_path_batch(
                            path_points=path_positions,
                            initial_joint_angles=last_end_q,
                            target_orientations=orientation_matrices,
                        )
                        if joint_path_reset is None:
                            print("[Pi Trajectory] Failed to plan reset path from last to first, disabling loop to avoid jerk.")
                            should_loop = False
                        else:
                            reset_move = {
                                'type': 'move',
                                'path': joint_path_reset,
                                'freq': frequency_hz,
                                'weld_active': False,
                            }

    # --- 2. Execution Phase ---
    print("\n--- Trajectory Ready. Starting Execution in a background thread ---")
    
    move_step_count = sum(
        1
        for step in planned_steps
        if step.get("type") == "move"
        and str(step.get("serialization_command", "move_absolute")) not in {"home", "move"}
    )
    pause_step_count = sum(1 for step in planned_steps if step.get("type") == "pause")
    joint_move_step_count = sum(
        1
        for step in planned_steps
        if step.get("type") == "joint_move"
        or str(step.get("serialization_command", "")) in {"home", "move"}
    )
    rtcore_segment_execution = _backend_supports_rtcore_execution() and move_step_count > 0
    program_segment_execution_policy = (
        "rtcore_queued" if rtcore_segment_execution else "controller_open_loop"
    )
    execution_steps = planned_steps
    if not should_loop and weld_meta is None:
        collapsed_steps = _collapse_runtime_move_pause_steps(planned_steps)
        if collapsed_steps is not None:
            execution_steps = collapsed_steps
            program_segment_execution_policy = (
                "rtcore_compound_path" if rtcore_segment_execution else "controller_compound_path"
            )
            print(
                f"[Pi Trajectory] Collapsed {len(planned_steps)} planned steps into "
                f"{len(execution_steps[0].get('path', []))} streamed samples "
                f"with explicit hold segments for authored pauses."
            )

    utils.trajectory_state_update(
        is_running=True,
        should_stop=False,
        active_program_name=str(trajectory_name),
        active_program_loop_enabled=bool(should_loop),
        active_program_use_cache=bool(use_cache),
        active_program_step_count=len(planned_steps),
        active_program_move_steps=move_step_count,
        active_program_pause_steps=pause_step_count,
        active_program_joint_move_steps=joint_move_step_count,
        active_program_rtcore_segments=rtcore_segment_execution,
        active_program_segment_execution_policy=program_segment_execution_policy,
        active_program_step_index=None,
        active_program_step_type=None,
        active_program_loop_iteration=0,
    )
    _update_program_status(
        name=str(trajectory_name),
        active=True,
        state="accepted",
        terminal_reason=None,
        failing_step_index=None,
        completed_step_count=0,
        completed_loop_count=0,
        loop_enabled=bool(should_loop),
        use_cache=bool(use_cache),
        step_count=len(planned_steps),
        move_steps=move_step_count,
        pause_steps=pause_step_count,
        joint_move_steps=joint_move_step_count,
        rtcore_segments=bool(rtcore_segment_execution),
        segment_execution_policy=program_segment_execution_policy,
        current_step_index=None,
        current_step_type=None,
        loop_iteration=0,
    )
    try:
        utils.set_motion_state("EXECUTING")
    except Exception:
        pass

    # NEW: For looping, first move to the trajectory's start point from current position.
    # Explanation: We re-plan this move every time (even for cache) to avoid jerk if starting from a different position.
    # Uses similar interpolation as the reset for smoothness.
    if should_loop and len(planned_steps) > 0:
        print("[Pi Trajectory] Looping enabled. Moving to trajectory start point first.")
        # Re-read the live-preferred start state so cached logical joints do not
        # send the runtime-only move-to-start wrapper toward a stale pose.
        current_q = _get_best_available_joint_state()
        current_pose = ik_solver.get_fk_matrix(current_q)
        if current_pose is None or first_pose is None:
            print("[Pi Trajectory] Failed to get current or first pose, aborting trajectory.")
            _update_program_status(
                active=False,
                state="faulted",
                terminal_reason="planner_failure",
                failing_step_index=0 if len(planned_steps) > 0 else None,
            )
            utils.trajectory_state_update(
                is_running=False,
                weld_active=False,
                current_weld_type=None,
            )
            try:
                utils.set_motion_state("IDLE")
            except Exception:
                pass
            raise RuntimeError("Failed to get current or first pose for looping trajectory start.")
        first_pos = first_pose[:3, 3]
        first_orient = first_pose[:3, :3]
        current_pos = current_pose[:3, 3]
        current_orient = current_pose[:3, :3]
        dist = np.linalg.norm(first_pos - current_pos)
        velocity = utils.DEFAULT_PROFILE_VELOCITY
        duration_s = max(0.5, dist / velocity)
        frequency_hz = 100
        num_steps = max(2, int(duration_s * frequency_hz))
        t = np.linspace(0, 1, num_steps)
        path_positions = [current_pos + ti * (first_pos - current_pos) for ti in t]
        key_times = [0, 1]
        key_rots = R.concatenate([R.from_matrix(current_orient), R.from_matrix(first_orient)])
        slerp = Slerp(key_times, key_rots)
        interp_rots = slerp(t)
        orientation_matrices = [r.as_matrix() for r in interp_rots]
        joint_path_initial = ik_solver.solve_ik_path_batch(
            path_points=path_positions,
            initial_joint_angles=current_q,
            target_orientations=orientation_matrices,
        )
        if joint_path_initial is None:
            print("[Pi Trajectory] Failed to plan initial move to first waypoint, aborting trajectory.")
            _update_program_status(
                active=False,
                state="faulted",
                terminal_reason="planner_failure",
                failing_step_index=0 if len(planned_steps) > 0 else None,
            )
            utils.trajectory_state_update(
                is_running=False,
                weld_active=False,
                current_weld_type=None,
            )
            try:
                utils.set_motion_state("IDLE")
            except Exception:
                pass
            raise RuntimeError("Failed to plan initial move to first waypoint.")
        initial_freq = frequency_hz

        # Build the repeating loop body as one compound RTCore trajectory, just
        # like the non-loop path. Running each move as a separate RTCore upload
        # creates visible gaps and can preempt a segment that has not reached
        # its final point before the next segment starts.
        loop_steps = list(planned_steps[1:])
        loop_steps.append(reset_move)  # Uses the smooth reset we planned, not the old initial path
        collapsed_loop_steps = _collapse_runtime_move_pause_steps(loop_steps)
        if collapsed_loop_steps is not None:
            collapsed_loop_steps[0]["require_completion"] = True
            loop_steps = collapsed_loop_steps
            program_segment_execution_policy = (
                "rtcore_loop_compound_path" if rtcore_segment_execution else "controller_loop_compound_path"
            )
            print(
                f"[Pi Trajectory] Collapsed loop body into "
                f"{len(loop_steps[0].get('path', []))} streamed samples "
                "with strict completion before each loop repeat."
            )
        else:
            for step in loop_steps:
                if isinstance(step, dict) and str(step.get("type", "")).strip().lower() == "move":
                    step["require_completion"] = True
            print(
                "[Pi Trajectory] WARNING: Could not collapse loop body; "
                "each loop move will require strict completion before advancing."
            )

        # The move-to-start wrapper now runs inside the background loop thread
        # (`_looping_trajectory_executor_thread`) so RUN_TRAJECTORY can ACK
        # without holding the controller behind physical motion. The wrapper
        # is strict-completion: a fault, abort, or settle timeout marks the
        # program faulted and the loop body never starts.
        executor_thread = threading.Thread(
            target=_looping_trajectory_executor_thread,
            kwargs={
                "initial_joint_path": joint_path_initial,
                "initial_frequency_hz": initial_freq,
                "loop_steps": loop_steps,
                "loop_enabled": should_loop,
            },
            daemon=True,
        )
    else:
        # Non-looping case (unchanged)
        executor_thread = threading.Thread(
            target=trajectory_execution._trajectory_executor_thread,
            args=(execution_steps, should_loop)
        )
    utils.trajectory_state["thread"] = executor_thread
    executor_thread.start()
    
    print("[Pi Trajectory] Trajectory thread started. Main loop is responsive.")
    return _build_motion_execution_metadata(
        accepted=True,
        completion_scope="controller_program_thread",
        state="accepted",
        use_rtcore_status=False,
        extra={
            "trajectory_name": str(trajectory_name),
            "use_cache": bool(use_cache),
            "loop_enabled": bool(should_loop),
            "program_segment_execution_policy": program_segment_execution_policy,
            "program_rtcore_segments": bool(rtcore_segment_execution),
            "program_step_count": int(len(planned_steps)),
            "program_move_steps": int(move_step_count),
            "program_pause_steps": int(pause_step_count),
            "program_joint_move_steps": int(joint_move_step_count),
        },
    )

def handle_stop_command():
    """
    Stops any currently running motion by setting a global flag and sending
    an immediate brake command to the servos.
    """
    print("[Controller] Received STOP command. Halting all motion.")
    _clear_bounded_endpoint("stop")
    # Set the flag to stop any high-level trajectory loops
    utils.trajectory_state_update(
        should_stop=True,
        weld_active=False,
        motion_stop_latched=True,
        motion_stop_latched_at=time.monotonic(),
        motion_stop_latched_reason="operator_abort",
    )
    utils.trajectory_state_set("stop_request_reason", "operator_abort")
    program_status = _program_status_from_snapshot(utils.trajectory_state_snapshot())
    program_state = str(program_status.get("state") or "idle").strip().lower() if program_status else "idle"
    if program_status and (
        bool(program_status.get("active", False)) or program_state in _PROGRAM_ACTIVE_STATES
    ):
        _update_program_status(
            active=bool(program_status.get("active", False)),
            terminal_reason="operator_abort",
            state=str(program_status.get("state") or "executing"),
        )
    try:
        utils.set_motion_state("IDLE")
    except Exception:
        pass

    try:
        backend = backend_registry.get_active_backend()
    except Exception:
        backend = None
    try:
        stop_active_jog_session(reason="controller-stop")
    except Exception as exc:
        print(f"[Controller] WARNING: active jog session stop failed: {exc}")
    if backend is not None and hasattr(backend, "abort_trajectory"):
        try:
            backend.abort_trajectory()  # type: ignore[attr-defined]
        except Exception as e:
            print(f"[Controller] WARNING: RTCore trajectory abort failed: {e}")

    if backend is not None and callable(getattr(backend, "get_execution_status", None)):
        # EtherCAT RTCore owns its own stop/disarm semantics. Sending a legacy
        # "hold current position" write here would itself become a one-point
        # RTCore trajectory and can re-latch motion status during power-down.
        print("[Controller] RTCore-backed stop: skipping legacy brake write.")
        return

    # Also send an immediate brake command to the physical servos
    # by commanding them to their current position with zero speed.
    current_angles = servo_driver.get_current_arm_state_rad(verbose=False)
    if current_angles:
        print(f"[Controller] Sending immediate brake command to current position: {np.round(current_angles, 2)}")
        # Use speed 0 and max acceleration to act as a hard stop
        servo_driver.set_servo_positions(current_angles, 0, 100)
    else:
        print("[Controller] WARNING: Could not get current position to send brake command.")


def _power_transition_result(
    *,
    action: str,
    accepted: bool,
    code: str,
    message: str,
    motion_payload: dict[str, object] | None = None,
    extra: dict[str, object] | None = None,
) -> dict[str, object]:
    payload = dict(motion_payload) if isinstance(motion_payload, dict) else get_motion_execution_status()
    payload.update(
        {
            "accepted": bool(accepted),
            "power_action": str(action),
            "code": str(code),
            "message": str(message),
        }
    )
    if extra:
        payload.update(extra)
    return payload


def handle_move_to_position_absolute(x: float, y: float, z: float):
    """
    Handles the 'MOVE' command (legacy).
    Performs a simple, blocking, single-point IK move to an absolute position
    with no orientation constraint.
    """
    utils.trajectory_state["should_stop"] = False # Reset stop flag on new move
    print(f"[Pi IK] Received MOVE command: x={x}, y={y}, z={z}")

    # 1. Get current logical joint angles from our global state
    initial_angles = utils.current_logical_joint_angles_rad
    print(f"[Pi IK] Initial logical joint angles (rad): {np.round(initial_angles, 3)}")
    print(f"[Pi IK] Orientation is UNLOCKED for this move.")

    # 2. Set the target position from the absolute coordinates
    target_pos_xyz = np.array([x, y, z])
    print(f"[Pi IK] Target EE position (m):  {np.round(target_pos_xyz, 4)}")

    # 3. Use Inverse Kinematics (IK) to find the required joint angles for the target position.
    #    We provide the current angles as the starting point. Orientation is not constrained.
    new_logical_joint_angles = ik_solver.solve_ik(
        target_position=target_pos_xyz,
        initial_joint_angles=initial_angles
    )

    if new_logical_joint_angles is None:
        print("[Pi IK] ERROR: IK solver failed to find a solution.")
        return

    print(f"[Pi IK] IK Solution Found (rad): {np.round(new_logical_joint_angles, 3)}")
    print(f"[Pi IK] IK Solution Found (deg): {np.round(np.rad2deg(new_logical_joint_angles), 2)}")

    # 4. Command the servos to the new angles
    #    Using default speed and acceleration for now. This could be made adjustable.
    servo_driver.set_servo_positions(new_logical_joint_angles, utils.DEFAULT_SERVO_SPEED, utils.DEFAULT_SERVO_ACCELERATION_DEG_S2)
    print("[Pi IK] Sent new positions to servos.")

    # 5. Get and print the final position for verification
    final_pos_xyz = ik_solver.get_fk(new_logical_joint_angles)
    if final_pos_xyz is not None:
        print(f"[Pi IK] Verification -> Target: {np.round(target_pos_xyz, 4)}, Final FK: {np.round(final_pos_xyz, 4)}")
        print(f"[Pi IK] Distance from target: {np.linalg.norm(final_pos_xyz - target_pos_xyz):.6f} m")


def handle_get_position(sock: 'socket.socket', addr: tuple):
    """
    Handles the 'GET_POSITION' command.
    Calculates the current end-effector pose (position and joint angles) using FK 
    and sends it back to the requester.
    """
    print(f"[Pi] Received GET_POSITION from {addr}.")

    # Fetch the latest joint angles directly from the physical servos
    try:
        current_angles = servo_driver.get_current_arm_state_rad(verbose=False)
    except Exception as exc:
        message = str(exc).strip() or exc.__class__.__name__
        code = "CANONICAL_JOINT_TRUTH_UNAVAILABLE" if "Canonical joint truth unavailable" in message else "GET_POSITION_FAILED"
        print(f"[Pi] ERROR: Could not fetch current position joints: {message}")
        try:
            sock.sendto(f"ERROR,GET_POSITION,{code},{message.replace(',', ';')}".encode("utf-8"), addr)
        except Exception as send_exc:
            print(f"[Pi] Error sending GET_POSITION failure to {addr}: {send_exc}")
        return
    
    # Get the current full pose using Forward Kinematics (matrix)
    pose_mx = ik_solver.get_fk_matrix(current_angles)

    if pose_mx is not None:
        pos_xyz = pose_mx[:3, 3]
        euler_deg = R.from_matrix(pose_mx[:3, :3]).as_euler('xyz', degrees=True)
        angles_deg = np.rad2deg(current_angles)
        pos_str = ",".join(f"{float(value):.8f}" for value in pos_xyz)
        euler_str = ",".join(f"{float(value):.8f}" for value in euler_deg)
        angles_str = ",".join(f"{float(value):.8f}" for value in angles_deg)
        pos_log = ",".join(f"{float(value):.4f}" for value in pos_xyz)
        euler_log = ",".join(f"{float(value):.3f}" for value in euler_deg)
        
        print(f"[Pi] Sending pose: pos={pos_log} eulerXYZdeg={euler_log}")
        print(f"[Pi] Sending joint angles: {angles_str}")

        # Extended format: CURRENT_POSE,x,y,z,roll,pitch,yaw,<angles...>
        reply_msg = f"CURRENT_POSE,{pos_str},{euler_str},{angles_str}"
        
        try:
            sock.sendto(reply_msg.encode("utf-8"), addr)
        except Exception as e:
            print(f"[Pi] Error sending CURRENT_POSE to {addr}: {e}")
    else:
        print(f"[Pi] ERROR: Could not calculate current position (FK failed) for joints={current_angles}.")
        try:
            sock.sendto("ERROR,FK_FAILED".encode("utf-8"), addr)
        except Exception as e:
            print(f"[Pi] Error sending FK_FAILED error to {addr}: {e}")

def handle_get_orientation(sock: 'socket.socket', addr: tuple):
    """
    Handles the 'GET_ORIENTATION' command.
    Calculates the current end-effector orientation (as a rotation matrix) using FK
    and sends it back to the requester.
    """
    print(f"[Pi] Received GET_ORIENTATION from {addr}.")

    current_angles = servo_driver.get_current_arm_state_rad(verbose=False)

    # Get the current orientation using Forward Kinematics
    current_pose_matrix = ik_solver.get_fk_matrix(current_angles)

    if current_pose_matrix is not None:
        current_orientation = current_pose_matrix[:3, :3]
        
        # Round the orientation matrix for cleaner display
        orientation_rounded = np.round(current_orientation, 4)

        orientation_str = ",".join(map(str, orientation_rounded.flatten()))

        print(f"[Pi] Sending orientation: {orientation_str}")

        reply_msg = f"CURRENT_ORIENTATION,{orientation_str}"

        try:
            sock.sendto(reply_msg.encode("utf-8"), addr)
        except Exception as e:
            print(f"[Pi] Error sending CURRENT_ORIENTATION to {addr}: {e}")
    else:
        print(f"[Pi] ERROR: Could not calculate current orientation (FK failed) for joints={current_angles}.")
        try:
            sock.sendto("ERROR,FK_FAILED".encode("utf-8"), addr)
        except Exception as e:
            print(f"[Pi] Error sending FK_FAILED error to {addr}: {e}")

def handle_move_line(target_x: float, target_y: float, target_z: float, velocity: float, acceleration: float, closed_loop: bool = False) -> dict[str, object]:
    """Convenience wrapper that calls the main profiled move handler, defaulting to open-loop."""
    utils.trajectory_state["should_stop"] = False # Reset stop flag on new move
    return handle_move_profiled(
        target_x, target_y, target_z, velocity, acceleration,
        closed_loop=closed_loop,
        use_smoothing=True,
        diagnostics=False,
        command_name="MOVE_LINE",
    )

def handle_move_line_relative(dx: float, dy: float, dz: float, speed: float = 1.0, closed_loop: bool = False) -> dict[str, object]:
    """Convenience wrapper that calls the main profiled move handler, defaulting to open-loop."""
    utils.trajectory_state["should_stop"] = False # Reset stop flag on new move
    _require_target_axes_motion_ready(None)
    current_q = servo_driver.get_control_arm_state_rad(verbose=False)
    if current_q is None:
        raise RuntimeError("Cannot start relative move, failed to get current position.")
        
    start_pos = ik_solver.get_fk(current_q)
    if start_pos is None:
        raise RuntimeError("Cannot start relative move, failed to get start position.")

    current_pose_matrix = ik_solver.get_fk_matrix(current_q)
    current_orientation_euler_deg: list[float] | None = None
    if current_pose_matrix is not None:
        try:
            current_orientation_euler_deg = [
                float(value)
                for value in R.from_matrix(current_pose_matrix[:3, :3]).as_euler("xyz", degrees=True)
            ]
        except Exception:
            current_orientation_euler_deg = None
    _, target_velocity, target_acceleration = _resolve_profile_params_for_speed_multiplier(speed)
    target_pos = start_pos + np.array([dx, dy, dz])
    if os.environ.get("MINI_ARM_IK_LOG", "0") == "1" or bool(
        utils.trajectory_state.get("diagnostics_enabled", False)
    ):
        print(
            "[Controller] MOVE_LINE_RELATIVE detail:"
            f" frame=base/world"
            f" delta_m={[round(float(v), 6) for v in [dx, dy, dz]]}"
            f" start_pos_m={[round(float(v), 6) for v in np.asarray(start_pos, dtype=float).reshape(3)]}"
            f" target_pos_m={[round(float(v), 6) for v in np.asarray(target_pos, dtype=float).reshape(3)]}"
            f" current_joints_deg={[round(float(v), 6) for v in np.rad2deg(current_q)]}"
            f" current_orientation_euler_deg={None if current_orientation_euler_deg is None else [round(float(v), 6) for v in current_orientation_euler_deg]}"
            f" speed_multiplier={float(speed):.3f}"
            f" target_velocity={float(target_velocity):.4f}"
            f" target_acceleration={float(target_acceleration):.4f}"
            f" closed_loop={bool(closed_loop)}"
        )

    result = handle_move_profiled(
        target_pos[0], target_pos[1], target_pos[2],
        velocity=target_velocity,
        acceleration=target_acceleration,
        closed_loop=closed_loop,
        use_smoothing=True,
        diagnostics=False,
        command_name="MOVE_LINE_RELATIVE",
    )
    if isinstance(result, dict):
        result["cartesian_relative_debug"] = {
            "frame": "base/world",
            "delta_m": [float(dx), float(dy), float(dz)],
            "start_pos_m": [float(value) for value in np.asarray(start_pos, dtype=float).reshape(3)],
            "target_pos_m": [float(value) for value in np.asarray(target_pos, dtype=float).reshape(3)],
            "current_joints_deg": [float(value) for value in np.rad2deg(current_q)],
            "current_orientation_euler_deg": current_orientation_euler_deg,
            "speed_multiplier": float(speed),
            "target_velocity": float(target_velocity),
            "target_acceleration": float(target_acceleration),
            "closed_loop": bool(closed_loop),
        }
    return result


def handle_wait_for_idle(timeout_s: float = _WAIT_FOR_IDLE_DEFAULT_TIMEOUT_S) -> dict[str, object]:
    """
    Block until controller-thread and RTCore-backed motion activity has quiesced.
    """
    resolved_timeout_s = float(timeout_s)
    if not np.isfinite(resolved_timeout_s) or resolved_timeout_s <= 0.0:
        raise ValueError("WAIT_FOR_IDLE timeout must be a positive finite number of seconds.")

    deadline = time.monotonic() + resolved_timeout_s
    waited_for_motion = False
    last_active_payload: dict[str, object] | None = None

    while True:
        payload = get_motion_execution_status()
        if _motion_payload_is_active(payload):
            if not waited_for_motion:
                print("[Controller] Waiting for motion execution to become idle...")
            waited_for_motion = True
            last_active_payload = payload
            if time.monotonic() > deadline:
                print("[Controller] WARNING: Timed out waiting for motion execution to become idle.")
                return _finalize_wait_for_idle_payload(
                    payload,
                    timeout_s=resolved_timeout_s,
                    waited_for_motion=waited_for_motion,
                    last_active_payload=last_active_payload,
                    timed_out=True,
                )
            time.sleep(_WAIT_FOR_IDLE_POLL_INTERVAL_S)
            continue

        result = _finalize_wait_for_idle_payload(
            payload,
            timeout_s=resolved_timeout_s,
            waited_for_motion=waited_for_motion,
            last_active_payload=last_active_payload,
            timed_out=False,
        )
        print(f"[Controller] WAIT_FOR_IDLE finished with state: {result['state']}")
        return result


def _recent_bounded_endpoint_for_joints(
    target_joint_indices: Sequence[int],
    *,
    max_age_s: float = 1.0,
) -> list[float] | None:
    snapshot = utils.trajectory_state_get("last_bounded_endpoint", None)
    if not isinstance(snapshot, dict):
        return None
    sampled_at = snapshot.get("final_point_monotonic")
    arm_rad = snapshot.get("arm_rad")
    endpoint_targets = snapshot.get("target_joint_indices", [])
    if not isinstance(sampled_at, (int, float)) or not isinstance(arm_rad, list):
        return None
    if time.monotonic() - float(sampled_at) > float(max_age_s):
        return None
    if isinstance(endpoint_targets, list) and endpoint_targets:
        requested = {int(value) for value in target_joint_indices}
        previous = {int(value) for value in endpoint_targets}
        if requested and previous and not requested.issubset(previous):
            return None
    return [float(value) for value in arm_rad]


def _bounded_joint_baseline(target_joint_indices: Sequence[int]) -> tuple[list[float], str]:
    try:
        current_q = servo_driver.get_control_arm_state_rad(verbose=False)
        if current_q is not None:
            return list(current_q), "control_feedback"
    except RuntimeError as exc:
        reason = str(exc)
        if "Control feedback unavailable" not in reason and "Canonical joint truth unavailable" not in reason:
            raise

    chained = _recent_bounded_endpoint_for_joints(target_joint_indices)
    if chained is not None:
        return chained, "recent_bounded_endpoint"

    raise RuntimeError("CONTROL_FEEDBACK_UNAVAILABLE: no live or recent bounded endpoint baseline")


def handle_apply_joint_setpoint(
    arm_angles_rad: list[float],
    *,
    gripper_rad: float | None = None,
    speed: int | None = None,
    acceleration: float | None = None,
    max_motor_rpm: float | None = None,
    target_joint_indices: Sequence[int] | None = None,
    canonical_wrap_target: bool = False,
) -> dict[str, object]:
    """
    Send a direct joint/gripper setpoint and return once the backend accepts it.

    This is intentionally an acknowledgement path, not a trajectory-completion path.
    """
    resolved_speed = _coerce_direct_setpoint_speed(
        speed if speed is not None else utils.DEFAULT_SERVO_SPEED
    )
    resolved_acceleration = _coerce_direct_setpoint_acceleration(
        acceleration
        if acceleration is not None
        else utils.DEFAULT_SERVO_ACCELERATION_DEG_S2
    )
    normalized_target_joint_indices: list[int] | None = None
    if target_joint_indices is not None:
        normalized_target_joint_indices = []
        for raw_index in target_joint_indices:
            joint_idx = int(raw_index)
            if joint_idx < 0 or joint_idx >= len(arm_angles_rad):
                raise ValueError(f"target_joint_indices contains out-of-range joint index {joint_idx}")
            if joint_idx not in normalized_target_joint_indices:
                normalized_target_joint_indices.append(joint_idx)
        if not normalized_target_joint_indices:
            normalized_target_joint_indices = None
    _begin_non_program_motion()
    if max_motor_rpm is not None and float(max_motor_rpm) > 0.0:
        _require_target_axes_motion_ready(normalized_target_joint_indices)
        if bool(utils.trajectory_state_get("is_running", False)):
            raise RuntimeError("Cannot start bounded joint setpoint, another task is running.")
        if canonical_wrap_target:
            # Home/unwind commands must plan from the operator-facing canonical
            # frame. The relaxed control-feedback path intentionally collapses
            # drive-native wraps for safety/control baselining; using it here
            # would make "home to zero" choose the nearest equivalent zero and
            # leave cable twists in place after a full-turn seam incident.
            current_q = servo_driver.get_current_arm_state_rad(verbose=False)
        else:
            current_q = servo_driver.get_control_arm_state_rad(verbose=False)
        if current_q is None:
            raise RuntimeError("Failed to read current joint state for bounded joint setpoint.")
        joint_path, duration_s = _build_bounded_joint_path(
            current_q[: len(arm_angles_rad)],
            arm_angles_rad,
            max_motor_rpm=float(max_motor_rpm),
        )
        try:
            current_deg = np.round(np.rad2deg(current_q[: len(arm_angles_rad)]), 3).tolist()
            target_deg = np.round(np.rad2deg(arm_angles_rad), 3).tolist()
            print(
                "[Controller] APPLY_JOINT_SETPOINT bounded move:"
                f" current_deg={current_deg}"
                f" target_deg={target_deg}"
                f" max_motor_rpm={float(max_motor_rpm):.1f}"
                f" canonical_wrap_target={bool(canonical_wrap_target)}"
                f" duration_s={float(duration_s):.3f}"
                f" points={len(joint_path)}"
            )
        except Exception:
            pass

        executor_thread = threading.Thread(
            target=trajectory_execution._open_loop_executor_thread,
            kwargs={
                "joint_path": joint_path,
                "frequency": _SAFE_JOINT_MOVE_FREQUENCY_HZ,
                "diagnostics": False,
                "target_joint_indices": normalized_target_joint_indices,
            },
            daemon=True,
        )
        utils.trajectory_state_update(
            thread=executor_thread,
            is_running=True,
            should_stop=False,
            rtcore_settle_traj_id=None,
        )
        try:
            utils.set_motion_state("EXECUTING")
        except Exception:
            pass
        executor_thread.start()

        if gripper_rad is not None:
            handle_set_gripper_state(np.rad2deg(gripper_rad), resolved_speed, resolved_acceleration)
        return _build_motion_execution_metadata(
            accepted=True,
            completion_scope=_default_completion_scope(
                closed_loop=False,
                prefer_rtcore=True,
            ),
            state="accepted",
            use_rtcore_status=True,
            extra={
                "speed": resolved_speed,
                "acceleration": resolved_acceleration,
                "max_motor_rpm": float(max_motor_rpm),
                "duration_s": float(duration_s),
                "frequency_hz": _SAFE_JOINT_MOVE_FREQUENCY_HZ,
                "target_joint_indices": list(normalized_target_joint_indices)
                if normalized_target_joint_indices is not None
                else None,
                "canonical_wrap_target": bool(canonical_wrap_target),
            },
        )

    servo_driver.set_servo_positions(arm_angles_rad, resolved_speed, resolved_acceleration)
    if gripper_rad is not None:
        handle_set_gripper_state(np.rad2deg(gripper_rad), resolved_speed, resolved_acceleration)
    return _build_motion_execution_metadata(
        accepted=True,
        completion_scope=(
            "rtcore_execution" if _backend_supports_rtcore_execution() else "controller_ack"
        ),
        state="accepted",
        use_rtcore_status=_backend_supports_rtcore_execution(),
        extra={
            "speed": resolved_speed,
            "acceleration": resolved_acceleration,
        },
    )


def handle_apply_joint_delta(
    *,
    joint: int,
    delta_deg: float,
    max_motor_rpm: float,
    wait_for_idle: bool = False,
) -> dict[str, object]:
    _require_motion_not_stop_latched("APPLY_JOINT_DELTA")
    try:
        num_logical_joints = int(utils.NUM_LOGICAL_JOINTS)
    except Exception:
        try:
            num_logical_joints = len(list(utils.current_logical_joint_angles_rad))
        except Exception:
            num_logical_joints = 0
    if num_logical_joints <= 0:
        num_logical_joints = 6
    joint_i = int(joint) - 1
    if joint_i < 0 or joint_i >= num_logical_joints:
        raise ValueError(f"joint must be in 1..{num_logical_joints}")
    target_joint_indices = [joint_i]
    _require_target_axes_motion_ready(target_joint_indices)
    if bool(utils.trajectory_state_get("is_running", False)):
        raise RuntimeError("Cannot start bounded joint delta, another task is running.")

    baseline_q, baseline_source = _bounded_joint_baseline(target_joint_indices)
    target_q = list(baseline_q[:num_logical_joints])
    if len(target_q) < num_logical_joints:
        raise RuntimeError("CANONICAL_JOINT_TRUTH_UNAVAILABLE: baseline joint vector is incomplete")
    target_q[joint_i] = float(target_q[joint_i]) + float(np.deg2rad(delta_deg))

    payload = handle_apply_joint_setpoint(
        target_q,
        max_motor_rpm=float(max_motor_rpm),
        target_joint_indices=target_joint_indices,
    )
    payload["baseline_source"] = baseline_source
    payload["delta_deg"] = float(delta_deg)
    payload["joint"] = int(joint)
    payload["wait_for_idle_requested"] = bool(wait_for_idle)
    if wait_for_idle:
        duration_s = float(payload.get("duration_s", 0.0) or 0.0)
        wait_payload = handle_wait_for_idle(timeout_s=max(1.0, duration_s + 0.75))
        payload["wait_result"] = wait_payload
        payload["waited_for_idle"] = not bool(wait_payload.get("timed_out", False))
        payload["state"] = wait_payload.get("state", payload.get("state", "accepted"))
    else:
        payload["waited_for_idle"] = False
    return payload


def handle_safe_power_down(wait_for_idle: bool = False) -> dict[str, object]:
    """
    Best-effort transition the active actuator backend into a non-active state.

    For EtherCAT RTCore this is intended to de-energize the axes without relying
    on the controller process exiting first.
    """
    print(
        "[Controller] Received SAFE_POWER_DOWN command."
        " drive_power_action=safe_power_down"
        f" wait_for_idle={bool(wait_for_idle)}"
    )
    try:
        handle_stop_command()
    except Exception as e:
        print(f"[Controller] WARNING: STOP during safe power-down failed: {e}")

    if wait_for_idle:
        try:
            handle_wait_for_idle()
        except Exception as e:
            print(f"[Controller] WARNING: WAIT_FOR_IDLE during safe power-down failed: {e}")

    try:
        backend = backend_registry.get_active_backend()
    except Exception as e:
        msg = f"No active backend: {e}"
        print(f"[Controller] WARNING: {msg}")
        return _power_transition_result(
            action="power_down",
            accepted=False,
            code="POWER_DOWN_UNAVAILABLE",
            message=msg,
            extra={"waited_for_idle": bool(wait_for_idle)},
        )

    safe_power_down = getattr(backend, "safe_power_down", None)
    if not callable(safe_power_down):
        print("[Controller] Active backend does not implement safe_power_down().")
        return _power_transition_result(
            action="power_down",
            accepted=True,
            code="BACKEND_NOOP",
            message="Active backend does not implement safe_power_down().",
            extra={"waited_for_idle": bool(wait_for_idle), "backend_handled": False},
        )

    try:
        print("[Controller] Dispatching backend safe_power_down drive_power_action=safe_power_down")
        try:
            handled = bool(
                safe_power_down(
                    wait_for_idle=wait_for_idle,
                    timeout_s=_POWER_TRANSITION_DEFAULT_TIMEOUT_S,
                    quick_stop=True,
                )
            )
        except TypeError:
            try:
                handled = bool(
                    safe_power_down(
                        wait_for_idle=wait_for_idle,
                        timeout_s=_POWER_TRANSITION_DEFAULT_TIMEOUT_S,
                    )
                )
            except TypeError:
                handled = bool(safe_power_down())
        print("[Controller] Backend safe power-down command sent.")
        return _power_transition_result(
            action="power_down",
            accepted=True,
            code="POWER_DOWN_SENT" if handled else "BACKEND_NOOP",
            message=(
                "Drive power-down requested with safe stop/disarm sequencing."
                if handled
                else "Backend reported no special power-down action."
            ),
            extra={"waited_for_idle": bool(wait_for_idle), "backend_handled": bool(handled)},
        )
    except Exception as e:
        msg = f"Power-down failed: {e}"
        print(f"[Controller] WARNING: {msg}")
        return _power_transition_result(
            action="power_down",
            accepted=False,
            code="POWER_DOWN_FAILED",
            message=msg,
            extra={"waited_for_idle": bool(wait_for_idle)},
        )


def handle_safe_power_up() -> dict[str, object]:
    """
    Best-effort transition the active actuator backend into an armed/energized state.
    """
    print("[Controller] Received SAFE_POWER_UP command.")
    _clear_motion_stop_latch()
    _clear_bounded_endpoint("safe_power_up")
    try:
        backend = backend_registry.get_active_backend()
    except Exception as e:
        msg = f"No active backend: {e}"
        print(f"[Controller] WARNING: {msg}")
        return _power_transition_result(
            action="power_up",
            accepted=False,
            code="POWER_UP_UNAVAILABLE",
            message=msg,
        )

    safe_power_up = getattr(backend, "safe_power_up", None)
    if not callable(safe_power_up):
        print("[Controller] Active backend does not implement safe_power_up().")
        return _power_transition_result(
            action="power_up",
            accepted=True,
            code="BACKEND_NOOP",
            message="Active backend does not implement safe_power_up().",
            extra={"backend_handled": False},
        )

    motion_payload = get_motion_execution_status()
    if not bool(motion_payload.get("safe_for_power_transition", False)):
        return _power_transition_result(
            action="power_up",
            accepted=False,
            code="POWER_UP_BLOCKED",
            message="Drive power-up blocked until motion is neutral, fault-free, and synchronized.",
            motion_payload=motion_payload,
        )

    synchronize_targets = getattr(backend, "synchronize_command_targets_to_feedback", None)
    if callable(synchronize_targets):
        try:
            sync_payload = synchronize_targets()
        except Exception as exc:
            sync_payload = {
                "synchronized": False,
                "reason": f"sync_failed:{exc}",
                "joint_positions_rad": [],
            }
        if not bool(sync_payload.get("synchronized", False)):
            sync_message = (
                "Drive power-up blocked because live feedback could not be synchronized to hold targets."
            )
            motion_payload = get_motion_execution_status()
            blocker_details = list(motion_payload.get("power_transition_blocker_details", []))
            blocker_details.append(
                {
                    "code": "not_synchronized",
                    "message": sync_message,
                    "reason": str(sync_payload.get("reason", "unknown")),
                }
            )
            blocker_codes = list(motion_payload.get("power_transition_blockers", [])) + ["not_synchronized"]
            motion_payload["safe_for_power_transition"] = False
            motion_payload["power_transition_blockers"] = blocker_codes
            motion_payload["power_transition_blocker_details"] = blocker_details
            execution_payload = motion_payload.get("execution")
            if isinstance(execution_payload, dict):
                execution_payload["safe_for_power_transition"] = False
                execution_payload["power_transition_blockers"] = blocker_codes
                execution_payload["power_transition_blocker_details"] = blocker_details
            return _power_transition_result(
                action="power_up",
                accepted=False,
                code="POWER_UP_BLOCKED",
                message=sync_message,
                motion_payload=motion_payload,
            )

    try:
        handled = bool(safe_power_up())
        post_payload = get_motion_execution_status()
        if handled:
            print("[Controller] Backend safe power-up command sent.")
            return _power_transition_result(
                action="power_up",
                accepted=True,
                code="POWER_UP_SENT",
                message="Drive power-up requested after neutral-state verification.",
                motion_payload=post_payload,
                extra={"backend_handled": True},
            )
        print("[Controller] Backend reported no special power-up action.")
        return _power_transition_result(
            action="power_up",
            accepted=True,
            code="BACKEND_NOOP",
            message="Backend reported no special power-up action.",
            motion_payload=post_payload,
            extra={"backend_handled": False},
        )
    except Exception as e:
        msg = f"Power-up failed: {e}"
        print(f"[Controller] WARNING: {msg}")
        return _power_transition_result(
            action="power_up",
            accepted=False,
            code="POWER_UP_FAILED",
            message=msg,
        )


def handle_reset_faults(logical_joint_index: int | None = None) -> dict[str, object]:
    print(
        "[Controller] Received RESET_FAULTS command."
        + (f" target_joint={logical_joint_index + 1}" if logical_joint_index is not None else "")
    )
    try:
        backend = backend_registry.get_active_backend()
    except Exception as exc:
        msg = f"No active backend: {exc}"
        print(f"[Controller] WARNING: {msg}")
        return _power_transition_result(
            action="reset_faults",
            accepted=False,
            code="RESET_FAULTS_UNAVAILABLE",
            message=msg,
            extra={"joint": None if logical_joint_index is None else logical_joint_index + 1},
        )

    try:
        handle_stop_command()
    except Exception as exc:
        print(f"[Controller] WARNING: STOP during reset-faults failed: {exc}")

    try:
        handle_wait_for_idle(timeout_s=_POWER_TRANSITION_DEFAULT_TIMEOUT_S)
    except Exception as exc:
        print(f"[Controller] WARNING: WAIT_FOR_IDLE during reset-faults failed: {exc}")

    try:
        handled = bool(backend.reset_faults(logical_joint_index=logical_joint_index))
    except Exception as exc:
        msg = f"Fault reset failed: {exc}"
        print(f"[Controller] WARNING: {msg}")
        return _power_transition_result(
            action="reset_faults",
            accepted=False,
            code="RESET_FAULTS_FAILED",
            message=msg,
            extra={"joint": None if logical_joint_index is None else logical_joint_index + 1},
        )

    if not handled:
        return _power_transition_result(
            action="reset_faults",
            accepted=False,
            code="RESET_FAULTS_FAILED",
            message="Active backend rejected the drive fault reset request.",
            extra={"joint": None if logical_joint_index is None else logical_joint_index + 1},
        )

    return _power_transition_result(
        action="reset_faults",
        accepted=True,
        code="RESET_FAULTS_SENT",
        message="Drive fault reset requested. Drives remain disarmed until an explicit safe power-up.",
        extra={
            "joint": None if logical_joint_index is None else logical_joint_index + 1,
            "disarmed_after_reset": True,
        },
    )


def handle_reset_encoder_data(logical_joint_index: int | None = None) -> dict[str, object]:
    print(
        "[Controller] Received RESET_ENCODER_DATA command."
        + (f" target_joint={logical_joint_index + 1}" if logical_joint_index is not None else "")
    )
    try:
        backend = backend_registry.get_active_backend()
    except Exception as exc:
        msg = f"No active backend: {exc}"
        print(f"[Controller] WARNING: {msg}")
        return _power_transition_result(
            action="reset_encoder_data",
            accepted=False,
            code="RESET_ENCODER_DATA_UNAVAILABLE",
            message=msg,
            extra={"joint": None if logical_joint_index is None else logical_joint_index + 1},
        )

    try:
        handle_stop_command()
    except Exception as exc:
        print(f"[Controller] WARNING: STOP during encoder-data reset failed: {exc}")

    try:
        handle_wait_for_idle(timeout_s=_POWER_TRANSITION_DEFAULT_TIMEOUT_S)
    except Exception as exc:
        print(f"[Controller] WARNING: WAIT_FOR_IDLE during encoder-data reset failed: {exc}")

    try:
        handled = bool(backend.reset_encoder_data(logical_joint_index=logical_joint_index))
    except Exception as exc:
        msg = f"Encoder data reset failed: {exc}"
        print(f"[Controller] WARNING: {msg}")
        return _power_transition_result(
            action="reset_encoder_data",
            accepted=False,
            code="RESET_ENCODER_DATA_FAILED",
            message=msg,
            extra={"joint": None if logical_joint_index is None else logical_joint_index + 1},
        )

    if not handled:
        return _power_transition_result(
            action="reset_encoder_data",
            accepted=False,
            code="RESET_ENCODER_DATA_FAILED",
            message="Active backend rejected the encoder-data reset request.",
            extra={"joint": None if logical_joint_index is None else logical_joint_index + 1},
        )

    return _power_transition_result(
        action="reset_encoder_data",
        accepted=True,
        code="RESET_ENCODER_DATA_SENT",
        message=(
            "Encoder data reset requested. Drives remain disarmed; perform a safe repower and "
            "native re-home before trusting absolute multi-turn position."
        ),
        extra={
            "joint": None if logical_joint_index is None else logical_joint_index + 1,
            "disarmed_after_reset": True,
            "requires_power_cycle": True,
            "requires_rehome": True,
        },
    )


# -----------------------------------------------------------------------------
# PID Tuning API
# -----------------------------------------------------------------------------

def handle_tune_pid_joint(joint_index: int, amplitude_deg: float = 5.0, frequency_hz: int = 100, duration_s: float = 3.0, move_to_zero_first: bool = True):
    """Runs the internal PID tuner for a single logical joint (blocking)."""
    if utils.trajectory_state.get("is_running"):
        print("[PID Tune] ERROR: Motion already active. Stop current move before tuning.")
        return
    try:
        print(f"[PID Tune] Starting tuning for joint J{joint_index+1}...")
        pid_tuner.tune_internal_pid_for_joint(
            logical_joint_index=joint_index,
            amplitude_deg=amplitude_deg,
            frequency_hz=frequency_hz,
            duration_s=duration_s,
            move_to_zero_first=move_to_zero_first,
        )
        print(f"[PID Tune] Tuning complete for joint J{joint_index+1}.")
    except Exception as e:
        print(f"[PID Tune] ERROR: {e}")


def handle_tune_pid_all(amplitude_deg: float = 5.0, frequency_hz: int = 50, duration_s: float = 3.0, move_to_zero_first_each: bool = True):
    """Tunes all logical joints sequentially (blocking)."""
    for j in range(utils.NUM_LOGICAL_JOINTS):
        if utils.trajectory_state.get("is_running"):
            print("[PID Tune] Motion became active mid-run; aborting all-joint tuning.")
            return
        handle_tune_pid_joint(j, amplitude_deg, frequency_hz, duration_s, move_to_zero_first_each)

# -----------------------------------------------------------------------------
# Gripper Control
# -----------------------------------------------------------------------------
def handle_set_gripper_state(angle_deg: float, speed: int = 50, accel: int = 0):
    """
    Handles the 'SET_GRIPPER' command. Commands the gripper to a specific angle.
    
    Args:
        angle_deg (float): The target angle for the gripper in degrees.
        speed (int): The speed for the movement.
        accel (int): The acceleration for the movement.
    """
    if not utils.gripper_present:
        print("[Controller] Cannot set gripper state: Gripper is not present.")
        return

    print(f"[Controller] Setting gripper to {angle_deg} degrees.")
    
    # Convert degrees to radians for internal use and validation
    angle_rad = np.deg2rad(angle_deg)

    # Validate against gripper limits
    min_rad, max_rad = utils.GRIPPER_LIMITS_RAD
    if not (min_rad <= angle_rad <= max_rad):
        print(f"[Controller] ERROR: Gripper angle {angle_deg}° is outside limits "
              f"({np.rad2deg(min_rad):.1f}° to {np.rad2deg(max_rad):.1f}°).")
        return

    # Use the existing single-servo write function
    servo_driver.set_single_servo_position_rads(
        servo_id=utils.SERVO_ID_GRIPPER,
        position_rad=angle_rad,
        speed=speed,
        accel=accel
    )
    # Update global state
    utils.current_gripper_angle_rad = angle_rad


def handle_get_gripper_state(sock: 'socket.socket', addr: tuple):
    """
    Handles the 'GET_GRIPPER_STATE' command. Reads the gripper's current
    angle and sends it back to the client.
    """
    if not utils.gripper_present:
        print("[Controller] Cannot get gripper state: Gripper is not present.")
        try:
            sock.sendto("ERROR,GRIPPER_NOT_PRESENT".encode("utf-8"), addr)
        except Exception as e:
            print(f"[Controller] Error sending GRIPPER_NOT_PRESENT error to {addr}: {e}")
        return

    print(f"[Controller] Received GET_GRIPPER_STATE from {addr}.")
    
    # Read the raw position from the servo (uses backend if available)
    raw_pos = servo_driver.read_single_servo_position(utils.SERVO_ID_GRIPPER)
    
    if raw_pos is not None:
        # Convert raw position to angle in degrees
        # This requires finding the correct config index for the gripper
        try:
            gripper_config_index = utils.SERVO_IDS.index(utils.SERVO_ID_GRIPPER)
            angle_rad = servo_driver.raw_to_angle_rad(raw_pos, gripper_config_index)
            angle_deg = np.rad2deg(angle_rad)
            
            # Update global state as well
            utils.current_gripper_angle_rad = angle_rad

            reply = f"GRIPPER_STATE,{angle_deg:.2f},{raw_pos}"
            print(f"[Controller] Sending gripper state: {reply}")
            sock.sendto(reply.encode("utf-8"), addr)
        except ValueError:
            print("[Controller] ERROR: Gripper servo ID not found in SERVO_IDS list.")
            sock.sendto("ERROR,GRIPPER_ID_NOT_CONFIGURED".encode("utf-8"), addr)
        except Exception as e:
            print(f"[Controller] ERROR: Could not convert raw position to angle: {e}")
            sock.sendto(f"ERROR,CONVERSION_FAILED".encode("utf-8"), addr)
    else:
        print("[Controller] ERROR: Failed to read gripper position.")
        sock.sendto("ERROR,READ_FAILED".encode("utf-8"), addr)


# -----------------------------------------------------------------------------
# Real-time Cartesian Jogging
# -----------------------------------------------------------------------------

JOG_CONTROL_FREQUENCY_HZ = 50
JOG_CONTROLLER_LEASE_TIMEOUT_S = 0.4
JOG_BACKEND_LEASE_TIMEOUT_S = 0.2
JOG_VELOCITY_TIMEOUT_S = JOG_CONTROLLER_LEASE_TIMEOUT_S
MAX_JOG_LINEAR_M_S = 0.2      # Safety cap per-axis
MAX_JOG_ANGULAR_DEG_S = 180.0 # Safety cap per-axis
JOG_LINEAR_ACCEL_M_S2 = float(os.environ.get("GRADIENT_JOG_LINEAR_ACCEL_M_S2", "0.75"))
JOG_ANGULAR_ACCEL_DEG_S2 = float(os.environ.get("GRADIENT_JOG_ANGULAR_ACCEL_DEG_S2", "540.0"))
JOG_UI_RELEASE_ZERO_SETTLE_S = float(os.environ.get("GRADIENT_JOG_UI_RELEASE_ZERO_SETTLE_S", "0.06"))
JOG_JACOBIAN_SIGMA_MIN_THRESHOLD = float(os.environ.get("GRADIENT_JOG_JACOBIAN_SIGMA_MIN_THRESHOLD", "0.05"))
JOG_JACOBIAN_DAMPING_LAMBDA_MAX = float(os.environ.get("GRADIENT_JOG_JACOBIAN_DAMPING_LAMBDA_MAX", "0.1"))
JOG_MAX_COMMAND_DRIFT_RAD = float(os.environ.get("GRADIENT_JOG_MAX_COMMAND_DRIFT_RAD", "0.15"))
JOG_USE_JACOBIAN = os.environ.get("GRADIENT_JOG_USE_JACOBIAN", "1") == "1"
_JOG_DIAGNOSTIC_AB_COMPARE = os.environ.get("GRADIENT_JOG_DIAGNOSTIC_AB_COMPARE", "0") == "1"
MAX_GRIPPER_JOG_DEG_S = 90.0 # Safety cap for gripper rotation rate
_JOG_SESSION_MANAGER = JogSessionManager()
_JOG_NEAR_SINGULARITY_ACTIVE = False
_JOG_NEAR_SINGULARITY_CLEAR_TICKS = 0


def _jog_execution_policy(jog_backend) -> str:
    return "joint_velocity_lease" if jog_backend is not None else "controller_cartesian_loop"


def set_jog_diagnostic_ab_compare(enabled: bool) -> None:
    global _JOG_DIAGNOSTIC_AB_COMPARE
    _JOG_DIAGNOSTIC_AB_COMPARE = bool(enabled)
    print(f"[Jog] Diagnostic IK-vs-Jacobian A/B compare set to {_JOG_DIAGNOSTIC_AB_COMPARE}")


def _emit_jog_singularity_advisory(diag: dict[str, object]) -> None:
    telemetry_alerts.push_alert(
        level="info",
        kind="JOG_NEAR_SINGULARITY",
        message=(
            "Cartesian jog is near a kinematic singularity; motion is "
            "attenuated in the singular direction (DLS damping active)."
        ),
        details={
            "sigma_min": diag.get("jacobian_sigma_min"),
            "condition_number": diag.get("jacobian_condition_number"),
            "damping_lambda_squared": diag.get("jacobian_damping_lambda_squared"),
        },
        key="JOG_NEAR_SINGULARITY",
    )


def _update_jog_singularity_advisory(diag: dict[str, object] | None) -> None:
    global _JOG_NEAR_SINGULARITY_ACTIVE, _JOG_NEAR_SINGULARITY_CLEAR_TICKS
    if not isinstance(diag, dict):
        return
    near = bool(diag.get("jacobian_near_singular", False))
    if near:
        _JOG_NEAR_SINGULARITY_CLEAR_TICKS = 0
        if not _JOG_NEAR_SINGULARITY_ACTIVE:
            _emit_jog_singularity_advisory(diag)
            _JOG_NEAR_SINGULARITY_ACTIVE = True
        return
    if _JOG_NEAR_SINGULARITY_ACTIVE:
        _JOG_NEAR_SINGULARITY_CLEAR_TICKS += 1
        if _JOG_NEAR_SINGULARITY_CLEAR_TICKS >= 5:
            _JOG_NEAR_SINGULARITY_ACTIVE = False
            _JOG_NEAR_SINGULARITY_CLEAR_TICKS = 0


def _slew_limited_vector(current: np.ndarray, target: np.ndarray, max_delta: float) -> np.ndarray:
    if max_delta <= 0.0:
        return np.asarray(target, dtype=float)
    current_arr = np.asarray(current, dtype=float)
    target_arr = np.asarray(target, dtype=float)
    delta = np.clip(target_arr - current_arr, -float(max_delta), float(max_delta))
    return current_arr + delta


def _should_gracefully_release_jog(reason: str) -> bool:
    return str(reason or "").strip().lower() in {"ui-release", "client-stop"}


def _jog_backend_timeout_s(jog_backend) -> float | None:
    if jog_backend is None:
        return None
    return min(JOG_CONTROLLER_LEASE_TIMEOUT_S, JOG_BACKEND_LEASE_TIMEOUT_S)


_JOG_QUICK_STOP_REASON_TOKENS = {
    "controller-stop",
    "controller-shutdown",
    "fk-failed",
}


def _jog_stop_reason_requests_quick_stop(reason: str) -> bool:
    """Return True only for jog stops that should leave normal hold semantics.

    Browser/controller session lease expiry is deliberately a soft RTCore jog
    stop. RTCore's own motor-side deadline remains the final quick-stop
    authority if Python actually stops refreshing the drive-facing lease.
    """
    reason_lower = str(reason or "").strip().lower()
    if not reason_lower:
        return False
    tokens = {
        token
        for token in reason_lower.replace("/", ":").split(":")
        if token
    }
    return bool(tokens & _JOG_QUICK_STOP_REASON_TOKENS)


def _jog_drive_power_action_for_stop(*, quick_stop: bool) -> str:
    return "rtcore_jog_quick_stop" if quick_stop else "rtcore_jog_stop"


def _jog_session_log_fields(snapshot: dict[str, object] | None = None) -> str:
    snap = snapshot if isinstance(snapshot, dict) else _JOG_SESSION_MANAGER.get_snapshot()
    fields = {
        "session_id": snap.get("session_id"),
        "owner_id": snap.get("owner_id"),
        "state": snap.get("state"),
        "last_stop_reason": snap.get("last_stop_reason"),
        "lease_expiry_count": snap.get("lease_expiry_count"),
        "last_seq_received": snap.get("last_seq_received"),
        "last_seq_applied": snap.get("last_seq_applied"),
    }
    return " ".join(f"{key}={value}" for key, value in fields.items())


def _log_jog_backend_stop_request(reason: str, *, quick_stop: bool, snapshot: dict[str, object] | None = None) -> None:
    print(
        "[Jog] Backend stop request:"
        f" reason={str(reason or 'unknown')}"
        f" quick_stop={bool(quick_stop)}"
        f" drive_power_action={_jog_drive_power_action_for_stop(quick_stop=quick_stop)}"
        f" {_jog_session_log_fields(snapshot)}"
    )


def _log_jog_thread_exit(reason: str, *, snapshot: dict[str, object] | None = None) -> None:
    quick_stop = _jog_stop_reason_requests_quick_stop(reason)
    print(
        "[Jog] Jog controller thread exiting:"
        f" reason={str(reason or 'unknown')}"
        f" drive_power_action={_jog_drive_power_action_for_stop(quick_stop=quick_stop)}"
        f" {_jog_session_log_fields(snapshot)}"
    )


def _coerce_jog_vector(
    vx: object,
    vy: object,
    vz: object,
    v_roll: object,
    v_pitch: object,
    v_yaw: object,
) -> tuple[float, float, float, float, float, float]:
    return (
        float(vx),
        float(vy),
        float(vz),
        float(v_roll),
        float(v_pitch),
        float(v_yaw),
    )


def _jog_vector_from_payload(payload: dict[str, object]) -> tuple[float, float, float, float, float, float]:
    return _coerce_jog_vector(
        payload.get("vx", 0.0),
        payload.get("vy", 0.0),
        payload.get("vz", 0.0),
        payload.get("v_roll", 0.0),
        payload.get("v_pitch", 0.0),
        payload.get("v_yaw", 0.0),
    )


def _sync_jog_trajectory_state(*, touch_last_command: bool = False) -> dict[str, object]:
    snapshot = _JOG_SESSION_MANAGER.get_snapshot()
    control = _JOG_SESSION_MANAGER.get_control_state()
    utils.trajectory_state_update(
        is_jogging=bool(snapshot.get("session_active", False)),
        jog_deadman=bool(control.get("deadman", False)),
        jog_velocities=np.array(control.get("velocity_vector", _coerce_jog_vector(0, 0, 0, 0, 0, 0)), dtype=float),
        jog_gripper_velocity_deg_s=float(control.get("gripper_velocity_deg_s", 0.0) or 0.0),
        jog_execution_policy=str(snapshot.get("backend_mode") or ""),
        jog_session_id=snapshot.get("session_id"),
        jog_session_owner_id=snapshot.get("owner_id"),
        jog_session_state=str(snapshot.get("state") or "idle"),
    )
    if touch_last_command:
        utils.trajectory_state_set("last_jog_command_time", time.monotonic())
    return snapshot


def _reset_jog_perf(execution_policy: str, rtcore_owned: bool) -> None:
    global _JOG_PERF, _JOG_NEAR_SINGULARITY_ACTIVE, _JOG_NEAR_SINGULARITY_CLEAR_TICKS
    with _JOG_PERF_LOCK:
        _JOG_PERF = _new_jog_perf_state()
        _JOG_PERF["control_frequency_hz"] = int(JOG_CONTROL_FREQUENCY_HZ)
        _JOG_PERF["execution_policy"] = str(execution_policy)
        _JOG_PERF["rtcore_owned"] = bool(rtcore_owned)
    _JOG_NEAR_SINGULARITY_ACTIVE = False
    _JOG_NEAR_SINGULARITY_CLEAR_TICKS = 0


def _ensure_jog_thread_running() -> None:
    thread = utils.trajectory_state.get("jog_thread")
    if thread is not None and thread.is_alive():
        return
    jog_thread = threading.Thread(target=_jog_controller_thread, daemon=True)
    utils.trajectory_state["jog_thread"] = jog_thread
    jog_thread.start()


def _wait_for_jog_thread_stop(timeout_s: float = 0.5) -> None:
    thread = utils.trajectory_state.get("jog_thread")
    if thread is not None and thread.is_alive() and thread is not threading.current_thread():
        thread.join(timeout=timeout_s)
    if thread is None or not thread.is_alive():
        utils.trajectory_state["jog_thread"] = None


def _stop_rtcore_jog_backend_best_effort(reason: str) -> None:
    jog_backend = _get_rtcore_jog_backend()
    if jog_backend is None:
        return
    quick_stop = _jog_stop_reason_requests_quick_stop(reason)
    _log_jog_backend_stop_request(reason, quick_stop=quick_stop)
    try:
        stop_joint_velocity_lease_jog = getattr(jog_backend, "stop_joint_velocity_lease_jog", None)
        if callable(stop_joint_velocity_lease_jog):
            stop_joint_velocity_lease_jog(quick_stop=quick_stop)
    except Exception as exc:
        print(f"[Jog] WARNING: Failed to stop RTCore jog backend cleanly ({reason}): {exc}")


def handle_get_jog_session_state() -> dict[str, object]:
    return _JOG_SESSION_MANAGER.get_snapshot()


def handle_jog_session_start(payload: dict[str, object]) -> dict[str, object]:
    _require_motion_not_stop_latched("JOG_SESSION_START")
    if bool(utils.trajectory_state_get("is_running", False)):
        raise JogSessionError("MOTION_ACTIVE", "Cannot start jog while another motion is running.")

    # Risk mitigation: run a control-feedback read at arm time so the
    # session cannot start on a drive fault, offline axis, or missing PDO
    # feedback. Strict canonical-truth diagnostics are advisory here; they
    # remain visible through /info/joints-detailed.
    #
    # 2026-04-21 update: the original single-shot check was too brittle
    # in practice — the A6-EC's servo loop takes 0.5-2 s to settle after
    # SAFE_POWER_UP, during which the command-frame-roundtrip gate
    # transiently trips (`drive_native_command_frame_roundtrip_mismatch`)
    # even though nothing is physically wrong. See terminal 29.txt:270-281
    # for the live rejection sequence: the operator pressed jog 4 times
    # over ~1 s and every attempt was 503'd while the drive was still
    # decelerating / settling. A short retry loop (500 ms / 50 ms cadence
    # = 10 attempts) gives the drive time to settle without silently
    # accepting a permanently-bad anchor. If truth is STILL unavailable
    # after the retry window, we reject — a real encoder fault will not
    # clear itself in half a second, so the safety guarantee is preserved.
    truth_valid_at_arm = False
    last_truth_exc: RuntimeError | None = None
    arm_retry_attempts = 0
    # 2026-04-21 fast-path: if canonical truth was observed valid
    # within the last _JOG_ARM_RECENT_TRUTH_WINDOW_S seconds, skip the
    # strict retry loop entirely. The jog thread's 50 Hz feedback read
    # stamps `_note_valid_canonical_truth()` on every success, so any
    # release-then-re-click within a few seconds hits this path and
    # arms instantly — eliminating the up-to-500 ms wall of lag that
    # the retry loop otherwise imposed while the drive was still
    # decelerating. Phase 0 still absorbs any transient flicker during
    # the new session.
    if _recently_valid_canonical_truth():
        truth_valid_at_arm = True
    else:
        arm_retry_deadline = time.monotonic() + _JOG_ARM_TRUTH_RETRY_BUDGET_S
        while True:
            arm_retry_attempts += 1
            try:
                servo_driver.get_control_arm_state_rad(verbose=False)
            except RuntimeError as exc:
                last_truth_exc = exc
                if time.monotonic() >= arm_retry_deadline:
                    break
                time.sleep(_JOG_ARM_TRUTH_RETRY_INTERVAL_S)
                continue
            else:
                truth_valid_at_arm = True
                _note_valid_canonical_truth()
                break
    if not truth_valid_at_arm:
        message = str(last_truth_exc).strip() if last_truth_exc else "control feedback unavailable"
        if not message:
            message = "control feedback unavailable"
        print(
            f"[Jog] Rejecting jog session start after {arm_retry_attempts} attempt(s) "
            f"over {_JOG_ARM_TRUTH_RETRY_BUDGET_S * 1000.0:.0f} ms: {message}",
            file=sys.stderr,
            flush=True,
        )
        raise JogSessionError(
            "CONTROL_FEEDBACK_UNAVAILABLE",
            f"Cannot start jog: control feedback is unavailable. {message}",
        ) from last_truth_exc

    jog_backend = _get_rtcore_jog_backend()
    execution_policy = _jog_execution_policy(jog_backend)
    snapshot = _JOG_SESSION_MANAGER.start_session(
        owner_id=str(payload.get("owner_id", "") or ""),
        seq=int(payload.get("seq", 0)),
        lease_timeout_s=float(payload.get("lease_timeout_s", JOG_CONTROLLER_LEASE_TIMEOUT_S)),
        deadman=bool(payload.get("deadman", False)),
        velocity_vector=_jog_vector_from_payload(payload),
        gripper_velocity_deg_s=float(payload.get("gripper_velocity_deg_s", 0.0) or 0.0),
        backend_mode=execution_policy,
        backend_timeout_s=_jog_backend_timeout_s(jog_backend),
        session_id=str(payload.get("session_id", "") or "") or None,
    )
    # Expose the arm-time truth verdict on the session snapshot so the
    # frontend (via GET /control/jog/session/state) and operators
    # can see that the session started with a verified anchor.
    snapshot["truth_valid_at_arm"] = bool(truth_valid_at_arm)
    print(
        f"[Jog] Starting jog session {snapshot.get('session_id')} "
        f"({execution_policy}, truth_valid_at_arm={truth_valid_at_arm})"
    )
    _reset_jog_perf(execution_policy, jog_backend is not None)
    _sync_jog_trajectory_state(touch_last_command=True)
    _ensure_jog_thread_running()
    return snapshot


def handle_jog_session_update(payload: dict[str, object]) -> dict[str, object]:
    try:
        snapshot = _JOG_SESSION_MANAGER.update_session(
            session_id=str(payload.get("session_id", "") or ""),
            owner_id=str(payload.get("owner_id", "") or ""),
            seq=int(payload.get("seq", 0)),
            lease_timeout_s=float(payload["lease_timeout_s"]) if "lease_timeout_s" in payload else None,
            deadman=bool(payload.get("deadman", False)),
            velocity_vector=_jog_vector_from_payload(payload),
            gripper_velocity_deg_s=float(payload.get("gripper_velocity_deg_s", 0.0) or 0.0),
        )
    except JogSessionError as exc:
        if str(getattr(exc, "code", "") or "").upper() == "SESSION_EXPIRED":
            print(
                "[Jog] Session update rejected after lease expiry:"
                f" drive_power_action={_jog_drive_power_action_for_stop(quick_stop=False)}"
                f" {_jog_session_log_fields()}"
            )
        raise
    velocity_vector = np.array(_jog_vector_from_payload(payload), dtype=float)
    _record_jog_velocity_update(velocity_vector)
    _sync_jog_trajectory_state(touch_last_command=True)
    try:
        if utils.trajectory_state.get("jog_debug", False) and np.any(np.abs(velocity_vector) > 1e-6):
            print(f"[Jog] Session update: lin(m/s)={np.round(velocity_vector[:3],3)}, ang(deg/s)={np.round(velocity_vector[3:],1)}")
    except Exception:
        pass
    return snapshot


def handle_jog_session_stop(payload: dict[str, object]) -> dict[str, object]:
    reason = str(payload.get("reason", "client-stop") or "client-stop")
    if _should_gracefully_release_jog(reason):
        try:
            active_snapshot = _JOG_SESSION_MANAGER.get_snapshot()
            jog_backend = _get_rtcore_jog_backend()
            if bool(active_snapshot.get("session_active", False)) and jog_backend is not None:
                update_joint_velocity_lease_jog = getattr(jog_backend, "update_joint_velocity_lease_jog", None)
                if callable(update_joint_velocity_lease_jog):
                    num_joints = int(utils.NUM_LOGICAL_JOINTS or 6)
                    update_joint_velocity_lease_jog(
                        [0.0] * num_joints,
                        timeout_s=_jog_backend_timeout_s(jog_backend) or JOG_BACKEND_LEASE_TIMEOUT_S,
                    )
                    if JOG_UI_RELEASE_ZERO_SETTLE_S > 0.0:
                        time.sleep(min(float(JOG_UI_RELEASE_ZERO_SETTLE_S), 0.15))
        except Exception as exc:
            print(f"[Jog] WARNING: graceful jog release pre-stop failed: {exc}")
    snapshot = _JOG_SESSION_MANAGER.stop_session(
        session_id=str(payload.get("session_id", "") or "") or None,
        owner_id=str(payload.get("owner_id", "") or "") or None,
        reason=reason,
        allow_missing=True,
    )
    print(f"[Jog] Stopping jog session {snapshot.get('session_id')} ({reason})")
    _sync_jog_trajectory_state()
    _stop_rtcore_jog_backend_best_effort(f"session-stop:{reason}")
    _wait_for_jog_thread_stop()
    return snapshot


def stop_active_jog_session(reason: str = "controller-stop") -> dict[str, object]:
    return handle_jog_session_stop({"reason": str(reason or "controller-stop")})


def _jog_controller_thread():
    print("[Jog] Jog controller thread started.")
    jog_backend = _get_rtcore_jog_backend()
    jog_backend_timeout_s = _jog_backend_timeout_s(jog_backend)
    jog_backend_active = False
    try:
        q_current = servo_driver.get_control_arm_state_rad(verbose=False)
    except RuntimeError as exc:
        q_current = None
        _record_jog_truth_flicker(str(exc))
        _record_control_feedback_miss(str(exc))
    if q_current is None:
        q_current = np.zeros(utils.NUM_LOGICAL_JOINTS, dtype=float)
    else:
        q_current = np.asarray(q_current, dtype=float)
        _note_valid_canonical_truth()
        _record_control_feedback_ok()
    last_loop_time = time.monotonic()
    last_status_log_time = time.monotonic()
    was_paused_for_motion = False
    was_motion_command_active = False
    pending_resync_reason: str | None = "jog-start"
    target_period_ms = 1000.0 / float(JOG_CONTROL_FREQUENCY_HZ)
    applied_linear_vel = np.zeros(3, dtype=float)
    applied_angular_deg_s = np.zeros(3, dtype=float)

    def _stop_backend_jog_now(reason: str) -> None:
        nonlocal jog_backend_active
        if jog_backend is None or not jog_backend_active:
            return
        quick_stop = _jog_stop_reason_requests_quick_stop(reason)
        _log_jog_backend_stop_request(
            reason,
            quick_stop=quick_stop,
            snapshot=_JOG_SESSION_MANAGER.get_snapshot(),
        )
        try:
            stop_joint_velocity_lease_jog = getattr(jog_backend, "stop_joint_velocity_lease_jog", None)
            if callable(stop_joint_velocity_lease_jog):
                stop_joint_velocity_lease_jog(quick_stop=quick_stop)
        except Exception as exc:
            print(f"[Jog] WARNING: Failed to stop backend jog cleanly ({reason}): {exc}")
        finally:
            jog_backend_active = False

    # 2026-04-21 thread-race guard: a JOG_SESSION_STOP / lease expiry /
    # controller-stop can mutate the session state to `stopping` or
    # `stopped` AFTER the top-of-loop `session_active` check but BEFORE
    # the later session-manager mutations on the same tick
    # (update_following_error, resync_command_state, pause_for_motion,
    # resume_after_motion). Those mutations raise
    # `JogSessionError("SESSION_INACTIVE")` and previously propagated
    # out of the thread entrypoint, killing the thread with an
    # uncaught-exception traceback (see terminal 29.txt:253 — crash
    # observed during live cartesian jog). `_safe_call_session` wraps
    # any such mutation so SESSION_INACTIVE returns a sentinel and the
    # caller can fall through to the top-of-loop break path instead of
    # crashing. Non-SESSION_INACTIVE JogSessionErrors still propagate
    # because they indicate a real bug.
    _session_gone_sentinel = object()

    def _safe_session_call(fn, *args, **kwargs):
        try:
            return fn(*args, **kwargs)
        except JogSessionError as exc:
            code = str(getattr(exc, "code", "") or "").upper()
            if code != "SESSION_INACTIVE":
                raise
            return _session_gone_sentinel

    while True:
        session_snapshot = _JOG_SESSION_MANAGER.expire_if_needed()
        if not bool(session_snapshot.get("session_active", False)):
            stop_reason = str(
                session_snapshot.get("last_stop_reason")
                or session_snapshot.get("state")
                or "session-inactive-before-loop"
            ).strip().lower()
            if stop_reason in {"lease-expired", "expired"}:
                exit_reason = "lease-expired-before-loop"
            else:
                exit_reason = "session-inactive-before-loop"
            _log_jog_thread_exit(exit_reason, snapshot=session_snapshot)
            _stop_backend_jog_now(exit_reason)
            break

        loop_start_time = time.monotonic()
        dt = loop_start_time - last_loop_time
        last_loop_time = loop_start_time

        if bool(utils.trajectory_state_get("is_running", False)):
            if _safe_session_call(_JOG_SESSION_MANAGER.pause_for_motion) is _session_gone_sentinel:
                _log_jog_thread_exit("session-inactive-during-pause")
                _stop_backend_jog_now("session-inactive-during-pause")
                break
            _sync_jog_trajectory_state()
            _stop_backend_jog_now("pause-for-motion")
            was_paused_for_motion = True
            was_motion_command_active = False
            loop_duration = time.monotonic() - loop_start_time
            sleep_time = (1.0 / JOG_CONTROL_FREQUENCY_HZ) - loop_duration
            if sleep_time > 0:
                time.sleep(sleep_time)
            continue

        if was_paused_for_motion:
            try:
                fresh_q = servo_driver.get_control_arm_state_rad(verbose=False)
            except RuntimeError as exc:
                fresh_q = None
                _record_jog_truth_flicker(str(exc))
                _record_control_feedback_miss(str(exc))
            if fresh_q is not None:
                q_current = np.asarray(fresh_q, dtype=float)
                _note_valid_canonical_truth()
                _record_control_feedback_ok()
            if _safe_session_call(_JOG_SESSION_MANAGER.resume_after_motion) is _session_gone_sentinel:
                _log_jog_thread_exit("session-inactive-during-resume")
                _stop_backend_jog_now("session-inactive-during-resume")
                break
            _sync_jog_trajectory_state()
            last_loop_time = time.monotonic()
            was_paused_for_motion = False
            pending_resync_reason = "resume-after-motion"
            continue

        control = _JOG_SESSION_MANAGER.get_control_state()
        if not bool(control.get("session_active", False)):
            _log_jog_thread_exit("session-inactive-before-control")
            break

        feedback_read_started = time.monotonic()
        try:
            fresh_q = servo_driver.get_control_arm_state_rad(verbose=False)
        except RuntimeError as exc:
            fresh_q = None
            _record_jog_truth_flicker(str(exc))
            _record_control_feedback_miss(str(exc))
        _record_jog_stage_metric(
            "feedback_read_ms",
            max(0.0, (time.monotonic() - feedback_read_started) * 1000.0),
        )
        if fresh_q is not None:
            q_current = np.asarray(fresh_q, dtype=float)
            # Keep the recent-valid-truth window fresh so a release-
            # then-reclick hits the arm-time fast-path instead of the
            # 500 ms retry loop.
            _note_valid_canonical_truth()
            _record_control_feedback_ok()
        else:
            hold_control = _JOG_SESSION_MANAGER.get_control_state()
            if not bool(hold_control.get("session_active", False)):
                _log_jog_thread_exit("session-inactive-during-feedback-hold")
                break
            commanded_joints_for_hold = np.asarray(
                hold_control.get("commanded_joint_vector", q_current),
                dtype=float,
            ).reshape(-1)
            if commanded_joints_for_hold.size == 0:
                commanded_joints_for_hold = np.asarray(q_current, dtype=float).reshape(-1)
            if jog_backend is not None and jog_backend_active:
                try:
                    update_joint_velocity_lease_jog = getattr(jog_backend, "update_joint_velocity_lease_jog", None)
                    if callable(update_joint_velocity_lease_jog):
                        update_joint_velocity_lease_jog(
                            [0.0] * int(commanded_joints_for_hold.size),
                            timeout_s=jog_backend_timeout_s or JOG_BACKEND_LEASE_TIMEOUT_S,
                        )
                except Exception as hold_exc:
                    print(f"[Jog] WARNING: Failed to hold backend jog command after feedback miss: {hold_exc}")
            pending_resync_reason = "control-feedback-unavailable"
            loop_duration = time.monotonic() - loop_start_time
            sleep_time = (1.0 / JOG_CONTROL_FREQUENCY_HZ) - loop_duration
            if sleep_time > 0:
                time.sleep(sleep_time)
            continue

        measured_pose_matrix = ik_solver.get_fk_matrix(q_current)
        if measured_pose_matrix is None:
            print("[Jog] ERROR: FK failed during jog loop. Stopping.")
            _stop_backend_jog_now("fk-failed")
            break

        current_position = np.asarray(measured_pose_matrix[:3, 3], dtype=float)
        current_orientation = np.asarray(measured_pose_matrix[:3, :3], dtype=float)
        current_pose_snapshot = _pose_snapshot_from_components(current_position, current_orientation)
        measured_joints_deg = _joint_angles_deg_list(q_current)
        if utils.trajectory_state.get("jog_debug", False):
            try:
                curr_eul_deg = R.from_matrix(current_orientation).as_euler("xyz", degrees=True)
                print(f"[Jog] CURR pos(m)={np.round(current_position,4)} eulXYZ(deg)={np.round(curr_eul_deg,2)}")
            except Exception:
                pass

        velocities = np.asarray(control.get("velocity_vector", _coerce_jog_vector(0, 0, 0, 0, 0, 0)), dtype=float)
        requested_linear_vel = np.clip(velocities[:3], -MAX_JOG_LINEAR_M_S, MAX_JOG_LINEAR_M_S)
        requested_angular_deg_s = np.clip(velocities[3:], -MAX_JOG_ANGULAR_DEG_S, MAX_JOG_ANGULAR_DEG_S)
        linear_vel = _slew_limited_vector(
            applied_linear_vel,
            requested_linear_vel,
            max(0.0, float(JOG_LINEAR_ACCEL_M_S2) * float(dt)),
        )
        angular_deg_s = _slew_limited_vector(
            applied_angular_deg_s,
            requested_angular_deg_s,
            max(0.0, float(JOG_ANGULAR_ACCEL_DEG_S2) * float(dt)),
        )
        applied_linear_vel = linear_vel
        applied_angular_deg_s = angular_deg_s
        motion_command_active = bool(
            np.any(np.abs(requested_linear_vel) > 1e-9) or np.any(np.abs(requested_angular_deg_s) > 1e-9)
        )
        if motion_command_active and not was_motion_command_active and pending_resync_reason is None:
            pending_resync_reason = "idle-resume"

        if pending_resync_reason is not None or not bool(control.get("command_state_valid", False)):
            if (
                _safe_session_call(
                    _JOG_SESSION_MANAGER.resync_command_state,
                    position_m=current_position.tolist(),
                    orientation_matrix=current_orientation.tolist(),
                    joint_vector=q_current.tolist(),
                    reason=pending_resync_reason or "boundary-resync",
                )
                is _session_gone_sentinel
            ):
                _log_jog_thread_exit("session-inactive-during-resync")
                _stop_backend_jog_now("session-inactive-during-resync")
                break
            control = _JOG_SESSION_MANAGER.get_control_state()
            pending_resync_reason = None

        commanded_position = np.asarray(control.get("commanded_position_m"), dtype=float).reshape(3)
        commanded_orientation = np.asarray(control.get("commanded_orientation_matrix"), dtype=float).reshape(3, 3)
        commanded_joints = np.asarray(control.get("commanded_joint_vector"), dtype=float).reshape(-1)
        current_commanded_pose_snapshot = _pose_snapshot_from_components(commanded_position, commanded_orientation)
        current_commanded_pose_matrix = np.eye(4, dtype=float)
        current_commanded_pose_matrix[:3, :3] = commanded_orientation
        current_commanded_pose_matrix[:3, 3] = commanded_position
        # Pure-math following-error snapshot is kept here because the
        # ik_debug payload below needs it. What USED to live here but
        # was moved to after the command send (see DEFERRED TELEMETRY
        # BLOCK): the `_JOG_SESSION_MANAGER.update_following_error`
        # call, a follow-up `get_control_state()` fetch, a
        # `_build_jog_command_state_perf_fields` build, and a
        # pre-send `_jog_perf_update` — that block cost ~8-12 ms per
        # tick on live hardware (lock acquisition + dict snapshots +
        # perf mutations) and was the primary driver of the "controls
        # feel laggy" complaint. Moving those to after the jog backend
        # receives the velocity command drops the user-visible
        # feedback→command window from ~20 ms to ~4 ms.
        following_error = _following_error_snapshot(
            commanded_position=commanded_position,
            commanded_orientation_matrix=commanded_orientation,
            commanded_joints=commanded_joints,
            measured_pose_matrix=measured_pose_matrix,
            measured_joints=q_current,
        )

        using_jacobian = bool(JOG_USE_JACOBIAN and getattr(ik_solver, "is_jacobian_available", lambda: False)())
        jacobian_status = (
            ik_solver.get_jacobian_status()
            if hasattr(ik_solver, "get_jacobian_status")
            else {"available": False, "unavailable_reason": "jacobian-status-unavailable"}
        )
        jacobian_unavailable_reason = (
            None
            if using_jacobian
            else str(jacobian_status.get("unavailable_reason", "") or "")
        )
        command_horizon_s = (
            max(float(dt), 1.0 / float(JOG_CONTROL_FREQUENCY_HZ))
            if using_jacobian
            else float(dt)
        )
        jacobian_diag: dict[str, object] | None = None
        command_drift_norm = float(np.linalg.norm(commanded_joints - q_current))
        command_drift_per_joint = [float(v) for v in (commanded_joints - q_current).tolist()]
        q_dot_rad_s: np.ndarray | None = None
        ab_compare: dict[str, object] | None = None

        if using_jacobian:
            jacobian_started = time.monotonic()
            twist = np.concatenate(
                [
                    np.asarray(linear_vel, dtype=float).reshape(3),
                    np.deg2rad(np.asarray(angular_deg_s, dtype=float).reshape(3)),
                ]
            )
            q_dot_rad_s, jacobian_diag = _compute_jog_joint_velocity_via_jacobian(
                q_seed=commanded_joints,
                twist=twist,
            )
            _record_jog_stage_metric(
                "jacobian_compute_ms",
                max(0.0, (time.monotonic() - jacobian_started) * 1000.0),
            )
            achieved_twist = np.asarray(
                jacobian_diag.get("achieved_twist", twist),
                dtype=float,
            ).reshape(6)
            _update_jog_singularity_advisory(jacobian_diag)
            q_target = commanded_joints + q_dot_rad_s * command_horizon_s
            target_position = commanded_position + achieved_twist[:3] * command_horizon_s
            target_orientation = (
                R.from_rotvec(achieved_twist[3:] * command_horizon_s).as_matrix()
                @ commanded_orientation
            )
        else:
            target_position = commanded_position + linear_vel * command_horizon_s
            delta_rotation = R.from_euler("xyz", angular_deg_s * command_horizon_s, degrees=True).as_matrix()
            target_orientation = delta_rotation @ commanded_orientation
            ik_started = time.monotonic()
            q_target = ik_solver.solve_ik(
                target_position=target_position,
                target_orientation_matrix=target_orientation,
                initial_joint_angles=commanded_joints,
            )
            _record_jog_stage_metric(
                "ik_solve_ms",
                max(0.0, (time.monotonic() - ik_started) * 1000.0),
            )

        target_pose_snapshot = _pose_snapshot_from_components(target_position, target_orientation)
        if utils.trajectory_state.get("jog_debug", False):
            try:
                targ_eul_deg = R.from_matrix(target_orientation).as_euler("xyz", degrees=True)
                print(f"[Jog] TARG pos(m)={np.round(target_position,4)} eulXYZ(deg)={np.round(targ_eul_deg,2)} vel_lin={np.round(linear_vel,4)} vel_ang(deg/s)={np.round(angular_deg_s,1)} dt={dt:.4f} horizon={command_horizon_s:.4f} jacobian={using_jacobian}")
            except Exception:
                pass

        if q_target is not None:
            try:
                q_arr = (
                    np.asarray(q_target, dtype=float).reshape(-1)
                    if using_jacobian
                    else _unwrap_jog_joint_target(commanded_joints, q_target)
                )
                if using_jacobian and command_drift_norm > JOG_MAX_COMMAND_DRIFT_RAD:
                    drift_details = {
                        "reason_code": "JOG_COMMAND_DRIFT_EXCEEDED",
                        "command_drift_norm_rad": command_drift_norm,
                        "command_drift_per_joint_rad": command_drift_per_joint,
                        "threshold_rad": float(JOG_MAX_COMMAND_DRIFT_RAD),
                    }
                    _safe_session_call(
                        _JOG_SESSION_MANAGER.record_gate_failure,
                        reason="JOG_COMMAND_DRIFT_EXCEEDED",
                        details=drift_details,
                    )
                    _emit_jog_gate_alert("JOG_COMMAND_DRIFT_EXCEEDED", drift_details)
                    if jog_backend is not None and jog_backend_active:
                        try:
                            update_joint_velocity_lease_jog = getattr(jog_backend, "update_joint_velocity_lease_jog", None)
                            if callable(update_joint_velocity_lease_jog):
                                update_joint_velocity_lease_jog(
                                    [0.0] * len(commanded_joints),
                                    timeout_s=jog_backend_timeout_s or JOG_BACKEND_LEASE_TIMEOUT_S,
                                )
                        except Exception as exc:
                            print(f"[Jog] WARNING: Failed to hold backend jog command after drift watchdog: {exc}")
                    pending_resync_reason = "command-drift-exceeded"
                    perf_fields_after_gate = _build_jog_command_state_perf_fields(
                        _JOG_SESSION_MANAGER.get_control_state()
                    )
                    _jog_perf_update(
                        measured_pose=current_pose_snapshot,
                        measured_joints_deg=measured_joints_deg,
                        **perf_fields_after_gate,
                    )
                    _jog_perf_update(
                        ik_debug=_build_jog_ik_debug_payload(
                            control=control,
                            dt=dt,
                            linear_vel=linear_vel,
                            angular_deg_s=angular_deg_s,
                            current_pose_snapshot=current_pose_snapshot,
                            current_commanded_pose_snapshot=current_commanded_pose_snapshot,
                            target_pose_snapshot=target_pose_snapshot,
                            solved_pose_matrix=current_commanded_pose_matrix,
                            applied_pose_matrix=current_commanded_pose_matrix,
                            gate_ok=False,
                            target_position=target_position,
                            target_orientation=target_orientation,
                            measured_joints_deg=measured_joints_deg,
                            commanded_joints=commanded_joints,
                            q_arr=q_arr,
                            applied_joint_vector=commanded_joints,
                            following_error=following_error,
                            perf_fields_after_gate=perf_fields_after_gate,
                            gate_reason="JOG_COMMAND_DRIFT_EXCEEDED",
                            gate_details=drift_details,
                            jacobian_diag=jacobian_diag,
                            command_drift_norm=command_drift_norm,
                            command_drift_per_joint=command_drift_per_joint,
                            ab_compare=None,
                            recovery_action=pending_resync_reason,
                            command_horizon_s=command_horizon_s,
                            using_jacobian=using_jacobian,
                            jacobian_unavailable_reason=jacobian_unavailable_reason,
                        )
                    )
                    was_motion_command_active = motion_command_active
                    loop_duration = time.monotonic() - loop_start_time
                    sleep_time = (1.0 / JOG_CONTROL_FREQUENCY_HZ) - loop_duration
                    if sleep_time > 0:
                        time.sleep(sleep_time)
                    continue
                gate_ok, gate_reason, gate_details, solved_pose_matrix = _validate_jog_step_candidate(
                    seed_joint_angles=commanded_joints,
                    candidate_joint_angles=q_arr,
                    target_position=target_position,
                    target_orientation=target_orientation,
                )
                applied_pose_matrix = solved_pose_matrix if gate_ok else current_commanded_pose_matrix
                applied_joint_vector = q_arr if gate_ok else commanded_joints
                if not gate_ok:
                    if gate_reason == "LIMIT_VIOLATION":
                        limits = np.array(utils.LOGICAL_JOINT_LIMITS_RAD, dtype=float)
                        mins = limits[:, 0]
                        maxs = limits[:, 1]
                        violating_idx = [
                            int(idx)
                            for idx in gate_details.get("violating_joint_indices", [])
                            if isinstance(idx, (int, float))
                        ]
                        if violating_idx:
                            _emit_joint_limit_alert(
                                violating_idx,
                                q_arr,
                                np.clip(q_arr, mins, maxs),
                                mins,
                                maxs,
                                source="jog",
                            )
                    _safe_session_call(
                        _JOG_SESSION_MANAGER.record_gate_failure,
                        reason=gate_reason,
                        details=gate_details,
                    )
                    _emit_jog_gate_alert(gate_reason, gate_details)
                    if jog_backend is not None and jog_backend_active:
                        try:
                            update_joint_velocity_lease_jog = getattr(jog_backend, "update_joint_velocity_lease_jog", None)
                            if callable(update_joint_velocity_lease_jog):
                                update_joint_velocity_lease_jog(
                                    [0.0] * len(commanded_joints),
                                    timeout_s=jog_backend_timeout_s or JOG_BACKEND_LEASE_TIMEOUT_S,
                                )
                        except Exception as exc:
                            print(f"[Jog] WARNING: Failed to hold backend jog command after gate rejection: {exc}")
                    # Gate-failure telemetry runs inline — latency is
                    # not user-visible in the failure path because no
                    # motion happens anyway. Sleep + continue to next
                    # tick after telemetry flushes.
                    perf_fields_after_gate = _build_jog_command_state_perf_fields(
                        _JOG_SESSION_MANAGER.get_control_state()
                    )
                    _jog_perf_update(
                        measured_pose=current_pose_snapshot,
                        measured_joints_deg=measured_joints_deg,
                        **perf_fields_after_gate,
                    )
                    _jog_perf_update(
                        ik_debug=_build_jog_ik_debug_payload(
                            control=control,
                            dt=dt,
                            linear_vel=linear_vel,
                            angular_deg_s=angular_deg_s,
                            current_pose_snapshot=current_pose_snapshot,
                            current_commanded_pose_snapshot=current_commanded_pose_snapshot,
                            target_pose_snapshot=target_pose_snapshot,
                            solved_pose_matrix=solved_pose_matrix,
                            applied_pose_matrix=applied_pose_matrix,
                            gate_ok=False,
                            target_position=target_position,
                            target_orientation=target_orientation,
                            measured_joints_deg=measured_joints_deg,
                            commanded_joints=commanded_joints,
                            q_arr=q_arr,
                            applied_joint_vector=applied_joint_vector,
                            following_error=following_error,
                            perf_fields_after_gate=perf_fields_after_gate,
                            gate_reason=gate_reason,
                            gate_details=gate_details,
                            jacobian_diag=jacobian_diag,
                            command_drift_norm=command_drift_norm,
                            command_drift_per_joint=command_drift_per_joint,
                            ab_compare=None,
                            recovery_action=pending_resync_reason,
                            command_horizon_s=command_horizon_s,
                            using_jacobian=using_jacobian,
                            jacobian_unavailable_reason=jacobian_unavailable_reason,
                        )
                    )
                    if utils.trajectory_state.get("jog_debug", False):
                        dq = q_arr - commanded_joints
                        print(f"[Jog] q_delta(rad)={np.round(dq, 5)} | lin={np.round(linear_vel,4)} m/s, ang={np.round(angular_deg_s,1)} deg/s, dt={dt:.4f}s")
                    was_motion_command_active = motion_command_active
                    loop_duration = time.monotonic() - loop_start_time
                    sleep_time = (1.0 / JOG_CONTROL_FREQUENCY_HZ) - loop_duration
                    if sleep_time > 0:
                        time.sleep(sleep_time)
                    continue

                # --- HOT PATH (gate_ok == True): send to RTCore ASAP ---
                # 2026-04-21 latency-kill: the entire "command send"
                # flow runs BEFORE any of the heavy telemetry below, so
                # the user-visible feedback→motion window stays under
                # ~4 ms regardless of how much UI state we build after.
                command_send_started = time.monotonic()
                control_before_send = _JOG_SESSION_MANAGER.get_control_state()
                if not bool(control_before_send.get("session_active", False)):
                    _log_jog_thread_exit("session-stopped-before-send")
                    _stop_backend_jog_now("session-stopped-before-send")
                    break
                if not bool(control_before_send.get("lease_valid", False)):
                    _JOG_SESSION_MANAGER.expire_if_needed()
                    _sync_jog_trajectory_state()
                    _log_jog_thread_exit("lease-expired-before-send")
                    _stop_backend_jog_now("lease-expired-before-send")
                    break
                if jog_backend is not None:
                    if not jog_backend_active:
                        start_joint_velocity_lease_jog = getattr(jog_backend, "start_joint_velocity_lease_jog", None)
                        if not callable(start_joint_velocity_lease_jog):
                            raise RuntimeError("Jog backend does not expose start_joint_velocity_lease_jog()")
                        start_joint_velocity_lease_jog(timeout_s=jog_backend_timeout_s or JOG_BACKEND_LEASE_TIMEOUT_S)
                        jog_backend_active = True
                    update_joint_velocity_lease_jog = getattr(jog_backend, "update_joint_velocity_lease_jog", None)
                    if not callable(update_joint_velocity_lease_jog):
                        raise RuntimeError("Jog backend does not expose update_joint_velocity_lease_jog()")
                    if using_jacobian and q_dot_rad_s is not None:
                        joint_velocity_cmd = q_dot_rad_s.tolist()
                    else:
                        dt_safe = max(dt, 1.0 / float(JOG_CONTROL_FREQUENCY_HZ))
                        joint_velocity_cmd = ((q_arr - commanded_joints) / dt_safe).tolist()
                    update_joint_velocity_lease_jog(
                        joint_velocity_cmd,
                        timeout_s=jog_backend_timeout_s or JOG_BACKEND_LEASE_TIMEOUT_S,
                    )
                else:
                    servo_driver.set_servo_positions(q_arr, 800, 0)
                _record_jog_stage_metric(
                    "command_send_ms",
                    max(0.0, (time.monotonic() - command_send_started) * 1000.0),
                )

                # --- DEFERRED TELEMETRY (runs AFTER command hits RTCore) ---
                # All of this used to block the command send. Moving it
                # here preserves the same UI state but eliminates the
                # user-visible latency spike before motion.
                if using_jacobian and _JOG_DIAGNOSTIC_AB_COMPARE:
                    q_target_ik = ik_solver.solve_ik(
                        target_position=target_position,
                        target_orientation_matrix=target_orientation,
                        initial_joint_angles=commanded_joints,
                    )
                    if q_target_ik is not None:
                        q_target_ik_unwrapped = _unwrap_jog_joint_target(commanded_joints, q_target_ik)
                        ik_minus_jacobian = q_target_ik_unwrapped - q_arr
                        ab_compare = {
                            "ik_q_target_rad": [float(v) for v in q_target_ik_unwrapped.tolist()],
                            "jacobian_q_target_rad": [float(v) for v in q_arr.tolist()],
                            "ik_minus_jacobian_max_abs_rad": float(np.max(np.abs(ik_minus_jacobian))),
                            "ik_minus_jacobian_per_joint_rad": [
                                float(v) for v in ik_minus_jacobian.tolist()
                            ],
                        }
                accepted_position = target_position
                accepted_orientation = target_orientation
                if using_jacobian and solved_pose_matrix is not None:
                    accepted_position = np.asarray(solved_pose_matrix[:3, 3], dtype=float)
                    accepted_orientation = np.asarray(solved_pose_matrix[:3, :3], dtype=float)
                accepted_commanded_pose_snapshot = _pose_snapshot_from_components(
                    accepted_position,
                    accepted_orientation,
                )
                _JOG_SESSION_MANAGER.accept_command_step(
                    position_m=accepted_position.tolist(),
                    orientation_matrix=accepted_orientation.tolist(),
                    joint_vector=q_arr.tolist(),
                )
                accepted_perf_fields = _build_jog_command_state_perf_fields(
                    _JOG_SESSION_MANAGER.get_control_state()
                )
                _jog_perf_update(
                    measured_pose=current_pose_snapshot,
                    measured_joints_deg=measured_joints_deg,
                    **accepted_perf_fields,
                )
                _jog_perf_update(
                    ik_debug=_build_jog_ik_debug_payload(
                        control=control,
                        dt=dt,
                        linear_vel=linear_vel,
                        angular_deg_s=angular_deg_s,
                        current_pose_snapshot=current_pose_snapshot,
                        current_commanded_pose_snapshot=current_commanded_pose_snapshot,
                        target_pose_snapshot=target_pose_snapshot,
                        solved_pose_matrix=solved_pose_matrix,
                        applied_pose_matrix=applied_pose_matrix,
                        gate_ok=True,
                        target_position=target_position,
                        target_orientation=target_orientation,
                        measured_joints_deg=measured_joints_deg,
                        commanded_joints=commanded_joints,
                        q_arr=q_arr,
                        applied_joint_vector=applied_joint_vector,
                        following_error=following_error,
                        perf_fields_after_gate=accepted_perf_fields,
                        gate_reason=gate_reason,
                        gate_details=gate_details,
                        jacobian_diag=jacobian_diag,
                        command_drift_norm=command_drift_norm,
                        command_drift_per_joint=command_drift_per_joint,
                        ab_compare=ab_compare,
                        recovery_action=pending_resync_reason,
                        command_horizon_s=command_horizon_s,
                        using_jacobian=using_jacobian,
                        jacobian_unavailable_reason=jacobian_unavailable_reason,
                        accepted_commanded_pose_snapshot=accepted_commanded_pose_snapshot,
                    )
                )
                if utils.trajectory_state.get("jog_debug", False):
                    dq = q_arr - commanded_joints
                    print(f"[Jog] q_delta(rad)={np.round(dq, 5)} | lin={np.round(linear_vel,4)} m/s, ang={np.round(angular_deg_s,1)} deg/s, dt={dt:.4f}s")
                _JOG_SESSION_MANAGER.mark_seq_applied(int(control.get("last_seq_received", -1)))
                try:
                    _safe_session_call(
                        _JOG_SESSION_MANAGER.update_following_error,
                        following_error,
                    )
                except Exception as exc:
                    print(f"[Jog] WARNING: deferred following-error update failed: {exc}")
            except Exception as exc:
                print(f"[Jog] WARNING: Failed to validate/apply jog step: {exc}")
        else:
            gate_reason = "IK_NO_SOLUTION"
            gate_details = {
                "max_joint_step_rad": None,
                "joint_limit_margin_rad": None,
                "cartesian_residual_m": None,
                "orientation_residual_deg": None,
                "violating_joint_indices": [],
            }
            _safe_session_call(
                _JOG_SESSION_MANAGER.record_gate_failure,
                reason=gate_reason,
                details=gate_details,
            )
            perf_fields_after_gate = _build_jog_command_state_perf_fields(_JOG_SESSION_MANAGER.get_control_state())
            _jog_perf_update(
                measured_pose=current_pose_snapshot,
                measured_joints_deg=measured_joints_deg,
                **perf_fields_after_gate,
            )
            _jog_perf_update(
                ik_debug={
                    "captured_at": datetime.datetime.now(datetime.timezone.utc).isoformat(),
                    "seq": int(control.get("last_seq_received", -1)),
                    "dt_s": float(dt),
                    "linear_velocity_m_s": _vector_to_float_list(linear_vel),
                    "angular_velocity_deg_s": _vector_to_float_list(angular_deg_s),
                    "current_pose": current_pose_snapshot,
                    "measured_pose": current_pose_snapshot,
                    "commanded_pose": current_commanded_pose_snapshot,
                    "target_pose": target_pose_snapshot,
                    "current_joints_deg": measured_joints_deg,
                    "measured_joints_deg": measured_joints_deg,
                    "commanded_joints_deg": _joint_angles_deg_list(commanded_joints),
                    "ik_seed_joints_deg": _joint_angles_deg_list(commanded_joints),
                    "ik_solution_joints_deg": None,
                    "applied_joints_deg": _joint_angles_deg_list(commanded_joints),
                    "solved_pose": None,
                    "applied_pose": current_commanded_pose_snapshot,
                    "target_vs_solved": None,
                    "target_vs_applied": _pose_error_snapshot(
                        target_position,
                        target_orientation,
                        current_commanded_pose_matrix,
                    ),
                    "clamped_joint_indices": [],
                    "clamped": False,
                    "solve_failed": True,
                    "following_error": following_error,
                    "last_resync_reason": perf_fields_after_gate.get("last_resync_reason"),
                    "last_resync_age_s": perf_fields_after_gate.get("last_resync_age_s"),
                    "gate_result": "rejected",
                    "gate_reason": gate_reason,
                    "gate_details": gate_details,
                    "jacobian_diagnostics": jacobian_diag,
                    "command_drift_norm_rad": command_drift_norm,
                    "command_drift_per_joint_rad": command_drift_per_joint,
                    "q_dot_rad_s": None,
                    "twist": None,
                    "ab_compare": None,
                    "recovery_action": pending_resync_reason,
                    "command_horizon_s": float(command_horizon_s),
                    "using_jacobian": bool(using_jacobian),
                    "jacobian_unavailable_reason": jacobian_unavailable_reason,
                }
            )
            print("[Jog] WARNING: IK solution not found for step.")
            _emit_jog_gate_alert(gate_reason, gate_details)
            if jog_backend is not None and jog_backend_active:
                try:
                    update_joint_velocity_lease_jog = getattr(jog_backend, "update_joint_velocity_lease_jog", None)
                    if callable(update_joint_velocity_lease_jog):
                        update_joint_velocity_lease_jog(
                            [0.0] * len(commanded_joints),
                            timeout_s=jog_backend_timeout_s or JOG_BACKEND_LEASE_TIMEOUT_S,
                        )
                except Exception as exc:
                    print(f"[Jog] WARNING: Failed to zero backend jog command after IK miss: {exc}")

        loop_duration = time.monotonic() - loop_start_time
        sleep_time = (1.0 / JOG_CONTROL_FREQUENCY_HZ) - loop_duration
        if sleep_time > 0:
            time.sleep(sleep_time)

        try:
            if utils.gripper_present:
                rate_deg_s = float(control.get("gripper_velocity_deg_s", 0.0) or 0.0)
                rate_deg_s = float(np.clip(rate_deg_s, -MAX_GRIPPER_JOG_DEG_S, MAX_GRIPPER_JOG_DEG_S))
                if abs(rate_deg_s) > 1e-3:
                    current_deg = float(np.rad2deg(utils.current_gripper_angle_rad))
                    target_deg = current_deg + rate_deg_s * dt
                    min_rad, max_rad = utils.GRIPPER_LIMITS_RAD
                    target_rad_unclamped = float(np.deg2rad(target_deg))
                    target_rad = float(np.clip(target_rad_unclamped, min_rad, max_rad))
                    if abs(target_rad - target_rad_unclamped) > 1e-6:
                        print("[Jog] NOTE: Gripper target clamped to limits.")
                    speed_scaled = max(100, min(800, int(abs(rate_deg_s) * 4 + 100)))
                    servo_driver.set_single_servo_position_rads(
                        servo_id=utils.SERVO_ID_GRIPPER,
                        position_rad=target_rad,
                        speed=speed_scaled,
                        accel=0,
                    )
                    utils.current_gripper_angle_rad = target_rad
        except Exception as exc:
            print(f"[Jog] WARNING: Gripper jog update failed: {exc}")

        now = time.monotonic()
        if now - last_status_log_time > 0.5:
            if utils.trajectory_state.get("jog_debug", False):
                print(f"[Jog] dt={dt*1000:.1f}ms, v_lin={np.round(linear_vel,3)}, v_ang(deg/s)={np.round(angular_deg_s,1)}")
            last_status_log_time = now
        was_motion_command_active = motion_command_active
        _record_jog_loop_metric(
            max(0.0, (time.monotonic() - loop_start_time) * 1000.0),
            target_period_ms=target_period_ms,
        )

    print("[Jog] Jog controller thread stopped.")
    _stop_backend_jog_now("thread-exit")
    if jog_backend is None:
        try:
            current_angles = servo_driver.get_control_arm_state_rad(verbose=False)
        except RuntimeError:
            current_angles = None
        if current_angles:
            servo_driver.set_servo_positions(current_angles, 0, 100)
    _sync_jog_trajectory_state()
    if utils.trajectory_state.get("jog_thread") is threading.current_thread():
        utils.trajectory_state["jog_thread"] = None


def handle_set_jog_debug(enabled: bool):
    utils.trajectory_state["jog_debug"] = bool(enabled)
    print(f"[Jog] Debug logging set to {enabled}")


# -----------------------------------------------------------------------------
# Recording subsystem: PLAN_TRAJECTORY / REC_POS / END_TRAJECTORY
# -----------------------------------------------------------------------------

# Holds intermediate state while a user is interactively recording a trajectory.
_recording_state = {
    "is_recording": False,
    "points": [],  # list of dicts: {"position": [...], "orientation_euler_deg": [...]}
    "start_time": None,
}

# Project root (three levels up from this file): .../GradientOS
_PROJECT_ROOT_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", ".."))
# Recorded trajectories live at the project root
RECORDED_TRAJ_DIR = os.path.join(_PROJECT_ROOT_DIR, "recorded_trajectories")

# Well-known filename for the most recent PLAN_TRAJECTORY_POINTS preview
PLANNED_PREVIEW_NAME = "__planner_preview__"
PLANNED_PREVIEW_FILENAME = f"{PLANNED_PREVIEW_NAME}.json"
WELD_PREVIEW_NAME = "__weld_preview__"


def _ensure_record_dir_exists():
    """Create the recorded_trajectories directory if it does not already exist."""
    try:
        os.makedirs(RECORDED_TRAJ_DIR, exist_ok=True)
    except Exception as e:
        print(f"[Recorder] ERROR: Could not create directory {RECORDED_TRAJ_DIR}: {e}")


def handle_plan_trajectory_start():
    """Initiate a new recording session."""
    if _recording_state["is_recording"]:
        print("[Recorder] WARNING: Already recording. Send END_TRAJECTORY first if you want to start a new one.")
        return

    _recording_state["is_recording"] = True
    _recording_state["points"].clear()
    _recording_state["start_time"] = datetime.datetime.now()
    print("[Recorder] *** Recording mode ENABLED. Use REC_POS to add way-points, END_TRAJECTORY,<name> to finish. ***")


def handle_record_position():
    """Record the current end-effector pose (position + orientation)."""
    if not _recording_state["is_recording"]:
        print("[Recorder] ERROR: Not currently recording. Send PLAN_TRAJECTORY first.")
        return

    # Query current joint angles and FK
    current_q = servo_driver.get_current_arm_state_rad(verbose=False)
    pose_matrix = ik_solver.get_fk_matrix(current_q)
    if pose_matrix is None:
        print("[Recorder] ERROR: FK failed – cannot record point.")
        return

    position = pose_matrix[:3, 3].tolist()
    # Convert orientation matrix → XYZ intrinsic Euler degrees for human-friendly storage
    orientation_euler_deg = R.from_matrix(pose_matrix[:3, :3]).as_euler('xyz', degrees=True).tolist()

    _recording_state["points"].append({
        "position": [round(p, 4) for p in position],
        "orientation_euler_deg": [round(o, 2) for o in orientation_euler_deg],
    })
    print(f"[Recorder] Way-point #{len(_recording_state['points'])} recorded: Pos={position}, EulerDeg={orientation_euler_deg}")


def handle_end_trajectory(traj_name: str):
    """Finalize the recording and dump to JSON file under recorded_trajectories/."""
    if not _recording_state["is_recording"]:
        print("[Recorder] ERROR: Not currently recording – nothing to end.")
        return

    if not traj_name:
        print("[Recorder] ERROR: Trajectory name required. Use END_TRAJECTORY,<name>.")
        return

    if len(_recording_state["points"]) == 0:
        print("[Recorder] WARNING: No points recorded – nothing will be saved.")
        _recording_state["is_recording"] = False
        return

    _ensure_record_dir_exists()

    file_path = os.path.join(RECORDED_TRAJ_DIR, f"{traj_name}.json")
    if os.path.exists(file_path):
        print(f"[Recorder] WARNING: File {file_path} already exists – it will be overwritten.")

    # Build moves list from the recorded poses without inserting implicit dwell.
    moves = []
    for i, p in enumerate(_recording_state["points"]):
        moves.append({
            "command": "move_absolute",
            "vector": p["position"],
            "orientation_euler_deg": p["orientation_euler_deg"],
        })

    traj_dict = {
        "description": f"Recorded on {_recording_state['start_time'].strftime('%Y-%m-%d %H:%M:%S')}",
        "loop": False,
        "orientation_euler_angles_deg": None,  # kept for future use
        "moves": moves,
    }

    try:
        with open(file_path, "w") as f:
            json.dump(traj_dict, f, indent=2)
        print(f"[Recorder] Trajectory saved to {file_path} (total moves: {len(moves)})")
    except Exception as e:
        print(f"[Recorder] ERROR: Failed to write file {file_path}: {e}")

    # Reset state
    _recording_state["is_recording"] = False
    _recording_state["points"].clear()


def _plan_preview_points_payload(
    points: list[list[float]] | list[tuple[float, float, float]],
    *,
    preview_name: str,
    description: str,
    weld_metadata: dict | None = None,
    sections: list[dict] | None = None,
    pose_waypoints: list[dict] | None = None,
) -> dict:
    if len(points) == 0:
        raise ValueError("At least one waypoint is required.")

    current_q = _get_best_available_joint_state()
    trajectory_start_fk = ik_solver.get_fk(current_q.tolist())
    trajectory_start_pos = (
        np.array(trajectory_start_fk, dtype=float)
        if trajectory_start_fk is not None
        else None
    )
    planned_steps = []
    waypoint_results = []
    cartesian_samples = []
    planning_warnings: list[str] = []
    planning_succeeded = True
    failed_waypoint_idx: int | None = None
    failed_waypoint_target: np.ndarray | None = None
    failed_segment_start_fk: np.ndarray | None = None
    plan_velocity = utils.DEFAULT_PROFILE_VELOCITY
    if plan_velocity is None or not np.isfinite(plan_velocity) or float(plan_velocity) <= 0:
        plan_velocity = 0.08
    plan_acceleration = utils.DEFAULT_PROFILE_ACCELERATION
    if plan_acceleration is None or not np.isfinite(plan_acceleration) or float(plan_acceleration) <= 0:
        plan_acceleration = 0.2
    standard_pose_waypoints = pose_waypoints if isinstance(pose_waypoints, list) and len(pose_waypoints) == len(points) else None

    def _append_cartesian_samples(joint_path: list[list[float]]) -> None:
        for joint_sample in joint_path:
            pose_matrix = ik_solver.get_fk_matrix(np.array(joint_sample))
            if pose_matrix is None:
                continue
            position = pose_matrix[:3, 3]
            cartesian_samples.append(
                [
                    round(float(position[0]), 4),
                    round(float(position[1]), 4),
                    round(float(position[2]), 4),
                ]
            )

    def _append_waypoint_result(joint_path: list[list[float]], fallback_target: np.ndarray) -> None:
        final_pose = ik_solver.get_fk_matrix(np.array(joint_path[-1]))
        if final_pose is not None:
            position = final_pose[:3, 3].tolist()
            orient_deg = R.from_matrix(final_pose[:3, :3]).as_euler('xyz', degrees=True).tolist()
            waypoint_results.append(
                {
                    "position": [round(p, 4) for p in position],
                    "orientation_euler_deg": [round(o, 2) for o in orient_deg],
                }
            )
        else:
            waypoint_results.append(
                {
                    "position": [round(float(val), 4) for val in fallback_target.tolist()],
                    "orientation_euler_deg": None,
                }
            )

    def _safe_float(raw_value, default_value: float) -> float:
        try:
            value = float(raw_value)
        except Exception:
            value = default_value
        if not np.isfinite(value):
            return default_value
        return value

    def _coerce_orientation_matrix(raw_waypoint) -> np.ndarray | None:
        if not isinstance(raw_waypoint, dict):
            return None
        orientation_raw = raw_waypoint.get("orientation_euler_deg")
        if isinstance(orientation_raw, dict):
            roll_raw = orientation_raw.get("roll", orientation_raw.get("x"))
            pitch_raw = orientation_raw.get("pitch", orientation_raw.get("y"))
            yaw_raw = orientation_raw.get("yaw", orientation_raw.get("z"))
        elif isinstance(orientation_raw, (list, tuple)) and len(orientation_raw) >= 3:
            roll_raw, pitch_raw, yaw_raw = orientation_raw[:3]
        else:
            roll_raw = raw_waypoint.get("rollDeg", raw_waypoint.get("roll_deg"))
            pitch_raw = raw_waypoint.get("pitchDeg", raw_waypoint.get("pitch_deg"))
            yaw_raw = raw_waypoint.get("yawDeg", raw_waypoint.get("yaw_deg"))
        if roll_raw is None or pitch_raw is None or yaw_raw is None:
            return None
        try:
            return R.from_euler(
                "xyz",
                [float(roll_raw), float(pitch_raw), float(yaw_raw)],
                degrees=True,
            ).as_matrix()
        except Exception:
            return None

    def _waypoint_move_type(raw_waypoint) -> str:
        if not isinstance(raw_waypoint, dict):
            return "linear"
        move_type_raw = str(
            raw_waypoint.get("move_type", raw_waypoint.get("moveType", "linear"))
        ).strip().lower()
        return move_type_raw if move_type_raw in {"linear", "joint", "home"} else "linear"

    def _normalize(vec: np.ndarray, fallback: np.ndarray) -> np.ndarray:
        norm = float(np.linalg.norm(vec))
        if norm < 1e-9:
            return fallback.copy()
        return vec / norm

    def _rotate_about_axis(vec: np.ndarray, axis: np.ndarray, angle_rad: float) -> np.ndarray:
        axis_n = _normalize(axis, np.array([0.0, 0.0, 1.0], dtype=float))
        c = float(np.cos(angle_rad))
        s = float(np.sin(angle_rad))
        return (vec * c) + (np.cross(axis_n, vec) * s) + (axis_n * np.dot(axis_n, vec) * (1.0 - c))

    def _resample_polyline(polyline_points: list[np.ndarray], spacing_m: float) -> list[np.ndarray]:
        if len(polyline_points) < 2:
            return list(polyline_points)
        spacing = max(1e-4, float(spacing_m))
        cumulative = [0.0]
        for idx in range(1, len(polyline_points)):
            cumulative.append(
                cumulative[-1]
                + float(np.linalg.norm(polyline_points[idx] - polyline_points[idx - 1]))
            )
        total = cumulative[-1]
        if total < 1e-9:
            return [polyline_points[0], polyline_points[-1]]
        samples = list(np.arange(0.0, total, spacing))
        if not samples or abs(samples[-1] - total) > 1e-6:
            samples.append(total)
        out: list[np.ndarray] = []
        seg_idx = 0
        for s_val in samples:
            while seg_idx < len(cumulative) - 2 and cumulative[seg_idx + 1] < s_val:
                seg_idx += 1
            seg_start = cumulative[seg_idx]
            seg_end = cumulative[seg_idx + 1]
            if seg_end - seg_start < 1e-9:
                out.append(polyline_points[seg_idx].copy())
                continue
            t = (s_val - seg_start) / (seg_end - seg_start)
            out.append(polyline_points[seg_idx] * (1.0 - t) + polyline_points[seg_idx + 1] * t)
        return out

    def _build_weld_orientations(
        path_points: list[np.ndarray],
        start_angles: np.ndarray,
        work_angle_deg: float,
        travel_angle_deg: float,
        spin_angle_deg: float,
        tool_rotation_inv: np.ndarray,
    ) -> list[np.ndarray]:
        start_pose = ik_solver.get_fk_matrix(start_angles.tolist())
        fallback_orientation = (
            np.array(start_pose[:3, :3], dtype=float)
            if start_pose is not None
            else np.identity(3)
        )
        fallback_forward = _normalize(fallback_orientation[:, 0], np.array([1.0, 0.0, 0.0], dtype=float))
        world_up = np.array([0.0, 0.0, 1.0], dtype=float)
        work_rad = np.deg2rad(np.clip(work_angle_deg, -180.0, 180.0))
        travel_rad = np.deg2rad(np.clip(travel_angle_deg, -180.0, 180.0))
        spin_rad = np.deg2rad(np.clip(spin_angle_deg, -180.0, 180.0))
        orientations: list[np.ndarray] = []
        last_forward = fallback_forward.copy()
        for idx, point in enumerate(path_points):
            prev_pt = path_points[idx - 1] if idx > 0 else point
            next_pt = path_points[idx + 1] if idx < len(path_points) - 1 else point
            forward_raw = next_pt - prev_pt
            forward = _normalize(forward_raw, last_forward)
            horizontal_forward = forward - (np.dot(forward, world_up) * world_up)
            horizontal_forward = _normalize(horizontal_forward, forward)
            side_axis = np.cross(world_up, horizontal_forward)
            side_axis = _normalize(side_axis, np.array([1.0, 0.0, 0.0], dtype=float))

            # Base Torch Frame:
            # Z = -world_up (torch points down)
            # X = horizontal_forward (travel)
            # Y = -side_axis (normal, but negated for right-handedness: X x Y = Z)
            z_axis = -world_up.copy()
            x_axis = horizontal_forward.copy()
            y_axis = -side_axis.copy()

            # Apply rigid rotations around the frame axes
            # 1. Work Angle: Rotate around X (Travel). Sweeps Torch in Normal-Up plane.
            z_axis = _rotate_about_axis(z_axis, horizontal_forward, work_rad)
            y_axis = _rotate_about_axis(y_axis, horizontal_forward, work_rad)
            x_axis = _rotate_about_axis(x_axis, horizontal_forward, work_rad)

            # 2. Travel Angle: Rotate around Y (Normal). Sweeps Torch in Travel-Up plane.
            # Using original side_axis for rotation base.
            z_axis = _rotate_about_axis(z_axis, side_axis, travel_rad)
            y_axis = _rotate_about_axis(y_axis, side_axis, travel_rad)
            x_axis = _rotate_about_axis(x_axis, side_axis, travel_rad)

            # 3. Spin Angle: Rotate around Z (Up). Sweeps Torch in Travel-Normal plane.
            # Using original world_up for rotation base.
            z_axis = _rotate_about_axis(z_axis, world_up, spin_rad)
            y_axis = _rotate_about_axis(y_axis, world_up, spin_rad)
            x_axis = _rotate_about_axis(x_axis, world_up, spin_rad)

            z_axis = _normalize(z_axis, -world_up)
            x_axis = _normalize(x_axis, fallback_forward)
            y_axis = _normalize(y_axis, -side_axis)
            
            desired_torch_orientation = np.column_stack((x_axis, y_axis, z_axis))
            # Convert desired world-frame torch pose to required flange/J6 pose
            # using the active tool definition's flange->tool rotation.
            orientation = desired_torch_orientation.dot(tool_rotation_inv)
            orientations.append(orientation)
            last_forward = forward
        return orientations

    print(f"[Pi Trajectory] Planning {len(points)} waypoint(s) for preview '{preview_name}'.")
    weld_points_for_payload: list[list[float]] | None = None
    active_tool_id = "identity"
    active_tool_rotation_inv = np.identity(3, dtype=float)
    try:
        runtime_snapshot = kinematics_runtime.get_runtime_state_snapshot()
        active_tool = runtime_snapshot.get("active_tool")
        if isinstance(active_tool, dict):
            tool_id_token = str(
                active_tool.get("active_tool_id")
                or active_tool.get("tool_id")
                or ""
            ).strip()
            if tool_id_token:
                active_tool_id = tool_id_token
        effective_offset = runtime_snapshot.get("offsets", {}).get("tool_effective")
        if isinstance(effective_offset, dict):
            tool_rot = effective_offset.get("rotation_deg")
            if isinstance(tool_rot, dict):
                active_tool_rotation = R.from_euler(
                    "xyz",
                    [
                        float(tool_rot.get("x", 0.0)),
                        float(tool_rot.get("y", 0.0)),
                        float(tool_rot.get("z", 0.0)),
                    ],
                    degrees=True,
                ).as_matrix()
                active_tool_rotation_inv = np.linalg.inv(active_tool_rotation)
    except Exception:
        active_tool_rotation_inv = np.identity(3, dtype=float)

    if weld_metadata:
        options = weld_metadata.get("options") if isinstance(weld_metadata.get("options"), dict) else {}
        interior_speed = max(0.005, _safe_float(options.get("interior_speed_m_s", plan_velocity), plan_velocity))
        transition_speed = max(0.01, _safe_float(options.get("transition_speed_m_s", plan_velocity), plan_velocity))
        transition_clearance_mm = max(
            1.0,
            _safe_float(options.get("transition_clearance_mm", 35.0), 35.0),
        )
        transition_clearance_m = transition_clearance_mm / 1000.0
        work_angle_deg = _safe_float(options.get("work_angle_deg", 45.0), 45.0)
        travel_angle_deg = _safe_float(options.get("travel_angle_deg", 0.0), 0.0)
        spin_angle_deg = _safe_float(
            options.get("spin_angle_deg", options.get("spinAngleDeg", 0.0)),
            0.0,
        )
        post_action_raw = str(options.get("post_action", "return_to_start")).strip().lower()
        post_action = (
            post_action_raw
            if post_action_raw in {"none", "lift", "return_to_start"}
            else "return_to_start"
        )
        options["post_action"] = post_action
        options["spin_angle_deg"] = spin_angle_deg
        if isinstance(weld_metadata.get("options"), dict):
            weld_metadata["options"]["post_action"] = post_action
            weld_metadata["options"]["spin_angle_deg"] = spin_angle_deg
        else:
            weld_metadata["options"] = {
                "post_action": post_action,
                "spin_angle_deg": spin_angle_deg,
            }
        planned_sections = sections if isinstance(sections, list) and len(sections) > 0 else [
            {"kind": "weld", "points": points, "weld_type": weld_metadata.get("type")}
        ]
        weld_points_for_payload = []
        for section_index, section in enumerate(planned_sections, start=1):
            section_kind = str(section.get("kind", "weld")).strip().lower()
            section_kind = "transition" if section_kind == "transition" else "weld"
            raw_section_points = section.get("points")
            if not isinstance(raw_section_points, list):
                continue
            section_points = [np.array(point, dtype=float) for point in raw_section_points]
            section_points = [point for point in section_points if point.shape[0] >= 3]
            if len(section_points) < 2:
                continue
            weld_points_for_payload.extend(
                [[round(float(v), 4) for v in point.tolist()[:3]] for point in section_points]
            )

            if section_kind == "weld":
                first_target = section_points[0]
                entry_orientation = None
                try:
                    # Use the same weld-angle orientation model for entry approach so
                    # we do not over-constrain entry with a stale orientation lock.
                    entry_orientations = _build_weld_orientations(
                        section_points,
                        current_q,
                        work_angle_deg=work_angle_deg,
                        travel_angle_deg=travel_angle_deg,
                        spin_angle_deg=spin_angle_deg,
                        tool_rotation_inv=active_tool_rotation_inv,
                    )
                    if entry_orientations:
                        entry_orientation = np.asarray(entry_orientations[0], dtype=float)
                except Exception as entry_orientation_error:
                    print(
                        "[Pi Trajectory] WARNING: Failed to derive weld-entry orientation "
                        f"for section #{section_index}: {entry_orientation_error}"
                    )
                current_pos = ik_solver.get_fk(current_q.tolist())
                if current_pos is None or np.linalg.norm(np.array(current_pos) - first_target) > 1e-4:
                    joint_path = trajectory_execution._plan_linear_move(
                        current_q,
                        first_target,
                        float(plan_velocity),
                        float(plan_acceleration),
                        100,
                        True,
                        forced_orientation=entry_orientation,
                    )
                    if not joint_path and entry_orientation is not None:
                        warning = (
                            f"Weld entry orientation interpolation failed for section #{section_index}; "
                            f"tool={active_tool_id}; retrying with orientation lock."
                        )
                        print(f"[Pi Trajectory] WARNING: {warning}")
                        planning_warnings.append(warning)
                        joint_path = trajectory_execution._plan_linear_move(
                            current_q,
                            first_target,
                            float(plan_velocity),
                            float(plan_acceleration),
                            100,
                            True,
                            forced_orientation=None,
                        )
                    if not joint_path:
                        print(f"[Pi Trajectory] ERROR: Failed weld entry for section #{section_index} -> {np.round(first_target, 4)}.")
                        planning_succeeded = False
                        failed_waypoint_idx = section_index
                        failed_waypoint_target = first_target
                        failed_segment_start_fk = (
                            np.array(current_pos, dtype=float)
                            if current_pos is not None
                            else None
                        )
                        break
                    planned_steps.append({"type": "move", "path": joint_path, "freq": 100, "weld_active": False})
                    _append_cartesian_samples(joint_path)
                    _append_waypoint_result(joint_path, first_target)
                    current_q = np.array(joint_path[-1], dtype=float)

                interior_points = _resample_polyline(section_points, interior_speed / 100.0)
                current_pos = ik_solver.get_fk(current_q.tolist())
                if (
                    current_pos is not None
                    and len(interior_points) > 1
                    and np.linalg.norm(interior_points[0] - np.array(current_pos)) < 1e-4
                ):
                    interior_points = interior_points[1:]
                if len(interior_points) == 0:
                    continue
                if len(interior_points) == 1:
                    interior_points.append(section_points[-1].copy())
                interior_orientations = _build_weld_orientations(
                    interior_points,
                    current_q,
                    work_angle_deg=work_angle_deg,
                    travel_angle_deg=travel_angle_deg,
                    spin_angle_deg=spin_angle_deg,
                    tool_rotation_inv=active_tool_rotation_inv,
                )
                joint_path = trajectory_execution._plan_high_fidelity_trajectory(
                    cartesian_points=[point.tolist()[:3] for point in interior_points],
                    start_q=current_q,
                    use_smoothing=True,
                    orientations_list=interior_orientations,
                )
                if not joint_path:
                    warning = (
                        f"Torch-angle orientations failed for weld section #{section_index}; "
                        f"tool={active_tool_id}; retrying with orientation lock."
                    )
                    print(f"[Pi Trajectory] WARNING: {warning}")
                    planning_warnings.append(warning)
                    joint_path = trajectory_execution._plan_high_fidelity_trajectory(
                        cartesian_points=[point.tolist()[:3] for point in interior_points],
                        start_q=current_q,
                        use_smoothing=True,
                        forced_orientation=None,
                        orientations_list=None,
                    )
                if not joint_path:
                    print(f"[Pi Trajectory] ERROR: Failed weld interior for section #{section_index}.")
                    planning_succeeded = False
                    failed_waypoint_idx = section_index
                    failed_waypoint_target = interior_points[-1]
                    failed_segment_start_fk = (
                        np.array(current_pos, dtype=float)
                        if current_pos is not None
                        else None
                    )
                    break
                planned_steps.append({"type": "move", "path": joint_path, "freq": 100, "weld_active": True})
                _append_cartesian_samples(joint_path)
                _append_waypoint_result(joint_path, interior_points[-1])
                current_q = np.array(joint_path[-1], dtype=float)
            else:
                for point_index, target_pos in enumerate(section_points, start=1):
                    current_pos = ik_solver.get_fk(current_q.tolist())
                    if current_pos is not None and np.linalg.norm(np.array(current_pos) - target_pos) < 1e-5:
                        continue
                    segment_start_fk = np.array(current_pos, dtype=float) if current_pos is not None else None
                    joint_path = trajectory_execution._plan_linear_move(
                        current_q,
                        target_pos,
                        float(transition_speed),
                        float(plan_acceleration),
                        100,
                        True,
                        forced_orientation=None,
                    )
                    if not joint_path:
                        print(
                            f"[Pi Trajectory] ERROR: Failed transition point #{point_index} in section #{section_index} -> {np.round(target_pos, 4)}."
                        )
                        planning_succeeded = False
                        failed_waypoint_idx = point_index
                        failed_waypoint_target = target_pos
                        failed_segment_start_fk = segment_start_fk
                        break
                    planned_steps.append({"type": "move", "path": joint_path, "freq": 100, "weld_active": False})
                    _append_cartesian_samples(joint_path)
                    _append_waypoint_result(joint_path, target_pos)
                    current_q = np.array(joint_path[-1], dtype=float)
                if not planning_succeeded:
                    break
        if planning_succeeded:
            def _plan_post_action_target(target_pos: np.ndarray, label: str) -> bool:
                nonlocal current_q

                current_pos_local = ik_solver.get_fk(current_q.tolist())
                if (
                    current_pos_local is not None
                    and np.linalg.norm(np.array(current_pos_local) - target_pos) < 1e-5
                ):
                    return True
                segment_start_fk = (
                    np.array(current_pos_local, dtype=float)
                    if current_pos_local is not None
                    else None
                )
                joint_path = trajectory_execution._plan_linear_move(
                    current_q,
                    target_pos,
                    float(transition_speed),
                    float(plan_acceleration),
                    100,
                    True,
                    forced_orientation=None,
                )
                if not joint_path:
                    warning = (
                        f"Post-action transition skipped ({label}) target="
                        f"{[round(float(v), 4) for v in target_pos.tolist()[:3]]}"
                    )
                    print(f"[Pi Trajectory] WARNING: {warning}.")
                    planning_warnings.append(warning)
                    return False
                planned_steps.append(
                    {"type": "move", "path": joint_path, "freq": 100, "weld_active": False}
                )
                _append_cartesian_samples(joint_path)
                _append_waypoint_result(joint_path, target_pos)
                current_q = np.array(joint_path[-1], dtype=float)
                return True

            end_fk = ik_solver.get_fk(current_q.tolist())
            end_pos = np.array(end_fk, dtype=float) if end_fk is not None else None
            if post_action == "lift" and end_pos is not None:
                lift_target = np.array(
                    [end_pos[0], end_pos[1], end_pos[2] + transition_clearance_m],
                    dtype=float,
                )
                _plan_post_action_target(lift_target, "lift")
            elif (
                post_action == "return_to_start"
                and end_pos is not None
                and trajectory_start_pos is not None
                and np.linalg.norm(end_pos - trajectory_start_pos) >= 1e-4
            ):
                lift_z = max(float(end_pos[2]), float(trajectory_start_pos[2])) + transition_clearance_m
                return_points = [
                    np.array([end_pos[0], end_pos[1], lift_z], dtype=float),
                    np.array([trajectory_start_pos[0], trajectory_start_pos[1], lift_z], dtype=float),
                    trajectory_start_pos.copy(),
                ]
                for idx, target in enumerate(return_points, start=1):
                    if not _plan_post_action_target(target, f"return_to_start:{idx}"):
                        break
    else:
        for idx, waypoint in enumerate(points, start=1):
            target_pos = np.array(waypoint, dtype=float)
            segment_start_fk = ik_solver.get_fk(current_q.tolist())
            start_q_for_step = np.array(current_q, dtype=float)
            authored_waypoint = (
                standard_pose_waypoints[idx - 1]
                if standard_pose_waypoints is not None
                else None
            )
            forced_orientation = (
                _coerce_orientation_matrix(authored_waypoint)
                if standard_pose_waypoints is not None
                else None
            )
            move_type = _waypoint_move_type(authored_waypoint)
            linear_speed_m_s = _extract_waypoint_linear_speed_m_s(authored_waypoint)
            linear_acceleration_m_s2 = _extract_waypoint_linear_acceleration_m_s2(authored_waypoint)
            rotation_speed_deg_s = _extract_waypoint_rotation_speed_deg_s(authored_waypoint)
            _, resolved_linear_speed_m_s, resolved_linear_acceleration_m_s2 = (
                _resolve_profile_params_for_linear_speed_m_s(
                    linear_speed_m_s if linear_speed_m_s is not None else plan_velocity,
                    linear_acceleration_m_s2,
                )
            )
            actual_rotation_speed_deg_s: float | None = None
            t_start = time.monotonic()
            if move_type == "home":
                joint_path, solved_q = _plan_joint_move_to_pose(
                    current_q,
                    target_q=[0.0] * utils.NUM_LOGICAL_JOINTS,
                    max_joint_deg_s=rotation_speed_deg_s,
                )
                actual_rotation_speed_deg_s = (
                    float(rotation_speed_deg_s)
                    if rotation_speed_deg_s is not None
                    else _estimate_joint_path_rotation_speed_deg_s(
                        start_q_for_step,
                        joint_path,
                        frequency_hz=_SAFE_JOINT_MOVE_FREQUENCY_HZ,
                    )
                )
                serialization_command = "home"
                serialization_freq = _SAFE_JOINT_MOVE_FREQUENCY_HZ
                fallback_target = (
                    np.array(ik_solver.get_fk([0.0] * utils.NUM_LOGICAL_JOINTS), dtype=float)
                    if ik_solver.get_fk([0.0] * utils.NUM_LOGICAL_JOINTS) is not None
                    else target_pos
                )
            elif move_type == "joint":
                joint_path, solved_q = _plan_joint_move_to_pose(
                    current_q,
                    target_pos=target_pos,
                    target_orientation=forced_orientation,
                    max_joint_deg_s=rotation_speed_deg_s,
                )
                actual_rotation_speed_deg_s = (
                    float(rotation_speed_deg_s)
                    if rotation_speed_deg_s is not None
                    else _estimate_joint_path_rotation_speed_deg_s(
                        start_q_for_step,
                        joint_path,
                        frequency_hz=_SAFE_JOINT_MOVE_FREQUENCY_HZ,
                    )
                )
                serialization_command = "move"
                serialization_freq = _SAFE_JOINT_MOVE_FREQUENCY_HZ
                fallback_target = target_pos
            else:
                current_position = (
                    np.array(segment_start_fk, dtype=float) if segment_start_fk is not None else None
                )
                is_pure_rotation = (
                    current_position is not None
                    and forced_orientation is not None
                    and np.linalg.norm(current_position - target_pos) <= 1e-5
                )
                if is_pure_rotation and forced_orientation is not None:
                    current_pose_matrix = ik_solver.get_fk_matrix(current_q.tolist())
                    current_orientation = (
                        np.asarray(current_pose_matrix[:3, :3], dtype=float)
                        if current_pose_matrix is not None
                        else None
                    )
                    angle_deg = 0.0
                    if current_orientation is not None:
                        rotation_delta = (
                            R.from_matrix(current_orientation).inv() * R.from_matrix(forced_orientation)
                        )
                        angle_deg = abs(float(np.rad2deg(rotation_delta.magnitude())))
                    actual_rotation_speed_deg_s = (
                        float(rotation_speed_deg_s)
                        if rotation_speed_deg_s is not None
                        else _resolve_default_orientation_speed_deg_s(angle_deg)
                    )
                    joint_path = _plan_orientation_only_move(
                        current_q,
                        target_pos=target_pos,
                        target_orientation=forced_orientation,
                        angular_speed_deg_s=actual_rotation_speed_deg_s,
                        frequency_hz=100,
                    )
                else:
                    joint_path = trajectory_execution._plan_linear_move(
                        current_q,
                        target_pos,
                        resolved_linear_speed_m_s,
                        resolved_linear_acceleration_m_s2,
                        100,
                        True,
                        forced_orientation=forced_orientation,
                    )
                solved_q = list(map(float, joint_path[-1])) if joint_path else None
                serialization_command = "move_absolute"
                serialization_freq = 100
                fallback_target = target_pos
            if not joint_path:
                print(f"[Pi Trajectory] ERROR: Failed to plan waypoint #{idx} -> {np.round(target_pos, 4)}.")
                planning_succeeded = False
                failed_waypoint_idx = idx
                failed_waypoint_target = target_pos
                failed_segment_start_fk = (
                    np.array(segment_start_fk, dtype=float)
                    if segment_start_fk is not None
                    else None
                )
                break

            planned_step = {
                "type": "move",
                "path": joint_path,
                "freq": serialization_freq,
                "serialization_command": serialization_command,
            }
            if serialization_command == "move_absolute":
                planned_step["linear_speed_mm_s"] = round(float(resolved_linear_speed_m_s * 1000.0), 3)
                planned_step["linear_acceleration_mm_s2"] = round(
                    float(resolved_linear_acceleration_m_s2 * 1000.0),
                    3,
                )
            if actual_rotation_speed_deg_s is not None:
                planned_step["rotation_speed_deg_s"] = round(float(actual_rotation_speed_deg_s), 3)
            planned_steps.append(planned_step)
            _append_cartesian_samples(joint_path)
            _append_waypoint_result(joint_path, fallback_target)

            current_q = np.array(solved_q if solved_q is not None else joint_path[-1], dtype=float)
            t_end = time.monotonic()
            print(
                f"[Pi Trajectory] Planned waypoint #{idx} ({move_type}) -> {np.round(target_pos, 4)} in {(t_end - t_start) * 1000:.2f} ms"
            )

    if not planning_succeeded or len(planned_steps) == 0:
        planner_diag_suffix = ""
        planner_diag = utils.trajectory_state.get("last_planner_diagnostics")
        if isinstance(planner_diag, dict):
            diag_parts: list[str] = []
            diag_reason = str(planner_diag.get("reason_code", "")).strip()
            if diag_reason:
                diag_parts.append(f"reason={diag_reason}")
            diag_attempt = str(planner_diag.get("attempt", "")).strip()
            if diag_attempt:
                diag_parts.append(f"attempt={diag_attempt}")
            fallback_level = planner_diag.get("fallback_level")
            if isinstance(fallback_level, (int, float)):
                diag_parts.append(f"fallback_level={int(fallback_level)}")
            residuals = planner_diag.get("residuals")
            if isinstance(residuals, dict):
                for key in (
                    "joint_limit_margin_rad",
                    "violating_joint_margin_rad",
                    "max_joint_step_rad",
                    "cartesian_residual_m",
                    "orientation_residual_deg",
                ):
                    value = residuals.get(key)
                    try:
                        numeric_value = float(value)
                    except (TypeError, ValueError):
                        continue
                    if np.isfinite(numeric_value):
                        diag_parts.append(f"{key}={numeric_value:.5f}")
                for key in (
                    "violating_joint_index",
                    "violating_pose_index",
                    "violating_joint_count",
                    "jump_pose_index",
                    "jump_joint_index",
                ):
                    value = residuals.get(key)
                    try:
                        numeric_value = int(float(value))
                    except (TypeError, ValueError):
                        continue
                    diag_parts.append(f"{key}={numeric_value}")
                step_source = str(residuals.get("step_source", "")).strip()
                if step_source:
                    diag_parts.append(f"step_source={step_source}")
                jump_context = str(residuals.get("jump_context", "")).strip()
                if jump_context:
                    diag_parts.append(f"jump_context={jump_context}")
                for key in (
                    "jump_joint_previous_rad",
                    "jump_joint_current_rad",
                    "jump_joint_raw_step_rad",
                    "jump_joint_wrapped_step_rad",
                ):
                    value = residuals.get(key)
                    try:
                        numeric_value = float(value)
                    except (TypeError, ValueError):
                        continue
                    if np.isfinite(numeric_value):
                        diag_parts.append(f"{key}={numeric_value:.5f}")
            if diag_parts:
                planner_diag_suffix = " planner_diag={" + ", ".join(diag_parts) + "}"
        if failed_waypoint_idx is not None and failed_waypoint_target is not None:
            failed_target = [round(float(v), 4) for v in failed_waypoint_target.tolist()]
            if failed_segment_start_fk is not None and failed_segment_start_fk.shape[0] >= 3:
                start_fk = [round(float(v), 4) for v in failed_segment_start_fk[:3].tolist()]
                raise RuntimeError(
                    f"Planning failed at waypoint #{failed_waypoint_idx} target={failed_target} "
                    f"from start_fk={start_fk}.{planner_diag_suffix}"
                )
            raise RuntimeError(
                f"Planning failed at waypoint #{failed_waypoint_idx} target={failed_target}.{planner_diag_suffix}"
            )
        raise RuntimeError(f"Planning failed for one or more waypoints.{planner_diag_suffix}")

    runtime_planned_steps = list(planned_steps)
    if standard_pose_waypoints is not None and not weld_metadata:
        runtime_planned_steps = []
        for index, step in enumerate(planned_steps):
            runtime_planned_steps.append(step)
            if step.get("type") != "move":
                continue
            pause_after_s = (
                _extract_waypoint_pause_after_s(standard_pose_waypoints[index])
                if index < len(standard_pose_waypoints)
                else None
            )
            if pause_after_s is not None and index < len(planned_steps) - 1:
                runtime_planned_steps.append(
                    {"type": "pause", "duration": round(float(pause_after_s), 3)}
                )

    # Build recorded trajectory representation from planned steps.
    moves = []
    move_counter = 0
    for step in runtime_planned_steps:
        if step.get("type") == "pause":
            moves.append({"command": "pause", "duration": float(step.get("duration", 1.0))})
            continue
        if step.get("type") != "move":
            continue
        step_path = step.get("path") or []
        if not step_path:
            continue
        final_pose = ik_solver.get_fk_matrix(np.array(step_path[-1]))
        if final_pose is not None:
            position = [round(float(p), 4) for p in final_pose[:3, 3].tolist()]
            orient_deg = R.from_matrix(final_pose[:3, :3]).as_euler('xyz', degrees=True).tolist()
            orient_payload = [round(float(o), 2) for o in orient_deg]
        else:
            position = [round(float(v), 4) for v in np.array(step_path[-1], dtype=float).tolist()[:3]]
            orient_payload = None
        serialization_command = str(step.get("serialization_command", "move_absolute")).strip() or "move_absolute"
        move = {
            "command": serialization_command,
        }
        if serialization_command in {"move_absolute", "move", "home"}:
            move["vector"] = position
        if orient_payload is not None and serialization_command in {"move_absolute", "move", "home"}:
            move["orientation_euler_deg"] = orient_payload
        linear_speed_mm_s = _coerce_optional_positive_speed(step.get("linear_speed_mm_s"))
        if linear_speed_mm_s is not None:
            move["linear_speed_mm_s"] = round(float(linear_speed_mm_s), 3)
        linear_acceleration_mm_s2 = _coerce_optional_positive_speed(step.get("linear_acceleration_mm_s2"))
        if linear_acceleration_mm_s2 is not None:
            move["linear_acceleration_mm_s2"] = round(float(linear_acceleration_mm_s2), 3)
        rotation_speed_deg_s = _coerce_optional_positive_speed(step.get("rotation_speed_deg_s"))
        if rotation_speed_deg_s is not None:
            move["rotation_speed_deg_s"] = round(float(rotation_speed_deg_s), 3)
        if bool(step.get("weld_active", False)):
            move["is_weld"] = True
            if weld_metadata:
                move["weld_type"] = weld_metadata.get("type")
        moves.append(move)
        move_counter += 1

    traj_dict = {
        "description": description,
        "loop": False,
        "orientation_euler_angles_deg": None,
        "moves": moves,
    }
    if planning_warnings:
        traj_dict["planning_warnings"] = planning_warnings
    if weld_metadata:
        traj_dict["weld"] = weld_metadata

    _ensure_record_dir_exists()
    preview_path = os.path.join(RECORDED_TRAJ_DIR, f"{preview_name}.json")
    with open(preview_path, "w") as f:
        json.dump(traj_dict, f, indent=2)
    print(f"[Pi Trajectory] Preview trajectory saved to {preview_path}")
    try:
        os.makedirs(utils.TRAJECTORY_CACHE_DIR, exist_ok=True)
        cache_path = os.path.join(utils.TRAJECTORY_CACHE_DIR, f"{preview_name}.json")
        with open(cache_path, "w") as f:
            json.dump(utils._convert_numpy_to_list(runtime_planned_steps), f, indent=2)
        print(f"[Pi Trajectory] Preview planned-steps cache saved to {cache_path}")
    except Exception as e:
        print(f"[Pi Trajectory] WARNING: Failed to save preview planned-steps cache: {e}")

    payload = {
        "name": preview_name,
        "trajectory": traj_dict,
        "cartesian_path": cartesian_samples,
        "waypoints": (
            weld_points_for_payload
            if weld_points_for_payload
            else [
                {
                    "x": round(float(item["position"][0]), 4),
                    "y": round(float(item["position"][1]), 4),
                    "z": round(float(item["position"][2]), 4),
                    "move_type": (
                        _waypoint_move_type(standard_pose_waypoints[index])
                        if standard_pose_waypoints is not None and index < len(standard_pose_waypoints)
                        else "linear"
                    ),
                    "linear_speed_mm_s": (
                        round(
                            float(
                                _coerce_optional_positive_speed(
                                    planned_steps[index].get("linear_speed_mm_s") if index < len(planned_steps) else None
                                )
                            ),
                            3,
                        )
                        if index < len(planned_steps)
                        and _coerce_optional_positive_speed(planned_steps[index].get("linear_speed_mm_s")) is not None
                        else None
                    ),
                    "linear_acceleration_mm_s2": (
                        round(
                            float(
                                _coerce_optional_positive_speed(
                                    planned_steps[index].get("linear_acceleration_mm_s2")
                                    if index < len(planned_steps)
                                    else None
                                )
                            ),
                            3,
                        )
                        if index < len(planned_steps)
                        and _coerce_optional_positive_speed(
                            planned_steps[index].get("linear_acceleration_mm_s2")
                        ) is not None
                        else None
                    ),
                    "rotation_speed_deg_s": (
                        round(
                            float(
                                _coerce_optional_positive_speed(
                                    planned_steps[index].get("rotation_speed_deg_s") if index < len(planned_steps) else None
                                )
                            ),
                            3,
                        )
                        if index < len(planned_steps)
                        and _coerce_optional_positive_speed(planned_steps[index].get("rotation_speed_deg_s")) is not None
                        else None
                    ),
                    "pause_after_s": (
                        round(
                            float(
                                _extract_waypoint_pause_after_s(standard_pose_waypoints[index])
                            ),
                            3,
                        )
                        if standard_pose_waypoints is not None
                        and index < len(standard_pose_waypoints)
                        and _extract_waypoint_pause_after_s(standard_pose_waypoints[index]) is not None
                        else None
                    ),
                    "orientation_euler_deg": (
                        {
                            "roll": round(float(item["orientation_euler_deg"][0]), 2),
                            "pitch": round(float(item["orientation_euler_deg"][1]), 2),
                            "yaw": round(float(item["orientation_euler_deg"][2]), 2),
                        }
                        if isinstance(item.get("orientation_euler_deg"), list)
                        and len(item["orientation_euler_deg"]) >= 3
                        else None
                    ),
                }
                for index, item in enumerate(waypoint_results)
            ]
        ),
        "file_path": preview_path,
        "step_summaries": [
            {"type": step.get("type"), "freq": step.get("freq"), "points": len(step.get("path", []))}
            for step in runtime_planned_steps
        ],
    }
    planner_diag = utils.trajectory_state.get("last_planner_diagnostics")
    if isinstance(planner_diag, dict):
        payload["planner_diagnostics"] = planner_diag
    if planning_warnings:
        payload["planning_warnings"] = planning_warnings
    if weld_metadata:
        weld_preview_joint_pose = None
        for planned_step in planned_steps:
            if planned_step.get("type") != "move" or not bool(planned_step.get("weld_active", False)):
                continue
            step_path = planned_step.get("path") or []
            if not step_path:
                continue
            sample_index = max(0, min(len(step_path) - 1, len(step_path) // 3))
            joint_sample = np.array(step_path[sample_index], dtype=float).tolist()
            weld_preview_joint_pose = [float(value) for value in joint_sample]
            break
        if weld_preview_joint_pose is not None:
            payload["weld_preview_joint_pose"] = weld_preview_joint_pose
        payload["weld"] = weld_metadata
    return payload


def plan_preview_trajectory_points(
    points: list[list[float]] | list[tuple[float, float, float]],
    *,
    preview_name: str = PLANNED_PREVIEW_NAME,
    weld_metadata: dict | None = None,
    sections: list[dict] | None = None,
    pose_waypoints: list[dict] | None = None,
) -> dict:
    description = (
        f"Planned on {datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S')} via "
        + ("PLAN_WELD" if weld_metadata else "PLAN_TRAJECTORY_POINTS")
    )
    return _plan_preview_points_payload(
        points,
        preview_name=preview_name,
        description=description,
        weld_metadata=weld_metadata,
        sections=sections,
        pose_waypoints=pose_waypoints,
    )


def handle_plan_trajectory_points(points, sock, addr):
    """
    Plan a Cartesian trajectory for a list of way-points and return the joint-space path
    without executing it. The resulting trajectory is written to the recorded_trajectories
    directory under a well-known name so it can be executed with RUN_TRAJECTORY.
    """
    if len(points) == 0:
        print("[Pi Trajectory] ERROR: PLAN_TRAJECTORY_POINTS requires at least one waypoint.")
        try:
            sock.sendto("ERROR,PLAN_TRAJECTORY_POINTS,NO_POINTS".encode("utf-8"), addr)
        except Exception as e:
            print(f"[Pi Trajectory] WARNING: Failed to send PLAN_TRAJECTORY_POINTS error response: {e}")
        return

    try:
        payload = plan_preview_trajectory_points(points, preview_name=PLANNED_PREVIEW_NAME)
    except Exception:
        try:
            sock.sendto("ERROR,PLAN_TRAJECTORY_POINTS,PLANNING_FAILED".encode("utf-8"), addr)
        except Exception as e:
            print(f"[Pi Trajectory] WARNING: Failed to send PLAN_TRAJECTORY_POINTS failure response: {e}")
        return

    message = "PLANNED_TRAJECTORY_POINTS," + json.dumps(payload)
    try:
        encoded = message.encode("utf-8")
        sock.sendto(encoded, addr)
    except Exception as e:
        print(
            "[Pi Trajectory] WARNING: Failed to send PLAN_TRAJECTORY_POINTS result "
            f"({len(message)} chars): {e}"
        )


# -----------------------------------------------------------------------------
# Utility: load trajectory file (default + recorded)
# -----------------------------------------------------------------------------

def _load_trajectory_by_name(name: str):
    """Return trajectory dict by checking recorded_trajectories first, then trajectories.json."""
    # 1) Recorded folder
    recorded_path = os.path.join(RECORDED_TRAJ_DIR, f"{name}.json")
    if os.path.exists(recorded_path):
        try:
            with open(recorded_path, "r") as f:
                print(f"[Pi Trajectory] Loading recorded trajectory: {recorded_path}")
                return json.load(f)
        except Exception as e:
            print(f"[Pi Trajectory] ERROR: Could not load recorded trajectory {recorded_path}: {e}")

    # 2) Built-in trajectories.json
    fallback_path = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", "trajectories.json"))
    try:
        with open(fallback_path, "r") as f:
            all_traj = json.load(f)
            return all_traj.get(name)
    except Exception as e:
        print(f"[Pi Trajectory] ERROR: Could not load fallback trajectories.json: {e}")

    return None


def get_motion_state() -> str:
    """
    Best-effort motion state snapshot for safety gating.

    The richer state machine is tracked elsewhere; this helper provides a strict
    idle-vs-active gate for runtime kinematics changes.
    """
    state = utils.get_motion_state()
    if state != "IDLE":
        return state
    if bool(utils.trajectory_state_get("is_jogging", False)):
        return "EXECUTING"
    return "IDLE"


def handle_get_kinematics_profile() -> dict:
    return kinematics_runtime.get_runtime_state_snapshot()


def handle_patch_runtime_offsets(payload: dict, expected_revision: int | None = None) -> dict:
    result = kinematics_runtime.patch_runtime_offsets(
        payload,
        expected_revision=expected_revision,
        motion_state=get_motion_state(),
    )
    utils.append_audit(
        "kinematics_patch",
        expected_revision=expected_revision,
        applied_revision=result.get("revision"),
    )
    return result


def handle_reset_runtime_offsets(expected_revision: int | None = None) -> dict:
    result = kinematics_runtime.reset_runtime_offsets(
        expected_revision=expected_revision,
        motion_state=get_motion_state(),
    )
    utils.append_audit(
        "kinematics_reset",
        expected_revision=expected_revision,
        applied_revision=result.get("revision"),
    )
    return result


def handle_apply_kinematics_profile(payload: dict, expected_revision: int | None = None) -> dict:
    backend_name = ik_solver.get_backend_name()
    result = kinematics_runtime.apply_profile_payload(
        payload,
        expected_revision=expected_revision,
        motion_state=get_motion_state(),
        backend_name=backend_name,
    )
    utils.append_audit(
        "kinematics_apply_profile",
        expected_revision=expected_revision,
        applied_revision=result.get("revision"),
        profile_id=result.get("profile", {}).get("profile_id"),
    )
    return result
