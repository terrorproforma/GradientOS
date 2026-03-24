from __future__ import annotations

import threading
import time
import uuid
from dataclasses import dataclass, field
from enum import Enum
from typing import Any


class JogSessionError(RuntimeError):
    def __init__(self, code: str, message: str, *, payload: dict[str, Any] | None = None) -> None:
        super().__init__(message)
        self.code = str(code)
        self.payload = dict(payload or {})


class JogSessionState(str, Enum):
    IDLE = "idle"
    ACTIVE = "active"
    PAUSED_FOR_MOTION = "paused_for_motion"
    STOPPED = "stopped"
    EXPIRED = "expired"


_TERMINAL_STATES = {JogSessionState.STOPPED, JogSessionState.EXPIRED}
_MATRIX3_SIZE = 3


def _zero_vector() -> tuple[float, float, float, float, float, float]:
    return (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)


def _normalize_position_vector(position: list[float] | tuple[float, ...]) -> tuple[float, float, float]:
    if len(position) != 3:
        raise JogSessionError("INVALID_POSITION", "Jog commanded position must contain 3 values.")
    try:
        return (
            float(position[0]),
            float(position[1]),
            float(position[2]),
        )
    except Exception as exc:
        raise JogSessionError("INVALID_POSITION", "Jog commanded position must be numeric.") from exc


def _normalize_joint_vector(joint_vector: list[float] | tuple[float, ...]) -> tuple[float, ...]:
    if len(joint_vector) == 0:
        raise JogSessionError("INVALID_JOINT_VECTOR", "Jog commanded joints must not be empty.")
    try:
        return tuple(float(value) for value in joint_vector)
    except Exception as exc:
        raise JogSessionError("INVALID_JOINT_VECTOR", "Jog commanded joints must be numeric.") from exc


def _normalize_orientation_matrix(
    orientation_matrix: list[list[float]] | tuple[tuple[float, ...], ...],
) -> tuple[tuple[float, float, float], tuple[float, float, float], tuple[float, float, float]]:
    if len(orientation_matrix) != _MATRIX3_SIZE:
        raise JogSessionError("INVALID_ORIENTATION", "Jog commanded orientation must be a 3x3 matrix.")
    rows: list[tuple[float, float, float]] = []
    try:
        for row in orientation_matrix:
            if len(row) != _MATRIX3_SIZE:
                raise JogSessionError("INVALID_ORIENTATION", "Jog commanded orientation must be a 3x3 matrix.")
            rows.append((float(row[0]), float(row[1]), float(row[2])))
    except JogSessionError:
        raise
    except Exception as exc:
        raise JogSessionError("INVALID_ORIENTATION", "Jog commanded orientation must be numeric.") from exc
    return (rows[0], rows[1], rows[2])


def _json_safe_copy(value: Any) -> Any:
    if isinstance(value, dict):
        return {str(key): _json_safe_copy(item) for key, item in value.items()}
    if isinstance(value, tuple):
        return [_json_safe_copy(item) for item in value]
    if isinstance(value, list):
        return [_json_safe_copy(item) for item in value]
    return value


@dataclass
class JogSessionRecord:
    session_id: str
    owner_id: str
    state: JogSessionState = JogSessionState.ACTIVE
    created_monotonic: float = field(default_factory=time.monotonic)
    updated_monotonic: float = field(default_factory=time.monotonic)
    lease_timeout_s: float = 0.4
    lease_deadline_monotonic: float = 0.0
    deadman: bool = False
    velocity_vector: tuple[float, float, float, float, float, float] = field(default_factory=_zero_vector)
    gripper_velocity_deg_s: float = 0.0
    last_seq_received: int = -1
    last_seq_applied: int = -1
    lease_expiry_count: int = 0
    pause_for_motion_count: int = 0
    stale_packet_rejects: int = 0
    owner_conflict_rejects: int = 0
    last_stop_reason: str | None = None
    backend_mode: str = "controller_cartesian_loop"
    backend_timeout_s: float | None = None
    commanded_position_m: tuple[float, float, float] | None = None
    commanded_orientation_matrix: (
        tuple[tuple[float, float, float], tuple[float, float, float], tuple[float, float, float]] | None
    ) = None
    commanded_joint_vector: tuple[float, ...] | None = None
    last_accepted_target_position_m: tuple[float, float, float] | None = None
    last_accepted_target_orientation_matrix: (
        tuple[tuple[float, float, float], tuple[float, float, float], tuple[float, float, float]] | None
    ) = None
    last_resync_reason: str | None = None
    last_resync_monotonic: float | None = None
    following_error_snapshot: dict[str, Any] | None = None
    last_gate_failure_reason: str | None = None
    last_gate_failure_details: dict[str, Any] | None = None

    def is_live(self) -> bool:
        return self.state not in _TERMINAL_STATES


class JogSessionManager:
    def __init__(self) -> None:
        self._lock = threading.RLock()
        self._record: JogSessionRecord | None = None

    def start_session(
        self,
        *,
        owner_id: str,
        seq: int,
        lease_timeout_s: float,
        deadman: bool,
        velocity_vector: tuple[float, float, float, float, float, float] | list[float],
        gripper_velocity_deg_s: float = 0.0,
        backend_mode: str = "controller_cartesian_loop",
        backend_timeout_s: float | None = None,
        session_id: str | None = None,
    ) -> dict[str, Any]:
        owner = self._normalize_owner(owner_id)
        seq_value = self._normalize_seq(seq)
        lease_timeout = self._normalize_timeout(lease_timeout_s)
        vector = self._normalize_vector(velocity_vector)
        now = time.monotonic()
        with self._lock:
            current = self._record
            if current is not None and current.is_live():
                if current.owner_id != owner:
                    current.owner_conflict_rejects += 1
                    raise JogSessionError(
                        "OWNER_CONFLICT",
                        "Another jog session owner is already active.",
                        payload={"active_session_id": current.session_id, "active_owner_id": current.owner_id},
                    )
                raise JogSessionError(
                    "SESSION_ALREADY_ACTIVE",
                    "A jog session is already active for this owner.",
                    payload={"session_id": current.session_id},
                )
            record = JogSessionRecord(
                session_id=session_id.strip() if isinstance(session_id, str) and session_id.strip() else uuid.uuid4().hex,
                owner_id=owner,
                state=JogSessionState.ACTIVE,
                created_monotonic=now,
                updated_monotonic=now,
                lease_timeout_s=lease_timeout,
                lease_deadline_monotonic=now + lease_timeout,
                deadman=bool(deadman),
                velocity_vector=vector if deadman else _zero_vector(),
                gripper_velocity_deg_s=float(gripper_velocity_deg_s),
                last_seq_received=seq_value,
                last_seq_applied=-1,
                last_stop_reason=None,
                backend_mode=str(backend_mode or "controller_cartesian_loop"),
                backend_timeout_s=self._normalize_optional_timeout(backend_timeout_s),
            )
            self._record = record
            return self._snapshot_locked(now)

    def update_session(
        self,
        *,
        session_id: str,
        owner_id: str,
        seq: int,
        lease_timeout_s: float | None,
        deadman: bool,
        velocity_vector: tuple[float, float, float, float, float, float] | list[float],
        gripper_velocity_deg_s: float = 0.0,
        backend_mode: str | None = None,
        backend_timeout_s: float | None = None,
    ) -> dict[str, Any]:
        owner = self._normalize_owner(owner_id)
        seq_value = self._normalize_seq(seq)
        vector = self._normalize_vector(velocity_vector)
        now = time.monotonic()
        with self._lock:
            record = self._require_matching_live_record(session_id=session_id, owner_id=owner)
            if record.lease_deadline_monotonic > 0.0 and now > record.lease_deadline_monotonic:
                self._expire_locked(record, reason="lease-expired")
                raise JogSessionError("SESSION_EXPIRED", "Jog session lease expired.", payload={"session_id": record.session_id})
            if seq_value <= int(record.last_seq_received):
                record.stale_packet_rejects += 1
                raise JogSessionError(
                    "STALE_SEQUENCE",
                    "Jog session update sequence is stale.",
                    payload={"session_id": record.session_id, "last_seq_received": int(record.last_seq_received)},
                )
            if lease_timeout_s is not None:
                record.lease_timeout_s = self._normalize_timeout(lease_timeout_s)
            record.updated_monotonic = now
            record.lease_deadline_monotonic = now + float(record.lease_timeout_s)
            record.deadman = bool(deadman)
            record.velocity_vector = vector if record.deadman else _zero_vector()
            record.gripper_velocity_deg_s = float(gripper_velocity_deg_s)
            record.last_seq_received = seq_value
            if backend_mode is not None:
                record.backend_mode = str(backend_mode or record.backend_mode)
            if backend_timeout_s is not None:
                record.backend_timeout_s = self._normalize_optional_timeout(backend_timeout_s)
            return self._snapshot_locked(now)

    def stop_session(
        self,
        *,
        session_id: str | None,
        owner_id: str | None = None,
        reason: str = "client-stop",
        allow_missing: bool = True,
    ) -> dict[str, Any]:
        owner = owner_id.strip() if isinstance(owner_id, str) and owner_id.strip() else None
        now = time.monotonic()
        with self._lock:
            record = self._record
            if record is None:
                if allow_missing:
                    return self._snapshot_locked(now)
                raise JogSessionError("SESSION_NOT_FOUND", "Jog session not found.")
            if session_id and record.session_id != str(session_id):
                if allow_missing:
                    return self._snapshot_locked(now)
                raise JogSessionError("WRONG_SESSION", "Jog session id does not match the active session.")
            if owner is not None and record.owner_id != owner:
                record.owner_conflict_rejects += 1
                raise JogSessionError("OWNER_CONFLICT", "Jog session owner does not match the active session owner.")
            self._stop_locked(record, reason=reason)
            return self._snapshot_locked(now)

    def expire_if_needed(self) -> dict[str, Any]:
        now = time.monotonic()
        with self._lock:
            record = self._record
            if record is not None and record.is_live() and record.lease_deadline_monotonic > 0.0 and now > record.lease_deadline_monotonic:
                self._expire_locked(record, reason="lease-expired")
            return self._snapshot_locked(now)

    def pause_for_motion(self) -> dict[str, Any]:
        now = time.monotonic()
        with self._lock:
            record = self._record
            if record is not None and record.is_live() and record.state != JogSessionState.PAUSED_FOR_MOTION:
                record.state = JogSessionState.PAUSED_FOR_MOTION
                record.pause_for_motion_count += 1
            return self._snapshot_locked(now)

    def resume_after_motion(self) -> dict[str, Any]:
        now = time.monotonic()
        with self._lock:
            record = self._record
            if record is not None and record.is_live():
                if record.lease_deadline_monotonic > 0.0 and now > record.lease_deadline_monotonic:
                    self._expire_locked(record, reason="motion-resume-lease-expired")
                elif record.state == JogSessionState.PAUSED_FOR_MOTION:
                    record.state = JogSessionState.ACTIVE
            return self._snapshot_locked(now)

    def mark_seq_applied(self, seq: int) -> dict[str, Any]:
        now = time.monotonic()
        with self._lock:
            record = self._record
            if record is not None and seq >= record.last_seq_applied:
                record.last_seq_applied = int(seq)
            return self._snapshot_locked(now)

    def get_snapshot(self) -> dict[str, Any]:
        with self._lock:
            return self._snapshot_locked(time.monotonic())

    def get_control_state(self) -> dict[str, Any]:
        now = time.monotonic()
        with self._lock:
            record = self._record
            if record is None:
                return {
                    "session_active": False,
                    "lease_valid": False,
                    "deadman": False,
                    "paused_for_motion": False,
                    "velocity_vector": _zero_vector(),
                    "gripper_velocity_deg_s": 0.0,
                    "backend_timeout_s": None,
                    "last_seq_received": -1,
                    "command_state_valid": False,
                    "commanded_position_m": None,
                    "commanded_orientation_matrix": None,
                    "commanded_joint_vector": None,
                    "last_accepted_target_position_m": None,
                    "last_accepted_target_orientation_matrix": None,
                    "last_resync_reason": None,
                    "last_resync_monotonic": None,
                    "following_error_snapshot": None,
                    "last_gate_failure_reason": None,
                    "last_gate_failure_details": None,
                }
            lease_valid = bool(record.is_live() and record.lease_deadline_monotonic > now)
            paused = record.state == JogSessionState.PAUSED_FOR_MOTION
            return {
                "session_active": bool(record.is_live()),
                "lease_valid": lease_valid,
                "deadman": bool(record.deadman),
                "paused_for_motion": paused,
                "velocity_vector": record.velocity_vector if lease_valid and not paused else _zero_vector(),
                "gripper_velocity_deg_s": float(record.gripper_velocity_deg_s) if lease_valid and not paused else 0.0,
                "backend_timeout_s": record.backend_timeout_s,
                "last_seq_received": int(record.last_seq_received),
                "command_state_valid": self._command_state_valid(record),
                "commanded_position_m": record.commanded_position_m,
                "commanded_orientation_matrix": record.commanded_orientation_matrix,
                "commanded_joint_vector": record.commanded_joint_vector,
                "last_accepted_target_position_m": record.last_accepted_target_position_m,
                "last_accepted_target_orientation_matrix": record.last_accepted_target_orientation_matrix,
                "last_resync_reason": record.last_resync_reason,
                "last_resync_monotonic": record.last_resync_monotonic,
                "following_error_snapshot": _json_safe_copy(record.following_error_snapshot),
                "last_gate_failure_reason": record.last_gate_failure_reason,
                "last_gate_failure_details": _json_safe_copy(record.last_gate_failure_details),
            }

    def resync_command_state(
        self,
        *,
        position_m: tuple[float, float, float] | list[float],
        orientation_matrix: list[list[float]] | tuple[tuple[float, ...], ...],
        joint_vector: tuple[float, ...] | list[float],
        reason: str,
    ) -> dict[str, Any]:
        now = time.monotonic()
        with self._lock:
            record = self._require_live_record_locked()
            self._set_command_state_locked(
                record,
                position_m=position_m,
                orientation_matrix=orientation_matrix,
                joint_vector=joint_vector,
            )
            record.last_resync_reason = str(reason or "unspecified")
            record.last_resync_monotonic = now
            record.last_gate_failure_reason = None
            record.last_gate_failure_details = None
            return self._snapshot_locked(now)

    def accept_command_step(
        self,
        *,
        position_m: tuple[float, float, float] | list[float],
        orientation_matrix: list[list[float]] | tuple[tuple[float, ...], ...],
        joint_vector: tuple[float, ...] | list[float],
    ) -> dict[str, Any]:
        now = time.monotonic()
        with self._lock:
            record = self._require_live_record_locked()
            self._set_command_state_locked(
                record,
                position_m=position_m,
                orientation_matrix=orientation_matrix,
                joint_vector=joint_vector,
            )
            record.last_gate_failure_reason = None
            record.last_gate_failure_details = None
            return self._snapshot_locked(now)

    def update_following_error(self, following_error_snapshot: dict[str, Any] | None) -> dict[str, Any]:
        now = time.monotonic()
        with self._lock:
            record = self._require_live_record_locked()
            record.following_error_snapshot = _json_safe_copy(following_error_snapshot)
            return self._snapshot_locked(now)

    def record_gate_failure(
        self,
        *,
        reason: str,
        details: dict[str, Any] | None = None,
    ) -> dict[str, Any]:
        now = time.monotonic()
        with self._lock:
            record = self._require_live_record_locked()
            record.last_gate_failure_reason = str(reason or "unspecified")
            record.last_gate_failure_details = _json_safe_copy(details or {})
            return self._snapshot_locked(now)

    def _normalize_owner(self, owner_id: str) -> str:
        owner = str(owner_id or "").strip()
        if not owner:
            raise JogSessionError("INVALID_OWNER", "Jog session owner_id is required.")
        return owner

    def _normalize_seq(self, seq: int) -> int:
        try:
            return int(seq)
        except Exception as exc:
            raise JogSessionError("INVALID_SEQUENCE", "Jog session seq must be an integer.") from exc

    def _normalize_timeout(self, timeout_s: float) -> float:
        try:
            value = float(timeout_s)
        except Exception as exc:
            raise JogSessionError("INVALID_TIMEOUT", "Jog session lease timeout must be a number.") from exc
        if not (value > 0.0):
            raise JogSessionError("INVALID_TIMEOUT", "Jog session lease timeout must be greater than zero.")
        return value

    def _normalize_optional_timeout(self, timeout_s: float | None) -> float | None:
        if timeout_s is None:
            return None
        return self._normalize_timeout(timeout_s)

    def _normalize_vector(
        self,
        velocity_vector: tuple[float, float, float, float, float, float] | list[float],
    ) -> tuple[float, float, float, float, float, float]:
        if len(velocity_vector) != 6:
            raise JogSessionError("INVALID_VECTOR", "Jog session velocity vector must contain 6 values.")
        try:
            return tuple(float(value) for value in velocity_vector)  # type: ignore[return-value]
        except Exception as exc:
            raise JogSessionError("INVALID_VECTOR", "Jog session velocity vector must be numeric.") from exc

    def _require_matching_live_record(self, *, session_id: str, owner_id: str) -> JogSessionRecord:
        record = self._record
        if record is None:
            raise JogSessionError("SESSION_NOT_FOUND", "Jog session not found.")
        if record.session_id != str(session_id):
            raise JogSessionError("WRONG_SESSION", "Jog session id does not match the active session.")
        if record.owner_id != owner_id:
            record.owner_conflict_rejects += 1
            raise JogSessionError("OWNER_CONFLICT", "Jog session owner does not match the active session owner.")
        if not record.is_live():
            raise JogSessionError("SESSION_INACTIVE", "Jog session is no longer active.", payload={"state": record.state.value})
        return record

    def _require_live_record_locked(self) -> JogSessionRecord:
        record = self._record
        if record is None:
            raise JogSessionError("SESSION_NOT_FOUND", "Jog session not found.")
        if not record.is_live():
            raise JogSessionError("SESSION_INACTIVE", "Jog session is no longer active.", payload={"state": record.state.value})
        return record

    def _command_state_valid(self, record: JogSessionRecord) -> bool:
        return (
            record.commanded_position_m is not None
            and record.commanded_orientation_matrix is not None
            and record.commanded_joint_vector is not None
            and record.last_accepted_target_position_m is not None
            and record.last_accepted_target_orientation_matrix is not None
        )

    def _set_command_state_locked(
        self,
        record: JogSessionRecord,
        *,
        position_m: tuple[float, float, float] | list[float],
        orientation_matrix: list[list[float]] | tuple[tuple[float, ...], ...],
        joint_vector: tuple[float, ...] | list[float],
    ) -> None:
        normalized_position = _normalize_position_vector(position_m)
        normalized_orientation = _normalize_orientation_matrix(orientation_matrix)
        normalized_joints = _normalize_joint_vector(joint_vector)
        record.commanded_position_m = normalized_position
        record.commanded_orientation_matrix = normalized_orientation
        record.commanded_joint_vector = normalized_joints
        record.last_accepted_target_position_m = normalized_position
        record.last_accepted_target_orientation_matrix = normalized_orientation

    def _stop_locked(self, record: JogSessionRecord, *, reason: str) -> None:
        record.state = JogSessionState.STOPPED
        record.deadman = False
        record.velocity_vector = _zero_vector()
        record.gripper_velocity_deg_s = 0.0
        record.lease_deadline_monotonic = 0.0
        record.last_stop_reason = str(reason or "client-stop")

    def _expire_locked(self, record: JogSessionRecord, *, reason: str) -> None:
        record.lease_expiry_count += 1
        record.state = JogSessionState.EXPIRED
        record.deadman = False
        record.velocity_vector = _zero_vector()
        record.gripper_velocity_deg_s = 0.0
        record.lease_deadline_monotonic = 0.0
        record.last_stop_reason = str(reason or "lease-expired")

    def _snapshot_locked(self, now: float) -> dict[str, Any]:
        record = self._record
        if record is None:
            return {
                "session_present": False,
                "session_active": False,
                "session_id": None,
                "owner_id": None,
                "state": JogSessionState.IDLE.value,
                "deadman": False,
                "paused_for_motion": False,
                "lease_timeout_s": None,
                "lease_remaining_s": None,
                "last_update_age_s": None,
                "last_seq_received": -1,
                "last_seq_applied": -1,
                "lease_expiry_count": 0,
                "pause_for_motion_count": 0,
                "backend_mode": None,
                "backend_timeout_s": None,
                "last_stop_reason": None,
                "stale_packet_rejects": 0,
                "owner_conflict_rejects": 0,
                "command_state_valid": False,
                "commanded_position_m": None,
                "commanded_orientation_matrix": None,
                "commanded_joint_vector": None,
                "last_accepted_target_position_m": None,
                "last_accepted_target_orientation_matrix": None,
                "last_resync_reason": None,
                "last_resync_age_s": None,
                "following_error_snapshot": None,
                "last_gate_failure_reason": None,
                "last_gate_failure_details": None,
            }
        lease_remaining_s = (
            max(0.0, float(record.lease_deadline_monotonic) - now)
            if record.lease_deadline_monotonic > 0.0
            else 0.0
        )
        last_update_age_s = max(0.0, now - float(record.updated_monotonic))
        last_resync_age_s = (
            max(0.0, now - float(record.last_resync_monotonic))
            if isinstance(record.last_resync_monotonic, (int, float))
            else None
        )
        return {
            "session_present": True,
            "session_active": bool(record.is_live()),
            "session_id": record.session_id,
            "owner_id": record.owner_id,
            "state": record.state.value,
            "deadman": bool(record.deadman),
            "paused_for_motion": record.state == JogSessionState.PAUSED_FOR_MOTION,
            "lease_timeout_s": float(record.lease_timeout_s),
            "lease_remaining_s": lease_remaining_s,
            "last_update_age_s": last_update_age_s,
            "last_seq_received": int(record.last_seq_received),
            "last_seq_applied": int(record.last_seq_applied),
            "lease_expiry_count": int(record.lease_expiry_count),
            "pause_for_motion_count": int(record.pause_for_motion_count),
            "backend_mode": record.backend_mode,
            "backend_timeout_s": record.backend_timeout_s,
            "last_stop_reason": record.last_stop_reason,
            "stale_packet_rejects": int(record.stale_packet_rejects),
            "owner_conflict_rejects": int(record.owner_conflict_rejects),
            "command_state_valid": self._command_state_valid(record),
            "commanded_position_m": _json_safe_copy(record.commanded_position_m),
            "commanded_orientation_matrix": _json_safe_copy(record.commanded_orientation_matrix),
            "commanded_joint_vector": _json_safe_copy(record.commanded_joint_vector),
            "last_accepted_target_position_m": _json_safe_copy(record.last_accepted_target_position_m),
            "last_accepted_target_orientation_matrix": _json_safe_copy(record.last_accepted_target_orientation_matrix),
            "last_resync_reason": record.last_resync_reason,
            "last_resync_age_s": last_resync_age_s,
            "following_error_snapshot": _json_safe_copy(record.following_error_snapshot),
            "last_gate_failure_reason": record.last_gate_failure_reason,
            "last_gate_failure_details": _json_safe_copy(record.last_gate_failure_details),
        }
