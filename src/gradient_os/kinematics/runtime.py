from __future__ import annotations

from dataclasses import dataclass
import threading
from typing import Any

import numpy as np
from scipy.spatial.transform import Rotation as R

from .. import robot_assets
from .profile import (
    KinematicsProfile,
    KinematicsProfileError,
    KinematicsProfileErrorCode,
    build_profile_from_payload,
    identity_matrix4,
    validate_profile_for_backend,
)


DEFAULT_OFFSET = {
    "position_m": {"x": 0.0, "y": 0.0, "z": 0.0},
    "rotation_deg": {"x": 0.0, "y": 0.0, "z": 0.0},
}


@dataclass(frozen=True)
class RuntimeKinematicsError(ValueError):
    code: str
    message: str

    def __str__(self) -> str:
        return self.message


_LOCK = threading.RLock()
_ACTIVE_PROFILE: KinematicsProfile | None = None
_REVISION = 0
_BASE_RUNTIME_OFFSET = dict(DEFAULT_OFFSET)
_TOOL_RUNTIME_OFFSET = dict(DEFAULT_OFFSET)
_TOOL_BASE_OFFSET = dict(DEFAULT_OFFSET)
_ACTIVE_TOOL: dict[str, Any] = {
    "active_tool_id": "identity",
    "display_name": "Identity (No Tool Offset)",
    "tool_type": "utility",
    "source": "default_identity",
    "offset": {
        "position_mm": {"x": 0.0, "y": 0.0, "z": 0.0},
        "rotation_deg": {"x": 0.0, "y": 0.0, "z": 0.0},
    },
    "mesh": None,
    "weld": {},
}


def _deep_copy_offset(offset: dict[str, Any]) -> dict[str, Any]:
    return {
        "position_m": {
            "x": float(offset["position_m"]["x"]),
            "y": float(offset["position_m"]["y"]),
            "z": float(offset["position_m"]["z"]),
        },
        "rotation_deg": {
            "x": float(offset["rotation_deg"]["x"]),
            "y": float(offset["rotation_deg"]["y"]),
            "z": float(offset["rotation_deg"]["z"]),
        },
    }


def _tool_offset_to_runtime_offset(raw: Any) -> dict[str, Any]:
    if not isinstance(raw, dict):
        return _deep_copy_offset(DEFAULT_OFFSET)
    if isinstance(raw.get("position_m"), dict):
        return _normalize_offset_payload(raw, "tool_runtime")
    offset = raw.get("offset")
    if isinstance(offset, dict):
        position_mm = offset.get("position_mm", {})
        rotation_deg = offset.get("rotation_deg", {})
    else:
        position_mm = raw.get("position_mm", {})
        rotation_deg = raw.get("rotation_deg", {})
    if not isinstance(position_mm, dict) or not isinstance(rotation_deg, dict):
        return _deep_copy_offset(DEFAULT_OFFSET)
    runtime_like = {
        "position_m": {
            "x": float(position_mm.get("x", 0.0)) / 1000.0,
            "y": float(position_mm.get("y", 0.0)) / 1000.0,
            "z": float(position_mm.get("z", 0.0)) / 1000.0,
        },
        "rotation_deg": {
            "x": float(rotation_deg.get("x", 0.0)),
            "y": float(rotation_deg.get("y", 0.0)),
            "z": float(rotation_deg.get("z", 0.0)),
        },
    }
    return _normalize_offset_payload(runtime_like, "tool_runtime")


def _coerce_axis_triplet(value: Any, field_name: str) -> dict[str, float]:
    if not isinstance(value, dict):
        raise RuntimeKinematicsError("INVALID_PAYLOAD", f"Field '{field_name}' must be an object.")
    out: dict[str, float] = {}
    for axis in ("x", "y", "z"):
        raw = value.get(axis)
        try:
            out[axis] = float(raw)
        except (TypeError, ValueError):
            raise RuntimeKinematicsError(
                "INVALID_PAYLOAD",
                f"Field '{field_name}.{axis}' must be numeric.",
            )
    return out


def _normalize_offset_payload(raw: Any, field_name: str) -> dict[str, Any]:
    if not isinstance(raw, dict):
        raise RuntimeKinematicsError("INVALID_PAYLOAD", f"Field '{field_name}' must be an object.")
    position = _coerce_axis_triplet(raw.get("position_m", {}), f"{field_name}.position_m")
    rotation = _coerce_axis_triplet(raw.get("rotation_deg", {}), f"{field_name}.rotation_deg")
    return {"position_m": position, "rotation_deg": rotation}


def _matrix_from_offset(offset: dict[str, Any]) -> np.ndarray:
    p = offset["position_m"]
    r = offset["rotation_deg"]
    mat = np.eye(4, dtype=float)
    mat[:3, :3] = R.from_euler(
        "xyz",
        [float(r["x"]), float(r["y"]), float(r["z"])],
        degrees=True,
    ).as_matrix()
    mat[:3, 3] = np.array([float(p["x"]), float(p["y"]), float(p["z"])], dtype=float)
    return mat


def _offset_from_matrix(matrix: np.ndarray) -> dict[str, Any]:
    rot = R.from_matrix(matrix[:3, :3]).as_euler("xyz", degrees=True)
    pos = matrix[:3, 3]
    return {
        "position_m": {"x": float(pos[0]), "y": float(pos[1]), "z": float(pos[2])},
        "rotation_deg": {"x": float(rot[0]), "y": float(rot[1]), "z": float(rot[2])},
    }


def _ensure_initialized() -> None:
    global _ACTIVE_PROFILE
    if _ACTIVE_PROFILE is not None:
        return
    profile = robot_assets.load_kinematics_profile()
    _ACTIVE_PROFILE = profile


def _set_revision_locked() -> None:
    global _REVISION
    _REVISION += 1


def _assert_idle_or_raise(motion_state: str) -> None:
    if (motion_state or "IDLE").upper() != "IDLE":
        raise RuntimeKinematicsError(
            "MOTION_ACTIVE",
            f"Kinematics update rejected while motion_state={motion_state}.",
        )


def _assert_revision_or_raise(expected_revision: int | None) -> None:
    if expected_revision is None:
        return
    if int(expected_revision) != _REVISION:
        raise RuntimeKinematicsError(
            "STALE_REVISION",
            f"Stale revision: expected={expected_revision}, active={_REVISION}.",
        )


def get_runtime_state_snapshot() -> dict[str, Any]:
    with _LOCK:
        _ensure_initialized()
        assert _ACTIVE_PROFILE is not None
        effective_tool_matrix = _matrix_from_offset(_TOOL_BASE_OFFSET).dot(
            _matrix_from_offset(_TOOL_RUNTIME_OFFSET)
        )
        return {
            "revision": int(_REVISION),
            "profile": _ACTIVE_PROFILE.to_payload(),
            "active_tool": dict(_ACTIVE_TOOL),
            "offsets": {
                "base": _deep_copy_offset(_BASE_RUNTIME_OFFSET),
                "tool": _deep_copy_offset(_TOOL_RUNTIME_OFFSET),
                "tool_base": _deep_copy_offset(_TOOL_BASE_OFFSET),
                "tool_effective": _offset_from_matrix(effective_tool_matrix),
            },
        }


def get_revision() -> int:
    """Return the active runtime-kinematics revision for cache invalidation."""
    with _LOCK:
        _ensure_initialized()
        return int(_REVISION)


def get_runtime_matrices() -> tuple[np.ndarray, np.ndarray]:
    with _LOCK:
        _ensure_initialized()
        base_matrix = _matrix_from_offset(_BASE_RUNTIME_OFFSET)
        tool_base_matrix = _matrix_from_offset(_TOOL_BASE_OFFSET)
        tool_runtime_trim = _matrix_from_offset(_TOOL_RUNTIME_OFFSET)
        tool_matrix = tool_base_matrix.dot(tool_runtime_trim)
    return base_matrix, tool_matrix


def runtime_offsets_are_identity() -> bool:
    """Return True when runtime base and tool transforms are both identity."""
    base_matrix, tool_matrix = get_runtime_matrices()
    identity = np.eye(4, dtype=float)
    return bool(np.allclose(base_matrix, identity) and np.allclose(tool_matrix, identity))


def set_active_tool_definition(
    tool_payload: dict[str, Any],
    *,
    expected_revision: int | None,
    motion_state: str,
    reset_runtime_trim: bool = True,
) -> dict[str, Any]:
    global _ACTIVE_TOOL, _TOOL_BASE_OFFSET, _TOOL_RUNTIME_OFFSET
    if not isinstance(tool_payload, dict):
        raise RuntimeKinematicsError("INVALID_PAYLOAD", "Tool payload must be an object.")
    with _LOCK:
        _ensure_initialized()
        _assert_idle_or_raise(motion_state)
        _assert_revision_or_raise(expected_revision)
        active_tool_id = str(
            tool_payload.get("active_tool_id")
            or tool_payload.get("tool_id")
            or "identity"
        ).strip()
        if not active_tool_id:
            active_tool_id = "identity"
        _ACTIVE_TOOL = {
            "active_tool_id": active_tool_id,
            "display_name": str(
                tool_payload.get("display_name")
                or tool_payload.get("name")
                or active_tool_id
            ).strip(),
            "tool_type": str(tool_payload.get("tool_type", "generic")).strip().lower(),
            "source": str(tool_payload.get("source", tool_payload.get("selection_source", "runtime"))),
            "offset": dict(tool_payload.get("offset", {}))
            if isinstance(tool_payload.get("offset"), dict)
            else {},
            "mesh": (
                dict(tool_payload.get("mesh"))
                if isinstance(tool_payload.get("mesh"), dict)
                else None
            ),
            "weld": dict(tool_payload.get("weld", {}))
            if isinstance(tool_payload.get("weld"), dict)
            else {},
        }
        _TOOL_BASE_OFFSET = _tool_offset_to_runtime_offset(tool_payload)
        if reset_runtime_trim:
            _TOOL_RUNTIME_OFFSET = _deep_copy_offset(DEFAULT_OFFSET)
        _set_revision_locked()
        return get_runtime_state_snapshot()


def compensate_target_pose_for_runtime(
    target_position: np.ndarray,
    target_rotation: np.ndarray,
) -> tuple[np.ndarray, np.ndarray]:
    base_runtime, tool_runtime = get_runtime_matrices()
    target = np.eye(4, dtype=float)
    target[:3, :3] = np.asarray(target_rotation, dtype=float).reshape(3, 3)
    target[:3, 3] = np.asarray(target_position, dtype=float).reshape(3)
    compensated = np.linalg.inv(base_runtime).dot(target).dot(np.linalg.inv(tool_runtime))
    return compensated[:3, 3], compensated[:3, :3]


def apply_runtime_to_fk_matrix(model_fk_matrix: np.ndarray) -> np.ndarray:
    base_runtime, tool_runtime = get_runtime_matrices()
    fk = np.asarray(model_fk_matrix, dtype=float).reshape(4, 4)
    return base_runtime.dot(fk).dot(tool_runtime)


def patch_runtime_offsets(
    patch: dict[str, Any],
    *,
    expected_revision: int | None,
    motion_state: str,
) -> dict[str, Any]:
    global _BASE_RUNTIME_OFFSET, _TOOL_RUNTIME_OFFSET
    if not isinstance(patch, dict):
        raise RuntimeKinematicsError("INVALID_PAYLOAD", "PATCH payload must be an object.")
    with _LOCK:
        _ensure_initialized()
        _assert_idle_or_raise(motion_state)
        _assert_revision_or_raise(expected_revision)
        if "base" in patch:
            _BASE_RUNTIME_OFFSET = _normalize_offset_payload(patch["base"], "base")
        if "tool" in patch:
            _TOOL_RUNTIME_OFFSET = _normalize_offset_payload(patch["tool"], "tool")
        if "base" not in patch and "tool" not in patch:
            raise RuntimeKinematicsError("INVALID_PAYLOAD", "PATCH must include 'base' and/or 'tool'.")
        _set_revision_locked()
        return get_runtime_state_snapshot()


def reset_runtime_offsets(*, expected_revision: int | None, motion_state: str) -> dict[str, Any]:
    global _BASE_RUNTIME_OFFSET, _TOOL_RUNTIME_OFFSET
    with _LOCK:
        _ensure_initialized()
        _assert_idle_or_raise(motion_state)
        _assert_revision_or_raise(expected_revision)
        _BASE_RUNTIME_OFFSET = _deep_copy_offset(DEFAULT_OFFSET)
        _TOOL_RUNTIME_OFFSET = _deep_copy_offset(DEFAULT_OFFSET)
        _set_revision_locked()
        return get_runtime_state_snapshot()


def apply_profile_payload(
    payload: dict[str, Any],
    *,
    expected_revision: int | None,
    motion_state: str,
    backend_name: str,
) -> dict[str, Any]:
    global _ACTIVE_PROFILE, _BASE_RUNTIME_OFFSET, _TOOL_RUNTIME_OFFSET
    with _LOCK:
        _ensure_initialized()
        assert _ACTIVE_PROFILE is not None
        _assert_idle_or_raise(motion_state)
        _assert_revision_or_raise(expected_revision)
        try:
            next_profile = build_profile_from_payload(payload, expected_robot_id=_ACTIVE_PROFILE.robot_id)
            validate_profile_for_backend(next_profile, backend_name)
        except KinematicsProfileError as exc:
            if exc.code == KinematicsProfileErrorCode.BACKEND_INCOMPATIBLE:
                raise RuntimeKinematicsError("PROFILE_INVALID", str(exc))
            raise RuntimeKinematicsError("INVALID_PAYLOAD", str(exc))

        # Runtime-only hot apply: keep nominal/calibration model fixed unless equal.
        if not (
            np.allclose(_ACTIVE_PROFILE.base_nominal, next_profile.base_nominal)
            and np.allclose(_ACTIVE_PROFILE.base_calib, next_profile.base_calib)
            and np.allclose(_ACTIVE_PROFILE.tool_nominal, next_profile.tool_nominal)
            and np.allclose(_ACTIVE_PROFILE.tool_calib, next_profile.tool_calib)
        ):
            raise RuntimeKinematicsError(
                "APPLY_REQUIRES_RESTART",
                "Applying profile with changed nominal/calibration transforms requires solver restart.",
            )

        _ACTIVE_PROFILE = next_profile
        _BASE_RUNTIME_OFFSET = _deep_copy_offset(DEFAULT_OFFSET)
        _TOOL_RUNTIME_OFFSET = _offset_from_matrix(next_profile.tool_runtime)
        _set_revision_locked()
        return get_runtime_state_snapshot()

