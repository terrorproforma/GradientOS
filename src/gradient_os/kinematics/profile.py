from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
import hashlib
import json
from typing import Any

import numpy as np


def identity_matrix4() -> np.ndarray:
    return np.eye(4, dtype=float)


class KinematicsProfileErrorCode(str, Enum):
    MISSING_FIELD = "MISSING_FIELD"
    INVALID_FIELD = "INVALID_FIELD"
    INVALID_SCHEMA = "INVALID_SCHEMA"
    INVALID_CHECKSUM = "INVALID_CHECKSUM"
    ROBOT_MISMATCH = "ROBOT_MISMATCH"
    BACKEND_INCOMPATIBLE = "BACKEND_INCOMPATIBLE"


class KinematicsProfileError(ValueError):
    def __init__(self, code: KinematicsProfileErrorCode, message: str):
        super().__init__(message)
        self.code = code


@dataclass(frozen=True)
class KinematicsProfile:
    profile_id: str
    version: str
    schema_version: int
    checksum: str
    robot_id: str
    robot_serial: str
    base_nominal: np.ndarray
    base_calib: np.ndarray
    tool_nominal: np.ndarray
    tool_calib: np.ndarray
    tool_runtime: np.ndarray
    backend_compatibility: tuple[str, ...]
    metadata: dict[str, Any]

    def to_payload(self) -> dict[str, Any]:
        return {
            "profile_id": self.profile_id,
            "version": self.version,
            "schema_version": self.schema_version,
            "checksum": self.checksum,
            "robot_id": self.robot_id,
            "robot_serial": self.robot_serial,
            "base_nominal": self.base_nominal.tolist(),
            "base_calib": self.base_calib.tolist(),
            "tool_nominal": self.tool_nominal.tolist(),
            "tool_calib": self.tool_calib.tolist(),
            "tool_runtime": self.tool_runtime.tolist(),
            "backend_compatibility": list(self.backend_compatibility),
            "metadata": self.metadata,
        }


def _parse_transform4(value: Any, field_name: str) -> np.ndarray:
    if isinstance(value, list):
        if len(value) == 16 and all(isinstance(v, (int, float)) for v in value):
            return np.asarray(value, dtype=float).reshape(4, 4)
        if len(value) == 4 and all(isinstance(row, list) and len(row) == 4 for row in value):
            flat = [v for row in value for v in row]
            if all(isinstance(v, (int, float)) for v in flat):
                return np.asarray(flat, dtype=float).reshape(4, 4)
    raise KinematicsProfileError(
        KinematicsProfileErrorCode.INVALID_FIELD,
        f"Invalid '{field_name}'. Expected 16-number flat list or 4x4 nested list.",
    )


def _required_str(payload: dict[str, Any], field_name: str) -> str:
    value = payload.get(field_name)
    if not isinstance(value, str) or not value.strip():
        raise KinematicsProfileError(
            KinematicsProfileErrorCode.MISSING_FIELD,
            f"Missing required string field '{field_name}'.",
        )
    return value.strip()


def _canonical_checksum_payload(payload: dict[str, Any]) -> dict[str, Any]:
    canonical = dict(payload)
    canonical.pop("checksum", None)
    return canonical


def _compute_checksum(payload: dict[str, Any]) -> str:
    canonical = _canonical_checksum_payload(payload)
    encoded = json.dumps(canonical, sort_keys=True, separators=(",", ":"), ensure_ascii=True)
    return hashlib.sha256(encoded.encode("utf-8")).hexdigest()


def build_profile_from_payload(payload: dict[str, Any], expected_robot_id: str | None = None) -> KinematicsProfile:
    if not isinstance(payload, dict):
        raise KinematicsProfileError(
            KinematicsProfileErrorCode.INVALID_FIELD,
            "Kinematics profile payload must be a JSON object.",
        )

    schema_version_raw = payload.get("schema_version", 1)
    try:
        schema_version = int(schema_version_raw)
    except (TypeError, ValueError):
        raise KinematicsProfileError(
            KinematicsProfileErrorCode.INVALID_SCHEMA,
            "Field 'schema_version' must be an integer.",
        )
    if schema_version != 1:
        raise KinematicsProfileError(
            KinematicsProfileErrorCode.INVALID_SCHEMA,
            f"Unsupported kinematics profile schema_version={schema_version}.",
        )

    robot_id = _required_str(payload, "robot_id")
    if expected_robot_id is not None and robot_id != expected_robot_id:
        raise KinematicsProfileError(
            KinematicsProfileErrorCode.ROBOT_MISMATCH,
            f"Profile robot_id '{robot_id}' does not match expected '{expected_robot_id}'.",
        )

    profile_id = _required_str(payload, "profile_id")
    version = _required_str(payload, "version")
    robot_serial = _required_str(payload, "robot_serial")

    base_nominal = _parse_transform4(payload.get("base_nominal", identity_matrix4().tolist()), "base_nominal")
    base_calib = _parse_transform4(payload.get("base_calib", identity_matrix4().tolist()), "base_calib")
    tool_nominal = _parse_transform4(payload.get("tool_nominal", identity_matrix4().tolist()), "tool_nominal")
    tool_calib = _parse_transform4(payload.get("tool_calib", identity_matrix4().tolist()), "tool_calib")
    tool_runtime = _parse_transform4(payload.get("tool_runtime", identity_matrix4().tolist()), "tool_runtime")

    backend_compatibility_raw = payload.get("backend_compatibility", ["ikfast", "numeric"])
    if not isinstance(backend_compatibility_raw, list) or not backend_compatibility_raw:
        raise KinematicsProfileError(
            KinematicsProfileErrorCode.INVALID_FIELD,
            "Field 'backend_compatibility' must be a non-empty list.",
        )
    backend_compatibility: list[str] = []
    for entry in backend_compatibility_raw:
        if not isinstance(entry, str) or not entry.strip():
            raise KinematicsProfileError(
                KinematicsProfileErrorCode.INVALID_FIELD,
                "Field 'backend_compatibility' contains a non-string value.",
            )
        backend_compatibility.append(entry.strip().lower())

    metadata = payload.get("metadata", {})
    if metadata is None:
        metadata = {}
    if not isinstance(metadata, dict):
        raise KinematicsProfileError(
            KinematicsProfileErrorCode.INVALID_FIELD,
            "Field 'metadata' must be an object.",
        )

    checksum = _compute_checksum(payload)
    payload_checksum = payload.get("checksum")
    if payload_checksum is not None:
        if not isinstance(payload_checksum, str) or payload_checksum.lower() != checksum.lower():
            raise KinematicsProfileError(
                KinematicsProfileErrorCode.INVALID_CHECKSUM,
                "Kinematics profile checksum mismatch.",
            )

    return KinematicsProfile(
        profile_id=profile_id,
        version=version,
        schema_version=schema_version,
        checksum=checksum,
        robot_id=robot_id,
        robot_serial=robot_serial,
        base_nominal=base_nominal,
        base_calib=base_calib,
        tool_nominal=tool_nominal,
        tool_calib=tool_calib,
        tool_runtime=tool_runtime,
        backend_compatibility=tuple(backend_compatibility),
        metadata=dict(metadata),
    )


def validate_profile_for_backend(profile: KinematicsProfile, backend_name: str) -> None:
    backend = (backend_name or "").strip().lower()
    if backend not in {"ikfast", "numeric", "trac"}:
        raise KinematicsProfileError(
            KinematicsProfileErrorCode.BACKEND_INCOMPATIBLE,
            f"Unsupported backend '{backend_name}'.",
        )
    if backend not in profile.backend_compatibility:
        raise KinematicsProfileError(
            KinematicsProfileErrorCode.BACKEND_INCOMPATIBLE,
            f"Profile '{profile.profile_id}' is not compatible with backend '{backend}'.",
        )

