"""Robot asset catalog resolver.

This module provides a strict, no-fallback path resolver for robot model assets
stored under the repository-level ``robots/`` directory.
"""

from __future__ import annotations

from dataclasses import dataclass
from functools import lru_cache
import json
import os
from pathlib import Path
from typing import Any


DEFAULT_ROBOT_ENV_VAR = "GRADIENT_ROBOT_ID"


@dataclass(frozen=True)
class RobotAssetManifest:
    """Resolved manifest data and canonical paths for a robot asset bundle."""

    robot_id: str
    name: str
    manifest_path: Path
    robot_root: Path
    dh_csv_path: Path
    controller_urdf_path: Path
    web_urdf_path: Path
    web_asset_source_dir: Path
    opw_urdf_path: Path | None
    is_default: bool
    manifest_data: dict[str, Any]


def _repo_root() -> Path:
    return Path(__file__).resolve().parents[2]


def _robots_root() -> Path:
    root = _repo_root() / "robots"
    if not root.is_dir():
        raise FileNotFoundError(f"Robots directory not found: {root}")
    return root


def _read_manifest_json(manifest_path: Path) -> dict[str, Any]:
    try:
        with manifest_path.open("r", encoding="utf-8") as f:
            data = json.load(f)
    except json.JSONDecodeError as exc:
        raise ValueError(f"Invalid JSON in robot manifest: {manifest_path}") from exc
    if not isinstance(data, dict):
        raise ValueError(f"Robot manifest must be a JSON object: {manifest_path}")
    return data


def _required_str(node: dict[str, Any], key: str, manifest_path: Path) -> str:
    value = node.get(key)
    if not isinstance(value, str) or not value.strip():
        raise ValueError(f"Missing required string '{key}' in {manifest_path}")
    return value


def _resolve_relative(root: Path, rel_path: str, manifest_path: Path, field_name: str) -> Path:
    path_obj = Path(rel_path)
    if path_obj.is_absolute():
        raise ValueError(f"Field '{field_name}' must be relative in {manifest_path}: {rel_path}")
    resolved = (root / path_obj).resolve()
    if not resolved.exists():
        raise FileNotFoundError(f"Field '{field_name}' points to missing path: {resolved}")
    return resolved


def _identity4_list() -> list[list[float]]:
    return [
        [1.0, 0.0, 0.0, 0.0],
        [0.0, 1.0, 0.0, 0.0],
        [0.0, 0.0, 1.0, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ]


def _matrix4_as_nested_list(value: Any, field_name: str, manifest_path: Path) -> list[list[float]]:
    if isinstance(value, list):
        if len(value) == 16 and all(isinstance(v, (int, float)) for v in value):
            flat = [float(v) for v in value]
            return [
                flat[0:4],
                flat[4:8],
                flat[8:12],
                flat[12:16],
            ]
        if len(value) == 4 and all(isinstance(row, list) and len(row) == 4 for row in value):
            flat = [v for row in value for v in row]
            if all(isinstance(v, (int, float)) for v in flat):
                return [[float(v) for v in row] for row in value]
    raise ValueError(
        f"Invalid '{field_name}' in {manifest_path}. "
        "Expected 16-number flat list or 4x4 nested list."
    )


def _build_legacy_kinematics_profile(
    *,
    robot_id: str,
    manifest_path: Path,
    manifest_data: dict[str, Any],
) -> dict[str, Any]:
    numeric_cfg = manifest_data.get("numeric", {})
    if numeric_cfg is None:
        numeric_cfg = {}
    if not isinstance(numeric_cfg, dict):
        raise ValueError(f"Field 'numeric' must be an object in {manifest_path}")

    base_nominal = _identity4_list()
    tool_nominal = _identity4_list()
    metadata: dict[str, Any] = {"source": "legacy_manifest_bridge"}

    if "tbase" in numeric_cfg:
        base_nominal = _matrix4_as_nested_list(
            numeric_cfg["tbase"],
            "numeric.tbase",
            manifest_path,
        )
    if "ttool" in numeric_cfg:
        tool_nominal = _matrix4_as_nested_list(
            numeric_cfg["ttool"],
            "numeric.ttool",
            manifest_path,
        )
    if "q_sign" in numeric_cfg:
        q_sign = numeric_cfg["q_sign"]
        if not isinstance(q_sign, list) or not all(isinstance(v, (int, float)) for v in q_sign):
            raise ValueError(
                f"Invalid 'numeric.q_sign' in {manifest_path}. Expected list of numeric values."
            )
        metadata["numeric_q_sign"] = [float(v) for v in q_sign]

    return {
        "profile_id": f"{robot_id}:legacy-default",
        "version": "legacy-bridge-v1",
        "schema_version": 1,
        "robot_id": robot_id,
        "robot_serial": "unknown",
        "base_nominal": base_nominal,
        "base_calib": _identity4_list(),
        "tool_nominal": tool_nominal,
        "tool_calib": _identity4_list(),
        "tool_runtime": _identity4_list(),
        "backend_compatibility": ["ikfast", "numeric"],
        "metadata": metadata,
    }


@lru_cache(maxsize=None)
def get_robot_manifest(robot_id: str) -> RobotAssetManifest:
    """Load and validate a robot asset manifest by ID."""

    if not robot_id or not robot_id.strip():
        raise ValueError("robot_id must be a non-empty string")

    root = _robots_root()
    robot_root = root / robot_id
    manifest_path = robot_root / "robot.json"
    if not manifest_path.is_file():
        raise FileNotFoundError(f"Robot manifest not found for '{robot_id}': {manifest_path}")

    data = _read_manifest_json(manifest_path)
    manifest_robot_id = _required_str(data, "robot_id", manifest_path)
    if manifest_robot_id != robot_id:
        raise ValueError(
            f"Manifest robot_id mismatch in {manifest_path}: "
            f"expected '{robot_id}', got '{manifest_robot_id}'"
        )

    name = _required_str(data, "name", manifest_path)

    kinematics = data.get("kinematics")
    models = data.get("models")
    web = data.get("web")
    if not isinstance(kinematics, dict) or not isinstance(models, dict) or not isinstance(web, dict):
        raise ValueError(
            f"Manifest {manifest_path} must include object sections: "
            "'kinematics', 'models', and 'web'"
        )

    dh_csv_rel = _required_str(kinematics, "dh_csv", manifest_path)
    controller_urdf_rel = _required_str(models, "controller_urdf", manifest_path)
    web_urdf_rel = _required_str(models, "web_urdf", manifest_path)
    web_source_rel = _required_str(web, "asset_source_dir", manifest_path)

    opw_urdf_rel = models.get("opw_urdf")
    if opw_urdf_rel is not None and not isinstance(opw_urdf_rel, str):
        raise ValueError(f"Field 'models.opw_urdf' must be a string when present: {manifest_path}")

    manifest = RobotAssetManifest(
        robot_id=robot_id,
        name=name,
        manifest_path=manifest_path,
        robot_root=robot_root.resolve(),
        dh_csv_path=_resolve_relative(robot_root, dh_csv_rel, manifest_path, "kinematics.dh_csv"),
        controller_urdf_path=_resolve_relative(
            robot_root,
            controller_urdf_rel,
            manifest_path,
            "models.controller_urdf",
        ),
        web_urdf_path=_resolve_relative(robot_root, web_urdf_rel, manifest_path, "models.web_urdf"),
        web_asset_source_dir=_resolve_relative(
            robot_root,
            web_source_rel,
            manifest_path,
            "web.asset_source_dir",
        ),
        opw_urdf_path=(
            _resolve_relative(robot_root, opw_urdf_rel, manifest_path, "models.opw_urdf")
            if isinstance(opw_urdf_rel, str)
            else None
        ),
        is_default=bool(data.get("default", False)),
        manifest_data=data,
    )
    return manifest


@lru_cache(maxsize=1)
def list_robot_ids() -> tuple[str, ...]:
    """List robot IDs that have a valid manifest."""

    root = _robots_root()
    robot_ids: list[str] = []
    for child in sorted(root.iterdir()):
        if not child.is_dir():
            continue
        manifest_path = child / "robot.json"
        if manifest_path.is_file():
            # Validate manifests up front to fail loudly during startup.
            get_robot_manifest(child.name)
            robot_ids.append(child.name)
    return tuple(robot_ids)


@lru_cache(maxsize=1)
def get_default_robot_id() -> str:
    """Return the single default robot from manifests."""

    defaults: list[str] = []
    for robot_id in list_robot_ids():
        if get_robot_manifest(robot_id).is_default:
            defaults.append(robot_id)
    if len(defaults) != 1:
        raise ValueError(
            "Exactly one robot manifest must set 'default: true'. "
            f"Found {len(defaults)} defaults: {defaults}"
        )
    return defaults[0]


def get_active_robot_id() -> str:
    """Resolve active robot ID from env, otherwise from manifest default."""

    env_robot_id = os.getenv(DEFAULT_ROBOT_ENV_VAR)
    if env_robot_id:
        # Validate that env value maps to a real manifest.
        get_robot_manifest(env_robot_id)
        return env_robot_id
    return get_default_robot_id()


def get_dh_csv_path(robot_id: str | None = None) -> Path:
    manifest = get_robot_manifest(robot_id or get_active_robot_id())
    return manifest.dh_csv_path


def get_controller_urdf_path(robot_id: str | None = None) -> Path:
    manifest = get_robot_manifest(robot_id or get_active_robot_id())
    return manifest.controller_urdf_path


def get_web_urdf_path(robot_id: str | None = None) -> Path:
    manifest = get_robot_manifest(robot_id or get_active_robot_id())
    return manifest.web_urdf_path


def get_robot_manifest_data(robot_id: str | None = None) -> dict[str, Any]:
    manifest = get_robot_manifest(robot_id or get_active_robot_id())
    return dict(manifest.manifest_data)


def get_kinematics_profile_payload(robot_id: str | None = None) -> dict[str, Any]:
    manifest = get_robot_manifest(robot_id or get_active_robot_id())
    payload = manifest.manifest_data.get("kinematics_profile")
    if payload is not None:
        if not isinstance(payload, dict):
            raise ValueError(
                f"Field 'kinematics_profile' must be an object in {manifest.manifest_path}"
            )
        return dict(payload)
    return _build_legacy_kinematics_profile(
        robot_id=manifest.robot_id,
        manifest_path=manifest.manifest_path,
        manifest_data=manifest.manifest_data,
    )


def load_kinematics_profile(robot_id: str | None = None, backend_name: str | None = None):
    """
    Load and validate a kinematics profile for the selected robot.

    Raises profile-specific validation errors instead of silently falling back.
    """
    from .kinematics.profile import build_profile_from_payload, validate_profile_for_backend

    resolved_robot_id = robot_id or get_active_robot_id()
    payload = get_kinematics_profile_payload(resolved_robot_id)
    profile = build_profile_from_payload(payload, expected_robot_id=resolved_robot_id)
    if backend_name is not None:
        validate_profile_for_backend(profile, backend_name)
    return profile
