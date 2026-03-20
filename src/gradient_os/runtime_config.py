from __future__ import annotations

import datetime
import json
import os
from typing import Any

from . import robot_assets
from . import tool_library
from .arm_controller.backends import registry as backend_registry
from .arm_controller.robots import (
    get_robot_config,
    get_robot_name_by_id,
    list_available_robots,
)

RUNTIME_CONFIG_ENV_VAR = "GRADIENT_RUNTIME_CONFIG_PATH"
ALLOW_UNSAFE_ENV_VAR = "GRADIENT_ALLOW_UNSAFE_OVERRIDES"
DEFAULT_RUNTIME_CONFIG_BASENAME = ".gradient_runtime_config.json"


def _utc_now_iso() -> str:
    return datetime.datetime.now(datetime.UTC).isoformat(timespec="seconds")


def get_runtime_config_path() -> str:
    configured = os.environ.get(RUNTIME_CONFIG_ENV_VAR, "").strip()
    if configured:
        return os.path.abspath(configured)
    repo_root = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
    return os.path.join(repo_root, DEFAULT_RUNTIME_CONFIG_BASENAME)


def _default_robot_name() -> str:
    default_robot_id = robot_assets.get_default_robot_id()
    mapped = get_robot_name_by_id(default_robot_id)
    if mapped:
        return mapped
    available = list_available_robots()
    if not available:
        raise RuntimeError("No robot configurations are registered.")
    return available[0]


def _empty_runtime_config() -> dict[str, Any]:
    return {
        "version": 1,
        "desired": {
            "robot": _default_robot_name(),
            "active_tool_id": None,
            "allow_unsafe_overrides": False,
            "overrides": {
                "ik_solver_backend": None,
                "servo_backend": None,
                "drive_profile": None,
            },
        },
        "meta": {
            "updated_at": _utc_now_iso(),
            "updated_by": "system-default",
        },
    }


def _normalize_robot_name(raw: Any, *, strict: bool = False) -> str:
    token = str(raw or "").strip()
    if token:
        if token in list_available_robots():
            return token
        by_id = get_robot_name_by_id(token)
        if by_id:
            return by_id
        if strict:
            raise ValueError(
                f"Unknown robot selector '{token}'. Expected registry name or robot_id."
            )
    return _default_robot_name()


def _normalize_ik_backend(raw: Any) -> str | None:
    if raw is None:
        return None
    token = str(raw).strip().lower()
    if token in {"", "none", "null"}:
        return None
    if token not in {"ikfast", "numeric"}:
        raise ValueError(f"Unsupported ik_solver_backend '{token}'.")
    return token


def _normalize_servo_backend(raw: Any) -> str | None:
    if raw is None:
        return None
    token = str(raw).strip().lower()
    if token in {"", "none", "null"}:
        return None
    return token


def _normalize_drive_profile(raw: Any) -> str | None:
    if raw is None:
        return None
    token = str(raw).strip().lower()
    if token in {"", "none", "null"}:
        return None
    return token


def _normalize_tool_id(raw: Any) -> str | None:
    if raw is None:
        return None
    token = str(raw).strip().lower()
    if token in {"", "none", "null"}:
        return None
    return token


def _normalize_runtime_config(raw: Any) -> dict[str, Any]:
    defaults = _empty_runtime_config()
    if not isinstance(raw, dict):
        return defaults

    desired = raw.get("desired")
    desired_obj = desired if isinstance(desired, dict) else {}
    overrides_raw = desired_obj.get("overrides")
    overrides_obj = overrides_raw if isinstance(overrides_raw, dict) else {}

    return {
        "version": 1,
        "desired": {
            "robot": _normalize_robot_name(desired_obj.get("robot", defaults["desired"]["robot"])),
            "active_tool_id": _normalize_tool_id(
                desired_obj.get("active_tool_id", defaults["desired"]["active_tool_id"])
            ),
            "allow_unsafe_overrides": bool(
                desired_obj.get(
                    "allow_unsafe_overrides",
                    defaults["desired"]["allow_unsafe_overrides"],
                )
            ),
            "overrides": {
                "ik_solver_backend": _normalize_ik_backend(
                    overrides_obj.get("ik_solver_backend")
                ),
                "servo_backend": _normalize_servo_backend(overrides_obj.get("servo_backend")),
                "drive_profile": _normalize_drive_profile(overrides_obj.get("drive_profile")),
            },
        },
        "meta": {
            "updated_at": str(raw.get("meta", {}).get("updated_at", defaults["meta"]["updated_at"])),
            "updated_by": str(raw.get("meta", {}).get("updated_by", defaults["meta"]["updated_by"])),
        },
    }


def load_runtime_config() -> dict[str, Any]:
    path = get_runtime_config_path()
    if not os.path.exists(path):
        return _empty_runtime_config()
    try:
        with open(path, "r", encoding="utf-8") as handle:
            payload = json.load(handle)
    except Exception:
        return _empty_runtime_config()
    return _normalize_runtime_config(payload)


def save_runtime_config(config: dict[str, Any], *, actor: str = "unknown") -> dict[str, Any]:
    normalized = _normalize_runtime_config(config)
    normalized["meta"] = {
        "updated_at": _utc_now_iso(),
        "updated_by": actor,
    }
    path = get_runtime_config_path()
    dirpath = os.path.dirname(path)
    if dirpath:
        os.makedirs(dirpath, exist_ok=True)
    temp_path = f"{path}.tmp"
    with open(temp_path, "w", encoding="utf-8") as handle:
        json.dump(normalized, handle, indent=2)
        handle.write("\n")
    os.replace(temp_path, path)
    return normalized


def update_runtime_config_desired(
    patch: dict[str, Any],
    *,
    actor: str = "unknown",
) -> dict[str, Any]:
    current = load_runtime_config()
    desired = dict(current.get("desired", {}))
    overrides = dict(desired.get("overrides", {}))

    if "robot" in patch:
        desired["robot"] = _normalize_robot_name(patch.get("robot"), strict=True)

    if "allow_unsafe_overrides" in patch:
        desired["allow_unsafe_overrides"] = bool(patch.get("allow_unsafe_overrides"))

    if "active_tool_id" in patch:
        desired["active_tool_id"] = _normalize_tool_id(patch.get("active_tool_id"))

    patch_overrides = patch.get("overrides")
    if isinstance(patch_overrides, dict):
        if "ik_solver_backend" in patch_overrides:
            overrides["ik_solver_backend"] = _normalize_ik_backend(
                patch_overrides.get("ik_solver_backend")
            )
        if "servo_backend" in patch_overrides:
            overrides["servo_backend"] = _normalize_servo_backend(
                patch_overrides.get("servo_backend")
            )
        if "drive_profile" in patch_overrides:
            overrides["drive_profile"] = _normalize_drive_profile(
                patch_overrides.get("drive_profile")
            )
    desired["overrides"] = overrides
    current["desired"] = desired
    return save_runtime_config(current, actor=actor)


def resolve_allow_unsafe_overrides(
    *,
    cli_flag: bool = False,
    desired_flag: bool = False,
) -> bool:
    env_flag = os.environ.get(ALLOW_UNSAFE_ENV_VAR, "").strip().lower() in {
        "1",
        "true",
        "yes",
        "on",
    }
    return bool(cli_flag or desired_flag or env_flag)


def resolve_effective_runtime(
    *,
    robot_name: str,
    sim_mode: bool,
    requested_ik_solver_backend: str | None = None,
    requested_servo_backend: str | None = None,
    requested_drive_profile: str | None = None,
    requested_active_tool_id: str | None = None,
    allow_unsafe_overrides: bool = False,
) -> dict[str, Any]:
    robot = get_robot_config(robot_name)
    effective_ik = robot.default_ik_solver_backend
    ik_source = "robot_policy"
    override_ik = _normalize_ik_backend(requested_ik_solver_backend)
    if override_ik and allow_unsafe_overrides:
        effective_ik = override_ik
        ik_source = "dev_override"

    effective_servo = "simulation" if sim_mode else robot.default_servo_backend
    servo_source = "sim_mode" if sim_mode else "robot_policy"
    override_servo = _normalize_servo_backend(requested_servo_backend)
    if override_servo and allow_unsafe_overrides:
        # Sim mode remains simulation for safety/determinism.
        if not sim_mode:
            effective_servo = override_servo
            servo_source = "dev_override"

    backend_default_drive_profile = (
        None
        if sim_mode
        else backend_registry.get_default_drive_profile_for_backend(effective_servo)
    )
    effective_drive_profile = backend_default_drive_profile
    drive_profile_source = "sim_mode" if sim_mode else "backend_default"
    override_drive_profile = _normalize_drive_profile(requested_drive_profile)
    if override_drive_profile and allow_unsafe_overrides and not sim_mode:
        effective_drive_profile = override_drive_profile
        drive_profile_source = "dev_override"

    selected_tool = tool_library.resolve_active_tool(
        robot_id=robot.robot_id,
        requested_tool_id=_normalize_tool_id(requested_active_tool_id),
    )

    return {
        "robot": {
            "name": robot_name,
            "robot_id": robot.robot_id,
            "display_name": robot.name,
            "version": robot.version,
        },
        "mode": {
            "sim": bool(sim_mode),
        },
        "ik_solver": {
            "effective_backend": effective_ik,
            "source": ik_source,
            "robot_default_backend": robot.default_ik_solver_backend,
            "override_backend": override_ik,
        },
        "servo_backend": {
            "effective_backend": effective_servo,
            "source": servo_source,
            "robot_default_backend": robot.default_servo_backend,
            "override_backend": override_servo,
        },
        "drive_profile": {
            "configured_profile": effective_drive_profile,
            "configured_source": drive_profile_source,
            "live_profile": None,
            "live_source": None,
            "effective_profile": effective_drive_profile,
            "source": drive_profile_source,
            "backend_default_profile": backend_default_drive_profile,
            "override_profile": override_drive_profile,
        },
        "tool": {
            "active_tool_id": selected_tool.get("tool_id"),
            "display_name": selected_tool.get("display_name"),
            "tool_type": selected_tool.get("tool_type"),
            "source": selected_tool.get("selection_source"),
            "offset": selected_tool.get("offset"),
            "mesh": selected_tool.get("mesh"),
            "compatible_robot_ids": selected_tool.get("compatible_robot_ids"),
            "weld": selected_tool.get("weld"),
        },
        "allow_unsafe_overrides": bool(allow_unsafe_overrides),
    }


def attach_live_drive_profile(
    runtime: dict[str, Any] | None,
    live_profile: Any,
    *,
    live_source: str = "rtcore_status_hello",
) -> dict[str, Any] | None:
    if not isinstance(runtime, dict):
        return runtime
    drive = runtime.get("drive_profile")
    drive_block = dict(drive) if isinstance(drive, dict) else {}

    configured_profile = _normalize_drive_profile(
        drive_block.get("configured_profile", drive_block.get("effective_profile"))
    )
    configured_source_raw = drive_block.get("configured_source", drive_block.get("source"))
    configured_source = str(configured_source_raw).strip() if configured_source_raw is not None else ""
    live_profile_token = _normalize_drive_profile(live_profile)

    drive_block["configured_profile"] = configured_profile
    drive_block["configured_source"] = configured_source or None
    drive_block["live_profile"] = live_profile_token
    drive_block["live_source"] = live_source if live_profile_token else None
    drive_block["effective_profile"] = live_profile_token or configured_profile
    drive_block["source"] = (live_source if live_profile_token else configured_source) or None
    runtime["drive_profile"] = drive_block
    return runtime


def _active_fields(
    runtime: dict[str, Any] | None,
) -> tuple[str | None, str | None, str | None, str | None, str | None]:
    if not isinstance(runtime, dict):
        return (None, None, None, None, None)
    robot_name = runtime.get("robot", {}).get("name") if isinstance(runtime.get("robot"), dict) else None
    ik_backend = runtime.get("ik_solver", {}).get("effective_backend") if isinstance(runtime.get("ik_solver"), dict) else None
    servo_backend = runtime.get("servo_backend", {}).get("effective_backend") if isinstance(runtime.get("servo_backend"), dict) else None
    drive_block = runtime.get("drive_profile", {}) if isinstance(runtime.get("drive_profile"), dict) else {}
    drive_profile = (
        drive_block.get("configured_profile", drive_block.get("effective_profile"))
        if isinstance(drive_block, dict)
        else None
    )
    active_tool_id = runtime.get("tool", {}).get("active_tool_id") if isinstance(runtime.get("tool"), dict) else None
    return (
        str(robot_name) if isinstance(robot_name, str) else None,
        str(ik_backend) if isinstance(ik_backend, str) else None,
        str(servo_backend) if isinstance(servo_backend, str) else None,
        str(drive_profile) if isinstance(drive_profile, str) else None,
        str(active_tool_id) if isinstance(active_tool_id, str) else None,
    )


def compute_restart_required(active_runtime: dict[str, Any] | None, desired_config: dict[str, Any]) -> bool:
    active_robot, active_ik, active_servo, active_drive_profile, _active_tool_id = _active_fields(active_runtime)
    active_mode = active_runtime.get("mode", {}) if isinstance(active_runtime, dict) else {}
    desired_obj = desired_config.get("desired", {}) if isinstance(desired_config, dict) else {}
    desired_robot_name = _normalize_robot_name(desired_obj.get("robot"))
    desired_overrides = desired_obj.get("overrides", {}) if isinstance(desired_obj.get("overrides"), dict) else {}
    desired_allow_unsafe = resolve_allow_unsafe_overrides(
        cli_flag=False,
        desired_flag=bool(desired_obj.get("allow_unsafe_overrides", False)),
    )
    desired_runtime = resolve_effective_runtime(
        robot_name=desired_robot_name,
        sim_mode=bool(active_mode.get("sim", False)) if isinstance(active_mode, dict) else False,
        requested_ik_solver_backend=desired_overrides.get("ik_solver_backend"),
        requested_servo_backend=desired_overrides.get("servo_backend"),
        requested_drive_profile=desired_overrides.get("drive_profile"),
        requested_active_tool_id=desired_obj.get("active_tool_id"),
        allow_unsafe_overrides=desired_allow_unsafe,
    )
    desired_ik = desired_runtime.get("ik_solver", {}).get("effective_backend")
    desired_servo = desired_runtime.get("servo_backend", {}).get("effective_backend")
    desired_drive_profile = desired_runtime.get("drive_profile", {}).get("effective_profile")
    if active_robot is None:
        return False
    return (
        active_robot != desired_robot_name
        or active_ik != desired_ik
        or active_servo != desired_servo
        or active_drive_profile != desired_drive_profile
    )

