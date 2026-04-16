from __future__ import annotations

from typing import Any

from ...ethercat_drive_catalog import list_ethercat_drive_profiles, render_ethercat_drive_rtcore_env
from .. import registry as backend_registry

RTCORE_DRIVE_PROFILE_UNKNOWN = 0
DEFAULT_RT_MAX_RPM = 6000.0

RTCORE_MOTION_MODE_IDLE = 0
RTCORE_MOTION_MODE_LEGACY_SETPOINT = 1
RTCORE_MOTION_MODE_TRAJECTORY = 2
RTCORE_MOTION_MODE_JOG = 3

RTCORE_EXEC_STATE_IDLE = 0
RTCORE_EXEC_STATE_ACCEPTED = 1
RTCORE_EXEC_STATE_QUEUED = 2
RTCORE_EXEC_STATE_EXECUTING = 3
RTCORE_EXEC_STATE_COMPLETED = 4
RTCORE_EXEC_STATE_ABORTED = 5
RTCORE_EXEC_STATE_FAULTED = 6
RTCORE_EXEC_STATE_UNDERRUN = 7

RTCORE_MOTION_CAP_LEGACY_SETPOINT = 1 << 0
RTCORE_MOTION_CAP_TRAJECTORY_UPLOAD = 1 << 1
RTCORE_MOTION_CAP_JOG_COMMAND = 1 << 2

RTCORE_JOG_STOP_REASON_NONE = 0
RTCORE_JOG_STOP_REASON_CMD_STOP = 1
RTCORE_JOG_STOP_REASON_TIMEOUT = 2
RTCORE_JOG_STOP_REASON_TRAJECTORY_PREEMPT = 3

RTCORE_MOTION_MODE_ID_TO_NAME: dict[int, str] = {
    RTCORE_MOTION_MODE_IDLE: "idle",
    RTCORE_MOTION_MODE_LEGACY_SETPOINT: "legacy_setpoint",
    RTCORE_MOTION_MODE_TRAJECTORY: "trajectory",
    RTCORE_MOTION_MODE_JOG: "jog",
}

RTCORE_EXEC_STATE_ID_TO_NAME: dict[int, str] = {
    RTCORE_EXEC_STATE_IDLE: "idle",
    RTCORE_EXEC_STATE_ACCEPTED: "accepted",
    RTCORE_EXEC_STATE_QUEUED: "queued",
    RTCORE_EXEC_STATE_EXECUTING: "executing",
    RTCORE_EXEC_STATE_COMPLETED: "completed",
    RTCORE_EXEC_STATE_ABORTED: "aborted",
    RTCORE_EXEC_STATE_FAULTED: "faulted",
    RTCORE_EXEC_STATE_UNDERRUN: "underrun",
}

RTCORE_JOG_STOP_REASON_ID_TO_NAME: dict[int, str] = {
    RTCORE_JOG_STOP_REASON_NONE: "none",
    RTCORE_JOG_STOP_REASON_CMD_STOP: "cmd_stop",
    RTCORE_JOG_STOP_REASON_TIMEOUT: "timeout",
    RTCORE_JOG_STOP_REASON_TRAJECTORY_PREEMPT: "trajectory_preempt",
}

RTCORE_FEEDBACK_WRAP_AXIS_MASK_ENV_VAR = "GRADIENT_RT_FEEDBACK_WRAP_AXIS_MASK"


def _rtcore_drive_profile_hash(token: str) -> int:
    value = 0x811C9DC5
    for ch in token:
        value ^= ord(ch)
        value = (value * 0x01000193) & 0xFFFFFFFF
    return value or RTCORE_DRIVE_PROFILE_UNKNOWN


def normalize_drive_profile_id(profile_id: object | None) -> str | None:
    token = str(profile_id or "").strip().lower()
    if not token:
        return None
    if token.isdigit() or token.startswith("0x"):
        resolved = rtcore_drive_profile_id_to_name(int(token, 0))
        return resolved
    return token


def rtcore_drive_profile_name_to_id(profile_id: object | None) -> int:
    token = normalize_drive_profile_id(profile_id)
    if not token:
        return RTCORE_DRIVE_PROFILE_UNKNOWN
    return _rtcore_drive_profile_hash(token)


def rtcore_drive_profile_id_to_name(profile_code: object | None) -> str | None:
    try:
        code = int(profile_code)
    except Exception:
        return None
    if code == RTCORE_DRIVE_PROFILE_UNKNOWN:
        return None
    for candidate in list_ethercat_drive_profiles():
        if _rtcore_drive_profile_hash(candidate) == code:
            return candidate
    return None


def rtcore_motion_mode_id_to_name(mode_code: object | None) -> str | None:
    try:
        code = int(mode_code)
    except Exception:
        return None
    return RTCORE_MOTION_MODE_ID_TO_NAME.get(code)


def rtcore_execution_state_id_to_name(state_code: object | None) -> str | None:
    try:
        code = int(state_code)
    except Exception:
        return None
    return RTCORE_EXEC_STATE_ID_TO_NAME.get(code)


def rtcore_jog_stop_reason_id_to_name(reason_code: object | None) -> str | None:
    try:
        code = int(reason_code)
    except Exception:
        return None
    return RTCORE_JOG_STOP_REASON_ID_TO_NAME.get(code)


def _drive_native_ratio_enabled_for_profile(profile_id: str | None) -> bool:
    payload = backend_registry.get_drive_position_semantics_config_for_backend(
        "ethercat_rtcore",
        drive_profile_id=profile_id,
    )
    return bool(payload.get("drive_native_ratio_enabled", False)) if isinstance(payload, dict) else False


def build_rtcore_axis_scaling(
    robot_config: dict[str, Any],
    *,
    drive_profile: str | None = None,
) -> dict[str, Any]:
    normalized_profile = normalize_drive_profile_id(drive_profile)
    drive_native_ratio_enabled = _drive_native_ratio_enabled_for_profile(normalized_profile)
    counts_per_rev = [int(value) for value in list(robot_config.get("actuator_encoder_counts_per_rev", []))]
    gear_ratios = [float(value) for value in list(robot_config.get("actuator_gear_ratios", []))]
    signs_raw = list(robot_config.get("actuator_position_signs", []))

    max_axes = int(robot_config.get("num_physical_actuators", min(len(counts_per_rev), len(gear_ratios))))
    num_axes = min(max_axes, len(counts_per_rev), len(gear_ratios))
    if num_axes <= 0:
        raise ValueError("Robot config does not expose valid RTCore axis scaling.")

    signs: list[int] = []
    for idx in range(num_axes):
        raw_sign = int(signs_raw[idx]) if idx < len(signs_raw) else 1
        signs.append(1 if raw_sign >= 0 else -1)

    return {
        "num_axes": num_axes,
        "counts_per_rev": counts_per_rev[:num_axes],
        "gear_ratio": gear_ratios[:num_axes],
        "sign": signs,
        "drive_native_ratio_enabled": drive_native_ratio_enabled,
    }


def build_rtcore_drive_startup_config(
    robot_config: dict[str, Any],
    *,
    drive_profile: str | None,
) -> dict[str, Any]:
    axis_scaling = build_rtcore_axis_scaling(
        robot_config,
        drive_profile=drive_profile,
    )
    num_axes = int(axis_scaling["num_axes"])
    raw_entries = robot_config.get("ethercat_drive_startup_config", [])
    if isinstance(raw_entries, list) and all(
        isinstance(entry, dict) and not entry for entry in raw_entries
    ):
        raw_entries = []
    normalized_profile = normalize_drive_profile_id(drive_profile)
    if normalized_profile and rtcore_drive_profile_name_to_id(normalized_profile) == RTCORE_DRIVE_PROFILE_UNKNOWN:
        raise ValueError(f"Unsupported RTCore drive profile '{drive_profile}'.")
    startup = backend_registry.build_drive_startup_config_for_backend(
        "ethercat_rtcore",
        raw_entries,
        num_axes=num_axes,
        drive_profile_id=normalized_profile,
        robot_config=robot_config,
    )
    if isinstance(startup, dict):
        return startup
    return {"profile_id": normalized_profile, "settings": {}, "env": {}}


def build_rtcore_drive_profile_env(
    *,
    drive_profile: str | None,
) -> dict[str, str]:
    normalized_profile = normalize_drive_profile_id(drive_profile)
    rendered = render_ethercat_drive_rtcore_env(normalized_profile)
    if normalized_profile and not rendered:
        raise ValueError(
            f"RTCore drive profile '{normalized_profile}' does not define an EtherCAT drive catalog entry."
        )
    rendered.setdefault("GRADIENT_RT_DRIVE_VENDOR_ID", "")
    rendered.setdefault("GRADIENT_RT_DRIVE_PRODUCT_CODE", "")
    rendered.setdefault("GRADIENT_RT_DRIVE_REVISION_NO", "")
    rendered.setdefault("GRADIENT_RT_DRIVE_RX_SYNC_INDEX", "2")
    rendered.setdefault("GRADIENT_RT_DRIVE_TX_SYNC_INDEX", "3")
    rendered.setdefault("GRADIENT_RT_DRIVE_DC_CYCLE_MULTIPLE_NS", "0")
    rendered.setdefault("GRADIENT_RT_DRIVE_RX_PDO", "")
    rendered.setdefault("GRADIENT_RT_DRIVE_TX_PDO", "")
    rendered.setdefault("GRADIENT_RT_DRIVE_RX_PDO_LAYOUT", "")
    rendered.setdefault("GRADIENT_RT_DRIVE_TX_PDO_LAYOUT", "")
    return rendered


def build_rtcore_native_home_env(
    *,
    drive_profile: str | None,
) -> dict[str, str]:
    normalized_profile = normalize_drive_profile_id(drive_profile)
    payload = backend_registry.get_drive_native_home_config_for_backend(
        "ethercat_rtcore",
        drive_profile_id=normalized_profile,
    )
    env = dict(payload.get("env", {})) if isinstance(payload, dict) else {}
    env.setdefault("GRADIENT_RT_NATIVE_HOME_CONFIG", "")
    return {str(key): str(value) for key, value in env.items()}


def build_rtcore_absolute_feedback_env(
    *,
    drive_profile: str | None,
) -> dict[str, str]:
    normalized_profile = normalize_drive_profile_id(drive_profile)
    payload = backend_registry.get_drive_absolute_feedback_config_for_backend(
        "ethercat_rtcore",
        drive_profile_id=normalized_profile,
    )
    env = dict(payload.get("env", {})) if isinstance(payload, dict) else {}
    env.setdefault("GRADIENT_RT_ABSOLUTE_FEEDBACK_CONFIG", "")
    return {str(key): str(value) for key, value in env.items()}


def build_rtcore_motion_feedback_env(
    *,
    num_axes: int,
    drive_profile: str | None,
) -> dict[str, str]:
    normalized_profile = normalize_drive_profile_id(drive_profile)
    payload = backend_registry.get_drive_motion_feedback_config_for_backend(
        "ethercat_rtcore",
        drive_profile_id=normalized_profile,
    )
    wrap_mask = 0
    if isinstance(payload, dict):
        raw_mask = payload.get("feedback_wrap_axis_mask")
        if raw_mask is not None:
            try:
                wrap_mask = int(raw_mask, 0) if isinstance(raw_mask, str) else int(raw_mask)
            except Exception as exc:
                raise ValueError(
                    f"Invalid feedback_wrap_axis_mask for RTCore drive profile '{normalized_profile}'."
                ) from exc
        else:
            raw_axes = payload.get("feedback_wrap_axes")
            if isinstance(raw_axes, list):
                for raw_axis in raw_axes:
                    axis_i = int(raw_axis)
                    if axis_i < 0 or axis_i >= int(num_axes):
                        raise ValueError(
                            f"feedback_wrap_axes entry {axis_i} is out of range for num_axes={int(num_axes)}."
                        )
                    wrap_mask |= 1 << axis_i
            elif bool(payload.get("feedback_counts_wrap")):
                wrap_mask = (1 << int(num_axes)) - 1 if int(num_axes) > 0 else 0
    valid_mask = (1 << int(num_axes)) - 1 if int(num_axes) > 0 else 0
    wrap_mask &= valid_mask
    return {
        RTCORE_FEEDBACK_WRAP_AXIS_MASK_ENV_VAR: f"0x{wrap_mask:x}",
    }


def build_rtcore_startup_env(
    *,
    robot_config: dict[str, Any],
    drive_profile: str | None,
    max_rpm: float | int | None = None,
) -> dict[str, str]:
    normalized_profile = normalize_drive_profile_id(drive_profile)
    if normalized_profile and rtcore_drive_profile_name_to_id(normalized_profile) == RTCORE_DRIVE_PROFILE_UNKNOWN:
        raise ValueError(f"Unsupported RTCore drive profile '{drive_profile}'.")
    axis_scaling = build_rtcore_axis_scaling(
        robot_config,
        drive_profile=normalized_profile,
    )
    startup_config = build_rtcore_drive_startup_config(
        robot_config,
        drive_profile=normalized_profile,
    )
    resolved_max_rpm = DEFAULT_RT_MAX_RPM if max_rpm is None else float(max_rpm)
    if resolved_max_rpm < 0.0:
        raise ValueError("RTCore max_rpm must be >= 0.")

    env = {
        "GRADIENT_RT_NUM_AXES": str(int(axis_scaling["num_axes"])),
        "GRADIENT_RT_COUNTS_PER_REV": ",".join(str(int(value)) for value in axis_scaling["counts_per_rev"]),
        "GRADIENT_RT_GEAR_RATIO": ",".join(f"{float(value):g}" for value in axis_scaling["gear_ratio"]),
        "GRADIENT_RT_SIGN": ",".join(str(int(value)) for value in axis_scaling["sign"]),
        "GRADIENT_RT_DRIVE_PROFILE": normalized_profile or "",
        "GRADIENT_RT_MAX_RPM": f"{resolved_max_rpm:g}",
    }
    env.update(build_rtcore_drive_profile_env(drive_profile=normalized_profile))
    env.update(build_rtcore_native_home_env(drive_profile=normalized_profile))
    env.update(build_rtcore_absolute_feedback_env(drive_profile=normalized_profile))
    env.update(
        build_rtcore_motion_feedback_env(
            num_axes=int(axis_scaling["num_axes"]),
            drive_profile=normalized_profile,
        )
    )
    for key, value in (startup_config.get("env") if isinstance(startup_config, dict) else {}).items():
        env[str(key)] = str(value)
    env.setdefault("GRADIENT_RT_DRIVE_STARTUP_SDO_CONFIG", "")
    env.setdefault("GRADIENT_RT_NATIVE_HOME_CONFIG", "")
    env.setdefault("GRADIENT_RT_ABSOLUTE_FEEDBACK_CONFIG", "")
    env.setdefault(RTCORE_FEEDBACK_WRAP_AXIS_MASK_ENV_VAR, "0x0")
    return env


def render_rtcore_systemd_env(
    *,
    robot_config: dict[str, Any],
    drive_profile: str | None,
    max_rpm: float | int | None = None,
) -> str:
    env = build_rtcore_startup_env(
        robot_config=robot_config,
        drive_profile=drive_profile,
        max_rpm=max_rpm,
    )
    lines = [
        "# Generated by systemd/rt-motion/install.sh",
        "# Regenerate after robot/runtime drive-profile changes.",
    ]
    for key in sorted(env.keys()):
        value = env[key].replace("\\", "\\\\").replace('"', '\\"')
        lines.append(f'{key}="{value}"')
    lines.append("")
    return "\n".join(lines)
