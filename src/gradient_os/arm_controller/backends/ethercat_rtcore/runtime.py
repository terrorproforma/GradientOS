from __future__ import annotations

from typing import Any

RTCORE_DRIVE_PROFILE_UNKNOWN = 0
RTCORE_DRIVE_PROFILE_A6EC_DS402 = 1
RTCORE_DRIVE_PROFILE_CIA402 = 2
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

RTCORE_DRIVE_PROFILE_NAME_TO_ID: dict[str, int] = {
    "a6ec_ds402": RTCORE_DRIVE_PROFILE_A6EC_DS402,
    "cia402": RTCORE_DRIVE_PROFILE_CIA402,
}

RTCORE_DRIVE_PROFILE_ID_TO_NAME: dict[int, str] = {
    value: key for key, value in RTCORE_DRIVE_PROFILE_NAME_TO_ID.items()
}

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


def normalize_drive_profile_id(profile_id: object | None) -> str | None:
    token = str(profile_id or "").strip().lower()
    if not token:
        return None
    if token.isdigit():
        return RTCORE_DRIVE_PROFILE_ID_TO_NAME.get(int(token))
    return token


def rtcore_drive_profile_name_to_id(profile_id: object | None) -> int:
    token = normalize_drive_profile_id(profile_id)
    if not token:
        return RTCORE_DRIVE_PROFILE_UNKNOWN
    return int(RTCORE_DRIVE_PROFILE_NAME_TO_ID.get(token, RTCORE_DRIVE_PROFILE_UNKNOWN))


def rtcore_drive_profile_id_to_name(profile_code: object | None) -> str | None:
    try:
        code = int(profile_code)
    except Exception:
        return None
    return RTCORE_DRIVE_PROFILE_ID_TO_NAME.get(code)


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


def build_rtcore_axis_scaling(robot_config: dict[str, Any]) -> dict[str, Any]:
    counts_per_rev = [int(value) for value in list(robot_config.get("actuator_encoder_counts_per_rev", []))]
    gear_ratios = [float(value) for value in list(robot_config.get("actuator_gear_ratios", []))]
    signs_raw = list(robot_config.get("actuator_position_signs", []))

    num_axes = min(
        int(robot_config.get("num_physical_actuators", min(len(counts_per_rev), len(gear_ratios)))),
        len(counts_per_rev),
        len(gear_ratios),
    )
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
    }


def build_rtcore_startup_env(
    *,
    robot_config: dict[str, Any],
    drive_profile: str | None,
    max_rpm: float | int | None = None,
) -> dict[str, str]:
    axis_scaling = build_rtcore_axis_scaling(robot_config)
    normalized_profile = normalize_drive_profile_id(drive_profile)
    if normalized_profile and rtcore_drive_profile_name_to_id(normalized_profile) == RTCORE_DRIVE_PROFILE_UNKNOWN:
        raise ValueError(f"Unsupported RTCore drive profile '{drive_profile}'.")
    resolved_max_rpm = DEFAULT_RT_MAX_RPM if max_rpm is None else float(max_rpm)
    if resolved_max_rpm < 0.0:
        raise ValueError("RTCore max_rpm must be >= 0.")

    return {
        "GRADIENT_RT_NUM_AXES": str(int(axis_scaling["num_axes"])),
        "GRADIENT_RT_COUNTS_PER_REV": ",".join(str(int(value)) for value in axis_scaling["counts_per_rev"]),
        "GRADIENT_RT_GEAR_RATIO": ",".join(f"{float(value):g}" for value in axis_scaling["gear_ratio"]),
        "GRADIENT_RT_SIGN": ",".join(str(int(value)) for value in axis_scaling["sign"]),
        "GRADIENT_RT_DRIVE_PROFILE": normalized_profile or "",
        "GRADIENT_RT_MAX_RPM": f"{resolved_max_rpm:g}",
    }


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
