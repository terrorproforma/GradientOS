from __future__ import annotations

import datetime
import json
import os
import re
from pathlib import Path
from typing import Any, Mapping

PHASE_BEFORE_POWER_DOWN = "before_power_down"
PHASE_AFTER_POWER_UP = "after_power_up"
_VALID_PHASES = {PHASE_BEFORE_POWER_DOWN, PHASE_AFTER_POWER_UP}


def _repo_root() -> Path:
    return Path(__file__).resolve().parents[3]


def get_encoder_retention_log_dir() -> Path:
    return _repo_root() / "logs" / "encoder-retention"


def _utc_now() -> datetime.datetime:
    return datetime.datetime.now(datetime.timezone.utc)


def _timestamp_token(moment: datetime.datetime | None = None) -> str:
    current = moment or _utc_now()
    return current.astimezone(datetime.timezone.utc).strftime("%Y%m%d-%H%M%S")


def normalize_retention_phase(value: object) -> str:
    token = str(value or "").strip().lower().replace("-", "_").replace(" ", "_")
    if token not in _VALID_PHASES:
        raise ValueError(
            f"phase must be one of: {PHASE_BEFORE_POWER_DOWN}, {PHASE_AFTER_POWER_UP}"
        )
    return token


def normalize_experiment_id(value: object | None) -> str | None:
    token = str(value or "").strip()
    if not token:
        return None
    normalized = re.sub(r"[^a-zA-Z0-9_.-]+", "-", token).strip("-._")
    if not normalized:
        raise ValueError("experiment_id must contain at least one safe filename character.")
    return normalized


def resolve_experiment_id(*, phase: str, experiment_id: str | None = None) -> str:
    normalized = normalize_experiment_id(experiment_id)
    if normalized:
        return normalized
    log_dir = get_encoder_retention_log_dir()
    if phase == PHASE_AFTER_POWER_UP and log_dir.exists():
        candidates = sorted(
            (
                path.name
                for path in log_dir.iterdir()
                if path.is_dir()
                and (path / f"{PHASE_BEFORE_POWER_DOWN}.json").exists()
                and not (path / f"{PHASE_AFTER_POWER_UP}.json").exists()
            ),
            reverse=True,
        )
        if candidates:
            return candidates[0]
    return _timestamp_token()


def _write_json(path: Path, payload: Mapping[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temp_path = path.with_suffix(f"{path.suffix}.tmp")
    with open(temp_path, "w", encoding="utf-8") as handle:
        json.dump(payload, handle, indent=2, sort_keys=True)
        handle.write("\n")
    os.replace(temp_path, path)


def _json_clone(payload: Any) -> Any:
    return json.loads(json.dumps(payload))


def _float_close(left: float | None, right: float | None, tolerance: float = 1e-9) -> bool:
    if left is None or right is None:
        return left is None and right is None
    return abs(float(left) - float(right)) <= tolerance


def _collect_fault_matches(snapshot: Mapping[str, Any] | None) -> list[dict[str, Any]]:
    if not isinstance(snapshot, Mapping):
        return []
    axes = snapshot.get("axes")
    if not isinstance(axes, list):
        return []
    matches: list[dict[str, Any]] = []
    for axis in axes:
        if not isinstance(axis, Mapping):
            continue
        fault = axis.get("manufacturer_fault") if isinstance(axis.get("manufacturer_fault"), Mapping) else None
        name = str((fault or {}).get("name", "")).strip().lower()
        code = str((fault or {}).get("code", "")).strip()
        if not name and not code:
            continue
        if "battery" in name or "multi-turn" in name or "multiturn" in name:
            matches.append(
                {
                    "axis": axis.get("axis"),
                    "logical_joint": axis.get("logical_joint"),
                    "manufacturer_error_code": axis.get("manufacturer_error_code"),
                    "manufacturer_error_code_hex": axis.get("manufacturer_error_code_hex"),
                    "code": code or None,
                    "name": (fault or {}).get("name"),
                }
            )
    return matches


def compare_retention_snapshots(
    *,
    before_snapshot: Mapping[str, Any],
    after_snapshot: Mapping[str, Any],
) -> dict[str, Any]:
    before_joint_state = before_snapshot.get("joint_state") if isinstance(before_snapshot.get("joint_state"), Mapping) else {}
    after_joint_state = after_snapshot.get("joint_state") if isinstance(after_snapshot.get("joint_state"), Mapping) else {}
    before_axis_counts = list(before_joint_state.get("axis_counts", [])) if isinstance(before_joint_state.get("axis_counts"), list) else []
    after_axis_counts = list(after_joint_state.get("axis_counts", [])) if isinstance(after_joint_state.get("axis_counts"), list) else []
    before_arm_rad = list(before_joint_state.get("arm_rad", [])) if isinstance(before_joint_state.get("arm_rad"), list) else []
    after_arm_rad = list(after_joint_state.get("arm_rad", [])) if isinstance(after_joint_state.get("arm_rad"), list) else []

    axis_count = max(len(before_axis_counts), len(after_axis_counts))
    joint_count = max(len(before_arm_rad), len(after_arm_rad))

    axis_comparison: list[dict[str, Any]] = []
    raw_encoder_mismatch = False
    for axis_index in range(axis_count):
        before_value = before_axis_counts[axis_index] if axis_index < len(before_axis_counts) else None
        after_value = after_axis_counts[axis_index] if axis_index < len(after_axis_counts) else None
        match = before_value == after_value
        raw_encoder_mismatch = raw_encoder_mismatch or not match
        axis_comparison.append(
            {
                "axis": axis_index,
                "before": before_value,
                "after": after_value,
                "match": match,
                "delta": None if before_value is None or after_value is None else int(after_value) - int(before_value),
            }
        )

    logical_joint_comparison: list[dict[str, Any]] = []
    logical_angle_mismatch = False
    for joint_index in range(joint_count):
        before_value = before_arm_rad[joint_index] if joint_index < len(before_arm_rad) else None
        after_value = after_arm_rad[joint_index] if joint_index < len(after_arm_rad) else None
        match = _float_close(before_value, after_value)
        logical_angle_mismatch = logical_angle_mismatch or not match
        logical_joint_comparison.append(
            {
                "joint": joint_index + 1,
                "before": before_value,
                "after": after_value,
                "match": match,
                "delta": None if before_value is None or after_value is None else float(after_value) - float(before_value),
            }
        )

    after_drive_faults = (
        after_snapshot.get("drive_faults") if isinstance(after_snapshot.get("drive_faults"), Mapping) else {}
    )
    startup_drive_config_mismatch_axes = []
    axes = after_drive_faults.get("axes")
    if isinstance(axes, list):
        for axis in axes:
            if not isinstance(axis, Mapping):
                continue
            startup_drive_config = axis.get("startup_drive_config")
            if isinstance(startup_drive_config, Mapping) and bool(startup_drive_config.get("configured")) and not bool(
                startup_drive_config.get("verified")
            ):
                startup_drive_config_mismatch_axes.append(
                    {
                        "axis": axis.get("axis"),
                        "logical_joint": axis.get("logical_joint"),
                        "setting_key": startup_drive_config.get("setting_key"),
                        "setting_label": startup_drive_config.get("setting_label"),
                        "commanded": startup_drive_config.get("commanded"),
                        "commanded_value_label": startup_drive_config.get("commanded_value_label"),
                        "readback": startup_drive_config.get("readback"),
                        "readback_value_label": startup_drive_config.get("readback_value_label"),
                        "readback_valid": bool(startup_drive_config.get("readback_valid")),
                    }
                )

    active_faults = _collect_fault_matches(after_drive_faults)
    return {
        "before_timestamp": before_snapshot.get("captured_at"),
        "after_timestamp": after_snapshot.get("captured_at"),
        "raw_encoder_mismatch": raw_encoder_mismatch,
        "logical_angle_mismatch": logical_angle_mismatch,
        "startup_drive_config_mismatch": bool(startup_drive_config_mismatch_axes),
        "axis_counts": axis_comparison,
        "logical_joints_rad": logical_joint_comparison,
        "active_battery_or_multiturn_faults": active_faults,
        "startup_drive_config_mismatch_axes": startup_drive_config_mismatch_axes,
    }


def _render_comparison_markdown(comparison: Mapping[str, Any]) -> str:
    def _status(flag: bool) -> str:
        return "MISMATCH" if flag else "OK"

    lines = [
        "# Encoder retention comparison",
        "",
        f"- Raw encoder counts: {_status(bool(comparison.get('raw_encoder_mismatch')))}",
        f"- Logical joint angles: {_status(bool(comparison.get('logical_angle_mismatch')))}",
        f"- Startup drive config: {_status(bool(comparison.get('startup_drive_config_mismatch')))}",
        f"- Battery/multi-turn faults after power-up: {len(list(comparison.get('active_battery_or_multiturn_faults', [])))}",
        "",
        "## Axis counts",
    ]
    for axis_entry in comparison.get("axis_counts", []):
        if not isinstance(axis_entry, Mapping):
            continue
        lines.append(
            f"- Axis {axis_entry.get('axis')}: before={axis_entry.get('before')} after={axis_entry.get('after')} match={axis_entry.get('match')}"
        )
    lines.append("")
    lines.append("## Logical joints (rad)")
    for joint_entry in comparison.get("logical_joints_rad", []):
        if not isinstance(joint_entry, Mapping):
            continue
        lines.append(
            f"- J{joint_entry.get('joint')}: before={joint_entry.get('before')} after={joint_entry.get('after')} match={joint_entry.get('match')}"
        )
    return "\n".join(lines) + "\n"


def capture_retention_snapshot(
    *,
    phase: str,
    snapshot_payload: Mapping[str, Any],
    experiment_id: str | None = None,
) -> dict[str, Any]:
    normalized_phase = normalize_retention_phase(phase)
    resolved_experiment_id = resolve_experiment_id(
        phase=normalized_phase,
        experiment_id=experiment_id,
    )
    experiment_dir = get_encoder_retention_log_dir() / resolved_experiment_id
    experiment_dir.mkdir(parents=True, exist_ok=True)

    captured_at = str(snapshot_payload.get("captured_at") or _utc_now().isoformat(timespec="seconds"))
    snapshot = {
        "experiment_id": resolved_experiment_id,
        "phase": normalized_phase,
        "captured_at": captured_at,
        **_json_clone(snapshot_payload),
    }
    snapshot_path = experiment_dir / f"{normalized_phase}.json"
    _write_json(snapshot_path, snapshot)

    result: dict[str, Any] = {
        "experiment_id": resolved_experiment_id,
        "phase": normalized_phase,
        "captured_at": captured_at,
        "snapshot_path": str(snapshot_path),
    }

    before_path = experiment_dir / f"{PHASE_BEFORE_POWER_DOWN}.json"
    after_path = experiment_dir / f"{PHASE_AFTER_POWER_UP}.json"
    if before_path.exists() and after_path.exists():
        with open(before_path, "r", encoding="utf-8") as handle:
            before_snapshot = json.load(handle)
        with open(after_path, "r", encoding="utf-8") as handle:
            after_snapshot = json.load(handle)
        comparison = compare_retention_snapshots(
            before_snapshot=before_snapshot,
            after_snapshot=after_snapshot,
        )
        comparison_path = experiment_dir / "comparison.json"
        _write_json(
            comparison_path,
            {
                "experiment_id": resolved_experiment_id,
                "before_snapshot_path": str(before_path),
                "after_snapshot_path": str(after_path),
                **comparison,
            },
        )
        comparison_md_path = experiment_dir / "comparison.md"
        comparison_md_path.write_text(_render_comparison_markdown(comparison), encoding="utf-8")
        result["comparison"] = {
            "comparison_path": str(comparison_path),
            "comparison_markdown_path": str(comparison_md_path),
            **comparison,
        }

    return result
