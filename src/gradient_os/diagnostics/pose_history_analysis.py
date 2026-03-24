from __future__ import annotations

import argparse
import datetime as dt
import json
import math
from pathlib import Path
from typing import Any


_LINEAR_AXES = ("x", "y", "z")
_ANGULAR_AXES = ("roll", "pitch", "yaw")
_PASSIVE_COMMAND_PREFIXES = ("GET_",)
_BOUNDARY_COMMANDS = {
    "JOG_SESSION_START",
    "JOG_SESSION_UPDATE",
    "JOG_SESSION_STOP",
    "SAFE_POWER_UP",
    "SAFE_POWER_DOWN",
    "STOP",
}
_EXTERNAL_MOTION_COMMAND_PREFIXES = (
    "MOVE_",
    "APPLY_",
    "EXECUTE_",
    "RUN_",
    "START_",
    "PLAN_",
)


def _safe_float(value: Any, default: float = 0.0) -> float:
    try:
        return float(value)
    except (TypeError, ValueError):
        return default


def _parse_timestamp(value: Any) -> dt.datetime | None:
    if not isinstance(value, str) or not value:
        return None
    text = value.replace("Z", "+00:00")
    try:
        return dt.datetime.fromisoformat(text)
    except ValueError:
        return None


def _seconds_between(start: Any, end: Any) -> float | None:
    start_dt = _parse_timestamp(start)
    end_dt = _parse_timestamp(end)
    if start_dt is None or end_dt is None:
        return None
    return round((end_dt - start_dt).total_seconds(), 6)


def _angle_diff_deg(current: Any, baseline: Any) -> float:
    delta = _safe_float(current) - _safe_float(baseline)
    while delta > 180.0:
        delta -= 360.0
    while delta < -180.0:
        delta += 360.0
    return delta


def _pose_delta(current: dict[str, Any] | None, baseline: dict[str, Any] | None) -> dict[str, Any] | None:
    if not isinstance(current, dict) or not isinstance(baseline, dict):
        return None
    current_pos = current.get("position_m")
    baseline_pos = baseline.get("position_m")
    current_orient = current.get("orientation_euler_deg")
    baseline_orient = baseline.get("orientation_euler_deg")
    if not isinstance(current_pos, dict) or not isinstance(baseline_pos, dict):
        return None
    if not isinstance(current_orient, dict) or not isinstance(baseline_orient, dict):
        return None
    position_mm = {
        axis: round((_safe_float(current_pos.get(axis)) - _safe_float(baseline_pos.get(axis))) * 1000.0, 6)
        for axis in _LINEAR_AXES
    }
    orientation_deg = {
        axis: round(_angle_diff_deg(current_orient.get(axis), baseline_orient.get(axis)), 6)
        for axis in _ANGULAR_AXES
    }
    position_norm_mm = round(
        math.sqrt(sum(position_mm[axis] * position_mm[axis] for axis in _LINEAR_AXES)),
        6,
    )
    orientation_norm_deg = round(
        math.sqrt(sum(orientation_deg[axis] * orientation_deg[axis] for axis in _ANGULAR_AXES)),
        6,
    )
    return {
        "position_mm": position_mm,
        "position_norm_mm": position_norm_mm,
        "orientation_deg": orientation_deg,
        "orientation_norm_deg": orientation_norm_deg,
    }


def _copy_pose(pose: Any) -> dict[str, Any] | None:
    if not isinstance(pose, dict):
        return None
    position_m = pose.get("position_m")
    orientation_euler_deg = pose.get("orientation_euler_deg")
    if not isinstance(position_m, dict) or not isinstance(orientation_euler_deg, dict):
        return None
    result: dict[str, Any] = {
        "position_m": {axis: _safe_float(position_m.get(axis)) for axis in _LINEAR_AXES},
        "orientation_euler_deg": {axis: _safe_float(orientation_euler_deg.get(axis)) for axis in _ANGULAR_AXES},
    }
    joints_deg = pose.get("joints_deg")
    if isinstance(joints_deg, list):
        result["joints_deg"] = [_safe_float(value) for value in joints_deg]
    return result


def _sample_measured_pose(sample: dict[str, Any]) -> dict[str, Any] | None:
    ik_debug = sample.get("ik_debug")
    if isinstance(ik_debug, dict):
        measured_pose = _copy_pose(ik_debug.get("measured_pose"))
        if measured_pose is not None:
            return measured_pose
    return _copy_pose(sample.get("pose"))


def _sample_commanded_pose(sample: dict[str, Any]) -> dict[str, Any] | None:
    ik_debug = sample.get("ik_debug")
    if not isinstance(ik_debug, dict):
        return None
    return _copy_pose(ik_debug.get("commanded_pose"))


def _round_vector(values: list[Any], digits: int) -> list[float]:
    return [round(_safe_float(value), digits) for value in values]


def _sample_signature(sample: dict[str, Any], *, digits: int = 3) -> dict[str, Any]:
    ik_debug = sample.get("ik_debug")
    if not isinstance(ik_debug, dict):
        linear = [0.0, 0.0, 0.0]
        angular = [0.0, 0.0, 0.0]
    else:
        linear = _round_vector(list(ik_debug.get("linear_velocity_m_s") or [0.0, 0.0, 0.0]), digits)
        angular = _round_vector(list(ik_debug.get("angular_velocity_deg_s") or [0.0, 0.0, 0.0]), digits)
    return {
        "linear_velocity_m_s": linear,
        "angular_velocity_deg_s": angular,
    }


def _classify_signature(signature: dict[str, Any]) -> dict[str, Any]:
    linear = [abs(_safe_float(value)) for value in signature.get("linear_velocity_m_s") or []]
    angular = [abs(_safe_float(value)) for value in signature.get("angular_velocity_deg_s") or []]
    linear_nonzero = [index for index, value in enumerate(linear) if value > 1e-6]
    angular_nonzero = [index for index, value in enumerate(angular) if value > 1e-6]
    linear_values = signature.get("linear_velocity_m_s") or [0.0, 0.0, 0.0]
    angular_values = signature.get("angular_velocity_deg_s") or [0.0, 0.0, 0.0]
    if len(linear_nonzero) == 1 and not angular_nonzero:
        idx = linear_nonzero[0]
        axis = _LINEAR_AXES[idx]
        value = _safe_float(linear_values[idx])
        direction = "+" if value >= 0.0 else "-"
        return {
            "kind": "linear",
            "axis": axis,
            "direction": direction,
            "label": f"{direction}{axis}",
            "magnitude": abs(value),
            "units": "m/s",
        }
    if len(angular_nonzero) == 1 and not linear_nonzero:
        idx = angular_nonzero[0]
        axis = _ANGULAR_AXES[idx]
        value = _safe_float(angular_values[idx])
        direction = "+" if value >= 0.0 else "-"
        return {
            "kind": "angular",
            "axis": axis,
            "direction": direction,
            "label": f"{direction}{axis}",
            "magnitude": abs(value),
            "units": "deg/s",
        }
    nonzero_count = len(linear_nonzero) + len(angular_nonzero)
    return {
        "kind": "blended" if nonzero_count > 1 else "stationary",
        "axis": None,
        "direction": None,
        "label": "blended" if nonzero_count > 1 else "stationary",
        "magnitude": None,
        "units": None,
    }


def _command_is_external_motion(command: Any) -> bool:
    if not isinstance(command, str) or not command:
        return False
    if command.startswith(_PASSIVE_COMMAND_PREFIXES):
        return False
    if command in _BOUNDARY_COMMANDS:
        return False
    return command.startswith(_EXTERNAL_MOTION_COMMAND_PREFIXES)


def _collect_segment_bounds(samples: list[dict[str, Any]]) -> list[tuple[int, int]]:
    bounds: list[tuple[int, int]] = []
    start_idx: int | None = None
    for index, sample in enumerate(samples):
        active = bool(sample.get("is_jogging"))
        if active and start_idx is None:
            start_idx = index
            continue
        if not active and start_idx is not None:
            bounds.append((start_idx, index - 1))
            start_idx = None
    if start_idx is not None:
        bounds.append((start_idx, len(samples) - 1))
    return bounds


def _max_following(active_samples: list[dict[str, Any]]) -> dict[str, Any]:
    max_position_mm = 0.0
    max_orientation_deg = 0.0
    max_joint_deg = 0.0
    for sample in active_samples:
        ik_debug = sample.get("ik_debug")
        if not isinstance(ik_debug, dict):
            continue
        following_error = ik_debug.get("following_error")
        if not isinstance(following_error, dict):
            continue
        pose = following_error.get("pose")
        joint = following_error.get("joint")
        if isinstance(pose, dict):
            max_position_mm = max(max_position_mm, _safe_float(pose.get("position_error_mm")))
            max_orientation_deg = max(max_orientation_deg, _safe_float(pose.get("orientation_error_deg")))
        if isinstance(joint, dict):
            max_joint_deg = max(max_joint_deg, _safe_float(joint.get("max_abs_joint_error_deg")))
    return {
        "position_error_mm": round(max_position_mm, 6),
        "orientation_error_deg": round(max_orientation_deg, 6),
        "max_abs_joint_error_deg": round(max_joint_deg, 6),
    }


def _segment_health(active_samples: list[dict[str, Any]]) -> dict[str, Any]:
    gate_results: dict[str, int] = {}
    gate_reasons: dict[str, int] = {}
    resync_reasons: set[str] = set()
    solve_failed_count = 0
    clamped_count = 0
    max_target_vs_applied_mm = 0.0
    max_target_vs_applied_deg = 0.0
    seq_values: list[int] = []
    dt_values: list[float] = []
    for sample in active_samples:
        ik_debug = sample.get("ik_debug")
        if not isinstance(ik_debug, dict):
            continue
        gate_result = str(ik_debug.get("gate_result") or "unknown")
        gate_reason = str(ik_debug.get("gate_reason") or "unknown")
        gate_results[gate_result] = gate_results.get(gate_result, 0) + 1
        gate_reasons[gate_reason] = gate_reasons.get(gate_reason, 0) + 1
        last_resync_reason = ik_debug.get("last_resync_reason")
        if isinstance(last_resync_reason, str) and last_resync_reason:
            resync_reasons.add(last_resync_reason)
        if bool(ik_debug.get("solve_failed")):
            solve_failed_count += 1
        if bool(ik_debug.get("clamped")):
            clamped_count += 1
        target_vs_applied = ik_debug.get("target_vs_applied")
        if isinstance(target_vs_applied, dict):
            max_target_vs_applied_mm = max(
                max_target_vs_applied_mm,
                _safe_float(target_vs_applied.get("position_error_mm")),
            )
            max_target_vs_applied_deg = max(
                max_target_vs_applied_deg,
                _safe_float(target_vs_applied.get("orientation_error_deg")),
            )
        seq = ik_debug.get("seq")
        if isinstance(seq, int):
            seq_values.append(seq)
        dt_s = ik_debug.get("dt_s")
        if dt_s is not None:
            dt_values.append(_safe_float(dt_s))
    return {
        "gate_results": gate_results,
        "gate_reasons": gate_reasons,
        "resync_reasons": sorted(resync_reasons),
        "solve_failed_count": solve_failed_count,
        "clamped_count": clamped_count,
        "max_target_vs_applied_mm": round(max_target_vs_applied_mm, 9),
        "max_target_vs_applied_deg": round(max_target_vs_applied_deg, 9),
        "seq_min": min(seq_values) if seq_values else None,
        "seq_max": max(seq_values) if seq_values else None,
        "mean_controller_dt_s": round(sum(dt_values) / len(dt_values), 9) if dt_values else None,
    }


def summarize_segment(
    samples: list[dict[str, Any]],
    start_idx: int,
    end_idx: int,
    next_start_idx: int | None,
    *,
    segment_index: int,
) -> dict[str, Any]:
    active_samples = samples[start_idx : end_idx + 1]
    pre_idx = start_idx - 1 if start_idx > 0 else None
    settle_end_idx = (next_start_idx - 1) if next_start_idx is not None else (len(samples) - 1)
    if settle_end_idx < end_idx:
        settle_end_idx = end_idx
    settle_samples = samples[end_idx + 1 : settle_end_idx + 1]
    first_active = active_samples[0]
    last_active = active_samples[-1]
    baseline_sample = samples[pre_idx] if pre_idx is not None else first_active
    baseline_pose = _sample_measured_pose(baseline_sample)
    active_end_measured_pose = _sample_measured_pose(last_active)
    active_end_commanded_pose = _sample_commanded_pose(last_active)
    settled_end_pose = _sample_measured_pose(samples[settle_end_idx]) if settle_end_idx >= 0 else None
    signature = _sample_signature(first_active)
    commands_between = sorted(
        {
            str(sample.get("controller_last_command"))
            for sample in settle_samples
            if isinstance(sample.get("controller_last_command"), str)
        }
    )
    external_motion_commands = sorted(command for command in commands_between if _command_is_external_motion(command))
    return {
        "segment_index": segment_index,
        "start_sample_index": start_idx,
        "end_sample_index": end_idx,
        "settle_end_sample_index": settle_end_idx,
        "sample_count": len(active_samples),
        "start_time": first_active.get("collected_at"),
        "end_time": last_active.get("collected_at"),
        "settled_end_time": samples[settle_end_idx].get("collected_at") if settle_end_idx >= 0 else None,
        "observed_active_window_s": _seconds_between(first_active.get("collected_at"), last_active.get("collected_at")),
        "observed_settle_window_s": (
            _seconds_between(last_active.get("collected_at"), samples[settle_end_idx].get("collected_at"))
            if settle_end_idx > end_idx
            else 0.0
        ),
        "signature": {
            **signature,
            "classification": _classify_signature(signature),
        },
        "baseline_pose": baseline_pose,
        "active_end_measured_pose": active_end_measured_pose,
        "active_end_commanded_pose": active_end_commanded_pose,
        "settled_end_pose": settled_end_pose,
        "active_end_measured_delta": _pose_delta(active_end_measured_pose, baseline_pose),
        "active_end_commanded_delta": _pose_delta(active_end_commanded_pose, baseline_pose),
        "settled_end_delta": _pose_delta(settled_end_pose, baseline_pose),
        "max_following_error": _max_following(active_samples),
        "health": _segment_health(active_samples),
        "settle_window_commands": commands_between,
        "settle_window_external_motion_commands": external_motion_commands,
    }


def _segments_from_samples(samples: list[dict[str, Any]]) -> list[dict[str, Any]]:
    bounds = _collect_segment_bounds(samples)
    segments: list[dict[str, Any]] = []
    for index, (start_idx, end_idx) in enumerate(bounds, start=1):
        next_start_idx = bounds[index][0] if index < len(bounds) else None
        segments.append(
            summarize_segment(
                samples,
                start_idx,
                end_idx,
                next_start_idx,
                segment_index=index,
            )
        )
    return segments


def _can_pair_round_trip(first: dict[str, Any], second: dict[str, Any]) -> bool:
    first_class = ((first.get("signature") or {}).get("classification") or {})
    second_class = ((second.get("signature") or {}).get("classification") or {})
    if first_class.get("kind") not in {"linear", "angular"}:
        return False
    if first_class.get("kind") != second_class.get("kind"):
        return False
    if first_class.get("axis") != second_class.get("axis"):
        return False
    if first_class.get("direction") == second_class.get("direction"):
        return False
    first_mag = _safe_float(first_class.get("magnitude"))
    second_mag = _safe_float(second_class.get("magnitude"))
    if abs(first_mag - second_mag) > max(1e-3, 0.05 * max(first_mag, second_mag, 1.0)):
        return False
    if (first.get("settle_window_external_motion_commands") or []):
        return False
    return True


def _summarize_round_trip(first: dict[str, Any], second: dict[str, Any], *, pair_index: int) -> dict[str, Any]:
    baseline_pose = first.get("baseline_pose")
    first_peak_pose = first.get("active_end_measured_pose")
    second_peak_pose = second.get("active_end_measured_pose")
    final_pose = second.get("settled_end_pose")
    first_class = ((first.get("signature") or {}).get("classification") or {})
    return {
        "pair_index": pair_index,
        "segment_indices": [first.get("segment_index"), second.get("segment_index")],
        "label": f"{first_class.get('label')} -> {((second.get('signature') or {}).get('classification') or {}).get('label')}",
        "kind": first_class.get("kind"),
        "axis": first_class.get("axis"),
        "first_leg_peak_delta": _pose_delta(first_peak_pose, baseline_pose),
        "second_leg_peak_delta": _pose_delta(second_peak_pose, baseline_pose),
        "final_residual_delta": _pose_delta(final_pose, baseline_pose),
        "max_following_error": {
            "position_error_mm": round(
                max(
                    _safe_float(((first.get("max_following_error") or {}).get("position_error_mm"))),
                    _safe_float(((second.get("max_following_error") or {}).get("position_error_mm"))),
                ),
                6,
            ),
            "orientation_error_deg": round(
                max(
                    _safe_float(((first.get("max_following_error") or {}).get("orientation_error_deg"))),
                    _safe_float(((second.get("max_following_error") or {}).get("orientation_error_deg"))),
                ),
                6,
            ),
            "max_abs_joint_error_deg": round(
                max(
                    _safe_float(((first.get("max_following_error") or {}).get("max_abs_joint_error_deg"))),
                    _safe_float(((second.get("max_following_error") or {}).get("max_abs_joint_error_deg"))),
                ),
                6,
            ),
        },
        "intervening_commands": list(first.get("settle_window_commands") or []),
    }


def _pair_round_trips(segments: list[dict[str, Any]]) -> list[dict[str, Any]]:
    pairs: list[dict[str, Any]] = []
    pair_index = 1
    cursor = 0
    while cursor < len(segments) - 1:
        first = segments[cursor]
        second = segments[cursor + 1]
        if _can_pair_round_trip(first, second):
            pairs.append(_summarize_round_trip(first, second, pair_index=pair_index))
            pair_index += 1
            cursor += 2
            continue
        cursor += 1
    return pairs


def analyze_pose_history_document(document: dict[str, Any], *, source_path: str = "") -> dict[str, Any]:
    samples = document.get("pose_history")
    if not isinstance(samples, list):
        raise ValueError("Pose history document must include a 'pose_history' list.")
    cleaned_samples = [sample for sample in samples if isinstance(sample, dict)]
    segments = _segments_from_samples(cleaned_samples)
    round_trips = _pair_round_trips(segments)
    max_position_error_mm = max(
        (_safe_float(((segment.get("max_following_error") or {}).get("position_error_mm"))) for segment in segments),
        default=0.0,
    )
    max_orientation_error_deg = max(
        (_safe_float(((segment.get("max_following_error") or {}).get("orientation_error_deg"))) for segment in segments),
        default=0.0,
    )
    max_joint_error_deg = max(
        (_safe_float(((segment.get("max_following_error") or {}).get("max_abs_joint_error_deg"))) for segment in segments),
        default=0.0,
    )
    return {
        "path": source_path,
        "source": document.get("source"),
        "exported_at": document.get("exported_at"),
        "saved_at": document.get("saved_at"),
        "sample_count": len(cleaned_samples),
        "segment_count": len(segments),
        "round_trip_count": len(round_trips),
        "segments": segments,
        "round_trips": round_trips,
        "aggregate": {
            "max_following_error": {
                "position_error_mm": round(max_position_error_mm, 6),
                "orientation_error_deg": round(max_orientation_error_deg, 6),
                "max_abs_joint_error_deg": round(max_joint_error_deg, 6),
            },
            "segments_with_external_gap_motion": sum(
                1 for segment in segments if segment.get("settle_window_external_motion_commands")
            ),
        },
    }


def analyze_pose_history_path(path: str | Path) -> dict[str, Any]:
    resolved = Path(path).resolve()
    document = json.loads(resolved.read_text(encoding="utf-8"))
    if not isinstance(document, dict):
        raise ValueError(f"Expected JSON object in {resolved}")
    return analyze_pose_history_document(document, source_path=str(resolved))


def expand_pose_history_inputs(paths: list[str]) -> list[Path]:
    resolved: list[Path] = []
    for raw in paths:
        path = Path(raw).expanduser().resolve()
        if path.is_dir():
            resolved.extend(sorted(path.glob("*-pose-history.json")))
        elif path.is_file():
            resolved.append(path)
        else:
            matches = sorted(Path.cwd().glob(raw))
            resolved.extend(match.resolve() for match in matches if match.is_file())
    unique_paths: list[Path] = []
    seen: set[str] = set()
    for path in resolved:
        key = str(path)
        if key in seen:
            continue
        seen.add(key)
        unique_paths.append(path)
    return unique_paths


def render_text_report(report: dict[str, Any]) -> str:
    lines: list[str] = []
    path = report.get("path") or "<memory>"
    lines.append(str(path))
    lines.append(
        "  "
        f"segments={report.get('segment_count', 0)} "
        f"round_trips={report.get('round_trip_count', 0)} "
        f"samples={report.get('sample_count', 0)} "
        f"source={report.get('source') or 'unknown'}"
    )
    aggregate = report.get("aggregate") or {}
    max_follow = aggregate.get("max_following_error") or {}
    lines.append(
        "  "
        f"max_following: pos={_safe_float(max_follow.get('position_error_mm')):.3f} mm "
        f"orient={_safe_float(max_follow.get('orientation_error_deg')):.3f} deg "
        f"joint={_safe_float(max_follow.get('max_abs_joint_error_deg')):.3f} deg"
    )
    for segment in report.get("segments") or []:
        signature = segment.get("signature") or {}
        classification = signature.get("classification") or {}
        measured_delta = segment.get("active_end_measured_delta") or {}
        settled_delta = segment.get("settled_end_delta") or {}
        max_following = segment.get("max_following_error") or {}
        lines.append(
            "  "
            f"[seg {segment.get('segment_index')}] {classification.get('label')} "
            f"kind={classification.get('kind')} samples={segment.get('sample_count')} "
            f"lin={signature.get('linear_velocity_m_s')} ang={signature.get('angular_velocity_deg_s')} "
            f"start={segment.get('start_time')} end={segment.get('end_time')}"
        )
        if measured_delta:
            pos = measured_delta.get("position_mm") or {}
            orient = measured_delta.get("orientation_deg") or {}
            lines.append(
                "    "
                f"active_end: "
                f"x={_safe_float(pos.get('x')):+.3f} y={_safe_float(pos.get('y')):+.3f} z={_safe_float(pos.get('z')):+.3f} mm | "
                f"roll={_safe_float(orient.get('roll')):+.3f} pitch={_safe_float(orient.get('pitch')):+.3f} yaw={_safe_float(orient.get('yaw')):+.3f} deg"
            )
        commanded_delta = segment.get("active_end_commanded_delta") or {}
        if commanded_delta:
            pos = commanded_delta.get("position_mm") or {}
            orient = commanded_delta.get("orientation_deg") or {}
            lines.append(
                "    "
                f"commanded_end: "
                f"x={_safe_float(pos.get('x')):+.3f} y={_safe_float(pos.get('y')):+.3f} z={_safe_float(pos.get('z')):+.3f} mm | "
                f"roll={_safe_float(orient.get('roll')):+.3f} pitch={_safe_float(orient.get('pitch')):+.3f} yaw={_safe_float(orient.get('yaw')):+.3f} deg"
            )
        if settled_delta:
            pos = settled_delta.get("position_mm") or {}
            orient = settled_delta.get("orientation_deg") or {}
            lines.append(
                "    "
                f"settled_end: "
                f"x={_safe_float(pos.get('x')):+.3f} y={_safe_float(pos.get('y')):+.3f} z={_safe_float(pos.get('z')):+.3f} mm | "
                f"roll={_safe_float(orient.get('roll')):+.3f} pitch={_safe_float(orient.get('pitch')):+.3f} yaw={_safe_float(orient.get('yaw')):+.3f} deg"
            )
        lines.append(
            "    "
            f"max_following: pos={_safe_float(max_following.get('position_error_mm')):.3f} mm "
            f"orient={_safe_float(max_following.get('orientation_error_deg')):.3f} deg "
            f"joint={_safe_float(max_following.get('max_abs_joint_error_deg')):.3f} deg"
        )
        external_gap_commands = segment.get("settle_window_external_motion_commands") or []
        if external_gap_commands:
            lines.append("    " f"gap_motion_commands={', '.join(external_gap_commands)}")
    if report.get("round_trips"):
        lines.append("  round_trips:")
    for pair in report.get("round_trips") or []:
        residual = pair.get("final_residual_delta") or {}
        pos = residual.get("position_mm") or {}
        orient = residual.get("orientation_deg") or {}
        lines.append(
            "    "
            f"[pair {pair.get('pair_index')}] {pair.get('label')} "
            f"residual: x={_safe_float(pos.get('x')):+.3f} y={_safe_float(pos.get('y')):+.3f} z={_safe_float(pos.get('z')):+.3f} mm | "
            f"roll={_safe_float(orient.get('roll')):+.3f} pitch={_safe_float(orient.get('pitch')):+.3f} yaw={_safe_float(orient.get('yaw')):+.3f} deg"
        )
    return "\n".join(lines)


def build_cli_report(paths: list[str]) -> dict[str, Any]:
    pose_history_paths = expand_pose_history_inputs(paths)
    if not pose_history_paths:
        raise FileNotFoundError("No pose-history files matched the provided inputs.")
    reports = [analyze_pose_history_path(path) for path in pose_history_paths]
    return {
        "generated_at": dt.datetime.now(dt.timezone.utc).isoformat(),
        "file_count": len(reports),
        "reports": reports,
        "aggregate": {
            "segment_count": sum(int(report.get("segment_count") or 0) for report in reports),
            "round_trip_count": sum(int(report.get("round_trip_count") or 0) for report in reports),
            "max_following_error": {
                "position_error_mm": round(
                    max(
                        (
                            _safe_float(
                                (((report.get("aggregate") or {}).get("max_following_error") or {}).get("position_error_mm"))
                            )
                            for report in reports
                        ),
                        default=0.0,
                    ),
                    6,
                ),
                "orientation_error_deg": round(
                    max(
                        (
                            _safe_float(
                                (((report.get("aggregate") or {}).get("max_following_error") or {}).get("orientation_error_deg"))
                            )
                            for report in reports
                        ),
                        default=0.0,
                    ),
                    6,
                ),
                "max_abs_joint_error_deg": round(
                    max(
                        (
                            _safe_float(
                                (((report.get("aggregate") or {}).get("max_following_error") or {}).get("max_abs_joint_error_deg"))
                            )
                            for report in reports
                        ),
                        default=0.0,
                    ),
                    6,
                ),
            },
        },
    }


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description="Split saved pose-history logs into individual jog segments and round-trip summaries."
    )
    parser.add_argument(
        "paths",
        nargs="+",
        help="Pose-history files, directories, or glob patterns to analyze.",
    )
    parser.add_argument(
        "--json",
        action="store_true",
        help="Print the full structured report as JSON instead of the text summary.",
    )
    parser.add_argument(
        "--output",
        default="",
        help="Optional path to save the structured JSON report.",
    )
    args = parser.parse_args(argv)

    payload = build_cli_report(args.paths)
    serialized = json.dumps(payload, indent=2, sort_keys=True)
    if args.output:
        output_path = Path(args.output).expanduser().resolve()
        output_path.parent.mkdir(parents=True, exist_ok=True)
        output_path.write_text(serialized + "\n", encoding="utf-8")
    if args.json:
        print(serialized)
    else:
        print(
            "\n\n".join(render_text_report(report) for report in payload.get("reports") or [])
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
