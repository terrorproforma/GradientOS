from __future__ import annotations

# NOTE: Do not re-enable `planned["useCache"]` for normal saved trajectories.
# The copied preview cache stores planned steps from the robot's live start
# state at preview time, so reusing it later from a different pose can cause
# erratic motion. Materialize the saved trajectory file, but force fresh
# planning on execution unless the flow is explicitly weld/cache-specific.

import asyncio
import base64
import binascii
import datetime
import json
import logging
import os
import shutil
import socket
import threading
import time
from contextlib import closing, asynccontextmanager
from typing import Any, Dict, Tuple

import argparse
import subprocess
import sys

from fastapi import Body, FastAPI, HTTPException
from fastapi.concurrency import run_in_threadpool
from fastapi.middleware.cors import CORSMiddleware
from sse_starlette.sse import EventSourceResponse
import numpy as np

from ..cad.topology_service import (
    CADTopologyService,
    TopologyDependencyError,
    TopologyModelNotFoundError,
)
from ..kinematics import runtime as kinematics_runtime
from .. import robot_assets
from .. import runtime_config
from .. import tool_library
from ..diagnostics.runtime_snapshot import get_runtime_diagnostics_snapshot
from ..arm_controller.robots import get_robot_name_by_id, list_robot_metadata
from ..joint_zero_offsets import load_joint_zero_offsets_store
from ..telemetry.drive_faults import build_drive_fault_snapshot
from ..telemetry.encoder_retention import capture_retention_snapshot

try:
    from ..arm_controller import utils as controller_utils
    from ..arm_controller import command_api as controller_command_api
except ImportError:
    controller_utils = None
    controller_command_api = None

_REST_POSE_RAD = [0.0, -1.4, 1.5, 0.0, 0.0, 0.0]
_REST_POSE_COMMAND = ",".join(str(value) for value in _REST_POSE_RAD)
_ALLOWED_WELD_TYPES = {"fillet", "butt", "lap", "tack/spot", "custom"}
_ALLOWED_PROGRAM_KINDS = {"trajectory", "weld"}
_PROJECT_ROOT_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", ".."))
_ROBOT_PROGRAM_DIR = os.path.join(_PROJECT_ROOT_DIR, "recorded_programs")
_RECORDED_TRAJECTORY_DIR = os.path.join(_PROJECT_ROOT_DIR, "recorded_trajectories")
_TRAJECTORY_CACHE_DIR = os.path.join(_PROJECT_ROOT_DIR, "src", "trajectory_cache")
_WELD_PROGRAM_DIR = os.path.join(_PROJECT_ROOT_DIR, "recorded_trajectories", "weld_programs")
_DIAGNOSTICS_LOG_DIR = os.path.join(_PROJECT_ROOT_DIR, "logs", "diagnostics")
_CONTROLLER_REPLY_MAX_BYTES = 65535
_DEFAULT_MONITOR_TELEMETRY_HZ = 50
_DEFAULT_MOVE_LINE_CLOSED_LOOP = False
_SAFE_COMMISSIONING_MAX_MOTOR_RPM = 100.0
_DEFAULT_RTCORE_METRICS_PATH = "/run/gradient-rt-motion/metrics.json"
_API_PERF_LOCK = threading.Lock()
_API_PERF: dict[str, Any] = {
    "last_command": None,
    "last_result": None,
    "last_round_trip_ms": None,
    "by_command": {},
}
_DEFAULT_JOG_SESSION_LEASE_TIMEOUT_S = 0.4


def _command_label(message: str) -> str:
    return message.split(",", 1)[0].strip().upper()


def _get_last_planner_diagnostics_snapshot() -> dict[str, Any] | None:
    state = getattr(controller_utils, "trajectory_state", None)
    if not isinstance(state, dict):
        return None
    diagnostics = state.get("last_planner_diagnostics")
    if not isinstance(diagnostics, dict):
        return None
    try:
        return json.loads(json.dumps(diagnostics))
    except Exception:
        return dict(diagnostics)


def _planner_failure_detail(message: str) -> str | dict[str, Any]:
    diagnostics = _get_last_planner_diagnostics_snapshot()
    if diagnostics is None:
        return message
    return {
        "message": message,
        "planner_diagnostics": diagnostics,
    }


def _record_api_command_metric(message: str, *, ok: bool, round_trip_ms: float, timed_out: bool) -> None:
    command = _command_label(message)
    with _API_PERF_LOCK:
        _API_PERF["last_command"] = command
        _API_PERF["last_result"] = "ok" if ok else ("timeout" if timed_out else "error")
        _API_PERF["last_round_trip_ms"] = float(round_trip_ms)
        by_command = _API_PERF.setdefault("by_command", {})
        entry = by_command.setdefault(
            command,
            {
                "count": 0,
                "ok_count": 0,
                "error_count": 0,
                "timeout_count": 0,
                "avg_round_trip_ms": 0.0,
                "max_round_trip_ms": 0.0,
                "last_round_trip_ms": 0.0,
            },
        )
        count = int(entry.get("count", 0)) + 1
        avg_ms = float(entry.get("avg_round_trip_ms", 0.0))
        entry["count"] = count
        entry["last_round_trip_ms"] = float(round_trip_ms)
        entry["avg_round_trip_ms"] = avg_ms + ((float(round_trip_ms) - avg_ms) / float(count))
        entry["max_round_trip_ms"] = max(float(entry.get("max_round_trip_ms", 0.0)), float(round_trip_ms))
        if ok:
            entry["ok_count"] = int(entry.get("ok_count", 0)) + 1
        elif timed_out:
            entry["timeout_count"] = int(entry.get("timeout_count", 0)) + 1
        else:
            entry["error_count"] = int(entry.get("error_count", 0)) + 1


def _get_api_command_metrics_snapshot() -> dict[str, Any]:
    with _API_PERF_LOCK:
        return json.loads(json.dumps(_API_PERF))


def _load_rtcore_metrics_summary() -> dict[str, Any] | None:
    metrics_path = os.environ.get("GRADIENT_RTCORE_METRICS", _DEFAULT_RTCORE_METRICS_PATH)
    try:
        with open(metrics_path, "r", encoding="utf-8") as fh:
            raw = json.load(fh)
    except FileNotFoundError:
        return None
    except Exception as exc:
        return {"error": str(exc), "metrics_path": metrics_path}

    if not isinstance(raw, dict):
        return {"error": "metrics payload was not a JSON object", "metrics_path": metrics_path}

    return {
        "metrics_path": metrics_path,
        "rt_frequency_hz": raw.get("rt_frequency_hz"),
        "rt_last_jitter_ns": raw.get("rt_last_jitter_ns"),
        "rt_max_abs_jitter_ns": raw.get("rt_max_abs_jitter_ns"),
        "rt_overrun_count": raw.get("rt_overrun_count"),
        "wkc_actual": raw.get("wkc_actual"),
        "wkc_expected": raw.get("wkc_expected"),
        "motion_active_command_seq": raw.get("motion_active_command_seq"),
        "motion_last_update_age_ms": raw.get("motion_last_update_age_ms"),
        "feedback_cycle_jitter_ns": raw.get("feedback_cycle_jitter_ns"),
    }


def _load_rtcore_metrics_raw() -> dict[str, Any] | None:
    metrics_path = os.environ.get("GRADIENT_RTCORE_METRICS", _DEFAULT_RTCORE_METRICS_PATH)
    try:
        with open(metrics_path, "r", encoding="utf-8") as fh:
            raw = json.load(fh)
    except FileNotFoundError:
        return None
    except Exception as exc:
        return {"error": str(exc), "metrics_path": metrics_path}
    if not isinstance(raw, dict):
        return {"error": "metrics payload was not a JSON object", "metrics_path": metrics_path}
    raw.setdefault("metrics_path", metrics_path)
    return raw


def _pose_history_filename(exported_at: str | None = None) -> str:
    if isinstance(exported_at, str) and exported_at.strip():
        raw = exported_at.strip()
        try:
            parsed = datetime.datetime.fromisoformat(raw.replace("Z", "+00:00"))
        except ValueError:
            parsed = None
        if parsed is not None:
            return f"{parsed.astimezone(datetime.timezone.utc).strftime('%Y%m%d-%H%M%S')}-pose-history.json"
    return f"{datetime.datetime.now(datetime.timezone.utc).strftime('%Y%m%d-%H%M%S')}-pose-history.json"


def _default_controller_port() -> int:
    if controller_utils is not None:
        port = getattr(controller_utils, "UDP_PORT", None)
        if port is not None:
            return int(port)
    return 3000


def _resolve_controller_endpoint() -> Tuple[str, int]:
    host = os.environ.get("GRADIENT_CONTROLLER_HOST", "127.0.0.1")
    port = int(os.environ.get("GRADIENT_CONTROLLER_PORT", _default_controller_port()))
    return host, port


def _read_int_env(name: str, default: int, *, minimum: int | None = None) -> int:
    raw = os.environ.get(name, str(default)).strip()
    try:
        value = int(raw)
    except ValueError:
        value = default
    if minimum is not None:
        value = max(minimum, value)
    return value


def _probe_controller(timeout: float = 0.5) -> Tuple[bool, str]:
    host, port = _resolve_controller_endpoint()
    payload = b"GET_STATUS"
    with closing(socket.socket(socket.AF_INET, socket.SOCK_DGRAM)) as sock:
        sock.settimeout(max(0.05, timeout))
        try:
            sock.sendto(payload, (host, port))
            data, _addr = sock.recvfrom(_CONTROLLER_REPLY_MAX_BYTES)
        except socket.timeout:
            return False, f"Timed out waiting for controller response at {host}:{port}"
        except OSError as exc:
            return (
                False,
                f"Socket error connecting to controller at {host}:{port}: {exc}",
            )

    text = data.decode("utf-8", errors="ignore")
    if text.startswith("STATUS"):
        return True, f"Controller reachable at {host}:{port}"
    return False, f"Unexpected controller response '{text}' from {host}:{port}"


def _send_controller_command(
    message: str, timeout: float = 0.5, expect_response: bool = True
) -> Tuple[bool, str]:
    host, port = _resolve_controller_endpoint()
    started = time.perf_counter()
    with closing(socket.socket(socket.AF_INET, socket.SOCK_DGRAM)) as sock:
        sock.settimeout(max(0.05, timeout))
        try:
            sock.sendto(message.encode("utf-8"), (host, port))
        except OSError as exc:
            _record_api_command_metric(
                message,
                ok=False,
                round_trip_ms=max(0.0, (time.perf_counter() - started) * 1000.0),
                timed_out=False,
            )
            return False, f"Socket error sending '{message}': {exc}"
        else:
            if not expect_response:
                _record_api_command_metric(
                    message,
                    ok=True,
                    round_trip_ms=max(0.0, (time.perf_counter() - started) * 1000.0),
                    timed_out=False,
                )
                return True, ""
        try:
            data, _addr = sock.recvfrom(_CONTROLLER_REPLY_MAX_BYTES)
        except socket.timeout:
            _record_api_command_metric(
                message,
                ok=False,
                round_trip_ms=max(0.0, (time.perf_counter() - started) * 1000.0),
                timed_out=True,
            )
            return False, f"No response for command '{message}'"
        except OSError as exc:
            _record_api_command_metric(
                message,
                ok=False,
                round_trip_ms=max(0.0, (time.perf_counter() - started) * 1000.0),
                timed_out=False,
            )
            return False, f"Socket receive error for '{message}': {exc}"
    text = data.decode("utf-8", errors="ignore")
    _record_api_command_metric(
        message,
        ok=not text.startswith("ERROR"),
        round_trip_ms=max(0.0, (time.perf_counter() - started) * 1000.0),
        timed_out=False,
    )
    if text.startswith("ERROR"):
        return False, text
    return True, text


def _resolve_cors_origins() -> list[str]:
    raw = os.environ.get("GRADIENT_API_CORS", "")
    if not raw:
        return ["*"]
    origins = [item.strip() for item in raw.split(",")]
    return [origin for origin in origins if origin]


def _normalize_weld_type(raw: Any) -> str:
    value = str(raw or "").strip().lower()
    if not value:
        return "fillet"
    if value == "spot":
        value = "tack/spot"
    if value not in _ALLOWED_WELD_TYPES:
        allowed = ", ".join(sorted(_ALLOWED_WELD_TYPES))
        raise HTTPException(status_code=400, detail=f"Invalid weld_type '{value}'. Allowed: {allowed}")
    return value


def _decode_base64_step_payload(payload: dict[str, Any]) -> tuple[str, bytes]:
    filename_raw = payload.get("filename")
    filename = str(filename_raw).strip() if isinstance(filename_raw, str) else ""
    if not filename:
        filename = "uploaded.step"

    encoded = payload.get("step_base64")
    if not isinstance(encoded, str) or not encoded.strip():
        encoded = payload.get("step_data_base64")
    if not isinstance(encoded, str) or not encoded.strip():
        raise HTTPException(status_code=400, detail="Field 'step_base64' is required.")

    try:
        raw_bytes = base64.b64decode(encoded, validate=True)
    except (binascii.Error, ValueError) as exc:
        raise HTTPException(status_code=400, detail=f"Invalid base64 STEP payload: {exc}") from exc
    if not raw_bytes:
        raise HTTPException(status_code=400, detail="Decoded STEP payload is empty.")
    return filename, raw_bytes


def _ensure_weld_program_dir() -> None:
    os.makedirs(_WELD_PROGRAM_DIR, exist_ok=True)


def _sanitize_weld_program_name(raw: Any) -> str:
    return _sanitize_program_name(raw)


def _coerce_step_transform(raw: Any) -> dict[str, Any]:
    default = {
        "position": {"x": 0.0, "y": 0.0, "z": 0.0},
        "rotationDeg": {"x": 0.0, "y": 0.0, "z": 0.0},
        "scale": 1.0,
    }
    if not isinstance(raw, dict):
        return default

    def _axis_triplet(value: Any) -> dict[str, float]:
        if not isinstance(value, dict):
            return {"x": 0.0, "y": 0.0, "z": 0.0}
        out = {}
        for axis in ("x", "y", "z"):
            try:
                out[axis] = float(value.get(axis, 0.0))
            except Exception:
                out[axis] = 0.0
        return out

    try:
        scale = max(1e-4, float(raw.get("scale", 1.0)))
    except Exception:
        scale = 1.0

    return {
        "position": _axis_triplet(raw.get("position")),
        "rotationDeg": _axis_triplet(raw.get("rotationDeg")),
        "scale": scale,
    }


def _ensure_robot_program_dir() -> None:
    os.makedirs(_ROBOT_PROGRAM_DIR, exist_ok=True)


def _sanitize_program_name(raw: Any) -> str:
    if not isinstance(raw, str):
        raise HTTPException(status_code=400, detail="Field 'name' must be a string.")
    trimmed = raw.strip()
    if not trimmed:
        raise HTTPException(status_code=400, detail="Field 'name' is required.")
    safe = "".join(ch if (ch.isalnum() or ch in {"-", "_"}) else "_" for ch in trimmed)
    safe = safe.strip("_")
    if not safe:
        raise HTTPException(status_code=400, detail="Field 'name' must contain letters or numbers.")
    return safe[:128]


def _normalize_program_kind(raw: Any) -> str:
    value = str(raw or "").strip().lower()
    if value not in _ALLOWED_PROGRAM_KINDS:
        allowed = ", ".join(sorted(_ALLOWED_PROGRAM_KINDS))
        raise HTTPException(status_code=400, detail=f"Invalid program kind '{value}'. Allowed: {allowed}")
    return value


def _robot_program_path(name: str) -> str:
    return os.path.join(_ROBOT_PROGRAM_DIR, f"{name}.json")


def _normalize_robot_program_record(raw: dict[str, Any]) -> dict[str, Any]:
    kind = _normalize_program_kind(raw.get("kind"))
    authoring = raw.get("authoring")
    if not isinstance(authoring, dict):
        raise HTTPException(status_code=500, detail="Saved program record is missing 'authoring'.")
    record: dict[str, Any] = {
        "name": _sanitize_program_name(raw.get("name")),
        "kind": kind,
        "saved_at": (
            str(raw.get("saved_at")).strip()
            if isinstance(raw.get("saved_at"), str) and str(raw.get("saved_at")).strip()
            else datetime.datetime.utcnow().isoformat(timespec="seconds") + "Z"
        ),
        "authoring": json.loads(json.dumps(authoring)),
        "planned_trajectory": (
            json.loads(json.dumps(raw.get("planned_trajectory")))
            if isinstance(raw.get("planned_trajectory"), dict)
            else None
        ),
    }
    metadata = raw.get("metadata")
    if isinstance(metadata, dict):
        record["metadata"] = json.loads(json.dumps(metadata))
    return record


def _normalize_legacy_weld_program_record(raw: dict[str, Any], fallback_name: str) -> dict[str, Any]:
    safe_name = _sanitize_program_name(raw.get("name", fallback_name))
    step_payload = raw.get("step")
    if not isinstance(step_payload, dict):
        raise HTTPException(status_code=500, detail="Legacy weld program is missing STEP payload.")
    record: dict[str, Any] = {
        "name": safe_name,
        "kind": "weld",
        "saved_at": (
            str(raw.get("saved_at")).strip()
            if isinstance(raw.get("saved_at"), str) and str(raw.get("saved_at")).strip()
            else datetime.datetime.utcnow().isoformat(timespec="seconds") + "Z"
        ),
        "authoring": {
            "step": {
                "filename": str(step_payload.get("filename", "uploaded.step")).strip() or "uploaded.step",
                "step_base64": str(step_payload.get("step_base64", "")).strip(),
                "transform": _coerce_step_transform(step_payload.get("transform")),
            },
            "weld_draft": json.loads(json.dumps(raw.get("weld_draft", {}))),
            "editable_waypoints": [
                {"x": x, "y": y, "z": z}
                for (x, y, z) in _coerce_waypoint_list(raw.get("editable_waypoints"))
            ],
        },
        "planned_trajectory": (
            json.loads(json.dumps(raw.get("planned_trajectory")))
            if isinstance(raw.get("planned_trajectory"), dict)
            else None
        ),
    }
    return record


def _load_robot_program_record(name: str, *, expected_kind: str | None = None) -> dict[str, Any]:
    safe_name = _sanitize_program_name(name)
    if expected_kind is not None:
        expected_kind = _normalize_program_kind(expected_kind)

    path = _robot_program_path(safe_name)
    if os.path.exists(path):
        try:
            with open(path, "r", encoding="utf-8") as f:
                record = _normalize_robot_program_record(json.load(f))
        except HTTPException:
            raise
        except Exception as exc:
            raise HTTPException(status_code=500, detail=f"Failed to load saved program: {exc}") from exc
        if expected_kind and record["kind"] != expected_kind:
            raise HTTPException(
                status_code=404,
                detail=f"Saved program '{safe_name}' is of kind '{record['kind']}', not '{expected_kind}'.",
            )
        return _annotate_and_materialize_saved_trajectory_plan(record)

    if expected_kind in {None, "weld"}:
        legacy_path = os.path.join(_WELD_PROGRAM_DIR, f"{safe_name}.json")
        if os.path.exists(legacy_path):
            try:
                with open(legacy_path, "r", encoding="utf-8") as f:
                    return _normalize_legacy_weld_program_record(json.load(f), safe_name)
            except HTTPException:
                raise
            except Exception as exc:
                raise HTTPException(status_code=500, detail=f"Failed to load weld program: {exc}") from exc

    raise HTTPException(status_code=404, detail=f"Saved program '{safe_name}' not found.")


def _save_robot_program_record(record: dict[str, Any]) -> dict[str, Any]:
    normalized = _normalize_robot_program_record(record)
    normalized = _annotate_and_materialize_saved_trajectory_plan(normalized)
    _ensure_robot_program_dir()
    path = _robot_program_path(normalized["name"])
    with open(path, "w", encoding="utf-8") as f:
        json.dump(normalized, f, indent=2)
    return normalized


def _pose_waypoints_match(
    authoring_waypoints: list[dict[str, Any]],
    planned_waypoints: list[dict[str, Any]],
    *,
    pos_tol: float = 1e-3,
    orient_tol_deg: float = 0.5,
) -> bool:
    def _wrapped_angle_delta_deg(lhs: float, rhs: float) -> float:
        return abs(((lhs - rhs + 180.0) % 360.0) - 180.0)

    def _speed_matches(lhs: dict[str, Any], rhs: dict[str, Any], key: str, tol: float = 1e-3) -> bool:
        left_value = lhs.get(key)
        right_value = rhs.get(key)
        if left_value is None and right_value is None:
            return True
        if left_value is None or right_value is None:
            return False
        return abs(float(left_value) - float(right_value)) <= tol

    if len(authoring_waypoints) != len(planned_waypoints):
        return False
    for authored, planned in zip(authoring_waypoints, planned_waypoints):
        authored_move_type = str(authored.get("move_type", authored.get("moveType", "linear"))).strip().lower()
        planned_move_type = str(planned.get("move_type", planned.get("moveType", "linear"))).strip().lower()
        if authored_move_type != planned_move_type:
            return False
        if not _speed_matches(authored, planned, "linear_speed_mm_s"):
            return False
        if not _speed_matches(authored, planned, "linear_acceleration_mm_s2"):
            return False
        if not _speed_matches(authored, planned, "rotation_speed_deg_s"):
            return False
        if not _speed_matches(authored, planned, "pause_after_s"):
            return False
        for axis in ("x", "y", "z"):
            if abs(float(authored[axis]) - float(planned[axis])) > pos_tol:
                return False
        authored_orient = authored.get("orientation_euler_deg")
        planned_orient = planned.get("orientation_euler_deg")
        if isinstance(authored_orient, dict) and isinstance(planned_orient, dict):
            for axis in ("roll", "pitch", "yaw"):
                authored_value = authored_orient.get(axis)
                planned_value = planned_orient.get(axis)
                if authored_value is None or planned_value is None:
                    return False
                if _wrapped_angle_delta_deg(float(authored_value), float(planned_value)) > orient_tol_deg:
                    return False
        elif authored_orient is None and planned_orient is None:
            continue
        else:
            return False
    return True


def _annotate_and_materialize_saved_trajectory_plan(record: dict[str, Any]) -> dict[str, Any]:
    if record.get("kind") != "trajectory":
        return record

    planned = record.get("planned_trajectory")
    if not isinstance(planned, dict):
        return record

    safe_name = _sanitize_program_name(record.get("name"))
    authored_waypoints = _coerce_pose_waypoint_list(
        (record.get("authoring") or {}).get("waypoints") if isinstance(record.get("authoring"), dict) else None
    )
    planned_waypoints = _coerce_pose_waypoint_list(planned.get("waypoints"))
    is_compatible = bool(authored_waypoints) and bool(planned_waypoints) and _pose_waypoints_match(
        authored_waypoints,
        planned_waypoints,
    )

    source_plan_name = str(planned.get("name", "")).strip()
    planned["name"] = safe_name
    planned["useCache"] = False
    planned["isStale"] = not is_compatible
    if source_plan_name:
        planned["sourcePlanName"] = source_plan_name
    elif "sourcePlanName" in planned:
        planned.pop("sourcePlanName", None)

    trajectory_payload = planned.get("trajectory")
    if not is_compatible or not isinstance(trajectory_payload, dict):
        record["planned_trajectory"] = planned
        return record

    os.makedirs(_RECORDED_TRAJECTORY_DIR, exist_ok=True)
    recorded_path = os.path.join(_RECORDED_TRAJECTORY_DIR, f"{safe_name}.json")
    with open(recorded_path, "w", encoding="utf-8") as f:
        json.dump(trajectory_payload, f, indent=2)

    # Standard saved trajectory previews are start-state dependent: the planned
    # steps cache begins at whatever live joint state was present when preview
    # planning ran. Reusing that cache later from a different robot pose can
    # jump straight into stale joint samples and produce erratic motion. Keep
    # the saved trajectory payload/recorded file, but force fresh planning on
    # each run instead of advertising cache reuse.
    planned["useCache"] = False
    planned["isStale"] = False
    record["planned_trajectory"] = planned
    return record


def _list_robot_program_names(kind: str | None = None) -> list[str]:
    expected_kind = _normalize_program_kind(kind) if kind is not None else None
    names: list[str] = []
    seen: set[str] = set()

    _ensure_robot_program_dir()
    for filename in os.listdir(_ROBOT_PROGRAM_DIR):
        if not filename.lower().endswith(".json"):
            continue
        name = filename[:-5]
        path = os.path.join(_ROBOT_PROGRAM_DIR, filename)
        try:
            with open(path, "r", encoding="utf-8") as f:
                record = _normalize_robot_program_record(json.load(f))
        except Exception:
            continue
        if expected_kind and record["kind"] != expected_kind:
            continue
        if name not in seen:
            names.append(name)
            seen.add(name)

    if expected_kind in {None, "weld"} and os.path.isdir(_WELD_PROGRAM_DIR):
        for filename in os.listdir(_WELD_PROGRAM_DIR):
            if not filename.lower().endswith(".json"):
                continue
            name = filename[:-5]
            if name in seen:
                continue
            names.append(name)
            seen.add(name)

    names.sort()
    return names


def _serialize_weld_program_record(record: dict[str, Any]) -> dict[str, Any]:
    authoring = record.get("authoring")
    if not isinstance(authoring, dict):
        raise HTTPException(status_code=500, detail="Saved weld program is missing authoring payload.")
    step_payload = authoring.get("step")
    if not isinstance(step_payload, dict):
        raise HTTPException(status_code=500, detail="Saved weld program is missing STEP payload.")
    payload: dict[str, Any] = {
        "name": record["name"],
        "saved_at": record.get("saved_at"),
        "step": json.loads(json.dumps(step_payload)),
        "weld_draft": json.loads(json.dumps(authoring.get("weld_draft", {}))),
        "editable_waypoints": json.loads(json.dumps(authoring.get("editable_waypoints", []))),
        "planned_trajectory": json.loads(json.dumps(record.get("planned_trajectory"))),
    }
    return payload


def _build_weld_program_record_from_payload(payload: dict[str, Any]) -> dict[str, Any]:
    safe_name = _sanitize_program_name(payload.get("name"))
    authoring_payload = payload.get("authoring")
    if isinstance(authoring_payload, dict):
        step_payload = authoring_payload.get("step")
        weld_draft_raw = authoring_payload.get("weld_draft")
        editable_waypoints_raw = authoring_payload.get("editable_waypoints")
    else:
        step_payload = payload.get("step")
        weld_draft_raw = payload.get("weld_draft")
        editable_waypoints_raw = payload.get("editable_waypoints")

    if not isinstance(step_payload, dict):
        raise HTTPException(status_code=400, detail="Field 'step' is required and must be an object.")
    step_filename, step_bytes = _decode_base64_step_payload(step_payload)
    step_base64 = base64.b64encode(step_bytes).decode("ascii")
    step_transform = _coerce_step_transform(step_payload.get("transform"))

    if not isinstance(weld_draft_raw, dict):
        raise HTTPException(status_code=400, detail="Field 'weld_draft' is required and must be an object.")

    default_weld_type = _normalize_weld_type(
        weld_draft_raw.get("weldType", weld_draft_raw.get("weld_type", "fillet"))
    )
    segments: list[dict[str, Any]] = []
    raw_segments = weld_draft_raw.get("segments")
    if isinstance(raw_segments, list):
        seen_edge_ids: set[str] = set()
        for raw_segment in raw_segments:
            if not isinstance(raw_segment, dict):
                continue
            edge_id = str(raw_segment.get("edgeId", raw_segment.get("edge_id", ""))).strip()
            if not edge_id or edge_id in seen_edge_ids:
                continue
            try:
                start_s = float(raw_segment.get("startS", raw_segment.get("start_s", 0.0)))
            except (TypeError, ValueError):
                start_s = 0.0
            try:
                end_s = float(raw_segment.get("endS", raw_segment.get("end_s", 1.0)))
            except (TypeError, ValueError):
                end_s = 1.0
            start_s = max(0.0, min(1.0, start_s))
            end_s = max(0.0, min(1.0, end_s))
            weld_type = _normalize_weld_type(
                raw_segment.get("weldType", raw_segment.get("weld_type", default_weld_type))
            )
            segments.append(
                {
                    "edgeId": edge_id,
                    "startS": start_s,
                    "endS": end_s,
                    "weldType": weld_type,
                }
            )
            seen_edge_ids.add(edge_id)

    legacy_edge_id = str(weld_draft_raw.get("edgeId", weld_draft_raw.get("edge_id", ""))).strip()
    try:
        legacy_start_s = float(weld_draft_raw.get("startS", weld_draft_raw.get("start_s", 0.0)))
    except (TypeError, ValueError):
        legacy_start_s = 0.0
    try:
        legacy_end_s = float(weld_draft_raw.get("endS", weld_draft_raw.get("end_s", 1.0)))
    except (TypeError, ValueError):
        legacy_end_s = 1.0
    legacy_start_s = max(0.0, min(1.0, legacy_start_s))
    legacy_end_s = max(0.0, min(1.0, legacy_end_s))
    if not segments and legacy_edge_id:
        segments.append(
            {
                "edgeId": legacy_edge_id,
                "startS": legacy_start_s,
                "endS": legacy_end_s,
                "weldType": default_weld_type,
            }
        )

    requested_active_edge_id = str(
        weld_draft_raw.get(
            "activeSegmentEdgeId",
            weld_draft_raw.get("active_segment_edge_id", legacy_edge_id),
        )
    ).strip()
    active_segment_edge_id = (
        requested_active_edge_id
        if requested_active_edge_id and any(seg["edgeId"] == requested_active_edge_id for seg in segments)
        else (segments[0]["edgeId"] if segments else "")
    )
    active_segment = next(
        (seg for seg in segments if seg["edgeId"] == active_segment_edge_id),
        segments[0] if segments else None,
    )
    active_weld_type = (
        _normalize_weld_type(active_segment.get("weldType", default_weld_type))
        if isinstance(active_segment, dict)
        else default_weld_type
    )
    post_action_raw = str(
        weld_draft_raw.get("postAction", weld_draft_raw.get("post_action", "return_to_start"))
    ).strip()
    post_action = (
        "none" if post_action_raw == "none" else ("lift" if post_action_raw == "lift" else "return_to_start")
    )

    weld_draft = {
        "modelId": str(weld_draft_raw.get("modelId", weld_draft_raw.get("model_id", ""))).strip(),
        "edgeId": active_segment["edgeId"] if active_segment else legacy_edge_id,
        "weldType": active_weld_type,
        "weldName": (
            str(weld_draft_raw.get("weldName", weld_draft_raw.get("weld_name", f"{active_weld_type} weld"))).strip()
            or f"{active_weld_type} weld"
        ),
        "workAngleDeg": float(weld_draft_raw.get("workAngleDeg", weld_draft_raw.get("work_angle_deg", 45.0))),
        "travelAngleDeg": float(weld_draft_raw.get("travelAngleDeg", weld_draft_raw.get("travel_angle_deg", 0.0))),
        "spinAngleDeg": float(
            weld_draft_raw.get("spinAngleDeg", weld_draft_raw.get("spin_angle_deg", 0.0))
        ),
        "transitionClearanceMm": float(
            weld_draft_raw.get("transitionClearanceMm", weld_draft_raw.get("transition_clearance_mm", 35.0))
        ),
        "postAction": post_action,
        "startS": active_segment["startS"] if active_segment else legacy_start_s,
        "endS": active_segment["endS"] if active_segment else legacy_end_s,
        "segments": segments,
        "activeSegmentEdgeId": active_segment_edge_id or None,
    }

    editable_waypoints = [
        {"x": x, "y": y, "z": z}
        for (x, y, z) in _coerce_waypoint_list(editable_waypoints_raw)
    ]
    planned_trajectory = payload.get("planned_trajectory")
    if planned_trajectory is not None and not isinstance(planned_trajectory, dict):
        raise HTTPException(status_code=400, detail="planned_trajectory must be an object or null.")

    return {
        "name": safe_name,
        "kind": "weld",
        "saved_at": datetime.datetime.utcnow().isoformat(timespec="seconds") + "Z",
        "authoring": {
            "step": {
                "filename": step_filename,
                "step_base64": step_base64,
                "transform": step_transform,
            },
            "weld_draft": weld_draft,
            "editable_waypoints": editable_waypoints,
        },
        "planned_trajectory": planned_trajectory if isinstance(planned_trajectory, dict) else None,
    }


def _build_trajectory_program_record_from_payload(payload: dict[str, Any]) -> dict[str, Any]:
    safe_name = _sanitize_program_name(payload.get("name"))
    authoring_payload = payload.get("authoring")
    waypoints_raw = (
        authoring_payload.get("waypoints")
        if isinstance(authoring_payload, dict)
        else payload.get("waypoints", payload.get("pose_waypoints"))
    )
    metadata_raw = authoring_payload.get("metadata") if isinstance(authoring_payload, dict) else payload.get("metadata")
    waypoints = _coerce_pose_waypoint_list(waypoints_raw)
    if len(waypoints) == 0:
        raise HTTPException(status_code=400, detail="Trajectory programs require at least one waypoint.")
    planned_trajectory = payload.get("planned_trajectory")
    if planned_trajectory is not None and not isinstance(planned_trajectory, dict):
        raise HTTPException(status_code=400, detail="planned_trajectory must be an object or null.")
    metadata = metadata_raw if isinstance(metadata_raw, dict) else {}
    return {
        "name": safe_name,
        "kind": "trajectory",
        "saved_at": datetime.datetime.utcnow().isoformat(timespec="seconds") + "Z",
        "authoring": {
            "waypoints": waypoints,
            "metadata": json.loads(json.dumps(metadata)),
        },
        "planned_trajectory": planned_trajectory if isinstance(planned_trajectory, dict) else None,
    }


class _TelemetryProtocol(asyncio.DatagramProtocol):
    def __init__(self, hub: "TelemetryHub") -> None:
        self.hub = hub

    def datagram_received(self, data: bytes, addr) -> None:  # type: ignore[override]
        self.hub.handle_datagram(data, addr)


class TelemetryHub:
    def __init__(self) -> None:
        self._subscribers: Dict[int, asyncio.Queue[str]] = {}
        self._counter = 0
        self._lock = asyncio.Lock()
        self._transport: asyncio.DatagramTransport | None = None
        self._aux_transport: asyncio.DatagramTransport | None = None
        self._servo_proc: subprocess.Popen | None = None
        self._advertise_host = os.environ.get("GRADIENT_MONITOR_HOST")
        self._bind_host = os.environ.get("GRADIENT_MONITOR_BIND", "127.0.0.1")
        self._telemetry_hz = _read_int_env(
            "GRADIENT_MONITOR_TELEMETRY_HZ",
            _DEFAULT_MONITOR_TELEMETRY_HZ,
            minimum=1,
        )
        self._listen_port: int | None = None
        # Optional fixed UDP port to ingest auxiliary telemetry (e.g., servo_telemetry_stream.py)
        # Set GRADIENT_AUX_TELEMETRY_PORT=0 to disable. Default 5556.
        try:
            self._aux_listen_port: int | None = int(os.environ.get("GRADIENT_AUX_TELEMETRY_PORT", "5556"))
        except ValueError:
            self._aux_listen_port = 5556
        # Autostart the servo telemetry streamer (default DISABLED; set to 1/true to enable)
        _auto_env = os.environ.get("GRADIENT_AUTOSTART_SERVO_TELEMETRY", "0").strip().lower()
        self._autostart_servo_telemetry: bool = _auto_env in {"1", "true", "yes", "on"}

    async def register(self) -> Tuple[int, asyncio.Queue[str]]:
        queue: asyncio.Queue[str] = asyncio.Queue(maxsize=50)
        async with self._lock:
            first_client = not self._subscribers
            if first_client:
                await self._start()
            self._counter += 1
            token = self._counter
            self._subscribers[token] = queue
        return token, queue

    async def unregister(self, token: int) -> None:
        async with self._lock:
            self._subscribers.pop(token, None)
            if not self._subscribers:
                await self._stop()

    async def _start(self) -> None:
        loop = asyncio.get_running_loop()
        transport, _protocol = await loop.create_datagram_endpoint(
            lambda: _TelemetryProtocol(self),
            local_addr=(self._bind_host, 0),
        )
        self._transport = transport
        sockname = transport.get_extra_info("sockname")
        assert sockname is not None
        listen_host = self._advertise_host or _resolve_controller_endpoint()[0]
        self._listen_port = sockname[1]
        start_cmd = f"START_TELEMETRY,{listen_host}:{self._listen_port},{self._telemetry_hz}"
        ok, detail = await run_in_threadpool(_send_controller_command, start_cmd)
        if not ok:
            await self._cleanup_transport()
            raise HTTPException(status_code=503, detail=detail)
        # Optionally also open a fixed auxiliary UDP port to ingest extra telemetry sources.
        if self._aux_listen_port and self._aux_listen_port > 0:
            try:
                aux_transport, _aux_proto = await loop.create_datagram_endpoint(
                    lambda: _TelemetryProtocol(self),
                    local_addr=(self._bind_host, self._aux_listen_port),
                )
                self._aux_transport = aux_transport
            except Exception:
                # If aux port binding fails, continue without it.
                self._aux_transport = None
        # Autostart the servo telemetry streamer so charts work by default
        if self._autostart_servo_telemetry and self._aux_listen_port and self._aux_listen_port > 0:
            try:
                cmd = [
                    sys.executable,
                    "-m",
                    "gradient_os.telemetry.servo_telemetry_stream",
                    "--fps",
                    "10",
                    "--udp",
                    f"127.0.0.1:{self._aux_listen_port}",
                ]
                self._servo_proc = subprocess.Popen(
                    cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, close_fds=True
                )
            except Exception:
                self._servo_proc = None

    async def _stop(self) -> None:
        if self._listen_port is None:
            return
        stop_cmd = "STOP_TELEMETRY"
        await run_in_threadpool(_send_controller_command, stop_cmd)
        await self._cleanup_transport()

    async def _cleanup_transport(self) -> None:
        if self._transport is not None:
            self._transport.close()
        self._transport = None
        if self._aux_transport is not None:
            try:
                self._aux_transport.close()
            except Exception:
                pass
        self._aux_transport = None
        # Stop autostarted servo telemetry if running
        if self._servo_proc is not None:
            try:
                self._servo_proc.terminate()
            except Exception:
                pass
            self._servo_proc = None
        self._listen_port = None

    def handle_datagram(self, data: bytes, addr) -> None:
        try:
            text = data.decode("utf-8")
        except UnicodeDecodeError:
            return
        event_payload = self._format_event(text)
        for queue in list(self._subscribers.values()):
            try:
                queue.put_nowait(event_payload)
            except asyncio.QueueFull:
                try:
                    _ = queue.get_nowait()
                except asyncio.QueueEmpty:
                    pass
                try:
                    queue.put_nowait(event_payload)
                except asyncio.QueueFull:
                    # If still full, skip this subscriber to avoid blocking.
                    continue

    def _format_event(self, text: str) -> str:
        try:
            parsed = json.loads(text)
        except json.JSONDecodeError:
            return text
        return json.dumps(parsed)


telemetry_hub = TelemetryHub()
topology_service = CADTopologyService()
logger = logging.getLogger("uvicorn.error")
_latest_plan_lock = asyncio.Lock()
_latest_plan: dict[str, Any] | None = None


def create_app() -> FastAPI:
    def _resolve_rest_pose() -> list[float]:
        raw = os.environ.get("GRADIENT_REST_POSE", "").strip()
        if raw:
            try:
                vals = [float(tok) for tok in raw.split(",") if tok.strip() != ""]
                # Expect 6 values; if not, still return what we have rather than crash
                return vals
            except Exception:
                pass
        # Fallback to the desktop UI default (radians)
        return [0.0, -1.4, 1.5, 0.0, 0.0, 0.0]
    @asynccontextmanager
    async def lifespan(app: FastAPI):
        ok, detail = await run_in_threadpool(_probe_controller)
        host, port = _resolve_controller_endpoint()
        if ok:
            logger.info("Controller: %s:%s", host, port)
        else:
            logger.warning("Controller: %s:%s (%s)", host, port, detail)
        yield

    api = FastAPI(title="GradientOS API", version="0.1.0", lifespan=lifespan)
    origins = _resolve_cors_origins()
    allow_credentials = "*" not in origins
    api.add_middleware(
        CORSMiddleware,
        allow_origins=origins,
        allow_methods=["*"],
        allow_headers=["*"],
        allow_credentials=allow_credentials,
    )

    @api.get("/health", summary="Controller health probe")
    async def health():
        ok, detail = await run_in_threadpool(_probe_controller)
        if not ok:
            raise HTTPException(status_code=503, detail=detail)
        host, port = _resolve_controller_endpoint()
        return {
            "status": "ok",
            "detail": detail,
            "controller": {"host": host, "port": port},
        }

    def _controller_call_or_503(
        command: str, *, timeout: float = 0.5, expect_response: bool = True
    ) -> str:
        ok, detail = _send_controller_command(
            command, timeout=timeout, expect_response=expect_response
        )
        if not ok:
            raise HTTPException(status_code=503, detail=detail)
        return detail

    def _encode_payload_b64(payload: dict[str, Any]) -> str:
        body = json.dumps(payload, separators=(",", ":"), ensure_ascii=True).encode("utf-8")
        return base64.urlsafe_b64encode(body).decode("ascii")

    def _decode_payload_b64(token: str) -> dict[str, Any]:
        try:
            raw = base64.urlsafe_b64decode(token.encode("ascii"))
            payload = json.loads(raw.decode("utf-8"))
        except Exception as exc:
            raise HTTPException(status_code=502, detail=f"Malformed controller payload: {exc}") from exc
        if not isinstance(payload, dict):
            raise HTTPException(status_code=502, detail="Controller payload must decode to a JSON object.")
        return payload

    def _parse_controller_ack_payload(
        detail: str,
        command: str,
        *,
        allow_plain_suffix: bool = False,
    ) -> dict[str, Any]:
        bare_prefix = f"ACK,{command}"
        if detail == bare_prefix:
            return {}
        prefix = bare_prefix + ","
        if not detail.startswith(prefix):
            raise HTTPException(status_code=502, detail=f"Malformed {command} reply: {detail}")
        token = detail[len(prefix) :].strip()
        if not token:
            return {}
        try:
            return _decode_payload_b64(token)
        except HTTPException:
            if allow_plain_suffix:
                return {"detail_token": token}
            raise

    def _parse_controller_error_payload(detail: str, command: str) -> dict[str, Any] | None:
        prefix = f"ERROR,{command},"
        if not detail.startswith(prefix):
            return None
        token = detail[len(prefix) :].strip()
        if not token:
            return {"code": "CONTROLLER_ERROR", "message": detail}
        try:
            return _decode_payload_b64(token)
        except HTTPException:
            return {"code": "CONTROLLER_ERROR", "message": detail}

    def _status_code_for_jog_session_error(code: str) -> int:
        normalized = str(code or "").strip().upper()
        if normalized.startswith("INVALID_"):
            return 400
        if normalized in {"OWNER_CONFLICT", "SESSION_ALREADY_ACTIVE", "STALE_SEQUENCE", "WRONG_SESSION", "MOTION_ACTIVE"}:
            return 409
        if normalized in {"SESSION_EXPIRED"}:
            return 410
        if normalized in {"SESSION_NOT_FOUND", "SESSION_INACTIVE"}:
            return 404
        return 503

    def _status_code_for_power_transition_error(code: str) -> int:
        normalized = str(code or "").strip().upper()
        if normalized in {"POWER_UP_BLOCKED", "RESET_FAULTS_BLOCKED"}:
            return 409
        return 503

    def _controller_structured_call(command: str, response_command: str, *, timeout: float) -> tuple[str, dict[str, Any]]:
        ok, detail = _send_controller_command(command, timeout=timeout, expect_response=True)
        if ok:
            return detail, _parse_controller_ack_payload(detail, response_command, allow_plain_suffix=True)
        error_payload = _parse_controller_error_payload(detail, response_command)
        if error_payload is not None:
            raise HTTPException(
                status_code=_status_code_for_power_transition_error(str(error_payload.get("code", ""))),
                detail=error_payload,
            )
        raise HTTPException(status_code=503, detail=detail)

    def _controller_jog_session_call(command: str, payload: dict[str, Any] | None = None) -> dict[str, Any]:
        message = command
        if payload is not None:
            message += f",{_encode_payload_b64(payload)}"
        ok, detail = _send_controller_command(message, timeout=1.0, expect_response=True)
        if not ok:
            error_payload = _parse_controller_error_payload(detail, command)
            if error_payload is not None:
                raise HTTPException(
                    status_code=_status_code_for_jog_session_error(str(error_payload.get("code", ""))),
                    detail=error_payload,
                )
            raise HTTPException(status_code=503, detail=detail)
        return _parse_controller_ack_payload(detail, command)

    def _jog_session_owner_id(payload: dict[str, Any]) -> str:
        owner_id = str(payload.get("owner_id", "") or "").strip()
        if not owner_id:
            raise HTTPException(status_code=400, detail="owner_id is required")
        return owner_id

    def _parse_motion_status_reply(detail: str) -> dict[str, Any]:
        prefix = "MOTION_STATUS,"
        if not detail.startswith(prefix):
            raise HTTPException(status_code=502, detail=f"Malformed motion-status reply: {detail}")
        token = detail[len(prefix) :].strip()
        if not token:
            raise HTTPException(status_code=502, detail="Motion-status reply did not include a payload.")
        return _decode_payload_b64(token)

    def _motion_status_matches_program(payload: dict[str, Any], expected_name: str) -> bool:
        expected = str(expected_name or "").strip()
        if not expected:
            return False
        program = payload.get("program") if isinstance(payload.get("program"), dict) else None
        program_name = str(
            (program.get("name") if program is not None else payload.get("program_name")) or ""
        ).strip()
        if program_name != expected:
            return False
        program_active = bool(
            program.get("active") if program is not None else payload.get("program_active", False)
        )
        program_state = str(
            (program.get("state") if program is not None else payload.get("program_state")) or ""
        ).strip().lower()
        return program_active or program_state in {"accepted", "planning", "executing"}

    async def _infer_run_trajectory_timeout_acceptance(
        *,
        trajectory_name: str,
        runtime_mode: str,
        execution_mode: str,
        timeout_detail: str,
    ) -> dict[str, Any] | None:
        for attempt_index in range(3):
            try:
                status_detail = await run_in_threadpool(
                    _controller_call_or_503, "GET_MOTION_STATUS", timeout=1.0, expect_response=True
                )
                status_payload = _parse_motion_status_reply(status_detail)
            except HTTPException:
                status_payload = None
                status_detail = ""
            if status_payload and _motion_status_matches_program(status_payload, trajectory_name):
                return {
                    "status": "ok",
                    "detail": status_detail,
                    "execution_mode": execution_mode,
                    "runtime_mode": runtime_mode,
                    "accepted": True,
                    "ack_inferred": True,
                    "run_request_timed_out": True,
                    "run_request_detail": timeout_detail,
                    **status_payload,
                }
            if attempt_index < 2:
                await asyncio.sleep(0.25)
        return None

    def _parse_kinematics_error(detail: str) -> HTTPException:
        if detail.startswith("ERROR,KINEMATICS,"):
            parts = detail.split(",", 3)
            code = parts[2] if len(parts) > 2 else "UNKNOWN"
            message = parts[3] if len(parts) > 3 else detail
            status_map = {
                "STALE_REVISION": 409,
                "MOTION_ACTIVE": 409,
                "APPLY_REQUIRES_RESTART": 409,
                "INVALID_PAYLOAD": 400,
                "PROFILE_INVALID": 400,
                "INTERNAL": 500,
            }
            return HTTPException(
                status_code=status_map.get(code, 400),
                detail={"code": code, "message": message},
            )
        return HTTPException(status_code=503, detail=detail)

    def _parse_apply_joint_setpoint_error(detail: str) -> HTTPException:
        prefix = "ERROR,APPLY_JOINT_SETPOINT,"
        if detail.startswith(prefix):
            message = detail[len(prefix) :].strip() or "Controller rejected direct joint setpoint."
            lowered = message.lower()
            if any(token in lowered for token in ("missing", "must be", "expected", "invalid")):
                status_code = 400
            elif any(
                token in lowered
                for token in (
                    "not connected",
                    "setpoint slot",
                    "cmd ring",
                    "overflow",
                    "did not report a valid num_axes",
                )
            ):
                status_code = 503
            else:
                status_code = 409
            return HTTPException(
                status_code=status_code,
                detail={
                    "code": "APPLY_JOINT_SETPOINT_REJECTED",
                    "message": message,
                },
            )
        return HTTPException(status_code=503, detail=detail)

    def _controller_kinematics_call(command: str, timeout: float = 2.0) -> dict[str, Any]:
        ok, detail = _send_controller_command(command, timeout=timeout, expect_response=True)
        if not ok:
            raise _parse_kinematics_error(detail)

        if detail.startswith("KINEMATICS_PROFILE,"):
            payload_json = detail[len("KINEMATICS_PROFILE,") :]
        elif detail.startswith("KINEMATICS_OK,"):
            payload_json = detail[len("KINEMATICS_OK,") :]
        else:
            raise HTTPException(status_code=502, detail=f"Malformed kinematics reply: {detail}")

        try:
            payload = json.loads(payload_json)
        except json.JSONDecodeError as exc:
            raise HTTPException(status_code=502, detail=f"Kinematics payload decode failure: {exc}") from exc
        if not isinstance(payload, dict):
            raise HTTPException(status_code=502, detail="Kinematics payload must be a JSON object.")
        return payload

    def _controller_runtime_config_call(timeout: float = 2.0) -> dict[str, Any]:
        ok, detail = _send_controller_command(
            "GET_RUNTIME_CONFIG",
            timeout=timeout,
            expect_response=True,
        )
        if not ok:
            raise HTTPException(status_code=503, detail=detail)
        if detail.startswith("ERROR,RUNTIME_CONFIG,"):
            message = detail.split(",", 2)[-1]
            raise HTTPException(status_code=503, detail=message)
        if not detail.startswith("RUNTIME_CONFIG,"):
            raise HTTPException(status_code=502, detail=f"Malformed runtime-config reply: {detail}")
        payload_json = detail[len("RUNTIME_CONFIG,") :]
        try:
            payload = json.loads(payload_json)
        except json.JSONDecodeError as exc:
            raise HTTPException(status_code=502, detail=f"Runtime-config decode failure: {exc}") from exc
        if not isinstance(payload, dict):
            raise HTTPException(status_code=502, detail="Runtime-config payload must be a JSON object.")
        return payload

    def _runtime_active_snapshot(runtime_snapshot: dict[str, Any]) -> dict[str, Any]:
        active = runtime_snapshot.get("active")
        if isinstance(active, dict):
            return active
        return runtime_snapshot if isinstance(runtime_snapshot, dict) else {}

    def _runtime_is_sim_mode(runtime_snapshot: dict[str, Any]) -> bool:
        active = _runtime_active_snapshot(runtime_snapshot)
        if not active:
            return False
        mode = active.get("mode")
        if isinstance(mode, dict) and bool(mode.get("sim")):
            return True
        servo_backend = active.get("servo_backend")
        if isinstance(servo_backend, dict):
            backend_name = str(servo_backend.get("effective_backend", "")).strip().lower()
            if backend_name == "simulation":
                return True
        return False

    def _controller_set_active_tool_call(tool_id: str | None, timeout: float = 2.0) -> str:
        tool_token = str(tool_id).strip() if tool_id is not None else ""
        ok, detail = _send_controller_command(
            f"SET_ACTIVE_TOOL,{tool_token}",
            timeout=timeout,
            expect_response=True,
        )
        if not ok:
            raise HTTPException(status_code=503, detail=detail)
        if detail.startswith("ERROR,SET_ACTIVE_TOOL,"):
            message = detail.split(",", 2)[-1]
            raise HTTPException(status_code=503, detail=message)
        if not detail.startswith("ACK,SET_ACTIVE_TOOL,"):
            raise HTTPException(status_code=502, detail=f"Malformed set-active-tool reply: {detail}")
        return detail

    def _sync_local_planner_runtime(timeout: float = 1.0) -> dict[str, Any]:
        """
        Keep API-process planner modules aligned with the active controller runtime.

        Weld/preview planning currently executes in-process through arm_controller
        modules, so we must mirror the controller's active robot + IK backend.
        """
        runtime_payload = _controller_runtime_config_call(timeout=timeout)
        robot_block = runtime_payload.get("robot")
        ik_block = runtime_payload.get("ik_solver")
        tool_block = runtime_payload.get("tool")
        if not isinstance(robot_block, dict) or not isinstance(ik_block, dict):
            raise RuntimeError("Controller runtime config payload missing robot/ik_solver blocks.")

        robot_name = str(robot_block.get("name", "")).strip()
        ik_backend = str(ik_block.get("effective_backend", "")).strip().lower()
        if not robot_name:
            raise RuntimeError("Controller runtime config did not include robot.name.")
        if not ik_backend:
            raise RuntimeError("Controller runtime config did not include ik_solver.effective_backend.")

        from ..arm_controller import robot_config as controller_robot_config
        from ..arm_controller.robots import get_robot_config
        from .. import ik_solver as api_ik_solver

        robot_cfg = get_robot_config(robot_name)
        controller_robot_config.set_active_robot(robot_cfg)
        api_ik_solver.configure(robot_id=robot_cfg.robot_id, backend_name=ik_backend)
        if isinstance(tool_block, dict):
            try:
                kinematics_runtime.set_active_tool_definition(
                    tool_block,
                    expected_revision=None,
                    motion_state="IDLE",
                    reset_runtime_trim=True,
                )
            except Exception:
                pass
        return {
            "robot_name": robot_name,
            "robot_id": robot_cfg.robot_id,
            "ik_backend": ik_backend,
            "active_tool_id": (
                str(tool_block.get("active_tool_id"))
                if isinstance(tool_block, dict) and tool_block.get("active_tool_id")
                else None
            ),
        }

    def _parse_bool_token(token: str) -> bool:
        return token.strip().lower() in {"1", "true", "yes", "on"}

    def _coerce_request_bool(name: str, raw: Any, *, default: bool | None = None) -> bool:
        if raw is None:
            if default is None:
                raise HTTPException(status_code=400, detail=f"{name} is required")
            return bool(default)
        if isinstance(raw, bool):
            return raw
        if isinstance(raw, (int, float)):
            if float(raw) in {0.0, 1.0}:
                return bool(raw)
            raise HTTPException(status_code=400, detail=f"{name} must be a boolean")
        token = str(raw).strip().lower()
        if token in {"1", "true", "yes", "on"}:
            return True
        if token in {"0", "false", "no", "off"}:
            return False
        raise HTTPException(status_code=400, detail=f"{name} must be a boolean")

    def _coerce_request_int(name: str, raw: Any, *, default: int | None = None) -> int:
        if raw is None:
            if default is None:
                raise HTTPException(status_code=400, detail=f"{name} is required")
            return int(default)
        if isinstance(raw, bool):
            raise HTTPException(status_code=400, detail=f"{name} must be an integer")
        try:
            value = int(raw)
        except Exception as exc:
            raise HTTPException(status_code=400, detail=f"{name} must be an integer") from exc
        return value

    @api.post("/control/stop", summary="Emergency stop")
    async def control_stop():
        detail = await run_in_threadpool(
            _controller_call_or_503, "STOP", timeout=1.0, expect_response=True
        )
        payload = _parse_controller_ack_payload(detail, "STOP")
        return {"status": "ok", "detail": detail, **payload}

    @api.post("/control/power-down", summary="Best-effort actuator power-down / de-energize")
    async def control_power_down(payload: dict[str, Any] | None = None):
        wait_for_idle = True
        if isinstance(payload, dict):
            wait_for_idle = bool(payload.get("wait_for_idle", True))
        command = "SAFE_POWER_DOWN,wait" if wait_for_idle else "SAFE_POWER_DOWN"
        detail, structured = await run_in_threadpool(
            _controller_structured_call, command, "SAFE_POWER_DOWN", timeout=5.0
        )
        return {
            "status": "ok",
            "detail": detail,
            "waited_for_idle": bool(structured.get("waited_for_idle", wait_for_idle)),
            **structured,
        }

    @api.post("/control/power-up", summary="Best-effort actuator power-up / arm")
    async def control_power_up():
        detail, structured = await run_in_threadpool(
            _controller_structured_call, "SAFE_POWER_UP", "SAFE_POWER_UP", timeout=5.0
        )
        return {"status": "ok", "detail": detail, **structured}

    @api.post(
        "/control/restart-controller",
        summary="Request graceful controller restart (external supervisor should restart process)",
    )
    async def control_restart_controller(payload: dict[str, Any] | None = None):
        reason = "api-request"
        if isinstance(payload, dict) and "reason" in payload:
            raw_reason = str(payload.get("reason", "")).strip()
            if raw_reason:
                reason = raw_reason.replace(",", ";")[:120]
        detail = await run_in_threadpool(
            _controller_call_or_503,
            f"REQUEST_RESTART,{reason}",
            timeout=2.0,
            expect_response=True,
        )
        return {
            "status": "ok",
            "detail": detail,
            "restart_requested": True,
            "reason": reason,
        }

    @api.post(
        "/control/runtime-mode",
        summary="Hot-switch controller runtime mode between LIVE and SIM without restart",
    )
    async def control_runtime_mode(payload: dict[str, Any] | None = None):
        if not isinstance(payload, dict):
            raise HTTPException(status_code=400, detail="JSON body must be an object.")
        mode = str(payload.get("mode", "") or "").strip().lower()
        if mode not in {"live", "simulate"}:
            raise HTTPException(status_code=400, detail="mode must be 'live' or 'simulate'")

        detail, structured = await run_in_threadpool(
            _controller_structured_call,
            f"SWITCH_RUNTIME_MODE,{mode}",
            "SWITCH_RUNTIME_MODE",
            timeout=20.0,
        )
        active_runtime = structured.get("runtime")
        if not isinstance(active_runtime, dict):
            active_runtime = await run_in_threadpool(_controller_runtime_config_call, 2.0)
        desired_config = runtime_config.load_runtime_config()
        restart_required = runtime_config.compute_restart_required(
            active_runtime=active_runtime,
            desired_config=desired_config,
        )
        return {
            "status": "ok",
            "detail": detail,
            "requested_mode": structured.get("requested_mode"),
            "active_mode": structured.get("active_mode"),
            "mode_changed": bool(structured.get("mode_changed", False)),
            "waited_for_idle": bool(structured.get("waited_for_idle", False)),
            "idle": structured.get("idle"),
            "active": active_runtime,
            "active_error": None,
            "desired": desired_config.get("desired", {}),
            "meta": desired_config.get("meta", {}),
            "restart_required": restart_required,
            "runtime_config_path": runtime_config.get_runtime_config_path(),
        }

    @api.post("/control/wait-for-idle", summary="Block until planner/trajectory motion completes")
    async def control_wait_for_idle(payload: dict[str, Any] | None = None):
        timeout_s = 30.0
        if payload is not None:
            if not isinstance(payload, dict):
                raise HTTPException(status_code=400, detail="JSON body must be an object if provided.")
            raw_timeout = payload.get("timeout_s")
            if raw_timeout is not None:
                try:
                    timeout_s = float(raw_timeout)
                except (TypeError, ValueError) as exc:
                    raise HTTPException(status_code=400, detail="Field 'timeout_s' must be a positive number.") from exc
                if timeout_s <= 0.0:
                    raise HTTPException(status_code=400, detail="Field 'timeout_s' must be a positive number.")
        command = "WAIT_FOR_IDLE" if timeout_s == 30.0 else f"WAIT_FOR_IDLE,{timeout_s}"
        detail = await run_in_threadpool(
            _controller_call_or_503,
            command,
            timeout=max(5.0, timeout_s + 5.0),
            expect_response=True,
        )
        payload = _parse_controller_ack_payload(detail, "WAIT_FOR_IDLE")
        return {"status": "ok", "detail": detail, **payload}

    @api.get("/control/motion-status", summary="Current controller/RTCore motion execution status")
    async def control_motion_status():
        detail = await run_in_threadpool(
            _controller_call_or_503, "GET_MOTION_STATUS", timeout=1.0, expect_response=True
        )
        payload = _parse_motion_status_reply(detail)
        return {"status": "ok", "detail": detail, **payload}

    @api.post(
        "/control/encoder-retention/capture",
        summary="Capture a before/after power-cycle encoder retention snapshot",
    )
    async def control_encoder_retention_capture(payload: dict[str, Any]):
        phase = payload.get("phase")
        if phase is None:
            raise HTTPException(status_code=400, detail="phase is required")
        notes_raw = payload.get("notes")
        notes = str(notes_raw).strip() if isinstance(notes_raw, str) and notes_raw.strip() else None
        experiment_id_raw = payload.get("experiment_id")
        experiment_id = (
            str(experiment_id_raw).strip()
            if isinstance(experiment_id_raw, str) and experiment_id_raw.strip()
            else None
        )
        snapshot_payload = await run_in_threadpool(
            _build_encoder_retention_capture_payload,
            phase=str(phase),
            notes=notes,
        )
        result = await run_in_threadpool(
            capture_retention_snapshot,
            phase=str(phase),
            snapshot_payload=snapshot_payload,
            experiment_id=experiment_id,
        )
        return {"status": "ok", **result}

    @api.post("/control/reset-faults", summary="Request DS402/drive fault reset")
    async def control_reset_faults(payload: dict[str, Any] | None = None):
        joint: int | None = None
        if isinstance(payload, dict) and payload.get("joint", payload.get("joint_num")) is not None:
            raw_joint = payload.get("joint", payload.get("joint_num"))
            try:
                joint = int(raw_joint)
            except Exception:
                raise HTTPException(status_code=400, detail="joint must be an integer")
            if joint <= 0:
                raise HTTPException(status_code=400, detail="joint must be >= 1")

        command = f"RESET_FAULTS,{joint}" if joint is not None else "RESET_FAULTS"
        detail, structured = await run_in_threadpool(
            _controller_structured_call, command, "RESET_FAULTS", timeout=5.0
        )
        return {"status": "ok", "detail": detail, "joint": joint, **structured}

    @api.post("/control/reset-encoder-data", summary="Request drive-side encoder data reset")
    async def control_reset_encoder_data(payload: dict[str, Any] | None = None):
        joint: int | None = None
        if isinstance(payload, dict) and payload.get("joint", payload.get("joint_num")) is not None:
            raw_joint = payload.get("joint", payload.get("joint_num"))
            try:
                joint = int(raw_joint)
            except Exception:
                raise HTTPException(status_code=400, detail="joint must be an integer")
            if joint <= 0:
                raise HTTPException(status_code=400, detail="joint must be >= 1")

        command = f"RESET_ENCODER_DATA,{joint}" if joint is not None else "RESET_ENCODER_DATA"
        detail, structured = await run_in_threadpool(
            _controller_structured_call, command, "RESET_ENCODER_DATA", timeout=5.0
        )
        return {"status": "ok", "detail": detail, "joint": joint, **structured}

    @api.post("/control/home", summary="Move all joints to zero position")
    async def control_home():
        detail = await run_in_threadpool(
            _controller_call_or_503,
            "APPLY_JOINT_SETPOINT,"
            + _encode_payload_b64(
                {
                    "arm_angles_rad": [0.0] * 6,
                    "max_motor_rpm": _SAFE_COMMISSIONING_MAX_MOTOR_RPM,
                }
            ),
            timeout=2.0,
            expect_response=True,
        )
        payload = _parse_controller_ack_payload(detail, "APPLY_JOINT_SETPOINT")
        return {
            "status": "ok",
            "detail": detail,
            "max_motor_rpm": _SAFE_COMMISSIONING_MAX_MOTOR_RPM,
            **payload,
        }

    @api.post("/control/zero-joint", summary="Capture the current physical pose as logical zero for one joint")
    async def control_zero_joint(payload: dict[str, Any]):
        raw_joint = payload.get("joint", payload.get("joint_num"))
        try:
            joint = int(raw_joint)
        except Exception:
            raise HTTPException(status_code=400, detail="joint must be an integer")
        if joint <= 0:
            raise HTTPException(status_code=400, detail="joint must be >= 1")
        detail = await run_in_threadpool(
            _controller_call_or_503,
            f"ZERO_JOINT,{joint}",
            timeout=5.0,
            expect_response=True,
        )
        return {"status": "ok", "joint": joint, "detail": detail}

    @api.post(
        "/control/home-joint-native",
        summary="Capture the current encoder position as a drive-native home for one joint",
    )
    async def control_home_joint_native(payload: dict[str, Any]):
        raw_joint = payload.get("joint", payload.get("joint_num"))
        try:
            joint = int(raw_joint)
        except Exception:
            raise HTTPException(status_code=400, detail="joint must be an integer")
        if joint <= 0:
            raise HTTPException(status_code=400, detail="joint must be >= 1")
        detail = await run_in_threadpool(
            _controller_call_or_503,
            f"NATIVE_HOME_JOINT,{joint}",
            timeout=5.0,
            expect_response=True,
        )
        return {"status": "ok", "joint": joint, "detail": detail}

    @api.post("/control/joint-jog", summary="Jog one joint by a relative angle in degrees")
    async def control_joint_jog(payload: dict[str, Any]):
        raw_joint = payload.get("joint", payload.get("joint_num"))
        try:
            joint = int(raw_joint)
        except Exception:
            raise HTTPException(status_code=400, detail="joint must be an integer")
        if joint <= 0:
            raise HTTPException(status_code=400, detail="joint must be >= 1")
        try:
            delta_deg = float(payload.get("delta_deg"))
        except Exception:
            raise HTTPException(status_code=400, detail="delta_deg must be a number")
        wait_for_idle = bool(payload.get("wait_for_idle", False))

        detail = await run_in_threadpool(
            _controller_call_or_503,
            "GET_JOINT_ANGLES",
            timeout=1.0,
            expect_response=True,
        )
        parts = detail.split(",")
        if not parts or parts[0] != "JOINT_ANGLES":
            raise HTTPException(status_code=502, detail=f"Malformed joint reply: {detail}")
        try:
            angles = list(map(float, parts[1:]))
        except ValueError as exc:
            raise HTTPException(status_code=502, detail=f"Invalid joint data: {exc}") from exc
        if len(angles) < joint:
            raise HTTPException(
                status_code=400,
                detail=f"Joint {joint} unavailable in controller feedback.",
            )

        target_arm_deg = list(angles[:6])
        target_arm_deg[joint - 1] = float(target_arm_deg[joint - 1]) + float(delta_deg)
        # Raw comma-separated joint commands are interpreted by the controller as radians.
        # `GET_JOINT_ANGLES` returns degrees for UI/API consumption, so convert back to
        # radians here before handing the command to the controller.
        target_arm_rad = [float(np.deg2rad(value)) for value in target_arm_deg]
        try:
            detail = await run_in_threadpool(
                _controller_call_or_503,
                "APPLY_JOINT_SETPOINT,"
                + _encode_payload_b64(
                    {
                        "arm_angles_rad": target_arm_rad,
                        "max_motor_rpm": _SAFE_COMMISSIONING_MAX_MOTOR_RPM,
                    }
                ),
                timeout=2.0,
                expect_response=True,
            )
        except HTTPException as exc:
            detail_text = exc.detail if isinstance(exc.detail, str) else json.dumps(exc.detail)
            raise _parse_apply_joint_setpoint_error(detail_text) from exc
        payload = _parse_controller_ack_payload(detail, "APPLY_JOINT_SETPOINT")
        command_acknowledged = bool(payload.get("accepted", detail.startswith("ACK,APPLY_JOINT_SETPOINT")))
        return {
            "status": "ok",
            "joint": joint,
            "delta_deg": delta_deg,
            "target_arm_deg": target_arm_deg,
            "target_arm_rad": target_arm_rad,
            "detail": detail,
            "command_acknowledged": command_acknowledged,
            "max_motor_rpm": _SAFE_COMMISSIONING_MAX_MOTOR_RPM,
            "wait_for_idle_requested": wait_for_idle,
            "waited_for_idle": False,
            **payload,
        }

    @api.post("/control/rest", summary="Move all joints to predefined REST pose")
    async def control_rest():
        detail = await run_in_threadpool(
            _controller_call_or_503,
            "APPLY_JOINT_SETPOINT,"
            + _encode_payload_b64(
                {
                    "arm_angles_rad": list(_resolve_rest_pose()),
                    "max_motor_rpm": _SAFE_COMMISSIONING_MAX_MOTOR_RPM,
                }
            ),
            timeout=2.0,
            expect_response=True,
        )
        payload = _parse_controller_ack_payload(detail, "APPLY_JOINT_SETPOINT")
        return {
            "status": "ok",
            "detail": detail,
            "max_motor_rpm": _SAFE_COMMISSIONING_MAX_MOTOR_RPM,
            **payload,
        }

    @api.post("/control/move-line-relative", summary="Move tool by dx,dy,dz with optional speed multiplier")
    async def control_move_line_relative(payload: dict[str, Any]):
        try:
            dx = float(payload.get("dx", 0.0))
            dy = float(payload.get("dy", 0.0))
            dz = float(payload.get("dz", 0.0))
        except Exception:
            raise HTTPException(status_code=400, detail="dx, dy, dz must be numbers")
        speed_multiplier = payload.get("speed_multiplier", None)
        try:
            sm = float(speed_multiplier) if speed_multiplier is not None else None
        except Exception:
            raise HTTPException(status_code=400, detail="speed_multiplier must be a number")
        closed = bool(payload.get("closed", _DEFAULT_MOVE_LINE_CLOSED_LOOP))
        # Command format: MOVE_LINE_RELATIVE,dx,dy,dz[,speed_multiplier][,closed]
        parts: list[str] = [
            "MOVE_LINE_RELATIVE",
            str(dx),
            str(dy),
            str(dz),
        ]
        if sm is not None:
            parts.append(str(sm))
        parts.append("true" if closed else "false")
        cmd = ",".join(parts)
        detail = await run_in_threadpool(_controller_call_or_503, cmd, timeout=2.0, expect_response=True)
        payload = _parse_controller_ack_payload(detail, "MOVE_LINE_RELATIVE")
        return {"status": "ok", "detail": detail, **payload}

    @api.post("/control/rotate", summary="Rotate tool by axis and angle in degrees (relative)")
    async def control_rotate(payload: dict[str, Any]):
        axis_in = str(payload.get("axis", "")).strip().lower()
        # Accept multiple synonyms and normalize to scipy-euler tokens: x/y/z
        axis_map = {
            "roll": "x", "r": "x", "x": "x",
            "pitch": "y", "p": "y", "y": "y",
            "yaw": "z", "w": "z", "z": "z",
        }
        axis = axis_map.get(axis_in)
        if axis is None:
            raise HTTPException(status_code=400, detail="axis must be one of roll,pitch,yaw (or x/y/z)")
        try:
            angle_deg = float(payload.get("angle_deg", 0.0))
        except Exception:
            raise HTTPException(status_code=400, detail="angle_deg must be a number")
        duration_s_token = payload.get("duration_s", None)
        duration_s: float | None = None
        if duration_s_token is not None:
            try:
                duration_s = float(duration_s_token)
            except Exception:
                raise HTTPException(status_code=400, detail="duration_s must be a number")
            if not np.isfinite(duration_s) or duration_s <= 0.0:
                raise HTTPException(status_code=400, detail="duration_s must be > 0")
        # Preserve relative-axis semantics by forwarding the command directly to the
        # controller's ROTATE path instead of reconstructing an absolute Euler target
        # from a fresh GET_POSITION sample.
        cmd = f"ROTATE,{axis},{angle_deg}"
        if duration_s is not None:
            cmd += f",{duration_s}"
        detail = await run_in_threadpool(
            _controller_call_or_503, cmd, timeout=2.0, expect_response=True
        )
        command_payload = _parse_controller_ack_payload(detail, "ROTATE")
        return {"status": "ok", "detail": detail, **command_payload}

    @api.post("/control/set-orientation", summary="Set absolute end-effector orientation (roll,pitch,yaw deg)")
    async def control_set_orientation(payload: dict[str, Any]):
        try:
            roll = float(payload.get("roll", 0.0))
            pitch = float(payload.get("pitch", 0.0))
            yaw = float(payload.get("yaw", 0.0))
        except Exception:
            raise HTTPException(status_code=400, detail="roll,pitch,yaw must be numbers")
        duration_s_token = payload.get("duration_s", None)
        duration_s: float | None = None
        if duration_s_token is not None:
            try:
                duration_s = float(duration_s_token)
            except Exception:
                raise HTTPException(status_code=400, detail="duration_s must be a number")
            if not np.isfinite(duration_s) or duration_s <= 0.0:
                raise HTTPException(status_code=400, detail="duration_s must be > 0")
        closed_token = payload.get("closed_loop", payload.get("closed", None))
        closed_loop: bool | None = None
        if closed_token is not None:
            if isinstance(closed_token, bool):
                closed_loop = closed_token
            elif isinstance(closed_token, (int, float)):
                closed_loop = bool(closed_token)
            else:
                token = str(closed_token).strip().lower()
                if token in {"true", "1", "yes", "closed", "on"}:
                    closed_loop = True
                elif token in {"false", "0", "no", "open", "off"}:
                    closed_loop = False
                else:
                    raise HTTPException(status_code=400, detail="closed_loop must be a boolean")
        parts = ["SET_ORIENTATION", str(roll), str(pitch), str(yaw)]
        if duration_s is not None or closed_loop is not None:
            parts.append("" if duration_s is None else str(duration_s))
        if closed_loop is not None:
            parts.append("true" if closed_loop else "false")
        cmd = ",".join(parts)
        detail = await run_in_threadpool(
            _controller_call_or_503, cmd, timeout=2.0, expect_response=True
        )
        command_payload = _parse_controller_ack_payload(detail, "SET_ORIENTATION")
        return {"status": "ok", "detail": detail, **command_payload}

    @api.post("/control/set-gripper", summary="Set gripper angle in degrees")
    async def control_set_gripper(payload: dict[str, Any]):
        try:
            angle = float(payload.get("angle_deg", 0.0))
        except Exception:
            raise HTTPException(status_code=400, detail="angle_deg must be a number")
        cmd = f"SET_GRIPPER,{angle}"
        await run_in_threadpool(_controller_call_or_503, cmd, timeout=1.0, expect_response=False)
        return {"status": "ok"}

    @api.post("/control/jog/debug", summary="Enable/disable jog debug logging")
    async def control_jog_debug(payload: dict[str, Any]):
        enabled = _coerce_request_bool("enabled", payload.get("enabled"), default=False)
        cmd = f"SET_JOG_DEBUG,{'true' if enabled else 'false'}"
        await run_in_threadpool(_controller_call_or_503, cmd, timeout=1.0, expect_response=False)
        return {"status": "ok"}

    @api.post("/control/jog/session/start", summary="Start a controller-owned jog session")
    async def control_jog_session_start(payload: dict[str, Any]):
        def _num(name: str, default: float = 0.0) -> float:
            try:
                return float(payload.get(name, default))
            except Exception:
                raise HTTPException(status_code=400, detail=f"{name} must be a number")

        session_payload: dict[str, Any] = {
            "owner_id": _jog_session_owner_id(payload),
            "seq": _coerce_request_int("seq", payload.get("seq"), default=0),
            "deadman": _coerce_request_bool("deadman", payload.get("deadman"), default=True),
            "vx": _num("vx"),
            "vy": _num("vy"),
            "vz": _num("vz"),
            "v_roll": _num("v_roll"),
            "v_pitch": _num("v_pitch"),
            "v_yaw": _num("v_yaw"),
            "gripper_velocity_deg_s": _num("gripper_velocity_deg_s"),
        }
        if "lease_timeout_s" in payload:
            session_payload["lease_timeout_s"] = _num("lease_timeout_s", _DEFAULT_JOG_SESSION_LEASE_TIMEOUT_S)
        snapshot = await run_in_threadpool(_controller_jog_session_call, "JOG_SESSION_START", session_payload)
        return {"status": "ok", "session": snapshot}

    @api.post("/control/jog/session/update", summary="Update an active controller-owned jog session")
    async def control_jog_session_update(payload: dict[str, Any]):
        session_id = str(payload.get("session_id", "") or "").strip()
        if not session_id:
            raise HTTPException(status_code=400, detail="session_id is required")

        def _num(name: str, default: float = 0.0) -> float:
            try:
                return float(payload.get(name, default))
            except Exception:
                raise HTTPException(status_code=400, detail=f"{name} must be a number")

        session_payload: dict[str, Any] = {
            "session_id": session_id,
            "owner_id": _jog_session_owner_id(payload),
            "seq": _coerce_request_int("seq", payload.get("seq"), default=0),
            "deadman": _coerce_request_bool("deadman", payload.get("deadman"), default=True),
            "vx": _num("vx"),
            "vy": _num("vy"),
            "vz": _num("vz"),
            "v_roll": _num("v_roll"),
            "v_pitch": _num("v_pitch"),
            "v_yaw": _num("v_yaw"),
            "gripper_velocity_deg_s": _num("gripper_velocity_deg_s"),
        }
        if "lease_timeout_s" in payload:
            session_payload["lease_timeout_s"] = _num("lease_timeout_s", _DEFAULT_JOG_SESSION_LEASE_TIMEOUT_S)
        snapshot = await run_in_threadpool(_controller_jog_session_call, "JOG_SESSION_UPDATE", session_payload)
        return {"status": "ok", "session": snapshot}

    @api.post("/control/jog/session/stop", summary="Stop a controller-owned jog session")
    async def control_jog_session_stop(payload: dict[str, Any]):
        session_id = str(payload.get("session_id", "") or "").strip()
        if not session_id:
            raise HTTPException(status_code=400, detail="session_id is required")
        session_payload: dict[str, Any] = {
            "session_id": session_id,
            "reason": str(payload.get("reason", "client-stop") or "client-stop"),
        }
        owner_id = str(payload.get("owner_id", "") or "").strip()
        if owner_id:
            session_payload["owner_id"] = owner_id
        snapshot = await run_in_threadpool(_controller_jog_session_call, "JOG_SESSION_STOP", session_payload)
        return {"status": "ok", "session": snapshot}

    @api.get("/control/jog/session/state", summary="Current controller-owned jog session state")
    async def control_jog_session_state():
        snapshot = await run_in_threadpool(_controller_jog_session_call, "GET_JOG_SESSION_STATE")
        return {"status": "ok", "session": snapshot}

    @api.get("/info/status", summary="Controller status snapshot")
    async def info_status():
        detail = await run_in_threadpool(_controller_call_or_503, "GET_STATUS")
        parts = detail.split(",")
        if len(parts) != 3 or parts[0] != "STATUS":
            raise HTTPException(status_code=502, detail=f"Malformed status reply: {detail}")
        return {"gripper_present": _parse_bool_token(parts[2])}

    @api.get("/info/robots", summary="Available robot policies for selection")
    async def info_robots():
        default_robot_id = robot_assets.get_default_robot_id()
        return {
            "default_robot_id": default_robot_id,
            "default_robot_name": get_robot_name_by_id(default_robot_id),
            "robots": list_robot_metadata(),
            "runtime_config_path": runtime_config.get_runtime_config_path(),
            "tool_library_path": tool_library.get_tool_library_path(),
        }

    @api.get("/tools/library", summary="List tool library entries with optional filtering")
    async def tools_library_list(
        robot_id: str | None = None,
        tool_type: str | None = None,
        q: str | None = None,
    ):
        try:
            tools = tool_library.list_tools(
                robot_id=robot_id.strip() if isinstance(robot_id, str) and robot_id.strip() else None,
                tool_type=tool_type.strip() if isinstance(tool_type, str) and tool_type.strip() else None,
                query=q,
            )
            library = tool_library.load_tool_library()
        except ValueError as exc:
            raise HTTPException(status_code=400, detail=str(exc)) from exc
        return {
            "tools": tools,
            "default_tool_id": library.get("default_tool_id"),
            "meta": library.get("meta", {}),
            "tool_library_path": tool_library.get_tool_library_path(),
        }

    @api.get("/tools/library/{tool_id}", summary="Get one tool definition")
    async def tools_library_get(tool_id: str):
        try:
            return tool_library.get_tool(tool_id)
        except ValueError as exc:
            raise HTTPException(status_code=404, detail=str(exc)) from exc

    @api.post("/tools/library", summary="Create a new tool definition")
    async def tools_library_create(payload: dict[str, Any]):
        if not isinstance(payload, dict):
            raise HTTPException(status_code=400, detail="JSON body required.")
        actor = str(payload.get("actor", "api")).strip() or "api"
        tool_payload = payload.get("tool")
        if not isinstance(tool_payload, dict):
            raise HTTPException(status_code=400, detail="Field 'tool' is required and must be an object.")
        try:
            tool_id = str(tool_payload.get("tool_id", "")).strip()
            if not tool_id:
                raise ValueError("Field 'tool.tool_id' is required.")
            existing = None
            try:
                existing = tool_library.get_tool(tool_id)
            except ValueError:
                existing = None
            if existing is not None:
                raise HTTPException(status_code=409, detail=f"Tool '{tool_id}' already exists.")
            saved = tool_library.upsert_tool(tool_payload, actor=actor)
        except HTTPException:
            raise
        except ValueError as exc:
            raise HTTPException(status_code=400, detail=str(exc)) from exc
        return {
            "status": "ok",
            "tool": tool_library.get_tool(str(tool_payload.get("tool_id"))),
            "default_tool_id": saved.get("default_tool_id"),
            "meta": saved.get("meta", {}),
        }

    @api.patch("/tools/library/{tool_id}", summary="Update a tool definition")
    async def tools_library_update(tool_id: str, payload: dict[str, Any]):
        if not isinstance(payload, dict):
            raise HTTPException(status_code=400, detail="JSON body required.")
        actor = str(payload.get("actor", "api")).strip() or "api"
        patch_tool = payload.get("tool")
        if not isinstance(patch_tool, dict):
            raise HTTPException(status_code=400, detail="Field 'tool' is required and must be an object.")
        try:
            current = tool_library.get_tool(tool_id)
        except ValueError as exc:
            raise HTTPException(status_code=404, detail=str(exc)) from exc
        merged = dict(current)
        merged.update(patch_tool)
        merged["tool_id"] = current.get("tool_id")
        try:
            saved = tool_library.upsert_tool(merged, actor=actor)
        except ValueError as exc:
            raise HTTPException(status_code=400, detail=str(exc)) from exc
        return {
            "status": "ok",
            "tool": tool_library.get_tool(str(current.get("tool_id"))),
            "default_tool_id": saved.get("default_tool_id"),
            "meta": saved.get("meta", {}),
        }

    @api.delete("/tools/library/{tool_id}", summary="Delete a tool definition")
    async def tools_library_delete(tool_id: str, actor: str = "api"):
        try:
            saved = tool_library.delete_tool(tool_id, actor=actor)
        except ValueError as exc:
            message = str(exc)
            status_code = 404 if "Unknown tool_id" in message else 400
            raise HTTPException(status_code=status_code, detail=message) from exc
        return {
            "status": "ok",
            "deleted_tool_id": tool_id,
            "default_tool_id": saved.get("default_tool_id"),
            "meta": saved.get("meta", {}),
        }

    @api.get("/info/runtime-config", summary="Active vs desired runtime config with restart-required flag")
    async def info_runtime_config():
        desired_config = runtime_config.load_runtime_config()
        active_runtime: dict[str, Any] | None = None
        active_error: str | None = None
        try:
            active_runtime = await run_in_threadpool(_controller_runtime_config_call, 2.0)
        except HTTPException as exc:
            active_error = str(exc.detail)

        restart_required = runtime_config.compute_restart_required(
            active_runtime=active_runtime,
            desired_config=desired_config,
        )
        return {
            "active": active_runtime,
            "active_error": active_error,
            "desired": desired_config.get("desired", {}),
            "meta": desired_config.get("meta", {}),
            "restart_required": restart_required,
            "runtime_config_path": runtime_config.get_runtime_config_path(),
        }

    @api.patch(
        "/info/runtime-config",
        summary="Stage desired runtime config (tool applies live; sim mode hot-switches; robot/backend changes require restart)",
    )
    async def patch_runtime_config(payload: dict[str, Any]):
        if not isinstance(payload, dict):
            raise HTTPException(status_code=400, detail="JSON body required.")
        actor_raw = payload.get("actor", "api")
        actor = str(actor_raw).strip() if str(actor_raw).strip() else "api"
        try:
            updated = runtime_config.update_runtime_config_desired(payload, actor=actor)
        except ValueError as exc:
            raise HTTPException(status_code=400, detail=str(exc)) from exc

        active_error: str | None = None
        if "active_tool_id" in payload:
            try:
                await run_in_threadpool(
                    _controller_set_active_tool_call,
                    payload.get("active_tool_id"),
                    2.0,
                )
            except HTTPException as exc:
                active_error = f"Live tool apply failed: {exc.detail}"

        active_runtime: dict[str, Any] | None = None
        try:
            active_runtime = await run_in_threadpool(_controller_runtime_config_call, 2.0)
        except HTTPException as exc:
            runtime_error = str(exc.detail)
            active_error = f"{active_error} | {runtime_error}" if active_error else runtime_error

        restart_required = runtime_config.compute_restart_required(
            active_runtime=active_runtime,
            desired_config=updated,
        )
        return {
            "status": "ok",
            "active": active_runtime,
            "active_error": active_error,
            "desired": updated.get("desired", {}),
            "meta": updated.get("meta", {}),
            "restart_required": restart_required,
            "runtime_config_path": runtime_config.get_runtime_config_path(),
        }

    @api.get("/info/pose", summary="Current tool pose and joint angles")
    async def info_pose():
        detail = await run_in_threadpool(_controller_call_or_503, "GET_POSITION", timeout=1.0)
        parts = detail.split(",")
        try:
            return _parse_pose_snapshot_response(detail)
        except ValueError as exc:
            raise HTTPException(status_code=502, detail=str(exc)) from exc

    @api.get("/debug/performance", summary="Controller/API/RTCore performance snapshot")
    async def debug_performance():
        detail = await run_in_threadpool(_controller_call_or_503, "GET_PERFORMANCE_STATE", timeout=1.0)
        try:
            controller_payload = _parse_performance_state_response(detail)
        except ValueError as exc:
            raise HTTPException(status_code=502, detail=str(exc)) from exc

        try:
            pose_detail = await run_in_threadpool(_controller_call_or_503, "GET_POSITION", timeout=1.0)
            controller_payload["pose"] = _parse_pose_snapshot_response(pose_detail)
        except (HTTPException, ValueError):
            # Keep timing diagnostics available even if a pose sample misses.
            pass

        return {
            "collected_at": datetime.datetime.now(datetime.timezone.utc).isoformat(),
            "api_udp": _get_api_command_metrics_snapshot(),
            "controller": controller_payload,
            "rtcore": _load_rtcore_metrics_summary(),
        }

    @api.post("/debug/pose-history", summary="Persist captured pose history locally")
    async def debug_pose_history_save(payload: dict[str, Any] = Body(...)):
        if not isinstance(payload, dict):
            raise HTTPException(status_code=400, detail="Pose history payload must be a JSON object.")
        pose_history = payload.get("pose_history")
        if not isinstance(pose_history, list) or len(pose_history) == 0:
            raise HTTPException(status_code=400, detail="Pose history payload must include a non-empty 'pose_history' list.")

        os.makedirs(_DIAGNOSTICS_LOG_DIR, exist_ok=True)
        filename = _pose_history_filename(payload.get("exported_at") if isinstance(payload.get("exported_at"), str) else None)
        output_path = os.path.join(_DIAGNOSTICS_LOG_DIR, filename)
        stored_payload = dict(payload)
        stored_payload["saved_at"] = datetime.datetime.now(datetime.timezone.utc).isoformat()
        stored_payload["sample_count"] = len(pose_history)
        with open(output_path, "w", encoding="utf-8") as fh:
            json.dump(stored_payload, fh, indent=2, ensure_ascii=True)
            fh.write("\n")

        return {
            "status": "ok",
            "path": output_path,
            "sample_count": len(pose_history),
        }

    @api.get("/info/orientation", summary="Current end-effector orientation matrix")
    async def info_orientation():
        detail = await run_in_threadpool(_controller_call_or_503, "GET_ORIENTATION", timeout=1.0)
        parts = detail.split(",")
        if not parts or parts[0] != "CURRENT_ORIENTATION" or len(parts) != 10:
            raise HTTPException(status_code=502, detail=f"Malformed orientation reply: {detail}")
        try:
            matrix_values = list(map(float, parts[1:]))
        except ValueError as exc:
            raise HTTPException(status_code=502, detail=f"Invalid orientation data: {exc}") from exc
        return {
            "matrix": [
                matrix_values[0:3],
                matrix_values[3:6],
                matrix_values[6:9],
            ]
        }

    @api.get("/info/joints", summary="Current joint angles")
    async def info_joints():
        detail = await run_in_threadpool(_controller_call_or_503, "GET_JOINT_ANGLES")
        try:
            arm, gripper = _parse_joint_angles_response(detail)
        except ValueError as exc:
            raise HTTPException(status_code=502, detail=str(exc)) from exc
        payload = {
            "arm_deg": arm,
            "arm_rad": [float(np.deg2rad(value)) for value in arm],
        }
        if gripper is not None:
            payload["gripper_deg"] = gripper
            payload["gripper_rad"] = float(np.deg2rad(gripper))
        return payload

    @api.get("/info/joints-detailed", summary="Current joint angles with raw feedback detail")
    async def info_joints_detailed():
        detail = await run_in_threadpool(_controller_call_or_503, "GET_JOINT_STATE", timeout=1.0)
        try:
            payload = _parse_joint_state_response(detail)
        except ValueError as exc:
            raise HTTPException(status_code=502, detail=str(exc)) from exc
        arm_deg = payload.get("arm_deg")
        arm_rad = payload.get("arm_rad")
        if not isinstance(arm_deg, list) or not isinstance(arm_rad, list):
            raise HTTPException(status_code=502, detail=f"Malformed detailed joint reply: {detail}")
        return payload

    @api.get("/debug/runtime", summary="Host/runtime diagnostics snapshot")
    async def debug_runtime():
        return await run_in_threadpool(
            get_runtime_diagnostics_snapshot,
            _PROJECT_ROOT_DIR,
            include_local_probes=True,
        )

    @api.get("/info/gripper", summary="Gripper angle snapshot")
    async def info_gripper():
        detail = await run_in_threadpool(_controller_call_or_503, "GET_GRIPPER_STATE")
        parts = detail.split(",")
        if len(parts) != 3 or parts[0] != "GRIPPER_STATE":
            raise HTTPException(status_code=502, detail=f"Malformed gripper reply: {detail}")
        try:
            angle_deg = float(parts[1])
            raw = int(float(parts[2]))
        except ValueError as exc:
            raise HTTPException(status_code=502, detail=f"Invalid gripper data: {exc}") from exc
        return {"angle_deg": angle_deg, "raw_position": raw}

    @api.get("/info/all-positions", summary="Raw servo positions")
    async def info_all_positions():
        detail = await run_in_threadpool(
            _controller_call_or_503, "GET_ALL_POSITIONS", timeout=1.0
        )
        parts = detail.split(",")
        if not parts or parts[0] != "ALL_POS_DATA" or len(parts) < 3:
            raise HTTPException(status_code=502, detail=f"Malformed positions reply: {detail}")
        if (len(parts) - 1) % 2 != 0:
            raise HTTPException(status_code=502, detail=f"Unexpected positions payload: {detail}")
        payload = []
        for i in range(1, len(parts), 2):
            servo_id = parts[i]
            position = parts[i + 1]
            try:
                servo_id_int = int(servo_id)
            except ValueError:
                servo_id_int = servo_id  # fall back to raw string if malformed
            try:
                position_int: int | None = None if position == "FAIL" else int(position)
            except ValueError:
                position_int = None
            payload.append({"servo_id": servo_id_int, "raw_position": position_int})
        return {"servos": payload}

    @api.get("/kinematics/profile", summary="Active kinematics profile and runtime offsets")
    async def kinematics_profile():
        payload = await run_in_threadpool(_controller_kinematics_call, "GET_KINEMATICS_PROFILE")
        return payload

    @api.patch("/kinematics/runtime-offsets", summary="Patch runtime TCP/base offsets with CAS revision")
    async def kinematics_patch_runtime_offsets(payload: dict[str, Any]):
        if not isinstance(payload, dict):
            raise HTTPException(status_code=400, detail="JSON body required.")
        expected_revision_raw = payload.get("expected_revision")
        if expected_revision_raw is None:
            expected_revision_token = ""
            expected_revision = None
        else:
            try:
                expected_revision = int(expected_revision_raw)
            except (TypeError, ValueError):
                raise HTTPException(status_code=400, detail="expected_revision must be an integer.")
            expected_revision_token = str(expected_revision)

        patch_payload: dict[str, Any] = {}
        if "base" in payload:
            patch_payload["base"] = payload["base"]
        if "tool" in payload:
            patch_payload["tool"] = payload["tool"]
        if not patch_payload:
            raise HTTPException(status_code=400, detail="Body must include 'base' and/or 'tool'.")

        encoded = _encode_payload_b64(patch_payload)
        command = f"PATCH_RUNTIME_OFFSETS,{expected_revision_token},{encoded}"
        state = await run_in_threadpool(_controller_kinematics_call, command, 2.0)

        # Keep API-local planning process aligned when controller and API are separate.
        try:
            kinematics_runtime.patch_runtime_offsets(
                patch_payload,
                expected_revision=None,
                motion_state="IDLE",
            )
        except Exception:
            pass
        return state

    @api.post("/kinematics/runtime-offsets/reset", summary="Reset runtime TCP/base offsets")
    async def kinematics_reset_runtime_offsets(payload: dict[str, Any] | None = None):
        expected_revision = None
        expected_revision_token = ""
        if isinstance(payload, dict) and "expected_revision" in payload:
            try:
                expected_revision = int(payload.get("expected_revision"))
            except (TypeError, ValueError):
                raise HTTPException(status_code=400, detail="expected_revision must be an integer.")
            expected_revision_token = str(expected_revision)
        command = f"RESET_RUNTIME_OFFSETS,{expected_revision_token}"
        state = await run_in_threadpool(_controller_kinematics_call, command, 2.0)
        try:
            kinematics_runtime.reset_runtime_offsets(
                expected_revision=None,
                motion_state="IDLE",
            )
        except Exception:
            pass
        return state

    @api.post("/kinematics/profile/apply", summary="Apply a kinematics profile revision")
    async def kinematics_apply_profile(payload: dict[str, Any]):
        if not isinstance(payload, dict):
            raise HTTPException(status_code=400, detail="JSON body required.")
        profile_payload = payload.get("profile")
        if not isinstance(profile_payload, dict):
            raise HTTPException(status_code=400, detail="Field 'profile' is required and must be an object.")
        expected_revision_raw = payload.get("expected_revision")
        if expected_revision_raw is None:
            expected_revision_token = ""
            expected_revision = None
        else:
            try:
                expected_revision = int(expected_revision_raw)
            except (TypeError, ValueError):
                raise HTTPException(status_code=400, detail="expected_revision must be an integer.")
            expected_revision_token = str(expected_revision)
        encoded = _encode_payload_b64(profile_payload)
        command = f"APPLY_KINEMATICS_PROFILE,{expected_revision_token},{encoded}"
        state = await run_in_threadpool(_controller_kinematics_call, command, 2.0)
        try:
            backend_name = os.environ.get("MINI_ARM_SOLVER", "ikfast").strip().lower()
            kinematics_runtime.apply_profile_payload(
                profile_payload,
                expected_revision=None,
                motion_state="IDLE",
                backend_name=backend_name,
            )
        except Exception:
            pass
        return state

    @api.get("/monitor", summary="Subscribe to controller telemetry stream")
    async def monitor():
        token, queue = await telemetry_hub.register()

        async def event_generator():
            try:
                while True:
                    message = await queue.get()
                    yield message
            except asyncio.CancelledError:
                raise
            finally:
                await telemetry_hub.unregister(token)

        return EventSourceResponse(event_generator(), ping=15)

    @api.post("/cad/topology/load-step", summary="Load STEP topology from exact CAD edges")
    async def cad_topology_load_step(payload: dict[str, Any]):
        if not isinstance(payload, dict):
            raise HTTPException(status_code=400, detail="JSON body required.")
        filename, step_bytes = _decode_base64_step_payload(payload)
        sample_count = payload.get("sample_count", 64)
        try:
            sample_count_int = int(sample_count)
        except Exception:
            raise HTTPException(status_code=400, detail="sample_count must be an integer")

        def _load():
            return topology_service.load_step(
                filename=filename,
                step_bytes=step_bytes,
                sample_count=sample_count_int,
            )

        try:
            return await run_in_threadpool(_load)
        except TopologyDependencyError as exc:
            raise HTTPException(status_code=503, detail=str(exc)) from exc
        except ValueError as exc:
            raise HTTPException(status_code=400, detail=str(exc)) from exc
        except Exception as exc:
            raise HTTPException(status_code=500, detail=f"Topology extraction failed: {exc}") from exc

    @api.get("/cad/topology/{model_id}", summary="Fetch topology edges for a loaded STEP model")
    async def cad_topology_detail(model_id: str):
        try:
            return topology_service.get_model(model_id)
        except TopologyModelNotFoundError:
            raise HTTPException(status_code=404, detail=f"Unknown topology model '{model_id}'.")

    @api.post("/robot-program/save", summary="Save an editable robot program record")
    async def robot_program_save(payload: dict[str, Any]):
        if not isinstance(payload, dict):
            raise HTTPException(status_code=400, detail="JSON body required.")
        kind = _normalize_program_kind(payload.get("kind"))
        record = (
            _build_weld_program_record_from_payload(payload)
            if kind == "weld"
            else _build_trajectory_program_record_from_payload(payload)
        )
        saved = _save_robot_program_record(record)
        return {"status": "ok", "name": saved["name"], "kind": saved["kind"]}

    @api.get("/robot-program/list", summary="List saved editable robot programs")
    async def robot_program_list(kind: str | None = None):
        names = _list_robot_program_names(kind)
        return {"programs": names, "kind": _normalize_program_kind(kind) if kind is not None else None}

    @api.get("/robot-program/{name}", summary="Load a saved editable robot program")
    async def robot_program_detail(name: str, kind: str | None = None):
        record = _load_robot_program_record(name, expected_kind=kind)
        return record

    @api.post("/weld-program/save", summary="Save weld program with embedded STEP payload")
    async def weld_program_save(payload: dict[str, Any]):
        if not isinstance(payload, dict):
            raise HTTPException(status_code=400, detail="JSON body required.")
        enriched_payload = dict(payload)
        enriched_payload["kind"] = "weld"
        record = _save_robot_program_record(_build_weld_program_record_from_payload(enriched_payload))
        return {"status": "ok", "name": record["name"]}

    @api.get("/weld-program/list", summary="List saved weld programs")
    async def weld_program_list():
        return {"programs": _list_robot_program_names("weld")}

    @api.get("/weld-program/{name}", summary="Load a saved weld program")
    async def weld_program_detail(name: str):
        return _serialize_weld_program_record(_load_robot_program_record(name, expected_kind="weld"))

    @api.post("/trajectory/plan", summary="Begin recording a new trajectory")
    async def trajectory_plan():
        await run_in_threadpool(
            _controller_call_or_503,
            "PLAN_TRAJECTORY",
            timeout=1.0,
            expect_response=False,
        )
        return {"status": "ok"}

    @api.post("/trajectory/record", summary="Record current pose into active trajectory")
    async def trajectory_record():
        await run_in_threadpool(
            _controller_call_or_503, "REC_POS", timeout=1.0, expect_response=False
        )
        return {"status": "ok"}

    @api.post("/trajectory/end", summary="Finish trajectory recording and save by name")
    async def trajectory_end(payload: dict):
        name = (payload or {}).get("name")
        if not isinstance(name, str) or not name.strip():
            raise HTTPException(status_code=400, detail="Field 'name' is required.")
        command = f"END_TRAJECTORY,{name.strip()}"
        await run_in_threadpool(
            _controller_call_or_503, command, timeout=2.0, expect_response=False
        )
        return {"status": "ok", "name": name.strip()}

    @api.post("/trajectory/plan-points", summary="Plan joint path for custom Cartesian way-points")
    async def trajectory_plan_points(payload: dict):
        if not isinstance(payload, dict):
            raise HTTPException(status_code=400, detail="JSON body required.")
        if controller_command_api is None:
            raise HTTPException(status_code=500, detail="Arm controller command API unavailable")

        pose_waypoints = _coerce_pose_waypoint_list(payload.get("waypoints"))
        if not pose_waypoints:
            points = _coerce_waypoint_list(payload.get("points"))
            pose_waypoints = [{"x": x, "y": y, "z": z} for (x, y, z) in points]
        if not pose_waypoints:
            raise HTTPException(
                status_code=400,
                detail="Field 'waypoints' or 'points' must contain at least one waypoint.",
            )

        preview_name_raw = payload.get("preview_name")
        preview_name = (
            str(preview_name_raw).strip()
            if isinstance(preview_name_raw, str) and preview_name_raw.strip()
            else getattr(controller_command_api, "PLANNED_PREVIEW_NAME", "__planner_preview__")
        )

        def _plan():
            _sync_local_planner_runtime(timeout=1.0)
            live_joints_rad = _get_live_joint_angles_from_controller(timeout=1.0)
            if controller_utils is not None:
                controller_utils.current_logical_joint_angles_rad = list(live_joints_rad)
            if hasattr(controller_command_api, "utils"):
                controller_command_api.utils.current_logical_joint_angles_rad = list(live_joints_rad)
            return controller_command_api.plan_preview_trajectory_points(
                [[float(item["x"]), float(item["y"]), float(item["z"])] for item in pose_waypoints],
                preview_name=preview_name,
                pose_waypoints=pose_waypoints,
            )

        try:
            payload_dict = await run_in_threadpool(_plan)
        except RuntimeError as exc:
            raise HTTPException(
                status_code=502,
                detail=_planner_failure_detail(f"Trajectory planning failed: {exc}"),
            ) from exc
        except ValueError as exc:
            raise HTTPException(status_code=400, detail=str(exc)) from exc
        except Exception as exc:
            raise HTTPException(
                status_code=500,
                detail=_planner_failure_detail(f"Trajectory planning failed unexpectedly: {exc}"),
            ) from exc

        payload_dict["source"] = {
            "mode": "pose_waypoints" if payload.get("waypoints") is not None else "points",
            "waypoint_count": len(pose_waypoints),
        }
        return payload_dict

    @api.post("/trajectory/plan-weld", summary="Plan a weld trajectory from selected CAD edge segment")
    async def trajectory_plan_weld(payload: dict[str, Any]):
        if not isinstance(payload, dict):
            raise HTTPException(status_code=400, detail="JSON body required.")
        if controller_command_api is None:
            raise HTTPException(status_code=500, detail="Arm controller command API unavailable")

        model_id = payload.get("model_id")
        edge_id = payload.get("edge_id")
        if not isinstance(model_id, str) or not model_id.strip():
            raise HTTPException(status_code=400, detail="Field 'model_id' is required.")
        if not isinstance(edge_id, str) or not edge_id.strip():
            raise HTTPException(status_code=400, detail="Field 'edge_id' is required.")

        try:
            start_s = float(payload.get("start_s", 0.0))
            end_s = float(payload.get("end_s", 1.0))
        except Exception:
            raise HTTPException(status_code=400, detail="start_s and end_s must be numbers in [0, 1].")
        sample_count_raw = payload.get("sample_count", 40)
        try:
            sample_count = max(2, int(sample_count_raw))
        except Exception:
            raise HTTPException(status_code=400, detail="sample_count must be an integer.")

        weld_type = _normalize_weld_type(payload.get("weld_type", "fillet"))
        weld_name_raw = payload.get("weld_name")
        weld_name = str(weld_name_raw).strip() if isinstance(weld_name_raw, str) else ""
        if not weld_name:
            weld_name = f"{weld_type} weld"

        waypoints_override = _coerce_waypoint_list(payload.get("waypoints_override"))
        sections = _coerce_plan_sections(payload.get("sections"))
        preview_name_raw = payload.get("preview_name")
        preview_name = (
            str(preview_name_raw).strip()
            if isinstance(preview_name_raw, str) and preview_name_raw.strip()
            else getattr(controller_command_api, "WELD_PREVIEW_NAME", "__weld_preview__")
        )

        weld_options = payload.get("options")
        if weld_options is None:
            weld_options = {}
        elif not isinstance(weld_options, dict):
            raise HTTPException(status_code=400, detail="options must be an object.")
        post_action_raw = str(weld_options.get("post_action", "return_to_start")).strip()
        weld_options["post_action"] = (
            "none"
            if post_action_raw == "none"
            else ("lift" if post_action_raw == "lift" else "return_to_start")
        )
        try:
            spin_angle_deg = float(
                weld_options.get("spin_angle_deg", weld_options.get("spinAngleDeg", 0.0))
            )
        except Exception:
            spin_angle_deg = 0.0
        weld_options["spin_angle_deg"] = spin_angle_deg

        if sections:
            weld_points = [
                [float(point[0]), float(point[1]), float(point[2])]
                for section in sections
                for point in section["points"]
            ]
            sampled_start = max(0.0, min(1.0, start_s))
            sampled_end = max(0.0, min(1.0, end_s))
        elif waypoints_override:
            weld_points = [list(point) for point in waypoints_override]
            sampled_start = max(0.0, min(1.0, start_s))
            sampled_end = max(0.0, min(1.0, end_s))
        else:
            try:
                sampled = topology_service.sample_edge_segment(
                    model_id=model_id.strip(),
                    edge_id=edge_id.strip(),
                    start_s=start_s,
                    end_s=end_s,
                    sample_count=sample_count,
                )
            except TopologyModelNotFoundError:
                raise HTTPException(status_code=404, detail=f"Unknown topology model '{model_id}'.")
            except KeyError as exc:
                raise HTTPException(status_code=404, detail=str(exc)) from exc
            except ValueError as exc:
                raise HTTPException(status_code=400, detail=str(exc)) from exc
            weld_points = [[float(p[0]), float(p[1]), float(p[2])] for p in sampled]
            sampled_start = max(0.0, min(1.0, start_s))
            sampled_end = max(0.0, min(1.0, end_s))

        weld_metadata = {
            "type": weld_type,
            "name": weld_name,
            "model_id": model_id.strip(),
            "edge_id": edge_id.strip(),
            "start_s": sampled_start,
            "end_s": sampled_end,
            "options": weld_options,
        }

        def _plan():
            _sync_local_planner_runtime(timeout=1.0)
            live_joints_rad = _get_live_joint_angles_from_controller(timeout=1.0)
            if controller_utils is not None:
                controller_utils.current_logical_joint_angles_rad = list(live_joints_rad)
            if hasattr(controller_command_api, "utils"):
                controller_command_api.utils.current_logical_joint_angles_rad = list(live_joints_rad)
            return controller_command_api.plan_preview_trajectory_points(
                weld_points,
                preview_name=preview_name,
                weld_metadata=weld_metadata,
                sections=sections if sections else None,
            )

        try:
            result = await run_in_threadpool(_plan)
        except RuntimeError as exc:
            raise HTTPException(
                status_code=502,
                detail=_planner_failure_detail(f"Weld planning failed: {exc}"),
            ) from exc
        except ValueError as exc:
            raise HTTPException(status_code=400, detail=str(exc)) from exc
        except Exception as exc:
            raise HTTPException(
                status_code=500,
                detail=_planner_failure_detail(f"Weld planning failed unexpectedly: {exc}"),
            ) from exc

        result["source"] = {
            "mode": "sections" if sections else ("waypoints_override" if waypoints_override else "edge_segment"),
            "sample_count": len(weld_points),
            "section_count": len(sections) if sections else 0,
        }
        return result

    @api.get("/trajectory/detail/{name}", summary="Fetch the definition of a recorded trajectory")
    async def trajectory_detail(name: str):
        if not isinstance(name, str) or not name.strip():
            raise HTTPException(status_code=400, detail="Trajectory name is required.")

        def _load() -> dict[str, Any] | None:
            if controller_command_api is None:
                raise RuntimeError("Arm controller command API unavailable")
            return controller_command_api._load_trajectory_by_name(name.strip())

        try:
            trajectory = await run_in_threadpool(_load)
        except RuntimeError as exc:
            raise HTTPException(status_code=500, detail=str(exc)) from exc

        if trajectory is None:
            raise HTTPException(status_code=404, detail=f"Trajectory '{name}' not found.")
        return {"name": name.strip(), "trajectory": trajectory}

    @api.get("/trajectory/list", summary="List available recorded trajectories")
    async def trajectory_list():
        detail = await run_in_threadpool(_controller_call_or_503, "GET_TRAJECTORIES")
        parts = detail.split(",")
        if not parts or parts[0] != "TRAJECTORIES":
            raise HTTPException(status_code=502, detail=f"Malformed trajectory list: {detail}")
        names = [name for name in parts[1:] if name]
        return {"trajectories": names}

    @api.post("/trajectory/run", summary="Execute a recorded trajectory")
    async def trajectory_run(payload: dict):
        if not isinstance(payload, dict):
            raise HTTPException(status_code=400, detail="JSON body required.")
        name = payload.get("name")
        if not isinstance(name, str) or not name.strip():
            raise HTTPException(status_code=400, detail="Field 'name' is required.")
        execution_mode_raw = payload.get("execution_mode")
        execution_mode = (
            str(execution_mode_raw).strip().lower()
            if isinstance(execution_mode_raw, str) and str(execution_mode_raw).strip()
            else None
        )
        if execution_mode not in {None, "simulate", "live"}:
            raise HTTPException(status_code=400, detail="execution_mode must be 'simulate' or 'live'.")
        runtime_snapshot = await run_in_threadpool(_controller_runtime_config_call, 1.5)
        runtime_is_sim = _runtime_is_sim_mode(runtime_snapshot)
        runtime_mode = "simulate" if runtime_is_sim else "live"
        if execution_mode == "simulate" and not runtime_is_sim:
            raise HTTPException(
                status_code=409,
                detail="Simulated execution requires the controller to be running in SIM mode.",
            )
        if execution_mode == "live" and runtime_is_sim:
            raise HTTPException(
                status_code=409,
                detail="Live execution is unavailable while the controller is running in SIM mode.",
            )
        use_cache = payload.get("use_cache", False)
        loop_override = payload.get("loop_override")
        parts = [name.strip()]
        if isinstance(use_cache, bool):
            parts.append("true" if use_cache else "false")
        else:
            parts.append("" if use_cache is None else str(use_cache))
        if isinstance(loop_override, bool):
            parts.append("true" if loop_override else "false")
        elif loop_override is not None:
            parts.append(str(loop_override))
        command = "RUN_TRAJECTORY," + ",".join(parts)
        request_execution_mode = execution_mode or runtime_mode
        try:
            detail = await run_in_threadpool(
                _controller_call_or_503, command, timeout=2.0, expect_response=True
            )
        except HTTPException as exc:
            timeout_detail = exc.detail if isinstance(exc.detail, str) else ""
            if exc.status_code == 503 and timeout_detail == f"No response for command '{command}'":
                inferred = await _infer_run_trajectory_timeout_acceptance(
                    trajectory_name=name.strip(),
                    runtime_mode=runtime_mode,
                    execution_mode=request_execution_mode,
                    timeout_detail=timeout_detail,
                )
                if inferred is not None:
                    return inferred
            raise
        payload = _parse_controller_ack_payload(detail, "RUN_TRAJECTORY")
        return {
            "status": "ok",
            "detail": detail,
            "execution_mode": request_execution_mode,
            "runtime_mode": runtime_mode,
            **payload,
        }

    @api.post("/trajectory/preview", summary="Plan a trajectory to a target point")
    async def trajectory_preview(payload: dict):
        plan = await _plan_point(payload)
        async with _latest_plan_lock:
            global _latest_plan
            _latest_plan = plan
        return plan

    @api.post("/trajectory/execute-preview", summary="Execute the last planned preview trajectory")
    async def trajectory_execute_preview():
        global _latest_plan
        async with _latest_plan_lock:
            plan = _latest_plan
        if plan is None:
            raise HTTPException(status_code=404, detail="No planned trajectory is available.")

        target = plan.get("target", {})
        try:
            x = float(target["x"])
            y = float(target["y"])
            z = float(target["z"])
        except (KeyError, TypeError, ValueError):
            raise HTTPException(status_code=500, detail="Stored plan is invalid.")

        velocity = float(plan.get("velocity", 0.1))
        acceleration = float(plan.get("acceleration", 0.05))
        closed_loop = bool(plan.get("closed_loop", _DEFAULT_MOVE_LINE_CLOSED_LOOP))
        closed_loop_token = "true" if closed_loop else "false"

        command = f"MOVE_LINE,{x},{y},{z},{velocity},{acceleration},{closed_loop_token}"
        dispatch_detail = await run_in_threadpool(
            _controller_call_or_503, command, timeout=5.0, expect_response=True
        )
        dispatch_payload = _parse_controller_ack_payload(dispatch_detail, "MOVE_LINE")
        completion_detail = await run_in_threadpool(
            _controller_call_or_503, "WAIT_FOR_IDLE", timeout=60.0, expect_response=True
        )
        completion_payload = _parse_controller_ack_payload(completion_detail, "WAIT_FOR_IDLE")

        async with _latest_plan_lock:
            _latest_plan = None
        return {
            "status": "ok",
            "dispatch_detail": dispatch_detail,
            "dispatch": dispatch_payload,
            "detail": completion_detail,
            **completion_payload,
        }

    @api.post("/trajectory/clear-preview", summary="Discard the stored preview trajectory")
    async def trajectory_clear_preview():
        global _latest_plan
        async with _latest_plan_lock:
            _latest_plan = None
        return {"status": "ok"}

    return api


def _parse_pose_snapshot_response(detail: str) -> dict[str, Any]:
    parts = detail.split(",")
    if len(parts) < 7 or parts[0] != "CURRENT_POSE":
        raise ValueError(f"Malformed pose reply: {detail}")
    try:
        pos = [float(value) for value in parts[1:4]]
        orient = [float(value) for value in parts[4:7]]
        joints = [float(value) for value in parts[7:]]
    except ValueError as exc:
        raise ValueError("Invalid pose data from controller") from exc
    return {
        "position_m": {"x": pos[0], "y": pos[1], "z": pos[2]},
        "orientation_euler_deg": {"roll": orient[0], "pitch": orient[1], "yaw": orient[2]},
        "joints_deg": joints,
    }


def _parse_pose_response(detail: str) -> list[float]:
    payload = _parse_pose_snapshot_response(detail)
    joints = payload.get("joints_deg")
    if not isinstance(joints, list):
        raise ValueError("Pose reply did not include joint angles.")
    return [float(value) for value in joints]


def _coerce_waypoint_list(raw_points: Any) -> list[tuple[float, float, float]]:
    if not isinstance(raw_points, list):
        return []
    points: list[tuple[float, float, float]] = []
    for idx, entry in enumerate(raw_points):
        try:
            if isinstance(entry, dict):
                x = float(entry["x"])
                y = float(entry["y"])
                z = float(entry["z"])
            elif isinstance(entry, (list, tuple)) and len(entry) == 3:
                x, y, z = (float(entry[0]), float(entry[1]), float(entry[2]))
            else:
                raise ValueError("Waypoint must be an object with x/y/z or a 3-element list/tuple.")
        except (TypeError, ValueError, KeyError) as exc:
            raise HTTPException(
                status_code=400, detail=f"Invalid waypoint at index {idx}: {exc}"
            ) from exc
        points.append((x, y, z))
    return points


def _coerce_pose_waypoint_list(raw_waypoints: Any) -> list[dict[str, Any]]:
    if not isinstance(raw_waypoints, list):
        return []
    waypoints: list[dict[str, Any]] = []
    def _coerce_optional_positive_speed(raw_value: Any) -> float | None:
        if raw_value is None:
            return None
        value = float(raw_value)
        return value if np.isfinite(value) and value > 0 else None
    for idx, entry in enumerate(raw_waypoints):
        try:
            if not isinstance(entry, dict):
                raise ValueError("Waypoint must be an object.")
            x = float(entry["x"])
            y = float(entry["y"])
            z = float(entry["z"])
            orientation = entry.get("orientation_euler_deg", entry.get("orientationEulerDeg"))
            roll_raw = entry.get("rollDeg", entry.get("roll_deg"))
            pitch_raw = entry.get("pitchDeg", entry.get("pitch_deg"))
            yaw_raw = entry.get("yawDeg", entry.get("yaw_deg"))
            if isinstance(orientation, dict):
                roll_raw = orientation.get("roll", orientation.get("x", roll_raw))
                pitch_raw = orientation.get("pitch", orientation.get("y", pitch_raw))
                yaw_raw = orientation.get("yaw", orientation.get("z", yaw_raw))
            elif isinstance(orientation, (list, tuple)) and len(orientation) >= 3:
                roll_raw, pitch_raw, yaw_raw = orientation[:3]

            roll = float(roll_raw) if roll_raw is not None else None
            pitch = float(pitch_raw) if pitch_raw is not None else None
            yaw = float(yaw_raw) if yaw_raw is not None else None
            move_type_raw = str(entry.get("move_type", entry.get("moveType", "linear"))).strip().lower()
            move_type = move_type_raw if move_type_raw in {"linear", "joint", "home"} else "linear"
            linear_speed_mm_s = _coerce_optional_positive_speed(
                entry.get(
                    "linear_speed_mm_s",
                    entry.get(
                        "linearSpeedMmS",
                        entry.get("linear_speed_mm_per_s", entry.get("linearSpeedMmPerSec")),
                    ),
                )
            )
            linear_acceleration_mm_s2 = _coerce_optional_positive_speed(
                entry.get(
                    "linear_acceleration_mm_s2",
                    entry.get(
                        "linearAccelerationMmS2",
                        entry.get("linear_acceleration_mm_per_s2", entry.get("linearAccelerationMmPerSec2")),
                    ),
                )
            )
            rotation_speed_deg_s = _coerce_optional_positive_speed(
                entry.get(
                    "rotation_speed_deg_s",
                    entry.get(
                        "rotationSpeedDegS",
                        entry.get("rotation_speed_deg_per_s", entry.get("rotationSpeedDegPerSec")),
                    ),
                )
            )
            pause_after_s = _coerce_optional_positive_speed(
                entry.get(
                    "pause_after_s",
                    entry.get(
                        "pauseAfterS",
                        entry.get(
                            "pause_after_sec",
                            entry.get(
                                "pauseAfterSec",
                                entry.get(
                                    "pause_after_seconds",
                                    entry.get(
                                        "pauseAfterSeconds",
                                        entry.get(
                                            "pause_duration_s",
                                            entry.get("pauseDurationS"),
                                        ),
                                    ),
                                ),
                            ),
                        ),
                    ),
                )
            )
        except (TypeError, ValueError, KeyError) as exc:
            raise HTTPException(
                status_code=400, detail=f"Invalid pose waypoint at index {idx}: {exc}"
            ) from exc

        waypoint: dict[str, Any] = {"x": x, "y": y, "z": z}
        if roll is not None and pitch is not None and yaw is not None:
            waypoint["orientation_euler_deg"] = {
                "roll": roll,
                "pitch": pitch,
                "yaw": yaw,
            }
        waypoint["move_type"] = move_type
        if linear_speed_mm_s is not None:
            waypoint["linear_speed_mm_s"] = linear_speed_mm_s
        if linear_acceleration_mm_s2 is not None:
            waypoint["linear_acceleration_mm_s2"] = linear_acceleration_mm_s2
        if rotation_speed_deg_s is not None:
            waypoint["rotation_speed_deg_s"] = rotation_speed_deg_s
        if pause_after_s is not None:
            waypoint["pause_after_s"] = pause_after_s
        waypoints.append(waypoint)
    return waypoints


def _coerce_plan_sections(raw_sections: Any) -> list[dict[str, Any]]:
    if not isinstance(raw_sections, list):
        return []
    sections: list[dict[str, Any]] = []
    for idx, raw_section in enumerate(raw_sections):
        if not isinstance(raw_section, dict):
            raise HTTPException(status_code=400, detail=f"Invalid section at index {idx}: expected object")
        kind_raw = str(raw_section.get("kind", "weld")).strip().lower()
        kind = "transition" if kind_raw == "transition" else "weld"
        points = _coerce_waypoint_list(raw_section.get("points"))
        if len(points) < 2:
            continue
        section: dict[str, Any] = {
            "kind": kind,
            "points": points,
        }
        weld_type_raw = raw_section.get("weld_type", raw_section.get("weldType"))
        if weld_type_raw is not None and str(weld_type_raw).strip():
            section["weld_type"] = _normalize_weld_type(weld_type_raw)
        edge_id_raw = raw_section.get("edge_id", raw_section.get("edgeId"))
        if isinstance(edge_id_raw, str) and edge_id_raw.strip():
            section["edge_id"] = edge_id_raw.strip()
        sections.append(section)
    return sections


def _get_live_joint_angles_from_controller(timeout: float = 1.0) -> list[float]:
    ok, detail = _send_controller_command("GET_POSITION", timeout=timeout)
    if not ok:
        raise HTTPException(status_code=503, detail=detail)
    try:
        joints_deg = _parse_pose_response(detail)
    except ValueError as exc:
        raise HTTPException(status_code=502, detail=str(exc)) from exc
    if len(joints_deg) == 0:
        raise HTTPException(status_code=502, detail="Controller returned no joint angles in pose reply.")
    # `GET_POSITION` reports joints in degrees for UI consumption; the in-process
    # planner modules expect radians.
    return [float(np.deg2rad(value)) for value in joints_deg]


def _parse_joint_angles_response(detail: str) -> tuple[list[float], float | None]:
    parts = detail.split(",")
    if not parts or parts[0] != "JOINT_ANGLES":
        raise ValueError(f"Malformed joint reply: {detail}")
    try:
        angles = [float(value) for value in parts[1:]]
    except ValueError as exc:
        raise ValueError(f"Invalid joint data: {exc}") from exc
    arm = angles[:6]
    gripper = angles[6] if len(angles) > 6 else None
    return arm, gripper


def _parse_joint_state_response(detail: str) -> dict[str, Any]:
    prefix = "JOINT_STATE_JSON,"
    if not detail.startswith(prefix):
        raise ValueError(f"Malformed detailed joint reply: {detail}")
    try:
        payload = json.loads(detail[len(prefix) :])
    except json.JSONDecodeError as exc:
        raise ValueError(f"Invalid detailed joint payload: {exc}") from exc
    if not isinstance(payload, dict):
        raise ValueError("Detailed joint payload must be a JSON object.")

    def _coerce_float_list(value: Any) -> list[float]:
        if not isinstance(value, list):
            raise ValueError("Expected a list of floats in detailed joint payload.")
        return [float(item) if item is not None else None for item in value]  # type: ignore[list-item]

    normalized: dict[str, Any] = dict(payload)
    if "arm_deg" in normalized:
        normalized["arm_deg"] = _coerce_float_list(normalized["arm_deg"])
    if "arm_rad" in normalized:
        normalized["arm_rad"] = _coerce_float_list(normalized["arm_rad"])
    if "axis_counts" in normalized and isinstance(normalized["axis_counts"], list):
        normalized["axis_counts"] = [
            (int(item) if item is not None else None) for item in normalized["axis_counts"]
        ]
    if "axis_statusword" in normalized and isinstance(normalized["axis_statusword"], list):
        normalized["axis_statusword"] = [int(item) for item in normalized["axis_statusword"]]
    if "axis_error_code" in normalized and isinstance(normalized["axis_error_code"], list):
        normalized["axis_error_code"] = [int(item) for item in normalized["axis_error_code"]]
    if "axis_manufacturer_error_code" in normalized and isinstance(normalized["axis_manufacturer_error_code"], list):
        normalized["axis_manufacturer_error_code"] = [int(item) for item in normalized["axis_manufacturer_error_code"]]
    if "axis_torque_raw" in normalized and isinstance(normalized["axis_torque_raw"], list):
        normalized["axis_torque_raw"] = [int(item) for item in normalized["axis_torque_raw"]]
    if "axis_mode_display" in normalized and isinstance(normalized["axis_mode_display"], list):
        normalized["axis_mode_display"] = [int(item) for item in normalized["axis_mode_display"]]
    if "axis_ds402_state_code" in normalized and isinstance(normalized["axis_ds402_state_code"], list):
        normalized["axis_ds402_state_code"] = [int(item) for item in normalized["axis_ds402_state_code"]]
    if "axis_di_bits" in normalized and isinstance(normalized["axis_di_bits"], list):
        normalized["axis_di_bits"] = [int(item) for item in normalized["axis_di_bits"]]
    if "axis_fault_flags" in normalized and isinstance(normalized["axis_fault_flags"], list):
        normalized["axis_fault_flags"] = [int(item) for item in normalized["axis_fault_flags"]]
    if "axis_brake_state" in normalized and isinstance(normalized["axis_brake_state"], list):
        normalized["axis_brake_state"] = [int(item) for item in normalized["axis_brake_state"]]
    if "axis_to_joint" in normalized and isinstance(normalized["axis_to_joint"], list):
        normalized["axis_to_joint"] = [int(item) for item in normalized["axis_to_joint"]]
    if "gripper_deg" in normalized and normalized["gripper_deg"] is not None:
        normalized["gripper_deg"] = float(normalized["gripper_deg"])
    if "gripper_rad" in normalized and normalized["gripper_rad"] is not None:
        normalized["gripper_rad"] = float(normalized["gripper_rad"])
    return normalized


def _build_encoder_retention_capture_payload(*, phase: str, notes: str | None = None) -> dict[str, Any]:
    ok, joint_state_detail = _send_controller_command("GET_JOINT_STATE", timeout=1.0, expect_response=True)
    if not ok:
        raise HTTPException(status_code=503, detail=joint_state_detail)
    joint_state = _parse_joint_state_response(joint_state_detail)

    ok, motion_status_detail = _send_controller_command("GET_MOTION_STATUS", timeout=1.0, expect_response=True)
    if not ok:
        raise HTTPException(status_code=503, detail=motion_status_detail)
    motion_prefix = "MOTION_STATUS,"
    if not motion_status_detail.startswith(motion_prefix):
        raise HTTPException(status_code=502, detail=f"Malformed motion-status reply: {motion_status_detail}")
    motion_token = motion_status_detail[len(motion_prefix) :].strip()
    if not motion_token:
        raise HTTPException(status_code=502, detail="Motion-status reply did not include a payload.")
    try:
        motion_status = json.loads(base64.urlsafe_b64decode(motion_token.encode("ascii")).decode("utf-8"))
    except Exception as exc:
        raise HTTPException(status_code=502, detail=f"Invalid motion-status payload: {exc}") from exc
    if not isinstance(motion_status, dict):
        raise HTTPException(status_code=502, detail="Motion-status payload must decode to an object.")

    metrics_raw = _load_rtcore_metrics_raw()
    desired_config = runtime_config.load_runtime_config()
    desired = desired_config.get("desired", {}) if isinstance(desired_config, dict) else {}
    overrides = desired.get("overrides", {}) if isinstance(desired.get("overrides"), dict) else {}
    configured_drive_profile = str(overrides.get("drive_profile", "") or "").strip() or None
    drive_faults = None
    if isinstance(metrics_raw, dict):
        drive_faults = build_drive_fault_snapshot(
            metrics=metrics_raw,
            servo_backend="ethercat_rtcore",
            drive_profile=configured_drive_profile,
            configured_drive_profile=configured_drive_profile,
            axis_to_joint=joint_state.get("axis_to_joint") if isinstance(joint_state.get("axis_to_joint"), list) else None,
            socket_present=True,
        )

    captured_at = datetime.datetime.now(datetime.timezone.utc).isoformat(timespec="seconds")
    return {
        "captured_at": captured_at,
        "phase": phase,
        "notes": notes,
        "joint_state": joint_state,
        "motion_status": motion_status,
        "drive_faults": drive_faults,
        "rtcore_metrics": metrics_raw,
        "joint_zero_offsets_store": load_joint_zero_offsets_store(),
    }


def _parse_performance_state_response(detail: str) -> dict[str, Any]:
    prefix = "PERFORMANCE_STATE_JSON,"
    if not detail.startswith(prefix):
        raise ValueError(f"Malformed performance reply: {detail}")
    try:
        payload = json.loads(detail[len(prefix) :])
    except json.JSONDecodeError as exc:
        raise ValueError(f"Invalid performance payload: {exc}") from exc
    if not isinstance(payload, dict):
        raise ValueError("Performance payload must be a JSON object.")
    return payload


async def _plan_point(payload: dict) -> dict[str, Any]:
    try:
        x = float(payload.get("x"))
        y = float(payload.get("y"))
        z = float(payload.get("z"))
    except (TypeError, ValueError):
        raise HTTPException(status_code=400, detail="Fields 'x', 'y', 'z' are required floats")

    velocity = float(payload.get("velocity", 0.1))
    acceleration = float(payload.get("acceleration", 0.05))
    closed_loop = bool(payload.get("closed_loop", _DEFAULT_MOVE_LINE_CLOSED_LOOP))

    def _compute_plan() -> dict[str, Any]:
        start_joints = _get_live_joint_angles_from_controller(timeout=1.0)

        from ..arm_controller import trajectory_execution
        from .. import ik_solver

        target = np.array([x, y, z], dtype=float)
        path = trajectory_execution._plan_smooth_move(
            start_q=start_joints,
            target_pos=target,
            velocity=velocity,
            acceleration=acceleration,
            frequency=100,
            use_smoothing=True,
        )
        if not path:
            raise HTTPException(status_code=502, detail="Planner failed to produce a path")

        cartesian_points: list[list[float]] = []
        for joints in path:
            try:
                fk_point = ik_solver.get_fk(joints)
            except Exception:
                fk_point = None
            if fk_point is None:
                continue
            arr = np.asarray(fk_point, dtype=float)
            if arr.shape[0] >= 3:
                cartesian_points.append(arr[:3].tolist())

        return {
            "target": {"x": x, "y": y, "z": z},
            "velocity": velocity,
            "acceleration": acceleration,
            "closed_loop": closed_loop,
            "joints_rad": path,
            "cartesian_m": cartesian_points,
        }

    return await run_in_threadpool(_compute_plan)


app = create_app()


def main(argv: list[str] | None = None) -> None:
    parser = argparse.ArgumentParser(description="GradientOS HTTP API server")
    parser.add_argument(
        "--host",
        default=os.environ.get("GRADIENT_API_HOST", "0.0.0.0"),
        help="Interface to bind the HTTP API (env: GRADIENT_API_HOST)",
    )
    parser.add_argument(
        "--port",
        type=int,
        default=int(os.environ.get("GRADIENT_API_PORT", "4000")),
        help="Port for the HTTP API (env: GRADIENT_API_PORT)",
    )
    parser.add_argument(
        "--dev",
        action="store_true",
        help="Enable developer mode (uvicorn reload)",
    )
    args = parser.parse_args(argv)

    reload_env = os.environ.get("GRADIENT_API_RELOAD", "").lower()
    reload_enabled = args.dev or (reload_env in {"1", "true", "yes", "on"})

    import uvicorn

    uvicorn.run(
        "gradient_os.api.main:app",
        host=args.host,
        port=args.port,
        reload=reload_enabled,
    )


if __name__ == "__main__":
    main()


def _resolve_cors_origins() -> list[str]:
    raw = os.environ.get("GRADIENT_API_CORS", "")
    if not raw:
        return ["*"]
    origins = [item.strip() for item in raw.split(",")]
    return [origin for origin in origins if origin]
