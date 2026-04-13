from __future__ import annotations

import datetime
import json
import os
from typing import Any, Sequence

ANCHORS_ENV_VAR = "GRADIENT_ABSOLUTE_ENCODER_ANCHORS_PATH"
DEFAULT_ANCHORS_BASENAME = ".gradient_absolute_encoder_anchors.json"


def _utc_now_iso() -> str:
    return datetime.datetime.now(datetime.UTC).isoformat(timespec="seconds")


def get_absolute_encoder_anchors_path() -> str:
    configured = os.environ.get(ANCHORS_ENV_VAR, "").strip()
    if configured:
        return os.path.abspath(configured)
    repo_root = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
    return os.path.join(repo_root, DEFAULT_ANCHORS_BASENAME)


def _empty_store() -> dict[str, Any]:
    return {
        "version": 1,
        "robots": {},
    }


def _normalize_anchor_entry(raw: Any) -> dict[str, Any] | None:
    if not isinstance(raw, dict):
        return None
    try:
        home_anchor_rad = float(raw.get("home_anchor_rad"))
    except Exception:
        return None
    axis_indices_raw = raw.get("axis_indices")
    axis_indices: list[int] = []
    if isinstance(axis_indices_raw, list):
        for value in axis_indices_raw:
            try:
                axis_indices.append(int(value))
            except Exception:
                continue
    source_raw = raw.get("source")
    updated_at_raw = raw.get("updated_at")
    updated_by_raw = raw.get("updated_by")
    return {
        "home_anchor_rad": float(home_anchor_rad),
        "source": str(source_raw).strip() if source_raw is not None else None,
        "axis_indices": axis_indices,
        "updated_at": str(updated_at_raw).strip() if updated_at_raw is not None else None,
        "updated_by": str(updated_by_raw).strip() if updated_by_raw is not None else None,
    }


def _normalize_anchor_entries(raw: Any, *, num_joints: int) -> list[dict[str, Any] | None]:
    entries: list[dict[str, Any] | None] = [None] * max(0, int(num_joints))
    if not isinstance(raw, list):
        return entries
    for idx, value in enumerate(raw[: len(entries)]):
        entries[idx] = _normalize_anchor_entry(value)
    return entries


def load_absolute_encoder_anchors_store() -> dict[str, Any]:
    path = get_absolute_encoder_anchors_path()
    if not os.path.exists(path):
        return _empty_store()
    try:
        with open(path, "r", encoding="utf-8") as handle:
            payload = json.load(handle)
    except Exception:
        return _empty_store()
    if not isinstance(payload, dict):
        return _empty_store()
    robots = payload.get("robots")
    return {
        "version": 1,
        "robots": robots if isinstance(robots, dict) else {},
    }


def load_absolute_encoder_anchors(
    robot_id: str,
    *,
    num_joints: int,
) -> list[dict[str, Any] | None]:
    store = load_absolute_encoder_anchors_store()
    robots = store.get("robots", {})
    entry = robots.get(str(robot_id), {}) if isinstance(robots, dict) else {}
    raw_entries = entry.get("logical_joint_absolute_home_anchors") if isinstance(entry, dict) else None
    return _normalize_anchor_entries(raw_entries, num_joints=num_joints)


def save_absolute_encoder_anchors(
    robot_id: str,
    anchors: Sequence[dict[str, Any] | None],
    *,
    actor: str = "unknown",
) -> dict[str, Any]:
    robot_key = str(robot_id or "").strip() or "unknown"
    store = load_absolute_encoder_anchors_store()
    robots = store.get("robots")
    if not isinstance(robots, dict):
        robots = {}
    normalized_entries: list[dict[str, Any] | None] = []
    for raw_entry in list(anchors):
        entry = _normalize_anchor_entry(raw_entry)
        if entry is None:
            normalized_entries.append(None)
            continue
        normalized_entries.append(
            {
                "home_anchor_rad": float(entry["home_anchor_rad"]),
                "source": entry.get("source"),
                "axis_indices": [int(value) for value in entry.get("axis_indices", [])],
                "updated_at": entry.get("updated_at") or _utc_now_iso(),
                "updated_by": entry.get("updated_by") or str(actor or "unknown"),
            }
        )
    robots[robot_key] = {
        "logical_joint_absolute_home_anchors": normalized_entries,
        "updated_at": _utc_now_iso(),
        "updated_by": str(actor or "unknown"),
    }
    store["robots"] = robots

    path = get_absolute_encoder_anchors_path()
    dirpath = os.path.dirname(path)
    if dirpath:
        os.makedirs(dirpath, exist_ok=True)
    temp_path = f"{path}.tmp"
    with open(temp_path, "w", encoding="utf-8") as handle:
        json.dump(store, handle, indent=2)
        handle.write("\n")
    os.replace(temp_path, path)
    return robots[robot_key]


def save_absolute_encoder_anchor(
    robot_id: str,
    *,
    num_joints: int,
    logical_joint_index: int,
    home_anchor_rad: float,
    source: str,
    axis_indices: Sequence[int] | None = None,
    actor: str = "unknown",
) -> dict[str, Any]:
    joint_i = int(logical_joint_index)
    if joint_i < 0 or joint_i >= int(num_joints):
        raise IndexError(
            f"Logical joint index {joint_i} is out of range for {int(num_joints)} joints."
        )
    anchors = load_absolute_encoder_anchors(robot_id, num_joints=num_joints)
    anchors[joint_i] = {
        "home_anchor_rad": float(home_anchor_rad),
        "source": str(source).strip() or None,
        "axis_indices": [int(value) for value in list(axis_indices or [])],
        "updated_at": _utc_now_iso(),
        "updated_by": str(actor or "unknown"),
    }
    saved = save_absolute_encoder_anchors(robot_id, anchors, actor=actor)
    return (
        saved.get("logical_joint_absolute_home_anchors", [])[joint_i]
        if isinstance(saved, dict)
        else {}
    )
