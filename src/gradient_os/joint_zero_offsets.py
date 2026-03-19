from __future__ import annotations

import datetime
import json
import os
from typing import Any, Sequence

ZERO_OFFSETS_ENV_VAR = "GRADIENT_JOINT_ZERO_OFFSETS_PATH"
DEFAULT_ZERO_OFFSETS_BASENAME = ".gradient_joint_zero_offsets.json"


def _utc_now_iso() -> str:
    return datetime.datetime.now(datetime.UTC).isoformat(timespec="seconds")


def get_joint_zero_offsets_path() -> str:
    configured = os.environ.get(ZERO_OFFSETS_ENV_VAR, "").strip()
    if configured:
        return os.path.abspath(configured)
    repo_root = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
    return os.path.join(repo_root, DEFAULT_ZERO_OFFSETS_BASENAME)


def _empty_store() -> dict[str, Any]:
    return {
        "version": 1,
        "robots": {},
    }


def _normalize_offsets(
    raw: Any,
    *,
    num_joints: int,
    defaults: Sequence[float] | None = None,
) -> list[float]:
    offsets = [0.0] * max(0, int(num_joints))
    if defaults is not None:
        for idx, value in enumerate(list(defaults)[: len(offsets)]):
            try:
                offsets[idx] = float(value)
            except Exception:
                offsets[idx] = 0.0
    if not isinstance(raw, list):
        return offsets
    for idx, value in enumerate(raw[: len(offsets)]):
        try:
            offsets[idx] = float(value)
        except Exception:
            continue
    return offsets


def load_joint_zero_offsets_store() -> dict[str, Any]:
    path = get_joint_zero_offsets_path()
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


def load_joint_zero_offsets(
    robot_id: str,
    *,
    num_joints: int,
    defaults: Sequence[float] | None = None,
) -> list[float]:
    store = load_joint_zero_offsets_store()
    robots = store.get("robots", {})
    entry = robots.get(str(robot_id), {}) if isinstance(robots, dict) else {}
    raw_offsets = entry.get("logical_joint_master_offsets_rad") if isinstance(entry, dict) else None
    return _normalize_offsets(raw_offsets, num_joints=num_joints, defaults=defaults)


def save_joint_zero_offsets(
    robot_id: str,
    offsets_rad: Sequence[float],
    *,
    actor: str = "unknown",
) -> dict[str, Any]:
    robot_key = str(robot_id or "").strip() or "unknown"
    store = load_joint_zero_offsets_store()
    robots = store.get("robots")
    if not isinstance(robots, dict):
        robots = {}
    normalized_offsets = [float(value) for value in list(offsets_rad)]
    robots[robot_key] = {
        "logical_joint_master_offsets_rad": normalized_offsets,
        "updated_at": _utc_now_iso(),
        "updated_by": str(actor or "unknown"),
    }
    store["robots"] = robots

    path = get_joint_zero_offsets_path()
    dirpath = os.path.dirname(path)
    if dirpath:
        os.makedirs(dirpath, exist_ok=True)
    temp_path = f"{path}.tmp"
    with open(temp_path, "w", encoding="utf-8") as handle:
        json.dump(store, handle, indent=2)
        handle.write("\n")
    os.replace(temp_path, path)
    return robots[robot_key]
