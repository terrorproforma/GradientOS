from __future__ import annotations

import datetime
import json
import os
from typing import Any, Sequence

ANCHORS_ENV_VAR = "GRADIENT_ABSOLUTE_ENCODER_ANCHORS_PATH"
DEFAULT_ANCHORS_BASENAME = ".gradient_absolute_encoder_anchors.json"


class AnchorStoreError(RuntimeError):
    """Raised when the authoritative absolute-home anchor store is invalid."""


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


def _normalize_last_seen(raw: Any) -> dict[str, Any] | None:
    """Defensively normalize the optional ``last_seen`` sidecar.

    Missing or malformed sidecars return ``None`` so older anchor files
    keep working untouched. The sidecar is diagnostic only: the store
    contract for ``home_anchor_rad`` stays identical.
    """
    if not isinstance(raw, dict):
        return None
    try:
        absolute_counts = int(raw.get("absolute_counts"))
    except Exception:
        return None
    entry: dict[str, Any] = {"absolute_counts": int(absolute_counts)}
    if "reference_counts" in raw:
        try:
            entry["reference_counts"] = int(raw.get("reference_counts"))
        except Exception:
            pass
    observed_at_raw = raw.get("observed_at")
    if observed_at_raw is not None:
        token = str(observed_at_raw).strip()
        if token:
            entry["observed_at"] = token
    if "observed_at_monotonic_ns" in raw:
        try:
            entry["observed_at_monotonic_ns"] = int(raw.get("observed_at_monotonic_ns"))
        except Exception:
            pass
    observed_by_raw = raw.get("observed_by")
    if observed_by_raw is not None:
        token = str(observed_by_raw).strip()
        if token:
            entry["observed_by"] = token
    return entry


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
    last_seen = _normalize_last_seen(raw.get("last_seen"))
    return {
        "home_anchor_rad": float(home_anchor_rad),
        "source": str(source_raw).strip() if source_raw is not None else None,
        "axis_indices": axis_indices,
        "updated_at": str(updated_at_raw).strip() if updated_at_raw is not None else None,
        "updated_by": str(updated_by_raw).strip() if updated_by_raw is not None else None,
        "last_seen": last_seen,
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
    except json.JSONDecodeError as exc:
        raise AnchorStoreError(
            f"Absolute encoder anchor store is malformed JSON: {path}: {exc}. "
            "Refusing to treat this as an empty anchor store."
        ) from exc
    except OSError as exc:
        raise AnchorStoreError(
            f"Absolute encoder anchor store could not be read: {path}: {exc}."
        ) from exc
    if not isinstance(payload, dict):
        raise AnchorStoreError(
            f"Absolute encoder anchor store must be a JSON object: {path}."
        )
    robots = payload.get("robots")
    if robots is not None and not isinstance(robots, dict):
        raise AnchorStoreError(
            f"Absolute encoder anchor store field 'robots' must be an object: {path}."
        )
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
        serialized_entry: dict[str, Any] = {
            "home_anchor_rad": float(entry["home_anchor_rad"]),
            "source": entry.get("source"),
            "axis_indices": [int(value) for value in entry.get("axis_indices", [])],
            "updated_at": entry.get("updated_at") or _utc_now_iso(),
            "updated_by": entry.get("updated_by") or str(actor or "unknown"),
        }
        last_seen = entry.get("last_seen")
        if isinstance(last_seen, dict):
            serialized_entry["last_seen"] = dict(last_seen)
        normalized_entries.append(serialized_entry)
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
    preserve_last_seen: bool = True,
) -> dict[str, Any]:
    joint_i = int(logical_joint_index)
    if joint_i < 0 or joint_i >= int(num_joints):
        raise IndexError(
            f"Logical joint index {joint_i} is out of range for {int(num_joints)} joints."
        )
    anchors = load_absolute_encoder_anchors(robot_id, num_joints=num_joints)
    existing_last_seen: dict[str, Any] | None = None
    if preserve_last_seen:
        existing_entry = anchors[joint_i] if isinstance(anchors[joint_i], dict) else None
        if isinstance(existing_entry, dict):
            maybe_last_seen = existing_entry.get("last_seen")
            if isinstance(maybe_last_seen, dict):
                existing_last_seen = dict(maybe_last_seen)
    new_entry: dict[str, Any] = {
        "home_anchor_rad": float(home_anchor_rad),
        "source": str(source).strip() or None,
        "axis_indices": [int(value) for value in list(axis_indices or [])],
        "updated_at": _utc_now_iso(),
        "updated_by": str(actor or "unknown"),
    }
    if existing_last_seen is not None:
        new_entry["last_seen"] = existing_last_seen
    anchors[joint_i] = new_entry
    saved = save_absolute_encoder_anchors(robot_id, anchors, actor=actor)
    return (
        saved.get("logical_joint_absolute_home_anchors", [])[joint_i]
        if isinstance(saved, dict)
        else {}
    )


def invalidate_absolute_encoder_anchors(
    robot_id: str,
    *,
    num_joints: int,
    logical_joint_indices: Sequence[int],
    actor: str = "unknown",
) -> dict[str, Any]:
    """Clear the persisted home-anchor entries for the given logical joints.

    Use this when an out-of-band event has made the multi-turn absolute
    position that anchors the joint's canonical frame unreliable - most
    notably an F31.10 ("Encoder data reset") SDO write on A6-EC drives,
    which zeroes the encoder's multi-turn register. After invalidation
    the affected joints show up as anchor-less on the next backend
    startup, which in turn forces a native re-home before motion is
    trusted.

    Non-destructive for any entry whose logical index is not in the
    provided list; those entries round-trip unchanged. Silently skips
    out-of-range indices so callers can pass a "best effort" list
    derived from the startup probe without pre-validating it.

    Returns the stored robot entry (same shape as
    ``save_absolute_encoder_anchors``).
    """
    anchors = load_absolute_encoder_anchors(robot_id, num_joints=num_joints)
    indices: set[int] = set()
    for value in logical_joint_indices:
        try:
            idx = int(value)
        except Exception:
            continue
        if 0 <= idx < len(anchors):
            indices.add(idx)
    if not indices:
        return save_absolute_encoder_anchors(robot_id, anchors, actor=actor)
    for idx in indices:
        anchors[idx] = None
    return save_absolute_encoder_anchors(robot_id, anchors, actor=actor)


def save_last_seen_absolute_counts(
    robot_id: str,
    *,
    num_joints: int,
    logical_joint_index: int,
    absolute_counts: int,
    reference_counts: int | None = None,
    observed_at: str | None = None,
    observed_at_monotonic_ns: int | None = None,
    actor: str = "unknown",
) -> dict[str, Any] | None:
    """Update only the optional ``last_seen`` sidecar on the anchor entry
    for the target joint. No-op when the joint has no recorded anchor -
    we never materialize a fake anchor just to hold a last-seen reading.

    Returns the updated per-joint anchor entry (including the new
    ``last_seen`` dict) on success, or ``None`` when there is no anchor
    for the joint yet.
    """
    joint_i = int(logical_joint_index)
    if joint_i < 0 or joint_i >= int(num_joints):
        raise IndexError(
            f"Logical joint index {joint_i} is out of range for {int(num_joints)} joints."
        )
    anchors = load_absolute_encoder_anchors(robot_id, num_joints=num_joints)
    existing = anchors[joint_i] if isinstance(anchors[joint_i], dict) else None
    if not isinstance(existing, dict):
        return None
    updated_entry = dict(existing)
    sidecar: dict[str, Any] = {"absolute_counts": int(absolute_counts)}
    if reference_counts is not None:
        try:
            sidecar["reference_counts"] = int(reference_counts)
        except Exception:
            pass
    sidecar["observed_at"] = str(observed_at).strip() if observed_at else _utc_now_iso()
    if observed_at_monotonic_ns is not None:
        try:
            sidecar["observed_at_monotonic_ns"] = int(observed_at_monotonic_ns)
        except Exception:
            pass
    sidecar["observed_by"] = str(actor or "unknown")
    updated_entry["last_seen"] = sidecar
    anchors[joint_i] = updated_entry
    saved = save_absolute_encoder_anchors(robot_id, anchors, actor=actor)
    return (
        saved.get("logical_joint_absolute_home_anchors", [])[joint_i]
        if isinstance(saved, dict)
        else None
    )
