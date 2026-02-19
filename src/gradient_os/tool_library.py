from __future__ import annotations

import datetime
import json
import os
import re
import shutil
from pathlib import Path
from typing import Any

TOOL_LIBRARY_ENV_VAR = "GRADIENT_TOOL_LIBRARY_PATH"
DEFAULT_TOOL_LIBRARY_DIR = os.path.join("tools", "library")
TOOL_DEF_FILENAME = "tool.json"
LIBRARY_META_FILENAME = "library.json"
LEGACY_LIBRARY_FILENAME = "tool_library.json"


def _utc_now_iso() -> str:
    return datetime.datetime.now(datetime.UTC).isoformat(timespec="seconds")


def _repo_root() -> Path:
    return Path(__file__).resolve().parents[2]


def get_tool_library_path() -> str:
    """
    Backward-compatible accessor.

    Returns the library *root directory* where tool folders live.
    """
    configured = os.environ.get(TOOL_LIBRARY_ENV_VAR, "").strip()
    if configured:
        resolved = os.path.abspath(configured)
        if resolved.lower().endswith(".json"):
            return os.path.dirname(resolved)
        return resolved
    return str((_repo_root() / DEFAULT_TOOL_LIBRARY_DIR).resolve())


def _library_meta_path(root: str) -> str:
    return os.path.join(root, LIBRARY_META_FILENAME)


def _legacy_library_path(root: str) -> str:
    return os.path.join(root, LEGACY_LIBRARY_FILENAME)


def _tool_folder_path(root: str, tool_id: str) -> str:
    return os.path.join(root, tool_id)


def _tool_definition_path(root: str, tool_id: str) -> str:
    return os.path.join(_tool_folder_path(root, tool_id), TOOL_DEF_FILENAME)


def _axis_triplet_mm(raw: Any, field_name: str) -> dict[str, float]:
    if not isinstance(raw, dict):
        raise ValueError(f"Field '{field_name}' must be an object.")
    out: dict[str, float] = {}
    for axis in ("x", "y", "z"):
        value = raw.get(axis)
        try:
            out[axis] = float(value)
        except (TypeError, ValueError):
            raise ValueError(f"Field '{field_name}.{axis}' must be numeric.")
    return out


def _axis_triplet_deg(raw: Any, field_name: str) -> dict[str, float]:
    if not isinstance(raw, dict):
        raise ValueError(f"Field '{field_name}' must be an object.")
    out: dict[str, float] = {}
    for axis in ("x", "y", "z"):
        value = raw.get(axis)
        try:
            out[axis] = float(value)
        except (TypeError, ValueError):
            raise ValueError(f"Field '{field_name}.{axis}' must be numeric.")
    return out


def _normalize_keywords(raw: Any) -> list[str]:
    if raw is None:
        return []
    if not isinstance(raw, list):
        raise ValueError("Field 'keywords' must be an array of strings.")
    out: list[str] = []
    for item in raw:
        token = str(item).strip().lower()
        if token:
            out.append(token)
    return sorted(set(out))


def _normalize_robot_ids(raw: Any) -> list[str]:
    if raw is None:
        return []
    if not isinstance(raw, list):
        raise ValueError("Field 'compatible_robot_ids' must be an array of strings.")
    out: list[str] = []
    for item in raw:
        token = str(item).strip()
        if token:
            out.append(token)
    return sorted(set(out))


def _normalize_tool_id(raw: Any) -> str:
    token = str(raw or "").strip().lower()
    if not token:
        raise ValueError("Field 'tool_id' is required.")
    token = re.sub(r"[^a-z0-9_-]+", "-", token)
    token = token.strip("-_")
    if not token:
        raise ValueError("Field 'tool_id' must contain letters or numbers.")
    return token[:80]


def _normalize_tool_type(raw: Any) -> str:
    token = str(raw or "").strip().lower()
    if not token:
        return "generic"
    token = re.sub(r"[^a-z0-9_-]+", "-", token).strip("-_")
    return token[:64] or "generic"


def _normalize_mesh(raw: Any, *, tool_id: str, tool_folder_path: str | None) -> dict[str, Any] | None:
    zero_mm = {"x": 0.0, "y": 0.0, "z": 0.0}
    zero_deg = {"x": 0.0, "y": 0.0, "z": 0.0}
    if raw is None:
        if not tool_folder_path:
            return None
        try:
            entries = sorted(os.listdir(tool_folder_path))
        except OSError:
            return None
        for entry in entries:
            lower = entry.lower()
            if lower.endswith(".stl") or lower.endswith(".glb") or lower.endswith(".gltf"):
                return {
                    "asset_path": f"{tool_id}/{entry}",
                    "scale": 1.0,
                    "position_mm": zero_mm,
                    "rotation_deg": zero_deg,
                }
        return None
    if isinstance(raw, str):
        raw = {"asset_path": raw}
    if not isinstance(raw, dict):
        raise ValueError("Field 'mesh' must be an object or string path.")
    asset_path = str(raw.get("asset_path", "")).strip()
    if not asset_path:
        return _normalize_mesh(None, tool_id=tool_id, tool_folder_path=tool_folder_path)
    if os.path.isabs(asset_path):
        raise ValueError("Field 'mesh.asset_path' must be a relative path.")
    normalized_asset = asset_path.replace("\\", "/").lstrip("./")
    if "/" not in normalized_asset and tool_id:
        normalized_asset = f"{tool_id}/{normalized_asset}"
    scale_raw = raw.get("scale", 1.0)
    try:
        scale = float(scale_raw)
    except (TypeError, ValueError):
        raise ValueError("Field 'mesh.scale' must be numeric.")
    scale = max(1e-5, scale)
    position_mm = _axis_triplet_mm(raw.get("position_mm", zero_mm), "mesh.position_mm")
    rotation_deg = _axis_triplet_deg(raw.get("rotation_deg", zero_deg), "mesh.rotation_deg")
    return {
        "asset_path": normalized_asset,
        "scale": scale,
        "position_mm": position_mm,
        "rotation_deg": rotation_deg,
    }


def _normalize_weld_meta(raw: Any) -> dict[str, Any]:
    if not isinstance(raw, dict):
        return {}
    out: dict[str, Any] = {}
    for key in ("torch_axis_angle_from_j6_deg", "work_angle_min_deg", "work_angle_max_deg"):
        if key in raw:
            try:
                out[key] = float(raw.get(key))
            except (TypeError, ValueError):
                raise ValueError(f"Field 'weld.{key}' must be numeric.")
    return out


def _normalize_tool(
    raw: Any,
    *,
    folder_tool_id: str | None = None,
    tool_folder_path: str | None = None,
) -> dict[str, Any]:
    if not isinstance(raw, dict):
        raise ValueError("Tool definition must be an object.")
    source_tool_id = raw.get("tool_id", folder_tool_id)
    tool_id = _normalize_tool_id(source_tool_id)
    if folder_tool_id:
        tool_id = _normalize_tool_id(folder_tool_id)
    display_name = str(raw.get("display_name") or raw.get("name") or "").strip()
    if not display_name:
        raise ValueError("Field 'display_name' is required.")
    description = str(raw.get("description", "")).strip()
    offset_raw = raw.get("offset", {})
    if not isinstance(offset_raw, dict):
        raise ValueError("Field 'offset' must be an object.")
    position_mm = _axis_triplet_mm(offset_raw.get("position_mm", {}), "offset.position_mm")
    rotation_deg = _axis_triplet_deg(offset_raw.get("rotation_deg", {}), "offset.rotation_deg")
    tool_type = _normalize_tool_type(raw.get("tool_type"))
    mesh = _normalize_mesh(raw.get("mesh"), tool_id=tool_id, tool_folder_path=tool_folder_path)
    return {
        "tool_id": tool_id,
        "display_name": display_name,
        "description": description,
        "tool_type": tool_type,
        "keywords": _normalize_keywords(raw.get("keywords")),
        "compatible_robot_ids": _normalize_robot_ids(raw.get("compatible_robot_ids")),
        "offset": {
            "position_mm": position_mm,
            "rotation_deg": rotation_deg,
        },
        "mesh": mesh,
        "weld": _normalize_weld_meta(raw.get("weld")),
    }


def _default_library() -> dict[str, Any]:
    return {
        "version": 1,
        "default_tool_id": "identity",
        "tools": [
            {
                "tool_id": "identity",
                "display_name": "Identity (No Tool Offset)",
                "description": "No additional tool transform.",
                "tool_type": "utility",
                "keywords": ["identity", "debug"],
                "compatible_robot_ids": [],
                "offset": {
                    "position_mm": {"x": 0.0, "y": 0.0, "z": 0.0},
                    "rotation_deg": {"x": 0.0, "y": 0.0, "z": 0.0},
                },
                "mesh": None,
                "weld": {},
            },
            {
                "tool_id": "tig-torch-65deg",
                "display_name": "TIG Torch 65deg",
                "description": (
                    "Template TIG torch definition from flange/J6 axis drawing. "
                    "Update XYZ mm offsets after final metrology."
                ),
                "tool_type": "tig_torch",
                "keywords": ["tig", "torch", "weld", "65deg"],
                "compatible_robot_ids": [],
                "offset": {
                    "position_mm": {"x": 0.0, "y": 0.0, "z": 0.0},
                    "rotation_deg": {"x": 0.0, "y": 65.0, "z": 0.0},
                },
                "mesh": None,
                "weld": {
                    "torch_axis_angle_from_j6_deg": 65.0,
                    "work_angle_min_deg": 0.0,
                    "work_angle_max_deg": 89.0,
                },
            },
        ],
        "meta": {"updated_at": _utc_now_iso(), "updated_by": "system-default"},
    }


def _normalize_library(raw: Any) -> dict[str, Any]:
    defaults = _default_library()
    if not isinstance(raw, dict):
        return defaults
    tools_raw = raw.get("tools")
    tools_in = tools_raw if isinstance(tools_raw, list) else defaults["tools"]
    tools_out: list[dict[str, Any]] = []
    seen: set[str] = set()
    for item in tools_in:
        normalized = _normalize_tool(item)
        tid = normalized["tool_id"]
        if tid in seen:
            raise ValueError(f"Duplicate tool_id '{tid}' in tool library.")
        seen.add(tid)
        tools_out.append(normalized)
    if "identity" not in seen:
        tools_out.insert(0, _normalize_tool(defaults["tools"][0]))
        seen.add("identity")
    default_tool_id = _normalize_tool_id(raw.get("default_tool_id", defaults["default_tool_id"]))
    if default_tool_id not in seen:
        default_tool_id = "identity"
    return {
        "version": 1,
        "default_tool_id": default_tool_id,
        "tools": tools_out,
        "meta": {
            "updated_at": str(raw.get("meta", {}).get("updated_at", defaults["meta"]["updated_at"])),
            "updated_by": str(raw.get("meta", {}).get("updated_by", defaults["meta"]["updated_by"])),
        },
    }


def load_tool_library() -> dict[str, Any]:
    root = get_tool_library_path()
    os.makedirs(root, exist_ok=True)

    meta_payload: dict[str, Any] = {}
    meta_path = _library_meta_path(root)
    legacy_path = _legacy_library_path(root)
    if os.path.exists(meta_path):
        try:
            with open(meta_path, "r", encoding="utf-8") as handle:
                raw_meta = json.load(handle)
            if isinstance(raw_meta, dict):
                meta_payload = raw_meta
        except Exception:
            meta_payload = {}
    elif os.path.exists(legacy_path):
        # Backward compatibility for monolithic tool_library.json.
        try:
            with open(legacy_path, "r", encoding="utf-8") as handle:
                raw_legacy = json.load(handle)
            if isinstance(raw_legacy, dict):
                meta_payload = {
                    "version": 1,
                    "default_tool_id": raw_legacy.get("default_tool_id", "identity"),
                    "meta": raw_legacy.get("meta", {}),
                }
                legacy_tools = raw_legacy.get("tools", [])
                if isinstance(legacy_tools, list):
                    for entry in legacy_tools:
                        try:
                            normalized = _normalize_tool(entry)
                        except Exception:
                            continue
                        tool_id = normalized["tool_id"]
                        tool_dir = _tool_folder_path(root, tool_id)
                        os.makedirs(tool_dir, exist_ok=True)
                        with open(
                            _tool_definition_path(root, tool_id),
                            "w",
                            encoding="utf-8",
                        ) as handle:
                            json.dump(normalized, handle, indent=2)
                            handle.write("\n")
        except Exception:
            meta_payload = {}

    tools: list[dict[str, Any]] = []
    for entry in sorted(os.listdir(root)):
        folder_path = os.path.join(root, entry)
        if not os.path.isdir(folder_path):
            continue
        if entry.startswith(".") or entry.startswith("_"):
            continue
        tool_json_path = _tool_definition_path(root, entry)
        if not os.path.isfile(tool_json_path):
            continue
        try:
            with open(tool_json_path, "r", encoding="utf-8") as handle:
                raw_tool = json.load(handle)
            normalized = _normalize_tool(
                raw_tool,
                folder_tool_id=entry,
                tool_folder_path=folder_path,
            )
            tools.append(normalized)
        except Exception:
            continue

    default_payload = _default_library()
    if not tools:
        tools = list(default_payload["tools"])
    elif not any(item.get("tool_id") == "identity" for item in tools):
        tools.insert(0, default_payload["tools"][0])

    lib = {
        "version": 1,
        "default_tool_id": str(meta_payload.get("default_tool_id", "identity")),
        "tools": tools,
        "meta": (
            meta_payload.get("meta")
            if isinstance(meta_payload.get("meta"), dict)
            else {"updated_at": _utc_now_iso(), "updated_by": "system-default"}
        ),
    }
    return _normalize_library(lib)


def save_tool_library(payload: dict[str, Any], *, actor: str = "unknown") -> dict[str, Any]:
    normalized = _normalize_library(payload)
    normalized_meta = {
        "updated_at": _utc_now_iso(),
        "updated_by": actor,
    }
    root = get_tool_library_path()
    os.makedirs(root, exist_ok=True)

    for tool in normalized.get("tools", []):
        if not isinstance(tool, dict):
            continue
        tool_id = str(tool.get("tool_id", "")).strip()
        if not tool_id:
            continue
        tool_dir = _tool_folder_path(root, tool_id)
        os.makedirs(tool_dir, exist_ok=True)
        tool_json_path = _tool_definition_path(root, tool_id)
        with open(tool_json_path, "w", encoding="utf-8") as handle:
            json.dump(tool, handle, indent=2)
            handle.write("\n")

    meta_payload = {
        "version": 1,
        "default_tool_id": normalized.get("default_tool_id", "identity"),
        "meta": normalized_meta,
    }
    meta_path = _library_meta_path(root)
    tmp_path = f"{meta_path}.tmp"
    with open(tmp_path, "w", encoding="utf-8") as handle:
        json.dump(meta_payload, handle, indent=2)
        handle.write("\n")
    os.replace(tmp_path, meta_path)

    normalized["meta"] = normalized_meta
    return normalized


def _is_tool_compatible(tool: dict[str, Any], robot_id: str | None) -> bool:
    compat = tool.get("compatible_robot_ids")
    if not isinstance(compat, list) or len(compat) == 0:
        return True
    if not robot_id:
        return True
    return str(robot_id).strip() in {str(item).strip() for item in compat}


def list_tools(
    *,
    robot_id: str | None = None,
    tool_type: str | None = None,
    query: str | None = None,
) -> list[dict[str, Any]]:
    library = load_tool_library()
    tools = library.get("tools", [])
    if not isinstance(tools, list):
        return []
    desired_type = _normalize_tool_type(tool_type) if tool_type else None
    q = str(query or "").strip().lower()
    out: list[dict[str, Any]] = []
    for item in tools:
        if not isinstance(item, dict):
            continue
        if not _is_tool_compatible(item, robot_id):
            continue
        if desired_type and _normalize_tool_type(item.get("tool_type")) != desired_type:
            continue
        if q:
            haystack = " ".join(
                [
                    str(item.get("tool_id", "")),
                    str(item.get("display_name", "")),
                    str(item.get("description", "")),
                    " ".join(str(v) for v in item.get("keywords", [])),
                    str(item.get("tool_type", "")),
                ]
            ).lower()
            if q not in haystack:
                continue
        out.append(item)
    return out


def get_tool(tool_id: str) -> dict[str, Any]:
    target = _normalize_tool_id(tool_id)
    for item in load_tool_library().get("tools", []):
        if isinstance(item, dict) and item.get("tool_id") == target:
            return item
    raise ValueError(f"Unknown tool_id '{target}'.")


def upsert_tool(payload: dict[str, Any], *, actor: str = "unknown") -> dict[str, Any]:
    root = get_tool_library_path()
    os.makedirs(root, exist_ok=True)
    normalized = _normalize_tool(payload)
    tool_id = normalized["tool_id"]
    tool_dir = _tool_folder_path(root, tool_id)
    os.makedirs(tool_dir, exist_ok=True)
    tool_json_path = _tool_definition_path(root, tool_id)
    with open(tool_json_path, "w", encoding="utf-8") as handle:
        json.dump(normalized, handle, indent=2)
        handle.write("\n")
    library = load_tool_library()
    if tool_id == "identity":
        library["default_tool_id"] = "identity"
    return save_tool_library(library, actor=actor)


def delete_tool(tool_id: str, *, actor: str = "unknown") -> dict[str, Any]:
    target = _normalize_tool_id(tool_id)
    if target == "identity":
        raise ValueError("Tool 'identity' cannot be deleted.")
    root = get_tool_library_path()
    tool_dir = _tool_folder_path(root, target)
    if not os.path.isdir(tool_dir):
        raise ValueError(f"Unknown tool_id '{target}'.")
    shutil.rmtree(tool_dir)
    library = load_tool_library()
    if library.get("default_tool_id") == target:
        library["default_tool_id"] = "identity"
    return save_tool_library(library, actor=actor)


def set_default_tool(tool_id: str, *, actor: str = "unknown") -> dict[str, Any]:
    target = _normalize_tool_id(tool_id)
    library = load_tool_library()
    known = {
        item.get("tool_id")
        for item in library.get("tools", [])
        if isinstance(item, dict)
    }
    if target not in known:
        raise ValueError(f"Unknown tool_id '{target}'.")
    library["default_tool_id"] = target
    return save_tool_library(library, actor=actor)


def resolve_active_tool(
    *,
    robot_id: str | None,
    requested_tool_id: str | None,
) -> dict[str, Any]:
    tools = list_tools(robot_id=robot_id)
    by_id = {str(item.get("tool_id")): item for item in tools if isinstance(item, dict)}
    library = load_tool_library()
    requested = None
    if requested_tool_id:
        try:
            requested = _normalize_tool_id(requested_tool_id)
        except ValueError:
            requested = None
    if requested and requested in by_id:
        source = "desired"
        selected = by_id[requested]
    else:
        default_tool_id = str(library.get("default_tool_id", "identity"))
        if default_tool_id in by_id:
            source = "library_default"
            selected = by_id[default_tool_id]
        elif "identity" in by_id:
            source = "fallback_identity"
            selected = by_id["identity"]
        else:
            source = "fallback_inline_identity"
            selected = _normalize_tool(
                {
                    "tool_id": "identity",
                    "display_name": "Identity (No Tool Offset)",
                    "tool_type": "utility",
                    "offset": {
                        "position_mm": {"x": 0.0, "y": 0.0, "z": 0.0},
                        "rotation_deg": {"x": 0.0, "y": 0.0, "z": 0.0},
                    },
                }
            )
    payload = dict(selected)
    payload["selection_source"] = source
    return payload


def to_runtime_offset(tool: dict[str, Any]) -> dict[str, Any]:
    offset = tool.get("offset", {}) if isinstance(tool, dict) else {}
    pos_mm = offset.get("position_mm", {}) if isinstance(offset, dict) else {}
    rot_deg = offset.get("rotation_deg", {}) if isinstance(offset, dict) else {}
    return {
        "position_m": {
            "x": float(pos_mm.get("x", 0.0)) / 1000.0,
            "y": float(pos_mm.get("y", 0.0)) / 1000.0,
            "z": float(pos_mm.get("z", 0.0)) / 1000.0,
        },
        "rotation_deg": {
            "x": float(rot_deg.get("x", 0.0)),
            "y": float(rot_deg.get("y", 0.0)),
            "z": float(rot_deg.get("z", 0.0)),
        },
    }

