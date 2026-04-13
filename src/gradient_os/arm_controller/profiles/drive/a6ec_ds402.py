from __future__ import annotations

import json
import os
from collections.abc import Mapping
from typing import Any

from ...ethercat_drive_catalog import build_default_startup_entries, get_ethercat_drive_profile
from . import cia402

PROFILE_ID = "a6ec_ds402"
PROFILE_LABEL = "A6-EC over CiA402"
FAULT_REFERENCE_LABEL = "A6-EC CiA402 0x603F / 0x203F fault codes"
FAULT_REFERENCE_RELATIVE_PATH = os.path.join("docs", "resources", "a6ec_manual_codes.json")
STARTUP_SETTING_KEY = "a6ec_encoder_position_tracking_mode"
STARTUP_SETTING_LABEL = "A6-EC encoder position tracking mode"
STARTUP_SETTING_OBJECT = "C00.07 / 0x2000:08"
STARTUP_SETTING_ENV_VAR = "GRADIENT_RT_DRIVE_STARTUP_SDO_CONFIG"
NATIVE_HOME_CONFIG_ENV_VAR = "GRADIENT_RT_NATIVE_HOME_CONFIG"
ABSOLUTE_FEEDBACK_CONFIG_ENV_VAR = "GRADIENT_RT_ABSOLUTE_FEEDBACK_CONFIG"
MOTION_FEEDBACK_CONFIG = {
    "profile_id": PROFILE_ID,
    "feedback_counts_wrap": True,
}
STARTUP_SETTING_VALUE_LABELS = {
    0: "Incremental encoder mode",
    1: "Absolute position linear mode",
    2: "Absolute position single-turn mode",
    3: "Absolute-position mode 3 (verify against vendor manual)",
    4: "Absolute position rotation mode",
    5: "Absolute-position mode 5 (verify against vendor manual)",
}
NATIVE_HOME_TRUTH_SOURCE = {
    "kind": "sdo",
    "index": 0x607C,
    "subindex": 0x00,
    "type": "i32",
}
NATIVE_HOME_CONFIG = {
    "profile_id": PROFILE_ID,
    "steady_state_mode": 8,
    "commissioning_mode": 6,
    "truth_source": NATIVE_HOME_TRUTH_SOURCE,
    "transaction": [
        {"op": "set_mode", "value": 6},
        {"op": "write_sdo", "index": 0x60E6, "subindex": 0x00, "type": "u8", "value": 0},
        {"op": "write_sdo", "index": 0x607C, "subindex": 0x00, "type": "i32", "value": 0},
        {"op": "write_sdo", "index": 0x6098, "subindex": 0x00, "type": "i8", "value": 35},
        {"op": "controlword_sequence", "values": [6, 7, 15]},
        {"op": "wait_statusword", "all_set_mask": 0x0227, "all_clear_mask": 0x2048},
        {"op": "controlword_sequence", "values": [31]},
        {"op": "wait_statusword", "all_set_mask": 0x9000, "all_clear_mask": 0x2000},
        {"op": "refresh_truth"},
        {"op": "restore_mode", "value": 8},
        {"op": "release_service_override"},
        {"op": "write_sdo", "index": 0x2031, "subindex": 0x11, "type": "u16", "value": 1},
        {"op": "wait_sdo", "index": 0x2031, "subindex": 0x11, "type": "u16", "value": 0},
        {"op": "write_sdo", "index": 0x2031, "subindex": 0x11, "type": "u16", "value": 2},
        {"op": "wait_sdo", "index": 0x2031, "subindex": 0x11, "type": "u16", "value": 0},
    ],
}
ENCODER_DATA_RESET_OPERATION = {
    "operation_key": "encoder_data_reset",
    "operation_label": "A6-EC encoder data reset",
    "parameter": "F31.10",
    "object_label": "Operation reset Parameter / Encoder data reset",
    "index": 0x2031,
    "subindex": 0x11,
    "type": "u16",
    "value": 4,
    "requires_power_cycle": True,
    "requires_rehome": True,
}

ABSOLUTE_FEEDBACK_FIELDS = [
    {
        "key": "absolute_position_reference",
        "label": "Absolute position feedback (reference unit)",
        "object": "U40.16 / 0x2040:17",
        "index": 0x2040,
        "subindex": 0x17,
        "type": "i32",
    },
    {
        "key": "encoder_single_turn_data",
        "label": "Encoder single-turn data",
        "object": "U40.1C / 0x2040:1D",
        "index": 0x2040,
        "subindex": 0x1D,
        "type": "i32",
    },
    {
        "key": "encoder_multi_turn_position",
        "label": "Encoder multi-turn position data",
        "object": "U40.1E / 0x2040:1F",
        "index": 0x2040,
        "subindex": 0x1F,
        "type": "u16",
    },
    {
        "key": "encoder_multi_turn_low",
        "label": "Encoder multi-turn data low 32 bits",
        "object": "U40.20 / 0x2040:21",
        "index": 0x2040,
        "subindex": 0x21,
        "type": "i32",
    },
    {
        "key": "encoder_multi_turn_high",
        "label": "Encoder multi-turn data high 32 bits",
        "object": "U40.22 / 0x2040:23",
        "index": 0x2040,
        "subindex": 0x23,
        "type": "i32",
    },
    {
        "key": "rotation_mode_position_reference",
        "label": "Rotation-mode position feedback (reference unit)",
        "object": "U40.28 / 0x2040:29",
        "index": 0x2040,
        "subindex": 0x29,
        "type": "i32",
    },
    {
        "key": "rotation_mode_encoder_low",
        "label": "Rotation-mode encoder feedback low 32 bits",
        "object": "U40.2A / 0x2040:2B",
        "index": 0x2040,
        "subindex": 0x2B,
        "type": "i32",
    },
    {
        "key": "rotation_mode_encoder_high",
        "label": "Rotation-mode encoder feedback high 32 bits",
        "object": "U40.2C / 0x2040:2D",
        "index": 0x2040,
        "subindex": 0x2D,
        "type": "i32",
    },
]

ABSOLUTE_FEEDBACK_SOURCES = [
    {
        "key": "encoder_multi_turn_counts",
        "label": "Combined encoder multi-turn counts",
        "kind": "signed_i64_pair",
        "low_key": "encoder_multi_turn_low",
        "high_key": "encoder_multi_turn_high",
    },
    {
        "key": "rotation_mode_encoder_counts",
        "label": "Combined rotation-mode encoder counts",
        "kind": "signed_i64_pair",
        "low_key": "rotation_mode_encoder_low",
        "high_key": "rotation_mode_encoder_high",
    },
]

ABSOLUTE_FEEDBACK_DISPLAY_SOURCE_KEYS = [
    "encoder_multi_turn_counts",
    "rotation_mode_encoder_counts",
]

_DRIVE_FAULT_CODEBOOK_CACHE: dict[str, Any] | None = None


def _repo_root() -> str:
    return os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", "..", "..", ".."))


def _drive_fault_codebook_path() -> str:
    return os.path.join(_repo_root(), FAULT_REFERENCE_RELATIVE_PATH)


def _load_drive_fault_codebook() -> dict[str, Any]:
    global _DRIVE_FAULT_CODEBOOK_CACHE
    if _DRIVE_FAULT_CODEBOOK_CACHE is not None:
        return _DRIVE_FAULT_CODEBOOK_CACHE
    path = _drive_fault_codebook_path()
    try:
        with open(path, "r", encoding="utf-8") as handle:
            payload = json.load(handle)
            if isinstance(payload, dict):
                _DRIVE_FAULT_CODEBOOK_CACHE = payload
                return _DRIVE_FAULT_CODEBOOK_CACHE
    except Exception:
        pass
    _DRIVE_FAULT_CODEBOOK_CACHE = {}
    return _DRIVE_FAULT_CODEBOOK_CACHE


def _coerce_code_to_u32(raw: Any) -> int | None:
    if raw is None:
        return None
    token = str(raw).strip()
    if not token:
        return None
    try:
        return int(token, 0) & 0xFFFFFFFF
    except Exception:
        return None


def _match_entry_by_numeric_code(table_name: str, field_name: str, code_u32: int) -> dict[str, Any] | None:
    book = _load_drive_fault_codebook()
    table = book.get("tables", {}).get(table_name, {})
    if not isinstance(table, dict):
        return None
    for candidate in table.values():
        if not isinstance(candidate, dict):
            continue
        if _coerce_code_to_u32(candidate.get(field_name)) == code_u32:
            return candidate
    return None


def decode_statusword(statusword: int) -> dict[str, Any]:
    payload = cia402.decode_statusword(statusword)
    payload["profile_id"] = PROFILE_ID
    payload["label"] = PROFILE_LABEL
    return payload


def get_fault_reference_metadata() -> dict[str, Any]:
    path = _drive_fault_codebook_path()
    return {
        "profile_id": PROFILE_ID,
        "label": FAULT_REFERENCE_LABEL,
        "source_path": path,
        "source_path_relative": FAULT_REFERENCE_RELATIVE_PATH,
        "available": os.path.exists(path),
    }


def describe_fault_code(error_code: int) -> dict[str, Any] | None:
    code_u16 = int(error_code) & 0xFFFF
    if code_u16 == 0:
        return None

    book = _load_drive_fault_codebook()
    key = f"0X{code_u16:04X}"
    bus_entry = book.get("tables", {}).get("bus_fault_codes", {}).get(key)
    if not isinstance(bus_entry, dict):
        bus_entry = None
    matched_fault_entry = None
    for candidate in book.get("tables", {}).get("fault_codes", {}).values():
        if not isinstance(candidate, dict):
            continue
        if str(candidate.get("bus_fault_code_603f", "")).strip().upper() == key:
            matched_fault_entry = candidate
            break

    if bus_entry is None and matched_fault_entry is None:
        return {
            "error_code_hex": f"0x{code_u16:04x}",
            "profile_id": PROFILE_ID,
            "decoded": False,
        }

    return {
        "error_code_hex": f"0x{code_u16:04x}",
        "profile_id": PROFILE_ID,
        "decoded": True,
        "code": matched_fault_entry.get("code") if isinstance(matched_fault_entry, dict) else None,
        "name": (
            matched_fault_entry.get("name")
            if isinstance(matched_fault_entry, dict) and matched_fault_entry.get("name")
            else (bus_entry.get("name") if isinstance(bus_entry, dict) else None)
        ),
        "class": matched_fault_entry.get("class") if isinstance(matched_fault_entry, dict) else None,
        "resettable": matched_fault_entry.get("resettable") if isinstance(matched_fault_entry, dict) else None,
        "bus_fault_name": bus_entry.get("name") if isinstance(bus_entry, dict) else None,
    }


def describe_manufacturer_fault_code(error_code: int) -> dict[str, Any] | None:
    code_u32 = int(error_code) & 0xFFFFFFFF
    if code_u32 == 0:
        return None

    alarm_entry = _match_entry_by_numeric_code("alarm_codes", "alarm_code_203f", code_u32)
    fault_entry = _match_entry_by_numeric_code("fault_codes", "fault_code_203f", code_u32)
    matched_entry = fault_entry if isinstance(fault_entry, dict) else alarm_entry
    if not isinstance(matched_entry, dict):
        return {
            "error_code_hex": f"0x{code_u32:08x}",
            "profile_id": PROFILE_ID,
            "decoded": False,
        }

    bus_fault_code = str(matched_entry.get("bus_fault_code_603f", "")).strip().upper() or None
    bus_fault_name = None
    if bus_fault_code:
        bus_entry = _load_drive_fault_codebook().get("tables", {}).get("bus_fault_codes", {}).get(bus_fault_code)
        if isinstance(bus_entry, dict):
            bus_fault_name = bus_entry.get("name")

    return {
        "error_code_hex": f"0x{code_u32:08x}",
        "profile_id": PROFILE_ID,
        "decoded": True,
        "code": matched_entry.get("code"),
        "name": matched_entry.get("name"),
        "class": matched_entry.get("class"),
        "resettable": matched_entry.get("resettable"),
        "bus_fault_code_hex": bus_fault_code.lower() if isinstance(bus_fault_code, str) else None,
        "bus_fault_name": bus_fault_name,
        "source": "manufacturer_error_code",
    }


def _startup_mode_value_label(raw_value: object) -> str | None:
    try:
        mode_value = int(raw_value)
    except Exception:
        return None
    return STARTUP_SETTING_VALUE_LABELS.get(mode_value)


def _render_native_home_config_spec(config: Mapping[str, Any]) -> str:
    parts = [
        f"steady_state_mode|{int(config.get('steady_state_mode', 0))}",
        f"commissioning_mode|{int(config.get('commissioning_mode', 0))}",
    ]
    truth_source = config.get("truth_source") if isinstance(config.get("truth_source"), Mapping) else {}
    parts.append(
        "truth_source|"
        f"0x{int(truth_source.get('index', 0)) & 0xFFFF:04X}|"
        f"0x{int(truth_source.get('subindex', 0)) & 0xFF:02X}|"
        f"{str(truth_source.get('type', 'i32')).strip().lower()}"
    )
    for step in list(config.get("transaction", [])):
        if not isinstance(step, Mapping):
            continue
        op = str(step.get("op", "")).strip().lower()
        if op in {"set_mode", "restore_mode"}:
            parts.append(f"op|{op}|{int(step.get('value', 0))}")
            continue
        if op == "write_sdo":
            parts.append(
                "op|write_sdo|"
                f"0x{int(step.get('index', 0)) & 0xFFFF:04X}|"
                f"0x{int(step.get('subindex', 0)) & 0xFF:02X}|"
                f"{str(step.get('type', 'u16')).strip().lower()}|"
                f"{int(step.get('value', 0))}"
            )
            continue
        if op == "wait_sdo":
            parts.append(
                "op|wait_sdo|"
                f"0x{int(step.get('index', 0)) & 0xFFFF:04X}|"
                f"0x{int(step.get('subindex', 0)) & 0xFF:02X}|"
                f"{str(step.get('type', 'u16')).strip().lower()}|"
                f"{int(step.get('value', 0))}"
            )
            continue
        if op == "release_service_override":
            parts.append("op|release_service_override")
            continue
        if op == "controlword_sequence":
            rendered_values = ",".join(str(int(value)) for value in list(step.get("values", [])))
            parts.append(f"op|controlword_sequence|{rendered_values}")
            continue
        if op == "wait_statusword":
            parts.append(
                "op|wait_statusword|"
                f"0x{int(step.get('all_set_mask', 0)) & 0xFFFF:04X}|"
                f"0x{int(step.get('all_clear_mask', 0)) & 0xFFFF:04X}"
            )
            continue
        if op == "refresh_truth":
            parts.append("op|refresh_truth")
    return ";".join(parts)


def _render_absolute_feedback_config_spec(config: Mapping[str, Any]) -> str:
    parts: list[str] = []
    for raw_field in list(config.get("fields", [])):
        if not isinstance(raw_field, Mapping):
            continue
        key = str(raw_field.get("key", "")).strip()
        field_type = str(raw_field.get("type", "")).strip().lower()
        if not key or not field_type:
            continue
        parts.append(
            f"{key}|"
            f"0x{int(raw_field.get('index', 0)) & 0xFFFF:04X}|"
            f"0x{int(raw_field.get('subindex', 0)) & 0xFF:02X}|"
            f"{field_type}"
        )
    return ";".join(parts)


def _normalize_absolute_feedback_field(
    raw: object,
    *,
    label: str,
    object_label: str,
) -> dict[str, Any]:
    field = raw if isinstance(raw, Mapping) else {}
    try:
        valid = bool(int(field.get("valid", 0)))
    except Exception:
        valid = False
    try:
        value = int(field.get("value", 0))
    except Exception:
        value = 0
    return {
        "label": str(label),
        "object": str(object_label),
        "valid": bool(valid),
        "value": int(value),
    }


def _combine_signed_i64_pair(low_field: Mapping[str, Any], high_field: Mapping[str, Any]) -> int | None:
    if not bool(low_field.get("valid")) or not bool(high_field.get("valid")):
        return None
    combined = ((int(high_field.get("value", 0)) & 0xFFFFFFFF) << 32) | (
        int(low_field.get("value", 0)) & 0xFFFFFFFF
    )
    if combined >= (1 << 63):
        combined -= 1 << 64
    return int(combined)


def build_startup_config(raw_entries: object, *, num_axes: int) -> dict[str, Any]:
    profile = get_ethercat_drive_profile(PROFILE_ID) or {}
    startup_schema = profile.get("startup_schema") if isinstance(profile, dict) else {}
    setting_schema = startup_schema.get(STARTUP_SETTING_KEY) if isinstance(startup_schema, dict) else {}
    min_value = int(setting_schema.get("min", 0)) if isinstance(setting_schema, dict) else 0
    max_value = int(setting_schema.get("max", 5)) if isinstance(setting_schema, dict) else 5
    object_spec = setting_schema.get("object") if isinstance(setting_schema, dict) else {}
    object_index = int(object_spec.get("index", 0x2000)) if isinstance(object_spec, dict) else 0x2000
    object_subindex = int(object_spec.get("subindex", 0x08)) if isinstance(object_spec, dict) else 0x08

    if raw_entries in (None, "", []):
        raw_entries = build_default_startup_entries(PROFILE_ID, num_axes=int(num_axes))
    if not isinstance(raw_entries, list):
        raise ValueError("EtherCAT drive startup config entries must be a list.")
    if len(raw_entries) not in (0, int(num_axes)):
        raise ValueError(
            "EtherCAT drive startup config entries must be empty or match num_physical_actuators."
        )

    encoder_position_tracking_modes: list[int] = []
    for axis_index, entry in enumerate(raw_entries):
        if not isinstance(entry, dict):
            raise ValueError(
                f"EtherCAT drive startup config entry[{axis_index}] must be a dict."
            )
        if STARTUP_SETTING_KEY not in entry:
            raise ValueError(
                f"EtherCAT drive startup config entry[{axis_index}] is missing {STARTUP_SETTING_KEY}."
            )
        try:
            tracking_mode = int(entry[STARTUP_SETTING_KEY])
        except Exception as exc:
            raise ValueError(
                f"EtherCAT drive startup config entry[{axis_index}] has an invalid {STARTUP_SETTING_KEY}."
            ) from exc
        if tracking_mode < min_value or tracking_mode > max_value:
            raise ValueError(
                f"EtherCAT drive startup config entry[{axis_index}] must use {STARTUP_SETTING_LABEL} {min_value}-{max_value}."
            )
        encoder_position_tracking_modes.append(tracking_mode)

    return {
        "profile_id": PROFILE_ID,
        "settings": {
            STARTUP_SETTING_KEY: encoder_position_tracking_modes,
        },
        "env": {
            STARTUP_SETTING_ENV_VAR: (
                f"{STARTUP_SETTING_KEY}|u16|0x{object_index & 0xFFFF:04X}|0x{object_subindex & 0xFF:02X}|"
                + ",".join(str(int(value)) for value in encoder_position_tracking_modes)
            ),
        },
    }


def extract_startup_config_axis(axis: Mapping[str, Any]) -> dict[str, Any] | None:
    startup_drive_config = axis.get("startup_drive_config")
    if not isinstance(startup_drive_config, Mapping):
        return None
    setting_key = str(startup_drive_config.get("setting_key", "")).strip()
    if setting_key != STARTUP_SETTING_KEY:
        return None
    return {
        "profile_id": PROFILE_ID,
        "setting_key": STARTUP_SETTING_KEY,
        "setting_label": STARTUP_SETTING_LABEL,
        "object": STARTUP_SETTING_OBJECT,
        "configured": bool(startup_drive_config.get("configured")),
        "commanded": int(startup_drive_config.get("commanded", 0)),
        "commanded_value_label": _startup_mode_value_label(startup_drive_config.get("commanded", 0)),
        "readback_valid": bool(startup_drive_config.get("readback_valid")),
        "readback": int(startup_drive_config.get("readback", 0)),
        "readback_value_label": _startup_mode_value_label(startup_drive_config.get("readback", 0)),
        "verified": bool(startup_drive_config.get("verified")),
    }


def get_encoder_data_reset_operation() -> dict[str, Any]:
    return dict(ENCODER_DATA_RESET_OPERATION)


def get_absolute_feedback_config() -> dict[str, Any]:
    payload = {
        "profile_id": PROFILE_ID,
        "fields": [dict(field) for field in ABSOLUTE_FEEDBACK_FIELDS],
        "sources": [dict(source) for source in ABSOLUTE_FEEDBACK_SOURCES],
        "preferred_display_source_keys": list(ABSOLUTE_FEEDBACK_DISPLAY_SOURCE_KEYS),
    }
    payload["env"] = {
        ABSOLUTE_FEEDBACK_CONFIG_ENV_VAR: _render_absolute_feedback_config_spec(payload),
    }
    return payload


def get_motion_feedback_config() -> dict[str, Any]:
    return dict(MOTION_FEEDBACK_CONFIG)


def normalize_absolute_feedback(raw_feedback: object) -> dict[str, Any] | None:
    if not isinstance(raw_feedback, Mapping):
        return None
    normalized: dict[str, Any] = {}
    for field in ABSOLUTE_FEEDBACK_FIELDS:
        key = str(field.get("key", "")).strip()
        if not key:
            continue
        normalized[key] = _normalize_absolute_feedback_field(
            raw_feedback.get(key),
            label=str(field.get("label", key)),
            object_label=str(field.get("object", "")).strip(),
        )
    for source in ABSOLUTE_FEEDBACK_SOURCES:
        kind = str(source.get("kind", "")).strip().lower()
        source_key = str(source.get("key", "")).strip()
        if kind != "signed_i64_pair" or not source_key:
            continue
        low_key = str(source.get("low_key", "")).strip()
        high_key = str(source.get("high_key", "")).strip()
        low_field = normalized.get(low_key)
        high_field = normalized.get(high_key)
        combined = _combine_signed_i64_pair(
            low_field if isinstance(low_field, Mapping) else {},
            high_field if isinstance(high_field, Mapping) else {},
        )
        if combined is None:
            continue
        normalized[source_key] = {
            "label": str(source.get("label", source_key)),
            "kind": kind,
            "valid": True,
            "value": int(combined),
        }
    return normalized


def resolve_absolute_feedback_counts(raw_feedback: object) -> dict[str, Any] | None:
    normalized = normalize_absolute_feedback(raw_feedback)
    if not isinstance(normalized, Mapping):
        return None
    for source_key in ABSOLUTE_FEEDBACK_DISPLAY_SOURCE_KEYS:
        source_field = normalized.get(source_key)
        if not isinstance(source_field, Mapping) or not bool(source_field.get("valid")):
            continue
        return {
            "source_key": str(source_key),
            "source_label": str(source_field.get("label", source_key)),
            "counts": int(source_field.get("value", 0)),
        }
    return None


def get_native_home_config() -> dict[str, Any]:
    payload = dict(NATIVE_HOME_CONFIG)
    payload["truth_source"] = dict(NATIVE_HOME_TRUTH_SOURCE)
    payload["transaction"] = [dict(step) for step in list(NATIVE_HOME_CONFIG.get("transaction", []))]
    payload["env"] = {
        NATIVE_HOME_CONFIG_ENV_VAR: _render_native_home_config_spec(payload),
    }
    return payload
