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


def get_native_home_config() -> dict[str, Any]:
    payload = dict(NATIVE_HOME_CONFIG)
    payload["truth_source"] = dict(NATIVE_HOME_TRUTH_SOURCE)
    payload["transaction"] = [dict(step) for step in list(NATIVE_HOME_CONFIG.get("transaction", []))]
    payload["env"] = {
        NATIVE_HOME_CONFIG_ENV_VAR: _render_native_home_config_spec(payload),
    }
    return payload
