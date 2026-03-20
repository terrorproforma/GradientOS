from __future__ import annotations

import json
import os
from typing import Any

from . import cia402

PROFILE_ID = "a6ec_ds402"
PROFILE_LABEL = "A6-EC over CiA402"
FAULT_REFERENCE_LABEL = "A6-EC CiA402 0x603F fault codes"
FAULT_REFERENCE_RELATIVE_PATH = os.path.join("docs", "resources", "a6ec_manual_codes.json")

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
