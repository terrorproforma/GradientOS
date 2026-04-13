from __future__ import annotations

import importlib
from types import ModuleType
from typing import Any

_DRIVE_PROFILE_MODULES: dict[str, str] = {
    "cia402": "gradient_os.arm_controller.profiles.drive.cia402",
    "a6ec_ds402": "gradient_os.arm_controller.profiles.drive.a6ec_ds402",
}

_FIELDBUS_PROFILE_MODULES: dict[str, str] = {
    "ethercat_al": "gradient_os.arm_controller.profiles.fieldbus.ethercat",
}


def _load_profile(module_map: dict[str, str], profile_id: str | None) -> ModuleType | None:
    token = str(profile_id or "").strip().lower()
    if not token:
        return None
    module_path = module_map.get(token)
    if not module_path:
        return None
    try:
        return importlib.import_module(module_path)
    except Exception:
        return None


def list_drive_profiles() -> list[str]:
    return sorted(_DRIVE_PROFILE_MODULES.keys())


def list_fieldbus_profiles() -> list[str]:
    return sorted(_FIELDBUS_PROFILE_MODULES.keys())


def get_drive_profile(profile_id: str | None) -> ModuleType | None:
    return _load_profile(_DRIVE_PROFILE_MODULES, profile_id)


def get_fieldbus_profile(profile_id: str | None) -> ModuleType | None:
    return _load_profile(_FIELDBUS_PROFILE_MODULES, profile_id)


def decode_drive_statusword(profile_id: str | None, statusword: int) -> dict[str, Any] | None:
    module = get_drive_profile(profile_id)
    decoder = getattr(module, "decode_statusword", None) if module is not None else None
    if callable(decoder):
        payload = decoder(int(statusword))
        return payload if isinstance(payload, dict) else None
    return None


def get_drive_fault_reference_metadata(profile_id: str | None) -> dict[str, Any] | None:
    module = get_drive_profile(profile_id)
    getter = getattr(module, "get_fault_reference_metadata", None) if module is not None else None
    if callable(getter):
        payload = getter()
        return payload if isinstance(payload, dict) else None
    return None


def describe_drive_fault_code(profile_id: str | None, error_code: int) -> dict[str, Any] | None:
    module = get_drive_profile(profile_id)
    decoder = getattr(module, "describe_fault_code", None) if module is not None else None
    if callable(decoder):
        payload = decoder(int(error_code))
        return payload if isinstance(payload, dict) else None
    return None


def describe_drive_manufacturer_fault_code(profile_id: str | None, error_code: int) -> dict[str, Any] | None:
    module = get_drive_profile(profile_id)
    decoder = getattr(module, "describe_manufacturer_fault_code", None) if module is not None else None
    if callable(decoder):
        payload = decoder(int(error_code))
        return payload if isinstance(payload, dict) else None
    return None


def build_drive_startup_config(
    profile_id: str | None,
    raw_entries: object,
    *,
    num_axes: int,
) -> dict[str, Any] | None:
    module = get_drive_profile(profile_id)
    builder = getattr(module, "build_startup_config", None) if module is not None else None
    if callable(builder):
        payload = builder(raw_entries, num_axes=int(num_axes))
        return payload if isinstance(payload, dict) else None
    return None


def extract_drive_startup_config_axis(
    profile_id: str | None,
    axis: dict[str, Any],
) -> dict[str, Any] | None:
    module = get_drive_profile(profile_id)
    extractor = getattr(module, "extract_startup_config_axis", None) if module is not None else None
    if callable(extractor):
        payload = extractor(axis)
        return payload if isinstance(payload, dict) else None
    return None


def get_drive_encoder_data_reset_operation(profile_id: str | None) -> dict[str, Any] | None:
    module = get_drive_profile(profile_id)
    getter = getattr(module, "get_encoder_data_reset_operation", None) if module is not None else None
    if callable(getter):
        payload = getter()
        return payload if isinstance(payload, dict) else None
    return None


def get_drive_native_home_config(profile_id: str | None) -> dict[str, Any] | None:
    module = get_drive_profile(profile_id)
    getter = getattr(module, "get_native_home_config", None) if module is not None else None
    if callable(getter):
        payload = getter()
        return payload if isinstance(payload, dict) else None
    return None


def get_drive_absolute_feedback_config(profile_id: str | None) -> dict[str, Any] | None:
    module = get_drive_profile(profile_id)
    getter = getattr(module, "get_absolute_feedback_config", None) if module is not None else None
    if callable(getter):
        payload = getter()
        return payload if isinstance(payload, dict) else None
    return None


def get_drive_motion_feedback_config(profile_id: str | None) -> dict[str, Any] | None:
    module = get_drive_profile(profile_id)
    getter = getattr(module, "get_motion_feedback_config", None) if module is not None else None
    if callable(getter):
        payload = getter()
        return payload if isinstance(payload, dict) else None
    return None


def normalize_drive_absolute_feedback(
    profile_id: str | None,
    raw_feedback: object,
) -> dict[str, Any] | None:
    module = get_drive_profile(profile_id)
    normalizer = getattr(module, "normalize_absolute_feedback", None) if module is not None else None
    if callable(normalizer):
        payload = normalizer(raw_feedback)
        return payload if isinstance(payload, dict) else None
    return None


def resolve_drive_absolute_feedback_counts(
    profile_id: str | None,
    raw_feedback: object,
) -> dict[str, Any] | None:
    module = get_drive_profile(profile_id)
    resolver = getattr(module, "resolve_absolute_feedback_counts", None) if module is not None else None
    if callable(resolver):
        payload = resolver(raw_feedback)
        return payload if isinstance(payload, dict) else None
    return None


def decode_fieldbus_state(profile_id: str | None, value: int) -> dict[str, Any] | None:
    module = get_fieldbus_profile(profile_id)
    decoder = getattr(module, "decode_al_state", None) if module is not None else None
    if callable(decoder):
        payload = decoder(int(value))
        return payload if isinstance(payload, dict) else None
    return None


def describe_fieldbus_master_state(
    profile_id: str | None,
    *,
    link_up: int,
    responding: int,
    operational: int,
    num_axes: int,
) -> str | None:
    module = get_fieldbus_profile(profile_id)
    describer = getattr(module, "describe_master_state", None) if module is not None else None
    if callable(describer):
        payload = describer(
            link_up=int(link_up),
            responding=int(responding),
            operational=int(operational),
            num_axes=int(num_axes),
        )
        return str(payload) if payload is not None else None
    return None
