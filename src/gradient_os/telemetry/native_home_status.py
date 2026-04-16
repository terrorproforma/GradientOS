from __future__ import annotations

from collections.abc import Mapping


_HM_STATUSWORD_BIT_12 = 1 << 12
_HM_STATUSWORD_BIT_13 = 1 << 13
_HM_STATUSWORD_BIT_15 = 1 << 15
_STATUSWORD_HM_SUCCESS_SOURCE = "statusword_bits12_15_clear13"
_STATUSWORD_BIT15_SOURCE = "statusword_bit15"


def _coerce_int(value: object, default: int = 0) -> int:
    try:
        return int(value)
    except Exception:
        return int(default)


def native_home_state_name(value: object) -> str:
    state = _coerce_int(value, 0)
    labels = {
        0: "idle",
        1: "requested",
        2: "succeeded",
        3: "failed",
    }
    return labels.get(state, f"unknown:{state}")


def statusword_indicates_valid_native_home_reference(statusword: object) -> bool:
    value = _coerce_int(statusword, 0) & 0xFFFF
    required_mask = _HM_STATUSWORD_BIT_12 | _HM_STATUSWORD_BIT_15
    return (value & required_mask) == required_mask and (value & _HM_STATUSWORD_BIT_13) == 0


def _statusword_has_coordinate_system_valid_bit(statusword: object) -> bool:
    value = _coerce_int(statusword, 0) & 0xFFFF
    return (value & _HM_STATUSWORD_BIT_15) != 0


def _coordinate_system_validity_from_statusword(
    statusword: object,
    *,
    require_hm_success_signature: bool = True,
) -> tuple[bool, str]:
    if statusword_indicates_valid_native_home_reference(statusword):
        return True, _STATUSWORD_HM_SUCCESS_SOURCE
    if not bool(require_hm_success_signature) and _statusword_has_coordinate_system_valid_bit(statusword):
        return True, _STATUSWORD_BIT15_SOURCE
    return False, "unverified"


def derive_effective_native_home_status(
    axis: Mapping[str, object],
    *,
    statusword: int,
    error_code: int,
    manufacturer_error_code: int,
) -> dict[str, object]:
    reported_state = _coerce_int(axis.get("native_home_state"), 0)
    reported_abort_code = _coerce_int(axis.get("native_home_last_abort_code"), 0)
    effective_state = reported_state
    effective_abort_code = reported_abort_code
    verification_source = "reported"

    # If the drive currently advertises the vendor-confirmed HM success signature
    # (bits 12 and 15 set, bit 13 clear) with no live fault, trust that fresh
    # wire-state over a stale last-operation result cached in metrics.
    if (
        effective_state not in {1, 2}
        and error_code == 0
        and manufacturer_error_code == 0
        and statusword_indicates_valid_native_home_reference(statusword)
    ):
        effective_state = 2
        effective_abort_code = 0
        verification_source = _STATUSWORD_HM_SUCCESS_SOURCE

    return {
        "native_home_state": int(effective_state),
        "native_home_state_name": native_home_state_name(effective_state),
        "native_home_last_abort_code": int(effective_abort_code),
        "native_home_last_abort_code_hex": f"0x{effective_abort_code & 0xFFFFFFFF:08x}",
        "native_home_state_reported": int(reported_state),
        "native_home_state_reported_name": native_home_state_name(reported_state),
        "native_home_last_abort_code_reported": int(reported_abort_code),
        "native_home_last_abort_code_reported_hex": f"0x{reported_abort_code & 0xFFFFFFFF:08x}",
        "native_home_verification_source": verification_source,
    }


def derive_drive_native_truth_validity(
    axis: Mapping[str, object] | None,
    *,
    statusword: int,
    error_code: int,
    manufacturer_error_code: int,
    require_hm_success_signature: bool = True,
) -> dict[str, object]:
    axis_payload = axis if isinstance(axis, Mapping) else {}
    signature_valid = statusword_indicates_valid_native_home_reference(statusword)
    coordinate_system_valid, verification_source = _coordinate_system_validity_from_statusword(
        statusword,
        require_hm_success_signature=require_hm_success_signature,
    )
    slave_online = None
    slave_operational = None
    native_home_active = None
    if "slave_online" in axis_payload:
        slave_online = bool(axis_payload.get("slave_online"))
    if "slave_operational" in axis_payload:
        slave_operational = bool(axis_payload.get("slave_operational"))
    if "native_home_active" in axis_payload:
        native_home_active = bool(axis_payload.get("native_home_active"))

    if statusword == 0:
        valid = False
        reason = "statusword_unavailable"
    elif error_code != 0:
        valid = False
        reason = "fault_present"
    elif manufacturer_error_code != 0:
        valid = False
        reason = "manufacturer_fault_present"
    elif slave_online is False:
        valid = False
        reason = "slave_offline"
    elif slave_operational is False:
        valid = False
        reason = "slave_not_operational"
    elif native_home_active:
        valid = False
        reason = "native_home_active"
    elif not coordinate_system_valid:
        valid = False
        reason = "coordinate_system_invalid"
    else:
        valid = True
        reason = "valid"

    return {
        "drive_native_truth_valid": bool(valid),
        "drive_native_truth_reason": str(reason),
        "drive_native_truth_signature_valid": bool(signature_valid),
        "coordinate_system_valid": bool(coordinate_system_valid),
        "drive_native_truth_verification_source": str(verification_source),
    }
