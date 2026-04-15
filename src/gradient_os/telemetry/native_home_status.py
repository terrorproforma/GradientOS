from __future__ import annotations

from collections.abc import Mapping


_HM_STATUSWORD_BIT_12 = 1 << 12
_HM_STATUSWORD_BIT_13 = 1 << 13
_HM_STATUSWORD_BIT_15 = 1 << 15
_STATUSWORD_HM_SUCCESS_SOURCE = "statusword_bits12_15_clear13"


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
