from __future__ import annotations

from collections.abc import Mapping


_HM_STATUSWORD_BIT_12 = 1 << 12
_HM_STATUSWORD_BIT_13 = 1 << 13
_HM_STATUSWORD_BIT_15 = 1 << 15
_STATUSWORD_HM_SUCCESS_SOURCE = "statusword_bits12_15_clear13"
# `statusword_bit15` identifies the relaxed restart-trust path that accepts
# a clean 6041 bit 15 as evidence that the drive's coordinate system
# survived power cycle even without fresh HM-success signature. On the
# A6-EC firmware we currently run, bit 15 is empirically cleared on every
# drive power cycle (see
# `a6ec_ds402.POSITION_SEMANTICS_CONFIG["firmware_bit15_retention_expected"]`
# which advertises this as `False`), so this source is unreachable on
# production A6-EC hardware. It is kept in the helper for future firmware
# releases / drive families that honour vendor Q9.
_STATUSWORD_BIT15_SOURCE = "statusword_bit15"
_PERSISTED_HOME_ANCHOR_AGREEMENT_SOURCE = "persisted_home_anchor_agreement"


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
    """
    Return ``(coordinate_system_valid, verification_source)`` from the
    raw statusword alone. The ``statusword_bit15`` source is kept for
    future firmware/drive families that honour vendor Q9; on the A6-EC
    firmware we currently run bit 15 is cleared on every drive power
    cycle so that source is unreachable in practice, and we rely on the
    persisted-home-anchor restart-trust path instead. See
    ``a6ec_ds402.POSITION_SEMANTICS_CONFIG["firmware_bit15_retention_expected"]``.
    """
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
    accept_persisted_home_anchor_as_restart_trust: bool = False,
    persisted_home_anchor_present: bool | None = None,
    persisted_home_anchor_consistent: bool | None = None,
    multi_turn_feedback_valid: bool | None = None,
    encoder_retention_fault_present: bool | None = None,
    last_seen_present: bool | None = None,
    last_seen_delta_physically_possible: bool | None = None,
) -> dict[str, object]:
    axis_payload = axis if isinstance(axis, Mapping) else {}
    signature_valid = statusword_indicates_valid_native_home_reference(statusword)
    coordinate_system_valid, verification_source = _coordinate_system_validity_from_statusword(
        statusword,
        require_hm_success_signature=require_hm_success_signature,
    )
    # Fallback trust path: when the drive no longer advertises bit 15 after
    # a power cycle (some firmware clears it despite vendor Q9), but we
    # still have a recorded absolute-home anchor AND the live multi-turn
    # reading agrees with it modulo RM, accept the home as still valid.
    # Requires an explicit opt-in per drive profile, all three signals
    # supplied, and a clean drive (fault / slave / native-home-active
    # checks below still short-circuit this path).
    if (
        not coordinate_system_valid
        and bool(accept_persisted_home_anchor_as_restart_trust)
        and persisted_home_anchor_present is True
        and persisted_home_anchor_consistent is True
        and multi_turn_feedback_valid is True
        and not bool(encoder_retention_fault_present)
    ):
        coordinate_system_valid = True
        verification_source = _PERSISTED_HOME_ANCHOR_AGREEMENT_SOURCE
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
    elif bool(encoder_retention_fault_present):
        # A live encoder-retention-family fault (e.g. Er20.9 multi-turn
        # error or ALF9.0 battery alarm) is a more specific interpretation
        # of the same underlying error_code / manufacturer_error_code
        # signal, so it outranks the generic fault branches below. The
        # persisted-home-anchor path depends on multi-turn integrity, so
        # retention loss must fail closed even when the shaft-frame gate
        # would otherwise agree.
        valid = False
        reason = "encoder_retention_fault_present"
        coordinate_system_valid = False
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
        # Provide a precise reason when the anchor path was opted in but
        # did not satisfy one of its preconditions.
        if (
            bool(accept_persisted_home_anchor_as_restart_trust)
            and persisted_home_anchor_present is False
        ):
            valid = False
            reason = "persisted_home_anchor_missing"
        elif (
            bool(accept_persisted_home_anchor_as_restart_trust)
            and multi_turn_feedback_valid is False
        ):
            valid = False
            reason = "multi_turn_feedback_invalid"
        elif (
            bool(accept_persisted_home_anchor_as_restart_trust)
            and persisted_home_anchor_consistent is False
        ):
            valid = False
            # If the backend has a last-seen U40.20/.22 sidecar on record
            # AND the delta between that stored reading and the live
            # value exceeds the drive's physically-possible off-motor
            # budget (>32k motor turns for A6-EC), the encoder almost
            # certainly lost state across the power cycle. Promote the
            # reason to a more specific "lost across power cycle" label
            # so operators can distinguish encoder-loss from a simple
            # anchor-vs-6064 disagreement.
            if (
                last_seen_present is True
                and last_seen_delta_physically_possible is False
            ):
                reason = "multi_turn_feedback_lost_across_power_cycle"
            else:
                reason = "persisted_home_anchor_inconsistent_with_live_6064"
        else:
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
