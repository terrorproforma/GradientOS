from __future__ import annotations

from collections.abc import Mapping, Sequence
from typing import Any

from ..arm_controller.backends import registry as backend_registry
from .native_home_status import (
    derive_drive_native_truth_validity,
    derive_effective_native_home_status,
)


def coerce_int(value: object, default: int = 0) -> int:
    try:
        return int(value)
    except Exception:
        return int(default)


def axis_snapshot_is_faulted(axis: Mapping[str, object]) -> bool:
    error_code = coerce_int(axis.get("error_code"), 0)
    ds402_state = str(axis.get("ds402_state", axis.get("drive_state", ""))).strip()
    return error_code != 0 or ds402_state in {"Fault", "FaultReactionActive"}


def build_startup_fault_reset_plan(probe: Mapping[str, object]) -> dict[str, Any]:
    physical_state = str(probe.get("physical_state", "")).strip().upper() or "UNKNOWN"
    driver_state = str(probe.get("driver_state", "")).strip().upper() or "UNKNOWN"
    rtcore_state = str(probe.get("rtcore_state", "")).strip().upper() or "UNKNOWN"
    armed = coerce_int(probe.get("armed"), 0)
    enable_mask = coerce_int(probe.get("axis_enable_mask"), 0)
    op_enabled_axes = coerce_int(probe.get("op_enabled_axes"), 0)

    faulted_axes: list[dict[str, Any]] = []
    faulted_axis_mask = 0
    for axis in probe.get("axes", []) if isinstance(probe.get("axes"), list) else []:
        if not isinstance(axis, Mapping):
            continue
        if not axis_snapshot_is_faulted(axis):
            continue
        axis_index = coerce_int(axis.get("axis", axis.get("index")), -1)
        if axis_index < 0:
            continue
        faulted_axis_mask |= 1 << axis_index
        logical_joint_raw = axis.get("logical_joint")
        logical_joint = (
            coerce_int(logical_joint_raw, -1)
            if logical_joint_raw is not None and str(logical_joint_raw).strip() != ""
            else None
        )
        fault = axis.get("fault") if isinstance(axis.get("fault"), Mapping) else {}
        label = f"axis{axis_index}"
        if isinstance(logical_joint, int) and logical_joint > 0:
            label = f"J{logical_joint}/axis{axis_index}"
        faulted_axes.append(
            {
                "axis": axis_index,
                "logical_joint": logical_joint,
                "label": label,
                "error_code_hex": str(
                    axis.get("error_code_hex", f"0x{coerce_int(axis.get('error_code'), 0) & 0xFFFF:04x}")
                ).strip(),
                "ds402_state": str(axis.get("ds402_state", axis.get("drive_state", "UNKNOWN"))).strip() or "UNKNOWN",
                "fault_code": str(fault.get("code", "")).strip() or None,
                "fault_name": str(fault.get("name", "")).strip() or None,
                "resettable": fault.get("resettable") is True,
            }
        )

    summaries: list[str] = []
    for axis in faulted_axes:
        parts = [str(axis["label"]), str(axis["error_code_hex"])]
        if axis.get("fault_code"):
            parts.append(str(axis["fault_code"]))
        if axis.get("fault_name"):
            parts.append(str(axis["fault_name"]))
        summaries.append(" ".join(parts))

    disarmed_safe = armed == 0 and enable_mask == 0 and op_enabled_axes == 0
    should_auto_reset = (
        physical_state == "FAULTED"
        and disarmed_safe
        and rtcore_state == "UP"
        and faulted_axis_mask != 0
    )
    blocks_startup = faulted_axis_mask != 0 and not should_auto_reset

    if faulted_axis_mask == 0:
        reason = "no_faulted_axes"
    elif physical_state != "FAULTED":
        reason = f"physical_state={physical_state}"
    elif not disarmed_safe:
        reason = (
            f"hardware_not_disarmed"
            f"(driver_state={driver_state} armed={armed} enable_mask=0x{enable_mask:x}"
            f" op_enabled_axes={op_enabled_axes})"
        )
    elif rtcore_state != "UP":
        reason = f"rtcore_state={rtcore_state}"
    else:
        reason = "faulted_disarmed_axes_ready_for_reset"

    return {
        "physical_state": physical_state,
        "driver_state": driver_state,
        "rtcore_state": rtcore_state,
        "armed": armed,
        "axis_enable_mask": enable_mask,
        "op_enabled_axes": op_enabled_axes,
        "faulted_axis_count": len(faulted_axes),
        "faulted_axis_mask": faulted_axis_mask,
        "faulted_axis_mask_hex": f"0x{faulted_axis_mask:x}",
        "faulted_axes": faulted_axes,
        "faulted_summary": ", ".join(summaries),
        "should_auto_reset": should_auto_reset,
        "blocks_startup": blocks_startup,
        "reason": reason,
    }


def build_drive_fault_snapshot(
    *,
    metrics: Mapping[str, object],
    servo_backend: str | None,
    drive_profile: str | None = None,
    configured_drive_profile: str | None = None,
    live_drive_profile: str | None = None,
    fieldbus_profile: str | None = None,
    axis_to_joint: Sequence[int] | None = None,
    socket_present: bool | None = None,
    axis_drive_native_truth_context: Mapping[int, Mapping[str, Any]] | None = None,
    axis_extended_telemetry_context: Mapping[int, Mapping[str, Any]] | None = None,
) -> dict[str, Any]:
    """Build the monitor/telemetry drive-fault snapshot.

    When ``axis_drive_native_truth_context`` is supplied (typically by
    ``run_controller._build_drive_fault_snapshot`` calling the active
    backend's ``get_display_feedback_snapshot()``), the per-axis
    ``drive_native_truth_*`` and ``coordinate_system_valid`` fields
    published here are taken directly from that context instead of
    being re-derived locally. This preserves the backend's
    persisted-home-anchor restart-trust decision for the /monitor
    stream and keeps the UI's drive-native-truth surface consistent
    with /info/joints-detailed. Without the context the local
    ``derive_drive_native_truth_validity`` fallback still runs but
    only has access to the signals visible on the RTCore metrics
    axis payload (statusword, error codes, slave state) and cannot
    evaluate the anchor-based trust path, so it will report
    ``coordinate_system_invalid`` for axes whose bit 15 is clear even
    though the backend may have upgraded them to
    ``persisted_home_anchor_agreement``.

    Phase 2 (2026-04-20): when ``axis_extended_telemetry_context`` is
    supplied (also by ``run_controller._build_drive_fault_snapshot``
    from the backend's extended PDO state), the new A6-EC 0x2040
    diagnostic fields (``bus_voltage_v``, ``load_rate_pct``,
    ``igbt_temp_c``, ``motor_temp_c``, ``position_error_counts``,
    ``drive_not_ready_bits``/``_text``, ``motor_not_rotating_code``/
    ``_text``) are merged into each per-axis payload entry. The UI
    commissioning panel reads these to render the per-axis health
    strip. Unsupplied entries leave the fields absent so the UI can
    tell "drive rejected extended PDO mapping" from "drive reports 0.0".
    """
    num_axes = coerce_int(metrics.get("num_axes"), 0)
    armed = coerce_int(metrics.get("armed"), 0)
    enable_mask = coerce_int(metrics.get("axis_enable_mask"), 0)
    native_home_active_axis_mask = coerce_int(metrics.get("native_home_active_axis_mask"), 0)
    link_up = coerce_int(metrics.get("link_up"), 0)
    responding = coerce_int(metrics.get("responding_slaves"), 0)
    online = coerce_int(metrics.get("online_slaves"), 0)
    operational = coerce_int(metrics.get("operational_slaves"), 0)
    startup_ready = coerce_int(metrics.get("startup_ready"), 0)
    wkc_actual = coerce_int(metrics.get("wkc_actual"), 0)
    wkc_expected = coerce_int(metrics.get("wkc_expected"), 0)
    master_al = coerce_int(metrics.get("master_al_states"), 0)
    enable_requested = armed != 0 or enable_mask != 0
    requested_axes = (
        (enable_mask & ((1 << num_axes) - 1)).bit_count()
        if num_axes > 0
        else 0
    )
    configured_profile = configured_drive_profile if configured_drive_profile is not None else drive_profile
    effective_drive_profile = live_drive_profile or drive_profile or configured_profile
    drive_profile_source = "live_rtcore" if live_drive_profile else "configured_fallback"
    resolved_drive_profile = backend_registry.resolve_drive_profile_for_backend(
        servo_backend,
        drive_profile_id=effective_drive_profile,
    )
    resolved_fieldbus_profile = backend_registry.resolve_fieldbus_profile_for_backend(
        servo_backend,
        fieldbus_profile_id=fieldbus_profile,
    )
    reference = backend_registry.get_drive_fault_reference_metadata_for_backend(
        servo_backend,
        drive_profile_id=resolved_drive_profile,
    )
    position_semantics = backend_registry.get_drive_position_semantics_config_for_backend(
        servo_backend,
        drive_profile_id=resolved_drive_profile,
    )
    master_al_detail = backend_registry.decode_fieldbus_state_for_backend(
        servo_backend,
        master_al,
        fieldbus_profile_id=resolved_fieldbus_profile,
    )
    drive_native_ratio_enabled = (
        bool(position_semantics.get("drive_native_ratio_enabled", False))
        if isinstance(position_semantics, Mapping)
        else False
    )
    startup_truth_requires_hm_success_signature = (
        bool(position_semantics.get("startup_truth_requires_hm_success_signature", True))
        if isinstance(position_semantics, Mapping)
        else True
    )
    position_semantics_source = (
        str(position_semantics.get("position_semantics_source", "absolute_encoder_anchor"))
        if isinstance(position_semantics, Mapping)
        else "absolute_encoder_anchor"
    )

    axes_raw = metrics.get("axes") if isinstance(metrics.get("axes"), list) else []
    axis_to_joint_list = list(axis_to_joint) if axis_to_joint is not None else []
    axes_payload: list[dict[str, object]] = []
    op_enabled_axes = 0
    faulted_axes = 0
    statusword_feedback_axes = 0
    slave_online_axes = 0
    slave_operational_axes = 0
    startup_drive_config_configured_axes = 0
    startup_drive_config_verified_axes = 0
    startup_drive_config_mismatch_axes = 0
    drive_native_startup_valid_axes = 0
    drive_native_startup_invalid_axes = 0
    drive_native_truth_valid_axes = 0
    drive_native_truth_invalid_axes = 0
    coordinate_system_valid_axes = 0

    for axis_index, axis in enumerate(axes_raw[:num_axes]):
        if not isinstance(axis, Mapping):
            continue
        statusword = coerce_int(axis.get("statusword"), 0)
        error_code = coerce_int(axis.get("error_code"), 0)
        manufacturer_error_code = coerce_int(axis.get("manufacturer_error_code"), 0)
        if statusword != 0:
            statusword_feedback_axes += 1
        drive_state = backend_registry.decode_drive_statusword_for_backend(
            servo_backend,
            statusword,
            drive_profile_id=resolved_drive_profile,
        )
        ds402_state = str(drive_state.get("state", "UNKNOWN")) if isinstance(drive_state, dict) else "UNKNOWN"
        if ds402_state == "OperationEnabled":
            op_enabled_axes += 1
        if axis_snapshot_is_faulted({"error_code": error_code, "ds402_state": ds402_state}):
            faulted_axes += 1
        native_home_status = derive_effective_native_home_status(
            axis,
            statusword=statusword,
            error_code=error_code,
            manufacturer_error_code=manufacturer_error_code,
        )
        # Ask the active drive profile whether the live fault codes belong
        # to the encoder-retention family. When so, pass that signal to
        # the truth-validity helper so operators see the more specific
        # "encoder_retention_fault_present" reason instead of the generic
        # "manufacturer_fault_present".
        encoder_retention_fault_detail = (
            backend_registry.describe_drive_encoder_retention_fault_for_backend(
                servo_backend,
                manufacturer_error_code=manufacturer_error_code,
                error_code=error_code,
                drive_profile_id=resolved_drive_profile,
            )
        )
        encoder_retention_fault_present = bool(
            isinstance(encoder_retention_fault_detail, Mapping)
            and encoder_retention_fault_detail.get("present", False)
        )
        drive_native_truth = derive_drive_native_truth_validity(
            axis,
            statusword=statusword,
            error_code=error_code,
            manufacturer_error_code=manufacturer_error_code,
            require_hm_success_signature=startup_truth_requires_hm_success_signature,
            encoder_retention_fault_present=encoder_retention_fault_present,
        )
        # When the active backend already ran the full truth-derivation
        # pipeline (with access to the persisted-home-anchor sidecar,
        # shaft-frame consistency, and retention-fault decoding) adopt
        # its per-axis result verbatim. The local call above is kept so
        # callers without backend context still get a best-effort
        # answer; but when the override is present it wins. This keeps
        # /monitor, /info/joints-detailed, and the UI aligned on a
        # single drive-native-truth decision per axis.
        if axis_drive_native_truth_context is not None:
            override = axis_drive_native_truth_context.get(axis_index)
            if isinstance(override, Mapping):
                drive_native_truth = {
                    "drive_native_truth_valid": bool(
                        override.get(
                            "drive_native_truth_valid",
                            drive_native_truth.get("drive_native_truth_valid", False),
                        )
                    ),
                    "drive_native_truth_reason": str(
                        override.get(
                            "drive_native_truth_reason",
                            drive_native_truth.get("drive_native_truth_reason", "unknown"),
                        )
                    ),
                    "drive_native_truth_signature_valid": bool(
                        override.get(
                            "drive_native_truth_signature_valid",
                            drive_native_truth.get("drive_native_truth_signature_valid", False),
                        )
                    ),
                    "coordinate_system_valid": bool(
                        override.get(
                            "coordinate_system_valid",
                            drive_native_truth.get("coordinate_system_valid", False),
                        )
                    ),
                    "drive_native_truth_verification_source": str(
                        override.get(
                            "drive_native_truth_verification_source",
                            drive_native_truth.get(
                                "drive_native_truth_verification_source", "unverified"
                            ),
                        )
                    ),
                }
        if bool(drive_native_truth.get("coordinate_system_valid", False)):
            coordinate_system_valid_axes += 1
        if drive_native_ratio_enabled:
            if bool(drive_native_truth.get("drive_native_truth_valid", False)):
                drive_native_truth_valid_axes += 1
            else:
                drive_native_truth_invalid_axes += 1

        logical_joint = None
        if axis_index < len(axis_to_joint_list):
            try:
                mapped_joint = int(axis_to_joint_list[axis_index])
                if mapped_joint >= 0:
                    logical_joint = mapped_joint + 1
            except Exception:
                logical_joint = None

        slave_al_state = coerce_int(axis.get("slave_al_state"), 0)
        slave_online = bool(coerce_int(axis.get("slave_online"), 0))
        slave_operational = bool(coerce_int(axis.get("slave_operational"), 0))
        if slave_online:
            slave_online_axes += 1
        if slave_operational:
            slave_operational_axes += 1
        startup_drive_config = backend_registry.extract_drive_startup_config_axis_for_backend(
            servo_backend,
            dict(axis),
            drive_profile_id=resolved_drive_profile,
        )
        absolute_feedback = backend_registry.normalize_drive_absolute_feedback_for_backend(
            servo_backend,
            axis.get("absolute_feedback"),
            drive_profile_id=resolved_drive_profile,
        )
        if isinstance(startup_drive_config, Mapping):
            if bool(startup_drive_config.get("configured")):
                startup_drive_config_configured_axes += 1
                if bool(startup_drive_config.get("verified")):
                    startup_drive_config_verified_axes += 1
                else:
                    startup_drive_config_mismatch_axes += 1
        if drive_native_ratio_enabled:
            if not isinstance(startup_drive_config, Mapping):
                drive_native_startup_valid = False
                drive_native_startup_reason = "startup_drive_config_missing"
            else:
                missing_setting_keys = startup_drive_config.get("missing_setting_keys", [])
                configured = bool(startup_drive_config.get("configured", False))
                readback_valid = bool(startup_drive_config.get("readback_valid", False))
                verified = bool(startup_drive_config.get("verified", False))
                if isinstance(missing_setting_keys, list) and missing_setting_keys:
                    drive_native_startup_valid = False
                    drive_native_startup_reason = "startup_drive_config_missing_required_settings"
                elif not configured:
                    drive_native_startup_valid = False
                    drive_native_startup_reason = "startup_drive_config_unconfigured"
                elif not readback_valid:
                    drive_native_startup_valid = False
                    drive_native_startup_reason = "startup_drive_config_unverified"
                elif not verified:
                    drive_native_startup_valid = False
                    drive_native_startup_reason = "startup_drive_config_mismatch"
                else:
                    drive_native_startup_valid = True
                    drive_native_startup_reason = "verified"
            if drive_native_startup_valid:
                drive_native_startup_valid_axes += 1
            else:
                drive_native_startup_invalid_axes += 1
        else:
            drive_native_startup_valid = False
            drive_native_startup_reason = "profile_disabled"
        slave_al_detail = backend_registry.decode_fieldbus_state_for_backend(
            servo_backend,
            slave_al_state,
            fieldbus_profile_id=resolved_fieldbus_profile,
        )
        # Phase 2 (2026-04-20): merge the extended A6-EC diagnostic
        # PDO fields from the per-axis context if provided. Fields that
        # are absent from the context stay absent from the payload so
        # consumers can distinguish "extended PDO not present" from
        # "extended PDO reports zero".
        extended_axis_extras: dict[str, Any] = {}
        if axis_extended_telemetry_context is not None:
            extended_override = axis_extended_telemetry_context.get(axis_index)
            if isinstance(extended_override, Mapping):
                for key in (
                    "bus_voltage_v",
                    "load_rate_pct",
                    "igbt_temp_c",
                    "motor_temp_c",
                    "position_error_counts",
                    "drive_not_ready_bits",
                    "drive_not_ready_text",
                    "motor_not_rotating_code",
                    "motor_not_rotating_text",
                ):
                    if key in extended_override:
                        extended_axis_extras[key] = extended_override[key]

        axes_payload.append(
            {
                "axis": axis_index,
                "logical_joint": logical_joint,
                "drive_state": ds402_state,
                "ds402_state": ds402_state,
                "statusword": statusword,
                "statusword_hex": f"0x{statusword & 0xFFFF:04x}",
                "error_code": error_code,
                "error_code_hex": f"0x{error_code & 0xFFFF:04x}",
                "manufacturer_error_code": manufacturer_error_code,
                "manufacturer_error_code_hex": f"0x{manufacturer_error_code & 0xFFFFFFFF:08x}",
                "startup_drive_config": startup_drive_config,
                "slave_online": slave_online,
                "slave_operational": slave_operational,
                "slave_al_state": slave_al_state,
                **extended_axis_extras,
                "slave_al_state_name": (
                    str(slave_al_detail.get("name", "UNKNOWN"))
                    if isinstance(slave_al_detail, dict)
                    else "UNKNOWN"
                ),
                "pos_counts": coerce_int(axis.get("pos_counts"), 0),
                "absolute_feedback": absolute_feedback,
                "position_semantics_source": position_semantics_source,
                "drive_native_ratio_enabled": drive_native_ratio_enabled,
                "drive_native_startup_valid": drive_native_startup_valid,
                "drive_native_startup_reason": drive_native_startup_reason,
                "drive_native_truth_valid": bool(drive_native_truth["drive_native_truth_valid"]),
                "drive_native_truth_reason": str(drive_native_truth["drive_native_truth_reason"]),
                "drive_native_truth_signature_valid": bool(
                    drive_native_truth["drive_native_truth_signature_valid"]
                ),
                "drive_native_truth_verification_source": str(
                    drive_native_truth["drive_native_truth_verification_source"]
                ),
                "coordinate_system_valid": bool(drive_native_truth["coordinate_system_valid"]),
                "native_home_state": int(native_home_status["native_home_state"]),
                "native_home_state_name": str(native_home_status["native_home_state_name"]),
                "native_home_active": bool((native_home_active_axis_mask & (1 << axis_index)) != 0),
                "native_home_position_offset": coerce_int(axis.get("native_home_position_offset"), 0),
                "native_home_last_abort_code": int(native_home_status["native_home_last_abort_code"]),
                "native_home_last_abort_code_hex": str(
                    native_home_status["native_home_last_abort_code_hex"]
                ),
                "native_home_state_reported": int(native_home_status["native_home_state_reported"]),
                "native_home_state_reported_name": str(
                    native_home_status["native_home_state_reported_name"]
                ),
                "native_home_last_abort_code_reported": int(
                    native_home_status["native_home_last_abort_code_reported"]
                ),
                "native_home_last_abort_code_reported_hex": str(
                    native_home_status["native_home_last_abort_code_reported_hex"]
                ),
                "native_home_verification_source": str(
                    native_home_status["native_home_verification_source"]
                ),
                "fault": backend_registry.describe_drive_fault_code_for_backend(
                    servo_backend,
                    error_code,
                    drive_profile_id=resolved_drive_profile,
                ),
                "manufacturer_fault": backend_registry.describe_drive_manufacturer_fault_code_for_backend(
                    servo_backend,
                    manufacturer_error_code,
                    drive_profile_id=resolved_drive_profile,
                ),
                "encoder_retention_fault": (
                    dict(encoder_retention_fault_detail)
                    if isinstance(encoder_retention_fault_detail, Mapping)
                    and encoder_retention_fault_detail.get("present", False)
                    else None
                ),
                "encoder_retention_fault_present": bool(encoder_retention_fault_present),
            }
        )

    if faulted_axes > 0:
        physical_state = "FAULTED"
    elif op_enabled_axes > 0:
        physical_state = "ACTIVE"
    elif link_up and responding > 0:
        physical_state = "BUS_UP_DISARMED"
    else:
        physical_state = "INACTIVE"

    if faulted_axes > 0:
        driver_state = "FAULTED"
    elif op_enabled_axes > 0:
        driver_state = "ACTIVE"
    elif link_up and responding > 0:
        driver_state = "DISARMED"
    else:
        driver_state = "INACTIVE"

    ethercat_master_state = backend_registry.describe_fieldbus_master_state_for_backend(
        servo_backend,
        link_up=link_up,
        responding=responding,
        operational=operational,
        num_axes=num_axes,
        fieldbus_profile_id=resolved_fieldbus_profile,
    ) or "DOWN"

    return {
        "servo_backend": str(servo_backend).strip() or None,
        "drive_profile": resolved_drive_profile,
        "configured_drive_profile": (
            backend_registry.resolve_drive_profile_for_backend(
                servo_backend,
                drive_profile_id=configured_profile,
            )
            if configured_profile
            else None
        ),
        "live_drive_profile": (
            backend_registry.resolve_drive_profile_for_backend(
                servo_backend,
                drive_profile_id=live_drive_profile,
            )
            if live_drive_profile
            else None
        ),
        "drive_profile_source": drive_profile_source,
        "fieldbus_profile": resolved_fieldbus_profile,
        "reference": reference,
        "physical_state": physical_state,
        "driver_state": driver_state,
        "ethercat_master_state": ethercat_master_state,
        "rtcore_state": "UP" if socket_present else "UNKNOWN",
        "armed": armed,
        "axis_enable_mask": enable_mask,
        "axis_enable_mask_hex": f"0x{enable_mask:x}",
        "native_home_active_axis_mask": native_home_active_axis_mask,
        "native_home_active_axis_mask_hex": f"0x{native_home_active_axis_mask:x}",
        "enable_requested": enable_requested,
        "requested_axes": requested_axes,
        "op_enabled_axes": op_enabled_axes,
        "num_axes": num_axes,
        "faulted_axes": faulted_axes,
        "statusword_feedback_axes": statusword_feedback_axes,
        "slave_online_axes": slave_online_axes,
        "slave_operational_axes": slave_operational_axes,
        "drive_native_ratio_enabled": drive_native_ratio_enabled,
        "position_semantics_source": position_semantics_source,
        "drive_native_startup_valid": (
            drive_native_ratio_enabled
            and num_axes > 0
            and drive_native_startup_invalid_axes == 0
            and drive_native_startup_valid_axes == num_axes
        ),
        "drive_native_startup_valid_axes": drive_native_startup_valid_axes,
        "drive_native_startup_invalid_axes": drive_native_startup_invalid_axes,
        "drive_native_truth_valid": (
            drive_native_ratio_enabled
            and num_axes > 0
            and drive_native_startup_invalid_axes == 0
            and drive_native_truth_invalid_axes == 0
            and drive_native_truth_valid_axes == num_axes
        ),
        "drive_native_truth_valid_axes": drive_native_truth_valid_axes,
        "drive_native_truth_invalid_axes": drive_native_truth_invalid_axes,
        "coordinate_system_valid_axes": coordinate_system_valid_axes,
        "link_up": link_up,
        "responding": responding,
        "online": online,
        "operational": operational,
        "startup_ready": startup_ready,
        "startup_drive_config_configured_axes": startup_drive_config_configured_axes,
        "startup_drive_config_verified_axes": startup_drive_config_verified_axes,
        "startup_drive_config_mismatch_axes": startup_drive_config_mismatch_axes,
        "wkc_actual": wkc_actual,
        "wkc_expected": wkc_expected,
        "master_al": master_al,
        "master_al_hex": f"0x{master_al:x}",
        "master_al_name": (
            str(master_al_detail.get("name", "UNKNOWN"))
            if isinstance(master_al_detail, dict)
            else "UNKNOWN"
        ),
        "axes": axes_payload,
    }
