from __future__ import annotations

from collections.abc import Mapping


def _coerce_int(value: object, default: int = 0) -> int:
    try:
        return int(value)
    except Exception:
        return default


def _journal_signatures(recent_log: str) -> list[str]:
    haystack = str(recent_log or "").lower()
    signatures: list[str] = []
    if "failed to reserve master: device or resource busy" in haystack:
        signatures.append("master_device_busy")
    if "error: ecrt_request_master(0) failed" in haystack:
        signatures.append("request_master_failed")
    if "found left-over process" in haystack:
        signatures.append("leftover_process")
    if "state 'stop-sigterm' timed out" in haystack:
        signatures.append("stop_sigterm_timeout")
    if "processes still around after sigkill" in haystack:
        signatures.append("sigkill_survivor")
    if "remains running after unit stopped" in haystack:
        signatures.append("unit_process_survived_stop")
    return signatures


def build_rtcore_startup_recovery_plan(
    probe: Mapping[str, object] | None,
    *,
    recent_log: str = "",
    recovery_attempted: bool = False,
) -> dict[str, object]:
    data = probe if isinstance(probe, Mapping) else {}

    num_axes = max(0, _coerce_int(data.get("num_axes"), 0))
    responding = max(0, _coerce_int(data.get("responding"), 0))
    online = max(0, _coerce_int(data.get("online"), 0))
    operational = max(0, _coerce_int(data.get("operational"), 0))
    startup_ready = _coerce_int(data.get("startup_ready"), 0)

    physical_state = str(data.get("physical_state", "")).strip().upper() or "UNKNOWN"
    driver_state = str(data.get("driver_state", "")).strip().upper() or "UNKNOWN"
    ethercat_master_state = str(data.get("ethercat_master_state", "")).strip().upper() or "UNKNOWN"
    rtcore_state = str(data.get("rtcore_state", "")).strip().upper() or "UNKNOWN"

    healthy = (
        num_axes > 0
        and startup_ready == 1
        and responding >= num_axes
        and online >= num_axes
        and operational >= num_axes
    )

    signatures = _journal_signatures(recent_log)
    master_busy_seen = (
        "master_device_busy" in signatures or "request_master_failed" in signatures
    )
    stale_owner_seen = any(
        signature in signatures
        for signature in (
            "leftover_process",
            "stop_sigterm_timeout",
            "sigkill_survivor",
            "unit_process_survived_stop",
        )
    )

    rtcore_up_master_down = (
        rtcore_state == "UP"
        and ethercat_master_state == "DOWN"
        and startup_ready == 0
        and physical_state == "INACTIVE"
        and driver_state == "INACTIVE"
    )

    should_recover = False
    reboot_required = False
    reason = "no_action"
    detail = "No startup recovery action is required."

    if healthy:
        reason = "healthy"
        detail = "RTCore reports a healthy operational bus."
    elif rtcore_up_master_down and master_busy_seen:
        if recovery_attempted:
            reboot_required = True
            reason = "master_busy_after_recovery"
            detail = (
                "RTCore is still up with EtherCAT DOWN after one recycle attempt, and the recent "
                "journal shows EtherCAT master reservation failure. This usually means a stale "
                "owner survived stop/kill and a host reboot is required."
            )
        else:
            should_recover = True
            reason = "master_busy_before_recovery"
            detail = (
                "RTCore is up but EtherCAT is DOWN, and the recent journal shows EtherCAT master "
                "reservation failure. Attempt one hard RTCore/EtherCAT recycle before escalating."
            )
    elif rtcore_up_master_down:
        if not recovery_attempted:
            should_recover = True
            reason = "rtcore_up_master_down"
            detail = (
                "RTCore is up but EtherCAT remains DOWN before controller launch. Attempt one hard "
                "RTCore/EtherCAT recycle before continuing startup."
            )
        else:
            reason = "rtcore_up_master_down_after_recovery"
            detail = (
                "RTCore still reports UP with EtherCAT DOWN after one recycle attempt, but no "
                "explicit master-busy signature was found. Continue with normal readiness checks."
            )
    elif rtcore_state != "UP" and physical_state == "INACTIVE" and not recovery_attempted:
        should_recover = True
        reason = "rtcore_not_up"
        detail = (
            "RTCore does not appear healthy after sync. Attempt one hard RTCore/EtherCAT recycle "
            "before controller launch."
        )
    elif recovery_attempted and stale_owner_seen and master_busy_seen:
        reboot_required = True
        reason = "stale_owner_after_recovery"
        detail = (
            "The recent journal shows a stale RTCore owner survived stop/kill and the EtherCAT "
            "master is still busy after one recycle attempt. A host reboot is required."
        )

    return {
        "healthy": healthy,
        "should_recover": should_recover,
        "reboot_required": reboot_required,
        "reason": reason,
        "detail": detail,
        "journal_signatures": signatures,
        "probe_summary": {
            "physical_state": physical_state,
            "driver_state": driver_state,
            "ethercat_master_state": ethercat_master_state,
            "rtcore_state": rtcore_state,
            "startup_ready": startup_ready,
            "responding": responding,
            "online": online,
            "operational": operational,
            "num_axes": num_axes,
        },
    }
