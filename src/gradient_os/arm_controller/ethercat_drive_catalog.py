from __future__ import annotations

from copy import deepcopy
from typing import Any

ETHERCAT_DRIVE_CATALOG: dict[str, dict[str, Any]] = {
    "a6ec_ds402": {
        "profile_id": "a6ec_ds402",
        "label": "StepperOnline / Leadshine A6-EC over CiA402",
        "fieldbus_profile_id": "ethercat_al",
        "rtcore": {
            "vendor_id": 0x00400000,
            "product_code": 0x00000715,
            "revision_no": 0x00002EF8,
            "rx_sync_index": 2,
            "tx_sync_index": 3,
            "dc_cycle_multiple_ns": 250000,
            "rx_pdo": 0x1702,
            "tx_pdo": 0x1B02,
            "rx_pdo_layout": [
                {"semantic": "cw", "index": 0x6040, "subindex": 0x00, "bits": 16},
                {"semantic": "target_pos", "index": 0x607A, "subindex": 0x00, "bits": 32},
                {"semantic": "target_vel", "index": 0x60FF, "subindex": 0x00, "bits": 32},
                {"semantic": "target_torque", "index": 0x6071, "subindex": 0x00, "bits": 16},
                {"semantic": "mode", "index": 0x6060, "subindex": 0x00, "bits": 8},
                {"semantic": "tp_func", "index": 0x60B8, "subindex": 0x00, "bits": 16},
                {"semantic": "max_profile_vel", "index": 0x607F, "subindex": 0x00, "bits": 32},
            ],
            "tx_pdo_layout": [
                # Phase 1 final layout (2026-04-21): 6 entries / 17 bytes.
                #
                # The path to atomic multi-turn via TxPDO is blocked by
                # A6-EC firmware: although the ESI declares the 0x2040
                # subitems as `<PdoMapping>t</PdoMapping>`, the drive
                # ACCEPTS the PDO assignment (0x1B02 shows 18 entries in
                # `ethercat pdos`) but REFUSES to populate the U40.20/
                # U40.22 bytes cyclically — SDO reads correctly return
                # the multi-turn value while the PDO bytes stay at 0.
                # Verified live 2026-04-21 with an 8-entry / 23 B custom
                # mapping: statusword transmitted at 0x1650, multi-turn
                # bytes at 0x00000000.
                #
                # The Phase 1 goal (eliminate the 200 ms shaft-frame
                # skew flicker) is now delivered by a different path:
                # RTCore atomically latches `axis_pos_counts[i]` at the
                # moment of the SDO upload of U40.20/U40.22 into the
                # `paired_pos_counts` field of `AbsoluteFeedbackAxis`.
                # The canonical-truth shaft-frame gate compares two
                # values from the SAME moment (see `perform_absolute_
                # feedback_refresh` in main.cpp), bounded by the
                # mailbox transit (~1-5 ms) instead of the 200 ms poll
                # period.
                #
                # The touch-probe TxPDO feedback entries
                # (tp_status/tp_pos1/tp_pos2 at 0x60B9/0x60BA/0x60BC)
                # stay removed because RTCore declared but never read
                # them — dead weight that only taxed SM3 capacity.
                {"semantic": "err", "index": 0x603F, "subindex": 0x00, "bits": 16},
                {"semantic": "sw", "index": 0x6041, "subindex": 0x00, "bits": 16},
                {"semantic": "pos", "index": 0x6064, "subindex": 0x00, "bits": 32},
                {"semantic": "torque", "index": 0x6077, "subindex": 0x00, "bits": 16},
                {"semantic": "mode_disp", "index": 0x6061, "subindex": 0x00, "bits": 8},
                {"semantic": "di", "index": 0x60FD, "subindex": 0x00, "bits": 32},
                # Symptom: the A6-EC drive's SM3 sync manager is sized
                # for the original ~25-byte TxPDO. Appending the 9
                # extended entries (22 more bytes) made the TxPDO too
                # large; `ethercat pdos` listed all 17 accepted
                # declarations but the live domain buffer transmitted
                # 0x0000 for every TxPDO byte. SDO upload of 0x6041
                # returned the correct 0x1650 in parallel, so the drive
                # itself was fine but the cyclic frame was blown.
                #
                # Atomic multi-turn and extended telemetry need to land
                # via a different mechanism: either a second TxPDO slot
                # (e.g. map `multi_turn_lo/hi` into a spare 0x1A0X /
                # 0x1B0X slot that SM3 can still fit) or bump SM3 size
                # via an explicit 0x1C13 re-assign. Tracked as a
                # follow-up under the canonical-truth stability
                # workstream; Phase 0 and Phase 3 arm-time strict check
                # still work without extended PDO (they fall back to the
                # SDO 5 Hz poll for multi-turn).
                #
                # Re-introducing entries here MUST be preceded by a live
                # check that `cat /run/gradient-rt-motion/metrics.json`
                # reports non-zero statusword on every axis at bus
                # power-up, and `ethercat data` shows real bytes in the
                # TxPDO region of each axis stride.
            ],
        },
        "startup_defaults": {
            "a6ec_encoder_position_tracking_mode": 4,
            "a6ec_rotation_mode_gear_ratio_numerator": None,
            "a6ec_rotation_mode_gear_ratio_denominator": None,
            # C10.16 "Reference running mode in rotation mode".
            # 0=Nearest (shortest path), 1=Forward only, 2=Reverse only,
            # 3=Keep current direction, 4=Not specified. Drive NVM defaults
            # to 0, but we pin it explicitly because a non-zero value causes
            # the drive to take the LONG path in rotation mode (C00.07=4)
            # when commanded near the seam, which produces full-shaft 360 deg
            # excursions from tiny jog commands. Leaving this implicit is a
            # safety hazard on larger joints (e.g. J6 with 10:1 gearing can
            # wrap a full joint revolution from a 1 deg jog request).
            "a6ec_rotation_mode_reference_running_direction": 0,
        },
        "startup_schema": {
            "a6ec_encoder_position_tracking_mode": {
                "type": "u16",
                "min": 0,
                "max": 5,
                "label": "A6-EC encoder position tracking mode",
                "object": {"index": 0x2000, "subindex": 0x08},
                "env_var": "GRADIENT_RT_DRIVE_STARTUP_SDO_CONFIG",
            },
            "a6ec_rotation_mode_gear_ratio_numerator": {
                "type": "u16",
                "min": 1,
                "max": 65535,
                "label": "A6-EC rotation-mode gear ratio numerator",
                "object": {"index": 0x2010, "subindex": 0x19},
                "env_var": "GRADIENT_RT_DRIVE_STARTUP_SDO_CONFIG",
            },
            "a6ec_rotation_mode_gear_ratio_denominator": {
                "type": "u16",
                "min": 1,
                "max": 65535,
                "label": "A6-EC rotation-mode gear ratio denominator",
                "object": {"index": 0x2010, "subindex": 0x1A},
                "env_var": "GRADIENT_RT_DRIVE_STARTUP_SDO_CONFIG",
            },
            "a6ec_rotation_mode_reference_running_direction": {
                "type": "u16",
                "min": 0,
                "max": 4,
                "label": "A6-EC rotation-mode reference running direction",
                "object": {"index": 0x2010, "subindex": 0x17},
                "env_var": "GRADIENT_RT_DRIVE_STARTUP_SDO_CONFIG",
            },
        },
    }
}


def normalize_ethercat_drive_profile_id(profile_id: object | None) -> str | None:
    token = str(profile_id or "").strip().lower()
    return token or None


def list_ethercat_drive_profiles() -> list[str]:
    return sorted(ETHERCAT_DRIVE_CATALOG.keys())


def get_ethercat_drive_profile(profile_id: object | None) -> dict[str, Any] | None:
    token = normalize_ethercat_drive_profile_id(profile_id)
    if not token:
        return None
    payload = ETHERCAT_DRIVE_CATALOG.get(token)
    return deepcopy(payload) if isinstance(payload, dict) else None


def get_ethercat_drive_rtcore_config(profile_id: object | None) -> dict[str, Any] | None:
    profile = get_ethercat_drive_profile(profile_id)
    rtcore = profile.get("rtcore") if isinstance(profile, dict) else None
    return deepcopy(rtcore) if isinstance(rtcore, dict) else None


def _render_pdo_layout(entries: list[dict[str, Any]]) -> str:
    rendered: list[str] = []
    for entry in entries:
        if not isinstance(entry, dict):
            continue
        rendered.append(
            f"{entry['semantic']}|0x{int(entry['index']) & 0xFFFF:04X}|0x{int(entry['subindex']) & 0xFF:02X}|{int(entry['bits'])}"
        )
    return ";".join(rendered)


def render_ethercat_drive_rtcore_env(profile_id: object | None) -> dict[str, str]:
    rtcore = get_ethercat_drive_rtcore_config(profile_id)
    if not isinstance(rtcore, dict):
        return {}
    return {
        "GRADIENT_RT_DRIVE_VENDOR_ID": f"0x{int(rtcore.get('vendor_id', 0)) & 0xFFFFFFFF:08X}",
        "GRADIENT_RT_DRIVE_PRODUCT_CODE": f"0x{int(rtcore.get('product_code', 0)) & 0xFFFFFFFF:08X}",
        "GRADIENT_RT_DRIVE_REVISION_NO": f"0x{int(rtcore.get('revision_no', 0)) & 0xFFFFFFFF:08X}",
        "GRADIENT_RT_DRIVE_RX_SYNC_INDEX": str(int(rtcore.get("rx_sync_index", 2))),
        "GRADIENT_RT_DRIVE_TX_SYNC_INDEX": str(int(rtcore.get("tx_sync_index", 3))),
        "GRADIENT_RT_DRIVE_DC_CYCLE_MULTIPLE_NS": str(int(rtcore.get("dc_cycle_multiple_ns", 0))),
        "GRADIENT_RT_DRIVE_RX_PDO": f"0x{int(rtcore.get('rx_pdo', 0)) & 0xFFFF:04X}",
        "GRADIENT_RT_DRIVE_TX_PDO": f"0x{int(rtcore.get('tx_pdo', 0)) & 0xFFFF:04X}",
        "GRADIENT_RT_DRIVE_RX_PDO_LAYOUT": _render_pdo_layout(list(rtcore.get("rx_pdo_layout", []))),
        "GRADIENT_RT_DRIVE_TX_PDO_LAYOUT": _render_pdo_layout(list(rtcore.get("tx_pdo_layout", []))),
    }


def build_default_startup_entries(profile_id: object | None, *, num_axes: int) -> list[dict[str, Any]]:
    profile = get_ethercat_drive_profile(profile_id)
    startup_defaults = profile.get("startup_defaults") if isinstance(profile, dict) else None
    if not isinstance(startup_defaults, dict) or int(num_axes) <= 0:
        return []
    return [deepcopy(startup_defaults) for _ in range(int(num_axes))]
