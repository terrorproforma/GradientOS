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
                {"semantic": "err", "index": 0x603F, "subindex": 0x00, "bits": 16},
                {"semantic": "sw", "index": 0x6041, "subindex": 0x00, "bits": 16},
                {"semantic": "pos", "index": 0x6064, "subindex": 0x00, "bits": 32},
                {"semantic": "torque", "index": 0x6077, "subindex": 0x00, "bits": 16},
                {"semantic": "mode_disp", "index": 0x6061, "subindex": 0x00, "bits": 8},
                {"semantic": "tp_status", "index": 0x60B9, "subindex": 0x00, "bits": 16},
                {"semantic": "tp_pos1", "index": 0x60BA, "subindex": 0x00, "bits": 32},
                {"semantic": "tp_pos2", "index": 0x60BC, "subindex": 0x00, "bits": 32},
                {"semantic": "di", "index": 0x60FD, "subindex": 0x00, "bits": 32},
            ],
        },
        "startup_defaults": {
            "a6ec_encoder_position_tracking_mode": 1,
        },
        "startup_schema": {
            "a6ec_encoder_position_tracking_mode": {
                "type": "u16",
                "min": 0,
                "max": 5,
                "label": "A6-EC encoder position tracking mode",
                "object": {"index": 0x2000, "subindex": 0x08},
                "env_var": "GRADIENT_RT_DRIVE_STARTUP_SDO_CONFIG",
            }
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
