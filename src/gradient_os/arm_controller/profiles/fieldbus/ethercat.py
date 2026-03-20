from __future__ import annotations

from typing import Any

PROFILE_ID = "ethercat_al"
PROFILE_LABEL = "EtherCAT AL state machine"

_AL_STATE_NAMES = {
    0x01: "INIT",
    0x02: "PREOP",
    0x04: "SAFEOP",
    0x08: "OP",
}


def decode_al_state(value: int) -> dict[str, Any]:
    state = int(value) & 0xFF
    name = _AL_STATE_NAMES.get(state, "UNKNOWN")
    return {
        "profile_id": PROFILE_ID,
        "label": PROFILE_LABEL,
        "decoded": state in _AL_STATE_NAMES,
        "state": state,
        "state_hex": f"0x{state:02x}",
        "name": name,
    }


def describe_master_state(
    *,
    link_up: int,
    responding: int,
    operational: int,
    num_axes: int,
) -> str:
    if int(link_up) and int(responding) > 0:
        return "OP" if int(num_axes) > 0 and int(operational) >= int(num_axes) else "BUS_UP"
    return "DOWN"
