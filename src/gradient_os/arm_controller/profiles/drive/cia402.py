from __future__ import annotations

from typing import Any

PROFILE_ID = "cia402"
PROFILE_LABEL = "CiA402 statusword state machine"


def decode_statusword(statusword: int) -> dict[str, Any]:
    sw = int(statusword) & 0xFFFF
    state = "Unknown"
    decoded = True
    if (sw & 0x004F) == 0x0000:
        state = "NotReady"
    elif (sw & 0x004F) == 0x0040:
        state = "SwitchOnDisabled"
    elif (sw & 0x006F) == 0x0021:
        state = "ReadyToSwitchOn"
    elif (sw & 0x006F) == 0x0023:
        state = "SwitchedOn"
    elif (sw & 0x006F) == 0x0027:
        state = "OperationEnabled"
    elif (sw & 0x006F) == 0x0007:
        state = "QuickStopActive"
    elif (sw & 0x004F) == 0x000F:
        state = "FaultReactionActive"
    elif (sw & 0x004F) == 0x0008:
        state = "Fault"
    else:
        decoded = False

    return {
        "profile_id": PROFILE_ID,
        "label": PROFILE_LABEL,
        "decoded": decoded,
        "state": state,
        "statusword": sw,
        "statusword_hex": f"0x{sw:04x}",
    }
