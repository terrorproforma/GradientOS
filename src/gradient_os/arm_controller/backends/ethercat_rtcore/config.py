"""
`ethercat_rtcore` backend configuration.

This backend is **not** a serial-servo protocol. It talks to the C++ RT motion
core (`gradient-rt-motion`) over IPC and the RTCore owns EtherCAT + DS402.

Why does a "config module" exist at all?
----------------------------------------
The legacy GradientOS controller boot path expects every backend to provide a
config module so `utils._populate_servo_constants()` can initialize a pile of
serial/packet constants. For EtherCAT this is irrelevant, and we explicitly mark
that here so the legacy initializer can skip.

Contract:
- `SERVO_PROTOCOL_SUPPORTED = False` → do not populate serial protocol constants.
- Provide minimal placeholders for registry helpers that still ask for
  `DEFAULT_BAUD_RATE` / encoder min/max/center (these should not be used for
  EtherCAT motion).
"""

from __future__ import annotations

from typing import Any, Optional

from ...profiles.drive import a6ec_ds402
from ...profiles.fieldbus import ethercat

# Tell the legacy utils initializer to skip serial protocol constants.
SERVO_PROTOCOL_SUPPORTED = False

# Minimal placeholders required by registry helper functions.
# These are intentionally "do not use" values for EtherCAT.
DEFAULT_BAUD_RATE = 0
SERIAL_READ_TIMEOUT = 0.0

SERVO_VALUE_MIN = 0
SERVO_VALUE_MAX = 0
SERVO_VALUE_CENTER = 0

# Placeholders for PID defaults (not used; tuning is a drive commissioning task).
DEFAULT_KP = 0
DEFAULT_KI = 0
DEFAULT_KD = 0


DEFAULT_FIELDBUS_PROFILE_ID = ethercat.PROFILE_ID
DEFAULT_DRIVE_PROFILE_ID = a6ec_ds402.PROFILE_ID
# Backward-compatible alias used by older registry helpers.
DRIVE_FAULT_PROFILE_ID = DEFAULT_DRIVE_PROFILE_ID


def get_drive_fault_reference_metadata() -> dict[str, Any]:
    return a6ec_ds402.get_fault_reference_metadata()


def describe_drive_fault_code(error_code: int) -> Optional[dict[str, Any]]:
    return a6ec_ds402.describe_fault_code(error_code)


