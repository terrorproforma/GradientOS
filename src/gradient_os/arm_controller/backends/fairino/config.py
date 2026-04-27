# backends/fairino/config.py
#
# Configuration constants and environment-variable overrides for the Fairino
# FR-series ActuatorBackend. The Fairino backend does not own servo register
# layouts the way feetech/config.py does — the FR10 controller owns its own
# servos. What we configure here is the *network client* targeting the FR10:
# IP address, ports, poll rates, lifecycle defaults.
#
# MILESTONE 3 PROMOTION: when END_EFFECTOR_OFFSET is moved off the global in
# `gradient_os.ik_solver` and onto RobotConfig, the active tool/wobj indices
# we read from the FR10 should be plumbed through this config too — they are
# session-state, not constants.

from __future__ import annotations

import os
from dataclasses import dataclass


# Default Fairino controller endpoint. The FR10 ships at this IP and these
# ports out of the box; both are documented in FR10_BRASS_TACKS_TOUR.md.
DEFAULT_CONTROLLER_IP = "192.168.58.2"
DEFAULT_RPC_PORT = 20003       # commands (XML-RPC over TCP)
DEFAULT_STATE_PORT = 20004     # state stream

# State-stream poll rate. The FR10 publishes at 1–125 Hz; 50 Hz is plenty for
# UI display and well within the controller's capability. Lower if CPU on the
# control PC is constrained; higher only if you need it (Milestone 4+).
DEFAULT_STATE_POLL_HZ = 50.0

# Connect timeout for the initial RPC handshake. Beyond this, initialize()
# returns False and the controller stays up in degraded mode (matches the
# ethercat_rtcore pattern).
DEFAULT_CONNECT_TIMEOUT_S = 3.0

# Maximum age of a state snapshot we'll trust when callers ask for joint
# positions. If the stream stalls for longer than this, get_joint_positions()
# logs a warning and returns the last known value (rather than blocking or
# raising — the UI must keep rendering).
DEFAULT_STALE_SNAPSHOT_S = 1.0

# FR10 has 6 joints. This is hardcoded because every FR-series arm shares it,
# and the value is also enforced via robots/fr10/config.py::num_logical_joints.
FR10_NUM_JOINTS = 6


# =============================================================================
# Legacy registry shim constants
# =============================================================================
# The registry.get_*() helpers (encoder_resolution, default_pid_gains,
# default_baud_rate, telemetry blocks, …) expect every backend's config
# module to expose a fixed set of module-level names. They were designed for
# serial-servo protocols (Feetech). The Fairino backend has none of that —
# the FR10 controller owns its own servos, encoders, PID, telemetry. We
# expose zero placeholders so the helpers don't AttributeError, mirroring
# ethercat_rtcore/config.py. None of these values are meaningful; do not
# use them. RobotConfig.default_pid_gains and friends should be overridden
# in robots/fr10/config.py to avoid hitting this path at all.

SERVO_PROTOCOL_SUPPORTED = False  # tell legacy utils to skip serial setup

DEFAULT_BAUD_RATE = 0
SERIAL_READ_TIMEOUT = 0.0

SERVO_VALUE_MIN = 0
SERVO_VALUE_MAX = 0
SERVO_VALUE_CENTER = 0

DEFAULT_KP = 0
DEFAULT_KI = 0
DEFAULT_KD = 0

# Telemetry blocks: not applicable. State arrives via FairinoStateStream, not
# via packed register reads.
TELEMETRY_BLOCK1_ADDRESS = 0
TELEMETRY_BLOCK1_LENGTH = 0
TELEMETRY_BLOCK2_ADDRESS = 0
TELEMETRY_BLOCK2_LENGTH = 0
TELEMETRY_BLOCK3_ADDRESS = 0
TELEMETRY_BLOCK3_LENGTH = 0


def parse_telemetry_block1(_data: bytes) -> dict:
    return {}


def parse_telemetry_block2(_data: bytes) -> dict:
    return {}


def parse_telemetry_block3(_data: bytes) -> dict:
    return {}


@dataclass(frozen=True)
class FairinoBackendConfig:
    """Resolved network/runtime config for one FairinoBackend instance.

    Built once at backend construction. All fields are overridable via
    environment variables so a developer can point at a staging FR10 or a
    SDK-mock test rig without editing code.
    """

    controller_ip: str = DEFAULT_CONTROLLER_IP
    rpc_port: int = DEFAULT_RPC_PORT
    state_port: int = DEFAULT_STATE_PORT
    state_poll_hz: float = DEFAULT_STATE_POLL_HZ
    connect_timeout_s: float = DEFAULT_CONNECT_TIMEOUT_S
    stale_snapshot_s: float = DEFAULT_STALE_SNAPSHOT_S

    @classmethod
    def from_env(cls) -> "FairinoBackendConfig":
        """Build a config, letting env vars override the defaults.

        Env vars (all optional):
        - GRADIENT_FAIRINO_IP
        - GRADIENT_FAIRINO_RPC_PORT
        - GRADIENT_FAIRINO_STATE_PORT
        - GRADIENT_FAIRINO_POLL_HZ
        - GRADIENT_FAIRINO_CONNECT_TIMEOUT_S
        - GRADIENT_FAIRINO_STALE_SNAPSHOT_S
        """

        def _str(name: str, default: str) -> str:
            return os.environ.get(name, default)

        def _int(name: str, default: int) -> int:
            raw = os.environ.get(name)
            if raw is None or raw == "":
                return default
            try:
                return int(raw)
            except ValueError:
                print(f"[Fairino config] WARNING: {name}={raw!r} is not an int; using {default}")
                return default

        def _float(name: str, default: float) -> float:
            raw = os.environ.get(name)
            if raw is None or raw == "":
                return default
            try:
                return float(raw)
            except ValueError:
                print(f"[Fairino config] WARNING: {name}={raw!r} is not a float; using {default}")
                return default

        return cls(
            controller_ip=_str("GRADIENT_FAIRINO_IP", DEFAULT_CONTROLLER_IP),
            rpc_port=_int("GRADIENT_FAIRINO_RPC_PORT", DEFAULT_RPC_PORT),
            state_port=_int("GRADIENT_FAIRINO_STATE_PORT", DEFAULT_STATE_PORT),
            state_poll_hz=_float("GRADIENT_FAIRINO_POLL_HZ", DEFAULT_STATE_POLL_HZ),
            connect_timeout_s=_float(
                "GRADIENT_FAIRINO_CONNECT_TIMEOUT_S", DEFAULT_CONNECT_TIMEOUT_S
            ),
            stale_snapshot_s=_float(
                "GRADIENT_FAIRINO_STALE_SNAPSHOT_S", DEFAULT_STALE_SNAPSHOT_S
            ),
        )
