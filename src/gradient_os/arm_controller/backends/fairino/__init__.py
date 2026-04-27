# backends/fairino/__init__.py
#
# Fairino FR-series ActuatorBackend (FR10 in particular).
# Read-only at Milestone 1: connects to the FR10 over its Python SDK and
# streams joint state into GradientOS for UI display. Motion commands land
# in Milestone 2.
#
# Layout:
# - backend.py       : FairinoBackend(ActuatorBackend) — the integration point
# - rpc_client.py    : quarantined Fairino SDK calls
# - state_stream.py  : daemon-thread state poller + thread-safe snapshot cache
# - frame_guard.py   : Option B coordinate-strategy enforcement
# - config.py        : network endpoints, poll rates, env-var overrides

from .backend import FairinoBackend
from .config import FairinoBackendConfig
from .frame_guard import FrameStrategyViolation
from .rpc_client import FairinoRPCClient, FairinoSDKAdapter, FairinoStatusFlags
from .state_stream import FairinoStateStream, StateSnapshot

__all__ = [
    "FairinoBackend",
    "FairinoBackendConfig",
    "FairinoRPCClient",
    "FairinoSDKAdapter",
    "FairinoStateStream",
    "FairinoStatusFlags",
    "FrameStrategyViolation",
    "StateSnapshot",
]
