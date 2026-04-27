# backends/fairino/rpc_client.py
#
# Quarantine for every Fairino Python SDK call. Nothing else in GradientOS
# should `import fairino` — only this file. Two reasons:
#
#  1. The Fairino SDK is a third-party dependency we don't yet have a copy of
#     locally to verify against. Method names ("RPC", "GetActualJointPosDegree",
#     "Connect", etc.) are taken from FR10_BRASS_TACKS_TOUR.md and Fairino's
#     public docs but must be confirmed once the SDK is pip-installed. By
#     keeping the SDK contact surface in one place, that confirmation is a
#     small diff against TODO markers below — not a hunt across the codebase.
#
#  2. Tests must run without the SDK installed. FairinoRPCClient takes a
#     `_sdk_factory` argument (defaults to the real `_RealFairinoSDK`) so a
#     fake adapter can be injected. Every method on the adapter is a thin
#     pass-through; the unit test verifies our usage, not the SDK.
#
# Joint-angle convention boundary lives here too: the Fairino SDK speaks
# degrees (FR10_INTEGRATION_GUIDE.md), GradientOS's ActuatorBackend speaks
# radians. Conversion happens at this layer so the rest of the backend never
# has to think about it.

from __future__ import annotations

import math
import threading
from dataclasses import dataclass, field
from typing import Callable, Optional, Protocol


@dataclass
class FairinoStatusFlags:
    """Status bits we surface to the rest of GradientOS for UI / alerts.

    All booleans default to False, which is also the "we haven't heard from
    the FR10 yet" state — a fresh FairinoRPCClient before connect() reports
    everything as not-faulted, and consumers should check `is_connected`
    rather than inferring from these flags alone.
    """

    in_collision: bool = False
    estopped: bool = False
    faulted: bool = False
    program_state: str = ""        # FR10's high-level mode string (free-form)
    last_error_code: int = 0       # 0 == OK by Fairino convention


class FairinoSDKAdapter(Protocol):
    """The narrow contract this backend needs from the Fairino SDK.

    Built as a Protocol so we can satisfy it with either the real SDK
    wrapper (`_RealFairinoSDK`) or an in-test fake. Keep this surface tiny —
    every method that lands here is one more SDK assumption to verify.
    """

    def connect(self, ip: str, timeout_s: float) -> None: ...
    def disconnect(self) -> None: ...
    def is_connected(self) -> bool: ...
    def read_joint_positions_deg(self) -> list[float]: ...
    def read_status_flags(self) -> FairinoStatusFlags: ...


class _RealFairinoSDK:
    """Production adapter: delegates to the real `fairino` Python SDK.

    Each method wraps one (or two) actual SDK calls. The exact call names
    are TODOs because we don't have the SDK in hand to verify; the FR10
    docs hint at the surface but don't show signatures. When the SDK is
    pip-installed, walk through each TODO below and replace the stub with
    the real call. **Keep all SDK-specific logic in this class.**
    """

    def __init__(self) -> None:
        # Lazy-imported in connect() so that:
        # 1) tests don't need the SDK installed;
        # 2) module import of this file (e.g. for type hints) doesn't fail
        #    on a system where fairino isn't available.
        self._sdk = None  # type: ignore[assignment]
        self._connected = False

    def connect(self, ip: str, timeout_s: float) -> None:
        # TODO(SDK): verify the import path. The Fairino docs reference both
        #   `from fairino import Robot` and `Robot.RPC(ip)` and `from fairino
        #   import RPC`. Pick whichever the installed wheel actually ships.
        try:
            from fairino import Robot  # type: ignore[import-not-found]
        except ImportError as e:
            raise RuntimeError(
                "Fairino Python SDK not installed. Install with "
                "`pip install -e '.[fairino]'` (or fix the dependency once "
                "the exact PyPI name is confirmed)."
            ) from e

        # TODO(SDK): confirm constructor — does Robot.RPC(ip) auto-connect
        #   and raise on failure, or return a status code? Some Fairino
        #   versions return (error_code, handle) tuples. The `timeout_s`
        #   value may need to be passed via a separate config call.
        self._sdk = Robot.RPC(ip)
        self._connected = True

    def disconnect(self) -> None:
        if self._sdk is None:
            return
        # TODO(SDK): confirm disconnect call. May be `CloseRPC()`,
        #   `LogOut()`, `Disconnect()`, or simply releasing the object.
        try:
            close = getattr(self._sdk, "CloseRPC", None)
            if callable(close):
                close()
        finally:
            self._sdk = None
            self._connected = False

    def is_connected(self) -> bool:
        return self._connected and self._sdk is not None

    def read_joint_positions_deg(self) -> list[float]:
        if self._sdk is None:
            raise RuntimeError("Fairino SDK not connected")
        # TODO(SDK): confirm this returns (error_code, [j1..j6]) tuple as
        #   degrees, with error_code==0 on success. The `0` arg (block flag
        #   vs. cached) varies by SDK version — verify against the wheel.
        error_code, joints = self._sdk.GetActualJointPosDegree(0)
        if error_code != 0:
            raise RuntimeError(
                f"FR10 GetActualJointPosDegree failed: code={error_code}"
            )
        if len(joints) < 6:
            raise RuntimeError(
                f"FR10 returned {len(joints)} joints, expected ≥6"
            )
        return [float(j) for j in joints[:6]]

    def read_status_flags(self) -> FairinoStatusFlags:
        if self._sdk is None:
            raise RuntimeError("Fairino SDK not connected")
        # TODO(SDK): the FR10 surfaces robot state via several methods. Most
        #   likely candidates: GetRobotErrcode(), GetRobotState(),
        #   GetRobotMotionDone(). The exact set of fields depends on the
        #   SDK version. For Milestone 1 we only need fault/estop/collision
        #   bits for the UI; everything else is decoration.
        flags = FairinoStatusFlags()
        try:
            err_code = self._sdk.GetRobotErrcode()
            if isinstance(err_code, tuple):
                # Some SDK methods return (return_code, value).
                err_code = err_code[-1]
            flags.last_error_code = int(err_code)
            flags.faulted = flags.last_error_code != 0
        except AttributeError:
            pass  # Method name unverified — leave defaults.
        return flags


class FairinoRPCClient:
    """Thread-safe lifecycle wrapper around a single Fairino SDK adapter.

    Owns the connect/disconnect transitions and serializes adapter calls
    behind a lock. Multiple state-stream pollers and the backend's own
    on-demand reads can share one instance.

    Inject `_sdk_factory=lambda: FakeAdapter()` in tests to bypass the real
    SDK entirely.
    """

    def __init__(
        self,
        ip: str,
        connect_timeout_s: float,
        _sdk_factory: Optional[Callable[[], FairinoSDKAdapter]] = None,
    ) -> None:
        self._ip = ip
        self._connect_timeout_s = connect_timeout_s
        self._sdk_factory = _sdk_factory or _RealFairinoSDK
        self._sdk: Optional[FairinoSDKAdapter] = None
        self._lock = threading.Lock()

    def connect(self) -> None:
        """Connect to the FR10 controller. Idempotent."""
        with self._lock:
            if self._sdk is not None and self._sdk.is_connected():
                return
            sdk = self._sdk_factory()
            sdk.connect(self._ip, self._connect_timeout_s)
            self._sdk = sdk

    def disconnect(self) -> None:
        """Disconnect from the FR10 controller. Idempotent and never raises."""
        with self._lock:
            if self._sdk is None:
                return
            try:
                self._sdk.disconnect()
            except Exception as e:
                print(f"[Fairino RPC] disconnect raised: {e}")
            finally:
                self._sdk = None

    @property
    def is_connected(self) -> bool:
        with self._lock:
            return self._sdk is not None and self._sdk.is_connected()

    def read_joint_positions_rad(self) -> list[float]:
        """Read the current 6 joint angles, converted from degrees to radians.

        Conversion is the SDK boundary — every layer above this gets radians,
        matching the rest of GradientOS.
        """
        with self._lock:
            if self._sdk is None:
                raise RuntimeError("FairinoRPCClient not connected")
            joints_deg = self._sdk.read_joint_positions_deg()
        return [math.radians(j) for j in joints_deg]

    def read_status_flags(self) -> FairinoStatusFlags:
        with self._lock:
            if self._sdk is None:
                raise RuntimeError("FairinoRPCClient not connected")
            return self._sdk.read_status_flags()
