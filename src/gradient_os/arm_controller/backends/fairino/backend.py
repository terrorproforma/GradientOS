# backends/fairino/backend.py
#
# Read-only ActuatorBackend for the Fairino FR-series (FR10 in particular).
# Milestone 1 scope: connect to the FR10 controller over the network, stream
# joint state into GradientOS so the web UI can display it. NO motion yet —
# every method that would command the robot raises NotImplementedError with
# "Milestone N" pointing at when it lands.
#
# Architecture sketch:
#
#   FairinoBackend
#     ├── FairinoRPCClient   (rpc_client.py — quarantined SDK calls)
#     ├── FairinoStateStream (state_stream.py — daemon poller + snapshot)
#     └── frame_guard        (frame_guard.py — Option B enforcement)
#
# This backend is much thinner than feetech (no register layouts, no
# encoder math) because the FR10 owns its own real-time loop, calibration,
# joint limits, PID gains, and safety stack. We are a network client.

from __future__ import annotations

from typing import Optional

from ...actuator_interface import ActuatorBackend
from .config import FR10_NUM_JOINTS, FairinoBackendConfig
from .frame_guard import assert_identity_end_effector_offset
from .rpc_client import FairinoRPCClient
from .state_stream import FairinoStateStream


class FairinoBackend(ActuatorBackend):
    """ActuatorBackend implementation for the Fairino FR-series (read-only).

    Milestone 1: state streaming only. Initialize() runs the Option B frame
    guard, then connects to the FR10 over its Python SDK and starts a 50 Hz
    state poller. `get_joint_positions()` reads from the latest snapshot.
    All motion-command methods raise NotImplementedError — wired up in
    Milestone 2 (`set_joint_positions` → `MoveJ`) and beyond.

    Thread safety: every method on this class can be called concurrently
    with the poller thread; the underlying RPCClient and StateStream
    serialize their own internal state.
    """

    def __init__(
        self,
        robot_config: dict,
        backend_config: Optional[FairinoBackendConfig] = None,
        rpc_client: Optional[FairinoRPCClient] = None,
        state_stream: Optional[FairinoStateStream] = None,
    ) -> None:
        # `robot_config` mirrors the dict produced by RobotConfig.get_config_dict().
        # Most fields don't apply to the FR10 (servo IDs, encoder mappings, PID
        # gains all live inside the FR10 controller itself); we only touch the
        # joint-count field for sanity checking.
        configured_joints = int(robot_config.get("num_logical_joints", FR10_NUM_JOINTS))
        if configured_joints != FR10_NUM_JOINTS:
            raise ValueError(
                f"FairinoBackend expects num_logical_joints={FR10_NUM_JOINTS} "
                f"(FR10 is a 6-DOF arm), got {configured_joints}. Check the "
                "active RobotConfig — you probably want robots/fr10."
            )

        self._robot_config = robot_config
        self._backend_config = backend_config or FairinoBackendConfig.from_env()

        self._rpc = rpc_client or FairinoRPCClient(
            ip=self._backend_config.controller_ip,
            connect_timeout_s=self._backend_config.connect_timeout_s,
        )
        self._stream = state_stream or FairinoStateStream(
            rpc=self._rpc,
            poll_hz=self._backend_config.state_poll_hz,
        )

        self._initialized = False

    # =========================================================================
    # Lifecycle
    # =========================================================================

    def initialize(self) -> bool:
        # Frame guard FIRST — before any network I/O. If the user has a
        # non-identity END_EFFECTOR_OFFSET, we should fail before anything
        # touches the robot.
        try:
            assert_identity_end_effector_offset()
        except Exception as e:
            # Print loudly and re-raise — this is operator-actionable, not
            # a "degraded mode" we recover from. Letting it propagate makes
            # the failure visible in run_controller.py's startup log.
            print(f"[FairinoBackend] FRAME STRATEGY VIOLATION: {e}")
            raise

        try:
            self._rpc.connect()
        except Exception as e:
            print(
                f"[FairinoBackend] WARNING: connect to FR10 at "
                f"{self._backend_config.controller_ip} failed: {e}"
            )
            self._initialized = False
            return False

        self._stream.start()
        self._initialized = True
        print(
            f"[FairinoBackend] Connected to FR10 at "
            f"{self._backend_config.controller_ip} "
            f"(state poll {self._backend_config.state_poll_hz} Hz)"
        )
        return True

    def shutdown(self) -> None:
        try:
            self._stream.stop()
        except Exception as e:
            print(f"[FairinoBackend] state stream stop raised: {e}")
        try:
            self._rpc.disconnect()
        except Exception as e:
            print(f"[FairinoBackend] rpc disconnect raised: {e}")
        self._initialized = False

    @property
    def num_joints(self) -> int:
        return FR10_NUM_JOINTS

    @property
    def is_initialized(self) -> bool:
        return self._initialized and self._rpc.is_connected

    @property
    def encoder_resolution(self) -> int:
        # The FR10 controller doesn't expose servo encoder counts to the
        # network client — we work in radians end-to-end. Returning 0 (same
        # as ethercat_rtcore) tells any legacy raw-encoder code to back off.
        return 0

    # =========================================================================
    # State (Milestone 1 surface)
    # =========================================================================

    def get_joint_positions(self, verbose: bool = False) -> list[float]:
        """Latest 6 joint angles in radians. Never blocks."""
        snapshot = self._stream.latest()
        if verbose:
            age = snapshot.age_s()
            state = "fresh" if age <= self._backend_config.stale_snapshot_s else "STALE"
            print(
                f"[FairinoBackend] get_joint_positions -> {snapshot.joint_positions_rad} "
                f"({state}, age={age:.3f}s, seq={snapshot.sequence})"
            )

        if not snapshot.has_reading:
            # State hasn't arrived yet (we're between connect and the first
            # poll). Return zeros rather than raising — the UI must keep
            # rendering during the bring-up window.
            return [0.0] * FR10_NUM_JOINTS

        return list(snapshot.joint_positions_rad)

    def get_present_actuator_ids(self) -> set[int]:
        # The FR10 doesn't have user-visible actuator IDs the way Feetech
        # does. We expose 1..6 as synthetic IDs so the rest of GradientOS
        # has something stable to key on (e.g. for telemetry rows).
        if not self.is_initialized:
            return set()
        return set(range(1, FR10_NUM_JOINTS + 1))

    def ping_actuator(self, actuator_id: int) -> bool:
        return actuator_id in self.get_present_actuator_ids()

    # =========================================================================
    # Motion commands — Milestone 2+
    # =========================================================================

    def set_joint_positions(
        self,
        positions_rad: list[float],
        speed: float,
        acceleration: float,
    ) -> None:
        raise NotImplementedError(
            "FairinoBackend.set_joint_positions is Milestone 2 — read-only "
            "state streaming only at Milestone 1."
        )

    def prepare_sync_write_commands(
        self,
        positions_rad: list[float],
        speed: int = 4095,
        accel: int = 0,
    ) -> list[tuple]:
        raise NotImplementedError(
            "FairinoBackend has no SYNC_WRITE concept — the FR10 owns its "
            "own control loop. Use set_joint_positions (Milestone 2)."
        )

    def sync_write(self, commands: list[tuple]) -> None:
        raise NotImplementedError(
            "FairinoBackend has no SYNC_WRITE concept — the FR10 owns its "
            "own control loop. Use set_joint_positions (Milestone 2)."
        )

    def sync_read_positions(
        self,
        timeout_s: Optional[float] = None,
    ) -> dict[int, int]:
        # Return the "raw" positions as round-tripped joint-id -> rad*1000
        # int. We have no raw encoder counts to surface; returning {} makes
        # any legacy consumer no-op until they migrate to get_joint_positions.
        return {}

    def raw_to_joint_positions(self, raw_positions: dict[int, int]) -> list[float]:
        # Same rationale as sync_read_positions — there is no raw layer.
        # Return the latest cached joint vector so callers that combine
        # sync_read + raw_to_joint still get a reading.
        return self.get_joint_positions()

    def set_single_actuator_position(
        self,
        actuator_id: int,
        position_rad: float,
        speed: int,
        accel: int,
    ) -> None:
        raise NotImplementedError(
            "FairinoBackend.set_single_actuator_position is Milestone 2."
        )

    def read_single_actuator_position(self, actuator_id: int) -> Optional[int]:
        # No raw encoder values — return None to signal "not supported".
        return None

    # =========================================================================
    # Calibration / configuration — owned by the FR10 controller, not us
    # =========================================================================

    def set_current_position_as_zero(self, actuator_id: int) -> bool:
        # Joint zero is set on the FR10 controller via its WebApp / pendant
        # calibration UI. Not exposed via the network SDK on purpose.
        return False

    def set_pid_gains(self, actuator_id: int, kp: int, ki: int, kd: int) -> bool:
        # PID lives inside the FR10 servo drivers. Not user-tunable from here.
        return False

    def apply_joint_limits(self) -> bool:
        # Soft limits are configured on the FR10 (WebApp → Settings).
        # GradientOS clamps planner output against `logical_joint_limits_rad`
        # in robot config; the FR10 enforces hardware limits in firmware.
        return False
