"""Unit tests for the Fairino FR10 ActuatorBackend (Milestone 1, read-only).

These tests do not touch the real Fairino SDK or any network; the SDK is
mocked through `FairinoSDKAdapter`. The tests cover:

- frame_guard: identity vs. non-identity END_EFFECTOR_OFFSET behavior.
- rpc_client: lifecycle, deg→rad conversion, error paths.
- state_stream: start/stop, snapshot freshness, error resilience.
- backend: initialize/shutdown, motion-stub raises, joint-count validation.
"""

import os

# Avoid importing the ikfast pybind .so during test collection — the .so is
# only built in dev/prod environments. The numeric backend prints a warning
# but allows ik_solver to import, which is all the frame_guard tests need.
os.environ.setdefault("MINI_ARM_SOLVER", "numeric")

import math
import threading
import time
import unittest
from typing import Optional
from unittest import mock

import numpy as np

# Force ik_solver into sys.modules so mock.patch can walk the dotted path.
import gradient_os.ik_solver  # noqa: F401

from gradient_os.arm_controller.backends.fairino import (
    FairinoBackend,
    FairinoBackendConfig,
    FairinoRPCClient,
    FairinoStateStream,
    FairinoStatusFlags,
    FrameStrategyViolation,
)
from gradient_os.arm_controller.backends.fairino import frame_guard
from gradient_os.arm_controller.robots import FR10Config


# =============================================================================
# Test fakes
# =============================================================================


class FakeFairinoSDK:
    """In-memory stand-in for the real Fairino SDK.

    Mirrors the FairinoSDKAdapter Protocol. Tests can mutate
    `joints_deg` / `status` to drive the backend's perception, and inspect
    `connect_calls` / `disconnect_calls` to verify lifecycle.
    """

    def __init__(self) -> None:
        self.joints_deg: list[float] = [0.0] * 6
        self.status: FairinoStatusFlags = FairinoStatusFlags()
        self.connected: bool = False
        self.connect_calls: list[tuple[str, float]] = []
        self.disconnect_calls: int = 0
        self.read_calls: int = 0
        self.fail_next_read_with: Optional[Exception] = None

    # FairinoSDKAdapter protocol --------------------------------------------

    def connect(self, ip: str, timeout_s: float) -> None:
        self.connect_calls.append((ip, timeout_s))
        self.connected = True

    def disconnect(self) -> None:
        self.disconnect_calls += 1
        self.connected = False

    def is_connected(self) -> bool:
        return self.connected

    def read_joint_positions_deg(self) -> list[float]:
        self.read_calls += 1
        if self.fail_next_read_with is not None:
            err = self.fail_next_read_with
            self.fail_next_read_with = None
            raise err
        return list(self.joints_deg)

    def read_status_flags(self) -> FairinoStatusFlags:
        return FairinoStatusFlags(
            in_collision=self.status.in_collision,
            estopped=self.status.estopped,
            faulted=self.status.faulted,
            program_state=self.status.program_state,
            last_error_code=self.status.last_error_code,
        )


def _identity_offset_patch() -> mock.patch:
    """Patch END_EFFECTOR_OFFSET to identity for the duration of a test."""
    return mock.patch(
        "gradient_os.ik_solver.END_EFFECTOR_OFFSET",
        np.zeros(3, dtype=float),
    )


def _make_rpc_with_fake(
    fake: Optional[FakeFairinoSDK] = None,
    ip: str = "10.0.0.1",
) -> tuple[FairinoRPCClient, FakeFairinoSDK]:
    fake = fake or FakeFairinoSDK()
    rpc = FairinoRPCClient(
        ip=ip,
        connect_timeout_s=0.5,
        _sdk_factory=lambda: fake,
    )
    return rpc, fake


# =============================================================================
# Frame guard
# =============================================================================


class TestFrameGuard(unittest.TestCase):

    def test_identity_offset_passes(self) -> None:
        with _identity_offset_patch():
            frame_guard.assert_identity_end_effector_offset()  # no raise

    def test_zero_within_tolerance_passes(self) -> None:
        # 1e-9 m is well below the 1e-6 tolerance.
        with mock.patch(
            "gradient_os.ik_solver.END_EFFECTOR_OFFSET",
            np.array([1e-9, 0.0, -1e-9]),
        ):
            frame_guard.assert_identity_end_effector_offset()

    def test_non_identity_offset_raises(self) -> None:
        with mock.patch(
            "gradient_os.ik_solver.END_EFFECTOR_OFFSET",
            np.array([0.180, 0.0, 0.0]),
        ):
            with self.assertRaises(FrameStrategyViolation) as ctx:
                frame_guard.assert_identity_end_effector_offset()
            msg = str(ctx.exception)
            # Message must surface the offending value AND the fix path.
            self.assertIn("0.18", msg)
            self.assertIn("ik_solver.py", msg)
            self.assertIn("Milestone 3", msg)

    def test_wrong_shape_raises(self) -> None:
        with mock.patch(
            "gradient_os.ik_solver.END_EFFECTOR_OFFSET",
            np.array([0.0, 0.0]),
        ):
            with self.assertRaises(FrameStrategyViolation):
                frame_guard.assert_identity_end_effector_offset()


# =============================================================================
# RPC client
# =============================================================================


class TestFairinoRPCClient(unittest.TestCase):

    def test_connect_passes_ip_and_timeout(self) -> None:
        rpc, fake = _make_rpc_with_fake(ip="192.168.58.2")
        rpc.connect()
        self.assertEqual(fake.connect_calls, [("192.168.58.2", 0.5)])
        self.assertTrue(rpc.is_connected)

    def test_connect_is_idempotent(self) -> None:
        rpc, fake = _make_rpc_with_fake()
        rpc.connect()
        rpc.connect()
        # Second call must not re-invoke the SDK.
        self.assertEqual(len(fake.connect_calls), 1)

    def test_disconnect_is_idempotent_and_safe(self) -> None:
        rpc, fake = _make_rpc_with_fake()
        rpc.disconnect()  # before connect
        rpc.connect()
        rpc.disconnect()
        rpc.disconnect()  # second call
        self.assertFalse(rpc.is_connected)
        self.assertEqual(fake.disconnect_calls, 1)  # only the post-connect one

    def test_read_joints_converts_deg_to_rad(self) -> None:
        rpc, fake = _make_rpc_with_fake()
        fake.joints_deg = [0.0, 90.0, -45.0, 180.0, -180.0, 0.0]
        rpc.connect()
        joints_rad = rpc.read_joint_positions_rad()
        expected = [
            0.0, math.pi / 2, -math.pi / 4, math.pi, -math.pi, 0.0,
        ]
        for got, want in zip(joints_rad, expected):
            self.assertAlmostEqual(got, want, places=9)

    def test_read_before_connect_raises(self) -> None:
        rpc, _ = _make_rpc_with_fake()
        with self.assertRaises(RuntimeError):
            rpc.read_joint_positions_rad()


# =============================================================================
# State stream
# =============================================================================


class TestFairinoStateStream(unittest.TestCase):

    def test_latest_before_start_is_sentinel(self) -> None:
        rpc, _ = _make_rpc_with_fake()
        stream = FairinoStateStream(rpc=rpc, poll_hz=10.0)
        snap = stream.latest()
        self.assertFalse(snap.has_reading)
        self.assertEqual(snap.sequence, 0)

    def test_poller_publishes_readings(self) -> None:
        rpc, fake = _make_rpc_with_fake()
        rpc.connect()
        fake.joints_deg = [10.0, 20.0, 30.0, 40.0, 50.0, 60.0]
        stream = FairinoStateStream(rpc=rpc, poll_hz=200.0)
        stream.start()
        try:
            self._wait_for(lambda: stream.latest().sequence > 0, timeout_s=1.0)
            snap = stream.latest()
            self.assertTrue(snap.has_reading)
            for got, want in zip(
                snap.joint_positions_rad,
                [math.radians(d) for d in fake.joints_deg],
            ):
                self.assertAlmostEqual(got, want, places=6)
        finally:
            stream.stop()

    def test_poller_survives_transient_errors(self) -> None:
        rpc, fake = _make_rpc_with_fake()
        rpc.connect()
        fake.joints_deg = [1.0, 2.0, 3.0, 4.0, 5.0, 6.0]
        stream = FairinoStateStream(rpc=rpc, poll_hz=200.0)
        stream.start()
        try:
            # Get one good reading first.
            self._wait_for(lambda: stream.latest().sequence > 0, timeout_s=1.0)
            seq_before = stream.latest().sequence

            # Inject one failure, then verify subsequent reads still happen.
            fake.fail_next_read_with = RuntimeError("transient")
            self._wait_for(
                lambda: stream.latest().sequence > seq_before, timeout_s=1.0
            )
        finally:
            stream.stop()

    def test_stop_returns_promptly(self) -> None:
        rpc, _ = _make_rpc_with_fake()
        rpc.connect()
        stream = FairinoStateStream(rpc=rpc, poll_hz=2.0)  # 500ms period
        stream.start()
        t0 = time.monotonic()
        stream.stop()
        elapsed = time.monotonic() - t0
        # Stop event short-circuits the long sleep — should join in << 500ms.
        self.assertLess(elapsed, 0.3)

    def test_zero_poll_hz_rejected(self) -> None:
        rpc, _ = _make_rpc_with_fake()
        with self.assertRaises(ValueError):
            FairinoStateStream(rpc=rpc, poll_hz=0.0)

    @staticmethod
    def _wait_for(predicate, timeout_s: float) -> None:
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            if predicate():
                return
            time.sleep(0.01)
        raise AssertionError("predicate never became true")


# =============================================================================
# FairinoBackend
# =============================================================================


class TestFairinoBackend(unittest.TestCase):

    def _make_backend(
        self,
        fake: Optional[FakeFairinoSDK] = None,
        poll_hz: float = 200.0,
    ) -> tuple[FairinoBackend, FakeFairinoSDK]:
        fake = fake or FakeFairinoSDK()
        rpc = FairinoRPCClient(
            ip="10.0.0.1",
            connect_timeout_s=0.1,
            _sdk_factory=lambda: fake,
        )
        cfg = FairinoBackendConfig(
            controller_ip="10.0.0.1",
            state_poll_hz=poll_hz,
            connect_timeout_s=0.1,
            stale_snapshot_s=1.0,
        )
        backend = FairinoBackend(
            robot_config=FR10Config().get_config_dict(),
            backend_config=cfg,
            rpc_client=rpc,
            state_stream=FairinoStateStream(rpc=rpc, poll_hz=poll_hz),
        )
        return backend, fake

    def test_num_joints_is_six(self) -> None:
        backend, _ = self._make_backend()
        self.assertEqual(backend.num_joints, 6)

    def test_initialize_with_identity_offset_succeeds(self) -> None:
        backend, fake = self._make_backend()
        with _identity_offset_patch():
            ok = backend.initialize()
        try:
            self.assertTrue(ok)
            self.assertTrue(backend.is_initialized)
            self.assertEqual(len(fake.connect_calls), 1)
        finally:
            backend.shutdown()
        self.assertEqual(fake.disconnect_calls, 1)
        self.assertFalse(backend.is_initialized)

    def test_initialize_with_non_identity_offset_raises(self) -> None:
        backend, fake = self._make_backend()
        with mock.patch(
            "gradient_os.ik_solver.END_EFFECTOR_OFFSET",
            np.array([0.180, 0.0, 0.0]),
        ):
            with self.assertRaises(FrameStrategyViolation):
                backend.initialize()
        # Frame guard runs BEFORE network I/O — no connect attempt.
        self.assertEqual(len(fake.connect_calls), 0)

    def test_get_joint_positions_returns_latest_snapshot(self) -> None:
        backend, fake = self._make_backend()
        fake.joints_deg = [0.0, 30.0, 60.0, 90.0, 120.0, 150.0]
        with _identity_offset_patch():
            backend.initialize()
        try:
            self._wait_for(
                lambda: backend.get_joint_positions()[1] != 0.0,
                timeout_s=1.0,
            )
            joints = backend.get_joint_positions()
            for got, want in zip(joints, [math.radians(d) for d in fake.joints_deg]):
                self.assertAlmostEqual(got, want, places=6)
        finally:
            backend.shutdown()

    def test_get_joint_positions_returns_zeros_before_first_reading(self) -> None:
        # If no poll has completed yet, get_joint_positions returns zeros
        # (sentinel) instead of raising — the UI must keep rendering.
        backend, _ = self._make_backend(poll_hz=0.5)  # 2s period
        with _identity_offset_patch():
            backend.initialize()
        try:
            joints = backend.get_joint_positions()
            self.assertEqual(joints, [0.0] * 6)
        finally:
            backend.shutdown()

    def test_set_joint_positions_raises_not_implemented(self) -> None:
        backend, _ = self._make_backend()
        with self.assertRaises(NotImplementedError) as ctx:
            backend.set_joint_positions([0.0] * 6, speed=1.0, acceleration=1.0)
        self.assertIn("Milestone 2", str(ctx.exception))

    def test_sync_write_raises_not_implemented(self) -> None:
        backend, _ = self._make_backend()
        with self.assertRaises(NotImplementedError):
            backend.sync_write([])

    def test_set_single_actuator_raises_not_implemented(self) -> None:
        backend, _ = self._make_backend()
        with self.assertRaises(NotImplementedError):
            backend.set_single_actuator_position(1, 0.0, 50, 0)

    def test_calibration_methods_return_false(self) -> None:
        backend, _ = self._make_backend()
        self.assertFalse(backend.set_current_position_as_zero(1))
        self.assertFalse(backend.set_pid_gains(1, 50, 0, 10))
        self.assertFalse(backend.apply_joint_limits())

    def test_wrong_num_joints_in_robot_config_raises(self) -> None:
        bad_config = FR10Config().get_config_dict()
        bad_config["num_logical_joints"] = 7
        with self.assertRaises(ValueError) as ctx:
            FairinoBackend(robot_config=bad_config)
        self.assertIn("FR10", str(ctx.exception))

    def test_present_actuator_ids_when_uninitialized(self) -> None:
        backend, _ = self._make_backend()
        self.assertEqual(backend.get_present_actuator_ids(), set())

    def test_present_actuator_ids_when_initialized(self) -> None:
        backend, _ = self._make_backend()
        with _identity_offset_patch():
            backend.initialize()
        try:
            self.assertEqual(backend.get_present_actuator_ids(), {1, 2, 3, 4, 5, 6})
        finally:
            backend.shutdown()

    @staticmethod
    def _wait_for(predicate, timeout_s: float) -> None:
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            if predicate():
                return
            time.sleep(0.01)
        raise AssertionError("predicate never became true")


# =============================================================================
# Registry wiring
# =============================================================================


class TestFairinoRegistry(unittest.TestCase):

    def test_fairino_registered_in_backend_classes(self) -> None:
        from gradient_os.arm_controller.backends import registry
        self.assertIn("fairino", registry.BACKEND_CLASSES)

    def test_fairino_config_module_registered(self) -> None:
        from gradient_os.arm_controller.backends import registry
        self.assertIn("fairino", registry.BACKEND_CONFIG_MODULES)

    def test_fr10_robot_config_targets_fairino_backend(self) -> None:
        cfg = FR10Config()
        self.assertEqual(cfg.default_servo_backend, "fairino")
        self.assertEqual(cfg.num_logical_joints, 6)

    def test_create_backend_via_registry_factory(self) -> None:
        # Exercises the full create_backend path end-to-end (the same path
        # run_controller.py uses at startup) so a future regression in the
        # factory wiring fails here, not in production.
        from gradient_os.arm_controller.backends import registry
        cfg = FR10Config().get_config_dict()
        backend = registry.create_backend("fairino", cfg)
        self.assertIsInstance(backend, FairinoBackend)
        self.assertEqual(backend.num_joints, 6)
        # Not initialized yet — must not have touched the network.
        self.assertFalse(backend.is_initialized)


# =============================================================================
# Status flags + snapshot semantics
# =============================================================================


class TestStatusAndSnapshotSemantics(unittest.TestCase):

    def test_status_flags_propagate_through_stream(self) -> None:
        rpc, fake = _make_rpc_with_fake()
        rpc.connect()
        fake.status = FairinoStatusFlags(
            in_collision=False,
            estopped=True,
            faulted=True,
            program_state="emergency_stop",
            last_error_code=42,
        )
        stream = FairinoStateStream(rpc=rpc, poll_hz=200.0)
        stream.start()
        try:
            _wait_for(lambda: stream.latest().sequence > 0, timeout_s=1.0)
            snap = stream.latest()
            self.assertTrue(snap.status.faulted)
            self.assertTrue(snap.status.estopped)
            self.assertEqual(snap.status.last_error_code, 42)
            self.assertEqual(snap.status.program_state, "emergency_stop")
        finally:
            stream.stop()

    def test_snapshot_is_deep_copied(self) -> None:
        # Mutating a returned snapshot must not affect the cache (or other
        # callers). This is what makes latest() safe to call concurrently
        # with the poller.
        rpc, fake = _make_rpc_with_fake()
        rpc.connect()
        fake.joints_deg = [10.0, 20.0, 30.0, 40.0, 50.0, 60.0]
        stream = FairinoStateStream(rpc=rpc, poll_hz=200.0)
        stream.start()
        try:
            _wait_for(lambda: stream.latest().sequence > 0, timeout_s=1.0)
            snap_a = stream.latest()
            snap_a.joint_positions_rad[0] = 999.0  # caller mutation
            snap_a.status.faulted = True
            snap_b = stream.latest()
            self.assertNotEqual(snap_b.joint_positions_rad[0], 999.0)
            self.assertFalse(snap_b.status.faulted)
        finally:
            stream.stop()

    def test_snapshot_age_s_is_finite_after_reading(self) -> None:
        rpc, _ = _make_rpc_with_fake()
        rpc.connect()
        stream = FairinoStateStream(rpc=rpc, poll_hz=200.0)
        stream.start()
        try:
            _wait_for(lambda: stream.latest().sequence > 0, timeout_s=1.0)
            age = stream.latest().age_s()
            self.assertGreaterEqual(age, 0.0)
            self.assertLess(age, 1.0)
        finally:
            stream.stop()

    def test_snapshot_age_s_is_inf_before_any_reading(self) -> None:
        rpc, _ = _make_rpc_with_fake()
        stream = FairinoStateStream(rpc=rpc, poll_hz=10.0)
        self.assertEqual(stream.latest().age_s(), math.inf)


# =============================================================================
# Config + env overrides
# =============================================================================


class TestFairinoBackendConfig(unittest.TestCase):

    def test_defaults_match_fr10_factory_settings(self) -> None:
        cfg = FairinoBackendConfig()
        self.assertEqual(cfg.controller_ip, "192.168.58.2")
        self.assertEqual(cfg.rpc_port, 20003)
        self.assertEqual(cfg.state_port, 20004)

    def test_env_overrides_apply(self) -> None:
        with mock.patch.dict(
            "os.environ",
            {
                "GRADIENT_FAIRINO_IP": "10.9.9.9",
                "GRADIENT_FAIRINO_POLL_HZ": "75",
                "GRADIENT_FAIRINO_CONNECT_TIMEOUT_S": "1.5",
            },
            clear=False,
        ):
            cfg = FairinoBackendConfig.from_env()
        self.assertEqual(cfg.controller_ip, "10.9.9.9")
        self.assertEqual(cfg.state_poll_hz, 75.0)
        self.assertEqual(cfg.connect_timeout_s, 1.5)

    def test_invalid_env_value_falls_back_to_default(self) -> None:
        with mock.patch.dict(
            "os.environ",
            {"GRADIENT_FAIRINO_POLL_HZ": "not-a-number"},
            clear=False,
        ):
            cfg = FairinoBackendConfig.from_env()
        # Falls back to module default rather than crashing startup.
        self.assertEqual(cfg.state_poll_hz, 50.0)


# =============================================================================
# Lifecycle
# =============================================================================


class TestFairinoBackendLifecycle(unittest.TestCase):

    def _make_backend(self, fake=None, poll_hz=200.0):
        fake = fake or FakeFairinoSDK()
        rpc = FairinoRPCClient(
            ip="10.0.0.1",
            connect_timeout_s=0.1,
            _sdk_factory=lambda: fake,
        )
        cfg = FairinoBackendConfig(
            controller_ip="10.0.0.1",
            state_poll_hz=poll_hz,
            connect_timeout_s=0.1,
        )
        backend = FairinoBackend(
            robot_config=FR10Config().get_config_dict(),
            backend_config=cfg,
            rpc_client=rpc,
            state_stream=FairinoStateStream(rpc=rpc, poll_hz=poll_hz),
        )
        return backend, fake

    def test_shutdown_is_idempotent(self) -> None:
        backend, fake = self._make_backend()
        with _identity_offset_patch():
            backend.initialize()
        backend.shutdown()
        backend.shutdown()  # second call must not raise
        self.assertEqual(fake.disconnect_calls, 1)
        self.assertFalse(backend.is_initialized)

    def test_initialize_returns_false_on_connect_failure(self) -> None:
        # If the SDK raises during connect, initialize logs and returns
        # False (so run_controller stays alive in degraded mode), rather
        # than propagating.
        class FlakySDK(FakeFairinoSDK):
            def connect(self, ip, timeout_s):  # type: ignore[override]
                raise ConnectionRefusedError("FR10 unreachable")

        backend, _ = self._make_backend(fake=FlakySDK())
        with _identity_offset_patch():
            ok = backend.initialize()
        self.assertFalse(ok)
        self.assertFalse(backend.is_initialized)

    def test_get_joint_positions_verbose_does_not_raise(self) -> None:
        # Smoke test: verbose logging path must not crash on a stale or
        # not-yet-arrived snapshot. The UI passes verbose=True for diagnostics.
        backend, _ = self._make_backend(poll_hz=0.5)
        with _identity_offset_patch():
            backend.initialize()
        try:
            joints = backend.get_joint_positions(verbose=True)
            self.assertEqual(len(joints), 6)
        finally:
            backend.shutdown()


def _wait_for(predicate, timeout_s: float) -> None:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(0.01)
    raise AssertionError("predicate never became true")


if __name__ == "__main__":
    unittest.main()
