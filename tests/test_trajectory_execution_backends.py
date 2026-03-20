import sys
from unittest.mock import MagicMock

sys.modules.setdefault("scipy.signal", MagicMock())

import pytest

from gradient_os.arm_controller import trajectory_execution
from gradient_os.arm_controller import utils


class FakeBackend:
    def __init__(self, actual_positions):
        self.actual_positions = list(actual_positions)
        self.is_initialized = True
        self.prepare_calls = []
        self.sync_write_calls = []
        self.sync_read_calls = []

    def prepare_sync_write_commands(self, positions_rad, speed=4095, accel=0):
        payload = list(positions_rad)
        self.prepare_calls.append((payload, speed, accel))
        return [("setpoint_rad", payload)]

    def sync_write(self, commands):
        self.sync_write_calls.append(commands)

    def sync_read_positions(self, timeout_s=None):
        self.sync_read_calls.append(timeout_s)
        return {idx: 1000 + idx for idx in range(len(self.actual_positions))}

    def raw_to_joint_positions(self, raw_positions):
        return list(self.actual_positions)

    def get_joint_positions(self, verbose=False):
        return list(self.actual_positions)


def _configure_backend_executor_test(monkeypatch, backend, *, num_joints=2):
    monkeypatch.setattr(trajectory_execution, "_get_backend", lambda: backend)
    monkeypatch.setattr(trajectory_execution, "_use_backend", lambda: True)
    monkeypatch.setattr(utils, "NUM_LOGICAL_JOINTS", num_joints, raising=False)
    monkeypatch.setattr(utils, "LOGICAL_JOINT_MASTER_OFFSETS_RAD", [0.0] * num_joints, raising=False)
    monkeypatch.setattr(utils, "CORRECTION_INTEGRAL_CLAMP_RAD", 1.0, raising=False)
    monkeypatch.setattr(utils, "ENCODER_RESOLUTION", None, raising=False)
    monkeypatch.setattr(utils, "current_logical_joint_angles_rad", [0.0] * num_joints, raising=False)
    monkeypatch.setattr(trajectory_execution.time, "sleep", lambda _seconds: None)
    utils.trajectory_state.update(
        {
            "diagnostics_enabled": False,
            "should_stop": False,
            "thread": None,
            "is_running": False,
        }
    )


def test_closed_loop_executor_uses_backend_joint_space(monkeypatch):
    backend = FakeBackend(actual_positions=[0.1, -0.2])
    _configure_backend_executor_test(monkeypatch, backend)

    monkeypatch.setattr(
        trajectory_execution.servo_driver,
        "servo_value_to_radians",
        lambda *_args, **_kwargs: (_ for _ in ()).throw(
            AssertionError("closed-loop backend mode should not use servo decode helpers")
        ),
    )

    telemetry = trajectory_execution._closed_loop_executor_thread(
        joint_path=[[0.3, -0.4], [0.35, -0.45]],
        frequency=50,
        diagnostics=False,
        return_telemetry=True,
        owns_trajectory_state=False,
    )

    assert backend.prepare_calls == [
        ([0.3, -0.4], 4095, 0),
        ([0.35, -0.45], 4095, 0),
    ]
    assert backend.sync_write_calls == [
        [("setpoint_rad", [0.3, -0.4])],
        [("setpoint_rad", [0.35, -0.45])],
    ]
    assert telemetry["actual_angles_per_joint"][0] == [0.1, 0.1]
    assert telemetry["actual_angles_per_joint"][1] == [-0.2, -0.2]
    assert telemetry["abs_errors_per_joint"][0] == pytest.approx([0.2, 0.25])
    assert telemetry["abs_errors_per_joint"][1] == pytest.approx([0.2, 0.25])


def test_open_loop_executor_uses_backend_precomputed_commands(monkeypatch):
    backend = FakeBackend(actual_positions=[0.0, 0.0])
    _configure_backend_executor_test(monkeypatch, backend)

    monkeypatch.setattr(
        trajectory_execution.servo_driver,
        "logical_q_to_syncwrite_tuple",
        lambda *_args, **_kwargs: (_ for _ in ()).throw(
            AssertionError("open-loop backend mode should not build legacy servo sync-write tuples")
        ),
    )

    trajectory_execution._open_loop_executor_thread(
        joint_path=[[0.2, -0.1]],
        frequency=100,
        diagnostics=False,
        owns_trajectory_state=False,
    )

    assert backend.prepare_calls == [([0.2, -0.1], 4095, 0)]
    assert backend.sync_write_calls == [[("setpoint_rad", [0.2, -0.1])]]
