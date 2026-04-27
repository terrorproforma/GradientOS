import sys
import threading
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


class FakeRTCoreTrajectoryBackend(FakeBackend):
    def __init__(self, actual_positions):
        super().__init__(actual_positions)
        self.execute_calls = []
        self.submit_calls = []
        self.final_point_calls = []
        self.complete_calls = []
        self.final_state_name = "executing"
        self.final_point_raises: Exception | None = None
        self.complete_raises: Exception | None = None
        self.on_complete = None
        self.abort_calls = []
        self.completion_diagnostics = None

    def logical_joint_indices_to_axis_mask(self, logical_joint_indices):
        return sum(1 << int(idx) for idx in logical_joint_indices)

    def execute_joint_trajectory(self, joint_path, frequency, axis_mask=None):
        self.execute_calls.append((joint_path, frequency, axis_mask))

        class _Status:
            state_name = "completed"
            active_traj_id = 77

        return _Status()

    def submit_joint_trajectory(self, joint_path, frequency, axis_mask=None):
        self.submit_calls.append((joint_path, frequency, axis_mask))

        class _Submission:
            traj_id = 77
            submitted_command_seq = 123
            expected_points = len(joint_path)
            duration_s = len(joint_path) / float(frequency)

        return _Submission()

    def wait_for_trajectory_final_point_sent(
        self,
        traj_id,
        *,
        expected_points,
        timeout_s,
        submitted_command_seq=None,
    ):
        self.final_point_calls.append((traj_id, expected_points, timeout_s, submitted_command_seq))
        if self.final_point_raises is not None:
            raise self.final_point_raises

        class _Status:
            pass

        _Status.state_name = self.final_state_name
        _Status.active_traj_id = traj_id

        return _Status()

    def abort_trajectory(self, traj_id=None):
        self.abort_calls.append(traj_id)

    def wait_for_trajectory_complete(self, traj_id, *, timeout_s, submitted_command_seq=None):
        self.complete_calls.append((traj_id, timeout_s, submitted_command_seq))
        if callable(self.on_complete):
            self.on_complete()
        if self.complete_raises is not None:
            raise self.complete_raises

        class _Status:
            state_name = "completed"
            active_traj_id = traj_id

        return _Status()

    def get_trajectory_completion_diagnostics(self, traj_id=None):
        return self.completion_diagnostics


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
            "motion_state": "IDLE",
            "rtcore_settle_traj_id": None,
            "last_bounded_endpoint": None,
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


def test_open_loop_executor_offloads_rtcore_trajectory_backend(monkeypatch):
    backend = FakeRTCoreTrajectoryBackend(actual_positions=[0.0, 0.0])
    _configure_backend_executor_test(monkeypatch, backend)

    trajectory_execution._open_loop_executor_thread(
        joint_path=[[0.1, -0.2], [0.2, -0.3], [0.3, -0.4]],
        frequency=100,
        diagnostics=False,
        owns_trajectory_state=False,
    )

    assert backend.submit_calls == [
        (
            [[0.1, -0.2], [0.2, -0.3], [0.3, -0.4]],
            100,
            None,
        )
    ]
    assert backend.final_point_calls
    assert backend.complete_calls == [(77, 0.5, 123)]
    assert backend.prepare_calls == []
    assert backend.sync_write_calls == []


def test_open_loop_executor_passes_targeted_rtcore_axis_mask(monkeypatch):
    backend = FakeRTCoreTrajectoryBackend(actual_positions=[0.0, 0.0])
    _configure_backend_executor_test(monkeypatch, backend)

    trajectory_execution._open_loop_executor_thread(
        joint_path=[[0.1, -0.2], [0.2, -0.3]],
        frequency=100,
        diagnostics=False,
        owns_trajectory_state=False,
        target_joint_indices=[1],
    )

    assert backend.submit_calls == [
        (
            [[0.1, -0.2], [0.2, -0.3]],
            100,
            0x2,
        )
    ]


def test_open_loop_executor_resets_motion_state_when_owning_lifecycle(monkeypatch):
    backend = FakeRTCoreTrajectoryBackend(actual_positions=[0.0, 0.0])
    _configure_backend_executor_test(monkeypatch, backend)

    utils.trajectory_state.update(
        {
            "thread": threading.current_thread(),
            "is_running": True,
            "motion_state": "EXECUTING",
        }
    )

    trajectory_execution._open_loop_executor_thread(
        joint_path=[[0.1, -0.2], [0.2, -0.3]],
        frequency=100,
        diagnostics=False,
        owns_trajectory_state=True,
    )

    assert utils.trajectory_state["is_running"] is False
    assert utils.trajectory_state["thread"] is None
    assert utils.get_motion_state() == "IDLE"


def test_rtcore_executor_does_not_record_endpoint_on_faulted_final_point(monkeypatch):
    backend = FakeRTCoreTrajectoryBackend(actual_positions=[0.0, 0.0])
    backend.final_state_name = "faulted"
    _configure_backend_executor_test(monkeypatch, backend)
    utils.trajectory_state.update(
        {
            "thread": threading.current_thread(),
            "is_running": True,
            "motion_state": "EXECUTING",
            "last_bounded_endpoint": {"arm_rad": [99.0, 99.0]},
        }
    )

    with pytest.raises(RuntimeError, match="trustworthy final endpoint"):
        trajectory_execution._open_loop_executor_thread(
            joint_path=[[0.1, -0.2], [0.2, -0.3]],
            frequency=100,
            diagnostics=False,
            owns_trajectory_state=True,
        )

    assert utils.trajectory_state["last_bounded_endpoint"] is None


def test_rtcore_executor_aborts_and_clears_endpoint_on_final_point_timeout(monkeypatch):
    backend = FakeRTCoreTrajectoryBackend(actual_positions=[0.0, 0.0])
    backend.final_point_raises = TimeoutError("final point never issued")
    _configure_backend_executor_test(monkeypatch, backend)
    utils.trajectory_state.update(
        {
            "thread": threading.current_thread(),
            "is_running": True,
            "motion_state": "EXECUTING",
            "last_bounded_endpoint": {"arm_rad": [99.0, 99.0]},
        }
    )

    trajectory_execution._open_loop_executor_thread(
        joint_path=[[0.1, -0.2], [0.2, -0.3]],
        frequency=100,
        diagnostics=False,
        owns_trajectory_state=True,
    )

    assert backend.abort_calls == [77]
    assert utils.trajectory_state["last_bounded_endpoint"] is None
    assert utils.trajectory_state["is_running"] is False
    assert utils.get_motion_state() == "IDLE"


def test_rtcore_executor_old_settle_watcher_cannot_idle_new_motion(monkeypatch):
    backend = FakeRTCoreTrajectoryBackend(actual_positions=[0.0, 0.0])
    _configure_backend_executor_test(monkeypatch, backend)
    utils.trajectory_state.update(
        {
            "thread": threading.current_thread(),
            "is_running": True,
            "motion_state": "EXECUTING",
        }
    )

    def _new_motion_started():
        utils.trajectory_state.update(
            {
                "is_running": True,
                "thread": object(),
                "motion_state": "EXECUTING",
                "rtcore_settle_traj_id": 77,
            }
        )

    backend.on_complete = _new_motion_started

    trajectory_execution._open_loop_executor_thread(
        joint_path=[[0.1, -0.2], [0.2, -0.3]],
        frequency=100,
        diagnostics=False,
        owns_trajectory_state=True,
    )

    assert utils.trajectory_state["is_running"] is True
    assert utils.get_motion_state() == "EXECUTING"


def test_closed_loop_executor_resets_motion_state_when_owning_lifecycle(monkeypatch):
    backend = FakeBackend(actual_positions=[0.1, -0.2])
    _configure_backend_executor_test(monkeypatch, backend)

    utils.trajectory_state.update(
        {
            "thread": threading.current_thread(),
            "is_running": True,
            "motion_state": "EXECUTING",
        }
    )

    trajectory_execution._closed_loop_executor_thread(
        joint_path=[[0.3, -0.4], [0.35, -0.45]],
        frequency=50,
        diagnostics=False,
        return_telemetry=False,
        owns_trajectory_state=True,
    )

    assert utils.trajectory_state["is_running"] is False
    assert utils.trajectory_state["thread"] is None
    assert utils.get_motion_state() == "IDLE"


def test_trajectory_executor_records_program_completion(monkeypatch):
    monkeypatch.setattr(trajectory_execution, "_get_backend", lambda: None)
    monkeypatch.setattr(trajectory_execution, "_execute_joint_path", lambda _path, _freq, **_kwargs: None)
    monkeypatch.setattr(
        trajectory_execution.servo_driver,
        "set_servo_positions",
        lambda *_args, **_kwargs: None,
    )
    monkeypatch.setattr(trajectory_execution.time, "sleep", lambda _seconds: None)

    utils.program_status_reset(
        name="alpha",
        active=True,
        state="accepted",
        step_count=3,
        move_steps=1,
        pause_steps=1,
        joint_move_steps=1,
    )
    utils.trajectory_state.update(
        {
            "should_stop": False,
            "stop_request_reason": None,
            "is_running": True,
            "thread": None,
            "active_program_name": "alpha",
        }
    )

    trajectory_execution._trajectory_executor_thread(
        [
            {"type": "move", "path": [[0.0, 0.0]], "freq": 100},
            {"type": "pause", "duration": 0.01},
            {"type": "joint_move", "target_q": [0.1, -0.1], "speed": 100, "duration": 0.01},
        ],
        should_loop=False,
    )

    program = utils.program_status_snapshot()
    assert program["state"] == "completed"
    assert program["terminal_reason"] == "completed"
    assert program["failing_step_index"] is None
    assert program["completed_step_count"] == 3
    assert program["completed_loop_count"] == 0


def test_trajectory_executor_counts_compound_move_logical_steps(monkeypatch):
    monkeypatch.setattr(trajectory_execution, "_get_backend", lambda: None)
    monkeypatch.setattr(trajectory_execution, "_execute_joint_path", lambda _path, _freq, **_kwargs: None)
    monkeypatch.setattr(trajectory_execution.time, "sleep", lambda _seconds: None)

    utils.program_status_reset(
        name="compound",
        active=True,
        state="accepted",
        step_count=3,
        move_steps=2,
        pause_steps=1,
    )
    utils.trajectory_state.update(
        {
            "should_stop": False,
            "stop_request_reason": None,
            "is_running": True,
            "thread": None,
            "active_program_name": "compound",
        }
    )

    trajectory_execution._trajectory_executor_thread(
        [
            {
                "type": "move",
                "path": [[0.0, 0.0], [0.1, 0.0], [0.1, 0.0], [0.2, 0.0]],
                "freq": 100,
                "logical_step_count": 3,
            }
        ],
        should_loop=False,
    )

    program = utils.program_status_snapshot()
    assert program["state"] == "completed"
    assert program["completed_step_count"] == 3


def test_trajectory_executor_records_rtcore_fault_terminal_reason(monkeypatch):
    class _FaultBackend:
        is_initialized = True

        @staticmethod
        def get_execution_status():
            class _Status:
                state_name = "faulted"

            return _Status()

    monkeypatch.setattr(trajectory_execution, "_get_backend", lambda: _FaultBackend())
    monkeypatch.setattr(
        trajectory_execution,
        "_execute_joint_path",
        lambda _path, _freq: (_ for _ in ()).throw(RuntimeError("rtcore fault")),
    )
    monkeypatch.setattr(trajectory_execution.time, "sleep", lambda _seconds: None)

    utils.program_status_reset(
        name="beta",
        active=True,
        state="accepted",
        step_count=1,
        move_steps=1,
    )
    utils.trajectory_state.update(
        {
            "should_stop": False,
            "stop_request_reason": None,
            "is_running": True,
            "thread": None,
            "active_program_name": "beta",
        }
    )

    trajectory_execution._trajectory_executor_thread(
        [{"type": "move", "path": [[0.0, 0.0]], "freq": 100}],
        should_loop=False,
    )

    program = utils.program_status_snapshot()
    assert program["state"] == "faulted"
    assert program["terminal_reason"] == "rtcore_fault"
    assert program["failing_step_index"] == 0
    assert program["completed_step_count"] == 0


def test_open_loop_executor_strict_completion_raises_on_settle_timeout(monkeypatch):
    backend = FakeRTCoreTrajectoryBackend(actual_positions=[0.0, 0.0])
    backend.complete_raises = TimeoutError("settle timeout")
    backend.completion_diagnostics = {
        "traj_id": 77,
        "final_due": True,
        "tolerance_counts": 128,
        "axes": [
            {
                "axis_index": 1,
                "target_counts": 1000,
                "feedback_counts": 1175,
                "error_counts": 175,
            }
        ],
    }
    _configure_backend_executor_test(monkeypatch, backend)
    monkeypatch.setattr(
        trajectory_execution.servo_driver,
        "get_control_arm_state_rad",
        lambda verbose=False: [0.25, -0.25],
    )

    with pytest.raises(TimeoutError, match="strict completion") as excinfo:
        trajectory_execution._open_loop_executor_thread(
            joint_path=[[0.1, -0.2], [0.2, -0.3]],
            frequency=100,
            diagnostics=False,
            owns_trajectory_state=False,
            require_completion=True,
        )

    assert backend.abort_calls == [77]
    assert backend.complete_calls == [
        (77, trajectory_execution.RTCORE_STRICT_COMPLETION_SETTLE_TIMEOUT_S, 123),
        (77, 0.5, 123),
    ]
    assert utils.trajectory_state["last_bounded_endpoint"] is None
    assert "live_endpoint_max_abs_err_rad=0.050000" in str(excinfo.value)
    assert "tolerance_counts=128" in str(excinfo.value)
    assert "axis1:target=1000:feedback=1175:error=175" in str(excinfo.value)


def test_open_loop_executor_strict_completion_raises_on_final_point_timeout(monkeypatch):
    backend = FakeRTCoreTrajectoryBackend(actual_positions=[0.0, 0.0])
    backend.final_point_raises = TimeoutError("final point never issued")
    _configure_backend_executor_test(monkeypatch, backend)

    with pytest.raises(TimeoutError, match="final point.*strict completion"):
        trajectory_execution._open_loop_executor_thread(
            joint_path=[[0.1, -0.2], [0.2, -0.3]],
            frequency=100,
            diagnostics=False,
            owns_trajectory_state=False,
            require_completion=True,
        )

    assert backend.abort_calls == [77]
    assert backend.complete_calls == [(77, 0.5, 123)]
    assert utils.trajectory_state["last_bounded_endpoint"] is None


def test_open_loop_executor_default_completion_timeout_stays_non_fatal(monkeypatch):
    backend = FakeRTCoreTrajectoryBackend(actual_positions=[0.0, 0.0])
    backend.complete_raises = TimeoutError("settle timeout")
    _configure_backend_executor_test(monkeypatch, backend)

    trajectory_execution._open_loop_executor_thread(
        joint_path=[[0.1, -0.2], [0.2, -0.3]],
        frequency=100,
        diagnostics=False,
        owns_trajectory_state=False,
    )

    assert backend.abort_calls == []


def test_trajectory_executor_threads_move_require_completion_flag(monkeypatch):
    calls: list[dict] = []
    _configure_backend_executor_test(
        monkeypatch,
        FakeRTCoreTrajectoryBackend(actual_positions=[0.0, 0.0]),
    )
    monkeypatch.setattr(
        trajectory_execution,
        "_open_loop_executor_thread",
        lambda *args, **kwargs: calls.append({"args": args, "kwargs": kwargs}),
    )

    trajectory_execution._trajectory_executor_thread(
        [
            {
                "type": "move",
                "path": [[0.0, 0.0], [0.1, 0.1]],
                "freq": 100,
                "require_completion": True,
            }
        ],
        should_loop=False,
    )

    assert calls
    assert calls[0]["kwargs"]["require_completion"] is True
