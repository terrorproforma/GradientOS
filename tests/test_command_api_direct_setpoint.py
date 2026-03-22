import numpy as np

from gradient_os.arm_controller import command_api


def test_handle_apply_joint_setpoint_uses_fallbacks_when_servo_defaults_missing(monkeypatch):
    calls: list[tuple[list[float], int, float]] = []

    monkeypatch.setattr(command_api.utils, "DEFAULT_SERVO_SPEED", None)
    monkeypatch.setattr(command_api.utils, "DEFAULT_SERVO_ACCELERATION_DEG_S2", None)
    monkeypatch.setattr(
        command_api.servo_driver,
        "set_servo_positions",
        lambda arm_angles_rad, speed_value, acceleration_value_deg_s2: calls.append(
            (list(arm_angles_rad), int(speed_value), float(acceleration_value_deg_s2))
        ),
    )

    result = command_api.handle_apply_joint_setpoint([0.1, 0.0, 0.0, 0.0, 0.0, 0.0])

    assert result == {
        "accepted": True,
        "completion_scope": "controller_ack",
        "trajectory_id": 0,
        "source_of_truth": "controller",
        "state": "accepted",
        "execution": {
            "controller_motion_state": "idle",
            "controller_thread_running": False,
            "last_correlation_id": None,
            "rtcore_status_present": False,
        },
        "speed": 500,
        "acceleration": 500.0,
    }
    assert calls == [([0.1, 0.0, 0.0, 0.0, 0.0, 0.0], 500, 500.0)]


def test_handle_apply_joint_setpoint_can_start_bounded_joint_trajectory(monkeypatch):
    thread_events: list[str] = []

    class _FakeRobot:
        actuator_gear_ratios = [100.0, 100.0, 100.0, 18.0, 18.1818, 10.0]

    class _FakeThread:
        def __init__(self, *, target, kwargs, daemon):
            self._target = target
            self._kwargs = kwargs
            self.daemon = daemon

        def start(self):
            thread_events.append("started")

    monkeypatch.setattr(command_api.utils, "DEFAULT_SERVO_SPEED", 500)
    monkeypatch.setattr(command_api.utils, "DEFAULT_SERVO_ACCELERATION_DEG_S2", 500.0)
    monkeypatch.setattr(command_api.utils, "trajectory_state", {"is_running": False, "should_stop": False, "thread": None})
    monkeypatch.setattr(command_api.utils, "trajectory_state_get", lambda key, default=None: command_api.utils.trajectory_state.get(key, default))
    monkeypatch.setattr(command_api.utils, "trajectory_state_update", lambda **kwargs: command_api.utils.trajectory_state.update(kwargs))
    monkeypatch.setattr(command_api.utils, "set_motion_state", lambda state: thread_events.append(f"state:{state}"))
    monkeypatch.setattr(command_api.servo_driver, "get_current_arm_state_rad", lambda verbose=False: [0.0] * 6)
    monkeypatch.setattr(command_api.servo_driver, "set_servo_positions", lambda *args, **kwargs: thread_events.append("direct-write"))
    monkeypatch.setattr(command_api.robot_config, "get_active_robot", lambda: _FakeRobot())
    monkeypatch.setattr(command_api.trajectory_execution, "_open_loop_executor_thread", lambda **kwargs: None)
    monkeypatch.setattr(command_api.threading, "Thread", _FakeThread)

    result = command_api.handle_apply_joint_setpoint(
        [0.2, 0.0, 0.0, 0.0, 0.0, 0.0],
        max_motor_rpm=100.0,
    )

    assert result["accepted"] is True
    assert result["completion_scope"] == "controller_trajectory_thread"
    assert result["state"] == "accepted"
    assert result["max_motor_rpm"] == 100.0
    assert result["frequency_hz"] == 100
    assert result["duration_s"] >= 1.9
    assert "started" in thread_events
    assert "direct-write" not in thread_events


def test_handle_wait_for_idle_returns_completed_after_activity_quiesces(monkeypatch):
    statuses = iter(
        [
            {
                "accepted": True,
                "state": "executing",
                "completion_scope": "controller_program_thread",
                "trajectory_id": 0,
                "source_of_truth": "controller",
                "execution": {
                    "controller_thread_running": True,
                    "rtcore_status_present": False,
                },
            },
            {
                "accepted": True,
                "state": "accepted",
                "completion_scope": "rtcore_execution",
                "trajectory_id": 7,
                "source_of_truth": "rtcore",
                "execution": {
                    "controller_thread_running": False,
                    "rtcore_status_present": True,
                    "active_mode_name": "trajectory_execute",
                    "state_name": "queued",
                    "active_traj_id": 7,
                    "queue_depth": 1,
                    "motion_done": False,
                },
            },
            {
                "accepted": True,
                "state": "accepted",
                "completion_scope": "rtcore_execution",
                "trajectory_id": 7,
                "source_of_truth": "rtcore",
                "execution": {
                    "controller_thread_running": False,
                    "rtcore_status_present": True,
                    "active_mode_name": "idle",
                    "state_name": "idle",
                    "active_traj_id": 0,
                    "queue_depth": 0,
                    "motion_done": False,
                },
            },
        ]
    )

    monkeypatch.setattr(command_api, "get_motion_execution_status", lambda: next(statuses))
    monkeypatch.setattr(command_api.time, "sleep", lambda _seconds: None)

    result = command_api.handle_wait_for_idle(timeout_s=1.0)

    assert result["state"] == "completed"
    assert result["completion_scope"] == "rtcore_execution"
    assert result["source_of_truth"] == "rtcore"
    assert result["waited_for_motion"] is True
    assert result["wait_timeout_s"] == 1.0
    assert result["wait_timed_out"] is False
    assert result["execution"]["wait_terminal_state"] == "completed"
    assert result["execution"]["wait_last_active_state_name"] == "queued"


def test_handle_wait_for_idle_returns_timeout_when_motion_stays_active(monkeypatch):
    active_payload = {
        "accepted": True,
        "state": "accepted",
        "completion_scope": "rtcore_execution",
        "trajectory_id": 7,
        "source_of_truth": "rtcore",
        "execution": {
            "controller_thread_running": False,
            "rtcore_status_present": True,
            "active_mode_name": "trajectory_execute",
            "state_name": "queued",
            "active_traj_id": 7,
            "queue_depth": 1,
            "motion_done": False,
        },
    }
    monotonic_values = iter([0.0, 0.0, 0.03])

    monkeypatch.setattr(command_api, "get_motion_execution_status", lambda: dict(active_payload))
    monkeypatch.setattr(command_api.time, "monotonic", lambda: next(monotonic_values))
    monkeypatch.setattr(command_api.time, "sleep", lambda _seconds: None)

    result = command_api.handle_wait_for_idle(timeout_s=0.02)

    assert result["state"] == "timeout"
    assert result["completion_scope"] == "rtcore_execution"
    assert result["waited_for_motion"] is True
    assert result["wait_timeout_s"] == 0.02
    assert result["wait_timed_out"] is True
    assert result["execution"]["wait_terminal_state"] == "timeout"


def test_handle_wait_for_idle_treats_rtcore_completed_with_latched_traj_id_as_idle(monkeypatch):
    statuses = iter(
        [
            {
                "accepted": True,
                "state": "executing",
                "completion_scope": "rtcore_execution",
                "trajectory_id": 1,
                "source_of_truth": "rtcore",
                "execution": {
                    "controller_thread_running": False,
                    "rtcore_status_present": True,
                    "active_mode_name": "trajectory",
                    "state_name": "executing",
                    "active_traj_id": 1,
                    "queue_depth": 1,
                    "motion_done": False,
                },
            },
            {
                "accepted": True,
                "state": "completed",
                "completion_scope": "rtcore_execution",
                "trajectory_id": 1,
                "source_of_truth": "rtcore",
                "execution": {
                    "controller_thread_running": False,
                    "rtcore_status_present": True,
                    "active_mode_name": "trajectory",
                    "state_name": "completed",
                    "active_traj_id": 1,
                    "queue_depth": 0,
                    "motion_done": True,
                },
            },
        ]
    )

    monkeypatch.setattr(command_api, "get_motion_execution_status", lambda: next(statuses))
    monkeypatch.setattr(command_api.time, "sleep", lambda _seconds: None)

    result = command_api.handle_wait_for_idle(timeout_s=1.0)

    assert result["state"] == "completed"
    assert result["waited_for_motion"] is True
    assert result["wait_timed_out"] is False
    assert result["execution"]["wait_terminal_state"] == "completed"
    assert result["execution"]["wait_last_active_state_name"] == "executing"


def test_get_motion_execution_status_surfaces_terminal_program_summary(monkeypatch):
    class _Backend:
        @staticmethod
        def get_last_submitted_trajectory_id():
            return 0

    class _Status:
        active_traj_id = 0
        active_mode = 0
        active_mode_name = "idle"
        state = 0
        state_name = "idle"
        current_point_index = None
        queue_depth = 0
        queue_capacity = 4096
        last_event_code = 0
        underrun_count = 0
        stale_command = False
        motion_done = True
        capability_flags = 0
        active_command_seq = 0
        last_update_ns = 123

    monkeypatch.setattr(
        command_api.utils,
        "trajectory_state_snapshot",
        lambda: {
            "motion_state": "IDLE",
            "is_running": False,
            "last_correlation_id": None,
            "program_status": {
                "name": "alpha",
                "active": False,
                "state": "completed",
                "terminal_reason": "completed",
                "completed_step_count": 4,
                "completed_loop_count": 1,
                "step_count": 4,
                "move_steps": 3,
                "pause_steps": 1,
                "joint_move_steps": 0,
                "rtcore_segments": True,
                "segment_execution_policy": "rtcore_queued",
            },
        },
    )
    monkeypatch.setattr(
        command_api,
        "_get_rtcore_execution_backend_and_status",
        lambda: (_Backend(), _Status()),
    )

    result = command_api.get_motion_execution_status()

    assert result["state"] == "completed"
    assert result["completion_scope"] == "controller_program_thread"
    assert result["source_of_truth"] == "controller_program_thread"
    assert result["program_name"] == "alpha"
    assert result["program_state"] == "completed"
    assert result["program_completed_step_count"] == 4
    assert result["program_completed_loop_count"] == 1
    assert result["program"]["terminal_reason"] == "completed"
    assert result["program"]["segment_execution_policy"] == "rtcore_queued"


def test_build_motion_execution_metadata_refreshes_stale_rtcore_ack_snapshot(monkeypatch):
    class _Status:
        def __init__(self, *, active_traj_id, state_name, queue_depth, motion_done, last_update_ns):
            self.active_traj_id = active_traj_id
            self.active_mode = 2
            self.active_mode_name = "trajectory"
            self.state = 4 if state_name == "completed" else 1
            self.state_name = state_name
            self.current_point_index = 99 if active_traj_id else None
            self.queue_depth = queue_depth
            self.queue_capacity = 4096
            self.last_event_code = 291
            self.underrun_count = 0
            self.stale_command = False
            self.motion_done = motion_done
            self.capability_flags = 6
            self.active_command_seq = 12 if active_traj_id == 2 else 13
            self.last_update_ns = last_update_ns

    old_status = _Status(
        active_traj_id=2,
        state_name="completed",
        queue_depth=0,
        motion_done=True,
        last_update_ns=100,
    )
    fresh_status = _Status(
        active_traj_id=3,
        state_name="queued",
        queue_depth=1,
        motion_done=False,
        last_update_ns=200,
    )

    class _Backend:
        def __init__(self):
            self._submitted_values = iter([2, 3])
            self._statuses = iter([fresh_status])

        def get_last_submitted_trajectory_id(self):
            return next(self._submitted_values, 3)

        def get_execution_status(self):
            return next(self._statuses, fresh_status)

        @staticmethod
        def get_last_trajectory_timing():
            return {
                "requested_frequency_hz": 100,
                "effective_frequency_hz": 100,
                "cycle_ns": 1_000_000,
                "step_ns": 10_000_000,
                "cycles_per_point": 10,
            }

    monkeypatch.setattr(
        command_api.utils,
        "trajectory_state_snapshot",
        lambda: {
            "motion_state": "EXECUTING",
            "is_running": True,
            "last_correlation_id": "corr-123",
            "active_program_name": None,
        },
    )
    monkeypatch.setattr(
        command_api,
        "_get_rtcore_execution_backend_and_status",
        lambda: (_Backend(), old_status),
    )
    monkeypatch.setattr(command_api.time, "sleep", lambda _seconds: None)

    result = command_api._build_motion_execution_metadata(
        accepted=True,
        completion_scope="rtcore_execution",
        state="accepted",
        use_rtcore_status=True,
    )

    assert result["state"] == "accepted"
    assert result["trajectory_id"] == 3
    assert result["source_of_truth"] == "rtcore"
    assert result["execution"]["active_traj_id"] == 3
    assert result["execution"]["state_name"] == "queued"
    assert result["execution"]["queue_depth"] == 1
    assert result["execution"]["last_submitted_traj_id"] == 3


def test_realtime_jog_loop_uses_rtcore_joint_velocity_backend(monkeypatch):
    class _FakeRTCoreJogBackend:
        def __init__(self):
            self.calls: list[tuple[str, list[float] | float]] = []

        def send_realtime_jog_command(self, joint_velocities_rad_s, *, timeout_s):
            self.calls.append(("send", list(joint_velocities_rad_s)))
            self.calls.append(("timeout", float(timeout_s)))
            command_api.utils.trajectory_state["is_jogging"] = False

        def stop_realtime_jog(self):
            self.calls.append(("stop", 0.0))

    backend = _FakeRTCoreJogBackend()
    direct_write_calls: list[object] = []

    monkeypatch.setattr(
        command_api.utils,
        "LOGICAL_JOINT_LIMITS_RAD",
        [(-1.0, 1.0)] * 6,
        raising=False,
    )
    command_api.utils.trajectory_state.update(
        {
            "is_jogging": True,
            "is_running": False,
            "jog_deadman": True,
            "jog_debug": False,
            "jog_velocities": np.array([0.01, 0.0, 0.0, 0.0, 0.0, 0.0], dtype=float),
            "jog_gripper_velocity_deg_s": 0.0,
            "last_jog_command_time": command_api.time.monotonic(),
            "jog_thread": None,
        }
    )
    monkeypatch.setattr(command_api, "_get_rtcore_jog_backend", lambda: backend)
    monkeypatch.setattr(command_api.utils, "gripper_present", False, raising=False)
    monkeypatch.setattr(command_api.time, "sleep", lambda _seconds: None)
    monkeypatch.setattr(
        command_api.servo_driver,
        "get_current_arm_state_rad",
        lambda verbose=False: [0.0] * 6,
    )
    monkeypatch.setattr(
        command_api.ik_solver,
        "get_fk_matrix",
        lambda q_current: np.eye(4, dtype=float),
    )
    monkeypatch.setattr(
        command_api.ik_solver,
        "solve_ik",
        lambda **_kwargs: [0.05] * 6,
    )
    monkeypatch.setattr(
        command_api.servo_driver,
        "set_servo_positions",
        lambda *args, **kwargs: direct_write_calls.append((args, kwargs)),
    )

    command_api._jog_controller_thread()

    send_calls = [entry for entry in backend.calls if entry[0] == "send"]
    timeout_calls = [entry for entry in backend.calls if entry[0] == "timeout"]
    assert len(send_calls) == 1
    assert len(send_calls[0][1]) == 6
    assert any(abs(float(value)) > 0.0 for value in send_calls[0][1])
    assert timeout_calls == [("timeout", command_api.JOG_VELOCITY_TIMEOUT_S)]
    assert direct_write_calls == []


def test_handle_move_line_forces_rtcore_path_when_closed_loop_requested(monkeypatch):
    executor_calls: list[tuple[str, int]] = []

    class _FakeThread:
        def __init__(self, *, target, kwargs, daemon):
            self._target = target
            self._kwargs = kwargs
            self.daemon = daemon

        def start(self):
            self._target(**self._kwargs)

    monkeypatch.setattr(command_api, "_backend_supports_rtcore_execution", lambda: True)
    monkeypatch.setattr(command_api.utils, "trajectory_state", {"is_running": False, "should_stop": False, "thread": None})
    monkeypatch.setattr(command_api.utils, "trajectory_state_get", lambda key, default=None: command_api.utils.trajectory_state.get(key, default))
    monkeypatch.setattr(command_api.utils, "trajectory_state_update", lambda **kwargs: command_api.utils.trajectory_state.update(kwargs))
    monkeypatch.setattr(command_api.utils, "trajectory_state_snapshot", lambda: {"motion_state": "IDLE", "is_running": False, "last_correlation_id": None})
    monkeypatch.setattr(command_api.utils, "set_motion_state", lambda _state: None)
    monkeypatch.setattr(command_api.servo_driver, "get_current_arm_state_rad", lambda verbose=False: [0.0] * 6)
    monkeypatch.setattr(
        command_api.trajectory_execution,
        "_plan_smooth_move",
        lambda **_kwargs: [[0.0] * 6, [0.1] * 6],
    )
    monkeypatch.setattr(
        command_api.trajectory_execution,
        "_open_loop_executor_thread",
        lambda **kwargs: executor_calls.append(("open", int(kwargs["frequency"]))),
    )
    monkeypatch.setattr(
        command_api.trajectory_execution,
        "_closed_loop_executor_thread",
        lambda **kwargs: executor_calls.append(("closed", int(kwargs["frequency"]))),
    )
    monkeypatch.setattr(command_api.threading, "Thread", _FakeThread)

    result = command_api.handle_move_line(0.1, 0.2, 0.3, 0.2, 0.1, closed_loop=True)

    assert result["completion_scope"] == "rtcore_execution"
    assert result["closed_loop_requested"] is True
    assert result["closed_loop_effective"] is False
    assert result["closed_loop"] is False
    assert result["execution_policy"] == "rtcore_queued"
    assert result["frequency_hz"] == 100
    assert executor_calls == [("open", 100)]


def test_handle_set_orientation_forces_rtcore_path_when_closed_loop_requested(monkeypatch):
    executor_calls: list[tuple[str, int]] = []

    class _FakeThread:
        def __init__(self, *, target, daemon):
            self._target = target
            self.daemon = daemon

        def start(self):
            self._target()

        def join(self):
            return None

    monkeypatch.setattr(command_api, "_backend_supports_rtcore_execution", lambda: True)
    monkeypatch.setattr(command_api.utils, "trajectory_state", {"is_running": False, "should_stop": False, "thread": None})
    monkeypatch.setattr(command_api.utils, "trajectory_state_update", lambda **kwargs: command_api.utils.trajectory_state.update(kwargs))
    monkeypatch.setattr(command_api.utils, "trajectory_state_snapshot", lambda: {"motion_state": "IDLE", "is_running": False, "last_correlation_id": None})
    monkeypatch.setattr(command_api.utils, "set_motion_state", lambda _state: None)
    monkeypatch.setattr(
        command_api,
        "_get_live_pose_snapshot",
        lambda: (
            np.zeros(6, dtype=float),
            np.array([0.1, 0.2, 0.3], dtype=float),
            np.eye(3, dtype=float),
        ),
    )
    monkeypatch.setattr(
        command_api.ik_solver,
        "solve_ik_path_batch",
        lambda **_kwargs: [np.zeros(6, dtype=float), np.full(6, 0.1, dtype=float)],
    )
    monkeypatch.setattr(
        command_api.ik_solver,
        "get_fk_matrix",
        lambda _joints: np.eye(4, dtype=float),
    )
    monkeypatch.setattr(
        command_api.trajectory_execution,
        "_open_loop_executor_thread",
        lambda **kwargs: executor_calls.append(("open", int(kwargs["frequency"]))),
    )
    monkeypatch.setattr(
        command_api.trajectory_execution,
        "_closed_loop_executor_thread",
        lambda **kwargs: executor_calls.append(("closed", int(kwargs["frequency"]))),
    )
    monkeypatch.setattr(command_api.threading, "Thread", _FakeThread)

    result = command_api.handle_set_orientation_command(10.0, 20.0, 30.0, closed_loop=True, duration_s=1.0)

    assert result["completion_scope"] == "rtcore_execution"
    assert result["closed_loop_requested"] is True
    assert result["closed_loop_effective"] is False
    assert result["closed_loop"] is False
    assert result["execution_policy"] == "rtcore_queued"
    assert result["frequency_hz"] == 100
    assert executor_calls == [("open", 100)]


def test_handle_move_line_preserves_closed_loop_on_non_rtcore_backend(monkeypatch):
    executor_calls: list[tuple[str, int]] = []

    class _FakeThread:
        def __init__(self, *, target, kwargs, daemon):
            self._target = target
            self._kwargs = kwargs
            self.daemon = daemon

        def start(self):
            self._target(**self._kwargs)

    monkeypatch.setattr(command_api, "_backend_supports_rtcore_execution", lambda: False)
    monkeypatch.setattr(command_api.utils, "trajectory_state", {"is_running": False, "should_stop": False, "thread": None})
    monkeypatch.setattr(command_api.utils, "trajectory_state_get", lambda key, default=None: command_api.utils.trajectory_state.get(key, default))
    monkeypatch.setattr(command_api.utils, "trajectory_state_update", lambda **kwargs: command_api.utils.trajectory_state.update(kwargs))
    monkeypatch.setattr(command_api.utils, "trajectory_state_snapshot", lambda: {"motion_state": "IDLE", "is_running": False, "last_correlation_id": None})
    monkeypatch.setattr(command_api.utils, "set_motion_state", lambda _state: None)
    monkeypatch.setattr(command_api.servo_driver, "get_current_arm_state_rad", lambda verbose=False: [0.0] * 6)
    monkeypatch.setattr(
        command_api.trajectory_execution,
        "_plan_smooth_move",
        lambda **_kwargs: [[0.0] * 6, [0.1] * 6],
    )
    monkeypatch.setattr(
        command_api.trajectory_execution,
        "_open_loop_executor_thread",
        lambda **kwargs: executor_calls.append(("open", int(kwargs["frequency"]))),
    )
    monkeypatch.setattr(
        command_api.trajectory_execution,
        "_closed_loop_executor_thread",
        lambda **kwargs: executor_calls.append(("closed", int(kwargs["frequency"]))),
    )
    monkeypatch.setattr(command_api.threading, "Thread", _FakeThread)

    result = command_api.handle_move_line(0.1, 0.2, 0.3, 0.2, 0.1, closed_loop=True)

    assert result["completion_scope"] == "controller_closed_loop"
    assert result["closed_loop_requested"] is True
    assert result["closed_loop_effective"] is True
    assert result["closed_loop"] is True
    assert result["execution_policy"] == "controller_closed_loop"
    assert result["frequency_hz"] == 50
    assert executor_calls == [("closed", 50)]


def test_handle_set_orientation_preserves_closed_loop_on_non_rtcore_backend(monkeypatch):
    executor_calls: list[tuple[str, int]] = []

    class _FakeThread:
        def __init__(self, *, target, daemon):
            self._target = target
            self.daemon = daemon

        def start(self):
            self._target()

        def join(self):
            return None

    monkeypatch.setattr(command_api, "_backend_supports_rtcore_execution", lambda: False)
    monkeypatch.setattr(command_api.utils, "trajectory_state", {"is_running": False, "should_stop": False, "thread": None})
    monkeypatch.setattr(command_api.utils, "trajectory_state_update", lambda **kwargs: command_api.utils.trajectory_state.update(kwargs))
    monkeypatch.setattr(command_api.utils, "trajectory_state_snapshot", lambda: {"motion_state": "IDLE", "is_running": False, "last_correlation_id": None})
    monkeypatch.setattr(command_api.utils, "set_motion_state", lambda _state: None)
    monkeypatch.setattr(
        command_api,
        "_get_live_pose_snapshot",
        lambda: (
            np.zeros(6, dtype=float),
            np.array([0.1, 0.2, 0.3], dtype=float),
            np.eye(3, dtype=float),
        ),
    )
    monkeypatch.setattr(
        command_api.ik_solver,
        "solve_ik_path_batch",
        lambda **_kwargs: [np.zeros(6, dtype=float), np.full(6, 0.1, dtype=float)],
    )
    monkeypatch.setattr(
        command_api.ik_solver,
        "get_fk_matrix",
        lambda _joints: np.eye(4, dtype=float),
    )
    monkeypatch.setattr(
        command_api.trajectory_execution,
        "_open_loop_executor_thread",
        lambda **kwargs: executor_calls.append(("open", int(kwargs["frequency"]))),
    )
    monkeypatch.setattr(
        command_api.trajectory_execution,
        "_closed_loop_executor_thread",
        lambda **kwargs: executor_calls.append(("closed", int(kwargs["frequency"]))),
    )
    monkeypatch.setattr(command_api.threading, "Thread", _FakeThread)

    result = command_api.handle_set_orientation_command(10.0, 20.0, 30.0, closed_loop=True, duration_s=1.0)

    assert result["completion_scope"] == "controller_closed_loop"
    assert result["closed_loop_requested"] is True
    assert result["closed_loop_effective"] is True
    assert result["closed_loop"] is True
    assert result["execution_policy"] == "controller_closed_loop"
    assert result["frequency_hz"] == 50
    assert executor_calls == [("closed", 50)]
