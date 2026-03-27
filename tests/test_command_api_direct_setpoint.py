import numpy as np

from gradient_os.arm_controller import command_api


def test_plan_preview_trajectory_points_prefers_live_joint_feedback(monkeypatch, tmp_path):
    monkeypatch.setattr(command_api, "RECORDED_TRAJ_DIR", str(tmp_path / "recorded"))
    monkeypatch.setattr(command_api.utils, "TRAJECTORY_CACHE_DIR", str(tmp_path / "cache"))
    monkeypatch.setattr(
        command_api.utils,
        "current_logical_joint_angles_rad",
        [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        raising=False,
    )

    live_q = [0.3, -0.2, 0.1, 0.4, -0.1, 0.2]
    captured: dict[str, list[float] | None] = {"start_q": None}

    monkeypatch.setattr(
        command_api.servo_driver,
        "get_current_arm_state_rad",
        lambda verbose=False: list(live_q),
    )
    monkeypatch.setattr(
        command_api.servo_driver.servo_protocol,
        "get_present_servo_ids",
        lambda: {10, 20, 30, 40, 50, 60},
    )
    monkeypatch.setattr(command_api.ik_solver, "get_fk", lambda _q: [0.0, 0.0, 0.0])

    def _fake_fk_matrix(q):
        matrix = np.eye(4, dtype=float)
        q_arr = np.asarray(q, dtype=float)
        matrix[:3, 3] = np.array(
            [
                float(q_arr[0]) if q_arr.size > 0 else 0.0,
                float(q_arr[1]) if q_arr.size > 1 else 0.0,
                float(q_arr[2]) if q_arr.size > 2 else 0.0,
            ],
            dtype=float,
        )
        return matrix

    monkeypatch.setattr(command_api.ik_solver, "get_fk_matrix", _fake_fk_matrix)

    def _fake_plan_joint_move_to_pose(current_q, **kwargs):
        captured["start_q"] = list(np.asarray(current_q, dtype=float).tolist())
        return [list(np.asarray(current_q, dtype=float).tolist())], list(np.asarray(current_q, dtype=float).tolist())

    monkeypatch.setattr(command_api, "_plan_joint_move_to_pose", _fake_plan_joint_move_to_pose)

    payload = command_api.plan_preview_trajectory_points(
        points=[[0.1, 0.2, 0.3]],
        preview_name="__live_joint_preview__",
        pose_waypoints=[
            {
                "x": 0.1,
                "y": 0.2,
                "z": 0.3,
                "move_type": "joint",
            }
        ],
    )

    assert captured["start_q"] == live_q
    assert payload["trajectory"]["moves"][0]["command"] == "move"


def test_plan_preview_trajectory_points_uses_cached_controller_joints_when_local_feedback_unavailable(
    monkeypatch,
    tmp_path,
):
    monkeypatch.setattr(command_api, "RECORDED_TRAJ_DIR", str(tmp_path / "recorded"))
    monkeypatch.setattr(command_api.utils, "TRAJECTORY_CACHE_DIR", str(tmp_path / "cache"))
    cached_q = [0.3, -0.2, 0.1, 0.4, -0.1, 0.2]
    monkeypatch.setattr(
        command_api.utils,
        "current_logical_joint_angles_rad",
        list(cached_q),
        raising=False,
    )

    live_read_calls: list[bool] = []
    captured: dict[str, list[float] | None] = {"start_q": None}

    def _no_backend():
        raise RuntimeError("backend unavailable")

    monkeypatch.setattr(command_api.backend_registry, "get_active_backend", _no_backend)
    monkeypatch.setattr(
        command_api.servo_driver.servo_protocol,
        "get_present_servo_ids",
        lambda: set(),
    )
    monkeypatch.setattr(
        command_api.servo_driver,
        "get_current_arm_state_rad",
        lambda verbose=False: live_read_calls.append(bool(verbose)) or [0.0] * 6,
    )
    monkeypatch.setattr(command_api.ik_solver, "get_fk", lambda _q: [0.0, 0.0, 0.0])

    def _fake_fk_matrix(q):
        matrix = np.eye(4, dtype=float)
        q_arr = np.asarray(q, dtype=float)
        matrix[:3, 3] = np.array(
            [
                float(q_arr[0]) if q_arr.size > 0 else 0.0,
                float(q_arr[1]) if q_arr.size > 1 else 0.0,
                float(q_arr[2]) if q_arr.size > 2 else 0.0,
            ],
            dtype=float,
        )
        return matrix

    monkeypatch.setattr(command_api.ik_solver, "get_fk_matrix", _fake_fk_matrix)

    def _fake_plan_joint_move_to_pose(current_q, **kwargs):
        captured["start_q"] = list(np.asarray(current_q, dtype=float).tolist())
        return [list(np.asarray(current_q, dtype=float).tolist())], list(np.asarray(current_q, dtype=float).tolist())

    monkeypatch.setattr(command_api, "_plan_joint_move_to_pose", _fake_plan_joint_move_to_pose)

    payload = command_api.plan_preview_trajectory_points(
        points=[[0.1, 0.2, 0.3]],
        preview_name="__cached_joint_preview__",
        pose_waypoints=[
            {
                "x": 0.1,
                "y": 0.2,
                "z": 0.3,
                "move_type": "joint",
            }
        ],
    )

    assert captured["start_q"] == cached_q
    assert live_read_calls == []
    assert payload["trajectory"]["moves"][0]["command"] == "move"


def test_plan_preview_trajectory_points_passes_joint_rotation_speed_and_serializes_it(
    monkeypatch,
    tmp_path,
):
    monkeypatch.setattr(command_api, "RECORDED_TRAJ_DIR", str(tmp_path / "recorded"))
    monkeypatch.setattr(command_api.utils, "TRAJECTORY_CACHE_DIR", str(tmp_path / "cache"))
    monkeypatch.setattr(
        command_api.utils,
        "current_logical_joint_angles_rad",
        [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        raising=False,
    )
    monkeypatch.setattr(
        command_api.servo_driver,
        "get_current_arm_state_rad",
        lambda verbose=False: [0.0] * 6,
    )
    monkeypatch.setattr(
        command_api.servo_driver.servo_protocol,
        "get_present_servo_ids",
        lambda: {10, 20, 30, 40, 50, 60},
    )
    monkeypatch.setattr(command_api.ik_solver, "get_fk", lambda _q: [0.0, 0.0, 0.0])

    def _fake_fk_matrix(q):
        matrix = np.eye(4, dtype=float)
        q_arr = np.asarray(q, dtype=float)
        matrix[:3, 3] = np.array(
            [
                float(q_arr[0]) if q_arr.size > 0 else 0.0,
                float(q_arr[1]) if q_arr.size > 1 else 0.0,
                float(q_arr[2]) if q_arr.size > 2 else 0.0,
            ],
            dtype=float,
        )
        return matrix

    monkeypatch.setattr(command_api.ik_solver, "get_fk_matrix", _fake_fk_matrix)
    captured: dict[str, float | None] = {"max_joint_deg_s": None}

    def _fake_plan_joint_move_to_pose(current_q, **kwargs):
        captured["max_joint_deg_s"] = kwargs.get("max_joint_deg_s")
        return [[0.0] * 6, [0.1, 0.2, 0.3, 0.0, 0.0, 0.0]], [0.1, 0.2, 0.3, 0.0, 0.0, 0.0]

    monkeypatch.setattr(command_api, "_plan_joint_move_to_pose", _fake_plan_joint_move_to_pose)

    payload = command_api.plan_preview_trajectory_points(
        points=[[0.1, 0.2, 0.3]],
        preview_name="__joint_speed_preview__",
        pose_waypoints=[
            {
                "x": 0.1,
                "y": 0.2,
                "z": 0.3,
                "move_type": "joint",
                "rotation_speed_deg_s": 12.5,
            }
        ],
    )

    assert captured["max_joint_deg_s"] == 12.5
    assert payload["trajectory"]["moves"][0]["command"] == "move"
    assert payload["trajectory"]["moves"][0]["rotation_speed_deg_s"] == 12.5
    assert payload["waypoints"][0]["rotation_speed_deg_s"] == 12.5


def test_plan_preview_trajectory_points_uses_orientation_only_speed_for_pure_rotation(
    monkeypatch,
    tmp_path,
):
    monkeypatch.setattr(command_api, "RECORDED_TRAJ_DIR", str(tmp_path / "recorded"))
    monkeypatch.setattr(command_api.utils, "TRAJECTORY_CACHE_DIR", str(tmp_path / "cache"))
    monkeypatch.setattr(
        command_api.utils,
        "current_logical_joint_angles_rad",
        [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        raising=False,
    )
    monkeypatch.setattr(
        command_api.servo_driver,
        "get_current_arm_state_rad",
        lambda verbose=False: [0.0] * 6,
    )
    monkeypatch.setattr(
        command_api.servo_driver.servo_protocol,
        "get_present_servo_ids",
        lambda: {10, 20, 30, 40, 50, 60},
    )
    monkeypatch.setattr(command_api.ik_solver, "get_fk", lambda _q: [0.0, 0.0, 0.0])

    def _fake_fk_matrix(q):
        matrix = np.eye(4, dtype=float)
        q_arr = np.asarray(q, dtype=float)
        if q_arr.size > 0 and abs(float(q_arr[0]) - 0.1) < 1e-9:
            matrix[:3, :3] = command_api.R.from_euler("z", 45.0, degrees=True).as_matrix()
        return matrix

    monkeypatch.setattr(command_api.ik_solver, "get_fk_matrix", _fake_fk_matrix)
    monkeypatch.setattr(
        command_api.trajectory_execution,
        "_plan_linear_move",
        lambda *args, **kwargs: (_ for _ in ()).throw(AssertionError("linear planner should not run")),
    )
    captured: dict[str, float | None] = {"angular_speed_deg_s": None}

    def _fake_plan_orientation_only_move(current_q, **kwargs):
        captured["angular_speed_deg_s"] = kwargs.get("angular_speed_deg_s")
        return [[0.0] * 6, [0.1, 0.0, 0.0, 0.0, 0.0, 0.0]]

    monkeypatch.setattr(command_api, "_plan_orientation_only_move", _fake_plan_orientation_only_move)

    payload = command_api.plan_preview_trajectory_points(
        points=[[0.0, 0.0, 0.0]],
        preview_name="__pure_rotation_preview__",
        pose_waypoints=[
            {
                "x": 0.0,
                "y": 0.0,
                "z": 0.0,
                "move_type": "linear",
                "orientation_euler_deg": {"roll": 0.0, "pitch": 0.0, "yaw": 45.0},
                "rotation_speed_deg_s": 20.0,
            }
        ],
    )

    assert captured["angular_speed_deg_s"] == 20.0
    assert payload["trajectory"]["moves"][0]["command"] == "move_absolute"
    assert payload["trajectory"]["moves"][0]["rotation_speed_deg_s"] == 20.0
    assert payload["waypoints"][0]["rotation_speed_deg_s"] == 20.0


def test_resolve_profile_params_for_linear_speed_defaults_to_one_second_ramp(monkeypatch):
    monkeypatch.setattr(command_api.utils, "DEFAULT_PROFILE_VELOCITY", 0.1)
    monkeypatch.setattr(command_api.utils, "DEFAULT_PROFILE_ACCELERATION", 0.05)

    multiplier, velocity, acceleration = command_api._resolve_profile_params_for_linear_speed_m_s(0.4)

    assert multiplier == 4.0
    assert velocity == 0.4
    assert acceleration == 0.4


def test_resolve_profile_params_for_linear_speed_uses_explicit_acceleration_override(monkeypatch):
    monkeypatch.setattr(command_api.utils, "DEFAULT_PROFILE_VELOCITY", 0.1)
    monkeypatch.setattr(command_api.utils, "DEFAULT_PROFILE_ACCELERATION", 0.05)

    multiplier, velocity, acceleration = command_api._resolve_profile_params_for_linear_speed_m_s(0.4, 0.9)

    assert multiplier == 4.0
    assert velocity == 0.4
    assert acceleration == 0.9


def test_plan_preview_trajectory_points_defaults_linear_acceleration_to_one_second_ramp(
    monkeypatch,
    tmp_path,
):
    monkeypatch.setattr(command_api, "RECORDED_TRAJ_DIR", str(tmp_path / "recorded"))
    monkeypatch.setattr(command_api.utils, "TRAJECTORY_CACHE_DIR", str(tmp_path / "cache"))
    monkeypatch.setattr(command_api.utils, "DEFAULT_PROFILE_VELOCITY", 0.1)
    monkeypatch.setattr(command_api.utils, "DEFAULT_PROFILE_ACCELERATION", 0.05)
    monkeypatch.setattr(
        command_api.utils,
        "current_logical_joint_angles_rad",
        [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        raising=False,
    )
    monkeypatch.setattr(
        command_api.servo_driver,
        "get_current_arm_state_rad",
        lambda verbose=False: [0.0] * 6,
    )
    monkeypatch.setattr(
        command_api.servo_driver.servo_protocol,
        "get_present_servo_ids",
        lambda: {10, 20, 30, 40, 50, 60},
    )
    monkeypatch.setattr(command_api.ik_solver, "get_fk", lambda _q: [0.0, 0.0, 0.0])

    def _fake_fk_matrix(_q):
        return np.eye(4, dtype=float)

    monkeypatch.setattr(command_api.ik_solver, "get_fk_matrix", _fake_fk_matrix)
    captured: dict[str, float | None] = {"velocity": None, "acceleration": None}

    def _fake_plan_linear_move(current_q, target_pos, velocity, acceleration, *args, **kwargs):
        captured["velocity"] = float(velocity)
        captured["acceleration"] = float(acceleration)
        return [list(np.asarray(current_q, dtype=float).tolist()), [0.1, 0.0, 0.0, 0.0, 0.0, 0.0]]

    monkeypatch.setattr(command_api.trajectory_execution, "_plan_linear_move", _fake_plan_linear_move)

    payload = command_api.plan_preview_trajectory_points(
        points=[[0.1, 0.0, 0.0]],
        preview_name="__linear_speed_override_preview__",
        pose_waypoints=[
            {
                "x": 0.1,
                "y": 0.0,
                "z": 0.0,
                "move_type": "linear",
                "linear_speed_mm_s": 400.0,
            }
        ],
    )

    assert captured["velocity"] == 0.4
    assert captured["acceleration"] == 0.4
    assert payload["trajectory"]["moves"][0]["linear_speed_mm_s"] == 400.0
    assert payload["trajectory"]["moves"][0]["linear_acceleration_mm_s2"] == 400.0
    assert payload["waypoints"][0]["linear_speed_mm_s"] == 400.0
    assert payload["waypoints"][0]["linear_acceleration_mm_s2"] == 400.0


def test_plan_preview_trajectory_points_passes_explicit_linear_acceleration_override(
    monkeypatch,
    tmp_path,
):
    monkeypatch.setattr(command_api, "RECORDED_TRAJ_DIR", str(tmp_path / "recorded"))
    monkeypatch.setattr(command_api.utils, "TRAJECTORY_CACHE_DIR", str(tmp_path / "cache"))
    monkeypatch.setattr(command_api.utils, "DEFAULT_PROFILE_VELOCITY", 0.1)
    monkeypatch.setattr(command_api.utils, "DEFAULT_PROFILE_ACCELERATION", 0.05)
    monkeypatch.setattr(
        command_api.utils,
        "current_logical_joint_angles_rad",
        [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        raising=False,
    )
    monkeypatch.setattr(
        command_api.servo_driver,
        "get_current_arm_state_rad",
        lambda verbose=False: [0.0] * 6,
    )
    monkeypatch.setattr(
        command_api.servo_driver.servo_protocol,
        "get_present_servo_ids",
        lambda: {10, 20, 30, 40, 50, 60},
    )
    monkeypatch.setattr(command_api.ik_solver, "get_fk", lambda _q: [0.0, 0.0, 0.0])

    def _fake_fk_matrix(_q):
        return np.eye(4, dtype=float)

    monkeypatch.setattr(command_api.ik_solver, "get_fk_matrix", _fake_fk_matrix)
    captured: dict[str, float | None] = {"velocity": None, "acceleration": None}

    def _fake_plan_linear_move(current_q, target_pos, velocity, acceleration, *args, **kwargs):
        captured["velocity"] = float(velocity)
        captured["acceleration"] = float(acceleration)
        return [list(np.asarray(current_q, dtype=float).tolist()), [0.1, 0.0, 0.0, 0.0, 0.0, 0.0]]

    monkeypatch.setattr(command_api.trajectory_execution, "_plan_linear_move", _fake_plan_linear_move)

    payload = command_api.plan_preview_trajectory_points(
        points=[[0.1, 0.0, 0.0]],
        preview_name="__linear_acceleration_override_preview__",
        pose_waypoints=[
            {
                "x": 0.1,
                "y": 0.0,
                "z": 0.0,
                "move_type": "linear",
                "linear_speed_mm_s": 400.0,
                "linear_acceleration_mm_s2": 900.0,
            }
        ],
    )

    assert captured["velocity"] == 0.4
    assert captured["acceleration"] == 0.9
    assert payload["trajectory"]["moves"][0]["linear_speed_mm_s"] == 400.0
    assert payload["trajectory"]["moves"][0]["linear_acceleration_mm_s2"] == 900.0
    assert payload["waypoints"][0]["linear_speed_mm_s"] == 400.0
    assert payload["waypoints"][0]["linear_acceleration_mm_s2"] == 900.0


def test_plan_preview_trajectory_points_serializes_explicit_pause_between_authored_moves(
    monkeypatch,
    tmp_path,
):
    monkeypatch.setattr(command_api, "RECORDED_TRAJ_DIR", str(tmp_path / "recorded"))
    monkeypatch.setattr(command_api.utils, "TRAJECTORY_CACHE_DIR", str(tmp_path / "cache"))
    monkeypatch.setattr(
        command_api.utils,
        "current_logical_joint_angles_rad",
        [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        raising=False,
    )
    monkeypatch.setattr(
        command_api.servo_driver,
        "get_current_arm_state_rad",
        lambda verbose=False: [0.0] * 6,
    )
    monkeypatch.setattr(
        command_api.servo_driver.servo_protocol,
        "get_present_servo_ids",
        lambda: {10, 20, 30, 40, 50, 60},
    )
    monkeypatch.setattr(command_api.ik_solver, "get_fk", lambda _q: [0.0, 0.0, 0.0])

    def _fake_fk_matrix(q):
        matrix = np.eye(4, dtype=float)
        q_arr = np.asarray(q, dtype=float)
        matrix[:3, 3] = np.array(
            [
                float(q_arr[0]) if q_arr.size > 0 else 0.0,
                float(q_arr[1]) if q_arr.size > 1 else 0.0,
                float(q_arr[2]) if q_arr.size > 2 else 0.0,
            ],
            dtype=float,
        )
        return matrix

    monkeypatch.setattr(command_api.ik_solver, "get_fk_matrix", _fake_fk_matrix)
    call_index = {"value": 0}

    def _fake_plan_linear_move(current_q, target_pos, velocity, acceleration, *args, **kwargs):
        call_index["value"] += 1
        planned_q = [float(call_index["value"]), 0.0, 0.0, 0.0, 0.0, 0.0]
        return [list(np.asarray(current_q, dtype=float).tolist()), planned_q]

    monkeypatch.setattr(command_api.trajectory_execution, "_plan_linear_move", _fake_plan_linear_move)

    payload = command_api.plan_preview_trajectory_points(
        points=[[0.1, 0.0, 0.0], [0.2, 0.0, 0.0], [0.3, 0.0, 0.0]],
        preview_name="__pause_preview__",
        pose_waypoints=[
            {"x": 0.1, "y": 0.0, "z": 0.0, "move_type": "linear"},
            {"x": 0.2, "y": 0.0, "z": 0.0, "move_type": "linear", "pause_after_s": 0.75},
            {"x": 0.3, "y": 0.0, "z": 0.0, "move_type": "linear"},
        ],
    )

    assert [move["command"] for move in payload["trajectory"]["moves"]] == [
        "move_absolute",
        "move_absolute",
        "pause",
        "move_absolute",
    ]
    assert payload["trajectory"]["moves"][2]["duration"] == 0.75
    assert payload["waypoints"][1]["pause_after_s"] == 0.75


def test_collapse_runtime_move_pause_steps_compiles_holds_into_single_path():
    collapsed = command_api._collapse_runtime_move_pause_steps(
        [
            {"type": "move", "path": [[0.0, 0.0], [0.1, 0.0]], "freq": 100},
            {"type": "pause", "duration": 0.1},
            {"type": "move", "path": [[0.1, 0.0], [0.2, 0.0]], "freq": 100},
        ]
    )

    assert collapsed is not None
    assert len(collapsed) == 1
    step = collapsed[0]
    assert step["type"] == "move"
    assert step["freq"] == 100
    assert step["logical_step_count"] == 3
    assert len(step["path"]) == 13
    assert step["path"][0] == [0.0, 0.0]
    assert step["path"][1] == [0.1, 0.0]
    assert step["path"][2:12] == [[0.1, 0.0]] * 10
    assert step["path"][12] == [0.2, 0.0]


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
        "safe_for_power_transition": True,
        "power_transition_blockers": [],
        "power_transition_blocker_details": [],
        "execution": {
            "controller_motion_state": "idle",
            "controller_thread_running": False,
            "last_correlation_id": None,
            "rtcore_status_present": False,
            "safe_for_power_transition": True,
            "power_transition_blockers": [],
            "power_transition_blocker_details": [],
            "power_transition_feedback_synchronized": False,
            "power_transition_faulted_axis_count": 0,
            "power_transition_active_jog": False,
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
        def __init__(self, manager):
            self.calls: list[tuple[str, list[float] | float]] = []
            self._manager = manager

        def start_joint_velocity_lease_jog(self, *, timeout_s):
            self.calls.append(("start", float(timeout_s)))

        def update_joint_velocity_lease_jog(self, joint_velocities_rad_s, *, timeout_s):
            self.calls.append(("send", list(joint_velocities_rad_s)))
            self.calls.append(("timeout", float(timeout_s)))
            self._manager.stop_session(
                session_id="test-session",
                owner_id="test-owner",
                reason="test-stop",
                allow_missing=True,
            )

        def stop_joint_velocity_lease_jog(self, *, quick_stop=False):
            self.calls.append(("stop", 0.0))

    fresh_manager = command_api.JogSessionManager()
    fresh_manager.start_session(
        owner_id="test-owner",
        seq=0,
        lease_timeout_s=command_api.JOG_CONTROLLER_LEASE_TIMEOUT_S,
        deadman=True,
        velocity_vector=(0.01, 0.0, 0.0, 0.0, 0.0, 0.0),
        backend_mode="joint_velocity_lease",
        backend_timeout_s=command_api.JOG_VELOCITY_TIMEOUT_S,
        session_id="test-session",
    )
    backend = _FakeRTCoreJogBackend(fresh_manager)
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
    monkeypatch.setattr(command_api, "_JOG_SESSION_MANAGER", fresh_manager)
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
    assert timeout_calls == [("timeout", command_api.JOG_BACKEND_LEASE_TIMEOUT_S)]
    assert direct_write_calls == []


def test_realtime_jog_loop_pure_rotation_keeps_commanded_xyz_fixed(monkeypatch):
    fresh_manager = command_api.JogSessionManager()
    fresh_manager.start_session(
        owner_id="test-owner",
        seq=0,
        lease_timeout_s=command_api.JOG_CONTROLLER_LEASE_TIMEOUT_S,
        deadman=True,
        velocity_vector=(0.0, 0.0, 0.0, 0.0, 0.0, 15.0),
        backend_mode="controller_cartesian_loop",
        session_id="test-session",
    )
    solve_calls: list[dict[str, list[float]]] = []
    final_measured_state = [0.2, 0.0, 0.0, 0.0, 0.0, 0.0]
    measured_states = iter(
        [
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            final_measured_state,
            final_measured_state,
        ]
    )
    direct_write_calls: list[list[float]] = []
    sleep_calls = 0

    monkeypatch.setattr(command_api, "_JOG_SESSION_MANAGER", fresh_manager)
    monkeypatch.setattr(command_api, "_get_rtcore_jog_backend", lambda: None)
    monkeypatch.setattr(command_api.utils, "NUM_LOGICAL_JOINTS", 6, raising=False)
    monkeypatch.setattr(command_api.utils, "LOGICAL_JOINT_LIMITS_RAD", [(-3.14, 3.14)] * 6, raising=False)
    monkeypatch.setattr(command_api.utils, "gripper_present", False, raising=False)
    
    def _sleep_and_stop(_seconds):
        nonlocal sleep_calls
        sleep_calls += 1
        if sleep_calls >= 2:
            fresh_manager.stop_session(
                session_id="test-session",
                owner_id="test-owner",
                reason="test-stop",
                allow_missing=True,
            )

    monkeypatch.setattr(command_api.time, "sleep", _sleep_and_stop)
    monkeypatch.setattr(
        command_api.servo_driver,
        "get_current_arm_state_rad",
        lambda verbose=False: next(measured_states, final_measured_state),
    )

    def _fake_fk_matrix(joints):
        pose = np.eye(4, dtype=float)
        pose[0, 3] = float(np.asarray(joints, dtype=float)[0])
        return pose

    monkeypatch.setattr(command_api.ik_solver, "get_fk_matrix", _fake_fk_matrix)

    def _capture_solve_ik(**kwargs):
        solve_calls.append(
            {
                "target_position": list(np.asarray(kwargs["target_position"], dtype=float)),
                "initial_joint_angles": list(np.asarray(kwargs["initial_joint_angles"], dtype=float)),
            }
        )
        return [0.0, 0.0, 0.0, 0.0, 0.0, 0.1 * len(solve_calls)]

    monkeypatch.setattr(command_api.ik_solver, "solve_ik", _capture_solve_ik)

    def _capture_set_servo_positions(logical_joint_angles_rad, speed_value, acceleration_value_deg_s2):
        direct_write_calls.append(list(np.asarray(logical_joint_angles_rad, dtype=float)))

    monkeypatch.setattr(command_api.servo_driver, "set_servo_positions", _capture_set_servo_positions)

    command_api._jog_controller_thread()

    assert len(solve_calls) >= 2
    assert np.allclose(solve_calls[0]["target_position"], [0.0, 0.0, 0.0])
    assert np.allclose(solve_calls[1]["target_position"], [0.0, 0.0, 0.0])


def test_realtime_jog_loop_pure_translation_keeps_commanded_orientation_fixed(monkeypatch):
    fresh_manager = command_api.JogSessionManager()
    fresh_manager.start_session(
        owner_id="test-owner",
        seq=0,
        lease_timeout_s=command_api.JOG_CONTROLLER_LEASE_TIMEOUT_S,
        deadman=True,
        velocity_vector=(0.01, 0.0, 0.0, 0.0, 0.0, 0.0),
        backend_mode="controller_cartesian_loop",
        session_id="test-session",
    )
    orientation_targets: list[np.ndarray] = []
    final_measured_state = [0.0, 0.0, 0.0, 0.0, 0.0, 0.4]
    measured_states = iter(
        [
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            final_measured_state,
            final_measured_state,
        ]
    )
    direct_write_count = 0
    sleep_calls = 0

    monkeypatch.setattr(command_api, "_JOG_SESSION_MANAGER", fresh_manager)
    monkeypatch.setattr(command_api, "_get_rtcore_jog_backend", lambda: None)
    monkeypatch.setattr(command_api.utils, "NUM_LOGICAL_JOINTS", 6, raising=False)
    monkeypatch.setattr(command_api.utils, "LOGICAL_JOINT_LIMITS_RAD", [(-3.14, 3.14)] * 6, raising=False)
    monkeypatch.setattr(command_api.utils, "gripper_present", False, raising=False)

    def _sleep_and_stop(_seconds):
        nonlocal sleep_calls
        sleep_calls += 1
        if sleep_calls >= 2:
            fresh_manager.stop_session(
                session_id="test-session",
                owner_id="test-owner",
                reason="test-stop",
                allow_missing=True,
            )

    monkeypatch.setattr(command_api.time, "sleep", _sleep_and_stop)
    monkeypatch.setattr(
        command_api.servo_driver,
        "get_current_arm_state_rad",
        lambda verbose=False: next(measured_states, final_measured_state),
    )

    def _fake_fk_matrix(joints):
        pose = np.eye(4, dtype=float)
        yaw = float(np.asarray(joints, dtype=float)[5])
        pose[:3, :3] = command_api.R.from_euler("z", yaw).as_matrix()
        return pose

    monkeypatch.setattr(command_api.ik_solver, "get_fk_matrix", _fake_fk_matrix)

    def _capture_solve_ik(**kwargs):
        orientation_targets.append(np.asarray(kwargs["target_orientation_matrix"], dtype=float))
        return [0.05 * len(orientation_targets), 0.0, 0.0, 0.0, 0.0, 0.0]

    monkeypatch.setattr(command_api.ik_solver, "solve_ik", _capture_solve_ik)

    def _capture_set_servo_positions(_logical_joint_angles_rad, _speed_value, _acceleration_value_deg_s2):
        nonlocal direct_write_count
        direct_write_count += 1

    monkeypatch.setattr(command_api.servo_driver, "set_servo_positions", _capture_set_servo_positions)

    command_api._jog_controller_thread()

    assert len(orientation_targets) >= 2
    assert np.allclose(orientation_targets[0], np.eye(3, dtype=float))
    assert np.allclose(orientation_targets[1], np.eye(3, dtype=float))


def test_realtime_jog_loop_seeds_ik_from_previous_commanded_joints(monkeypatch):
    fresh_manager = command_api.JogSessionManager()
    fresh_manager.start_session(
        owner_id="test-owner",
        seq=0,
        lease_timeout_s=command_api.JOG_CONTROLLER_LEASE_TIMEOUT_S,
        deadman=True,
        velocity_vector=(0.01, 0.0, 0.0, 0.0, 0.0, 0.0),
        backend_mode="controller_cartesian_loop",
        session_id="test-session",
    )
    seed_history: list[list[float]] = []
    final_measured_state = [0.6, 0.0, 0.0, 0.0, 0.0, 0.0]
    measured_states = iter(
        [
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            final_measured_state,
            final_measured_state,
        ]
    )
    direct_write_count = 0
    sleep_calls = 0

    monkeypatch.setattr(command_api, "_JOG_SESSION_MANAGER", fresh_manager)
    monkeypatch.setattr(command_api, "_get_rtcore_jog_backend", lambda: None)
    monkeypatch.setattr(command_api.utils, "NUM_LOGICAL_JOINTS", 6, raising=False)
    monkeypatch.setattr(command_api.utils, "LOGICAL_JOINT_LIMITS_RAD", [(-3.14, 3.14)] * 6, raising=False)
    monkeypatch.setattr(command_api.utils, "gripper_present", False, raising=False)

    def _sleep_and_stop(_seconds):
        nonlocal sleep_calls
        sleep_calls += 1
        if sleep_calls >= 2:
            fresh_manager.stop_session(
                session_id="test-session",
                owner_id="test-owner",
                reason="test-stop",
                allow_missing=True,
            )

    monkeypatch.setattr(command_api.time, "sleep", _sleep_and_stop)
    monkeypatch.setattr(
        command_api.servo_driver,
        "get_current_arm_state_rad",
        lambda verbose=False: next(measured_states, final_measured_state),
    )
    monkeypatch.setattr(command_api.ik_solver, "get_fk_matrix", lambda _q: np.eye(4, dtype=float))

    def _capture_solve_ik(**kwargs):
        seed_history.append(list(np.asarray(kwargs["initial_joint_angles"], dtype=float)))
        return [0.1 * len(seed_history), 0.0, 0.0, 0.0, 0.0, 0.0]

    monkeypatch.setattr(command_api.ik_solver, "solve_ik", _capture_solve_ik)

    def _capture_set_servo_positions(_logical_joint_angles_rad, _speed_value, _acceleration_value_deg_s2):
        nonlocal direct_write_count
        direct_write_count += 1

    monkeypatch.setattr(command_api.servo_driver, "set_servo_positions", _capture_set_servo_positions)

    command_api._jog_controller_thread()

    assert len(seed_history) >= 2
    assert np.allclose(seed_history[0], [0.0] * 6)
    assert np.allclose(seed_history[1], [0.1, 0.0, 0.0, 0.0, 0.0, 0.0])
    assert fresh_manager.get_snapshot()["last_resync_reason"] == "jog-start"


def test_realtime_jog_loop_emits_joint_limit_alert(monkeypatch):
    class _FakeRTCoreJogBackend:
        def __init__(self, manager):
            self.calls: list[tuple[str, list[float] | float]] = []
            self._manager = manager

        def start_joint_velocity_lease_jog(self, *, timeout_s):
            self.calls.append(("start", float(timeout_s)))

        def update_joint_velocity_lease_jog(self, joint_velocities_rad_s, *, timeout_s):
            self.calls.append(("send", list(joint_velocities_rad_s)))
            self.calls.append(("timeout", float(timeout_s)))
            self._manager.stop_session(
                session_id="test-session",
                owner_id="test-owner",
                reason="test-stop",
                allow_missing=True,
            )

        def stop_joint_velocity_lease_jog(self, *, quick_stop=False):
            self.calls.append(("stop", 0.0))

    fresh_manager = command_api.JogSessionManager()
    fresh_manager.start_session(
        owner_id="test-owner",
        seq=0,
        lease_timeout_s=command_api.JOG_CONTROLLER_LEASE_TIMEOUT_S,
        deadman=True,
        velocity_vector=(0.0, 0.0, 0.0, 0.0, 0.0, 1.0),
        backend_mode="joint_velocity_lease",
        backend_timeout_s=command_api.JOG_VELOCITY_TIMEOUT_S,
        session_id="test-session",
    )
    backend = _FakeRTCoreJogBackend(fresh_manager)
    pushed_alerts: list[dict[str, object]] = []

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
            "jog_velocities": np.array([0.0, 0.0, 0.0, 0.0, 0.0, 1.0], dtype=float),
            "jog_gripper_velocity_deg_s": 0.0,
            "last_jog_command_time": command_api.time.monotonic(),
            "jog_thread": None,
        }
    )
    monkeypatch.setattr(command_api, "_JOG_SESSION_MANAGER", fresh_manager)
    monkeypatch.setattr(command_api, "_get_rtcore_jog_backend", lambda: backend)
    monkeypatch.setattr(command_api.utils, "gripper_present", False, raising=False)
    sleep_calls = 0

    def _sleep_and_stop(_seconds):
        nonlocal sleep_calls
        sleep_calls += 1
        if sleep_calls >= 1:
            fresh_manager.stop_session(
                session_id="test-session",
                owner_id="test-owner",
                reason="test-stop",
                allow_missing=True,
            )

    monkeypatch.setattr(command_api.time, "sleep", _sleep_and_stop)
    monkeypatch.setattr(
        command_api.servo_driver,
        "get_current_arm_state_rad",
        lambda verbose=False: [0.0, 0.0, 0.0, 0.0, 0.0, 0.9],
    )
    monkeypatch.setattr(
        command_api.ik_solver,
        "get_fk_matrix",
        lambda q_current: np.eye(4, dtype=float),
    )
    monkeypatch.setattr(
        command_api.ik_solver,
        "solve_ik",
        lambda **_kwargs: [0.0, 0.0, 0.0, 0.0, 0.0, 1.01],
    )
    monkeypatch.setattr(
        command_api.telemetry_alerts,
        "push_alert",
        lambda level, kind, message, servo_ids=None, details=None, key=None: pushed_alerts.append(
            {
                "level": level,
                "kind": kind,
                "message": message,
                "servo_ids": servo_ids,
                "details": details,
                "key": key,
            }
        ),
    )
    monkeypatch.setattr(
        command_api.servo_driver,
        "set_servo_positions",
        lambda *args, **kwargs: None,
    )

    command_api._jog_controller_thread()

    joint_limit_alerts = [alert for alert in pushed_alerts if alert["kind"] == "JOINT_LIMIT"]
    gate_alerts = [alert for alert in pushed_alerts if alert["kind"] == "JOG_GATE_REJECTED"]
    assert len(joint_limit_alerts) == 1
    alert = joint_limit_alerts[0]
    assert alert["level"] == "warning"
    assert "J6 upper" in str(alert["message"])
    details = alert["details"]
    assert isinstance(details, dict)
    assert details["logical_joints"] == [6]
    assert details["joint_labels"] == ["J6 upper"]
    assert len(gate_alerts) == 1
    assert gate_alerts[0]["details"]["reason_code"] == "LIMIT_VIOLATION"
    assert fresh_manager.get_snapshot()["last_gate_failure_reason"] == "LIMIT_VIOLATION"


def test_realtime_jog_loop_gate_rejection_does_not_send_direct_joint_command(monkeypatch):
    fresh_manager = command_api.JogSessionManager()
    fresh_manager.start_session(
        owner_id="test-owner",
        seq=0,
        lease_timeout_s=command_api.JOG_CONTROLLER_LEASE_TIMEOUT_S,
        deadman=True,
        velocity_vector=(0.02, 0.0, 0.0, 0.0, 0.0, 0.0),
        backend_mode="controller_cartesian_loop",
        session_id="test-session",
    )
    direct_write_calls: list[object] = []
    sleep_calls = 0

    monkeypatch.setattr(command_api, "_JOG_SESSION_MANAGER", fresh_manager)
    monkeypatch.setattr(command_api, "_get_rtcore_jog_backend", lambda: None)
    monkeypatch.setattr(command_api.utils, "NUM_LOGICAL_JOINTS", 6, raising=False)
    monkeypatch.setattr(command_api.utils, "LOGICAL_JOINT_LIMITS_RAD", [(-3.14, 3.14)] * 6, raising=False)
    monkeypatch.setattr(command_api.utils, "gripper_present", False, raising=False)

    def _sleep_and_stop(_seconds):
        nonlocal sleep_calls
        sleep_calls += 1
        if sleep_calls >= 1:
            fresh_manager.stop_session(
                session_id="test-session",
                owner_id="test-owner",
                reason="test-stop",
                allow_missing=True,
            )

    monkeypatch.setattr(command_api.time, "sleep", _sleep_and_stop)
    monkeypatch.setattr(
        command_api.servo_driver,
        "get_current_arm_state_rad",
        lambda verbose=False: [0.0] * 6,
    )
    monkeypatch.setattr(command_api.ik_solver, "get_fk_matrix", lambda _q: np.eye(4, dtype=float))
    monkeypatch.setattr(command_api.ik_solver, "solve_ik", lambda **_kwargs: [1.5, 0.0, 0.0, 0.0, 0.0, 0.0])
    monkeypatch.setattr(
        command_api.servo_driver,
        "set_servo_positions",
        lambda *args, **kwargs: direct_write_calls.append((args, kwargs)),
    )

    command_api._jog_controller_thread()

    motion_writes = [call for call in direct_write_calls if len(call[0]) >= 3 and call[0][1] == 800]
    assert motion_writes == []
    assert fresh_manager.get_snapshot()["last_gate_failure_reason"] == "IK_JUMP_REJECTED"


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


def test_handle_safe_power_up_rejects_when_runtime_is_not_safe(monkeypatch):
    backend_calls: list[str] = []

    class _Backend:
        def safe_power_up(self):
            backend_calls.append("safe_power_up")
            return True

    motion_payload = {
        "accepted": True,
        "state": "accepted",
        "completion_scope": "rtcore_execution",
        "trajectory_id": 11,
        "source_of_truth": "rtcore",
        "safe_for_power_transition": False,
        "power_transition_blockers": ["active_trajectory", "queued_motion"],
        "power_transition_blocker_details": [
            {"code": "active_trajectory", "message": "An RTCore trajectory is still latched or active.", "active_traj_id": 11},
            {"code": "queued_motion", "message": "Queued RTCore motion points are still pending.", "queue_depth": 3},
        ],
        "execution": {
            "controller_thread_running": False,
            "rtcore_status_present": True,
            "safe_for_power_transition": False,
            "power_transition_blockers": ["active_trajectory", "queued_motion"],
            "power_transition_blocker_details": [
                {"code": "active_trajectory", "message": "An RTCore trajectory is still latched or active.", "active_traj_id": 11},
                {"code": "queued_motion", "message": "Queued RTCore motion points are still pending.", "queue_depth": 3},
            ],
        },
    }

    monkeypatch.setattr(command_api.backend_registry, "get_active_backend", lambda: _Backend())
    monkeypatch.setattr(command_api, "get_motion_execution_status", lambda: dict(motion_payload))

    result = command_api.handle_safe_power_up()

    assert result["accepted"] is False
    assert result["code"] == "POWER_UP_BLOCKED"
    assert result["power_transition_blockers"] == ["active_trajectory", "queued_motion"]
    assert backend_calls == []


def test_handle_safe_power_down_waits_for_idle_and_calls_backend(monkeypatch):
    backend_calls: list[tuple[bool, float | None]] = []
    stop_calls: list[str] = []
    wait_calls: list[float] = []

    class _Backend:
        def safe_power_down(self, *, wait_for_idle: bool = False, timeout_s: float | None = None):
            backend_calls.append((bool(wait_for_idle), timeout_s))
            return True

    completed_motion = {
        "accepted": True,
        "state": "completed",
        "completion_scope": "rtcore_execution",
        "trajectory_id": 0,
        "source_of_truth": "rtcore",
        "safe_for_power_transition": True,
        "power_transition_blockers": [],
        "power_transition_blocker_details": [],
        "execution": {
            "controller_thread_running": False,
            "rtcore_status_present": True,
            "safe_for_power_transition": True,
            "power_transition_blockers": [],
            "power_transition_blocker_details": [],
        },
    }

    monkeypatch.setattr(command_api, "handle_stop_command", lambda: stop_calls.append("stop"))
    monkeypatch.setattr(command_api, "handle_wait_for_idle", lambda timeout_s=30.0: wait_calls.append(float(timeout_s)) or dict(completed_motion))
    monkeypatch.setattr(command_api.backend_registry, "get_active_backend", lambda: _Backend())
    monkeypatch.setattr(command_api, "get_motion_execution_status", lambda: dict(completed_motion))

    result = command_api.handle_safe_power_down(wait_for_idle=True)

    assert result["accepted"] is True
    assert result["code"] == "POWER_DOWN_SENT"
    assert result["waited_for_idle"] is True
    assert stop_calls == ["stop"]
    assert wait_calls == [30.0]
    assert backend_calls == [(True, 1.0)]


def test_handle_reset_faults_leaves_system_disarmed(monkeypatch):
    reset_calls: list[int | None] = []

    class _Backend:
        def reset_faults(self, *, logical_joint_index: int | None = None):
            reset_calls.append(logical_joint_index)
            return True

    completed_motion = {
        "accepted": True,
        "state": "completed",
        "completion_scope": "rtcore_execution",
        "trajectory_id": 0,
        "source_of_truth": "rtcore",
        "safe_for_power_transition": True,
        "power_transition_blockers": [],
        "power_transition_blocker_details": [],
        "execution": {
            "controller_thread_running": False,
            "rtcore_status_present": True,
            "safe_for_power_transition": True,
            "power_transition_blockers": [],
            "power_transition_blocker_details": [],
        },
    }

    monkeypatch.setattr(command_api, "handle_stop_command", lambda: None)
    monkeypatch.setattr(command_api, "handle_wait_for_idle", lambda timeout_s=30.0: dict(completed_motion))
    monkeypatch.setattr(command_api.backend_registry, "get_active_backend", lambda: _Backend())
    monkeypatch.setattr(command_api, "get_motion_execution_status", lambda: dict(completed_motion))

    result = command_api.handle_reset_faults(logical_joint_index=1)

    assert result["accepted"] is True
    assert result["code"] == "RESET_FAULTS_SENT"
    assert result["disarmed_after_reset"] is True
    assert result["joint"] == 2
    assert reset_calls == [1]


def test_handle_stop_command_skips_legacy_brake_write_for_rtcore_backend(monkeypatch):
    servo_writes: list[list[float]] = []
    aborted: list[str] = []
    jog_stops: list[str] = []

    class _Backend:
        def abort_trajectory(self):
            aborted.append("abort")

        def get_execution_status(self):
            return object()

    monkeypatch.setattr(command_api.utils, "trajectory_state_update", lambda **kwargs: None)
    monkeypatch.setattr(command_api.utils, "trajectory_state_set", lambda *_args, **_kwargs: None)
    monkeypatch.setattr(command_api.utils, "trajectory_state_snapshot", lambda: {})
    monkeypatch.setattr(command_api.utils, "set_motion_state", lambda _state: None)
    monkeypatch.setattr(command_api, "_program_status_from_snapshot", lambda _snapshot: None)
    monkeypatch.setattr(command_api, "stop_active_jog_session", lambda reason="controller-stop": jog_stops.append(str(reason)) or {})
    monkeypatch.setattr(command_api.backend_registry, "get_active_backend", lambda: _Backend())
    monkeypatch.setattr(command_api.servo_driver, "get_current_arm_state_rad", lambda verbose=False: [0.1] * 6)
    monkeypatch.setattr(
        command_api.servo_driver,
        "set_servo_positions",
        lambda logical_joint_angles_rad, speed_value, acceleration_value_deg_s2: servo_writes.append(list(logical_joint_angles_rad)),
    )

    command_api.handle_stop_command()

    assert jog_stops == ["controller-stop"]
    assert aborted == ["abort"]
    assert servo_writes == []
