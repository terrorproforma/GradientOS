import time

import numpy as np

from gradient_os.arm_controller import command_api
from gradient_os.arm_controller import servo_driver


class FakeSingleActuatorBackend:
    def __init__(self):
        self.is_initialized = True
        self.calls: list[tuple[int, float, int, int]] = []

    def set_single_actuator_position(self, actuator_id: int, position_rad: float, speed: int, accel: int) -> None:
        self.calls.append((actuator_id, float(position_rad), int(speed), int(accel)))


def test_set_single_servo_position_rads_backend_uses_single_actuator_api(monkeypatch):
    backend = FakeSingleActuatorBackend()
    monkeypatch.setattr(servo_driver, "_get_backend", lambda: backend)
    monkeypatch.setattr(servo_driver, "_use_backend", lambda: True)
    monkeypatch.setattr(servo_driver.utils, "ENCODER_RESOLUTION", None, raising=False)

    servo_driver.set_single_servo_position_rads(servo_id=3, position_rad=0.25, speed=800, accel=0)

    assert backend.calls == [(3, 0.25, 800, 0)]


def test_realtime_jog_loop_commands_joint_space_via_servo_driver(monkeypatch):
    commanded: list[tuple[list[float], int, float]] = []
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
    sleep_calls = 0

    monkeypatch.setattr(command_api.utils, "ENCODER_RESOLUTION", None, raising=False)
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
            "last_jog_command_time": time.monotonic(),
        }
    )
    monkeypatch.setattr(command_api, "_JOG_SESSION_MANAGER", fresh_manager)
    monkeypatch.setattr(command_api, "_get_rtcore_jog_backend", lambda: None)
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

    def _capture_set_servo_positions(logical_joint_angles_rad, speed_value, acceleration_value_deg_s2):
        commanded.append((list(logical_joint_angles_rad), int(speed_value), float(acceleration_value_deg_s2)))

    monkeypatch.setattr(command_api.servo_driver, "set_servo_positions", _capture_set_servo_positions)

    command_api._jog_controller_thread()

    motion_writes = [entry for entry in commanded if entry[1] == 800]
    assert motion_writes == [([0.05] * 6, 800, 0.0)]
