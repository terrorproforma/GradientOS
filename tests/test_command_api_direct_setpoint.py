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
        "completion_scope": "direct_setpoint_ack",
        "speed": 500,
        "acceleration": 500.0,
    }
    assert calls == [([0.1, 0.0, 0.0, 0.0, 0.0, 0.0], 500, 500.0)]
