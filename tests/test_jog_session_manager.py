import time

import pytest

from gradient_os.arm_controller.jog_session import JogSessionError, JogSessionManager


def _vector(x: float = 0.0) -> tuple[float, float, float, float, float, float]:
    return (x, 0.0, 0.0, 0.0, 0.0, 0.0)


def test_start_creates_session():
    manager = JogSessionManager()

    snapshot = manager.start_session(
        owner_id="ui-a",
        seq=0,
        lease_timeout_s=0.4,
        deadman=True,
        velocity_vector=_vector(0.05),
    )

    assert snapshot["session_active"] is True
    assert snapshot["owner_id"] == "ui-a"
    assert snapshot["state"] == "active"
    assert snapshot["last_seq_received"] == 0


def test_wrong_owner_rejected():
    manager = JogSessionManager()
    manager.start_session(
        owner_id="ui-a",
        seq=0,
        lease_timeout_s=0.4,
        deadman=True,
        velocity_vector=_vector(0.05),
    )

    with pytest.raises(JogSessionError, match="owner"):
        manager.update_session(
            session_id=manager.get_snapshot()["session_id"],
            owner_id="ui-b",
            seq=1,
            lease_timeout_s=None,
            deadman=True,
            velocity_vector=_vector(0.05),
        )


def test_same_owner_duplicate_start_is_not_counted_as_owner_conflict():
    manager = JogSessionManager()
    snapshot = manager.start_session(
        owner_id="ui-a",
        seq=0,
        lease_timeout_s=0.4,
        deadman=True,
        velocity_vector=_vector(0.05),
    )

    with pytest.raises(JogSessionError, match="already active"):
        manager.start_session(
            owner_id="ui-a",
            seq=1,
            lease_timeout_s=0.4,
            deadman=True,
            velocity_vector=_vector(0.05),
        )

    after = manager.get_snapshot()
    assert after["session_id"] == snapshot["session_id"]
    assert after["owner_conflict_rejects"] == 0


def test_stale_sequence_rejected():
    manager = JogSessionManager()
    snapshot = manager.start_session(
        owner_id="ui-a",
        seq=2,
        lease_timeout_s=0.4,
        deadman=True,
        velocity_vector=_vector(0.05),
    )

    with pytest.raises(JogSessionError, match="stale"):
        manager.update_session(
            session_id=snapshot["session_id"],
            owner_id="ui-a",
            seq=2,
            lease_timeout_s=None,
            deadman=True,
            velocity_vector=_vector(0.01),
        )


def test_lease_expiry_zeroes_motion():
    manager = JogSessionManager()
    manager.start_session(
        owner_id="ui-a",
        seq=0,
        lease_timeout_s=0.01,
        deadman=True,
        velocity_vector=_vector(0.05),
    )

    time.sleep(0.02)
    snapshot = manager.expire_if_needed()
    control = manager.get_control_state()

    assert snapshot["state"] == "expired"
    assert snapshot["session_active"] is False
    assert control["lease_valid"] is False
    assert control["velocity_vector"] == _vector(0.0)


def test_deadman_false_zeroes_motion():
    manager = JogSessionManager()
    snapshot = manager.start_session(
        owner_id="ui-a",
        seq=0,
        lease_timeout_s=0.4,
        deadman=False,
        velocity_vector=_vector(0.05),
    )
    control = manager.get_control_state()

    assert snapshot["deadman"] is False
    assert control["velocity_vector"] == _vector(0.0)


def test_stop_is_idempotent():
    manager = JogSessionManager()
    snapshot = manager.start_session(
        owner_id="ui-a",
        seq=0,
        lease_timeout_s=0.4,
        deadman=True,
        velocity_vector=_vector(0.05),
    )

    first = manager.stop_session(session_id=snapshot["session_id"], owner_id="ui-a", reason="client-stop")
    second = manager.stop_session(session_id=snapshot["session_id"], owner_id="ui-a", reason="client-stop")

    assert first["state"] == "stopped"
    assert second["state"] == "stopped"


def test_pause_for_motion_resumes_only_with_valid_lease():
    manager = JogSessionManager()
    snapshot = manager.start_session(
        owner_id="ui-a",
        seq=0,
        lease_timeout_s=0.01,
        deadman=True,
        velocity_vector=_vector(0.05),
    )

    paused = manager.pause_for_motion()
    assert paused["state"] == "paused_for_motion"

    time.sleep(0.02)
    resumed = manager.resume_after_motion()
    assert resumed["session_id"] == snapshot["session_id"]
    assert resumed["state"] == "expired"


def test_resync_command_state_records_controller_owned_pose_and_joints():
    manager = JogSessionManager()
    manager.start_session(
        owner_id="ui-a",
        seq=0,
        lease_timeout_s=0.4,
        deadman=True,
        velocity_vector=_vector(0.05),
    )

    snapshot = manager.resync_command_state(
        position_m=[0.1, 0.2, 0.3],
        orientation_matrix=[[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
        joint_vector=[0.01, 0.02, 0.03, 0.04, 0.05, 0.06],
        reason="jog-start",
    )
    control = manager.get_control_state()

    assert snapshot["command_state_valid"] is True
    assert snapshot["last_resync_reason"] == "jog-start"
    assert snapshot["commanded_position_m"] == [0.1, 0.2, 0.3]
    assert control["commanded_joint_vector"] == (0.01, 0.02, 0.03, 0.04, 0.05, 0.06)


def test_accept_command_step_and_gate_failure_are_visible_in_snapshot():
    manager = JogSessionManager()
    manager.start_session(
        owner_id="ui-a",
        seq=0,
        lease_timeout_s=0.4,
        deadman=True,
        velocity_vector=_vector(0.05),
    )
    manager.resync_command_state(
        position_m=[0.0, 0.0, 0.0],
        orientation_matrix=[[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
        joint_vector=[0.0] * 6,
        reason="jog-start",
    )

    manager.record_gate_failure(
        reason="IK_JUMP_REJECTED",
        details={"max_joint_step_rad": 1.2},
    )
    after_reject = manager.get_snapshot()
    manager.accept_command_step(
        position_m=[0.02, 0.0, 0.0],
        orientation_matrix=[[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
        joint_vector=[0.02, 0.0, 0.0, 0.0, 0.0, 0.0],
    )
    manager.update_following_error({"pose": {"position_error_mm": 0.5}, "joint": {"max_abs_joint_error_deg": 0.2}})
    final_snapshot = manager.get_snapshot()

    assert after_reject["last_gate_failure_reason"] == "IK_JUMP_REJECTED"
    assert after_reject["last_gate_failure_details"]["max_joint_step_rad"] == 1.2
    assert final_snapshot["last_gate_failure_reason"] is None
    assert final_snapshot["commanded_position_m"] == [0.02, 0.0, 0.0]
    assert final_snapshot["following_error_snapshot"]["pose"]["position_error_mm"] == 0.5
