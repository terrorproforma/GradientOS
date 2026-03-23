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
