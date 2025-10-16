import pytest
pytest.importorskip("httpx")

from contextlib import contextmanager
from fastapi.testclient import TestClient

from gradient_os.api.main import create_app


@contextmanager
def patch_send(monkeypatch):
    responses = {
        "STOP": (True, "ACK,STOP"),
        "WAIT_FOR_IDLE": (True, "ACK,WAIT_FOR_IDLE"),
        "GET_STATUS": (True, "STATUS,gripper_present,True"),
        "GET_POSITION": (
            True,
            "CURRENT_POSE,0.1,0.2,0.3,10.0,20.0,30.0,1,2,3,4,5,6",
        ),
        "GET_JOINT_ANGLES": (True, "JOINT_ANGLES,1,2,3,4,5,6,7"),
        "GET_GRIPPER_STATE": (True, "GRIPPER_STATE,45.0,2048"),
    }

    def fake_send(command, timeout=0.5, expect_response=True):
        return responses.get(command, (False, f"unexpected {command}"))

    monkeypatch.setattr("gradient_os.api.main._send_controller_command", fake_send)
    monkeypatch.setattr(
        "gradient_os.api.main._probe_controller", lambda timeout=0.5: (True, "ok")
    )
    yield


@pytest.fixture
def client(monkeypatch):
    with patch_send(monkeypatch):
        app = create_app()
        with TestClient(app) as client:
            yield client


def test_control_stop(client):
    resp = client.post("/control/stop")
    assert resp.status_code == 200
    assert resp.json()["detail"] == "ACK,STOP"


def test_control_wait_for_idle(client):
    resp = client.post("/control/wait-for-idle")
    assert resp.status_code == 200
    assert resp.json()["detail"] == "ACK,WAIT_FOR_IDLE"


def test_info_status(client):
    resp = client.get("/info/status")
    assert resp.status_code == 200
    assert resp.json() == {"gripper_present": True}


def test_info_pose(client):
    resp = client.get("/info/pose")
    assert resp.status_code == 200
    body = resp.json()
    assert body["position_m"]["x"] == pytest.approx(0.1)
    assert body["orientation_euler_deg"]["yaw"] == pytest.approx(30.0)
    assert body["joints_deg"] == [1.0, 2.0, 3.0, 4.0, 5.0, 6.0]


def test_info_joints(client):
    resp = client.get("/info/joints")
    assert resp.status_code == 200
    assert resp.json() == {"arm_deg": [1, 2, 3, 4, 5, 6], "gripper_deg": 7}


def test_info_gripper(client):
    resp = client.get("/info/gripper")
    assert resp.status_code == 200
    assert resp.json() == {"angle_deg": 45.0, "raw_position": 2048}
