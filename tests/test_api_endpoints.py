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
        "GET_ALL_POSITIONS": (
            True,
            "ALL_POS_DATA,10,2048,20,2050,21,2050",
        ),
        "GET_ORIENTATION": (
            True,
            "CURRENT_ORIENTATION,1,0,0,0,1,0,0,0,1",
        ),
        "GET_TRAJECTORIES": (True, "TRAJECTORIES,alpha,beta"),
    }
    no_reply_commands = {
        "PLAN_TRAJECTORY",
        "REC_POS",
        "END_TRAJECTORY,test",
        "RUN_TRAJECTORY,alpha,false",
    }
    call_log = []

    def fake_send(command, timeout=0.5, expect_response=True):
        call_log.append((command, timeout, expect_response))
        if command in no_reply_commands:
            if expect_response:
                return False, f"unexpected response requested for {command}"
            return True, ""
        return responses.get(command, (False, f"unexpected {command}"))

    monkeypatch.setattr("gradient_os.api.main._send_controller_command", fake_send)
    monkeypatch.setattr(
        "gradient_os.api.main._probe_controller", lambda timeout=0.5: (True, "ok")
    )
    yield call_log


@pytest.fixture
def client(monkeypatch):
    with patch_send(monkeypatch) as call_log:
        app = create_app()
        with TestClient(app) as client:
            client.command_calls = call_log  # type: ignore[attr-defined]
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


def test_info_all_positions(client):
    resp = client.get("/info/all-positions")
    assert resp.status_code == 200
    assert resp.json() == {
        "servos": [
            {"servo_id": 10, "raw_position": 2048},
            {"servo_id": 20, "raw_position": 2050},
            {"servo_id": 21, "raw_position": 2050},
        ]
    }


def test_info_orientation(client):
    resp = client.get("/info/orientation")
    assert resp.status_code == 200
    assert resp.json() == {
        "matrix": [
            [1.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
            [0.0, 0.0, 1.0],
        ]
    }


def test_trajectory_plan_record_end(client):
    resp = client.post("/trajectory/plan")
    assert resp.status_code == 200
    assert client.command_calls[-1] == ("PLAN_TRAJECTORY", 1.0, False)

    resp = client.post("/trajectory/record")
    assert resp.status_code == 200
    assert client.command_calls[-1] == ("REC_POS", 1.0, False)

    resp = client.post("/trajectory/end", json={"name": "test"})
    assert resp.status_code == 200
    assert client.command_calls[-1] == ("END_TRAJECTORY,test", 2.0, False)


def test_trajectory_list(client):
    resp = client.get("/trajectory/list")
    assert resp.status_code == 200
    assert resp.json() == {"trajectories": ["alpha", "beta"]}


def test_trajectory_run(client):
    resp = client.post("/trajectory/run", json={"name": "alpha"})
    assert resp.status_code == 200
    assert client.command_calls[-1] == ("RUN_TRAJECTORY,alpha,false", 2.0, False)
