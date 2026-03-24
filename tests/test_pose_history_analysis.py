import unittest

from gradient_os.diagnostics.pose_history_analysis import analyze_pose_history_document


def _pose(x_mm: float = 0.0, y_mm: float = 0.0, z_mm: float = 0.0) -> dict[str, object]:
    return {
        "position_m": {
            "x": x_mm / 1000.0,
            "y": y_mm / 1000.0,
            "z": z_mm / 1000.0,
        },
        "orientation_euler_deg": {
            "roll": 0.0,
            "pitch": 0.0,
            "yaw": 0.0,
        },
        "joints_deg": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    }


def _sample(
    collected_at: str,
    *,
    pose_x_mm: float,
    is_jogging: bool,
    session_state: str,
    command: str,
    linear_velocity_m_s: list[float] | None = None,
    angular_velocity_deg_s: list[float] | None = None,
    seq: int | None = None,
    commanded_x_mm: float | None = None,
    follow_position_mm: float = 0.0,
) -> dict[str, object]:
    ik_debug = {
        "linear_velocity_m_s": list(linear_velocity_m_s or [0.0, 0.0, 0.0]),
        "angular_velocity_deg_s": list(angular_velocity_deg_s or [0.0, 0.0, 0.0]),
        "seq": seq,
        "dt_s": 0.02,
        "measured_pose": _pose(pose_x_mm),
        "commanded_pose": _pose(commanded_x_mm if commanded_x_mm is not None else pose_x_mm),
        "target_vs_applied": {
            "position_error_mm": 0.0,
            "orientation_error_deg": 0.0,
        },
        "following_error": {
            "pose": {
                "position_error_mm": follow_position_mm,
                "orientation_error_deg": 0.0,
            },
            "joint": {
                "max_abs_joint_error_deg": 0.0,
            },
        },
        "gate_result": "accepted",
        "gate_reason": "OK",
        "solve_failed": False,
        "clamped": False,
        "last_resync_reason": "jog-start",
    }
    return {
        "collected_at": collected_at,
        "motion_state": "jogging" if is_jogging else "idle",
        "is_jogging": is_jogging,
        "session_state": session_state,
        "controller_last_command": command,
        "pose": _pose(pose_x_mm),
        "ik_debug": ik_debug,
    }


class PoseHistoryAnalysisTest(unittest.TestCase):
    def test_analyze_pose_history_splits_segments_and_pairs_round_trip(self):
        doc = {
            "source": "auto-stop",
            "pose_history": [
                _sample(
                    "2026-03-24T06:00:00+00:00",
                    pose_x_mm=0.0,
                    is_jogging=False,
                    session_state="idle",
                    command="GET_POSITION",
                ),
                _sample(
                    "2026-03-24T06:00:01+00:00",
                    pose_x_mm=10.0,
                    commanded_x_mm=10.0,
                    is_jogging=True,
                    session_state="active",
                    command="JOG_SESSION_START",
                    linear_velocity_m_s=[0.02, 0.0, 0.0],
                    seq=0,
                    follow_position_mm=0.8,
                ),
                _sample(
                    "2026-03-24T06:00:02+00:00",
                    pose_x_mm=20.0,
                    commanded_x_mm=20.0,
                    is_jogging=True,
                    session_state="active",
                    command="JOG_SESSION_UPDATE",
                    linear_velocity_m_s=[0.02, 0.0, 0.0],
                    seq=1,
                    follow_position_mm=1.2,
                ),
                _sample(
                    "2026-03-24T06:00:03+00:00",
                    pose_x_mm=20.0,
                    commanded_x_mm=20.0,
                    is_jogging=False,
                    session_state="stopped",
                    command="JOG_SESSION_STOP",
                    linear_velocity_m_s=[0.02, 0.0, 0.0],
                    seq=1,
                ),
                _sample(
                    "2026-03-24T06:00:04+00:00",
                    pose_x_mm=19.0,
                    commanded_x_mm=20.0,
                    is_jogging=False,
                    session_state="stopped",
                    command="GET_POSITION",
                    linear_velocity_m_s=[0.02, 0.0, 0.0],
                    seq=1,
                ),
                _sample(
                    "2026-03-24T06:00:05+00:00",
                    pose_x_mm=10.0,
                    commanded_x_mm=10.0,
                    is_jogging=True,
                    session_state="active",
                    command="JOG_SESSION_START",
                    linear_velocity_m_s=[-0.02, 0.0, 0.0],
                    seq=0,
                    follow_position_mm=0.6,
                ),
                _sample(
                    "2026-03-24T06:00:06+00:00",
                    pose_x_mm=1.0,
                    commanded_x_mm=0.0,
                    is_jogging=True,
                    session_state="active",
                    command="JOG_SESSION_UPDATE",
                    linear_velocity_m_s=[-0.02, 0.0, 0.0],
                    seq=1,
                    follow_position_mm=0.9,
                ),
                _sample(
                    "2026-03-24T06:00:07+00:00",
                    pose_x_mm=0.5,
                    commanded_x_mm=0.0,
                    is_jogging=False,
                    session_state="stopped",
                    command="JOG_SESSION_STOP",
                    linear_velocity_m_s=[-0.02, 0.0, 0.0],
                    seq=1,
                ),
                _sample(
                    "2026-03-24T06:00:08+00:00",
                    pose_x_mm=0.5,
                    commanded_x_mm=0.0,
                    is_jogging=False,
                    session_state="stopped",
                    command="GET_POSITION",
                    linear_velocity_m_s=[-0.02, 0.0, 0.0],
                    seq=1,
                ),
            ],
        }

        report = analyze_pose_history_document(doc)

        self.assertEqual(report["segment_count"], 2)
        self.assertEqual(report["round_trip_count"], 1)

        first_segment = report["segments"][0]
        self.assertEqual(first_segment["signature"]["classification"]["label"], "+x")
        self.assertEqual(first_segment["active_end_measured_delta"]["position_mm"]["x"], 20.0)
        self.assertEqual(first_segment["active_end_commanded_delta"]["position_mm"]["x"], 20.0)
        self.assertEqual(first_segment["settled_end_delta"]["position_mm"]["x"], 19.0)
        self.assertEqual(first_segment["max_following_error"]["position_error_mm"], 1.2)
        self.assertEqual(first_segment["settle_window_external_motion_commands"], [])

        second_segment = report["segments"][1]
        self.assertEqual(second_segment["signature"]["classification"]["label"], "-x")
        self.assertEqual(second_segment["active_end_measured_delta"]["position_mm"]["x"], -18.0)
        self.assertEqual(second_segment["active_end_commanded_delta"]["position_mm"]["x"], -19.0)
        self.assertEqual(second_segment["settled_end_delta"]["position_mm"]["x"], -18.5)

        round_trip = report["round_trips"][0]
        self.assertEqual(round_trip["axis"], "x")
        self.assertEqual(round_trip["final_residual_delta"]["position_mm"]["x"], 0.5)

    def test_analyze_pose_history_skips_round_trip_when_gap_has_external_motion(self):
        doc = {
            "source": "auto-stop",
            "pose_history": [
                _sample(
                    "2026-03-24T06:10:00+00:00",
                    pose_x_mm=0.0,
                    is_jogging=False,
                    session_state="idle",
                    command="GET_POSITION",
                ),
                _sample(
                    "2026-03-24T06:10:01+00:00",
                    pose_x_mm=8.0,
                    commanded_x_mm=8.0,
                    is_jogging=True,
                    session_state="active",
                    command="JOG_SESSION_START",
                    linear_velocity_m_s=[0.02, 0.0, 0.0],
                    seq=0,
                ),
                _sample(
                    "2026-03-24T06:10:02+00:00",
                    pose_x_mm=8.0,
                    commanded_x_mm=8.0,
                    is_jogging=False,
                    session_state="stopped",
                    command="MOVE_LINE_RELATIVE",
                    linear_velocity_m_s=[0.02, 0.0, 0.0],
                    seq=0,
                ),
                _sample(
                    "2026-03-24T06:10:03+00:00",
                    pose_x_mm=2.0,
                    commanded_x_mm=2.0,
                    is_jogging=True,
                    session_state="active",
                    command="JOG_SESSION_START",
                    linear_velocity_m_s=[-0.02, 0.0, 0.0],
                    seq=0,
                ),
                _sample(
                    "2026-03-24T06:10:04+00:00",
                    pose_x_mm=2.0,
                    commanded_x_mm=2.0,
                    is_jogging=False,
                    session_state="stopped",
                    command="GET_POSITION",
                    linear_velocity_m_s=[-0.02, 0.0, 0.0],
                    seq=0,
                ),
            ],
        }

        report = analyze_pose_history_document(doc)

        self.assertEqual(report["segment_count"], 2)
        self.assertEqual(report["round_trip_count"], 0)
        self.assertEqual(report["segments"][0]["settle_window_external_motion_commands"], ["MOVE_LINE_RELATIVE"])


if __name__ == "__main__":
    unittest.main()
