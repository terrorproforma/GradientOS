import json
import base64
import tempfile
import os
from pathlib import Path
import pytest

pytest.importorskip("httpx")

from contextlib import contextmanager
from fastapi.testclient import TestClient

from gradient_os.api import main as api_main
from gradient_os.api.main import create_app


def _payload_token(payload: dict[str, object]) -> str:
    body = json.dumps(payload, separators=(",", ":"), ensure_ascii=True).encode("utf-8")
    return base64.urlsafe_b64encode(body).decode("ascii")


def _decode_command_payload(command: str) -> dict[str, object]:
    _, payload_b64 = command.split(",", 1)
    payload_json = base64.urlsafe_b64decode(payload_b64.encode("ascii")).decode("utf-8")
    return json.loads(payload_json)


@contextmanager
def patch_send(monkeypatch):
    accepted_motion_payload = {
        "accepted": True,
        "state": "accepted",
        "completion_scope": "rtcore_execution",
        "trajectory_id": 7,
        "source_of_truth": "rtcore",
        "execution": {
            "controller_motion_state": "executing",
            "controller_thread_running": False,
            "rtcore_status_present": True,
            "active_mode_name": "trajectory_execute",
            "state_name": "queued",
            "active_traj_id": 7,
            "queue_depth": 1,
            "queue_capacity": 4096,
            "motion_done": False,
            "stale_command": False,
            "underrun_count": 0,
        },
    }
    completed_motion_payload = {
        "accepted": True,
        "state": "completed",
        "completion_scope": "rtcore_execution",
        "trajectory_id": 7,
        "source_of_truth": "rtcore",
        "execution": {
            "controller_motion_state": "idle",
            "controller_thread_running": False,
            "rtcore_status_present": True,
            "active_mode_name": "trajectory_execute",
            "state_name": "completed",
            "active_traj_id": 7,
            "queue_depth": 0,
            "queue_capacity": 4096,
            "motion_done": True,
            "stale_command": False,
            "underrun_count": 0,
        },
    }
    active_program = {
        "name": "alpha",
        "active": True,
        "state": "executing",
        "terminal_reason": None,
        "failing_step_index": None,
        "completed_step_count": 1,
        "completed_loop_count": 0,
        "loop_enabled": False,
        "use_cache": False,
        "step_count": 3,
        "move_steps": 2,
        "pause_steps": 1,
        "joint_move_steps": 0,
        "rtcore_segments": True,
        "segment_execution_policy": "rtcore_queued",
        "current_step_index": 1,
        "current_step_type": "pause",
        "loop_iteration": 0,
    }
    accepted_program_motion_payload = {
        **accepted_motion_payload,
        "program": active_program,
        "program_name": "alpha",
        "program_active": True,
        "program_state": "executing",
        "program_terminal_reason": None,
        "program_failing_step_index": None,
        "program_completed_step_count": 1,
        "program_completed_loop_count": 0,
        "program_loop_enabled": False,
        "program_use_cache": False,
        "program_step_count": 3,
        "program_move_steps": 2,
        "program_pause_steps": 1,
        "program_joint_move_steps": 0,
        "program_rtcore_segments": True,
        "program_segment_execution_policy": "rtcore_queued",
        "program_current_step_index": 1,
        "program_current_step_type": "pause",
        "program_loop_iteration": 0,
    }
    accepted_run_program_payload = {
        **accepted_motion_payload,
        "completion_scope": "controller_program_thread",
        "source_of_truth": "controller",
        "program": {
            **active_program,
            "state": "accepted",
            "current_step_index": None,
            "current_step_type": None,
            "completed_step_count": 0,
        },
        "program_name": "alpha",
        "program_active": True,
        "program_state": "accepted",
        "program_terminal_reason": None,
        "program_failing_step_index": None,
        "program_completed_step_count": 0,
        "program_completed_loop_count": 0,
        "program_loop_enabled": False,
        "program_use_cache": False,
        "program_step_count": 3,
        "program_move_steps": 2,
        "program_pause_steps": 1,
        "program_joint_move_steps": 0,
        "program_rtcore_segments": True,
        "program_segment_execution_policy": "rtcore_queued",
        "program_current_step_index": None,
        "program_current_step_type": None,
        "program_loop_iteration": 0,
    }
    responses = {
        "STOP": (True, f"ACK,STOP,{_payload_token({**completed_motion_payload, 'state': 'aborted'})}"),
        "SAFE_POWER_UP": (True, "ACK,SAFE_POWER_UP,POWER_UP_SENT"),
        "SAFE_POWER_DOWN": (True, "ACK,SAFE_POWER_DOWN,POWER_DOWN_SENT"),
        "SAFE_POWER_DOWN,wait": (True, "ACK,SAFE_POWER_DOWN,POWER_DOWN_SENT"),
        "WAIT_FOR_IDLE": (True, f"ACK,WAIT_FOR_IDLE,{_payload_token(completed_motion_payload)}"),
        "WAIT_FOR_IDLE,12.5": (True, f"ACK,WAIT_FOR_IDLE,{_payload_token({**completed_motion_payload, 'wait_timeout_s': 12.5, 'waited_for_motion': True, 'wait_timed_out': False})}"),
        "RESET_FAULTS": (True, "ACK,RESET_FAULTS,ALL"),
        "RESET_FAULTS,1": (True, "ACK,RESET_FAULTS,JOINT,1"),
        "ZERO_JOINT,3": (True, "ACK,ZERO_JOINT,3"),
        "GET_MOTION_STATUS": (True, f"MOTION_STATUS,{_payload_token(accepted_program_motion_payload)}"),
        "GET_STATUS": (True, "STATUS,gripper_present,True"),
        "GET_POSITION": (
            True,
            "CURRENT_POSE,0.1,0.2,0.3,10.0,20.0,30.0,1,2,3,4,5,6",
        ),
        "GET_JOINT_ANGLES": (True, "JOINT_ANGLES,1,2,3,4,5,6,7"),
        "GET_JOINT_STATE": (
            True,
            "JOINT_STATE_JSON,"
            + json.dumps(
                {
                    "arm_rad": [0.1, 0.2, 0.3, 0.4, 0.5, 0.6],
                    "arm_deg": [1.0, 2.0, 3.0, 4.0, 5.0, 6.0],
                    "gripper_rad": 0.7,
                    "gripper_deg": 7.0,
                    "axis_counts": [101, 202, 303, 404, 505, 606],
                    "axis_torque_raw": [11, 12, 13, 14, 15, 16],
                    "axis_statusword": [4663, 4663, 4663, 4663, 4663, 4663],
                    "axis_error_code": [0, 0, 0, 0, 0, 0],
                    "axis_mode_display": [8, 8, 8, 8, 8, 8],
                    "axis_mode_display_name": [
                        "cyclic_sync_position",
                        "cyclic_sync_position",
                        "cyclic_sync_position",
                        "cyclic_sync_position",
                        "cyclic_sync_position",
                        "cyclic_sync_position",
                    ],
                    "axis_ds402_state_code": [5, 5, 5, 5, 5, 5],
                    "axis_di_bits": [1, 2, 4, 8, 16, 32],
                    "axis_fault_flags": [0, 0, 0, 0, 0, 0],
                    "axis_brake_state": [1, 1, 1, 1, 1, 1],
                    "axis_to_joint": [0, 1, 2, 3, 4, 5],
                    "backend_name": "ethercat_rtcore",
                    "read_source": "live_feedback",
                    "numeric_precision": "float64",
                },
                separators=(",", ":"),
            ),
        ),
        "GET_PERFORMANCE_STATE": (
            True,
            "PERFORMANCE_STATE_JSON,"
            + json.dumps(
                {
                    "udp": {
                        "last_command": "JOG_SESSION_UPDATE",
                        "last_dispatch_ms": 1.25,
                        "dispatch_ms": {"count": 3, "avg_ms": 0.9, "max_ms": 1.25, "last_ms": 1.25},
                        "interarrival_ms": {"count": 3, "avg_ms": 52.0, "max_ms": 60.0, "last_ms": 50.0},
                    },
                    "jog": {
                        "control_frequency_hz": 50,
                        "loop": {"count": 4, "avg_ms": 3.1, "max_ms": 4.0, "last_ms": 3.2, "overrun_count": 0},
                        "ik_debug": {
                            "captured_at": "2026-03-24T04:00:00+00:00",
                            "seq": 41,
                            "dt_s": 0.02,
                            "target_vs_solved": {"position_error_mm": 0.42, "orientation_error_deg": 0.03},
                            "target_vs_applied": {"position_error_mm": 1.95, "orientation_error_deg": 0.03},
                            "clamped_joint_indices": [5],
                            "clamped": True,
                            "solve_failed": False,
                        },
                    },
                    "motion_state": "executing",
                    "is_running": False,
                    "is_jogging": True,
                    "last_command_age_s": 0.02,
                    "command_link_stale": False,
                    "recent_udp_reset_count": 0,
                },
                separators=(",", ":"),
            ),
        ),
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
        "RUN_TRAJECTORY,alpha,false": (
            True,
            f"ACK,RUN_TRAJECTORY,{_payload_token(accepted_run_program_payload)}",
        ),
    }
    no_reply_commands = {
        "PLAN_TRAJECTORY",
        "REC_POS",
        "END_TRAJECTORY,test",
        "0,0,0,0,0,0",
    }
    planner_payload = {
        "name": "__planner_preview__",
        "steps": [
            {"type": "move", "path": [[1.0, 2.0, 3.0]], "freq": 100},
        ],
        "trajectory": {
            "description": "Planned",
            "loop": False,
            "orientation_euler_angles_deg": None,
            "moves": [
                {"command": "move_absolute", "vector": [0.1, 0.2, 0.3]},
                {"command": "pause", "duration": 1.0},
            ],
        },
        "cartesian_path": [[0.1, 0.2, 0.3]],
        "waypoints": [[0.1, 0.2, 0.3]],
        "file_path": "/tmp/recorded_trajectories/__planner_preview__.json",
    }
    call_log = []
    kinematics_state = {
        "revision": 0,
        "profile": {
            "profile_id": "mini-6dof-arm:legacy-default",
            "version": "legacy-bridge-v1",
            "schema_version": 1,
            "checksum": "dummy",
            "robot_id": "mini-6dof-arm",
            "robot_serial": "unknown",
            "base_nominal": [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]],
            "base_calib": [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]],
            "tool_nominal": [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]],
            "tool_calib": [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]],
            "tool_runtime": [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]],
            "backend_compatibility": ["ikfast", "numeric"],
            "metadata": {},
        },
        "offsets": {
            "base": {"position_m": {"x": 0.0, "y": 0.0, "z": 0.0}, "rotation_deg": {"x": 0.0, "y": 0.0, "z": 0.0}},
            "tool": {"position_m": {"x": 0.0, "y": 0.0, "z": 0.0}, "rotation_deg": {"x": 0.0, "y": 0.0, "z": 0.0}},
        },
    }
    runtime_state = {
        "robot": {
            "name": "gradient05",
            "robot_id": "gradient-05",
            "display_name": "Gradient-05",
            "version": "0.1.0-template",
        },
        "mode": {"sim": False},
        "ik_solver": {
            "effective_backend": "numeric",
            "source": "robot_policy",
            "robot_default_backend": "numeric",
            "override_backend": None,
        },
        "servo_backend": {
            "effective_backend": "ethercat_rtcore",
            "source": "robot_policy",
            "robot_default_backend": "ethercat_rtcore",
            "override_backend": None,
        },
        "drive_profile": {
            "effective_profile": "a6ec_ds402",
            "source": "backend_default",
            "backend_default_profile": "a6ec_ds402",
            "override_profile": None,
        },
        "rtcore": {
            "configured_max_rpm": 6000.0,
            "configured_source": "runtime_config",
            "effective_max_rpm": 6000.0,
            "source": "runtime_config",
            "default_max_rpm": 6000.0,
            "override_max_rpm": 6000.0,
            "clamp_disabled": False,
        },
        "tool": {
            "active_tool_id": "identity",
            "display_name": "Identity (No Tool Offset)",
            "tool_type": "utility",
            "source": "library_default",
            "offset": {
                "position_mm": {"x": 0.0, "y": 0.0, "z": 0.0},
                "rotation_deg": {"x": 0.0, "y": 0.0, "z": 0.0},
            },
            "mesh": None,
            "weld": {},
        },
        "allow_unsafe_overrides": False,
    }
    session_state = {
        "session_present": False,
        "session_active": False,
        "session_id": None,
        "owner_id": None,
        "state": "idle",
        "deadman": False,
        "paused_for_motion": False,
        "lease_timeout_s": 0.4,
        "lease_remaining_s": None,
        "last_update_age_s": None,
        "last_seq_received": -1,
        "last_seq_applied": -1,
        "lease_expiry_count": 0,
        "pause_for_motion_count": 0,
        "backend_mode": "joint_velocity_lease",
        "backend_timeout_s": 0.2,
        "last_stop_reason": None,
        "stale_packet_rejects": 0,
        "owner_conflict_rejects": 0,
    }

    def _kin_reply(prefix: str) -> tuple[bool, str]:
        return True, f"{prefix},{json.dumps(kinematics_state, separators=(',', ':'))}"

    def _parse_expected_revision(parts: list[str]) -> int | None:
        if len(parts) < 2 or parts[1] == "":
            return None
        return int(parts[1])

    def _session_error(command_name: str, code: str, message: str) -> tuple[bool, str]:
        return False, f"ERROR,{command_name},{_payload_token({'code': code, 'message': message})}"

    def fake_send(command, timeout=0.5, expect_response=True):
        call_log.append((command, timeout, expect_response))
        if command.startswith("JOG_SESSION_START,"):
            payload = json.loads(base64.urlsafe_b64decode(command.split(",", 1)[1].encode("ascii")).decode("utf-8"))
            owner_id = str(payload.get("owner_id", "")).strip()
            if session_state["session_active"] and session_state["owner_id"] != owner_id:
                session_state["owner_conflict_rejects"] += 1
                return _session_error("JOG_SESSION_START", "OWNER_CONFLICT", "Another jog session owner is already active.")
            session_state.update({
                "session_present": True,
                "session_active": True,
                "session_id": "session-123",
                "owner_id": owner_id,
                "state": "active",
                "deadman": bool(payload.get("deadman", True)),
                "lease_remaining_s": 0.4,
                "last_update_age_s": 0.0,
                "last_seq_received": int(payload.get("seq", 0)),
                "last_stop_reason": None,
            })
            return True, f"ACK,JOG_SESSION_START,{_payload_token(dict(session_state))}"
        if command.startswith("JOG_SESSION_UPDATE,"):
            payload = json.loads(base64.urlsafe_b64decode(command.split(",", 1)[1].encode("ascii")).decode("utf-8"))
            session_id = str(payload.get("session_id", "")).strip()
            if session_id != session_state["session_id"]:
                return _session_error("JOG_SESSION_UPDATE", "WRONG_SESSION", "Jog session id does not match the active session.")
            seq = int(payload.get("seq", 0))
            if seq <= int(session_state["last_seq_received"]):
                session_state["stale_packet_rejects"] += 1
                return _session_error("JOG_SESSION_UPDATE", "STALE_SEQUENCE", "Jog session update sequence is stale.")
            session_state.update({
                "deadman": bool(payload.get("deadman", True)),
                "lease_remaining_s": 0.4,
                "last_update_age_s": 0.0,
                "last_seq_received": seq,
                "last_seq_applied": seq,
            })
            return True, f"ACK,JOG_SESSION_UPDATE,{_payload_token(dict(session_state))}"
        if command.startswith("JOG_SESSION_STOP,"):
            payload = json.loads(base64.urlsafe_b64decode(command.split(",", 1)[1].encode("ascii")).decode("utf-8"))
            session_id = str(payload.get("session_id", "")).strip()
            if session_id != session_state["session_id"]:
                return _session_error("JOG_SESSION_STOP", "WRONG_SESSION", "Jog session id does not match the active session.")
            session_state.update({
                "session_active": False,
                "state": "stopped",
                "deadman": False,
                "lease_remaining_s": 0.0,
                "last_stop_reason": str(payload.get("reason", "client-stop")),
            })
            return True, f"ACK,JOG_SESSION_STOP,{_payload_token(dict(session_state))}"
        if command == "GET_JOG_SESSION_STATE":
            return True, f"ACK,GET_JOG_SESSION_STATE,{_payload_token(dict(session_state))}"
        if command == "GET_PERFORMANCE_STATE":
            return (
                True,
                "PERFORMANCE_STATE_JSON,"
                + json.dumps(
                    {
                        "udp": {
                            "last_command": "JOG_SESSION_UPDATE",
                            "last_dispatch_ms": 1.25,
                            "dispatch_ms": {"count": 3, "avg_ms": 0.9, "max_ms": 1.25, "last_ms": 1.25},
                            "interarrival_ms": {"count": 3, "avg_ms": 52.0, "max_ms": 60.0, "last_ms": 50.0},
                        },
                        "jog": {
                            "control_frequency_hz": 50,
                            "loop": {"count": 4, "avg_ms": 3.1, "max_ms": 4.0, "last_ms": 3.2, "overrun_count": 0},
                            "command_state_valid": True,
                            "commanded_pose": {
                                "position_m": {"x": 0.1, "y": 0.2, "z": 0.3},
                                "orientation_euler_deg": {"roll": 10.0, "pitch": 20.0, "yaw": 30.0},
                            },
                            "commanded_joints_deg": [1.0, 2.0, 3.0, 4.0, 5.0, 6.0],
                            "measured_pose": {
                                "position_m": {"x": 0.1, "y": 0.2, "z": 0.3},
                                "orientation_euler_deg": {"roll": 10.0, "pitch": 20.0, "yaw": 30.0},
                            },
                            "measured_joints_deg": [1.0, 2.0, 3.0, 4.0, 5.0, 6.0],
                            "following_error": {
                                "pose": {"position_error_mm": 0.42, "orientation_error_deg": 0.03},
                                "joint": {"max_abs_joint_error_deg": 0.2},
                            },
                            "last_resync_reason": "jog-start",
                            "last_resync_age_s": 0.04,
                            "last_gate_failure_reason": None,
                            "last_gate_failure_details": None,
                            "ik_debug": {
                                "captured_at": "2026-03-24T04:00:00+00:00",
                                "seq": 41,
                                "dt_s": 0.02,
                                "target_vs_solved": {"position_error_mm": 0.42, "orientation_error_deg": 0.03},
                                "target_vs_applied": {"position_error_mm": 1.95, "orientation_error_deg": 0.03},
                                "clamped_joint_indices": [],
                                "clamped": False,
                                "solve_failed": False,
                            },
                        },
                        "jog_session": dict(session_state),
                        "motion_state": "executing",
                        "is_running": False,
                        "is_jogging": bool(session_state["session_active"]),
                        "last_command_age_s": 0.02,
                        "command_link_stale": False,
                        "recent_udp_reset_count": 0,
                    },
                    separators=(",", ":"),
                )
            )
        if command == "GET_RUNTIME_CONFIG":
            return True, f"RUNTIME_CONFIG,{json.dumps(runtime_state, separators=(',', ':'))}"
        if command.startswith("ROTATE,"):
            return True, f"ACK,ROTATE,{_payload_token({**completed_motion_payload, 'axis': 'x'})}"
        if command.startswith("SET_ORIENTATION,"):
            payload = {
                **completed_motion_payload,
                "roll_deg": 10.0,
                "pitch_deg": 20.0,
                "yaw_deg": 30.0,
            }
            return True, f"ACK,SET_ORIENTATION,{_payload_token(payload)}"
        if command.startswith("MOVE_LINE,"):
            return True, f"ACK,MOVE_LINE,{_payload_token(accepted_motion_payload)}"
        if command.startswith("SET_ACTIVE_TOOL,"):
            requested = command.split(",", 1)[1].strip() if "," in command else ""
            resolved = requested or "identity"
            if resolved == "tig-torch-65deg":
                runtime_state["tool"] = {
                    "active_tool_id": "tig-torch-65deg",
                    "display_name": "TIG Torch 65deg",
                    "tool_type": "tig_torch",
                    "source": "live_update",
                    "offset": {
                        "position_mm": {"x": 0.0, "y": 0.0, "z": 0.0},
                        "rotation_deg": {"x": 0.0, "y": 65.0, "z": 0.0},
                    },
                    "mesh": None,
                    "weld": {"torch_axis_angle_from_j6_deg": 65.0},
                }
            else:
                runtime_state["tool"] = {
                    "active_tool_id": "identity",
                    "display_name": "Identity (No Tool Offset)",
                    "tool_type": "utility",
                    "source": "live_update",
                    "offset": {
                        "position_mm": {"x": 0.0, "y": 0.0, "z": 0.0},
                        "rotation_deg": {"x": 0.0, "y": 0.0, "z": 0.0},
                    },
                    "mesh": None,
                    "weld": {},
                }
            return True, f"ACK,SET_ACTIVE_TOOL,{runtime_state['tool']['active_tool_id']}"
        if command.startswith("APPLY_JOINT_SETPOINT,"):
            return True, f"ACK,APPLY_JOINT_SETPOINT,{_payload_token(accepted_motion_payload)}"
        if command.startswith("REQUEST_RESTART,"):
            reason = command.split(",", 1)[1] if "," in command else "api-request"
            return True, f"ACK,REQUEST_RESTART,{reason}"
        if command == "GET_KINEMATICS_PROFILE":
            return _kin_reply("KINEMATICS_PROFILE")
        if command.startswith("PATCH_RUNTIME_OFFSETS,"):
            parts = command.split(",", 2)
            expected = _parse_expected_revision(parts)
            if expected is not None and expected != kinematics_state["revision"]:
                return False, (
                    f"ERROR,KINEMATICS,STALE_REVISION,Stale revision: expected={expected}, "
                    f"active={kinematics_state['revision']}."
                )
            payload = json.loads(base64.urlsafe_b64decode(parts[2].encode("ascii")).decode("utf-8"))
            if "base" in payload:
                kinematics_state["offsets"]["base"] = payload["base"]
            if "tool" in payload:
                kinematics_state["offsets"]["tool"] = payload["tool"]
            kinematics_state["revision"] += 1
            return _kin_reply("KINEMATICS_OK")
        if command.startswith("RESET_RUNTIME_OFFSETS,"):
            parts = command.split(",", 1)
            expected = _parse_expected_revision(parts + [""] if len(parts) == 1 else parts)
            if expected is not None and expected != kinematics_state["revision"]:
                return False, (
                    f"ERROR,KINEMATICS,STALE_REVISION,Stale revision: expected={expected}, "
                    f"active={kinematics_state['revision']}."
                )
            kinematics_state["offsets"] = {
                "base": {"position_m": {"x": 0.0, "y": 0.0, "z": 0.0}, "rotation_deg": {"x": 0.0, "y": 0.0, "z": 0.0}},
                "tool": {"position_m": {"x": 0.0, "y": 0.0, "z": 0.0}, "rotation_deg": {"x": 0.0, "y": 0.0, "z": 0.0}},
            }
            kinematics_state["revision"] += 1
            return _kin_reply("KINEMATICS_OK")
        if command.startswith("APPLY_KINEMATICS_PROFILE,"):
            parts = command.split(",", 2)
            expected = _parse_expected_revision(parts)
            if expected is not None and expected != kinematics_state["revision"]:
                return False, (
                    f"ERROR,KINEMATICS,STALE_REVISION,Stale revision: expected={expected}, "
                    f"active={kinematics_state['revision']}."
                )
            profile_payload = json.loads(base64.urlsafe_b64decode(parts[2].encode("ascii")).decode("utf-8"))
            kinematics_state["profile"] = profile_payload
            kinematics_state["revision"] += 1
            return _kin_reply("KINEMATICS_OK")
        if command.startswith("PLAN_TRAJECTORY_POINTS"):
            return True, f"PLANNED_TRAJECTORY_POINTS,{json.dumps(planner_payload)}"
        if command in no_reply_commands:
            if expect_response:
                return False, f"unexpected response requested for {command}"
            return True, ""
        if not expect_response:
            return True, ""
        return responses.get(command, (False, f"unexpected {command}"))

    class DummyCommandApi:
        sample_traj = {
            "description": "Sample trajectory",
            "loop": False,
            "orientation_euler_angles_deg": None,
            "moves": [
                {"command": "move_absolute", "vector": [0.1, 0.2, 0.3]},
                {"command": "pause", "duration": 1.0},
                {"command": "move_absolute", "vector": [0.4, 0.5, 0.6]},
            ],
        }

        @staticmethod
        def _load_trajectory_by_name(name):
            if name in {"alpha", "beta", "__planner_preview__"}:
                return DummyCommandApi.sample_traj
            return None

        @staticmethod
        def plan_preview_trajectory_points(
            points,
            preview_name="__planner_preview__",
            weld_metadata=None,
            sections=None,
        ):
            if not points:
                raise ValueError("no points")
            body = dict(planner_payload)
            body["name"] = preview_name
            body["waypoints"] = points
            body["cartesian_path"] = points
            body["trajectory"] = dict(DummyCommandApi.sample_traj)
            if weld_metadata:
                body["trajectory"]["weld"] = weld_metadata
            return body

    class DummyTopologyService:
        model = {
            "model_id": "step-test",
            "filename": "fixture.step",
            "fingerprint": "abc123",
            "parts": [{"id": "part_0", "edge_count": 1}],
            "edges": [
                {
                    "id": "part_0:edge_00000",
                    "part_id": "part_0",
                    "samples": [[0.0, 0.0, 0.0], [0.1, 0.0, 0.0], [0.2, 0.0, 0.0]],
                }
            ],
        }

        def load_step(self, *, filename, step_bytes, sample_count):
            assert filename
            assert step_bytes
            return dict(self.model)

        def get_model(self, model_id):
            if model_id != self.model["model_id"]:
                raise KeyError(model_id)
            return dict(self.model)

        def sample_edge_segment(self, *, model_id, edge_id, start_s, end_s, sample_count):
            assert model_id == self.model["model_id"]
            assert edge_id == self.model["edges"][0]["id"]
            return [
                (0.0, 0.0, 0.0),
                (0.1, 0.0, 0.0),
                (0.2, 0.0, 0.0),
            ]

    monkeypatch.setattr("gradient_os.api.main._send_controller_command", fake_send)
    monkeypatch.setattr(
        "gradient_os.api.main._probe_controller", lambda timeout=0.5: (True, "ok")
    )
    monkeypatch.setattr(
        "gradient_os.api.main.controller_command_api", DummyCommandApi
    )
    monkeypatch.setattr(
        "gradient_os.api.main.topology_service", DummyTopologyService()
    )
    monkeypatch.setattr(
        "gradient_os.api.main._WELD_PROGRAM_DIR", tempfile.mkdtemp(prefix="weld-programs-")
    )
    runtime_cfg_path = os.path.join(tempfile.mkdtemp(prefix="runtime-config-"), "runtime.json")
    monkeypatch.setenv("GRADIENT_RUNTIME_CONFIG_PATH", runtime_cfg_path)
    tool_library_root = tempfile.mkdtemp(prefix="tool-library-")
    monkeypatch.setenv("GRADIENT_TOOL_LIBRARY_PATH", tool_library_root)
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
    body = resp.json()
    assert body["detail"].startswith("ACK,STOP,")
    assert body["state"] == "aborted"
    assert body["completion_scope"] == "rtcore_execution"


def test_control_power_down(client):
    resp = client.post("/control/power-down")
    assert resp.status_code == 200
    assert resp.json()["detail"] == "ACK,SAFE_POWER_DOWN,POWER_DOWN_SENT"
    assert resp.json()["waited_for_idle"] is False
    assert client.command_calls[-1] == ("SAFE_POWER_DOWN", 5.0, True)


def test_control_power_down_waits_when_requested(client):
    resp = client.post("/control/power-down", json={"wait_for_idle": True})
    assert resp.status_code == 200
    assert resp.json()["detail"] == "ACK,SAFE_POWER_DOWN,POWER_DOWN_SENT"
    assert resp.json()["waited_for_idle"] is True
    assert client.command_calls[-1] == ("SAFE_POWER_DOWN,wait", 5.0, True)


def test_control_wait_for_idle(client):
    resp = client.post("/control/wait-for-idle")
    assert resp.status_code == 200
    body = resp.json()
    assert body["detail"].startswith("ACK,WAIT_FOR_IDLE,")
    assert body["state"] == "completed"
    assert body["completion_scope"] == "rtcore_execution"


def test_control_wait_for_idle_accepts_timeout_override(client):
    resp = client.post("/control/wait-for-idle", json={"timeout_s": 12.5})
    assert resp.status_code == 200
    body = resp.json()
    assert body["wait_timeout_s"] == 12.5
    assert client.command_calls[-1] == ("WAIT_FOR_IDLE,12.5", 17.5, True)


def test_control_motion_status(client):
    resp = client.get("/control/motion-status")
    assert resp.status_code == 200
    body = resp.json()
    assert body["detail"].startswith("MOTION_STATUS,")
    assert body["state"] == "accepted"
    assert body["trajectory_id"] == 7
    assert body["execution"]["state_name"] == "queued"
    assert body["program"]["name"] == "alpha"
    assert body["program"]["state"] == "executing"
    assert body["program_current_step_type"] == "pause"


def test_control_rotate_returns_motion_metadata(client):
    resp = client.post(
        "/control/rotate",
        json={"axis": "roll", "angle_deg": 15.0, "duration_s": 0.5},
    )
    assert resp.status_code == 200
    body = resp.json()
    assert body["detail"].startswith("ACK,ROTATE,")
    assert body["state"] == "completed"
    assert body["completion_scope"] == "rtcore_execution"
    assert client.command_calls[-1] == ("ROTATE,x,15.0,0.5", 2.0, True)


def test_control_set_orientation_returns_motion_metadata(client):
    resp = client.post(
        "/control/set-orientation",
        json={"roll": 10.0, "pitch": 20.0, "yaw": 30.0},
    )
    assert resp.status_code == 200
    body = resp.json()
    assert body["detail"].startswith("ACK,SET_ORIENTATION,")
    assert body["state"] == "completed"
    assert body["completion_scope"] == "rtcore_execution"
    assert client.command_calls[-1] == ("SET_ORIENTATION,10.0,20.0,30.0", 2.0, True)


def test_control_reset_faults_all(client):
    resp = client.post("/control/reset-faults")
    assert resp.status_code == 200
    assert resp.json()["detail"] == "ACK,RESET_FAULTS,ALL"
    assert resp.json()["joint"] is None
    assert client.command_calls[-1] == ("RESET_FAULTS", 5.0, True)


def test_control_power_up(client):
    resp = client.post("/control/power-up")
    assert resp.status_code == 200
    assert resp.json()["detail"] == "ACK,SAFE_POWER_UP,POWER_UP_SENT"
    assert client.command_calls[-1] == ("SAFE_POWER_UP", 5.0, True)


def test_control_reset_faults_joint(client):
    resp = client.post("/control/reset-faults", json={"joint": 1})
    assert resp.status_code == 200
    assert resp.json()["detail"] == "ACK,RESET_FAULTS,JOINT,1"
    assert resp.json()["joint"] == 1
    assert client.command_calls[-1] == ("RESET_FAULTS,1", 5.0, True)


def test_control_home(client):
    resp = client.post("/control/home")
    assert resp.status_code == 200
    body = resp.json()
    assert body["completion_scope"] == "rtcore_execution"
    assert body["trajectory_id"] == 7
    command, timeout, expect_response = client.command_calls[-1]
    assert timeout == 2.0
    assert expect_response is True
    assert command.startswith("APPLY_JOINT_SETPOINT,")
    payload = json.loads(base64.urlsafe_b64decode(command.split(",", 1)[1]).decode("utf-8"))
    assert payload["arm_angles_rad"] == [0.0] * 6
    assert payload["max_motor_rpm"] == pytest.approx(100.0)


def test_control_rest(client):
    resp = client.post("/control/rest")
    assert resp.status_code == 200
    body = resp.json()
    assert body["completion_scope"] == "rtcore_execution"
    assert body["trajectory_id"] == 7
    command, timeout, expect_response = client.command_calls[-1]
    assert timeout == 2.0
    assert expect_response is True
    assert command.startswith("APPLY_JOINT_SETPOINT,")
    payload = json.loads(base64.urlsafe_b64decode(command.split(",", 1)[1]).decode("utf-8"))
    assert payload["arm_angles_rad"] == pytest.approx([0.0, -1.4, 1.5, 0.0, 0.0, 0.0])
    assert payload["max_motor_rpm"] == pytest.approx(100.0)


def test_control_zero_joint(client):
    resp = client.post("/control/zero-joint", json={"joint": 3})
    assert resp.status_code == 200
    assert resp.json()["joint"] == 3
    assert resp.json()["detail"] == "ACK,ZERO_JOINT,3"
    assert client.command_calls[-1] == ("ZERO_JOINT,3", 5.0, True)


def test_control_joint_jog(client):
    resp = client.post("/control/joint-jog", json={"joint": 3, "delta_deg": 2.5})
    assert resp.status_code == 200
    body = resp.json()
    assert body["joint"] == 3
    assert body["delta_deg"] == pytest.approx(2.5)
    assert body["target_arm_deg"] == [1.0, 2.0, 5.5, 4.0, 5.0, 6.0]
    assert body["target_arm_rad"] == pytest.approx([
        0.017453292519943295,
        0.03490658503988659,
        0.09599310885968812,
        0.06981317007977318,
        0.08726646259971647,
        0.10471975511965978,
    ])
    assert body["command_acknowledged"] is True
    assert body["completion_scope"] == "rtcore_execution"
    assert body["state"] == "accepted"
    assert body["trajectory_id"] == 7
    assert body["waited_for_idle"] is False
    assert client.command_calls[-2] == ("GET_JOINT_ANGLES", 1.0, True)
    last_command, timeout_s, expect_response = client.command_calls[-1]
    assert timeout_s == 2.0
    assert expect_response is True
    assert last_command.startswith("APPLY_JOINT_SETPOINT,")


def test_control_joint_jog_surfaces_backend_rejection(client, monkeypatch):
    def fake_send(command: str, timeout: float = 0.5, expect_response: bool = True):
        if command == "GET_JOINT_ANGLES":
            return True, "JOINT_ANGLES,1,2,3,4,5,6,7"
        if command.startswith("APPLY_JOINT_SETPOINT,"):
            return False, "ERROR,APPLY_JOINT_SETPOINT,RTCore did not provide a setpoint slot"
        return True, "ACK"

    monkeypatch.setattr("gradient_os.api.main._send_controller_command", fake_send)

    resp = client.post("/control/joint-jog", json={"joint": 1, "delta_deg": 1.0})

    assert resp.status_code == 503
    assert resp.json()["detail"] == {
        "code": "APPLY_JOINT_SETPOINT_REJECTED",
        "message": "RTCore did not provide a setpoint slot",
    }


def test_info_status(client):
    resp = client.get("/info/status")
    assert resp.status_code == 200
    assert resp.json() == {"gripper_present": True}


def test_info_robots(client):
    resp = client.get("/info/robots")
    assert resp.status_code == 200
    body = resp.json()
    assert body["default_robot_id"] == "gradient-05"
    names = [item["name"] for item in body["robots"]]
    assert "gradient05" in names
    assert "gradient0" in names


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
    body = resp.json()
    assert body["arm_deg"] == [1, 2, 3, 4, 5, 6]
    assert body["arm_rad"] == pytest.approx([
        0.017453292519943295,
        0.03490658503988659,
        0.05235987755982989,
        0.06981317007977318,
        0.08726646259971647,
        0.10471975511965978,
    ])
    assert body["gripper_deg"] == 7
    assert body["gripper_rad"] == pytest.approx(0.12217304763960307)


def test_info_joints_detailed(client):
    resp = client.get("/info/joints-detailed")
    assert resp.status_code == 200
    assert resp.json() == {
        "arm_rad": [0.1, 0.2, 0.3, 0.4, 0.5, 0.6],
        "arm_deg": [1.0, 2.0, 3.0, 4.0, 5.0, 6.0],
        "gripper_rad": 0.7,
        "gripper_deg": 7.0,
        "axis_counts": [101, 202, 303, 404, 505, 606],
        "axis_torque_raw": [11, 12, 13, 14, 15, 16],
        "axis_statusword": [4663, 4663, 4663, 4663, 4663, 4663],
        "axis_error_code": [0, 0, 0, 0, 0, 0],
        "axis_mode_display": [8, 8, 8, 8, 8, 8],
        "axis_mode_display_name": [
            "cyclic_sync_position",
            "cyclic_sync_position",
            "cyclic_sync_position",
            "cyclic_sync_position",
            "cyclic_sync_position",
            "cyclic_sync_position",
        ],
        "axis_ds402_state_code": [5, 5, 5, 5, 5, 5],
        "axis_di_bits": [1, 2, 4, 8, 16, 32],
        "axis_fault_flags": [0, 0, 0, 0, 0, 0],
        "axis_brake_state": [1, 1, 1, 1, 1, 1],
        "axis_to_joint": [0, 1, 2, 3, 4, 5],
        "backend_name": "ethercat_rtcore",
        "read_source": "live_feedback",
        "numeric_precision": "float64",
    }


def test_debug_performance(client, monkeypatch, tmp_path):
    metrics_path = tmp_path / "metrics.json"
    metrics_path.write_text(
        json.dumps(
            {
                "rt_frequency_hz": 1000,
                "rt_last_jitter_ns": 1200,
                "rt_max_abs_jitter_ns": 3200,
                "rt_overrun_count": 2,
                "wkc_actual": 14,
                "wkc_expected": 14,
                "motion_active_command_seq": 77,
                "motion_last_update_age_ms": 4.2,
                "feedback_cycle_jitter_ns": 800,
            }
        ),
        encoding="utf-8",
    )
    monkeypatch.setenv("GRADIENT_RTCORE_METRICS", str(metrics_path))

    resp = client.get("/debug/performance")
    assert resp.status_code == 200
    body = resp.json()
    assert body["controller"]["motion_state"] == "executing"
    assert body["controller"]["udp"]["last_command"] == "JOG_SESSION_UPDATE"
    assert body["controller"]["jog"]["control_frequency_hz"] == 50
    assert body["controller"]["jog"]["ik_debug"]["seq"] == 41
    assert body["controller"]["jog"]["ik_debug"]["target_vs_solved"]["position_error_mm"] == 0.42
    assert body["controller"]["jog"]["ik_debug"]["clamped_joint_indices"] == []
    assert body["controller"]["jog"]["command_state_valid"] is True
    assert body["controller"]["jog"]["last_resync_reason"] == "jog-start"
    assert body["controller"]["jog_session"]["state"] == "idle"
    assert body["controller"]["jog_session"]["session_active"] is False
    assert body["controller"]["pose"]["position_m"] == {"x": 0.1, "y": 0.2, "z": 0.3}
    assert body["controller"]["pose"]["orientation_euler_deg"] == {"roll": 10.0, "pitch": 20.0, "yaw": 30.0}
    assert body["controller"]["pose"]["joints_deg"] == [1.0, 2.0, 3.0, 4.0, 5.0, 6.0]
    assert body["rtcore"]["rt_frequency_hz"] == 1000
    assert body["rtcore"]["rt_overrun_count"] == 2


def test_debug_pose_history_save(client, monkeypatch, tmp_path):
    monkeypatch.setattr(api_main, "_DIAGNOSTICS_LOG_DIR", str(tmp_path))
    payload = {
        "exported_at": "2026-03-24T03:57:55.780Z",
        "sample_count": 1,
        "pose_history": [
            {
                "collected_at": "2026-03-24T03:57:55.780Z",
                "motion_state": "idle",
                "is_jogging": False,
                "session_state": "stopped",
                "pose": {
                    "position_m": {"x": 0.1, "y": 0.2, "z": 0.3},
                    "orientation_euler_deg": {"roll": 10.0, "pitch": 20.0, "yaw": 30.0},
                    "joints_deg": [1, 2, 3, 4, 5, 6],
                },
            }
        ],
    }
    resp = client.post("/debug/pose-history", json=payload)
    assert resp.status_code == 200
    body = resp.json()
    assert body["status"] == "ok"
    assert body["sample_count"] == 1
    saved_path = body["path"]
    saved_doc = json.loads(Path(saved_path).read_text(encoding="utf-8"))
    assert saved_doc["sample_count"] == 1
    assert saved_doc["pose_history"][0]["pose"]["position_m"] == {"x": 0.1, "y": 0.2, "z": 0.3}
    assert "saved_at" in saved_doc


def test_debug_runtime(client, monkeypatch):
    snapshot = {
        "collected_at": "2026-03-23T04:10:00+00:00",
        "host": {"hostname": "revpi"},
        "resources": {"memory": {"used_percent": 72.5}, "swap": {"used_percent": 10.0}},
        "disk": {"project_root": {"free_bytes": 123456}},
        "processes": {
            "current_process": {"pid": 101},
            "interesting": {"browser": [{"pid": 202, "name": "chromium"}]},
            "top_rss": [],
        },
        "raspberry_pi": {"thermal_zone0_c": 58.2},
        "kernel_hints": {"matches": ["Killed process 999 (chromium) total-vm:1234kB"]},
        "latest_startup_logs": {
            "latest_path": "/tmp/latest",
            "logs": {"web.log": ["VITE ready"]},
        },
        "probes": {"web_root": {"ok": True, "status": 200}},
    }

    def _fake_snapshot(project_root, *, include_local_probes=False):
        assert include_local_probes is True
        assert project_root
        return snapshot

    monkeypatch.setattr(api_main, "get_runtime_diagnostics_snapshot", _fake_snapshot)

    resp = client.get("/debug/runtime")
    assert resp.status_code == 200
    assert resp.json() == snapshot


def test_control_jog_session_start_update_stop_and_state(client):
    start_len = len(client.command_calls)

    resp = client.post(
        "/control/jog/session/start",
        json={
            "owner_id": "ui-a",
            "seq": 0,
            "deadman": True,
            "vx": 0.05,
            "vy": 0.0,
            "vz": 0.0,
            "v_roll": 0.0,
            "v_pitch": 0.0,
            "v_yaw": 0.0,
        },
    )
    assert resp.status_code == 200
    body = resp.json()
    assert body["session"]["session_active"] is True
    assert body["session"]["session_id"] == "session-123"
    assert body["session"]["last_seq_received"] == 0

    commands = [entry[0] for entry in client.command_calls[start_len:]]
    assert len(commands) == 1
    assert commands[0].startswith("JOG_SESSION_START,")

    update_len = len(client.command_calls)
    resp = client.post(
        "/control/jog/session/update",
        json={
            "session_id": "session-123",
            "owner_id": "ui-a",
            "seq": 1,
            "deadman": True,
            "vx": 0.01,
            "vy": 0.0,
            "vz": 0.0,
            "v_roll": 0.0,
            "v_pitch": 0.0,
            "v_yaw": 0.0,
        },
    )
    assert resp.status_code == 200
    update_body = resp.json()
    assert update_body["session"]["last_seq_received"] == 1

    state_resp = client.get("/control/jog/session/state")
    assert state_resp.status_code == 200
    assert state_resp.json()["session"]["session_id"] == "session-123"

    stop_len = len(client.command_calls)
    resp = client.post(
        "/control/jog/session/stop",
        json={
            "session_id": "session-123",
            "owner_id": "ui-a",
            "reason": "ui-release",
        },
    )
    assert resp.status_code == 200
    stop_body = resp.json()
    assert stop_body["session"]["session_active"] is False
    assert stop_body["session"]["last_stop_reason"] == "ui-release"
    stop_commands = [entry[0] for entry in client.command_calls[update_len:]]
    assert stop_commands[0].startswith("JOG_SESSION_UPDATE,")
    assert stop_commands[1] == "GET_JOG_SESSION_STATE"
    assert stop_commands[2].startswith("JOG_SESSION_STOP,")


def test_control_jog_session_payload_parsing_is_fail_closed(client):
    start_len = len(client.command_calls)

    resp = client.post(
        "/control/jog/session/start",
        json={
            "owner_id": "ui-a",
            "seq": "4",
            "deadman": "false",
            "vx": 0.05,
            "vy": 0.0,
            "vz": 0.0,
            "v_roll": 0.0,
            "v_pitch": 0.0,
            "v_yaw": 0.0,
        },
    )
    assert resp.status_code == 200
    assert resp.json()["session"]["deadman"] is False

    start_command = client.command_calls[start_len][0]
    start_payload = _decode_command_payload(start_command)
    assert start_payload["seq"] == 4
    assert start_payload["deadman"] is False

    bad_resp = client.post(
        "/control/jog/session/update",
        json={
            "session_id": "session-123",
            "owner_id": "ui-a",
            "seq": "not-an-int",
            "deadman": True,
            "vx": 0.01,
            "vy": 0.0,
            "vz": 0.0,
            "v_roll": 0.0,
            "v_pitch": 0.0,
            "v_yaw": 0.0,
        },
    )
    assert bad_resp.status_code == 400
    assert bad_resp.json()["detail"] == "seq must be an integer"


def test_legacy_jog_routes_are_removed(client):
    legacy_requests = [
        ("post", "/control/jog/start", None),
        ("post", "/control/jog/stop", None),
        ("post", "/control/jog/velocity", {"vx": 0.0, "vy": 0.0, "vz": 0.0, "v_roll": 0.0, "v_pitch": 0.0, "v_yaw": 0.0}),
        ("post", "/control/jog/deadman", {"enabled": False}),
        ("post", "/control/jog/state", {"active": False}),
    ]

    for method, path, payload in legacy_requests:
        response = getattr(client, method)(path, json=payload) if payload is not None else getattr(client, method)(path)
        assert response.status_code == 404


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
    body = resp.json()
    assert body["completion_scope"] == "controller_program_thread"
    assert body["state"] == "accepted"
    assert body["program"]["name"] == "alpha"
    assert body["program"]["state"] == "accepted"
    assert body["program_segment_execution_policy"] == "rtcore_queued"
    assert body["program_rtcore_segments"] is True
    assert client.command_calls[-1] == ("RUN_TRAJECTORY,alpha,false", 2.0, True)


def test_trajectory_plan_points_success(client):
    resp = client.post(
        "/trajectory/plan-points",
        json={"points": [{"x": 0.1, "y": 0.2, "z": 0.3}, [0.4, 0.5, 0.6]]},
    )
    assert resp.status_code == 200
    body = resp.json()
    assert body["name"] == "__planner_preview__"
    assert body["trajectory"]["moves"][0]["vector"] == [0.1, 0.2, 0.3]
    assert client.command_calls[-1][0].startswith("PLAN_TRAJECTORY_POINTS,0.1,0.2,0.3,0.4,0.5,0.6")
    assert client.command_calls[-1][2] is True


def test_trajectory_plan_points_validation(client):
    start_len = len(client.command_calls)
    resp = client.post("/trajectory/plan-points", json={"points": [{"x": 1.0}]})
    assert resp.status_code == 400
    assert len(client.command_calls) == start_len


def test_trajectory_detail(client):
    resp = client.get("/trajectory/detail/alpha")
    assert resp.status_code == 200
    body = resp.json()
    assert body["name"] == "alpha"
    assert body["trajectory"]["moves"][0]["vector"] == [0.1, 0.2, 0.3]


def test_cad_topology_load_and_get(client):
    raw = b"STEP-MOCK"
    encoded = base64.b64encode(raw).decode("ascii")
    resp = client.post(
        "/cad/topology/load-step",
        json={"filename": "fixture.step", "step_base64": encoded},
    )
    assert resp.status_code == 200
    body = resp.json()
    assert body["model_id"] == "step-test"
    assert body["edges"][0]["id"] == "part_0:edge_00000"

    resp = client.get("/cad/topology/step-test")
    assert resp.status_code == 200
    assert resp.json()["parts"][0]["id"] == "part_0"


def test_trajectory_plan_weld(client):
    resp = client.post(
        "/trajectory/plan-weld",
        json={
            "model_id": "step-test",
            "edge_id": "part_0:edge_00000",
            "start_s": 0.1,
            "end_s": 0.9,
            "weld_type": "fillet",
            "weld_name": "test weld",
            "options": {"spinAngleDeg": 22.5},
        },
    )
    assert resp.status_code == 200
    body = resp.json()
    assert body["name"] == "__weld_preview__"
    assert body["trajectory"]["weld"]["type"] == "fillet"
    assert body["trajectory"]["weld"]["options"]["spin_angle_deg"] == pytest.approx(22.5)
    assert body["source"]["mode"] == "edge_segment"


def test_weld_program_save_list_load(client):
    step_payload = base64.b64encode(b"STEP-MOCK").decode("ascii")
    save_resp = client.post(
        "/weld-program/save",
        json={
            "name": "demo_program",
            "step": {
                "filename": "fixture.step",
                "step_base64": step_payload,
                "transform": {
                    "position": {"x": 0.1, "y": 0.2, "z": 0.3},
                    "rotationDeg": {"x": 1.0, "y": 2.0, "z": 3.0},
                    "scale": 1.5,
                },
            },
            "weld_draft": {
                "modelId": "step-test",
                "edgeId": "part_0:edge_00000",
                "weldType": "fillet",
                "weldName": "demo weld",
                "startS": 0.2,
                "endS": 0.8,
                "spinAngleDeg": 17.0,
            },
            "editable_waypoints": [{"x": 0.0, "y": 0.0, "z": 0.0}, {"x": 0.2, "y": 0.0, "z": 0.0}],
            "planned_trajectory": {"name": "__weld_preview__", "waypoints": [{"x": 0.0, "y": 0.0, "z": 0.0}]},
        },
    )
    assert save_resp.status_code == 200
    assert save_resp.json()["name"] == "demo_program"

    list_resp = client.get("/weld-program/list")
    assert list_resp.status_code == 200
    assert "demo_program" in list_resp.json()["programs"]

    load_resp = client.get("/weld-program/demo_program")
    assert load_resp.status_code == 200
    payload = load_resp.json()
    assert payload["name"] == "demo_program"
    assert payload["weld_draft"]["edgeId"] == "part_0:edge_00000"
    assert payload["weld_draft"]["spinAngleDeg"] == pytest.approx(17.0)
    assert payload["step"]["filename"] == "fixture.step"


def test_preview_execute_clear(client, monkeypatch):
    planned_path = [
        [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        [0.05, 0.04, 0.03, 0.02, 0.01, 0.0],
    ]

    def fake_plan(start_q, target_pos, velocity, acceleration, frequency, use_smoothing):
        return planned_path

    monkeypatch.setattr(
        "gradient_os.arm_controller.trajectory_execution._plan_smooth_move",
        fake_plan,
    )

    def fake_fk(joints):
        return [0.2, 0.1, 0.3]

    monkeypatch.setattr(
        "gradient_os.ik_solver.get_fk",
        fake_fk,
    )

    resp = client.post(
        "/trajectory/preview",
        json={"x": 0.2, "y": 0.1, "z": 0.3, "velocity": 0.2, "acceleration": 0.1, "closed_loop": False},
    )
    assert resp.status_code == 200
    body = resp.json()
    assert body["joints_rad"] == planned_path
    assert body.get("cartesian_m") == [[0.2, 0.1, 0.3], [0.2, 0.1, 0.3]]
    assert client.command_calls[-1] == ("GET_POSITION", 1.0, True)

    resp = client.post("/trajectory/execute-preview")
    assert resp.status_code == 200
    body = resp.json()
    assert body["dispatch_detail"].startswith("ACK,MOVE_LINE,")
    assert body["detail"].startswith("ACK,WAIT_FOR_IDLE,")
    assert body["dispatch"]["state"] == "accepted"
    assert body["state"] == "completed"
    # Last two commands: MOVE_LINE..., WAIT_FOR_IDLE
    assert client.command_calls[-2] == ("MOVE_LINE,0.2,0.1,0.3,0.2,0.1,false", 5.0, True)
    assert client.command_calls[-1] == ("WAIT_FOR_IDLE", 60.0, True)

    # Preview cleared, executing again should fail
    resp = client.post("/trajectory/execute-preview")
    assert resp.status_code == 404

    resp = client.post("/trajectory/clear-preview")
    assert resp.status_code == 200


def test_kinematics_profile_get(client):
    resp = client.get("/kinematics/profile")
    assert resp.status_code == 200
    body = resp.json()
    assert body["revision"] == 0
    assert body["profile"]["robot_id"] == "mini-6dof-arm"


def test_kinematics_patch_and_stale_revision(client):
    patch_resp = client.patch(
        "/kinematics/runtime-offsets",
        json={
            "expected_revision": 0,
            "tool": {
                "position_m": {"x": 0.01, "y": 0.0, "z": 0.0},
                "rotation_deg": {"x": 0.0, "y": 0.0, "z": 0.0},
            },
        },
    )
    assert patch_resp.status_code == 200
    assert patch_resp.json()["revision"] == 1
    stale_resp = client.patch(
        "/kinematics/runtime-offsets",
        json={
            "expected_revision": 0,
            "tool": {
                "position_m": {"x": 0.02, "y": 0.0, "z": 0.0},
                "rotation_deg": {"x": 0.0, "y": 0.0, "z": 0.0},
            },
        },
    )
    assert stale_resp.status_code == 409
    assert stale_resp.json()["detail"]["code"] == "STALE_REVISION"


def test_kinematics_reset_and_apply_profile(client):
    reset_resp = client.post("/kinematics/runtime-offsets/reset", json={"expected_revision": 0})
    assert reset_resp.status_code == 200
    assert reset_resp.json()["revision"] == 1

    profile = reset_resp.json()["profile"]
    profile["profile_id"] = "mini-6dof-arm:applied"
    apply_resp = client.post(
        "/kinematics/profile/apply",
        json={"expected_revision": 1, "profile": profile},
    )
    assert apply_resp.status_code == 200
    assert apply_resp.json()["revision"] == 2
    assert apply_resp.json()["profile"]["profile_id"] == "mini-6dof-arm:applied"


def test_runtime_config_get_and_patch(client):
    get_resp = client.get("/info/runtime-config")
    assert get_resp.status_code == 200
    body = get_resp.json()
    assert body["active"]["robot"]["name"] == "gradient05"
    assert body["desired"]["robot"] == "gradient05"
    assert body["restart_required"] is False

    patch_resp = client.patch(
        "/info/runtime-config",
        json={"robot": "gradient0", "active_tool_id": "tig-torch-65deg", "actor": "pytest"},
    )
    assert patch_resp.status_code == 200
    patch_body = patch_resp.json()
    assert patch_body["desired"]["robot"] == "gradient0"
    assert patch_body["desired"]["active_tool_id"] == "tig-torch-65deg"
    assert patch_body["restart_required"] is True


def test_runtime_config_patch_applies_tool_live_without_restart(client):
    patch_resp = client.patch(
        "/info/runtime-config",
        json={"active_tool_id": "tig-torch-65deg", "actor": "pytest"},
    )
    assert patch_resp.status_code == 200
    patch_body = patch_resp.json()
    assert patch_body["desired"]["active_tool_id"] == "tig-torch-65deg"
    assert patch_body["active"]["tool"]["active_tool_id"] == "tig-torch-65deg"
    assert patch_body["restart_required"] is False
    commands = [entry[0] for entry in client.command_calls]
    assert "SET_ACTIVE_TOOL,tig-torch-65deg" in commands


def test_send_controller_command_uses_large_udp_receive_buffer(monkeypatch):
    recv_sizes: list[int] = []

    class FakeSocket:
        def settimeout(self, _timeout):
            return None

        def sendto(self, _payload, _addr):
            return None

        def recvfrom(self, bufsize):
            recv_sizes.append(int(bufsize))
            return (b"RUNTIME_CONFIG,{}", ("127.0.0.1", 3000))

        def close(self):
            return None

    monkeypatch.setattr("gradient_os.api.main._resolve_controller_endpoint", lambda: ("127.0.0.1", 3000))
    monkeypatch.setattr("gradient_os.api.main.socket.socket", lambda *_args, **_kwargs: FakeSocket())

    ok, detail = api_main._send_controller_command("GET_RUNTIME_CONFIG", timeout=1.0, expect_response=True)
    assert ok is True
    assert detail == "RUNTIME_CONFIG,{}"
    assert recv_sizes == [api_main._CONTROLLER_REPLY_MAX_BYTES]


def test_tools_library_crud_and_filter(client):
    list_resp = client.get("/tools/library")
    assert list_resp.status_code == 200
    body = list_resp.json()
    tool_ids = {item["tool_id"] for item in body["tools"]}
    assert "identity" in tool_ids
    assert "tig-torch-65deg" in tool_ids

    create_resp = client.post(
        "/tools/library",
        json={
            "actor": "pytest",
            "tool": {
                "tool_id": "pytest-tool",
                "display_name": "Pytest Tool",
                "tool_type": "fixture",
                "keywords": ["pytest", "fixture"],
                "compatible_robot_ids": ["gradient-05"],
                "offset": {
                    "position_mm": {"x": 10.0, "y": 0.0, "z": 50.0},
                    "rotation_deg": {"x": 0.0, "y": 10.0, "z": 0.0},
                },
                "mesh": None,
            },
        },
    )
    assert create_resp.status_code == 200
    assert create_resp.json()["tool"]["tool_id"] == "pytest-tool"

    get_resp = client.get("/tools/library/pytest-tool")
    assert get_resp.status_code == 200
    assert get_resp.json()["display_name"] == "Pytest Tool"

    filtered_resp = client.get("/tools/library?tool_type=fixture&q=pytest")
    assert filtered_resp.status_code == 200
    filtered_ids = {item["tool_id"] for item in filtered_resp.json()["tools"]}
    assert "pytest-tool" in filtered_ids

    patch_resp = client.patch(
        "/tools/library/pytest-tool",
        json={
            "actor": "pytest",
            "tool": {
                "display_name": "Pytest Tool Updated",
                "offset": {
                    "position_mm": {"x": 12.0, "y": 1.0, "z": 55.0},
                    "rotation_deg": {"x": 1.0, "y": 11.0, "z": 2.0},
                },
            },
        },
    )
    assert patch_resp.status_code == 200
    assert patch_resp.json()["tool"]["display_name"] == "Pytest Tool Updated"

    delete_resp = client.delete("/tools/library/pytest-tool?actor=pytest")
    assert delete_resp.status_code == 200
    assert delete_resp.json()["deleted_tool_id"] == "pytest-tool"


def test_control_restart_controller(client):
    resp = client.post("/control/restart-controller", json={"reason": "test-restart"})
    assert resp.status_code == 200
    body = resp.json()
    assert body["restart_requested"] is True
    assert body["detail"].startswith("ACK,REQUEST_RESTART,test-restart")
