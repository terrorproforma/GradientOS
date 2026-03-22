from __future__ import annotations

import importlib.util
import struct
import time
from pathlib import Path

import pytest

from gradient_os.arm_controller.backends import registry as backend_registry
from gradient_os.arm_controller.backends.ethercat_rtcore.backend import (
    EthercatRTCoreBackend,
    RTCoreExecutionStatus,
    _TRAJ_POINTF_HAS_VELOCITY,
    _AxisConfig,
)
from gradient_os.arm_controller.backends.ethercat_rtcore.runtime import RTCORE_EXEC_STATE_COMPLETED
from gradient_os.arm_controller.backends.simulation.backend import SimulationBackend
from gradient_os.arm_controller.robots.gradient05.config import Gradient05Config
from gradient_os import ik_solver
from gradient_os.joint_zero_offsets import load_joint_zero_offsets
from gradient_os.run_controller import _rtcore_metrics_ready


def _load_sync_script_module():
    project_root = Path(__file__).resolve().parents[1]
    script_path = project_root / "scripts" / "sync_urdf_limits.py"
    spec = importlib.util.spec_from_file_location("sync_urdf_limits_script", script_path)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_gradient05_config_defaults_and_mapping_shape():
    cfg = Gradient05Config()

    assert cfg.default_ik_solver_backend == "numeric"
    assert cfg.default_servo_backend == "ethercat_rtcore"
    assert cfg.actuator_ids == [0, 1, 2, 3, 4, 5]
    assert cfg.logical_to_physical_map == {
        0: [0],
        1: [1],
        2: [2],
        3: [3],
        4: [4],
        5: [5],
    }
    assert cfg.actuator_encoder_counts_per_rev == [131072] * 6
    assert cfg.actuator_gear_ratios == [100.0, 100.0, 100.0, 18.0, 20.0, 10.0]
    assert cfg.actuator_position_signs == [1, 1, 1, 1, 1, 1]
    assert cfg.logical_joint_limits_rad == [
        (-6.3, 6.3),
        (-1.9, 1.9),
        (-4.2, 1.53),
        (-6.3, 6.3),
        (-6.3, 6.3),
        (-6.3, 6.3),
    ]
    assert cfg.actuator_limits_rad == cfg.logical_joint_limits_rad


def test_simulation_backend_accepts_gradient05_robot_config():
    cfg_dict = Gradient05Config().get_config_dict()
    backend = SimulationBackend(robot_config=cfg_dict)

    assert backend.num_joints == 6
    assert backend.has_gripper is False
    assert backend.get_present_actuator_ids() == {0, 1, 2, 3, 4, 5}

    commanded = [0.1, -0.2, 0.3, -0.4, 0.5, -0.6]
    backend.set_joint_positions(commanded, speed=500, acceleration=0)
    assert backend.get_joint_positions() == commanded


def test_sync_script_main_dry_run_and_write(tmp_path):
    module = _load_sync_script_module()

    urdf_path = tmp_path / "robot.urdf"
    urdf_path.write_text(
        """<?xml version="1.0"?>
<robot name="tmp">
  <joint name="joint1" type="revolute"><limit lower="-1.0" upper="1.0"/></joint>
  <joint name="joint2" type="revolute"><limit lower="-2.5" upper="2.5"/></joint>
</robot>
""",
        encoding="utf-8",
    )

    config_path = tmp_path / "config.py"
    config_path.write_text(
        """
class GradientTmpConfig:
    @property
    def logical_joint_limits_rad(self) -> list[tuple[float, float]]:
        return [
            (-9.0, 9.0),  # J1
            (-9.0, 9.0),  # J2
        ]
""".lstrip(),
        encoding="utf-8",
    )
    original = config_path.read_text(encoding="utf-8")

    dry_run_rc = module.main(
        [
            "--urdf-path",
            str(urdf_path),
            "--config-path",
            str(config_path),
            "--joint-count",
            "2",
            "--dry-run",
        ]
    )
    assert dry_run_rc == 0
    assert config_path.read_text(encoding="utf-8") == original

    write_rc = module.main(
        [
            "--urdf-path",
            str(urdf_path),
            "--config-path",
            str(config_path),
            "--joint-count",
            "2",
        ]
    )
    assert write_rc == 0
    updated = config_path.read_text(encoding="utf-8")
    assert "(-1.0, 1.0),  # J1" in updated
    assert "(-2.5, 2.5),  # J2" in updated


def test_ethercat_axis_mapping_defaults_to_direct_order(monkeypatch):
    monkeypatch.delenv("GRADIENT_RTCORE_CONTROL_JOINTS", raising=False)
    backend = EthercatRTCoreBackend(robot_config=Gradient05Config().get_config_dict())

    assert backend._resolve_axis_to_joint_map(num_axes=6, num_joints=6) == [0, 1, 2, 3, 4, 5]
    assert backend._resolve_axis_to_joint_map(num_axes=2, num_joints=6) == [0, 1]


def test_ethercat_axis_mapping_honors_env_override(monkeypatch):
    monkeypatch.setenv("GRADIENT_RTCORE_CONTROL_JOINTS", "3,4")
    backend = EthercatRTCoreBackend(robot_config=Gradient05Config().get_config_dict())

    assert backend._resolve_axis_to_joint_map(num_axes=2, num_joints=6) == [2, 3]


def test_ethercat_axis_mapping_invalid_override_falls_back(monkeypatch):
    monkeypatch.setenv("GRADIENT_RTCORE_CONTROL_JOINTS", "9,10")
    backend = EthercatRTCoreBackend(robot_config=Gradient05Config().get_config_dict())

    assert backend._resolve_axis_to_joint_map(num_axes=2, num_joints=6) == [0, 1]


def test_ethercat_backend_applies_master_offsets_to_setpoints(monkeypatch, tmp_path):
    monkeypatch.setenv("GRADIENT_JOINT_ZERO_OFFSETS_PATH", str(tmp_path / "joint_zero_offsets.json"))
    backend = EthercatRTCoreBackend(robot_config=Gradient05Config().get_config_dict())
    backend._connected = True
    backend._rt_num_axes = 2
    backend._axis_to_joint = [0, 1]
    backend._master_offsets_rad[0] = 0.25
    backend._master_offsets_rad[1] = -0.50

    captured: dict[str, object] = {"begin": None, "points": None, "commit": None}

    monkeypatch.setattr(
        backend,
        "begin_trajectory",
        lambda **kwargs: captured.__setitem__("begin", kwargs) or 123,
    )
    monkeypatch.setattr(
        backend,
        "enqueue_trajectory_points",
        lambda traj_id, points: captured.__setitem__("points", (traj_id, points)),
    )
    monkeypatch.setattr(
        backend,
        "commit_trajectory",
        lambda traj_id: captured.__setitem__("commit", traj_id),
    )
    backend.set_joint_positions([1.0, 2.0, 0.0, 0.0, 0.0, 0.0], speed=0.0, acceleration=0.0)

    assert captured["begin"] == {"expected_points": 1}
    assert captured["commit"] == 123
    traj_id, points = captured["points"]
    assert traj_id == 123
    assert len(points) == 1
    assert points[0]["positions_rad"] == pytest.approx([1.0, 2.0, 0.0, 0.0, 0.0, 0.0])
    assert points[0]["axis_mask"] == 0x3


def test_ethercat_backend_converts_feedback_using_axis_scaling(monkeypatch, tmp_path):
    monkeypatch.setenv("GRADIENT_JOINT_ZERO_OFFSETS_PATH", str(tmp_path / "joint_zero_offsets.json"))
    backend = EthercatRTCoreBackend(robot_config=Gradient05Config().get_config_dict())
    backend._axis_to_joint = [0, 1]
    backend._axis_config = _AxisConfig(
        num_axes=2,
        counts_per_unit=[100.0, 200.0] + [0.0] * 14,
        sign=[1, -1] + [0] * 14,
    )
    backend._master_offsets_rad[0] = 0.50
    backend._master_offsets_rad[1] = -0.25

    logical_positions = backend.raw_to_joint_positions({0: 50, 1: -100})

    assert logical_positions[:2] == pytest.approx([0.0, 0.75])


def test_ethercat_backend_prefers_robot_defined_axis_scaling(monkeypatch, tmp_path):
    monkeypatch.setenv("GRADIENT_JOINT_ZERO_OFFSETS_PATH", str(tmp_path / "joint_zero_offsets.json"))
    cfg = Gradient05Config().get_config_dict()
    backend = EthercatRTCoreBackend(robot_config=cfg)
    backend._axis_to_joint = [2]

    # Simulate RTCore publishing placeholder runtime scaling; robot config should win.
    backend._runtime_axis_config = _AxisConfig(
        num_axes=1,
        counts_per_unit=[100.0] + [0.0] * 15,
        sign=[1] + [0] * 15,
    )
    backend._axis_config = backend._robot_axis_config

    logical_positions = backend.raw_to_joint_positions({0: -1046180})

    expected = -1046180 / cfg["actuator_counts_per_radian"][2]
    assert logical_positions[2] == pytest.approx(expected)


def test_ethercat_backend_zero_capture_persists_joint_offsets(monkeypatch, tmp_path):
    offsets_path = tmp_path / "joint_zero_offsets.json"
    monkeypatch.setenv("GRADIENT_JOINT_ZERO_OFFSETS_PATH", str(offsets_path))
    backend = EthercatRTCoreBackend(robot_config=Gradient05Config().get_config_dict())
    backend._connected = True
    backend._axis_to_joint = [2, 3]
    backend._axis_config = _AxisConfig(
        num_axes=2,
        counts_per_unit=[100.0, 100.0] + [0.0] * 14,
        sign=[1, 1] + [0] * 14,
    )
    backend._axis_counts[0] = 150
    backend._axis_counts[1] = 25

    assert backend.set_logical_joint_current_position_as_zero(2) is True

    loaded_offsets = load_joint_zero_offsets("gradient-05", num_joints=6)
    assert loaded_offsets[2] == pytest.approx(1.5)


def test_ethercat_backend_safe_power_down_disables_axes_and_disarms(monkeypatch, tmp_path):
    monkeypatch.setenv("GRADIENT_JOINT_ZERO_OFFSETS_PATH", str(tmp_path / "joint_zero_offsets.json"))
    backend = EthercatRTCoreBackend(robot_config=Gradient05Config().get_config_dict())
    backend._connected = True
    backend._rt_num_axes = 3

    calls: list[tuple[str, object]] = []
    monkeypatch.setattr(
        backend,
        "_send_cmd_axis_disable",
        lambda axis_mask: calls.append(("disable", axis_mask)),
    )
    monkeypatch.setattr(
        backend,
        "_send_cmd_arm",
        lambda arm: calls.append(("arm", arm)),
    )

    backend._best_effort_safe_power_down()

    assert calls == [("disable", 0x7), ("arm", False)]


def test_ethercat_backend_safe_power_up_arms_sets_mode_and_enables(monkeypatch, tmp_path):
    monkeypatch.setenv("GRADIENT_JOINT_ZERO_OFFSETS_PATH", str(tmp_path / "joint_zero_offsets.json"))
    backend = EthercatRTCoreBackend(robot_config=Gradient05Config().get_config_dict())
    backend._connected = True
    backend._rt_num_axes = 3

    calls: list[tuple[str, object]] = []
    monkeypatch.setattr(
        backend,
        "_send_cmd_arm",
        lambda arm: calls.append(("arm", arm)),
    )
    monkeypatch.setattr(
        backend,
        "_send_cmd_set_mode",
        lambda axis_mask, mode: calls.append(("mode", (axis_mask, mode))),
    )
    monkeypatch.setattr(
        backend,
        "_send_cmd_axis_enable",
        lambda axis_mask: calls.append(("enable", axis_mask)),
    )

    backend._best_effort_safe_power_up()

    assert calls == [("arm", True), ("mode", (0x7, 8)), ("enable", 0x7)]


def test_ethercat_backend_defaults_to_disarmed_connect(monkeypatch, tmp_path):
    monkeypatch.setenv("GRADIENT_JOINT_ZERO_OFFSETS_PATH", str(tmp_path / "joint_zero_offsets.json"))
    monkeypatch.delenv("GRADIENT_RTCORE_AUTO_ARM", raising=False)
    backend = EthercatRTCoreBackend(robot_config=Gradient05Config().get_config_dict())
    assert backend._auto_arm is False


def test_ethercat_backend_execution_status_defaults_idle(monkeypatch, tmp_path):
    monkeypatch.setenv("GRADIENT_JOINT_ZERO_OFFSETS_PATH", str(tmp_path / "joint_zero_offsets.json"))
    backend = EthercatRTCoreBackend(robot_config=Gradient05Config().get_config_dict())

    status = backend.get_execution_status()

    assert isinstance(status, RTCoreExecutionStatus)
    assert status.active_mode_name == "idle"
    assert status.state_name == "idle"
    assert status.motion_done is True


def test_ethercat_backend_quantizes_trajectory_frequency_to_rtcore_cycle(monkeypatch, tmp_path):
    monkeypatch.setenv("GRADIENT_JOINT_ZERO_OFFSETS_PATH", str(tmp_path / "joint_zero_offsets.json"))
    backend = EthercatRTCoreBackend(robot_config=Gradient05Config().get_config_dict())
    monkeypatch.setattr(backend, "get_rtcore_cycle_ns", lambda: 1_000_000)

    timing = backend.resolve_trajectory_frequency(333)

    assert timing == {
        "requested_frequency_hz": 333,
        "effective_frequency_hz": 250,
        "cycle_ns": 1_000_000,
        "step_ns": 4_000_000,
        "cycles_per_point": 4,
    }


def test_ethercat_backend_execute_joint_trajectory_uses_quantized_timing(monkeypatch, tmp_path):
    monkeypatch.setenv("GRADIENT_JOINT_ZERO_OFFSETS_PATH", str(tmp_path / "joint_zero_offsets.json"))
    backend = EthercatRTCoreBackend(robot_config=Gradient05Config().get_config_dict())
    monkeypatch.setattr(backend, "get_rtcore_cycle_ns", lambda: 1_000_000)
    backend._rt_num_axes = 6
    backend._axis_to_joint = [0, 1, 2, 3, 4, 5]

    captured: dict[str, object] = {}
    status = RTCoreExecutionStatus(
        active_mode=2,
        active_mode_name="trajectory",
        state=4,
        state_name="completed",
        active_traj_id=55,
        current_point_index=2,
        queue_depth=0,
        queue_capacity=4096,
        last_event_code=0,
        underrun_count=0,
        stale_command=False,
        motion_done=True,
        capability_flags=0,
        active_command_seq=123,
        last_update_ns=456,
    )

    def _begin_trajectory(expected_points):
        captured["begin"] = expected_points
        return 55

    def _enqueue_trajectory_points(traj_id, points):
        captured["points"] = (traj_id, points)

    def _commit_trajectory(traj_id):
        captured["commit"] = traj_id

    def _wait_for_trajectory_complete(traj_id, *, timeout_s):
        captured["wait"] = (traj_id, timeout_s)
        return status

    monkeypatch.setattr(backend, "begin_trajectory", _begin_trajectory)
    monkeypatch.setattr(
        backend,
        "enqueue_trajectory_points",
        _enqueue_trajectory_points,
    )
    monkeypatch.setattr(backend, "commit_trajectory", _commit_trajectory)
    monkeypatch.setattr(
        backend,
        "wait_for_trajectory_complete",
        _wait_for_trajectory_complete,
    )
    joint_path = [
        [0.1, 0.2, 0.0, 0.0, 0.0, 0.0],
        [0.2, 0.3, 0.0, 0.0, 0.0, 0.0],
        [0.3, 0.4, 0.0, 0.0, 0.0, 0.0],
    ]

    result = backend.execute_joint_trajectory(
        joint_path,
        frequency=333,
    )

    assert result is status
    assert captured["begin"] == 3
    assert captured["commit"] == 55
    points_traj_id, points_payload = captured["points"]
    assert points_traj_id == 55
    assert len(points_payload) == 3
    for idx, point in enumerate(points_payload):
        assert point["positions_rad"] == joint_path[idx]
        assert point["flags"] == _TRAJ_POINTF_HAS_VELOCITY
        assert point["t_from_start_ns"] == idx * 4_000_000
        assert point["qd"] == pytest.approx([25.0, 25.0, 0.0, 0.0, 0.0, 0.0])
    wait_traj_id, wait_timeout_s = captured["wait"]
    assert wait_traj_id == 55
    assert wait_timeout_s == pytest.approx(5.012)
    assert backend.get_last_trajectory_timing() == {
        "requested_frequency_hz": 333,
        "effective_frequency_hz": 250,
        "cycle_ns": 1_000_000,
        "step_ns": 4_000_000,
        "cycles_per_point": 4,
    }


def test_ethercat_backend_parses_motion_state_status(monkeypatch, tmp_path):
    monkeypatch.setenv("GRADIENT_JOINT_ZERO_OFFSETS_PATH", str(tmp_path / "joint_zero_offsets.json"))
    backend = EthercatRTCoreBackend(robot_config=Gradient05Config().get_config_dict())

    payload = struct.pack(
        "<IIQIIIIIIIIQQ",
        1,   # legacy_setpoint
        4,   # completed
        0,
        0xFFFFFFFF,
        0,
        0,
        0x0123,
        0,
        0,
        1,
        0x1,
        42,
        123456789,
    )

    status = backend._parse_motion_state(payload)

    assert status.active_mode_name == "legacy_setpoint"
    assert status.state_name == "completed"
    assert status.current_point_index is None
    assert status.motion_done is True
    assert status.capability_flags == 0x1
    assert status.active_command_seq == 42


def test_ethercat_backend_wait_for_trajectory_complete_ignores_stale_previous_completion(monkeypatch, tmp_path):
    monkeypatch.setenv("GRADIENT_JOINT_ZERO_OFFSETS_PATH", str(tmp_path / "joint_zero_offsets.json"))
    backend = EthercatRTCoreBackend(robot_config=Gradient05Config().get_config_dict())

    statuses = iter(
        [
            RTCoreExecutionStatus(
                active_mode=2,
                active_mode_name="trajectory",
                state=RTCORE_EXEC_STATE_COMPLETED,
                state_name="completed",
                active_traj_id=1,
                current_point_index=401,
                queue_depth=0,
                queue_capacity=4096,
                last_event_code=291,
                underrun_count=0,
                stale_command=False,
                motion_done=True,
                capability_flags=0,
                active_command_seq=100,
                last_update_ns=1000,
            ),
            RTCoreExecutionStatus(
                active_mode=2,
                active_mode_name="trajectory",
                state=3,
                state_name="executing",
                active_traj_id=2,
                current_point_index=5,
                queue_depth=10,
                queue_capacity=4096,
                last_event_code=290,
                underrun_count=0,
                stale_command=False,
                motion_done=False,
                capability_flags=0,
                active_command_seq=101,
                last_update_ns=1100,
            ),
            RTCoreExecutionStatus(
                active_mode=2,
                active_mode_name="trajectory",
                state=RTCORE_EXEC_STATE_COMPLETED,
                state_name="completed",
                active_traj_id=2,
                current_point_index=401,
                queue_depth=0,
                queue_capacity=4096,
                last_event_code=291,
                underrun_count=0,
                stale_command=False,
                motion_done=True,
                capability_flags=0,
                active_command_seq=101,
                last_update_ns=1200,
            ),
        ]
    )

    monkeypatch.setattr(backend, "get_execution_status", lambda: next(statuses))
    monkeypatch.setattr(time, "sleep", lambda _seconds: None)

    result = backend.wait_for_trajectory_complete(2, timeout_s=0.1)

    assert result.active_traj_id == 2
    assert result.state_name == "completed"


def test_ethercat_backend_sends_realtime_jog_command(monkeypatch, tmp_path):
    monkeypatch.setenv("GRADIENT_JOINT_ZERO_OFFSETS_PATH", str(tmp_path / "joint_zero_offsets.json"))
    backend = EthercatRTCoreBackend(robot_config=Gradient05Config().get_config_dict())
    backend._connected = True
    backend._initialized = True
    backend._rt_num_axes = 3
    backend._axis_to_joint = [0, 1, 2]

    captured: list[tuple[int, bytes]] = []
    monkeypatch.setattr(backend, "_cmd_ring_write", lambda msg_type, payload: captured.append((msg_type, payload)))

    backend.send_realtime_jog_command(
        [0.1, -0.2, 0.3, 0.0, 0.0, 0.0],
        timeout_s=0.5,
    )

    assert len(captured) == 1
    msg_type, payload = captured[0]
    assert msg_type == 0x0130
    axis_mask, flags, timeout_ns, *qd = struct.unpack("<IIQ16d", payload)
    assert axis_mask == 0x7
    assert flags == 0x1
    assert timeout_ns == 500_000_000
    assert qd[:3] == pytest.approx([0.1, -0.2, 0.3])


def test_ethercat_backend_stop_realtime_jog_sends_stop_flag(monkeypatch, tmp_path):
    monkeypatch.setenv("GRADIENT_JOINT_ZERO_OFFSETS_PATH", str(tmp_path / "joint_zero_offsets.json"))
    backend = EthercatRTCoreBackend(robot_config=Gradient05Config().get_config_dict())
    backend._connected = True
    backend._rt_num_axes = 2

    captured: list[tuple[int, bytes]] = []
    monkeypatch.setattr(backend, "_cmd_ring_write", lambda msg_type, payload: captured.append((msg_type, payload)))

    backend.stop_realtime_jog()

    assert len(captured) == 1
    msg_type, payload = captured[0]
    assert msg_type == 0x0130
    axis_mask, flags, timeout_ns, *qd = struct.unpack("<IIQ16d", payload)
    assert axis_mask == 0
    assert flags == 0x2
    assert timeout_ns == 0
    assert qd[:2] == pytest.approx([0.0, 0.0])


def test_registry_telemetry_blocks_are_optional_for_ethercat_backend():
    backend_registry.set_active_backend("feetech")
    assert len(backend_registry.get_telemetry_blocks()) == 3

    backend_registry.set_active_backend("ethercat_rtcore")
    assert backend_registry.get_telemetry_blocks() == []
    assert backend_registry.parse_telemetry_block(0, b"\x00\x01") == {}

    backend_registry.set_active_backend("feetech")


def test_rtcore_metrics_ready_requires_full_startup_signal():
    ready, detail = _rtcore_metrics_ready(
        {
            "link_up": 1,
            "responding_slaves": 6,
            "online_slaves": 6,
            "operational_slaves": 5,
            "startup_ready": 0,
            "wkc_actual": 12,
            "wkc_expected": 12,
        },
        expected_axes=6,
    )

    assert ready is False
    assert detail == "operational=5/6"


def test_rtcore_metrics_ready_accepts_live_operational_bus():
    ready, detail = _rtcore_metrics_ready(
        {
            "link_up": 1,
            "responding_slaves": 6,
            "online_slaves": 6,
            "operational_slaves": 6,
            "startup_ready": 1,
            "wkc_actual": 18,
            "wkc_expected": 12,
        },
        expected_axes=6,
    )

    assert ready is True
    assert detail == "startup_ready=1 operational=6/6 wkc=18/12"


def test_ik_solver_configure_falls_back_to_ikfast_when_numeric_is_unavailable(monkeypatch):
    calls: list[str] = []

    def _fake_numeric_backend():
        calls.append("numeric")
        return False

    def _fake_ikfast_backend():
        calls.append("ikfast")
        return True

    monkeypatch.setattr(ik_solver, "_init_numeric_backend", _fake_numeric_backend)
    monkeypatch.setattr(ik_solver, "_init_ikfast_backend", _fake_ikfast_backend)

    result = ik_solver.configure(robot_id=ik_solver.get_robot_id(), backend_name="numeric")

    assert result["backend_name"] == "ikfast"
    assert calls == ["numeric", "ikfast"]

