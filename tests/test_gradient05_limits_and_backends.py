from __future__ import annotations

import importlib.util
from pathlib import Path

import pytest

from gradient_os.arm_controller.backends import registry as backend_registry
from gradient_os.arm_controller.backends.ethercat_rtcore.backend import (
    EthercatRTCoreBackend,
    _AxisConfig,
)
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

    captured: dict[str, object] = {}

    def _capture_write(positions_rad: list[float], axis_mask: int) -> None:
        captured["positions_rad"] = list(positions_rad)
        captured["axis_mask"] = axis_mask

    backend._write_setpoint = _capture_write  # type: ignore[method-assign]
    backend.set_joint_positions([1.0, 2.0, 0.0, 0.0, 0.0, 0.0], speed=0.0, acceleration=0.0)

    assert captured["positions_rad"] == pytest.approx([1.25, 1.50])
    assert captured["axis_mask"] == 0x3


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

