from __future__ import annotations

import importlib.util
import json
import math
import struct
import threading
import time
from fractions import Fraction
from pathlib import Path

import pytest

from gradient_os.arm_controller.backends.ethercat_rtcore import backend as rtcore_backend_module
from gradient_os.arm_controller.backends import registry as backend_registry
from gradient_os.arm_controller.backends.ethercat_rtcore.backend import (
    EthercatRTCoreBackend,
    RTCoreExecutionStatus,
    RTCoreJogDebugStatus,
    _AbsoluteFeedbackAxisMetrics,
    _MAGIC_RING,
    _MSG_HEADER_STRUCT,
    _RING_HEADER_STRUCT,
    _TRAJ_POINTF_HAS_VELOCITY,
    _TRAJECTORY_POINT_STRUCT,
    _ShmHeader,
    _AxisConfig,
)
from gradient_os.arm_controller.backends.ethercat_rtcore.runtime import (
    RTCORE_EXEC_STATE_COMPLETED,
    RTCORE_EXEC_STATE_IDLE,
)
from gradient_os.arm_controller.backends.simulation import backend as sim_backend_module
from gradient_os.arm_controller.backends.simulation.backend import SimulationBackend
from gradient_os.arm_controller.robots.gradient05.config import Gradient05Config
from gradient_os import ik_solver
from gradient_os.absolute_encoder_anchors import load_absolute_encoder_anchors, save_absolute_encoder_anchor
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


def _write_rtcore_metrics_snapshot(path: Path, axes: list[dict[str, object]]) -> None:
    path.write_text(
        json.dumps(
            {
                "time_ns": 123456789,
                "axes": axes,
            },
            separators=(",", ":"),
        ),
        encoding="utf-8",
    )


def _force_legacy_truth_fallback(backend: EthercatRTCoreBackend) -> EthercatRTCoreBackend:
    backend._drive_native_ratio_enabled = lambda profile_id=None: False
    backend._legacy_truth_fallback_enabled = lambda profile_id=None: True
    backend._absolute_home_anchor_required = lambda profile_id=None: True
    return backend


def _a6ec_startup_drive_configs_for_ratio(
    raw_ratio: object,
    *,
    mode_value: int = 4,
    reference_direction: int = 0,
) -> list[dict[str, int | str]]:
    ratio = Fraction(str(raw_ratio))
    numerator = int(ratio.numerator)
    denominator = int(ratio.denominator)
    return [
        {
            "setting_key": "a6ec_encoder_position_tracking_mode",
            "configured": 1,
            "commanded": mode_value,
            "readback_valid": 1,
            "readback": mode_value,
            "verified": 1,
        },
        {
            "setting_key": "a6ec_rotation_mode_gear_ratio_numerator",
            "configured": 1,
            "commanded": numerator,
            "readback_valid": 1,
            "readback": numerator,
            "verified": 1,
        },
        {
            "setting_key": "a6ec_rotation_mode_gear_ratio_denominator",
            "configured": 1,
            "commanded": denominator,
            "readback_valid": 1,
            "readback": denominator,
            "verified": 1,
        },
        {
            "setting_key": "a6ec_rotation_mode_reference_running_direction",
            "configured": 1,
            "commanded": reference_direction,
            "readback_valid": 1,
            "readback": reference_direction,
            "verified": 1,
        },
    ]


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
    assert cfg.actuator_gear_ratios == [100.0, 100.0, 100.0, 18.0, 31.25, 10.0]
    assert cfg.actuator_position_signs == [-1, 1, -1, -1, -1, -1]
    assert cfg.logical_joint_limits_rad == [
        (-6.3, 6.3),
        (-1.9, 1.9),
        (-4.2, 1.53),
        (-6.3, 6.3),
        (-6.3, 6.3),
        (-10.0, 10.0),
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


def test_simulation_backend_sync_write_round_trips_logical_positions():
    cfg_dict = Gradient05Config().get_config_dict()
    backend = SimulationBackend(robot_config=cfg_dict)

    commanded = [0.15, -0.35, 0.45, -0.25, 0.55, -0.65]
    commands = backend.prepare_sync_write_commands(commanded, speed=500, accel=0)

    backend.sync_write(commands)

    actual = backend.get_joint_positions()
    assert actual == pytest.approx(commanded, abs=2e-3)


def test_simulation_backend_joint_velocity_lease_jog_advances_and_expires(monkeypatch):
    cfg_dict = Gradient05Config().get_config_dict()
    backend = SimulationBackend(robot_config=cfg_dict)
    clock = {"now": 100.0}

    monkeypatch.setattr(sim_backend_module.time, "monotonic", lambda: clock["now"])

    backend.start_joint_velocity_lease_jog(timeout_s=0.2)
    backend.update_joint_velocity_lease_jog([0.5, 0.0, 0.0, 0.0, 0.0, 0.0], timeout_s=0.2)

    clock["now"] = 100.1
    assert backend.get_joint_positions()[0] == pytest.approx(0.05, abs=2e-3)

    backend.update_joint_velocity_lease_jog([1.0, 0.0, 0.0, 0.0, 0.0, 0.0], timeout_s=0.2)

    clock["now"] = 100.2
    assert backend.get_joint_positions()[0] == pytest.approx(0.15, abs=2e-3)

    clock["now"] = 100.35
    assert backend.get_joint_positions()[0] == pytest.approx(0.25, abs=2e-3)

    clock["now"] = 100.6
    held_position = backend.get_joint_positions()[0]
    assert held_position == pytest.approx(0.25, abs=2e-3)

    clock["now"] = 101.0
    assert backend.get_joint_positions()[0] == pytest.approx(held_position, abs=2e-3)


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


def test_tx_pdo_layout_fits_a6ec_sm3_capacity_and_preserves_classic_entries():
    """Phase 1 (2026-04-20 post-revert) regression. On 2026-04-20 bring-up
    the A6-EC drives accepted our 18-entry extended TxPDO mapping via
    SDO but could not transmit the 47-byte frame; `ethercat data`
    showed zeros across every TxPDO byte while the exact same SDO
    upload returned the correct statusword. The extended entries
    have been removed until we can land them via a second TxPDO slot
    or an explicit 0x1C13 sync-manager re-assign. This test pins
    both halves of the invariant: the classic 9 entries must stay,
    and the total layout must fit the observed SM3 capacity (~28
    bytes) so nobody accidentally re-blanks the frame."""
    from gradient_os.arm_controller.ethercat_drive_catalog import ETHERCAT_DRIVE_CATALOG

    layout = ETHERCAT_DRIVE_CATALOG["a6ec_ds402"]["rtcore"]["tx_pdo_layout"]
    by_semantic = {entry["semantic"]: entry for entry in layout}
    # Phase 1 final (2026-04-21): after two live probes (47 B and 33 B
    # both silently blanked the A6-EC TxPDO frame), characterising the
    # drive revealed the real issue isn't SM3 capacity per se — it's
    # that U40.20/U40.22 are declared PDO-mappable in the ESI but the
    # firmware does NOT actually populate the bytes cyclically. The
    # drive accepts the PDO assignment but sends zeros for those
    # subitems while 0x6064 in the same frame populates correctly.
    #
    # Phase 1 now delivers atomic multi-turn via a different path:
    # RTCore latches 0x6064 at the moment of the SDO absolute-feedback
    # upload and publishes it as `paired_pos_counts` inside the
    # absolute_feedback JSON payload. The Python shaft-frame gate
    # reads that paired value instead of live-now 0x6064, so the two
    # values it compares come from the same moment (bounded at
    # mailbox transit ~1-5 ms) instead of the 200 ms SDO-poll skew.
    #
    # The TxPDO stays at 6 essential entries (17 bytes): err, sw, pos,
    # torque, mode_disp, di. Touch-probe feedback entries (tp_status/
    # tp_pos1/tp_pos2) were dropped because RTCore never read them
    # from process data — only `tp_func` on the RxPDO side is
    # exercised and it stays in place.
    required_entries = {
        ("err", 0x603F, 0x00, 16),
        ("sw", 0x6041, 0x00, 16),
        ("pos", 0x6064, 0x00, 32),
        ("torque", 0x6077, 0x00, 16),
        ("mode_disp", 0x6061, 0x00, 8),
        ("di", 0x60FD, 0x00, 32),
    }
    for semantic, index, subindex, bits in required_entries:
        entry = by_semantic.get(semantic)
        assert entry is not None, f"missing tx_pdo entry: {semantic}"
        assert int(entry["index"]) == index, f"wrong index for {semantic}"
        assert int(entry["subindex"]) == subindex, f"wrong subindex for {semantic}"
        assert int(entry["bits"]) == bits, f"wrong bits for {semantic}"

    # Regression guards: entries deliberately kept OUT of the TxPDO.
    assert "multi_turn_lo" not in by_semantic, (
        "multi_turn_lo removed 2026-04-21 — A6-EC firmware does not "
        "populate U40.20 in custom TxPDO despite ESI claiming it is "
        "PDO-mappable. Canonical-truth atomicity is now delivered by "
        "the `paired_pos_counts` snapshot in the absolute_feedback JSON."
    )
    assert "multi_turn_hi" not in by_semantic, "multi_turn_hi removed 2026-04-21"
    assert "tp_status" not in by_semantic, (
        "tp_status removed 2026-04-21 — RTCore never read it from process data"
    )
    assert "tp_pos1" not in by_semantic, "tp_pos1 removed 2026-04-21"
    assert "tp_pos2" not in by_semantic, "tp_pos2 removed 2026-04-21"

    # Guardrail: A6-EC live bring-up (2026-04-20 / 2026-04-21) showed
    # the firmware silently blanks the TxPDO frame when the custom
    # mapping pushes past the drive's tolerance. The 6-entry / 17-byte
    # layout is the known-safe steady state; anything larger must be
    # accompanied by live-hardware verification that `statusword`
    # continues to read non-zero via PDO on every axis. The 28 B cap
    # below bakes in a safety margin vs the 33 B probe that broke it.
    total_bits = sum(int(entry["bits"]) for entry in layout)
    assert total_bits <= 28 * 8, (
        f"tx_pdo_layout total {total_bits} bits exceeds the A6-EC SM3 "
        f"known-safe capacity (28 B). A 33 B mapping silently blanked "
        f"the TxPDO frame on hardware. See 2026-04-21 scratchpad."
    )


def test_ethercat_axis_mapping_honors_env_override(monkeypatch):
    monkeypatch.setenv("GRADIENT_RTCORE_CONTROL_JOINTS", "3,4")
    backend = _force_legacy_truth_fallback(
        EthercatRTCoreBackend(robot_config=Gradient05Config().get_config_dict())
    )

    assert backend._resolve_axis_to_joint_map(num_axes=2, num_joints=6) == [2, 3]


def test_ethercat_axis_mapping_invalid_override_falls_back(monkeypatch):
    monkeypatch.setenv("GRADIENT_RTCORE_CONTROL_JOINTS", "9,10")
    backend = _force_legacy_truth_fallback(
        EthercatRTCoreBackend(robot_config=Gradient05Config().get_config_dict())
    )

    assert backend._resolve_axis_to_joint_map(num_axes=2, num_joints=6) == [0, 1]


def test_ethercat_backend_applies_master_offsets_to_setpoints(monkeypatch, tmp_path):
    monkeypatch.setenv("GRADIENT_JOINT_ZERO_OFFSETS_PATH", str(tmp_path / "joint_zero_offsets.json"))
    backend = _force_legacy_truth_fallback(
        EthercatRTCoreBackend(robot_config=Gradient05Config().get_config_dict())
    )
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
    assert "qd" not in points[0]
    assert "flags" not in points[0]
    assert points[0]["axis_mask"] == 0x3


def test_ethercat_backend_sync_write_ignores_legacy_speed_and_accel(monkeypatch, tmp_path):
    monkeypatch.setenv("GRADIENT_JOINT_ZERO_OFFSETS_PATH", str(tmp_path / "joint_zero_offsets.json"))
    backend = _force_legacy_truth_fallback(
        EthercatRTCoreBackend(robot_config=Gradient05Config().get_config_dict())
    )

    commands = backend.prepare_sync_write_commands([1.0, 0.0, 0.0, 0.0, 0.0, 0.0], speed=321, accel=7)
    captured: dict[str, object] = {}
    monkeypatch.setattr(
        backend,
        "set_joint_positions",
        lambda positions_rad, speed, acceleration: captured.update(
            {
                "positions_rad": positions_rad,
                "speed": speed,
                "acceleration": acceleration,
            }
        ),
    )

    backend.sync_write(commands)

    assert captured == {
        "positions_rad": [1.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        "speed": 0.0,
        "acceleration": 0.0,
    }


def test_ethercat_backend_single_actuator_setpoint_emits_position_only_trajectory(monkeypatch, tmp_path):
    monkeypatch.setenv("GRADIENT_JOINT_ZERO_OFFSETS_PATH", str(tmp_path / "joint_zero_offsets.json"))
    backend = _force_legacy_truth_fallback(
        EthercatRTCoreBackend(robot_config=Gradient05Config().get_config_dict())
    )
    backend._connected = True
    backend._rt_num_axes = 2
    backend._axis_to_joint = [0, 1]

    captured: dict[str, object] = {"begin": None, "points": None, "commit": None}

    monkeypatch.setattr(
        backend,
        "begin_trajectory",
        lambda **kwargs: captured.__setitem__("begin", kwargs) or 456,
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

    backend.set_single_actuator_position(1, 0.5, speed=640, accel=0)

    assert captured["begin"] == {"axis_mask": 0x2, "expected_points": 1}
    assert captured["commit"] == 456
    traj_id, points = captured["points"]
    assert traj_id == 456
    assert len(points) == 1
    assert points[0]["axis_q"] == pytest.approx([0.0, 0.5])
    assert "qd" not in points[0]
    assert "flags" not in points[0]
    assert points[0]["axis_mask"] == 0x2
    assert backend._last_joint_setpoint_rad[1] == pytest.approx(0.5)


def test_ethercat_backend_set_joint_positions_does_not_advance_cache_on_commit_failure(monkeypatch, tmp_path):
    monkeypatch.setenv("GRADIENT_JOINT_ZERO_OFFSETS_PATH", str(tmp_path / "joint_zero_offsets.json"))
    backend = _force_legacy_truth_fallback(
        EthercatRTCoreBackend(robot_config=Gradient05Config().get_config_dict())
    )
    backend._connected = True
    backend._rt_num_axes = 2
    backend._axis_to_joint = [0, 1]
    backend._last_joint_setpoint_rad = [0.2, -0.3, 0.0, 0.0, 0.0, 0.0]

    monkeypatch.setattr(backend, "begin_trajectory", lambda **kwargs: 999)
    monkeypatch.setattr(backend, "enqueue_trajectory_points", lambda traj_id, points: None)
    monkeypatch.setattr(backend, "commit_trajectory", lambda traj_id: (_ for _ in ()).throw(RuntimeError("commit failed")))

    with pytest.raises(RuntimeError, match="commit failed"):
        backend.set_joint_positions([0.8, 0.1, 0.0, 0.0, 0.0, 0.0], speed=500.0, acceleration=0.0)

    assert backend._last_joint_setpoint_rad[:2] == pytest.approx([0.2, -0.3])


def test_ethercat_backend_converts_feedback_using_axis_scaling(monkeypatch, tmp_path):
    monkeypatch.setenv("GRADIENT_JOINT_ZERO_OFFSETS_PATH", str(tmp_path / "joint_zero_offsets.json"))
    metrics_path = tmp_path / "metrics.json"
    monkeypatch.setattr(rtcore_backend_module, "_RTCORE_METRICS_PATH", metrics_path)
    _write_rtcore_metrics_snapshot(
        metrics_path,
        [
            {
                "absolute_feedback": {
                    "encoder_multi_turn_low": {"valid": 1, "value": 50},
                    "encoder_multi_turn_high": {"valid": 1, "value": 0},
                },
            },
            {
                "absolute_feedback": {
                    "encoder_multi_turn_low": {"valid": 1, "value": -100},
                    "encoder_multi_turn_high": {"valid": 1, "value": -1},
                },
            },
        ],
    )
    backend = _force_legacy_truth_fallback(
        EthercatRTCoreBackend(robot_config=Gradient05Config().get_config_dict())
    )
    backend._axis_to_joint = [0, 1]
    backend._axis_config = _AxisConfig(
        num_axes=2,
        counts_per_unit=[100.0, 200.0] + [0.0] * 14,
        sign=[1, -1] + [0] * 14,
    )
    backend._rt_drive_profile_id = "a6ec_ds402"
    backend._master_offsets_rad[0] = 0.50
    backend._master_offsets_rad[1] = -0.25
    backend._absolute_encoder_home_anchors[0] = {"home_anchor_rad": 0.0, "source": "pytest", "axis_indices": [0]}
    backend._absolute_encoder_home_anchors[1] = {"home_anchor_rad": 0.0, "source": "pytest", "axis_indices": [1]}

    logical_positions = backend.raw_to_joint_positions({0: 50, 1: -100})

    assert logical_positions[:2] == pytest.approx([0.0, 0.75])


def test_ethercat_backend_uses_multi_turn_absolute_feedback_as_canonical_truth(monkeypatch, tmp_path):
    monkeypatch.setenv("GRADIENT_JOINT_ZERO_OFFSETS_PATH", str(tmp_path / "joint_zero_offsets.json"))
    metrics_path = tmp_path / "metrics.json"
    monkeypatch.setattr(rtcore_backend_module, "_RTCORE_METRICS_PATH", metrics_path)
    _write_rtcore_metrics_snapshot(
        metrics_path,
        [
            {
                "absolute_feedback": {
                    "encoder_multi_turn_low": {"valid": 1, "value": 131060},
                    "encoder_multi_turn_high": {"valid": 1, "value": 0},
                },
            }
        ],
    )
    backend = _force_legacy_truth_fallback(
        EthercatRTCoreBackend(robot_config=Gradient05Config().get_config_dict())
    )
    backend._axis_to_joint = [0]
    backend._axis_config = _AxisConfig(
        num_axes=1,
        counts_per_unit=[100.0] + [0.0] * 15,
        sign=[1] + [0] * 15,
        counts_per_rev=[131072] + [0] * 15,
    )
    backend._rt_num_axes = 1
    backend._rt_drive_profile_id = "a6ec_ds402"
    backend._absolute_encoder_home_anchors[0] = {"home_anchor_rad": 0.0, "source": "pytest", "axis_indices": [0]}

    logical_positions = backend.raw_to_joint_positions({0: 131060})
    assert logical_positions[0] == pytest.approx(1310.6)


def test_ethercat_backend_keeps_raw_truth_across_single_turn_wrap_even_when_display_truth_fails(
    monkeypatch, tmp_path
):
    monkeypatch.setenv("GRADIENT_JOINT_ZERO_OFFSETS_PATH", str(tmp_path / "joint_zero_offsets.json"))
    metrics_path = tmp_path / "metrics.json"
    monkeypatch.setattr(rtcore_backend_module, "_RTCORE_METRICS_PATH", metrics_path)
    backend = _force_legacy_truth_fallback(
        EthercatRTCoreBackend(robot_config=Gradient05Config().get_config_dict())
    )
    backend._axis_to_joint = [0]
    backend._axis_config = _AxisConfig(
        num_axes=1,
        counts_per_unit=[100.0] + [0.0] * 15,
        sign=[1] + [0] * 15,
        counts_per_rev=[131072] + [0] * 15,
    )
    backend._rt_num_axes = 1
    backend._rt_drive_profile_id = "a6ec_ds402"
    backend._absolute_encoder_home_anchors[0] = {"home_anchor_rad": 0.0, "source": "pytest", "axis_indices": [0]}

    _write_rtcore_metrics_snapshot(
        metrics_path,
        [
            {
                "absolute_feedback": {
                    "encoder_multi_turn_low": {"valid": 1, "value": 131060},
                    "encoder_multi_turn_high": {"valid": 1, "value": 0},
                },
            }
        ],
    )
    first = backend.raw_to_joint_positions({0: 131060})
    _write_rtcore_metrics_snapshot(
        metrics_path,
        [
            {
                "absolute_feedback": {
                    "encoder_multi_turn_low": {"valid": 1, "value": 131092},
                    "encoder_multi_turn_high": {"valid": 1, "value": 0},
                },
            }
        ],
    )
    backend._native_home_metrics_mtime_ns = 0
    snapshot = backend.get_display_feedback_snapshot({0: 20})

    assert first[0] == pytest.approx(1310.60)
    assert isinstance(snapshot, dict)
    assert snapshot["truth_available"] is False
    axis_detail = snapshot["axis_absolute_feedback"][0]
    # Workstream 3 primary gate names this condition explicitly; the older
    # continuous-frame roundtrip would have surfaced it as
    # command_frame_roundtrip_mismatch.
    assert axis_detail["truth_reason"] in {
        "multi_turn_anchor_inconsistent_with_live_6064",
        "command_frame_roundtrip_mismatch",
        "absolute_home_anchor_stale",
    }
    wrapped_positions = backend.raw_to_joint_positions({0: 20})
    commanded_axis_q = backend._axis_q_from_joint_positions(wrapped_positions)

    assert wrapped_positions[0] == pytest.approx(1310.92)
    # Modular comparison: under the 2026-04-19 multi-turn-aware fold,
    # seam-adjacent positions emit wire values that may differ from the
    # single-turn-wrapped 0x6064 by an integer multiple of RM. Live
    # motor counts = 131092 is within 20 counts of RM = 131072, which
    # trips the seam-band disambiguation; the fold correctly emits a
    # continuous target at the multi-turn position (131092) rather than
    # the wrapped value (20). The drive sees them as equivalent
    # modularly.
    rm = 131072
    wire_counts = int(round(commanded_axis_q[0] * 100.0))
    modular_delta = (wire_counts - 20) % rm
    if modular_delta > rm // 2:
        modular_delta -= rm
    assert abs(modular_delta) <= 1, (
        f"wire_counts={wire_counts} live_6064=20 modular_delta={modular_delta} rm={rm}"
    )


def test_ethercat_backend_uses_drive_native_truth_when_startup_and_status_are_valid(
    monkeypatch, tmp_path
):
    monkeypatch.setenv("GRADIENT_JOINT_ZERO_OFFSETS_PATH", str(tmp_path / "joint_zero_offsets.json"))
    monkeypatch.setenv(
        "GRADIENT_ABSOLUTE_ENCODER_ANCHORS_PATH",
        str(tmp_path / "absolute_encoder_anchors.json"),
    )
    metrics_path = tmp_path / "metrics.json"
    monkeypatch.setattr(rtcore_backend_module, "_RTCORE_METRICS_PATH", metrics_path)
    robot_cfg = Gradient05Config().get_config_dict()
    _write_rtcore_metrics_snapshot(
        metrics_path,
        [
            {
                "statusword": 0x9650,
                "error_code": 0,
                "manufacturer_error_code": 0,
                "startup_drive_config": _a6ec_startup_drive_configs_for_ratio(
                    robot_cfg["actuator_gear_ratios"][0]
                )[0],
                "startup_drive_configs": _a6ec_startup_drive_configs_for_ratio(
                    robot_cfg["actuator_gear_ratios"][0]
                ),
                "absolute_feedback": {
                    "encoder_multi_turn_low": {"valid": 1, "value": 250},
                    "encoder_multi_turn_high": {"valid": 1, "value": 0},
                },
            }
        ],
    )
    backend = EthercatRTCoreBackend(robot_config=robot_cfg)
    backend._axis_to_joint = [0]
    backend._axis_config = _AxisConfig(
        num_axes=1,
        counts_per_unit=[100.0] + [0.0] * 15,
        sign=[1] + [0] * 15,
        counts_per_rev=[131072] + [0] * 15,
    )
    backend._rt_num_axes = 1
    backend._rt_drive_profile_id = "a6ec_ds402"
    backend._absolute_encoder_home_anchors[0] = {
        "home_anchor_rad": 0.0,
        "source": "pytest",
        "axis_indices": [0],
    }

    logical_positions = backend.raw_to_joint_positions({0: 250})
    snapshot = backend.get_display_feedback_snapshot({0: 250})

    assert logical_positions[0] == pytest.approx(2.5)
    assert isinstance(snapshot, dict)
    assert snapshot["truth_available"] is True
    assert snapshot["drive_native_ratio_enabled"] is True
    assert snapshot["drive_native_startup_valid"] is True
    assert snapshot["drive_native_truth_available"] is True
    assert snapshot["position_semantics_source"] == "drive_output_shaft"
    axis_detail = snapshot["axis_absolute_feedback"][0]
    assert axis_detail["truth_source"] == "drive_output_shaft"
    assert axis_detail["display_source"] == "drive_output_shaft"
    assert axis_detail["configured_canonical_truth_source"] == "encoder_multi_turn_counts"
    assert axis_detail["canonical_truth_source"] == "encoder_multi_turn_counts"
    assert axis_detail["canonical_truth_counts_source"] == "encoder_multi_turn_counts"
    assert axis_detail["absolute_counts"] == 250
    assert axis_detail["drive_native_startup_valid"] is True
    assert axis_detail["drive_native_truth_valid"] is True


def test_ethercat_backend_drive_native_truth_unwraps_public_joint_positions_but_preserves_raw_write_frame(
    monkeypatch, tmp_path
):
    monkeypatch.setenv("GRADIENT_JOINT_ZERO_OFFSETS_PATH", str(tmp_path / "joint_zero_offsets.json"))
    monkeypatch.setenv(
        "GRADIENT_ABSOLUTE_ENCODER_ANCHORS_PATH",
        str(tmp_path / "absolute_encoder_anchors.json"),
    )
    metrics_path = tmp_path / "metrics.json"
    monkeypatch.setattr(rtcore_backend_module, "_RTCORE_METRICS_PATH", metrics_path)
    robot_cfg = Gradient05Config().get_config_dict()
    _write_rtcore_metrics_snapshot(
        metrics_path,
        [
            {
                "statusword": 0x9650,
                "error_code": 0,
                "manufacturer_error_code": 0,
                "startup_drive_config": _a6ec_startup_drive_configs_for_ratio(
                    robot_cfg["actuator_gear_ratios"][0]
                )[0],
                "startup_drive_configs": _a6ec_startup_drive_configs_for_ratio(
                    robot_cfg["actuator_gear_ratios"][0]
                ),
                "absolute_feedback": {
                    "encoder_multi_turn_low": {"valid": 1, "value": 0},
                    "encoder_multi_turn_high": {"valid": 1, "value": 0},
                },
            }
        ],
    )
    backend = EthercatRTCoreBackend(robot_config=robot_cfg)
    backend._connected = True
    backend._axis_to_joint = [0]
    backend._axis_config = _AxisConfig(
        num_axes=1,
        counts_per_unit=[100.0] + [0.0] * 15,
        sign=[1] + [0] * 15,
        counts_per_rev=[628] + [0] * 15,
    )
    backend._rt_num_axes = 1
    backend._rt_drive_profile_id = "a6ec_ds402"
    backend._axis_counts[0] = 620
    backend._absolute_encoder_home_anchors[0] = {
        "home_anchor_rad": 0.08,
        "source": "pytest",
        "axis_indices": [0],
    }

    logical_positions = backend.raw_to_joint_positions({0: 620})
    live_positions = backend.get_joint_positions()
    commanded_axis_q = backend._axis_q_from_joint_positions(logical_positions)

    assert logical_positions[0] == pytest.approx(-0.08)
    assert live_positions[0] == pytest.approx(logical_positions[0])
    # Wire-space 607A target must land on live 6064 MODULO RM. Under the
    # 2026-04-19 multi-turn-aware fold, seam-adjacent positions (here,
    # live_6064=620 is within 8 counts of RM=628) use multi-turn to
    # disambiguate and may emit a target an integer multiple of RM away
    # from live_6064. Physical motion is identical (drive does modular
    # comparison per Chapter 5 Fig 5-1).
    rm = 628
    wire_counts = int(round(commanded_axis_q[0] * 100.0))
    modular_delta = (wire_counts - 620) % rm
    if modular_delta > rm // 2:
        modular_delta -= rm
    assert abs(modular_delta) <= 1, (
        f"wire_counts={wire_counts} live_6064=620 modular_delta={modular_delta} rm={rm}"
    )


def test_ethercat_backend_drive_native_truth_requires_absolute_home_anchor(monkeypatch, tmp_path):
    monkeypatch.setenv("GRADIENT_JOINT_ZERO_OFFSETS_PATH", str(tmp_path / "joint_zero_offsets.json"))
    monkeypatch.setenv(
        "GRADIENT_ABSOLUTE_ENCODER_ANCHORS_PATH",
        str(tmp_path / "absolute_encoder_anchors.json"),
    )
    metrics_path = tmp_path / "metrics.json"
    monkeypatch.setattr(rtcore_backend_module, "_RTCORE_METRICS_PATH", metrics_path)
    robot_cfg = Gradient05Config().get_config_dict()
    _write_rtcore_metrics_snapshot(
        metrics_path,
        [
            {
                "statusword": 0x9650,
                "error_code": 0,
                "manufacturer_error_code": 0,
                "startup_drive_config": _a6ec_startup_drive_configs_for_ratio(
                    robot_cfg["actuator_gear_ratios"][0]
                )[0],
                "startup_drive_configs": _a6ec_startup_drive_configs_for_ratio(
                    robot_cfg["actuator_gear_ratios"][0]
                ),
                "absolute_feedback": {
                    "encoder_multi_turn_low": {"valid": 1, "value": 250},
                    "encoder_multi_turn_high": {"valid": 1, "value": 0},
                },
            }
        ],
    )
    backend = EthercatRTCoreBackend(robot_config=robot_cfg)
    backend._axis_to_joint = [0]
    backend._axis_config = _AxisConfig(
        num_axes=1,
        counts_per_unit=[100.0] + [0.0] * 15,
        sign=[1] + [0] * 15,
        counts_per_rev=[131072] + [0] * 15,
    )
    backend._rt_num_axes = 1
    backend._rt_drive_profile_id = "a6ec_ds402"

    snapshot = backend.get_display_feedback_snapshot({0: 250})

    assert isinstance(snapshot, dict)
    assert snapshot["truth_available"] is False
    axis_detail = snapshot["axis_absolute_feedback"][0]
    assert axis_detail["truth_reason"] == "drive_native_absolute_home_anchor_missing"


def test_ethercat_backend_fails_closed_when_required_drive_native_startup_settings_are_missing(
    monkeypatch, tmp_path
):
    monkeypatch.setenv("GRADIENT_JOINT_ZERO_OFFSETS_PATH", str(tmp_path / "joint_zero_offsets.json"))
    metrics_path = tmp_path / "metrics.json"
    monkeypatch.setattr(rtcore_backend_module, "_RTCORE_METRICS_PATH", metrics_path)
    _write_rtcore_metrics_snapshot(
        metrics_path,
        [
            {
                "statusword": 0x9650,
                "error_code": 0,
                "manufacturer_error_code": 0,
                "startup_drive_config": {
                    "setting_key": "a6ec_encoder_position_tracking_mode",
                    "configured": 1,
                    "commanded": 4,
                    "readback_valid": 1,
                    "readback": 4,
                    "verified": 1,
                },
            }
        ],
    )
    backend = EthercatRTCoreBackend(robot_config=Gradient05Config().get_config_dict())
    backend._axis_to_joint = [4]
    backend._axis_config = _AxisConfig(
        num_axes=1,
        counts_per_unit=[131072.0 / (2.0 * 3.141592653589793)] + [0.0] * 15,
        sign=[-1] + [0] * 15,
        counts_per_rev=[131072] + [0] * 15,
    )
    backend._rt_num_axes = 1
    backend._rt_drive_profile_id = "a6ec_ds402"

    snapshot = backend.get_display_feedback_snapshot({0: -250})

    assert snapshot["truth_available"] is False
    assert snapshot["drive_native_startup_valid"] is False
    assert snapshot["drive_native_truth_available"] is False
    axis_detail = snapshot["axis_absolute_feedback"][0]
    assert axis_detail["drive_native_startup_valid"] is False
    assert axis_detail["drive_native_startup_reason"] == "startup_drive_config_missing_required_settings"
    assert axis_detail["truth_reason"] == "drive_native_startup_drive_config_missing_required_settings"


def test_ethercat_backend_normalizes_j3_style_wrapped_feedback_counts_for_display():
    backend = _force_legacy_truth_fallback(
        EthercatRTCoreBackend(robot_config=Gradient05Config().get_config_dict())
    )
    backend._axis_config = _AxisConfig(
        num_axes=1,
        counts_per_unit=[(131072.0 * 100.0) / (2.0 * 3.141592653589793)] + [0.0] * 15,
        sign=[-1] + [0] * 15,
        counts_per_rev=[131072] + [0] * 15,
    )
    backend._rt_drive_profile_id = "a6ec_ds402"

    assert backend._display_feedback_counts_for_axis(0, 131039) == -33
    assert backend._display_axis_q_from_raw_feedback_counts(0, 131039) == pytest.approx(
        33.0 / ((131072.0 * 100.0) / (2.0 * 3.141592653589793)),
        abs=1e-9,
    )


def test_ethercat_backend_j3_style_native_home_capture_should_zero_pose_at_wrap_seam(
    monkeypatch, tmp_path
):
    zero_offsets_path = tmp_path / "joint_zero_offsets.json"
    absolute_anchor_path = tmp_path / "absolute_encoder_anchors.json"
    metrics_path = tmp_path / "metrics.json"
    monkeypatch.setenv("GRADIENT_JOINT_ZERO_OFFSETS_PATH", str(zero_offsets_path))
    monkeypatch.setenv("GRADIENT_ABSOLUTE_ENCODER_ANCHORS_PATH", str(absolute_anchor_path))
    monkeypatch.setattr(rtcore_backend_module, "_RTCORE_METRICS_PATH", metrics_path)
    _write_rtcore_metrics_snapshot(
        metrics_path,
        [
            {
                "native_home_position_offset": 0,
                "absolute_feedback": {
                    "absolute_position_reference": {"valid": 1, "value": -32},
                    "encoder_multi_turn_low": {"valid": 1, "value": 77850},
                    "encoder_multi_turn_high": {"valid": 1, "value": 0},
                    "rotation_mode_position_reference": {"valid": 1, "value": 131041},
                    "rotation_mode_encoder_low": {"valid": 1, "value": 131041},
                    "rotation_mode_encoder_high": {"valid": 1, "value": 0},
                },
            }
        ],
    )
    backend = _force_legacy_truth_fallback(
        EthercatRTCoreBackend(robot_config=Gradient05Config().get_config_dict())
    )
    backend._connected = True
    backend._rt_num_axes = 1
    backend._axis_to_joint = [2]
    backend._axis_config = _AxisConfig(
        num_axes=1,
        counts_per_unit=[(131072.0 * 100.0) / (2.0 * 3.141592653589793)] + [0.0] * 15,
        sign=[-1] + [0] * 15,
        counts_per_rev=[131072] + [0] * 15,
    )
    backend._rt_drive_profile_id = "a6ec_ds402"

    raw_positions = {0: 131039}
    captured_anchor = backend._capture_absolute_home_anchor_for_joint(
        2,
        raw_positions=raw_positions,
        actor="pytest:j3_wrap_home",
        reference_mode="display",
    )
    snapshot = backend.get_display_feedback_snapshot(raw_positions)

    assert captured_anchor is not None
    assert isinstance(snapshot, dict)
    axis_detail = snapshot["axis_absolute_feedback"][0]

    # Product expectation: after defining the current physical pose as native-home zero,
    # the displayed operator pose should be near zero at that same seam-wrapped sample.
    assert axis_detail["truth_available"] is True
    assert axis_detail["absolute_counts"] == 77850
    assert backend._display_feedback_counts_for_axis(0, 131039) == -33
    assert axis_detail["absolute_axis_q_rad"] == pytest.approx(
        -77850.0 / ((131072.0 * 100.0) / (2.0 * 3.141592653589793)),
        abs=1e-9,
    )
    assert captured_anchor["home_anchor_rad"] == pytest.approx(
        axis_detail["absolute_axis_q_rad"],
        abs=5e-5,
    )
    assert axis_detail["canonical_rad"] == pytest.approx(0.0, abs=5e-5)
    assert snapshot["joint_positions_rad_partial"][2] == pytest.approx(0.0, abs=5e-5)


def test_ethercat_backend_translates_canonical_truth_back_into_raw_wire_counts(monkeypatch, tmp_path):
    monkeypatch.setenv("GRADIENT_JOINT_ZERO_OFFSETS_PATH", str(tmp_path / "joint_zero_offsets.json"))
    metrics_path = tmp_path / "metrics.json"
    monkeypatch.setattr(rtcore_backend_module, "_RTCORE_METRICS_PATH", metrics_path)
    _write_rtcore_metrics_snapshot(
        metrics_path,
        [
            {
                "absolute_feedback": {
                    "encoder_multi_turn_low": {"valid": 1, "value": 1234},
                    "encoder_multi_turn_high": {"valid": 1, "value": 0},
                },
            }
        ],
    )
    backend = _force_legacy_truth_fallback(
        EthercatRTCoreBackend(robot_config=Gradient05Config().get_config_dict())
    )
    backend._axis_to_joint = [0]
    backend._axis_config = _AxisConfig(
        num_axes=1,
        counts_per_unit=[100.0] + [0.0] * 15,
        sign=[1] + [0] * 15,
        counts_per_rev=[131072] + [0] * 15,
    )
    backend._rt_num_axes = 1
    backend._rt_drive_profile_id = "a6ec_ds402"
    backend._absolute_encoder_home_anchors[0] = {"home_anchor_rad": 10.0, "source": "pytest", "axis_indices": [0]}

    controller_positions = backend.raw_to_joint_positions({0: 234})
    display_positions = backend.raw_to_display_joint_positions({0: 234})

    controller_axis_q = backend._axis_q_from_joint_positions(controller_positions)
    display_axis_q = backend._axis_q_from_joint_positions(display_positions)

    assert controller_positions[0] == pytest.approx(2.34)
    assert display_positions[0] == pytest.approx(2.34)
    assert round(controller_axis_q[0] * 100.0) == 234
    assert round(display_axis_q[0] * 100.0) == 234


def test_a6ec_small_jog_at_seam_stays_within_half_rm_wire_delta(monkeypatch, tmp_path):
    # Near the RM-1 seam, a small canonical jog must produce a 607A target
    # whose SHORTEST-ANGULAR distance from live 6064 is the requested jog
    # amount, even if that means the linear counts land on the opposite
    # side of the [0, RM) seam (which is now the command path's output
    # range, see the 2026-04-17 wrap-to-[0,RM) change in
    # _nearest_turn_fold_axis_q_for_axis).
    monkeypatch.setenv("GRADIENT_JOINT_ZERO_OFFSETS_PATH", str(tmp_path / "joint_zero_offsets.json"))
    monkeypatch.setenv(
        "GRADIENT_ABSOLUTE_ENCODER_ANCHORS_PATH",
        str(tmp_path / "absolute_encoder_anchors.json"),
    )
    metrics_path = tmp_path / "metrics.json"
    monkeypatch.setattr(rtcore_backend_module, "_RTCORE_METRICS_PATH", metrics_path)
    _write_rtcore_metrics_snapshot(metrics_path, [{"absolute_feedback": {}}])
    counts_per_rev = 131072
    gear_ratio = 10.0
    counts_per_unit = (float(counts_per_rev) * gear_ratio) / (2.0 * math.pi)
    rm = int(round(counts_per_unit * 2.0 * math.pi))
    # Use the real drive-native-ratio fold path (not legacy fallback). The
    # wrap-to-[0, RM) behavior under test only matters when `period_counts`
    # is the joint-revolution period, which is what the A6-EC native ratio
    # mode produces (counts_per_unit * 2*pi = 1,310,720 for J6-style 10:1
    # gearing).
    backend = EthercatRTCoreBackend(robot_config=Gradient05Config().get_config_dict())
    backend._connected = True
    backend._rt_num_axes = 1
    backend._axis_to_joint = [0]
    backend._axis_config = _AxisConfig(
        num_axes=1,
        counts_per_unit=[counts_per_unit] + [0.0] * 15,
        sign=[-1] + [0] * 15,
        counts_per_rev=[counts_per_rev] + [0] * 15,
    )
    backend._rt_drive_profile_id = "a6ec_ds402"

    # Seam-adjacent live reading: a few counts below the positive seam.
    live_counts = rm - 4
    backend._axis_counts[0] = live_counts

    # Equivalent canonical position of the live 6064 is -4/(sign*cpu) rad.
    # A +0.5 deg canonical jog crosses the shaft seam from the host's
    # continuous-frame perspective.
    canonical_current = float(live_counts) / (-1.0 * counts_per_unit)
    jog_rad = math.radians(0.5)
    target_canonical = canonical_current + jog_rad
    target_axis_q = backend._command_axis_q_for_joint_value(
        axis_i=0,
        logical_joint_idx=0,
        canonical_q=target_canonical,
    )
    target_counts = int(round(target_axis_q * -1.0 * counts_per_unit))

    # 1. Command output must live in the drive's [0, RM) single-turn
    #    presentation range. Anything outside is ambiguous for the A6-EC
    #    in rotation mode regardless of C10.16.
    assert 0 <= target_counts < rm, (
        f"607A target must land in [0, RM), got {target_counts} vs RM={rm}"
    )
    # 2. Physical intent: deliver ~counts_per_unit*0.5deg on the shaft,
    #    measured as the SHORTEST-ANGULAR distance from live 6064.
    linear_delta = target_counts - live_counts
    half_period = rm // 2
    angular_delta = ((linear_delta + half_period) % rm) - half_period
    expected_step = int(round(counts_per_unit * math.radians(0.5)))
    assert abs(abs(angular_delta) - expected_step) < 2, (
        f"|shortest-angular 607A delta| should match the 0.5 deg jog, got "
        f"{angular_delta} (linear={linear_delta}, expected ~+/-{expected_step})"
    )
    # 3. And the linear delta itself must NOT exceed RM/2 in magnitude
    #    along the wrap boundary - otherwise a naive drive that doesn't
    #    do shortest-path would pick the wrong direction. The wrap-to-
    #    [0, RM) invariant + the small-jog requirement together bound
    #    the linear delta to a single shaft revolution minus the jog.
    assert abs(linear_delta) <= rm - 1, (
        f"607A linear delta must stay within one shaft turn, got {linear_delta}"
    )


def test_a6ec_experimental_continuous_607a_keeps_nearest_turn_without_single_turn_wrap(
    monkeypatch, tmp_path
):
    monkeypatch.setenv("GRADIENT_JOINT_ZERO_OFFSETS_PATH", str(tmp_path / "joint_zero_offsets.json"))
    monkeypatch.setenv(
        "GRADIENT_ABSOLUTE_ENCODER_ANCHORS_PATH",
        str(tmp_path / "absolute_encoder_anchors.json"),
    )
    monkeypatch.setenv("GRADIENT_RTCORE_EXPERIMENTAL_CONTINUOUS_607A_JOINTS", "1")
    metrics_path = tmp_path / "metrics.json"
    monkeypatch.setattr(rtcore_backend_module, "_RTCORE_METRICS_PATH", metrics_path)
    _write_rtcore_metrics_snapshot(metrics_path, [{"absolute_feedback": {}}])
    counts_per_rev = 131072
    gear_ratio = 10.0
    counts_per_unit = (float(counts_per_rev) * gear_ratio) / (2.0 * math.pi)
    rm = int(round(counts_per_unit * 2.0 * math.pi))
    backend = EthercatRTCoreBackend(robot_config=Gradient05Config().get_config_dict())
    backend._connected = True
    backend._rt_num_axes = 1
    backend._axis_to_joint = [0]
    backend._axis_config = _AxisConfig(
        num_axes=1,
        counts_per_unit=[counts_per_unit] + [0.0] * 15,
        sign=[-1] + [0] * 15,
        counts_per_rev=[counts_per_rev] + [0] * 15,
    )
    backend._rt_drive_profile_id = "a6ec_ds402"

    live_counts = rm - 4
    target_wrapped_counts = 5
    target_canonical = float(target_wrapped_counts) / (-1.0 * counts_per_unit)
    target_axis_q = backend._command_axis_q_for_joint_value(
        axis_i=0,
        logical_joint_idx=0,
        canonical_q=target_canonical,
        live_reference_counts=live_counts,
    )
    target_counts = int(round(target_axis_q * -1.0 * counts_per_unit))

    assert target_counts >= rm, "experimental path should preserve the continuous nearest-turn frame"
    assert abs(target_counts - (rm + target_wrapped_counts)) <= 1
    assert abs(target_counts - live_counts) <= 16


def test_a6ec_experimental_continuous_607a_first_point_uses_logicalized_live_counts(
    monkeypatch, tmp_path
):
    monkeypatch.setenv("GRADIENT_JOINT_ZERO_OFFSETS_PATH", str(tmp_path / "joint_zero_offsets.json"))
    monkeypatch.setenv(
        "GRADIENT_ABSOLUTE_ENCODER_ANCHORS_PATH",
        str(tmp_path / "absolute_encoder_anchors.json"),
    )
    monkeypatch.setenv("GRADIENT_RTCORE_EXPERIMENTAL_CONTINUOUS_607A_JOINTS", "1")
    metrics_path = tmp_path / "metrics.json"
    monkeypatch.setattr(rtcore_backend_module, "_RTCORE_METRICS_PATH", metrics_path)
    counts_per_rev = 131072
    gear_ratio = 10.0
    counts_per_unit = (float(counts_per_rev) * gear_ratio) / (2.0 * math.pi)
    rm = int(round(counts_per_unit * 2.0 * math.pi))
    half_rm = rm // 2
    _write_rtcore_metrics_snapshot(
        metrics_path,
        [
            {
                "native_home_position_offset": -half_rm,
                "absolute_feedback": {
                    "encoder_multi_turn_low": {"valid": 1, "value": 35771},
                    "encoder_multi_turn_high": {"valid": 1, "value": 0},
                },
            }
        ],
    )
    backend = EthercatRTCoreBackend(robot_config=Gradient05Config().get_config_dict())
    backend._connected = True
    backend._rt_num_axes = 1
    backend._axis_to_joint = [0]
    backend._axis_config = _AxisConfig(
        num_axes=1,
        counts_per_unit=[counts_per_unit] + [0.0] * 15,
        sign=[-1] + [0] * 15,
        counts_per_rev=[counts_per_rev] + [0] * 15,
    )
    backend._rt_drive_profile_id = "a6ec_ds402"

    live_counts = half_rm - 4
    target_wrapped_counts = -4
    target_canonical = float(target_wrapped_counts) / (-1.0 * counts_per_unit)
    target_axis_q = backend._command_axis_q_for_joint_value(
        axis_i=0,
        logical_joint_idx=0,
        canonical_q=target_canonical,
        live_reference_counts=live_counts,
    )

    backend._enforce_trajectory_wire_frame_safety(
        axis_q=[target_axis_q],
        axis_mask=0x1,
        previous_axis_counts=[None],
        initial_live_counts=[live_counts],
        point_index=0,
        traj_id=19,
    )


def test_a6ec_experimental_continuous_607a_bypasses_seam_fail_closed_guard(
    monkeypatch, tmp_path
):
    monkeypatch.setenv("GRADIENT_JOINT_ZERO_OFFSETS_PATH", str(tmp_path / "joint_zero_offsets.json"))
    monkeypatch.setenv(
        "GRADIENT_ABSOLUTE_ENCODER_ANCHORS_PATH",
        str(tmp_path / "absolute_encoder_anchors.json"),
    )
    monkeypatch.setenv("GRADIENT_RTCORE_EXPERIMENTAL_CONTINUOUS_607A_JOINTS", "1")
    metrics_path = tmp_path / "metrics.json"
    monkeypatch.setattr(rtcore_backend_module, "_RTCORE_METRICS_PATH", metrics_path)
    counts_per_rev = 131072
    gear_ratio = 10.0
    counts_per_unit = (float(counts_per_rev) * gear_ratio) / (2.0 * math.pi)
    rm = int(round(counts_per_unit * 2.0 * math.pi))
    half_rm = rm // 2
    _write_rtcore_metrics_snapshot(
        metrics_path,
        [
            {
                "native_home_position_offset": -half_rm,
                "absolute_feedback": {
                    "encoder_multi_turn_low": {"valid": 1, "value": 35771},
                    "encoder_multi_turn_high": {"valid": 1, "value": 0},
                },
            }
        ],
    )
    backend = EthercatRTCoreBackend(robot_config=Gradient05Config().get_config_dict())
    backend._connected = True
    backend._rt_num_axes = 1
    backend._axis_to_joint = [0]
    backend._axis_config = _AxisConfig(
        num_axes=1,
        counts_per_unit=[counts_per_unit] + [0.0] * 15,
        sign=[-1] + [0] * 15,
        counts_per_rev=[counts_per_rev] + [0] * 15,
    )
    backend._rt_drive_profile_id = "a6ec_ds402"

    previous_axis_counts: list[int | None] = [None]
    initial_live_counts: list[int | None] = [half_rm - 4]

    first_target_counts = rm - 4
    first_axis_q = float(first_target_counts) / (-1.0 * counts_per_unit)
    backend._enforce_trajectory_wire_frame_safety(
        axis_q=[first_axis_q],
        axis_mask=0x1,
        previous_axis_counts=previous_axis_counts,
        initial_live_counts=initial_live_counts,
        point_index=0,
        traj_id=23,
    )
    assert previous_axis_counts == [first_target_counts]


def test_a6ec_native_home_waits_for_drive_disarmed_before_hm35(monkeypatch, tmp_path):
    # Vendor Q2: HM35 should only start after the drive confirms it has left
    # OperationEnabled. If the statusword stays OP-enabled for the full
    # disarm window, the backend must NOT send MSG_CMD_NATIVE_HOME and must
    # surface NATIVE_HOME_DISARM_PRECONDITION_TIMEOUT with the synthesized
    # abort code.
    monkeypatch.setenv("GRADIENT_JOINT_ZERO_OFFSETS_PATH", str(tmp_path / "joint_zero_offsets.json"))
    monkeypatch.setenv(
        "GRADIENT_ABSOLUTE_ENCODER_ANCHORS_PATH",
        str(tmp_path / "absolute_encoder_anchors.json"),
    )
    metrics_path = tmp_path / "metrics.json"
    monkeypatch.setattr(rtcore_backend_module, "_RTCORE_METRICS_PATH", metrics_path)
    _write_rtcore_metrics_snapshot(metrics_path, [{"absolute_feedback": {}}])
    # Keep the disarm window tight so the test is fast.
    monkeypatch.setenv("GRADIENT_RTCORE_NATIVE_HOME_DISARM_TIMEOUT_S", "0.1")

    backend = EthercatRTCoreBackend(robot_config=Gradient05Config().get_config_dict())
    backend._connected = True
    backend._rt_num_axes = 1
    backend._axis_to_joint = [0]
    backend._axis_config = _AxisConfig(
        num_axes=1,
        counts_per_unit=[100.0] + [0.0] * 15,
        sign=[1] + [0] * 15,
        counts_per_rev=[131072] + [0] * 15,
    )
    backend._rt_drive_profile_id = "a6ec_ds402"
    # Simulate an axis that is stuck in OperationEnabled (DS402 state 5).
    backend._axis_ds402_state[0] = 5

    sent_native_home: list[int] = []
    sent_axis_disable: list[int] = []

    def _capture_native_home(axis_mask: int) -> None:
        sent_native_home.append(int(axis_mask))

    def _capture_axis_disable(axis_mask: int) -> None:
        sent_axis_disable.append(int(axis_mask))

    monkeypatch.setattr(backend, "_send_cmd_native_home", _capture_native_home)
    monkeypatch.setattr(backend, "_send_cmd_axis_disable", _capture_axis_disable)
    monkeypatch.setattr(backend, "abort_trajectory", lambda *a, **kw: None)
    monkeypatch.setattr(backend, "stop_joint_velocity_lease_jog", lambda *a, **kw: None)

    result = backend.native_home_joint(0)

    assert sent_native_home == [], (
        "HM35 must not be dispatched while the axis is still OperationEnabled"
    )
    assert sent_axis_disable, (
        "Backend must explicitly request disable before waiting for disarm"
    )
    assert result["accepted"] is False
    assert result["verified"] is False
    assert result["code"] == "NATIVE_HOME_DISARM_PRECONDITION_TIMEOUT"
    assert result["native_home_last_abort_code"] == 0xF1000001
    assert result["native_home_last_abort_code_hex"] == "0xF1000001"
    assert result.get("disarm_precondition_timed_out") is True
    assert 0 in list(result.get("drive_op_enabled_axes", []))


def test_a6ec_truth_unavailable_when_anchor_and_6064_disagree_modulo_rm(
    monkeypatch, tmp_path
):
    # Workstream 3: when the anchored U40.20/.22 truth disagrees with live
    # 6064 modulo RM, canonical truth must fail closed with the explicit
    # reason multi_turn_anchor_inconsistent_with_live_6064.
    monkeypatch.setenv("GRADIENT_JOINT_ZERO_OFFSETS_PATH", str(tmp_path / "joint_zero_offsets.json"))
    monkeypatch.setenv(
        "GRADIENT_ABSOLUTE_ENCODER_ANCHORS_PATH",
        str(tmp_path / "absolute_encoder_anchors.json"),
    )
    metrics_path = tmp_path / "metrics.json"
    monkeypatch.setattr(rtcore_backend_module, "_RTCORE_METRICS_PATH", metrics_path)

    counts_per_unit = 100.0
    counts_per_rev = 628  # small RM to keep the math easy
    rm = counts_per_rev
    absolute_counts = 100  # anchored multi-turn counts
    # Disagree by RM/3 on the shaft frame.
    live_reference_counts = absolute_counts + rm // 3

    _write_rtcore_metrics_snapshot(
        metrics_path,
        [
            {
                "native_home_position_offset": 0,
                "absolute_feedback": {
                    "encoder_multi_turn_low": {"valid": 1, "value": absolute_counts},
                    "encoder_multi_turn_high": {"valid": 1, "value": 0},
                },
            }
        ],
    )
    backend = _force_legacy_truth_fallback(
        EthercatRTCoreBackend(robot_config=Gradient05Config().get_config_dict())
    )
    backend._axis_to_joint = [0]
    backend._axis_config = _AxisConfig(
        num_axes=1,
        counts_per_unit=[counts_per_unit] + [0.0] * 15,
        sign=[1] + [0] * 15,
        counts_per_rev=[counts_per_rev] + [0] * 15,
    )
    backend._rt_num_axes = 1
    backend._rt_drive_profile_id = "a6ec_ds402"
    # Anchor chosen so canonical_q = absolute_axis_q - anchor - master_offset
    # puts the anchored view at wire-counts = absolute_counts. With live 6064
    # RM/3 counts away, mod-RM delta is |RM/3| which is well above the gate.
    backend._absolute_encoder_home_anchors[0] = {
        "home_anchor_rad": 0.0,
        "source": "pytest",
        "axis_indices": [0],
    }

    snapshot = backend.get_display_feedback_snapshot({0: live_reference_counts})

    assert isinstance(snapshot, dict)
    assert snapshot["truth_available"] is False
    axis_detail = snapshot["axis_absolute_feedback"][0]
    assert axis_detail["truth_reason"] == "multi_turn_anchor_inconsistent_with_live_6064"
    assert axis_detail["shaft_frame_consistent"] is False
    assert abs(float(axis_detail["shaft_frame_mod_rm_delta_counts"])) > 16.0


def test_a6ec_command_frame_rejects_oversized_trajectory_step(monkeypatch, tmp_path):
    # A trajectory whose consecutive points step by more than the per-point
    # angular safety cage on the wire is a symptom of wrong-turn command-
    # frame math. The host pre-commit gate must refuse the upload with
    # command_frame_oversized_step. Since 2026-04-17 the cage measures
    # SHORTEST-ANGULAR distance (mod RM) instead of linear count distance
    # because the command path now wraps into [0, RM) and seam-adjacent
    # points can legitimately sit on opposite sides of the wrap while
    # being physically adjacent. The pathological case to catch is an
    # angular step near half-RM that can't be anything other than a
    # fold bug.
    monkeypatch.setenv("GRADIENT_JOINT_ZERO_OFFSETS_PATH", str(tmp_path / "joint_zero_offsets.json"))
    backend = EthercatRTCoreBackend(robot_config=Gradient05Config().get_config_dict())
    backend._connected = True
    backend._rt_num_axes = 1
    counts_per_rev = 131072
    gear_ratio = 10.0
    counts_per_unit = (float(counts_per_rev) * gear_ratio) / (2.0 * math.pi)
    rm = int(round(counts_per_unit * 2.0 * math.pi))
    backend._axis_config = _AxisConfig(
        num_axes=1,
        counts_per_unit=[counts_per_unit] + [0.0] * 15,
        sign=[1] + [0] * 15,
        counts_per_rev=[counts_per_rev] + [0] * 15,
    )
    backend._rt_drive_profile_id = "a6ec_ds402"

    # First point sits at live 6064 = 0 exactly; second point is offset by
    # ~100 deg in the angular-step frame (far above _TRAJECTORY_MAX_PER_POINT_STEP_RAD
    # of 0.35 rad ~= 20 deg), which can only come from a host fold bug that
    # lost track of which shaft turn the axis is on mid-trajectory.
    first_axis_q = 0.0
    second_counts = int(round(counts_per_unit * math.radians(100.0)))
    second_axis_q = float(second_counts) / counts_per_unit

    previous_axis_counts: list[int | None] = [None]
    initial_live_counts: list[int | None] = [0]
    backend._enforce_trajectory_wire_frame_safety(
        axis_q=[first_axis_q],
        axis_mask=0x1,
        previous_axis_counts=previous_axis_counts,
        initial_live_counts=initial_live_counts,
        point_index=0,
        traj_id=7,
    )
    with pytest.raises(RuntimeError, match="command_frame_oversized_step"):
        backend._enforce_trajectory_wire_frame_safety(
            axis_q=[second_axis_q],
            axis_mask=0x1,
            previous_axis_counts=previous_axis_counts,
            initial_live_counts=initial_live_counts,
            point_index=1,
            traj_id=7,
        )


def test_a6ec_command_frame_allows_seam_crossing_step_in_linear_counts(monkeypatch, tmp_path):
    # Regression for the 2026-04-19 continuous-607A landing: with
    # `command_frame_seam_crossing_unsafe=False` on the A6-EC profile and
    # RTCore emitting continuous `0x607A`, consecutive trajectory points
    # that straddle the single-turn seam in linear counts are valid. The
    # drive absorbs the wrap internally via its rotation-mode position
    # loop (C00.07=4, C10.16=0), so the host upload path must pass the
    # seam-crossing sequence through instead of failing closed on it.
    # Fails loudly if anyone re-enables the old
    # `command_frame_seam_crossing_unsafe=True` posture without the live
    # evidence that the new drive firmware semantics no longer hold.
    monkeypatch.setenv("GRADIENT_JOINT_ZERO_OFFSETS_PATH", str(tmp_path / "joint_zero_offsets.json"))
    backend = EthercatRTCoreBackend(robot_config=Gradient05Config().get_config_dict())
    backend._connected = True
    backend._rt_num_axes = 1
    counts_per_rev = 131072
    gear_ratio = 10.0
    counts_per_unit = (float(counts_per_rev) * gear_ratio) / (2.0 * math.pi)
    rm = int(round(counts_per_unit * 2.0 * math.pi))
    backend._axis_config = _AxisConfig(
        num_axes=1,
        counts_per_unit=[counts_per_unit] + [0.0] * 15,
        sign=[1] + [0] * 15,
        counts_per_rev=[counts_per_rev] + [0] * 15,
    )
    backend._rt_drive_profile_id = "a6ec_ds402"

    # Point 0 a few counts below the seam; point 1 a few counts above 0.
    # Linear step magnitude approaches RM; angular step is only ~+10 counts.
    first_counts = rm - 5
    second_counts = 5
    first_axis_q = float(first_counts) / counts_per_unit
    second_axis_q = float(second_counts) / counts_per_unit

    previous_axis_counts: list[int | None] = [None]
    initial_live_counts: list[int | None] = [first_counts]
    backend._enforce_trajectory_wire_frame_safety(
        axis_q=[first_axis_q],
        axis_mask=0x1,
        previous_axis_counts=previous_axis_counts,
        initial_live_counts=initial_live_counts,
        point_index=0,
        traj_id=13,
    )
    # Must NOT raise: seam-crossing step is allowed under continuous 607A.
    backend._enforce_trajectory_wire_frame_safety(
        axis_q=[second_axis_q],
        axis_mask=0x1,
        previous_axis_counts=previous_axis_counts,
        initial_live_counts=initial_live_counts,
        point_index=1,
        traj_id=13,
    )
    assert previous_axis_counts == [second_counts]


def test_a6ec_command_frame_rejects_point_far_from_live_reference(monkeypatch, tmp_path):
    # Direct regression for the 2026-04-17 J6 incident: a single point whose
    # 607A target lands far from live 6064 in SHORTEST-ANGULAR terms must
    # be rejected with command_frame_live_deviation_out_of_range. This
    # catches any fold/turn-selection bug before RTCore sees the upload.
    # Since the wrap-to-[0, RM) fold change, we measure angular distance
    # (mod RM) so a seam-straddling legitimate jog does not false-fail.
    monkeypatch.setenv("GRADIENT_JOINT_ZERO_OFFSETS_PATH", str(tmp_path / "joint_zero_offsets.json"))
    backend = EthercatRTCoreBackend(robot_config=Gradient05Config().get_config_dict())
    backend._connected = True
    backend._rt_num_axes = 1
    counts_per_rev = 131072
    gear_ratio = 10.0
    counts_per_unit = (float(counts_per_rev) * gear_ratio) / (2.0 * math.pi)
    backend._axis_config = _AxisConfig(
        num_axes=1,
        counts_per_unit=[counts_per_unit] + [0.0] * 15,
        sign=[1] + [0] * 15,
        counts_per_rev=[counts_per_rev] + [0] * 15,
    )
    backend._rt_drive_profile_id = "a6ec_ds402"

    live_counts = int(round(0.0 * counts_per_unit))
    # Target 45 deg away from live 6064 in angular terms - far above the
    # _TRAJECTORY_MAX_FIRST_POINT_DEVIATION_FROM_LIVE_RAD of 0.35 rad
    # (~20 deg). That is the pathological case a first-point teleport
    # bug would produce: canonical_q being badly mismatched with live 6064.
    target_counts = live_counts + int(round(counts_per_unit * math.radians(45.0)))
    target_axis_q = float(target_counts) / counts_per_unit

    previous_axis_counts: list[int | None] = [None]
    initial_live_counts: list[int | None] = [live_counts]
    with pytest.raises(RuntimeError, match="command_frame_live_deviation_out_of_range"):
        backend._enforce_trajectory_wire_frame_safety(
            axis_q=[target_axis_q],
            axis_mask=0x1,
            previous_axis_counts=previous_axis_counts,
            initial_live_counts=initial_live_counts,
            point_index=0,
            traj_id=9,
        )


def test_a6ec_command_frame_allows_seam_straddling_first_point(monkeypatch, tmp_path):
    # Regression for the 2026-04-19 continuous-607A landing: under the
    # A6-EC profile's `command_frame_seam_crossing_unsafe=False` posture
    # plus RTCore's continuous `0x607A` emission, a first-point target
    # that lands across the single-turn seam from live `6064` in LINEAR
    # counts must be accepted. The shortest-angular deviation is what
    # matters for Chapter 5 Figure 5-1 rotation-mode semantics, and the
    # host gate now uses that metric instead of the linear one.
    monkeypatch.setenv("GRADIENT_JOINT_ZERO_OFFSETS_PATH", str(tmp_path / "joint_zero_offsets.json"))
    backend = EthercatRTCoreBackend(robot_config=Gradient05Config().get_config_dict())
    backend._connected = True
    backend._rt_num_axes = 1
    counts_per_rev = 131072
    gear_ratio = 10.0
    counts_per_unit = (float(counts_per_rev) * gear_ratio) / (2.0 * math.pi)
    rm = int(round(counts_per_unit * 2.0 * math.pi))
    backend._axis_config = _AxisConfig(
        num_axes=1,
        counts_per_unit=[counts_per_unit] + [0.0] * 15,
        sign=[1] + [0] * 15,
        counts_per_rev=[counts_per_rev] + [0] * 15,
    )
    backend._rt_drive_profile_id = "a6ec_ds402"

    live_counts = rm - 4
    target_counts = 3
    target_axis_q = float(target_counts) / counts_per_unit

    previous_axis_counts: list[int | None] = [None]
    initial_live_counts: list[int | None] = [live_counts]
    # Must NOT raise: seam-straddling first point is allowed under
    # continuous 607A emission.
    backend._enforce_trajectory_wire_frame_safety(
        axis_q=[target_axis_q],
        axis_mask=0x1,
        previous_axis_counts=previous_axis_counts,
        initial_live_counts=initial_live_counts,
        point_index=0,
        traj_id=17,
    )
    assert previous_axis_counts == [target_counts]


def test_ethercat_backend_refuses_display_feedback_when_absolute_anchor_does_not_roundtrip(
    monkeypatch, tmp_path
):
    zero_offsets_path = tmp_path / "joint_zero_offsets.json"
    absolute_anchor_path = tmp_path / "absolute_encoder_anchors.json"
    metrics_path = tmp_path / "metrics.json"
    monkeypatch.setenv("GRADIENT_JOINT_ZERO_OFFSETS_PATH", str(zero_offsets_path))
    monkeypatch.setenv("GRADIENT_ABSOLUTE_ENCODER_ANCHORS_PATH", str(absolute_anchor_path))
    monkeypatch.setattr(rtcore_backend_module, "_RTCORE_METRICS_PATH", metrics_path)
    cfg = Gradient05Config().get_config_dict()
    save_absolute_encoder_anchor(
        cfg["robot_id"],
        num_joints=cfg["num_logical_joints"],
        logical_joint_index=0,
        home_anchor_rad=10.0,
        source="encoder_multi_turn_counts",
        axis_indices=[0],
        actor="pytest",
    )
    _write_rtcore_metrics_snapshot(
        metrics_path,
        [
            {
                "native_home_position_offset": 0,
                "absolute_feedback": {
                    "encoder_multi_turn_low": {"valid": 1, "value": 1234},
                    "encoder_multi_turn_high": {"valid": 1, "value": 0},
                },
            }
        ],
    )
    backend = _force_legacy_truth_fallback(EthercatRTCoreBackend(robot_config=cfg))
    backend._axis_to_joint = [0]
    backend._axis_config = _AxisConfig(
        num_axes=1,
        counts_per_unit=[100.0] + [0.0] * 15,
        sign=[1] + [0] * 15,
        counts_per_rev=[131072] + [0] * 15,
    )
    backend._rt_num_axes = 1
    backend._rt_drive_profile_id = "a6ec_ds402"

    snapshot = backend.get_display_feedback_snapshot({0: 131060})

    assert isinstance(snapshot, dict)
    assert snapshot["truth_available"] is False
    axis_detail = snapshot["axis_absolute_feedback"][0]
    # The Workstream 3 shaft-frame gate intercepts anchor-vs-6064 disagreement
    # before the older continuous-frame roundtrip; either reason indicates a
    # real frame inconsistency and both must keep truth unavailable.
    assert axis_detail["truth_reason"] in {
        "multi_turn_anchor_inconsistent_with_live_6064",
        "command_frame_roundtrip_mismatch",
        "absolute_home_anchor_stale",
    }
    assert snapshot["joint_positions_rad_partial"][0] is None
    assert all(value is None for value in snapshot["joint_positions_rad_partial"])
    with pytest.raises(RuntimeError, match="Operator display joint truth unavailable"):
        backend.raw_to_display_joint_positions({0: 131060})


def test_ethercat_backend_native_home_captures_absolute_encoder_anchor(monkeypatch, tmp_path):
    zero_offsets_path = tmp_path / "joint_zero_offsets.json"
    absolute_anchor_path = tmp_path / "absolute_encoder_anchors.json"
    metrics_path = tmp_path / "metrics.json"
    monkeypatch.setenv("GRADIENT_JOINT_ZERO_OFFSETS_PATH", str(zero_offsets_path))
    monkeypatch.setenv("GRADIENT_ABSOLUTE_ENCODER_ANCHORS_PATH", str(absolute_anchor_path))
    monkeypatch.setattr(rtcore_backend_module, "_RTCORE_METRICS_PATH", metrics_path)
    _write_rtcore_metrics_snapshot(
        metrics_path,
        [
            {
                "native_home_position_offset": 0,
                "absolute_feedback": {
                    "encoder_multi_turn_low": {"valid": 1, "value": 1025},
                    "encoder_multi_turn_high": {"valid": 1, "value": 0},
                },
            }
        ],
    )
    backend = _force_legacy_truth_fallback(
        EthercatRTCoreBackend(robot_config=Gradient05Config().get_config_dict())
    )
    backend._connected = True
    backend._rt_num_axes = 1
    backend._axis_to_joint = [0]
    backend._axis_config = _AxisConfig(
        num_axes=1,
        counts_per_unit=[100.0] + [0.0] * 15,
        sign=[1] + [0] * 15,
        counts_per_rev=[131072] + [0] * 15,
    )
    backend._rt_drive_profile_id = "a6ec_ds402"
    backend._axis_counts[0] = 25
    monkeypatch.setattr(backend, "prepare_for_power_transition", lambda **kwargs: None)
    monkeypatch.setattr(backend, "_send_cmd_native_home", lambda axis_mask: None)
    monkeypatch.setattr(
        backend,
        "_wait_for_native_home_result",
        lambda *args, **kwargs: {
            "verified": True,
            "terminal_state": "succeeded",
            "native_home_state": 2,
            "native_home_last_abort_code": 0,
            "metrics_time_ns": 123456789,
        },
    )

    result = backend.native_home_joint(0)

    anchors = load_absolute_encoder_anchors(backend._robot_id, num_joints=backend._num_joints)
    assert result["verified"] is True
    assert anchors[0] is not None
    assert anchors[0]["home_anchor_rad"] == pytest.approx(10.0)
    assert anchors[0]["source"] == "encoder_multi_turn_counts"


def test_ethercat_backend_native_home_reports_anchor_refresh_failure_when_capture_raises(
    monkeypatch, tmp_path
):
    zero_offsets_path = tmp_path / "joint_zero_offsets.json"
    absolute_anchor_path = tmp_path / "absolute_encoder_anchors.json"
    metrics_path = tmp_path / "metrics.json"
    monkeypatch.setenv("GRADIENT_JOINT_ZERO_OFFSETS_PATH", str(zero_offsets_path))
    monkeypatch.setenv("GRADIENT_ABSOLUTE_ENCODER_ANCHORS_PATH", str(absolute_anchor_path))
    monkeypatch.setattr(rtcore_backend_module, "_RTCORE_METRICS_PATH", metrics_path)
    _write_rtcore_metrics_snapshot(
        metrics_path,
        [
            {
                "native_home_position_offset": 0,
                "absolute_feedback": {
                    "encoder_multi_turn_low": {"valid": 1, "value": 1025},
                    "encoder_multi_turn_high": {"valid": 1, "value": 0},
                },
            }
        ],
    )
    backend = _force_legacy_truth_fallback(
        EthercatRTCoreBackend(robot_config=Gradient05Config().get_config_dict())
    )
    backend._connected = True
    backend._rt_num_axes = 1
    backend._axis_to_joint = [0]
    backend._axis_config = _AxisConfig(
        num_axes=1,
        counts_per_unit=[100.0] + [0.0] * 15,
        sign=[1] + [0] * 15,
        counts_per_rev=[131072] + [0] * 15,
    )
    backend._rt_drive_profile_id = "a6ec_ds402"
    backend._axis_counts[0] = 25
    monkeypatch.setattr(backend, "prepare_for_power_transition", lambda **kwargs: None)
    monkeypatch.setattr(backend, "_send_cmd_native_home", lambda axis_mask: None)
    monkeypatch.setattr(
        backend,
        "_wait_for_native_home_result",
        lambda *args, **kwargs: {
            "verified": True,
            "terminal_state": "succeeded",
            "native_home_state": 2,
            "native_home_last_abort_code": 0,
            "metrics_time_ns": 123456789,
        },
    )

    def _raise_capture(*args, **kwargs):
        raise RuntimeError("anchor capture blew up")

    monkeypatch.setattr(backend, "_capture_absolute_home_anchor_for_joint", _raise_capture)

    result = backend.native_home_joint(0)

    anchors = load_absolute_encoder_anchors(backend._robot_id, num_joints=backend._num_joints)
    assert result["accepted"] is True
    assert result["verified"] is False
    assert result["code"] == "NATIVE_HOME_ANCHOR_REFRESH_FAILED"
    assert result["absolute_home_anchor_capture_succeeded"] is False
    assert result["absolute_home_anchor_refresh_ok"] is False
    assert "anchor" in result["message"].lower()
    assert "anchor capture blew up" in result["post_home_anchor_refresh_error"]
    assert anchors[0] is None


def test_ethercat_backend_native_home_requires_post_home_anchor_validation(monkeypatch, tmp_path):
    zero_offsets_path = tmp_path / "joint_zero_offsets.json"
    absolute_anchor_path = tmp_path / "absolute_encoder_anchors.json"
    metrics_path = tmp_path / "metrics.json"
    monkeypatch.setenv("GRADIENT_JOINT_ZERO_OFFSETS_PATH", str(zero_offsets_path))
    monkeypatch.setenv("GRADIENT_ABSOLUTE_ENCODER_ANCHORS_PATH", str(absolute_anchor_path))
    monkeypatch.setattr(rtcore_backend_module, "_RTCORE_METRICS_PATH", metrics_path)
    _write_rtcore_metrics_snapshot(
        metrics_path,
        [
            {
                "native_home_position_offset": 0,
                "statusword": 0x9650,
                "error_code": 0,
                "manufacturer_error_code": 0,
                "native_home_state": 0,
                "native_home_last_abort_code": 0,
                "slave_online": 1,
                "slave_operational": 1,
                "absolute_feedback": {
                    "encoder_multi_turn_low": {"valid": 1, "value": 1025},
                    "encoder_multi_turn_high": {"valid": 1, "value": 0},
                },
            }
        ],
    )
    backend = _force_legacy_truth_fallback(
        EthercatRTCoreBackend(robot_config=Gradient05Config().get_config_dict())
    )
    backend._connected = True
    backend._rt_num_axes = 1
    backend._axis_to_joint = [0]
    backend._axis_config = _AxisConfig(
        num_axes=1,
        counts_per_unit=[100.0] + [0.0] * 15,
        sign=[1] + [0] * 15,
        counts_per_rev=[131072] + [0] * 15,
    )
    backend._rt_drive_profile_id = "a6ec_ds402"
    backend._axis_counts[0] = 25
    backend._absolute_encoder_home_anchors[0] = {"home_anchor_rad": 0.0, "source": "pytest", "axis_indices": [0]}
    monkeypatch.setattr(backend, "prepare_for_power_transition", lambda **kwargs: None)
    monkeypatch.setattr(backend, "_send_cmd_native_home", lambda axis_mask: None)
    monkeypatch.setattr(
        backend,
        "_wait_for_native_home_result",
        lambda *args, **kwargs: {
            "verified": True,
            "terminal_state": "succeeded",
            "native_home_state": 2,
            "native_home_last_abort_code": 0,
            "metrics_time_ns": 123456789,
        },
    )
    monkeypatch.setattr(
        backend,
        "_capture_absolute_home_anchor_for_joint",
        lambda *args, **kwargs: {
            "home_anchor_rad": 10.0,
            "source": "encoder_multi_turn_counts",
            "axis_indices": [0],
        },
    )

    result = backend.native_home_joint(0)

    assert result["accepted"] is True
    assert result["verified"] is False
    assert result["code"] == "NATIVE_HOME_ANCHOR_REFRESH_FAILED"
    assert result["absolute_home_anchor_capture_succeeded"] is True
    assert result["absolute_home_anchor_refresh_ok"] is False
    # Workstream 3 primary gate names this condition; older installations
    # may surface it as absolute_home_anchor_stale via the legacy path.
    assert result["post_home_truth_reason"] in {
        "multi_turn_anchor_inconsistent_with_live_6064",
        "absolute_home_anchor_stale",
    }
    assert result["post_home_axis"] == 0
    # Accept either the legacy roundtrip error field or the Workstream 3
    # shaft-frame delta field; both encode the same ~1000-count anchor
    # disagreement depending on which path surfaced it first.
    legacy_error_counts = result.get("post_home_command_roundtrip_reference_error_counts")
    shaft_delta_counts = result.get("post_home_shaft_frame_mod_rm_delta_counts")
    if legacy_error_counts is not None:
        assert legacy_error_counts == pytest.approx(1000.0)
    else:
        assert shaft_delta_counts is not None, (
            "Either roundtrip error counts or shaft-frame mod-RM delta must be "
            "present in the post-home detail"
        )
        assert abs(float(shaft_delta_counts)) == pytest.approx(1000.0)


def test_ethercat_backend_native_home_uses_raw_reference_mode_for_post_home_anchor_refresh(
    monkeypatch, tmp_path
):
    zero_offsets_path = tmp_path / "joint_zero_offsets.json"
    absolute_anchor_path = tmp_path / "absolute_encoder_anchors.json"
    metrics_path = tmp_path / "metrics.json"
    monkeypatch.setenv("GRADIENT_JOINT_ZERO_OFFSETS_PATH", str(zero_offsets_path))
    monkeypatch.setenv("GRADIENT_ABSOLUTE_ENCODER_ANCHORS_PATH", str(absolute_anchor_path))
    monkeypatch.setattr(rtcore_backend_module, "_RTCORE_METRICS_PATH", metrics_path)
    _write_rtcore_metrics_snapshot(
        metrics_path,
        [
            {
                "native_home_position_offset": 655360,
                "statusword": 0x9650,
                "error_code": 0,
                "manufacturer_error_code": 0,
                "native_home_state": 2,
                "native_home_last_abort_code": 0,
                "slave_online": 1,
                "slave_operational": 1,
                "absolute_feedback": {
                    "encoder_multi_turn_low": {"valid": 1, "value": 35771},
                    "encoder_multi_turn_high": {"valid": 1, "value": 0},
                },
            }
        ],
    )
    backend = _force_legacy_truth_fallback(
        EthercatRTCoreBackend(robot_config=Gradient05Config().get_config_dict())
    )
    backend._connected = True
    backend._rt_num_axes = 1
    backend._axis_to_joint = [0]
    backend._axis_config = _AxisConfig(
        num_axes=1,
        counts_per_unit=[(131072.0 * 10.0) / (2.0 * math.pi)] + [0.0] * 15,
        sign=[-1] + [0] * 15,
        counts_per_rev=[131072] + [0] * 15,
    )
    backend._rt_drive_profile_id = "a6ec_ds402"
    backend._axis_counts[0] = 655360
    monkeypatch.setattr(backend, "prepare_for_power_transition", lambda **kwargs: None)
    monkeypatch.setattr(backend, "_send_cmd_native_home", lambda axis_mask: None)
    monkeypatch.setattr(
        backend,
        "_wait_for_native_home_result",
        lambda *args, **kwargs: {
            "verified": True,
            "terminal_state": "succeeded",
            "native_home_state": 2,
            "native_home_last_abort_code": 0,
            "metrics_time_ns": 123456789,
        },
    )
    monkeypatch.setattr(
        backend,
        "_wait_for_native_home_post_settle_result",
        lambda *args, **kwargs: {
            "ok": True,
            "timed_out": False,
            "hard_failure": False,
            "failure_reason": None,
            "metrics_time_ns": 456,
            "axis_results": [],
        },
    )
    monkeypatch.setattr(backend, "_refresh_native_home_offsets_from_metrics", lambda: None)
    monkeypatch.setattr(backend, "sync_read_positions", lambda: {0: 655360})

    captured_modes: list[str] = []
    validated_modes: list[str] = []

    def _capture(*args, **kwargs):
        captured_modes.append(str(kwargs.get("reference_mode")))
        return {"home_anchor_rad": 1.0, "source": "encoder_multi_turn_counts", "axis_indices": [0]}

    def _validate(*args, **kwargs):
        validated_modes.append(str(kwargs.get("reference_mode")))
        return {
            "ok": True,
            "truth_available": True,
            "truth_reason": None,
            "axis": 0,
            "logical_joint": 1,
        }

    monkeypatch.setattr(backend, "_capture_absolute_home_anchor_for_joint", _capture)
    monkeypatch.setattr(backend, "_absolute_home_anchor_validation_for_joint", _validate)

    result = backend.native_home_joint(0)

    assert result["accepted"] is True
    assert result["verified"] is True
    assert captured_modes == ["raw"]
    assert validated_modes == ["raw"]


def test_ethercat_backend_software_zero_refreshes_absolute_encoder_anchor(monkeypatch, tmp_path):
    zero_offsets_path = tmp_path / "joint_zero_offsets.json"
    absolute_anchor_path = tmp_path / "absolute_encoder_anchors.json"
    metrics_path = tmp_path / "metrics.json"
    monkeypatch.setenv("GRADIENT_JOINT_ZERO_OFFSETS_PATH", str(zero_offsets_path))
    monkeypatch.setenv("GRADIENT_ABSOLUTE_ENCODER_ANCHORS_PATH", str(absolute_anchor_path))
    monkeypatch.setattr(rtcore_backend_module, "_RTCORE_METRICS_PATH", metrics_path)
    _write_rtcore_metrics_snapshot(
        metrics_path,
        [
            {
                "native_home_position_offset": 0,
                "absolute_feedback": {
                    "encoder_multi_turn_low": {"valid": 1, "value": 1200},
                    "encoder_multi_turn_high": {"valid": 1, "value": 0},
                },
            }
        ],
    )
    backend = _force_legacy_truth_fallback(
        EthercatRTCoreBackend(robot_config=Gradient05Config().get_config_dict())
    )
    backend._connected = True
    backend._rt_num_axes = 1
    backend._axis_to_joint = [0]
    backend._axis_config = _AxisConfig(
        num_axes=1,
        counts_per_unit=[100.0] + [0.0] * 15,
        sign=[1] + [0] * 15,
        counts_per_rev=[131072] + [0] * 15,
    )
    backend._rt_drive_profile_id = "a6ec_ds402"
    backend._axis_counts[0] = 200

    assert backend.set_logical_joint_current_position_as_zero(0) is True

    anchors = load_absolute_encoder_anchors(backend._robot_id, num_joints=backend._num_joints)
    loaded_offsets = load_joint_zero_offsets(backend._robot_id, num_joints=backend._num_joints)
    assert anchors[0] is not None
    assert anchors[0]["home_anchor_rad"] == pytest.approx(10.0)
    assert loaded_offsets[0] == pytest.approx(2.0)


def test_ethercat_backend_prefers_robot_defined_axis_scaling(monkeypatch, tmp_path):
    monkeypatch.setenv("GRADIENT_JOINT_ZERO_OFFSETS_PATH", str(tmp_path / "joint_zero_offsets.json"))
    metrics_path = tmp_path / "metrics.json"
    monkeypatch.setattr(rtcore_backend_module, "_RTCORE_METRICS_PATH", metrics_path)
    _write_rtcore_metrics_snapshot(
        metrics_path,
        [
            {
                "absolute_feedback": {
                    "encoder_multi_turn_low": {"valid": 1, "value": -1046180},
                    "encoder_multi_turn_high": {"valid": 1, "value": -1},
                },
            }
        ],
    )
    cfg = Gradient05Config().get_config_dict()
    cfg["configured_drive_profile_id"] = "cia402"
    backend = _force_legacy_truth_fallback(EthercatRTCoreBackend(robot_config=cfg))
    backend._axis_to_joint = [2]

    # Simulate RTCore publishing placeholder runtime scaling; robot config should win.
    backend._runtime_axis_config = _AxisConfig(
        num_axes=1,
        counts_per_unit=[100.0] + [0.0] * 15,
        sign=[1] + [0] * 15,
    )
    backend._axis_config = backend._robot_axis_config
    backend._rt_drive_profile_id = "a6ec_ds402"
    backend._absolute_encoder_home_anchors[2] = {"home_anchor_rad": 0.0, "source": "pytest", "axis_indices": [0]}

    logical_positions = backend.raw_to_joint_positions({0: -1046180})

    expected = -1046180 / (-cfg["actuator_counts_per_radian"][2])
    assert logical_positions[2] == pytest.approx(expected)


def test_ethercat_backend_zero_capture_persists_joint_offsets(monkeypatch, tmp_path):
    offsets_path = tmp_path / "joint_zero_offsets.json"
    monkeypatch.setenv("GRADIENT_JOINT_ZERO_OFFSETS_PATH", str(offsets_path))
    # Isolate the RTCore metrics path so a live stack's metrics.json
    # (at /run/gradient-rt-motion/metrics.json) cannot leak a non-zero
    # `native_home_position_offset` into the zero-capture math. Without
    # this, running pytest while the stack is up pollutes the expected
    # `physical_q = raw / cpu` with the drive's live native-home offset.
    monkeypatch.setattr(
        rtcore_backend_module, "_RTCORE_METRICS_PATH", tmp_path / "metrics.json"
    )
    backend = _force_legacy_truth_fallback(
        EthercatRTCoreBackend(robot_config=Gradient05Config().get_config_dict())
    )
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


def test_ethercat_backend_connected_reads_return_canonical_feedback(monkeypatch, tmp_path):
    monkeypatch.setenv("GRADIENT_JOINT_ZERO_OFFSETS_PATH", str(tmp_path / "joint_zero_offsets.json"))
    metrics_path = tmp_path / "metrics.json"
    monkeypatch.setattr(rtcore_backend_module, "_RTCORE_METRICS_PATH", metrics_path)
    _write_rtcore_metrics_snapshot(
        metrics_path,
        [
            {
                "absolute_feedback": {
                    "encoder_multi_turn_low": {"valid": 1, "value": 25},
                    "encoder_multi_turn_high": {"valid": 1, "value": 0},
                },
            },
            {
                "absolute_feedback": {
                    "encoder_multi_turn_low": {"valid": 1, "value": 50},
                    "encoder_multi_turn_high": {"valid": 1, "value": 0},
                },
            },
        ],
    )
    backend = _force_legacy_truth_fallback(
        EthercatRTCoreBackend(robot_config=Gradient05Config().get_config_dict())
    )
    backend._connected = True
    backend._rt_num_axes = 2
    backend._axis_to_joint = [0, 1]
    backend._axis_config = _AxisConfig(
        num_axes=2,
        counts_per_unit=[100.0, 100.0] + [0.0] * 14,
        sign=[1, 1] + [0] * 14,
    )
    backend._axis_counts[0] = 25
    backend._axis_counts[1] = 50
    backend._rt_drive_profile_id = "a6ec_ds402"
    backend._absolute_encoder_home_anchors[0] = {"home_anchor_rad": 0.0, "source": "pytest", "axis_indices": [0]}
    backend._absolute_encoder_home_anchors[1] = {"home_anchor_rad": 0.0, "source": "pytest", "axis_indices": [1]}

    assert backend.sync_read_positions() == {0: 25, 1: 50}
    assert backend.get_joint_positions()[:2] == pytest.approx([0.25, 0.5])


def test_ethercat_backend_marks_truth_unavailable_when_absolute_anchor_does_not_roundtrip(
    monkeypatch, tmp_path
):
    monkeypatch.setenv("GRADIENT_JOINT_ZERO_OFFSETS_PATH", str(tmp_path / "joint_zero_offsets.json"))
    metrics_path = tmp_path / "metrics.json"
    monkeypatch.setattr(rtcore_backend_module, "_RTCORE_METRICS_PATH", metrics_path)
    _write_rtcore_metrics_snapshot(
        metrics_path,
        [
            {
                "native_home_position_offset": 0,
                "absolute_feedback": {
                    "encoder_multi_turn_low": {"valid": 1, "value": 1234},
                    "encoder_multi_turn_high": {"valid": 1, "value": 0},
                },
            }
        ],
    )
    backend = _force_legacy_truth_fallback(
        EthercatRTCoreBackend(robot_config=Gradient05Config().get_config_dict())
    )
    backend._connected = True
    backend._rt_num_axes = 1
    backend._axis_to_joint = [0]
    backend._axis_config = _AxisConfig(
        num_axes=1,
        counts_per_unit=[100.0] + [0.0] * 15,
        sign=[1] + [0] * 15,
        counts_per_rev=[131072] + [0] * 15,
    )
    backend._rt_drive_profile_id = "a6ec_ds402"
    backend._axis_counts[0] = 234
    backend._absolute_encoder_home_anchors[0] = {"home_anchor_rad": 0.0, "source": "pytest", "axis_indices": [0]}

    snapshot = backend.get_display_feedback_snapshot({0: 234})

    assert isinstance(snapshot, dict)
    assert snapshot["truth_available"] is False
    axis_detail = snapshot["axis_absolute_feedback"][0]
    # Workstream 3 primary gate catches this at the mod-RM check; the
    # legacy continuous-frame roundtrip reason remains acceptable for
    # installations that see the condition via the older path.
    assert axis_detail["truth_reason"] in {
        "multi_turn_anchor_inconsistent_with_live_6064",
        "command_frame_roundtrip_mismatch",
    }
    with pytest.raises(RuntimeError, match="Canonical joint truth unavailable"):
        backend.get_joint_positions()


def test_ethercat_backend_accepts_stationary_roundtrip_wander_within_configured_tolerance(
    monkeypatch, tmp_path
):
    monkeypatch.setenv("GRADIENT_JOINT_ZERO_OFFSETS_PATH", str(tmp_path / "joint_zero_offsets.json"))
    metrics_path = tmp_path / "metrics.json"
    monkeypatch.setattr(rtcore_backend_module, "_RTCORE_METRICS_PATH", metrics_path)
    tolerated_counts = int(rtcore_backend_module._COMMAND_ROUNDTRIP_TOLERANCE_COUNTS)
    _write_rtcore_metrics_snapshot(
        metrics_path,
        [
            {
                "native_home_position_offset": 0,
                "absolute_feedback": {
                    "encoder_multi_turn_low": {"valid": 1, "value": 234 + tolerated_counts},
                    "encoder_multi_turn_high": {"valid": 1, "value": 0},
                },
            }
        ],
    )
    backend = _force_legacy_truth_fallback(
        EthercatRTCoreBackend(robot_config=Gradient05Config().get_config_dict())
    )
    backend._connected = True
    backend._rt_num_axes = 1
    backend._axis_to_joint = [0]
    backend._axis_config = _AxisConfig(
        num_axes=1,
        counts_per_unit=[100.0] + [0.0] * 15,
        sign=[1] + [0] * 15,
        counts_per_rev=[131072] + [0] * 15,
    )
    backend._rt_drive_profile_id = "a6ec_ds402"
    backend._axis_counts[0] = 234
    backend._absolute_encoder_home_anchors[0] = {"home_anchor_rad": 0.0, "source": "pytest", "axis_indices": [0]}

    snapshot = backend.get_display_feedback_snapshot({0: 234})

    assert isinstance(snapshot, dict)
    assert snapshot["truth_available"] is True
    axis_detail = snapshot["axis_absolute_feedback"][0]
    assert axis_detail["command_roundtrip_consistent"] is True
    assert axis_detail["command_roundtrip_reference_error_counts"] == pytest.approx(float(tolerated_counts))
    assert axis_detail["truth_available"] is True
    assert axis_detail["canonical_rad"] == pytest.approx((234 + tolerated_counts) / 100.0)
    assert backend.get_joint_positions()[0] == pytest.approx((234 + tolerated_counts) / 100.0)


def test_ethercat_backend_rejects_roundtrip_wander_beyond_configured_tolerance(monkeypatch, tmp_path):
    monkeypatch.setenv("GRADIENT_JOINT_ZERO_OFFSETS_PATH", str(tmp_path / "joint_zero_offsets.json"))
    metrics_path = tmp_path / "metrics.json"
    monkeypatch.setattr(rtcore_backend_module, "_RTCORE_METRICS_PATH", metrics_path)
    tolerated_counts = int(rtcore_backend_module._COMMAND_ROUNDTRIP_TOLERANCE_COUNTS)
    _write_rtcore_metrics_snapshot(
        metrics_path,
        [
            {
                "native_home_position_offset": 0,
                "absolute_feedback": {
                    "encoder_multi_turn_low": {"valid": 1, "value": 235 + tolerated_counts},
                    "encoder_multi_turn_high": {"valid": 1, "value": 0},
                },
            }
        ],
    )
    backend = _force_legacy_truth_fallback(
        EthercatRTCoreBackend(robot_config=Gradient05Config().get_config_dict())
    )
    backend._connected = True
    backend._rt_num_axes = 1
    backend._axis_to_joint = [0]
    backend._axis_config = _AxisConfig(
        num_axes=1,
        counts_per_unit=[100.0] + [0.0] * 15,
        sign=[1] + [0] * 15,
        counts_per_rev=[131072] + [0] * 15,
    )
    backend._rt_drive_profile_id = "a6ec_ds402"
    backend._axis_counts[0] = 234
    backend._absolute_encoder_home_anchors[0] = {"home_anchor_rad": 0.0, "source": "pytest", "axis_indices": [0]}

    snapshot = backend.get_display_feedback_snapshot({0: 234})

    assert isinstance(snapshot, dict)
    assert snapshot["truth_available"] is False
    axis_detail = snapshot["axis_absolute_feedback"][0]
    assert axis_detail["command_roundtrip_consistent"] is False
    assert axis_detail["command_roundtrip_reference_error_counts"] == pytest.approx(float(tolerated_counts + 1))
    assert axis_detail["truth_reason"] == "command_frame_roundtrip_mismatch"


def test_ethercat_backend_diagnoses_stale_absolute_home_anchor_when_clean_homed_frame_disagrees(
    monkeypatch, tmp_path
):
    monkeypatch.setenv("GRADIENT_JOINT_ZERO_OFFSETS_PATH", str(tmp_path / "joint_zero_offsets.json"))
    monkeypatch.setenv(
        "GRADIENT_ABSOLUTE_ENCODER_ANCHORS_PATH",
        str(tmp_path / "absolute_encoder_anchors.json"),
    )
    metrics_path = tmp_path / "metrics.json"
    monkeypatch.setattr(rtcore_backend_module, "_RTCORE_METRICS_PATH", metrics_path)
    _write_rtcore_metrics_snapshot(
        metrics_path,
        [
            {
                "native_home_position_offset": 0,
                "statusword": 0x9650,
                "error_code": 0,
                "manufacturer_error_code": 0,
                "native_home_state": 0,
                "native_home_last_abort_code": 0,
                "slave_online": 1,
                "slave_operational": 1,
                "absolute_feedback": {
                    "encoder_multi_turn_low": {"valid": 1, "value": 1234},
                    "encoder_multi_turn_high": {"valid": 1, "value": 0},
                },
            }
        ],
    )
    backend = _force_legacy_truth_fallback(
        EthercatRTCoreBackend(robot_config=Gradient05Config().get_config_dict())
    )
    backend._connected = True
    backend._rt_num_axes = 1
    backend._axis_to_joint = [0]
    backend._axis_config = _AxisConfig(
        num_axes=1,
        counts_per_unit=[100.0] + [0.0] * 15,
        sign=[1] + [0] * 15,
        counts_per_rev=[131072] + [0] * 15,
    )
    backend._rt_drive_profile_id = "a6ec_ds402"
    backend._axis_counts[0] = 234
    backend._absolute_encoder_home_anchors[0] = {"home_anchor_rad": 0.0, "source": "pytest", "axis_indices": [0]}

    snapshot = backend.get_display_feedback_snapshot({0: 234})

    assert isinstance(snapshot, dict)
    axis_detail = snapshot["axis_absolute_feedback"][0]
    assert axis_detail["truth_available"] is False
    # Workstream 3 primary gate names this condition explicitly. The
    # stale-anchor diagnostic fields are still populated alongside so
    # operators keep the full detail; the older reason remains acceptable
    # for installations that surface it via the legacy roundtrip path.
    assert axis_detail["truth_reason"] in {
        "multi_turn_anchor_inconsistent_with_live_6064",
        "absolute_home_anchor_stale",
    }
    assert axis_detail["absolute_home_anchor_stale"] is True
    assert axis_detail["absolute_home_anchor_implied_rad"] == pytest.approx(10.0)
    assert axis_detail["absolute_home_anchor_delta_counts"] == pytest.approx(1000.0)
    assert axis_detail["absolute_home_anchor_stale_tolerance_counts"] == pytest.approx(8.0, abs=1e-5)
    assert axis_detail["native_home_state_name"] == "succeeded"
    assert axis_detail["native_home_verification_source"] == "statusword_bits12_15_clear13"
    assert axis_detail["statusword_hex"] == "0x9650"


def test_ethercat_backend_disconnected_get_joint_positions_fails_closed(monkeypatch, tmp_path):
    monkeypatch.setenv("GRADIENT_JOINT_ZERO_OFFSETS_PATH", str(tmp_path / "joint_zero_offsets.json"))
    backend = EthercatRTCoreBackend(robot_config=Gradient05Config().get_config_dict())
    backend._store_last_joint_setpoint_rad([1.0] * backend.num_joints)

    with pytest.raises(RuntimeError, match="Canonical joint truth unavailable"):
        backend.get_joint_positions()


def test_ethercat_backend_connected_without_feedback_config_fails_closed(monkeypatch, tmp_path):
    monkeypatch.setenv("GRADIENT_JOINT_ZERO_OFFSETS_PATH", str(tmp_path / "joint_zero_offsets.json"))
    backend = EthercatRTCoreBackend(robot_config=Gradient05Config().get_config_dict())
    backend._connected = True
    backend._axis_config = None
    backend._store_last_joint_setpoint_rad([1.0] * backend.num_joints)
    monkeypatch.setattr(backend, "_wait_for_feedback_ready", lambda **_kwargs: False)

    with pytest.raises(RuntimeError, match="Canonical joint truth unavailable"):
        backend.get_joint_positions()


def test_ethercat_backend_startup_bootstraps_missing_absolute_home_anchor(monkeypatch, tmp_path):
    monkeypatch.setenv("GRADIENT_JOINT_ZERO_OFFSETS_PATH", str(tmp_path / "joint_zero_offsets.json"))
    monkeypatch.setenv(
        "GRADIENT_ABSOLUTE_ENCODER_ANCHORS_PATH",
        str(tmp_path / "absolute_encoder_anchors.json"),
    )
    metrics_path = tmp_path / "metrics.json"
    monkeypatch.setattr(rtcore_backend_module, "_RTCORE_METRICS_PATH", metrics_path)
    _write_rtcore_metrics_snapshot(
        metrics_path,
        [
            {
                "absolute_feedback": {
                    "encoder_multi_turn_low": {"valid": 1, "value": 25},
                    "encoder_multi_turn_high": {"valid": 1, "value": 0},
                },
            }
        ],
    )
    backend = _force_legacy_truth_fallback(
        EthercatRTCoreBackend(robot_config=Gradient05Config().get_config_dict())
    )
    backend._connected = True
    backend._rt_num_axes = 1
    backend._axis_to_joint = [0]
    backend._axis_config = _AxisConfig(
        num_axes=1,
        counts_per_unit=[100.0] + [0.0] * 15,
        sign=[1] + [0] * 15,
    )
    backend._axis_counts[0] = 25
    backend._rt_drive_profile_id = "a6ec_ds402"

    result = backend._bootstrap_missing_absolute_home_anchors(actor="pytest:startup_alignment")

    anchors = load_absolute_encoder_anchors(backend._robot_id, num_joints=backend._num_joints)
    assert result == {"created_joints": [1], "missing_joints": []}
    assert anchors[0] is not None
    assert anchors[0]["home_anchor_rad"] == pytest.approx(0.0)
    assert backend.get_joint_positions()[0] == pytest.approx(0.25)


def test_ethercat_backend_startup_bootstrap_uses_display_reference_mode(monkeypatch, tmp_path):
    monkeypatch.setenv("GRADIENT_JOINT_ZERO_OFFSETS_PATH", str(tmp_path / "joint_zero_offsets.json"))
    monkeypatch.setenv(
        "GRADIENT_ABSOLUTE_ENCODER_ANCHORS_PATH",
        str(tmp_path / "absolute_encoder_anchors.json"),
    )
    metrics_path = tmp_path / "metrics.json"
    monkeypatch.setattr(rtcore_backend_module, "_RTCORE_METRICS_PATH", metrics_path)
    _write_rtcore_metrics_snapshot(
        metrics_path,
        [
            {
                "native_home_position_offset": 0,
                "absolute_feedback": {
                    "absolute_position_reference": {"valid": 1, "value": -32},
                    "encoder_multi_turn_low": {"valid": 1, "value": 77850},
                    "encoder_multi_turn_high": {"valid": 1, "value": 0},
                    "rotation_mode_position_reference": {"valid": 1, "value": 131041},
                    "rotation_mode_encoder_low": {"valid": 1, "value": 131041},
                    "rotation_mode_encoder_high": {"valid": 1, "value": 0},
                },
            }
        ],
    )
    backend = _force_legacy_truth_fallback(
        EthercatRTCoreBackend(robot_config=Gradient05Config().get_config_dict())
    )
    backend._connected = True
    backend._rt_num_axes = 1
    backend._axis_to_joint = [2]
    backend._axis_config = _AxisConfig(
        num_axes=1,
        counts_per_unit=[(131072.0 * 100.0) / (2.0 * 3.141592653589793)] + [0.0] * 15,
        sign=[-1] + [0] * 15,
        counts_per_rev=[131072] + [0] * 15,
    )
    backend._axis_counts[0] = 131039
    backend._rt_drive_profile_id = "a6ec_ds402"

    result = backend._bootstrap_missing_absolute_home_anchors(actor="pytest:startup_alignment")
    snapshot = backend.get_display_feedback_snapshot({0: 131039})

    anchors = load_absolute_encoder_anchors(backend._robot_id, num_joints=backend._num_joints)
    assert result == {"created_joints": [3], "missing_joints": []}
    assert anchors[2] is not None
    assert isinstance(snapshot, dict)
    assert snapshot["truth_available"] is True
    assert snapshot["joint_positions_rad_partial"][2] == pytest.approx(0.0, abs=5e-5)


def test_ethercat_backend_startup_bootstrap_keeps_existing_absolute_home_anchor(monkeypatch, tmp_path):
    monkeypatch.setenv("GRADIENT_JOINT_ZERO_OFFSETS_PATH", str(tmp_path / "joint_zero_offsets.json"))
    monkeypatch.setenv(
        "GRADIENT_ABSOLUTE_ENCODER_ANCHORS_PATH",
        str(tmp_path / "absolute_encoder_anchors.json"),
    )
    save_absolute_encoder_anchor(
        "gradient-05",
        num_joints=6,
        logical_joint_index=0,
        home_anchor_rad=9.0,
        source="pytest:existing",
        axis_indices=[0],
        actor="pytest:seed_existing_anchor",
    )
    metrics_path = tmp_path / "metrics.json"
    monkeypatch.setattr(rtcore_backend_module, "_RTCORE_METRICS_PATH", metrics_path)
    _write_rtcore_metrics_snapshot(
        metrics_path,
        [
            {
                "absolute_feedback": {
                    "encoder_multi_turn_low": {"valid": 1, "value": 25},
                    "encoder_multi_turn_high": {"valid": 1, "value": 0},
                },
            }
        ],
    )
    backend = EthercatRTCoreBackend(robot_config=Gradient05Config().get_config_dict())
    backend._connected = True
    backend._rt_num_axes = 1
    backend._axis_to_joint = [0]
    backend._axis_config = _AxisConfig(
        num_axes=1,
        counts_per_unit=[100.0] + [0.0] * 15,
        sign=[1] + [0] * 15,
    )
    backend._axis_counts[0] = 25
    backend._rt_drive_profile_id = "a6ec_ds402"

    result = backend._bootstrap_missing_absolute_home_anchors(actor="pytest:startup_alignment")

    anchors = load_absolute_encoder_anchors(backend._robot_id, num_joints=backend._num_joints)
    assert result == {"created_joints": [], "missing_joints": []}
    assert anchors[0] is not None
    assert anchors[0]["home_anchor_rad"] == pytest.approx(9.0)
    assert backend._absolute_encoder_home_anchors[0]["home_anchor_rad"] == pytest.approx(9.0)


def test_ethercat_backend_connected_reads_fail_closed_when_anchor_bootstrap_cannot_reconstruct_truth(
    monkeypatch, tmp_path
):
    monkeypatch.setenv("GRADIENT_JOINT_ZERO_OFFSETS_PATH", str(tmp_path / "joint_zero_offsets.json"))
    monkeypatch.setenv(
        "GRADIENT_ABSOLUTE_ENCODER_ANCHORS_PATH",
        str(tmp_path / "absolute_encoder_anchors.json"),
    )
    metrics_path = tmp_path / "metrics.json"
    monkeypatch.setattr(rtcore_backend_module, "_RTCORE_METRICS_PATH", metrics_path)
    _write_rtcore_metrics_snapshot(metrics_path, [{}])
    backend = _force_legacy_truth_fallback(
        EthercatRTCoreBackend(robot_config=Gradient05Config().get_config_dict())
    )
    backend._connected = True
    backend._rt_num_axes = 1
    backend._axis_to_joint = [0]
    backend._axis_config = _AxisConfig(
        num_axes=1,
        counts_per_unit=[100.0] + [0.0] * 15,
        sign=[1] + [0] * 15,
    )
    backend._axis_counts[0] = 25
    backend._rt_drive_profile_id = "a6ec_ds402"

    result = backend._bootstrap_missing_absolute_home_anchors(actor="pytest:startup_alignment")

    assert result == {"created_joints": [], "missing_joints": [1]}
    with pytest.raises(RuntimeError, match="Canonical joint truth unavailable"):
        backend.get_joint_positions()


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
    backend._axis_to_joint = [0, 1, 2]
    backend._axis_config = _AxisConfig(
        num_axes=3,
        counts_per_unit=[100.0, 100.0, 100.0] + [0.0] * 13,
        sign=[1, 1, 1] + [0] * 13,
    )
    backend._axis_counts[0] = 25
    backend._native_home_offset_counts[0] = -25
    monkeypatch.setattr(backend, "_refresh_native_home_offsets_from_metrics", lambda: None)

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
    # Python power-up synchronization should keep controller setpoints in the
    # same logical/native-home-aware frame exposed through joint feedback.
    assert backend._last_joint_setpoint_rad[0] == pytest.approx(0.0)


def test_ethercat_backend_native_home_waits_for_neutral_before_request(monkeypatch, tmp_path):
    monkeypatch.setenv("GRADIENT_JOINT_ZERO_OFFSETS_PATH", str(tmp_path / "joint_zero_offsets.json"))
    backend = EthercatRTCoreBackend(robot_config=Gradient05Config().get_config_dict())
    backend._connected = True
    backend._rt_num_axes = 3
    backend._axis_to_joint = [0, 1, 2]

    calls: list[tuple[str, object]] = []
    monkeypatch.setattr(
        backend,
        "prepare_for_power_transition",
        lambda **kwargs: calls.append(("prepare", dict(kwargs))),
    )
    monkeypatch.setattr(
        backend,
        "_send_cmd_native_home",
        lambda axis_mask: calls.append(("native_home", axis_mask)),
    )
    monkeypatch.setattr(
        backend,
        "_wait_for_native_home_result",
        lambda axis_mask, *, timeout_s, min_metrics_time_ns=0, min_metrics_mtime_ns=0: (
            calls.append(
                (
                    "wait_native_home",
                    (axis_mask, timeout_s, min_metrics_time_ns, min_metrics_mtime_ns),
                )
            )
            or {
                "verified": True,
                "timed_out": False,
                "terminal_state": "succeeded",
                "native_home_state": 2,
                "native_home_last_abort_code": 0,
                "metrics_time_ns": 123,
            }
        ),
    )
    monkeypatch.setattr(backend, "_refresh_native_home_offsets_from_metrics", lambda: None)
    monkeypatch.setattr(backend, "sync_read_positions", lambda: {1: 0})
    monkeypatch.setattr(
        backend,
        "_capture_absolute_home_anchor_for_joint",
        lambda *args, **kwargs: {
            "home_anchor_rad": 0.0,
            "source": "encoder_multi_turn_counts",
            "axis_indices": [1],
        },
    )
    monkeypatch.setattr(
        backend,
        "_absolute_home_anchor_validation_for_joint",
        lambda *_args, **_kwargs: {
            "ok": True,
            "truth_available": True,
            "axis": 1,
            "logical_joint": 1,
            "command_roundtrip_reference_error_counts": 0.0,
            "command_roundtrip_reference_error_rad": 0.0,
        },
    )
    monkeypatch.setattr(
        backend,
        "_wait_for_native_home_post_settle_result",
        lambda axis_mask, *, timeout_s, min_metrics_time_ns=0, min_metrics_mtime_ns=0: (
            calls.append(
                (
                    "wait_post_home_settle",
                    (axis_mask, timeout_s, min_metrics_time_ns, min_metrics_mtime_ns),
                )
            )
            or {
                "ok": True,
                "timed_out": False,
                "hard_failure": False,
                "failure_reason": None,
                "metrics_time_ns": 321,
                "axis_results": [],
            }
        ),
    )

    result = backend.native_home_joint(1)
    assert result["accepted"] is True
    assert result["verified"] is True
    assert result["code"] == "NATIVE_HOME_VERIFIED"
    # native_home_joint now also asserts drive-confirmed disarm via
    # require_drive_disarmed + require_drive_disarmed_axis_mask, per
    # vendor Q2 "stationary and inactive" before HM35.
    assert calls[0][0] == "prepare"
    prepare_kwargs = calls[0][1]
    assert prepare_kwargs.get("wait_for_idle") is True
    assert prepare_kwargs.get("timeout_s") == 1.0
    assert prepare_kwargs.get("require_drive_disarmed") is True
    assert int(prepare_kwargs.get("require_drive_disarmed_axis_mask", 0)) != 0
    assert calls[1] == ("native_home", 0x2)
    assert calls[2][0] == "wait_native_home"
    wait_axis_mask, wait_timeout_s, wait_time_ns, wait_mtime_ns = calls[2][1]
    assert wait_axis_mask == 0x2
    assert wait_timeout_s == 20.0
    assert wait_time_ns >= 0
    assert wait_mtime_ns >= 0
    assert calls[3][0] == "wait_post_home_settle"
    settle_axis_mask, settle_timeout_s, settle_time_ns, settle_mtime_ns = calls[3][1]
    assert settle_axis_mask == 0x2
    assert settle_timeout_s == 3.0
    assert settle_time_ns == 123
    assert settle_mtime_ns == 0


def test_ethercat_backend_native_home_downgrades_verified_result_when_post_home_settle_faults(
    monkeypatch, tmp_path
):
    monkeypatch.setenv("GRADIENT_JOINT_ZERO_OFFSETS_PATH", str(tmp_path / "joint_zero_offsets.json"))
    backend = _force_legacy_truth_fallback(
        EthercatRTCoreBackend(robot_config=Gradient05Config().get_config_dict())
    )
    backend._connected = True
    backend._rt_num_axes = 3
    backend._axis_to_joint = [0, 1, 2]

    monkeypatch.setattr(backend, "prepare_for_power_transition", lambda **_kwargs: None)
    monkeypatch.setattr(backend, "_send_cmd_native_home", lambda axis_mask: None)
    monkeypatch.setattr(
        backend,
        "_wait_for_native_home_result",
        lambda *args, **kwargs: {
            "verified": True,
            "timed_out": False,
            "terminal_state": "succeeded",
            "native_home_state": 2,
            "native_home_last_abort_code": 0,
            "metrics_time_ns": 123,
        },
    )
    monkeypatch.setattr(backend, "_refresh_native_home_offsets_from_metrics", lambda: None)
    monkeypatch.setattr(backend, "sync_read_positions", lambda: {1: 140})
    monkeypatch.setattr(
        backend,
        "_capture_absolute_home_anchor_for_joint",
        lambda *args, **kwargs: {
            "home_anchor_rad": 0.25,
            "source": "encoder_multi_turn_counts",
            "axis_indices": [1],
        },
    )
    monkeypatch.setattr(
        backend,
        "_absolute_home_anchor_validation_for_joint",
        lambda *_args, **_kwargs: {
            "ok": True,
            "truth_available": True,
            "axis": 1,
            "logical_joint": 1,
            "command_roundtrip_reference_error_counts": 0.0,
            "command_roundtrip_reference_error_rad": 0.0,
        },
    )
    monkeypatch.setattr(
        backend,
        "_wait_for_native_home_post_settle_result",
        lambda *args, **kwargs: {
            "ok": False,
            "timed_out": False,
            "hard_failure": True,
            "failure_reason": "drive_faulted",
            "metrics_time_ns": 456,
            "axis_results": [
                {
                    "axis": 1,
                    "native_home_state": 2,
                    "native_home_state_name": "succeeded",
                    "native_home_last_abort_code": 0,
                    "native_home_last_abort_code_hex": "0x00000000",
                    "statusword": 0x9638,
                    "statusword_hex": "0x9638",
                    "statusword_fault": True,
                    "error_code": 0xFF00,
                    "error_code_hex": "0xFF00",
                    "manufacturer_error_code": 0,
                    "manufacturer_error_code_hex": "0x00000000",
                    "slave_online": True,
                    "slave_operational": True,
                    "native_home_active": False,
                    "failure_reason": "drive_faulted",
                    "clean": False,
                }
            ],
        },
    )

    result = backend.native_home_joint(1)

    assert result["accepted"] is True
    assert result["verified"] is False
    assert result["code"] == "NATIVE_HOME_POST_HOME_SETTLE_FAILED"
    assert result["absolute_home_anchor_capture_succeeded"] is True
    assert result["absolute_home_anchor_refresh_ok"] is True
    assert result["post_home_settle_ok"] is False
    assert result["post_home_settle_hard_failure"] is True
    assert result["post_home_settle_reason"] == "drive_faulted"
    assert result["post_home_settle_error_code"] == 0xFF00
    assert result["post_home_settle_statusword_hex"] == "0x9638"
    assert result["metrics_time_ns"] == 456


def test_ethercat_backend_native_home_retries_transient_post_home_truth_failure_after_settle(
    monkeypatch, tmp_path
):
    monkeypatch.setenv("GRADIENT_JOINT_ZERO_OFFSETS_PATH", str(tmp_path / "joint_zero_offsets.json"))
    backend = EthercatRTCoreBackend(robot_config=Gradient05Config().get_config_dict())
    backend._connected = True
    backend._rt_num_axes = 3
    backend._axis_to_joint = [0, 1, 2]

    monkeypatch.setattr(backend, "_absolute_home_anchor_required", lambda: False)
    monkeypatch.setattr(backend, "prepare_for_power_transition", lambda **_kwargs: None)
    monkeypatch.setattr(backend, "_send_cmd_native_home", lambda axis_mask: None)
    monkeypatch.setattr(
        backend,
        "_wait_for_native_home_result",
        lambda *args, **kwargs: {
            "verified": True,
            "timed_out": False,
            "terminal_state": "succeeded",
            "native_home_state": 2,
            "native_home_last_abort_code": 0,
            "metrics_time_ns": 123,
        },
    )
    monkeypatch.setattr(backend, "_refresh_native_home_offsets_from_metrics", lambda: None)
    monkeypatch.setattr(backend, "sync_read_positions", lambda: {1: 140})
    validations = iter(
        [
            {
                "ok": False,
                "truth_available": False,
                "truth_reason": "drive_native_fault_present",
                "axis": 1,
                "logical_joint": 1,
            },
            {
                "ok": True,
                "truth_available": True,
                "axis": 1,
                "logical_joint": 1,
            },
        ]
    )
    monkeypatch.setattr(
        backend,
        "_absolute_home_anchor_validation_for_joint",
        lambda *_args, **_kwargs: next(validations),
    )
    monkeypatch.setattr(
        backend,
        "_wait_for_native_home_post_settle_result",
        lambda *args, **kwargs: {
            "ok": True,
            "timed_out": False,
            "hard_failure": False,
            "failure_reason": None,
            "metrics_time_ns": 456,
            "axis_results": [],
        },
    )

    result = backend.native_home_joint(1)

    assert result["accepted"] is True
    assert result["verified"] is True
    assert result["code"] == "NATIVE_HOME_VERIFIED"
    assert result["post_home_truth_retry_after_settle"] is True
    assert result["post_home_settle_ok"] is True
    assert result["post_home_truth_available"] is True
    assert "post_home_truth_reason" not in result
    assert result["metrics_time_ns"] == 123


def test_ethercat_backend_native_home_aborts_when_pre_neutralization_fails(monkeypatch, tmp_path):
    monkeypatch.setenv("GRADIENT_JOINT_ZERO_OFFSETS_PATH", str(tmp_path / "joint_zero_offsets.json"))
    backend = EthercatRTCoreBackend(robot_config=Gradient05Config().get_config_dict())
    backend._connected = True
    backend._rt_num_axes = 3
    backend._axis_to_joint = [0, 1, 2]

    calls: list[tuple[str, object]] = []

    def _raise_prepare(**_kwargs):
        raise RuntimeError("neutralization failed")

    monkeypatch.setattr(backend, "prepare_for_power_transition", _raise_prepare)
    monkeypatch.setattr(
        backend,
        "_send_cmd_native_home",
        lambda axis_mask: calls.append(("native_home", axis_mask)),
    )

    result = backend.native_home_joint(1)
    assert result["accepted"] is False
    assert result["verified"] is False
    assert result["code"] == "NATIVE_HOME_PRECONDITION_FAILED"
    assert calls == []


def test_ethercat_backend_native_home_reports_pending_when_verification_times_out(monkeypatch, tmp_path):
    monkeypatch.setenv("GRADIENT_JOINT_ZERO_OFFSETS_PATH", str(tmp_path / "joint_zero_offsets.json"))
    backend = EthercatRTCoreBackend(robot_config=Gradient05Config().get_config_dict())
    backend._connected = True
    backend._rt_num_axes = 3
    backend._axis_to_joint = [0, 1, 2]

    calls: list[tuple[str, object]] = []
    monkeypatch.setattr(
        backend,
        "prepare_for_power_transition",
        lambda **kwargs: calls.append(("prepare", dict(kwargs))),
    )
    monkeypatch.setattr(
        backend,
        "_send_cmd_native_home",
        lambda axis_mask: calls.append(("native_home", axis_mask)),
    )
    monkeypatch.setattr(
        backend,
        "_wait_for_native_home_result",
        lambda axis_mask, *, timeout_s, min_metrics_time_ns=0, min_metrics_mtime_ns=0: (
            calls.append(
                (
                    "wait_native_home",
                    (axis_mask, timeout_s, min_metrics_time_ns, min_metrics_mtime_ns),
                )
            )
            or {
                "verified": False,
                "timed_out": True,
                "terminal_state": "pending",
                "native_home_state": 1,
                "native_home_last_abort_code": 0,
                "metrics_time_ns": 456,
            }
        ),
    )
    monkeypatch.setattr(
        backend,
        "_load_rtcore_metrics_snapshot",
        lambda: {
            "time_ns": 999,
            "axes": [{}, {"native_home_state": 1, "native_home_last_abort_code": 0, "statusword": 0x0233}, {}],
        },
    )

    result = backend.native_home_joint(1)
    assert result["accepted"] is True
    assert result["verified"] is False
    assert result["timed_out"] is True
    assert result["code"] == "NATIVE_HOME_PENDING_VERIFICATION"
    # native_home_joint now also asserts drive-confirmed disarm via
    # require_drive_disarmed + require_drive_disarmed_axis_mask, per
    # vendor Q2 "stationary and inactive" before HM35.
    assert calls[0][0] == "prepare"
    prepare_kwargs = calls[0][1]
    assert prepare_kwargs.get("wait_for_idle") is True
    assert prepare_kwargs.get("timeout_s") == 1.0
    assert prepare_kwargs.get("require_drive_disarmed") is True
    assert int(prepare_kwargs.get("require_drive_disarmed_axis_mask", 0)) != 0
    assert calls[1] == ("native_home", 0x2)
    assert calls[2][0] == "wait_native_home"
    wait_axis_mask, wait_timeout_s, wait_time_ns, wait_mtime_ns = calls[2][1]
    assert wait_axis_mask == 0x2
    assert wait_timeout_s == 20.0
    assert wait_time_ns >= 0
    assert wait_mtime_ns >= 0


def test_ethercat_backend_native_home_recovers_clean_success_after_wait_timeout(monkeypatch, tmp_path):
    monkeypatch.setenv("GRADIENT_JOINT_ZERO_OFFSETS_PATH", str(tmp_path / "joint_zero_offsets.json"))
    monkeypatch.setenv(
        "GRADIENT_ABSOLUTE_ENCODER_ANCHORS_PATH",
        str(tmp_path / "absolute_encoder_anchors.json"),
    )
    metrics_path = tmp_path / "metrics.json"
    monkeypatch.setattr(rtcore_backend_module, "_RTCORE_METRICS_PATH", metrics_path)
    _write_rtcore_metrics_snapshot(metrics_path, [{"absolute_feedback": {}}])
    backend = EthercatRTCoreBackend(robot_config=Gradient05Config().get_config_dict())
    backend._connected = True
    backend._rt_num_axes = 3
    backend._axis_to_joint = [0, 1, 2]
    backend._axis_config = _AxisConfig(
        num_axes=3,
        counts_per_unit=[100.0, 100.0, 100.0] + [0.0] * 13,
        sign=[1, 1, 1] + [0] * 13,
        counts_per_rev=[131072, 131072, 131072] + [0] * 13,
    )
    backend._rt_drive_profile_id = "a6ec_ds402"

    calls: list[tuple[str, object]] = []
    monkeypatch.setattr(
        backend,
        "prepare_for_power_transition",
        lambda **kwargs: calls.append(("prepare", dict(kwargs))),
    )
    monkeypatch.setattr(
        backend,
        "_send_cmd_native_home",
        lambda axis_mask: calls.append(("native_home", axis_mask)),
    )
    monkeypatch.setattr(
        backend,
        "_wait_for_native_home_result",
        lambda *args, **kwargs: {
            "verified": False,
            "timed_out": True,
            "terminal_state": "pending",
            "native_home_state": 1,
            "native_home_last_abort_code": 0,
            "metrics_time_ns": 456,
        },
    )
    monkeypatch.setattr(
        backend,
        "_load_rtcore_metrics_snapshot",
        lambda: {
            "time_ns": 789,
            "axes": [
                {},
                {
                    "native_home_state": 2,
                    "native_home_last_abort_code": 0,
                    "statusword": 0x9650,
                    "error_code": 0,
                    "manufacturer_error_code": 0,
                },
                {},
            ],
        },
    )
    monkeypatch.setattr(
        backend,
        "_wait_for_native_home_post_settle_result",
        lambda *args, **kwargs: {
            "ok": True,
            "timed_out": False,
            "hard_failure": False,
            "failure_reason": None,
            "metrics_time_ns": 987,
            "axis_results": [],
        },
    )
    monkeypatch.setattr(backend, "_refresh_native_home_offsets_from_metrics", lambda: None)
    monkeypatch.setattr(backend, "sync_read_positions", lambda: {0: 0, 1: 25, 2: 0})
    monkeypatch.setattr(
        backend,
        "_capture_absolute_home_anchor_for_joint",
        lambda *args, **kwargs: {"home_anchor_rad": 1.25, "source": "encoder_multi_turn_counts"},
    )
    monkeypatch.setattr(
        backend,
        "_absolute_home_anchor_validation_for_joint",
        lambda *args, **kwargs: {
            "ok": True,
            "truth_available": True,
            "truth_reason": None,
            "axis": 1,
            "logical_joint": 2,
        },
    )

    result = backend.native_home_joint(1)

    assert result["accepted"] is True
    assert result["verified"] is True
    assert result["code"] == "NATIVE_HOME_VERIFIED"
    assert result["post_home_verification_retry_after_timeout"] is True
    assert result["absolute_home_anchor_capture_succeeded"] is True
    assert result["absolute_home_anchor_refresh_ok"] is True
    assert result["metrics_time_ns"] == 789


def test_native_home_metrics_result_accepts_statusword_hm_success_bits_fallback(monkeypatch, tmp_path):
    monkeypatch.setenv("GRADIENT_JOINT_ZERO_OFFSETS_PATH", str(tmp_path / "joint_zero_offsets.json"))
    backend = EthercatRTCoreBackend(robot_config=Gradient05Config().get_config_dict())
    backend._rt_num_axes = 3

    result = backend._native_home_metrics_result(
        [1],
        snapshot={
            "time_ns": 12345,
            "axes": [
                {},
                {
                    "native_home_state": 0,
                    "native_home_last_abort_code": 0,
                    "statusword": 0x9650,
                },
            ],
        },
    )

    assert result["verified"] is True
    assert result["terminal_state"] == "succeeded"
    assert result["native_home_state"] == 2
    assert result["native_home_state_name"] == "succeeded"
    assert result["axis_results"][0]["native_home_state_reported"] == 0
    assert result["axis_results"][0]["verification_source"] == "statusword_bits12_15_clear13"


def test_native_home_metrics_result_requires_bit12_alongside_bit15_for_fallback(
    monkeypatch, tmp_path
):
    monkeypatch.setenv("GRADIENT_JOINT_ZERO_OFFSETS_PATH", str(tmp_path / "joint_zero_offsets.json"))
    backend = EthercatRTCoreBackend(robot_config=Gradient05Config().get_config_dict())
    backend._rt_num_axes = 3

    result = backend._native_home_metrics_result(
        [1],
        snapshot={
            "time_ns": 12345,
            "axes": [
                {},
                {
                    "native_home_state": 3,
                    "native_home_last_abort_code": 0x06010002,
                    "statusword": 0x8650,
                    "error_code": 0,
                    "manufacturer_error_code": 0,
                },
            ],
        },
    )

    assert result["verified"] is False
    assert result["terminal_state"] == "failed"
    assert result["native_home_state"] == 3
    assert result["native_home_last_abort_code"] == 0x06010002
    assert result["axis_results"][0]["verification_source"] == "native_home_state"


def test_native_home_post_settle_result_marks_drive_fault_as_hard_failure(monkeypatch, tmp_path):
    monkeypatch.setenv("GRADIENT_JOINT_ZERO_OFFSETS_PATH", str(tmp_path / "joint_zero_offsets.json"))
    backend = EthercatRTCoreBackend(robot_config=Gradient05Config().get_config_dict())
    backend._rt_num_axes = 3

    result = backend._native_home_post_settle_result(
        [1],
        snapshot={
            "time_ns": 12345,
            "native_home_active_axis_mask": 0,
            "axes": [
                {},
                {
                    "native_home_state": 2,
                    "native_home_last_abort_code": 0,
                    "statusword": 0x9638,
                    "error_code": 0xFF00,
                    "manufacturer_error_code": 0,
                    "slave_online": 1,
                    "slave_operational": 1,
                },
            ],
        },
    )

    assert result["ok"] is False
    assert result["hard_failure"] is True
    assert result["failure_reason"] == "drive_faulted"
    assert result["axis_results"][0]["statusword_fault"] is True
    assert result["axis_results"][0]["error_code"] == 0xFF00


def test_native_home_post_settle_result_accepts_clean_axis(monkeypatch, tmp_path):
    monkeypatch.setenv("GRADIENT_JOINT_ZERO_OFFSETS_PATH", str(tmp_path / "joint_zero_offsets.json"))
    backend = EthercatRTCoreBackend(robot_config=Gradient05Config().get_config_dict())
    backend._rt_num_axes = 3

    result = backend._native_home_post_settle_result(
        [1],
        snapshot={
            "time_ns": 12345,
            "native_home_active_axis_mask": 0,
            "axes": [
                {},
                {
                    "native_home_state": 2,
                    "native_home_last_abort_code": 0,
                    "statusword": 0x1630,
                    "error_code": 0,
                    "manufacturer_error_code": 0,
                    "slave_online": 1,
                    "slave_operational": 1,
                },
            ],
        },
    )

    assert result["ok"] is True
    assert result["hard_failure"] is False
    assert result["failure_reason"] is None
    assert result["axis_results"][0]["clean"] is True


def test_native_home_metrics_result_overrides_stale_failed_report_with_clean_statusword_success(
    monkeypatch, tmp_path
):
    monkeypatch.setenv("GRADIENT_JOINT_ZERO_OFFSETS_PATH", str(tmp_path / "joint_zero_offsets.json"))
    backend = EthercatRTCoreBackend(robot_config=Gradient05Config().get_config_dict())
    backend._rt_num_axes = 3

    result = backend._native_home_metrics_result(
        [1],
        snapshot={
            "time_ns": 12345,
            "axes": [
                {},
                {
                    "native_home_state": 3,
                    "native_home_last_abort_code": 0x06010002,
                    "statusword": 0x9650,
                    "error_code": 0,
                    "manufacturer_error_code": 0,
                },
            ],
        },
    )

    assert result["verified"] is True
    assert result["terminal_state"] == "succeeded"
    assert result["native_home_state"] == 2
    assert result["native_home_last_abort_code"] == 0
    assert result["axis_results"][0]["native_home_state_reported"] == 3
    assert result["axis_results"][0]["native_home_last_abort_code_reported"] == 0x06010002
    assert result["axis_results"][0]["verification_source"] == "statusword_bits12_15_clear13"


def test_native_home_metrics_result_preserves_failed_state_when_live_fault_is_present(monkeypatch, tmp_path):
    monkeypatch.setenv("GRADIENT_JOINT_ZERO_OFFSETS_PATH", str(tmp_path / "joint_zero_offsets.json"))
    backend = EthercatRTCoreBackend(robot_config=Gradient05Config().get_config_dict())
    backend._rt_num_axes = 3

    result = backend._native_home_metrics_result(
        [1],
        snapshot={
            "time_ns": 12345,
            "axes": [
                {},
                {
                    "native_home_state": 3,
                    "native_home_last_abort_code": 0x06010002,
                    "statusword": 0x9650,
                    "error_code": 0x2310,
                    "manufacturer_error_code": 0,
                },
            ],
        },
    )

    assert result["verified"] is False
    assert result["terminal_state"] == "failed"
    assert result["native_home_state"] == 3
    assert result["native_home_last_abort_code"] == 0x06010002
    assert result["axis_results"][0]["verification_source"] == "native_home_state"


def test_wait_for_native_home_result_waits_for_active_mask_clear_before_statusword_fallback(
    monkeypatch, tmp_path
):
    monkeypatch.setenv("GRADIENT_JOINT_ZERO_OFFSETS_PATH", str(tmp_path / "joint_zero_offsets.json"))
    backend = EthercatRTCoreBackend(robot_config=Gradient05Config().get_config_dict())
    backend._rt_num_axes = 3

    snapshots = [
        {
            "time_ns": 1,
            "_mtime_ns": 1,
            "native_home_active_axis_mask": 0x2,
            "axes": [{}, {"native_home_state": 0, "native_home_last_abort_code": 0, "statusword": 0x9650}],
        },
        {
            "time_ns": 2,
            "_mtime_ns": 2,
            "native_home_active_axis_mask": 0x0,
            "axes": [{}, {"native_home_state": 0, "native_home_last_abort_code": 0, "statusword": 0x9650}],
        },
    ]
    snapshot_iter = iter(snapshots)
    last_snapshot = snapshots[-1]
    monkeypatch.setattr(
        backend,
        "_load_rtcore_metrics_snapshot",
        lambda: next(snapshot_iter, last_snapshot),
    )
    monotonic_values = iter([0.0, 0.0, 0.1, 0.2])
    monkeypatch.setattr(rtcore_backend_module.time, "monotonic", lambda: next(monotonic_values))
    monkeypatch.setattr(rtcore_backend_module.time, "sleep", lambda _delay: None)

    result = backend._wait_for_native_home_result(0x2, timeout_s=1.0)

    assert result["verified"] is True
    assert result["terminal_state"] == "succeeded"
    assert result["axis_results"][0]["verification_source"] == "statusword_bits12_15_clear13"


def test_wait_for_native_home_result_ignores_stale_failed_report_before_active_mask_seen(
    monkeypatch, tmp_path
):
    monkeypatch.setenv("GRADIENT_JOINT_ZERO_OFFSETS_PATH", str(tmp_path / "joint_zero_offsets.json"))
    backend = EthercatRTCoreBackend(robot_config=Gradient05Config().get_config_dict())
    backend._rt_num_axes = 3

    snapshots = [
        {
            "time_ns": 1,
            "_mtime_ns": 1,
            "native_home_active_axis_mask": 0x0,
            "axes": [
                {},
                {
                    "native_home_state": 3,
                    "native_home_last_abort_code": 0x06010002,
                    "statusword": 0x9650,
                    "error_code": 0,
                    "manufacturer_error_code": 0,
                },
            ],
        },
        {
            "time_ns": 2,
            "_mtime_ns": 2,
            "native_home_active_axis_mask": 0x0,
            "axes": [
                {},
                {
                    "native_home_state": 3,
                    "native_home_last_abort_code": 0x06010002,
                    "statusword": 0x9650,
                    "error_code": 0,
                    "manufacturer_error_code": 0,
                },
            ],
        },
    ]
    snapshot_iter = iter(snapshots)
    last_snapshot = snapshots[-1]
    monkeypatch.setattr(
        backend,
        "_load_rtcore_metrics_snapshot",
        lambda: next(snapshot_iter, last_snapshot),
    )
    monotonic_values = iter([0.0, 0.0, 0.5, 1.1])
    monkeypatch.setattr(rtcore_backend_module.time, "monotonic", lambda: next(monotonic_values))
    monkeypatch.setattr(rtcore_backend_module.time, "sleep", lambda _delay: None)

    result = backend._wait_for_native_home_result(0x2, timeout_s=1.0)

    assert result["verified"] is False
    assert result["timed_out"] is True
    assert result["terminal_state"] == "pending"
    assert result["native_home_state"] == 1
    assert result["native_home_state_name"] == "requested"
    assert result["native_home_last_abort_code"] == 0
    assert result["metrics_time_ns"] == 2
    assert result["axis_results"][0]["native_home_state"] == 3
    assert result["axis_results"][0]["native_home_last_abort_code"] == 0x06010002


def test_wait_for_native_home_result_reports_failed_after_active_mask_seen(
    monkeypatch, tmp_path
):
    monkeypatch.setenv("GRADIENT_JOINT_ZERO_OFFSETS_PATH", str(tmp_path / "joint_zero_offsets.json"))
    backend = EthercatRTCoreBackend(robot_config=Gradient05Config().get_config_dict())
    backend._rt_num_axes = 3

    snapshots = [
        {
            "time_ns": 1,
            "_mtime_ns": 1,
            "native_home_active_axis_mask": 0x2,
            "axes": [{}, {"native_home_state": 1, "native_home_last_abort_code": 0, "statusword": 0x0233}],
        },
        {
            "time_ns": 2,
            "_mtime_ns": 2,
            "native_home_active_axis_mask": 0x0,
            "axes": [
                {},
                {
                    "native_home_state": 3,
                    "native_home_last_abort_code": 0x06010002,
                    "statusword": 0x0250,
                    "error_code": 0,
                    "manufacturer_error_code": 0,
                },
            ],
        },
        {
            "time_ns": 3,
            "_mtime_ns": 3,
            "native_home_active_axis_mask": 0x0,
            "axes": [
                {},
                {
                    "native_home_state": 3,
                    "native_home_last_abort_code": 0x06010002,
                    "statusword": 0x0250,
                    "error_code": 0,
                    "manufacturer_error_code": 0,
                },
            ],
        },
    ]
    snapshot_iter = iter(snapshots)
    last_snapshot = snapshots[-1]
    monkeypatch.setattr(
        backend,
        "_load_rtcore_metrics_snapshot",
        lambda: next(snapshot_iter, last_snapshot),
    )
    monotonic_values = iter([0.0, 0.0, 0.1, 0.2, 0.3, 0.4])
    monkeypatch.setattr(rtcore_backend_module.time, "monotonic", lambda: next(monotonic_values))
    monkeypatch.setattr(rtcore_backend_module.time, "sleep", lambda _delay: None)

    result = backend._wait_for_native_home_result(0x2, timeout_s=1.0)

    assert result["verified"] is False
    assert result["timed_out"] is False
    assert result["terminal_state"] == "failed"
    assert result["native_home_state"] == 3
    assert result["native_home_last_abort_code"] == 0x06010002
    assert result["axis_results"][0]["verification_source"] == "native_home_state"


def test_wait_for_native_home_result_recovers_when_clean_success_follows_single_failed_snapshot_post_clear(
    monkeypatch, tmp_path
):
    monkeypatch.setenv("GRADIENT_JOINT_ZERO_OFFSETS_PATH", str(tmp_path / "joint_zero_offsets.json"))
    backend = EthercatRTCoreBackend(robot_config=Gradient05Config().get_config_dict())
    backend._rt_num_axes = 3

    snapshots = [
        {
            "time_ns": 1,
            "_mtime_ns": 1,
            "native_home_active_axis_mask": 0x2,
            "axes": [{}, {"native_home_state": 1, "native_home_last_abort_code": 0, "statusword": 0x0233}],
        },
        {
            "time_ns": 2,
            "_mtime_ns": 2,
            "native_home_active_axis_mask": 0x0,
            "axes": [
                {},
                {
                    "native_home_state": 3,
                    "native_home_last_abort_code": 0x06010002,
                    "statusword": 0x0250,
                    "error_code": 0,
                    "manufacturer_error_code": 0,
                },
            ],
        },
        {
            "time_ns": 3,
            "_mtime_ns": 3,
            "native_home_active_axis_mask": 0x0,
            "axes": [
                {},
                {
                    "native_home_state": 3,
                    "native_home_last_abort_code": 0x06010002,
                    "statusword": 0x9650,
                    "error_code": 0,
                    "manufacturer_error_code": 0,
                },
            ],
        },
    ]
    snapshot_iter = iter(snapshots)
    last_snapshot = snapshots[-1]
    monkeypatch.setattr(
        backend,
        "_load_rtcore_metrics_snapshot",
        lambda: next(snapshot_iter, last_snapshot),
    )
    monotonic_values = iter([0.0, 0.0, 0.1, 0.2, 0.3, 0.4])
    monkeypatch.setattr(rtcore_backend_module.time, "monotonic", lambda: next(monotonic_values))
    monkeypatch.setattr(rtcore_backend_module.time, "sleep", lambda _delay: None)

    result = backend._wait_for_native_home_result(0x2, timeout_s=1.0)

    assert result["verified"] is True
    assert result["timed_out"] is False
    assert result["terminal_state"] == "succeeded"
    assert result["native_home_state"] == 2
    assert result["native_home_last_abort_code"] == 0
    assert result["axis_results"][0]["native_home_state_reported"] == 3
    assert result["axis_results"][0]["native_home_last_abort_code_reported"] == 0x06010002
    assert result["axis_results"][0]["verification_source"] == "statusword_bits12_15_clear13"


def test_wait_for_native_home_result_accepts_stale_failed_report_after_active_mask_clears(
    monkeypatch, tmp_path
):
    monkeypatch.setenv("GRADIENT_JOINT_ZERO_OFFSETS_PATH", str(tmp_path / "joint_zero_offsets.json"))
    backend = EthercatRTCoreBackend(robot_config=Gradient05Config().get_config_dict())
    backend._rt_num_axes = 3

    snapshots = [
        {
            "time_ns": 1,
            "_mtime_ns": 1,
            "native_home_active_axis_mask": 0x2,
            "axes": [{}, {"native_home_state": 1, "native_home_last_abort_code": 0, "statusword": 0x0250}],
        },
        {
            "time_ns": 2,
            "_mtime_ns": 2,
            "native_home_active_axis_mask": 0x0,
            "axes": [
                {},
                {
                    "native_home_state": 3,
                    "native_home_last_abort_code": 0x06010002,
                    "statusword": 0x9650,
                    "error_code": 0,
                    "manufacturer_error_code": 0,
                },
            ],
        },
    ]
    snapshot_iter = iter(snapshots)
    last_snapshot = snapshots[-1]
    monkeypatch.setattr(
        backend,
        "_load_rtcore_metrics_snapshot",
        lambda: next(snapshot_iter, last_snapshot),
    )
    monotonic_values = iter([0.0, 0.0, 0.1, 0.2])
    monkeypatch.setattr(rtcore_backend_module.time, "monotonic", lambda: next(monotonic_values))
    monkeypatch.setattr(rtcore_backend_module.time, "sleep", lambda _delay: None)

    result = backend._wait_for_native_home_result(0x2, timeout_s=1.0)

    assert result["verified"] is True
    assert result["terminal_state"] == "succeeded"
    assert result["native_home_state"] == 2
    assert result["native_home_last_abort_code"] == 0
    assert result["axis_results"][0]["native_home_state_reported"] == 3
    assert result["axis_results"][0]["native_home_last_abort_code_reported"] == 0x06010002
    assert result["axis_results"][0]["verification_source"] == "statusword_bits12_15_clear13"


def test_ethercat_backend_applies_native_home_offsets_to_feedback_but_not_command_targets(
    monkeypatch, tmp_path
):
    monkeypatch.setenv("GRADIENT_JOINT_ZERO_OFFSETS_PATH", str(tmp_path / "joint_zero_offsets.json"))
    metrics_path = tmp_path / "metrics.json"
    monkeypatch.setattr(rtcore_backend_module, "_RTCORE_METRICS_PATH", metrics_path)
    _write_rtcore_metrics_snapshot(
        metrics_path,
        [
            {
                "native_home_position_offset": -25,
                "absolute_feedback": {
                    "encoder_multi_turn_low": {"valid": 1, "value": 0},
                    "encoder_multi_turn_high": {"valid": 1, "value": 0},
                },
            }
        ],
    )
    backend = _force_legacy_truth_fallback(
        EthercatRTCoreBackend(robot_config=Gradient05Config().get_config_dict())
    )
    backend._axis_to_joint = [0]
    backend._rt_num_axes = 1
    backend._axis_config = _AxisConfig(
        num_axes=1,
        counts_per_unit=[100.0] + [0.0] * 15,
        sign=[1] + [0] * 15,
    )
    backend._native_home_offset_counts[0] = -25
    backend._rt_drive_profile_id = "a6ec_ds402"
    backend._absolute_encoder_home_anchors[0] = {"home_anchor_rad": 0.0, "source": "pytest", "axis_indices": [0]}

    logical_positions = backend.raw_to_joint_positions({0: 25})
    commanded_axis_q = backend._axis_q_from_joint_positions([0.0] * backend.num_joints)

    assert logical_positions[0] == pytest.approx(0.0)
    assert commanded_axis_q[0] == pytest.approx(0.0)


def test_a6ec_midpoint_native_home_offset_cancels_raw_midpoint_reference(monkeypatch, tmp_path):
    monkeypatch.setenv("GRADIENT_JOINT_ZERO_OFFSETS_PATH", str(tmp_path / "joint_zero_offsets.json"))
    metrics_path = tmp_path / "metrics.json"
    monkeypatch.setattr(rtcore_backend_module, "_RTCORE_METRICS_PATH", metrics_path)
    counts_per_rev = 131072
    gear_ratio = 10.0
    counts_per_unit = (float(counts_per_rev) * gear_ratio) / (2.0 * math.pi)
    rm = int(round(counts_per_unit * 2.0 * math.pi))
    half_rm = rm // 2
    _write_rtcore_metrics_snapshot(
        metrics_path,
        [
            {
                "native_home_position_offset": -half_rm,
                "absolute_feedback": {
                    "encoder_multi_turn_low": {"valid": 1, "value": 35771},
                    "encoder_multi_turn_high": {"valid": 1, "value": 0},
                },
            }
        ],
    )
    backend = _force_legacy_truth_fallback(
        EthercatRTCoreBackend(robot_config=Gradient05Config().get_config_dict())
    )
    backend._axis_to_joint = [0]
    backend._rt_num_axes = 1
    backend._axis_config = _AxisConfig(
        num_axes=1,
        counts_per_unit=[counts_per_unit] + [0.0] * 15,
        sign=[-1] + [0] * 15,
        counts_per_rev=[counts_per_rev] + [0] * 15,
    )
    backend._rt_drive_profile_id = "a6ec_ds402"

    reference_q = backend._reference_q_before_master_offset_for_axis(
        0,
        half_rm,
        reference_mode="raw",
    )

    assert reference_q == pytest.approx(0.0, abs=1e-9)


def test_a6ec_shaft_frame_consistency_uses_logicalized_live_reference_counts(monkeypatch, tmp_path):
    monkeypatch.setenv("GRADIENT_JOINT_ZERO_OFFSETS_PATH", str(tmp_path / "joint_zero_offsets.json"))
    metrics_path = tmp_path / "metrics.json"
    monkeypatch.setattr(rtcore_backend_module, "_RTCORE_METRICS_PATH", metrics_path)
    counts_per_rev = 131072
    gear_ratio = 10.0
    counts_per_unit = (float(counts_per_rev) * gear_ratio) / (2.0 * math.pi)
    rm = int(round(counts_per_unit * 2.0 * math.pi))
    half_rm = rm // 2
    _write_rtcore_metrics_snapshot(
        metrics_path,
        [
            {
                "native_home_position_offset": -half_rm,
                "absolute_feedback": {
                    "encoder_multi_turn_low": {"valid": 1, "value": 35771},
                    "encoder_multi_turn_high": {"valid": 1, "value": 0},
                },
            }
        ],
    )
    backend = _force_legacy_truth_fallback(
        EthercatRTCoreBackend(robot_config=Gradient05Config().get_config_dict())
    )
    backend._axis_to_joint = [0]
    backend._rt_num_axes = 1
    backend._axis_config = _AxisConfig(
        num_axes=1,
        counts_per_unit=[counts_per_unit] + [0.0] * 15,
        sign=[-1] + [0] * 15,
        counts_per_rev=[counts_per_rev] + [0] * 15,
    )
    backend._rt_drive_profile_id = "a6ec_ds402"

    detail = backend._shaft_frame_consistency_detail(
        axis_i=0,
        canonical_q=0.0,
        logical_joint_idx=0,
        live_reference_counts=half_rm,
    )

    assert detail is not None
    assert detail["shaft_frame_consistent"] is True
    assert detail["shaft_frame_mod_rm_delta_counts"] == pytest.approx(0.0)
    assert detail["shaft_frame_live_reference_logical_counts"] == pytest.approx(0.0)


def test_command_roundtrip_tolerance_widens_with_velocity(monkeypatch, tmp_path):
    """2026-04-21 velocity-aware command-roundtrip tolerance: at rest the
    tolerance stays at the tight 15-count base, but under motion it
    widens proportionally to |velocity_counts_per_s| × the motion skew
    budget so the drive's servo-loop following error (PLUS the
    canonical_q-vs-reference_q sample-moment skew) no longer trips
    `drive_native_command_frame_roundtrip_mismatch` on every motion
    start. Pinning the relationship here keeps future tuning explicit
    and prevents silently reverting the widening."""
    monkeypatch.setenv("GRADIENT_JOINT_ZERO_OFFSETS_PATH", str(tmp_path / "joint_zero_offsets.json"))
    metrics_path = tmp_path / "metrics.json"
    monkeypatch.setattr(rtcore_backend_module, "_RTCORE_METRICS_PATH", metrics_path)
    counts_per_rev = 131072
    gear_ratio = 10.0
    counts_per_unit = (float(counts_per_rev) * gear_ratio) / (2.0 * math.pi)
    _write_rtcore_metrics_snapshot(metrics_path, [{}])
    backend = _force_legacy_truth_fallback(
        EthercatRTCoreBackend(robot_config=Gradient05Config().get_config_dict())
    )
    backend._axis_to_joint = [0]
    backend._rt_num_axes = 1
    backend._axis_config = _AxisConfig(
        num_axes=1,
        counts_per_unit=[counts_per_unit] + [0.0] * 15,
        sign=[1] + [0] * 15,
        counts_per_rev=[counts_per_rev] + [0] * 15,
    )

    # At rest the tolerance equals the tight 15-count base (plus the
    # fixed 1e-9 rad safety pad that `_counts_tolerance_rad_for_axis`
    # always adds so zero-counts tolerances never underflow).
    tol_rest_rad = backend._command_roundtrip_tolerance_rad_for_axis(
        0, velocity_counts_per_s=0.0
    )
    base_counts_rad = (
        float(rtcore_backend_module._COMMAND_ROUNDTRIP_TOLERANCE_COUNTS)
        / float(counts_per_unit)
    ) + 1e-9
    assert tol_rest_rad == pytest.approx(base_counts_rad, abs=1e-15)

    # With a velocity estimate, widen by |v| * skew_budget (counts-frame).
    vel_counts_per_s = 20000.0
    skew_budget = float(rtcore_backend_module._COMMAND_ROUNDTRIP_MOTION_SKEW_BUDGET_S)
    tol_motion_rad = backend._command_roundtrip_tolerance_rad_for_axis(
        0, velocity_counts_per_s=vel_counts_per_s
    )
    expected_total_counts = (
        float(rtcore_backend_module._COMMAND_ROUNDTRIP_TOLERANCE_COUNTS)
        + abs(vel_counts_per_s) * skew_budget
    )
    expected_rad = (expected_total_counts / float(counts_per_unit)) + 1e-9
    assert tol_motion_rad == pytest.approx(expected_rad, rel=1e-9)
    # Widened tolerance must be DECISIVELY larger than rest tolerance —
    # a handful of counts would just be noise. Require at least 10x.
    assert tol_motion_rad > 10.0 * tol_rest_rad, (
        "Motion-widened tolerance MUST be substantially larger than "
        "rest-state tolerance, else the widening is ineffective and "
        "jog sessions will still trip the roundtrip gate on every "
        "acceleration."
    )

    # Sign of velocity does not matter — widening is always absolute.
    tol_negative_vel_rad = backend._command_roundtrip_tolerance_rad_for_axis(
        0, velocity_counts_per_s=-vel_counts_per_s
    )
    assert tol_negative_vel_rad == pytest.approx(tol_motion_rad, rel=1e-9)


def test_command_roundtrip_detail_includes_velocity_field(monkeypatch, tmp_path):
    """The velocity estimate must surface on the diagnostic detail dict
    so operators can confirm from `/info/joints-detailed` whether the
    widening is active during a live jog. If the field disappears,
    debugging a future flicker regression becomes much harder."""
    monkeypatch.setenv("GRADIENT_JOINT_ZERO_OFFSETS_PATH", str(tmp_path / "joint_zero_offsets.json"))
    metrics_path = tmp_path / "metrics.json"
    monkeypatch.setattr(rtcore_backend_module, "_RTCORE_METRICS_PATH", metrics_path)
    counts_per_rev = 131072
    gear_ratio = 10.0
    counts_per_unit = (float(counts_per_rev) * gear_ratio) / (2.0 * math.pi)
    _write_rtcore_metrics_snapshot(metrics_path, [{}])
    backend = _force_legacy_truth_fallback(
        EthercatRTCoreBackend(robot_config=Gradient05Config().get_config_dict())
    )
    backend._axis_to_joint = [0]
    backend._rt_num_axes = 1
    backend._axis_config = _AxisConfig(
        num_axes=1,
        counts_per_unit=[counts_per_unit] + [0.0] * 15,
        sign=[1] + [0] * 15,
        counts_per_rev=[counts_per_rev] + [0] * 15,
    )
    backend._rt_drive_profile_id = "a6ec_ds402"

    detail = backend._command_roundtrip_detail_for_axis(
        axis_i=0,
        logical_joint_idx=0,
        canonical_q=0.0,
        reference_q=0.0,
        reference_mode="raw",
        velocity_counts_per_s=12345.0,
    )

    assert "command_roundtrip_velocity_counts_per_s" in detail
    assert detail["command_roundtrip_velocity_counts_per_s"] == pytest.approx(12345.0)
    assert detail["command_roundtrip_consistent"] is True


def test_ethercat_backend_enqueue_trajectory_points_keeps_controller_logical_frame_with_native_home_offset(
    monkeypatch, tmp_path
):
    monkeypatch.setenv("GRADIENT_JOINT_ZERO_OFFSETS_PATH", str(tmp_path / "joint_zero_offsets.json"))
    backend = EthercatRTCoreBackend(robot_config=Gradient05Config().get_config_dict())
    backend._connected = True
    backend._rt_num_axes = 1
    backend._axis_to_joint = [0]
    backend._master_offsets_rad[0] = 0.5
    backend._native_home_offset_counts[0] = -25
    monkeypatch.setattr(backend, "_refresh_native_home_offsets_from_metrics", lambda: None)

    # Stage live 6064 to match the commanded point so the pre-commit
    # wire-frame safety cage (which blocks first-point teleports) does
    # not reject this synthetic single-point upload. The test checks
    # frame conversion, not operator motion safety.
    robot_cfg = Gradient05Config().get_config_dict()
    axis0_cpu = (
        float(robot_cfg["actuator_encoder_counts_per_rev"][0])
        * float(robot_cfg["actuator_gear_ratios"][0])
        / (2.0 * math.pi)
    )
    axis0_sign = int(robot_cfg["actuator_position_signs"][0])
    staged_axis_q = 1.0 + 0.5  # positions_rad[0] + master_offset
    backend._axis_counts[0] = int(round(staged_axis_q * axis0_sign * axis0_cpu))

    captured: list[tuple[int, bytes]] = []
    monkeypatch.setattr(
        backend,
        "_cmd_ring_write",
        lambda msg_type, payload: captured.append((msg_type, payload)) or 1,
    )

    backend.enqueue_trajectory_points(
        7,
        [
            {
                "positions_rad": [1.0] + ([0.0] * (backend.num_joints - 1)),
                "qd": [0.25],
                "flags": _TRAJ_POINTF_HAS_VELOCITY,
                "t_from_start_ns": 12_000_000,
            }
        ],
    )

    assert len(captured) == 1
    _msg_type, payload = captured[0]
    unpacked = _TRAJECTORY_POINT_STRUCT.unpack(payload)
    q_values = unpacked[4:20]
    qd_values = unpacked[20:36]
    axis_mask = unpacked[36]

    # Python uploads controller/logical q values. RTCore converts them once into
    # raw CSP wire counts before writing 0x607A.
    assert q_values[0] == pytest.approx(1.5)
    assert qd_values[0] == pytest.approx(0.25)
    assert axis_mask == 0x1


def test_ethercat_backend_enqueue_trajectory_points_keeps_nonunit_a6ec_axis_in_logical_radians(
    monkeypatch, tmp_path
):
    monkeypatch.setenv("GRADIENT_JOINT_ZERO_OFFSETS_PATH", str(tmp_path / "joint_zero_offsets.json"))
    robot_cfg = Gradient05Config().get_config_dict()
    backend = EthercatRTCoreBackend(robot_config=robot_cfg)
    backend._connected = True
    backend._rt_num_axes = 1
    backend._axis_to_joint = [4]
    neutral_counts_per_unit = 131072.0 / (2.0 * 3.141592653589793)
    legacy_counts_per_unit = neutral_counts_per_unit * float(robot_cfg["actuator_gear_ratios"][4])
    assert backend._robot_axis_config is not None
    assert backend._robot_axis_config.counts_per_unit[4] == pytest.approx(legacy_counts_per_unit)
    assert legacy_counts_per_unit != pytest.approx(neutral_counts_per_unit)
    backend._axis_config = _AxisConfig(
        num_axes=1,
        counts_per_unit=[legacy_counts_per_unit] + [0.0] * 15,
        sign=[-1] + [0] * 15,
        counts_per_rev=[131072] + [0] * 15,
    )
    monkeypatch.setattr(backend, "_refresh_native_home_offsets_from_metrics", lambda: None)

    captured: list[tuple[int, bytes]] = []
    monkeypatch.setattr(
        backend,
        "_cmd_ring_write",
        lambda msg_type, payload: captured.append((msg_type, payload)) or 1,
    )

    commanded_joint_q = 0.4
    # Stage live 6064 to match the commanded point so the pre-commit
    # wire-frame safety cage (which blocks first-point teleports) does
    # not reject this synthetic single-point upload. The test checks
    # frame conversion, not operator motion safety.
    backend._axis_counts[0] = int(round(commanded_joint_q * -1 * legacy_counts_per_unit))
    backend.enqueue_trajectory_points(
        11,
        [
            {
                "positions_rad": [0.0, 0.0, 0.0, 0.0, commanded_joint_q, 0.0],
                "t_from_start_ns": 1_000_000,
            }
        ],
    )

    assert len(captured) == 1
    _msg_type, payload = captured[0]
    unpacked = _TRAJECTORY_POINT_STRUCT.unpack(payload)
    q_values = unpacked[4:20]
    axis_mask = unpacked[36]

    expected_neutral_counts = round(commanded_joint_q * neutral_counts_per_unit * -1.0)
    expected_legacy_counts = round(commanded_joint_q * legacy_counts_per_unit * -1.0)

    assert axis_mask == 0x1
    assert q_values[0] == pytest.approx(commanded_joint_q)
    assert round(q_values[0] * legacy_counts_per_unit * -1.0) == expected_legacy_counts
    assert expected_neutral_counts != expected_legacy_counts


def test_ethercat_backend_j6_display_feedback_uses_rotation_mode_ratio_scaling(monkeypatch, tmp_path):
    monkeypatch.setenv("GRADIENT_JOINT_ZERO_OFFSETS_PATH", str(tmp_path / "joint_zero_offsets.json"))
    backend = EthercatRTCoreBackend(robot_config=Gradient05Config().get_config_dict())
    backend._rt_drive_profile_id = "a6ec_ds402"
    assert backend._robot_axis_config is not None
    backend._axis_config = backend._robot_axis_config

    display_counts = backend._display_feedback_counts_for_axis(5, 1310650)
    display_axis_q = backend._display_axis_q_from_raw_feedback_counts(5, 1310650)

    j6_counts_per_unit = backend._robot_axis_config.counts_per_unit[5]
    expected_axis_q = float(display_counts) / (-1.0 * float(j6_counts_per_unit))

    assert display_counts == -70
    assert display_axis_q == pytest.approx(expected_axis_q, abs=1e-12)
    assert display_axis_q == pytest.approx(0.000335558297349984, abs=1e-12)


def test_ethercat_backend_a6ec_reference_wrap_period_uses_per_axis_gear_ratios(
    monkeypatch, tmp_path
):
    monkeypatch.setenv("GRADIENT_JOINT_ZERO_OFFSETS_PATH", str(tmp_path / "joint_zero_offsets.json"))
    robot_cfg = Gradient05Config().get_config_dict()
    backend = EthercatRTCoreBackend(robot_config=robot_cfg)
    backend._rt_drive_profile_id = "a6ec_ds402"
    assert backend._robot_axis_config is not None
    backend._axis_config = backend._robot_axis_config

    expected_period_counts = [
        int(round(float(counts_per_rev) * float(gear_ratio)))
        for counts_per_rev, gear_ratio in zip(
            robot_cfg["actuator_encoder_counts_per_rev"][:6],
            robot_cfg["actuator_gear_ratios"][:6],
        )
    ]
    assert expected_period_counts[4] == 4_096_000

    for axis_i, expected_period in enumerate(expected_period_counts):
        counts_per_unit = float(backend._axis_config.counts_per_unit[axis_i])
        assert int(round(counts_per_unit * (2.0 * math.pi))) == expected_period
        assert backend._reference_wrap_period_counts_for_axis(axis_i) == expected_period


def test_ethercat_backend_power_transition_snapshot_reports_coordinate_system_invalid(
    monkeypatch, tmp_path
):
    monkeypatch.setenv("GRADIENT_JOINT_ZERO_OFFSETS_PATH", str(tmp_path / "joint_zero_offsets.json"))
    # Isolate the absolute-home anchor store so the test does not pick up
    # real anchor data from the repo's live encoder-retention workflow; the
    # intent of this test is specifically the "no usable home state on any
    # axis" scenario.
    monkeypatch.setenv(
        "GRADIENT_ABSOLUTE_ENCODER_ANCHORS_PATH",
        str(tmp_path / "absolute_encoder_anchors.json"),
    )
    robot_cfg = Gradient05Config().get_config_dict()
    backend = EthercatRTCoreBackend(robot_config=robot_cfg)
    backend._connected = True
    backend._rt_drive_profile_id = "a6ec_ds402"
    backend._rt_num_axes = 6
    backend._axis_to_joint = [0, 1, 2, 3, 4, 5]
    assert backend._robot_axis_config is not None
    backend._axis_config = backend._robot_axis_config
    backend._status_snapshot_event.set()
    monkeypatch.setattr(backend, "_refresh_native_home_offsets_from_metrics", lambda: None)

    raw_counts = [5, 4408, 13105469, 2359290, 138, 1310651]
    metrics_axes: list[dict[str, object]] = []
    for axis_i, (counts, gear_ratio) in enumerate(
        zip(raw_counts, robot_cfg["actuator_gear_ratios"][:6], strict=False)
    ):
        backend._axis_counts[axis_i] = int(counts)
        backend._absolute_feedback_by_axis[axis_i] = _AbsoluteFeedbackAxisMetrics.from_mapping(
            {
                "absolute_position_reference": {"valid": 1, "value": int(counts)},
                "rotation_mode_position_reference": {"valid": 1, "value": int(counts)},
                "rotation_mode_encoder_low": {"valid": 1, "value": int(counts)},
                "rotation_mode_encoder_high": {"valid": 1, "value": 0},
            }
        )
        metrics_axes.append(
            {
                "statusword": 0x1650,
                "error_code": 0,
                "manufacturer_error_code": 0,
                "startup_drive_config": _a6ec_startup_drive_configs_for_ratio(gear_ratio)[0],
                "startup_drive_configs": _a6ec_startup_drive_configs_for_ratio(gear_ratio),
                "slave_online": 1,
                "slave_operational": 1,
                "slave_al_state": 8,
            }
        )
    monkeypatch.setattr(backend, "_load_rtcore_metrics_snapshot", lambda: {"axes": metrics_axes})

    snapshot = backend.get_power_transition_snapshot()

    assert snapshot["feedback_synchronized"] is False
    assert snapshot["feedback_truth_available"] is False
    assert snapshot["live_feedback_joint_positions_rad"] == []
    # The validity helper now surfaces the narrow reason. Accept the
    # generic legacy reason too so older installations without the
    # persisted-home-anchor restart trust flag keep working.
    assert snapshot["feedback_truth_reasons"] in (
        ["drive_native_coordinate_system_invalid"],
        ["drive_native_persisted_home_anchor_missing"],
    )
    assert snapshot["feedback_truth_unavailable_axes"] == [0, 1, 2, 3, 4, 5]
    assert snapshot["feedback_truth_unavailable_joints"] == [1, 2, 3, 4, 5, 6]
    assert snapshot["feedback_truth_statuswords"] == ["0x1650"]


def test_ethercat_backend_power_transition_snapshot_accepts_bit15_restart_truth(
    monkeypatch, tmp_path
):
    monkeypatch.setenv("GRADIENT_JOINT_ZERO_OFFSETS_PATH", str(tmp_path / "joint_zero_offsets.json"))
    monkeypatch.setenv(
        "GRADIENT_ABSOLUTE_ENCODER_ANCHORS_PATH",
        str(tmp_path / "absolute_encoder_anchors.json"),
    )
    robot_cfg = Gradient05Config().get_config_dict()
    backend = EthercatRTCoreBackend(robot_config=robot_cfg)
    backend._connected = True
    backend._rt_drive_profile_id = "a6ec_ds402"
    backend._rt_num_axes = 6
    backend._axis_to_joint = [0, 1, 2, 3, 4, 5]
    assert backend._robot_axis_config is not None
    backend._axis_config = backend._robot_axis_config
    backend._status_snapshot_event.set()
    monkeypatch.setattr(backend, "_refresh_native_home_offsets_from_metrics", lambda: None)

    raw_counts = [5, 4408, 13105469, 2359290, 138, 1310651]
    metrics_axes: list[dict[str, object]] = []
    for axis_i, (counts, gear_ratio) in enumerate(
        zip(raw_counts, robot_cfg["actuator_gear_ratios"][:6], strict=False)
    ):
        backend._axis_counts[axis_i] = int(counts)
        backend._absolute_encoder_home_anchors[axis_i] = {
            "home_anchor_rad": 0.0,
            "source": "pytest",
            "axis_indices": [axis_i],
        }
        backend._absolute_feedback_by_axis[axis_i] = _AbsoluteFeedbackAxisMetrics.from_mapping(
            {
                "encoder_multi_turn_low": {"valid": 1, "value": int(counts)},
                "encoder_multi_turn_high": {"valid": 1, "value": 0},
                "absolute_position_reference": {"valid": 1, "value": int(counts)},
                "rotation_mode_position_reference": {"valid": 1, "value": int(counts)},
                "rotation_mode_encoder_low": {"valid": 1, "value": int(counts)},
                "rotation_mode_encoder_high": {"valid": 1, "value": 0},
            }
        )
        metrics_axes.append(
            {
                "statusword": 0x8650,
                "error_code": 0,
                "manufacturer_error_code": 0,
                "startup_drive_config": _a6ec_startup_drive_configs_for_ratio(gear_ratio)[0],
                "startup_drive_configs": _a6ec_startup_drive_configs_for_ratio(gear_ratio),
                "slave_online": 1,
                "slave_operational": 1,
                "slave_al_state": 8,
            }
        )
    monkeypatch.setattr(backend, "_load_rtcore_metrics_snapshot", lambda: {"axes": metrics_axes})

    snapshot = backend.get_power_transition_snapshot()

    assert snapshot["feedback_synchronized"] is True
    assert snapshot["feedback_truth_available"] is True
    assert len(snapshot["live_feedback_joint_positions_rad"]) == 6
    assert snapshot["feedback_truth_reasons"] == []
    assert snapshot["feedback_truth_statuswords"] == []
    assert snapshot["power_up_ready"] is True


def _build_a6ec_restart_trust_test_backend(
    monkeypatch,
    tmp_path,
    *,
    joint_index: int = 5,
    statusword: int = 0x1650,
    hm35_6064_counts: int = 8,
    hm35_u40_20_counts: int = 120191,
    live_6064_counts: int | None = None,
    live_u40_20_counts: int | None = None,
    multi_turn_valid: bool = True,
    encoder_retention_fault_code: int = 0,
    last_seen_absolute_counts: int | None = None,
    last_seen_reference_counts: int | None = None,
) -> tuple[EthercatRTCoreBackend, dict[str, object]]:
    """Shared scaffold for A6-EC restart-trust scenarios on any single joint.

    The caller supplies the 6064 + U40.20 raw counts that were present at
    HM35 time (used to derive the stored absolute-home anchor) and the
    raw counts the drive is reporting now, after the power cycle. The
    helper sets up the same axis plumbing the bit-15 restart-trust test
    uses, so the persisted-home-anchor gate evaluates realistic live data.

    ``joint_index`` defaults to ``5`` (J6) so the original four 2026-04-17
    tests keep their semantics; parametrised coverage passes other joint
    indices to catch per-joint scaling / sign mistakes in the path.
    """
    monkeypatch.setenv("GRADIENT_JOINT_ZERO_OFFSETS_PATH", str(tmp_path / "joint_zero_offsets.json"))
    monkeypatch.setenv(
        "GRADIENT_ABSOLUTE_ENCODER_ANCHORS_PATH",
        str(tmp_path / "absolute_encoder_anchors.json"),
    )
    robot_cfg = Gradient05Config().get_config_dict()
    backend = EthercatRTCoreBackend(robot_config=robot_cfg)
    backend._connected = True
    backend._rt_drive_profile_id = "a6ec_ds402"
    backend._rt_num_axes = 6
    backend._axis_to_joint = [0, 1, 2, 3, 4, 5]
    assert backend._robot_axis_config is not None
    backend._axis_config = backend._robot_axis_config
    backend._status_snapshot_event.set()
    monkeypatch.setattr(backend, "_refresh_native_home_offsets_from_metrics", lambda: None)

    # Compute the anchor from the HM35-time raw counts using the same math
    # the live code uses. This keeps the test honest across future cpu/sign
    # changes instead of hardcoding a float, and tracks the caller-selected
    # joint's scaling automatically.
    cpu_target = float(robot_cfg["actuator_encoder_counts_per_rev"][joint_index]) * float(
        robot_cfg["actuator_gear_ratios"][joint_index]
    ) / (2.0 * math.pi)
    sign_target = float(robot_cfg["actuator_position_signs"][joint_index])
    absolute_axis_q_at_hm35 = float(hm35_u40_20_counts) / (sign_target * cpu_target)
    reference_q_at_hm35 = float(hm35_6064_counts) / (sign_target * cpu_target)
    anchor_rad = absolute_axis_q_at_hm35 - reference_q_at_hm35

    effective_live_6064 = int(
        live_6064_counts if live_6064_counts is not None else hm35_6064_counts
    )
    effective_live_u40_20 = int(
        live_u40_20_counts if live_u40_20_counts is not None else hm35_u40_20_counts
    )

    # Baseline realistic live counts pulled from the A6-EC chapter-5 probe
    # dataset; the non-target axes stay at these values under untrusted
    # (0x1650) statuswords so they do not accidentally satisfy the
    # persisted-anchor gate and thereby mask target-joint assertions.
    baseline_raw_counts = [5, 4408, 13105469, 2359290, 138, 9]
    baseline_multi_turn = [-82705, -57649, 46894, 118015, 65554, 120189]

    # Override the target joint slot with the scenario values.
    scenario_raw_counts = list(baseline_raw_counts)
    scenario_multi_turn = list(baseline_multi_turn)
    scenario_raw_counts[joint_index] = effective_live_6064
    scenario_multi_turn[joint_index] = effective_live_u40_20

    metrics_axes: list[dict[str, object]] = []
    for axis_i, gear_ratio in enumerate(robot_cfg["actuator_gear_ratios"][:6]):
        counts = int(scenario_raw_counts[axis_i])
        mt_counts = int(scenario_multi_turn[axis_i])
        sw = int(statusword) if axis_i == joint_index else 0x1650
        backend._axis_counts[axis_i] = int(counts)
        if axis_i == joint_index:
            anchor_entry_for_joint: dict[str, object] = {
                "home_anchor_rad": float(anchor_rad),
                "source": "pytest",
                "axis_indices": [joint_index],
            }
            if last_seen_absolute_counts is not None:
                last_seen_entry: dict[str, object] = {
                    "absolute_counts": int(last_seen_absolute_counts),
                    "observed_at": "2026-04-17T00:00:00+00:00",
                    "observed_by": "pytest",
                }
                if last_seen_reference_counts is not None:
                    last_seen_entry["reference_counts"] = int(last_seen_reference_counts)
                anchor_entry_for_joint["last_seen"] = last_seen_entry
            backend._absolute_encoder_home_anchors[joint_index] = anchor_entry_for_joint
        multi_turn_valid_flag = 1 if (axis_i != joint_index or multi_turn_valid) else 0
        # Split mt_counts into signed i32 low/high so the backend's
        # `_combine_signed_i64_pair` decoder reconstructs the intended
        # signed value. Historically the helper staged negative values
        # with `high=0` which silently inflated them to huge positives.
        low_u = mt_counts & 0xFFFFFFFF
        high_u = (mt_counts >> 32) & 0xFFFFFFFF
        mt_low_signed = low_u - (1 << 32) if low_u >= (1 << 31) else low_u
        mt_high_signed = high_u - (1 << 32) if high_u >= (1 << 31) else high_u
        backend._absolute_feedback_by_axis[axis_i] = _AbsoluteFeedbackAxisMetrics.from_mapping(
            {
                "encoder_multi_turn_low": {
                    "valid": multi_turn_valid_flag,
                    "value": int(mt_low_signed),
                },
                "encoder_multi_turn_high": {
                    "valid": multi_turn_valid_flag,
                    "value": int(mt_high_signed),
                },
                "absolute_position_reference": {"valid": 1, "value": int(counts)},
                "rotation_mode_position_reference": {"valid": 1, "value": int(counts)},
                "rotation_mode_encoder_low": {"valid": 1, "value": int(counts)},
                "rotation_mode_encoder_high": {"valid": 1, "value": 0},
            }
        )
        # Only stamp the retention-family fault code onto the target
        # joint so adjacent axes do not accidentally pick up a fault
        # reason that masks the target-joint assertions.
        axis_manufacturer_error_code = (
            int(encoder_retention_fault_code) if axis_i == joint_index else 0
        )
        metrics_axes.append(
            {
                "statusword": sw,
                "error_code": 0,
                "manufacturer_error_code": int(axis_manufacturer_error_code),
                "startup_drive_config": _a6ec_startup_drive_configs_for_ratio(gear_ratio)[0],
                "startup_drive_configs": _a6ec_startup_drive_configs_for_ratio(gear_ratio),
                "slave_online": 1,
                "slave_operational": 1,
                "slave_al_state": 8,
            }
        )
    monkeypatch.setattr(backend, "_load_rtcore_metrics_snapshot", lambda: {"axes": metrics_axes})
    return backend, {"metrics_axes": metrics_axes, "anchor_rad": anchor_rad, "joint_index": joint_index}


@pytest.mark.parametrize("joint_index", list(range(6)))
def test_a6ec_restart_trust_via_persisted_anchor_passes_when_axis_unmoved_while_off(
    monkeypatch, tmp_path, joint_index
):
    # Scenario: any joint was successfully homed previously. Drive power
    # cycled. Statusword dropped to 0x1650 (bit 15 cleared, matching the
    # empirical drive behavior). Live 6064 and U40.20/.22 are within
    # encoder wander of the values captured at HM35. The persisted-home-
    # anchor gate must accept this axis as still-trusted without requiring
    # a re-home, regardless of which joint is under test.
    backend, _ = _build_a6ec_restart_trust_test_backend(
        monkeypatch,
        tmp_path,
        joint_index=joint_index,
        statusword=0x1650,
        hm35_6064_counts=8,
        hm35_u40_20_counts=120191,
        live_6064_counts=9,            # +1 count wander
        live_u40_20_counts=120189,     # -2 count wander
        multi_turn_valid=True,
    )

    snapshot = backend._canonical_joint_positions_from_raw_feedback(
        {axis_i: int(backend._axis_counts[axis_i]) for axis_i in range(6)},
        reference_mode="raw",
    )
    target_detail = snapshot["axis_absolute_feedback"][joint_index]

    assert target_detail["truth_available"] is True
    assert target_detail["statusword_hex"] == "0x1650"
    assert target_detail["drive_native_truth_signature_valid"] is False
    assert target_detail["coordinate_system_valid"] is True
    assert (
        target_detail["drive_native_truth_verification_source"]
        == "persisted_home_anchor_agreement"
    )
    assert target_detail["shaft_frame_consistent"] is True


def test_a6ec_restart_trust_uses_verified_startup_drive_configs_when_legacy_primary_is_stale(
    monkeypatch, tmp_path
):
    backend, context = _build_a6ec_restart_trust_test_backend(
        monkeypatch,
        tmp_path,
        statusword=0x1650,
        hm35_6064_counts=8,
        hm35_u40_20_counts=120191,
        live_6064_counts=9,
        live_u40_20_counts=120189,
        multi_turn_valid=True,
    )
    joint_index = int(context["joint_index"])
    context["metrics_axes"][joint_index]["startup_drive_config"] = {
        # Model the legacy single-entry field after it was cleared during a
        # startup epoch reset; the verified per-descriptor list remains the
        # authoritative startup contract exported by RTCore.
        "setting_key": "a6ec_encoder_position_tracking_mode",
        "configured": 0,
        "commanded": 0,
        "readback_valid": 0,
        "readback": 0,
        "verified": 0,
    }

    snapshot = backend._canonical_joint_positions_from_raw_feedback(
        {axis_i: int(backend._axis_counts[axis_i]) for axis_i in range(6)},
        reference_mode="raw",
    )
    target_detail = snapshot["axis_absolute_feedback"][joint_index]

    assert target_detail["drive_native_startup_valid"] is True
    assert target_detail["drive_native_startup_reason"] == "verified"
    assert target_detail["truth_available"] is True
    assert (
        target_detail["drive_native_truth_verification_source"]
        == "persisted_home_anchor_agreement"
    )


@pytest.mark.parametrize("joint_index", list(range(6)))
def test_a6ec_restart_trust_via_persisted_anchor_survives_full_shaft_turn_of_manual_rotation(
    monkeypatch, tmp_path, joint_index
):
    # Scenario: drive was homed, then power-cycled, and the joint was
    # physically rotated +1 full shaft revolution while the drive was off.
    # Both 6064 and U40.20/.22 reflect the new mechanical position. The
    # mod-RM check must still accept the home as trusted because the
    # absolute-home anchor relationship is invariant under whole-shaft-turn
    # motion for EVERY joint, not just J6.
    cfg = Gradient05Config()
    counts_per_rev = int(cfg.actuator_encoder_counts_per_rev[joint_index])
    gear_ratio = float(cfg.actuator_gear_ratios[joint_index])
    rm = int(round(counts_per_rev * gear_ratio))

    backend, _ = _build_a6ec_restart_trust_test_backend(
        monkeypatch,
        tmp_path,
        joint_index=joint_index,
        statusword=0x1650,
        hm35_6064_counts=8,
        hm35_u40_20_counts=120191,
        # 6064 wraps at RM in absolute rotation mode so a full shaft turn
        # while off leaves 6064 at the same raw value post-cycle.
        live_6064_counts=8,
        # But U40.20 (motor-side multi-turn counter) increased by one RM of
        # motor-encoder counts, i.e. gear_ratio * counts_per_rev for this
        # axis.
        live_u40_20_counts=120191 + rm,
        multi_turn_valid=True,
    )

    snapshot = backend._canonical_joint_positions_from_raw_feedback(
        {axis_i: int(backend._axis_counts[axis_i]) for axis_i in range(6)},
        reference_mode="raw",
    )
    target_detail = snapshot["axis_absolute_feedback"][joint_index]

    assert target_detail["truth_available"] is True
    assert target_detail["coordinate_system_valid"] is True
    assert (
        target_detail["drive_native_truth_verification_source"]
        == "persisted_home_anchor_agreement"
    )
    assert target_detail["shaft_frame_consistent"] is True
    assert abs(int(target_detail.get("shaft_frame_wrap_turns", 0))) <= 1


@pytest.mark.parametrize("joint_index", list(range(6)))
def test_a6ec_restart_trust_via_persisted_anchor_rejects_sub_shaft_turn_drift(
    monkeypatch, tmp_path, joint_index
):
    # Scenario: the anchor file and live 6064 disagree mod-RM by RM/3 for
    # the joint under test. That indicates a real frame inconsistency
    # (encoder data loss, stale anchor, bad commissioning) and must NOT be
    # accepted as trusted home; truth must fail closed.
    cfg = Gradient05Config()
    counts_per_rev = int(cfg.actuator_encoder_counts_per_rev[joint_index])
    gear_ratio = float(cfg.actuator_gear_ratios[joint_index])
    rm = int(round(counts_per_rev * gear_ratio))

    backend, _ = _build_a6ec_restart_trust_test_backend(
        monkeypatch,
        tmp_path,
        joint_index=joint_index,
        statusword=0x1650,
        hm35_6064_counts=8,
        hm35_u40_20_counts=120191,
        live_6064_counts=8 + (rm // 3),
        live_u40_20_counts=120191,   # multi-turn did NOT track the drift
        multi_turn_valid=True,
    )

    snapshot = backend._canonical_joint_positions_from_raw_feedback(
        {axis_i: int(backend._axis_counts[axis_i]) for axis_i in range(6)},
        reference_mode="raw",
    )
    target_detail = snapshot["axis_absolute_feedback"][joint_index]

    assert target_detail["truth_available"] is False
    assert target_detail["coordinate_system_valid"] is False
    assert target_detail["shaft_frame_consistent"] is False
    # Either naming is acceptable: the Workstream 3 short-circuit names the
    # condition directly; the restart-trust validity helper names the same
    # thing via the drive_native_ prefix after it rejected the anchor path.
    assert target_detail["truth_reason"] in {
        "multi_turn_anchor_inconsistent_with_live_6064",
        "drive_native_persisted_home_anchor_inconsistent_with_live_6064",
    }


@pytest.mark.parametrize("joint_index", list(range(6)))
def test_a6ec_restart_trust_via_persisted_anchor_requires_multi_turn_valid(
    monkeypatch, tmp_path, joint_index
):
    # Scenario: encoder battery failed / multi-turn overflow on any joint.
    # Even with an otherwise-consistent anchor, we cannot trust U40.20/.22
    # so we must refuse the persisted-anchor trust path and demand a fresh
    # HM35, for every joint individually.
    backend, _ = _build_a6ec_restart_trust_test_backend(
        monkeypatch,
        tmp_path,
        joint_index=joint_index,
        statusword=0x1650,
        hm35_6064_counts=8,
        hm35_u40_20_counts=120191,
        live_6064_counts=9,
        live_u40_20_counts=120189,
        multi_turn_valid=False,  # U40.20/.22 flagged invalid
    )

    snapshot = backend._canonical_joint_positions_from_raw_feedback(
        {axis_i: int(backend._axis_counts[axis_i]) for axis_i in range(6)},
        reference_mode="raw",
    )
    target_detail = snapshot["axis_absolute_feedback"][joint_index]

    # With multi-turn invalid, the absolute-axis source itself cannot be
    # trusted so the joint falls back to the existing "absolute feedback
    # unavailable" reason rather than accepting the anchor gate.
    assert target_detail["truth_available"] is False
    assert target_detail["coordinate_system_valid"] is False
    assert target_detail["truth_reason"] in {
        "drive_native_absolute_feedback_unavailable",
        "drive_native_multi_turn_feedback_invalid",
    }


def test_a6ec_encoder_retention_fault_takes_precedence_over_anchor_path(
    monkeypatch, tmp_path
):
    # Scenario: same happy-path anchor geometry as the un-moved-while-off
    # regression (so the persisted-anchor restart-trust path would
    # otherwise upgrade coordinate_system_valid to True), but the drive is
    # simultaneously asserting Er20.9 (encoder multi-turn error) via
    # manufacturer_error_code 0x209. Retention-family faults must outrank
    # the anchor path: multi-turn integrity is the exact precondition
    # anchor agreement depends on, so the "looks agreed mod-RM" result is
    # a false positive. Truth must fail closed with the new specific
    # reason.
    joint_index = 5
    backend, _ = _build_a6ec_restart_trust_test_backend(
        monkeypatch,
        tmp_path,
        joint_index=joint_index,
        statusword=0x1650,
        hm35_6064_counts=8,
        hm35_u40_20_counts=120191,
        live_6064_counts=9,
        live_u40_20_counts=120189,
        multi_turn_valid=True,
        encoder_retention_fault_code=0x209,
    )

    snapshot = backend._canonical_joint_positions_from_raw_feedback(
        {axis_i: int(backend._axis_counts[axis_i]) for axis_i in range(6)},
        reference_mode="raw",
    )
    target_detail = snapshot["axis_absolute_feedback"][joint_index]

    assert target_detail["truth_available"] is False
    assert target_detail["coordinate_system_valid"] is False
    assert target_detail["drive_native_truth_reason"] == "encoder_retention_fault_present"
    assert target_detail["truth_reason"] == "drive_native_encoder_retention_fault_present"
    assert target_detail.get("encoder_retention_fault_present") is True
    retention_detail = target_detail.get("encoder_retention_fault")
    assert isinstance(retention_detail, dict)
    assert "Er20.9" in retention_detail.get("codes", [])
    assert "Encoder multi-turn error" in retention_detail.get("names", [])


def test_a6ec_encoder_retention_fault_distinguishes_battery_alarm(
    monkeypatch, tmp_path
):
    # ALF9.0 "Encoder battery voltage low" lives in the alarm_codes
    # table but is still retention-family per the profile. Same
    # precedence rule: block trust even when anchor agreement would
    # otherwise succeed, and label with the vendor name so operators can
    # distinguish a battery alarm from a multi-turn fault without
    # reading raw hex.
    joint_index = 5
    backend, _ = _build_a6ec_restart_trust_test_backend(
        monkeypatch,
        tmp_path,
        joint_index=joint_index,
        statusword=0x1650,
        hm35_6064_counts=8,
        hm35_u40_20_counts=120191,
        live_6064_counts=9,
        live_u40_20_counts=120189,
        multi_turn_valid=True,
        encoder_retention_fault_code=0xF90,
    )

    snapshot = backend._canonical_joint_positions_from_raw_feedback(
        {axis_i: int(backend._axis_counts[axis_i]) for axis_i in range(6)},
        reference_mode="raw",
    )
    target_detail = snapshot["axis_absolute_feedback"][joint_index]

    assert target_detail["truth_available"] is False
    assert target_detail["coordinate_system_valid"] is False
    assert target_detail["drive_native_truth_reason"] == "encoder_retention_fault_present"
    retention_detail = target_detail.get("encoder_retention_fault")
    assert isinstance(retention_detail, dict)
    assert "ALF9.0" in retention_detail.get("codes", [])
    assert "Encoder battery voltage low" in retention_detail.get("names", [])


def test_a6ec_encoder_retention_fault_includes_multi_turn_overflow(
    monkeypatch, tmp_path
):
    # ErA0.1 "Multi-turn overflow fault" (manufacturer_error_code 0xA01)
    # is a primary signal of U40.20/.22 unreliability per vendor email 4
    # Q2(a). Its bus-level 0x603F value is 0X7305 (same encoder-error
    # class as Er20.8 / Er20.9), and the vendor notes flag it as one of
    # the three conditions requiring F31.10 recovery. It must be treated
    # as retention-family and block the persisted-home-anchor restart
    # trust path with the specific "encoder_retention_fault_present"
    # reason rather than falling into the generic
    # "manufacturer_fault_present" label.
    joint_index = 5
    backend, _ = _build_a6ec_restart_trust_test_backend(
        monkeypatch,
        tmp_path,
        joint_index=joint_index,
        statusword=0x1650,
        hm35_6064_counts=8,
        hm35_u40_20_counts=120191,
        live_6064_counts=9,
        live_u40_20_counts=120189,
        multi_turn_valid=True,
        encoder_retention_fault_code=0xA01,
    )

    snapshot = backend._canonical_joint_positions_from_raw_feedback(
        {axis_i: int(backend._axis_counts[axis_i]) for axis_i in range(6)},
        reference_mode="raw",
    )
    target_detail = snapshot["axis_absolute_feedback"][joint_index]

    assert target_detail["truth_available"] is False
    assert target_detail["coordinate_system_valid"] is False
    assert target_detail["drive_native_truth_reason"] == "encoder_retention_fault_present"
    retention_detail = target_detail.get("encoder_retention_fault")
    assert isinstance(retention_detail, dict)
    assert "ErA0.1" in retention_detail.get("codes", [])
    assert "Multi-turn overflow fault" in retention_detail.get("names", [])


def test_a6ec_firmware_bit15_retention_flag_documented_false():
    # The bit-15 restart-trust path in derive_drive_native_truth_validity
    # is intentionally kept as vestigial coverage for future firmware /
    # drive families that honour vendor Q9. The A6-EC profile must
    # advertise that this current firmware empirically clears bit 15 on
    # every drive power cycle, so the bit-15 path is unreachable on
    # production hardware. This flag is documentation metadata only.
    from gradient_os.arm_controller.profiles.drive.a6ec_ds402 import (
        POSITION_SEMANTICS_CONFIG,
    )

    assert "firmware_bit15_retention_expected" in POSITION_SEMANTICS_CONFIG
    assert POSITION_SEMANTICS_CONFIG["firmware_bit15_retention_expected"] is False


def test_a6ec_profile_emits_continuous_607a_command_in_rotation_mode():
    # Regression for the 2026-04-19 continuous-607A landing. Vendor
    # Chapter 5 §5.3 Figure 5-1 shows target position as a continuous
    # linear ramp while 6064 is a sawtooth. The A6-EC profile must
    # therefore (a) keep `feedback_counts_wrap=True` so 6064 comparisons
    # and completion checks stay modulo-RM, (b) set `command_counts_wrap`
    # to False so RTCore pipes continuous 0x607A to the drive, and (c)
    # classify the seam as safe so the host-side safety guard stops
    # rejecting seam-crossing trajectories. All three flags together are
    # the documented rotation-mode contract; flipping any one of them
    # back is a regression that breaks multi-turn.
    from gradient_os.arm_controller.profiles.drive.a6ec_ds402 import (
        MOTION_FEEDBACK_CONFIG,
        POSITION_SEMANTICS_CONFIG,
    )

    assert MOTION_FEEDBACK_CONFIG["feedback_counts_wrap"] is True
    assert "command_counts_wrap" in MOTION_FEEDBACK_CONFIG
    assert MOTION_FEEDBACK_CONFIG["command_counts_wrap"] is False
    assert POSITION_SEMANTICS_CONFIG["command_frame_seam_crossing_unsafe"] is False


def test_a6ec_backend_routes_profile_command_counts_wrap_to_fold_decision(
    monkeypatch, tmp_path
):
    # The backend must read `command_counts_wrap` from the active drive
    # profile's MOTION_FEEDBACK_CONFIG (not from an env var) and pass the
    # negation as `wrap_to_single_turn` to `_nearest_turn_fold_axis_q_for_axis`.
    # This is the regression that caught the original
    # GRADIENT_RTCORE_EXPERIMENTAL_CONTINUOUS_607A_JOINTS env var being a
    # no-op on the wire: env was set, host-side fold stayed linear, but
    # RTCore wrapped anyway.
    monkeypatch.setenv("GRADIENT_JOINT_ZERO_OFFSETS_PATH", str(tmp_path / "joint_zero_offsets.json"))
    backend = EthercatRTCoreBackend(robot_config=Gradient05Config().get_config_dict())
    backend._connected = True
    backend._rt_drive_profile_id = "a6ec_ds402"
    # A6-EC profile says command_counts_wrap=False -> continuous emission
    # -> helper returns False -> alias returns True ("experimental
    # continuous enabled"). Both helpers must agree.
    for joint_idx in range(6):
        assert backend._command_counts_wrap_for_joint(joint_idx) is False
        assert backend._experimental_continuous_607a_enabled_for_joint(joint_idx) is True


def test_a6ec_continuous_607a_fold_does_not_flip_turn_at_midpoint_home(
    monkeypatch, tmp_path
):
    # Regression for the 2026-04-19 Move A 350-deg long-way excursion.
    # With the A6-EC drive homed at 607C = RM/2 (midpoint bias), a
    # small +10 deg canonical move from home MUST emit a wire target
    # close to the current 6064 reading, NOT one turn above it. The
    # fold was previously comparing base_counts (raw axis-q frame)
    # against observed_counts (live 6064, wire frame) which differ by
    # native_home_position_offset = -607C; at midpoint home that bias
    # is exactly RM/2, so a small move produced delta/period ~= 0.528
    # which round() snapped to 1 turn and emitted 0x607A = RM + 618_952
    # instead of 618_952. Under continuous-607A emission (RTCore no
    # longer wraps) that 1-turn error leaks onto the wire and the
    # drive takes the long way by ~1 rev. This test asserts the fold
    # returns +0.1745 rad (+10 deg) given canonical_q=+10 deg, not
    # -6.111 rad (-350 deg).
    monkeypatch.setenv("GRADIENT_JOINT_ZERO_OFFSETS_PATH", str(tmp_path / "joint_zero_offsets.json"))
    monkeypatch.setenv(
        "GRADIENT_ABSOLUTE_ENCODER_ANCHORS_PATH",
        str(tmp_path / "absolute_encoder_anchors.json"),
    )
    metrics_path = tmp_path / "metrics.json"
    monkeypatch.setattr(rtcore_backend_module, "_RTCORE_METRICS_PATH", metrics_path)
    counts_per_rev = 131072
    gear_ratio = 10.0
    counts_per_unit = (float(counts_per_rev) * gear_ratio) / (2.0 * math.pi)
    rm = int(round(counts_per_unit * 2.0 * math.pi))
    half_rm = rm // 2
    # Mirror the live Move A state: drive at midpoint home, native home
    # offset = -RM/2 (from 607C = +RM/2).
    _write_rtcore_metrics_snapshot(
        metrics_path,
        [
            {
                "native_home_position_offset": -half_rm,
                "absolute_feedback": {},
            }
        ],
    )
    backend = EthercatRTCoreBackend(robot_config=Gradient05Config().get_config_dict())
    backend._connected = True
    backend._rt_num_axes = 1
    backend._axis_to_joint = [0]
    backend._axis_config = _AxisConfig(
        num_axes=1,
        counts_per_unit=[counts_per_unit] + [0.0] * 15,
        sign=[-1] + [0] * 15,
        counts_per_rev=[counts_per_rev] + [0] * 15,
    )
    backend._rt_drive_profile_id = "a6ec_ds402"

    # Live 6064 at midpoint home (= RM/2), canonical target +10 deg.
    live_counts = half_rm  # 655,360 (home, wire frame)
    target_canonical_rad = math.radians(10.0)
    target_axis_q = backend._command_axis_q_for_joint_value(
        axis_i=0,
        logical_joint_idx=0,
        canonical_q=target_canonical_rad,
        live_reference_counts=live_counts,
    )
    # Under the bugged fold this returned -6.111 rad (-350 deg) because
    # observed was compared to base in mismatched frames. The fix
    # normalizes observed to the axis-q frame before the round.
    assert math.isclose(target_axis_q, target_canonical_rad, abs_tol=1e-6), (
        f"fold snapped to wrong turn: got {target_axis_q!r} rad "
        f"({math.degrees(target_axis_q):.3f} deg), expected "
        f"{target_canonical_rad!r} rad (+10.000 deg)"
    )
    # Derived wire value must stay near live.
    wire_counts = int(round(target_axis_q * (-1) * counts_per_unit)) - (-half_rm)
    wire_distance_from_live = abs(wire_counts - live_counts)
    assert wire_distance_from_live < half_rm, (
        f"wire_counts={wire_counts} too far from live_counts={live_counts}: "
        f"distance={wire_distance_from_live} > RM/2={half_rm}"
    )


# --------------------------------------------------------------------------
# 2026-04-19 UI whip regression: nearest-turn fold must use the multi-turn
# encoder reference (`encoder_multi_turn_counts`) at the seam boundary,
# not live 0x6064 alone.
#
# Scenario: J6 is parked exactly on the seam at canonical +180 deg after a
# prior jog. The drive's live 0x6064 can read EITHER 0 OR RM-1 depending
# on sub-count encoder noise. With the old fold (single-turn reference),
# round(delta/period) flipped turn count between 0 and +1 based on which
# reading came in, which caused the 2026-04-19 UI-driven whip (operator
# jogged +5 deg -> +180, waited, jogged +5 deg -> +185; second jog whipped
# +360 deg the long way because the fold picked turn=+1). With the fix,
# the fold reads U40.20/.22 combined signed i64 via the profile's
# standard `encoder_multi_turn_counts` key, which is unambiguous, and
# picks the correct turn regardless of sub-count noise on 0x6064.
# --------------------------------------------------------------------------
def _setup_midpoint_home_fold_backend(
    monkeypatch, tmp_path, *, u40_20_motor_counts: int
) -> tuple["EthercatRTCoreBackend", int, int, float]:
    """Build an A6-EC-like backend at midpoint home with a supplied U40.20
    motor-frame multi-turn reading. Returns (backend, rm, half_rm,
    counts_per_unit).
    """
    monkeypatch.setenv(
        "GRADIENT_JOINT_ZERO_OFFSETS_PATH",
        str(tmp_path / "joint_zero_offsets.json"),
    )
    monkeypatch.setenv(
        "GRADIENT_ABSOLUTE_ENCODER_ANCHORS_PATH",
        str(tmp_path / "absolute_encoder_anchors.json"),
    )
    metrics_path = tmp_path / "metrics.json"
    monkeypatch.setattr(rtcore_backend_module, "_RTCORE_METRICS_PATH", metrics_path)
    counts_per_rev = 131072
    gear_ratio = 10.0  # A6-EC C10.18/C10.19 = 10/1 for J6
    counts_per_unit = (float(counts_per_rev) * gear_ratio) / (2.0 * math.pi)
    rm = int(round(counts_per_unit * 2.0 * math.pi))
    half_rm = rm // 2
    # Split the 64-bit motor counts across the low/high 32-bit fields.
    mt_u64 = int(u40_20_motor_counts) & ((1 << 64) - 1)
    low_signed = mt_u64 & 0xFFFFFFFF
    if low_signed >= (1 << 31):
        low_signed -= 1 << 32
    high_signed = (mt_u64 >> 32) & 0xFFFFFFFF
    if high_signed >= (1 << 31):
        high_signed -= 1 << 32
    _write_rtcore_metrics_snapshot(
        metrics_path,
        [
            {
                "native_home_position_offset": -half_rm,
                "absolute_feedback": {
                    "encoder_multi_turn_low": {"valid": 1, "value": int(low_signed)},
                    "encoder_multi_turn_high": {"valid": 1, "value": int(high_signed)},
                },
            }
        ],
    )
    backend = EthercatRTCoreBackend(robot_config=Gradient05Config().get_config_dict())
    backend._connected = True
    backend._rt_num_axes = 1
    backend._axis_to_joint = [0]
    backend._axis_config = _AxisConfig(
        num_axes=1,
        counts_per_unit=[counts_per_unit] + [0.0] * 15,
        sign=[-1] + [0] * 15,
        counts_per_rev=[counts_per_rev] + [0] * 15,
    )
    backend._rt_drive_profile_id = "a6ec_ds402"
    return backend, rm, half_rm, counts_per_unit


def test_a6ec_fold_uses_multi_turn_reference_at_seam_low_side(
    monkeypatch, tmp_path
):
    """J6 at canonical +180 deg with live 0x6064 = 0 (the low-side ambiguous
    reading at the seam). Commanding canonical +185 deg from here MUST emit
    a target near axis-q +185 deg -- short path, turn 0 -- not +185+360 deg
    (long path, turn +1 whip).

    In motor-frame counts, canonical +180 deg for J6 (sign=-1, gear=10):
      output_rev = 180 / 360 = 0.5
      motor_rev  = output_rev * gear = 5
      motor_cts  = motor_rev * counts_per_motor_rev * sign(=-1) = -655,360
    """
    backend, rm, half_rm, counts_per_unit = _setup_midpoint_home_fold_backend(
        monkeypatch, tmp_path, u40_20_motor_counts=-655_360
    )
    # Live 0x6064 at the seam low side (wire = 0). Without the fix, the
    # legacy path sees observed_counts = 0 + (-RM/2) = -RM/2 and base_counts
    # = +185*-1*counts_per_unit ~= -673,785. delta/period = (-655,360 -
    # -673,785)/RM = +0.014 -> round() = 0. Correct by luck. But the
    # high-side test below (wire = RM-1) would flip to +1 without U40.20/.22.
    target_axis_q = backend._command_axis_q_for_joint_value(
        axis_i=0,
        logical_joint_idx=0,
        canonical_q=math.radians(185.0),
        live_reference_counts=0,
    )
    # Expected short-path target: axis-q = +185 deg = +3.228... rad
    assert math.isclose(
        target_axis_q, math.radians(185.0), abs_tol=1e-6
    ), (
        f"fold snapped to wrong turn: got {target_axis_q!r} rad "
        f"({math.degrees(target_axis_q):.3f} deg), expected +185.000 deg"
    )


def test_a6ec_fold_uses_multi_turn_reference_at_seam_high_side(
    monkeypatch, tmp_path
):
    """Mirror of the low-side test: J6 at canonical +180 deg with live
    0x6064 = RM-1 (the high-side ambiguous reading at the seam). Without
    the multi-turn fix this is EXACTLY the case that whipped on 2026-04-19:
    the old fold computes delta/period ~= +1.014, rounds to +1, emits a
    target RM above where it should be, and the drive goes the long way.

    With the multi-turn fix, U40.20/.22 unambiguously says we're at motor
    count -655,360 (canonical +180 deg, turn 0), and the fold picks
    wrap_turns = 0 regardless of which ambiguous 0x6064 reading came in.
    """
    backend, rm, half_rm, counts_per_unit = _setup_midpoint_home_fold_backend(
        monkeypatch, tmp_path, u40_20_motor_counts=-655_360
    )
    # Same motor-frame multi-turn position (-655,360 motor counts =
    # canonical +180 deg), but 0x6064 happens to read RM-1 this time.
    # This is the reading that TRIGGERED the whip on 2026-04-19.
    target_axis_q = backend._command_axis_q_for_joint_value(
        axis_i=0,
        logical_joint_idx=0,
        canonical_q=math.radians(185.0),
        live_reference_counts=rm - 1,
    )
    # Must still produce short-path target, not one turn above.
    assert math.isclose(
        target_axis_q, math.radians(185.0), abs_tol=1e-6
    ), (
        f"fold took the long path at the seam high side: got "
        f"{target_axis_q!r} rad ({math.degrees(target_axis_q):.3f} deg), "
        f"expected +185.000 deg. This is the 2026-04-19 UI whip regression."
    )


def test_a6ec_fold_away_from_seam_preserves_single_turn_emission(
    monkeypatch, tmp_path
):
    """Path-B regression: when J6 is NOT near the seam (i.e., live
    0x6064 is well away from the 0/RM boundary), the multi-turn data
    must NOT shift observed_counts. The fold's wire emission must stay
    identical to the pre-fix single-turn-aware behavior everywhere
    outside the +/- ~22 deg seam-adjacent band. This regression guards
    against the multi-turn disambiguation leaking outside its intended
    scope (see Phase 6 matrix in DEVLOG 2026-04-19 18:30: verified-safe
    wire semantics only apply within +/-RM/2 of live 0x6064).
    """
    # J6 at canonical +90 deg (wire 6064 at RM/4 = 327,680) -- mid-band,
    # nowhere near the seam. The multi-turn register unambiguously
    # says we're at canonical +90 deg motor-frame. Commanding +91 deg
    # should produce axis-q = +91 deg and a wire target close to live
    # 0x6064 (not one turn above, which would be the failure mode if
    # multi-turn disambiguation fired outside the seam band).
    backend, rm, half_rm, counts_per_unit = _setup_midpoint_home_fold_backend(
        monkeypatch, tmp_path, u40_20_motor_counts=int(math.radians(90.0) * (-1) * (131072.0 * 10.0 / (2.0 * math.pi)))
    )
    # Live 0x6064 at canonical +90 deg for J6 (sign=-1, midpoint home):
    #   wire = home_offset (+RM/2) - canonical * RM/(2pi) * |sign|
    #        = RM/2 - (RM/4) = RM/4 = 327,680
    live_counts = rm // 4
    target_axis_q = backend._command_axis_q_for_joint_value(
        axis_i=0,
        logical_joint_idx=0,
        canonical_q=math.radians(91.0),
        live_reference_counts=live_counts,
    )
    # target canonical +91 deg -> axis-q +91 deg (fold didn't flip turn)
    assert math.isclose(
        target_axis_q, math.radians(91.0), abs_tol=1e-6
    ), (
        f"away-from-seam emission changed: got {target_axis_q!r} rad "
        f"({math.degrees(target_axis_q):.3f} deg), expected +91.000 deg. "
        f"Multi-turn disambiguation must not fire outside the seam-adjacent band."
    )
    # Compute the actual wire value emitted on the wire. Following the
    # existing midpoint-home fold regression test pattern:
    wire_counts = int(round(target_axis_q * (-1) * counts_per_unit)) - (-half_rm)
    # Wire should be within one period of live (single-turn emission
    # preserved; pre-fix behavior equals post-fix behavior here).
    assert abs(wire_counts - live_counts) < rm // 2, (
        f"wire_counts={wire_counts} drifted more than RM/2 from live_counts={live_counts}; "
        f"multi-turn disambiguation leaked outside the seam band."
    )


def test_a6ec_multi_turn_reference_falls_back_when_profile_omits_key(
    monkeypatch, tmp_path
):
    """Vendor-agnostic safety net: when the profile's absolute_feedback
    does NOT include `encoder_multi_turn_counts` (e.g., a drive profile
    that does not expose a multi-turn register), the helper must return
    None so the fold cleanly falls back to the legacy single-turn path.
    """
    monkeypatch.setenv(
        "GRADIENT_JOINT_ZERO_OFFSETS_PATH",
        str(tmp_path / "joint_zero_offsets.json"),
    )
    monkeypatch.setenv(
        "GRADIENT_ABSOLUTE_ENCODER_ANCHORS_PATH",
        str(tmp_path / "absolute_encoder_anchors.json"),
    )
    metrics_path = tmp_path / "metrics.json"
    monkeypatch.setattr(rtcore_backend_module, "_RTCORE_METRICS_PATH", metrics_path)
    counts_per_rev = 131072
    counts_per_unit = (float(counts_per_rev) * 10.0) / (2.0 * math.pi)
    half_rm = int(round(counts_per_unit * math.pi))
    # Write metrics with EMPTY absolute_feedback -- profile cannot derive a
    # signed_i64_pair source and the helper must return None.
    _write_rtcore_metrics_snapshot(
        metrics_path,
        [
            {
                "native_home_position_offset": -half_rm,
                "absolute_feedback": {},
            }
        ],
    )
    backend = EthercatRTCoreBackend(robot_config=Gradient05Config().get_config_dict())
    backend._connected = True
    backend._rt_num_axes = 1
    backend._axis_to_joint = [0]
    backend._axis_config = _AxisConfig(
        num_axes=1,
        counts_per_unit=[counts_per_unit] + [0.0] * 15,
        sign=[-1] + [0] * 15,
        counts_per_rev=[counts_per_rev] + [0] * 15,
    )
    backend._rt_drive_profile_id = "a6ec_ds402"
    result = backend._multi_turn_reference_counts_for_axis(0)
    assert result is None, (
        f"helper must return None when the profile does not expose "
        f"encoder_multi_turn_counts, got {result!r}"
    )


# --------------------------------------------------------------------------
# 2026-04-20 `_display_feedback_counts_for_axis` multi-turn preference.
#
# Scenario: at startup the drive publishes `pos_counts = 0, statusword = 0`
# for several seconds while the PDO mapping latches (pre-OP). Any caller that
# invokes the display-unwrap path during that window used to seed the cache
# from the bogus zero; when the drive subsequently reported a near-seam
# position (~RM/2 counts away from 0), the nearest-turn unwrap picked the
# -1 turn and propagated that stale frame indefinitely, leaving the display
# frame permanently offset by one full revolution. The user-visible symptom
# was `/info/joints-detailed` reporting `command_roundtrip_consistent=False`
# with `error_counts = ±RM`, which surfaced through
# `truth_reason=absolute_home_anchor_stale` and caused the
# `/control/joint-jog` endpoint to reject with
# `CANONICAL_JOINT_TRUTH_UNAVAILABLE`.
#
# The durable fix delegates the display-unwrap path to the drive's multi-turn
# register (profile contract `encoder_multi_turn_counts`) when both the
# multi-turn reading AND a captured home anchor are available. That ground-
# truth path has no dependency on pre-OP PDO reads or HM35 reference rewrites.
# The fallback accumulated-unwrap path gains a pre-OP seed gate and an HM35-
# invalidation hook for drives that don't expose multi-turn.
# --------------------------------------------------------------------------
def test_a6ec_display_feedback_prefers_multi_turn_when_anchored(
    monkeypatch, tmp_path
):
    """J6 at canonical 0 deg (home) with live 0x6064 = RM/2 and multi-turn
    reading at the anchor counts. The display unwrap must return RM/2 (the
    wire-frame representation of axis-q = 0), NOT -RM/2 (which would yield
    reference_q = 2*pi = +360 deg in the canonical frame and falsely flag
    the axis as stale-anchor).
    """
    backend, rm, half_rm, counts_per_unit = _setup_midpoint_home_fold_backend(
        monkeypatch, tmp_path, u40_20_motor_counts=0
    )
    # Install an anchor at home (motor_counts in the metrics is also 0 via
    # u40_20_motor_counts=0; with anchor at 0 rad they are consistent).
    with backend._status_lock:
        backend._absolute_encoder_home_anchors = [
            {"home_anchor_rad": 0.0, "source": "encoder_multi_turn_counts"}
        ]
    # Verify the helper returns the correct axis-q motor counts (= 0 at home).
    multi_turn_axis_q = backend._multi_turn_reference_counts_for_axis_when_anchored(0)
    assert multi_turn_axis_q == 0, (
        f"multi-turn axis-q at home must be 0, got {multi_turn_axis_q}"
    )
    # The key invariant: display_counts at home wire (RM/2) must equal RM/2.
    # This yields physical_q = -pi, and + native_home_offset_q = +pi produces
    # reference_q = 0 (canonical at home), not 2*pi (= canonical one-turn-off).
    display_counts = backend._display_feedback_counts_for_axis(0, half_rm)
    assert display_counts == half_rm, (
        f"display_feedback_counts at home wire must equal RM/2 = {half_rm}, "
        f"got {display_counts}. A -RM/2 return would make reference_q = 2*pi "
        f"and falsely flag the axis as stale-anchor (the 2026-04-20 UI "
        f"jog-rejection regression)."
    )


def test_a6ec_display_feedback_multi_turn_path_survives_pre_op_zero_reads(
    monkeypatch, tmp_path
):
    """Directly reproduces the 2026-04-20 UI-jog rejection: at startup the
    backend received several cycles of `pos_counts=0, statusword=0` before
    the drive latched a near-seam absolute position. With the pre-fix code,
    those zero-reads permanently offset the display unwrap cache by -1 turn.
    With the multi-turn-preferred path, the display unwrap ignores the
    host-accumulated cache entirely when multi-turn + anchor are available.
    """
    backend, rm, half_rm, counts_per_unit = _setup_midpoint_home_fold_backend(
        monkeypatch, tmp_path, u40_20_motor_counts=0
    )
    # Install an anchor at home so the multi-turn-preferred path engages.
    with backend._status_lock:
        backend._absolute_encoder_home_anchors = [
            {"home_anchor_rad": 0.0, "source": "encoder_multi_turn_counts"}
        ]
    # Simulate the pre-OP zero-read path: invoke display-unwrap with
    # raw_counts = 0 and statusword = 0 (the pattern observed at sessions 0-8s).
    # This used to poison the cache with 0; with the fix, the multi-turn path
    # takes over and returns the anchor-relative wire frame (= half_rm at home).
    with backend._status_lock:
        backend._axis_statusword[0] = 0
    first_call = backend._display_feedback_counts_for_axis(0, 0)
    assert first_call == half_rm, (
        f"multi-turn-preferred path must return the anchor-relative wire "
        f"frame at home ({half_rm}), NOT 0 (pre-OP cache). got {first_call}"
    )
    # Drive comes up. Sample near-seam like the 2026-04-20 startup trace.
    near_seam_wire = rm - 18_253
    with backend._status_lock:
        backend._axis_statusword[0] = 0x9650
    second_call = backend._display_feedback_counts_for_axis(0, near_seam_wire)
    # multi-turn is STILL 0 (joint physically hasn't moved from anchor);
    # wire_unwrapped = 0 - native_home_offset = 0 - (-half_rm) = half_rm.
    # Pre-fix this would have returned -18_253 (near-seam representation
    # with stale -1 turn cache), yielding reference_q = 2*pi offset.
    assert second_call == half_rm, (
        f"multi-turn path must continue returning half_rm={half_rm} as long "
        f"as physical motor_counts haven't changed. got {second_call} "
        f"(pre-fix behavior yielded -18_253)."
    )


def test_a6ec_display_feedback_fallback_gates_seed_on_pre_op_zero(
    monkeypatch, tmp_path
):
    """Profile without a captured anchor must fall back to accumulated-unwrap.
    In that fallback, the pre-OP `(raw=0, sw=0)` pattern must NOT seed the
    cache, otherwise a subsequent near-seam report poisons the cache by -1
    turn. The seed is deferred until either raw or statusword becomes
    non-zero.
    """
    backend, rm, half_rm, counts_per_unit = _setup_midpoint_home_fold_backend(
        monkeypatch, tmp_path, u40_20_motor_counts=0
    )
    # No anchor installed: multi-turn-preferred path declines, fallback runs.
    with backend._status_lock:
        backend._absolute_encoder_home_anchors = [None]
        backend._axis_statusword[0] = 0
    # Pre-OP: raw=0, sw=0. Seed MUST be deferred.
    assert backend._display_feedback_counts_for_axis(0, 0) == 0
    with backend._status_lock:
        assert not backend._feedback_unwrapped_valid[0], (
            "fallback must refuse to seed from the (raw=0, sw=0) pre-OP "
            "pattern; otherwise the subsequent drive-latch sample poisons "
            "the unwrap cache by -1 turn for the rest of the session."
        )
    # Drive comes up, reports near-seam. With the gate in place, THIS is the
    # first seed (not the earlier zero), and nearest-turn unwrap correctly
    # tracks from here.
    near_seam_wire = rm - 18_253
    with backend._status_lock:
        backend._axis_statusword[0] = 0x9650
    result = backend._display_feedback_counts_for_axis(0, near_seam_wire)
    # Seeded from normalized_counts which for near_seam_wire = rm - 18_253
    # gives ((rm - 18_253 + half_rm) % rm) - half_rm = (-18_253 + half_rm) -
    # half_rm = ... Just compute directly.
    expected_normalized = ((near_seam_wire + half_rm) % rm) - half_rm
    assert result == expected_normalized, (
        f"first post-pre-OP seed must be the normalized near-seam value "
        f"({expected_normalized}), got {result}"
    )
    with backend._status_lock:
        assert backend._feedback_unwrapped_valid[0]


def test_a6ec_display_feedback_fallback_invalidates_cache_on_hm35(
    monkeypatch, tmp_path
):
    """When the drive rewrites 0x607C (HM35 completes), the
    `_refresh_native_home_offsets_from_metrics` hook must invalidate the
    affected axis's fallback unwrap cache so the next display query re-seeds
    from the post-HM35 wire value. Without this invalidation, a pre-HM35
    stale turn count propagates through the home procedure indefinitely
    (because the HM35-induced 6064 jump is ~half_rm, which rounds to 0 turns
    in nearest-turn unwrap and just keeps the stale count).
    """
    backend, rm, half_rm, counts_per_unit = _setup_midpoint_home_fold_backend(
        monkeypatch, tmp_path, u40_20_motor_counts=0
    )
    # Seed the fallback cache with a pre-HM35 value: imagine the axis was at
    # canonical +175 deg before HM35 (wire near seam, unwrapped to a slightly
    # negative value under the pre-HM35 reference frame).
    with backend._status_lock:
        backend._absolute_encoder_home_anchors = [None]  # force fallback
        backend._feedback_unwrapped_counts[0] = -18_253
        backend._feedback_unwrapped_valid[0] = True
        # Simulate a prior native_home_offset value that is about to change.
        backend._native_home_offset_counts[0] = 0
    # Now HM35 writes 607C = RM/2, so native_home_position_offset changes to
    # -RM/2. Re-write the metrics file with the new offset and force the
    # refresh (mtime_ns reset so `_refresh_native_home_offsets_from_metrics`
    # doesn't short-circuit on unchanged mtime).
    metrics_path = rtcore_backend_module._RTCORE_METRICS_PATH
    _write_rtcore_metrics_snapshot(
        metrics_path,
        [
            {
                "native_home_position_offset": -half_rm,
                "absolute_feedback": {
                    "encoder_multi_turn_low": {"valid": 1, "value": 0},
                    "encoder_multi_turn_high": {"valid": 1, "value": 0},
                },
            }
        ],
    )
    with backend._status_lock:
        backend._native_home_metrics_mtime_ns = -1  # force re-read
    backend._refresh_native_home_offsets_from_metrics()
    with backend._status_lock:
        assert not backend._feedback_unwrapped_valid[0], (
            "HM35-induced native_home_position_offset change MUST invalidate "
            "the fallback unwrap cache; otherwise the pre-HM35 stale turn "
            "count propagates through the home procedure indefinitely."
        )


# --------------------------------------------------------------------------
# 2026-04-20 Phase 5 whip regression: when the host has multi-turn state and
# a captured home anchor, the command path must preserve the commanded
# direction. The operator's observed failure mode was: J6 at canonical +365°
# (one full rev forward in the multi-turn register), user commands delta
# -185° to reach canonical +180°. Pre-fix, the fold's round(delta/period)
# used live_6064 (short-form +5°) as the anchor and produced a trajectory
# where adjacent waypoints picked DIFFERENT wrap_turns, creating a non-
# monotonic wire-frame target path that RTCore's velocity planner chased at
# the drive's max RPM (6000 motor RPM) — even though the commanded speed
# was 100 motor RPM. Post-fix, with an anchored multi-turn register, the
# command path skips the fold's turn-shift and emits base_axis_q directly
# (= canonical + master_offset), which is monotonic across the trajectory
# by construction because canonical is already multi-turn-continuous.
# --------------------------------------------------------------------------
def test_a6ec_command_axis_q_preserves_direction_with_multi_turn_anchor(
    monkeypatch, tmp_path
):
    """J6 physically at canonical +365° (one forward rev + 5°). Multi-turn
    register at motor_counts corresponding to +365° axis-q. Anchor captured
    at home. When user commands canonical +365° (= no motion from current),
    the emitted axis_q must equal +365° rad EXACTLY, not +5° (collapsed
    modular form).
    """
    # Motor at canonical +365° with sign=-1: motor_counts = +365° * -1 * cpu
    # ~= -1,329,115 counts (one full rev forward in motor frame).
    backend, rm, half_rm, counts_per_unit = _setup_midpoint_home_fold_backend(
        monkeypatch, tmp_path, u40_20_motor_counts=-1_329_115
    )
    # Install home anchor captured at motor zero (canonical 0°).
    with backend._status_lock:
        backend._absolute_encoder_home_anchors = [
            {"home_anchor_rad": 0.0, "source": "encoder_multi_turn_counts"}
        ]
    # Live wire reads canonical +5° (modular short-form) — this is what the
    # fold's pre-fix logic would use as the anchor and where it would COLLAPSE
    # the command to. We want the command path to IGNORE this and honor the
    # multi-turn-aware canonical input instead.
    live_wire = half_rm + int(round(math.radians(5.0) * (-1.0) * counts_per_unit))
    target_axis_q = backend._command_axis_q_for_joint_value(
        axis_i=0,
        logical_joint_idx=0,
        canonical_q=math.radians(365.0),
        live_reference_counts=live_wire,
    )
    assert math.isclose(
        target_axis_q, math.radians(365.0), abs_tol=1e-6
    ), (
        f"direction-preserving command path must honor multi-turn-aware "
        f"canonical, got {target_axis_q!r} rad ({math.degrees(target_axis_q):.3f}°), "
        f"expected +365.000° exactly. A result near +5° would be the Phase 5 "
        f"whip pre-fix behavior."
    )


def test_a6ec_command_axis_q_trajectory_waypoints_monotonic_with_multi_turn(
    monkeypatch, tmp_path
):
    """Simulate the Phase 5 trajectory: J6 at canonical +365° (multi-turn
    register), user commands delta -185° via an s-curve trajectory through
    canonical {+365°, +272.5°, +180°}. Every emitted axis_q must be
    monotonic (no mid-trajectory turn-flip). Pre-fix, the middle waypoint
    at +272.5° picked turn +1 while the start (+365°) and end (+180°)
    picked turn 0, producing a non-monotonic wire-frame path that caused
    the observed whip.
    """
    backend, rm, half_rm, counts_per_unit = _setup_midpoint_home_fold_backend(
        monkeypatch, tmp_path, u40_20_motor_counts=-1_329_115
    )
    with backend._status_lock:
        backend._absolute_encoder_home_anchors = [
            {"home_anchor_rad": 0.0, "source": "encoder_multi_turn_counts"}
        ]
    # Motor at canonical +365° modularly = +5° (wire counts for +5°):
    live_wire = half_rm + int(round(math.radians(5.0) * (-1.0) * counts_per_unit))
    waypoints_canonical_deg = [365.0, 340.0, 310.0, 272.5, 225.0, 200.0, 180.0]
    emitted_axis_q = []
    for canonical_deg in waypoints_canonical_deg:
        axis_q = backend._command_axis_q_for_joint_value(
            axis_i=0,
            logical_joint_idx=0,
            canonical_q=math.radians(canonical_deg),
            live_reference_counts=live_wire,
        )
        emitted_axis_q.append(axis_q)
    # Assert monotonic decrease (canonical values decrease, so axis_q with
    # sign=-1 increases toward zero, i.e. emitted values monotonically
    # DECREASE in magnitude for decreasing canonical).
    for prev, curr, prev_deg, curr_deg in zip(
        emitted_axis_q[:-1], emitted_axis_q[1:],
        waypoints_canonical_deg[:-1], waypoints_canonical_deg[1:],
    ):
        # Since canonical decreases and sign=-1, each axis_q should decrease too.
        assert curr < prev, (
            f"non-monotonic axis_q between waypoints "
            f"{prev_deg}° (axis_q={prev:.4f}) and {curr_deg}° (axis_q={curr:.4f}): "
            f"the Phase 5 whip pre-fix signature. Expected monotonic decrease."
        )
    # Also: each emitted axis_q must equal base_axis_q exactly (no turn-shift).
    for canonical_deg, axis_q in zip(waypoints_canonical_deg, emitted_axis_q):
        assert math.isclose(
            axis_q, math.radians(canonical_deg), abs_tol=1e-6
        ), (
            f"waypoint at canonical {canonical_deg}° emitted axis_q "
            f"{axis_q!r} (= {math.degrees(axis_q):.3f}°) instead of "
            f"{canonical_deg}° exactly. The fold introduced a turn-shift "
            f"when it should have passed canonical through directly."
        )


def test_a6ec_command_axis_q_falls_back_to_fold_without_anchor(
    monkeypatch, tmp_path
):
    """When no home anchor is captured (e.g., fresh boot, never homed), the
    direction-preserving path must NOT fire. The existing fold path
    (including its seam-only multi-turn disambiguation) remains the
    behavior. This preserves the 2026-04-19 UI-whip seam-crossing
    regression coverage for pre-commissioning joints.
    """
    # Set motor at canonical +180° (seam); multi-turn axis_q = -RM/2.
    # RM = counts_per_motor_rev(131072) * gear(10) = 1,310,720; half_rm = 655,360.
    backend, rm, half_rm, counts_per_unit = _setup_midpoint_home_fold_backend(
        monkeypatch, tmp_path, u40_20_motor_counts=-655_360
    )
    with backend._status_lock:
        backend._absolute_encoder_home_anchors = [None]  # NO anchor captured
    # Live at the high-side seam ambiguity (wire=RM-1, canonical ≈ ±180°).
    # The fold's seam disambiguation MUST still fire here because this is
    # the exact 2026-04-19 UI-whip reproduction.
    target_axis_q = backend._command_axis_q_for_joint_value(
        axis_i=0,
        logical_joint_idx=0,
        canonical_q=math.radians(185.0),
        live_reference_counts=rm - 1,
    )
    # Fold with seam disambiguation picks the short-path target (no +RM whip).
    assert math.isclose(
        target_axis_q, math.radians(185.0), abs_tol=1e-6
    ), (
        f"fold fallback must still do seam disambiguation at the high-side "
        f"ambiguous reading even without an anchor (2026-04-19 regression). "
        f"got {target_axis_q!r} rad ({math.degrees(target_axis_q):.3f}°), "
        f"expected +185.000°."
    )


def test_a6ec_multi_turn_reference_when_anchored_returns_none_without_anchor(
    monkeypatch, tmp_path
):
    """The stricter `_when_anchored` variant must return None when no home
    anchor has been captured, so the display path can cleanly fall back to
    the accumulated-unwrap semantics. (The permissive sibling would return
    motor-encoder-internal counts which are useless for operator display.)
    """
    backend, rm, half_rm, counts_per_unit = _setup_midpoint_home_fold_backend(
        monkeypatch, tmp_path, u40_20_motor_counts=123_456
    )
    with backend._status_lock:
        backend._absolute_encoder_home_anchors = [None]
    assert (
        backend._multi_turn_reference_counts_for_axis_when_anchored(0) is None
    ), (
        "_multi_turn_reference_counts_for_axis_when_anchored must return "
        "None when no anchor is captured; the permissive sibling "
        "_multi_turn_reference_counts_for_axis would return "
        "motor-encoder-internal counts here."
    )
    # Install an anchor and verify the helper now returns a value.
    with backend._status_lock:
        backend._absolute_encoder_home_anchors = [
            {"home_anchor_rad": 0.0, "source": "encoder_multi_turn_counts"}
        ]
    result = backend._multi_turn_reference_counts_for_axis_when_anchored(0)
    assert result == 123_456, (
        f"with anchor at 0 rad, helper must return motor_counts unchanged, "
        f"got {result}"
    )


def test_a6ec_last_seen_sidecar_surfaces_delta_when_joint_unchanged_while_off(
    monkeypatch, tmp_path
):
    # Scenario: drive was homed at HM35, a last-seen sidecar recorded
    # the U40.20 reading at that moment, drive was powered down, and
    # the joint did not move while off. On restart, the live U40.20 is
    # identical to the sidecar. The anchor path should still accept
    # truth (bit 15 is cleared, same restart-trust path as the earlier
    # regressions), and the new diagnostic fields
    # `last_seen_delta_counts=0` and
    # `last_seen_delta_physically_possible=True` should ride along on
    # the axis detail.
    joint_index = 5
    hm35_u40_20 = 120191
    backend, _ = _build_a6ec_restart_trust_test_backend(
        monkeypatch,
        tmp_path,
        joint_index=joint_index,
        statusword=0x1650,
        hm35_6064_counts=8,
        hm35_u40_20_counts=hm35_u40_20,
        live_6064_counts=9,
        live_u40_20_counts=hm35_u40_20,
        multi_turn_valid=True,
        last_seen_absolute_counts=hm35_u40_20,
        last_seen_reference_counts=9,
    )

    snapshot = backend._canonical_joint_positions_from_raw_feedback(
        {axis_i: int(backend._axis_counts[axis_i]) for axis_i in range(6)},
        reference_mode="raw",
    )
    target_detail = snapshot["axis_absolute_feedback"][joint_index]

    assert target_detail["truth_available"] is True
    assert target_detail["coordinate_system_valid"] is True
    assert (
        target_detail["drive_native_truth_verification_source"]
        == "persisted_home_anchor_agreement"
    )
    assert int(target_detail["last_seen_delta_counts"]) == 0
    assert target_detail["last_seen_delta_physically_possible"] is True
    assert int(target_detail["last_seen_absolute_counts"]) == hm35_u40_20
    # Delta budget is `32767 * counts_per_rev[J6]`; for a 2^17 CPR
    # encoder that is 131072 * 32767 = 4,294,836,224. Just assert it
    # is a positive integer, not the exact literal, so the test is
    # robust to future CPR changes.
    assert int(target_detail["last_seen_delta_budget_counts"]) > 0


def test_a6ec_last_seen_sidecar_upgrades_reason_on_impossible_delta(
    monkeypatch, tmp_path
):
    # Scenario: drive was homed, a last-seen sidecar was recorded, and
    # then encoder state was lost across a power cycle (battery died,
    # multi-turn overflow, etc.). The live U40.20 jumps by more than
    # 32767 motor revolutions from the stored value, which is
    # physically impossible during an off-window. The shaft-frame
    # consistency gate correctly fails; the new reason should upgrade
    # to `multi_turn_feedback_lost_across_power_cycle` so operators
    # see the specific encoder-retention failure class.
    joint_index = 5
    hm35_u40_20 = 120191
    # Intentionally larger than 32767 * 131072 so the physicality
    # check fails. The shaft-frame gate also fails because the
    # anchor/6064 relationship no longer holds.
    impossible_u40_20 = hm35_u40_20 + (32_768 * 131_072) + 7_000_000
    backend, _ = _build_a6ec_restart_trust_test_backend(
        monkeypatch,
        tmp_path,
        joint_index=joint_index,
        statusword=0x1650,
        hm35_6064_counts=8,
        hm35_u40_20_counts=hm35_u40_20,
        # Shaft-frame gate also disagrees mod-RM so the anchor path
        # rejects, matching the physical scenario of lost retention.
        live_6064_counts=8 + (131_072 // 3),
        live_u40_20_counts=impossible_u40_20,
        multi_turn_valid=True,
        last_seen_absolute_counts=hm35_u40_20,
        last_seen_reference_counts=8,
    )

    snapshot = backend._canonical_joint_positions_from_raw_feedback(
        {axis_i: int(backend._axis_counts[axis_i]) for axis_i in range(6)},
        reference_mode="raw",
    )
    target_detail = snapshot["axis_absolute_feedback"][joint_index]

    assert target_detail["truth_available"] is False
    assert target_detail["coordinate_system_valid"] is False
    # Drive-native reason upgrades to the specific label when the
    # sidecar says the delta is physically impossible.
    assert (
        target_detail["drive_native_truth_reason"]
        == "multi_turn_feedback_lost_across_power_cycle"
    )
    assert target_detail["last_seen_delta_physically_possible"] is False
    assert abs(int(target_detail["last_seen_delta_counts"])) > int(
        target_detail["last_seen_delta_budget_counts"]
    )


def test_a6ec_last_seen_sidecar_keeps_legacy_reason_when_absent(
    monkeypatch, tmp_path
):
    # Scenario: no last-seen sidecar recorded (older anchor file). The
    # shaft-frame gate fails - same setup as the existing
    # `rejects_sub_shaft_turn_drift` regression - so the validity helper
    # must fall back to the legacy `persisted_home_anchor_inconsistent
    # _with_live_6064` reason rather than inventing a retention story
    # without evidence.
    joint_index = 5
    cfg = Gradient05Config()
    counts_per_rev = int(cfg.actuator_encoder_counts_per_rev[joint_index])
    gear_ratio = float(cfg.actuator_gear_ratios[joint_index])
    rm = int(round(counts_per_rev * gear_ratio))
    backend, _ = _build_a6ec_restart_trust_test_backend(
        monkeypatch,
        tmp_path,
        joint_index=joint_index,
        statusword=0x1650,
        hm35_6064_counts=8,
        hm35_u40_20_counts=120191,
        live_6064_counts=8 + (rm // 3),
        live_u40_20_counts=120191,
        multi_turn_valid=True,
        # No last_seen_absolute_counts -> no sidecar on the anchor.
    )

    snapshot = backend._canonical_joint_positions_from_raw_feedback(
        {axis_i: int(backend._axis_counts[axis_i]) for axis_i in range(6)},
        reference_mode="raw",
    )
    target_detail = snapshot["axis_absolute_feedback"][joint_index]

    assert target_detail["truth_available"] is False
    assert target_detail["coordinate_system_valid"] is False
    assert target_detail["shaft_frame_consistent"] is False
    # Legacy reason, matching the existing restart-trust regression
    # (either the backend short-circuit or the drive_native_ prefixed
    # helper label). Must NOT upgrade to the W1 label without sidecar
    # evidence.
    assert target_detail["truth_reason"] in {
        "multi_turn_anchor_inconsistent_with_live_6064",
        "drive_native_persisted_home_anchor_inconsistent_with_live_6064",
    }
    # Extra defense-in-depth: none of the W1 sidecar fields should be
    # present when there is no sidecar on the anchor.
    assert "last_seen_delta_counts" not in target_detail
    assert "last_seen_delta_physically_possible" not in target_detail


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

    def _begin_trajectory(expected_points, axis_mask=None):
        captured["begin"] = (expected_points, axis_mask)
        return 55

    def _enqueue_trajectory_points(traj_id, points):
        captured["points"] = (traj_id, points)

    def _commit_trajectory(traj_id):
        captured["commit"] = traj_id

    def _wait_for_trajectory_complete(traj_id, *, timeout_s, submitted_command_seq=None):
        captured["wait"] = (traj_id, timeout_s, submitted_command_seq)
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
        axis_mask=0x20,
    )

    assert result is status
    assert captured["begin"] == (3, 0x20)
    assert captured["commit"] == 55
    points_traj_id, points_payload = captured["points"]
    assert points_traj_id == 55
    assert len(points_payload) == 3
    for idx, point in enumerate(points_payload):
        assert point["axis_mask"] == 0x20
        assert point["positions_rad"] == joint_path[idx]
        assert point["flags"] == _TRAJ_POINTF_HAS_VELOCITY
        assert point["t_from_start_ns"] == idx * 4_000_000
        assert point["qd"] == pytest.approx([25.0, 25.0, 0.0, 0.0, 0.0, 0.0])
    wait_traj_id, wait_timeout_s, wait_submitted_command_seq = captured["wait"]
    assert wait_traj_id == 55
    assert wait_timeout_s == pytest.approx(5.012)
    assert wait_submitted_command_seq is None
    assert backend.get_last_trajectory_timing() == {
        "requested_frequency_hz": 333,
        "effective_frequency_hz": 250,
        "cycle_ns": 1_000_000,
        "step_ns": 4_000_000,
        "cycles_per_point": 4,
    }


def test_ethercat_backend_execute_joint_trajectory_uses_quantized_timing_for_j5_axis_mask(
    monkeypatch, tmp_path
):
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
        active_traj_id=56,
        current_point_index=2,
        queue_depth=0,
        queue_capacity=4096,
        last_event_code=0,
        underrun_count=0,
        stale_command=False,
        motion_done=True,
        capability_flags=0,
        active_command_seq=124,
        last_update_ns=456,
    )

    def _begin_trajectory(expected_points, axis_mask=None):
        captured["begin"] = (expected_points, axis_mask)
        return 56

    def _enqueue_trajectory_points(traj_id, points):
        captured["points"] = (traj_id, points)

    def _commit_trajectory(traj_id):
        captured["commit"] = traj_id

    def _wait_for_trajectory_complete(traj_id, *, timeout_s, submitted_command_seq=None):
        captured["wait"] = (traj_id, timeout_s, submitted_command_seq)
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
        [0.0, 0.0, 0.0, 0.0, -0.004, 0.0],
        [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        [0.0, 0.0, 0.0, 0.0, 0.004, 0.0],
    ]

    result = backend.execute_joint_trajectory(
        joint_path,
        frequency=333,
        axis_mask=0x10,
    )

    assert result is status
    assert captured["begin"] == (3, 0x10)
    assert captured["commit"] == 56
    points_traj_id, points_payload = captured["points"]
    assert points_traj_id == 56
    assert len(points_payload) == 3
    for idx, point in enumerate(points_payload):
        assert point["axis_mask"] == 0x10
        assert point["positions_rad"] == joint_path[idx]
        assert point["flags"] == _TRAJ_POINTF_HAS_VELOCITY
        assert point["t_from_start_ns"] == idx * 4_000_000
        assert point["qd"] == pytest.approx([0.0, 0.0, 0.0, 0.0, 1.0, 0.0])
    wait_traj_id, wait_timeout_s, wait_submitted_command_seq = captured["wait"]
    assert wait_traj_id == 56
    assert wait_timeout_s == pytest.approx(5.012)
    assert wait_submitted_command_seq is None
    assert backend.get_last_trajectory_timing() == {
        "requested_frequency_hz": 333,
        "effective_frequency_hz": 250,
        "cycle_ns": 1_000_000,
        "step_ns": 4_000_000,
        "cycles_per_point": 4,
    }


def test_ethercat_backend_cmd_ring_write_waits_for_space():
    backend = EthercatRTCoreBackend(robot_config=Gradient05Config().get_config_dict())
    backend._cmd_shm = bytearray(256)
    backend._cmd_hdr = _ShmHeader(
        kind=1,
        num_axes=6,
        cycle_ns=1_000_000,
        topology_hash=0,
        ring_offset=0,
        ring_capacity=1,
        ring_msg_bytes=64,
        setpoint_offset=0,
    )
    backend._cmd_eventfd = None
    backend._cmd_seq = 11

    ring_hdr_off, ring_entries_off = backend._cmd_ring_offsets()
    backend._cmd_shm[ring_hdr_off : ring_hdr_off + _RING_HEADER_STRUCT.size] = _RING_HEADER_STRUCT.pack(
        _MAGIC_RING,
        1,
        64,
        1,
        0,
        0,
        0,
    )

    def _free_ring_slot() -> None:
        time.sleep(0.01)
        backend._cmd_shm[ring_hdr_off + 16 : ring_hdr_off + 20] = struct.pack("<I", 1)

    drainer = threading.Thread(target=_free_ring_slot)
    drainer.start()
    backend._cmd_ring_write(0x1234, b"abc")
    drainer.join(timeout=1.0)

    header = backend._cmd_shm[ring_hdr_off : ring_hdr_off + _RING_HEADER_STRUCT.size]
    magic, capacity, msg_bytes, write_idx, read_idx, dropped, reserved0 = _RING_HEADER_STRUCT.unpack(header)
    assert magic == _MAGIC_RING
    assert capacity == 1
    assert msg_bytes == 64
    assert write_idx == 2
    assert read_idx == 1
    assert dropped == 0

    msg_header = backend._cmd_shm[ring_entries_off : ring_entries_off + _MSG_HEADER_STRUCT.size]
    msg_type, flags, size_bytes, seq, time_ns = _MSG_HEADER_STRUCT.unpack(msg_header)
    assert msg_type == 0x1234
    assert flags == 0
    assert size_bytes == _MSG_HEADER_STRUCT.size + 3
    assert seq == 11
    assert time_ns > 0
    assert bytes(backend._cmd_shm[ring_entries_off + _MSG_HEADER_STRUCT.size : ring_entries_off + _MSG_HEADER_STRUCT.size + 3]) == b"abc"


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


def test_ethercat_backend_parses_jog_debug_status(monkeypatch, tmp_path):
    monkeypatch.setenv("GRADIENT_JOINT_ZERO_OFFSETS_PATH", str(tmp_path / "joint_zero_offsets.json"))
    backend = EthercatRTCoreBackend(robot_config=Gradient05Config().get_config_dict())

    payload = struct.pack(
        "<12I7Q16i16i16i16i",
        3,   # num_axes
        1,   # active_jog
        0x7,  # active_jog_axis_mask
        0x3,  # command_sp_mask
        0x7,  # have_hold_mask
        0x3,  # have_jog_target_mask
        0x1,  # snap_hold_mask
        0x7,  # latest_cmd_axis_mask
        0x1,  # latest_cmd_flags
        2,   # timeout
        0x3,  # last_stop_axis_mask
        0x2,  # stop_arrest_mask
        111,  # sample_time_ns
        222,  # active_jog_cmd_seq
        333,  # latest_jog_seq_seen
        444,  # active_jog_deadline_ns
        555,  # latest_cmd_timeout_ns
        666,  # last_stop_time_ns
        777,  # last_stop_cmd_seq
        *([10, 20, 30] + [0] * 13),
        *([11, 21, 31] + [0] * 13),
        *([12, 22, 32] + [0] * 13),
        *([13, 23, 33] + [0] * 13),
    )

    status = backend._parse_jog_debug_state(payload)

    assert isinstance(status, RTCoreJogDebugStatus)
    assert status.num_axes == 3
    assert status.active_jog is True
    assert status.active_jog_axis_mask == 0x7
    assert status.command_sp_mask == 0x3
    assert status.snap_hold_mask == 0x1
    assert status.stop_arrest_mask == 0x2
    assert status.last_stop_reason == 2
    assert status.last_stop_reason_name == "timeout"
    assert status.last_stop_cmd_seq == 777
    assert status.feedback_pos_counts[:3] == [10, 20, 30]
    assert status.hold_target_counts[:3] == [11, 21, 31]
    assert status.output_target_counts[:3] == [12, 22, 32]
    assert status.output_target_velocity_counts_per_s[:3] == [13, 23, 33]


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


def test_ethercat_backend_wait_for_short_trajectory_completion_without_observed_active_id(monkeypatch, tmp_path):
    monkeypatch.setenv("GRADIENT_JOINT_ZERO_OFFSETS_PATH", str(tmp_path / "joint_zero_offsets.json"))
    backend = EthercatRTCoreBackend(robot_config=Gradient05Config().get_config_dict())

    statuses = iter(
        [
            RTCoreExecutionStatus(
                active_mode=0,
                active_mode_name="idle",
                state=RTCORE_EXEC_STATE_IDLE,
                state_name="idle",
                active_traj_id=0,
                current_point_index=None,
                queue_depth=0,
                queue_capacity=4096,
                last_event_code=0,
                underrun_count=0,
                stale_command=False,
                motion_done=True,
                capability_flags=0,
                active_command_seq=200,
                last_update_ns=1000,
            ),
            RTCoreExecutionStatus(
                active_mode=0,
                active_mode_name="idle",
                state=RTCORE_EXEC_STATE_IDLE,
                state_name="idle",
                active_traj_id=0,
                current_point_index=None,
                queue_depth=0,
                queue_capacity=4096,
                last_event_code=291,
                underrun_count=0,
                stale_command=False,
                motion_done=True,
                capability_flags=0,
                active_command_seq=205,
                last_update_ns=1100,
            ),
        ]
    )

    monkeypatch.setattr(backend, "get_execution_status", lambda: next(statuses))
    monkeypatch.setattr(time, "sleep", lambda _seconds: None)

    result = backend.wait_for_trajectory_complete(
        3,
        timeout_s=0.1,
        submitted_command_seq=205,
    )

    assert result.active_traj_id == 0
    assert result.state_name == "idle"
    assert result.active_command_seq == 205


def test_ethercat_backend_wait_for_trajectory_complete_waits_past_queue_empty_executing_snapshot(
    monkeypatch, tmp_path
):
    monkeypatch.setenv("GRADIENT_JOINT_ZERO_OFFSETS_PATH", str(tmp_path / "joint_zero_offsets.json"))
    backend = EthercatRTCoreBackend(robot_config=Gradient05Config().get_config_dict())

    statuses = iter(
        [
            RTCoreExecutionStatus(
                active_mode=2,
                active_mode_name="trajectory",
                state=3,
                state_name="executing",
                active_traj_id=4,
                current_point_index=22,
                queue_depth=1,
                queue_capacity=4096,
                last_event_code=290,
                underrun_count=0,
                stale_command=False,
                motion_done=False,
                capability_flags=0,
                active_command_seq=133,
                last_update_ns=1000,
            ),
            RTCoreExecutionStatus(
                active_mode=2,
                active_mode_name="trajectory",
                state=3,
                state_name="executing",
                active_traj_id=4,
                current_point_index=24,
                queue_depth=0,
                queue_capacity=4096,
                last_event_code=290,
                underrun_count=0,
                stale_command=False,
                motion_done=False,
                capability_flags=0,
                active_command_seq=133,
                last_update_ns=1100,
            ),
            RTCoreExecutionStatus(
                active_mode=2,
                active_mode_name="trajectory",
                state=RTCORE_EXEC_STATE_COMPLETED,
                state_name="completed",
                active_traj_id=4,
                current_point_index=24,
                queue_depth=0,
                queue_capacity=4096,
                last_event_code=291,
                underrun_count=0,
                stale_command=False,
                motion_done=True,
                capability_flags=0,
                active_command_seq=133,
                last_update_ns=1200,
            ),
        ]
    )

    monkeypatch.setattr(backend, "get_execution_status", lambda: next(statuses))
    monkeypatch.setattr(time, "sleep", lambda _seconds: None)

    result = backend.wait_for_trajectory_complete(
        4,
        timeout_s=0.1,
        submitted_command_seq=133,
    )

    assert result.active_traj_id == 4
    assert result.state_name == "completed"
    assert result.motion_done is True


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


def test_ethercat_backend_stop_joint_velocity_lease_jog_can_request_quick_stop(monkeypatch, tmp_path):
    monkeypatch.setenv("GRADIENT_JOINT_ZERO_OFFSETS_PATH", str(tmp_path / "joint_zero_offsets.json"))
    backend = EthercatRTCoreBackend(robot_config=Gradient05Config().get_config_dict())
    backend._connected = True
    backend._rt_num_axes = 2

    captured: list[tuple[int, bytes]] = []
    monkeypatch.setattr(backend, "_cmd_ring_write", lambda msg_type, payload: captured.append((msg_type, payload)))

    backend.stop_joint_velocity_lease_jog(quick_stop=True)

    assert len(captured) == 1
    msg_type, payload = captured[0]
    assert msg_type == 0x0130
    axis_mask, flags, timeout_ns, *qd = struct.unpack("<IIQ16d", payload)
    assert axis_mask == 0
    assert flags == 0x6
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

