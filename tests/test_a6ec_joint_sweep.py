"""Parametrised per-joint A6-EC sweep regressions.

Context
-------

After the 2026-04-16 vendor realignment + 2026-04-17 persisted-home-anchor
restart-trust workstream, the encoder-to-``607A`` command path is
deterministic: every write folds against live ``6064``, truth fails closed
on mod-``RM`` disagreement, and the host carries no per-axis wrap-lift
state. That means a pure-Python test can traverse every joint's full
``logical_joint_limits_rad`` range and verify the shaft-frame / nearest-
turn behaviour without RTCore or hardware.

These tests exercise, for each of ``J1..J6``:

- the fresh-HM trust path (``statusword=0x9650``, vendor Q5/Q6 signature);
- the post-power-cycle trust path (``statusword=0x1650`` with the
  persisted-home-anchor gate in play);
- explicit failure injections (anchor drift, missing anchor, invalid
  multi-turn, sub-shaft-turn drift, oversized trajectory step).

The seeding pattern intentionally mirrors
``_build_a6ec_restart_trust_test_backend`` in
``tests/test_gradient05_limits_and_backends.py`` so both test files share
the same mental model of "how the backend consumes live feedback".

Test-hygiene invariants (see
``docs/ethercat/a6ec-frame-semantics-and-native-home.md`` and the
2026-04-17 scratchpad note):

- Every test MUST isolate ``GRADIENT_ABSOLUTE_ENCODER_ANCHORS_PATH`` to
  ``tmp_path`` so the repo-root anchor file cannot silently satisfy the
  persisted-anchor path and mask a missing-anchor scenario.
- Tests that exercise the persisted-anchor trust path must populate
  ``backend._absolute_encoder_home_anchors[joint_i]`` after the backend
  is constructed (the constructor loads the anchor file exactly once).
- Post-power-cycle mode requires ``statusword=0x1650`` (or another
  bit-15-cleared fault-free value). ``0x9650`` will take the fresh-HM
  signature path and NOT exercise the persisted-anchor logic.
"""

from __future__ import annotations

import math

import pytest

from gradient_os.arm_controller.backends.ethercat_rtcore.backend import (
    EthercatRTCoreBackend,
    _AbsoluteFeedbackAxisMetrics,
)
from gradient_os.arm_controller.robots.gradient05.config import Gradient05Config

# ---------------------------------------------------------------------------
# Shared constants
# ---------------------------------------------------------------------------

_JOINT_INDICES = list(range(6))
_FRESH_HM_STATUSWORD = 0x9650
_POST_CYCLE_STATUSWORD = 0x1650
_COUNTS_PER_REV_REF = 131072


# ---------------------------------------------------------------------------
# Per-joint scaling helper
# ---------------------------------------------------------------------------


def _axis_scaling_for_joint(joint_index: int) -> dict[str, float]:
    """Return per-joint scaling values sourced from the live robot config."""
    cfg = Gradient05Config()
    counts_per_rev = int(cfg.actuator_encoder_counts_per_rev[joint_index])
    gear_ratio = float(cfg.actuator_gear_ratios[joint_index])
    sign = int(cfg.actuator_position_signs[joint_index])
    counts_per_unit = (float(counts_per_rev) * gear_ratio) / (2.0 * math.pi)
    rm = int(round(counts_per_unit * 2.0 * math.pi))
    lim_min, lim_max = cfg.logical_joint_limits_rad[joint_index]
    return {
        "counts_per_rev": counts_per_rev,
        "gear_ratio": gear_ratio,
        "sign": sign,
        "counts_per_unit": counts_per_unit,
        "rm": rm,
        "lim_min": float(lim_min),
        "lim_max": float(lim_max),
    }


def _counts_for_canonical_q(
    q_rad: float,
    *,
    sign: int,
    counts_per_unit: float,
    rm: int,
) -> tuple[int, int]:
    """Return (wire_6064, continuous_u40) for a given canonical q with anchor=0.

    The wire value is always folded into ``[0, RM)`` to match the drive's
    absolute-rotation-mode reporting; the U40.20/.22 counterpart is
    continuous (not wrapped) because the real drive exposes it that way
    in motor-encoder space.
    """
    continuous = int(round(float(q_rad) * float(sign) * float(counts_per_unit)))
    wire = continuous % int(rm)
    if wire < 0:
        wire += int(rm)
    return wire, continuous


def _u40_low_high_signed_i32(continuous: int) -> tuple[int, int]:
    """Split a continuous signed i64 into (low_i32, high_i32) as the drive reports.

    The A6-EC exposes U40.20/.22 as two signed int32s whose combination
    (``(high << 32) | (low & 0xFFFFFFFF)``) is the continuous multi-turn
    counter. For negative continuous values, both words carry the sign
    bit. Returning the two components in signed form keeps the JSON/
    feedback shape faithful to what the live drive produces.
    """
    low_u = int(continuous) & 0xFFFFFFFF
    high_u = (int(continuous) >> 32) & 0xFFFFFFFF
    low_signed = low_u - (1 << 32) if low_u >= (1 << 31) else low_u
    high_signed = high_u - (1 << 32) if high_u >= (1 << 31) else high_u
    return int(low_signed), int(high_signed)


# ---------------------------------------------------------------------------
# Sweep sample generator
# ---------------------------------------------------------------------------


def _sample_sweep_for_joint(joint_index: int) -> list[float]:
    """Return a sorted list of canonical_q samples covering the full limit range.

    The list always includes:
    - ``lim_min`` and ``lim_max`` themselves;
    - ``~20`` evenly spaced interior samples (every ~``(lim_max - lim_min) / 20``);
    - explicit seam-adjacent samples where the wire-frame 6064 would
      sit within ``{1, 16, 128}`` counts of the RM boundary on either side.
    """
    scaling = _axis_scaling_for_joint(joint_index)
    sign = scaling["sign"]
    cpu = scaling["counts_per_unit"]
    rm = scaling["rm"]
    lim_min = scaling["lim_min"]
    lim_max = scaling["lim_max"]

    # Even-spacing pass.
    span = lim_max - lim_min
    samples: list[float] = []
    for i in range(21):
        samples.append(lim_min + span * i / 20.0)

    # Seam-adjacent additions: find q values that place the wire reading
    # exactly ``offset`` counts either side of 0 in the [0, RM) fold. Note
    # that sign flips the relationship between q and wire counts.
    for offset in (1, 16, 128):
        for target_wire in (offset, rm - offset):
            delta_counts = float(target_wire)
            q_candidate = delta_counts / (float(sign) * float(cpu))
            if lim_min <= q_candidate <= lim_max:
                samples.append(q_candidate)
            # Also try the seam from the other direction with a whole-shaft
            # turn baked in so we cover multi-turn-capable joints properly.
            for turn in (-1, 1):
                q_candidate_turn = q_candidate + (2.0 * math.pi * turn) / float(sign)
                if lim_min <= q_candidate_turn <= lim_max:
                    samples.append(q_candidate_turn)

    # Deduplicate while preserving sorted order for continuity asserts.
    seen: set[float] = set()
    unique: list[float] = []
    for q in sorted(samples):
        key = round(q, 9)
        if key in seen:
            continue
        seen.add(key)
        unique.append(q)
    return unique


# ---------------------------------------------------------------------------
# Backend builder
# ---------------------------------------------------------------------------


def _build_sweep_backend(
    monkeypatch: pytest.MonkeyPatch,
    tmp_path,
    *,
    joint_index: int,
    statusword: int,
    canonical_q: float,
    anchor_offset_counts: int = 0,
    anchor_present: bool = True,
    multi_turn_valid: bool = True,
    live_6064_override: int | None = None,
    live_u40_override: int | None = None,
) -> tuple[EthercatRTCoreBackend, dict[str, object]]:
    """Build a backend staged for the given scenario on a single joint.

    - ``canonical_q``: target planner-space q for the joint under test.
      The helper seeds 6064/U40.20/.22 such that, with anchor=0 and
      master_offset=0, the backend's truth path recovers exactly this q.
    - ``anchor_offset_counts``: extra wire counts baked into the anchor
      so a non-zero stored anchor can be tested. The helper adjusts the
      U40 value so the *intended* canonical_q stays at the argument's
      value; anchor-drift tests override this.
    - ``anchor_present``: if False, the anchor entry is cleared so the
      persisted-anchor path surfaces ``persisted_home_anchor_missing``.
    - ``multi_turn_valid``: if False, the seeded ``encoder_multi_turn_*``
      fields are written with ``valid=0``.
    - ``live_6064_override``/``live_u40_override``: inject values that
      deliberately disagree with the derived q (used by sub-shaft-turn
      drift and anchor-drift scenarios).
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

    # Sizing for the joint under test.
    scaling = _axis_scaling_for_joint(joint_index)
    sign = scaling["sign"]
    cpu = scaling["counts_per_unit"]
    rm = int(scaling["rm"])

    # Derive the expected live readings for the target canonical_q with
    # anchor offset baked in. The anchor is captured as
    #   anchor_rad = absolute_axis_q_hm35 - reference_q_hm35.
    # To keep the sweep math clean we pick reference_q_hm35 = 0 and
    # absolute_axis_q_hm35 = anchor_rad, giving anchor_offset_counts =
    # round(anchor_rad * sign * cpu). That way the live U40 needed for
    # truth == canonical_q is round((canonical_q + anchor_rad) * sign * cpu).
    anchor_rad = float(anchor_offset_counts) / (float(sign) * float(cpu)) if cpu > 0.0 else 0.0
    target_wire, continuous_at_q = _counts_for_canonical_q(
        canonical_q, sign=sign, counts_per_unit=cpu, rm=rm
    )
    live_u40 = continuous_at_q + int(anchor_offset_counts)
    live_6064 = int(target_wire)
    if live_6064_override is not None:
        live_6064 = int(live_6064_override)
    if live_u40_override is not None:
        live_u40 = int(live_u40_override)

    metrics_axes: list[dict[str, object]] = []
    for axis_i, gear_ratio in enumerate(robot_cfg["actuator_gear_ratios"][:6]):
        if axis_i == joint_index:
            counts = live_6064
            mt_counts = live_u40
            sw = int(statusword)
            if anchor_present:
                backend._absolute_encoder_home_anchors[joint_index] = {
                    "home_anchor_rad": float(anchor_rad),
                    "source": "pytest",
                    "axis_indices": [joint_index],
                }
            else:
                backend._absolute_encoder_home_anchors[joint_index] = None
            mt_valid_flag = 1 if multi_turn_valid else 0
        else:
            # Quiet, clean axes so the target-joint assertions are not
            # entangled with other joints' truth states.
            counts = 0
            mt_counts = 0
            sw = _FRESH_HM_STATUSWORD
            backend._absolute_encoder_home_anchors[axis_i] = {
                "home_anchor_rad": 0.0,
                "source": "pytest",
                "axis_indices": [axis_i],
            }
            mt_valid_flag = 1
        backend._axis_counts[axis_i] = int(counts)
        mt_low, mt_high = _u40_low_high_signed_i32(int(mt_counts))
        rot_low, rot_high = _u40_low_high_signed_i32(int(counts))
        backend._absolute_feedback_by_axis[axis_i] = _AbsoluteFeedbackAxisMetrics.from_mapping(
            {
                "encoder_multi_turn_low": {"valid": mt_valid_flag, "value": mt_low},
                "encoder_multi_turn_high": {"valid": mt_valid_flag, "value": mt_high},
                "absolute_position_reference": {"valid": 1, "value": int(counts)},
                "rotation_mode_position_reference": {"valid": 1, "value": int(counts)},
                "rotation_mode_encoder_low": {"valid": 1, "value": rot_low},
                "rotation_mode_encoder_high": {"valid": 1, "value": rot_high},
            }
        )
        # Match the existing test-scaffold pattern for RTCore metrics so
        # the drive startup config gate passes.
        metrics_axes.append(
            {
                "statusword": sw,
                "error_code": 0,
                "manufacturer_error_code": 0,
                "startup_drive_config": _startup_drive_config_entry(gear_ratio),
                "startup_drive_configs": _startup_drive_config_entries(gear_ratio),
                "slave_online": 1,
                "slave_operational": 1,
                "slave_al_state": 8,
            }
        )
    monkeypatch.setattr(backend, "_load_rtcore_metrics_snapshot", lambda: {"axes": metrics_axes})
    return backend, {
        "anchor_rad": anchor_rad,
        "live_6064": live_6064,
        "live_u40": live_u40,
        "scaling": scaling,
    }


def _startup_drive_config_entries(raw_ratio: object) -> list[dict[str, object]]:
    # Avoid importing from the sibling test module; keep a local copy with
    # the exact shape the backend startup-config decoder expects.
    from fractions import Fraction

    ratio = Fraction(str(raw_ratio))
    numerator = int(ratio.numerator)
    denominator = int(ratio.denominator)
    return [
        {
            "setting_key": "a6ec_encoder_position_tracking_mode",
            "configured": 1,
            "commanded": 4,
            "readback_valid": 1,
            "readback": 4,
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
    ]


def _startup_drive_config_entry(raw_ratio: object) -> dict[str, object]:
    return _startup_drive_config_entries(raw_ratio)[0]


def _j_detail(snapshot: dict[str, object], joint_index: int) -> dict[str, object]:
    absfb = snapshot["axis_absolute_feedback"]
    assert isinstance(absfb, list)
    for detail in absfb:
        if isinstance(detail, dict) and int(detail.get("axis", -1)) == joint_index:
            return detail
    raise AssertionError(f"joint detail missing for joint_index={joint_index}")


def _run_truth_snapshot(backend: EthercatRTCoreBackend, reference_mode: str = "raw") -> dict[str, object]:
    return backend._canonical_joint_positions_from_raw_feedback(
        {axis_i: int(backend._axis_counts[axis_i]) for axis_i in range(6)},
        reference_mode=reference_mode,
    )


# ---------------------------------------------------------------------------
# Fresh-HM sweep
# ---------------------------------------------------------------------------


@pytest.mark.parametrize("joint_index", _JOINT_INDICES)
def test_a6ec_joint_full_range_sweep_fresh_hm_keeps_truth_continuous(
    monkeypatch, tmp_path, joint_index
):
    """Full-range sweep with statusword=0x9650 recovers canonical_q continuously."""
    scaling = _axis_scaling_for_joint(joint_index)
    cpu = scaling["counts_per_unit"]
    rm = int(scaling["rm"])
    sign = scaling["sign"]

    previous_q: float | None = None
    for q in _sample_sweep_for_joint(joint_index):
        backend, _ctx = _build_sweep_backend(
            monkeypatch,
            tmp_path,
            joint_index=joint_index,
            statusword=_FRESH_HM_STATUSWORD,
            canonical_q=q,
        )
        snapshot = _run_truth_snapshot(backend)
        detail = _j_detail(snapshot, joint_index)

        assert detail["truth_available"] is True, (
            f"joint={joint_index} q={q:.6f} should be truth-available but was not: {detail}"
        )
        assert detail["drive_native_truth_verification_source"] == "statusword_bits12_15_clear13"
        assert detail["shaft_frame_consistent"] is True

        recovered_q = float(detail["canonical_rad"])
        # canonical_q recovery tolerance: up to the per-count resolution.
        per_count_rad = 1.0 / float(cpu)
        assert abs(recovered_q - q) <= 2.0 * per_count_rad, (
            f"joint={joint_index} q={q:.6f} recovered={recovered_q:.6f} per_count={per_count_rad:.6f}"
        )

        # Nearest-turn 607A round-trip: the wire target should land on the
        # live 6064 exactly (to within rounding).
        target_axis_q = backend._command_axis_q_for_joint_value(
            axis_i=joint_index,
            logical_joint_idx=joint_index,
            canonical_q=recovered_q,
        )
        wire_counts = int(round(target_axis_q * float(sign) * float(cpu)))
        live_6064 = int(backend._axis_counts[joint_index])
        assert abs(wire_counts - live_6064) <= 1, (
            f"joint={joint_index} q={q:.6f} wire_counts={wire_counts} live_6064={live_6064}"
        )

        # Continuity vs the previous sample (no wrap-class jumps).
        if previous_q is not None:
            assert abs(recovered_q - previous_q) < math.pi / 2.0 + abs(q - previous_q) + 1e-6, (
                f"joint={joint_index} q_prev={previous_q:.6f} q_cur={q:.6f} "
                f"recovered_cur={recovered_q:.6f}"
            )
        previous_q = recovered_q
        # RM sanity belt-and-suspenders: the live reading is always inside one
        # shaft revolution.
        assert 0 <= live_6064 < rm, (
            f"joint={joint_index} live_6064={live_6064} RM={rm}"
        )


@pytest.mark.parametrize("joint_index", _JOINT_INDICES)
def test_a6ec_joint_sweep_fresh_hm_small_jog_stays_within_half_rm(
    monkeypatch, tmp_path, joint_index
):
    """A +0.5 deg canonical jog at every seam sample stays within half-RM on the wire."""
    scaling = _axis_scaling_for_joint(joint_index)
    cpu = scaling["counts_per_unit"]
    rm = int(scaling["rm"])
    sign = scaling["sign"]
    jog_rad = math.radians(0.5)
    expected_step_counts = cpu * jog_rad

    for q in _sample_sweep_for_joint(joint_index):
        backend, _ctx = _build_sweep_backend(
            monkeypatch,
            tmp_path,
            joint_index=joint_index,
            statusword=_FRESH_HM_STATUSWORD,
            canonical_q=q,
        )
        live_6064 = int(backend._axis_counts[joint_index])
        target_q = q + jog_rad
        target_axis_q = backend._command_axis_q_for_joint_value(
            axis_i=joint_index,
            logical_joint_idx=joint_index,
            canonical_q=target_q,
        )
        wire_counts = int(round(target_axis_q * float(sign) * float(cpu)))
        delta = wire_counts - live_6064
        assert abs(delta) <= rm // 2, (
            f"joint={joint_index} q={q:.6f} delta={delta} RM/2={rm // 2}"
        )
        # Magnitude of the step should match the expected jog modulo the
        # per-count rounding band.
        assert abs(abs(delta) - expected_step_counts) < 2.0, (
            f"joint={joint_index} q={q:.6f} delta={delta} expected~+/-{expected_step_counts:.1f}"
        )


@pytest.mark.parametrize("joint_index", _JOINT_INDICES)
def test_a6ec_joint_sweep_fresh_hm_trajectory_pre_commit_gate_accepts_monotone(
    monkeypatch, tmp_path, joint_index
):
    """A 20-point contiguous trajectory across a seam must NOT trip the half-RM gate.

    Models the real trajectory upload path: ``live_6064`` is frozen at the
    first point, and the nearest-turn fold runs against that fixed
    snapshot for every subsequent point. The resulting ``axis_q`` sequence
    stays continuous and the per-step wire delta is bounded by the jog
    spacing.
    """
    scaling = _axis_scaling_for_joint(joint_index)
    lim_min = scaling["lim_min"]
    lim_max = scaling["lim_max"]

    # Pick a small monotone sweep centred at the joint midpoint. Step size
    # is small (0.25 deg) so any wrap-class step between consecutive points
    # would be the sign of a command-frame bug, not a legitimate motion.
    n_points = 20
    step_rad = math.radians(0.25)
    midpoint = 0.5 * (lim_min + lim_max)
    points = [midpoint + (i - n_points // 2) * step_rad for i in range(n_points)]
    points = [p for p in points if lim_min <= p <= lim_max]

    # Stage the backend ONCE at the first trajectory point (mirrors a real
    # upload where 6064 is read once). Do NOT restage live_6064 per point;
    # the fold is supposed to run against a frozen snapshot.
    backend, _ctx = _build_sweep_backend(
        monkeypatch,
        tmp_path,
        joint_index=joint_index,
        statusword=_FRESH_HM_STATUSWORD,
        canonical_q=points[0],
    )

    previous_axis_counts: list[int | None] = [None] * backend._rt_num_axes
    for idx, q in enumerate(points):
        axis_q_vector = [0.0] * backend._rt_num_axes
        axis_q_vector[joint_index] = backend._command_axis_q_for_joint_value(
            axis_i=joint_index,
            logical_joint_idx=joint_index,
            canonical_q=q,
        )
        backend._enforce_trajectory_step_within_half_rm(
            axis_q=axis_q_vector,
            axis_mask=(1 << joint_index),
            previous_axis_counts=previous_axis_counts,
            point_index=idx,
            traj_id=7,
        )


@pytest.mark.parametrize("joint_index", _JOINT_INDICES)
def test_a6ec_joint_sweep_fresh_hm_trajectory_pre_commit_gate_rejects_whole_turn_jump(
    monkeypatch, tmp_path, joint_index
):
    """Two trajectory points differing by one shaft revolution raise command_frame_oversized_step."""
    scaling = _axis_scaling_for_joint(joint_index)
    cpu = scaling["counts_per_unit"]
    sign = scaling["sign"]
    rm = int(scaling["rm"])

    # Use two axis_q values that map to the SAME wire-frame counts mod RM
    # but are one shaft revolution apart in the continuous frame. That is
    # exactly the "off by one turn" command-frame bug the gate must catch.
    base_axis_q = 0.0
    turn_axis_q = float(rm) / (float(sign) * float(cpu))
    backend, _ctx = _build_sweep_backend(
        monkeypatch,
        tmp_path,
        joint_index=joint_index,
        statusword=_FRESH_HM_STATUSWORD,
        canonical_q=0.0,
    )

    previous_axis_counts: list[int | None] = [None] * backend._rt_num_axes
    axis_q_vec_first = [0.0] * backend._rt_num_axes
    axis_q_vec_first[joint_index] = base_axis_q
    backend._enforce_trajectory_step_within_half_rm(
        axis_q=axis_q_vec_first,
        axis_mask=(1 << joint_index),
        previous_axis_counts=previous_axis_counts,
        point_index=0,
        traj_id=11,
    )
    axis_q_vec_second = [0.0] * backend._rt_num_axes
    axis_q_vec_second[joint_index] = turn_axis_q
    with pytest.raises(RuntimeError, match="command_frame_oversized_step"):
        backend._enforce_trajectory_step_within_half_rm(
            axis_q=axis_q_vec_second,
            axis_mask=(1 << joint_index),
            previous_axis_counts=previous_axis_counts,
            point_index=1,
            traj_id=11,
        )


# ---------------------------------------------------------------------------
# Post-power-cycle sweep (persisted-anchor trust path)
# ---------------------------------------------------------------------------


@pytest.mark.parametrize("joint_index", _JOINT_INDICES)
def test_a6ec_joint_sweep_post_power_cycle_uses_persisted_anchor_path(
    monkeypatch, tmp_path, joint_index
):
    """Same sweep with statusword=0x1650 must validate via persisted-anchor agreement."""
    scaling = _axis_scaling_for_joint(joint_index)
    cpu = scaling["counts_per_unit"]

    for q in _sample_sweep_for_joint(joint_index):
        backend, _ctx = _build_sweep_backend(
            monkeypatch,
            tmp_path,
            joint_index=joint_index,
            statusword=_POST_CYCLE_STATUSWORD,
            canonical_q=q,
        )
        snapshot = _run_truth_snapshot(backend)
        detail = _j_detail(snapshot, joint_index)

        assert detail["truth_available"] is True, (
            f"joint={joint_index} q={q:.6f} post-power-cycle should still be trusted: {detail}"
        )
        assert detail["statusword_hex"] == "0x1650"
        assert detail["drive_native_truth_signature_valid"] is False
        assert detail["coordinate_system_valid"] is True
        assert detail["drive_native_truth_verification_source"] == "persisted_home_anchor_agreement"
        assert detail["persisted_home_anchor_present"] is True
        assert detail["multi_turn_feedback_valid"] is True
        assert detail["persisted_home_anchor_consistent"] is True
        assert detail["shaft_frame_consistent"] is True

        per_count_rad = 1.0 / float(cpu)
        assert abs(float(detail["canonical_rad"]) - q) <= 2.0 * per_count_rad


# ---------------------------------------------------------------------------
# Failure injections
# ---------------------------------------------------------------------------


@pytest.mark.parametrize("joint_index", _JOINT_INDICES)
def test_a6ec_joint_sweep_flags_mid_sweep_anchor_drift(
    monkeypatch, tmp_path, joint_index
):
    """With fresh-HM statusword, an RM/3 anchor offset must fail closed."""
    scaling = _axis_scaling_for_joint(joint_index)
    cpu = scaling["counts_per_unit"]
    rm = int(scaling["rm"])

    # Pick a mid-range q and shift the stored anchor by RM/3 counts.
    midpoint = 0.5 * (scaling["lim_min"] + scaling["lim_max"])
    drift_counts = rm // 3
    backend, _ctx = _build_sweep_backend(
        monkeypatch,
        tmp_path,
        joint_index=joint_index,
        statusword=_FRESH_HM_STATUSWORD,
        canonical_q=midpoint,
        anchor_offset_counts=drift_counts,
        live_u40_override=_counts_for_canonical_q(
            midpoint, sign=scaling["sign"], counts_per_unit=cpu, rm=rm
        )[1],  # U40 stays at the honest continuous value (ignoring the anchor drift)
    )
    snapshot = _run_truth_snapshot(backend)
    detail = _j_detail(snapshot, joint_index)

    assert detail["truth_available"] is False
    # Workstream 3 short-circuit names this condition directly.
    assert detail["truth_reason"] == "multi_turn_anchor_inconsistent_with_live_6064"
    mod_delta = abs(float(detail["shaft_frame_mod_rm_delta_counts"]))
    assert abs(mod_delta - drift_counts) <= 2.0, (
        f"joint={joint_index} mod_delta={mod_delta} expected~{drift_counts}"
    )


@pytest.mark.parametrize("joint_index", _JOINT_INDICES)
def test_a6ec_joint_sweep_accepts_whole_shaft_turn_anchor_offset(
    monkeypatch, tmp_path, joint_index
):
    """A whole-shaft-turn anchor offset is legitimate multi-turn state."""
    scaling = _axis_scaling_for_joint(joint_index)
    rm = int(scaling["rm"])

    midpoint = 0.5 * (scaling["lim_min"] + scaling["lim_max"])
    backend, _ctx = _build_sweep_backend(
        monkeypatch,
        tmp_path,
        joint_index=joint_index,
        statusword=_FRESH_HM_STATUSWORD,
        canonical_q=midpoint,
        anchor_offset_counts=rm,  # exactly one shaft revolution off
    )
    snapshot = _run_truth_snapshot(backend)
    detail = _j_detail(snapshot, joint_index)

    assert detail["truth_available"] is True
    assert detail["shaft_frame_consistent"] is True
    # The gate's wrap_turns captures the legitimate full-turn offset.
    assert abs(int(detail.get("shaft_frame_wrap_turns", 0))) <= 1


@pytest.mark.parametrize("joint_index", _JOINT_INDICES)
def test_a6ec_joint_sweep_post_cycle_rejects_missing_anchor(
    monkeypatch, tmp_path, joint_index
):
    """Post-power-cycle without a persisted anchor must refuse truth."""
    scaling = _axis_scaling_for_joint(joint_index)
    midpoint = 0.5 * (scaling["lim_min"] + scaling["lim_max"])
    backend, _ctx = _build_sweep_backend(
        monkeypatch,
        tmp_path,
        joint_index=joint_index,
        statusword=_POST_CYCLE_STATUSWORD,
        canonical_q=midpoint,
        anchor_present=False,
    )
    snapshot = _run_truth_snapshot(backend)
    detail = _j_detail(snapshot, joint_index)

    assert detail["truth_available"] is False
    # Anchor missing short-circuits the canonical_q computation earlier
    # than the validity helper (the backend raises the profile-level
    # "drive_native_absolute_home_anchor_missing" reason).
    assert detail["truth_reason"] in {
        "drive_native_absolute_home_anchor_missing",
        "drive_native_persisted_home_anchor_missing",
    }


@pytest.mark.parametrize("joint_index", _JOINT_INDICES)
def test_a6ec_joint_sweep_post_cycle_rejects_invalid_multi_turn(
    monkeypatch, tmp_path, joint_index
):
    """Invalid U40.20/.22 under post-power-cycle statusword must refuse truth."""
    scaling = _axis_scaling_for_joint(joint_index)
    midpoint = 0.5 * (scaling["lim_min"] + scaling["lim_max"])
    backend, _ctx = _build_sweep_backend(
        monkeypatch,
        tmp_path,
        joint_index=joint_index,
        statusword=_POST_CYCLE_STATUSWORD,
        canonical_q=midpoint,
        multi_turn_valid=False,
    )
    snapshot = _run_truth_snapshot(backend)
    detail = _j_detail(snapshot, joint_index)

    assert detail["truth_available"] is False
    # With multi-turn invalid we fall back to the "absolute feedback
    # unavailable" short-circuit rather than reaching the validity helper.
    assert detail["truth_reason"] in {
        "drive_native_absolute_feedback_unavailable",
        "drive_native_multi_turn_feedback_invalid",
    }


@pytest.mark.parametrize("joint_index", _JOINT_INDICES)
def test_a6ec_joint_sweep_post_cycle_rejects_sub_shaft_turn_drift(
    monkeypatch, tmp_path, joint_index
):
    """Post-power-cycle anchor disagreement (RM/3) must fail closed."""
    scaling = _axis_scaling_for_joint(joint_index)
    cpu = scaling["counts_per_unit"]
    rm = int(scaling["rm"])
    midpoint = 0.5 * (scaling["lim_min"] + scaling["lim_max"])

    # Shift 6064 by RM/3 without moving U40 — the encoder-data-loss
    # fingerprint. The mod-RM gate must refuse this.
    backend, _ctx = _build_sweep_backend(
        monkeypatch,
        tmp_path,
        joint_index=joint_index,
        statusword=_POST_CYCLE_STATUSWORD,
        canonical_q=midpoint,
        live_u40_override=_counts_for_canonical_q(
            midpoint, sign=scaling["sign"], counts_per_unit=cpu, rm=rm
        )[1],
        live_6064_override=(_counts_for_canonical_q(
            midpoint, sign=scaling["sign"], counts_per_unit=cpu, rm=rm
        )[0] + rm // 3) % rm,
    )
    snapshot = _run_truth_snapshot(backend)
    detail = _j_detail(snapshot, joint_index)

    assert detail["truth_available"] is False
    assert detail["shaft_frame_consistent"] is False
    assert detail["truth_reason"] in {
        "multi_turn_anchor_inconsistent_with_live_6064",
        "drive_native_persisted_home_anchor_inconsistent_with_live_6064",
    }
