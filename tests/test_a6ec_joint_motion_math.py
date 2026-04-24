"""Unit tests for the self-contained A6-EC joint-motion math module.

The numbers in these tests are drawn directly from:

* ``docs/ethercat/a6ec-frame-semantics-and-native-home.md``
* ``docs/ethercat/a6ec-manufacturer-notes-2026-04-15.md``
* ``.cursor/memory/AGENT_SCRATCHPAD.md`` 2026-04-17 J6 root-cause entry

so that each test doubles as a regression of the very bug the
canonical math is supposed to prevent (J6 360 deg long-way excursion,
stale anchor mod-RM drift, seam-straddling upload, etc.).
"""
from __future__ import annotations

import math

import pytest

from gradient_os.arm_controller.math import a6ec_joint_motion as jm


# ---------------------------------------------------------------------------
# Kinematics construction.
# ---------------------------------------------------------------------------


def _j6_kinematics() -> jm.A6ECAxisKinematics:
    """Representative multi-turn J6-style axis with ``sign=+1``.

    Same ``10:1`` output-shaft reduction as the real J6
    (``rm_counts = 10 * 2^17 = 1,310,720``) but with a ``+1`` sign
    convention so direction-positive tests stay intuitive. Tests that
    need to exactly reproduce the 2026-04-17 J6 incident numbers
    (where base_counts came out ``+3,623`` from a negative
    canonical_q) use :func:`_j6_incident_kinematics` instead.
    """
    return jm.A6ECAxisKinematics(
        encoder_counts_per_rev=jm.A6EC_COUNTS_PER_MOTOR_REV,
        gear_ratio_num=10,
        gear_ratio_den=1,
        sign=1,
        master_offset_rad=0.0,
    )


def _j6_incident_kinematics() -> jm.A6ECAxisKinematics:
    """Exact J6 kinematics from the 2026-04-17 incident.

    The scratchpad recorded:

        canonical_q = -0.01737 rad -> base_counts = +3,623

    For that pairing to hold, the axis sign must be ``-1`` (positive
    radian = negative counts). We preserve that convention here so the
    incident reproduction tests exercise the exact wire-frame path
    the incident ran through.
    """
    return jm.A6ECAxisKinematics(
        encoder_counts_per_rev=jm.A6EC_COUNTS_PER_MOTOR_REV,
        gear_ratio_num=10,
        gear_ratio_den=1,
        sign=-1,
        master_offset_rad=0.0,
    )


def _j1_kinematics_with_master_offset() -> jm.A6ECAxisKinematics:
    return jm.A6ECAxisKinematics(
        encoder_counts_per_rev=jm.A6EC_COUNTS_PER_MOTOR_REV,
        gear_ratio_num=50,
        gear_ratio_den=1,
        sign=-1,
        master_offset_rad=0.25,
    )


class TestAxisKinematics:
    def test_rm_counts_matches_vendor_formula(self) -> None:
        k = _j6_kinematics()
        assert k.rm_counts == 131_072 * 10
        assert k.rm_counts == 1_310_720

    def test_counts_per_unit_equals_rm_over_two_pi(self) -> None:
        k = _j6_kinematics()
        assert k.counts_per_unit == pytest.approx(k.rm_counts / (2.0 * math.pi))

    def test_gear_ratio_is_exact_fraction(self) -> None:
        k = _j6_kinematics()
        assert k.gear_ratio.numerator == 10
        assert k.gear_ratio.denominator == 1

    def test_rejects_invalid_sign(self) -> None:
        with pytest.raises(ValueError):
            jm.A6ECAxisKinematics(
                encoder_counts_per_rev=jm.A6EC_COUNTS_PER_MOTOR_REV,
                gear_ratio_num=1,
                gear_ratio_den=1,
                sign=0,
            )

    def test_rejects_zero_encoder_resolution(self) -> None:
        with pytest.raises(ValueError):
            jm.A6ECAxisKinematics(
                encoder_counts_per_rev=0,
                gear_ratio_num=1,
                gear_ratio_den=1,
            )

    def test_rejects_non_positive_gear_ratio(self) -> None:
        with pytest.raises(ValueError):
            jm.A6ECAxisKinematics(
                encoder_counts_per_rev=jm.A6EC_COUNTS_PER_MOTOR_REV,
                gear_ratio_num=0,
                gear_ratio_den=1,
            )


# ---------------------------------------------------------------------------
# Raw encoder reconstruction (Chapter 5).
# ---------------------------------------------------------------------------


class TestRawReconstruction:
    def test_sign_extend_16_positive(self) -> None:
        assert jm.sign_extend_16(0) == 0
        assert jm.sign_extend_16(1) == 1
        assert jm.sign_extend_16(0x7FFF) == 32_767

    def test_sign_extend_16_negative(self) -> None:
        assert jm.sign_extend_16(0x8000) == -32_768
        assert jm.sign_extend_16(0xFFFF) == -1

    def test_chapter5_formula_at_motor_origin(self) -> None:
        reconstructed = jm.reconstruct_multiturn_counts_from_u40_1c_1e(
            u40_1c=0,
            u40_1e=0,
        )
        assert reconstructed == 0

    def test_chapter5_formula_with_positive_revs(self) -> None:
        reconstructed = jm.reconstruct_multiturn_counts_from_u40_1c_1e(
            u40_1c=131_072 + 5_000,
            u40_1e=7,
        )
        assert reconstructed == 7 * 131_072 + 5_000

    def test_chapter5_formula_with_negative_revs(self) -> None:
        reconstructed = jm.reconstruct_multiturn_counts_from_u40_1c_1e(
            u40_1c=5_000,
            u40_1e=0xFFFF,
        )
        assert reconstructed == -1 * 131_072 + 5_000

    def test_chapter5_u40_1c_is_reduced_modulo_counts_per_rev(self) -> None:
        reconstructed = jm.reconstruct_multiturn_counts_from_u40_1c_1e(
            u40_1c=131_072 * 4 + 1_000,
            u40_1e=0,
        )
        assert reconstructed == 1_000


class TestCombineSignedI64Pair:
    def test_zero_pair(self) -> None:
        assert jm.combine_signed_i64_pair(low=0, high=0) == 0

    def test_positive_pair(self) -> None:
        combined = jm.combine_signed_i64_pair(low=0x12345678, high=0x9ABC)
        assert combined == (0x9ABC << 32) | 0x12345678

    def test_negative_pair_with_high_bit_set(self) -> None:
        combined = jm.combine_signed_i64_pair(low=0, high=-1)
        assert combined == -(1 << 32)

    def test_signed_boundary(self) -> None:
        combined = jm.combine_signed_i64_pair(low=0, high=0x80000000)
        assert combined == -(1 << 63)

    def test_known_u40_20_22_reading(self) -> None:
        combined = jm.combine_signed_i64_pair(
            low=1_000,
            high=0,
        )
        assert combined == 1_000
        combined = jm.combine_signed_i64_pair(
            low=-1_000 & 0xFFFFFFFF,
            high=-1 & 0xFFFFFFFF,
        )
        assert combined == -1_000


# ---------------------------------------------------------------------------
# Counts <-> joint radians.
# ---------------------------------------------------------------------------


class TestCountsAxisQConversion:
    def test_zero_counts_maps_to_zero_radians(self) -> None:
        k = _j6_kinematics()
        assert jm.axis_q_rad_from_counts(counts=0, kinematics=k) == 0.0

    def test_one_shaft_turn_maps_to_two_pi(self) -> None:
        k = _j6_kinematics()
        axis_q = jm.axis_q_rad_from_counts(counts=k.rm_counts, kinematics=k)
        assert axis_q == pytest.approx(2.0 * math.pi)

    def test_negative_sign_flips_direction(self) -> None:
        k = _j1_kinematics_with_master_offset()
        axis_q = jm.axis_q_rad_from_counts(counts=k.rm_counts, kinematics=k)
        assert axis_q == pytest.approx(-2.0 * math.pi)

    def test_roundtrip_axis_q_to_counts_and_back(self) -> None:
        k = _j6_kinematics()
        for axis_q in (-3.14, -1.0, 0.0, 0.5, 1.7, 5.0):
            counts = jm.counts_from_axis_q_rad(axis_q_rad=axis_q, kinematics=k)
            restored = jm.axis_q_rad_from_counts(counts=counts, kinematics=k)
            assert restored == pytest.approx(axis_q, abs=1e-9)


# ---------------------------------------------------------------------------
# Home anchor + canonical truth.
# ---------------------------------------------------------------------------


class TestHomeAnchorAndCanonicalTruth:
    def test_home_anchor_at_physical_zero_is_zero(self) -> None:
        # After HM35 with absolute_axis_q == reference_axis_q, the
        # anchor is zero.
        anchor = jm.compute_home_anchor_rad(
            absolute_axis_q_rad=1.234,
            reference_axis_q_rad=1.234,
        )
        assert anchor == pytest.approx(0.0, abs=1e-12)

    def test_home_anchor_captures_shaft_turn_offset(self) -> None:
        # absolute_axis_q carries multi-turn continuity; reference
        # wraps at 2*pi (for a 1:1 joint). The anchor records the
        # whole-shaft-turn offset between the two.
        anchor = jm.compute_home_anchor_rad(
            absolute_axis_q_rad=2.0 * math.pi + 0.3,
            reference_axis_q_rad=0.3,
        )
        assert anchor == pytest.approx(2.0 * math.pi, abs=1e-12)

    def test_canonical_q_subtracts_anchor_and_master_offset(self) -> None:
        k = _j1_kinematics_with_master_offset()
        canonical_q = jm.canonical_joint_q_rad(
            absolute_axis_q_rad=3.0,
            home_anchor_rad=1.0,
            kinematics=k,
        )
        assert canonical_q == pytest.approx(3.0 - 1.0 - k.master_offset_rad)

    def test_canonical_q_is_zero_at_anchor_when_no_master_offset(self) -> None:
        k = _j6_kinematics()
        canonical_q = jm.canonical_joint_q_rad(
            absolute_axis_q_rad=7.5,
            home_anchor_rad=7.5,
            kinematics=k,
        )
        assert canonical_q == pytest.approx(0.0, abs=1e-12)


# ---------------------------------------------------------------------------
# Shaft-frame consistency gate (mod-RM).
# ---------------------------------------------------------------------------


class TestShaftFrameConsistency:
    def test_consistent_when_anchor_matches_live_6064_exactly(self) -> None:
        k = _j6_kinematics()
        canonical_q = 0.1
        live_counts = int(
            jm.counts_from_axis_q_rad(axis_q_rad=canonical_q, kinematics=k)
        )
        result = jm.shaft_frame_consistency(
            canonical_q_rad=canonical_q,
            live_reference_counts=live_counts,
            kinematics=k,
        )
        assert result.consistent is True
        # Live 6064 is integer counts but expected_counts is a float;
        # a sub-count residual up to ~1 is legitimate quantization.
        assert abs(result.mod_rm_delta_counts) < 1.5
        assert result.wrap_turns == 0
        assert result.period_counts == k.rm_counts

    def test_consistent_across_whole_shaft_turn_offset(self) -> None:
        # 6064 wraps at RM; the gate MUST fold delta to nearest turn
        # so a legitimate multi-turn anchor offset does not read as a
        # frame bug.
        k = _j6_kinematics()
        canonical_q = 5.0 * (2.0 * math.pi) + 0.2
        live_counts = int(
            jm.counts_from_axis_q_rad(axis_q_rad=0.2, kinematics=k)
        )
        result = jm.shaft_frame_consistency(
            canonical_q_rad=canonical_q,
            live_reference_counts=live_counts,
            kinematics=k,
        )
        assert result.consistent is True
        assert result.wrap_turns == 5
        assert abs(result.mod_rm_delta_counts) < 1.5

    def test_inconsistent_when_sub_shaft_turn_drift_exceeds_tolerance(self) -> None:
        # Inject 50 counts of sub-shaft-turn drift. Explicit
        # tolerance=16 is passed below (historical pin), so 50 > 16
        # forces the gate closed regardless of the production
        # `_SHAFT_FRAME_CONSISTENCY_TOLERANCE_COUNTS` default (currently
        # 4096). The test uses a tight explicit tolerance to exercise
        # the closed-gate path with a small drift.
        k = _j6_kinematics()
        canonical_q = 0.2
        live_counts = int(
            jm.counts_from_axis_q_rad(axis_q_rad=canonical_q, kinematics=k)
            + 50
        )
        result = jm.shaft_frame_consistency(
            canonical_q_rad=canonical_q,
            live_reference_counts=live_counts,
            kinematics=k,
            tolerance_counts=16.0,
        )
        assert result.consistent is False
        assert abs(result.mod_rm_delta_counts) > 16.0

    def test_jitter_within_tolerance_stays_consistent(self) -> None:
        # A6-EC probe work: stationary bridges wander by 0..3 counts
        # (occasionally 7-9 post-restart). A 10-count drift must NOT
        # fail closed under a 16-count explicit tolerance; the
        # production default is larger (4096 counts, sized for the
        # full-revolution failure class) so this is also safe there.
        k = _j6_kinematics()
        canonical_q = 0.0
        live_counts = int(
            jm.counts_from_axis_q_rad(axis_q_rad=canonical_q, kinematics=k)
            + 10
        )
        result = jm.shaft_frame_consistency(
            canonical_q_rad=canonical_q,
            live_reference_counts=live_counts,
            kinematics=k,
            tolerance_counts=16.0,
        )
        assert result.consistent is True


# ---------------------------------------------------------------------------
# Shortest-angular fold.
# ---------------------------------------------------------------------------


class TestShortestAngularCounts:
    def test_zero_delta(self) -> None:
        assert jm.shortest_angular_counts(
            linear_delta_counts=0.0,
            rm_counts=1_310_720,
        ) == pytest.approx(0.0)

    def test_small_positive_delta_unchanged(self) -> None:
        assert jm.shortest_angular_counts(
            linear_delta_counts=3_623.0,
            rm_counts=1_310_720,
        ) == pytest.approx(3_623.0)

    def test_exact_half_period_folds_to_minus_half(self) -> None:
        rm = 1_310_720
        # (linear + RM/2) % RM - RM/2 with linear = RM/2 gives
        # (RM) % RM - RM/2 = -RM/2. This is the convention we want:
        # shortest-angular maps the half-period boundary to the
        # negative half to keep the fold open on the positive side.
        assert jm.shortest_angular_counts(
            linear_delta_counts=float(rm // 2),
            rm_counts=rm,
        ) == pytest.approx(-float(rm // 2))

    def test_seam_straddle_wraps_to_short_distance(self) -> None:
        rm = 1_310_720
        # A linear step of RM - 10 corresponds to the physically
        # short path of +10 counts in the other direction.
        assert jm.shortest_angular_counts(
            linear_delta_counts=float(rm - 10),
            rm_counts=rm,
        ) == pytest.approx(-10.0)


# ---------------------------------------------------------------------------
# Command fold: the 2026-04-17 J6 incident as a regression test.
# ---------------------------------------------------------------------------


class TestCommandFold:
    def test_small_jog_inside_seam_lands_in_single_turn(self) -> None:
        k = _j6_kinematics()
        canonical_q = 0.1
        live_counts = int(k.rm_counts // 3)  # somewhere in [0, RM)
        result = jm.fold_canonical_q_to_command_counts(
            canonical_q_rad=canonical_q,
            live_reference_counts=live_counts,
            kinematics=k,
        )
        assert 0 <= result.adjusted_counts < k.rm_counts

    def test_2026_04_17_j6_long_way_incident_is_wrapped(self) -> None:
        # Exact numbers from the scratchpad 2026-04-17 entry:
        #
        #   live_6064 before move 1 = 1,310,694  (= -26 signed)
        #   canonical_q target      = -0.01737 rad  (= -1 deg joint)
        #   base_counts             = +3,623     (sign=-1 on J6)
        #
        # The old fold produced adjusted_counts = 1,314,343 (= RM +
        # 3,623), which lived outside [0, RM). The A6-EC then took
        # the long way by one shaft turn. The new wrapped fold must
        # return a value inside [0, RM) that matches the intended
        # physical target.
        k = _j6_incident_kinematics()
        canonical_q = -0.01737
        live_counts = 1_310_694
        result = jm.fold_canonical_q_to_command_counts(
            canonical_q_rad=canonical_q,
            live_reference_counts=live_counts,
            kinematics=k,
            wrap_to_single_turn=True,
        )
        assert 0 <= result.adjusted_counts < k.rm_counts
        # Intended physical target is +3,623 counts (= +1 deg joint
        # in the reference frame), within a ~1 count rounding margin.
        assert abs(result.adjusted_counts - 3_623) <= 2

    def test_legacy_linear_windowed_fold_reproduces_incident_signature(self) -> None:
        # With wrap_to_single_turn=False (the diagnostic roundtrip
        # path), the fold is allowed to emit out-of-range values.
        # This test pins the original linear-windowed behavior so the
        # roundtrip diagnostic keeps working unchanged.
        k = _j6_incident_kinematics()
        canonical_q = -0.01737
        live_counts = 1_310_694
        result = jm.fold_canonical_q_to_command_counts(
            canonical_q_rad=canonical_q,
            live_reference_counts=live_counts,
            kinematics=k,
            wrap_to_single_turn=False,
        )
        # Under the legacy fold the output is RM + base_counts, i.e.
        # one shaft turn above zero, which is exactly the J6 incident
        # value from the scratchpad (1,314,343 ±1 count).
        assert result.adjusted_counts > k.rm_counts
        assert result.wrap_lift_counts == k.rm_counts
        assert abs(result.adjusted_counts - 1_314_343) <= 2

    def test_fold_is_stateless_across_consecutive_calls(self) -> None:
        k = _j6_kinematics()
        live_a = 5_000
        live_b = k.rm_counts - 5_000
        canonical_q = 0.0
        result_a = jm.fold_canonical_q_to_command_counts(
            canonical_q_rad=canonical_q,
            live_reference_counts=live_a,
            kinematics=k,
        )
        result_b = jm.fold_canonical_q_to_command_counts(
            canonical_q_rad=canonical_q,
            live_reference_counts=live_b,
            kinematics=k,
        )
        # canonical_q == 0 -> target should be 0 after wrap, and the
        # SHORTEST-ANGULAR delta to both sides of the seam is about
        # 5,000 counts each.
        assert result_a.adjusted_counts == 0
        assert result_b.adjusted_counts == 0
        assert abs(result_a.angular_delta_counts) < 6_000
        assert abs(result_b.angular_delta_counts) < 6_000

    def test_fold_handles_negative_canonical_q(self) -> None:
        k = _j6_kinematics()
        canonical_q = -5.0
        live_counts = int(k.rm_counts // 2)
        result = jm.fold_canonical_q_to_command_counts(
            canonical_q_rad=canonical_q,
            live_reference_counts=live_counts,
            kinematics=k,
        )
        assert 0 <= result.adjusted_counts < k.rm_counts


class TestInverseCanonicalQ:
    def test_roundtrip_canonical_q_through_command_fold(self) -> None:
        k = _j6_kinematics()
        for canonical_q in (-0.5, -0.01, 0.0, 0.1, 0.5):
            live_counts = int(k.rm_counts // 4)
            fold = jm.fold_canonical_q_to_command_counts(
                canonical_q_rad=canonical_q,
                live_reference_counts=live_counts,
                kinematics=k,
            )
            restored = jm.inverse_canonical_q_from_command_counts(
                command_counts=fold.adjusted_counts,
                live_reference_counts=live_counts,
                kinematics=k,
            )
            # One-count quantization is expected because adjusted_counts
            # is rounded at the wire boundary.
            tolerance_rad = 2.0 / k.counts_per_unit
            assert restored == pytest.approx(canonical_q, abs=tolerance_rad)


# ---------------------------------------------------------------------------
# Trajectory wire-frame safety gates.
# ---------------------------------------------------------------------------


class TestPerPointStep:
    def test_small_step_passes(self) -> None:
        k = _j6_kinematics()
        cpu = k.counts_per_unit
        step_counts = int(round(0.01 * cpu))  # ~0.6 deg joint
        violation = jm.check_per_point_step(
            axis_i=5,
            logical_joint_idx=5,
            current_counts=10_000 + step_counts,
            previous_counts=10_000,
            kinematics=k,
        )
        assert violation is None

    def test_oversized_angular_step_is_rejected(self) -> None:
        k = _j6_kinematics()
        cpu = k.counts_per_unit
        # 100 deg step, well above the 20 deg default bound.
        step_counts = int(round(math.radians(100.0) * cpu))
        violation = jm.check_per_point_step(
            axis_i=5,
            logical_joint_idx=5,
            current_counts=100_000 + step_counts,
            previous_counts=100_000,
            kinematics=k,
        )
        assert violation is not None
        assert violation.code == "command_frame_oversized_step"

    def test_seam_crossing_step_flagged_when_unsafe(self) -> None:
        # Linear step ~RM with tiny shortest-angular step. Under
        # seam_crossing_unsafe=True (A6-EC rotation-mode default) we
        # fail closed even though the angular delta is small.
        k = _j6_kinematics()
        violation = jm.check_per_point_step(
            axis_i=5,
            logical_joint_idx=5,
            current_counts=10,
            previous_counts=k.rm_counts - 10,
            kinematics=k,
            seam_crossing_unsafe=True,
        )
        assert violation is not None
        assert violation.code == "command_frame_seam_crossing_step_disallowed"

    def test_seam_crossing_step_allowed_when_safe(self) -> None:
        # Same scenario; seam_crossing_unsafe=False (e.g. a future
        # drive family that honors C10.16 cleanly) allows it through.
        k = _j6_kinematics()
        violation = jm.check_per_point_step(
            axis_i=5,
            logical_joint_idx=5,
            current_counts=10,
            previous_counts=k.rm_counts - 10,
            kinematics=k,
            seam_crossing_unsafe=False,
        )
        assert violation is None


class TestFirstPointLiveDeviation:
    def test_first_point_near_live_passes(self) -> None:
        k = _j6_kinematics()
        cpu = k.counts_per_unit
        target = 400_000 + int(round(0.02 * cpu))
        violation = jm.check_first_point_live_deviation(
            axis_i=5,
            logical_joint_idx=5,
            current_counts=target,
            live_reference_counts=400_000,
            kinematics=k,
        )
        assert violation is None

    def test_first_point_far_from_live_is_rejected(self) -> None:
        k = _j6_kinematics()
        cpu = k.counts_per_unit
        target = 400_000 + int(round(math.radians(45.0) * cpu))
        violation = jm.check_first_point_live_deviation(
            axis_i=5,
            logical_joint_idx=5,
            current_counts=target,
            live_reference_counts=400_000,
            kinematics=k,
        )
        assert violation is not None
        assert violation.code == "command_frame_live_deviation_out_of_range"

    def test_first_point_seam_straddle_blocked_when_unsafe(self) -> None:
        k = _j6_kinematics()
        violation = jm.check_first_point_live_deviation(
            axis_i=5,
            logical_joint_idx=5,
            current_counts=10,
            live_reference_counts=k.rm_counts - 10,
            kinematics=k,
            seam_crossing_unsafe=True,
        )
        assert violation is not None
        assert violation.code == "command_frame_seam_crossing_first_point_disallowed"


class TestEnforceTrajectoryWireFrameSafety:
    def test_empty_trajectory_is_noop(self) -> None:
        jm.enforce_trajectory_wire_frame_safety(
            axis_i=0,
            logical_joint_idx=0,
            point_counts=[],
            live_reference_counts=0,
            kinematics=_j6_kinematics(),
        )

    def test_safe_trajectory_passes(self) -> None:
        k = _j6_kinematics()
        cpu = k.counts_per_unit
        start = 300_000
        points = [
            start + int(round(0.001 * cpu * i))
            for i in range(1, 11)
        ]
        jm.enforce_trajectory_wire_frame_safety(
            axis_i=5,
            logical_joint_idx=5,
            point_counts=points,
            live_reference_counts=start,
            kinematics=k,
        )

    def test_oversized_step_raises_runtime_error(self) -> None:
        k = _j6_kinematics()
        cpu = k.counts_per_unit
        live = 300_000
        first = live + int(round(0.02 * cpu))
        second = first + int(round(math.radians(60.0) * cpu))
        with pytest.raises(RuntimeError) as excinfo:
            jm.enforce_trajectory_wire_frame_safety(
                axis_i=5,
                logical_joint_idx=5,
                point_counts=[first, second],
                live_reference_counts=live,
                kinematics=k,
            )
        assert "command_frame_oversized_step" in str(excinfo.value)

    def test_first_point_far_from_live_raises_runtime_error(self) -> None:
        k = _j6_kinematics()
        cpu = k.counts_per_unit
        live = 300_000
        first = live + int(round(math.radians(45.0) * cpu))
        with pytest.raises(RuntimeError) as excinfo:
            jm.enforce_trajectory_wire_frame_safety(
                axis_i=5,
                logical_joint_idx=5,
                point_counts=[first],
                live_reference_counts=live,
                kinematics=k,
            )
        assert "command_frame_live_deviation_out_of_range" in str(excinfo.value)

    def test_violation_message_includes_joint_number_and_period(self) -> None:
        k = _j6_kinematics()
        cpu = k.counts_per_unit
        live = 300_000
        first = live + int(round(math.radians(45.0) * cpu))
        with pytest.raises(RuntimeError) as excinfo:
            jm.enforce_trajectory_wire_frame_safety(
                axis_i=5,
                logical_joint_idx=5,
                point_counts=[first],
                live_reference_counts=live,
                kinematics=k,
            )
        message = str(excinfo.value)
        # Joint index is 1-based in the operator-facing message so
        # the failure mirrors the existing backend log format.
        assert "joint=6" in message
        assert f"period_counts={k.rm_counts}" in message


# ---------------------------------------------------------------------------
# HM35 607C bias helper.
# ---------------------------------------------------------------------------


class TestHm35OriginOffsetBias:
    def test_midpoint_bias_is_rm_over_two(self) -> None:
        k = _j6_kinematics()
        bias = jm.hm35_origin_offset_biased_to_midpoint(kinematics=k)
        assert bias == k.rm_counts // 2

    def test_quarter_turn_bias(self) -> None:
        k = _j6_kinematics()
        bias = jm.hm35_origin_offset_biased_to_midpoint(
            kinematics=k,
            fraction_numerator=1,
            fraction_denominator=4,
        )
        assert bias == k.rm_counts // 4

    def test_rejects_fraction_at_or_above_one(self) -> None:
        k = _j6_kinematics()
        with pytest.raises(ValueError):
            jm.hm35_origin_offset_biased_to_midpoint(
                kinematics=k,
                fraction_numerator=2,
                fraction_denominator=2,
            )

    def test_bias_is_always_in_single_turn_range(self) -> None:
        # Vendor email 2 Q6: 607C must live in [0, RM-1] in rotation
        # mode. The helper must never emit a value outside that range.
        k = _j6_kinematics()
        for num, den in [(0, 1), (1, 3), (1, 2), (2, 3), (9, 10)]:
            bias = jm.hm35_origin_offset_biased_to_midpoint(
                kinematics=k,
                fraction_numerator=num,
                fraction_denominator=den,
            )
            assert 0 <= bias < k.rm_counts
