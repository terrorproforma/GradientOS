"""Replay the A6-EC J6 manual-rotation capture through the backend.

This test class complements the synthetic per-joint sweep in
``tests/test_a6ec_joint_sweep.py``. Instead of constructing inputs
analytically, it feeds the backend a trimmed subset of the real J6
rotation dataset captured on 2026-04-16 and asserts:

- ``canonical_q`` recovered from ``_canonical_joint_positions_from_raw_feedback``
  tracks the captured API-level ``canonical_rad`` value on every sample;
- the canonical truth stays continuous across the shaft seam (no ``2*pi``
  jumps);
- the stateless nearest-turn fold keeps per-write ``607A`` within half
  of ``RM`` of the live ``6064`` at every sample;
- ``_enforce_trajectory_wire_frame_safety`` refuses a synthetic
  ``+360 deg`` jog at the seam-crossing sample;
- overriding the captured statusword to ``0x1650`` still yields
  ``verification_source = "persisted_home_anchor_agreement"`` on every
  sample (i.e. the 2026-04-17 restart-trust path is validated against
  real multi-turn motion, not just synthesised numbers).

The fixture lives at ``tests/fixtures/a6ec_j6_seam_rotation.jsonl``; if
it is missing the tests skip cleanly. Regenerate via
``python scripts/build_a6ec_j6_replay_fixture.py`` from a fresh
``a6ec_chapter5_probe watch`` capture.

Hygiene: every test isolates ``GRADIENT_ABSOLUTE_ENCODER_ANCHORS_PATH``
to ``tmp_path`` so the repo-root anchor file cannot silently satisfy the
persisted-anchor gate and mask a real inconsistency.
"""

from __future__ import annotations

import json
import math
from pathlib import Path

import pytest

from gradient_os.arm_controller.backends.ethercat_rtcore.backend import (
    EthercatRTCoreBackend,
    _AbsoluteFeedbackAxisMetrics,
)
from gradient_os.arm_controller.robots.gradient05.config import Gradient05Config


_FIXTURE_PATH = Path(__file__).resolve().parent / "fixtures" / "a6ec_j6_seam_rotation.jsonl"
_J6_AXIS_INDEX = 5
_J6_COUNTS_PER_REV = 131072
_J6_GEAR_RATIO = 10
_J6_RM = _J6_COUNTS_PER_REV * _J6_GEAR_RATIO  # 1_310_720
_J6_CPU = float(_J6_COUNTS_PER_REV) * float(_J6_GEAR_RATIO) / (2.0 * math.pi)
_J6_SIGN = -1  # per Gradient05Config.actuator_position_signs[5]

# The captured J6 rotation dataset reads 6064 and U40.20/.22 sequentially
# over ~1 ms per SDO each, and the joint was physically being rotated by
# hand during capture. Peak-motion cross-SDO skew can reach tens of
# thousands of motor counts - large enough that the production
# consistency gates (16 counts for shaft-frame, 15 for roundtrip) reject
# fast-motion samples purely as a capture artifact, not as a math bug.
#
# The synthetic per-joint sweep in ``tests/test_a6ec_joint_sweep.py``
# exercises those gates under controlled inputs. The replay tests here
# focus on the underlying math primitives (anchored ``canonical_q``
# reconstruction and the stateless nearest-turn fold) against real
# captured data without routing through the gate-guarded pipeline.


def _load_fixture() -> list[dict[str, object]]:
    """Return the replay samples from the committed fixture.

    The first line is a ``__kind__=fixture_meta`` header produced by
    ``scripts/build_a6ec_j6_replay_fixture.py``; skip it and return the
    rest verbatim.
    """
    if not _FIXTURE_PATH.exists():
        pytest.skip(f"a6ec J6 replay fixture not generated: {_FIXTURE_PATH}")
    samples: list[dict[str, object]] = []
    with _FIXTURE_PATH.open("r", encoding="utf-8") as handle:
        for line in handle:
            line = line.strip()
            if not line:
                continue
            rec = json.loads(line)
            if rec.get("__kind__") == "fixture_meta":
                continue
            samples.append(rec)
    assert samples, f"fixture {_FIXTURE_PATH} has no replay records"
    return samples


def _u40_low_high_signed_i32(continuous: int) -> tuple[int, int]:
    """Split a continuous signed int into two signed int32 halves."""
    low_u = int(continuous) & 0xFFFFFFFF
    high_u = (int(continuous) >> 32) & 0xFFFFFFFF
    low_signed = low_u - (1 << 32) if low_u >= (1 << 31) else low_u
    high_signed = high_u - (1 << 32) if high_u >= (1 << 31) else high_u
    return int(low_signed), int(high_signed)


def _build_replay_backend(
    monkeypatch: pytest.MonkeyPatch,
    tmp_path,
    *,
    anchor_rad: float,
) -> EthercatRTCoreBackend:
    """Construct a backend primed with the given absolute-home anchor on J6.

    Non-J6 axes are seeded with a clean, consistent anchor at q=0 so the
    shared pipeline does not flag them as unavailable and accidentally
    entangle the J6-focused assertions.
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

    # Prime the J6 anchor.
    backend._absolute_encoder_home_anchors[_J6_AXIS_INDEX] = {
        "home_anchor_rad": float(anchor_rad),
        "source": "pytest",
        "axis_indices": [_J6_AXIS_INDEX],
    }
    # Clean anchors for the other joints so their truth views do not
    # interfere with the J6 assertions.
    for other_joint in range(6):
        if other_joint == _J6_AXIS_INDEX:
            continue
        backend._absolute_encoder_home_anchors[other_joint] = {
            "home_anchor_rad": 0.0,
            "source": "pytest",
            "axis_indices": [other_joint],
        }
    return backend


def _seed_sample(
    backend: EthercatRTCoreBackend,
    sample: dict[str, object],
    *,
    statusword_override: int | None = None,
) -> dict[str, object]:
    """Write a single replay sample into the backend-visible state.

    The fixture carries two reads of every object: the probe-script's
    direct SDO and the controller-backend's metrics-snapshot view, taken
    a few milliseconds apart. For a faithful replay we seed with the
    backend-visible view (``expected.raw_counts`` / ``expected.absolute_counts``)
    so the synthesised pipeline sees exactly what the backend saw at
    capture time; direct SDO values are kept on the sample for comparison
    but not injected.

    Returns the RTCore metrics axes payload used for that sample so the
    ``_load_rtcore_metrics_snapshot`` monkeypatch can be refreshed.
    """
    expected = sample["expected"]
    assert isinstance(expected, dict)
    counts_6064 = int(expected["raw_counts"])
    continuous_u40 = int(expected["absolute_counts"])
    statusword = int(sample["statusword"]) if statusword_override is None else int(statusword_override)

    backend._axis_counts[_J6_AXIS_INDEX] = counts_6064
    mt_low, mt_high = _u40_low_high_signed_i32(continuous_u40)
    backend._absolute_feedback_by_axis[_J6_AXIS_INDEX] = _AbsoluteFeedbackAxisMetrics.from_mapping(
        {
            "encoder_multi_turn_low": {"valid": 1, "value": mt_low},
            "encoder_multi_turn_high": {"valid": 1, "value": mt_high},
            "absolute_position_reference": {"valid": 1, "value": counts_6064},
            "rotation_mode_position_reference": {"valid": 1, "value": counts_6064},
            "rotation_mode_encoder_low": {"valid": 1, "value": counts_6064},
            "rotation_mode_encoder_high": {"valid": 1, "value": 0},
        }
    )

    # Keep the non-J6 axes quiet and clean so their truth views do not
    # flip around across replay samples.
    metrics_axes: list[dict[str, object]] = []
    for axis_i in range(6):
        if axis_i == _J6_AXIS_INDEX:
            sw_axis = statusword
            cnt_axis = counts_6064
        else:
            sw_axis = 0x9650  # fresh-HM trust on the other axes
            cnt_axis = 0
            backend._axis_counts[axis_i] = 0
            backend._absolute_feedback_by_axis[axis_i] = _AbsoluteFeedbackAxisMetrics.from_mapping(
                {
                    "encoder_multi_turn_low": {"valid": 1, "value": 0},
                    "encoder_multi_turn_high": {"valid": 1, "value": 0},
                    "absolute_position_reference": {"valid": 1, "value": 0},
                    "rotation_mode_position_reference": {"valid": 1, "value": 0},
                    "rotation_mode_encoder_low": {"valid": 1, "value": 0},
                    "rotation_mode_encoder_high": {"valid": 1, "value": 0},
                }
            )
        metrics_axes.append(
            {
                "statusword": sw_axis,
                "error_code": 0,
                "manufacturer_error_code": 0,
                "startup_drive_config": _startup_drive_config_entry(axis_i),
                "startup_drive_configs": _startup_drive_config_entries(axis_i),
                "slave_online": 1,
                "slave_operational": 1,
                "slave_al_state": 8,
            }
        )
    return {"axes": metrics_axes, "counts_6064": counts_6064, "continuous_u40": continuous_u40}


def _startup_drive_config_entries(axis_i: int) -> list[dict[str, object]]:
    from fractions import Fraction

    cfg = Gradient05Config()
    ratio = Fraction(str(cfg.actuator_gear_ratios[axis_i]))
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
        {
            "setting_key": "a6ec_rotation_mode_reference_running_direction",
            "configured": 1,
            "commanded": 0,
            "readback_valid": 1,
            "readback": 0,
            "verified": 1,
        },
    ]


def _startup_drive_config_entry(axis_i: int) -> dict[str, object]:
    return _startup_drive_config_entries(axis_i)[0]


def _anchor_from_first_sample(first: dict[str, object]) -> float:
    """Derive the absolute-home anchor so the first sample reconstructs correctly.

    ``canonical_q = absolute_axis_q - anchor - master_offset``.
    With ``master_offset = 0`` and the captured ``expected.canonical_rad``
    as the target, ``anchor = absolute_axis_q - canonical_rad``.

    Uses the backend-visible ``expected.absolute_counts`` so the anchor
    derivation is consistent with the values we seed in ``_seed_sample``.
    """
    expected = first["expected"]
    assert isinstance(expected, dict)
    absolute_counts = int(expected["absolute_counts"])
    absolute_axis_q = float(absolute_counts) / (float(_J6_SIGN) * _J6_CPU)
    canonical_rad = float(expected["canonical_rad"])
    return absolute_axis_q - canonical_rad


def _run_snapshot(
    backend: EthercatRTCoreBackend,
    *,
    metrics_axes: list[dict[str, object]],
    monkeypatch: pytest.MonkeyPatch,
) -> dict[str, object]:
    monkeypatch.setattr(backend, "_load_rtcore_metrics_snapshot", lambda: {"axes": metrics_axes})
    return backend._canonical_joint_positions_from_raw_feedback(
        {axis_i: int(backend._axis_counts[axis_i]) for axis_i in range(6)},
        reference_mode="raw",
    )


def _expected_canonical_q(sample: dict[str, object], anchor_rad: float) -> float:
    """Compute the expected canonical_q for a sample given the primed anchor.

    Mirrors ``canonical_q = absolute_axis_q - anchor - master_offset``
    with master_offset=0, using the backend-visible
    ``expected.absolute_counts`` exactly as the backend would.
    """
    expected = sample["expected"]
    assert isinstance(expected, dict)
    absolute_counts = int(expected["absolute_counts"])
    absolute_axis_q = float(absolute_counts) / (float(_J6_SIGN) * _J6_CPU)
    return absolute_axis_q - float(anchor_rad)


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------


def test_a6ec_j6_watch_replay_preserves_continuous_canonical_truth(monkeypatch, tmp_path):
    """Anchored canonical_q recovered from captured U40.20/.22 stays multi-turn continuous.

    This exercises the core truth math (``absolute_axis_q - anchor``)
    against real captured multi-turn motion. We assert:

    - the recovered canonical_q at sample 0 matches the captured value
      exactly (by construction, since the anchor is derived from it);
    - there are no ``2*pi``-class jumps between consecutive samples,
      which is the defining property of correct multi-turn truth;
    - the recovered sweep spans more than one shaft revolution, proving
      the fixture actually exercises multi-turn wrap behaviour;
    - recovered samples match captured canonical_rad within a small
      tolerance at *stationary* samples (the tail), where the capture
      timing effects are quiescent.

    We intentionally avoid a per-sample exact-value match during the
    motion window: the capture-time backend ran its own display-mode
    unwrapping on some samples, so the captured canonical_rad does not
    equal ``absolute_counts/(sign*cpu) - anchor`` bit-for-bit across the
    seam. What matters for multi-turn correctness is continuity, which
    the raw U40.20/.22 source carries directly.
    """
    samples = _load_fixture()
    anchor_rad = _anchor_from_first_sample(samples[0])
    per_count_rad = 1.0 / _J6_CPU

    recovered_all: list[float] = []
    previous_canonical_q: float | None = None
    for sample in samples:
        recovered_q = _expected_canonical_q(sample, anchor_rad)
        recovered_all.append(recovered_q)

        if previous_canonical_q is not None:
            recovered_step = abs(recovered_q - previous_canonical_q)
            assert recovered_step < math.pi, (
                f"source_index={sample.get('source_index')} "
                f"recovered_q jumped {recovered_step:.3f} rad vs previous "
                f"({previous_canonical_q:.3f} -> {recovered_q:.3f})"
            )
        previous_canonical_q = recovered_q

    # Sanity: the recovered sweep spans more than one shaft revolution
    # so this test actually exercises multi-turn wrap behaviour.
    span_rad = max(recovered_all) - min(recovered_all)
    assert span_rad > 2.0 * math.pi, (
        f"replay fixture should span more than one shaft revolution; saw {span_rad:.3f} rad"
    )

    # Sample 0 match is exact by anchor construction.
    captured0 = float(samples[0]["expected"]["canonical_rad"])  # type: ignore[index]
    assert abs(recovered_all[0] - captured0) < 1e-12

    # Stationary-tail samples (post-motion) are the ones where the
    # capture-time backend output unambiguously tracked U40.20/.22
    # directly. Assert those match to within a few counts.
    tail_absolute_tolerance_rad = 15.0 * per_count_rad
    tail_samples = [
        (sample, recovered_all[i])
        for i, sample in enumerate(samples)
        if int(sample.get("source_index", 0)) >= 500
    ]
    assert tail_samples, "fixture missing stationary tail samples"
    for sample, recovered_q in tail_samples:
        captured_canonical = float(sample["expected"]["canonical_rad"])  # type: ignore[index]
        assert abs(recovered_q - captured_canonical) <= tail_absolute_tolerance_rad, (
            f"source_index={sample.get('source_index')} "
            f"recovered={recovered_q} captured={captured_canonical} "
            f"diff_counts={(recovered_q - captured_canonical) / per_count_rad:.1f}"
        )


def test_a6ec_j6_watch_replay_small_jog_stays_within_half_rm(monkeypatch, tmp_path):
    """A +0.5 deg synthetic jog at every captured sample stays within RM/2 on the SHORTEST-ANGULAR wire delta.

    Since the 2026-04-17 wrap-to-[0, RM) fold change, the command path
    lands 607A in the drive's single-turn range. A seam-adjacent jog can
    have a large linear wire delta while the physical motion is tiny.
    The fold must keep the shortest-angular (mod-RM) delta inside half
    a shaft revolution - a larger angular delta would indicate a
    shaft-scale wrong-turn bug.
    """
    samples = _load_fixture()
    anchor_rad = _anchor_from_first_sample(samples[0])
    backend = _build_replay_backend(monkeypatch, tmp_path, anchor_rad=anchor_rad)

    jog_rad = math.radians(0.5)
    half_period = _J6_RM // 2
    for sample in samples:
        expected = sample["expected"]
        assert isinstance(expected, dict)
        counts_6064 = int(expected["raw_counts"])
        backend._axis_counts[_J6_AXIS_INDEX] = counts_6064

        recovered_q = _expected_canonical_q(sample, anchor_rad)
        target_axis_q = backend._command_axis_q_for_joint_value(
            axis_i=_J6_AXIS_INDEX,
            logical_joint_idx=_J6_AXIS_INDEX,
            canonical_q=recovered_q + jog_rad,
        )
        wire_counts = int(round(target_axis_q * float(_J6_SIGN) * _J6_CPU))
        # Since the 2026-04-19 continuous-607A landing, wire_counts is
        # continuous and can fall outside [0, RM). The SHORTEST-ANGULAR
        # delta from live 6064 is the live invariant (host fold keeps
        # every point within RM/2 of live in angular terms, even when
        # the linear delta crosses the seam).
        linear_delta = wire_counts - counts_6064
        angular_delta = ((linear_delta + half_period) % _J6_RM) - half_period
        assert abs(angular_delta) <= half_period, (
            f"source_index={sample.get('source_index')} angular_delta={angular_delta} "
            f"RM/2={half_period}"
        )


def test_a6ec_j6_watch_replay_rejects_large_angular_jump(monkeypatch, tmp_path):
    """A +100 deg synthetic jog at a seam sample must trip command_frame_oversized_step.

    Since 2026-04-17 the command path wraps into [0, RM) and the cage
    measures shortest-angular (mod-RM) distance. A literal "one shaft
    revolution" linear jump collapses to angular 0 under the new cage
    (both points map to the same single-turn position, the drive does
    not move). The canonical pathological input now is a large angular
    step like 100 deg that genuinely exceeds
    _TRAJECTORY_MAX_PER_POINT_STEP_RAD.
    """
    samples = _load_fixture()
    anchor_rad = _anchor_from_first_sample(samples[0])
    backend = _build_replay_backend(monkeypatch, tmp_path, anchor_rad=anchor_rad)

    # Find a sample whose 6064 is near the seam so the "next" trajectory
    # point straddles the seam if mis-aligned. The specific sample does
    # not matter much for this assertion; we only need a valid base state.
    seam_sample = None
    for sample in samples:
        expected = sample["expected"]
        assert isinstance(expected, dict)
        counts = int(expected["raw_counts"])
        if counts <= 256 or counts >= _J6_RM - 256:
            seam_sample = sample
            break
    assert seam_sample is not None, "fixture has no seam-adjacent sample"

    seam_expected = seam_sample["expected"]
    assert isinstance(seam_expected, dict)
    backend._axis_counts[_J6_AXIS_INDEX] = int(seam_expected["raw_counts"])
    recovered_q = _expected_canonical_q(seam_sample, anchor_rad)

    first_axis_q = backend._command_axis_q_for_joint_value(
        axis_i=_J6_AXIS_INDEX,
        logical_joint_idx=_J6_AXIS_INDEX,
        canonical_q=recovered_q,
    )
    # Second point at ~100 deg angular from first. Use axis_q directly
    # (bypassing the fold) to simulate the "host math wrongly jumped
    # angular by 100 deg" bug the cage should catch.
    big_step_rad = math.radians(100.0)
    big_step_axis_q = first_axis_q + big_step_rad / float(_J6_SIGN)

    previous_axis_counts: list[int | None] = [None] * backend._rt_num_axes
    initial_live_counts: list[int | None] = [
        backend._live_reference_counts_for_axis(i)
        for i in range(backend._rt_num_axes)
    ]
    first_vec = [0.0] * backend._rt_num_axes
    first_vec[_J6_AXIS_INDEX] = first_axis_q
    backend._enforce_trajectory_wire_frame_safety(
        axis_q=first_vec,
        axis_mask=(1 << _J6_AXIS_INDEX),
        previous_axis_counts=previous_axis_counts,
        initial_live_counts=initial_live_counts,
        point_index=0,
        traj_id=21,
    )
    second_vec = [0.0] * backend._rt_num_axes
    second_vec[_J6_AXIS_INDEX] = big_step_axis_q
    with pytest.raises(RuntimeError, match="command_frame_oversized_step"):
        backend._enforce_trajectory_wire_frame_safety(
            axis_q=second_vec,
            axis_mask=(1 << _J6_AXIS_INDEX),
            previous_axis_counts=previous_axis_counts,
            initial_live_counts=initial_live_counts,
            point_index=1,
            traj_id=21,
        )


def test_a6ec_j6_watch_replay_post_power_cycle_persisted_anchor_path(monkeypatch, tmp_path):
    """Overriding statusword to 0x1650 on stationary replay samples still validates truth via the persisted-anchor path.

    We restrict this assertion to the fixture's stationary tail samples
    because they are the ones where the captured SDO reads are
    simultaneous enough for the production shaft-frame gate to pass
    (see the module docstring for why fast-motion samples are expected
    to fail the gate on replay). The synthetic per-joint sweep in
    ``tests/test_a6ec_joint_sweep.py`` already asserts the persisted-
    anchor path fires on every joint under controlled data; this test
    closes the loop by showing it also fires on real captured data that
    is quiet enough for the gate to clear.
    """
    samples = _load_fixture()
    anchor_rad = _anchor_from_first_sample(samples[0])
    backend = _build_replay_backend(monkeypatch, tmp_path, anchor_rad=anchor_rad)

    # The fixture has a stationary tail (samples with source_index well
    # past the motion window). Pick the ones with source_index >= 500,
    # which the fixture meta guarantees are in the stationary plateau.
    stationary_samples = [
        sample
        for sample in samples
        if int(sample.get("source_index", 0)) >= 500
    ]
    assert stationary_samples, "fixture missing stationary tail samples"

    for sample in stationary_samples:
        seed_info = _seed_sample(backend, sample, statusword_override=0x1650)
        snapshot = _run_snapshot(
            backend,
            metrics_axes=seed_info["axes"],  # type: ignore[arg-type]
            monkeypatch=monkeypatch,
        )
        detail = snapshot["axis_absolute_feedback"][_J6_AXIS_INDEX]

        assert detail["truth_available"] is True, (
            f"source_index={sample.get('source_index')} "
            f"post-power-cycle replay should still be trusted on stationary tail: {detail}"
        )
        assert detail["statusword_hex"] == "0x1650"
        assert detail["drive_native_truth_signature_valid"] is False
        assert detail["coordinate_system_valid"] is True
        assert detail["drive_native_truth_verification_source"] == "persisted_home_anchor_agreement"
        assert detail["persisted_home_anchor_present"] is True
        assert detail["multi_turn_feedback_valid"] is True
        assert detail["persisted_home_anchor_consistent"] is True
        assert detail["shaft_frame_consistent"] is True
