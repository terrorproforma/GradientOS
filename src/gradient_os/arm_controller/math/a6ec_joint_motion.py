"""Pure-functional joint-motion math for the A6-EC servo drive.

This module is the canonical, stateless source for every equation
that moves a joint on an A6-EC drive in Absolute Rotation Mode
(``C00.07 = 4``) with the drive-native gear ratio programmed into
``C10.18 / C10.19``.

It is derived directly from:

* `docs/ethercat/a6ec-frame-semantics-and-native-home.md` -- the
  internal workstream note on frame separation and canonical truth.
* `docs/ethercat/a6ec-manufacturer-notes-2026-04-15.md` -- the vendor
  correspondence (Q1-Q11 plus the "Model B" email) that pins down
  ``RM`` semantics, ``6064`` sawtooth behavior, and HM35 trust
  conditions.
* Chapter 5 (absolute system) and Chapter 11 (parameter list) of the
  A6-EC series servo drive manual.

The production command and feedback paths in
``arm_controller.backends.ethercat_rtcore.backend`` thread live state
into these functions; the functions themselves are intentionally side
effect free so they can be unit-tested with vendor-derived numbers.

Frame vocabulary (used consistently throughout this module):

* **Raw encoder motor frame** -- ``U40.1C`` (single-turn), ``U40.1E``
  (motor-rev word), and the combined multi-turn pairs
  ``U40.20/.22`` / ``U40.2A/.2C``. Rotates one full turn per motor
  revolution; the pairs are signed 64-bit and continuous across up
  to ``32,767`` motor turns (vendor Q1).
* **Rotation-mode / shaft frame** -- ``U40.28`` (reference units),
  ``U40.2A/.2C`` (encoder units). Tracks the output shaft AFTER the
  gearbox when ``C10.18 / C10.19`` hold the real mechanical ratio.
* **Reference / CSP / home frame** -- ``6064`` (feedback),
  ``607A`` (target), ``U40.16`` (raw reference), ``60FC`` (position
  reference). In rotation mode these are a sawtooth in ``[0, RM-1]``
  (vendor email 3 Q1/Q5). ``RM = encoder_cpr * num / den``.

Canonical joint truth is expressed in **joint-space radians**, which
is the output-shaft angle after the gearbox with an optional signed
``master_offset`` so the planner zero lines up with the robot's
kinematic zero.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from fractions import Fraction
from typing import Iterable, Optional


A6EC_COUNTS_PER_MOTOR_REV: int = 1 << 17
"""A6-EC motor-side encoder resolution per manual Chapter 5 (``2^17``).

This is the per-motor-turn count used by the raw encoder reconstruction
formulas (``U40.1C``, ``U40.1E``) and by the multi-turn retention
ceiling (``A6EC_MAX_OFF_MOTOR_REVOLUTIONS * counts``).
"""

A6EC_MAX_OFF_MOTOR_REVOLUTIONS: int = 32_767
"""Vendor Q1 / Q11 multi-turn retention budget.

Rotating the motor more than this many turns while the drive is
powered off exhausts the encoder's multi-turn counter, and the
persisted-home-anchor restart trust path must fail closed.
"""


# ---------------------------------------------------------------------------
# Kinematics: immutable per-axis math constants.
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class A6ECAxisKinematics:
    """Immutable per-axis math constants for an A6-EC axis.

    All derived quantities (``counts_per_unit``, ``rm_counts``) are
    computed once at construction time and exposed as attributes so
    call sites never need to rederive them.

    Parameters
    ----------
    encoder_counts_per_rev:
        Encoder counts per single motor revolution. Always
        ``A6EC_COUNTS_PER_MOTOR_REV`` on the A6-EC today; parameterized
        so the module can be reused against other drive families.
    gear_ratio_num / gear_ratio_den:
        Output-shaft reduction. In the drive-native posture these are
        programmed into ``C10.18 / C10.19`` so the drive itself reports
        output-shaft positions in ``6064``/``U40.28``. The host MUST
        carry the exact same ratio here so host math agrees with the
        drive's frame.
    sign:
        ``+1`` or ``-1``. Flips the direction convention between motor
        shaft rotation and planner/joint-space positive direction.
    master_offset_rad:
        Signed joint-space offset between the anchored canonical zero
        and the robot's kinematic zero. Added on the command path and
        subtracted on the feedback path so callers see a planner-native
        zero that matches the robot's URDF/IK conventions.
    """

    encoder_counts_per_rev: int
    gear_ratio_num: int
    gear_ratio_den: int
    sign: int = 1
    master_offset_rad: float = 0.0

    def __post_init__(self) -> None:
        if self.encoder_counts_per_rev <= 0:
            raise ValueError("encoder_counts_per_rev must be > 0")
        if self.gear_ratio_num <= 0 or self.gear_ratio_den <= 0:
            raise ValueError("gear_ratio_num/den must be > 0 (rotation-mode RM definition)")
        if self.sign not in (-1, 1):
            raise ValueError("sign must be +1 or -1")

    @property
    def gear_ratio(self) -> Fraction:
        """Exact output-shaft gear ratio as a :class:`fractions.Fraction`."""
        return Fraction(int(self.gear_ratio_num), int(self.gear_ratio_den))

    @property
    def rm_counts(self) -> int:
        """Single-turn shaft period ``RM`` in drive-native command units.

        Vendor email 2 Q1:

            ``RM = encoder_resolution * C10.18 / C10.19``

        This is the period at which ``6064`` and ``607A`` wrap in
        rotation mode. In our drive-native posture (ratio programmed
        into the drive) the period equals one full OUTPUT-shaft
        revolution, which is ``counts_per_unit * 2*pi`` as well.
        """
        return int(self.encoder_counts_per_rev * self.gear_ratio_num // self.gear_ratio_den)

    @property
    def counts_per_unit(self) -> float:
        """Counts per radian of output-shaft rotation.

        Because ``RM`` already equals one shaft revolution and one
        shaft revolution is ``2*pi`` radians, this is simply
        ``RM / (2*pi)``. The host MUST NOT multiply by the gearbox
        ratio on top of this -- the ratio is already baked into
        ``rm_counts`` via ``C10.18/.19`` on the drive side.
        """
        return float(self.rm_counts) / (2.0 * math.pi)

    @property
    def motor_counts_per_unit(self) -> float:
        """Motor-side counts per radian of output-shaft rotation.

        Same as ``counts_per_unit`` in the drive-native posture (where
        ``6064`` already lives on the output shaft), but retained as a
        distinct name so call sites that consume the raw
        ``U40.20/.22`` multi-turn counts can be explicit about the
        fact that motor counts scale the same way per radian of
        output-shaft motion once the gear ratio is factored in.
        """
        return float(self.rm_counts) / (2.0 * math.pi)


# ---------------------------------------------------------------------------
# Raw encoder reconstruction (Chapter 5).
# ---------------------------------------------------------------------------


def sign_extend_16(value: int) -> int:
    """Sign-extend a 16-bit value to a Python signed int.

    Per the frame-semantics note:

        ``sign_extend_16(x) = x`` when ``x < 32768``;
        ``sign_extend_16(x) = x - 65536`` when ``x >= 32768``.

    This is required because ``U40.1E`` is documented as a 16-bit
    motor-revolution word, and the driver sometimes hands it up as an
    unsigned 16-bit quantity in SDO payloads.
    """
    value = int(value) & 0xFFFF
    return value - 0x10000 if value >= 0x8000 else value


def reconstruct_multiturn_counts_from_u40_1c_1e(
    *,
    u40_1c: int,
    u40_1e: int,
    counts_per_motor_rev: int = A6EC_COUNTS_PER_MOTOR_REV,
) -> int:
    """Chapter 5 raw multi-turn reconstruction from ``U40.1C``+``U40.1E``.

    Formulas (verbatim from the frame-semantics note):

        single_turn_mod      = U40.1C mod counts_per_motor_rev
        signed_multiturn     = sign_extend_16(U40.1E)
        reconstructed_counts = signed_multiturn * counts_per_motor_rev
                              + single_turn_mod

    This is the "encoder count plus rotation count times 131072"
    arithmetic the a6ec_chapter5_probe confirms to within 0-3 counts
    of ``combined(U40.20/.22)`` on every stationary sample.
    """
    if counts_per_motor_rev <= 0:
        raise ValueError("counts_per_motor_rev must be > 0")
    single_turn_mod = int(u40_1c) % int(counts_per_motor_rev)
    signed_multiturn = sign_extend_16(u40_1e)
    return int(signed_multiturn) * int(counts_per_motor_rev) + int(single_turn_mod)


def combine_signed_i64_pair(*, low: int, high: int) -> int:
    """Combine two 32-bit words into a signed 64-bit integer.

    Used by ``U40.20/.22`` (raw multi-turn counts, preferred truth
    source on the A6-EC) and ``U40.2A/.2C`` (rotation-mode encoder
    counts).

    Bit layout:

        combined_u64 = (high & 0xFFFFFFFF) << 32 | (low & 0xFFFFFFFF)
        if high bit 31 is set:
            combined -= 2**64

    Always returns a Python :class:`int` (arbitrary precision).
    """
    combined = ((int(high) & 0xFFFFFFFF) << 32) | (int(low) & 0xFFFFFFFF)
    if combined >= (1 << 63):
        combined -= 1 << 64
    return int(combined)


# ---------------------------------------------------------------------------
# Counts <-> joint radians (drive-native posture, rotation mode).
# ---------------------------------------------------------------------------


def axis_q_rad_from_counts(
    *,
    counts: int,
    kinematics: A6ECAxisKinematics,
) -> float:
    """Convert a signed count reading to axis-space radians.

    Works for both the raw multi-turn truth source (continuous across
    shaft turns) and the reference-frame ``6064`` reading (sawtooth
    in ``[0, RM-1]``). Which frame the result represents is a property
    of the INPUT counts, not of this conversion.

    Equation:

        axis_q = counts / (sign * counts_per_unit)
    """
    denom = float(kinematics.sign) * float(kinematics.counts_per_unit)
    if denom == 0.0:
        raise ValueError("counts_per_unit * sign resolved to 0; invalid kinematics")
    return float(counts) / denom


def counts_from_axis_q_rad(
    *,
    axis_q_rad: float,
    kinematics: A6ECAxisKinematics,
) -> float:
    """Inverse of :func:`axis_q_rad_from_counts`.

    Returns ``float`` so callers control the rounding convention; the
    command path quantizes to int only at the wire boundary.

    Equation:

        counts = axis_q * sign * counts_per_unit
    """
    return float(axis_q_rad) * float(kinematics.sign) * float(kinematics.counts_per_unit)


# ---------------------------------------------------------------------------
# Home anchor + canonical joint truth.
# ---------------------------------------------------------------------------


def compute_home_anchor_rad(
    *,
    absolute_axis_q_rad: float,
    reference_axis_q_rad: float,
) -> float:
    """Compute the absolute-home anchor at HM35 success time.

    Per the frame-semantics note's "Restart Trust via Persisted Home
    Anchor" section:

        home_anchor_rad = absolute_axis_q - reference_axis_q

    This relationship is invariant under manual rotation while the
    drive is off, because ``absolute_axis_q`` (from ``U40.20/.22``)
    and ``reference_axis_q`` (from ``6064``) track the mechanical
    motion identically mod-``2*pi``; the anchor captures exactly the
    mapping between the drive's sawtooth reference frame and the
    continuous multi-turn absolute frame.
    """
    return float(absolute_axis_q_rad) - float(reference_axis_q_rad)


def canonical_joint_q_rad(
    *,
    absolute_axis_q_rad: float,
    home_anchor_rad: float,
    kinematics: A6ECAxisKinematics,
) -> float:
    """Compute the canonical, planner-space joint angle.

    This is the "Canonical Truth Math" contract from the frame note:

        canonical_q = absolute_axis_q - home_anchor - master_offset

    It is the only joint position that stays continuous across shaft
    seams (because it rides ``U40.20/.22``) AND across drive power
    cycles (because the anchor and the multi-turn counter both
    persist). ``6064`` alone cannot carry this invariant for any
    joint whose software limits straddle more than one shaft
    revolution.
    """
    return (
        float(absolute_axis_q_rad)
        - float(home_anchor_rad)
        - float(kinematics.master_offset_rad)
    )


# ---------------------------------------------------------------------------
# Shaft-frame consistency gate (mod-RM agreement with live 6064).
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class ShaftFrameConsistencyResult:
    """Outcome of the mod-RM shaft-frame consistency gate.

    Mirrors the fields produced by the backend's
    ``_shaft_frame_consistency_detail`` so downstream diagnostics can
    be cross-checked without re-implementing the math there.
    """

    consistent: bool
    mod_rm_delta_counts: float
    mod_rm_delta_rad: float
    tolerance_counts: float
    tolerance_rad: float
    period_counts: int
    wrap_turns: int
    expected_reference_counts: float
    live_reference_counts: int


def shaft_frame_consistency(
    *,
    canonical_q_rad: float,
    live_reference_counts: int,
    kinematics: A6ECAxisKinematics,
    tolerance_counts: float = 16.0,
) -> ShaftFrameConsistencyResult:
    """Gate that rejects canonical truth if it disagrees with live 6064.

    Per Workstream 3 in the frame note:

        expected_counts = (canonical_q + master_offset) * sign * cpu
        delta           = expected_counts - live_reference_counts
        wrap_turns      = round(delta / RM)
        mod_rm_delta    = delta - wrap_turns * RM
        consistent      = abs(mod_rm_delta) <= tolerance_counts

    A whole-shaft-turn offset is legitimate (that is precisely what
    the absolute-home anchor encodes for a multi-turn joint), so the
    gate folds ``delta`` to the nearest shaft turn and only fires on
    the sub-shaft-turn residual. The default tolerance of ``16 counts``
    matches the A6-EC probe's observed stationary jitter ceiling
    (``0..9`` counts post-restart, well below ``16``).

    ``live_reference_counts`` is the raw ``6064`` reading in wire
    units. Do NOT pass a pre-subtracted value; the math folds it
    itself.
    """
    if tolerance_counts <= 0.0:
        raise ValueError("tolerance_counts must be > 0")
    period = kinematics.rm_counts
    if period <= 0:
        raise ValueError("rm_counts must be > 0; check gear ratio")
    expected_reference_q = float(canonical_q_rad) + float(kinematics.master_offset_rad)
    expected_counts = counts_from_axis_q_rad(
        axis_q_rad=expected_reference_q,
        kinematics=kinematics,
    )
    delta = float(expected_counts) - float(live_reference_counts)
    wrap_turns = int(round(delta / float(period)))
    mod_rm_delta_counts = delta - float(wrap_turns * period)
    cpu = float(kinematics.counts_per_unit)
    sign = float(kinematics.sign)
    if cpu <= 0.0 or sign == 0.0:
        raise ValueError("kinematics produced a zero conversion factor")
    mod_rm_delta_rad = float(mod_rm_delta_counts) / (sign * cpu)
    tolerance_rad = float(tolerance_counts) / (sign * cpu)
    return ShaftFrameConsistencyResult(
        consistent=abs(mod_rm_delta_counts) <= float(tolerance_counts),
        mod_rm_delta_counts=float(mod_rm_delta_counts),
        mod_rm_delta_rad=float(mod_rm_delta_rad),
        tolerance_counts=float(tolerance_counts),
        tolerance_rad=float(tolerance_rad),
        period_counts=int(period),
        wrap_turns=int(wrap_turns),
        expected_reference_counts=float(expected_counts),
        live_reference_counts=int(live_reference_counts),
    )


# ---------------------------------------------------------------------------
# Command path: stateless nearest-turn fold with wrap-to-[0, RM).
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class CommandFoldResult:
    """Outcome of folding a canonical_q target into wire-frame 607A.

    ``adjusted_counts`` is the ``607A`` value that should be shipped to
    the drive. ``wrap_lift_counts`` is the ``round(delta/RM) * RM``
    offset the fold applied before the optional single-turn wrap; it
    is retained for diagnostic roundtrip telemetry.
    """

    adjusted_counts: int
    wrap_lift_counts: int
    linear_delta_counts: float
    angular_delta_counts: float
    rm_counts: int


def shortest_angular_counts(
    *,
    linear_delta_counts: float,
    rm_counts: int,
) -> float:
    """Fold a linear count delta into ``(-RM/2, +RM/2]``.

    This matches how the A6-EC drive internally resolves motion under
    ``C10.16 = 0`` (Nearest / shortest path). It is the sole basis for
    distinguishing a "seam-crossing but physically tiny" step from a
    "real 180 deg excursion" step in every downstream safety check.
    """
    if rm_counts <= 0:
        raise ValueError("rm_counts must be > 0")
    period = float(rm_counts)
    half = 0.5 * period
    return ((float(linear_delta_counts) + half) % period) - half


def fold_canonical_q_to_command_counts(
    *,
    canonical_q_rad: float,
    live_reference_counts: int,
    kinematics: A6ECAxisKinematics,
    wrap_to_single_turn: bool = True,
) -> CommandFoldResult:
    """Stateless, per-write nearest-turn fold.

    This is the mathematical core of the 2026-04-17 J6 fix. It takes
    the target joint angle in canonical (planner) space, the drive's
    live ``6064`` reading, and the axis's math constants, and returns
    a wire-frame ``607A`` value that:

    1. Lands in the same shaft turn as live ``6064`` in LINEAR counts
       (step 1, classical nearest-turn fold).
    2. Is then wrapped into the drive's ``[0, RM)`` single-turn
       presentation range (step 2, only when ``wrap_to_single_turn``
       is True -- which the command path ALWAYS sets).

    Step 2 is what the A6-EC firmware actually requires. Without it,
    commanding ``607A = RM + k`` while live ``6064 = RM - 26`` caused
    the J6 360-deg long-way incident even with ``C10.16 = 0`` pinned.
    Callers that need a linear-windowed value for diagnostic roundtrip
    comparisons (e.g. ``command_roundtrip_reference_error``) can pass
    ``wrap_to_single_turn=False`` to recover the pre-wrap behavior.

    Equations:

        base_counts         = canonical_q + master_offset * sign * cpu
        delta               = live_reference_counts - base_counts
        wrap_turns          = round(delta / RM)
        wrap_lift_counts    = wrap_turns * RM
        adjusted_counts     = base_counts + wrap_lift_counts
        if wrap_to_single_turn:
            adjusted_counts = adjusted_counts mod RM        (= [0, RM))

    The returned ``angular_delta_counts`` is the SHORTEST-ANGULAR
    delta between the wrapped target and live reference and is used
    downstream by :func:`enforce_command_frame_oversized_delta` and
    the trajectory-safety gates.
    """
    period = kinematics.rm_counts
    if period <= 0:
        raise ValueError("rm_counts must be > 0; check gear ratio")
    base_axis_q = float(canonical_q_rad) + float(kinematics.master_offset_rad)
    base_counts = counts_from_axis_q_rad(
        axis_q_rad=base_axis_q,
        kinematics=kinematics,
    )
    live_counts_f = float(live_reference_counts)
    delta = live_counts_f - float(base_counts)
    wrap_turns = int(round(delta / float(period)))
    wrap_lift_counts = wrap_turns * int(period)
    adjusted_counts_f = float(base_counts) + float(wrap_lift_counts)
    if wrap_to_single_turn:
        period_f = float(period)
        adjusted_counts_f = adjusted_counts_f - period_f * math.floor(
            adjusted_counts_f / period_f
        )
        if adjusted_counts_f < 0.0:
            adjusted_counts_f += period_f
        if adjusted_counts_f >= period_f:
            adjusted_counts_f -= period_f
    linear_delta = float(adjusted_counts_f) - live_counts_f
    angular_delta = shortest_angular_counts(
        linear_delta_counts=linear_delta,
        rm_counts=int(period),
    )
    return CommandFoldResult(
        adjusted_counts=int(round(adjusted_counts_f)),
        wrap_lift_counts=int(wrap_lift_counts),
        linear_delta_counts=float(linear_delta),
        angular_delta_counts=float(angular_delta),
        rm_counts=int(period),
    )


def inverse_canonical_q_from_command_counts(
    *,
    command_counts: int,
    live_reference_counts: int,
    kinematics: A6ECAxisKinematics,
) -> float:
    """Recover ``canonical_q`` from a wrapped ``607A`` command value.

    Used by the backend's ``_canonical_joint_q_from_command_axis_q``
    reverse map to keep the last-submitted joint setpoint aligned
    with what the drive actually accepted after the fold. It re-folds
    the command against the live reference so consecutive seam-
    adjacent writes cannot lose track of the current shaft turn.
    """
    period = kinematics.rm_counts
    if period <= 0:
        raise ValueError("rm_counts must be > 0; check gear ratio")
    target_counts = float(command_counts)
    live_counts = float(live_reference_counts)
    delta = target_counts - live_counts
    wrap_turns = int(round(delta / float(period)))
    base_counts = target_counts - float(wrap_turns * int(period))
    base_axis_q = axis_q_rad_from_counts(
        counts=int(round(base_counts)),
        kinematics=kinematics,
    )
    return float(base_axis_q) - float(kinematics.master_offset_rad)


# ---------------------------------------------------------------------------
# Trajectory wire-frame safety gates.
# ---------------------------------------------------------------------------


DEFAULT_TRAJECTORY_MAX_PER_POINT_STEP_RAD: float = 0.35
"""Joint-space step bound between consecutive queued points (~20 deg).

A bounded move at typical commissioning rates stays well under this;
a mid-trajectory fold flip would produce a step of roughly one shaft
revolution (``2*pi`` rad) and trip the gate by ~18x.
"""

DEFAULT_TRAJECTORY_MAX_FIRST_POINT_DEVIATION_RAD: float = 0.35
"""Joint-space bound on point-0 deviation from live 6064 (~20 deg).

Point 0's canonical_q is drawn from live feedback, so after the fold
its wire counts should land essentially on top of live ``6064``. Any
larger deviation means the fold, the rotation-mode config, or the
anchor picked the wrong shaft turn and would teleport the joint.
"""


@dataclass(frozen=True)
class WireFrameSafetyViolation:
    """Structured description of a trajectory wire-frame rejection.

    The backend raises a :class:`RuntimeError` with this payload
    rendered into the message; the dataclass exists so unit tests and
    operator diagnostics can inspect individual fields without
    regex-parsing the string.
    """

    code: str
    axis_i: int
    logical_joint_idx: int
    target_counts: int
    live_counts: Optional[int]
    step_counts: Optional[int]
    linear_step_counts: Optional[int]
    deviation_counts: Optional[int]
    linear_deviation_counts: Optional[int]
    period_counts: int

    def as_runtime_error_message(self) -> str:
        """Render the violation the same way the backend's runtime error does."""
        parts = [self.code + ":", f"axis={self.axis_i}", f"joint={self.logical_joint_idx + 1}"]
        if self.target_counts is not None:
            parts.append(f"target_counts={self.target_counts}")
        if self.live_counts is not None:
            parts.append(f"live_counts={self.live_counts}")
        if self.step_counts is not None:
            parts.append(f"step_counts={self.step_counts}")
        if self.linear_step_counts is not None:
            parts.append(f"linear_step_counts={self.linear_step_counts}")
        if self.deviation_counts is not None:
            parts.append(f"deviation_counts={self.deviation_counts}")
        if self.linear_deviation_counts is not None:
            parts.append(f"linear_deviation_counts={self.linear_deviation_counts}")
        parts.append(f"period_counts={self.period_counts}")
        return " ".join(parts)


def check_per_point_step(
    *,
    axis_i: int,
    logical_joint_idx: int,
    current_counts: int,
    previous_counts: int,
    kinematics: A6ECAxisKinematics,
    max_step_rad: float = DEFAULT_TRAJECTORY_MAX_PER_POINT_STEP_RAD,
    seam_crossing_unsafe: bool = True,
) -> Optional[WireFrameSafetyViolation]:
    """Per-point trajectory step gate.

    Returns None if the step is safe, or a
    :class:`WireFrameSafetyViolation` describing which guard fired.
    Two independent checks live here:

    * Shortest-angular step must fit under ``max_step_rad``. This
      catches a real operator-facing angular excursion regardless of
      which side of the ``0/RM`` seam the points live on.
    * If ``seam_crossing_unsafe`` is set and the LINEAR step straddles
      the seam (``abs(linear_step) > RM/2``), fail closed. Live
      A6-EC verification on 2026-04-17 showed that even when the
      shortest-angular step is tiny, a seam-straddling absolute
      ``607A`` point sequence can still execute the long way by ~RM
      counts on current firmware. Profiles that opt in to
      ``command_frame_seam_crossing_unsafe`` refuse to submit such
      sequences until a seam-biased wire-frame policy is validated.
    """
    period = kinematics.rm_counts
    if period <= 0:
        return None
    cpu = float(kinematics.counts_per_unit)
    if cpu <= 0.0:
        return None
    max_step_counts = int(round(float(max_step_rad) * cpu))
    linear_step = int(current_counts) - int(previous_counts)
    step_counts = int(round(shortest_angular_counts(
        linear_delta_counts=float(linear_step),
        rm_counts=int(period),
    )))
    if max_step_counts > 0 and abs(step_counts) > max_step_counts:
        return WireFrameSafetyViolation(
            code="command_frame_oversized_step",
            axis_i=int(axis_i),
            logical_joint_idx=int(logical_joint_idx),
            target_counts=int(current_counts),
            live_counts=None,
            step_counts=int(step_counts),
            linear_step_counts=int(linear_step),
            deviation_counts=None,
            linear_deviation_counts=None,
            period_counts=int(period),
        )
    half_period = int(period) // 2
    if seam_crossing_unsafe and abs(linear_step) > half_period:
        return WireFrameSafetyViolation(
            code="command_frame_seam_crossing_step_disallowed",
            axis_i=int(axis_i),
            logical_joint_idx=int(logical_joint_idx),
            target_counts=int(current_counts),
            live_counts=None,
            step_counts=int(step_counts),
            linear_step_counts=int(linear_step),
            deviation_counts=None,
            linear_deviation_counts=None,
            period_counts=int(period),
        )
    return None


def check_first_point_live_deviation(
    *,
    axis_i: int,
    logical_joint_idx: int,
    current_counts: int,
    live_reference_counts: int,
    kinematics: A6ECAxisKinematics,
    max_deviation_rad: float = DEFAULT_TRAJECTORY_MAX_FIRST_POINT_DEVIATION_RAD,
    seam_crossing_unsafe: bool = True,
) -> Optional[WireFrameSafetyViolation]:
    """First-point deviation gate.

    Same logic as :func:`check_per_point_step` but with the "previous"
    slot filled from live 6064. Applied ONLY to point 0 of an uploaded
    trajectory, because later points may legitimately travel far from
    live over the course of a long bounded move.
    """
    period = kinematics.rm_counts
    if period <= 0:
        return None
    cpu = float(kinematics.counts_per_unit)
    if cpu <= 0.0:
        return None
    max_deviation_counts = int(round(float(max_deviation_rad) * cpu))
    linear_deviation = int(current_counts) - int(live_reference_counts)
    deviation_counts = int(round(shortest_angular_counts(
        linear_delta_counts=float(linear_deviation),
        rm_counts=int(period),
    )))
    if max_deviation_counts > 0 and abs(deviation_counts) > max_deviation_counts:
        return WireFrameSafetyViolation(
            code="command_frame_live_deviation_out_of_range",
            axis_i=int(axis_i),
            logical_joint_idx=int(logical_joint_idx),
            target_counts=int(current_counts),
            live_counts=int(live_reference_counts),
            step_counts=None,
            linear_step_counts=None,
            deviation_counts=int(deviation_counts),
            linear_deviation_counts=int(linear_deviation),
            period_counts=int(period),
        )
    half_period = int(period) // 2
    if seam_crossing_unsafe and abs(linear_deviation) > half_period:
        return WireFrameSafetyViolation(
            code="command_frame_seam_crossing_first_point_disallowed",
            axis_i=int(axis_i),
            logical_joint_idx=int(logical_joint_idx),
            target_counts=int(current_counts),
            live_counts=int(live_reference_counts),
            step_counts=None,
            linear_step_counts=None,
            deviation_counts=int(deviation_counts),
            linear_deviation_counts=int(linear_deviation),
            period_counts=int(period),
        )
    return None


def enforce_trajectory_wire_frame_safety(
    *,
    axis_i: int,
    logical_joint_idx: int,
    point_counts: Iterable[int],
    live_reference_counts: int,
    kinematics: A6ECAxisKinematics,
    max_step_rad: float = DEFAULT_TRAJECTORY_MAX_PER_POINT_STEP_RAD,
    max_first_point_deviation_rad: float = DEFAULT_TRAJECTORY_MAX_FIRST_POINT_DEVIATION_RAD,
    seam_crossing_unsafe: bool = True,
) -> None:
    """Walk an uploaded axis trajectory and fail closed on safety violations.

    Mirrors the backend's ``_enforce_trajectory_wire_frame_safety``
    behavior: check the first point against live 6064, then each
    subsequent point against its predecessor. On the first violation
    the function raises a :class:`RuntimeError` whose message matches
    the existing backend log format so operator tooling that parses
    those lines keeps working unchanged.
    """
    points = [int(c) for c in point_counts]
    if not points:
        return
    first_violation = check_first_point_live_deviation(
        axis_i=axis_i,
        logical_joint_idx=logical_joint_idx,
        current_counts=points[0],
        live_reference_counts=live_reference_counts,
        kinematics=kinematics,
        max_deviation_rad=max_first_point_deviation_rad,
        seam_crossing_unsafe=seam_crossing_unsafe,
    )
    if first_violation is not None:
        raise RuntimeError(first_violation.as_runtime_error_message())
    for idx in range(1, len(points)):
        step_violation = check_per_point_step(
            axis_i=axis_i,
            logical_joint_idx=logical_joint_idx,
            current_counts=points[idx],
            previous_counts=points[idx - 1],
            kinematics=kinematics,
            max_step_rad=max_step_rad,
            seam_crossing_unsafe=seam_crossing_unsafe,
        )
        if step_violation is not None:
            raise RuntimeError(step_violation.as_runtime_error_message())


# ---------------------------------------------------------------------------
# HM35 bias helper -- choose 607C so the operator's normal working zone
# sits away from the 0/RM seam.
# ---------------------------------------------------------------------------


def hm35_origin_offset_biased_to_midpoint(
    *,
    kinematics: A6ECAxisKinematics,
    fraction_numerator: int = 1,
    fraction_denominator: int = 2,
) -> int:
    """Compute a ``607C`` value biased to ``RM * num / den``.

    Vendor email 2 Q6 says ``607C`` must live in ``[0, RM-1]`` in
    rotation mode, and recommends a positive value near ``RM-1`` for
    seam-adjacent homes rather than a negative offset. Our production
    posture keeps the canonical zero in host-owned space (via the
    absolute anchor captured post-HM35) and biases the drive's
    sawtooth presentation to the midpoint of its wrap period so
    small operator-commanded jogs don't live on the raw seam.

    The default ``1/2`` bias matches
    ``NATIVE_HOME_CONFIG["transaction"]`` in the A6-EC drive profile;
    callers can override for seam-adjacent joints that need a
    different bias.
    """
    if fraction_denominator <= 0:
        raise ValueError("fraction_denominator must be > 0")
    if fraction_numerator < 0 or fraction_numerator >= fraction_denominator:
        raise ValueError(
            "fraction_numerator must satisfy 0 <= num < den; 607C lives in [0, RM-1]"
        )
    period = kinematics.rm_counts
    if period <= 0:
        raise ValueError("rm_counts must be > 0; check gear ratio")
    return int(period * int(fraction_numerator) // int(fraction_denominator))


__all__ = [
    "A6EC_COUNTS_PER_MOTOR_REV",
    "A6EC_MAX_OFF_MOTOR_REVOLUTIONS",
    "A6ECAxisKinematics",
    "sign_extend_16",
    "reconstruct_multiturn_counts_from_u40_1c_1e",
    "combine_signed_i64_pair",
    "axis_q_rad_from_counts",
    "counts_from_axis_q_rad",
    "compute_home_anchor_rad",
    "canonical_joint_q_rad",
    "ShaftFrameConsistencyResult",
    "shaft_frame_consistency",
    "CommandFoldResult",
    "shortest_angular_counts",
    "fold_canonical_q_to_command_counts",
    "inverse_canonical_q_from_command_counts",
    "DEFAULT_TRAJECTORY_MAX_PER_POINT_STEP_RAD",
    "DEFAULT_TRAJECTORY_MAX_FIRST_POINT_DEVIATION_RAD",
    "WireFrameSafetyViolation",
    "check_per_point_step",
    "check_first_point_live_deviation",
    "enforce_trajectory_wire_frame_safety",
    "hm35_origin_offset_biased_to_midpoint",
]
