# robots/fr10/config.py
#
# Configuration for the Fairino FR10 6-DOF industrial arm.
#
# The FR10 differs from the Gradient0 in important ways that shape this
# config:
#   - It is a sealed industrial appliance: the controller owns calibration,
#     PID, encoder mapping, joint limits (in firmware), and the real-time
#     control loop. GradientOS is a network client, not a driver.
#   - There are no Feetech-style "actuator IDs" exposed to us. We invent
#     synthetic IDs 1..6 so the rest of GradientOS (telemetry, UI tables)
#     has stable handles, but they don't correspond to any hardware register.
#   - PID gains, encoder mapping, and master offsets are owned by the FR10
#     controller — we override the relevant RobotConfig hooks to return safe
#     no-op values rather than letting the base class call into the
#     fairino backend's (zero) registry shims.
#
# MILESTONE 3 PROMOTION: when END_EFFECTOR_OFFSET moves off the global in
# `gradient_os.ik_solver` and onto RobotConfig, this class will declare
# `end_effector_offset = np.zeros(3)` (the FR10 owns the tool transform via
# `toolcoord1`). Until then, the frame_guard module enforces the Option B
# rule by reading the global and refusing to start if it's non-zero.

import math
from typing import Optional

from ..base import RobotConfig


# Joint limits in radians.
# TODO: confirm against FR10 datasheet / WebApp soft-limit settings before
#   Milestone 2 (motion). Values below are conservative placeholders chosen
#   to be smaller than typical FR10 hardware envelopes so a planner that
#   clamps against these will never demand motion the FR10 will reject.
#   For Milestone 1 (read-only) these are display-only — no clamping happens.
_FR10_JOINT_LIMITS_RAD: list[tuple[float, float]] = [
    (-math.radians(170.0), math.radians(170.0)),  # J1 base    — verify
    (-math.radians( 90.0), math.radians( 90.0)),  # J2 shoulder — verify
    (-math.radians(150.0), math.radians(150.0)),  # J3 elbow    — verify
    (-math.radians(170.0), math.radians(170.0)),  # J4 wrist 1  — verify
    (-math.radians(120.0), math.radians(120.0)),  # J5 wrist 2  — verify
    (-math.radians(180.0), math.radians(180.0)),  # J6 wrist 3  — verify
]


class FR10Config(RobotConfig):
    """RobotConfig for the Fairino FR10 6-DOF industrial arm.

    Pairs with backends/fairino/. The default_servo_backend is "fairino" so
    `gradient-controller --robot fr10` selects the right backend without an
    explicit --backend flag.
    """

    # =========================================================================
    # Identity
    # =========================================================================

    @property
    def name(self) -> str:
        return "FR10"

    @property
    def version(self) -> str:
        return "1.0.0"

    @property
    def default_servo_backend(self) -> str:
        return "fairino"

    # =========================================================================
    # Kinematic structure
    # =========================================================================

    @property
    def num_logical_joints(self) -> int:
        return 6

    @property
    def num_physical_actuators(self) -> int:
        # No twin motors; one logical joint per controlled axis.
        return 6

    @property
    def actuator_ids(self) -> list[int]:
        # Synthetic IDs 1..6 — the FR10 doesn't expose hardware servo IDs
        # to network clients. These exist only to give telemetry and UI
        # tables stable keys.
        return [1, 2, 3, 4, 5, 6]

    @property
    def logical_to_physical_map(self) -> dict[int, list[int]]:
        return {i: [i] for i in range(6)}

    # =========================================================================
    # Joint limits (display only at Milestone 1)
    # =========================================================================

    @property
    def logical_joint_limits_rad(self) -> list[tuple[float, float]]:
        return list(_FR10_JOINT_LIMITS_RAD)

    @property
    def actuator_limits_rad(self) -> list[tuple[float, float]]:
        # 1:1 with logical joints — there is no twin-motor or gear-ratio
        # remapping at this layer; the FR10 controller owns that.
        return list(_FR10_JOINT_LIMITS_RAD)

    # =========================================================================
    # Encoder mapping — not applicable
    # =========================================================================

    @property
    def actuator_mapping_ranges_rad(self) -> list[tuple[float, float]]:
        # Required by the base class but unused — the Fairino backend works
        # in radians end-to-end and never converts to raw encoder counts.
        # Return ±π so any code that does try to use it gets a benign value.
        return [(-math.pi, math.pi)] * 6

    @property
    def inverted_actuator_ids(self) -> set[int]:
        # The FR10 reports joint angles in its own canonical convention via
        # the SDK. Sign handling is the controller's problem, not ours.
        return set()

    # =========================================================================
    # Calibration — owned by the FR10 controller
    # =========================================================================

    @property
    def logical_joint_master_offsets_rad(self) -> list[float]:
        # Calibration lives on the FR10 (its zero-position teach is an
        # operator-side workflow on the WebApp / pendant). GradientOS adds
        # nothing on top.
        return [0.0] * 6

    # =========================================================================
    # Gripper — TBD
    # =========================================================================

    @property
    def gripper_actuator_id(self) -> Optional[int]:
        # The FR10's gripper / welder I/O surface is Milestone 7. Set to
        # None for now so `has_gripper` reports False and the gripper UI
        # stays hidden.
        return None

    # =========================================================================
    # Motion parameters
    # =========================================================================

    @property
    def default_speed(self) -> int:
        # Fairino's MoveJ/MoveL accept a speed percentage (0–100).
        # 5 (5%) is the bring-up speed recommended in
        # FR10_GRADIENTOS_INTEGRATION_PLAN.md for Milestone 2.
        return 5

    @property
    def default_acceleration_deg_s2(self) -> float:
        # Acceleration is owned by the FR10 controller; this is a no-op.
        # Return a non-zero value so any consumer that divides by it
        # doesn't blow up.
        return 100.0

    # =========================================================================
    # PID — owned by the FR10 servo drivers, not user-tunable from here
    # =========================================================================

    @property
    def default_pid_gains(self) -> tuple[int, int, int]:
        # Override the base class default (which calls into the active
        # backend's config). Fairino has no PID concept at the network
        # client layer — return zeros so anyone who reads this is forced to
        # think about whether they actually need it for an FR10.
        return (0, 0, 0)

    @property
    def actuator_pid_gains(self) -> dict[int, tuple[int, int, int]]:
        return {aid: (0, 0, 0) for aid in self.actuator_ids}
