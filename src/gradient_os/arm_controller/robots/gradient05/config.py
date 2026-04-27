# robots/gradient05/config.py
#
# Configuration for the Gradient-05 robotic arm.

import math
from typing import Optional

from ..base import RobotConfig

class Gradient05Config(RobotConfig):
    """
    Configuration for the Gradient-05 6-DOF robotic arm.
    """

    # =========================================================================
    # Robot Identity
    # =========================================================================

    @property
    def robot_id(self) -> str:
        return "gradient-05"

    @property
    def name(self) -> str:
        return "Gradient-05"

    @property
    def version(self) -> str:
        return "1.0.0"

    @property
    def default_servo_backend(self) -> str:
        """
        Gradient-05 production hardware path uses EtherCAT RTCore.
        Simulation mode still maps to the in-memory backend via controller mode.
        """
        return "ethercat_rtcore"

    @property
    def default_ik_solver_backend(self) -> str:
        """Gradient-05 policy uses QuIK numeric IK."""
        return "numeric"

    # =========================================================================
    # Kinematic Structure
    # =========================================================================

    @property
    def num_logical_joints(self) -> int:
        return 6

    @property
    def num_physical_actuators(self) -> int:
        return 6

    @property
    def actuator_ids(self) -> list[int]:
        """
        EtherCAT/RTCore uses 0-based node/axis indices.
        """
        return [0, 1, 2, 3, 4, 5]

    @property
    def logical_to_physical_map(self) -> dict[int, list[int]]:
        """1-to-1 mapping for Gradient-05."""
        return {
            0: [0],
            1: [1],
            2: [2],
            3: [3],
            4: [4],
            5: [5],
        }

    @property
    def actuator_encoder_counts_per_rev(self) -> list[int]:
        """
        Absolute encoder resolution per motor revolution.

        Current Gradient-05 EtherCAT commissioning assumes 17-bit motor-side
        encoder counts from the drive position objects.
        """
        return [131072] * 6

    @property
    def actuator_gear_ratios(self) -> list[float]:
        """
        Motor-revolutions to joint-revolution reduction ratios.

        These values come from the current Gradient-05 EtherCAT bring-up notes
        and are used by the RTCore backend to convert raw counts into radians.
        """
        # J5 is commissioned as an exact 100:11 drive-native ratio.
        return [100.0, 100.0, 100.0, 18.0, 100.0 / 11.0, 10.0]

    @property
    def actuator_position_signs(self) -> list[int]:
        """
        Positive joint-direction sign convention for raw encoder counts.

        Keep this robot-defined rather than backend-defined so any future sign
        flips from wiring or gearbox changes stay in the robot description.
        """
        return [-1, 1, -1, -1, -1, -1]

    # =========================================================================
    # Joint Limits
    # =========================================================================

    @property
    def logical_joint_limits_rad(self) -> list[tuple[float, float]]:
        """
        Software-enforced motion limits (radians) for planner/controller guards.

        Operational contract:
        - Runtime checks consume this property (not URDF parsing at control time).
        - Keep this block synchronized from URDF via:
          `.\\.venv\\Scripts\\python scripts/sync_urdf_limits.py`
        """
        return [
            (-6.3, 6.3),  # J1
            (-1.9, 1.9),  # J2
            (-4.2, 1.53),  # J3
            (-6.3, 6.3),  # J4
            (-6.3, 6.3),  # J5
            (-10.0, 10.0),  # J6
        ]

    @property
    def actuator_limits_rad(self) -> list[tuple[float, float]]:
        """
        Physical actuator limits consumed by legacy serial-limit writer paths.

        Gradient-05 is 1:1 logical-to-actuator, so this mirrors logical limits.
        """
        return self.logical_joint_limits_rad

    # =========================================================================
    # Angle Mapping
    # =========================================================================

    @property
    def actuator_mapping_ranges_rad(self) -> list[tuple[float, float]]:
        """
        Angle->raw mapping span for backends that require normalized conversion.

        EtherCAT RTCore path does not use this mapping for safety limits, but
        simulation/compatibility code expects a complete list. Keep 1:1 six-axis
        entries aligned with `actuator_ids` order.
        """
        return [(-math.pi, math.pi)] * 6

    @property
    def inverted_actuator_ids(self) -> set[int]:
        """
        Inversions should be handled inside RTCore SDO config.
        """
        return set()

    # =========================================================================
    # Calibration
    # =========================================================================

    @property
    def logical_joint_master_offsets_rad(self) -> list[float]:
        return [0.0] * 6

    @property
    def ethercat_drive_startup_config(self) -> list[dict[str, int]]:
        """
        Optional robot-specific override hook.

        Gradient-05 should not own manufacturer-specific drive startup policy by
        default because the same robot may be commissioned with different drive
        families. EtherCAT drive defaults now live in the separate drive-profile
        catalog and this hook is left empty unless a robot-specific override is
        explicitly required.
        """
        return []

    @property
    def default_pid_gains(self) -> tuple[int, int, int]:
        """
        Gradient-05 defaults for PID gains. Since this relies on EtherCAT RTCore,
        PID gains are generally managed by the drive (SDO configs), but we provide
        safe defaults to satisfy base class.
        """
        return (0, 0, 0)

    @property
    def collision_watchdog_thresholds(self) -> list:
        """
        Phase 4 (2026-04-20) — per-axis collision detection bounds.

        CONSERVATIVE PLACEHOLDERS. These numbers MUST be calibrated on
        hardware during Phase 4.5:

          1. Run normal motion sweeps and jog for 5 minutes.
          2. Record peak |torque_raw| and peak |position_error_counts|
             per axis from ``GET /info/joints-detailed``.
          3. Set ``torque_abs_max_raw`` and ``position_error_counts_max``
             to ~1.5x the observed peak per axis.
          4. Controlled soft-obstacle test: intentionally push the end
             effector into foam during a slow jog. Verify the watchdog
             fires within 30 ms, no false positives during normal
             motion.

        Unit reference (A6-EC manual):
          * torque_abs_max_raw  — 0x6077 units of 0.1 % rated torque.
            2000 ~= 200 % rated; good default for soft-stop.
          * position_error_counts_max  — drive counts at the motor
            encoder resolution; on the gradient-05 17-bit encoder,
            5000 counts ~= 3.8 deg motor-shaft tracking error.

        The ``CollisionThresholds`` dataclass lives in
        ``gradient_os.arm_controller.collision_watchdog``. We return a
        plain list of objects so run_controller can import the module
        once and use the instances directly.
        """
        from ...collision_watchdog import CollisionThresholds

        return [
            # J1 — base joint, largest mass loading, widest swings.
            CollisionThresholds(
                torque_abs_max_raw=2500,
                position_error_counts_max=8000,
                sustained_samples=3,
                sample_period_s=0.01,
            ),
            # J2 — shoulder, high gravity torque under load.
            CollisionThresholds(
                torque_abs_max_raw=2500,
                position_error_counts_max=8000,
                sustained_samples=3,
                sample_period_s=0.01,
            ),
            # J3 — elbow, similar gravity profile to J2.
            CollisionThresholds(
                torque_abs_max_raw=2200,
                position_error_counts_max=7000,
                sustained_samples=3,
                sample_period_s=0.01,
            ),
            # J4 — wrist rotation, low static torque.
            CollisionThresholds(
                torque_abs_max_raw=1800,
                position_error_counts_max=5000,
                sustained_samples=3,
                sample_period_s=0.01,
            ),
            # J5 — wrist pitch, low static torque.
            CollisionThresholds(
                torque_abs_max_raw=1800,
                position_error_counts_max=5000,
                sustained_samples=3,
                sample_period_s=0.01,
            ),
            # J6 — tool flange, 10:1 gearing + single-turn wrap concerns.
            # Tighter PE threshold: J6 is the seam-crossing axis from
            # the 2026-04-17 incident and any position-error growth
            # there deserves quicker attention.
            CollisionThresholds(
                torque_abs_max_raw=1800,
                position_error_counts_max=4000,
                sustained_samples=3,
                sample_period_s=0.01,
            ),
        ]

