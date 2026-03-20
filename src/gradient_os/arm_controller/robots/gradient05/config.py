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
        return [100.0, 100.0, 100.0, 18.0, 18.1818181818, 10.0] # the 18.18 ratio is from the pulled setup on the timing belt: 22T driving 20T after the 20:1 planetary gearbox

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
            (-6.3, 6.3),  # J6
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
    def default_pid_gains(self) -> tuple[int, int, int]:
        """
        Gradient-05 defaults for PID gains. Since this relies on EtherCAT RTCore,
        PID gains are generally managed by the drive (SDO configs), but we provide
        safe defaults to satisfy base class.
        """
        return (0, 0, 0)

