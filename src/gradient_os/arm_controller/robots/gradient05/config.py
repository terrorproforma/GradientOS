# robots/gradient05/config.py
#
# Template configuration for the Gradient-05 robot.
#
# This starts as a compatibility baseline by inheriting Gradient0 settings so
# the robot can be selected immediately in simulation. Replace overridden and
# inherited properties as hardware specifics become available.

from ..gradient0 import Gradient0Config


class Gradient05Config(Gradient0Config):
    """Template robot config for Gradient-05."""

    @property
    def robot_id(self) -> str:
        return "gradient-05"

    @property
    def name(self) -> str:
        return "Gradient-05"

    @property
    def version(self) -> str:
        return "0.1.0-template"

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
