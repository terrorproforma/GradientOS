# arm_controller.math
#
# Pure-functional, stateless math for joint motion on EtherCAT drives.
# This package contains no I/O, no drive state, and no RTCore plumbing:
# every function takes plain numbers in, returns plain numbers out, and
# is unit-testable in isolation.
#
# The production command/feedback paths live in
# `arm_controller.backends.ethercat_rtcore.backend` and are responsible
# for threading live state through these functions; this package is the
# canonical, testable source for the equations themselves.

from . import a6ec_joint_motion

__all__ = ["a6ec_joint_motion"]
