# backends/fairino/frame_guard.py
#
# Enforces the Option B coordinate-frame strategy chosen for FR10 integration:
# the FR10 controller owns the calibrated tool/workpiece frames; GradientOS
# does NOT apply its own END_EFFECTOR_OFFSET on top. Mixing the two is the
# "offset-squared" bug — robot ends up double-offset, crashes the fixture.
# See Fairino-FR10/Docs/FR10_GRADIENTOS_COMPARISON.md for the full rationale.
#
# This module is called from FairinoBackend.initialize() *before* any network
# I/O. If the guard trips, we fail loud with FrameStrategyViolation rather
# than connecting and silently letting the user command motion.
#
# MILESTONE 3 PROMOTION: this guard goes away when END_EFFECTOR_OFFSET is
# moved off `gradient_os.ik_solver` (a global) and onto RobotConfig (a
# per-robot field). At that point the FR10 RobotConfig declares
# `end_effector_offset = np.zeros(3)` and ik_solver reads it from the active
# robot — there is nothing left to "double up" because the value is owned by
# exactly one place. Until that refactor lands, this guard is the one thing
# standing between us and a 380 mm crash.

from __future__ import annotations

import numpy as np


# Tolerance for "is this offset effectively zero?". 1 micron is well below
# any meaningful mechanical resolution and well above floating-point noise.
_IDENTITY_TOLERANCE_M = 1e-6


class FrameStrategyViolation(RuntimeError):
    """Raised when GradientOS's frame state conflicts with FR10's.

    Carries instructions for the operator in str(exc) — the message is the
    primary fix-it artifact, so keep it specific.
    """


def assert_identity_end_effector_offset() -> None:
    """Refuse to run Fairino if GradientOS holds a non-zero tool offset.

    Reads the current value of `gradient_os.ik_solver.END_EFFECTOR_OFFSET`
    fresh on every call — no caching — so a developer who patches the value
    at runtime gets the new behavior on the next initialize().

    Raises:
        FrameStrategyViolation: if the offset is non-identity. The message
            includes the actual offset value and the exact one-line edit.
    """
    # Imported inline to avoid pulling ik_solver (and numpy) into module-load
    # cost for environments that don't actually use the Fairino backend.
    from gradient_os.ik_solver import END_EFFECTOR_OFFSET

    offset = np.asarray(END_EFFECTOR_OFFSET, dtype=float).reshape(-1)
    if offset.size != 3:
        raise FrameStrategyViolation(
            f"END_EFFECTOR_OFFSET has unexpected shape {offset.shape}; "
            "expected a 3-vector."
        )

    if float(np.linalg.norm(offset)) <= _IDENTITY_TOLERANCE_M:
        return  # identity — Option B holds, FR10 owns the tool frame.

    raise FrameStrategyViolation(
        "Refusing to start the Fairino backend: the FR10 controller already "
        "applies its calibrated tool frame (Option B), and "
        f"gradient_os.ik_solver.END_EFFECTOR_OFFSET is currently {offset.tolist()} "
        "(non-zero). Running with both active produces an offset-squared error "
        "and will crash the robot into the fixture.\n"
        "\n"
        "Fix: edit src/gradient_os/ik_solver.py line ~33 to "
        "`END_EFFECTOR_OFFSET = np.array([0.0, 0.0, 0.0], dtype=float)` while "
        "the Fairino backend is active. Restore the original value when you "
        "switch back to feetech / ethercat_rtcore.\n"
        "\n"
        "Followup (Milestone 3): END_EFFECTOR_OFFSET will move from this "
        "global onto RobotConfig so each robot owns its own value and this "
        "guard becomes unnecessary."
    )
