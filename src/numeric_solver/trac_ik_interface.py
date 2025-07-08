from __future__ import annotations

"""trac_ik_interface.py
A high-level, easy-to-use Python wrapper around the ROS *TRAC-IK* library.

This module hides most of the boiler-plate required to instantiate and query a
`trac_ik_python.trac_ik.IK` solver instance.  It is meant to be completely
robot-agnostic: the solver is configured **exclusively** from your URDF and the
`base_link`/`tip_link` names – no Denavit–Hartenberg parameters or other
hard-coded geometry assumptions are used [[memory:2236650]].

The primary entry-point is the `TracIKInterface` class, which provides:
  • Construction from a URDF string (or the ROS `/robot_description` parameter).
  • Query functions for joint limits, names, and other metadata.
  • A single `solve()` method that returns a NumPy array of joint positions for
    a desired end-effector pose.

Example
-------
>>> from trac_ik_interface import TracIKInterface
>>> import numpy as np
>>> pos = np.array([0.5, 0.0, 0.3], dtype=float)          # metres
>>> quat = np.array([0.0, 0.0, 0.0, 1.0], dtype=float)    # x, y, z, w
>>> ik = TracIKInterface("base_link", "tool0")
>>> solution = ik.solve(pos, quat)
>>> print(solution)  # array of joint angles in radians, or `None` if no IK

Installation prerequisites
-------------------------
`trac_ik_python` must be discoverable in your Python environment.  See the
project-level *README* for build instructions.  At run-time only **NumPy** and
**trac_ik_python** are required; no ROS messages or TF transforms are needed.

"""

from typing import Optional, Sequence, Tuple

import numpy as np

try:
    # `trac_ik_python` is provided by the TRAC-IK source tree once built with
    # `catkin_make` *or* by the Debian/Ubuntu binary package
    #   sudo apt-get install ros-<distro>-trac-ik
    from trac_ik_python.trac_ik import IK  # type: ignore

    _TRAC_IK_AVAILABLE: bool = True
except ImportError:  # pragma: no cover – library not present in environment
    _TRAC_IK_AVAILABLE = False

__all__ = [
    "TracIKInterface",
]


class TracIKInterface:
    """A thin, NumPy-friendly adaptor over :class:`trac_ik_python.trac_ik.IK`.

    Parameters
    ----------
    base_link : str
        Name of the chain root link in the URDF.
    tip_link : str
        Name of the chain end-effector link in the URDF.
    urdf_string : str | None, optional
        Raw URDF XML string.  If *None* (default) the wrapped TRAC-IK solver
        will attempt to read the description from the ROS parameter server
        (`/robot_description`).  Supplying the string directly makes the class
        usable *outside* of a running ROS master.
    timeout : float, default 0.005
        Maximum time (seconds) to spend searching for a solution.  Note that a
        failed query always runs for the full timeout; keep this small if you
        call :py:meth:`solve` inside a tight control loop.
    epsilon : float, default 1e-5
        Positional/orientational convergence criterion passed straight through
        to TRAC-IK.

    Raises
    ------
    ImportError
        If `trac_ik_python` cannot be imported.  Follow the *README* build
        instructions if this happens.
    RuntimeError
        If the underlying solver cannot be constructed (e.g. malformed URDF).
    """

    # ---------------------------------------------------------------------
    # Construction helpers
    # ---------------------------------------------------------------------

    def __init__(
        self,
        base_link: str,
        tip_link: str,
        *,
        urdf_string: Optional[str] = None,
        timeout: float = 0.005,
        epsilon: float = 1e-5,
    ) -> None:
        if not _TRAC_IK_AVAILABLE:
            raise ImportError(
                "`trac_ik_python` is not available.  See the project README "
                "for detailed installation instructions."
            )

        self._ik = IK(
            base_link,
            tip_link,
            urdf_string=urdf_string,
            timeout=timeout,
            epsilon=epsilon,
        )

        # Cache immutable meta-data for quick access.
        self.joint_names: Tuple[str, ...] = self._ik.joint_names
        self.link_names: Tuple[str, ...] = self._ik.link_names
        self.number_of_joints: int = self._ik.number_of_joints

    # ------------------------------------------------------------------
    # Public helpers
    # ------------------------------------------------------------------

    def get_joint_limits(self) -> Tuple[np.ndarray, np.ndarray]:
        """Return lower and upper joint limits as NumPy arrays (dtype=float)."""
        lower, upper = self._ik.get_joint_limits()
        return (np.array(lower, dtype=float), np.array(upper, dtype=float))

    def set_joint_limits(
        self,
        lower: Sequence[float],
        upper: Sequence[float],
    ) -> None:
        """Override the default joint limits.

        This is occasionally useful when the URDF omits limits (e.g. for
        continuous revolute joints) or you wish to narrow the allowed range.
        """
        self._ik.set_joint_limits(lower, upper)

    # ------------------------------------------------------------------
    # The core IK call
    # ------------------------------------------------------------------

    def solve(
        self,
        position: np.ndarray,
        orientation: np.ndarray,
        *,
        seed: Optional[np.ndarray] = None,
        pos_bounds: Optional[np.ndarray] = None,
        rot_bounds: Optional[np.ndarray] = None,
    ) -> Optional[np.ndarray]:
        """Compute inverse kinematics for a single 6-DoF pose.

        Parameters
        ----------
        position : ndarray shape (3,)
            Desired end-effector XYZ position **in metres**, expressed in the
            same base frame used by *base_link*.
        orientation : ndarray shape (4,)
            Desired end-effector orientation as a unit quaternion *(x, y, z, w)*.
        seed : ndarray shape (N,), optional
            Initial joint guess.  If *None* a mid-range pose is used.
        pos_bounds : ndarray shape (3,), optional
            Per-axis absolute bounds (metres) for XYZ error tolerance.  If
            *None* TRAC-IK's defaults are in effect.
        rot_bounds : ndarray shape (3,), optional
            Per-axis absolute bounds (radians) for roll-pitch-yaw error
            tolerance.  If *None* the defaults are used.

        Returns
        -------
        ndarray | None
            Joint angles *(radians)* if a solution is found, otherwise *None*.
        """
        position = np.asarray(position, dtype=float)
        orientation = np.asarray(orientation, dtype=float)

        if position.shape != (3,):
            raise ValueError("`position` must be shape (3,)")
        if orientation.shape != (4,):
            raise ValueError("`orientation` must be shape (4,)")

        if seed is None:
            # A nice generic seed is the midpoint between lower and upper limits.
            lower, upper = self.get_joint_limits()
            seed_arr = (lower + upper) / 2.0
        else:
            seed_arr = np.asarray(seed, dtype=float)
            if seed_arr.shape != (self.number_of_joints,):
                raise ValueError(
                    f"`seed` must be shape ({self.number_of_joints},) but is "
                    f"{seed_arr.shape}"
                )

        # bounds may be None; TRAC-IK handles that fine.
        result = self._ik.get_ik(
            seed_arr.tolist(),
            *position.tolist(),
            *orientation.tolist(),
            *(pos_bounds.tolist() if pos_bounds is not None else []),
            *(rot_bounds.tolist() if rot_bounds is not None else []),
        )

        if result is None:
            return None  # No IK solution within the timeout/accuracy.
        return np.array(result, dtype=float)