"""numeric_k.py
A high-level façade around the underlying C++ QuIK solver (exposed via
`pyquik`).  The public API intentionally mirrors the expectations of the
rest of the Mini-Arm codebase: you feed it a URDF file (or directly a DH
table) and then call `fk()` / `ik()`.

This keeps 100 % of the heavy lifting inside the highly-optimised C++
code while presenting a clean, dependency-free (no ROS) Python surface.
"""
from __future__ import annotations

from pathlib import Path
from typing import Tuple, Sequence, Optional

import numpy as np

# Local C++ bindings ---------------------------------------------------------
try:
    import pyquik  # type: ignore
except ModuleNotFoundError as exc:
    # Attempt to load the compiled shared object from the local build dir.
    import importlib.util as _ilu
    import sys as _sys
    from pathlib import Path as _P

    root_dir = _P(__file__).resolve().parent / "pyquik"
    for so_file in root_dir.rglob("pyquik*.so"):
        spec = _ilu.spec_from_file_location("pyquik", so_file)
        if spec and spec.loader:
            pyquik = _ilu.module_from_spec(spec)  # type: ignore
            _sys.modules["pyquik"] = pyquik
            spec.loader.exec_module(pyquik)  # type: ignore[arg-type]
            break
else:
    # If import succeeded but does not expose expected symbols, we likely
    # imported the stub package from QuIK (src/numeric_solver/quik/python), not
    # our compiled extension. Replace it with the compiled one.
    if not hasattr(pyquik, "Robot"):
        import importlib.util as _ilu
        import sys as _sys
        from pathlib import Path as _P

        _pkg_path = _P(__file__).resolve().parent / "pyquik"
        for _file in _pkg_path.rglob("pyquik*.so"):
            _spec = _ilu.spec_from_file_location("pyquik", _file)
            if _spec and _spec.loader:
                module = _ilu.module_from_spec(_spec)
                _spec.loader.exec_module(module)  # type: ignore[arg-type]
                _sys.modules["pyquik"] = module
                pyquik = module  # type: ignore
                break

ArrayF = np.ndarray  # shorthand


# ---------------------------------------------------------------------------
# Utility helpers
# ---------------------------------------------------------------------------

def _urdf_to_dh(urdf_path: Path) -> Tuple[np.ndarray, Sequence[int]]:  # noqa: D401
    """VERY naive URDF→DH converter.

    For now this is **placeholder** logic – it merely raises to signal to the
    caller that an implementation is still needed.  The QuIK back-end uses
    the classical Denavit-Hartenberg parameters, so you need to supply a
    *DH table* *(dof×4)* and the list of *link types* (0=revolute,
    1=prismatic).
    """
    raise NotImplementedError("URDF to DH conversion has not been implemented yet")


# ---------------------------------------------------------------------------
# Main façade
# ---------------------------------------------------------------------------

class MiniArmKinematics:
    """High-level wrapper around QuIK exposing FK / IK in numpy‐land."""

    def __init__(
        self,
        *,
        dh: Optional[ArrayF] = None,
        link_types: Optional[Sequence[int]] = None,
        urdf_path: Optional[str | Path] = None,
        q_sign: Optional[ArrayF] = None,
        t_base: Optional[ArrayF] = None,
        t_tool: Optional[ArrayF] = None,
        solver_kwargs: Optional[dict] = None,
    ) -> None:
        """Create the robot model.

        Parameters
        ----------
        dh
            *(dof×4)* array of DH parameters `[a, α, d, θ]` in **radians** and
            **metres**.
        link_types
            Sequence with `0` for revolute, `1` for prismatic links.
        urdf_path
            If provided, `dh`/`link_types` are ignored and derived from the
            URDF instead (not yet implemented, see `_urdf_to_dh`).
        q_sign
            Optional sign adjustments (`±1`) for each joint.  Defaults to all
            `+1`.
        t_base, t_tool
            Optional `4×4` homogeneous transforms for base and tool frames.
            Default to identity.
        solver_kwargs
            Extra settings forwarded verbatim to `pyquik.IKSolver` – you rarely
            need to tweak these.
        """
        if urdf_path is not None:
            dh, link_types = _urdf_to_dh(Path(urdf_path))

        if dh is None or link_types is None:
            raise ValueError("Either (dh & link_types) *or* urdf_path must be provided.")

        dh = np.asarray(dh, dtype=float)
        if dh.ndim != 2 or dh.shape[1] != 4:
            raise ValueError("DH array must be (dof×4)")

        if len(link_types) != dh.shape[0]:
            raise ValueError("link_types length must match number of DH rows")

        q_sign = np.asarray(q_sign, dtype=float) if q_sign is not None else np.ones(dh.shape[0], float)
        if q_sign.size != dh.shape[0]:
            raise ValueError("q_sign length must match number of DH rows")

        t_base = np.asarray(t_base, dtype=float) if t_base is not None else np.eye(4, dtype=float)
        t_tool = np.asarray(t_tool, dtype=float) if t_tool is not None else np.eye(4, dtype=float)

        # Build the underlying C++ objects ----------------------------------
        self._robot = pyquik.Robot(dh, list(link_types), q_sign, t_base, t_tool)
        solver_kwargs = solver_kwargs or {}
        self._solver = pyquik.IKSolver(self._robot, **solver_kwargs)  # type: ignore[arg-type]

    # ---------------------------------------------------------------------
    # Public API
    # ---------------------------------------------------------------------

    @property
    def dof(self) -> int:
        return int(self._robot.dof)

    def fk(self, q: ArrayF) -> ArrayF:
        """Return the *4×4* tool transform for given joint configuration."""
        q = np.asarray(q, dtype=float)
        if q.size != self.dof:
            raise ValueError(f"FK expects {self.dof} joint values, got {q.size}")
        return self._robot.FK(q)  # bottomRows(4) already returned by binding

    def ik(
        self,
        quat: ArrayF,
        pos: ArrayF,
        q0: Optional[ArrayF] = None,
    ) -> Tuple[ArrayF, ArrayF, int, int]:
        """Solve inverse kinematics.

        Parameters
        ----------
        quat
            Quaternion `[x, y, z, w]`.
        pos
            Position `[x, y, z]` (metres).
        q0
            Initial joint guess.  Defaults to zeros.

        Returns
        -------
        q_star
            Solved joint positions *(dof,)*.
        e_star
            Residual twist `(6,)`.
        iterations, break_reason
            Metadata directly from QuIK.
        """
        quat = np.asarray(quat, dtype=float)
        pos = np.asarray(pos, dtype=float)
        if quat.shape != (4,):
            raise ValueError("quat must be shape (4,)")
        if pos.shape != (3,):
            raise ValueError("pos must be shape (3,)")
        if q0 is None:
            q0 = np.zeros(self.dof, float)
        q0 = np.asarray(q0, dtype=float)
        if q0.shape != (self.dof,):
            raise ValueError("q0 must have shape (dof,)")

        q_star, e_star, iters, reason = self._solver.solve(quat, pos, q0)
        return np.asarray(q_star), np.asarray(e_star), int(iters), int(reason)

    # ------------------------------------------------------------------
    # Convenience constructors
    # ------------------------------------------------------------------

    @classmethod
    def from_dh(
        cls,
        dh: ArrayF,
        link_types: Sequence[int],
        **kwargs,
    ) -> "MiniArmKinematics":
        return cls(dh=dh, link_types=link_types, **kwargs)
