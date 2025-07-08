#!/usr/bin/env python3
"""Minimal sanity‑check for the analytic POE solver.

Build C++ with CMake first so that the pybind11 module
`spherical_ik_py` (or `spherical_ik`) is importable via PYTHONPATH.
"""

import numpy as np
import math
import sys

# ------------------------------------------------------------------ #
#  Load the module (depends on your CMake target name)
# ------------------------------------------------------------------ #
try:
    import spherical_ik_py as sik
except ModuleNotFoundError:
    import spherical_ik as sik           # fallback target name

ik = sik.SphericalIK()

# ------------------------------------------------------------------ #
#  Zero pose and a tool offset
# ------------------------------------------------------------------ #



q_zero = np.radians([0, 0, 0, 0, 0, 0])
tool_offset = np.array([0.154, 0.0, 0.0])    # 154 mm along tool +X

def fk_with_tool(q):
    """Base→tool homogeneous transform."""
    T06 = ik.fk(q)
    T_tool = np.eye(4)
    T_tool[:3, 3] = tool_offset
    return T06 @ T_tool

# ------------------------------------------------------------------ #
#  Forward kinematics at home
# ------------------------------------------------------------------ #
T_home = fk_with_tool(q_zero)
p_home = T_home[:3, 3]
R_home = T_home[:3, :3]
print("FK @ zero pose  : p =", p_home)

# ------------------------------------------------------------------ #
#  Spatial Jacobian at the zero pose
# ------------------------------------------------------------------ #
# The Jacobian maps joint-rate vectors \(\dot{q}\) to spatial twist of
# the tool frame expressed in the base frame. It is useful for
# sensitivity analysis and control. Here we simply compute it at the
# zero configuration and display the result for inspection.

J_home = ik.jacobian(q_zero)
np.set_printoptions(precision=5, suppress=True)
print("Jacobian @ zero pose:\n", J_home)

# ------------------------------------------------------------------ #
#  Ask IK to reproduce that pose
# ------------------------------------------------------------------ #
# Solver expects the wrist origin, so subtract the tool offset
p_wrist = p_home - R_home @ tool_offset
sols = ik.solve(R_home, p_wrist)
print(f"IK returned {len(sols)} solution(s)")

best = None
best_err = 1e9
for s in sols:
    p = fk_with_tool(s.q)[:3, 3]
    err = np.linalg.norm(p - p_home)
    if err < best_err:
        best_err, best = err, s.q

    q_deg = np.degrees(s.q)
    q_deg[np.abs(q_deg) < 1e-5] = 0.
    
    err_disp = err if err >= 1e-5 else 0.0
    
    q_str = ' '.join(f'{v:8.2f}' for v in q_deg)
    print(f" q[deg] = [ {q_str} ]  FK‑err = {err_disp:.8f} m")

if best_err < 1e-6:
    print("\nSUCCESS: solver recovers zero pose to machine precision.")
    # sys.exit(0)
else:
    print("\nFAIL: something is still off.")

# ------------------------------------------------------------------ #
#  Second investigation point p1
# ------------------------------------------------------------------ #

# Define a second tool-frame target position while keeping the same
# orientation as the home pose. The new position is 20 cm in +Y and
# 20 cm in −Z relative to the home position, and 20 cm shorter in +X.

p1_tool = np.array([0.231706, 0.0, 0.2128])

print(f"p1_position = {p1_tool}")

# As before, convert to wrist centre coordinates expected by the IK
# solver by subtracting the tool offset rotated into the target frame.

p1_wrist = p1_tool - R_home @ tool_offset

sols_p1 = ik.solve(R_home, p1_wrist)
print(f"\nIK returned {len(sols_p1)} solution(s) for p1 target")

best1 = None
best_err1 = 1e9
for s in sols_p1:
    p = fk_with_tool(s.q)[:3, 3]
    err = np.linalg.norm(p - p1_tool)
    if err < best_err1:
        best_err1, best1 = err, s.q

    q_deg = np.degrees(s.q)
    q_deg[np.abs(q_deg) < 1e-5] = 0.

    err_disp = err if err >= 1e-5 else 0.0

    q_str = ' '.join(f'{v:8.2f}' for v in q_deg)
    print(f" q[deg] = [ {q_str} ]  FK-err = {err_disp:.8f} m")

if best_err1 < 1e-4:
    print("\nSUCCESS: solver reaches p1 within 0.1 mm.")
    # sys.exit(0)
else:
    print("\nFAIL: p1 target not reached with sufficient accuracy.")
    sys.exit(1)
