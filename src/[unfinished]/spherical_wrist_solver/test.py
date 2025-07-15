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

# The tool offset is implicitly handled by the C++ solver via M_HOME and d6.
# The Python test script does not need to manually apply it.

def fk(q):
    """Base→tool homogeneous transform."""
    return ik.fk(q)

# ------------------------------------------------------------------ #
#  Forward kinematics at home
# ------------------------------------------------------------------ #
T_home = fk(q_zero)
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
#  Verify Jacobian against finite differences
# ------------------------------------------------------------------ #
def numerical_jacobian(q, delta=1e-7):
    """Computes spatial Jacobian via finite differences."""
    J_num = np.zeros((6, len(q)))
    T_base = fk(q)

    for i in range(len(q)):
        q_plus = q.copy()
        q_plus[i] += delta
        T_plus = fk(q_plus)

        # The spatial twist V_s is defined by V_s_hat = T_dot * T_inv
        # We approximate T_dot with finite differences
        T_dot_approx = (T_plus - T_base) / delta
        V_hat_s = T_dot_approx @ np.linalg.inv(T_base)

        # Extract w and v from the se(3) matrix V_hat_s
        w_s = np.array([V_hat_s[2, 1], V_hat_s[0, 2], V_hat_s[1, 0]])
        v_s = V_hat_s[:3, 3]

        # The C++ solver uses (w, v) format for twists
        J_num[:, i] = np.hstack([w_s, v_s])

    return J_num

J_num_home = numerical_jacobian(q_zero)
J_err = np.linalg.norm(J_home - J_num_home)

print(f"\nNumerical Jacobian @ zero pose (error = {J_err:.2e}):\n", J_num_home)
if J_err < 1e-5:
    print("SUCCESS: Analytic Jacobian matches numerical approximation.")
else:
    print("FAIL: Analytic Jacobian is incorrect.")
    sys.exit(1)


# ------------------------------------------------------------------ #
#  Ask IK to reproduce that pose
# ------------------------------------------------------------------ #
# The C++ solver expects the target pose of the TOOL TIP. It calculates
# the required wrist center position internally.
sols = ik.solve(R_home, p_home)
print(f"IK returned {len(sols)} solution(s)")

best = None
best_err = 1e9
for s in sols:
    p = fk(s.q)[:3, 3]
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
#  Test FK at a non-zero configuration
# ------------------------------------------------------------------ #
print("\n" + "-"*50)
print("Testing FK for a q1=90deg rotation")

q_90 = np.radians([90, 0, 0, 0, 0, 0])
T_90 = fk(q_90)
p_90 = T_90[:3, 3]
R_90 = T_90[:3, :3]

print("FK @ q1=90deg   : p =", p_90)
# At q1=90, the X-coordinate of home pose should become the Y-coordinate,
# and Y should become -X. Z remains the same.
p_home_check = np.array([T_home[1, 3], -T_home[0, 3], T_home[2, 3]])
fk_p_err = np.linalg.norm(p_90 - p_home_check)
print(f"Expected p      : {p_home_check}")
print(f"FK position error: {fk_p_err:.2e}")

# ------------------------------------------------------------------ #
#  Test IK for that non-zero configuration
# ------------------------------------------------------------------ #
print("\nTesting IK for q1=90deg pose")
sols_90 = ik.solve(R_90, p_90)
print(f"IK returned {len(sols_90)} solution(s)")

best_90 = None
best_err_90 = 1e9
found_sol = False
for s in sols_90:
    p = fk(s.q)[:3, 3]
    err = np.linalg.norm(p - p_90)
    if err < best_err_90:
        best_err_90, best_90 = err, s.q

    q_deg = np.degrees(s.q)
    q_deg[np.abs(q_deg) < 1e-5] = 0.
    err_disp = err if err >= 1e-5 else 0.0
    q_str = ' '.join(f'{v:8.2f}' for v in q_deg)
    print(f" q[deg] = [ {q_str} ]  FK-err = {err_disp:.8f} m")

    # Check if we found a solution close to the original q_90
    q_err = np.linalg.norm(s.q - q_90)
    if q_err < 1e-6:
        found_sol = True

if found_sol and best_err_90 < 1e-6:
    print("\nSUCCESS: solver recovers q1=90deg pose to machine precision.")
else:
    print("\nFAIL: solver did not recover the q1=90deg pose.")


# ------------------------------------------------------------------ #
#  Second investigation point p1
# ------------------------------------------------------------------ #

# Define a second tool-frame target position while keeping the same
# orientation as the home pose. The new position is 20 cm in +Y and
# 20 cm in −Z relative to the home position, and 20 cm shorter in +X.

p1_tool = np.array([0.289006, 0.0, 0.1701])

print(f"p1_position = {p1_tool}")

# As before, call the IK solver with the desired TOOL TIP pose.
sols_p1 = ik.solve(R_home, p1_tool)
print(f"\nIK returned {len(sols_p1)} solution(s) for p1 target")

best1 = None
best_err1 = 1e9
for s in sols_p1:
    p = fk(s.q)[:3, 3]
    err = np.linalg.norm(p - p1_tool)
    if err < best_err1:
        best_err1, best = err, s.q

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
