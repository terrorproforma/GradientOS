#!/usr/bin/env python3
"""Simple command-line utility to sanity-check the forward and inverse
kinematics (FK / IK) functions provided by *src/ik_solver.py*.

This **does not** command any hardware – it only runs the mathematical
solver to verify that an IK solution fed through FK reproduces the
original tool-tip pose.

Usage examples
--------------
1) Run 10 random pose tests (default)
   $ python scripts/test_kinematics.py

2) Increase to 100 random tests with a fixed RNG seed
   $ python scripts/test_kinematics.py -n 100 --seed 42

3) Test a specific position with default orientation (identity)
   $ python scripts/test_kinematics.py --pos 0.3 0.1 0.25

The script prints the target pose, the IK joint angles found (in rad),
then the FK-computed pose and the Euclidean error.  A final summary
shows how many tests succeeded within tolerance.
"""

import argparse
import sys
import numpy as np
from pathlib import Path

# Make sure the repo root is on sys.path so that `import ik_solver` works
REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT / "src"))

from ik_solver import solve_ik, get_fk_matrix, NUM_JOINTS  # noqa: E402

# --------------------------- Helpers -----------------------------------------

def _rand_position(workspace_radius=0.5, z_min=0.05, z_max=0.45):
    """Generate a random reachable tool-tip position within a cylinder."""
    r = np.sqrt(np.random.uniform(0.0, workspace_radius ** 2))
    theta = np.random.uniform(0.0, 2.0 * np.pi)
    x = r * np.cos(theta)
    y = r * np.sin(theta)
    z = np.random.uniform(z_min, z_max)
    return np.array([x, y, z], dtype=float)


def _print_vec(label, vec, precision=4):
    arr = np.asarray(vec)
    if arr.ndim == 1:
        formatted = ", ".join(f"{float(v):.{precision}f}" for v in arr)
        print(f"{label}: [{formatted}]")
    else:
        print(f"{label} (multiple):")
        for row in arr:
            formatted = ", ".join(f"{float(v):.{precision}f}" for v in row)
            print(f"  [{formatted}]")

# --------------------------- Main routine ------------------------------------

def run_tests(args):
    # Prepare list of target poses (positions only; orientation = identity)
    if args.pos:
        targets = [np.array(args.pos, dtype=float)]
    else:
        targets = [_rand_position() for _ in range(args.num)]

    np.set_printoptions(precision=4, suppress=True)

    tol = args.tol
    failures = 0

    for idx, pos in enumerate(targets, start=1):
        print(f"\n=== Test {idx}/{len(targets)} ===")
        _print_vec("Target position", pos)

        ik_solution = solve_ik(pos, target_orientation_matrix=None, initial_joint_angles=None)

        if ik_solution is None:
            print("IK failed – no solution returned.")
            failures += 1
            continue

        _print_vec("IK joint angles (rad)", ik_solution)

        fk_matrix = get_fk_matrix(ik_solution)
        if fk_matrix is None:
            print("FK failed – no matrix returned.")
            failures += 1
            continue

        fk_pos = fk_matrix[:3, 3]
        _print_vec("FK position", fk_pos)

        err = np.linalg.norm(fk_pos - pos)
        print(f"Position error: {err:.6f} m")

        if err > tol:
            print(f"❌  Error exceeds tolerance ({tol} m)")
            failures += 1
        else:
            print("✅  Within tolerance")

    passed = len(targets) - failures
    print("\n================ Summary ================")
    print(f"Tests run       : {len(targets)}")
    print(f"Passed          : {passed}")
    print(f"Failed          : {failures}")
    if failures == 0:
        print("\nAll tests passed! 🎉")
    else:
        sys.exit(1)

# --------------------------- CLI --------------------------------------------

def main():
    parser = argparse.ArgumentParser(
        description="Mini-Arm kinematics utility – run random validation tests, or query FK / IK directly.")

    # Mutually-exclusive high-level modes
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--fk", nargs=NUM_JOINTS, type=float,
                      metavar=("J1", "J2", "J3", "J4", "J5", "J6"),
                      help="Compute forward kinematics for the provided joint angles (rad).")

    mode.add_argument("--ik-pos", nargs=3, type=float, metavar=("X", "Y", "Z"),
                      help="Target tool-tip position (meters) for inverse kinematics query.")

    # IK orientation – only meaningful with --ik-pos
    parser.add_argument("--ik-orient", nargs='+', type=float,
                        metavar='R',
                        help="Target orientation. Provide 9 numbers (row-major rotation matrix),\n"
                             "4 numbers (xyzw quaternion), or 3 numbers (roll pitch yaw rad).\n"
                             "Defaults to identity if omitted.")

    # Random test options (only if neither FK nor IK explicit query is given)
    parser.add_argument("-n", "--num", type=int, default=10,
                        help="Number of random tests to run (ignored for explicit FK/IK query).")
    parser.add_argument("--seed", type=int, help="RNG seed for reproducibility (random tests).")
    parser.add_argument("--tol", type=float, default=1e-3,
                        help="Acceptable position error when validating random tests (m).")

    args = parser.parse_args()

    # FK query ----------------------------------------------------------
    if args.fk is not None:
        joint_angles = np.array(args.fk, dtype=float)
        if joint_angles.size != NUM_JOINTS:
            print(f"Expected {NUM_JOINTS} joint values but got {joint_angles.size}.")
            sys.exit(1)

        fk_mat = get_fk_matrix(joint_angles)
        if fk_mat is None:
            print("FK solver failed.")
            sys.exit(1)

        pos = fk_mat[:3, 3]
        rot = fk_mat[:3, :3]
        _print_vec("Position", pos)
        print("Rotation matrix:\n", np.array2string(rot, formatter={'float': lambda x: f"{x: .4f}"}))
        return

    # IK query ----------------------------------------------------------
    if args.ik_pos is not None:
        position = np.array(args.ik_pos, dtype=float)

        # Parse orientation if any
        orient_input = args.ik_orient
        if orient_input is None:
            orientation_matrix = np.identity(3)
        else:
            orient_arr = np.asarray(orient_input, dtype=float)
            if orient_arr.size == 9:
                orientation_matrix = orient_arr.reshape(3, 3)
            elif orient_arr.size == 4:  # quaternion xyzw
                from scipy.spatial.transform import Rotation as R
                orientation_matrix = R.from_quat(orient_arr).as_matrix()
            elif orient_arr.size == 3:  # RPY
                orientation_matrix = R.from_euler('xyz', orient_arr).as_matrix()
            else:
                print("--ik-orient expects 3 (rpy), 4 (quat xyzw) or 9 (matrix) numbers.")
                sys.exit(1)

        ik_sol_all = solve_ik(position, target_orientation_matrix=orientation_matrix, initial_joint_angles=None)
        if ik_sol_all is None:
            print("IK solver failed – no solution.")
            sys.exit(1)

        # Handle multiple solutions – pick the first, but display count
        if np.asarray(ik_sol_all).ndim > 1:
            print(f"Solver returned {len(ik_sol_all)} solutions; showing the first one.")
            ik_sol = ik_sol_all[0]
        else:
            ik_sol = ik_sol_all

        _print_vec("IK joint angles (rad)", ik_sol)
        return

    # Otherwise run random validation tests ----------------------------
    if args.seed is not None:
        np.random.seed(args.seed)

    run_tests(args)


if __name__ == "__main__":
    main() 