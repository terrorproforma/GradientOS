"""Numeric kinematics test script.

This script now loads Denavit–Hartenberg (DH) parameters directly from the
``mini-6dof-arm/dh_params.csv`` file that accompanies the repository.  To aid
readability and to make clear which column maps to which DH parameter, the
script prints out the header row found in the CSV before constructing the
``DH`` numpy array.  This ensures that anyone running the test can immediately
verify the order of the parameters without having to open the CSV file
manually.

Expected header order in the CSV (after the *joint* label column):

1. **a**     – Link length (metres)
2. **alpha** – Link twist (radians)
3. **d**     – Link offset (metres)
4. **theta** – Joint angle (radians, normally the variable)
"""

from pathlib import Path
import csv
import numpy as np
from scipy.spatial.transform import Rotation

np.set_printoptions(suppress=True, precision=6, floatmode='fixed')  # nicer print

from numeric_solver.numeric_k import MiniArmKinematics
from numeric_k import compute_ref_rot, user_to_ee_quat, ee_to_user_euler  # Import the new standalone functions

# ---------------------------------------------------------------------------
# Locate and load the CSV file containing the DH table
# ---------------------------------------------------------------------------

# Resolve the path to ``mini-6dof-arm/dh_params.csv`` relative to this file so
# that the script works no matter where it is launched from.

CSV_PATH = Path(__file__).resolve().parents[2] / "mini-6dof-arm" / "dh_params.csv"

# Read the header row to display the column order to the user.
with CSV_PATH.open(newline="") as csvfile:
    reader = csv.reader(csvfile)
    header = next(reader)  # First row contains column names

# The first column is a joint label; we discard it when building the numeric
# array, but we keep it in the printout so the user sees the full structure.
print("Loaded DH parameter CSV from:", CSV_PATH)
print("Column order in CSV:", header)

# Load the numeric data, skipping the header row.  We take only columns 1-4
# (indices start at 0) because column 0 contains the joint label strings.
D_H_NUMERIC_COLUMNS = (1, 2, 3, 4)
DH = np.genfromtxt(CSV_PATH, delimiter=",", skip_header=1, usecols=D_H_NUMERIC_COLUMNS, dtype=float)

# ---------------------------------------------------------------------------
# Instantiate the kinematics class and perform a quick FK / IK round-trip
# ---------------------------------------------------------------------------

kin = MiniArmKinematics.from_dh(DH, [0] * DH.shape[0])

# Compute the reference rotation matrix using the new standalone function
# This is done once after creating the kinematics instance to get the zero-pose orientation
ref_rot = compute_ref_rot(kin)
print("\nComputed Reference Rotation Matrix:\n", np.round(ref_rot, 6))

q = np.zeros(6)
T = kin.fk(q)

# Nicer FK matrix output (round tiny numerical noise to zero)
T_clean = np.where(np.abs(T) < 1e-10, 0, np.round(T, 6))
print("\n--- FK Test (q = all zeros) ---")
print("Full Transform Matrix (T):\n", T_clean)

# Extract and print world position and orientation
position = T_clean[:3, 3]
rotation_matrix = T_clean[:3, :3]
euler_angles = Rotation.from_matrix(rotation_matrix).as_euler('xyz', degrees=True)

# New: Convert the end effector rotation to user Euler using the standalone function
# This demonstrates the output conversion, which should give [0,0,0] for zero pose without gimbal lock issues
user_euler = ee_to_user_euler(rotation_matrix, ref_rot)
print("\nUser-Friendly Orientation (Roll, Pitch, Yaw degrees):", np.round(user_euler, 4))

print("\nWorld Coordinates at Zero-Angle Pose:")
print(f"  Position (X, Y, Z):      {position}")
print(f"  Orientation (Roll, Pitch, Yaw): {np.round(euler_angles, 4)} degrees")


print("\n--- IK Test ---")
# Test the input conversion function
# Define a user Euler input (here, [0,0,0] to match the zero pose and test round-trip)
# Detailed explanation: This converts user Euler to a quaternion for IK input.
# Step 1: Create input array with dtype=float.
# Step 2: Call user_to_ee_quat with ref_rot.
# Step 3: Use the resulting quat in IK to verify it solves correctly (should match zero angles).
user_euler_input = np.array([0.0, 0.0, 0.0], dtype=float)
print("\n--- Testing Input Conversion ---\nUser Euler Input (degrees):", user_euler_input)
quat_from_user = user_to_ee_quat(user_euler_input, ref_rot)
print("Converted Quaternion [x, y, z, w]:", np.round(quat_from_user, 6))

# Use the converted quaternion in IK (replaces hard-coded quat)
pos = np.array([0.4, 0.0, 0.25], dtype=float)
q_star, e_star, iters, reason = kin.ik(quat_from_user, pos, q)

# IK output clean
q_star_clean = np.where(np.abs(q_star) < 1e-10, 0, np.round(q_star, 6))
e_star_clean = np.where(np.abs(e_star) < 1e-10, 0, np.round(e_star, 6))
print("IK Target Position: ", pos)
print("IK Solution (Joint Angles):", q_star_clean)
print("IK Solution Error (norm):", np.linalg.norm(e_star_clean))
print(f"(Solver took {iters} iterations, reason: {reason})")
