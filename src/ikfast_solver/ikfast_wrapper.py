import numpy as np
# The 'ikfast_pybind' module is the C++ extension built by scikit-build-core.
# We use an absolute import to ensure we get the installed package.
from . import ikfast_pybind

class IKFastSolver:
    """
    A Python wrapper for the pybind11-based IKFast solver.
    
    This class provides a Pythonic interface to the high-performance C++ module.
    The C++ extension is built and installed automatically using the configuration
    in `pyproject.toml`.
    """
    def __init__(self):
        """
        Initializes the solver by linking to the C++ extension module.
        """
        self.num_joints = ikfast_pybind.get_num_joints()

    def solve_ik(self, translation, rotation_matrix, initial_joint_angles=None):
        """
        Calls the IK solver to find solutions for a given target pose.

        :param translation: A 3-element list or numpy array [x, y, z].
        :param rotation_matrix: A 9-element list or numpy array representing the
                                3x3 row-major rotation matrix.
        :param initial_joint_angles: An optional list or numpy array of the current
                                     joint angles. If provided, the closest solution
                                     is returned. If None, all solutions are returned.
        :return: A numpy array of the best solution, or all solutions, or None.
        """
        eetrans = np.array(translation, dtype=np.float64)
        eerot = np.array(rotation_matrix, dtype=np.float64)

        if initial_joint_angles is not None:
            initial_angles_np = np.array(initial_joint_angles, dtype=np.float64)
            return ikfast_pybind.solve_ik(eetrans, eerot, initial_angles_np)
        else:
            return ikfast_pybind.solve_ik(eetrans, eerot)

    def solve_ik_path(self, poses_batch, initial_joint_angles):
        """
        Solves an entire path of IK poses with a single call to the C++ library.

        :param poses_batch: A NumPy array of shape (N, 12) where N is the number of poses.
        :param initial_joint_angles: A NumPy array (num_joints,) for the starting configuration.
        :return: A NumPy array of shape (N, num_joints) with the solutions, or None if the path fails.
        """
        initial_angles_np = np.array(initial_joint_angles, dtype=np.float64)
        return ikfast_pybind.solve_ik_batch(poses_batch, initial_angles_np)

    def compute_fk(self, joint_angles):
        """
        Calculates forward kinematics for a given set of joint angles.

        :param joint_angles: A list or numpy array of joint angles.
        :return: A tuple containing the translation (np.array) and rotation matrix (np.array).
        """
        joint_angles_np = np.array(joint_angles, dtype=np.float64)
        return ikfast_pybind.compute_fk(joint_angles_np)

if __name__ == '__main__':
    # A simple example of how to use the IKFastSolver class
    print("--- Testing IKFastSolver Wrapper ---")
    try:
        solver = IKFastSolver()
        print(f"Kinematic chain has {solver.num_joints} joints.")

        # Define a target pose
        target_translation = [0.3, 0.0, 0.4]
        target_rotation = [1, 0, 0, 0, 1, 0, 0, 0, 1] # Identity matrix

        print(f"\nSolving for target pose:")
        print(f"  Translation: {target_translation}")
        print(f"  Rotation Matrix: {target_rotation}")
        
        solutions = solver.solve_ik(target_translation, target_rotation)

        if solutions is not None:
            print(f"\nFound {solutions.shape[0]} solutions:")
            for i, sol in enumerate(solutions):
                sol_deg = np.rad2deg(sol)
                print(f"  Solution {i + 1} (degrees): {np.round(sol_deg, 4)}")
            
            # --- Test FK on the first solution ---
            if solutions.size > 0:
                print("\n--- Testing FK on first solution ---")
                fk_trans, fk_rot = solver.compute_fk(solutions[0])
                fk_rot_matrix = np.array(fk_rot).reshape(3, 3)
                print(f"  FK Translation: {np.round(fk_trans, 4)}")
                print(f"  FK Rotation Matrix:\n{np.round(fk_rot_matrix, 4)}")

        else:
            print("\nNo IK solution found for the given pose.")

    except (RuntimeError, OSError) as e:
        print(f"\nAn error occurred: {e}") 