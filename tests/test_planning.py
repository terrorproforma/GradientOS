import unittest
import sys
import os
import math
import numpy as np

# Mock the scipy savgol_filter before importing the module that uses it
from unittest.mock import MagicMock
sys.modules['scipy.signal'] = MagicMock()

from gradient_os.arm_controller import trajectory_execution
from gradient_os.arm_controller import utils

class TestTrajectoryPlanning(unittest.TestCase):
    """
    Unit tests for the trajectory planning functions.
    """

    @classmethod
    def setUpClass(cls) -> None:
        # Planner unit tests run without full controller bootstrap, so ensure
        # joint metadata is present for unwrap/smoothing checks.
        if utils.NUM_LOGICAL_JOINTS is None:
            utils.NUM_LOGICAL_JOINTS = 6
        if utils.LOGICAL_JOINT_LIMITS_RAD is None:
            utils.LOGICAL_JOINT_LIMITS_RAD = [(-math.pi, math.pi)] * int(utils.NUM_LOGICAL_JOINTS)

    @unittest.mock.patch('gradient_os.arm_controller.trajectory_execution._validate_joint_trajectory_gates')
    @unittest.mock.patch('gradient_os.ik_solver.solve_ik_path_batch')
    def test_path_unwrapping_and_smoothing(
        self,
        mock_solve_ik: unittest.mock.Mock,
        mock_validate: unittest.mock.Mock,
    ) -> None:
        """
        Tests that the planning pipeline correctly unwraps and smooths a raw
        trajectory that contains a 2*pi jump.
        """
        # 1. Define a raw trajectory with a wrap-around on the last joint
        # from +3.1 to -3.1
        raw_path = [
            [0.0, 0.0, 0.0, 0.0, 0.0, 3.0],
            [0.1, 0.1, 0.1, 0.1, 0.1, 3.1],
            [0.2, 0.2, 0.2, 0.2, 0.2, -3.1], # Jump occurs here
            [0.3, 0.3, 0.3, 0.3, 0.3, -3.0],
        ]
        mock_solve_ik.return_value = raw_path
        mock_validate.return_value = (True, "OK", {})

        # Define other inputs for the planner
        start_q = [0.0] * 6
        cartesian_points = [[0,0,0]] * 4 # Dummy points, as IK is mocked

        # 2. Call the planner
        # We disable smoothing here to isolate the unwrapping logic first
        unwrapped_path = trajectory_execution._plan_high_fidelity_trajectory(
            cartesian_points, start_q, use_smoothing=False
        )
        
        # 3. Assert that the planner respects joint limits when considering unwraps
        # The value should remain within the configured bounds rather than exceeding them.
        self.assertAlmostEqual(unwrapped_path[2][5], -3.1, places=2)

        # 4. Test with smoothing enabled — the path should still be generated even
        # when the filter is skipped due to insufficient samples.
        smoothed_path = trajectory_execution._plan_high_fidelity_trajectory(
            cartesian_points, start_q, use_smoothing=True
        )
        self.assertEqual(len(smoothed_path), len(raw_path))
        self.assertAlmostEqual(smoothed_path[2][5], -3.1, places=2)

    @unittest.mock.patch('gradient_os.arm_controller.trajectory_execution._validate_joint_trajectory_gates')
    @unittest.mock.patch('gradient_os.ik_solver.solve_ik_path_batch')
    def test_smoothing_applied_on_long_path(
        self,
        mock_solve_ik: unittest.mock.Mock,
        mock_validate: unittest.mock.Mock,
    ) -> None:
        """
        Verifies that the Savitzky-Golay filter is invoked when the trajectory is long enough.
        """
        # Construct a long raw trajectory ( > default window_length ) with simple ramp data.
        raw_path = [[i * 0.01] * utils.NUM_LOGICAL_JOINTS for i in range(30)]
        mock_solve_ik.return_value = raw_path
        mock_validate.return_value = (True, "OK", {})

        start_q = [0.0] * utils.NUM_LOGICAL_JOINTS
        cartesian_points = [[0, 0, 0]] * len(raw_path)

        with unittest.mock.patch('gradient_os.arm_controller.trajectory_execution.savgol_filter', wraps=lambda arr, window_length, polyorder, axis: arr * 0.9) as mock_filter:
            result = trajectory_execution._plan_high_fidelity_trajectory(
                cartesian_points, start_q, use_smoothing=True
            )
            mock_filter.assert_called()
            self.assertEqual(len(result), len(raw_path))
            # Confirm smoothing altered the values (our wrapped lambda scales by 0.9)
            self.assertNotEqual(result[-1][0], raw_path[-1][0])

    @unittest.mock.patch('gradient_os.ik_solver.get_fk_matrix')
    def test_validate_gates_emits_json_safe_residuals(self, mock_fk_matrix: unittest.mock.Mock) -> None:
        """
        Ensure gate diagnostics never emit NaN/Inf values that break JSON responses.
        """
        fk = np.eye(4, dtype=float)
        mock_fk_matrix.return_value = fk
        original_limits = utils.LOGICAL_JOINT_LIMITS_RAD
        try:
            utils.LOGICAL_JOINT_LIMITS_RAD = None
            ok, reason, residuals = trajectory_execution._validate_joint_trajectory_gates(
                joint_trajectory=[[0.0] * 6, [0.01] * 6],
                cartesian_points=[[0.0, 0.0, 0.0], [0.0, 0.0, 0.0]],
                orientations_list=[np.eye(3, dtype=float), np.eye(3, dtype=float)],
            )
            self.assertTrue(ok)
            self.assertEqual(reason, "OK")
            self.assertIn("joint_limit_margin_rad", residuals)
            self.assertIsNone(residuals["joint_limit_margin_rad"])
        finally:
            utils.LOGICAL_JOINT_LIMITS_RAD = original_limits


if __name__ == '__main__':
    unittest.main() 
