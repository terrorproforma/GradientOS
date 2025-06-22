import unittest
import sys
import os
import math
import numpy as np

# Add the 'src' directory to the Python path to allow importing the arm_controller package
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'src')))

# Mock the scipy savgol_filter before importing the module that uses it
from unittest.mock import MagicMock
sys.modules['scipy.signal'] = MagicMock()

from arm_controller import trajectory_execution
from arm_controller import utils

class TestTrajectoryPlanning(unittest.TestCase):
    """
    Unit tests for the trajectory planning functions.
    """

    @unittest.mock.patch('ik_solver.solve_ik_path_batch')
    def test_path_unwrapping_and_smoothing(self, mock_solve_ik: unittest.mock.Mock) -> None:
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

        # Define other inputs for the planner
        start_q = [0.0] * 6
        cartesian_points = [[0,0,0]] * 4 # Dummy points, as IK is mocked

        # 2. Call the planner
        # We disable smoothing here to isolate the unwrapping logic first
        unwrapped_path = trajectory_execution._plan_high_fidelity_trajectory(
            cartesian_points, start_q, use_smoothing=False
        )
        
        # 3. Assert that the unwrapping was successful
        # The value after 3.1 should be approx 3.18 (i.e., -3.1 + 2*pi)
        self.assertAlmostEqual(unwrapped_path[2][5], -3.1 + (2 * math.pi), places=2)

        # 4. Test with smoothing enabled
        # We need to re-mock the savgol_filter as it's a module-level mock
        with unittest.mock.patch('scipy.signal.savgol_filter') as mock_savgol:
            # Set the filter to just return the input to isolate our logic
            mock_savgol.side_effect = lambda x, w, p, axis: x

            smoothed_path = trajectory_execution._plan_high_fidelity_trajectory(
                cartesian_points, start_q, use_smoothing=True
            )
            # Check that the filter was called
            mock_savgol.assert_called_once()
            # Check that the unwrapping still happened before smoothing
            self.assertAlmostEqual(smoothed_path[2][5], -3.1 + (2 * math.pi), places=2)


if __name__ == '__main__':
    unittest.main() 