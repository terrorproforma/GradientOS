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

    def test_unwrap_recenters_equivalent_branch_away_from_limit(self) -> None:
        original_limits = utils.LOGICAL_JOINT_LIMITS_RAD
        try:
            utils.LOGICAL_JOINT_LIMITS_RAD = [(-6.3, 6.3)] + [(-math.pi, math.pi)] * 5
            unwrapped = trajectory_execution._unwrap_joint_trajectory(
                [
                    [6.18, 0.0, 0.0, 0.0, 0.0, 0.0],
                    [6.22, 0.0, 0.0, 0.0, 0.0, 0.0],
                    [6.26, 0.0, 0.0, 0.0, 0.0, 0.0],
                ],
                start_reference=[0.0] * 6,
            )
            self.assertLess(unwrapped[-1][0], 0.0)
            self.assertAlmostEqual(unwrapped[-1][0], 6.26 - (2.0 * math.pi), places=3)
        finally:
            utils.LOGICAL_JOINT_LIMITS_RAD = original_limits

    def test_unwrap_recentering_preserves_start_reference_branch(self) -> None:
        original_limits = utils.LOGICAL_JOINT_LIMITS_RAD
        try:
            utils.LOGICAL_JOINT_LIMITS_RAD = [(-6.3, 6.3)] + [(-math.pi, math.pi)] * 5
            unwrapped = trajectory_execution._unwrap_joint_trajectory(
                [
                    [-3.52, 0.0, 0.0, 0.0, 0.0, 0.0],
                    [-3.48, 0.0, 0.0, 0.0, 0.0, 0.0],
                    [-3.44, 0.0, 0.0, 0.0, 0.0, 0.0],
                ],
                start_reference=[-3.52] + [0.0] * 5,
            )
            self.assertAlmostEqual(unwrapped[0][0], -3.52, places=3)
            self.assertAlmostEqual(unwrapped[-1][0], -3.44, places=3)
        finally:
            utils.LOGICAL_JOINT_LIMITS_RAD = original_limits

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

    @unittest.mock.patch('gradient_os.ik_solver.get_fk_matrix')
    def test_wrap_capable_joint_uses_narrower_limit_margin(
        self,
        mock_fk_matrix: unittest.mock.Mock,
    ) -> None:
        fk = np.eye(4, dtype=float)
        mock_fk_matrix.return_value = fk
        original_limits = utils.LOGICAL_JOINT_LIMITS_RAD
        try:
            utils.LOGICAL_JOINT_LIMITS_RAD = [(-math.pi, math.pi)] * 3 + [(-6.3, 6.3)] + [(-math.pi, math.pi)] * 2
            ok, reason, residuals = trajectory_execution._validate_joint_trajectory_gates(
                joint_trajectory=[[0.0, 0.0, 0.0, 6.27004, 0.0, 0.0]],
                cartesian_points=[[0.0, 0.0, 0.0]],
                orientations_list=[np.eye(3, dtype=float)],
            )
            self.assertTrue(ok)
            self.assertEqual(reason, "OK")
            self.assertAlmostEqual(residuals["joint_limit_margin_rad"], 6.3 - 6.27004, places=5)
        finally:
            utils.LOGICAL_JOINT_LIMITS_RAD = original_limits

    @unittest.mock.patch('gradient_os.ik_solver.get_fk_matrix')
    def test_validate_gates_reports_failing_jump_joint_details(
        self,
        mock_fk_matrix: unittest.mock.Mock,
    ) -> None:
        fk = np.eye(4, dtype=float)
        mock_fk_matrix.return_value = fk
        ok, reason, residuals = trajectory_execution._validate_joint_trajectory_gates(
            joint_trajectory=[[0.0, 0.0, 0.0, 0.0, 0.0, (2.0 * math.pi) - 1e-4]],
            cartesian_points=[[0.0, 0.0, 0.0]],
            orientations_list=[np.eye(3, dtype=float)],
            start_q=[0.0] * 6,
            jump_context="dense_sequential:post_unwrap",
        )
        self.assertFalse(ok)
        self.assertEqual(reason, "IK_JUMP_REJECTED")
        self.assertEqual(residuals["jump_joint_index"], 6.0)
        self.assertEqual(residuals["jump_context"], "dense_sequential:post_unwrap")
        self.assertAlmostEqual(residuals["jump_joint_raw_step_rad"], (2.0 * math.pi) - 1e-4, places=4)
        self.assertAlmostEqual(residuals["jump_joint_wrapped_step_rad"], 1e-4, places=3)

    @unittest.mock.patch('gradient_os.arm_controller.trajectory_execution._validate_joint_trajectory_gates')
    @unittest.mock.patch('gradient_os.arm_controller.trajectory_execution._solve_ik_path')
    def test_high_fidelity_trajectory_recovers_jump_with_local_reseed(
        self,
        mock_solve_path: unittest.mock.Mock,
        mock_validate: unittest.mock.Mock,
    ) -> None:
        """
        If a solved path is rejected for a late jump, the planner should retry the
        failing suffix sequentially from the last accepted joint sample.
        """
        jump_residuals = {
            "jump_pose_index": 1.0,
            "step_source": "trajectory",
            "max_joint_step_rad": 1.8,
        }
        initial_candidate = [
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            [1.8, 1.8, 1.8, 1.8, 1.8, 1.8],
            [1.9, 1.9, 1.9, 1.9, 1.9, 1.9],
        ]
        recovered_suffix = [
            [0.12, 0.12, 0.12, 0.12, 0.12, 0.12],
            [0.24, 0.24, 0.24, 0.24, 0.24, 0.24],
        ]
        mock_solve_path.side_effect = [initial_candidate, recovered_suffix]

        def validate_side_effect(
            joint_trajectory,
            cartesian_points,
            orientations_list,
            start_q=None,
            jump_context=None,
        ):
            if len(joint_trajectory) == 3 and joint_trajectory[1][0] > 1.0:
                return False, "IK_JUMP_REJECTED", jump_residuals
            return True, "OK", {"max_joint_step_rad": 0.24}

        mock_validate.side_effect = validate_side_effect

        result = trajectory_execution._plan_high_fidelity_trajectory(
            cartesian_points=[[0.0, 0.0, 0.0], [0.05, 0.0, 0.0], [0.10, 0.0, 0.0]],
            start_q=[0.0] * 6,
            use_smoothing=False,
            orientations_list=[np.eye(3, dtype=float)] * 3,
        )

        self.assertIsNotNone(result)
        self.assertAlmostEqual(result[1][0], 0.12, places=3)
        self.assertAlmostEqual(result[2][0], 0.24, places=3)
        planner_diag = utils.trajectory_state.get("last_planner_diagnostics", {})
        self.assertEqual(planner_diag.get("reason_code"), "OK")
        self.assertEqual(planner_diag.get("seed_used"), "jump_recovery_seed")
        self.assertTrue(planner_diag.get("recovery", {}).get("used"))
        self.assertEqual(planner_diag.get("recovery", {}).get("strategy"), "local_suffix_reseed")

    @unittest.mock.patch('gradient_os.arm_controller.trajectory_execution._validate_joint_trajectory_gates')
    @unittest.mock.patch('gradient_os.arm_controller.trajectory_execution._solve_ik_path')
    def test_high_fidelity_trajectory_uses_branch_anchor_recovery_for_repeated_pose(
        self,
        mock_solve_path: unittest.mock.Mock,
        mock_validate: unittest.mock.Mock,
    ) -> None:
        jump_residuals = {
            "jump_pose_index": 1.0,
            "step_source": "trajectory",
            "max_joint_step_rad": 2.9,
            "jump_joint_index": 1.0,
            "jump_joint_previous_rad": 0.0,
            "jump_joint_current_rad": -2.9,
            "jump_joint_raw_step_rad": 2.9,
            "jump_joint_wrapped_step_rad": 2.9,
        }
        initial_candidate = [
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            [-2.9, 0.0, 0.0, 0.0, 0.0, 0.0],
            [-2.8, 0.0, 0.0, 0.0, 0.0, 0.0],
        ]
        failed_suffix = [
            [-2.9, 0.0, 0.0, 0.0, 0.0, 0.0],
            [-2.8, 0.0, 0.0, 0.0, 0.0, 0.0],
        ]
        recovered_suffix = [
            [0.12, 0.0, 0.0, 0.0, 0.0, 0.0],
            [0.24, 0.0, 0.0, 0.0, 0.0, 0.0],
        ]
        mock_solve_path.side_effect = [
            initial_candidate,
            failed_suffix,
            failed_suffix,
            recovered_suffix,
        ]

        def validate_side_effect(
            joint_trajectory,
            cartesian_points,
            orientations_list,
            start_q=None,
            jump_context=None,
        ):
            if any(abs(sample[0]) > 1.0 for sample in joint_trajectory):
                return False, "IK_JUMP_REJECTED", dict(jump_residuals, jump_context=jump_context)
            return True, "OK", {"max_joint_step_rad": 0.24}

        mock_validate.side_effect = validate_side_effect

        result = trajectory_execution._plan_high_fidelity_trajectory(
            cartesian_points=[[0.0, 0.0, 0.0], [0.05, 0.0, 0.0], [0.10, 0.0, 0.0]],
            start_q=[0.0] * 6,
            use_smoothing=False,
            orientations_list=[np.eye(3, dtype=float)] * 3,
            branch_anchor_q=[0.0] * 6,
        )

        self.assertIsNotNone(result)
        self.assertAlmostEqual(result[1][0], 0.12, places=3)
        self.assertAlmostEqual(result[2][0], 0.24, places=3)
        planner_diag = utils.trajectory_state.get("last_planner_diagnostics", {})
        self.assertTrue(planner_diag.get("branch_anchor_available"))
        self.assertEqual(planner_diag.get("seed_used"), "jump_recovery_seed")
        self.assertEqual(
            planner_diag.get("recovery", {}).get("strategy"),
            "branch_anchor_suffix_reseed",
        )

    @unittest.mock.patch('gradient_os.arm_controller.trajectory_execution._plan_high_fidelity_trajectory')
    @unittest.mock.patch('gradient_os.arm_controller.trajectory_execution.trajectory_planner.generate_trapezoidal_profile')
    @unittest.mock.patch('gradient_os.ik_solver.get_fk')
    def test_linear_move_splits_jump_rejected_segment(
        self,
        mock_get_fk: unittest.mock.Mock,
        mock_generate_profile: unittest.mock.Mock,
        mock_plan_high_fidelity: unittest.mock.Mock,
    ) -> None:
        """
        If the first high-fidelity solve fails with IK_JUMP_REJECTED, the planner
        should retry the single linear move as smaller seeded subsegments.
        """
        previous_diag = utils.trajectory_state.get("last_planner_diagnostics")
        mock_get_fk.side_effect = lambda q: np.array(list(q[:3]), dtype=float)
        mock_generate_profile.side_effect = (
            lambda start, target, velocity, acceleration, frequency: [
                np.asarray(start, dtype=float).tolist(),
                np.asarray(target, dtype=float).tolist(),
            ]
        )
        call_counter = {"count": 0}

        def plan_high_fidelity_side_effect(*args, **kwargs):
            call_counter["count"] += 1
            if call_counter["count"] == 1:
                utils.trajectory_state["last_planner_diagnostics"] = {
                    "reason_code": "IK_JUMP_REJECTED",
                    "attempt": "dense_sequential",
                    "fallback_level": 3,
                    "residuals": {
                        "jump_pose_index": 3.0,
                        "step_source": "trajectory",
                        "max_joint_step_rad": 1.1,
                    },
                }
                return None
            cartesian_points = kwargs.get("cartesian_points", args[0] if args else [])
            start_q = kwargs.get("start_q", args[1] if len(args) > 1 else [0.0] * 6)
            target_point = cartesian_points[-1] if cartesian_points else [0.0, 0.0, 0.0]
            segment_end = float(target_point[0])
            utils.trajectory_state["last_planner_diagnostics"] = {
                "reason_code": "OK",
                "attempt": "sequential",
                "fallback_level": 1,
                "residuals": {"max_joint_step_rad": 0.1},
            }
            return [
                list(start_q),
                [segment_end] * 6,
            ]

        mock_plan_high_fidelity.side_effect = plan_high_fidelity_side_effect

        try:
            result = trajectory_execution._plan_linear_move(
                start_q=[0.0] * 6,
                target_pos=np.array([0.20, 0.0, 0.0], dtype=float),
                velocity=0.1,
                acceleration=0.2,
                frequency=100,
                use_smoothing=False,
            )
            self.assertIsNotNone(result)
            self.assertEqual(len(result), 4)
            self.assertAlmostEqual(result[-1][0], 0.2, places=3)
            planner_diag = utils.trajectory_state.get("last_planner_diagnostics", {})
            self.assertEqual(planner_diag.get("reason_code"), "OK")
            self.assertTrue(planner_diag.get("split_recovery", {}).get("used"))
            self.assertEqual(planner_diag.get("split_recovery", {}).get("split_count"), 3)
        finally:
            utils.trajectory_state["last_planner_diagnostics"] = previous_diag


if __name__ == '__main__':
    unittest.main() 
