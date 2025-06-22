import unittest
import sys
import os
import math

# Add the 'src' directory to the Python path to allow importing the arm_controller package
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'src')))

from arm_controller import servo_driver
from arm_controller import utils

class TestServoDriver(unittest.TestCase):
    """
    Unit tests for the high-level servo driver functions.
    These tests do not require a hardware connection.
    """

    def test_radian_to_raw_conversion(self) -> None:
        """
        Tests the conversion from raw servo values to radians, checking both a
        direct-mapped and an inverted servo.
        """
        # Test a direct-mapped servo (e.g., servo index 0 for J1)
        # Center value (2047) should correspond to ~0.0 rad
        rad_val_center = servo_driver.servo_value_to_radians(2047, 0)
        self.assertAlmostEqual(rad_val_center, 0.0, places=2)

        # 3/4 value (3071) should correspond to ~+PI/2 rad
        rad_val_pi_half = servo_driver.servo_value_to_radians(3071, 0)
        self.assertAlmostEqual(rad_val_pi_half, math.pi / 2, places=2)

        # Test an inverted servo (servo index 7 for J5)
        # Center value (2047) should still correspond to ~0.0 rad
        rad_val_center_inv = servo_driver.servo_value_to_radians(2047, 7)
        self.assertAlmostEqual(rad_val_center_inv, 0.0, places=2)

        # 1/4 value (1023) for an inverted servo should correspond to ~+PI/2 rad
        rad_val_pi_half_inv = servo_driver.servo_value_to_radians(1023, 7)
        self.assertAlmostEqual(rad_val_pi_half_inv, math.pi / 2, places=2)

    @unittest.mock.patch('arm_controller.servo_protocol.sync_write_goal_pos_speed_accel')
    def test_j1_gear_ratio(self, mock_sync_write: unittest.mock.Mock) -> None:
        """
        Tests that a command to the logical Joint 1 results in a physical command
        that is double the angle due to the 2:1 gear ratio.
        """
        # Command J1 to PI/4 radians (45 degrees)
        # The physical servos should be commanded to PI/2 radians (90 degrees)
        logical_angles = [math.pi / 4, 0, 0, 0, 0, 0]
        
        servo_driver.set_servo_positions(logical_angles, 100, 0)

        # Get the arguments that sync_write was called with
        call_args = mock_sync_write.call_args[0][0]
        
        # Find the command for the first servo of J1 (ID 10, index 0)
        servo_10_command = next((cmd for cmd in call_args if cmd[0] == 10), None)
        self.assertIsNotNone(servo_10_command)
        
        # Convert the raw command back to radians to check it
        raw_pos_cmd = servo_10_command[1] # The position value
        
        # Convert raw value back to physical radians
        # This uses the inverse logic of set_servo_positions
        is_direct = utils._is_servo_direct_mapping(0)
        normalized = raw_pos_cmd / 4095.0
        if not is_direct:
            normalized = 1.0 - normalized
            
        min_rad, max_rad = utils.EFFECTIVE_MAPPING_RANGES[0]
        physical_angle_rad = normalized * (max_rad - min_rad) + min_rad

        # The physical angle should be double the logical angle
        self.assertAlmostEqual(physical_angle_rad, math.pi / 2, places=2)


if __name__ == '__main__':
    unittest.main() 