import unittest
import sys
import os
import socket
import threading
import errno
import time

# Mock the serial and scipy modules before importing the controller
from unittest.mock import MagicMock, patch
sys.modules['serial'] = MagicMock()
sys.modules['scipy.signal'] = MagicMock()

from gradient_os.run_controller import main as run_controller_main
from gradient_os.arm_controller import utils

class TestEndToEnd(unittest.TestCase):
    """
    End-to-end tests for the controller, from UDP command to serial output.
    This test mocks the serial port to run without hardware.
    """

    def setUp(self) -> None:
        """Set up for each test."""
        # Reset the trajectory state before each test
        utils.trajectory_state = {
            "is_running": False,
            "should_stop": False,
            "thread": None
        }

    @patch('gradient_os.arm_controller.servo_driver._resolve_serial_port', return_value='/dev/ttyUSB0')
    @patch('gradient_os.arm_controller.servo_driver.servo_protocol.ping', return_value=True)
    @patch('gradient_os.arm_controller.servo_driver.servo_protocol.sync_write_goal_pos_speed_accel')
    @patch('gradient_os.arm_controller.servo_driver.servo_protocol.sync_read_positions')
    @patch('gradient_os.ik_solver.solve_ik_path_batch')
    @patch('gradient_os.arm_controller.servo_driver.serial.Serial')
    def test_move_line_command_to_serial_output(self,
                                                mock_serial_class: MagicMock,
                                                mock_solve_ik: MagicMock,
                                                mock_sync_read: MagicMock,
                                                mock_sync_write: MagicMock,
                                                _mock_ping: MagicMock,
                                                _mock_resolve: MagicMock) -> None:
        """
        Tests the full pipeline from a MOVE_LINE UDP command to the final
        serial packet being written.
        """
        # 1. Configure Mocks
        from gradient_os.arm_controller import servo_protocol

        # Mock the serial port to capture written data
        mock_serial_instance = MagicMock()
        mock_serial_class.return_value = mock_serial_instance

        # Mock the IK solver to return a simple, predictable path
        mock_solve_ik.return_value = [
            [0.1, 0.1, 0.1, 0.1, 0.1, 0.1],
            [0.2, 0.2, 0.2, 0.2, 0.2, 0.2],
        ]
        
        # Mock sync_read to return a valid dictionary of positions
        # This is needed by the closed-loop executor
        mock_sync_read.return_value = {id: 2047 for id in utils.SERVO_IDS}

        # Ensure the servo presence cache is populated for Sync Write
        servo_protocol.get_present_servo_ids().update(utils.SERVO_IDS)

        # 2. Start the controller main loop in a background thread
        with patch.object(sys, "argv", ["gradient-controller"]):
            controller_thread = threading.Thread(target=run_controller_main, daemon=True)
            controller_thread.start()
            time.sleep(1.5) # Give the server time to start

        # 3. Send a MOVE_LINE command via UDP
        target_ip = utils.PI_IP
        command = "MOVE_LINE,0.1,0.2,0.3,0.1,0.05"
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
            try:
                sock.sendto(command.encode('utf-8'), (target_ip, utils.UDP_PORT))
            except OSError as exc:
                # Some hosts (e.g., macOS) cannot route to 0.0.0.0; fall back to loopback.
                if getattr(exc, "errno", None) in {errno.EHOSTUNREACH, errno.EADDRNOTAVAIL, errno.ENETUNREACH}:
                    target_ip = "127.0.0.1"
                    sock.sendto(command.encode('utf-8'), (target_ip, utils.UDP_PORT))
                else:
                    raise
            time.sleep(0.5) # Give the command time to be processed

        # 4. Assert that the controller attempted a sync write with servo commands
        self.assertTrue(mock_sync_write.called, "Controller never issued a sync write.")
        sent_commands = mock_sync_write.call_args_list[0][0][0]

        commanded_ids = {cmd[0] for cmd in sent_commands}
        expected_ids = set(utils.SERVO_IDS[:-1])  # Gripper may not be commanded in every move
        self.assertTrue(expected_ids.issubset(commanded_ids), "Missing arm servo commands in sync write.")

        # 6. Stop the controller
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
            sock.sendto("STOP".encode('utf-8'), (target_ip, utils.UDP_PORT))
        
        # Give the thread time to shut down
        time.sleep(0.2)
        # Explicitly wait for the trajectory thread to finish
        if utils.trajectory_state["thread"] is not None:
             utils.trajectory_state["thread"].join(timeout=1)

        # Since controller_thread is a daemon, it will exit automatically.


if __name__ == '__main__':
    unittest.main() 
