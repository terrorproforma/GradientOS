import unittest
import sys
import os
import socket
import threading
import time

# Add the 'src' directory to the Python path
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'src')))

# Mock the serial and scipy modules before importing the controller
from unittest.mock import MagicMock, patch
sys.modules['serial'] = MagicMock()
sys.modules['scipy.signal'] = MagicMock()

from run_controller import main as run_controller_main
from arm_controller import utils

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

    @patch('arm_controller.servo_driver.servo_protocol.sync_read_positions')
    @patch('ik_solver.solve_ik_path_batch')
    @patch('serial.Serial')
    def test_move_line_command_to_serial_output(self, mock_serial_class: MagicMock, 
                                                mock_solve_ik: MagicMock,
                                                mock_sync_read: MagicMock) -> None:
        """
        Tests the full pipeline from a MOVE_LINE UDP command to the final
        serial packet being written.
        """
        # 1. Configure Mocks
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

        # 2. Start the controller main loop in a background thread
        controller_thread = threading.Thread(target=run_controller_main, daemon=True)
        controller_thread.start()
        time.sleep(0.2) # Give the server time to start

        # 3. Send a MOVE_LINE command via UDP
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
            command = "MOVE_LINE,0.1,0.2,0.3,0.1,0.05"
            sock.sendto(command.encode('utf-8'), (utils.PI_IP, utils.UDP_PORT))
            time.sleep(0.2) # Give the command time to be processed

        # 4. Assert that the serial port was written to
        self.assertTrue(mock_serial_instance.write.called)
        
        # 5. Check the content of the first packet written
        first_packet = mock_serial_instance.write.call_args_list[0][0][0]
        
        # Assert it's a SYNC WRITE packet
        self.assertEqual(first_packet[4], utils.SERVO_INSTRUCTION_SYNC_WRITE)
        # Assert it's for all servos
        num_servos = first_packet[3] // (utils.SYNC_WRITE_DATA_LEN_PER_SERVO + 1)
        self.assertEqual(num_servos, utils.NUM_PHYSICAL_SERVOS)

        # 6. Stop the controller
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
            sock.sendto("STOP".encode('utf-8'), (utils.PI_IP, utils.UDP_PORT))
        
        # Give the thread time to shut down
        time.sleep(0.1)
        # Explicitly wait for the trajectory thread to finish
        if utils.trajectory_state["thread"] is not None:
             utils.trajectory_state["thread"].join(timeout=1)

        # Since controller_thread is a daemon, it will exit automatically.


if __name__ == '__main__':
    unittest.main() 