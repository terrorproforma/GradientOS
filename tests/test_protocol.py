import unittest
import sys
import os

from gradient_os.arm_controller import servo_protocol
from gradient_os.arm_controller import utils
from gradient_os.arm_controller.backends import registry as backend_registry

class TestServoProtocol(unittest.TestCase):
    """
    Unit tests for the low-level servo protocol functions.
    These tests do not require a hardware connection.
    """

    def setUp(self) -> None:
        backend_registry.set_active_backend("feetech")

    def test_calculate_checksum(self) -> None:
        """
        Tests the checksum calculation against a known correct value.
        The example is taken from the Feetech servo documentation for writing
        the value 0x1A to register 0x04 on servo 0x01.
        """
        packet_data = bytearray([0x01, 0x04, 0x03, 0x04, 0x1A])
        # Expected checksum from documentation is 0xD9
        expected_checksum = 0xD9
        calculated_checksum = servo_protocol.calculate_checksum(packet_data)
        self.assertEqual(calculated_checksum, expected_checksum)

    def test_sync_write_packet_structure(self) -> None:
        """
        Tests that the sync_write_goal_pos_speed_accel function generates a
        packet with the correct structure and checksum for a two-servo command.
        """
        # Command servo 10 to pos 1000 and servo 20 to pos 2000
        servo_data = [
            (10, 1000, 100, 1), # (id, pos, speed, accel)
            (20, 2000, 200, 2),
        ]
        
        # Manually construct the expected packet
        # pos 1000 = 0x03E8 -> [0xE8, 0x03]
        # pos 2000 = 0x07D0 -> [0xD0, 0x07]
        # speed 100 = 0x0064 -> [0x64, 0x00]
        # speed 200 = 0x00C8 -> [0xC8, 0x00]
        
        # Expected data block for servo 10:
        # ID(1) + Accel(1) + Pos(2) + Time(2) + Speed(2) = 8 bytes
        # 10, 1, 0xE8, 0x03, 0x00, 0x00, 0x64, 0x00
        
        # Expected data block for servo 20:
        # 20, 2, 0xD0, 0x07, 0x00, 0x00, 0xC8, 0x00

        # L = (Num Servos * (Len Per Servo + 1)) + 4 = (2 * (7+1)) + 4 = 20 = 0x14
        
        # Construct the part of the packet that gets checksummed
        checksum_data = bytearray([
            0xFE, # Broadcast ID
            20,   # Packet Length (L)
            servo_protocol.SERVO_INSTRUCTION_SYNC_WRITE, # Instruction
            servo_protocol.SYNC_WRITE_START_ADDRESS,     # Start Address
            servo_protocol.SYNC_WRITE_DATA_LEN_PER_SERVO, # Len per servo
            10, 1, 0xE8, 0x03, 0x00, 0x00, 0x64, 0x00, # Servo 10 data
            20, 2, 0xD0, 0x07, 0x00, 0x00, 0xC8, 0x00, # Servo 20 data
        ])
        
        expected_checksum = servo_protocol.calculate_checksum(checksum_data)
        
        expected_packet = bytearray([0xFF, 0xFF]) + checksum_data + bytearray([expected_checksum])

        # To test, we need to mock the ser.write call
        with unittest.mock.patch('gradient_os.arm_controller.utils.ser') as mock_serial:
            servo_protocol.sync_write_goal_pos_speed_accel(servo_data)
            # Check that write was called
            mock_serial.write.assert_called_once()
            # Get the actual packet that was written
            actual_packet = mock_serial.write.call_args[0][0]
            self.assertEqual(actual_packet, expected_packet)

    def test_non_serial_backend_sync_reads_do_not_emit_serial_warnings(self) -> None:
        original_ser = utils.ser
        try:
            utils.ser = None
            backend_registry.set_active_backend("ethercat_rtcore")

            with unittest.mock.patch("builtins.print") as mock_print:
                self.assertEqual(servo_protocol.sync_read_positions([0, 1]), {})
                self.assertEqual(servo_protocol.fast_sync_read_positions([0, 1]), {})
                self.assertEqual(
                    servo_protocol.sync_read_block([0, 1], start_address=0x38, data_len=8),
                    {},
                )

            printed = "\n".join(
                str(call.args[0]) for call in mock_print.call_args_list if call.args
            )
            self.assertNotIn("Serial port not initialized", printed)
        finally:
            utils.ser = original_ser
            backend_registry.set_active_backend("feetech")


if __name__ == '__main__':
    unittest.main() 