# robots/gradient0/config.py
#
# Configuration for the Gradient0 robotic arm.
#
# The Gradient0 is a 6-DOF robotic arm with the following specifications:
# - 6 logical joints (base, shoulder, elbow, wrist roll, wrist pitch, wrist yaw)
# - 9 physical actuators (Feetech STS3215 servos):
#   - Joint 1 (Base): 1 servo (ID 10)
#   - Joint 2 (Shoulder): 2 twin servos (IDs 20, 21)
#   - Joint 3 (Elbow): 2 twin servos (IDs 30, 31)
#   - Joints 4-6 (Wrist): 3 servos (IDs 40, 50, 60)
#   - Gripper: 1 servo (ID 100)
#
# Note: This configuration is independent of the servo type. Servo-specific
# settings (encoder resolution, protocol constants, PID gains) are defined
# in the servo backend (e.g., backends/feetech/config.py).

import math
from typing import Optional

from ..base import RobotConfig


class Gradient0Config(RobotConfig):
    """
    Configuration for the Gradient0 6-DOF robotic arm.
    
    This class defines all robot-specific parameters including:
    - Kinematic structure (6 logical joints, 9 physical servos)
    - Joint limits and mapping ranges
    - Servo orientation (direct vs inverted mounting)
    - Calibration offsets
    - Motion parameters
    
    The Gradient0 uses Feetech STS3215 servos, but this configuration
    is actuator-agnostic - it could theoretically work with any actuator
    backend that implements the ActuatorBackend interface.
    """
    
    # =========================================================================
    # Robot Identity
    # =========================================================================
    
    @property
    def robot_id(self) -> str:
        return "mini-6dof-arm"

    @property
    def name(self) -> str:
        return "Gradient0"
    
    @property
    def version(self) -> str:
        return "1.0.0"
    
    @property
    def default_servo_backend(self) -> str:
        """
        The Gradient0 uses Feetech STS3215 servos.
        """
        return "feetech"

    @property
    def default_ik_solver_backend(self) -> str:
        """Default IK backend policy for Gradient0."""
        return "ikfast"
    
    # =========================================================================
    # Kinematic Structure
    # =========================================================================
    
    @property
    def num_logical_joints(self) -> int:
        """Gradient0 has 6 degrees of freedom (6 controllable joints)."""
        return 6
    
    @property
    def num_physical_actuators(self) -> int:
        """
        Gradient0 has 9 physical servos:
        - 1 for J1 (base)
        - 2 for J2 (shoulder, twin motors)
        - 2 for J3 (elbow, twin motors)
        - 1 for J4 (wrist roll)
        - 1 for J5 (wrist pitch)
        - 1 for J6 (wrist yaw)
        - 1 for gripper
        """
        return 9
    
    @property
    def actuator_ids(self) -> list[int]:
        """
        Hardware IDs of all servos in order.
        
        The order defines the "config index" for each servo:
        - Index 0: ID 10 (J1)
        - Index 1: ID 20 (J2 primary)
        - Index 2: ID 21 (J2 secondary)
        - Index 3: ID 30 (J3 primary)
        - Index 4: ID 31 (J3 secondary)
        - Index 5: ID 40 (J4)
        - Index 6: ID 50 (J5)
        - Index 7: ID 60 (J6)
        - Index 8: ID 100 (Gripper)
        """
        return [10, 20, 21, 30, 31, 40, 50, 60, 100]
    
    @property
    def logical_to_physical_map(self) -> dict[int, list[int]]:
        """
        Maps each logical joint index to its physical servo config indices.
        
        Joint 2 (shoulder) and Joint 3 (elbow) use twin motors for increased torque.
        Both motors in a pair receive the same position command.
        """
        return {
            0: [0],      # J1 (Base) -> Servo index 0 (ID 10)
            1: [1, 2],   # J2 (Shoulder) -> Servo indices 1, 2 (IDs 20, 21)
            2: [3, 4],   # J3 (Elbow) -> Servo indices 3, 4 (IDs 30, 31)
            3: [5],      # J4 (Wrist Roll) -> Servo index 5 (ID 40)
            4: [6],      # J5 (Wrist Pitch) -> Servo index 6 (ID 50)
            5: [7],      # J6 (Wrist Yaw) -> Servo index 7 (ID 60)
        }
    
    @property
    def twin_motor_actuator_ids(self) -> dict[int, int]:
        """
        Map of logical joint index to secondary actuator ID for twin-motor joints.
        
        On the Gradient0, joints 2 and 3 use twin motors for increased torque.
        This property identifies the secondary motor for each twin-motor joint.
        
        Returns:
            dict[int, int]: Map from logical joint index to secondary actuator ID
        """
        return {
            1: 21,  # J2 secondary motor
            2: 31,  # J3 secondary motor
        }
    
    # =========================================================================
    # Joint Limits
    # =========================================================================
    
    @property
    def logical_joint_limits_rad(self) -> list[tuple[float, float]]:
        """
        Motion limits for each logical joint in radians.
        
        These are the software limits used by motion planning.
        Hardware limits (written to servo EEPROM) may be slightly different.
        """
        return [
            (-math.pi, math.pi),      # J1 (Base): ±180°
            (-math.pi/2, math.pi/2),  # J2 (Shoulder): ±90°
            (-math.pi/2, math.pi/2),  # J3 (Elbow): ±90°
            (-math.pi, math.pi),      # J4 (Wrist Roll): ±180°
            (-1.8326, 2.0944),        # J5 (Wrist Pitch): ~-105° to +120°
            (-math.pi, math.pi),      # J6 (Wrist Yaw): ±180°
        ]
    
    @property
    def actuator_limits_rad(self) -> list[tuple[float, float]]:
        """
        Motion limits for each physical servo in radians.
        
        For twin-motor joints, both servos get the same limits as their logical joint.
        These values are written to the servos' EEPROM for hardware-level enforcement.
        """
        logical = self.logical_joint_limits_rad
        return [
            logical[0],  # Servo 10 (J1)
            logical[1],  # Servo 20 (J2)
            logical[1],  # Servo 21 (J2) - same as primary
            logical[2],  # Servo 30 (J3)
            logical[2],  # Servo 31 (J3) - same as primary
            logical[3],  # Servo 40 (J4)
            logical[4],  # Servo 50 (J5)
            logical[5],  # Servo 60 (J6)
            (0, math.pi),  # Servo 100 (Gripper): 0° to 180°
        ]
    
    @property
    def gripper_limits_rad(self) -> Optional[tuple[float, float]]:
        """Gripper motion limits: 0 (closed) to π (fully open)."""
        return (0, math.pi)
    
    # =========================================================================
    # Angle Mapping
    # =========================================================================
    
    @property
    def actuator_mapping_ranges_rad(self) -> list[tuple[float, float]]:
        """
        Angular range that maps to the full encoder range for each servo.
        
        The Feetech STS3215 servos have a 360° rotation range that maps
        to encoder values 0-4095. This corresponds to ±π radians from center.
        """
        return [
            (-math.pi, math.pi),  # Servo 10 (J1)
            (-math.pi, math.pi),  # Servo 20 (J2)
            (-math.pi, math.pi),  # Servo 21 (J2)
            (-math.pi, math.pi),  # Servo 30 (J3)
            (-math.pi, math.pi),  # Servo 31 (J3)
            (-math.pi, math.pi),  # Servo 40 (J4)
            (-math.pi, math.pi),  # Servo 50 (J5)
            (-math.pi, math.pi),  # Servo 60 (J6)
            (-math.pi, math.pi),  # Servo 100 (Gripper)
        ]
    
    @property
    def inverted_actuator_ids(self) -> set[int]:
        """
        Set of servo IDs with inverted mounting.
        
        For inverted servos, the encoder value DECREASES when the joint
        angle INCREASES. This is determined by how the servo is physically
        mounted on the robot.
        
        On the Gradient0, most servos are inverted due to their mounting orientation.
        """
        return {10, 20, 30, 40, 50, 60, 100}
    
    # =========================================================================
    # Calibration
    # =========================================================================
    
    @property
    def logical_joint_master_offsets_rad(self) -> list[float]:
        """
        Master calibration offsets for each logical joint in radians.
        
        These are applied to commanded angles before conversion to servo values.
        They compensate for minor assembly variations without requiring
        individual servo recalibration.
        
        These offsets are typically adjusted during initial robot setup
        to ensure the arm's home position matches the CAD model.
        """
        return [
            0.0,  # J1 (Base)
            0.0,  # J2 (Shoulder)
            0.0,  # J3 (Elbow)
            0.0,  # J4 (Wrist Roll)
            0.0,  # J5 (Wrist Pitch)
            0.0,  # J6 (Wrist Yaw)
        ]
    
    # =========================================================================
    # Gripper Configuration
    # =========================================================================
    
    @property
    def gripper_actuator_id(self) -> Optional[int]:
        """Hardware ID of the gripper servo."""
        return 100
    
    # =========================================================================
    # Motion Parameters
    # =========================================================================
    
    @property
    def default_speed(self) -> int:
        """
        Default servo speed (0-4095 scale).
        
        500 provides a moderate speed suitable for most operations.
        Higher values = faster movement.
        """
        return 500
    
    @property
    def default_acceleration_deg_s2(self) -> float:
        """
        Default acceleration in degrees per second squared.
        
        500 deg/s² provides smooth acceleration/deceleration.
        """
        return 500.0
    
    @property
    def acceleration_scale_factor(self) -> float:
        """
        Factor for converting deg/s² to servo register value.
        
        For Feetech STS3215 servos:
        - Register accepts 0-254 (0 = maximum acceleration)
        - register_value = deg/s² / 100
        
        Example: 500 deg/s² -> register value 5
        """
        return 100.0
    
    # =========================================================================
    # Communication
    # =========================================================================
    
    @property
    def default_serial_port(self) -> str:
        """Default serial port for USB-TTL adapter."""
        return "/dev/ttyUSB0"
    
    @property
    def udp_listen_ip(self) -> str:
        """Listen on all interfaces for remote control."""
        return "0.0.0.0"
    
    @property
    def udp_port(self) -> int:
        """UDP port for receiving motion commands."""
        return 3000
    
    @property
    def udp_buffer_size(self) -> int:
        """UDP receive buffer size in bytes."""
        return 1024
    
    # =========================================================================
    # Closed-Loop Control
    # =========================================================================
    
    @property
    def correction_kp_gain(self) -> float:
        """
        Proportional gain for trajectory correction.
        
        Set to 0.0 to rely solely on the servo's internal PID controller.
        This is often preferable during servo PID tuning.
        """
        return 0.0
    
    @property
    def correction_ki_gain(self) -> float:
        """
        Integral gain for trajectory correction.
        
        Set to 0.0 to disable integral correction.
        """
        return 0.0
    
    @property
    def correction_integral_clamp_rad(self) -> float:
        """
        Anti-windup clamp for integral term in radians.
        
        0.6 rad ≈ ±34° maximum integral accumulation.
        """
        return 0.6
    
    # =========================================================================
    # Trapezoidal Profile Defaults
    # =========================================================================
    
    @property
    def default_profile_velocity(self) -> float:
        """Default velocity for trapezoidal profiles: 0.1 m/s."""
        return 0.1
    
    @property
    def default_profile_acceleration(self) -> float:
        """Default acceleration for trapezoidal profiles: 0.05 m/s²."""
        return 0.05
    
    # =========================================================================
    # PID Gains (Robot-specific tuning for Feetech STS3215 servos)
    # =========================================================================
    
    @property
    def default_pid_gains(self) -> tuple[int, int, int]:
        """
        Default PID gains (Kp, Ki, Kd) if not specified per-actuator.
        
        These values are tuned for Feetech STS3215 servos on the Gradient0 arm.
        """
        return (50, 1, 30)
    
    @property
    def actuator_pid_gains(self) -> dict[int, tuple[int, int, int]]:
        """
        Per-actuator PID gains (Kp, Ki, Kd) tuned for this robot.
        
        These gains are optimized for the Gradient0's specific mechanical
        characteristics (weight distribution, gear ratios, mounting, etc.)
        with Feetech STS3215 servos.
        
        Returns:
            dict[int, tuple[int, int, int]]: Map from actuator ID to (Kp, Ki, Kd)
        """
        # Per-joint tuned gains
        j1_gains = (50, 1, 20)   # Base joint
        j2_gains = (65, 1, 30)   # Shoulder (higher P for heavier load)
        j3_gains = (50, 1, 25)   # Elbow
        j4_gains = (50, 0, 10)   # Wrist roll (lighter, less I needed)
        j5_gains = (50, 0, 10)   # Wrist pitch
        j6_gains = (50, 0, 10)   # Wrist yaw
        gripper_gains = (40, 0, 10)  # Gripper (lower P for softer grip)
        
        return {
            10: j1_gains,
            20: j2_gains,
            21: j2_gains,  # Twin motor, same gains
            30: j3_gains,
            31: j3_gains,  # Twin motor, same gains
            40: j4_gains,
            50: j5_gains,
            60: j6_gains,
            100: gripper_gains,
        }

