# robots/base.py
#
# Abstract base class defining the interface for robot configurations.
# Each robot (gradient0, gradient0_5, etc.) must implement this interface
# to provide its kinematic structure, servo mappings, and motion parameters.
#
# This abstraction allows GradientOS to support multiple robot designs
# by simply swapping the robot configuration module.

from abc import ABC, abstractmethod
from typing import Optional
import math


class RobotConfig(ABC):
    """
    Abstract base class defining the interface for robot-specific configuration.
    
    A robot configuration defines the physical and kinematic properties of a robot:
    - Number of joints and their limits
    - Mapping from logical joints to physical actuators
    - Calibration offsets
    - Motion parameters
    - Gripper configuration (if applicable)
    
    The configuration is independent of the actuator type (Feetech, Dynamixel, etc.).
    Actuator-specific settings (encoder resolution, protocol constants, etc.) are
    handled by the ActuatorBackend implementation.
    
    To add a new robot:
    1. Create a new subfolder under robots/ (e.g., robots/my_new_robot/)
    2. Create a config.py that defines a class inheriting from RobotConfig
    3. Implement all abstract methods/properties
    4. Update robots/__init__.py to expose the new config
    
    Example Usage:
    --------------
    ```python
    from gradient_os.arm_controller.robots import Gradient0Config
    
    robot = Gradient0Config()
    print(f"Robot has {robot.num_logical_joints} joints")
    print(f"Joint 1 limits: {robot.logical_joint_limits_rad[0]}")
    ```
    """
    
    # =========================================================================
    # Robot Identity
    # =========================================================================
    
    @property
    @abstractmethod
    def robot_id(self) -> str:
        """
        Stable robot asset identifier used for model/kinematics lookup.

        Returns:
            str: Robot asset ID (e.g., "mini-6dof-arm")
        """
        pass

    @property
    @abstractmethod
    def name(self) -> str:
        """
        Human-readable name of the robot.
        
        Returns:
            str: Robot name (e.g., "Gradient0", "Gradient0.5")
        """
        pass
    
    @property
    @abstractmethod
    def version(self) -> str:
        """
        Version string for this robot configuration.
        
        Returns:
            str: Version (e.g., "1.0.0")
        """
        pass
    
    # =========================================================================
    # Kinematic Structure
    # =========================================================================
    
    @property
    @abstractmethod
    def num_logical_joints(self) -> int:
        """
        Number of controllable joints in the kinematic model.
        
        This is the number of degrees of freedom for motion planning.
        Twin-motor joints still count as a single logical joint.
        
        Returns:
            int: Number of logical joints (typically 6 for a 6-DOF arm)
        """
        pass
    
    @property
    @abstractmethod
    def num_physical_actuators(self) -> int:
        """
        Total number of physical actuators (servos/motors) in the robot.
        
        This includes all arm actuators plus gripper if present.
        For robots with twin-motor joints, this will be > num_logical_joints.
        
        Returns:
            int: Total number of physical actuators
        """
        pass
    
    @property
    @abstractmethod
    def actuator_ids(self) -> list[int]:
        """
        Hardware IDs of all actuators in the robot.
        
        The order matters - it defines the "config index" used for looking up
        per-actuator settings like mapping ranges and limits.
        
        Returns:
            list[int]: List of actuator hardware IDs
        """
        pass
    
    @property
    @abstractmethod
    def logical_to_physical_map(self) -> dict[int, list[int]]:
        """
        Mapping from logical joint index to physical actuator config indices.
        
        This defines how each logical joint (used by motion planning) maps to
        one or more physical actuators. For twin-motor joints, a single logical
        joint maps to two physical actuator indices.
        
        The values are indices into the actuator_ids list, not the IDs themselves.
        
        Example:
            {
                0: [0],      # Logical J1 -> actuator_ids[0] only
                1: [1, 2],   # Logical J2 -> actuator_ids[1] and actuator_ids[2]
                ...
            }
        
        Returns:
            dict[int, list[int]]: Mapping from logical joint to actuator indices
        """
        pass
    
    @property
    def twin_motor_actuator_ids(self) -> dict[int, int]:
        """
        Map of logical joint index to secondary actuator ID for twin-motor joints.
        
        For robots with twin-motor joints (two motors driving one joint for
        increased torque), this identifies which joints have a secondary motor
        and what its ID is.
        
        Returns:
            dict[int, int]: Map from logical joint index to secondary actuator ID.
                           Empty dict if no twin-motor joints.
        """
        return {}  # Default: no twin-motor joints
    
    @property
    def logical_joint_to_actuator_ids(self) -> dict[int, list[int]]:
        """
        Mapping from 1-based logical joint numbers to actual actuator IDs.
        
        This is a convenience property that combines logical_to_physical_map
        and actuator_ids to produce a ready-to-use mapping for commands like
        SET_ZERO that work with joint numbers.
        
        Example for Gradient0:
            {
                1: [10],       # Joint 1 (base) -> Servo ID 10
                2: [20, 21],   # Joint 2 (shoulder) -> Servo IDs 20, 21
                3: [30, 31],   # Joint 3 (elbow) -> Servo IDs 30, 31
                4: [40],       # Joint 4 (wrist roll) -> Servo ID 40
                5: [50],       # Joint 5 (wrist pitch) -> Servo ID 50
                6: [60],       # Joint 6 (wrist yaw) -> Servo ID 60
                7: [100],      # Joint 7 (gripper) -> Servo ID 100 (if present)
            }
        
        Returns:
            dict[int, list[int]]: Map from 1-based joint number to actuator IDs
        """
        result = {}
        actuator_ids = self.actuator_ids
        
        # Map arm joints (1-based)
        for logical_idx, config_indices in self.logical_to_physical_map.items():
            joint_num = logical_idx + 1  # Convert 0-based to 1-based
            result[joint_num] = [actuator_ids[i] for i in config_indices]
        
        # Add gripper as joint 7 if present
        if self.gripper_actuator_id is not None:
            gripper_joint = self.num_logical_joints + 1  # Typically 7
            result[gripper_joint] = [self.gripper_actuator_id]
        
        return result
    
    @property
    @abstractmethod
    def default_servo_backend(self) -> str:
        """
        The default servo backend for this robot.
        
        This specifies which actuator backend to use if not explicitly
        overridden via command-line arguments.
        
        Each robot MUST declare its servo backend explicitly.
        
        Returns:
            str: Backend name (e.g., "feetech", "dynamixel")
        """
        pass

    @property
    @abstractmethod
    def default_ik_solver_backend(self) -> str:
        """
        The default IK solver backend for this robot.

        This policy is the source of truth for solver selection in normal
        operation. Development overrides may supersede it only when explicitly
        enabled.

        Returns:
            str: Backend name (e.g., "ikfast", "numeric")
        """
        pass
    
    # =========================================================================
    # Joint Limits
    # =========================================================================
    
    @property
    @abstractmethod
    def logical_joint_limits_rad(self) -> list[tuple[float, float]]:
        """
        Safe motion limits for each logical joint in radians.
        
        These define the software limits used by motion planning.
        Each tuple is (min_limit, max_limit) in radians.
        
        Returns:
            list[tuple[float, float]]: List of (min, max) limits per joint
        """
        pass
    
    @property
    @abstractmethod
    def actuator_limits_rad(self) -> list[tuple[float, float]]:
        """
        Motion limits for each physical actuator in radians.
        
        These are written to the actuator's EEPROM for hardware-level limit enforcement.
        For twin-motor joints, both actuators get the same limits as their logical joint.
        
        Returns:
            list[tuple[float, float]]: List of (min, max) limits per actuator
        """
        pass
    
    # =========================================================================
    # Angle Mapping
    # =========================================================================
    
    @property
    @abstractmethod
    def actuator_mapping_ranges_rad(self) -> list[tuple[float, float]]:
        """
        The angular range that maps to the full encoder range for each actuator.
        
        Most servos map their full encoder range (e.g., 0-4095) to a specific
        angular range. This is typically (-π, +π) for 360° rotation, but may
        differ for limited-rotation servos.
        
        Each tuple is (min_angle, max_angle) in radians, where:
        - min_angle maps to encoder value 0 (or 4095 for inverted servos)
        - max_angle maps to encoder value 4095 (or 0 for inverted servos)
        
        Returns:
            list[tuple[float, float]]: Angular range per actuator
        """
        pass
    
    @property
    @abstractmethod
    def inverted_actuator_ids(self) -> set[int]:
        """
        Set of actuator IDs that use inverted mapping.
        
        For inverted actuators, the encoder value DECREASES when the angle
        INCREASES. This is determined by how the actuator is physically mounted.
        
        Returns:
            set[int]: Set of actuator IDs with inverted mapping
        """
        pass
    
    def is_actuator_direct_mapping(self, actuator_config_index: int) -> bool:
        """
        Check if an actuator uses direct mapping (encoder increases with angle).
        
        Args:
            actuator_config_index: Index of the actuator in actuator_ids
        
        Returns:
            bool: True if direct mapping, False if inverted
        """
        actuator_id = self.actuator_ids[actuator_config_index]
        return actuator_id not in self.inverted_actuator_ids
    
    # =========================================================================
    # Calibration
    # =========================================================================
    
    @property
    @abstractmethod
    def logical_joint_master_offsets_rad(self) -> list[float]:
        """
        Master calibration offsets per logical joint in radians.
        
        These offsets are added to commanded angles before conversion to
        actuator values. They compensate for mechanical assembly variations.
        
        Returns:
            list[float]: Offset in radians for each logical joint
        """
        pass
    
    # =========================================================================
    # Gripper Configuration
    # =========================================================================
    
    @property
    def has_gripper(self) -> bool:
        """
        Check if this robot has a gripper.
        
        Returns:
            bool: True if gripper is configured
        """
        return self.gripper_actuator_id is not None
    
    @property
    def gripper_actuator_id(self) -> Optional[int]:
        """
        Hardware ID of the gripper actuator.
        
        Returns:
            Optional[int]: Gripper actuator ID, or None if no gripper
        """
        return None
    
    @property
    def gripper_limits_rad(self) -> Optional[tuple[float, float]]:
        """
        Motion limits for the gripper in radians.
        
        Returns:
            Optional[tuple[float, float]]: (min, max) limits, or None if no gripper
        """
        return None
    
    # =========================================================================
    # Motion Parameters (Default values, can be overridden)
    # =========================================================================
    
    @property
    def default_speed(self) -> int:
        """
        Default movement speed for actuators.
        
        The units are actuator-specific (typically 0-4095 for servos).
        
        Returns:
            int: Default speed value
        """
        return 500
    
    @property
    def default_acceleration_deg_s2(self) -> float:
        """
        Default acceleration in degrees per second squared.
        
        Returns:
            float: Default acceleration value
        """
        return 500.0
    
    @property
    def acceleration_scale_factor(self) -> float:
        """
        Factor for converting deg/s² to actuator register value.
        
        register_value = acceleration_deg_s2 / acceleration_scale_factor
        
        Returns:
            float: Scale factor for acceleration conversion
        """
        return 100.0
    
    # =========================================================================
    # Communication Configuration
    # =========================================================================
    
    @property
    def default_serial_port(self) -> str:
        """
        Default serial port path for actuator communication.
        
        Returns:
            str: Serial port path (e.g., "/dev/ttyUSB0")
        """
        return "/dev/ttyUSB0"
    
    # =========================================================================
    # UDP Configuration (for remote control)
    # =========================================================================
    
    @property
    def udp_listen_ip(self) -> str:
        """
        IP address to listen on for UDP commands.
        
        Returns:
            str: IP address (e.g., "0.0.0.0" for all interfaces)
        """
        return "0.0.0.0"
    
    @property
    def udp_port(self) -> int:
        """
        UDP port for receiving commands.
        
        Returns:
            int: Port number
        """
        return 3000
    
    @property
    def udp_buffer_size(self) -> int:
        """
        Buffer size for UDP receive operations.
        
        Returns:
            int: Buffer size in bytes
        """
        return 1024
    
    # =========================================================================
    # Closed-Loop Control Parameters
    # =========================================================================
    
    @property
    def correction_kp_gain(self) -> float:
        """
        Proportional gain for closed-loop trajectory correction.
        
        Set to 0.0 to disable (rely on actuator's internal PID only).
        
        Returns:
            float: Proportional gain
        """
        return 0.0
    
    @property
    def correction_ki_gain(self) -> float:
        """
        Integral gain for closed-loop trajectory correction.
        
        Set to 0.0 to disable.
        
        Returns:
            float: Integral gain
        """
        return 0.0
    
    @property
    def correction_integral_clamp_rad(self) -> float:
        """
        Anti-windup clamp for integral term in radians.
        
        Returns:
            float: Maximum integral accumulation
        """
        return 0.6
    
    # =========================================================================
    # Trapezoidal Profile Defaults
    # =========================================================================
    
    @property
    def default_profile_velocity(self) -> float:
        """
        Default velocity for trapezoidal motion profiles in m/s.
        
        Returns:
            float: Velocity in meters per second
        """
        return 0.1
    
    @property
    def default_profile_acceleration(self) -> float:
        """
        Default acceleration for trapezoidal motion profiles in m/s².
        
        Returns:
            float: Acceleration in meters per second squared
        """
        return 0.05
    
    # =========================================================================
    # PID Configuration
    # =========================================================================
    
    @property
    def default_pid_gains(self) -> tuple[int, int, int]:
        """
        Default PID gains (Kp, Ki, Kd) for actuators not specified individually.
        
        These defaults come from the active servo backend configuration. Override in
        robot subclasses if different defaults are needed for your robot.
        
        Returns:
            tuple[int, int, int]: (Kp, Ki, Kd) gains
        """
        # Get defaults from the active servo backend via registry
        from ..backends import registry as backend_registry
        return backend_registry.get_default_pid_gains()
    
    @property
    def actuator_pid_gains(self) -> dict[int, tuple[int, int, int]]:
        """
        Per-actuator PID gains (Kp, Ki, Kd).
        
        Override this in robot subclasses to provide tuned PID gains
        for each actuator. These gains depend on both the robot's mechanical
        characteristics and the specific actuator type being used.
        
        Returns:
            dict[int, tuple[int, int, int]]: Map from actuator ID to (Kp, Ki, Kd)
        """
        # Default: use default_pid_gains for all actuators
        default = self.default_pid_gains
        return {aid: default for aid in self.actuator_ids}
    
    # =========================================================================
    # Utility Methods
    # =========================================================================
    
    def get_config_dict(self) -> dict:
        """
        Export the robot configuration as a dictionary.
        
        This is useful for passing configuration to the ActuatorBackend
        or for serialization.
        
        Returns:
            dict: Configuration dictionary
        """
        return {
            'robot_id': self.robot_id,
            'name': self.name,
            'version': self.version,
            'num_logical_joints': self.num_logical_joints,
            'num_physical_actuators': self.num_physical_actuators,
            'actuator_ids': self.actuator_ids,
            'logical_to_physical_map': self.logical_to_physical_map,
            'logical_joint_limits_rad': self.logical_joint_limits_rad,
            'actuator_limits_rad': self.actuator_limits_rad,
            'actuator_mapping_ranges_rad': self.actuator_mapping_ranges_rad,
            'inverted_actuator_ids': self.inverted_actuator_ids,
            'logical_joint_master_offsets_rad': self.logical_joint_master_offsets_rad,
            'has_gripper': self.has_gripper,
            'gripper_actuator_id': self.gripper_actuator_id,
            'gripper_limits_rad': self.gripper_limits_rad,
            'default_speed': self.default_speed,
            'default_acceleration_deg_s2': self.default_acceleration_deg_s2,
            'default_serial_port': self.default_serial_port,
            'actuator_pid_gains': self.actuator_pid_gains,
            'default_servo_backend': self.default_servo_backend,
            'default_ik_solver_backend': self.default_ik_solver_backend,
        }
    
    def get_actuator_config_index(self, actuator_id: int) -> Optional[int]:
        """
        Get the config index for an actuator ID.
        
        Args:
            actuator_id: Hardware ID of the actuator
        
        Returns:
            Optional[int]: Index in actuator_ids, or None if not found
        """
        try:
            return self.actuator_ids.index(actuator_id)
        except ValueError:
            return None
    
    def get_arm_actuator_ids(self) -> list[int]:
        """
        Get actuator IDs for the arm only (excluding gripper).
        
        Returns:
            list[int]: List of arm actuator IDs
        """
        if self.gripper_actuator_id is not None:
            return [aid for aid in self.actuator_ids if aid != self.gripper_actuator_id]
        return list(self.actuator_ids)

