# actuator_interface.py
# 
# Abstract base class defining the interface that any actuator backend must implement.
# This allows GradientOS to work with different servo types (Feetech, Dynamixel, etc.)
# or even different actuator systems (steppers, hydraulics) by implementing this interface.
#
# The interface is designed around the concept of "logical joints" - the kinematic joints
# of the robot - rather than physical servos. This abstraction allows the backend to
# handle the mapping from logical joints to physical actuators (e.g., twin-motor joints,
# gear ratios, etc.) internally.

from abc import ABC, abstractmethod
from typing import Optional, TYPE_CHECKING
import math
import numpy as np

if TYPE_CHECKING:
    from .robots.base import RobotConfig


class ActuatorBackend(ABC):
    """
    Abstract base class defining the interface for actuator control systems.
    
    Any actuator backend (Feetech servos, Dynamixel servos, stepper motors, etc.)
    must implement this interface to be compatible with GradientOS's motion planning
    and trajectory execution systems.
    
    The interface operates on "logical joints" which represent the kinematic joints
    of the robot. The backend is responsible for mapping these to physical actuators.
    
    Key Concepts:
    -------------
    - Logical Joint: A joint in the kinematic model (e.g., shoulder, elbow)
    - Physical Actuator: The actual hardware (servo, motor) that moves the joint
    - A single logical joint may be driven by multiple physical actuators (e.g., twin motors)
    
    Thread Safety:
    --------------
    Implementations should be thread-safe, as methods may be called from multiple threads
    (e.g., the main control loop and a trajectory executor thread).
    """
    
    # =========================================================================
    # Initialization & Configuration
    # =========================================================================
    
    @abstractmethod
    def initialize(self) -> bool:
        """
        Initialize the actuator system (open serial ports, ping devices, etc.).
        
        This method should:
        - Establish communication with the hardware
        - Detect which actuators are present
        - Apply default configuration (PID gains, limits, etc.)
        - Populate internal state about the system
        
        Returns:
            bool: True if initialization was successful, False otherwise.
        """
        pass
    
    @abstractmethod
    def shutdown(self) -> None:
        """
        Cleanly shut down the actuator system.
        
        This method should:
        - Stop any ongoing motion
        - Close serial ports/connections
        - Release any resources
        """
        pass

    def safe_power_down(self) -> bool:
        """
        Best-effort transition to a non-active hardware state without requiring
        the whole controller process to exit first.

        Returns:
            bool: True if the backend handled a hardware power-down/de-energize
            action, False if no special power-down behavior is implemented.
        """
        return False

    def safe_power_up(self) -> bool:
        """
        Best-effort transition to an active/armed hardware state.

        Returns:
            bool: True if the backend handled a hardware power-up/enable action,
            False if no special power-up behavior is implemented.
        """
        return False
    
    @property
    @abstractmethod
    def num_joints(self) -> int:
        """
        Returns the number of logical joints this backend controls.
        
        Returns:
            int: Number of controllable joints in the kinematic model.
        """
        pass
    
    @property
    @abstractmethod
    def is_initialized(self) -> bool:
        """
        Check if the backend has been successfully initialized.
        
        Returns:
            bool: True if initialize() has been called successfully.
        """
        pass
    
    @property
    @abstractmethod
    def encoder_resolution(self) -> int:
        """
        Returns the encoder resolution (maximum raw value) for this actuator type.
        
        For example:
        - Feetech STS3215 servos: 4095 (12-bit encoder)
        - Dynamixel servos: may vary by model (typically 4095 or 65535)
        - Stepper motors: depends on microstepping configuration
        
        This value is used for converting between angles and raw encoder values.
        
        Returns:
            int: Maximum raw encoder value (e.g., 4095 for 12-bit encoders)
        """
        pass
    
    @property
    def encoder_center(self) -> int:
        """
        Returns the center encoder value (representing 0 radians).
        
        Default implementation returns encoder_resolution // 2 (e.g., 2048 for 4095).
        Override if your actuator uses a different center value.
        
        Returns:
            int: Center encoder value
        """
        return self.encoder_resolution // 2
    
    # =========================================================================
    # Position Control - Standard Interface
    # =========================================================================
    
    @abstractmethod
    def set_joint_positions(
        self,
        positions_rad: list[float],
        speed: float,
        acceleration: float,
    ) -> None:
        """
        Command all joints to move to specified positions.
        
        This is the primary method for commanding joint motion. The backend is
        responsible for translating these logical joint angles into the appropriate
        commands for the physical actuators.
        
        Args:
            positions_rad: List of target positions in radians, one per logical joint.
                           Length must equal `num_joints`.
            speed: Movement speed. Units are backend-specific but typically:
                   - For servos: 0-4095 (servo register value) or deg/s
                   - For steppers: steps/second
            acceleration: Movement acceleration. Units are backend-specific but typically:
                          - For servos: deg/s² or register value
                          - For steppers: steps/s²
        
        Raises:
            ValueError: If len(positions_rad) != num_joints
        """
        pass
    
    @abstractmethod
    def get_joint_positions(self, verbose: bool = False) -> list[float]:
        """
        Read the current position of all logical joints.
        
        Args:
            verbose: If True, print debug information about the read operation.
        
        Returns:
            list[float]: Current joint positions in radians, one per logical joint.
                         Length equals `num_joints`.
        """
        pass
    
    # =========================================================================
    # High-Speed Batch Operations (for real-time control loops)
    # =========================================================================
    
    @abstractmethod
    def prepare_sync_write_commands(
        self,
        positions_rad: list[float],
        speed: int = 4095,
        accel: int = 0,
    ) -> list[tuple]:
        """
        Pre-compute the low-level commands for a sync write operation.
        
        This method converts logical joint positions to the format required by the
        hardware, allowing the actual write to be performed with minimal latency
        in a real-time control loop.
        
        Args:
            positions_rad: Target joint positions in radians.
            speed: Speed value (backend-specific, typically 0-4095 for servos).
            accel: Acceleration value (backend-specific, typically 0-254 for servos).
        
        Returns:
            list[tuple]: Backend-specific command data ready for sync_write().
        """
        pass
    
    @abstractmethod
    def sync_write(self, commands: list[tuple]) -> None:
        """
        Execute a batch write operation with pre-computed commands.
        
        This is the fastest way to command multiple actuators simultaneously.
        The commands should be prepared using prepare_sync_write_commands().
        
        Args:
            commands: Pre-computed command data from prepare_sync_write_commands().
        """
        pass
    
    @abstractmethod
    def sync_read_positions(
        self,
        timeout_s: Optional[float] = None,
    ) -> dict[int, int]:
        """
        Read positions from all actuators in a single batch operation.
        
        This is the fastest way to get feedback from multiple actuators for
        closed-loop control.
        
        Args:
            timeout_s: Optional timeout for the read operation in seconds.
        
        Returns:
            dict[int, int]: Mapping of actuator ID to raw position value.
                           The backend is responsible for providing a method
                           to convert these to joint angles if needed.
        """
        pass
    
    @abstractmethod
    def raw_to_joint_positions(self, raw_positions: dict[int, int]) -> list[float]:
        """
        Convert raw actuator position readings to logical joint angles.
        
        Args:
            raw_positions: Mapping of actuator ID to raw position value
                          (as returned by sync_read_positions).
        
        Returns:
            list[float]: Joint positions in radians, one per logical joint.
        """
        pass
    
    # =========================================================================
    # Single Actuator Control (for gripper, calibration, etc.)
    # =========================================================================
    
    @abstractmethod
    def set_single_actuator_position(
        self,
        actuator_id: int,
        position_rad: float,
        speed: int,
        accel: int,
    ) -> None:
        """
        Command a single actuator to a specific position.
        
        Used for controlling auxiliary actuators (gripper) or for calibration.
        
        Args:
            actuator_id: The hardware ID of the actuator.
            position_rad: Target position in radians.
            speed: Movement speed (backend-specific units).
            accel: Movement acceleration (backend-specific units).
        """
        pass
    
    @abstractmethod
    def read_single_actuator_position(self, actuator_id: int) -> Optional[int]:
        """
        Read the raw position of a single actuator.
        
        Args:
            actuator_id: The hardware ID of the actuator.
        
        Returns:
            Optional[int]: Raw position value, or None if read failed.
        """
        pass
    
    # =========================================================================
    # Calibration & Configuration
    # =========================================================================
    
    @abstractmethod
    def set_current_position_as_zero(self, actuator_id: int) -> bool:
        """
        Set the current physical position of an actuator as its zero point.
        
        This is used for calibration - the actuator will treat its current
        position as the origin (typically raw value 2048 for servos).
        
        Args:
            actuator_id: The hardware ID of the actuator to calibrate.
        
        Returns:
            bool: True if calibration was successful, False otherwise.
        """
        pass
    
    @abstractmethod
    def set_pid_gains(
        self,
        actuator_id: int,
        kp: int,
        ki: int,
        kd: int,
    ) -> bool:
        """
        Set the PID gains for a specific actuator's internal controller.
        
        Args:
            actuator_id: The hardware ID of the actuator.
            kp: Proportional gain.
            ki: Integral gain.
            kd: Derivative gain.
        
        Returns:
            bool: True if gains were set successfully, False otherwise.
        """
        pass
    
    @abstractmethod
    def apply_joint_limits(self) -> bool:
        """
        Apply the configured joint limits to all actuators.
        
        This writes the limit values to the actuators' EEPROM so they
        enforce the limits at the hardware level.
        
        Returns:
            bool: True if all limits were set successfully, False otherwise.
        """
        pass
    
    # =========================================================================
    # Gripper Support (Optional - default implementations provided)
    # =========================================================================
    
    @property
    def has_gripper(self) -> bool:
        """
        Check if this backend has a gripper attached.
        
        Returns:
            bool: True if a gripper is present and controllable.
        """
        return False
    
    @property
    def gripper_actuator_id(self) -> Optional[int]:
        """
        Get the hardware ID of the gripper actuator.
        
        Returns:
            Optional[int]: Gripper actuator ID, or None if no gripper.
        """
        return None
    
    def set_gripper_position(self, position_rad: float, speed: int = 50, accel: int = 0) -> None:
        """
        Set the gripper to a specific position.
        
        Args:
            position_rad: Target gripper position in radians.
            speed: Movement speed.
            accel: Movement acceleration.
        """
        if self.has_gripper and self.gripper_actuator_id is not None:
            self.set_single_actuator_position(
                self.gripper_actuator_id, position_rad, speed, accel
            )
    
    def get_gripper_position(self) -> Optional[float]:
        """
        Read the current gripper position.
        
        Returns:
            Optional[float]: Gripper position in radians, or None if not available.
        """
        return None
    
    # =========================================================================
    # Diagnostics & Utilities
    # =========================================================================
    
    @abstractmethod
    def get_present_actuator_ids(self) -> set[int]:
        """
        Get the set of actuator IDs that were detected during initialization.
        
        Returns:
            set[int]: Set of hardware IDs for actuators that responded to ping.
        """
        pass
    
    @abstractmethod
    def ping_actuator(self, actuator_id: int) -> bool:
        """
        Check if a specific actuator is responsive.
        
        Args:
            actuator_id: The hardware ID to ping.
        
        Returns:
            bool: True if the actuator responded, False otherwise.
        """
        pass
    
    def factory_reset_actuator(self, actuator_id: int) -> bool:
        """
        Reset an actuator to factory defaults.
        
        Args:
            actuator_id: The hardware ID of the actuator to reset.
        
        Returns:
            bool: True if reset was successful, False otherwise.
        """
        return False  # Default: not supported
    
    def restart_actuator(self, actuator_id: int) -> bool:
        """
        Restart/reboot an actuator.
        
        Args:
            actuator_id: The hardware ID of the actuator to restart.
        
        Returns:
            bool: True if restart command was sent successfully.
        """
        return False  # Default: not supported

    def reset_faults(self, logical_joint_index: Optional[int] = None) -> bool:
        """
        Best-effort reset of active drive/controller faults.

        Args:
            logical_joint_index: Optional 0-based logical joint index to target.
                If omitted, reset faults for all supported joints/axes.

        Returns:
            bool: True if a fault-reset request was sent successfully.
        """
        return False  # Default: not supported


# =============================================================================
# SimulationBackend - Now located in backends/simulation/backend.py
# =============================================================================
# Re-export for backward compatibility - new code should import from:
#   from gradient_os.arm_controller.backends.simulation import SimulationBackend

from .backends.simulation import SimulationBackend

__all__ = ['ActuatorBackend', 'SimulationBackend']

