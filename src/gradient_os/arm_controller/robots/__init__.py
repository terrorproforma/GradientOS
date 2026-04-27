# robots/__init__.py
#
# Robot configuration package for GradientOS.
# This package contains configuration classes for different robot models.
#
# Each robot has its own subfolder with a config.py defining a RobotConfig subclass.
# The base RobotConfig class provides the interface that all robots must implement.
#
# Supported Robots:
# -----------------
# - Gradient0: Original 6-DOF arm with 9 Feetech servos (8 arm + 1 gripper)
# - [Future] Gradient0.5: Next iteration with improvements
#
# Usage:
# ------
# ```python
# from gradient_os.arm_controller.robots import Gradient0Config, RobotConfig
#
# # Get the config for Gradient0
# robot = Gradient0Config()
#
# # Or get by name
# robot = get_robot_config("gradient0")
# ```

from .base import RobotConfig
from .fr10 import FR10Config
from .gradient0 import Gradient0Config

# Registry of available robot configurations
_ROBOT_REGISTRY: dict[str, type[RobotConfig]] = {
    'gradient0': Gradient0Config,
    'fr10': FR10Config,
}


def get_robot_config(robot_name: str) -> RobotConfig:
    """
    Get a robot configuration instance by name.
    
    Args:
        robot_name: Name of the robot (case-insensitive)
    
    Returns:
        RobotConfig: An instance of the robot's configuration class
    
    Raises:
        ValueError: If the robot name is not recognized
    """
    name_lower = robot_name.lower()
    if name_lower not in _ROBOT_REGISTRY:
        available = ', '.join(_ROBOT_REGISTRY.keys())
        raise ValueError(f"Unknown robot '{robot_name}'. Available: {available}")
    return _ROBOT_REGISTRY[name_lower]()


def list_available_robots() -> list[str]:
    """
    List all available robot configurations.
    
    Returns:
        list[str]: Names of available robots
    """
    return list(_ROBOT_REGISTRY.keys())


def register_robot(name: str, config_class: type[RobotConfig]) -> None:
    """
    Register a new robot configuration.
    
    This allows external packages to add robot configurations at runtime.
    
    Args:
        name: Name to register the robot under
        config_class: The RobotConfig subclass to register
    """
    _ROBOT_REGISTRY[name.lower()] = config_class


__all__ = [
    'RobotConfig',
    'Gradient0Config',
    'FR10Config',
    'get_robot_config',
    'list_available_robots',
    'register_robot',
]

