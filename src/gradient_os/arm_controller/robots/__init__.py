# robots/__init__.py
#
# Robot configuration package for GradientOS.
# This package contains configuration classes for different robot models.
#
# Each robot has its own subfolder with a config.py defining a RobotConfig subclass.
# The base RobotConfig class provides the interface that all robots must implement.
#
# Note:
# Robot *assets* (URDF, DH CSV, meshes) are stored in the repository-level
# `robots/<robot_id>/` catalog with a `robot.json` manifest. Runtime config
# classes in this package must expose `robot_id` so solver/UI layers can resolve
# those canonical assets.
#
# Supported Robots:
# -----------------
# - Gradient0: Original 6-DOF arm with 9 Feetech servos (8 arm + 1 gripper)
# - Gradient05: Template scaffold for the next robot iteration
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
from .gradient0 import Gradient0Config
from .gradient05 import Gradient05Config

# Registry of available robot configurations
_ROBOT_REGISTRY: dict[str, type[RobotConfig]] = {
    'gradient0': Gradient0Config,
    'gradient05': Gradient05Config,
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


def get_robot_name_by_id(robot_id: str) -> str | None:
    """
    Resolve registry name (e.g. ``gradient05``) from stable robot asset ID.
    """
    target = str(robot_id).strip()
    if not target:
        return None
    for name, config_class in _ROBOT_REGISTRY.items():
        try:
            cfg = config_class()
        except Exception:
            continue
        if cfg.robot_id == target:
            return name
    return None


def list_robot_metadata() -> list[dict[str, str]]:
    """
    Return robot metadata suitable for API/UI selection lists.
    """
    from ..backends import registry as backend_registry

    out: list[dict[str, str]] = []
    for name in list_available_robots():
        cfg = get_robot_config(name)
        out.append(
            {
                "name": name,
                "robot_id": cfg.robot_id,
                "display_name": cfg.name,
                "version": cfg.version,
                "default_servo_backend": cfg.default_servo_backend,
                "default_ik_solver_backend": cfg.default_ik_solver_backend,
                "default_drive_profile": backend_registry.get_default_drive_profile_for_backend(
                    cfg.default_servo_backend
                ),
            }
        )
    return out


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
    'Gradient05Config',
    'get_robot_config',
    'get_robot_name_by_id',
    'list_available_robots',
    'list_robot_metadata',
    'register_robot',
]

