# backends/__init__.py
#
# Actuator backend implementations for GradientOS.
# Each subdirectory contains a complete implementation of the ActuatorBackend interface
# for a specific servo/actuator type.
#
# Available backends:
# - feetech: Feetech STS/SCS series serial bus servos
# - simulation: In-memory simulation (no hardware required)
#
# Usage:
# ------
# At startup (in run_controller.py):
#   from gradient_os.arm_controller.backends import registry
#   
#   # 1. Set active backend config
#   registry.set_active_backend("feetech")
#   
#   # 2. Create and initialize backend instance
#   robot_config_dict = selected_robot.get_config_dict()
#   backend = registry.create_backend("feetech", robot_config_dict)
#   backend.initialize()
#   registry.set_active_backend_instance(backend)
#
# In modules that need servo I/O:
#   from gradient_os.arm_controller.backends import registry
#   backend = registry.get_active_backend()
#   backend.sync_write(commands)

import math
import os

from . import registry

# Import backend classes
from .feetech import FeetechBackend
from .ethercat_rtcore import EthercatRTCoreBackend
from .fairino import FairinoBackend
from .simulation import SimulationBackend

# =============================================================================
# Backend Factory Functions
# =============================================================================

def _create_feetech_backend(robot_config: dict, **kwargs) -> FeetechBackend:
    """
    Factory for FeetechBackend that maps RobotConfig dict to FeetechBackend format.
    
    The RobotConfig.get_config_dict() uses different key names than FeetechBackend
    expects, so we need to map them here.
    """
    # Map from RobotConfig keys to FeetechBackend keys
    feetech_config = {
        'servo_ids': robot_config.get('actuator_ids', []),
        'logical_to_physical_map': robot_config.get('logical_to_physical_map', {}),
        'inverted_servo_ids': robot_config.get('inverted_actuator_ids', set()),
        'joint_limits_rad': robot_config.get('logical_joint_limits_rad', []),
        'master_offsets_rad': robot_config.get('logical_joint_master_offsets_rad', []),
        'gripper_servo_id': robot_config.get('gripper_actuator_id'),
        'gripper_limits_rad': robot_config.get('gripper_limits_rad', [0, math.pi]),
        'pid_gains': robot_config.get('actuator_pid_gains', {}),
    }
    
    # Extract kwargs that FeetechBackend accepts
    serial_port = kwargs.get('serial_port', robot_config.get('default_serial_port'))
    baud_rate = kwargs.get('baud_rate')  # None uses FeetechBackend default
    
    return FeetechBackend(
        robot_config=feetech_config,
        serial_port=serial_port,
        baud_rate=baud_rate if baud_rate else 1000000,  # Default to 1M
    )


def _create_simulation_backend(robot_config: dict, **kwargs) -> SimulationBackend:
    """Factory for SimulationBackend that extracts relevant config."""
    return SimulationBackend(
        num_joints=robot_config.get('num_logical_joints', 6),
        has_gripper=robot_config.get('gripper_actuator_id') is not None,
        robot_config=robot_config,  # Pass full config for servo ID mapping
    )

def _create_fairino_backend(robot_config: dict, **kwargs) -> FairinoBackend:
    """
    Factory for FairinoBackend (network client targeting an FR10 controller).

    The FR10 owns its own real-time loop, calibration, and PID, so the bulk
    of `robot_config` is unused here — only `num_logical_joints` is checked.
    All network endpoints come from FairinoBackendConfig.from_env() (see
    backends/fairino/config.py for the list of GRADIENT_FAIRINO_* env vars).
    """
    return FairinoBackend(robot_config=robot_config)


def _create_ethercat_rtcore_backend(robot_config: dict, **kwargs) -> EthercatRTCoreBackend:
    """
    Factory for EthercatRTCoreBackend (RTCore proxy).

    Notes:
    - This backend does not use `serial_port` / `baud_rate`.
    - IPC socket path can be overridden for dev via `GRADIENT_RTCORE_SOCKET_PATH`.
    """
    socket_path = os.environ.get("GRADIENT_RTCORE_SOCKET_PATH", "/run/gradient-rt-motion/ipc.sock")
    return EthercatRTCoreBackend(
        robot_config=robot_config,
        socket_path=socket_path,
    )


# =============================================================================
# Register available backends
# =============================================================================

# Feetech STS/SCS series servos
registry.register_backend_class(
    name="feetech",
    factory=_create_feetech_backend,
    config_module_path="gradient_os.arm_controller.backends.feetech.config",
)

# In-memory simulation (no hardware)
registry.register_backend_class(
    name="simulation",
    factory=_create_simulation_backend,
    config_module_path=None,  # Simulation uses feetech config as fallback
)

# EtherCAT RTCore proxy backend
registry.register_backend_class(
    name="ethercat_rtcore",
    factory=_create_ethercat_rtcore_backend,
    config_module_path="gradient_os.arm_controller.backends.ethercat_rtcore.config",
)

# Fairino FR-series network client (FR10)
registry.register_backend_class(
    name="fairino",
    factory=_create_fairino_backend,
    config_module_path="gradient_os.arm_controller.backends.fairino.config",
)

__all__ = ['FeetechBackend', 'SimulationBackend', 'EthercatRTCoreBackend', 'FairinoBackend', 'registry']
