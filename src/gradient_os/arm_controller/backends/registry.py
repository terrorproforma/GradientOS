# backends/registry.py
#
# Global registry for the active servo backend.
# This module manages both the backend CONFIGURATION (constants, register addresses)
# and the backend INSTANCE (the actual ActuatorBackend object that does I/O).
#
# Usage:
# ------
# At startup (in run_controller.py):
#   from gradient_os.arm_controller.backends import registry
#   
#   # 1. Set active backend (loads config module)
#   registry.set_active_backend("feetech")
#   
#   # 2. Create and initialize backend instance
#   backend = registry.create_backend("feetech", robot_config_dict)
#   backend.initialize()
#   registry.set_active_backend_instance(backend)
#
# In any module that needs to perform servo I/O:
#   from gradient_os.arm_controller.backends import registry
#   backend = registry.get_active_backend()
#   backend.sync_write(commands)
#
# In any module that needs config values only:
#   from gradient_os.arm_controller.backends import registry
#   encoder_resolution = registry.get_encoder_resolution()

from typing import Optional, TYPE_CHECKING, Type, Callable
import importlib

if TYPE_CHECKING:
    from types import ModuleType
    from ..actuator_interface import ActuatorBackend

# =============================================================================
# Module State
# =============================================================================

# The active backend name and config module
_active_backend_name: Optional[str] = None
_active_backend_config: Optional['ModuleType'] = None

# The active backend INSTANCE (for I/O operations)
_active_backend_instance: Optional['ActuatorBackend'] = None

# Mapping of backend names to their config module paths
BACKEND_CONFIG_MODULES: dict[str, str] = {
    "feetech": "gradient_os.arm_controller.backends.feetech.config",
    # Add more backends here as they're implemented:
    # "dynamixel": "gradient_os.arm_controller.backends.dynamixel.config",
    # "simulation": "gradient_os.arm_controller.backends.simulation.config",
}

# Mapping of backend names to their backend class factory functions
# Each factory takes a robot_config dict and returns an ActuatorBackend instance
BACKEND_CLASSES: dict[str, Callable[..., 'ActuatorBackend']] = {}


# =============================================================================
# Exceptions
# =============================================================================

class BackendNotConfiguredError(Exception):
    """Raised when trying to access backend config before it's been set."""
    pass


class BackendInstanceNotSetError(Exception):
    """Raised when trying to access backend instance before it's been created."""
    pass


def register_backend_class(
    name: str, 
    factory: Callable[..., 'ActuatorBackend'],
    config_module_path: Optional[str] = None,
) -> None:
    """
    Register a backend class with the registry.
    
    This is called from backends/__init__.py to register available backends.
    
    Args:
        name: Backend name (e.g., "feetech", "simulation")
        factory: Callable that creates an ActuatorBackend instance.
                 Signature: factory(robot_config: dict, **kwargs) -> ActuatorBackend
        config_module_path: Optional path to config module (for backends with config)
    """
    BACKEND_CLASSES[name] = factory
    if config_module_path:
        BACKEND_CONFIG_MODULES[name] = config_module_path
    print(f"[Backend Registry] Registered backend: {name}")


def set_active_backend(backend_name: str) -> None:
    """
    Set the active servo backend (loads config module).
    
    This should be called once at startup (e.g., in run_controller.py) before
    any modules try to access backend configuration.
    
    Note: This only sets the CONFIG. To create a backend INSTANCE, call
    create_backend() and set_active_backend_instance().
    
    Args:
        backend_name: Name of the backend (e.g., "feetech", "dynamixel")
    
    Raises:
        ValueError: If the backend name is not recognized
    """
    global _active_backend_name, _active_backend_config
    
    if backend_name not in BACKEND_CONFIG_MODULES:
        available = ", ".join(BACKEND_CONFIG_MODULES.keys())
        raise ValueError(f"Unknown servo backend '{backend_name}'. Available: {available}")
    
    module_path = BACKEND_CONFIG_MODULES[backend_name]
    _active_backend_config = importlib.import_module(module_path)
    _active_backend_name = backend_name
    
    print(f"[Backend Registry] Active servo backend set to: {backend_name}")


def get_active_backend_name() -> str:
    """
    Get the name of the active servo backend.
    
    Returns:
        str: Backend name (e.g., "feetech")
    
    Raises:
        BackendNotConfiguredError: If no backend has been set
    """
    if _active_backend_name is None:
        raise BackendNotConfiguredError(
            "No servo backend configured. Call set_active_backend() at startup."
        )
    return _active_backend_name


def get_config() -> 'ModuleType':
    """
    Get the active backend's config module.
    
    Returns:
        ModuleType: The backend config module (e.g., backends.feetech.config)
    
    Raises:
        BackendNotConfiguredError: If no backend has been set
    """
    if _active_backend_config is None:
        raise BackendNotConfiguredError(
            "No servo backend configured. Call set_active_backend() at startup."
        )
    return _active_backend_config


# =============================================================================
# Convenience functions for commonly accessed config values
# =============================================================================

def get_encoder_resolution() -> int:
    """Get the encoder resolution (max value) for the active backend."""
    return get_config().SERVO_VALUE_MAX


def get_encoder_center() -> int:
    """Get the encoder center value for the active backend."""
    return get_config().SERVO_VALUE_CENTER


def get_default_baud_rate() -> int:
    """Get the default baud rate for the active backend."""
    return get_config().DEFAULT_BAUD_RATE


def get_default_pid_gains() -> tuple[int, int, int]:
    """Get the default PID gains (Kp, Ki, Kd) for the active backend."""
    cfg = get_config()
    return (cfg.DEFAULT_KP, cfg.DEFAULT_KI, cfg.DEFAULT_KD)


def get_serial_read_timeout() -> float:
    """Get the serial read timeout for the active backend."""
    return get_config().SERIAL_READ_TIMEOUT


def is_configured() -> bool:
    """Check if a backend CONFIG has been set."""
    return _active_backend_config is not None


def is_instance_set() -> bool:
    """Check if a backend INSTANCE has been created and set."""
    return _active_backend_instance is not None


def list_available_backends() -> list[str]:
    """List all available backend names."""
    # Return union of config modules and registered classes
    return list(set(BACKEND_CONFIG_MODULES.keys()) | set(BACKEND_CLASSES.keys()))


# =============================================================================
# Backend Instance Management
# =============================================================================

def create_backend(
    backend_name: str,
    robot_config: dict,
    serial_port: Optional[str] = None,
    baud_rate: Optional[int] = None,
) -> 'ActuatorBackend':
    """
    Create a new backend instance for the specified backend type.
    
    This creates but does NOT set the active instance. Call set_active_backend_instance()
    to make it the active backend for all modules.
    
    Args:
        backend_name: Name of the backend (e.g., "feetech", "simulation")
        robot_config: Dictionary containing robot configuration:
            - servo_ids: List of physical servo IDs
            - logical_to_physical_map: Dict mapping logical joint to physical servo indices
            - inverted_servo_ids: Set of servo IDs using inverted mapping
            - joint_limits_rad: List of [min, max] limits per logical joint
            - gripper_servo_id: Optional gripper servo ID
            - pid_gains: Optional dict mapping servo_id to (kp, ki, kd)
        serial_port: Optional serial port path (for hardware backends)
        baud_rate: Optional baud rate (for hardware backends)
    
    Returns:
        ActuatorBackend: The created backend instance (not yet initialized)
    
    Raises:
        ValueError: If the backend name is not registered
    """
    if backend_name not in BACKEND_CLASSES:
        available = ", ".join(BACKEND_CLASSES.keys())
        raise ValueError(
            f"Unknown backend class '{backend_name}'. "
            f"Available: {available}. "
            f"Make sure the backend is registered in backends/__init__.py"
        )
    
    factory = BACKEND_CLASSES[backend_name]
    
    # Build kwargs based on backend type
    kwargs = {"robot_config": robot_config}
    if serial_port is not None:
        kwargs["serial_port"] = serial_port
    if baud_rate is not None:
        kwargs["baud_rate"] = baud_rate
    
    backend = factory(**kwargs)
    print(f"[Backend Registry] Created {backend_name} backend instance")
    return backend


def set_active_backend_instance(backend: 'ActuatorBackend') -> None:
    """
    Set the active backend instance.
    
    This makes the given backend available to all modules via get_active_backend().
    The backend should already be initialized before calling this.
    
    Args:
        backend: The ActuatorBackend instance to make active
    """
    global _active_backend_instance
    _active_backend_instance = backend
    print(f"[Backend Registry] Active backend instance set: {type(backend).__name__}")


def get_active_backend() -> 'ActuatorBackend':
    """
    Get the active backend instance.
    
    Use this to perform servo I/O operations like sync_write, sync_read, etc.
    
    Returns:
        ActuatorBackend: The active backend instance
    
    Raises:
        BackendInstanceNotSetError: If no backend instance has been set
    
    Example:
        backend = registry.get_active_backend()
        backend.sync_write([(10, 2048, 100, 0), (20, 2048, 100, 0)])
    """
    if _active_backend_instance is None:
        raise BackendInstanceNotSetError(
            "No backend instance set. Call create_backend() and "
            "set_active_backend_instance() at startup in run_controller.py"
        )
    return _active_backend_instance


def shutdown_backend() -> None:
    """
    Shutdown and clear the active backend instance.
    
    This should be called during cleanup (e.g., when run_controller exits).
    """
    global _active_backend_instance
    if _active_backend_instance is not None:
        try:
            _active_backend_instance.shutdown()
        except Exception as e:
            print(f"[Backend Registry] Error during backend shutdown: {e}")
        _active_backend_instance = None
        print("[Backend Registry] Backend instance cleared")


# =============================================================================
# Telemetry functions
# =============================================================================

def get_telemetry_blocks() -> list[tuple[int, int]]:
    """
    Get the telemetry register blocks for the active backend.
    
    Returns:
        list[tuple[int, int]]: List of (start_address, length) tuples
    """
    cfg = get_config()
    blocks: list[tuple[int, int]] = []
    for block_index in range(1, 4):
        addr = getattr(cfg, f"TELEMETRY_BLOCK{block_index}_ADDRESS", None)
        length = getattr(cfg, f"TELEMETRY_BLOCK{block_index}_LENGTH", None)
        if addr is None or length is None:
            return []
        blocks.append((int(addr), int(length)))
    return blocks


def parse_telemetry_block(block_index: int, data: bytes) -> dict:
    """
    Parse telemetry data for a specific block.
    
    Args:
        block_index: Which block (0, 1, or 2) to parse
        data: Raw bytes from the block read
    
    Returns:
        dict: Parsed telemetry data
    """
    cfg = get_config()

    parser = getattr(cfg, f"parse_telemetry_block{block_index + 1}", None)
    if callable(parser):
        return parser(data)

    return {}

