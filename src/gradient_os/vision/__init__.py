"""
Vision module for GradientOS - handles camera operations and computer vision tasks.

This module provides functionality for:
- Pi Camera initialization and configuration
- Image capture and streaming
- Basic computer vision processing
- Camera calibration utilities
"""

from .camera_driver import (
    PiCameraDriver,
    USBCameraDriver,
    available_backends,
    create_camera_driver,
    list_cameras_for_backend,
    BACKEND_AUTO,
    BACKEND_PICAMERA,
    BACKEND_USB,
)
from .image_processor import ImageProcessor
from .cli import main as vision_cli_main

__all__ = [
    'PiCameraDriver',
    'USBCameraDriver',
    'ImageProcessor',
    'vision_cli_main',
    'available_backends',
    'create_camera_driver',
    'list_cameras_for_backend',
    'BACKEND_AUTO',
    'BACKEND_PICAMERA',
    'BACKEND_USB',
]
