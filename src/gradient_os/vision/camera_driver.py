"""
Pi Camera driver for GradientOS.

This module provides a high-level interface for controlling Raspberry Pi cameras,
including initialization, configuration, image capture, and video streaming.
"""

import time
import threading
from typing import Optional, Tuple, Callable, List, Dict
import logging

try:
    from picamera2 import Picamera2
    from libcamera import controls
    PICAMERA2_AVAILABLE = True
except ImportError:
    PICAMERA2_AVAILABLE = False
    print("Warning: picamera2 not available. Install with: pip install picamera2")

try:
    import cv2
    OPENCV_AVAILABLE = True
except ImportError:
    OPENCV_AVAILABLE = False
    print("Warning: OpenCV not available. Install with: pip install opencv-python")

# USB cameras are accessed through OpenCV, so stay disabled if OpenCV import failed.
USB_BACKEND_AVAILABLE = OPENCV_AVAILABLE

# Canonical names exposed to the CLI/factory helpers.
BACKEND_PICAMERA = "picamera"
BACKEND_USB = "usb"
BACKEND_AUTO = "auto"

logger = logging.getLogger(__name__)


class PiCameraDriver:
    """
    Driver class for Raspberry Pi Camera operations.

    This class provides methods for:
    - Camera initialization and configuration
    - Image capture (still images and video)
    - Camera parameter adjustment
    - Streaming capabilities
    """

    def __init__(self, camera_id: int = 0, resolution: Tuple[int, int] = (640, 480), framerate: int = 30):
        """
        Initialize the Pi Camera driver.

        Args:
            camera_id (int): Camera device ID (0 for default camera)
            resolution (Tuple[int, int]): Camera resolution as (width, height)
            framerate (int): Camera framerate in FPS
        """
        self.camera_id = camera_id
        self.resolution = resolution
        self.framerate = framerate

        self.camera: Optional[Picamera2] = None
        self.is_streaming = False
        self.stream_thread: Optional[threading.Thread] = None
        self.stream_callback: Optional[Callable] = None

        if not PICAMERA2_AVAILABLE:
            raise ImportError("picamera2 is required for Pi Camera functionality")

        logger.info(f"PiCameraDriver initialized with camera {camera_id}, resolution {resolution}, framerate {framerate}")

    @staticmethod
    def list_cameras() -> List[Dict]:
        """
        Return information about all detected cameras.

        Returns:
            list[dict]: List of camera info dictionaries as reported by Picamera2
        """
        if not PICAMERA2_AVAILABLE:
            return []

        try:
            return Picamera2.global_camera_info()
        except Exception as e:
            logger.error(f"Failed to enumerate cameras: {e}")
            return []

    def initialize(self) -> bool:
        """
        Initialize the camera hardware and configure settings.

        Returns:
            bool: True if initialization successful, False otherwise
        """
        try:
            # Validate that at least one camera is present
            available_cameras = self.list_cameras()
            if not available_cameras:
                logger.error("No cameras detected by libcamera/Picamera2. Check cabling, power, and overlays.")
                return False

            if self.camera_id < 0 or self.camera_id >= len(available_cameras):
                logger.error(f"Requested camera_id {self.camera_id} is out of range. "
                             f"Detected cameras: {len(available_cameras)}")
                for idx, info in enumerate(available_cameras):
                    logger.error(f" - [{idx}] {info}")
                return False

            self.camera = Picamera2(self.camera_id)

            # Configure camera settings
            config = self.camera.create_still_configuration(
                main={"size": self.resolution, "format": "RGB888"},
                controls={"FrameRate": self.framerate}
            )

            self.camera.configure(config)
            self.camera.start()

            logger.info("Camera initialized successfully")
            return True

        except Exception as e:
            logger.error(f"Failed to initialize camera: {e}")
            return False

    def capture_image(self) -> Optional['numpy.ndarray']:
        """
        Capture a single image from the camera.

        Returns:
            numpy.ndarray: Captured image as numpy array, or None if capture failed
        """
        if not self.camera:
            logger.error("Camera not initialized")
            return None

        try:
            # Capture image
            image = self.camera.capture_array()

            logger.debug("Image captured successfully")
            return image

        except Exception as e:
            logger.error(f"Failed to capture image: {e}")
            return None

    def set_exposure(self, exposure_time_us: int) -> bool:
        """
        Set camera exposure time.

        Args:
            exposure_time_us (int): Exposure time in microseconds

        Returns:
            bool: True if setting applied successfully
        """
        if not self.camera:
            return False

        try:
            self.camera.set_controls({"ExposureTime": exposure_time_us})
            logger.info(f"Exposure time set to {exposure_time_us} μs")
            return True
        except Exception as e:
            logger.error(f"Failed to set exposure: {e}")
            return False

    def set_gain(self, analogue_gain: float) -> bool:
        """
        Set camera analogue gain.

        Args:
            analogue_gain (float): Analogue gain value

        Returns:
            bool: True if setting applied successfully
        """
        if not self.camera:
            return False

        try:
            self.camera.set_controls({"AnalogueGain": analogue_gain})
            logger.info(f"Analogue gain set to {analogue_gain}")
            return True
        except Exception as e:
            logger.error(f"Failed to set gain: {e}")
            return False

    def start_streaming(self, callback: Callable[['numpy.ndarray'], None]) -> bool:
        """
        Start streaming camera frames to a callback function.

        Args:
            callback: Function to call with each frame (receives numpy array)

        Returns:
            bool: True if streaming started successfully
        """
        if not self.camera or self.is_streaming:
            return False

        self.stream_callback = callback
        self.is_streaming = True

        self.stream_thread = threading.Thread(target=self._streaming_loop, daemon=True)
        self.stream_thread.start()

        logger.info("Camera streaming started")
        return True

    def stop_streaming(self) -> bool:
        """
        Stop camera streaming.

        Returns:
            bool: True if streaming stopped successfully
        """
        if not self.is_streaming:
            return False

        self.is_streaming = False

        if self.stream_thread:
            self.stream_thread.join(timeout=1.0)

        logger.info("Camera streaming stopped")
        return True

    def _streaming_loop(self):
        """Internal streaming loop that captures and sends frames to callback."""
        while self.is_streaming and self.camera:
            try:
                frame = self.camera.capture_array()
                if self.stream_callback:
                    self.stream_callback(frame)
                time.sleep(1.0 / self.framerate)  # Control frame rate
            except Exception as e:
                logger.error(f"Error in streaming loop: {e}")
                break

    def get_camera_info(self) -> dict:
        """
        Get information about the camera.

        Returns:
            dict: Camera information dictionary
        """
        info = {
            "camera_id": self.camera_id,
            "resolution": self.resolution,
            "framerate": self.framerate,
            "initialized": self.camera is not None,
            "streaming": self.is_streaming,
            "picamera2_available": PICAMERA2_AVAILABLE,
            "opencv_available": OPENCV_AVAILABLE
        }
        return info

    def close(self):
        """Close the camera and clean up resources."""
        self.stop_streaming()

        if self.camera:
            try:
                self.camera.stop()
                self.camera.close()
                self.camera = None
                logger.info("Camera closed successfully")
            except Exception as e:
                logger.error(f"Error closing camera: {e}")


class USBCameraDriver:
    """
    USB camera driver that relies on OpenCV's VideoCapture API.

    Works on macOS, Linux, and Raspberry Pi for UVC-compatible cameras.
    """

    def __init__(self, camera_id: int = 0, resolution: Tuple[int, int] = (640, 480), framerate: int = 30):
        if not USB_BACKEND_AVAILABLE:
            raise ImportError("OpenCV is required for USB camera functionality. Install with: pip install opencv-python")

        self.camera_id = camera_id
        self.resolution = resolution
        self.framerate = framerate

        self.cap: Optional['cv2.VideoCapture'] = None
        self.is_streaming = False
        self.stream_thread: Optional[threading.Thread] = None
        self.stream_callback: Optional[Callable[['numpy.ndarray'], None]] = None

        logger.info(f"USBCameraDriver initialized with device {camera_id}, resolution {resolution}, framerate {framerate}")

    @staticmethod
    def list_cameras(max_devices: int = 8) -> List[Dict]:
        """
        Enumerate USB cameras by probing the first N device indices.
        """
        if not USB_BACKEND_AVAILABLE:
            return []

        devices: List[Dict] = []
        for idx in range(max_devices):
            cap = cv2.VideoCapture(idx, cv2.CAP_ANY)
            if not cap or not cap.isOpened():
                if cap:
                    cap.release()
                continue

            width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)) or None
            height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT)) or None
            fps = cap.get(cv2.CAP_PROP_FPS)
            if fps and fps > 1200:  # Some drivers report absurd defaults; normalize to None
                fps = None

            devices.append(
                {
                    "id": idx,
                    "name": f"USB Camera {idx}",
                    "resolution": (width, height) if width and height else None,
                    "fps": fps if fps and fps > 0 else None,
                }
            )
            cap.release()
        return devices

    def initialize(self) -> bool:
        """
        Open the USB camera via OpenCV.
        """
        try:
            self.cap = cv2.VideoCapture(self.camera_id, cv2.CAP_ANY)
            if not self.cap or not self.cap.isOpened():
                logger.error(f"Failed to open USB camera index {self.camera_id}")
                self.cap = None
                return False

            width, height = self.resolution
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
            self.cap.set(cv2.CAP_PROP_FPS, self.framerate)

            # Warm-up grab so sensors that need settling start delivering frames.
            self.cap.read()

            logger.info("USB camera initialized successfully")
            return True
        except Exception as exc:
            logger.error(f"USB camera initialization error: {exc}")
            if self.cap:
                self.cap.release()
                self.cap = None
            return False

    def capture_image(self) -> Optional['numpy.ndarray']:
        if not self.cap or not self.cap.isOpened():
            logger.error("USB camera not initialized")
            return None

        try:
            ret, frame = self.cap.read()
            if not ret:
                logger.error("Failed to grab frame from USB camera")
                return None
            return frame
        except Exception as exc:
            logger.error(f"Failed to capture USB camera frame: {exc}")
            return None

    def start_streaming(self, callback: Callable[['numpy.ndarray'], None]) -> bool:
        if not self.cap or not self.cap.isOpened() or self.is_streaming:
            return False

        self.stream_callback = callback
        self.is_streaming = True
        self.stream_thread = threading.Thread(target=self._streaming_loop, daemon=True)
        self.stream_thread.start()
        logger.info("USB camera streaming started")
        return True

    def _streaming_loop(self) -> None:
        poll_delay = 1.0 / max(1, int(self.framerate))
        while self.is_streaming and self.cap and self.cap.isOpened():
            try:
                ret, frame = self.cap.read()
                if not ret:
                    time.sleep(poll_delay)
                    continue
                if self.stream_callback:
                    self.stream_callback(frame)
            except Exception as exc:
                logger.error(f"Error in USB streaming loop: {exc}")
                break
            time.sleep(0.0 if poll_delay < 0.005 else poll_delay / 4)

    def stop_streaming(self) -> bool:
        if not self.is_streaming:
            return False
        self.is_streaming = False
        if self.stream_thread:
            self.stream_thread.join(timeout=1.0)
        logger.info("USB camera streaming stopped")
        return True

    def get_camera_info(self) -> dict:
        info = {
            "camera_id": self.camera_id,
            "resolution": self.resolution,
            "framerate": self.framerate,
            "initialized": bool(self.cap and self.cap.isOpened()),
            "streaming": self.is_streaming,
            "backend": BACKEND_USB,
            "opencv_available": OPENCV_AVAILABLE,
        }
        return info

    def close(self) -> None:
        self.stop_streaming()
        if self.cap:
            try:
                self.cap.release()
            except Exception as exc:
                logger.error(f"Error releasing USB camera: {exc}")
            finally:
                self.cap = None
                logger.info("USB camera released")


def available_backends() -> List[str]:
    """Return a list of camera backends currently available on this system."""
    backends: List[str] = []
    if PICAMERA2_AVAILABLE:
        backends.append(BACKEND_PICAMERA)
    if USB_BACKEND_AVAILABLE:
        backends.append(BACKEND_USB)
    return backends


def is_backend_available(backend: str) -> bool:
    """Check if a specific backend can be used."""
    if backend == BACKEND_PICAMERA:
        return PICAMERA2_AVAILABLE
    if backend == BACKEND_USB:
        return USB_BACKEND_AVAILABLE
    return False


def resolve_backend(preferred: Optional[str]) -> str:
    """
    Resolve a preferred backend into a concrete backend or raise if none is usable.

    Args:
        preferred: Requested backend name (picamera/usb/auto/None)
    """
    if not preferred or preferred == BACKEND_AUTO:
        for candidate in (BACKEND_PICAMERA, BACKEND_USB):
            if is_backend_available(candidate):
                return candidate
        raise RuntimeError("No camera backend available. Install picamera2 or opencv-python.")

    if not is_backend_available(preferred):
        raise RuntimeError(f"Requested backend '{preferred}' is not available. "
                           "Install the required dependencies or choose --backend auto.")

    return preferred


def list_cameras_for_backend(backend: Optional[str], max_devices: int = 8) -> List[Dict]:
    """List cameras for the resolved backend."""
    backend_name = resolve_backend(backend)
    if backend_name == BACKEND_PICAMERA:
        try:
            return PiCameraDriver.list_cameras()
        except Exception as exc:
            logger.error(f"Failed to list Pi cameras: {exc}")
            return []
    if backend_name == BACKEND_USB:
        return USBCameraDriver.list_cameras(max_devices=max_devices)
    return []


def create_camera_driver(backend: Optional[str], **kwargs):
    """Factory that instantiates the appropriate camera driver implementation."""
    backend_name = resolve_backend(backend)
    if backend_name == BACKEND_PICAMERA:
        return PiCameraDriver(**kwargs)
    if backend_name == BACKEND_USB:
        return USBCameraDriver(**kwargs)
    raise RuntimeError(f"Unsupported backend '{backend_name}'")
