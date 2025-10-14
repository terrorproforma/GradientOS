# GradientOS Vision Module

This module provides camera functionality and computer vision capabilities for the GradientOS robotic system.

## Features

- **Pi Camera Driver**: Full control of Raspberry Pi cameras with picamera2
- **Image Processing**: OpenCV-based image processing and computer vision
- **Object Detection**: Color-based object detection and tracking
- **Real-time Streaming**: Live camera streaming with callback support
- **Camera Calibration**: Utilities for camera parameter adjustment

## Installation

### System Dependencies

First, install the required system packages for Raspberry Pi camera support:

```bash
sudo apt update
sudo apt install -y python3-libcamera python3-kms++
```

These packages provide the low-level camera drivers and Python bindings that picamera2 depends on.

### Python Dependencies

Install the Python packages:

```bash
# Activate virtual environment with camera support
cd /path/to/GradientOS
source ./start.sh

# Or manually activate and set PYTHONPATH
source .venv/bin/activate
export PYTHONPATH="/usr/lib/python3/dist-packages:$(pwd)/src:$PYTHONPATH"

# Install GradientOS (installs Python dependencies)
pip install -e .
```

#### Optional: AI/YOLO Dependencies

To enable YOLO11 AI detection overlays in the MJPEG server, install the optional AI extra (platform-specific wheels may still be required):

```bash
pip install -e '.[ai]'
```

Notes:
- On Raspberry Pi/ARM, prefer vendor-provided wheels or PyTorch’s official install selector for your Python/architecture.
- Start with `yolo11n.pt`, `--imgsz 640`, and CPU for best chance of real-time on Pi.

### Environment Setup

For camera functionality to work properly, you need to either:

**Option 1: Use the activation script (recommended)**
```bash
cd /path/to/GradientOS
source ./start.sh
```

**Option 2: Manual environment setup**
```bash
cd /path/to/GradientOS
source .venv/bin/activate
export PYTHONPATH="/usr/lib/python3/dist-packages:$(pwd)/src:$PYTHONPATH"
```

### Installation Order

Always install in this order:

1. **System dependencies first**: `sudo apt install -y python3-libcamera python3-kms++`
2. **Virtual environment**: Create/activate your Python virtual environment
3. **Python dependencies**: `pip install -e .` *(append `.[ai]` if you want the YOLO stack)*
4. **Environment setup**: Use activation script or set PYTHONPATH

Key dependencies:
- `picamera2==0.3.30`: For Raspberry Pi camera control
- `opencv-python==4.10.0.84`: For computer vision functionality
- System packages: `python3-libcamera`, `python3-kms++`

## Quick Start

### Command-line Usage

After activating your environment, install the package in editable mode to register console scripts:

```bash
cd .
pip install -e .
```


Run the vision CLI (you can also use the module form shown below). With no subcommand, it defaults to streaming using full-sensor friendly 4:3 defaults (camera 0, 1280x960 @ 30 FPS) to maximize FOV:

```bash
gradient-vision            # Streams from camera 0 at 1280x960@30 (4:3 for full sensor FOV)


# List available cameras
gradient-vision list

# Test camera initialization and capture a single frame
gradient-vision init --camera 0 --width 640 --height 480 --fps 30

# Run basic image processing checks
gradient-vision processing --camera 0 --width 640 --height 480 --fps 30

# Stream frames and print FPS (10 seconds)
gradient-vision stream --camera 0 --width 640 --height 480 --fps 30 --duration 10
# MJPEG HTTP server (auto-dual if two cameras detected; stacked vertically)
gradient-vision mjpeg

  # Defaults: host=0.0.0.0, port=8080, width=1280, height=960, fps=30, jpeg_quality=80

  # Visit http://<host>:8080/  (cam0 and cam1 vertically stacked if both present)
  gradient-vision mjpeg --vflip --hflip

# MJPEG with image processing overlay (e.g., color-based object detection)
gradient-vision mjpeg img-proc --object-detection --color yellow --vflip --hflip
  # Overlay text shows which color is detected, HSV range, min_area, and count

# MJPEG with Ultralytics AI detection overlay (generic)

gradient-vision mjpeg ai --weights <ultralytics-weights> --conf 0.25 --imgsz 640 --device cpu --vflip --hflip

  # Works with most Ultralytics weights via a unified API. Examples:
  #  - YOLO11 detection: --weights yolo11n.pt
  #  - YOLO11 segmentation: use subcommand ai-seg and seg weights (yolo11n-seg.pt)
  #  - YOLO11 pose: use subcommand ai-pose and pose weights (yolo11n-pose.pt)
  #  - RT-DETR: --weights rtdetr-l.pt (auto-fallback to RTDETR loader)

Notes:
- The loader first tries Ultralytics `YOLO(...)` for most weights; if that fails it falls back to `RTDETR(...)` automatically. This covers the common Ultralytics families (YOLO*, -seg, -pose, OBB, and RT-DETR) under the same CLI.
- Some specialized models may require additional classes; open an issue if you hit one.
- RT-DETR docs for reference: `https://docs.ultralytics.com/models/rtdetr/#pre-trained-models`
  # Optional: --classes "0,1,2" to restrict classes, --vflip/--hflip

# MJPEG with YOLO segmentation overlay (requires yolo11*-seg.pt)
gradient-vision mjpeg ai-seg --weights yolo11n-seg.pt --conf 0.25 --imgsz 640 --device cpu

# MJPEG with YOLO pose overlay (requires yolo11*-pose.pt)
gradient-vision mjpeg ai-pose --weights yolo11n-pose.pt --conf 0.25 --imgsz 640 --device cpu


# Enable verbose logs with -v
gradient-vision list -v
```

### Using cameras with the telemetry recorder

You can record synchronized frames and robot state via `record_episode.py`. If your camera MJPEG endpoints are not already running, the recorder can auto-start the MJPEG server.

Default behavior: cameras ON (auto-start MJPEG, dual endpoints if available)

```bash
python -m gradient_os.telemetry.record_episode
# Uses http://127.0.0.1:8080/cam0.mjpg and /cam1.mjpg if available, otherwise falls back gracefully.
```

Single camera (auto-start MJPEG on localhost and record):

```bash
python -m gradient_os.telemetry.record_episode \
  --auto-start-mjpeg \
  --mjpeg-port 8080 \
  --fps 10 \
  --resize 256
# The recorder will read frames from http://127.0.0.1:8080/stream.mjpg
```

Dual cameras (auto-start dual MJPEG; recorder uses cam0 and cam1):

```bash
python -m gradient_os.telemetry.record_episode \
  --auto-start-mjpeg \
  --mjpeg-port 8080 \
  --mjpeg-both \
  --fps 10 \
  --resize 256
# The recorder will read cam0 from /cam0.mjpg and cam1 from /cam1.mjpg
```

If you already have an MJPEG server running, pass the URLs directly:

```bash
python -m gradient_os.telemetry.record_episode \
  --base-cam http://127.0.0.1:8080/stream.mjpg \
  --fps 10 --resize 256

# Dual:
python -m gradient_os.telemetry.record_episode \
  --base-cam http://127.0.0.1:8080/cam0.mjpg \
  --wrist-cam http://127.0.0.1:8080/cam1.mjpg \
  --fps 10 --resize 256
```

Disable cameras (e.g., state-only recording):

```bash
python -m gradient_os.telemetry.record_episode --no-cameras --fps 10
```

If you prefer not to install console scripts, you can run the module directly. With no args it streams using the same defaults:

```bash
python -m gradient_os.vision         # Streams using defaults (1280x960@30, 4:3)

python -m gradient_os.vision list
python -m gradient_os.vision init --camera 0 --width 640 --height 480 --fps 30
python -m gradient_os.vision processing --camera 0 --width 640 --height 480 --fps 30
python -m gradient_os.vision stream --camera 0 --width 640 --height 480 --fps 30 --duration 10
```

### Basic Camera Usage

```python
from gradient_os.vision import PiCameraDriver

# Initialize camera
camera = PiCameraDriver(resolution=(640, 480), framerate=30)
camera.initialize()

# Capture an image
image = camera.capture_image()

# Clean up
camera.close()
```

### Image Processing

```python
from gradient_os.vision import ImageProcessor

processor = ImageProcessor()

# Preprocess image
processed = processor.preprocess_image(image, resize=(320, 240), blur=3)

# Detect objects by color (e.g., red objects)
lower_red = (0, 50, 50)
upper_red = (10, 255, 255)
objects = processor.detect_objects_by_color(image, lower_red, upper_red)

# Draw bounding boxes
result = processor.draw_bounding_boxes(image, objects)
```

### Real-time Streaming

```python
def frame_callback(frame):
    # Process each frame
    print(f"Received frame: {frame.shape}")

camera = PiCameraDriver()
camera.initialize()
camera.start_streaming(frame_callback)

# Stream for some time...
time.sleep(10)

camera.stop_streaming()
camera.close()
```

## Camera Configuration

### Adjusting Camera Parameters

```python
camera = PiCameraDriver()
camera.initialize()

# Set exposure
camera.set_exposure(10000)  # 10ms exposure

# Set gain
camera.set_gain(2.0)  # 2x analogue gain

# Get camera information
info = camera.get_camera_info()
print(info)
```

### Supported Resolutions

- 640x480 (VGA) - Default
- 1280x720 (HD)
- 1920x1080 (Full HD)
- 3280x2464 (Max for Pi Camera v2)

## Object Detection

### Color-based Detection

The module supports detection of objects based on HSV color ranges:

```python
# Define color ranges
colors = {
    'red': ((0, 50, 50), (10, 255, 255)),
    'green': ((40, 50, 50), (80, 255, 255)),
    'blue': ((90, 50, 50), (130, 255, 255))
}

for color_name, (lower, upper) in colors.items():
    objects = processor.detect_objects_by_color(image, lower, upper)
    print(f"Found {len(objects)} {color_name} objects")
```

### Finding Specific Objects

```python
objects = processor.detect_objects_by_color(image, lower_bound, upper_bound)

# Find largest object
largest = processor.find_largest_object(objects)
if largest:
    print(f"Center: ({largest['center_x']}, {largest['center_y']})")
    print(f"Size: {largest['width']}x{largest['height']}")
```

## Image Processing Operations

### Edge Detection

```python
edges = processor.detect_edges(image, threshold1=50, threshold2=150)
```

### Thresholding

```python
thresh, _ = processor.apply_threshold(image, threshold=127)
```

### Color Space Conversion

```python
gray = processor.convert_to_grayscale(image)
```

## Testing

First, ensure your environment is properly set up:

```bash
cd /path/to/GradientOS
source ./start.sh
```

Then run the test script to verify camera functionality:

```bash
python scripts/test_camera.py
```

Or run specific tests:

```bash
# Test camera initialization
python scripts/test_camera.py --test init

# Test image processing
python scripts/test_camera.py --test processing

# Test streaming
python scripts/test_camera.py --test streaming
```

### Test Results

The test script will check:
- ✅ **PiCamera2 availability**: Confirms picamera2 can import libcamera
- ✅ **OpenCV availability**: Confirms computer vision libraries work
- ✅ **Camera initialization**: Tests actual camera hardware access
- ✅ **Image capture**: Verifies image capture functionality
- ✅ **Image processing**: Tests computer vision operations
- ✅ **Real-time streaming**: Tests live camera streaming

## Hardware Setup

### Raspberry Pi Camera Connection

1. Ensure the camera is properly connected to the Raspberry Pi's CSI connector
2. Enable the camera interface in raspi-config:
   ```bash
   sudo raspi-config
   # Navigate to Interfacing Options > Camera > Enable
   ```

3. Test camera with `libcamera-hello`:
   ```bash
   libcamera-hello
   ```

### Camera Compatibility

- Raspberry Pi Camera Module v1
- Raspberry Pi Camera Module v2
- Raspberry Pi Camera Module v3
- Raspberry Pi High Quality Camera

## Integration with Robot Controller

The vision module can be integrated with the arm controller for applications like:

- **Visual Servoing**: Using camera feedback for precise positioning
- **Object Detection**: Finding objects for pick-and-place operations
- **Calibration**: Camera calibration for accurate coordinate transformation
- **Inspection**: Quality control and defect detection

Example integration:

```python
from gradient_os.arm_controller import command_api
from gradient_os.vision import PiCameraDriver, ImageProcessor

# Initialize systems
camera = PiCameraDriver()
processor = ImageProcessor()

camera.initialize()

# Capture and process image
image = camera.capture_image()
objects = processor.detect_objects_by_color(image, lower_bound, upper_bound)

# Use detected objects for robot control
if objects:
    target = processor.find_largest_object(objects)
    # Move arm to object position...
    command_api.move_to_position(target['center_x'], target['center_y'])

camera.close()
```

## Troubleshooting

### Common Issues

1. **Camera not found**: Check camera connection and enable CSI interface
2. **Import errors with libcamera**: Make sure system dependencies are installed:
   ```bash
   sudo apt install -y python3-libcamera python3-kms++
   ```
3. **ModuleNotFoundError: No module named 'libcamera'**: Ensure PYTHONPATH is set correctly:
   ```bash
   export PYTHONPATH="/usr/lib/python3/dist-packages:$PYTHONPATH"
   ```
4. **Virtual environment issues**: Use the activation script or set PYTHONPATH manually
5. **Low frame rate**: Reduce resolution or adjust camera parameters
6. **Poor image quality**: Adjust exposure, gain, and lighting conditions

### Performance Tips

- Use lower resolutions for faster processing
- Apply blur before color detection for better results
- Use appropriate HSV ranges for reliable color detection
- Consider threading for concurrent camera and robot operations

## Future Enhancements

Planned features:
- Camera calibration utilities
- 3D vision with stereo cameras
- Machine learning-based object detection
- Integration with robotic vision pipelines
- Real-time object tracking
