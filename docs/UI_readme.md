# UI Setup and Running Guide for mini-arm Project

This guide explains how to set up and run the UI (`ui_start.py`) on a Raspberry Pi 5. The UI uses PySide6 for the interface and pyqtgraph for 3D simulation.

## Prerequisites
- Raspberry Pi 5 with Raspberry Pi OS (64-bit recommended).
- Python 3.11+ installed.
- Virtual environment (.venv) in the project root.

## Installation
1. Activate the virtual environment:
   ```bash
   source .venv/bin/activate
   ```

2. Install GradientOS (installs the required Python packages):
   ```bash
   pip install -e .
   ```

3. For OpenGL support (required for simulation page):
   ```bash
   sudo apt-get update && sudo apt-get install -y libgl1-mesa-glx libgl1-mesa-dri mesa-utils
   ```
   - After installation, reboot: `sudo reboot`.
   - Verify OpenGL: `glxinfo | grep 'OpenGL renderer'` (should show hardware renderer like 'VC4 V3D').

## Running the UI
1. Ensure venv is activated.
2. Run the script:
   ```bash
   python ui_start.py
   ```

## Troubleshooting
- **UI hangs or OpenGL warnings**: Increase GPU memory in `sudo raspi-config` > Performance Options > GPU Memory (set to 128MB or higher). Reboot.
- **QOpenGLWidget not supported**: Ensure Mesa packages are installed and system is rebooted.
- **Deprecation warnings**: The code uses app.exec() to avoid issues.
- If simulation page causes issues, comment out pyqtgraph.opengl usage in the code.

This setup provides a functional UI for robot control. Report issues in the repo. 

## Performance notes (Raspberry Pi)
- The UI no longer imports OpenGL on startup. The `pyqtgraph.opengl` import was removed from `src/gradient_os/ui_start.py` to avoid slow and fragile OpenGL initialization on headless or low-power systems. If a future simulation view is added, load OpenGL lazily within that feature only.
- Trajectory list refresh is now asynchronous. `RealControlPage.refresh_trajectory_list` sends `GET_TRAJECTORIES` and relies on the UI dispatcher to populate the list when the response arrives. This avoids blocking the UI thread for up to 3 seconds while waiting for UDP responses.