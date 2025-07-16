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

2. Install required Python packages:
   ```bash
   pip install pyside6 pyqtgraph numpy
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