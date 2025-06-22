# IKFast Solver for Mini-6DOF-Arm

This directory contains the C++ source code for the high-performance inverse kinematics (IK) solver used by the `mini-arm` controller. It uses a pre-generated solver from `IKFast` and wraps it in a Python module using `pybind11`.

## Overview

- **`ikfast.h`**: The header file from OpenRAVE defining the IKFast C++ API.
- **`ikfast_solver.cpp`**: The auto-generated C++ code for the mini-6dof-arm's specific kinematics. This file is highly complex and should not be edited manually.
- **`ik_wrapper.cpp`**: A C++ wrapper that uses `pybind11` to expose the core IKFast functions (`ComputeFk`, `ComputeIk`, etc.) to Python. This is where the logic for selecting the best IK solution from multiple results resides.
- **`ikfast_wrapper.py`**: A Python class that provides a high-level, user-friendly interface to the compiled C++ module. This is the class that is imported and used by the `arm_controller` package.
- **`CMakeLists.txt`**: The build script for compiling the C++ source into a Python extension module (`.so` file on Linux).

## Building

This solver is not intended to be built as a standalone package. It is automatically compiled as a Python extension module when the main `mini-arm` project is installed (e.g., by running `pip install -e .` from the project root). The `pyproject.toml` file in the root directory is configured to use `scikit-build-core` to handle the CMake build process for this C++ extension.

When the project is installed, a compiled library file (e.g., `ikfast_solver.cpython-311-aarch64-linux-gnu.so`) will be placed in this directory, allowing it to be imported directly by the Python code in the `src` folder. 