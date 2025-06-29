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

# IKFast Solver – Build & Re-build Guide

This mini-README records the exact steps required to compile (or re-compile) the C++/pybind11 IKFast extension inside a virtual-environment.  Follow it any time you change the C++ sources (e.g. `ik_wrapper.cpp`) or pull updates.

---
## 1.  Activate your venv
```bash
python -m venv .venv      # once, to create
source .venv/bin/activate  # each shell session
```

## 2.  Install build-time dependencies (once)
```bash
python -m pip install -U pip wheel
python -m pip install -U scikit-build-core pybind11 pybind11-global ninja
```
* `scikit-build-core` – PEP-517 backend that drives CMake
* `pybind11` + `pybind11-global` – headers **and** CMake config files
* `ninja` (optional) – faster build tool; CMake falls back to make if absent

## 3.  Build / rebuild the extension in editable mode
```bash
# from the project root (the directory with pyproject.toml)
python -m pip install --no-build-isolation --force-reinstall -e .
```
Flags explained:
* `-e .` – *editable* install; drops the compiled `.so` next to the Python sources
* `--force-reinstall` – rebuild even if the version string hasn't changed
* `--no-build-isolation` – reuse your venv's installed build tools (faster)

On success you'll see output similar to:
```
[100%] Built target ikfast_solver
Successfully built ikfast_solver
Successfully installed ikfast_solver-0.1.0
```

## 4.  Reload
Restart any Python processes (scripts, notebooks, ROS nodes, etc.) so they load the newly-compiled shared library.

---
### Troubleshooting
| Symptom                                             | Fix                                                         |
|-----------------------------------------------------|-------------------------------------------------------------|
| `ModuleNotFoundError: scikit_build_core`            | `pip install -U scikit-build-core`                          |
| CMake error "Could not find pybind11Config.cmake"   | `pip install -U pybind11 pybind11-global`                   |
| No compiler found                                   | Install `build-essential` (Ubuntu) or the equivalent tool-chain |

---
**Tip:** add `export CMAKE_GENERATOR=Ninja` to your shell profile if you have ninja installed – builds are noticeably faster. 