# Spherical-IK Solver – End-to-End Installation and Usage Guide

This document explains **every single step** required to:

1. Create a clean Python environment.
2. Install all necessary system and Python packages.
3. Configure, build and test the C++ / pybind11 solver from a fresh clone.
4. Regenerate the solver after any edit to the URDF file.

It is intentionally detailed so a newcomer can succeed **without prior C++, CMake or Python packaging experience**.

---
## 1 Prerequisites you must install once per machine

| What you need | Exact version / source | Why it is needed |
|--------------|-----------------------|------------------|
| **Git** | https://git-scm.com/downloads | Fetch Eigen / pybind11 and manage your own repo. |
| **Visual Studio 2022 Community**<br/>with the *Desktop development with C++* workload | https://visualstudio.microsoft.com | Provides the MSVC 19.x C++17 compiler and ‑std:c++17 STL. |
| **CMake ≥ 3.16** | https://cmake.org/download/ | Generates VS projects and fetches dependencies. |
| **Python ≥ 3.8 (64-bit)** | https://www.python.org/downloads/windows/ | Hosts the test script and provides `pip`. |
| **PowerShell 7 (optional)** | https://aka.ms/powershell | A modern shell (but classic Windows PowerShell also works). |

> After each install log out / log in to refresh your `PATH`.

---
## 2 Clone the repository

```powershell
cd C:\work        # any folder you like
git clone https://example.com/spherical-wrist-solver.git
cd spherical-wrist-solver
```

Throughout this guide *project-root* refers to this directory.

---
## 3 Create and activate a Python virtual-environment (optional but recommended)

```powershell
python -m venv venv                      # create venv in sub-folder
& .\venv\Scripts\Activate.ps1          # activate it (every new shell)
```

The prompt changes to `(venv)` so you know the environment is active.

---
## 4 Install required Python packages **inside the venv**

```powershell
pip install --upgrade pip               # fetch latest pip
pip install numpy urdfpy networkx pybind11[global]
```

* `numpy`, `urdfpy`, `networkx` are runtime deps for the header generator.
* `pybind11[global]` installs CMake configuration files so CMake can find the headers automatically.

---
## 5 Generate the PoE header once (checks the URDF)

```powershell
python tools/generate_poe_header.py mini-6dof-arm/spherical-mini-6dof-arm.urdf tool_link > build/generated/ik_poe.hpp
```

• `tool_link` **must be** the end-effector link name in the URDF.  
• The script prints the joint chain and writes `build/generated/ik_poe.hpp` containing:
  – one row per actuated joint (6 rows total)  
  – the home-pose matrix `M_HOME`  
  – the wrist-flange offset `d6`.

> If the URDF changes you **must rerun** this command.

---
## 6 Configure CMake (creates VS solution in `build/`)

```powershell
cmake -S . -B build -DURDF_FILE=mini-6dof-arm/spherical-mini-6dof-arm.urdf -DTOOL_LINK=tool_link
```

Explanation of the flags:
* `-S .` tells CMake where the source tree is.
* `-B build` tells it to write all generated files into `build/`.
* `URDF_FILE` and `TOOL_LINK` are cached so regeneration happens automatically at later builds.

During configure CMake will:
1. Detect MSVC and enable `/std:c++17`.
2. Download Eigen 3.4.0 if it is not installed system-wide.
3. Locate (or fetch) pybind11 v2.11.1.
4. Write *Visual Studio 17 2022* project files.

A successful run ends with
```
-- Configuring done
-- Generating done
-- Build files have been written to: …\build
```

---
## 7 Build everything (C++ library + Python module)

```powershell
cmake --build build --config Release --target ALL_BUILD
```

The first build can take a few minutes (Eigen + pybind11 fetch).  The final lines should read:
```
…\spherical_ik_py.cp38-win_amd64.pyd  # path ends with your Python version
```

The `.pyd` file lives in `build\Release\` (for Release config) or `build\Debug\`.

---
## 8 Make the module visible to Python

```powershell
$env:PYTHONPATH = "$env:PYTHONPATH;$(Resolve-Path build\Release)"
```

Run this **every new shell** before executing Python scripts.

---
## 9 Run the smoke-test script

```powershell
python .\test.py
```

Expected output:
```
T06 =
 [[ 0.  0.  1.  0.489006]
  [ 0. -1.  0.  0.      ]
  [ 1.  0.  0.  0.3701  ]
  [ 0.  0.  0.  1.      ]]
re-solve gave 8 solutions; diff (deg):
[0. 0. 0. 0. 0. 0.]
…
```

Any non-zero values indicate the wrong header or module is being loaded.

---
## 10 Workflow when the URDF changes

1. Edit your `.urdf` file.
2. Regenerate the header:
   ```powershell
   python tools/generate_poe_header.py <new.urdf> tool_link > build/generated/ik_poe.hpp
   ```
3. Re-build:
   ```powershell
   cmake --build build --config Release --target ALL_BUILD
   ```
4. Run tests again.

No need to re-run the `cmake -S` configure step unless `URDF_FILE` or `TOOL_LINK` changes.

---
## Troubleshooting Cheat-Sheet

| Symptom | Cause | Fix |
|---------|-------|-----|
| `error C2078: too many initializers` in `ik_poe.hpp` | Header lists ≠ `N` rows (e.g., 7 rows instead of 6). | Regenerate the header after you modified the generator script. |
| `ImportError: PyInit_spherical_ik_py` | `PYBIND11_MODULE` name mismatch | Ensure `spherical_ik_py.cpp` calls `PYBIND11_MODULE(spherical_ik_py, m)` and rebuild. |
| `ModuleNotFoundError` | PYTHONPATH does not include the folder that holds `.pyd`. | Append the correct directory (`build\Release` or `build\`). |
| CMake cannot find Eigen / pybind11 | No internet; corporate proxy | Download tarballs manually and point `CMAKE_PREFIX_PATH` to the extracted folders. |

---
## 11 Cleaning everything

```powershell
Remove-Item -Recurse -Force .\build               # delete build tree
Remove-Item .\ik_poe.hpp                          # delete stale root-level headers (keep generated one)
```

Then repeat steps **5 → 9**.

---
### Congratulations 🎉
You have a fully automated, DH-free analytic 6-DoF inverse-kinematics solver
that rebuilds from raw URDF data in seconds. 