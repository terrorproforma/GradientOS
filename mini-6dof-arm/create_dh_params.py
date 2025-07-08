"""
================================================================================
Mini-6-DoF Arm – DH Parameter Extraction Utility
================================================================================

This module is a **self-contained command-line tool** that reads the URDF model
`mini-6dof-arm/mini-6dof-arm.urdf`, automatically computes the *Modified* Denavit–
Hartenberg (DH) parameters for each actuated joint, and stores the resulting
table in CSV form next to the URDF.

Why this script exists
---------------------
Industrial robot research frequently relies on DH parameters – four concise
numbers ⟨a, α, d, θ⟩ – instead of full 4 × 4 homogeneous transforms.  URDF
files, however, encode robot geometry using **frame origins** and **joint
rotation axes**.  Translating between these two representations by hand is
error-prone; this script performs that translation programmatically so that the
result is always consistent with the source URDF.

Key features
~~~~~~~~~~~~
* **No hard-coded geometry** – all values are extracted from the URDF so the
  algorithm works for any sequential chain whose joints are named
  `joint1`, `joint2`, …, `jointN`.
* **Modified DH convention** – matches common robotics texts and ROS MoveIt
  expectations.
* **Standalone** – only depends on ``numpy`` and ``urdfpy`` at runtime.  The
  upstream *nptyping* dependency is optional and gracefully degraded.
* **CSV output** – human readable, editable, and can be plotted in
  spreadsheets or imported into MATLAB / Python.

Installation
------------
Activate your virtual environment (optional but recommended):

```bash
python -m venv .venv
source .venv/bin/activate
```

Install the required packages:

```bash
pip install numpy urdfpy
# Optional, for nicer type annotations only
pip install nptyping
```

Running the extractor
--------------------
From the repository root (the path containing the ``mini-6dof-arm`` directory):

```bash
python mini-6dof-arm/create_dh_params.py
```

Upon success you will see a message similar to:

```
DH parameters saved to: dh_params.csv
```

The resulting ``dh_params.csv`` will contain:

```
joint,a,alpha,d,theta
joint1,0.0000,0.0000,0.0000,0.0000
joint2,0.0432,1.5708,0.0000,0.0000
…
```

(Numbers shown are **examples only** – your robot’s geometry dictates the real
values.)

Algorithm overview
------------------
1. **Forward-kinematics to world frame** – Using *urdfpy* we compute the world
   transform for each link and joint.
2. **Axis/point extraction** – For every joint *i* we record
   * the vector **zᵢ** (joint axis direction in world frame)
   * the point **pᵢ** (joint origin in world frame)
3. **Inter-frame geometry** – For consecutive joints \((i, i+1)\) we call the
   helper functions in ``utilities/urdf2dh``:
   * ``get_a`` – shortest distance *aᵢ* between **zᵢ** and **zᵢ₊₁**.
   * ``get_alpha`` – twist angle αᵢ between **zᵢ** and **zᵢ₊₁**.
4. **d and θ** – *dᵢ* is the projection of (pᵢ − pᵢ₋₁) onto **zᵢ₋₁**.
   θᵢ is the joint variable; we initialise it to 0 rad.
5. **Write CSV** – Parameters are saved with header order
   ``joint,a,alpha,d,theta`` so they can be directly opened in spreadsheet
   software.

Limitations & future work
-------------------------
* The script currently assumes **revolute** joints and a **serial, open chain**
  naming convention.  Parallel or branched manipulators will need special
  handling.
* θᵢ is set to 0 rad because DH tables traditionally separate fixed geometry
  from the joint variable.  To reflect a specific robot pose you may modify
  this field before writing the CSV.

Module layout
-------------
The remainder of this file is structured as follows:

* *Imports & path handling* – ensures the local `utilities/urdf2dh` package is
  importable even when this script is executed directly.
* *Internal helpers* – small functions for world-frame transformations.
* ``extract_dh_from_urdf`` – core extraction routine, returns ``list[dict]``.
* ``save_dh_table_csv`` – convenience wrapper.
* *`__main__` guard* – enables CLI usage.

"""
## use utilities/urdf2dh/urdf2dh/dhparam.py to create DH params from urdf

import sys
import os
import csv
from pathlib import Path
import numpy as np

# -----------------------------------------------------------------------------
# Add project root as well as the utilities folder to PYTHONPATH so that we can
# import the urdf2dh package no matter where this script is executed from.
# -----------------------------------------------------------------------------
PROJECT_ROOT = Path(__file__).resolve().parents[1]
UTILITIES_DIR = PROJECT_ROOT / "utilities"
# Add both the utilities directory *and* the exact package directory so that
# `import urdf2dh` resolves no matter how the project is laid out.

# Pre-pend instead of append to ensure they are found before site-packages.
sys.path.insert(0, str(PROJECT_ROOT))
sys.path.insert(0, str(UTILITIES_DIR))
# In some setups the direct package directory can be useful:
PACKAGE_DIR = UTILITIES_DIR / "urdf2dh"
if PACKAGE_DIR.exists():
    sys.path.insert(0, str(PACKAGE_DIR))

# -----------------------------------------------------------------------------
# External dependencies
# -----------------------------------------------------------------------------
try:
    from urdfpy import URDF  # type: ignore
except ImportError as exc:  # pragma: no cover
    raise ImportError(
        "The 'urdfpy' package is required for DH extraction. "
        "Install it with: pip install urdfpy"
    ) from exc

# -----------------------------------------------------------------------------
# Internal helpers from the urdf2dh package written by Yuji YAMAMOTO.
# -----------------------------------------------------------------------------
from urdf2dh.dhparam import get_a, get_alpha  # type: ignore

# -----------------------------------------------------------------------------
# Script constants
# -----------------------------------------------------------------------------
URDF_FILENAME = "mini-6dof-arm.urdf"
JOINT_NAME_TEMPLATE = "joint{}"  # names: joint1, joint2, ...
OUTPUT_CSV = "dh_params.csv"  # file will be created next to this script


def _world_transform_for_link(robot: URDF, link_name: str):
    """Return 4x4 homogeneous transform of *link_name* expressed in the base frame."""
    # `link_fk()` returns a mapping {link: 4x4 matrix}
    return robot.link_fk()[robot.link_map[link_name]]


def _joint_origin_world(robot: URDF, joint):
    """Return origin position (3,) and axis direction (3,) in world frame."""
    parent_tf = _world_transform_for_link(robot, joint.parent)
    joint_tf = parent_tf @ joint.origin  # transform of joint frame in world

    origin_world = joint_tf[:3, 3]
    axis_world = (joint_tf[:3, :3] @ joint.axis).flatten()
    return origin_world, axis_world


def extract_dh_from_urdf(urdf_path: Path):
    """Parse *urdf_path* and extract Modified DH parameters.

    Returns
    -------
    list[dict]: Each element corresponds to one joint and contains keys
                "a", "alpha", "d", and "theta".
    """
    robot = URDF.load(str(urdf_path))

    # 1. Gather joint origins (points) and z-axes directions in world frame
    z_axes = []  # list[np.ndarray], shape (3,)
    points = []  # list[np.ndarray], shape (3,)
    joint_order = []

    # We assume joints are named joint1, joint2, ..., jointN
    num_joints = sum(1 for j in robot.joints if j.joint_type != "fixed")
    for idx in range(1, num_joints + 1):
        jname = JOINT_NAME_TEMPLATE.format(idx)
        joint = robot.joint_map[jname]
        p, z = _joint_origin_world(robot, joint)
        points.append(p)
        z_axes.append(z)
        joint_order.append(jname)

    z_axes = [np.asarray(z, dtype=float) for z in z_axes]
    points = [np.asarray(p, dtype=float) for p in points]

    # 2. Compute DH parameters for successive pairs
    dh_rows = []
    n = len(z_axes)
    for i in range(n):
        if i < n - 1:
            # a_i & alpha_i: between frame i and i+1
            a_i, _parallel, _cp1, _cp2 = get_a(z_axes[i], z_axes[i + 1], points[i], points[i + 1])
            alpha_i = get_alpha(z_axes[i], z_axes[i + 1])
        else:
            # Last link: there is no subsequent joint
            a_i = 0.0
            alpha_i = 0.0

        # d_i: projection of vector between origins onto z_{i-1}
        if i == 0:
            # For the first joint, d is simply distance along its own z to its origin (zero)
            d_i = 0.0
        else:
            d_i = np.dot(z_axes[i - 1] / np.linalg.norm(z_axes[i - 1]), points[i] - points[i - 1])

        theta_i = 0.0  # For revolute joints, theta is the variable; default to 0

        dh_rows.append({"joint": joint_order[i], "a": a_i, "alpha": alpha_i, "d": d_i, "theta": theta_i})

    return dh_rows


def save_dh_table_csv(dh_rows, csv_path: Path):
    """Write *dh_rows* to *csv_path* in human-readable CSV format."""
    with csv_path.open("w", newline="") as fp:
        writer = csv.DictWriter(fp, fieldnames=["joint", "a", "alpha", "d", "theta"])
        writer.writeheader()
        for row in dh_rows:
            writer.writerow(row)


if __name__ == "__main__":
    script_dir = Path(__file__).resolve().parent
    urdf_path = script_dir / URDF_FILENAME
    if not urdf_path.exists():
        raise FileNotFoundError(f"URDF file '{urdf_path}' was not found.")

    dh_rows = extract_dh_from_urdf(urdf_path)

    csv_path = script_dir / OUTPUT_CSV
    save_dh_table_csv(dh_rows, csv_path)
    print(f"DH parameters saved to: {csv_path.relative_to(script_dir)}")



