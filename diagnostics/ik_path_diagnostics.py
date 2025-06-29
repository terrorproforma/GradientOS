"""ik_path_diagnostics.py
=================================
Standalone helper script for analysing IK-solver behaviour along a Cartesian
trajectory.  It is useful for spotting singularities that cause large joint
angle swings or end-effector tracking errors.

Usage (examples)
----------------
1. Straight-line path, 200 points, 10 cm downward:

    python diagnostics/ik_path_diagnostics.py --line 0 0 0.1 200

2. Load a JSON file containing a list of Cartesian points:

    python diagnostics/ik_path_diagnostics.py --json path_to_points.json

Arguments
~~~~~~~~~
--line  dx dy dz N     Generate a straight-line path of *N* points starting at
                       the current tool-tip position and offsetting by the
                       given vector (in metres).

--json  file           Use points loaded from a JSON file (list of [x,y,z]).

--closed-loop          If passed, the script will also compute closed-loop
                       metrics assuming a 400 Hz execution (joint velocity &
                       acceleration).

Outputs
~~~~~~~
* CSV  → diagnostics/ik_log_<timestamp>.csv
* PNGs → diagnostics/ik_angles_<timestamp>.png, ik_error_<timestamp>.png
"""

from __future__ import annotations

import argparse
import csv
import datetime as _dt
from pathlib import Path
from typing import List

import numpy as np
import matplotlib
matplotlib.use("Agg")  # headless
import matplotlib.pyplot as plt

try:
    import ik_solver
except ImportError as e:
    raise SystemExit("ERROR: Script must be run from the project root so 'ik_solver' is importable") from e


def _load_json_points(path: Path) -> List[List[float]]:
    import json, os
    if not path.exists():
        raise FileNotFoundError(path)
    with open(path, "r") as fp:
        pts = json.load(fp)
    if not pts or not isinstance(pts[0], (list, tuple)) or len(pts[0]) != 3:
        raise ValueError("JSON must contain a list of [x,y,z] points")
    return [list(map(float, p)) for p in pts]


def _generate_line(start_pos: np.ndarray, offset: np.ndarray, n: int) -> List[List[float]]:
    return [list(start_pos + offset * (i / (n - 1))) for i in range(n)]


def main() -> None:
    ap = argparse.ArgumentParser(description="IK Path Diagnostics")
    ap.add_argument("--line", nargs=4, type=float, metavar=("dx", "dy", "dz", "N"),
                    help="Generate N-point straight line from current tip offset by (dx,dy,dz)")
    ap.add_argument("--json", type=Path, help="Load list of [x,y,z] points from JSON file")
    ap.add_argument("--closed-loop", action="store_true", dest="closed_loop",
                    help="Compute closed-loop metrics (400 Hz) in addition to open-loop")
    args = ap.parse_args()

    if not args.line and not args.json:
        ap.error("Must supply either --line or --json")

    # ------------------------------------------------------------------
    # Determine start joint configuration & position
    # ------------------------------------------------------------------
    start_q = np.zeros(ik_solver.NUM_JOINTS)
    start_pos = ik_solver.get_fk(start_q)
    if start_pos is None:
        raise RuntimeError("FK failed for zero pose – cannot continue")

    # ------------------------------------------------------------------
    # Build Cartesian path
    # ------------------------------------------------------------------
    if args.line:
        dx, dy, dz, n_pts_f = args.line
        n_pts = int(n_pts_f)
        path_points = _generate_line(start_pos, np.array([dx, dy, dz]), n_pts)
    else:
        path_points = _load_json_points(args.json)
        n_pts = len(path_points)

    print(f"[Diag] Path with {n_pts} points ready → solving IK…")

    # ------------------------------------------------------------------
    # Solve IK (batch) with orientation lock to initial pose
    # ------------------------------------------------------------------
    initial_orient = np.identity(3)
    orientations = [initial_orient] * n_pts

    joint_path = ik_solver.solve_ik_path_batch(
        path_points=path_points,
        initial_joint_angles=start_q,
        target_orientations=orientations,
    )
    if joint_path is None:
        raise RuntimeError("IK solver failed on provided path")

    # ------------------------------------------------------------------
    # Compute FK to evaluate tracking error & joint deltas
    # ------------------------------------------------------------------
    errors = []
    for p_target, q in zip(path_points, joint_path):
        tip_pos = ik_solver.get_fk(q)
        if tip_pos is None:
            errors.append(np.nan)
        else:
            errors.append(float(np.linalg.norm(tip_pos - p_target)))

    joint_path_np = np.array(joint_path)
    # first-order differences (rad per step)
    vel = np.diff(joint_path_np, axis=0)

    # ------------------------------------------------------------------
    # Save CSV
    # ------------------------------------------------------------------
    ts = _dt.datetime.now().strftime("%Y%m%d_%H%M%S")
    out_dir = Path("diagnostics/ik_path")
    out_dir.mkdir(exist_ok=True, parents=True)
    csv_path = out_dir / f"ik_log_{ts}.csv"
    with open(csv_path, "w", newline="") as fp:
        writer = csv.writer(fp)
        header = [
            "index", "target_x", "target_y", "target_z", *[f"J{i+1}_rad" for i in range(ik_solver.NUM_JOINTS)], "pos_error_m"
        ]
        writer.writerow(header)
        for idx, (pt, q, err) in enumerate(zip(path_points, joint_path, errors)):
            writer.writerow([idx, *pt, *q, err])
    print(f"[Diag] CSV written → {csv_path}")

    # ------------------------------------------------------------------
    # Basic plots
    # ------------------------------------------------------------------
    fig, ax = plt.subplots(1, 1, figsize=(10, 4))
    for j in range(ik_solver.NUM_JOINTS):
        ax.plot(joint_path_np[:, j], label=f"J{j+1}")
    ax.set_title("Joint Angles (rad)")
    ax.set_xlabel("Step index")
    ax.legend(ncol=3)
    angle_png = out_dir / f"ik_angles_{ts}.png"
    fig.tight_layout()
    fig.savefig(angle_png)
    plt.close(fig)

    fig, ax = plt.subplots(1, 1, figsize=(10, 3))
    ax.plot(errors)
    ax.set_title("Tool-Tip Positional Error (m)")
    ax.set_xlabel("Step index")
    ax.set_ylabel("Error")
    error_png = out_dir / f"ik_error_{ts}.png"
    fig.tight_layout()
    fig.savefig(error_png)
    plt.close(fig)

    print(f"[Diag] Plots saved → {angle_png}, {error_png}")


if __name__ == "__main__":
    main() 