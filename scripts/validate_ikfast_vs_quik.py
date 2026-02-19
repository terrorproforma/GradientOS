"""
Validate QuIK (DH-based) against IKFast on mini-6dof-arm.

This script treats IKFast as reference and compares:
1) FK agreement (tool-frame transform error)
2) IK behavior on sampled target poses (success/error statistics)

It can be pointed at any DH CSV, so it is ideal for validating extracted DH
tables before wiring them into runtime.
"""

from __future__ import annotations

import argparse
import json
import math
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

from ikfast_solver.ikfast_wrapper import IKFastSolver

try:
    import importlib

    pyquik = importlib.import_module("numeric_solver.pyquik.pyquik")
except ModuleNotFoundError as exc:
    raise RuntimeError(
        "pyquik extension not found. Build it first (src/numeric_solver/pyquik)."
    ) from exc


def wrap_to_pi(delta: np.ndarray) -> np.ndarray:
    return (delta + np.pi) % (2.0 * np.pi) - np.pi


def load_joint_limits_from_urdf(urdf_path: Path) -> tuple[np.ndarray, np.ndarray]:
    root = ET.parse(urdf_path).getroot()
    limits: dict[int, tuple[float, float]] = {}
    for j in root.findall("joint"):
        name = j.attrib.get("name", "")
        if not name.startswith("joint") or not name[5:].isdigit():
            continue
        idx = int(name[5:])
        lim = j.find("limit")
        if lim is None:
            continue
        lower = float(lim.attrib.get("lower", str(-math.pi)))
        upper = float(lim.attrib.get("upper", str(math.pi)))
        limits[idx] = (lower, upper)

    if not limits:
        raise ValueError(f"No joint limits found in {urdf_path}")

    max_idx = max(limits)
    ordered = [limits[i] for i in range(1, max_idx + 1)]
    lows = np.array([x[0] for x in ordered], dtype=float)
    highs = np.array([x[1] for x in ordered], dtype=float)
    return lows, highs


def parse_matrix4(value, field_name: str) -> np.ndarray:
    if isinstance(value, list):
        if len(value) == 16 and all(isinstance(v, (int, float)) for v in value):
            return np.array(value, dtype=float).reshape(4, 4)
        if len(value) == 4 and all(isinstance(row, list) and len(row) == 4 for row in value):
            return np.array([v for row in value for v in row], dtype=float).reshape(4, 4)
    raise ValueError(f"Invalid numeric.{field_name}; expected 16-list or 4x4 nested list.")


def load_numeric_overrides(manifest_path: Path, dof: int) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    with manifest_path.open("r", encoding="utf-8") as f:
        data = json.load(f)

    cfg = data.get("numeric", {})
    if not isinstance(cfg, dict):
        raise ValueError(f"'numeric' must be object in {manifest_path}")

    tbase = np.eye(4, dtype=float)
    ttool = np.eye(4, dtype=float)
    q_sign = np.ones(dof, dtype=float)
    if "tbase" in cfg:
        tbase = parse_matrix4(cfg["tbase"], "tbase")
    if "ttool" in cfg:
        ttool = parse_matrix4(cfg["ttool"], "ttool")
    if "q_sign" in cfg:
        arr = cfg["q_sign"]
        if not isinstance(arr, list) or len(arr) != dof:
            raise ValueError(f"numeric.q_sign must be list length {dof}")
        q_sign = np.array(arr, dtype=float)
    return tbase, ttool, q_sign


def build_quik_solver(dh_csv: Path, manifest_path: Path):
    dh = np.genfromtxt(str(dh_csv), delimiter=",", skip_header=1, usecols=(1, 2, 3, 4), dtype=float)
    if dh.ndim != 2 or dh.shape[1] != 4:
        raise ValueError(f"DH CSV malformed: {dh_csv}")
    dof = int(dh.shape[0])
    link_types = [0] * dof
    tbase, ttool, q_sign = load_numeric_overrides(manifest_path, dof)
    robot = pyquik.Robot(dh, link_types, q_sign, tbase, ttool)
    solver = pyquik.IKSolver(robot)
    return robot, solver


def ikfast_fk_matrix(ikfast: IKFastSolver, q: np.ndarray) -> np.ndarray:
    t, r = ikfast.compute_fk(q)
    T = np.eye(4, dtype=float)
    T[:3, :3] = np.array(r, dtype=float).reshape(3, 3)
    T[:3, 3] = np.array(t, dtype=float)
    return T


def run_validation(
    urdf_path: Path,
    manifest_path: Path,
    dh_csv: Path,
    samples: int,
    seed: int,
) -> dict[str, float]:
    ikfast = IKFastSolver()
    robot, quik_solver = build_quik_solver(dh_csv, manifest_path)

    lows, highs = load_joint_limits_from_urdf(urdf_path)
    rng = np.random.default_rng(seed)

    fk_pos_err: list[float] = []
    fk_rot_err_deg: list[float] = []
    ik_joint_err_ikfast: list[float] = []
    ik_joint_err_quik: list[float] = []
    ik_pos_err_quik: list[float] = []
    ik_rot_err_quik_deg: list[float] = []
    ik_success_ikfast = 0
    ik_success_quik = 0

    q_samples = rng.uniform(lows, highs, size=(samples, len(lows))).astype(float)
    if samples > 0:
        q_samples[0, :] = 0.0

    for q_true in q_samples:
        T_ref = ikfast_fk_matrix(ikfast, q_true)
        T_quik = robot.FK(q_true)

        fk_pos_err.append(float(np.linalg.norm(T_ref[:3, 3] - T_quik[:3, 3])))
        rot_delta = Rotation.from_matrix(T_ref[:3, :3].T @ T_quik[:3, :3]).as_rotvec()
        fk_rot_err_deg.append(float(np.rad2deg(np.linalg.norm(rot_delta))))

        target_pos = T_ref[:3, 3]
        target_rot = T_ref[:3, :3]
        target_quat = Rotation.from_matrix(target_rot).as_quat()

        seed_q = q_true + rng.normal(0.0, 0.08, size=q_true.shape)

        q_if = ikfast.solve_ik(target_pos, target_rot.flatten(), seed_q)
        if q_if is not None:
            ik_success_ikfast += 1
            q_if = np.asarray(q_if, dtype=float)
            ik_joint_err_ikfast.append(float(np.linalg.norm(wrap_to_pi(q_if - q_true))))

        q_qk, _e, _iters, _br = quik_solver.solve(target_quat, target_pos, seed_q)
        if q_qk is not None:
            ik_success_quik += 1
            q_qk = np.asarray(q_qk, dtype=float)
            ik_joint_err_quik.append(float(np.linalg.norm(wrap_to_pi(q_qk - q_true))))

            T_qk_sol = robot.FK(q_qk)
            ik_pos_err_quik.append(float(np.linalg.norm(T_ref[:3, 3] - T_qk_sol[:3, 3])))
            rot_sol = Rotation.from_matrix(T_ref[:3, :3].T @ T_qk_sol[:3, :3]).as_rotvec()
            ik_rot_err_quik_deg.append(float(np.rad2deg(np.linalg.norm(rot_sol))))

    def _mean(arr: list[float]) -> float:
        return float(np.mean(arr)) if arr else float("nan")

    def _max(arr: list[float]) -> float:
        return float(np.max(arr)) if arr else float("nan")

    return {
        "samples": float(samples),
        "fk_pos_mean_m": _mean(fk_pos_err),
        "fk_pos_max_m": _max(fk_pos_err),
        "fk_rot_mean_deg": _mean(fk_rot_err_deg),
        "fk_rot_max_deg": _max(fk_rot_err_deg),
        "ikfast_success_rate": ik_success_ikfast / max(1, samples),
        "quik_success_rate": ik_success_quik / max(1, samples),
        "ikfast_joint_err_mean_rad": _mean(ik_joint_err_ikfast),
        "quik_joint_err_mean_rad": _mean(ik_joint_err_quik),
        "quik_ik_pose_pos_mean_m": _mean(ik_pos_err_quik),
        "quik_ik_pose_pos_max_m": _max(ik_pos_err_quik),
        "quik_ik_pose_rot_mean_deg": _mean(ik_rot_err_quik_deg),
        "quik_ik_pose_rot_max_deg": _max(ik_rot_err_quik_deg),
    }


def maybe_plot(report: dict[str, float], plot_path: Path) -> None:
    try:
        import matplotlib.pyplot as plt
    except ImportError:
        print("[validate_ikfast_vs_quik] matplotlib not installed; skipping plot.")
        return

    keys = [
        "fk_pos_mean_m",
        "fk_rot_mean_deg",
        "ikfast_joint_err_mean_rad",
        "quik_joint_err_mean_rad",
        "quik_ik_pose_pos_mean_m",
        "quik_ik_pose_rot_mean_deg",
    ]
    vals = [report[k] for k in keys]

    fig = plt.figure(figsize=(10, 4))
    ax = fig.add_subplot(1, 1, 1)
    ax.bar(range(len(keys)), vals)
    ax.set_xticks(range(len(keys)))
    ax.set_xticklabels(keys, rotation=25, ha="right")
    ax.set_title("IKFast vs QuIK Validation Summary")
    ax.grid(True, axis="y", alpha=0.3)
    fig.tight_layout()
    plot_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(plot_path, dpi=170)
    plt.close(fig)
    print(f"[validate_ikfast_vs_quik] Saved plot: {plot_path}")


def main() -> None:
    parser = argparse.ArgumentParser(description="Validate QuIK against IKFast on mini-6dof-arm.")
    parser.add_argument("--urdf", type=Path, default=Path("robots/mini-6dof-arm/mini-6dof-arm.urdf"))
    parser.add_argument("--manifest", type=Path, default=Path("robots/mini-6dof-arm/robot.json"))
    parser.add_argument("--dh-csv", type=Path, default=Path("robots/mini-6dof-arm/dh_params.csv"))
    parser.add_argument("--samples", type=int, default=300)
    parser.add_argument("--seed", type=int, default=7)
    parser.add_argument("--plot", type=Path, default=Path("robots/mini-6dof-arm/ikfast_vs_quik_validation.png"))
    parser.add_argument("--no-plot", action="store_true")
    args = parser.parse_args()

    report = run_validation(
        urdf_path=args.urdf.resolve(),
        manifest_path=args.manifest.resolve(),
        dh_csv=args.dh_csv.resolve(),
        samples=max(1, int(args.samples)),
        seed=int(args.seed),
    )

    print("\n[validate_ikfast_vs_quik] Summary")
    for k, v in report.items():
        print(f"  {k}: {v}")

    if not args.no_plot:
        maybe_plot(report, args.plot.resolve())


if __name__ == "__main__":
    main()
