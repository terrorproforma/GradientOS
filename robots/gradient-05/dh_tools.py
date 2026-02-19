"""
Gradient-05 DH extraction + fitting + validation utilities.

Why this exists
---------------
Naive URDF->DH extraction is usually not enough because DH convention imposes
specific frame assignments. This tool provides a practical, repeatable flow:

1) Build an initial DH seed from URDF geometry
2) Fit DH parameters against URDF FK samples (nonlinear least-squares)
3) Validate numerically (position/orientation residuals)
4) Generate a visual comparison plot (URDF chain vs DH chain)

The produced `dh_params.csv` is still an engineering model that should be
reviewed, but this gets us much closer than manual transcription.
"""

from __future__ import annotations

import argparse
import csv
import math
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable

import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation


SCRIPT_DIR = Path(__file__).resolve().parent
URDF_PATH_DEFAULT = SCRIPT_DIR / "gradient-05.urdf"
CSV_PATH_DEFAULT = SCRIPT_DIR / "dh_params.csv"
PLOT_PATH_DEFAULT = SCRIPT_DIR / "dh_validation.png"


@dataclass(frozen=True)
class JointSpec:
    name: str
    index: int
    joint_type: str
    parent: str
    child: str
    origin_xyz: np.ndarray
    origin_rpy: np.ndarray
    axis: np.ndarray
    lower_limit: float
    upper_limit: float


@dataclass(frozen=True)
class DhRow:
    joint: str
    a: float
    alpha: float
    d: float
    theta: float


@dataclass(frozen=True)
class FitResult:
    success: bool
    message: str
    nfev: int
    cost: float


@dataclass(frozen=True)
class ValidationResult:
    pos_errors_m: np.ndarray
    rot_errors_deg: np.ndarray
    q_samples: np.ndarray
    urdf_points: list[np.ndarray]
    dh_points: list[np.ndarray]
    worst_index: int


def _parse_triplet(text: str | None, default: tuple[float, float, float]) -> np.ndarray:
    if text is None or not text.strip():
        return np.array(default, dtype=float)
    parts = [float(x) for x in text.split()]
    if len(parts) != 3:
        raise ValueError(f"Expected 3 values, got {len(parts)} from '{text}'")
    return np.array(parts, dtype=float)


def _joint_index(name: str) -> int | None:
    if not name.startswith("joint"):
        return None
    suffix = name[5:]
    if not suffix.isdigit():
        return None
    return int(suffix)


def _rpy_to_rot_matrix(rpy: np.ndarray) -> np.ndarray:
    roll, pitch, yaw = [float(v) for v in rpy]
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    # URDF convention: Rz(yaw) @ Ry(pitch) @ Rx(roll)
    return np.array(
        [
            [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr],
        ],
        dtype=float,
    )


def _transform_from_xyz_rpy(xyz: np.ndarray, rpy: np.ndarray) -> np.ndarray:
    T = np.eye(4, dtype=float)
    T[:3, :3] = _rpy_to_rot_matrix(rpy)
    T[:3, 3] = xyz
    return T


def _rotation_about_axis(axis: np.ndarray, q: float) -> np.ndarray:
    axis = np.asarray(axis, dtype=float)
    norm = float(np.linalg.norm(axis))
    if norm == 0.0:
        raise ValueError("Joint axis must be non-zero.")
    axis = axis / norm
    R = Rotation.from_rotvec(axis * float(q)).as_matrix()
    T = np.eye(4, dtype=float)
    T[:3, :3] = R
    return T


def load_joint_chain(urdf_path: Path) -> list[JointSpec]:
    root = ET.parse(urdf_path).getroot()
    joints: list[JointSpec] = []

    for je in root.findall("joint"):
        name = je.attrib.get("name", "").strip()
        idx = _joint_index(name)
        if idx is None:
            continue

        jtype = je.attrib.get("type", "revolute")
        if jtype == "fixed":
            continue

        parent_el = je.find("parent")
        child_el = je.find("child")
        if parent_el is None or child_el is None:
            raise ValueError(f"Joint '{name}' missing parent/child tags.")
        parent = parent_el.attrib.get("link", "").strip()
        child = child_el.attrib.get("link", "").strip()
        if not parent or not child:
            raise ValueError(f"Joint '{name}' has invalid parent/child names.")

        origin_el = je.find("origin")
        axis_el = je.find("axis")
        limit_el = je.find("limit")

        origin_xyz = _parse_triplet(origin_el.attrib.get("xyz") if origin_el is not None else None, (0.0, 0.0, 0.0))
        origin_rpy = _parse_triplet(origin_el.attrib.get("rpy") if origin_el is not None else None, (0.0, 0.0, 0.0))
        axis = _parse_triplet(axis_el.attrib.get("xyz") if axis_el is not None else None, (0.0, 0.0, 1.0))

        if limit_el is not None:
            lower = float(limit_el.attrib.get("lower", str(-math.pi)))
            upper = float(limit_el.attrib.get("upper", str(math.pi)))
        else:
            lower = -math.pi
            upper = math.pi

        joints.append(
            JointSpec(
                name=name,
                index=idx,
                joint_type=jtype,
                parent=parent,
                child=child,
                origin_xyz=origin_xyz,
                origin_rpy=origin_rpy,
                axis=axis,
                lower_limit=lower,
                upper_limit=upper,
            )
        )

    if not joints:
        raise ValueError(f"No revolute joints named jointN found in {urdf_path}")

    joints.sort(key=lambda j: j.index)
    expected = list(range(1, len(joints) + 1))
    actual = [j.index for j in joints]
    if actual != expected:
        raise ValueError(f"Expected contiguous joint indices {expected}, found {actual}")
    return joints


def _dh_distance_and_contacts(
    z1: np.ndarray, z2: np.ndarray, p1: np.ndarray, p2: np.ndarray
) -> tuple[float, np.ndarray, np.ndarray]:
    z1 = np.asarray(z1, dtype=float)
    z2 = np.asarray(z2, dtype=float)
    p1 = np.asarray(p1, dtype=float)
    p2 = np.asarray(p2, dtype=float)
    z1 = z1 / np.linalg.norm(z1)
    z2 = z2 / np.linalg.norm(z2)

    cosz = float(np.dot(z1, z2))
    det = float(1.0 - cosz * cosz)
    if abs(det) <= 1e-8:
        det = 0.0

    b = np.array([np.dot(p2 - p1, z1), np.dot(p2 - p1, z2)], dtype=float)
    if det != 0.0:
        Ainv = (1.0 / det) * np.array(
            [[-np.dot(z2, z2), np.dot(z1, z2)], [-np.dot(z1, z2), np.dot(z1, z1)]],
            dtype=float,
        )
        s = Ainv @ b
    else:
        s = np.array([b[0], 0.0], dtype=float)

    cp1 = p1 + s[0] * z1
    cp2 = p2 + s[1] * z2
    return float(np.linalg.norm(cp1 - cp2)), cp1, cp2


def _dh_alpha(z1: np.ndarray, z2: np.ndarray) -> float:
    z1 = np.asarray(z1, dtype=float)
    z2 = np.asarray(z2, dtype=float)
    z1 = z1 / np.linalg.norm(z1)
    z2 = z2 / np.linalg.norm(z2)
    cosang = float(np.clip(np.dot(z1, z2), -1.0, 1.0))
    return float(math.acos(cosang))


def _world_axes_and_origins_zero(joints: list[JointSpec]) -> tuple[list[np.ndarray], list[np.ndarray]]:
    T = np.eye(4, dtype=float)
    z_axes: list[np.ndarray] = []
    origins: list[np.ndarray] = []
    for j in joints:
        T = T @ _transform_from_xyz_rpy(j.origin_xyz, j.origin_rpy)
        z_world = T[:3, :3] @ (j.axis / np.linalg.norm(j.axis))
        z_axes.append(z_world.astype(float))
        origins.append(T[:3, 3].copy().astype(float))
    return z_axes, origins


def build_seed_dh_rows(joints: list[JointSpec]) -> list[DhRow]:
    z_axes, points = _world_axes_and_origins_zero(joints)
    rows: list[DhRow] = []
    n = len(joints)
    for i in range(n):
        if i < n - 1:
            a_i, _cp1, _cp2 = _dh_distance_and_contacts(z_axes[i], z_axes[i + 1], points[i], points[i + 1])
            alpha_i = _dh_alpha(z_axes[i], z_axes[i + 1])
        else:
            a_i = 0.0
            alpha_i = 0.0

        if i == 0:
            d_i = float(np.dot(z_axes[i] / np.linalg.norm(z_axes[i]), points[i]))
        else:
            d_i = float(np.dot(z_axes[i - 1] / np.linalg.norm(z_axes[i - 1]), points[i] - points[i - 1]))

        rows.append(DhRow(joint=joints[i].name, a=float(a_i), alpha=float(alpha_i), d=float(d_i), theta=0.0))
    return rows


def _dh_transform(a: float, alpha: float, d: float, theta: float) -> np.ndarray:
    # Matches QuIK Robot.hpp exactly (standard DH form used there).
    ct, st = math.cos(theta), math.sin(theta)
    ca, sa = math.cos(alpha), math.sin(alpha)
    return np.array(
        [
            [ct, -st * ca, st * sa, a * ct],
            [st, ct * ca, -ct * sa, a * st],
            [0.0, sa, ca, d],
            [0.0, 0.0, 0.0, 1.0],
        ],
        dtype=float,
    )


def fk_urdf_chain(joints: list[JointSpec], q: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    if q.shape != (len(joints),):
        raise ValueError(f"Expected q shape {(len(joints),)}, got {q.shape}")
    T = np.eye(4, dtype=float)
    points = [T[:3, 3].copy()]
    for j, qi in zip(joints, q, strict=True):
        T = T @ _transform_from_xyz_rpy(j.origin_xyz, j.origin_rpy)
        T = T @ _rotation_about_axis(j.axis, float(qi))
        points.append(T[:3, 3].copy())
    return T, np.array(points, dtype=float)


def fk_dh(rows: list[DhRow], q: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    if q.shape != (len(rows),):
        raise ValueError(f"Expected q shape {(len(rows),)}, got {q.shape}")
    T = np.eye(4, dtype=float)
    points = [T[:3, 3].copy()]
    for row, qi in zip(rows, q, strict=True):
        T = T @ _dh_transform(row.a, row.alpha, row.d, row.theta + float(qi))
        points.append(T[:3, 3].copy())
    return T, np.array(points, dtype=float)


def _rows_to_vector(rows: list[DhRow]) -> np.ndarray:
    vals: list[float] = []
    for r in rows:
        vals.extend([r.a, r.alpha, r.d, r.theta])
    return np.array(vals, dtype=float)


def _vector_to_rows(x: np.ndarray, joints: list[JointSpec]) -> list[DhRow]:
    out: list[DhRow] = []
    for i, j in enumerate(joints):
        a, alpha, d, theta = [float(v) for v in x[4 * i: 4 * i + 4]]
        out.append(DhRow(joint=j.name, a=a, alpha=alpha, d=d, theta=theta))
    return out


def _sample_q(joints: list[JointSpec], samples: int, seed: int) -> np.ndarray:
    rng = np.random.default_rng(seed)
    low = np.array([j.lower_limit for j in joints], dtype=float)
    high = np.array([j.upper_limit for j in joints], dtype=float)
    # For very large limits, cap identification range to 2*pi around center.
    span = high - low
    center = (high + low) * 0.5
    cap = 2.0 * math.pi
    low_eff = np.where(span > cap, center - cap * 0.5, low)
    high_eff = np.where(span > cap, center + cap * 0.5, high)
    qs = rng.uniform(low_eff, high_eff, size=(samples, len(joints))).astype(float)
    if samples > 0:
        qs[0, :] = 0.0
    return qs


def fit_dh_rows(
    joints: list[JointSpec],
    seed_rows: list[DhRow],
    fit_samples: int,
    rng_seed: int,
    pos_weight: float,
    rot_weight: float,
    reg_weight: float,
    max_nfev: int,
) -> tuple[list[DhRow], FitResult]:
    x0 = _rows_to_vector(seed_rows)
    qs = _sample_q(joints, max(10, fit_samples), rng_seed)

    # Bounds: keep physically sensible and numerically stable.
    n = len(joints)
    lb = np.tile(np.array([-5.0, -math.pi, -5.0, -math.pi], dtype=float), n)
    ub = np.tile(np.array([5.0, math.pi, 5.0, math.pi], dtype=float), n)

    def residual(x: np.ndarray) -> np.ndarray:
        rows = _vector_to_rows(x, joints)
        parts: list[np.ndarray] = []
        for q in qs:
            T_u, _ = fk_urdf_chain(joints, q)
            T_d, _ = fk_dh(rows, q)
            pos_res = (T_d[:3, 3] - T_u[:3, 3]) * pos_weight
            rot_res = Rotation.from_matrix(T_u[:3, :3].T @ T_d[:3, :3]).as_rotvec() * rot_weight
            parts.append(pos_res)
            parts.append(rot_res)
        if reg_weight > 0:
            parts.append((x - x0) * reg_weight)
        return np.concatenate(parts).astype(float)

    sol = least_squares(
        residual,
        x0,
        bounds=(lb, ub),
        method="trf",
        loss="soft_l1",
        max_nfev=max_nfev,
        ftol=1e-10,
        xtol=1e-10,
        gtol=1e-10,
    )

    rows = _vector_to_rows(sol.x, joints)
    fit = FitResult(success=bool(sol.success), message=str(sol.message), nfev=int(sol.nfev), cost=float(sol.cost))
    return rows, fit


def validate_rows(joints: list[JointSpec], rows: list[DhRow], samples: int, seed: int) -> ValidationResult:
    qs = _sample_q(joints, max(2, samples), seed)
    pos_errors: list[float] = []
    rot_errors_deg: list[float] = []
    urdf_points: list[np.ndarray] = []
    dh_points: list[np.ndarray] = []

    for q in qs:
        T_u, p_u = fk_urdf_chain(joints, q)
        T_d, p_d = fk_dh(rows, q)
        pos_errors.append(float(np.linalg.norm(T_u[:3, 3] - T_d[:3, 3])))
        angle_deg = float(np.rad2deg(np.linalg.norm(Rotation.from_matrix(T_u[:3, :3].T @ T_d[:3, :3]).as_rotvec())))
        rot_errors_deg.append(angle_deg)
        urdf_points.append(p_u)
        dh_points.append(p_d)

    pe = np.array(pos_errors, dtype=float)
    re = np.array(rot_errors_deg, dtype=float)
    worst = int(np.argmax(pe + np.deg2rad(re)))
    return ValidationResult(
        pos_errors_m=pe,
        rot_errors_deg=re,
        q_samples=qs,
        urdf_points=urdf_points,
        dh_points=dh_points,
        worst_index=worst,
    )


def _set_axes_equal(ax, points: np.ndarray) -> None:
    mins = points.min(axis=0)
    maxs = points.max(axis=0)
    center = (mins + maxs) * 0.5
    span = max(float((maxs - mins).max()), 1e-6)
    half = span * 0.55
    ax.set_xlim(center[0] - half, center[0] + half)
    ax.set_ylim(center[1] - half, center[1] + half)
    ax.set_zlim(center[2] - half, center[2] + half)


def save_validation_plot(result: ValidationResult, plot_path: Path) -> None:
    try:
        import matplotlib.pyplot as plt
    except ImportError:
        print("[dh_tools] matplotlib not installed; skipping plot. Install with: pip install matplotlib")
        return

    fig = plt.figure(figsize=(14, 5))
    ax1 = fig.add_subplot(1, 3, 1, projection="3d")
    ax2 = fig.add_subplot(1, 3, 2, projection="3d")
    ax3 = fig.add_subplot(1, 3, 3)

    # Zero pose
    p_u0 = result.urdf_points[0]
    p_d0 = result.dh_points[0]
    ax1.plot(p_u0[:, 0], p_u0[:, 1], p_u0[:, 2], "o-", label="URDF")
    ax1.plot(p_d0[:, 0], p_d0[:, 1], p_d0[:, 2], "s--", label="DH")
    ax1.set_title("Zero Pose Overlay")
    ax1.set_xlabel("X")
    ax1.set_ylabel("Y")
    ax1.set_zlabel("Z")
    ax1.legend(loc="best")
    _set_axes_equal(ax1, np.vstack([p_u0, p_d0]))

    # Worst sample
    wi = result.worst_index
    p_uw = result.urdf_points[wi]
    p_dw = result.dh_points[wi]
    ax2.plot(p_uw[:, 0], p_uw[:, 1], p_uw[:, 2], "o-", label="URDF")
    ax2.plot(p_dw[:, 0], p_dw[:, 1], p_dw[:, 2], "s--", label="DH")
    ax2.set_title("Worst Sample Overlay")
    ax2.set_xlabel("X")
    ax2.set_ylabel("Y")
    ax2.set_zlabel("Z")
    ax2.legend(loc="best")
    _set_axes_equal(ax2, np.vstack([p_uw, p_dw]))

    # Error traces
    x = np.arange(len(result.pos_errors_m))
    ax3.plot(x, result.pos_errors_m, label="Position error (m)")
    ax3.plot(x, result.rot_errors_deg, label="Orientation error (deg)")
    ax3.set_title("Validation Errors")
    ax3.set_xlabel("Sample index")
    ax3.grid(True, alpha=0.3)
    ax3.legend(loc="best")

    fig.tight_layout()
    plot_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(plot_path, dpi=170)
    plt.close(fig)
    print(f"[dh_tools] Saved validation plot: {plot_path}")


def write_dh_csv(rows: Iterable[DhRow], csv_path: Path) -> None:
    with csv_path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=["joint", "a", "alpha", "d", "theta"])
        writer.writeheader()
        for r in rows:
            writer.writerow(
                {
                    "joint": r.joint,
                    "a": f"{r.a:.12g}",
                    "alpha": f"{r.alpha:.12g}",
                    "d": f"{r.d:.12g}",
                    "theta": f"{r.theta:.12g}",
                }
            )


def print_rows(rows: list[DhRow], title: str) -> None:
    print(f"\n[dh_tools] {title}")
    print("joint,a,alpha,d,theta")
    for r in rows:
        print(f"{r.joint},{r.a:.12g},{r.alpha:.12g},{r.d:.12g},{r.theta:.12g}")


def print_validation_summary(v: ValidationResult) -> None:
    print("\n[dh_tools] Validation summary")
    print(f"  samples                 : {len(v.pos_errors_m)}")
    print(f"  mean position error (m) : {v.pos_errors_m.mean():.6g}")
    print(f"  max position error (m)  : {v.pos_errors_m.max():.6g}")
    print(f"  mean orient error (deg) : {v.rot_errors_deg.mean():.6g}")
    print(f"  max orient error (deg)  : {v.rot_errors_deg.max():.6g}")


def main() -> None:
    parser = argparse.ArgumentParser(description="Build/fix DH parameters for Gradient-05 from URDF.")
    parser.add_argument("--urdf", type=Path, default=URDF_PATH_DEFAULT)
    parser.add_argument("--csv", type=Path, default=CSV_PATH_DEFAULT)
    parser.add_argument("--write", action="store_true", help="Write resulting DH rows to --csv.")
    parser.add_argument("--validate", action="store_true", help="Run validation against URDF FK.")
    parser.add_argument("--seed-only", action="store_true", help="Skip fitting and keep geometric seed rows.")
    parser.add_argument("--fit-samples", type=int, default=700, help="Random samples used during fitting.")
    parser.add_argument("--samples", type=int, default=350, help="Random samples used during validation.")
    parser.add_argument("--seed", type=int, default=42, help="Random seed.")
    parser.add_argument("--pos-weight", type=float, default=1.0, help="Position residual weight for fitting.")
    parser.add_argument("--rot-weight", type=float, default=0.3, help="Orientation residual weight for fitting.")
    parser.add_argument("--reg-weight", type=float, default=1e-3, help="Regularization strength toward seed DH.")
    parser.add_argument("--max-nfev", type=int, default=3500, help="Maximum least-squares function evaluations.")
    parser.add_argument("--plot", type=Path, default=PLOT_PATH_DEFAULT, help="Validation plot path.")
    parser.add_argument("--no-plot", action="store_true", help="Disable plot output even when validating.")
    args = parser.parse_args()

    urdf_path = args.urdf.resolve()
    csv_path = args.csv.resolve()
    plot_path = args.plot.resolve()

    joints = load_joint_chain(urdf_path)
    seed_rows = build_seed_dh_rows(joints)
    print_rows(seed_rows, "Geometric seed DH rows")

    final_rows = seed_rows
    fit_info: FitResult | None = None
    if not args.seed_only:
        final_rows, fit_info = fit_dh_rows(
            joints=joints,
            seed_rows=seed_rows,
            fit_samples=max(10, int(args.fit_samples)),
            rng_seed=int(args.seed),
            pos_weight=float(args.pos_weight),
            rot_weight=float(args.rot_weight),
            reg_weight=float(args.reg_weight),
            max_nfev=max(100, int(args.max_nfev)),
        )
        print_rows(final_rows, "Fitted DH rows")
        print(
            f"\n[dh_tools] Fit status: success={fit_info.success}, nfev={fit_info.nfev}, "
            f"cost={fit_info.cost:.6g}, message='{fit_info.message}'"
        )

    if args.write:
        write_dh_csv(final_rows, csv_path)
        print(f"[dh_tools] Wrote CSV: {csv_path}")

    if args.validate:
        validation = validate_rows(
            joints=joints,
            rows=final_rows,
            samples=max(2, int(args.samples)),
            seed=int(args.seed) + 101,
        )
        print_validation_summary(validation)
        if not args.no_plot:
            save_validation_plot(validation, plot_path)


if __name__ == "__main__":
    main()
