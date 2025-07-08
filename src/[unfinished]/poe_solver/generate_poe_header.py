#!/usr/bin/env python3
"""
Generate ik_poe.hpp directly from a URDF model.

This helper replaces the legacy DH-based pipeline.  It inspects a URDF with
urdfpy, extracts the joint origins (**pᵢ**) and axes (**ẑᵢ**) at the zero
configuration, computes the wrist-flange offset **d₆** as well as the tool
frame home pose **M** and dumps them into a tiny C++ header that can be
included at compile-time.  No manual parameter editing is required: whenever
CAD geometry changes simply re-run this script and rebuild.

Usage
-----
    python tools/generate_poe_header.py <robot.urdf> [<tool_link>] \
        > ik_poe.hpp

If *tool_link* is omitted the last link in the kinematic chain from base to
leaf is assumed to be the tool.
"""
from __future__ import annotations

import sys
from pathlib import Path
from typing import List

# -----------------------------------------------------------------------------
# Work-around for NumPy ≥2.0 where `np.int`, `np.float`, ... were removed.  A
# few packages (e.g. older NetworkX versions pulled in by urdfpy) still expect
# these symbols.  We re-introduce them as aliases to the built-in types so the
# import succeeds without forcing users to downgrade NumPy.
# -----------------------------------------------------------------------------

import numpy as np  # isort: skip

for _alias, _builtin in (("int", int), ("float", float), ("bool", bool)):
    if not hasattr(np, _alias):
        setattr(np, _alias, _builtin)

# urdfpy pulls in networkx ⟶ may rely on the aliases above
from urdfpy import URDF

# -----------------------------------------------------------------------------
# Helpers
# -----------------------------------------------------------------------------

def _usage() -> None:  # pragma: no cover
    print("Usage: generate_poe_header.py <robot.urdf> [tool_link]", file=sys.stderr)
    sys.exit(1)


def _flatten_col_major(T: np.ndarray) -> List[float]:
    """Return a 4×4 transform flattened in column-major order (Eigen style)."""
    return [float(T[i, j]) for j in range(4) for i in range(4)]


# -----------------------------------------------------------------------------
# Entry
# -----------------------------------------------------------------------------

def main() -> None:  # pragma: no cover
    if len(sys.argv) < 2:
        _usage()

    urdf_path = Path(sys.argv[1])
    if not urdf_path.is_file():
        print(f"URDF file not found: {urdf_path}", file=sys.stderr)
        sys.exit(2)

    tip_link_name = sys.argv[2] if len(sys.argv) >= 3 else None

    robot = URDF.load(str(urdf_path))
    base_link_name = robot.base_link.name if hasattr(robot.base_link, "name") else robot.base_link

    # Determine the kinematic chain (joint order) ------------------------------------------------
    if tip_link_name is None:
        # choose the leaf furthest from base (heuristic)
        leaves = [l.name for l in robot.links if not any(j.parent == l for j in robot.joints)]
        if not leaves:
            raise RuntimeError("Could not determine tool link – specify explicitly")
        tip_link_name = leaves[0]

    # Resolve the joint name sequence from base → tip.  Newer urdfpy
    # versions expose `URDF.get_chain`, older ones do not – so we fall back
    # to a simple parent map traversal.
    try:
        chain_joint_names = robot.get_chain(base_link_name, tip_link_name, links=False, joints=True)  # type: ignore[attr-defined]
    except AttributeError:
        # Build a map: child_link_name → joint
        child_to_joint = {}
        for j in robot.joints:
            child_name = j.child.name if hasattr(j.child, "name") else j.child
            child_to_joint[child_name] = j

        chain_rev = []  # from tip back to base
        cur = tip_link_name
        while cur != base_link_name:
            if cur not in child_to_joint:
                raise RuntimeError(f"Link '{cur}' is not reachable from base '{base_link_name}'")
            j = child_to_joint[cur]
            chain_rev.append(j.name)
            parent_name = j.parent.name if hasattr(j.parent, "name") else j.parent
            cur = parent_name
        chain_joint_names = list(reversed(chain_rev))

    # FK at zero pose ---------------------------------------------------------------------------
    cfg0 = {j.name: 0.0 for j in robot.joints if j.joint_type != "fixed"}
    T_links = robot.link_fk(cfg0)

    rows = []  # each: (joint, px,py,pz, zx,zy,zz)
    sys.stderr.write("Joint chain (base → tip):\n")

    for idx, jn in enumerate(chain_joint_names):
        joint = robot.joint_map[jn]

        # Resolve parent link object regardless of urdfpy version -----------
        if hasattr(joint.parent, "name"):
            parent_link_obj = joint.parent  # Link instance
            parent_name = joint.parent.name
        else:
            parent_name = joint.parent  # str
            parent_link_obj = robot.link_map[parent_name]

        parent_T = T_links[parent_link_obj]
        T_joint_origin = parent_T @ joint.origin
        p = T_joint_origin[:3, 3]

        # Use <axis xyz="..."> if provided (default 0,0,1)
        axis_local = np.array(joint.axis) if hasattr(joint, "axis") and joint.axis is not None else np.array([0.0, 0.0, 1.0])
        axis_local = axis_local.astype(float)
        if np.linalg.norm(axis_local) == 0:
            axis_local = np.array([0.0, 0.0, 1.0])
        axis_local /= np.linalg.norm(axis_local)
        # rotate into base frame using joint-origin rotation
        R_joint_origin = T_joint_origin[:3, :3]
        z = R_joint_origin @ axis_local
        z /= np.linalg.norm(z)

        if joint.joint_type == "fixed":
            continue          # skip fixed joints – not part of DOF 6
        if len(rows) >= 6:
            break             # ignore extras beyond 6-DoF
        rows.append((jn, *p, *z))

        child_name = joint.child.name if hasattr(joint.child, "name") else joint.child
        sys.stderr.write(f"  {idx+1}: joint {jn}  parent={parent_name} → child={child_name}  p=({p[0]:.4f},{p[1]:.4f},{p[2]:.4f})  z=({z[0]:.2f},{z[1]:.2f},{z[2]:.2f})\n")

    # Wrist centre is the origin of joint6 (rows[-1]) because z4,z5,z6 intersect there
    p_wc = np.array(rows[-1][1:4])
    tool_T = T_links[robot.link_map[tip_link_name]]
    z6 = np.array(rows[-1][4:7])
    d6 = float((tool_T[:3, 3] - p_wc) @ z6)

    # Diagnostic: flange origin (wrist centre + d6·z6)
    flange_p = p_wc + d6 * z6
    sys.stderr.write(f"Flange origin in base frame : ({flange_p[0]:.4f}, {flange_p[1]:.4f}, {flange_p[2]:.4f})  d6={d6:.6f} m\n")

    # Tool home pose ---------------------------------------------------------------------------
    M_home = _flatten_col_major(tool_T)

    # -------------------------------------------------------------------------
    # Emit header
    # -------------------------------------------------------------------------
    print("// Auto-generated by generate_poe_header.py – DO NOT EDIT\n#pragma once\n#include <array>\nnamespace ik {")
    print("constexpr int N = 6;")
    print("constexpr std::array<std::array<double,6>, N> POE = {{")
    for jn, px, py, pz, zx, zy, zz in rows:
        print(f"  /* {jn:>8} */ {{ {px:.10f}, {py:.10f}, {pz:.10f}, {zx:.10f}, {zy:.10f}, {zz:.10f} }},")
    print("}};")

    print("constexpr std::array<double, 16> M_HOME = {")
    print("  " + ", ".join(f"{v:.10f}" for v in M_home) + " };")
    print(f"constexpr double d6 = {d6:.10f};")
    print("} // namespace ik")


if __name__ == "__main__":  # pragma: no cover
    main() 