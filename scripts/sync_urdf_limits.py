#!/usr/bin/env python3
"""
Sync URDF joint limits into robot Python config.

By default this script reads:
  robots/gradient-05/gradient-05.urdf
and updates:
  src/gradient_os/arm_controller/robots/gradient05/config.py
"""

from __future__ import annotations

import argparse
import re
import sys
import xml.etree.ElementTree as ET
from pathlib import Path


_LIMITS_BLOCK_PATTERN = re.compile(
    r"("
    r"def\s+logical_joint_limits_rad\s*\(self\)\s*->\s*list\[tuple\[float,\s*float\]\]:.*?"
    r"return\s+\["
    r")"
    r"(.*?)"
    r"(\n\s+\])",
    re.DOTALL,
)


def _project_root() -> Path:
    return Path(__file__).resolve().parent.parent


def _default_paths(robot_id: str) -> tuple[Path, Path]:
    normalized_config_key = re.sub(r"[^a-zA-Z0-9_]+", "", robot_id)
    root = _project_root()
    urdf_path = root / "robots" / robot_id / f"{robot_id}.urdf"
    config_path = (
        root
        / "src"
        / "gradient_os"
        / "arm_controller"
        / "robots"
        / normalized_config_key
        / "config.py"
    )
    return urdf_path, config_path


def _read_urdf_joint_limits(
    urdf_path: Path,
    joint_prefix: str,
    joint_count: int,
) -> list[tuple[float, float]]:
    if not urdf_path.exists():
        raise FileNotFoundError(f"URDF not found: {urdf_path}")

    try:
        root = ET.parse(urdf_path).getroot()
    except ET.ParseError as exc:
        raise ValueError(f"Failed to parse URDF XML at {urdf_path}: {exc}") from exc

    limits: list[tuple[float, float]] = []
    for i in range(1, joint_count + 1):
        joint_name = f"{joint_prefix}{i}"
        joint_el = root.find(f".//joint[@name='{joint_name}']")
        if joint_el is None:
            raise ValueError(f"Missing <joint name='{joint_name}'> in {urdf_path}")

        limit_el = joint_el.find("limit")
        if limit_el is None:
            raise ValueError(f"Joint '{joint_name}' is missing <limit> in {urdf_path}")

        if "lower" not in limit_el.attrib or "upper" not in limit_el.attrib:
            raise ValueError(
                f"Joint '{joint_name}' must define both lower and upper attributes."
            )

        lower = float(limit_el.attrib["lower"])
        upper = float(limit_el.attrib["upper"])
        if lower >= upper:
            raise ValueError(
                f"Invalid range for '{joint_name}': lower={lower} must be < upper={upper}"
            )
        limits.append((lower, upper))

    return limits


def _render_limits_block(limits: list[tuple[float, float]]) -> str:
    # Keep indentation stable with current config style.
    lines = [f"\n            ({lower}, {upper}),  # J{idx}" for idx, (lower, upper) in enumerate(limits, start=1)]
    return "".join(lines)


def _sync_content(original: str, limits: list[tuple[float, float]]) -> str:
    match = _LIMITS_BLOCK_PATTERN.search(original)
    if not match:
        raise ValueError(
            "Could not locate `logical_joint_limits_rad` return list in target config. "
            "Expected a method block containing `def logical_joint_limits_rad(...)` and `return [`."
        )

    replacement = f"{match.group(1)}{_render_limits_block(limits)}{match.group(3)}"
    start, end = match.span()
    return original[:start] + replacement + original[end:]


def _build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Sync URDF joint limits into robot config.")
    parser.add_argument(
        "--robot-id",
        default="gradient-05",
        help="Robot asset id used for default path resolution (default: gradient-05).",
    )
    parser.add_argument(
        "--urdf-path",
        type=Path,
        default=None,
        help="Explicit URDF path (overrides --robot-id default URDF path).",
    )
    parser.add_argument(
        "--config-path",
        type=Path,
        default=None,
        help="Explicit Python config path (overrides --robot-id default config path).",
    )
    parser.add_argument(
        "--joint-prefix",
        default="joint",
        help="Joint name prefix in URDF (default: joint -> joint1..jointN).",
    )
    parser.add_argument(
        "--joint-count",
        type=int,
        default=6,
        help="Number of joints to sync (default: 6).",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Preview extracted limits and whether config would change, without writing.",
    )
    return parser


def main(argv: list[str] | None = None) -> int:
    parser = _build_arg_parser()
    args = parser.parse_args(argv)

    if args.joint_count <= 0:
        parser.error("--joint-count must be > 0")

    default_urdf, default_config = _default_paths(args.robot_id)
    urdf_path: Path = args.urdf_path or default_urdf
    config_path: Path = args.config_path or default_config

    if not config_path.exists():
        raise FileNotFoundError(f"Config file not found: {config_path}")

    print(f"[sync_urdf_limits] URDF:   {urdf_path}")
    print(f"[sync_urdf_limits] Config: {config_path}")

    limits = _read_urdf_joint_limits(urdf_path, args.joint_prefix, args.joint_count)
    print(f"[sync_urdf_limits] Extracted {len(limits)} limits: {limits}")

    original_content = config_path.read_text(encoding="utf-8")
    new_content = _sync_content(original_content, limits)

    changed = new_content != original_content
    if args.dry_run:
        state = "would update" if changed else "already up-to-date"
        print(f"[sync_urdf_limits] Dry run: config {state}.")
        return 0

    if changed:
        config_path.write_text(new_content, encoding="utf-8")
        print("[sync_urdf_limits] Config updated successfully.")
    else:
        print("[sync_urdf_limits] No changes needed; config already in sync.")

    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except Exception as exc:
        print(f"[sync_urdf_limits] ERROR: {exc}", file=sys.stderr)
        raise SystemExit(1)
