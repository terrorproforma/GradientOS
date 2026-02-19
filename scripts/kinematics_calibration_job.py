from __future__ import annotations

import argparse
from pathlib import Path

from gradient_os import robot_assets
from gradient_os.kinematics.calibration import (
    ProfilePromotionStore,
    build_profile_candidate_from_fit,
    load_dataset,
    validate_profile_candidate,
)


def main() -> int:
    parser = argparse.ArgumentParser(description="Run kinematics calibration fit/validate/promote workflow.")
    parser.add_argument("--dataset", required=True, help="Path to calibration dataset JSON.")
    parser.add_argument(
        "--store",
        default="kinematics_profiles",
        help="Profile promotion store root directory.",
    )
    parser.add_argument(
        "--actor",
        default="calibration-cli",
        help="Actor label recorded in activation history.",
    )
    parser.add_argument(
        "--activate",
        action="store_true",
        help="Activate approved profile after validation.",
    )
    args = parser.parse_args()

    dataset = load_dataset(Path(args.dataset))
    base_profile = robot_assets.load_kinematics_profile(dataset.robot_id)
    candidate = build_profile_candidate_from_fit(base_profile, dataset)
    validation = validate_profile_candidate(candidate, dataset)

    print(
        f"Validation: ok={validation.ok} rmse={validation.rmse_m:.6f}m "
        f"max_abs={validation.max_abs_m:.6f}m samples={validation.sample_count}"
    )
    if not validation.ok:
        print("Calibration candidate did not satisfy thresholds; promotion aborted.")
        return 2

    store = ProfilePromotionStore(Path(args.store))
    profile_id = store.create_draft(candidate)
    store.mark_validated(profile_id, validation)
    store.approve(profile_id)
    print(f"Profile promoted to APPROVED: {profile_id}")
    if args.activate:
        store.activate(profile_id, actor=args.actor)
        print(f"Profile set ACTIVE: {profile_id}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

