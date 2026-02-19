from pathlib import Path

from gradient_os import robot_assets
from gradient_os.kinematics.calibration import (
    CalibrationDataset,
    CalibrationSample,
    CalibrationValidationResult,
    ProfilePromotionStore,
    build_profile_candidate_from_fit,
    save_dataset,
    load_dataset,
    validate_profile_candidate,
)


def _build_dataset() -> CalibrationDataset:
    return CalibrationDataset(
        robot_id="mini-6dof-arm",
        robot_serial="SERIAL-001",
        session_id="sess-001",
        samples=(
            CalibrationSample(
                commanded_xyz_m=(0.40, 0.00, 0.20),
                measured_xyz_m=(0.395, -0.001, 0.198),
                tool_id="torch-a",
            ),
            CalibrationSample(
                commanded_xyz_m=(0.35, 0.05, 0.22),
                measured_xyz_m=(0.345, 0.049, 0.218),
                tool_id="torch-a",
            ),
        ),
        metadata={"operator": "test"},
    )


def test_calibration_dataset_roundtrip(tmp_path: Path):
    dataset = _build_dataset()
    output = tmp_path / "dataset.json"
    save_dataset(dataset, output)
    loaded = load_dataset(output)
    assert loaded.robot_id == dataset.robot_id
    assert len(loaded.samples) == len(dataset.samples)


def test_fit_validate_and_promotion_workflow(tmp_path: Path):
    dataset = _build_dataset()
    base = robot_assets.load_kinematics_profile("mini-6dof-arm")
    candidate = build_profile_candidate_from_fit(base, dataset)
    validation = validate_profile_candidate(candidate, dataset)
    assert validation.ok
    assert isinstance(validation, CalibrationValidationResult)

    store = ProfilePromotionStore(tmp_path / "store")
    pid1 = store.create_draft(candidate)
    store.mark_validated(pid1, validation)
    store.approve(pid1)
    store.activate(pid1, actor="pytest")

    # Create and activate a second profile so rollback has a prior target.
    candidate2 = dict(candidate)
    candidate2["profile_id"] = "mini-6dof-arm:alt"
    pid2 = store.create_draft(candidate2)
    store.mark_validated(pid2, validation)
    store.approve(pid2)
    store.activate(pid2, actor="pytest")

    rolled_back_to = store.rollback(actor="pytest")
    assert rolled_back_to == pid1

