"""Kinematics profile and runtime utilities."""

from .profile import (
    KinematicsProfile,
    KinematicsProfileError,
    KinematicsProfileErrorCode,
    build_profile_from_payload,
    identity_matrix4,
    validate_profile_for_backend,
)
from .runtime import (
    RuntimeKinematicsError,
    apply_profile_payload,
    apply_runtime_to_fk_matrix,
    compensate_target_pose_for_runtime,
    get_runtime_matrices,
    get_runtime_state_snapshot,
    patch_runtime_offsets,
    reset_runtime_offsets,
)
from .calibration import (
    CalibrationDataset,
    CalibrationSample,
    CalibrationThresholds,
    CalibrationValidationResult,
    ProfilePromotionStore,
    build_profile_candidate_from_fit,
    fit_tool_runtime_translation,
    load_dataset,
    save_dataset,
    validate_profile_candidate,
)

__all__ = [
    "KinematicsProfile",
    "KinematicsProfileError",
    "KinematicsProfileErrorCode",
    "build_profile_from_payload",
    "identity_matrix4",
    "validate_profile_for_backend",
    "RuntimeKinematicsError",
    "apply_profile_payload",
    "apply_runtime_to_fk_matrix",
    "compensate_target_pose_for_runtime",
    "get_runtime_matrices",
    "get_runtime_state_snapshot",
    "patch_runtime_offsets",
    "reset_runtime_offsets",
    "CalibrationDataset",
    "CalibrationSample",
    "CalibrationThresholds",
    "CalibrationValidationResult",
    "ProfilePromotionStore",
    "build_profile_candidate_from_fit",
    "fit_tool_runtime_translation",
    "load_dataset",
    "save_dataset",
    "validate_profile_candidate",
]

