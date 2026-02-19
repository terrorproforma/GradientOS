import pytest

from gradient_os import robot_assets
from gradient_os.kinematics.profile import (
    KinematicsProfileError,
    KinematicsProfileErrorCode,
    build_profile_from_payload,
    validate_profile_for_backend,
)


def test_legacy_manifest_bridge_produces_valid_profile():
    payload = robot_assets.get_kinematics_profile_payload("mini-6dof-arm")
    profile = build_profile_from_payload(payload, expected_robot_id="mini-6dof-arm")
    assert profile.robot_id == "mini-6dof-arm"
    assert profile.schema_version == 1
    assert profile.base_nominal.shape == (4, 4)
    assert profile.tool_nominal.shape == (4, 4)


def test_profile_checksum_validation_rejects_tampered_payload():
    payload = robot_assets.get_kinematics_profile_payload("mini-6dof-arm")
    profile = build_profile_from_payload(payload, expected_robot_id="mini-6dof-arm")
    tampered = dict(profile.to_payload())
    tampered["tool_runtime"] = [
        [1.0, 0.0, 0.0, 0.01],
        [0.0, 1.0, 0.0, 0.00],
        [0.0, 0.0, 1.0, 0.00],
        [0.0, 0.0, 0.0, 1.00],
    ]
    with pytest.raises(KinematicsProfileError) as exc:
        build_profile_from_payload(tampered, expected_robot_id="mini-6dof-arm")
    assert exc.value.code == KinematicsProfileErrorCode.INVALID_CHECKSUM


def test_backend_compatibility_rejects_incompatible_backend():
    payload = robot_assets.get_kinematics_profile_payload("mini-6dof-arm")
    payload["backend_compatibility"] = ["numeric"]
    profile = build_profile_from_payload(payload, expected_robot_id="mini-6dof-arm")
    with pytest.raises(KinematicsProfileError) as exc:
        validate_profile_for_backend(profile, "ikfast")
    assert exc.value.code == KinematicsProfileErrorCode.BACKEND_INCOMPATIBLE

