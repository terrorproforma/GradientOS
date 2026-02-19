from __future__ import annotations

from dataclasses import dataclass, asdict
import json
from pathlib import Path
from typing import Any
import uuid

import numpy as np

from .profile import KinematicsProfile, build_profile_from_payload


@dataclass(frozen=True)
class CalibrationSample:
    commanded_xyz_m: tuple[float, float, float]
    measured_xyz_m: tuple[float, float, float]
    tool_id: str
    temperature_c: float | None = None


@dataclass(frozen=True)
class CalibrationDataset:
    robot_id: str
    robot_serial: str
    session_id: str
    samples: tuple[CalibrationSample, ...]
    metadata: dict[str, Any]


@dataclass(frozen=True)
class CalibrationThresholds:
    max_rmse_m: float = 0.008
    max_abs_m: float = 0.02


@dataclass(frozen=True)
class CalibrationValidationResult:
    ok: bool
    rmse_m: float
    max_abs_m: float
    sample_count: int
    details: dict[str, Any]


def _sample_to_dict(sample: CalibrationSample) -> dict[str, Any]:
    return asdict(sample)


def _sample_from_dict(payload: dict[str, Any]) -> CalibrationSample:
    cmd = payload.get("commanded_xyz_m")
    meas = payload.get("measured_xyz_m")
    if not isinstance(cmd, (list, tuple)) or len(cmd) != 3:
        raise ValueError("Each sample requires commanded_xyz_m [x,y,z].")
    if not isinstance(meas, (list, tuple)) or len(meas) != 3:
        raise ValueError("Each sample requires measured_xyz_m [x,y,z].")
    tool_id = str(payload.get("tool_id", "")).strip()
    if not tool_id:
        raise ValueError("Each sample requires non-empty tool_id.")
    temperature = payload.get("temperature_c")
    temperature_c = float(temperature) if temperature is not None else None
    return CalibrationSample(
        commanded_xyz_m=(float(cmd[0]), float(cmd[1]), float(cmd[2])),
        measured_xyz_m=(float(meas[0]), float(meas[1]), float(meas[2])),
        tool_id=tool_id,
        temperature_c=temperature_c,
    )


def save_dataset(dataset: CalibrationDataset, output_path: Path) -> None:
    payload = {
        "robot_id": dataset.robot_id,
        "robot_serial": dataset.robot_serial,
        "session_id": dataset.session_id,
        "samples": [_sample_to_dict(s) for s in dataset.samples],
        "metadata": dataset.metadata,
    }
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(json.dumps(payload, indent=2), encoding="utf-8")


def load_dataset(path: Path) -> CalibrationDataset:
    payload = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(payload, dict):
        raise ValueError("Calibration dataset must be a JSON object.")
    samples_raw = payload.get("samples")
    if not isinstance(samples_raw, list) or not samples_raw:
        raise ValueError("Calibration dataset requires non-empty samples array.")
    samples = tuple(_sample_from_dict(item) for item in samples_raw)
    return CalibrationDataset(
        robot_id=str(payload.get("robot_id", "")).strip(),
        robot_serial=str(payload.get("robot_serial", "")).strip(),
        session_id=str(payload.get("session_id", "")).strip(),
        samples=samples,
        metadata=dict(payload.get("metadata", {})),
    )


def fit_tool_runtime_translation(dataset: CalibrationDataset) -> np.ndarray:
    """
    Fit tool runtime translation that best maps measured end-point to commanded target.

    A positive fitted vector means commanded points should be shifted by this amount
    in tool frame compensation.
    """
    if not dataset.samples:
        raise ValueError("Calibration dataset has no samples.")
    residuals = []
    for sample in dataset.samples:
        commanded = np.asarray(sample.commanded_xyz_m, dtype=float)
        measured = np.asarray(sample.measured_xyz_m, dtype=float)
        residuals.append(commanded - measured)
    return np.mean(np.asarray(residuals, dtype=float), axis=0)


def build_profile_candidate_from_fit(
    base_profile: KinematicsProfile,
    dataset: CalibrationDataset,
) -> dict[str, Any]:
    translation = fit_tool_runtime_translation(dataset)
    candidate = base_profile.to_payload()
    tool_runtime = np.asarray(candidate["tool_runtime"], dtype=float).reshape(4, 4)
    tool_runtime[:3, 3] = tool_runtime[:3, 3] + translation
    candidate["tool_runtime"] = tool_runtime.tolist()
    candidate["version"] = f"{base_profile.version}+calib-{dataset.session_id}"
    candidate["metadata"] = {
        **dict(candidate.get("metadata", {})),
        "calibration_session_id": dataset.session_id,
        "calibration_sample_count": len(dataset.samples),
        "calibration_fit_translation_m": translation.tolist(),
    }
    candidate.pop("checksum", None)
    # Re-validate and auto-refresh checksum
    profile = build_profile_from_payload(candidate, expected_robot_id=dataset.robot_id)
    return profile.to_payload()


def validate_profile_candidate(
    candidate_payload: dict[str, Any],
    dataset: CalibrationDataset,
    *,
    thresholds: CalibrationThresholds = CalibrationThresholds(),
) -> CalibrationValidationResult:
    profile = build_profile_from_payload(candidate_payload, expected_robot_id=dataset.robot_id)
    tool_runtime = np.asarray(profile.tool_runtime, dtype=float).reshape(4, 4)
    translation = tool_runtime[:3, 3]

    errors = []
    for sample in dataset.samples:
        commanded = np.asarray(sample.commanded_xyz_m, dtype=float)
        measured = np.asarray(sample.measured_xyz_m, dtype=float)
        corrected = measured + translation
        errors.append(np.linalg.norm(corrected - commanded))
    errors_arr = np.asarray(errors, dtype=float)
    rmse = float(np.sqrt(np.mean(errors_arr ** 2)))
    max_abs = float(np.max(errors_arr))
    ok = rmse <= thresholds.max_rmse_m and max_abs <= thresholds.max_abs_m
    return CalibrationValidationResult(
        ok=ok,
        rmse_m=rmse,
        max_abs_m=max_abs,
        sample_count=len(errors),
        details={
            "thresholds": asdict(thresholds),
            "fit_translation_m": translation.tolist(),
        },
    )


class ProfilePromotionStore:
    """
    File-backed promotion workflow:
      draft -> validated -> approved -> active
    """

    def __init__(self, root: Path):
        self.root = root
        self.index_path = self.root / "index.json"
        self.profiles_dir = self.root / "profiles"
        self.root.mkdir(parents=True, exist_ok=True)
        self.profiles_dir.mkdir(parents=True, exist_ok=True)
        if not self.index_path.exists():
            self._write_index(
                {
                    "draft": [],
                    "validated": [],
                    "approved": [],
                    "active_profile_id": None,
                    "activation_history": [],
                }
            )

    def _read_index(self) -> dict[str, Any]:
        return json.loads(self.index_path.read_text(encoding="utf-8"))

    def _write_index(self, payload: dict[str, Any]) -> None:
        self.index_path.write_text(json.dumps(payload, indent=2), encoding="utf-8")

    def _profile_path(self, profile_id: str) -> Path:
        return self.profiles_dir / f"{profile_id}.json"

    def create_draft(self, profile_payload: dict[str, Any]) -> str:
        normalized_payload = dict(profile_payload)
        normalized_payload.pop("checksum", None)
        profile = build_profile_from_payload(normalized_payload)
        profile_id = profile.profile_id or f"profile-{uuid.uuid4().hex[:10]}"
        payload = profile.to_payload()
        payload["profile_id"] = profile_id
        self._profile_path(profile_id).write_text(json.dumps(payload, indent=2), encoding="utf-8")
        idx = self._read_index()
        if profile_id not in idx["draft"]:
            idx["draft"].append(profile_id)
        self._write_index(idx)
        return profile_id

    def mark_validated(self, profile_id: str, validation: CalibrationValidationResult) -> None:
        idx = self._read_index()
        if profile_id not in idx["draft"] and profile_id not in idx["validated"]:
            raise ValueError(f"Profile '{profile_id}' is not a draft.")
        if not validation.ok:
            raise ValueError("Cannot promote profile; validation failed.")
        if profile_id in idx["draft"]:
            idx["draft"].remove(profile_id)
        if profile_id not in idx["validated"]:
            idx["validated"].append(profile_id)
        self._write_index(idx)

    def approve(self, profile_id: str) -> None:
        idx = self._read_index()
        if profile_id not in idx["validated"] and profile_id not in idx["approved"]:
            raise ValueError(f"Profile '{profile_id}' is not validated.")
        if profile_id in idx["validated"]:
            idx["validated"].remove(profile_id)
        if profile_id not in idx["approved"]:
            idx["approved"].append(profile_id)
        self._write_index(idx)

    def activate(self, profile_id: str, *, actor: str) -> None:
        idx = self._read_index()
        if profile_id not in idx["approved"]:
            raise ValueError(f"Profile '{profile_id}' is not approved.")
        previous = idx.get("active_profile_id")
        idx["active_profile_id"] = profile_id
        idx["activation_history"].append(
            {"action": "activate", "profile_id": profile_id, "previous": previous, "actor": actor}
        )
        self._write_index(idx)

    def rollback(self, *, actor: str) -> str:
        idx = self._read_index()
        history = idx.get("activation_history", [])
        if not history:
            raise ValueError("No activation history available for rollback.")
        current = idx.get("active_profile_id")
        previous = None
        for row in reversed(history):
            if row.get("action") == "activate" and row.get("profile_id") == current:
                previous = row.get("previous")
                break
        if previous is None:
            raise ValueError("No prior active profile available for rollback.")
        idx["active_profile_id"] = previous
        idx["activation_history"].append(
            {"action": "rollback", "profile_id": previous, "from": current, "actor": actor}
        )
        self._write_index(idx)
        return str(previous)

