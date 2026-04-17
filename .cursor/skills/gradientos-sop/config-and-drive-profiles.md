# Config and Drive Profiles

Primary SOP sections: `§12`, `§12A`

Use this file when deciding whether behavior belongs in robot config, backend config, runtime config, or the EtherCAT drive catalog.

## Source-of-Truth Hierarchy

- Robot definitions own mechanics, joint scaling, geometry, kinematics, tools, and robot-specific policy.
- Backend config owns backend-specific integration behavior and compatibility details.
- Drive profiles and the EtherCAT drive catalog own drive-family policy and manufacturer metadata.
- Runtime config selects the active robot, backend, and mode; it should not become a second source of robot or drive truth.

## EtherCAT Drive Rules

- Keep drive identity, PDO defaults, sync indices, DC timing, startup SDO defaults, and similar family-level policy out of robot config.
- The same robot may be paired with different drive families, so robot config must stay manufacturer-neutral.
- Render RTCore startup env from a merge of robot/runtime policy plus drive-catalog data.
- For the active A6-EC frame-semantics workstream, keep the raw absolute encoder frame, the rotation-mode bridge frame, and the CSP/reference-home frame explicitly separate.
- See [../../../docs/ethercat/a6ec-frame-semantics-and-native-home.md](../../../docs/ethercat/a6ec-frame-semantics-and-native-home.md) for the durable workstream note that captures the current equations, object families, probe rationale, and anchor semantics.

## Drive-Profile Position-Semantics Flags

Drive profiles declare the intended truth model and restart trust policy via `POSITION_SEMANTICS_CONFIG`. Current A6-EC values and semantics (see also `src/gradient_os/arm_controller/profiles/drive/a6ec_ds402.py`):

- `drive_native_ratio_enabled`: `C10.18 / C10.19` is programmed in the drive so `6064` / `U40.28` are output-shaft-scaled. Host `counts_per_unit` uses the same mechanical ratio so no double-scaling is applied.
- `position_semantics_source = "drive_output_shaft"`: what the published canonical frame is semantically.
- `canonical_truth_source = "encoder_multi_turn_counts"`: which feedback family the controller uses as the authoritative multi-turn truth. Alternative `"drive_reference_frame"` means `6064`-only (valid only for single-turn-bounded joints; Gradient-05 joints are NOT bounded that way).
- `absolute_home_anchor_required`: the `.gradient_absolute_encoder_anchors.json` file must contain an entry for each joint before canonical truth can be published. A missing anchor fails closed.
- `startup_truth_requires_hm_success_signature`: when `True`, every boot requires the vendor `bit 12 ∧ bit 15 ∧ ¬bit 13` HM-success signature. When `False`, a retained `bit 15` alone is enough for restart trust per vendor Q9. Vestigial on drives that clear bit 15 every power cycle.
- `accept_persisted_home_anchor_as_restart_trust`: when `True`, a missing `bit 15` on restart is acceptable if the persisted absolute-home anchor still agrees with live `6064` modulo `RM` within tolerance and `U40.20/.22` reports valid multi-turn data. Initial HM35 is still required to establish the anchor entry; this flag only controls whether subsequent power cycles can reuse it.

New drive families should mirror this pattern rather than inventing new position-semantics fields, so the controller's canonical-truth path stays vendor-neutral.

## Backend Rules

- Backend behavior should operate on logical joints unless there is a hard physical reason not to.
- Keep servo-specific and fieldbus-specific details in backend/profile layers rather than in generic robot manifests.
- Maintain honest capability advertisement so controller code can make correct decisions.

## Tool and Geometry Rules

- Keep robot geometry and tool/TCP definitions in their intended configuration layers.
- Do not bury tool offsets, kinematic parameters, or robot-specific scaling inside UI code or ad hoc runtime helpers.

## Templates for New Work

- New robot: add a robot definition/config first.
- New backend: add backend config/registry integration first.
- New EtherCAT drive family: add or extend the drive catalog and drive profile first, then thread it through runtime loading.

## First Files

- `src/gradient_os/arm_controller/robots/base.py`
- `src/gradient_os/arm_controller/robots/`
- `src/gradient_os/arm_controller/backends/`
- `src/gradient_os/arm_controller/profiles/drive/`
- `src/gradient_os/arm_controller/ethercat_drive_catalog.py`
- `src/gradient_os/arm_controller/backends/ethercat_rtcore/runtime.py`
- `src/gradient_os/runtime_config.py`
