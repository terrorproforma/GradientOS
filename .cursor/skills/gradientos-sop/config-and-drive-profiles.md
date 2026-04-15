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
