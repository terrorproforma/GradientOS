## 2026-02-21 03:00 +11:00

- Task summary:
  - Detached `gradient-05` robot configuration from the placeholder `gradient0` setup and created its own fully explicit configuration file.
  - Extracted URDF limits for `gradient-05` and ensured they act as the runtime source of truth for joint limits in Python.
  - Ensured the simulator backend initializes properly when run against the new config.
- Changes:
  - Wrote a new `scripts/sync_urdf_limits.py` helper script to extract `<limit lower="..." upper="..."/>` directly from `gradient-05.urdf` and sync them into the Python config.
  - Replaced `robots/gradient05/config.py` content to fully implement `RobotConfig` independently.
  - Mapped EtherCAT RTCore axes 0-5 directly instead of using twin motor IDs (as they existed on `gradient0`).
  - Set `ethercat_rtcore` and `numeric` QuIK solver as the defaults for the 05 config.
  - Fixed an unhandled `None` case in `SimulationBackend._read_single_actuator_position` that caused `TypeError` when checking length bounds against a `None` gripper actuator id.
- Validation:
  - `python scripts/sync_urdf_limits.py` successfully updated `config.py` with limits extracted from the URDF: `[(-6.3, 6.3), (-1.9, 1.9), (-4.2, 1.53), (-6.3, 6.3), (-6.3, 6.3), (-6.3, 6.3)]`.
  - Ran `.\run-sim.ps1` to confirm `gradient-05` config correctly loads and initializes the simulator backend without error.
- Follow-up notes / risks:
  - If the physical design updates the URDF bounds, developers should re-run `python scripts/sync_urdf_limits.py` to ensure the Python runtime matches the CAD limits.