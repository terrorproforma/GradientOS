# Validation and Debugging

Primary SOP sections: `§3.5`, `§13`, `§17-19`

Use this file after substantive changes or when debugging controller, RTCore, EtherCAT, API, or UI behavior.

## Validation Strategy

- Validate in the layer you changed first.
- Prefer focused tests and targeted build checks over broad ritual commands.
- Do not claim validation you did not run.
- If behavior depends on runtime services or hardware, say so explicitly.

## Typical Checks

- Python/backend/controller paths:
  - `source "/home/pi/GradientOS/.venv/bin/activate" && PYTHONPATH=src python -m pytest ...`
  - `source "/home/pi/GradientOS/.venv/bin/activate" && PYTHONPATH=src python -m py_compile ...`
- RTCore:
  - `make -C src/gradient_rt_motion`
- Frontend:
  - `npm run build` in `web-ui`
  - `ReadLints` on touched TS/TSX files

Choose the narrowest command set that actually proves the change.

## Live Bring-Up Loops

- For iterative live EtherCAT testing, prefer `./start-stack.sh probe`, then `./start-stack.sh stop --hard`, then `./start-stack.sh` over rebooting between every healthy cycle.
- A reboot is not required when `stop --hard` tears the host down cleanly and the post-stop probe lands `physical_state=INACTIVE`, `ethercat_master_state=DOWN`, and `rtcore_state=DOWN`.
- Use `./start-stack.sh probe` before and after each cycle to distinguish a healthy teardown from a poisoned host.
- If a run leaves stale-owner symptoms such as `ecrt_request_master(0)` busy, `Master already in use`, a hung kernel task in the RTCore/metrics path, or `stop --hard` failing to clear EtherCAT ownership, treat the host as poisoned and reboot before the next live bring-up attempt.
- A single launcher-managed RTCore/EtherCAT recycle during startup recovery is acceptable if the stack still reaches `BUS_UP_DISARMED`; do not treat that recovery path alone as proof that a reboot is required.

## Debug by Layer

- Fieldbus bring-up issue: inspect EtherCAT host, RTCore readiness, and metrics before blaming Python.
- Controller orchestration issue: inspect runtime selection, backend activation, and command/state flow.
- UI/API issue: inspect normalized payload shape and the existing source object before adding new fields.
- RT completion issue: prefer status-driven reasoning over request ACK text alone.

## Common Lessons

- A healthy software stack can still hide a dead fieldbus.
- Slave discovery is not the same as operational motion readiness.
- Very short trajectories can complete between polls.
- Backend switching can preserve stale assumptions if telemetry/config is cached too early.
- STOP and restart logic need explicit backend-aware handling.
- Vendor persistence claims do not always match firmware behavior. Validate `bit 15` retention across an actual drive power cycle before trusting it; on the current A6-EC firmware `bit 15` is cleared every cycle even though `6064`, `U40.20/.22`, and `607C` all persist cleanly.

## A6-EC Command-Frame Failure Modes

`Er87.1` (excessive position reference increment, > 5x max speed) and `Er47.0` (following error) on seam-adjacent small jogs are almost always a host-side command-frame turn-selection bug, not drive misconfiguration. Diagnostic path:

- Confirm the trajectory upload payload's per-axis `607A` step size (host `command_frame_oversized_step` gate would have refused a genuine whole-turn jump before commit).
- Confirm live `6064` vs the host's `canonical_q * sign * counts_per_unit + master_offset` agrees modulo `RM` within 16 counts (`shaft_frame_consistent` in axis detail). A large `shaft_frame_mod_rm_delta_counts` signals a frame mismatch, not a drive fault.
- Do not blame vendor scaling (`6091` / `C10.18` / `C10.19`) before confirming the command frame is well-formed; these are already validated by startup readback.

## Power-Cycle Persistence Workflow

When testing that a joint retains its homed state across a real drive power cycle, follow the encoder-retention workflow:

- Capture a pre-cycle snapshot via `scripts/a6ec_chapter5_probe.py snapshot --label pre-power-cycle --axes J1 ... J6 --experiment-id <name>` so all raw SDO reads, RTCore metrics, API truth, and monitor samples land in one artifact under `logs/encoder-retention/<experiment-id>/`.
- After the cycle, capture a matching `post-power-cycle` snapshot.
- Diff the key witnesses: `6041` statusword (especially bit 15), `6064`, `607C`, `U40.16`, `U40.20 / U40.22`. Expect wander of `0..3` counts on `6064` / `U40.20/.22` stationary; anything larger than the 16-count shaft-frame tolerance without a matching mechanical explanation is a real divergence.
- Track `drive_native_truth_verification_source` across the cycle. A switch from `statusword_bits12_15_clear13` (fresh home) to `persisted_home_anchor_agreement` (restart-trust fallback) on the same joint without a re-home is expected on drives that clear bit 15 and is the canonical success case.

## Debug Artifacts

- Prefer current logs, metrics, and existing status payloads before adding new debug channels.
- When possible, add or extend focused tests that lock down the discovered failure mode.
- For the active A6-EC frame-semantics workstream, use `scripts/a6ec_chapter5_probe.py` plus [../../../docs/ethercat/a6ec-frame-semantics-and-native-home.md](../../../docs/ethercat/a6ec-frame-semantics-and-native-home.md) to capture raw, reference, rotation-mode, and API truth views in one snapshot across `boot -> native-home -> power cycle -> restart`.

## First Files

- `.cursor/skills/gradientos-sop/RTOS_ETHERCAT_MASTER_OPERATING_PRINCIPLES.md`
- `docs/ethercat/bringup.md`
- `src/gradient_os/run_controller.py`
- `src/gradient_rt_motion/main.cpp`
- `tests/`
