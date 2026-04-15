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
