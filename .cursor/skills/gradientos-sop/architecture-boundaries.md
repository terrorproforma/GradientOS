# Architecture Boundaries

Primary SOP sections: `§1-2`, `§12A`, `§14-16`, `§20-21`

Use this file to decide where new logic belongs and to avoid duplicating existing pathways.

## Core Layer Ownership

- Python controller owns orchestration, planning, IK, runtime policy, program sequencing, and operator-facing motion semantics.
- RTCore owns deterministic timing, ring consumption, jog/trajectory execution, DS402-safe target handling, and execution truth.
- EtherCAT host/master setup owns NIC binding, IRQ/NIC tuning, slave discovery, and stable process-data transport.
- API and frontend present and transport existing controller semantics; they should not create new control truth.

## Feature Placement Rules

- Put logic in the frontend if it is presentation, local interaction flow, or transient UI state.
- Put logic in the API if it is request parsing, response shaping, or cross-process transport glue.
- Put logic in the controller if it is planning, runtime policy, state orchestration, sequencing, or backend-agnostic motion semantics.
- Put logic in RTCore if it must execute deterministically at RT cadence or requires direct fieldbus control.
- Put logic in backend/profile layers if it differs by backend or drive family but should still plug into generic controller pathways.

## Anti-Duplication Rules

- Extend existing payloads before creating a new endpoint, stream, or status object.
- Extend existing IPC/status structs before inventing side channels.
- Reuse existing controller commands for commissioning actions when possible.
- Keep a single owner for each kind of truth:
  - motion completion truth from RTCore/backend status
  - runtime selection truth from the controller/runtime config
  - drive-family policy from drive profiles and the EtherCAT drive catalog

## Config Ownership

- Robot config owns mechanics, scaling, geometry, tools, and robot-specific policy.
- Backend config owns backend-specific behavior and compatibility shims.
- EtherCAT drive catalog owns drive-family identity, PDO layouts, sync indices, startup SDO defaults, and similar manufacturer policy.
- Runtime config selects which robot, backend, and mode are active; it is not the authoritative definition of those things.

## Mental Model

Think in three questions before editing:

1. Who should decide this behavior?
2. Who must execute it deterministically?
3. Which existing contract already transports it?

If those answers point to different layers, split the change instead of forcing one layer to do everything.

## First Files

- `src/gradient_os/run_controller.py`
- `src/gradient_os/arm_controller/command_api.py`
- `src/gradient_os/arm_controller/backends/registry.py`
- `src/gradient_os/arm_controller/ethercat_drive_catalog.py`
- `src/gradient_rt_motion/main.cpp`
- `src/gradient_rt_motion/ipc_v1.hpp`
- `src/gradient_os/api/main.py`
- `web-ui/src/App.tsx`
