---
name: gradientos-sop
description: GradientOS architecture and operating-principles router for the controller, RTCore, EtherCAT, API, frontend, telemetry, commissioning, and config ownership. Use when working on the GradientOS motion stack, deciding where new logic belongs, extending telemetry or command paths, changing startup/runtime behavior, or touching safety-critical control flows.
---

# GradientOS SOP

Use this skill when a task touches the GradientOS control stack or when you need to decide where a change belongs.

## Canonical Source

The long-form canonical SOP/source document for this skill is:

- `.cursor/skills/gradientos-sop/RTOS_ETHERCAT_MASTER_OPERATING_PRINCIPLES.md`

Treat this skill as the compact routed operational layer derived from that document, not as a replacement for it.

## Quick Start

1. Identify the target layer before editing:
   - robot/config definition
   - backend/profile/runtime selection
   - Python controller policy/orchestration
   - RTCore deterministic execution
   - API/frontend presentation
2. Reuse existing command, telemetry, and IPC pathways before adding new ones.
3. Keep ownership boundaries sharp:
   - Python owns policy, planning, orchestration, and operator-facing semantics.
   - RTCore owns deterministic timing, EtherCAT execution truth, and RT-safe motion handling.
   - API/frontend stay thin and should not invent controller or drive semantics.
4. Prefer extending existing payloads and status objects over creating parallel contracts.
5. For architecture or SOP updates, also read [skill-maintenance-policy.md](./skill-maintenance-policy.md).

## Routing

Read the file that matches the task:

- [architecture-boundaries.md](./architecture-boundaries.md)
  Use for layer ownership, feature placement, mental model, and anti-duplication rules.
- [controller-runtime.md](./controller-runtime.md)
  Use for controller activation, LIVE/SIM switching, IPC, motion state, and command-path reuse.
- [rtcore-ethercat.md](./rtcore-ethercat.md)
  Use for RTCore, EtherCAT master behavior, drive startup config, fault telemetry, and startup readiness.
- [ui-api-telemetry.md](./ui-api-telemetry.md)
  Use for frontend/API thinness, telemetry extension rules, and generic payload contracts.
- [config-and-drive-profiles.md](./config-and-drive-profiles.md)
  Use for robot config, backend config, runtime selection, and EtherCAT drive-profile ownership.
- [commissioning-safety.md](./commissioning-safety.md)
  Use for startup state, power transitions, zero vs native home, conservative commissioning flows, and encoder retention checks.
- [validation-and-debugging.md](./validation-and-debugging.md)
  Use for validation strategy, build/test commands, and debugging by layer.

## Active Workstream Notes

- [../../../docs/ethercat/a6ec-frame-semantics-and-native-home.md](../../../docs/ethercat/a6ec-frame-semantics-and-native-home.md)
  Use for the A6-EC frame-semantics and native-home workstream note: probe rationale, raw vs reference vs rotation frame separation, anchor math, persistence lessons, UI trust implications, and remaining open questions.
  Durable project doc; the settled rules from this workstream are promoted into the master principles doc under `§9.4`-`§9.7` and into [commissioning-safety.md](./commissioning-safety.md).

## Non-Negotiable Rules

- Do not duplicate comms or telemetry pathways if an existing payload can carry the new data.
- Do not move policy into RTCore just because RTCore is closer to the hardware.
- Do not move execution truth into Python just because it is easier to inspect there.
- Do not put drive-family specifics in generic controller or RTCore layers when they belong in drive profiles/catalog data.
- Do not put frontend/API defaults where the controller already owns the behavior.
- Do not canonize temporary experiments into the GradientOS skill set; keep those in scratchpad or devlog until the pattern is validated.

## First Files To Inspect

- `.cursor/skills/gradientos-sop/RTOS_ETHERCAT_MASTER_OPERATING_PRINCIPLES.md`
- `src/gradient_os/run_controller.py`
- `src/gradient_os/arm_controller/command_api.py`
- `src/gradient_os/arm_controller/backends/ethercat_rtcore/backend.py`
- `src/gradient_os/arm_controller/backends/ethercat_rtcore/runtime.py`
- `src/gradient_os/arm_controller/ethercat_drive_catalog.py`
- `src/gradient_rt_motion/main.cpp`
- `src/gradient_rt_motion/ipc_v1.hpp`
- `src/gradient_os/api/main.py`
- `src/gradient_os/telemetry/drive_faults.py`
- `web-ui/src/App.tsx`
- `web-ui/src/ControlPanel.tsx`

## Maintenance

The canonical GradientOS skill is intentionally slower-moving than `.cursor/memory/AGENT_SCRATCHPAD.md` and `.cursor/memory/DEVLOG.md`.

- Put fast-moving discoveries, mistakes, and temporary guardrails in the scratchpad.
- Put chronological implementation and validation evidence in the devlog.
- Update this skill set only for architecture changes or workstreams that are complete and validated enough to become stable operating guidance.
