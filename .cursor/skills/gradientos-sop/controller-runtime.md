# Controller Runtime

Primary SOP sections: `§3.2-3.5`, `§5-8`

Use this file when changing controller startup, LIVE/SIM behavior, IPC usage, motion state handling, or command reuse.

## Controller Rules

- Keep one activation path for startup and hot-switch; do not fork separate initialization flows.
- The controller owns runtime selection and handoff sequencing.
- Startup should land synchronized and disarmed before any operator power-up.
- The controller should wait for RTCore readiness when the active backend is `ethercat_rtcore`.

## LIVE and SIM

- LIVE/SIM switching is controller-owned, not UI-owned.
- The UI may request a mode change, but the controller must stage, restart, and reinitialize the active backend correctly.
- Do not let a runtime switch preserve stale backend-specific telemetry assumptions.
- Preserve generic controller semantics across backends even when backend-native commands differ.

## Python <-> RTCore IPC

- Use the existing handshake, socket, ring, and status publication model.
- Add new RTCore commands/status only through the established IPC ABI.
- Keep RT-critical logic in RTCore, but keep orchestration and operator semantics in Python.

## Motion Truth

- Completion truth must come from backend/RTCore status, not only ACK text or request success.
- Treat trajectories and jog as distinct motion modes with distinct controller semantics.
- Scheduled trajectories should be committed and replayed by RTCore, not streamed point-by-point from Python.
- Jog commands should use the controller's existing command pathways and RTCore's timeout/hold behavior.

## Reuse Rules

- Reuse existing controller commands before adding new endpoints or frontend-specific variants.
- Reuse the same execution-state pipeline for startup, manual controls, and program execution whenever possible.
- When a new commissioning flow needs movement, route it through the same backend abstractions and safety checks as normal motion.

## First Files

- `src/gradient_os/run_controller.py`
- `src/gradient_os/arm_controller/command_api.py`
- `src/gradient_os/arm_controller/backends/ethercat_rtcore/backend.py`
- `src/gradient_os/arm_controller/backends/registry.py`
- `src/gradient_os/runtime_config.py`
- `src/gradient_rt_motion/ipc_v1.hpp`
