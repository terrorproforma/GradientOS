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

## Jog Control Loop

`_jog_controller_thread` in `command_api.py` runs at `JOG_CONTROL_FREQUENCY_HZ = 50 Hz` and must preserve a lean hot path from feedback read to command send. Anything that does not gate motion control is deferred until after the command reaches RTCore.

- Hot path (target `<5 ms`): `servo_driver.get_current_arm_state_rad` → commanded-pose extraction from the cached control dict → target pose math → `ik_solver.solve_ik` → safety gate (`_validate_jog_step_candidate`) → `update_joint_velocity_lease_jog`.
- Deferred path (runs AFTER `update_joint_velocity_lease_jog`): `_JOG_SESSION_MANAGER.accept_command_step`, the `_build_jog_command_state_perf_fields` / `_jog_perf_update` chain, the `_build_jog_ik_debug_payload` helper (pulled out of the inline dict for reuse), `mark_seq_applied`, `_safe_session_call(update_following_error, ...)`. Identical values and state updates as before; only the ordering relative to `update_joint_velocity_lease_jog` changed.
- Gate-failure path keeps its telemetry inline because no motion is happening; latency there is not operator-visible.
- Do not reintroduce per-tick work before command send unless it is strictly required for safety (IK + step validation) or for the target-pose computation itself. Telemetry-only work belongs in the deferred block.

### Jog arm-time canonical-truth fast-path

`handle_jog_session_start` previously ran a strict `get_current_arm_state_rad()` with up to `500 ms` of retry on every click. After a jog release, the drive is still decelerating and the drive-native truth gates (`drive_native_command_frame_roundtrip_mismatch`, `drive_native_persisted_home_anchor_inconsistent_with_live_6064`) transiently fail, so click-after-release ate the entire retry budget — the dominant source of operator-perceived jog lag.

Contract:

- `_note_valid_canonical_truth()` stamps `_LAST_VALID_CANONICAL_TRUTH_MONOTONIC` on every successful feedback read inside `_jog_controller_thread` (initial read, resume-after-motion, and per-tick hot-path read). Each successful `get_current_arm_state_rad()` at 50 Hz keeps the timestamp within `~20 ms` of fresh during any active session and for several seconds after release.
- `handle_jog_session_start` calls `_recently_valid_canonical_truth()` first. If the stamp is within `_JOG_ARM_RECENT_TRUTH_WINDOW_S` (currently `5.0 s`), the strict retry loop is skipped entirely and the session arms immediately. Phase 0's per-tick `try/except RuntimeError` around `get_current_arm_state_rad` absorbs any transient flicker that occurs during the newly started session.
- The strict retry path (`_JOG_ARM_TRUTH_RETRY_BUDGET_S`, `_JOG_ARM_TRUTH_RETRY_INTERVAL_S`) still runs when the cached timestamp is missing or stale (first boot, post-power-cycle, long idle), so the safety intent of the original check is preserved: a real encoder-retention fault will NOT have produced a recent valid reading.

When adding new jog entry points (e.g., commissioning flows that invoke the jog thread), call `_note_valid_canonical_truth()` on every successful feedback read so the fast-path stays warm across mode transitions.

### Session-manager race safety

`_JOG_SESSION_MANAGER` mutations that require a live record (`update_following_error`, `resync_command_state`, `record_gate_failure`) can race with a `JOG_SESSION_STOP` / lease expiry / controller-stop that lands between the top-of-loop `session_active` check and the later mutation. The raw mutation would raise `JogSessionError("SESSION_INACTIVE")` and kill the daemon thread.

Contract:

- Every raise-prone session-manager mutation inside `_jog_controller_thread` must be wrapped with the `_safe_session_call(...)` helper defined in that thread. The helper absorbs `SESSION_INACTIVE` into a sentinel return and lets the caller fall through to a clean `break`; any other `JogSessionError` code still propagates as a real bug.
- The regression test `test_jog_thread_source_contains_session_inactive_race_guard` enforces the wrapping at the source level so refactors cannot silently drop it.

### Metrics.json hot-path cache

`EthercatRTCoreBackend._load_rtcore_metrics_snapshot()` is on the hot path (called from `_canonical_joint_positions_from_raw_feedback`, which runs on every feedback read across the controller status poll, jog thread, and API endpoints). The raw implementation would disk-read + `json.loads` the `~11 KB` metrics file on every call. Cache:

- `self._rtcore_metrics_cache` holds the last parsed payload keyed by `mtime_ns` from `stat`. Calls within the same RTCore write cycle (`~200 ms`) hit the cache in `<1 ms`.
- Any new consumer of metrics.json in the controller should go through `_load_rtcore_metrics_snapshot()` (not re-read the file directly) so the cache benefits persist.

## First Files

- `src/gradient_os/run_controller.py`
- `src/gradient_os/arm_controller/command_api.py`
- `src/gradient_os/arm_controller/backends/ethercat_rtcore/backend.py`
- `src/gradient_os/arm_controller/backends/registry.py`
- `src/gradient_os/runtime_config.py`
- `src/gradient_rt_motion/ipc_v1.hpp`
