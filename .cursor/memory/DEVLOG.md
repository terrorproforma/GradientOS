## 2026-04-13 plan refinement

- Task summary:
  - Integrated the latest canonical-truth regression lessons into the lossless startup dashboard plan and repo memory.
- Changes:
  - Updated `/home/pi/.cursor/plans/lossless_startup_dashboard_422d79b5.plan.md`:
    - promoted single canonical truth and explicit no-fallback semantics into plan decisions
    - added integrated lessons covering startup anchor bootstrap, explicit unavailable states, compatibility-alias handling, and anchor-aware write-frame integrity
    - expanded the event-ledger shape to record canonical truth lifecycle and anchor-bootstrap events
    - expanded validation to cover no `cached_fallback`, blocked motion baselining without truth, and the nonzero-anchor inversion bug class behind the J3/J4 snap-back regression
  - Updated `.cursor/memory/AGENT_SCRATCHPAD.md` with durable guardrails from the latest regression thread.
- Validation:
  - Read the current lossless dashboard plan and the linked transcript to extract the stable lessons and fixes.
  - Confirmed the plan now explicitly distinguishes canonical truth from diagnostics and requires startup repair paths instead of encoder fallbacks.
- Follow-up notes / risks:
  - The later implementation pass should make the dashboard/event schema reflect these rules directly so operator-visible status cannot drift back into blank telemetry, hidden fallback, or misleading dual-truth presentation.

## 2026-04-13 00:00 +0000

- Task summary:
  - Extended the startup dashboard plan to make lossless recording a first-class design constraint.
- Changes:
  - Updated `/home/pi/.cursor/plans/lossless_startup_dashboard_422d79b5.plan.md`:
    - marked the recording contract todo complete and the split-view layout todo in progress
    - added non-negotiable invariants that dashboard cleanup must never discard evidence
    - defined the planned artifact set: raw service logs, rendered launcher transcript, structured dashboard events, and session metadata
    - added failure-proofing, retention shape, and validation criteria focused on postmortem reconstruction
  - Updated `.cursor/memory/AGENT_SCRATCHPAD.md` with the new user requirement that live dashboard filtering must remain a presentation-only layer.
- Validation:
  - Read the current plan, scratchpad, and latest devlog context before refining the plan.
  - Confirmed the plan now explicitly requires reconstructability of both raw process output and operator-facing dashboard state.
- Follow-up notes / risks:
  - The next implementation pass must preserve this contract in `start-stack.sh` and `src/gradient_os/telemetry/terminal_dashboard.py`; readability improvements must not be coupled to any data-dropping path.

## 2026-04-08 17:49 +0000

- Task summary:
  - Removed the introduced RTCore legacy-speed/single-point velocity shim, restored the pre-native-home degree-step commissioning jog assumptions, and kept the native-home frame fixes intact.
- Changes:
  - Updated `src/gradient_os/arm_controller/backends/ethercat_rtcore/backend.py`:
    - deleted the introduced legacy speed-to-velocity helper path (`_LEGACY_SERVO_SPEED_MAX` and related `_single_point_*` helpers)
    - restored direct RTCore one-point `set_joint_positions()` / `set_single_actuator_position()` uploads to position-only trajectory points
    - restored `prepare_sync_write_commands()` / `sync_write()` to the older backend-private point contract instead of the timed helper branch
    - removed the `get_joint_positions()` / `sync_read_positions()` freshness gate so controller joint snapshots can again provide mapped RTCore feedback for commissioning jog
    - preserved the post-commit setpoint-cache update, native-home feedback conversion, command-frame handling, and zero-capture logic
  - Updated `src/gradient_os/arm_controller/trajectory_execution.py`:
    - removed the timed sync-write fallback branch and restored the older backend sync-write preparation path when RTCore trajectory offload is not used
  - Updated `tests/test_gradient05_limits_and_backends.py`:
    - rewrote the direct RTCore setpoint tests to assert position-only trajectory points instead of invented `qd`
    - replaced the stale-feedback rejection test with coverage that connected RTCore reads still return mapped joint feedback for commissioning
    - kept cache-poisoning coverage for failed RTCore commit and preserved native-home frame/zero tests
  - Verified the existing commissioning-path regression coverage still anchors the intended flow:
    - `tests/test_command_api_direct_setpoint.py::test_handle_apply_joint_setpoint_can_start_bounded_joint_trajectory`
    - `tests/test_api_endpoints.py::test_control_joint_jog`
    - `tests/test_trajectory_execution_backends.py::test_open_loop_executor_offloads_rtcore_trajectory_backend`
- Validation:
  - `source ./start.sh && python -m pytest tests/test_gradient05_limits_and_backends.py tests/test_command_api_direct_setpoint.py tests/test_api_endpoints.py tests/test_trajectory_execution_backends.py -q -k 'applies_master_offsets_to_setpoints or sync_write_ignores_legacy_speed_and_accel or single_actuator_setpoint_emits_position_only_trajectory or set_joint_positions_does_not_advance_cache_on_commit_failure or connected_reads_return_feedback_without_freshness_gate or applies_native_home_offsets_to_feedback_but_not_command_targets or zero_capture_persists_joint_offsets or execute_joint_trajectory_enqueues_velocity_points or handle_apply_joint_setpoint_can_start_bounded_joint_trajectory or control_joint_jog or open_loop_executor_offloads_rtcore_trajectory_backend'`
  - `source ./start.sh && python -m py_compile src/gradient_os/arm_controller/backends/ethercat_rtcore/backend.py src/gradient_os/arm_controller/trajectory_execution.py tests/test_gradient05_limits_and_backends.py tests/test_command_api_direct_setpoint.py tests/test_trajectory_execution_backends.py tests/test_api_endpoints.py`
  - `ReadLints` on touched files returned clean.
- Follow-up notes / risks:
  - This restores the older commissioning-jog assumptions by letting connected RTCore reads flow back into `GET_JOINT_STATE`; confirm on hardware that the UI no longer falls back to cached-only feedback before trusting live commissioning again.
  - Rebuilding Python-side code alone does not update any deployed RTCore binary; live retest still depends on the running RTCore service state on the target machine.

## 2026-04-08 18:10 +0000

- Task summary:
  - Investigated whether the next Joint Commissioning J2 retest needs an RTCore/C++ rebuild or only a Python/controller retest.
- Changes:
  - No product code changed.
  - Updated `.cursor/memory/AGENT_SCRATCHPAD.md` with the verified deployment rule for RTCore commissioning follow-ups on this machine.
- Validation:
  - Read `start.sh` to confirm it only bootstraps the repo Python environment.
  - Reviewed the current diff in `src/gradient_os/arm_controller/backends/ethercat_rtcore/backend.py`, `src/gradient_os/arm_controller/trajectory_execution.py`, and `tests/test_gradient05_limits_and_backends.py`.
  - Searched `src/` for `_LEGACY_SERVO_SPEED_MAX`, `_resolved_single_point_motor_rpm`, `_single_point_joint_velocities`, and `_single_point_axis_velocity`; no matches remained.
  - Verified the live RTCore backend still uses `qd` only for real trajectory/jog paths, while `set_joint_positions()`, `set_single_actuator_position()`, `prepare_sync_write_commands()`, and `sync_write()` now use position-only one-point uploads for the compatibility path.
  - Read `systemd/rt-motion/gradient-rt-motion.service` and `systemd/rt-motion/sync-runtime.sh` to confirm the service runs `/usr/local/bin/gradient-rt-motion` and only picks up repo RTCore binary changes after an explicit sync/restart step.
- Follow-up notes / risks:
  - For this specific shim-removal follow-up, the next cautious live check should not require rebuilding RTCore unless we intentionally want to test separate dirty-branch C++ or RTCore service/env changes.
  - If the commissioning retest shows behavior that implicates RTCore execution rather than Python/controller logic, then reassess whether the installed RTCore binary/env needs to be rebuilt and synced before widening scope.

## 2026-04-08 18:15 +0000

- Task summary:
  - Clarified when the dirty-branch RTCore/EtherCAT-side changes do require a rebuild before commissioning.
- Changes:
  - No product code changed.
  - Updated `.cursor/memory/AGENT_SCRATCHPAD.md` to record that the current dirty-branch `main.cpp` / `ipc_v1.hpp` changes are not live until RTCore is rebuilt and synced into the installed service binary.
- Validation:
  - Reviewed `git diff` for `src/gradient_rt_motion/main.cpp` and `src/gradient_rt_motion/ipc_v1.hpp`.
  - Confirmed the dirty RTCore changes include native-home hold-target alignment to `native_home_position_offset`, refreshed offset readback on startup, and a new service-SDO-write IPC command path.
  - Confirmed again from `systemd/rt-motion/gradient-rt-motion.service` and `systemd/rt-motion/sync-runtime.sh` that these C++ changes only reach hardware after rebuild plus sync/restart of `/usr/local/bin/gradient-rt-motion`.
- Follow-up notes / risks:
  - This is an RTCore rebuild/sync question, not an IgH EtherCAT master rebuild question; no current diff evidence showed repo changes to the master package itself.
  - If the live retest intends to validate native-home alignment or service-SDO-write behavior, rebuilding and syncing RTCore is required before trusting the result.

## 2026-04-08 19:22 +0000

- Task summary:
  - Investigated why the Joint Commissioning panel locked out further jogs after a single J3 step on live RTCore hardware.
- Changes:
  - No product code changed.
  - Updated `.cursor/memory/AGENT_SCRATCHPAD.md` with the confirmed commissioning lockout pattern.
- Validation:
  - Reviewed the attached controller log showing:
    - repeated successful `GET_JOINT_STATE` handling
    - `Open-Loop Executor finished`
    - thread exception from `trajectory_execution._open_loop_executor_thread()` because `backend.wait_for_trajectory_complete()` raised `TimeoutError` for RTCore trajectory `24`
  - Read `src/gradient_os/arm_controller/backends/ethercat_rtcore/backend.py`:
    - `execute_joint_trajectory()` submits the queued RTCore path and then waits for terminal completion
    - `wait_for_trajectory_complete()` raises if RTCore never reports a terminal state for the submitted command sequence within the timeout
  - Read `web-ui/src/ControlPanel.tsx`:
    - commissioning jog buttons are disabled whenever `motionBusy` is true
    - `motionBusy` is true for motion states `accepted`, `queued`, or `executing`
  - Read `src/gradient_os/api/main.py`:
    - `/control/joint-jog` parses `wait_for_idle`
    - but currently returns the initial `APPLY_JOINT_SETPOINT` ACK payload directly with `waited_for_idle: False`
    - so the route does not actually wait for or return a terminal motion state before the UI updates its local `motionStatus`
- Follow-up notes / risks:
  - The lockout is consistent with a stale `"accepted"` motion status after a queued RTCore jog times out in the controller waiter.
  - Immediate recovery should prefer checking `/control/motion-status` and using STOP / refresh to clear stale motion state before attempting another commissioning jog.

## 2026-04-08 20:15 +0000

- Task summary:
  - Rebuilt and resynced RTCore, then traced the live J2 wrong-direction commissioning motion to missing native-home offset truth in RTCore metrics.
- Changes:
  - No repo product files changed.
  - Rebuilt RTCore with `source ./start.sh && make -C src/gradient_rt_motion`.
  - Synced/started the installed RTCore service with `source ./start.sh && ./systemd/rt-motion/sync-runtime.sh --ensure-active`.
  - Updated `.cursor/memory/AGENT_SCRATCHPAD.md` with the confirmed native-home-offset-loss pattern.
- Validation:
  - Confirmed the service is now running again as `/usr/local/bin/gradient-rt-motion`.
  - Read current `/run/gradient-rt-motion/metrics.json`; all axes, including J2, currently report `native_home_position_offset: 0`.
  - Searched the full RTCore journal and found the earlier J2 home save evidence:
    - `Apr 08 19:18:51 ... EtherCAT native_home axis=1 ... desired_offset=-107506 saved=1`
  - Compared that with the later live metrics and restart state, which no longer carry that offset into RTCore startup/readback.
- Follow-up notes / risks:
  - The rebuilt RTCore service alone does not fix the bad J2 motion because the live RTCore/native-home state currently says J2 has no saved offset.
  - The remaining root issue is now narrowed to offset retention/readback truth across restart/power-up, not the commissioning `joint-jog` route semantics or the 100 RPM bounded planner path.

## 2026-04-08 20:27 +0000

- Task summary:
  - Proved whether the J2 native-home offset loss is on the drive side or the RTCore readback side.
- Changes:
  - No repo product files changed.
  - Updated `.cursor/memory/AGENT_SCRATCHPAD.md` with the confirmed drive-vs-RTCore boundary evidence.
- Validation:
  - Read J2 directly from EtherCAT with:
    - `source ./start.sh && sudo ethercat upload -p 1 -t int32 0x60B0 0`
    - result: `0xfffe5c0e -107506`
  - Spot-checked neighboring axes:
    - `source ./start.sh && sudo ethercat upload -p 0 -t int32 0x60B0 0`
    - `source ./start.sh && sudo ethercat upload -p 2 -t int32 0x60B0 0`
    - result: both `0`
  - Compared those direct drive reads against current `/run/gradient-rt-motion/metrics.json`, which still reports `native_home_position_offset: 0` for J2.
  - Read the RTCore startup code in `src/gradient_rt_motion/main.cpp` and confirmed it currently snapshots `0x60B0` only once before the startup convergence loop, then never refreshes it later unless a new native-home command is issued.
- Follow-up notes / risks:
  - The drive still has the correct saved J2 native-home offset; RTCore startup/readback is the component currently losing or failing to refresh that truth.
  - This explains why the bounded `+1 deg` commissioning command can still move badly after restart/power-up: RTCore hold-target alignment is operating with a false zero native-home offset while the drive itself is not.

## 2026-04-08 17:21 +0000

- Task summary:
  - Reviewed and tightened the RTCore single-point regression fix so streamed `sync_write` points use explicit controller-timestep `qd` semantics instead of the guessed legacy speed-to-RPM helper.
- Changes:
  - Updated `src/gradient_os/arm_controller/backends/ethercat_rtcore/backend.py`:
    - added `prepare_timed_sync_write_commands()` and `set_joint_positions_for_duration()` for explicit-duration RTCore point uploads
    - validated timed point durations against at least one RTCore cycle instead of accepting near-zero durations
    - delayed `_last_joint_setpoint_rad` updates until after successful `commit_trajectory()`
    - updated the single-actuator success path to keep the fallback setpoint cache aligned
    - reused one seed snapshot inside the legacy single-point helper so duration and `qd` are derived from the same source sample
  - Updated `src/gradient_os/arm_controller/trajectory_execution.py`:
    - use the timed backend helper when available so backend `sync_write()` streams preserve the controller timestep
  - Updated `tests/test_gradient05_limits_and_backends.py`:
    - added regression tests for timed sync-write `qd` math
    - added a guard test that rejects sub-cycle timed point durations
    - extended single-actuator coverage to verify cache updates on success
    - added a failure-path test that confirms failed RTCore commit does not advance `_last_joint_setpoint_rad`
- Validation:
  - `source ./start.sh && python -m pytest tests/test_gradient05_limits_and_backends.py -q -k 'sync_write_preserves_requested_speed or timed_sync_write_uses_explicit_duration_for_velocity or timed_sync_write_rejects_subcycle_duration or single_actuator_setpoint_includes_velocity or set_joint_positions_does_not_advance_cache_on_commit_failure or applies_master_offsets_to_setpoints or execute_joint_trajectory_enqueues_velocity_points'`
  - `source ./start.sh && python -m py_compile src/gradient_os/arm_controller/backends/ethercat_rtcore/backend.py src/gradient_os/arm_controller/trajectory_execution.py tests/test_gradient05_limits_and_backends.py`
  - `ReadLints` on touched files returned clean.
- Follow-up notes / risks:
  - The direct RTCore compatibility helpers (`set_joint_positions()` / `set_single_actuator_position()`) still use a legacy speed heuristic for true one-point commands; the streamed executor path no longer does.
  - Live hardware retest should focus on the J2 commissioning path that exercises the timed `sync_write()` behavior before trusting the broader direct single-point compatibility path.

## 2026-04-08 20:40 +0000

- Task summary:
  - Implemented the RTCore native-home startup readback fix, preserved the ACK-only commissioning jog contract, and ran the requested live retest on J2 after rebuilding/deploying RTCore.
- Changes:
  - Updated `src/gradient_rt_motion/main.cpp`:
    - hoisted the `0x60B0` SDO helper so both RTCore startup logic and native-home code share the same read path
    - added a post-`startup_ready` native-home offset refresh in the metrics thread with retries and success/failure logging
    - refreshed published `latest_feedback.native_home_position_offset` only on successful SDO upload so a failed read does not overwrite a valid in-memory value
  - Updated `tests/test_api_endpoints.py`:
    - added a regression test proving `/control/joint-jog` remains ACK-only and keeps `max_motor_rpm=100.0` even when `wait_for_idle=true`
- Validation:
  - `source ./start.sh && make -C src/gradient_rt_motion`
  - `source ./start.sh && python -m pytest tests/test_api_endpoints.py -q`
  - `source ./start.sh && python -m py_compile src/gradient_os/api/main.py src/gradient_os/arm_controller/command_api.py src/gradient_os/arm_controller/backends/ethercat_rtcore/backend.py`
  - `ReadLints` on `src/gradient_rt_motion/main.cpp` and `tests/test_api_endpoints.py` returned clean
  - `source ./start.sh && python -m pytest tests/test_api_endpoints.py tests/test_command_api_direct_setpoint.py tests/test_gradient05_limits_and_backends.py -q`
    - result: `tests/test_api_endpoints.py` and `tests/test_command_api_direct_setpoint.py` passed, but the broader run still has existing dirty-branch failures in `tests/test_gradient05_limits_and_backends.py::test_gradient05_config_defaults_and_mapping_shape` and `tests/test_gradient05_limits_and_backends.py::test_ethercat_backend_prefers_robot_defined_axis_scaling`
  - `source ./start.sh && ./systemd/rt-motion/sync-runtime.sh --ensure-active`
  - `sudo ethercat upload -p 1 -t int32 0x60B0 0`
    - result: `-107506`
  - Read `/run/gradient-rt-motion/metrics.json`
    - result: axis 1 now reports `native_home_position_offset=-107506` after restart, matching the drive
  - Started a headless controller/API stack and ran a live J2 commissioning proof:
    - baseline J2 before power-up: about `-16.391 deg`
    - powered-up J2 stayed about `-16.391 deg`
    - issued `POST /control/joint-jog {"joint": 2, "delta_deg": 1.0}`
    - controller logged a bounded target of `-16.391 -> -15.391` at `max_motor_rpm=100.0`
    - observed reported J2 after the move attempt: about `-21.297 deg`
    - RTCore never returned the trajectory to idle; controller logged `TimeoutError: Timed out waiting for RTCore trajectory 1 to complete`
    - sent `POST /control/stop`, then `POST /control/power-down`, and verified final J2 state powered back down
- Follow-up notes / risks:
  - The startup native-home offset readback bug is fixed and proven live, but it was not the only cause of the wrong-direction commissioning move.
  - There is still a second live bug in the J2 commissioning path: RTCore/controller accept a `+1 deg` bounded target yet reported feedback moves about `-4.9 deg` and the RTCore trajectory remains latched until externally stopped.

## 2026-04-09 01:05 +0000

- Task summary:
  - Continued into the next commissioning bug after the user clarified that the drives had been physically power-cycled, then tightened native-home completion so it only reports success after verified post-save readback.
- Changes:
  - Updated `src/gradient_rt_motion/main.cpp`:
    - after writing J2/Jn `0x60B0` and issuing `0x1010:01 = "save"`, RTCore now waits and re-reads `0x60B0` until the desired offset is observed before marking `native_home_state=SUCCEEDED`
    - native-home success/failure logging now includes the verified readback offset
  - Updated `src/gradient_os/arm_controller/backends/ethercat_rtcore/backend.py`:
    - `native_home_joint()` now waits for targeted axes to reach a verified native-home terminal result in RTCore metrics instead of returning success immediately after enqueuing the command
    - added focused metrics polling helpers for native-home completion
  - Updated `tests/test_gradient05_limits_and_backends.py`:
    - extended native-home tests to assert the backend waits for verified completion
    - added a failure-path test when that verification times out
- Validation:
  - `source ./start.sh && make -C src/gradient_rt_motion`
  - `source ./start.sh && python -m pytest tests/test_api_endpoints.py tests/test_gradient05_limits_and_backends.py -q -k 'control_home_joint_native or native_home'`
  - `source ./start.sh && python -m py_compile src/gradient_os/arm_controller/backends/ethercat_rtcore/backend.py`
  - `ReadLints` on touched files returned clean
  - Restarted the full stack cleanly after removing stale controller/API/web children from an old launcher process
  - Live checks after the user clarified the drives had been power-cycled:
    - before re-home: `sudo ethercat upload -p 1 -t int32 0x60B0 0` returned `0`, and `/run/gradient-rt-motion/metrics.json` also showed J2 `native_home_position_offset=0`
    - issued `POST /control/home-joint-native {"joint": 2}` and timed the request
    - RTCore journal now shows: `EtherCAT native_home axis=1 ... desired_offset=668218 readback_offset=668218 saved=1`
    - after re-home settled: `sudo ethercat upload -p 1 -t int32 0x60B0 0` returned `668218`
    - after re-home settled: `/info/joints-detailed` reported J2 `arm_deg=0.0`
- Follow-up notes / risks:
  - The live state change after the user's drive power cycle reframed this bug as a persistence-across-real-power-loss problem, not just an RTCore restart/readback issue.
  - The new code proves native-home now waits for a verified saved offset while the drives remain powered, but persistence across another real drive power cycle is still the remaining manual proof step.

## 2026-04-09 01:22 +0000

- Task summary:
  - Validated the user's E-stop drive power cycle, proved RTCore metrics could stay stale across bus recovery, and patched RTCore to rerun startup/native-home offset refresh after later startup-reset epochs.
- Changes:
  - Updated `src/gradient_rt_motion/main.cpp`:
    - added metrics-thread tracking for `startup_ready` and `startup_reset_count`
    - when the startup epoch changes, RTCore now re-arms both the post-`startup_ready` drive-config readback and the native-home offset refresh instead of treating them as one-shot per process lifetime
  - Updated `.cursor/memory/AGENT_SCRATCHPAD.md` with the new post-power-cycle stale-metrics guardrail.
- Validation:
  - Before the patch, after the user's E-stop power cycle:
    - `sudo ethercat upload -p 1 -t int32 0x60B0 0` returned `0`
    - `/run/gradient-rt-motion/metrics.json` still showed axis 1 `native_home_position_offset=668218`
    - `/info/joints-detailed` still showed the stale logical J2 position near `0 deg`
  - Rebuilt and deployed RTCore:
    - `source ./start.sh && make -C src/gradient_rt_motion`
    - `source ./start.sh && ./systemd/rt-motion/sync-runtime.sh --ensure-active`
  - Post-deploy checks:
    - `ReadLints` on `src/gradient_rt_motion/main.cpp` returned clean
    - RTCore journal showed a fresh `EtherCAT native-home offset refresh axis=1 ... offset=0`
    - `sudo ethercat upload -p 1 -t int32 0x60B0 0` returned `0`
    - `/run/gradient-rt-motion/metrics.json` now reports axis 1 `native_home_position_offset=0`
    - `/info/joints-detailed` now reports J2 near `-18.35 deg`, matching the loss of native-home offset truth
- Follow-up notes / risks:
  - The stale-metrics masking bug is fixed, but the underlying persistence problem remains: J2 `0x60B0` still drops to `0` across a real drive power cycle.
  - The remaining live proof for the new RTCore refresh behavior requires one more manual drive power cycle while this newly deployed RTCore process stays running.

## 2026-04-09 01:36 +0000

- Task summary:
  - Diagnosed the repeated `Power up RTCore-controlled drives now?` prompt as UI label ambiguity between drive-power controls and jog arming, then clarified the labels so the homing flow is unambiguous.
- Changes:
  - Updated `web-ui/src/ControlPanel.tsx`:
    - runtime-header drive buttons now read `Power Up` / `Power Down` instead of generic `Arm` / `Disarm`
    - realtime jog toggle now reads `Arm Jog` / `Disarm Jog`
    - updated nearby helper copy to refer to `Jog mode` instead of ambiguous `Arm mode`
  - Updated `web-ui/src/ControlPanel.test.tsx`:
    - adjusted the jog-session tests to use the new `Arm Jog` / `Disarm Jog` labels
    - added regression coverage that the main control panel separates `Power Up Drives` from `Arm Jog`
    - added coverage that the runtime header exposes `Power Up` / `Power Down` instead of generic `Arm`
  - Updated `.cursor/memory/AGENT_SCRATCHPAD.md` with the new operator-label guardrail.
- Validation:
  - `ReadLints` on `web-ui/src/ControlPanel.tsx` and `web-ui/src/ControlPanel.test.tsx` returned clean
  - `cd web-ui && npm run test -- src/ControlPanel.test.tsx`
  - `cd web-ui && npm run build`
- Follow-up notes / risks:
  - This fixes the UI ambiguity that made drive-enable prompts look like a generic `Arm` action, but it does not change the underlying native-home persistence behavior being debugged.
  - Reload the browser tab before the next homing attempt so the updated labels are visible.

## 2026-04-08 16:51 +0000

- Task summary:
  - Updated the memory-maintenance skills so archive-first rollover is the canonical procedure for oversized scratchpad/devlog files.
- Changes:
  - Updated `.cursor/skills/learning-scratchpad-loop/SKILL.md`:
    - added explicit maintenance guidance to rename the live scratchpad into a dated snapshot instead of deleting old content
    - added the required steps to prepend an archive summary and recreate a slim live scratchpad with only retained lessons
    - clarified that a user request for a "fresh slate" means preserve history via rollover, not deletion
  - Updated `.cursor/skills/devlog-loop/SKILL.md`:
    - added the matching archive-first rollover procedure for `DEVLOG.md`
    - documented that the new live devlog should start with a rollover entry that points to the preserved snapshot
    - clarified that paired scratchpad/devlog cleanup should usually roll both files together
  - Updated `.cursor/memory/AGENT_SCRATCHPAD.md`:
    - recorded the new standard rule that old memory should be preserved in dated snapshots rather than deleted during cleanup
- Validation:
  - Read back the updated skill files and live memory files after editing.
  - `ReadLints` will be run on the touched files after the edits complete.
- Follow-up notes / risks:
  - This updates the skill workflow only; it does not yet refactor any older archive structure beyond the rollover already completed today.

## 2026-04-08 16:45 +0000

- Task summary:
  - Rolled over the oversized live `AGENT_SCRATCHPAD.md` and `DEVLOG.md` into dated snapshots, then recreated slim live memory files that retain only the durable guardrails and the active native-home context.
- Changes:
  - Renamed the previous live files to:
    - `.cursor/memory/AGENT_SCRATCHPAD_2026-02-21_to_2026-04-08.md`
    - `.cursor/memory/DEVLOG_2026-02-16_to_2026-04-08.md`
  - Prepended archive summaries to both renamed snapshots so they explain the work they cover without needing to scan the full file first.
  - Created a new compact `.cursor/memory/AGENT_SCRATCHPAD.md` that carries forward:
    - user workflow preferences
    - validation/deployment guardrails
    - the current native-home regression rules for A6-EC / RTCore
  - Reset `.cursor/memory/DEVLOG.md` to this new active timeline entry instead of carrying the full historical ledger forward.
  - Retained the most important current workstream context:
    - native-home remains an active commissioning path and should be treated as unresolved until live hardware behavior is rechecked after risky changes
    - `native_home_position_offset` belongs on feedback/read and hold-target alignment paths, not as an extra subtraction on outgoing RTCore logical position targets
    - successful native home should leave only the homed axis disabled and should surface explicit result/state in telemetry/UI
- Validation:
  - Verified the old live files were preserved as dated snapshots and the new live files were recreated at the original paths.
  - No product code, tests, or runtime behavior were changed in this cleanup task.
- Follow-up notes / risks:
  - Keep the new live memory files intentionally short; archive again before they turn back into full historical ledgers.
  - For future native-home work, use the new scratchpad for preflight guardrails and the dated snapshots for deeper historical debugging context when needed.

## 2026-04-09 02:33 +0000

- Task summary:
  - Completed the direct EtherCAT SDO isolation experiment for J2 native-home persistence and narrowed the remaining fault to drive/object semantics rather than our native-home call sequence.
- Changes:
  - Performed a direct drive write on J2 while disarmed:
    - `sudo ethercat download -p 1 -t int32 0x60B0 0 -- "-96134"`
    - `sudo ethercat download -p 1 -t uint32 0x1010 1 0x65766173`
  - Verified the write immediately after save and after a settle window:
    - `sudo ethercat upload -p 1 -t int32 0x60B0 0` returned `-96134`
    - `sudo ethercat upload -p 1 -t uint16 0x603F 0` returned `0`
    - `sudo ethercat upload -p 1 -t uint16 0x6041 0` returned `0x1650`
  - After the user performed a real drive power cycle, re-read the live state before any jog/home/power-up action:
    - `sudo ethercat upload -p 1 -t int32 0x60B0 0` returned `0`
    - `/run/gradient-rt-motion/metrics.json` axis 1 reported `native_home_position_offset=0`
    - `curl -sf http://127.0.0.1:4000/info/joints-detailed` showed J2 near its raw physical angle instead of logical zero
    - `journalctl -u gradient-rt-motion.service -n 60 --no-pager` showed the refreshed post-startup offset readback also landing on `offset=0`
  - Read the A6-EC ESI metadata for the next root-cause branch:
    - confirmed `0x60B0` is `Position offset`
    - confirmed `0x607C` is `Home offset`
    - confirmed vendor object `0x2013:17` (`Update function code values written via communication to EEPROM`) exists and currently reads back `1`
- Validation:
  - The direct-save experiment reproduced the same post-power-loss reset without using the Python native-home path.
  - Read-only follow-up checks showed `0x607C=0`, `0x2013:17=1`, and J2 still in `SwitchOnDisabled` after power restore.
- Follow-up notes / risks:
  - This is strong evidence that `0x60B0` should not be relied on as the durable hardware-zero store on this A6-EC, even when `0x1010:01` reports an immediate successful save.
  - The next live experiment should target `0x607C` or the vendor-native persistent zero parameter rather than continuing to harden the existing `0x60B0` save loop.

## 2026-04-09 02:40 +0000

- Task summary:
  - Clarified the implications of a possible migration from `0x60B0` to `0x607C` and the current scope of the persistence fault.
- Changes:
  - No product code changed.
  - Reconciled the current evidence into two guardrails:
    - if `0x607C` is adopted as the persistent home object, RTCore/controller command and feedback paths must be re-audited to avoid double-applying offsets
    - the current proof should not be labeled strictly `J2-only`, because only J2 has been exercised deeply even though same-drive semantics suggest a model-wide behavior is more likely
- Validation:
  - Based on the already verified direct-save result (`0x60B0` survives while powered, resets to `0` after real power loss) plus the A6-EC ESI metadata showing both `0x60B0 Position offset` and `0x607C Home offset`
- Follow-up notes / risks:
  - A single additional cross-axis persistence probe would tell us whether this is clearly drive-model-wide or a J2-drive-specific hardware anomaly.

## 2026-04-09 03:43 +0000

- Task summary:
  - Completed step 1 of the production-fix plan by running a direct `0x607C` persistence probe on J1 and J2 with a real drive power cycle.
- Changes:
  - No product code changed.
  - Captured a disarmed/fault-free baseline:
    - J1 `0x607C = 0`, J2 `0x607C = 0`
    - both axes `0x6041 = 0x1650`
    - both axes `0x603F = 0`
  - Wrote and saved distinct test values while disarmed:
    - `sudo ethercat download -p 0 -t int32 0x607C 0 12345`
    - `sudo ethercat download -p 1 -t int32 0x607C 0 -- -23456`
    - `sudo ethercat download -p <axis> -t uint32 0x1010 1 0x65766173`
  - Verified immediate post-save readback:
    - J1 `0x607C = 12345`
    - J2 `0x607C = -23456`
  - After a real drive power cycle, re-read before any motion/power-up action:
    - J1 `0x607C = 12345`
    - J2 `0x607C = -23456`
    - both axes still `0x603F = 0`
  - Collected live-state evidence showing the current code still only refreshes `0x60B0`:
    - `/run/gradient-rt-motion/metrics.json` kept `native_home_position_offset=0`
    - RTCore journal refreshed `offset=0` in the existing native-home offset refresh path
- Validation:
  - The direct `0x607C` values survived a real power cycle on two axes.
  - This contrasts with the earlier direct `0x60B0` experiment, where the written value reset to `0` after power loss.
- Follow-up notes / risks:
  - This strongly supports `0x607C` as the durable drive-home object for this A6-EC setup and makes a J2-only hardware anomaly much less likely.
  - No motion should be commissioned from these experimental values until the software path is migrated and the offset-application frame is re-audited.

## 2026-04-09 03:49 +0000

- Task summary:
  - Produced the smallest-safe production migration plan for moving native-home persistence from `0x60B0` to the now-validated `0x607C`.
- Changes:
  - No product code changed.
  - Traced the minimal active code surface that currently depends on `0x60B0` semantics:
    - RTCore SDO read helper and startup refresh in `src/gradient_rt_motion/main.cpp`
    - RTCore native-home write/verify path in `src/gradient_rt_motion/main.cpp`
    - RTCore feedback-aligned enable/hold-target logic that consumes `native_home_position_offset`
    - Python metrics refresh and command-frame guardrails in `src/gradient_os/arm_controller/backends/ethercat_rtcore/backend.py`
- Validation:
  - Based on direct code inspection plus the completed J1/J2 `0x607C` power-cycle proof.
- Follow-up notes / risks:
  - The safest migration is a narrow source-of-truth change around `native_home_position_offset`, not a broad register-name replacement.

## 2026-04-11 20:02 +0000

- Task summary:
  - Updated the A6-EC bring-up/commissioning documentation and the live native-homing plan to reflect the current manual-backed model: startup absolute mode and DS402 runtime modes are separate layers, normal motion stays in `CSP`, and native home should be treated as a commissioning-only HM transaction.
- Changes:
  - Updated `docs/ethercat/bringup.md` with:
    - a mode-layer table distinguishing `C00.07`, `0x6060`, HM objects, and `0x60B0`
    - the current validated conclusions (`C00.07` rotation-mode concern, `0x60B0` non-persistence, `0x607C` persistence, HM mode `35` relevance)
    - a documented GradientOS commissioning workflow for native home
  - Updated `.cursor/skills/gradientos-sop/commissioning-safety.md` to remove the stale "`0x60B0` is durable native home" assumption and replace it with the commissioning-workflow model.
  - Updated `.cursor/plans/a6ec-native-homing_16b87693.plan.md` to:
    - explicitly keep steady-state motion in `CSP`
    - add the "commissioning-only native-home workflow" requirement
    - list the exact product surfaces/files to change in profile/catalog, RTCore, controller/API/UI, and tests
- Validation:
  - Re-read the A6-EC manual sections for HM, CSP, and absolute-system settings in `docs/resources/A6-EC_series_servo_drive_manual.pdf`.
  - Reviewed the current runtime mode usage in `src/gradient_os/arm_controller/backends/ethercat_rtcore/backend.py`.
  - Reviewed the resulting documentation diff with `git diff`.
  - No code tests were run because this pass only changed documentation/plan files.
- Follow-up notes / risks:
  - Product code still uses the old A6-EC startup-mode labels/default and the old `0x60B0`-based native-home implementation; the docs/plan now lead the code and need to be implemented in a later execution pass.

## 2026-04-11 20:28 +0000

- Task summary:
  - Tightened the live homing plan to make the RTCore boundary explicit and added linked source documents at the top of the plan.
- Changes:
  - Updated `.cursor/plans/a6ec-native-homing_16b87693.plan.md` to:
    - add a `Source Documents` section linking the manual PDF, bring-up doc, ESI, and decoded fault reference
    - replace the RTCore section with an explicit "no vendor-specific homing knowledge in RTCore" rule
    - state that RTCore must not hardcode A6-EC object IDs, homing method `35`, save rules, or durable-home assumptions
    - require RTCore to execute only generic transaction primitives driven by profile/catalog/backend-owned descriptors
- Validation:
  - Re-read the updated top-of-file source links and RTCore section in the plan file.
  - Confirmed the revised text now states that the choice of HM objects, values, and ordering must live outside RTCore.
  - A readonly `git diff` check against the plan path failed because the plan file lives outside the repository root; no further shell validation was needed.
- Follow-up notes / risks:
  - The implementation pass must preserve this stricter boundary and avoid drifting back toward "generic RTCore plus a few A6-EC exceptions."

## 2026-04-11 21:00 +0000

- Task summary:
  - Removed the remaining implementation ambiguity from the homing plan so it can serve as a true cold handoff for a fresh build agent.
- Changes:
  - Updated `.cursor/plans/a6ec-native-homing_16b87693.plan.md` to add:
    - `Cold Handoff Defaults` with explicit first-cut values/choices:
      - `C00.07 = 4`
      - steady-state `CSP`
      - commissioning `HM`
      - `6098 = 35`
      - `60E6 = 0`
      - `607C = 0`
      - `607C` as persistent truth source
      - no explicit `0x1010` in the first HM implementation unless bench evidence requires it
      - post-home explicit re-power expectation
      - validation axes `J4` and `J2`
    - `Cold Handoff Rules` telling a fresh agent not to reopen architecture decisions without bench evidence
    - a minimum descriptor contract example showing what data belongs in the A6-EC profile rather than RTCore
    - explicit `native_home_position_offset` semantics for the first HM-based cut
  - Added the post-home operator expectation to the controller/API/UI section of the plan.
- Validation:
  - Re-read the newly added cold-handoff sections in the plan file.
  - Confirmed the plan now specifies defaults, descriptor shape, field semantics, validation axes, and post-home state instead of leaving them implicit.
- Follow-up notes / risks:
  - The remaining uncertainty is live bench proof only; the plan no longer leaves material build choices undecided for the first implementation cut.

## 2026-04-11 21:13 +0000

- Task summary:
  - Started the first production build cut for A6-EC native homing by moving the HM workflow into profile-owned descriptor data, switching the startup absolute-mode default to the manual-backed rotation mode, and replacing the RTCore hardcoded `0x60B0` native-home path with a generic descriptor executor.
- Changes:
  - Updated `src/gradient_os/arm_controller/profiles/drive/a6ec_ds402.py`:
    - corrected the human-readable `C00.07 / 0x2000:08` labels so value `4` is the rotation-mode default
    - added a profile-owned native-home descriptor for the first HM cut (`steady_state_mode=8`, `commissioning_mode=6`, `truth_source=0x607C`, HM method `35`, `60E6=0`, `607C=0`)
    - rendered that descriptor into the new RTCore env var `GRADIENT_RT_NATIVE_HOME_CONFIG`
  - Updated `src/gradient_os/arm_controller/ethercat_drive_catalog.py`:
    - changed the A6-EC startup default from `1` to `4`
  - Updated `src/gradient_os/arm_controller/profiles/registry.py`, `src/gradient_os/arm_controller/backends/registry.py`, and `src/gradient_os/arm_controller/backends/ethercat_rtcore/runtime.py`:
    - threaded the profile-owned native-home config through the backend/profile registry
    - included `GRADIENT_RT_NATIVE_HOME_CONFIG` in the rendered RTCore systemd env
  - Updated `src/gradient_rt_motion/main.cpp`:
    - added a generic native-home descriptor parser for `set_mode`, `write_sdo`, `controlword_sequence`, `wait_statusword`, `refresh_truth`, and `restore_mode`
    - replaced the old A6-EC-specific `0x60B0` write/save implementation with a descriptor-driven executor
    - switched startup/native-home truth refresh from the hardcoded `0x60B0` path to the descriptor-selected truth source
    - made `MSG_CMD_SET_MODE` update a real per-axis desired-mode path instead of writing to an unused single atomic
    - added a generic per-axis service override so RTCore can drive PDO-owned `0x6040` / `0x6060` fields during HM without SDO-vs-PDO races
  - Updated `systemd/rt-motion/gradient-rt-motion.service`:
    - passed the new `GRADIENT_RT_NATIVE_HOME_CONFIG` env var through to the RTCore CLI
  - Updated `src/gradient_os/api/main.py` and `web-ui/src/ControlPanel.tsx`:
    - reframed native home as a commissioning-only transaction in the operator-facing API/UI copy
  - Updated `tests/test_rtcore_runtime.py`, `tests/test_api_endpoints.py`, and `tests/test_encoder_retention.py`:
    - aligned startup-default expectations with `C00.07 = 4`
    - added coverage that the rendered RTCore env now includes the native-home descriptor
    - updated startup-mode label expectations to the manual-backed `Absolute position linear mode` wording for value `1`
- Validation:
  - `make -C src/gradient_rt_motion`
  - `source ./start.sh && python -m pytest tests/test_rtcore_runtime.py tests/test_api_endpoints.py tests/test_encoder_retention.py -q`
  - `source ./start.sh && python -m py_compile src/gradient_os/arm_controller/profiles/drive/a6ec_ds402.py src/gradient_os/arm_controller/profiles/registry.py src/gradient_os/arm_controller/backends/registry.py src/gradient_os/arm_controller/backends/ethercat_rtcore/runtime.py src/gradient_os/api/main.py tests/test_rtcore_runtime.py tests/test_api_endpoints.py tests/test_encoder_retention.py`
  - `cd web-ui && npm run build`
  - `ReadLints` on all touched product/test files returned clean
- Follow-up notes / risks:
  - This pass proves the code and focused tests, but the HM transaction still needs live bench validation on `J4` and `J2` to confirm the A6-EC statusword masks, post-home truth (`0x607C -> 0`), and safe power-up behavior.
  - The RTCore generic executor is currently tailored to the first descriptor shape; if a future drive family needs richer waits or multi-axis commissioning semantics, extend the descriptor/parser instead of reintroducing vendor branches in `main.cpp`.

## 2026-04-11 22:50 +0000

- Task summary:
  - Fixed the A6-EC HM start-edge timing for `J4`, redeployed the runtime, and completed a clean live proof that the commissioning native-home transaction now succeeds on `J4`.
- Changes:
  - Updated `src/gradient_os/arm_controller/profiles/drive/a6ec_ds402.py`:
    - split the HM controlword handshake so the descriptor now does:
      - `controlword_sequence [6,7,15]`
      - `wait_statusword all_set=0x0227 all_clear=0x2048`
      - `controlword_sequence [31]`
    - kept the existing completion wait (`all_set=0x9000`, `all_clear=0x2000`) and `0x607C` truth refresh unchanged
  - Updated `tests/test_rtcore_runtime.py`:
    - aligned the expected `GRADIENT_RT_NATIVE_HOME_CONFIG` env string with the new two-phase HM handshake
  - Synced the updated runtime into the installed service with `systemd/rt-motion/sync-runtime.sh --ensure-active`
  - Performed a full stack restart after discovering that restarting RTCore alone left the controller on a stale IPC session
- Validation:
  - `source ./start.sh && python -m pytest tests/test_rtcore_runtime.py -q`
  - `source ./start.sh && python -m py_compile src/gradient_os/arm_controller/profiles/drive/a6ec_ds402.py tests/test_rtcore_runtime.py`
  - Verified the installed env at `/etc/default/gradient-rt-motion` now contains:
    - `op|controlword_sequence|6,7,15`
    - `op|wait_statusword|0x0227|0x2048`
    - `op|controlword_sequence|31`
  - Verified full stack reconnection after restart:
    - RTCore journal logged `Controller connected` and `IPC handshake complete`
    - controller log showed `EtherCAT RTCore Connected` and `Feedback ready`
  - Clean live `J4` proof:
    - API `POST /control/home-joint-native` returned `200 OK` with `ACK,NATIVE_HOME_JOINT,4`
    - RTCore journal logged `EtherCAT native_home axis=3 ... steps=10` followed by `Native-home success`
    - live EtherCAT sampling on slave position `3` showed `0x6061: 8 -> 6 -> 8` and `0x6041: 0x1650 -> 0x0633 -> 0x0237 -> 0x9650`
    - `0x607C` stayed `0` throughout the successful J4 run
    - final metrics show axis 3 `native_home_state=2`, `native_home_last_abort_code=0`, `statusword=38480 (0x9650)`, and feedback position near zero (`pos_counts=-1`)
- Follow-up notes / risks:
  - A retry issued after only restarting `gradient-rt-motion.service` failed misleadingly until the controller/API stack was restarted; future commissioning validation after RTCore restarts should always confirm a fresh IPC reconnect first.
  - `startup_drive_config.readback_valid/verified` was still `0` in the immediate post-start metrics snapshot even though the later startup logs completed successfully; this looks like timing of when the snapshot was read, not a regression, but should be remembered when sampling right after restart.
  - The next meaningful scope step is to repeat the same clean proof on the next target joint (`J2` or whichever joint the user chooses) before generalizing the result across all axes.

## 2026-04-11 23:58 +0000

- Task summary:
  - Investigated a live commissioning regression where a UI jog intended for `J4` visibly moved `J1` and left `J2` faulted after power-down.
- Changes:
  - No product code changed in this pass.
  - Collected and correlated evidence from:
    - `logs/startups/20260411-234934/controller.log`
    - `logs/startups/20260411-234934/api.log`
    - `journalctl -u gradient-rt-motion.service`
    - `/run/gradient-rt-motion/metrics.json`
    - direct EtherCAT SDO reads on J2 (`0x603F`, `0x203F`, `0x6041`)
- Validation:
  - Confirmed the stack itself restarted cleanly:
    - `gradient-rt-motion.service` and `ethercat.service` active
    - API `/health` returning OK
    - RTCore metrics reporting `startup_ready=1`, `operational_slaves=6`
  - Confirmed the first jog command path was formed for software joint 4, not joint 1:
    - controller log showed `target_deg` changing only the 4th joint from about `-0.002` to `-1.002`
  - Confirmed the resulting motion/fault contradicted that intended target:
    - next controller feedback sample showed `J1` changed from about `-1.876` to `-2.215`
    - RTCore-backed trajectory execution ended in state `faulted`
  - Decoded the unexpected J2 fault:
    - `0x603F = 0xFF00`
    - `0x203F = 0x0871`
    - A6-EC manual reference maps that to `Er87.1` ("one-time excessive position reference increment")
  - Correlated the unintended motion with the live native-home offsets still present on uninvolved axes:
    - metrics showed `J1` offset `12345` counts and `J2` offset `-96135` counts
    - `12345` counts converts to about `0.339 deg`, which matches the observed unintended J1 movement almost exactly
  - Read the relevant code paths and identified a likely frame mismatch:
    - RTCore hold-target alignment uses `pos - native_home_position_offset`
    - backend trajectory upload currently converts joint positions to axis targets without compensating for that offset frame
- Follow-up notes / risks:
  - Current evidence says this is not a simple `J4 -> J1` joint-index remap bug; it is more likely a commanded-target vs hold-target frame mismatch that becomes dangerous whenever uninvolved axes still have nonzero native-home offsets.
  - Because queued trajectories currently upload all-axis points, a jog on one joint can inject hidden target steps on other axes that are merely meant to hold position.
  - If the user truly pressed the `J4` jog button only once, there may also be a second UI/session anomaly: the later jog POSTs in the same browser session appear to target software joint 3. That is worth verifying separately, but it is not required to explain the observed `J1` motion and `J2` `Er87.1` fault.
  - Safest next step is a code fix plus a controlled retest before any more commissioning jog or re-home attempts on live hardware.

## 2026-04-12 01:35 +0000

- Task summary:
  - Implemented the RTCore/backend jog-frame fix so queued trajectories, realtime RTCore jog, and completion checks now use the same native-home-aware target frame.
- Changes:
  - Updated `src/gradient_rt_motion/main.cpp` to:
    - document the per-axis frame contract explicitly
    - translate queued trajectory point targets into RTCore's feedback-aligned hold frame when trajectory points are latched
    - seed realtime jog target accumulation from the same feedback-aligned frame instead of raw `pos_counts`
    - compare final trajectory completion against feedback translated into the same frame as the queued target
  - Updated `src/gradient_os/arm_controller/backends/ethercat_rtcore/backend.py` comments so the Python side now explicitly documents that it uploads controller/logical targets and leaves the final native-home reframe to RTCore.
  - Updated `tests/test_gradient05_limits_and_backends.py` with a focused regression test that locks the queued-trajectory payload contract when a native-home offset is present.
- Validation:
  - `make -C src/gradient_rt_motion`
  - `source "/home/pi/GradientOS/.venv/bin/activate" && PYTHONPATH=src python -m pytest tests/test_gradient05_limits_and_backends.py -q -k "applies_native_home_offsets_to_feedback_but_not_command_targets or enqueue_trajectory_points_keeps_controller_logical_frame_with_native_home_offset"`
  - `source "/home/pi/GradientOS/.venv/bin/activate" && PYTHONPATH=src python -m py_compile src/gradient_os/arm_controller/backends/ethercat_rtcore/backend.py tests/test_gradient05_limits_and_backends.py`
  - `ReadLints` on `src/gradient_rt_motion/main.cpp`, `src/gradient_os/arm_controller/backends/ethercat_rtcore/backend.py`, and `tests/test_gradient05_limits_and_backends.py` returned clean.
  - A broad `python -m pytest tests/test_gradient05_limits_and_backends.py -q` run on this live machine still reports unrelated failures because some older tests pull current RTCore metrics and inherit the present nonzero `native_home_position_offset` state.
- Follow-up notes / risks:
  - This pass is compile-tested and covered by focused backend tests, but it still needs the planned hardware proof with the current nonzero `J1/J2` `0x607C` values left intact.
  - Live retest should follow the existing safety plan: clear the `J2` fault, keep `J1/J2` offsets unchanged, then verify a single `J4 -1 deg` commissioning jog moves only `J4` and reaches a clean RTCore terminal state.

## 2026-04-12 02:00 +0000

- Task summary:
  - Ran the post-implementation live proof with the rebuilt RTCore deployed and found that the remaining unsafe step occurs on `SAFE_POWER_UP` before the `J4` jog itself.
- Changes:
  - No product code changed in this pass.
  - Captured live proof artifacts from:
    - `http://127.0.0.1:4000/info/joints-detailed`
    - `http://127.0.0.1:4000/control/motion-status`
    - `http://127.0.0.1:4000/control/power-up`
    - `http://127.0.0.1:4000/control/joint-jog`
    - `http://127.0.0.1:4000/control/wait-for-idle`
    - `http://127.0.0.1:4000/control/power-down`
    - `/run/gradient-rt-motion/metrics.json`
    - `logs/startups/20260412-015821/controller.log`
    - `logs/startups/20260412-015821/api.log`
    - direct `sudo ethercat upload` reads for `0x607C`, `0x603F`, `0x203F`, and `0x6041`
- Validation:
  - Confirmed the installed RTCore binary matches the rebuilt repo binary:
    - `sha256sum /home/pi/GradientOS/src/gradient_rt_motion/gradient-rt-motion /usr/local/bin/gradient-rt-motion`
  - Confirmed the proof condition still held before motion:
    - `J1 0x607C = 12345`
    - `J2 0x607C = -96135`
    - `J4 0x607C = 0`
    - API motion status was `idle`
  - Live proof result:
    - before power-up, `J1` was about `105276` counts / `-3.2306 deg`
    - after `SAFE_POWER_UP`, `J1` was about `92889` counts / `-2.8903 deg`
    - the `J1` delta was about `-12387` counts, effectively the persisted `J1` offset magnitude
    - `J2` faulted during/after power-up, before the jog proved anything useful:
      - `0x603F = 0xFF00`
      - `0x203F = 0x0871`
      - `0x6041 = 0x1638`
    - the subsequent single `J4 -1 deg` jog returned `accepted` but RTCore ended `faulted`, `WAIT_FOR_IDLE` timed out, and `J4` did not make the intended `-1 deg` move
  - Controller log evidence:
    - `SAFE_POWER_UP` completed
    - `APPLY_JOINT_SETPOINT` targeted only joint 4
    - controller thread raised `RuntimeError: RTCore trajectory execution ended in state 'faulted'`
- Follow-up notes / risks:
  - The original queued-target/jog-frame fix was not sufficient for live safety because RTCore still appears to inject the persisted home offset on enable/hold synchronization.
  - The new highest-confidence hypothesis is that subtracting `native_home_position_offset` in RTCore hold-target alignment is itself wrong for live `0x607C` behavior; the bench now suggests `0x6064`/`0x607A` may already be in the drive's homed frame, so the software subtraction may be double-applying the offset.
  - The next implementation pass should target the enable/hold-target contract first, not the API jog route.

## 2026-04-12 03:10 +0000

- Task summary:
  - Fixed the remaining RTCore power-up frame bug so enable/hold synchronization stays in raw CSP wire counts, then proved on hardware that `SAFE_POWER_UP` and the original `J4 -1 deg` commissioning jog both behave safely with nonzero persisted `0x607C` offsets on other axes.
- Changes:
  - Updated `src/gradient_rt_motion/main.cpp` to:
    - keep drive-facing hold/output targets in raw `0x6064` / `0x607A` counts during pre-enable, passive startup, stop-collapse, and first-`OperationEnabled` latch
    - keep queued trajectory-point conversion explicit by subtracting `native_home_position_offset` only when converting controller/logical targets into raw CSP wire counts
    - restore realtime jog target seeding and trajectory completion checks to the raw CSP wire frame
  - Updated `src/gradient_os/arm_controller/backends/ethercat_rtcore/backend.py` comments so the Python contract now explicitly says RTCore converts queued controller/logical targets once into raw CSP wire counts.
  - Updated `tests/test_gradient05_limits_and_backends.py` so the focused backend tests cover safe-power-up synchronization and keep the queued-target payload contract explicit with native-home offsets present.
- Validation:
  - `make -C src/gradient_rt_motion`
  - `source "/home/pi/GradientOS/.venv/bin/activate" && PYTHONPATH=src python -m pytest tests/test_gradient05_limits_and_backends.py -q -k "safe_power_up_arms_sets_mode_and_enables or applies_native_home_offsets_to_feedback_but_not_command_targets or enqueue_trajectory_points_keeps_controller_logical_frame_with_native_home_offset"`
  - `source "/home/pi/GradientOS/.venv/bin/activate" && PYTHONPATH=src python -m py_compile src/gradient_os/arm_controller/backends/ethercat_rtcore/backend.py tests/test_gradient05_limits_and_backends.py`
  - Live proof on the rebuilt headless stack using the exact controller/API path:
    - Stage 1 `POST /control/power-up` with `J1 0x607C=12345`, `J2 0x607C=-96135`, `J4 0x607C=0`
    - direct `ethercat upload` reads for `0x607C`, `0x6064`, `0x607A`, `0x6041`, `0x603F`, `0x203F` on J1/J2/J4 before and after power-up
    - result: no nonzero fault words, J1 delta about `-42` counts / `+0.001 deg`, J2 delta about `+1284` counts / `+0.035 deg`, J4 delta about `+15` counts / `-0.002 deg`
    - Stage 2 `POST /control/joint-jog {"joint":4,"delta_deg":-1.0}` followed by `POST /control/wait-for-idle`
    - result: only J4 moved (about `-1.01 deg`, `+6619` counts), terminal state was `completed`, and RTCore reported the normal trajectory-completed event rather than a fault
- Follow-up notes / risks:
  - The live proof now contradicts the earlier assumption recorded during the first jog-frame fix: the safe wire-frame contract is raw CSP counts for hold/output/jog-completion, not `pos_counts - native_home_position_offset`.
  - `J2` still showed a noticeable raw-count delta during power-up without meaningful joint-angle motion or any fault words; treat joint-space deltas plus SDO/API fault checks as the real go/no-go signal, not a raw-count threshold alone.

## 2026-04-12 04:32 +0000

- Task summary:
  - Fixed the native-home false-negative/UI contradiction so long-running drive home operations no longer fall back to a generic request failure while RTCore telemetry later reports success.
- Changes:
  - Updated `src/gradient_os/arm_controller/backends/ethercat_rtcore/backend.py` to:
    - extend the native-home verification wait window from `3.0s` to `10.0s`
    - require a fresh post-command RTCore metrics sample before trusting native-home terminal state
    - return a structured native-home result with `accepted`, `verified`, `timed_out`, `code`, `message`, and live native-home status fields instead of a bare boolean
  - Updated `src/gradient_os/run_controller.py` so `NATIVE_HOME_JOINT` replies preserve the structured backend payload across the UDP controller boundary.
  - Updated `src/gradient_os/api/main.py` so `/control/home-joint-native` returns the structured native-home result for both ACK and controller-level ERROR replies instead of collapsing domain outcomes into a generic `503`.
  - Updated `web-ui/src/ControlPanel.tsx` so drive-native home shows:
    - success for verified results
    - warning for accepted-but-pending verification or missing post-home feedback refresh
    - error only for true native-home failure / request failure
  - Updated focused regressions in:
    - `tests/test_gradient05_limits_and_backends.py`
    - `tests/test_api_endpoints.py`
    - `web-ui/src/ControlPanel.test.tsx`
- Validation:
  - `source "/home/pi/GradientOS/.venv/bin/activate" && PYTHONPATH=src python -m py_compile src/gradient_os/arm_controller/backends/ethercat_rtcore/backend.py src/gradient_os/run_controller.py src/gradient_os/api/main.py tests/test_gradient05_limits_and_backends.py tests/test_api_endpoints.py`
  - `source "/home/pi/GradientOS/.venv/bin/activate" && PYTHONPATH=src python -m pytest tests/test_gradient05_limits_and_backends.py -q -k "native_home or safe_power_up_arms_sets_mode_and_enables or applies_native_home_offsets_to_feedback_but_not_command_targets or enqueue_trajectory_points_keeps_controller_logical_frame_with_native_home_offset"`
  - `source "/home/pi/GradientOS/.venv/bin/activate" && PYTHONPATH=src python -m pytest tests/test_api_endpoints.py -q -k "home_joint_native"`
  - `npm test -- src/ControlPanel.test.tsx`
  - `ReadLints` on the edited backend/controller/API/UI/test files returned no diagnostics
- Follow-up notes / risks:
  - The `10.0s` native-home wait is a safer default for current observed drive behavior, but if a particular axis routinely exceeds it the UI will now degrade to a warning/pending-verification state rather than a hard failure.
  - This pass improves controller/API/UI truthfulness for native-home, but live bench revalidation on the actual J1 flow is still the right next proof step.

## 2026-04-12 04:39 +0000

- Task summary:
  - Verified the post-restart J1 native-home flow on live logs/metrics and promoted the now-stable native-home frame/status rules into the canonical GradientOS SOP skill files.
- Changes:
  - Verified the latest restart session in `logs/startups/20260412-043326/`:
    - `controller.log` shows `Received: 'NATIVE_HOME_JOINT,1'` followed by `[EtherCAT RTCore] Native drive-home verified: joint=1 axis_mask=0x1`
    - `api.log` shows `POST /control/home-joint-native HTTP/1.1" 200 OK`
    - `/run/gradient-rt-motion/metrics.json` shows axis 0 with `native_home_state=2`, `native_home_last_abort_code=0`, and `axis_enable_mask=62`, which matches the intended "J1 left disabled after home" contract
  - Updated canonical SOP skill references:
    - `.cursor/skills/gradientos-sop/rtcore-ethercat.md`
    - `.cursor/skills/gradientos-sop/commissioning-safety.md`
    - `.cursor/skills/gradientos-sop/ui-api-telemetry.md`
  - Replaced the stale RTCore native-home frame note with the validated contract:
    - drive-facing CSP hold/output/enable targets stay in raw `0x6064` / `0x607A` wire counts
    - subtract `native_home_position_offset` only when converting queued controller/logical targets into raw CSP wire counts
  - Added stable commissioning/UI guidance:
    - native-home keeps the homed axis disabled until explicit safe power-up
    - native-home verification must use a fresh post-command RTCore metrics sample
    - accepted-but-still-verifying native-home should surface as pending/warning, not generic request failure
- Validation:
  - log/metrics inspection only; no new code tests were needed for the skill-file consolidation pass
- Follow-up notes / risks:
  - The canonical skill now matches the validated native-home frame and status contract observed on live hardware after restart.

## 2026-04-12 04:44 +0000

- Task summary:
  - Updated the long-form canonical SOP source file so the master GradientOS operating-principles document no longer carries stale native-home frame/status guidance.
- Changes:
  - Updated `.cursor/skills/gradientos-sop/RTOS_ETHERCAT_MASTER_OPERATING_PRINCIPLES.md` to:
    - clarify that safe-enable/power-up synchronization must happen in the actual drive-facing CSP wire frame
    - state the validated A6-EC rule that `0x607C` is durable native-home truth while hold/output/enable targets remain raw `0x6064` / `0x607A` counts
    - document that `native_home_position_offset` is applied only when converting queued controller/logical targets into raw CSP wire counts
    - describe the finalized native-home execution/status contract (`accepted`, `verified`, `timed_out`) and the requirement to wait for fresh post-command RTCore metrics
    - note that non-converged native-home verification should degrade to pending/warning messaging instead of generic request failure
- Validation:
  - searched the `gradientos-sop` corpus for stale native-home wording after the edit
  - `ReadLints` on the edited master markdown file returned no diagnostics
- Follow-up notes / risks:
  - The routed SOP files and the long-form master file are now aligned for the native-home workstream.

## 2026-04-12 04:47 +0000

- Task summary:
  - Ran the formal encoder-retention power-cycle comparison and confirmed the current pose does not survive a real power cycle on this setup yet.
- Changes:
  - Captured retention experiment `20260412-044300` using the built-in API workflow:
    - `before_power_down` snapshot at `2026-04-12T04:43:00+00:00`
    - `after_power_up` snapshot at `2026-04-12T04:45:43+00:00`
  - Artifacts written to:
    - `logs/encoder-retention/20260412-044300/before_power_down.json`
    - `logs/encoder-retention/20260412-044300/after_power_up.json`
    - `logs/encoder-retention/20260412-044300/comparison.json`
    - `logs/encoder-retention/20260412-044300/comparison.md`
- Validation:
  - Comparison outcome:
    - `raw_encoder_mismatch = true`
    - `logical_angle_mismatch = true`
    - `startup_drive_config_mismatch = true`
    - `active_battery_or_multiturn_faults = []`
  - Representative before/after raw-count deltas:
    - axis 0: `6 -> 38326`
    - axis 1: `73 -> 101086`
    - axis 2: `-47 -> 25346`
    - axis 3: `-13 -> 4769`
    - axis 4: `35 -> 21398`
    - axis 5: `6 -> 18696`
  - Representative logical-angle drift:
    - `J1: -2.876e-06 -> -0.01837 rad`
    - `J6: -2.876e-05 -> -0.08962 rad`
- Follow-up notes / risks:
  - The decisive signal is the raw/logical mismatch on every axis after a real power cycle.
  - The after-power snapshot did not show battery/multi-turn faults, but startup drive-config verification remained unresolved on all axes (`readback_valid=false`, `verified=false`), so the next debugging pass should focus on proving the startup absolute-position/encoder-tracking mode rather than assuming retained position is trustworthy.

## 2026-04-12 04:55 +0000

- Task summary:
  - Tested the immediate post-retention hypotheses directly and ruled out both stale software-side home writes and an incorrect startup absolute-mode value as the primary cause of the cold-boot pose drift.
- Changes:
  - Verified the startup config path in code:
    - `a6ec_encoder_position_tracking_mode` defaults to `4` in `src/gradient_os/arm_controller/ethercat_drive_catalog.py`
    - RTCore writes that startup SDO during bring-up and performs deferred readback in `src/gradient_rt_motion/main.cpp`
  - Collected privileged live EtherCAT reads after cold boot:
    - all axes `0x2000:08 = 4`
    - all axes `0x607C = 0`
    - all axes `0x6041 = 0x1650`
    - all axes `0x6064` matched the large after-power counts from the retention snapshot
  - Verified RTCore journal/readback evidence:
    - journal logged `EtherCAT startup readback ... commanded=4 readback=4 verified=1` on all six axes
    - current `/run/gradient-rt-motion/metrics.json` now shows `startup_drive_config.readback_valid=1` and `verified=1` on every axis
- Validation:
  - `journalctl -u gradient-rt-motion.service -n 200 --no-pager`
  - privileged `ethercat upload` reads for `0x2000:08`, `0x607C`, `0x6064`, and `0x6041`
  - live metrics inspection via `/run/gradient-rt-motion/metrics.json`
- Follow-up notes / risks:
  - This evidence strongly weakens the "leftover direct writes" theory and the "startup mode wrong" theory.
  - The remaining highest-confidence hypothesis is drive-side absolute/reference validity not surviving cold boot even though the configured startup mode value is correct and readback-verifiable.
  - A notable clue is the all-axis statusword change from `0x9650` before power loss to `0x1650` after cold boot, which may indicate a manufacturer-specific absolute/reference-valid bit clearing without a corresponding battery/multi-turn fault.

## 2026-04-12 05:03 +0000

- Task summary:
  - Reviewed the A6-EC manual text directly to interpret the `0x9650 -> 0x1650` statusword change against the drive's own HM/home-offset semantics.
- Changes:
  - Confirmed from `docs/resources/A6-EC_series_servo_drive_manual.pdf`:
    - in homing mode, `6041h` bit 15 = `Homing completed`
    - in homing mode, `6041h` bit 12 = `Homing completion output`
    - `607Ch` home offset is active only when the drive is powered on, homing is complete, and `6041h` bit 15 is `1`
  - Compared that manual definition to the observed values:
    - `0x9650` = bits `15,12,10,9,6,4`
    - `0x1650` = bits `12,10,9,6,4`
    - the cold-boot difference is therefore exactly loss of bit 15
- Validation:
  - direct PDF text inspection with `ReadFile` on `docs/resources/A6-EC_series_servo_drive_manual.pdf`
- Follow-up notes / risks:
  - This strengthens the hypothesis that the drive loses full homing-complete/reference-active state on cold boot even though the startup mode and zero home offset values remain correct.

## 2026-04-12 18:20 +0000

- Task summary:
  - Tested the "hidden persisted variable from our old direct writes" hypothesis against the latest cold boot and found no evidence that the likely EEPROM-backed C10 absolute-position offset objects are carrying our old values.
- Changes:
  - Reviewed the latest startup bundle `logs/startups/20260412-181258/`.
  - Observed the shifted pose immediately at startup in `controller.log` on the first `GET_POSITION`:
    - `J1..J6 = -1.05268250, 2.77649231, -0.69628601, -0.72601318, -1.88077148, -5.13803101 deg`
  - Collected privileged EtherCAT reads for hidden C10 candidates:
    - `0x2010:11` (multi-turn absolute position offset low 32 bits) = `0` on axes 0-5
    - `0x2010:13` (multi-turn absolute position offset high 32 bits) = `0` on axes 0-5
    - `0x2010:1F` (single-turn homing absolute value offset) = `0` on axes 0-5
    - `0x2010:15` = `1` on axes 0 and 3, `0` elsewhere
    - `0x2010:16` and `0x2010:17` = `0` on all axes
  - Compared those against current `0x6064` values and found none of the nonzero C10 entries numerically resemble the shifted raw positions.
- Validation:
  - latest startup log inspection
  - privileged `ethercat upload` reads for the `0x2010:*` C10 subindices plus `0x6064`
- Follow-up notes / risks:
  - This weakens the "we wrote some hidden EEPROM position variable and it is still haunting us" theory substantially.
  - The shifted pose is present immediately on cold boot and does not currently map to the obvious EEPROM-backed offset objects, so the remaining issue still looks drive-side/reference-state-related rather than a leftover software write.

## 2026-04-12 18:31 +0000

- Task summary:
  - Pulled the exact manual references for the `0x9650 -> 0x1650` statusword difference and probed a broader set of live encoder/reference objects to see whether the drive itself agrees with the shifted cold-boot pose.
- Changes:
  - Manual findings from `docs/resources/A6-EC_series_servo_drive_manual.pdf`:
    - HM statusword table defines `6041h` bit 15 as `Homing completed` and bit 12 as `Homing completion output`
    - `607Ch` says home offset is active only when powered on, homing is complete, and `6041h` bit 15 is `1`
    - CSP table still labels bits 14-15 as manufacturer-specific / not supported, so the HM/object-specific wording is the more useful interpretation for the observed cold-boot issue
  - Derived statusword difference:
    - `0x9650` = bits `15,12,10,9,6,4`
    - `0x1650` = bits `12,10,9,6,4`
    - only bit 15 drops across cold boot
  - Broader live object probe:
    - many `0x2040:*` position-like channels (`:21`, `:23`, `:25`, `:27`, `:33`, `:37`, `:41`, `:43`) numerically track the shifted `0x6064` counts
    - readable `0x2010:*` bias/limit fields remain zero or static defaults
    - several ESI-listed absolute-feedback subindices under `0x2010` are absent or unsupported as readable SDOs on this firmware
- Validation:
  - direct PDF text inspection
  - privileged EtherCAT reads of `0x2040:*`, `0x2010:*`, `0x6064`, and `0x6041`
- Follow-up notes / risks:
  - The shifted pose is not just a DS402/UI translation artifact; the drive's own live `0x2040` monitor objects largely agree with `0x6064`.
  - The strongest remaining hypothesis is that the drive reboots into a different internal reference/homing-validity state, not that a hidden persisted offset written by us is being reapplied.

## 2026-04-12 18:42 +0000

- Task summary:
  - Probed whether a normal fault reset or the vendor software-reset object can restore the lost HM/reference-valid state after the cold-boot mismatch.
- Changes:
  - Verified pre-probe RTCore health from `/run/gradient-rt-motion/metrics.json`:
    - `armed=0`
    - `axis_enable_mask=0`
    - `startup_ready=1`
    - `wkc_actual=18`
  - Baseline privileged EtherCAT reads showed all axes at:
    - `0x6041 = 0x1650`
    - `0x607C = 0`
    - shifted `0x6064` counts still present
  - Issued `POST /control/reset-faults` and re-read the same objects:
    - statuswords remained `0x1650`
    - home offsets remained `0`
    - raw position counts did not return to the pre-power-cycle pose
  - Wrote vendor software reset `0x2031:02 = 1` while disarmed:
    - RTCore briefly dropped to `startup_ready=0`, `wkc_actual=11`
    - after settling it recovered to `startup_ready=1`, `wkc_actual=18`
    - axes still returned with the shifted `0x6064` counts and no recovery of HM bit 15
  - Observed an induced transient fault on axis 1 after the software-reset probe:
    - `0x6041 = 0x1618`
    - `0x603F = 0x8700`
    - `0x203F = 0x0C20`
  - Cleared that induced fault with a normal `POST /control/reset-faults`, returning all axes to `0x6041 = 0x1650` and zero fault registers.
- Validation:
  - live privileged `ethercat upload` reads for `0x6041`, `0x603F`, `0x203F`, `0x6064`, `0x607C`, and `0x2040:*`
  - live `POST /control/reset-faults`
  - live RTCore metrics checks before, during, and after the probe
- Follow-up notes / risks:
  - Neither a standard fault reset nor the vendor software-reset object restores HM bit 15 or the pre-boot pose, so the mismatch does not look like a stale software-side latch that a soft reset can clear.
  - The next investigation should target the drive's absolute-reference boot conditions directly: what preconditions make the A6-EC assert HM bit 15 again after power-up, and whether only a full native-home/HM cycle re-establishes the reference-active state.

## 2026-04-12 19:08 +0000

- Task summary:
  - Re-reviewed the A6-EC manuals and implemented a startup-config fix so RTCore can program the drive's absolute-rotation gear-ratio parameters instead of leaving them at the vendor `1:1` defaults.
- Changes:
  - Manual findings from `docs/resources/A6-EC_series_servo_drive_manual.pdf` and `docs/resources/chapter 11 - parameter list - A6-EC_series_servo_drive_manual (2).pdf`:
    - Chapter 5 says the battery-backed absolute encoder should remove the need for re-homing after power-up when the absolute system is configured correctly.
    - Chapter 5 also says changing the electronic gear ratio changes the mechanical position abruptly and requires homing.
    - In absolute rotation mode, the drive reconstructs the mechanical absolute position using `C10.1A/C10.1C` first, otherwise `C10.18/C10.19`.
  - Live privileged reads showed every axis currently boots with:
    - `C00.07 = 4`
    - `C10.18 = 1`
    - `C10.19 = 1`
    - `C10.1A = 0`
    - `C10.1C = 0`
  - Confirmed this conflicts with the actual Gradient-05 robot reductions from `src/gradient_os/arm_controller/robots/gradient05/config.py`:
    - `J1-J3 = 100:1`
    - `J4 = 18:1`
    - `J5 = 31.25:1` (rendered as `125/4`)
    - `J6 = 10:1`
  - Extended the startup-config builder and RTCore startup parser so `GRADIENT_RT_DRIVE_STARTUP_SDO_CONFIG` now carries three startup descriptors:
    - `C00.07 / 0x2000:08`
    - `C10.18 / 0x2010:19`
    - `C10.19 / 0x2010:1A`
  - Kept the existing `startup_drive_config` metrics/API contract anchored on the primary `C00.07` entry to avoid breaking the current UI/telemetry consumers while still applying the extra ratio SDOs at startup.
- Validation:
  - `source "/home/pi/GradientOS/.venv/bin/activate" && PYTHONPATH=src python -m pytest tests/test_rtcore_runtime.py -q`
  - `make -C src/gradient_rt_motion`
  - rendered startup env check:
    - `a6ec_encoder_position_tracking_mode|u16|0x2000|0x08|4,4,4,4,4,4;a6ec_rotation_mode_gear_ratio_numerator|u16|0x2010|0x19|100,100,100,18,125,10;a6ec_rotation_mode_gear_ratio_denominator|u16|0x2010|0x1A|1,1,1,1,4,1`
- Follow-up notes / risks:
  - This code change is the most plausible manual-backed fix for the cold-boot mismatch, but it still needs live deployment and proof on hardware.
  - Because the manual says changing the electronic gear ratio changes mechanical position abruptly, the first deployment will require one explicit re-home after startup to seed the corrected EEPROM reference; the goal is to remove the need for re-home on subsequent power cycles, not to avoid that one migration step.

## 2026-04-12 19:22 +0000

- Task summary:
  - Backed out the drive-side rotation gear-ratio startup change after re-evaluating the ownership boundary and the user challenge that it did not address the real encoder-retention failure mode.
- Changes:
  - Reverted the A6-EC startup-config extension that had added `C10.18/C10.19` to the RTCore startup SDO env.
  - Restored the prior startup contract where RTCore only emits and parses the primary `C00.07 / 0x2000:08` startup SDO.
  - Kept the investigation result that the drive currently reports rotation-mode ratio defaults, but stopped treating that as the proposed fix.
- Validation:
  - `source "/home/pi/GradientOS/.venv/bin/activate" && PYTHONPATH=src python -m pytest tests/test_rtcore_runtime.py -q`
  - `make -C src/gradient_rt_motion`
- Follow-up notes / risks:
  - The better current framing is that cold boot is failing to restore the drive's saved absolute-reference correction, not that GradientOS forgot to mirror robot gear ratios into the drive.
  - The next useful investigation should target the drive's save/restore path itself: what object actually changes after native home to represent the saved mechanical-vs-encoder deviation, and whether `F31.10` read/write encoder operations are required to commit or reload that state.

## 2026-04-12 20:45 +0000

- Task summary:
  - Converted `docs/resources/chapter 11 - parameter list - A6-EC_series_servo_drive_manual (2).pdf` into a repo-local Markdown reference at `docs/resources/a6ec_manual_chapter_11_parameter_list.md`.
- Changes:
  - Extracted the PDF with `pdftotext -layout` to preserve the original table geometry.
  - Generated a structured Markdown document with:
    - a short intro and address-range table for `11.1`
    - sectioned group headings for `11.2`
    - a separate `11.3` section for parameter descriptions
    - fenced `text` blocks for wide tables and manual formatting that would be lossy in pipe-table form
  - Removed the temporary extracted text file after generating the final Markdown.
- Validation:
  - Spot-checked the generated Markdown with `ReadFile`
  - Ran `ReadLints` on `docs/resources/a6ec_manual_chapter_11_parameter_list.md` with no diagnostics reported
- Follow-up notes / risks:
  - The document is faithful to the manual text and page structure, but some original line wrapping inside very wide tables remains intentionally preserved rather than aggressively reflowed.

## 2026-04-12 21:05 +0000

- Task summary:
  - Ran the first half of the single-axis `F31.10` save/restore experiment on `J1`/axis `0` to separate raw encoder state from the drive-applied absolute reference frame.
- Changes:
  - Verified the stack was healthy and disarmed before the probe:
    - `armed=0`
    - `axis_enable_mask=0`
    - `startup_ready=1`
    - `wkc_actual=18`
  - Baseline cold-boot snapshot on axis `0`:
    - `0x6041 = 0x1650`
    - `0x6063 ≈ 38329`
    - `0x6064 ≈ 38327`
    - `U40.14/.16/.18/.1A ≈ 38328`
    - `U40.1C ≈ 38328`
    - `U40.20 ≈ 38329`
    - `0x607C = 0`
  - Ran `POST /control/home-joint-native` for `J1`; the API returned `accepted=true`, `verified=true`, `code=NATIVE_HOME_VERIFIED`.
  - Post-home snapshot on the same axis:
    - `0x6041 = 0x9650`
    - `0x6063 ≈ 5`
    - `0x6064 ≈ 5`
    - `U40.14/.16 ≈ 4`
    - `U40.1C ≈ 38331`
    - `U40.20 ≈ 38331`
    - `0x607C = 0`
  - Issued direct commissioning-only encoder operations while still disarmed:
    - `F31.10 = 1` (`Read encoder`)
    - `F31.10 = 2` (`Write encoder`)
  - Settling poll after `F31.10` showed:
    - `F31.10` self-cleared back to `0`
    - no drive fault (`0x603F = 0`, `0x203F = 0`)
    - `0x6041` returned to `0x9650`
    - reference-unit channels stayed near zero while raw encoder-oriented channels stayed near `38330`
- Validation:
  - live privileged `ethercat upload` / `download` reads and writes for `0x2031:11`, `0x6041`, `0x6063`, `0x6064`, `0x607C`, and `0x2040:*`
  - live `POST /control/home-joint-native`
- Follow-up notes / risks:
  - This strongly suggests native home and `F31.10` affect a drive-side reference transform rather than the raw battery-backed encoder counts themselves.
  - The decisive remaining step is the real power cycle: if the post-`F31.10` cold boot comes back near zero/reference-aligned on the same axis, then the missing fix is likely an explicit encoder read/write commit or reload step rather than a repeated home requirement.

## 2026-04-12 21:12 +0000

- Task summary:
  - Completed the second half of the single-axis `F31.10` experiment by reading the same object set after a real drive-only power cycle.
- Changes:
  - Verified RTCore saw the drive-only restart and recovered cleanly:
    - `startup_reset_count = 1`
    - `startup_ready = 1`
    - `wkc_actual = 18`
    - stack remained disarmed
  - Post-power-cycle axis 0 snapshot (the only axis that received native home + `F31.10` read/write):
    - `0x6041 = 0x1650`
    - `0x6063 ≈ 2..4`
    - `0x6064 ≈ 1..3`
    - `U40.14/U40.16 ≈ 1..3`
    - `U40.1C/U40.20 ≈ 38330`
    - `0x607C = 0`
  - Control comparison across axes:
    - axis 0 kept the corrected near-zero reference frame
    - untouched axes 1-5 still came back in the old shifted frame, for example:
      - axis 1 `0x6064 ≈ 101087`
      - axis 2 `0x6064 ≈ 25347`
      - axis 3 `0x6064 ≈ 4758`
      - axis 4 `0x6064 ≈ 21396`
      - axis 5 `0x6064 ≈ 18704`
- Validation:
  - live privileged EtherCAT reads of `0x6041`, `0x6063`, `0x6064`, `0x607C`, and `U40.14/.16/.1C/.20` on axis 0 and all six axes after the drive-only power cycle
- Follow-up notes / risks:
  - This is the strongest evidence so far that `F31.10` read/write changes whether the drive restores the native-home reference transform across a later drive-only power cycle.
  - It also weakens the earlier inference that the `0x9650 -> 0x1650` bit-15 drop is itself the cause of the wrong pose: axis 0 lost bit 15 again but still preserved the corrected reference frame.
  - The next implementation step should be a commissioning-safe way to apply the `F31.10` read/write sequence as part of the native-home persistence workflow, followed by a focused hardware proof on one axis before broadening to all joints.

## 2026-04-12 22:59 +0000

- Task summary:
  - Rolled the `F31.10` persistence tail into the integrated A6-EC native-home workflow, validated it on fresh axes, and confirmed the rollout survives a later drive-only power cycle.
- Changes:
  - Extended the RTCore native-home transaction language with:
    - `wait_sdo`
    - `release_service_override`
  - Updated the A6-EC native-home profile transaction so it now performs, after the HM/home capture succeeds:
    - `refresh_truth`
    - `restore_mode`
    - `release_service_override`
    - `F31.10 = 1` (`Read encoder`) + wait for `0`
    - `F31.10 = 2` (`Write encoder`) + wait for `0`
  - Increased the backend-side native-home wait ceiling from `10.0s` to `20.0s`.
  - Tightened backend verification so fresh post-command snapshots can treat `statusword bit 15` with zero abort code as a valid success signal, rather than depending only on `native_home_state`.
  - Increased the API timeout for `/control/home-joint-native` from `5.0s` to `25.0s` so the controller reply can survive the longer persistence tail.
  - Validation results:
    - axis 1 (`J2`) became semantically home-aligned under the integrated flow, but it was already contaminated by earlier experiments, so it was not used as the clean persistence proof axis
    - untouched axis 2 (`J3`) integrated proof:
      - before home: `0x6064 ~= 25346`, `U40.16 ~= 25344`, `0x6041 = 0x1650`
      - after integrated endpoint: `0x6064 ~= 131062`, `U40.16 ~= -13`, `0x6041 = 0x9650`
      - after later drive-only power cycle: `0x6064 ~= 131060`, `U40.16 ~= 131059`, raw `U40.1C ~= 25334`, `0x6041 = 0x1650`
    - axis 0 and axis 1 also remained in corrected post-home frames across the latest drive-only power cycle
- Validation:
  - `source "/home/pi/GradientOS/.venv/bin/activate" && PYTHONPATH=src python -m pytest tests/test_rtcore_runtime.py -q`
  - `source "/home/pi/GradientOS/.venv/bin/activate" && PYTHONPATH=src python -m pytest tests/test_api_endpoints.py -q -k "home_joint_native"`
  - `source "/home/pi/GradientOS/.venv/bin/activate" && PYTHONPATH=src python -m pytest tests/test_gradient05_limits_and_backends.py -q -k "native_home"`
  - `make -C src/gradient_rt_motion`
  - live `systemd/rt-motion/install.sh`, RTCore restarts, launcher-managed controller/API reloads, integrated API calls, and privileged EtherCAT reads before/after the final drive-only power cycle
- Follow-up notes / risks:
  - The persistence rollout is now bench-proven on a fresh axis, but `native_home_state` / `native_home_last_abort_code` can remain stale in RTCore metrics after restart or after a previously failed attempt even when the live drive objects show the corrected frame.
  - The timeout is now less important to correctness because success/failure can terminate from fresh terminal signals; it should be treated as a deadman ceiling, not the primary completion criterion.

## 2026-04-12 23:59 +0000

- Task summary:
  - Cleaned up stale native-home telemetry so the frontend no longer reports false `failed` badges for already-persisted axes such as `J2` and `J3`.
- Changes:
  - RTCore metrics cleanup:
    - reset `native_home_state` to `idle`
    - reset `native_home_last_abort_code` to `0`
    - perform that reset when a new startup epoch is detected (`startup_reset_count` change or `startup_ready` dropping during a drive restart)
  - Drive-fault snapshot cleanup:
    - derive an effective UI-facing native-home state from live wire-state
    - if statusword bit 15 is present and there is no current fault, prefer that fresh signal over a stale failed result
    - preserve the raw reported result in parallel `*_reported` fields for debugging
  - After rolling the cleanup into the live stack and restarting RTCore plus controller/API:
    - `/run/gradient-rt-motion/metrics.json` shows all axes back at `native_home_state = 0`, `native_home_last_abort_code = 0`
    - a live `build_drive_fault_snapshot(...)` check for `J2` and `J3` reports:
      - `native_home_state_name = idle`
      - `native_home_last_abort_code = 0`
      - `statusword = 0x1650`
    - live drive objects for `J2`/`J3` still show the corrected persisted frame, so the false UI failure signal is gone without losing the real retention result
- Validation:
  - `source "/home/pi/GradientOS/.venv/bin/activate" && PYTHONPATH=src python -m pytest tests/test_drive_faults.py -q`
  - `source "/home/pi/GradientOS/.venv/bin/activate" && PYTHONPATH=src python -m pytest tests/test_rtcore_runtime.py -q -k "drive_fault_snapshot"`
  - `make -C src/gradient_rt_motion`
  - live RTCore install/restart and launcher-managed stack restart
  - live metrics readback plus live `build_drive_fault_snapshot(...)` reconstruction for `J2` and `J3`
- Follow-up notes / risks:
  - The frontend should now stop showing stale native-home failures after drive restarts, but the commissioning card still only shows “success” while a fresh success signal is present; after a later reboot it intentionally falls back to idle rather than trying to infer “persisted success forever” from last-operation state.

## 2026-04-13 00:19 +0000

- Task summary:
  - Fixed the remaining wrapped-angle conversion bug so persisted native-home axes no longer show false `±3.6°` joint offsets or feed misleading angles into the frontend visualizer.
- Changes:
  - Extended the Python RTCore backend axis config to retain `counts_per_rev` alongside `counts_per_unit` and sign.
  - Added A6-EC-specific feedback normalization in `EthercatRTCoreBackend`:
    - for `a6ec_ds402`, convert controller-facing feedback counts into a signed single-turn range using encoder counts-per-rev before converting to joint radians
    - this turns values like `131060` into `-12` counts instead of `+131060`
  - Updated stale config expectations in focused backend tests:
    - current Gradient-05 signs
    - current `J5` ratio `31.25`
    - current `J6` limit `(-10.0, 10.0)`
  - Reloaded the launcher-managed controller/API stack so the live endpoints and SSE monitor picked up the new conversion.
- Validation:
  - `source "/home/pi/GradientOS/.venv/bin/activate" && PYTHONPATH=src python -m pytest tests/test_gradient05_limits_and_backends.py -q -k "gradient05_config_defaults_and_mapping_shape or feedback_using_axis_scaling or wraps_a6ec_single_turn_feedback_counts or prefers_robot_defined_axis_scaling"`
  - live `/info/joints-detailed` after reload:
    - `J2 ≈ -0.00033°`
    - `J3 ≈ +0.00033°`
    - `axis_counts` still near `131060`, confirming the change is in interpretation rather than raw drive state
  - live `/info/pose` after reload matched the same near-zero `J2/J3` values
  - live `/monitor` SSE samples also matched the same near-zero `J2/J3` values over repeated samples
- Follow-up notes / risks:
  - I could not directly view the rendered browser scene from this environment, so I verified the viewer input feeds instead.
  - The backend/HTTP/SSE sources are now stable and near zero for `J2/J3`; if a large visible flicker is still present in the 3D robot after a page refresh, the remaining issue is likely a frontend-only render artifact rather than a controller/RTCore data bug.

## 2026-04-13 00:31 +0000

- Task summary:
  - Tightened native-home completion so rapid successive Drive Home requests cannot cut over on an early partial-success signal while RTCore is still running the persistence tail.
- Changes:
  - Added `native_home_active_axis_mask` to RTCore metrics JSON for the exact duration of each in-flight native-home transaction.
  - Updated the Python native-home wait logic so:
    - statusword-bit-15 fallback is not allowed until the active mask has been seen and then cleared
    - the request remains pending until RTCore actually finishes the tail for that axis
  - Focused regression coverage now includes the race case where statusword bit 15 appears while `native_home_active_axis_mask` is still set.
  - Rolled the updated RTCore binary plus controller/backend reload into the live stack.
- Validation:
  - `source "/home/pi/GradientOS/.venv/bin/activate" && PYTHONPATH=src python -m pytest tests/test_gradient05_limits_and_backends.py -q -k "native_home"`
  - `make -C src/gradient_rt_motion`
  - live RTCore install/restart and launcher-managed controller/API reload
  - stack returned healthy: `controller up`, `api up`, `web up`, RTCore `BUS_UP_DISARMED`
- Follow-up notes / risks:
  - Yes, clicking the next Drive Home too soon after the previous one could cause a real issue before this fix, because the first request could report success before RTCore finished the internal persistence tail.
  - With this change, the first home should hold the request open until RTCore finishes the tail, so the UI’s existing global pending-action gate can serialize the next click naturally instead of releasing too early.

## 2026-04-13 01:17 +0000

- Task summary:
  - Reviewed and cleaned up the generated Chapter 11 Markdown after the first-pass PDF conversion rendered poorly and still contained row-grouping mistakes.
- Changes:
  - Rebuilt the `11.2` parameter sections in `docs/resources/a6ec_manual_chapter_11_parameter_list.md` into actual HTML tables instead of raw fenced text blocks.
  - Corrected several extraction/presentation issues:
    - removed leaked header/page-number artifacts from table rows
    - fixed `2000h` row grouping so names/options attach to the correct parameter
    - fixed hex parameter-code parsing such as `C01.0A`
    - restored the missing `607Dh` grouped-object `subindex 0` row in the `6000h` table
  - Kept the long `11.3` narrative material intact while preserving the overall chapter structure.
  - Deleted temporary TSV extraction files after the cleanup pass.
- Validation:
  - spot-checked the regenerated Markdown with `ReadFile`
  - targeted content checks with `rg` for known trouble rows such as `C00.14`, `C00.31`, `607Dh`, `607Fh`, and `60E3h`
  - `ReadLints` on `docs/resources/a6ec_manual_chapter_11_parameter_list.md` reported no diagnostics
- Follow-up notes / risks:
  - The chapter is now much cleaner and more reviewable, but some mathematical ranges inherited from the PDF still use manual-style spacing such as `2 32 -1` rather than fully typeset superscripts.

## 2026-04-13 01:03 +0000

- Task summary:
  - Persisted the in-flight native-home state into the existing frontend telemetry path so the UI can show “still working” and block further Drive Home clicks without adding a new API surface.
- Changes:
  - Extended `build_drive_fault_snapshot(...)` to carry:
    - top-level `native_home_active_axis_mask`
    - top-level `native_home_active_axis_mask_hex`
    - per-axis `native_home_active`
  - Kept the transport path unchanged:
    - RTCore metrics
    - `drive_faults` snapshot builder
    - existing `/monitor` SSE payload
    - existing `driveFaults` prop into `ControlPanel`
  - Updated `ControlPanel.tsx` so the commissioning section:
    - shows a persistent amber banner while any native-home transaction is still active
    - shows `Drive Home requested...` on the active joint row
    - disables all Drive Home buttons until the active-home mask clears
  - Reloaded the live stack and verified the live `driveFaults` snapshot now includes the new fields, with idle state currently reporting `native_home_active_axis_mask = 0x0`.
- Validation:
  - `source "/home/pi/GradientOS/.venv/bin/activate" && PYTHONPATH=src python -m pytest tests/test_drive_faults.py -q`
  - `cd web-ui && npm test -- --run ControlPanel.test.tsx -t "native-home transaction is still active"`
  - live stack restart plus direct live `build_drive_fault_snapshot(...)` check confirming `native_home_active_axis_mask` and per-axis `native_home_active` are present on the existing payload
- Follow-up notes / risks:
  - The next real-world proof is simply the next time you click Drive Home twice in quick succession: the first joint should stay visibly “running,” and the second Drive Home buttons should remain disabled until RTCore finishes the tail.

## 2026-04-13 01:07 +0000

- Task summary:
  - Captured the formal before-power baseline for the final all-axis retention proof.
- Changes:
  - Triggered the existing retention workflow:
    - experiment id `20260413-010728`
    - snapshot path `logs/encoder-retention/20260413-010728/before_power_down.json`
  - Collected a privileged all-axis SDO baseline for:
    - `0x2000:08`
    - `0x6041`
    - `0x6061`
    - `0x6063`
    - `0x6064`
    - `0x607C`
    - `0x603F`
    - `0x203F`
    - `0x2031:11`
    - `U40.14/.16/.18/.1A/.1C/.1E/.1F/.20/.22`
  - Key pre-cycle reference-unit positions:
    - axis 0 `0x6064 = 2`
    - axis 1 `0x6064 = 131059`
    - axis 2 `0x6064 = 131059`
    - axis 3 `0x6064 = 3`
    - axis 4 `0x6064 = 5`
    - axis 5 `0x6064 = 131071`
  - Key pre-cycle raw single-turn channels:
    - axis 0 `U40.1C = 38330`
    - axis 1 `U40.1C = 101040`
    - axis 2 `U40.1C = 25333`
    - axis 3 `U40.1C = 4759`
    - axis 4 `U40.1C = 21404`
    - axis 5 `U40.1C = 18702`
  - RTCore metrics at capture:
    - `armed = 0`
    - `axis_enable_mask = 0`
    - `native_home_active_axis_mask = 0`
    - `startup_ready = 1`
    - `wkc_actual = 18/12`
- Validation:
  - live `POST /control/encoder-retention/capture` with `phase=before_power_down`
  - live privileged `ethercat upload` readback for all six axes and the tracked encoder/drive objects
- Follow-up notes / risks:
  - This is the exact pre-power snapshot to compare against immediately after the next drive-only power cycle with the stack left running and disarmed.

## 2026-04-13 01:59 +0000

- Task summary:
  - Captured the formal after-power snapshot for the final all-axis retention proof and compared it against the pre-power baseline.
- Changes:
  - Triggered the existing retention workflow:
    - experiment id `20260413-010728`
    - after snapshot `logs/encoder-retention/20260413-010728/after_power_up.json`
    - comparison artifacts `comparison.json` and `comparison.md`
  - Post-power RTCore metrics at capture:
    - `armed = 0`
    - `axis_enable_mask = 0`
    - `native_home_active_axis_mask = 0`
    - `startup_ready = 1`
    - `startup_reset_count = 1`
    - `wkc_actual = 18/12`
  - Raw post-power drive observations:
    - axis 0 stayed near zero in reference units (`0x6064: 2 -> 4`)
    - axis 1 stayed in the persisted wrapped frame (`0x6064: 131059 -> 131059`)
    - axis 2 stayed in the persisted wrapped frame (`0x6064: 131059 -> 131058`)
    - axis 3 stayed near zero in reference units (`0x6064: 3 -> 5`)
    - axis 4 stayed near zero in reference units (`0x6064: 5 -> 4`)
    - axis 5 crossed the single-turn wrap boundary but stayed semantically near zero (`0x6064: 131071 -> 4`)
  - Raw encoder-oriented channels remained effectively stable on every axis within a few counts:
    - axis 0 `U40.1C/U40.20: 38330/38331 -> 38330/38330`
    - axis 1 `101040/101041 -> 101042/101042`
    - axis 2 `25333/25334 -> 25335/25332`
    - axis 3 `4759/4757 -> 4760/4760`
    - axis 4 `21404/21405 -> 21404/21405`
    - axis 5 `18702/18701 -> 18704/18703`
  - The formal comparison artifact still flags mismatch because it uses exact equality on axis counts and logical angles, which is now too strict for:
    - a few-count post-power dither
    - modulo-equivalent near-zero values like `131071` versus `4`
- Validation:
  - live `POST /control/encoder-retention/capture` with `phase=after_power_up` and `experiment_id=20260413-010728`
  - live privileged all-axis `ethercat upload` readback for the tracked encoder/drive objects
- Follow-up notes / risks:
  - Interpreting the raw drive state, the all-axis persistence rollout is successful: all six axes returned to the same semantic home frame after the final drive-only power cycle.
  - The remaining gap is report quality, not drive behavior: `comparison.json` / `comparison.md` should be made tolerance- and wrap-aware if we want the formal artifact to agree with the successful raw proof.

## 2026-04-13 02:17 +0000

- Task summary:
  - Wrote a repo-local fresh-agent handoff for the current unresolved `J2` native-home false-failure contradiction after hard stop + restart.
- Changes:
  - Added `HANDOFF_J2_NATIVE_HOME_FALSE_FAILURE_2026-04-13.md` with:
    - current branch/worktree state
    - relevant manual and experiment history
    - current live drive objects, raw metrics, and effective `driveFaults` snapshot for `J2`
    - the exact mismatch between command-response failure and live success
    - concrete next debugging directions and constraints
- Validation:
  - manual review of latest logs, metrics, direct EtherCAT reads, and live `build_drive_fault_snapshot(...)` output before writing the handoff
- Follow-up notes / risks:
  - The handoff is intentionally specific to the current false-failure contradiction and assumes the broader native-home persistence and display work already on the branch remains in place.

## 2026-04-13 02:27 +0000

- Task summary:
  - Fixed the remaining `J2` native-home false-failure by aligning backend command-result semantics with the already-correct live `driveFaults` interpretation for a clean HM-bit15 success state.
- Changes:
  - Updated `src/gradient_os/arm_controller/backends/ethercat_rtcore/backend.py` so `_native_home_metrics_result()` now:
    - treats a stale reported `failed` state / abort code as superseded when the active-home mask has cleared and the live wire-state shows HM bit 15 with zero live faults
    - preserves the raw reported abort code separately for debugging
  - Added focused regressions in `tests/test_gradient05_limits_and_backends.py` for:
    - stale failed report + clean live statusword success => verified success
    - stale failed report + live fault present => remains failed
- Validation:
  - `source "/home/pi/GradientOS/.venv/bin/activate" && PYTHONPATH=src python -m pytest tests/test_gradient05_limits_and_backends.py -q -k "native_home_metrics_result or wait_for_native_home_result_waits_for_active_mask_clear_before_statusword_fallback"`
  - `source "/home/pi/GradientOS/.venv/bin/activate" && PYTHONPATH=src python -m pytest tests/test_drive_faults.py -q -k native_home`
  - `source "/home/pi/GradientOS/.venv/bin/activate" && PYTHONPATH=src python -m py_compile src/gradient_os/arm_controller/backends/ethercat_rtcore/backend.py`
  - direct non-invasive live check against `/run/gradient-rt-motion/metrics.json` showing `J2` now evaluates to `terminal_state=succeeded`, `verified=true`, and `native_home_last_abort_code=0` while preserving the reported raw failure fields
- Follow-up notes / risks:
  - This removes the operator/API false-negative without changing the raw RTCore metrics fields themselves; any other consumer that reads raw `native_home_state` directly may still need the same effective-status interpretation instead of treating the raw field as final truth.

## 2026-04-13 03:06 +0000

- Task summary:
  - Fixed the `J6` jog / cross-axis regression by separating display-only A6-EC feedback normalization from the motion-safe controller feedback frame, then carried that split through the API and frontend monitor paths.
- Changes:
  - Updated `src/gradient_os/arm_controller/backends/ethercat_rtcore/backend.py` so:
    - `raw_to_joint_positions()` now preserves raw-wire-derived counts for controller/motion use
    - new `raw_to_display_joint_positions()` retains the signed single-turn / continuous display behavior for UI consumers only
    - `_axis_q_from_counts()` no longer normalizes A6-EC feedback counts, and display normalization now lives behind an explicit display helper
  - Updated `src/gradient_os/run_controller.py` to publish display-only joint angles separately:
    - `GET_JOINT_STATE` now includes `arm_display_rad` / `arm_display_deg`
    - monitor telemetry now includes `display_joints` alongside the motion-safe `joints`
  - Updated `src/gradient_os/api/main.py` so `/info/joints` now reuses `GET_JOINT_STATE` and surfaces optional `arm_display_*` fields instead of being limited to `GET_JOINT_ANGLES`
  - Updated `web-ui/src/ControlPanel.tsx`, `web-ui/src/liveState.tsx`, `web-ui/src/App.tsx`, and `web-ui/src/TelemetryCharts.tsx` so the frontend prefers display-only joint angles when present while leaving the command baseline untouched
  - Added/updated focused regressions in:
    - `tests/test_gradient05_limits_and_backends.py`
    - `tests/test_api_endpoints.py`
    - `web-ui/src/ControlPanel.test.tsx`
- Validation:
  - Live containment / recovery:
    - `curl -sS http://127.0.0.1:4000/control/motion-status` confirmed `state=faulted`, `active_traj_id=3`, `queue_depth=24`, and faulted axes `0/1/3`
    - `curl -sS -X POST http://127.0.0.1:4000/control/stop` followed by `curl -sS http://127.0.0.1:4000/control/motion-status` confirmed RTCore returned to `state=idle`, `active_traj_id=0`, `queue_depth=0`
    - `curl -sS -X POST http://127.0.0.1:4000/control/power-down -H 'Content-Type: application/json' -d '{"wait_for_idle":true}'` plus `/run/gradient-rt-motion/metrics.json` confirmed `armed=0` and `axis_enable_mask=0`
  - Focused regressions:
    - `source .venv/bin/activate && PYTHONPATH=src python -m pytest tests/test_gradient05_limits_and_backends.py -q -k "wraps_a6ec_single_turn_feedback_counts or unwraps_a6ec_feedback_continuously or preserves_raw_feedback_frame_for_controller_requeue or zero_capture_preserves_logical_zero_with_native_home_offset or enqueue_trajectory_points_keeps_controller_logical_frame_with_native_home_offset"`
    - `source .venv/bin/activate && PYTHONPATH=src python -m pytest tests/test_api_endpoints.py -q -k "info_joints or info_joints_detailed or control_joint_jog"`
    - `cd /home/pi/GradientOS/web-ui && npm test -- src/ControlPanel.test.tsx`
    - `source .venv/bin/activate && python -m py_compile src/gradient_os/arm_controller/backends/ethercat_rtcore/backend.py src/gradient_os/api/main.py src/gradient_os/run_controller.py`
- Follow-up notes / risks:
  - The drives remain faulted on axes `0/1/3` after the safe stop/power-down; the queue is cleared and power is down, but a deliberate fault-reset / power-up / re-test cycle is still needed before motion can be trusted again.

## 2026-04-13 04:14 +0000

- Task summary:
  - Implemented the multi-turn encoder truth rollout from the attached plan without changing the plan file: RTCore now exports raw A6-EC absolute objects, Python reconstructs a display-only continuous joint path with a persisted absolute-home anchor, and the existing API/monitor paths carry the new diagnostics.
- Changes:
  - Added `src/gradient_os/absolute_encoder_anchors.py` to persist per-joint absolute-home anchors separately from software zero offsets.
  - Extended `src/gradient_rt_motion/main.cpp` so the metrics thread polls and publishes per-axis raw `absolute_feedback` SDO fields for `U40.16`, `U40.1C`, `U40.1E`, `U40.20`, `U40.22`, `U40.28`, `U40.2A`, and `U40.2C`.
  - Updated `src/gradient_os/arm_controller/backends/ethercat_rtcore/backend.py` to:
    - cache the new `absolute_feedback` metrics block
    - reconstruct display-only joint angles from raw multi-turn encoder counts plus a persisted anchor
    - keep `raw_to_joint_positions()` motion-safe and unchanged
    - refresh the persisted anchor on verified native-home success and software-zero capture
    - expose `get_display_feedback_snapshot()` so controller/API code can reuse the same display snapshot and diagnostics.
  - Updated `src/gradient_os/run_controller.py` and `src/gradient_os/telemetry/drive_faults.py` so existing telemetry contracts carry the new data:
    - `display_joints` still comes through `/monitor`
    - `arm_display_*` still comes through `GET_JOINT_STATE`
    - detailed joint snapshots now include `axis_absolute_feedback`
    - `drive_faults.axes[*]` now includes normalized `absolute_feedback` plus combined multi-turn counts for proof/debug work.
  - Updated TypeScript payload types in `web-ui/src/liveState.tsx`, `web-ui/src/App.tsx`, and `web-ui/src/ControlPanel.tsx` for the expanded `drive_faults` payload.
  - Added focused regression coverage in:
    - `tests/test_gradient05_limits_and_backends.py`
    - `tests/test_drive_faults.py`
    - `tests/test_api_endpoints.py`
- Validation:
  - `source .venv/bin/activate && PYTHONPATH=src python -m pytest tests/test_gradient05_limits_and_backends.py tests/test_drive_faults.py tests/test_api_endpoints.py -q`
  - `source .venv/bin/activate && PYTHONPATH=src python -m pytest tests/test_rtcore_runtime.py -q`
  - `make -C src/gradient_rt_motion`
  - `cd /home/pi/GradientOS/web-ui && npm test -- src/ControlPanel.test.tsx`
  - `ReadLints` on the touched Python and TypeScript files reported no diagnostics.
- Follow-up notes / risks:
  - The display path now prefers persisted absolute truth only when both raw `absolute_feedback` metrics and a stored anchor are available; otherwise it deliberately falls back to the prior display-only normalization.
  - No live hardware proof was run in this implementation pass, so the bench matrix from the plan (`J6` multi-turn sweep, native-home shift check, drive-only power-cycle check) still needs to be executed before anyone considers migrating controller/IK seed truth away from the current `0x6064` motion frame.

## 2026-04-13 04:40 +0000

- Task summary:
  - Refactored the absolute-feedback rollout to restore the architecture boundary: A6-EC field definitions and source policy now live in the drive profile, while RTCore/backend/telemetry consume profile descriptors generically.
- Changes:
  - Added absolute-feedback descriptor, normalization, and source-resolution helpers to `src/gradient_os/arm_controller/profiles/drive/a6ec_ds402.py`.
  - Exposed those helpers through `src/gradient_os/arm_controller/profiles/registry.py` and `src/gradient_os/arm_controller/backends/registry.py`.
  - Extended `src/gradient_os/arm_controller/backends/ethercat_rtcore/runtime.py` and `systemd/rt-motion/gradient-rt-motion.service` with `GRADIENT_RT_ABSOLUTE_FEEDBACK_CONFIG`.
  - Refactored `src/gradient_rt_motion/main.cpp` to parse the new descriptor, poll the profile-specified SDO list, and emit metrics keyed by descriptor names instead of hardcoded `U40.*` slots.
  - Refactored `src/gradient_os/arm_controller/backends/ethercat_rtcore/backend.py` and `src/gradient_os/telemetry/drive_faults.py` to normalize and resolve absolute feedback through the drive-profile registry instead of embedded A6-EC maps.
  - Updated focused fixtures/tests in `tests/test_gradient05_limits_and_backends.py`, `tests/test_drive_faults.py`, `tests/test_api_endpoints.py`, and `tests/test_rtcore_runtime.py`.
- Validation:
  - `source .venv/bin/activate && PYTHONPATH=src python -m pytest tests/test_gradient05_limits_and_backends.py tests/test_drive_faults.py tests/test_api_endpoints.py tests/test_rtcore_runtime.py -q`
  - `make -C src/gradient_rt_motion`
  - `ReadLints` on the touched Python/test files reported no diagnostics.
- Follow-up notes / risks:
  - The metrics/API payload now uses profile-defined semantic keys such as `encoder_multi_turn_low` and `encoder_multi_turn_counts` instead of vendor object ids; any ad hoc tooling that scraped `u40_*` keys will need to follow the profile-driven schema.
  - No live hardware validation was run in this refactor pass; the existing multi-turn bench proof matrix still needs to be rerun after deploying/restarting the stack.

## 2026-04-13 04:50 +0000

- Task summary:
  - Investigated the live startup/current state after a hard stop and drive power cycle.
- What changed:
  - No code changes; this was a live-state inspection using the launcher, RTCore journal, metrics file, and API state snapshots.
- Validation / observations:
  - `./start-stack.sh status`
  - `./start-stack.sh probe`
  - `journalctl -u gradient-rt-motion.service --since "2026-04-13 04:44:00" --no-pager`
  - Read `/run/gradient-rt-motion/metrics.json`
  - Queried `http://127.0.0.1:4000/info/joints-detailed`
  - Startup outcome was healthy at the bus/drive level:
    - controller/api/web were up under `start-stack.sh`
    - RTCore reached EtherCAT `OP`
    - `responding/online/operational=6/6`
    - `physical_state=BUS_UP_DISARMED`
    - all axes reported `SwitchOnDisabled`, `statusword=0x1650`, `error_code=0x0000`
  - RTCore journal showed startup SDO writes for `a6ec_encoder_position_tracking_mode=4` succeeded on all six axes and the bus converged in about `8.7s`.
  - RTCore metrics still showed `startup_drive_config.readback_valid=0` and `verified=0` on all axes minutes later, despite the successful startup writes.
  - Live `/info/joints-detailed` showed valid per-axis absolute-feedback fields, but all axes were still using `display_source=raw_feedback_fallback` with no persisted `absolute_home_anchor_*` fields present in the payload.
- Follow-up notes / risks:
  - The current live issue is not a drive fault or bus-up problem; it is the missing startup-readback verification telemetry plus the absence of a currently applied absolute-home anchor in the display path.
  - Local direct `/monitor` fetches hung during this inspection even though API logs showed other `/monitor` requests returning `200`; `probe`, RTCore metrics, and `/info/joints-detailed` were used as the reliable live sources for this pass.

## 2026-04-13 04:58 +0000

- Task summary:
  - Re-reviewed the live logs and end-state values after drive power-up plus a J4 jog.
- What changed:
  - No code changes; this was a second live-state/log correlation pass.
- Validation / observations:
  - Read `logs/startups/latest/controller.log`, `logs/startups/latest/api.log`, and terminal `17.txt`
  - Queried `http://127.0.0.1:4000/info/joints-detailed`
  - Read `/run/gradient-rt-motion/metrics.json`
  - Ran `./start-stack.sh probe`
  - Controller log sequence:
    - `SAFE_POWER_UP` accepted
    - `APPLY_JOINT_SETPOINT` for J4 from `-19.943 deg` to `-20.943 deg`
    - open-loop executor started for 25 points at 100 Hz
    - controller later raised `TimeoutError` waiting for RTCore trajectory `1` to complete
  - Despite that timeout, the live post-jog state remained healthy:
    - `driver_state=ACTIVE`
    - `armed=1`, `enable_mask=0x3f`, `op_enabled_axes=6/6`
    - all six axes `OperationEnabled`
    - all six axes `error_code=0x0000`
  - Pre-jog vs post-jog correlation:
    - J4 `arm_display_rad` changed by about `-1.001 deg`, matching the commanded jog
    - J4 `absolute_counts` changed `4376 -> 10935` (`+6559`)
    - J4 raw `pos_counts` wrapped `130692 -> 6179`, making `arm_rad` appear to jump by about `+19 deg`
    - other joints only drifted slightly (`J2` about `+0.037 deg`, `J3` about `+0.031 deg`, others effectively zero)
  - Current end-state display values:
    - `J1`: raw `-3.595 deg`, display `0.005 deg`
    - `J2`: raw/display `0.042 deg`
    - `J3`: raw `-2.434 deg`, display `1.166 deg`
    - `J4`: raw/display `-0.943 deg`
    - `J5`: raw/display `-0.065 deg`
    - `J6`: raw/display `-6.287 deg`
  - All axes still reported `display_source=raw_feedback_fallback`, not persisted absolute-anchor display truth.
- Follow-up notes / risks:
  - The main “many values changed” effect on J4 was raw-frame wrap, not evidence of a large unintended physical move.
  - The remaining live bug is the trajectory completion timeout / bookkeeping path, plus the still-missing anchored absolute display path after restart.

## 2026-04-13 05:15 +0000

- Task summary:
  - Fixed the RTCore trajectory-completion false-timeout path for wrapped A6-EC motion feedback and attempted a live deploy/restart.
- Changes:
  - Added profile-owned motion-feedback wrap metadata to `src/gradient_os/arm_controller/profiles/drive/a6ec_ds402.py`, plus registry accessors in `src/gradient_os/arm_controller/profiles/registry.py` and `src/gradient_os/arm_controller/backends/registry.py`.
  - Extended `src/gradient_os/arm_controller/backends/ethercat_rtcore/runtime.py` and `systemd/rt-motion/gradient-rt-motion.service` with `GRADIENT_RT_FEEDBACK_WRAP_AXIS_MASK` / `--feedback-wrap-axis-mask`.
  - Updated `src/gradient_rt_motion/main.cpp` so trajectory completion uses shortest-periodic-error math modulo `counts_per_rev` when the active drive profile marks wrapped motion feedback.
  - Updated `tests/test_rtcore_runtime.py` to assert the rendered RTCore env now includes the new wrap-mask setting.
- Validation:
  - `pytest -q tests/test_rtcore_runtime.py`
  - `pytest -q tests/test_gradient05_limits_and_backends.py -k wait_for_trajectory_complete`
  - `make -C src/gradient_rt_motion`
  - `ReadLints` on the touched Python/C++/test files reported no diagnostics.
- Live deployment notes:
  - Synced the rebuilt RTCore binary/unit/env into `/usr/local/bin/gradient-rt-motion` and `/etc/default/gradient-rt-motion`; `systemctl status gradient-rt-motion.service` confirms the new process starts with `--feedback-wrap-axis-mask 0x3f`.
  - The live deploy hit an unrelated host-level blocker: the previous RTCore instance left a deleted-binary `metrics` thread stuck in `D` state plus a surviving `EtherCAT-OP` kernel thread, so new RTCore instances now log `Failed to reserve master: Device or resource busy`.
  - `sudo ethercat master` still reports `Master0 Active: yes` and 1 kHz traffic even while the fresh RTCore instance reports `ethercat_master_state=DOWN`, which shows the stale master owner is outside the new process and was not cleared by repeated `gradient-rt-motion.service` / `ethercat.service` restarts.
- Follow-up notes / risks:
  - The code fix is in the repo and the new binary is installed, but live hardware validation is blocked until the stale EtherCAT master owner is cleared; the current evidence points to a host reboot or deeper kernel/driver recovery rather than another application-level restart.

## 2026-04-13 06:00 +0000

- Task summary:
  - Investigated the post-reboot/post-power-up web UI pose flicker, proved the frontend was rendering the wrapped motion-safe joint channel, and switched the operator-facing UI back to the stable display-joint stream already published by the backend.
- Changes:
  - Updated `web-ui/src/App.tsx`:
    - parse `/monitor.display_joints` from the SSE payload instead of silently dropping it
    - add a display-preferred joint selector that falls back to raw `joints` only when no display pose is available
    - feed that preferred display pose into the telemetry panel and the 3D `ArmVisualizer`
  - Updated `web-ui/src/TelemetryCharts.tsx`:
    - chart the same display-preferred pose source so the joint history panels do not keep showing wrapped raw-count snap-backs
- Validation:
  - Live 30 s capture of `http://127.0.0.1:4000/monitor`
    - collected 1389 packets
    - raw `joints` toggled between wrap-adjacent poses on all six axes
    - `display_joints` remained stable within micro-radians
  - Live 6 s poll of `http://127.0.0.1:4000/info/joints-detailed`
    - `axis_counts` alternated among `0`, `1`, `131071`, and `131070`, matching wrapped single-turn boundary dithering rather than physical motion
  - `ReadLints` on `web-ui/src/App.tsx` and `web-ui/src/TelemetryCharts.tsx` reported no diagnostics
  - `npm run build` in `web-ui`
- Follow-up notes / risks:
  - The backend still intentionally publishes raw `/monitor.joints` for motion/debug semantics; any future operator-facing widget should prefer `display_joints` or `/info/joints.arm_display_*` rather than assuming `joints` is display-safe near a wrap boundary.

## 2026-04-13 06:00 +0000

- Task summary:
  - Investigated whether J3’s erratic commissioning motion was caused by display telemetry leaking back into the controller, proved from logs/code that motion still uses the raw controller frame, and added joint-jog diagnostics so future J3 commands expose the exact raw-vs-display baseline used.
- Changes:
  - Updated `src/gradient_os/api/main.py`:
    - changed `/control/joint-jog` preflight from `GET_JOINT_ANGLES` to `GET_JOINT_STATE`
    - preserved the existing raw `arm_deg` target math so motion semantics stay unchanged
    - added `_selected_joint_feedback_snapshot(...)` and route response fields for `current_arm_deg`, `current_arm_display_deg`, and `selected_joint_feedback`
    - added warning logs when a jog baseline shows `display_source=raw_feedback_fallback` or a significant raw-vs-display delta
  - Updated `tests/test_api_endpoints.py`:
    - refreshed joint-jog tests for the `GET_JOINT_STATE` preflight and new diagnostic response fields
- Validation:
  - Reviewed live controller logs from the active stack:
    - repeated J3 bounded jogs used raw `current_deg` samples of about `-3.569`, `-0.970`, `-1.970`, `-2.970`, and `-0.371` on successive requests
    - each target was computed as current minus `1.0 deg`, which proves the controller was using the raw baseline rather than a display-only value
  - Live `http://127.0.0.1:4000/info/joints-detailed` probe for J3:
    - current raw about `-1.3705 deg`
    - current display about `-4.9705 deg`
    - `absolute_source=encoder_multi_turn_counts`
    - `display_source=raw_feedback_fallback`
  - Confirmed there is currently no persisted `.gradient_absolute_encoder_anchors.json` file in the repo root, so the anchored absolute-display path is not active on this boot
  - `source "/home/pi/GradientOS/.venv/bin/activate" && PYTHONPATH=src python -m pytest tests/test_api_endpoints.py -q -k 'control_joint_jog'`
  - `source "/home/pi/GradientOS/.venv/bin/activate" && PYTHONPATH=src python -m py_compile src/gradient_os/api/main.py tests/test_api_endpoints.py`
  - `ReadLints` on the touched files reported no diagnostics
- Follow-up notes / risks:
  - This pass improves observability and proves that display telemetry is not what is driving J3 motion. The remaining motion bug is that the raw commissioning baseline itself can be untrustworthy across commands, and J3 is still on `raw_feedback_fallback` instead of an anchored absolute display source after reboot.

## 2026-04-13 06:58 +0000

- Task summary:
  - Implemented the canonical anchored-joint-truth plan so the controller/API/UI now prefer one logical joint truth derived from absolute multi-turn counts plus persisted home anchors, while RTCore keeps using its raw drive-wire frame only for command translation.
- Changes:
  - Updated `src/gradient_os/arm_controller/backends/ethercat_rtcore/backend.py`:
    - replaced raw-wire-derived `raw_to_joint_positions()` / `raw_to_display_joint_positions()` reads with a shared anchored-absolute canonical snapshot path
    - removed the silent `raw_feedback_fallback` behavior from operator/controller truth and made missing anchors or absolute counts fail closed with explicit `truth_available` diagnostics
    - kept `_axis_q_from_joint_positions()` as the canonical-truth -> raw-wire conversion step for queued RTCore targets
  - Updated `src/gradient_os/run_controller.py`:
    - made `arm_display_*` and `/monitor.display_joints` compatibility aliases of the canonical joint truth instead of a separate pose source
    - added `canonical_joint_truth_available` and unavailable-axis/joint metadata to the joint snapshot payload
    - forwarded `axis_absolute_feedback` on `/monitor` as diagnostics only
  - Updated `src/gradient_os/api/main.py`:
    - made `/control/joint-jog` reject cached or anchor-missing baselines with `CANONICAL_JOINT_TRUTH_UNAVAILABLE`
    - extended selected-joint diagnostics with canonical-truth availability/source fields
  - Updated `web-ui/src/ControlPanel.tsx`, `web-ui/src/App.tsx`, and `web-ui/src/TelemetryCharts.tsx`:
    - prefer canonical `arm_deg` / `joints` first and treat `arm_display_*` / `display_joints` as legacy aliases only
  - Updated `tests/test_gradient05_limits_and_backends.py`, `tests/test_api_endpoints.py`, and `web-ui/src/ControlPanel.test.tsx`:
    - replaced split-truth assertions with continuous canonical-truth coverage
    - added fail-closed coverage for missing anchors/canonical truth during joint-jog baselining
- Validation:
  - `source ./start.sh && python -m pytest tests/test_gradient05_limits_and_backends.py tests/test_api_endpoints.py -q -k 'canonical or absolute or joint_jog or connected_reads or native_home_offsets_to_feedback_but_not_command_targets or robot_defined_axis_scaling or converts_feedback_using_axis_scaling or info_joints or info_joints_detailed or info_pose'`
    - result: `18 passed, 101 deselected`
  - `source ./start.sh && python -m py_compile src/gradient_os/arm_controller/backends/ethercat_rtcore/backend.py src/gradient_os/run_controller.py src/gradient_os/api/main.py tests/test_gradient05_limits_and_backends.py tests/test_api_endpoints.py`
  - `npm run test -- src/ControlPanel.test.tsx` in `web-ui`
    - result: `12 passed`
  - `npm run build` in `web-ui`
  - `ReadLints` on all touched Python/TS/TSX files reported no diagnostics
- Follow-up notes / risks:
  - This was validated with focused local tests/builds only; no live hardware proof was run in this pass, so the bench still needs the canonical truth / no-wrap / no-cross-axis-motion sequence from the plan.
  - Older mock-based API tests still allow `arm_display_*` to differ from `arm_*` because the API intentionally preserves legacy payload fields during transition; the real controller/runtime path now aliases them to the canonical truth instead.

## 2026-04-13 20:51 +0000

- Task summary:
  - Fixed the live canonical-truth regression by restoring anchor availability at startup instead of weakening the no-fallback contract.
- Changes:
  - Updated `src/gradient_os/arm_controller/backends/ethercat_rtcore/backend.py`:
    - added `_bootstrap_missing_absolute_home_anchors(...)`
    - call that bootstrap from `initialize()` after RTCore feedback is ready so missing `absolute_home_anchor_*` state is reconstructed from live raw-frame alignment plus absolute multi-turn counts
    - removed the attempted raw-live fallback path so `get_joint_positions()` still depends on canonical truth rather than a second read contract
  - Updated `tests/test_gradient05_limits_and_backends.py`:
    - added coverage that startup bootstrap creates missing anchors when raw + absolute alignment is sufficient
    - kept fail-closed coverage when bootstrap cannot reconstruct canonical truth
- Validation:
  - `source ./start.sh && python -m pytest tests/test_gradient05_limits_and_backends.py -q -k 'startup_bootstraps_missing_absolute_home_anchor or anchor_bootstrap_cannot_reconstruct_truth or canonical or connected_reads or native_home_offsets_to_feedback_but_not_command_targets'`
    - result: `7 passed, 45 deselected`
  - `source ./start.sh && python -m py_compile src/gradient_os/arm_controller/backends/ethercat_rtcore/backend.py tests/test_gradient05_limits_and_backends.py`
  - Live stack cycle:
    - `./start-stack.sh stop --hard`
    - `./start-stack.sh`
  - Live startup evidence:
    - controller log: `Bootstrapped absolute-home anchors from live raw/absolute alignment: joints=[1, 2, 3, 4, 5, 6] actor=ethercat_rtcore:startup_alignment`
    - startup banner: `// LIVE STATE // canonical truth: AVAILABLE`
  - Live API checks:
    - `curl -sf http://127.0.0.1:4000/info/joints-detailed`
      - result: `read_source="live_feedback"`, `canonical_joint_truth_available=true`, per-axis `absolute_home_anchor_*` populated
    - `curl -sf http://127.0.0.1:4000/info/pose`
      - result: `200 OK` with nonzero pose/joint payload
  - `ReadLints` on touched files reported no diagnostics
- Follow-up notes / risks:
  - Startup bootstrap currently trusts the live raw frame as the migration bridge when anchors are absent; if bench evidence later shows a case where the raw frame is not already the intended logical truth at startup, tighten the bootstrap preconditions rather than reintroducing a read fallback.

## 2026-04-13 21:38 +0000

- Task summary:
  - Investigated the remaining J3/J4 erratic commissioning behavior, traced it to a canonical-read / raw-write frame mismatch, and removed the last Python-side encoder fallback paths.
- Changes:
  - Updated `src/gradient_os/arm_controller/backends/ethercat_rtcore/backend.py`:
    - fixed `_axis_q_from_joint_positions()` so canonical targets now re-apply both `absolute_home_anchor_rad` and software zero before RTCore converts into raw wire counts
    - updated `_store_last_axis_target_q()` to translate raw reference-frame targets back into canonical joint space using the same anchor-aware inverse
    - kept the no-anchor fail-close contract by raising if command conversion is attempted without an absolute-home anchor
  - Updated `src/gradient_os/run_controller.py`:
    - removed `GET_JOINT_STATE` downgrade to `cached_fallback`; encoder truth is now `live_feedback` or explicit `unavailable`
  - Updated `src/gradient_os/arm_controller/trajectory_execution.py`:
    - removed the secondary fallback from failed `raw_to_joint_positions()` conversion to `backend.get_joint_positions()`
  - Updated `tests/test_gradient05_limits_and_backends.py`:
    - changed the canonical->raw-wire regression test to use a nonzero absolute-home anchor
    - added a guard test that command conversion raises when no absolute-home anchor exists
- Validation:
  - Reviewed live controller logs from `logs/startups/20260413-205037/controller.log`
    - J4 jogs moved from about `-18.988 -> -23.990 deg`, then later back toward `-19.966 deg`
    - J3 jogs then targeted about `-1.718 -> -3.717 deg` while J4 held at about `-19.966 deg`
    - these commands were single-joint targets, which pointed away from obvious multi-joint command chatter and toward frame conversion mismatch
  - Compared live startup anchors in `.gradient_absolute_encoder_anchors.json`
    - J4 startup anchor about `0.3364 rad` (`~19.27 deg`)
    - J3 startup anchor about `0.1135 rad` (`~6.50 deg`)
  - `source ./start.sh && python -m pytest tests/test_gradient05_limits_and_backends.py -q -k 'translates_canonical_truth_back_into_raw_wire_counts or refuses_command_conversion_without_absolute_home_anchor or startup_bootstraps_missing_absolute_home_anchor or anchor_bootstrap_cannot_reconstruct_truth or native_home_offsets_to_feedback_but_not_command_targets'`
    - result: `5 passed, 48 deselected`
  - `source ./start.sh && python -m py_compile src/gradient_os/arm_controller/backends/ethercat_rtcore/backend.py src/gradient_os/run_controller.py src/gradient_os/arm_controller/trajectory_execution.py tests/test_gradient05_limits_and_backends.py`
  - `ReadLints` on touched files reported no diagnostics
- Follow-up notes / risks:
  - I could not live-retest the new write-frame fix because the post-edit stack reload hit the known host-level stale-RTCore-owner failure: `./start-stack.sh` escalated to `REBOOT REQUIRED` after one recycle attempt (`master_device_busy`, `request_master_failed`, `leftover_process`, `sigkill_survivor`).
  - The code fix is ready, but the next meaningful proof requires a host reboot, then a fresh J4/J3 jog retest on the restarted stack.

## 2026-04-13 21:47 +0000

- Task summary:
  - Reviewed the startup-stack cleanup regression and fixed the newly introduced false-positive reboot path in RTCore startup recovery.
- Changes:
  - Updated `src/gradient_os/telemetry/startup_recovery.py`:
    - removed the immediate `rtcore_not_up` hard-recycle condition
    - replaced it with a non-recovering `rtcore_starting_or_down` classification that tells the launcher to continue normal startup/readiness waits unless real stale-owner/master-busy signatures are present
  - Updated `tests/test_startup_recovery.py`:
    - added regression coverage that `rtcore_state=UNKNOWN`, `ethercat_master_state=DOWN`, `physical_state=INACTIVE` does **not** trigger an automatic recycle without busy signatures
- Validation:
  - Reviewed the referenced transcript and current `start-stack.sh` flow:
    - transcript intent: recycle only the `RTCore UP / EtherCAT DOWN / master busy` class
    - current regression: `ensure_rtcore_runtime_sync()` called the classifier immediately after `sync-runtime.sh --ensure-active`, and the classifier treated `rtcore not up yet` as a hard-recycle case
  - `source ./start.sh && python -m pytest tests/test_startup_recovery.py -q`
    - result: `5 passed`
  - `source ./start.sh && python -m py_compile src/gradient_os/telemetry/startup_recovery.py tests/test_startup_recovery.py`
  - `ReadLints` on touched files reported no diagnostics
- Follow-up notes / risks:
  - The current machine is still in the stale-owner state shown in `terminals/1.txt`; that host-level condition can still genuinely require a reboot. This fix prevents the launcher from creating/escalating that state on normal slow startups, but it cannot retroactively clear the already-stuck owner without a clean reboot.

## 2026-04-13 18:45 +0000

- Task summary:
  - Hardened startup recovery for the RTCore/EtherCAT stack so the launcher no longer trusts an "active" RTCore that failed to reserve the EtherCAT master, and so stale-master-owner failures escalate with an explicit reboot-required message after one controlled recycle attempt.
- Changes:
  - Added `src/gradient_os/telemetry/startup_recovery.py`:
    - classifies a startup probe plus recent RTCore journal text into `healthy`, `should_recover`, or `reboot_required`
    - detects the specific stale-owner signatures seen on the bench (`Device or resource busy`, `ecrt_request_master(0) failed`, leftover-process / SIGKILL-survivor messages)
  - Updated `start-stack.sh`:
    - split raw RTCore sync into `sync_rtcore_runtime_once()`
    - added one-shot prelaunch RTCore/EtherCAT recycle logic when the probe shows `rtcore_state=UP` but `ethercat_master_state=DOWN`
    - emits an explicit reboot-required startup failure when the same stale-owner signatures remain after one recycle attempt
  - Updated `src/gradient_rt_motion/main.cpp`:
    - turns `ecrt_request_master(0)` failure into a real process exit path by setting `g_stop` and returning a nonzero exit code from the service
  - Added `tests/test_startup_recovery.py`:
    - covers healthy startup, recoverable `RTCore UP / EtherCAT DOWN`, reboot-required stale-owner signatures after recovery, and the no-false-positive "normal bus convergence" case
- Validation:
  - `bash -n start-stack.sh`
  - `source ./start.sh && python -m pytest tests/test_startup_recovery.py tests/test_rtcore_runtime.py -q`
    - result: `12 passed`
  - `source ./start.sh && python -m py_compile src/gradient_os/telemetry/startup_recovery.py`
  - `make -C src/gradient_rt_motion`
  - `ReadLints` on `src/gradient_os/telemetry/startup_recovery.py`, `tests/test_startup_recovery.py`, `start-stack.sh`, and `src/gradient_rt_motion/main.cpp`
    - result: no diagnostics
- Follow-up notes / risks:
  - This pass improves recovery when the old master owner is still killable, but it cannot guarantee recovery from a kernel-blocked `D`-state RTCore/EtherCAT owner; that path still correctly escalates to a host reboot.
  - Live bench validation is still needed to confirm the launcher takes the new recycle path automatically on hardware and produces the expected reboot-required message only when the master remains reserved.

## 2026-04-13 18:58 +0000

- Task summary:
  - Added an operator-facing CLI startup banner so `start-stack.sh` now prints a big `GradientOS` header plus useful live stack/runtime info when the staged launcher starts.
- Changes:
  - Updated `start-stack.sh`:
    - added `render_runtime_banner_summary()` to resolve the active desired runtime summary from `runtime_config`
    - added `print_start_banner()` with a large ASCII `GradientOS` header and startup metadata
    - banner includes mode, run id, robot, tool, IK backend, servo backend, drive profile, RT max RPM, controller/API/web endpoints, log path, and common launcher commands
    - called the banner from `start_stack()` immediately after environment bootstrap so it appears once at startup before RTCore/controller bring-up logs
- Validation:
  - `bash -n start-stack.sh`
  - `./start-stack.sh --help`
  - `ReadLints` on `start-stack.sh`
    - result: no diagnostics
- Follow-up notes / risks:
  - This was validated as a syntax/help-path change only; I did not run a full live startup on hardware in this pass, so the exact terminal appearance should be confirmed on the next real `./start-stack.sh` run.

## 2026-04-13 19:05 +0000

- Task summary:
  - Added ANSI color to the `start-stack.sh` startup banner with safe terminal detection and opt-out controls.
- Changes:
  - Updated `start-stack.sh`:
    - added `init_banner_palette()` and `banner_stat_line()` helpers
    - colorized the banner border, `GradientOS` ASCII header, labels, values, and command hint rows
    - added `GRADIENT_STACK_COLOR=auto|0|1` support and auto-disabled color for non-interactive/dumb terminals or when `NO_COLOR` is set
- Validation:
  - `bash -n start-stack.sh`
  - `ReadLints` on `start-stack.sh`
    - result: no diagnostics
- Follow-up notes / risks:
  - I still did not run a full live stack startup in this pass, so the exact palette/spacing should be eyeballed on the next terminal launch and tweaked if you want a more neon, minimal, or boxed style.

## 2026-04-13 19:14 +0000

- Task summary:
  - Refined the terminal styling to better match the user’s industrial reference, and made recovery/reboot actions stand out as color-coded callout blocks instead of plain inline log text.
- Changes:
  - Updated `start-stack.sh`:
    - colorized `WARNING` and `ERROR` prefixes
    - added reusable text styling helpers for danger, warning, info, and command actions
    - added `print_callout_block()` for visually separated recovery/reboot panels
    - updated the startup banner to include an industrial caution line and more branded copy
    - highlighted banner commands (`probe`, `status`, `stop`, `stop --hard`) in green
    - changed startup recovery output so `REBOOT REQUIRED`, `REBOOT HOST`, and the one-shot recycle plan render as separate emphasized blocks
    - updated the interactive console ready line to colorize the common commands
- Validation:
  - `bash -n start-stack.sh`
  - `ReadLints` on `start-stack.sh`
    - result: no diagnostics
- Follow-up notes / risks:
  - I did not rerun a live failing startup after this visual-only pass, so the exact operator-facing appearance of the new recovery callouts should be checked on the next real terminal run and adjusted if you want even stronger contrast or tighter spacing.

## 2026-04-13 19:22 +0000

- Task summary:
  - Added a true green success path so the launcher now visibly announces when the full staged boot has actually completed.
- Changes:
  - Updated `start-stack.sh`:
    - added a `SUCCESS` log helper with bright green styling
    - changed the bus-ready message to emit as a green success indicator
    - added `print_boot_success_block()` so the launcher prints a green `SYSTEM ONLINE` callout after controller, bus, API, and optional web readiness all pass
    - reused the runtime summary from the startup banner so the success panel shows the real robot/backend/drive values rather than generic placeholders
- Validation:
  - `bash -n start-stack.sh`
  - `ReadLints` on `start-stack.sh`
    - result: no diagnostics
- Follow-up notes / risks:
  - I still did not run a full live successful startup in this pass, so the exact terminal presentation of the new green success block should be checked on the next successful `./start-stack.sh` run and tuned if needed.

## 2026-04-13 19:38 +0000

- Task summary:
  - Expanded the terminal UX beyond the banner so staged startup now has quieter bootstrap output, styled timestamps/log lines, stage-by-stage success messages, and tty-only loading indicators that better match the website’s industrial aesthetic.
- Changes:
  - Updated `start.sh`:
    - added `GRADIENT_START_QUIET=1` support so `start-stack.sh` can suppress the old plain bootstrap chatter while still sourcing the environment normally
  - Updated `start-stack.sh`:
    - added tty-only loading helpers (`ui_loading_status`, `ui_status_clear`) with animated ASCII progress frames
    - switched log output to a shared formatter with styled timestamp and `[start-stack]` tag
    - added an `INFO` level and kept `SUCCESS` / `WARNING` / `ERROR` visually distinct
    - replaced raw bootstrap output with a branded `ENVIRONMENT READY` callout
    - reformatted RTCore sync into an explicit stage with a cleaner failure panel and highlighted inspection commands
    - added loading/success feedback for controller readiness, generic probe waits, API readiness, web readiness, bus convergence, and selected shutdown waits
    - made process launches report as explicit success events instead of plain pid lines
- Validation:
  - `bash -n start-stack.sh && bash -n start.sh`
  - `bash -lc 'cd /home/pi/GradientOS && export GRADIENT_START_QUIET=1 && source ./start.sh >/tmp/grad_start_quiet.out && wc -l /tmp/grad_start_quiet.out'`
    - result: `0 /tmp/grad_start_quiet.out`
  - `ReadLints` on `start-stack.sh` and `start.sh`
    - result: no diagnostics
- Follow-up notes / risks:
  - This pass was validated structurally rather than with a full live stack start, so the exact pacing/visual density of the spinner lines and staged success output should be checked on the next real `./start-stack.sh` run and tuned from the live terminal capture.

## 2026-04-13 19:48 +0000

- Task summary:
  - Fixed a `set -u` regression in the new terminal styling layer where the launcher crashed before startup because the palette globals were referenced before initialization.
- Changes:
  - Updated `start-stack.sh`:
    - predeclared all `BANNER_*` and `UI_*` palette variables near the top-level variable block so styled logger/helper calls are safe before `init_banner_palette()` runs
- Validation:
  - `bash -n start-stack.sh`
  - `./start-stack.sh --help`
  - `./start-stack.sh`
    - result: progressed through the styled startup path and exited with the expected real stale-owner `REBOOT REQUIRED` failure instead of `UI_INFO: unbound variable`
- Follow-up notes / risks:
  - The immediate crash is fixed; any further startup failures are now genuine runtime/service issues rather than shell formatting regressions.

## 2026-04-13 20:06 +0000

- Task summary:
  - Added real stage timing instrumentation to the staged launcher and used a live startup run to confirm which phases are actually slow enough to justify animated indicators.
- Changes:
  - Updated `start.sh`:
    - added `GRADIENT_START_QUIET=1` support so `start-stack.sh` can suppress legacy bootstrap chatter and present a single styled boot flow
  - Updated `start-stack.sh`:
    - added `now_ms()` and `format_duration_ms()` helpers
    - extended tty spinner status lines to show `t+...` elapsed time while a stage is in progress
    - added persistent elapsed-time success/failure logs for environment bootstrap, RTCore sync, generic probe waits, controller readiness, bus readiness, API readiness, web readiness, systemd service stop, and full recovery recycle
    - aligned the ASCII logo rows so the `GradientOS` mark no longer leans left in the middle
- Validation:
  - `bash -n start-stack.sh && bash -n start.sh`
  - `ReadLints` on `start-stack.sh` and `start.sh`
    - result: no diagnostics
  - `./start-stack.sh`
    - progressed through the styled startup path and produced real stage timings before hitting the expected stale-owner reboot path
    - observed timings:
      - environment ready: about `66-68ms`
      - initial RTCore sync: about `17.348s`
      - `ethercat.service` stop during recycle: about `38.670s`
      - full recovery recycle: about `41.991s`
- Follow-up notes / risks:
  - The timing instrumentation now gives enough data to decide where the animated status line is worthwhile. The slow RTCore/EtherCAT phases are clearly worth it; the environment stage is probably too fast to need persistent animation unless the styling is desired for consistency.

## 2026-04-13 20:13 +0000

- Task summary:
  - Styled the recovery-time probe snapshot so launcher probe dumps no longer fall back to the old raw `probe: key=value ...` format.
- Changes:
  - Updated `start-stack.sh`:
    - added `style_probe_state()` to colorize `UP`/`DOWN`/`INACTIVE`/`FAULTED`/`DISARMED`-style values appropriately
    - added `style_probe_ratio()` to colorize readiness and `wkc` ratios
    - replaced `log_probe_snapshot()`’s raw Python one-liner with a styled `PROBE SNAPSHOT` callout block that shows physical/driver/EtherCAT/RTCore state plus armed/mask/op-enabled/WKC status
- Validation:
  - `bash -n start-stack.sh`
  - `ReadLints` on `start-stack.sh`
    - result: no diagnostics
- Follow-up notes / risks:
  - This only styles the launcher-side snapshot block used during startup/recovery flows. If we also want the explicit `./start-stack.sh probe` command output to adopt the same callout/palette language, that can be done in a follow-up pass.

## 2026-04-13 20:20 +0000

- Task summary:
  - Replaced the pseudo-loading line with a real tty-only animated spinner for the long blocking startup stages.
- Changes:
  - Updated `start-stack.sh`:
    - added `UI_SPINNER_PID`, `ui_loading_begin()`, and `run_with_loading_capture()`
    - changed `ui_status_clear()` to stop any active spinner process before printing the next log/callout line
    - wrapped blocking RTCore sync (`sync-runtime.sh --ensure-active`) in the new spinner helper
    - wrapped blocking `systemctl stop` calls in the new spinner helper so slow EtherCAT stop paths animate too
- Validation:
  - `bash -n start-stack.sh`
  - `ReadLints` on `start-stack.sh`
    - result: no diagnostics
  - `./start-stack.sh`
    - progressed through the styled startup path and hit the expected RTCore sync failure path after about `34.438s`
    - confirms the long blocking sync stage is now a genuine spinner candidate rather than a one-frame status line
- Follow-up notes / risks:
  - The animated spinner is tty-only and redraws in place, so it will not appear as an animation in static screenshots or persisted terminal transcripts; only the live terminal shows the moving frames.

## 2026-04-13 20:29 +0000

- Task summary:
  - Investigated why the latest startup run did not complete and patched the launcher so active bus convergence extends the readiness deadline instead of tripping the generic 20 s timeout.
- Changes:
  - Updated `start-stack.sh`:
    - added `GRADIENT_STACK_BUS_PROGRESS_GRACE_S` (default `15`) and `GRADIENT_STACK_BUS_MAX_TIMEOUT_S` (default `60`)
    - changed `wait_for_bus_operational()` to track fieldbus progress (`responding`, `online`, `operational`, `startup_ready`, `link_up`)
    - when progress increases, extend the readiness window up to the hard cap instead of failing immediately at the base timeout
    - updated the timeout error to report total elapsed wait plus the base/grace/hard-cap values
    - passed dynamic remaining time into the live timeout counter so the fieldbus spinner reflects the extended deadline
- Validation:
  - `bash -n start-stack.sh`
  - `ReadLints` on `start-stack.sh`, `src/gradient_os/telemetry/startup_recovery.py`, and `tests/test_startup_recovery.py`
    - result: no diagnostics
- Follow-up notes / risks:
  - This patch addresses the specific run where the bus was still converging (`0/6 -> 5/6 -> BUS_UP_DISARMED`) as the old 20 s deadline expired. Live hardware validation is still needed to confirm the extended deadline now gives the bus enough time to reach `startup_ready=1` instead of falling out on the generic timeout.

## 2026-04-13 20:40 +0000

- Task summary:
  - Fixed the remaining startup blocker so `start-stack.sh` no longer exits before the web UI when canonical pose truth is unavailable during bring-up, and confirmed the full controller/API/web stack now stays live.
- Changes:
  - Updated `src/gradient_os/arm_controller/command_api.py`:
    - wrapped `handle_get_position()` joint sampling in a local exception handler
    - when canonical joint truth is unavailable, the controller now sends `ERROR,GET_POSITION,...` back to the caller instead of throwing a traceback through the main loop
  - Updated `start-stack.sh`:
    - changed `wait_for_api_readiness()` so `/info/pose` is best-effort during startup rather than a hard gate for web bring-up
    - API health, runtime-config sanity, and joints readiness still remain required
- Validation:
  - `bash -n start-stack.sh`
  - `source ./start.sh && python -m pytest tests/test_api_endpoints.py -q -k 'info_pose'`
    - result: `2 passed, 67 deselected`
  - `source ./start.sh && python -m py_compile src/gradient_os/arm_controller/command_api.py src/gradient_os/api/main.py`
  - `ReadLints` on `start-stack.sh`, `src/gradient_os/arm_controller/command_api.py`, and `tests/test_api_endpoints.py`
    - result: no diagnostics
  - `./start-stack.sh status`
    - result: launcher `running`, `controller: up`, `api: up`, `web: up`
- Follow-up notes / risks:
  - The stack is now staying live through web bring-up, but canonical joint truth is still unavailable on the bench, so the controller continues to log `GET_JOINT_STATE` warnings. That is a real backend/absolute-truth issue to solve next, separate from the launcher completion bug.

## 2026-04-13 22:25 +0000

- Task summary:
  - Tightened the EtherCAT RTCore joint-read contract so the backend fails closed instead of silently returning the last commanded setpoint when live canonical truth is unavailable.
- Changes:
  - Updated `src/gradient_os/arm_controller/backends/ethercat_rtcore/backend.py`:
    - changed `get_joint_positions()` to raise `Canonical joint truth unavailable (...)` when RTCore is disconnected or feedback config is not ready
    - removed the getter path that returned `_last_joint_setpoint_rad` as pseudo-feedback
    - clarified the `_last_joint_setpoint_rad` comment so it is explicitly command bookkeeping only, not read truth
  - Updated `tests/test_gradient05_limits_and_backends.py`:
    - kept the corrected canonical-to-raw command-frame regression coverage intact
    - added fail-closed tests for disconnected reads and connected-but-feedback-not-ready reads
  - Updated `.cursor/memory/AGENT_SCRATCHPAD.md`:
    - marked the earlier same-day "re-apply anchor on writes" note as superseded
    - recorded the corrected transform algebra and the new fail-closed getter guardrail
- Validation:
  - `source ./start.sh && python -m pytest tests/test_gradient05_limits_and_backends.py -q -k 'translates_canonical_truth_back_into_raw_wire_counts or connected_reads_return_canonical_feedback or disconnected_get_joint_positions_fails_closed or connected_without_feedback_config_fails_closed or startup_bootstraps_missing_absolute_home_anchor or anchor_bootstrap_cannot_reconstruct_truth or native_home_offsets_to_feedback_but_not_command_targets'`
    - result: `7 passed, 47 deselected`
  - `source ./start.sh && python -m py_compile src/gradient_os/arm_controller/backends/ethercat_rtcore/backend.py tests/test_gradient05_limits_and_backends.py`
  - `ReadLints` on `src/gradient_os/arm_controller/backends/ethercat_rtcore/backend.py` and `tests/test_gradient05_limits_and_backends.py`
    - result: no diagnostics
- Follow-up notes / risks:
  - The repo now enforces the single-source/fail-closed read contract locally, but live hardware proof after the already-reverted anchor write-path fix is still outstanding.
  - I did not restart the stack or command the robot in this pass; the next meaningful validation is a controlled J4-only then J3-only live jog proof against fresh `/info/joints-detailed` snapshots.

## 2026-04-13 22:50 +0000

- Task summary:
  - Investigated the newer live regression run and hardened the backend so anchored absolute feedback is only considered motion-safe when it round-trips back into the current raw/reference command frame.
- Changes:
  - Updated `src/gradient_os/arm_controller/backends/ethercat_rtcore/backend.py`:
    - added per-axis command-frame roundtrip diagnostics (`command_roundtrip_*`) inside `_canonical_joint_positions_from_raw_feedback()`
    - fail-closes canonical truth with `truth_reason=command_frame_roundtrip_mismatch` when `canonical -> command frame` does not reconstruct the current `reference_pre_zero_rad` within about one count
    - factored the command transform through `_command_axis_q_for_joint_value()` / `_canonical_joint_q_from_command_axis_q()` so the diagnostic uses the exact same math as the upload path
  - Updated `tests/test_gradient05_limits_and_backends.py`:
    - added a regression test that marks truth unavailable when `absolute_feedback + anchor` is present but does not round-trip into the current raw/reference motion frame
  - Updated `.cursor/memory/AGENT_SCRATCHPAD.md`:
    - recorded the live roundtrip-mismatch diagnosis and the new rule that canonical truth must be proven motion-safe, not merely present
- Validation:
  - Live inspection on the already-running stack:
    - `curl -s http://127.0.0.1:4000/info/joints-detailed`
    - `curl -s http://127.0.0.1:4000/info/pose`
    - `logs/startups/20260413-224227/controller.log`
    - key evidence: the controller sent a J4-only target, but the live snapshot simultaneously reported `canonical_joint_truth_available=true` while exposing a frame inconsistency (`J1 canonical_rad=0.21456`, `reference_pre_zero_rad=-0.03677`, zero software offsets), which means the current anchored absolute source was not command-frame-safe
  - Local focused tests:
    - `source ./start.sh && python -m pytest tests/test_gradient05_limits_and_backends.py -q -k 'translates_canonical_truth_back_into_raw_wire_counts or connected_reads_return_canonical_feedback or marks_truth_unavailable_when_absolute_anchor_does_not_roundtrip or disconnected_get_joint_positions_fails_closed or connected_without_feedback_config_fails_closed or startup_bootstraps_missing_absolute_home_anchor or anchor_bootstrap_cannot_reconstruct_truth or native_home_offsets_to_feedback_but_not_command_targets'`
    - result: `8 passed, 47 deselected`
  - `source ./start.sh && python -m py_compile src/gradient_os/arm_controller/backends/ethercat_rtcore/backend.py tests/test_gradient05_limits_and_backends.py`
  - `ReadLints` on the touched backend/test files
    - result: no diagnostics
- Follow-up notes / risks:
  - The running stack has not picked up this guardrail yet; a controller/backend restart is required before the live API will stop advertising the current inconsistent anchored source as motion-safe.
  - This change intentionally blocks motion when the chosen `encoder_multi_turn_counts` source and current native-home/reference frame are semantically inconsistent. It does not yet prove which underlying source is wrong; it only prevents the stack from trusting it for motion until that proof exists.

## 2026-04-13 23:33 +0000

- Task summary:
  - Attempted the requested clean restart and gathered a fresh disarmed proof snapshot plus manual-backed conclusions about which A6-EC objects are raw encoder-unit state versus drive reference/home state.
- Changes:
  - No additional source-code changes in this pass beyond the earlier roundtrip guard; this pass focused on runtime verification and manual interpretation.
  - Updated `.cursor/memory/AGENT_SCRATCHPAD.md` with the manual-confirmed frame split and the failed-restart fieldbus blocker.
- Validation:
  - `./start-stack.sh status`
    - result before restart: stack fully down
  - `./start-stack.sh`
    - result: failed during bus readiness on run `20260413-233217`
    - observed progress: `responding=6/6`, then stalled at `online=5/6 operational=5/6 startup_ready=0`
  - `./start-stack.sh probe`
    - result: `J5/axis4` offline (`slave_online=False`, `slave_operational=False`, `sw=0x1640`) while the other five axes were `SwitchOnDisabled` / `0x1650`
  - Manual/source review:
    - `docs/resources/a6ec_manual_chapter_11_parameter_list.md`
    - confirmed `C00.07=4` label (`Absolute position rotation mode`)
    - confirmed `U40.20/.22` are encoder-unit multi-turn data
    - confirmed `U40.16`, `6064h`, `607Ah`, and `607Ch` are reference/home-frame quantities
    - confirmed `607Ch` is active only when powered on, homing complete, and `6041h bit 15 = 1`, and that after homing `6064h = 607Ch`
- Follow-up notes / risks:
  - Because the fieldbus never reached `startup_ready=1`, the controller/API never launched and the new Python fail-closed truth guard is not active in the live stack yet.
  - The strongest current conclusion is not "raw encoder unstable" but "raw encoder-unit state and reference/home-frame state are being conflated"; vendor confirmation is still needed for the exact manufacturer-intended canonical source and homing/reference workflow.

## 2026-04-14 00:17 +0000

- Task summary:
  - Completed the fresh-boot disarmed object comparison and verified that the guarded stack now blocks unsafe joint truth instead of feeding false positions to the frontend.
- Changes:
  - No new source edits in this pass.
  - Updated `.cursor/memory/AGENT_SCRATCHPAD.md` with the fresh-boot SDO comparison findings.
- Validation:
  - Successful guarded restart on run `20260413-235354` after the user hard-stopped and power-cycled the drives.
  - Live API checks:
    - `curl -s http://127.0.0.1:4000/info/joints-detailed`
    - `curl -s -i http://127.0.0.1:4000/info/pose`
    - result: `canonical_joint_truth_available=false`, `command_frame_roundtrip_mismatch`, and `GET /info/pose` `503`
  - Direct disarmed EtherCAT SDO comparison (two snapshots one second apart) for every axis:
    - `0x6041`, `0x6064`, `0x607C`, `0x2040:17`, `0x2040:21`, `0x2040:23`, `0x2040:2B`, `0x2040:2D`
    - result highlights:
      - all axes `0x6041 = 0x1650`, `bit15 = 0`, `bit12 = 1`
      - all axes `0x607C = 0`
      - `0x6064`, `U40.16`, and `U40.2A/.2C` were mutually close and stable
      - `U40.20/.22` were also stable (`0..3` count drift over 1 s), so the raw multi-turn source does not currently look noisy
      - the large mismatch remains semantic/frame-related rather than transport noise
- Follow-up notes / risks:
  - The frontend is blank because the backend intentionally returns empty joint arrays when canonical truth is unsafe; this is expected fail-closed behavior, not a separate frontend transport bug.
  - The fresh-boot evidence now points away from "unstable encoder reads" and toward "stable raw encoder data but wrong/inactive relationship to the drive's reference/home frame after boot."

## 2026-04-14 00:17 +0000

- Task summary:
  - Ran the first controlled per-axis native-home experiment on `J4` to test whether the drive's home/reference-valid state is something startup should merely expose or something the native-home transaction actively establishes.
- Changes:
  - No new code changes in this pass.
  - Updated `.cursor/memory/AGENT_SCRATCHPAD.md` with the J4 before/after findings.
- Validation:
  - Pre-home `J4` snapshot:
    - direct SDOs (`axis3`): `6041=0x1650`, `bit15=0`, `607C=0`, `6064=28359`, `U40.16=28359`, `U40.20=163933`, `U40.2A=28358`
    - live API axis detail: `truth_reason=command_frame_roundtrip_mismatch`
  - Command:
    - `curl -s -X POST http://127.0.0.1:4000/control/home-joint-native -H 'Content-Type: application/json' -d '{"joint":4}'`
    - result: `verified=true`, `terminal_state=succeeded`, `native_home_state=2`, `disarmed_after_home=true`
  - Post-home `J4` snapshot:
    - direct SDOs (`axis3`): `6041=0x9650`, `bit15=1`, `bit12=1`, `607C=0`, `6064≈131063`, `U40.16≈-8`, `U40.20≈32852`, `U40.2A≈131064`
    - one-second stability re-read stayed within a few counts
    - live API axis detail: `truth_available=true`, `command_roundtrip_consistent=true` for axis 3
    - global API status still unavailable because other axes remain mismatched
- Follow-up notes / risks:
  - This strongly suggests native home changes the drive's actual state for that axis rather than us merely "using" an already-present startup-valid flag.
  - Persistence across a later power cycle is still the key unresolved test; J4 is now the cleanest candidate for that follow-up.

## 2026-04-14 00:38 +0000

- Task summary:
  - Completed the `J4` persistence follow-up after a real drive power cycle and confirmed that J4's corrected frame survived reboot even though HM bit 15 did not stay set.
- Changes:
  - No source edits in this pass.
  - Updated `.cursor/memory/AGENT_SCRATCHPAD.md` with the post-restart J4 persistence finding.
- Validation:
  - Post-restart stack/probe on run `20260414-003639`:
    - all 6 axes online/operational, disarmed
    - `J4/axis3 pos_counts=131062`
  - Direct post-restart `J4` SDO snapshot:
    - `6041=0x1650` (`bit15=0`, `bit12=1`)
    - `607C=0`
    - `6064=131063`
    - `U40.16=131062`
    - `U40.20=32848`
    - `U40.2A=131061`
  - Live API snapshot after restart:
    - `axis 3` remained `command_roundtrip_consistent=true` / `truth_available=true`
    - global unavailable axes shrank to `[0, 1, 2]`
- Follow-up notes / risks:
  - This disproves the simpler hypothesis that persistence requires HM bit 15 to remain high after reboot. The semantic/reference-frame correction for J4 persisted, but the statusword dropped back to `0x1650`.
  - `607C` stayed zero throughout the successful J4 home and reboot, so the current workflow's persisted effect is not obviously witnessed by nonzero `607C`.
  - The next systematic tests should repeat the same sequence on another failing axis (best candidates: `J3`, then `J1/J2`) and keep separating frame persistence from status-bit persistence.

## 2026-04-14 01:02 +0000

- Task summary:
  - Correlated the user's selected Group 6000 manual clauses with live `J3`/`J4` reads to see which parts match the observed behavior and which parts do not.
- Changes:
  - No code edits in this pass.
  - Updated `.cursor/memory/AGENT_SCRATCHPAD.md` with the manual/live-correlation findings.
- Validation:
  - Manual extracts reviewed from `docs/resources/a6ec_manual_chapter_11_parameter_list.md`:
    - `607Ch` home offset
    - `6099.02h` speed during search for zero
    - `60E3.01h` supported homing method semantics
    - `60E6h` actual position calculation method
    - `60F4h`, `60FCh`, `60FDh`
    - `U40.16` and `U40.28` descriptions
  - Live object reads on `J3/axis2` and `J4/axis3`:
    - `6098=35`
    - `60E6=0`
    - `60F4=0`
    - `60FD=0`
    - `60FC` closely matched the drive-facing rotation/reference frame (`J3 52562`, `J4 131061`)
    - `60E3` supported-method entries include `35`, and every listed method currently reports both relative and absolute support bits set
- Follow-up notes / risks:
  - The manual strongly supports focusing on `60E6` and `60FC` for the canonical-truth question.
  - The short `607C` paragraph does not fully explain the successful J4 path, because J4 still shows `607C=0` while the corrected frame persisted and round-tripped safely.

## 2026-04-14 02:42 +0000

- Task summary:
  - Ran the full controlled `J3` native-home persistence sequence: pre-home snapshot, native-home transaction, immediate post-home capture, soft-stop while keeping RTCore/EtherCAT up, real drive power cycle, guarded restart, and post-restart verification.
- Changes:
  - No source-code edits in this pass.
  - Updated `.cursor/memory/AGENT_SCRATCHPAD.md` with the new `J3` persistence findings.
  - Stored runtime artifacts under `logs/encoder-retention/20260414-020640-j3-native-home-sequence`.
- Validation:
  - Pre-home `J3` snapshot:
    - `6041=0x1650`, `bit15=0`, `607C=0`, `6064=52564`, `U40.16=52563`, `60FC=52562`, `U40.20=-446379`
    - API reported `truth_reason=command_frame_roundtrip_mismatch` for axis 2
  - Native-home command:
    - `curl -s -X POST http://127.0.0.1:4000/control/home-joint-native -H 'Content-Type: application/json' -d '{"joint":3}'`
    - result: `verified=true`, `terminal_state=succeeded`, `native_home_state=2`, `disarmed_after_home=true`
  - Immediate post-home `J3` snapshot:
    - `6041=0x9650`, `bit15=1`, `607C=0`, `6064~=131041..131042`, `U40.16~=-30..-32`, `60FC~=131041..131042`, `U40.20=77880`
    - API initially accepted axis 2 with anchor `0.02548325583875021`
  - Post-home truth poll:
    - both `J3` and `J4` flickered between accepted and rejected as roundtrip error moved between `1` and `2-3` counts
  - Soft-stop / power-cycle staging:
    - first `./start-stack.sh stop` removed the launcher only
    - second `./start-stack.sh stop` performed the intended soft stop, leaving `rtcore_state=UP`, `ethercat_master_state=OP`, and `physical_state=BUS_UP_DISARMED`
  - Direct post-power-cycle pre-restart snapshot:
    - `J3` came back with `6041=0x1650`, `bit15=0`, `607C=0`, but the corrected frame persisted: `6064=131042`, `U40.16=131041`, `60FC=131041`, `U40.28=131041`, `U40.2A=131041`, `U40.20=77882`
  - Guarded restart on run `20260414-024152`:
    - full stack reached `STACK BOOT COMPLETE`
    - startup banner still reported global canonical truth unavailable because some other axes remain unresolved
  - Post-restart API and SDO snapshot:
    - global unavailable joints `[1, 2, 6]` / axes `[0, 1, 5]`
    - `J3`: `6041=0x1650`, `bit15=0`, `607C=0`, `6064=131042`, `U40.16=131040`, `60FC=131041`, `U40.20=77881`, API `truth_available=true`, near-zero roundtrip error
    - `J4`: `6041=0x1650`, `bit15=0`, `607C=0`, `6064=131051`, `U40.16=131053`, `60FC=131053`, `U40.20=32840`, API `truth_available=true`, near-zero roundtrip error
  - Post-restart truth poll:
    - `J3` still flickered (`true,true,true,true,true,true,true,false,false,true,false,true`) as error hopped between `0/1` and `2` counts
    - `J4` also flickered early before settling mostly true
- Follow-up notes / risks:
  - `J3` now matches `J4` on the persistence question: the corrected reference/frame effect survives real power loss even though `6041 bit15` clears after reboot and `607C` remains `0`.
  - The remaining issue for `J3/J4` is no longer "does the native-home frame persist?" but "why does the roundtrip guard ride a 1-2 count edge and intermittently reject otherwise semantically-correct axes?"

## 2026-04-14 02:48 +0000

- Task summary:
  - Re-reviewed the live motion read/write path in code to answer which counts are actually used for motion versus canonical truth reconstruction.
- Changes:
  - No source-code edits in this pass.
  - Updated `.cursor/memory/AGENT_SCRATCHPAD.md` with the code-level transform conclusion.
- Validation:
  - Confirmed in `src/gradient_os/arm_controller/backends/ethercat_rtcore/backend.py`:
    - `sync_read_positions()` returns RTCore `_axis_counts`
    - `raw_to_joint_positions()` reconstructs canonical truth from absolute feedback plus persisted anchor and then roundtrip-checks against the motion/reference frame
    - `_command_axis_q_for_joint_value()` adds only software zero
    - `_canonical_joint_q_from_command_axis_q()` subtracts only software zero
    - `_reference_q_before_master_offset_for_axis()` applies `native_home_offset` on the read/reference side
  - Confirmed in `src/gradient_rt_motion/ipc_v1.hpp` and `src/gradient_rt_motion/main.cpp`:
    - RTCore `AxisStatusV1.pos_counts` is documented as `0x6064`
    - queued CSP targets are stored in the same raw wire-count frame the drive publishes on `0x6064` and expects on `0x607A`
    - RTCore subtracts `native_home_offset_counts` exactly once when converting controller targets to CSP wire counts
  - Confirmed in `src/gradient_os/arm_controller/profiles/drive/a6ec_ds402.py`:
    - preferred absolute source order remains `encoder_multi_turn_counts` (`U40.20/.22`) first, `rotation_mode_encoder_counts` (`U40.2A/.2C`) second
  - Confirmed in `tests/test_gradient05_limits_and_backends.py`:
    - canonical truth translates back into the same raw wire counts
    - native-home offsets apply to feedback but not command targets
- Follow-up notes / risks:
  - The motion command path is now clean and does not re-apply the persisted absolute-home anchor.
  - The remaining open question is not command-path math but whether `U40.20/.22 + anchor` is the right long-term canonical host truth for all axes/states, or whether the corrected rotation/reference frame should eventually replace it.

## 2026-04-14 02:55 +0000

- Task summary:
  - Verified the manual/math around 17-bit encoder resolution versus the exposed multi-turn objects and answered why large gear ratios do not require commanding raw multi-turn encoder counts directly.
- Changes:
  - No source-code edits in this pass.
  - Updated `.cursor/memory/AGENT_SCRATCHPAD.md` with the clarified numeric/manual conclusion.
- Validation:
  - Manual checks in `docs/resources/a6ec_manual_chapter_11_parameter_list.md`:
    - `C00.07=4` is `Absolute position rotation mode`
    - `6064h` is position actual value in `user-defined unit`
    - `607Ah` target position unit is `Reference unit`
    - `6091h` states motor position feedback in encoder units is load/reference feedback times gear ratio
    - `60FCh` states encoder-unit position reference is derived from reference-unit position reference via `6091h`
    - `U40.1E` (`Encoder multi-turn position data`) is `U16`, range `0-65535 Rev`
    - `U40.20` / `U40.22` are low/high 32-bit halves of encoder multi-turn data in encoder units
  - Conclusion verified:
    - `2^32 / 2^17 = 32768` is only the span of a hypothetical unsigned 32-bit encoder-count accumulator
    - it is not a manufacturer-verified limit for the actual exposed A6-EC multi-turn objects we are reading
- Follow-up notes / risks:
  - The current open problem remains semantic frame selection for canonical truth, not insufficient turn range on the CSP motion path.
  - If we want the exact physical retained multi-turn limit of the encoder hardware itself, the parameter-list extract does not state it cleanly; that may require a deeper Chapter 5/manual lookup or vendor confirmation.

## 2026-04-14 04:22 +0000

- Task summary:
  - Read Chapter 5 more deeply, verified the current live gear-ratio objects, and added a read-only Chapter 5 probe script so future before/after-home/restart checks can test the frame model directly.
- Changes:
  - Added `scripts/a6ec_chapter5_probe.py`.
  - Updated `.cursor/memory/AGENT_SCRATCHPAD.md` with the new Chapter 5 probe findings.
- Validation:
  - Manual findings from `docs/resources/chapter 5 absolute system - extract from A6-EC_series_servo_drive_manual (2).pdf`:
    - absolute encoder is `131072 (2^17)` counts/rev with `16-bit multi-turn data saved`
    - `U40.1E` is `0..65535 Rev`
    - rotation mode applies when unidirectional load revolutions are `<32767`
  - Live SDO verification on `J3` and `J6`:
    - `6091.01 = 1`, `6091.02 = 1`
    - `C10.18 = 1`, `C10.19 = 1`
    - current live stack is not using drive-side gear-ratio mapping
  - Spot checks on `J1/J2/J3/J6`:
    - `combined(U40.20/.22)` matches `sign_extend16(U40.1E) * 131072 + (U40.1C mod 131072)` within `0..1` count
  - Script validation:
    - `python -m py_compile scripts/a6ec_chapter5_probe.py`
    - `python scripts/a6ec_chapter5_probe.py snapshot --label current --axes J3 J6`
    - result artifact: `logs/encoder-retention/20260414-042146-a6ec-ch5-probe/current.json`
    - result summary:
      - `J3` and `J6` both show `6091 = 1:1`
      - `J3` and `J6` both show `C10.18/C10.19 = 1:1`
      - `6063 ~= 6064*6091`, `60FC ~= 6062*6091`, and `U40.2A/.2C ~= U40.28*(C10.18/C10.19)` all matched within `0..1` count
- Follow-up notes / risks:
  - The raw multi-turn formula is now supported both by Chapter 5 structure and by live data, but the remaining architectural question is still which frame should be host canonical truth after native home.
  - The new script is the right harness for the next manual-backed experiment: capture the same axes across boot, native-home, and restart and compare which of the raw/reference/rotation bridges remain stable.

## 2026-04-14 05:29 +0000

- Task summary:
  - Ran the new Chapter 5 probe on `J6` first, as requested, and verified the manual-derived formulas there with both a single snapshot and a short repeated poll.
- Changes:
  - No source-code edits in this pass beyond the earlier script addition.
  - Stored new `J6` artifacts under `logs/encoder-retention/20260414-052921-a6ec-ch5-probe/`.
  - Updated `.cursor/memory/AGENT_SCRATCHPAD.md` with the `J6` verification result.
- Validation:
  - Probe command:
    - `python scripts/a6ec_chapter5_probe.py snapshot --label j6-current --axes J6`
  - Artifact:
    - `logs/encoder-retention/20260414-052921-a6ec-ch5-probe/j6-current.json`
    - `logs/encoder-retention/20260414-052921-a6ec-ch5-probe/j6-current.md`
  - Single-snapshot `J6` result:
    - `6091.01=1`, `6091.02=1`
    - `C10.18=1`, `C10.19=1`
    - raw formula `combined(U40.20/.22) ~= sign_extend16(U40.1E)*131072 + (U40.1C mod 131072)` matched with `delta=+1`
    - `6063 ~= 6064*6091` matched with `delta=-1`
    - `60FC ~= 6062*6091` matched with `delta=0`
    - `U40.2A/.2C ~= U40.28*(C10.18/C10.19)` landed at `delta=+2` on that sample
  - Repeated poll:
    - saved as `logs/encoder-retention/20260414-052921-a6ec-ch5-probe/j6-bridge-poll.json`
    - observed jitter bands:
      - raw formula delta `-2 .. +2`
      - `6063 - 6064*6091` delta `-1 .. +2`
      - `60FC - 6062*6091` delta `-3 .. +2`
      - rotation-mode bridge delta `-2 .. +2`
- Follow-up notes / risks:
  - `J6` does support the Chapter 5 frame model semantically.
  - The practical verification lesson is the same as with the roundtrip guard work: live reads wander by a couple of counts, so a single `2-count` miss should be treated as jitter unless it persists systematically.

## 2026-04-14 05:44 +0000

- Task summary:
  - Completed the requested `J6` native-home persistence sequence after the user's drive power cycle, then compared the result against the earlier `J3` and `J4` power-cycle proofs and wrote a consolidated evidence table into `.cursor/memory/AGENT_SCRATCHPAD.md`.
- Changes:
  - No production source-code changes in this pass.
  - Captured new experiment artifacts under `logs/encoder-retention/20260414-053539-j6-ch5-persistence-controls/`.
  - Updated `.cursor/memory/AGENT_SCRATCHPAD.md` with a three-axis persistence table covering `J3`, `J4`, and `J6`.
- Validation:
  - Pre-restart disarmed probe after the user's power cycle:
    - `./start-stack.sh probe`
    - result: RTCore/EtherCAT stayed up and disarmed; `J6/axis5` returned `sw=0x1650` and `pos_counts=0`
  - Snapshot sequence:
    - `python scripts/a6ec_chapter5_probe.py snapshot --label post-power-cycle-pre-restart --axes J3 J4 J6 --experiment-id 20260414-053539-j6-ch5-persistence-controls`
    - `python scripts/a6ec_chapter5_probe.py snapshot --label post-restart --axes J3 J4 J6 --experiment-id 20260414-053539-j6-ch5-persistence-controls`
  - Restart run:
    - `./start-stack.sh`
    - startup run id: `20260414-053857`
  - Post-restart truth poll:
    - saved to `logs/encoder-retention/20260414-053539-j6-ch5-persistence-controls/post-restart-truth-poll.json`
    - acceptance counts over 12 samples:
      - `J3`: `11/12` true
      - `J4`: `9/12` true
      - `J6`: `7/12` true
    - all three axes stayed inside the previously observed `0..3` count roundtrip/jitter band rather than showing a large semantic mismatch
- Follow-up notes / risks:
  - `J6` now matches the same persistence pattern already seen on `J3` and `J4`: the corrected reference frame survives a real drive power cycle, while `6041 bit15` clears after boot and `607C` remains `0`.
  - The remaining live issue is not missing persistence on these tested axes; it is that the current one-count roundtrip acceptance threshold flickers on otherwise semantically-correct axes when live reads wander by `1..3` counts.
  - Global truth still remains unavailable on this restart because unresolved axes plus that tolerance-edge flicker keep some joints marked unavailable at any given sample.

## 2026-04-14 05:50 +0000

- Task summary:
  - Re-read the latest scratchpad/devlog plus commissioning safety guidance and turned the new `J3/J4/J6` evidence into a concrete recommendation for how to validate the remaining unverified joints.
- Changes:
  - No production code changes.
  - Added a procedural recommendation to `.cursor/memory/AGENT_SCRATCHPAD.md` for batching the remaining native-home proofs in one shared power-cycle session while keeping homes strictly sequential.
- Validation:
  - Re-read latest evidence entries in `.cursor/memory/AGENT_SCRATCHPAD.md` and `.cursor/memory/DEVLOG.md`.
  - Re-read `.cursor/skills/gradientos-sop/commissioning-safety.md` to keep the recommendation aligned with the current commissioning rule set.
- Follow-up notes / risks:
  - Recommendation is to batch the remaining target axes in one session, but only with `home -> snapshot/poll` done one axis at a time and a single shared power cycle after all immediate post-home captures.
  - A previously persisted axis should be included as a control in shared pre/post-power-cycle captures when practical.
  - This batching recommendation applies to persistence validation only; it does not resolve the separate roundtrip-guard jitter issue that still affects global truth/frontend stability.

## 2026-04-14 06:05 +0000

- Task summary:
  - Started the all-joints batch experiment, homed `J5` and `J1` successfully, then retried `J2` under explicitly idle RTCore conditions to test whether the earlier `J2` failure was just "too soon after `J1`".
- Changes:
  - No production code changes.
  - Captured new all-joints artifacts under `logs/encoder-retention/20260414-055631-all-joints-native-home-batch/`, including `pre-home`, `post-home-j5`, `post-home-j1`, `post-home-j2-failed`, `pre-retry-j2`, and `post-retry-j2-failed` plus truth-poll JSONs.
  - Updated `.cursor/memory/AGENT_SCRATCHPAD.md` with the new `J2` retry conclusion.
- Validation:
  - `J5` home:
    - `POST /control/home-joint-native {"joint":5}` -> `verified=true`
    - immediate truth poll summary: `J5 true 7/8`
  - `J1` home:
    - `POST /control/home-joint-native {"joint":1}` -> `verified=true`
    - immediate truth poll summary: `J1 true 5/8`
  - `J2` first attempt in this batch:
    - `POST /control/home-joint-native {"joint":2}` -> `verified=false`, abort `0x06010002`
    - immediate `/info/joints-detailed` still showed `J2` with a large stable roundtrip mismatch around `-52184` counts
  - Serialized retry preconditions:
    - `./start-stack.sh probe` showed `BUS_UP_DISARMED`
    - `/run/gradient-rt-motion/metrics.json` showed `native_home_active_axis_mask = 0`
  - `J2` serialized retry:
    - second `POST /control/home-joint-native {"joint":2}` -> same abort `0x06010002`
    - follow-up probe showed axis 1 faulted with `sw=0x9638`, `err=0xff00`, `Er11.0`
  - Fault cleanup attempt:
    - `POST /control/reset-faults` was accepted
    - immediate follow-up probe still showed axis 1 faulted, controller state `FAULTED`, drives disarmed
- Follow-up notes / risks:
  - The clean retry strongly argues this is not just a "frontend guardrail / clicked too soon after `J1`" issue; RTCore idle was confirmed before the retry and the same `J2` abort reproduced.
  - `J2` is now the blocker for the shared batch. Do not continue to the shared power-cycle stage until `J2` is either recovered cleanly or explicitly excluded from this run.
  - The experiment currently left the stack disarmed but faulted on axis 1 after the retry sequence.

## 2026-04-14 06:15 +0000

- Task summary:
  - Investigated the contradictory frontend success message reported by the user, confirmed that the per-joint row was deriving success from fallback telemetry instead of surfacing the contradiction, and patched the UI to fail safe.
- Changes:
  - Updated `web-ui/src/ControlPanel.tsx` so a row that is only "successful" via `statusword_bit15` fallback but still has a reported failed native-home result with nonzero abort code now renders `Drive Home verification conflicted | reported failed ...` instead of `Drive Home succeeded`.
  - Added a targeted regression test to `web-ui/src/ControlPanel.test.tsx`.
  - Updated `.cursor/memory/AGENT_SCRATCHPAD.md` with the new UI/root-cause rule and the user's observation that `J2` audibly de-energised much faster than the other axes.
- Validation:
  - `npm run test -- --run src/ControlPanel.test.tsx` in `web-ui`
  - result: `13 passed`
  - `ReadLints` on the changed frontend files returned no diagnostics.
- Follow-up notes / risks:
  - This UI fix addresses the misleading success message only; it does not solve the underlying `J2` native-home failure/retry behavior.
  - The user's audible timing clue supports the idea that `J2` is aborting early rather than completing the normal post-home tail.

## 2026-04-14 06:30 +0000

- Task summary:
  - Restarted from the user's soft-stopped state, explained the stale-anchor hypothesis against live `J2` data, and ran a focused `J2` trace to see whether a clean restarted epoch could home `J2` successfully.
- Changes:
  - No production backend changes in this pass.
  - Captured a focused `J2` experiment under `logs/encoder-retention/20260414-062709-j2-focused-trace/`.
  - Updated `.cursor/memory/AGENT_SCRATCHPAD.md` with the new `J2` stale-anchor interpretation and successful focused retry result.
- Validation:
  - Soft-stopped pre-restart probe:
    - `./start-stack.sh probe`
    - result: `J2 sw=0x9650 err=0x0000 pos_counts=178`, launcher absent, RTCore/EtherCAT still up
  - Pre-restart and pre-home snapshots:
    - `python scripts/a6ec_chapter5_probe.py snapshot --label pre-restart --axes J2 --experiment-id 20260414-062709-j2-focused-trace`
    - `python scripts/a6ec_chapter5_probe.py snapshot --label pre-home-post-restart --axes J2 --experiment-id 20260414-062709-j2-focused-trace`
  - Restart run:
    - `./start-stack.sh`
    - startup run id: `20260414-062712`
  - Pre-home API state after restart:
    - `/info/joints-detailed` still showed `J2 command_roundtrip_reference_error_counts ~= -52159` with old anchor `0.04842154167659891`
  - Focused trace:
    - saved to `logs/encoder-retention/20260414-062709-j2-focused-trace/j2-home-trace.json`
    - result summary:
      - `native_home_active_axis_mask` was seen active for `J2`
      - statuswords seen: `0x8233`, `0x9650`
      - error codes seen: `0x0000`
      - command result: `NATIVE_HOME_VERIFIED`
      - command finished at about `4.9s`
  - Post-home verification:
    - `./start-stack.sh probe` -> `J2 sw=0x9650 err=0x0000 pos_counts=27`
    - `python scripts/a6ec_chapter5_probe.py snapshot --label post-home-success --axes J2 --experiment-id 20260414-062709-j2-focused-trace`
    - post-home API poll saved to `post-home-success-poll.json`
    - poll result: `J2 true 10/12`, roundtrip error `-1 .. +2` counts, refreshed anchor `0.02350346188438531`
- Follow-up notes / risks:
  - This strongly supports the stale-anchor explanation for the huge software-side mismatch we saw before the focused retry.
  - A clean restarted epoch can still home `J2` successfully, so the earlier repeated batch failures were not enough to conclude that `J2` is fundamentally broken.
  - The earlier `Er11.0` event still needs caution; the new result does not prove that fault was benign, only that the current clean single-axis retry path can succeed.

## 2026-04-14 06:59 +0000

- Task summary:
  - Implemented the stale-anchor hardening plan in the EtherCAT RTCore backend so clean stale anchors diagnose explicitly and native-home verification now depends on coherent post-home anchor refresh.
- Changes:
  - Updated `src/gradient_os/arm_controller/backends/ethercat_rtcore/backend.py`:
    - reused `derive_effective_native_home_status()` for the stale-anchor diagnosis path
    - added helpers that compute implied anchor delta/tolerance and classify clean mismatches as `absolute_home_anchor_stale`
    - changed `native_home_joint()` so post-home anchor capture failures are no longer swallowed and the command returns `NATIVE_HOME_ANCHOR_REFRESH_FAILED` unless capture plus roundtrip validation both succeed
    - added `_absolute_home_anchor_validation_for_joint()` so the verified path proves stored-anchor coherence before reporting success
  - Updated `tests/test_gradient05_limits_and_backends.py` with focused coverage for:
    - stale-anchor diagnosis on a clean homed-looking axis
    - native-home anchor capture failure
    - native-home post-home validation failure
    - conservative startup bootstrap behavior with an existing anchor
  - Updated `.cursor/memory/AGENT_SCRATCHPAD.md` with the new backend guardrails and validation rule.
- Validation:
  - `source "/home/pi/GradientOS/.venv/bin/activate" && PYTHONPATH=src python -m py_compile "src/gradient_os/arm_controller/backends/ethercat_rtcore/backend.py" "tests/test_gradient05_limits_and_backends.py"`
  - `source "/home/pi/GradientOS/.venv/bin/activate" && PYTHONPATH=src python -m pytest tests/test_gradient05_limits_and_backends.py -q -k 'native_home_captures_absolute_encoder_anchor or native_home_reports_anchor_refresh_failure_when_capture_raises or native_home_requires_post_home_anchor_validation or marks_truth_unavailable_when_absolute_anchor_does_not_roundtrip or diagnoses_stale_absolute_home_anchor_when_clean_homed_frame_disagrees or startup_bootstraps_missing_absolute_home_anchor or startup_bootstrap_keeps_existing_absolute_home_anchor'`
    - result: `7 passed, 52 deselected`
  - `source "/home/pi/GradientOS/.venv/bin/activate" && PYTHONPATH=src python -m pytest tests/test_api_endpoints.py -q -k 'control_home_joint_native'`
    - result: `3 passed, 66 deselected`
  - `npm test -- ControlPanel.test.tsx` in `web-ui`
    - result: `13 passed`
  - `ReadLints` on the edited backend/test files returned no diagnostics.
- Follow-up notes / risks:
  - The stale-anchor threshold is intentionally wider than the observed `0..3` count jitter band so only material anchor drift gets classified as stale; the broader roundtrip jitter issue itself remains open.
  - This pass validated the new command/result contract with focused tests only; I did not re-run live hardware native-home experiments after the code change.

## 2026-04-14 19:31 +0000

- Task summary:
  - Captured the current A6-EC frame-semantics and native-home lessons in durable repo docs, then routed the `gradientos-sop` skill to that note without pretending the workstream is already fully canonical.
- Changes:
  - Added `docs/ethercat/a6ec-frame-semantics-and-native-home.md` as a durable but explicitly provisional workstream note covering:
    - why `scripts/a6ec_chapter5_probe.py` exists
    - raw vs rotation vs CSP/reference frame separation
    - the current frame equations and anchor math
    - persistence, jitter, verification, UI-trust, and methodology lessons
    - the remaining open questions
  - Updated `.cursor/skills/gradientos-sop/SKILL.md` with an `Active Workstream Notes` route to the new doc.
  - Updated `.cursor/skills/gradientos-sop/config-and-drive-profiles.md` to point A6-EC drive/profile work at the new frame-semantics note.
  - Updated `.cursor/skills/gradientos-sop/commissioning-safety.md` to point A6-EC native-home reasoning at the new note and corrected the stale `607C` persistence wording so it matches the newer evidence.
  - Updated `.cursor/skills/gradientos-sop/validation-and-debugging.md` to point A6-EC debugging at `scripts/a6ec_chapter5_probe.py` plus the new note.
  - Updated `.cursor/memory/AGENT_SCRATCHPAD.md` with the documentation-pattern lesson from this pass.
- Validation:
  - Re-read `scripts/a6ec_chapter5_probe.py` header and reconstruction formulas before drafting the note.
  - Re-read the canonical-truth math area in `src/gradient_os/arm_controller/backends/ethercat_rtcore/backend.py` before documenting the equations.
  - Ran `ReadLints` on all edited Markdown files; no diagnostics were reported.
- Follow-up notes / risks:
  - The new doc is intentionally labeled as a durable WIP workstream note, not a final canonical SOP consolidation.
  - No unit tests or live hardware checks were run in this pass because the changes were documentation and skill-routing only.

## 2026-04-14 19:36 +0000

- Task summary:
  - Closed the remaining Python-side native-home wait-loop gap so a stale terminal result cannot fail a fresh home request before RTCore has advertised the new request as active.
- Changes:
  - Updated `src/gradient_os/arm_controller/backends/ethercat_rtcore/backend.py`:
    - `_wait_for_native_home_result()` now only trusts terminal `failed`/`succeeded` states after the target axis has been observed in `native_home_active_axis_mask`
    - if fresh metrics arrive but the target axis is never observed active, the wait now degrades to `pending` / `timed_out` with a zero top-level abort code instead of surfacing a stale previous failure
  - Updated `tests/test_gradient05_limits_and_backends.py` with two focused regressions for:
    - stale failed telemetry before the active epoch is seen
    - genuine failure after the active epoch is seen and clears
- Validation:
  - `source "/home/pi/GradientOS/.venv/bin/activate" && PYTHONPATH=src python -m py_compile "src/gradient_os/arm_controller/backends/ethercat_rtcore/backend.py" "tests/test_gradient05_limits_and_backends.py"`
  - `source "/home/pi/GradientOS/.venv/bin/activate" && PYTHONPATH=src python -m pytest tests/test_gradient05_limits_and_backends.py -q -k 'wait_for_native_home_result_waits_for_active_mask_clear_before_statusword_fallback or wait_for_native_home_result_ignores_stale_failed_report_before_active_mask_seen or wait_for_native_home_result_reports_failed_after_active_mask_seen or wait_for_native_home_result_accepts_stale_failed_report_after_active_mask_clears'`
    - result: `4 passed, 57 deselected`
  - `source "/home/pi/GradientOS/.venv/bin/activate" && PYTHONPATH=src python -m pytest tests/test_gradient05_limits_and_backends.py -q -k 'native_home_metrics_result or wait_for_native_home_result'`
    - result: `7 passed, 54 deselected`
  - `ReadLints` on `src/gradient_os/arm_controller/backends/ethercat_rtcore/backend.py` and `tests/test_gradient05_limits_and_backends.py`
    - result: no diagnostics
- Follow-up notes / risks:
  - This closes the Python-side stale-failure race only; I did not re-run live hardware native-home validation in this pass.
  - Telemetry/UI still intentionally expose the broader effective native-home semantics; this change only tightens when the backend accepts a terminal result for the current request.

## 2026-04-14 19:48 +0000

- Task summary:
  - Stabilized the disarmed runtime header safety badge so the frontend no longer flashes a transient green `SAFE` state while the drives are still inactive and sync readiness is still settling.
- Changes:
  - Updated `web-ui/src/ControlPanel.tsx`:
    - `ControlPanelRuntimeHeader` now treats a disarmed, sync-only unsettled readiness condition as neutral `CHECK` instead of `BLOCKED`
    - the green `SAFE` badge is now held behind a short disarmed-only stabilization window so transient `safe_for_power_transition=true` packets do not make the header look active
    - the runtime-header `Power Up` button now follows that same stabilized disarmed-ready signal, which is stricter than the raw backend bit and avoids enable-button flicker during sync jitter
  - Updated `web-ui/src/ControlPanel.test.tsx` with focused regressions for:
    - sync-only unsettled readiness showing `CHECK` while disarmed
    - delayed `SAFE` display after a stable safe signal while disarmed
- Validation:
  - `cd /home/pi/GradientOS/web-ui && npm test -- ControlPanel.test.tsx`
    - result: `15 passed`
  - `ReadLints` on `web-ui/src/ControlPanel.tsx` and `web-ui/src/ControlPanel.test.tsx`
    - result: no diagnostics
- Follow-up notes / risks:
  - This hardens the misleading header flicker and disarmed enable-button jitter only; it does not change the underlying backend reason the sync-readiness bit may still oscillate near the current roundtrip/jitter threshold.

## 2026-04-14 20:04 +0000

- Task summary:
  - Reproduced the unexpected startup failure from the launcher side and traced it to a stale-owner EtherCAT master reservation caused by a hung RTCore metrics thread, not by the recent frontend-only changes.
- Changes:
  - No code changes in this pass.
  - Collected live bring-up evidence from:
    - `./start-stack.sh probe`
    - `systemctl status ethercat.service gradient-rt-motion.service --no-pager`
    - `journalctl -u ethercat.service -u gradient-rt-motion.service ...`
    - `journalctl -k -n 80 --no-pager`
    - `./start-stack.sh stop --hard`
    - fresh launcher reproduction `./start-stack.sh` on run `20260414-200318`
- Validation:
  - Initial failed launcher run confirmed in terminal/logs: `20260414-195850`
  - Fresh reproduction run: `20260414-200318`
    - launcher again stalled at `responding=0/6 online=0/6 operational=0/6 startup_ready=0 wkc=0`
    - launcher exited with `bus failed readiness`
  - `gradient-rt-motion.service` repeatedly failed with:
    - `Failed to reserve master: Device or resource busy`
    - `ecrt_request_master(0) failed`
  - Kernel log showed the stuck owner directly:
    - `task metrics:42143 blocked for more than 120 seconds`
    - blocked in `ecrt_master_sdo_upload` / `ec_ioctl` in `ec_master`
  - Hard stop result:
    - `ethercat.service` stop failed with `rmmod: ERROR: Module ec_generic is in use`
    - `lsmod` still showed `ec_generic` and `ec_master` loaded
- Follow-up notes / risks:
  - The visible `42130` `gradient-rt-mot` process is a zombie marker; the real blocker is the hung `metrics` thread (`pid 42143`, `tgid 42130`) in uninterruptible kernel sleep.
  - Because the owner is stuck in kernel space and survives SIGKILL/systemd stop, the most likely recovery is a host reboot before any fresh launcher run can succeed.
  - This evidence points to a stale-owner/runtime-kernel hang, not to the recent `ControlPanel.tsx` UI patch.

## 2026-04-14 20:12 +0000

- Task summary:
  - Added first-failure fieldbus diagnostics and stopped RTCore from restart-looping on the specific EtherCAT master-reservation failure so stale-owner events are easier to identify and less noisy.
- Changes:
  - Updated `start-stack.sh`:
    - on bus-readiness timeout, the launcher now writes `fieldbus-failure-diagnostics/` inside the current run log directory
    - captured artifacts include `probe.json`, `systemd-status.txt`, `unit-journal.txt`, `kernel-journal.txt`, `processes.txt`, `kernel-modules.txt`, and `summary.txt`
    - the launcher now echoes the heuristic summary lines directly in the terminal/log on failure
  - Updated `src/gradient_rt_motion/main.cpp`:
    - `ecrt_request_master(0)` failure now exits with dedicated code `75`
    - the RTCore log now states that this path likely means a stale owner or hung EtherCAT kernel task
  - Updated `systemd/rt-motion/gradient-rt-motion.service`:
    - `Restart=on-failure`
    - `RestartPreventExitStatus=75`
    - result: systemd no longer immediately restart-loops RTCore on the master-busy path
- Validation:
  - `bash -n ./start-stack.sh`
  - `systemd-analyze verify /home/pi/GradientOS/systemd/rt-motion/gradient-rt-motion.service`
  - `make -C src/gradient_rt_motion`
  - reproduced failure with new diagnostics on run `20260414-201011`
    - launcher created `fieldbus-failure-diagnostics/`
    - `systemd-status.txt` captured RTCore exit `status=75`
  - re-ran after summary parser fix on run `20260414-201135`
    - launcher summary now explicitly reported:
      - `likely_cause=rtcore_master_reservation_failed`
      - `hung_kernel_task=metrics:42143`
      - `rtcore_zombie_marker_present=1`
      - `ethercat_modules_loaded=1`
    - `journalctl -u gradient-rt-motion.service -S "2026-04-14 20:11:35"` showed a single failure window with no follow-up scheduled restart loop
- Follow-up notes / risks:
  - This improves observability and reduces restart thrash, but it does not fix the underlying kernel-space hang once it has already happened.
  - Recent repo/devlog history still points at the Apr 12-13 RTCore metrics-thread SDO polling/readback additions as the leading "why this is new" correlation; that path likely needs the next hardening pass if the goal is recurrence reduction rather than better diagnosis.

## 2026-04-14 20:22 +0000

- Task summary:
  - Added per-feature RTCore metrics-thread isolation toggles so the next reboot can identify which SDO polling path is actually responsible for the new stale-owner wedge instead of relying on inference.
- Changes:
  - Updated `src/gradient_rt_motion/main.cpp`:
    - added `parse_env_flag(...)`
    - introduced independent env-controlled toggles for:
      - startup drive-config readback
      - native-home offset refresh
      - absolute-feedback polling
    - RTCore now logs the toggle states at startup
    - `metrics.json` now includes:
      - `metrics_startup_readback_enabled`
      - `metrics_native_home_refresh_enabled`
      - `metrics_absolute_feedback_poll_enabled`
  - Updated `systemd/rt-motion/gradient-rt-motion.service` with default envs:
    - `GRADIENT_RT_METRICS_STARTUP_READBACK_ENABLED=1`
    - `GRADIENT_RT_METRICS_NATIVE_HOME_REFRESH_ENABLED=1`
    - `GRADIENT_RT_METRICS_ABSOLUTE_FEEDBACK_POLL_ENABLED=1`
  - Updated `start-stack.sh` probe rendering to print the RTCore metrics-SDO toggle state in the human-readable hardware summary.
- Validation:
  - `bash -n ./start-stack.sh`
  - `systemd-analyze verify /home/pi/GradientOS/systemd/rt-motion/gradient-rt-motion.service`
  - `make -C src/gradient_rt_motion`
  - `ReadLints` on:
    - `src/gradient_rt_motion/main.cpp`
    - `start-stack.sh`
    - `systemd/rt-motion/gradient-rt-motion.service`
    - result: no diagnostics
- Follow-up notes / risks:
  - I could not live-validate the new toggle output on a healthy boot yet because the host is still in the existing stale-owner kernel-hang state; a reboot is still required before any clean bring-up experiment can succeed.
  - The intended next step is an evidence-first reboot matrix:
    - all toggles enabled
    - then disable only absolute-feedback polling
    - then disable native-home refresh
    - then disable startup readback
  - This gives us the first real A/B path to identify the culprit metrics-SDO behavior without assuming in advance that absolute-feedback polling is the only problem.

## 2026-04-14 20:59 +0000

- Task summary:
  - Ran the first live matrix trial after reboot, confirmed that the startup wedge reproduces with all metrics-thread SDO features enabled, and staged the next reboot to run with periodic absolute-feedback polling disabled.
- Changes:
  - No repo code changes in this pass.
  - Added a persistent systemd drop-in override:
    - `/etc/systemd/system/gradient-rt-motion.service.d/99-metrics-isolation.conf`
    - `Environment=GRADIENT_RT_METRICS_ABSOLUTE_FEEDBACK_POLL_ENABLED=0`
  - Reloaded systemd daemon after writing the drop-in.
- Validation:
  - Post-reboot pre-test health check:
    - `systemctl is-active ethercat.service gradient-rt-motion.service` -> both `active`
    - `./start-stack.sh probe` -> `ethercat_master_state=OP`, `rtcore_state=UP`, `physical_state=BUS_UP_DISARMED`
  - Baseline matrix trial:
    - `./start-stack.sh`
    - run id: `20260414-205728`
    - `RTCORE SYNC COMPLETE` took `42.272s`
    - launcher then failed with `bus failed readiness`
    - diagnostics summary: `likely_cause=rtcore_master_reservation_failed`
  - Post-failure inspection:
    - `./start-stack.sh probe` -> `ethercat_master_state=DOWN`, `rtcore_state=DOWN`
    - `systemctl status gradient-rt-motion.service ethercat.service --no-pager`
      - RTCore failed with exit `75`
      - leftover zombie marker `pid 1642`
      - metrics thread `pid 1769` survived SIGKILL during stop
- Follow-up notes / risks:
  - The baseline result means the regression is real and reproducible under the fully enabled metrics-thread SDO configuration.
  - The host is contaminated again after the baseline trial, so another reboot is required before the second matrix step can run.
  - The next trial is already staged to disable only periodic absolute-feedback polling, which is the highest-value first isolation because it is the only continuous metrics-thread SDO path.

## 2026-04-14 21:40 +0000

- Task summary:
  - Ran the second post-reboot matrix trial with only `GRADIENT_RT_METRICS_ABSOLUTE_FEEDBACK_POLL_ENABLED=0`, proved that the startup wedge no longer reproduces under that A/B condition, fixed the now-unmasked launcher preflight circular import, and reran the stack to a full healthy bring-up.
- Changes:
  - Added `src/gradient_os/telemetry/native_home_status.py` to hold the shared native-home status derivation helper.
  - Updated `src/gradient_os/telemetry/drive_faults.py` to import the shared helper instead of defining it inline.
  - Updated `src/gradient_os/arm_controller/backends/ethercat_rtcore/backend.py` to import the helper from `telemetry.native_home_status` instead of from `drive_faults.py`.
  - Added a focused circular-import regression in `tests/test_drive_faults.py` that imports both the drive-fault path and the EtherCAT backend in one Python process.
- Validation:
  - Isolation check before rerun:
    - `systemctl is-active ethercat.service gradient-rt-motion.service` -> both `active`
    - `/run/gradient-rt-motion/metrics.json` showed:
      - `metrics_startup_readback_enabled=1`
      - `metrics_native_home_refresh_enabled=1`
      - `metrics_absolute_feedback_poll_enabled=0`
  - First isolated launcher run:
    - `./start-stack.sh`
    - run id: `20260414-213541`
    - `RTCORE SYNC COMPLETE` in `1.006s`
    - `BUS READY` in `1.708s`
    - no stale-owner/master-reservation failure reproduced
    - launcher then failed later with `startup preflight could not build a fault-reset plan from the probe payload`
  - Root-cause reproduction for the new launcher failure:
    - direct Python import of `gradient_os.telemetry.drive_faults` reproduced:
      - `ImportError: cannot import name 'derive_effective_native_home_status' from partially initialized module 'gradient_os.telemetry.drive_faults'`
  - Focused code checks after the import fix:
    - `source "/home/pi/GradientOS/.venv/bin/activate" && PYTHONPATH=src python -m py_compile src/gradient_os/telemetry/native_home_status.py src/gradient_os/telemetry/drive_faults.py src/gradient_os/arm_controller/backends/ethercat_rtcore/backend.py tests/test_drive_faults.py`
    - `source "/home/pi/GradientOS/.venv/bin/activate" && PYTHONPATH=src python -m pytest tests/test_drive_faults.py -q` -> `9 passed`
    - `ReadLints` on touched Python files -> no diagnostics
  - Full rerun after the import fix:
    - `./start-stack.sh`
    - run id: `20260414-213805`
    - `RTCORE SYNC COMPLETE` in `992ms`
    - `BUS READY` in `1.964s`
    - startup preflight passed: `no disarmed drive faults detected`
    - controller, API, and web all came online
    - launcher reported `STACK BOOT COMPLETE in 19.775s`
    - follow-up `./start-stack.sh probe` showed:
      - `controller_udp: up`
      - `api_http: up`
      - `rtcore_state: UP`
      - `physical_state: BUS_UP_DISARMED`
      - all 6 axes `SwitchOnDisabled` with `err=0x0000`
- Follow-up notes / risks:
  - This is the strongest live evidence so far that periodic absolute-feedback polling is the leading trigger for the new startup wedge.
  - The result is still one A/B datapoint, not a full proof that the other two metrics-thread SDO features are innocent; additional reboot-cycle trials are still useful if we want to narrow the exact offending path further.
  - The supervised stack from run `20260414-213805` remains up at the end of this task.

## 2026-04-14 22:34 +0000

- Task summary:
  - Verified that a clean `./start-stack.sh stop --hard` is enough for iterative stop/start testing under the non-wedged isolation config; a reboot is not required between healthy cycles.
- Changes:
  - No repo code changes in this pass.
- Validation:
  - Starting point:
    - `./start-stack.sh probe` from run `20260414-213805` showed a healthy live stack:
      - `controller_udp: up`
      - `api_http: up`
      - `physical_state: BUS_UP_DISARMED`
      - `rtcore_state: UP`
      - `ethercat_master_state: OP`
  - Hard stop test:
    - `./start-stack.sh stop --hard`
    - launcher stopped
    - `gradient-rt-motion.service` stopped in `333ms`
    - `ethercat.service` stopped in `1.178s`
    - final stop probe reported:
      - `physical: INACTIVE`
      - `ethercat: DOWN`
      - `rtcore: DOWN`
  - Post-stop checks:
    - `systemctl is-active ethercat.service gradient-rt-motion.service` -> both `inactive`
    - `./start-stack.sh probe` -> `physical_state=INACTIVE`, `ethercat_master_state=DOWN`, `rtcore_state=DOWN`
  - Restart without reboot:
    - `./start-stack.sh`
    - run id: `20260414-223306`
    - startup recovery briefly classified `rtcore_up_master_down`
    - launcher performed one hard RTCore/EtherCAT recycle automatically
    - `BUS READY` then succeeded in `9.105s`
    - stack reached `STACK BOOT COMPLETE in 37.496s`
  - Post-restart health:
    - `./start-stack.sh probe` showed:
      - `controller_udp: up`
      - `api_http: up`
      - `physical_state: BUS_UP_DISARMED`
      - `rtcore_state: UP`
      - `ethercat_master_state: OP`
      - all six axes `SwitchOnDisabled` with `err=0x0000`
- Follow-up notes / risks:
  - Operationally, this means we can continue the metrics-thread bug hunt with `stop --hard` / start cycles as long as the host has not already entered the hung-kernel-thread stale-owner state.
  - The reboot requirement still applies once the bad configuration poisons the host, because that kernel-space stuck owner is not cleared by user-space stop commands.

## 2026-04-14 23:01 +0000

- Task summary:
  - Promoted the validated `stop --hard` versus reboot testing rule into the canonical GradientOS SOP skill under validation/debugging guidance.
- Changes:
  - Updated `.cursor/skills/gradientos-sop/validation-and-debugging.md`:
    - added `## Live Bring-Up Loops`
    - documented the healthy-cycle rule:
      - use `./start-stack.sh probe`
      - then `./start-stack.sh stop --hard`
      - then `./start-stack.sh`
    - documented the reboot threshold:
      - reboot is only required after stale-owner symptoms or failed EtherCAT ownership teardown
    - documented that one launcher-managed RTCore/EtherCAT recovery recycle is acceptable if startup still reaches `BUS_UP_DISARMED`
- Validation:
  - Read back the edited SOP file to confirm wording and placement.
  - `ReadLints` on `.cursor/skills/gradientos-sop/validation-and-debugging.md` -> no diagnostics.
- Follow-up notes / risks:
  - The canonical skill now reflects the proven live-testing workflow, but the exact offending metrics-thread SDO path still needs more isolation trials before that narrower conclusion should be promoted further.

## 2026-04-14 23:11 +0000

- Task summary:
  - Resumed the encoder-retention workstream with the planned no-motion all-joints consistency control before any new home or jog.
- Changes:
  - No repo code changes in this pass.
  - Captured new experiment `logs/encoder-retention/20260414-230845-all-joints-stationary-consistency/` with:
    - `stationary-1.json/.md`
    - `stationary-2.json/.md`
    - `stationary-3.json/.md`
    - `info-joints-detailed-current.json`
    - `metrics-current.json`
    - `anchors-current.json`
- Validation:
  - Pre-check:
    - `./start-stack.sh probe` showed a healthy stack in `BUS_UP_DISARMED` with all six axes clean.
  - Capture commands:
    - `scripts/a6ec_chapter5_probe.py snapshot --label stationary-1 --axes J1 J2 J3 J4 J5 J6 --experiment-id 20260414-230845-all-joints-stationary-consistency`
    - repeated for `stationary-2` and `stationary-3` with short waits
    - `curl -sf http://127.0.0.1:4000/info/joints-detailed`
    - copied `/run/gradient-rt-motion/metrics.json`
    - copied `.gradient_absolute_encoder_anchors.json`
  - Comparison against retained baselines:
    - `J2/J3/J4/J6` remained in the same overall raw/reference/rotation family with no large semantic shift
    - `J1/J5` showed larger-than-jitter but still small absolute deltas versus older immediate-home baselines (`J1 raw +6`, `J5 raw +17` counts), not whole-turn or thousand-count changes
    - all anchor values matched the current anchor file exactly; no unexpected anchor mutation occurred
    - current stationary spans inside the new experiment stayed small:
      - raw absolute `U40.20` spans `0..3`
      - reference-family spans stayed in the low single digits, worst observed `J4 6064 span=4`
  - API constraint discovered during this run:
    - `info-joints-detailed-current.json` reported `canonical_joint_truth_available=false`
    - all joints showed `truth_reason=absolute_feedback_unavailable`
    - `metrics-current.json` confirmed the safe isolation is still active: `metrics_absolute_feedback_poll_enabled=0`
- Follow-up notes / risks:
  - The no-motion control passed on direct SDO frame/anchor evidence strongly enough to proceed to the next planned step: a drive-only power-cycle control with the same all-joints capture set.
  - The probe's current one-count match booleans over-flagged normal live-read wander again (`2..4` count deltas), so those booleans should not be treated as semantic failures by themselves.
  - API truth/roundtrip criteria are currently blind by design under the startup-wedge isolation config; if that part of the old checklist is required again, we will have to deliberately re-enable the risky metrics absolute-feedback polling path and accept the possibility of re-poisoning the host.

## 2026-04-14 23:25 +0000

- Task summary:
  - Replaced the probe's pure one-count drift reporting with explicit drift-magnitude categories while keeping backward-compatible one-count booleans.
- Changes:
  - Updated `scripts/a6ec_chapter5_probe.py`:
    - added drift bucket thresholds:
      - `standard <= 2`
      - `medium <= 6`
      - `large <= 10`
      - `excessive <= 100`
      - `extreme > 100`
    - added `_classify_count_delta(...)`
    - added `_delta_summary(...)`
    - added category and absolute-magnitude fields for the existing bridge deltas
    - updated markdown and condensed console output to include the new categories
  - Added `tests/test_a6ec_chapter5_probe.py` to lock the bucket boundaries and rendered category output.
- Validation:
  - `source "/home/pi/GradientOS/.venv/bin/activate" && PYTHONPATH=src python -m py_compile scripts/a6ec_chapter5_probe.py tests/test_a6ec_chapter5_probe.py`
  - `source "/home/pi/GradientOS/.venv/bin/activate" && PYTHONPATH=src python -m pytest tests/test_a6ec_chapter5_probe.py -q` -> `3 passed`
  - `ReadLints` on the touched files -> no diagnostics
  - Live proof:
    - captured `stationary-4-classified` under experiment `20260414-230845-all-joints-stationary-consistency`
    - confirmed the new condensed output now reports categories such as:
      - `standard` for `0..2`
      - `medium` for a live `-3` count bridge delta on `J2`
- Follow-up notes / risks:
  - This improves interpretation, but it does not by itself change the actual persistence decision rule: isolated `medium` drift is still descriptive, not proof of a semantic frame change.

## 2026-04-14 23:46 +0000

- Task summary:
  - Completed the planned drive-only power-cycle control and compared the post-cycle all-joints stationary captures against the pre-cycle control and retained baselines.
- Changes:
  - No repo code changes in this pass.
  - Brought the stack back up after the user hard stop + drive power cycle:
    - run id: `20260414-234249`
    - startup used one launcher-managed `rtcore_up_master_down` recycle
    - then reached `STACK BOOT COMPLETE`
  - Added post-cycle artifacts under `logs/encoder-retention/20260414-230845-all-joints-stationary-consistency/`:
    - `post-power-cycle-1.json/.md`
    - `post-power-cycle-2.json/.md`
    - `post-power-cycle-3.json/.md`
    - `info-joints-detailed-post-power-cycle.json`
    - `metrics-post-power-cycle.json`
    - `anchors-post-power-cycle.json`
- Validation:
  - Pre-start verification:
    - `systemctl is-active ethercat.service gradient-rt-motion.service` -> both `inactive`
    - `./start-stack.sh probe` -> `physical_state=INACTIVE`, `rtcore_state=DOWN`, `ethercat_master_state=DOWN`
  - Post-start verification:
    - launcher reported `STACK BOOT COMPLETE in 37.179s`
    - bus landed `READY and DISARMED`
  - Post-cycle capture commands:
    - `scripts/a6ec_chapter5_probe.py snapshot --label post-power-cycle-1 --axes J1 J2 J3 J4 J5 J6 --experiment-id 20260414-230845-all-joints-stationary-consistency`
    - repeated for `post-power-cycle-2` and `post-power-cycle-3` with short waits
    - `curl -sf http://127.0.0.1:4000/info/joints-detailed`
    - copied `/run/gradient-rt-motion/metrics.json`
    - copied `.gradient_absolute_encoder_anchors.json`
  - Post-cycle direct evidence:
    - raw absolute `U40.20` spans stayed `0..3` counts across all joints
    - reference-family spans stayed in the low single digits
    - only a few `medium` bucket deltas appeared, all at `3` counts
    - no `large`, `excessive`, or `extreme` post-cycle deltas appeared
  - Pre-vs-post latest comparison:
    - `J1 raw delta = 0`
    - `J2 raw delta = -1`
    - `J3 raw delta = 0`
    - `J4 raw delta = -2`
    - `J5 raw delta = 0`
    - `J6 raw delta = -2`
    - all post-cycle anchor values matched the pre-cycle anchors exactly
  - `J2` focused note:
    - raw absolute `-1` count across the power cycle
    - `6064 +2`, `6063 -1`, `6062 -2`, `60FC -1`, `U40.28 +1`
    - anchor unchanged at `0.02350346188438531`
  - API limitation remains:
    - `info-joints-detailed-post-power-cycle.json` still reported `canonical_joint_truth_available=false`
    - all joints still showed `truth_reason=absolute_feedback_unavailable`
    - `metrics-post-power-cycle.json` confirmed `metrics_absolute_feedback_poll_enabled=0`
- Follow-up notes / risks:
  - This drive-only power-cycle control passed on direct SDO and anchor evidence for all six joints.
  - `J2` did not reproduce the earlier fragile behavior across this control; current evidence says it is power-cycle-stable under the present configuration.
  - Older baseline mismatches that already existed pre-cycle (especially weaker `J1/J5` comparisons and wrap-adjacent `J6` reference fields) should not be misattributed to the power cycle itself, because the pre-vs-post control stayed tight.
  - If we need API truth and roundtrip evidence again, we will have to intentionally re-enable the risky absolute-feedback metrics polling path and accept the chance of reintroducing the startup wedge.

## 2026-04-15 00:13 +0000

- What changed:
  - Hardened `src/gradient_rt_motion/main.cpp` by serializing helper/metrics SDO uploads and downloads against `ecrt_release_master()`.
  - Rebuilt `src/gradient_rt_motion/gradient-rt-motion`, reinstalled `/usr/local/bin/gradient-rt-motion`, and restored the live systemd override so `GRADIENT_RT_METRICS_ABSOLUTE_FEEDBACK_POLL_ENABLED=1`.
  - Updated `src/gradient_os/arm_controller/backends/ethercat_rtcore/backend.py` so command-frame roundtrip acceptance uses a `3`-count tolerance while stale-anchor diagnosis stays at `8` counts.
  - Added a new tolerance regression and tightened the existing native-home sequencing test in `tests/test_gradient05_limits_and_backends.py`.
- Validation:
  - `make` in `src/gradient_rt_motion`
  - `pytest tests/test_gradient05_limits_and_backends.py -k "roundtrip or stale_absolute_home_anchor or native_home"` -> `18 passed`
  - Live restart validation with full metrics enabled:
    - `./start-stack.sh stop --hard` -> clean `physical_state=INACTIVE`, `rtcore_state=DOWN`, `ethercat_master_state=DOWN`
    - `./start-stack.sh` -> `STACK BOOT COMPLETE` on run `20260415-000137`
    - repeated `./start-stack.sh stop --hard` -> `./start-stack.sh` -> `STACK BOOT COMPLETE` on run `20260415-000821`
  - Live `J2` native-home revalidation:
    - experiment `20260415-000821-j2-native-home-revalidation`
    - `pre-home`, `post-home-immediate`, `post-home-settle`, and `post-home-fault` probe snapshots captured
    - sidecars saved: `info-joints-detailed-*`, `metrics-*`, `anchors-*`
    - API command `POST /control/home-joint-native {"joint":2}` returned `NATIVE_HOME_VERIFIED` with anchor capture/refresh success and updated `J2` anchor `0.023517842954271735`
- Follow-up notes / risks:
  - The startup wedge with full metrics did not reproduce after the RTCore SDO/release fence; this looks like a meaningful live fix, not just another isolation.
  - `J2` native-home is still not end-to-end clean: a few seconds after the verified return, `./start-stack.sh probe` showed `J2` faulted with `err=0xff00` / `Er11.0 | Excessive motor speed upon servo drive power-on`, while `native_home_state` remained `2`.
  - The backend’s current native-home success contract is therefore still too optimistic for A6-EC: we now need a post-home fault-free settle check, not just verified terminal state plus anchor refresh.
  - After the tolerance patch, `J2` itself no longer causes the live command-roundtrip false negative; the current global truth-unavailable flapping is coming from `J1` at roughly `4` counts.

## 2026-04-15 00:32 +0000

- What changed:
  - Updated `src/gradient_os/arm_controller/backends/ethercat_rtcore/backend.py` so `native_home_joint()` no longer returns `NATIVE_HOME_VERIFIED` immediately after anchor refresh. It now also waits through a short post-home settle window and downgrades the result if the target axis faults, drops offline, or otherwise fails to stay clean.
  - Added new backend result paths for post-home settle failures and pending settle verification:
    - `NATIVE_HOME_POST_HOME_SETTLE_FAILED`
    - `NATIVE_HOME_POST_HOME_SETTLE_PENDING`
  - Added backend regressions in `tests/test_gradient05_limits_and_backends.py` for:
    - verified success including the settle step
    - settle downgrade when the target axis reports a hard post-home fault (`drive_faulted`, modeled on the live `0xff00` case)
    - direct settle helper coverage for both clean and faulted axis snapshots
  - Updated `web-ui/src/ControlPanel.tsx` so short `/info/joints-detailed` dropouts no longer immediately clear the displayed joint values; the UI now holds the last good live joint values briefly while backend truth is flapping.
  - Added `web-ui/src/ControlPanel.test.tsx` coverage for the transient joint-feedback hold behavior.
- Validation:
  - Live diagnostic sampling before any restart:
    - `curl -sf http://127.0.0.1:4000/info/joints-detailed`
    - `./start-stack.sh probe`
    - repeated 20-sample `/info/joints-detailed` loop
  - Findings from the 20-sample loop:
    - `canonical_joint_truth_available` still flaps on the live stack
    - the flapping is not just `J1`; sampled dropouts implicated `J1`, `J4`, `J5`, and `J6`
    - current UI disappearing values are therefore backend truth flapping plus frontend blank-on-miss behavior, not a literal UI transport disconnect
  - `pytest tests/test_gradient05_limits_and_backends.py -k "native_home or post_settle or roundtrip or stale_absolute_home_anchor"` -> `21 passed`
  - `cd web-ui && npx vitest run src/ControlPanel.test.tsx` -> `16 passed`
- Follow-up notes / risks:
  - The backend hardening is not yet live-loaded into the currently running controller process; a controller/stack restart is still required before the next real `J2` native-home retest.
  - The new UI hold reduces visible flicker, but it does not solve the underlying truth-flap cause. The current live command-roundtrip threshold edge is still being crossed by multiple axes, not only `J1`.

## 2026-04-15 00:39 +0000

- What changed:
  - Reverted the temporary 1.5 s joint-feedback hold in `web-ui/src/ControlPanel.tsx` so the frontend returns to the original immediate-clear behavior on `/info/joints-detailed` misses.
  - Updated `src/gradient_os/arm_controller/backends/ethercat_rtcore/backend.py` so `_COMMAND_ROUNDTRIP_TOLERANCE_COUNTS = 6.0` instead of `3.0`.
  - Updated `tests/test_gradient05_limits_and_backends.py` to accept `6` counts of stationary roundtrip wander and still reject `7` counts.
  - Updated `web-ui/src/ControlPanel.test.tsx` to match the restored immediate-clear frontend behavior.
- Validation:
  - Captured a 90 s stationary diagnostic run:
    - raw log: `logs/joint-truth-jitter/20260415-003401-joints-detailed-90s.jsonl`
    - summary: `logs/joint-truth-jitter/20260415-003401-joints-detailed-90s.summary.json`
  - Key measured absolute roundtrip-error ranges from that run:
    - `J1`: max `5`, p95 `4`
    - `J2`: max `3`, p95 `2`
    - `J3`: max `3`, p95 `2`
    - `J4`: max `5`, p95 `3`, p99 `4`
    - `J5`: max `6`, p95 `4`, p99 `5`
    - `J6`: max `6`, p95 `5`, p99 `6`
  - Threshold replay against the 90 s log:
    - values exceeding `3` counts: `249`
    - values exceeding `4` counts: `63`
    - values exceeding `5` counts: `9`
    - values exceeding `6` counts: `0`
  - `pytest tests/test_gradient05_limits_and_backends.py -k "roundtrip or native_home or stale_absolute_home_anchor or post_settle"` -> `22 passed`
  - `cd web-ui && npx vitest run src/ControlPanel.test.tsx` -> `16 passed`
- Follow-up notes / risks:
  - The new `6`-count threshold is measurement-backed for the current stationary live state and should remove the observed false truth dropouts, but it is not yet live-loaded into the running controller process.
  - The next meaningful live step is still a restart into this build, then re-check `/info/joints-detailed` stability before the next `J2` native-home retest.

## 2026-04-15 00:43 +0000

- What changed:
  - Performed live post-restart validation after the user soft-stopped and restarted the stack with the new backend build loaded.
- Validation:
  - `./start-stack.sh probe` after restart:
    - `physical_state=BUS_UP_DISARMED`
    - all six axes `SwitchOnDisabled`
    - `J2` no longer faulted
  - Fresh `curl -sf http://127.0.0.1:4000/info/joints-detailed` sample showed:
    - `canonical_joint_truth_available=true`
    - all six axes truth-available
    - larger `command_roundtrip_tolerance_rad` values consistent with the live-loaded `6`-count threshold
  - 20-sample post-restart loop:
    - `canonical_joint_truth_available=true` on all 20 samples
    - high-but-accepted outliers still appeared on `J4/J5/J6`, including `J6 ~= -6` counts
  - 60 s post-restart soak:
    - `300/300` reads succeeded
    - `14/300` samples still went `canonical_joint_truth_available=false`
    - all 14 failures were `command_frame_roundtrip_mismatch`
    - max absolute roundtrip error by axis:
      - `J1`: `4`
      - `J2`: `3`
      - `J3`: `3`
      - `J4`: `5`
      - `J5`: `6`
      - `J6`: `9`
  - Follow-up 20 s axis breakdown:
    - remaining false samples were concentrated on `J6`
    - directly observed a failing `J6` sample at about `-7` counts
- Follow-up notes / risks:
  - The new `6`-count tolerance materially improved truth stability after restart, but it did not eliminate flapping completely.
  - The remaining live truth dropouts now look `J6`-dominated rather than broadly multi-axis. The next tuning decision may need to be axis-specific, or otherwise specifically justified around `J6`, instead of another blanket global threshold increase.
  - The user-pasted terminal flapping is therefore still relevant after restart; the `No UDP commands received` warnings are separate controller-idle link warnings, not encoder-jitter signals.

## 2026-04-15 00:52 +0000

- What changed:
  - Updated `src/gradient_os/arm_controller/backends/ethercat_rtcore/backend.py` so `_COMMAND_ROUNDTRIP_TOLERANCE_COUNTS = 10.0` per user request.
  - Updated `tests/test_gradient05_limits_and_backends.py` so the roundtrip regressions now accept `10` counts and reject `11`.
- Validation:
  - `pytest tests/test_gradient05_limits_and_backends.py -k "roundtrip or native_home or stale_absolute_home_anchor or post_settle"` -> `22 passed`
  - Investigated the current on-screen joint values against live API and persisted calibration state:
    - current live `/info/joints-detailed` sample matched the UI:
      - `J3 ~= -3.599°`
      - `J4 ~= -19.996°`
    - `.gradient_joint_zero_offsets.json` still stores `0.0` for all six software-zero offsets
    - `.gradient_absolute_encoder_anchors.json` contains persisted absolute-home anchors for all six joints
    - backend canonical truth still computes `absolute_axis_q - home_anchor_rad - software_zero`
- Follow-up notes / risks:
  - The `10`-count tolerance is implemented and tested, but it is not yet live-loaded into the running controller process; another restart is required before we can verify whether the remaining `J6`-driven truth flapping is gone.
  - The current `J3/J4` nonzero readouts do not appear to be a UI bug. Under the present calibration contract, they are semantically real because the stored software-zero offsets are all zero. If the intended parked pose should display `0.00` for every joint, that is a calibration/zero-contract issue, not a rendering issue.

## 2026-04-15 01:35 +0000

- What changed:
  - Investigated the user's challenge against the retained evidence directly instead of relying on the earlier summary.
  - Compared the current live `/info/joints-detailed` joint angles to yesterday's retained stationary and post-power-cycle snapshots in:
    - `logs/encoder-retention/20260414-230845-all-joints-stationary-consistency/stationary-3.json`
    - `logs/encoder-retention/20260414-230845-all-joints-stationary-consistency/post-power-cycle-3.json`
- Validation:
  - Current live sample:
    - `J3 = -3.5990936279296877°`
    - `J4 = -19.996185302734375°`
  - Retained baseline comparison:
    - vs `stationary-3`: `J3 delta = 0.0°`, `J4 delta = +0.000305°`
    - vs `post-power-cycle-3`: `J3 delta = +0.000027°`, `J4 delta = -0.000153°`
  - Also confirmed the other joints remain near the same parked values:
    - current live `J1/J2/J5/J6 = -0.0005768°, +0.0039001°, -0.0051855°, -0.0016479°`
    - all remain in the same tiny neighborhood as yesterday's retained snapshots
- Follow-up notes / risks:
  - This confirms the current `J3/J4` readouts are consistent with yesterday's persisted state; there is no evidence in the retained artifacts that those joints newly drifted or that the UI invented these values.
  - If the intended parked pose should display `0.00` on `J3/J4`, the remaining issue is calibration semantics (`home`/`zero` contract), not persistence failure.

## 2026-04-15 01:41 +0000

- What changed:
  - Re-checked the retained native-home evidence to answer whether "we homed all joints at their current position, therefore all joints should read zero."
  - Confirmed the persisted calibration split:
    - `.gradient_absolute_encoder_anchors.json` contains native-home anchors for all six joints
    - `.gradient_joint_zero_offsets.json` still contains `0.0` software-zero offsets for all six joints
  - Reconfirmed from backend code that native-home anchor capture and software zero are separate operations.
- Validation:
  - Backend contract:
    - canonical truth = `absolute_axis_q - home_anchor_rad - software_zero`
    - `set_logical_joint_current_position_as_zero()` writes `software_zero` / `_master_offsets_rad`
  - Retained post-home artifacts show:
    - `J1` post-home `canonical_rad ~= -7.67e-06`
    - `J2` clean post-home `canonical_rad ~= +1.29e-05`
    - `J5` post-home `canonical_rad ~= -7.06e-05`
    - `J6` post-home `canonical_rad ~= -4.79e-06`
    - `J3` post-home `canonical_rad ~= -0.062815`
    - `J4` persisted post-home `canonical_rad ~= -0.349013`
  - `J3` post-home truth poll also showed `reference_pre_zero_rad` remaining around `-0.0628`, while `J4` in the same poll remained around `-0.3490`, so those axes were nonzero immediately after the retained successful home state.
- Follow-up notes / risks:
  - The retained evidence says native-home is not equivalent to "set current pose to displayed zero" on all axes in the present A6-EC contract.
  - If the intended commissioning contract is "all joints should read `0.00` at this parked pose after home," then that contract is specifically wrong for `J3/J4`; the current values are not a false UI/backend rendering bug.

## 2026-04-15 05:26 +0000

- What changed:
  - Added a focused `J3` wrap-seam regression to `tests/test_gradient05_limits_and_backends.py` without changing backend/runtime behavior yet.
  - Added `test_ethercat_backend_normalizes_j3_style_wrapped_feedback_counts_for_display` to prove the backend already normalizes `131039 -> -33` counts for A6-EC display-style feedback.
  - Added strict `xfail` regression `test_ethercat_backend_j3_style_native_home_capture_should_zero_pose_at_wrap_seam` to encode the desired product behavior: after capturing native-home at the current `J3` seam-wrapped pose, the displayed/operator pose should be near zero and the captured home anchor should match the absolute pose seen at home.
  - Updated two stale nearby tests so they align with the current fail-closed anchor/roundtrip contract instead of expecting canonical/display truth to ignore frame mismatch:
    - `test_ethercat_backend_marks_truth_unavailable_across_raw_wrap_without_coherent_anchor`
    - `test_ethercat_backend_refuses_display_feedback_when_absolute_anchor_does_not_roundtrip`
- Validation:
  - Focused slice:
    - `python -m pytest tests/test_gradient05_limits_and_backends.py -q -k "normalizes_j3_style_wrapped_feedback_counts_for_display or j3_style_native_home_capture_should_zero_pose_at_wrap_seam or uses_multi_turn_absolute_feedback_as_canonical_truth or marks_truth_unavailable_across_raw_wrap_without_coherent_anchor or translates_canonical_truth_back_into_raw_wire_counts"` -> `4 passed, 1 xfailed`
  - Broader backend subset:
    - `python -m pytest tests/test_gradient05_limits_and_backends.py -q -k "canonical or absolute_home_anchor or native_home or display_feedback"` -> `25 passed, 1 xfailed`
  - `ReadLints` on `tests/test_gradient05_limits_and_backends.py` returned clean.
- Follow-up notes / risks:
  - The new `xfail` captures the currently reproduced product bug without changing live motion semantics yet.
  - The test evidence supports the narrower diagnosis that the backend already knows how to normalize seam-wrapped raw feedback for display, but the native-home anchor/reference capture path still encodes the wrong zero contract for `J3`-style wrap cases.
  - The final implementation should be driven by the new regression, but changing controller canonical truth still requires care because the motion/command path presently inverts through the same reference-frame assumptions.

## 2026-04-15 06:00 +0000

- What changed:
  - Updated `src/gradient_os/arm_controller/backends/ethercat_rtcore/backend.py` so the explicit operator display path can use `reference_mode="display"` while controller canonical truth remains on the raw controller/reference frame.
  - Switched native-home and software-zero absolute-home anchor capture/validation onto that display reference mode, so new `J3`-style seam homes collapse to display zero instead of preserving the wrapped `0x6064` offset.
  - Updated `src/gradient_os/run_controller.py` so `arm_display_rad` / `arm_display_deg` are only populated from the backend display snapshot, not copied from `arm_rad` / `arm_deg` by default.
  - Updated `web-ui/src/ControlPanel.tsx` so operator joint feedback uses only explicit display values (`arm_display_deg` / `display_joints`) with no canonical fallback.
  - Updated `tests/test_gradient05_limits_and_backends.py` to turn the `J3` seam-home regression into a required pass and align old-anchor display expectations with the new fail-closed display contract.
  - Updated `web-ui/src/ControlPanel.test.tsx` to assert the no-fallback display behavior and refreshed the pending-native-home fixture to provide display joints.
- Validation:
  - `source "/home/pi/GradientOS/.venv/bin/activate" && PYTHONPATH=src python -m py_compile src/gradient_os/arm_controller/backends/ethercat_rtcore/backend.py src/gradient_os/run_controller.py`
  - `source "/home/pi/GradientOS/.venv/bin/activate" && PYTHONPATH=src python -m pytest tests/test_gradient05_limits_and_backends.py -q -k "j3_style_native_home_capture_should_zero_pose_at_wrap_seam or normalizes_j3_style_wrapped_feedback_counts_for_display or uses_multi_turn_absolute_feedback_as_canonical_truth or marks_truth_unavailable_across_raw_wrap_without_coherent_anchor or translates_canonical_truth_back_into_raw_wire_counts or display_feedback or native_home_captures_absolute_encoder_anchor"` -> `7 passed, 61 deselected`
  - `source "/home/pi/GradientOS/.venv/bin/activate" && PYTHONPATH=src python -m pytest tests/test_run_controller_helpers.py -q` -> `5 passed`
  - `cd web-ui && npx vitest run src/ControlPanel.test.tsx` -> `17 passed`
  - `ReadLints` on touched files returned clean
- Follow-up notes / risks:
  - This intentionally does not change controller canonical / command-path semantics yet; raw `0x607A` wire-target wrap mapping still needs a deliberate design if you later want the controller-facing canonical path to match the same seam-normalized operator contract.
  - Existing anchors captured under the older/raw-style contract may now leave the explicit display path unavailable until those joints are re-homed or otherwise recaptured under the new display reference mode.

## 2026-04-15 06:33 +0000

- What changed:
  - Investigated the live blank-commissioning-pane regression with the running stack and confirmed it is not just a frontend transport miss.
  - Updated `src/gradient_os/arm_controller/backends/ethercat_rtcore/backend.py` to return `joint_positions_rad_partial` so operator display consumers can distinguish explicit per-joint truth from cached all-joint setpoints, and switched `_bootstrap_missing_absolute_home_anchors()` to capture missing anchors with `reference_mode="display"`.
  - Updated `src/gradient_os/run_controller.py` so `arm_display_rad` / `arm_display_deg` can publish partial explicit display truth while preserving the existing fail-closed canonical/raw semantics and status flags.
  - Updated `web-ui/src/ControlPanel.tsx` so partial display payloads render as per-joint values plus `--` for unavailable joints without falling back to canonical joint angles or external telemetry.
  - Added/updated regressions in `tests/test_gradient05_limits_and_backends.py`, `tests/test_run_controller_helpers.py`, and `web-ui/src/ControlPanel.test.tsx` for partial display publishing and display-mode startup bootstrap.
- Validation:
  - Live diagnostic:
    - `curl -sf http://127.0.0.1:4000/info/joints-detailed`
    - Confirmed `arm_rad=[]`, `arm_display_deg=[]`, and display-axis failures on `J3/J4` with approximately one-turn anchor deltas consistent with legacy/raw-style anchors under the new display contract.
  - `source "/home/pi/GradientOS/.venv/bin/activate" && PYTHONPATH=src python -m py_compile src/gradient_os/arm_controller/backends/ethercat_rtcore/backend.py src/gradient_os/run_controller.py`
  - `source "/home/pi/GradientOS/.venv/bin/activate" && PYTHONPATH=src python -m pytest tests/test_run_controller_helpers.py -q` -> `6 passed`
  - `source "/home/pi/GradientOS/.venv/bin/activate" && PYTHONPATH=src python -m pytest tests/test_gradient05_limits_and_backends.py -q -k "j3_style_native_home_capture_should_zero_pose_at_wrap_seam or normalizes_j3_style_wrapped_feedback_counts_for_display or refuses_display_feedback_when_absolute_anchor_does_not_roundtrip or startup_bootstrap"` -> `6 passed`
  - `cd web-ui && npx vitest run src/ControlPanel.test.tsx` -> `18 passed`
  - `ReadLints` on touched files returned clean.
- Follow-up notes / risks:
  - The running controller/UI processes have not been restarted by this change set, so the live stack will continue to show the old behavior until the user intentionally reloads it.
  - Existing `J3/J4` entries in `.gradient_absolute_encoder_anchors.json` still appear to predate the display-reference contract and should be deliberately re-homed or recaptured before expecting those joints to publish display truth.
  - Raw canonical truth and RTCore `0x607A` wrap safety remain intentionally unchanged; do not broaden the seam-normalized contract onto the motion command path until the raw-wire target redesign is explicit.

## 2026-04-15 06:44 +0000

- What changed:
  - Re-read the active A6-EC scratchpad/devlog history and re-ran live diagnostics after the user's hard stop/restart.
  - Ran the read-only Chapter 5 probe across all six joints with:
    - `source "/home/pi/GradientOS/.venv/bin/activate" && PYTHONPATH=src python scripts/a6ec_chapter5_probe.py snapshot --label hard-restart-all-joints --axes J1 J2 J3 J4 J5 J6`
  - Used an offline backend reconstruction against the same live raw counts to separate raw-frame failures from display-frame failures.
- Validation:
  - Live API after restart:
    - `curl -sf http://127.0.0.1:4000/info/joints-detailed`
    - Result: `arm_display_deg` had values for `J1/J2/J5/J6`, `J3/J4` were `null`, but `read_source` was still `unavailable` because the raw canonical path was still failing.
  - Probe artifact:
    - JSON: `logs/encoder-retention/20260415-064241-a6ec-ch5-probe/hard-restart-all-joints.json`
    - Markdown: `logs/encoder-retention/20260415-064241-a6ec-ch5-probe/hard-restart-all-joints.md`
  - Probe summary:
    - all six axes returned Chapter 5 SDO reads successfully (`failed_reads=[]` for `J1..J6`)
    - all bridge deltas stayed in the normal `0..2` count wander band
  - Offline backend truth split using the same live raw counts:
    - raw mode unavailable joints: `[4, 6]`
    - display mode unavailable joints: `[3, 4]`
    - `J3`: display-mode anchor stale by about `+131072` counts
    - `J4`: raw/display mismatch by about `+131068` counts
    - `J6`: raw-mode mismatch by about `-131076` counts, display mode coherent
- Follow-up notes / risks:
  - This confirms the underlying problem is not “cannot read the motors.” The drive-side objects are readable; the host-side truth contract is what is failing.
  - `J3` is the old/raw-style-anchor-against-display-contract case.
  - `J4` appears to have a genuinely wrong/stale stored anchor in both contracts.
  - `J6` is the raw `6064` wrap-seam issue that remains intentionally unsolved on the command/canonical path for safety reasons.
  - The commissioning UI can still blank all joints whenever it keys off `read_source=unavailable`, even if explicit display truth is present for a subset of joints.

## 2026-04-15 06:54 +0000

- What changed:
  - Investigated the live commissioning-pane mismatch as a browser/runtime issue, not a deploy/git issue, after the user pointed out that the backend was already returning partial `arm_display_deg`.
  - Used browser network tooling to confirm the page was the local Vite UI at `http://127.0.0.1:8000` and that it was polling `http://127.0.0.1:4000/info/joints-detailed` with `200` responses.
  - Identified the actual UI bug in `web-ui/src/ControlPanel.tsx`: `refreshJointAngles()` was still clearing all displayed joints whenever `read_source !== "live_feedback"`, even when explicit operator display truth was present in `arm_display_deg`.
  - Removed that stale `read_source` gate so the commissioning pane now renders explicit display truth per joint and keeps the no-canonical-fallback contract.
  - Updated `web-ui/src/ControlPanel.test.tsx` so the frontend regression coverage now uses the real live shape: `read_source="unavailable"` with valid full or partial `arm_display_deg`.
- Validation:
  - `cd web-ui && npx vitest run src/ControlPanel.test.tsx` -> `18 passed`
  - `ReadLints` on `web-ui/src/ControlPanel.tsx` and `web-ui/src/ControlPanel.test.tsx` returned clean.
- Follow-up notes / risks:
  - I could prove the browser was hitting the correct host and endpoint, but the browser MCP became unstable after the first successful inspection and would no longer keep the tab open for a second end-to-end visual check.
  - `J3/J4` remain a backend anchor problem, not a UI problem: the latest live payload still reports those display joints unavailable while `J1/J2/J5/J6` are valid.
  - There is still no dedicated API to clear an absolute-home anchor entry directly. Today the safe options are either:
    - overwrite it by running drive-native home for the affected joint, which captures a fresh display-mode anchor after verification
    - or clear the relevant entries in `.gradient_absolute_encoder_anchors.json` and restart/reload before recapturing them deliberately
  - The live monitor/SSE parse path in `web-ui/src/App.tsx` still filters non-finite `display_joints` values, so if monitor packets start carrying `null` placeholders that path may need a follow-up fix to preserve slot alignment.

## 2026-04-15 07:00 +0000

- What changed:
  - Re-checked the live backend truth after the UI fix and confirmed the persisted anchor file was still stale for both `J3` and `J4`.
  - Ran live drive-native home recapture for `J3` via `POST /control/home-joint-native {"joint": 3}`.
  - Verified that `J3` immediately regained display truth and that `.gradient_absolute_encoder_anchors.json` updated the `J3` anchor timestamp to `2026-04-15T06:59:50+00:00`.
  - Ran live drive-native home recapture for `J4` via `POST /control/home-joint-native {"joint": 4}`.
  - Verified that `J4` immediately regained display truth and that `.gradient_absolute_encoder_anchors.json` updated the `J4` anchor timestamp to `2026-04-15T07:00:10+00:00`.
- Validation:
  - Live `info/joints-detailed` before action still showed:
    - `J3 truth_reason=absolute_home_anchor_stale` with `absolute_home_anchor_delta_counts=131072`
    - `J4 truth_reason=command_frame_roundtrip_mismatch` with `absolute_home_anchor_delta_counts=131069`
  - `POST /control/home-joint-native` for `J3` returned:
    - `accepted=true`
    - `verified=true`
    - `absolute_home_anchor_capture_succeeded=true`
    - `absolute_home_anchor_refresh_ok=true`
    - `post_home_truth_available=true`
  - `POST /control/home-joint-native` for `J4` returned:
    - `accepted=true`
    - `verified=true`
    - `absolute_home_anchor_capture_succeeded=true`
    - `absolute_home_anchor_refresh_ok=true`
    - `post_home_truth_available=true`
  - Live `info/joints-detailed` after both recaptures showed:
    - six finite `arm_display_deg` values
    - `display_joint_truth_unavailable_joints=[]`
    - `canonical_joint_truth_unavailable_joints=[]`
  - Stability sample:
    - 30 reads of `http://127.0.0.1:4000/info/joints-detailed` at ~100 ms cadence
    - no fetch failures
    - no `null` display joints
    - all six `arm_display_deg` entries finite on every sample
- Follow-up notes / risks:
  - The underlying `J3/J4` null-display problem is fixed live by fresh display-mode anchor capture.
  - The top-level payload still reports `read_source="unavailable"` even while all six joints now have coherent truth; that field is now inconsistent with the reconstructed state and may still deserve a cleanup pass elsewhere.
  - `J4`'s native-home response had a contradictory post-settle tail: the top-level result verified successfully, but `post_home_settle_native_home_state_name="failed"` with abort `0x06010002`. Since truth remained available and the new anchor persisted, this looks like a follow-up telemetry/settle-state inconsistency rather than a current blocker.

## 2026-04-15 07:07 +0000

- What changed:
  - Ran a fresh J3/J4 Chapter 5 probe after the display-truth repair:
    - `source "/home/pi/GradientOS/.venv/bin/activate" && PYTHONPATH=src python scripts/a6ec_chapter5_probe.py snapshot --label j3-j4-live-power-block --axes J3 J4`
  - Re-checked live `/info/joints-detailed` and `/control/motion-status` to identify the actual power-up blocker.
  - Read the power-transition guard path in `src/gradient_os/arm_controller/command_api.py` and `src/gradient_os/arm_controller/backends/ethercat_rtcore/backend.py`.
- Validation:
  - Probe artifact:
    - JSON: `logs/encoder-retention/20260415-070648-a6ec-ch5-probe/j3-j4-live-power-block.json`
    - Markdown: `logs/encoder-retention/20260415-070648-a6ec-ch5-probe/j3-j4-live-power-block.md`
  - Probe summary:
    - `J3` and `J4` raw encoder / bridge checks were healthy
    - all key bridge deltas stayed within `0..1` counts
  - Live API:
    - `/info/joints-detailed` showed `arm_display_deg` populated for all six joints and `display_joint_truth_available=true`
    - the same payload still showed `arm_rad=[]`, `arm_deg=[]`, `read_source="unavailable"`, and `raw_canonical_joint_truth_available=false`
  - Live motion guard:
    - `/control/motion-status` returned `safe_for_power_transition=false`
    - blocker list: `["not_synchronized"]`
    - detail: `Live feedback is not synchronized yet; keep the drives disarmed.`
  - Code path confirmation:
    - `backend.get_power_transition_snapshot()` computes `feedback_synchronized` from `raw_to_joint_positions(...)`
    - `command_api._build_power_transition_guard()` blocks power-up when that flag is false
- Follow-up notes / risks:
  - The current power-up blocker is not “bad J3/J4 encoder values.” It is the unresolved raw canonical synchronization path.
  - The display/home contract is now healthy, but the raw/controller contract still cannot produce a full canonical list for synchronization and hold-target seeding.
  - `run_controller.py` currently mirrors display-truth unavailable lists into the canonical-unavailable fields, which hides the raw blocker details in `joints-detailed` and makes diagnosis harder.

## 2026-04-15 07:15 +0000

- What changed:
  - Patched `src/gradient_os/run_controller.py` so `joints-detailed` stops overwriting raw canonical unavailable fields with display unavailable fields.
  - Added parsing of the existing raw-truth error string so canonical unavailable axes/joints can still be surfaced even when display truth is available.
  - Added a regression in `tests/test_run_controller_helpers.py` covering the mixed state:
    - raw canonical truth unavailable with joints `[3, 4, 6]`
    - display truth available for all joints
- Validation:
  - `source "/home/pi/GradientOS/.venv/bin/activate" && PYTHONPATH=src python -m pytest tests/test_run_controller_helpers.py -q` -> `7 passed`
  - `ReadLints` on `src/gradient_os/run_controller.py` and `tests/test_run_controller_helpers.py` returned clean.
- Follow-up notes / risks:
  - This patch improves live diagnostics only; it does not change the current power-up block.
  - The actual runtime blocker remains `feedback_synchronized=false` in `backend.get_power_transition_snapshot()`, which still depends on the raw canonical path via `raw_to_joint_positions()`.
  - A future power-up fix must be careful not to broaden display-mode semantics onto the raw command path without explicitly preserving RTCore’s raw wire-frame safety guarantees.

## 2026-04-15 07:45 +0000

- What changed:
  - Patched `src/gradient_os/arm_controller/backends/ethercat_rtcore/backend.py` to make the raw/controller frame wrap-aware without changing display-mode semantics.
  - Added a per-axis raw-reference wrap lift in counts, learned from live raw feedback during `raw_to_joint_positions()`.
  - Reused that same wrap lift when inverting canonical joint positions back into controller axis-q targets, so the outbound command path stays on the live raw branch.
  - Updated `tests/test_gradient05_limits_and_backends.py`:
    - raw truth now remains coherent across a single-turn raw seam even when display truth still fails on an old/raw-style display anchor
    - a J3-style display-mode anchor keeps raw command targets on the live `131039` branch
    - tolerance tests now follow the configured backend roundtrip threshold instead of hard-coding `10` counts
- Validation:
  - `source "/home/pi/GradientOS/.venv/bin/activate" && PYTHONPATH=src python -m py_compile src/gradient_os/arm_controller/backends/ethercat_rtcore/backend.py`
  - `source "/home/pi/GradientOS/.venv/bin/activate" && PYTHONPATH=src python -m pytest tests/test_gradient05_limits_and_backends.py -q` -> `70 passed`
  - `source "/home/pi/GradientOS/.venv/bin/activate" && PYTHONPATH=src python -m pytest tests/test_run_controller_helpers.py -q` -> `7 passed`
  - `ReadLints` on `src/gradient_os/arm_controller/backends/ethercat_rtcore/backend.py` and `tests/test_gradient05_limits_and_backends.py` returned clean
- Follow-up notes / risks:
  - This change is code-and-test validated but not live-loaded into the running controller process; a controller restart is still required before re-checking `/control/motion-status`.
  - The fix is intentionally backend-side: it preserves the display/native-home contract and avoids pushing operator-facing display semantics into RTCore.

## 2026-04-15 08:24 +0000

- What changed:
  - Investigated the user-reported live `J2` wrong-direction / violent jog symptom after power-up by reading:
    - active controller terminal output
    - `logs/startups/20260415-080110/controller.log`
    - `logs/startups/20260415-080110/api.log`
    - direct EtherCAT objects for `J2`
    - `/run/gradient-rt-motion/metrics.json`
    - live `/info/joints-detailed` and `/control/motion-status`
  - Captured a fresh post-home probe with:
    - `source "/home/pi/GradientOS/.venv/bin/activate" && PYTHONPATH=src python scripts/a6ec_chapter5_probe.py snapshot --label j2-post-home-now --axes J2`
  - Wrote a comparison artifact:
    - `logs/encoder-retention/20260415-082343-a6ec-ch5-probe/j2-pre-vs-post-home-summary.md`
- Validation:
  - Controller/API logs showed the third discrete jog targeted `J2`, after which canonical truth became unavailable across multiple axes and the next `/control/joint-jog` returned `409 Conflict`
  - Pre-home `J2` direct drive reads:
    - `0x60B0 = 0`
    - `0x607C = 0`
    - `0x6041 = 0x1650`
    - `0x603F = 0x0000`
  - Post-home `J2` direct drive reads:
    - `0x60B0 = 0`
    - `0x607C = 0`
    - `0x6041 = 0x9650`
    - `0x603F = 0x0000`
  - Post-home RTCore/API state:
    - `/run/gradient-rt-motion/metrics.json` axis 1 reports `native_home_state = 2` but `native_home_position_offset = 0`
    - `/info/joints-detailed` reports `J2 arm_deg ~= 0.0006866` with `read_source = live_feedback` and raw canonical truth available
    - `/control/motion-status` reports `safe_for_power_transition = true`
  - Anchor persistence:
    - `.gradient_absolute_encoder_anchors.json` refreshed `J2` to `home_anchor_rad = 0.007341536177021437` at `2026-04-15T08:12:47+00:00`
  - Post-home probe artifact:
    - `logs/encoder-retention/20260415-082343-a6ec-ch5-probe/j2-post-home-now.json`
    - bridge checks remained in the standard `0..1` count wander band
- Follow-up notes / risks:
  - The latest `J2` home clearly repaired the anchored absolute-truth path and restored a near-zero API readout.
  - It did not restore a nonzero drive-native offset in `0x60B0` / `0x607C` or RTCore `native_home_position_offset`, so this still resembles the old `J2` frame/home mismatch family more than a fully clean native-home repair.
  - Current `power_transition_feedback_synchronized = true`, so the raw truth block is not presently preventing re-enable; the bigger concern is whether a new `J2` jog can still re-trigger the mismatch under motion.

## 2026-04-15 08:39 +0000

- What changed:
  - Captured a fresh pre-jog `J2` Chapter 5 probe with:
    - `source "/home/pi/GradientOS/.venv/bin/activate" && PYTHONPATH=src python scripts/a6ec_chapter5_probe.py snapshot --label j2-pre-jog --axes J2 --experiment-id 20260415-0824-j2-jog-frame-check`
  - Saved a consolidated live runtime baseline for `J2` at:
    - `logs/encoder-retention/20260415-0824-j2-jog-frame-check/j2-runtime-pre-jog.json`
    - `logs/encoder-retention/20260415-0824-j2-jog-frame-check/j2-runtime-pre-jog.md`
- Validation:
  - Probe artifacts:
    - `logs/encoder-retention/20260415-0824-j2-jog-frame-check/j2-pre-jog.json`
    - `logs/encoder-retention/20260415-0824-j2-jog-frame-check/j2-pre-jog.md`
  - Probe summary for `J2`:
    - `raw_formula_match = false`
    - `raw_formula_delta_counts = -3`
    - `bridge_6063_from_6064`, `bridge_60fc_from_6062`, and `bridge_u402a_from_u4028` all stayed within the normal `0..1` count band
  - Current direct reads captured into the runtime baseline:
    - `0x60B0 = 0`
    - `0x607C = 0`
    - `0x6041 = 0x9650`
    - `0x603F = 0x0000`
  - Current RTCore/API state in the same baseline:
    - axis 1 `native_home_state = 2`
    - axis 1 `native_home_position_offset = 0`
    - `/info/joints-detailed` `J2 arm_deg ~= 0.0006866`
    - `/control/motion-status` `safe_for_power_transition = true`
- Follow-up notes / risks:
  - This is now a clean pre-jog capture set for `J2`; the next meaningful comparison is to take the same probe/runtime snapshots immediately after the next tiny `J2` jog.
  - The system is presently safe to re-enable, but the zero drive-native offset path (`0x60B0 = 0`, `0x607C = 0`, RTCore offset `0`) still leaves `J2` suspicious even though API/controller truth is coherent right now.

## 2026-04-15 08:47 +0000

- What changed:
  - Investigated the user's "we should be able to just read a joint position" complaint against the live API before the next hard stop / drive power cycle.
  - Queried the current live payloads:
    - `http://127.0.0.1:4000/info/joints-detailed`
    - `http://127.0.0.1:4000/info/joints`
  - Re-read the jog gating code in `src/gradient_os/api/main.py` and the truth-flag composition in `src/gradient_os/run_controller.py`.
- Validation:
  - Live `/info/joints-detailed` currently reports:
    - `read_source = live_feedback`
    - `raw_canonical_joint_truth_available = true`
    - finite `arm_deg` for all six joints
    - `arm_display_deg` is `null` only for `J3`
    - `display_joint_truth_unavailable_joints = [3]`
    - `display_joint_truth_reason = absolute_home_anchor_stale`
    - `canonical_joint_truth_available = false`
  - `src/gradient_os/run_controller.py` still computes top-level `canonical_joint_truth_available` by starting from raw/live truth and then AND-ing in display truth.
  - `src/gradient_os/api/main.py` `/control/joint-jog` still rejects a jog when that global top-level flag is false, before it reaches the selected-joint truth check.
- Follow-up notes / risks:
  - The current live issue is not "the system cannot read joint position." The current live issue is that one `J3` display-anchor failure poisons the old global truth flag and the jog route still uses that global flag as a hard precondition.
  - A hard stop / drive power cycle may reset drive-side runtime objects, but by itself it does not guarantee the host-side stale display anchor or the API's global jog gate will clear.

## 2026-04-15 08:55 +0000

- What changed:
  - Reviewed the manufacturer's written reply against the existing A6-EC bench evidence and workstream notes.
  - Reconciled which parts are now explicitly vendor-confirmed versus still unresolved.
- Validation:
  - Vendor-confirmed points that match our bench direction:
    - `C00.07 = 4` is the intended absolute rotation mode
    - HM method `35` with `0x6060 = 6`, `0x6098 = 35`, `0x60E6 = 0`, then return to `0x6060 = 8`
    - `0x607C` is the persistent origin/home object and auto-saves
    - `0x60B0` is runtime-only
    - `0x6064` is the authoritative CSP/application position after homing
    - HM success/reference validity requires `0x6041 bit12 = 1` and `bit15 = 1`
  - Checked `0x9650` bit decode directly: set bits are `[4, 6, 9, 10, 12, 15]`, so it matches the vendor-stated HM success condition with bit 13 clear.
- Follow-up notes / risks:
  - This is strong confirmation that the long-running `0x60B0` persistence path was the wrong object model for A6-EC and that `0x9650` is a meaningful success terminal state, not an incidental value.
  - The reply still does not answer several earlier questions we care about:
    - exact role of `U40.16` relative to `0x6064`
    - whether direct `0x607C` writes alone establish a valid homing/reference state
    - `0x607C` signed/range behavior in rotation mode (`0..RM-1` vs persisted negative writes)
    - whether `C10.18/C10.19` must match true mechanics
    - what `0x2013:17` and `F31.10` do in this persistence/reference workflow

## 2026-04-15 09:05 +0000

- What changed:
  - Re-read the A6-EC manual excerpts to answer the user's follow-up on the vendor phrase "one full revolution of the load" and whether `RM` needs clarification.
- Validation:
  - `docs/resources/a6ec_manual_chapter_11_parameter_list.md` states under `6091`:
    - "The gear ratio is used to establish the proportional relationship between the load shaft displacement designated by the user and the motor shaft displacement."
    - "Motor position feedback = Load shaft position feedback x Gear ratio"
  - The same manual section for rotation mode still leaves the implementation source of `RM` ambiguous relative to `C10.1A/C10.1C`, `C10.18/C10.19`, and other scaling objects.
- Follow-up notes / risks:
  - Best current interpretation: "full revolution of the load" means one revolution of the output/load shaft in the user/application sense, not one motor-shaft turn.
  - This should still be sent back for clarification because the vendor did not define `RM` algebraically, did not state which objects determine it in absolute rotation mode, and did not reconcile the claimed `0..RM-1` `0x607C` range with our persisted negative `0x607C` observations.

## 2026-04-15 09:19 +0000

- What changed:
  - Tightened native-home status fallback semantics to match the new vendor guidance:
    - updated `src/gradient_os/telemetry/native_home_status.py`
    - updated `src/gradient_os/arm_controller/backends/ethercat_rtcore/backend.py`
  - Changed the statusword-derived success marker from `statusword_bit15` to `statusword_bits12_15_clear13`.
  - Updated `web-ui/src/ControlPanel.tsx` and `web-ui/src/ControlPanel.test.tsx` so the UI still recognizes both the new and legacy statusword-derived success markers when rendering conservative drive-home status.
  - Updated `scripts/a6ec_chapter5_probe.py` and `tests/test_a6ec_chapter5_probe.py` so probe snapshots now expose an explicit `vendor_hm_success_signature` plus `bit15_reference_attained`.
  - Updated focused regression expectations in:
    - `tests/test_gradient05_limits_and_backends.py`
    - `tests/test_drive_faults.py`
- Validation:
  - `source "/home/pi/GradientOS/.venv/bin/activate" && PYTHONPATH=src python -m pytest tests/test_gradient05_limits_and_backends.py -q -k "native_home_metrics_result or native_home_post_settle or wait_for_native_home_result or absolute_home_anchor_stale"` -> `10 passed`
  - `source "/home/pi/GradientOS/.venv/bin/activate" && PYTHONPATH=src python -m pytest tests/test_drive_faults.py tests/test_a6ec_chapter5_probe.py -q` -> `14 passed`
  - `cd /home/pi/GradientOS/web-ui && npx vitest run src/ControlPanel.test.tsx` -> `18 passed`
  - `ReadLints` on all touched product/test files returned clean
- Follow-up notes / risks:
  - The RTCore HM executor already matched the vendor-confirmed terminal condition (`bits 12+15 set, bit 13 clear`); this pass only corrected the Python-side fallback/telemetry interpretation.
  - I intentionally did not change RTCore queued-target conversion (`controller_target_counts - native_home_offset_counts`) in this pass. Existing bench notes still indicate that motion-path frame change needs separate live proof before it is safe to alter.

## 2026-04-15 09:31 +0000

- What changed:
  - Investigated the user's latest post-restart `J2` native-home run using:
    - `logs/startups/20260415-092423/controller.log`
    - `logs/startups/20260415-092423/api.log`
    - `journalctl -u gradient-rt-motion.service -n 120 --no-pager`
    - live `/info/joints-detailed`
    - live `/control/motion-status`
    - direct SDO reads of `J2` `0x607C`, `0x6064`, `U40.16`, and `0x6041`
    - `/run/gradient-rt-motion/metrics.json`
  - Implemented a new `watch` subcommand in `scripts/a6ec_chapter5_probe.py` for live hand-rotation / streaming captures.
  - Added tests covering the new watch helpers in `tests/test_a6ec_chapter5_probe.py`.
- Validation:
  - Latest `J2` native-home evidence after restart:
    - controller log shows `NATIVE_HOME_JOINT,2` followed by `Native drive-home verified`
    - RTCore journal logs `EtherCAT native_home axis=1 ... feedback_counts=2420 truth_value=0 commissioning_mode=6 steady_state_mode=8`
    - direct SDO reads return:
      - `0x607C = 0`
      - `0x6064 = 20`
      - `U40.16 = 21`
      - `0x6041 = 0x9650`
    - RTCore metrics axis 1 report:
      - `native_home_state = 2`
      - `native_home_position_offset = 0`
      - `statusword = 0x9650`
      - `pos_counts = 20`
    - live `/info/joints-detailed` reports `J2 arm_deg ~= 0.0005768`
    - live `/control/motion-status` reports `safe_for_power_transition = true`
  - New probe watch validation:
    - `source "/home/pi/GradientOS/.venv/bin/activate" && PYTHONPATH=src python -m pytest tests/test_a6ec_chapter5_probe.py -q` -> `6 passed`
    - `source "/home/pi/GradientOS/.venv/bin/activate" && PYTHONPATH=src python scripts/a6ec_chapter5_probe.py watch --label j6-dry-run --axes J6 --samples 1 --interval-s 0.1 --quiet` -> wrote a valid JSONL sample under `logs/encoder-retention/20260415-093037-a6ec-ch5-probe/`
  - `ReadLints` on the touched probe script/test returned clean
- Follow-up notes / risks:
  - This latest `J2` run strongly indicates the standard production HM workflow is now landing in the intended zero-offset case (`truth_value=0`, `0x607C=0`), so RTCore's queued-target subtraction is currently a no-op for that workflow.
  - The unresolved motion-path concern is now narrower: it mainly matters for nonzero-`607C` scenarios or alternate origin conventions, not for the standard `HM 35 + 607C=0` home sequence.

## 2026-04-15 09:38 +0000

- What changed:
  - Started a live `J6` hand-rotation watcher for the user's upcoming manual brake-release experiment with:
    - `source "/home/pi/GradientOS/.venv/bin/activate" && PYTHONPATH=src python scripts/a6ec_chapter5_probe.py watch --label j6-hand-rotate-live --axes J6 --interval-s 0.25`
- Validation:
  - Watcher reached healthy steady state and wrote metadata plus live samples to:
    - `logs/encoder-retention/20260415-093803-a6ec-ch5-probe/j6-hand-rotate-live.watch.jsonl`
  - Initial observed `J6` baseline from the first live stream lines:
    - `6064 ~= 108296`
    - `607C = 0`
    - `U40.16 ~= -22776`
    - raw absolute `U40.20/.22 ~= 33350`
    - rotation-mode counts `U40.2A/.2C ~= 108296`
    - API `arm_deg ~= 6.257`
    - `vendor_hm_success_signature = false`
- Follow-up notes / risks:
  - This is a high-value starting state because it already shows `607C = 0` while the HM-valid signature is false and `6064` is nonzero.
  - If the user rotates `J6` by hand while the drive electronics stay powered and the axis stays disarmed, the new watch stream should let us see exactly which position-domain families track the manual motion together.

## 2026-04-15 09:42 +0000

- What changed:
  - Stopped the live `J6` hand-rotation watcher after the user completed a large manual rotation in both directions.
  - Summarized the finished stream from:
    - `logs/encoder-retention/20260415-093803-a6ec-ch5-probe/j6-hand-rotate-live.watch.jsonl`
- Validation:
  - Completed stream size: `257` samples
  - Invariants observed across the run:
    - `607C` stayed exactly `0`
    - `statusword` stayed `0x1650`
    - `vendor_hm_success_signature` stayed `false`
    - `6064` stayed in a bounded one-turn-like band (`4129 .. 129172`)
    - `U40.28/U40.2A/.2C` stayed in the same bounded rotation/reference band (`3265 .. 130438`)
    - `U40.16` and raw absolute `U40.20/.22` moved through large multi-turn ranges (`~ -2.84M .. +1.87M`, `~ -2.79M .. +1.87M`)
    - API `arm_deg` / `arm_display_deg` also ranged widely (`~ -377° .. +780°`)
- Follow-up notes / risks:
  - This is strong read-side evidence that `607C = 0` alone does not establish an active valid homed/reference frame; the HM-valid statusword bits still matter.
  - This run did not exercise the `0x607A` command/write path, so it does not by itself justify changing RTCore's queued-target subtraction logic.
  - The motion-path concern is now narrower: to answer it directly we need a controlled nonzero-`607C` experiment or another command-path proof, not just more read-only hand-rotation traces.

## 2026-04-15 09:56 +0000

- What changed:
  - Ran the next controlled nonzero-`607C` write-path experiment on `J2` while the axis was still in the clean post-home state (`0x6041 = 0x9650`, `vendor_hm_success_signature = true`, `0x607C = 0`).
  - Captured three probe snapshots under `logs/encoder-retention/20260415-j2-607c-write-test/`:
    - `j2-pre-607c-write.json/.md`
    - `j2-post-607c-write.json/.md`
    - `j2-post-607c-restore.json/.md`
  - Wrote a temporary positive home offset with:
    - `sudo ethercat download -p 1 -t int32 0x607C 0 12345`
  - Restored `0x607C` to zero before any motion with:
    - `sudo ethercat download -p 1 -t int32 0x607C 0 0`
- Validation:
  - Immediate readback after the temporary write:
    - `0x607C = 12345`
    - `0x6041 = 0x9650`
    - `0x6064 = 21`
    - `U40.16 = 22`
  - Snapshot-to-snapshot comparison showed no large frame jump when `0x607C` changed:
    - `6064`: `21 -> 22 -> 22`
    - `U40.16`: `21 -> 22 -> 22`
    - raw absolute `U40.20/.22`: `17758 -> 17759 -> 17758`
    - API `raw_counts`: `23 -> 21 -> 21`
    - API `absolute_counts`: `17758 -> 17761 -> 17760`
    - API `canonical_rad`: `9.587e-06 -> 1.103e-05 -> 1.055e-05`
  - `0x6041` stayed `0x9650` and `vendor_hm_success_signature` stayed true for all three snapshots.
- Follow-up notes / risks:
  - This is strong evidence that a direct nonzero `0x607C` write is not being immediately absorbed into the live `6064`/`U40.16`/API truth frame in the current steady-state workflow.
  - That weakens the live double-apply hypothesis for RTCore's current queued-target subtraction, but it still does not fully close the question for post-write motion, re-arm, HM rerun, or power-cycle activation paths.

## 2026-04-15 10:09 +0000

- What changed:
  - Ran the next activation-timing experiment on `J2` to test whether a nonzero `0x607C` becomes active on `SAFE_POWER_UP`.
  - Stored six snapshots under `logs/encoder-retention/20260415-j2-607c-powerup-activation-test/`:
    - `j2-pre-powerup-activation.json/.md`
    - `j2-post-write-disarmed.json/.md`
    - `j2-post-power-up.json/.md`
    - `j2-post-restore-write-disarmed.json/.md`
    - `j2-post-restore-power-up.json/.md`
    - `j2-final-disarmed.json/.md`
  - Executed this sequence:
    - verified initial `safe_for_power_transition = true`
    - wrote `sudo ethercat download -p 1 -t int32 0x607C 0 12345`
    - issued API `POST /control/power-up`
    - issued API `POST /control/power-down`
    - wrote `sudo ethercat download -p 1 -t int32 0x607C 0 0`
    - issued API `POST /control/power-up`
    - issued API `POST /control/power-down`
- Validation:
  - Direct write while disarmed again showed no immediate large jump:
    - `0x607C: 0 -> 12345`
    - `0x6064: 21 -> 23`
    - `U40.16: 23 -> 23`
    - API `canonical_rad` unchanged at `~1.10e-05`
  - First `SAFE_POWER_UP` changed `J2`, but not by the written `0x607C` amount:
    - `0x607C` stayed `12345`
    - `0x6064: 21 -> 2253`
    - `U40.16: 23 -> 2252`
    - raw absolute `U40.20/.22: 17761 -> 19991`
    - API `absolute_counts: 17761 -> 19990`
    - API `canonical_rad: ~1.10e-05 -> ~1.08e-03`
  - Key bridge invariants stayed effectively constant across the whole run:
    - `combined(U40.20/.22) - 6064 ~= 17737..17740`
    - `api absolute_counts - raw_counts ~= 17736..17739`
    - `absolute_home_anchor_rad` stayed exactly `0.008503047254848595`
  - After restoring `0x607C = 0`, a second `SAFE_POWER_UP` still shifted the same families again:
    - `0x6064: 2283 -> 4464`
    - `U40.16: 2283 -> 4464`
    - raw absolute `U40.20/.22: 20022 -> 22203`
  - Final post-settle state after the closing `SAFE_POWER_DOWN`:
    - controller returned to `safe_for_power_transition = true`
    - final snapshot: `0x607C = 0`, `0x6041 = 0x9650`, `0x6064 ~= 4495`, `U40.16 ~= 4495`, `U40.20/.22 ~= 22232`
- Follow-up notes / risks:
  - This further weakens the hypothesis that the current power-up path is simply "activating nonzero `0x607C` into `6064` and causing RTCore double-apply." The observed shift was not `12345` counts and it moved the raw absolute and reference families together.
  - The new leading question is why `J2` whole-frame counts move coherently by about `~2.2k` counts across idle `SAFE_POWER_UP` transitions, even after `0x607C` is restored to zero.
  - The system was returned to a disarmed, settled state at the end of the experiment, but `J2` did not numerically return to its original near-zero `6064`/API state.

## 2026-04-15 10:22 +0000

- What changed:
  - Re-read the attached manual extracts for:
    - `docs/resources/chapter 5 absolute system - extract from A6-EC_series_servo_drive_manual (2).pdf`
    - `docs/resources/chapter 11 - parameter list - A6-EC_series_servo_drive_manual (2).pdf`
  - Cross-referenced the manual wording against the vendor reply and the latest `J2` / `J6` experiments.
- Validation:
  - Chapter 5 confirms:
    - `C00.07 = 4` is absolute position rotation mode.
    - rotation mode is intended for unlimited load travel with `< 32767` unidirectional revolutions.
    - `RM` is `encoder pulses per load revolution`.
    - in rotation mode while in HM, the home-offset range is `0 .. (RM - 1)`.
    - the drive calculates the upper limit of mechanical absolute position from `C10.1A/C10.1C` first, otherwise `C10.18/C10.19`.
  - Chapter 11 confirms:
    - `6064` is reference-unit position actual value and `6064 * 6091 = 6063`.
    - `607C` is home offset.
    - `60B0` is position offset.
    - `60E6` is the actual-position calculation method after homing.
    - `607C` is said to be active when powered on, homing is complete, and `6041 bit15 = 1`.
    - after homing, `6064` is said to equal `607C`.
    - `6091` defines the proportional relationship between load-shaft displacement and motor-shaft displacement.
  - Manual/bench alignment:
    - the documented `6064`/`6063` split matches our observed reference-vs-encoder frame split
    - the `6091` wording reinforces the interpretation that "load" means load/output shaft
    - the attached Chapter 5/11 extracts do **not** spell out the vendor-confirmed HM success bits (`bit12 + bit15`, bit13 clear)
  - Manual/bench tension:
    - our clean `J2` tests met the documented `607C` activation preconditions (`powered on`, homed, `bit15=1`) but direct `607C` writes still did not immediately rebase `6064`
    - Chapter 5 lists `U40.16` under absolute linear mode and `U40.28` under absolute rotation mode, yet on the live rotation-mode axes `U40.16` is still present and behaved differently from `6064`
- Follow-up notes / risks:
  - The manual revisit usefully narrows the open questions: the main unresolved issues are now `607C` direct-write activation semantics, the exact role of `60E6`, and the meaning/validity of `U40.16` in rotation mode.
  - A strong new manufacturer follow-up is to ask whether direct manual `607C` writes are supposed to affect `6064` immediately once the documented activation conditions are already true, or only after a specific refresh event such as HM, software reset, or repower.

## 2026-04-15 10:30 +0000

- What changed:
  - Ran the requested zero-`607C` `J6` control sequence with probe plus safe power transitions only, no jog:
    - `python3 scripts/a6ec_chapter5_probe.py snapshot --label j6-pre-zero-607c-control --axes J6 --experiment-id 20260415-j6-zero-607c-power-control`
    - API `POST /control/power-up`
    - `python3 scripts/a6ec_chapter5_probe.py snapshot --label j6-post-power-up --axes J6 --experiment-id 20260415-j6-zero-607c-power-control`
    - API `POST /control/power-down`
    - `python3 scripts/a6ec_chapter5_probe.py snapshot --label j6-final-disarmed --axes J6 --experiment-id 20260415-j6-zero-607c-power-control`
  - Re-checked final controller motion status after the sequence.
- Validation:
  - Final controller state returned to:
    - `safe_for_power_transition = true`
    - `power_transition_blockers = []`
    - `state = idle`
  - Across the three snapshots:
    - `0x607C` stayed `0`
    - `vendor_hm_success_signature` stayed `false`
    - pre snapshot: `0x6041 = 0x1650`, `6064 = 40736`, `U40.16 = -90338`, raw absolute `U40.20/.22 = -34214`
    - post power-up: all major families shifted only about `41..46` counts
    - final disarmed: `6064` returned exactly to baseline, `U40.16` returned within `1` count, raw absolute `U40.20/.22` within `5` counts
    - bridge invariants stayed effectively flat:
      - `abs_minus_ref`: `-74950 -> -74945 -> -74945`
      - `api_abs_minus_raw`: `-74947 -> -74946 -> -74945`
      - `absolute_home_anchor_rad` unchanged
- Follow-up notes / risks:
  - This is a strong control result against the `J2` anomaly: `SAFE_POWER_UP` / `SAFE_POWER_DOWN` by themselves do **not** inherently cause the large coherent multi-family shift seen on `J2`.
  - The leading interpretation is now that the `J2` transition behavior is axis-specific (for example load/gravity/brake/compliance or another `J2`-local effect), not a universal `607C` activation behavior.
  - The generic `11.3.11 Group U40` prose remains non-probative for `U40.16`/`U40.20/.22`/`U40.28/.2A/.2C`; it only documents the low-number generic monitor fields and does not settle rotation-mode semantics for the fields we actually care about.

## 2026-04-15 10:35 +0000

- What changed:
  - Ran the next `J6` nonzero-`607C` power-control sequence, still with no jog:
    - `python3 scripts/a6ec_chapter5_probe.py snapshot --label j6-pre-nonzero-607c-control --axes J6 --experiment-id 20260415-j6-nonzero-607c-power-control`
    - `sudo ethercat download -p 5 -t int32 0x607C 0 4096`
    - `python3 scripts/a6ec_chapter5_probe.py snapshot --label j6-post-write-disarmed --axes J6 --experiment-id 20260415-j6-nonzero-607c-power-control`
    - API `POST /control/power-up`
    - `python3 scripts/a6ec_chapter5_probe.py snapshot --label j6-post-power-up --axes J6 --experiment-id 20260415-j6-nonzero-607c-power-control`
    - API `POST /control/power-down`
    - `python3 scripts/a6ec_chapter5_probe.py snapshot --label j6-post-power-down-nonzero --axes J6 --experiment-id 20260415-j6-nonzero-607c-power-control`
    - `sudo ethercat download -p 5 -t int32 0x607C 0 0`
    - `python3 scripts/a6ec_chapter5_probe.py snapshot --label j6-final-disarmed --axes J6 --experiment-id 20260415-j6-nonzero-607c-power-control`
  - Re-checked final controller motion status after restoring `0x607C = 0`.
- Validation:
  - Initial state:
    - `0x6041 = 0x1650`
    - `vendor_hm_success_signature = false`
    - `0x607C = 0`
    - `6064 = 40734`
    - `U40.16 = -90337`
    - raw absolute `U40.20/.22 = -34210`
  - Direct nonzero write remained inert while disarmed:
    - `0x607C: 0 -> 4096`
    - `6064: 40734 -> 40734`
    - `api_raw_counts: 40734 -> 40734`
    - `api_absolute_counts: -34213 -> -34212`
  - `SAFE_POWER_UP` still caused only the same small drift-band movement as the zero-`607C` control:
    - `6064: 40734 -> 40691` (`-43`)
    - `U40.16: -90337 -> -90383` (`-46`)
    - raw absolute `U40.20/.22: -34210 -> -34255` (`-45`)
    - API `canonical_rad` changed by only `~2.06e-04`
  - Bridge invariants stayed effectively flat:
    - `abs_minus_ref: -74944 -> -74946 -> -74948 -> -74946`
    - `api_abs_minus_raw: -74947 -> -74946 -> -74948 -> -74946 -> -74947`
    - `absolute_home_anchor_rad` unchanged
  - Final restored state:
    - `0x607C = 0`
    - `safe_for_power_transition = true`
    - `power_transition_blockers = []`
- Follow-up notes / risks:
  - This shows that on `J6`, while HM-valid remains false, a small direct nonzero `607C` still does not produce any observable selective rebase of `6064`, `U40.16`, or API truth.
  - Together with the zero-`607C` control, this further isolates the `J2` large-shift behavior as axis-specific rather than a generic `607C` activation behavior.
  - Scope limit remains important: because `J6` never entered a clean HM-valid state in this run, this does not answer the distinct question of what nonzero `607C` would do on a clean homed `J6`-style axis.

## 2026-04-15 10:49 +0000

- What changed:
  - Ran the requested `J6` zero-`607C` jog experiment as a new sequence under `logs/encoder-retention/20260415-j6-zero-607c-jog-control/`:
    - `j6-pre-jog-control.json/.md`
    - `j6-post-power-up-pre-jog.json/.md`
    - `j6-post-jog.json/.md`
    - `j6-final-disarmed.json/.md`
  - Powered up with API `POST /control/power-up`.
  - Because `/control/joint-jog` is still blocked by the unrelated top-level canonical/display truth gate, sent the same underlying controller command directly over UDP:
    - live-read current `arm_deg` from `/info/joints-detailed`
    - added `+0.25 deg` to `J6`
    - sent `APPLY_JOINT_SETPOINT,<base64-json>` with `max_motor_rpm = 100.0`
  - Waited for idle with API `POST /control/wait-for-idle`.
  - Powered down with API `POST /control/power-down`.
- Validation:
  - Command accepted cleanly:
    - controller replied `ACK,APPLY_JOINT_SETPOINT,...`
    - RTCore reported trajectory `id=1`, `duration_s=0.25`, `frequency_hz=100`
  - `WAIT_FOR_IDLE` reported the move `completed`.
  - During the post-complete state, RTCore kept `active_trajectory` latched briefly; `SAFE_POWER_DOWN` cleared that latch and returned the system to:
    - `safe_for_power_transition = true`
    - `active_traj_id = 0`
  - Motion result from powered pre-jog to post-jog:
    - API `canonical_deg`: `24.82525634765625 -> 25.07601928710937` (`+0.25076293945312145 deg`)
    - `6064`: `40690 -> 39777` (`-913`)
    - `U40.16`: `-90382 -> -91295` (`-913`)
    - raw absolute `U40.20/.22`: `-34256 -> -35169` (`-913`)
    - `U40.28`: `40689 -> 39775` (`-914`)
    - `U40.2A/.2C`: `40689 -> 39776` (`-913`)
  - Coherence checks stayed healthy:
    - `combined(U40.20/.22) - 6064` stayed constant
    - `absolute_home_anchor_rad` stayed unchanged
    - final disarmed state remained near the post-jog pose rather than falling back to the pre-jog pose
- Follow-up notes / risks:
  - This is the strongest direct evidence so far that `J6` motion semantics at `607C = 0` are healthy: the tiny commanded move produced a matching tiny canonical pose change and the raw/reference families moved together coherently.
  - The public `/control/joint-jog` route remains misleadingly blocked by the top-level global truth gate; that API problem is now even more clearly separate from the underlying `J6` motion path.
  - The next clean extension, if desired, is the same tiny `J6` jog test on the nonzero-`607C` branch to see whether motion stays equally coherent there.

## 2026-04-15 10:59 +0000

- What changed:
  - Ran the requested nonzero-`607C` `J6` jog experiment as a new sequence under `logs/encoder-retention/20260415-j6-nonzero-607c-jog-control/`:
    - `j6-pre-nonzero-jog-control.json/.md`
    - `j6-post-write-disarmed.json/.md`
    - `j6-post-power-up-pre-jog.json/.md`
    - `j6-post-jog.json/.md`
    - `j6-post-power-down-nonzero.json/.md`
    - `j6-final-disarmed.json/.md`
  - Wrote `J6 0x607C = 4096` while disarmed, then powered up with API `POST /control/power-up`.
  - As with the prior jog experiment, bypassed the broken public `/control/joint-jog` route and sent the underlying controller command directly over UDP:
    - live-read current `arm_deg` from `/info/joints-detailed`
    - added `+0.25 deg` to `J6`
    - sent `APPLY_JOINT_SETPOINT,<base64-json>` with `max_motor_rpm = 100.0`
  - Waited for idle with API `POST /control/wait-for-idle`, then powered down with `POST /control/power-down`.
  - Restored `J6 0x607C = 0` after the run.
- Validation:
  - Immediate disarmed write still looked inert:
    - `j6-pre-nonzero-jog-control` had `607C = 0`
    - `j6-post-write-disarmed` had `607C = 4096`
    - API canonical pose stayed unchanged across that write
  - Motion command accepted cleanly:
    - controller replied `ACK,APPLY_JOINT_SETPOINT,...`
    - RTCore reported trajectory `id=2`, `duration_s=0.25`, `frequency_hz=100`
    - `WAIT_FOR_IDLE` reported the move `completed`
  - Motion result from powered pre-jog to post-jog:
    - API `canonical_deg`: `25.083984375 -> 25.33529663085937` (`+0.25131225585937145 deg`)
    - `6064`: `39746 -> 38831` (`-915`)
    - `U40.16`: `-91326 -> -92237` (`-911`)
    - raw absolute `U40.20/.22`: `-35200 -> -36114` (`-914`)
    - `U40.28`: `39746 -> 38834` (`-912`)
    - `U40.2A/.2C`: `39748 -> 38833` (`-915`)
  - Coherence checks stayed healthy:
    - `combined(U40.20/.22) - 6064` stayed within `1` count
    - `api absolute_counts - raw_counts` stayed within `3` counts
    - `absolute_home_anchor_rad` stayed unchanged
  - Powered-down and restored-zero states remained clean:
    - `j6-post-power-down-nonzero` still showed nonzero `607C = 4096`
    - `j6-final-disarmed` showed restored `607C = 0`
    - final motion status was `safe_for_power_transition = true`, `active_traj_id = 0`
- Follow-up notes / risks:
  - This materially strengthens the earlier conclusion: on non-HM-valid `J6`, a small nonzero `607C` does not appear to change the live motion semantics. The tiny commanded move stayed coherent and almost numerically identical to the zero-`607C` control jog.
  - That pushes the remaining uncertainty away from generic `J6` nonzero-`607C` behavior and toward either:
    - axis-specific `J2` behavior, or
    - the still-untested case where an axis is both nonzero-`607C` and cleanly HM-valid.
  - The public `/control/joint-jog` route is still blocked by the unrelated global truth gate, so future motion experiments will remain easier to interpret if they continue using the direct `APPLY_JOINT_SETPOINT` path until that API gate is fixed.

## 2026-04-15 11:07 +0000

- What changed:
  - Re-checked the A6-EC manual directly for reset/default behavior before any `J2` pre-replacement recommendation.
  - Confirmed from `docs/resources/chapter 11 - parameter list - A6-EC_series_servo_drive_manual (2).pdf` that the vendor reset family is under `2031h/F31`:
    - `F31.00 / 0x2031:01` = fault reset
    - `F31.01 / 0x2031:02` = software reset
    - `F31.02 / 0x2031:03` = parameter initialization
    - `F31.03 / 0x2031:04` = drive/motor parameter reset
    - `F31.10 / 0x2031:11` = encoder data reset/read/write/fault reset
  - Confirmed manual semantics:
    - all are `At stop` and `Immediately` effective
    - `F31.01` software reset is allowed only while the drive is disabled and there is no non-resettable fault
    - `F31.10` warns that resetting multi-turn encoder data changes saved absolute position abruptly and requires mechanical homing
  - Rechecked prior live bench evidence from the earlier `J2` reset probe:
    - normal `POST /control/reset-faults` did not restore the pre-boot pose
    - vendor software reset `0x2031:02 = 1` also did not restore HM/reference-valid state
- Validation:
  - Read manual reset table and descriptions from the Chapter 11 parameter-list extract
  - Re-read the prior reset experiment entry in `.cursor/memory/DEVLOG.md`
- Follow-up notes / risks:
  - The manual does contain the factory/default reset operations that were missing from the active SOP notes, but that does not make them the best next `J2` step.
  - Current recommendation remains: do not jump straight to `F31.02`/`F31.03` factory/default resets before backup and a full recommission plan, because they are destructive and our current evidence does not yet point to a simple stale-parameter problem.
  - If the user wants this made durable in team-facing docs, the next documentation task is to add a reset-object subsection to the commissioning SOP with the manual-backed guardrails and the project-specific caution about `J2`.

## 2026-04-15 11:15 +0000

- What changed:
  - Ran the requested softer reset probe on `J2` using the manual-backed vendor software-reset object `F31.01 / 0x2031:02`.
  - Captured before/after probe artifacts under `logs/encoder-retention/20260415-j2-software-reset-probe/`:
    - `j2-pre-software-reset.json/.md`
    - `j2-post-software-reset.json/.md`
  - Verified baseline controller state was idle/disarmed before the write.
  - Confirmed `J2` is still slave `-p 1` from direct EtherCAT reads.
  - Wrote `sudo ethercat download -p 1 -t uint16 0x2031 0x02 1`.
  - Polled RTCore metrics through the reset and then issued `POST /control/reset-faults` to clear the induced transient drive fault.
- Validation:
  - Pre-reset `J2` snapshot:
    - `0x6041 = 0x9650`
    - `vendor_hm_success_signature = true`
    - `0x607C = 0`
    - `0x6064 = 13350`
    - raw absolute `U40.20/.22 = 31087`
    - API `canonical_deg ~= 0.36661376953124997`
    - roundtrip error `0`
  - Software-reset transition:
    - RTCore dropped to `startup_ready = 0`
    - `wkc_actual` temporarily dropped as low as `7`
    - after recovery, RTCore returned to `startup_ready = 1`, `wkc_actual = 18`
  - Post-reset `J2` snapshot:
    - `0x6041 = 0x1650`
    - `vendor_hm_success_signature = false`
    - `0x607C = 0`
    - `0x6064 = 113099`
    - raw absolute `U40.20/.22` still `31087`
    - API canonical truth unavailable
    - roundtrip error `~31322` counts
  - Side effects:
    - controller reported transient `fault_present` on axis `0` and `not_synchronized`
    - `metrics.json` showed axis `0` fault code `34560` (`0x8700`) before cleanup
    - `POST /control/reset-faults` cleared the drive fault, but controller/API still remained in `not_synchronized`
    - `/info/joints-detailed` currently reports `read_source = unavailable` and empty `arm_deg`
- Follow-up notes / risks:
  - This re-run is stronger evidence than the earlier summary alone: `F31.01` software reset is not a harmless pre-replacement diagnostic on the current stack. It can actively destroy a previously coherent `J2` home/reference-valid state without fixing the underlying issue.
  - Fault reset alone was not enough to restore synchronized truth after this probe. Recovery likely needs a higher-level controller/stack restart or a fresh native-home workflow on `J2`.
  - This result makes a factory/default reset look less attractive as a "safe practice" step, because the softer reset already moves the system away from a usable truth state rather than toward one.

## 2026-04-15 11:24 +0000

- What changed:
  - Reviewed a new manufacturer reply covering:
    - `RM` / "load" meaning
    - `C10.18 / C10.19` vs `6091`
    - the need to re-home after changing the mechanical ratio
    - manual `607C` writes vs HM Method 35
    - negative `607C` interpretation in rotary mode
    - trust-state bits and `0x9650`
    - `C13.10`
    - `F31.10`
  - Updated the active workstream note `docs/ethercat/a6ec-frame-semantics-and-native-home.md` with a new manufacturer-clarification section and explicit integration implications.
  - Updated `.cursor/skills/gradientos-sop/commissioning-safety.md` so the SOP now captures the manufacturer-backed rotary-mode guidance about:
    - `C10.18 / C10.19`
    - re-home-after-ratio-change
    - `607C` manual write insufficiency
    - avoiding negative `607C` in rotary mode
- Validation:
  - Cross-checked the reply against:
    - current workstream note
    - current startup-config code shape
    - manual extracts already in the repo
- Follow-up notes / risks:
  - The manufacturer reply materially strengthens one previously-reverted implementation direction: programming the true mechanical ratio in `C10.18 / C10.19` at startup and then doing one explicit re-home migration.
  - Current code impact is localized rather than architectural: the RTCore startup-config pipeline still exists, but the active A6-EC profile currently emits only `C00.07`, so adding `C10.18 / C10.19` back would mainly be a drive-profile/startup-config extension plus tests.
  - Even with the stronger vendor guidance, this should still be treated as a deliberate migration step, not a blind hotfix, because changing `C10.18 / C10.19` immediately changes the coordinate system and therefore requires a controlled re-home on hardware.

## 2026-04-15 21:20 +0000

- What changed:
  - Extended the A6-EC drive-native migration through the runtime, backend, telemetry, tests, and docs:
    - `src/gradient_os/arm_controller/backends/ethercat_rtcore/runtime.py`
    - `src/gradient_os/arm_controller/backends/ethercat_rtcore/backend.py`
    - `src/gradient_os/telemetry/native_home_status.py`
    - `src/gradient_os/telemetry/drive_faults.py`
    - `tests/test_rtcore_runtime.py`
    - `tests/test_gradient05_limits_and_backends.py`
    - `docs/ethercat/a6ec-frame-semantics-and-native-home.md`
    - `.cursor/skills/gradientos-sop/commissioning-safety.md`
  - Added shared `derive_drive_native_truth_validity(...)` logic so telemetry/backend code use one conservative restart-validity contract for A6-EC drive-native truth.
  - Added explicit `drive_native_ratio_enabled`, startup-validity, coordinate-system-validity, and drive-native-truth fields to the drive fault snapshot and backend truth details.
  - Rebased the backend canonical/display truth path so A6-EC can use the drive reference/output-shaft frame directly, but only when both:
    - startup drive-config verification succeeds
    - the live statusword still carries the vendor-confirmed HM-valid signature with no active alarms
  - Kept the fallback conservative: if startup verification is missing/mismatched or the HM-valid signature is absent, the backend stays on the legacy absolute-anchor reconstruction path.
  - Neutralized host-side double-scaling for the drive-native posture by rendering `GRADIENT_RT_GEAR_RATIO=1` in RTCore startup env for A6-EC while leaving the cold-start Python fallback config conservative until RTCore reports live axis scaling.
  - Added focused regression coverage for:
    - A6-EC drive-native RTCore axis scaling/env rendering
    - drive-native startup/truth status in `build_drive_fault_snapshot(...)`
    - backend direct truth selection once startup verification and `0x9650` are both present
- Validation:
  - `python3 -m py_compile src/gradient_os/arm_controller/backends/ethercat_rtcore/backend.py src/gradient_os/arm_controller/backends/ethercat_rtcore/runtime.py src/gradient_os/telemetry/drive_faults.py src/gradient_os/telemetry/native_home_status.py tests/test_rtcore_runtime.py tests/test_gradient05_limits_and_backends.py`
  - `ReadLints` on the edited Python files returned no diagnostics.
  - Attempted focused pytest slices, but the environment could not run them:
    - `pytest ...` failed because `pytest` is not installed on `PATH`
    - `python3 -m pytest ...` failed because `pytest` is not installed in the system interpreter
    - `uv run --extra dev python -m pytest ...` failed offline with DNS resolution errors while fetching dependencies
- Follow-up notes / risks:
  - The backend now has the intended conservative gate, but full regression confidence still depends on running the focused pytest slices in an environment with the project dev dependencies installed.
  - The startup-validity gate currently rides on the existing primary `startup_drive_config` readback channel; if the UI later needs per-ratio visibility, the telemetry contract may need a richer multi-descriptor summary without breaking current consumers.

## 2026-04-15 21:45 +0000

- What changed:
  - Tightened the A6-EC contract from "drive-native preferred with legacy fallback" to "drive-native only, fail closed when not valid":
    - `src/gradient_os/arm_controller/profiles/drive/a6ec_ds402.py`
    - `src/gradient_os/arm_controller/backends/ethercat_rtcore/backend.py`
    - `src/gradient_os/run_controller.py`
  - Added explicit A6-EC profile flags so the backend knows there is no legacy truth fallback and no absolute-home-anchor requirement for active A6-EC truth.
  - Moved `configured_drive_profile_id` capture earlier in backend init so cold-start axis scaling can honor non-default drive profiles before RTCore hello arrives.
  - Updated the backend/native-home flow so A6-EC:
    - keeps `drive_output_shaft` as the configured truth source
    - reports truth unavailable when startup verification or HM-valid status is missing
    - does not bootstrap or require absolute-home anchors for active A6-EC truth
  - Updated legacy backend tests to opt into an explicit legacy-anchor mode where needed, so non-drive-native coverage still exists without reintroducing fallback to production A6-EC behavior.
  - Updated docs and SOP guidance:
    - `docs/ethercat/a6ec-frame-semantics-and-native-home.md`
    - `.cursor/skills/gradientos-sop/commissioning-safety.md`
- Validation:
  - `source ./start.sh`
  - `python -m pytest tests/test_rtcore_runtime.py tests/test_gradient05_limits_and_backends.py tests/test_drive_faults.py tests/test_run_controller_helpers.py tests/test_terminal_dashboard.py -q`
    - result: `106 passed`
  - Live managed-stack validation:
    - started with `GRADIENT_STACK_INTERACTIVE_CONSOLE=0 ./start-stack.sh`
    - confirmed stack boot completed cleanly and then soft-stopped it with `./start-stack.sh stop`
    - `./start-stack.sh probe` showed runtime/profile selection was live on `a6ec_ds402`
    - `/etc/default/gradient-rt-motion` confirmed:
      - `GRADIENT_RT_GEAR_RATIO="1,1,1,1,1,1"`
      - `GRADIENT_RT_DRIVE_STARTUP_SDO_CONFIG="a6ec_encoder_position_tracking_mode ... ; a6ec_rotation_mode_gear_ratio_numerator ... ; a6ec_rotation_mode_gear_ratio_denominator ..."`
    - live drive-fault snapshot built from `/run/gradient-rt-motion/metrics.json` confirmed:
      - `drive_native_ratio_enabled = true`
      - `drive_native_startup_valid = true` on all 6 axes
      - `drive_native_truth_valid = false` on all 6 axes
      - every axis `statusword_hex = 0x1650`
      - every axis `drive_native_truth_reason = coordinate_system_invalid`
    - `/info/joints-detailed` axis detail matched the same conclusion: the controller is selecting the new drive-native path and failing closed because the coordinate system is not yet HM-valid.
- Follow-up notes / risks:
  - The live blocker is now clearly hardware/commissioning state, not code-path ambiguity: the axes still need a fresh HM35 re-home so the statusword returns to the vendor-confirmed `0x9650` trust state.
  - The current startup-validity witness still comes through the primary `startup_drive_config` readback object; if we later need operator-visible proof of `C10.18/C10.19` specifically, telemetry should gain a richer multi-descriptor summary.

## 2026-04-15 21:54 +0000

- What changed:
  - Removed the last mixed A6-EC fallback semantics from the active implementation/doc surface:
    - `src/gradient_os/arm_controller/profiles/drive/a6ec_ds402.py`
    - `src/gradient_os/arm_controller/backends/ethercat_rtcore/backend.py`
    - `docs/ethercat/a6ec-frame-semantics-and-native-home.md`
  - Dropped the now-misleading `legacy_truth_fallback_enabled` A6-EC profile flag.
  - Simplified backend anchor ownership so active A6-EC behavior is driven only by `absolute_home_anchor_required`:
    - A6-EC no longer loads anchor state into active truth decisions
    - A6-EC software-zero capture no longer refreshes absolute-home anchors
    - non-drive-native profiles still retain anchor-based truth where required
  - Cleaned the active A6-EC work note so it no longer describes any A6-EC fallback/debug truth contract.
- Validation:
  - `source ./start.sh && python -m pytest tests/test_rtcore_runtime.py tests/test_gradient05_limits_and_backends.py tests/test_drive_faults.py tests/test_run_controller_helpers.py tests/test_terminal_dashboard.py -q`
    - result: `106 passed`
  - `ReadLints` on:
    - `src/gradient_os/arm_controller/backends/ethercat_rtcore/backend.py`
    - `src/gradient_os/arm_controller/profiles/drive/a6ec_ds402.py`
    - `docs/ethercat/a6ec-frame-semantics-and-native-home.md`
    - result: no diagnostics
- Follow-up notes / risks:
  - Remaining `fallback` naming in the repo is now outside the active A6-EC truth contract:
    - generic non-A6EC legacy tests
    - unrelated planner/runtime fallback concepts
    - native-home statusword verification fallback logic
  - The A6-EC runtime behavior itself remains fail-closed and still needs a fresh HM35 re-home on hardware to move from `0x1650` to `0x9650`.

## 2026-04-15 22:31 +0000

- What changed:
  - Closed the remaining startup-verification gap in the A6-EC drive-native migration:
    - `src/gradient_rt_motion/main.cpp`
    - `src/gradient_os/arm_controller/profiles/drive/a6ec_ds402.py`
    - `src/gradient_os/arm_controller/backends/ethercat_rtcore/backend.py`
    - `src/gradient_os/telemetry/drive_faults.py`
    - `tests/test_rtcore_runtime.py`
    - `tests/test_gradient05_limits_and_backends.py`
    - `docs/ethercat/a6ec-frame-semantics-and-native-home.md`
    - `.cursor/memory/AGENT_SCRATCHPAD.md`
  - RTCore metrics now publish a richer per-axis `startup_drive_configs` array for all configured startup SDO descriptors while keeping the old primary `startup_drive_config` field for compatibility.
  - RTCore now clears startup-drive-config verification feedback when the startup epoch changes so restart-time truth fails closed instead of inheriting stale verification from the previous epoch.
  - The A6-EC profile extractor now aggregates `C00.07`, `C10.18`, and `C10.19` and only reports startup verification success when all required descriptors are present, readable, and verified.
  - The backend startup-validity gate now uses that aggregated extractor rather than the raw primary metrics field, so A6-EC truth availability no longer turns on without ratio-SDO proof.
  - Added focused regressions for:
    - missing-required-startup-settings fail-closed behavior
    - non-`1:1` A6-EC trajectory upload staying in logical radians while host scaling is neutralized
- Validation:
  - `make -C src/gradient_rt_motion`
  - `source ./start.sh && python -m pytest tests/test_rtcore_runtime.py tests/test_gradient05_limits_and_backends.py tests/test_api_endpoints.py tests/test_encoder_retention.py -q`
    - result: `154 passed`
  - `source ./start.sh && python -m pytest tests/test_rtcore_runtime.py tests/test_gradient05_limits_and_backends.py tests/test_drive_faults.py tests/test_run_controller_helpers.py tests/test_terminal_dashboard.py tests/test_api_endpoints.py tests/test_encoder_retention.py -q`
    - result: `178 passed`
  - `ReadLints` on:
    - `src/gradient_os/arm_controller/backends/ethercat_rtcore/backend.py`
    - `src/gradient_os/arm_controller/profiles/drive/a6ec_ds402.py`
    - `src/gradient_os/telemetry/drive_faults.py`
    - `src/gradient_rt_motion/main.cpp`
    - `tests/test_rtcore_runtime.py`
    - `tests/test_gradient05_limits_and_backends.py`
    - result: no diagnostics
- Follow-up notes / risks:
  - This turn validated build and tests only; it did not repeat a live stack bring-up after the multi-descriptor startup-readback change.
  - The commissioning blocker remains unchanged: hardware still needs a fresh HM35 re-home so statusword moves from `0x1650` to the vendor-valid `0x9650` trust state.

## 2026-04-15 22:45 +0000

- What changed:
  - Executed the live A6-EC validation plan against hardware without changing the attached plan file.
  - Rebuilt RTCore and reran the planned automated regression gate before touching the stack.
  - Brought the stack up in supervised non-interactive mode and confirmed the intended pre-home baseline:
    - `drive_native_startup_valid = true` on all six axes
    - `drive_native_truth_valid = false` on all six axes
    - every axis at `statusword_hex = 0x1650`
  - Ran `POST /control/home-joint-native` on `J6` first and verified the expected per-axis success state:
    - `statusword_hex = 0x9650`
    - `drive_native_truth_valid = true`
    - axis remained disabled after home
  - Found a live workflow constraint in the current controller path: public `SAFE_POWER_UP` stayed blocked with `not_synchronized` until all six axes had been HM35-homed, because the power-transition and `GET_JOINT_STATE` paths still require a complete live joint vector.
  - Completed HM35 on `J1`-`J5`, then ran the full public smoke path:
    - `POST /control/power-up`
    - tiny `POST /control/joint-jog` on `J6` with `delta_deg = 0.2`
    - `POST /control/power-down`
- Validation:
  - `make -C src/gradient_rt_motion`
  - `source ./start.sh && python -m pytest tests/test_rtcore_runtime.py tests/test_gradient05_limits_and_backends.py tests/test_drive_faults.py tests/test_run_controller_helpers.py tests/test_terminal_dashboard.py tests/test_api_endpoints.py tests/test_encoder_retention.py -q`
    - result: `178 passed`
  - Live API / hardware checks:
    - after HM35 on all axes, `/info/joints-detailed` returned `read_source = live_feedback` and `canonical_joint_truth_available = true`
    - tiny `J6` jog succeeded through the public API with observed delta about `+0.205994` deg for a requested `+0.2` deg
    - post-jog `./start-stack.sh probe` returned `BUS_UP_DISARMED`, `0/6` enabled, all axes still `0x9650`, and no faults
- Follow-up notes / risks:
  - The drive-native startup-verification and HM-valid truth contract is now validated live, not just in unit tests.
  - The public API smoke path currently needs the whole arm HM-valid before enable/jog; a single-axis HM35 is not enough to satisfy the existing controller synchronization guard.
  - The optional persistence / restart slice from the plan was not run in this pass.

## 2026-04-15 23:00 +0000

- What changed:
  - Cleaned up the A6-EC frontend pose path after the drive-native truth migration:
    - `src/gradient_os/run_controller.py`
    - `tests/test_run_controller_helpers.py`
    - `web-ui/src/App.tsx`
    - `web-ui/src/TelemetryCharts.tsx`
    - `web-ui/src/poseTelemetry.ts`
    - `web-ui/src/poseTelemetry.test.ts`
  - Fixed the controller monitor SSE contract so `display_joints` is no longer copied from raw canonical `q`; it now comes from the backend display snapshot, matching the same operator-facing truth already exposed by `/info/joints-detailed`.
  - Added focused controller helper regressions to prove monitor payload separation between raw `joints` and operator `display_joints`.
  - Added a tiny shared frontend helper for preferred operator pose selection and switched app/chart consumers to prefer `display_joints` over raw `joints`.
  - Fixed the app fallback pose path so `/info/joints` display feedback is stored in `display_joints` rather than being mislabeled as raw `joints`.
- Validation:
  - `source ./start.sh && python -m pytest tests/test_run_controller_helpers.py -q`
    - result: `9 passed`
  - `npm --prefix web-ui test -- src/poseTelemetry.test.ts src/ControlPanel.test.tsx`
    - result: `21 passed`
  - `ReadLints` on changed controller/frontend files
    - result: no diagnostics
  - Controlled disarmed stack restart, then live payload checks:
    - `/info/joints-detailed.arm_display_deg` showed the expected small operator-facing angles
    - `/monitor` now reports small `display_joints` while raw `joints` remain wrapped on `J3/J4/J6`, proving the streams are separated again
- Follow-up notes / risks:
  - I validated the live payload contract after restart, but I did not run an interactive browser visual pass against the commissioning panel itself.
  - The monitor stream still publishes both raw `joints` and operator `display_joints`; future UI code should keep preferring the display stream unless a view explicitly wants raw canonical motion state.

## 2026-04-15 23:10 +0000

- What changed:
  - Ran a live read-only ratio proof to confirm the A6-EC drives now hold the native mechanical gear ratios in `C10.18 / C10.19`.
  - Captured a full all-axis Chapter 5 probe artifact:
    - `logs/encoder-retention/native-ratio-proof/native-ratio-proof.json`
    - `logs/encoder-retention/native-ratio-proof/native-ratio-proof.md`
  - Cross-checked the direct probe with RTCore startup SDO readback from `/run/gradient-rt-motion/metrics.json`.
- Validation:
  - `source ./start.sh && ./start-stack.sh probe`
    - stack remained `BUS_UP_DISARMED`, `0/6` enabled, all axes `0x9650`, no faults before the read-only probe
  - `source ./start.sh && python scripts/a6ec_chapter5_probe.py snapshot --label native-ratio-proof --axes J1 J2 J3 J4 J5 J6 --experiment-id native-ratio-proof`
    - direct drive readback:
      - `J1/J2/J3`: `C00.07=4`, `C10.18=100`, `C10.19=1`
      - `J4`: `C00.07=4`, `C10.18=18`, `C10.19=1`
      - `J5`: `C00.07=4`, `C10.18=125`, `C10.19=4`
      - `J6`: `C00.07=4`, `C10.18=10`, `C10.19=1`
  - `source ./start.sh && python - <<'PY' ... /run/gradient-rt-motion/metrics.json ... PY`
    - startup SDO metrics readback matched those same commanded values and reported `verified = 1` for:
      - `a6ec_encoder_position_tracking_mode`
      - `a6ec_rotation_mode_gear_ratio_numerator`
      - `a6ec_rotation_mode_gear_ratio_denominator`
- Follow-up notes / risks:
  - This is the right proof for native gearing because it reads the rotary-mode mechanical ratio objects directly; `6091` remained `1:1`, which is expected and should not be confused with the native mechanical ratio path.

## 2026-04-15 23:18 +0000

- What changed:
  - Ran an explicit host-side double-count sanity check after the native ratio proof.
  - Verified the live RTCore env still neutralizes software gear ratios to `1,1,1,1,1,1`.
  - Verified the active A6-EC backend math for a non-`1:1` axis uses neutral counts-per-radian instead of legacy `encoder_counts_per_rev * gear_ratio / (2*pi)`.
- Validation:
  - `ReadFile /etc/default/gradient-rt-motion`
    - confirmed `GRADIENT_RT_GEAR_RATIO="1,1,1,1,1,1"`
  - `source ./start.sh && python -m pytest tests/test_gradient05_limits_and_backends.py -q -k nonunit_a6ec_axis_in_logical_radians`
    - result: `1 passed`
  - `source ./start.sh && python - <<'PY' ... EthercatRTCoreBackend(robot_config=Gradient05Config().get_config_dict()) ... PY`
    - confirmed:
      - `drive_native_ratio_enabled = True`
      - `robot_cfg_j5_gear_ratio = 31.25`
      - `counts_per_unit_j5 = 131072 / (2*pi)` exactly
      - host `counts_per_unit_j5` was not multiplied by `31.25`
- Follow-up notes / risks:
  - This is the expected “no double counting” posture: the drive owns the mechanical ratio in `C10.18/C10.19`, while the host keeps RTCore gear ratio at `1` and commands in logical/reference radians.

## 2026-04-15 23:42 +0000

- What changed:
  - Investigated a fresh live J6 commissioning jog failure from `logs/startups/latest/controller.log`.
  - Confirmed the failure pattern was:
    - `SAFE_POWER_UP`
    - bounded `APPLY_JOINT_SETPOINT` for J6 at `100 Hz` / `25` points
    - open-loop executor thread timing out on `backend.wait_for_trajectory_complete(...)`
    - later `/control/motion-status` returning clean RTCore idle with `last_submitted_traj_id=1`
  - Patched the targeted commissioning jog path so `/control/joint-jog` now includes `target_joint_indices=[joint-1]` in the `APPLY_JOINT_SETPOINT` payload.
  - Updated the controller/executor/RTCore backend path so targeted bounded trajectories are masked to only the selected logical joint axes when offloaded to RTCore.
  - Improved the RTCore timeout message to include the last observed execution snapshot (`state_name`, `active_traj_id`, `queue_depth`, `motion_done`, `active_command_seq`, `submitted_command_seq`) for future live diagnosis.
- Validation:
  - `source ./start.sh && python -m pytest tests/test_api_endpoints.py::test_control_joint_jog tests/test_api_endpoints.py::test_control_joint_jog_ignores_wait_for_idle_flag tests/test_command_api_direct_setpoint.py::test_handle_apply_joint_setpoint_can_start_bounded_joint_trajectory tests/test_gradient05_limits_and_backends.py::test_ethercat_backend_execute_joint_trajectory_uses_quantized_timing tests/test_trajectory_execution_backends.py::test_open_loop_executor_offloads_rtcore_trajectory_backend tests/test_trajectory_execution_backends.py::test_open_loop_executor_passes_targeted_rtcore_axis_mask -q`
    - result: `6 passed`
  - `source ./start.sh && python -m py_compile src/gradient_os/api/main.py src/gradient_os/run_controller.py src/gradient_os/arm_controller/command_api.py src/gradient_os/arm_controller/trajectory_execution.py src/gradient_os/arm_controller/backends/ethercat_rtcore/backend.py tests/test_api_endpoints.py tests/test_command_api_direct_setpoint.py tests/test_gradient05_limits_and_backends.py tests/test_trajectory_execution_backends.py`
    - result: success
  - `ReadLints` on the touched files
    - result: no diagnostics
  - Read-only live checks during investigation:
    - `GET /control/motion-status` showed RTCore idle, `queue_depth=0`, `motion_done=true`, `last_submitted_traj_id=1`
    - `ReadFile /run/gradient-rt-motion/metrics.json` showed all axes still `0x9650`, startup verified, no faults, disarmed after stop/power-down
- Follow-up notes / risks:
  - I did not re-run a live powered J6 jog after the patch because that would move hardware.
  - If the next live retry still times out, the new timeout payload should tell us whether the residual issue is completion bookkeeping or a deeper seam/command-path problem on the selected axis itself.

## 2026-04-15 23:56 +0000

- What changed:
  - Restarted the live stack after the user's hard stop and drive power cycle, then investigated the reported commissioning-angle flicker with read-only runtime checks before any new motion.
  - Confirmed the restarted stack came up healthy after one launcher-managed RTCore/EtherCAT recycle and landed in the expected fail-closed post-power-cycle state:
    - `safe_for_power_transition=false`
    - blocker `not_synchronized`
    - `/info/joints-detailed` returned `arm_deg=[]`, `arm_display_deg=[]`
    - `/run/gradient-rt-motion/metrics.json` showed all six axes at `0x1650` with startup verification still intact and no faults
  - Quantified read-only rest jitter from repeated `/info/joints-detailed` sampling:
    - raw/reference counts moved only about `1..4` counts per axis
    - display/reference-angle wander was about `0.0082..0.0110 deg`
    - the current post-power-cycle read-only run did not reproduce the earlier `~0.1 deg` J6 flicker
  - Identified the main UI amplification factor in `web-ui/src/ControlPanel.tsx`:
    - commissioning angles updated for changes above `0.001 deg`
    - labels rendered at `0.01 deg` precision
    - normal count-level rest jitter therefore showed up as visible last-digit chatter
  - Implemented a display-only stabilization path in `web-ui/src/ControlPanel.tsx`:
    - added a `0.02 deg` deadband for idle/disarmed EtherCAT commissioning angles
    - left controller/API telemetry and motion semantics unchanged
  - Added a focused regression in `web-ui/src/ControlPanel.test.tsx` to prove the panel stays steady through sub-deadband rest jitter but still updates on a larger change.
- Validation:
  - `source ./start.sh && ./start-stack.sh probe`
    - result: stack fully down before restart (`launcher_state: absent`, controller/API down, RTCore down)
  - `source ./start.sh && python -m pytest tests/test_api_endpoints.py::test_control_joint_jog tests/test_api_endpoints.py::test_control_joint_jog_ignores_wait_for_idle_flag tests/test_command_api_direct_setpoint.py::test_handle_apply_joint_setpoint_can_start_bounded_joint_trajectory tests/test_gradient05_limits_and_backends.py::test_ethercat_backend_execute_joint_trajectory_uses_quantized_timing tests/test_run_controller_helpers.py tests/test_trajectory_execution_backends.py::test_open_loop_executor_offloads_rtcore_trajectory_backend tests/test_trajectory_execution_backends.py::test_open_loop_executor_passes_targeted_rtcore_axis_mask -q`
    - result: `15 passed`
  - `source ./start.sh && GRADIENT_STACK_INTERACTIVE_CONSOLE=0 ./start-stack.sh`
    - result: stack online; latest run `20260415-235055`
  - live runtime checks:
    - `curl -s http://127.0.0.1:4000/control/motion-status`
    - `curl -s http://127.0.0.1:4000/info/joints-detailed`
    - repeated `python` polling against `/info/joints-detailed`
    - decoded `/monitor` SSE payload shape
  - `npm --prefix web-ui test -- src/ControlPanel.test.tsx src/poseTelemetry.test.ts`
    - result: `22 passed`
  - `ReadLints` on `web-ui/src/ControlPanel.tsx` and `web-ui/src/ControlPanel.test.tsx`
    - result: no diagnostics
- Follow-up notes / risks:
  - The new UI deadband addresses the current evidence-backed issue: the panel visibly amplifying harmless disarmed rest jitter.
  - Because the power cycle reset the axes back to `0x1650`, this pass did not reproduce the earlier post-home `~0.1 deg` J6 wobble. If that larger wobble returns after a fresh HM35/home-valid state, it still needs a separate post-home investigation.
  - I intentionally did not issue a new home or jog command in this pass, so there was no additional hardware motion.

## 2026-04-16 00:15 +0000

- What changed:
  - Revisited the A6-EC flicker investigation after the user correctly challenged the neutral-scaling assumption.
  - Proved read-only that the prior repo-wide A6-EC scaling posture was inconsistent with live drive objects:
    - J6 probe showed `6064 = U40.16 = U40.28 = 1310650`
    - `C10.18/C10.19 = 10/1`
    - the old host posture still forced neutral `GRADIENT_RT_GEAR_RATIO=1` and neutral backend `counts_per_unit`
  - Removed the temporary commissioning-panel deadband from `web-ui/src/ControlPanel.tsx` because it was only a presentation workaround and would mask validation of the real fix.
  - Restored physical A6-EC scaling at the actual backend/runtime layer:
    - `src/gradient_os/arm_controller/backends/ethercat_rtcore/runtime.py`
      - RTCore env generation now renders mechanical gear ratios for A6-EC instead of forcing `1`
    - `src/gradient_os/arm_controller/backends/ethercat_rtcore/backend.py`
      - backend robot axis config now uses physical `actuator_counts_per_radian` again instead of forcing neutral counts-per-radian for drive-native A6-EC
  - Updated regressions:
    - `tests/test_rtcore_runtime.py`
    - `tests/test_gradient05_limits_and_backends.py`
    - `web-ui/src/ControlPanel.test.tsx`
  - Added a direct J6 regression proving the same `1310650` feedback sample now decodes to `0.019226... deg`, not `0.19226... deg`.
- Validation:
  - focused runtime/backend regressions
    - `source ./start.sh && python -m pytest tests/test_rtcore_runtime.py -q tests/test_gradient05_limits_and_backends.py -q -k 'build_rtcore_axis_scaling_uses_drive_native_ratio_for_a6ec or render_rtcore_systemd_env_contains_scaling_and_profile or enqueue_trajectory_points_keeps_nonunit_a6ec_axis_in_logical_radians or j6_display_feedback_uses_rotation_mode_ratio_scaling'`
    - result: `4 passed`
  - frontend regressions after removing the deadband
    - `npm --prefix web-ui test -- src/ControlPanel.test.tsx src/poseTelemetry.test.ts`
    - result: `21 passed`
  - broader controller/runtime/trajectory slice
    - `source ./start.sh && python -m pytest tests/test_api_endpoints.py::test_control_joint_jog tests/test_api_endpoints.py::test_control_joint_jog_ignores_wait_for_idle_flag tests/test_command_api_direct_setpoint.py::test_handle_apply_joint_setpoint_can_start_bounded_joint_trajectory tests/test_gradient05_limits_and_backends.py tests/test_rtcore_runtime.py tests/test_trajectory_execution_backends.py::test_open_loop_executor_offloads_rtcore_trajectory_backend tests/test_trajectory_execution_backends.py::test_open_loop_executor_passes_targeted_rtcore_axis_mask -q`
    - result: `90 passed`
  - `ReadLints` on touched runtime/backend/test/frontend files
    - result: no diagnostics
  - direct code-level sanity check
    - patched backend J6 sample `1310650` now yields:
      - wrap period `1310720`
      - decoded angle `0.01922607421875 deg`
  - live restart and read-only verification
    - `source ./start.sh && ./start-stack.sh stop --hard`
    - `source ./start.sh && GRADIENT_STACK_INTERACTIVE_CONSOLE=0 ./start-stack.sh`
    - `/etc/default/gradient-rt-motion` now contains `GRADIENT_RT_GEAR_RATIO="100,100,100,18,31.25,10"`
    - live `/info/joints-detailed` now reports J6 `reference_pre_zero_deg ~= 0.0189514`
    - repeated J6 read-only sampling shows jitter range about `0.0010986 deg`
- Follow-up notes / risks:
  - The stack is still correctly blocked from power-up because the user’s power cycle returned all axes to `0x1650` / `not_synchronized`; this pass did not re-home or power up.
  - This turn corrected the host-side scaling assumption with read-only live proof, but it did not yet re-run a home-valid powered jog on hardware.

## 2026-04-16 00:18 +0000 - Verified per-axis A6-EC count totals are now correct

- Context:
  - The user clarified the required invariant: each axis must use its own mechanical gear ratio, so one output-shaft rotation equals `2^17 * gear_ratio[j]` counts for that axis.
  - This was a read-only verification pass after the earlier scaling correction.
- Validation that ran:
  - `source ./start.sh && python - <<'PY' ... PY`
  - The script compared four sources for all six axes:
    - `actuator_encoder_counts_per_rev`
    - `actuator_gear_ratios`
    - `build_rtcore_axis_scaling(..., drive_profile="a6ec_ds402")`
    - `EthercatRTCoreBackend(...)._reference_wrap_period_counts_for_axis(axis)`
    - `/etc/default/gradient-rt-motion` `GRADIENT_RT_GEAR_RATIO`
- Result:
  - all six axes passed
  - expected and actual wrap periods matched exactly:
    - J1 `13107200`
    - J2 `13107200`
    - J3 `13107200`
    - J4 `2359296`
    - J5 `4096000`
    - J6 `1310720`
  - `GRADIENT_RT_GEAR_RATIO` also matches per-axis robot config values exactly: `100,100,100,18,31.25,10`
- Follow-up implementation:
  - added `test_ethercat_backend_a6ec_reference_wrap_period_uses_per_axis_gear_ratios` to `tests/test_gradient05_limits_and_backends.py`
  - ran focused regressions:
    - `tests/test_rtcore_runtime.py::test_build_rtcore_axis_scaling_uses_drive_native_ratio_for_a6ec`
    - `tests/test_rtcore_runtime.py::test_render_rtcore_systemd_env_contains_scaling_and_profile`
    - `tests/test_gradient05_limits_and_backends.py::test_ethercat_backend_j6_display_feedback_uses_rotation_mode_ratio_scaling`
    - `tests/test_gradient05_limits_and_backends.py::test_ethercat_backend_a6ec_reference_wrap_period_uses_per_axis_gear_ratios`
  - result: `4 passed`
- Lint check:
  - `ReadLints` on `tests/test_gradient05_limits_and_backends.py`
  - result: no diagnostics
- Notes:
  - J5's fractional ratio `31.25` still yields an exact integer period because `131072 * 31.25 = 4096000`.
  - This started as a read-only verification pass, then turned into a test hardening pass so the earlier scaling fix now has an explicit all-axis regression guard, not just the prior J6-specific check.

## 2026-04-16 00:37 +0000 - Surfaced the real post-power-cycle blocker: coordinate system invalid

- Context:
  - The user reported they were still blocked from power-up and wanted the live failure explained and fixed properly.
  - Live logs/endpoints showed the block was real, but the software was collapsing it to a generic `not_synchronized` message.
- Code changes:
  - `src/gradient_os/arm_controller/backends/ethercat_rtcore/backend.py`
    - `get_power_transition_snapshot()` now carries truth-availability diagnostics (`feedback_truth_reasons`, unavailable axes/joints, statusword summary) instead of swallowing canonical-truth failure into an empty joint list.
    - canonical-truth exceptions now include summarized reasons and statuswords, e.g. `drive_native_coordinate_system_invalid` and `0x1650`.
  - `src/gradient_os/arm_controller/command_api.py`
    - power-transition guard now maps backend truth failures into specific blockers:
      - `coordinate_system_invalid` when drive-native truth is invalid
      - `canonical_truth_unavailable` for other truth-unavailable cases
    - keeps the generic `not_synchronized` blocker only for truly reasonless sync gaps.
  - `web-ui/src/ControlPanel.tsx`
    - runtime header now renders a specific operator message for `coordinate_system_invalid` instead of treating it like an ordinary sync-settle check.
- Tests added/updated:
  - `tests/test_gradient05_limits_and_backends.py`
    - added `test_ethercat_backend_power_transition_snapshot_reports_coordinate_system_invalid`
  - `tests/test_command_api_direct_setpoint.py`
    - added `test_build_power_transition_guard_surfaces_coordinate_system_invalid_blocker`
  - `web-ui/src/ControlPanel.test.tsx`
    - added runtime-header coverage for the blocked/native-home-required state
- Validation that ran:
  - focused backend/controller regressions:
    - `tests/test_gradient05_limits_and_backends.py::test_ethercat_backend_power_transition_snapshot_reports_coordinate_system_invalid`
    - `tests/test_gradient05_limits_and_backends.py::test_ethercat_backend_j6_display_feedback_uses_rotation_mode_ratio_scaling`
    - `tests/test_command_api_direct_setpoint.py::test_build_power_transition_guard_surfaces_coordinate_system_invalid_blocker`
    - `tests/test_rtcore_runtime.py::test_build_rtcore_axis_scaling_uses_drive_native_ratio_for_a6ec`
    - result: `4 passed`
  - focused power-up/API regressions:
    - `tests/test_api_endpoints.py::test_control_power_up`
    - `tests/test_api_endpoints.py::test_control_power_up_returns_conflict_when_safety_gate_blocks`
    - `tests/test_command_api_direct_setpoint.py::test_handle_safe_power_up_rejects_when_runtime_is_not_safe`
    - result: `3 passed`
  - frontend regression:
    - `npm --prefix web-ui test -- src/ControlPanel.test.tsx`
    - result: `19 passed`
  - `ReadLints` on touched backend/command-api/frontend/test files
    - result: no diagnostics
- Runtime verification:
  - restarted the stack onto the patched code (`logs/startups/20260416-003638`)
  - live `GET /control/motion-status` now reports:
    - `power_transition_blockers=["coordinate_system_invalid"]`
    - `truth_unavailable_joints=[1,2,3,4,5,6]`
    - `statuswords=["0x1650"]`
    - `requires_native_home=true`
  - live `GET /info/joints-detailed` now reports:
    - `canonical_joint_truth_error="Canonical joint truth unavailable (... reasons=['drive_native_coordinate_system_invalid'], statuswords=['0x1650'])"`
  - controller log now records the same reason explicitly.
- Follow-up / risk:
  - The software diagnosis is fixed, but the physical unblock has not been executed in this pass.
  - All six axes still need a clean native-home/HM35 cycle to return to the vendor-valid `0x9650` state before power-up can legitimately succeed again.

## 2026-04-16 00:44 +0000 - Unblocked Drive Home when canonical truth is unavailable

- Context:
  - After surfacing the real `coordinate_system_invalid` blocker, the user correctly reported they were still blocked because the commissioning UI had disabled every `Drive Home` button.
  - This was a UI deadlock: the recovery action was gated by the very truth signal that native home is supposed to restore.
- Code changes:
  - `web-ui/src/ControlPanel.tsx`
    - decoupled `Drive Home` button enablement from `zeroDisabled`
    - native-home now stays enabled when per-axis drive telemetry is present, even if `arm_display_deg`/canonical display truth is unavailable
    - tooltip now explains the live-drive-telemetry fallback
    - confirmation dialog fallback text now says `current live drive feedback`
- Validation that ran:
  - `npm --prefix web-ui test -- src/ControlPanel.test.tsx`
  - result: `20 passed`
  - `ReadLints` on `web-ui/src/ControlPanel.tsx` and `web-ui/src/ControlPanel.test.tsx`
  - result: no diagnostics
  - live dev server verification:
    - running stack Vite log reported `hmr update /src/ControlPanel.tsx, /src/index.css`
- Tests added/updated:
  - `web-ui/src/ControlPanel.test.tsx`
    - added coverage for the exact recovery state: canonical angles unavailable, live `0x1650` drive telemetry present, `Drive Home` buttons remain enabled
- Follow-up / risk:
  - This fixes the software deadlock in the commissioning panel.
  - The physical recovery still requires actually running native-home/HM35 on the affected axes; this pass did not issue those hardware commands.

## 2026-04-16 01:09 +0000 - Stabilized the commissioning message rail so live status updates stop moving the joint controls

- Context:
  - The user reported that the live message labels above the commissioning controls were flickering in and out and making the whole panel jump, which made the buttons difficult to click.
- Code changes:
  - `web-ui/src/ControlPanel.tsx`
    - replaced the conditional commissioning banner stack with a fixed three-slot message rail
    - each slot now keeps a constant `h-9` height and empty slots remain `invisible` so the surrounding panel height stays stable
    - long messages are clamped inside the slot instead of resizing the container
  - `web-ui/src/ControlPanel.test.tsx`
    - added a regression that asserts the commissioning rail always renders three fixed slots while a live native-home status banner is active
- Validation that ran:
  - `npm --prefix web-ui test -- src/ControlPanel.test.tsx`
  - result: `21 passed`
  - `ReadLints` on `web-ui/src/ControlPanel.tsx` and `web-ui/src/ControlPanel.test.tsx`
  - result: no diagnostics
- Follow-up / risk:
  - The UI now stays stable for clicking, but the fixed-height rail intentionally clips longer messages to preserve panel stability.

## 2026-04-16 01:36 +0000 - Separated J5 false-failure from J1 transient post-home fault during native-home log review

- Context:
  - The user asked why `J5` and `J1` showed native-home errors even though both appeared okay after a fault reset.
- Investigation performed:
  - reviewed the active controller terminal output for the current startup run
  - checked live `/control/motion-status` and `/info/joints-detailed`
  - read raw `/run/gradient-rt-motion/metrics.json`
  - re-read the native-home verification and post-home refresh logic in `src/gradient_os/arm_controller/backends/ethercat_rtcore/backend.py`
  - re-read `src/gradient_os/telemetry/native_home_status.py` to confirm how live telemetry upgrades stale failed metrics to a clean `0x9650` success
- Findings:
  - `J5` logged `native drive-home failed verification ... abort=0x06010002`
  - current raw RTCore metrics still show `J5 native_home_state=3` and abort `0x06010002`, while the same axis also shows clean `statusword=0x9650` and zero live faults
  - current live `/info/joints-detailed` therefore marks `J5` as succeeded via `native_home_verification_source=statusword_bits12_15_clear13`
  - `J1` logged `native drive-home reached a verified terminal state, but post-home anchor refresh failed ... (drive_native_fault_present)`
  - a `RESET_FAULTS` command was issued immediately after the `J1` warning, and canonical truth became available again
  - current raw metrics for `J1` are already clean (`native_home_state=2`, abort `0`), so `J1` was not the stale-`0x06010002` path
- Interpretation:
  - `J5` is the known false-failure/result-contract bug family where stale raw native-home metrics contradict the clean live drive wire-state
  - `J1` was a separate transient live fault/anchor-refresh failure after the home had already verified; the reset cleared that transient condition
- Validation that ran:
  - `python` fetch of live `/control/motion-status` and `/info/joints-detailed`
  - `python` probe printing live native-home fields for `J1` and `J5`
  - `ReadFile` on `/run/gradient-rt-motion/metrics.json`
- Follow-up / risk:
  - no code changes in this pass
  - the current controller warning for the `J1` path still lacks the exact transient `statusword`/`error_code`; reproducing with richer logging would be needed if we want the precise drive fault identity

## 2026-04-16 01:55 +0000 - Reduced recurring native-home false signals in backend wait logic, post-home validation, and UI row copy

- Context:
  - The user explicitly called out that the same native-home false signals kept recurring and needed to be fixed rather than re-explained.
- Code changes:
  - `src/gradient_os/arm_controller/backends/ethercat_rtcore/backend.py`
    - added a failed-result stabilization rule in `_wait_for_native_home_result()` so a single bad post-clear snapshot no longer latches a hard failure before the clean `0x9650` success snapshot arrives
    - added a targeted retry path in `native_home_joint()` for transient post-home truth failures like `drive_native_fault_present`, using the existing post-home settle window before returning `NATIVE_HOME_ANCHOR_REFRESH_FAILED`
  - `tests/test_gradient05_limits_and_backends.py`
    - added a regression for the J1-style transient post-home truth failure that now recovers after settle
    - added a regression for the J5-style single failed post-clear snapshot that now recovers when a clean live-success snapshot follows
    - updated the direct failed-after-clear regression so it still asserts hard failure only after two failed post-clear snapshots
  - `web-ui/src/ControlPanel.tsx`
    - changed per-joint native-home status text so a clean live `succeeded` state is shown as success even if stale reported abort metadata still exists underneath
  - `web-ui/src/ControlPanel.test.tsx`
    - updated the UI regression to assert success for that clean-live/stale-reported case
- Validation that ran:
  - `pytest tests/test_gradient05_limits_and_backends.py -k "native_home or wait_for_native_home_result"`
  - result: `22 passed`
  - `npm --prefix web-ui test -- src/ControlPanel.test.tsx`
  - result: `21 passed`
  - `ReadLints` on touched backend/test/frontend files
  - result: no diagnostics
- Follow-up / risk:
  - this fixes the code paths, but the already-running controller process is still using the pre-patch Python backend until the stack is restarted
  - raw RTCore metrics can still retain stale reported native-home failure fields; this pass stops those raw fields from dominating operator-facing status, but it does not yet redesign the RTCore metrics contract itself

## 2026-04-16 01:50 +0000 - Investigated J5 jog timeout after targeted-axis masking was already present

- Context:
  - The user asked why the current jogging attempt was failing with `TimeoutError: Timed out waiting for RTCore trajectory 4 to complete`.
- Investigation performed:
  - read the active terminal excerpt plus `logs/startups/20260416-012149/controller.log`
  - re-read the bounded jog/offloaded trajectory path in `src/gradient_os/arm_controller/command_api.py`, `src/gradient_os/arm_controller/trajectory_execution.py`, and `src/gradient_os/arm_controller/backends/ethercat_rtcore/backend.py`
  - re-read RTCore trajectory-completion logic in `src/gradient_rt_motion/main.cpp`
  - read raw `/run/gradient-rt-motion/metrics.json`
  - fetched live `curl -s http://127.0.0.1:4000/control/motion-status`
- Findings:
  - the failing bounded move was a J5 seam-crossing jog from `-0.048 deg` to `+0.952 deg`
  - the request already carried `target_joint_indices=[4]`, so the older "all held axes must satisfy completion" bug was not the direct cause of this timeout
  - the timeout snapshot was `saw_target=True state=executing active_traj_id=4 queue_depth=0 motion_done=False active_command_seq=133 submitted_command_seq=133`
  - after the timeout, the controller log showed `SAFE_POWER_DOWN,wait`, `Received STOP command`, and `WAIT_FOR_IDLE finished with state: completed`
  - live `/control/motion-status` after recovery showed RTCore idle with `last_submitted_traj_id=4`
  - raw `/run/gradient-rt-motion/metrics.json` after the run showed J5 `rotation_mode_position_reference=4085206` on the `31.25` ratio axis whose full wrap period is `4096000` counts, plus a separate J6 fault state (`statusword=0x9618`, `error_code=65280`)
- Interpretation:
  - this failure is later than the API ACK path: RTCore accepted the selected-axis trajectory, consumed the uploaded points, but did not satisfy the completion condition before the backend wait expired
  - strongest code-level suspicion to verify next: RTCore completion currently reduces wrapped final error with raw `counts_per_rev`, while the backend/Python A6-EC reference logic already uses the full wrapped period implied by `counts_per_unit * 2*pi`
- Validation that ran:
  - `ReadFile` on the current controller log and `/run/gradient-rt-motion/metrics.json`
  - `curl -s http://127.0.0.1:4000/control/motion-status`
- Follow-up / risk:
  - no code changes in this pass
  - the wrap-period mismatch is still a hypothesis until we capture/lock the exact J5 seam-crossing error counts in a focused repro or regression

## 2026-04-16 02:53 +0000 - Aligned RTCore wrapped completion with geared A6-EC rotary period

- Context:
  - Implemented the approved fix for the J5 seam-crossing jog timeout where RTCore consumed the queued points but stayed `executing` with `queue_depth=0` and `motion_done=false`.
- Code changes:
  - `src/gradient_rt_motion/main.cpp`
    - renamed `shortest_periodic_error_counts()` parameter to `period_counts` for clarity
    - added `completion_wrap_period_counts(const AxisConfig&)` to derive the wrapped rotary completion period from `counts_per_unit * 2*pi`, with `counts_per_rev` fallback
    - updated the trajectory-completion block to use that derived period instead of raw `counts_per_rev`
  - `tests/test_gradient05_limits_and_backends.py`
    - strengthened the A6-EC wrap-period regression with an explicit J5 `4096000`-count assertion
    - added `test_ethercat_backend_execute_joint_trajectory_uses_quantized_timing_for_j5_axis_mask`
    - added `test_ethercat_backend_wait_for_trajectory_complete_waits_past_queue_empty_executing_snapshot`
- Validation that ran:
  - `make -C src/gradient_rt_motion`
    - result: success
  - `source /home/pi/GradientOS/.venv/bin/activate && PYTHONPATH=src python -m pytest tests/test_gradient05_limits_and_backends.py::test_ethercat_backend_a6ec_reference_wrap_period_uses_per_axis_gear_ratios tests/test_gradient05_limits_and_backends.py::test_ethercat_backend_execute_joint_trajectory_uses_quantized_timing tests/test_gradient05_limits_and_backends.py::test_ethercat_backend_execute_joint_trajectory_uses_quantized_timing_for_j5_axis_mask tests/test_gradient05_limits_and_backends.py::test_ethercat_backend_wait_for_trajectory_complete_ignores_stale_previous_completion tests/test_gradient05_limits_and_backends.py::test_ethercat_backend_wait_for_short_trajectory_completion_without_observed_active_id tests/test_gradient05_limits_and_backends.py::test_ethercat_backend_wait_for_trajectory_complete_waits_past_queue_empty_executing_snapshot -q`
    - result: `6 passed`
  - `ReadLints` on `src/gradient_rt_motion/main.cpp` and `tests/test_gradient05_limits_and_backends.py`
    - result: no diagnostics
- Follow-up / risk:
  - live hardware confirmation still remained to be run after this code-only pass

## 2026-04-16 03:11 +0000 - Live J5 seam-crossing jog completed cleanly on the patched RTCore stack

- Context:
  - The user asked to test the RTCore seam-completion fix live rather than stopping at build/test validation.
- Runtime actions:
  - observed that the previously running stack had been hard-stopped from the interactive launcher, so the stack was down
  - restarted the stack with `./start-stack.sh` onto the patched RTCore binary (`logs/startups/20260416-030940`)
  - verified preflight state:
    - `/control/motion-status` returned `safe_for_power_transition=true`
    - `/info/joints-detailed` showed canonical/display truth available and J5 `arm_display_deg ~= +0.9484`
  - issued `POST /control/power-up`
  - issued `POST /control/joint-jog {"joint":5,"delta_deg":-1.0}` to cross the J5 seam from about `+0.9316 deg` display to about `-0.0683 deg`
  - issued `POST /control/power-down` after the verification
- Findings:
  - the controller log showed:
    - `APPLY_JOINT_SETPOINT bounded move ... target_deg=[..., -360.068, ...]`
    - `[Pi OL] RTCore trajectory execution finished: state=completed traj_id=1 elapsed=0.349s`
    - no `TimeoutError`
  - post-jog `/control/motion-status` showed:
    - `state=completed`
    - `active_traj_id=1`
    - `queue_depth=0`
    - `motion_done=true`
    - `last_event_code=291`
  - post-jog `/info/joints-detailed` showed J5 had crossed the seam successfully to `arm_display_deg ~= -0.0683`
  - power-down response returned RTCore to `idle` with `safe_for_power_transition=true`
- Validation that ran:
  - live stack restart via `./start-stack.sh`
  - `curl -s http://127.0.0.1:4000/control/motion-status`
  - `curl -s http://127.0.0.1:4000/info/joints-detailed`
  - `curl -s -X POST http://127.0.0.1:4000/control/power-up`
  - `curl -s -X POST http://127.0.0.1:4000/control/joint-jog -H 'Content-Type: application/json' -d '{"joint":5,"delta_deg":-1.0}'`
  - `curl -s -X POST http://127.0.0.1:4000/control/power-down`
- Follow-up / risk:
  - the targeted live J5 seam-crossing repro is now good evidence that the specific timeout regression is fixed
  - broader motion coverage across other seam-adjacent axes or larger moves was not exercised in this pass

## 2026-04-16 03:58 +0000 - Investigated post-power-cycle trust gating and the new transient J5 Er87.1 fault

- Context:
  - After a hard restart and drive power cycle, the user reported being locked out from power-up with no trusted telemetry, then asked whether our code was incorrectly demanding `0x9650` at startup instead of following the manufacturer restart rule that only calls for `6041 bit 15 = 1`.
  - The user also reported a fresh J5 `Er87.1` fault and asked why.
- Findings:
  - `src/gradient_os/arm_controller/profiles/drive/a6ec_ds402.py` currently sets `startup_truth_requires_hm_success_signature = True`.
  - `src/gradient_os/telemetry/native_home_status.py` currently treats drive-native truth as valid only when the live statusword has bits 12 and 15 set, bit 13 clear, and no active `error_code` / `manufacturer_error_code`.
  - The gate is therefore stricter than the vendor restart note, but it is not a literal exact-`0x9650` check; any clean signature-carrying state (for example a powered-up homed state) can pass.
  - The latest lockout still reflected a genuine drive-side invalid coordinate indication: live probe/API output showed all axes at `0x1650`, so `bit 15` was actually low. Relaxing the software rule to "bit15 only" would not have fixed that specific observed state.
  - `docs/resources/a6ec_manual_codes.md` maps `Er87.1` to "One-time excessive position reference increment (One-time increment of the target position is over 5 times of the maximum speed)".
  - `logs/startups/20260416-034141/controller.log` shows `NATIVE_HOME_JOINT,5` reached a verified terminal state and then faulted during the post-home settle window, which points to a transient post-home reference jump rather than the earlier RTCore seam-completion timeout.
- Follow-up / risk:
  - There is likely still a product decision to make about separating strict HM-success verification from restart-persistence verification so startup trust can align with the vendor bit-15 guidance.
  - The immediate lockout after the reported power cycle was still rooted in the live drive state not advertising retained coordinate validity.
  - The J5 `Er87.1` needs fresh live capture at the instant it happens if we want to prove whether the jump occurs on restore-to-CSP, hold-target resync, or another post-home handoff step.

## 2026-04-16 04:21 +0000 - Relaxed A6-EC startup trust to accept retained bit-15 coordinate validity

- Context:
  - The user asked to do the startup-truth change first so A6-EC restart validation matches the manufacturer guidance more closely without weakening fresh HM35 verification.
- Code changes:
  - `src/gradient_os/telemetry/native_home_status.py`
    - kept `statusword_indicates_valid_native_home_reference()` strict for HM-success verification
    - added a separate statusword-to-coordinate-validity path so drive-native truth can accept either the strict HM signature or a relaxed bit-15-only startup rule, depending on profile config
    - extended `derive_drive_native_truth_validity()` with `require_hm_success_signature`
  - `src/gradient_os/arm_controller/backends/ethercat_rtcore/backend.py`
    - added `_startup_truth_requires_hm_success_signature()`
    - passed the profile-controlled startup-truth rule into drive-native truth evaluation
  - `src/gradient_os/telemetry/drive_faults.py`
    - threaded the same position-semantics flag into the probe/runtime drive-fault snapshot path
  - `src/gradient_os/arm_controller/profiles/drive/a6ec_ds402.py`
    - set `startup_truth_requires_hm_success_signature = False` with a comment tying that choice to the vendor restart guidance
  - `tests/test_rtcore_runtime.py`
    - added a regression proving `build_drive_fault_snapshot()` accepts A6-EC `0x8650` as valid retained startup truth while still marking the strict HM signature as absent
  - `tests/test_gradient05_limits_and_backends.py`
    - added a regression proving `EthercatRTCoreBackend.get_power_transition_snapshot()` now treats `0x8650` as synchronized/valid feedback for A6-EC startup
- Validation that ran:
  - `source "/home/pi/GradientOS/.venv/bin/activate" && python -m py_compile src/gradient_os/telemetry/native_home_status.py src/gradient_os/telemetry/drive_faults.py src/gradient_os/arm_controller/backends/ethercat_rtcore/backend.py src/gradient_os/arm_controller/profiles/drive/a6ec_ds402.py`
    - result: success
  - `source "/home/pi/GradientOS/.venv/bin/activate" && PYTHONPATH=src python -m pytest tests/test_drive_faults.py::test_statusword_indicates_valid_native_home_reference_requires_vendor_success_bits tests/test_gradient05_limits_and_backends.py::test_native_home_metrics_result_requires_bit12_alongside_bit15_for_fallback tests/test_gradient05_limits_and_backends.py::test_ethercat_backend_power_transition_snapshot_reports_coordinate_system_invalid tests/test_gradient05_limits_and_backends.py::test_ethercat_backend_power_transition_snapshot_accepts_bit15_restart_truth tests/test_rtcore_runtime.py::test_drive_fault_snapshot_marks_drive_native_truth_valid_when_startup_and_status_are_valid tests/test_rtcore_runtime.py::test_drive_fault_snapshot_accepts_a6ec_bit15_only_restart_truth -q`
    - result: `6 passed`
  - `ReadLints` on touched Python/test files
    - result: no diagnostics
- Follow-up / risk:
  - This change intentionally does not fix states where the drive still comes back as `0x1650`; those remain true drive-side invalid-coordinate cases that still require re-home/recovery.
  - The separate J5 `Er87.1` post-home settle fault still needs its own investigation.

## 2026-04-16 04:32 +0000 - Investigated new J3 Er47.0 jog fault and the confusing `-359 deg` raw-angle logs

- Context:
  - The user reported that J3 moved erratically in the opposite direction during jogging, then faulted with `Er47.0`, and asked why many current positions in the logs showed values near `-359 deg` instead of near zero.
- Findings:
  - The failing sequence in the live controller log shows:
    - one J3 seam-adjacent jog completed cleanly (`target_deg` about `-360.979`)
    - the next J3 jog was baselined from `current_deg` about `-0.980` and targeted `-1.980`
    - that second jog faulted in the background executor, after the API had already acknowledged the request
  - Live post-fault API state now shows:
    - `/control/motion-status`: `fault_present` and `canonical_truth_unavailable` on axis 2 / joint 3
    - `/info/joints-detailed`: J3 `statusword=0xB638`, `error_code=34321 (0x8611 / Er47.0)`, truth unavailable because of the active drive fault
  - `docs/resources/a6ec_manual_codes.md` confirms `Er47.0` is `Excessive position deviation`, and the manual parameter list ties that to `6062 - 6064` exceeding the following-error window/time.
  - The zero-offset store `.gradient_joint_zero_offsets.json` still contains all-zero logical master offsets, so the `-359 deg` numbers are not caused by stale software zeroing.
  - The `-359`/`355` values come from the raw command/reference frame, not the operator display frame:
    - `servo_driver.get_current_arm_state_rad()` calls backend `get_joint_positions()`
    - `/control/joint-jog` baselines the next target from `arm_deg`
    - `web-ui/src/ControlPanel.tsx` still prefers `arm_display_deg` for presentation, which is why the UI can be near zero while the controller logs show seam-equivalent raw angles
  - Current live data confirms the split:
    - healthy joints still report operator display angles near zero or a few degrees
    - the raw command frame can differ by whole turns (for example J2 display about `-4.935 deg` while prior jog logs printed `355.063 deg`)
  - The post-fault J3 raw reference landed around `+12.3 deg` while the failing second jog target was `-1.98 deg`, so the axis ended up on the wrong side of the command by roughly `14 deg` before tripping the following fault.
- Interpretation:
  - This is most consistent with the old persistent J3/J4 commissioning bug family: seam / wrap-turn command mapping is still unstable across successive seam-adjacent jogs.
  - In this run, the first J3 seam move completed, then the next `1 deg` jog likely got translated into the wrong equivalent turn in the raw write frame, producing the opposite-direction lurch and the eventual `Er47.0`.
- Validation that ran:
  - read current controller/terminal logs around the failing jog sequence
  - `curl -s http://127.0.0.1:4000/control/motion-status`
  - `curl -s http://127.0.0.1:4000/info/joints-detailed`
  - read `/run/gradient-rt-motion/metrics.json`
  - inspected `src/gradient_os/api/main.py`, `src/gradient_os/arm_controller/command_api.py`, `src/gradient_os/arm_controller/servo_driver.py`, and `src/gradient_os/arm_controller/backends/ethercat_rtcore/backend.py`
- Follow-up / risk:
  - No code change in this pass yet; this was diagnosis only.
  - The next likely fix path is to stop baselining jog steps from a seam-sensitive raw frame and/or harden the raw-write wrap-turn selection after a seam-crossing move, especially for the persistent J3/J4 family.

## 2026-04-16 04:49 +0000 - Switched public A6-EC joint truth to continuous semantics while preserving raw write-frame turn selection

- Context:
  - The user explicitly called out that the live stack was still behaving like a single-turn wrapped system even though the drives expose multi-turn data, and rejected `-359` appearing adjacent to `0` in public/controller truth.
- Changes:
  - Updated `src/gradient_os/arm_controller/backends/ethercat_rtcore/backend.py`
    - changed `raw_to_joint_positions()` to return the continuous `reference_mode="display"` truth instead of the wrapped raw RTCore/reference frame
    - extended `_canonical_joint_positions_from_raw_feedback()` so display-mode truth also runs a second raw-frame roundtrip against the live wrapped reference and stores `raw_reference_wrap_lift_counts`
    - clear cached raw wrap-lift state whenever truth is unavailable or either the public-truth roundtrip or raw command-frame roundtrip fails, to avoid stale equivalent-turn reuse
  - Updated `tests/test_gradient05_limits_and_backends.py`
    - added `test_ethercat_backend_drive_native_truth_unwraps_public_joint_positions_but_preserves_raw_write_frame` to lock down the seam case: public truth reads back as continuous `-0.08 rad`, while converting that truth back into the raw command frame still reconstructs the original wrapped count
- Validation that ran:
  - `source "/home/pi/GradientOS/.venv/bin/activate" && python -m py_compile src/gradient_os/arm_controller/backends/ethercat_rtcore/backend.py tests/test_gradient05_limits_and_backends.py`
    - result: success
  - `source "/home/pi/GradientOS/.venv/bin/activate" && PYTHONPATH=src python -m pytest tests/test_gradient05_limits_and_backends.py::test_ethercat_backend_uses_drive_native_truth_when_startup_and_status_are_valid tests/test_gradient05_limits_and_backends.py::test_ethercat_backend_drive_native_truth_unwraps_public_joint_positions_but_preserves_raw_write_frame tests/test_gradient05_limits_and_backends.py::test_ethercat_backend_translates_canonical_truth_back_into_raw_wire_counts tests/test_gradient05_limits_and_backends.py::test_ethercat_backend_startup_bootstrap_uses_display_reference_mode -q`
    - result: `4 passed`
  - `ReadLints` on touched backend/test files
    - result: no diagnostics
- Follow-up / risk:
  - This fixes the immediate seam-wrapped public-truth/jog-baseline bug without moving the raw command upload path off the RTCore `6064/607A` frame.
  - It does not yet settle the larger architectural question of whether long-term A6-EC canonical truth should come directly from anchored `U40.20/.22` absolute counts instead of the drive-native reference frame plus continuous unwrapping.

## 2026-04-16 05:07 +0000 - Expanded the J6 Chapter 5 probe to capture controller, frontend, and RTCore views in one experiment artifact

- Context:
  - The user wants to rerun the manual J6 rotation experiment and answer the still-open question directly: do the encoder counts wrap, or do they continue monotonically across turns?
  - The user also explicitly asked to record not only raw drive objects, but also what the controller and frontend see during the experiment.
- Changes:
  - Updated `scripts/a6ec_chapter5_probe.py`
    - added direct UDP controller capture for `GET_JOINT_STATE` and `GET_MOTION_STATUS`
    - added API capture for `/info/joints`, `/info/joints-detailed`, `/control/motion-status`, and a one-event `/monitor` sample
    - added RTCore metrics capture from `/run/gradient-rt-motion/metrics.json`
    - threaded those views into both `snapshot` and `watch` outputs so each artifact now contains raw SDO reads, controller truth, frontend-facing payloads, and RTCore state together
    - added `--controller-host`, `--controller-port`, and `--monitor-timeout-s` CLI flags
    - preserved partial-capture behavior so powered-down/unavailable phases record explicit `ok/error` results instead of silently dropping failed reads
  - Updated `tests/test_a6ec_chapter5_probe.py`
    - added focused tests for base64 motion-status parsing, SSE monitor-event parsing, merged watch samples with controller/frontend/monitor/metrics fields, and snapshot assembly with the new captures
- Validation that ran:
  - `source "/home/pi/GradientOS/.venv/bin/activate" && python -m py_compile scripts/a6ec_chapter5_probe.py tests/test_a6ec_chapter5_probe.py`
    - result: success
  - `source "/home/pi/GradientOS/.venv/bin/activate" && PYTHONPATH=src python -m pytest tests/test_a6ec_chapter5_probe.py -q`
    - result: `9 passed`
  - `source "/home/pi/GradientOS/.venv/bin/activate" && python scripts/a6ec_chapter5_probe.py --help`
    - result: success
  - `source "/home/pi/GradientOS/.venv/bin/activate" && python scripts/a6ec_chapter5_probe.py snapshot --help`
    - result: success
  - `source "/home/pi/GradientOS/.venv/bin/activate" && python scripts/a6ec_chapter5_probe.py watch --help`
    - result: success
  - `ReadLints` on touched probe/test files
    - result: no diagnostics
- Follow-up / risk:
  - This change prepares the experiment harness, but it does not itself answer the wrap/monotonic question; that still requires the live J6 manual-rotation run.
  - The powered-down phase may legitimately produce missing controller/API/monitor data depending on how much of the stack remains reachable, but the artifact will now show that explicitly instead of hiding it.

## 2026-04-16 05:42 +0000 - Live J6 manual-rotation experiment shows multi-turn absolute continuity and a separate wrapped raw-reference family

- Context:
  - The user ran the live J6 experiment: rotate `> +360 deg`, back to zero, then `> -360 deg`, with the expanded probe recording raw SDO objects, controller replies, API payloads, `/monitor`, and RTCore metrics together.
- Runtime artifacts:
  - watch: `logs/encoder-retention/j6-manual-rotate-20260416-053435/j6-manual-rotate-live.watch.jsonl`
  - final snapshot: `logs/encoder-retention/j6-manual-rotate-20260416-053435/j6-manual-rotate-final.json`
  - final markdown: `logs/encoder-retention/j6-manual-rotate-20260416-053435/j6-manual-rotate-final.md`
- Findings:
  - Initial near-zero plateau:
    - `6064 ~= 3`
    - `U40.16 ~= 3`
    - `encoder_multi_turn_counts ~= 113075`
    - controller/frontend truth near `0 deg`
  - After the positive `>360` sweep and return near zero:
    - `encoder_multi_turn_counts` returned near the same neighborhood (`~113040`)
    - controller/frontend truth stayed near zero (`~0.008 deg`)
    - but the reference family sat around `6064 ~= 1310690`, `U40.16 ~= -30`, `rotation_mode_encoder_counts ~= 1310690`
    - interpretation: the absolute source returned near its starting count while the reference family kept a one-turn-lifted seam-equivalent state
  - During the longer sweep:
    - `encoder_multi_turn_counts` moved through large multi-turn values such as `-1216460`, `-1839631`, and `2190820`
    - controller/frontend truth also moved continuously to about `-570.9 deg`
    - interpretation: the absolute source is not single-turn wrapped; it remains multi-turn continuous over the excursion
  - Final stable snapshot:
    - controller truth: `arm_deg = 2.0687255859375`, `axis_counts = 1303188`
    - selected axis detail: `raw_counts = 1303188`, `reference_pre_zero_rad = 0.03610607279485828`, `raw_reference_pre_zero_rad = -6.247079234384728`
    - wrap bookkeeping: `raw_command_roundtrip_reference_wrap_lift_counts = 1310720`, `raw_command_roundtrip_reference_wrap_lift_turns = 1.0`
    - absolute source: `absolute_counts = 105539`, `absolute_source = encoder_multi_turn_counts`
  - Secondary observation:
    - final snapshot shows `U40.28 = 1303190` and `rotation_mode_encoder_counts = 1303190`
    - the older probe bridge assumption `U40.2A/.2C ~= U40.28 * C10_ratio` is false in the current posture because both values already matched directly while `C10.18/C10.19 = 10.0`
- Interpretation:
  - The J6 experiment directly supports the user's objection: the A6-EC exposes a continuous multi-turn count path, and that path is not behaving like a single-turn wrapped signal.
  - The wrapped/seam-equivalent behavior still present in the stack belongs to the drive reference/raw command family and the host-side lift used to reconcile it, not to the existence of the multi-turn absolute counts themselves.
- Validation that ran:
  - live `watch` capture during the manual experiment
  - stable post-run `snapshot` capture on the same experiment id
- Follow-up / risk:
  - This does not yet by itself decide the final write-path architecture, but it materially weakens the argument for treating the direct multi-turn absolute source as if it were inherently single-turn wrapped.

## 2026-04-16 06:08 +0000 - Built a trimmed dual-axis canvas for the J6 watch dataset

- What changed:
  - Generated `/home/pi/.cursor/projects/home-pi-GradientOS/canvases/j6-manual-rotate-dataset.canvas.tsx`
  - The canvas embeds the J6 watch time series from `logs/encoder-retention/j6-manual-rotate-20260416-053435/j6-manual-rotate-live.watch.jsonl` and renders a dual-axis SVG chart with counts-like fields on the left axis and all other numeric fields on the right
  - Added presets and per-series toggles so the full numeric capture remains explorable without forcing every series to stay visible at once
  - Froze the active `053435` probe and trimmed the long stationary tail before chart generation, reducing the plotted slice from `1189` total samples to `144`
- Validation performed:
  - confirmed the active `053435` watch file stopped growing after terminating its writer process
  - computed the flat-tail cutoff from the frozen dataset using trailing ranges of `combined_u4020_22_signed_counts <= 8`, `api_absolute_counts <= 8`, and `api_arm_deg <= 0.02`
  - `ReadLints` on `/home/pi/.cursor/projects/home-pi-GradientOS/canvases/j6-manual-rotate-dataset.canvas.tsx`
    - result: no diagnostics
- Follow-up / risk:
  - A separate older J6 probe for experiment `20260416-052258` is still running against its own file; it does not affect this canvas, but it can confuse future capture audits if left running

## 2026-04-16 06:18 +0000 - Investigated how to open the J6 canvas artifact

- What changed:
  - No product code changed.
  - Verified `/home/pi/.cursor/projects/home-pi-GradientOS/canvases/j6-manual-rotate-dataset.canvas.tsx` exists under the managed canvas directory.
  - Confirmed no `j6-manual-rotate-dataset.canvas.status.json` sidecar exists yet, which is consistent with the canvas not having been rendered/built once yet.
  - Updated `.cursor/memory/AGENT_SCRATCHPAD.md` with a durable canvas-opening guardrail.
- Validation performed:
  - Read the current canvas skill guidance plus the latest scratchpad/devlog context.
  - `Glob` search for `j6-manual-rotate-dataset.canvas*`
- Follow-up / risk:
  - If clicking/opening the canvas path still does not render it, the next step is to inspect the canvas source itself or wait for the first build attempt to emit a `.canvas.status.json` diagnostic sidecar.

## 2026-04-16 06:29 +0000 - Corrected the J6 canvas open-path guidance

- What changed:
  - No product code changed.
  - Identified that the earlier chat reply wrapped the canvas path with leading/trailing spaces inside the backticks, which likely caused Cursor to try opening the wrong literal path.
  - Updated `.cursor/memory/AGENT_SCRATCHPAD.md` with a guardrail to keep clickable file paths exact.
- Validation performed:
  - Re-read the existing canvas file at `/home/pi/.cursor/projects/home-pi-GradientOS/canvases/j6-manual-rotate-dataset.canvas.tsx`
  - Confirmed the real file exists, is readable, and has normal permissions
- Follow-up / risk:
  - If the corrected exact path still fails to open, the remaining likely causes are a Cursor-side path-opening quirk for this hidden managed directory or a canvas runtime issue that only appears on first build.

## 2026-04-16 06:47 +0000 - Rebuilt the J6 canvas as a normal web page for SSH/browser use

- What changed:
  - Added a standalone React page at `web-ui/j6-manual-rotate-dataset.html` with entrypoint `web-ui/src/j6-manual-rotate-dataset.tsx`
  - Added `web-ui/src/J6ManualRotateDatasetPage.tsx`, which ports the trimmed J6 manual-rotation dataset into a browser-native page with:
    - the archived experiment summary
    - the dual-axis SVG chart
    - the same preset-based filtering and per-series toggles as the canvas
    - a sample scrubber and selected-sample detail tables
  - Updated `web-ui/vite.config.ts` so `vite build` emits both the main app and the standalone J6 dataset page
  - Added `web-ui/src/J6ManualRotateDatasetPage.test.tsx` covering page render and preset switching
- Validation performed:
  - `npm test -- J6ManualRotateDatasetPage.test.tsx`
    - result: `2 passed`
  - `npm run build`
    - result: success; emitted `dist/j6-manual-rotate-dataset.html`
- Follow-up / risk:
  - The standalone page intentionally avoids threading this archive view through the main app shell, which keeps the SSH/browser delivery path simple and low-risk.
  - The existing large `ArmVisualizer` build chunk warning remains in the main app build and was not introduced by this dataset page.

## 2026-04-16 06:20 +0000 - Rooted A6-EC planner truth in anchored `encoder_multi_turn_counts`

- What changed:
  - Updated `src/gradient_os/arm_controller/profiles/drive/a6ec_ds402.py` so A6-EC declares `canonical_truth_source = "encoder_multi_turn_counts"` and `absolute_home_anchor_required = True`.
  - Narrowed A6-EC absolute-truth resolution to `encoder_multi_turn_counts` instead of allowing the truth resolver to fall through to the rotation-mode family.
  - Patched `src/gradient_os/arm_controller/backends/ethercat_rtcore/backend.py` so the drive-native A6-EC read-truth path reconstructs canonical joint truth from `absolute_axis_q_rad - absolute_home_anchor_rad - master_offset`, while still preserving raw 6064-family wrap-lift bookkeeping for the write path.
  - Split planner/control truth from operator display semantics more explicitly: `raw_to_joint_positions()` now validates the anchored absolute truth against the raw/write-frame roundtrip, while display snapshots remain on the stricter display-mode path.
  - Added and updated focused regressions in `tests/test_gradient05_limits_and_backends.py` for anchored A6-EC truth, preserved raw write-frame conversion, required-anchor fail-closed behavior, and restart/power-transition truth setup.
- Validation performed:
  - `pytest -q tests/test_gradient05_limits_and_backends.py -k "drive_native_truth or startup_bootstrap or absolute_anchor"`
    - result: `8 passed`
  - `pytest -q tests/test_gradient05_limits_and_backends.py tests/test_run_controller_helpers.py`
    - result: `93 passed`
  - `pytest -q tests/test_api_endpoints.py -k "joint or monitor"`
    - result: `13 passed, 56 deselected`
  - `ReadLints` on the touched backend/profile/test files
    - result: no diagnostics
- Follow-up / risk:
  - The read-truth path is now correctly anchored to the continuous encoder source, but the RTCore/drive write contract still targets the 607A/6064 CSP reference family. Commanding directly in the encoder-multiturn object family would require a deliberate write-path redesign rather than another read-truth tweak.
