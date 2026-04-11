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
