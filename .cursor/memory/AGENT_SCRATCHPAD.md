# Agent Scratchpad

Use this file as persistent, repo-local execution memory.

## File Policy

- Current policy: `COMMITTED`
- Rationale:
  - The user explicitly wants persistent scratchpad/devlog use with periodic rollover instead of unbounded growth.

## How To Use

1. Read the latest retained lessons before meaningful work.
2. Build a short preflight checklist from the durable guardrails that apply to the task.
3. Re-read before risky operations.
4. Log only high-signal learnings that should change future behavior.
5. Roll over again before this file grows back into a full historical ledger.

## Entry Rules

- Tag operational notes with source: `[self]`, `[user]`, or `[tool]`.
- Prefer concrete, testable rules tied to files, commands, or live checks.
- Keep durable lessons here; move long chronological narratives into dated snapshots.

## Retained Lessons

### User Preferences
- [user] Prefer implementation over discussion; do the fix, do not only explain it.
- [user] Always maintain both `.cursor/memory/AGENT_SCRATCHPAD.md` and `.cursor/memory/DEVLOG.md` for meaningful tasks.
- [user] For local validation on this machine, use `source ./start.sh` or the repo `.venv`; do not reach for `uv`.
- [user] Check repo-local A6-EC references before web lookup: `docs/resources/A6-EC_series_servo_drive_manual.pdf`, `docs/resources/a6ec_manual_codes.json`, and `docs/resources/a6ec_manual_codes.md`.
- [user] When extending motion/program behavior, reuse existing `command_api` vocabulary and semantics rather than inventing parallel command names.

### Regression Guardrails
- [self] Physical robot behavior outranks stale UI/API/telemetry assumptions; do not declare a live fix complete until the real hardware behavior matches the operator-visible outcome.
- [self] Safety-critical REST booleans, runtime gates, and commissioning state transitions must be explicit; never rely on Python truthiness or ambiguous fallback behavior.
- [self] When a field or behavior crosses stack boundaries, update the relevant UI, API, controller/backend, and persistence/docs layers together.
- [self] For RTCore / EtherCAT changes, verify the running binary or deployment path; rebuilding in-repo alone does not update `/usr/local/bin/gradient-rt-motion`.
- [self] Keep drive-family-specific config and semantics in drive profiles/catalogs, not in generic runtime, telemetry, or controller layers.

### Native-Home Active Workstream
- [self] Native-home work is still active and regression-prone; after risky changes, verify the full UI -> API -> controller -> RTCore -> live feedback chain on hardware before calling it fixed.
- [self] For A6-EC native home, the saved drive offset capture must be `desired_offset = -pos_counts`.
- [self] Apply `native_home_position_offset` on feedback/logical read paths and RTCore hold-target alignment, but do not subtract it again on outgoing RTCore logical position commands.
- [self] Successful native home should leave only the homed axis disabled; do not accidentally disable all axes unless the workflow explicitly intends that.
- [self] Surface native-home result/state in telemetry and UI; operators must not have to infer success from brake clicks, missing torque, or changed enable masks alone.
- [self] If a single axis is faulted while RTCore is otherwise healthy, prefer the targeted RTCore / DS402 reset path while disarmed before broader restarts.
- [self] Bounded RTCore commissioning moves can appear to fail after acceptance; compare live motion status and last submitted trajectory before reporting a timeout as a real motion failure.

### Validation Habits
- [tool] Fast repo validation loop: targeted `pytest`, then `python -m py_compile`, then `ReadLints` on touched files.
- [tool] For frontend work, `npm run build` plus `ReadLints` catches regressions quickly.
- [self] When a control-stack or commissioning behavior materially changes, update docs/SOP in the same pass instead of leaving the knowledge only in chat history.

## Session Entries

*(Rolled over on `2026-04-08`. Detailed prior history now lives in `.cursor/memory/AGENT_SCRATCHPAD_2026-02-21_to_2026-04-08.md`. Older archive material remains in `.cursor/memory/AGENT_SCRATCHPAD_ARCHIVE.md`.)*

### 2026-04-08 - Scratchpad rollover + retained native-home guardrails
- [self] Consolidated the oversized live scratchpad into a dated snapshot and carried forward only durable user preferences, validation rules, and the active native-home regression guardrails.
- [self] Keep this file under roughly 200 lines; archive again once entries become repetitive or the active workstream meaningfully changes.

### 2026-04-08 - Archive-first memory rollover is now the standard rule
- [user] Do not delete old scratchpad/devlog material just to reduce context; preserve it in dated snapshots so historical debugging context remains recoverable.
- [self] Corrective rule: when the live scratchpad or devlog needs a fresh slate, rename the current live file to a dated snapshot, prepend an archive summary, create a new slim live file, and leave older archives intact unless the user explicitly requests deeper reorganization.

### 2026-04-08 - RTCore streamed `qd` must follow real timing
- [user] Do not invent a legacy `0..4095 -> motor RPM` velocity mapping for RTCore payloads when an existing timed trajectory or joint-velocity path already provides the right semantics.
- [self] For RTCore streamed `sync_write()` points, derive `qd` from the controller timestep and joint delta, not from the single-point minimum-duration helper.
- [self] Only advance `_last_joint_setpoint_rad` after `commit_trajectory()` succeeds; failed RTCore writes must not poison fallback state for the next move.
- [self] The direct RTCore one-point compatibility helpers still rely on a legacy speed heuristic; prefer controller-timed trajectory or jog paths for live validation until that direct path is replaced with a validated controller-owned plan.

### 2026-04-08 - Commissioning degree-step jog depends on backend reads staying usable
- [self] The Joint Commissioning `+/- deg` jog controls are blocked in the UI whenever `/info/joints-detailed` reports `read_source="cached_fallback"`.
- [self] In the RTCore backend, tightening `get_joint_positions()` / `sync_read_positions()` with a freshness gate can silently disable commissioning jog even if the older working path could still read mapped joint angles.
- [self] Regression-prevention rule: when restoring commissioning jog behavior, compare against the pre-native-home bounded path baseline and verify the UI can obtain joint feedback through `GET_JOINT_STATE` without reintroducing any legacy speed-to-velocity shim.

### 2026-04-08 - RTCore commissioning retest does not automatically mean rebuild
- [self] For Python-only commissioning fixes in `src/gradient_os/arm_controller/...`, the next live check is usually a controller/API/UI retest against the existing RTCore service, not a `main.cpp` rebuild.
- [self] Rebuild + reinstall RTCore only when the desired behavior depends on `src/gradient_rt_motion/*`, `systemd/rt-motion/*`, or regenerated RTCore runtime env changes; the running service executes `/usr/local/bin/gradient-rt-motion`, not the repo binary directly.
- [self] Use `systemd/rt-motion/sync-runtime.sh --ensure-active` only when intentionally syncing the repo RTCore binary/unit/env into the installed service, not as a blanket step for every Python commissioning follow-up.
- [self] Current dirty-branch RTCore changes include `main.cpp` / `ipc_v1.hpp` native-home hold-target alignment and service-SDO-write support; those changes are invisible to live hardware until RTCore is rebuilt and synced into `/usr/local/bin/gradient-rt-motion`.

### 2026-04-08 - Commissioning jog can self-lock after accepted RTCore move
- [self] If a Joint Commissioning jog is accepted by RTCore but `wait_for_trajectory_complete()` times out, the controller thread logs a timeout even though `GET_JOINT_STATE` feedback can still be live.
- [self] The UI disables commissioning jog whenever `motionStatus.state` is `accepted`, `queued`, or `executing`; `/control/joint-jog` currently parses `wait_for_idle` but returns the initial ACK payload without actually waiting for a terminal state.
- [self] Practical result: one small jog can leave the commissioning panel locked out on a stale `"accepted"` motion status until a later motion-status update, STOP, or refresh clears it.

### 2026-04-08 - J2 wrong-direction move traced to lost native-home offset truth
- [tool] Full RTCore journal shows J2 native home really wrote `desired_offset=-107506` with `saved=1` at `2026-04-08 19:18:51`, so the native-home command itself was accepted by RTCore.
- [tool] After RTCore restart/rebuild, live `/run/gradient-rt-motion/metrics.json` reports `native_home_position_offset: 0` for every axis, including J2.
- [self] If RTCore metrics read J2 `native_home_position_offset=0`, the hold-target alignment and Python feedback compensation both operate as if no native-home offset exists, even though the earlier home flow reported success.
- [self] Practical commissioning rule: if a native-homed axis moves the wrong direction/magnitude after power-up, compare the earlier RTCore `desired_offset` journal line against current metrics before blaming the `joint-jog` route or planner.
- [tool] Direct EtherCAT SDO read proves the drive still holds J2 `0x60B0 = -107506` while RTCore metrics still show `native_home_position_offset=0`.
- [self] The current RTCore startup path reads `0x60B0` only once, before EtherCAT startup convergence/process-data-live is established; if that early SDO upload returns a stale/default value, RTCore never refreshes it later and keeps the wrong zero-offset truth for the whole session.

### 2026-04-08 - Startup offset readback fix is real, but not sufficient
- [self] After moving `0x60B0` refresh to post-`startup_ready`, live RTCore metrics now publish the real J2 offset again (`native_home_position_offset=-107506` for axis 1) after restart; confirm this before assuming startup offset truth is still broken.
- [tool] Direct live proof still failed after that fix: with J2 powered up at about `-16.39 deg`, a commissioning `POST /control/joint-jog {"joint": 2, "delta_deg": 1.0}` drove reported J2 to about `-21.30 deg` and the RTCore trajectory stayed latched until `STOP`.
- [tool] Controller logs for that failed proof show a bounded target of `current_deg=-16.391 -> target_deg=-15.391` at `max_motor_rpm=100.0`, followed by `Open-Loop Executor finished` and then `TimeoutError: Timed out waiting for RTCore trajectory 1 to complete`.
- [self] Updated rule: fixing startup `native_home_position_offset` readback does not by itself explain or eliminate the wrong-direction / never-idle commissioning bug; the next investigation must focus on a second issue in RTCore execution/target interpretation after the bounded path is enqueued.

### 2026-04-09 - Drive power cycle changed the live bug into a persistence problem
- [user] Clarification: the drives were actually power-cycled, so any later `0x60B0=0` observation must be interpreted as a real post-power-loss state change, not just stale RTCore metrics.
- [tool] After that power cycle, direct EtherCAT SDO read and RTCore metrics both showed J2 `0x60B0/native_home_position_offset = 0`, which means the drive itself no longer retained the previous native-home offset.
- [self] Corrective rule: when debugging A6-EC native home, distinguish "RTCore restart while drives stay powered" from a real drive power cycle; only the latter proves whether `0x1010:01` persistence actually survived NVM storage.
- [tool] New patch/result: native-home now waits for RTCore to publish a verified terminal result, and RTCore now waits for post-`0x1010` `0x60B0` readback before marking success. Live J2 re-home now logs `desired_offset=668218 readback_offset=668218 saved=1`, direct `ethercat upload` returns `668218`, and `/info/joints-detailed` reports J2 `0.0 deg`.
- [self] Remaining blocker: the only meaningful next proof is another real drive power cycle followed by a fresh `ethercat upload -p 1 -t int32 0x60B0 0`; without that manual step, native-home persistence across power loss is still unproven.

### 2026-04-09 - Power-cycle bus recovery can mask truth unless RTCore re-arms refresh
- [tool] After the user performed an E-stop power cycle, direct EtherCAT still showed J2 `0x60B0 = 0`, but the still-running RTCore process continued publishing the old in-memory `native_home_position_offset=668218` until it was restarted.
- [self] Corrective rule: if drives power-cycle while `gradient-rt-motion` stays alive, do not trust `native_home_position_offset` until RTCore has rerun its post-`startup_ready` refresh or the process has restarted.
- [self] New fix: re-arm RTCore startup readback and native-home offset refresh whenever the startup epoch changes (`startup_reset_count` changes or `startup_ready` falls), not only on first process boot.
- [tool] Live verification after rebuilding/syncing RTCore: direct `ethercat upload -p 1 -t int32 0x60B0 0` returned `0`, `/run/gradient-rt-motion/metrics.json` axis 1 now also reports `native_home_position_offset=0`, and `/info/joints-detailed` shows J2 near `-18.35 deg` instead of the stale logical `0 deg`.
- [self] Safety rule: after any real drive power loss, do not commission jog a previously native-homed axis until RTCore metrics and direct `0x60B0` reads agree on the current offset truth.

### 2026-04-09 - Drive power and jog arming must never share ambiguous labels
- [user] Repeated "Power up RTCore-controlled drives now?" prompts were confusing during J2 re-home prep because the UI exposed both drive-enable and jog-enable actions with overlapping `Arm/Disarm` wording.
- [self] Corrective rule: operator-facing labels must distinguish drive power transitions from jog-session arming; never present both as generic `Arm` in the same control surface.
- [self] New UI fix: runtime-header drive controls now read `Power Up` / `Power Down`, while the realtime jog toggle now reads `Arm Jog` / `Disarm Jog`.
- [tool] Frontend validation: `npm run test -- src/ControlPanel.test.tsx` passed and `npm run build` succeeded after the label split.

### 2026-04-09 - Direct SDO proof says `0x60B0` is not surviving real power loss
- [tool] After writing J2 `0x60B0 = -96134` directly via EtherCAT SDO, issuing `0x1010:01 = "save"`, and waiting for settle, immediate readback still returned `-96134` with no fault (`0x603F=0`) while the axis remained in `SwitchOnDisabled` (`0x6041=0x1650`).
- [tool] After the next real drive power cycle, the first fresh read returned `0x60B0 = 0`, RTCore metrics refreshed axis 1 `native_home_position_offset=0`, and `/info/joints-detailed` again showed J2 near its raw physical angle instead of logical zero.
- [tool] The A6-EC ESI exposes `0x607C Home offset` alongside `0x60B0 Position offset`; it also shows vendor object `0x2013:17 = 1` (`Update function code values written via communication to EEPROM`), so the EtherCAT-side EEPROM update gate already appears enabled.
- [self] New root-cause rule: a direct `0x60B0` write+`0x1010` save still evaporating across real power loss means the remaining persistence bug is not specific to our native-home workflow sequencing.
- [self] Working hypothesis: `0x60B0` behaves as a runtime offset on this A6-EC and should not be treated as the durable hardware-zero store; validate `0x607C` or the vendor-native persistent zero parameter before changing commissioning code again.
- [self] Scope rule: current proof is strongest for the A6-EC object semantics on the tested drive and should not be labeled "J2-only" or "all axes" without an additional cross-axis persistence check; same model/firmware makes a drive-wide behavior more likely than a single-axis software bug.
- [self] Migration rule: if native-home persistence moves from `0x60B0` to `0x607C`, re-audit RTCore/controller feedback and command math so the persistent home offset is not applied once in-drive and then again in software.
- [tool] Cross-axis proof: direct writes to `0x607C` on J1 (`12345`) and J2 (`-23456`) both survived a real drive power cycle with no faults, while `0x60B0` had previously reset to `0` under the same class of test.
- [self] New root-cause confidence: for this A6-EC setup, durable drive-home behavior aligns with `0x607C Home offset`, not `0x60B0 Position offset`.
- [self] New integration rule: RTCore metrics currently refresh only `0x60B0`, so after any `0x607C` experiment the existing `native_home_position_offset` telemetry remains zero until code is migrated to the new source-of-truth object.
- [self] Smallest-safe migration surface: replace the RTCore read/write helper behind `native_home_position_offset` with a profile-owned "persistent native-home object" source, then revalidate only the frame-sensitive paths that already consume `native_home_position_offset` (startup refresh, native-home command success, feedback-aligned target sync, Python metrics refresh, and commanded-target no-double-apply rules).

### 2026-04-11 - Keep mode layers separate and treat native home as commissioning-only
- [self] New architecture rule: do not conflate A6-EC `C00.07 / 0x2000:08` startup absolute-system selection with DS402 runtime operating modes like `HM` (`0x6060 = 6`) and `CSP` (`0x6060 = 8`); they operate at different layers.
- [self] New workflow rule: for GradientOS, steady-state motion should remain `CSP`, while native home should be modeled as a one-shot commissioning transaction that temporarily enters `HM`, captures home, returns to `CSP`, and resynchronizes targets before later motion.
- [self] New docs rule: when the homing model changes, update both public bring-up docs and the internal commissioning SOP in the same pass so future sessions do not inherit contradictory `0x60B0` / `C00.07=1` guidance.
- [user] Explicit requirement: the plan and later implementation must keep RTCore free of all vendor-specific homing hardcodes; object IDs, method numbers, ordering, and save semantics must live in profile/catalog/backend-owned descriptor data, not in `main.cpp`.
- [self] New plan rule: place linked documentation sources at the start of the homing plan so the implementation pass can trace every claim back to the manual, ESI, or bring-up docs without re-searching.
- [self] New cold-handoff rule: if the build is handed to a fresh agent, the first-cut defaults are now fixed unless bench evidence disproves them: `C00.07=4`, steady-state `CSP`, commissioning `HM`, `6098=35`, `60E6=0`, `607C=0`, `607C` as persistent truth source, no explicit `0x1010` in the first HM implementation, and validation on `J4` plus `J2`.
- [self] New contract rule: for the first HM-based cut, preserve the `native_home_position_offset` field name but expect its published value to be `0` after a successful A6-EC home because `0x6064` should already be in the homed frame; if bench evidence contradicts that, revise the descriptor/truth model before touching frame math.

### 2026-04-11 - Existing RTCore set-mode command is not a usable HM switch
- [self] During the HM build pass, verify the actual RTCore mode application path before relying on `_send_cmd_set_mode()` or `MSG_CMD_SET_MODE`; in the current code, the helper thread stores a mode value but the cyclic loop still hardcodes `mode_out = 8` for enabled axes.
- [self] Corrective rule: the first HM/native-home implementation must either make the runtime mode path real or perform explicit descriptor-driven `0x6060` writes inside the generic native-home transaction; do not assume the existing set-mode IPC already switches DS402 modes.

### 2026-04-11 - HM executor must use RTCore service overrides, not helper-thread SDO writes to PDO-owned objects
- [self] New implementation rule: when a commissioning transaction needs to drive `0x6040` or `0x6060` on A6-EC, do not write those objects via helper-thread SDO while the cyclic PDO loop is alive; the cyclic `RxPDO 0x1702` path will keep overwriting them.
- [self] Corrective pattern: express vendor homing specifics in profile-owned descriptor data, then let RTCore execute generic primitives by combining SDO writes for non-PDO objects (`0x6098`, `0x60E6`, `0x607C`) with a generic per-axis service override for PDO-owned controlword/mode fields.
- [self] Regression-prevention rule: if `native_home_position_offset` changes to a truth source like `0x607C`, revalidate both the startup refresh path and the feedback-aligned hold-target math together; a clean compile is not enough for commissioning safety.

### 2026-04-11 - J4 HM trace proves mode switch works but terminal condition is still wrong
- [tool] Live J4 capture during `NATIVE_HOME_JOINT,4` showed `0x6061` switching from `8` to `6`, while `0x6041` progressed `0x1650 -> 0x0633 -> 0x0237`, then stayed at `0x0237` for about 10 seconds before RTCore timed out and restored `0x1650` / `0x6061=8`.
- [self] New diagnosis rule: the current failure is not an early SDO abort or failure to enter HM; it is a post-enable HM completion problem. Treat `native_home_last_abort_code=0` plus final `statusword=0x0237` as evidence that the descriptor's terminal condition and/or homing-start handshake is wrong.
- [self] Next-debug rule: before widening scope to other joints, capture and reason about the real A6-EC HM completion semantics for J4 (statusword bits and possible controlword start-edge requirements) instead of assuming the current `wait_statusword all_set=0x9000 all_clear=0x2000` mask is correct.

### 2026-04-11 - J4 HM succeeds when the start edge is delayed until HM is op-enabled
- [tool] After changing the A6-EC native-home descriptor from one `controlword_sequence [6,7,15,31]` step to `controlword_sequence [6,7,15] -> wait_statusword all_set=0x0227 all_clear=0x2048 -> controlword_sequence [31]`, a clean end-to-end `NATIVE_HOME_JOINT,4` retry succeeded.
- [tool] Live J4 capture on the successful run showed `0x6061` switching `8 -> 6`, `0x6041` progressing `0x1650 -> 0x0633 -> 0x0237 -> 0x9650`, then restoring `0x6061=8` while RTCore logged `Native-home success`; `0x607C` stayed `0`.
- [self] New commissioning rule: for this A6-EC HM method-35 flow, treat `0x0237` in HM as the precondition for asserting the homing-start edge and treat `0x9650` (bits 15 and 12 set, bit 13 clear) as the observed J4 completion signature.

### 2026-04-11 - Restarting RTCore invalidates the controller IPC session until the stack reconnects
- [tool] A `J4` retry issued after `gradient-rt-motion.service` was restarted but before the controller stack was restarted returned API `503` and controller-side `native drive-home did not reach a verified terminal state`, yet RTCore showed no new `Controller connected` / `IPC handshake complete` lines and the drive-side sampler never left `0x6041=0x1650`, `0x6061=8`.
- [self] New validation rule: after restarting RTCore during commissioning, do not trust the next controller/API command until RTCore logs a fresh controller connection/IPC handshake or the full stack has been restarted against the new RTCore instance.

### 2026-04-11 - Jogging one joint can shove other axes if command-frame and hold-frame disagree
- [tool] During a fresh post-restart commissioning test, the first UI jog request targeted software `J4` correctly (`target_deg` changed only the 4th joint from about `-0.002` to `-1.002`), yet the next feedback sample showed `J1` had moved from about `-1.876` to `-2.215` while the RTCore trajectory ended in state `faulted`.
- [tool] Live metrics at the same time showed stale nonzero native-home offsets on `J1/J2` (`12345` and `-96135` counts) plus a new `J2` drive fault (`0x603F=0xFF00`, `0x203F=0x0871`, A6-EC `Er87.1` excessive position reference increment).
- [self] New root-cause rule: when RTCore aligns CSP hold targets as `pos - native_home_position_offset`, the Python/backend path that uploads queued trajectory targets must generate commands in that same frame. If it uploads all-axis targets without compensating for nonzero native-home offsets, a jog on one logical joint can silently inject target jumps on other axes that are merely supposed to hold position.
- [self] Bench clue: `12345` counts on the J1-class scaling is about `0.339 deg`, which matches the observed unintended J1 movement almost exactly; treat that as strong evidence that the current bug is a frame mismatch, not a simple joint-index remap.
- [self] Safety rule: do not trust commissioning jog on a stack where any uninvolved axis still has nonzero `native_home_position_offset` until the queued-trajectory command frame is proven consistent with RTCore hold-target alignment.

### 2026-04-12 - RTCore must reframe queued targets once, Python must not
- [self] The stable contract for the jog-frame fix is: Python/controller uploads queued axis targets in controller logical space, then RTCore converts them once into its feedback-aligned hold frame (`pos_counts - native_home_position_offset`) when it latches trajectory points.
- [self] Realtime RTCore jog must seed its internal target accumulator from that same feedback-aligned frame; starting from raw `pos_counts` recreates the same hidden offset-step bug for velocity-jog paths.
- [tool] `tests/test_gradient05_limits_and_backends.py` is not hermetic on a live machine when current RTCore metrics expose nonzero `native_home_position_offset`; broad file runs can inherit live offsets unless the test freezes `_refresh_native_home_offsets_from_metrics`.

### 2026-04-12 - Live proof shows the unsafe step happens on power-up, not only on jog
- [tool] With the rebuilt RTCore deployed and the proof condition intact (`J1 0x607C=12345`, `J2 0x607C=-96135`, `J4 0x607C=0`), a fresh `SAFE_POWER_UP` changed `J1` from about `105276` counts / `-3.2306 deg` to about `92889` counts / `-2.8903 deg` before the `J4` jog even mattered.
- [self] The `J1` power-up delta was about `-12387` counts, which is effectively the persisted `J1` home offset magnitude; treat this as strong evidence that RTCore's current hold-target alignment (`pos - native_home_position_offset`) is itself the wrong drive-target frame for live `0x607C` behavior.
- [tool] The subsequent single `J4 -1 deg` jog still ended `faulted`, but the run was already contaminated because `J2` faulted during power-up (`0x603F=0xFF00`, `0x203F=0x0871`, `0x6041=0x1638`).
- [self] Next corrective rule: re-check the assumption that `0x6064`/`0x607A` need software-side `- native_home_position_offset` alignment at all when `0x607C` is the persisted native-home truth source; the live bench now suggests that subtraction may be double-applying the drive's own homed frame on enable.

### 2026-04-12 - A6-EC `0x607C` truth still needs raw CSP hold targets
- [tool] After changing RTCore hold/output mirroring back to raw `0x6064` counts while keeping queued-target latch conversion explicit, the live two-stage proof passed on the same nonzero-offset condition: `SAFE_POWER_UP` produced no `Er87.1`, J1 moved only about `+0.001 deg`, J2 about `+0.035 deg`, and a single `J4 -1 deg` commissioning jog completed cleanly with only J4 moving about `-1.01 deg`.
- [self] Corrective rule: persisted A6-EC `0x607C` is durable native-home truth, but drive-facing CSP enable/hold/output targets must stay in the raw PDO/wire frame (`0x6064` / `0x607A` counts). Subtract `native_home_position_offset` only when converting queued controller/logical targets into raw CSP counts; do not subtract it again when mirroring live feedback into hold targets.
- [self] Correction to the earlier jog-frame note: realtime jog target seeding should start from raw `pos_counts`, not `pos_counts - native_home_position_offset`, because the drive-facing CSP accumulator lives in the raw wire frame too.

### 2026-04-12 - Native-home timeouts should degrade to pending verification, not generic failure
- [tool] A live `J1` native-home retry produced contradictory UI output because the backend returned `False` after a short verification wait, which made the controller/API emit a generic failure while RTCore telemetry later converged to `native_home_state=2` (`succeeded`).
- [self] Corrective rule: do not collapse drive-native home into a boolean. Carry a structured result across backend/controller/API/UI with separate `accepted`, `verified`, and `timed_out` fields so the operator can distinguish verified success, pending verification, and hard failure.
- [self] Regression-prevention rule: native-home verification must ignore stale pre-command metrics samples and wait for a fresh RTCore metrics update before trusting `requested/succeeded/failed`; otherwise old success state or short waits can produce false UI outcomes.

### 2026-04-12 - Post-restart J1 native-home revalidation matched the new contract
- [tool] After a hard stop and fresh stack restart (`logs/startups/20260412-043326/`), `controller.log` recorded `Received: 'NATIVE_HOME_JOINT,1'` followed immediately by `[EtherCAT RTCore] Native drive-home verified: joint=1 axis_mask=0x1`.
- [tool] The matching `api.log` entry for the same session was `POST /control/home-joint-native HTTP/1.1" 200 OK`, and live RTCore metrics showed axis 0 with `native_home_state=2`, `native_home_last_abort_code=0`, and `axis_enable_mask=62` (J1 left disabled while the other axes remained enabled).
- [self] Promotion rule: once live restart-proof validation agrees with the code/test contract, consolidate the stable guidance into the canonical GradientOS SOP files instead of leaving it only in scratchpad/devlog.

### 2026-04-12 - Canonical SOP updates must include the long-form master file too
- [self] After promoting the native-home rules into the routed SOP files, the long-form master file `RTOS_ETHERCAT_MASTER_OPERATING_PRINCIPLES.md` still carried the older wording. When a pattern graduates into the canonical skill set, update both the subsystem reference files and the master source document in the same pass.

### 2026-04-12 - Fresh power-cycle retention capture shows pose does not survive reboot yet
- [tool] Retention experiment `20260412-044300` captured `before_power_down` at `2026-04-12T04:43:00+00:00` and `after_power_up` at `2026-04-12T04:45:43+00:00`; the generated `comparison.md` reported `Raw encoder counts: MISMATCH` and `Logical joint angles: MISMATCH` on all six axes/joints.
- [tool] The after-power snapshot showed no active battery/multi-turn faults, but all axes jumped to large new `pos_counts` values while `startup_drive_config` remained unverified (`readback_valid=false`, `verified=false`) and each axis was in `SwitchOnDisabled` / `0x1650`.
- [self] New diagnosis rule: if a cold power cycle changes raw counts on every axis without battery faults, treat the startup absolute-position/encoder-tracking path as untrusted until the startup drive-config verification/readback path is proven and the retained-position mode is confirmed at real power-up.

### 2026-04-12 - Direct post-boot reads ruled out stale home writes and wrong startup mode
- [tool] Privileged live `ethercat upload` reads after the cold boot showed all axes had `0x2000:08 = 4`, `0x607C = 0`, `0x6064` values matching the after-power retention snapshot, and `0x6041 = 0x1650`.
- [tool] RTCore journal later logged successful startup readback on all axes (`commanded=4 readback=4 verified=1`), and live `/run/gradient-rt-motion/metrics.json` now shows `startup_drive_config.readback_valid=1` and `verified=1` on every axis.
- [self] Corrective rule: if cold-boot retention fails while direct reads show `0x2000:08` correct and `0x607C` zero, stop blaming stale software-side writes; the problem is more likely drive-side absolute-reference validity/retention than command-path offset residue.

### 2026-04-12 - Manual review ties the cold-boot delta to HM statusword bit 15
- [tool] The A6-EC manual states in homing mode that `6041h` bit 15 means `Homing completed` and bit 12 means `Homing completion output`, while `607Ch` is active only when the drive is powered on, homing is complete, and `6041h` bit 15 is 1.
- [self] New diagnosis rule: the observed `0x9650 -> 0x1650` transition is not random; it is exactly a loss of bit 15 with the other HM-related bits still present. Treat that as evidence that the drive remembers some homing-related status but does not consider homing fully completed/active after cold boot.

### 2026-04-12 - Hidden C10 absolute-position offsets are zero after cold boot
- [tool] Privileged reads of the EEPROM-backed C10 candidates (`0x2010:11`, `0x2010:13`, `0x2010:1F`) returned `0` on all six axes after the latest power cycle, while `0x6064` still showed the shifted cold-boot positions.
- [tool] The wrong pose was already present in `logs/startups/20260412-181258/controller.log` on the first startup `GET_POSITION` (`J1..J6 = -1.0527, 2.7765, -0.6963, -0.7260, -1.8808, -5.1380 deg`), before any new home/power-up/jog interaction.
- [self] Corrective rule: if EEPROM-backed absolute-position offset objects are all zero and the shifted pose appears immediately at boot, stop attributing the drift to a stale software-written offset; the drive is booting into that alternate reference on its own.

### 2026-04-12 - Manual plus live probes point to reference-state loss, not a hidden saved bias
- [tool] The A6-EC manual says `607Ch` is active only when powered on, homing is complete, and `6041h` bit 15 is 1; the HM statusword table defines bit 15 as `Homing completed` and bit 12 as `Homing completion output`.
- [tool] The latest wider drive probe showed many live `0x2040:*` position-like channels numerically track the shifted `0x6064` values, while the readable `0x2010:*` bias/limit fields remain zero. That means the shifted pose is coming from the drive's live internal reference state, not from an obvious persisted offset register.
- [self] New diagnosis rule: when multiple live `0x2040` position channels agree with the cold-boot-shifted `0x6064` counts, treat the shifted pose as the drive's own current truth and debug why the drive re-established that truth after boot instead of looking for more hidden software-written bias slots.

### 2026-04-12 - Fault reset and software reset do not restore HM bit 15
- [tool] A disarmed live probe showed `/control/reset-faults` leaves all axes at `0x6041 = 0x1650`, `0x607C = 0`, and the same shifted `0x6064` counts; it does not recover the pre-power-cycle pose or reassert HM bit 15.
- [tool] Writing vendor software reset `0x2031:02 = 1` briefly dropped RTCore startup health (`startup_ready=0`, `wkc_actual=11`) while drives re-enumerated, then recovered to a healthy disarmed bus (`startup_ready=1`, `wkc_actual=18`) with the shifted counts still present and HM bit 15 still absent.
- [tool] The software-reset probe transiently faulted axis 1 (`0x603F = 0x8700`, `0x203F = 0x0C20`), and a normal `/control/reset-faults` cleared it back to `0x6041 = 0x1650` / zero fault registers on all axes.
- [self] Diagnostic rule: if both DS402 fault reset and vendor software reset fail to restore bit 15 or the pre-boot pose, stop treating the mismatch as a stale software latch; focus next on manufacturer boot/reference-validity conditions or on whether only a full native-home/HM cycle can re-establish the reference-active state.

### 2026-04-12 - A6-EC rotation mode must not boot with default 1:1 gear ratio
- [tool] Manual review of Chapter 5 plus live SDO reads showed all axes were booting with `C00.07 = 4` while `C10.18 = 1`, `C10.19 = 1`, `C10.1A = 0`, and `C10.1C = 0`, so the drive was reconstructing rotation-mode absolute position from the vendor default `1:1` model instead of the robot's real reductions.
- [tool] The robot config already carries the correct per-axis reductions `[100, 100, 100, 18, 31.25, 10]`, which map to startup ratio SDOs `C10.18/C10.19 = [100/1, 100/1, 100/1, 18/1, 125/4, 10/1]`.
- [self] Implementation rule: for A6-EC absolute rotation mode, program the rotation-mode gear-ratio startup SDOs alongside `C00.07`; otherwise the drive may boot into a valid-but-wrong internal absolute frame even when battery-backed encoder data still exists.
- [self] Manual caveat: Chapter 5 states changing the electronic gear ratio changes the mechanical position abruptly and requires homing, so this fix needs one explicit post-deployment re-home to seed the corrected EEPROM reference, but should remove the need to re-home on every later cold boot.

### 2026-04-12 - Do not conflate drive reference scaling with encoder retention root cause
- [user] The robot's gear ratios are intentionally owned in software, and changing drive-side ratio parameters is not an acceptable first-line fix for the cold-boot retention bug.
- [self] Corrective rule: when a manual parameter seems related to absolute-position math, distinguish "changes drive-side reference-unit scaling" from "changes raw encoder retention/state"; do not ship a startup-parameter fix that crosses that ownership boundary without stronger proof.
- [self] Follow-up rule: treat the current cold-boot problem as "the drive is not reapplying its saved absolute-reference correction after power loss" until proven otherwise, not as a software gear-ratio mismatch.

### 2026-04-12 - Large PDF-to-Markdown conversions should preserve tables in fenced text blocks
- [user] Explicit preference reinforced: do not just describe the approach; actually produce the converted artifact in the requested folder.
- [tool] `pdftotext -layout` preserved the A6-EC chapter table geometry well enough to generate `docs/resources/a6ec_manual_chapter_11_parameter_list.md` without introducing OCR dependencies.
- [self] Corrective rule: for very wide or multi-page manual tables, do not force lossy Markdown pipe tables. Use normal Markdown headings and notes around fenced `text` blocks so the content stays readable and faithful to the source layout.
- [self] Cleanup rule: when a conversion requires an intermediate extracted text file, delete that temp artifact before handoff so the repo only keeps the requested deliverable.

### 2026-04-12 - Native home rewrites reference units, not raw encoder channels
- [tool] On axis 0 after a verified `home-joint-native`, the reference-unit channels (`0x6063`, `0x6064`, `U40.14`, `U40.16`) collapsed from about `38328` counts to about `4`, while the raw encoder-oriented channels (`U40.1C`, `U40.20`) stayed near `38331`.
- [self] Diagnostic rule: treat the A6-EC native-home transaction as a drive-side reference-frame transform layered on top of the raw absolute encoder state; if cold boot loses the pose, debug the save/restore of that transform, not the raw encoder battery counts.
- [tool] Direct `F31.10 = 1` (`Read encoder`) and `F31.10 = 2` (`Write encoder`) on the same disarmed axis completed without faults, self-cleared back to `0`, and left the axis back at `0x6041 = 0x9650`.
- [self] Next-step rule: the decisive proof now requires a real power cycle after `F31.10` read/write to see whether that operation commits or reloads the missing absolute-reference correction across boot.

### 2026-04-12 - `F31.10` read/write preserved the homed reference across drive-only power cycle on one axis
- [tool] After a drive-only power cycle with the stack left running, axis 0 returned with `0x6041 = 0x1650` but its reference-unit channels still near zero (`0x6063/0x6064/U40.14/U40.16 ~= 1..4`), while its raw encoder-oriented channels stayed near `38330` (`U40.1C/U40.20`).
- [tool] Untouched axes 1-5 all returned in the old shifted frame, with both reference-unit and raw encoder-oriented channels still near their cold-boot counts (`101087`, `25347`, `4758`, `21396`, `18704`, etc.).
- [self] Corrective rule: loss of HM bit 15 on cold boot does not by itself explain the wrong pose; axis 0 kept the corrected reference frame even after bit 15 dropped back to `0x1650`.
- [self] Strongest current hypothesis: an explicit `F31.10` encoder read/write commit or reload step is needed around native home so the drive restores the saved reference transform on later drive-only power cycles.

### 2026-04-12 - Integrated native-home persistence rollout preserves new axes too
- [tool] With the `F31.10` persistence tail integrated into the A6-EC native-home workflow, untouched axis 2 (`J3`) was brought from the shifted cold-boot frame (`0x6064 ~= 25346`, `U40.16 ~= 25344`) to the wrapped near-zero home frame (`0x6064 ~= 131062`, `U40.16 ~= -13`) through the normal `/control/home-joint-native` endpoint.
- [tool] After the subsequent drive-only power cycle, axis 2 stayed in that corrected frame (`0x6064 ~= 131060`, `U40.16 ~= 131059`, raw `U40.1C ~= 25334`), while untouched axes 3-5 remained in their shifted cold-boot frames.
- [self] Rollout rule: the persistence fix should live in the A6-EC native-home flow itself, not as a separate manual maintenance action, because the integrated endpoint now reproduces the same durable behavior as the earlier manual `F31.10` experiment.
- [self] Follow-up rule: the remaining timeout should be treated only as a deadman ceiling. Terminal success/failure should be driven by explicit signals (`native_home_state`, abort code, or statusword bit 15 on a fresh snapshot), not by the wall-clock alone.

### 2026-04-12 - Native-home result fields must reset on startup epoch and UI should prefer live proof over stale result
- [tool] After the integrated rollout, controller logs for `J2`/`J3` reported verified native-home success while `/run/gradient-rt-motion/metrics.json` still carried stale `native_home_state=3` and the old abort code until the next startup epoch.
- [self] Corrective rule: `native_home_state` and `native_home_last_abort_code` are last-operation fields, not durable drive state. Clear them when a new startup epoch begins so a drive reboot cannot keep an old red failure badge alive in the UI.
- [self] UI-facing telemetry rule: when the live statusword shows HM bit 15 with no current fault, prefer that fresh wire-state over a stale failed native-home result when building operator-facing drive-home status.
- [tool] After the cleanup rollout and stack restart, the live `driveFaults` snapshot for `J2` and `J3` reports `native_home_state_name = idle` and zero abort code, matching the persisted good frame instead of the earlier false `failed` badge.

### 2026-04-13 - A6-EC persisted-home feedback counts need signed single-turn normalization
- [tool] After the persistence rollout, `J2`/`J3` came back with `0x6064 ~= 131060` and `U40.16 ~= 131059`, which software had been converting to about `±3.6°` even though they semantically represented near-zero wrapped counts.
- [self] Corrective rule: for `a6ec_ds402`, normalize controller-facing feedback counts into a signed single-turn range using the encoder counts-per-rev before converting to joint radians; otherwise wrapped persisted-home values near `131072` render as false multi-degree joint offsets.
- [tool] After reloading the backend with that normalization, `/info/joints-detailed`, `/info/pose`, and the `/monitor` SSE stream all report `J2/J3` near zero (`about -0.00036° / +0.00033°`) instead of `±3.6°`.
- [self] Diagnostic rule: if the visualizer still appears to flicker after backend joint feeds are stable within a few encoder counts, the remaining bug is in the viewer layer rather than the controller/RTCore data path.

### 2026-04-13 - Native-home completion must wait for RTCore tail, not just early statusword success
- [tool] The `J5 -> J6` re-home issue lines up with a real race: the backend could return success as soon as statusword bit 15 appeared, even if RTCore was still executing the post-home `F31.10` persistence tail.
- [self] Corrective rule: treat native-home completion as "fresh success signal AND no native-home transaction still active for that axis", not just "statusword looks good once".
- [tool] Added an RTCore `native_home_active_axis_mask` metric and updated the Python wait logic so statusword-based success fallback is only allowed after that active mask clears.
- [self] Follow-up rule: fast consecutive Drive Home clicks on different joints should now naturally serialize because the first request stays pending until RTCore truly finishes the tail, instead of clearing early on a partial success signal.

### 2026-04-13 - PDF table conversions need presentation review, not just text completeness
- [user] Explicit correction: a manual-to-Markdown conversion that is text-complete but visibly messy is not acceptable; review the rendered presentation and fix extraction artifacts, not just the raw content.
- [self] Mistake: preserving wide manual tables as fenced raw text kept the chapter content but produced a poor reading experience and hid row-misalignment bugs.
- [self] Corrective rule: for PDF manuals with repeated column layouts, rebuild tables into real HTML/Markdown tables when possible, then spot-check rendered output for row drift, missing grouped-object rows, and page-number/header leakage.
- [tool] Working pattern: `pdftotext -tsv` provides enough positional data to reconstruct table rows, but grouped vendor objects (for example `607D`) may need special handling when the PDF drops the trailing `h` or merges subindex `0` with the object title.

### 2026-04-13 - Persist in-flight home state through existing driveFaults path only
- [user] Preference reinforced: reuse existing data pathways and avoid bloat; do not add a parallel frontend/API status channel just to carry home-in-progress state.
- [self] Corrective rule: route in-flight native-home status through the existing `drive_faults` snapshot (`metrics -> build_drive_fault_snapshot -> /monitor -> driveFaults prop`) rather than adding a new endpoint or frontend-only cache.
- [tool] The frontend now gets `native_home_active_axis_mask` and per-axis `native_home_active` from the existing `driveFaults` payload, uses that to show a persistent “still running” banner, and disables all Drive Home buttons until RTCore’s active mask clears.

### 2026-04-13 - Final all-axis power-cycle proof needs tolerance- and wrap-aware interpretation
- [tool] The formal retention comparison artifact for experiment `20260413-010728` still reports mismatch because it uses exact equality on controller-axis counts and logical angles.
- [tool] The raw post-power SDO snapshot shows the persisted-home frame survived on all axes within a few counts, and `J6` crossed the single-turn wrap boundary (`0x6064: 131071 -> 4`) while the underlying raw single-turn/multi-turn encoder channels stayed effectively unchanged (`U40.1C/U40.20: 18702/18701 -> 18704/18703`).
- [self] Corrective rule: for A6-EC persisted-home validation, interpret success using tolerance plus wrap equivalence, not strict equality of the reference-unit position channel.
- [self] Follow-up rule: if we keep the formal retention report as an operator artifact, update it to understand small count drift and modulo-equivalent near-zero values so it does not falsely mark a successful power-cycle proof as failed.

### 2026-04-13 - Backend native-home results must match live driveFaults semantics
- [self] The remaining `J2` false-failure existed because `_native_home_metrics_result()` only allowed statusword-bit15 fallback when the raw metrics did not already say `failed` and the abort code was zero, while `drive_faults` already treated the same clean live wire-state as effective success.
- [self] Corrective rule: once `native_home_active_axis_mask` has cleared, treat a clean live wire-state (`statusword` HM bit 15 set, `error_code == 0`, `manufacturer_error_code == 0`) as authoritative for backend command-result semantics too; preserve the stale reported failure state and abort code separately for debugging instead of surfacing a hard failure.
- [tool] Efficient validation pattern: when a safety-critical commissioning bug is already captured in `/run/gradient-rt-motion/metrics.json`, prove the semantic fix against that live snapshot directly instead of re-triggering another physical home cycle.

### 2026-04-13 - Display-friendly A6-EC feedback must stay out of the motion command frame
- [tool] During the `J6` jog regression, live RTCore status showed `state=faulted`, `active_traj_id=3`, `queue_depth=24`, and faulted axes `0/1/3` while `/info/joints-detailed` still reported display-normalized near-zero angles for wrapped raw counts like `130908` and `130694`.
- [self] Root-cause rule: A6-EC single-turn normalization / continuous unwrapping is UI-only. Any backend path that feeds `servo_driver.get_current_arm_state_rad()` or queued RTCore targets must preserve the raw-wire-derived count frame so re-queueing the current joint state round-trips back to the same `0x607A` counts after RTCore subtracts `native_home_position_offset`.
- [self] Corrective rule: keep two explicit feedback paths: motion-safe controller feedback (`raw_to_joint_positions`, `/info/joints`, monitor `joints`) and display-only feedback (`raw_to_display_joint_positions`, `arm_display_*`, monitor `display_joints`). Never reuse display-normalized angles as the baseline for `GET_JOINT_ANGLES` / `APPLY_JOINT_SETPOINT`.
- [tool] Safe live recovery pattern: use `/control/stop` first to collapse a latched RTCore queue (`active_traj_id -> 0`, `queue_depth -> 0`), then request `/control/power-down` so `armed=0` and `axis_enable_mask=0` before more debugging or code rollout.

### 2026-04-13 - Use `start-stack.sh` recovery paths when preflight faults block restart
- [tool] After the software fix, `./start-stack.sh` refused to restart the controller/API because startup preflight still saw disarmed drive faults; a cold `stop --hard` / restart reduced the blocker to `J1/axis0 0xff00 Er11.0`, but the preflight reset still would not clear it.
- [self] Recovery rule: if `start-stack.sh` aborts on startup fault-reset preflight, do not leave RTCore half-up or try to outsmart the launcher with ad-hoc child-process kills. Use `./start-stack.sh stop --hard` to return to a fully inactive state, then treat the remaining drive fault as a hardware/startup blocker for the next live retest.

### 2026-04-13 - Multi-turn display truth should stay raw in RTCore and anchored in Python
- [user] Preference reinforced: when implementing a staged architecture plan, execute it directly, do not edit the plan file itself, and move through the existing to-dos in order.
- [self] Corrective rule: publish A6-EC multi-turn objects from RTCore as raw per-axis `absolute_feedback` fields keyed by the actual `U40.*` object names; keep source selection, anchor math, and display semantics in Python so RTCore stays transport-only for this feature.
- [self] Guardrail: the safe persisted anchor for continuous display is `absolute_axis_q - reference_pre_zero_q`, where `reference_pre_zero_q` is the current raw `0x6064` logical frame before software-zero subtraction. Refresh that anchor on verified native-home completion and on explicit software-zero capture; never derive motion targets from it.
- [tool] Validation pattern that worked: `make -C src/gradient_rt_motion`, focused `pytest` on backend / drive-fault / API / RTCore runtime suites, and `cd /home/pi/GradientOS/web-ui && npm test -- src/ControlPanel.test.tsx`.
- [self] Mistake caught quickly: after inserting the new absolute-feedback helpers into `drive_faults.py`, the `native_home_state_name()` return was accidentally left below the new helper block. A focused regression rerun caught the broken labels immediately; after helper insertions, re-read the surrounding function boundaries before moving on.

### 2026-04-13 - Absolute-feedback descriptors must stay in the drive profile
- [user] Preference reinforced: do not hardcode vendor-specific items into shared OS/controller code; keep manufacturer object maps, labels, and source policy in drive/profile modules.
- [self] Mistake: I initially hardcoded A6-EC `U40.*` field names/source priority in `src/gradient_os/arm_controller/backends/ethercat_rtcore/backend.py`, `src/gradient_os/telemetry/drive_faults.py`, and literal SDO reads in `src/gradient_rt_motion/main.cpp`, which violated the profile boundary even though the motion/display split was otherwise correct.
- [self] Corrective rule: RTCore should accept a profile-rendered `GRADIENT_RT_ABSOLUTE_FEEDBACK_CONFIG` descriptor, export generic per-field metrics keyed by profile-provided semantic names, and let the drive-profile registry normalize/resolve absolute display truth for Python consumers.
- [tool] Validation that caught the last gap: focused `pytest` failed when `test_build_drive_fault_snapshot_normalizes_absolute_feedback` stubbed a fake drive profile; routing the test through the real `a6ec_ds402` profile verified the new registry-based normalization path.
- [self] Supersedes the narrower rule above: raw absolute feedback still belongs in RTCore transport, but the field names/object map must be descriptor-driven from the drive profile rather than hardcoded as `U40.*` keys in shared code.

### 2026-04-13 - After hard stop + drive power cycle, separate healthy bus state from missing startup verification telemetry
- [tool] `./start-stack.sh probe` after the hard stop/power-cycle showed a clean live state: `physical_state=BUS_UP_DISARMED`, `armed=0`, `enable_mask=0x0`, `master_al=0x8`, `responding/online/operational=6/6`, and all axes `SwitchOnDisabled` with `err=0x0000`.
- [tool] RTCore journal confirmed the startup SDO write `a6ec_encoder_position_tracking_mode=4` succeeded on all six axes and the bus converged to OP in about `8.7s`, with native-home truth refresh reading `0` offsets on all axes immediately after startup.
- [self] Guardrail: when post-restart health looks good but `metrics.json` still shows `startup_drive_config.readback_valid=0` and `verified=0` for every axis minutes later, treat that as a startup-verification telemetry/readback gap, not as evidence that the bus or drives are faulted.
- [tool] Live `/info/joints-detailed` showed every axis using `display_source=raw_feedback_fallback` with no `absolute_home_anchor_*` fields, so the UI is not currently applying a persisted absolute-home anchor after this restart.

### 2026-04-13 - Judge live jogs by display delta plus absolute-count delta, not by wrapped raw counts alone
- [tool] After `SAFE_POWER_UP`, the controller logged a J4 jog from `-19.943 deg` to `-20.943 deg`, then timed out waiting for RTCore trajectory `1` to report complete, even though the post-jog live probe showed all six axes still `OperationEnabled` with `err=0x0000`.
- [tool] Comparing pre-jog vs post-jog live snapshots: J4 `arm_rad` appeared to jump by about `+19.0 deg` because `pos_counts` wrapped from `130692 -> 6179`, but J4 `arm_display_rad` changed by `-1.001 deg` and `absolute_counts` changed by `+6559`, which is the real commanded small move.
- [self] Corrective rule: when a powered-on jog seems to change many displayed numbers, compare `arm_display_rad` and `absolute_counts` before concluding there was a multi-axis physical move; the raw motion-safe frame can wrap on a single axis and look dramatic while the operator-facing delta is correct.
- [tool] Residual issue: the controller-side open-loop executor can still raise `TimeoutError: Timed out waiting for RTCore trajectory 1 to complete` even when the live end state is healthy and enabled, so trajectory-completion bookkeeping still needs investigation separately from drive faults or display wrapping.

### 2026-04-13 - Wrapped motion completion must be profile-owned and modulo-aware
- [self] If a drive keeps the motion wire frame in wrapped single-turn `0x6064` / `0x607A` counts, RTCore trajectory completion must compare final target error modulo `counts_per_rev`; otherwise a boundary-crossing jog can physically finish yet stay latched in `executing`, which blocks the commissioning UI.
- [self] Keep that behavior profile-owned: expose a generic motion-feedback-wrap capability from the drive profile, render it into RTCore runtime env/CLI, and let generic RTCore code apply shortest-periodic-error math only when the active drive profile opts in.
- [tool] Local validation after adding `GRADIENT_RT_FEEDBACK_WRAP_AXIS_MASK` / `--feedback-wrap-axis-mask`: `pytest -q tests/test_rtcore_runtime.py` and `pytest -q tests/test_gradient05_limits_and_backends.py -k wait_for_trajectory_complete` both passed.

### 2026-04-13 - A stale RTCore D-state thread can keep EtherCAT master reserved across restarts
- [tool] After syncing the new RTCore binary, the old deleted-binary instance remained as a `metrics` thread in `D` state plus a live `EtherCAT-OP` kernel thread, while fresh RTCore instances logged `ecrt_request_master(0) failed` with `Device or resource busy`.
- [tool] `sudo ethercat master` still showed `Active: yes` and 1 kHz frame traffic even when the fresh RTCore instance reported `ethercat_master_state=DOWN`, which proved the stale master owner lived outside the new process.
- [self] Recovery rule: if `ps -L` shows an old deleted-binary RTCore thread stuck in `D` and repeated `ethercat.service` / `gradient-rt-motion.service` restarts do not clear master ownership, stop looping service restarts and escalate to a host reboot or deeper kernel/driver recovery.

### 2026-04-13 - `start-stack.sh stop --hard` can be correct even when RTCore survives
- [tool] The launcher hard-stop path does call `systemctl stop` on `gradient-rt-motion.service`, waits, escalates with `systemctl kill --signal=SIGKILL`, and then warns if RTCore/ethercat are still not inactive.
- [self] Corrective rule: if `stop --hard` appears not to stop RTCore, inspect `journalctl` and `ps -L` before blaming the launcher. In this failure mode, the launcher logic was correct; the real blocker was a kernel-blocked RTCore thread in `D` state that systemd could not reap.
- [tool] A full Pi reboot cleared that stale owner; afterward `gradient-rt-motion.service`, `ethercat.service`, and the launcher stack all came back up cleanly on fresh PIDs.

### 2026-04-13 - Operator-facing web pose must use `display_joints`, not wrapped motion-safe `joints`
- [tool] A 30 s live capture of `http://127.0.0.1:4000/monitor` recorded 1389 packets; raw `joints` jumped between wrap-adjacent poses on every axis (for example max single-step deltas `0.062831`, `0.349063`, `0.201060`, `0.628314` rad) while `display_joints` stayed stable within micro-radians for the entire trace.
- [tool] A 6 s poll of `/info/joints-detailed` showed `axis_counts` rapidly alternating among `0`, `1`, `131071`, and `131070` after reboot/power-up, confirming the visible jump was modulo-boundary dithering in the raw `pos_counts` frame rather than the robot physically moving.
- [self] Corrective rule: for A6-EC wrapped feedback near zero, keep `/monitor.joints` as the motion-safe/debug channel only. Any operator-facing UI pose (3D stage, commissioning readout, telemetry chart) must prefer `/monitor.display_joints` or `/info/joints{,-detailed}.arm_display_*`.
- [tool] Minimal UI fix that worked: parse `display_joints` in `web-ui/src/App.tsx` and route the preferred display pose into the visualizer/telemetry widgets; leave the raw `joints` payload intact for non-display consumers.

### 2026-04-13 - J3 commissioning jogs are driven by unstable raw baseline, not by display telemetry leaking into motion
- [tool] Controller logs for the live J3 jog sequence show bounded moves computed from raw `current_deg` values that jumped between `-3.569`, `-0.970`, `-1.970`, `-2.970`, and `-0.371` on successive commands, while each target remained just `-1.0 deg` from that current sample.
- [self] Corrective rule: when diagnosing wild commissioning motion on EtherCAT RTCore, distinguish "display stream leaked into controller" from "controller is using raw motion-safe feedback that is itself unstable." The current code path still uses raw `GET_JOINT_ANGLES` / `get_current_arm_state_rad()` for motion.
- [tool] Code proof: `/control/joint-jog` previously built its relative target from controller `GET_JOINT_ANGLES`, `GET_JOINT_ANGLES` came from `_build_joint_state_snapshot().arm_deg`, and `handle_apply_joint_setpoint(... max_motor_rpm=...)` built the bounded path from a fresh `servo_driver.get_current_arm_state_rad()` sample.
- [tool] Live J3 snapshot after the investigation still reports `absolute_source=encoder_multi_turn_counts` but `display_source=raw_feedback_fallback`; there is no persisted `.gradient_absolute_encoder_anchors.json` file yet, so the fully anchored absolute display path is not active on this boot.
- [self] Safe implementation rule: improve jog observability first. The updated `/control/joint-jog` now uses `GET_JOINT_STATE` for its pre-command snapshot and logs/returns selected-joint raw-vs-display diagnostics (`current_raw_deg`, `current_display_deg`, `raw_minus_display_deg`, `display_source`, `absolute_source`, counts) without changing motion-target semantics.

### 2026-04-13 - Canonical anchored joint truth now supersedes the temporary display-truth workaround
- [self] Supersedes the earlier `display_joints`-preferred workaround for operator pose: once `EthercatRTCoreBackend.raw_to_joint_positions()` / `get_joint_positions()` publish anchored absolute truth, `arm_rad/deg` and `/monitor.joints` become the canonical operator/controller fields and `arm_display_*` / `display_joints` should be treated as compatibility aliases only.
- [self] New guardrail: if anchored absolute truth cannot be reconstructed (`absolute_feedback` missing or no persisted absolute-home anchor), fail closed with `canonical_joint_truth_available=false` and block `POST /control/joint-jog` baselining instead of silently reusing wrapped raw counts or cached fallback state.
- [tool] Regression coverage that now matters most for this contract lives in `tests/test_gradient05_limits_and_backends.py`, `tests/test_api_endpoints.py`, and `web-ui/src/ControlPanel.test.tsx`; targeted pytest/vitest/build checks passed on this machine after the change.

### 2026-04-13 - Startup should classify RTCore-up/master-down before launching the controller
- [self] If `sync-runtime.sh --ensure-active` reports `gradient-rt-motion.service already active; runtime config is current`, do not assume the RTCore is usable. Check the live probe: `rtcore_state=UP` combined with `ethercat_master_state=DOWN`, `physical_state=INACTIVE`, and `startup_ready=0` is an unhealthy prelaunch signature, not normal bus convergence.
- [self] Corrective rule: for that signature, attempt exactly one launcher-driven RTCore/EtherCAT recycle before controller startup, and only escalate to "reboot required" if the post-recycle journal still shows `Failed to reserve master: Device or resource busy` / `ecrt_request_master(0) failed`.
- [self] RTCore should fail fast on `ecrt_request_master(0)` failure so systemd/launchers see a real service failure instead of an "active but dead" process that leaves `/run/gradient-rt-motion/ipc.sock` and misleading `rtcore_state=UP` behind.

### 2026-04-13 - Operator CLI banner belongs in `start-stack.sh`, not low-level launch helpers
- [user] Preference: the boot/start CLI should feel polished, with a big `GradientOS` banner plus useful live info instead of plain startup logs only.
- [self] Implementation rule: put operator-facing terminal chrome in `start-stack.sh`, because that is the staged stack launcher users see directly. Keep `run.sh` and lower-level helpers minimal so systemd/manual subprocess logs do not get decorative noise.
- [tool] A good startup banner should carry actual operational context, not just art: mode, robot, IK/backend/drive profile, ports, log path, and the common control commands (`probe`, `status`, `stop`, `stop --hard`).
- [self] Color rule: auto-enable ANSI color only when a real interactive terminal is present, respect `NO_COLOR`, and offer a simple `GRADIENT_STACK_COLOR=auto|0|1` override so startup output stays readable in logs and non-interactive launches.
- [user] Design preference: terminal styling should lean industrial/cinematic, not generic devtool output. Use highlighted caution/status bars and give urgent operator actions like `REBOOT HOST` or `stop --hard` their own visibly separated callouts rather than burying them inside long prose log lines.
- [user] Success-state preference: startup should celebrate a truly ready system, not just failures. Add a clear bright-green success indicator only after the controller, RTCore/bus, API, and optional web UI have all passed readiness.
- [self] Keep terminal animation tty-only. For `start-stack.sh`, write spinners/loading indicators only to the live terminal and keep the launcher log factual; otherwise carriage-return animation pollutes the durable startup logs.
- [self] If a lower-level helper like `start.sh` already prints plain bootstrap text, add a quiet-mode env flag and let the staged launcher present a single branded flow instead of mixing two output styles.
- [self] Shell guardrail: with `set -u`, any styled logger wrapper that passes `${UI_*}` or `${BANNER_*}` into another function will crash if those globals have not been initialized yet. Predeclare all palette globals near the top of `start-stack.sh` before the first `log()` call.
- [tool] Real staged timing data matters more than guessed spinner placement. Live startup timing showed `environment` around `66-68ms`, first `RTCORE SYNC COMPLETE` around `17.3s`, `ethercat.service stopped` around `38.7s`, and full recovery recycle around `42.0s`; those are the stages worth keeping animated/high-visibility.
- [self] ASCII logo guardrail: keep every art row indented consistently. A one-character left shift on the middle lines makes the `GradientOS` mark look broken even if the glyphs themselves are correct.
- [user] Probe output preference: when the launcher dumps a live probe snapshot during failure/recovery, it should stay in the same visual language as the rest of the terminal UI. Avoid raw `probe: key=value ...` lines once styled panels/status colors exist.
- [self] Animation rule: a single `ui_loading_status(...)` repaint is not a true animated loader for blocking shell commands. For long one-shot operations like RTCore sync or `systemctl stop`, use a background tty-only spinner process that redraws in place until the command exits.
