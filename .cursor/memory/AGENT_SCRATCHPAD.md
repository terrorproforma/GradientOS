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
