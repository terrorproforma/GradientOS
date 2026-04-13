# Native Home False-Failure Handoff

## Scope

This handoff is for a fresh agent taking over the current unresolved issue:

- after a hard stop + restart of the controller/API stack, running drive-native home on `J2`
  can return a **failed verification / abort `0x06010002`** at the command-response level
- but the **live drive state then shows that `J2` is actually homed successfully**
- the user explicitly wants the agent to **fix this properly** with **no half measures**
- the user also explicitly asked to **reuse existing data pathways** and **avoid bloat / parallel status channels**

This file includes:

- the overall investigation history that matters to the current bug
- the current branch state
- what has already been implemented and validated on hardware
- the exact current inconsistency
- the likely next debugging/fix directions

## User Requirements and Preferences

- Use the existing data path wherever possible. Do not add a parallel API or frontend-only status channel if the current telemetry path can carry the data.
- Operator-facing joint angles should be **continuous multi-turn angles**. Example from the user:
  - `J6` should be able to show `720°`, `-540°`, etc.
  - it should not wrap back to `0°` just because one internal count channel wraps.
- Implement the fix; do not stop at explanation-only answers.
- The user asked previously that no subagents be used for the hardware investigation.
- The user wants the handoff to leave no relevant information out.

## Branch / Repo State

Current branch:

- `multi-robot-architecture`

Current dirty worktree at handoff time:

- `.cursor/memory/AGENT_SCRATCHPAD.md`
- `.cursor/memory/DEVLOG.md`
- `src/gradient_os/api/main.py`
- `src/gradient_os/arm_controller/backends/ethercat_rtcore/backend.py`
- `src/gradient_os/arm_controller/profiles/drive/a6ec_ds402.py`
- `src/gradient_os/telemetry/drive_faults.py`
- `src/gradient_rt_motion/main.cpp`
- `tests/test_api_endpoints.py`
- `tests/test_drive_faults.py`
- `tests/test_gradient05_limits_and_backends.py`
- `tests/test_rtcore_runtime.py`
- `web-ui/src/App.tsx`
- `web-ui/src/ControlPanel.test.tsx`
- `web-ui/src/ControlPanel.tsx`
- `web-ui/src/liveState.tsx`
- `docs/resources/a6ec_manual_chapter_11_parameter_list.md`
- `docs/resources/chapter 11 - parameter list - A6-EC_series_servo_drive_manual (2).pdf`
- plus unrelated pre-existing dirty state in `src/numeric_solver/quik`

Important:

- do **not** revert unrelated user changes
- the memory/devlog files are intentionally updated during this investigation

## Big-Picture Investigation History

### 1. Original native-home persistence problem

Earlier in this session, the stack proved that:

- the drive-native home itself can succeed
- but after a real drive-only power cycle, the pose did **not** survive reliably
- the key breakthrough came from probing `F31.10`:
  - `1 = Read encoder`
  - `2 = Write encoder`

Manual references that mattered:

- `docs/resources/A6-EC_series_servo_drive_manual.pdf`
- `docs/resources/a6ec_manual_chapter_11_parameter_list.md`
- `docs/resources/a6ec_manual_codes.json`

Key manual facts established:

- the absolute encoder stores single-turn + multi-turn state with battery backup
- after homing, the drive stores the deviation between mechanical absolute position and saved encoder state
- `F31.10` has explicit `Read encoder` and `Write encoder` operations
- `0x607C` is not the whole story; the persisted post-home frame involves a drive-side reference transform layered on top of the raw encoder state

### 2. Persistence rollout that is already implemented

The A6-EC native-home transaction was extended so that after the HM/home capture succeeds, the integrated flow does:

- `refresh_truth`
- `restore_mode`
- `release_service_override`
- `F31.10 = 1` and wait for it to clear
- `F31.10 = 2` and wait for it to clear

This was implemented in:

- `src/gradient_rt_motion/main.cpp`
- `src/gradient_os/arm_controller/profiles/drive/a6ec_ds402.py`

Generic RTCore transaction support added:

- `wait_sdo`
- `release_service_override`

The backend/API wait logic was also improved:

- native-home timeout ceiling increased
- verification can use fresh statusword bit 15 fallback
- completion can be delayed until RTCore’s native-home tail actually finishes

Relevant files:

- `src/gradient_os/arm_controller/backends/ethercat_rtcore/backend.py`
- `src/gradient_os/api/main.py`
- `src/gradient_os/telemetry/drive_faults.py`
- `web-ui/src/ControlPanel.tsx`

### 3. Persistence proof that was already achieved

The persistence rollout was already validated on hardware before the current bug:

- untouched axes were homed with the integrated endpoint
- drive-only power cycles were performed
- raw drive objects showed that the semantic post-home frame survived

Important nuance:

- the formal retention comparison artifact still uses exact equality and does **not** understand small count drift or modulo-equivalent near-zero values
- raw drive proof indicated success even when the formal artifact still said mismatch

Experiment id for the final all-axis retention proof:

- `20260413-010728`

Relevant artifact paths:

- `logs/encoder-retention/20260413-010728/before_power_down.json`
- `logs/encoder-retention/20260413-010728/after_power_up.json`
- `logs/encoder-retention/20260413-010728/comparison.json`
- `logs/encoder-retention/20260413-010728/comparison.md`

Interpretation:

- raw drive state showed the all-axis persistence rollout was successful
- the comparison report is too strict and should later be made tolerance- and wrap-aware

## Operator Display / Angle Display Work Already Done

### 1. Stale false-failure cleanup

There was a previous bug where the frontend could keep showing false native-home failures after restart because RTCore metrics carried stale `native_home_state` / `native_home_last_abort_code`.

Fixes already implemented:

- RTCore clears those last-operation fields on a new startup epoch
- drive-fault snapshot prefers current wire-state where appropriate instead of stale last-operation fields

Files:

- `src/gradient_rt_motion/main.cpp`
- `src/gradient_os/telemetry/drive_faults.py`

### 2. Wrapped near-zero angle fix

There was another bug where persisted-home axes such as `J2` / `J3` came back with values like `131060` in controller-facing counts and were displayed as about `+/-3.6°` instead of near zero.

That was fixed by normalizing A6-EC feedback counts into a signed single-turn range before converting to radians.

Important: this fixed the **persisted-home near-zero interpretation**, but it is **not** the final continuous multi-turn operator display behavior the user wants.

The user explicitly wants:

- all displayed joint angles to continue counting on a continuous number line
- not to wrap to zero
- example: `J6` should show `720°`, `-540°`, etc.

Current code state on branch:

- the backend now has `counts_per_rev` in `_AxisConfig`
- there is `_normalize_feedback_counts_for_axis()`
- there is `_display_feedback_counts_for_axis()`
- `raw_to_joint_positions()` currently uses `display_counts = _display_feedback_counts_for_axis(...)`

This means the current branch is attempting to move from wrapped interpretation toward a continuous operator display, **but the current live proof below still needs to be understood correctly**.

### 3. Live in-flight home state through existing path

The user asked to persist “still working” state to the frontend and block clicking another home button, but only through existing data pathways.

Implemented path:

- RTCore metrics now include `native_home_active_axis_mask`
- `build_drive_fault_snapshot()` carries:
  - top-level `native_home_active_axis_mask`
  - top-level `native_home_active_axis_mask_hex`
  - per-axis `native_home_active`
- the existing `/monitor` SSE payload carries the same `drive_faults` object
- `web-ui/src/ControlPanel.tsx` uses that existing `driveFaults` prop to:
  - show persistent “still running” messaging
  - show per-axis “Drive Home requested...”
  - disable all Drive Home buttons while a home is active

No new endpoint was added for this.

## Latest Restart / Current Runtime Context

At handoff time, the latest stack run is:

- `logs/startups/20260413-021353`

Current `start-stack` status before handoff:

- launcher running
- controller up
- API up
- web up

Current RTCore metrics summary at handoff:

- `armed = 0`
- `axis_enable_mask = 0`
- `native_home_active_axis_mask = 0`
- `startup_ready = 1`
- `startup_reset_count = 0`
- `wkc_actual = 18`
- `wkc_expected = 12`

## Current Unresolved Issue

### User-facing symptom

After the user performed a hard stop + restart and then tried to home `J2`, the UI showed:

- a global red commissioning error:
  - `Drive-native commissioning home failed verification. Abort code 0x06010002.`
- but the joint row for `J2` showed success-like state

The user-provided screenshot is:

- `/home/pi/.cursor/projects/home-pi-GradientOS/assets/c__Users_angus_AppData_Roaming_Cursor_User_workspaceStorage_70b7af93d0dca9a0af5d7dafe24517d5_images_image-f1542f47-101b-4635-938c-859cb0f28e0c.png`

From the screenshot:

- the top global commissioning message is red / failure
- `J2` row shows `Drive Home succeeded | offset 0 cnt | axis currently disarmed`

This is the contradiction the fresh agent needs to fix.

### Latest logs for the failing attempt

Latest controller log evidence:

- file: `logs/startups/latest/controller.log`
- relevant lines:
  - received `NATIVE_HOME_JOINT,2`
  - immediately logged:
    - `native drive-home failed verification: joint=2 axis_mask=0x2 abort=0x06010002`

Latest API log evidence:

- file: `logs/startups/latest/api.log`
- the endpoint returned `200 OK`
- meaning the HTTP route itself completed, but the payload likely represented domain failure

### Current raw RTCore metrics for J2 at handoff

From `/run/gradient-rt-motion/metrics.json` for axis 1 / `J2`:

- `native_home_state = 3`
- `native_home_last_abort_code = 100728834`
- that is hex `0x06010002`
- `statusword = 38480` which is `0x9650`
- `pos_counts = 131061`

This means the raw metrics field still says “failed”.

### Current live drive objects for J2 at handoff

Direct EtherCAT reads on axis `1` / `J2` at handoff:

- `0x6041 = 0x9650`
- `0x6061 = 8`
- `0x6063 = 131059`
- `0x6064 = 131060`
- `0x607C = 0`
- `0x603F = 0`
- `0x203F = 0`
- `0x2031:11 = 0`
- `U40.14 = 131060`
- `U40.16 = -13`
- `U40.1C = 101012`
- `U40.20 = 101010`

Interpretation:

- no live drive fault
- statusword indicates homing complete signal is present
- reference-unit channels are in the wrapped near-zero persisted-home frame
- raw encoder-oriented channels are stable and separate

This looks like **success on the drive**, not an actual failed home.

### Current live drive-fault snapshot for J2 at handoff

Building the live `driveFaults` snapshot right now yields for `J2`:

- effective `native_home_state = 2`
- `native_home_state_name = succeeded`
- `native_home_last_abort_code = 0`
- `native_home_verification_source = statusword_bit15`
- but also preserves the raw reported values:
  - `native_home_state_reported = 3`
  - `native_home_state_reported_name = failed`
  - `native_home_last_abort_code_reported = 0x06010002`

This is crucial:

- the **live snapshot path says success**
- the **raw last-operation metric field still says failed**
- the **command response / commissioning status path still appears to be surfacing failure**

## Strongest Current Diagnosis

The unresolved bug is **not** “the drive really failed to home”.

It is more likely one of these:

1. the integrated home transaction completes in a semantically successful state on the drive,
   but a later persistence-tail substep or verifier path still marks the command as failed
2. the raw per-axis `native_home_state` / abort code are still not being cleared or overwritten correctly after a successful integrated home on the same startup epoch
3. the API / frontend commissioning result path is still treating the raw failed metric field as authoritative even when the live drive state is clearly successful

The evidence strongly supports “drive succeeded, response contract false-negative”:

- controller log says failed
- raw metrics say failed
- direct drive objects say success
- derived live `driveFaults` snapshot says success
- screenshot shows exactly that contradiction: global error + row success

## Most Relevant Files for the Fresh Agent

### Core RTCore execution / metrics

- `src/gradient_rt_motion/main.cpp`

Relevant concepts already added there:

- integrated A6-EC native-home tail with `F31.10`
- `wait_sdo`
- `release_service_override`
- `native_home_active_axis_mask`
- startup-epoch clearing of stale native-home fields

### Python backend native-home result logic

- `src/gradient_os/arm_controller/backends/ethercat_rtcore/backend.py`

Relevant concepts:

- `_wait_for_native_home_result()`
- `_native_home_metrics_result()`
- statusword bit-15 fallback
- wait behavior gated by `native_home_active_axis_mask`
- count normalization / display interpretation

### API response shaping

- `src/gradient_os/api/main.py`

Relevant:

- `/control/home-joint-native`
- timeout currently lengthened for integrated flow

### Live UI telemetry derivation

- `src/gradient_os/telemetry/drive_faults.py`

Relevant:

- `derive_effective_native_home_status()`
- `build_drive_fault_snapshot()`

### Frontend rendering of native-home state

- `web-ui/src/ControlPanel.tsx`
- `web-ui/src/liveState.tsx`
- `web-ui/src/App.tsx`

Relevant:

- row-level native-home status text comes from `driveFaults`
- global commissioning banner comes from the command result path (`commissioningStatus`)

## Existing Tests Already Added

Relevant test files already modified in this session:

- `tests/test_gradient05_limits_and_backends.py`
- `tests/test_api_endpoints.py`
- `tests/test_drive_faults.py`
- `tests/test_rtcore_runtime.py`
- `web-ui/src/ControlPanel.test.tsx`

Important current test themes:

- native-home timeout / pending semantics
- drive-fault snapshot live-state fallback behavior
- in-flight active-home mask UI disable state
- backend count conversion behavior

## What the Fresh Agent Should Probably Check Next

### 1. Reproduce the contradiction directly

Use the latest running stack and:

- trigger `J2` native home again if safe, or inspect the last command result path without moving further
- compare:
  - controller/API command response
  - raw RTCore metrics axis fields
  - direct EtherCAT drive objects
  - derived `build_drive_fault_snapshot()` output

The key question:

- why does the integrated command still surface failure when the live drive state is already home-complete and fault-free?

### 2. Inspect where raw failure can survive despite effective success

Likely hotspots:

- in `main.cpp`, after `native_home_axis()`:
  - is `latest_feedback.native_home_state[axis]` ever left as failed by a substep even though the final drive state is good?
  - is there a later stale write or race that reassigns the failed state?
- in `backend.py`:
  - is the final response for `native_home_joint()` still too anchored to raw `native_home_state` / abort code when the live drive state is unambiguously successful?

### 3. Decide proper command semantics

Fresh agent will need to choose whether:

- the command should return **success** when the final live drive state is good, even if an internal persistence-tail substep transiently complained
- or return **warning / partial success**
- but it should almost certainly stop returning a hard failure in the current J2-shaped case

### 4. Keep using existing data pathways

Do **not** add:

- a new “native home status” endpoint
- a new frontend-only cache
- a second telemetry stream

Use the current path:

- RTCore metrics
- existing controller command reply
- existing `drive_faults` snapshot
- existing `/monitor` and existing command responses

## Current Manual / Semantic Conclusions the Fresh Agent Should Not Re-litigate

These were already proved well enough during this session:

- `F31.10 = 1/2` is the missing persistence mechanism for the A6-EC native-home frame
- the persistence tail belongs in the native-home workflow itself
- stale `native_home_state` / abort fields can lie after restart
- wrapped near-zero persisted-home values should not be shown as `±3.6°`
- the user wants continuous multi-turn operator display eventually
- the user does not want extra telemetry bloat

## Existing Artifacts Worth Reading

- latest controller log:
  - `logs/startups/latest/controller.log`
- latest API log:
  - `logs/startups/latest/api.log`
- final all-axis power-cycle proof:
  - `logs/encoder-retention/20260413-010728/`
- full earlier parent transcript:
  - [Jog frame fix](bec0f664-14af-4f89-960f-a2ab92692254)

## Current One-Line Summary

The branch has already solved native-home persistence and the major display issues, but after a hard stop + restart the integrated `J2` home can still return a **false failure** (`0x06010002`) even though the live drive state and effective frontend snapshot both indicate **success**. The next agent should fix that command-result / raw-metrics contradiction using the existing telemetry and reply pathways only.
