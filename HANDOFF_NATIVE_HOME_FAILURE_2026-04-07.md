# GradientOS Native-Home Failure Handoff

## Purpose

This handoff is for a fresh agent taking over the unresolved live hardware issue where clicking `Drive Home` in the UI still:

- deactivates/faults the joint
- leaves the displayed angle/offset wrong
- leaves the visualization inconsistent with the user's physical home position

Do not assume this is fixed. A source patch was made, RTCore was rebuilt and redeployed, but the live end-to-end behavior is still broken.

## One-paragraph summary

The original problem was that RTCore native home (`MSG_CMD_NATIVE_HOME`) explicitly removed the target axis from `axis_enable_mask`, so the brake/axis dropped out and the UI gave no useful feedback. I patched RTCore source to restore the previous `axis_enable_mask` and `armed` state after successful native home, added native-home telemetry to the Python/UI path, rebuilt RTCore, redeployed it, and verified the direct drive-fault reset path works safely. However, after the user clicked `Drive Home` again in the live UI, the same core problem still remained: J2 still deactivated/faulted, the displayed angle/offset was still wrong, and the visualization still did not reflect the true home position. The current live evidence strongly suggests the remaining issue is not just "restore enable mask", but a deeper post-home reference/hold-target/pose-sync problem.

## Current live state at handoff

These were the latest live reads at handoff time.

### RTCore metrics

From `/run/gradient-rt-motion/metrics.json`:

- `armed=1`
- `axis_enable_mask=63`
- `startup_ready=1`
- `wkc_actual=18`
- all 6 slaves online and operational

Per-axis highlight:

- J2 / axis1:
  - `native_home_state=2` (`SUCCEEDED`)
  - `native_home_position_offset=-368092`
  - `error_code=65280` (`0xFF00`)
  - `statusword=5688` (`0x1638`, faulted)

This is the key contradiction: native home is reported as successful, but the same axis is faulted afterward.

### Live API joint state

From `GET http://127.0.0.1:4000/info/joints-detailed`:

- `read_source="live_feedback"`
- `arm_deg=[0.018594360351562502, 3.3985931396484377, -1.7793731689453125, -19.265594482421875, -0.21911132812499998, -5.2734375]`
- `axis_error_code=[0, 65280, 0, 0, 0, 0, ...]`
- J2 is still being presented to the UI as live feedback at about `3.3986 deg`

The user says this is still not the true post-home physical reference, and the visualization is still wrong.

## What the user is upset about

The user's current complaint is valid and should be treated as the authoritative state:

- clicking `Drive Home` still produces the same failure
- the offset/angle shown after home is still wrong
- the visualization does not show the correct home position
- the joint still deactivates/faults

Do not argue from earlier partial fixes. Start from the latest live evidence above.

## What has already been changed in the repo

### RTCore / EtherCAT / controller genericization work already in progress

Large parts of the stack were already refactored before this debugging round:

- drive profile / EtherCAT details moved toward generic descriptor-driven config
- A6-EC startup mode uses `a6ec_encoder_position_tracking_mode`
- telemetry contract was made more generic
- UI/API/backend were updated to carry generic startup-drive-config info

This is real background context, but it is not the immediate blocker right now.

### Specific native-home and UI changes already made

These changes are already in the working tree:

- `src/gradient_rt_motion/main.cpp`
  - patched `MSG_CMD_NATIVE_HOME` to:
    - store prior `axis_enable_mask`
    - store prior `armed`
    - disable the target axis during the home SDO write
    - restore prior enable/armed state if native home succeeds
  - logs `Native-home restore: ...` on success

- `src/gradient_os/telemetry/drive_faults.py`
  - added native-home fields into drive telemetry

- `web-ui/src/liveState.tsx`
  - updated types for native-home telemetry

- `web-ui/src/App.tsx`
  - added stale-joint warnings / handling

- `web-ui/src/ControlPanel.tsx`
  - displays per-joint native-home status
  - software `Zero` hidden by default behind a settings toggle

- `src/gradient_os/arm_controller/backends/ethercat_rtcore/backend.py`
  - rejects stale joint feedback instead of silently returning cached values as live

- `src/gradient_os/run_controller.py`
  - telemetry only publishes joints when live feedback is actually available

- `.gradient_joint_zero_offsets.json`
  - reset to all zeros for `gradient-05`

### RTCore was actually rebuilt and redeployed

This matters because earlier in the session some C++ changes existed only in source.

This time RTCore was actually:

- built with `make -j4` in `src/gradient_rt_motion`
- installed via `./systemd/rt-motion/sync-runtime.sh --ensure-active`
- restarted as `/usr/local/bin/gradient-rt-motion`

So the current live failure is happening on the rebuilt service, not only on stale source.

## What was verified to work

These are real successes and should not be rediscovered from scratch:

- The bad A6-EC TxPDO layout bug was previously found and fixed:
  - `0x203F` had been incorrectly inserted into fixed TxPDO `0x1B02`
  - that shifted feedback offsets by 4 bytes
  - removing it restored real statusword / position truth

- The direct RTCore fault-reset pathway works safely:
  - targeted J2 reset was done with:
    - `PYTHONPATH=/home/pi/GradientOS/src /home/pi/GradientOS/.venv/bin/python /home/pi/GradientOS/scripts/rtcore_jog.py fault_reset --mask 0x2`
  - this cleared J2 fault while keeping:
    - `armed=0`
    - `axis_enable_mask=0`

- RTCore rebuild/redeploy path is known and working:
  - `make -j4`
  - `./systemd/rt-motion/sync-runtime.sh --ensure-active`

- The UI/API/controller stack can run and poll `GET /info/joints-detailed`

## What is still not fixed

The following are still unresolved and are the real handoff target:

1. Clicking `Drive Home` still causes the same operator-facing failure on live hardware.
2. J2 can end up with:
   - `native_home_state=2`
   - saved native-home offset
   - but also `0xFF00` fault afterward.
3. The post-home displayed angle is still not physically correct according to the user.
4. The visualization still does not show the correct post-home reference.
5. The axis still appears to deactivate/fault during or after the home workflow.

## Strongest current hypothesis

The remaining bug is probably not just "restore enable mask".

The likely failure is that native home changes the drive's internal position reference, but RTCore/controller/UI do not fully re-synchronize all of the following immediately afterward:

- RTCore hold target / commanded CSP target
- any pending target/trajectory/jog state
- the logical interpretation used by controller/UI
- potentially the DS402 enable/fault path if the drive sees a large step immediately after the offset change

This would explain the exact observed contradiction:

- drive home succeeds at the drive level
- offset is saved
- then a stale or mismatched commanded target causes a `0xFF00` family fault
- meanwhile the UI shows live feedback, but the live reference is not the user's expected physical home truth

## Highest-value files to inspect next

- `src/gradient_rt_motion/main.cpp`
  - `MSG_CMD_NATIVE_HOME`
  - `native_home_axis`
  - hold-target logic around `hold_target_counts`
  - any state that survives native home:
    - jog state
    - trajectory state
    - motion intent
    - target counts
    - output target counts

- `src/gradient_os/arm_controller/backends/ethercat_rtcore/backend.py`
  - native-home command path
  - joint readback path
  - any sync-to-feedback logic before/after arm / enable / home

- `src/gradient_os/run_controller.py`
  - telemetry publish path for joint truth
  - native-home command handling

- `src/gradient_os/telemetry/drive_faults.py`
  - how fault state and native-home state are combined

- `web-ui/src/ControlPanel.tsx`
  - what it does immediately after a native-home click
  - how it refreshes joint angles

- `web-ui/src/App.tsx`
  - how stage pose is fed from live state vs fallback

- `src/gradient_os/joint_zero_offsets.py`
  - confirm software zero is not reintroduced anywhere unexpectedly

## Commands / checks that already produced useful evidence

### RTCore metrics

Use:

```bash
python3 - <<'PY'
import json
from pathlib import Path
p = Path('/run/gradient-rt-motion/metrics.json')
print(json.dumps(json.loads(p.read_text()), indent=2))
PY
```

### Live UI-facing joint truth

Use:

```bash
curl -sS http://127.0.0.1:4000/info/joints-detailed
```

This is important because it shows exactly what the UI is being told after native home.

### RTCore journal

Use:

```bash
journalctl -u gradient-rt-motion.service -n 120 --since '15 minutes ago' --no-pager
```

Also search for native-home logs:

```bash
journalctl -u gradient-rt-motion.service --since '30 minutes ago' --no-pager | rg "native_home|restore|FAULT|0x60B0|0x1010"
```

### Targeted direct fault reset

Use only if needed and only while keeping the robot disarmed:

```bash
PYTHONPATH=/home/pi/GradientOS/src \
/home/pi/GradientOS/.venv/bin/python \
/home/pi/GradientOS/scripts/rtcore_jog.py fault_reset --mask 0x2
```

## Suggested next debugging steps

1. Reproduce one more `Drive Home` click while capturing:
   - RTCore journal filtered on `native_home`
   - `/run/gradient-rt-motion/metrics.json` before and after
   - `GET /info/joints-detailed` before and after

2. Prove whether the rebuilt RTCore really logs the restore path on the live `Drive Home` click:
   - look for `Native-home restore: axis_mask ...`
   - if missing, determine whether:
     - native home did not reach the success branch
     - or the branch ran but another path faulted afterward

3. Inspect whether native home should also explicitly resynchronize these RTCore variables after success:
   - `hold_target_counts[axis]`
   - `have_hold[axis]`
   - any current `target_counts[axis]`
   - active jog / trajectory state for that axis
   - any command queue / motion intent state

4. Verify whether the controller/UI should force-refresh from true live feedback after home and refuse to present a "home complete" pose until:
   - no axis fault
   - joint state source is live
   - DS402 state is sane

5. Check whether the correct operator behavior should be:
   - axis temporarily disabled during home
   - then explicit re-sync to new feedback position
   - only then restore enable request

This may be safer than simply restoring the prior `axis_enable_mask` immediately after the SDO write.

## Important repo state

There are many modified files in the working tree. Do not assume a clean repo.

At handoff, `git status --short` included:

- `.cursor/memory/AGENT_SCRATCHPAD.md`
- `.cursor/memory/DEVLOG.md`
- `.cursor/skills/gradientos-sop/RTOS_ETHERCAT_MASTER_OPERATING_PRINCIPLES.md`
- `.cursor/skills/gradientos-sop/rtcore-ethercat.md`
- `.gradient_joint_zero_offsets.json`
- `docs/ethercat/bringup.md`
- `src/gradient_os/api/main.py`
- `src/gradient_os/arm_controller/actuator_interface.py`
- `src/gradient_os/arm_controller/backends/ethercat_rtcore/backend.py`
- `src/gradient_os/arm_controller/command_api.py`
- `src/gradient_os/arm_controller/ethercat_drive_catalog.py`
- `src/gradient_os/arm_controller/profiles/drive/a6ec_ds402.py`
- `src/gradient_os/arm_controller/profiles/registry.py`
- `src/gradient_os/joint_zero_offsets.py`
- `src/gradient_os/run_controller.py`
- `src/gradient_os/telemetry/drive_faults.py`
- `src/gradient_rt_motion/ipc_v1.hpp`
- `src/gradient_rt_motion/main.cpp`
- `src/numeric_solver/quik` (user/unrelated existing dirty state)
- `tests/test_api_endpoints.py`
- `tests/test_command_api_direct_setpoint.py`
- `tests/test_drive_faults.py`
- `tests/test_gradient05_limits_and_backends.py`
- `tests/test_rtcore_runtime.py`
- `web-ui/src/App.tsx`
- `web-ui/src/ControlPanel.test.tsx`
- `web-ui/src/ControlPanel.tsx`
- `web-ui/src/liveState.tsx`

Do not revert unrelated user changes.

## Prior transcript reference

There is a long prior conversation transcript that contains the earlier architectural and debugging context. If needed, refer to:

- [EtherCAT native-home troubleshooting](6f5cc426-e0b0-4b9e-97ad-660dd69e71f8)

## Bottom line

Do not take over with the assumption "native-home fix is done and just needs cleanup."

The real current state is:

- RTCore source was patched
- RTCore was rebuilt and redeployed
- the direct drive reset path works
- but the actual user-facing `Drive Home` workflow is still broken on live hardware

The next agent should focus on post-home target/reference synchronization and live pose truth, not on redoing the already-known genericization or PDO-layout work.
