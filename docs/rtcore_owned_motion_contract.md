# RTCore-Owned Motion Contract

This note freezes the intended ownership boundary for EtherCAT motion so the repo can migrate from Python-timed setpoint streaming to RTCore-timed execution without moving IK and planner policy into C++ first.

## Boundary

Python owns:

- Cartesian planning and waypoint generation
- IK / batched IK
- singularity-aware shaping
- home/rest and higher-level policy
- job/program sequencing

RTCore owns:

- deterministic replay timing at the RT cycle
- buffered trajectory storage and consumption
- jog integration and timeout handling
- final motor-rate safety ceilings
- execution progress / completion / underrun / fault status

## Motion Modes

The shared IPC contract now distinguishes four motion modes:

- `idle`: no active RT-owned motion
- `legacy_setpoint`: deprecated compatibility artifact in the ABI/state enum space
- `trajectory`: buffered scheduled joint motion
- `jog`: continuous velocity-style motion with timeout

`legacy_setpoint` should be documented only as a compatibility/deprecation state. Scheduled RTCore motion in GradientOS should use the buffered trajectory contract, not fall back to latest-wins setpoint writes.

## Execution States

RTCore is the source of truth for execution lifecycle:

- `idle`
- `accepted`
- `queued`
- `executing`
- `completed`
- `aborted`
- `faulted`
- `underrun`

Higher layers should stop inferring completion from Python thread joins or joint polling alone.

## Power-Transition Safety Contract

For EtherCAT RTCore-backed drive power transitions, higher layers should treat
the following as the safe/neutral contract:

- `neutral` means:
  - no controller program thread running
  - no active RTCore trajectory
  - no queued RTCore motion
  - no active jog command
  - `motion_done=true`
  - `stale_command=false`
  - no live drive faults
  - feedback has been synchronized to the current hold target
- `safe_for_power_transition` is the derived boolean exported by the controller/API
  and should be the single gating signal for UI/operator power-up flows.
- `power_transition_blockers` / `power_transition_blocker_details` are the
  structured refusal reasons when the system is not neutral.

Power actions must follow this contract:

- `power-up`
  - rejected unless `safe_for_power_transition=true`
  - synchronizes command targets from live feedback before `arm -> CSP -> enable`
- `power-down`
  - stops jog
  - aborts RTCore trajectory ownership
  - waits for neutral execution state when requested
  - disables axes
  - disarms
- `fault reset`
  - first forces stop/abort/disarm behavior
  - then pulses DS402 fault reset
  - must leave the system `BUS_UP_DISARMED`, never implicitly re-enabled

Important compatibility rule:

- On RTCore-backed EtherCAT, the generic controller `STOP` path must **not**
  send the legacy "hold current position" write through `servo_driver`, because
  that write becomes a one-point RTCore trajectory and can falsely relatch
  motion status during power-down.

## Shared-Memory / Ring Contract

Current compatibility path:

- `SetpointSlotV1` stays in shared memory only as a deprecated compatibility artifact in the ABI layout.
- GradientOS scheduled RTCore motion must not depend on it as an execution fallback.

Buffered path:

- trajectory upload uses explicit command-ring messages:
  - `MSG_CMD_TRAJECTORY_BEGIN`
  - `MSG_CMD_TRAJECTORY_POINT`
  - `MSG_CMD_TRAJECTORY_COMMIT`
  - `MSG_CMD_TRAJECTORY_ABORT`
- points are represented by `TrajectoryPointV1`
- realtime jog uses `MSG_CMD_JOG`
- current first slice keeps Cartesian jog shaping in Python, but the final timed joint-velocity command and stale-command timeout now live in RTCore

Status path:

- `MSG_STATUS_MOTION_STATE` publishes RTCore execution state
- `MSG_STATUS_SNAPSHOT` remains the coarse servo/bus snapshot

Controller/API reply path:

- higher layers should use structured acknowledgement (ACK) payloads for controller acceptance metadata
- RTCore physical progress/completion truth should still come from motion-status snapshots/events, not from a one-shot ACK alone
- multi-step program playback should expose a separate controller-program contract alongside RTCore segment status, including:
  - `program_state`
  - `program_terminal_reason`
  - `program_failing_step_index`
  - `program_completed_step_count`
  - `program_completed_loop_count`
- that program contract should persist terminal truth after the controller worker thread exits so UI/API code does not have to infer final outcome from a missing thread or the last RTCore segment alone

## Timing Contract

- RTCore owns the realtime loop cadence.
- Python planners may request a nominal trajectory frequency, but queued upload timestamps should be quantized onto integer multiples of the RTCore cycle.
- The effective queued sample period therefore:
  - fits cleanly inside the RTCore schedule
  - never runs faster than the RTCore loop
  - never runs faster than the requested planner frequency
- If a requested frequency does not divide the RTCore rate neatly, it should be scaled down to the nearest RTCore-aligned rate rather than approximated with off-cycle timestamps.

## First Migration Slice

The intended implementation order is:

1. Keep Python planning intact.
2. Publish RTCore-owned motion status now, even while deprecated compatibility fields still exist in the ABI.
3. Add buffered trajectory upload and commit semantics behind feature flags.
4. Move EtherCAT scheduled moves from Python sample loops to RTCore replay.
5. Keep jog as a separate RT control mode instead of emulating it with streamed position setpoints.
   Current first slice: Python still converts Cartesian jog intent into joint-velocity updates, but RTCore owns the jog command timing, timeout, and per-cycle target integration.

## Compatibility Rule

No higher layer should assume that controller acceptance means physical completion.

For scheduled EtherCAT RTCore motion, higher-level `closed_loop=true` request flags should not reactivate a Python-timed sample loop. Those requests should be coerced onto the RTCore queued path or rejected explicitly.

Until every higher-level caller is migrated, the repo should treat compatibility fields and wait helpers as transitional scaffolding and prefer RTCore status snapshots over ad hoc polling wherever possible.

## Live Validation Reference

The safe no-motion power-cycle validation for this contract is:

1. `./start-stack.sh stop --hard`
2. `./start-stack.sh --headless`
3. Verify `./start-stack.sh probe` reports `physical_state=BUS_UP_DISARMED`
4. Verify `GET /control/motion-status` reports:
   - `state=idle`
   - `active_traj_id=0`
   - `queue_depth=0`
   - `safe_for_power_transition=true`
5. `POST /control/power-up`
6. Verify `./start-stack.sh probe` reports:
   - `physical_state=ACTIVE`
   - `op_enabled_axes=6/6`
   - all axes `OperationEnabled`
   - all drive errors `0x0000`
7. Send **no motion commands**
8. `POST /control/power-down` with `{"wait_for_idle": true}`
9. Verify:
   - `./start-stack.sh probe` returns to `physical_state=BUS_UP_DISARMED`
   - `GET /control/motion-status` returns to `idle` with `active_traj_id=0`
   - `safe_for_power_transition=true`
10. `./start-stack.sh stop --hard`

This validation was re-run after fixing the RTCore-backed `STOP` path so it no
longer injected a one-point hold trajectory during power-down.
