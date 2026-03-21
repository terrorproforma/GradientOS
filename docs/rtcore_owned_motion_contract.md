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
