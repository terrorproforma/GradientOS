# J6 Move B Seam-Crossing Fault Handoff (Updated 2026-04-19)

## Scope

You are taking over a live A6-EC / J6 fault-classification investigation. Phase 1
of [`/home/pi/.cursor/plans/rtcore_proof_and_math_5813457e.plan.md`](/home/pi/.cursor/plans/rtcore_proof_and_math_5813457e.plan.md)
has now classified the failure to an exact vendor subcode. Your job is to pick
the correct host-side remediation, make J6 Move B pass clean, then run Move C,
WITHOUT regressing the architectural constraints the operator has reconfirmed.

## The Single Paragraph You Must Internalize First

With the correct `home-joint-native -> /control/power-up -> APPLY_JOINT_SETPOINT`
sequence, J6 Move A (mid-range `+10 deg` jog at `max_motor_rpm = 100`) now
completes cleanly under the `GRADIENT_RTCORE_EXPERIMENTAL_CONTINUOUS_607A_JOINTS=6`
experiment flag. The UDP transport, controller trajectory executor, shared-memory
ring, RTCore scheduler, and per-cycle `607A` PDO writes are all proven healthy
on this hardware. J6 Move B (pre-position near `+/-175 deg`, then seam-crossing
jog to `+/-185 deg` at `max_motor_rpm = 10`) faults from BOTH directions with
the same signature: RTCore branch `FAULT_EXEC_E2`, statusword `0x9638`, bus
fault `0x603F = 0xFF00`, manufacturer fault `0x203F = 0x0871` which the
A6-EC codebook maps to **`Er87.1` — "One-time excessive position reference
increment"**. This is direction-independent and confirmed by direct SDO reads,
not inferred. You are not debugging a pre-wire failure, a seam-direction
issue, or a generic `0xFF00` ambiguity — you are debugging why the host emits
a single `607A` cycle-delta the drive considers excessive on the sample that
straddles the `0 / RM` wrap.

## Architectural Invariants (Do Not Break)

The operator restated these explicitly and they supersede any older scratchpad
wording that leaned toward `60B0` promotion:

1. `60B0` is runtime-only. Do NOT use it as home truth, persistence, or
   canonical multi-turn state. Only consider it later as a conditional runtime
   fallback IF Phase 1 evidence proves it necessary.
2. The A6-EC control architecture is:
   - Read truth: `U40.20/U40.22` (continuous multi-turn).
   - Convert to output-shaft semantics on host using `C10.18/C10.19`.
   - Status/homing trust: `6041` bits 12 and 15.
   - CSP write target: `607A`.
   - Persistent home/reference offset: `607C` (midpoint-biased at `RM/2`).
   - `6064` is single-turn only. Use it ONLY as the drive's single-turn
     wire/reference frame (host consistency gate, diagnostic). Never as planner
     truth.
3. HM35 writes `607C = RM/2` via `write_sdo_wrap_fraction(1, 2)`. Do NOT revert
   to `607C = 0`. Do NOT reintroduce routine `F31.10` into HM35.
4. [`a6ec_joint_motion.py`](/home/pi/GradientOS/src/gradient_os/arm_controller/math/a6ec_joint_motion.py)
   is not production truth yet. Phase 2 (math adoption) is gated on Phase 1
   finishing cleanly.

## What Is Already Proven (Do Not Re-Test)

- Move A (mid-range jog, no seam): PASS. Direct UDP `APPLY_JOINT_SETPOINT` to
  J6 `+10 deg` at `max_motor_rpm = 100` completed cleanly. `/control/wait-for-idle`
  returned `state='completed'`. `/info/joints` showed J6 `10.000854 deg`.
  No fault. Watch:
  `logs/encoder-retention/j6-proof-matrix-20260418/move-a-midrange-jog-100rpm-safe-power-up-udp.watch.jsonl`.
- Move B pre-position to `+175 deg`: PASS. Watch:
  `logs/encoder-retention/j6-proof-matrix-20260418/move-b-preposition-truth-check.watch.jsonl`.
  Placed J6 about 5 degrees off the wire seam (`6064 ~= 18204`, `607A = 18204`,
  `603F = 0x0000`).
- Phase 1 RTCore fault instrumentation works. The `FAULT_EXEC_E2` tag fired on
  Move B with all the required context fields.

## The Confirmed Failure Signature

Move B seam-crossing faults from BOTH directions with an identical, unambiguous
signature. Do not re-derive it; it is captured below.

**Positive-side repro (2026-04-18 23:50 devlog entry):**
- Watch: `logs/encoder-retention/j6-proof-matrix-20260418/move-b-seam-crossing-10rpm-safe-power-up-udp.watch.jsonl`
- Wire behavior: `607A` ran `18204 -> 14544 -> 527 -> 1169`; `6064` ran
  `18204 -> 15828 -> 2904 -> 1169`; `603F` flipped `0x0000 -> 0xFF00`; statusword
  became `0x9638`.

**Negative-side repro (2026-04-19 00:38 devlog entry):**
- Watch: `logs/encoder-retention/j6-proof-matrix-20260419/move-b-negative-seam-crossing-10rpm-safe-power-up-udp.watch.jsonl`
- Wire behavior: `607A` ran `1292516 -> 1292535 -> 1299916 -> 1309972`; `6064`
  ran `1292516 -> 1292514 -> 1298035 -> 1309972`; `603F` flipped to `0xFF00`
  before the drive successfully wrapped.
- Direct SDO reads while faulted on axis 5 (J6):
  - `sudo ethercat upload -p 5 -t uint16 0x603F 0x00` -> `0xff00`
  - `sudo ethercat upload -p 5 -t uint16 0x6041 0x00` -> `0x9638`
  - `sudo ethercat upload -p 5 -t uint16 0x203F 0x00` -> `0x0871`

**Vendor code decoded from `0x0871`:** `Er87.1 — One-time excessive position
reference increment` (bus class `0X7305` in the `a6ec_manual_codes.json`
fault table; "One-time increment of the target position is over 5 times of
the maximum speed").

**Shared with both runs:**
- RTCore branch tag `FAULT_EXEC_E2` fires in `/var/log/syslog`:
  `FAULT_EXEC_E2 diag_now_ns=<ns> traj_id=<n> final_due=0 axis_index=5 error_code=0xFF00 ds402_state=5`
- `/control/wait-for-idle` -> `state='timeout'` with RTCore
  `state_name='faulted'`, `last_event_code = 293`, `current_point_index = 83`,
  `queue_depth = 83`.
- Probe after fault: J6 `sw = 0x9638`, `err = 0xff00`. Note that the older
  probe output labeled this as "Er11.0" because it decoded only `0x603F`
  and `0xFF00` is shared by seven vendor codes (see table below); the
  2026-04-19 offline prep patch to `start-stack.sh` now also prints the
  `0x203F` decode next to `err=...`, so the next run will show `Er87.1`
  directly.

Reference table (kept for completeness; the actual code on J6 is `Er87.1`):

| `0x203F` | Vendor code | Meaning |
| --- | --- | --- |
| `0x1100` | `Er11.0` | Excessive motor speed upon servo drive power-on |
| `0x4500` | `Er45.0` | S-ON enabling failure |
| `0x8403` | `Er84.3` | Home position setting error |
| `0x8701` | `Er87.1` | **One-time excessive position reference increment (THIS RUN)** |
| `0x8702` | `Er87.2` | Continuous excessive position reference increment |
| `0x8703` | `Er87.3` | Overflow of 32-bit sign bit of target position |
| `0x8704` | `Er87.4` | Target position exceeds max mechanical single-turn position (rotating mode) |

(See [`docs/resources/a6ec_manual_codes.md`](/home/pi/GradientOS/docs/resources/a6ec_manual_codes.md).)

## Offline Prep Already Landed (2026-04-19)

Before starting hardware work, read these three landed changes. They change
what the next probe run will show and narrow the code surface you need to
edit:

1. **Probe script width fix**: `scripts/a6ec_chapter5_probe.py` now reads
   `0x203F` as `uint16`. Prior runs showed `203F = None` because the SDO
   descriptor used `uint32` and the drive returns 2 bytes. The watch JSONL
   from the next run will have the real `0x203F` value per sample.
2. **`./start-stack.sh probe` per-axis summary**: the probe CLI line now
   appends `[mfr <Er-code> | <name>]` after the bus-level `[Er-code | name]`
   block when `manufacturer_fault` is decoded. Operators no longer have to
   parse `/run/gradient-rt-motion/metrics.json` to see the disambiguating
   vendor code.
3. **Retention-family set extended**: `ErA0.1 (Multi-turn overflow fault)`
   is now a member of `ENCODER_RETENTION_FAULT_CODES` in
   `src/gradient_os/arm_controller/profiles/drive/a6ec_ds402.py`. Vendor
   email 4 Q2(a) explicitly lists it as a primary retention signal
   (alongside `Er20.8`, `Er20.9`, `ALF9.0`) but it was absent from the set.
   Regression coverage in
   `tests/test_gradient05_limits_and_backends.py::test_a6ec_encoder_retention_fault_includes_multi_turn_overflow`.
   This is not directly related to the Move B fault but closes a gap
   surfaced during the same EMAIL 4 review.

## Live Sequence Rules You Must Follow (In Order)

1. Fresh start-from-dead:
   ```
   ./start-stack.sh stop --hard
   GRADIENT_RTCORE_EXPERIMENTAL_CONTINUOUS_607A_JOINTS=6 ./start-stack.sh
   ```
2. Home J6:
   ```
   curl -sS -X POST -H 'Content-Type: application/json' \
     --data '{"logical_joint_index":5}' \
     http://127.0.0.1:4400/control/home-joint-native
   ```
   Expect `disarmed_after_home=true` and the explicit message
   "The homed axis remains disabled until an explicit safe power-up."
3. **ALWAYS** call `/control/power-up` BEFORE any motion command:
   ```
   curl -sS -X POST http://127.0.0.1:4400/control/power-up
   ```
   Confirm with `./start-stack.sh probe`: `armed = 1`, `enable_mask = 0x3f`,
   `op_enabled_axes = 6/6`, J6 `sw = 0x9637`.
   This is the rule that reclassified the earlier "no-motion" Move A failures
   as simple test-sequence omissions. Do not skip it.
4. For the proof matrix, use the **raw UDP** `APPLY_JOINT_SETPOINT` path, NOT
   `/control/joint-jog`. `/control/joint-jog` adds a commissioning gate that
   currently rejects seam-adjacent moves with
   `CANONICAL_JOINT_TRUTH_UNAVAILABLE / absolute_home_anchor_stale`, which
   masks RTCore/drive behavior. The exact UDP invocation used successfully:
   ```
   python3 -c "import base64, json, math, socket, urllib.request; \
     arm = json.load(urllib.request.urlopen('http://127.0.0.1:4400/info/joints'))['arm_rad']; \
     arm[5] = math.radians(<TARGET_DEG>); \
     payload = {'arm_angles_rad': arm, 'max_motor_rpm': <MOTOR_RPM>, 'target_joint_indices': [5]}; \
     msg = 'APPLY_JOINT_SETPOINT,' + base64.urlsafe_b64encode(json.dumps(payload, separators=(',',':')).encode('utf-8')).decode('ascii'); \
     sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM); \
     sock.settimeout(2.0); \
     sock.sendto(msg.encode('utf-8'), ('127.0.0.1', 3000)); \
     print(sock.recvfrom(65535)[0].decode('utf-8'))"
   ```
5. Start the probe **before** the move, never after:
   ```
   source ./start.sh && PYTHONPATH=src python scripts/a6ec_chapter5_probe.py watch \
     --label <run-label> --axes J6 --interval-s 0.02 --duration-s 15 \
     --fast-proof --experiment-id j6-proof-matrix-<YYYYMMDD>
   ```
6. After a fault, tear down hard before any further test:
   ```
   ./start-stack.sh stop --hard
   ```
   Do NOT leave J6 faulted and the stack armed.

## Code-Path Analysis (2026-04-19 offline pass)

A careful read of the RTCore motion path in
[`src/gradient_rt_motion/main.cpp`](/home/pi/GradientOS/src/gradient_rt_motion/main.cpp)
narrows the fix to two real options. The original "wrap into `[0, RM)` vs
clamp seam step" framing was slightly wrong; the details matter.

### What RTCore currently does on the wire

1. **Trajectory segment interpolation** (lines ~3399-3557): on wrapped axes
   (`feedback_counts_wrap=True`, which A6-EC sets via
   `GRADIENT_RT_FEEDBACK_WRAP_AXIS_MASK=0x3f`), it computes the interpolated
   target using shortest-periodic delta between `p0` and `p1`, then wraps
   the result into `[0, period)` at line 3553-3554 before rounding into
   `target_counts[i]`.
2. **Hold-target advance** (line 3742-3752): calls
   `advance_csp_hold_target_counts(hold_target_counts[i], target_counts[i],
   max_step_counts_per_cycle, wrap_period_counts)`. For wrapped axes this
   uses shortest-periodic math to clamp by `max_step_counts_per_cycle` and
   always returns a value in `[0, period)`.
3. **Wire write** (line 3796): `EC_WRITE_S32(axis_pd + off[i].target_pos,
   target_pos_out)` where `target_pos_out = hold_target_counts[i]`.

So the final `607A` stream on the wire is always wrapped into `[0, RM)`
for A6-EC axes, regardless of anything the host does.

### What the experiment flag actually does

`GRADIENT_RTCORE_EXPERIMENTAL_CONTINUOUS_607A_JOINTS=6` affects only the
Python backend:
- `_nearest_turn_fold_axis_q_for_axis(..., wrap_to_single_turn=False)` is
  used for J6 trajectory planning, so the host-emitted trajectory points
  can span multiple RM windows in continuous counts.
- Host-side seam-crossing safety guard (`command_frame_seam_crossing_unsafe`)
  is bypassed for J6.

**But** RTCore re-wraps the host's continuous values at lines 3553 and 3751.
**So the experiment flag is effectively a no-op on the wire output.** Whether
it is ON or OFF, the drive sees wrapped `607A`. The only observable
difference is whether the host refuses the trajectory upfront.

### Why this produces `Er87.1`

With RTCore wrapping into `[0, RM)` every cycle, the cycle that straddles
the seam emits consecutive `607A` values like `3 → 1310710`. The drive's
`Er87.1` check evaluates the absolute wire-frame delta `|607A[n+1] -
607A[n]|` (not shortest-periodic) and fires because
`1310707 counts` grossly exceeds `5 x max_motor_speed_counts_per_cycle`.

`advance_csp_hold_target_counts` DOES clamp by `max_step_counts_per_cycle`
(currently `~13,107` counts/cycle at `max_rpm=6000`, 1 kHz), but only in
shortest-periodic space. A `3 → 1310710` wire jump is a shortest-periodic
delta of `-13`, well under the `13,107` clamp, so the clamp does NOT fire
and the seam wrap goes through untouched.

### Watch evidence confirming the path

Full per-sample extraction of both Move B watches (see
`logs/encoder-retention/j6-proof-matrix-*/move-b-*seam-crossing-10rpm*.watch.jsonl`):

- **Positive Move B** (trajectory goes `+175° → +185°`, wire `18204` down
  past the seam at `0/RM` to `-18101` = `RM-18101 = 1292619`):
  - sample 13 (~0.58 s after start): `607A=14544`, `6064=15828`,
    `cpi=3/166`, `executing`
  - sample 14 (~1.05 s): `607A=527`, `6064=2904`, `cpi=54/166`, `executing`
  - sample 15 (~1.55 s): `607A=1169`, `6064=1169`, `603F=0xFF00`,
    `sw=0x9638`, `cpi=83/166`, **faulted** — trajectory stopped mid-seam
- **Negative Move B** (trajectory goes `-175° → -185°`, wire `1292516` up
  past the seam to `18098`):
  - sample 23: `607A=1292535`, `6064=1292516` (at pre-position)
  - sample 24: `607A=1299916`, `6064=1298035`, `cpi=14/166`, `executing`
  - sample 25: `607A=1309972`, `6064=1309972`, `cpi=64/166`, `executing`
    — at this point we are `RM - 748` counts from the seam
  - sample 26: `607A=1309923`, `603F=0xFF00`, `sw=0x9638`, `cpi=83/166`,
    **faulted** — drive faulted immediately before crossing `0/RM`

Both runs fault at `cpi=83/166`, which is right where the trajectory point
closest to the seam gets wrapped. The 500 ms probe cadence cannot capture
the exact cycle — the actual `607A[n] → 607A[n+1]` wrap happens between
probe samples — but the bracketing evidence (pre-fault `6064` marches
toward the seam, fault fires immediately after) matches the predicted
seam-wrap mechanism exactly.

## The Two Real Options

### Option A — Retire seam crossing for rotation-mode axes (SAFE, ONE-LINE)

Treat the A6-EC wire-frame seam as a hard boundary operator must plan
around.

**Change:**
- [`a6ec_ds402.py`](/home/pi/GradientOS/src/gradient_os/arm_controller/profiles/drive/a6ec_ds402.py):
  flip `POSITION_SEMANTICS_CONFIG["command_frame_seam_crossing_unsafe"]` to
  `True`. This re-enables the host-side safety guard
  (`_enforce_trajectory_wire_frame_safety`) that rejects any consecutive
  trajectory points whose wire-space delta crosses the seam.
- Retire `GRADIENT_RTCORE_EXPERIMENTAL_CONTINUOUS_607A_JOINTS` (remove the
  env var or change its default to empty). The experiment path was supposed
  to send continuous `607A` but RTCore's wrap neutralizes it; it only
  serves to bypass the host guard, which is no longer desirable.

**What operators see:**
- Any commanded trajectory that would cross `+180°` or `-180°` canonical on
  J6 (with `607C=RM/2`) is rejected at the host with
  `command_frame_seam_crossing_step_disallowed` or
  `command_frame_seam_crossing_first_point_disallowed`.
- Multi-turn rotation for J6 in rotation mode is impossible; the joint is
  operationally limited to moves that stay within a single mechanical
  revolution bounded by the seam.

**Pros:**
- One-line change in the drive profile.
- Fully reversible.
- No firmware behavior dependencies.
- Host guard + regression coverage already exist.

**Cons:**
- Loses multi-turn operation for J6. If the planner ever needs `>360°`
  rotation on J6 this option blocks it.

### Option B — Emit continuous (unwrapped) `607A` at the wire (RISKY, LIVE TEST)

Actually bypass the RTCore wrap so `607A` is a monotonic linear stream that
can cross the seam without a discontinuity.

**Change:**
- Per-axis drop the wrap mask bit for J6 by editing the runtime config so
  `feedback_wrap_axis_mask` becomes `0x1f` instead of `0x3f`. This uses the
  existing `GRADIENT_RT_FEEDBACK_WRAP_AXIS_MASK` plumbing — no RTCore code
  change required.
- With wrap disabled for J6:
  - trajectory interpolation at line 3553 no longer wraps the output
  - `advance_csp_hold_target_counts(... , wrap_period_counts=0)` falls into
    the linear `desired - cur` branch with the `max_step_counts_per_cycle`
    clamp still applied
  - `607A` on the wire can grow without bound and cross the seam smoothly
- Requires a live Move B rerun to verify the drive accepts continuous
  `607A` in rotation mode. The vendor's rotation-mode doc (email 3 Q5)
  is explicit that `6064` reports `[0, RM-1]`; it does NOT explicitly
  state what `607A` must look like. `C10.16 = 0` (Nearest/shortest
  path) suggests the drive picks shortest-path from current `6064` to the
  commanded `607A` regardless of wraps; if that interpretation is
  consistent across a continuous-`607A` stream, this works.

**What operators see (if it works):**
- Seam crossing succeeds; `U40.20/.22` increments continuously; `6064`
  sawtooths through the wrap.
- Multi-turn operation for J6 is possible.

**What operators see (if it fails):**
- Drive still reports some fault on the seam cycle (but unlikely `Er87.1`
  since the wire delta is now bounded by `max_step_counts_per_cycle`).
- Some other fault family might fire (e.g. `Er87.4` "target exceeds max
  mechanical single-turn position" if the drive rejects targets outside
  `[0, RM-1]` in rotation mode).

**Pros:**
- Architecturally correct (continuous `607A` + drive absorbs wrap
  internally).
- Enables multi-turn rotation.
- Still bounded by the existing `max_step_counts_per_cycle` clamp.

**Cons:**
- Drive firmware behavior with out-of-range `607A` in rotation mode is NOT
  vendor-confirmed.
- Requires live hardware testing on J6 with the operator present.
- Completion check semantics on non-wrapped axes use linear delta; need to
  verify the completion check still lands at the right point.

## Recommended Order

1. **Immediate: ship Option A** as the stable fix. This fully resolves the
   `Er87.1` fault and locks the system to vendor-documented behavior.
2. **Deferred: try Option B** as a contained experiment when the operator
   wants multi-turn J6 back. Requires a live hardware session and a
   targeted `GRADIENT_RT_FEEDBACK_WRAP_AXIS_MASK` override. If Option B
   works, Option A can be relaxed; if it does not, Option A stays.

Do NOT attempt Option B without the operator watching; the seam-crossing
cycle is still a drive-fault candidate and may produce a different error
family.

## Evidence Capture Requirements (when rerunning either option)

Before reporting "fixed":

- the Move B positive watch JSONL path
- the Move B negative watch JSONL path
- the final `6064` value for both runs (should be `≈ 18098` and `≈ 1292619`
  respectively, matching the commanded `±185°` canonical target after
  midpoint-biased home)
- any `0x203F` reads (should be absent / zero)
- the `/var/log/syslog` window around the move (should not contain any
  `FAULT_*` tag)
- `/control/motion-status` returning `state='completed'` for the trajectory
- `/info/joints` reporting J6 at `±185°` canonical

## Decision Gate

- Do NOT run Move C (`+360 deg` multi-turn) until Move B passes clean under
  the chosen option. Move C by definition crosses the seam twice.
- Do NOT branch into Phase 2 (math module adoption) until Move B passes.
- If Option A ships first and Option B is later attempted and fails,
  restore Option A with no regressions.

## Known API / Controller Nuances To Watch

- `/control/joint-jog` currently rejects at the commissioning gate with
  `CANONICAL_JOINT_TRUTH_UNAVAILABLE / absolute_home_anchor_stale` after a
  seam-adjacent pre-position. This is a truth-refresh wrinkle in the higher-level
  API path and does NOT block the raw UDP `APPLY_JOINT_SETPOINT` proof. Log it
  and defer the fix until Move B passes.
- `--fast-proof` probe cadence is still ~0.4-1.5 s/sample on this host even with
  `--interval-s 0.02`, because SDO/controller polling dominates. Sub-second
  transient detection in the watch file is currently weak. If you need denser
  capture around the seam fault, consider temporarily dropping the probe's
  low-priority SDOs (`U40.1C` / `U40.1E`) in fast mode or using `strace`-level
  timing on the RTCore side, but do NOT invest in probe perf work unless Step 2
  says the fault timing window matters.

## Validation Before Declaring Anything Fixed

- On any candidate fix, `./start-stack.sh stop --hard && ./start-stack.sh` +
  re-home J6 + `/control/power-up` + the full proof matrix (Move A, then
  Move B pre-position, then Move B seam-crossing, then Move C) under a fresh
  probe for each.
- Expected PASS signature for Move B under a correct fix:
  - `603F = 0x0000` throughout
  - `203F` stays empty
  - `statusword` stays `OperationEnabled` (`0x9637` family)
  - `U40.20/U40.22` moves by exactly one seam crossing worth of counts
  - RTCore `state='completed'`
  - J6 `/info/joints` reports about `+185 deg`
- Expected PASS signature for Move C (`+360 deg`):
  - `U40.20/U40.22` increments by exactly one full shaft revolution
  - `6064` sawtooths through two rollovers
  - no fault, no `FAULT_*` tag
- Do NOT skip any of: `./start-stack.sh probe`,
  `curl -sS http://127.0.0.1:4400/info/joints`,
  `rg 'FAULT_' /var/log/syslog -B 3 -A 3 --head-limit 10`,
  `/control/motion-status`.

## Required Scratchpad / Devlog Updates

At the end of your run, append:

- `.cursor/memory/AGENT_SCRATCHPAD.md`:
  one dated entry under the existing `### 2026-04-` section style, tagged
  with `[tool]` + `[self]`, covering:
  - the specific vendor code decoded from `0x203F`
  - which hypothesis branch matched
  - whether Move B now passes clean under the fix
  - whether Move C ran and its outcome
- `.cursor/memory/DEVLOG.md`:
  one dated heading `## 2026-04-<DD> <HH:MM> +0000 - <brief>` with the full
  evidence block, matching the 2026-04-18 23:50 entry style.

## Do NOT

- Do NOT promote `60B0` as a fallback. The expanded manual-rotate dataset
  already confirmed it stays `0` through a full multi-turn sweep.
- Do NOT reintroduce routine `F31.10` writes into HM35.
- Do NOT skip `/control/power-up` after `home-joint-native`. That is the rule
  that reclassified the 2026-04-18 "no-motion" Move A runs as sequence bugs.
- Do NOT switch the proof path to `/control/joint-jog` until its commissioning
  gate is separately fixed.
- Do NOT run Move C before Move B passes.
- Do NOT start Phase 2 (math module adoption) before Move B passes AND Move C
  passes.
- Do NOT change `607C = RM/2`.

## Key Files To Read Before Starting

- Plan: [`/home/pi/.cursor/plans/rtcore_proof_and_math_5813457e.plan.md`](/home/pi/.cursor/plans/rtcore_proof_and_math_5813457e.plan.md)
- Scratchpad tail (2026-04-18 section):
  [`.cursor/memory/AGENT_SCRATCHPAD.md`](/home/pi/GradientOS/.cursor/memory/AGENT_SCRATCHPAD.md)
- Devlog tail (2026-04-18 23:50 entry):
  [`.cursor/memory/DEVLOG.md`](/home/pi/GradientOS/.cursor/memory/DEVLOG.md)
- Vendor notes (email 4 validates host-side model; email 2 defines `607C` range;
  email 3 defines CSP architecture):
  [`docs/ethercat/a6ec-manufacturer-notes-2026-04-15.md`](/home/pi/GradientOS/docs/ethercat/a6ec-manufacturer-notes-2026-04-15.md)
- Fault code reference:
  [`docs/resources/a6ec_manual_codes.md`](/home/pi/GradientOS/docs/resources/a6ec_manual_codes.md)
  and [`docs/resources/a6ec_manual_codes.json`](/home/pi/GradientOS/docs/resources/a6ec_manual_codes.json)
- RTCore `FAULT_*` instrumentation source (understand what each tag means):
  [`src/gradient_rt_motion/main.cpp`](/home/pi/GradientOS/src/gradient_rt_motion/main.cpp)
- Experiment gate source:
  [`src/gradient_os/arm_controller/backends/ethercat_rtcore/backend.py`](/home/pi/GradientOS/src/gradient_os/arm_controller/backends/ethercat_rtcore/backend.py)
  (`GRADIENT_RTCORE_EXPERIMENTAL_CONTINUOUS_607A_JOINTS`,
  `_nearest_turn_fold_axis_q_for_axis`, `_enforce_trajectory_wire_frame_safety`)
- Drive profile:
  [`src/gradient_os/arm_controller/profiles/drive/a6ec_ds402.py`](/home/pi/GradientOS/src/gradient_os/arm_controller/profiles/drive/a6ec_ds402.py)
  (`POSITION_SEMANTICS_CONFIG["command_frame_seam_crossing_unsafe"]`,
  `NATIVE_HOME_CONFIG`)

## TL;DR For Another Agent

1. Move A is clean. Don't retest it except as smoke.
2. Move B seam-crossing faults from BOTH directions with the same signature:
   RTCore `FAULT_EXEC_E2`, statusword `0x9638`, `0x603F = 0xFF00`,
   **`0x203F = 0x0871`** = **`Er87.1`** ("One-time excessive position reference
   increment"). Confirmed by direct `ethercat upload -t uint16 0x203F`
   reads; the probe's older `203F=None` output was a width bug that has
   been fixed.
3. **The root cause is in RTCore, not the host**: RTCore's trajectory
   interpolation (`main.cpp:3553`) and `advance_csp_hold_target_counts`
   (`main.cpp:3751`) always wrap `607A` into `[0, RM)` for wrapped axes,
   regardless of the `GRADIENT_RTCORE_EXPERIMENTAL_CONTINUOUS_607A_JOINTS`
   experiment flag. The clamp uses shortest-periodic math, so a seam wrap
   (e.g. `607A = 3 → 1,310,710`) passes through the clamp as a small
   shortest-periodic delta (`-13`) while the drive's `Er87.1` logic sees
   the absolute wire delta (`1,310,707`) and fires.
4. The experiment flag is effectively a no-op on the wire output — it only
   controls host-side safety guards and trajectory planning math, both of
   which get neutralized by RTCore's unconditional wrap.
5. Two real fix options:
   - **Option A (recommended, safe, one-line):** flip
     `POSITION_SEMANTICS_CONFIG["command_frame_seam_crossing_unsafe"]=True`
     in `a6ec_ds402.py`, retire the experiment flag. Host guard rejects
     seam-crossing trajectories. Operator plans around `±180°`.
     Multi-turn J6 in rotation mode becomes impossible; single-revolution
     operation still works.
   - **Option B (experimental, requires live test):** flip
     `GRADIENT_RT_FEEDBACK_WRAP_AXIS_MASK` from `0x3f` to `0x1f` so RTCore
     stops wrapping `607A` for J6. Requires verifying the drive accepts
     continuous `607A` in rotation mode. Enables multi-turn if the drive
     tolerates it.
6. Do NOT run Move C until Move B passes. Do NOT promote `60B0`. Do NOT skip
   `/control/power-up`. Always tear down faulted state with `stop --hard`.
7. `./start-stack.sh probe` now shows the vendor subcode alongside the
   bus-level code in the per-axis line (`[Er11.0 | ...][mfr Er87.1 | ...]`),
   so future probe lines are self-describing.
