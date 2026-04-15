# A6-EC Frame Semantics and Native-Home Verification

Status: active workstream note. This is durable project documentation, but it is intentionally provisional rather than canonical SOP. It captures the validated lessons so far from the current A6-EC frame-semantics investigation and should later be consolidated once the remaining open questions settle.

## Why This Note Exists

The core lesson from this workstream is that the difficult bug class was not "random bad telemetry." It was repeated frame mixing:

- raw absolute encoder counts
- drive reference and home counts
- stored software anchors

Treating those as if they were one frame created ambiguous debugging, unsafe command-path reasoning, and misleading operator feedback. This note records the current model so future work starts from the right mental split.

## Why The Probe Exists

`scripts/a6ec_chapter5_probe.py` exists because trial-and-error jogging and single-object spot checks were too ambiguous for this bug class.

We needed one read-only harness that captures all of the following in the same snapshot:

- raw encoder objects
- drive reference and home objects
- rotation-mode objects
- API truth view

The intended comparison is across:

- `boot`
- `native-home`
- `power cycle`
- `restart`

The script header states that purpose directly. The point is to compare the drive's raw, bridged, and controller-visible position families without moving the robot unnecessarily.

## Current Frame Model

The A6-EC exposes at least three distinct position families that must be kept separate.

- Raw absolute encoder motor frame:
  - `U40.1C`
  - `U40.1E`
  - `U40.20/.22`
- Rotation-mode or mechanical bridge frame:
  - `U40.28`
  - `U40.2A/.2C`
  - `C10.*`
- CSP or reference or home frame:
  - `6064`
  - `607A`
  - `6063`
  - `60FC`
  - `U40.16`

The most important behavioral correction is simple: these are related frames, not interchangeable fields.

## Explicit Equations

### Raw Multi-Turn Reconstruction

The probe makes the Chapter 5 reconstruction explicit:

- `COUNTS_PER_MOTOR_REV = 131072 = 2^17`
- `single_turn_mod = U40.1C mod 131072`
- `signed_multiturn_revs = sign_extend_16(U40.1E)`
- `reconstructed_abs_counts = signed_multiturn_revs * 131072 + single_turn_mod`

Where:

- `sign_extend_16(x) = x` when `x < 32768`
- `sign_extend_16(x) = x - 65536` when `x >= 32768`

This is the precise math behind the earlier "encoder count plus rotation count times 131072" reasoning.

### Preferred Live Absolute Source

The live A6-EC profile is not limited to the `U40.1E` revolution word. It prefers the full multi-turn pair:

- first choice: `U40.20 / U40.22`
- second choice: `U40.2A / U40.2C`

The 64-bit combine math is:

- `combined = ((high & 0xffffffff) << 32) | (low & 0xffffffff)`
- if `high` has bit 31 set, then `combined -= 2^64`

So the preferred absolute source is:

- `absolute_counts = signed_i64(U40.22:U40.20)`

That matters because the live backend is not fundamentally constrained by the standalone `U40.1E` signed-16-bit revolution interpretation.

### Counts To Joint Angle

The backend converts counts into axis-space radians with the configured scaling:

- `counts_per_radian = (encoder_counts_per_rev * gear_ratio) / (2 * pi)`
- `axis_q_rad = raw_counts / (sign * counts_per_radian)`

Equivalently:

- `axis_q_rad = sign * raw_counts * 2 * pi / (encoder_counts_per_rev * gear_ratio)`

This is why large-ratio joints still need a continuous multi-turn motor source for truth reconstruction, but not necessarily for direct command generation.

### Canonical Truth Math

The current canonical-truth model is:

- `absolute_axis_q = absolute_counts / (sign * counts_per_radian)`
- `reference_q = q_from_6064_frame + native_home_offset`
- `home_anchor_rad = absolute_axis_q - reference_q` at anchor-capture time
- `canonical_q = absolute_axis_q - home_anchor_rad - software_zero`

So the write path should round-trip as:

- `reference_q = canonical_q + software_zero`

No extra anchor term belongs on the command path. The anchor already bridges absolute encoder space to the reference or home space.

## Findings That Look Stable So Far

- The problem was not "just bad telemetry." It was multiple position frames being conflated.
- `U40.20/.22` really are the raw multi-turn encoder counts we wanted to verify.
- The Chapter 5 raw composition formula matches live data within normal read jitter.
- `6064`, `607A`, `U40.16`, and `60FC` are not raw encoder counts. They belong to the drive's reference or home machinery.
- `60FC` turned out to be a key clue that the drive internally maps reference-unit motion into encoder-unit position reference.
- `60E6` matters more than it first appeared. The manual text about actual-position calculation after homing aligns closely with the semantic issue we were chasing.
- `607C` is not a sufficient witness for success or persistence in this workflow.
- `6041 bit15` is not a sufficient witness for persistence across reboot.
- `60B0` behaved like a runtime motion-frame object, not the durable semantic-home store we needed.
- The integrated `F31.10` read/write tail matters for persistence. It was a major breakthrough in making the corrected frame restore across power cycles.

## Motion And Safety Lessons

- The multi-axis jog regression was a command-frame math bug, not just a display bug.
- The bad change was re-applying `absolute_home_anchor_rad` on the write path.
- That was wrong because the anchor cancels algebraically on the command path.
- Command upload must stay in the RTCore and `6064` and `607A` reference frame.
- The command path must not "helpfully" add the absolute anchor back in.
- Raw absolute multi-turn counts are essential for diagnostics and truth reconstruction, but they are not automatically the right frame to command motion from.
- The current live stack is not using drive-side gear-ratio mapping on the tested axes. `6091` and `C10.18/C10.19` were `1:1`, so software still owns the higher-level joint semantics.

## Truth, Anchors, And Fail-Closed Behavior

- Removing fallback behavior was the right call. Fallbacks made it too easy to hide frame problems behind something merely plausible.
- The system should fail closed when canonical truth cannot be reconstructed safely.
- A stale software anchor can make a good drive-side frame look bad.
- The specific `command_frame_roundtrip_mismatch` diagnosis is our host-side diagnosis, not a manufacturer object or drive-side fault.
- The anchor is stored in `.gradient_absolute_encoder_anchors.json`.
- The anchor is calculated from:
  - absolute encoder-derived axis position
  - minus the live RTCore or reference-frame position
- If that stored anchor is stale, canonical reconstruction can be off by tens of thousands of counts even when the drive is already sitting in a near-zero home frame.

## Jitter And Measurement Lessons

- The encoders did not look wildly unstable in the probe work.
- What we actually observed was small live-read wander.
- Across repeated reads, `0..3` counts of drift is normal enough that a single `2-count` miss should not be over-interpreted.
- A one-count acceptance threshold is therefore too brittle for every sample of "truth available" reasoning.
- A single failed bridge comparison at `2` counts is weak evidence by itself. Systematic divergence matters much more.
- The probe was useful because it compared all bridges in one snapshot instead of arguing from isolated reads.

## Native-Home Verification Lessons

- Native-home completion is not just "I saw bit 15 once."
- Correct verification has to include:
  - fresh metrics after the command
  - whether RTCore observed the native-home transaction as active
  - whether the tail completed
  - whether the anchor refreshed coherently
- Fast back-to-back homes can create ambiguity if the first transaction tail is still in flight.
- `J2` taught an additional lesson: "too soon after J1" was plausible but incomplete. Stale-anchor and verification-epoch issues also mattered.
- A clean restarted single-axis retry on `J2` succeeded and refreshed the anchor, which proved the joint was not fundamentally broken.

## UI And Operator Trust Lessons

- The frontend must never flatten contradictory native-home telemetry into a cheerful success message.
- We found a real UI bug where fallback telemetry made a row look effectively successful while the command result still said failure.
- Commissioning messaging must stay conservative. If telemetry conflicts, show conflict or failure, not optimism.
- The user's auditory observation about `J2` de-energizing faster was useful evidence. It fit the theory that the transaction was aborting early rather than following the normal tail.

## Test Methodology Lessons

- Separate these questions explicitly:
  - Did the drive preserve the corrected frame?
  - Did the homing-complete bit persist?
  - Did a specific object such as `607C` witness that persistence?
- We needed both manual-backed reasoning and live hardware evidence. Either one alone was not enough.
- Shared before-and-after power-cycle captures were more valuable than isolated anecdotes.
- Already-proven joints should remain in the capture set as controls while new joints are tested.
- For this bug class, the right evidence stack is:
  - direct SDO or object snapshot
  - RTCore metrics
  - API truth view
  - persisted anchor file
  - power-cycle comparison

## Working Conclusions From The Persistence Experiments

- Persistence across power cycle is real on `J3`, `J4`, and `J6`.
- A focused retry also demonstrated clean persistence behavior for `J2`.
- Those results matter because they show the remaining hard problems are not "mystery encoder instability."
- The remaining hard problems look like:
  - stale-anchor handling
  - native-home verification semantics
  - jitter-tolerant but still fail-closed truth acceptance
  - preserving operator trust in the UI

## Probe Checks To Keep Reusing

The probe is most useful when it validates all bridge families together:

- `combined(U40.20/.22) ~= sign_extend_16(U40.1E) * 131072 + (U40.1C mod 131072)`
- `6063 ~= 6064 * 6091`
- `60FC ~= 6062 * 6091`
- `combined(U40.2A/.2C) ~= U40.28 * C10_ratio`

Those formulas usually match within `0..3` counts rather than exactly zero on every sample because the reads are live, sequential, and not perfectly simultaneous.

## What Remains Open

- How stale-anchor detection and refresh should work operationally.
- Which verification semantics should govern native-home success versus pending versus failed.
- How tolerant the roundtrip acceptance threshold should be while still failing closed for real semantic mismatches.
- Whether the long-term host canonical truth should remain `U40.20/.22 + anchor` for all states, or whether the drive's corrected reference or rotation frame should become host truth after native home.

## Bottom Line

The arithmetic is now explicit and testable. The dangerous failures came from mixing the absolute multi-turn frame, the drive's reference or home frame, and the stored software anchor as if they were the same thing.

In one sentence: the probe and verification workflow taught us that the A6-EC behavior is internally coherent, but only if we stop mixing raw absolute counts, drive reference counts, and stored software anchors as though they were one frame.
