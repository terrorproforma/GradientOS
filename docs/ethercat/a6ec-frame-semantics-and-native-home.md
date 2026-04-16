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

There are now two relevant host-side scaling postures:

- Legacy host-owned posture:
  - `counts_per_radian = (encoder_counts_per_rev * gear_ratio) / (2 * pi)`
  - `axis_q_rad = raw_counts / (sign * counts_per_radian)`
- Drive-native ratio posture:
  - the real mechanical ratio is programmed into `C10.18 / C10.19`
  - the host command/reference path should therefore use `counts_per_radian = encoder_counts_per_rev / (2 * pi)` for the `6064` / `607A` frame, so the gearbox ratio is not applied twice

Equivalently in the legacy posture:

- `axis_q_rad = sign * raw_counts * 2 * pi / (encoder_counts_per_rev * gear_ratio)`

This is why large-ratio joints originally needed a continuous multi-turn motor source for truth reconstruction, but the long-term drive-native posture can instead let the drive expose output-shaft coordinates directly.

### Canonical Truth Math

The active A6-EC controller truth contract is now single-path:

- Drive-native production mode:
  - `canonical_q = q_from_6064_frame - software_zero`
  - no host-side absolute-home anchor participates in active truth
- Fail-closed mode:
  - keep the configured truth source as the drive output-shaft frame
  - verify the startup posture through the existing `startup_drive_config` readback channel
  - trust canonical truth only when the live statusword also shows the vendor-confirmed HM-valid signature (`bit12 = 1`, `bit15 = 1`, `bit13 = 0`) with no active alarms
  - if startup verification is missing or the live statusword is not HM-valid, report truth unavailable instead of reconstructing any host-owned alternative

In the active A6-EC path, the write path should round-trip as:

- `reference_q = canonical_q + software_zero`

No extra anchor term belongs on the command path. Once A6-EC is in the drive-native posture, the host should not bridge back through a stored absolute-home anchor to recover canonical truth.

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

## Manufacturer Clarifications 2026-04-15

The vendor reply added several clarifications that materially sharpen this note.

- "Load" and `RM` in rotary mode mean the output or load shaft after the gearbox, not the motor shaft.
- The vendor's recommended final configuration is to set the real mechanical ratio in `C10.18 / C10.19`.
- When `C10.1A / C10.1C` are zero, rotary-mode `RM` is calculated from encoder resolution times the mechanical ratio in `C10.18 / C10.19`.
- The vendor now explicitly says that if `C10.18 / C10.19` are left at `1:1`, then `6064` and `U40.28` remain motor-side rather than output-shaft coordinates, and the host must continue doing the higher-level conversion work itself.
- That means a `1:1` ratio setup is best understood as a host-conversion/debug posture, not the intended long-term production configuration.
- The vendor also explicitly distinguishes `C10.18 / C10.19` from `6091`:
  - `C10.18 / C10.19` govern rotary-mode absolute-position reconstruction and `RM`
  - `6091` is the electronic gear ratio used for command-unit conversion
- This does not make the Chapter 11 `6064 * 6091 ~= 6063` bridge checks useless; it means those checks validate the command/reference-unit mapping, not the rotary-mode absolute reconstruction policy by themselves.
- Changing `C10.18 / C10.19` requires a fresh homing cycle afterward, even though no power cycle is required.
- The vendor states that manually writing `607C` alone is not sufficient to establish a valid homing/reference state:
  - `6064` does not jump immediately
  - bits 12 and 15 do not become valid merely from the write
  - HM Method 35 is still required
- That exactly matches our bench result that direct nonzero `607C` writes did not selectively rebase `6064` or API truth on their own.
- The vendor now says negative `607C` values in rotary mode are semantically invalid even if the implementation permits them to be written and read back.
- Their recommendation is to keep `607C` in `0 .. RM-1` and, for seam-adjacent homes, use a positive value near `RM-1` rather than a negative offset.
- The vendor confirms `0x9650` as the expected good post-HM state:
  - bit 12 = 1
  - bit 15 = 1
  - bit 13 = 0
  - no active alarms
- The vendor also frames `F31.10` as a recovery tool for encoder battery/multi-turn failure cases, not as a routine commissioning step. After `F31.10 = 4`, physical homing is required again.

## Integration Implications From That Reply

- The strongest newly-supported implementation direction is to reintroduce actual `C10.18 / C10.19` startup configuration in the drive profile/runtime path and then re-home once to seed the corrected coordinate system.
- That should be treated as a deliberate migration, not a casual config tweak, because the vendor explicitly warns that changing the ratio changes the coordinate system immediately.
- The code now programs `C00.07` plus `C10.18 / C10.19` at startup from robot-config mechanics, and the startup-validity gate now requires all three startup descriptors to read back coherently before the drive posture is treated as verified.
- The host canonical truth path now treats the drive reference/output-shaft frame as the only A6-EC truth source, but it only becomes available when both conditions hold:
  - startup posture is verified
  - live statusword still carries the vendor-confirmed HM-valid signature with no active alarms
- If either condition is missing, the controller deliberately leaves truth unavailable rather than pretending a host-side reconstruction is trustworthy.
- The write-path guardrail still stands: once the drive owns the mechanical ratio, the host command/reference conversion must not multiply by the gearbox ratio again.
- A live `start-stack.sh` validation on 2026-04-15 confirmed the active runtime env now carries:
  - `GRADIENT_RT_GEAR_RATIO="1,1,1,1,1,1"`
  - `GRADIENT_RT_DRIVE_STARTUP_SDO_CONFIG` with `C00.07`, `C10.18`, and `C10.19`
- That same live run also confirmed the current hardware state is still pre-trust rather than code-broken:
  - all six axes reported `drive_native_startup_valid = true`
  - all six axes reported `drive_native_truth_valid = false`
  - every axis statusword was `0x1650`, so the coordinate system was still invalid until a fresh HM35 re-home re-establishes the vendor-confirmed `0x9650` state

## Motion And Safety Lessons

- The multi-axis jog regression was a command-frame math bug, not just a display bug.
- The bad change was re-applying `absolute_home_anchor_rad` on the write path.
- That was wrong because the anchor cancels algebraically on the command path.
- Command upload must stay in the RTCore and `6064` and `607A` reference frame.
- The command path must not "helpfully" add the absolute anchor back in.
- Raw absolute multi-turn counts are essential for diagnostics and truth reconstruction, but they are not automatically the right frame to command motion from.
- Historically the live stack ran with `C10.18/C10.19 = 1:1`, which forced the host to own the higher-level joint semantics. The current migration removes that posture by programming the real mechanical ratio into the drive at startup and refusing to invent a second host-owned truth path.

## Truth, Anchors, And Fail-Closed Behavior

- Removing alternate host truth reconstruction was the right call. Those alternate paths made it too easy to hide frame problems behind something merely plausible.
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
- We found a real UI bug where a secondary telemetry interpretation made a row look effectively successful while the command result still said failure.
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
