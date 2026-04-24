# Commissioning and Safety

Primary SOP sections: `§3.4-3.5`, `§10`, `§12.3-12.4`, `§19.2-19.4`

Use this file for power transitions, safe restarts, commissioning motion flows, zero/home behavior, and encoder-retention verification.

## Startup and Restart Rules

- Startup should land `BUS_UP_DISARMED`, not auto-enabled.
- Preserve the no-sudden-move contract across restart, enable, and disable flows.
- Treat power-up and power-down as first-class operations with explicit guards and status.

## Zero vs Native Home

- `ZERO_JOINT` is a software logical offset operation.
- Native drive home is a drive-side home/reference operation.
- Keep both available, but keep their semantics distinct in code, telemetry, and UI copy.
- Do not quietly collapse one concept into the other.
- For A6-EC, distinguish:
  - startup absolute-system selection via `C00.07 / 0x2000:08`
  - rotary-mode mechanical ratio via `C10.18 / C10.19`
  - runtime DS402 operating mode via `0x6060`
  - the homing/reference contract via `0x6098`, `0x607C`, `0x60E6`, `0x6040`, and `0x6041`
  - the CSP runtime position-offset term `0x60B0`
- Current manufacturer guidance says the intended rotary-mode production setup is to configure the real output-shaft mechanical ratio in `C10.18 / C10.19`; leaving them at `1:1` makes `6064`/`U40.28` motor-side and forces the host to own the output-shaft conversion itself.
- Any change to `C10.18 / C10.19` requires a fresh homing cycle afterward, even if no power cycle is needed.
- `0x60B0` is a runtime motion-frame object, not a durable native-home store. Do not treat it as persistent home state.
- `0x607C` is the manufacturer-documented persistent home offset (auto-saved to EEPROM per vendor Q3/Q7). It DOES survive drive power cycle. But it is the home-capture target, not a standalone runtime trust witness.
- For A6-EC, do not fall back to legacy anchored reconstruction when `drive_native_ratio_enabled` is on; instead let the canonical truth path run with its three trust sources (see "Restart Trust Model" below). If all three fail, mark truth unavailable and require a clean HM35 before trusting the pose again.
- For Gradient-05 style rotary joints, Chapter 5 strongly suggests the startup absolute mode should be the rotation-mode setting, not the older linear-mode assumption.
- Native home should be modeled as a commissioning-only workflow: jog in normal `CSP`, run a one-shot HM capture transaction, refresh truth, return to `CSP`, and then re-sync targets before further motion.
- After native home, keep the homed axis disabled until an explicit safe power-up; do not silently re-enable it as part of the home transaction.
- Native-home verification should use a fresh post-command RTCore metrics sample before trusting `requested`, `succeeded`, or `failed`; stale pre-command metrics can create false commissioning outcomes.
- For the active A6-EC native-home workstream, use [../../../docs/ethercat/a6ec-frame-semantics-and-native-home.md](../../../docs/ethercat/a6ec-frame-semantics-and-native-home.md) when reasoning about `607C`, `6041 bit15`, `60B0`, `60E6`, `60FC`, persisted anchors, and the `F31.10` tail.
- Manufacturer guidance on rotary-mode homing:
  - manually writing `607C` alone does not establish a valid homed/reference state; HM Method 35 must still run
  - keep `607C` within `0 .. RM-1` in rotary mode; avoid negative seam-crossing offsets and prefer a positive value near `RM-1`
- If encoder battery health or multi-turn retention is suspect, expect vendor faults such as `ALF9.0` (battery voltage low), `Er20.8` (encoder battery failure), `Er20.9` (encoder multi-turn error), or `ErA0.1` (multi-turn overflow fault), and re-validate native home before trusting the pose after a power cycle. Vendor email 4 Q2(a) lists these four codes as the primary signals that `U40.20/U40.22` is no longer reliable.

## Native-Home Disarm Precondition

Vendor Q2 requires the motor to be "stationary AND inactive" before HM35. The host-side `motion_intent_cleared` flag is not sufficient on its own. Contract:

- RTCore `MSG_CMD_NATIVE_HOME` runs in two stages. Stage A clears `axis_enable_mask` / `armed` / motion intent, then polls each targeted axis's `statusword` until it has left `OperationEnabled` / `QuickStopActive` for several consecutive cycles (`~500 ms` budget). Stage B runs the existing HM35 transaction unchanged. Timeout synthesizes abort code `NATIVE_HOME_ABORT_DISARM_PRECONDITION_TIMEOUT = 0xF1000001` in the reserved `0xFxxxxxxx` RTCore-side range.
- On the Python side, `EthercatRTCoreBackend.prepare_for_power_transition` accepts `require_drive_disarmed=True` + `require_drive_disarmed_axis_mask=<mask>`. When set, neutrality is `motion_intent_cleared AND all targeted axes not OperationEnabled` sourced from `get_power_transition_snapshot().per_axis_drive_disarmed`. `native_home_joint` always calls the stronger variant before `_send_cmd_native_home`.
- Observable expectation: one brake click (disarm) per native-home action, not a disarm + HM35-enable pair. A second click means the precondition contract was bypassed.

## Command-Frame Turn Selection (A6-EC)

`607A` targets must land in the same shaft turn the drive is currently observing. Getting this wrong produces `Er87.1` (excessive position reference increment) or `Er47.0` (following error) on seam-adjacent small jogs. Contract:

- Turn selection is stateless per write. The host computes `target_counts = round((canonical_q + master_offset) * sign * counts_per_unit)` and folds to the nearest shaft turn of live `6064` using `delta -= round(delta / RM) * RM`.
- `abs(target_607A - live_6064) <= RM / 2` is an invariant by construction. Violating it raises `command_frame_oversized_delta` as a regression guard.
- Do NOT carry a cached `raw_reference_wrap_lift` quantity between writes; stale-state is what produced the original `Er87.1` family.
- Trajectory upload path has a pre-commit sanity gate: each consecutive point's `607A` step must be `<= 0.5 * RM` or the upload is rejected with `command_frame_oversized_step`. This is a frame-sanity fence, not a motion clamp; per-cycle motion clamping itself lives in RTCore `max_step_counts_per_cycle`.

## Multi-Turn Truth and Shaft-Frame Consistency (A6-EC)

Vendor Q4/Q10 says "host only needs `6064`". That claim applies to single-turn-bounded axes. It is NOT sufficient for joints whose software limits exceed one shaft revolution (Gradient-05 J1/J4/J5/J6). `6064` wraps at `RM` in absolute rotation mode and cannot carry multi-turn continuity. Contract:

- Canonical planner/controller truth is `canonical_q = absolute_axis_q − absolute_home_anchor − master_offset` where `absolute_axis_q` is derived from the multi-turn `U40.20 / U40.22` pair and the anchor lives in `.gradient_absolute_encoder_anchors.json`.
- Before publishing canonical truth, verify that `(canonical_q + master_offset) * sign * counts_per_unit` agrees with the paired `6064` snapshot modulo `RM` within `_SHAFT_FRAME_CONSISTENCY_TOLERANCE_COUNTS` (currently `4096` counts).
- The tolerance is intentionally sized for the "drive lost track of WHICH full revolution" failure mode (`131072` counts on G05 encoders). Sub-revolution deltas up to `~2000` counts are motion-pair-skew between the drive's internal `U40.20 / U40.22` and `0x6064` sampling and should not trip the gate. `4096` is `32×` smaller than a full revolution, so real retention failures remain detectable. Do NOT tighten back toward `16` unless the paired-snapshot and pair-skew physics fundamentally change.
- Whole-shaft-turn offsets between the anchored view and the paired `6064` are legitimate (that is what the anchor encodes). Sub-shaft-turn drift above tolerance is a frame bug; truth fails closed with reason `multi_turn_anchor_inconsistent_with_live_6064` (or the drive-native-prefixed equivalent) and the stale-anchor diagnostic fields are attached.
- Do not demote `U40.20/.22` to diagnostics-only for this drive family; multi-turn-capable joints require it.

### Paired-Snapshot Reference for the Gate (A6-EC)

The shaft-frame gate must compare values from the SAME moment. The A6-EC advertises `U40.20 / U40.22` as PDO-mappable but firmware does NOT populate them cyclically, so multi-turn is SDO-only and asymmetric with the `0x6064` PDO cycle. RTCore latches `0x6064` just before issuing each axis's `U40.20 / U40.22` SDO upload and publishes it as `paired_pos_counts` inside the `absolute_feedback` JSON payload. Python's gate reads the paired value via `_AbsoluteFeedbackAxisMetrics.paired_pos_counts()` and passes it in as the shaft-frame reference, NOT the live-now PDO value. Fallback to live `0x6064` only when `paired_valid = 0` (pre-paired window). `shaft_frame_reference_source = "paired_sdo_snapshot" | "live_pdo"` on every axis diagnostic confirms which path ran. See `RTOS_ETHERCAT_MASTER_OPERATING_PRINCIPLES.md §9.5A` for the full rationale.

## Restart Trust Model (A6-EC)

Vendor Q6/Q9 says `6041 bit 15` persists across drive power cycle. On this A6-EC firmware it empirically does NOT — bit 15 is cleared on every drive power-up while `6064`, `U40.20/.22`, and `607C` restore cleanly (including across manual joint rotation while the drive is off, within the vendor Q1 32,767-motor-turn budget). Canonical truth is established via three trust paths evaluated in order (first match wins):

1. Fresh-home strict: `bit 12 ∧ bit 15 ∧ ¬bit 13` with no active alarms → `drive_native_truth_verification_source = "statusword_bits12_15_clear13"`.
2. Bit-15-alone (vendor Q9 literal path): `bit 15` set and `startup_truth_requires_hm_success_signature=False` in the profile → `"statusword_bit15"`. **Documented-but-unreachable on the A6-EC firmware we currently run**, because bit 15 is cleared on every drive power cycle (see `POSITION_SEMANTICS_CONFIG["firmware_bit15_retention_expected"] = False` in `src/gradient_os/arm_controller/profiles/drive/a6ec_ds402.py`). Kept in code for future drives / firmware revisions that honour Q9. See `RTOS_ETHERCAT_MASTER_OPERATING_PRINCIPLES.md §9.7` for the canonical version of this rule.
3. Persisted-home-anchor agreement: profile sets `accept_persisted_home_anchor_as_restart_trust=True` AND the `.gradient_absolute_encoder_anchors.json` entry for the joint is present AND `U40.20/.22` reports `valid=1` AND the mod-`RM` shaft-frame check is consistent within tolerance AND no encoder-retention-family fault is live → `"persisted_home_anchor_agreement"`.

If none match, truth fails closed with a specific reason (`encoder_retention_fault_present`, `persisted_home_anchor_missing`, `multi_turn_feedback_invalid`, `persisted_home_anchor_inconsistent_with_live_6064`, `multi_turn_feedback_lost_across_power_cycle`, or the generic `coordinate_system_invalid`). The operator must re-home.

Contract:

- Initial HM35 is always required to establish the anchor file entry per joint. The restart-trust path reuses state; it does not invent trust.
- After that initial home, power cycles do NOT require re-homing as long as the encoder's multi-turn counter stays valid.
- Manual joint rotation while the drive is off is accepted by the restart-trust path because `absolute_axis_q − reference_q` is invariant under shaft motion. Only encoder data loss (battery death, `> 32,767` motor turns, or catastrophic encoder fault) breaks the invariant and forces a re-home.
- Live `Er20.1 .. Er20.9` / `ErA0.1` / `ALF9.0` (encoder-retention-family fault codes decoded from `manufacturer_error_code` / `error_code`) surface as `encoder_retention_fault_present` and block trust regardless of anchor agreement. This outranks the generic `fault_present` / `manufacturer_fault_present` branches because retention is a more specific interpretation of the same underlying 0x603F / 0x203F signal.
- `F31.10 = 4` (encoder data reset) always requires a fresh HM35 afterward; restart trust is not a substitute.
- Optional last-seen U40.20/.22 sidecar: the backend persists the live `absolute_counts` onto the anchor entry under `last_seen` on every trusted-axis canonical-truth cycle (rate-limited to once per 5 s per joint). When the shaft-frame consistency gate later fails AND the delta from the stored sidecar exceeds `32,767 × counts_per_rev` (physically impossible during an off-window), the rejection reason upgrades to `multi_turn_feedback_lost_across_power_cycle` instead of the generic anchor-disagreement label, so operators can tell "joint moved while off" apart from "encoder data lost while off".

## Commissioning Motion Rules

- Reuse the existing controller/backends command pathways for jog, zero, and home.
- Keep commissioning motion conservative and explicitly capped.
- Make safety limits real in the payload/controller path, not just explanatory text in the UI.
- Use clear operator messaging about what the move does and what state the robot must be in first.
- Keep native-home execution scoped to one axis at a time until startup mode, frame composition, and persistence are revalidated.
- Do not treat native-home capture as a normal runtime motion feature; it is a commissioning workflow with extra preconditions and post-checks.
- If native-home verification does not converge within the current wait window, degrade operator messaging to "pending verification" rather than a generic request failure, and require the axis to stay disabled until telemetry settles.

## Power and STOP

- STOP behavior must be backend-aware and must not re-inject stale targets.
- Power transitions should reflect actual blockers and readiness, not optimistic guesses.
- Neutral-state and disarmed-state rules must stay visible in operator-facing status.

## Drive Reset Objects

- For A6-EC, the manual-backed reset family lives under `2031h / F31` and should be treated as part of commissioning/recovery guidance rather than tribal knowledge:
  - `F31.00 / 0x2031:01`: fault reset
  - `F31.01 / 0x2031:02`: software reset
  - `F31.02 / 0x2031:03`: parameter initialization
  - `F31.03 / 0x2031:04`: drive/motor parameter reset
  - `F31.10 / 0x2031:11`: encoder data reset/read/write/fault reset
- These objects are documented as `At stop` and `Immediately` effective.
- `F31.01` software reset is only valid while the drive is disabled and there is no non-resettable fault.
- `F31.10` encoder reset is high risk for absolute-position workflows: multi-turn reset can abruptly change the saved absolute position and therefore requires mechanical homing before the pose is trusted again.
- Do not use `F31.02` or `F31.03` as the first recovery step for unexplained pose/frame anomalies. They are destructive enough to turn a diagnosable commissioning problem into a full recommissioning problem.
- Preferred reset ordering for current A6-EC commissioning:
  - `F31.00` fault reset when a resettable fault is present
  - `F31.01` software reset as a controlled low-risk probe when disarmed
  - `F31.10`, `F31.02`, and `F31.03` only with a parameter backup, a re-home plan, and explicit intent to re-establish trust from scratch

## Encoder Retention

- Treat encoder retention verification as a formal commissioning workflow.
- Capture before/after snapshots through the existing endpoints and telemetry paths.
- Compare raw counts, logical angles, startup drive config verification, and fault state together.
- Store experiment artifacts under `logs/encoder-retention/`.

## First Files

- `src/gradient_os/api/main.py`
- `src/gradient_os/telemetry/encoder_retention.py`
- `src/gradient_os/arm_controller/backends/ethercat_rtcore/backend.py`
- `src/gradient_os/run_controller.py`
- `web-ui/src/ControlPanel.tsx`
- `docs/ethercat/bringup.md`
- `docs/resources/A6-EC_series_servo_drive_manual.pdf`
- `docs/resources/a6ec_manual_codes.md`
