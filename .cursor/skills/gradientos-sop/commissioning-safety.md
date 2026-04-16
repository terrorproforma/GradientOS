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
- Current evidence says `0x60B0` should be treated as a runtime motion-frame object, not the durable native-home store.
- Current evidence also says `0x607C` is not a reliable witness for persisted semantic-home state on this A6-EC setup; corrected frame behavior has persisted across power cycles while `0x607C` remained `0`.
- In the drive-native migration, treat `drive_native_ratio_enabled` and startup verification as separate conditions:
  - profile/runtime config can declare the intended drive-native posture
  - controller truth should switch to the drive-owned output-shaft frame only after startup drive-config verification succeeds and the live statusword still shows the vendor-confirmed HM-valid signature with no active alarms
  - for A6-EC, do not fall back to legacy anchored reconstruction anymore; if either check fails, mark truth unavailable and require a clean HM35/vendor-valid coordinate state before trusting the pose again
- For Gradient-05 style rotary joints, Chapter 5 strongly suggests the startup absolute mode should be the rotation-mode setting, not the older linear-mode assumption.
- Native home should therefore be modeled as a commissioning-only workflow: jog in normal `CSP`, run a one-shot HM capture transaction, refresh truth, return to `CSP`, and then re-sync targets before further motion.
- After native home, keep the homed axis disabled until an explicit safe power-up; do not silently re-enable it as part of the home transaction.
- Native-home verification should use a fresh post-command RTCore metrics sample before trusting `requested`, `succeeded`, or `failed`; stale pre-command metrics can create false commissioning outcomes.
- For the active A6-EC native-home workstream, use [../../../docs/ethercat/a6ec-frame-semantics-and-native-home.md](../../../docs/ethercat/a6ec-frame-semantics-and-native-home.md) when reasoning about `607C`, `6041 bit15`, `60B0`, `60E6`, `60FC`, persisted anchors, and the `F31.10` tail.
- Current evidence from that note says `607C` and `6041 bit15` are not sufficient persistence witnesses by themselves; verification must include fresh metrics, tail completion, and coherent anchor refresh.
- Manufacturer guidance now also makes two rotary-mode rules explicit:
  - manually writing `607C` alone does not establish a valid homed/reference state; HM Method 35 must still run
  - keep `607C` within `0 .. RM-1` in rotary mode; avoid negative seam-crossing offsets and prefer a positive value near `RM-1`
- If encoder battery health or multi-turn retention is suspect, expect vendor faults such as `ALF9.0` (battery voltage low), `Er20.8` (encoder battery failure), or `Er20.9` (encoder multi-turn error), and re-validate native home before trusting the pose after a power cycle.

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
