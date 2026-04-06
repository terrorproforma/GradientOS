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

## Commissioning Motion Rules

- Reuse the existing controller/backends command pathways for jog, zero, and home.
- Keep commissioning motion conservative and explicitly capped.
- Make safety limits real in the payload/controller path, not just explanatory text in the UI.
- Use clear operator messaging about what the move does and what state the robot must be in first.

## Power and STOP

- STOP behavior must be backend-aware and must not re-inject stale targets.
- Power transitions should reflect actual blockers and readiness, not optimistic guesses.
- Neutral-state and disarmed-state rules must stay visible in operator-facing status.

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
