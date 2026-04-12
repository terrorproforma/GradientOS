# UI, API, and Telemetry

Primary SOP sections: `§5.5`, `§9.2B-9.2C`, `§15-16`

Use this file when extending REST/SSE payloads, live state, UI controls, or operator-facing status text.

## Thinness Rules

- The frontend owns presentation, local interaction flow, and display-level state.
- The API owns transport, request parsing, response shaping, and controller wiring.
- Neither layer should invent motion policy, drive semantics, or backend truth that belongs lower in the stack.

## Telemetry Rules

- Extend existing payloads before creating new ones.
- Keep telemetry contracts generic in shared layers even when the underlying source is drive-profile-specific.
- Prefer structured objects over piles of vendor-named alias fields.
- Add descriptive labels when raw numeric values are not enough for operators.

## Command Rules

- UI actions should reuse existing controller/API pathways where possible.
- Do not create frontend-only commissioning semantics if a controller command already exists.
- Keep disabled states and messaging honest about runtime mode, backend state, and actual blockers.

## Payload Design Rules

- If data belongs to an existing live snapshot, motion status response, or fault payload, extend that shape.
- Avoid parallel status channels that can drift from the existing source of truth.
- Keep naming generic in common payloads and leave manufacturer-specific wording to the profile layer or operator-facing formatting.
- For commissioning actions with asynchronous verification, preserve structured command results such as `accepted`, `verified`, and `timed_out` instead of collapsing them into a boolean or generic transport failure.
- If live telemetry can later refine the operator view, UI messaging should degrade to a warning/pending state rather than contradicting a successful live status with a generic request failure.

## Common Checks

- If a new field is shown in the UI, confirm the same field is available and typed consistently through:
  - controller/backend snapshot
  - API normalization
  - frontend live-state types
  - consuming components
- If a new action is added, verify it uses an existing backend/controller contract before creating a new API surface.

## First Files

- `src/gradient_os/api/main.py`
- `src/gradient_os/telemetry/drive_faults.py`
- `src/gradient_os/telemetry/encoder_retention.py`
- `web-ui/src/liveState.tsx`
- `web-ui/src/App.tsx`
- `web-ui/src/ControlPanel.tsx`
