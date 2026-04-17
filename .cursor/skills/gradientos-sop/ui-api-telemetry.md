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

## Canonical-Truth Verification Surface

Per-axis canonical truth carries two related fields that operator tooling should surface verbatim rather than re-interpret:

- `drive_native_truth_verification_source`: how truth was established. Stable values currently emitted by shared profile code:
  - `"statusword_bits12_15_clear13"` — fresh HM success signature (vendor Q5/Q6).
  - `"statusword_bit15"` — retained coordinate system via bit 15 alone (vendor Q9; documented-but-unreachable on the A6-EC firmware we currently run, see `POSITION_SEMANTICS_CONFIG["firmware_bit15_retention_expected"]`).
  - `"persisted_home_anchor_agreement"` — host-side agreement between the persisted absolute-home anchor and live `6064` / `U40.20/.22`, opt-in per drive profile.
  - `"unverified"` — no trust source was satisfied.
- `drive_native_truth_reason` when truth is unavailable: specific cause to show the operator, including `encoder_retention_fault_present`, `persisted_home_anchor_missing`, `multi_turn_feedback_invalid`, `multi_turn_feedback_lost_across_power_cycle`, `persisted_home_anchor_inconsistent_with_live_6064`, `multi_turn_anchor_inconsistent_with_live_6064`, `coordinate_system_invalid`, `fault_present`, `manufacturer_fault_present`, `slave_offline`, `slave_not_operational`, `native_home_active`, and the `drive_native_*`-prefixed equivalents.
- `encoder_retention_fault` (per-axis, optional): matched vendor-code payload when the active drive profile decodes `manufacturer_error_code` / `error_code` into the encoder-retention family. Includes `present: bool`, `codes: [...]`, `names: [...]`, `matched_sources: [...]`. Surface the code/name directly to operators rather than raw hex; it is the specific reason the anchor path refused trust.
- `last_seen_absolute_counts` / `last_seen_delta_counts` / `last_seen_delta_physically_possible` / `last_seen_delta_budget_counts` / `last_seen_observed_at` (per-axis, optional): diagnostic sidecar on the absolute-home anchor. `last_seen_delta_counts` is live `absolute_counts − stored_last_seen_absolute_counts`. When the shaft-frame gate fails AND `last_seen_delta_physically_possible` is `False`, the reason upgrades to `multi_turn_feedback_lost_across_power_cycle`.

UI messaging should:

- show the verification source when truth is available (operators should know which path is carrying trust, especially when `persisted_home_anchor_agreement` is active and bit 15 is cleared),
- surface the specific reason verbatim when truth is unavailable instead of collapsing to a generic "not ready",
- treat `multi_turn_feedback_invalid` / `multi_turn_feedback_lost_across_power_cycle` / `persisted_home_anchor_inconsistent_with_live_6064` / `encoder_retention_fault_present` as "operator must re-home" states rather than as generic faults. `encoder_retention_fault_present` specifically implies the underlying encoder battery / multi-turn integrity needs attention before a re-home will stick.

## Native-Home Result Surface

- `native_home_joint` preserves structured outcome: `accepted`, `verified`, `timed_out`, `code`, `native_home_state`, `native_home_last_abort_code` / `_hex`, and `disarmed_after_home`.
- Abort codes in the `0xFxxxxxxx` synthesized range are host/RTCore-side (not CoE SDO aborts). Currently `0xF1000001 = disarm_precondition_timeout`. Surface these as distinct codes, not as generic failures.
- `disarm_precondition_timed_out=True` on a result means Stage-A drive-confirmed disarm never happened; show this as a pre-HM35 precondition failure rather than an HM35 failure.

## First Files

- `src/gradient_os/api/main.py`
- `src/gradient_os/telemetry/drive_faults.py`
- `src/gradient_os/telemetry/encoder_retention.py`
- `web-ui/src/liveState.tsx`
- `web-ui/src/App.tsx`
- `web-ui/src/ControlPanel.tsx`
