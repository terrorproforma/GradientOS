# Web UI Design System (Living Doc)

This file defines the baseline visual system for drawer-style panels and related controls in `web-ui`.
Update it whenever UI styling decisions are made so consistency is intentional, not accidental.

## Purpose

- Keep typography, spacing, and component rhythm consistent across STEP, Trajectory, Weld, and Telemetry drawers.
- Provide a shared source of truth for UI polish changes.
- Reduce one-off class drift in `web-ui/src/App.tsx`.

## Design Direction

- Theme: dark industrial control UI.
- Tone: restrained, high-contrast, readable under dense information.
- Priority: clarity and hierarchy over decorative styling.

## Typography Hierarchy

- Section titles:
  - Use stronger hierarchy: `text-[14px] font-semibold text-slate-200`.
- Field labels:
  - Use regular hierarchy: `text-[13px] font-normal text-slate-300`.
- Inputs/select text:
  - Use `text-[13px]` for readability and consistency.
- Meta/supporting text:
  - Use `text-[12px] text-slate-400`.
- Primary action text:
  - Use `text-[13px] font-semibold`.

## Shared Panel Tokens

Current shared token names in `web-ui/src/App.tsx`:

- `DRAWER_LABEL_CLASS`
- `DRAWER_INPUT_CLASS`
- `DRAWER_INLINE_INPUT_CLASS`
- `DRAWER_META_TEXT_CLASS`
- `DRAWER_SECTION_TITLE_CLASS`
- `DRAWER_ACTION_TEXT_CLASS`

Use these tokens before introducing new text-size classes.

## Drawer Layout Rules

- Drawer shell must stay as one clipped container:
  - Rounded corners on outer shell.
  - Header and scroll body inside same shell.
  - `overflow-hidden` on shell to preserve corner shape while scrolling.
- Avoid nested outer panel shells inside drawer content:
  - Do not wrap panel content in an additional full-card frame (`rounded/border/bg/shadow`) inside the drawer body.
  - Keep only section-level cards inside panel content (e.g. topology card, edge segment card) so content has more usable width.
- Vertical placement:
  - Use fixed shared insets (`top-6`, `bottom-6`) for the drawer overlay lane.
  - Keep non-drawer UI (e.g. Robot Control, vision card) aligned to the same `bottom-6` baseline.
- Height behavior:
  - All drawer panels use content-driven shell (`max-h-full`) so sparse panels do not appear stretched to the bottom.
  - When content exceeds available lane height, drawer caps at lane bounds (`top-6`/`bottom-6`) and body scrolls internally.
  - Never let long content extend beyond the overlay lane; content must scroll internally.
- Scrolling:
  - Use internal scroll region only.
  - Keep custom themed scrollbar (`gradient-scrollbar`) for visual consistency.
- Width behavior:
  - Default drawers use standard width.
  - Telemetry/charts drawer may use a wider variant to prevent horizontal overflow.

## Settings Modal Layout Rules

- Settings modal should cap vertical size to viewport (`max-h` with top/bottom breathing room), not overflow the window.
- Keep header + tab row fixed; content area scrolls internally with `gradient-scrollbar`.
- Do not allow tool/kinematics forms to push action controls beyond viewport bounds.

## Tooltip and Popover Rules

- Explainer tooltips/popovers tied to drawer fields must render in a portal (`document.body`) with fixed positioning.
- Do not render tooltips inside drawer scroll containers (`overflow-auto` / `overflow-hidden`) or they will clip.
- Clamp overlay position to viewport bounds on open, resize, and scroll.
- Support outside-click and `Escape` close behavior for overlays.
- Interactive diagrams in tooltips should use pointer capture while dragging for stable interaction.

## Form and Spacing Rules

- Prefer 8px rhythm (`gap-2`, `mt-2`, `py-2`) for dense controls.
- Keep section cards consistent:
  - Border weight/tone should match existing panel cards.
  - Avoid mixed corner radii inside same panel.
- Inputs/selects/buttons in same visual group should share height and text size.

## Consistency Checklist (Before Handoff)

- Are section title and field label hierarchy visually distinct?
- Do STEP, Trajectory, and Weld use the same typographic scale?
- Is scrollbar inside panel shell and themed?
- Are bottom corners rounded at all scroll positions?
- Is drawer bottom aligned to the Robot Control baseline (`bottom-6`)?
- Do sparse panels (STEP / Trajectory / Telemetry) avoid full-height empty stretching?
- Do tooltips/popovers open fully without clipping at panel edges?
- Does `npm run build` pass?
- Are edited files lint-clean?

## Change Log Notes

When adjusting design rules:

1. Update this file.
2. Reference the change in `AGENTS.md`.
3. Log implementation/validation in `DEVLOG.md`.
