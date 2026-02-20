---
description: "Retained lessons, user preferences, and workflow rules for agents to follow."
alwaysApply: true
---

# Retained Agent Lessons & Guardrails

These rules and preferences must be applied to all execution sessions to prevent regressions.

## Workflow & Process
- **Bias to action:** Prefer implementation over discussion; "do it, do not only explain."
- **Memory loops are mandatory:** Always use and update both `.cursor/memory/AGENT_SCRATCHPAD.md` and `.cursor/memory/DEVLOG.md`. Missing updates to either file is an incomplete task (blocker).
- **Windows testing:** On Windows sessions in this repo, default to `.\.venv\Scripts\python -m pytest` instead of `pytest` to avoid path/environment mismatch.
- **Fast validation:** Build and lint checks (`npm run build`, `ReadLints`) catch regressions quickly in the web-ui workflow. Run them frequently.
- **Config parameter synchronization:** For new configuration parameters, update all four layers together: UI draft type + API normalization + planner option parse + persistence (weld program save/load).
- **URDF source of truth:** Treat user-provided calibrated URDF joint origins as the source-of-truth over prior assumptions.

## UI / Frontend Layout & Styling
- **Minimal, clean UI:** UI preferences are specific and iterative; keep changes minimal and visual hierarchy clean.
- **Header spacing:** Keep close controls on the same line as the panel title area. UI controls must never overlap. Use structural row layouts with flex constraints (e.g., `min-w-0`, `shrink-0`) over padding hacks for header spacing.
- **No nested frames:** Remove redundant visual framing (no duplicate outer border effects/nested shells).
- **Drawer placement:** Keep robot control aligned on the right and collapsible. Keep side menus fully contained within the visible window (max-height). Visually related overlays should have matching baseline/inset behavior (e.g., `top-6 bottom-6`).
- **Scrollbar integration:** Scrollbars should feel like part of the panel, nested inside the same rounded element that defines the visual frame. Use consistent theme styling.
- **Typography hierarchy:** Visual hierarchy should be obvious; field labels should not compete with section headings. Define and use reusable typographic tokens.
- **Popovers:** Never place explainer popovers inside scrolling/clipped containers; use portal overlays (`createPortal(document.body)`) for any panel-help UI.

## React State & Effects
- **Workflow context:** Preserve in-progress workflow context (loaded STEP + weld setup) while changing tool configuration.
- **Scene bootstraps:** When debugging "panel click resets scene" reports, inspect large `useEffect` dependency arrays first. Keep scene/bootstrap effects scoped to structural identity changes (e.g., robot/model) and handle mutable visual overlays in dedicated incremental effects.
- **Equality checks:** Whenever state is hydrated from fetches, assume reference churn; gate expensive visual/resource operations behind semantic equality checks (e.g., config signatures) rather than object identity.
- **UI Sync:** Keep sync effects state-specific (selection-to-selection), and keep view-navigation state controlled only by explicit user actions.
- **Bidirectional sync:** Always track the source-of-truth per interaction (e.g., `panelSelectionOriginRef`) to prevent feedback loops/flickering.
