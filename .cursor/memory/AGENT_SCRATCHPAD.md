# Agent Scratchpad

Use this file as persistent, repo-local execution memory.

## File Policy

- Current policy: `COMMITTED`
- Rationale:
  - The user explicitly asked for persistent use of scratchpad/devlog skills and visible top-level references.

## How To Use

1. Read latest entries before starting meaningful work.
2. Build a short preflight checklist from recurring mistakes and preferences.
3. Re-read before risky operations (migrations, broad refactors, unfamiliar tooling, destructive commands).
4. Log high-signal learnings immediately during the task.
5. Append one new session entry before handoff.
6. Keep entries concrete, concise, and testable.

## Entry Rules

- Tag operational notes with source: `[self]`, `[user]`, or `[tool]`.
- Prefer facts tied to files, commands, and outcomes.
- Do not log low-signal reminders.

## Retained Lessons

### Workflow & Process
- [user] Prefer implementation over discussion; "do it, do not only explain."
- [user] Always use and update both `.cursor/memory/AGENT_SCRATCHPAD.md` and `.cursor/memory/DEVLOG.md`. Missing updates to either file is an incomplete task (blocker).
- [self] On Windows sessions in this repo, default to `.\.venv\Scripts\python -m pytest` instead of `pytest` to avoid path/environment mismatch.
- [tool] Build and lint checks (`npm run build`, `ReadLints`) catch regressions quickly in the web-ui workflow. Run them frequently.
- [self] For new configuration parameters, update all four layers together: UI draft type + API normalization + planner option parse + persistence (weld program save/load).
- [user] Treat user-provided calibrated URDF joint origins as the source-of-truth over prior assumptions.

### UI / Frontend Layout & Styling
- [user] UI preferences are specific and iterative; keep changes minimal and visual hierarchy clean.
- [user] Keep close controls on the same line as the panel title area. UI controls must never overlap.
- [self] Use structural row layouts with flex constraints (e.g. `min-w-0`, `shrink-0`) over padding hacks for header spacing.
- [user] Remove redundant visual framing (no duplicate outer border effects/nested shells).
- [user] Keep robot control aligned on the right and collapsible.
- [user] Keep side menus fully contained within the visible window (max-height). Visually related overlays should have matching baseline/inset behavior (e.g., `top-6 bottom-6`).
- [user] Scrollbars should feel like part of the panel, nested inside the same rounded element that defines the visual frame. Use consistent theme styling.
- [user] Visual hierarchy should be obvious; field labels should not compete with section headings. Define and use reusable typographic tokens.
- [self] Never place explainer popovers inside scrolling/clipped containers; use portal overlays (`createPortal(document.body)`) for any panel-help UI.

### React State & Effects
- [user] Preserve in-progress workflow context (loaded STEP + weld setup) while changing tool configuration.
- [self] When debugging "panel click resets scene" reports, inspect large `useEffect` dependency arrays first. Keep scene/bootstrap effects scoped to structural identity changes (e.g., robot/model) and handle mutable visual overlays in dedicated incremental effects.
- [self] Whenever state is hydrated from fetches, assume reference churn; gate expensive visual/resource operations behind semantic equality checks (e.g. config signatures) rather than object identity.
- [self] Keep sync effects state-specific (selection-to-selection), and keep view-navigation state controlled only by explicit user actions.
- [self] For bidirectional UI sync, always track the source-of-truth per interaction (e.g. `panelSelectionOriginRef`) to prevent feedback loops/flickering.

## Session Entries

*(Session entries are cleared regularly. Chronological implementation logs belong in `.cursor/memory/DEVLOG.md`. Historical scratchpad entries are archived in `.cursor/memory/AGENT_SCRATCHPAD_ARCHIVE.md`)*

### 2026-02-21 - Gradient-05 limits/backend hardening
- [self] When replacing structured config blocks from scripts, use explicit parser/validator steps plus `--dry-run` before writes; this avoids accidental regex-only blind edits.
- [self] For backend mapping policy changes, add focused regression tests for both default behavior and env overrides in the same task to lock migration intent.
- [user] Preferred policy for EtherCAT mapping is direct-order defaults with env override retained; do not keep hidden special-case defaults (like implicit J3/J4 for 2-axis bring-up).
