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

- [user] Prefer implementation over discussion; "do it, do not only explain."
- [user] UI preferences are specific and iterative; keep changes minimal and visual hierarchy clean.
- [tool] Build and lint checks (`npm run build`, `ReadLints`) catch regressions quickly in the web-ui workflow.

## Session Entries

### 2026-02-16 00:14 +11:00 - Sidebar UX refinement and workflow persistence

#### Task Summary

- Adjusted drawer close-button placement and panel framing behavior per user screenshot feedback.
- Kept robot control right-docked and collapsible.
- Added explicit top-level workflow pointers and persistent memory files.

#### Mistakes And Fixes

- Source: `[self]`
- Mistake:
  - Initial close-button placement still appeared outside the panel because drawer width did not match panel width behavior.
- Detection:
  - User screenshot showed the close icon floating outside the card boundary.
- Fix:
  - Synchronized drawer width to panel scale and repositioned close button offsets.
- Preventive rule:
  - When overlay controls must align with a child card, validate parent width/position assumptions before tweaking z-index/offsets.

#### User Preferences

- New or reinforced preference:
  - Keep close controls on the same line as the panel title area.
  - Remove redundant visual framing (no duplicate outer border effect).
  - Keep robot control aligned on the right and collapsible.
  - Always maintain devlog/scratchpad workflow and keep `.cursor/skills` references visible.
- How it changed execution:
  - Prioritized layout simplification and added top-level workflow references.

#### What Worked

- Pattern/check that worked:
  - Small targeted CSS/class updates in drawer wrapper and deterministic build verification.

#### What Did Not Work

- Failed attempt and why:
  - Width-only tweak without checking `w-full max-w-*` interactions can leave floating controls misaligned.

#### Guardrails For Next Session

- Preflight rule:
  - Read this scratchpad + `DEVLOG.md` first, then align any overlay control to the actual rendered panel width before finalizing.

#### Follow-Ups / Risks

- Remaining risk or pending check:
  - Confirm visual alignment at multiple viewport sizes after future panel style changes.

### 2026-02-16 00:20 +11:00 - Prevent tab lock from tree sync

#### Task Summary

- Fixed behavior where loading STEP or existing tree selection auto-forced Weld tab.
- Restored manual tab switching while preserving weld/tree selection sync.

#### Mistakes And Fixes

- Source: `[self]`
- Mistake:
  - Program-tree synchronization effect controlled `activePanel`, unintentionally overriding user tab changes.
- Detection:
  - User reported automatic tab jump to Weld and inability to switch tabs afterward.
- Fix:
  - Removed panel-forcing from tree-sync effect; moved panel-open behavior to explicit tree click handler.
- Preventive rule:
  - Keep sync effects state-specific (selection-to-selection), and keep view-navigation state controlled only by explicit user actions.

#### User Preferences

- New or reinforced preference:
  - Loading a STEP model must not auto-navigate to Weld.
  - User must be able to switch tabs freely at all times.
- How it changed execution:
  - Prioritized decoupling `activePanel` from background sync logic.

#### What Worked

- Pattern/check that worked:
  - Isolating tree sync side effects and validating with build quickly confirmed fix stability.

#### What Did Not Work

- Failed attempt and why:
  - Coupling panel navigation to derived tree focus caused repeated tab override loops.

#### Guardrails For Next Session

- Preflight rule:
  - Before adding `useEffect` state sync, verify it cannot override explicit user UI navigation state.

#### Follow-Ups / Risks

- Remaining risk or pending check:
  - If new tree node types introduce `openPanel`, ensure only direct selection handlers apply that field.

### 2026-02-16 21:51 +11:00 - Multi-select edge flicker and panel-control placement

#### Task Summary

- Moved STEP Import `Reset Pose` control to the bottom of the panel.
- Fixed tree/weld synchronization conflict that could cause active segment flicker when two edges were selected.

#### Mistakes And Fixes

- Source: `[self]`
- Mistake:
  - Bidirectional sync lacked interaction-source gating, allowing tree and weld selection effects to fight each other.
- Detection:
  - User reported flickering behavior when two lines/segments were selected.
- Fix:
  - Added `panelSelectionOriginRef` and only applied tree->weld active-segment sync when selection originated from tree clicks.
- Preventive rule:
  - For bidirectional UI sync, always track source-of-truth per interaction to prevent feedback loops.

#### User Preferences

- New or reinforced preference:
  - Keep key panel actions (e.g. `Reset Pose`) at intuitive positions near related transform controls.
- How it changed execution:
  - Repositioned control directly in `StepImportPanel` footer.

#### What Worked

- Pattern/check that worked:
  - Interaction-origin refs are a lightweight, reliable way to stop cross-effect oscillation in React state sync.

#### What Did Not Work

- Failed attempt and why:
  - Pure dependency-based effects without origin markers were insufficient for multi-source selection flows.

#### Guardrails For Next Session

- Preflight rule:
  - When implementing two-way sync between panels/tree/scene, define and enforce a source tag before writing effects.

#### Follow-Ups / Risks

- Remaining risk or pending check:
  - If GPU-specific flicker remains, replace selected line rendering with a single authoritative overlay layer and suppress base-line rendering for selected edges.

### 2026-02-17 10:32 +11:00 - Enforce automatic scratchpad and devlog context

#### Task Summary

- Added a repo-level Cursor rule to make scratchpad/devlog workflow mandatory for all meaningful tasks.

#### Mistakes And Fixes

- Source: `[self]`
- Mistake:
  - Relying on optional workflow habits instead of enforcing them through an always-apply project rule.
- Detection:
  - User explicitly requested both loops be automatic in every agent session.
- Fix:
  - Added `.cursor/rules/agent-memory-loops.md` with start/during/end requirements for both files.
  - Updated `.cursor/rules/agent-gated-checklist.md` to require scratchpad/devlog read at Gate 0 and writeback at Gate 7.
- Preventive rule:
  - When the user asks for persistent agent behavior, encode it in `.cursor/rules` instead of relying on ad-hoc reminders.

#### User Preferences

- New or reinforced preference:
  - Always use and update both `AGENT_SCRATCHPAD.md` and `DEVLOG.md`.
- How it changed execution:
  - Implemented an always-apply rule and logged this change in both memory files immediately.

#### What Worked

- Pattern/check that worked:
  - Converting skill guidance into a concise always-apply rule provides durable enforcement across sessions.

#### What Did Not Work

- Failed attempt and why:
  - N/A for this task.

#### Guardrails For Next Session

- Preflight rule:
  - Before substantial edits, read `AGENT_SCRATCHPAD.md` + `DEVLOG.md`; before handoff, append both.

#### Follow-Ups / Risks

- Remaining risk or pending check:
  - Existing active sessions may need a fresh prompt/turn to fully align with newly added rule text.

### 2026-02-17 10:45 +11:00 - Explicit skill-to-file pointers for memory loops

#### Task Summary

- Added explicit references linking each memory file to its owning skill and template.

#### Mistakes And Fixes

- Source: `[self]`
- Mistake:
  - Previous rule text enforced the workflow but did not explicitly map the exact skill files and templates.
- Detection:
  - User asked to point to the specific skills and files.
- Fix:
  - Updated `.cursor/rules/agent-memory-loops.md` with required skill/template/file mapping.
  - Updated `QUICK_START.md` workflow pointers with direct skill-to-file paths.
- Preventive rule:
  - When documenting persistent behavior from skills, always include concrete source-skill paths and destination files.

#### User Preferences

- New or reinforced preference:
  - Keep explicit references to the exact skills and the files they manage.
- How it changed execution:
  - Added direct path mapping in both the always-apply rule and top-level quick-start docs.

#### What Worked

- Pattern/check that worked:
  - Short path mapping bullets remove ambiguity and make compliance auditable in one glance.

#### What Did Not Work

- Failed attempt and why:
  - N/A for this task.

#### Guardrails For Next Session

- Preflight rule:
  - If a process is skill-driven, verify docs include both `SKILL.md` path and target file path.

#### Follow-Ups / Risks

- Remaining risk or pending check:
  - None identified for this documentation update.

### 2026-02-17 00:12 +11:00 - Duplicate skill mapping across all always-on rules

#### Task Summary

- Added explicit scratchpad/devlog skill-to-file mapping blocks to all `alwaysApply: true` rule files.

#### Mistakes And Fixes

- Source: `[self]`
- Mistake:
  - Mapping existed in only part of the rule set, leaving room for inconsistent context anchoring.
- Detection:
  - User explicitly requested this be added to the rules (plural) so it is always in context.
- Fix:
  - Updated `.cursor/rules/agent-gated-checklist.md`, `.cursor/rules/agent-ambiguity-triggers.md`, `.cursor/rules/agent-subagents.md`, and `.cursor/rules/rtos-ethercat-readme.md` with the same required mapping block.
- Preventive rule:
  - For mandatory context anchors, mirror the same source-of-truth mapping across every `alwaysApply` rule file.

#### User Preferences

- New or reinforced preference:
  - Keep scratchpad/devlog skill links explicitly present across the entire always-on rule surface.
- How it changed execution:
  - Applied a repeated mapping section to each always-apply rule, not just memory-focused docs.

#### What Worked

- Pattern/check that worked:
  - Uniform, copy-identical mapping sections reduce ambiguity and audit time.

#### What Did Not Work

- Failed attempt and why:
  - N/A for this task.

#### Guardrails For Next Session

- Preflight rule:
  - When user says "always in context," verify all `alwaysApply` rules carry the same mandatory pointers.

#### Follow-Ups / Risks

- Remaining risk or pending check:
  - If new `alwaysApply` rules are added later, they must include the same mapping block.

### 2026-02-17 00:41 +11:00 - Weld Motion + Tree UX delivery and checklist compliance fix

#### Task Summary

- Delivered full requested pass:
  - compact + chronological Program Tree UX
  - weld section planning with transitions
  - torch-angle controls (UI -> API -> planner)
  - planner robustness and diagnostics.
- Closed workflow loop by writing explicit session entries to both `DEVLOG.md` and `AGENT_SCRATCHPAD.md`.

#### Mistakes And Fixes

- Source: `[self]`
- Mistake:
  - Completed feature implementation but initially missed the two trailing checklist items (memory writeback + future backlog todo creation).
- Detection:
  - User called it out directly ("last 2 on the list ... didn't touch").
- Fix:
  - Immediately added memory-loop writeback entry to both files and created explicit future backlog tracking todo.
- Preventive rule:
  - Before handoff, verify every visible checklist/todo item (including process tasks) is handled, not just code tasks.

#### User Preferences

- New or reinforced preference:
  - Process tasks are first-class requirements; do not skip memory/devlog updates when explicitly listed.
  - Strong preference for direct execution over explanation-only updates.
- How it changed execution:
  - Added explicit final pass for process compliance and backlog traceability in the same turn.

#### What Worked

- Pattern/check that worked:
  - Section-based weld planning model (`weld` vs `transition`) made it practical to implement contiguous weld continuation and safe-lift transitions without a full collision engine.
  - Runtime fallback from torch-angle orientation solve to orientation-lock avoided planner hard-fails.

#### What Did Not Work

- Failed attempt and why:
  - Strict torch-angle orientation path can be IK-infeasible on some geometries; required fallback behavior to keep planning usable.

#### Guardrails For Next Session

- Preflight rule:
  - Track implementation to-dos and workflow to-dos separately, and do a final checklist sweep that includes both.

#### Follow-Ups / Risks

- Remaining risk or pending check:
  - Full collision-aware transition planner is still pending and should replace heuristic safe-lift logic in a future phase.

### 2026-02-17 00:47 +11:00 - Viewport-clamped sidebar drawer

#### Task Summary

- Fixed menu overflow issue where left drawer panels could exceed the viewport height.

#### Mistakes And Fixes

- Source: `[self]`
- Mistake:
  - Drawer container allowed unconstrained vertical growth when panel content (especially Weld panel with long waypoint lists) got tall.
- Detection:
  - User screenshot and explicit feedback: "can't let menus grow larger than window size."
- Fix:
  - Added viewport max-height and internal scroll behavior in `web-ui/src/components/SidebarDrawer.tsx`.
- Preventive rule:
  - Any absolute overlay panel should define a viewport max-height and internal scrolling before adding content-heavy sections.

#### User Preferences

- New or reinforced preference:
  - Keep side menus fully contained within the visible window.
- How it changed execution:
  - Prioritized layout containment fix over feature additions.

#### What Worked

- Pattern/check that worked:
  - Applying max-height at the shared drawer wrapper fixed all drawer-hosted panels at once.

#### What Did Not Work

- Failed attempt and why:
  - N/A for this task.

#### Guardrails For Next Session

- Preflight rule:
  - For UI overlays, validate worst-case content height against viewport before handoff.

#### Follow-Ups / Risks

- Remaining risk or pending check:
  - If non-drawer floating panels are expanded in future, they may need the same containment pattern.

### 2026-02-17 00:50 +11:00 - Drawer header overlap guard band

#### Task Summary

- Fixed a visual overlap where the drawer close button covered panel header controls on the right side.

#### Mistakes And Fixes

- Source: `[self]`
- Mistake:
  - The close button was absolutely positioned over content with insufficient reserved horizontal space.
- Detection:
  - User reported overlap and shared screenshot showing Weld badge collision near the close icon.
- Fix:
  - Added a right-side content guard band in `web-ui/src/components/SidebarDrawer.tsx` by increasing inner wrapper padding to `pr-10`.
- Preventive rule:
  - Any persistent overlay control (close/help/action) must reserve explicit layout space rather than relying on visual luck.

#### User Preferences

- New or reinforced preference:
  - UI controls must never overlap; title/header actions must remain readable and clickable.
- How it changed execution:
  - Prioritized spacing/layout correction over adding new interactions.

#### What Worked

- Pattern/check that worked:
  - Shared-container spacing fixes in one wrapper corrected multiple panel variants without touching feature-specific components.

#### What Did Not Work

- Failed attempt and why:
  - N/A for this task.

#### Guardrails For Next Session

- Preflight rule:
  - For absolute-positioned controls, verify both vertical and horizontal guard space at smallest supported drawer width.

#### Follow-Ups / Risks

- Remaining risk or pending check:
  - If header content density increases (more badges/buttons), migrate to an explicit shared drawer header row to keep spacing deterministic.

### 2026-02-17 00:53 +11:00 - Escalation handoff note for next model

#### Task Summary

- Added a high-priority takeover TODO in `QUICK_START.md` so a new model can continue unresolved UI overlap cleanup immediately.

#### Mistakes And Fixes

- Source: `[self]`
- Mistake:
  - Prior spacing fix did not meet user quality expectations.
- Detection:
  - Direct user feedback: overlap still unacceptable.
- Fix:
  - Wrote explicit handoff requirements + acceptance criteria at the top of `QUICK_START.md` to avoid context loss across model handoff.
- Preventive rule:
  - When user asks for takeover, document exact failure mode + required end-state in a top-level onboarding doc.

#### User Preferences

- New or reinforced preference:
  - Do not paper over visual defects; require robust layout fixes.
- How it changed execution:
  - Prioritized cross-model continuity and clear ownership transfer instructions.

#### What Worked

- Pattern/check that worked:
  - A concrete handoff checklist in `QUICK_START.md` gives immediate actionability for the next model.

#### What Did Not Work

- Failed attempt and why:
  - Padding-only overlap mitigation was not perceived as a complete fix.

#### Guardrails For Next Session

- Preflight rule:
  - For overlay/header defects, prefer structural layout changes (shared header row) over spacing-only adjustments.

#### Follow-Ups / Risks

- Remaining risk or pending check:
  - The actual UI fix is still pending; this entry only captures handoff context.

### 2026-02-17 19:27 +11:00 - Shared drawer header row implementation

#### Task Summary

- Implemented structural drawer-header fix from `QUICK_START.md` to prevent overlap between header content and close control.
- Moved weld title/badge into shared drawer header surface and removed duplicate in-panel title rows.

#### Mistakes And Fixes

- Source: `[self]`
- Mistake:
  - Earlier workaround depended on right-side padding (`pr-10`) while keeping close action absolutely positioned.
- Detection:
  - User screenshot + takeover note confirmed overlap remained unacceptable in real weld-header content.
- Fix:
  - Replaced overlay close action with a dedicated `SidebarDrawer` header row (`headerContent` + close button) so layout guarantees non-overlap.
- Preventive rule:
  - For any dismiss/action control near dynamic header content, use structural row layout with flex constraints (`min-w-0`, `shrink-0`) instead of padding buffers.

#### User Preferences

- New or reinforced preference:
  - UI fixes should be robust by structure, not spacing hacks.
  - "Implement, do not only explain" remains the default execution style.
- How it changed execution:
  - Applied direct component refactor and validation in the same turn instead of proposing-only guidance.

#### What Worked

- Pattern/check that worked:
  - Centralizing header composition in `SidebarDrawer` allowed one fix to cover all panel types while keeping panel body logic unchanged.
  - Immediate `npm run build` + `ReadLints` checks caught regressions quickly.

#### What Did Not Work

- Failed attempt and why:
  - Keeping titles in both drawer header and panel cards created duplicated heading surfaces; removed duplicated panel titles where appropriate.

#### Guardrails For Next Session

- Preflight rule:
  - If a shared container now owns a title area, remove duplicate in-panel titles unless they carry unique controls.

#### Follow-Ups / Risks

- Remaining risk or pending check:
  - Perform visual verification at narrow window widths to confirm spacing and interaction feel for all drawer panels in live UI.

### 2026-02-17 20:34 +11:00 - Drawer bottom inset alignment + AGENTS skill catalog refresh

#### Task Summary

- Corrected left drawer vertical sizing so it keeps a bottom inset instead of visually running to the edge.
- Updated `AGENTS.md` to reflect the rename from `QUICK_START.md` and documented all installed skills with usage triggers.

#### Mistakes And Fixes

- Source: `[self]`
- Mistake:
  - Previous drawer height model used viewport-based max-height math, which could feel mismatched with sibling overlays in a header+main layout.
- Detection:
  - User screenshot highlighted asymmetry between the left drawer and right robot-control panel bottom spacing.
- Fix:
  - Refactored drawer container to inset-based vertical layout (`inset-y-6`) with `flex` + `min-h-0`; retained scroll using `flex-1 overflow-y-auto`.
- Preventive rule:
  - For overlay alignment across a shared surface, prefer consistent positional insets (`top/bottom`) over independent max-height calculations.

#### User Preferences

- New or reinforced preference:
  - Visually related overlays should have matching baseline/inset behavior.
  - Agent docs must stay current when top-level onboarding files are renamed.
  - Design-oriented skill usage should be explicit and discoverable.
- How it changed execution:
  - Applied layout fix first, then codified full skill relevance in `AGENTS.md`.

#### What Worked

- Pattern/check that worked:
  - `inset-y-*` + `flex-1` scroll gives deterministic alignment while preserving long-content usability.
  - Using `frontend-design` guidance for implementation direction and `web-design-guidelines` guidance for post-change review framing kept UI decisions intentional.

#### What Did Not Work

- Failed attempt and why:
  - Treating drawer max-height independent of main container created perceived edge contact even when scroll technically worked.

#### Guardrails For Next Session

- Preflight rule:
  - If two overlay panels are expected to align, compare both vertical anchors (`top`, `bottom`, internal scroll shell) before finalizing styles.

#### Follow-Ups / Risks

- Remaining risk or pending check:
  - Confirm final visual balance during live interaction at very small window heights and high content density.

### 2026-02-17 20:41 +11:00 - Themed drawer scrollbar styling

#### Task Summary

- Replaced default browser-style drawer scrollbar with a custom theme-matched scrollbar.
- Kept behavior cross-browser by styling both Firefox and WebKit engines.

#### Mistakes And Fixes

- Source: `[self]`
- Mistake:
  - Previously left the primary drawer scroller unstyled, which looked inconsistent with the polished panel visual design.
- Detection:
  - User feedback with screenshot: "scrollbar is ugly" and requested UI-consistent styling.
- Fix:
  - Added reusable `.gradient-scrollbar` utility in `web-ui/src/index.css` and applied it to the drawer scroll shell in `SidebarDrawer.tsx`.
- Preventive rule:
  - Any prominent always-visible scrollbar in core UI panels should receive explicit theme styling and not rely on OS defaults.

#### User Preferences

- New or reinforced preference:
  - Styling details (including scrollbars) must match the overall interface quality bar.
- How it changed execution:
  - Prioritized direct visual polish in production code with immediate build/lint validation.

#### What Worked

- Pattern/check that worked:
  - Utility-class approach (`gradient-scrollbar`) makes it easy to reuse consistent scrollbar styling across other scrollable panel sections.
  - Combining Firefox and WebKit declarations ensures broad browser coverage.

#### What Did Not Work

- Failed attempt and why:
  - N/A for this change.

#### Guardrails For Next Session

- Preflight rule:
  - For UI polish requests, inspect for native browser defaults (scrollbars, focus rings, select arrows) and theme them where they are visually dominant.

#### Follow-Ups / Risks

- Remaining risk or pending check:
  - If users request thicker or subtler scrollbar contrast, tune width/color alpha in `.gradient-scrollbar` rather than duplicating new classes.

### 2026-02-17 20:49 +11:00 - Scrollbar integrated into rounded drawer shell

#### Task Summary

- Integrated header and scroll body into a single drawer shell so the scrollbar appears inside the panel.
- Ensured rounded bottom corners remain visible regardless of scroll position.

#### Mistakes And Fixes

- Source: `[self]`
- Mistake:
  - Scroll region still sat outside the main framed shell, making the scrollbar appear detached and corners feel inconsistent.
- Detection:
  - User screenshot highlighted scrollbar placement and requested persistent rounded bottom corners while scrolling.
- Fix:
  - Reworked `SidebarDrawer` to a single `rounded-xl overflow-hidden` container with internal header and body scroller.
- Preventive rule:
  - If users ask for persistent corner shape during scrolling, clipping must happen at the outermost rounded container.

#### User Preferences

- New or reinforced preference:
  - Scrollbar should feel like part of the panel, not adjacent to it.
  - Rounded geometry should remain stable at all scroll offsets.
- How it changed execution:
  - Prioritized container hierarchy/layout over color-only styling tweaks.

#### What Worked

- Pattern/check that worked:
  - One-shell layout with `border-b` header divider gives cleaner structure and deterministic corner clipping.

#### What Did Not Work

- Failed attempt and why:
  - Styling the scrollbar alone without container clipping did not fully solve the visual integration request.

#### Guardrails For Next Session

- Preflight rule:
  - For any scrollable card/panel, confirm the scroll container is nested inside the same rounded element that defines the visual frame.

#### Follow-Ups / Risks

- Remaining risk or pending check:
  - Optional future polish: reduce nested card framing inside drawer bodies if a flatter visual style is desired.

### 2026-02-17 21:28 +11:00 - Weld typography consistency normalization

#### Task Summary

- Applied a consistent font-size system to the Weld panel (labels, metadata, section headings, inputs, and action text).
- Kept CTA emphasis while reducing random micro-size jumps in the rest of the panel.

#### Mistakes And Fixes

- Source: `[self]`
- Mistake:
  - Weld UI accumulated mixed ad-hoc text sizes (`text-xs`, `text-[11px]`, `text-[10px]`) without shared typography tokens.
- Detection:
  - User requested consistent styling/sizing and screenshot showed uneven typography rhythm.
- Fix:
  - Added shared weld typography class constants in `App.tsx` and refactored key Weld panel elements to use them.
- Preventive rule:
  - For dense forms, define reusable typographic tokens first, then apply them consistently instead of per-control one-off sizing.

#### User Preferences

- New or reinforced preference:
  - Typography should feel intentionally consistent, not piecemeal.
- How it changed execution:
  - Prioritized text hierarchy cleanup (label/meta/control consistency) immediately after structural layout fixes.

#### What Worked

- Pattern/check that worked:
  - Local constants for panel typography made broad consistency changes safer and easier to review.

#### What Did Not Work

- Failed attempt and why:
  - N/A for this update.

#### Guardrails For Next Session

- Preflight rule:
  - When touching any large panel, run a quick typography pass to ensure no unnecessary size variants remain.

#### Follow-Ups / Risks

- Remaining risk or pending check:
  - STEP and Trajectory panels may still contain independent typography choices and can be normalized in a dedicated follow-up.

### 2026-02-17 21:31 +11:00 - Text hierarchy correction for section title vs field label

#### Task Summary

- Adjusted typography hierarchy so `Weld Program` (section title) and `Program Name` (field label) are visually distinct.

#### Mistakes And Fixes

- Source: `[self]`
- Mistake:
  - Initial typography normalization still left section title and label weights too close, reading as both bold in practice.
- Detection:
  - User feedback called out both lines appearing bold despite hierarchy intent.
- Fix:
  - Set `WELD_LABEL_CLASS` to normal weight and strengthened `WELD_SECTION_TITLE_CLASS` size/contrast for clearer hierarchy.
- Preventive rule:
  - After typographic refactors, verify key adjacent text pairs (section title vs label) in rendered UI, not just by class names.

#### User Preferences

- New or reinforced preference:
  - Visual hierarchy should be obvious; labels should not compete with section headings.
- How it changed execution:
  - Applied immediate token-level correction instead of adding more one-off local class overrides.

#### What Worked

- Pattern/check that worked:
  - Centralized typography constants enabled a quick, low-risk hierarchy adjustment.

#### What Did Not Work

- Failed attempt and why:
  - Equalized sizing pass alone did not guarantee perceived hierarchy when both styles still had elevated weight.

#### Guardrails For Next Session

- Preflight rule:
  - For dense forms, reserve stronger weight/color for section titles and keep field labels at regular weight unless emphasis is intentional.

#### Follow-Ups / Risks

- Remaining risk or pending check:
  - Consider applying the same heading-vs-label hierarchy tokens to STEP and Trajectory drawers for full consistency.

### 2026-02-17 21:34 +11:00 - Cross-panel typography alignment + living design doc

#### Task Summary

- Extended typography consistency work from Weld to STEP and Trajectory panels.
- Added `web-ui/design.md` as the living design-system document and referenced it from `AGENTS.md`.

#### Mistakes And Fixes

- Source: `[self]`
- Mistake:
  - Typography tokenization was initially panel-local (`WELD_*`) and not clearly positioned as a shared drawer system.
- Detection:
  - User approved extending hierarchy consistency across all drawer tabs and requested a persistent living design doc.
- Fix:
  - Introduced shared `DRAWER_*` tokens in `App.tsx` and aligned STEP/Trajectory class usage with those tokens.
  - Created `web-ui/design.md` with rules/checklist and linked it from `AGENTS.md`.
- Preventive rule:
  - When UI consistency request spans multiple panels, establish or update a repo-local design source-of-truth before further styling changes.

#### User Preferences

- New or reinforced preference:
  - Consistency should be systematic and documented, not just fixed one screen at a time.
- How it changed execution:
  - Combined implementation changes with living documentation in the same turn.

#### What Worked

- Pattern/check that worked:
  - Shared token strategy (`DRAWER_*`) allowed quick normalization without major component rewrites.
  - A living doc with checklist creates durable guardrails for future UI edits.

#### What Did Not Work

- Failed attempt and why:
  - N/A for this update.

#### Guardrails For Next Session

- Preflight rule:
  - Before editing drawer panel styles, read `web-ui/design.md` and use existing `DRAWER_*` tokens unless intentionally evolving the design system.

#### Follow-Ups / Risks

- Remaining risk or pending check:
  - Global typography outside drawer panels still may not fully match the new panel system and can be standardized later.

### 2026-02-17 21:43 +11:00 - Hard requirement language for memory-loop completion

#### Task Summary

- Strengthened `AGENTS.md` so updating both `DEVLOG.md` and `AGENT_SCRATCHPAD.md` is explicitly non-optional.

#### Mistakes And Fixes

- Source: `[user]`
- Mistake:
  - Existing wording listed both files but was not strong enough to prevent potential omission.
- Detection:
  - User explicitly requested stronger emphasis that these tasks must never be left undone.
- Fix:
  - Added MUST language on both workflow bullets and a non-negotiable blocker rule in `AGENTS.md`.
- Preventive rule:
  - If user says "every time", encode it with explicit "MUST" + "blocker" phrasing in the top-level onboarding doc.

#### User Preferences

- New or reinforced preference:
  - Memory-loop updates are mandatory on every meaningful task with zero exceptions.
- How it changed execution:
  - Immediately hardened policy text in `AGENTS.md` and logged the change in both memory files.

#### What Worked

- Pattern/check that worked:
  - Converting soft guidance into explicit completion criteria reduces ambiguity and missed process steps.

#### What Did Not Work

- Failed attempt and why:
  - Soft descriptive wording ("maintain these files") did not clearly communicate non-negotiable enforcement.

#### Guardrails For Next Session

- Preflight rule:
  - Treat absent updates in either `DEVLOG.md` or `AGENT_SCRATCHPAD.md` as a stop condition before final handoff.

#### Follow-Ups / Risks

- Remaining risk or pending check:
  - None for this doc-policy reinforcement; now explicitly codified.

### 2026-02-17 21:46 +11:00 - Remove nested drawer shell for more usable width

#### Task Summary

- Removed the extra inner full-card shell from drawer panel content to eliminate the double-layer frame.
- Increased usable content room in the drawer without changing the outer shell behavior.

#### Mistakes And Fixes

- Source: `[user]`
- Mistake:
  - Panel content still had a nested full-shell wrapper (rounded/border/bg/shadow) inside the drawer shell, causing visual duplication.
- Detection:
  - User screenshot highlighted unnecessary double layer and requested more content room.
- Fix:
  - Removed root shell classes from drawer panel roots in `App.tsx` and reduced shelling in `TelemetryCharts.tsx`.
  - Added a permanent "no nested outer shell" rule to `web-ui/design.md`.
- Preventive rule:
  - In drawer UIs, keep one primary shell only; use section cards for grouping, not another full wrapper.

#### User Preferences

- New or reinforced preference:
  - Avoid double framing; prioritize cleaner visual hierarchy and usable space.
- How it changed execution:
  - Applied structural class removal instead of spacing-only patching.

#### What Worked

- Pattern/check that worked:
  - Removing duplicated shell classes immediately reduced visual noise and reclaimed width.

#### What Did Not Work

- Failed attempt and why:
  - Prior refinements (scrollbar, typography) improved polish but did not remove the underlying duplicated-shell structure.

#### Guardrails For Next Session

- Preflight rule:
  - Before finalizing drawer visuals, verify only one full-shell container exists in the panel stack.

#### Follow-Ups / Risks

- Remaining risk or pending check:
  - Fine-tune section card spacing if certain panel states appear too sparse after shell removal.

### 2026-02-17 21:50 +11:00 - Adaptive drawer height + wider telemetry panel

#### Task Summary

- Changed drawer sizing behavior so short-content panels no longer stretch to full-height.
- Added a wider drawer width variant for telemetry/charts to avoid horizontal overflow.

#### Mistakes And Fixes

- Source: `[user]`
- Mistake:
  - Forcing drawer to `inset-y` full height made sparse panels (STEP/Trajectory/Telemetry idle) look mostly empty.
- Detection:
  - User screenshots showed excessive empty vertical space and horizontal scrollbar in charts panel.
- Fix:
  - Switched drawer to content-driven height with viewport max-height cap and internal scroll.
  - Added panel-specific width prop and set telemetry to wider width.
  - Added `overflow-x-hidden` in drawer body to suppress unintended sideways scroll.
- Preventive rule:
  - Drawer height should be content-first with max-height constraints; reserve full-height overlays only for intentionally immersive panels.

#### User Preferences

- New or reinforced preference:
  - Keep max-height safety, but avoid unnecessary empty space in light-content tabs.
  - Charts panel should prioritize readable layout over strict shared-width parity.
- How it changed execution:
  - Implemented adaptive layout plus targeted width override rather than a single global sizing rule.

#### What Worked

- Pattern/check that worked:
  - Width variant via prop (`widthClassName`) cleanly supports per-panel layout needs without duplicating drawer component logic.

#### What Did Not Work

- Failed attempt and why:
  - Earlier one-size full-height behavior suited long Weld content but degraded sparse tabs.

#### Guardrails For Next Session

- Preflight rule:
  - Validate each tab in both sparse and dense states before finalizing shared container sizing.

#### Follow-Ups / Risks

- Remaining risk or pending check:
  - If telemetry charts add more columns/cards, add responsive width tiers rather than reintroducing horizontal scroll.

### 2026-02-17 22:10 +11:00 - Weld drawer baseline + tooltip clipping regression fix

#### Task Summary

- Fixed Weld drawer vertical sizing so its bottom baseline stays aligned with Robot Control.
- Fixed angle-help tooltip clipping by moving it out of the scroll container into a fixed portal overlay.
- Codified these constraints in `web-ui/design.md`.

#### Mistakes And Fixes

- Source: `[user]`
- Mistake:
  - Drawer sizing logic shifted between content-driven and max-height variants, causing bottom misalignment and visible clipping near footer-adjacent controls.
  - Tooltip was rendered inside an overflowed panel region, so it got clipped/cut off.
- Detection:
  - User screenshots clearly showed the panel extending into terminal/footer region and tooltip content cut off by panel bounds.
- Fix:
  - Anchored drawer with explicit `top-6` + `bottom-6` and `h-full` shell.
  - Rendered angle explainer tooltip via `createPortal(document.body)` with fixed positioning and viewport clamping.
  - Set tooltip to open on the right by default with left fallback only when viewport space is constrained.
- Preventive rule:
  - Never place explainer popovers inside scrolling/clipped containers; use portal overlays for any panel-help UI.
  - For consistency-critical overlays, align by shared anchor insets rather than mixing content-height and max-height modes.

#### User Preferences

- New or reinforced preference:
  - Strong preference for consistent panel baselines and no clipped UI.
  - When regressions are reported with screenshots, prioritize direct fixes over exploratory redesign.
- How it changed execution:
  - Moved from incremental class tweaks to hard layout anchoring + portalized overlay behavior.

#### What Worked

- Pattern/check that worked:
  - Using `absolute top-6 bottom-6` + internal scroll gives stable, predictable panel bounds across dense Weld content.
  - Portal + fixed positioning immediately removed tooltip clipping from drawer overflow constraints.

#### What Did Not Work

- Failed attempt and why:
  - Intermediate max-height-only tuning was not robust; it still produced inconsistent bottoms in real viewport states.

#### Guardrails For Next Session

- Preflight rule:
  - For all floating panels, verify top and bottom anchors against adjacent UI baselines before finalizing.
  - For tooltips/popovers inside drawers, require portal rendering and viewport-bound checks by default.

#### Follow-Ups / Risks

- Remaining risk or pending check:
  - If additional help popovers are added, they must reuse the same portal + clamp pattern to avoid repeat clipping regressions.

### 2026-02-17 22:24 +11:00 - Weld end-action semantics correction

#### Task Summary

- Corrected weld post-action behavior so `return_to_start` means return to trajectory start/home-start pose (not weld start).
- Added new post-action mode `lift` for a small vertical retract from weld end.
- Synced backend planner semantics, API normalization, and UI enum/options.

#### Mistakes And Fixes

- Source: `[user]`
- Mistake:
  - Existing `return_to_start` behavior returned to the first weld point, which is semantically wrong for full-trajectory flow.
- Detection:
  - User provided explicit behavior definition + annotated image showing desired home-start target.
- Fix:
  - Removed frontend section-level weld-start return insertion.
  - Implemented backend post-action planning in `command_api.py`:
    - `return_to_start` now routes from weld end back to trajectory start pose (captured at planning start), using a lifted transition.
    - `lift` now performs a vertical retract by transition clearance.
  - Added `lift` normalization in `main.py` and UI type/select handling in `App.tsx`.
- Preventive rule:
  - End-action semantics must be owned by backend planner state (which has true start pose), not pre-baked by frontend geometry assumptions.

#### User Preferences

- New or reinforced preference:
  - "Return to start" must always refer to trajectory/program start, not local weld segment start.
  - Add practical post-weld finishing action(s) like lift for safer motion behavior.
- How it changed execution:
  - Prioritized behavior semantics over UI-only labeling and implemented planner-level logic.

#### What Worked

- Pattern/check that worked:
  - Centralizing end-action logic in backend keeps preview/execution behavior consistent and source-of-truth aligned.

#### What Did Not Work

- Failed attempt and why:
  - Previous frontend-only return transition generation could not represent trajectory start correctly because it lacked planner start-pose context.

#### Guardrails For Next Session

- Preflight rule:
  - For any motion semantic label (`return`, `home`, `safe`), verify mapping against planner/control definitions before shipping UI text.

#### Follow-Ups / Risks

- Remaining risk or pending check:
  - If a dedicated configurable "home" waypoint is introduced later, `return_to_start` should explicitly choose between recorded trajectory start vs configured home target.

### 2026-02-17 22:57 +11:00 - Weld-program load must clear stale preview state

#### Task Summary

- Fixed stale path rendering when loading saved weld programs that do not include a planned trajectory payload.

#### Mistakes And Fixes

- Source: `[user]`
- Mistake:
  - Loading `test_0` could leave the previous preview path visible because restore logic only set new preview when present, but did not clear old preview when missing.
- Detection:
  - User reported loaded program retained prior plan/path visuals.
- Fix:
  - In weld-program restore path, explicitly clear `previewPlan` + `plannerPoints` when `pendingWeldProgramRestore.previewPlan` is null.
  - Also clear `previewPlan` + `plannerPoints` immediately after successful program payload validation so stale geometry is removed during restore.
- Preventive rule:
  - Any optional payload restore must include explicit "else clear" handling for stateful visuals.

#### User Preferences

- New or reinforced preference:
  - Loading a saved program must never retain stale path overlays from previous sessions/plans.
- How it changed execution:
  - Prioritized deterministic state reset behavior over preserving transient UI visuals between loads.

#### What Worked

- Pattern/check that worked:
  - Clearing both source states (`previewPlan` and `plannerPoints`) ensures visual path fallback logic cannot display old geometry.

#### What Did Not Work

- Failed attempt and why:
  - Implicit state replacement only on "truthy new plan" left stale values alive in null-plan restore cases.

#### Guardrails For Next Session

- Preflight rule:
  - For every restore/load flow, enumerate each visual state and handle both "present" and "absent" payload branches explicitly.

#### Follow-Ups / Risks

- Remaining risk or pending check:
  - If additional derived visual states are added (e.g., cached highlight ranges), ensure they are reset alongside preview state on load.

### 2026-02-17 23:29 +11:00 - Weld-run visual flicker spike filtering

#### Task Summary

- Added runtime telemetry filtering to prevent single-frame snap-back/flicker artifacts in arm visualization during weld execution.

#### Mistakes And Fixes

- Source: `[user]`
- Mistake:
  - Visualization occasionally jumped to a stale weld-start-like pose for one frame while actual motion continued, creating star-like flicker trails.
- Detection:
  - User screenshot showed repeated visual spokes from a fixed point during weld run.
- Fix:
  - Added telemetry guards in `web-ui/src/App.tsx`:
    - drop out-of-order packets using source telemetry timestamp (`t`),
    - reject implausible one-frame joint spikes (`maxJump > 0.8 rad` within `<=0.25s`) likely caused by stale/outlier packets.
  - Reset telemetry filter refs on disconnect.
- Preventive rule:
  - Treat UI pose stream as potentially noisy/reordered; enforce monotonic timestamp acceptance and outlier rejection before rendering.

#### User Preferences

- New or reinforced preference:
  - Weld execution visualization must remain stable and trustworthy; no transient “teleport” artifacts.
- How it changed execution:
  - Added ingestion-layer robustness rather than only tuning rendering interpolation.

#### What Worked

- Pattern/check that worked:
  - Filtering at message-ingest stage avoids contaminating both immediate and smoothed pose updates.

#### What Did Not Work

- Failed attempt and why:
  - Relying on smoothing alone cannot prevent stale packet flashes because stale targets still get applied instantly.

#### Guardrails For Next Session

- Preflight rule:
  - For realtime robot UI streams, always define packet-order and spike-handling policy explicitly.

#### Follow-Ups / Risks

- Remaining risk or pending check:
  - If a second legitimate telemetry source is intentionally mixed in future, add explicit source tagging/selection instead of relying on timestamp-only arbitration.

### 2026-02-18 00:08 +11:00 - Weld program run gating + start-from-current execution

#### Task Summary

- Fixed inability to run loaded weld programs when draft restoration is missing/invalid but a runnable preview trajectory exists.
- Enforced run-time re-planning from current robot state for weld preview execution.

#### Mistakes And Fixes

- Source: `[user]`
- Mistake:
  - Run button in Weld panel was gated on `draft` existence instead of actual runnable preview presence, blocking execution for some loaded programs.
  - Weld preview run path used cached planning (`use_cache: true`), which can execute stale joint paths not guaranteed to reflect current robot state.
- Detection:
  - User loaded `test_02` and observed run action unavailable despite loaded trajectory/waypoints.
- Fix:
  - Updated Weld panel run gating to use `canRunPreview` (`Boolean(previewPlan?.name)`) rather than `draft`.
  - Switched preview run request to `use_cache: false` so backend re-plans from current state, naturally including current->start motion.
- Preventive rule:
  - UI action enablement must track actual execution prerequisites (runnable plan), not adjacent editor state (draft availability).

#### User Preferences

- New or reinforced preference:
  - Loaded weld programs should be runnable even when edge-edit context is unavailable.
  - Execution should start from current robot pose with an explicit approach to program start.
- How it changed execution:
  - Prioritized run-time correctness and operability over cache-first speed.

#### What Worked

- Pattern/check that worked:
  - Decoupling run enablement from `draft` immediately restores operability for loaded plans.
  - Re-plan from current state guarantees start approach behavior without additional special-case injection.

#### What Did Not Work

- Failed attempt and why:
  - Previous cache-first preview execution assumed planning-time and run-time robot state equivalence.

#### Guardrails For Next Session

- Preflight rule:
  - For any "Run" control, verify its disabled condition maps exactly to runtime required data, then confirm loaded-from-file flows satisfy that condition.

#### Follow-Ups / Risks

- Remaining risk or pending check:
  - If run latency becomes noticeable due to re-planning, introduce an explicit "replan-on-run" toggle with clear UX semantics.

### 2026-02-18 00:18 +11:00 - Panel-aware drawer height mode (keep weld full, un-stretch others)

#### Task Summary

- Fixed the drawer height regression where STEP / Trajectory / Telemetry looked stretched to the bottom with large empty space.
- Preserved Weld Planning as full-height because that dense workflow benefits from a stable full overlay band.

#### Mistakes And Fixes

- Source: `[user]`
- Mistake:
  - A global full-height drawer shell (`h-full` inside `top-6 bottom-6`) was applied to all tabs, which regressed sparse panels into visibly stretched empty containers.
- Detection:
  - User screenshots showed STEP / Trajectory / Live Charts extending to the bottom while Weld looked acceptable.
- Fix:
  - Added `heightMode` to `SidebarDrawer`:
    - `full` for Weld (`h-full`),
    - `content` for STEP / Trajectory / Telemetry (`max-h-full` with internal scrolling preserved).
  - Kept common overlay lane (`top-6 bottom-6`) and moved pointer-event handling to panel shell so empty transparent lane area does not block workspace interaction.
  - Updated `web-ui/design.md` rules to codify mixed-mode behavior.
- Preventive rule:
  - Do not apply one global drawer height strategy across panels with different content density; explicitly model panel height intent (content-fit vs full-height).

#### User Preferences

- New or reinforced preference:
  - Weld panel baseline/behavior is acceptable and should remain unchanged when fixing other tabs.
  - Sparse panels should not appear stretched to the viewport bottom.
- How it changed execution:
  - Used panel-specific height mode instead of another global class toggle.

#### What Worked

- Pattern/check that worked:
  - Shared wrapper + per-panel shell height mode is a low-risk way to preserve weld behavior while fixing sparse tabs.
  - Keeping internal scroll inside the same shell retained dense-content safety without reintroducing clipping.

#### What Did Not Work

- Failed attempt and why:
  - Previous "all panels full-height" rule solved weld alignment but caused immediate UX regressions for sparse tabs.

#### Guardrails For Next Session

- Preflight rule:
  - For shared drawer/container refactors, validate all tabs in both sparse and dense states before finalizing.
  - If one panel is intentionally different, encode that in props rather than ad-hoc class forks.

#### Follow-Ups / Risks

- Remaining risk or pending check:
  - Confirm in live UI that click-through around non-full-height drawer shell feels natural at narrow and wide viewport sizes.

### 2026-02-18 01:13 +11:00 - Local repo skill installation into Codex home

#### Task Summary

- Installed all local skills from `.cursor/skills` into `C:\Users\angus\.codex\skills`.
- Verified destination skill set matches source local skills and complies with AGENTS workflow logging requirements.

#### Mistakes And Fixes

- Source: `[self]`
- Mistake:
  - Initial assumption from AGENTS list suggested `.cursor/skills-cursor` might also require installation.
- Detection:
  - Repository inspection showed `.cursor` contains only `rules` and `skills`; no `.cursor/skills-cursor` directory exists in this workspace.
- Fix:
  - Scoped installation to `.cursor/skills` folders that contain `SKILL.md`, then audited source-vs-destination skill names.
- Preventive rule:
  - Before bulk install/sync operations, verify referenced directories exist in the current repo snapshot rather than relying only on docs.

#### User Preferences

- New or reinforced preference:
  - Use `AGENTS.md` as startup context and install all repo-local skills when requested.
- How it changed execution:
  - Followed skill-installer guidance for workflow framing, then performed local copy/install for all `.cursor/skills` folders.

#### What Worked

- Pattern/check that worked:
  - Filtering source directories by existence of `SKILL.md` prevents copying non-skill folders.
  - Compare-object audit after install quickly confirms there are no missing skill names.

#### What Did Not Work

- Failed attempt and why:
  - None in this task; install path and audit succeeded on first pass.

#### Guardrails For Next Session

- Preflight rule:
  - For skill installation requests, check both `.cursor/skills` and any AGENTS-referenced paths, but install only paths present in the active workspace.
  - Always finish by updating both `DEVLOG.md` and `AGENT_SCRATCHPAD.md` before handoff.

#### Follow-Ups / Risks

- Remaining risk or pending check:
  - Codex usually loads skills at startup; restart Codex after installation to ensure all newly installed skills are available.

### 2026-02-18 01:17 +11:00 - Weld preview run should use high-fidelity cache, not sparse endpoint re-plan

#### Task Summary

- Fixed mismatch where weld preview execution diverged from previewed/interpolated path because run used endpoint re-planning.
- Added explicit cache-readiness handling for weld runs and clarified UI wording around editable weld points.

#### Mistakes And Fixes

- Source: `[user]`
- Mistake:
  - Run path used `/trajectory/run` with `use_cache: false` globally, so weld trajectories were rebuilt from sparse `trajectory.moves` endpoints (few `move_absolute` nodes) instead of using the high-fidelity cached weld path.
- Detection:
  - User screenshot + report showed large gap between expected weld curve and simulated run path, while Program Tree showed only a handful of moves.
- Fix:
  - In `web-ui/src/App.tsx`:
    - added `weldPreviewCacheReady` tracking,
    - made weld runs use `use_cache: true` so backend executes full planned steps cache,
    - when weld cache is stale, auto-refresh preview via `requestWeldPreview` before run,
    - reset cache readiness on clear/disconnect/load transitions.
  - Added UI copy update (`Editable Control Points`) to avoid implying that the list is every interpolated sample.
  - In `web-ui/src/previewUtils.ts`, surfaced path sample count in Program Tree subtitle for better operator visibility.
- Preventive rule:
  - For trajectory systems with both coarse declarative moves and dense cached execution plans, never treat them as interchangeable at run time for weld/high-fidelity workflows.

#### User Preferences

- New or reinforced preference:
  - Displayed/selected weld path and executed weld path must match; no hidden downsampling that changes robot motion.
  - If a mismatch is suspected, prioritize run-time correctness over prior convenience assumptions.
- How it changed execution:
  - Weld run path is now anchored to planned cache validity, with explicit stale-cache refresh.

#### What Worked

- Pattern/check that worked:
  - Keeping non-weld behavior unchanged while branching weld execution policy minimized regression risk.
  - Cache readiness flag cleanly coordinates plan/run state across clear/load/restore flows.

#### What Did Not Work

- Failed attempt and why:
  - Earlier global `use_cache: false` approach improved “start from current pose” semantics but broke weld trajectory fidelity by collapsing to endpoint commands.

#### Guardrails For Next Session

- Preflight rule:
  - If Program Tree move count is far smaller than expected path complexity, verify whether run path uses cached planned steps or endpoint re-planning.
  - For weld runs, treat cache freshness as a first-class precondition.

#### Follow-Ups / Risks

- Remaining risk or pending check:
  - Program Tree still emphasizes operation-level moves; consider adding a dedicated interpolated-path inspector node if operators need per-sample introspection.

### 2026-02-18 01:28 +11:00 - Exact path visibility: remove planner payload downsampling + tree from path samples

#### Task Summary

- Implemented full-fidelity path visibility so Program Tree can show exact planned path samples instead of trimmed endpoint-derived approximations.
- Removed planner payload downsampling that previously hid intermediate cartesian samples.

#### Mistakes And Fixes

- Source: `[user]`
- Mistake:
  - UI path inspection relied on lower-resolution representations (coarse move endpoints and downsampled cartesian payload), creating a trust gap for weld motion verification.
- Detection:
  - User explicitly rejected trimmed output and required exact movement visibility in Program Tree.
- Fix:
  - In `src/gradient_os/arm_controller/command_api.py`, removed `sample_stride` downsampling in `_append_cartesian_samples` so payload carries full planned cartesian samples.
  - In `web-ui/src/previewUtils.ts`, rewired `buildProgramTree` to use `plan.pathPoints` as primary execution tree content:
    - `Exact Path Samples` in grouped mode,
    - `Execution Path (Exact)` in chronological mode,
    - preserved control-point and controller-command groups as secondary views.
- Preventive rule:
  - For robotics inspection UIs, never downsample the authoritative displayed path unless user explicitly opts into a performance mode.

#### User Preferences

- New or reinforced preference:
  - Program Tree must reflect exactly where robot will move; no hidden trimming.
  - Coarse representations are acceptable only as supplemental metadata, not as the primary motion truth.
- How it changed execution:
  - Prioritized operator-trust visibility over payload compactness by default.

#### What Worked

- Pattern/check that worked:
  - Maintaining dual views (exact path + command metadata) preserved debugging utility without compromising motion fidelity visibility.

#### What Did Not Work

- Failed attempt and why:
  - Prior “show move count + path sample count” transparency helped diagnostics but did not satisfy requirement for exact per-sample tree inspection.

#### Guardrails For Next Session

- Preflight rule:
  - If a user asks for exact robot path visibility, ensure both backend payload and frontend tree model are fidelity-preserving end-to-end.

#### Follow-Ups / Risks

- Remaining risk or pending check:
  - Extremely long paths can create large tree DOMs; prefer virtualization if performance issues appear, not sample trimming.

### 2026-02-18 01:42 +11:00 - Remove approximate segment highlighting when exact mapping is unavailable

#### Task Summary

- Removed approximate weld-segment path highlighting from Program Tree to keep display semantics strictly truthful.
- Preserved command-level tree data only as reference metadata when exact path samples already exist.

#### Mistakes And Fixes

- Source: `[user]`
- Mistake:
  - Weld segment nodes were still assigning proportional `pathRange` guesses, which could imply false precision even after exact-path sample support was added.
- Detection:
  - User requirement emphasized exact reflection between Program Tree and rendered path; inferred ranges violate that constraint.
- Fix:
  - Removed weld segment `pathRange` inference from `web-ui/src/previewUtils.ts`.
  - Weld segment nodes now only target weld-edge focus (`weldSegmentEdgeId`) without claiming exact path subset.
  - Simplified command grouping so command nodes are clearly labeled as reference when exact path nodes are present.
- Preventive rule:
  - If exact mapping data is not available, do not synthesize approximate range overlays in robotics inspection views.

#### User Preferences

- New or reinforced preference:
  - Program Tree must never imply precision it does not actually have.
  - Exact path truth takes precedence over convenience grouping.
- How it changed execution:
  - Removed inferred path focus fields unless backed by exact sample indices.

#### What Worked

- Pattern/check that worked:
  - Separating "exact execution samples" from "controller command metadata" keeps debugging utility while preserving trust.

#### What Did Not Work

- Failed attempt and why:
  - Earlier proportional segment-range mapping was useful visually but not acceptable for exactness-critical inspection.

#### Guardrails For Next Session

- Preflight rule:
  - Any tree node that highlights path must be backed by explicit deterministic indices from planner output; otherwise omit the highlight mapping.

#### Follow-Ups / Risks

- Remaining risk or pending check:
  - If per-weld-segment exact highlighting is required later, backend should return section-to-sample index spans as part of planner payload.

### 2026-02-18 01:53 +11:00 - Waypoint editing migrated from Weld drawer into Program Tree

#### Task Summary

- Removed the `Editable Control Points` editor block from Weld drawer UI.
- Implemented Program Tree-native control-point editing flow so waypoint edits are driven from selected tree nodes.

#### Mistakes And Fixes

- Source: `[user]`
- Mistake:
  - Drawer-local waypoint editor duplicated editing context and conflicted with requirement that tree/path inspection be the source of truth.
- Detection:
  - User explicitly requested complete removal from drawer and routing edits through Program Tree.
- Fix:
  - In `web-ui/src/App.tsx`, removed Weld panel waypoint-edit props/UI and added tree-driven handlers:
    - edit selected control point coordinates,
    - add/remove control point,
    - apply edits via weld replan (or generic point replan for non-weld).
  - In `web-ui/src/components/ProgramFeatureTree.tsx`, added an inline editor section that appears when a `control_point_*` node is selected.
- Preventive rule:
  - Avoid duplicated edit surfaces for the same motion data; keep one primary editing interaction path tied to the inspection model.

#### User Preferences

- New or reinforced preference:
  - Waypoint editing should be centralized in Program Tree, not scattered in panel forms.
  - The path/tree workflow must remain coherent and trustworthy for motion changes.
- How it changed execution:
  - Shifted from drawer-local form controls to selection-driven tree editing.

#### What Worked

- Pattern/check that worked:
  - Reusing existing waypoint state and planner callbacks minimized risk while moving the UI interaction surface.

#### What Did Not Work

- Failed attempt and why:
  - Keeping both drawer and tree editors would continue UX ambiguity and contradict user’s “single source” editing requirement.

#### Guardrails For Next Session

- Preflight rule:
  - When a user requests “drive from X only,” remove parallel controls in other panels rather than trying to keep them synchronized ad hoc.

#### Follow-Ups / Risks

- Remaining risk or pending check:
  - If users miss discoverability, add small guidance text in Program Tree when no control point is selected.

### 2026-02-18 01:54 +11:00 - Tree node panel focus should follow weld context

#### Task Summary

- Adjusted Program Tree node focus target so selecting control/path nodes in weld plans keeps interaction in weld context.

#### Mistakes And Fixes

- Source: `[self]`
- Mistake:
  - After moving editing to Program Tree, control-point nodes still targeted trajectory panel by default, which could feel inconsistent for weld-first workflows.
- Detection:
  - Post-change review of `ProgramNode.focus.openPanel` mapping in `previewUtils.ts`.
- Fix:
  - Set default tree-node focus panel dynamically:
    - weld plan (`trajectory.weld` present) -> `"weld"`,
    - otherwise -> `"trajectory"`.
- Preventive rule:
  - When relocating an editing surface, re-check navigation/focus semantics so node selection context matches the new workflow.

#### User Preferences

- New or reinforced preference:
  - Program Tree should be the primary interaction context for waypoint edits.
- How it changed execution:
  - Ensured tree node selection supports weld-context editing rather than bouncing users to trajectory panel unintentionally.

#### What Worked

- Pattern/check that worked:
  - Deriving a `defaultFocusPanel` once in tree builder avoided repeated branching and kept node focus consistent.

#### What Did Not Work

- Failed attempt and why:
  - Static `openPanel: "trajectory"` across all plans was too rigid once weld editing moved to tree.

#### Guardrails For Next Session

- Preflight rule:
  - Any time node semantics change, validate both data fidelity and panel-navigation behavior together.

#### Follow-Ups / Risks

- Remaining risk or pending check:
  - If users want tree selection decoupled from panel switching, add a toggle for “selection-only mode” in settings.

### 2026-02-18 02:00 +11:00 - Preview waypoint marker size reduced to 1mm

#### Task Summary

- Reduced yellow preview waypoint sphere radius to 1mm for less visual clutter in the scene.

#### Mistakes And Fixes

- Source: `[user]`
- Mistake:
  - Waypoint spheres were oversized for dense weld-path inspection.
- Detection:
  - User requested “much smaller, maybe 1mm radius.”
- Fix:
  - Updated marker mesh radius in `web-ui/src/ArmVisualizer.tsx` from `0.008` to `0.001` meters in the preview path marker block.
- Preventive rule:
  - For dense robot path overlays, keep default markers small enough to avoid obscuring the path geometry.

#### User Preferences

- New or reinforced preference:
  - Preview waypoint markers should be visually subtle and not dominate the path view.
- How it changed execution:
  - Applied a direct geometry-radius change instead of additional styling complexity.

#### What Worked

- Pattern/check that worked:
  - Single-parameter radius change in the marker geometry cleanly addressed the request.

#### What Did Not Work

- Failed attempt and why:
  - None in this task.

#### Guardrails For Next Session

- Preflight rule:
  - When adjusting 3D markers, treat units as meters and validate requested real-world sizing directly in geometry values.

#### Follow-Ups / Risks

- Remaining risk or pending check:
  - Tiny markers may be hard to pick out at very wide zoom; consider optional user-adjustable marker scale if requested.

### 2026-02-18 02:04 +11:00 - Weld return_to_start must replan from current pre-run pose every run

#### Task Summary

- Fixed critical weld end-action regression where `return_to_start` could resolve to stale/wrong targets (including weld start) if an old preview plan cache was reused.

#### Mistakes And Fixes

- Source: `[user]`
- Mistake:
  - Weld run path could execute cached plan without guaranteed per-run replan from current robot state, so `return_to_start` target was not always the actual pre-weld run start pose.
- Detection:
  - User reported repeated return to weld start despite selecting `Return to trajectory start`.
- Fix:
  - In `web-ui/src/App.tsx`, updated weld run logic in `handleRunPreview`:
    - always call `requestWeldPreview(weldDraft)` immediately before running weld preview,
    - then execute with `use_cache: true` against the just-refreshed plan.
  - This forces backend planner to recapture current start pose each run and regenerate post-action transitions accordingly.
- Preventive rule:
  - For semantics that depend on runtime start context (like `return_to_start`), never allow weld execution to skip replan on run.

#### User Preferences

- New or reinforced preference:
  - `Return to trajectory start` must mean “the robot pose right before this weld run starts,” never weld-start fallback.
- How it changed execution:
  - Prioritized semantic correctness and determinism over cache-only run latency.

#### What Worked

- Pattern/check that worked:
  - Replan-then-run for weld previews preserves high-fidelity path execution while guaranteeing correct start-context capture.

#### What Did Not Work

- Failed attempt and why:
  - Conditional cache refresh based on stale flags was insufficient for strict runtime start semantics.

#### Guardrails For Next Session

- Preflight rule:
  - If an end-action references “start” and operator intent is per-run, enforce replan-at-run regardless of prior cache freshness.

#### Follow-Ups / Risks

- Remaining risk or pending check:
  - Additional replan time before each weld run is expected; optimize only if needed, without compromising start-context correctness.

### 2026-02-18 02:19 +11:00 - Stabilize weld run-state lifecycle and isolate jog loop

#### Task Summary

- Fixed a backend execution-state bug that could clear motion state mid-trajectory and allow control-loop contention.
- Added trajectory-start guard to stop active jog mode before weld/trajectory playback.

#### Mistakes And Fixes

- Source: `[self]`
- Mistake:
  - Nested step execution in `trajectory_execution._execute_joint_path` reused `_open_loop_executor_thread` which could clear `trajectory_state` (`is_running`, `thread`) during a still-active multi-step weld run.
- Detection:
  - User-reported jitter/snap behavior during weld execution; code audit showed executor cleanup was tied only to thread identity, which matches nested step execution.
- Fix:
  - Added `owns_trajectory_state` parameter to open/closed executors and disabled state cleanup for nested step calls.
  - Updated `handle_run_trajectory` to stop jog mode before run and abort if jog remains active.
- Preventive rule:
  - Any low-level executor used both standalone and nested must have explicit lifecycle ownership; never let nested calls mutate global run flags.

#### User Preferences

- New or reinforced preference:
  - Execution correctness and deterministic robot behavior are higher priority than convenience/background control loops.
  - User expects direct fixes, not speculative discussion.
- How it changed execution:
  - Focused on controller run-state/jog isolation, implemented concrete backend patches first, then validated syntax/lints.

#### What Worked

- Pattern/check that worked:
  - Tracing end-to-end from UI symptom to controller state transitions exposed the lifecycle race quickly.

#### What Did Not Work

- Failed attempt and why:
  - Looking only at weld planning math was insufficient; the dominant issue was runtime executor/jog interaction, not weld geometry sampling itself.

#### Guardrails For Next Session

- Preflight rule:
  - For motion bugs with "random snaps/jitter," inspect global motion flags (`is_running`, `is_jogging`, `thread`) and thread cleanup points before tuning planners.

#### Follow-Ups / Risks

- Remaining risk or pending check:
  - Validate with live weld run in simulator to confirm no residual jitter under active UI polling and no unintended return-to-weld-start behavior.

### 2026-02-18 02:26 +11:00 - Runtime confirmation: execution-state fix is primary root cause

#### Task Summary

- Recorded user confirmation that weld execution now behaves correctly after the controller patch.
- Captured that the issue previously occurred even with jog disabled, reinforcing execution-state lifecycle as primary fault domain.

#### Mistakes And Fixes

- Source: `[user]`
- Mistake:
  - Earlier suspicion that jog could be the sole cause was incomplete.
- Detection:
  - User explicitly reported prior reproduction with jog disabled.
- Fix:
  - Keep execution-state lifecycle fix as the core resolution.
  - Retain jog-stop guard as non-invasive protection against future control-loop contention.
- Preventive rule:
  - For motion jitter/snap bugs, prioritize controller state lifecycle and thread ownership analysis before attributing solely to UI-side control streams.

#### User Preferences

- New or reinforced preference:
  - Preserve practical safety guards if they do not add downside, even when not the primary fix.
- How it changed execution:
  - Kept jog isolation check in place as defense-in-depth rather than removing it after root-cause confirmation.

#### What Worked

- Pattern/check that worked:
  - Combining code-level race fix with runtime user validation quickly converged on true root cause.

#### What Did Not Work

- Failed attempt and why:
  - Treating jog contention as the only likely source would have underexplained the jog-disabled reproductions.

#### Guardrails For Next Session

- Preflight rule:
  - If a bug reproduces with a suspected subsystem disabled, immediately elevate investigation to shared/global state and thread-lifecycle paths.

#### Follow-Ups / Risks

- Remaining risk or pending check:
  - None immediate; monitor for recurrence under long runs or repeated run/stop cycles.

### 2026-02-18 11:47 +11:00 - README refresh for merge readiness

#### Task Summary

- Added a root repository `README.md` and refreshed `web-ui/README.md` to reflect current product behavior.
- Prepared branch-level merge commit message guidance.

#### Mistakes And Fixes

- Source: `[self]`
- Mistake:
  - Assuming a root README existed would have left the “main repo readme” request partially done.
- Detection:
  - File check showed no `README.md` at repository root.
- Fix:
  - Created root `README.md` and updated `web-ui/README.md` with current architecture/workflow notes.
- Preventive rule:
  - For docs requests, verify file existence first and create missing canonical docs rather than only editing submodule docs.

#### User Preferences

- New or reinforced preference:
  - Wants merge-ready artifacts: clear commit messaging plus up-to-date top-level and UI docs.
- How it changed execution:
  - Prioritized practical documentation updates and concise merge messaging over deep code changes.

#### What Worked

- Pattern/check that worked:
  - Pairing root + feature-area README updates keeps main-branch handoff clearer for maintainers/operators.

#### What Did Not Work

- Failed attempt and why:
  - None in this step.

#### Guardrails For Next Session

- Preflight rule:
  - When asked for "main repo README," explicitly confirm root-level presence and update/create accordingly.

#### Follow-Ups / Risks

- Remaining risk or pending check:
  - Consider later consolidation between `README.md`, `AGENTS.md`, and `docs/README.md` to reduce duplicated startup guidance.

### 2026-02-18 11:55 +11:00 - Main README clarification: docs/README is canonical

#### Task Summary

- Updated `docs/README.md` after user clarified this is the canonical "main README" for repo-level documentation.
- Built a comprehensive commit-message draft based on full branch diff context rather than only current unstaged files.

#### Mistakes And Fixes

- Source: `[user]`
- Mistake:
  - Initial README update targeted root-level docs first; user clarified canonical main README is `docs/README.md`.
- Detection:
  - Direct user correction in follow-up message.
- Fix:
  - Added a dedicated `STEP_LOADER Branch Highlights` section in `docs/README.md` with end-to-end scope summary.
- Preventive rule:
  - In this repo, treat `docs/README.md` as the primary documentation entrypoint unless user asks otherwise.

#### User Preferences

- New or reinforced preference:
  - Wants branch merge materials to be comprehensive and grounded in the full branch scope.
- How it changed execution:
  - Used `master..HEAD` log/stat context before drafting commit message language.

#### What Worked

- Pattern/check that worked:
  - Pairing user clarification with git-range analysis produced accurate high-level change framing.

#### What Did Not Work

- Failed attempt and why:
  - A root-only README update was insufficient for this repository's doc convention.

#### Guardrails For Next Session

- Preflight rule:
  - For merge/prep requests, confirm canonical docs path and summarize against `base..HEAD` rather than local unstaged delta only.

#### Follow-Ups / Risks

- Remaining risk or pending check:
  - None immediate.

### 2026-02-18 11:59 +11:00 - docs/README rewritten as true onboarding entrypoint

#### Task Summary

- Replaced `docs/README.md` content with newcomer-first documentation: features, system function, and usage workflow.
- Removed release-note style sectioning that did not match the file's purpose.

#### Mistakes And Fixes

- Source: `[user]`
- Mistake:
  - Prior update style emphasized branch-change summaries instead of serving as a practical start guide.
- Detection:
  - Explicit user correction on expected README role and tone.
- Fix:
  - Full rewrite of `docs/README.md` around onboarding flow and operational usage.
- Preventive rule:
  - For canonical README files, optimize for "what it is / how it works / how to use it" before changelog-style content.

#### User Preferences

- New or reinforced preference:
  - README must function as the primary onboarding document for new repo users.
- How it changed execution:
  - Shifted from patching sections to a complete structure reset aligned to onboarding intent.

#### What Worked

- Pattern/check that worked:
  - Rebuilding the file from scratch avoided carrying over conflicting structure and tone.

#### What Did Not Work

- Failed attempt and why:
  - Incremental edits against prior structure kept reintroducing non-onboarding framing.

#### Guardrails For Next Session

- Preflight rule:
  - Before editing a "main README", define target reader and first-use journey explicitly, then shape sections around that path.

#### Follow-Ups / Risks

- Remaining risk or pending check:
  - None immediate.

### 2026-02-18 12:28 +11:00 - Diagram renderer compatibility fix only

#### Task Summary

- Applied targeted fixes to Mermaid blocks in `docs/README.md` so diagrams render in the repo preview environment.

#### Mistakes And Fixes

- Source: `[user]`
- Mistake:
  - Diagram syntax was too permissive/complex for the active markdown renderer and produced "No diagram type detected" errors.
- Detection:
  - User screenshots showed Mermaid parser failures in rendered README sections.
- Fix:
  - Rewrote Mermaid blocks with strict `flowchart TD` and simplified `sequenceDiagram` content.
  - Removed HTML line breaks and complex quoted labels from diagram text.
- Preventive rule:
  - For README diagrams, prefer conservative Mermaid syntax over decorative labels.

#### User Preferences

- New or reinforced preference:
  - Scope must stay exactly on requested fix when user asks for a targeted patch.
- How it changed execution:
  - Limited edits to diagram blocks only.

#### What Worked

- Pattern/check that worked:
  - Re-typing diagram blocks from scratch prevented hidden syntax issues from surviving.

#### What Did Not Work

- Failed attempt and why:
  - None in this step.

#### Guardrails For Next Session

- Preflight rule:
  - When diagrams fail, first simplify to minimal valid Mermaid syntax before broader doc edits.

#### Follow-Ups / Risks

- Remaining risk or pending check:
  - None immediate.

### 2026-02-18 23:07 +11:00 - No-fallback robot catalog migration

#### Task Summary

- Migrated robot assets from legacy root path to canonical `robots/<robot_id>/`.
- Added strict manifest-based resolver and rewired controller/IK/numeric/web to use it.
- Removed legacy mini-arm asset files and hardcoded web/solver path usage.

#### Mistakes And Fixes

- Source: `[self]`
- Mistake:
  - First web sync script copied entire robot asset directories into `web-ui/public/assets/robots/`, including non-runtime docs/scripts.
- Detection:
  - Post-sync file inspection showed extra copied files not required by the browser asset loader.
- Fix:
  - Reduced sync scope to canonical URDF + optional `stl-files/` and generated canonical `robot.urdf`.
- Preventive rule:
  - For build-time asset sync, copy the minimum runtime surface by default; only add broader copy lists intentionally.

#### User Preferences

- Source: `[user]`
- New or reinforced preference:
  - Commit to migration with no fallback code paths and clean legacy references.
  - Prefer concrete implementation over explanation-only responses.
- How it changed execution:
  - Removed old root asset files and switched runtime to manifest-driven resolution in one pass.

#### What Worked

- Source: `[tool]`
- Pattern/check that worked:
  - Running `run_controller --list-robots` plus a short unbuffered `--sim` startup smoke gave fast confidence that registry + runtime wiring stayed healthy.
  - `ReadLints` + `npm run build` caught no regressions after broad multi-file migration.

#### What Did Not Work

- Source: `[self]`
- Failed attempt and why:
  - Buffered simulation run produced no immediate logs in the first smoke attempt; switched to `python -u` to verify startup state reliably.

#### Guardrails For Next Session

- Preflight rule:
  - For additional robots, require `robots/<robot_id>/robot.json` first and fail fast if missing instead of adding defaults.
  - Keep controller-facing robot config (`robot_id`) decoupled from user-facing config key (`--robot` registry name).
  - Re-run asset sync before web build/dev (`npm run sync:robot-assets`) whenever robot catalog content changes.

#### Follow-Ups / Risks

- Remaining risk or pending check:
  - If future URDFs reference textures or mesh folders outside `stl-files/`, update `web-ui/scripts/sync-robot-assets.mjs` include logic explicitly.

### 2026-02-18 23:22 +11:00 - Mesh/USD recovery after robot asset migration

#### Task Summary

- Restored missing mini-arm mesh assets required by URDF rendering in web UI.
- Reinstated USD asset in canonical robot bundle and ensured sync keeps USD artifacts.

#### Mistakes And Fixes

- Source: `[self]`
- Mistake:
  - Removed/omitted required STL/USD robot files during migration cleanup, which left the web URDF without mesh payloads.
- Detection:
  - User reported blank robot render; terminal startup logs were healthy, indicating a front-end asset issue rather than controller failure.
- Fix:
  - Restored legacy tracked files via `git restore`.
  - Copied `mini-6dof-arm/stl-files/*.stl` and `mini-6dof-arm/mini-6dof-arm.usd` into `robots/mini-6dof-arm/`.
  - Re-ran `npm run sync:robot-assets` and confirmed generated web assets include STL + USD.
- Preventive rule:
  - For URDF migrations, treat mesh + scene files as required runtime artifacts; validate file presence in both canonical catalog and web output before handoff.

#### User Preferences

- Source: `[user]`
- New or reinforced preference:
  - Keep USD assets preserved because they are useful for external tools like Isaac Sim.
  - Prioritize immediate implementation/fixes over explanation-only responses.
- How it changed execution:
  - Added explicit USD preservation in manifest and sync behavior and ran full sync/build verification.

#### What Worked

- Source: `[tool]`
- Pattern/check that worked:
  - `git ls-files "mini-6dof-arm/*" "mini-6dof-arm/stl-files/*"` quickly confirmed STL/USD were tracked and recoverable.
  - `Get-ChildItem -Recurse` checks on `web-ui/public/assets/robots/mini-6dof-arm` verified the effective runtime payload after sync.

#### What Did Not Work

- Source: `[tool]`
- Failed attempt and why:
  - `Glob`/text-file searches did not surface `.stl`/`.usd` binaries, which initially obscured file existence.

#### Guardrails For Next Session

- Preflight rule:
  - When migrating robot assets, inventory all binary artifacts (`.stl`, `.usd`, etc.) from git index before deleting legacy paths.
  - After any `sync-robot-assets` change, verify generated `web-ui/public/assets/robots/<id>/` contains mesh files referenced by URDF.

#### Follow-Ups / Risks

- Remaining risk or pending check:
  - Consider adding a CI check that parses URDF mesh references and asserts corresponding files exist in synced web assets.

### 2026-02-18 23:25 +11:00 - USD optionality clarification

#### Task Summary

- Applied user clarification: USD files should be preserved when present but never required.

#### Mistakes And Fixes

- Source: `[self]`
- Mistake:
  - Added `models.usd` in `robot.json`, which could imply required semantics even though runtime did not enforce it.
- Detection:
  - User explicitly clarified that USD is not a hard requirement and must not block future flows.
- Fix:
  - Removed `models.usd` from manifest.
  - Kept sync script behavior as optional pass-through copy for `.usd/.usda/.usdc`.
- Preventive rule:
  - For non-runtime auxiliary artifacts, avoid placing them in required manifest schema unless there is a real consumer and enforcement intent.

#### User Preferences

- Source: `[user]`
- New or reinforced preference:
  - Preserve USD files, but do not introduce hard checks for them.
- How it changed execution:
  - Kept USD copy support and removed manifest-level required implication.

#### What Worked

- Source: `[tool]`
- Pattern/check that worked:
  - Quick `sync:robot-assets` + `build` loop validated optional USD handling without regressions.

#### What Did Not Work

- Source: `[tool]`
- Failed attempt and why:
  - Used `&&` in PowerShell command chain; shell requires `;` in this environment.

#### Guardrails For Next Session

- Preflight rule:
  - Treat manifest entries as contract surface; only include fields intended for long-term validation.

#### Follow-Ups / Risks

- Remaining risk or pending check:
  - If USD becomes a first-class runtime input later, add explicit schema/versioning before enforcing checks.

### 2026-02-18 23:39 +11:00 - Gradient-05 template scaffold

#### Task Summary

- Created a ready-to-fill template for a second robot model (`gradient-05`) in both asset catalog and controller registry.
- Kept migration constraints intact (manifest-driven assets, no fallback requirements changed).

#### Mistakes And Fixes

- Source: `[self]`
- Mistake:
  - None during this scaffold pass.
- Detection:
  - N/A.
- Fix:
  - N/A.
- Preventive rule:
  - For new robot templates, keep them non-default and provide valid placeholder files so sync/build/list checks stay green.

#### User Preferences

- Source: `[user]`
- New or reinforced preference:
  - User wants implementation-first assistance and practical setup scaffolds to fill in real robot details incrementally.
- How it changed execution:
  - Added concrete folder/file templates plus immediate registry wiring instead of providing only instructions.

#### What Worked

- Source: `[tool]`
- Pattern/check that worked:
  - Inheriting `Gradient05Config` from `Gradient0Config` provided a safe starter config while exposing a new `robot_id`.
  - Running `run_controller --list-robots` and `sync:robot-assets` confirmed end-to-end registration quickly.

#### What Did Not Work

- Source: `[tool]`
- Failed attempt and why:
  - None in this pass.

#### Guardrails For Next Session

- Preflight rule:
  - Keep controller key (`gradient05`) and asset ID (`gradient-05`) explicit and documented to avoid naming drift.
  - Ensure each new robot manifest references real files from day one, even if placeholders.

#### Follow-Ups / Risks

- Remaining risk or pending check:
  - Active simulator/API sessions can mutate runtime files (observed `recorded_trajectories/__weld_preview__.json`); avoid touching unrelated runtime outputs during scaffolding tasks.

### 2026-02-18 23:59 +11:00 - Gradient-05 URDF mesh path wiring

#### Task Summary

- Updated `robots/gradient-05/gradient-05.urdf` to point link visuals at `stl-files/*` mesh filenames.

#### Mistakes And Fixes

- Source: `[self]`
- Mistake:
  - None in this pass.
- Detection:
  - N/A.
- Fix:
  - N/A.
- Preventive rule:
  - When users request mesh-path wiring, keep references relative (`stl-files/...`) to match existing resolver/web-loader behavior.

#### User Preferences

- Source: `[user]`
- New or reinforced preference:
  - User wants direct implementation updates to files currently being authored.
- How it changed execution:
  - Patched URDF immediately with concrete mesh filename references instead of leaving primitive placeholders.

#### What Worked

- Source: `[tool]`
- Pattern/check that worked:
  - Referencing the existing mini-arm URDF mesh naming convention (`base`, `L1`-`L5`, `wrist`) provided a consistent template.

#### What Did Not Work

- Source: `[tool]`
- Failed attempt and why:
  - None in this pass.

#### Guardrails For Next Session

- Preflight rule:
  - Keep scaffold URDF and expected mesh filenames synchronized with files that will be dropped into `stl-files/`.

#### Follow-Ups / Risks

- Remaining risk or pending check:
  - Rendering for `gradient-05` will remain incomplete until matching STL files are added.

### 2026-02-19 00:14 +11:00 - Move link offsets into joints

#### Task Summary

- Moved non-zero visual offsets from links into joint origins for `robots/gradient-05/gradient-05.urdf`.
- Reset affected link visual origins back to `xyz="0 0 0"` / `rpy="0 0 0"`.

#### Mistakes And Fixes

- Source: `[self]`
- Mistake:
  - None in this pass.
- Detection:
  - N/A.
- Fix:
  - N/A.
- Preventive rule:
  - For "transfer offsets" requests, apply a deterministic mapping and document it explicitly for user verification.

#### User Preferences

- Source: `[user]`
- New or reinforced preference:
  - User wants quick direct transform edits in the active URDF file.
- How it changed execution:
  - Implemented immediate in-file transfer (no planning detour) and preserved exact numeric values.

#### What Worked

- Source: `[tool]`
- Pattern/check that worked:
  - Reading full URDF before patching made it straightforward to move offsets and preserve mesh path sections unchanged.

#### What Did Not Work

- Source: `[tool]`
- Failed attempt and why:
  - None in this pass.

#### Guardrails For Next Session

- Preflight rule:
  - When base link also has a visual offset, call out that transfer mapping choice (sequential link-to-joint mapping here) so frame assumptions remain explicit.

#### Follow-Ups / Risks

- Remaining risk or pending check:
  - If the intended mapping was "incoming joint per link child frame" rather than sequential order, joint origins may need a remap.

### 2026-02-19 00:59 +11:00 - DH extraction and visual validation tooling

#### Task Summary

- Implemented `robots/gradient-05/dh_tools.py` to extract initial DH rows from URDF and validate against URDF FK.
- Added optional visual comparison output (`dh_validation.png`) for fast geometric sanity checks.

#### Mistakes And Fixes

- Source: `[tool]`
- Mistake:
  - Initial script import relied on `utilities/urdf2dh` package internals that required `nptyping`, which is not installed in repo venv.
- Detection:
  - Runtime error: `ModuleNotFoundError: No module named 'nptyping'`.
- Fix:
  - Inlined the required `get_a` / `get_alpha` math in `dh_tools.py` to remove external import dependency.
- Preventive rule:
  - For repo-local utilities, avoid pulling optional-typing dependencies into runtime tooling unless they are already in project deps.

#### User Preferences

- Source: `[user]`
- New or reinforced preference:
  - Wants practical DH setup support plus visual validation, not just conceptual advice.
- How it changed execution:
  - Delivered executable extraction + validation flow and generated a visual artifact.

#### What Worked

- Source: `[tool]`
- Pattern/check that worked:
  - Combined numeric FK error metrics + a side-by-side 3D plot to quickly distinguish "extractable" vs "needs manual tuning".

#### What Did Not Work

- Source: `[tool]`
- Failed attempt and why:
  - Auto extraction produced high mismatch (meter-scale position error, ~180 deg orientation max), indicating frame convention/theta offsets are not solved by naive extraction alone.

#### Guardrails For Next Session

- Preflight rule:
  - Treat URDF-derived DH as an initial estimate; always run FK residual checks before wiring into numeric solver runtime.
  - If validation is poor, move to manual convention fitting (especially theta offsets and frame assignments), not blind CSV acceptance.

#### Follow-Ups / Risks

- Remaining risk or pending check:
  - `gradient-05` DH table still requires manual tuning before numeric IK can be trusted in control loops.

### 2026-02-19 01:10 +11:00 - DH workflow restart + visualization build

#### Task Summary

- Rebuilt `robots/gradient-05/dh_tools.py` from scratch around a fit-to-URDF workflow.
- Generated fresh DH CSV and visualization output in one pass.

#### Mistakes And Fixes

- Source: `[tool]`
- Mistake:
  - First rerun used an overly heavy fit configuration (`fit-samples=1200`, high eval budget), causing a long-running job with no timely output.
- Detection:
  - Command exceeded the blocking timeout and continued in background without completing within practical iteration time.
- Fix:
  - Added `--max-nfev` CLI control, switched to robust `soft_l1` loss, reduced sample counts, and reran with lighter settings.
- Preventive rule:
  - For nonlinear identification loops, expose evaluation/sampling controls in CLI and start with moderate sample sizes before scaling up.

#### User Preferences

- Source: `[user]`
- New or reinforced preference:
  - User explicitly wants visualization built as part of DH validation.
- How it changed execution:
  - Validation plot generation was included in the main workflow and regenerated in the successful run.

#### What Worked

- Source: `[tool]`
- Pattern/check that worked:
  - Restarting from a cleaner fit pipeline reduced position residuals and produced repeatable artifacts (`dh_params.csv`, `dh_validation.png`) in one command.

#### What Did Not Work

- Source: `[tool]`
- Failed attempt and why:
  - Even with fitting, orientation residual remains very high (~124 deg mean), and some parameters saturate bounds, suggesting the DH-only model is still under-parameterized for this URDF representation.

#### Guardrails For Next Session

- Preflight rule:
  - Treat high orientation residual + bound saturation as a structural model warning (not just optimizer tuning noise).
  - If residuals stay high, add base/tool transform fitting or revise frame assumptions before accepting DH output.

#### Follow-Ups / Risks

- Remaining risk or pending check:
  - The current `gradient-05` DH CSV is not yet trustworthy for production numeric IK without an improved kinematic mapping model.

### 2026-02-19 01:20 +11:00 - IKFast vs QuIK validation on mini-arm

#### Task Summary

- Ran requested cross-check using `mini-6dof-arm`: IKFast treated as reference; compared QuIK using canonical DH vs freshly extracted DH.
- Repaired QuIK integration path so numeric backend can actually be benchmarked in this environment.

#### Mistakes And Fixes

- Source: `[tool]`
- Mistake:
  - QuIK backend was effectively disconnected: no built `pyquik` extension and wrapper expected methods (`ik`/`fk`) not exposed by bindings (`solve`/`FK`).
- Detection:
  - Runtime errors: missing `pyquik`, then attribute mismatch (`IKSolver` missing `ik`).
- Fix:
  - Built `pyquik` locally and added method adapter in `numeric_wrapper`.
  - Added manifest-driven numeric frame/sign overrides and set mini-arm overrides in `robot.json`.
- Preventive rule:
  - Keep wrapper API assumptions in lockstep with actual pybind export names; add adapters instead of monkey-patching extension objects.

#### User Preferences

- Source: `[user]`
- New or reinforced preference:
  - Validate extraction quality empirically against IKFast baseline, not by intuition.
- How it changed execution:
  - Added a dedicated benchmark script and generated quantitative + visual comparisons for canonical vs extracted DH.

#### What Worked

- Source: `[tool]`
- Pattern/check that worked:
  - Estimating and applying a constant QuIK frame transform (`numeric.ttool`) for mini-arm collapsed FK mismatch to near numerical zero with canonical DH.
  - Canonical-DH benchmark produced strong FK/IK agreement with IKFast.

#### What Did Not Work

- Source: `[tool]`
- Failed attempt and why:
  - URDF seed extraction method (`--seed-only`) produced poor FK alignment against IKFast on mini-arm (large translational and rotational error), confirming it is not sufficient as-is.

#### Guardrails For Next Session

- Preflight rule:
  - Any new DH extraction method must pass IKFast-vs-QuIK FK/IK benchmark on mini-arm before being trusted for new robots.
  - Validate both frame alignment (Tbase/Ttool) and DH values; poor results can come from either.

#### Follow-Ups / Risks

- Remaining risk or pending check:
  - For `gradient-05`, we still need a robust frame-aware extraction/fitting approach that reproduces mini-arm style benchmark performance.

### 2026-02-19 09:24 +11:00 - QuIK upstream status check

#### Task Summary

- Performed read-only verification that nested `src/numeric_solver/quik` matches upstream `origin/main`.

#### Mistakes And Fixes

- Source: `[self]`
- Mistake:
  - None in this step.
- Detection:
  - N/A.
- Fix:
  - N/A.
- Preventive rule:
  - For vendor repos marked as important, verify with fetch+commit comparison before proposing updates.

#### User Preferences

- Source: `[user]`
- New or reinforced preference:
  - Do not edit the `quik` repo content; only ensure/update to latest when needed.
- How it changed execution:
  - Kept checks strictly read-only and avoided checkout/pull/update operations because repo is already current.

#### What Worked

- Source: `[tool]`
- Pattern/check that worked:
  - `git rev-list --left-right --count HEAD...origin/main` provided an immediate authoritative up-to-date result (`0 0`).

#### What Did Not Work

- Source: `[tool]`
- Failed attempt and why:
  - None in this step.

#### Guardrails For Next Session

- Preflight rule:
  - Treat detached HEAD in nested dependency repos as potentially intentional pinning unless user explicitly requests branch-tracking behavior.

#### Follow-Ups / Risks

- Remaining risk or pending check:
  - None; `quik` repo local commit and upstream `origin/main` currently match exactly.

### 2026-02-19 11:58 +11:00 - Full kinematics hardening execution

#### Task Summary

- Implemented all stages in the kinematics reliability plan end-to-end:
  - IK semantics fix,
  - versioned profile foundation + runtime control plane,
  - UI offset controls,
  - planner gates/fallback diagnostics,
  - calibration workflow,
  - motion-state/audit hardening.

#### Mistakes And Fixes

- Source: `[self]`
- Mistake:
  - API kinematics helper initially used a keyword-only timeout signature but was called with positional args via `run_in_threadpool`, causing endpoint runtime failures.
- Detection:
  - `tests/test_api_endpoints.py` failed with `TypeError: ... takes 1 positional argument but 2 were given`.
- Fix:
  - Changed helper signature to accept positional timeout and re-ran API tests to green.
- Preventive rule:
  - For threadpool callback helpers in FastAPI, prefer positional-compatible signatures unless every call site uses keyword binding.

- Source: `[tool]`
- Mistake:
  - Planner hardening gates broke legacy unit tests that used dummy Cartesian points incompatible with FK residual checks.
- Detection:
  - `tests/test_planning.py` started failing after gate insertion.
- Fix:
  - Updated tests to patch `_validate_joint_trajectory_gates` for behavior-isolation in unwrap/smoothing unit cases.
- Preventive rule:
  - When adding physical-validity gates, adapt logic-unit tests to isolate the behavior under test rather than relying on previously permissive planner side effects.

#### User Preferences

- Source: `[user]`
- New or reinforced preference:
  - Implement changes directly; do not stop at discussion.
  - Execute the full staged plan and keep todo status accurate.
  - Always update both `DEVLOG.md` and `AGENT_SCRATCHPAD.md` for meaningful work.
- How it changed execution:
  - Kept continuous implementation flow across all stages in one turn, with full test/build/lint validation and mandatory memory-loop writeback.

#### What Worked

- Source: `[tool]`
- Pattern/check that worked:
  - Stage-by-stage with frequent regression runs (`pytest` subsets + `npm run build`) prevented hidden cross-layer breakage.
  - Adding explicit runtime/profile abstractions (`kinematics/runtime.py`, `kinematics/profile.py`) made API/controller/UI integration straightforward.

#### What Did Not Work

- Source: `[tool]`
- Failed attempt and why:
  - Running tests from PowerShell required explicit `&` call syntax and local venv deps (`pytest`, `httpx`) before endpoint tests could execute.

#### Guardrails For Next Session

- Preflight rule:
  - For broad multi-stage reliability changes, lock in a quick regression suite early and rerun after each stage.
  - If planner semantics tighten, immediately audit and update tests that intentionally use synthetic/non-physical trajectories.
  - Before final handoff, always append both memory files and run `ReadLints` on changed paths.

#### Follow-Ups / Risks

- Remaining risk or pending check:
  - Continue migrating remaining legacy direct `trajectory_state[...]` accesses to lock helpers for complete thread-safety coverage.
  - Consider adding dedicated long-window diagnostics aggregation for planner residual trends (beyond per-plan snapshots).

### 2026-02-19 12:24 +11:00 - Live weld planning crash fix (`NaN` JSON + post-action fallback)

#### Task Summary

- Debugged live API/controller logs for weld planning failures and applied targeted runtime fixes.
- Eliminated FastAPI response crash caused by non-JSON-safe planner diagnostics values.
- Made optional post-action transition failures non-fatal with explicit warnings.

#### Mistakes And Fixes

- Source: `[tool]`
- Mistake:
  - Planner diagnostics could emit `NaN`/`Inf` (`joint_limit_margin_rad`) which broke JSON response encoding in `/trajectory/plan-weld`.
- Detection:
  - Live API trace showed `ValueError: Out of range float values are not JSON compliant: nan`.
- Fix:
  - Added JSON-safe float conversion in `trajectory_execution._validate_joint_trajectory_gates` and regression coverage in `tests/test_planning.py`.
- Preventive rule:
  - Any telemetry/payload diagnostics intended for HTTP JSON must sanitize non-finite floats before storing/returning.

- Source: `[self]`
- Mistake:
  - Optional post-action transitions (`return_to_start` segments) were treated as hard planning failures, causing avoidable 502 responses.
- Detection:
  - Live terminal logs showed weld interior succeeded while post-action transition failed and aborted whole plan.
- Fix:
  - Converted post-action transition failures to warnings and skipped just that optional segment; preserved successful weld preview output.
- Preventive rule:
  - Keep optional workflow features non-fatal unless explicitly marked safety-critical; surface clear warnings instead.

#### User Preferences

- Source: `[user]`
- New or reinforced preference:
  - Fast, direct debugging from real runtime logs and concrete fixes over theory.
- How it changed execution:
  - Started from live terminal evidence first, then patched exact failure path and validated with tests.

#### What Worked

- Source: `[tool]`
- Pattern/check that worked:
  - Reading active API/sim logs immediately exposed root cause and avoided speculative debugging.

#### What Did Not Work

- Source: `[tool]`
- Failed attempt and why:
  - None in this pass.

#### Guardrails For Next Session

- Preflight rule:
  - For runtime API failures, always inspect terminal stack traces first and prioritize crash-proof serialization paths for diagnostics-heavy payloads.

#### Follow-Ups / Risks

- Remaining risk or pending check:
  - Monitor frequency of post-action warnings; if frequent, add adaptive return strategy options (clearance scaling or alternate post-action policy).

### 2026-02-19 12:08 +11:00 - Mermaid diagram parse error fix in docs

#### Task Summary

- Fixed a non-rendering Mermaid diagram in `docs/ik_solver.md` by simplifying flowchart syntax to a parser-safe form.

#### Mistakes And Fixes

- Source: `[self]`
- Mistake:
  - Previous Mermaid block used syntax that was accepted by some engines but failed in the active renderer (`<br/>`, mixed node forms, trailing semicolons).
- Detection:
  - Renderer error reported parse failure near the first node label (`got 'PS'`).
- Fix:
  - Rewrote the block using `flowchart TD`, quoted labels, and cleaner decision/edge syntax.
- Preventive rule:
  - For docs Mermaid diagrams, prefer conservative syntax (quoted text labels, no HTML tags) for cross-renderer compatibility.

#### User Preferences

- Source: `[user]`
- New or reinforced preference:
  - Explain changes, but implement directly instead of only discussing.
- How it changed execution:
  - Applied the doc patch immediately, then logged exact changes and validation.

#### What Worked

- Source: `[tool]`
- Pattern/check that worked:
  - Reading the exact selected lines from `docs/ik_solver.md` before patching made it easy to target the failing Mermaid fragment precisely.

#### What Did Not Work

- Source: `[tool]`
- Failed attempt and why:
  - None in this pass.

#### Guardrails For Next Session

- Preflight rule:
  - When Mermaid parse errors mention unexpected tokens, first remove HTML tags and unquoted complex labels, then validate with minimal syntax.

#### Follow-Ups / Risks

- Remaining risk or pending check:
  - If another renderer is used (e.g., different Mermaid version), keep fallback to even simpler labels without punctuation if needed.

### 2026-02-19 12:12 +11:00 - Mermaid fallback syntax hardening

#### Task Summary

- Applied a stricter Mermaid fallback for the IK docs diagram after parse errors persisted in the active Markdown renderer.

#### Mistakes And Fixes

- Source: `[self]`
- Mistake:
  - First fix still assumed renderer support for quoted labels and richer edge text.
- Detection:
  - User screenshot showed parse failure continuing on the conditional branch line.
- Fix:
  - Replaced diagram labels with plain words and minimal symbols only.
- Preventive rule:
  - For unknown Mermaid engine versions, default immediately to minimal syntax before using richer label formats.

#### User Preferences

- Source: `[user]`
- New or reinforced preference:
  - Keep implementing concrete fixes until the issue is actually resolved.
- How it changed execution:
  - Performed a second immediate patch instead of only suggesting possible causes.

#### What Worked

- Source: `[tool]`
- Pattern/check that worked:
  - Iterative simplification with direct file patching is effective for parser-compatibility issues.

#### What Did Not Work

- Source: `[tool]`
- Failed attempt and why:
  - The first post-error Mermaid variant remained too expressive for this renderer.

#### Guardrails For Next Session

- Preflight rule:
  - For docs visuals, start with compatibility-first Mermaid syntax, then add expressiveness only if confirmed supported.

#### Follow-Ups / Risks

- Remaining risk or pending check:
  - If errors continue, use non-Mermaid fallback (ASCII/bulleted flow) in docs to avoid renderer dependency.

### 2026-02-19 12:13 +11:00 - Mermaid diagram type compatibility fallback

#### Task Summary

- Switched the IK docs diagram to legacy Mermaid grammar to handle renderers that do not recognize `flowchart`.

#### Mistakes And Fixes

- Source: `[tool]`
- Mistake:
  - Assumed active renderer supported modern `flowchart` keyword after syntax cleanup.
- Detection:
  - Preview error changed to "No diagram type detected," indicating diagram-type parsing mismatch.
- Fix:
  - Migrated to `graph TD;` format with semicolons and legacy edge-label syntax.
- Preventive rule:
  - When Mermaid errors shift from token parse errors to "no diagram type," downgrade diagram keyword/version features first.

#### User Preferences

- Source: `[user]`
- New or reinforced preference:
  - Continue implementing until the visual issue is fully resolved.
- How it changed execution:
  - Performed another direct patch immediately from screenshot evidence.

#### What Worked

- Source: `[self]`
- Pattern/check that worked:
  - Reading the exact renderer error wording helps choose the right compatibility fallback quickly.

#### What Did Not Work

- Source: `[tool]`
- Failed attempt and why:
  - `flowchart` syntax was still incompatible with the active Markdown Mermaid renderer.

#### Guardrails For Next Session

- Preflight rule:
  - Use this fallback order for Mermaid docs issues: content simplification -> legacy `graph TD` -> non-Mermaid fallback text.

#### Follow-Ups / Risks

- Remaining risk or pending check:
  - If parser still fails after `graph TD`, inspect preview extension/settings and treat doc syntax as likely no longer the root cause.

### 2026-02-19 12:13 +11:00 - Ultra-minimal Mermaid syntax fallback

#### Task Summary

- Replaced the IK diagram text with parser-safe token labels (`underscore_style`) to support strict or outdated Mermaid grammars.

#### Mistakes And Fixes

- Source: `[self]`
- Mistake:
  - Previous fallback still used natural-language labels with spaces and optional punctuation.
- Detection:
  - Continued rendering uncertainty after multiple syntax downgrades.
- Fix:
  - Reduced diagram to minimal grammar: `graph TD`, compact operators, simple alphanumeric/underscore labels.
- Preventive rule:
  - For compatibility-sensitive diagrams, start with tokenized labels first; add readability labels only after renderer confirmation.

#### User Preferences

- Source: `[user]`
- New or reinforced preference:
  - Keep iterating with concrete edits until the issue is fully fixed.
- How it changed execution:
  - Implemented one more immediate patch rather than waiting for further prompting.

#### What Worked

- Source: `[self]`
- Pattern/check that worked:
  - Progressive simplification produced a deterministic lowest-common-denominator Mermaid form.

#### What Did Not Work

- Source: `[tool]`
- Failed attempt and why:
  - Richer label styles likely exceeded active renderer compatibility envelope.

#### Guardrails For Next Session

- Preflight rule:
  - Use an explicit compatibility ladder for Mermaid: token labels -> simple labels -> rich labels.

#### Follow-Ups / Risks

- Remaining risk or pending check:
  - If render still fails, request/inspect Markdown preview Mermaid configuration and version support.

### 2026-02-19 12:23 +11:00 - Weld orientation fallback visibility in UI

#### Task Summary

- Added operator-facing weld planning warnings so orientation degradations are visible in the Weld drawer instead of only backend/API logs.

#### Mistakes And Fixes

- Source: `[self]`
- Mistake:
  - Planner warnings were not surfaced in the weld workflow UI, reducing operator awareness of quality-impacting fallbacks.
- Detection:
  - User asked for explicit explanation and requested these warnings be brought into the UI because orientation quality matters.
- Fix:
  - Threaded `planning_warnings` through `previewFromPlannerPayload` into `PreviewPlan` and rendered a dedicated warning section in `WeldPanel`.
- Preventive rule:
  - When backend introduces degraded-but-successful modes, always expose them as explicit operator warnings in the related panel.

#### User Preferences

- Source: `[user]`
- New or reinforced preference:
  - Explain behavior deeply, define terms clearly, and implement requested UX changes immediately.
- How it changed execution:
  - Combined detailed technical explanation with a direct UI implementation in the same turn.

#### What Worked

- Source: `[tool]`
- Pattern/check that worked:
  - Running `npm run build` plus scoped `ReadLints` after UI edits quickly verified the warning pipeline change.

#### What Did Not Work

- Source: `[tool]`
- Failed attempt and why:
  - N/A for this task.

#### Guardrails For Next Session

- Preflight rule:
  - Inspect planner payload contracts (`planning_warnings`, diagnostics fields) before assuming the UI currently surfaces them.

#### Follow-Ups / Risks

- Remaining risk or pending check:
  - Consider a secondary global notification (toast/monitor banner) if warnings must remain visible outside the Weld panel context.

### 2026-02-19 12:24 +11:00 - End-to-end warning plumbing check

#### Task Summary

- Ensured torch-angle fallback warnings are emitted in planner payloads, not only printed in backend logs.

#### Mistakes And Fixes

- Source: `[self]`
- Mistake:
  - Initially wired UI rendering for `planning_warnings` before confirming that torch-angle fallback warnings were actually appended to that payload list.
- Detection:
  - Code review of `command_api.py` showed the fallback warning was printed but not stored in `planning_warnings`.
- Fix:
  - Added `planning_warnings.append(...)` for torch-angle fallback retries in weld section planning.
- Preventive rule:
  - For any UI surfacing task, verify producer and consumer both handle the same event key end-to-end.

#### User Preferences

- Source: `[user]`
- New or reinforced preference:
  - Welding-orientation quality signals must be visible to operators in UI, not hidden in backend logs.
- How it changed execution:
  - Added backend event propagation in the same pass instead of stopping at UI-only presentation.

#### What Worked

- Source: `[tool]`
- Pattern/check that worked:
  - Targeted regression run (`tests/test_api_endpoints.py` + `tests/test_planning.py`) validated no behavioral breakage from warning-path changes.

#### What Did Not Work

- Source: `[self]`
- Failed attempt and why:
  - UI-only warning integration was incomplete until backend payload append was added.

#### Guardrails For Next Session

- Preflight rule:
  - When exposing diagnostics in UI, confirm log-only warnings are also serialized in API responses.

#### Follow-Ups / Risks

- Remaining risk or pending check:
  - If warning volume grows, switch to structured warning objects (`code`, `message`, `section`) for cleaner UI filtering.

### 2026-02-19 13:08 +11:00 - Gradient-05 production switching rollout

#### Task Summary

- Completed end-to-end implementation for restart-required robot switching with robot-config-derived solver policy and supervisor-oriented restart flow.

#### Mistakes And Fixes

- Source: `[self]`
- Mistake:
  - Initially wired `POST /control/restart-controller` using positional arguments into `_controller_call_or_503`, which has keyword-only parameters.
- Detection:
  - Code review during API integration showed this would raise a runtime `TypeError`.
- Fix:
  - Switched to keyword arguments (`timeout=...`, `expect_response=...`) in the threadpool call.
- Preventive rule:
  - Re-check helper signatures when wrapping threadpool calls, especially if `*` keyword-only parameters are used.

#### User Preferences

- Source: `[user]`
- New or reinforced preference:
  - Robot solver selection must come from robot config policy; UI can stage runtime config but not pick arbitrary solver in normal mode.
- How it changed execution:
  - Implemented policy-driven resolver first, then gated overrides behind explicit unsafe flags and surfaced override source in API/UI.

#### What Worked

- Source: `[tool]`
- Pattern/check that worked:
  - Shared `runtime_config.py` module for both controller and API kept desired-vs-active semantics consistent and simplified endpoint logic.
  - Running backend pytest matrix plus `npm run build` after each major integration pass caught issues quickly.

#### What Did Not Work

- Source: `[self]`
- Failed attempt and why:
  - Treating runtime desired config as implicit fallback-only made invalid robot selectors silently default instead of failing fast.
- Correction:
  - Added strict validation for PATCH updates so invalid robot selectors return explicit 400 errors.

#### Guardrails For Next Session

- Preflight rule:
  - For restart-required runtime settings, always implement and verify all three pieces together: desired config store, active runtime introspection, and explicit restart request endpoint.
- Preflight rule:
  - When policy can be overridden in dev mode, expose override source (`robot_policy` vs `dev_override`) in both logs and UI.

#### Follow-Ups / Risks

- Remaining risk or pending check:
  - Supervisor behavior (restart-on-exit code policy and backoff settings) must be validated in deployment environment to complete industrial hardening.

### 2026-02-19 13:18 +11:00 - Python environment activation guardrail

#### Task Summary

- Added a strict execution preference for Python-driven work: always verify the environment is active first; if not active, prefer activating via `start.sh`.

#### Mistakes And Fixes

- Source: `[user]`
- Mistake:
  - Environment activation could be assumed in ad-hoc Python command contexts.
- Detection:
  - User explicitly requested a hard rule to enforce environment activation and use `start.sh` as the preferred recovery path.
- Fix:
  - Recorded the preflight rule in persistent scratchpad guidance.
- Preventive rule:
  - Before any direct Python command, confirm env activation status; if uncertain/inactive, initialize through `start.sh` before proceeding.

#### User Preferences

- Source: `[user]`
- New or reinforced preference:
  - For Python-driven workflows, do not proceed without an active environment; default remediation path is `start.sh`.
- How it changed execution:
  - Added explicit environment preflight checks to the agent's operational guardrails.

#### What Worked

- Source: `[self]`
- Pattern/check that worked:
  - Converting the instruction into a persistent scratchpad rule prevents future regressions across sessions.

#### What Did Not Work

- Source: `[tool]`
- Failed attempt and why:
  - N/A for this process-only update.

#### Guardrails For Next Session

- Preflight rule:
  - Treat environment activation as mandatory for all Python-driven steps; if not active, run via `start.sh` first.

#### Follow-Ups / Risks

- Remaining risk or pending check:
  - Keep launch instructions aligned between shell variants (`start.sh` and Windows launcher scripts) so the activation path is unambiguous.

### 2026-02-19 13:29 +11:00 - Launcher bootstrap enforcement

#### Task Summary

- Implemented environment-activation preflight directly in backend/controller launch scripts so startup paths are robust even when invoked from UI-driven or external orchestration flows.

#### Mistakes And Fixes

- Source: `[tool]`
- Mistake:
  - Tried using `bash -n` validation on a Windows host where `bash` is unavailable on PATH.
- Detection:
  - Command errors showed `bash` command not recognized.
- Fix:
  - Validated PowerShell launcher behavior with `--help` smoke runs and recorded the bash-validation gap explicitly.
- Preventive rule:
  - On Windows, check shell availability before scheduling bash-specific validation steps.

#### User Preferences

- Source: `[user]`
- New or reinforced preference:
  - If Web UI (or any orchestrator) is used to start controller/backend processes, environment activation must happen first so required packages are always accessible.
- How it changed execution:
  - Added explicit startup bootstrap logic to launchers instead of relying on implicit active-shell state.

#### What Worked

- Source: `[self]`
- Pattern/check that worked:
  - Centralizing activation checks in launch scripts protects both manual and automated startup paths without requiring extra operator steps.

#### What Did Not Work

- Source: `[tool]`
- Failed attempt and why:
  - Full bash syntax linting could not run on this machine due missing `bash`.

#### Guardrails For Next Session

- Preflight rule:
  - For startup reliability tasks, validate launch scripts in the native host shell first, then run cross-shell checks only when that shell runtime exists.

#### Follow-Ups / Risks

- Remaining risk or pending check:
  - If you want strict Linux-side verification, run `bash -n run.sh run-sim.sh run-api.sh` on a host with bash/WSL.

### 2026-02-19 13:36 +11:00 - API startup warning root-cause fix

#### Task Summary

- Removed `runpy` warning during API startup by eliminating eager `main` import side effects in `gradient_os.api` package init.

#### Mistakes And Fixes

- Source: `[self]`
- Mistake:
  - Package `src/gradient_os/api/__init__.py` imported `.main` eagerly, which caused `gradient_os.api.main` to already exist in `sys.modules` before `python -m gradient_os.api.main` executed it.
- Detection:
  - Warning text in startup logs explicitly indicated preloaded module before runpy execution.
- Fix:
  - Converted package exports to lazy-loading (`create_app` proxy + `__getattr__` for `app`) so package import is side-effect free.
- Preventive rule:
  - Avoid importing runnable `__main__` targets from package `__init__` files when those modules are launched via `-m`.

#### User Preferences

- Source: `[user]`
- New or reinforced preference:
  - Treat startup warnings as robustness defects and investigate to root cause.
- How it changed execution:
  - Implemented warning removal, not just explanation, and validated with launcher smoke tests.

#### What Worked

- Source: `[tool]`
- Pattern/check that worked:
  - Re-running `run-api.ps1 --help` immediately after patch gave a fast, deterministic confirmation that warning was removed.

#### What Did Not Work

- Source: `[tool]`
- Failed attempt and why:
  - N/A for this change.

#### Guardrails For Next Session

- Preflight rule:
  - For `python -m package.module` entrypoints, inspect package `__init__.py` for eager imports that can preload the target module.

#### Follow-Ups / Risks

- Remaining risk or pending check:
  - Accessing lazy `app` still imports `main` intentionally; startup logs from backend registry/IK initialization at that moment are expected behavior.

### 2026-02-19 13:44 +11:00 - UDP reset and command-link health hardening

#### Task Summary

- Improved controller robustness for Windows UDP behavior and made command-link loss visible in telemetry.

#### Mistakes And Fixes

- Source: `[self]`
- Mistake:
  - Connection-reset events from UDP were treated as generic unexpected errors, generating scary traceback logs despite non-fatal behavior.
- Detection:
  - Sim terminal showed `ConnectionResetError [WinError 10054]` during active operation, followed by normal command processing.
- Fix:
  - Added explicit handling for `winerror=10054` as benign datagram reset noise with throttled warning and continue.
  - Added command silence tracking and stale-link warning/restore logging.
  - Added comms-health telemetry fields for operator visibility.
- Preventive rule:
  - For UDP controllers on Windows, classify known socket reset codes before falling through to generic exception handling.

#### User Preferences

- Source: `[user]`
- New or reinforced preference:
  - Robust software means warnings should be investigated and real connection-loss risk should be addressed, not dismissed.
- How it changed execution:
  - Implemented both noise suppression and actual link-health observability in the same patch.

#### What Worked

- Source: `[tool]`
- Pattern/check that worked:
  - Fast syntax compile plus launcher smoke run gave quick confidence that comms hardening didn’t break startup behavior.

#### What Did Not Work

- Source: `[tool]`
- Failed attempt and why:
  - N/A for this change.

#### Guardrails For Next Session

- Preflight rule:
  - Distinguish transport-noise handling from true link-health detection; implement both when hardening comms paths.

#### Follow-Ups / Risks

- Remaining risk or pending check:
  - If strict command delivery guarantees are required, layer heartbeat/ack/retry semantics above UDP instead of relying on socket-level behavior.

### 2026-02-19 13:53 +11:00 - Weld entry IK no-solution mitigation

#### Task Summary

- Addressed weld-entry planning failures by removing unnecessary orientation-lock rigidity during the transition into the first weld point.

#### Mistakes And Fixes

- Source: `[self]`
- Mistake:
  - Weld entry path used `_plan_linear_move(..., forced_orientation=None)`, which implicitly locks orientation to current pose and can make an otherwise reachable entry point infeasible.
- Detection:
  - Logs showed `IK_NO_SOLUTION` on dense sequential fallback during weld entry even for a nearby, expected-reachable start point.
- Fix:
  - Derived an entry orientation from `_build_weld_orientations(...)` and planned entry with interpolated orientation toward that target.
  - Added deterministic fallback back to legacy orientation-lock if interpolated entry fails.
  - Added warning plumbing so fallback use is visible.
- Preventive rule:
  - For transition-to-process segments (e.g., weld entry), avoid defaulting to strict start-pose orientation lock when process orientation is known.

#### User Preferences

- Source: `[user]`
- New or reinforced preference:
  - If a path should be solvable in practice, planning should degrade gracefully before hard-failing with `IK_NO_SOLUTION`.
- How it changed execution:
  - Implemented entry-specific fallback ladder instead of leaving single rigid entry orientation behavior.

#### What Worked

- Source: `[tool]`
- Pattern/check that worked:
  - Reusing existing weld orientation builder allowed a minimal targeted fix without changing solver internals.

#### What Did Not Work

- Source: `[tool]`
- Failed attempt and why:
  - N/A for this change.

#### Guardrails For Next Session

- Preflight rule:
  - When diagnosing `IK_NO_SOLUTION`, inspect whether failures occur on approach/transition segments with mismatched orientation constraints before changing global IK settings.

#### Follow-Ups / Risks

- Remaining risk or pending check:
  - If edge geometry and process angle still conflict at entry, add a bounded intermediate waypoint strategy as the next fallback stage.

### 2026-02-19 14:02 +11:00 - API planner runtime alignment

#### Task Summary

- Corrected weld planning failures caused by API-process planner runtime drift from controller runtime (robot/solver mismatch).

#### Mistakes And Fixes

- Source: `[self]`
- Mistake:
  - Weld planning (`/trajectory/plan-weld`) executed through API-process `command_api` without ensuring local planner robot/IK config matched active controller runtime.
- Detection:
  - Terminal evidence showed controller pose at ~`[0.807, 0.0, 0.91]` while failing dense path points started around `[0.489, 0.0, 0.37]`, indicating mismatched kinematic model/backend during planning.
- Fix:
  - Added pre-plan runtime sync in API weld planning path using controller `GET_RUNTIME_CONFIG`, then set active robot + IK backend in API-process planner modules before generating weld path.
- Secondary issue:
  - Introduced `NameError` by calling the scoped helper from module-level preview planner.
- Secondary fix:
  - Removed preview-path call to scoped helper; kept sync where needed for weld planning.
- Preventive rule:
  - When planning can run in multiple processes, always align robot/solver runtime explicitly before planning; do not rely on startup defaults.

#### User Preferences

- Source: `[user]`
- New or reinforced preference:
  - A path that should be reachable must not fail due to configuration drift or hidden runtime mismatch.
- How it changed execution:
  - Prioritized runtime-consistency fixes over tuning IK thresholds.

#### What Worked

- Source: `[tool]`
- Pattern/check that worked:
  - Cross-checking API logs against controller telemetry/pose made process-mismatch diagnosis fast and concrete.

#### What Did Not Work

- Source: `[self]`
- Failed attempt and why:
  - Initial helper placement created scope mismatch for module-level preview function.

#### Guardrails For Next Session

- Preflight rule:
  - For API endpoints invoking local planning modules, verify they consume the same robot/solver runtime as controller before trajectory generation.

#### Follow-Ups / Risks

- Remaining risk or pending check:
  - If weld planning still fails after runtime sync + entry fallback, add intermediate entry waypoint fallback with bounded clearance.

### 2026-02-19 21:57 +11:00 - Speed multiplier felt ineffective

#### Task Summary

- Fixed motion speed scaling so UI speed multiplier changes are observable for short profile moves, not only long cruise segments.

#### Mistakes And Fixes

- Source: `[self]`
- Mistake:
  - Relative/profiled move code scaled only profile velocity while keeping acceleration constant.
- Detection:
  - User reported that speed multiplier "seems to do nothing"; code trace showed `speed_multiplier` forwarded correctly but acceleration remained fixed (`DEFAULT_PROFILE_ACCELERATION`), which keeps many short moves acceleration-limited.
- Fix:
  - Added `_resolve_profile_params_for_speed_multiplier(...)` in `command_api.py`.
  - Scaled velocity by `speed_mult` and acceleration by `speed_mult^2`.
  - Applied this logic to relative move handlers and trajectory planning move branches.
- Preventive rule:
  - When exposing a UI speed multiplier for motion profiles, verify both velocity and acceleration scaling, otherwise short moves can appear unaffected.

#### User Preferences

- Source: `[user]`
- New or reinforced preference:
  - If behavior is wrong, implement the fix directly and explain what changed (not just analysis).
- How it changed execution:
  - Delivered code change + tests in one pass and provided concrete root-cause explanation.

#### What Worked

- Source: `[tool]`
- Pattern/check that worked:
  - End-to-end path tracing (UI payload -> API command -> controller parser -> planner params) quickly isolated where multiplier influence dropped.

#### What Did Not Work

- Source: `[tool]`
- Failed attempt and why:
  - N/A for this task.

#### Guardrails For Next Session

- Preflight rule:
  - For "slider has no effect" reports, inspect unit conversions and whether the scaled parameter is dominant in the active motion regime (acceleration-limited vs velocity-limited).

#### Follow-Ups / Risks

- Remaining risk or pending check:
  - Very high multipliers can reduce planned sample count and feel abrupt; consider optional runtime cap tuning per robot profile if operators report jerkiness.

### 2026-02-19 22:14 +11:00 - Sim backend identity mismatch

#### Task Summary

- Corrected simulation startup so backend config identity remains `simulation` (instead of logging/setting `feetech`) while keeping telemetry/config compatibility.

#### Mistakes And Fixes

- Source: `[self]`
- Mistake:
  - `run_controller` forced `config_backend = "feetech"` whenever servo backend was `simulation`.
- Detection:
  - User pointed out startup line: `[Backend Registry] Active servo backend set to: feetech` during `run-sim.ps1`.
- Fix:
  - Added `backends/simulation/config.py` and registered simulation with its own config module path.
  - Changed controller startup to set active config backend directly from resolved `servo_backend`.
- Preventive rule:
  - For each runtime backend, keep registry identity and config identity aligned; use compatibility aliases inside backend-specific config modules instead of swapping names at call sites.

#### User Preferences

- Source: `[user]`
- New or reinforced preference:
  - In sim mode, software state/logs must accurately reflect simulation backend selection (no misleading hardware-backend labels).
- How it changed execution:
  - Implemented the fix immediately and validated backend activation output with a direct runtime check.

#### What Worked

- Source: `[tool]`
- Pattern/check that worked:
  - Direct `registry.set_active_backend("simulation")` smoke test quickly validated both naming and constant resolution after patch.

#### What Did Not Work

- Source: `[tool]`
- Failed attempt and why:
  - N/A for this task.

#### Guardrails For Next Session

- Preflight rule:
  - When reviewing startup logs, treat backend-name mismatches as correctness issues (not cosmetic), because they can mislead diagnostics and safety triage.

#### Follow-Ups / Risks

- Remaining risk or pending check:
  - Keep simulation config compatibility in sync with hardware config expectations if telemetry schema evolves.

### 2026-02-19 22:17 +11:00 - Realtime jog slider stale callback

#### Task Summary

- Fixed realtime jog so speed multiplier changes take effect immediately during active jog sessions.

#### Mistakes And Fixes

- Source: `[self]`
- Mistake:
  - Realtime jog interval captured `sendJogTick` at jog-start time, so slider/deadman/base changes could be stale while interval kept running.
- Detection:
  - User reported speed multiplier worked for incremental jogs but not realtime jog controls; code inspection showed `setInterval(() => sendJogTick(), ...)` with a closure-bound callback.
- Fix:
  - Added `sendJogTickRef` and `useEffect` to keep interval callback pointing at the latest `sendJogTick`.
  - Interval now executes `sendJogTickRef.current()` so updated speed multiplier/base values are always used.
- Preventive rule:
  - For long-running timers in React that depend on frequently changing state, use a callback ref (or explicit timer restart logic) to avoid stale-closure control bugs.

#### User Preferences

- Source: `[user]`
- New or reinforced preference:
  - Validate behavior in the actual interaction mode (realtime jog) and fix the real flow, not just adjacent features.
- How it changed execution:
  - Traced and patched the realtime client loop directly in UI instead of further tuning backend profile math.

#### What Worked

- Source: `[tool]`
- Pattern/check that worked:
  - Path split between incremental and realtime control handlers quickly isolated the bug to the frontend interval lifecycle.

#### What Did Not Work

- Source: `[tool]`
- Failed attempt and why:
  - N/A for this task.

#### Guardrails For Next Session

- Preflight rule:
  - When a control appears to apply only after restart/toggle, inspect React hook closure lifetimes before changing transport/backend logic.

#### Follow-Ups / Risks

- Remaining risk or pending check:
  - If operators still perceive plateauing at high multipliers, check backend jog caps and expose cap telemetry in UI for clarity.

### 2026-02-19 23:12 +11:00 - Tool library + TIG semantics

#### Task Summary

- Implemented global tool library support (backend/API/UI/visualizer) and moved weld angle interpretation to torch-target semantics compensated by active tool definition.

#### Mistakes And Fixes

- Source: `[self]`
- Mistake:
  - Initial restart-required logic treated missing `active.tool` in legacy runtime payloads as a mismatch and incorrectly flagged restart required.
- Detection:
  - `tests/test_api_endpoints.py::test_runtime_config_get_and_patch` failed (`restart_required` unexpectedly `True`).
- Fix:
  - Updated `runtime_config.compute_restart_required(...)` to only compare tool ids when active runtime payload includes a tool id.
- Preventive rule:
  - When extending active runtime payload contracts, keep backward compatibility for older controller payload shapes in restart/health predicates.

#### User Preferences

- Source: `[user]`
- New or reinforced preference:
  - Tool definitions should drive weld-angle behavior (not static conversion hacks), and tool management must be practical in UI with filtering and quick swapping.
- How it changed execution:
  - Implemented explicit active-tool runtime policy, filterable global tool library, and weld orientation compensation tied to selected tool metadata.

#### What Worked

- Source: `[tool]`
- Pattern/check that worked:
  - Building the feature around existing runtime kinematics offset layering (base/tool runtime) enabled minimal invasive IK integration while keeping deterministic restart-required policy flow.

#### What Did Not Work

- Source: `[tool]`
- Failed attempt and why:
  - N/A for this task.

#### Guardrails For Next Session

- Preflight rule:
  - For any new geometry/tool abstraction, update both planner semantics and visualization in the same pass to avoid UI/runtime mismatch during operator validation.

#### Follow-Ups / Risks

- Remaining risk or pending check:
  - TIG template currently has 65 deg rotation with zero XYZ placeholder; replace with metrology-calibrated offsets before production weld quality validation.

### 2026-02-19 23:47 +11:00 - Folder-per-tool library layout

#### Task Summary

- Refactored tool storage to per-tool folders so dropping `tools/library/<tool_id>/tool.json` (+ local mesh file) is enough for backend + UI asset sync discovery.

#### Mistakes And Fixes

- Source: `[self]`
- Mistake:
  - Initial loader fallback logic always re-added `tig-torch-65deg` when missing, which would have prevented intentional deletion of TIG from a customized library.
- Detection:
  - Code review after refactor showed fallback behavior was too aggressive for non-empty libraries.
- Fix:
  - Changed fallback policy to seed both default tools only for an empty library; for non-empty libraries, only enforce identity fallback.
- Preventive rule:
  - When seeding defaults in discovery-based storage, distinguish bootstrap behavior (empty state) from steady-state behavior (operator-managed set).

#### User Preferences

- Source: `[user]`
- New or reinforced preference:
  - Tool definitions must be self-contained and drop-in, with definition + STL colocated in each tool folder.
- How it changed execution:
  - Replaced monolithic `tool_library.json` design with folder discovery, metadata sidecar, and sync logic that reads each `tool.json`.

#### What Worked

- Source: `[tool]`
- Pattern/check that worked:
  - Keeping `GRADIENT_TOOL_LIBRARY_PATH` as a backward-compatible env var while interpreting `.json` values as legacy paths allowed migration without breaking existing tests/mocks.

#### What Did Not Work

- Source: `[tool]`
- Failed attempt and why:
  - N/A for this task.

#### Guardrails For Next Session

- Preflight rule:
  - For library/file-layout refactors, update both runtime loaders and build-time sync scripts in the same change to avoid backend/UI asset drift.

#### Follow-Ups / Risks

- Remaining risk or pending check:
  - Local mesh autodetect is currently root-folder only; nested mesh assets need explicit `mesh.asset_path` until recursive detection is added.

### 2026-02-20 00:41 +11:00 - Tool drawer integration

#### Task Summary

- Added a dedicated `Tool Library` drawer in the left sidebar so operators can load/select tools in the same workflow lane as STEP/Trajectory/Weld panels.

#### Mistakes And Fixes

- Source: `[self]`
- Mistake:
  - Tool library/runtime fetch triggers were previously scoped to Settings dialog only.
- Detection:
  - UI flow review showed tool drawer would open stale/empty unless Settings was opened first.
- Fix:
  - Runtime and tool-library fetch effects now trigger when either Settings is open or `activePanel === "tools"`.
- Preventive rule:
  - For any feature moved from modal settings into primary workflow drawers, re-check data-fetch lifecycle triggers and not just visual component placement.

#### User Preferences

- Source: `[user]`
- New or reinforced preference:
  - Tool library loading should live in the left main menu flow, not buried in Settings.
- How it changed execution:
  - Added sidebar `Tool Library` panel with quick filter/select/stage/restart actions and kept Settings as a full editor path.

#### What Worked

- Source: `[tool]`
- Pattern/check that worked:
  - Reusing existing runtime-config handlers (`handleApplyRuntimeConfig`, `handleRestartController`) avoided divergent behavior between Settings and drawer flows.

#### What Did Not Work

- Source: `[tool]`
- Failed attempt and why:
  - N/A for this task.

#### Guardrails For Next Session

- Preflight rule:
  - When adding new sidebar drawer modes, update `SidebarPanelId`, persisted panel parsing, keyboard map, and drawer header/content switch together to avoid hidden dead states.

#### Follow-Ups / Risks

- Remaining risk or pending check:
  - Tool CRUD UI still lives in Settings; if operators need full create/edit in-drawer, extract shared editor component instead of duplicating forms.

### 2026-02-20 00:48 +11:00 - Drawer fit-to-content behavior

#### Task Summary

- Adjusted Tool Library and Weld drawers to avoid forced full-height stretching when content is short, while preserving internal scrolling when content exceeds lane height.

#### Mistakes And Fixes

- Source: `[self]`
- Mistake:
  - Previous iteration forced `heightMode="full"` for Weld/Tool Library to stabilize baseline, which conflicted with user preference for non-stretched sparse panels.
- Detection:
  - User screenshot/feedback showed unwanted empty vertical space down to drawer bottom.
- Fix:
  - Switched active drawer mode back to content-fit (`max-h-full`) universally and updated the design rule.
- Preventive rule:
  - Treat “fit-content-until-overflow” as the default drawer height contract unless user explicitly asks for fixed full-height.

#### User Preferences

- Source: `[user]`
- New or reinforced preference:
  - Drawer should not stretch when content is short; if content is long it should cap and scroll in-place.
- How it changed execution:
  - Unified drawer height mode across Tool Library and Weld to content-fit with max-lane internal scrolling.

#### What Worked

- Source: `[tool]`
- Pattern/check that worked:
  - Existing `SidebarDrawer` structure (`max-h-full` shell + internal `overflow-y-auto` body) already supports desired behavior; only panel mode selection needed adjustment.

#### What Did Not Work

- Source: `[tool]`
- Failed attempt and why:
  - Prior dense-mode `h-full` strategy improved alignment but violated fit-content UX expectation.

#### Guardrails For Next Session

- Preflight rule:
  - Validate drawer height decisions against both sparse and dense panel screenshots before locking design rules.

#### Follow-Ups / Risks

- Remaining risk or pending check:
  - If future panel content introduces sticky footers, re-test content-fit + scroll interaction to avoid clipped controls.

### 2026-02-20 00:53 +11:00 - Tool editor tabbed settings flow

#### Task Summary

- Changed Tool Library drawer “Open Full Tool Editor” behavior to open a focused tool-settings tab in Settings, and split Settings into tabs to reduce on-screen overload.

#### Mistakes And Fixes

- Source: `[self]`
- Mistake:
  - Earlier implementation opened Settings directly without context, showing too many unrelated controls at once.
- Detection:
  - User feedback and screenshot indicated high visual density and wrong navigation target for the drawer action.
- Fix:
  - Added settings tab model (`General`, `Tool Library`, `Kinematics`) and an `initialTab` flow from caller context.
- Preventive rule:
  - Any cross-entry “open editor” CTA should deep-link into the relevant settings section/tab rather than opening global settings defaults.

#### User Preferences

- Source: `[user]`
- New or reinforced preference:
  - “Open full library” must show tool parameters only, and tool settings should live on their own tab in Settings.
- How it changed execution:
  - Introduced context-aware settings open behavior (`tools` from drawer, `general` from header).

#### What Worked

- Source: `[tool]`
- Pattern/check that worked:
  - Adding an `initialTab` prop to `SettingsDialog` provided a low-risk way to support deep-linking without creating an entirely separate modal codepath.

#### What Did Not Work

- Source: `[tool]`
- Failed attempt and why:
  - N/A for this task.

#### Guardrails For Next Session

- Preflight rule:
  - For large settings dialogs, enforce tab segregation before adding new major feature controls to prevent UI clutter regressions.

#### Follow-Ups / Risks

- Remaining risk or pending check:
  - If stakeholders want a truly standalone tool modal later, extract Tool Library tab content into a shared component to avoid duplicating editor logic.

### 2026-02-20 01:02 +11:00 - Settings modal constraints + mesh transform schema

#### Task Summary

- Applied bounded-height + internal-scroll behavior to Settings modal and added separate mesh visual transform fields (`mesh.position_mm`, `mesh.rotation_deg`) so mesh origin does not have to coincide with TCP/tool-tip.

#### Mistakes And Fixes

- Source: `[self]`
- Mistake:
  - Initial Settings modal refactor introduced mismatched JSX closing tags and broke `npm run build`.
- Detection:
  - Vite/esbuild parse errors at `web-ui/src/App.tsx` with invalid JSX closing sequence.
- Fix:
  - Re-counted modal wrapper hierarchy and corrected final closing `</div>` count.
- Preventive rule:
  - After structural JSX wrapper changes, validate opening/closing div counts before running full build to avoid avoidable parse churn.

#### User Preferences

- Source: `[user]`
- New or reinforced preference:
  - Settings modal should follow same cap-and-scroll behavior as side drawers.
  - Tool config must separately define TCP offset/orientation vs mesh visual offset/orientation.
- How it changed execution:
  - Settings layout now uses fixed header/tab with internal scroll body, and tool schema/UI/visualizer now expose dedicated mesh transform controls.

#### What Worked

- Source: `[tool]`
- Pattern/check that worked:
  - Extending normalization at backend (`tool_library.py`) first enabled backward-compatible schema migration (mesh string shorthand + richer object) without breaking existing tool files.

#### What Did Not Work

- Source: `[tool]`
- Failed attempt and why:
  - One build cycle failed due a JSX wrapper mismatch after modal refactor.

#### Guardrails For Next Session

- Preflight rule:
  - When introducing new geometry fields, update all three layers in one pass: backend normalization, UI editor fields, and runtime visualizer application.

#### Follow-Ups / Risks

- Remaining risk or pending check:
  - Mesh transform defaults are zeroed; CAD-origin-specific offsets/rotations should be measured and filled for each production tool mesh.

### 2026-02-20 01:16 +11:00 - Mesh load diagnosis and J6-relative visual frame

#### Task Summary

- Diagnosed why TIG mesh is not showing and updated visual semantics so mesh transform is J6/flange-relative while TCP offset remains independent.

#### Mistakes And Fixes

- Source: `[self]`
- Mistake:
  - Earlier visualizer applied mesh transform under the TCP offset group, coupling mesh placement to tool-tip frame.
- Detection:
  - User requirement explicitly asked for mesh zero point at J6, not tool-tip.
- Fix:
  - Refactored visualizer hierarchy: J6/flange anchor root + TCP subgroup; mesh now applies transform on anchor root.
- Preventive rule:
  - Separate kinematic TCP transforms from visual mesh transforms in scene graph hierarchy to avoid frame-coupling regressions.

#### User Preferences

- Source: `[user]`
- New or reinforced preference:
  - Mesh origin/zero should be relative to J6 anchor, and tool-tip offset semantics should remain separate.
- How it changed execution:
  - Updated UI labels/docs and runtime visualizer transform frame semantics accordingly.

#### What Worked

- Source: `[tool]`
- Pattern/check that worked:
  - Verifying on-disk tool folder contents (`tools/library/tig-torch-65deg`) quickly identified root cause: missing STL file for configured path.

#### What Did Not Work

- Source: `[tool]`
- Failed attempt and why:
  - N/A for this task.

#### Guardrails For Next Session

- Preflight rule:
  - When mesh fails to render, first validate `tool.json mesh.asset_path` against actual files in the tool folder before changing rendering code.

#### Follow-Ups / Risks

- Remaining risk or pending check:
  - TIG tool will keep showing fallback marker until `tool_mesh.stl` (or whichever file name is configured) is present in `tools/library/tig-torch-65deg/`.

### 2026-02-20 01:19 +11:00 - Corrected TIG mesh file state

#### Task Summary

- Confirmed user was correct: `tool_mesh.stl` exists for TIG tool; prior “missing file” diagnosis became stale.

#### Mistakes And Fixes

- Source: `[self]`
- Mistake:
  - Reported mesh file missing based on earlier filesystem snapshot.
- Detection:
  - User escalation + direct `ls` verification showed file present in `tools/library/tig-torch-65deg/`.
- Fix:
  - Revalidated live disk state, ran `npm run sync:tool-assets`, and confirmed file copied into `web-ui/public/assets/tools/tig-torch-65deg/`.
- Preventive rule:
  - When user disputes file existence, re-check live filesystem immediately before asserting diagnostics.

#### User Preferences

- Source: `[user]`
- New or reinforced preference:
  - Direct, accurate file-state checks over assumptions.
- How it changed execution:
  - Added immediate filesystem + sync verification pass and reported concrete paths.

#### What Worked

- Source: `[tool]`
- Pattern/check that worked:
  - Fast triage via `ls` on source folder + synced public folder + reading `index.json` confirms entire asset path chain.

#### What Did Not Work

- Source: `[tool]`
- Failed attempt and why:
  - Earlier absence check was outdated once file was added.

#### Guardrails For Next Session

- Preflight rule:
  - For asset-loading issues, always validate source file, synced public file, and generated asset index in one checklist.

#### Follow-Ups / Risks

- Remaining risk or pending check:
  - Browser tab/dev server may need refresh/restart to pick up newly synced static assets.

### 2026-02-20 01:32 +11:00 - Live tool apply (no restart) + UDP reset hardening

#### Task Summary

- Implemented live active-tool apply path and removed tool-only restart gating; fixed API UDP receive error handling that previously escalated controller restart churn into 500s.

#### Mistakes And Fixes

- Source: `[self]`
- Mistake:
  - Tool change flow was still restart-gated despite user requirement for operational hot-swap behavior.
- Detection:
  - User reported clicking Apply+Restart and called out that tool changes should not require controller restart.
- Fix:
  - Added controller command `SET_ACTIVE_TOOL` + API live-apply call on runtime-config patch; removed tool mismatch from restart predicate.
- Preventive rule:
  - Treat “hot-swappable runtime policy” fields separately from startup-only policy fields in restart calculations.

#### User Preferences

- Source: `[user]`
- New or reinforced preference:
  - Active tool changes should apply immediately without restart.
- How it changed execution:
  - Runtime config patch now performs live tool apply and keeps restart requirement for robot/backend policy changes only.

#### What Worked

- Source: `[tool]`
- Pattern/check that worked:
  - Extending existing fake controller command harness in API tests (`patch_send`) made it straightforward to validate the live apply command and restart_required behavior.

#### What Did Not Work

- Source: `[tool]`
- Failed attempt and why:
  - N/A for this task.

#### Guardrails For Next Session

- Preflight rule:
  - For UDP controller calls in API paths, always guard both send and receive with socket error handling; never allow transport resets to bubble as uncaught exceptions.

#### Follow-Ups / Risks

- Remaining risk or pending check:
  - Live tool apply currently waits for idle via existing blocking wait path; if future UX needs asynchronous/non-blocking apply queues, introduce explicit pending-state telemetry.

### 2026-02-20 01:36 +11:00 - Ground plane pinned back to robot base

#### Task Summary

- Fixed a 3D visualizer regression where the floor reference no longer aligned with the robot's base after tool-visual integration changes.

#### Mistakes And Fixes

- Source: `[self]`
- Mistake:
  - Grounding used a box that could include active tool visuals, letting tool mesh origin/offset influence base-floor alignment.
- Detection:
  - User reported that the ground plane was no longer at the bottom of the robot.
- Fix:
  - Updated grounding path in `web-ui/src/ArmVisualizer.tsx` to temporarily hide `activeToolGroup` while computing base bounds.
- Preventive rule:
  - Never allow end-effector visual assets to participate in floor/base grounding calculations; ground from robot base geometry only.

#### User Preferences

- Source: `[user]`
- New or reinforced preference:
  - Ground plane must remain anchored to the bottom of the robot base consistently.
- How it changed execution:
  - Added guardrails in grounding logic so tool visuals cannot perturb floor placement.

#### What Worked

- Source: `[tool]`
- Pattern/check that worked:
  - A focused patch plus `npm run build` and lint check validated the fix quickly without touching unrelated planner/runtime paths.

#### What Did Not Work

- Source: `[tool]`
- Failed attempt and why:
  - N/A for this task.

#### Guardrails For Next Session

- Preflight rule:
  - For any scene-graph additions under wrist/tool links, re-check whether grounding/bounds logic should exclude those nodes.

#### Follow-Ups / Risks

- Remaining risk or pending check:
  - If any specific robot still appears offset, inspect that robot CAD base mesh origin in the URDF asset bundle.

### 2026-02-20 01:48 +11:00 - Hide fallback marker when real tool mesh is loaded

#### Task Summary

- Fixed duplicated/confusing tool visuals by stopping fallback TCP marker from rendering when the actual STL tool mesh loads successfully.

#### Mistakes And Fixes

- Source: `[self]`
- Mistake:
  - Fallback marker was always attached, even when the real tool mesh was present, creating a second "ghost tool" in scene.
- Detection:
  - User screenshot showed detached orange marker while blue TIG mesh was already loaded at the wrist.
- Fix:
  - Added `fallbackMarker` visibility logic in `web-ui/src/ArmVisualizer.tsx`:
    - default hidden when mesh asset is configured,
    - forced hidden on successful STL load,
    - shown on load failure or unsupported mesh extension.
- Preventive rule:
  - Visual fallbacks should be conditional diagnostics, not always-on overlays when primary asset path succeeds.

#### User Preferences

- Source: `[user]`
- New or reinforced preference:
  - UI should not show confusing duplicate tool geometry after selecting/applying a tool.
- How it changed execution:
  - Implemented cleaner default visualization behavior that prioritizes the actual mesh once available.

#### What Worked

- Source: `[tool]`
- Pattern/check that worked:
  - Fast code-path isolation in `attachActiveToolVisual` + immediate build/lint cycle confirmed fix without touching runtime config APIs.

#### What Did Not Work

- Source: `[tool]`
- Failed attempt and why:
  - N/A for this task.

#### Guardrails For Next Session

- Preflight rule:
  - When adding visual fallback objects, explicitly define success/failure visibility transitions in async loader callbacks.

#### Follow-Ups / Risks

- Remaining risk or pending check:
  - If calibration users want TCP marker always visible, gate it behind a user-controlled debug toggle.

### 2026-02-20 01:51 +11:00 - J6-first anchor for tool mesh placement

#### Task Summary

- Fixed mesh placement frame selection so active tool visuals anchor to J6 joint first instead of being attached to arbitrary tool/wrist links.

#### Mistakes And Fixes

- Source: `[self]`
- Mistake:
  - Anchor resolution prioritized links containing `tool`/`wrist`, which can be offset from J6 and cause placement drift.
- Detection:
  - User reported STL placement still looked off after prior fixes.
- Fix:
  - Updated anchor order in `web-ui/src/ArmVisualizer.tsx` to: `joint6` -> flange link -> tool/wrist link -> robot root.
- Preventive rule:
  - When frame semantics are explicitly "J6-relative", always hard-prioritize `joint6` in scene graph anchor lookup.

#### User Preferences

- Source: `[user]`
- New or reinforced preference:
  - Tool mesh placement must match intended J6-relative frame semantics in runtime visualization.
- How it changed execution:
  - Anchor lookup became deterministic and aligned with requested frame contract.

#### What Worked

- Source: `[tool]`
- Pattern/check that worked:
  - Minimal targeted edit + immediate `npm run build` and lint check quickly validated behavior and safety.

#### What Did Not Work

- Source: `[tool]`
- Failed attempt and why:
  - N/A for this task.

#### Guardrails For Next Session

- Preflight rule:
  - For any robot-agnostic link selection logic, define explicit semantic priority order before using fuzzy name matching.

#### Follow-Ups / Risks

- Remaining risk or pending check:
  - CAD mesh local origin may still require per-tool transform tuning via `mesh.position_mm`/`mesh.rotation_deg`.

### 2026-02-20 01:56 +11:00 - Enforced joint6/child-frame anchoring and cleaned tool0 link

#### Task Summary

- Corrected `gradient-05` tool-frame handling by anchoring tool visuals to `joint6` semantics and removing invalid placeholder mesh geometry from `tool0` in URDF.

#### Mistakes And Fixes

- Source: `[tool]`
- Mistake:
  - URDF contained an invalid placeholder (`<mesh/>`) in `tool0` and visual anchoring logic still used fuzzy name-based fallbacks.
- Detection:
  - User flagged that `tool0` visual reference was not valid and tool placement must reference joint6.
- Fix:
  - Updated `web-ui/src/ArmVisualizer.tsx` to resolve J6 deterministically and prefer the child link of J6 as anchor frame.
  - Updated `robots/gradient-05/gradient-05.urdf` `tool0` link to be frame-only with explanatory comment.
- Preventive rule:
  - For URDF frame links (tool/tcp), avoid placeholder geometry and keep frame semantics explicit in both URDF and scene-graph code.

#### User Preferences

- Source: `[user]`
- New or reinforced preference:
  - Tool placement should be referenced to actual `joint6` transform contract in URDF, not inferred from potentially missing visual assets.
- How it changed execution:
  - Replaced fuzzy matching with deterministic joint-frame lookup and cleaned source URDF frame declaration.

#### What Worked

- Source: `[tool]`
- Pattern/check that worked:
  - Checking filesystem for missing `wrist.stl` and then aligning both URDF + visualizer in one pass removed ambiguity quickly.

#### What Did Not Work

- Source: `[tool]`
- Failed attempt and why:
  - Initial patch attempt on URDF failed due stale context; re-read exact local file block and patched successfully.

#### Guardrails For Next Session

- Preflight rule:
  - When user cites URDF lines, re-read exact current block before patching to avoid context drift errors.

#### Follow-Ups / Risks

- Remaining risk or pending check:
  - Final physical match still depends on per-tool mesh local origin and may require small tool-editor transform tuning.

### 2026-02-20 02:02 +11:00 - Base-only floor grounding (never tool-driven)

#### Task Summary

- Reworked floor grounding math so the robot is grounded from base-link geometry only, preventing tool mesh offsets/origins from moving the floor level.

#### Mistakes And Fixes

- Source: `[self]`
- Mistake:
  - Prior grounding relied on `setFromObject(baseLink)` and temporary visibility toggles, which can still include descendants and let tool geometry influence `min.z`.
- Detection:
  - User reinforced non-negotiable rule: floor/ground plane must be at robot base, never where the tool is.
- Fix:
  - Added `computeGroundingBoxFromBase(...)` in `web-ui/src/ArmVisualizer.tsx` to compute bounds from base-link mesh geometry while stopping traversal at URDF joint nodes and excluding active-tool subtree.
- Preventive rule:
  - For grounding semantics, never use broad descendant bounds of articulated hierarchies; isolate base-frame geometry explicitly.

#### User Preferences

- Source: `[user]`
- New or reinforced preference:
  - Floor/ground plane must always align to robot base and must never follow tool position/origin.
- How it changed execution:
  - Grounding computation now enforces a base-only geometry contract with explicit exclusions.

#### What Worked

- Source: `[tool]`
- Pattern/check that worked:
  - Immediate `npm run build` + `ReadLints` after the targeted patch validated correctness quickly without unrelated churn.

#### What Did Not Work

- Source: `[tool]`
- Failed attempt and why:
  - The previous visibility-toggle strategy was insufficient because bounding expansion can still include hidden descendants depending on traversal behavior.

#### Guardrails For Next Session

- Preflight rule:
  - If a user reports grounding drift, first audit which scene subtrees are included in bounds calculation before adjusting robot/tool transforms.

#### Follow-Ups / Risks

- Remaining risk or pending check:
  - Robots with incorrect base CAD origin may still appear offset; fix belongs in robot base mesh/URDF frame definitions.

### 2026-02-20 02:10 +11:00 - J6-only anchor contract for tool STL

#### Task Summary

- Tightened tool visual frame semantics so STL attachment is anchored to `joint6` frame first and offset values remain TCP-only.

#### Mistakes And Fixes

- Source: `[self]`
- Mistake:
  - Previous anchor logic preferred `joint6` child link, which can introduce ambiguity when users require strict J6 flange semantics.
- Detection:
  - User clarified non-negotiable requirement: tool STL origin is at J6 flange/J6 location (`gradient-05.urdf` joint6 frame).
- Fix:
  - Updated `web-ui/src/ArmVisualizer.tsx` anchor resolution to use `joint6` directly (fallback only if unresolved), and added runtime warning on fallback usage.
- Preventive rule:
  - When user specifies an exact URDF joint frame contract, anchor visuals to that joint object explicitly rather than descendant links.

#### User Preferences

- Source: `[user]`
- New or reinforced preference:
  - Tool mesh origin must stay at J6 flange location; tool offset values are for TCP semantics and should not shift STL origin.
- How it changed execution:
  - Added explicit code comment/behavior split: `offset.*` affects TCP frame only; mesh stays in J6 frame.

#### What Worked

- Source: `[tool]`
- Pattern/check that worked:
  - Reading exact URDF/tool JSON lines first and then doing a targeted anchor patch kept the fix small and deterministic.
  - Verifying against live API on the actual port (`:4000`) exposed stale active-tool offsets and confirmed live re-apply behavior.

#### What Did Not Work

- Source: `[tool]`
- Failed attempt and why:
  - Initial API checks against wrong ports (`:8001`, `:8000`) failed/misled validation; controller API in this session is served on `:4000`.

#### Guardrails For Next Session

- Preflight rule:
  - For tool-frame regressions, verify three layers in order: URDF joint frame, visualizer anchor frame, then tool JSON offset/mesh splits.
  - Before concluding an offset mismatch, compare `/tools/library` vs `/info/runtime-config`; active runtime may be stale until `active_tool_id` is re-applied.

#### Follow-Ups / Risks

- Remaining risk or pending check:
  - If visual mismatch persists after this patch, inspect STL local CAD origin/orientation (mesh authoring), not runtime offset math.
  - `GET /kinematics/profile` currently hits WinError 10040 (UDP payload size path), so runtime offset verification should use `/info/runtime-config` until that transport issue is fixed.

### 2026-02-20 02:25 +11:00 - User-corrected J6 URDF origin accepted

#### Task Summary

- Incorporated user-corrected `joint6` URDF origin for `gradient-05` and synced web robot assets so tool/flange frame visualization follows the corrected transform.

#### Mistakes And Fixes

- Source: `[user]`
- Mistake:
  - Previous `joint6` origin values in URDF were incorrect for this robot build.
- Detection:
  - User supplied corrected `gradient-05.urdf` joint block values directly.
- Fix:
  - Confirmed `joint6` now uses `<origin xyz="0.0933 0.0478830 0" ...>` and synced web robot assets (`npm run sync:robot-assets`).
- Preventive rule:
  - Treat user-provided calibrated URDF joint origins as source-of-truth over prior assumptions/log history.

#### User Preferences

- Source: `[user]`
- New or reinforced preference:
  - J6/tool0 frame alignment must reflect corrected URDF values exactly.
- How it changed execution:
  - No transform reinterpretation added; runtime now consumes corrected URDF directly.

#### What Worked

- Source: `[tool]`
- Pattern/check that worked:
  - Fast verify + asset sync loop avoids unnecessary code churn when correction is data-only (URDF).

#### What Did Not Work

- Source: `[tool]`
- Failed attempt and why:
  - N/A for this task.

#### Guardrails For Next Session

- Preflight rule:
  - After URDF frame edits, always run `npm run sync:robot-assets` before judging visualizer alignment.

#### Follow-Ups / Risks

- Remaining risk or pending check:
  - Browser cache/dev session can mask updated robot bundles; use hard refresh/restart if old frame persists.

### 2026-02-20 02:43 +11:00 - Tangent-roll weld orientation implemented end-to-end

#### Task Summary

- Implemented a third weld angle (`tangent_roll_deg`) that rotates torch orientation about the per-step path tangent and wired it through planner/API/UI/save-load + tests.

#### Mistakes And Fixes

- Source: `[tool]`
- Mistake:
  - Attempted `pytest ...` directly in PowerShell before confirming command availability in this environment.
- Detection:
  - Shell returned `pytest : The term 'pytest' is not recognized...`.
- Fix:
  - Re-ran tests via repo venv interpreter: `.\.venv\Scripts\python -m pytest ...`.
- Preventive rule:
  - On Windows sessions in this repo, default to `.\.venv\Scripts\python -m pytest` unless `pytest` command availability is already confirmed.

#### User Preferences

- Source: `[user]`
- New or reinforced preference:
  - Add real implementation changes (not just explanation), and support exact weld orientation control with tangent-relative behavior.
- How it changed execution:
  - Implemented planner math + API/UI contract + persistence + tests in one pass, rather than limiting to conceptual guidance.

#### What Worked

- Source: `[self]`
- Pattern/check that worked:
  - Reusing existing `_rotate_about_axis(...)` with tangent (`forward`) axis kept the orientation change minimal and robust.
  - Normalizing option names at API boundary (`tangentRollDeg` -> `tangent_roll_deg`) prevented UI/backend mismatch.
  - Capturing `orientations_list` in a monkeypatched planner test verified actual orientation delta for nonzero tangent roll.

#### What Did Not Work

- Source: `[tool]`
- Failed attempt and why:
  - Running plain `pytest` failed due PATH/environment mismatch in PowerShell.

#### Guardrails For Next Session

- Preflight rule:
  - For new weld angle parameters, update all four layers together: UI draft type + API normalization + planner option parse + persistence (weld program save/load).
  - Validate both default-backward compatibility (`0.0`) and nonzero behavior through tests before handoff.

#### Follow-Ups / Risks

- Remaining risk or pending check:
  - Operator ergonomics may still prefer inverted sign direction for tangent roll; if requested, invert in UI input mapping while keeping planner right-hand-rule internal semantics stable.
