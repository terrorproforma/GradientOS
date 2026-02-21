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

### 2026-02-21 - Weld angle visualization context
- [user] Operator expectation: angle fields must be understandable directly in the 3D scene while editing, not only via textual/tooltip explanation.
- [self] For angle preview features, mirror backend math in UI helpers (same clamps, same axis order) to avoid drift between what users see and what planner executes.
- [self] Keep weld overlays conditional on weld-focused context (`activePanel === "weld"` with active segment) to reduce visual noise in general operation modes.

### 2026-02-21 - Startup-only Tool Library reset follow-up
- [self] A startup-only reset can come from first-time data hydration, not just repeated panel-open effects; inspect `null -> value` identity transitions for props that gate scene bootstrap (`robotId`, `activeTool`).
- [self] Removing side-effectful fetches from passive UI actions (opening drawer tabs) is safer than trying to mask downstream remount symptoms.
- [user] Menu open must be inert; runtime/tool mutations should happen only on explicit user intent (apply/mount/refresh actions).
- [self] Keep expensive visualizer identity props stable at startup with explicit fallbacks (avoid transient `null` IDs that later resolve to the same default and trigger avoidable remounts).

### 2026-02-21 - Overlay readability tuning
- [user] For in-scene helpers, prioritize legibility over visual flair; oversized labels reduce clarity when they overlap geometry.
- [self] Use mesh-based arrows (`createAxisArrow`) instead of `ArrowHelper` when the user asks for thicker axis lines; mesh radius is reliable across platforms.
- [user] Continue tuning in small increments; single-constant label scale changes are preferred for quick visual iteration.
- [self] Label clipping came from fixed 128px canvas + fixed 82px font, not only world scale; fit font to measured text width before drawing.
- [user] Domain semantics: weld normal vector is derived from travel and remains perpendicular to travel; user "normal angle" should rotate the torch relative to that fixed normal, not rotate the normal itself.
- [self] For whole-robot pose ghosts in weld preview, source joint samples from backend planner payload rather than approximating from frontend vectors.
- [user] Preview expectation: ghost pose must react live while editing angle inputs; add debounced auto-replan keyed on weld-angle state so backend-driven ghost keeps pace with UI.
- [self] To make frame-of-reference obvious, attach an explicit EE frame marker (XYZ + origin) to live and ghost end-effector nodes (`active-tool-tcp-*` preferred, then tool0/flange/j6 fallbacks).
- [self] On cloned URDF ghosts, avoid relying on `.links`/`.joints` maps for spatial anchor lookup; resolve EE nodes via cloned hierarchy traversal and apply joint values through discovered `setJointValue` nodes by name.
- [user] Do not blame URDF changes when bug is frame anchoring logic; tool-tip/EE visualization must come from tool offset semantics regardless of unchanged URDF.

### 2026-02-21 - EE frame TCP resolver hardening
- [user] EE frame must anchor at configured tool-tip offset (tool config semantics), not drift to arbitrary nodes.
- [self] For EE markers, validate TCP candidates against the active tool id (`active-tool-tcp-${tool_id}` / synth variant) instead of generic first-match traversal.
- [self] Before trusting stored object refs (e.g. `liveToolTcpNodeRef`), re-resolve them against current robot hierarchy (`getObjectById`) to avoid stale-ref anchoring after reattach/remount.
- [self] Keep synthetic TCP creation idempotent by reusing existing `active-tool-tcp-synth-${tool_id}` nodes; this prevents duplicate fallback anchors across repeated effects.

### 2026-02-21 - EE visibility UX toggle
- [user] Operators want explicit control to show/hide EE frame on demand without requiring weld preview planning.
- [self] Gate EE marker rendering behind a dedicated UI toggle prop (`showEndEffectorFrame`) rather than weld-preview state to avoid hidden coupling.
- [self] Keep ghost EE marker honoring the same visibility toggle so live/ghost frame visibility stays consistent.

### 2026-02-21 - EE anchor source of truth
- [self] For live EE marker placement, resolve TCP from `activeToolGroupRef` subtree first; this guarantees marker follows the exact transform chain used by the visible tool attachment.
- [self] Avoid broad root-level fallback resolution when a tool group exists, since generic traversal can latch onto unintended nodes under complex scene state.

### 2026-02-21 - Remove wrong EE fallback paths
- [user] When correctness is critical, prefer "no marker" over misleading marker; eliminate permissive fallback paths that can silently attach to wrong nodes.
- [self] Source visualizer tool data from runtime active tool payload first (same context solver uses) before falling back to library snapshots.

### 2026-02-21 - Solver-sourced EE pose
- [user] When asked to match solver/source-of-truth, derive live EE marker from controller-reported pose endpoint rather than reconstructing from frontend hierarchy assumptions.
- [self] Polling `/info/pose` and drawing EE marker in world coordinates avoids URDF/tool-frame ambiguity and aligns with runtime solver output semantics.

### 2026-02-21 - Polling rollback and rotation-order parity
- [user] Prioritize instant transform-coupled updates over periodic polling for in-scene EE markers; polling was perceived as performance regression.
- [self] Keep EE marker parented to tool TCP node for zero-latency updates, but match backend offset rotation convention exactly (`R = Rz * Ry * Rx`, SciPy lower-case `xyz`) when building frontend tool TCP transforms.

### 2026-02-21 - Stale marker + anchor fail-closed
- [self] During hot-reload/rapid iteration, stale EE markers can persist in scene graph; proactively traverse/remove all `ee-frame-ee` nodes before attaching a fresh one.
- [self] Never fall back to robot-root anchoring when `joint6` is missing; fail closed and skip attach to avoid misleading EE frame placement.

### 2026-02-21 - Event-driven solver EE pose
- [self] Best parity path is to stream solver FK pose in controller telemetry (`ee_pose`) and render EE from that payload; this avoids frontend hierarchy inference drift while avoiding periodic REST polling.
- [user] Strongly prefer deterministic source-of-truth + low overhead over repeated fallback tweaks; when in doubt, wire explicit telemetry fields rather than guess from scene graph names.

### 2026-02-21 - Avoid controller telemetry load
- [user] Do not add solver-heavy telemetry work when a fixed-offset local scene transform can provide the same EE marker behavior.
- [self] For EE visuals, prefer scene-side `joint6.matrixWorld * tool_offset` reconstruction and reserve solver telemetry augmentation for explicit profiling-approved needs.

### 2026-02-21 - Syntax safety in large effects
- [tool] In long `useEffect` blocks, stray pasted statements can silently break chained expressions (`.clone().sub(...)`) and cause Babel parse failures far from the true intent.
- [self] After hot edits near camera/grounding logic, re-read full surrounding function block to catch misplaced lines before relying on incremental dev-server output.

### 2026-02-21 - EE frame rendering interpolation
- [user] If a marker is attached directly to the global scene instead of the robot's specific joint/tool node, it will not interpolate during `animate()` joint updates and will appear stuck.
- [self] Use `liveToolTcpNodeRef.current.add(marker)` rather than `scene.add(marker)` so that visual overlays inherently follow the smooth geometry transitions from joint state changes.

### 2026-02-21 - URDF to DH axis mapping
- [self] The solver's mathematical definition of tool parameters (like offset Z) is relative to the DH frame standard (where Z is the joint axis), but the physical CAD mesh sits in the URDF frame (e.g. `gradient-05` has its joint axis along X).
- [user] Rather than forcing all tool configs into physical frames or changing the URDF, visually map the solver's Z-offset vector directly to the URDF's dynamic joint axis using a rotation mapping group in the visualizer.
- [self] When mapping coordinates (like Solver Z to URDF X), avoid `quaternion.setFromUnitVectors` as it computes the shortest-path rotation which can leave secondary axes (X, Y) inverted. Use explicit `Matrix4` mappings (e.g. `[0,0,1,0, 0,-1,0,0, 1,0,0,0, 0,0,0,1]`) to preserve the exact right-handedness of the solver frame.
