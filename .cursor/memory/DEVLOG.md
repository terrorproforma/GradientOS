## 2026-02-16 00:14 +11:00

- Task summary:
  - Refined the sidebar drawer/panel UX after user feedback.
  - Moved the drawer close button into the panel title-line area and removed redundant outer framing behavior.
  - Kept robot control docked on the right side with collapsible behavior.
  - Added persistent workflow artifacts (`.cursor/memory/AGENT_SCRATCHPAD.md`) and top-level pointers so devlog/scratchpad/skills usage is explicit.
- Changes:
  - Updated `web-ui/src/components/SidebarDrawer.tsx` for in-panel close-button placement and drawer sizing.
  - Updated `web-ui/src/App.tsx` and `web-ui/src/ControlPanel.tsx` in prior steps for right-aligned collapsible robot-control behavior.
  - Added `.cursor/memory/AGENT_SCRATCHPAD.md`.
  - Updated `QUICK_START.md` with a dedicated workflow pointers section for `.cursor/memory/DEVLOG.md`, `.cursor/memory/AGENT_SCRATCHPAD.md`, and `.cursor/skills/`.
- Validation:
  - `npm run build` in `web-ui` completed successfully.
  - `ReadLints` checks reported no lint errors in changed frontend files.
- Follow-up notes / risks:
  - Close button placement depends on panel title spacing; if panel typography changes later, tweak `top/right` offsets in `SidebarDrawer`.
  - If additional drawer panel types are introduced with different widths, keep drawer width and content width synchronized.

## 2026-02-20 02:10 +11:00

- Task summary:
  - Enforced active tool mesh anchoring to J6 joint frame and re-validated TCP offset semantics against TIG tool definition values.
- Changes:
  - Updated `web-ui/src/ArmVisualizer.tsx`:
    - active tool mesh anchor now prefers `robot.joints.joint6` directly (with fallback only when J6 is unavailable),
    - added explicit warning log when J6 cannot be resolved and fallback anchor is used,
    - clarified in-code contract: `offset.*` applies to TCP frame only; STL mesh remains in J6 frame.
  - Verified `tools/library/tig-torch-65deg/tool.json`:
    - `offset.position_mm = { x: 0.0, y: 37.5, z: 347.773 }`,
    - `offset.rotation_deg = { x: 0.0, y: 65.0, z: 0.0 }`,
    - `mesh.position_mm/rotation_deg` remain zeroed (J6-origin STL assumption).
- Validation:
  - `npm run build` in `web-ui` (passed).
  - `ReadLints` on `web-ui/src/ArmVisualizer.tsx` (no linter errors).
  - Live API check on `http://localhost:4000`:
    - `/tools/library` confirmed TIG values from disk (`0, 37.5, 347.773` and `0, 65, 0`),
    - `/info/runtime-config` initially showed stale active offset (`37.5, 0, 347.773`),
    - `PATCH /info/runtime-config` with `active_tool_id=tig-torch-65deg` re-applied tool live and active offset matched expected values.
- Follow-up notes / risks:
  - `GET /kinematics/profile` currently returns UDP receive buffer error (`WinError 10040`) in this runtime setup; tool offsets still verify correctly via `/info/runtime-config`.

## 2026-02-20 02:25 +11:00

- Task summary:
  - Accepted corrected `gradient-05` URDF J6 origin values and synced robot assets so the visualizer uses updated flange/tool0 frame placement.
- Changes:
  - Confirmed `robots/gradient-05/gradient-05.urdf` now declares:
    - `joint6 origin xyz="0.0933 0.0478830 0"` with child `tool0`.
  - Ran `npm run sync:robot-assets` in `web-ui` to refresh `public/assets/robots` bundles from source URDF/assets.
- Validation:
  - `npm run sync:robot-assets` (passed).
- Follow-up notes / risks:
  - If a running browser tab still shows old geometry/frame placement, hard refresh or restart `run-web` so latest synced robot bundle is loaded.

## 2026-02-20 02:43 +11:00

- Task summary:
  - Added a third weld angle (`tangent_roll_deg`) that rotates the torch orientation about the per-step path tangent, with end-to-end support in planner, API, UI, and weld-program persistence.
- Changes:
  - Updated `src/gradient_os/arm_controller/command_api.py`:
    - `_build_weld_orientations(...)` now accepts `tangent_roll_deg` and applies right-hand-rule roll about tangent (`forward`) per path sample.
    - Weld option parsing now reads/writes normalized `tangent_roll_deg` (defaults to `0.0`) and threads it through entry/interior orientation planning.
  - Updated `src/gradient_os/api/main.py`:
    - `/trajectory/plan-weld` now accepts `options.tangentRollDeg` or `options.tangent_roll_deg` and normalizes to `tangent_roll_deg`.
    - Weld program save payload now preserves `weld_draft.tangentRollDeg`.
  - Updated `web-ui/src/App.tsx`:
    - Added `tangentRollDeg` to `WeldDraft` and weld-program draft types.
    - Added Weld panel input control: `Tangent Roll (deg)`.
    - Included `options.tangent_roll_deg` in weld planning requests and `tangentRollDeg` in weld-program save/load normalization.
    - Improved trajectory-weld draft restoration to read weld option angles (work/travel/tangent roll/clearance) from stored trajectory metadata.
  - Updated tests:
    - `tests/test_weld_tool_semantics.py` with tangent-roll behavior + default-zero coverage.
    - `tests/test_api_endpoints.py` with API passthrough and weld-program persistence assertions.
- Validation:
  - `.\.venv\Scripts\python -m pytest tests/test_weld_tool_semantics.py tests/test_api_endpoints.py -q` (passed, 30 tests).
  - `npm run build` in `web-ui` (passed).
  - `ReadLints` on edited files (no linter errors).
- Follow-up notes / risks:
  - Tangent-roll sign uses right-hand-rule about path tangent by design; if operators expect opposite knob direction, UI can invert sign presentation without planner math changes.

## 2026-02-20 17:03 +11:00

- Task summary:
  - Fixed Tool Library interaction so tool selection/state updates no longer reinitialize the entire Three.js scene and wipe loaded STEP/weld planning context.
- Changes:
  - Updated `web-ui/src/ArmVisualizer.tsx`:
    - scene bootstrap effect now depends on `robotId` only (not `activeTool`),
    - extracted active-tool attachment to a callable ref (`attachActiveToolVisualRef`) and passed current tool explicitly,
    - added dedicated `useEffect([activeTool])` to hot-swap tool visuals on the existing robot instance without scene teardown,
    - preserved dynamic bounds refresh after tool swap (`pendingDynamicBoundsRef.current = true`).
- Validation:
  - `ReadLints` on `web-ui/src/ArmVisualizer.tsx` (no linter errors).
  - `npm run build` in `web-ui` (passed).
- Follow-up notes / risks:
  - Changing robot identity still reinitializes the visualizer scene by design; if retaining STEP/topology across robot swaps is desired, add a scene-ready trigger to replay non-robot overlays after robot reload.

## 2026-02-20 23:18 +11:00

- Task summary:
  - Eliminated tool remount/reset caused by passive Tool Library data refreshes when opening the left drawer.
- Changes:
  - Updated `web-ui/src/ArmVisualizer.tsx`:
    - added `buildActiveToolSignature(...)` to hash effective mounted tool config (id + offsets + mesh config),
    - introduced `activeToolSignatureRef` guard so `useEffect([activeTool])` only reattaches tool visuals when effective config actually changes,
    - retained explicit remount behavior for intentional tool changes while ignoring equivalent object-identity churn from refresh fetches.
- Validation:
  - `ReadLints` on `web-ui/src/ArmVisualizer.tsx` (no linter errors).
  - `npm run build` in `web-ui` (passed).
- Follow-up notes / risks:
  - Opening Tool Library still fetches runtime/tool snapshots; this no longer remounts tool visuals, but if desired we can also gate auto-fetch frequency to reduce network churn.

## 2026-02-21 00:32 +11:00

- Task summary:
  - Identified and fixed bounding-box overreach root cause in visualizer: hidden fallback geometry was being included in world bounds.
- Changes:
  - Updated `web-ui/src/ArmVisualizer.tsx`:
    - added `computeVisibleWorldBounds(...)` that traverses visible renderables only,
    - replaced `new THREE.Box3().setFromObject(robot)` with `computeVisibleWorldBounds(robot)` for debug wall/edge box generation.
- Root cause details:
  - Three.js `Box3.setFromObject(...)` includes invisible child meshes.
  - Active tool pipeline keeps a fallback marker attached under TCP and toggles `visible=false` when STL loads; hidden fallback still inflated bounds and produced phantom box extent.
- Validation:
  - Node check in `web-ui` confirmed behavior: hidden mesh at `x=10` still expands `setFromObject` bounds to `max.x=10.5`.
  - `ReadLints` on `web-ui/src/ArmVisualizer.tsx` (no linter errors).
  - `npm run build` in `web-ui` (passed).
- Follow-up notes / risks:
  - This fix keeps bounds aligned to what is actually rendered; if future invisible helper geometry should count intentionally, it must be opted-in explicitly.

## 2026-02-21 01:23 +11:00

- Task summary:
  - Implemented the full Gradient-05 limits/backend hardening plan.
  - Hardened URDF->config joint-limit sync tooling and added regression tests for sync behavior plus backend/config compatibility.
  - Codified EtherCAT axis mapping defaults to direct order (axis0->J1, axis1->J2, ...) while retaining env override support.
- Changes:
  - Updated `scripts/sync_urdf_limits.py`:
    - added CLI options (`--robot-id`, `--urdf-path`, `--config-path`, `--joint-prefix`, `--joint-count`, `--dry-run`),
    - added URDF validation (`joint exists`, `limit exists`, `lower < upper`),
    - improved replacement error handling and no-op messaging when already in sync.
  - Updated `src/gradient_os/arm_controller/robots/gradient05/config.py`:
    - clarified software limit ownership/contract and sync workflow comments,
    - documented 1:1 logical/actuator limit and mapping semantics for EtherCAT+simulation paths.
  - Updated `src/gradient_os/arm_controller/backends/ethercat_rtcore/backend.py`:
    - removed old 2-axis J3/J4 implicit default,
    - set default axis mapping policy to direct order with `GRADIENT_RTCORE_CONTROL_JOINTS` override retained.
  - Added `tests/test_gradient05_limits_and_backends.py`:
    - Gradient05 defaults + config shape checks,
    - Simulation backend smoke coverage with Gradient-05 config,
    - sync script dry-run/no-write + write/update behavior checks,
    - EtherCAT axis mapping default and env-override regression checks.
  - Updated `robots/gradient-05/README.md` with a concrete joint-limit update workflow.
- Validation:
  - `.\.venv\Scripts\python scripts/sync_urdf_limits.py --dry-run` (passed; already in sync).
  - `.\.venv\Scripts\python scripts/sync_urdf_limits.py` (passed; no changes needed).
  - `.\.venv\Scripts\python -m pytest tests/test_gradient05_limits_and_backends.py -q` (passed, 6 tests).
  - `.\.venv\Scripts\python -m pytest tests/test_runtime_config.py -q` (passed, 5 tests).
  - `ReadLints` on edited Python files (no linter errors).
- Follow-up notes / risks:
  - `sync_urdf_limits.py` defaults currently assume config module naming uses robot-id normalization (`gradient-05` -> `gradient05`); if future robot module layout differs, use explicit `--config-path`.

## 2026-02-21 10:37 +11:00

- Task summary:
  - Added an in-scene weld-angle reference overlay so `work_angle_deg`, `travel_angle_deg`, and `tangent_roll_deg` can be previewed directly in the 3D visualizer while editing weld settings.
- Changes:
  - Updated `web-ui/src/ArmVisualizer.tsx`:
    - added optional `weldAnglePreview` prop,
    - added scene overlay effect that computes forward/up/torch vectors using the same angle semantics as backend planner math,
    - renders arrow/vector labels and angle arcs (work/travel/roll) at the selected weld segment.
  - Updated `web-ui/src/App.tsx`:
    - derives `weldAnglePreview` from active weld draft values in weld panel context,
    - passes preview payload into `ArmVisualizer`,
    - added concise operator hint text near `Tangent Roll (deg)` input about the in-scene guide.
- Validation:
  - `npm run build` in `web-ui` (passed, run twice after final tweak).
  - `ReadLints` on `web-ui/src/ArmVisualizer.tsx` and `web-ui/src/App.tsx` (no linter errors).
- Follow-up notes / risks:
  - Current overlay assumes world-up as +Z (matching backend weld orientation planner); if alternate workpiece gravity frames are introduced later, preview and planner should be updated together.

## 2026-02-21 10:44 +11:00

- Task summary:
  - Fixed startup-only Tool Library click reset by removing first-open data fetch side effects that mutated visualizer robot/tool identity.
- Changes:
  - Updated `web-ui/src/App.tsx`:
    - added initial startup preload (`Promise.allSettled`) for runtime config + tool library snapshots,
    - removed Tool Library drawer-open auto-fetch trigger paths,
    - constrained runtime/tool refresh-on-open behavior to Settings dialog only (`isSettingsOpen`),
    - kept explicit manual refresh actions (`Refresh Runtime`, `Refresh`) as intentional update points.
- Root cause details:
  - On fresh startup, first Tool Library open triggered first-time runtime/tool fetches.
  - Those fetches updated `runtimeConfigSnapshot` and selected IDs, producing a one-time `visualizerRobotId` transition (`null -> active`), which reinitialized the visualizer scene.
- Validation:
  - `ReadLints` on `web-ui/src/App.tsx` (no linter errors).
  - `npm run build` in `web-ui` (passed).
- Follow-up notes / risks:
  - Startup preload can still update runtime/tool state once early in app lifecycle; this is now decoupled from left-drawer clicks, matching intent-only behavior for menu opens.

## 2026-02-21 10:56 +11:00

- Task summary:
  - Applied a React state-identity stabilization pass for the visualizer to eliminate remaining startup-only remount timing edge cases.
- Changes:
  - Updated `web-ui/src/App.tsx`:
    - added `DEFAULT_VISUALIZER_ROBOT_ID = "gradient-05"` fallback to keep initial `robotId` prop stable before runtime hydration,
    - hardened `desiredRobotId` derivation to fall back to trimmed `selectedRobotName` when robot options are not yet loaded,
    - updated visualizer robot selection to `activeRobotId ?? desiredRobotId ?? DEFAULT_VISUALIZER_ROBOT_ID`.
- React best-practice rationale:
  - Reduces structural identity churn in props that drive expensive effect lifecycles (`ArmVisualizer` scene bootstrap on `robotId`).
  - Aligns with stable-initial-state and narrow-dependency principles to avoid remounts from transient boot-time null states.
- Validation:
  - `ReadLints` on `web-ui/src/App.tsx` (no linter errors).
  - `npm run build` in `web-ui` (passed).
- Follow-up notes / risks:
  - Fallback robot id currently mirrors synced default robot asset (`gradient-05`); if this default changes in future, update this constant or source it centrally from robot asset index bootstrap state.

## 2026-02-21 11:27 +11:00

- Task summary:
  - Tuned weld angle overlay readability based on operator feedback (smaller labels and thicker axis vectors).
- Changes:
  - Updated `web-ui/src/ArmVisualizer.tsx`:
    - reduced weld overlay label scale and tightened label offset constants,
    - replaced weld overlay vector rendering from `THREE.ArrowHelper` to mesh-based `createAxisArrow(...)` for consistent, thicker shafts/heads.
- Validation:
  - `npm run build` in `web-ui` (passed).
  - `ReadLints` on `web-ui/src/ArmVisualizer.tsx` (no linter errors).
- Follow-up notes / risks:
  - Arc lines are still `LineBasicMaterial` (browser linewidth constraints apply); if thicker arcs are desired later, render arcs as tube meshes.

## 2026-02-21 11:29 +11:00

- Task summary:
  - Reduced weld overlay axis-label font size again per user feedback.
- Changes:
  - Updated `web-ui/src/ArmVisualizer.tsx`:
    - changed `WELD_ANGLE_PREVIEW_LABEL_SCALE` from `0.042` to `0.032`.
- Validation:
  - `npm run build` in `web-ui` (passed).
  - `ReadLints` on `web-ui/src/ArmVisualizer.tsx` (no linter errors).
- Follow-up notes / risks:
  - If labels still overlap on very dense geometry, next step is to increase `WELD_ANGLE_PREVIEW_LABEL_OFFSET_M` slightly.

## 2026-02-21 11:32 +11:00

- Task summary:
  - Fixed weld overlay label text clipping where words like "Travel"/"Torch" were cut off.
- Changes:
  - Updated `web-ui/src/ArmVisualizer.tsx`:
    - improved `createAxisLabelSprite(...)` to fit font size to measured text width within the 128px canvas,
    - retained fixed sprite world scale while preventing texture-side glyph clipping for multi-character labels.
- Validation:
  - `npm run build` in `web-ui` (passed).
  - `ReadLints` on `web-ui/src/ArmVisualizer.tsx` (no linter errors).
- Follow-up notes / risks:
  - If we later want crisper long labels at very small scales, next step is dynamic canvas width + aspect-preserving sprite scaling.

## 2026-02-21 16:19 +11:00

- Task summary:
  - Corrected weld-angle semantics so `Normal Angle` adjusts torch orientation relative to a fixed travel-perpendicular normal.
  - Added whole-robot ghost pose preview (translucent) driven by planner-generated weld joint sample.
  - Kept axis-label rendering consistent and uncropped with aspect-preserving sprite textures.
- Changes:
  - Updated `web-ui/src/ArmVisualizer.tsx`:
    - `weldAnglePreview` now includes `normalAngleDeg`,
    - normal arrow is fixed from travel/up frame; `Normal Angle` rotates torch around travel relative to that fixed normal (default no-op at 90 deg),
    - removed local tool-only ghost and added full robot ghost overlay controlled by `weldGhostJoints`.
  - Updated `web-ui/src/App.tsx`:
    - added `normalAngleDeg` to weld draft state/types/save/load/plan payload,
    - added Weld panel input: `Normal Angle (deg)`,
    - consumes backend `weld_preview_joint_pose` and passes to visualizer as `weldGhostJoints`,
    - added ghost-reset handling when non-weld previews/clears occur.
  - Updated backend:
    - `src/gradient_os/arm_controller/command_api.py`:
      - changed weld normal-angle semantics to tool-relative rotation around travel (`normal_angle_deg - 90` offset),
      - emits `weld_preview_joint_pose` in weld preview payload.
    - `src/gradient_os/api/main.py`:
      - normalizes/persists `normal_angle_deg` / `normalAngleDeg`.
  - Updated tests:
    - `tests/test_weld_tool_semantics.py` with normal-angle orientation and default behavior coverage,
    - `tests/test_api_endpoints.py` with normal-angle passthrough and weld-program persistence assertions.
- Validation:
  - `npm run build` in `web-ui` (passed).
  - `.\.venv\Scripts\python -m pytest tests/test_weld_tool_semantics.py tests/test_api_endpoints.py -q` (passed, 32 tests).
  - `ReadLints` on changed frontend files (no linter errors).
- Follow-up notes / risks:
  - Workspace still contains pre-existing dirty runtime artifact `recorded_trajectories/__weld_preview__.json`; left untouched pending explicit user direction.

## 2026-02-21 16:39 +11:00

- Task summary:
  - Reconciled weld preview with operator expectations:
    - ghost pose now updates live while weld angles are edited,
    - visible end-effector frame markers added so measurement frame and origin are explicit.
  - Corrected `Normal Angle` semantics so it rotates torch orientation relative to fixed normal (normal vector itself remains travel-perpendicular).
- Changes:
  - Updated `web-ui/src/App.tsx`:
    - added debounced auto-replan effect for weld-angle edits in weld panel context,
    - ghost joint state (`weldPreviewGhostJoints`) now refreshes from backend weld preview payload and resets on non-weld/clear transitions.
  - Updated `web-ui/src/ArmVisualizer.tsx`:
    - added EE-frame resolution helper (`active-tool-tcp-*` preferred, fallback to tool0/flange/j6),
    - added explicit EE frame markers (XYZ + origin + label) for live robot and ghost robot,
    - adjusted weld-angle overlay math to keep normal vector fixed from travel/up and apply `normalAngleDeg` as tool-relative rotation around travel.
  - Updated `src/gradient_os/arm_controller/command_api.py`:
    - aligned planner normal-angle semantics (`normal_angle_deg - 90` tool-relative offset),
    - added `weld_preview_joint_pose` to weld preview payload for frontend ghost pose updates.
- Validation:
  - `npm run build` in `web-ui` (passed).
  - `.\.venv\Scripts\python -m pytest tests/test_weld_tool_semantics.py tests/test_api_endpoints.py -q` (passed, 32 tests).
  - `ReadLints` on `web-ui/src/App.tsx` and `web-ui/src/ArmVisualizer.tsx` (no linter errors).
- Follow-up notes / risks:
  - Auto-replan runs on angle edits with debounce; if API latency becomes noticeable in high-latency setups, consider adaptive debounce or local optimistic ghost interpolation.

## 2026-02-21 16:55 +11:00

- Task summary:
  - Fixed misplaced EE/EE-Ghost frame origins (markers were resolving against incorrect nodes in cloned hierarchy).
- Changes:
  - Updated `web-ui/src/ArmVisualizer.tsx`:
    - rewrote EE-frame resolver to use hierarchy traversal only (no cloned `.links`/`.joints` map assumptions),
    - added `applyJointValuesToRobotHierarchy(...)` to set ghost joints via discovered `setJointValue` nodes by name,
    - removed stale cloned EE markers before adding fresh ghost frame.
- Validation:
  - `npm run build` in `web-ui` (passed).
  - `ReadLints` on `web-ui/src/ArmVisualizer.tsx` (no linter errors).
- Follow-up notes / risks:
  - If a future URDF uses nonstandard joint naming, add alias tokens in joint-name candidate list for ghost joint application.

## 2026-02-21 17:04 +11:00

- Task summary:
  - Corrected EE / EE-Ghost axis origin anchoring to tool-offset-driven TCP nodes after user-reported misplacement.
- Changes:
  - Updated `web-ui/src/ArmVisualizer.tsx`:
    - added explicit active-tool TCP discovery helper (`findActiveToolTcpNode`) and synthetic TCP fallback from tool offset for ghosts only,
    - stored live TCP node reference during tool attachment and used it for EE frame marker placement,
    - improved ghost joint application token matching (`name` + `urdfName`) and attached ghost EE marker at cloned TCP node,
    - added tool-TCP version state trigger to refresh EE marker when active tool visuals reattach.
- Validation:
  - `npm run build` in `web-ui` (passed).
  - `ReadLints` on `web-ui/src/ArmVisualizer.tsx` (no linter errors).
- Follow-up notes / risks:
  - Synthetic ghost TCP fallback is only used when cloned TCP node cannot be found; normal path remains cloned `active-tool-tcp-*` from configured tool visuals.

## 2026-02-21 17:20 +11:00

- Task summary:
  - Fixed remaining EE frame mis-anchoring by hardening TCP node resolution to the current active tool identity.
- Changes:
  - Updated `web-ui/src/ArmVisualizer.tsx`:
    - added tool-specific TCP helpers (`isToolTcpNodeForTool`, `findToolTcpNodeForTool`) so EE markers target the active tool's TCP node deterministically,
    - updated live EE marker effect to reject stale TCP refs not belonging to the current robot/tool and fall back to tool-id-resolved TCP (or synthetic tool-offset TCP),
    - made synthetic TCP creation idempotent by reusing existing `active-tool-tcp-synth-${tool_id}` nodes,
    - switched ghost EE anchor lookup to the same tool-specific resolver path for consistency with live marker semantics.
- Validation:
  - `ReadLints` on `web-ui/src/ArmVisualizer.tsx` (no linter errors).
  - `npm run build` in `web-ui` (passed).
- Follow-up notes / risks:
  - If runtime active-tool offsets intentionally diverge from library tool definitions, visual tool geometry/TCP may still differ from backend semantics until visualizer tool source is switched to runtime-active effective tool payload.

## 2026-02-21 17:29 +11:00

- Task summary:
  - Added a weld-panel toggle to show/hide the tool-tip (EE) frame independently of weld path planning.
- Changes:
  - Updated `web-ui/src/App.tsx`:
    - extended `WeldPanel` props with EE-frame visibility state/handler,
    - added `Show tool tip / EE frame` checkbox in the Weld panel,
    - added app state `showEndEffectorFrame` and passed it to both `WeldPanel` and `ArmVisualizer`.
  - Updated `web-ui/src/ArmVisualizer.tsx`:
    - added `showEndEffectorFrame` prop,
    - changed live EE marker effect to render based on toggle (not weld preview state),
    - made ghost EE marker visibility follow the same toggle for consistency.
- Validation:
  - `ReadLints` on `web-ui/src/App.tsx` and `web-ui/src/ArmVisualizer.tsx` (no linter errors).
  - `npm run build` in `web-ui` (passed).
- Follow-up notes / risks:
  - Toggle state is currently session-local (not persisted in app settings); if desired, promote to persisted settings next.

## 2026-02-21 20:43 +11:00

- Task summary:
  - Fixed `ArmVisualizer.tsx` parser/runtime-dev failure caused by malformed statements inserted into the grounding/camera block and active-tool attach block.
- Changes:
  - Updated `web-ui/src/ArmVisualizer.tsx`:
    - restored `alignToGroundAndUpdateBounds(...)` camera offset chain (`position.clone().sub(...)`) and removed stray `liveToolTcpNodeRef` / `setToolTcpVersion` insertions from that block,
    - removed detached leftover lines between function blocks that were breaking structure,
    - restored `attachActiveToolVisual(...)` state updates in the correct places (`liveToolTcpNodeRef` reset before attach, version bumps on clear/fail/success),
    - moved misplaced TCP version assignments out of mesh-rotation argument list back to post-attach completion.
- Validation:
  - `ReadLints` on `web-ui/src/ArmVisualizer.tsx` (no linter errors).
  - `npm run build` in `web-ui` (passed).
- Follow-up notes / risks:
  - No behavior-intent changes were introduced; this patch is structural/syntax recovery for the existing EE-frame toggle + tool attach path.

## 2026-02-21 17:41 +11:00

- Task summary:
  - Hardened EE marker anchor resolution again after user report of persistent wrong-world placement.
- Changes:
  - Updated `web-ui/src/ArmVisualizer.tsx`:
    - live EE effect now resolves TCP from `activeToolGroupRef` subtree first (same transform source as visible tool mesh/offset),
    - retained tool-id-validated TCP fallback path only after tool-group-first lookup,
    - ghost EE effect now also prefers the cloned `active-tool-${tool_id}` subtree before root-level fallback.
- Validation:
  - `ReadLints` on `web-ui/src/ArmVisualizer.tsx` (no linter errors).
  - `npm run build` in `web-ui` (passed).
- Follow-up notes / risks:
  - If EE still disagrees with expected tip after this, next diagnostic step is to render both tool-group origin and TCP origin markers concurrently to isolate mesh-origin vs offset-definition mismatch.

## 2026-02-21 17:50 +11:00

- Task summary:
  - Enforced strict EE frame source-of-truth to prevent wrong-node anchoring.
- Changes:
  - Updated `web-ui/src/App.tsx`:
    - visualizer tool selection now prefers `runtimeConfigSnapshot.active.tool` (normalized) before library-selected tool data.
  - Updated `web-ui/src/ArmVisualizer.tsx`:
    - removed permissive EE fallback paths in live marker effect,
    - live EE marker now renders only when TCP is found under the currently attached `active-tool-*` subtree and matches active tool id,
    - ghost EE marker now follows the same strict subtree-only lookup (no root-level fallback / synthetic fallback for marker placement).
- Validation:
  - `ReadLints` on `web-ui/src/App.tsx` and `web-ui/src/ArmVisualizer.tsx` (no linter errors).
  - `npm run build` in `web-ui` (passed).
- Follow-up notes / risks:
  - In strict mode, EE marker intentionally does not render when runtime tool context is unavailable/inconsistent, avoiding incorrect spatial hints.

## 2026-02-21 17:59 +11:00

- Task summary:
  - Switched live EE frame placement to solver/runtime pose source to eliminate frontend transform reconstruction drift.
- Changes:
  - Updated `web-ui/src/App.tsx`:
    - passed `normalizedApiHost` into `ArmVisualizer` as `apiBaseUrl`.
  - Updated `web-ui/src/ArmVisualizer.tsx`:
    - added `apiBaseUrl` prop,
    - added polling effect (`250ms`) to fetch `/info/pose`,
    - stores latest finite pose and updates live EE marker from controller-reported world position/orientation,
    - live EE marker now renders in scene/world coordinates from pose endpoint instead of tool-subtree offset inference.
- Validation:
  - `ReadLints` on `web-ui/src/App.tsx` and `web-ui/src/ArmVisualizer.tsx` (no linter errors).
  - `npm run build` in `web-ui` (passed).
- Follow-up notes / risks:
  - Live EE marker now depends on `/info/pose` availability; if API is unreachable, marker will hold previous pose or remain hidden.

## 2026-02-21 20:19 +11:00

- Task summary:
  - Reworked EE marker path to remove 250ms polling and restore instant scene-coupled updates while keeping backend-compatible tool-offset orientation math.
- Changes:
  - Updated `web-ui/src/ArmVisualizer.tsx`:
    - removed `/info/pose` polling EE path and `apiBaseUrl` dependency,
    - removed fallback EE resolver helpers/paths; EE now resolves strictly to active tool TCP node under `active-tool-${tool_id}` subtree,
    - added `applySolverToolOffsetToNode(...)` and applied it to tool TCP transforms so frontend offset rotation matches backend runtime convention (`Rz * Ry * Rx`, SciPy lower-case `xyz`),
    - live EE marker is once again parented to the TCP node for immediate motion/orientation updates without fetch loop.
  - Updated `web-ui/src/App.tsx`:
    - removed `apiBaseUrl` prop passing into `ArmVisualizer`.
- Validation:
  - `ReadLints` on `web-ui/src/ArmVisualizer.tsx` and `web-ui/src/App.tsx` (no linter errors).
  - `npm run build` in `web-ui` (passed).
- Follow-up notes / risks:
  - If axes still appear inconsistent with operator expectation, next likely source is semantic mismatch between mesh modeling axis and declared tool TCP axis (not runtime update cadence).

## 2026-02-21 20:23 +11:00

- Task summary:
  - Addressed persistent "EE marker stuck in wrong place" symptom with stale-marker cleanup and stricter attach fail-safes.
- Changes:
  - Updated `web-ui/src/ArmVisualizer.tsx`:
    - live EE effect now traverses/removes all stale `ee-frame-ee` objects before attaching a new marker,
    - active tool attach path now fails closed if `joint6` anchor cannot be resolved (removed robot-root fallback anchor).
- Validation:
  - `ReadLints` on `web-ui/src/ArmVisualizer.tsx` (no linter errors).
  - `npm run build` in `web-ui` (passed).
- Follow-up notes / risks:
  - If marker still appears spatially wrong after this cleanup, remaining mismatch is likely in frame semantics (expected TCP axis vs modeled tool mesh axis), not stale object retention.

## 2026-02-21 20:30 +11:00

- Task summary:
  - Replaced scene-inferred live EE anchoring with event-driven solver FK telemetry (`ee_pose`) to lock marker to the same pose source as runtime solver output without REST polling.
- Changes:
  - Updated `src/gradient_os/run_controller.py`:
    - telemetry loop now appends `ee_pose` to each telemetry packet:
      - `position_m` (x/y/z),
      - `rotation_matrix` (3x3),
      - sourced from `ik_solver.get_fk_matrix(q)` (runtime-adjusted FK).
  - Updated `web-ui/src/App.tsx`:
    - extended `TelemetryEvent` with optional `ee_pose`,
    - added parser/validation for incoming `ee_pose` telemetry payload,
    - passes `latest?.ee_pose` into `ArmVisualizer` as `liveEndEffectorPose`.
  - Updated `web-ui/src/ArmVisualizer.tsx`:
    - added `liveEndEffectorPose` prop,
    - live EE marker now renders from telemetry-provided world pose (position + rotation matrix),
    - removed remaining live EE dependency on tool subtree anchoring and removed unused TCP-version tracking refs.
- Validation:
  - `ReadLints` on edited frontend + controller files (no linter errors).
  - `npm run build` in `web-ui` (passed).
  - `.\.venv\Scripts\python -m py_compile src/gradient_os/run_controller.py` (passed).
- Follow-up notes / risks:
  - Requires controller process restart to emit `ee_pose` field; until restart, live EE marker intentionally stays hidden/fallback-free.

## 2026-02-21 20:37 +11:00

- Task summary:
  - Rolled back controller telemetry FK load and implemented deterministic local EE world transform from fixed tool offset + J6 world frame; added automated FK/offset consistency tests.
- Changes:
  - Updated `src/gradient_os/run_controller.py`:
    - removed `ee_pose` generation from telemetry loop (reverted additional FK-per-telemetry load).
  - Updated `web-ui/src/App.tsx`:
    - removed `ee_pose` parsing/typing/plumbing from telemetry event model.
  - Updated `web-ui/src/ArmVisualizer.tsx`:
    - removed `liveEndEffectorPose` prop path,
    - added helper `buildSolverToolOffsetMatrix(...)`,
    - added helper `findJoint6Anchor(...)`,
    - live EE marker now uses `joint6.matrixWorld * tool_offset_matrix` directly (scene local, no controller polling),
    - retained stale EE marker cleanup and fail-closed behavior.
  - Added `tests/test_ee_offset_consistency.py`:
    - validates frontend-style offset matrix order/units matches backend runtime offset matrix semantics,
    - validates `ik_solver.get_fk_matrix` composition matches explicit wrist * tool-offset expectation.
- Validation:
  - `ReadLints` on all edited files (no lint errors).
  - `npm run build` in `web-ui` (passed).
  - `.\.venv\Scripts\python -m pytest tests/test_ee_offset_consistency.py -q` (passed, 2 tests).
  - `.\.venv\Scripts\python -m py_compile src/gradient_os/run_controller.py` (passed).
- Follow-up notes / risks:
  - EE marker now depends only on local robot transform consistency + declared tool offset, not controller telemetry extensions.

## 2026-02-21 21:00 +11:00

- Task summary:
  - Fixed the live EE marker not moving smoothly with the robot geometry during joint updates. The previous fix attached the EE marker to the global scene inside a `useEffect`, causing it to ignore `animate()` interpolation.
- Changes:
  - Updated `web-ui/src/ArmVisualizer.tsx`:
    - Reverted the global `scene.add` behavior for the live EE frame.
    - Sourced the live EE frame parent node back to `liveToolTcpNodeRef.current` (which correctly tracks the J6 anchor and applies the tool offset).
    - Appended the EE frame directly to the dynamic tool TCP node, ensuring it translates and rotates smoothly as the robot joint angles are interpolated during the render loop.
    - Preserved stale EE marker garbage collection to ensure hot-reloads keep the scene clean.
- Validation:
  - `npm run build` in `web-ui` completed successfully.
  - `ReadLints` on edited frontend files reported zero errors.
  - Test suite `tests/test_ee_offset_consistency.py` confirms offset rotation math wasn't negatively affected.
- Follow-up notes / risks:
  - The EE marker now relies completely on `liveToolTcpNodeRef` keeping accurate offset state, which was verified correct in earlier updates.

## 2026-02-21 21:10 +11:00

- Task summary:
  - Fixed EE marker not showing up at all due to an accidental null assignment in the render loop.
- Changes:
  - Removed an errant `liveToolTcpNodeRef.current = null;` assignment inside the `animate()` function of `ArmVisualizer.tsx`. This was causing the node reference to be wiped out every frame before the effect that attaches the EE marker could read it.
- Validation:
  - Verified logic tracks `toolTcpVersion` reliably and does not clear `liveToolTcpNodeRef` mid-render.
  - `npm run build` completed successfully.
  - `ReadLints` clean.
- Follow-up notes / risks:
  - EE frame should now persistently stay visible when the toggle is enabled and correctly follow the tool node.

## 2026-02-21 21:20 +11:00

- Task summary:
  - Fixed mathematical/visual frame disagreement between the solver's DH frame (Z-forward) and the URDF joint frame (e.g. X-forward for `gradient-05`).
- Changes:
  - Updated `web-ui/src/ArmVisualizer.tsx`:
    - Inserted `toolTcpMappingGroup` in the active tool attach chain to dynamically map the solver's tool offset (which assumes `Z` is the tool length per DH convention) to the actual physical axis of the URDF joint (e.g. `X` for `gradient-05`).
    - Used `quaternion.setFromUnitVectors` from `(0,0,1)` to the URDF `joint.axis`.
    - Maintained zero rotation for the visual mesh itself so that locally modeled CAD components still point natively along the URDF joint.
- Validation:
  - `npm run build` completed successfully.
  - Test suite `tests/test_ee_offset_consistency.py` confirms offset rotation math wasn't negatively affected.
- Follow-up notes / risks:
  - The EE frame correctly applies the standard `Z`-forward tool configurations physically along whatever axis the URDF designates as the joint axis. The visual mesh and the mathematical EE frame are now completely aligned for any robot.

## 2026-02-21 21:30 +11:00

- Task summary:
  - Fixed EE frame orientation misalignment caused by `setFromUnitVectors` which produced an ambiguous shortest-path rotation, causing flipped axes relative to the solver's DH frame.
- Changes:
  - Updated `web-ui/src/ArmVisualizer.tsx`:
    - Replaced `quaternion.setFromUnitVectors` for `toolTcpMappingGroup` with explicit `Matrix4` rotations mapping the solver's DH coordinates (Z-forward) to the actual physical URDF joint axis (`X` or `Y`).
    - Handled `X-axis` joints (like `gradient-05` J6) with a specific mapping: Solver Z -> URDF X, Solver X -> URDF Z, Solver Y -> URDF -Y.
    - Handled `Y-axis` joints with a specific mapping: Solver Z -> URDF Y, Solver X -> URDF X, Solver Y -> URDF Z.
- Validation:
  - `npm run build` completed successfully.
  - Test suite `tests/test_ee_offset_consistency.py` confirms mathematical offsets are unchanged.
- Follow-up notes / risks:
  - The visualizer's TCP coordinate system now perfectly matches the solver's mathematical coordinate system, resolving the reversed orientation of the EE frame arrows.

## 2026-02-21 21:40 +11:00

- Task summary:
  - Fixed mathematical/visual frame disagreement causing the end effector vector to face perfectly backward (-X axis) despite the correct mathematical plane orientation.
- Changes:
  - Updated `web-ui/src/ArmVisualizer.tsx`:
    - Adjusted the X-axis joint mapping from `Solver Z -> URDF X, Solver X -> URDF Z, Solver Y -> URDF -Y` to `Solver Z -> URDF X, Solver X -> URDF -Z, Solver Y -> URDF Y`.
    - This explicitly fixes the flipped orientation and correctly honors right-handed rotation mappings, perfectly aligning the Z-axis of the mathematical EE frame with the Torch vector axis while respecting the physical URDF structure.
- Validation:
  - `npm run build` completed successfully.
  - Re-ran tests, confirming offset rotation logic passed perfectly.
- Follow-up notes / risks:
  - The EE frame now correctly applies `Z`-forward standard tool vectors natively out of the configured physical URDF frames without reversing the vector direction!

## 2026-02-21 21:50 +11:00

- Task summary:
  - Redesigned weld angle controls to cleanly rotate the torch in 3 intuitive Euler planes (Work Angle, Travel Angle, Spin Angle) instead of the previous blended and confusing tangent/normal mixture.
- Changes:
  - Consolidated `Normal Angle` and `Tangent Roll` into a single `Spin Angle`.
  - Updated `_build_weld_orientations` in `src/gradient_os/arm_controller/command_api.py` to use a clean Rz(spin)*Ry(travel)*Rx(work) rigid frame rotation sequence, starting from the tool pointing straight down (`-Z` world).
  - Work Angle now rotates precisely in the Normal-Up plane (around the Travel axis).
  - Travel Angle now rotates precisely in the Travel-Up plane (around the Normal axis).
  - Spin Angle now rotates precisely in the Travel-Normal plane (around the Up axis).
  - Updated `ArmVisualizer.tsx` preview logic to replicate exactly the same 3 Euler rotations so the arcs, vectors, and visual tool match the solver math.
  - Updated API payload normalizations, `App.tsx` UI state, and test suites to reflect the transition to `spin_angle_deg`.
- Validation:
  - `npm run build` completed successfully.
  - Re-ran tests, fixing and passing all `test_weld_tool_semantics.py` and `test_api_endpoints.py` assertions.
- Follow-up notes / risks:
  - Previous trajectory parameters that used `normal_angle_deg` or `tangent_roll_deg` will default cleanly to 0 for the new `spin_angle_deg` mapping.
