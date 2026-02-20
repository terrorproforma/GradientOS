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
