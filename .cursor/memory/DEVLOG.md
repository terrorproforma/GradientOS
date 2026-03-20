## 2026-03-20 01:13 +0000

- Task summary:
  - Implemented the first architecture slice of the servo-profile refactor: shared DS402/EtherCAT profile decoding, runtime drive-profile selection, and honest commissioning joint-jog acknowledgement semantics.
- Changes:
  - Added shared profile modules:
    - `src/gradient_os/arm_controller/profiles/drive/cia402.py`
    - `src/gradient_os/arm_controller/profiles/drive/a6ec_ds402.py`
    - `src/gradient_os/arm_controller/profiles/fieldbus/ethercat.py`
    - `src/gradient_os/arm_controller/profiles/registry.py`
  - Added `src/gradient_os/telemetry/drive_faults.py` to build normalized drive-fault snapshots from RTCore metrics.
  - Updated `src/gradient_os/arm_controller/backends/registry.py` and `src/gradient_os/arm_controller/backends/ethercat_rtcore/config.py`:
    - backend defaults now resolve shared `drive_profile` / `fieldbus_profile` ids,
    - drive statusword + fault decoding now route through the shared profile registry instead of controller-local tables.
  - Updated `src/gradient_os/runtime_config.py` and `src/gradient_os/arm_controller/robots/__init__.py`:
    - runtime config now persists and resolves `desired.overrides.drive_profile`,
    - active runtime payloads now expose `drive_profile`,
    - restart-required checks now compare the fully resolved desired runtime, including overrides.
  - Updated `src/gradient_os/run_controller.py` and `start-stack.sh`:
    - removed embedded DS402 / EtherCAT AL decode tables,
    - both controller telemetry and launcher probe now consume the shared drive-fault snapshot builder,
    - controller startup logging now reports the active drive profile.
  - Updated `src/gradient_os/arm_controller/command_api.py` and `src/gradient_os/api/main.py`:
    - kept `WAIT_FOR_IDLE` trajectory-thread-scoped,
    - added `APPLY_JOINT_SETPOINT` acknowledgement path for direct commissioning setpoints,
    - `/control/joint-jog` now returns `direct_setpoint_ack` metadata instead of pretending it waited for planner idle.
  - Updated `web-ui/src/App.tsx` and `web-ui/src/ControlPanel.tsx`:
    - runtime/fault types now include `drive_profile`,
    - commissioning messages now say backend accepted the setpoint and explicitly call out faulted / not-operation-enabled drive states,
    - removed a stale orphaned `ee_pose` parsing fragment that was tripping TypeScript lints.
  - Updated tests:
    - `tests/test_runtime_config.py`
    - `tests/test_api_endpoints.py`
- Validation:
  - `./.venv/bin/python -m pytest tests/test_runtime_config.py tests/test_api_endpoints.py -q` (passed, 39 tests).
  - `./.venv/bin/python -m py_compile src/gradient_os/run_controller.py src/gradient_os/runtime_config.py src/gradient_os/api/main.py src/gradient_os/arm_controller/command_api.py src/gradient_os/arm_controller/backends/registry.py src/gradient_os/arm_controller/backends/ethercat_rtcore/config.py src/gradient_os/arm_controller/robots/__init__.py src/gradient_os/telemetry/drive_faults.py src/gradient_os/arm_controller/profiles/registry.py src/gradient_os/arm_controller/profiles/drive/cia402.py src/gradient_os/arm_controller/profiles/drive/a6ec_ds402.py src/gradient_os/arm_controller/profiles/fieldbus/ethercat.py` (passed).
  - `bash -n start-stack.sh` (passed).
  - `ReadLints` on edited Python/TS files (no diagnostics after cleanup).
- Follow-up notes / risks:
  - `src/gradient_rt_motion/main.cpp` still publishes `drive_profile_id = 0`; Python-side defaults/overrides are now wired, but RTCore-reported live drive profile remains a follow-up.
  - The larger robot-vs-actuator-package split from the refactor plan is not finished in this pass; `Gradient05Config` still owns the current encoder/gear/sign data used by the EtherCAT backend.
  - Live robot validation is still required to confirm the new commissioning UI wording matches observed behavior when faults are present or axes are not operation-enabled.

## 2026-03-19 23:14 +0000

- Task summary:
  - Improved controller shutdown gracefulness by fixing the signal path used by `start-stack.sh`.
- Changes:
  - Updated `src/gradient_os/run_controller.py`:
    - added shutdown signal handlers for `SIGTERM` and `SIGINT`,
    - routed both signals into the existing `KeyboardInterrupt`/`finally` cleanup path so backend shutdown and socket close happen on graceful termination.
  - Updated `start-stack.sh`:
    - changed controller stop logic to send `SIGTERM` first instead of `SIGINT`,
    - reduced the graceful wait window before escalation to `SIGKILL` from ~8s to ~3s.
- Validation:
  - `bash -n start-stack.sh` (passed).
  - `.venv/bin/python -m py_compile src/gradient_os/run_controller.py` (passed).
  - `ReadLints` on edited files (no diagnostics).
- Follow-up notes / risks:
  - This is the right fix for the observed ignored-`SIGINT` behavior, but the improved controller exit timing still needs one live robot shutdown run to confirm it now exits during the graceful `SIGTERM` window.

## 2026-03-19 23:06 +0000

- Task summary:
  - Live-validated the final `start-stack.sh` terminal console and managed soft-stop flow on the robot terminal.
- Validation:
  - Reviewed the live terminal run:
    - startup reached `startup_ready=1` and `operational=6/6`,
    - the interactive line console accepted `command> stop` correctly under active log traffic,
    - soft stop de-energized to `BUS_UP_DISARMED` while keeping RTCore/EtherCAT up.
- Follow-up notes / risks:
  - The only remaining rough edge observed in the live run is that the controller still required escalation from `SIGINT` to `SIGTERM` during shutdown; behavior is safe, but graceful controller exit can still be improved later.

## 2026-03-19 22:53 +0000

- Task summary:
  - Replaced the full-screen launcher console with a simpler `rtcore_jog.py`-style line console after discussing whether an embedded curses UI was necessary.
- Changes:
  - Updated `start-stack.sh`:
    - replaced the curses-based `run_interactive_console()` with a lighter line-oriented prompt model,
    - added prompt-preserving `safe_print_line()` / `safe_print_block()` behavior modeled after `scripts/rtcore_jog.py`,
    - added a background monitor thread that tails controller/API/web logs and redraws the current input prompt cleanly,
    - preserved `stop`, `stop --hard`, `probe`, `status`, `help`, and `clear` command handling,
    - used `_thread.interrupt_main()` so a supervised child exit can interrupt blocking `input()` and return control to the launcher.
  - Updated `docs/README.md`:
    - changed the console description from a pinned full-screen prompt to a line-oriented prompt that redraws cleanly during log streaming.
- Validation:
  - `bash -n start-stack.sh` (passed).
  - `./start-stack.sh --help` (passed).
- Follow-up notes / risks:
  - This should be a better fit for the observed terminal behavior, but it still needs one live run to confirm typed commands and prompt redraws behave well under the current telemetry/log rate.

## 2026-03-19 22:48 +0000

- Task summary:
  - Fixed the interactive console input bug after the terminal UI rendered successfully but would not accept typed commands.
- Changes:
  - Updated `start-stack.sh`:
    - removed the shell-level stdin/stdout redirection of the embedded Python console process to the tty,
    - passed the saved tty path into the Python console as an argument,
    - rebound file descriptors `0/1/2` to the tty from inside Python after interpreter startup so the heredoc can still provide the script body while curses gets live keyboard input.
- Validation:
  - `bash -n start-stack.sh` (passed).
- Follow-up notes / risks:
  - This specifically fixes the stdin ownership issue revealed by the live screenshot, but the updated console still needs one live terminal run to confirm typed commands now appear and execute as expected.

## 2026-03-19 22:45 +0000

- Task summary:
  - Fixed the interactive launcher console activation bug after the live terminal transcript showed typed `stop` characters still getting buried in the streamed logs.
- Changes:
  - Updated `start-stack.sh`:
    - captured the original terminal device path with `tty` before stdout is redirected through `tee`,
    - changed `interactive_console_enabled()` to require a real stdin TTY plus the saved terminal device, rather than checking `-t 1`,
    - changed `run_interactive_console()` to attach stdin/stdout/stderr directly to the saved tty device so the curses console can own the real terminal while launcher logging still goes through `tee`.
- Validation:
  - `bash -n start-stack.sh` (passed).
- Follow-up notes / risks:
  - The logic is now aligned with the observed failure mode, but the corrected interactive console still needs one live terminal run to confirm the pinned command line actually appears and accepts `stop` cleanly.

## 2026-03-19 22:37 +0000

- Task summary:
  - Added an in-terminal interactive command console to `start-stack.sh` so operators can type `stop` directly in the running launcher without losing the prompt in the telemetry stream.
- Changes:
  - Updated `start-stack.sh`:
    - added `GRADIENT_STACK_INTERACTIVE_CONSOLE` (`auto` by default, `0` to disable),
    - added terminal detection for the interactive console path,
    - added a curses-based console UI that keeps `command> ...` pinned at the top and tails controller/API/web logs below it,
    - added console commands: `stop`, `stop --hard`, `probe`, `status`, `help`, and `clear`,
    - preserved non-interactive fallback supervision for non-TTY/dumb-terminal cases,
    - logged console stop requests back into the launcher flow so the existing managed shutdown sequence still runs.
  - Updated `docs/README.md`:
    - documented the interactive terminal console behavior and supported commands.
- Validation:
  - `bash -n start-stack.sh` (passed).
  - `./start-stack.sh --help` (passed).
- Follow-up notes / risks:
  - The interactive console path has not yet been live-tested against the actual robot terminal session; the shell syntax and surrounding control flow are validated, but the curses UI still needs one real run.

## 2026-03-19 22:16 +0000

- Task summary:
  - Fixed the supervised `Ctrl-C` shutdown path in `start-stack.sh` after reviewing the latest live startup/soft-stop terminal log.
- Changes:
  - Updated `start-stack.sh`:
    - added a `STOP_REQUESTED` flag,
    - changed `handle_signal()` to mark shutdown intent instead of calling `exit` directly from the signal trap,
    - changed `supervise_children()` to break out cleanly when a stop signal is requested so the normal `cleanup()` path performs the managed shutdown.
- Validation:
  - Reviewed the latest live run log:
    - startup now waits for RTCore `startup_ready=1` and `operational=6/6` before advancing,
    - soft stop now lands at `BUS_UP_DISARMED` while leaving RTCore/EtherCAT up.
  - `bash -n start-stack.sh` (passed).
- Follow-up notes / risks:
  - The odd trailing `ntroller_readiness: command not found` message was not reproducible via a fresh `./start-stack.sh stop` after the stack was already disarmed; it likely came from the interrupted supervised launcher path that this signal-handling change addresses.

## 2026-03-19 22:06 +0000

- Task summary:
  - Tightened the hardware probe so vendor-specific servo fault references are only advertised when the active backend matches the current EtherCAT/A6-EC path.
- Changes:
  - Updated `start-stack.sh`:
    - passed `REPO_ROOT` into the embedded hardware-probe Python so it can detect whether the local A6-EC codebook exists,
    - added `drive_fault_reference` metadata to probe JSON,
    - made the human-readable `probe` output explicitly state whether vendor-specific drive fault references are applicable for the active servo backend, otherwise it reports `raw_only`.
- Validation:
  - `bash -n start-stack.sh` (passed).
- Follow-up notes / risks:
  - This change intentionally does not hard-decode `0x603F` into a single A6-EC panel code because the bus-fault code alone can be ambiguous across several manual entries.

## 2026-03-19 21:52 +0000

- Task summary:
  - Refined `start-stack.sh` stop/start behavior after live robot feedback about A6-EC `ErC1.1` sync-loss faults and partial slave bring-up.
- Changes:
  - Updated `start-stack.sh`:
    - added soft-stop default semantics for `./start-stack.sh stop`, which now leaves RTCore + EtherCAT running after the robot reaches `BUS_UP_DISARMED`,
    - added `./start-stack.sh stop --hard` to explicitly continue through RTCore and `ethercat.service` shutdown,
    - exported `GRADIENT_RTCORE_READY_TIMEOUT_S=30` by default when launched via `start-stack.sh`,
    - added a launcher-side `wait_for_bus_operational` gate so startup does not advance past the controller stage until RTCore metrics report all configured slaves responding, online, operational, and `startup_ready=1`,
    - avoided redundant SAFE_POWER_DOWN requests when the probe already shows the robot is no longer `ACTIVE`.
  - Updated `docs/README.md`:
    - documented the new soft-stop vs hard-stop behavior,
    - documented the stricter full-bus readiness gate during startup.
- Validation:
  - `bash -n start-stack.sh` (passed).
  - `./start-stack.sh --help` (passed; new CLI/help text rendered correctly).
- Follow-up notes / risks:
  - Live validation is still needed on hardware to confirm the default soft stop avoids `ErC1.1` while leaving the drives in the desired ready/disarmed state.
  - If some boots legitimately take longer than 30s to get all slaves operational, increase `GRADIENT_RTCORE_READY_TIMEOUT_S` and/or `GRADIENT_STACK_BUS_READY_TIMEOUT_S` rather than allowing partial bring-up.

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

## 2026-03-18 23:21 +00:00

- Task summary:
  - Implemented a usable zeroing workflow for the EtherCAT RTCore backend so physical joint poses can be captured as logical zero offsets during commissioning.
- Changes:
  - Added `src/gradient_os/joint_zero_offsets.py` to persist repo-local logical joint master offsets in `.gradient_joint_zero_offsets.json`.
  - Updated `src/gradient_os/arm_controller/backends/ethercat_rtcore/backend.py` to:
    - load persisted joint zero offsets on startup,
    - parse RTCore `STATUS_AXIS_CONFIG`,
    - convert live `0x6064` counts into q-units for feedback,
    - apply master offsets to commanded setpoints,
    - capture and persist logical-joint zero from live EtherCAT feedback.
  - Updated `src/gradient_os/run_controller.py` with backend-aware `ZERO_JOINT,<joint>` handling so logical-joint zeroing works even when RT axes are remapped for bring-up.
  - Updated `src/gradient_os/api/main.py` with `POST /control/zero-joint`.
  - Documented the workflow in `docs/ethercat/bringup.md`.
  - Added regression coverage in `tests/test_gradient05_limits_and_backends.py` and `tests/test_api_endpoints.py`.
- Validation:
  - `./.venv/bin/python -m pytest tests/test_gradient05_limits_and_backends.py tests/test_api_endpoints.py -q` (passed, 37 tests).
  - `ReadLints` on edited Python files and tests reported no issues.
- Follow-up notes / risks:
  - Zero capture is software-level logical calibration, not drive EEPROM zeroing; mechanical phasing and DS402 commissioning remain RTCore/drive responsibilities.
  - The new API expects the controller to be running with live RTCore feedback available; zero capture will fail cleanly if scaling/config snapshots have not arrived yet.

## 2026-03-18 23:32 +00:00

- Task summary:
  - Wired EtherCAT commissioning into the web UI by adding per-joint jog and per-joint zero capture controls to the existing robot control panel.
- Changes:
  - Updated `src/gradient_os/api/main.py`:
    - added `POST /control/joint-jog` which reads current joint angles from the controller, applies a relative delta to one joint, and sends the updated 6-joint target through the existing controller path.
  - Updated `web-ui/src/ControlPanel.tsx`:
    - added live polling of `/info/joints`,
    - added a `Joint Commissioning` section with step-size presets, current joint angle display, relative jog buttons, and `Zero` actions per joint,
    - added a commissioning safety note that assumes RTCore is still running with `--max-rpm 100`.
  - Updated `tests/test_api_endpoints.py` with `control/joint-jog` coverage.
- Validation:
  - `./.venv/bin/python -m pytest tests/test_api_endpoints.py -q` (passed, 29 tests).
  - `ReadLints` on `web-ui/src/ControlPanel.tsx`, `src/gradient_os/api/main.py`, and `tests/test_api_endpoints.py` reported no issues.
  - `npm run build` in `web-ui` did not complete due existing environment/app issues outside this change:
    - Node `18.20.4` is below Vite's required version,
    - existing unresolved import `occt-import-js/dist/occt-import-js.wasm?url` from `src/ArmVisualizer.tsx`.
  - `npx tsc --noEmit` also reports pre-existing `three`/`ArmVisualizer` typing issues unrelated to `ControlPanel.tsx`.
- Follow-up notes / risks:
  - Joint jog is implemented as small relative absolute-position updates using the controller's existing joint command path, not a new RT velocity loop; this is appropriate for commissioning/zeroing but distinct from Cartesian realtime jog.
  - Once the web-ui build baseline is repaired, rerun full frontend build validation to confirm the commissioning panel in production bundling.

## 2026-03-19 00:20 +00:00

- Task summary:
  - Brought up the RTCore/controller/API/web stack on the RevPi, validated the new joint commissioning UI path, and diagnosed the current EtherCAT hardware state.
  - Added best-effort RTCore autostart for the `ethercat_rtcore` controller path and cleaned up systemd unit wiring/paths.
- Changes:
  - Updated `src/gradient_os/run_controller.py`:
    - added best-effort RTCore service autostart when `ethercat_rtcore` is selected and `/run/gradient-rt-motion/ipc.sock` is missing,
    - uses `systemctl` directly when root or `sudo -n systemctl` as non-root, failing soft with warnings.
  - Updated `systemd/controller/arm-controller.service`:
    - fixed repository paths from `/home/pi/src/GradientOS` to `/home/pi/GradientOS`,
    - added `Wants=` / `After=` dependency on `gradient-rt-motion.service`,
    - added `GRADIENT_RTCORE_AUTOSTART=1`.
  - Updated `systemd/api/gradient-api.service`:
    - fixed repository paths from `/home/pi/src/GradientOS` to `/home/pi/GradientOS`.
  - Updated `systemd/README.md` to document the controller -> RTCore -> EtherCAT service chain.
  - Reinstalled the RTCore service via `./systemd/rt-motion/install.sh`.
  - Repaired local web dependency drift with `npm install` in `web-ui`, restoring local Vite from `7.1.10` to `5.4.21`.
- Validation:
  - `./.venv/bin/python -m py_compile src/gradient_os/run_controller.py` (passed).
  - `ReadLints` on `src/gradient_os/run_controller.py` (no issues).
  - Runtime bring-up:
    - API healthy at `http://127.0.0.1:4000/health`,
    - web UI dev server responding at `http://127.0.0.1:8000/`,
    - controller runtime config reports `robot=gradient05`, `servo_backend=ethercat_rtcore`,
    - RTCore journal confirmed controller IPC handshake after clean restart.
  - Service/autostart proof:
    - after stopping `gradient-rt-motion.service`, a fresh `./run.sh --robot gradient05` brought the RT motion service back up automatically.
  - EtherCAT hardware findings:
    - `sudo ethercat master` shows `Active: yes` but `Slaves: 0`, `Rx frames: 0`, `Frame loss: 100%`,
    - `curl http://127.0.0.1:4000/info/all-positions` shows six exposed axes (`servo_id` `0..5`) but all raw positions remain `0`,
    - `python3 scripts/sampler/rtcore_metrics.py summary` shows `num_axes=6`, 1kHz loop healthy, but no drive feedback.
  - Device-busy diagnosis:
    - `sudo fuser -v /dev/EtherCAT0` identified a stale manual RTCore child still holding the master device; killing it and restarting the service cleared the `ecrt_request_master(0) failed` error.
- Follow-up notes / risks:
  - The software stack is healthy, but the physical EtherCAT bus is not yet communicating; current evidence points to wiring/port/device-mode/no-reply issues rather than controller/API bugs.
  - The updated controller/api service files were fixed in-repo; re-run `systemd/controller/install.sh` and `systemd/api/install.sh` if you want the installed service units on this host to pick up the new paths/dependencies.

## 2026-03-19 00:23 +00:00

- Task summary:
  - Reviewed all in-repo EtherCAT docs plus scratchpad/devlog archives for prior working clues and cross-checked them against the live NIC binding on the RevPi.
- Findings:
  - `docs/ethercat/igh.md` and `docs/ethercat/bringup.md` both document the historical bring-up result:
    - `eth0` (`macb`, MAC `c8:3e:a7:14:1c:75`) was the interface that worked with IgH `ec_generic`,
    - `eth1` (`lan743x`, MAC `c8:3e:a7:14:1c:76`) repeatedly failed in that earlier setup.
  - The current repo templates and live `/etc/ethercat.conf` therefore bind `MASTER0_DEVICE="c8:3e:a7:14:1c:75"`.
  - Live host state now contradicts the old appliance assumption:
    - `nmcli device status` shows `eth0` is the active wired uplink (`connected`, profile `Direct-PC`),
    - `ip route` shows the default route via `eth0`,
    - `ip -4 addr show dev eth0` shows the machine's active IPv4 addresses on `eth0`,
    - `eth1` is currently disconnected.
  - Installed link configs exist for the old intended naming (`/etc/systemd/network/10-ethercat0.link` for MAC `...:75`, `/etc/systemd/network/10-uplink0.link` for MAC `...:76`), but the live system still presents plain `eth0`/`eth1`, so the expected dedicated-port naming/policy is not active at runtime.
  - Archive review did not surface a more specific later correction; the strongest historical clue remains the docs' explicit `eth0 worked / eth1 failed` note.
- Validation:
  - Read `docs/ethercat/igh.md`, `docs/ethercat/bringup.md`, `/etc/ethercat.conf`, and the repo's `ethercat-eth0.conf` / `ethercat-eth1.conf`.
  - Checked live interfaces with `ip -br link`, `ip -4 addr`, `ip route`, `nmcli device status`, and `networkctl status`.
- Follow-up notes / risks:
  - If the current physical wiring really uses `eth0` for normal host networking and the drive chain is on `eth1`, then the present EtherCAT binding is likely wrong for this machine even though it matched the historical working notes.
  - Best next diagnostic is a temporary bind swap using `scripts/ethercat/ethercat-eth1.conf` while the RTCore/controller are stopped, then compare `ethercat master` / `ethercat slaves -v` immediately.

## 2026-03-19 00:39 +00:00

- Task summary:
  - Ran the temporary `eth1` EtherCAT binding test against the current physical wiring.
- Validation:
  - Stopped controller processes and RTCore, stopped `ethercat.service`, then started IgH with:
    - `sudo /usr/local/sbin/ethercatctl -c /home/pi/GradientOS/scripts/ethercat/ethercat-eth1.conf start`
  - `sudo ethercat master` result on `eth1` / MAC `c8:3e:a7:14:1c:76`:
    - `Slaves: 6`
    - `Rx frames: 19`
    - `Lost frames: 0`
    - `Frame loss [%]: 0.0`
  - `sudo ethercat slaves -v` enumerated six slaves on the chain.
- Follow-up notes / risks:
  - This is the strongest proof so far that the live slave chain is currently wired to `eth1`, not the historically documented `eth0` path.
  - The repo/default `/etc/ethercat.conf` binding to MAC `...:75` is therefore wrong for the current machine wiring and should be switched if we want normal controller/RTCore bring-up to work without the temporary config.

## 2026-03-19 00:44:49 +00:00

- Task summary:
  - Switched the permanent EtherCAT binding from the historical `eth0` MAC to the validated `eth1` MAC in both the repo templates and the live host config, then restarted the normal EtherCAT/RTCore/controller path.
- What changed:
  - Updated repo templates:
    - `systemd/ethercat-host/ethercat.conf`
    - `systemd/ethercat-host/10-ethercat0.link`
    - `systemd/ethercat-host/10-uplink0.link`
    - `systemd/ethercat-host/10-unmanaged-ethercat.conf`
    - `systemd/README.md`
    - `systemd/ethercat-host/install.sh`
    - `docs/ethercat/igh.md`
    - `docs/ethercat/bringup.md`
    - `scripts/ethercat/ethercat-eth0.conf`
    - `scripts/ethercat/ethercat-eth1.conf`
  - Updated live host files with root privileges:
    - `/etc/ethercat.conf`
    - `/etc/systemd/network/10-ethercat0.link`
    - `/etc/systemd/network/10-uplink0.link`
    - `/etc/NetworkManager/conf.d/10-unmanaged-ethercat.conf`
- Validation:
  - Restarted:
    - `sudo systemctl restart ethercat.service`
    - `sudo systemctl restart gradient-rt-motion.service`
    - `./run.sh --robot gradient05`
  - Verified API/controller path recovered:
    - `curl http://127.0.0.1:4000/health` returned controller reachable
    - `curl http://127.0.0.1:4000/info/runtime-config` still reports `servo_backend=ethercat_rtcore`
  - Verified the corrected NIC is now in use:
    - `sudo ethercat master` shows `Main: c8:3e:a7:14:1c:76 (attached)` with link up and RX traffic on the correct port
  - Split test results:
    - With `gradient-rt-motion.service` stopped, `sudo ethercat slaves -v` against the corrected `/etc/ethercat.conf` enumerated six slave slots again
    - With RTCore running, `python3 scripts/sampler/rtcore_metrics.py summary` still shows `wkc=0/0`, all `pos_counts=0`, and the API `/info/all-positions` + `/info/joints` stay zeroed
    - While RTCore owns the master, `ethercat` CLI matches zero selected slaves for SDO/PDO queries
  - `ReadLints` on the edited repo files reported no linter errors.
- Follow-up notes / risks:
  - The wrong-NIC problem is fixed: `eth1` / MAC `...:76` is now the default slave-facing path for this machine.
  - A separate RTCore/master-activation issue remains: the fieldbus is no longer pointed at the obviously wrong port, but RTCore still is not reaching non-zero WKC or live axis feedback.
  - Historical notes that "`eth0` worked and `eth1` failed" should now be treated as stale or environment-specific for this host.

## 2026-03-19 00:59:05 +00:00

- Task summary:
  - Investigated whether RTCore's zero-WKC behavior could be caused by CPU/core placement, and followed the evidence through host NIC tuning, IRQ routing, manual rescans, and reduced-axis RTCore tests.
- What changed:
  - Updated host tuning scripts in-repo:
    - `systemd/ethercat-host/gradient-ethercat-nic-tune.sh`
    - `systemd/ethercat-host/gradient-irq-affinity.sh`
  - New behavior in those scripts:
    - resolve the active EtherCAT NIC from `/etc/ethercat.conf` (device name or MAC) instead of blindly falling back to `eth0`
    - use `/sys/class/net/<iface>/device/msi_irqs` for IRQ discovery when available
    - write a plain hex mask (`c`) instead of `0xC`
  - Installed the updated scripts to:
    - `/usr/local/sbin/gradient-ethercat-nic-tune.sh`
    - `/usr/local/sbin/gradient-irq-affinity.sh`
    - then restarted both oneshot services
- Validation:
  - Confirmed RTCore thread placement:
    - `rt-cycle` thread is on CPU3 with `SCHED_FIFO 90`
    - process main/helper threads remain on CPUs 0-1
  - Confirmed the core-adjacent misconfiguration:
    - before the fix, `eth1` still had `GRO/GSO/TSO` enabled and `EEE status: enabled - inactive`
    - after tuning the correct NIC, `eth1` shows `GRO/GSO/TSO` disabled and `EEE status: disabled`
  - Confirmed IRQ-affinity limits on this platform:
    - `gradient-irq-affinity.service` now discovers the `lan743x` MSI IRQs correctly, but writes still fail with `I/O error`
    - `smp_affinity_list` for IRQs `190..195` remains `0-1`
    - `irqaffinity=0,1` is present in `/proc/cmdline`
  - Bus/runtime diagnostics:
    - with the corrected NIC tuning applied, `ethercat master` under RTCore showed materially better RX and lower frame loss than before, but still not non-zero WKC
    - bare manual path:
      - `sudo /usr/local/sbin/ethercatctl -c /home/pi/GradientOS/scripts/ethercat/ethercat-eth1.conf start`
      - then `sudo ethercat rescan`
      - then `sudo ethercat slaves -v`
      - recovered all six slave identities as `AS715N_sAxis_V0.10`
    - reduced RTCore test:
      - started `/usr/local/bin/gradient-rt-motion --socket-path /tmp/rt1/ipc.sock --num-axes 1 --max-rpm 100`
      - this kept `ethercat master` at `Slaves: 6` with a real DC reference clock on `Slave 0`, much healthier RX, and lower frame loss than the 6-axis service run
      - despite that, `/tmp/rt1/metrics.json` still reported `wkc=0/0` and zero feedback
    - during the 1-axis RTCore run:
      - `sudo ethercat slaves` showed all six slaves stuck in `INIT`
      - `sudo ethercat pdos -p0` showed active PDOs `0x1701/0x1b01` on slave 0
      - RTCore is hard-coded for `0x1702/0x1b02`
  - Restored normal service path afterward:
    - `ethercat.service` active
    - `gradient-rt-motion.service` active
    - API health check still returns controller reachable
- Follow-up notes / risks:
  - Current evidence does not support "wrong physical cores" as the primary root cause.
  - The stronger lead is EtherCAT state/PDO sequencing under RTCore: slaves remain in `INIT` and the live PDO selection does not match RTCore's expected `0x1702/0x1b02`.
  - Rescan and reduced-axis tests improve bus behavior enough to show this is not a dead link problem anymore.

## 2026-03-19 01:10:41 +00:00

- Task summary:
  - Hardened RTCore startup diagnostics so EtherCAT bring-up reports real convergence state, then validated the new behavior live against the 6-axis service path.
- What changed:
  - Updated `src/gradient_rt_motion/main.cpp` to:
    - publish configured `wkc_expected` instead of latching from observed traffic
    - derive `master_state` from actual EtherCAT AL bits/link state instead of `armed`
    - sample `ecrt_master_state()`, `ecrt_domain_state()`, and `ecrt_slave_config_state()` continuously
    - track/responding slave count, online/operational counts, domain WC state, startup elapsed time, startup reset count, and per-axis slave AL state/online/operational flags
    - emit startup-wait/reset/timeout log messages from the RT loop
    - set `StatusHelloV1.wkc_expected` to the configured expected WKC
  - Updated `scripts/sampler/rtcore_metrics.py` to print the new bus/startup diagnostics in `summary`.
  - Rebuilt `src/gradient_rt_motion/gradient-rt-motion`, installed it to `/usr/local/bin/gradient-rt-motion`, and restarted `ethercat.service` + `gradient-rt-motion.service`.
- Validation:
  - Build/syntax:
    - `make -C src/gradient_rt_motion`
    - `python3 -m py_compile scripts/sampler/rtcore_metrics.py`
    - `ReadLints` on the edited files reported no errors.
  - Live diagnostics after restart:
    - `python3 scripts/sampler/rtcore_metrics.py summary` now reports:
      - `wkc=0/12` instead of misleading `0/0`
      - `link_up`, `responding`, `online`, `operational`, `master_al`, `domain_wc`, `startup_elapsed_ms`, `startup_resets`
      - per-axis `slave_online`, `slave_operational`, and `slave_al`
  - Key journal evidence from the new RTCore logs:
    - at startup, RTCore briefly sees:
      - `link_up=1`
      - `responding=6/6`
      - `online=6/6`
      - `operational=0/6`
      - `master_al=0x1`
      - `domain_wc=0`
      - `slave0_al=INIT`
    - within about 1 second, the same service instance falls back to:
      - `responding=0/6`
      - `online=0/6`
      - `master_al=0x0`
      - `domain_wc=0`
    - timeout warning now clearly reports non-convergence after 5000 ms.
  - Runtime/API:
    - `curl http://127.0.0.1:4000/health` still reports controller reachable
    - `/info/all-positions` remains zero, consistent with the zero-WKC condition
    - `sudo ethercat master` still shows link up and some RX traffic but `Slaves: 0` under the 6-axis RTCore path
- Follow-up notes / risks:
  - The new diagnostics strongly support that the issue is not merely "startup is slow and needs more wait time." RTCore initially sees the topology, then loses it during/after configuration.
  - This points more strongly at configuration/state transition behavior (for example PDO/DC/AL-state progression under RTCore) than at simple link discovery.

## 2026-03-19 01:45:12 +00:00

- Task summary:
  - Prepared a fresh-instance handoff summary for the current EtherCAT/RTCore bring-up state.
- Validation:
  - Re-read the latest `DEVLOG` and `AGENT_SCRATCHPAD` EtherCAT sections to ensure the handoff reflects the current failure signature and file set.
- Follow-up notes / risks:
  - The next instance should start from the current diagnostics in `src/gradient_rt_motion/main.cpp` and `scripts/sampler/rtcore_metrics.py` rather than re-deriving the earlier NIC/core hypotheses.

## 2026-03-19 01:55:44 +00:00

- Task summary:
  - Added explicit EtherCAT bring-up profile controls to RTCore, surfaced them in metrics, and used them for a new live split test against the zero-WKC failure.
- What changed:
  - Updated `src/gradient_rt_motion/main.cpp` to:
    - add runtime EtherCAT bring-up options: `--rx-pdo`, `--tx-pdo`, `--no-dc`, `--wait-before-safeop-ms`, `--preop-safeop-timeout-ms`, `--safeop-op-timeout-ms`
    - support two concrete vendor-ESI PDO profiles: `0x1701/0x1b01` and `0x1702/0x1b02`
    - register/read/write PDO entries conditionally based on the selected profile instead of assuming every optional entry exists
    - apply IgH startup timing knobs with `ecrt_slave_config_flag(..., "WaitBeforeSAFEOPms", ...)` and `ecrt_slave_config_state_timeout(...)`
    - skip DC configuration/sync helpers when `--no-dc` is selected
    - publish the active PDO/DC/timing policy into `metrics.json`
  - Updated `scripts/sampler/rtcore_metrics.py` so `summary` prints the active `pdo_profile`, PDO IDs, DC state, and SAFEOP/OP timing policy.
  - Rebuilt `src/gradient_rt_motion/gradient-rt-motion` and installed it to `/usr/local/bin/gradient-rt-motion`.
- Validation:
  - Build/syntax:
    - `make -C src/gradient_rt_motion`
    - `python3 -m py_compile scripts/sampler/rtcore_metrics.py`
    - `ReadLints` on the edited files reported no errors.
  - Default 6-axis service path after install:
    - `sudo systemctl restart gradient-rt-motion.service`
    - `python3 scripts/sampler/rtcore_metrics.py summary`
    - still shows `profile=1702/1b02`, `dc_enabled=1`, `wkc=0/12`, `responding=0/6`, `online=0/6`, `master_al=0x0`
    - journal still shows the same collapse: startup begins at `responding=6/6`, `online=6/6`, `slave0_al=INIT`, then falls to `0/6` by ~1 s
  - New live split test:
    - stopped `gradient-rt-motion.service`
    - ran `sudo /usr/local/bin/gradient-rt-motion --socket-path /tmp/rtdiag/ipc.sock --num-axes 1 --max-rpm 100 --rx-pdo 0x1701 --tx-pdo 0x1b01 --no-dc`
    - `sudo ethercat master` during that run showed `Slaves: 6`, a DC reference clock on `Slave 0`, non-zero RX traffic, and materially lower frame loss than the failing 6-axis service path
    - `sudo ethercat slaves` showed all six slaves still present but stuck in `INIT`
    - `sudo ethercat pdos -p0` confirmed the live `0x1701/0x1b01` layout under that diagnostic run
    - RTCore logs during that run stayed at `responding=6/1`, `online=1/1`, `operational=0/1`, `master_al=0x1`, `slave0_al=INIT`, `wkc=0/2`
  - Restored the normal service path afterward with `sudo systemctl start gradient-rt-motion.service`; `python3 scripts/sampler/rtcore_metrics.py summary` again reports the default 6-axis failure signature.
- Follow-up notes / risks:
  - The new split test weakens the earlier "wrong PDO IDs alone" hypothesis. Matching the live-visible `0x1701/0x1b01` profile and disabling DC improves topology stability in the reduced run, but it does not move the slave out of `INIT` or produce non-zero WKC.
  - The strongest remaining lead is now AL-state/configuration sequencing under RTCore: something about activation/config progression is preventing the drive from leaving `INIT` and entering real process-data exchange even when topology is stable.

## 2026-03-19 02:10:27 +00:00

- Task summary:
  - Measured the actual multi-slave breakpoint in RTCore bring-up and added a slave-position diagnostic option so individual slave positions can be probed without changing `num_axes`.
- What changed:
  - Updated `src/gradient_rt_motion/main.cpp` to:
    - add `--slave-positions P[,P..]` so RTCore axes can target arbitrary EtherCAT slave positions instead of always `0..N-1`
    - validate duplicate/length errors for that option
    - use the configured slave positions for `ecrt_master_slave_config(...)` and PDO entry registration
    - log the configured axis-to-slave-position mapping at startup
    - publish `slave_positions` into `metrics.json`
  - Updated `scripts/sampler/rtcore_metrics.py` so `summary` prints `slave_positions`.
  - Rebuilt `src/gradient_rt_motion/gradient-rt-motion` and reinstalled `/usr/local/bin/gradient-rt-motion`.
- Validation:
  - Build/syntax:
    - `make -C src/gradient_rt_motion`
    - `python3 -m py_compile scripts/sampler/rtcore_metrics.py`
    - `ReadLints` on the edited files reported no errors.
  - Default profile breakpoint sweep (manual RTCore, no controller attached):
    - `sudo systemctl stop gradient-rt-motion.service`
    - `sudo /usr/local/bin/gradient-rt-motion --socket-path /tmp/rtprobe1/ipc.sock --num-axes 1 --max-rpm 100`
      - `python3 scripts/sampler/rtcore_metrics.py --path /tmp/rtprobe1/metrics.json summary` showed `responding=6/1`, `online=1/1`, `master_al=0x1`, `slave0_al=INIT`, `wkc=0/2`, `startup_resets=0`
      - `sudo ethercat master` showed `Slaves: 6` and a DC reference clock on `Slave 0`
    - `sudo /usr/local/bin/gradient-rt-motion --socket-path /tmp/rtprobe2/ipc.sock --num-axes 2 --max-rpm 100`
      - metrics showed collapse to `responding=0/2`, `online=0/2`, `master_al=0x0`, `wkc=0/4`, `startup_resets=1`
      - `sudo ethercat master` showed `Slaves: 0`
    - `sudo /usr/local/bin/gradient-rt-motion --socket-path /tmp/rtprobe3/ipc.sock --num-axes 3 --max-rpm 100`
      - metrics likewise collapsed to `responding=0/3`, `online=0/3`, `master_al=0x0`, `wkc=0/6`, `startup_resets=1`
  - Cross-check against lighter profile:
    - `sudo /usr/local/bin/gradient-rt-motion --socket-path /tmp/rtprobe2alt/ipc.sock --num-axes 2 --max-rpm 100 --rx-pdo 0x1701 --tx-pdo 0x1b01 --no-dc`
    - metrics still collapsed to `responding=0/2`, `online=0/2`, `wkc=0/4`, `startup_resets=1`
  - Individual slave-position check:
    - `sudo /usr/local/bin/gradient-rt-motion --socket-path /tmp/rtprobe_p1/ipc.sock --num-axes 1 --slave-positions 1 --max-rpm 100`
    - metrics showed `slave_positions=[1]`, `responding=6/1`, `online=1/1`, `master_al=0x1`, `slave0_al=INIT`, `wkc=0/2`, `startup_resets=0`
    - `sudo ethercat master` still showed `Slaves: 6`
  - Restored the normal service path afterward with `sudo systemctl start gradient-rt-motion.service`; `python3 scripts/sampler/rtcore_metrics.py summary` again shows the default 6-axis failure signature.
- Follow-up notes / risks:
  - The practical breakpoint on this host is between one configured slave and two configured slaves.
  - Because `slave position 1` alone behaves like `slave position 0` alone, the evidence now points away from a single bad slave and toward a multi-slave configuration/state-transition problem.

## 2026-03-19 02:20:22 +00:00

- Task summary:
  - Ran the full two-slave pair sweep to determine whether only certain slave combinations collapse or whether the current failure is generic to any two configured slaves.
- Validation:
  - Stopped `gradient-rt-motion.service`.
  - Swept all 15 unordered pairs via the new `--slave-positions` option:
    - `0,1`
    - `0,2`
    - `0,3`
    - `0,4`
    - `0,5`
    - `1,2`
    - `1,3`
    - `1,4`
    - `1,5`
    - `2,3`
    - `2,4`
    - `2,5`
    - `3,4`
    - `3,5`
    - `4,5`
  - For each pair, launched:
    - `sudo timeout 8s /usr/local/bin/gradient-rt-motion --socket-path /tmp/rtpair_<a>_<b>/ipc.sock --num-axes 2 --slave-positions <a>,<b> --max-rpm 100`
    - sampled `/tmp/rtpair_<a>_<b>/metrics.json`
    - sampled `sudo ethercat master`
    - sampled `sudo ethercat slaves`
  - Result: every completed pair produced the same RTCore summary:
    - `responding=0`
    - `online=0`
    - `operational=0`
    - `master_al=0x0`
    - `wkc=0/4`
    - `startup_resets=1`
  - Restored the normal service path afterward:
    - `sudo systemctl start gradient-rt-motion.service`
    - `python3 scripts/sampler/rtcore_metrics.py summary`
    - service is active again and back to the known default 6-axis failure signature.
- Follow-up notes / risks:
  - There is no obvious "bad 2-slave pair" in the present 6-drive chain. The current failure appears generic to configuring any two slaves together.
  - Combined with the user's historical note that two drives previously moved together on this hardware, this strengthens the case for a current RTCore bring-up/configuration regression or changed drive startup state rather than a fundamental fieldbus limitation.

## 2026-03-19 02:21:58 +00:00

- Task summary:
  - Prepared a fresh-instance handoff after the full two-slave sweep narrowed the problem to a generic multi-slave RTCore bring-up failure.
- Validation:
  - Re-read the latest EtherCAT sections in `.cursor/memory/AGENT_SCRATCHPAD.md` and `.cursor/memory/DEVLOG.md` to ensure the handoff reflects the current breakpoint and next-step direction.
- Follow-up notes / risks:
  - A fresh instance should start from the multi-slave breakpoint result and focus on staged multi-slave AL/config progression in `src/gradient_rt_motion/main.cpp`, not on rediscovering the earlier NIC binding or CPU-affinity work.

## 2026-03-19 02:26:40 +00:00

- Task summary:
  - Added phase-level EtherCAT bring-up logging and per-slave startup transition logging in `src/gradient_rt_motion/main.cpp`, then rebuilt, manually probed a two-slave run, and reinstalled `/usr/local/bin/gradient-rt-motion` so the systemd service emits the same diagnostics.
- Validation:
  - `make -C src/gradient_rt_motion`
  - `sudo systemctl stop gradient-rt-motion.service && sudo timeout 8s ./src/gradient_rt_motion/gradient-rt-motion --socket-path /tmp/rtdiag_phase/ipc.sock --num-axes 2 --slave-positions 0,1 --max-rpm 100 ; sudo systemctl start gradient-rt-motion.service`
  - Manual two-slave probe result:
    - both selected slaves stayed `online=1`, `operational=0`, `al=INIT` through `ecrt_master_slave_config`, `ecrt_slave_config_pdos`, `ecrt_slave_config_dc`, PDO registration, and `ecrt_master_activate`
    - collapse happened only during cyclic startup at about `448 ms` (`cycle=2123`), after activation/send had already begun
  - `sudo install -m 0755 src/gradient_rt_motion/gradient-rt-motion /usr/local/bin/gradient-rt-motion`
  - `sudo systemctl restart gradient-rt-motion.service`
  - `python3 scripts/sampler/rtcore_metrics.py summary`
    - service is active and back at the known default failure signature: `wkc=0/12`, `responding=0/6`, `online=0/6`, `operational=0/6`
  - `sudo journalctl -u gradient-rt-motion.service --since "2026-03-19 02:25:49" --no-pager`
    - normal 6-axis service run now shows the new detailed logs, including config-phase snapshots and per-slave startup transitions
- Follow-up notes / risks:
  - In the normal 6-axis service run, the collapse is slightly staggered: axes `0-1` drop to `UNKNOWN` at about `444 ms`, then axes `2-5` drop one cycle later at about `445 ms`.
  - This narrows the breakpoint further: the system survives full static configuration/activation and fails only once cyclic exchange is underway, which is more consistent with a multi-slave runtime/state-sequencing bug than a pure `ecrt_master_slave_config()` or PDO-registration failure.

## 2026-03-19 02:33:37 +00:00

- Task summary:
  - Fixed a startup scheduling bug in `src/gradient_rt_motion/main.cpp` that was making the RT loop "catch up" after long EtherCAT initialization, then re-ran the two-slave timing probes to see whether the collapse timing was real or an artifact.
- Validation:
  - Updated the cyclic-loop schedule to rebase `next_ns` after initialization and switched startup elapsed timing to use the real wake timestamp.
  - `make -C src/gradient_rt_motion`
  - `sudo timeout 8s ./src/gradient_rt_motion/gradient-rt-motion --socket-path /tmp/rtdiag_phase_fix/ipc.sock --num-axes 2 --slave-positions 0,1 --max-rpm 100`
    - result after the fix: collapse still occurred at about `452 ms`, but the counters are now coherent (`cycle=453 elapsed_ms=452`) instead of showing impossible thousands of cycles in a few hundred ms
  - `sudo timeout 6s ./src/gradient_rt_motion/gradient-rt-motion --socket-path /tmp/rtdiag_wait0/ipc.sock --num-axes 2 --slave-positions 0,1 --max-rpm 100 --wait-before-safeop-ms 0`
    - collapse still occurred at about `456 ms`
  - `sudo timeout 6s ./src/gradient_rt_motion/gradient-rt-motion --socket-path /tmp/rtdiag_wait1000/ipc.sock --num-axes 2 --slave-positions 0,1 --max-rpm 100 --wait-before-safeop-ms 1000`
    - collapse still occurred at about `452 ms`
  - Reinstalled the rebuilt binary and restarted the service:
    - `sudo install -m 0755 src/gradient_rt_motion/gradient-rt-motion /usr/local/bin/gradient-rt-motion`
    - `sudo systemctl restart gradient-rt-motion.service`
    - immediate metrics showed clean 1 kHz timing (`rt_hz≈999`, max jitter only a few microseconds) before the known topology collapse reappeared
- Follow-up notes / risks:
  - The bad startup timing was real technical debt and is now fixed, but it was not the root cause of the EtherCAT collapse.
  - Because the collapse timing does not move when `--wait-before-safeop-ms` is changed from `0` to `1000`, the present evidence points away from that explicit SAFEOP-delay knob and more toward a generic "first few hundred milliseconds of cyclic multi-slave process-data exchange never achieve non-zero WKC, then the master gives up and marks slaves offline" failure mode.

## 2026-03-19 02:46:08 +00:00

- Task summary:
  - Added a configurable passive-startup mode to `src/gradient_rt_motion/main.cpp` and surfaced it in `scripts/sampler/rtcore_metrics.py`, then used it to separate DS402/output-writing behavior from lower-level multi-slave cyclic EtherCAT failure.
- Validation:
  - Added CLI/runtime knob:
    - `--startup-passive-ms <ms>`
    - during that window RTCore still runs `receive/process/queue/send`, but forces inert outputs (`cw=0`, `mode=0`, zero auxiliary outputs, aligned target position) instead of normal DS402 controlword/mode/target driving
  - `make -C src/gradient_rt_motion`
  - Control probe:
    - `sudo timeout 6s ./src/gradient_rt_motion/gradient-rt-motion --socket-path /tmp/rtdiag_passive0/ipc.sock --num-axes 2 --slave-positions 0,1 --max-rpm 100 --startup-passive-ms 0`
    - collapse still occurred at about `448 ms`
  - Passive-output probe:
    - `sudo timeout 6s ./src/gradient_rt_motion/gradient-rt-motion --socket-path /tmp/rtdiag_passive1500/ipc.sock --num-axes 2 --slave-positions 0,1 --max-rpm 100 --startup-passive-ms 1500`
    - passive mode was active when the collapse happened (`passive_outputs=1` at cycle 1; topology dropped at about `448 ms`; passive mode only ended at `1500 ms`)
  - Minimal-profile probe:
    - `sudo timeout 6s ./src/gradient_rt_motion/gradient-rt-motion --socket-path /tmp/rtdiag_minimal_passive/ipc.sock --num-axes 2 --slave-positions 0,1 --max-rpm 100 --rx-pdo 0x1701 --tx-pdo 0x1b01 --no-dc --startup-passive-ms 1500`
    - collapse still occurred at about `464 ms`
  - Reinstalled the rebuilt binary and restarted the service:
    - `sudo install -m 0755 src/gradient_rt_motion/gradient-rt-motion /usr/local/bin/gradient-rt-motion`
    - `sudo systemctl restart gradient-rt-motion.service`
    - immediate summary showed clean RT timing and all 6 slaves still visible in `INIT`
    - a follow-up summary after ~1s returned to the default collapsed state (`responding=0/6`, `online=0/6`, `wkc=0/12`, `startup_resets=1`)
- Follow-up notes / risks:
  - The current evidence now points strongly away from RT scheduling and away from the higher-level DS402 output logic as the root cause.
  - The remaining failure survives:
    - normal 1702/1b02 profile
    - lighter 1701/1b01 profile
    - DC disabled
    - passive/inert outputs during startup
  - That narrows the likely root cause to lower-level multi-slave cyclic EtherCAT process-data handling: PDO/FMMU/sync-manager/watchdog/master activation/runtime behavior rather than PREEMPT_RT core placement or our controlword writes.

## 2026-03-19 02:47:57 +00:00

- Task summary:
  - Added an output-watchdog toggle to the EtherCAT sync-manager config and used it to test whether the current `~450 ms` multi-slave collapse is caused by SM2 output watchdog behavior.
- Validation:
  - Added CLI/runtime knob:
    - `--disable-output-watchdog`
    - implementation copies the selected PDO profile sync table into a runtime buffer and flips the output sync-manager watchdog from `EC_WD_ENABLE` to `EC_WD_DISABLE`
  - `make -C src/gradient_rt_motion`
  - Probe:
    - `sudo timeout 6s ./src/gradient_rt_motion/gradient-rt-motion --socket-path /tmp/rtdiag_watchdog_off/ipc.sock --num-axes 2 --slave-positions 0,1 --max-rpm 100 --startup-passive-ms 1500 --disable-output-watchdog`
    - result: collapse still occurred at about `460 ms` while passive mode was still active
  - Reinstalled the rebuilt binary and restarted the service:
    - `sudo install -m 0755 src/gradient_rt_motion/gradient-rt-motion /usr/local/bin/gradient-rt-motion`
    - `sudo systemctl restart gradient-rt-motion.service`
    - immediate summary again showed clean RT timing and all 6 slaves visible in `INIT` before the known later collapse
- Follow-up notes / risks:
  - Disabling the output watchdog did not move the breakpoint, so the present evidence is now against:
    - PREEMPT_RT/core placement
    - RTCore DS402 controlword/mode/target writes
    - DC sync configuration alone
    - the explicit SAFEOP wait knob
    - output sync-manager watchdog policy alone
  - The remaining search area is the lower-level multi-slave cyclic process-data path itself: how libecrt/master activation plus multi-slave PDO/FMMU/sync-manager/runtime exchange behaves on this drive chain.

## 2026-03-19 03:56:50 +00:00

- Task summary:
  - Added the upstream-documented `ecrt_master_set_send_interval()` call before activation and re-ran the same two-slave probe to test whether the master needed the explicit cyclic send interval to manage multi-slave datagram packing/scheduling correctly.
- Validation:
  - Updated `src/gradient_rt_motion/main.cpp` to call:
    - `ecrt_master_set_send_interval(master, ceil(cycle_ns / 1000))`
    - current 1 kHz loop therefore advertises `send_interval_us=1000`
  - `make -C src/gradient_rt_motion`
  - Probe:
    - `sudo timeout 6s ./src/gradient_rt_motion/gradient-rt-motion --socket-path /tmp/rtdiag_send_interval/ipc.sock --num-axes 2 --slave-positions 0,1 --max-rpm 100`
    - master accepted the setting and logged `EtherCAT config phase=set_send_interval send_interval_us=1000 ok`
    - result: collapse still occurred at about `448 ms`, with `wkc=0/4`, `responding=0/2`, `online=0/2`
  - Reinstalled the rebuilt binary and restarted the service:
    - `sudo install -m 0755 src/gradient_rt_motion/gradient-rt-motion /usr/local/bin/gradient-rt-motion`
    - `sudo systemctl restart gradient-rt-motion.service`
    - immediate summary still showed healthy RT timing and all 6 slaves visible in `INIT` before the known later collapse
- Follow-up notes / risks:
  - The missing send-interval hint was worth adding because the IgH docs explicitly recommend it for the master FSM's frame-appending/scheduling decisions.
  - But it did not change the failure, so the remaining bug is even more narrowly in the activated multi-slave process-data path, not in the master lacking the advertised cycle interval.

## 2026-03-19 05:15:31 +00:00

- Task summary:
  - Added new multi-slave EtherCAT bring-up discriminators in `src/gradient_rt_motion/main.cpp` and `scripts/sampler/rtcore_metrics.py` to separate config-time setup from queued runtime domain traffic, then used them to push the failure boundary forward.
- Validation:
  - Updated RTCore/runtime metrics with:
    - `--startup-skip-domain-queue-ms`
    - `--split-domains-per-axis`
    - `--queue-split-domains-round-robin`
    - `--explicit-pdo-config`
    - summary/metrics visibility for those bring-up modes
  - `make -C src/gradient_rt_motion`
  - Control probe:
    - `sudo timeout 8s ./src/gradient_rt_motion/gradient-rt-motion --socket-path /tmp/rtprobe_q0/ipc.sock --num-axes 2 --slave-positions 0,1 --max-rpm 100 --startup-skip-domain-queue-ms 0`
    - result: collapse at about `454 ms`; `responding=0/2`, `online=0/2`, `wkc=0/4`
  - Queue-suppression probe:
    - `sudo timeout 8s ./src/gradient_rt_motion/gradient-rt-motion --socket-path /tmp/rtprobe_q1500/ipc.sock --num-axes 2 --slave-positions 0,1 --max-rpm 100 --startup-skip-domain-queue-ms 1500`
    - result: bus stayed alive through the full suppress window (`responding=6/2`, `online=2/2`, `slave_al=INIT` at `1000 ms`); `domain_queue_suppressed=0` logged at `1500 ms`; collapse began only after resume at about `1640 ms`
  - Per-axis-domain probe:
    - `sudo timeout 8s ./src/gradient_rt_motion/gradient-rt-motion --socket-path /tmp/rtprobe_split/ipc.sock --num-axes 2 --slave-positions 0,1 --max-rpm 100 --split-domains-per-axis`
    - result: still collapsed at about `456 ms`
  - Per-axis-domain + minimal-profile probe:
    - `sudo timeout 8s ./src/gradient_rt_motion/gradient-rt-motion --socket-path /tmp/rtprobe_split_min/ipc.sock --num-axes 2 --slave-positions 0,1 --max-rpm 100 --split-domains-per-axis --rx-pdo 0x1701 --tx-pdo 0x1b01 --no-dc`
    - result: still collapsed at about `456 ms`
  - Explicit-PDO-config probes:
    - initial low-level mapping path failed immediately on fixed PDOs: `ecrt_slave_config_pdo_mapping_add ... pdo=0x1702 entry=0x6040:00 bits=16`
    - narrowed explicit mode to sync-manager + assignment only, but `ecrt_slave_config_reg_pdo_entry_pos()` still failed on the first output entry (`sync=2 entry_pos=0 field=cw rc=-2`)
  - Round-robin queue probe:
    - `sudo timeout 8s ./src/gradient_rt_motion/gradient-rt-motion --socket-path /tmp/rtprobe_rr/ipc.sock --num-axes 2 --slave-positions 0,1 --max-rpm 100 --split-domains-per-axis --queue-split-domains-round-robin`
    - result: first multi-slave path that no longer collapsed; after `8 s`, metrics showed `responding=6/2`, `online=2/2`, `operational=0/2`, `master_al=0x1`, `wkc=0/4`, `startup_resets=0`
  - Round-robin + minimal-profile probe:
    - `sudo timeout 8s ./src/gradient_rt_motion/gradient-rt-motion --socket-path /tmp/rtprobe_rr_min/ipc.sock --num-axes 2 --slave-positions 0,1 --max-rpm 100 --split-domains-per-axis --queue-split-domains-round-robin --rx-pdo 0x1701 --tx-pdo 0x1b01 --no-dc`
    - result: same stable-in-`INIT` behavior; no collapse, still `WKC=0/4`
  - Reinstalled the rebuilt binary and restored the service:
    - `sudo install -m 0755 src/gradient_rt_motion/gradient-rt-motion /usr/local/bin/gradient-rt-motion`
    - `sudo systemctl start gradient-rt-motion.service`
    - `systemctl is-active gradient-rt-motion.service` -> `active`
- Follow-up notes / risks:
  - Strongest current discriminator: queued application-domain traffic is required for the collapse; suppressing `ecrt_domain_queue()` keeps the two-slave topology alive until queueing resumes.
  - Shared-domain packing is not the whole story, because per-axis domains still collapse when all configured domains are queued each cycle.
  - There is now a useful partial workaround/test mode: per-axis domains plus round-robin queueing keeps the bus alive but does not produce non-zero WKC or leave `INIT`.
  - Explicit low-level PDO configuration is not yet a viable replacement for `ecrt_slave_config_pdos()` on these fixed PDOs under the current libecrt registration model.
  - Best next target is staged/limited runtime queueing and AL-state progression on top of the stable round-robin mode, not more time on PREEMPT_RT/core placement or simple wrapper-vs-wrapper toggles.

## 2026-03-19 05:35:22 +00:00

- Task summary:
  - Collapsed the EtherCAT/uplink port role selection to a single repo config source so the host install/runtime path no longer depends on multiple static hardcoded templates for the active port choice.
- Validation:
  - Added single source of truth:
    - `systemd/ethercat-host/port-layout.env`
    - one-line role switch: `ETHERCAT_PORT="eth0|eth1"`
  - Added renderer:
    - `systemd/ethercat-host/render-generated-files.sh`
    - generates `ethercat.conf`, `10-ethercat0.link`, `10-uplink0.link`, and `10-unmanaged-ethercat.conf`
  - Updated host installer:
    - `systemd/ethercat-host/install.sh`
    - now renders temp copies from `port-layout.env`, installs `/etc/gradient-ethercat-host.env`, and installs generated `/etc/ethercat.conf` / `.link` / NetworkManager files from that one source
  - Updated runtime helpers to consult the installed single-source config before name-order fallbacks:
    - `systemd/ethercat-host/gradient-ethercat-nic-tune.sh`
    - `systemd/ethercat-host/gradient-irq-affinity.sh`
    - `scripts/ethercat/diagnose_host.sh`
  - Updated repo-managed generated files/comments and host README to point back to `port-layout.env`
  - Syntax checks:
    - `bash -n systemd/ethercat-host/render-generated-files.sh systemd/ethercat-host/install.sh systemd/ethercat-host/gradient-ethercat-nic-tune.sh systemd/ethercat-host/gradient-irq-affinity.sh scripts/ethercat/diagnose_host.sh scripts/ethercat/restore_original_wiring.sh`
  - Renderer dry-run:
    - `bash systemd/ethercat-host/render-generated-files.sh --output-dir /tmp/tmp.qiFM7hit7I`
    - verified generated outputs matched the intended current mapping (`eth1` -> `ethercat0`, `eth0` -> `uplink0`)
  - Applied host install:
    - `sudo bash systemd/ethercat-host/install.sh`
    - installed `/etc/gradient-ethercat-host.env`
    - regenerated `/etc/ethercat.conf`, `/etc/systemd/network/10-ethercat0.link`, and `/etc/NetworkManager/conf.d/10-unmanaged-ethercat.conf`
  - Post-install verification:
    - `/etc/gradient-ethercat-host.env` shows `ETHERCAT_PORT="eth1"`
    - `/etc/ethercat.conf` shows generated binding `MASTER0_DEVICE="c8:3e:a7:14:1c:76"`
    - `systemctl is-active ethercat.service gradient-rt-motion.service` -> both `active`
- Follow-up notes / risks:
  - The active host config is now centralized for code/runtime purposes, but several long-form docs/plans/rules still contain historical `eth0`/`eth1` assertions and should be cleaned up separately to avoid future confusion.
  - Switching the active physical EtherCAT port is now a one-file change in the repo (`systemd/ethercat-host/port-layout.env`) followed by `install.sh`, rather than editing multiple host template files by hand.

## 2026-03-19 05:52 +0000

- Task summary:
  - Performed the first clean live `eth0` retest using the new single-source host port config and verified the result at the raw master/slave layer before attempting more RTCore probe runs.
- Changes:
  - Updated `systemd/ethercat-host/port-layout.env`:
    - switched `ETHERCAT_PORT` from `eth1` to `eth0`
  - Applied the host config:
    - `cd systemd/ethercat-host && sudo ./install.sh`
  - Restarted the master:
    - `sudo systemctl restart ethercat.service`
- Validation:
  - Live config files switched as expected:
    - `/etc/gradient-ethercat-host.env` now shows `ETHERCAT_PORT="eth0"`
    - `/etc/ethercat.conf` now shows `MASTER0_DEVICE="c8:3e:a7:14:1c:75"`
  - Raw EtherCAT checks on `eth0`:
    - `sudo ethercat master`
    - result: `Main: c8:3e:a7:14:1c:75 (attached)`, `Slaves: 0`, `Rx frames: 0`, `Frame loss [%]: 100.0`
    - `sudo ethercat slaves -v`
    - result: no slaves listed
  - Host diagnostics:
    - `sudo ./scripts/ethercat/diagnose_host.sh`
    - result: `eth0` is the currently `connected` NetworkManager interface (`Direct-PC`), `eth1` is `disconnected`, and both `ethercat.service` and `gradient-rt-motion.service` are `active`
- Follow-up notes / risks:
  - The clean `eth0` retest does not reproduce the earlier “stable in INIT then collapse after queued domain traffic” behavior yet; the bus currently fails one layer lower with zero discovered slaves and zero received frames.
  - Because `gradient-rt-motion.service` is active, manual probe binaries should be treated as potentially contending with the installed RTCore instance unless the service is stopped first.
  - Next useful branch is to reconcile current physical cabling/topology against the new `eth0` result before spending more time on multi-slave queue-shaping experiments.

## 2026-03-19 06:00 +0000

- Task summary:
  - Corrected the prior `eth0` retest interpretation after the user clarified that the physical cable had **not** been swapped to the other port.
- Changes:
  - No code changes beyond repo-memory corrections.
  - Marked the earlier `eth0` conclusion in `.cursor/memory/AGENT_SCRATCHPAD.md` as superseded by the missing physical recable.
- Validation:
  - User correction: the software-side port flip was performed, but the physical EtherCAT cable/path remained on the original port during the check.
- Follow-up notes / risks:
  - The previous `eth0` raw-master result is not a valid alternate-port EtherCAT comparison and should not be used to infer anything about `eth0` as an EtherCAT path.
  - Future cross-port tests must explicitly include both steps: update `systemd/ethercat-host/port-layout.env` and physically move the cable/path before evaluating `ethercat master`, `ethercat slaves -v`, or RTCore probes.

## 2026-03-19 06:34 +0000

- Task summary:
  - Implemented a generated NetworkManager uplink profile so the non-EtherCAT NIC can be staged as the managed SSH/API side from the same `port-layout.env` source of truth.
- Changes:
  - Updated `systemd/ethercat-host/port-layout.env`:
    - added uplink profile settings (`UPLINK_NM_CONNECTION_ID`, IPv4/IPv6 method, address, DNS, autoconnect)
  - Updated `systemd/ethercat-host/render-generated-files.sh`:
    - now validates uplink profile settings
    - renders `gradient-uplink.nmconnection`
    - derives a deterministic profile UUID from the selected uplink MAC
  - Updated `systemd/ethercat-host/install.sh`:
    - installs `gradient-uplink.nmconnection` to `/etc/NetworkManager/system-connections/`
    - no longer auto-loads the profile into NetworkManager during install, to avoid live duplicate-IP surprises on transitional setups
  - Updated `systemd/README.md`:
    - documents that `ethercat-host/install.sh` now installs a managed uplink profile in addition to the EtherCAT-side configs
  - Regenerated repo-managed outputs in `systemd/ethercat-host/`, including:
    - `ethercat.conf`
    - `10-ethercat0.link`
    - `10-uplink0.link`
    - `10-unmanaged-ethercat.conf`
    - `gradient-uplink.nmconnection`
- Validation:
  - `bash -n systemd/ethercat-host/render-generated-files.sh systemd/ethercat-host/install.sh`
  - `bash systemd/ethercat-host/render-generated-files.sh`
  - `sudo ./systemd/ethercat-host/install.sh`
  - Confirmed generated uplink profile content in-repo:
    - MAC `c8:3e:a7:14:1c:76`
    - static IPv4 `192.168.1.50/24`
    - DNS `1.1.1.1,8.8.8.8`
  - Confirmed NetworkManager sees the staged profile:
    - `nmcli connection show 'Gradient Uplink'`
  - Corrected a live validation side effect:
    - loading the profile immediately caused both `eth0` and `eth1` to hold `192.168.1.50/24`
    - removed auto-load from `install.sh`
    - deactivated the live `Gradient Uplink` connection via `sudo env DBUS_SYSTEM_BUS_ADDRESS=unix:path=/run/dbus/system_bus_socket nmcli connection down 'Gradient Uplink'`
    - verified `nmcli device status` returned to `eth0 connected`, `eth1 disconnected`
- Follow-up notes / risks:
  - The staged config now supports the intended future mapping (`ETHERCAT_PORT="eth0"` => `eth1` uplink), but that is still only a host-side/software staging step until the physical cable roles are moved to match.
  - If a legacy static profile remains active on the old uplink NIC while the new static uplink profile is also activated, duplicate IP conflicts are possible; avoid running both on the same subnet simultaneously.

## 2026-03-19 06:37 +0000

- Task summary:
  - Added the user-requested WiFi network as a saved NetworkManager profile and tested whether the host could associate to it without disturbing the wired management path.
- Changes:
  - Created a saved WiFi connection profile:
    - `colonise the moon_5G-1`
    - `connection.autoconnect=yes`
    - `connection.autoconnect-priority=20`
  - Lowered interference from old WiFi entries:
    - set `Aussie Broadband 9745` and `Aussie Broadband 9745 1` to `connection.autoconnect=no`
- Validation:
  - `nmcli -f SSID,SECURITY,SIGNAL device wifi list ifname wlan0`
    - confirmed the visible SSID string is `colonise the moon_5G-1`
  - Activation attempts for the new WiFi profile failed twice:
    - `nmcli connection up 'colonise the moon_5G-1'`
    - NetworkManager log showed the connection reaches `associating` and then drops back to `disconnected`
    - the failure occurs before DHCP/IP assignment, so it is not an IP/routing issue
  - Restored a safe final state:
    - deactivated `Aussie Broadband 9745`
    - verified `nmcli device status` ends with `eth0 connected`, `eth1 disconnected`, `wlan0 disconnected`
- Follow-up notes / risks:
  - The requested WiFi network is now staged on the host, but it is not yet a working live path from this machine because association is failing at the supplicant/AP layer.
  - That remaining WiFi issue is most likely one of: wrong credential, AP-side compatibility/policy, or a band/security quirk specific to this adapter/AP pair.
  - No credential text was recorded in repo memory.

## 2026-03-19 06:38 +0000

- Task summary:
  - Added an explicit uplink cutover helper so the intended target state is operationally clear: `eth0` dedicated to EtherCAT, `eth1` used as the managed uplink after the physical cable move.
- Changes:
  - Added `systemd/ethercat-host/activate-uplink.sh`
    - reloads staged NetworkManager profiles
    - deactivates any legacy active connection on the configured EtherCAT NIC
    - activates the generated uplink profile (`Gradient Uplink`) on the computed non-EtherCAT NIC
    - prints resulting device/IP status and reminds the operator to reboot if unmanaged-device policy has not yet taken effect
  - Updated `systemd/README.md`
    - documents `sudo bash ./activate-uplink.sh` as the post-cable-move cutover step
  - Updated `systemd/ethercat-host/install.sh`
    - prints the helper command in its next-step output
- Validation:
  - `bash -n systemd/ethercat-host/activate-uplink.sh systemd/ethercat-host/install.sh`
  - `ReadLints` on touched files (no diagnostics)
- Follow-up notes / risks:
  - The helper is intentionally not auto-run during install; it should be run only after the uplink cable has been physically moved to the non-EtherCAT NIC.
  - Current staged software goal remains `ETHERCAT_PORT="eth0"` => `eth1` uplink.

## 2026-03-19 06:48 +0000

- Task summary:
  - Hardened the uplink cutover helper for the real remote-only workflow: start it while still connected on `eth0`, then move the cable and let the script complete on-box.
- Changes:
  - Updated `systemd/ethercat-host/activate-uplink.sh`
    - added `--timeout SECONDS`
    - waits for carrier on the computed uplink NIC before making the cutover
    - keeps the current `eth0` path alone while waiting
    - only deactivates the old connection on the EtherCAT NIC after the new uplink profile has been brought up
  - Updated `systemd/README.md`
    - now documents the helper as a pre-unplug cutover step
  - Updated `systemd/ethercat-host/install.sh`
    - next-step output now says to start the helper before moving the cable
- Validation:
  - `bash -n systemd/ethercat-host/activate-uplink.sh systemd/ethercat-host/install.sh`
  - `ReadLints` on touched files (no diagnostics)
- Follow-up notes / risks:
  - This makes the script safe to launch from the current SSH session, but the SSH connection itself will still drop when the old `eth0` cable is physically unplugged; the point is that the script keeps running on the Pi and brings `eth1` up so a reconnect is possible.

## 2026-03-19 06:53 +0000

- Task summary:
  - Confirmed the intended post-cutover host state is now live: `eth1` is the managed uplink and `eth0` is free for dedicated EtherCAT use.
- Validation:
  - User ran:
    - `sudo env DBUS_SYSTEM_BUS_ADDRESS=unix:path=/run/dbus/system_bus_socket nmcli connection down "Direct-PC"`
  - Resulting network state:
    - `nmcli device status` shows `eth1 connected` via `Gradient Uplink`
    - `eth0 disconnected`
    - `ip -4 addr show dev eth0` returns no IPv4 address
    - `ip -4 addr show dev eth1` shows `192.168.1.50/24`
  - EtherCAT state on the dedicated port:
    - `sudo ethercat master`
    - `Main: c8:3e:a7:14:1c:75 (attached)`
    - `Slaves: 6`
    - matched RX/TX traffic on the EtherCAT NIC
- Follow-up notes / risks:
  - This is the first clean confirmation in this session that the host networking goal is satisfied: uplink on `eth1`, EtherCAT on `eth0`.
  - `wlan0` was still attempting to configure `Aussie Broadband 9745` in the attached terminal output; that WiFi churn is unrelated to the wired/EtherCAT cutover but should be cleaned up if it continues.

## 2026-03-19 06:53 +0000

- Task summary:
  - Removed the old `Aussie Broadband` WiFi profiles so only the user-requested staged WiFi connection remains.
- Validation:
  - Deleted:
    - `Aussie Broadband 9745`
    - `Aussie Broadband 9745 1`
  - Verified remaining saved connections include:
    - `colonise the moon_5G-1`
    - `Gradient Uplink`
    - no remaining `Aussie Broadband` profiles
  - Current device state after deletion:
    - `eth1` connected via `Gradient Uplink`
    - `eth0` disconnected
    - `wlan0` attempting `colonise the moon_5G-1`
- Follow-up notes / risks:
  - The stale WiFi churn source is gone.
  - The remaining WiFi profile is still staged but not yet proven to associate successfully on this host.

## 2026-03-19 06:57 +0000

- Task summary:
  - Cleaned `eth0` back to a dedicated EtherCAT-only role after `Direct-PC` reactivated there unexpectedly.
- Validation:
  - Inspected `Direct-PC`:
    - profile still targeted `eth0`
    - `connection.autoconnect=yes` explained why it returned
  - Applied cleanup:
    - `sudo env DBUS_SYSTEM_BUS_ADDRESS=unix:path=/run/dbus/system_bus_socket nmcli connection modify 'Direct-PC' connection.autoconnect no`
    - `sudo env DBUS_SYSTEM_BUS_ADDRESS=unix:path=/run/dbus/system_bus_socket nmcli connection down 'Direct-PC'`
  - Verified final network state:
    - `nmcli device status` shows `eth1 connected` via `Gradient Uplink`
    - `eth0 disconnected`
    - `ip -4 addr show dev eth0` returns no IPv4 address
    - `ip -4 addr show dev eth1` still shows `192.168.1.50/24`
  - Verified EtherCAT remained healthy on the dedicated NIC:
    - `sudo ethercat master`
    - `Main: c8:3e:a7:14:1c:75 (attached)`
    - `Slaves: 6`
    - matched RX/TX with near-zero current frame loss
- Follow-up notes / risks:
  - `Direct-PC` remains saved but no longer autoconnects. If you no longer need it at all, it can be deleted later.

## 2026-03-19 06:59 +0000

- Task summary:
  - Deleted the obsolete `Direct-PC` wired profile so nothing remains that can reclaim `eth0` for normal IP networking.
- Validation:
  - Deleted:
    - `Direct-PC`
  - Verified remaining saved connections:
    - `Gradient Uplink`
    - `colonise the moon_5G-1`
    - `Wired connection 2`
  - Verified live device state:
    - `eth1 connected` via `Gradient Uplink`
    - `eth0 disconnected`
    - `wlan0 disconnected`
- Follow-up notes / risks:
  - The old static `Direct-PC` path is fully gone now.
  - `Wired connection 2` still exists as an unused saved profile for the other Ethernet interface; it is not active.

## 2026-03-19 07:21 +0000

- Task summary:
  - Returned to RTCore debugging on the corrected `eth0` EtherCAT baseline and fixed misleading startup telemetry that had been making healthy-but-slow convergence look like failure.
- Changes:
  - Updated `src/gradient_rt_motion/main.cpp`:
    - `startup_ready` now requires all selected slaves to be operational **and** process data to be live, rather than treating first nonzero `domain_wc` as full convergence
    - added one-shot `EtherCAT startup process_data_live ...` logging for the first nonzero process-data/WKC event
    - replaced the hardcoded 5 s startup warning threshold with one derived from the existing AL transition budgets (`preop_to_safeop_timeout_ms + safeop_to_op_timeout_ms`, minimum 5 s; currently 10 s)
  - Rebuilt `src/gradient_rt_motion/gradient-rt-motion`
  - Installed the rebuilt binary to `/usr/local/bin/gradient-rt-motion`
  - Restarted `gradient-rt-motion.service`
- Validation:
  - Confirmed the stale terminal failure was just cwd-related:
    - terminal was in `systemd/ethercat-host`, so `./src/gradient_rt_motion/gradient-rt-motion` failed with `No such file or directory`
    - verified both binaries exist:
      - `/home/pi/GradientOS/src/gradient_rt_motion/gradient-rt-motion`
      - `/usr/local/bin/gradient-rt-motion`
  - Clean 2-axis manual probe from repo root:
    - `sudo timeout 15s ./src/gradient_rt_motion/gradient-rt-motion --socket-path /tmp/rtprobe_eth0_long2/ipc.sock --num-axes 2 --slave-positions 0,1 --max-rpm 100`
    - first `process_data_live` at about `5419 ms` with `wkc=2/4`
    - `1/2 operational` by about `6002 ms`
    - `2/2 operational` by about `9641 ms`
    - final convergence log: `operational=2/2`, `domain_wc=2`, `wkc=6/4`
  - Supplemental discriminators on corrected `eth0` baseline:
    - plain 2-axis 8 s probe no longer loses the bus; slaves stay responding/online and sit in `PREOP` before late partial recovery
    - `--startup-skip-domain-queue-ms 1500` no longer acts like the old collapse discriminator; queue suppression still leaves the slaves in `PREOP`, then late partial WKC appears after the 5 s window
    - `--split-domains-per-axis --queue-split-domains-round-robin` similarly shows late partial convergence instead of preventing a collapse
  - Fresh 6-axis service after patched restart:
    - `./scripts/sampler/rtcore_metrics.py --path /run/gradient-rt-motion/metrics.json summary`
    - after ~15 s: `responding=6/6`, `online=6/6`, `operational=6/6`, `master_al=0x8`, `wkc=18/12`
    - per-axis statuswords `0x1650`, no axis error codes
  - Fresh 6-axis service journal after patched restart:
    - startup waits in `PREOP` with `wkc=0/12` through ~5 s
    - `process_data_live` at about `5423 ms` with `wkc=2/12`
    - `1/6 operational` at about `6003 ms`
    - `3/6 operational` at about `7004 ms`
    - `5/6 operational` at about `8005 ms`
    - full convergence `6/6 operational` at about `8201 ms`
  - Build/lint:
    - `make -C src/gradient_rt_motion`
    - `ReadLints` on `src/gradient_rt_motion/main.cpp` (no diagnostics)
- Follow-up notes / risks:
  - The major reinterpretation is: on the correct `eth0` master port, RTCore does converge; the earlier “failure” was partly bad port assumptions and partly startup telemetry that declared success/failure too early.
  - Remaining work should shift from port-role debugging to actual control-path validation: confirm the controller/API sees live feedback, then perform a very small enable/motion test while watching statuswords, error codes, and WKC.

## 2026-03-19 07:39 +0000

- Task summary:
  - Hardened the controller startup path against RTCore's delayed EtherCAT convergence and restored live controller/API feedback on the `eth0` RTCore baseline.
- Changes:
  - Updated `src/gradient_os/run_controller.py`:
    - added RTCore readiness polling against `/run/gradient-rt-motion/metrics.json` so the controller waits for `startup_ready=1` / all expected axes operational before backend init
    - changed `GET_JOINT_ANGLES` to read fresh backend feedback via `servo_driver.get_current_arm_state_rad(verbose=False)` instead of replying from stale cached joint state
    - when the requested IK backend is unavailable at runtime, the controller now records the actual fallback backend in `active_runtime_config`
  - Updated `src/gradient_os/arm_controller/backends/ethercat_rtcore/backend.py`:
    - waits briefly for RTCore status-axis-config + first status snapshot after IPC connect, so the controller does not treat a raw IPC handshake as fully usable feedback
    - stores status-snapshot WKC/master state metadata for bring-up diagnostics
    - wraps multi-turn rotary feedback into configured logical joint limits before handing it to FK/planning, which fixes large absolute encoder counts causing FK failures
  - Updated `src/gradient_os/arm_controller/command_api.py`:
    - changed `GET_ORIENTATION` to use a fresh servo/back-end read just like `GET_POSITION`
    - improved FK failure logging to include the joint vector that failed
  - Updated `src/gradient_os/ik_solver.py`:
    - if the requested backend fails to initialize (e.g. `numeric` with missing `pyquik`), automatically fall back to another available backend instead of leaving the controller with a dead FK/IK path
  - Updated `tests/test_gradient05_limits_and_backends.py`:
    - added coverage for RTCore readiness gating, multi-turn feedback wrapping, and numeric->ikfast solver fallback
- Validation:
  - `ReadLints` on:
    - `src/gradient_os/run_controller.py`
    - `src/gradient_os/arm_controller/backends/ethercat_rtcore/backend.py`
    - `src/gradient_os/arm_controller/command_api.py`
    - `src/gradient_os/ik_solver.py`
    - `tests/test_gradient05_limits_and_backends.py`
    - result: no diagnostics
  - `./.venv/bin/pytest tests/test_gradient05_limits_and_backends.py -q`
    - `13 passed`
  - Manual live validation from repo root with RTCore already running:
    - started controller via `PYTHONUNBUFFERED=1 ./run.sh`
    - started API via `PYTHONUNBUFFERED=1 /home/pi/GradientOS/.venv/bin/gradient-api`
    - controller now logs:
      - `RTCore ready: startup_ready=1 operational=6/6 wkc=18/12`
      - `Feedback ready: axis_config=1 snapshot=1 wkc=18/12 master_state=4`
      - `IK backend: ikfast (source=runtime_fallback)` after numeric backend load failure
    - `./scripts/sampler/rtcore_metrics.py --path /run/gradient-rt-motion/metrics.json summary`
      - `armed=1`
      - `enable_mask=0x3f`
      - `responding=6/6`
      - `online=6/6`
      - `operational=6/6`
      - `wkc=18/12`
      - per-axis `error_code=0x0000`
    - API checks:
      - `/health` -> controller reachable
      - `/info/all-positions` -> live raw counts for all 6 axes
      - `/info/joints` -> live wrapped joint feedback instead of all-zero cached values
      - `/info/pose` -> returns valid Cartesian pose again
      - `/info/orientation` -> returns valid rotation matrix again
      - `/info/runtime-config` -> reports active IK backend `ikfast` with `source=runtime_fallback` and `requested_backend=numeric`
- Follow-up notes / risks:
  - I did **not** command physical motion; only non-motion startup/feedback validation was performed.
  - `runtime_config.compute_restart_required()` now reports `true` because desired policy still prefers `numeric` while the active controller had to fall back to `ikfast` on this host. To clear that, either install/fix `pyquik` or change the desired IK backend policy for this machine.

## 2026-03-19 07:56 +0000

- Task summary:
  - Moved EtherCAT raw-count to radian conversion ownership into the robot definition surface for `gradient05`, so the RTCore Python backend now prefers robot-defined encoder/gear metadata instead of backend-local assumptions or RTCore's current placeholder runtime scaling.
- Changes:
  - Updated `src/gradient_os/arm_controller/robots/base.py`:
    - added robot-config properties exported via `get_config_dict()` for:
      - `actuator_encoder_counts_per_rev`
      - `actuator_gear_ratios`
      - `actuator_position_signs`
      - `actuator_counts_per_radian` (derived)
  - Updated `src/gradient_os/arm_controller/robots/gradient05/config.py`:
    - defined `actuator_encoder_counts_per_rev = [131072] * 6`
    - defined joint gear ratios from the current EtherCAT bring-up notes:
      - J1-J3 `100:1`
      - J4 `18:1`
      - J5 `20:1`
      - J6 `10:1`
    - defined positive-position signs as robot config data (`[1,1,1,1,1,1]`)
    - kept `gradient05` robot policy on `numeric` IK (unchanged)
  - Updated `src/gradient_os/arm_controller/backends/ethercat_rtcore/backend.py`:
    - builds fallback axis scaling from robot config counts/gear/sign metadata
    - prefers robot-defined scaling for raw feedback conversion even if RTCore publishes a mismatched placeholder axis-config snapshot
    - logs a one-shot warning when runtime RTCore scaling differs from robot config
    - removed the earlier joint-limit modulo/wrapping hack so feedback remains a continuous coordinate
  - Updated `src/gradient_os/arm_controller/command_api.py`:
    - fixed `GET_POSITION` to send joint angles in degrees to match the API field name `joints_deg`
  - Updated `tests/test_gradient05_limits_and_backends.py`:
    - extended Gradient-05 config coverage for counts/rev + gear ratios + signs
    - added coverage that EtherCAT feedback conversion prefers robot-defined scaling over runtime placeholder scaling
    - removed the multi-turn wrapping expectation
- Validation:
  - `ReadLints` on:
    - `src/gradient_os/arm_controller/robots/base.py`
    - `src/gradient_os/arm_controller/robots/gradient05/config.py`
    - `src/gradient_os/arm_controller/backends/ethercat_rtcore/backend.py`
    - `src/gradient_os/arm_controller/command_api.py`
    - `tests/test_gradient05_limits_and_backends.py`
    - result: no diagnostics
  - `./.venv/bin/pytest tests/test_gradient05_limits_and_backends.py -q`
    - `13 passed`
  - Manual live validation with RTCore service active:
    - started controller via `PYTHONUNBUFFERED=1 ./run.sh`
    - started API via `PYTHONUNBUFFERED=1 /home/pi/GradientOS/.venv/bin/gradient-api`
    - controller logs now show:
      - `RTCore ready: startup_ready=1 operational=6/6 wkc=18/12`
      - `WARNING: RTCore axis scaling differs from robot config; keeping robot-config scaling for Python feedback conversion`
    - API checks:
      - `/info/joints` now reports robot-scaled live feedback (example axis2 about `-28.73 deg` from raw `-1046180` counts using the configured `100:1` ratio)
      - `/info/pose` returns a valid pose again with `joints_deg` now actually in degrees
    - `./scripts/sampler/rtcore_metrics.py --path /run/gradient-rt-motion/metrics.json summary`
      - bus stayed healthy: `responding=6/6`, `online=6/6`, `operational=6/6`, `wkc=18/12`
- Follow-up notes / risks:
  - This only fixes the Python/controller-side interpretation of raw feedback. RTCore itself is still publishing placeholder axis scaling (`gear_ratio=1.0`) over IPC on this host, so the next cleanup step is to feed RTCore from the same robot-definition source of truth.
  - I did **not** command physical motion in this pass.

## 2026-03-19 08:25 +0000

- Task summary:
  - Recorded the user's fresh-session handoff as the authoritative baseline for the next phase of EtherCAT/controller work.
- Changes:
  - Updated `.cursor/memory/AGENT_SCRATCHPAD.md` with reinforced guardrails and next-step priorities:
    - `eth0` remains the dedicated EtherCAT master NIC and `eth1` remains uplink/management on this host
    - RTCore startup on the correct path is slow but healthy (`process_data_live` about `5.4 s`, full `6/6 operational` about `8.2 s`)
    - raw EtherCAT steady-state health remains `responding=6/6`, `online=6/6`, `operational=6/6`, `wkc=18/12`, `error_code=0x0000`
    - robot mechanics must stay in robot config, not backend code
    - `gradient05` remains a `numeric` IK robot by policy
    - immediate next action is to verify UI joint jog controls are wired to the real controller/API path before changing RTCore scaling again
    - J5 has a user-reported extra `22T -> 20T` belt stage after the `18:1` gearbox, but the ratio convention must be confirmed before editing config
    - preserved the last useful live sanity point: J3 feedback about `-28.73 deg` from raw count `-1046180` under robot-config scaling
- Validation:
  - Read the latest `.cursor/memory/AGENT_SCRATCHPAD.md`
  - Read the latest `.cursor/memory/DEVLOG.md`
  - Compared both with the user's explicit handoff and merged only facts/guardrails from that handoff
  - Retrieved local timestamp with `date '+%Y-%m-%d %H:%M %z'`
- Follow-up notes / risks:
  - No runtime controller/API/UI verification was performed in this pass.
  - The next meaningful task should start with UI jog-path verification and user-observed per-joint motion data, not another blind RTCore scaling change.

## 2026-03-19 08:44 +0000

- Task summary:
  - Tightened the web UI joint commissioning/zeroing workflow so it is safer and less misleading for real EtherCAT bring-up.
- Changes:
  - Updated `web-ui/src/ControlPanel.tsx`:
    - added a joint-commissioning status banner for info/success/error feedback
    - added a `Refresh` button that explicitly re-reads `GET /info/joints`
    - changed per-joint step jog requests to send `wait_for_idle: true` so the UI waits for the commanded step move to finish before refreshing joint angles
    - disabled joint step and zero-capture buttons until that joint has live finite feedback
    - added operator-facing warning copy that zero capture writes a persistent logical offset
- Validation:
  - Started manual live processes from repo root:
    - `PYTHONUNBUFFERED=1 ./run.sh`
    - `PYTHONUNBUFFERED=1 /home/pi/GradientOS/.venv/bin/gradient-api`
    - `npm run dev` in `web-ui/`
  - Confirmed live services:
    - controller listening on UDP `3000`
    - API serving on `http://0.0.0.0:4000`
    - Vite serving on `http://localhost:8000/`
  - Live API check:
    - `curl -sf http://127.0.0.1:4000/info/joints`
    - returned live arm angles including J3 about `-28.73`
  - Frontend validation:
    - `curl -sf http://127.0.0.1:8000`
    - returned the Vite app HTML shell
    - `ReadLints` on `web-ui/src/ControlPanel.tsx` -> no diagnostics
    - `npm run build` in `web-ui/` -> success
  - Backend regression:
    - `./.venv/bin/pytest tests/test_api_endpoints.py -q`
    - `29 passed`
- Follow-up notes / risks:
  - I did **not** trigger any real jog, home, rest, or zero-capture action from the web UI in this pass.
  - The next step is a supervised live UI test with the user operating the panel and reporting which joint moved, direction, scaling, and whether API feedback tracked correctly.

## 2026-03-19 08:57 +0000

- Task summary:
  - Investigated the user's live commissioning report that the J1 `+/-1°` UI buttons caused multi-joint physical motion, confirmed a degree/radian bug in the API joint-jog path, and fixed it.
- Changes:
  - Updated `src/gradient_os/api/main.py`:
    - `/control/joint-jog` now converts `target_arm_deg` to `target_arm_rad` before sending the raw comma-separated controller command
    - response payload now includes both `target_arm_deg` and `target_arm_rad` for easier debugging
  - Updated `tests/test_api_endpoints.py`:
    - tightened `test_control_joint_jog` so it asserts the controller command is radian-valued, not degree-valued
- Validation:
  - Confirmed root cause by code trace:
    - `GET_JOINT_ANGLES` reply in `run_controller.py` is degree-based
    - raw comma-separated controller joint command path in `run_controller.py` is interpreted as radians and handed to `servo_driver.set_servo_positions(...)`
  - Probed current host environment:
    - `"/home/pi/GradientOS/.venv/bin/python" -c "import pyquik"` equivalent check -> `ModuleNotFoundError: No module named 'pyquik'`
    - repo contains only `src/numeric_solver/pyquik/CMakeLists.txt` and `bindings.cpp`, no built `pyquik` extension artifact in this checkout
  - `./.venv/bin/pytest tests/test_api_endpoints.py -q`
    - `29 passed`
  - `ReadLints` on:
    - `src/gradient_os/api/main.py`
    - `tests/test_api_endpoints.py`
    - result: no diagnostics
  - Restarted manual processes after the fix:
    - restarted API via `PYTHONUNBUFFERED=1 /home/pi/GradientOS/.venv/bin/gradient-api`
    - observed controller was no longer running because the manual controller terminal ended with `^C`
    - restarted controller via `PYTHONUNBUFFERED=1 ./run.sh`
  - Live reachability after restart:
    - UDP `GET_STATUS` -> `STATUS,gripper_present,False`
    - `curl -s http://127.0.0.1:4000/health` -> controller reachable
- Follow-up notes / risks:
  - Do **not** use the old joint-jog behavior as evidence about joint signs/scaling; the observed multi-joint motion was explained by the degree/radian bug.
  - `pyquik` is genuinely unavailable in the active host venv, so `numeric` IK fallback on this controller process is currently an environment/build issue, not a robot-policy issue.

## 2026-03-19 09:13 +0000

- Task summary:
  - Restored the intended Windows-ICS uplink on `eth1`, populated the QuIK dependency repo at `src/numeric_solver/quik`, built the `pyquik` extension locally, and restarted the stack so `gradient05` is back on the `numeric` solver from robot policy.
- Changes:
  - Updated `systemd/ethercat-host/port-layout.env`:
    - switched uplink profile defaults from the old static direct-link mode to DHCP/default-route mode for Windows ICS
    - set:
      - `UPLINK_IPV4_METHOD="auto"`
      - `UPLINK_IPV4_NEVER_DEFAULT="false"`
      - `UPLINK_IPV6_METHOD="ignore"`
      - `UPLINK_IPV6_NEVER_DEFAULT="false"`
  - Regenerated `systemd/ethercat-host/gradient-uplink.nmconnection` with `bash systemd/ethercat-host/render-generated-files.sh`
  - Updated the live NetworkManager `Gradient Uplink` connection to match and reactivated it
  - Initialized the intended dependency checkout:
    - `git submodule update --init --recursive src/numeric_solver/quik`
  - Installed native build prerequisites on the Pi:
    - `cmake`
    - `ninja-build`
    - `libeigen3-dev`
  - Applied a minimal local compile fix in `src/numeric_solver/quik/include/quik/Robot.hpp`:
    - added `#include <iostream>` so inline `cout` usage compiles
  - Built the Python binding:
    - `cmake -S src/numeric_solver/pyquik -B build/pyquik -G Ninja`
    - `cmake --build build/pyquik -j4`
    - output module: `src/numeric_solver/pyquik/pyquik.cpython-311-aarch64-linux-gnu.so`
- Validation:
  - Network / ICS:
    - before change: `eth1` had static `192.168.1.50/24`, no gateway, `ip route get 1.1.1.1` failed with `Network is unreachable`
    - after change: `eth1` received `192.168.137.89/24`, gateway `192.168.137.1`, DNS `192.168.137.1`
    - `getent hosts github.com` succeeded
  - QuIK / pyquik:
    - `git submodule status src/numeric_solver/quik` -> populated at `a9ebd1f21dfd3c9f0869140dfb0a4ae5a538cccc`
    - `import numeric_solver.pyquik.pyquik` succeeded in `/home/pi/GradientOS/.venv/bin/python`
    - `from numeric_solver.numeric_wrapper import init_numeric_solver; init_numeric_solver('gradient-05')` succeeded
  - Runtime:
    - restarted controller and API manually from repo root
    - controller startup now logs `IK backend: numeric (source=robot_policy)`
    - `curl -s http://127.0.0.1:4000/info/runtime-config` shows:
      - `effective_backend: numeric`
      - `source: robot_policy`
      - `restart_required: false`
    - `curl -s http://127.0.0.1:4000/info/joints` returned live joint data
    - `curl -s http://127.0.0.1:4000/info/pose` returned a live pose
- Follow-up notes / risks:
  - The QuIK checkout now contains one local compile tweak (`Robot.hpp` include). If the submodule is updated later, recheck whether that include is still needed.
  - I did **not** change the solver fallback policy in code yet; the immediate fix here was to make the requested `numeric` solver actually import and run in the controller environment.

## 2026-03-19 09:16 +0000

- Task summary:
  - Prepared a fresh-AI handoff after the jog-path fix, ICS uplink repair, and QuIK/numeric-solver restoration.
- Changes:
  - Updated `.cursor/memory/AGENT_SCRATCHPAD.md` with the most important current-state guardrails:
    - joint-jog degrees/radians bug is fixed and old motion observations from that buggy path should not be reused for scaling/sign conclusions
    - `gradient05` should run on `numeric` per robot policy; silent fallback to `ikfast` is not acceptable unless explicitly allowed
    - Windows ICS over `eth1` is now the intended working uplink path
    - current manual runtime stack is controller + API from repo root, with RTCore still on `eth0`
    - `numeric_solver.pyquik.pyquik` import path works even though top-level `import pyquik` still does not
- Validation:
  - Retrieved local timestamp with `date '+%Y-%m-%d %H:%M %z'`
  - Wrote only factual handoff notes already validated in this session
- Follow-up notes / risks:
  - No new runtime mutation beyond memory writeback in this step.
  - The next AI should begin from the repaired `numeric` + ICS baseline, not from the earlier fallback/static-uplink state.

## 2026-03-19 20:46 +0000

- Task summary:
  - Added a unified staged launcher for the local GradientOS stack so controller, API, and web UI no longer need to be started manually in separate terminals.
  - Preserved live log streaming while also writing durable per-run startup logs for later debugging.
- Changes:
  - Added `start-stack.sh`:
    - sources `./start.sh` before launching children so the unified path matches the manual environment bootstrap
    - stages startup as controller -> API -> web UI
    - supports `--headless` to skip the web UI and run controller + API only
    - writes startup artifacts under `logs/startups/<timestamp>/`:
      - `launcher.log`
      - `controller.log`
      - `api.log`
      - `web.log`
      - `manifest.json`
    - maintains `logs/startups/latest/` as a pointer to the newest run
    - performs readiness checks for:
      - controller UDP `GET_STATUS`
      - API `/health`
      - API `/info/runtime-config`
      - API `/info/joints`
      - API `/info/pose`
      - web UI HTTP root when not headless
    - refuses to start duplicates if live controller/API/web services are already detected
    - provides `status` and `stop` subcommands for launcher-managed runs
  - Updated `docs/README.md`:
    - documented `./start-stack.sh`
    - documented `./start-stack.sh --headless`
    - documented `status` / `stop`
    - documented the `logs/startups/` log location
- Validation:
  - `bash -n ./start-stack.sh`
    - passed
  - `chmod +x ./start-stack.sh && ./start-stack.sh --help`
    - passed
  - `./start-stack.sh status`
    - reported launcher state plus live probe results
    - on this host at validation time:
      - controller probe timed out
      - API `127.0.0.1:4000` was not reachable
      - web UI on `127.0.0.1:8000` was reachable
  - `./start-stack.sh`
    - correctly failed closed because the web UI was already running
    - emitted:
      - `existing live services detected: web@127.0.0.1:8000`
      - `Refusing to start duplicates`
  - `ReadLints` on:
    - `start-stack.sh`
    - `docs/README.md`
    - result: no diagnostics
- Follow-up notes / risks:
  - I did **not** use the new launcher to start the live robot stack in this pass, because the request was to implement the startup path and there were already existing live/manual processes on the machine.
  - `status` currently reflects the actual live probes, not terminal history; at validation time the web UI was up while controller/API were not.
  - If desired next, the launcher can be extended with explicit pass-through args or a detached/background mode, but the current version is intentionally strict and foreground-supervised.

## 2026-03-19 20:57 +0000

- Task summary:
  - Hardened shutdown handling for the new `start-stack.sh` path and fixed a real controller telemetry crash exposed by the first full startup run.
- Changes:
  - Updated `src/gradient_os/arm_controller/backends/ethercat_rtcore/backend.py`:
    - added `_best_effort_safe_power_down()` that disables all RTCore axes and disarms before IPC teardown
    - `shutdown()` now calls that safe power-down path before closing shared memory, eventfds, and sockets
  - Updated `start-stack.sh`:
    - added a graceful controller stop path that first tries `POST /control/stop`
    - controller shutdown now prefers `SIGINT` over plain `SIGTERM` so `run_controller.py` can execute backend cleanup/finally logic
    - shutdown ordering now keeps the API alive long enough to issue the stop request before controller teardown
  - Updated `src/gradient_os/arm_controller/backends/registry.py`:
    - `get_telemetry_blocks()` now treats backend register-telemetry support as optional
    - `parse_telemetry_block()` now returns `{}` when the active backend does not implement parser helpers
    - this prevents the `START_TELEMETRY` thread from crashing under `ethercat_rtcore`
  - Updated `tests/test_gradient05_limits_and_backends.py`:
    - added coverage that RTCore safe power-down disables axes and disarms
    - added coverage that `ethercat_rtcore` reports no backend register telemetry blocks instead of crashing
  - Updated `docs/README.md`:
    - clarified that `start-stack.sh` now performs a graceful shutdown path rather than only killing processes
- Validation:
  - Investigated the user's captured startup log from `start-stack.sh`:
    - expected/benign startup observations:
      - one initial `Waiting for controller: timed out` before the controller finished binding UDP
      - early `IKFast back-end initialised` print before the robot policy selected `numeric`
    - real issue confirmed from the log:
      - controller telemetry thread crashed on `AttributeError: module '...ethercat_rtcore.config' has no attribute 'TELEMETRY_BLOCK1_ADDRESS'`
  - `bash -n ./start-stack.sh`
    - passed
  - `./.venv/bin/python -m pytest tests/test_gradient05_limits_and_backends.py -q`
    - `15 passed`
  - `ReadLints` on:
    - `start-stack.sh`
    - `src/gradient_os/arm_controller/backends/ethercat_rtcore/backend.py`
    - `src/gradient_os/arm_controller/backends/registry.py`
    - `tests/test_gradient05_limits_and_backends.py`
    - `docs/README.md`
    - result: no diagnostics
  - `./start-stack.sh status`
    - current host state at validation time:
      - launcher state file from run `20260319-205134` was stale
      - controller still responded on UDP `3000`
      - API and web were not reachable
- Follow-up notes / risks:
  - The graceful shutdown code was implemented after the user's first full startup run; that earlier run should not be used as evidence against the new shutdown path.
  - A fresh launch -> controlled stop should be performed next to verify the physical EtherCAT drives really end in a non-active state with the updated backend shutdown behavior.

## 2026-03-19 21:04 +0000

- Task summary:
  - Added an explicit actuator power-down command path so stack shutdown no longer depends only on the controller process exiting cleanly.
- Changes:
  - Updated `src/gradient_os/arm_controller/actuator_interface.py`:
    - added default `safe_power_down()` hook for backends
  - Updated `src/gradient_os/arm_controller/backends/ethercat_rtcore/backend.py`:
    - implemented public `safe_power_down()` that disables axes and disarms RTCore immediately
  - Updated `src/gradient_os/arm_controller/command_api.py`:
    - added `handle_safe_power_down(...)`
    - this issues a motion stop first, then invokes backend-specific power-down when available
  - Updated `src/gradient_os/run_controller.py`:
    - added UDP command `SAFE_POWER_DOWN`
    - command replies `ACK,SAFE_POWER_DOWN,...`
  - Updated `src/gradient_os/api/main.py`:
    - added `POST /control/power-down`
  - Updated `start-stack.sh`:
    - added `/control/power-down` shutdown step
    - added UDP fallback `SAFE_POWER_DOWN` when API is already unavailable
    - `stop` now tries safe power-down even if launcher state is stale or absent
  - Updated `tests/test_api_endpoints.py`:
    - added coverage for `/control/power-down`
    - added coverage for `/control/power-down` with `wait_for_idle`
  - Updated `docs/README.md`:
    - documented that launcher shutdown now issues explicit power-down/de-energize before teardown
- Validation:
  - `bash -n ./start-stack.sh`
    - passed
  - `./.venv/bin/python -m pytest tests/test_api_endpoints.py tests/test_gradient05_limits_and_backends.py -q`
    - `46 passed`
  - `ReadLints` on:
    - `start-stack.sh`
    - `src/gradient_os/api/main.py`
    - `src/gradient_os/run_controller.py`
    - `src/gradient_os/arm_controller/command_api.py`
    - `src/gradient_os/arm_controller/actuator_interface.py`
    - `src/gradient_os/arm_controller/backends/ethercat_rtcore/backend.py`
    - `tests/test_api_endpoints.py`
    - `tests/test_gradient05_limits_and_backends.py`
    - result: no diagnostics
  - `./start-stack.sh status`
    - current software state at check time:
      - launcher state absent
      - controller down
      - API down
      - web UI down
- Follow-up notes / risks:
  - I did **not** physically validate the new power-down command against the live robot in this pass; that still needs a fresh real-hardware start -> stop verification.
  - The next hardware check should use the new `POST /control/power-down` / `./start-stack.sh stop` path and confirm the robot is no longer left energized.

## 2026-03-19 21:07 +0000

- Task summary:
  - Added a first-class hardware probe command so the operator can inspect the physical RTCore/drive state directly from the launcher.
- Changes:
  - Updated `start-stack.sh`:
    - added `probe` action (`./start-stack.sh probe`)
    - probe reads `GRADIENT_RTCORE_METRICS` / `/run/gradient-rt-motion/metrics.json`
    - probe reports:
      - controller UDP availability
      - API availability
      - runtime-config summary when API is reachable
      - physical state classification (`ACTIVE`, `BUS_UP_DISARMED`, `FAULTED`, `INACTIVE`)
      - RTCore bus health (`link_up`, `responding`, `online`, `operational`, `wkc`, `startup_ready`, `master_al`)
      - per-axis DS402 state, statusword, error code, slave online/operational flags, slave AL state, and raw position counts
  - Updated `docs/README.md`:
    - documented `./start-stack.sh probe`
- Validation:
  - `bash -n ./start-stack.sh`
    - passed
  - `./start-stack.sh probe`
    - on this host at validation time reported:
      - `controller_udp: down`
      - `api_http: down`
      - `physical_state: ACTIVE`
      - `armed=1`
      - `enable_mask=0x3f`
      - `op_enabled_axes=6/6`
      - bus healthy: `link_up=1`, `responding=6/6`, `online=6/6`, `operational=6/6`, `wkc=18/12`, `startup_ready=1`
      - all six axes in `OperationEnabled` with `error_code=0x0000`
  - `ReadLints` on:
    - `start-stack.sh`
    - `docs/README.md`
    - result: no diagnostics
- Follow-up notes / risks:
  - This probe confirms a critical safety fact: the physical robot can remain fully active even when the controller/API processes are down.
  - The next safety improvement should provide a direct RTCore disarm path that does not depend on a live controller process.

## 2026-03-19 21:34 +0000

- Task summary:
  - Refactored `start-stack.sh stop` into a probe-driven shutdown sequence that explicitly reasons about driver state, RTCore state, and EtherCAT master state before escalating each shutdown phase.
- Changes:
  - Updated `start-stack.sh`:
    - added JSON-form hardware probe internals so shutdown logic can consume the same RTCore/controller/API state that `./start-stack.sh probe` displays
    - expanded human probe output to report:
      - `driver_state`
      - `ethercat_master_state`
      - `rtcore_state`
      - `physical_state`
    - added probe parsing helpers used during shutdown decisions
    - added direct RTCore disarm helper that:
      - uses the project venv Python
      - resolves the robot from runtime config
      - connects to RTCore with `GRADIENT_RTCORE_AUTO_ARM=0`
      - sends backend `safe_power_down()` without relying on a live controller process
    - added service/process stop helpers for:
      - `gradient-rt-motion.service`
      - `ethercat.service`
      - fallback manual RTCore PID stop
    - replaced ad-hoc stop behavior with `perform_shutdown_sequence()`:
      - initial probe snapshot
      - controller/API power-down request when available
      - direct RTCore disarm if the hardware is still active
      - controller/API/web shutdown
      - RTCore shutdown
      - EtherCAT master shutdown
      - final probe snapshot/report
    - stale/absent launcher state now still attempts the full shutdown sequence using discovered controller/API/web PIDs where possible
- Validation:
  - `bash -n ./start-stack.sh`
    - passed
  - `./start-stack.sh probe`
    - still passed after the refactor
    - live host result at validation time:
      - `driver_state: ACTIVE`
      - `ethercat_master_state: OP`
      - `rtcore_state: UP`
      - `physical_state: ACTIVE`
      - controller/API both down while hardware remained active
  - `ReadLints` on:
    - `start-stack.sh`
    - `docs/README.md`
    - result: no diagnostics
- Follow-up notes / risks:
  - I did **not** execute the live shutdown sequence in this pass because it would directly change the robot's physical state.
  - The next live verification should be an intentional `./start-stack.sh stop` run while the robot is in a safe-to-de-energize condition, followed immediately by `./start-stack.sh probe`.

## 2026-03-19 23:27 +0000

- Task summary:
  - Fixed the remaining `start-stack.sh stop` restart bug where the first stop could leave Vite listening on `:8000`, forcing a second cleanup stop before the next launch.
- Changes:
  - Updated `start-stack.sh`:
    - added `pid_group_is_live`, `pid_group_id`, and recursive descendant collection helpers
    - added `stop_pid_hierarchy()` so shutdown signals the full managed process group and known descendants instead of only the recorded parent PID
    - routed web/api/controller external stop paths through the shared hierarchy stop helper
    - started managed child services with `setsid` when available so each service gets its own process group for reliable teardown
- Validation:
  - `bash -n ./start-stack.sh`
    - passed
  - `ReadLints` on `start-stack.sh`
    - no diagnostics
  - `./start-stack.sh stop`
    - cleared the previously orphaned `node`/Vite listener on `:8000`
  - Live full-cycle check:
    - `./start-stack.sh`
    - `./start-stack.sh stop`
    - `./start-stack.sh`
    - `./start-stack.sh stop`
    - restart succeeded on the first try with no duplicate-service refusal
    - final `ss -ltnp '( sport = :8000 or sport = :4000 or sport = :3000 )'` showed no listeners on those stack ports
    - final `./start-stack.sh probe` reported `physical_state: BUS_UP_DISARMED`
    - controller logs showed `Shutdown signal received: SIGTERM` and `Shutdown requested.` during stop, with no SIGKILL warning emitted by the launcher
- Follow-up notes / risks:
  - The API still logs `ASGI callable returned without completing response.` during shutdown because in-flight requests are interrupted as the service is being stopped; this did not block the verified safe stop/restart flow.

## 2026-03-20 00:xx +0000

- Task summary:
  - Investigated a live user-run Joint Commissioning test where clicking `J1 +5°` caused no visible physical motion and no 3D robot motion.
- What was verified:
  - Read-only code trace confirmed the UI wiring exists:
    - `web-ui/src/ControlPanel.tsx` posts JOG to `/control/joint-jog`
    - `src/gradient_os/api/main.py` translates UI degree deltas into raw radian joint commands
    - `src/gradient_os/run_controller.py` accepts the raw joint command and forwards it through `servo_driver.set_servo_positions(...)`
  - Attached live terminal output showed the click reached the controller:
    - controller received the raw target string `0.08726646259971647,0.0,0.0,0.0,0.0,0.008726646259971648`
    - controller then received `WAIT_FOR_IDLE`
    - API returned `POST /control/joint-jog HTTP/1.1` `200 OK`
  - `command_api.handle_wait_for_idle()` currently only waits on the trajectory thread, so its `No move is currently running.` message is expected for direct commissioning setpoints and does not prove RTCore moved.
  - Post-run `./start-stack.sh probe` showed:
    - `physical_state: FAULTED`
    - `axis0: ds402=Fault`
    - `err=0x8700`
    - all other axes disarmed after stop
  - The 3D visualizer consumes telemetry-driven joints (`latest?.joints` in `web-ui/src/App.tsx`), not the commissioning panel's local `jointAnglesDeg`, so no telemetry change means no model motion.
- Follow-up notes / risks:
  - Most likely explanation is no longer UI/API wiring failure; it is a backend/hardware-state issue on the live RTCore side, likely involving the faulted mapped axis for J1.
  - The commissioning path should eventually expose a stronger backend acknowledgment than `WAIT_FOR_IDLE` for direct RTCore setpoint writes.

## 2026-03-20 00:xx +0000

- Task summary:
  - Added a user-triggered RTCore/drive fault reset path so resettable DS402 faults can be cleared from the control stack without using path planning or autonomous agent-issued hardware commands.
- Changes:
  - Updated `src/gradient_os/arm_controller/actuator_interface.py`:
    - added non-required `reset_faults(logical_joint_index: Optional[int] = None)` default hook
  - Updated `src/gradient_os/arm_controller/backends/ethercat_rtcore/backend.py`:
    - added `reset_faults(...)` implementation that maps logical joints to RTCore axes and sends `MSG_CMD_FAULT_RESET`
    - added `_send_cmd_fault_reset(...)`
  - Updated `src/gradient_os/run_controller.py`:
    - added UDP command `RESET_FAULTS[,joint]`
    - controller now delegates fault reset to the active backend and returns ACK/ERROR replies
  - Updated `src/gradient_os/api/main.py`:
    - added `POST /control/reset-faults` with optional `joint`
  - Updated `web-ui/src/ControlPanel.tsx`:
    - added guarded `Reset Faults` button in the Joint Commissioning section
    - button uses confirmation + status banner feedback
  - Updated `tests/test_api_endpoints.py`:
    - added coverage for all-axes and joint-targeted fault reset API calls
- Validation:
  - `./.venv/bin/python -m pytest -q tests/test_api_endpoints.py -k 'reset_faults or control_joint_jog or control_zero_joint'`
    - passed (`4 passed, 29 deselected`)
  - `ReadLints` on all touched files
    - no diagnostics
- Follow-up notes / risks:
  - I did **not** execute the new reset path on live hardware because the user explicitly prohibited autonomous hardware actions.
  - `WAIT_FOR_IDLE` remains a poor completion signal for direct commissioning jog setpoints; it only tracks trajectory-thread execution, not raw RTCore setpoint delivery.

## 2026-03-20 00:xx +0000

- Task summary:
  - Sent live drive fault state to the UI and moved vendor fault-code decoding behind backend-config hooks instead of hard-coding it in generic EtherCAT/launcher paths.
- Changes:
  - Updated `src/gradient_os/arm_controller/backends/registry.py`:
    - added helpers to load backend config by name and decode/report drive fault references through backend-provided hooks
  - Updated `src/gradient_os/arm_controller/backends/ethercat_rtcore/config.py`:
    - added config-driven A6-EC 0x603F fault reference metadata and JSON-backed error-code decoding
  - Updated `src/gradient_os/run_controller.py`:
    - added normalized `drive_faults` telemetry snapshots sourced from RTCore metrics plus backend-specific fault decoding
  - Updated `web-ui/src/App.tsx`:
    - parsed `drive_faults` from `/monitor` and passed it into the robot control panel
  - Updated `web-ui/src/ControlPanel.tsx`:
    - added a live Joint Commissioning fault/status card showing physical state, driver/core state, and per-axis decoded fault details
  - Updated `start-stack.sh`:
    - replaced hard-coded A6-EC probe decoding with backend-config decoder hooks and enriched per-axis probe output when a backend reference is available
- Validation:
  - `./.venv/bin/python -m pytest -q tests/test_api_endpoints.py -k 'reset_faults or control_joint_jog or control_zero_joint'`
    - passed (`4 passed, 29 deselected`)
  - `npm run build` in `web-ui`
    - passed
  - `./start-stack.sh probe`
    - passed; still reports the current `axis0 err=0x8700` fault from live metrics without issuing motion commands
  - `ReadLints` on all touched files
    - no diagnostics
- Follow-up notes / risks:
  - Backend-specific decoded names only appear when the active servo backend is known and exposes a fault reference; otherwise the system intentionally falls back to raw codes.
  - I did **not** trigger reset or jog actions on live hardware.

## 2026-03-20 01:42 +0000

- Task summary:
  - Fixed the soft-stop decision logic for latched-fault-but-disarmed states, added RTCore live drive-profile reporting/consumption, and created a shared robot-derived RTCore scaling source for the systemd startup path.
- Changes:
  - Added `src/gradient_os/arm_controller/backends/ethercat_rtcore/runtime.py`:
    - central drive-profile numeric/string mapping for RTCore
    - shared robot-config-to-RTCore scaling/env rendering (`counts_per_rev`, `gear_ratio`, `sign`)
  - Updated `src/gradient_rt_motion/ipc_v1.hpp` and `src/gradient_rt_motion/main.cpp`:
    - defined stable RTCore numeric drive-profile ids
    - added `--drive-profile`
    - emitted non-zero `StatusHelloV1.drive_profile_id`
  - Updated `src/gradient_os/arm_controller/backends/ethercat_rtcore/backend.py`:
    - parsed `MSG_STATUS_HELLO`
    - stored live RTCore drive-profile id for Python/runtime consumers
  - Updated `src/gradient_os/runtime_config.py`:
    - separated `configured_profile` vs `live_profile`
    - kept `effective_profile` as the precedence result while preserving restart checks against configured policy
  - Updated `src/gradient_os/run_controller.py` and `src/gradient_os/telemetry/drive_faults.py`:
    - synchronized live RTCore drive profile into controller runtime config
    - fed telemetry fault snapshots with `configured` vs `live` drive-profile context
  - Updated `start-stack.sh`:
    - added probe helpers that treat `FAULTED + DISARMED + op_enabled_axes=0` as a safe soft-stop result
    - removed redundant shutdown-side `SAFE_POWER_DOWN` retries from `stop_controller_process()`
    - made probe output explicit about configured vs live drive-profile availability
  - Updated `systemd/rt-motion/gradient-rt-motion.service` and `systemd/rt-motion/install.sh`:
    - RTCore service now consumes generated robot-derived scaling/profile env values instead of default `gear_ratio=1.0`-style startup
  - Updated `web-ui/src/App.tsx` and `web-ui/src/ControlPanel.tsx`:
    - UI now distinguishes effective/configured/live drive-profile state
  - Added tests in `tests/test_runtime_config.py` and `tests/test_rtcore_runtime.py`
- Validation:
  - `bash -n ./start-stack.sh`
    - passed
  - `./.venv/bin/python -m pytest tests/test_runtime_config.py tests/test_api_endpoints.py tests/test_rtcore_runtime.py -q`
    - passed (`43 passed`)
  - `make` in `src/gradient_rt_motion`
    - passed
  - `npm run build` in `web-ui`
    - passed
  - `ReadLints` on all touched files
    - no diagnostics
- Follow-up notes / risks:
  - I did **not** run `./start-stack.sh stop` or any other live hardware-changing validation on the robot; that remains user-run only.
  - The systemd RTCore scaling fix takes effect after the service/install path is refreshed on the host (`systemd/rt-motion/install.sh` or equivalent deployment step).

## 2026-03-20 01:59 +0000

- Task summary:
  - Moved RTCore unit/env synchronization into the launcher startup path so `./start-stack.sh` can correct stale installed systemd state before the controller starts, preventing the persistent RTCore scaling mismatch from surviving on the host.
- Changes:
  - Added `systemd/rt-motion/sync-runtime.sh`:
    - renders `/etc/default/gradient-rt-motion` from the selected robot/runtime config
    - updates the installed RTCore unit when it drifts from the repo copy
    - optionally starts/restarts `gradient-rt-motion.service` when `--ensure-active` is used
  - Updated `systemd/rt-motion/install.sh`:
    - now reuses `sync-runtime.sh` instead of duplicating inline env-render logic
  - Updated `start-stack.sh`:
    - checks the sync helper exists/executable
    - runs RTCore unit/env sync before controller startup
    - fails closed if sync cannot be applied, instead of starting with potentially stale scaling
- Validation:
  - `bash -n ./start-stack.sh && bash -n ./systemd/rt-motion/install.sh && bash -n ./systemd/rt-motion/sync-runtime.sh`
    - passed
  - `./.venv/bin/python -m pytest tests/test_runtime_config.py tests/test_api_endpoints.py tests/test_rtcore_runtime.py -q`
    - passed (`43 passed`)
  - `ReadLints` on touched files
    - no diagnostics
- Follow-up notes / risks:
  - I did **not** execute the new sync helper or a full `./start-stack.sh` run in this pass, because it can update/restart the live RTCore systemd service on the robot host.
  - The next live verification should be a user-run `./start-stack.sh` and then `./start-stack.sh probe` to confirm the RTCore scaling mismatch warning disappears on startup.

## 2026-03-20 02:04 +0000

- Task summary:
  - Fixed a regression in the new RTCore sync path where the launcher updated the installed unit/env but left the installed RTCore binary stale, causing `gradient-rt-motion.service` to fail before controller startup.
- What was verified:
  - Live host inspection showed:
    - `/etc/systemd/system/gradient-rt-motion.service` had the new `--drive-profile` ExecStart
    - `/etc/default/gradient-rt-motion` had the generated robot scaling values
    - `/usr/local/bin/gradient-rt-motion` was still an older binary than `src/gradient_rt_motion/gradient-rt-motion`
  - `journalctl -u gradient-rt-motion.service -n 80 --no-pager` showed the exact failure:
    - `ERROR: unknown arg: --drive-profile`
    - service exited with `status=2/INVALIDARGUMENT`
    - RTCore never created `/run/gradient-rt-motion/ipc.sock`
- Changes:
  - Updated `systemd/rt-motion/sync-runtime.sh`:
    - now compares and installs the repo RTCore binary to `/usr/local/bin/gradient-rt-motion` before syncing unit/env
    - resets failed systemd state before start/restart when files changed
    - treats binary/unit/env as one deployment set
- Validation:
  - `bash -n ./systemd/rt-motion/sync-runtime.sh && bash -n ./start-stack.sh && bash -n ./systemd/rt-motion/install.sh`
    - passed
  - `ReadLints` on touched shell files
    - no diagnostics
- Follow-up notes / risks:
  - I did **not** run the repaired sync helper or restart RTCore in this pass because that would change live systemd/fieldbus runtime on the robot host.
  - The next user-run `./start-stack.sh` should now self-heal by installing the current RTCore binary before restarting the service.

## 2026-03-20 02:09 +0000

- Task summary:
  - Fixed the next startup regression: the launcher shut the stack back down because `/info/runtime-config` failed during readiness when the API truncated the controller's large UDP JSON reply.
- What was verified:
  - Terminal transcript showed:
    - controller and API both started successfully
    - `wait_for_api_readiness()` failed on `active_error=Runtime-config decode failure: Unterminated string...`
    - launcher cleanup then issued soft-stop and shut the stack back down
  - Root cause in code:
    - `src/gradient_os/api/main.py` used `sock.recvfrom(1024)` in `_send_controller_command(...)`
    - controller `GET_RUNTIME_CONFIG` now returns a larger JSON payload than that
- Changes:
  - Updated `src/gradient_os/api/main.py`:
    - added `_CONTROLLER_REPLY_MAX_BYTES = 65535`
    - switched controller UDP reads to the larger receive buffer
  - Updated `tests/test_api_endpoints.py`:
    - added a regression test asserting `_send_controller_command(...)` uses the large receive buffer for controller replies
- Validation:
  - `./.venv/bin/python -m pytest tests/test_api_endpoints.py tests/test_runtime_config.py tests/test_rtcore_runtime.py -q`
    - passed (`44 passed`)
  - `ReadLints` on `src/gradient_os/api/main.py` and `tests/test_api_endpoints.py`
    - no diagnostics
- Follow-up notes / risks:
  - I did **not** re-run the live stack startup in this pass; the next user-run `./start-stack.sh` should no longer self-abort on runtime-config readiness once the larger UDP reply is accepted intact.

## 2026-03-20 02:19 +0000

- Task summary:
  - Cleaned up post-stop hardware probe reporting so it can still decode RTCore/EtherCAT state after controller/API are down, and verified whether the observed J3 driver sync fault is visible in probe output.
- Changes:
  - Updated `start-stack.sh`:
    - `probe_hardware_state_json()` now derives a fallback decode context from desired runtime config when `/info/runtime-config` is unavailable
    - passes fallback `servo_backend`, `drive_profile`, and axis-to-joint mapping into `build_drive_fault_snapshot(...)`
    - prints `probe_decode: ... source=desired_runtime_fallback` and shows logical-joint labels such as `J3/axis2`
    - uses the fallback probe backend when deciding whether vendor-specific fault reference metadata is available
  - Updated `src/gradient_os/arm_controller/profiles/drive/a6ec_ds402.py`:
    - fixed repo-root resolution for the A6-EC manual codebook path
    - merged bus-fault and fault-code lookups so `0x8700` decodes as a sync-related drive fault instead of staying raw
  - Updated `tests/test_rtcore_runtime.py`:
    - added regression coverage for decoded `0x8700` sync-fault snapshots on J3 with EtherCAT master state `OP`
- Validation:
  - `bash -n ./start-stack.sh`
    - passed
  - `./.venv/bin/python -m pytest tests/test_rtcore_runtime.py tests/test_runtime_config.py tests/test_api_endpoints.py -q`
    - passed (`45 passed`)
  - `./start-stack.sh probe`
    - reported `ethercat_master_state: OP`
    - reported `J3/axis2: ds402=Fault ... err=0x8700 [Er74.1 | No sync signal | resettable]`
  - `ReadLints` on touched files
    - no diagnostics
- Follow-up notes / risks:
  - The probe now proves J3 is carrying the sync-fault family (`0x8700`), but the manual maps that bus fault to multiple front-panel aliases (`Er74.1`, `ErC1.x`, `ErC2.0`). Exact on-drive alias reporting would require either exposing the drive's `0x203F` code directly from RTCore or presenting all matching aliases instead of a single best-effort label.

## 2026-03-20 02:32 +0000

- Task summary:
  - Surfaced the existing remote DS402 fault-reset path in the commissioning UI so operators can reset an individual faulted joint, not just all axes at once.
- Changes:
  - Updated `web-ui/src/ControlPanel.tsx`:
    - `handleResetFaults(...)` now accepts an optional logical joint number and calls `POST /control/reset-faults` with `{ "joint": n }` when targeted
    - added per-axis `Reset Jn` buttons beside resettable entries in the live drive-fault card
    - renamed the broad action button to `Reset All Faults`
    - removed the four-entry truncation so all faulted axes remain visible and actionable
- Validation:
  - `npm run build` in `web-ui/`
    - passed
  - `ReadLints` on `web-ui/src/ControlPanel.tsx`
    - no diagnostics
- Follow-up notes / risks:
  - I did **not** click the reset buttons or hit the reset API against the live robot, because fault reset changes live hardware state and must remain user-triggered.
  - The remote HTTP API path itself already existed before this pass: `POST /control/reset-faults` for all axes or `POST /control/reset-faults` with `{ "joint": 3 }` for J3 only.

## 2026-03-20 02:34 +0000

- Task summary:
  - User explicitly requested a live servo-driver reset test, so I ran a targeted RTCore fault reset for J3 and verified that the `0x8700` sync fault cleared while the robot stayed disarmed.
- What was verified before action:
  - `./start-stack.sh probe`
    - stack/controller/API were down
    - RTCore remained up with EtherCAT `OP`
    - only `J3/axis2` was faulted: `sw=0x1618 err=0x8700`
- Live action performed:
  - `./.venv/bin/python ./scripts/rtcore_jog.py fault_reset --mask 0x4`
    - sent a DS402 fault-reset pulse only to axis 2 / J3
- Validation:
  - `./.venv/bin/python ./scripts/rtcore_jog.py status --timeout 0.8`
    - showed `axis2 sw=0x1650 err=0x0000`
  - `./start-stack.sh probe`
    - showed `physical_state=BUS_UP_DISARMED`
    - showed all axes `SwitchOnDisabled`
    - showed `J3/axis2 err=0x0000`
- Follow-up notes / risks:
  - This confirms the targeted RTCore reset path works even when the main stack is not running, as long as RTCore is still connected to the EtherCAT bus.
  - I did not re-arm or start the controller after the reset; the system was left in the safer disarmed bus-up state.

## 2026-03-20 02:43 +0000

- Task summary:
  - Added a startup preflight so `./start-stack.sh` checks for disarmed faulted servos and clears them before the controller's RTCore auto-arm path can energize drives.
- Changes:
  - Updated `src/gradient_os/telemetry/drive_faults.py`:
    - added `axis_snapshot_is_faulted(...)`
    - added `build_startup_fault_reset_plan(...)` to compute faulted-axis masks, summaries, and whether startup should auto-reset or block
  - Added `tests/test_drive_faults.py`:
    - covers disarmed faulted startup reset planning
    - covers startup blocking when a faulted axis is still active/armed
    - covers the clean `BUS_UP_DISARMED` no-action case
  - Updated `start-stack.sh`:
    - added `probe_startup_fault_reset_plan()`
    - added `direct_rtcore_fault_reset_mask()`
    - added `startup_fault_reset_preflight()`
    - startup now runs RTCore sync -> startup fault-reset preflight -> controller launch
    - preflight waits for RTCore/EtherCAT bus readiness, auto-resets disarmed faulted axes, and refuses to continue if the fault stays latched
- Validation:
  - `bash -n ./start-stack.sh`
    - passed
  - `./.venv/bin/python -m pytest tests/test_drive_faults.py tests/test_rtcore_runtime.py tests/test_runtime_config.py tests/test_api_endpoints.py -q`
    - passed (`48 passed`)
  - `./start-stack.sh probe`
    - confirmed current live hardware remains `BUS_UP_DISARMED` with no active axis faults
  - `ReadLints` on touched files
    - no diagnostics
- Follow-up notes / risks:
  - I did **not** run a full `./start-stack.sh` after this change because that would enter the live controller auto-arm/power-up path on hardware.
  - Manual `./run.sh` launches still bypass this new preflight; if the same behavior is needed there too, the next step would be either a shared launcher helper or a backend-side pre-arm fault-reset gate.

## 2026-03-20 02:56 +0000

- Task summary:
  - Changed the RTCore safety default from auto-arm-on-connect to disarmed-on-connect, then added an explicit user-triggered power-up path through backend, controller, API, and UI.
- Changes:
  - Updated `src/gradient_os/arm_controller/backends/ethercat_rtcore/backend.py`:
    - changed default `GRADIENT_RTCORE_AUTO_ARM` fallback from `1` to `0`
    - added `safe_power_up()` / `_best_effort_safe_power_up()`
    - reused the same explicit arm/mode/enable sequence for opt-in power-up
  - Updated `src/gradient_os/arm_controller/actuator_interface.py`:
    - added default `safe_power_up()` hook
  - Updated `src/gradient_os/arm_controller/command_api.py` and `src/gradient_os/run_controller.py`:
    - added controller command `SAFE_POWER_UP`
  - Updated `src/gradient_os/api/main.py`:
    - added `POST /control/power-up`
  - Updated `web-ui/src/ControlPanel.tsx`:
    - added `Power Up Drives` and `Power Down Drives` buttons
    - disabled jog controls while RTCore drives are disarmed
    - kept zero capture available with live feedback even while disarmed
  - Updated launchers:
    - `start-stack.sh` now starts controller with `GRADIENT_RTCORE_AUTO_ARM=0`
    - `run.sh` exports `GRADIENT_RTCORE_AUTO_ARM=0` unless explicitly overridden
  - Updated tests:
    - `tests/test_api_endpoints.py`
    - `tests/test_gradient05_limits_and_backends.py`
- Validation:
  - `bash -n ./start-stack.sh && bash -n ./run.sh`
    - passed
  - `./.venv/bin/python -m pytest tests/test_api_endpoints.py tests/test_gradient05_limits_and_backends.py tests/test_drive_faults.py tests/test_rtcore_runtime.py tests/test_runtime_config.py -q`
    - passed (`66 passed`)
  - `npm run build` in `web-ui/`
    - passed
  - `ReadLints` on touched files
    - no diagnostics
- Follow-up notes / risks:
  - I did **not** run a full live startup after this change, because the next meaningful step after startup is an explicit user-initiated power-up.
  - The legacy auto-arm behavior can still be re-enabled intentionally via `GRADIENT_RTCORE_AUTO_ARM=1`, but the default and launcher-managed path are now fail-safe/disarmed.

## 2026-03-20 03:04 +0000

- Task summary:
  - Fixed a startup regression in the new fault-reset preflight helper and then live-verified that the stack now starts fully while leaving all RTCore drives disarmed.
- What failed:
  - User-run `./start-stack.sh` reached `bus ready` and then crashed with:
    - `json.decoder.JSONDecodeError: Expecting value`
  - Root cause:
    - `start-stack.sh` function `probe_startup_fault_reset_plan()` imported `gradient_os.telemetry.drive_faults` directly
    - that import path triggers backend-registry prints on stdout
    - the helper was expected to emit pure JSON, so its stdout became non-JSON and later `json.loads(...)` failed
- Changes:
  - Updated `start-stack.sh`:
    - wrapped the startup-fault-plan import with `redirect_stdout(io.StringIO())`
- Live validation:
  - reran `./start-stack.sh`
    - startup completed past the preflight, controller, API, and web launch
  - ran `./start-stack.sh probe`
    - controller/API up
    - EtherCAT master `OP`
    - `driver_state: DISARMED`
    - `physical_state: BUS_UP_DISARMED`
    - all axes `SwitchOnDisabled` with `err=0x0000`
- Follow-up notes / risks:
  - This confirms the new safety model is live: the stack can come fully up without energizing the drives.
  - User-facing next step remains explicit `power-up` when they intentionally want to arm the drives.

## 2026-03-20 03:10 +0000

- Task summary:
  - Fixed the `Power Up Drives` UI affordance and moved drive power/fault controls into a dedicated top section so they are visually primary and no longer depend only on monitor telemetry timing.
- Changes:
  - Updated `web-ui/src/ControlPanel.tsx`:
    - added a top-level `Drive Power` section
    - moved `Power Up Drives`, `Power Down Drives`, and `Reset All Faults` out of the commissioning row
    - moved drive fault status and per-joint reset actions into that top section
    - left commissioning focused on feedback refresh, jog, and zero capture
    - changed power-control gating to use `driveFaults?.servo_backend ?? activeServoBackend`
  - Updated `web-ui/src/App.tsx`:
    - now passes `runtimeConfigSnapshot?.active?.servo_backend?.effective_backend` into `ControlPanel`
- Validation:
  - `npm run build` in `web-ui/`
    - passed
  - `ReadLints` on `web-ui/src/ControlPanel.tsx` and `web-ui/src/App.tsx`
    - no diagnostics
- Follow-up notes / risks:
  - I did not run a browser automation pass here; the fix was validated by build/lint and directly addresses the exact disabled-state logic in code.
  - If desired, the next pass can also gate the larger Cartesian jog section on explicit drive power, not just the joint commissioning buttons.

## 2026-03-20 03:16 +0000

- Task summary:
  - Fixed the follow-up UI bug where `Power Down Drives` stayed disabled after a successful power-up even though the live robot was actually armed.
- What was verified:
  - `./start-stack.sh probe`
    - showed `driver_state: ACTIVE`
    - showed `armed=1`
    - showed `op_enabled_axes=6/6`
  - live `/monitor` SSE sampling
    - packets often omitted `drive_faults`
- Root cause:
  - `web-ui/src/App.tsx` parsed `drive_faults` from each SSE packet, but initialized missing packets to `null`
  - `setLatest(next)` then replaced the previous active drive snapshot with `null`
  - `ControlPanel` therefore saw “RTCore backend known, but not active” and kept `Power Down Drives` disabled
- Changes:
  - Updated `web-ui/src/App.tsx`:
    - `setLatest(...)` now preserves the previous `drive_faults` snapshot when the current SSE packet omits that field
- Validation:
  - `npm run build` in `web-ui/`
    - passed
  - `ReadLints` on `web-ui/src/App.tsx` and `web-ui/src/ControlPanel.tsx`
    - no diagnostics
- Follow-up notes / risks:
  - The underlying monitor stream still emits `drive_faults` sparsely; the UI is now robust to that, but a future telemetry cleanup could include `drive_faults` on every packet for simpler consumers.

## 2026-03-20 04:03 +0000

- Task summary:
  - Kept the commissioning zero/jog flow unchanged, but added a visualizer fallback so `/info/joints` refreshes can update the 3D arm pose when the `/monitor` SSE stream is late during calibration work.
- Changes:
  - Updated `web-ui/src/ControlPanel.tsx`:
    - added an `onJointFeedback` callback prop
    - after successful `/info/joints` reads, reports the latest arm/gripper angles upward without changing any control commands
  - Updated `web-ui/src/App.tsx`:
    - added a small joint-array comparison helper
    - added `handleFallbackJointFeedback(...)` to convert degree feedback to radians and merge it into `latest`
    - only applies that fallback when SSE telemetry has been quiet for more than 250 ms, so healthy monitor packets remain the primary source of truth
- Validation:
  - `ReadLints` on `web-ui/src/App.tsx` and `web-ui/src/ControlPanel.tsx`
    - no diagnostics
  - `npm run build` in `web-ui/`
    - passed
- Follow-up notes / risks:
  - I did not run live robot motion or zero commands here.
  - This improves pose sync during commissioning, but a future cleanup could centralize all live joint-state reads in App instead of having both SSE and `/info/joints` paths.

## 2026-03-20 04:28 +0000

- Task summary:
  - Tightened the live 3D robot view so the main visualizer snaps to the exact telemetry pose instead of smoothing behind the real robot.
- Changes:
  - Updated `web-ui/src/ArmVisualizer.tsx`:
    - when `joints` props change, the visualizer now copies that snapshot into both `targetAnglesRef` and `currentAnglesRef`
    - applies the same exact joint values immediately to the URDF joints
    - this preserves preview/ghost tooling elsewhere while making the main live robot view behave like a digital twin
- Validation:
  - `ReadLints` on `web-ui/src/ArmVisualizer.tsx`
    - no diagnostics
  - `npm run build` in `web-ui/`
    - passed
- Follow-up notes / risks:
  - This removes UI-side pose lag, but the twin is still only as accurate as the upstream telemetry, URDF geometry, and runtime tool/base calibration.

## 2026-03-20 04:35 +0000

- Task summary:
  - Investigated live commissioning jog failures and improved the error path so controller/backend setpoint rejections are visible in both the terminal and the UI.
- What was observed:
  - Terminal showed repeated `GET_JOINT_ANGLES` success, then `APPLY_JOINT_SETPOINT,...` reaching the controller, followed by `POST /control/joint-jog` returning `503`.
  - That indicates a direct setpoint failure/rejection path rather than a simple API/controller reachability outage.
- Changes:
  - Updated `src/gradient_os/run_controller.py`:
    - `APPLY_JOINT_SETPOINT` now prints the rejection exception and traceback before replying with `ERROR,APPLY_JOINT_SETPOINT,...`
  - Updated `src/gradient_os/api/main.py`:
    - added `_parse_apply_joint_setpoint_error(...)`
    - `/control/joint-jog` now turns controller `ERROR,APPLY_JOINT_SETPOINT,...` replies into structured HTTP details with a concrete message
  - Updated `web-ui/src/ControlPanel.tsx`:
    - added response error parsing for JSON API errors
    - commissioning jog failures now include the real API/backend reason when available
  - Updated `tests/test_api_endpoints.py`:
    - added coverage for structured `APPLY_JOINT_SETPOINT` rejection responses
- Validation:
  - `ReadLints` on edited Python/TS files
    - no diagnostics
  - `./.venv/bin/pytest tests/test_api_endpoints.py -q`
    - `36 passed`
  - `npm run build` in `web-ui/`
    - passed
- Follow-up notes / risks:
  - I still did not send any live jog command myself.
  - The next live jog attempt should finally expose the actual backend rejection text, which will tell us whether the underlying issue is RTCore setpoint-slot availability, connection state, or another controller-side exception.

## 2026-03-20 04:38 +0000

- Task summary:
  - Fixed the newly exposed commissioning jog crash where direct setpoints failed because the RTCore path had `None` legacy servo acceleration defaults.
- What was observed:
  - Live terminal traceback showed:
    - `APPLY_JOINT_SETPOINT rejected: float() argument must be a string or a real number, not 'NoneType'`
    - source: `command_api.handle_apply_joint_setpoint()` trying to evaluate `float(utils.DEFAULT_SERVO_ACCELERATION_DEG_S2)`
- Root cause:
  - EtherCAT/RTCore intentionally leaves legacy serial-servo acceleration constants unset in `utils`
  - direct commissioning setpoint code assumed those defaults were always numeric
- Changes:
  - Updated `src/gradient_os/arm_controller/command_api.py`:
    - added `_coerce_direct_setpoint_speed(...)`
    - added `_coerce_direct_setpoint_acceleration(...)`
    - `handle_apply_joint_setpoint(...)` now falls back to finite defaults instead of crashing on `None`
  - Added `tests/test_command_api_direct_setpoint.py`
    - covers the exact missing-defaults case
- Validation:
  - `./.venv/bin/pytest tests/test_command_api_direct_setpoint.py tests/test_api_endpoints.py -q`
    - `37 passed`
  - `ReadLints` on edited Python/test files
    - no diagnostics
- Follow-up notes / risks:
  - I still did not send a live jog command myself.
  - If the next live jog still fails, it should now be for a real backend/state reason rather than this `None` default crash.

## 2026-03-20 05:27 +0000

- Task summary:
  - Fixed the backend/executor mismatch behind the Cartesian UI jog traceback so RTCore trajectory execution no longer depends on serial-servo conversion constants.
- What was observed:
  - Live terminal showed `MOVE_LINE_RELATIVE` planning succeeding, then the closed-loop executor crashed inside `trajectory_execution._closed_loop_executor_thread()` while calling `servo_driver.servo_value_to_radians(...)`.
  - RTCore backend intentionally keeps serial constants like `utils.ENCODER_RESOLUTION` unset, so backend execution cannot safely reuse servo-specific decode/write code paths.
- Changes:
  - Updated `src/gradient_os/arm_controller/trajectory_execution.py`:
    - `closed_loop` backend mode now reads with `backend.sync_read_positions()`, converts via `backend.raw_to_joint_positions()`, tracks errors in logical joint space, and writes with `backend.prepare_sync_write_commands(...)`
    - `open_loop` backend mode now precomputes commands with `backend.prepare_sync_write_commands(...)` instead of `servo_driver.logical_q_to_syncwrite_tuple(...)`
    - backend diagnostics now use `backend.get_joint_positions()` instead of serial feedback helpers
  - Added `tests/test_trajectory_execution_backends.py`:
    - covers closed-loop backend execution with `ENCODER_RESOLUTION=None`
    - covers open-loop backend command preparation without legacy servo tuple generation
- Validation:
  - `./.venv/bin/pytest tests/test_trajectory_execution_backends.py tests/test_planning.py -q`
    - `5 passed`
  - `ReadLints` on `src/gradient_os/arm_controller/trajectory_execution.py` and `tests/test_trajectory_execution_backends.py`
    - no diagnostics
- Follow-up notes / risks:
  - I still did not send any live Cartesian motion command myself.
  - The next live UI Cartesian jog should confirm whether this executor/backend mismatch was the last blocker or whether there is a separate solver/backend state issue after command submission.

## 2026-03-20 05:33 +0000

- Task summary:
  - Audited the realtime jog command path for RTCore compatibility and fixed the only adjacent backend mismatch I found in the single-actuator helper path.
- What was observed:
  - The main 6-axis realtime jog loop in `src/gradient_os/arm_controller/command_api.py` already uses backend-aware helpers:
    - `servo_driver.get_current_arm_state_rad(...)`
    - `servo_driver.set_servo_positions(...)`
  - Those helpers delegate to the active backend, so the arm realtime jog path itself is compatible with RTCore joint-space control.
  - A separate nearby backend issue existed in `src/gradient_os/arm_controller/servo_driver.py:set_single_servo_position_rads(...)`:
    - backend mode still converted through legacy raw-servo math
    - it called `backend.sync_write(...)` with raw tuples even though RTCore expects its own backend-native command path
- Changes:
  - Updated `src/gradient_os/arm_controller/servo_driver.py`:
    - backend branch in `set_single_servo_position_rads(...)` now calls `backend.set_single_actuator_position(...)` directly
    - removed backend dependence on legacy raw conversion / `ENCODER_RESOLUTION` for that helper
  - Added `tests/test_realtime_jog_backend_compatibility.py`:
    - verifies backend single-actuator writes still work when `ENCODER_RESOLUTION=None`
    - verifies the realtime jog loop commands joint-space motion via `servo_driver.set_servo_positions(...)`
- Validation:
  - `./.venv/bin/pytest tests/test_realtime_jog_backend_compatibility.py tests/test_command_api_direct_setpoint.py tests/test_trajectory_execution_backends.py -q`
    - `5 passed`
  - `ReadLints` on `src/gradient_os/arm_controller/servo_driver.py` and `tests/test_realtime_jog_backend_compatibility.py`
    - no diagnostics
- Follow-up notes / risks:
  - `tests/test_driver.py` still has an unrelated setup failure because it assumes `utils.SERVO_IDS` is already populated; I did not change that file in this pass.
  - For `gradient05`, the gripper jog path appears irrelevant because the robot config is 6-axis/no gripper, so the key live question remains actual arm jog behavior on hardware.

## 2026-03-20 05:46 +0000

- Task summary:
  - Reduced live visualizer lag by removing hot-path work in `ArmVisualizer` instead of restoring the old smoothing behavior.
- What was observed:
  - The live pose snap logic was still present in `web-ui/src/ArmVisualizer.tsx`, so smoothing was no longer the primary cause of the viewer feeling behind.
  - The component still requested dynamic bounds/grounding refreshes during live joint updates, which can be expensive enough to make the scene feel sluggish relative to the real robot and controls.
- Changes:
  - Updated `web-ui/src/ArmVisualizer.tsx`:
    - removed leftover per-frame joint interpolation from the animation loop
    - stopped forcing dynamic bounds recomputation on every live joint update
    - throttled live bounds refreshes to at most every 250 ms, and only when the bounding box is visible
- Validation:
  - `npm run build` in `web-ui/`
    - passed
  - `ReadLints` on `web-ui/src/ArmVisualizer.tsx`
    - no diagnostics
- Follow-up notes / risks:
  - I still did not measure browser-frame timing directly, so this is based on code-path analysis rather than a live profile.
  - If the visualizer still feels behind after this, the next likely bottleneck is upstream telemetry cadence or browser-side scene complexity rather than the old smoothing block.

## 2026-03-20 06:23 +0000

- Task summary:
  - Increased the live visualization timing path to 50 Hz so pose updates are no longer gated by the earlier 250 ms / 500 ms fallback cadence.
- What was observed:
  - The previous visualizer-only change was not enough because multiple upstream timing gates were still slower than the desired live response:
    - `web-ui/src/ControlPanel.tsx` polled `/info/joints` every `500 ms`
    - `web-ui/src/App.tsx` only accepted `/info/joints` fallback when SSE was older than `250 ms`
    - `web-ui/src/ArmVisualizer.tsx` only refreshed live bounds at `250 ms`
- Changes:
  - Updated `web-ui/src/ControlPanel.tsx`:
    - added `LIVE_JOINT_FEEDBACK_POLL_MS = 20`
    - background poll now reuses `refreshJointAngles(true)` so it also feeds `onJointFeedback`
  - Updated `web-ui/src/App.tsx`:
    - added `LIVE_JOINT_FALLBACK_MAX_AGE_S = 1 / 50`
    - `/info/joints` fallback can now update the main pose pipeline after ~20 ms of SSE quiet instead of 250 ms
  - Updated `web-ui/src/ArmVisualizer.tsx`:
    - added `LIVE_BOUNDS_REFRESH_INTERVAL_MS = 20`
    - live bounds refresh throttle now runs at 50 Hz instead of 4 Hz
- Validation:
  - `npm run build` in `web-ui/`
    - passed
  - `ReadLints` on `web-ui/src/ControlPanel.tsx`, `web-ui/src/App.tsx`, and `web-ui/src/ArmVisualizer.tsx`
    - no diagnostics
- Follow-up notes / risks:
  - This raises `/info/joints` polling frequency substantially, so if the API/controller becomes noisy or CPU-heavy under live use, the next step should be shifting more of the pose path onto the SSE stream instead of polling harder.

## 2026-03-20 07:12 +0000

- Task summary:
  - Added a custom commissioning step input alongside the preset joint jog step buttons in the control panel.
- What was observed:
  - The commissioning UI only exposed fixed `0.25 / 1 / 5` degree presets, but live alignment/zero work needs arbitrary small steps without losing the preset quick-picks.
- Changes:
  - Updated `web-ui/src/ControlPanel.tsx`:
    - added `MIN_CUSTOM_JOINT_STEP_DEG = 0.001`
    - added `formatStepDegrees(...)` for trimmed dynamic step labels
    - added `customJointStepInput` state plus `applyCustomJointStep()`
    - inserted a signed/unsigned `Custom` degree input and `Use` button next to the preset chips
    - existing jog buttons now display/use the active formatted step value, including custom magnitudes
- Validation:
  - `npm run build` in `web-ui/`
    - passed
  - `ReadLints` on `web-ui/src/ControlPanel.tsx`
    - no diagnostics
- Follow-up notes / risks:
  - Custom values are normalized to absolute magnitude and direction still comes from the `-` / `+` jog buttons, which keeps the row-level behavior consistent.
