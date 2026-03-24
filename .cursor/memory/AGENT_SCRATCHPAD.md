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

### 2026-03-23 - Jog mode idle-gap regression
- [self] In `web-ui/src/ControlPanel.tsx`, once a realtime jog session has started, do not suppress unchanged zero-velocity ticks forever. If zero keepalives stop, the controller lease can expire during the short gap between direction presses and the next hold will hit `SESSION_INACTIVE`.
- [self] Treat `SESSION_INACTIVE` / `SESSION_EXPIRED` / `SESSION_NOT_FOUND` as recoverable session-loss in the frontend publish loop: clear session tracking and retry/start a fresh session for the current non-zero payload instead of forcing the UI back to incremental mode.
- [tool] `logs/startups/latest/controller.log` + `api.log` together were the decisive check: they showed a zero-velocity `JOG_SESSION_UPDATE`, then the jog thread stopping, then the next direction falling through to `MOVE_LINE_RELATIVE`.

### 2026-03-23 - Lag review heuristics
- [self] When the user reports lag during motion, check `logs/startups/latest/api.log` for polling fanout before blaming planner/runtime. In this session the latest API slice showed at least four concurrent localhost clients polling `/info/joints` and `/control/motion-status` at the same time.
- [tool] `logs/startups/latest/controller.log` is good for separating real motion/planner latency from UI feel: this run showed coarse `MOVE_LINE_RELATIVE` planning around `275-284 ms`, small 1 mm moves around `38-42 ms`, and one `APPLY_JOINT_SETPOINT` that took `3.926 s` because the commanded trajectory itself was 384 steps long.
- [tool] There is a real host-side legacy connectivity service outside the web UI: `systemd/wifi/gradient-wifi-keepalive.sh`. On this Pi it was confirmed `enabled` and `active` while `nmcli device status` showed `eth1 connected` and `wlan0 disconnected`, so if Wi-Fi fallback is no longer desired this service is a good cleanup target before chasing smaller browser `/health` polls.
- [self] Low-risk polling consolidation pattern: keep `App` as the owner of shared live state like `motionStatus`, and let `ControlPanel` consume/update that shared state via props instead of polling the same endpoint independently. This removed a duplicate `/control/motion-status` loop without touching jog deadman or unload-stop safety.

### 2026-03-24 - Shared live-state implementation
- [self] Best low-risk read-path consolidation here is hybrid, not all-at-once: first make the frontend consume `/monitor` as the primary shared source, then move missing fields like `motion_status` into the existing controller telemetry packet instead of inventing a second live snapshot protocol.
- [self] For `ControlPanel`, the important fallback rule is not "provider present" but "monitor fresh with usable joints". When `/monitor` is fresh, derive panel joint angles from SSE and suppress `/info/joints`; when stale, fall back to REST polling. This preserves commissioning readouts without forcing duplicate high-rate polling during normal operation.
- [tool] `run_controller.py` can safely expose motion summary in monitor telemetry by caching `command_api.get_motion_execution_status()` and refreshing it at a lower rate (for example 10 Hz) than the main telemetry packet. That keeps `/monitor` useful without paying the full motion-status assembly cost on every 50 Hz frame.

### 2026-03-24 - Post-change log review
- [tool] After the shared live-state change, the controller-side request mix is the clearest success metric: in this session `logs/startups/latest/controller.log` showed only `GET_JOINT_ANGLES=4` and `GET_MOTION_STATUS=11`, while `GET_PERFORMANCE_STATE=437` dominated the remaining deliberate polling. Use controller-side command counts to judge whether core polling actually dropped.
- [self] If the diagnostics panel is open, `/debug/performance` becomes the primary remaining live poller and can easily dominate API/controller request volume. Treat diagnostics polling as an opt-in load source when reviewing lag reports.
- [tool] Repeated `[Jog] NOTE: IK target clamped at joints: [5]` during yaw jog indicates a limit-hit condition, not a session/control-path failure. When users report “lag” or weak response in that direction, check for clamp notes before blaming transport or UI cadence.

### 2026-03-24 - Diagnostics polling + yaw investigation
- [self] Diagnostics collection here is useful even while the operator is on other tabs because the panel preserves the last successful sample after shutdown. Better compromise than hard-gating by active tab: keep background polling while the diagnostics tab feature is visible, but slow it down (for example `1000 ms`) so it remains useful without dominating request volume as much.
- [self] The clamp log uses zero-based indices. `[Jog] NOTE: IK target clamped at joints: [5]` means the sixth logical joint (`J6`), not `J5`.
- [tool] Normal startup/controller logs do not include end-effector `xyz/rpy` for jog steps by default. In this repo, `SET_JOG_DEBUG,true` enables per-step jog pose logs (`CURR pos` / `TARG pos`), and `/info/pose` can return `CURRENT_POSE`, but this latest session did not have either in the logs.
- [self] The active runtime clamp source is `robot.logical_joint_limits_rad` via `robot_config.LOGICAL_JOINT_LIMITS_RAD`, not live URDF parsing at jog time. The URDF remains an upstream reference, but config drift is possible and must be checked directly.
- [self] Low-risk way to make pose/joint evidence durable for post-run diagnosis: attach a best-effort `GET_POSITION` snapshot to `/debug/performance` and render it in the diagnostics panel. If the pose probe misses once, do not fail the whole performance endpoint.

### 2026-03-24 - Joint limit operator warning
- [self] When the user asks for an obvious warning, prefer reusing the existing monitor alert rail instead of inventing a new diagnostics-only path. That keeps the warning visible during normal operation, not only after the fact.
- [self] For jog-limit hits, emit the alert at the actual clamp site in `command_api._jog_controller_thread()` so the warning reflects a real applied controller constraint, not just an inferred near-limit condition.
- [self] Include joint labels in human terms (`J6 upper`) and remember the zero-based-to-operator numbering conversion. A backend/internal index like `[5]` should never be the only thing the user sees.

### 2026-03-24 - New run drift confirmation
- [tool] The latest run in `logs/startups/latest/controller.log` shows real TCP drift during angular-only jog with no `IK target clamped` and no `JOINT_LIMIT` entries. Do not assume the newest drift reports are still limit-related.
- [tool] In the latest captured pose samples, the clearly aligned angular-only segments were roll commands (`v_roll=+-15`, `v_yaw=0`) and still showed position drift on the order of centimeters over the sampled window. This confirms a broader orientation-only pose-hold problem, not just a yaw-specific limit issue.

### 2026-03-24 - Full diagnostics pose history
- [user] When the user asks to "code it", prefer shipping the next usable diagnostic instrument immediately rather than debating architecture. For this drift workflow, a visible/exportable pose timeline is more useful than another verbal analysis pass.
- [self] Best low-risk implementation for post-run drift inspection is frontend retention of `/debug/performance` pose snapshots: keep collecting while diagnostics are enabled, preserve the samples after shutdown, and expose clear/export controls so the operator can inspect or share the full trace.
- [self] Single latest pose samples are not enough for jog drift debugging. Capture a time-series including `xyz`, `rpy`, joints, motion state, and session state so `rpy`-only and future `xyz` tests can be compared sample-by-sample.
- [self] A good compromise for diagnostics load versus motion fidelity here is adaptive polling: faster while motion/jog is active (implemented at `250 ms`) and slower when idle (`1000 ms`) so the trace is denser during motion without making idle diagnostics the dominant load source.

### 2026-03-24 - Latest session log interpretation
- [tool] In the newest `logs/startups/latest/controller.log`, one positive-yaw segment still hits repeated `[Jog] NOTE: IK target clamped at joints: [5]` while pose samples drift upward in `z`; do not treat that portion as proof of the general drift root cause because it is limit-constrained.
- [tool] The same session also contains a separate negative-yaw segment with `v_yaw=-15.0` and no clamp notes in the inspected window, yet the sampled TCP still drifts by millimeters in `x/y/z`. That is stronger evidence the angular jog path still has a non-limit-related Cartesian hold issue.
- [tool] The new full pose history is browser-resident unless exported. `logs/diagnostics/` does not automatically receive a new file from the frontend capture, so if exact post-run sample-by-sample analysis is needed outside the live browser, the operator must use the JSON export button.

### 2026-03-20 - Servo profile refactor
- [user] DS402 / EtherCAT AL decoding must live in shared profile code, not in `src/gradient_os/run_controller.py` or `start-stack.sh`.
- [self] For runtime policy changes, `compute_restart_required()` must compare the fully resolved desired runtime, including unsafe backend/profile overrides; comparing only robot defaults hides real restart requirements.
- [self] Keep `WAIT_FOR_IDLE` explicitly trajectory-thread-scoped. Direct commissioning setpoints should return backend acceptance and let the UI describe `accepted` vs `faulted` / `not operation-enabled`, rather than claiming the move completed.
- [tool] `ReadLints` on large TS files can expose stale orphaned code outside the new edit. In this task `web-ui/src/App.tsx` had an old `ee_pose` fragment referencing undefined names and needed cleanup before handoff.

### 2026-03-19 - EtherCAT stop/start sequencing
- [user] For robot shutdown, the preferred normal outcome is a safe disarmed/ready fieldbus state, not blindly tearing down EtherCAT if that provokes drive faults.
- [self] `ErC1.1` on the A6-EC drives maps to EtherCAT synchronization loss; if shutdown drops RTCore or `ethercat.service` after the drives are still synced, the panels will fault even if the brakes engaged safely.
- [self] In `start-stack.sh`, make soft stop the default and reserve fieldbus teardown for an explicit hard-stop mode.
- [self] Do not advance staged startup just because the controller UDP socket is alive; also gate on RTCore metrics showing all configured slaves responding, online, operational, and `startup_ready=1`.
- [user] Servo fault-code references are drive-specific. Only apply A6-EC fault interpretation when the active servo backend matches the current EtherCAT/A6-EC path; otherwise keep probe output raw.
- [self] Avoid calling `exit` directly from signal traps while `start-stack.sh` is blocked in `wait -n`; use a shutdown flag and let the supervise loop return normally so cleanup runs deterministically.
- [user] The live launcher terminal should accept commands directly. Keep the command entry pinned at the top so telemetry/log spam does not bury the input line.
- [self] If a script redirects stdout through `tee`, do not use `-t 1` to decide whether a terminal UI is available. Capture the real tty path early and bind the interactive UI to that device explicitly.
- [self] A heredoc-fed `python - <<'PY'` script cannot also use stdin for live keyboard input. For terminal UIs, pass the tty path as an argument and rebind fd `0/1/2` to the tty from inside Python after the script has started.
- [user] Prefer the lighter `scripts/rtcore_jog.py`-style line console over a full-screen embedded curses console when terminal control becomes brittle. Redrawing the active prompt is enough.
- [tool] Live validation on 2026-03-19 confirmed the `start-stack.sh` line console accepts `stop` cleanly under active log spam; this pattern is validated on the robot terminal and should be preferred over the earlier curses attempt.
- [self] Background children launched from bash can inherit ignored `SIGINT`, so a launcher-side `kill -INT` is not reliable for graceful shutdown. Prefer a controller-owned `SIGTERM` handler plus launcher-side `SIGTERM` as the primary graceful stop signal.

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

### 2026-03-18 - EtherCAT zeroing workflow
- [user] For hardware bring-up on the EtherCAT backend, prioritize an actual zeroing implementation over a design-only answer.
- [self] `SET_ZERO` semantics from the legacy serial path are not sufficient for `ethercat_rtcore` when RT axes are remapped (for example bring-up overrides like `GRADIENT_RTCORE_CONTROL_JOINTS="3,4"`); use backend-aware logical-joint zeroing instead of static robot actuator IDs.
- [self] RTCore already exposes enough information to implement software zero capture cleanly: read `STATUS_AXIS_CONFIG`, convert `0x6064` counts to q-units, then persist logical master offsets repo-locally rather than trying to write drive EEPROM zero during commissioning.
- [self] After EtherCAT backend changes, run focused API/backend pytest coverage plus `ReadLints`; this task passed with `./.venv/bin/python -m pytest tests/test_gradient05_limits_and_backends.py tests/test_api_endpoints.py -q`.

### 2026-03-18 - Frontend joint commissioning
- [user] Frontend commissioning flow must expose individual joint jog plus joint zero capture for the EtherCAT hardware path; reuse existing controller/run-controller pathways rather than inventing a separate control stack.
- [user] During setup/zeroing, preserve the existing RTCore safety expectation of `--max-rpm 100`; UI should reinforce that commissioning assumption instead of encouraging large/faster moves.
- [self] `web-ui/src/ControlPanel.tsx` already had Cartesian jog; extending the same panel with live joint feedback polling (`/info/joints`), relative joint jog (`/control/joint-jog`), and per-joint zero capture (`/control/zero-joint`) keeps commissioning controls close to the existing stop/home/rest affordances.
- [tool] Full `web-ui` build/typecheck is currently blocked by pre-existing environment/app issues outside this panel change: Vite wants newer Node than `18.20.4`, there is an existing `occt-import-js.wasm` resolution failure, and `npx tsc --noEmit` reports many pre-existing `three`/`ArmVisualizer` typing issues. Use `ReadLints` plus focused backend tests to validate incremental UI work until that baseline is fixed.

### 2026-03-19 - EtherCAT stack bring-up validation
- [self] A healthy software bring-up can still mask a dead fieldbus: RTCore/controller/API may all come up and expose 6 logical axes while EtherCAT still reports `Slaves: 0`, `Rx frames: 0`, `Frame loss: 100%`, and all raw positions remain `0`.
- [self] When `gradient-rt-motion.service` reports `ecrt_request_master(0) failed` with `Device or resource busy`, check `/dev/EtherCAT0` ownership with `sudo fuser -v /dev/EtherCAT0`; in this session a stale root-owned manual RTCore child (`/usr/local/bin/gradient-rt-motion`) was the blocker.
- [self] `run.sh` did not previously start RTCore automatically for `ethercat_rtcore`; adding a best-effort autostart in `run_controller.py` plus wiring `arm-controller.service` to `gradient-rt-motion.service` makes the high-level path much closer to appliance behavior.
- [tool] `npm install` in `web-ui` corrected local dependency drift from Vite `7.1.10` back to `5.4.21`, allowing the dev server to run on Node `18.20.4`. Distinguish that from separate frontend build/type issues when debugging startup.

### 2026-03-19 - EtherCAT NIC binding contradiction
- [self] Historical EtherCAT docs in `docs/ethercat/igh.md` and `docs/ethercat/bringup.md` explicitly say discovery worked on `eth0` (`macb`, MAC `...:75`) and repeatedly failed on `eth1` (`lan743x`, MAC `...:76`), so the repo templates bind `MASTER0_DEVICE` to `c8:3e:a7:14:1c:75`.
- [tool] Live host state can invalidate that old assumption: on this machine `nmcli device status` shows `eth0` is the active uplink (`connected`, default route, IPv4 addresses) while `eth1` is disconnected, even though `/etc/systemd/network/10-ethercat0.link` and `10-uplink0.link` are installed.
- [self] If the operator remembers the normal wired connection is on `eth0`, then current `/etc/ethercat.conf` is binding the EtherCAT master to the uplink-facing NIC, which is a strong candidate root cause for `Slaves: 0` / `Rx frames: 0`.
- [self] The old rename/unmanaged configs are installed but not actually effective in runtime naming/policy here; verify physical wiring and consider a temporary `ethercat-eth1.conf` diagnostic run before assuming the historical `eth0` choice still applies.
- [tool] The temporary `ethercat-eth1.conf` diagnostic produced an immediate positive result: `sudo ethercat master` showed `Slaves: 6`, `Rx frames: 19`, `Lost frames: 0`, proving the live slave chain is currently reachable on MAC `...:76` / `eth1`.

### 2026-03-19 - EtherCAT eth1 default switch
- [self] After confirming the slave chain is on `eth1`, update both the repo templates and the live host config together: `systemd/ethercat-host/ethercat.conf`, `10-ethercat0.link`, `10-uplink0.link`, `10-unmanaged-ethercat.conf`, plus the matching live `/etc/*` files. Leaving them split would preserve the contradiction for the next reboot/install.
- [tool] The normal stack can look "more alive" on the correct NIC without actually being healthy end-to-end: after switching `/etc/ethercat.conf` to MAC `c8:3e:a7:14:1c:76`, `ethercat master` showed RX traffic on `eth1`, the controller/API came back, but RTCore metrics still stayed `wkc=0/0` and raw positions remained zero.
- [self] Use a split test to separate NIC binding from RTCore behavior: bare IgH on the corrected `/etc/ethercat.conf` could enumerate six slave slots again, but once `gradient-rt-motion.service` owns the master the CLI matches zero selected slaves and RTCore remains at zero WKC. Treat those as two different layers of the problem.
- [self] Historical notes saying "eth0 worked, eth1 failed" are not strong enough to override a fresh live probe on the same machine. The likely failure mode is not that `eth1` became impossible, but that the earlier conclusion captured a different live wiring/runtime state and then got fossilized into docs/templates.

### 2026-03-19 - RTCore WKC follow-up
- [self] The RT thread/core setup is not the primary failure: `gradient-rt-motion` runs its `rt-cycle` thread on CPU3 with `SCHED_FIFO 90` exactly as intended. The suspicious core-adjacent problem was host tuning drift, not RT thread pinning itself.
- [self] `systemd/ethercat-host/gradient-ethercat-nic-tune.sh` and `gradient-irq-affinity.sh` were still assuming pre-rename fallback `eth0`; on this host that meant the actual EtherCAT NIC (`eth1`, MAC `...:76`) kept `GRO/GSO/TSO` enabled and `EEE` enabled. Updating those scripts to resolve the active EtherCAT NIC from `/etc/ethercat.conf` is necessary.
- [tool] The `lan743x` MSI IRQs for `eth1` are exposed via `/sys/class/net/eth1/device/msi_irqs`, not by interface name in `/proc/interrupts`. Even after fixing discovery and the hex mask format, writing affinity still returns `I/O error` for those IRQs on this kernel, so do not assume the planned CPU2-3 IRQ pinning is actually possible on this platform.
- [tool] Applying the NIC tuning to `eth1` materially improved bus behavior under RTCore: RX traffic increased significantly and frame loss dropped from near-total to partial loss, which proves the tuning path matters, but it still did not restore non-zero WKC.
- [self] A manual `sudo ethercat rescan` is currently a crucial diagnostic step: after rescan, bare IgH recovers full identities for all six `AS715N_sAxis_V0.10` slaves with CoE mailboxes on `eth1`.
- [self] The strongest current root-cause lead is PDO/state sequencing, not CPU cores. In a one-axis RTCore run, slaves stay visible and the master gets a DC reference clock, but all slaves remain `INIT`, `wkc` stays `0`, and `ethercat pdos -p0` shows live PDOs `0x1701/0x1b01` while RTCore is hard-coded for `0x1702/0x1b02`.

### 2026-03-19 - RTCore startup diagnostics hardening
- [self] The old RTCore diagnostics were actively misleading: `wkc_expected` latched from observed traffic and stayed `0` during failures, and `master_state` was derived from `armed` rather than real EtherCAT AL state. Fixing those made the startup picture much clearer immediately.
- [self] Expose real startup data from the RT loop itself: master link/al-state, responding slave count, online/operational counts, domain WC state, startup elapsed time, reset count, and per-axis slave AL state/online/operational flags. Surface them in both `metrics.json` and `scripts/sampler/rtcore_metrics.py summary`.
- [tool] IgH explicitly documents that `ecrt_slave_config_state()` can take a few cycles to reflect changes, so a bounded startup-convergence window is justified. However, on this host the new logs show more than simple slowness: RTCore sees `6/6` slaves online in `INIT` at `t=0`, then falls back to `0/6` responders within about 1 second.
- [self] That regression means "just wait longer" is not sufficient by itself. The new wait window is still valuable because it distinguishes slow convergence from active loss of topology/state during RTCore bring-up.

### 2026-03-19 - EtherCAT handoff summary
- [self] For a fresh-instance handoff on this issue, point the next agent first to `docs/ethercat/bringup.md`, `docs/ethercat/igh.md`, `.cursor/memory/DEVLOG.md`, `.cursor/memory/AGENT_SCRATCHPAD.md`, `src/gradient_rt_motion/main.cpp`, `scripts/sampler/rtcore_metrics.py`, and the `systemd/ethercat-host/` scripts.
- [self] Preserve the exact current failure signature in the handoff: RTCore startup initially sees `6/6` slaves online in `INIT`, then regresses to `0/6` responders within about 1 second; final steady state is `wkc=0/12`, `link_up=1`, `responding=0/6`, `online=0/6`, `operational=0/6`.

### 2026-03-19 - RTCore profile split tests
- [self] The hard-coded `0x1702/0x1b02` assumption was worth turning into a real runtime knob: `src/gradient_rt_motion/main.cpp` now supports `--rx-pdo`, `--tx-pdo`, `--no-dc`, `--wait-before-safeop-ms`, `--preop-safeop-timeout-ms`, and `--safeop-op-timeout-ms`, and `scripts/sampler/rtcore_metrics.py summary` now prints the active bring-up profile/policy.
- [tool] The vendor ESI in `docs/resources/ethercat/esi/stepperonline/A6-EC/STEPPERONLINE_A6_Servo_V0.02.xml` confirms `0x1701/0x1b01` is not a cosmetic alias for `0x1702/0x1b02`; the layouts differ materially (`0x1701` lacks `0x6060/0x60FF/0x6071/0x607F`, `0x1b01` exposes `0x60F4` instead of `0x6061`).
- [tool] Live split result on this host: `sudo /usr/local/bin/gradient-rt-motion --socket-path /tmp/rtdiag/ipc.sock --num-axes 1 --max-rpm 100 --rx-pdo 0x1701 --tx-pdo 0x1b01 --no-dc` kept topology visible (`responding=6/1`, `online=1/1`, `master_al=0x1`, `ethercat master` showed `Slaves: 6`) but still stayed `slave0_al=INIT` with `wkc=0/2`.
- [self] That means "wrong PDO IDs alone" is not sufficient to explain the failure anymore. Matching the live-visible `1701/0x1b01` profile and disabling DC improved stability in the reduced test, but the remaining blocker is still AL-state/configuration progression into real process-data exchange.

### 2026-03-19 - Multi-slave breakpoint
- [self] The important breakpoint is not "somewhere near 6 axes"; with the default `0x1702/0x1b02` + DC path, `--num-axes 1` is stable-in-`INIT`, while `--num-axes 2` and `--num-axes 3` already collapse to `responding=0` / `online=0` by about 1 second.
- [tool] Matching the lighter profile and disabling DC does not remove that breakpoint: `--num-axes 2 --rx-pdo 0x1701 --tx-pdo 0x1b01 --no-dc` still collapses to `0/2`, even though `--num-axes 1` with the same profile remains stable-in-`INIT`.
- [self] To distinguish "slave position 1 is bad" from "any multi-slave configuration is bad", RTCore now supports `--slave-positions`; `--num-axes 1 --slave-positions 1` behaved like `--slave-positions 0` and stayed stable-in-`INIT` with `responding=6/1`, `online=1/1`, `wkc=0/2`.
- [self] Current best inference: the failure is triggered when RTCore tries to configure more than one slave at once. That points more strongly at multi-slave AL-state/configuration sequencing than at a single bad slave or pure PDO-ID mismatch.

### 2026-03-19 - Two-slave full sweep
- [tool] Full 15-pair sweep with the default path (`--num-axes 2 --slave-positions i,j --max-rpm 100`) produced the same failure for every pair: `responding=0`, `online=0`, `operational=0`, `master_al=0x0`, `wkc=0/4`, `startup_resets=1`.
- [self] That rules out a special-case "bad pair" among the current six slaves, at least at the coarse level of the present diagnostics. The current software failure mode is generic to configuring any two slaves together.
- [self] Given the user's note that two drives previously moved simultaneously on this hardware, treat this as a current bring-up/configuration regression or changed startup state, not a fundamental EtherCAT topology limit of the machine.

### 2026-03-19 - Fresh-instance handoff emphasis
- [self] The most important current conclusion is: one configured slave stays stable in `INIT`, any two configured slaves collapse, and all 15 two-slave pairs fail the same way. A fresh instance should start from that breakpoint, not from older NIC/core hypotheses.
- [self] The best next implementation direction is `src/gradient_rt_motion/main.cpp`: add per-slave AL-state transition logging for the first second and experiment with staged multi-slave activation/configuration instead of configuring all selected slaves in one shot.

### 2026-03-19 - Post-activate collapse timing
- [self] When debugging RTCore bring-up, distinguish config-time success from post-activate success. On this host the multi-slave path survives `ecrt_master_slave_config`, PDO assignment, DC config, PDO registration, and `ecrt_master_activate`, then collapses only after cyclic exchange starts.
- [tool] The new manual two-slave probe (`sudo timeout 8s ./src/gradient_rt_motion/gradient-rt-motion --socket-path /tmp/rtdiag_phase/ipc.sock --num-axes 2 --slave-positions 0,1 --max-rpm 100`) showed both slaves stable in `INIT` until about `448 ms`, then both dropped together to `online=0` / `al=0x0`.
- [tool] In the normal 6-axis service run, the collapse is slightly staggered: axes `0-1` disappear at about `444 ms`, while axes `2-5` disappear one cycle later at about `445 ms`. Preserve that ordering in future hypotheses and staged-bring-up experiments.
- [self] Rebuilding `src/gradient_rt_motion/gradient-rt-motion` is not enough for live service diagnostics; `gradient-rt-motion.service` runs `/usr/local/bin/gradient-rt-motion`, so install the rebuilt binary before trusting the service journal.

### 2026-03-19 - Timing bug versus real breakpoint
- [self] If startup logs show impossible timing like `cycle=2042` at `elapsed_ms=444` in a 1 kHz loop, suspect a stale absolute-sleep schedule. On this host `next_ns` was initialized before multi-second EtherCAT setup, so the loop tried to "catch up" after activation.
- [self] Fix that by rebasing `next_ns = now_monotonic_ns()` after initialization and by using the real wake timestamp, not the future sleep target, for startup elapsed diagnostics.
- [tool] After that fix, the two-slave collapse still happened at about `452-456 ms`, and changing `--wait-before-safeop-ms` between `0`, `250`, and `1000` did not move the breakpoint. Treat the `~450 ms` collapse as real, not as a scheduler artifact and not as a direct consequence of the SAFEOP wait knob.
- [self] Current best interpretation: the master survives static config/activate, but the first few hundred milliseconds of multi-slave cyclic process-data exchange never achieve non-zero WKC; after enough failed cycles, IgH marks the configured slaves offline. Focus next on why multi-slave cyclic PDO traffic is never acknowledged, not on the old schedule bug.

### 2026-03-19 - Passive startup result
- [self] A passive-output startup window is a good discriminator: if the bus still collapses while RTCore sends inert outputs only, the root cause is below DS402 controlword/mode/target logic.
- [tool] New knob in `src/gradient_rt_motion/main.cpp`: `--startup-passive-ms`. During that window RTCore still runs cyclic EtherCAT (`receive/process/queue/send`) but forces inert outputs instead of normal DS402 writes.
- [tool] On this host, `--startup-passive-ms 1500` did **not** help the two-slave case: both slaves still collapsed at about `448 ms`, while passive mode was still active.
- [tool] Even the stripped-down combination `--rx-pdo 0x1701 --tx-pdo 0x1b01 --no-dc --startup-passive-ms 1500` still collapsed at about `464 ms`.
- [self] Therefore the current blame shifts away from PREEMPT_RT/core placement and away from higher-level RTCore output sequencing. The remaining target is lower-level multi-slave process-data handling in the EtherCAT/libecrt path (PDO/FMMU/sync-manager/watchdog/runtime behavior).

### 2026-03-19 - Output watchdog toggle result
- [self] A `~450 ms` collapse invites watchdog suspicion, so add a runtime way to disable the output sync-manager watchdog before assuming a deeper master bug.
- [tool] New knob in `src/gradient_rt_motion/main.cpp`: `--disable-output-watchdog`. It copies the selected sync table into a mutable runtime table and flips the output SM watchdog to `EC_WD_DISABLE`.
- [tool] On this host, `--disable-output-watchdog --startup-passive-ms 1500` still collapsed at about `460 ms` in the two-slave probe.
- [self] That further narrows the failure: it is not just PREEMPT_RT timing, not just DS402 writes, and not just the output SM watchdog. Focus future debugging on lower-level multi-slave cyclic process-data activation/exchange semantics.

### 2026-03-19 - Send interval result
- [self] The IgH docs explicitly call out `ecrt_master_set_send_interval()` as input to how the master FSM decides how much data to append to frames, so it is a worthwhile master-side knob to test in multi-slave cyclic failures.
- [tool] Added `ecrt_master_set_send_interval(master, 1000)` before activation for the 1 kHz loop; the master accepted it and logged success.
- [tool] On this host, that did not change the two-slave breakpoint: collapse still happened at about `448 ms` with `WKC=0`.
- [self] So the failure is not explained by PREEMPT_RT timing, DS402 writes, output watchdogs, or the master lacking its advertised send interval. The remaining target is deeper activated multi-slave process-data behavior.

### 2026-03-19 - Domain queue discriminator
- [tool] New knob in `src/gradient_rt_motion/main.cpp`: `--startup-skip-domain-queue-ms`. In the two-slave probe, suppressing `ecrt_domain_queue()` for `1500 ms` kept topology alive (`responding=6/2`, `online=2/2`, slaves still `INIT`) and collapse started only after queueing resumed at about `1640 ms`.
- [self] Treat queued application-domain traffic as the strongest trigger now. That does **not** exonerate config-time PDO/domain setup, but it rules out RT scheduling, DS402 writes, and "activate alone" as the sole cause.
- [tool] Splitting to per-axis domains alone (`--split-domains-per-axis`) and even combining that with `--rx-pdo 0x1701 --tx-pdo 0x1b01 --no-dc` did **not** move the `~456 ms` collapse when every configured domain was queued each cycle.
- [tool] New experimental runtime path: `--split-domains-per-axis --queue-split-domains-round-robin`. Queueing only one axis-domain per cycle is the first multi-slave mode that keeps the bus alive for the full 8 s probe (`responding=6/2`, `online=2/2`, `startup_resets=0`) on both `1702/1b02`+DC and `1701/1b01 --no-dc`, but `WKC` stays `0` and both slaves remain in `INIT`.
- [self] Explicit low-level PDO config is not yet a viable replacement path here: fixed PDO mapping calls failed (`ecrt_slave_config_pdo_mapping_add` on `0x1702`), and even explicit assign + `ecrt_slave_config_reg_pdo_entry_pos()` failed on the first output entry. Keep `ecrt_slave_config_pdos()` as the working configuration primitive for now and focus next on staged/limited runtime queueing plus AL-state progression.

### 2026-03-19 - Single-source EtherCAT port selection
- [user] The active EtherCAT/uplink port choice should be editable in one obvious place, not duplicated across many repo templates and host scripts.
- [self] Centralize the host role choice in `systemd/ethercat-host/port-layout.env` with a single `ETHERCAT_PORT="eth0|eth1"` switch and derive the generated `/etc` files from there.
- [tool] `systemd/ethercat-host/install.sh` now renders `/etc/ethercat.conf`, `/etc/systemd/network/10-ethercat0.link`, `/etc/systemd/network/10-uplink0.link`, and `/etc/NetworkManager/conf.d/10-unmanaged-ethercat.conf` from that one file, and installs `/etc/gradient-ethercat-host.env` as the durable host copy.
- [self] Runtime helper scripts should prefer the installed single-source config before hardcoded fallbacks. `gradient-ethercat-nic-tune.sh`, `gradient-irq-affinity.sh`, and `scripts/ethercat/diagnose_host.sh` now consult `/etc/gradient-ethercat-host.env`.

### 2026-03-19 - Clean eth0 retest changed the failure layer
- [tool] After changing `systemd/ethercat-host/port-layout.env` to `ETHERCAT_PORT="eth0"` and re-running `sudo ./systemd/ethercat-host/install.sh`, `/etc/gradient-ethercat-host.env` and `/etc/ethercat.conf` correctly switched the live master binding to MAC `c8:3e:a7:14:1c:75`.
- [self] Superseded by user correction: this was **not** a valid alternate-port EtherCAT comparison because the physical cable/path had not yet been moved to `eth0`.
- [user] When testing the other physical NIC, confirm the cable has actually been swapped before interpreting `no slaves` / `100% frame loss` as a meaningful port result.
- [self] Guardrail: treat `port-layout.env` + `install.sh` as only the logical host-side switch. A real cross-port test requires both the software switch and the physical recable/path change.
- [self] `gradient-rt-motion.service` was active during the retest. Before running manual RTCore probes, confirm whether the service should be stopped so probe binaries are not competing with the installed `/usr/local/bin/gradient-rt-motion` instance for the same master.

### 2026-03-19 - Uplink profile staging guardrail
- [self] The repo previously handled only the EtherCAT side (`/etc/ethercat.conf`, `.link` files, and NetworkManager unmanaged config). If the uplink role moves to the other NIC, also install a managed NetworkManager profile for that computed uplink port or SSH/API access will depend on stale manual host state.
- [tool] A generated `systemd/ethercat-host/gradient-uplink.nmconnection` profile now comes from `systemd/ethercat-host/port-layout.env` and follows the selected uplink NIC by MAC address, not by transient `eth0`/`eth1` naming.
- [self] Guardrail: do **not** auto-load a new static uplink profile during `install.sh` validation on a live box that still has an older static uplink profile active on the other NIC. That can activate both NICs with the same IP and create a self-inflicted duplicate-address problem.
- [tool] Safe pattern: install the keyfile to `/etc/NetworkManager/system-connections/` but leave activation to a later reboot, `nmcli connection reload`, or an explicit operator step once the physical uplink cable has been moved.

### 2026-03-19 - WiFi staging guardrail
- [user] The host should also have the user-specified WiFi network staged as an additional management path.
- [tool] `nmcli device wifi list ifname wlan0` showed the visible SSID is `colonise the moon_5G-1` (underscore form), not the space-only spelling from the message. Use the live scanned SSID string for NetworkManager profiles.
- [tool] A saved NetworkManager profile for that SSID can be created successfully, but on this host the activation attempt failed at the association stage (`supplicant: associating -> disconnected`) before DHCP/IP started.
- [self] Guardrail: if a requested WiFi profile fails before IP assignment, report it as an AP/auth/association-level issue and keep the machine on the known-good wired path instead of repeatedly churning WiFi state.

### 2026-03-19 - Cutover goal clarity
- [user] The operational goal is specifically: move the normal uplink back to `eth1` and keep `eth0` dedicated to EtherCAT.
- [self] When that is the goal, avoid drifting into side investigations about alternative port tests. The useful action is to make the software cutover path explicit and easy to execute after the cable move.
- [tool] Added `systemd/ethercat-host/activate-uplink.sh` so the staged `Gradient Uplink` profile can be activated on the computed non-EtherCAT NIC.
- [self] Important correction: for remote-only operation, the helper must be started **before** unplugging the current uplink cable. It should wait for carrier on the future uplink NIC and only then activate the new path; otherwise the SSH session dies before the cutover command can run.
- [tool] Final confirmed state after cleanup: `nmcli device status` shows `eth1` connected via `Gradient Uplink`, `eth0` disconnected from normal IP management, and `sudo ethercat master` on MAC `c8:3e:a7:14:1c:75` reports `Slaves: 6` with healthy RX/TX on `eth0`.
- [tool] Final WiFi cleanup: deleted both stale NetworkManager profiles `Aussie Broadband 9745` and `Aussie Broadband 9745 1`. Only the user-requested `colonise the moon_5G-1` WiFi profile remains staged.
- [tool] `Direct-PC` can reappear on `eth0` because the saved profile still exists and originally had `connection.autoconnect=yes`. Final cleanup is: set `Direct-PC` autoconnect to `no`, bring it down, then delete the profile entirely. After that only `Gradient Uplink` remains for wired networking and `eth0` cannot be reclaimed by the old static profile.

### 2026-03-19 - Correct eth0 baseline changes RTCore interpretation
- [user] The user was right that pushing EtherCAT master work onto `eth1` wasted time. For this host/wiring, stop re-litigating port selection once `eth0` is confirmed as the dedicated master NIC and `eth1` is clean uplink.
- [tool] On the corrected `eth0` baseline, the old “multi-slave bus collapses at ~450 ms” result no longer reproduces as the primary symptom. Raw EtherCAT on `eth0` is healthy (`Slaves: 6`, matched RX/TX), and RTCore does eventually converge.
- [self] Guardrail: if a manual probe says `./src/gradient_rt_motion/gradient-rt-motion: No such file or directory`, check the working directory before diagnosing EtherCAT. Running `./src/...` from `systemd/ethercat-host` is a false failure; use `/home/pi/GradientOS` as cwd or `/usr/local/bin/gradient-rt-motion`.
- [tool] Clean 2-axis manual probe on `eth0` (`--num-axes 2 --slave-positions 0,1`) showed delayed convergence, not permanent collapse: first nonzero process data at about `5419 ms`, `1/2 operational` by about `6002 ms`, and `2/2 operational` by about `9641 ms`, with final `wkc=6/4`.
- [tool] Fresh 6-axis service on `eth0` shows the same staged convergence: `process_data_live` at about `5423 ms`, then `1/6`, `3/6`, `5/6`, and full `6/6 operational` by about `8201 ms`, with stable metrics later showing `wkc=18/12`, `master_al=0x8`, and per-axis statusword `0x1650`.
- [self] Important telemetry fix: `domain_wc != 0` is **not** equivalent to “startup converged.” Require all selected slaves to be operational for `startup_ready`, and log the first nonzero WKC/domain state separately as `process_data_live`.
- [self] The hardcoded 5 s startup warning was misleading on this host. Startup warning thresholds should follow the configured PREOP->SAFEOP and SAFEOP->OP budgets; with current defaults the realistic total startup budget is about 10 s, not 5 s.

### 2026-03-19 - Controller/API validation on RTCore baseline
- [self] If RTCore is healthy but the controller/API still report zero joints, check for stale controller caching before blaming EtherCAT. `GET_JOINT_ANGLES` had been replying from `utils.current_logical_joint_angles_rad`; use a fresh `servo_driver.get_current_arm_state_rad(verbose=False)` read for live feedback endpoints.
- [self] `After=gradient-rt-motion.service` is not enough for this host. RTCore can take ~8 s to reach full OP, so the controller should gate startup on `/run/gradient-rt-motion/metrics.json` showing `startup_ready=1` and all expected axes operational, not merely on the service process existing.
- [tool] The RTCore IPC handshake can succeed before the Python backend has actually received status-axis-config / snapshot data. A short post-connect wait for status metadata avoids treating “connected UDS socket” as “usable live feedback.”
- [tool] One axis can report multi-turn raw counts on startup (observed axis2 around `-1046180`). For bounded rotary joints, wrap the converted feedback into the configured logical joint-limit window before FK/planning, or pose/orientation requests can fail even though raw servo feedback is present.
- [tool] Superseded by user correction on 2026-03-19: this note said missing `pyquik` should fall back to `ikfast`. The user explicitly rejected that policy for `gradient05`. Treat fallback to another solver as unacceptable unless the robot policy/config explicitly allows it.
- [tool] When stopping manual long-running validation processes started from Cursor, killing only the shell wrapper is not enough. Verify the actual Python children (`gradient_os.run_controller`, `gradient-api`) are gone and ports `3000`/`4000` are free, or the stale controller can keep the RTCore IPC slot and make the next startup look like an RTCore regression.

### 2026-03-19 - User correction on RTCore scaling ownership
- [user] Do not hard-code encoder-count to radian conversion or gear ratios in the EtherCAT backend. Robot mechanics must live in the robot definition/config files.
- [user] `gradient05` should stay a `numeric` IK robot by policy. The simulation backend already proves the robot definition is valid; do not "fix" that by changing the robot definition away from numeric.
- [self] The right split is:
  - robot config defines per-joint encoder counts/rev, gear ratios, signs, and derived counts-per-radian
  - RTCore/Python EtherCAT backend consumes that robot-defined scaling for feedback conversion
  - backend config modules stay backend-generic, not robot-specific
- [tool] RTCore currently publishes placeholder runtime scaling (`gear_ratio=1.0`) over IPC on this host, so if the Python backend blindly trusts the runtime axis-config message it overwrites the correct robot-defined conversion. Prefer robot-config scaling for Python feedback conversion until RTCore is also fed from the same robot definition.
- [self] A multi-turn count is not itself a bug. With correct robot-defined counts-per-radian, continuous encoder feedback can remain continuous and still produce valid FK without modulo wrapping hacks.

### 2026-03-19 - Handoff guardrails for jog-first validation
- [user] Treat the following as settled host ground truth unless new live evidence disproves it: `eth0` is the dedicated EtherCAT master NIC, `eth1` is uplink/management, RTCore on the correct `eth0` path converges slowly but reliably (first live process data about `5.4 s`, full `6/6 operational` about `8.2 s`), and raw EtherCAT health at steady state is `responding=6/6`, `online=6/6`, `operational=6/6`, `wkc=18/12`, `error_code=0x0000`.
- [user] Do **not** reopen the port-role debate or treat current host fallback away from `numeric` IK as a robot-definition problem. `gradient05` stays a `numeric` IK robot by policy.
- [user] Do **not** hard-code robot mechanics in the EtherCAT backend. Encoder counts/rev, gear ratios, and sign conventions must come from the robot definition/config surface.
- [self] Next-phase priority is **not** another blind RTCore scaling edit. First verify whether the existing web UI joint jog controls actually drive the real controller/API jog path, then have the user jog each joint individually and report actual motion, direction, apparent scaling, API feedback consistency, and any statusword/error/WKC anomalies.
- [self] Use those per-joint jog observations to confirm J1-J6 sign conventions and scaling sanity before unifying RTCore C++ command scaling with the same robot-definition source used by Python feedback conversion.
- [user] New mechanical correction for `gradient05` J5: there is a belt stage after the `18:1` planetary gearbox (`22T` driving `20T`). Before editing config, confirm the in-code `gear_ratio` convention direction. If it means motor revs per joint rev, the effective J5 ratio is likely about `18 * 20 / 22 = 16.36`, but this must be confirmed against code convention first.
- [tool] Helpful live sanity point from the last session: Python feedback using robot-config scaling produced a sane J3 reading of about `-28.73 deg` from raw count `-1046180`, which is evidence that controller-side robot-defined conversion is now being applied.

### 2026-03-19 - Frontend zeroing workflow guardrails
- [self] The joint commissioning panel already existed in `web-ui/src/ControlPanel.tsx`, but for real hardware commissioning it was too optimistic: joint step actions posted `/control/joint-jog` without `wait_for_idle`, then refreshed angles immediately. On live hardware that can make the UI look stale or broken even when the command path is fine.
- [self] For commissioning/zeroing, prefer correctness over responsiveness: explicit small joint steps should wait for motion completion before refreshing live feedback.
- [self] Do **not** allow zero capture from a UI row that does not currently show live joint feedback. Disabled step/zero controls until `GET /info/joints` returns a finite value for that joint make the workflow safer and reduce false debugging trails.
- [tool] A small commissioning status banner plus an explicit `Refresh` button in the panel makes it much easier to distinguish:
  - no live feedback yet
  - step command in progress
  - zero capture succeeded but refresh failed
  - refresh succeeded and the operator can continue
- [tool] Live stack sanity during this pass: controller on UDP `3000`, API on HTTP `4000`, and Vite on HTTP `8000` all started cleanly; `/info/joints` returned live values including J3 about `-28.73 deg`.
- [self] Important safety boundary: I did **not** click any jog/home/rest/zero controls in this pass, because the user should perform the first physical commissioning interaction and report what the arm actually does.

### 2026-03-19 - Critical joint-jog unit mismatch
- [user] Live commissioning report: clicking the J1 `-1°` / `+1°` buttons in the UI made multiple joints move several degrees on the real robot.
- [self] Root cause: `/control/joint-jog` in `src/gradient_os/api/main.py` read `GET_JOINT_ANGLES` in **degrees**, added a degree delta, then forwarded the raw comma-separated joint command directly to the controller.
- [tool] `src/gradient_os/run_controller.py` treats the raw comma-separated joint command path as **radians** and passes those values into `servo_driver.set_servo_positions(...)`, which also expects radians. This made current degree feedback like `-28.73` get reinterpreted as `-28.73 rad`, causing large unintended multi-joint motion.
- [self] Guardrail: never treat the raw comma-separated controller joint command as degree-valued just because API/UI feedback endpoints are degree-based. Explicitly convert back to radians at the API boundary.
- [tool] Fixed by converting `target_arm_deg -> target_arm_rad` inside `/control/joint-jog` before sending the raw command, and extended `tests/test_api_endpoints.py` so the endpoint now asserts the controller command is radian-valued.
- [tool] Another concrete environment fact: in this host venv, `import pyquik` fails with `ModuleNotFoundError`. There is no built/installed `pyquik` extension visible in the repo checkout, only source files under `src/numeric_solver/pyquik/`.
- [self] Important clarification for future sessions: servo backend (`simulation` vs `ethercat_rtcore`) and IK backend (`numeric` vs `ikfast`) are separate. If the user says “sim backend works,” do not assume `pyquik` is installed in the current controller environment; verify the active process/environment before drawing conclusions.

### 2026-03-19 - User correction on solver fallback policy
- [user] Strong correction: for `gradient05`, using the wrong IK solver is not acceptable. Do **not** silently fail over to `ikfast` unless that fallback is explicitly flagged as allowed in the robot config/policy.
- [self] Execution rule going forward: if `gradient05` requests `numeric`, the right fix is to make `pyquik` import/build correctly in the controller environment, not to treat `ikfast` fallback as an acceptable steady state.

### 2026-03-19 - ICS uplink + QuIK install path
- [tool] The `Gradient Uplink` profile was still configured for an old static direct-link setup (`192.168.1.50/24`, `never-default=true`), which is why the Pi had no default route and could not resolve `github.com`.
- [tool] Windows ICS expectation on this host is different: `eth1` should DHCP from the main computer and accept the shared uplink as the default route. After switching the live profile to DHCP, `eth1` obtained `192.168.137.89/24`, gateway `192.168.137.1`, and DNS `192.168.137.1`.
- [tool] Repo source-of-truth updated in `systemd/ethercat-host/port-layout.env` and regenerated `systemd/ethercat-host/gradient-uplink.nmconnection` now renders:
  - `UPLINK_IPV4_METHOD="auto"`
  - `UPLINK_IPV4_NEVER_DEFAULT="false"`
  - `UPLINK_IPV6_METHOD="ignore"`
- [tool] With uplink restored, `git submodule update --init --recursive src/numeric_solver/quik` succeeded and populated the intended dependency repo at `src/numeric_solver/quik`.
- [tool] Required native build tools on this Pi were missing (`cmake`, `Eigen`). Installing `cmake`, `ninja-build`, and `libeigen3-dev` was enough to build `src/numeric_solver/pyquik/pyquik.cpython-311-aarch64-linux-gnu.so`.
- [tool] One local compile fix was needed in the checked-out QuIK tree: `src/numeric_solver/quik/include/quik/Robot.hpp` needed `#include <iostream>` for inline `cout` usage.
- [tool] Verified controller import path after build:
  - `import numeric_solver.pyquik.pyquik` works
  - `from numeric_solver.numeric_wrapper import init_numeric_solver; init_numeric_solver('gradient-05')` works
- [tool] Fresh controller startup after rebuild now reports `IK backend: numeric (source=robot_policy)`, and `/info/runtime-config` shows `effective_backend=numeric` with `restart_required=false`.

### 2026-03-19 - Fresh-AI handoff essentials
- [self] The most important recent corrections are:
  - joint jog bug was a real degrees/radians mismatch and is fixed in `src/gradient_os/api/main.py`
  - `gradient05` must use the robot-policy `numeric` solver, not silent fallback to `ikfast`
  - Windows ICS over `eth1` is now the intended uplink path and is working via DHCP/default route
- [tool] Current live runtime baseline after fixes:
  - controller started manually with `PYTHONUNBUFFERED=1 ./run.sh`
  - API started manually with `PYTHONUNBUFFERED=1 /home/pi/GradientOS/.venv/bin/gradient-api`
  - RTCore on `eth0` is healthy and startup still converges slowly but successfully
  - `/info/runtime-config` now reports `effective_backend=numeric`, `source=robot_policy`, `restart_required=false`
- [tool] Important QuIK nuance: top-level `import pyquik` still fails, but the path actually used by `numeric_wrapper` (`numeric_solver.pyquik.pyquik`) now imports successfully because the built module lives in `src/numeric_solver/pyquik/`.
- [self] Remaining meaningful follow-up is policy hardening, not emergency repair:
  - remove or constrain silent solver fallback behavior so `gradient05` cannot quietly run on the wrong IK backend
  - re-run the smallest per-joint UI commissioning test now that both the jog unit bug and the numeric solver environment are fixed

### 2026-03-19 - Unified startup supervisor
- [user] The manual stack startup should be consolidated into one robust script, with a `--headless` mode that skips the web UI but still stages controller -> API startup.
- [self] Keep `run.sh`, `run-api.sh`, and `run-web.sh` as the component entrypoints; put staging, health checks, log capture, and duplicate-process protection in a separate top-level supervisor (`start-stack.sh`) instead of duplicating launch logic.
- [self] Source `./start.sh` once in the supervisor before launching children so the unified path matches the manual environment bootstrap the user relies on.
- [self] For startup reliability, fail closed on pre-existing live services instead of silently starting duplicates. The first validation of `start-stack.sh` correctly refused startup when Vite was already serving on `127.0.0.1:8000`.
- [tool] Durable startup traces now belong in `logs/startups/<timestamp>/` with per-service raw logs plus a launcher log and manifest; keep live log streaming in the terminal as well so operators still get immediate feedback.

### 2026-03-19 - Startup shutdown + telemetry follow-up
- [user] A stack shutdown is not safe enough if the EtherCAT drives remain energized/ready after the supervisor exits. The launcher must unwind the hardware into a non-active state, not just kill processes.
- [self] For RTCore-backed controller shutdown, `SIGTERM` alone is not sufficient because the controller's Python `finally:` cleanup may not run. Prefer an in-band stop request first, then `SIGINT` so `run_controller.py` executes backend shutdown logic.
- [self] `EthercatRTCoreBackend.shutdown()` must explicitly disable axes and disarm before IPC teardown; simply closing shared memory/socket resources can leave the robot physically enabled.
- [tool] The telemetry-thread crash seen during startup was real: `backend_registry.get_telemetry_blocks()` assumed every backend had Feetech-style telemetry register constants. `ethercat_rtcore` does not, so backend telemetry support must be treated as optional instead of universal.
- [self] After changing shutdown semantics in `start-stack.sh`, do a fresh end-to-end launcher run and stop to verify all three outcomes together:
  - controller exits via graceful path
  - API/web stop as expected
  - physical robot is left de-energized / non-active

### 2026-03-19 - Explicit power-down command path
- [self] Relying on process exit alone is still too fragile for hardware-safe stop. The control stack needs an explicit in-band actuator power-down command that can be invoked before controller teardown.
- [tool] Added `SAFE_POWER_DOWN` through the controller/API path so `start-stack.sh stop` can request de-energize over HTTP first and fall back to direct UDP if the API is already down.
- [self] `./start-stack.sh stop` should attempt safe power-down even when launcher state is stale or absent; safety action should not depend solely on an active supervisor PID file.

### 2026-03-19 - Hardware probe command
- [user] The operator needs a direct way to probe the physical hardware state, not just process state.
- [tool] `./start-stack.sh probe` now reads `/run/gradient-rt-motion/metrics.json` and reports a hardware-focused state summary (`ACTIVE`, `BUS_UP_DISARMED`, `FAULTED`, or `INACTIVE`) plus per-axis DS402/statusword/error/AL-state detail.
- [self] Important live finding from the first probe run: software services can be down while the physical RTCore/drive state is still `ACTIVE` (`armed=1`, `enable_mask=0x3f`, `OperationEnabled` on all 6 axes). Do not assume “controller down” implies “robot safe.”

### 2026-03-19 - Probe-driven shutdown sequencing
- [user] The shutdown procedure should use the probe results and explicitly reason about driver state, EtherCAT master state, and RTCore state before deciding what to stop next.
- [self] A safe full shutdown sequence is: probe -> request power-down through controller/API if available -> re-probe -> direct RTCore disarm if needed -> re-probe -> stop controller/API/web -> stop RTCore -> stop `ethercat.service` -> final probe/report.
- [self] Direct RTCore disarm must not auto-arm while connecting; use the project venv with `GRADIENT_RTCORE_AUTO_ARM=0` for any emergency/disconnected power-down helper.

### 2026-03-19 - Web shutdown must stop the whole process tree
- [tool] Live restart failure reproduced after `./start-stack.sh stop`: the launcher logged `Stopping web (pid=117277)` but `ss -ltnp '( sport = :8000 )'` still showed a different `node`/Vite PID listening on `0.0.0.0:8000`.
- [self] Root cause: the supervisor tracked/killed the wrapper PID, but Vite could survive as a child process and keep the port bound, so the next `./start-stack.sh` falsely looked like a duplicate external web service.
- [tool] Fix in `start-stack.sh`: launch managed services under `setsid` when available and stop them via a helper that signals the full process group plus recursively discovered descendants before escalating.
- [tool] Live validation after the fix:
  - running the updated `./start-stack.sh stop` cleared the orphaned `:8000` listener without a second stop
  - full start -> stop -> restart -> stop cycle succeeded
  - controller shutdown showed `Shutdown signal received: SIGTERM` / `Shutdown requested.` with no SIGKILL warning
- [self] Guardrail: when supervising dev servers or npm-based launchers, do not assume the recorded parent PID is the long-lived listener; validate stop logic against the actual bound port and descendant processes.

### 2026-03-20 - No autonomous joint commands on live robot
- [user] Hard safety rule: do **not** send any joint commands when connected to the live industrial robot. Any motion-capable test must be run by the user.
- [self] Operational boundary: restrict my actions to read-only inspection, passive UI verification, log review, and non-motion code/test analysis unless the user explicitly changes this rule.
- [self] Guardrail: even seemingly small commissioning actions like joint jog, home, rest, or zero-adjacent flows must be treated as user-run-only when they can reach live hardware.

### 2026-03-20 - Joint commissioning click reached controller but produced no motion
- [user] Live report: clicking `+5°` on J1 in the Joint Commissioning panel produced no visible physical motion and no 3D-view robot motion.
- [tool] Attached terminal evidence showed the full request chain did fire: controller received the raw radian target string, then `WAIT_FOR_IDLE`, and the API returned `POST /control/joint-jog HTTP/1.1 200 OK`.
- [self] Important nuance: `WAIT_FOR_IDLE` currently only watches the trajectory thread in `command_api.handle_wait_for_idle()`. The commissioning jog path sends a direct backend setpoint through `servo_driver.set_servo_positions(...)`, so `No move is currently running.` is expected for this path and is **not** a reliable confirmation of RTCore motion completion.
- [tool] Post-run probe showed `physical_state: FAULTED` with `axis0: ds402=Fault err=0x8700`, while the stop snapshot from the same session showed `driver_state=ACTIVE` but only `op_enabled_axes=5/6`.
- [self] Working hypothesis for next session: the UI/API/controller wiring is intact, but J1 commissioning can still be a no-op at the hardware layer when the mapped RTCore axis is faulted/not fully operation-enabled. Also, the 3D view is driven by telemetry (`latest?.joints`), not the commissioning panel's local state, so it will remain static if no joint-state update arrives.

### 2026-03-20 - User-triggered drive fault reset path
- [user] Requirement: when a drive is faulted but resettable, provide a way for the user to reset the drivers; do not have the agent execute the reset autonomously.
- [self] Clarification recorded: the Joint Commissioning jog path does **not** use planner/path-execution threads; it sends a direct RTCore setpoint. Only the follow-up `WAIT_FOR_IDLE` call was trajectory-oriented and therefore misleading.
- [tool] Implemented a manual reset path through:
  - `ActuatorBackend.reset_faults(...)`
  - `EthercatRTCoreBackend.reset_faults(...)` using RTCore `MSG_CMD_FAULT_RESET`
  - controller UDP command `RESET_FAULTS[,joint]`
  - API endpoint `POST /control/reset-faults`
  - a guarded `Reset Faults` button in `web-ui/src/ControlPanel.tsx`
- [self] Guardrail: even though fault reset is non-motion, it still changes live hardware state; I must not trigger it myself on the real robot. Only the user should click/use it.

### 2026-03-20 - Fault visibility must reach the UI
- [user] Faults should be sent to the UI so the operator can see them directly during commissioning.
- [user] Error-code handling is servo-driver-specific and independent of the EtherCAT master/RTCore path; do not hard-code vendor fault maps in generic bring-up logic when backend config can provide them.
- [tool] Added backend-config fault decoding hooks in `backends/registry.py` and `backends/ethercat_rtcore/config.py`, then reused that decoder in both controller telemetry and `start-stack.sh probe`.
- [tool] Added `drive_faults` to the `/monitor` telemetry payload and surfaced it in `web-ui/src/ControlPanel.tsx` as a live commissioning fault/status card with per-axis decoded fault lines when available.
- [self] Guardrail: protocol-state decoding like DS402 state is generic, but vendor/manual error-code naming must come from the active servo backend config or remain raw.

### 2026-03-20 - Soft-stop safety vs latched-fault semantics
- [self] For `./start-stack.sh stop`, do not key success only off `physical_state`. A final probe with `physical_state=FAULTED` can still be a safe soft-stop result when `armed=0`, `axis_enable_mask=0`, and `op_enabled_axes=0`.
- [self] Avoid duplicate `SAFE_POWER_DOWN` timeout noise by keeping stop policy in `perform_shutdown_sequence()` and making `stop_controller_process()` a pure process-stop helper. If another stop path needs retries, define them explicitly in one place.
- [tool] The clean contract that kept Python/UI/probe aligned was: runtime config exposes both `configured_profile` and `live_profile`, while `effective_profile` is `live_profile` when RTCore `STATUS_HELLO` is available and otherwise falls back to configured/default policy.
- [tool] RTCore scaling should come from one shared helper that renders systemd startup args from the robot config (`counts_per_rev`, `gear_ratio`, `sign`) instead of trusting placeholder RTCore defaults or recomputing different rules in multiple places.
- [self] Safe validation boundary remains unchanged: build/test/syntax/probe analysis are fine, but do not run live motion-capable stop/jog/reset actions on the industrial robot unless the user explicitly asks.

### 2026-03-20 - RTCore mismatch was installed-unit drift
- [tool] On this host, the repo service file had already been updated for env-driven RTCore scaling, but `/etc/systemd/system/gradient-rt-motion.service` was still the old `ExecStart=/usr/local/bin/gradient-rt-motion` unit and `/etc/default/gradient-rt-motion` was missing. The runtime mismatch warning therefore persisted even though the repo code looked fixed.
- [self] Guardrail: when a warning involves systemd-managed binaries/config, inspect the installed unit files under `/etc` instead of assuming the repo copy is what the machine is actually running.
- [tool] Durable fix: put the RTCore unit/env reconciliation into the startup path (`start-stack.sh` calling `systemd/rt-motion/sync-runtime.sh --ensure-active`) so stale installed files cannot silently survive across sessions.

### 2026-03-20 - Sync unit and binary together
- [self] New regression I introduced: syncing the installed RTCore unit/env without also updating `/usr/local/bin/gradient-rt-motion` broke startup completely on this host because the installed binary was older and did not support `--drive-profile`.
- [tool] Concrete symptom from `journalctl -u gradient-rt-motion.service`: `ERROR: unknown arg: --drive-profile`, followed by missing `/run/gradient-rt-motion/ipc.sock`, controller `metrics unavailable`, and launcher startup timeout.
- [self] Guardrail: for systemd-managed native binaries, treat unit file, env file, and installed binary as one atomic deployment set. Never roll forward the unit args alone.

### 2026-03-20 - Large runtime-config replies can exceed 1024 bytes
- [tool] After adding `configured_profile`/`live_profile` fields and richer tool/runtime metadata, the controller's `GET_RUNTIME_CONFIG` UDP reply became large enough to exceed the API's old `recvfrom(1024)` buffer.
- [self] Concrete failure mode: `/info/runtime-config` returned `active_error=Runtime-config decode failure: Unterminated string...`, `wait_for_api_readiness()` treated that as fatal, and `start-stack.sh` then soft-stopped the stack during cleanup even though controller/API themselves had started.
- [self] Guardrail: when a UDP command carries JSON payloads, size the receive buffer for realistic payload growth (`65535` here), not legacy tiny status strings.

### 2026-03-20 - Post-stop probe should fall back to desired runtime context
- [user] Asked to clean up the misleading post-stop probe and specifically asked whether J3 driver code `ErC2.0` was being picked up.
- [self] Mistake in the earlier probe path: once controller/API were down, I let probe decoding depend on live runtime-config only, which made `servo_backend=None` and mislabeled the fieldbus as `DOWN` even while RTCore metrics still showed `link_up=1`, `operational=6/6`, and `wkc=18/12`.
- [tool] Fix in `start-stack.sh`: `probe_hardware_state_json()` now falls back to desired runtime config when live runtime-config is unavailable, passes axis-to-joint mapping into `build_drive_fault_snapshot(...)`, and prints `probe_decode: ... source=desired_runtime_fallback` so the operator can see why decoding still works after shutdown.
- [tool] Validation: read-only `./start-stack.sh probe` now reports `ethercat_master_state: OP`, labels the faulted drive as `J3/axis2`, and keeps RTCore/EtherCAT reporting consistent after controller/API stop.
- [tool] Additional fix in `src/gradient_os/arm_controller/profiles/drive/a6ec_ds402.py`: corrected the repo-root path used for the A6-EC manual codebook so the decoder stops silently missing `docs/resources/a6ec_manual_codes.json`.
- [self] Important nuance: A6-EC bus fault `0x8700` maps to a sync-fault family in the manual, not a single unique front-panel alias. The probe can confirm the J3 drive is on the `0x8700` sync-fault family, but exact front-panel `0x203F` aliases like `ErC2.0` require care when presenting them.
- [self] Guardrail: when probe/UI decoding depends on runtime metadata that may disappear during shutdown, add an explicit fallback source label rather than silently pretending the bus/backend is unavailable.

### 2026-03-20 - Remote fault reset should be per-joint, not only global
- [user] Requested a practical remote way to reset servo drivers because many commissioning faults clear with a simple driver reset.
- [self] Important context rediscovered from prior work: the backend/controller/API remote reset path already existed, so the missing piece was operator discoverability and per-joint targeting in the UI rather than backend protocol support.
- [tool] Improved `web-ui/src/ControlPanel.tsx` so the drive-fault panel:
  - shows all current faulted axes instead of truncating after four
  - exposes `Reset Jn` buttons for resettable faults with a mapped logical joint
  - renames the old broad action to `Reset All Faults` for clarity
- [self] Guardrail: when the system already has a safe backend capability, prefer surfacing it clearly in the operator workflow instead of building a second parallel reset mechanism.

### 2026-03-20 - Targeted RTCore fault reset works while stack is down
- [user] Explicitly asked me to run the remote reset and verify the result on live hardware.
- [tool] Baseline before reset from `./start-stack.sh probe`: stack down, RTCore up, EtherCAT `OP`, `physical_state=FAULTED`, and only `J3/axis2` faulted with `sw=0x1618 err=0x8700`.
- [tool] Direct RTCore recovery path works without controller/API: `./.venv/bin/python ./scripts/rtcore_jog.py fault_reset --mask 0x4` cleared the J3 fault while the robot remained disarmed.
- [tool] Verification after reset:
  - `./.venv/bin/python ./scripts/rtcore_jog.py status --timeout 0.8` showed `axis2 sw=0x1650 err=0x0000`
  - `./start-stack.sh probe` showed all six axes `SwitchOnDisabled` with `err=0x0000` and `physical_state=BUS_UP_DISARMED`
- [self] Guardrail: when the main stack is down but RTCore remains up, a targeted `fault_reset --mask` pulse is the least invasive live recovery check compared with restarting the whole stack.

### 2026-03-20 - Startup should clear disarmed drive faults before controller auto-arm
- [user] New requirement: during startup, check for faulted servos at the safe point and reset them before any power-up sequence begins.
- [self] Important architectural finding: the controller-side `EthercatRTCoreBackend` still auto-arms on connect by default, so the safest insertion point for startup fault reset is **before** launching the controller, not after controller readiness.
- [tool] Implemented a startup preflight in `start-stack.sh`:
  - waits for RTCore/EtherCAT bus readiness while the stack is still down
  - builds a tested `build_startup_fault_reset_plan(...)` from the probe payload
  - auto-resets only faulted axes that are present while the robot is still disarmed
  - aborts startup if a fault remains latched after the reset timeout, rather than auto-arming into a bad state
- [tool] Added Python tests in `tests/test_drive_faults.py` for:
  - disarmed faulted axis -> auto-reset plan
  - still-active faulted axis -> block startup
  - clean bus-up disarmed state -> no action
- [self] Guardrail: when startup behavior touches live power-state transitions, prefer a fail-closed preflight that runs before controller auto-arm over “fix it after init” logic inside the already-arming backend path.

### 2026-03-20 - Safe default is disarmed, with explicit user power-up
- [user] Safety preference: the system should not auto-arm on startup; arming should be an explicit user action.
- [self] Correction to prior approach: a startup fault-reset preflight alone was not sufficient, because the backend still auto-armed immediately after connect. Safe startup requires changing the default power policy, not just inserting more checks before it.
- [tool] Implemented the new default:
  - `EthercatRTCoreBackend` now defaults `GRADIENT_RTCORE_AUTO_ARM` to `0`
  - `start-stack.sh` explicitly launches the controller with `GRADIENT_RTCORE_AUTO_ARM=0`
  - `run.sh` also exports `GRADIENT_RTCORE_AUTO_ARM=0` unless something explicitly overrides it
- [tool] Added explicit user-triggered power-up path:
  - backend `safe_power_up()`
  - controller UDP command `SAFE_POWER_UP`
  - API endpoint `POST /control/power-up`
  - UI buttons `Power Up Drives` / `Power Down Drives`
- [self] Guardrail: when removing an unsafe default like auto-arm, always add a clear explicit replacement action in the same pass so operators are not forced into hidden or ad-hoc recovery workflows.

### 2026-03-20 - Startup preflight helper must suppress import-time stdout
- [tool] Regression found during live `./start-stack.sh`: the new `probe_startup_fault_reset_plan()` helper imported `gradient_os.telemetry.drive_faults` without redirecting stdout, and import-time backend registration logs polluted the helper's JSON output.
- [self] Concrete failure mode: launcher got past `bus ready`, then crashed with `json.decoder.JSONDecodeError: Expecting value` while parsing the plan payload because the string started with log text instead of JSON.
- [tool] Fix in `start-stack.sh`: wrap the `build_startup_fault_reset_plan` import in `redirect_stdout(io.StringIO())`, matching the earlier probe helper hardening.
- [tool] Live validation after the fix:
  - `./start-stack.sh` completed startup
  - `./start-stack.sh probe` showed controller/API up, EtherCAT `OP`, and `driver_state: DISARMED` / `physical_state: BUS_UP_DISARMED`
- [self] Guardrail: any inline Python helper in shell that emits machine-readable JSON must silence import-time stdout from project modules, especially registry/bootstrap code that prints on import.

### 2026-03-20 - Critical power controls should not depend only on telemetry timing
- [user] Reported that `Power Up Drives` was not clickable and that power controls belong at the top in their own section, not under commissioning.
- [self] Likely cause confirmed in UI code: the power buttons were enabled/disabled solely from `driveFaults?.servo_backend`, so they could render disabled while monitor telemetry lagged even though runtime-config already knew the active backend was `ethercat_rtcore`.
- [tool] UI fix in `web-ui/src/ControlPanel.tsx` and `web-ui/src/App.tsx`:
  - added `activeServoBackend` prop from runtime-config as a fallback source for enabling explicit drive-power controls
  - moved `Power Up Drives`, `Power Down Drives`, and `Reset All Faults` into a dedicated top-level `Drive Power` section
  - moved drive fault details/reset actions into that same top section
  - left joint commissioning focused on refresh/jog/zero only
- [self] Guardrail: high-priority safety controls should be grouped in their own dedicated section and should not become unavailable just because a secondary telemetry panel has not populated yet.

### 2026-03-20 - Keep drive power state sticky across sparse SSE packets
- [user] Live report after power-up: brakes/drives armed fine, but `Power Down Drives` stayed unavailable in the UI.
- [tool] Live verification proved the backend state was correct: `./start-stack.sh probe` showed `driver_state: ACTIVE`, `armed=1`, and `op_enabled_axes=6/6`.
- [self] Root cause in frontend telemetry handling: `/monitor` only includes `drive_faults` on some packets, but `App.tsx` was overwriting `latest.drive_faults` with `null` on every packet that omitted the field. The power section then fell back to “not active” even while hardware stayed armed.
- [tool] Fix in `web-ui/src/App.tsx`: preserve the last known `drive_faults` snapshot until a newer packet explicitly replaces it.
- [self] Guardrail: for sparse/optional telemetry sub-blocks, merge with previous UI state instead of treating omission as an authoritative null.

### 2026-03-20 - Calibration feedback and 3D pose must share a fallback path
- [user] Next task is individual joint calibration/zero testing, with the requirement that the 3D visualizer reflect the robot pose during commissioning.
- [self] Important split in the current UI: `ControlPanel.tsx` refreshes `/info/joints` in degrees for zero/jog feedback, while `ArmVisualizer` follows `latest.joints` from `/monitor` SSE in radians. If SSE lags, the calibration numbers can update before the 3D arm does.
- [tool] Updated `web-ui/src/ControlPanel.tsx` and `web-ui/src/App.tsx` so successful `/info/joints` refreshes report back into App as a fallback joint source, converted to radians and applied only when the SSE stream has been quiet for more than 250 ms.
- [self] Guardrail: when a commissioning panel reads live robot state through a side channel, either unify it with the main visualization state or explicitly use it as a stale-telemetry fallback so operator numbers and 3D pose cannot drift apart.

### 2026-03-20 - Live digital twin should not smooth away real pose
- [user] Explicit preference: the 3D visualization should be the perfect digital twin of the robot's real position.
- [self] Root cause in `web-ui/src/ArmVisualizer.tsx`: even after receiving fresh live joints, the render loop still interpolated toward `targetAnglesRef` using `smoothing = 12`, which can make the model visibly chase the robot instead of matching the sample exactly.
- [tool] Fix in `web-ui/src/ArmVisualizer.tsx`: when live `joints` props change, set both `targetAnglesRef` and `currentAnglesRef` to the exact telemetry snapshot and apply those joint values immediately to the URDF hierarchy.
- [self] Guardrail: for a live robot pose view, do not smooth or animate away the primary telemetry sample; save interpolation for previews/ghosts, not the main digital twin.

### 2026-03-20 - Commissioning jog failures need explicit backend reasons
- [user] Reported a live joint jog failure; terminal showed `APPLY_JOINT_SETPOINT` reached the controller, but the API still returned `503 Service Unavailable`.
- [self] Diagnostic conclusion: that pattern means the problem is not basic controller reachability. It is either a controller-side `ERROR,APPLY_JOINT_SETPOINT,...` reply or a missing ACK on the direct setpoint path.
- [tool] Improvements added:
  - `src/gradient_os/run_controller.py` now logs the actual exception and traceback when `APPLY_JOINT_SETPOINT` is rejected
  - `src/gradient_os/api/main.py` now converts `ERROR,APPLY_JOINT_SETPOINT,...` into a structured HTTP error with a concrete message
  - `web-ui/src/ControlPanel.tsx` now parses JSON error payloads and shows the real jog failure reason in the commissioning status instead of only saying "connectivity"
- [self] Guardrail: when live motion commands can fail for stateful backend reasons, never hide them behind generic transport wording; surface the backend rejection text all the way to the operator and terminal.

### 2026-03-20 - RTCore commissioning path must not depend on legacy servo accel defaults
- [user] New terminal traceback exposed the real cause of jog failure: `handle_apply_joint_setpoint()` tried `float(utils.DEFAULT_SERVO_ACCELERATION_DEG_S2)` while that value was `None`.
- [self] Root cause: the EtherCAT/RTCore backend intentionally clears legacy serial-servo constants in `utils`, so direct commissioning code cannot assume those defaults are always populated.
- [tool] Fix in `src/gradient_os/arm_controller/command_api.py`:
  - added coercion helpers for direct setpoint speed/acceleration
  - missing/invalid defaults now fall back to finite commissioning-safe values instead of crashing
  - added `tests/test_command_api_direct_setpoint.py` to lock in the `None`-defaults case
- [self] Guardrail: when code runs on both serial-servo and RTCore backends, never read legacy servo defaults without a finite fallback or backend-aware resolution path.

### 2026-03-20 - Trajectory executors must stay backend-native under RTCore
- [user] Cartesian jog from the UI hit a live closed-loop executor traceback while following a planned move.
- [self] Root cause in `src/gradient_os/arm_controller/trajectory_execution.py`: backend mode was still running serial-servo executor logic, decoding RTCore axis counts through `servo_driver.servo_value_to_radians(...)` and preparing legacy raw servo sync-write tuples even though RTCore intentionally leaves serial constants like `utils.ENCODER_RESOLUTION` unset.
- [tool] Fix in `src/gradient_os/arm_controller/trajectory_execution.py`:
  - `closed_loop` backend path now reads raw counts with `backend.sync_read_positions()`, converts them with `backend.raw_to_joint_positions()`, records joint-space telemetry/errors directly, and writes commands via `backend.prepare_sync_write_commands(...)`
  - `open_loop` backend path now also precomputes commands with `backend.prepare_sync_write_commands(...)` and uses `backend.get_joint_positions()` for diagnostics instead of serial helpers
  - added `tests/test_trajectory_execution_backends.py` to lock in backend execution with `ENCODER_RESOLUTION=None`
- [self] Guardrail: when an `ActuatorBackend` is active, trajectory executors should only use backend conversion/write hooks; bypassing them with servo-specific math will eventually break on RTCore or any backend that does not emulate serial-servo constants.

### 2026-03-20 - Realtime arm jog is backend-safe, but single-actuator backend writes must avoid legacy raw conversion
- [user] Asked whether realtime jog commands are compatible with the RTCore/EtherCAT backend.
- [self] Audit result: the main 6-axis realtime jog loop in `src/gradient_os/arm_controller/command_api.py` is compatible because it reads with `servo_driver.get_current_arm_state_rad(...)` and writes with `servo_driver.set_servo_positions(...)`, both of which delegate to the active backend in joint space.
- [self] Remaining mismatch found nearby: `src/gradient_os/arm_controller/servo_driver.py:set_single_servo_position_rads(...)` still used legacy raw-servo conversion and `backend.sync_write(...)` in backend mode, which could fail when RTCore leaves `utils.ENCODER_RESOLUTION` unset and does not accept raw servo tuples.
- [tool] Fix:
  - `set_single_servo_position_rads(...)` now calls `backend.set_single_actuator_position(...)` directly in backend mode
  - added `tests/test_realtime_jog_backend_compatibility.py` to cover:
    - backend single-actuator writes with `ENCODER_RESOLUTION=None`
    - the realtime jog loop issuing joint-space `servo_driver.set_servo_positions(...)` commands without touching legacy encoder constants
- [self] Guardrail: if a helper is meant to work across backends, never synthesize legacy raw servo tuples in backend mode when the interface already has a dedicated actuator-level API.

### 2026-03-20 - Live visualizer lag was dominated by bounds refreshes, not just smoothing
- [user] Observed that the robot animation still felt laggy and suggested restoring the older smoothing behavior.
- [self] Inspection result in `web-ui/src/ArmVisualizer.tsx`: the live pose already snapped directly to incoming telemetry, but the component still triggered dynamic bounds/grounding refreshes during live joint updates. That expensive per-update work can make the viewer feel behind even when the joint angles themselves are applied immediately.
- [tool] Fix in `web-ui/src/ArmVisualizer.tsx`:
  - removed leftover per-frame joint interpolation logic from the render loop
  - stopped forcing dynamic bounds recomputation on every live telemetry update
  - now throttles live bounds refresh to visible-bounds mode only, at most every 250 ms
- [self] Guardrail: when the main goal is a live digital twin, do not reintroduce smoothing to mask performance problems; first remove expensive nonessential work from the live telemetry hot path.

### 2026-03-20 - Live pose tuning may need coordinated timing across poll, fallback, and bounds refresh
- [user] Reported that the visualizer still felt laggy and explicitly asked to try 50 Hz because 250 ms gating felt too slow.
- [self] Important system-level lesson: there were multiple timing gates, not just one:
  - `ControlPanel.tsx` background `/info/joints` poll was still `500 ms`
  - `App.tsx` fallback from `/info/joints` into main visualizer state required the SSE stream to be quiet for `250 ms`
  - `ArmVisualizer.tsx` live bounds refresh was throttled to `250 ms`
- [tool] Tuning pass:
  - `ControlPanel.tsx` background joint poll now runs at `20 ms` and reuses `refreshJointAngles(true)` so it also feeds `onJointFeedback`
  - `App.tsx` live fallback delay now uses `1 / 50` seconds
  - `ArmVisualizer.tsx` live bounds refresh interval now uses `20 ms`
- [self] Guardrail: when tuning live pose latency, adjust the whole pipeline consistently; a fast render loop cannot fix a slower upstream poll/fallback gate.

### 2026-03-20 - Commissioning step UI should support custom magnitude without losing quick presets
- [user] Asked for a custom joint-step value alongside the existing `0.25 / 1 / 5` commissioning presets and also asked what the 17-bit encoder resolution implies.
- [tool] Updated `web-ui/src/ControlPanel.tsx`:
  - added a `Custom` numeric input and `Use` button next to the preset step chips
  - custom entries accept signed or unsigned degrees, normalize to absolute magnitude, and reuse the existing `- / +` jog buttons for direction
  - step labels/status text now use trimmed dynamic formatting instead of assuming preset values
- [self] Guardrail: for operator step controls, preserve the fast preset path but allow precise custom magnitudes without replacing the existing muscle-memory UI.

### 2026-03-21 - Lag experiments need upstream telemetry and caller defaults changed together
- [user] Explicitly asked for implementation, not just explanation, after identifying `/monitor` telemetry cadence and closed-loop defaults as the highest-value remaining lag suspects.
- [tool] Updated `src/gradient_os/api/main.py` so `TelemetryHub` now reads `GRADIENT_MONITOR_TELEMETRY_HZ` and defaults `/monitor` startup to `50 Hz` instead of a hard-coded `10 Hz`.
- [tool] Flipped default Cartesian move-line execution to open-loop in `src/gradient_os/api/main.py`, `src/gradient_os/arm_controller/command_api.py`, `web-ui/src/ControlPanel.tsx`, `src/gradient_os/ui/pages/real_control_page.py`, and `src/gradient_os/ui/commands.py`.
- [self] Guardrail: when A/B testing motion responsiveness, update explicit UI/API payload defaults as well as backend fallbacks; otherwise old `closed: true` callers can hide the effect of a backend-only default change.
- [self] Guardrail: prefer env-configurable telemetry cadence over swapping one magic number for another, so live tuning can continue without another code edit.
- [self] Follow-up risk: the RT path is still latest-wins from Python into RTCore, so faster monitor telemetry and open-loop defaults do not yet provide queued RT-side trajectory scheduling.

### 2026-03-21 - Orientation controls can desync when step and jog paths use different rotation semantics
- [user] Reported that roll/pitch/yaw controls still felt unsynchronized even after Cartesian motion improved, and narrowed it to step rotate plus realtime angular jog rather than Cartesian moves.
- [self] Root cause: `src/gradient_os/api/main.py` had `/control/rotate` doing `GET_POSITION` plus absolute Euler read-modify-write into `SET_ORIENTATION`, which is a different path from the controller's existing relative `ROTATE` command and is vulnerable to Euler coupling/stale pose reconstruction.
- [tool] Updated `src/gradient_os/api/main.py` so `/control/rotate` now forwards `ROTATE,<axis>,<angle>` directly to the controller.
- [self] Root cause: `src/gradient_os/arm_controller/command_api.py` jog loop was integrating `v_roll/v_pitch/v_yaw` as a raw rotation vector, which does not match the UI's labeled roll/pitch/yaw step semantics when combining angular axes.
- [tool] Updated realtime jog orientation integration to use `R.from_euler('xyz', angular_deg_s * dt, degrees=True)` and keep the same pre-multiply base-frame application as `handle_rotate_command()`.
- [self] Guardrail: if operator controls are labeled `roll/pitch/yaw`, step rotate and realtime jog must share the same rotation convention; avoid mixing Euler read-modify-write in one path with rotvec integration in another.

### 2026-03-21 - Orientation step moves should use live-state profiled execution, not cached one-shot IK
- [self] Deeper root cause after the first orientation fix: `handle_rotate_command()` still used cached `utils.current_logical_joint_angles_rad` plus a single direct `servo_driver.set_servo_positions(...)`, while the Cartesian path that felt good started from `get_current_arm_state_rad()` and ran through a profiled executor.
- [tool] Refactored `src/gradient_os/arm_controller/command_api.py`:
  - added `_get_live_pose_snapshot()` so orientation moves start from physical joint feedback rather than cached commanded state
  - added `_execute_orientation_path()` so orientation-only moves reuse SLERP + batched IK + executor-thread machinery
  - changed `handle_rotate_command()` to execute a short smooth open-loop orientation path instead of a one-shot direct servo command
  - changed `handle_set_orientation_command()` to use the same live-state helper path
  - changed realtime jog to refresh `q_current` from `get_current_arm_state_rad(verbose=False)` every loop before FK/IK
- [self] Guardrail: if Cartesian motion is stable because it is planned from live hardware state, orientation controls should not keep a separate legacy cached-state path.

### 2026-03-21 - RTCore max-rpm is currently a global runtime clamp, not a commissioning-only hint
- [user] Reported motion feeling worse and asked whether all moves are limited by the `100 RPM` cap and how the EtherCAT master is actually fed.
- [tool] Verified live unit/runtime state:
  - `/etc/systemd/system/gradient-rt-motion.service` does not pass `--max-rpm`
  - `/etc/default/gradient-rt-motion` contains scaling/profile env only, no max-rpm override
  - active process args also omit `--max-rpm`
- [self] Therefore the running RTCore instance is using the binary default `max_rpm = 100.0` from `src/gradient_rt_motion/main.cpp`, which means all RTCore-driven setpoints are currently clamp-limited unless the service is launched differently.
- [self] Important architecture reminder: Python motion code calls `servo_driver.set_servo_positions(...)` -> EtherCAT RTCore backend `set_joint_positions(...)` -> shared-memory setpoint slot + eventfd wake -> RTCore helper converts radians to target counts -> 1 kHz cyclic EtherCAT loop rate-limits per-axis motion via `max_step_counts_per_cycle` and writes `0x607A` target position plus `0x607F` max profile velocity.
- [self] Guardrail: tuning Python-side planners/executors will not bypass a live RTCore `--max-rpm` cap; if normal production motion should exceed commissioning limits, the runtime/service layer must expose a separate max-rpm setting.

### 2026-03-21 - Safe home/rest should use bounded joint trajectories, not raw joint setpoints
- [user] Wanted `home` and `rest` specifically clamped to a gentle `100 RPM` equivalent while normal motion remains governed by the higher-level runtime RTCore limit.
- [self] Important distinction: `home` / `rest` in `src/gradient_os/api/main.py` were raw joint-target shortcuts, so the Cartesian speed multiplier and profile velocity/acceleration path did not affect them at all.
- [tool] Implemented bounded home/rest motion by extending `APPLY_JOINT_SETPOINT` with optional `max_motor_rpm` and making `handle_apply_joint_setpoint()` build a smooth joint-space path from live feedback when that override is present.
- [tool] The bounded path computes per-joint max rad/s from the active robot gear ratios and requested motor RPM, then executes a smoothstep-interpolated open-loop joint path at `100 Hz`.
- [tool] Updated `/control/home` and `/control/rest` to use `APPLY_JOINT_SETPOINT` with `max_motor_rpm=100.0`, so those buttons stay conservative without forcing the global RTCore runtime clamp back down.
- [self] Guardrail: when a UI control must be safe but RTCore lacks a live per-command clamp knob, prefer a bounded planned path over a raw direct setpoint.

### 2026-03-21 - 6000 RPM exposed coarse Python-side motion paths and overlapping joint polling
- [user] Reported that with live RTCore `--max-rpm 6000`, realtime moves became super jerky, Cartesian step moves stayed smooth but slow, and RPY moves became insanely fast.
- [tool] Terminal `1.txt` showed RTCore truly running with `--max-rpm 6000` and also showed heavy `/info/joints` traffic; the terminal contained `489` `GET_JOINT_ANGLES` entries and repeated API requests from multiple client ports.
- [self] Root cause 1: `web-ui/src/ControlPanel.tsx` polled `/info/joints` every `20 ms` with no in-flight guard, so requests could overlap and hammer the controller, stealing Python-side motion timing.
- [self] Root cause 2: realtime jog in `src/gradient_os/arm_controller/command_api.py` was still only `25 Hz`, so once the old RTCore 100 RPM smoothing ceiling was removed, the coarse jog setpoint cadence became visible as jerk.
- [self] Root cause 3: incremental RPY step moves ignored the speed multiplier and used a short fixed duration heuristic in `handle_rotate_command()`, so they became much faster than Cartesian step moves once the global clamp was high.
- [tool] Fixes:
  - added an in-flight guard and visibility check to the `/info/joints` poll loop in `web-ui/src/ControlPanel.tsx`
  - increased `JOG_CONTROL_FREQUENCY_HZ` from `25` to `50`
  - extended `/control/rotate` / `ROTATE` to accept `duration_s`
  - made web RPY step buttons send `duration_s = abs(angle_deg) / effectiveAngularDegS` so step rotation speed follows the UI's angular rate control
- [self] Guardrail: once RTCore clamp is raised, any coarse Python cadence or overlapping polling that was previously masked by the clamp will become immediately visible in motion quality.

### 2026-03-21 - RTCore contract-first motion redesign
- [user] For RTCore motion redesign, do not stop at architecture commentary; implement the contract in code and explain the result.
- [self] Safe first slice for a boundary shift this large: land the docs + shared IPC ABI + RTCore-published execution-state snapshot before attempting buffered replay or planner migration.
- [tool] Focused validation should avoid unrelated baseline failures in broad hardware/backend suites; in this pass `tests/test_gradient05_limits_and_backends.py` still has pre-existing J5 ratio/sign expectation failures, so I validated the new contract with targeted tests plus a native `cmake --build build` of `src/gradient_rt_motion`.
- [self] Guardrail: when introducing new RTCore status semantics, mirror the numeric enums in shared Python helpers immediately so controller/API/UI code can consume names instead of duplicating magic numbers.

### 2026-03-21 - Queue-only RTCore motion path
- [user] New hard requirement: do not leave any RTCore motion fallback to the old latest-wins slot path.
- [self] Practical migration rule: if legacy high-level APIs still exist (`set_joint_positions`, `sync_write`, stop/brake helpers), re-implement them as thin wrappers over queued trajectory messages instead of keeping the slot alive.
- [tool] Important hidden dependency: `scripts/rtcore_jog.py` also assumed the setpoint slot. When disabling `setpoint_offset` in RTCore, update operator tooling in the same pass or it silently becomes the last remaining old-path dependency.
- [self] Guardrail: when removing an old IPC path, search the whole repo for struct fields like `setpoint_offset` and helper names like `_write_setpoint` so debug tools and tests do not quietly reintroduce the deprecated contract.

### 2026-03-21 - RTCore completion semantics need a query path, not just richer one-shot ACKs
- [user] Asked to continue the RTCore redesign by wiring execution/completion state through `run_controller`, `api/main.py`, and the UI, with RTCore as the source of truth.
- [self] Implemented a concrete contract: controller motion commands can now return structured base64 JSON payloads over UDP, and `GET_MOTION_STATUS` provides a dedicated query path for queued/executing/completed state.
- [tool] Proven pattern: keep the controller reply transport comma-safe by base64-encoding compact JSON payloads (`ACK,<COMMAND>,<payload_b64>` / `MOTION_STATUS,<payload_b64>`) instead of inventing another ad-hoc CSV schema.
- [self] Guardrail: for RTCore-backed motion, do not reuse old names like `direct_setpoint_ack` or `trajectory_thread_ack`; return explicit `state`, `completion_scope`, and `trajectory_id` so API/UI code cannot confuse controller acceptance with physical completion.
- [self] Guardrail: when the UI needs motion lifecycle state, poll or subscribe to the explicit motion-status endpoint and disable controls from that state; do not infer completion from `/info/joints` refresh success.

### 2026-03-21 - Keep the plan file's "current state" sections fresh or it becomes actively misleading
- [user] Asked for the plan to be updated with the real current state and all remaining no-legacy cleanup work before handing off to a fresh chat.
- [self] Important maintenance rule: after large redesign slices, update the plan's top-level `overview`, todo statuses, and "Validated Current State" block together; leaving an old pre-redesign context section in place causes the next agent to optimize for already-removed constraints.
- [tool] Added an explicit cross-cutting legacy-retirement track to the plan so remaining stale assumptions are visible as first-class work instead of getting lost between Phase 4 / 6 / 8.
- [self] Guardrail: when the user says "no old RTCore pathways left," audit not just executable code but also docs, tests, and fire-and-forget call sites that still normalize old semantics.

### 2026-03-21 - Structured motion ACKs require handler-level results, not just API transport changes
- [user] Continued the RTCore redesign by prioritizing structured execution metadata for `ROTATE` / `SET_ORIENTATION` plus `App.tsx` trajectory/program UI status wiring.
- [self] Root cause: orientation endpoints could not safely switch to `expect_response=True` while `src/gradient_os/arm_controller/command_api.py` still returned `None` and hid executor failures behind a helper thread join.
- [tool] Updated `src/gradient_os/arm_controller/command_api.py`, `src/gradient_os/run_controller.py`, and `src/gradient_os/api/main.py` so blocking orientation commands now return structured payloads, propagate planner/executor errors, and no longer rely on fire-and-forget API semantics.
- [tool] Updated `web-ui/src/App.tsx` to poll `/control/motion-status`, persist latest motion metadata, and show execution state in trajectory/weld drawers beyond local request booleans.
- [tool] Updated `tests/test_api_endpoints.py` and `docs/README.md` to lock the new rotate/set-orientation/preview behavior and retire a stale web-rotate description.
- [self] Guardrail: when a blocking motion path is migrated onto the RTCore completion contract, make the command handler itself return metadata or raise; changing only the UDP/API `expect_response` flag produces fake ACKs and hides execution failures.
- [self] Guardrail: for program/preview UI, separate "submit in progress" from "motion still active"; the former is a local fetch state, the latter must come from `/control/motion-status` or a structured command payload.

### 2026-03-21 - RTCore queued timing should snap to RT-cycle multiples, not approximate them
- [user] Called out that queued motion planning frequency should fit neatly inside the RTCore schedule, effectively behaving like a factor/divisor relationship with the RTCore loop instead of using arbitrary off-cycle timing.
- [self] Root cause: `src/gradient_os/arm_controller/backends/ethercat_rtcore/backend.py` previously derived `t_from_start_ns` via `round(1e9 / frequency)`, which can produce sample periods that are not integer multiples of the RTCore cycle and therefore do not land cleanly on RT boundaries.
- [tool] Updated the EtherCAT RTCore backend so queued trajectory timing is quantized onto integer multiples of `cycle_ns`; effective queued frequency can never exceed the requested planner frequency or the RTCore loop rate.
- [tool] Exposed the last queued-trajectory timing metadata back through `src/gradient_os/arm_controller/command_api.py` motion-status payloads for observability.
- [tool] Updated `docs/rtcore_owned_motion_contract.md` and the RTCore redesign plan to spell out `ACK` as `acknowledgement` and describe the RTCore-aligned timing rule explicitly.
- [self] Guardrail: when Python specifies nominal timing for RTCore-owned motion, convert it into an RTCore-cycle-aligned step size first; never stamp buffered points with fractional/off-cycle timing just because the requested frequency is awkward.

### 2026-03-21 - For EtherCAT scheduled motion, compatibility closed-loop flags should not pull timing back into Python
- [user] Re-emphasized that the whole point of the redesign is to move as much motion execution responsibility into RTCore as makes sense, with industrial-robot robustness as the target.
- [self] Decision taken: for scheduled EtherCAT RTCore motion, older `closed_loop=true` request flags should not reactivate the Python-timed closed-loop executor just because that compatibility knob still exists in higher layers.
- [tool] Updated `src/gradient_os/arm_controller/command_api.py` so scheduled `MOVE_LINE`, `MOVE_LINE_RELATIVE`, and `SET_ORIENTATION` requests are coerced onto the RTCore queued path when RTCore execution is available, while still reporting both requested and effective execution policy in metadata.
- [tool] Updated `/home/pi/.cursor/plans/rtcore_motion_redesign_a2ff8300.plan.md`, `docs/rtcore_owned_motion_contract.md`, `docs/command_api.md`, and `docs/README.md` so the code/docs agree on that policy.
- [self] Guardrail: if a compatibility flag would move scheduled EtherCAT motion timing back out of RTCore and into Python, prefer coercing or rejecting it explicitly; do not quietly preserve the old Python timing path in the name of backward compatibility.

### 2026-03-21 - Preserve non-RT backends while clarifying RTCore-backed program semantics
- [user] Asked where the RTCore redesign stands, requested a todo refresh, and explicitly warned not to close off controls for non-EtherCAT backends like simulation or Feetech.
- [self] Important scope rule: RTCore-preference policies must be conditional on actual RTCore capability; simulation and Feetech should keep their controller-owned execution semantics unless and until they gain an equivalent RT execution path.
- [tool] Updated `src/gradient_os/arm_controller/command_api.py` and `src/gradient_os/arm_controller/trajectory_execution.py` so `RUN_TRAJECTORY` reports controller-program-thread acceptance plus per-program metadata about RTCore-backed motion segments, active step, and loop iteration.
- [tool] Updated `/home/pi/.cursor/plans/rtcore_motion_redesign_a2ff8300.plan.md` to add explicit completed todos for RTCore timing quantization and scheduled-motion ownership, plus a new in-progress todo for program-status semantics.
- [tool] Added tests in `tests/test_command_api_direct_setpoint.py` proving non-RT backends still preserve `closed_loop` execution for `MOVE_LINE` and `SET_ORIENTATION`, and extended `tests/test_api_endpoints.py` to lock the richer `RUN_TRAJECTORY` acknowledgement payload.
- [self] Guardrail: when a motion path is "mixed" (controller-managed program sequencing plus RTCore-owned scheduled segments), expose both layers in metadata instead of collapsing everything into a single fake acknowledgement or pretending all backends behave like EtherCAT RTCore.

### 2026-03-21 - `WAIT_FOR_IDLE` should poll composite execution truth, not one implementation detail
- [user] Asked to do the next RTCore redesign item immediately after the program-status slice, which was the `WAIT_FOR_IDLE` semantics cleanup.
- [self] Corrective rule: `WAIT_FOR_IDLE` must not be modeled as "join one thread" because scheduled motion can be controller-managed, RTCore-owned, or mixed; use `get_motion_execution_status()` as the single wait predicate instead.
- [tool] Updated `src/gradient_os/arm_controller/command_api.py` so `handle_wait_for_idle()` now polls unified motion status, returns explicit wait metadata (`waited_for_motion`, `wait_timeout_s`, `wait_timed_out`), and reports a normalized terminal state such as `completed` or `timeout`.
- [tool] Updated `src/gradient_os/run_controller.py` and `src/gradient_os/api/main.py` so `WAIT_FOR_IDLE` accepts an optional timeout override while remaining backward compatible with existing callers.
- [tool] Updated `tests/test_command_api_direct_setpoint.py`, `tests/test_api_endpoints.py`, `docs/command_api.md`, `docs/README.md`, and the RTCore redesign plan to reflect the new status-aware wait behavior.
- [self] Guardrail: if a helper is supposed to wait for "motion completion," make it wait on the same status contract the UI/API expose; otherwise mixed controller+RTCore paths will drift into contradictory completion semantics.

### 2026-03-21 - First RTCore jog slice should move timing/timeout into RT even if Cartesian shaping stays in Python
- [user] Asked to prioritize getting RT jog working before the next semantics cleanup, while still keeping non-EtherCAT backends open.
- [self] Implementation rule: for EtherCAT RTCore, keep the current Cartesian jog planner/IK bridge only as a policy layer for now, but move the final timed jog command, per-cycle integration, and stale-command timeout into RTCore.
- [tool] Updated `src/gradient_rt_motion/ipc_v1.hpp` and `src/gradient_rt_motion/main.cpp` so RTCore now accepts `MSG_CMD_JOG`, integrates joint-velocity jog intent in the RT loop, exposes `MOTION_MODE_JOG`, and terminates stale jog commands deterministically.
- [tool] Updated `src/gradient_os/arm_controller/backends/ethercat_rtcore/backend.py` so the controller has a real RTCore jog command path (`send_realtime_jog_command`, `stop_realtime_jog`) instead of only trajectory upload APIs.
- [tool] Updated `src/gradient_os/arm_controller/command_api.py` so EtherCAT RTCore jog now routes through RTCore joint-velocity commands, while simulation and Feetech still keep the existing controller-owned jog loop.
- [self] Mistake/fix: the first RT jog unit test hung because I forgot to seed `LOGICAL_JOINT_LIMITS_RAD`; the jog loop never reached the RTCore send path and kept warning in a loop. Fix was to initialize realistic joint limits in the test harness.
- [self] Guardrail: when migrating mixed-control paths like jog, validate both layers separately: (1) backend command packing / RTCore build, and (2) controller behavior switching away from direct servo writes when RTCore support is present.

### 2026-03-21 - Program terminal semantics must persist separately from active RTCore segment state
- [user] Asked for the highest-value next slice: explicit multi-step program terminal truth with fields like overall state, terminal reason, failing step, completed steps, and completed loops, while keeping non-RT backends compatible.
- [self] Corrective rule: active-only `active_program_*` fields are not enough; persist a separate `program_status` object in `utils.trajectory_state` so `GET_MOTION_STATUS` can still report the last controller-program outcome after the worker thread exits.
- [tool] Updated `src/gradient_os/arm_controller/command_api.py` and `src/gradient_os/arm_controller/utils.py` so `RUN_TRAJECTORY` ACKs and motion-status snapshots now share one program contract (`program` plus mirrored `program_*` fields) instead of exposing only thread-acceptance hints.
- [tool] Updated `src/gradient_os/arm_controller/trajectory_execution.py` so the executor records terminal reasons (`completed`, `operator_abort`, `planner_failure`, `rtcore_fault`, `timeout`, `compatibility_interruption`) and keeps completed-step / completed-loop counters in sync with controller program flow.
- [tool] Updated `web-ui/src/App.tsx` so the trajectory and weld status cards show controller-program lifecycle details, not just the RTCore segment state badge.
- [self] Guardrail: for mixed controller-program + RTCore-segment flows, keep RTCore segment truth in `execution.*` and controller program truth in `program.*`; do not collapse them into one top-level state machine or the UI will confuse "between segments" with "done."

### 2026-03-21 - After a major motion-contract slice, answer "can we test now?" with a concrete rollout ladder
- [user] Asked what is next, when testing can start, and what remains after the program-terminal-semantics implementation.
- [self] Guardrail: when a redesign slice is code-complete and targeted tests pass, say so directly and propose a staged test ladder (targeted software tests -> simulation/non-RT regression -> EtherCAT RTCore dry run -> physical loaded runs) instead of giving a vague "more work first" answer.

### 2026-03-21 - Do not mistake a clean disarmed startup for a motion-path failure
- [user] Reported that "it’s not working" after restarting RTCore/EtherCAT and provided `./start-stack.sh` terminal output as the evidence.
- [self] Key debugging rule: read the controller/start-stack logs carefully before blaming motion code. In this case the stack actually started cleanly, but `GRADIENT_RTCORE_AUTO_ARM=0` left the robot intentionally disarmed, no jog/move commands were ever received, and the user then issued `stop`, which shut controller/API down again.
- [tool] Live `./start-stack.sh probe` after that session confirmed `launcher_state: absent`, `controller_udp: down`, `api_http: down`, and `physical_state: BUS_UP_DISARMED`.
- [self] Guardrail: for live EtherCAT RTCore motion testing, require this sequence before judging the motion code: stack up -> explicit power-up -> verify `physical_state: ACTIVE` -> then issue move/jog commands and inspect resulting controller log lines.

### 2026-03-21 - `start-stack.sh` web readiness is a real localhost fetch, not just a log scrape
- [user] Clarified that they stopped the stack because the UI was not loading, so they could not trigger power-up or any motion command.
- [tool] Confirmed from `start-stack.sh` that `wait_for_web_readiness()` calls `probe_web()`, which does `curl http://127.0.0.1:${WEB_PORT}/` before reporting startup complete.
- [self] Guardrail: if `start-stack.sh` says startup completed, the Vite server did serve `/` locally on the Pi; if the operator still cannot see the UI, the next suspects are the browser URL, network path to port `8000`, or a client-side rendering error after the initial HTML load.

### 2026-03-21 - A localhost `:8000` loading screen is a real frontend/runtime blocker, not a startup myth
- [user] Explicitly corrected the investigation direction: the UI was already open at `http://localhost:8000/` and stuck in a perpetual loading state, with screenshot evidence of the dark loading page.
- [tool] Current live run `20260321-080255` showed controller, API, and Vite all up; `start-stack.sh` had already passed its real localhost web probe and a manual `curl -I http://127.0.0.1:8000/` also returned `200 OK`.
- [self] Corrective rule: when HTML is served locally but the operator sees a blank/loading React shell, do not fall back to "wrong URL", "UI not opened", or "startup failed" unless new evidence appears.
- [self] Highest-value next step is browser-side debugging: inspect console/runtime errors, failed network requests, DOM state, and initial `App.tsx` / `ControlPanel.tsx` / `useEndpoint.ts` / `main.tsx` load gates before resuming live jog or move validation.
- [self] Validation order after the UI is fixed: explicit power-up, confirm `physical_state: ACTIVE`, tiny joint jog, tiny realtime jog, tiny scheduled move, then larger trajectory/program flows.

### 2026-03-21 - Vite dev can serve static JSON indexes from `/public/...` even when sibling robot assets load from `/assets/...`
- [tool] While probing the live stack run `20260321-081917`, `http://127.0.0.1:8000/assets/robots/index.json` and `/assets/tools/index.json` returned the SPA HTML shell (`text/html`), but `/public/assets/robots/index.json` and `/public/assets/tools/index.json` returned the expected JSON payloads.
- [tool] `web-ui/src/ArmVisualizer.tsx` was assuming `/assets/robots/index.json` always served JSON, so the robot visualizer startup path could fail immediately even though the synced asset files existed on disk.
- [tool] Implemented a concrete fix: `resolveRobotUrdfConfig()` now retries the robot asset index via both `/assets/robots/index.json` and `/public/assets/robots/index.json`, and rejects non-JSON responses instead of blindly parsing the HTML shell.
- [self] Guardrail: in this web UI, treat static JSON indexes as potentially different from sibling binary/text assets under Vite dev; verify the served `content-type` before assuming `/assets/...` behaves uniformly.
- [self] Follow-up risk: this fix addresses the robot asset index mismatch, which was a real startup/runtime bug, but if the UI is still visually broken after refresh then the next step is to surface any remaining visualizer/bootstrap failure directly in-app instead of leaving it console-only.

### 2026-03-21 - Distinguish soft-stop from full EtherCAT/RTCore teardown explicitly
- [user] Asked how to completely shut down the EtherCAT master and RTCore after seeing `./start-stack.sh stop` leave the hardware stack in `BUS_UP_DISARMED`.
- [self] Guardrail: when answering stack shutdown questions, check `start-stack.sh` usage/teardown flow instead of inferring from terminal output alone; `stop` is intentionally a soft stop, while `stop --hard` additionally stops `gradient-rt-motion.service` and `ethercat.service`.
- [tool] Manual fallback path confirmed in repo docs/scripts: `sudo systemctl stop gradient-rt-motion.service` then `sudo systemctl stop ethercat.service`; use `./start-stack.sh probe` to confirm RTCore/master report `DOWN`.

### 2026-03-21 - Scheduled RTCore completion must tolerate real feedback settle error
- [user] Reported that the web UI now works and telemetry streams, but linear move buttons produced no visible response while ready-state / brake actions still worked.
- [tool] `logs/startups/latest/controller.log` showed `MOVE_LINE_RELATIVE` requests reaching the controller, planning successfully, and starting the RTCore open-loop executor, followed by `TimeoutError: Timed out waiting for RTCore trajectory ... to complete`.
- [self] Root cause pattern: scheduled RTCore completion in `src/gradient_rt_motion/main.cpp` was requiring exact final encoder-count equality on every commanded axis, which is too strict for real EtherCAT drives even when the trajectory has effectively finished.
- [tool] Implemented two concrete mitigations:
- [tool] `src/gradient_rt_motion/main.cpp` now treats a queued trajectory as completed once the final point is due and every commanded axis is within a small encoder-count tolerance instead of demanding exact equality.
- [tool] `src/gradient_os/arm_controller/backends/ethercat_rtcore/backend.py` now gives RTCore scheduled motion a larger post-plan settle margin before Python declares a completion timeout.
- [self] Guardrail: if a live motion command plans and uploads successfully but the executor times out later, inspect RTCore completion semantics before blaming the UI button path.
- [tool] Deployment nuance: `start-stack.sh` launches `gradient-rt-motion.service`, and that systemd unit runs `/usr/local/bin/gradient-rt-motion`; a local repo build alone does not change live hardware behavior until the RTCore binary is installed/replaced.

### 2026-03-21 - Live EtherCAT RTCore retest succeeded after service install and wait-predicate fixes
- [tool] Installed the patched RTCore binary through `./systemd/rt-motion/install.sh`, which rebuilt `src/gradient_rt_motion/gradient-rt-motion` and deployed it to `/usr/local/bin/gradient-rt-motion` for `gradient-rt-motion.service`.
- [tool] Cold-started the stack, powered up successfully, and live-tested a tiny scheduled move via `/control/move-line-relative` with `dx=0.002`, `speed_multiplier=0.1`, `closed=false`.
- [tool] Observed the end effector move from about `x=0.809` to `x=0.811` m on the second retest while `WAIT_FOR_IDLE` returned `completed` instead of timing out.
- [self] Additional race found: `EthercatRTCoreBackend.wait_for_trajectory_complete()` could accept a stale `motion_done` snapshot from the previous trajectory before seeing the new `traj_id`; fix is to ignore supersession-style completion until the target trajectory has been observed once.
- [self] Residual risk: RTCore status snapshots/ACKs can still look stale at submit time on back-to-back moves (`state_name=completed` or an older `trajectory_id` in the immediate ACK) even though the physical move path now works. Treat that as an observability freshness issue to clean up next, not proof that motion failed.

### 2026-03-21 - For live motion bugs, preserve operator-visible symptoms in the handoff and prioritize the likely concrete failure site
- [user] Explicitly asked for a fresh-chat handoff that treats movement and jog commands as the main validation target, preserves the dirty worktree, and does not gaslight operator observations such as UI error banners.
- [self] Corrective rule: if the operator reports a specific UI motion error, carry that exact banner text into the next handoff and investigate that symptom first instead of downgrading it to unrelated background noise.
- [tool] Current high-value rotate blocker to carry forward: `ERROR,ROTATE,The truth value of an array with more than one element is ambiguous. Use a.any() or a.all()`.
- [self] Likely immediate fix: make the orientation-path handoff NumPy-safe before `src/gradient_os/arm_controller/backends/ethercat_rtcore/backend.py::execute_joint_trajectory(...)` evaluates `if not joint_path`; either normalize `_execute_orientation_path()` output to a plain list or change the backend emptiness check to use length/shape-safe logic.
- [self] Guardrail: keep non-EtherCAT backends compatible while patching this; simulation and Feetech should not inherit EtherCAT RTCore assumptions unless real RTCore support exists there.

### 2026-03-21 - For RTCore accepted-ACK freshness, the controller may need a tiny submission-observation window
- [tool] Live validation after the rotate fix reproduced the remaining observability bug concretely: immediate `MOVE_LINE_RELATIVE` ACKs could show the previous completed RTCore trajectory even while controller state already said `executing`.
- [self] Root cause: `src/gradient_os/arm_controller/command_api.py::_build_motion_execution_metadata(...)` was being called immediately after a background executor thread started, before that thread had necessarily uploaded/committed the new RTCore trajectory.
- [tool] Working fix: normalize batch-IK output in `src/gradient_os/ik_solver.py`, make `execute_joint_trajectory(...)` emptiness checks NumPy-safe, and add a short RTCore-status refresh loop in `command_api.py` so accepted ACKs wait briefly to observe the newly submitted trajectory id/status before serializing metadata.
- [tool] Proven live outcome after the refresh fix and a clean stack restart: immediate tiny linear-move ACK now reported the fresh RTCore state (`trajectory_id=1`, `state_name=executing`, `queue_depth=392`) instead of the stale previous-completed snapshot.
- [self] Residual risk: after motion completion, `controller_motion_state` can still remain `"executing"` in some `WAIT_FOR_IDLE` / `GET_MOTION_STATUS` payloads even though `controller_thread_running=false` and RTCore reports `completed`. Treat that as a separate controller-state cleanup, not a regression of the move path.
- [tool] Operational lesson: `POST /control/restart-controller` was not a clean hot-reload path here; it led to launcher/controller/API teardown and disarmed the robot. For Python control-path changes, prefer a clean `./start-stack.sh` restart when live validation is needed.

### 2026-03-21 - Top-level executor threads must restore controller motion state, not just clear thread bookkeeping
- [self] Root cause for the last status mismatch: `src/gradient_os/arm_controller/trajectory_execution.py` top-level `_open_loop_executor_thread(...)` and `_closed_loop_executor_thread(...)` cleared `is_running/thread` on completion but did not call `utils.set_motion_state("IDLE")` in their owning-lifecycle teardown.
- [tool] Working fix: when an executor owns lifecycle management (`owns_trajectory_state=True` and it is the registered trajectory thread), it now restores `motion_state` to `IDLE` at teardown, matching the program executor's existing behavior.
- [tool] Added regression tests in `tests/test_trajectory_execution_backends.py` proving both top-level open-loop and closed-loop executors reset the controller motion state back to `IDLE` when they finish.
- [tool] Proven live outcome on run `20260321-231102`: after a tiny RTCore-backed `MOVE_LINE_RELATIVE`, both `POST /control/wait-for-idle` and `GET /control/motion-status` reported `execution.controller_motion_state="idle"` with `controller_thread_running=false` while RTCore reported `completed`.

### 2026-03-21 - Scheduled RTCore motion was still losing smoothing information before CSP output
- [user] Reported that jog and linear moves basically worked, but joint history still looked stop-start/jerky and one jog command showed a 1-2 s delay before motion.
- [tool] `terminals/1.txt` only showed poll cadence and repeated zero-jog keepalives; it did not contain the actual joint samples needed to judge smoothness, so raw captures are necessary for this class of issue.
- [tool] Inspection of `src/gradient_rt_motion/main.cpp` showed RTCore already interpolated queued trajectories at 1 kHz, so the remaining problem was not simply "100 Hz uploads with no RT interpolation".
- [self] More plausible root cause: scheduled uploads were rounding every uploaded waypoint to integer counts immediately and discarding the optional per-point `qd` velocity channel, so RTCore replay could only use rounded position counts and always wrote `target_vel_out = 0`.
- [tool] Implemented a concrete smoothing patch:
- [tool] `src/gradient_os/arm_controller/backends/ethercat_rtcore/backend.py` now estimates per-sample joint velocities for scheduled trajectories, maps them to RT axes, and uploads them with `TRAJ_POINTF_HAS_VELOCITY`.
- [tool] `src/gradient_rt_motion/main.cpp` now stores scheduled trajectory points as double-precision counts instead of pre-rounded integers, rounds only at the final CSP write boundary, carries interpolated/slope-derived velocity feedforward through replay, and writes nonzero `target_vel_out` for scheduled motion and RTCore jog.
- [tool] Regression coverage updated in `tests/test_gradient05_limits_and_backends.py`; focused RTCore backend tests passed and `src/gradient_rt_motion` rebuilt successfully.
- [tool] Live deployment nuance discovered: `./systemd/rt-motion/install.sh` copies the binary but only runs `systemctl start`; if `gradient-rt-motion.service` is already active that does **not** reload the new executable. Explicit `sudo systemctl restart gradient-rt-motion.service` is required before live retest.
- [tool] Live retest on run `20260321-233817`: installed binary, explicitly restarted RTCore, cold-started the stack, powered up, ran a tiny `/control/move-line-relative` (`dx=0.002`, `speed_multiplier=0.1`, `closed=false`), received a fresh executing ACK, and completed successfully. Saved capture artifacts:
- [tool] `diagnostics/live_capture/20260321-233918-move-line-relative-summary.json`
- [tool] `diagnostics/live_capture/20260321-233918-move-line-relative-joints.csv`
- [self] Important measurement caveat: `/info/joints` / UDP `GET_JOINT_ANGLES` only expose 0.01 deg precision, and `/run/gradient-rt-motion/metrics.json` updates at 10 Hz, so current captures are too coarse to conclusively quantify smoothness improvement. They confirm command acceptance/completion, not a final verdict on jerk elimination.
- [self] Next best slice if the operator still sees stop-start motion: add a higher-resolution target/feedback capture path (raw counts or RTCore trace) instead of relying on rounded API/UI joint readouts.

### 2026-03-21 - Treat operator-observed physical motion as ground truth, and fix observability precision before arguing from charts
- [user] Explicitly clarified that when reporting jerk/lag they are watching the real robot move, not only reading logs or charts; logs are supplemental evidence for the agent, not the primary symptom source.
- [self] Guardrail: if the operator reports visible motion artifacts, assume the physical symptom is real even when current telemetry is too coarse to prove it. Improve the readout path instead of downgrading the report.
- [tool] Found that `run_controller.py::GET_JOINT_ANGLES` still rounded to 0.01 deg and `command_api.py::GET_POSITION` still rounded joint values on the wire, so observability itself was throwing away useful motion detail after the RTCore smoothing fix.
- [tool] Added high-precision joint readout support:
- [tool] `GET_JOINT_ANGLES` now emits high-precision degrees, `GET_JOINT_STATE` now returns a JSON snapshot with `arm_rad`, `arm_deg`, and backend raw axis counts when available, and `api/main.py` exposes `/info/joints-detailed` for capture/debug use without bloating the hot `/info/joints` poll path too much.
- [tool] Side fix for responsiveness/noise: `web-ui/src/ControlPanel.tsx` no longer keeps spamming unchanged zero jog vectors on the keepalive timer, and the jog tick interval was relaxed from 20 ms to 50 ms.
- [self] Guardrail: zero-velocity jog keepalives are not free; if they are unchanged, prefer one explicit zero on transition plus controller-side timeout semantics over periodic zero spam.
 
### 2026-03-22 - Lag diagnosis needs stitched timing across API, controller UDP, jog loop, and RTCore
- [user] Asked for concrete analytics/logs to catch lag and identify bottlenecks when motion still occasionally looks delayed between command issue and command receipt.
- [self] Corrective rule: when the complaint is "lag between issue and receive", do not add more generic console spam first; instrument the exact handoff layers with rolling timing data so the slow hop is observable.
- [tool] Added controller-side UDP performance stats in `src/gradient_os/run_controller.py` and a new `GET_PERFORMANCE_STATE` UDP command so timing can be queried without scraping logs.
- [tool] Added jog-loop timing stats in `src/gradient_os/arm_controller/command_api.py` covering velocity update cadence, loop duration/overruns, and stage timings for feedback read, IK solve, and command send.
- [tool] Added API-side UDP round-trip timing plus `GET /debug/performance` in `src/gradient_os/api/main.py`, which merges API metrics, controller metrics, and RTCore `metrics.json` jitter/overrun data into one snapshot.
- [self] Guardrail: the first thing to inspect on future lag reports should be `/debug/performance`; compare API UDP round-trip, controller dispatch time/interarrival gaps, jog overrun counts, and RTCore jitter before changing motion logic again.

### 2026-03-22 - Live tiny-move retest showed queued-move delay is mostly planning plus intended motion time, not transport lag
- [tool] Cold-started the stack headless from a hard stop; RTCore and EtherCAT returned to `BUS_UP_DISARMED`, then powered up successfully for live testing.
- [tool] Ran three tiny queued `/control/move-line-relative` commands (`+2 mm`, `+2 mm`, `-4 mm`, all `closed=false`, `speed_multiplier=0.08`) plus a tiny realtime jog pulse (`vx=0.008 m/s` with 6 updates at ~50 ms cadence).
- [tool] `/debug/performance` and launcher logs agreed that API/controller command transport was fast: `SAFE_POWER_UP` dispatch about `0.7 ms`, `SET_JOG_VELOCITY` dispatch about `0.84 ms` avg / `1.64 ms` max, API UDP round-trip for jog about `0.05 ms`, and RTCore jitter stayed around `1.7-2.0 us` with `rt_overrun_count=0`.
- [self] Important interpretation: the visibly "slow" queued moves were dominated by synchronous planning/upload and the commanded trajectory duration, not a blocked command pipe. Live logs showed batch IK about `229-313 ms`, total plan/prepare about `486-643 ms`, then RTCore execution about `4.07 s`, `4.08 s`, and `5.94 s` for the tiny queued trajectories.
- [self] Guardrail: `WAIT_FOR_IDLE` timing is not a command-latency metric; it intentionally includes the whole motion duration. Likewise `MOVE_LINE_RELATIVE` HTTP time includes planning before the ACK is returned.
- [self] Metrics nuance: the jog loop marked every cycle as an "overrun" because the current threshold counts any loop slightly above `20.0 ms`; observed max overshoot was only about `0.11 ms`, so treat this as scheduler granularity / threshold noise unless it climbs into multi-millisecond territory.

### 2026-03-22 - UI-driven diagnostics should be visible where the operator tests, not only in API output
- [user] Asked how to ensure timing diagnostics record while testing with the web UI, and whether the UI itself exposes enough information to spot end-to-end bottlenecks.
- [self] Corrective rule: if instrumentation exists only in backend endpoints, surface the most decision-useful subset in the operator-facing UI so live testing does not require juggling curl calls during motion.
- [tool] Confirmed that the diagnostics were already recording automatically whenever the UI calls the API, because the metrics live in `api/main.py`, `run_controller.py`, and `command_api.py`.
- [tool] Added a live `Timing Diagnostics` panel to `web-ui/src/ControlPanel.tsx` that polls `/debug/performance` every `500 ms` and shows API RTT, controller dispatch/interarrival, jog loop timings, and RTCore jitter/overruns.
- [self] Guardrail: label `MOVE_LINE_RELATIVE` and `WAIT_FOR_IDLE` carefully in the UI because those numbers include planning time and full motion time respectively; they are not pure network latency.

### 2026-03-22 - If the operator says a panel is unusable in context, move it to the workflow surface they actually watch
- [user] Rejected the diagnostics block inside `ControlPanel` because it crowded the robot control card and was not usable during live chart observation.
- [self] Corrective rule: when the operator says a diagnostic belongs "below the live charts", treat that as a layout requirement, not a style preference.
- [tool] Moved diagnostics into a dedicated `web-ui/src/PerformanceDiagnosticsPanel.tsx` component, mounted under the `Live Charts` drawer in `web-ui/src/App.tsx`, and added a hide/show toggle there.
- [tool] Removed the duplicated diagnostics block and polling state from `web-ui/src/ControlPanel.tsx` so the UI only polls `/debug/performance` once.

### 2026-03-22 - For telemetry tooling, prefer workspace tabs over stacking more panels into one narrow drawer
- [user] Reported that even under the live charts, the diagnostics were still too cramped and asked for a wider panel plus clearer visualizations, suggesting another tab in the live charts area.
- [self] Corrective rule: when a diagnostics surface competes with dense charts in a narrow drawer, split the tasks into tabs instead of forcing both into one scrolling column.
- [tool] Added `web-ui/src/TelemetryWorkspace.tsx` to provide `Live Charts` / `Diagnostics` tabs plus a show/hide toggle for the diagnostics tab itself.
- [tool] Widened the telemetry drawer in `web-ui/src/App.tsx` and upgraded `web-ui/src/PerformanceDiagnosticsPanel.tsx` with top-level signal cards / progress bars for quicker scanning.

### 2026-03-22 - EtherCAT RTCore already had richer servo diagnostics, but Python/UI were dropping most of them
- [user] Asked whether the diagnostics panel could read more feedback directly from the servos and requested that any extra telemetry coming back should be surfaced too.
- [self] Corrective rule: when a telemetry view is blank on one backend, check whether the backend has a parallel status path before assuming the hardware cannot report the data.
- [tool] Found that `src/gradient_rt_motion/main.cpp` already publishes `pos_counts`, `torque_raw`, `statusword`, `error_code`, `mode_display`, `ds402_state`, `di_bits`, `axis_fault_flags`, and `brake_state` in `StatusSnapshotV1`, but `src/gradient_os/arm_controller/backends/ethercat_rtcore/backend.py` only unpacked counts/status/error.
- [tool] Extended the RTCore backend snapshot parsing plus `src/gradient_os/run_controller.py` joint/telemetry snapshots so EtherCAT telemetry now carries those extra per-axis fields into the UI stream on every telemetry tick.
- [tool] Updated `web-ui/src/App.tsx` and `web-ui/src/TelemetryCharts.tsx` so the telemetry tab shows RTCore-specific charts/status chips when voltage/current/temp are unavailable on EtherCAT.
- [self] Guardrail: expensive live register reads can stay throttled, but RTCore shared-memory status is cheap enough to publish every telemetry frame.
- [tool] Validation: `"/home/pi/GradientOS/.venv/bin/python" -m pytest tests/test_api_endpoints.py`, `"/home/pi/GradientOS/.venv/bin/python" -m py_compile ...`, `npm run build`, and `ReadLints` on the touched files all passed.

### 2026-03-22 - Do not hide the operator's baseline telemetry cards when optional data is missing
- [self] I introduced a UI regression by filtering the lower telemetry cards out when their series arrays were empty, which made the whole lower chart area disappear until data arrived.
- [user] Immediate feedback made it clear the missing cards read as "you removed my charts", not as a graceful empty state.
- [self] Corrective rule: preserve the baseline card layout for operator-facing telemetry even when values are absent; add backend-specific charts only as extras, never by replacing/hiding the originals.
- [tool] Fixed `web-ui/src/TelemetryCharts.tsx` to always render the original `Voltage`, `Current`, `Torque/Load`, and `Temp` cards, while keeping RTCore-specific charts additive-only.
- [tool] Validation: `npm run build` in `web-ui` and `ReadLints` on `web-ui/src/TelemetryCharts.tsx` both passed.

### 2026-03-22 - Make backend telemetry availability explicit instead of implying fields exist live when they do not
- [user] Called out the mismatch between my earlier "we have torque/status/etc" explanation and the fact that some expected charts like voltage/current/temp still were not visible/populated.
- [self] Corrective rule: distinguish between "field supported somewhere in code" and "field live on the current backend path". Especially for multi-backend telemetry, state clearly which metrics are actually streaming on EtherCAT vs only on Feetech.
- [tool] Confirmed `feetech` is the only backend with `TELEMETRY_BLOCK*` register parsing for voltage/current/temp, while EtherCAT RTCore currently publishes counts, torque raw, mode, statusword, error code, and digital inputs.
- [tool] Updated `web-ui/src/TelemetryCharts.tsx` to add per-panel hide/show toggles, always-rendered chart slots with empty-state messaging, an explicit telemetry-availability note for EtherCAT RTCore, and a fuller per-axis drive feedback table showing all currently available live fields.
- [tool] Validation: `npm run build` in `web-ui` and `ReadLints` on `web-ui/src/TelemetryCharts.tsx` both passed.

### 2026-03-22 - If the scene is alive but the robot is invisible, check the shipped web meshes before touching camera math
- [user] Reported "where's the robot" while the 3D workspace still showed axes/grid, meaning rendering/camera were alive but model geometry was missing.
- [self] Mistake: my earlier file search was wrong for this repo state. A direct directory listing showed the real `gradient-05` STL files do exist under both `robots/gradient-05/stl-files/` and `web-ui/public/assets/robots/gradient-05/stl-files/`.
- [self] Corrective rule: for asset/render regressions involving binary files, verify with direct directory listing or runtime browser evidence instead of trusting a single search result.
- [tool] Removed the bulky top telemetry toggle strip from `web-ui/src/TelemetryCharts.tsx` and kept per-chart hide controls plus a compact `Restore Hidden Panels` action instead.
- [tool] Reverted the fallback robot hack from `web-ui/src/ArmVisualizer.tsx`; the real `gradient-05` mesh-based robot renders again in the live browser.
- [tool] Validation: live browser check on `http://127.0.0.1:8000/` showed the actual robot model visible again; `npm run build` in `web-ui` and `ReadLints` on `web-ui/src/TelemetryCharts.tsx` + `web-ui/src/ArmVisualizer.tsx` both passed.

### 2026-03-22 - Delayed motion then burst execution can be a browser/API queueing bug even when RTCore timing is healthy
- [user] Reported a live run where controls sometimes felt unresponsive and then later all pushed through, causing unexpected robot motion.
- [tool] The timing panel snapshot showed `jog dispatch` staying sub-5 ms and RT jitter staying in the low microseconds, but `move ACK` stayed around `339 ms` and `jog velocity gap` reached hundreds of ms to multi-second spikes.
- [tool] `logs/startups/20260322-033216/controller.log` showed the concrete symptom: bursts of identical `SET_JOG_VELOCITY` commands from many source ports, with stale nonzero and zero jog commands interleaving before `JOG_STOP`.
- [self] Root-cause rule: if the controller log shows repeated identical motion commands arriving in bursts while controller dispatch and RT jitter remain healthy, suspect client/API request pileup before changing RT motion logic.
- [tool] Fixed `web-ui/src/ControlPanel.tsx` so realtime jog command sends are single-flight and latest-wins/coalesced instead of starting a new `fetch()` every `50 ms` while previous sends are still in flight.
- [tool] Added a discrete-motion guard in `web-ui/src/ControlPanel.tsx` so incremental cartesian moves/rotates and `Home`/`Rest` cannot stack multiple motion requests while one command is still being issued.
- [self] Guardrail: treat `MOVE_LINE_RELATIVE` RTT in the diagnostics panel as synchronous planning/upload time, not transport lag. Long move ACKs can make the UI feel stalled, so the frontend must coalesce or lock motion commands instead of letting stale clicks/jog updates queue up.

### 2026-03-22 - Diagnostics collection tied to a hidden UI tab is not reliable enough for live motion debugging
- [user] Reported that the latest run still showed lag and that if the diagnostics tab was not open, the diagnostics did not appear to record.
- [tool] Confirmed the architecture bug in `web-ui/src/TelemetryWorkspace.tsx` and `web-ui/src/PerformanceDiagnosticsPanel.tsx`: the `/debug/performance` polling loop only existed inside the diagnostics component, so sampling stopped whenever the `Diagnostics` tab was not selected.
- [tool] Latest run `logs/startups/20260322-051448/controller.log` still showed two separate timing truths:
- [tool] queued `MOVE_LINE_RELATIVE,0.05,...` spent about `303 ms` in planning before execution, then about `2.0 s` in RTCore motion time
- [tool] jog still showed bursts of repeated identical `SET_JOG_VELOCITY` commands, so a backend-side safety net was still worthwhile even after the frontend single-flight fix
- [tool] Updated `web-ui/src/TelemetryWorkspace.tsx` so diagnostics polling runs continuously while the telemetry workspace is mounted, regardless of whether the `Diagnostics` tab is selected, and removed the old visibility-gate dependency by making `PerformanceDiagnosticsPanel.tsx` presentational.
- [tool] Added API-side duplicate jog coalescing in `src/gradient_os/api/main.py` so identical `POST /control/jog/velocity` payloads arriving within a very short window are acknowledged but not forwarded repeatedly to the controller.
- [self] Guardrail: for operator diagnostics, put recording/sampling in an always-mounted path, and for safety-critical motion inputs add backend-side coalescing or dedupe so one stale/noisy browser session cannot flood the controller even if a UI-side fix regresses or a stale bundle stays open.

### 2026-03-22 - Any jog dedupe optimization must remain strictly fail-closed on release/disconnect
- [user] Explicitly reinforced the safety invariant: while jogging, if the button is not being held the robot must stop moving, and if the frontend drops mid-hold we must not leave motion logically stuck "on".
- [tool] Existing controller safety path already mattered here: `src/gradient_os/arm_controller/command_api.py` keeps `JOG_VELOCITY_TIMEOUT_S = 0.5`, zeros jog velocities on timeout, enforces `jog_deadman`, and `handle_jog_stop()` tears the jog thread down.
- [self] Corrective rule: never let API/frontend dedupe suppress a zero-release command. Nonzero duplicates can be coalesced, but zero commands and deadman-off transitions must always be forwarded immediately.
- [tool] Hardened `src/gradient_os/api/main.py` so:
- [tool] all-zero jog vectors are never coalesced
- [tool] coalescing state resets on `JOG_START`, `JOG_STOP`, and `SET_JOG_DEADMAN,false`
- [tool] `/debug/performance` still reports coalescing counters for visibility
- [tool] Hardened `web-ui/src/ControlPanel.tsx` so page hide / unload / panel teardown triggers a best-effort jog failsafe stop: send deadman false, send zero velocity, send jog stop, and clear the local jog timer/count state.
- [tool] Added regression coverage in `tests/test_api_endpoints.py` proving that duplicate nonzero jog commands may be coalesced but duplicate zero-release jog commands are still forwarded every time.
- [self] Guardrail: for jog safety, rely on layered fail-safe semantics, not a single happy-path stop message: pointer release zero, UI teardown best-effort stop, API no-zero-coalesce rule, controller deadman gate, and controller timeout zeroing.

### 2026-03-22 - STEP topology failure on this Pi was a missing CAD binding, not a bad STEP parser
- [tool] The live error banner `Failed to load CAD topology: {"detail":"No usable OpenCascade Python binding found..."}` matched the backend import failure in `src/gradient_os/cad/topology_service.py`, before any STEP geometry was parsed.
- [tool] On this host, neither `OCC` nor `OCP` was installed initially in `.venv`; `cadquery-ocp` installed cleanly and `_load_occ_api()` then succeeded.
- [self] Mistake: I first assumed the Raspberry Pi-friendly package would be `cadquery-ocp-arm`, but both `uv` and `pip` could not resolve a usable Linux wheel here.
- [self] Corrective rule: for OpenCascade bring-up on Linux ARM64 in this repo, test the actual wheel resolution on the target host before codifying package advice; prefer the package that proves importable (`cadquery-ocp` here), not the one that sounds platform-specific.
- [tool] Repo guardrails added:
- [tool] `pyproject.toml` `cad` extra now installs `cadquery-ocp`
- [tool] `setup.sh` now prompts for CAD STEP topology support explicitly
- [tool] `docs/README.md` now documents the `cad` extra
- [tool] `topology_service.py` now returns an actionable install hint pointing to `uv pip install -e '.[cad]'`
- [tool] Validation: `pytest tests/test_topology_service.py`, `py_compile src/gradient_os/cad/topology_service.py`, `uv pip install -e ".[cad]"`, and a direct `_load_occ_api()` import check all passed.

### 2026-03-22 - If scheduled moves “freeze” but status polling stays alive, check RTCore command-ring backpressure before blaming the whole controller
- [user] Reported that the latest controller run seemed completely unresponsive.
- [tool] Latest startup log `logs/startups/20260322-183832/controller.log` showed the controller still answering `GET_MOTION_STATUS`, `GET_JOINT_ANGLES`, and `GET_PERFORMANCE_STATE`, but a scheduled move thread crashed on the third `MOVE_LINE_RELATIVE` with `RuntimeError: cmd ring overflow`.
- [self] Corrective rule: distinguish “controller process dead” from “motion worker failed while polls still work”. If only scheduled motion dies after back-to-back moves, inspect RTCore command-ring producer/consumer pressure before changing UI timing again.
- [tool] Updated `src/gradient_os/arm_controller/backends/ethercat_rtcore/backend.py` so `_cmd_ring_write()` waits briefly for ring space to free up instead of failing immediately on a transient full-ring sample.
- [tool] Added regression coverage in `tests/test_gradient05_limits_and_backends.py` proving a temporarily full RTCore command ring can drain and then accept the pending write without raising.
- [tool] Validation: `python3 -m py_compile src/gradient_os/arm_controller/backends/ethercat_rtcore/backend.py` and `pytest -q tests/test_gradient05_limits_and_backends.py -k "execute_joint_trajectory_uses_quantized_timing or cmd_ring_write_waits_for_space"` both passed.

### 2026-03-22 - A diagnostics error banner captured after stack shutdown can coexist with healthy last-sample metrics
- [user] Shared a diagnostics screenshot taken after shutdown and asked whether the controller still looked unresponsive.
- [tool] Newest run `logs/startups/20260322-211403/controller.log` showed all scheduled moves and rotates finishing cleanly with `RTCore trajectory execution finished: state=completed` and no `cmd ring overflow`.
- [tool] `logs/startups/20260322-211403/api.log` returned `GET /debug/performance` as `200 OK` throughout the run, then only switched to `503 Service Unavailable` during normal shutdown.
- [self] Corrective rule: when a screenshot is taken after shutdown, treat a “No response for command …” banner as a post-stop polling failure unless the live logs show matching errors before SIGTERM. The timing tiles may simply be the last good sample frozen on screen.
- [tool] Remaining live issue in that run was jog cadence, not a dead controller: the controller still logged repeated `SET_JOG_VELOCITY` bursts plus `Jog Timeout 0.50s` zeroing events, and the API log still showed many `POST /control/jog/velocity` requests during each jog burst.

### 2026-03-22 - For safe manual jog, the browser should publish intent while the API owns the steady heartbeat and lease-expiry stop
- [user] Asked to fix the diagnostics confusion first, then step back and rethink the jog sender architecture with safety as the primary constraint.
- [self] Corrective rule: do not let a browser tab be the only component responsible for maintaining a safety-critical realtime cadence to the controller. Let the UI publish current intent, but move the stable command heartbeat and disconnect lease handling into the API.
- [tool] Updated `web-ui/src/PerformanceDiagnosticsPanel.tsx` so a failed `/debug/performance` poll no longer keeps a misleading `live` badge; the panel now marks the snapshot `offline/stale` and explicitly says it is showing the last successful sample plus the last poll error.
- [tool] Added an API-side jog bridge in `src/gradient_os/api/main.py` with a new `POST /control/jog/state` endpoint:
- [tool] the UI now sends full jog intent (`active`, `deadman`, velocity vector)
- [tool] the API bridge owns the steady controller-side jog cadence
- [tool] lease expiry fails closed with zero velocity, deadman false, and jog stop
- [tool] `/debug/performance` now exposes `api_jog_bridge` state for visibility
- [tool] Updated `web-ui/src/ControlPanel.tsx` to use `/control/jog/state` instead of streaming separate start/deadman/velocity requests as the primary path, while keeping the unload/page-hide failsafe as a best-effort inactive state publish.
- [tool] Added API regression coverage in `tests/test_api_endpoints.py` for the new jog state bridge start/stop sequence and for the new diagnostics payload field.
- [tool] Validation: `python3 -m py_compile src/gradient_os/api/main.py`, `pytest -q tests/test_api_endpoints.py -k "debug_performance or control_jog_velocity_zero_release_is_never_coalesced or control_jog_state_bridge_starts_and_stops_jog"`, `npm run build` in `web-ui`, and `ReadLints` on edited files all passed.

### 2026-03-22 - Startup log review should quantify live polling load before blaming RTCore or controller responsiveness
- [user] The target architecture is controller-owned jog sessions with backend-native execution where available; review startup logs against that direction, not only for hardware faults.
- [tool] `logs/startups/20260322-220241` was a clean idle bring-up: `responding=6/6`, `operational=6/6`, no disarmed drive faults, and a clean soft stop back to `BUS_UP_DISARMED`.
- [tool] `logs/startups/20260322-220715` showed healthy hardware too, but active UI use generated very high control-plane traffic: `26729` `GET /info/joints`, `29044` `GET /control/motion-status`, `996` `GET /debug/performance`, and `722` `POST /control/jog/state` requests in one run.
- [tool] The same active run still fanned those requests into legacy controller commands (`29` `JOG_START`, `58` `SET_JOG_DEADMAN`, `2641` `SET_JOG_VELOCITY`, `29` `JOG_STOP`), so ownership is still effectively request/heartbeat-shaped rather than session/sequence-shaped.
- [self] Corrective rule: when reviewing jog behavior, inspect both `api.log` request volume and `controller.log` command fan-out together. High-rate polling plus legacy `JOG_*` fan-out is an architecture smell even when the bus itself is healthy.
- [tool] During shutdown, `logs/startups/20260322-220715/api.log` ended with `ERROR: ASGI callable returned without completing response.` plus several `503 Service Unavailable` responses; treat that as teardown/drain behavior to harden, not as a live-motion fault.

### 2026-03-23 - Controller-owned jog sessions should make the API thin, even for compatibility endpoints
- [user] Asked to implement the new controller-owned jog architecture and thin API from the attached plan, not just discuss it.
- [tool] Added `src/gradient_os/arm_controller/jog_session.py` plus backend capability hooks so the controller now owns session id, owner, lease, sequence, pause-for-motion, stale packet rejects, and backend-mode diagnostics.
- [tool] Updated `src/gradient_os/run_controller.py`, `src/gradient_os/api/main.py`, and `web-ui/src/ControlPanel.tsx` so the main path is now `JOG_SESSION_START/UPDATE/STOP` via thin REST proxies (`/control/jog/session/*`) and the UI keeps local `session_id` / `seq` state.
- [self] Corrective rule: if a compatibility endpoint must remain during a migration, keep it stateless if possible. Here `/control/jog/state` became a deprecated thin mapper that emits legacy controller commands directly, instead of reviving an API-owned lease/heartbeat thread.
- [tool] Validation that specifically covered the refactor slice passed: `pytest -q tests/test_api_endpoints.py tests/test_jog_session_manager.py`, `pytest -q tests/test_gradient05_limits_and_backends.py -k "realtime_jog or jog"`, `python -m py_compile ...command_api.py run_controller.py api/main.py...`, and `npm run build` in `web-ui`.
- [self] Guardrail: when diagnostics move from API-owned state to controller-owned state, update both the payload shape and the UI renderer in the same change so the operator never ends up reading stale architecture labels like `api_jog_bridge`.

### 2026-03-23 - For new-agent handoffs, update the plan doc with implemented state, not just desired architecture
- [user] Explicitly asked for the jog-session plan document itself to be updated with full handoff notes for a fresh agent.
- [self] Corrective rule: when a refactor is partly complete and the user asks for a handoff, update the original plan file so it reflects reality: completed slices, touched files, validations run, remaining risks, and the exact next steps.
- [tool] Updated `/home/pi/.cursor/plans/jog-session-refactor_01c763e3.plan.md` to mark completed rollout slices, add a current implementation status section, and include a detailed next-agent handoff focused on live RTCore validation and compatibility cleanup.

### 2026-03-23 - Safety-critical REST booleans must never rely on Python truthiness
- [self] Mistake pattern: plain `bool(payload.get(...))` is unsafe for API request parsing because string inputs like `"false"` evaluate truthy and can accidentally keep `active` or `deadman` enabled.
- [self] Corrective rule: for jog-related REST endpoints, explicitly coerce booleans from `true/false/1/0/yes/no/on/off` and reject anything else with `400`, especially for fail-closed fields like `active`, `deadman`, and `enabled`.
- [tool] Hardened `src/gradient_os/api/main.py` so `/control/jog/state`, `/control/jog/deadman`, `/control/jog/debug`, and `/control/jog/session/start|update` now use explicit boolean parsing plus integer parsing for `seq`, returning `400` instead of leaking malformed values into `500`s.
- [tool] Removed the orphaned `_ApiJogStateBridge` implementation from `src/gradient_os/api/main.py` after confirming it no longer had any references in the controller-owned session architecture.
- [tool] Added regression coverage in `tests/test_api_endpoints.py` proving string `"false"` stays fail-closed on jog endpoints and malformed session `seq` values are rejected cleanly.
- [tool] Validation: `"/home/pi/GradientOS/.venv/bin/python" -m pytest -q tests/test_api_endpoints.py tests/test_jog_session_manager.py`, `"/home/pi/GradientOS/.venv/bin/python" -m py_compile src/gradient_os/api/main.py`, and `ReadLints` on touched files all passed.

### 2026-03-23 - When removing a deprecated control path, delete the observability and secondary-UI remnants too
- [user] Explicitly asked to remove all remaining legacy jog fallback surfaces so there is no chance of silently falling back onto the old implementation.
- [self] Corrective rule: for architecture cleanups, do not stop at the primary API/controller handlers. Also remove stale diagnostics fields, docs, tests, and secondary UIs that still imply the old contract exists.
- [tool] Removed legacy jog REST routes from `src/gradient_os/api/main.py`, removed the legacy adapter functions from `src/gradient_os/arm_controller/command_api.py`, and changed `src/gradient_os/run_controller.py` so stale `JOG_START` / `SET_JOG_*` callers now receive explicit `LEGACY_JOG_REMOVED` errors instead of compatibility behavior.
- [tool] Updated `web-ui/src/PerformanceDiagnosticsPanel.tsx` and `docs/README.md` / `docs/command_api.md` so operator-facing diagnostics and docs now refer only to `JOG_SESSION_*` and the session endpoints.
- [tool] Disabled the old desktop UDP jog panel in `src/gradient_os/ui/pages/real_control_page.py` so the supported jog path remains the web `ControlPanel` -> API session endpoints -> controller session manager.
- [tool] Validation: `"/home/pi/GradientOS/.venv/bin/python" -m pytest -q tests/test_api_endpoints.py tests/test_jog_session_manager.py`, `"/home/pi/GradientOS/.venv/bin/python" -m py_compile src/gradient_os/api/main.py src/gradient_os/run_controller.py src/gradient_os/arm_controller/command_api.py src/gradient_os/ui/pages/real_control_page.py src/gradient_os/ui_start.py`, `npm run build` in `web-ui`, and `ReadLints` on touched files all passed.

### 2026-03-23 - Live architecture validation should separate non-invasive session-path proof from armed motion proof
- [user] Approved proceeding with live validation after the legacy-path cleanup.
- [self] Corrective rule: when a live robotics stack is up but disarmed, still validate the live control-plane invariants first: removed-route enforcement, removed-UDP enforcement, session start/update/owner-conflict/expiry/stop semantics, and diagnostics/logs proving the active path. Then explicitly label motion-coupled/watchdog cases as still unproven instead of overstating coverage.
- [tool] Started a fresh live stack run `20260323-025500` and confirmed healthy bring-up (`responding=6/6`, `operational=6/6`, `wkc=18`).
- [tool] Live checks passed:
  - removed legacy REST jog routes returned `404`
  - removed legacy UDP `JOG_START` returned structured `LEGACY_JOG_REMOVED`
  - `JOG_SESSION_START/UPDATE/STOP` succeeded live with `backend_mode=joint_velocity_lease`
  - owner conflict returned `409 OWNER_CONFLICT`
  - lease expiry transitioned to `expired`
  - explicit `pagehide` stop transitioned to `stopped`
  - stale updates after `expired`/`stopped` returned `SESSION_INACTIVE`
  - `controller.log` showed `JOG_SESSION_*` commands only for the supported path
- [self] Guardrail: if a deliberate legacy-probe command appears once in the controller log during validation, distinguish that from fallback usage. Here the single logged `JOG_START` was the intentional negative test that produced `LEGACY_JOG_REMOVED`.

### 2026-03-23 - If the user says they hard-stopped the stack/RTCore, verify runtime state before assuming the API is still up
- [user] Clarified that the stack had been intentionally stopped and that I needed to restart it again before continuing.
- [self] Corrective rule: after a user-triggered hard stop, do not treat connection-refused on `:4000` as mysterious flakiness. Re-check the supervised stack state and restart it explicitly before continuing live validation.
- [tool] Restarted the supervised stack with `./start-stack.sh` into run `20260323-030328` and confirmed healthy bring-up again (`responding=6/6`, `operational=6/6`, `wkc=18`, API on `:4000`, web on `:8000`).

### 2026-03-23 - If the Pi says the UI is "not loading" but `:8000` serves HTML, suspect browser-side startup cost before blaming Vite
- [user] Reported repeated cases where the UI looked like it was not loading even after the stack was live, and explicitly stopped me when browser-based inspection got stuck waiting on the page.
- [tool] Runtime checks showed the distinction clearly:
- [tool] when the stack was stopped, `http://127.0.0.1:8000/` and `http://127.0.0.1:4000/health` returned connection refused
- [tool] once restarted, both endpoints returned `200`, `src/main.tsx` and `src/App.tsx` served normally, and Vite reported healthy startup in `logs/startups/20260323-035243/web.log`
- [self] Mistake / corrective rule: if browser automation hangs or aborts while local HTTP probes are healthy, stop retrying the browser tool as the primary diagnostic. Treat it as evidence of a client-side load/perf problem and inspect the app's first-render path directly.
- [tool] Root-cause candidate found in `web-ui/src/App.tsx`: the page eagerly mounted `ArmVisualizer` on first paint, and `web-ui/src/ArmVisualizer.tsx` immediately created a `THREE.WebGLRenderer`, loaded URDF assets, and started a continuous animation loop.
- [tool] Fix applied: `App.tsx` now lazy-loads the 3D visualizer chunk and starts in a lightweight mode with a manual `Load 3D Workspace` button so the control UI can open without paying the WebGL/URDF startup cost up front.
- [tool] Additional load-shedding: reduced the motion-status polling interval from `200 ms` to `500 ms`.
- [tool] Validation: `npm run build` passed, `ReadLints` on `web-ui/src/App.tsx` found no diagnostics, the build output showed `ArmVisualizer` as its own chunk, and live `GET http://127.0.0.1:8000/src/App.tsx` confirmed the new lightweight startup strings were being served.

### 2026-03-23 - Runtime diagnostics for "UI hung" cases should work from shell first and avoid fuzzy heuristics
- [user] Asked to add diagnostics so the next browser/UI stall can be diagnosed instead of solved only by rebooting.
- [tool] Added `src/gradient_os/diagnostics/runtime_snapshot.py` plus API surface `GET /debug/runtime` and docs for `python -m gradient_os.diagnostics.runtime_snapshot`.
- [tool] The standalone CLI path already works immediately and writes `logs/diagnostics/<timestamp>-runtime.json`; the HTTP route exists in code but any already-running API process must be restarted before `/debug/runtime` appears live.
- [self] Corrective rule: when adding debug endpoints during an active stack session, separately verify "code/tests pass" vs "live process has reloaded". A `404` on the new route can simply mean the current API process predates the code change.
- [self] Corrective rule: avoid overly broad substring heuristics in diagnostics. Initial versions falsely matched kernel `changed` lines as `hang` hints and classified Cursor node processes as `browser`; tighten with regex/explicit process groups so the snapshot stays actionable.

### 2026-03-23 - Realtime jog sessions must not start "alive but idle" and the UI must preserve controller error codes
- [user] Asked to inspect the newest logs because jog commands appeared broken.
- [tool] Latest `logs/startups/latest/api.log` showed the exact sequence: `POST /control/jog/session/start` returned `200`, then repeated `POST /control/jog/session/update` calls returned `404`.
- [tool] Matching `logs/startups/latest/controller.log` showed why the UI stayed broken:
- [tool] the controller started a jog session, then the jog thread stopped before meaningful updates arrived
- [tool] afterward the UI kept sending `JOG_SESSION_UPDATE` with the same `seq=1` over and over, while the controller treated the session as inactive
- [tool] Root causes found in `web-ui/src/ControlPanel.tsx`:
- [tool] `ensureJogStarted()` eagerly opened a controller session with an all-zero command, so a user could arm realtime jog mode and let the session lease expire before pressing an actual direction
- [tool] `readErrorMessage()` stripped controller error codes like `SESSION_INACTIVE`, leaving only the human message, so terminal-session detection failed and the UI kept retrying against a dead session
- [tool] the publish loop only advanced `jogSessionSeqRef` after a successful response, so once an update failed the UI could get stuck reusing the same stale sequence forever
- [tool] Fix applied:
- [tool] realtime jog now only enables the local timer on Start; it does not create a controller session until the first nonzero command is sent
- [tool] all-zero active payloads with no active session are skipped instead of opening an idle lease
- [tool] controller error messages now preserve `code: message`, so `SESSION_INACTIVE` / `SESSION_EXPIRED` are detectable and the UI can reset session state
- [tool] sequence numbers are advanced optimistically before the request so failed publishes do not pin the UI on one stale `seq`
- [tool] Validation: `npm run build` passed and `ReadLints` on `web-ui/src/ControlPanel.tsx` found no diagnostics.

### 2026-03-23 - Keep jog mode armed for UX, but only keep the controller session alive while a button is actually held
- [user] Wanted to avoid the terrible UX of having to click the jog-mode button before every single move, but still wanted the final architecture to match the safer hold-scoped session model.
- [self] Corrective rule: split `armed jog mode` from `active hold`. The UI may stay armed between clicks, but it must not keep the controller session, heartbeat, or backend jog lease alive unless at least one jog button is actively held.
- [tool] Updated `web-ui/src/ControlPanel.tsx` so the button now arms/disarms jog mode, button press starts the session/heartbeat, and final release enqueues `/control/jog/session/stop` instead of leaving an idle session running.
- [self] `owner_conflict_rejects` should mean real cross-owner conflicts only. Do not increment it for same-owner duplicate starts; those are `SESSION_ALREADY_ACTIVE`, not owner conflicts.
- [tool] When adding frontend tests on this Pi, do not keep the latest `vitest`/`jsdom` pair by default. Node `18.20.4` requires a compatible harness (`vitest@3.x`, `jsdom@26.x`), and `jsdom` needs `setPointerCapture` / `releasePointerCapture` stubbed for these pointer-event tests.
- [tool] Safe live validation against run `20260323-220541` worked without commanding motion by using `deadman=false`: the controller reported `backend_mode=joint_velocity_lease`, lease expiry produced `last_stop_reason=lease-expired`, and explicit stop preserved the caller-supplied reason.

### 2026-03-23 - Motion-coupled RTCore jog tests show session semantics are right, but physical stop behavior still lags behind the logical stop
- [tool] In run `20260323-220541`, a tiny live jog test at `vz=0.003 m/s` with real motion confirmed that power-up transitioned DS402 state codes from `2` to `5`, so the axes were actually operation-enabled before the test.
- [tool] Explicit release path:
- [tool] controller/API path behaved correctly (`JOG_SESSION_STOP`, controller state `stopped`, `last_stop_reason=agent-explicit-release`), but the arm still drifted about `0.159 mm` after stop before settling.
- [tool] Lease-expiry path:
- [tool] controller state transitioned to `expired` with `last_stop_reason=lease-expired`, but measured pose continued moving after expiry, reaching about `1.41 mm` past the hold-end position by `0.45 s` and about `1.67 mm` by `0.75 s`.
- [tool] `controller.log` showed the jog thread had already stopped before those later pose samples, so continued motion was not caused by the Python controller still publishing updates.
- [self] Likely mechanism to inspect next: RTCore stop/timeout ends jog setpoint generation, but does not immediately replace the last jog target with a current-position hold setpoint, so CSP keeps chasing the previously generated target for a while after logical stop/expiry.
- [self] Corrective rule: do not treat `session state = stopped/expired` as proof of physical stop. For safety signoff, always pair logical stop checks with measured post-stop pose deltas.

### 2026-03-23 - RTCore jog stop now snaps hold targets to feedback, and controller stop/expiry paths stop trusting stale loop snapshots
- [tool] Updated `src/gradient_rt_motion/main.cpp` so jog stop/timeout snapshots the previously jogging axes and snaps their CSP hold targets to live feedback in the same RT cycle instead of leaving the last jog target latched.
- [self] Follow-up investigation showed the Python jog thread could still finish FK/IK and emit one more backend update after a stop or expiry landed because it only checked session state at the top of the loop.
- [tool] Updated `src/gradient_os/arm_controller/command_api.py` to re-check `session_active` and `lease_valid` immediately before backend send, stop the RTCore jog backend immediately when the loop notices the session has gone inactive, and send an immediate best-effort backend stop from the API stop handler.
- [tool] Validation used `make -C src/gradient_rt_motion`, `python3 -m py_compile src/gradient_os/arm_controller/command_api.py`, repeated headless stack restarts, and repeated live motion probes at `vz=0.003 m/s`.
- [tool] Explicit stop still shows about `0.19-0.23 mm` of post-stop residual motion after the stop response returns, so the stop path is improved in code structure but not yet physically signed off.
- [tool] Lease-expiry behavior improved materially versus the earlier `~1.67 mm` post-expiry bleed-through: one fine-grained run plateaued from `0.42 s` to `0.50 s` with no additional motion after expiry, but a later run still showed a renewed delta by `0.70 s`, so the path is better but still noisy and not fully deterministic.
- [self] Corrective rule: validate jog safety around the actual backend-stop boundary with dense sampling; coarse `state=stopped/expired` snapshots can look healthy while late pose deltas still reveal residual motion.

### 2026-03-23 - Hard shutdown / cold restart rerun proved the residual stop behavior survives a clean bring-up
- [tool] After the user hard-stopped RTCore, verified `gradient-rt-motion.service` and `ethercat.service` were both inactive and API health on `:4000` was refused before restarting anything.
- [tool] Brought the stack back with `GRADIENT_STACK_INTERACTIVE_CONSOLE=0 ./start-stack.sh --headless`; startup showed full bus recovery (`responding=6/6`, `online=6/6`, `operational=6/6`, `wkc=18`) and the live probe again confirmed DS402 transitioned `2 -> 4 -> 5` before motion.
- [tool] Fresh post-hard-shutdown explicit-stop probe at `vz=0.003 m/s` still showed about `0.169-0.183 mm` residual motion after the stop response, so the remaining stop lag is reproducible across cold restart and is not just stale session state.
- [tool] Fresh post-hard-shutdown lease-expiry probe still accumulated motion after expiry, reaching about `1.391 mm` by `0.42 s` and about `1.423 mm` by `0.50 s`, then settling slightly lower around `1.417 mm` by `0.70 s`.
- [self] Corrective rule: treat cold-restart reproducibility as evidence that the remaining issue is architectural/runtime behavior, not a leftover process or stale RTCore state from previous testing.

### 2026-03-23 - RTCore-native jog debug instrumentation proved the remaining stop motion is now drive convergence to a frozen hold target, not RTCore continuing to advance the target
- [tool] Added RTCore-native jog debug status plumbing across `src/gradient_rt_motion/ipc_v1.hpp`, `src/gradient_rt_motion/main.cpp`, `src/gradient_os/arm_controller/backends/ethercat_rtcore/backend.py`, and `src/gradient_os/arm_controller/command_api.py`, then exposed it through `/debug/performance` as `controller.jog.rtcore_jog_debug`.
- [tool] `./start-stack.sh stop --hard` does not actually stop `gradient-rt-motion.service` / `ethercat.service` in this environment; explicit `systemctl restart ethercat.service gradient-rt-motion.service` was required to get the new RTCore binary live.
- [tool] After explicit RTCore service restart, the new debug block became live and reported motor-side `feedback_pos_counts`, `hold_target_counts`, `output_target_counts`, `output_target_velocity_counts_per_s`, stop reason, and stop age at ~50 Hz.
- [tool] Instrumented explicit-stop probe at `vz=0.003 m/s` showed:
- [tool] before stop, RTCore output targets were still ahead of feedback (`max_abs_output_minus_feedback_counts ~= 248`) with nonzero commanded velocity.
- [tool] by `~20 ms` after stop, `active_jog=false`, `latest_cmd_flags=2`, `last_stop_reason_name=cmd_stop`, and `output_target_velocity_counts_per_s` was all zero.
- [tool] after stop, `output_target_counts` stayed fixed while `feedback_pos_counts` continued moving toward that frozen hold target; the residual end-effector motion therefore came from servo convergence, not from RTCore advancing the target after stop.
- [tool] Instrumented lease-expiry probe showed the same pattern:
- [tool] session expired at the controller level, but RTCore ultimately observed a `cmd_stop` because the controller thread sends an explicit stop as it tears down the expired session.
- [tool] at expiry boundary, `active_jog` dropped false and `output_target_velocity_counts_per_s` went to zero immediately, while `output_target_counts` remained fixed and feedback continued moving toward the frozen target.
- [self] Corrective rule: with stale-target runaway ruled out, the next fix must focus on the stop-hold policy itself (for example repeated feedback re-latching / stronger arrest semantics) rather than more command-lifetime plumbing.

### 2026-03-23 - `stop --hard` bug was in the launcher handoff, and larger jog probes showed the residual stop motion stays roughly absolute
- [self] Root cause of the broken hard stop: `./start-stack.sh stop --hard` only signaled the already-running launcher, but that launcher still had `HARD_STOP=0` in its own environment, so its `cleanup()` path always executed a soft stop.
- [tool] Fix in `start-stack.sh`: after a CLI-requested hard stop kills the launcher, the caller now completes `perform_shutdown_sequence` itself, and systemd service stops now wait for `inactive` with a `systemctl kill` fallback before declaring success.
- [tool] Live verification after the patch: running `./start-stack.sh stop --hard` left both `gradient-rt-motion.service` and `ethercat.service` at `inactive`, and `http://127.0.0.1:4000/health` refused connections.
- [tool] Larger explicit-stop probe at `vz=0.003 m/s` with ~70 updates produced about `11.06 mm` commanded hold displacement; post-stop end-effector motion was still only about `0.139 mm` at `0.10 s`, settling to about `0.097 mm` by `1.0 s`.
- [tool] Larger lease-expiry probe at the same velocity produced about `10.92 mm` hold displacement; motion after expiry still peaked around `1.49 mm` by `0.50 s` and remained about `1.45 mm` at `1.0 s`, while RTCore had already frozen output targets and zeroed output velocities.
- [self] Corrective rule: treat the explicit-stop residual as a mostly absolute servo-settling / frozen-target convergence effect rather than a tiny-move measurement artifact; the much larger lease-expiry overshoot remains the higher-priority stop-hold problem.

### 2026-03-23 - The RTCore post-stop arrest window works as implemented, but only modestly improves lease-expiry travel
- [tool] Implemented a `~200 ms` RTCore stop-arrest window in `src/gradient_rt_motion/main.cpp` that re-latches `hold_target_counts` to live feedback each cycle after jog `cmd_stop` / timeout, and exposed its active axes through `stop_arrest_mask` in the existing `StatusJogDebugV1` payload.
- [tool] Propagated `stop_arrest_mask` through `src/gradient_os/arm_controller/backends/ethercat_rtcore/backend.py`, `src/gradient_os/arm_controller/command_api.py`, and the targeted parser test in `tests/test_gradient05_limits_and_backends.py`.
- [tool] Live explicit-stop probe after deploying the new RTCore build showed `stop_arrest_mask=63` and zero `hold_minus_feedback_counts` / `output_minus_feedback_counts` for the first ~`0.1 s`, proving the relatch window is active and stale-target chase is eliminated during that interval.
- [tool] Despite that, explicit-stop end-effector motion after an ~`11.07 mm` move still sat around `0.153 mm` at `0.10 s` and `0.148 mm` at `1.0 s`, so this change did not materially reduce the explicit-stop residual.
- [tool] Lease-expiry probe after the same change also showed `stop_arrest_mask=63` with zero hold/output error while the arrest window was active, but post-expiry motion only improved modestly: about `1.49 mm -> 1.41 mm` at `0.50 s`, and about `1.45 mm -> 1.40 mm` at `1.0 s`.
- [self] Corrective rule: once RTCore is demonstrably following live feedback during the arrest window and the arm still travels materially after expiry, the next fix should shift from CSP target relatching to a stronger motor-side halt / brake semantic (for example DS402 quick-stop / halt behavior), not a longer version of the same relatch trick.

### 2026-03-24 - DS402 quick-stop should be a distinct fail-closed stop class, not the default operator-release stop
- [tool] Implemented DS402 quick-stop command support end-to-end: added `JOG_FLAG_QUICK_STOP` to `src/gradient_rt_motion/ipc_v1.hpp`, taught RTCore `src/gradient_rt_motion/main.cpp` to start a bounded quick-stop controlword window only when that flag is present, and plumbed the optional `quick_stop` stop argument through `src/gradient_os/arm_controller/backends/ethercat_rtcore/backend.py`, `src/gradient_os/arm_controller/actuator_interface.py`, `src/gradient_os/arm_controller/backends/simulation/backend.py`, and `src/gradient_os/arm_controller/command_api.py`.
- [tool] Live evidence on the current A6-EC setup: when quick-stop was applied to *all* stop commands, explicit operator stop entered DS402 state `6` and then `4`, but explicit-stop motion got worse; do not treat “more aggressive stop semantics” as automatically safer without measuring the actual arm behavior.
- [self] Important diagnosis: lease expiry does not naturally appear to RTCore as a timeout; the Python jog thread was tearing down the expired session by sending a normal stop command, so RTCore saw only `latest_cmd_flags=2` unless the controller explicitly tagged the stop as fail-closed.
- [tool] After fixing the early `session-inactive-before-loop` path in `command_api.py`, lease-expiry teardown now reaches RTCore as `latest_cmd_flags=6` and drives enter DS402 state `6` (`QuickStopActive`) on expiry, while ordinary explicit stop remains untagged.
- [tool] Mixed live result with the new selective fail-closed flag: on the final expiry probe the quick-stop path definitely engaged (`latest_cmd_flags=6`, DS402 `6` at `0.42-0.50 s`), but motion still peaked around `1.54 mm` before settling to about `0.91 mm` by `1.0 s`; the behavior is different, but not yet unequivocally “better” across the whole window.
- [self] Corrective rule: now that the plumbing is selective and proven, the next step is to tune / explicitly configure the drive stop semantics (`0x605A`, `0x605D`, `6085h`) rather than changing higher-level session logic again.

### 2026-03-24 - For UI live-state consolidation plans, make ownership and freshness rules explicit before implementation
- [user] Asked for the `unify-live-state` plan to be updated in-place to reflect the review feedback rather than just discussed.
- [self] When a plan proposes “one stream” or “one store,” convert that into an explicit ownership/freshness contract: define what is primary, what is fallback-only, and which signals distinguish SSE staleness vs API reachability vs controller/backend freshness.
- [self] Reuse existing normalized controller contracts where possible; for motion state, prefer the shape from `src/gradient_os/arm_controller/command_api.py:get_motion_execution_status()` instead of creating a second bespoke SSE schema.
- [self] Keep rich RTCore jog debug / performance data opt-in and outside the default `/monitor` payload, especially after adding stop-arrest and selective quick-stop telemetry on the command side.

### 2026-03-24 - Prefer on-machine diagnostics persistence over browser export for this repo's jog forensics
- [user] Explicit preference: pose-history capture should save locally on the Pi instead of relying on browser-side JSON export.
- [tool] Implemented local persistence via `POST /debug/pose-history` in `src/gradient_os/api/main.py`; UI now auto-saves captured history to `logs/diagnostics/*-pose-history.json` when a jog run transitions from active to stopped and also offers a manual "Save Local JSON" action.
- [self] For IK drift diagnosis, capture the comparison at the source of truth inside `src/gradient_os/arm_controller/command_api.py` right after `solve_ik`: log `current_pose`, `target_pose`, `solved_pose`, `applied_pose`, and target-vs-solved/applied error so drift can be attributed to solver vs clamp/apply.
- [tool] `npm run build` in `web-ui/` runs `sync:assets` first and can dirty generated asset files / lockfiles even when the task only targets diagnostics UI; check `git status` after validation and avoid surprising the user with generated diffs.
- [tool] Validation available in this environment: `ReadLints`, `npm run build`, and `python3 -m py_compile ...` succeeded; `python3 -m pytest ...` could not run because `pytest` is not installed globally here.

### 2026-03-24 - When comparing saved pose history to IK snapshots, distinguish model-consistency from sample timing
- [tool] In `logs/diagnostics/20260324-041345-pose-history.json`, every active sample had non-null `ik_debug` plus `current_joints_deg`, `ik_solution_joints_deg`, and `applied_joints_deg`; there were zero clamps and zero solve failures.
- [self] Important interpretation rule: `target_vs_solved` / `target_vs_applied` can be numerically perfect while the top-level polled `pose` in the same history sample still differs, because the polled pose and the controller's latest IK snapshot are captured on different loops/cadences.
- [self] If `ik_solution_joints_deg == applied_joints_deg` and `target_vs_applied ~= 0`, then the internal IK/model path is behaving consistently; remaining drift must be investigated in model calibration mismatch, execution lag, or feedback-vs-command alignment rather than solver clamp error.

### 2026-03-24 - Industrial jog refactors must keep the controller-owned command model and the test harness aligned
- [user] Explicit preference reinforced: for this jog work, be meticulous and implement the refactor rather than only discussing it.
- [tool] The safe architecture for `src/gradient_os/arm_controller/command_api.py` is: measured FK only seeds or supervises at explicit boundaries, while active-hold target integration and IK seeding stay on controller-owned commanded pose/joints stored in `src/gradient_os/arm_controller/jog_session.py`.
- [self] Corrective rule: when replacing jog clamping with single-step gate rejection, update adjacent tests and fixtures to match the new semantics (`JOG_GATE_REJECTED`, no silent clamp drift, `last_gate_failure_reason`) instead of preserving old clamp expectations by habit.
- [tool] Direct-servo jog tests need two specific guardrails now:
- [tool] `_jog_controller_thread()` reads feedback once before the loop and again in the first cycle, so mocked feedback sequences need an extra initial sample if the test cares about first-cycle anchoring.
- [tool] The controller_cartesian_loop path sends a final zero-speed hold write on thread exit; tests that only care about motion commands should filter out that shutdown hold instead of treating it as a regression.

### 2026-03-24 - When a user asks to update a plan after implementation, reflect both completed scope and remaining validation
- [user] Wants the plan files kept in sync with actual completed work, not left as if everything were still pending.
- [self] Corrective rule: when updating a plan after implementation, change the frontmatter/todo status markers and add a short progress section that separates completed code/test work from still-pending live validation or UI follow-through.

### 2026-03-24 - For live jog validation after a hard stop, verify service/API state first and record commanded-state spans, not just measured motion
- [user] Explicitly hard-stopped everything before asking for live validation and expected me to bring the stack back up before testing.
- [tool] Safe restart sequence here was: confirm `gradient-rt-motion.service` / `ethercat.service` are inactive and HTTP probes refuse connections, then launch `GRADIENT_STACK_INTERACTIVE_CONSOLE=0 ./start-stack.sh --headless`, and wait for `responding=6/6`, `operational=6/6`, `wkc=18`, controller ready, and API `/health` = `200`.
- [tool] After hard stop, `info/joints-detailed` initially showed DS402 state `2` on the active axes; `POST /control/power-up` brought them back to `5` before motion.
- [self] Corrective rule: for this industrial-jog refactor, the primary live proof should come from commanded-state spans gathered from `controller.jog.commanded_pose` plus re-sync/gate metadata, not from raw measured pose alone.

### 2026-03-24 - Full live jog sweeps should log both leg displacement and return-to-baseline residual
- [tool] The saved history from the full sweep lives at `logs/diagnostics/20260324-061926-pose-history.json`.
- [tool] Useful reporting pattern for these back-and-forth jog runs:
- [tool] for each forward leg, record achieved end displacement relative to the per-move baseline
- [tool] for each full out-and-back move, record final residual relative to that same baseline
- [self] Corrective rule: a successful sweep is not just “no aborts.” Call out the actual return residuals by axis/angle so small repeatability errors are visible session-to-session.

### 2026-03-24 - Multi-move auto-saved pose histories need segment-aware interpretation
- [tool] The newer UI-generated captures around `logs/diagnostics/20260324-0625*.json` bundle multiple sequential jog segments into one `source: "auto-stop"` history.
- [self] Corrective rule: do not treat whole-file final-vs-initial pose deltas as proof of per-move return error unless the file is first segmented by active command blocks or session boundaries.
- [tool] In `logs/diagnostics/20260324-062529-pose-history.json`, every active sample still showed `gate_result="accepted"`, `gate_reason="OK"`, `solve_failed=false`, `clamped=false`, and `target_vs_applied ~= 0`.
- [self] Interpretation rule: when `target_vs_applied` stays essentially zero but `following_error` grows large, the controller-owned planning/IK path is behaving consistently and the remaining problem is execution/following lag or commanded-rate scaling, not solver drift.
- [tool] This session reached much larger commanded rates than the earlier validation sweep, including `vy=0.05 m/s` and `v_yaw=66.388 deg/s`; the worst observed following envelopes were about `15.11 mm` position, `9.82 deg` orientation, and `14.92 deg` max joint error.
