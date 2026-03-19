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
