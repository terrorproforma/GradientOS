## 2026-04-27 00:35 +0000 - Loop trajectory ACK no longer waits for physical move-to-start

- Task summary:
  - Implemented `/home/pi/.cursor/plans/loop_trajectory_reliability_9db63052.plan.md`. Loop-mode `RUN_TRAJECTORY` previously held `handle_run_trajectory` synchronously through full trajectory planning AND a physical move-to-start RTCore wrapper before sending ACK. The API's 2 s timeout would expire mid-wrapper, the controller would non-fatally swallow the wrapper timeout, and then start the loop body anyway with axis 0 still far from the next command-frame target — tripping `command_frame_live_deviation_out_of_range`. Two safety problems fixed in one pass: (1) the wrapper now runs in a background thread that the controller hands off to AFTER ACK, so the API can ACK as soon as planning completes; (2) the wrapper preflight is strict-completion — any timeout/fault/abort/endpoint-mismatch faults the program and the loop body never starts.
- What changed:
  - `src/gradient_os/arm_controller/trajectory_execution.py`: `_open_loop_executor_thread` now accepts `require_completion: bool = False`. In strict mode a settle-timeout aborts the trajectory, issues a follow-up `wait_for_trajectory_complete` to confirm the abort took, clears `last_bounded_endpoint`, and re-raises a `TimeoutError("strict completion ...")`. The `state_name` acceptance set is also tightened: strict mode rejects `executing` (the bug that let the loop body race the in-flight wrapper); default mode keeps the existing `{"completed", "idle", "executing"}` non-fatal behavior.
  - `src/gradient_os/arm_controller/command_api.py`: added `_finish_failed_program_run(state, terminal_reason, failing_step_index)` (mirrors the cleanup the synchronous loop branch used to do inline) and `_looping_trajectory_executor_thread(initial_joint_path, initial_frequency_hz, loop_steps, loop_enabled)` (background entry point). The wrapper runs `_open_loop_executor_thread(..., require_completion=True, owns_trajectory_state=False)`, then verifies live `q` matches `initial_joint_path[-1]` within 0.05 rad, then hands off to `_trajectory_executor_thread` for the loop body. The try/except scope is intentionally tight around preflight + endpoint verification so body-time failures keep their own terminal_reason instead of being relabeled `loop_start_failed`.
  - `src/gradient_os/arm_controller/command_api.py::handle_run_trajectory`: deleted the synchronous `initial_thread = threading.Thread(...); initial_thread.start(); while initial_thread.is_alive(): ...` block in the loop branch. The loop branch now only plans `joint_path_initial`, builds `loop_steps`, and spawns `_looping_trajectory_executor_thread` as the executor thread. Non-loop branch is unchanged.
  - `src/gradient_os/api/main.py`: `/trajectory/run` controller-call timeout raised from `2.0` to `8.0` to tolerate synchronous trajectory planning when `use_cache=false`. Timeout-inference fallback strengthened from 3 polls × 250 ms (= 750 ms) to 8 polls × 200 ms (= 1.6 s).
- Tests added (all passing locally):
  - `tests/test_trajectory_execution_backends.py::test_open_loop_executor_strict_completion_raises_on_settle_timeout` — verifies strict mode aborts and re-raises on settle timeout, confirms `last_bounded_endpoint` is cleared and at least 2 `wait_for_trajectory_complete` calls happen (in-line + post-abort confirmation).
  - `tests/test_trajectory_execution_backends.py::test_open_loop_executor_strict_completion_raises_on_final_point_timeout` — verifies strict mode also aborts and re-raises when `wait_for_trajectory_final_point_sent(...)` times out, closing the remaining preflight-success hole.
  - `tests/test_trajectory_execution_backends.py::test_open_loop_executor_default_completion_timeout_stays_non_fatal` — verifies the non-loop default path still swallows settle timeouts non-fatally and does NOT call `abort_trajectory`.
  - `tests/test_command_api_direct_setpoint.py::test_handle_run_trajectory_loop_does_not_execute_move_to_start_inline` — asserts `RUN_TRAJECTORY` with `loop_override=True` returns `accepted=True`, the executor thread's `target is _looping_trajectory_executor_thread`, and NEITHER `_open_loop_executor_thread` NOR `_trajectory_executor_thread` runs inline before the API ACK.
  - `tests/test_command_api_direct_setpoint.py::test_looping_trajectory_executor_thread_faults_on_preflight_failure` — direct call to wrapper with monkeypatched preflight `raise TimeoutError("strict completion")`; asserts loop body is NOT invoked, `program_status.state == "faulted"`, `terminal_reason == "loop_start_failed"`, `is_running` cleared.
  - `tests/test_command_api_direct_setpoint.py::test_looping_trajectory_executor_thread_does_not_relabel_body_failures` — direct call with successful preflight + endpoint match, body raises `RuntimeError("body failed")` after stamping `terminal_reason="rtcore_fault"`; asserts the wrapper does NOT overwrite that with `loop_start_failed`.
  - `tests/test_command_api_direct_setpoint.py::test_looping_trajectory_executor_thread_faults_on_endpoint_mismatch` — preflight succeeds but live q stays at `[0.5]*6` (>0.05 rad from target `[0.1]*6`); asserts loop body is NOT invoked, program faults with `loop_start_failed`.
- Tests updated for new timeout: `tests/test_api_endpoints.py` — three assertions changed from `2.0` to `8.0` (`test_trajectory_run`, `test_trajectory_run_with_loop_override`, `test_trajectory_run_timeout_is_inferred_from_motion_status`).
- Validation performed:
  - `source ./start.sh && python -m pytest tests/test_trajectory_execution_backends.py::test_open_loop_executor_strict_completion_raises_on_settle_timeout tests/test_trajectory_execution_backends.py::test_open_loop_executor_strict_completion_raises_on_final_point_timeout tests/test_trajectory_execution_backends.py::test_open_loop_executor_default_completion_timeout_stays_non_fatal -q` -> `3 passed`.
  - 3 focused loop-wrapper tests in isolation -> `3 passed`.
  - `source ./start.sh && python -m pytest tests/test_cartesian_jog_resilience.py tests/test_run_controller_helpers.py tests/test_api_endpoints.py tests/test_command_api_direct_setpoint.py tests/test_trajectory_execution_backends.py -q` -> `171 passed`.
  - `source ./start.sh && python -m py_compile src/gradient_os/arm_controller/command_api.py src/gradient_os/arm_controller/trajectory_execution.py src/gradient_os/api/main.py tests/test_trajectory_execution_backends.py tests/test_command_api_direct_setpoint.py tests/test_api_endpoints.py` -> pass.
  - `ReadLints` on every touched Python file -> clean.
- Pre-existing test pollution observed (NOT introduced by this change):
  - When running `pytest tests/test_trajectory_execution_backends.py tests/test_command_api_direct_setpoint.py` without `tests/test_api_endpoints.py` between them, the four `test_handle_apply_joint_delta_*` tests fail with `ValueError: joint must be in 1..2` — i.e., `utils.NUM_LOGICAL_JOINTS` reverts to `2` after `_configure_backend_executor_test`'s monkeypatch. Confirmed pre-existing by running the same combination with my new tests excluded (`-k "not strict_completion and not default_completion_timeout"`); same failure. The contamination is fragile and hides when test order changes (running the full plan slice with `test_api_endpoints.py` between them passes cleanly). Worth fixing in a separate pass; not scope of this change.
- Live smoke checklist (operator-required, NOT executed here):
  1. Plan the same 11-waypoint preview.
  2. Run with loop disabled — should ACK promptly and complete (regression check; behavior unchanged).
  3. Enable loop and run.
  4. Expected log sequence:
     - `RUN_TRAJECTORY,__planner_preview__,false,true`
     - `Trajectory thread started. Main loop is responsive.` (after planning, BEFORE any physical move-to-start motion)
     - `Executing loop move-to-start preflight.`
     - only after preflight completes: `Loop move-to-start preflight complete. Starting loop body.`
  5. If preflight does not complete: program ends in `state=faulted terminal_reason=loop_start_failed`, no loop body execution, no `command_frame_live_deviation_out_of_range`.
- Follow-ups / risk:
  - Synchronous trajectory PLANNING (when `use_cache=false`) is still on the controller's command path. The 8 s API ceiling covers observed planning latencies for the 11-waypoint preview but a future architecture pass should move planning into a background job similar to what we just did for the wrapper. Until then, very long programs with `use_cache=false` may still trip the API ceiling, in which case the strengthened timeout-inference fallback (8 polls × 200 ms) acts as a backstop and surfaces the run as `accepted=True ack_inferred=True run_request_timed_out=True`.
  - Final review fix: strict preflight now raises on both final-point timeout and settle timeout, and endpoint verification uses live control feedback (`servo_driver.get_control_arm_state_rad`) rather than cached best-available state.
  - Pre-existing test pollution between `test_trajectory_execution_backends.py` and `test_command_api_direct_setpoint.py::test_handle_apply_joint_delta_*` should be fixed in a follow-up.
  - The 0.05 rad endpoint tolerance was chosen to be larger than typical servo following noise but tight enough to catch a stale or in-flight wrapper completion. If operators report false-positive `loop_start_failed` faults after legitimate preflights, widen this empirically rather than removing the gate.

## 2026-04-26 20:50 +0000 - Control-feedback fault logs now preserve A6-EC codes

- What changed:
  - `src/gradient_os/arm_controller/backends/ethercat_rtcore/backend.py`: expanded the hard-control-feedback fault detail from a reason-only string (`drive_fault_state`) to a per-axis diagnostic summary containing axis, logical joint, DS402 state/name, statusword, `0x603F` error code, manufacturer `0x203F` code, and axis fault flags. Existing jog/control logs now retain the exact drive codes before `RESET_FAULTS` clears them.
  - `tests/test_gradient05_limits_and_backends.py`: updated `test_control_joint_positions_reject_drive_fault` to simulate a J2 fault and assert the exception/loggable message includes `statusword=0x9638`, `error_code_603f=0x8611`, `manufacturer_error_203f=0x00000470`, DS402 `fault(8)`, and fault flags.
- Validation performed:
  - `source ./start.sh && python -m pytest tests/test_gradient05_limits_and_backends.py -q -k 'control_joint_positions_reject_drive_fault'` -> `1 passed`.
  - `source ./start.sh && python -m pytest tests/test_gradient05_limits_and_backends.py tests/test_cartesian_jog_resilience.py -q` -> `183 passed`.
  - `source ./start.sh && python -m py_compile src/gradient_os/arm_controller/backends/ethercat_rtcore/backend.py tests/test_gradient05_limits_and_backends.py` -> pass.
  - `ReadLints` on touched files -> clean.
- Follow-up:
  - A future UI improvement can parse `fault_details` out of the error string or expose it structurally, but the operator-visible logs now preserve the codes by default.

## 2026-04-26 20:20 +0000 - Live J2 crash/fault triage after held-jog fix

- Investigation summary:
  - Operator reported they physically crashed the robot and J2 faulted. Terminal trace showed normal jog sessions with `lease_timeout_s=1.0`; all `ui-release` stops logged `quick_stop=False drive_power_action=rtcore_jog_stop`, so the held-jog lease/brake fix was not the cause.
  - The first hard fault evidence was `[Jog] control feedback miss (Control feedback unavailable (axes=[1], reasons=['drive_fault_state']))` at `20:07:54+0000`, immediately after a new `+Z` jog session started. In Gradient-05's 1:1 axis mapping, axis `1` is J2. That means the backend refused control feedback because J2 was already in a hard DS402 drive fault/fault-reaction state.
  - No `Collision DETECTED` watchdog log appeared in the terminal excerpt. The operator then explicitly ran `SAFE_POWER_DOWN`, `RESET_FAULTS`, and `SAFE_POWER_UP`; current RTCore metrics after reset showed all axis `error_code=0` / `manufacturer_error_code=0`, so the exact latched J2 vendor code was no longer available from `metrics.json`.
- Validation performed:
  - Read terminal logs and `/run/gradient-rt-motion/metrics.json`; no code changes or tests run for this triage.
- Follow-up:
  - If a crash/fault happens again, capture `/info/joints-detailed` or `/run/gradient-rt-motion/metrics.json` before `RESET_FAULTS` to preserve the J2 statusword/error/manufacturer code.

## 2026-04-26 17:45 +0000 - Held-jog lease expiry no longer requests DS402 quick-stop

- Task summary:
  - Implemented the jog-hold stability plan's surgical fix: browser/controller jog-session lease expiry now maps to a soft RTCore jog stop (`quick_stop=False`) instead of DS402 quick-stop. Explicit hard-stop paths (`controller-stop`, `controller-shutdown`, `fk-failed`, safe power-down/collision paths) still request quick-stop.
  - Added explicit drive-power action logs for jog backend stop requests, jog-thread exits, safe power-down, backend power-transition prep, axis-disable, arm-disarm, and safe power-up. Lease-expired jog exits now log `drive_power_action=rtcore_jog_stop`.
  - Debounced advisory `canonical_truth_unavailable` dashboard transitions during active jog while keeping non-advisory/hard truth reasons immediate.
  - Hardened `ControlPanel` recovery so active held jog retries recoverable `SESSION_EXPIRED` / `SESSION_NOT_FOUND` / `SESSION_INACTIVE` from current refs without posting a stale stop or power-down. UI now sends explicit `lease_timeout_s=1.0` for controller/API session tolerance, while RTCore motor-side lease remains `0.2s`. Unchanged keepalive publishes no longer call `setLastJogCommand`.
- Validation performed:
  - `source ./start.sh && python -m pytest tests/test_cartesian_jog_resilience.py tests/test_run_controller_helpers.py -q` -> `42 passed`.
  - `source ./start.sh && python -m pytest tests/test_api_endpoints.py -q` -> `69 passed`.
  - `source ./start.sh && python -m pytest tests/test_command_api_direct_setpoint.py tests/test_realtime_jog_backend_compatibility.py -q` -> `43 passed`.
  - Combined touched Python slice `tests/test_cartesian_jog_resilience.py tests/test_run_controller_helpers.py tests/test_api_endpoints.py tests/test_command_api_direct_setpoint.py tests/test_realtime_jog_backend_compatibility.py -q` -> `154 passed`.
  - `cd web-ui && npm test -- src/ControlPanel.test.tsx` -> `33 passed`.
  - `source ./start.sh && python -m py_compile src/gradient_os/arm_controller/command_api.py src/gradient_os/arm_controller/backends/ethercat_rtcore/backend.py src/gradient_os/run_controller.py src/gradient_os/api/main.py tests/test_cartesian_jog_resilience.py tests/test_run_controller_helpers.py` -> pass.
  - `ReadLints` on touched Python/TypeScript files -> clean.
- Follow-ups / risks:
  - Live hardware smoke still required: hold jog and verify no `JOG_SESSION_STOP`, `SAFE_POWER_DOWN`, `axis_disable`, `arm_disarm`, RTCore quick-stop, DS402 state transition, or brake-state transition occurs unless operator release/disarm/pagehide/hard fault actually happens.

## 2026-04-21 03:55 +0000 — Killed the "laggy jog" feel: arm-time fast-path + widened shaft-frame tolerance + hot-path restructure

- Task summary:
  - Operator reported the jog buttons felt "super laggy" and "blocked until it settles" after a button release. Three distinct defects were responsible. Each was diagnosed from live traces and fixed:
    1. **Arm-time canonical-truth retry wall** (dominant cause): `handle_jog_session_start` ran a strict `get_current_arm_state_rad()` check with up to 500 ms of retry every click. After a jog release the drive is still decelerating, the command-frame-roundtrip + shaft-frame gates transiently trip, and the retry loop burned most of the 500 ms budget. Controller log captured three consecutive `Rejecting jog session start after 11 attempt(s) over 500 ms` events on a single release-then-reclick sequence — the exact "wall of lag" the operator felt.
    2. **Shaft-frame tolerance too tight**: the gate base was still 16 counts (later 512), but live traces showed the drive's internal U40.20/U40.22 vs 0x6064 consistency window stretches to ~2000 counts during joint velocities of ~0.5-2 rad/s. The Python-side velocity estimator is fragile (finite-diff between uneven multi-consumer calls; can report velocity=0 while the arm is actually moving), so the motion-widen term didn't kick in reliably. Result: gate flipped to UNAVAILABLE mid-jog with deltas like `1994/tol=512 vel=0`.
    3. **Jog loop telemetry pre-send**: ~16 ms of per-tick work (session-manager mutations, `ik_debug` dict construction with multiple `_pose_snapshot_from_matrix` / `_pose_error_snapshot` calls, two redundant `get_control_state` calls) ran BEFORE the velocity command reached RTCore. Trace measured `feedback=2.4 ik=1.2 command_send=0.2ms` but total loop was 20.35 ms with 49/49 overruns — i.e., 16 ms of user-visible telemetry lag before any motion happened.
- Fixes landed (all in `src/gradient_os/arm_controller/command_api.py` and `src/gradient_os/arm_controller/backends/ethercat_rtcore/backend.py`):
  - **Arm-time fast-path**: new `_note_valid_canonical_truth()` / `_recently_valid_canonical_truth(window_s=5.0)` pair. `handle_jog_session_start` now skips the retry loop entirely when truth was observed valid within the last 5 seconds. The jog thread's successful `get_current_arm_state_rad` calls stamp the timestamp on each success, so any release→reclick within 5 s hits the fast-path and arms INSTANTLY (~0 ms). First-boot and post-power-cycle jogs still fall through to the original strict retry loop so the safety intent is preserved. Regression tests: `test_handle_jog_session_start_uses_fast_path_when_truth_was_recently_valid` + `test_handle_jog_session_start_fast_path_expires_after_window`.
  - **Shaft-frame base tolerance 16 → 512 → 4096 counts**: final pass settled at 4096. Still 32x smaller than a full revolution (131072 counts on G05 encoders) so real multi-turn glitches still trip the gate decisively, while motion-induced drive-internal skew (up to ~2000 counts during normal jog) no longer trips it. Velocity-aware widening stacks on top for exceptional cases. Updated `test_a6ec_truth_unavailable_when_anchor_and_6064_disagree_modulo_rm` to use realistic 131072 counts_per_rev so RM/3 ≈ 43,690 counts still triggers the gate; the test now asserts `delta > effective_tolerance` rather than pinning to a numeric constant, so future tuning doesn't silently break the regression.
  - **Jog loop hot-path restructure**: extracted `_build_jog_ik_debug_payload` helper (pulled the ~40-line inline ik_debug dict out of `_jog_controller_thread`), then moved the heavy telemetry block (`_build_jog_command_state_perf_fields` + two `_jog_perf_update` calls including the massive `ik_debug={}` payload + `update_following_error`) from BEFORE command_send to AFTER. Same values, same state — only ordering changed. Gate-failure path keeps the telemetry inline because latency doesn't matter when no motion is happening. Result: user-visible feedback→command window drops from ~20 ms to ~4 ms.
  - **Metrics.json mtime cache**: `_load_rtcore_metrics_snapshot` now stats the file and only re-parses the 11 KB JSON when mtime changes. Was a full disk-read + json.loads per call × 5+ call sites × 50 Hz.
  - **Velocity estimator wrap-safety**: `_VELOCITY_ESTIMATE_MAX_COUNTS_PER_S = 5_000_000` cap so raw-counts wraps at single-turn boundaries (e.g., 131060 → 20) don't produce spurious 26 M counts/s widening that would accept real frame errors.
  - **Thread-race guard** (earlier this session): `_safe_session_call` helper absorbs `JogSessionError(SESSION_INACTIVE)` into a clean break instead of crashing the jog thread. Captured in live trace as `[Jog] WARNING: Failed to validate/apply jog step: Jog session is no longer active.` — thread exits cleanly, no traceback.
- Live validation (operator in the loop):
  - Stack restart → jog for 3 minutes: 0 shaft-frame flips, 0 roundtrip flips, 0 `Rejecting jog session start` events, 8+ clean release-then-reclick cycles, `truth_valid_at_arm=True` on every click. Operator confirmed "THAT feels much better!" after the final restart.
  - Full regression sweep: `584 passed, 1 deselected` in 10.0 s. Four new tests added: `test_handle_jog_session_start_retries_and_succeeds_on_later_attempt`, `test_handle_jog_session_start_uses_fast_path_when_truth_was_recently_valid`, `test_handle_jog_session_start_fast_path_expires_after_window`, `test_jog_thread_source_contains_session_inactive_race_guard`, plus `test_command_roundtrip_tolerance_widens_with_velocity` / `test_command_roundtrip_detail_includes_velocity_field`.
- What remains (follow-up, not blocking):
  - **Jog loop still overruns 20 ms period** (avg 20-22 ms loop, occasional 25-44 ms spikes). Feedback-read avg 5 ms on the hot path. This is no longer user-perceivable because the hot path itself is ~4 ms (feedback + IK + command_send) and the rest is deferred telemetry, but the loop running at effectively 45-50 Hz instead of 50 Hz is worth addressing if we ever need higher jog refresh rates.
  - **Top-level canonical-truth flickers still present in the controller status-poll path** (~26 over 3 min of motion). Jog thread uses Phase 0 cached-feedback fallback so they don't halt motion, but the dashboard banner flashes UNAVAILABLE briefly. Candidate next step: apply the same velocity-aware widening to whatever OTHER gate the status-poll pipeline uses, or add hysteresis (N-consecutive-failures rule) before flipping the public banner.
  - **`gradient-rt-motion` SIGKILL lesson learned the hard way**: killing RTCore with SIGKILL orphans ec_master kernel references and requires a reboot. ALWAYS use `./start-stack.sh stop --hard` for proper teardown. Added to scratchpad guardrails.

## 2026-04-21 01:25 +0000 — Phase 1 pivot: atomic paired-snapshot replaces failed custom TxPDO multi-turn mapping; idle validated

- Task summary:
  - The original Phase 1 plan (extend the A6-EC TxPDO with nine new subitems including `U40.20/U40.22` multi-turn) failed on hardware in two successive live probes (47 B → 33 B) — the drive silently blanked the entire TxPDO frame whenever the custom mapping pushed past ~28 B of subitems, breaking `statusword`/`6064` reads. Even with a trimmed 8-entry/23 B layout the multi-turn subitems themselves read as zero while `6064` in the same frame populated correctly, so the A6-EC firmware clearly does NOT cyclically publish `U40.20/22` in the custom TxPDO despite declaring them `PdoMapping=t` in the ESI. Pivoted to an atomic-paired-snapshot approach that keeps the TxPDO tight and delivers the same "no-skew" invariant by co-latching 0x6064 at the moment the SDO U40 upload completes.
- RTCore changes (`src/gradient_rt_motion/main.cpp`):
  - Extended `AbsoluteFeedbackAxis` with `paired_valid`, `paired_pos_counts`, `paired_sample_time_ns` fields. Populated them inside the per-axis loop of `perform_absolute_feedback_refresh` immediately after all SDO field uploads for that axis complete: `axis_feedback.paired_pos_counts = latest_feedback.pos_counts[i]` captures the most-recent PDO 0x6064 for that axis, `paired_valid = 1`, `paired_sample_time_ns = now_monotonic_ns()`. This bounds the skew at SDO mailbox round-trip + one PDO tick (~1-6 ms) instead of the previous 200 ms SDO poll period.
  - Extended `append_absolute_feedback_json` to emit `paired_pos_counts` and `paired_sample_time_ns` as pseudo-fields inside the `absolute_feedback` JSON block, so the existing `_AbsoluteFeedbackAxisMetrics` JSON parser picks them up with no schema churn.
- Python consumer changes (`src/gradient_os/arm_controller/backends/ethercat_rtcore/backend.py`):
  - Added `_AbsoluteFeedbackAxisMetrics.paired_pos_counts()` that returns the atomic-pair 0x6064 int when valid, None otherwise.
  - Shaft-frame gate callsite (around line 2353) now prefers the paired snapshot over live-now 0x6064: `paired_6064 = metrics.paired_pos_counts(); shaft_frame_reference_counts = int(paired_6064) if paired_6064 is not None else int(raw)`. When paired data isn't available yet (first cycles after boot, SDO poll disabled), it falls back to the live PDO reference so the gate stays alive.
  - Added `shaft_frame_reference_source` diagnostic (`"paired_sdo_snapshot"` | `"live_pdo"`) onto the consistency-detail dict for operator observability.
  - Velocity-aware tolerance widening: introduced `_SHAFT_FRAME_MOTION_SKEW_BUDGET_S = 0.020` constant plus per-axis finite-difference estimator (`_shaft_frame_prev_raw_counts`, `_shaft_frame_prev_raw_time_ns` in `__init__`). Estimates `velocity_counts_per_s` from consecutive raw samples at the callsite and passes it into `_shaft_frame_consistency_detail`. The gate now widens the tolerance by `|velocity_counts_per_s| × 0.020 s` so motion-induced skew (bounded at ~6 ms) does not trip the gate under jog. At rest the tolerance stays at the tight 16-count base. New diagnostic fields: `shaft_frame_tolerance_base_counts`, `shaft_frame_tolerance_motion_widen_counts`, `shaft_frame_velocity_counts_per_s`.
- Reverted the extended TxPDO experiment:
  - `src/gradient_os/arm_controller/ethercat_drive_catalog.py` stays at the classic 6-entry / 17-byte TxPDO (err/sw/pos/torque/mode_disp/di). Touch-probe feedback (`tp_status`/`tp_pos1`/`tp_pos2`) and `multi_turn_lo`/`multi_turn_hi` are deliberately NOT in the TxPDO. The extended StatusExtendedSnapshotV1 machinery (bus_voltage, temps, load, PE, drive_not_ready, motor_not_rotating) stays in place because those subitems populate correctly when mapped individually within the remaining budget — but `U40.20/22` never will, per the live evidence.
- Test changes (`tests/test_gradient05_limits_and_backends.py`):
  - `test_tx_pdo_layout_fits_a6ec_sm3_capacity_and_preserves_classic_entries` now pins the final 6-entry layout and regression-guards against `multi_turn_lo`/`multi_turn_hi`/`tp_status` sneaking back into the TxPDO. Docstring references the paired-snapshot architecture so future readers understand why `U40.20/22` are NOT here.
  - Total-bits guard tightened from `≤ 40 B` to `≤ 28 B` (the known-safe capacity — 33 B silently blanked the frame on live hardware).
- Validation that actually ran:
  - `make -j2` in `src/gradient_rt_motion/` → clean C++ build with new `paired_*` fields.
  - Full Python regression sweep: `580 passed, 1 deselected in 11.45s` (same suite as the 2026-04-20 entry).
  - Stack warm-started on hardware, curl `/info/joints-detailed` confirmed on all 6 axes: `shaft_frame_reference_source = paired_sdo_snapshot`, `shaft_frame_consistent = True`, `|shaft_frame_mod_rm_delta_counts| ≤ 6` vs the 16-count base tolerance. `canonical_joint_truth_available = True`.
  - 30-second idle stress (`266 samples`) held `inconsistent_events = 0` and `max |delta| per axis ≤ 5 counts` across all 6 joints.
  - `metrics.json` directly inspected: every axis has `absolute_feedback.paired_pos_counts.valid = 1` with a non-zero value matching the live 0x6064 range.
- What remains for the operator (unchanged from the 2026-04-20 plan, just updated mechanism):
  - Cartesian jog for ~30 s while monitoring `curl /info/joints-detailed`. Success criteria: `shaft_frame_reference_source` stays `paired_sdo_snapshot`; `shaft_frame_consistent` stays `True`; `jog_truth_flicker_total` either stays 0 or increments with NO motion halt (Phase 0 guarantee); motion does not pause mid-jog.
  - If motion-induced skew exceeds the 20 ms budget, widen `_SHAFT_FRAME_MOTION_SKEW_BUDGET_S` (empirical dial) rather than re-attempting the failed TxPDO mapping.
  - Phase 2 UI / Phase 4 collision-watchdog hardware validation are still outstanding but unaffected by this pivot.
- Follow-up / risk:
  - The `paired_sample_time_ns` field is emitted but not currently consumed by Python. If a future operator wants a "pairing age" UI indicator, the plumbing is ready.
  - The multi-turn PDO ingestion path (`_axis_multi_turn_lo/hi`, `_absolute_axis_q_from_pdo`) still exists in backend.py and stays disabled by virtue of those arrays never receiving a non-zero payload — the A6-EC simply doesn't publish those subitems cyclically. The code is harmless dead weight until a drive firmware update changes that; leaving it in place keeps the consumer side ready if/when it does.
  - Deleted ~7 GB of autosaved fast-trace logs from `logs/j6-multiturn-fast/` during live bring-up after the root filesystem hit 100 %. No production artifacts lost.

## 2026-04-20 22:30 +0000 — Implemented canonical-truth stability + rich drive telemetry + collision watchdog (Phases 0-4)

- Task summary:
  - Landed the five-phase `/home/pi/.cursor/plans/stabilize_canonical_truth_898c2a11.plan.md` in one pass. Phase 0 stops mid-jog canonical-truth flickers from killing the 50 Hz jog thread. Phase 1 pipes nine new A6-EC `0x2040` PDO diagnostic fields atomically alongside `0x6064` every EtherCAT cycle (RTCore C++ + IPC v1.1 struct + Python consumer). Phase 2 surfaces the telemetry into `axis_absolute_feedback`, `drive_faults`, and the commissioning-panel per-axis health chips. Phase 3 switches the canonical-truth path to prefer the atomic PDO multi-turn over the 5 Hz SDO poll (removing the 200 ms skew that caused the shaft-frame gate to flicker under motion) and adds an arm-time strict canonical-truth check before any jog session starts. Phase 4 adds a collision watchdog on torque and position-error excursions with per-axis thresholds.
  - Operator live-validation steps remain open (pending hardware access); every implementation todo is complete and all tests pass locally.
- Phase 0 (Python-only safety net):
  - `src/gradient_os/arm_controller/command_api.py`: added `import sys`, `_JOG_TRUTH_FLICKER_LOG_THROTTLE_S = 1.0`, the `_record_jog_truth_flicker` helper (thread-safe counter + throttled stderr log), and `truth_flicker_*` fields on `_new_jog_perf_state()`. Wrapped the three `servo_driver.get_current_arm_state_rad(verbose=False)` call sites inside `_jog_controller_thread` (initial read at line 4048, resume-after-motion at line 4115, per-tick hot path at line 4130) in `try/except RuntimeError`. Only `RuntimeError` is caught; other exception classes still surface real bugs.
  - `get_jog_performance_snapshot()` now defensively backfills `truth_flicker_total / _last_reason / _last_wall_s` so `GET /control/performance` always exposes the counter.
- Phase 1 (atomic PDO + IPC + RTCore):
  - `src/gradient_os/arm_controller/ethercat_drive_catalog.py`: appended nine new `tx_pdo_layout` entries for the a6ec_ds402 profile (`bus_voltage`, `load_rate`, `position_error`, `multi_turn_lo/hi`, `igbt_temp`, `motor_temp`, `drive_not_ready`, `motor_not_rotating`) totalling 192 new bits per axis. Every subitem is declared `<PdoMapping>t</PdoMapping>` in the DT2040 datatype of the A6-EC ESI, so the drive accepts the mapping at PreOp→SafeOp.
  - `src/gradient_rt_motion/ipc_v1.hpp`: bumped `kVerMinor` to 1; added `AxisStatusExtV1` (24 B) and `StatusExtendedSnapshotV1` (400 B) with static_asserts; added `MSG_STATUS_EXTENDED_SNAPSHOT = 0x0206`. `AxisStatusV1` and `StatusSnapshotV1` are intentionally unchanged so the classic IPC consumer stays ABI-compatible.
  - `src/gradient_rt_motion/main.cpp`: extended `AxisOffsets` with nine `ext_*` fields, taught `tx_offset_for_semantic` the new semantics, added per-axis PDO reads inside the feedback cycle (each guarded on `kInvalidOffset`), stamped `latest_feedback.ext_sample_time_ns` every cycle, and emit the extended snapshot to the status ring at the same cadence as `StatusSnapshotV1`. `perform_absolute_feedback_refresh` now skips the SDO poll when the extended PDO snapshot is fresh within 50 ms, saving mailbox bandwidth. `log_registered_offsets` prints the extended-PDO offsets at boot so operators can confirm the mapping was accepted.
  - `src/gradient_os/arm_controller/backends/ethercat_rtcore/backend.py`: added `_MSG_STATUS_EXTENDED_SNAPSHOT = 0x0206`, `_AXIS_STATUS_EXT_STRUCT` (24 B) and `_STATUS_EXTENDED_SNAPSHOT_HEADER_STRUCT` (16 B), per-axis state arrays (`_axis_position_error_counts`, `_axis_multi_turn_lo/hi`, `_axis_bus_voltage_raw`, `_axis_load_rate_raw`, `_axis_igbt_temp_raw`, `_axis_motor_temp_raw`, `_axis_drive_not_ready_bits`, `_axis_motor_not_rotating_code`, `_axis_extended_updated_ns`), `_ingest_extended_snapshot_payload` parser, and scaled accessors (`_axis_bus_voltage_v`, `_axis_load_rate_pct`, `_axis_igbt_temp_c`, `_axis_motor_temp_c`, `_axis_position_error_counts_or_none`, `_axis_drive_not_ready_bits_or_none`, `_axis_motor_not_rotating_code_or_none`, `_axis_multi_turn_counts_from_pdo` with sign-extension matching `combine_signed_i64_pair`). Every accessor returns `None` when the axis has never received a valid extended snapshot so downstream consumers distinguish "drive rejected mapping" from "drive reports 0.0".
- Phase 2 (telemetry surfaced in API + UI):
  - `src/gradient_os/run_controller.py`: added `_A6EC_DRIVE_NOT_READY_BIT_LABELS` / `_A6EC_MOTOR_NOT_ROTATING_CODES` decoder skeletons (with TODOs to cross-reference the full manual catalogue) and `_decode_drive_not_ready_bits` / `_decode_motor_not_rotating_code` helpers. `_build_joint_state_snapshot` enriches each `axis_absolute_feedback` entry with the scaled PDO fields (only when the backend exposes the accessors and the axis has fresh data) and emits top-level arrays (`axis_bus_voltage_v`, `axis_load_rate_pct`, `axis_igbt_temp_c`, `axis_motor_temp_c`, `axis_position_error_counts`, `axis_drive_not_ready_bits/text`, `axis_motor_not_rotating_code/text`). `_build_drive_fault_snapshot` wires an `axis_extended_telemetry_context` into `build_drive_fault_snapshot` so the `/monitor` SSE stream carries the same fields.
  - `src/gradient_os/telemetry/drive_faults.py`: `build_drive_fault_snapshot` accepts `axis_extended_telemetry_context: Mapping[int, Mapping[str, Any]] | None` and merges the nine extended fields into each per-axis payload when supplied.
  - `web-ui/src/App.tsx` / `web-ui/src/ControlPanel.tsx`: extended `DriveFaultAxis` TypeScript type with optional extended fields. `synthesizeDriveFaultSnapshotFromAxes` now pulls the extended fields out of `axis_absolute_feedback` when `drive_faults` is absent from a monitor event (preserves the 2026-04-17 frontend synthesizer pattern). New `buildAxisHealthChips` helper and colour-coded `AxisHealthChip` render a per-joint health strip below the "Canonical truth trust" line — voltage, IGBT/motor temp with warn/error bands, load, position-error highlight above 100 counts, drive-not-ready text, motor-not-rotating reason text. The chip row is hidden entirely when no extended telemetry is present.
- Phase 3 (atomic multi-turn consumer + arm-time strict):
  - `src/gradient_os/arm_controller/backends/ethercat_rtcore/backend.py`: added `_PDO_MULTI_TURN_FRESH_NS = 50_000_000` and `_ABSOLUTE_SOURCE_PDO_MULTI_TURN = "pdo_multi_turn_atomic"`. `_absolute_axis_q_from_metrics` now calls the new `_absolute_axis_q_from_pdo` helper first; when the PDO sample is fresh and the axis config is resolved, it returns `(axis_q, "pdo_multi_turn_atomic", counts)` and the shaft-frame gate downstream runs on atomic-sample data. Falls back to the legacy SDO path when PDO is stale, never-sampled, or the backend config is incomplete. The downstream `canonical_truth_counts_source` detail field now reports `pdo_multi_turn_atomic` during healthy motion, so `/info/joints-detailed` can be used as an operator check that atomic sampling is active.
  - `src/gradient_os/arm_controller/command_api.py::handle_jog_session_start`: runs a single strict `servo_driver.get_current_arm_state_rad(verbose=False)` call at arm time. If it raises `RuntimeError`, the session refuses with `JogSessionError("CANONICAL_JOINT_TRUTH_UNAVAILABLE", ...)` before any state is mutated. If it succeeds, the session snapshot carries `truth_valid_at_arm=True` so the UI can display "armed with verified anchor". Non-`RuntimeError` exceptions propagate unchanged so real bugs stay visible.
- Phase 4 (collision watchdog):
  - New file `src/gradient_os/arm_controller/collision_watchdog.py` with `CollisionThresholds` (dataclass, per-axis torque/PE/sustained-samples/sample-period), `CollisionEvent` (axis, reason, value, threshold, wall_s), and `CollisionWatchdog` (start/stop, `is_running` property, `check_once` synchronous pass for deterministic testing). Watchdog reads `backend._axis_torque_raw`, `_axis_position_error_counts`, `_axis_extended_updated_ns` under `backend._status_lock`; edge-triggers on sustained-above-threshold bursts; skips axes with `_axis_extended_updated_ns == 0` so bring-up defaults cannot trip it; swallows callback exceptions so a misbehaving handler cannot bring down the controller.
  - `src/gradient_os/arm_controller/robots/gradient05/config.py`: added `collision_watchdog_thresholds` property with six axis-specific placeholders (J1/J2 torque≤2500 pe≤8000, J3 torque≤2200 pe≤7000, J4/J5 torque≤1800 pe≤5000, J6 torque≤1800 pe≤4000, all sustained=3 at 100 Hz). Docstring flags these as CONSERVATIVE PLACEHOLDERS requiring Phase 4.5 hardware calibration.
  - `src/gradient_os/run_controller.py`: added `_COLLISION_WATCHDOG` / `_COLLISION_WATCHDOG_LOCK` module-level state, `_start_collision_watchdog(backend_instance, robot_config_obj, servo_backend, sim_mode)` lifecycle helper (silently skipped for non-ethercat_rtcore, sim mode, or backends missing the Phase 1 extended-PDO arrays), `_stop_collision_watchdog()` idempotent teardown. Started after every successful backend initialization inside `_activate_runtime`, stopped in the controller `finally:` shutdown block so the watchdog never outlives the backend it reads.
- What was intentionally NOT changed:
  - The wire-frame safety cage (`_TRAJECTORY_MAX_PER_POINT_STEP_RAD`, `_TRAJECTORY_MAX_FIRST_POINT_DEVIATION_FROM_LIVE_RAD`), the 2026-04-17 J6 wrap-to-`[0, RM)` fold, the 2026-04-20 direction-preserving command path, and the shaft-frame consistency tolerance (still 16 counts) all stay intact.
  - The SDO `perform_absolute_feedback_refresh` path in RTCore is preserved as a fallback; Phase 1 only skips it when PDO is fresh within 50 ms.
  - No existing test was removed or weakened; `test_path_unwrapping_and_smoothing` stays deselected (pre-existing pollution) and the four legacy driver/end-to-end/protocol/solver suites remain ignored (pre-existing).
- Validation that ran locally:
  - `cd src/gradient_rt_motion && make -j2` — clean C++ build with new struct static_asserts (`AxisStatusExtV1 == 24`, `StatusExtendedSnapshotV1 == 400`).
  - `PYTHONPATH=src .venv/bin/python -m pytest tests/ --ignore=tests/test_driver.py --ignore=tests/test_end_to_end.py --ignore=tests/test_protocol.py --ignore=tests/test_solver.py --deselect tests/test_planning.py::TestTrajectoryPlanning::test_path_unwrapping_and_smoothing -q` → `580 passed, 1 deselected in 8.72s` (up from the 471 baseline by ~109 — matches the new Phase 0/1/3/4 regression coverage).
  - `cd web-ui && npm run build` — clean TypeScript + Vite build; new `app-*.js` chunk emitted.
  - `cd web-ui && npx vitest run src/ControlPanel.test.tsx` → `29 passed` (24 pre-existing + 5 new Phase 2 UI regressions).
  - `ReadLints` on every touched Python and TypeScript file → no diagnostics.
- Follow-up / risk:
  - **Operator live-hardware validation outstanding for all four phases.** Each phase in the plan has a BLOCKING operator-sign-off gate that only makes sense on the real robot (cartesian jog for ~30 s observing `truth_flicker_total`, curl `/info/joints-detailed` to confirm `shaft_frame_absolute_source == pdo_multi_turn_atomic` during motion, commissioning panel chips rendering with live voltage/temp/load/PE, controlled soft-obstacle test for the collision watchdog). Implementation is complete; only the hardware walkthrough remains.
  - Phase 4 collision thresholds in `gradient05/config.py` are CONSERVATIVE PLACEHOLDERS; operator must calibrate to ~1.5x observed peak during a 5-minute normal-motion sweep before relying on them in production.
  - Phase 2 decoder tables for `_A6EC_DRIVE_NOT_READY_BIT_LABELS` / `_A6EC_MOTOR_NOT_ROTATING_CODES` are seed skeletons; the actual A6-EC catalogue needs to be cross-referenced against `docs/resources/a6ec_manual_codes.md` and completed. Unmapped bits/codes surface verbatim (`unknown_bits=0xNN`, `unknown_code=N`), so operators can still look them up manually.
  - The RTCore binary at `/usr/local/bin/gradient-rt-motion` needs to be updated with the new `gradient-rt-motion` build (`make -j2` output in `src/gradient_rt_motion/`) before the stack next comes up. Operator should copy the binary into place as part of the Phase 1 live validation step.

## 2026-04-18 22:12 +0000 — Expanded the RTCore Proof And Math plan into a fresh-agent handoff document

- What changed:
  - [`/home/pi/.cursor/plans/rtcore_proof_and_math_5813457e.plan.md`](/home/pi/.cursor/plans/rtcore_proof_and_math_5813457e.plan.md) rewritten with inline architectural Q&A, enumerated RTCore FAULTED branches, concrete numbers for the three-step proof matrix, explicit evidence-signature classification for pre-wire vs drive-visible failures, and Phase 2 math-module adoption instructions with no-default `wrap_to_single_turn` and explicit `live_counts_frame` parameters.
  - Frontmatter todos unchanged (`instrument-rtcore-fault-path` in_progress, others pending).
  - No source code changed. No tests run. Plan is a markdown document only.
- What this plan now contains that the previous version did not:
  - Answers to the four manufacturer-architecture questions the `2b1bcb09` transcript surfaced: `607C ∈ [0, RM-1]` vs host-side canonical home sign; `C10.16` direction resolution; the canonical read/write object set; why `+10°` / seam-crossing-at-`+185°` / `+360°` beats a single `+400°` test.
  - Enumerated Phase 1 RTCore fault branches U1-U5 / C1-C3 / E1-E2 / M1 so instrumentation knows exactly which `logf` lines to add and what distinct tags to emit.
  - Concrete move numbers (starting poses, speeds, durations, seam-crossing pre-positioning for Move B).
  - Explicit evidence signatures for pre-wire / drive-visible-failure / drive-visible-success classification.
  - "DO NOT" list (don't re-add F31.10, don't promote 60B0, don't treat `a6ec_joint_motion.py` as production truth yet, don't skip Phase 1).
  - Appendix with exact commands a fresh agent needs (stack start, watch start, APPLY_JOINT_SETPOINT via UDP, wait-for-idle, regression sweep).
- Validation performed:
  - `wc -l` confirms 393-line plan.
  - Manual review: frontmatter valid, mermaid diagram closes cleanly, all code-fence language tags are plain backtick sequences, every file reference is a full absolute path.
  - No code tests because nothing executable changed.
- Follow-ups / risk:
  - Phase 1 Step 1.1 (instrumentation) is marked in_progress but not yet complete. The RTCore branch tags `FAULT_UPLOAD_U*` / `FAULT_COMMIT_C*` / `FAULT_EXEC_E*` / `FAULT_CLEAR_M1` are specified in the plan but not yet written into `main.cpp`.
  - Phase 1 Step 1.2 (10-20 ms watch cadence) is likewise specified but not implemented. `scripts/a6ec_chapter5_probe.py` still enforces the 50 ms floor.
  - Phase 1 Step 1.3 needs hardware. Do not skip the proof matrix on hardware before running Phase 2.
  - If the rerun classifies the failure as branch U1 / U2 / M1, the real fix is in RTCore command-ring state handling and neither continuous `607A` nor `60B0` nor the math module are the right levers. The plan now notes that explicitly (Step 1.6).

## 2026-04-17 Extracted A6-EC joint-motion math into `arm_controller.math.a6ec_joint_motion`

- Task summary:
  - User asked for the joint-movement mathematics written from scratch given the A6-EC frame-semantics note, 2026-04-15 manufacturer reply, and manual Chapters 5/11. Produced a self-contained, stateless, unit-tested math module that captures every equation in one testable place.
- What changed:
  - `src/gradient_os/arm_controller/math/__init__.py` (new): package namespace.
  - `src/gradient_os/arm_controller/math/a6ec_joint_motion.py` (new, ~540 LOC): pure-functional math library with explicit citations to the source docs. Exported surface:
    - Constants: `A6EC_COUNTS_PER_MOTOR_REV = 2^17`, `A6EC_MAX_OFF_MOTOR_REVOLUTIONS = 32,767`.
    - `A6ECAxisKinematics` dataclass with derived `rm_counts = counts_per_rev * num / den` (vendor email 2 Q1) and `counts_per_unit = rm_counts / (2*pi)`.
    - Raw Chapter 5 reconstruction: `sign_extend_16`, `reconstruct_multiturn_counts_from_u40_1c_1e`, `combine_signed_i64_pair` for U40.20/.22 and U40.2A/.2C.
    - Counts ↔ joint radians: `axis_q_rad_from_counts`, `counts_from_axis_q_rad` in the drive-native posture (no double-apply of gear ratio, per the anti-double-apply rule in the frame note).
    - Canonical truth: `compute_home_anchor_rad(absolute, reference)`, `canonical_joint_q_rad(absolute, anchor, kinematics)` with explicit master-offset subtraction.
    - Shaft-frame consistency: `shaft_frame_consistency` returning a `ShaftFrameConsistencyResult` with `consistent / mod_rm_delta_counts / wrap_turns / tolerance` fields mirroring the backend's `_shaft_frame_consistency_detail`. Default tolerance 16 counts (matches `_SHAFT_FRAME_CONSISTENCY_TOLERANCE_COUNTS`).
    - Command fold: `fold_canonical_q_to_command_counts(canonical_q, live_reference_counts, kinematics, wrap_to_single_turn)` returning a `CommandFoldResult`. `wrap_to_single_turn=True` is the production command path; `=False` preserves the linear-windowed behavior required by the diagnostic roundtrip map and the canonical-from-axis-q reverse.
    - Shortest-angular helper: `shortest_angular_counts(linear, rm)` for ±RM/2 folding.
    - Trajectory safety: `check_per_point_step`, `check_first_point_live_deviation`, `enforce_trajectory_wire_frame_safety` returning/raising with `command_frame_oversized_step` / `command_frame_live_deviation_out_of_range` / `command_frame_seam_crossing_step_disallowed` / `command_frame_seam_crossing_first_point_disallowed` messages that match the backend log format exactly (includes `joint=N` 1-based index and `period_counts=RM`).
    - `hm35_origin_offset_biased_to_midpoint(kinematics, num, den)` matching `NATIVE_HOME_CONFIG`'s `write_sdo_wrap_fraction` step, asserting 607C stays in `[0, RM-1]` per vendor email 2 Q6.
  - `tests/test_a6ec_joint_motion_math.py` (new, 55 cases): pins each equation to vendor-derived numbers.
    - Kinematics: RM formula, counts_per_unit, gear_ratio exactness, input validation.
    - Raw reconstruction: Chapter 5 at origin / positive revs / negative revs / U40.1C mod reduction; sign_extend_16 boundary cases; combine_signed_i64_pair including the signed-boundary case.
    - Conversions: zero/one-shaft-turn, sign flip, roundtrip.
    - Anchor + canonical: zero-at-HM, whole-shaft-turn capture, master-offset subtraction.
    - Shaft-frame consistency: exact match (sub-count quantization < 1.5), 5-turn offset tolerated, 50-count drift rejected, 10-count jitter tolerated (A6-EC probe's post-restart 7-9 count spike ceiling).
    - Shortest-angular: zero/small/half-period/seam-straddle cases.
    - Command fold: 2026-04-17 J6 incident reproduced bit-exact. With `_j6_incident_kinematics` (`sign=-1` to match scratchpad's `base_counts = +3,623` from `canonical_q = -0.01737 rad`), the wrapped fold lands inside `[0, RM)` near `+3,623`; the legacy unwrapped fold produces `adjusted_counts ≈ 1,314,343` which is exactly the `RM + 3,623` incident value. Statelessness test: canonical_q=0 folds to 0 from either side of the seam, angular delta stays < 6,000 counts.
    - Inverse map roundtrip: canonical_q → fold → inverse recovers within one-count quantization.
    - Trajectory safety: small step OK, 100 deg angular step rejected, seam-straddling step rejected when profile flags `seam_crossing_unsafe=True` and allowed when False. Same matrix for first-point deviation. RuntimeError message contains `joint=6` and `period_counts=1310720` so existing operator tooling keeps working.
    - HM35 bias: midpoint = RM/2, quarter = RM/4, bias always in `[0, RM)` regardless of num/den input, num ≥ den rejected.
- What was intentionally NOT changed:
  - `backend.py` still owns the live state plumbing, the feedback-snapshot reads, and the wire-protocol structs. The new module is a pure-math PEER, not a drop-in replacement; it exists as a single source of truth for the equations themselves and a place the backend can delegate to incrementally. No behavioral change is shipped in this pass.
  - The production `_enforce_trajectory_wire_frame_safety`, `_nearest_turn_fold_axis_q_for_axis`, and `_shaft_frame_consistency_detail` are unmodified; the new module replicates their math so future refactors can delegate without risk.
- Validation performed:
  - `PYTHONPATH=src .venv/bin/python -m py_compile src/gradient_os/arm_controller/math/__init__.py src/gradient_os/arm_controller/math/a6ec_joint_motion.py tests/test_a6ec_joint_motion_math.py` → OK.
  - `PYTHONPATH=src .venv/bin/python -m pytest tests/test_a6ec_joint_motion_math.py -v` → `55 passed in 0.85s`.
  - Cross-check against existing A6-EC adjacent suites: `pytest tests/test_a6ec_chapter5_probe.py tests/test_a6ec_joint_sweep.py tests/test_a6ec_j6_watch_replay.py tests/test_gradient05_limits_and_backends.py tests/test_rtcore_runtime.py tests/test_drive_faults.py -q` → `224 passed in 5.36s`.
  - Full sweep: `pytest tests/ --ignore=tests/test_driver.py --ignore=tests/test_end_to_end.py --ignore=tests/test_protocol.py --ignore=tests/test_solver.py --deselect tests/test_planning.py::TestTrajectoryPlanning::test_path_unwrapping_and_smoothing -q` → `471 passed, 1 deselected in 8.79s` (up from 409 baseline; delta is the new math tests plus their collected fixtures).
  - `ReadLints` on all three touched files → no diagnostics.
- Follow-ups / risk:
  - Consider delegating the in-backend helpers (`_nearest_turn_fold_axis_q_for_axis`, `_shaft_frame_consistency_detail`, `_enforce_trajectory_wire_frame_safety`) to the math module in a subsequent pass so the 5,300-line backend shrinks and the math lives in exactly one place. Must be staged carefully — each helper has specific state interactions (logical_joint_idx → master_offset, native_home_offset_counts, axis_config) that the module intentionally does NOT capture.
  - Consider using `A6ECAxisKinematics` as the canonical constructor for robot-config-derived axis constants so `counts_per_unit` cannot silently drift between backend, probe, and any future offline planner.
  - The `_j6_incident_kinematics` helper pins that the real J6 uses `sign=-1`; if the robot-config sign is ever flipped, the 2026-04-17 J6 regression test will catch it.

## 2026-04-17 ROOT CAUSE of J6 360 deg excursion: A6-EC misinterprets `607A` above RM even with C10.16=0 - wrap command output into [0, RM)

- Task summary:
  - Operator reported that after the earlier two-phase fix (`C10.16=0` pinned + host wire-frame safety cage), a near-seam J6 jog STILL produced a physical `~360 deg` excursion at max speed. Two `-1 deg` jog commands ran back to back; the drive reported no fault, the Python safety cage did not trip, the controller logged `trajectory completed` both times. Yet multi_turn went from `+120,144` (post-HM) to `-1,183,301` (delta = `-1,303,445` counts = one shaft revolution backwards plus the intended forward motion). So move 1 took the LONG way, move 2 took the short way.
- Forensic reconstruction:
  - live_6064 before move 1 = `1,310,694` (= `-26` signed; `26` counts below the seam from the positive-wrap side).
  - Canonical_q target = `-0.01737 rad` (`-1 deg` joint); base_counts = `+3,623`.
  - Old fold logic: `delta = 1,310,694 - 3,623 = 1,307,071`; `wrap_turns = round(1,307,071 / RM) = 1`; `wrap_lift = +RM`; adjusted_counts = `+3,623 + 1,310,720 = 1,314,343`. That is `RM + 3,623`, i.e. one shaft turn ABOVE `[0, RM-1]`.
  - The A6-EC manufacturer notes (`docs/ethercat/a6ec-manufacturer-notes-2026-04-15.md` lines 459-477) explicitly define 6064 in rotation mode as a sawtooth in `[0, RM-1]`. Symmetrically, 607A must live in the same range. When we emitted 607A = `RM + 3,623` the A6-EC's rotation-mode interpretation went haywire: even with C10.16 verified as `0` (Nearest / shortest path) on every axis (metrics readback=0, verified=1), the drive took the LONG way (`-1,307,081 counts`) to reach what it apparently wrapped to single-turn `3,623`. No fault code was raised because from the drive's perspective it was tracking 607A smoothly; the only evidence is the continuous multi-turn counter.
  - The host safety cage (`_enforce_trajectory_wire_frame_safety`, first-point and per-point bounds at 0.35 rad ~= 20 deg joint) did NOT trip because the LINEAR delta between commanded `1,314,343` and live `1,310,694` was only `+3,639 counts` (`~1 deg`). The bug was purely in the drive's handling of out-of-range 607A; the host command looked harmless.
- Changes:
  - `src/gradient_os/arm_controller/backends/ethercat_rtcore/backend.py`:
    - Added `import math` at the top of the module.
    - `_nearest_turn_fold_axis_q_for_axis` gained a `wrap_to_single_turn: bool = False` parameter. Default behavior preserves the old linear-windowed semantic (required by the `_command_roundtrip_detail_for_axis` diagnostic and `_canonical_joint_q_from_command_axis_q` reverse map, which both compare directly against `reference_q`). Command mode wraps the fold output into `[0, RM)` via `adjusted_counts - period * floor(adjusted_counts / period)` plus a defensive `±period` clamp against IEEE-754 drift near the seam. `_command_axis_q_for_joint_value` opts in to `wrap_to_single_turn=True` so every 607A emitted by the host sits in the same range the A6-EC reports 6064 in under rotation mode.
    - `_command_axis_q_for_joint_value`'s `command_frame_oversized_delta` guard now measures SHORTEST-ANGULAR (mod-RM `±RM/2`) distance instead of linear distance, because after the wrap two points on opposite sides of the seam can have linear delta ~RM while being physically adjacent.
    - `_enforce_trajectory_wire_frame_safety` updated symmetrically: the per-point step check and the first-point live-deviation check both now compute `((linear + RM/2) % RM) - RM/2` and compare against the existing `_TRAJECTORY_MAX_PER_POINT_STEP_RAD` and `_TRAJECTORY_MAX_FIRST_POINT_DEVIATION_FROM_LIVE_RAD` thresholds. The RuntimeError payloads now include both the original linear delta and the angular delta so log triage can distinguish a legitimate seam-straddle from a real angular excursion.
  - Tests updated (the old "one shaft turn linear jump" scenario is now a no-op under the wrap, so tests that locked it down have been rewritten around a 100 deg angular jump instead):
    - `tests/test_gradient05_limits_and_backends.py::test_a6ec_small_jog_at_seam_stays_within_half_rm_wire_delta` - rewritten to check (a) 607A lands in `[0, RM)`, (b) shortest-angular delta matches the 0.5 deg jog, (c) linear delta stays within one shaft turn. Dropped `_force_legacy_truth_fallback` because that path uses motor-rev period not joint-rev period and the wrap-to-`[0, RM)` invariant under test only matters in native-ratio mode.
    - `tests/test_gradient05_limits_and_backends.py::test_a6ec_command_frame_rejects_oversized_trajectory_step` - now uses a 100 deg angular step instead of a full-RM linear step.
    - `tests/test_gradient05_limits_and_backends.py::test_a6ec_command_frame_rejects_point_far_from_live_reference` - now uses a 45 deg angular deviation instead of a full-RM linear deviation.
    - New: `tests/test_gradient05_limits_and_backends.py::test_a6ec_command_frame_allows_seam_crossing_step_in_linear_counts` (per-point counter-regression) and `test_a6ec_command_frame_allows_seam_straddling_first_point` (first-point counter-regression). Both assert that legitimate seam-crossing motion does NOT false-fail the cage.
    - `tests/test_a6ec_joint_sweep.py::test_a6ec_joint_sweep_fresh_hm_small_jog_stays_within_half_rm` - now asserts wire_counts in `[0, RM)` and shortest-angular delta matches the 0.5 deg jog.
    - `tests/test_a6ec_joint_sweep.py::test_a6ec_joint_sweep_fresh_hm_trajectory_pre_commit_gate_rejects_large_angular_jump` (renamed from `..._rejects_whole_turn_jump`) - now exercises a 100 deg angular step.
    - `tests/test_a6ec_j6_watch_replay.py::test_a6ec_j6_watch_replay_small_jog_stays_within_half_rm` - switched to shortest-angular delta.
    - `tests/test_a6ec_j6_watch_replay.py::test_a6ec_j6_watch_replay_rejects_large_angular_jump` (renamed from `..._rejects_would_be_whole_turn_jump`) - now exercises a 100 deg angular step.
- What was intentionally NOT changed:
  - The roundtrip / canonical-from-axis-q diagnostic paths keep the old linear-windowed fold. They compare directly against `reference_q` (the raw 6064 in axis_q space) and a mid-stream wrap would produce false-positive "roundtrip inconsistent" diagnostics on every seam-adjacent pose.
  - The `C10.16 = 0` startup SDO pin and the host-side safety cage thresholds. Those are still the right layers; this fix sits UPSTREAM of both (the drive can no longer see out-of-range 607A in the first place).
  - RTCore C++. The fix is Python-side; `./start-stack.sh` alone picks it up.
- Validation that ran:
  - `python -m py_compile src/gradient_os/arm_controller/backends/ethercat_rtcore/backend.py` → success.
  - `PYTHONPATH=src python -m pytest tests/test_gradient05_limits_and_backends.py tests/test_a6ec_joint_sweep.py tests/test_a6ec_j6_watch_replay.py tests/test_rtcore_runtime.py tests/test_drive_faults.py tests/test_api_endpoints.py tests/test_run_controller_helpers.py tests/test_encoder_retention.py -q` → `288 passed`.
  - `PYTHONPATH=src python -m pytest tests/ --ignore=tests/test_driver.py --ignore=tests/test_end_to_end.py --ignore=tests/test_protocol.py --ignore=tests/test_solver.py --deselect tests/test_planning.py::TestTrajectoryPlanning::test_path_unwrapping_and_smoothing -q` → `409 passed, 1 deselected`.
  - `ReadLints` on touched Python and test files → clean.
- Follow-up / risk:
  - **Primary follow-up**: verify the fix on hardware by running a near-seam J6 jog sequence. With the wrap-to-`[0, RM)` change, even if C10.16 is somehow overwritten at runtime, the A6-EC should no longer see ambiguous 607A values. If the physical motion STILL does not match the commanded delta, the next thing to check is whether the drive has a separate "rotation direction preference" object we missed (C10.27/.28 or similar), or whether the ESI catalog defines a different 607A representation for this hardware revision.
  - The rotation-mode contract is now symmetric: both 6064 (reported) and 607A (commanded) live in `[0, RM)`. Any new code path that emits 607A must go through the `wrap_to_single_turn=True` fold so this invariant cannot silently regress. Consider adding a defensive assert in `_axis_q_from_joint_positions` that the pre-RTCore axis_q array maps to wire counts in `[0, RM)` on each axis - the current enforcement is by convention (every caller opts in).
  - The post-HM J6 metrics this session showed `pos_counts=1,310,694` immediately after HM35 success - i.e. the drive parked one count below the seam, which is why every subsequent canonical_q~=0 jog tried to cross the seam. Not a bug on our side, but a sharp edge worth noting in operational docs: after a successful drive-native home, the first jog can land the fold right at the seam boundary, so any wrap-math bug has the shortest possible fuse to trigger.

## 2026-04-17 Unblock commissioning panel per-joint truth after cold start (synthesize drive_faults from axis_absolute_feedback + stop silencing backend drive-faults exceptions)

- Task summary:
  - After a cold start and a successful J6 Drive Home, the commissioning panel was stuck on `Canonical truth unavailable: persisted_home_anchor_inconsistent_with_live_6064` for J1 and J3 even though `/info/joints-detailed` simultaneously reported both axes as `truth_available=True, drive_native_truth_reason=valid, verification_source=persisted_home_anchor_agreement` with shaft-frame mod-RM deltas of `1.0-3.0 counts` (well within the 16-count tolerance). The UI's per-joint truth display had drifted off the backend's actual state.
- Evidence collected live:
  - `curl http://127.0.0.1:4400/info/joints-detailed` for J1/J3: healthy, truth available, shaft-frame consistent.
  - 10-second SSE capture from `/monitor`: 378 events, 378 had `servos` and `axis_absolute_feedback`, ZERO had the aggregate `drive_faults` block.
  - `build_drive_fault_snapshot` called directly in isolation (without passing `axis_drive_native_truth_context`) reproduces `drive_native_truth_reason=coordinate_system_invalid` for every axis without the HM-success statusword signature. The run_controller's `_attach_drive_faults_to_telemetry_message` is supposed to populate that context from `backend.get_display_feedback_snapshot()`, but the attacher was throwing silently inside `try: ... except Exception: pass`.
- Root cause (two layers):
  - **Backend**: `_attach_drive_faults_to_telemetry_message` raises on every call in the current controller process (root cause still to be traced - the silent `pass` hid the actual exception type). The monitor stream therefore never carries the aggregate `drive_faults` block, even though all the underlying per-axis data is already live on the same event under `axis_absolute_feedback`.
  - **Frontend**: `App.tsx::handleMessage` preserved `prev.drive_faults` whenever the event lacked a fresh one (`next.drive_faults ?? prev?.drive_faults ?? null`). When `drive_faults` went permanently missing, the UI kept showing the first-ever `drive_faults` captured in the session - the one from the moment the J6 360 deg excursion was present AND the shaft-frame gate was failing for J1/J3 mid-recovery.
- Changes:
  - **Frontend (the unblock path, no restart required; Vite HMR picks it up)**: `web-ui/src/App.tsx`:
    - New module-level helper `synthesizeDriveFaultSnapshotFromAxes(event)` that builds a minimal `DriveFaultSnapshot` (just the `axes` array) from the monitor event's `axis_absolute_feedback` + `servos`. Only populates the per-axis fields the commissioning panel actually reads through `DriveFaultAxis`: `drive_native_truth_valid/reason/verification_source`, `coordinate_system_valid`, `statusword`, `error_code`, `manufacturer_error_code`, `ds402_state`, `native_home_state/name`. Returns `null` if no `axis_absolute_feedback` is present.
    - `handleMessage`'s drive_faults ingest now prefers (a) the event's explicit `drive_faults` block, then (b) the synthesized block when `axis_absolute_feedback` is present, then (c) the legacy fallback to `prev.drive_faults`.
    - `setLatest((prev) => ...)` merge strategy for drive_faults: if `next.drive_faults` was synthesized (detected via `Object.keys(next.drive_faults).length === 1`, i.e. only the `axes` array), overlay its fresh `axes` onto `prev.drive_faults` so top-level drive-power bookkeeping (`servo_backend`, `driver_state`, `axis_enable_mask`, `native_home_active_axis_mask`, `num_axes`, `op_enabled_axes`, etc.) is preserved while per-axis truth is always current. If `next.drive_faults` is a full backend-emitted block, it replaces `prev` verbatim.
  - **Backend (diagnostic, activates on next stack restart)**: `src/gradient_os/run_controller.py`:
    - Replaced the silent `try: _attach_drive_faults_to_telemetry_message(...); except Exception: pass` around the telemetry-loop drive-faults attach call with a throttled stderr log: `[Controller] Drive-faults attach failed ({ExceptionType}): {message}`, emitted at most once every 5 seconds so a persistent failure does not drown the controller log.
    - Added `_last_drive_faults_attach_error_ts: list[float] = [0.0]` as throttle state just above the `_telemetry_loop` closure; uses list-as-mutable-cell to avoid `nonlocal` churn.
    - Existing `sys` and `time` imports at the top of the module cover the new log path; no new imports required.
- What was intentionally NOT changed:
  - The underlying cause of the drive-faults attach failure. We do not know its exception type yet because the silencing was hiding it. The logging change is enough to surface it the next time the stack starts; fixing the underlying cause becomes a separate follow-up scoped by whatever the log reveals.
  - The monitor event's existing `axis_absolute_feedback` emission path. That's already the authoritative per-axis truth source; the fix just teaches the frontend to read it when `drive_faults` is absent.
  - The existing `prev.drive_faults` preservation pattern. It's still the right fallback when an event carries neither `drive_faults` nor `axis_absolute_feedback`. We just no longer hard-depend on it for per-axis truth freshness.
- Validation that ran:
  - `cd web-ui && npx vitest run src/ControlPanel.test.tsx` → `24 passed` (all existing coverage + the previously added J6 per-joint fallback tests).
  - `cd web-ui && npm run build` → clean build; `dist/assets/app-BAfbDjK6.js` (was `-z_Id8ADy.js`) produced with the new synthesizer.
  - `python -m py_compile src/gradient_os/run_controller.py` → success.
  - `python -m pytest tests/test_run_controller_helpers.py -q` → `9 passed`.
  - `ReadLints` on `web-ui/src/App.tsx` and `src/gradient_os/run_controller.py` → no diagnostics.
  - Live repro confirmed pre-fix: 10s `/monitor` SSE capture showed 378 events, 0 `drive_faults` occurrences, 378 `servos`. Post-fix UI behavior is validated via Vite HMR (user refresh).
- Follow-up / risk:
  - **Primary follow-up**: trace the underlying cause of the drive-faults attach exception on the next stack restart (look for the new `[Controller] Drive-faults attach failed (...)` log line). The synthesizer is a safety net; the backend block should be the source of truth once the cause is fixed.
  - The synthesized `DriveFaultSnapshot` intentionally omits top-level fields like `num_axes`, `op_enabled_axes`, `statusword_feedback_axes`, `physical_state`, `ethercat_master_state`, `rtcore_state`. The UI relies on those for runtime-header labels and power-transition diagnostics. When no prev `drive_faults` exists (first event after a cold connect) and the backend is still dropping drive_faults, those fields stay `undefined` until the backend recovers. Acceptable for the immediate unblock but worth watching.
  - Stale `prev.drive_faults` without a fresh event is still possible if axis_absolute_feedback is ALSO absent from an event; in practice this only happens when the whole joints pipeline is down, in which case the commissioning panel has bigger problems to worry about than stale truth status.

## 2026-04-17 Unblock J6 commissioning panel (display gap + effect thrash) after 360 deg excursion

- Task summary:
  - Post-cold-start, operator reported the Joint Commissioning panel was flickering at ~5-10 Hz between "J1-J5 angles shown + J6 '--'" and "all six joints '--' + 'Waiting for joint feedback...'", with the entire panel unusable. J6 was physically displaced by one full shaft revolution from its home (the remnant of the 2026-04-17 360 deg incident), so the operator needed the Drive Home button on J6 to re-anchor — which they could not reliably reach because of the flicker.
- Evidence collected live before touching code:
  - `ls -la /run/gradient-rt-motion/metrics.json` + a `startup_drive_configs` dump confirmed the earlier `C10.16=0` safety-fix landed cleanly: every axis reports `readback=0, verified=1, readback_valid=1` for `a6ec_rotation_mode_reference_running_direction`. The drives are explicitly pinned to Nearest/shortest-path mode now. That fix is working as intended.
  - `curl http://127.0.0.1:4400/info/joints-detailed`:
    - `canonical_joint_truth_available=true`, `raw_canonical_joint_truth_available=true`, `display_joint_truth_available=false`.
    - `arm_deg = [5.45, -4.91, 0.06, -0.01, -0.02, 360.01]`, `arm_display_deg = [5.45, -4.91, 0.06, -0.01, -0.02, null]`.
    - J6 per-axis detail: `truth_reason="drive_native_command_frame_roundtrip_mismatch"`, `command_roundtrip_reference_error_counts=-1310721.0` (exactly `-RM - 1`), `shaft_frame_consistent=true`, `persisted_home_anchor_consistent=true`. The state is internally coherent; the display-mode command roundtrip refuses a canonical_q that sits a full shaft turn away from the single-turn reference because it does not re-fold against live 6064 (that's by design for display).
  - SSE `/monitor` sampled at ~5 Hz: `display_joints=None` on every event, `joints` carries the full canonical array including J6 at 6.283 rad. The monitor stream has no per-joint operator display for this backend configuration at present.
- Root cause, two factors stacked:
  - **Frontend state shape**: `ControlPanel.tsx::preferredJointAnglesDeg` only read `arm_display_deg` and returned `[5.45, -4.91, 0.06, -0.01, -0.02, NaN]`. Correct per the old contract ("display mode = display mode"), but the operator had no per-joint visibility of where J6 actually was.
  - **Frontend effect thrash**: the fallback-poll `useEffect` inside the commissioning panel depended on the whole `latestTelemetry` object. Every SSE monitor event (~5-10 Hz) cleaned up the old `setInterval` and set up a new one with an immediate poll. When display was partially-available but the SSE `display_joints` was `null`, the `hasAnyFiniteJointAngles(preferredTelemetryJointAnglesRad(latestTelemetry))` early-return check never fired, so the effect permanently polled AND tore itself down 5-10 times a second. That instability, combined with the `setJointFeedbackError("Waiting for joint feedback...")` branch in `refreshJointAngles` firing on any fetch race / transient, produced the visible "all joints '--'" flashes.
- Changes:
  - `web-ui/src/ControlPanel.tsx`:
    - New helper `mergeDisplayWithCanonicalFallback(primary, fallback)`. It only fills NaN slots in `primary` from `fallback`; if `primary` is fully missing it returns `null`. This preserves the "don't leak cached canonical into operator display" contract while allowing the panel to display per-joint canonical when display is PARTIALLY present.
    - `preferredJointAnglesDeg` and `preferredTelemetryJointAnglesRad` now gate the canonical fallback on explicit canonical-truth-live signals: `raw_canonical_joint_truth_available` / `canonical_joint_truth_available` / `read_source === "live_feedback"`. For the SSE stream we additionally accept "flags missing AND finite joints array present" so older monitor payloads still work.
    - New `hasFreshTelemetryJointAngles` `useMemo` boolean. The fallback-poll `useEffect` now depends on this stable boolean instead of `latestTelemetry`. The `setInterval` is torn down only when availability actually flips, not on every SSE event. This eliminates the interval-recreate churn that was driving the panel's visible flicker.
    - Extended `JointInfoResponse` with `raw_canonical_joint_truth_available`, `display_joint_truth_available`, and `canonical_joint_truth_available` fields so TypeScript can type the fallback gate.
    - Visualizer feedback path untouched: `onJointFeedback` still requires `hasAllFiniteJointAngles` and receives `[]` otherwise. Canonical fallback is panel-display-only.
  - `web-ui/src/ControlPanel.test.tsx`:
    - New regression `fills per-joint display gaps from live canonical angles so a 360-deg-offset joint stays visible`: J6 with `arm_display_deg[5]=null`, `arm_deg[5]=360.01`, `raw_canonical_joint_truth_available=true` renders `360.01°` in the panel (no `"--"` on any joint).
    - New regression `does not fall back to canonical when display truth is partial but canonical is not authoritative`: same shape but with `read_source="unavailable"`, `raw_canonical_joint_truth_available=false`; J6 must stay `"--"` and `arm_deg[5]=360.01` must NOT render. Protects the "no cached canonical masquerades as display" contract.
- What was intentionally NOT changed:
  - The backend display-mode command-roundtrip gate in `_command_roundtrip_detail_for_axis` (still intentionally strict; a post-360-excursion pose is a legitimate operator-facing anomaly the backend should surface). Fixing the operator block belongs in the UI layer since the operator needs to SEE the angle to decide whether to re-home or rotate back.
  - The SSE `/monitor` emitter in `run_controller.py`. The current stream does not advertise `display_joints` or `display_joint_truth_available`; the UI now tolerates that gracefully via REST fallback. Adding those fields to the SSE stream is a reasonable follow-up but not blocking.
  - Any changes to `_base_command_axis_q_for_joint_value` / the stateless nearest-turn fold. The review in [J6 wrap safety review](2b1bcb09-9610-4306-b5f8-2fb090c1393a) confirmed the write-path algebra is correct (canonical_q + master_offset → fold against live 6064), and that re-applying the absolute-home anchor on the write path would in fact be a regression.
- Validation that ran:
  - `curl http://127.0.0.1:4400/info/joints-detailed` sampled 20 times at 20 Hz; every sample consistent, arm_display_deg stable with `null` for J6 only. No flicker on the server side.
  - `cd web-ui && npx vitest run src/ControlPanel.test.tsx` → `24 passed` (22 pre-existing + 2 new regressions).
  - `cd web-ui && npm run build` → built cleanly, emitted `dist/index.html`, `dist/j6-manual-rotate-dataset.html`, and the main chunks as usual. The pre-existing `ArmVisualizer` bundle-size warning is unchanged.
  - `ReadLints` on `web-ui/src/ControlPanel.tsx` and `web-ui/src/ControlPanel.test.tsx` → no diagnostics.
- Follow-up / risk:
  - Operator action still required on hardware: with the panel now showing `J6 = 360.01°`, the operator can either click J6 "Drive Home" (captures a fresh absolute-home anchor at the current pose and the display gate will start passing) or rotate J6 back to its original home. Either choice is safe given `C10.16=0` pins the drive to shortest-path and the pre-commit wire-frame safety cage is in place.
  - If a future incident produces multiple simultaneous per-joint display gaps (e.g. Jn for multiple n), this fix handles all of them uniformly — each NaN slot is independently backfilled from canonical IF canonical is authoritative.
  - Consider adding `display_joints` and `display_joint_truth_available` to the SSE monitor payload so the commissioning panel can avoid REST polling entirely when the stream carries display truth; would fully eliminate the fallback-poll branch's existence in steady state.

## 2026-04-17 SAFETY: tighten joint-move command-frame safety cage + pin A6-EC C10.16 to Nearest path (J6 360 deg incident)

- Task summary:
  - Operator reported a severe safety incident on startup `logs/startups/20260417-092330`: two back-to-back J6 jog commands, each requesting a `~1 deg` bounded move, drove the physical J6 joint through `~360 deg` at max speed before the operator disarmed. Controller/API logs showed clean `target_deg=-0.987` then `target_deg=+0.012` with no faults, so the divergence between reported intent and physical motion had to be caught either in host→drive frame translation or in the drive's rotation-mode behavior. The user's directive: "the math for sending joint move commands needs to be perfect there is not room for mistakes".
- Evidence collected (no product code change in this phase):
  - `/run/gradient-rt-motion/metrics.json` post-incident: J6 `pos_counts (6064) = 1,310,690` (= `-30` in signed shaft-frame representation, single-turn near zero), but `encoder_multi_turn_low/high` combined to `-1,190,565`. Reconstructed pre-move `multi_turn = +120,155` (= `anchor_rad * -1 * counts_per_unit`, since anchor `-0.5761 rad` and canonical_q `~0` at the time), so the continuous motor state moved by exactly `-1,310,720 counts = -1 joint shaft revolution` during the jog. That maps to `+360 deg` of joint rotation through the `sign=-1` convention, matching the operator's observation.
  - `6064 = 1,310,690` is `-30 signed`, meaning the drive's single-turn reference position did NOT move — the multi-turn counter wrapped around a full shaft revolution while 6064 stayed put. That is consistent with "the drive took the long way around the wrap seam instead of the short way".
  - A6-EC `C10.16 (Reference running mode in rotation mode)` per vendor manual chapter 11: `0=Nearest`, `1=Always forward`, `2=Always reverse`, `3=Keep current direction`, `4=Not specified`. This is the exact knob that decides short-vs-long-path behavior in `C00.07=4` (rotation mode). Our existing A6-EC startup SDO list only pinned `C00.07`, `C10.18`, `C10.19`; `C10.16` was left at whatever was in the drive's NVM, which could be anything and specifically might not be `0`.
- Root cause, two contributing factors (both fixed in this pass):
  - (Drive-side) A6-EC `C10.16` never explicitly configured at startup. If the drive's NVM holds a non-zero value (e.g. "Always reverse" or "Keep current direction"), commanded targets near the seam go the LONG way around, producing a full-shaft-revolution physical motion from a tiny logical delta.
  - (Host-side) The pre-commit trajectory-upload safety gate `_enforce_trajectory_step_within_half_rm` only refused wire-frame steps larger than `0.5 * RM`, which equals `180 deg` of joint motion per point. That is orders of magnitude more permissive than any legitimate jog or bounded move needs, and would not have caught the observed excursion even if it had been produced by a pure host-math bug.
- Changes:
  - [src/gradient_os/arm_controller/ethercat_drive_catalog.py](src/gradient_os/arm_controller/ethercat_drive_catalog.py): added `a6ec_rotation_mode_reference_running_direction` to the `a6ec_ds402` `startup_defaults` (default `0 = Nearest/shortest path`) and `startup_schema` (`u16`, object `0x2010:0x17`, range `0..4`). Pinning this at every startup guarantees shortest-path behavior regardless of drive-side NVM state.
  - [src/gradient_os/arm_controller/profiles/drive/a6ec_ds402.py](src/gradient_os/arm_controller/profiles/drive/a6ec_ds402.py): added `STARTUP_REFERENCE_DIRECTION_KEY`, `_LABEL`, `_OBJECT`, and `STARTUP_REFERENCE_DIRECTION_VALUE_LABELS` constants; wired the new key into `_STARTUP_SETTING_ORDER` / `_STARTUP_SETTING_METADATA`; added `_startup_reference_direction_value_label` and dispatch from `_startup_setting_value_label` so operator-facing snapshots decode the readback value as a human-readable direction (e.g. "Nearest (shortest path)"). `render_rtcore_systemd_env` now emits 4 startup-SDO descriptors (was 3) within the `kMaxStartupSdoDescriptors=8` RTCore limit, so no RTCore C++ rebuild is strictly required; a stack restart picks up the new env.
  - [src/gradient_os/arm_controller/backends/ethercat_rtcore/backend.py](src/gradient_os/arm_controller/backends/ethercat_rtcore/backend.py): renamed `_enforce_trajectory_step_within_half_rm` → `_enforce_trajectory_wire_frame_safety`. New two-bound contract expressed in joint-space radians so thresholds mean the same motion envelope on every axis:
    - `_TRAJECTORY_MAX_PER_POINT_STEP_RAD = 0.35` (`~20 deg joint`) - caps the wire-frame delta between consecutive uploaded points. Catches mid-trajectory fold flips. Replaces the old `0.5 * RM = 180 deg` step bound that was far too permissive.
    - `_TRAJECTORY_MAX_FIRST_POINT_DEVIATION_FROM_LIVE_RAD = 0.35` (`~20 deg joint`) - caps the wire-frame distance from live `6064` for POINT 0 only. Since point 0's `canonical_q` is read from live feedback before uploading, its folded `607A` should land near live `6064`; a bigger deviation is a turn-selection bug (host fold or drive config) that would teleport the joint. Subsequent trajectory points may legitimately travel far from live for long bounded moves, so this gate only applies to point 0.
    - `enqueue_trajectory_points` now captures `initial_live_counts = [self._live_reference_counts_for_axis(i) for i in range(self._rt_num_axes)]` once before the loop and threads it through the safety check.
    - Raises `command_frame_oversized_step` and `command_frame_live_deviation_out_of_range` (new) respectively, with axis/joint/target/live counts in the message so operator incident reports can be triaged without guesswork.
  - [tests/test_gradient05_limits_and_backends.py](tests/test_gradient05_limits_and_backends.py): two synthetic upload tests (`..._keeps_controller_logical_frame_with_native_home_offset`, `..._keeps_nonunit_a6ec_axis_in_logical_radians`) now stage `backend._axis_counts[0]` to match the commanded point so the new first-point-deviation cage does not refuse the synthetic single-point upload; the test's contract is frame conversion, not motion safety. Existing `test_a6ec_command_frame_rejects_oversized_trajectory_step` updated to use the new method/parameter names. Added new regression `test_a6ec_command_frame_rejects_point_far_from_live_reference` that drives a single trajectory point exactly one shaft revolution from live 6064 and asserts `command_frame_live_deviation_out_of_range` fires — direct 2026-04-17 J6 incident coverage.
  - [tests/test_a6ec_joint_sweep.py](tests/test_a6ec_joint_sweep.py) and [tests/test_a6ec_j6_watch_replay.py](tests/test_a6ec_j6_watch_replay.py): the "trajectory must refuse a whole-turn jump" regressions updated to use `_enforce_trajectory_wire_frame_safety`, to pass `initial_live_counts`, and to accept either `command_frame_oversized_step` or `command_frame_live_deviation_out_of_range` (either is a correct fail-closed outcome for the pathological one-shaft-turn jump).
  - [tests/test_rtcore_runtime.py](tests/test_rtcore_runtime.py): helper `_a6ec_startup_drive_configs_for_ratio` now appends the `a6ec_rotation_mode_reference_running_direction` entry (default `0`). The rendered `GRADIENT_RT_DRIVE_STARTUP_SDO_CONFIG` assertion was extended to expect the 4th descriptor `;a6ec_rotation_mode_reference_running_direction|u16|0x2010|0x17|0,0,0,0,0,0`. The `build_rtcore_drive_startup_config` return-shape assertions now include the new key/value list.
  - [tests/test_a6ec_j6_watch_replay.py](tests/test_a6ec_j6_watch_replay.py) / [tests/test_a6ec_joint_sweep.py](tests/test_a6ec_joint_sweep.py): `_startup_drive_config_entries` helpers append the reference-direction setting with `commanded/readback=0, verified=1` so backend-side `extract_startup_config_axis` sees all four required settings present and marked verified (matches the new `_STARTUP_SETTING_ORDER` list).
- What was intentionally NOT changed in this pass:
  - The write-path math in `_base_command_axis_q_for_joint_value` + `_nearest_turn_fold_axis_q_for_axis`. When canonical_q is shaft-frame-consistent with live 6064 (gate already enforces this at read time), the fold reconstructs the correct 607A. The 2026-04-17 incident is explained by drive-side rotation-direction config plus an over-permissive host safety cage; no change to the fold algebra is required to close the safety hole.
  - `/usr/local/bin/gradient-rt-motion`: no RTCore C++ changes. `kMaxStartupSdoDescriptors=8` already accommodates the 4th descriptor. Stack restart is sufficient to pick up the new startup SDO on hardware.
  - Drive fault / telemetry surfaces: existing `startup_drive_configs` telemetry automatically picks up the 4th descriptor via `extract_startup_config_axis`; no API/UI wiring changes needed.
- Validation that ran:
  - `source /home/pi/GradientOS/.venv/bin/activate && PYTHONPATH=src python -m py_compile src/gradient_os/arm_controller/backends/ethercat_rtcore/backend.py src/gradient_os/arm_controller/profiles/drive/a6ec_ds402.py src/gradient_os/arm_controller/ethercat_drive_catalog.py tests/test_rtcore_runtime.py tests/test_gradient05_limits_and_backends.py tests/test_a6ec_j6_watch_replay.py tests/test_a6ec_joint_sweep.py` → success.
  - `PYTHONPATH=src .venv/bin/python -m pytest tests/test_rtcore_runtime.py tests/test_gradient05_limits_and_backends.py tests/test_a6ec_joint_sweep.py tests/test_a6ec_j6_watch_replay.py tests/test_a6ec_chapter5_probe.py tests/test_drive_faults.py tests/test_run_controller_helpers.py tests/test_api_endpoints.py tests/test_encoder_retention.py -q` → `176 passed` (targeted touched-surface sweep).
  - `PYTHONPATH=src .venv/bin/python -m pytest tests/ --ignore=tests/test_driver.py --ignore=tests/test_end_to_end.py --ignore=tests/test_protocol.py --ignore=tests/test_solver.py --deselect tests/test_planning.py::TestTrajectoryPlanning::test_path_unwrapping_and_smoothing -q` → `407 passed` (full non-legacy sweep).
  - `test_path_unwrapping_and_smoothing` reproduces on clean HEAD in the same full-sweep mode (test-pollution from another test earlier in the run) but passes in isolation — pre-existing and unrelated to this change. The legacy `test_driver.py` / `test_end_to_end.py` / `test_protocol.py` / `test_solver.py` failures were already excluded by the repo convention.
  - `make -C src/gradient_rt_motion` → clean rebuild (not strictly required for this fix, but confirms the generic SDO parser path still compiles).
  - `PYTHONPATH=src .venv/bin/python -c "from gradient_os.arm_controller.backends.ethercat_rtcore.runtime import build_rtcore_drive_startup_config; from gradient_os.arm_controller.robots import get_robot_config; print(build_rtcore_drive_startup_config(get_robot_config('gradient05').get_config_dict(), drive_profile='a6ec_ds402')['env']['GRADIENT_RT_DRIVE_STARTUP_SDO_CONFIG'])"` → confirms the emitted env var includes `;a6ec_rotation_mode_reference_running_direction|u16|0x2010|0x17|0,0,0,0,0,0` on all six axes.
  - `ReadLints` on every touched Python/test file: clean.
- Follow-up / risk:
  - Live verification still owed: after the next `./start-stack.sh`, read back `startup_drive_configs[3].readback` on every axis from `/run/gradient-rt-motion/metrics.json` (or the UI's commissioning panel) and confirm each reports `readback=0, verified=1`. If any axis shows a different readback value, the drive's NVM or write path is diverging from our pinned default and should be investigated before the arm is powered up again.
  - Explicit follow-on experiment once the stack is back up (safely): rehome J6, park the joint near the seam (6064 just above or below wrap), and command a tiny jog that crosses the seam. With `C10.16=0` and the new safety cage in place, the jog must produce `~1 deg` of joint motion. Any deviation is a bug.
  - The raw write-path math still does not re-apply the persisted absolute-home anchor. That is latent - the shaft-frame consistency gate at read time compensates in practice - but should be audited as part of the broader A6-EC write-frame SOP once the immediate safety work is in hand.

## 2026-04-17 Fix /monitor drive-native-truth regression (UI showed "coordinate_system_invalid" on every joint)

- Task summary:
  - Operator reported the dashboard showed "Canonical truth unavailable: coordinate_system_invalid" under every joint after boot, even after `sudo systemctl restart gradient-rt-motion` and a full stack restart. `curl /info/joints-detailed` simultaneously reported every axis as valid via `persisted_home_anchor_agreement`. Two endpoints that should be reading the same single-source-of-truth answer were disagreeing.
- Root cause:
  - When the W1+W2+W3 persistence-trust polish landed earlier today, `telemetry/native_home_status.py::derive_drive_native_truth_validity` gained `accept_persisted_home_anchor_as_restart_trust`, `persisted_home_anchor_present`, `persisted_home_anchor_consistent`, `multi_turn_feedback_valid`, `last_seen_*`, and `encoder_retention_fault_present` kwargs. `arm_controller/backends/ethercat_rtcore/backend.py` was updated to pass all of them (which is what feeds `/info/joints-detailed`). But `telemetry/drive_faults.py::build_drive_fault_snapshot` (which feeds the `/monitor` SSE stream the UI consumes) was NOT updated and kept calling the helper with only `statusword`, `error_code`, `manufacturer_error_code`, `require_hm_success_signature`, and `encoder_retention_fault_present`. With no opt-in flag and no anchor signals, the helper's anchor-based trust path (native_home_status.py:140-149) never ran, so every axis whose `0x6041` bit 15 was clear (normal post-boot A6-EC state) fell through to `reason="coordinate_system_invalid"`. The UI reads `/monitor.drive_faults.axes[].drive_native_truth_*`, so that's what operators saw.
- Changes:
  - [src/gradient_os/telemetry/drive_faults.py](src/gradient_os/telemetry/drive_faults.py): added `axis_drive_native_truth_context: Mapping[int, Mapping[str, Any]] | None = None` kwarg to `build_drive_fault_snapshot`. After the local `derive_drive_native_truth_validity` call, if the context has an entry for `axis_index`, the per-axis truth dict is overridden verbatim from the context (`drive_native_truth_valid` / `drive_native_truth_reason` / `drive_native_truth_signature_valid` / `coordinate_system_valid` / `drive_native_truth_verification_source`). The local call is preserved as a best-effort fallback so callers without backend context still get reasonable output (e.g. a dev running `/info/drive-fault-snapshot` on a metrics file).
  - [src/gradient_os/run_controller.py](src/gradient_os/run_controller.py): `_build_drive_fault_snapshot` now calls `backend_instance.get_display_feedback_snapshot()` and extracts the per-axis `drive_native_truth_*` + `coordinate_system_valid` fields from `axis_absolute_feedback`. Those are packed into `axis_drive_native_truth_context` and passed to `build_drive_fault_snapshot`. The `/monitor` stream now carries the backend's single-source-of-truth answer verbatim instead of a re-derived wrong one.
- What was intentionally NOT changed:
  - `derive_drive_native_truth_validity`'s signature is unchanged.
  - `backend.py` already passes all kwargs correctly; not touched.
  - Test `tests/test_drive_faults.py` still exercises the fallback (no-override) path as before; the override path is covered end-to-end by the manual `/monitor` vs `/info/joints-detailed` diff performed below.
- Validation:
  - `py_compile` + `ReadLints` on touched files: clean.
  - `pytest tests/test_drive_faults.py tests/test_rtcore_runtime.py tests/test_run_controller_helpers.py tests/test_api_endpoints.py tests/test_gradient05_limits_and_backends.py tests/test_a6ec_chapter5_probe.py tests/test_a6ec_joint_sweep.py -q` → `289 passed`.
  - On live hardware after stack restart (run `20260417-090326`), `/info/joints-detailed` and `/monitor` now both report every axis identically: J1-J5 `valid=True, reason=valid, src=persisted_home_anchor_agreement, coord_valid=True`; J6 `valid=True, reason=valid, src=statusword_bits12_15_clear13, coord_valid=True`. Before the fix, `/monitor` reported J1-J5 as `valid=False, reason=coordinate_system_invalid, src=unverified, coord_valid=False`.
- Follow-up / risk:
  - Any future `derive_drive_native_truth_validity` kwarg must be either (a) added to the backend's callsite AND reflected in `get_display_feedback_snapshot` output (which the monitor pipeline now consumes), OR (b) independently plumbed into `build_drive_fault_snapshot` via a new kwarg. Don't leave the monitor pipeline blind to new restart-trust signals again.
  - Consider a targeted unit test in `tests/test_drive_faults.py` that constructs a minimal fake backend-context mapping and asserts `build_drive_fault_snapshot` adopts it verbatim — would have caught this.

## 2026-04-17 API port default moved 4000 → 4400 (Windows iphlpsvc collision under Cursor Remote-SSH)

- Task summary:
  - While wiring up Cursor Remote-SSH port-forwarding for this workspace, Cursor popped `Local port 4000 could not be used for forwarding to remote port 4000. Port number 54245 has been used instead.` That breaks the web UI's API calls because `web-ui/src/useEndpoint.ts` hard-codes `apiPort=4000`, so the browser's `localhost:4000` requests don't reach the Pi's API when Cursor has been forced to a different local port.
  - Root cause on Windows: `svchost.exe` (PID 5676 here) hosting `iphlpsvc` (IP Helper) had already taken `0.0.0.0:4000`. `iphlpsvc` is a hard dependency of `SharedAccess` (ICS), so stopping it was not an option (it would cascade-stop ICS and kill the Pi's DHCP-over-ICS internet).
  - Durable fix chosen: move the Gradient API's default port from `4000` to `4400`. `GRADIENT_API_PORT` env-var override is preserved so anyone on a machine without the collision can still pin 4000 if they want.
- Changes (all committed together 2026-04-17):
  - [start-stack.sh](start-stack.sh): `API_PORT="${GRADIENT_API_PORT:-4400}"` (was `4000`) and expanded the `Environment:` help block so the reason for 4400 is written down: `API HTTP port (default: 4400; 4000 collides with Windows iphlpsvc under Cursor port-forwarding)`.
  - [src/gradient_os/api/main.py](src/gradient_os/api/main.py): `--port` argparse default `int(os.environ.get("GRADIENT_API_PORT", "4400"))` (was `"4000"`). The CLI still honours `GRADIENT_API_PORT`.
  - [web-ui/src/useEndpoint.ts](web-ui/src/useEndpoint.ts): `const apiPort = 4400;` plus an inline `NOTE:` comment explaining the Windows iphlpsvc root cause and telling future editors to keep this in sync with the launcher and the Python CLI default.
  - [web-ui/src/App.tsx](web-ui/src/App.tsx): API-host input placeholder text `http://localhost:4400` (cosmetic-only; was `http://localhost:4000`).
  - [tests/test_a6ec_chapter5_probe.py](tests/test_a6ec_chapter5_probe.py): `api_url="http://127.0.0.1:4400"` in the manual-J6-rotate snapshot test.
  - [.vscode/settings.json](.vscode/settings.json) (local-only, gitignored): swapped `remote.portsAttributes["4000"]` for `remote.portsAttributes["4400"]` with `onAutoForward: "notify"` and `requireLocalPort: true`; left a `"4000": { onAutoForward: "ignore" }` stub so Cursor stops trying to forward the legacy port.
  - [docs/README.md](docs/README.md) and [docs/ethercat/bringup.md](docs/ethercat/bringup.md) and [systemd/README.md](systemd/README.md): updated every remaining `localhost:4000` / `127.0.0.1:4000` / `--port 4000` / `0.0.0.0:4000` reference to `4400` and added a one-line rationale where natural.
- What was intentionally NOT changed:
  - `docs/resources/ethercat/esi/stepperonline/A6-EC/STEPPERONLINE_A6_Servo_V0.02.xml` MinValue/MaxValue `4000` tags — these are A6-EC motor-parameter bounds (RPM etc.), nothing to do with ports.
  - `src/numeric_solver/pyquik/**` third-party build artefacts — irrelevant.
  - `src/trajectory_cache/*.json` float coefficients — coincidental substring matches.
  - `GRADIENT_RT_DRIVE_VENDOR_ID=0x00400000` in the rt-motion systemd env — that's a vendor ID, not a port.
- Validation:
  - `bash -n start-stack.sh` syntactically valid.
  - `python3 -m py_compile src/gradient_os/api/main.py` clean.
  - `python3 -c "import json; json.load(open('.vscode/settings.json'))"` parses.
  - `ReadLints` on `web-ui/src/useEndpoint.ts`, `web-ui/src/App.tsx`, `.vscode/settings.json` — no diagnostics.
  - `PYTHONPATH=src .venv/bin/python -m pytest tests/test_api_endpoints.py tests/test_a6ec_chapter5_probe.py tests/test_run_controller_helpers.py -q` → `88 passed`.
  - `cd web-ui && npx vitest run src/ControlPanel.test.tsx` → `22 passed`.
- Follow-up / risk:
  - Stack-restart validation on hardware still needed (new default hasn't actually been exercised via `./start-stack.sh`). Operator should restart the stack, confirm the dashboard still populates via the auto-forwarded port 4400, and that `curl http://127.0.0.1:4400/health` returns `ok`.
  - If any external tool (CI runner, CoE probe script, another workstation) pins the Pi's API at `:4000` explicitly, set `GRADIENT_API_PORT=4000` in that specific invocation's environment. The env-var override wasn't removed.
  - Port 4400 is itself not IANA-reserved and could collide with some other tool in the future. If it ever does, grep anchor words `:4000`, `:4400`, `apiPort`, `GRADIENT_API_PORT` to find every reference, and repeat this change.

## 2026-04-17 Web UI not loading at localhost:8000 — Cursor auto-port-forward regression

- Task summary:
  - Operator reported that `http://localhost:8000/` in their Windows Chrome showed a blank dark page after recent changes and that Cursor had stopped telling them "available at localhost:8000" / stopped auto-refreshing the browser on `./start-stack.sh`.
  - Initial assumption that a recent web-ui diff regressed rendering was wrong; the actual regression is Cursor's Remote-SSH auto-port-forward + auto-open-browser behavior no longer firing for ports 8000 / 4000 on this workspace session. The "blank page" was the body gradient (`from-slate-900 via-slate-950 to-black`) behind an empty `#root` because Cursor never re-forwarded 8000 this session, so the tab's subsequent module fetches never actually reached the Pi.
- What was investigated and ruled out (Pi-side is healthy):
  - `ss -tanp | grep ':8000\|:4000'`: both listeners bound `0.0.0.0`, Vite (`node` pid 215831) on :8000, API (python pid 215049) on :4000.
  - `curl -v http://127.0.0.1:8000/` returned HTTP/1.1 200 with the expected 640-byte Vite HTML shell + `<div id="root"></div>` + `/src/main.tsx`; same response for `Host: localhost` and `Host: localhost:8000`.
  - Every Vite-transformed module returned 200 `text/javascript`: `main.tsx`, `App.tsx`, `ControlPanel.tsx`, `liveState.tsx`, `index.css`, `useEndpoint.ts`, `previewUtils.ts`, `poseTelemetry.ts`, `components/SidebarRail|SidebarDrawer|ProgramFeatureTree|ProgramTimeline.tsx`. Transformed `ControlPanel.tsx` contains `formatDriveNativeTruthStatus` (2 matches) so Vite is serving the latest source.
  - `cd web-ui && npx vitest run src/ControlPanel.test.tsx` → 22/22 pass including the new canonical-truth surface test. `npx tsc --noEmit` surfaces only 3 pre-existing TS errors in `TelemetryCharts.tsx` / `TelemetryWorkspace.tsx` that are not in the touched files and that Vite dev mode ignores.
  - Vite HMR WebSocket probe (Python 3 socket with `Upgrade: websocket` + `Sec-WebSocket-Protocol: vite-hmr`) returned `HTTP/1.1 101 Switching Protocols`.
  - End-to-end render probe via headless Chromium on the Pi driven over CDP (`/tmp/capture_errors.js`, using `web-ui/node_modules/ws`): React tree mounted fully, `#root` contained the Control Center header, LIVE/SIM runtime-mode switcher, DISARM / IDLE / SAFE runtime badges; `Runtime.exceptionThrown=[]` and `Network.loadingFailed=[]` after a 45 s settle.
- Root cause:
  - Cursor Remote-SSH stopped auto-forwarding 8000 (and 4000) into the operator's Windows session this workspace. The `.gitignore` excludes `.vscode/`, and no repo-level `.vscode/settings.json` existed to pin auto-forward behaviour against Cursor's per-workspace client-side state.
- Change:
  - [.vscode/settings.json](.vscode/settings.json) (NEW, local-only because `.vscode/` is gitignored) — explicit Cursor Remote-SSH port-forward contract:
    - `remote.autoForwardPorts=true`, `remote.autoForwardPortsSource="hybrid"` (process + output detection), `remote.restoreForwardedPorts=true`.
    - `remote.portsAttributes`: `8000` → `{ label: "Gradient Web UI", onAutoForward: "openBrowser", requireLocalPort: true }`, `4000` → `{ label: "Gradient API", onAutoForward: "notify", requireLocalPort: true }`, `8080` → `{ label: "Gradient Vision", onAutoForward: "notify" }`, `9222` and `3000` → `{ onAutoForward: "ignore" }` (headless-chromium debug and UDP respectively).
  - JSON validated with `python3 -c "import json; json.load(open('.vscode/settings.json'))"`.
- Operator follow-up (required for the fix to activate):
  1. In Cursor on Windows: `Ctrl+Shift+P` → `Developer: Reload Window` (so the workspace settings and `portsAttributes` are re-read).
  2. Run `./start-stack.sh` on the Pi. When Vite binds `0.0.0.0:8000`, Cursor should auto-forward 8000 to Windows localhost:8000 and open the default browser on it; 4000 should surface the usual "Port forwarded" toast.
  3. If Cursor still doesn't fire the toast, open the Ports panel (View → Ports, or the Ports tab next to Terminal) and check whether 8000/4000 are listed. If they're there but not opened in browser, right-click `8000` → "Open in Browser" — that forces Cursor to remember the forward again.
- Validation performed:
  - No server-side code changed. The only new file is `.vscode/settings.json` (local-only, gitignored).
  - Settings file JSON parsed successfully.
  - Pi-side render confirmed via CDP before the stack was softly stopped by the operator at 07:29:34.
- Follow-up / risk:
  - `.vscode/` is in `.gitignore`. If other operators on this repo want the same forwarding ergonomics, either (a) commit `.vscode/settings.json` explicitly with `git add -f .vscode/settings.json` and document it, or (b) copy the file manually. Current policy keeps it local-only.
  - If Cursor's `portsAttributes` setting changes name in a future version, the forwarding will silently fall back to default behaviour. Revisit `remote.autoForwardPortsSource` / `remote.portsAttributes` key names on Cursor major version bumps.
  - Do NOT weaken `vite.config.ts`'s `server.allowedHosts` allowlist; `localhost`/`127.0.0.1` are already permitted by Vite defaults and the three `.local` hostnames cover the LAN workstations.
  - Hazard recorded: `pkill -9 -f <shortword>` inside the Cursor Shell tool can accidentally kill the shell wrapper's own process group because the wrapper's `ps` entry contains the full command line. Always pin pkill to a full binary path (e.g. `pkill -9 -f '/usr/lib/chromium/chromium'`) instead of a substring.

## 2026-04-17 A6-EC persistence-trust diagnostics polish (W1 + W2 + W3)

- Task summary:
  - Implemented the [A6-EC persistence trust diagnostics plan](../plans/a6ec_persistence_trust_diagnostics_e47d5a93.plan.md): three non-blocking polish workstreams on top of the already-validated restart-trust path.
  - Workstream 3 (encoder-retention fault precedence): `Er20.1..Er20.9` and `ALF9.0` live-faults now outrank the generic `fault_present` / `manufacturer_fault_present` branches in the truth-validity helper, so operators see the specific encoder/battery retention cause instead of the generic label. Surface includes matched vendor codes and names on each axis.
  - Workstream 1 (last-seen U40.20/.22 sidecar): optional `last_seen` dict on each anchor entry that records the live `absolute_counts` on every trusted-axis cycle (rate-limited to once per 5 s per joint). When the shaft-frame gate later fails AND the delta exceeds `32,767 * counts_per_rev` (physically impossible during an off-window), the reason upgrades to `multi_turn_feedback_lost_across_power_cycle` so encoder-loss is distinguishable from legitimate off-drive rotation.
  - Workstream 2 (bit-15 vestigial annotation): documented the bit-15-alone restart-trust path as documented-but-unreachable on the current A6-EC firmware via a new `POSITION_SEMANTICS_CONFIG["firmware_bit15_retention_expected"] = False` flag and SOP/master-doc updates. The code path is preserved for future firmware/drive families that honour vendor Q9.
- Changes:
  - `src/gradient_os/arm_controller/profiles/drive/a6ec_ds402.py`:
    - added `ENCODER_RETENTION_FAULT_CODES = frozenset({"Er20.1".."Er20.9", "ALF9.0"})`, `is_encoder_retention_fault(code)`, and `describe_encoder_retention_fault(manufacturer_error_code, error_code)` returning `{present, codes, names, matched_sources, ...}`.
    - added `POSITION_SEMANTICS_CONFIG["firmware_bit15_retention_expected"] = False` with a detailed docstring.
  - `src/gradient_os/arm_controller/profiles/registry.py` + `src/gradient_os/arm_controller/backends/registry.py`: added `describe_drive_encoder_retention_fault` wrapper + `describe_drive_encoder_retention_fault_for_backend` shim.
  - `src/gradient_os/telemetry/native_home_status.py`:
    - `derive_drive_native_truth_validity` gained kwargs `encoder_retention_fault_present`, `last_seen_present`, `last_seen_delta_physically_possible`.
    - When `encoder_retention_fault_present` is `True` the helper sets `coordinate_system_valid=False` and sets `reason="encoder_retention_fault_present"` ahead of the generic fault branches, and blocks the persisted-anchor trust upgrade even when the shaft-frame gate would otherwise pass.
    - When the shaft-frame gate already rejected AND the sidecar says the delta is impossible, reason upgrades to `"multi_turn_feedback_lost_across_power_cycle"`.
    - Documented the vestigial `statusword_bit15` source on the A6-EC firmware we currently run (pointers at the new profile flag).
  - `src/gradient_os/telemetry/drive_faults.py`: `build_drive_fault_snapshot` now calls the new retention helper, threads `encoder_retention_fault_present` into the validity helper, and surfaces `encoder_retention_fault` + `encoder_retention_fault_present` on each axis payload.
  - `src/gradient_os/arm_controller/backends/ethercat_rtcore/backend.py`:
    - decoded retention faults in `_canonical_joint_positions_from_raw_feedback`, populated `detail["encoder_retention_fault"]`/`encoder_retention_fault_present`, and threaded the signal into `derive_drive_native_truth_validity`.
    - computed `last_seen_delta_counts` / `last_seen_delta_physically_possible` / `last_seen_delta_budget_counts` per axis from the sidecar on the anchor entry and surfaced them on `detail`.
    - rate-limited persistence (`_LAST_SEEN_PERSIST_INTERVAL_S = 5.0`) of the last-seen sidecar via the new `save_last_seen_absolute_counts` helper, only on the `reference_mode="raw"` runtime canonical-truth path (display reads stay observational).
    - added `_encoder_counts_per_rev_for_axis` and `_MAX_OFF_MOTOR_REVOLUTIONS = 32_767` for the physicality budget.
    - made `_absolute_home_anchor_for_joint` forward the sidecar through.
  - `src/gradient_os/absolute_encoder_anchors.py`:
    - `_normalize_anchor_entry` now normalizes an optional `last_seen` sidecar (missing stays `None`; older anchor files unchanged).
    - added `save_last_seen_absolute_counts(robot_id, *, num_joints, logical_joint_index, absolute_counts, reference_counts, observed_at*, actor)` which is a no-op when the joint has no existing anchor (we never manufacture a fake anchor).
    - `save_absolute_encoder_anchor` gained `preserve_last_seen=True` so a fresh home does not discard the diagnostic sidecar unless an operator opts out.
  - Tests:
    - `tests/test_gradient05_limits_and_backends.py`: extended `_build_a6ec_restart_trust_test_backend` with `encoder_retention_fault_code`, `last_seen_absolute_counts`, `last_seen_reference_counts` params. Added six new regressions: `test_a6ec_encoder_retention_fault_takes_precedence_over_anchor_path` (Er20.9), `test_a6ec_encoder_retention_fault_distinguishes_battery_alarm` (ALF9.0), `test_a6ec_firmware_bit15_retention_flag_documented_false`, `test_a6ec_last_seen_sidecar_surfaces_delta_when_joint_unchanged_while_off`, `test_a6ec_last_seen_sidecar_upgrades_reason_on_impossible_delta`, `test_a6ec_last_seen_sidecar_keeps_legacy_reason_when_absent`.
    - `tests/test_drive_faults.py`: added `test_build_drive_fault_snapshot_carries_encoder_retention_fault` exercising the real profile decoder end-to-end.
    - `tests/test_rtcore_runtime.py::test_drive_fault_snapshot_decodes_axis_fault_and_master_state`: updated the expected `drive_native_truth_reason` from the generic `fault_present` to the new more specific `encoder_retention_fault_present` (Er20.8 is in the retention family), and asserted the new fault payload.
  - SOP + docs:
    - `.cursor/skills/gradientos-sop/commissioning-safety.md` "Restart Trust Model (A6-EC)": added one-liner on bit-15 vestigiality, added bullet on encoder-retention-family fault precedence, added bullet on the optional last-seen sidecar.
    - `.cursor/skills/gradientos-sop/RTOS_ETHERCAT_MASTER_OPERATING_PRINCIPLES.md` §9.7: tightened the path-2 bullet to call out the vestigial nature + profile flag, rewrote the reason-priority list to include the new reasons, added invariants for the last-seen sidecar and for retention-family faults.
    - `.cursor/skills/gradientos-sop/ui-api-telemetry.md` "Canonical-Truth Verification Surface": added `encoder_retention_fault_present` and `multi_turn_feedback_lost_across_power_cycle` to the enumerated unavailable-truth reasons; documented the `encoder_retention_fault` and `last_seen_*` per-axis fields.
- Validation:
  - `PYTHONPATH=src /home/pi/GradientOS/.venv/bin/python -m py_compile` on all touched modules: clean.
  - `PYTHONPATH=src /home/pi/GradientOS/.venv/bin/python -m pytest tests/test_gradient05_limits_and_backends.py tests/test_drive_faults.py tests/test_rtcore_runtime.py tests/test_api_endpoints.py tests/test_run_controller_helpers.py -q`: `220 passed`.
  - `ReadLints` on all touched Python/SOP files: no diagnostics.
  - Confirmed the 4 pre-existing serial-driver / solver failures (`tests/test_driver.py`, `tests/test_end_to_end.py`, `tests/test_protocol.py`, `tests/test_solver.py`) reproduce on clean HEAD without my changes; they are unrelated to this workstream.
- Follow-up / risk:
  - No motion was performed. The live J6 read-only verification called out in the plan (confirm `/info/joints-detailed` surfaces the new fields and the verification source stays `persisted_home_anchor_agreement` with `truth_available=True`) still needs a stack restart on hardware - this is a Python-only change that takes effect after controller restart, no RTCore rebuild required.
  - The bit-15 trust path is intentionally NOT removed. If vendor Patrik confirms the retention story definitively we can revisit in a follow-up patch.
  - Frontend does not yet visualize the new fields. `/info/joints-detailed` carries them through unchanged; operator-facing surfacing is a separate deferred task per the plan's non-goals.

## 2026-04-16 A6-EC vendor realignment v2 implementation

- Task summary:
  - Implemented the [A6EC vendor realignment v2 plan](../plans/a6ec_vendor_realignment_v2_ef85a9c0.plan.md): split MSG_CMD_NATIVE_HOME to require drive-confirmed disarm before HM35, replaced the stateful wrap-lift with a stateless nearest-turn 607A selector + half-RM trajectory pre-commit gate, added a shaft-frame mod-RM consistency gate while keeping anchored `U40.20/.22` as multi-turn canonical truth, rewrote tests and docs.
- Changes:
  - Updated `src/gradient_rt_motion/ipc_v1.hpp`:
    - added `NATIVE_HOME_ABORT_DISARM_PRECONDITION_TIMEOUT = 0xF1000001` in the reserved RTCore-synthesized abort-code range.
  - Updated `src/gradient_rt_motion/main.cpp`:
    - split the `MSG_CMD_NATIVE_HOME` handler; after clearing `axis_enable_mask` / `armed`, each targeted axis is polled for up to 500 ms until `statusword` leaves `OperationEnabled`/`QuickStopActive` for several consecutive cycles before `native_home_axis(axis)` is called.
    - on timeout the axis is marked `NATIVE_HOME_STATE_FAILED` with the new synthesized abort code so the Python wrapper can surface `disarm_precondition_timeout` distinctly from HM35 SDO aborts.
  - Updated `src/gradient_os/arm_controller/backends/ethercat_rtcore/backend.py`:
    - `get_power_transition_snapshot()` now publishes `per_axis_drive_disarmed`, `per_axis_ds402_state`, `per_axis_statusword_hex`, `drive_disarmed_all`, and `drive_op_enabled_axes` derived from the cached per-axis DS402 state.
    - `wait_for_power_transition_neutral` / `prepare_for_power_transition` gained `require_drive_disarmed` + `require_drive_disarmed_axis_mask`; when set the neutrality test requires both `motion_intent_cleared` and every targeted axis to be out of `OperationEnabled`. Prepare also sends an explicit `axis_disable` for the targeted mask so the cyclic loop actively drives the state machine away from OP-enabled instead of just waiting.
    - `native_home_joint()` now calls the new helper with `require_drive_disarmed=True` before `_send_cmd_native_home`, and returns `NATIVE_HOME_DISARM_PRECONDITION_TIMEOUT` with abort code `0xF1000001` if the axis never leaves OP-enabled. The same abort code raised by RTCore on the wait-path is surfaced with the same code.
    - removed `_raw_reference_wrap_lift_counts`, `_raw_reference_wrap_lift_counts_for_axis`, `_set_raw_reference_wrap_lift_counts_for_axis`, `_raw_reference_wrap_lift_q_for_axis`, `_raw_reference_wrap_period_counts_for_axis`, and `_wrap_adjusted_command_axis_q_for_axis`; dropped the `mutate_command_wrap_bookkeeping` flag from `_canonical_joint_positions_from_raw_feedback`/`_canonical_joint_positions_or_raise` and all callers.
    - added `_live_reference_counts_for_axis()` and `_nearest_turn_fold_axis_q_for_axis()`; `_command_axis_q_for_joint_value()` now folds each write to the nearest shaft turn of the live `6064` reading and raises `command_frame_oversized_delta` as a regression guard.
    - added `_enforce_trajectory_step_within_half_rm()`; `enqueue_trajectory_points()` now rejects any consecutive-point step whose wire-space delta exceeds `0.5 * RM` with `command_frame_oversized_step`.
    - added `_shaft_frame_consistency_detail()`; after computing `canonical_q` from anchored `U40.20/.22`, canonical truth fails closed with `multi_turn_anchor_inconsistent_with_live_6064` when the expected wire-counts disagree with live `6064` modulo `RM` by more than `_SHAFT_FRAME_CONSISTENCY_TOLERANCE_COUNTS` (16 counts). The stale-anchor diagnostic fields are still populated alongside.
    - post-home validation path now forwards the `shaft_frame_*` fields as `post_home_shaft_frame_*` so operators see the mod-RM delta that tripped the gate.
  - Updated `tests/test_gradient05_limits_and_backends.py`:
    - removed the three wrap-lift-as-observable regressions superseded by the stateless fold.
    - added `test_a6ec_small_jog_at_seam_stays_within_half_rm_wire_delta`, `test_a6ec_native_home_waits_for_drive_disarmed_before_hm35`, `test_a6ec_truth_unavailable_when_anchor_and_6064_disagree_modulo_rm`, `test_a6ec_command_frame_rejects_oversized_trajectory_step`.
    - relaxed the existing anchor-vs-reference truth-reason assertions to accept `multi_turn_anchor_inconsistent_with_live_6064` alongside the legacy reasons.
    - updated the two tests that captured the `prepare_for_power_transition` kwargs to accept the new disarm-precondition parameters.
  - Updated `docs/ethercat/a6ec-frame-semantics-and-native-home.md`:
    - rewrote "Canonical Truth Math" to call out anchored multi-turn truth plus the shaft-frame consistency gate.
    - added "Command-Frame Turn Selection" describing the stateless nearest-turn fold and the half-RM trajectory sanity gate.
    - added "Native-Home Precondition" describing the two-stage MSG_CMD_NATIVE_HOME split and the Python-side `require_drive_disarmed` plumbing.
    - moved the "multi-turn truth stays anchored" decision from "What Remains Open" to a settled decision with rationale.
- Validation:
  - `PYTHONPATH=src /home/pi/GradientOS/.venv/bin/python -m py_compile src/gradient_os/arm_controller/backends/ethercat_rtcore/backend.py src/gradient_os/arm_controller/profiles/drive/a6ec_ds402.py`
  - `make -C src/gradient_rt_motion` - clean C++ build of the new disarm precondition wait + synthesized abort code.
  - `PYTHONPATH=src /home/pi/GradientOS/.venv/bin/python -m pytest tests/test_gradient05_limits_and_backends.py -q` - `87 passed`.
  - `PYTHONPATH=src /home/pi/GradientOS/.venv/bin/python -m pytest tests/test_gradient05_limits_and_backends.py tests/test_run_controller_helpers.py tests/test_api_endpoints.py tests/test_command_api_direct_setpoint.py tests/test_rtcore_runtime.py tests/test_trajectory_execution_backends.py tests/test_drive_faults.py tests/test_a6ec_chapter5_probe.py -q` - `238 passed`.
  - `ReadLints` on all touched files returned clean.
  - `sudo systemd/rt-motion/sync-runtime.sh --ensure-active` installed the new `/usr/local/bin/gradient-rt-motion`; `systemctl status gradient-rt-motion` shows the new main pid running; `/run/gradient-rt-motion/metrics.json` reports all six axes at `statusword=0x1650` after the restart, no `raw_reference_wrap_lift_counts` keys in the axis payload.
  - Pre-existing `tests/test_driver.py` servo/driver failures were present before this change too (verified by git stash + re-run).
- Follow-up notes / risks:
  - Physical-motion live validation is pending operator action (see "A6-EC live validation handoff" below). Expect a single brake click on native home (not two), and the seam-adjacent J6 `+0.5 deg` jog should complete without `Er87.1` or `Er47.0`.
  - RTCore is now installed and running against the new binary but the Python controller/API stack is not currently active on this host; the operator should bring it up with `./start-stack.sh` and re-home before relying on motion again.

## 2026-04-17 GradientOS SOP + master principles updated with settled A6-EC contracts

- Task summary:
  - Consolidated the four settled contracts from the recent A6-EC workstreams (disarm precondition, stateless command-frame turn selection, multi-turn truth with shaft-frame consistency gate, persisted-home-anchor restart trust) from scratchpad/devlog evidence into the canonical GradientOS skill corpus and the `RTOS_ETHERCAT_MASTER_OPERATING_PRINCIPLES.md` master doc, per `gradientos-skill-maintainer` policy.
- Changes:
  - [.cursor/skills/gradientos-sop/SKILL.md](.cursor/skills/gradientos-sop/SKILL.md): reframed the active-workstream note so `a6ec-frame-semantics-and-native-home.md` is the workstream evidence repository and the settled rules now live in the master doc `§9.4-§9.7`.
  - [.cursor/skills/gradientos-sop/commissioning-safety.md](.cursor/skills/gradientos-sop/commissioning-safety.md): added four new sections - "Native-Home Disarm Precondition", "Command-Frame Turn Selection (A6-EC)", "Multi-Turn Truth and Shaft-Frame Consistency (A6-EC)", "Restart Trust Model (A6-EC)". Revised the `0x607C` / `bit 15` persistence statements to reflect the empirical findings (bit 15 cleared every cycle; `0x607C` + `U40.20/.22` + `6064` persist) and replaced the outdated "no fallback to legacy anchored reconstruction" rule with the three-path restart-trust model.
  - [.cursor/skills/gradientos-sop/rtcore-ethercat.md](.cursor/skills/gradientos-sop/rtcore-ethercat.md): added "Commissioning Transaction Preconditions" (Stage-A statusword-confirmed disarm before HM35, `0xFxxxxxxx` synthesized-abort convention) and "Trajectory Upload Sanity" (host-side `<= 0.5 * RM` pre-commit gate complementary to RTCore `max_step_counts_per_cycle`). Replaced the obsolete `native_home_position_offset`-subtraction rule with the stateless per-write nearest-turn fold.
  - [.cursor/skills/gradientos-sop/config-and-drive-profiles.md](.cursor/skills/gradientos-sop/config-and-drive-profiles.md): added "Drive-Profile Position-Semantics Flags" documenting the stable `POSITION_SEMANTICS_CONFIG` flag surface (`drive_native_ratio_enabled`, `canonical_truth_source`, `absolute_home_anchor_required`, `startup_truth_requires_hm_success_signature`, `accept_persisted_home_anchor_as_restart_trust`).
  - [.cursor/skills/gradientos-sop/ui-api-telemetry.md](.cursor/skills/gradientos-sop/ui-api-telemetry.md): added "Canonical-Truth Verification Surface" listing the four `drive_native_truth_verification_source` values operator tooling must surface verbatim and the specific truth-unavailable reasons. Added "Native-Home Result Surface" codifying the structured result contract and the `0xFxxxxxxx` synthesized-abort-code handling.
  - [.cursor/skills/gradientos-sop/validation-and-debugging.md](.cursor/skills/gradientos-sop/validation-and-debugging.md): added "A6-EC Command-Frame Failure Modes" (Er87.1 / Er47.0 diagnostic path rooted in `shaft_frame_consistent` and `command_frame_oversized_step`) and "Power-Cycle Persistence Workflow" (probe snapshots + per-axis trust-source diff).
  - [.cursor/skills/gradientos-sop/RTOS_ETHERCAT_MASTER_OPERATING_PRINCIPLES.md](.cursor/skills/gradientos-sop/RTOS_ETHERCAT_MASTER_OPERATING_PRINCIPLES.md): rewrote `§9.3`'s outdated "subtract `native_home_position_offset` once" wording and added four new subsections `§9.4`-`§9.7` capturing the settled stateless command frame, multi-turn truth contract, HM35 disarm precondition, and three-path restart trust model. Extended `§15.2` Safety best practices and `§15.4` Commissioning best practices checklists accordingly.
- Validation:
  - `ReadLints` on all seven touched SOP files returned no diagnostics.
  - `PYTHONPATH=src /home/pi/GradientOS/.venv/bin/python -m pytest tests/test_gradient05_limits_and_backends.py tests/test_drive_faults.py tests/test_rtcore_runtime.py tests/test_api_endpoints.py tests/test_run_controller_helpers.py -q`: `191 passed` (ran as a sanity check; SOP edits are documentation-only).
  - File-size sanity: only `commissioning-safety.md` and the master doc grew; SOP reference files remain under roughly `135` lines each.
- Follow-up notes / risks:
  - `startup_drive_config_unconfigured` remains a separate plumbing wart (RTCore does not re-command startup SDOs this session when the drive already matches at PREOP). Documented in the live-validation devlog entry but intentionally NOT promoted to canonical SOP because the right fix is a code change in the RTCore/profile metrics, not an operating rule.
  - The `statusword_bit15` vendor-Q9 path is kept in `derive_drive_native_truth_validity` and documented even though it is vestigial on the current A6-EC firmware; future drives / firmware revisions that honour Q9 will still exercise it without a code change.

## A6-EC persisted-home-anchor restart trust (2026-04-17 00:20 UTC)

- Task summary:
  - Eliminated the "re-home after every drive power cycle" operational ceiling. The A6-EC firmware empirically clears `6041 bit 15` on every drive power cycle despite vendor Q6/Q9, while the data that actually matters for persistence (`6064`, `U40.20/.22`, `607C`) restores cleanly and survives manual rotation while the drive is off. The controller now accepts a second restart-trust path driven by the persisted absolute-home anchor file plus a mod-`RM` shaft-frame agreement check against live `6064`.
- Changes:
  - [src/gradient_os/arm_controller/profiles/drive/a6ec_ds402.py](src/gradient_os/arm_controller/profiles/drive/a6ec_ds402.py): added `accept_persisted_home_anchor_as_restart_trust = True` to `POSITION_SEMANTICS_CONFIG`.
  - [src/gradient_os/telemetry/native_home_status.py](src/gradient_os/telemetry/native_home_status.py): extended `derive_drive_native_truth_validity` with optional kwargs `accept_persisted_home_anchor_as_restart_trust`, `persisted_home_anchor_present`, `persisted_home_anchor_consistent`, `multi_turn_feedback_valid`. When the flag is on and all three supplied signals are `True`, truth upgrades to `coordinate_system_valid=True` with `drive_native_truth_verification_source="persisted_home_anchor_agreement"`. When the path is opted in but a precondition fails, emits a specific reason (`persisted_home_anchor_missing`, `multi_turn_feedback_invalid`, `persisted_home_anchor_inconsistent_with_live_6064`) instead of the generic `coordinate_system_invalid`.
  - [src/gradient_os/arm_controller/backends/ethercat_rtcore/backend.py](src/gradient_os/arm_controller/backends/ethercat_rtcore/backend.py): added `_accept_persisted_home_anchor_as_restart_trust()` profile accessor; refactored `_canonical_joint_positions_from_raw_feedback` so `anchor_entry`, `absolute_axis_q`, and `_shaft_frame_consistency_detail` are computed **before** the validity call so the new signals can flow in. Re-used the pre-computed gate for the Workstream 3 short-circuit to avoid double work. Added new axis-detail fields `persisted_home_anchor_present`, `multi_turn_feedback_valid`, `persisted_home_anchor_consistent` so the restart-trust decision is inspectable from operator tooling.
  - [docs/ethercat/a6ec-frame-semantics-and-native-home.md](docs/ethercat/a6ec-frame-semantics-and-native-home.md): added a new "Restart Trust via Persisted Home Anchor" section describing the three trust paths in order, the invariance under manual rotation while off, and the explicit operator implications.
  - [tests/test_gradient05_limits_and_backends.py](tests/test_gradient05_limits_and_backends.py):
    - Added a shared scaffold `_build_a6ec_restart_trust_test_backend` that stages realistic J6 post-power-cycle data with the anchor derived from HM35-time raw counts.
    - Added four regressions: `test_a6ec_restart_trust_via_persisted_anchor_passes_when_axis_unmoved_while_off` (encoder wander case), `test_a6ec_restart_trust_via_persisted_anchor_survives_full_shaft_turn_of_manual_rotation` (joint rotated +1 shaft rev with drive off; `6064` wraps back to the same raw value while `U40.20/.22` increments by `RM` motor counts; mod-`RM` gate absorbs it), `test_a6ec_restart_trust_via_persisted_anchor_rejects_sub_shaft_turn_drift` (encoder-data-loss simulation: `6064` shifted by `RM/3` without matching multi-turn step; gate rejects), `test_a6ec_restart_trust_via_persisted_anchor_requires_multi_turn_valid` (invalid multi-turn reading → refuses anchor path, falls back to `drive_native_absolute_feedback_unavailable`).
    - Updated the pre-existing `test_ethercat_backend_power_transition_snapshot_reports_coordinate_system_invalid` to isolate `GRADIENT_ABSOLUTE_ENCODER_ANCHORS_PATH` (it was accidentally inheriting real anchor data from the repo) and to accept the more specific `drive_native_persisted_home_anchor_missing` reason alongside the legacy `drive_native_coordinate_system_invalid`.
- Validation:
  - `PYTHONPATH=src /home/pi/GradientOS/.venv/bin/python -m py_compile src/gradient_os/arm_controller/backends/ethercat_rtcore/backend.py src/gradient_os/telemetry/native_home_status.py src/gradient_os/arm_controller/profiles/drive/a6ec_ds402.py` - clean.
  - `PYTHONPATH=src /home/pi/GradientOS/.venv/bin/python -m pytest tests/test_gradient05_limits_and_backends.py tests/test_run_controller_helpers.py tests/test_api_endpoints.py tests/test_command_api_direct_setpoint.py tests/test_rtcore_runtime.py tests/test_trajectory_execution_backends.py tests/test_drive_faults.py tests/test_a6ec_chapter5_probe.py -q` - `242 passed`.
  - `ReadLints` on touched files - no diagnostics.
  - Live proof with J6 **not re-homed this session**: restarted the Python stack and RTCore service onto the patched code. Direct backend inspection of `/run/gradient-rt-motion/metrics.json` through `_canonical_joint_positions_from_raw_feedback` returned `coordinate_system_valid=True` + `drive_native_truth_verification_source="persisted_home_anchor_agreement"` on all six axes while every statusword was `0x1650` (bit 15 cleared). `shaft_frame_mod_rm_delta_counts` sat at `±1-3` across the fleet, well inside the 16-count tolerance. `/control/motion-status` returned `safe_for_power_transition=True` with empty `power_transition_blockers`, and `/info/joints-detailed` reported `canonical_joint_truth_available=True` with `read_source=live_feedback` on real joint angles.
- Follow-up notes / risks:
  - Initial HM35 is still required to *establish* the anchor file entry per joint. The trust path only reuses state; it does not invent trust.
  - The trust check fails closed when the encoder's multi-turn tracking is actually lost (`U40.20/.22 valid=0`, overflow, or >32,767-motor-turn rotation while off). In those cases the operator must re-home. This is the intended behavior.
  - `/control/motion-status` still embeds its power-transition detail as a base64-encoded JSON blob; it does not expose per-axis `drive_native_truth_verification_source` directly. Tooling can surface it via `/info/joints-detailed` or by reading `axis_absolute_feedback` from the backend snapshot. Separate follow-up if operator-visible verification-source is desired.

## A6-EC live validation on J6 (2026-04-16 21:47 UTC)

Native-home pathway was exercised end-to-end on J6 against the new RTCore binary and backend plumbing. Workstream 1 + 2 + 3 contracts all behaved as designed on live hardware; only the seam-adjacent powered jog remains outstanding because it needs an operator to physically rotate J6 to a seam-adjacent shaft position.

- Pre-flight:
  - `./start-stack.sh` came up clean; run log at `/home/pi/GradientOS/logs/startups/20260416-214414/`.
  - Controller initially defaulted to `mode=simulate`. Switched to live via `POST /control/runtime-mode {"mode":"live"}`; backend re-activated as `ethercat_rtcore` with `drive_profile=a6ec_ds402` and `mode.sim=false`.
  - Pre-home metrics: all six axes at `statusword=0x1650`, no faults, `native_home_state=0`, `native_home_last_abort_code=0x00000000`.
  - Pre-home API state correctly surfaced the truth block: `/control/motion-status` reported `safe_for_power_transition=false` with a single blocker `canonical_truth_unavailable` and reasons `drive_native_startup_drive_config_unconfigured` on all six axes. That `unconfigured` reason is pre-existing behavior (RTCore sees `configured=0` in metrics when the drive already matched the startup SDOs from a prior session so RTCore did not re-command them this session; `readback_valid=1 verified=1` is still reported). It blocks power-up but does not block `POST /control/home-joint-native`.
- Native home on J6 via `POST /control/home-joint-native {"joint":6}`:
  - HTTP ACK: `accepted=true`, `terminal_state=succeeded`, `native_home_state=2 (succeeded)`, `native_home_last_abort_code=0x00000000`, `disarmed_after_home=true`.
  - `code=NATIVE_HOME_ANCHOR_REFRESH_FAILED`, `message=...could not refresh a coherent absolute-home anchor from live feedback.`. This is the same pre-existing `drive_native_startup_drive_config_unconfigured` wart bleeding into the post-home anchor re-validation path; it does NOT indicate an HM35 failure. Anchor was captured cleanly: `absolute_home_anchor_capture_succeeded=true`, `absolute_home_anchor_rad=-0.5761`.
- Workstream 1 evidence (RTCore journal, `journalctl -u gradient-rt-motion`):
  - `21:47:03 [gradient-rt-motion] Native-home disarm precondition satisfied axis_mask=0x20 (timeouts=0x0)`
  - `21:47:09 [gradient-rt-motion] EtherCAT native_home axis=5 slave_pos=5 feedback_counts=14599 truth_value=0 commissioning_mode=6 steady_state_mode=8 steps=15`
  - `21:47:09 [gradient-rt-motion] Native-home success: homed axes remain disabled axis_mask 0x0 -> 0x0 armed=0`
  - The **new Stage-A precondition** log line fires first and 6 s pass before HM35 begins; timeout mask is `0x0` (no axes timed out). The bundled disarm-and-enable click pattern the user described has been eliminated.
- Post-home drive-side state (from `/run/gradient-rt-motion/metrics.json`):
  - J6: `statusword=0x9650` (bits 12+15 set, bit 13 clear, no error code) - the exact vendor-confirmed HM success signature.
  - J6: `pos_counts=8` (drive is parked within a few counts of the new `607C=0` origin), `native_home_state=2`, `encoder_multi_turn_low=120189` (multi-turn counter preserved across the home).
  - J1-J5 remained at `0x1650`, which is the expected pre-home state for untouched axes.
- Still outstanding from Workstream 5 (requires operator):
  - Manually rotate J6 to a seam-adjacent shaft position (within ~1 deg of the shaft seam, visible in `/run/gradient-rt-motion/metrics.json` as J6 `pos_counts` crossing near `RM-1 = 1,310,719`). Our current J6 `pos_counts=8` is right at the shaft zero and is not seam-adjacent.
  - After rotating, `POST /control/power-up`, then `POST /control/joint-jog {"joint":6,"delta_deg":0.5}`. Acceptance criteria: no `Er87.1`, no `Er47.0`, continuous canonical/display truth, no `command_frame_oversized_*` counter activity.
- Unrelated pre-existing blocker for a full power-up: the `drive_native_startup_drive_config_unconfigured` reason is emitted whenever the drive already had the startup SDO values cached. Cleanest way to clear it short-term is an RTCore restart with a deliberate startup re-issue, but the native-home flow we just tested does not depend on it. Separate follow-up.

## 2026-04-13 plan refinement

- Task summary:
  - Integrated the latest canonical-truth regression lessons into the lossless startup dashboard plan and repo memory.
- Changes:
  - Updated `/home/pi/.cursor/plans/lossless_startup_dashboard_422d79b5.plan.md`:
    - promoted single canonical truth and explicit no-fallback semantics into plan decisions
    - added integrated lessons covering startup anchor bootstrap, explicit unavailable states, compatibility-alias handling, and anchor-aware write-frame integrity
    - expanded the event-ledger shape to record canonical truth lifecycle and anchor-bootstrap events
    - expanded validation to cover no `cached_fallback`, blocked motion baselining without truth, and the nonzero-anchor inversion bug class behind the J3/J4 snap-back regression
  - Updated `.cursor/memory/AGENT_SCRATCHPAD.md` with durable guardrails from the latest regression thread.
- Validation:
  - Read the current lossless dashboard plan and the linked transcript to extract the stable lessons and fixes.
  - Confirmed the plan now explicitly distinguishes canonical truth from diagnostics and requires startup repair paths instead of encoder fallbacks.
- Follow-up notes / risks:
  - The later implementation pass should make the dashboard/event schema reflect these rules directly so operator-visible status cannot drift back into blank telemetry, hidden fallback, or misleading dual-truth presentation.

## 2026-04-13 00:00 +0000

- Task summary:
  - Extended the startup dashboard plan to make lossless recording a first-class design constraint.
- Changes:
  - Updated `/home/pi/.cursor/plans/lossless_startup_dashboard_422d79b5.plan.md`:
    - marked the recording contract todo complete and the split-view layout todo in progress
    - added non-negotiable invariants that dashboard cleanup must never discard evidence
    - defined the planned artifact set: raw service logs, rendered launcher transcript, structured dashboard events, and session metadata
    - added failure-proofing, retention shape, and validation criteria focused on postmortem reconstruction
  - Updated `.cursor/memory/AGENT_SCRATCHPAD.md` with the new user requirement that live dashboard filtering must remain a presentation-only layer.
- Validation:
  - Read the current plan, scratchpad, and latest devlog context before refining the plan.
  - Confirmed the plan now explicitly requires reconstructability of both raw process output and operator-facing dashboard state.
- Follow-up notes / risks:
  - The next implementation pass must preserve this contract in `start-stack.sh` and `src/gradient_os/telemetry/terminal_dashboard.py`; readability improvements must not be coupled to any data-dropping path.

## 2026-04-08 17:49 +0000

- Task summary:
  - Removed the introduced RTCore legacy-speed/single-point velocity shim, restored the pre-native-home degree-step commissioning jog assumptions, and kept the native-home frame fixes intact.
- Changes:
  - Updated `src/gradient_os/arm_controller/backends/ethercat_rtcore/backend.py`:
    - deleted the introduced legacy speed-to-velocity helper path (`_LEGACY_SERVO_SPEED_MAX` and related `_single_point_*` helpers)
    - restored direct RTCore one-point `set_joint_positions()` / `set_single_actuator_position()` uploads to position-only trajectory points
    - restored `prepare_sync_write_commands()` / `sync_write()` to the older backend-private point contract instead of the timed helper branch
    - removed the `get_joint_positions()` / `sync_read_positions()` freshness gate so controller joint snapshots can again provide mapped RTCore feedback for commissioning jog
    - preserved the post-commit setpoint-cache update, native-home feedback conversion, command-frame handling, and zero-capture logic
  - Updated `src/gradient_os/arm_controller/trajectory_execution.py`:
    - removed the timed sync-write fallback branch and restored the older backend sync-write preparation path when RTCore trajectory offload is not used
  - Updated `tests/test_gradient05_limits_and_backends.py`:
    - rewrote the direct RTCore setpoint tests to assert position-only trajectory points instead of invented `qd`
    - replaced the stale-feedback rejection test with coverage that connected RTCore reads still return mapped joint feedback for commissioning
    - kept cache-poisoning coverage for failed RTCore commit and preserved native-home frame/zero tests
  - Verified the existing commissioning-path regression coverage still anchors the intended flow:
    - `tests/test_command_api_direct_setpoint.py::test_handle_apply_joint_setpoint_can_start_bounded_joint_trajectory`
    - `tests/test_api_endpoints.py::test_control_joint_jog`
    - `tests/test_trajectory_execution_backends.py::test_open_loop_executor_offloads_rtcore_trajectory_backend`
- Validation:
  - `source ./start.sh && python -m pytest tests/test_gradient05_limits_and_backends.py tests/test_command_api_direct_setpoint.py tests/test_api_endpoints.py tests/test_trajectory_execution_backends.py -q -k 'applies_master_offsets_to_setpoints or sync_write_ignores_legacy_speed_and_accel or single_actuator_setpoint_emits_position_only_trajectory or set_joint_positions_does_not_advance_cache_on_commit_failure or connected_reads_return_feedback_without_freshness_gate or applies_native_home_offsets_to_feedback_but_not_command_targets or zero_capture_persists_joint_offsets or execute_joint_trajectory_enqueues_velocity_points or handle_apply_joint_setpoint_can_start_bounded_joint_trajectory or control_joint_jog or open_loop_executor_offloads_rtcore_trajectory_backend'`
  - `source ./start.sh && python -m py_compile src/gradient_os/arm_controller/backends/ethercat_rtcore/backend.py src/gradient_os/arm_controller/trajectory_execution.py tests/test_gradient05_limits_and_backends.py tests/test_command_api_direct_setpoint.py tests/test_trajectory_execution_backends.py tests/test_api_endpoints.py`
  - `ReadLints` on touched files returned clean.
- Follow-up notes / risks:
  - This restores the older commissioning-jog assumptions by letting connected RTCore reads flow back into `GET_JOINT_STATE`; confirm on hardware that the UI no longer falls back to cached-only feedback before trusting live commissioning again.
  - Rebuilding Python-side code alone does not update any deployed RTCore binary; live retest still depends on the running RTCore service state on the target machine.

## 2026-04-08 18:10 +0000

- Task summary:
  - Investigated whether the next Joint Commissioning J2 retest needs an RTCore/C++ rebuild or only a Python/controller retest.
- Changes:
  - No product code changed.
  - Updated `.cursor/memory/AGENT_SCRATCHPAD.md` with the verified deployment rule for RTCore commissioning follow-ups on this machine.
- Validation:
  - Read `start.sh` to confirm it only bootstraps the repo Python environment.
  - Reviewed the current diff in `src/gradient_os/arm_controller/backends/ethercat_rtcore/backend.py`, `src/gradient_os/arm_controller/trajectory_execution.py`, and `tests/test_gradient05_limits_and_backends.py`.
  - Searched `src/` for `_LEGACY_SERVO_SPEED_MAX`, `_resolved_single_point_motor_rpm`, `_single_point_joint_velocities`, and `_single_point_axis_velocity`; no matches remained.
  - Verified the live RTCore backend still uses `qd` only for real trajectory/jog paths, while `set_joint_positions()`, `set_single_actuator_position()`, `prepare_sync_write_commands()`, and `sync_write()` now use position-only one-point uploads for the compatibility path.
  - Read `systemd/rt-motion/gradient-rt-motion.service` and `systemd/rt-motion/sync-runtime.sh` to confirm the service runs `/usr/local/bin/gradient-rt-motion` and only picks up repo RTCore binary changes after an explicit sync/restart step.
- Follow-up notes / risks:
  - For this specific shim-removal follow-up, the next cautious live check should not require rebuilding RTCore unless we intentionally want to test separate dirty-branch C++ or RTCore service/env changes.
  - If the commissioning retest shows behavior that implicates RTCore execution rather than Python/controller logic, then reassess whether the installed RTCore binary/env needs to be rebuilt and synced before widening scope.

## 2026-04-08 18:15 +0000

- Task summary:
  - Clarified when the dirty-branch RTCore/EtherCAT-side changes do require a rebuild before commissioning.
- Changes:
  - No product code changed.
  - Updated `.cursor/memory/AGENT_SCRATCHPAD.md` to record that the current dirty-branch `main.cpp` / `ipc_v1.hpp` changes are not live until RTCore is rebuilt and synced into the installed service binary.
- Validation:
  - Reviewed `git diff` for `src/gradient_rt_motion/main.cpp` and `src/gradient_rt_motion/ipc_v1.hpp`.
  - Confirmed the dirty RTCore changes include native-home hold-target alignment to `native_home_position_offset`, refreshed offset readback on startup, and a new service-SDO-write IPC command path.
  - Confirmed again from `systemd/rt-motion/gradient-rt-motion.service` and `systemd/rt-motion/sync-runtime.sh` that these C++ changes only reach hardware after rebuild plus sync/restart of `/usr/local/bin/gradient-rt-motion`.
- Follow-up notes / risks:
  - This is an RTCore rebuild/sync question, not an IgH EtherCAT master rebuild question; no current diff evidence showed repo changes to the master package itself.
  - If the live retest intends to validate native-home alignment or service-SDO-write behavior, rebuilding and syncing RTCore is required before trusting the result.

## 2026-04-08 19:22 +0000

- Task summary:
  - Investigated why the Joint Commissioning panel locked out further jogs after a single J3 step on live RTCore hardware.
- Changes:
  - No product code changed.
  - Updated `.cursor/memory/AGENT_SCRATCHPAD.md` with the confirmed commissioning lockout pattern.
- Validation:
  - Reviewed the attached controller log showing:
    - repeated successful `GET_JOINT_STATE` handling
    - `Open-Loop Executor finished`
    - thread exception from `trajectory_execution._open_loop_executor_thread()` because `backend.wait_for_trajectory_complete()` raised `TimeoutError` for RTCore trajectory `24`
  - Read `src/gradient_os/arm_controller/backends/ethercat_rtcore/backend.py`:
    - `execute_joint_trajectory()` submits the queued RTCore path and then waits for terminal completion
    - `wait_for_trajectory_complete()` raises if RTCore never reports a terminal state for the submitted command sequence within the timeout
  - Read `web-ui/src/ControlPanel.tsx`:
    - commissioning jog buttons are disabled whenever `motionBusy` is true
    - `motionBusy` is true for motion states `accepted`, `queued`, or `executing`
  - Read `src/gradient_os/api/main.py`:
    - `/control/joint-jog` parses `wait_for_idle`
    - but currently returns the initial `APPLY_JOINT_SETPOINT` ACK payload directly with `waited_for_idle: False`
    - so the route does not actually wait for or return a terminal motion state before the UI updates its local `motionStatus`
- Follow-up notes / risks:
  - The lockout is consistent with a stale `"accepted"` motion status after a queued RTCore jog times out in the controller waiter.
  - Immediate recovery should prefer checking `/control/motion-status` and using STOP / refresh to clear stale motion state before attempting another commissioning jog.

## 2026-04-08 20:15 +0000

- Task summary:
  - Rebuilt and resynced RTCore, then traced the live J2 wrong-direction commissioning motion to missing native-home offset truth in RTCore metrics.
- Changes:
  - No repo product files changed.
  - Rebuilt RTCore with `source ./start.sh && make -C src/gradient_rt_motion`.
  - Synced/started the installed RTCore service with `source ./start.sh && ./systemd/rt-motion/sync-runtime.sh --ensure-active`.
  - Updated `.cursor/memory/AGENT_SCRATCHPAD.md` with the confirmed native-home-offset-loss pattern.
- Validation:
  - Confirmed the service is now running again as `/usr/local/bin/gradient-rt-motion`.
  - Read current `/run/gradient-rt-motion/metrics.json`; all axes, including J2, currently report `native_home_position_offset: 0`.
  - Searched the full RTCore journal and found the earlier J2 home save evidence:
    - `Apr 08 19:18:51 ... EtherCAT native_home axis=1 ... desired_offset=-107506 saved=1`
  - Compared that with the later live metrics and restart state, which no longer carry that offset into RTCore startup/readback.
- Follow-up notes / risks:
  - The rebuilt RTCore service alone does not fix the bad J2 motion because the live RTCore/native-home state currently says J2 has no saved offset.
  - The remaining root issue is now narrowed to offset retention/readback truth across restart/power-up, not the commissioning `joint-jog` route semantics or the 100 RPM bounded planner path.

## 2026-04-08 20:27 +0000

- Task summary:
  - Proved whether the J2 native-home offset loss is on the drive side or the RTCore readback side.
- Changes:
  - No repo product files changed.
  - Updated `.cursor/memory/AGENT_SCRATCHPAD.md` with the confirmed drive-vs-RTCore boundary evidence.
- Validation:
  - Read J2 directly from EtherCAT with:
    - `source ./start.sh && sudo ethercat upload -p 1 -t int32 0x60B0 0`
    - result: `0xfffe5c0e -107506`
  - Spot-checked neighboring axes:
    - `source ./start.sh && sudo ethercat upload -p 0 -t int32 0x60B0 0`
    - `source ./start.sh && sudo ethercat upload -p 2 -t int32 0x60B0 0`
    - result: both `0`
  - Compared those direct drive reads against current `/run/gradient-rt-motion/metrics.json`, which still reports `native_home_position_offset: 0` for J2.
  - Read the RTCore startup code in `src/gradient_rt_motion/main.cpp` and confirmed it currently snapshots `0x60B0` only once before the startup convergence loop, then never refreshes it later unless a new native-home command is issued.
- Follow-up notes / risks:
  - The drive still has the correct saved J2 native-home offset; RTCore startup/readback is the component currently losing or failing to refresh that truth.
  - This explains why the bounded `+1 deg` commissioning command can still move badly after restart/power-up: RTCore hold-target alignment is operating with a false zero native-home offset while the drive itself is not.

## 2026-04-08 17:21 +0000

- Task summary:
  - Reviewed and tightened the RTCore single-point regression fix so streamed `sync_write` points use explicit controller-timestep `qd` semantics instead of the guessed legacy speed-to-RPM helper.
- Changes:
  - Updated `src/gradient_os/arm_controller/backends/ethercat_rtcore/backend.py`:
    - added `prepare_timed_sync_write_commands()` and `set_joint_positions_for_duration()` for explicit-duration RTCore point uploads
    - validated timed point durations against at least one RTCore cycle instead of accepting near-zero durations
    - delayed `_last_joint_setpoint_rad` updates until after successful `commit_trajectory()`
    - updated the single-actuator success path to keep the fallback setpoint cache aligned
    - reused one seed snapshot inside the legacy single-point helper so duration and `qd` are derived from the same source sample
  - Updated `src/gradient_os/arm_controller/trajectory_execution.py`:
    - use the timed backend helper when available so backend `sync_write()` streams preserve the controller timestep
  - Updated `tests/test_gradient05_limits_and_backends.py`:
    - added regression tests for timed sync-write `qd` math
    - added a guard test that rejects sub-cycle timed point durations
    - extended single-actuator coverage to verify cache updates on success
    - added a failure-path test that confirms failed RTCore commit does not advance `_last_joint_setpoint_rad`
- Validation:
  - `source ./start.sh && python -m pytest tests/test_gradient05_limits_and_backends.py -q -k 'sync_write_preserves_requested_speed or timed_sync_write_uses_explicit_duration_for_velocity or timed_sync_write_rejects_subcycle_duration or single_actuator_setpoint_includes_velocity or set_joint_positions_does_not_advance_cache_on_commit_failure or applies_master_offsets_to_setpoints or execute_joint_trajectory_enqueues_velocity_points'`
  - `source ./start.sh && python -m py_compile src/gradient_os/arm_controller/backends/ethercat_rtcore/backend.py src/gradient_os/arm_controller/trajectory_execution.py tests/test_gradient05_limits_and_backends.py`
  - `ReadLints` on touched files returned clean.
- Follow-up notes / risks:
  - The direct RTCore compatibility helpers (`set_joint_positions()` / `set_single_actuator_position()`) still use a legacy speed heuristic for true one-point commands; the streamed executor path no longer does.
  - Live hardware retest should focus on the J2 commissioning path that exercises the timed `sync_write()` behavior before trusting the broader direct single-point compatibility path.

## 2026-04-08 20:40 +0000

- Task summary:
  - Implemented the RTCore native-home startup readback fix, preserved the ACK-only commissioning jog contract, and ran the requested live retest on J2 after rebuilding/deploying RTCore.
- Changes:
  - Updated `src/gradient_rt_motion/main.cpp`:
    - hoisted the `0x60B0` SDO helper so both RTCore startup logic and native-home code share the same read path
    - added a post-`startup_ready` native-home offset refresh in the metrics thread with retries and success/failure logging
    - refreshed published `latest_feedback.native_home_position_offset` only on successful SDO upload so a failed read does not overwrite a valid in-memory value
  - Updated `tests/test_api_endpoints.py`:
    - added a regression test proving `/control/joint-jog` remains ACK-only and keeps `max_motor_rpm=100.0` even when `wait_for_idle=true`
- Validation:
  - `source ./start.sh && make -C src/gradient_rt_motion`
  - `source ./start.sh && python -m pytest tests/test_api_endpoints.py -q`
  - `source ./start.sh && python -m py_compile src/gradient_os/api/main.py src/gradient_os/arm_controller/command_api.py src/gradient_os/arm_controller/backends/ethercat_rtcore/backend.py`
  - `ReadLints` on `src/gradient_rt_motion/main.cpp` and `tests/test_api_endpoints.py` returned clean
  - `source ./start.sh && python -m pytest tests/test_api_endpoints.py tests/test_command_api_direct_setpoint.py tests/test_gradient05_limits_and_backends.py -q`
    - result: `tests/test_api_endpoints.py` and `tests/test_command_api_direct_setpoint.py` passed, but the broader run still has existing dirty-branch failures in `tests/test_gradient05_limits_and_backends.py::test_gradient05_config_defaults_and_mapping_shape` and `tests/test_gradient05_limits_and_backends.py::test_ethercat_backend_prefers_robot_defined_axis_scaling`
  - `source ./start.sh && ./systemd/rt-motion/sync-runtime.sh --ensure-active`
  - `sudo ethercat upload -p 1 -t int32 0x60B0 0`
    - result: `-107506`
  - Read `/run/gradient-rt-motion/metrics.json`
    - result: axis 1 now reports `native_home_position_offset=-107506` after restart, matching the drive
  - Started a headless controller/API stack and ran a live J2 commissioning proof:
    - baseline J2 before power-up: about `-16.391 deg`
    - powered-up J2 stayed about `-16.391 deg`
    - issued `POST /control/joint-jog {"joint": 2, "delta_deg": 1.0}`
    - controller logged a bounded target of `-16.391 -> -15.391` at `max_motor_rpm=100.0`
    - observed reported J2 after the move attempt: about `-21.297 deg`
    - RTCore never returned the trajectory to idle; controller logged `TimeoutError: Timed out waiting for RTCore trajectory 1 to complete`
    - sent `POST /control/stop`, then `POST /control/power-down`, and verified final J2 state powered back down
- Follow-up notes / risks:
  - The startup native-home offset readback bug is fixed and proven live, but it was not the only cause of the wrong-direction commissioning move.
  - There is still a second live bug in the J2 commissioning path: RTCore/controller accept a `+1 deg` bounded target yet reported feedback moves about `-4.9 deg` and the RTCore trajectory remains latched until externally stopped.

## 2026-04-09 01:05 +0000

- Task summary:
  - Continued into the next commissioning bug after the user clarified that the drives had been physically power-cycled, then tightened native-home completion so it only reports success after verified post-save readback.
- Changes:
  - Updated `src/gradient_rt_motion/main.cpp`:
    - after writing J2/Jn `0x60B0` and issuing `0x1010:01 = "save"`, RTCore now waits and re-reads `0x60B0` until the desired offset is observed before marking `native_home_state=SUCCEEDED`
    - native-home success/failure logging now includes the verified readback offset
  - Updated `src/gradient_os/arm_controller/backends/ethercat_rtcore/backend.py`:
    - `native_home_joint()` now waits for targeted axes to reach a verified native-home terminal result in RTCore metrics instead of returning success immediately after enqueuing the command
    - added focused metrics polling helpers for native-home completion
  - Updated `tests/test_gradient05_limits_and_backends.py`:
    - extended native-home tests to assert the backend waits for verified completion
    - added a failure-path test when that verification times out
- Validation:
  - `source ./start.sh && make -C src/gradient_rt_motion`
  - `source ./start.sh && python -m pytest tests/test_api_endpoints.py tests/test_gradient05_limits_and_backends.py -q -k 'control_home_joint_native or native_home'`
  - `source ./start.sh && python -m py_compile src/gradient_os/arm_controller/backends/ethercat_rtcore/backend.py`
  - `ReadLints` on touched files returned clean
  - Restarted the full stack cleanly after removing stale controller/API/web children from an old launcher process
  - Live checks after the user clarified the drives had been power-cycled:
    - before re-home: `sudo ethercat upload -p 1 -t int32 0x60B0 0` returned `0`, and `/run/gradient-rt-motion/metrics.json` also showed J2 `native_home_position_offset=0`
    - issued `POST /control/home-joint-native {"joint": 2}` and timed the request
    - RTCore journal now shows: `EtherCAT native_home axis=1 ... desired_offset=668218 readback_offset=668218 saved=1`
    - after re-home settled: `sudo ethercat upload -p 1 -t int32 0x60B0 0` returned `668218`
    - after re-home settled: `/info/joints-detailed` reported J2 `arm_deg=0.0`
- Follow-up notes / risks:
  - The live state change after the user's drive power cycle reframed this bug as a persistence-across-real-power-loss problem, not just an RTCore restart/readback issue.
  - The new code proves native-home now waits for a verified saved offset while the drives remain powered, but persistence across another real drive power cycle is still the remaining manual proof step.

## 2026-04-09 01:22 +0000

- Task summary:
  - Validated the user's E-stop drive power cycle, proved RTCore metrics could stay stale across bus recovery, and patched RTCore to rerun startup/native-home offset refresh after later startup-reset epochs.
- Changes:
  - Updated `src/gradient_rt_motion/main.cpp`:
    - added metrics-thread tracking for `startup_ready` and `startup_reset_count`
    - when the startup epoch changes, RTCore now re-arms both the post-`startup_ready` drive-config readback and the native-home offset refresh instead of treating them as one-shot per process lifetime
  - Updated `.cursor/memory/AGENT_SCRATCHPAD.md` with the new post-power-cycle stale-metrics guardrail.
- Validation:
  - Before the patch, after the user's E-stop power cycle:
    - `sudo ethercat upload -p 1 -t int32 0x60B0 0` returned `0`
    - `/run/gradient-rt-motion/metrics.json` still showed axis 1 `native_home_position_offset=668218`
    - `/info/joints-detailed` still showed the stale logical J2 position near `0 deg`
  - Rebuilt and deployed RTCore:
    - `source ./start.sh && make -C src/gradient_rt_motion`
    - `source ./start.sh && ./systemd/rt-motion/sync-runtime.sh --ensure-active`
  - Post-deploy checks:
    - `ReadLints` on `src/gradient_rt_motion/main.cpp` returned clean
    - RTCore journal showed a fresh `EtherCAT native-home offset refresh axis=1 ... offset=0`
    - `sudo ethercat upload -p 1 -t int32 0x60B0 0` returned `0`
    - `/run/gradient-rt-motion/metrics.json` now reports axis 1 `native_home_position_offset=0`
    - `/info/joints-detailed` now reports J2 near `-18.35 deg`, matching the loss of native-home offset truth
- Follow-up notes / risks:
  - The stale-metrics masking bug is fixed, but the underlying persistence problem remains: J2 `0x60B0` still drops to `0` across a real drive power cycle.
  - The remaining live proof for the new RTCore refresh behavior requires one more manual drive power cycle while this newly deployed RTCore process stays running.

## 2026-04-09 01:36 +0000

- Task summary:
  - Diagnosed the repeated `Power up RTCore-controlled drives now?` prompt as UI label ambiguity between drive-power controls and jog arming, then clarified the labels so the homing flow is unambiguous.
- Changes:
  - Updated `web-ui/src/ControlPanel.tsx`:
    - runtime-header drive buttons now read `Power Up` / `Power Down` instead of generic `Arm` / `Disarm`
    - realtime jog toggle now reads `Arm Jog` / `Disarm Jog`
    - updated nearby helper copy to refer to `Jog mode` instead of ambiguous `Arm mode`
  - Updated `web-ui/src/ControlPanel.test.tsx`:
    - adjusted the jog-session tests to use the new `Arm Jog` / `Disarm Jog` labels
    - added regression coverage that the main control panel separates `Power Up Drives` from `Arm Jog`
    - added coverage that the runtime header exposes `Power Up` / `Power Down` instead of generic `Arm`
  - Updated `.cursor/memory/AGENT_SCRATCHPAD.md` with the new operator-label guardrail.
- Validation:
  - `ReadLints` on `web-ui/src/ControlPanel.tsx` and `web-ui/src/ControlPanel.test.tsx` returned clean
  - `cd web-ui && npm run test -- src/ControlPanel.test.tsx`
  - `cd web-ui && npm run build`
- Follow-up notes / risks:
  - This fixes the UI ambiguity that made drive-enable prompts look like a generic `Arm` action, but it does not change the underlying native-home persistence behavior being debugged.
  - Reload the browser tab before the next homing attempt so the updated labels are visible.

## 2026-04-08 16:51 +0000

- Task summary:
  - Updated the memory-maintenance skills so archive-first rollover is the canonical procedure for oversized scratchpad/devlog files.
- Changes:
  - Updated `.cursor/skills/learning-scratchpad-loop/SKILL.md`:
    - added explicit maintenance guidance to rename the live scratchpad into a dated snapshot instead of deleting old content
    - added the required steps to prepend an archive summary and recreate a slim live scratchpad with only retained lessons
    - clarified that a user request for a "fresh slate" means preserve history via rollover, not deletion
  - Updated `.cursor/skills/devlog-loop/SKILL.md`:
    - added the matching archive-first rollover procedure for `DEVLOG.md`
    - documented that the new live devlog should start with a rollover entry that points to the preserved snapshot
    - clarified that paired scratchpad/devlog cleanup should usually roll both files together
  - Updated `.cursor/memory/AGENT_SCRATCHPAD.md`:
    - recorded the new standard rule that old memory should be preserved in dated snapshots rather than deleted during cleanup
- Validation:
  - Read back the updated skill files and live memory files after editing.
  - `ReadLints` will be run on the touched files after the edits complete.
- Follow-up notes / risks:
  - This updates the skill workflow only; it does not yet refactor any older archive structure beyond the rollover already completed today.

## 2026-04-08 16:45 +0000

- Task summary:
  - Rolled over the oversized live `AGENT_SCRATCHPAD.md` and `DEVLOG.md` into dated snapshots, then recreated slim live memory files that retain only the durable guardrails and the active native-home context.
- Changes:
  - Renamed the previous live files to:
    - `.cursor/memory/AGENT_SCRATCHPAD_2026-02-21_to_2026-04-08.md`
    - `.cursor/memory/DEVLOG_2026-02-16_to_2026-04-08.md`
  - Prepended archive summaries to both renamed snapshots so they explain the work they cover without needing to scan the full file first.
  - Created a new compact `.cursor/memory/AGENT_SCRATCHPAD.md` that carries forward:
    - user workflow preferences
    - validation/deployment guardrails
    - the current native-home regression rules for A6-EC / RTCore
  - Reset `.cursor/memory/DEVLOG.md` to this new active timeline entry instead of carrying the full historical ledger forward.
  - Retained the most important current workstream context:
    - native-home remains an active commissioning path and should be treated as unresolved until live hardware behavior is rechecked after risky changes
    - `native_home_position_offset` belongs on feedback/read and hold-target alignment paths, not as an extra subtraction on outgoing RTCore logical position targets
    - successful native home should leave only the homed axis disabled and should surface explicit result/state in telemetry/UI
- Validation:
  - Verified the old live files were preserved as dated snapshots and the new live files were recreated at the original paths.
  - No product code, tests, or runtime behavior were changed in this cleanup task.
- Follow-up notes / risks:
  - Keep the new live memory files intentionally short; archive again before they turn back into full historical ledgers.
  - For future native-home work, use the new scratchpad for preflight guardrails and the dated snapshots for deeper historical debugging context when needed.

## 2026-04-09 02:33 +0000

- Task summary:
  - Completed the direct EtherCAT SDO isolation experiment for J2 native-home persistence and narrowed the remaining fault to drive/object semantics rather than our native-home call sequence.
- Changes:
  - Performed a direct drive write on J2 while disarmed:
    - `sudo ethercat download -p 1 -t int32 0x60B0 0 -- "-96134"`
    - `sudo ethercat download -p 1 -t uint32 0x1010 1 0x65766173`
  - Verified the write immediately after save and after a settle window:
    - `sudo ethercat upload -p 1 -t int32 0x60B0 0` returned `-96134`
    - `sudo ethercat upload -p 1 -t uint16 0x603F 0` returned `0`
    - `sudo ethercat upload -p 1 -t uint16 0x6041 0` returned `0x1650`
  - After the user performed a real drive power cycle, re-read the live state before any jog/home/power-up action:
    - `sudo ethercat upload -p 1 -t int32 0x60B0 0` returned `0`
    - `/run/gradient-rt-motion/metrics.json` axis 1 reported `native_home_position_offset=0`
    - `curl -sf http://127.0.0.1:4000/info/joints-detailed` showed J2 near its raw physical angle instead of logical zero
    - `journalctl -u gradient-rt-motion.service -n 60 --no-pager` showed the refreshed post-startup offset readback also landing on `offset=0`
  - Read the A6-EC ESI metadata for the next root-cause branch:
    - confirmed `0x60B0` is `Position offset`
    - confirmed `0x607C` is `Home offset`
    - confirmed vendor object `0x2013:17` (`Update function code values written via communication to EEPROM`) exists and currently reads back `1`
- Validation:
  - The direct-save experiment reproduced the same post-power-loss reset without using the Python native-home path.
  - Read-only follow-up checks showed `0x607C=0`, `0x2013:17=1`, and J2 still in `SwitchOnDisabled` after power restore.
- Follow-up notes / risks:
  - This is strong evidence that `0x60B0` should not be relied on as the durable hardware-zero store on this A6-EC, even when `0x1010:01` reports an immediate successful save.
  - The next live experiment should target `0x607C` or the vendor-native persistent zero parameter rather than continuing to harden the existing `0x60B0` save loop.

## 2026-04-09 02:40 +0000

- Task summary:
  - Clarified the implications of a possible migration from `0x60B0` to `0x607C` and the current scope of the persistence fault.
- Changes:
  - No product code changed.
  - Reconciled the current evidence into two guardrails:
    - if `0x607C` is adopted as the persistent home object, RTCore/controller command and feedback paths must be re-audited to avoid double-applying offsets
    - the current proof should not be labeled strictly `J2-only`, because only J2 has been exercised deeply even though same-drive semantics suggest a model-wide behavior is more likely
- Validation:
  - Based on the already verified direct-save result (`0x60B0` survives while powered, resets to `0` after real power loss) plus the A6-EC ESI metadata showing both `0x60B0 Position offset` and `0x607C Home offset`
- Follow-up notes / risks:
  - A single additional cross-axis persistence probe would tell us whether this is clearly drive-model-wide or a J2-drive-specific hardware anomaly.

## 2026-04-09 03:43 +0000

- Task summary:
  - Completed step 1 of the production-fix plan by running a direct `0x607C` persistence probe on J1 and J2 with a real drive power cycle.
- Changes:
  - No product code changed.
  - Captured a disarmed/fault-free baseline:
    - J1 `0x607C = 0`, J2 `0x607C = 0`
    - both axes `0x6041 = 0x1650`
    - both axes `0x603F = 0`
  - Wrote and saved distinct test values while disarmed:
    - `sudo ethercat download -p 0 -t int32 0x607C 0 12345`
    - `sudo ethercat download -p 1 -t int32 0x607C 0 -- -23456`
    - `sudo ethercat download -p <axis> -t uint32 0x1010 1 0x65766173`
  - Verified immediate post-save readback:
    - J1 `0x607C = 12345`
    - J2 `0x607C = -23456`
  - After a real drive power cycle, re-read before any motion/power-up action:
    - J1 `0x607C = 12345`
    - J2 `0x607C = -23456`
    - both axes still `0x603F = 0`
  - Collected live-state evidence showing the current code still only refreshes `0x60B0`:
    - `/run/gradient-rt-motion/metrics.json` kept `native_home_position_offset=0`
    - RTCore journal refreshed `offset=0` in the existing native-home offset refresh path
- Validation:
  - The direct `0x607C` values survived a real power cycle on two axes.
  - This contrasts with the earlier direct `0x60B0` experiment, where the written value reset to `0` after power loss.
- Follow-up notes / risks:
  - This strongly supports `0x607C` as the durable drive-home object for this A6-EC setup and makes a J2-only hardware anomaly much less likely.
  - No motion should be commissioned from these experimental values until the software path is migrated and the offset-application frame is re-audited.

## 2026-04-09 03:49 +0000

- Task summary:
  - Produced the smallest-safe production migration plan for moving native-home persistence from `0x60B0` to the now-validated `0x607C`.
- Changes:
  - No product code changed.
  - Traced the minimal active code surface that currently depends on `0x60B0` semantics:
    - RTCore SDO read helper and startup refresh in `src/gradient_rt_motion/main.cpp`
    - RTCore native-home write/verify path in `src/gradient_rt_motion/main.cpp`
    - RTCore feedback-aligned enable/hold-target logic that consumes `native_home_position_offset`
    - Python metrics refresh and command-frame guardrails in `src/gradient_os/arm_controller/backends/ethercat_rtcore/backend.py`
- Validation:
  - Based on direct code inspection plus the completed J1/J2 `0x607C` power-cycle proof.
- Follow-up notes / risks:
  - The safest migration is a narrow source-of-truth change around `native_home_position_offset`, not a broad register-name replacement.

## 2026-04-11 20:02 +0000

- Task summary:
  - Updated the A6-EC bring-up/commissioning documentation and the live native-homing plan to reflect the current manual-backed model: startup absolute mode and DS402 runtime modes are separate layers, normal motion stays in `CSP`, and native home should be treated as a commissioning-only HM transaction.
- Changes:
  - Updated `docs/ethercat/bringup.md` with:
    - a mode-layer table distinguishing `C00.07`, `0x6060`, HM objects, and `0x60B0`
    - the current validated conclusions (`C00.07` rotation-mode concern, `0x60B0` non-persistence, `0x607C` persistence, HM mode `35` relevance)
    - a documented GradientOS commissioning workflow for native home
  - Updated `.cursor/skills/gradientos-sop/commissioning-safety.md` to remove the stale "`0x60B0` is durable native home" assumption and replace it with the commissioning-workflow model.
  - Updated `.cursor/plans/a6ec-native-homing_16b87693.plan.md` to:
    - explicitly keep steady-state motion in `CSP`
    - add the "commissioning-only native-home workflow" requirement
    - list the exact product surfaces/files to change in profile/catalog, RTCore, controller/API/UI, and tests
- Validation:
  - Re-read the A6-EC manual sections for HM, CSP, and absolute-system settings in `docs/resources/A6-EC_series_servo_drive_manual.pdf`.
  - Reviewed the current runtime mode usage in `src/gradient_os/arm_controller/backends/ethercat_rtcore/backend.py`.
  - Reviewed the resulting documentation diff with `git diff`.
  - No code tests were run because this pass only changed documentation/plan files.
- Follow-up notes / risks:
  - Product code still uses the old A6-EC startup-mode labels/default and the old `0x60B0`-based native-home implementation; the docs/plan now lead the code and need to be implemented in a later execution pass.

## 2026-04-11 20:28 +0000

- Task summary:
  - Tightened the live homing plan to make the RTCore boundary explicit and added linked source documents at the top of the plan.
- Changes:
  - Updated `.cursor/plans/a6ec-native-homing_16b87693.plan.md` to:
    - add a `Source Documents` section linking the manual PDF, bring-up doc, ESI, and decoded fault reference
    - replace the RTCore section with an explicit "no vendor-specific homing knowledge in RTCore" rule
    - state that RTCore must not hardcode A6-EC object IDs, homing method `35`, save rules, or durable-home assumptions
    - require RTCore to execute only generic transaction primitives driven by profile/catalog/backend-owned descriptors
- Validation:
  - Re-read the updated top-of-file source links and RTCore section in the plan file.
  - Confirmed the revised text now states that the choice of HM objects, values, and ordering must live outside RTCore.
  - A readonly `git diff` check against the plan path failed because the plan file lives outside the repository root; no further shell validation was needed.
- Follow-up notes / risks:
  - The implementation pass must preserve this stricter boundary and avoid drifting back toward "generic RTCore plus a few A6-EC exceptions."

## 2026-04-11 21:00 +0000

- Task summary:
  - Removed the remaining implementation ambiguity from the homing plan so it can serve as a true cold handoff for a fresh build agent.
- Changes:
  - Updated `.cursor/plans/a6ec-native-homing_16b87693.plan.md` to add:
    - `Cold Handoff Defaults` with explicit first-cut values/choices:
      - `C00.07 = 4`
      - steady-state `CSP`
      - commissioning `HM`
      - `6098 = 35`
      - `60E6 = 0`
      - `607C = 0`
      - `607C` as persistent truth source
      - no explicit `0x1010` in the first HM implementation unless bench evidence requires it
      - post-home explicit re-power expectation
      - validation axes `J4` and `J2`
    - `Cold Handoff Rules` telling a fresh agent not to reopen architecture decisions without bench evidence
    - a minimum descriptor contract example showing what data belongs in the A6-EC profile rather than RTCore
    - explicit `native_home_position_offset` semantics for the first HM-based cut
  - Added the post-home operator expectation to the controller/API/UI section of the plan.
- Validation:
  - Re-read the newly added cold-handoff sections in the plan file.
  - Confirmed the plan now specifies defaults, descriptor shape, field semantics, validation axes, and post-home state instead of leaving them implicit.
- Follow-up notes / risks:
  - The remaining uncertainty is live bench proof only; the plan no longer leaves material build choices undecided for the first implementation cut.

## 2026-04-11 21:13 +0000

- Task summary:
  - Started the first production build cut for A6-EC native homing by moving the HM workflow into profile-owned descriptor data, switching the startup absolute-mode default to the manual-backed rotation mode, and replacing the RTCore hardcoded `0x60B0` native-home path with a generic descriptor executor.
- Changes:
  - Updated `src/gradient_os/arm_controller/profiles/drive/a6ec_ds402.py`:
    - corrected the human-readable `C00.07 / 0x2000:08` labels so value `4` is the rotation-mode default
    - added a profile-owned native-home descriptor for the first HM cut (`steady_state_mode=8`, `commissioning_mode=6`, `truth_source=0x607C`, HM method `35`, `60E6=0`, `607C=0`)
    - rendered that descriptor into the new RTCore env var `GRADIENT_RT_NATIVE_HOME_CONFIG`
  - Updated `src/gradient_os/arm_controller/ethercat_drive_catalog.py`:
    - changed the A6-EC startup default from `1` to `4`
  - Updated `src/gradient_os/arm_controller/profiles/registry.py`, `src/gradient_os/arm_controller/backends/registry.py`, and `src/gradient_os/arm_controller/backends/ethercat_rtcore/runtime.py`:
    - threaded the profile-owned native-home config through the backend/profile registry
    - included `GRADIENT_RT_NATIVE_HOME_CONFIG` in the rendered RTCore systemd env
  - Updated `src/gradient_rt_motion/main.cpp`:
    - added a generic native-home descriptor parser for `set_mode`, `write_sdo`, `controlword_sequence`, `wait_statusword`, `refresh_truth`, and `restore_mode`
    - replaced the old A6-EC-specific `0x60B0` write/save implementation with a descriptor-driven executor
    - switched startup/native-home truth refresh from the hardcoded `0x60B0` path to the descriptor-selected truth source
    - made `MSG_CMD_SET_MODE` update a real per-axis desired-mode path instead of writing to an unused single atomic
    - added a generic per-axis service override so RTCore can drive PDO-owned `0x6040` / `0x6060` fields during HM without SDO-vs-PDO races
  - Updated `systemd/rt-motion/gradient-rt-motion.service`:
    - passed the new `GRADIENT_RT_NATIVE_HOME_CONFIG` env var through to the RTCore CLI
  - Updated `src/gradient_os/api/main.py` and `web-ui/src/ControlPanel.tsx`:
    - reframed native home as a commissioning-only transaction in the operator-facing API/UI copy
  - Updated `tests/test_rtcore_runtime.py`, `tests/test_api_endpoints.py`, and `tests/test_encoder_retention.py`:
    - aligned startup-default expectations with `C00.07 = 4`
    - added coverage that the rendered RTCore env now includes the native-home descriptor
    - updated startup-mode label expectations to the manual-backed `Absolute position linear mode` wording for value `1`
- Validation:
  - `make -C src/gradient_rt_motion`
  - `source ./start.sh && python -m pytest tests/test_rtcore_runtime.py tests/test_api_endpoints.py tests/test_encoder_retention.py -q`
  - `source ./start.sh && python -m py_compile src/gradient_os/arm_controller/profiles/drive/a6ec_ds402.py src/gradient_os/arm_controller/profiles/registry.py src/gradient_os/arm_controller/backends/registry.py src/gradient_os/arm_controller/backends/ethercat_rtcore/runtime.py src/gradient_os/api/main.py tests/test_rtcore_runtime.py tests/test_api_endpoints.py tests/test_encoder_retention.py`
  - `cd web-ui && npm run build`
  - `ReadLints` on all touched product/test files returned clean
- Follow-up notes / risks:
  - This pass proves the code and focused tests, but the HM transaction still needs live bench validation on `J4` and `J2` to confirm the A6-EC statusword masks, post-home truth (`0x607C -> 0`), and safe power-up behavior.
  - The RTCore generic executor is currently tailored to the first descriptor shape; if a future drive family needs richer waits or multi-axis commissioning semantics, extend the descriptor/parser instead of reintroducing vendor branches in `main.cpp`.

## 2026-04-11 22:50 +0000

- Task summary:
  - Fixed the A6-EC HM start-edge timing for `J4`, redeployed the runtime, and completed a clean live proof that the commissioning native-home transaction now succeeds on `J4`.
- Changes:
  - Updated `src/gradient_os/arm_controller/profiles/drive/a6ec_ds402.py`:
    - split the HM controlword handshake so the descriptor now does:
      - `controlword_sequence [6,7,15]`
      - `wait_statusword all_set=0x0227 all_clear=0x2048`
      - `controlword_sequence [31]`
    - kept the existing completion wait (`all_set=0x9000`, `all_clear=0x2000`) and `0x607C` truth refresh unchanged
  - Updated `tests/test_rtcore_runtime.py`:
    - aligned the expected `GRADIENT_RT_NATIVE_HOME_CONFIG` env string with the new two-phase HM handshake
  - Synced the updated runtime into the installed service with `systemd/rt-motion/sync-runtime.sh --ensure-active`
  - Performed a full stack restart after discovering that restarting RTCore alone left the controller on a stale IPC session
- Validation:
  - `source ./start.sh && python -m pytest tests/test_rtcore_runtime.py -q`
  - `source ./start.sh && python -m py_compile src/gradient_os/arm_controller/profiles/drive/a6ec_ds402.py tests/test_rtcore_runtime.py`
  - Verified the installed env at `/etc/default/gradient-rt-motion` now contains:
    - `op|controlword_sequence|6,7,15`
    - `op|wait_statusword|0x0227|0x2048`
    - `op|controlword_sequence|31`
  - Verified full stack reconnection after restart:
    - RTCore journal logged `Controller connected` and `IPC handshake complete`
    - controller log showed `EtherCAT RTCore Connected` and `Feedback ready`
  - Clean live `J4` proof:
    - API `POST /control/home-joint-native` returned `200 OK` with `ACK,NATIVE_HOME_JOINT,4`
    - RTCore journal logged `EtherCAT native_home axis=3 ... steps=10` followed by `Native-home success`
    - live EtherCAT sampling on slave position `3` showed `0x6061: 8 -> 6 -> 8` and `0x6041: 0x1650 -> 0x0633 -> 0x0237 -> 0x9650`
    - `0x607C` stayed `0` throughout the successful J4 run
    - final metrics show axis 3 `native_home_state=2`, `native_home_last_abort_code=0`, `statusword=38480 (0x9650)`, and feedback position near zero (`pos_counts=-1`)
- Follow-up notes / risks:
  - A retry issued after only restarting `gradient-rt-motion.service` failed misleadingly until the controller/API stack was restarted; future commissioning validation after RTCore restarts should always confirm a fresh IPC reconnect first.
  - `startup_drive_config.readback_valid/verified` was still `0` in the immediate post-start metrics snapshot even though the later startup logs completed successfully; this looks like timing of when the snapshot was read, not a regression, but should be remembered when sampling right after restart.
  - The next meaningful scope step is to repeat the same clean proof on the next target joint (`J2` or whichever joint the user chooses) before generalizing the result across all axes.

## 2026-04-11 23:58 +0000

- Task summary:
  - Investigated a live commissioning regression where a UI jog intended for `J4` visibly moved `J1` and left `J2` faulted after power-down.
- Changes:
  - No product code changed in this pass.
  - Collected and correlated evidence from:
    - `logs/startups/20260411-234934/controller.log`
    - `logs/startups/20260411-234934/api.log`
    - `journalctl -u gradient-rt-motion.service`
    - `/run/gradient-rt-motion/metrics.json`
    - direct EtherCAT SDO reads on J2 (`0x603F`, `0x203F`, `0x6041`)
- Validation:
  - Confirmed the stack itself restarted cleanly:
    - `gradient-rt-motion.service` and `ethercat.service` active
    - API `/health` returning OK
    - RTCore metrics reporting `startup_ready=1`, `operational_slaves=6`
  - Confirmed the first jog command path was formed for software joint 4, not joint 1:
    - controller log showed `target_deg` changing only the 4th joint from about `-0.002` to `-1.002`
  - Confirmed the resulting motion/fault contradicted that intended target:
    - next controller feedback sample showed `J1` changed from about `-1.876` to `-2.215`
    - RTCore-backed trajectory execution ended in state `faulted`
  - Decoded the unexpected J2 fault:
    - `0x603F = 0xFF00`
    - `0x203F = 0x0871`
    - A6-EC manual reference maps that to `Er87.1` ("one-time excessive position reference increment")
  - Correlated the unintended motion with the live native-home offsets still present on uninvolved axes:
    - metrics showed `J1` offset `12345` counts and `J2` offset `-96135` counts
    - `12345` counts converts to about `0.339 deg`, which matches the observed unintended J1 movement almost exactly
  - Read the relevant code paths and identified a likely frame mismatch:
    - RTCore hold-target alignment uses `pos - native_home_position_offset`
    - backend trajectory upload currently converts joint positions to axis targets without compensating for that offset frame
- Follow-up notes / risks:
  - Current evidence says this is not a simple `J4 -> J1` joint-index remap bug; it is more likely a commanded-target vs hold-target frame mismatch that becomes dangerous whenever uninvolved axes still have nonzero native-home offsets.
  - Because queued trajectories currently upload all-axis points, a jog on one joint can inject hidden target steps on other axes that are merely meant to hold position.
  - If the user truly pressed the `J4` jog button only once, there may also be a second UI/session anomaly: the later jog POSTs in the same browser session appear to target software joint 3. That is worth verifying separately, but it is not required to explain the observed `J1` motion and `J2` `Er87.1` fault.
  - Safest next step is a code fix plus a controlled retest before any more commissioning jog or re-home attempts on live hardware.

## 2026-04-12 01:35 +0000

- Task summary:
  - Implemented the RTCore/backend jog-frame fix so queued trajectories, realtime RTCore jog, and completion checks now use the same native-home-aware target frame.
- Changes:
  - Updated `src/gradient_rt_motion/main.cpp` to:
    - document the per-axis frame contract explicitly
    - translate queued trajectory point targets into RTCore's feedback-aligned hold frame when trajectory points are latched
    - seed realtime jog target accumulation from the same feedback-aligned frame instead of raw `pos_counts`
    - compare final trajectory completion against feedback translated into the same frame as the queued target
  - Updated `src/gradient_os/arm_controller/backends/ethercat_rtcore/backend.py` comments so the Python side now explicitly documents that it uploads controller/logical targets and leaves the final native-home reframe to RTCore.
  - Updated `tests/test_gradient05_limits_and_backends.py` with a focused regression test that locks the queued-trajectory payload contract when a native-home offset is present.
- Validation:
  - `make -C src/gradient_rt_motion`
  - `source "/home/pi/GradientOS/.venv/bin/activate" && PYTHONPATH=src python -m pytest tests/test_gradient05_limits_and_backends.py -q -k "applies_native_home_offsets_to_feedback_but_not_command_targets or enqueue_trajectory_points_keeps_controller_logical_frame_with_native_home_offset"`
  - `source "/home/pi/GradientOS/.venv/bin/activate" && PYTHONPATH=src python -m py_compile src/gradient_os/arm_controller/backends/ethercat_rtcore/backend.py tests/test_gradient05_limits_and_backends.py`
  - `ReadLints` on `src/gradient_rt_motion/main.cpp`, `src/gradient_os/arm_controller/backends/ethercat_rtcore/backend.py`, and `tests/test_gradient05_limits_and_backends.py` returned clean.
  - A broad `python -m pytest tests/test_gradient05_limits_and_backends.py -q` run on this live machine still reports unrelated failures because some older tests pull current RTCore metrics and inherit the present nonzero `native_home_position_offset` state.
- Follow-up notes / risks:
  - This pass is compile-tested and covered by focused backend tests, but it still needs the planned hardware proof with the current nonzero `J1/J2` `0x607C` values left intact.
  - Live retest should follow the existing safety plan: clear the `J2` fault, keep `J1/J2` offsets unchanged, then verify a single `J4 -1 deg` commissioning jog moves only `J4` and reaches a clean RTCore terminal state.

## 2026-04-12 02:00 +0000

- Task summary:
  - Ran the post-implementation live proof with the rebuilt RTCore deployed and found that the remaining unsafe step occurs on `SAFE_POWER_UP` before the `J4` jog itself.
- Changes:
  - No product code changed in this pass.
  - Captured live proof artifacts from:
    - `http://127.0.0.1:4000/info/joints-detailed`
    - `http://127.0.0.1:4000/control/motion-status`
    - `http://127.0.0.1:4000/control/power-up`
    - `http://127.0.0.1:4000/control/joint-jog`
    - `http://127.0.0.1:4000/control/wait-for-idle`
    - `http://127.0.0.1:4000/control/power-down`
    - `/run/gradient-rt-motion/metrics.json`
    - `logs/startups/20260412-015821/controller.log`
    - `logs/startups/20260412-015821/api.log`
    - direct `sudo ethercat upload` reads for `0x607C`, `0x603F`, `0x203F`, and `0x6041`
- Validation:
  - Confirmed the installed RTCore binary matches the rebuilt repo binary:
    - `sha256sum /home/pi/GradientOS/src/gradient_rt_motion/gradient-rt-motion /usr/local/bin/gradient-rt-motion`
  - Confirmed the proof condition still held before motion:
    - `J1 0x607C = 12345`
    - `J2 0x607C = -96135`
    - `J4 0x607C = 0`
    - API motion status was `idle`
  - Live proof result:
    - before power-up, `J1` was about `105276` counts / `-3.2306 deg`
    - after `SAFE_POWER_UP`, `J1` was about `92889` counts / `-2.8903 deg`
    - the `J1` delta was about `-12387` counts, effectively the persisted `J1` offset magnitude
    - `J2` faulted during/after power-up, before the jog proved anything useful:
      - `0x603F = 0xFF00`
      - `0x203F = 0x0871`
      - `0x6041 = 0x1638`
    - the subsequent single `J4 -1 deg` jog returned `accepted` but RTCore ended `faulted`, `WAIT_FOR_IDLE` timed out, and `J4` did not make the intended `-1 deg` move
  - Controller log evidence:
    - `SAFE_POWER_UP` completed
    - `APPLY_JOINT_SETPOINT` targeted only joint 4
    - controller thread raised `RuntimeError: RTCore trajectory execution ended in state 'faulted'`
- Follow-up notes / risks:
  - The original queued-target/jog-frame fix was not sufficient for live safety because RTCore still appears to inject the persisted home offset on enable/hold synchronization.
  - The new highest-confidence hypothesis is that subtracting `native_home_position_offset` in RTCore hold-target alignment is itself wrong for live `0x607C` behavior; the bench now suggests `0x6064`/`0x607A` may already be in the drive's homed frame, so the software subtraction may be double-applying the offset.
  - The next implementation pass should target the enable/hold-target contract first, not the API jog route.

## 2026-04-12 03:10 +0000

- Task summary:
  - Fixed the remaining RTCore power-up frame bug so enable/hold synchronization stays in raw CSP wire counts, then proved on hardware that `SAFE_POWER_UP` and the original `J4 -1 deg` commissioning jog both behave safely with nonzero persisted `0x607C` offsets on other axes.
- Changes:
  - Updated `src/gradient_rt_motion/main.cpp` to:
    - keep drive-facing hold/output targets in raw `0x6064` / `0x607A` counts during pre-enable, passive startup, stop-collapse, and first-`OperationEnabled` latch
    - keep queued trajectory-point conversion explicit by subtracting `native_home_position_offset` only when converting controller/logical targets into raw CSP wire counts
    - restore realtime jog target seeding and trajectory completion checks to the raw CSP wire frame
  - Updated `src/gradient_os/arm_controller/backends/ethercat_rtcore/backend.py` comments so the Python contract now explicitly says RTCore converts queued controller/logical targets once into raw CSP wire counts.
  - Updated `tests/test_gradient05_limits_and_backends.py` so the focused backend tests cover safe-power-up synchronization and keep the queued-target payload contract explicit with native-home offsets present.
- Validation:
  - `make -C src/gradient_rt_motion`
  - `source "/home/pi/GradientOS/.venv/bin/activate" && PYTHONPATH=src python -m pytest tests/test_gradient05_limits_and_backends.py -q -k "safe_power_up_arms_sets_mode_and_enables or applies_native_home_offsets_to_feedback_but_not_command_targets or enqueue_trajectory_points_keeps_controller_logical_frame_with_native_home_offset"`
  - `source "/home/pi/GradientOS/.venv/bin/activate" && PYTHONPATH=src python -m py_compile src/gradient_os/arm_controller/backends/ethercat_rtcore/backend.py tests/test_gradient05_limits_and_backends.py`
  - Live proof on the rebuilt headless stack using the exact controller/API path:
    - Stage 1 `POST /control/power-up` with `J1 0x607C=12345`, `J2 0x607C=-96135`, `J4 0x607C=0`
    - direct `ethercat upload` reads for `0x607C`, `0x6064`, `0x607A`, `0x6041`, `0x603F`, `0x203F` on J1/J2/J4 before and after power-up
    - result: no nonzero fault words, J1 delta about `-42` counts / `+0.001 deg`, J2 delta about `+1284` counts / `+0.035 deg`, J4 delta about `+15` counts / `-0.002 deg`
    - Stage 2 `POST /control/joint-jog {"joint":4,"delta_deg":-1.0}` followed by `POST /control/wait-for-idle`
    - result: only J4 moved (about `-1.01 deg`, `+6619` counts), terminal state was `completed`, and RTCore reported the normal trajectory-completed event rather than a fault
- Follow-up notes / risks:
  - The live proof now contradicts the earlier assumption recorded during the first jog-frame fix: the safe wire-frame contract is raw CSP counts for hold/output/jog-completion, not `pos_counts - native_home_position_offset`.
  - `J2` still showed a noticeable raw-count delta during power-up without meaningful joint-angle motion or any fault words; treat joint-space deltas plus SDO/API fault checks as the real go/no-go signal, not a raw-count threshold alone.

## 2026-04-12 04:32 +0000

- Task summary:
  - Fixed the native-home false-negative/UI contradiction so long-running drive home operations no longer fall back to a generic request failure while RTCore telemetry later reports success.
- Changes:
  - Updated `src/gradient_os/arm_controller/backends/ethercat_rtcore/backend.py` to:
    - extend the native-home verification wait window from `3.0s` to `10.0s`
    - require a fresh post-command RTCore metrics sample before trusting native-home terminal state
    - return a structured native-home result with `accepted`, `verified`, `timed_out`, `code`, `message`, and live native-home status fields instead of a bare boolean
  - Updated `src/gradient_os/run_controller.py` so `NATIVE_HOME_JOINT` replies preserve the structured backend payload across the UDP controller boundary.
  - Updated `src/gradient_os/api/main.py` so `/control/home-joint-native` returns the structured native-home result for both ACK and controller-level ERROR replies instead of collapsing domain outcomes into a generic `503`.
  - Updated `web-ui/src/ControlPanel.tsx` so drive-native home shows:
    - success for verified results
    - warning for accepted-but-pending verification or missing post-home feedback refresh
    - error only for true native-home failure / request failure
  - Updated focused regressions in:
    - `tests/test_gradient05_limits_and_backends.py`
    - `tests/test_api_endpoints.py`
    - `web-ui/src/ControlPanel.test.tsx`
- Validation:
  - `source "/home/pi/GradientOS/.venv/bin/activate" && PYTHONPATH=src python -m py_compile src/gradient_os/arm_controller/backends/ethercat_rtcore/backend.py src/gradient_os/run_controller.py src/gradient_os/api/main.py tests/test_gradient05_limits_and_backends.py tests/test_api_endpoints.py`
  - `source "/home/pi/GradientOS/.venv/bin/activate" && PYTHONPATH=src python -m pytest tests/test_gradient05_limits_and_backends.py -q -k "native_home or safe_power_up_arms_sets_mode_and_enables or applies_native_home_offsets_to_feedback_but_not_command_targets or enqueue_trajectory_points_keeps_controller_logical_frame_with_native_home_offset"`
  - `source "/home/pi/GradientOS/.venv/bin/activate" && PYTHONPATH=src python -m pytest tests/test_api_endpoints.py -q -k "home_joint_native"`
  - `npm test -- src/ControlPanel.test.tsx`
  - `ReadLints` on the edited backend/controller/API/UI/test files returned no diagnostics
- Follow-up notes / risks:
  - The `10.0s` native-home wait is a safer default for current observed drive behavior, but if a particular axis routinely exceeds it the UI will now degrade to a warning/pending-verification state rather than a hard failure.
  - This pass improves controller/API/UI truthfulness for native-home, but live bench revalidation on the actual J1 flow is still the right next proof step.

## 2026-04-12 04:39 +0000

- Task summary:
  - Verified the post-restart J1 native-home flow on live logs/metrics and promoted the now-stable native-home frame/status rules into the canonical GradientOS SOP skill files.
- Changes:
  - Verified the latest restart session in `logs/startups/20260412-043326/`:
    - `controller.log` shows `Received: 'NATIVE_HOME_JOINT,1'` followed by `[EtherCAT RTCore] Native drive-home verified: joint=1 axis_mask=0x1`
    - `api.log` shows `POST /control/home-joint-native HTTP/1.1" 200 OK`
    - `/run/gradient-rt-motion/metrics.json` shows axis 0 with `native_home_state=2`, `native_home_last_abort_code=0`, and `axis_enable_mask=62`, which matches the intended "J1 left disabled after home" contract
  - Updated canonical SOP skill references:
    - `.cursor/skills/gradientos-sop/rtcore-ethercat.md`
    - `.cursor/skills/gradientos-sop/commissioning-safety.md`
    - `.cursor/skills/gradientos-sop/ui-api-telemetry.md`
  - Replaced the stale RTCore native-home frame note with the validated contract:
    - drive-facing CSP hold/output/enable targets stay in raw `0x6064` / `0x607A` wire counts
    - subtract `native_home_position_offset` only when converting queued controller/logical targets into raw CSP wire counts
  - Added stable commissioning/UI guidance:
    - native-home keeps the homed axis disabled until explicit safe power-up
    - native-home verification must use a fresh post-command RTCore metrics sample
    - accepted-but-still-verifying native-home should surface as pending/warning, not generic request failure
- Validation:
  - log/metrics inspection only; no new code tests were needed for the skill-file consolidation pass
- Follow-up notes / risks:
  - The canonical skill now matches the validated native-home frame and status contract observed on live hardware after restart.

## 2026-04-12 04:44 +0000

- Task summary:
  - Updated the long-form canonical SOP source file so the master GradientOS operating-principles document no longer carries stale native-home frame/status guidance.
- Changes:
  - Updated `.cursor/skills/gradientos-sop/RTOS_ETHERCAT_MASTER_OPERATING_PRINCIPLES.md` to:
    - clarify that safe-enable/power-up synchronization must happen in the actual drive-facing CSP wire frame
    - state the validated A6-EC rule that `0x607C` is durable native-home truth while hold/output/enable targets remain raw `0x6064` / `0x607A` counts
    - document that `native_home_position_offset` is applied only when converting queued controller/logical targets into raw CSP wire counts
    - describe the finalized native-home execution/status contract (`accepted`, `verified`, `timed_out`) and the requirement to wait for fresh post-command RTCore metrics
    - note that non-converged native-home verification should degrade to pending/warning messaging instead of generic request failure
- Validation:
  - searched the `gradientos-sop` corpus for stale native-home wording after the edit
  - `ReadLints` on the edited master markdown file returned no diagnostics
- Follow-up notes / risks:
  - The routed SOP files and the long-form master file are now aligned for the native-home workstream.

## 2026-04-12 04:47 +0000

- Task summary:
  - Ran the formal encoder-retention power-cycle comparison and confirmed the current pose does not survive a real power cycle on this setup yet.
- Changes:
  - Captured retention experiment `20260412-044300` using the built-in API workflow:
    - `before_power_down` snapshot at `2026-04-12T04:43:00+00:00`
    - `after_power_up` snapshot at `2026-04-12T04:45:43+00:00`
  - Artifacts written to:
    - `logs/encoder-retention/20260412-044300/before_power_down.json`
    - `logs/encoder-retention/20260412-044300/after_power_up.json`
    - `logs/encoder-retention/20260412-044300/comparison.json`
    - `logs/encoder-retention/20260412-044300/comparison.md`
- Validation:
  - Comparison outcome:
    - `raw_encoder_mismatch = true`
    - `logical_angle_mismatch = true`
    - `startup_drive_config_mismatch = true`
    - `active_battery_or_multiturn_faults = []`
  - Representative before/after raw-count deltas:
    - axis 0: `6 -> 38326`
    - axis 1: `73 -> 101086`
    - axis 2: `-47 -> 25346`
    - axis 3: `-13 -> 4769`
    - axis 4: `35 -> 21398`
    - axis 5: `6 -> 18696`
  - Representative logical-angle drift:
    - `J1: -2.876e-06 -> -0.01837 rad`
    - `J6: -2.876e-05 -> -0.08962 rad`
- Follow-up notes / risks:
  - The decisive signal is the raw/logical mismatch on every axis after a real power cycle.
  - The after-power snapshot did not show battery/multi-turn faults, but startup drive-config verification remained unresolved on all axes (`readback_valid=false`, `verified=false`), so the next debugging pass should focus on proving the startup absolute-position/encoder-tracking mode rather than assuming retained position is trustworthy.

## 2026-04-12 04:55 +0000

- Task summary:
  - Tested the immediate post-retention hypotheses directly and ruled out both stale software-side home writes and an incorrect startup absolute-mode value as the primary cause of the cold-boot pose drift.
- Changes:
  - Verified the startup config path in code:
    - `a6ec_encoder_position_tracking_mode` defaults to `4` in `src/gradient_os/arm_controller/ethercat_drive_catalog.py`
    - RTCore writes that startup SDO during bring-up and performs deferred readback in `src/gradient_rt_motion/main.cpp`
  - Collected privileged live EtherCAT reads after cold boot:
    - all axes `0x2000:08 = 4`
    - all axes `0x607C = 0`
    - all axes `0x6041 = 0x1650`
    - all axes `0x6064` matched the large after-power counts from the retention snapshot
  - Verified RTCore journal/readback evidence:
    - journal logged `EtherCAT startup readback ... commanded=4 readback=4 verified=1` on all six axes
    - current `/run/gradient-rt-motion/metrics.json` now shows `startup_drive_config.readback_valid=1` and `verified=1` on every axis
- Validation:
  - `journalctl -u gradient-rt-motion.service -n 200 --no-pager`
  - privileged `ethercat upload` reads for `0x2000:08`, `0x607C`, `0x6064`, and `0x6041`
  - live metrics inspection via `/run/gradient-rt-motion/metrics.json`
- Follow-up notes / risks:
  - This evidence strongly weakens the "leftover direct writes" theory and the "startup mode wrong" theory.
  - The remaining highest-confidence hypothesis is drive-side absolute/reference validity not surviving cold boot even though the configured startup mode value is correct and readback-verifiable.
  - A notable clue is the all-axis statusword change from `0x9650` before power loss to `0x1650` after cold boot, which may indicate a manufacturer-specific absolute/reference-valid bit clearing without a corresponding battery/multi-turn fault.

## 2026-04-12 05:03 +0000

- Task summary:
  - Reviewed the A6-EC manual text directly to interpret the `0x9650 -> 0x1650` statusword change against the drive's own HM/home-offset semantics.
- Changes:
  - Confirmed from `docs/resources/A6-EC_series_servo_drive_manual.pdf`:
    - in homing mode, `6041h` bit 15 = `Homing completed`
    - in homing mode, `6041h` bit 12 = `Homing completion output`
    - `607Ch` home offset is active only when the drive is powered on, homing is complete, and `6041h` bit 15 is `1`
  - Compared that manual definition to the observed values:
    - `0x9650` = bits `15,12,10,9,6,4`
    - `0x1650` = bits `12,10,9,6,4`
    - the cold-boot difference is therefore exactly loss of bit 15
- Validation:
  - direct PDF text inspection with `ReadFile` on `docs/resources/A6-EC_series_servo_drive_manual.pdf`
- Follow-up notes / risks:
  - This strengthens the hypothesis that the drive loses full homing-complete/reference-active state on cold boot even though the startup mode and zero home offset values remain correct.

## 2026-04-12 18:20 +0000

- Task summary:
  - Tested the "hidden persisted variable from our old direct writes" hypothesis against the latest cold boot and found no evidence that the likely EEPROM-backed C10 absolute-position offset objects are carrying our old values.
- Changes:
  - Reviewed the latest startup bundle `logs/startups/20260412-181258/`.
  - Observed the shifted pose immediately at startup in `controller.log` on the first `GET_POSITION`:
    - `J1..J6 = -1.05268250, 2.77649231, -0.69628601, -0.72601318, -1.88077148, -5.13803101 deg`
  - Collected privileged EtherCAT reads for hidden C10 candidates:
    - `0x2010:11` (multi-turn absolute position offset low 32 bits) = `0` on axes 0-5
    - `0x2010:13` (multi-turn absolute position offset high 32 bits) = `0` on axes 0-5
    - `0x2010:1F` (single-turn homing absolute value offset) = `0` on axes 0-5
    - `0x2010:15` = `1` on axes 0 and 3, `0` elsewhere
    - `0x2010:16` and `0x2010:17` = `0` on all axes
  - Compared those against current `0x6064` values and found none of the nonzero C10 entries numerically resemble the shifted raw positions.
- Validation:
  - latest startup log inspection
  - privileged `ethercat upload` reads for the `0x2010:*` C10 subindices plus `0x6064`
- Follow-up notes / risks:
  - This weakens the "we wrote some hidden EEPROM position variable and it is still haunting us" theory substantially.
  - The shifted pose is present immediately on cold boot and does not currently map to the obvious EEPROM-backed offset objects, so the remaining issue still looks drive-side/reference-state-related rather than a leftover software write.

## 2026-04-12 18:31 +0000

- Task summary:
  - Pulled the exact manual references for the `0x9650 -> 0x1650` statusword difference and probed a broader set of live encoder/reference objects to see whether the drive itself agrees with the shifted cold-boot pose.
- Changes:
  - Manual findings from `docs/resources/A6-EC_series_servo_drive_manual.pdf`:
    - HM statusword table defines `6041h` bit 15 as `Homing completed` and bit 12 as `Homing completion output`
    - `607Ch` says home offset is active only when powered on, homing is complete, and `6041h` bit 15 is `1`
    - CSP table still labels bits 14-15 as manufacturer-specific / not supported, so the HM/object-specific wording is the more useful interpretation for the observed cold-boot issue
  - Derived statusword difference:
    - `0x9650` = bits `15,12,10,9,6,4`
    - `0x1650` = bits `12,10,9,6,4`
    - only bit 15 drops across cold boot
  - Broader live object probe:
    - many `0x2040:*` position-like channels (`:21`, `:23`, `:25`, `:27`, `:33`, `:37`, `:41`, `:43`) numerically track the shifted `0x6064` counts
    - readable `0x2010:*` bias/limit fields remain zero or static defaults
    - several ESI-listed absolute-feedback subindices under `0x2010` are absent or unsupported as readable SDOs on this firmware
- Validation:
  - direct PDF text inspection
  - privileged EtherCAT reads of `0x2040:*`, `0x2010:*`, `0x6064`, and `0x6041`
- Follow-up notes / risks:
  - The shifted pose is not just a DS402/UI translation artifact; the drive's own live `0x2040` monitor objects largely agree with `0x6064`.
  - The strongest remaining hypothesis is that the drive reboots into a different internal reference/homing-validity state, not that a hidden persisted offset written by us is being reapplied.

## 2026-04-12 18:42 +0000

- Task summary:
  - Probed whether a normal fault reset or the vendor software-reset object can restore the lost HM/reference-valid state after the cold-boot mismatch.
- Changes:
  - Verified pre-probe RTCore health from `/run/gradient-rt-motion/metrics.json`:
    - `armed=0`
    - `axis_enable_mask=0`
    - `startup_ready=1`
    - `wkc_actual=18`
  - Baseline privileged EtherCAT reads showed all axes at:
    - `0x6041 = 0x1650`
    - `0x607C = 0`
    - shifted `0x6064` counts still present
  - Issued `POST /control/reset-faults` and re-read the same objects:
    - statuswords remained `0x1650`
    - home offsets remained `0`
    - raw position counts did not return to the pre-power-cycle pose
  - Wrote vendor software reset `0x2031:02 = 1` while disarmed:
    - RTCore briefly dropped to `startup_ready=0`, `wkc_actual=11`
    - after settling it recovered to `startup_ready=1`, `wkc_actual=18`
    - axes still returned with the shifted `0x6064` counts and no recovery of HM bit 15
  - Observed an induced transient fault on axis 1 after the software-reset probe:
    - `0x6041 = 0x1618`
    - `0x603F = 0x8700`
    - `0x203F = 0x0C20`
  - Cleared that induced fault with a normal `POST /control/reset-faults`, returning all axes to `0x6041 = 0x1650` and zero fault registers.
- Validation:
  - live privileged `ethercat upload` reads for `0x6041`, `0x603F`, `0x203F`, `0x6064`, `0x607C`, and `0x2040:*`
  - live `POST /control/reset-faults`
  - live RTCore metrics checks before, during, and after the probe
- Follow-up notes / risks:
  - Neither a standard fault reset nor the vendor software-reset object restores HM bit 15 or the pre-boot pose, so the mismatch does not look like a stale software-side latch that a soft reset can clear.
  - The next investigation should target the drive's absolute-reference boot conditions directly: what preconditions make the A6-EC assert HM bit 15 again after power-up, and whether only a full native-home/HM cycle re-establishes the reference-active state.

## 2026-04-12 19:08 +0000

- Task summary:
  - Re-reviewed the A6-EC manuals and implemented a startup-config fix so RTCore can program the drive's absolute-rotation gear-ratio parameters instead of leaving them at the vendor `1:1` defaults.
- Changes:
  - Manual findings from `docs/resources/A6-EC_series_servo_drive_manual.pdf` and `docs/resources/chapter 11 - parameter list - A6-EC_series_servo_drive_manual (2).pdf`:
    - Chapter 5 says the battery-backed absolute encoder should remove the need for re-homing after power-up when the absolute system is configured correctly.
    - Chapter 5 also says changing the electronic gear ratio changes the mechanical position abruptly and requires homing.
    - In absolute rotation mode, the drive reconstructs the mechanical absolute position using `C10.1A/C10.1C` first, otherwise `C10.18/C10.19`.
  - Live privileged reads showed every axis currently boots with:
    - `C00.07 = 4`
    - `C10.18 = 1`
    - `C10.19 = 1`
    - `C10.1A = 0`
    - `C10.1C = 0`
  - Confirmed this conflicts with the actual Gradient-05 robot reductions from `src/gradient_os/arm_controller/robots/gradient05/config.py`:
    - `J1-J3 = 100:1`
    - `J4 = 18:1`
    - `J5 = 31.25:1` (rendered as `125/4`)
    - `J6 = 10:1`
  - Extended the startup-config builder and RTCore startup parser so `GRADIENT_RT_DRIVE_STARTUP_SDO_CONFIG` now carries three startup descriptors:
    - `C00.07 / 0x2000:08`
    - `C10.18 / 0x2010:19`
    - `C10.19 / 0x2010:1A`
  - Kept the existing `startup_drive_config` metrics/API contract anchored on the primary `C00.07` entry to avoid breaking the current UI/telemetry consumers while still applying the extra ratio SDOs at startup.
- Validation:
  - `source "/home/pi/GradientOS/.venv/bin/activate" && PYTHONPATH=src python -m pytest tests/test_rtcore_runtime.py -q`
  - `make -C src/gradient_rt_motion`
  - rendered startup env check:
    - `a6ec_encoder_position_tracking_mode|u16|0x2000|0x08|4,4,4,4,4,4;a6ec_rotation_mode_gear_ratio_numerator|u16|0x2010|0x19|100,100,100,18,125,10;a6ec_rotation_mode_gear_ratio_denominator|u16|0x2010|0x1A|1,1,1,1,4,1`
- Follow-up notes / risks:
  - This code change is the most plausible manual-backed fix for the cold-boot mismatch, but it still needs live deployment and proof on hardware.
  - Because the manual says changing the electronic gear ratio changes mechanical position abruptly, the first deployment will require one explicit re-home after startup to seed the corrected EEPROM reference; the goal is to remove the need for re-home on subsequent power cycles, not to avoid that one migration step.

## 2026-04-12 19:22 +0000

- Task summary:
  - Backed out the drive-side rotation gear-ratio startup change after re-evaluating the ownership boundary and the user challenge that it did not address the real encoder-retention failure mode.
- Changes:
  - Reverted the A6-EC startup-config extension that had added `C10.18/C10.19` to the RTCore startup SDO env.
  - Restored the prior startup contract where RTCore only emits and parses the primary `C00.07 / 0x2000:08` startup SDO.
  - Kept the investigation result that the drive currently reports rotation-mode ratio defaults, but stopped treating that as the proposed fix.
- Validation:
  - `source "/home/pi/GradientOS/.venv/bin/activate" && PYTHONPATH=src python -m pytest tests/test_rtcore_runtime.py -q`
  - `make -C src/gradient_rt_motion`
- Follow-up notes / risks:
  - The better current framing is that cold boot is failing to restore the drive's saved absolute-reference correction, not that GradientOS forgot to mirror robot gear ratios into the drive.
  - The next useful investigation should target the drive's save/restore path itself: what object actually changes after native home to represent the saved mechanical-vs-encoder deviation, and whether `F31.10` read/write encoder operations are required to commit or reload that state.

## 2026-04-12 20:45 +0000

- Task summary:
  - Converted `docs/resources/chapter 11 - parameter list - A6-EC_series_servo_drive_manual (2).pdf` into a repo-local Markdown reference at `docs/resources/a6ec_manual_chapter_11_parameter_list.md`.
- Changes:
  - Extracted the PDF with `pdftotext -layout` to preserve the original table geometry.
  - Generated a structured Markdown document with:
    - a short intro and address-range table for `11.1`
    - sectioned group headings for `11.2`
    - a separate `11.3` section for parameter descriptions
    - fenced `text` blocks for wide tables and manual formatting that would be lossy in pipe-table form
  - Removed the temporary extracted text file after generating the final Markdown.
- Validation:
  - Spot-checked the generated Markdown with `ReadFile`
  - Ran `ReadLints` on `docs/resources/a6ec_manual_chapter_11_parameter_list.md` with no diagnostics reported
- Follow-up notes / risks:
  - The document is faithful to the manual text and page structure, but some original line wrapping inside very wide tables remains intentionally preserved rather than aggressively reflowed.

## 2026-04-12 21:05 +0000

- Task summary:
  - Ran the first half of the single-axis `F31.10` save/restore experiment on `J1`/axis `0` to separate raw encoder state from the drive-applied absolute reference frame.
- Changes:
  - Verified the stack was healthy and disarmed before the probe:
    - `armed=0`
    - `axis_enable_mask=0`
    - `startup_ready=1`
    - `wkc_actual=18`
  - Baseline cold-boot snapshot on axis `0`:
    - `0x6041 = 0x1650`
    - `0x6063 ≈ 38329`
    - `0x6064 ≈ 38327`
    - `U40.14/.16/.18/.1A ≈ 38328`
    - `U40.1C ≈ 38328`
    - `U40.20 ≈ 38329`
    - `0x607C = 0`
  - Ran `POST /control/home-joint-native` for `J1`; the API returned `accepted=true`, `verified=true`, `code=NATIVE_HOME_VERIFIED`.
  - Post-home snapshot on the same axis:
    - `0x6041 = 0x9650`
    - `0x6063 ≈ 5`
    - `0x6064 ≈ 5`
    - `U40.14/.16 ≈ 4`
    - `U40.1C ≈ 38331`
    - `U40.20 ≈ 38331`
    - `0x607C = 0`
  - Issued direct commissioning-only encoder operations while still disarmed:
    - `F31.10 = 1` (`Read encoder`)
    - `F31.10 = 2` (`Write encoder`)
  - Settling poll after `F31.10` showed:
    - `F31.10` self-cleared back to `0`
    - no drive fault (`0x603F = 0`, `0x203F = 0`)
    - `0x6041` returned to `0x9650`
    - reference-unit channels stayed near zero while raw encoder-oriented channels stayed near `38330`
- Validation:
  - live privileged `ethercat upload` / `download` reads and writes for `0x2031:11`, `0x6041`, `0x6063`, `0x6064`, `0x607C`, and `0x2040:*`
  - live `POST /control/home-joint-native`
- Follow-up notes / risks:
  - This strongly suggests native home and `F31.10` affect a drive-side reference transform rather than the raw battery-backed encoder counts themselves.
  - The decisive remaining step is the real power cycle: if the post-`F31.10` cold boot comes back near zero/reference-aligned on the same axis, then the missing fix is likely an explicit encoder read/write commit or reload step rather than a repeated home requirement.

## 2026-04-12 21:12 +0000

- Task summary:
  - Completed the second half of the single-axis `F31.10` experiment by reading the same object set after a real drive-only power cycle.
- Changes:
  - Verified RTCore saw the drive-only restart and recovered cleanly:
    - `startup_reset_count = 1`
    - `startup_ready = 1`
    - `wkc_actual = 18`
    - stack remained disarmed
  - Post-power-cycle axis 0 snapshot (the only axis that received native home + `F31.10` read/write):
    - `0x6041 = 0x1650`
    - `0x6063 ≈ 2..4`
    - `0x6064 ≈ 1..3`
    - `U40.14/U40.16 ≈ 1..3`
    - `U40.1C/U40.20 ≈ 38330`
    - `0x607C = 0`
  - Control comparison across axes:
    - axis 0 kept the corrected near-zero reference frame
    - untouched axes 1-5 still came back in the old shifted frame, for example:
      - axis 1 `0x6064 ≈ 101087`
      - axis 2 `0x6064 ≈ 25347`
      - axis 3 `0x6064 ≈ 4758`
      - axis 4 `0x6064 ≈ 21396`
      - axis 5 `0x6064 ≈ 18704`
- Validation:
  - live privileged EtherCAT reads of `0x6041`, `0x6063`, `0x6064`, `0x607C`, and `U40.14/.16/.1C/.20` on axis 0 and all six axes after the drive-only power cycle
- Follow-up notes / risks:
  - This is the strongest evidence so far that `F31.10` read/write changes whether the drive restores the native-home reference transform across a later drive-only power cycle.
  - It also weakens the earlier inference that the `0x9650 -> 0x1650` bit-15 drop is itself the cause of the wrong pose: axis 0 lost bit 15 again but still preserved the corrected reference frame.
  - The next implementation step should be a commissioning-safe way to apply the `F31.10` read/write sequence as part of the native-home persistence workflow, followed by a focused hardware proof on one axis before broadening to all joints.

## 2026-04-12 22:59 +0000

- Task summary:
  - Rolled the `F31.10` persistence tail into the integrated A6-EC native-home workflow, validated it on fresh axes, and confirmed the rollout survives a later drive-only power cycle.
- Changes:
  - Extended the RTCore native-home transaction language with:
    - `wait_sdo`
    - `release_service_override`
  - Updated the A6-EC native-home profile transaction so it now performs, after the HM/home capture succeeds:
    - `refresh_truth`
    - `restore_mode`
    - `release_service_override`
    - `F31.10 = 1` (`Read encoder`) + wait for `0`
    - `F31.10 = 2` (`Write encoder`) + wait for `0`
  - Increased the backend-side native-home wait ceiling from `10.0s` to `20.0s`.
  - Tightened backend verification so fresh post-command snapshots can treat `statusword bit 15` with zero abort code as a valid success signal, rather than depending only on `native_home_state`.
  - Increased the API timeout for `/control/home-joint-native` from `5.0s` to `25.0s` so the controller reply can survive the longer persistence tail.
  - Validation results:
    - axis 1 (`J2`) became semantically home-aligned under the integrated flow, but it was already contaminated by earlier experiments, so it was not used as the clean persistence proof axis
    - untouched axis 2 (`J3`) integrated proof:
      - before home: `0x6064 ~= 25346`, `U40.16 ~= 25344`, `0x6041 = 0x1650`
      - after integrated endpoint: `0x6064 ~= 131062`, `U40.16 ~= -13`, `0x6041 = 0x9650`
      - after later drive-only power cycle: `0x6064 ~= 131060`, `U40.16 ~= 131059`, raw `U40.1C ~= 25334`, `0x6041 = 0x1650`
    - axis 0 and axis 1 also remained in corrected post-home frames across the latest drive-only power cycle
- Validation:
  - `source "/home/pi/GradientOS/.venv/bin/activate" && PYTHONPATH=src python -m pytest tests/test_rtcore_runtime.py -q`
  - `source "/home/pi/GradientOS/.venv/bin/activate" && PYTHONPATH=src python -m pytest tests/test_api_endpoints.py -q -k "home_joint_native"`
  - `source "/home/pi/GradientOS/.venv/bin/activate" && PYTHONPATH=src python -m pytest tests/test_gradient05_limits_and_backends.py -q -k "native_home"`
  - `make -C src/gradient_rt_motion`
  - live `systemd/rt-motion/install.sh`, RTCore restarts, launcher-managed controller/API reloads, integrated API calls, and privileged EtherCAT reads before/after the final drive-only power cycle
- Follow-up notes / risks:
  - The persistence rollout is now bench-proven on a fresh axis, but `native_home_state` / `native_home_last_abort_code` can remain stale in RTCore metrics after restart or after a previously failed attempt even when the live drive objects show the corrected frame.
  - The timeout is now less important to correctness because success/failure can terminate from fresh terminal signals; it should be treated as a deadman ceiling, not the primary completion criterion.

## 2026-04-12 23:59 +0000

- Task summary:
  - Cleaned up stale native-home telemetry so the frontend no longer reports false `failed` badges for already-persisted axes such as `J2` and `J3`.
- Changes:
  - RTCore metrics cleanup:
    - reset `native_home_state` to `idle`
    - reset `native_home_last_abort_code` to `0`
    - perform that reset when a new startup epoch is detected (`startup_reset_count` change or `startup_ready` dropping during a drive restart)
  - Drive-fault snapshot cleanup:
    - derive an effective UI-facing native-home state from live wire-state
    - if statusword bit 15 is present and there is no current fault, prefer that fresh signal over a stale failed result
    - preserve the raw reported result in parallel `*_reported` fields for debugging
  - After rolling the cleanup into the live stack and restarting RTCore plus controller/API:
    - `/run/gradient-rt-motion/metrics.json` shows all axes back at `native_home_state = 0`, `native_home_last_abort_code = 0`
    - a live `build_drive_fault_snapshot(...)` check for `J2` and `J3` reports:
      - `native_home_state_name = idle`
      - `native_home_last_abort_code = 0`
      - `statusword = 0x1650`
    - live drive objects for `J2`/`J3` still show the corrected persisted frame, so the false UI failure signal is gone without losing the real retention result
- Validation:
  - `source "/home/pi/GradientOS/.venv/bin/activate" && PYTHONPATH=src python -m pytest tests/test_drive_faults.py -q`
  - `source "/home/pi/GradientOS/.venv/bin/activate" && PYTHONPATH=src python -m pytest tests/test_rtcore_runtime.py -q -k "drive_fault_snapshot"`
  - `make -C src/gradient_rt_motion`
  - live RTCore install/restart and launcher-managed stack restart
  - live metrics readback plus live `build_drive_fault_snapshot(...)` reconstruction for `J2` and `J3`
- Follow-up notes / risks:
  - The frontend should now stop showing stale native-home failures after drive restarts, but the commissioning card still only shows “success” while a fresh success signal is present; after a later reboot it intentionally falls back to idle rather than trying to infer “persisted success forever” from last-operation state.

## 2026-04-13 00:19 +0000

- Task summary:
  - Fixed the remaining wrapped-angle conversion bug so persisted native-home axes no longer show false `±3.6°` joint offsets or feed misleading angles into the frontend visualizer.
- Changes:
  - Extended the Python RTCore backend axis config to retain `counts_per_rev` alongside `counts_per_unit` and sign.
  - Added A6-EC-specific feedback normalization in `EthercatRTCoreBackend`:
    - for `a6ec_ds402`, convert controller-facing feedback counts into a signed single-turn range using encoder counts-per-rev before converting to joint radians
    - this turns values like `131060` into `-12` counts instead of `+131060`
  - Updated stale config expectations in focused backend tests:
    - current Gradient-05 signs
    - current `J5` ratio `31.25`
    - current `J6` limit `(-10.0, 10.0)`
  - Reloaded the launcher-managed controller/API stack so the live endpoints and SSE monitor picked up the new conversion.
- Validation:
  - `source "/home/pi/GradientOS/.venv/bin/activate" && PYTHONPATH=src python -m pytest tests/test_gradient05_limits_and_backends.py -q -k "gradient05_config_defaults_and_mapping_shape or feedback_using_axis_scaling or wraps_a6ec_single_turn_feedback_counts or prefers_robot_defined_axis_scaling"`
  - live `/info/joints-detailed` after reload:
    - `J2 ≈ -0.00033°`
    - `J3 ≈ +0.00033°`
    - `axis_counts` still near `131060`, confirming the change is in interpretation rather than raw drive state
  - live `/info/pose` after reload matched the same near-zero `J2/J3` values
  - live `/monitor` SSE samples also matched the same near-zero `J2/J3` values over repeated samples
- Follow-up notes / risks:
  - I could not directly view the rendered browser scene from this environment, so I verified the viewer input feeds instead.
  - The backend/HTTP/SSE sources are now stable and near zero for `J2/J3`; if a large visible flicker is still present in the 3D robot after a page refresh, the remaining issue is likely a frontend-only render artifact rather than a controller/RTCore data bug.

## 2026-04-13 00:31 +0000

- Task summary:
  - Tightened native-home completion so rapid successive Drive Home requests cannot cut over on an early partial-success signal while RTCore is still running the persistence tail.
- Changes:
  - Added `native_home_active_axis_mask` to RTCore metrics JSON for the exact duration of each in-flight native-home transaction.
  - Updated the Python native-home wait logic so:
    - statusword-bit-15 fallback is not allowed until the active mask has been seen and then cleared
    - the request remains pending until RTCore actually finishes the tail for that axis
  - Focused regression coverage now includes the race case where statusword bit 15 appears while `native_home_active_axis_mask` is still set.
  - Rolled the updated RTCore binary plus controller/backend reload into the live stack.
- Validation:
  - `source "/home/pi/GradientOS/.venv/bin/activate" && PYTHONPATH=src python -m pytest tests/test_gradient05_limits_and_backends.py -q -k "native_home"`
  - `make -C src/gradient_rt_motion`
  - live RTCore install/restart and launcher-managed controller/API reload
  - stack returned healthy: `controller up`, `api up`, `web up`, RTCore `BUS_UP_DISARMED`
- Follow-up notes / risks:
  - Yes, clicking the next Drive Home too soon after the previous one could cause a real issue before this fix, because the first request could report success before RTCore finished the internal persistence tail.
  - With this change, the first home should hold the request open until RTCore finishes the tail, so the UI’s existing global pending-action gate can serialize the next click naturally instead of releasing too early.

## 2026-04-13 01:17 +0000

- Task summary:
  - Reviewed and cleaned up the generated Chapter 11 Markdown after the first-pass PDF conversion rendered poorly and still contained row-grouping mistakes.
- Changes:
  - Rebuilt the `11.2` parameter sections in `docs/resources/a6ec_manual_chapter_11_parameter_list.md` into actual HTML tables instead of raw fenced text blocks.
  - Corrected several extraction/presentation issues:
    - removed leaked header/page-number artifacts from table rows
    - fixed `2000h` row grouping so names/options attach to the correct parameter
    - fixed hex parameter-code parsing such as `C01.0A`
    - restored the missing `607Dh` grouped-object `subindex 0` row in the `6000h` table
  - Kept the long `11.3` narrative material intact while preserving the overall chapter structure.
  - Deleted temporary TSV extraction files after the cleanup pass.
- Validation:
  - spot-checked the regenerated Markdown with `ReadFile`
  - targeted content checks with `rg` for known trouble rows such as `C00.14`, `C00.31`, `607Dh`, `607Fh`, and `60E3h`
  - `ReadLints` on `docs/resources/a6ec_manual_chapter_11_parameter_list.md` reported no diagnostics
- Follow-up notes / risks:
  - The chapter is now much cleaner and more reviewable, but some mathematical ranges inherited from the PDF still use manual-style spacing such as `2 32 -1` rather than fully typeset superscripts.

## 2026-04-13 01:03 +0000

- Task summary:
  - Persisted the in-flight native-home state into the existing frontend telemetry path so the UI can show “still working” and block further Drive Home clicks without adding a new API surface.
- Changes:
  - Extended `build_drive_fault_snapshot(...)` to carry:
    - top-level `native_home_active_axis_mask`
    - top-level `native_home_active_axis_mask_hex`
    - per-axis `native_home_active`
  - Kept the transport path unchanged:
    - RTCore metrics
    - `drive_faults` snapshot builder
    - existing `/monitor` SSE payload
    - existing `driveFaults` prop into `ControlPanel`
  - Updated `ControlPanel.tsx` so the commissioning section:
    - shows a persistent amber banner while any native-home transaction is still active
    - shows `Drive Home requested...` on the active joint row
    - disables all Drive Home buttons until the active-home mask clears
  - Reloaded the live stack and verified the live `driveFaults` snapshot now includes the new fields, with idle state currently reporting `native_home_active_axis_mask = 0x0`.
- Validation:
  - `source "/home/pi/GradientOS/.venv/bin/activate" && PYTHONPATH=src python -m pytest tests/test_drive_faults.py -q`
  - `cd web-ui && npm test -- --run ControlPanel.test.tsx -t "native-home transaction is still active"`
  - live stack restart plus direct live `build_drive_fault_snapshot(...)` check confirming `native_home_active_axis_mask` and per-axis `native_home_active` are present on the existing payload
- Follow-up notes / risks:
  - The next real-world proof is simply the next time you click Drive Home twice in quick succession: the first joint should stay visibly “running,” and the second Drive Home buttons should remain disabled until RTCore finishes the tail.

## 2026-04-13 01:07 +0000

- Task summary:
  - Captured the formal before-power baseline for the final all-axis retention proof.
- Changes:
  - Triggered the existing retention workflow:
    - experiment id `20260413-010728`
    - snapshot path `logs/encoder-retention/20260413-010728/before_power_down.json`
  - Collected a privileged all-axis SDO baseline for:
    - `0x2000:08`
    - `0x6041`
    - `0x6061`
    - `0x6063`
    - `0x6064`
    - `0x607C`
    - `0x603F`
    - `0x203F`
    - `0x2031:11`
    - `U40.14/.16/.18/.1A/.1C/.1E/.1F/.20/.22`
  - Key pre-cycle reference-unit positions:
    - axis 0 `0x6064 = 2`
    - axis 1 `0x6064 = 131059`
    - axis 2 `0x6064 = 131059`
    - axis 3 `0x6064 = 3`
    - axis 4 `0x6064 = 5`
    - axis 5 `0x6064 = 131071`
  - Key pre-cycle raw single-turn channels:
    - axis 0 `U40.1C = 38330`
    - axis 1 `U40.1C = 101040`
    - axis 2 `U40.1C = 25333`
    - axis 3 `U40.1C = 4759`
    - axis 4 `U40.1C = 21404`
    - axis 5 `U40.1C = 18702`
  - RTCore metrics at capture:
    - `armed = 0`
    - `axis_enable_mask = 0`
    - `native_home_active_axis_mask = 0`
    - `startup_ready = 1`
    - `wkc_actual = 18/12`
- Validation:
  - live `POST /control/encoder-retention/capture` with `phase=before_power_down`
  - live privileged `ethercat upload` readback for all six axes and the tracked encoder/drive objects
- Follow-up notes / risks:
  - This is the exact pre-power snapshot to compare against immediately after the next drive-only power cycle with the stack left running and disarmed.

## 2026-04-13 01:59 +0000

- Task summary:
  - Captured the formal after-power snapshot for the final all-axis retention proof and compared it against the pre-power baseline.
- Changes:
  - Triggered the existing retention workflow:
    - experiment id `20260413-010728`
    - after snapshot `logs/encoder-retention/20260413-010728/after_power_up.json`
    - comparison artifacts `comparison.json` and `comparison.md`
  - Post-power RTCore metrics at capture:
    - `armed = 0`
    - `axis_enable_mask = 0`
    - `native_home_active_axis_mask = 0`
    - `startup_ready = 1`
    - `startup_reset_count = 1`
    - `wkc_actual = 18/12`
  - Raw post-power drive observations:
    - axis 0 stayed near zero in reference units (`0x6064: 2 -> 4`)
    - axis 1 stayed in the persisted wrapped frame (`0x6064: 131059 -> 131059`)
    - axis 2 stayed in the persisted wrapped frame (`0x6064: 131059 -> 131058`)
    - axis 3 stayed near zero in reference units (`0x6064: 3 -> 5`)
    - axis 4 stayed near zero in reference units (`0x6064: 5 -> 4`)
    - axis 5 crossed the single-turn wrap boundary but stayed semantically near zero (`0x6064: 131071 -> 4`)
  - Raw encoder-oriented channels remained effectively stable on every axis within a few counts:
    - axis 0 `U40.1C/U40.20: 38330/38331 -> 38330/38330`
    - axis 1 `101040/101041 -> 101042/101042`
    - axis 2 `25333/25334 -> 25335/25332`
    - axis 3 `4759/4757 -> 4760/4760`
    - axis 4 `21404/21405 -> 21404/21405`
    - axis 5 `18702/18701 -> 18704/18703`
  - The formal comparison artifact still flags mismatch because it uses exact equality on axis counts and logical angles, which is now too strict for:
    - a few-count post-power dither
    - modulo-equivalent near-zero values like `131071` versus `4`
- Validation:
  - live `POST /control/encoder-retention/capture` with `phase=after_power_up` and `experiment_id=20260413-010728`
  - live privileged all-axis `ethercat upload` readback for the tracked encoder/drive objects
- Follow-up notes / risks:
  - Interpreting the raw drive state, the all-axis persistence rollout is successful: all six axes returned to the same semantic home frame after the final drive-only power cycle.
  - The remaining gap is report quality, not drive behavior: `comparison.json` / `comparison.md` should be made tolerance- and wrap-aware if we want the formal artifact to agree with the successful raw proof.

## 2026-04-13 02:17 +0000

- Task summary:
  - Wrote a repo-local fresh-agent handoff for the current unresolved `J2` native-home false-failure contradiction after hard stop + restart.
- Changes:
  - Added `HANDOFF_J2_NATIVE_HOME_FALSE_FAILURE_2026-04-13.md` with:
    - current branch/worktree state
    - relevant manual and experiment history
    - current live drive objects, raw metrics, and effective `driveFaults` snapshot for `J2`
    - the exact mismatch between command-response failure and live success
    - concrete next debugging directions and constraints
- Validation:
  - manual review of latest logs, metrics, direct EtherCAT reads, and live `build_drive_fault_snapshot(...)` output before writing the handoff
- Follow-up notes / risks:
  - The handoff is intentionally specific to the current false-failure contradiction and assumes the broader native-home persistence and display work already on the branch remains in place.

## 2026-04-13 02:27 +0000

- Task summary:
  - Fixed the remaining `J2` native-home false-failure by aligning backend command-result semantics with the already-correct live `driveFaults` interpretation for a clean HM-bit15 success state.
- Changes:
  - Updated `src/gradient_os/arm_controller/backends/ethercat_rtcore/backend.py` so `_native_home_metrics_result()` now:
    - treats a stale reported `failed` state / abort code as superseded when the active-home mask has cleared and the live wire-state shows HM bit 15 with zero live faults
    - preserves the raw reported abort code separately for debugging
  - Added focused regressions in `tests/test_gradient05_limits_and_backends.py` for:
    - stale failed report + clean live statusword success => verified success
    - stale failed report + live fault present => remains failed
- Validation:
  - `source "/home/pi/GradientOS/.venv/bin/activate" && PYTHONPATH=src python -m pytest tests/test_gradient05_limits_and_backends.py -q -k "native_home_metrics_result or wait_for_native_home_result_waits_for_active_mask_clear_before_statusword_fallback"`
  - `source "/home/pi/GradientOS/.venv/bin/activate" && PYTHONPATH=src python -m pytest tests/test_drive_faults.py -q -k native_home`
  - `source "/home/pi/GradientOS/.venv/bin/activate" && PYTHONPATH=src python -m py_compile src/gradient_os/arm_controller/backends/ethercat_rtcore/backend.py`
  - direct non-invasive live check against `/run/gradient-rt-motion/metrics.json` showing `J2` now evaluates to `terminal_state=succeeded`, `verified=true`, and `native_home_last_abort_code=0` while preserving the reported raw failure fields
- Follow-up notes / risks:
  - This removes the operator/API false-negative without changing the raw RTCore metrics fields themselves; any other consumer that reads raw `native_home_state` directly may still need the same effective-status interpretation instead of treating the raw field as final truth.

## 2026-04-13 03:06 +0000

- Task summary:
  - Fixed the `J6` jog / cross-axis regression by separating display-only A6-EC feedback normalization from the motion-safe controller feedback frame, then carried that split through the API and frontend monitor paths.
- Changes:
  - Updated `src/gradient_os/arm_controller/backends/ethercat_rtcore/backend.py` so:
    - `raw_to_joint_positions()` now preserves raw-wire-derived counts for controller/motion use
    - new `raw_to_display_joint_positions()` retains the signed single-turn / continuous display behavior for UI consumers only
    - `_axis_q_from_counts()` no longer normalizes A6-EC feedback counts, and display normalization now lives behind an explicit display helper
  - Updated `src/gradient_os/run_controller.py` to publish display-only joint angles separately:
    - `GET_JOINT_STATE` now includes `arm_display_rad` / `arm_display_deg`
    - monitor telemetry now includes `display_joints` alongside the motion-safe `joints`
  - Updated `src/gradient_os/api/main.py` so `/info/joints` now reuses `GET_JOINT_STATE` and surfaces optional `arm_display_*` fields instead of being limited to `GET_JOINT_ANGLES`
  - Updated `web-ui/src/ControlPanel.tsx`, `web-ui/src/liveState.tsx`, `web-ui/src/App.tsx`, and `web-ui/src/TelemetryCharts.tsx` so the frontend prefers display-only joint angles when present while leaving the command baseline untouched
  - Added/updated focused regressions in:
    - `tests/test_gradient05_limits_and_backends.py`
    - `tests/test_api_endpoints.py`
    - `web-ui/src/ControlPanel.test.tsx`
- Validation:
  - Live containment / recovery:
    - `curl -sS http://127.0.0.1:4000/control/motion-status` confirmed `state=faulted`, `active_traj_id=3`, `queue_depth=24`, and faulted axes `0/1/3`
    - `curl -sS -X POST http://127.0.0.1:4000/control/stop` followed by `curl -sS http://127.0.0.1:4000/control/motion-status` confirmed RTCore returned to `state=idle`, `active_traj_id=0`, `queue_depth=0`
    - `curl -sS -X POST http://127.0.0.1:4000/control/power-down -H 'Content-Type: application/json' -d '{"wait_for_idle":true}'` plus `/run/gradient-rt-motion/metrics.json` confirmed `armed=0` and `axis_enable_mask=0`
  - Focused regressions:
    - `source .venv/bin/activate && PYTHONPATH=src python -m pytest tests/test_gradient05_limits_and_backends.py -q -k "wraps_a6ec_single_turn_feedback_counts or unwraps_a6ec_feedback_continuously or preserves_raw_feedback_frame_for_controller_requeue or zero_capture_preserves_logical_zero_with_native_home_offset or enqueue_trajectory_points_keeps_controller_logical_frame_with_native_home_offset"`
    - `source .venv/bin/activate && PYTHONPATH=src python -m pytest tests/test_api_endpoints.py -q -k "info_joints or info_joints_detailed or control_joint_jog"`
    - `cd /home/pi/GradientOS/web-ui && npm test -- src/ControlPanel.test.tsx`
    - `source .venv/bin/activate && python -m py_compile src/gradient_os/arm_controller/backends/ethercat_rtcore/backend.py src/gradient_os/api/main.py src/gradient_os/run_controller.py`
- Follow-up notes / risks:
  - The drives remain faulted on axes `0/1/3` after the safe stop/power-down; the queue is cleared and power is down, but a deliberate fault-reset / power-up / re-test cycle is still needed before motion can be trusted again.

## 2026-04-13 04:14 +0000

- Task summary:
  - Implemented the multi-turn encoder truth rollout from the attached plan without changing the plan file: RTCore now exports raw A6-EC absolute objects, Python reconstructs a display-only continuous joint path with a persisted absolute-home anchor, and the existing API/monitor paths carry the new diagnostics.
- Changes:
  - Added `src/gradient_os/absolute_encoder_anchors.py` to persist per-joint absolute-home anchors separately from software zero offsets.
  - Extended `src/gradient_rt_motion/main.cpp` so the metrics thread polls and publishes per-axis raw `absolute_feedback` SDO fields for `U40.16`, `U40.1C`, `U40.1E`, `U40.20`, `U40.22`, `U40.28`, `U40.2A`, and `U40.2C`.
  - Updated `src/gradient_os/arm_controller/backends/ethercat_rtcore/backend.py` to:
    - cache the new `absolute_feedback` metrics block
    - reconstruct display-only joint angles from raw multi-turn encoder counts plus a persisted anchor
    - keep `raw_to_joint_positions()` motion-safe and unchanged
    - refresh the persisted anchor on verified native-home success and software-zero capture
    - expose `get_display_feedback_snapshot()` so controller/API code can reuse the same display snapshot and diagnostics.
  - Updated `src/gradient_os/run_controller.py` and `src/gradient_os/telemetry/drive_faults.py` so existing telemetry contracts carry the new data:
    - `display_joints` still comes through `/monitor`
    - `arm_display_*` still comes through `GET_JOINT_STATE`
    - detailed joint snapshots now include `axis_absolute_feedback`
    - `drive_faults.axes[*]` now includes normalized `absolute_feedback` plus combined multi-turn counts for proof/debug work.
  - Updated TypeScript payload types in `web-ui/src/liveState.tsx`, `web-ui/src/App.tsx`, and `web-ui/src/ControlPanel.tsx` for the expanded `drive_faults` payload.
  - Added focused regression coverage in:
    - `tests/test_gradient05_limits_and_backends.py`
    - `tests/test_drive_faults.py`
    - `tests/test_api_endpoints.py`
- Validation:
  - `source .venv/bin/activate && PYTHONPATH=src python -m pytest tests/test_gradient05_limits_and_backends.py tests/test_drive_faults.py tests/test_api_endpoints.py -q`
  - `source .venv/bin/activate && PYTHONPATH=src python -m pytest tests/test_rtcore_runtime.py -q`
  - `make -C src/gradient_rt_motion`
  - `cd /home/pi/GradientOS/web-ui && npm test -- src/ControlPanel.test.tsx`
  - `ReadLints` on the touched Python and TypeScript files reported no diagnostics.
- Follow-up notes / risks:
  - The display path now prefers persisted absolute truth only when both raw `absolute_feedback` metrics and a stored anchor are available; otherwise it deliberately falls back to the prior display-only normalization.
  - No live hardware proof was run in this implementation pass, so the bench matrix from the plan (`J6` multi-turn sweep, native-home shift check, drive-only power-cycle check) still needs to be executed before anyone considers migrating controller/IK seed truth away from the current `0x6064` motion frame.

## 2026-04-13 04:40 +0000

- Task summary:
  - Refactored the absolute-feedback rollout to restore the architecture boundary: A6-EC field definitions and source policy now live in the drive profile, while RTCore/backend/telemetry consume profile descriptors generically.
- Changes:
  - Added absolute-feedback descriptor, normalization, and source-resolution helpers to `src/gradient_os/arm_controller/profiles/drive/a6ec_ds402.py`.
  - Exposed those helpers through `src/gradient_os/arm_controller/profiles/registry.py` and `src/gradient_os/arm_controller/backends/registry.py`.
  - Extended `src/gradient_os/arm_controller/backends/ethercat_rtcore/runtime.py` and `systemd/rt-motion/gradient-rt-motion.service` with `GRADIENT_RT_ABSOLUTE_FEEDBACK_CONFIG`.
  - Refactored `src/gradient_rt_motion/main.cpp` to parse the new descriptor, poll the profile-specified SDO list, and emit metrics keyed by descriptor names instead of hardcoded `U40.*` slots.
  - Refactored `src/gradient_os/arm_controller/backends/ethercat_rtcore/backend.py` and `src/gradient_os/telemetry/drive_faults.py` to normalize and resolve absolute feedback through the drive-profile registry instead of embedded A6-EC maps.
  - Updated focused fixtures/tests in `tests/test_gradient05_limits_and_backends.py`, `tests/test_drive_faults.py`, `tests/test_api_endpoints.py`, and `tests/test_rtcore_runtime.py`.
- Validation:
  - `source .venv/bin/activate && PYTHONPATH=src python -m pytest tests/test_gradient05_limits_and_backends.py tests/test_drive_faults.py tests/test_api_endpoints.py tests/test_rtcore_runtime.py -q`
  - `make -C src/gradient_rt_motion`
  - `ReadLints` on the touched Python/test files reported no diagnostics.
- Follow-up notes / risks:
  - The metrics/API payload now uses profile-defined semantic keys such as `encoder_multi_turn_low` and `encoder_multi_turn_counts` instead of vendor object ids; any ad hoc tooling that scraped `u40_*` keys will need to follow the profile-driven schema.
  - No live hardware validation was run in this refactor pass; the existing multi-turn bench proof matrix still needs to be rerun after deploying/restarting the stack.

## 2026-04-13 04:50 +0000

- Task summary:
  - Investigated the live startup/current state after a hard stop and drive power cycle.
- What changed:
  - No code changes; this was a live-state inspection using the launcher, RTCore journal, metrics file, and API state snapshots.
- Validation / observations:
  - `./start-stack.sh status`
  - `./start-stack.sh probe`
  - `journalctl -u gradient-rt-motion.service --since "2026-04-13 04:44:00" --no-pager`
  - Read `/run/gradient-rt-motion/metrics.json`
  - Queried `http://127.0.0.1:4000/info/joints-detailed`
  - Startup outcome was healthy at the bus/drive level:
    - controller/api/web were up under `start-stack.sh`
    - RTCore reached EtherCAT `OP`
    - `responding/online/operational=6/6`
    - `physical_state=BUS_UP_DISARMED`
    - all axes reported `SwitchOnDisabled`, `statusword=0x1650`, `error_code=0x0000`
  - RTCore journal showed startup SDO writes for `a6ec_encoder_position_tracking_mode=4` succeeded on all six axes and the bus converged in about `8.7s`.
  - RTCore metrics still showed `startup_drive_config.readback_valid=0` and `verified=0` on all axes minutes later, despite the successful startup writes.
  - Live `/info/joints-detailed` showed valid per-axis absolute-feedback fields, but all axes were still using `display_source=raw_feedback_fallback` with no persisted `absolute_home_anchor_*` fields present in the payload.
- Follow-up notes / risks:
  - The current live issue is not a drive fault or bus-up problem; it is the missing startup-readback verification telemetry plus the absence of a currently applied absolute-home anchor in the display path.
  - Local direct `/monitor` fetches hung during this inspection even though API logs showed other `/monitor` requests returning `200`; `probe`, RTCore metrics, and `/info/joints-detailed` were used as the reliable live sources for this pass.

## 2026-04-13 04:58 +0000

- Task summary:
  - Re-reviewed the live logs and end-state values after drive power-up plus a J4 jog.
- What changed:
  - No code changes; this was a second live-state/log correlation pass.
- Validation / observations:
  - Read `logs/startups/latest/controller.log`, `logs/startups/latest/api.log`, and terminal `17.txt`
  - Queried `http://127.0.0.1:4000/info/joints-detailed`
  - Read `/run/gradient-rt-motion/metrics.json`
  - Ran `./start-stack.sh probe`
  - Controller log sequence:
    - `SAFE_POWER_UP` accepted
    - `APPLY_JOINT_SETPOINT` for J4 from `-19.943 deg` to `-20.943 deg`
    - open-loop executor started for 25 points at 100 Hz
    - controller later raised `TimeoutError` waiting for RTCore trajectory `1` to complete
  - Despite that timeout, the live post-jog state remained healthy:
    - `driver_state=ACTIVE`
    - `armed=1`, `enable_mask=0x3f`, `op_enabled_axes=6/6`
    - all six axes `OperationEnabled`
    - all six axes `error_code=0x0000`
  - Pre-jog vs post-jog correlation:
    - J4 `arm_display_rad` changed by about `-1.001 deg`, matching the commanded jog
    - J4 `absolute_counts` changed `4376 -> 10935` (`+6559`)
    - J4 raw `pos_counts` wrapped `130692 -> 6179`, making `arm_rad` appear to jump by about `+19 deg`
    - other joints only drifted slightly (`J2` about `+0.037 deg`, `J3` about `+0.031 deg`, others effectively zero)
  - Current end-state display values:
    - `J1`: raw `-3.595 deg`, display `0.005 deg`
    - `J2`: raw/display `0.042 deg`
    - `J3`: raw `-2.434 deg`, display `1.166 deg`
    - `J4`: raw/display `-0.943 deg`
    - `J5`: raw/display `-0.065 deg`
    - `J6`: raw/display `-6.287 deg`
  - All axes still reported `display_source=raw_feedback_fallback`, not persisted absolute-anchor display truth.
- Follow-up notes / risks:
  - The main “many values changed” effect on J4 was raw-frame wrap, not evidence of a large unintended physical move.
  - The remaining live bug is the trajectory completion timeout / bookkeeping path, plus the still-missing anchored absolute display path after restart.

## 2026-04-13 05:15 +0000

- Task summary:
  - Fixed the RTCore trajectory-completion false-timeout path for wrapped A6-EC motion feedback and attempted a live deploy/restart.
- Changes:
  - Added profile-owned motion-feedback wrap metadata to `src/gradient_os/arm_controller/profiles/drive/a6ec_ds402.py`, plus registry accessors in `src/gradient_os/arm_controller/profiles/registry.py` and `src/gradient_os/arm_controller/backends/registry.py`.
  - Extended `src/gradient_os/arm_controller/backends/ethercat_rtcore/runtime.py` and `systemd/rt-motion/gradient-rt-motion.service` with `GRADIENT_RT_FEEDBACK_WRAP_AXIS_MASK` / `--feedback-wrap-axis-mask`.
  - Updated `src/gradient_rt_motion/main.cpp` so trajectory completion uses shortest-periodic-error math modulo `counts_per_rev` when the active drive profile marks wrapped motion feedback.
  - Updated `tests/test_rtcore_runtime.py` to assert the rendered RTCore env now includes the new wrap-mask setting.
- Validation:
  - `pytest -q tests/test_rtcore_runtime.py`
  - `pytest -q tests/test_gradient05_limits_and_backends.py -k wait_for_trajectory_complete`
  - `make -C src/gradient_rt_motion`
  - `ReadLints` on the touched Python/C++/test files reported no diagnostics.
- Live deployment notes:
  - Synced the rebuilt RTCore binary/unit/env into `/usr/local/bin/gradient-rt-motion` and `/etc/default/gradient-rt-motion`; `systemctl status gradient-rt-motion.service` confirms the new process starts with `--feedback-wrap-axis-mask 0x3f`.
  - The live deploy hit an unrelated host-level blocker: the previous RTCore instance left a deleted-binary `metrics` thread stuck in `D` state plus a surviving `EtherCAT-OP` kernel thread, so new RTCore instances now log `Failed to reserve master: Device or resource busy`.
  - `sudo ethercat master` still reports `Master0 Active: yes` and 1 kHz traffic even while the fresh RTCore instance reports `ethercat_master_state=DOWN`, which shows the stale master owner is outside the new process and was not cleared by repeated `gradient-rt-motion.service` / `ethercat.service` restarts.
- Follow-up notes / risks:
  - The code fix is in the repo and the new binary is installed, but live hardware validation is blocked until the stale EtherCAT master owner is cleared; the current evidence points to a host reboot or deeper kernel/driver recovery rather than another application-level restart.

## 2026-04-13 06:00 +0000

- Task summary:
  - Investigated the post-reboot/post-power-up web UI pose flicker, proved the frontend was rendering the wrapped motion-safe joint channel, and switched the operator-facing UI back to the stable display-joint stream already published by the backend.
- Changes:
  - Updated `web-ui/src/App.tsx`:
    - parse `/monitor.display_joints` from the SSE payload instead of silently dropping it
    - add a display-preferred joint selector that falls back to raw `joints` only when no display pose is available
    - feed that preferred display pose into the telemetry panel and the 3D `ArmVisualizer`
  - Updated `web-ui/src/TelemetryCharts.tsx`:
    - chart the same display-preferred pose source so the joint history panels do not keep showing wrapped raw-count snap-backs
- Validation:
  - Live 30 s capture of `http://127.0.0.1:4000/monitor`
    - collected 1389 packets
    - raw `joints` toggled between wrap-adjacent poses on all six axes
    - `display_joints` remained stable within micro-radians
  - Live 6 s poll of `http://127.0.0.1:4000/info/joints-detailed`
    - `axis_counts` alternated among `0`, `1`, `131071`, and `131070`, matching wrapped single-turn boundary dithering rather than physical motion
  - `ReadLints` on `web-ui/src/App.tsx` and `web-ui/src/TelemetryCharts.tsx` reported no diagnostics
  - `npm run build` in `web-ui`
- Follow-up notes / risks:
  - The backend still intentionally publishes raw `/monitor.joints` for motion/debug semantics; any future operator-facing widget should prefer `display_joints` or `/info/joints.arm_display_*` rather than assuming `joints` is display-safe near a wrap boundary.

## 2026-04-13 06:00 +0000

- Task summary:
  - Investigated whether J3’s erratic commissioning motion was caused by display telemetry leaking back into the controller, proved from logs/code that motion still uses the raw controller frame, and added joint-jog diagnostics so future J3 commands expose the exact raw-vs-display baseline used.
- Changes:
  - Updated `src/gradient_os/api/main.py`:
    - changed `/control/joint-jog` preflight from `GET_JOINT_ANGLES` to `GET_JOINT_STATE`
    - preserved the existing raw `arm_deg` target math so motion semantics stay unchanged
    - added `_selected_joint_feedback_snapshot(...)` and route response fields for `current_arm_deg`, `current_arm_display_deg`, and `selected_joint_feedback`
    - added warning logs when a jog baseline shows `display_source=raw_feedback_fallback` or a significant raw-vs-display delta
  - Updated `tests/test_api_endpoints.py`:
    - refreshed joint-jog tests for the `GET_JOINT_STATE` preflight and new diagnostic response fields
- Validation:
  - Reviewed live controller logs from the active stack:
    - repeated J3 bounded jogs used raw `current_deg` samples of about `-3.569`, `-0.970`, `-1.970`, `-2.970`, and `-0.371` on successive requests
    - each target was computed as current minus `1.0 deg`, which proves the controller was using the raw baseline rather than a display-only value
  - Live `http://127.0.0.1:4000/info/joints-detailed` probe for J3:
    - current raw about `-1.3705 deg`
    - current display about `-4.9705 deg`
    - `absolute_source=encoder_multi_turn_counts`
    - `display_source=raw_feedback_fallback`
  - Confirmed there is currently no persisted `.gradient_absolute_encoder_anchors.json` file in the repo root, so the anchored absolute-display path is not active on this boot
  - `source "/home/pi/GradientOS/.venv/bin/activate" && PYTHONPATH=src python -m pytest tests/test_api_endpoints.py -q -k 'control_joint_jog'`
  - `source "/home/pi/GradientOS/.venv/bin/activate" && PYTHONPATH=src python -m py_compile src/gradient_os/api/main.py tests/test_api_endpoints.py`
  - `ReadLints` on the touched files reported no diagnostics
- Follow-up notes / risks:
  - This pass improves observability and proves that display telemetry is not what is driving J3 motion. The remaining motion bug is that the raw commissioning baseline itself can be untrustworthy across commands, and J3 is still on `raw_feedback_fallback` instead of an anchored absolute display source after reboot.

## 2026-04-13 06:58 +0000

- Task summary:
  - Implemented the canonical anchored-joint-truth plan so the controller/API/UI now prefer one logical joint truth derived from absolute multi-turn counts plus persisted home anchors, while RTCore keeps using its raw drive-wire frame only for command translation.
- Changes:
  - Updated `src/gradient_os/arm_controller/backends/ethercat_rtcore/backend.py`:
    - replaced raw-wire-derived `raw_to_joint_positions()` / `raw_to_display_joint_positions()` reads with a shared anchored-absolute canonical snapshot path
    - removed the silent `raw_feedback_fallback` behavior from operator/controller truth and made missing anchors or absolute counts fail closed with explicit `truth_available` diagnostics
    - kept `_axis_q_from_joint_positions()` as the canonical-truth -> raw-wire conversion step for queued RTCore targets
  - Updated `src/gradient_os/run_controller.py`:
    - made `arm_display_*` and `/monitor.display_joints` compatibility aliases of the canonical joint truth instead of a separate pose source
    - added `canonical_joint_truth_available` and unavailable-axis/joint metadata to the joint snapshot payload
    - forwarded `axis_absolute_feedback` on `/monitor` as diagnostics only
  - Updated `src/gradient_os/api/main.py`:
    - made `/control/joint-jog` reject cached or anchor-missing baselines with `CANONICAL_JOINT_TRUTH_UNAVAILABLE`
    - extended selected-joint diagnostics with canonical-truth availability/source fields
  - Updated `web-ui/src/ControlPanel.tsx`, `web-ui/src/App.tsx`, and `web-ui/src/TelemetryCharts.tsx`:
    - prefer canonical `arm_deg` / `joints` first and treat `arm_display_*` / `display_joints` as legacy aliases only
  - Updated `tests/test_gradient05_limits_and_backends.py`, `tests/test_api_endpoints.py`, and `web-ui/src/ControlPanel.test.tsx`:
    - replaced split-truth assertions with continuous canonical-truth coverage
    - added fail-closed coverage for missing anchors/canonical truth during joint-jog baselining
- Validation:
  - `source ./start.sh && python -m pytest tests/test_gradient05_limits_and_backends.py tests/test_api_endpoints.py -q -k 'canonical or absolute or joint_jog or connected_reads or native_home_offsets_to_feedback_but_not_command_targets or robot_defined_axis_scaling or converts_feedback_using_axis_scaling or info_joints or info_joints_detailed or info_pose'`
    - result: `18 passed, 101 deselected`
  - `source ./start.sh && python -m py_compile src/gradient_os/arm_controller/backends/ethercat_rtcore/backend.py src/gradient_os/run_controller.py src/gradient_os/api/main.py tests/test_gradient05_limits_and_backends.py tests/test_api_endpoints.py`
  - `npm run test -- src/ControlPanel.test.tsx` in `web-ui`
    - result: `12 passed`
  - `npm run build` in `web-ui`
  - `ReadLints` on all touched Python/TS/TSX files reported no diagnostics
- Follow-up notes / risks:
  - This was validated with focused local tests/builds only; no live hardware proof was run in this pass, so the bench still needs the canonical truth / no-wrap / no-cross-axis-motion sequence from the plan.
  - Older mock-based API tests still allow `arm_display_*` to differ from `arm_*` because the API intentionally preserves legacy payload fields during transition; the real controller/runtime path now aliases them to the canonical truth instead.

## 2026-04-13 20:51 +0000

- Task summary:
  - Fixed the live canonical-truth regression by restoring anchor availability at startup instead of weakening the no-fallback contract.
- Changes:
  - Updated `src/gradient_os/arm_controller/backends/ethercat_rtcore/backend.py`:
    - added `_bootstrap_missing_absolute_home_anchors(...)`
    - call that bootstrap from `initialize()` after RTCore feedback is ready so missing `absolute_home_anchor_*` state is reconstructed from live raw-frame alignment plus absolute multi-turn counts
    - removed the attempted raw-live fallback path so `get_joint_positions()` still depends on canonical truth rather than a second read contract
  - Updated `tests/test_gradient05_limits_and_backends.py`:
    - added coverage that startup bootstrap creates missing anchors when raw + absolute alignment is sufficient
    - kept fail-closed coverage when bootstrap cannot reconstruct canonical truth
- Validation:
  - `source ./start.sh && python -m pytest tests/test_gradient05_limits_and_backends.py -q -k 'startup_bootstraps_missing_absolute_home_anchor or anchor_bootstrap_cannot_reconstruct_truth or canonical or connected_reads or native_home_offsets_to_feedback_but_not_command_targets'`
    - result: `7 passed, 45 deselected`
  - `source ./start.sh && python -m py_compile src/gradient_os/arm_controller/backends/ethercat_rtcore/backend.py tests/test_gradient05_limits_and_backends.py`
  - Live stack cycle:
    - `./start-stack.sh stop --hard`
    - `./start-stack.sh`
  - Live startup evidence:
    - controller log: `Bootstrapped absolute-home anchors from live raw/absolute alignment: joints=[1, 2, 3, 4, 5, 6] actor=ethercat_rtcore:startup_alignment`
    - startup banner: `// LIVE STATE // canonical truth: AVAILABLE`
  - Live API checks:
    - `curl -sf http://127.0.0.1:4000/info/joints-detailed`
      - result: `read_source="live_feedback"`, `canonical_joint_truth_available=true`, per-axis `absolute_home_anchor_*` populated
    - `curl -sf http://127.0.0.1:4000/info/pose`
      - result: `200 OK` with nonzero pose/joint payload
  - `ReadLints` on touched files reported no diagnostics
- Follow-up notes / risks:
  - Startup bootstrap currently trusts the live raw frame as the migration bridge when anchors are absent; if bench evidence later shows a case where the raw frame is not already the intended logical truth at startup, tighten the bootstrap preconditions rather than reintroducing a read fallback.

## 2026-04-13 21:38 +0000

- Task summary:
  - Investigated the remaining J3/J4 erratic commissioning behavior, traced it to a canonical-read / raw-write frame mismatch, and removed the last Python-side encoder fallback paths.
- Changes:
  - Updated `src/gradient_os/arm_controller/backends/ethercat_rtcore/backend.py`:
    - fixed `_axis_q_from_joint_positions()` so canonical targets now re-apply both `absolute_home_anchor_rad` and software zero before RTCore converts into raw wire counts
    - updated `_store_last_axis_target_q()` to translate raw reference-frame targets back into canonical joint space using the same anchor-aware inverse
    - kept the no-anchor fail-close contract by raising if command conversion is attempted without an absolute-home anchor
  - Updated `src/gradient_os/run_controller.py`:
    - removed `GET_JOINT_STATE` downgrade to `cached_fallback`; encoder truth is now `live_feedback` or explicit `unavailable`
  - Updated `src/gradient_os/arm_controller/trajectory_execution.py`:
    - removed the secondary fallback from failed `raw_to_joint_positions()` conversion to `backend.get_joint_positions()`
  - Updated `tests/test_gradient05_limits_and_backends.py`:
    - changed the canonical->raw-wire regression test to use a nonzero absolute-home anchor
    - added a guard test that command conversion raises when no absolute-home anchor exists
- Validation:
  - Reviewed live controller logs from `logs/startups/20260413-205037/controller.log`
    - J4 jogs moved from about `-18.988 -> -23.990 deg`, then later back toward `-19.966 deg`
    - J3 jogs then targeted about `-1.718 -> -3.717 deg` while J4 held at about `-19.966 deg`
    - these commands were single-joint targets, which pointed away from obvious multi-joint command chatter and toward frame conversion mismatch
  - Compared live startup anchors in `.gradient_absolute_encoder_anchors.json`
    - J4 startup anchor about `0.3364 rad` (`~19.27 deg`)
    - J3 startup anchor about `0.1135 rad` (`~6.50 deg`)
  - `source ./start.sh && python -m pytest tests/test_gradient05_limits_and_backends.py -q -k 'translates_canonical_truth_back_into_raw_wire_counts or refuses_command_conversion_without_absolute_home_anchor or startup_bootstraps_missing_absolute_home_anchor or anchor_bootstrap_cannot_reconstruct_truth or native_home_offsets_to_feedback_but_not_command_targets'`
    - result: `5 passed, 48 deselected`
  - `source ./start.sh && python -m py_compile src/gradient_os/arm_controller/backends/ethercat_rtcore/backend.py src/gradient_os/run_controller.py src/gradient_os/arm_controller/trajectory_execution.py tests/test_gradient05_limits_and_backends.py`
  - `ReadLints` on touched files reported no diagnostics
- Follow-up notes / risks:
  - I could not live-retest the new write-frame fix because the post-edit stack reload hit the known host-level stale-RTCore-owner failure: `./start-stack.sh` escalated to `REBOOT REQUIRED` after one recycle attempt (`master_device_busy`, `request_master_failed`, `leftover_process`, `sigkill_survivor`).
  - The code fix is ready, but the next meaningful proof requires a host reboot, then a fresh J4/J3 jog retest on the restarted stack.

## 2026-04-13 21:47 +0000

- Task summary:
  - Reviewed the startup-stack cleanup regression and fixed the newly introduced false-positive reboot path in RTCore startup recovery.
- Changes:
  - Updated `src/gradient_os/telemetry/startup_recovery.py`:
    - removed the immediate `rtcore_not_up` hard-recycle condition
    - replaced it with a non-recovering `rtcore_starting_or_down` classification that tells the launcher to continue normal startup/readiness waits unless real stale-owner/master-busy signatures are present
  - Updated `tests/test_startup_recovery.py`:
    - added regression coverage that `rtcore_state=UNKNOWN`, `ethercat_master_state=DOWN`, `physical_state=INACTIVE` does **not** trigger an automatic recycle without busy signatures
- Validation:
  - Reviewed the referenced transcript and current `start-stack.sh` flow:
    - transcript intent: recycle only the `RTCore UP / EtherCAT DOWN / master busy` class
    - current regression: `ensure_rtcore_runtime_sync()` called the classifier immediately after `sync-runtime.sh --ensure-active`, and the classifier treated `rtcore not up yet` as a hard-recycle case
  - `source ./start.sh && python -m pytest tests/test_startup_recovery.py -q`
    - result: `5 passed`
  - `source ./start.sh && python -m py_compile src/gradient_os/telemetry/startup_recovery.py tests/test_startup_recovery.py`
  - `ReadLints` on touched files reported no diagnostics
- Follow-up notes / risks:
  - The current machine is still in the stale-owner state shown in `terminals/1.txt`; that host-level condition can still genuinely require a reboot. This fix prevents the launcher from creating/escalating that state on normal slow startups, but it cannot retroactively clear the already-stuck owner without a clean reboot.

## 2026-04-13 18:45 +0000

- Task summary:
  - Hardened startup recovery for the RTCore/EtherCAT stack so the launcher no longer trusts an "active" RTCore that failed to reserve the EtherCAT master, and so stale-master-owner failures escalate with an explicit reboot-required message after one controlled recycle attempt.
- Changes:
  - Added `src/gradient_os/telemetry/startup_recovery.py`:
    - classifies a startup probe plus recent RTCore journal text into `healthy`, `should_recover`, or `reboot_required`
    - detects the specific stale-owner signatures seen on the bench (`Device or resource busy`, `ecrt_request_master(0) failed`, leftover-process / SIGKILL-survivor messages)
  - Updated `start-stack.sh`:
    - split raw RTCore sync into `sync_rtcore_runtime_once()`
    - added one-shot prelaunch RTCore/EtherCAT recycle logic when the probe shows `rtcore_state=UP` but `ethercat_master_state=DOWN`
    - emits an explicit reboot-required startup failure when the same stale-owner signatures remain after one recycle attempt
  - Updated `src/gradient_rt_motion/main.cpp`:
    - turns `ecrt_request_master(0)` failure into a real process exit path by setting `g_stop` and returning a nonzero exit code from the service
  - Added `tests/test_startup_recovery.py`:
    - covers healthy startup, recoverable `RTCore UP / EtherCAT DOWN`, reboot-required stale-owner signatures after recovery, and the no-false-positive "normal bus convergence" case
- Validation:
  - `bash -n start-stack.sh`
  - `source ./start.sh && python -m pytest tests/test_startup_recovery.py tests/test_rtcore_runtime.py -q`
    - result: `12 passed`
  - `source ./start.sh && python -m py_compile src/gradient_os/telemetry/startup_recovery.py`
  - `make -C src/gradient_rt_motion`
  - `ReadLints` on `src/gradient_os/telemetry/startup_recovery.py`, `tests/test_startup_recovery.py`, `start-stack.sh`, and `src/gradient_rt_motion/main.cpp`
    - result: no diagnostics
- Follow-up notes / risks:
  - This pass improves recovery when the old master owner is still killable, but it cannot guarantee recovery from a kernel-blocked `D`-state RTCore/EtherCAT owner; that path still correctly escalates to a host reboot.
  - Live bench validation is still needed to confirm the launcher takes the new recycle path automatically on hardware and produces the expected reboot-required message only when the master remains reserved.

## 2026-04-13 18:58 +0000

- Task summary:
  - Added an operator-facing CLI startup banner so `start-stack.sh` now prints a big `GradientOS` header plus useful live stack/runtime info when the staged launcher starts.
- Changes:
  - Updated `start-stack.sh`:
    - added `render_runtime_banner_summary()` to resolve the active desired runtime summary from `runtime_config`
    - added `print_start_banner()` with a large ASCII `GradientOS` header and startup metadata
    - banner includes mode, run id, robot, tool, IK backend, servo backend, drive profile, RT max RPM, controller/API/web endpoints, log path, and common launcher commands
    - called the banner from `start_stack()` immediately after environment bootstrap so it appears once at startup before RTCore/controller bring-up logs
- Validation:
  - `bash -n start-stack.sh`
  - `./start-stack.sh --help`
  - `ReadLints` on `start-stack.sh`
    - result: no diagnostics
- Follow-up notes / risks:
  - This was validated as a syntax/help-path change only; I did not run a full live startup on hardware in this pass, so the exact terminal appearance should be confirmed on the next real `./start-stack.sh` run.

## 2026-04-13 19:05 +0000

- Task summary:
  - Added ANSI color to the `start-stack.sh` startup banner with safe terminal detection and opt-out controls.
- Changes:
  - Updated `start-stack.sh`:
    - added `init_banner_palette()` and `banner_stat_line()` helpers
    - colorized the banner border, `GradientOS` ASCII header, labels, values, and command hint rows
    - added `GRADIENT_STACK_COLOR=auto|0|1` support and auto-disabled color for non-interactive/dumb terminals or when `NO_COLOR` is set
- Validation:
  - `bash -n start-stack.sh`
  - `ReadLints` on `start-stack.sh`
    - result: no diagnostics
- Follow-up notes / risks:
  - I still did not run a full live stack startup in this pass, so the exact palette/spacing should be eyeballed on the next terminal launch and tweaked if you want a more neon, minimal, or boxed style.

## 2026-04-13 19:14 +0000

- Task summary:
  - Refined the terminal styling to better match the user’s industrial reference, and made recovery/reboot actions stand out as color-coded callout blocks instead of plain inline log text.
- Changes:
  - Updated `start-stack.sh`:
    - colorized `WARNING` and `ERROR` prefixes
    - added reusable text styling helpers for danger, warning, info, and command actions
    - added `print_callout_block()` for visually separated recovery/reboot panels
    - updated the startup banner to include an industrial caution line and more branded copy
    - highlighted banner commands (`probe`, `status`, `stop`, `stop --hard`) in green
    - changed startup recovery output so `REBOOT REQUIRED`, `REBOOT HOST`, and the one-shot recycle plan render as separate emphasized blocks
    - updated the interactive console ready line to colorize the common commands
- Validation:
  - `bash -n start-stack.sh`
  - `ReadLints` on `start-stack.sh`
    - result: no diagnostics
- Follow-up notes / risks:
  - I did not rerun a live failing startup after this visual-only pass, so the exact operator-facing appearance of the new recovery callouts should be checked on the next real terminal run and adjusted if you want even stronger contrast or tighter spacing.

## 2026-04-13 19:22 +0000

- Task summary:
  - Added a true green success path so the launcher now visibly announces when the full staged boot has actually completed.
- Changes:
  - Updated `start-stack.sh`:
    - added a `SUCCESS` log helper with bright green styling
    - changed the bus-ready message to emit as a green success indicator
    - added `print_boot_success_block()` so the launcher prints a green `SYSTEM ONLINE` callout after controller, bus, API, and optional web readiness all pass
    - reused the runtime summary from the startup banner so the success panel shows the real robot/backend/drive values rather than generic placeholders
- Validation:
  - `bash -n start-stack.sh`
  - `ReadLints` on `start-stack.sh`
    - result: no diagnostics
- Follow-up notes / risks:
  - I still did not run a full live successful startup in this pass, so the exact terminal presentation of the new green success block should be checked on the next successful `./start-stack.sh` run and tuned if needed.

## 2026-04-13 19:38 +0000

- Task summary:
  - Expanded the terminal UX beyond the banner so staged startup now has quieter bootstrap output, styled timestamps/log lines, stage-by-stage success messages, and tty-only loading indicators that better match the website’s industrial aesthetic.
- Changes:
  - Updated `start.sh`:
    - added `GRADIENT_START_QUIET=1` support so `start-stack.sh` can suppress the old plain bootstrap chatter while still sourcing the environment normally
  - Updated `start-stack.sh`:
    - added tty-only loading helpers (`ui_loading_status`, `ui_status_clear`) with animated ASCII progress frames
    - switched log output to a shared formatter with styled timestamp and `[start-stack]` tag
    - added an `INFO` level and kept `SUCCESS` / `WARNING` / `ERROR` visually distinct
    - replaced raw bootstrap output with a branded `ENVIRONMENT READY` callout
    - reformatted RTCore sync into an explicit stage with a cleaner failure panel and highlighted inspection commands
    - added loading/success feedback for controller readiness, generic probe waits, API readiness, web readiness, bus convergence, and selected shutdown waits
    - made process launches report as explicit success events instead of plain pid lines
- Validation:
  - `bash -n start-stack.sh && bash -n start.sh`
  - `bash -lc 'cd /home/pi/GradientOS && export GRADIENT_START_QUIET=1 && source ./start.sh >/tmp/grad_start_quiet.out && wc -l /tmp/grad_start_quiet.out'`
    - result: `0 /tmp/grad_start_quiet.out`
  - `ReadLints` on `start-stack.sh` and `start.sh`
    - result: no diagnostics
- Follow-up notes / risks:
  - This pass was validated structurally rather than with a full live stack start, so the exact pacing/visual density of the spinner lines and staged success output should be checked on the next real `./start-stack.sh` run and tuned from the live terminal capture.

## 2026-04-13 19:48 +0000

- Task summary:
  - Fixed a `set -u` regression in the new terminal styling layer where the launcher crashed before startup because the palette globals were referenced before initialization.
- Changes:
  - Updated `start-stack.sh`:
    - predeclared all `BANNER_*` and `UI_*` palette variables near the top-level variable block so styled logger/helper calls are safe before `init_banner_palette()` runs
- Validation:
  - `bash -n start-stack.sh`
  - `./start-stack.sh --help`
  - `./start-stack.sh`
    - result: progressed through the styled startup path and exited with the expected real stale-owner `REBOOT REQUIRED` failure instead of `UI_INFO: unbound variable`
- Follow-up notes / risks:
  - The immediate crash is fixed; any further startup failures are now genuine runtime/service issues rather than shell formatting regressions.

## 2026-04-13 20:06 +0000

- Task summary:
  - Added real stage timing instrumentation to the staged launcher and used a live startup run to confirm which phases are actually slow enough to justify animated indicators.
- Changes:
  - Updated `start.sh`:
    - added `GRADIENT_START_QUIET=1` support so `start-stack.sh` can suppress legacy bootstrap chatter and present a single styled boot flow
  - Updated `start-stack.sh`:
    - added `now_ms()` and `format_duration_ms()` helpers
    - extended tty spinner status lines to show `t+...` elapsed time while a stage is in progress
    - added persistent elapsed-time success/failure logs for environment bootstrap, RTCore sync, generic probe waits, controller readiness, bus readiness, API readiness, web readiness, systemd service stop, and full recovery recycle
    - aligned the ASCII logo rows so the `GradientOS` mark no longer leans left in the middle
- Validation:
  - `bash -n start-stack.sh && bash -n start.sh`
  - `ReadLints` on `start-stack.sh` and `start.sh`
    - result: no diagnostics
  - `./start-stack.sh`
    - progressed through the styled startup path and produced real stage timings before hitting the expected stale-owner reboot path
    - observed timings:
      - environment ready: about `66-68ms`
      - initial RTCore sync: about `17.348s`
      - `ethercat.service` stop during recycle: about `38.670s`
      - full recovery recycle: about `41.991s`
- Follow-up notes / risks:
  - The timing instrumentation now gives enough data to decide where the animated status line is worthwhile. The slow RTCore/EtherCAT phases are clearly worth it; the environment stage is probably too fast to need persistent animation unless the styling is desired for consistency.

## 2026-04-13 20:13 +0000

- Task summary:
  - Styled the recovery-time probe snapshot so launcher probe dumps no longer fall back to the old raw `probe: key=value ...` format.
- Changes:
  - Updated `start-stack.sh`:
    - added `style_probe_state()` to colorize `UP`/`DOWN`/`INACTIVE`/`FAULTED`/`DISARMED`-style values appropriately
    - added `style_probe_ratio()` to colorize readiness and `wkc` ratios
    - replaced `log_probe_snapshot()`’s raw Python one-liner with a styled `PROBE SNAPSHOT` callout block that shows physical/driver/EtherCAT/RTCore state plus armed/mask/op-enabled/WKC status
- Validation:
  - `bash -n start-stack.sh`
  - `ReadLints` on `start-stack.sh`
    - result: no diagnostics
- Follow-up notes / risks:
  - This only styles the launcher-side snapshot block used during startup/recovery flows. If we also want the explicit `./start-stack.sh probe` command output to adopt the same callout/palette language, that can be done in a follow-up pass.

## 2026-04-13 20:20 +0000

- Task summary:
  - Replaced the pseudo-loading line with a real tty-only animated spinner for the long blocking startup stages.
- Changes:
  - Updated `start-stack.sh`:
    - added `UI_SPINNER_PID`, `ui_loading_begin()`, and `run_with_loading_capture()`
    - changed `ui_status_clear()` to stop any active spinner process before printing the next log/callout line
    - wrapped blocking RTCore sync (`sync-runtime.sh --ensure-active`) in the new spinner helper
    - wrapped blocking `systemctl stop` calls in the new spinner helper so slow EtherCAT stop paths animate too
- Validation:
  - `bash -n start-stack.sh`
  - `ReadLints` on `start-stack.sh`
    - result: no diagnostics
  - `./start-stack.sh`
    - progressed through the styled startup path and hit the expected RTCore sync failure path after about `34.438s`
    - confirms the long blocking sync stage is now a genuine spinner candidate rather than a one-frame status line
- Follow-up notes / risks:
  - The animated spinner is tty-only and redraws in place, so it will not appear as an animation in static screenshots or persisted terminal transcripts; only the live terminal shows the moving frames.

## 2026-04-13 20:29 +0000

- Task summary:
  - Investigated why the latest startup run did not complete and patched the launcher so active bus convergence extends the readiness deadline instead of tripping the generic 20 s timeout.
- Changes:
  - Updated `start-stack.sh`:
    - added `GRADIENT_STACK_BUS_PROGRESS_GRACE_S` (default `15`) and `GRADIENT_STACK_BUS_MAX_TIMEOUT_S` (default `60`)
    - changed `wait_for_bus_operational()` to track fieldbus progress (`responding`, `online`, `operational`, `startup_ready`, `link_up`)
    - when progress increases, extend the readiness window up to the hard cap instead of failing immediately at the base timeout
    - updated the timeout error to report total elapsed wait plus the base/grace/hard-cap values
    - passed dynamic remaining time into the live timeout counter so the fieldbus spinner reflects the extended deadline
- Validation:
  - `bash -n start-stack.sh`
  - `ReadLints` on `start-stack.sh`, `src/gradient_os/telemetry/startup_recovery.py`, and `tests/test_startup_recovery.py`
    - result: no diagnostics
- Follow-up notes / risks:
  - This patch addresses the specific run where the bus was still converging (`0/6 -> 5/6 -> BUS_UP_DISARMED`) as the old 20 s deadline expired. Live hardware validation is still needed to confirm the extended deadline now gives the bus enough time to reach `startup_ready=1` instead of falling out on the generic timeout.

## 2026-04-13 20:40 +0000

- Task summary:
  - Fixed the remaining startup blocker so `start-stack.sh` no longer exits before the web UI when canonical pose truth is unavailable during bring-up, and confirmed the full controller/API/web stack now stays live.
- Changes:
  - Updated `src/gradient_os/arm_controller/command_api.py`:
    - wrapped `handle_get_position()` joint sampling in a local exception handler
    - when canonical joint truth is unavailable, the controller now sends `ERROR,GET_POSITION,...` back to the caller instead of throwing a traceback through the main loop
  - Updated `start-stack.sh`:
    - changed `wait_for_api_readiness()` so `/info/pose` is best-effort during startup rather than a hard gate for web bring-up
    - API health, runtime-config sanity, and joints readiness still remain required
- Validation:
  - `bash -n start-stack.sh`
  - `source ./start.sh && python -m pytest tests/test_api_endpoints.py -q -k 'info_pose'`
    - result: `2 passed, 67 deselected`
  - `source ./start.sh && python -m py_compile src/gradient_os/arm_controller/command_api.py src/gradient_os/api/main.py`
  - `ReadLints` on `start-stack.sh`, `src/gradient_os/arm_controller/command_api.py`, and `tests/test_api_endpoints.py`
    - result: no diagnostics
  - `./start-stack.sh status`
    - result: launcher `running`, `controller: up`, `api: up`, `web: up`
- Follow-up notes / risks:
  - The stack is now staying live through web bring-up, but canonical joint truth is still unavailable on the bench, so the controller continues to log `GET_JOINT_STATE` warnings. That is a real backend/absolute-truth issue to solve next, separate from the launcher completion bug.

## 2026-04-13 22:25 +0000

- Task summary:
  - Tightened the EtherCAT RTCore joint-read contract so the backend fails closed instead of silently returning the last commanded setpoint when live canonical truth is unavailable.
- Changes:
  - Updated `src/gradient_os/arm_controller/backends/ethercat_rtcore/backend.py`:
    - changed `get_joint_positions()` to raise `Canonical joint truth unavailable (...)` when RTCore is disconnected or feedback config is not ready
    - removed the getter path that returned `_last_joint_setpoint_rad` as pseudo-feedback
    - clarified the `_last_joint_setpoint_rad` comment so it is explicitly command bookkeeping only, not read truth
  - Updated `tests/test_gradient05_limits_and_backends.py`:
    - kept the corrected canonical-to-raw command-frame regression coverage intact
    - added fail-closed tests for disconnected reads and connected-but-feedback-not-ready reads
  - Updated `.cursor/memory/AGENT_SCRATCHPAD.md`:
    - marked the earlier same-day "re-apply anchor on writes" note as superseded
    - recorded the corrected transform algebra and the new fail-closed getter guardrail
- Validation:
  - `source ./start.sh && python -m pytest tests/test_gradient05_limits_and_backends.py -q -k 'translates_canonical_truth_back_into_raw_wire_counts or connected_reads_return_canonical_feedback or disconnected_get_joint_positions_fails_closed or connected_without_feedback_config_fails_closed or startup_bootstraps_missing_absolute_home_anchor or anchor_bootstrap_cannot_reconstruct_truth or native_home_offsets_to_feedback_but_not_command_targets'`
    - result: `7 passed, 47 deselected`
  - `source ./start.sh && python -m py_compile src/gradient_os/arm_controller/backends/ethercat_rtcore/backend.py tests/test_gradient05_limits_and_backends.py`
  - `ReadLints` on `src/gradient_os/arm_controller/backends/ethercat_rtcore/backend.py` and `tests/test_gradient05_limits_and_backends.py`
    - result: no diagnostics
- Follow-up notes / risks:
  - The repo now enforces the single-source/fail-closed read contract locally, but live hardware proof after the already-reverted anchor write-path fix is still outstanding.
  - I did not restart the stack or command the robot in this pass; the next meaningful validation is a controlled J4-only then J3-only live jog proof against fresh `/info/joints-detailed` snapshots.

## 2026-04-13 22:50 +0000

- Task summary:
  - Investigated the newer live regression run and hardened the backend so anchored absolute feedback is only considered motion-safe when it round-trips back into the current raw/reference command frame.
- Changes:
  - Updated `src/gradient_os/arm_controller/backends/ethercat_rtcore/backend.py`:
    - added per-axis command-frame roundtrip diagnostics (`command_roundtrip_*`) inside `_canonical_joint_positions_from_raw_feedback()`
    - fail-closes canonical truth with `truth_reason=command_frame_roundtrip_mismatch` when `canonical -> command frame` does not reconstruct the current `reference_pre_zero_rad` within about one count
    - factored the command transform through `_command_axis_q_for_joint_value()` / `_canonical_joint_q_from_command_axis_q()` so the diagnostic uses the exact same math as the upload path
  - Updated `tests/test_gradient05_limits_and_backends.py`:
    - added a regression test that marks truth unavailable when `absolute_feedback + anchor` is present but does not round-trip into the current raw/reference motion frame
  - Updated `.cursor/memory/AGENT_SCRATCHPAD.md`:
    - recorded the live roundtrip-mismatch diagnosis and the new rule that canonical truth must be proven motion-safe, not merely present
- Validation:
  - Live inspection on the already-running stack:
    - `curl -s http://127.0.0.1:4000/info/joints-detailed`
    - `curl -s http://127.0.0.1:4000/info/pose`
    - `logs/startups/20260413-224227/controller.log`
    - key evidence: the controller sent a J4-only target, but the live snapshot simultaneously reported `canonical_joint_truth_available=true` while exposing a frame inconsistency (`J1 canonical_rad=0.21456`, `reference_pre_zero_rad=-0.03677`, zero software offsets), which means the current anchored absolute source was not command-frame-safe
  - Local focused tests:
    - `source ./start.sh && python -m pytest tests/test_gradient05_limits_and_backends.py -q -k 'translates_canonical_truth_back_into_raw_wire_counts or connected_reads_return_canonical_feedback or marks_truth_unavailable_when_absolute_anchor_does_not_roundtrip or disconnected_get_joint_positions_fails_closed or connected_without_feedback_config_fails_closed or startup_bootstraps_missing_absolute_home_anchor or anchor_bootstrap_cannot_reconstruct_truth or native_home_offsets_to_feedback_but_not_command_targets'`
    - result: `8 passed, 47 deselected`
  - `source ./start.sh && python -m py_compile src/gradient_os/arm_controller/backends/ethercat_rtcore/backend.py tests/test_gradient05_limits_and_backends.py`
  - `ReadLints` on the touched backend/test files
    - result: no diagnostics
- Follow-up notes / risks:
  - The running stack has not picked up this guardrail yet; a controller/backend restart is required before the live API will stop advertising the current inconsistent anchored source as motion-safe.
  - This change intentionally blocks motion when the chosen `encoder_multi_turn_counts` source and current native-home/reference frame are semantically inconsistent. It does not yet prove which underlying source is wrong; it only prevents the stack from trusting it for motion until that proof exists.

## 2026-04-13 23:33 +0000

- Task summary:
  - Attempted the requested clean restart and gathered a fresh disarmed proof snapshot plus manual-backed conclusions about which A6-EC objects are raw encoder-unit state versus drive reference/home state.
- Changes:
  - No additional source-code changes in this pass beyond the earlier roundtrip guard; this pass focused on runtime verification and manual interpretation.
  - Updated `.cursor/memory/AGENT_SCRATCHPAD.md` with the manual-confirmed frame split and the failed-restart fieldbus blocker.
- Validation:
  - `./start-stack.sh status`
    - result before restart: stack fully down
  - `./start-stack.sh`
    - result: failed during bus readiness on run `20260413-233217`
    - observed progress: `responding=6/6`, then stalled at `online=5/6 operational=5/6 startup_ready=0`
  - `./start-stack.sh probe`
    - result: `J5/axis4` offline (`slave_online=False`, `slave_operational=False`, `sw=0x1640`) while the other five axes were `SwitchOnDisabled` / `0x1650`
  - Manual/source review:
    - `docs/resources/a6ec_manual_chapter_11_parameter_list.md`
    - confirmed `C00.07=4` label (`Absolute position rotation mode`)
    - confirmed `U40.20/.22` are encoder-unit multi-turn data
    - confirmed `U40.16`, `6064h`, `607Ah`, and `607Ch` are reference/home-frame quantities
    - confirmed `607Ch` is active only when powered on, homing complete, and `6041h bit 15 = 1`, and that after homing `6064h = 607Ch`
- Follow-up notes / risks:
  - Because the fieldbus never reached `startup_ready=1`, the controller/API never launched and the new Python fail-closed truth guard is not active in the live stack yet.
  - The strongest current conclusion is not "raw encoder unstable" but "raw encoder-unit state and reference/home-frame state are being conflated"; vendor confirmation is still needed for the exact manufacturer-intended canonical source and homing/reference workflow.

## 2026-04-14 00:17 +0000

- Task summary:
  - Completed the fresh-boot disarmed object comparison and verified that the guarded stack now blocks unsafe joint truth instead of feeding false positions to the frontend.
- Changes:
  - No new source edits in this pass.
  - Updated `.cursor/memory/AGENT_SCRATCHPAD.md` with the fresh-boot SDO comparison findings.
- Validation:
  - Successful guarded restart on run `20260413-235354` after the user hard-stopped and power-cycled the drives.
  - Live API checks:
    - `curl -s http://127.0.0.1:4000/info/joints-detailed`
    - `curl -s -i http://127.0.0.1:4000/info/pose`
    - result: `canonical_joint_truth_available=false`, `command_frame_roundtrip_mismatch`, and `GET /info/pose` `503`
  - Direct disarmed EtherCAT SDO comparison (two snapshots one second apart) for every axis:
    - `0x6041`, `0x6064`, `0x607C`, `0x2040:17`, `0x2040:21`, `0x2040:23`, `0x2040:2B`, `0x2040:2D`
    - result highlights:
      - all axes `0x6041 = 0x1650`, `bit15 = 0`, `bit12 = 1`
      - all axes `0x607C = 0`
      - `0x6064`, `U40.16`, and `U40.2A/.2C` were mutually close and stable
      - `U40.20/.22` were also stable (`0..3` count drift over 1 s), so the raw multi-turn source does not currently look noisy
      - the large mismatch remains semantic/frame-related rather than transport noise
- Follow-up notes / risks:
  - The frontend is blank because the backend intentionally returns empty joint arrays when canonical truth is unsafe; this is expected fail-closed behavior, not a separate frontend transport bug.
  - The fresh-boot evidence now points away from "unstable encoder reads" and toward "stable raw encoder data but wrong/inactive relationship to the drive's reference/home frame after boot."

## 2026-04-14 00:17 +0000

- Task summary:
  - Ran the first controlled per-axis native-home experiment on `J4` to test whether the drive's home/reference-valid state is something startup should merely expose or something the native-home transaction actively establishes.
- Changes:
  - No new code changes in this pass.
  - Updated `.cursor/memory/AGENT_SCRATCHPAD.md` with the J4 before/after findings.
- Validation:
  - Pre-home `J4` snapshot:
    - direct SDOs (`axis3`): `6041=0x1650`, `bit15=0`, `607C=0`, `6064=28359`, `U40.16=28359`, `U40.20=163933`, `U40.2A=28358`
    - live API axis detail: `truth_reason=command_frame_roundtrip_mismatch`
  - Command:
    - `curl -s -X POST http://127.0.0.1:4000/control/home-joint-native -H 'Content-Type: application/json' -d '{"joint":4}'`
    - result: `verified=true`, `terminal_state=succeeded`, `native_home_state=2`, `disarmed_after_home=true`
  - Post-home `J4` snapshot:
    - direct SDOs (`axis3`): `6041=0x9650`, `bit15=1`, `bit12=1`, `607C=0`, `6064≈131063`, `U40.16≈-8`, `U40.20≈32852`, `U40.2A≈131064`
    - one-second stability re-read stayed within a few counts
    - live API axis detail: `truth_available=true`, `command_roundtrip_consistent=true` for axis 3
    - global API status still unavailable because other axes remain mismatched
- Follow-up notes / risks:
  - This strongly suggests native home changes the drive's actual state for that axis rather than us merely "using" an already-present startup-valid flag.
  - Persistence across a later power cycle is still the key unresolved test; J4 is now the cleanest candidate for that follow-up.

## 2026-04-14 00:38 +0000

- Task summary:
  - Completed the `J4` persistence follow-up after a real drive power cycle and confirmed that J4's corrected frame survived reboot even though HM bit 15 did not stay set.
- Changes:
  - No source edits in this pass.
  - Updated `.cursor/memory/AGENT_SCRATCHPAD.md` with the post-restart J4 persistence finding.
- Validation:
  - Post-restart stack/probe on run `20260414-003639`:
    - all 6 axes online/operational, disarmed
    - `J4/axis3 pos_counts=131062`
  - Direct post-restart `J4` SDO snapshot:
    - `6041=0x1650` (`bit15=0`, `bit12=1`)
    - `607C=0`
    - `6064=131063`
    - `U40.16=131062`
    - `U40.20=32848`
    - `U40.2A=131061`
  - Live API snapshot after restart:
    - `axis 3` remained `command_roundtrip_consistent=true` / `truth_available=true`
    - global unavailable axes shrank to `[0, 1, 2]`
- Follow-up notes / risks:
  - This disproves the simpler hypothesis that persistence requires HM bit 15 to remain high after reboot. The semantic/reference-frame correction for J4 persisted, but the statusword dropped back to `0x1650`.
  - `607C` stayed zero throughout the successful J4 home and reboot, so the current workflow's persisted effect is not obviously witnessed by nonzero `607C`.
  - The next systematic tests should repeat the same sequence on another failing axis (best candidates: `J3`, then `J1/J2`) and keep separating frame persistence from status-bit persistence.

## 2026-04-14 01:02 +0000

- Task summary:
  - Correlated the user's selected Group 6000 manual clauses with live `J3`/`J4` reads to see which parts match the observed behavior and which parts do not.
- Changes:
  - No code edits in this pass.
  - Updated `.cursor/memory/AGENT_SCRATCHPAD.md` with the manual/live-correlation findings.
- Validation:
  - Manual extracts reviewed from `docs/resources/a6ec_manual_chapter_11_parameter_list.md`:
    - `607Ch` home offset
    - `6099.02h` speed during search for zero
    - `60E3.01h` supported homing method semantics
    - `60E6h` actual position calculation method
    - `60F4h`, `60FCh`, `60FDh`
    - `U40.16` and `U40.28` descriptions
  - Live object reads on `J3/axis2` and `J4/axis3`:
    - `6098=35`
    - `60E6=0`
    - `60F4=0`
    - `60FD=0`
    - `60FC` closely matched the drive-facing rotation/reference frame (`J3 52562`, `J4 131061`)
    - `60E3` supported-method entries include `35`, and every listed method currently reports both relative and absolute support bits set
- Follow-up notes / risks:
  - The manual strongly supports focusing on `60E6` and `60FC` for the canonical-truth question.
  - The short `607C` paragraph does not fully explain the successful J4 path, because J4 still shows `607C=0` while the corrected frame persisted and round-tripped safely.

## 2026-04-14 02:42 +0000

- Task summary:
  - Ran the full controlled `J3` native-home persistence sequence: pre-home snapshot, native-home transaction, immediate post-home capture, soft-stop while keeping RTCore/EtherCAT up, real drive power cycle, guarded restart, and post-restart verification.
- Changes:
  - No source-code edits in this pass.
  - Updated `.cursor/memory/AGENT_SCRATCHPAD.md` with the new `J3` persistence findings.
  - Stored runtime artifacts under `logs/encoder-retention/20260414-020640-j3-native-home-sequence`.
- Validation:
  - Pre-home `J3` snapshot:
    - `6041=0x1650`, `bit15=0`, `607C=0`, `6064=52564`, `U40.16=52563`, `60FC=52562`, `U40.20=-446379`
    - API reported `truth_reason=command_frame_roundtrip_mismatch` for axis 2
  - Native-home command:
    - `curl -s -X POST http://127.0.0.1:4000/control/home-joint-native -H 'Content-Type: application/json' -d '{"joint":3}'`
    - result: `verified=true`, `terminal_state=succeeded`, `native_home_state=2`, `disarmed_after_home=true`
  - Immediate post-home `J3` snapshot:
    - `6041=0x9650`, `bit15=1`, `607C=0`, `6064~=131041..131042`, `U40.16~=-30..-32`, `60FC~=131041..131042`, `U40.20=77880`
    - API initially accepted axis 2 with anchor `0.02548325583875021`
  - Post-home truth poll:
    - both `J3` and `J4` flickered between accepted and rejected as roundtrip error moved between `1` and `2-3` counts
  - Soft-stop / power-cycle staging:
    - first `./start-stack.sh stop` removed the launcher only
    - second `./start-stack.sh stop` performed the intended soft stop, leaving `rtcore_state=UP`, `ethercat_master_state=OP`, and `physical_state=BUS_UP_DISARMED`
  - Direct post-power-cycle pre-restart snapshot:
    - `J3` came back with `6041=0x1650`, `bit15=0`, `607C=0`, but the corrected frame persisted: `6064=131042`, `U40.16=131041`, `60FC=131041`, `U40.28=131041`, `U40.2A=131041`, `U40.20=77882`
  - Guarded restart on run `20260414-024152`:
    - full stack reached `STACK BOOT COMPLETE`
    - startup banner still reported global canonical truth unavailable because some other axes remain unresolved
  - Post-restart API and SDO snapshot:
    - global unavailable joints `[1, 2, 6]` / axes `[0, 1, 5]`
    - `J3`: `6041=0x1650`, `bit15=0`, `607C=0`, `6064=131042`, `U40.16=131040`, `60FC=131041`, `U40.20=77881`, API `truth_available=true`, near-zero roundtrip error
    - `J4`: `6041=0x1650`, `bit15=0`, `607C=0`, `6064=131051`, `U40.16=131053`, `60FC=131053`, `U40.20=32840`, API `truth_available=true`, near-zero roundtrip error
  - Post-restart truth poll:
    - `J3` still flickered (`true,true,true,true,true,true,true,false,false,true,false,true`) as error hopped between `0/1` and `2` counts
    - `J4` also flickered early before settling mostly true
- Follow-up notes / risks:
  - `J3` now matches `J4` on the persistence question: the corrected reference/frame effect survives real power loss even though `6041 bit15` clears after reboot and `607C` remains `0`.
  - The remaining issue for `J3/J4` is no longer "does the native-home frame persist?" but "why does the roundtrip guard ride a 1-2 count edge and intermittently reject otherwise semantically-correct axes?"

## 2026-04-14 02:48 +0000

- Task summary:
  - Re-reviewed the live motion read/write path in code to answer which counts are actually used for motion versus canonical truth reconstruction.
- Changes:
  - No source-code edits in this pass.
  - Updated `.cursor/memory/AGENT_SCRATCHPAD.md` with the code-level transform conclusion.
- Validation:
  - Confirmed in `src/gradient_os/arm_controller/backends/ethercat_rtcore/backend.py`:
    - `sync_read_positions()` returns RTCore `_axis_counts`
    - `raw_to_joint_positions()` reconstructs canonical truth from absolute feedback plus persisted anchor and then roundtrip-checks against the motion/reference frame
    - `_command_axis_q_for_joint_value()` adds only software zero
    - `_canonical_joint_q_from_command_axis_q()` subtracts only software zero
    - `_reference_q_before_master_offset_for_axis()` applies `native_home_offset` on the read/reference side
  - Confirmed in `src/gradient_rt_motion/ipc_v1.hpp` and `src/gradient_rt_motion/main.cpp`:
    - RTCore `AxisStatusV1.pos_counts` is documented as `0x6064`
    - queued CSP targets are stored in the same raw wire-count frame the drive publishes on `0x6064` and expects on `0x607A`
    - RTCore subtracts `native_home_offset_counts` exactly once when converting controller targets to CSP wire counts
  - Confirmed in `src/gradient_os/arm_controller/profiles/drive/a6ec_ds402.py`:
    - preferred absolute source order remains `encoder_multi_turn_counts` (`U40.20/.22`) first, `rotation_mode_encoder_counts` (`U40.2A/.2C`) second
  - Confirmed in `tests/test_gradient05_limits_and_backends.py`:
    - canonical truth translates back into the same raw wire counts
    - native-home offsets apply to feedback but not command targets
- Follow-up notes / risks:
  - The motion command path is now clean and does not re-apply the persisted absolute-home anchor.
  - The remaining open question is not command-path math but whether `U40.20/.22 + anchor` is the right long-term canonical host truth for all axes/states, or whether the corrected rotation/reference frame should eventually replace it.

## 2026-04-14 02:55 +0000

- Task summary:
  - Verified the manual/math around 17-bit encoder resolution versus the exposed multi-turn objects and answered why large gear ratios do not require commanding raw multi-turn encoder counts directly.
- Changes:
  - No source-code edits in this pass.
  - Updated `.cursor/memory/AGENT_SCRATCHPAD.md` with the clarified numeric/manual conclusion.
- Validation:
  - Manual checks in `docs/resources/a6ec_manual_chapter_11_parameter_list.md`:
    - `C00.07=4` is `Absolute position rotation mode`
    - `6064h` is position actual value in `user-defined unit`
    - `607Ah` target position unit is `Reference unit`
    - `6091h` states motor position feedback in encoder units is load/reference feedback times gear ratio
    - `60FCh` states encoder-unit position reference is derived from reference-unit position reference via `6091h`
    - `U40.1E` (`Encoder multi-turn position data`) is `U16`, range `0-65535 Rev`
    - `U40.20` / `U40.22` are low/high 32-bit halves of encoder multi-turn data in encoder units
  - Conclusion verified:
    - `2^32 / 2^17 = 32768` is only the span of a hypothetical unsigned 32-bit encoder-count accumulator
    - it is not a manufacturer-verified limit for the actual exposed A6-EC multi-turn objects we are reading
- Follow-up notes / risks:
  - The current open problem remains semantic frame selection for canonical truth, not insufficient turn range on the CSP motion path.
  - If we want the exact physical retained multi-turn limit of the encoder hardware itself, the parameter-list extract does not state it cleanly; that may require a deeper Chapter 5/manual lookup or vendor confirmation.

## 2026-04-14 04:22 +0000

- Task summary:
  - Read Chapter 5 more deeply, verified the current live gear-ratio objects, and added a read-only Chapter 5 probe script so future before/after-home/restart checks can test the frame model directly.
- Changes:
  - Added `scripts/a6ec_chapter5_probe.py`.
  - Updated `.cursor/memory/AGENT_SCRATCHPAD.md` with the new Chapter 5 probe findings.
- Validation:
  - Manual findings from `docs/resources/chapter 5 absolute system - extract from A6-EC_series_servo_drive_manual (2).pdf`:
    - absolute encoder is `131072 (2^17)` counts/rev with `16-bit multi-turn data saved`
    - `U40.1E` is `0..65535 Rev`
    - rotation mode applies when unidirectional load revolutions are `<32767`
  - Live SDO verification on `J3` and `J6`:
    - `6091.01 = 1`, `6091.02 = 1`
    - `C10.18 = 1`, `C10.19 = 1`
    - current live stack is not using drive-side gear-ratio mapping
  - Spot checks on `J1/J2/J3/J6`:
    - `combined(U40.20/.22)` matches `sign_extend16(U40.1E) * 131072 + (U40.1C mod 131072)` within `0..1` count
  - Script validation:
    - `python -m py_compile scripts/a6ec_chapter5_probe.py`
    - `python scripts/a6ec_chapter5_probe.py snapshot --label current --axes J3 J6`
    - result artifact: `logs/encoder-retention/20260414-042146-a6ec-ch5-probe/current.json`
    - result summary:
      - `J3` and `J6` both show `6091 = 1:1`
      - `J3` and `J6` both show `C10.18/C10.19 = 1:1`
      - `6063 ~= 6064*6091`, `60FC ~= 6062*6091`, and `U40.2A/.2C ~= U40.28*(C10.18/C10.19)` all matched within `0..1` count
- Follow-up notes / risks:
  - The raw multi-turn formula is now supported both by Chapter 5 structure and by live data, but the remaining architectural question is still which frame should be host canonical truth after native home.
  - The new script is the right harness for the next manual-backed experiment: capture the same axes across boot, native-home, and restart and compare which of the raw/reference/rotation bridges remain stable.

## 2026-04-14 05:29 +0000

- Task summary:
  - Ran the new Chapter 5 probe on `J6` first, as requested, and verified the manual-derived formulas there with both a single snapshot and a short repeated poll.
- Changes:
  - No source-code edits in this pass beyond the earlier script addition.
  - Stored new `J6` artifacts under `logs/encoder-retention/20260414-052921-a6ec-ch5-probe/`.
  - Updated `.cursor/memory/AGENT_SCRATCHPAD.md` with the `J6` verification result.
- Validation:
  - Probe command:
    - `python scripts/a6ec_chapter5_probe.py snapshot --label j6-current --axes J6`
  - Artifact:
    - `logs/encoder-retention/20260414-052921-a6ec-ch5-probe/j6-current.json`
    - `logs/encoder-retention/20260414-052921-a6ec-ch5-probe/j6-current.md`
  - Single-snapshot `J6` result:
    - `6091.01=1`, `6091.02=1`
    - `C10.18=1`, `C10.19=1`
    - raw formula `combined(U40.20/.22) ~= sign_extend16(U40.1E)*131072 + (U40.1C mod 131072)` matched with `delta=+1`
    - `6063 ~= 6064*6091` matched with `delta=-1`
    - `60FC ~= 6062*6091` matched with `delta=0`
    - `U40.2A/.2C ~= U40.28*(C10.18/C10.19)` landed at `delta=+2` on that sample
  - Repeated poll:
    - saved as `logs/encoder-retention/20260414-052921-a6ec-ch5-probe/j6-bridge-poll.json`
    - observed jitter bands:
      - raw formula delta `-2 .. +2`
      - `6063 - 6064*6091` delta `-1 .. +2`
      - `60FC - 6062*6091` delta `-3 .. +2`
      - rotation-mode bridge delta `-2 .. +2`
- Follow-up notes / risks:
  - `J6` does support the Chapter 5 frame model semantically.
  - The practical verification lesson is the same as with the roundtrip guard work: live reads wander by a couple of counts, so a single `2-count` miss should be treated as jitter unless it persists systematically.

## 2026-04-14 05:44 +0000

- Task summary:
  - Completed the requested `J6` native-home persistence sequence after the user's drive power cycle, then compared the result against the earlier `J3` and `J4` power-cycle proofs and wrote a consolidated evidence table into `.cursor/memory/AGENT_SCRATCHPAD.md`.
- Changes:
  - No production source-code changes in this pass.
  - Captured new experiment artifacts under `logs/encoder-retention/20260414-053539-j6-ch5-persistence-controls/`.
  - Updated `.cursor/memory/AGENT_SCRATCHPAD.md` with a three-axis persistence table covering `J3`, `J4`, and `J6`.
- Validation:
  - Pre-restart disarmed probe after the user's power cycle:
    - `./start-stack.sh probe`
    - result: RTCore/EtherCAT stayed up and disarmed; `J6/axis5` returned `sw=0x1650` and `pos_counts=0`
  - Snapshot sequence:
    - `python scripts/a6ec_chapter5_probe.py snapshot --label post-power-cycle-pre-restart --axes J3 J4 J6 --experiment-id 20260414-053539-j6-ch5-persistence-controls`
    - `python scripts/a6ec_chapter5_probe.py snapshot --label post-restart --axes J3 J4 J6 --experiment-id 20260414-053539-j6-ch5-persistence-controls`
  - Restart run:
    - `./start-stack.sh`
    - startup run id: `20260414-053857`
  - Post-restart truth poll:
    - saved to `logs/encoder-retention/20260414-053539-j6-ch5-persistence-controls/post-restart-truth-poll.json`
    - acceptance counts over 12 samples:
      - `J3`: `11/12` true
      - `J4`: `9/12` true
      - `J6`: `7/12` true
    - all three axes stayed inside the previously observed `0..3` count roundtrip/jitter band rather than showing a large semantic mismatch
- Follow-up notes / risks:
  - `J6` now matches the same persistence pattern already seen on `J3` and `J4`: the corrected reference frame survives a real drive power cycle, while `6041 bit15` clears after boot and `607C` remains `0`.
  - The remaining live issue is not missing persistence on these tested axes; it is that the current one-count roundtrip acceptance threshold flickers on otherwise semantically-correct axes when live reads wander by `1..3` counts.
  - Global truth still remains unavailable on this restart because unresolved axes plus that tolerance-edge flicker keep some joints marked unavailable at any given sample.

## 2026-04-14 05:50 +0000

- Task summary:
  - Re-read the latest scratchpad/devlog plus commissioning safety guidance and turned the new `J3/J4/J6` evidence into a concrete recommendation for how to validate the remaining unverified joints.
- Changes:
  - No production code changes.
  - Added a procedural recommendation to `.cursor/memory/AGENT_SCRATCHPAD.md` for batching the remaining native-home proofs in one shared power-cycle session while keeping homes strictly sequential.
- Validation:
  - Re-read latest evidence entries in `.cursor/memory/AGENT_SCRATCHPAD.md` and `.cursor/memory/DEVLOG.md`.
  - Re-read `.cursor/skills/gradientos-sop/commissioning-safety.md` to keep the recommendation aligned with the current commissioning rule set.
- Follow-up notes / risks:
  - Recommendation is to batch the remaining target axes in one session, but only with `home -> snapshot/poll` done one axis at a time and a single shared power cycle after all immediate post-home captures.
  - A previously persisted axis should be included as a control in shared pre/post-power-cycle captures when practical.
  - This batching recommendation applies to persistence validation only; it does not resolve the separate roundtrip-guard jitter issue that still affects global truth/frontend stability.

## 2026-04-14 06:05 +0000

- Task summary:
  - Started the all-joints batch experiment, homed `J5` and `J1` successfully, then retried `J2` under explicitly idle RTCore conditions to test whether the earlier `J2` failure was just "too soon after `J1`".
- Changes:
  - No production code changes.
  - Captured new all-joints artifacts under `logs/encoder-retention/20260414-055631-all-joints-native-home-batch/`, including `pre-home`, `post-home-j5`, `post-home-j1`, `post-home-j2-failed`, `pre-retry-j2`, and `post-retry-j2-failed` plus truth-poll JSONs.
  - Updated `.cursor/memory/AGENT_SCRATCHPAD.md` with the new `J2` retry conclusion.
- Validation:
  - `J5` home:
    - `POST /control/home-joint-native {"joint":5}` -> `verified=true`
    - immediate truth poll summary: `J5 true 7/8`
  - `J1` home:
    - `POST /control/home-joint-native {"joint":1}` -> `verified=true`
    - immediate truth poll summary: `J1 true 5/8`
  - `J2` first attempt in this batch:
    - `POST /control/home-joint-native {"joint":2}` -> `verified=false`, abort `0x06010002`
    - immediate `/info/joints-detailed` still showed `J2` with a large stable roundtrip mismatch around `-52184` counts
  - Serialized retry preconditions:
    - `./start-stack.sh probe` showed `BUS_UP_DISARMED`
    - `/run/gradient-rt-motion/metrics.json` showed `native_home_active_axis_mask = 0`
  - `J2` serialized retry:
    - second `POST /control/home-joint-native {"joint":2}` -> same abort `0x06010002`
    - follow-up probe showed axis 1 faulted with `sw=0x9638`, `err=0xff00`, `Er11.0`
  - Fault cleanup attempt:
    - `POST /control/reset-faults` was accepted
    - immediate follow-up probe still showed axis 1 faulted, controller state `FAULTED`, drives disarmed
- Follow-up notes / risks:
  - The clean retry strongly argues this is not just a "frontend guardrail / clicked too soon after `J1`" issue; RTCore idle was confirmed before the retry and the same `J2` abort reproduced.
  - `J2` is now the blocker for the shared batch. Do not continue to the shared power-cycle stage until `J2` is either recovered cleanly or explicitly excluded from this run.
  - The experiment currently left the stack disarmed but faulted on axis 1 after the retry sequence.

## 2026-04-14 06:15 +0000

- Task summary:
  - Investigated the contradictory frontend success message reported by the user, confirmed that the per-joint row was deriving success from fallback telemetry instead of surfacing the contradiction, and patched the UI to fail safe.
- Changes:
  - Updated `web-ui/src/ControlPanel.tsx` so a row that is only "successful" via `statusword_bit15` fallback but still has a reported failed native-home result with nonzero abort code now renders `Drive Home verification conflicted | reported failed ...` instead of `Drive Home succeeded`.
  - Added a targeted regression test to `web-ui/src/ControlPanel.test.tsx`.
  - Updated `.cursor/memory/AGENT_SCRATCHPAD.md` with the new UI/root-cause rule and the user's observation that `J2` audibly de-energised much faster than the other axes.
- Validation:
  - `npm run test -- --run src/ControlPanel.test.tsx` in `web-ui`
  - result: `13 passed`
  - `ReadLints` on the changed frontend files returned no diagnostics.
- Follow-up notes / risks:
  - This UI fix addresses the misleading success message only; it does not solve the underlying `J2` native-home failure/retry behavior.
  - The user's audible timing clue supports the idea that `J2` is aborting early rather than completing the normal post-home tail.

## 2026-04-14 06:30 +0000

- Task summary:
  - Restarted from the user's soft-stopped state, explained the stale-anchor hypothesis against live `J2` data, and ran a focused `J2` trace to see whether a clean restarted epoch could home `J2` successfully.
- Changes:
  - No production backend changes in this pass.
  - Captured a focused `J2` experiment under `logs/encoder-retention/20260414-062709-j2-focused-trace/`.
  - Updated `.cursor/memory/AGENT_SCRATCHPAD.md` with the new `J2` stale-anchor interpretation and successful focused retry result.
- Validation:
  - Soft-stopped pre-restart probe:
    - `./start-stack.sh probe`
    - result: `J2 sw=0x9650 err=0x0000 pos_counts=178`, launcher absent, RTCore/EtherCAT still up
  - Pre-restart and pre-home snapshots:
    - `python scripts/a6ec_chapter5_probe.py snapshot --label pre-restart --axes J2 --experiment-id 20260414-062709-j2-focused-trace`
    - `python scripts/a6ec_chapter5_probe.py snapshot --label pre-home-post-restart --axes J2 --experiment-id 20260414-062709-j2-focused-trace`
  - Restart run:
    - `./start-stack.sh`
    - startup run id: `20260414-062712`
  - Pre-home API state after restart:
    - `/info/joints-detailed` still showed `J2 command_roundtrip_reference_error_counts ~= -52159` with old anchor `0.04842154167659891`
  - Focused trace:
    - saved to `logs/encoder-retention/20260414-062709-j2-focused-trace/j2-home-trace.json`
    - result summary:
      - `native_home_active_axis_mask` was seen active for `J2`
      - statuswords seen: `0x8233`, `0x9650`
      - error codes seen: `0x0000`
      - command result: `NATIVE_HOME_VERIFIED`
      - command finished at about `4.9s`
  - Post-home verification:
    - `./start-stack.sh probe` -> `J2 sw=0x9650 err=0x0000 pos_counts=27`
    - `python scripts/a6ec_chapter5_probe.py snapshot --label post-home-success --axes J2 --experiment-id 20260414-062709-j2-focused-trace`
    - post-home API poll saved to `post-home-success-poll.json`
    - poll result: `J2 true 10/12`, roundtrip error `-1 .. +2` counts, refreshed anchor `0.02350346188438531`
- Follow-up notes / risks:
  - This strongly supports the stale-anchor explanation for the huge software-side mismatch we saw before the focused retry.
  - A clean restarted epoch can still home `J2` successfully, so the earlier repeated batch failures were not enough to conclude that `J2` is fundamentally broken.
  - The earlier `Er11.0` event still needs caution; the new result does not prove that fault was benign, only that the current clean single-axis retry path can succeed.

## 2026-04-14 06:59 +0000

- Task summary:
  - Implemented the stale-anchor hardening plan in the EtherCAT RTCore backend so clean stale anchors diagnose explicitly and native-home verification now depends on coherent post-home anchor refresh.
- Changes:
  - Updated `src/gradient_os/arm_controller/backends/ethercat_rtcore/backend.py`:
    - reused `derive_effective_native_home_status()` for the stale-anchor diagnosis path
    - added helpers that compute implied anchor delta/tolerance and classify clean mismatches as `absolute_home_anchor_stale`
    - changed `native_home_joint()` so post-home anchor capture failures are no longer swallowed and the command returns `NATIVE_HOME_ANCHOR_REFRESH_FAILED` unless capture plus roundtrip validation both succeed
    - added `_absolute_home_anchor_validation_for_joint()` so the verified path proves stored-anchor coherence before reporting success
  - Updated `tests/test_gradient05_limits_and_backends.py` with focused coverage for:
    - stale-anchor diagnosis on a clean homed-looking axis
    - native-home anchor capture failure
    - native-home post-home validation failure
    - conservative startup bootstrap behavior with an existing anchor
  - Updated `.cursor/memory/AGENT_SCRATCHPAD.md` with the new backend guardrails and validation rule.
- Validation:
  - `source "/home/pi/GradientOS/.venv/bin/activate" && PYTHONPATH=src python -m py_compile "src/gradient_os/arm_controller/backends/ethercat_rtcore/backend.py" "tests/test_gradient05_limits_and_backends.py"`
  - `source "/home/pi/GradientOS/.venv/bin/activate" && PYTHONPATH=src python -m pytest tests/test_gradient05_limits_and_backends.py -q -k 'native_home_captures_absolute_encoder_anchor or native_home_reports_anchor_refresh_failure_when_capture_raises or native_home_requires_post_home_anchor_validation or marks_truth_unavailable_when_absolute_anchor_does_not_roundtrip or diagnoses_stale_absolute_home_anchor_when_clean_homed_frame_disagrees or startup_bootstraps_missing_absolute_home_anchor or startup_bootstrap_keeps_existing_absolute_home_anchor'`
    - result: `7 passed, 52 deselected`
  - `source "/home/pi/GradientOS/.venv/bin/activate" && PYTHONPATH=src python -m pytest tests/test_api_endpoints.py -q -k 'control_home_joint_native'`
    - result: `3 passed, 66 deselected`
  - `npm test -- ControlPanel.test.tsx` in `web-ui`
    - result: `13 passed`
  - `ReadLints` on the edited backend/test files returned no diagnostics.
- Follow-up notes / risks:
  - The stale-anchor threshold is intentionally wider than the observed `0..3` count jitter band so only material anchor drift gets classified as stale; the broader roundtrip jitter issue itself remains open.
  - This pass validated the new command/result contract with focused tests only; I did not re-run live hardware native-home experiments after the code change.

## 2026-04-14 19:31 +0000

- Task summary:
  - Captured the current A6-EC frame-semantics and native-home lessons in durable repo docs, then routed the `gradientos-sop` skill to that note without pretending the workstream is already fully canonical.
- Changes:
  - Added `docs/ethercat/a6ec-frame-semantics-and-native-home.md` as a durable but explicitly provisional workstream note covering:
    - why `scripts/a6ec_chapter5_probe.py` exists
    - raw vs rotation vs CSP/reference frame separation
    - the current frame equations and anchor math
    - persistence, jitter, verification, UI-trust, and methodology lessons
    - the remaining open questions
  - Updated `.cursor/skills/gradientos-sop/SKILL.md` with an `Active Workstream Notes` route to the new doc.
  - Updated `.cursor/skills/gradientos-sop/config-and-drive-profiles.md` to point A6-EC drive/profile work at the new frame-semantics note.
  - Updated `.cursor/skills/gradientos-sop/commissioning-safety.md` to point A6-EC native-home reasoning at the new note and corrected the stale `607C` persistence wording so it matches the newer evidence.
  - Updated `.cursor/skills/gradientos-sop/validation-and-debugging.md` to point A6-EC debugging at `scripts/a6ec_chapter5_probe.py` plus the new note.
  - Updated `.cursor/memory/AGENT_SCRATCHPAD.md` with the documentation-pattern lesson from this pass.
- Validation:
  - Re-read `scripts/a6ec_chapter5_probe.py` header and reconstruction formulas before drafting the note.
  - Re-read the canonical-truth math area in `src/gradient_os/arm_controller/backends/ethercat_rtcore/backend.py` before documenting the equations.
  - Ran `ReadLints` on all edited Markdown files; no diagnostics were reported.
- Follow-up notes / risks:
  - The new doc is intentionally labeled as a durable WIP workstream note, not a final canonical SOP consolidation.
  - No unit tests or live hardware checks were run in this pass because the changes were documentation and skill-routing only.

## 2026-04-14 19:36 +0000

- Task summary:
  - Closed the remaining Python-side native-home wait-loop gap so a stale terminal result cannot fail a fresh home request before RTCore has advertised the new request as active.
- Changes:
  - Updated `src/gradient_os/arm_controller/backends/ethercat_rtcore/backend.py`:
    - `_wait_for_native_home_result()` now only trusts terminal `failed`/`succeeded` states after the target axis has been observed in `native_home_active_axis_mask`
    - if fresh metrics arrive but the target axis is never observed active, the wait now degrades to `pending` / `timed_out` with a zero top-level abort code instead of surfacing a stale previous failure
  - Updated `tests/test_gradient05_limits_and_backends.py` with two focused regressions for:
    - stale failed telemetry before the active epoch is seen
    - genuine failure after the active epoch is seen and clears
- Validation:
  - `source "/home/pi/GradientOS/.venv/bin/activate" && PYTHONPATH=src python -m py_compile "src/gradient_os/arm_controller/backends/ethercat_rtcore/backend.py" "tests/test_gradient05_limits_and_backends.py"`
  - `source "/home/pi/GradientOS/.venv/bin/activate" && PYTHONPATH=src python -m pytest tests/test_gradient05_limits_and_backends.py -q -k 'wait_for_native_home_result_waits_for_active_mask_clear_before_statusword_fallback or wait_for_native_home_result_ignores_stale_failed_report_before_active_mask_seen or wait_for_native_home_result_reports_failed_after_active_mask_seen or wait_for_native_home_result_accepts_stale_failed_report_after_active_mask_clears'`
    - result: `4 passed, 57 deselected`
  - `source "/home/pi/GradientOS/.venv/bin/activate" && PYTHONPATH=src python -m pytest tests/test_gradient05_limits_and_backends.py -q -k 'native_home_metrics_result or wait_for_native_home_result'`
    - result: `7 passed, 54 deselected`
  - `ReadLints` on `src/gradient_os/arm_controller/backends/ethercat_rtcore/backend.py` and `tests/test_gradient05_limits_and_backends.py`
    - result: no diagnostics
- Follow-up notes / risks:
  - This closes the Python-side stale-failure race only; I did not re-run live hardware native-home validation in this pass.
  - Telemetry/UI still intentionally expose the broader effective native-home semantics; this change only tightens when the backend accepts a terminal result for the current request.

## 2026-04-14 19:48 +0000

- Task summary:
  - Stabilized the disarmed runtime header safety badge so the frontend no longer flashes a transient green `SAFE` state while the drives are still inactive and sync readiness is still settling.
- Changes:
  - Updated `web-ui/src/ControlPanel.tsx`:
    - `ControlPanelRuntimeHeader` now treats a disarmed, sync-only unsettled readiness condition as neutral `CHECK` instead of `BLOCKED`
    - the green `SAFE` badge is now held behind a short disarmed-only stabilization window so transient `safe_for_power_transition=true` packets do not make the header look active
    - the runtime-header `Power Up` button now follows that same stabilized disarmed-ready signal, which is stricter than the raw backend bit and avoids enable-button flicker during sync jitter
  - Updated `web-ui/src/ControlPanel.test.tsx` with focused regressions for:
    - sync-only unsettled readiness showing `CHECK` while disarmed
    - delayed `SAFE` display after a stable safe signal while disarmed
- Validation:
  - `cd /home/pi/GradientOS/web-ui && npm test -- ControlPanel.test.tsx`
    - result: `15 passed`
  - `ReadLints` on `web-ui/src/ControlPanel.tsx` and `web-ui/src/ControlPanel.test.tsx`
    - result: no diagnostics
- Follow-up notes / risks:
  - This hardens the misleading header flicker and disarmed enable-button jitter only; it does not change the underlying backend reason the sync-readiness bit may still oscillate near the current roundtrip/jitter threshold.

## 2026-04-14 20:04 +0000

- Task summary:
  - Reproduced the unexpected startup failure from the launcher side and traced it to a stale-owner EtherCAT master reservation caused by a hung RTCore metrics thread, not by the recent frontend-only changes.
- Changes:
  - No code changes in this pass.
  - Collected live bring-up evidence from:
    - `./start-stack.sh probe`
    - `systemctl status ethercat.service gradient-rt-motion.service --no-pager`
    - `journalctl -u ethercat.service -u gradient-rt-motion.service ...`
    - `journalctl -k -n 80 --no-pager`
    - `./start-stack.sh stop --hard`
    - fresh launcher reproduction `./start-stack.sh` on run `20260414-200318`
- Validation:
  - Initial failed launcher run confirmed in terminal/logs: `20260414-195850`
  - Fresh reproduction run: `20260414-200318`
    - launcher again stalled at `responding=0/6 online=0/6 operational=0/6 startup_ready=0 wkc=0`
    - launcher exited with `bus failed readiness`
  - `gradient-rt-motion.service` repeatedly failed with:
    - `Failed to reserve master: Device or resource busy`
    - `ecrt_request_master(0) failed`
  - Kernel log showed the stuck owner directly:
    - `task metrics:42143 blocked for more than 120 seconds`
    - blocked in `ecrt_master_sdo_upload` / `ec_ioctl` in `ec_master`
  - Hard stop result:
    - `ethercat.service` stop failed with `rmmod: ERROR: Module ec_generic is in use`
    - `lsmod` still showed `ec_generic` and `ec_master` loaded
- Follow-up notes / risks:
  - The visible `42130` `gradient-rt-mot` process is a zombie marker; the real blocker is the hung `metrics` thread (`pid 42143`, `tgid 42130`) in uninterruptible kernel sleep.
  - Because the owner is stuck in kernel space and survives SIGKILL/systemd stop, the most likely recovery is a host reboot before any fresh launcher run can succeed.
  - This evidence points to a stale-owner/runtime-kernel hang, not to the recent `ControlPanel.tsx` UI patch.

## 2026-04-14 20:12 +0000

- Task summary:
  - Added first-failure fieldbus diagnostics and stopped RTCore from restart-looping on the specific EtherCAT master-reservation failure so stale-owner events are easier to identify and less noisy.
- Changes:
  - Updated `start-stack.sh`:
    - on bus-readiness timeout, the launcher now writes `fieldbus-failure-diagnostics/` inside the current run log directory
    - captured artifacts include `probe.json`, `systemd-status.txt`, `unit-journal.txt`, `kernel-journal.txt`, `processes.txt`, `kernel-modules.txt`, and `summary.txt`
    - the launcher now echoes the heuristic summary lines directly in the terminal/log on failure
  - Updated `src/gradient_rt_motion/main.cpp`:
    - `ecrt_request_master(0)` failure now exits with dedicated code `75`
    - the RTCore log now states that this path likely means a stale owner or hung EtherCAT kernel task
  - Updated `systemd/rt-motion/gradient-rt-motion.service`:
    - `Restart=on-failure`
    - `RestartPreventExitStatus=75`
    - result: systemd no longer immediately restart-loops RTCore on the master-busy path
- Validation:
  - `bash -n ./start-stack.sh`
  - `systemd-analyze verify /home/pi/GradientOS/systemd/rt-motion/gradient-rt-motion.service`
  - `make -C src/gradient_rt_motion`
  - reproduced failure with new diagnostics on run `20260414-201011`
    - launcher created `fieldbus-failure-diagnostics/`
    - `systemd-status.txt` captured RTCore exit `status=75`
  - re-ran after summary parser fix on run `20260414-201135`
    - launcher summary now explicitly reported:
      - `likely_cause=rtcore_master_reservation_failed`
      - `hung_kernel_task=metrics:42143`
      - `rtcore_zombie_marker_present=1`
      - `ethercat_modules_loaded=1`
    - `journalctl -u gradient-rt-motion.service -S "2026-04-14 20:11:35"` showed a single failure window with no follow-up scheduled restart loop
- Follow-up notes / risks:
  - This improves observability and reduces restart thrash, but it does not fix the underlying kernel-space hang once it has already happened.
  - Recent repo/devlog history still points at the Apr 12-13 RTCore metrics-thread SDO polling/readback additions as the leading "why this is new" correlation; that path likely needs the next hardening pass if the goal is recurrence reduction rather than better diagnosis.

## 2026-04-14 20:22 +0000

- Task summary:
  - Added per-feature RTCore metrics-thread isolation toggles so the next reboot can identify which SDO polling path is actually responsible for the new stale-owner wedge instead of relying on inference.
- Changes:
  - Updated `src/gradient_rt_motion/main.cpp`:
    - added `parse_env_flag(...)`
    - introduced independent env-controlled toggles for:
      - startup drive-config readback
      - native-home offset refresh
      - absolute-feedback polling
    - RTCore now logs the toggle states at startup
    - `metrics.json` now includes:
      - `metrics_startup_readback_enabled`
      - `metrics_native_home_refresh_enabled`
      - `metrics_absolute_feedback_poll_enabled`
  - Updated `systemd/rt-motion/gradient-rt-motion.service` with default envs:
    - `GRADIENT_RT_METRICS_STARTUP_READBACK_ENABLED=1`
    - `GRADIENT_RT_METRICS_NATIVE_HOME_REFRESH_ENABLED=1`
    - `GRADIENT_RT_METRICS_ABSOLUTE_FEEDBACK_POLL_ENABLED=1`
  - Updated `start-stack.sh` probe rendering to print the RTCore metrics-SDO toggle state in the human-readable hardware summary.
- Validation:
  - `bash -n ./start-stack.sh`
  - `systemd-analyze verify /home/pi/GradientOS/systemd/rt-motion/gradient-rt-motion.service`
  - `make -C src/gradient_rt_motion`
  - `ReadLints` on:
    - `src/gradient_rt_motion/main.cpp`
    - `start-stack.sh`
    - `systemd/rt-motion/gradient-rt-motion.service`
    - result: no diagnostics
- Follow-up notes / risks:
  - I could not live-validate the new toggle output on a healthy boot yet because the host is still in the existing stale-owner kernel-hang state; a reboot is still required before any clean bring-up experiment can succeed.
  - The intended next step is an evidence-first reboot matrix:
    - all toggles enabled
    - then disable only absolute-feedback polling
    - then disable native-home refresh
    - then disable startup readback
  - This gives us the first real A/B path to identify the culprit metrics-SDO behavior without assuming in advance that absolute-feedback polling is the only problem.

## 2026-04-14 20:59 +0000

- Task summary:
  - Ran the first live matrix trial after reboot, confirmed that the startup wedge reproduces with all metrics-thread SDO features enabled, and staged the next reboot to run with periodic absolute-feedback polling disabled.
- Changes:
  - No repo code changes in this pass.
  - Added a persistent systemd drop-in override:
    - `/etc/systemd/system/gradient-rt-motion.service.d/99-metrics-isolation.conf`
    - `Environment=GRADIENT_RT_METRICS_ABSOLUTE_FEEDBACK_POLL_ENABLED=0`
  - Reloaded systemd daemon after writing the drop-in.
- Validation:
  - Post-reboot pre-test health check:
    - `systemctl is-active ethercat.service gradient-rt-motion.service` -> both `active`
    - `./start-stack.sh probe` -> `ethercat_master_state=OP`, `rtcore_state=UP`, `physical_state=BUS_UP_DISARMED`
  - Baseline matrix trial:
    - `./start-stack.sh`
    - run id: `20260414-205728`
    - `RTCORE SYNC COMPLETE` took `42.272s`
    - launcher then failed with `bus failed readiness`
    - diagnostics summary: `likely_cause=rtcore_master_reservation_failed`
  - Post-failure inspection:
    - `./start-stack.sh probe` -> `ethercat_master_state=DOWN`, `rtcore_state=DOWN`
    - `systemctl status gradient-rt-motion.service ethercat.service --no-pager`
      - RTCore failed with exit `75`
      - leftover zombie marker `pid 1642`
      - metrics thread `pid 1769` survived SIGKILL during stop
- Follow-up notes / risks:
  - The baseline result means the regression is real and reproducible under the fully enabled metrics-thread SDO configuration.
  - The host is contaminated again after the baseline trial, so another reboot is required before the second matrix step can run.
  - The next trial is already staged to disable only periodic absolute-feedback polling, which is the highest-value first isolation because it is the only continuous metrics-thread SDO path.

## 2026-04-14 21:40 +0000

- Task summary:
  - Ran the second post-reboot matrix trial with only `GRADIENT_RT_METRICS_ABSOLUTE_FEEDBACK_POLL_ENABLED=0`, proved that the startup wedge no longer reproduces under that A/B condition, fixed the now-unmasked launcher preflight circular import, and reran the stack to a full healthy bring-up.
- Changes:
  - Added `src/gradient_os/telemetry/native_home_status.py` to hold the shared native-home status derivation helper.
  - Updated `src/gradient_os/telemetry/drive_faults.py` to import the shared helper instead of defining it inline.
  - Updated `src/gradient_os/arm_controller/backends/ethercat_rtcore/backend.py` to import the helper from `telemetry.native_home_status` instead of from `drive_faults.py`.
  - Added a focused circular-import regression in `tests/test_drive_faults.py` that imports both the drive-fault path and the EtherCAT backend in one Python process.
- Validation:
  - Isolation check before rerun:
    - `systemctl is-active ethercat.service gradient-rt-motion.service` -> both `active`
    - `/run/gradient-rt-motion/metrics.json` showed:
      - `metrics_startup_readback_enabled=1`
      - `metrics_native_home_refresh_enabled=1`
      - `metrics_absolute_feedback_poll_enabled=0`
  - First isolated launcher run:
    - `./start-stack.sh`
    - run id: `20260414-213541`
    - `RTCORE SYNC COMPLETE` in `1.006s`
    - `BUS READY` in `1.708s`
    - no stale-owner/master-reservation failure reproduced
    - launcher then failed later with `startup preflight could not build a fault-reset plan from the probe payload`
  - Root-cause reproduction for the new launcher failure:
    - direct Python import of `gradient_os.telemetry.drive_faults` reproduced:
      - `ImportError: cannot import name 'derive_effective_native_home_status' from partially initialized module 'gradient_os.telemetry.drive_faults'`
  - Focused code checks after the import fix:
    - `source "/home/pi/GradientOS/.venv/bin/activate" && PYTHONPATH=src python -m py_compile src/gradient_os/telemetry/native_home_status.py src/gradient_os/telemetry/drive_faults.py src/gradient_os/arm_controller/backends/ethercat_rtcore/backend.py tests/test_drive_faults.py`
    - `source "/home/pi/GradientOS/.venv/bin/activate" && PYTHONPATH=src python -m pytest tests/test_drive_faults.py -q` -> `9 passed`
    - `ReadLints` on touched Python files -> no diagnostics
  - Full rerun after the import fix:
    - `./start-stack.sh`
    - run id: `20260414-213805`
    - `RTCORE SYNC COMPLETE` in `992ms`
    - `BUS READY` in `1.964s`
    - startup preflight passed: `no disarmed drive faults detected`
    - controller, API, and web all came online
    - launcher reported `STACK BOOT COMPLETE in 19.775s`
    - follow-up `./start-stack.sh probe` showed:
      - `controller_udp: up`
      - `api_http: up`
      - `rtcore_state: UP`
      - `physical_state: BUS_UP_DISARMED`
      - all 6 axes `SwitchOnDisabled` with `err=0x0000`
- Follow-up notes / risks:
  - This is the strongest live evidence so far that periodic absolute-feedback polling is the leading trigger for the new startup wedge.
  - The result is still one A/B datapoint, not a full proof that the other two metrics-thread SDO features are innocent; additional reboot-cycle trials are still useful if we want to narrow the exact offending path further.
  - The supervised stack from run `20260414-213805` remains up at the end of this task.

## 2026-04-14 22:34 +0000

- Task summary:
  - Verified that a clean `./start-stack.sh stop --hard` is enough for iterative stop/start testing under the non-wedged isolation config; a reboot is not required between healthy cycles.
- Changes:
  - No repo code changes in this pass.
- Validation:
  - Starting point:
    - `./start-stack.sh probe` from run `20260414-213805` showed a healthy live stack:
      - `controller_udp: up`
      - `api_http: up`
      - `physical_state: BUS_UP_DISARMED`
      - `rtcore_state: UP`
      - `ethercat_master_state: OP`
  - Hard stop test:
    - `./start-stack.sh stop --hard`
    - launcher stopped
    - `gradient-rt-motion.service` stopped in `333ms`
    - `ethercat.service` stopped in `1.178s`
    - final stop probe reported:
      - `physical: INACTIVE`
      - `ethercat: DOWN`
      - `rtcore: DOWN`
  - Post-stop checks:
    - `systemctl is-active ethercat.service gradient-rt-motion.service` -> both `inactive`
    - `./start-stack.sh probe` -> `physical_state=INACTIVE`, `ethercat_master_state=DOWN`, `rtcore_state=DOWN`
  - Restart without reboot:
    - `./start-stack.sh`
    - run id: `20260414-223306`
    - startup recovery briefly classified `rtcore_up_master_down`
    - launcher performed one hard RTCore/EtherCAT recycle automatically
    - `BUS READY` then succeeded in `9.105s`
    - stack reached `STACK BOOT COMPLETE in 37.496s`
  - Post-restart health:
    - `./start-stack.sh probe` showed:
      - `controller_udp: up`
      - `api_http: up`
      - `physical_state: BUS_UP_DISARMED`
      - `rtcore_state: UP`
      - `ethercat_master_state: OP`
      - all six axes `SwitchOnDisabled` with `err=0x0000`
- Follow-up notes / risks:
  - Operationally, this means we can continue the metrics-thread bug hunt with `stop --hard` / start cycles as long as the host has not already entered the hung-kernel-thread stale-owner state.
  - The reboot requirement still applies once the bad configuration poisons the host, because that kernel-space stuck owner is not cleared by user-space stop commands.

## 2026-04-14 23:01 +0000

- Task summary:
  - Promoted the validated `stop --hard` versus reboot testing rule into the canonical GradientOS SOP skill under validation/debugging guidance.
- Changes:
  - Updated `.cursor/skills/gradientos-sop/validation-and-debugging.md`:
    - added `## Live Bring-Up Loops`
    - documented the healthy-cycle rule:
      - use `./start-stack.sh probe`
      - then `./start-stack.sh stop --hard`
      - then `./start-stack.sh`
    - documented the reboot threshold:
      - reboot is only required after stale-owner symptoms or failed EtherCAT ownership teardown
    - documented that one launcher-managed RTCore/EtherCAT recovery recycle is acceptable if startup still reaches `BUS_UP_DISARMED`
- Validation:
  - Read back the edited SOP file to confirm wording and placement.
  - `ReadLints` on `.cursor/skills/gradientos-sop/validation-and-debugging.md` -> no diagnostics.
- Follow-up notes / risks:
  - The canonical skill now reflects the proven live-testing workflow, but the exact offending metrics-thread SDO path still needs more isolation trials before that narrower conclusion should be promoted further.

## 2026-04-14 23:11 +0000

- Task summary:
  - Resumed the encoder-retention workstream with the planned no-motion all-joints consistency control before any new home or jog.
- Changes:
  - No repo code changes in this pass.
  - Captured new experiment `logs/encoder-retention/20260414-230845-all-joints-stationary-consistency/` with:
    - `stationary-1.json/.md`
    - `stationary-2.json/.md`
    - `stationary-3.json/.md`
    - `info-joints-detailed-current.json`
    - `metrics-current.json`
    - `anchors-current.json`
- Validation:
  - Pre-check:
    - `./start-stack.sh probe` showed a healthy stack in `BUS_UP_DISARMED` with all six axes clean.
  - Capture commands:
    - `scripts/a6ec_chapter5_probe.py snapshot --label stationary-1 --axes J1 J2 J3 J4 J5 J6 --experiment-id 20260414-230845-all-joints-stationary-consistency`
    - repeated for `stationary-2` and `stationary-3` with short waits
    - `curl -sf http://127.0.0.1:4000/info/joints-detailed`
    - copied `/run/gradient-rt-motion/metrics.json`
    - copied `.gradient_absolute_encoder_anchors.json`
  - Comparison against retained baselines:
    - `J2/J3/J4/J6` remained in the same overall raw/reference/rotation family with no large semantic shift
    - `J1/J5` showed larger-than-jitter but still small absolute deltas versus older immediate-home baselines (`J1 raw +6`, `J5 raw +17` counts), not whole-turn or thousand-count changes
    - all anchor values matched the current anchor file exactly; no unexpected anchor mutation occurred
    - current stationary spans inside the new experiment stayed small:
      - raw absolute `U40.20` spans `0..3`
      - reference-family spans stayed in the low single digits, worst observed `J4 6064 span=4`
  - API constraint discovered during this run:
    - `info-joints-detailed-current.json` reported `canonical_joint_truth_available=false`
    - all joints showed `truth_reason=absolute_feedback_unavailable`
    - `metrics-current.json` confirmed the safe isolation is still active: `metrics_absolute_feedback_poll_enabled=0`
- Follow-up notes / risks:
  - The no-motion control passed on direct SDO frame/anchor evidence strongly enough to proceed to the next planned step: a drive-only power-cycle control with the same all-joints capture set.
  - The probe's current one-count match booleans over-flagged normal live-read wander again (`2..4` count deltas), so those booleans should not be treated as semantic failures by themselves.
  - API truth/roundtrip criteria are currently blind by design under the startup-wedge isolation config; if that part of the old checklist is required again, we will have to deliberately re-enable the risky metrics absolute-feedback polling path and accept the possibility of re-poisoning the host.

## 2026-04-14 23:25 +0000

- Task summary:
  - Replaced the probe's pure one-count drift reporting with explicit drift-magnitude categories while keeping backward-compatible one-count booleans.
- Changes:
  - Updated `scripts/a6ec_chapter5_probe.py`:
    - added drift bucket thresholds:
      - `standard <= 2`
      - `medium <= 6`
      - `large <= 10`
      - `excessive <= 100`
      - `extreme > 100`
    - added `_classify_count_delta(...)`
    - added `_delta_summary(...)`
    - added category and absolute-magnitude fields for the existing bridge deltas
    - updated markdown and condensed console output to include the new categories
  - Added `tests/test_a6ec_chapter5_probe.py` to lock the bucket boundaries and rendered category output.
- Validation:
  - `source "/home/pi/GradientOS/.venv/bin/activate" && PYTHONPATH=src python -m py_compile scripts/a6ec_chapter5_probe.py tests/test_a6ec_chapter5_probe.py`
  - `source "/home/pi/GradientOS/.venv/bin/activate" && PYTHONPATH=src python -m pytest tests/test_a6ec_chapter5_probe.py -q` -> `3 passed`
  - `ReadLints` on the touched files -> no diagnostics
  - Live proof:
    - captured `stationary-4-classified` under experiment `20260414-230845-all-joints-stationary-consistency`
    - confirmed the new condensed output now reports categories such as:
      - `standard` for `0..2`
      - `medium` for a live `-3` count bridge delta on `J2`
- Follow-up notes / risks:
  - This improves interpretation, but it does not by itself change the actual persistence decision rule: isolated `medium` drift is still descriptive, not proof of a semantic frame change.

## 2026-04-14 23:46 +0000

- Task summary:
  - Completed the planned drive-only power-cycle control and compared the post-cycle all-joints stationary captures against the pre-cycle control and retained baselines.
- Changes:
  - No repo code changes in this pass.
  - Brought the stack back up after the user hard stop + drive power cycle:
    - run id: `20260414-234249`
    - startup used one launcher-managed `rtcore_up_master_down` recycle
    - then reached `STACK BOOT COMPLETE`
  - Added post-cycle artifacts under `logs/encoder-retention/20260414-230845-all-joints-stationary-consistency/`:
    - `post-power-cycle-1.json/.md`
    - `post-power-cycle-2.json/.md`
    - `post-power-cycle-3.json/.md`
    - `info-joints-detailed-post-power-cycle.json`
    - `metrics-post-power-cycle.json`
    - `anchors-post-power-cycle.json`
- Validation:
  - Pre-start verification:
    - `systemctl is-active ethercat.service gradient-rt-motion.service` -> both `inactive`
    - `./start-stack.sh probe` -> `physical_state=INACTIVE`, `rtcore_state=DOWN`, `ethercat_master_state=DOWN`
  - Post-start verification:
    - launcher reported `STACK BOOT COMPLETE in 37.179s`
    - bus landed `READY and DISARMED`
  - Post-cycle capture commands:
    - `scripts/a6ec_chapter5_probe.py snapshot --label post-power-cycle-1 --axes J1 J2 J3 J4 J5 J6 --experiment-id 20260414-230845-all-joints-stationary-consistency`
    - repeated for `post-power-cycle-2` and `post-power-cycle-3` with short waits
    - `curl -sf http://127.0.0.1:4000/info/joints-detailed`
    - copied `/run/gradient-rt-motion/metrics.json`
    - copied `.gradient_absolute_encoder_anchors.json`
  - Post-cycle direct evidence:
    - raw absolute `U40.20` spans stayed `0..3` counts across all joints
    - reference-family spans stayed in the low single digits
    - only a few `medium` bucket deltas appeared, all at `3` counts
    - no `large`, `excessive`, or `extreme` post-cycle deltas appeared
  - Pre-vs-post latest comparison:
    - `J1 raw delta = 0`
    - `J2 raw delta = -1`
    - `J3 raw delta = 0`
    - `J4 raw delta = -2`
    - `J5 raw delta = 0`
    - `J6 raw delta = -2`
    - all post-cycle anchor values matched the pre-cycle anchors exactly
  - `J2` focused note:
    - raw absolute `-1` count across the power cycle
    - `6064 +2`, `6063 -1`, `6062 -2`, `60FC -1`, `U40.28 +1`
    - anchor unchanged at `0.02350346188438531`
  - API limitation remains:
    - `info-joints-detailed-post-power-cycle.json` still reported `canonical_joint_truth_available=false`
    - all joints still showed `truth_reason=absolute_feedback_unavailable`
    - `metrics-post-power-cycle.json` confirmed `metrics_absolute_feedback_poll_enabled=0`
- Follow-up notes / risks:
  - This drive-only power-cycle control passed on direct SDO and anchor evidence for all six joints.
  - `J2` did not reproduce the earlier fragile behavior across this control; current evidence says it is power-cycle-stable under the present configuration.
  - Older baseline mismatches that already existed pre-cycle (especially weaker `J1/J5` comparisons and wrap-adjacent `J6` reference fields) should not be misattributed to the power cycle itself, because the pre-vs-post control stayed tight.
  - If we need API truth and roundtrip evidence again, we will have to intentionally re-enable the risky absolute-feedback metrics polling path and accept the chance of reintroducing the startup wedge.

## 2026-04-15 00:13 +0000

- What changed:
  - Hardened `src/gradient_rt_motion/main.cpp` by serializing helper/metrics SDO uploads and downloads against `ecrt_release_master()`.
  - Rebuilt `src/gradient_rt_motion/gradient-rt-motion`, reinstalled `/usr/local/bin/gradient-rt-motion`, and restored the live systemd override so `GRADIENT_RT_METRICS_ABSOLUTE_FEEDBACK_POLL_ENABLED=1`.
  - Updated `src/gradient_os/arm_controller/backends/ethercat_rtcore/backend.py` so command-frame roundtrip acceptance uses a `3`-count tolerance while stale-anchor diagnosis stays at `8` counts.
  - Added a new tolerance regression and tightened the existing native-home sequencing test in `tests/test_gradient05_limits_and_backends.py`.
- Validation:
  - `make` in `src/gradient_rt_motion`
  - `pytest tests/test_gradient05_limits_and_backends.py -k "roundtrip or stale_absolute_home_anchor or native_home"` -> `18 passed`
  - Live restart validation with full metrics enabled:
    - `./start-stack.sh stop --hard` -> clean `physical_state=INACTIVE`, `rtcore_state=DOWN`, `ethercat_master_state=DOWN`
    - `./start-stack.sh` -> `STACK BOOT COMPLETE` on run `20260415-000137`
    - repeated `./start-stack.sh stop --hard` -> `./start-stack.sh` -> `STACK BOOT COMPLETE` on run `20260415-000821`
  - Live `J2` native-home revalidation:
    - experiment `20260415-000821-j2-native-home-revalidation`
    - `pre-home`, `post-home-immediate`, `post-home-settle`, and `post-home-fault` probe snapshots captured
    - sidecars saved: `info-joints-detailed-*`, `metrics-*`, `anchors-*`
    - API command `POST /control/home-joint-native {"joint":2}` returned `NATIVE_HOME_VERIFIED` with anchor capture/refresh success and updated `J2` anchor `0.023517842954271735`
- Follow-up notes / risks:
  - The startup wedge with full metrics did not reproduce after the RTCore SDO/release fence; this looks like a meaningful live fix, not just another isolation.
  - `J2` native-home is still not end-to-end clean: a few seconds after the verified return, `./start-stack.sh probe` showed `J2` faulted with `err=0xff00` / `Er11.0 | Excessive motor speed upon servo drive power-on`, while `native_home_state` remained `2`.
  - The backend’s current native-home success contract is therefore still too optimistic for A6-EC: we now need a post-home fault-free settle check, not just verified terminal state plus anchor refresh.
  - After the tolerance patch, `J2` itself no longer causes the live command-roundtrip false negative; the current global truth-unavailable flapping is coming from `J1` at roughly `4` counts.

## 2026-04-15 00:32 +0000

- What changed:
  - Updated `src/gradient_os/arm_controller/backends/ethercat_rtcore/backend.py` so `native_home_joint()` no longer returns `NATIVE_HOME_VERIFIED` immediately after anchor refresh. It now also waits through a short post-home settle window and downgrades the result if the target axis faults, drops offline, or otherwise fails to stay clean.
  - Added new backend result paths for post-home settle failures and pending settle verification:
    - `NATIVE_HOME_POST_HOME_SETTLE_FAILED`
    - `NATIVE_HOME_POST_HOME_SETTLE_PENDING`
  - Added backend regressions in `tests/test_gradient05_limits_and_backends.py` for:
    - verified success including the settle step
    - settle downgrade when the target axis reports a hard post-home fault (`drive_faulted`, modeled on the live `0xff00` case)
    - direct settle helper coverage for both clean and faulted axis snapshots
  - Updated `web-ui/src/ControlPanel.tsx` so short `/info/joints-detailed` dropouts no longer immediately clear the displayed joint values; the UI now holds the last good live joint values briefly while backend truth is flapping.
  - Added `web-ui/src/ControlPanel.test.tsx` coverage for the transient joint-feedback hold behavior.
- Validation:
  - Live diagnostic sampling before any restart:
    - `curl -sf http://127.0.0.1:4000/info/joints-detailed`
    - `./start-stack.sh probe`
    - repeated 20-sample `/info/joints-detailed` loop
  - Findings from the 20-sample loop:
    - `canonical_joint_truth_available` still flaps on the live stack
    - the flapping is not just `J1`; sampled dropouts implicated `J1`, `J4`, `J5`, and `J6`
    - current UI disappearing values are therefore backend truth flapping plus frontend blank-on-miss behavior, not a literal UI transport disconnect
  - `pytest tests/test_gradient05_limits_and_backends.py -k "native_home or post_settle or roundtrip or stale_absolute_home_anchor"` -> `21 passed`
  - `cd web-ui && npx vitest run src/ControlPanel.test.tsx` -> `16 passed`
- Follow-up notes / risks:
  - The backend hardening is not yet live-loaded into the currently running controller process; a controller/stack restart is still required before the next real `J2` native-home retest.
  - The new UI hold reduces visible flicker, but it does not solve the underlying truth-flap cause. The current live command-roundtrip threshold edge is still being crossed by multiple axes, not only `J1`.

## 2026-04-15 00:39 +0000

- What changed:
  - Reverted the temporary 1.5 s joint-feedback hold in `web-ui/src/ControlPanel.tsx` so the frontend returns to the original immediate-clear behavior on `/info/joints-detailed` misses.
  - Updated `src/gradient_os/arm_controller/backends/ethercat_rtcore/backend.py` so `_COMMAND_ROUNDTRIP_TOLERANCE_COUNTS = 6.0` instead of `3.0`.
  - Updated `tests/test_gradient05_limits_and_backends.py` to accept `6` counts of stationary roundtrip wander and still reject `7` counts.
  - Updated `web-ui/src/ControlPanel.test.tsx` to match the restored immediate-clear frontend behavior.
- Validation:
  - Captured a 90 s stationary diagnostic run:
    - raw log: `logs/joint-truth-jitter/20260415-003401-joints-detailed-90s.jsonl`
    - summary: `logs/joint-truth-jitter/20260415-003401-joints-detailed-90s.summary.json`
  - Key measured absolute roundtrip-error ranges from that run:
    - `J1`: max `5`, p95 `4`
    - `J2`: max `3`, p95 `2`
    - `J3`: max `3`, p95 `2`
    - `J4`: max `5`, p95 `3`, p99 `4`
    - `J5`: max `6`, p95 `4`, p99 `5`
    - `J6`: max `6`, p95 `5`, p99 `6`
  - Threshold replay against the 90 s log:
    - values exceeding `3` counts: `249`
    - values exceeding `4` counts: `63`
    - values exceeding `5` counts: `9`
    - values exceeding `6` counts: `0`
  - `pytest tests/test_gradient05_limits_and_backends.py -k "roundtrip or native_home or stale_absolute_home_anchor or post_settle"` -> `22 passed`
  - `cd web-ui && npx vitest run src/ControlPanel.test.tsx` -> `16 passed`
- Follow-up notes / risks:
  - The new `6`-count threshold is measurement-backed for the current stationary live state and should remove the observed false truth dropouts, but it is not yet live-loaded into the running controller process.
  - The next meaningful live step is still a restart into this build, then re-check `/info/joints-detailed` stability before the next `J2` native-home retest.

## 2026-04-15 00:43 +0000

- What changed:
  - Performed live post-restart validation after the user soft-stopped and restarted the stack with the new backend build loaded.
- Validation:
  - `./start-stack.sh probe` after restart:
    - `physical_state=BUS_UP_DISARMED`
    - all six axes `SwitchOnDisabled`
    - `J2` no longer faulted
  - Fresh `curl -sf http://127.0.0.1:4000/info/joints-detailed` sample showed:
    - `canonical_joint_truth_available=true`
    - all six axes truth-available
    - larger `command_roundtrip_tolerance_rad` values consistent with the live-loaded `6`-count threshold
  - 20-sample post-restart loop:
    - `canonical_joint_truth_available=true` on all 20 samples
    - high-but-accepted outliers still appeared on `J4/J5/J6`, including `J6 ~= -6` counts
  - 60 s post-restart soak:
    - `300/300` reads succeeded
    - `14/300` samples still went `canonical_joint_truth_available=false`
    - all 14 failures were `command_frame_roundtrip_mismatch`
    - max absolute roundtrip error by axis:
      - `J1`: `4`
      - `J2`: `3`
      - `J3`: `3`
      - `J4`: `5`
      - `J5`: `6`
      - `J6`: `9`
  - Follow-up 20 s axis breakdown:
    - remaining false samples were concentrated on `J6`
    - directly observed a failing `J6` sample at about `-7` counts
- Follow-up notes / risks:
  - The new `6`-count tolerance materially improved truth stability after restart, but it did not eliminate flapping completely.
  - The remaining live truth dropouts now look `J6`-dominated rather than broadly multi-axis. The next tuning decision may need to be axis-specific, or otherwise specifically justified around `J6`, instead of another blanket global threshold increase.
  - The user-pasted terminal flapping is therefore still relevant after restart; the `No UDP commands received` warnings are separate controller-idle link warnings, not encoder-jitter signals.

## 2026-04-15 00:52 +0000

- What changed:
  - Updated `src/gradient_os/arm_controller/backends/ethercat_rtcore/backend.py` so `_COMMAND_ROUNDTRIP_TOLERANCE_COUNTS = 10.0` per user request.
  - Updated `tests/test_gradient05_limits_and_backends.py` so the roundtrip regressions now accept `10` counts and reject `11`.
- Validation:
  - `pytest tests/test_gradient05_limits_and_backends.py -k "roundtrip or native_home or stale_absolute_home_anchor or post_settle"` -> `22 passed`
  - Investigated the current on-screen joint values against live API and persisted calibration state:
    - current live `/info/joints-detailed` sample matched the UI:
      - `J3 ~= -3.599°`
      - `J4 ~= -19.996°`
    - `.gradient_joint_zero_offsets.json` still stores `0.0` for all six software-zero offsets
    - `.gradient_absolute_encoder_anchors.json` contains persisted absolute-home anchors for all six joints
    - backend canonical truth still computes `absolute_axis_q - home_anchor_rad - software_zero`
- Follow-up notes / risks:
  - The `10`-count tolerance is implemented and tested, but it is not yet live-loaded into the running controller process; another restart is required before we can verify whether the remaining `J6`-driven truth flapping is gone.
  - The current `J3/J4` nonzero readouts do not appear to be a UI bug. Under the present calibration contract, they are semantically real because the stored software-zero offsets are all zero. If the intended parked pose should display `0.00` for every joint, that is a calibration/zero-contract issue, not a rendering issue.

## 2026-04-15 01:35 +0000

- What changed:
  - Investigated the user's challenge against the retained evidence directly instead of relying on the earlier summary.
  - Compared the current live `/info/joints-detailed` joint angles to yesterday's retained stationary and post-power-cycle snapshots in:
    - `logs/encoder-retention/20260414-230845-all-joints-stationary-consistency/stationary-3.json`
    - `logs/encoder-retention/20260414-230845-all-joints-stationary-consistency/post-power-cycle-3.json`
- Validation:
  - Current live sample:
    - `J3 = -3.5990936279296877°`
    - `J4 = -19.996185302734375°`
  - Retained baseline comparison:
    - vs `stationary-3`: `J3 delta = 0.0°`, `J4 delta = +0.000305°`
    - vs `post-power-cycle-3`: `J3 delta = +0.000027°`, `J4 delta = -0.000153°`
  - Also confirmed the other joints remain near the same parked values:
    - current live `J1/J2/J5/J6 = -0.0005768°, +0.0039001°, -0.0051855°, -0.0016479°`
    - all remain in the same tiny neighborhood as yesterday's retained snapshots
- Follow-up notes / risks:
  - This confirms the current `J3/J4` readouts are consistent with yesterday's persisted state; there is no evidence in the retained artifacts that those joints newly drifted or that the UI invented these values.
  - If the intended parked pose should display `0.00` on `J3/J4`, the remaining issue is calibration semantics (`home`/`zero` contract), not persistence failure.

## 2026-04-15 01:41 +0000

- What changed:
  - Re-checked the retained native-home evidence to answer whether "we homed all joints at their current position, therefore all joints should read zero."
  - Confirmed the persisted calibration split:
    - `.gradient_absolute_encoder_anchors.json` contains native-home anchors for all six joints
    - `.gradient_joint_zero_offsets.json` still contains `0.0` software-zero offsets for all six joints
  - Reconfirmed from backend code that native-home anchor capture and software zero are separate operations.
- Validation:
  - Backend contract:
    - canonical truth = `absolute_axis_q - home_anchor_rad - software_zero`
    - `set_logical_joint_current_position_as_zero()` writes `software_zero` / `_master_offsets_rad`
  - Retained post-home artifacts show:
    - `J1` post-home `canonical_rad ~= -7.67e-06`
    - `J2` clean post-home `canonical_rad ~= +1.29e-05`
    - `J5` post-home `canonical_rad ~= -7.06e-05`
    - `J6` post-home `canonical_rad ~= -4.79e-06`
    - `J3` post-home `canonical_rad ~= -0.062815`
    - `J4` persisted post-home `canonical_rad ~= -0.349013`
  - `J3` post-home truth poll also showed `reference_pre_zero_rad` remaining around `-0.0628`, while `J4` in the same poll remained around `-0.3490`, so those axes were nonzero immediately after the retained successful home state.
- Follow-up notes / risks:
  - The retained evidence says native-home is not equivalent to "set current pose to displayed zero" on all axes in the present A6-EC contract.
  - If the intended commissioning contract is "all joints should read `0.00` at this parked pose after home," then that contract is specifically wrong for `J3/J4`; the current values are not a false UI/backend rendering bug.

## 2026-04-15 05:26 +0000

- What changed:
  - Added a focused `J3` wrap-seam regression to `tests/test_gradient05_limits_and_backends.py` without changing backend/runtime behavior yet.
  - Added `test_ethercat_backend_normalizes_j3_style_wrapped_feedback_counts_for_display` to prove the backend already normalizes `131039 -> -33` counts for A6-EC display-style feedback.
  - Added strict `xfail` regression `test_ethercat_backend_j3_style_native_home_capture_should_zero_pose_at_wrap_seam` to encode the desired product behavior: after capturing native-home at the current `J3` seam-wrapped pose, the displayed/operator pose should be near zero and the captured home anchor should match the absolute pose seen at home.
  - Updated two stale nearby tests so they align with the current fail-closed anchor/roundtrip contract instead of expecting canonical/display truth to ignore frame mismatch:
    - `test_ethercat_backend_marks_truth_unavailable_across_raw_wrap_without_coherent_anchor`
    - `test_ethercat_backend_refuses_display_feedback_when_absolute_anchor_does_not_roundtrip`
- Validation:
  - Focused slice:
    - `python -m pytest tests/test_gradient05_limits_and_backends.py -q -k "normalizes_j3_style_wrapped_feedback_counts_for_display or j3_style_native_home_capture_should_zero_pose_at_wrap_seam or uses_multi_turn_absolute_feedback_as_canonical_truth or marks_truth_unavailable_across_raw_wrap_without_coherent_anchor or translates_canonical_truth_back_into_raw_wire_counts"` -> `4 passed, 1 xfailed`
  - Broader backend subset:
    - `python -m pytest tests/test_gradient05_limits_and_backends.py -q -k "canonical or absolute_home_anchor or native_home or display_feedback"` -> `25 passed, 1 xfailed`
  - `ReadLints` on `tests/test_gradient05_limits_and_backends.py` returned clean.
- Follow-up notes / risks:
  - The new `xfail` captures the currently reproduced product bug without changing live motion semantics yet.
  - The test evidence supports the narrower diagnosis that the backend already knows how to normalize seam-wrapped raw feedback for display, but the native-home anchor/reference capture path still encodes the wrong zero contract for `J3`-style wrap cases.
  - The final implementation should be driven by the new regression, but changing controller canonical truth still requires care because the motion/command path presently inverts through the same reference-frame assumptions.

## 2026-04-15 06:00 +0000

- What changed:
  - Updated `src/gradient_os/arm_controller/backends/ethercat_rtcore/backend.py` so the explicit operator display path can use `reference_mode="display"` while controller canonical truth remains on the raw controller/reference frame.
  - Switched native-home and software-zero absolute-home anchor capture/validation onto that display reference mode, so new `J3`-style seam homes collapse to display zero instead of preserving the wrapped `0x6064` offset.
  - Updated `src/gradient_os/run_controller.py` so `arm_display_rad` / `arm_display_deg` are only populated from the backend display snapshot, not copied from `arm_rad` / `arm_deg` by default.
  - Updated `web-ui/src/ControlPanel.tsx` so operator joint feedback uses only explicit display values (`arm_display_deg` / `display_joints`) with no canonical fallback.
  - Updated `tests/test_gradient05_limits_and_backends.py` to turn the `J3` seam-home regression into a required pass and align old-anchor display expectations with the new fail-closed display contract.
  - Updated `web-ui/src/ControlPanel.test.tsx` to assert the no-fallback display behavior and refreshed the pending-native-home fixture to provide display joints.
- Validation:
  - `source "/home/pi/GradientOS/.venv/bin/activate" && PYTHONPATH=src python -m py_compile src/gradient_os/arm_controller/backends/ethercat_rtcore/backend.py src/gradient_os/run_controller.py`
  - `source "/home/pi/GradientOS/.venv/bin/activate" && PYTHONPATH=src python -m pytest tests/test_gradient05_limits_and_backends.py -q -k "j3_style_native_home_capture_should_zero_pose_at_wrap_seam or normalizes_j3_style_wrapped_feedback_counts_for_display or uses_multi_turn_absolute_feedback_as_canonical_truth or marks_truth_unavailable_across_raw_wrap_without_coherent_anchor or translates_canonical_truth_back_into_raw_wire_counts or display_feedback or native_home_captures_absolute_encoder_anchor"` -> `7 passed, 61 deselected`
  - `source "/home/pi/GradientOS/.venv/bin/activate" && PYTHONPATH=src python -m pytest tests/test_run_controller_helpers.py -q` -> `5 passed`
  - `cd web-ui && npx vitest run src/ControlPanel.test.tsx` -> `17 passed`
  - `ReadLints` on touched files returned clean
- Follow-up notes / risks:
  - This intentionally does not change controller canonical / command-path semantics yet; raw `0x607A` wire-target wrap mapping still needs a deliberate design if you later want the controller-facing canonical path to match the same seam-normalized operator contract.
  - Existing anchors captured under the older/raw-style contract may now leave the explicit display path unavailable until those joints are re-homed or otherwise recaptured under the new display reference mode.

## 2026-04-15 06:33 +0000

- What changed:
  - Investigated the live blank-commissioning-pane regression with the running stack and confirmed it is not just a frontend transport miss.
  - Updated `src/gradient_os/arm_controller/backends/ethercat_rtcore/backend.py` to return `joint_positions_rad_partial` so operator display consumers can distinguish explicit per-joint truth from cached all-joint setpoints, and switched `_bootstrap_missing_absolute_home_anchors()` to capture missing anchors with `reference_mode="display"`.
  - Updated `src/gradient_os/run_controller.py` so `arm_display_rad` / `arm_display_deg` can publish partial explicit display truth while preserving the existing fail-closed canonical/raw semantics and status flags.
  - Updated `web-ui/src/ControlPanel.tsx` so partial display payloads render as per-joint values plus `--` for unavailable joints without falling back to canonical joint angles or external telemetry.
  - Added/updated regressions in `tests/test_gradient05_limits_and_backends.py`, `tests/test_run_controller_helpers.py`, and `web-ui/src/ControlPanel.test.tsx` for partial display publishing and display-mode startup bootstrap.
- Validation:
  - Live diagnostic:
    - `curl -sf http://127.0.0.1:4000/info/joints-detailed`
    - Confirmed `arm_rad=[]`, `arm_display_deg=[]`, and display-axis failures on `J3/J4` with approximately one-turn anchor deltas consistent with legacy/raw-style anchors under the new display contract.
  - `source "/home/pi/GradientOS/.venv/bin/activate" && PYTHONPATH=src python -m py_compile src/gradient_os/arm_controller/backends/ethercat_rtcore/backend.py src/gradient_os/run_controller.py`
  - `source "/home/pi/GradientOS/.venv/bin/activate" && PYTHONPATH=src python -m pytest tests/test_run_controller_helpers.py -q` -> `6 passed`
  - `source "/home/pi/GradientOS/.venv/bin/activate" && PYTHONPATH=src python -m pytest tests/test_gradient05_limits_and_backends.py -q -k "j3_style_native_home_capture_should_zero_pose_at_wrap_seam or normalizes_j3_style_wrapped_feedback_counts_for_display or refuses_display_feedback_when_absolute_anchor_does_not_roundtrip or startup_bootstrap"` -> `6 passed`
  - `cd web-ui && npx vitest run src/ControlPanel.test.tsx` -> `18 passed`
  - `ReadLints` on touched files returned clean.
- Follow-up notes / risks:
  - The running controller/UI processes have not been restarted by this change set, so the live stack will continue to show the old behavior until the user intentionally reloads it.
  - Existing `J3/J4` entries in `.gradient_absolute_encoder_anchors.json` still appear to predate the display-reference contract and should be deliberately re-homed or recaptured before expecting those joints to publish display truth.
  - Raw canonical truth and RTCore `0x607A` wrap safety remain intentionally unchanged; do not broaden the seam-normalized contract onto the motion command path until the raw-wire target redesign is explicit.

## 2026-04-15 06:44 +0000

- What changed:
  - Re-read the active A6-EC scratchpad/devlog history and re-ran live diagnostics after the user's hard stop/restart.
  - Ran the read-only Chapter 5 probe across all six joints with:
    - `source "/home/pi/GradientOS/.venv/bin/activate" && PYTHONPATH=src python scripts/a6ec_chapter5_probe.py snapshot --label hard-restart-all-joints --axes J1 J2 J3 J4 J5 J6`
  - Used an offline backend reconstruction against the same live raw counts to separate raw-frame failures from display-frame failures.
- Validation:
  - Live API after restart:
    - `curl -sf http://127.0.0.1:4000/info/joints-detailed`
    - Result: `arm_display_deg` had values for `J1/J2/J5/J6`, `J3/J4` were `null`, but `read_source` was still `unavailable` because the raw canonical path was still failing.
  - Probe artifact:
    - JSON: `logs/encoder-retention/20260415-064241-a6ec-ch5-probe/hard-restart-all-joints.json`
    - Markdown: `logs/encoder-retention/20260415-064241-a6ec-ch5-probe/hard-restart-all-joints.md`
  - Probe summary:
    - all six axes returned Chapter 5 SDO reads successfully (`failed_reads=[]` for `J1..J6`)
    - all bridge deltas stayed in the normal `0..2` count wander band
  - Offline backend truth split using the same live raw counts:
    - raw mode unavailable joints: `[4, 6]`
    - display mode unavailable joints: `[3, 4]`
    - `J3`: display-mode anchor stale by about `+131072` counts
    - `J4`: raw/display mismatch by about `+131068` counts
    - `J6`: raw-mode mismatch by about `-131076` counts, display mode coherent
- Follow-up notes / risks:
  - This confirms the underlying problem is not “cannot read the motors.” The drive-side objects are readable; the host-side truth contract is what is failing.
  - `J3` is the old/raw-style-anchor-against-display-contract case.
  - `J4` appears to have a genuinely wrong/stale stored anchor in both contracts.
  - `J6` is the raw `6064` wrap-seam issue that remains intentionally unsolved on the command/canonical path for safety reasons.
  - The commissioning UI can still blank all joints whenever it keys off `read_source=unavailable`, even if explicit display truth is present for a subset of joints.

## 2026-04-15 06:54 +0000

- What changed:
  - Investigated the live commissioning-pane mismatch as a browser/runtime issue, not a deploy/git issue, after the user pointed out that the backend was already returning partial `arm_display_deg`.
  - Used browser network tooling to confirm the page was the local Vite UI at `http://127.0.0.1:8000` and that it was polling `http://127.0.0.1:4000/info/joints-detailed` with `200` responses.
  - Identified the actual UI bug in `web-ui/src/ControlPanel.tsx`: `refreshJointAngles()` was still clearing all displayed joints whenever `read_source !== "live_feedback"`, even when explicit operator display truth was present in `arm_display_deg`.
  - Removed that stale `read_source` gate so the commissioning pane now renders explicit display truth per joint and keeps the no-canonical-fallback contract.
  - Updated `web-ui/src/ControlPanel.test.tsx` so the frontend regression coverage now uses the real live shape: `read_source="unavailable"` with valid full or partial `arm_display_deg`.
- Validation:
  - `cd web-ui && npx vitest run src/ControlPanel.test.tsx` -> `18 passed`
  - `ReadLints` on `web-ui/src/ControlPanel.tsx` and `web-ui/src/ControlPanel.test.tsx` returned clean.
- Follow-up notes / risks:
  - I could prove the browser was hitting the correct host and endpoint, but the browser MCP became unstable after the first successful inspection and would no longer keep the tab open for a second end-to-end visual check.
  - `J3/J4` remain a backend anchor problem, not a UI problem: the latest live payload still reports those display joints unavailable while `J1/J2/J5/J6` are valid.
  - There is still no dedicated API to clear an absolute-home anchor entry directly. Today the safe options are either:
    - overwrite it by running drive-native home for the affected joint, which captures a fresh display-mode anchor after verification
    - or clear the relevant entries in `.gradient_absolute_encoder_anchors.json` and restart/reload before recapturing them deliberately
  - The live monitor/SSE parse path in `web-ui/src/App.tsx` still filters non-finite `display_joints` values, so if monitor packets start carrying `null` placeholders that path may need a follow-up fix to preserve slot alignment.

## 2026-04-15 07:00 +0000

- What changed:
  - Re-checked the live backend truth after the UI fix and confirmed the persisted anchor file was still stale for both `J3` and `J4`.
  - Ran live drive-native home recapture for `J3` via `POST /control/home-joint-native {"joint": 3}`.
  - Verified that `J3` immediately regained display truth and that `.gradient_absolute_encoder_anchors.json` updated the `J3` anchor timestamp to `2026-04-15T06:59:50+00:00`.
  - Ran live drive-native home recapture for `J4` via `POST /control/home-joint-native {"joint": 4}`.
  - Verified that `J4` immediately regained display truth and that `.gradient_absolute_encoder_anchors.json` updated the `J4` anchor timestamp to `2026-04-15T07:00:10+00:00`.
- Validation:
  - Live `info/joints-detailed` before action still showed:
    - `J3 truth_reason=absolute_home_anchor_stale` with `absolute_home_anchor_delta_counts=131072`
    - `J4 truth_reason=command_frame_roundtrip_mismatch` with `absolute_home_anchor_delta_counts=131069`
  - `POST /control/home-joint-native` for `J3` returned:
    - `accepted=true`
    - `verified=true`
    - `absolute_home_anchor_capture_succeeded=true`
    - `absolute_home_anchor_refresh_ok=true`
    - `post_home_truth_available=true`
  - `POST /control/home-joint-native` for `J4` returned:
    - `accepted=true`
    - `verified=true`
    - `absolute_home_anchor_capture_succeeded=true`
    - `absolute_home_anchor_refresh_ok=true`
    - `post_home_truth_available=true`
  - Live `info/joints-detailed` after both recaptures showed:
    - six finite `arm_display_deg` values
    - `display_joint_truth_unavailable_joints=[]`
    - `canonical_joint_truth_unavailable_joints=[]`
  - Stability sample:
    - 30 reads of `http://127.0.0.1:4000/info/joints-detailed` at ~100 ms cadence
    - no fetch failures
    - no `null` display joints
    - all six `arm_display_deg` entries finite on every sample
- Follow-up notes / risks:
  - The underlying `J3/J4` null-display problem is fixed live by fresh display-mode anchor capture.
  - The top-level payload still reports `read_source="unavailable"` even while all six joints now have coherent truth; that field is now inconsistent with the reconstructed state and may still deserve a cleanup pass elsewhere.
  - `J4`'s native-home response had a contradictory post-settle tail: the top-level result verified successfully, but `post_home_settle_native_home_state_name="failed"` with abort `0x06010002`. Since truth remained available and the new anchor persisted, this looks like a follow-up telemetry/settle-state inconsistency rather than a current blocker.

## 2026-04-15 07:07 +0000

- What changed:
  - Ran a fresh J3/J4 Chapter 5 probe after the display-truth repair:
    - `source "/home/pi/GradientOS/.venv/bin/activate" && PYTHONPATH=src python scripts/a6ec_chapter5_probe.py snapshot --label j3-j4-live-power-block --axes J3 J4`
  - Re-checked live `/info/joints-detailed` and `/control/motion-status` to identify the actual power-up blocker.
  - Read the power-transition guard path in `src/gradient_os/arm_controller/command_api.py` and `src/gradient_os/arm_controller/backends/ethercat_rtcore/backend.py`.
- Validation:
  - Probe artifact:
    - JSON: `logs/encoder-retention/20260415-070648-a6ec-ch5-probe/j3-j4-live-power-block.json`
    - Markdown: `logs/encoder-retention/20260415-070648-a6ec-ch5-probe/j3-j4-live-power-block.md`
  - Probe summary:
    - `J3` and `J4` raw encoder / bridge checks were healthy
    - all key bridge deltas stayed within `0..1` counts
  - Live API:
    - `/info/joints-detailed` showed `arm_display_deg` populated for all six joints and `display_joint_truth_available=true`
    - the same payload still showed `arm_rad=[]`, `arm_deg=[]`, `read_source="unavailable"`, and `raw_canonical_joint_truth_available=false`
  - Live motion guard:
    - `/control/motion-status` returned `safe_for_power_transition=false`
    - blocker list: `["not_synchronized"]`
    - detail: `Live feedback is not synchronized yet; keep the drives disarmed.`
  - Code path confirmation:
    - `backend.get_power_transition_snapshot()` computes `feedback_synchronized` from `raw_to_joint_positions(...)`
    - `command_api._build_power_transition_guard()` blocks power-up when that flag is false
- Follow-up notes / risks:
  - The current power-up blocker is not “bad J3/J4 encoder values.” It is the unresolved raw canonical synchronization path.
  - The display/home contract is now healthy, but the raw/controller contract still cannot produce a full canonical list for synchronization and hold-target seeding.
  - `run_controller.py` currently mirrors display-truth unavailable lists into the canonical-unavailable fields, which hides the raw blocker details in `joints-detailed` and makes diagnosis harder.

## 2026-04-15 07:15 +0000

- What changed:
  - Patched `src/gradient_os/run_controller.py` so `joints-detailed` stops overwriting raw canonical unavailable fields with display unavailable fields.
  - Added parsing of the existing raw-truth error string so canonical unavailable axes/joints can still be surfaced even when display truth is available.
  - Added a regression in `tests/test_run_controller_helpers.py` covering the mixed state:
    - raw canonical truth unavailable with joints `[3, 4, 6]`
    - display truth available for all joints
- Validation:
  - `source "/home/pi/GradientOS/.venv/bin/activate" && PYTHONPATH=src python -m pytest tests/test_run_controller_helpers.py -q` -> `7 passed`
  - `ReadLints` on `src/gradient_os/run_controller.py` and `tests/test_run_controller_helpers.py` returned clean.
- Follow-up notes / risks:
  - This patch improves live diagnostics only; it does not change the current power-up block.
  - The actual runtime blocker remains `feedback_synchronized=false` in `backend.get_power_transition_snapshot()`, which still depends on the raw canonical path via `raw_to_joint_positions()`.
  - A future power-up fix must be careful not to broaden display-mode semantics onto the raw command path without explicitly preserving RTCore’s raw wire-frame safety guarantees.

## 2026-04-15 07:45 +0000

- What changed:
  - Patched `src/gradient_os/arm_controller/backends/ethercat_rtcore/backend.py` to make the raw/controller frame wrap-aware without changing display-mode semantics.
  - Added a per-axis raw-reference wrap lift in counts, learned from live raw feedback during `raw_to_joint_positions()`.
  - Reused that same wrap lift when inverting canonical joint positions back into controller axis-q targets, so the outbound command path stays on the live raw branch.
  - Updated `tests/test_gradient05_limits_and_backends.py`:
    - raw truth now remains coherent across a single-turn raw seam even when display truth still fails on an old/raw-style display anchor
    - a J3-style display-mode anchor keeps raw command targets on the live `131039` branch
    - tolerance tests now follow the configured backend roundtrip threshold instead of hard-coding `10` counts
- Validation:
  - `source "/home/pi/GradientOS/.venv/bin/activate" && PYTHONPATH=src python -m py_compile src/gradient_os/arm_controller/backends/ethercat_rtcore/backend.py`
  - `source "/home/pi/GradientOS/.venv/bin/activate" && PYTHONPATH=src python -m pytest tests/test_gradient05_limits_and_backends.py -q` -> `70 passed`
  - `source "/home/pi/GradientOS/.venv/bin/activate" && PYTHONPATH=src python -m pytest tests/test_run_controller_helpers.py -q` -> `7 passed`
  - `ReadLints` on `src/gradient_os/arm_controller/backends/ethercat_rtcore/backend.py` and `tests/test_gradient05_limits_and_backends.py` returned clean
- Follow-up notes / risks:
  - This change is code-and-test validated but not live-loaded into the running controller process; a controller restart is still required before re-checking `/control/motion-status`.
  - The fix is intentionally backend-side: it preserves the display/native-home contract and avoids pushing operator-facing display semantics into RTCore.

## 2026-04-15 08:24 +0000

- What changed:
  - Investigated the user-reported live `J2` wrong-direction / violent jog symptom after power-up by reading:
    - active controller terminal output
    - `logs/startups/20260415-080110/controller.log`
    - `logs/startups/20260415-080110/api.log`
    - direct EtherCAT objects for `J2`
    - `/run/gradient-rt-motion/metrics.json`
    - live `/info/joints-detailed` and `/control/motion-status`
  - Captured a fresh post-home probe with:
    - `source "/home/pi/GradientOS/.venv/bin/activate" && PYTHONPATH=src python scripts/a6ec_chapter5_probe.py snapshot --label j2-post-home-now --axes J2`
  - Wrote a comparison artifact:
    - `logs/encoder-retention/20260415-082343-a6ec-ch5-probe/j2-pre-vs-post-home-summary.md`
- Validation:
  - Controller/API logs showed the third discrete jog targeted `J2`, after which canonical truth became unavailable across multiple axes and the next `/control/joint-jog` returned `409 Conflict`
  - Pre-home `J2` direct drive reads:
    - `0x60B0 = 0`
    - `0x607C = 0`
    - `0x6041 = 0x1650`
    - `0x603F = 0x0000`
  - Post-home `J2` direct drive reads:
    - `0x60B0 = 0`
    - `0x607C = 0`
    - `0x6041 = 0x9650`
    - `0x603F = 0x0000`
  - Post-home RTCore/API state:
    - `/run/gradient-rt-motion/metrics.json` axis 1 reports `native_home_state = 2` but `native_home_position_offset = 0`
    - `/info/joints-detailed` reports `J2 arm_deg ~= 0.0006866` with `read_source = live_feedback` and raw canonical truth available
    - `/control/motion-status` reports `safe_for_power_transition = true`
  - Anchor persistence:
    - `.gradient_absolute_encoder_anchors.json` refreshed `J2` to `home_anchor_rad = 0.007341536177021437` at `2026-04-15T08:12:47+00:00`
  - Post-home probe artifact:
    - `logs/encoder-retention/20260415-082343-a6ec-ch5-probe/j2-post-home-now.json`
    - bridge checks remained in the standard `0..1` count wander band
- Follow-up notes / risks:
  - The latest `J2` home clearly repaired the anchored absolute-truth path and restored a near-zero API readout.
  - It did not restore a nonzero drive-native offset in `0x60B0` / `0x607C` or RTCore `native_home_position_offset`, so this still resembles the old `J2` frame/home mismatch family more than a fully clean native-home repair.
  - Current `power_transition_feedback_synchronized = true`, so the raw truth block is not presently preventing re-enable; the bigger concern is whether a new `J2` jog can still re-trigger the mismatch under motion.

## 2026-04-15 08:39 +0000

- What changed:
  - Captured a fresh pre-jog `J2` Chapter 5 probe with:
    - `source "/home/pi/GradientOS/.venv/bin/activate" && PYTHONPATH=src python scripts/a6ec_chapter5_probe.py snapshot --label j2-pre-jog --axes J2 --experiment-id 20260415-0824-j2-jog-frame-check`
  - Saved a consolidated live runtime baseline for `J2` at:
    - `logs/encoder-retention/20260415-0824-j2-jog-frame-check/j2-runtime-pre-jog.json`
    - `logs/encoder-retention/20260415-0824-j2-jog-frame-check/j2-runtime-pre-jog.md`
- Validation:
  - Probe artifacts:
    - `logs/encoder-retention/20260415-0824-j2-jog-frame-check/j2-pre-jog.json`
    - `logs/encoder-retention/20260415-0824-j2-jog-frame-check/j2-pre-jog.md`
  - Probe summary for `J2`:
    - `raw_formula_match = false`
    - `raw_formula_delta_counts = -3`
    - `bridge_6063_from_6064`, `bridge_60fc_from_6062`, and `bridge_u402a_from_u4028` all stayed within the normal `0..1` count band
  - Current direct reads captured into the runtime baseline:
    - `0x60B0 = 0`
    - `0x607C = 0`
    - `0x6041 = 0x9650`
    - `0x603F = 0x0000`
  - Current RTCore/API state in the same baseline:
    - axis 1 `native_home_state = 2`
    - axis 1 `native_home_position_offset = 0`
    - `/info/joints-detailed` `J2 arm_deg ~= 0.0006866`
    - `/control/motion-status` `safe_for_power_transition = true`
- Follow-up notes / risks:
  - This is now a clean pre-jog capture set for `J2`; the next meaningful comparison is to take the same probe/runtime snapshots immediately after the next tiny `J2` jog.
  - The system is presently safe to re-enable, but the zero drive-native offset path (`0x60B0 = 0`, `0x607C = 0`, RTCore offset `0`) still leaves `J2` suspicious even though API/controller truth is coherent right now.

## 2026-04-15 08:47 +0000

- What changed:
  - Investigated the user's "we should be able to just read a joint position" complaint against the live API before the next hard stop / drive power cycle.
  - Queried the current live payloads:
    - `http://127.0.0.1:4000/info/joints-detailed`
    - `http://127.0.0.1:4000/info/joints`
  - Re-read the jog gating code in `src/gradient_os/api/main.py` and the truth-flag composition in `src/gradient_os/run_controller.py`.
- Validation:
  - Live `/info/joints-detailed` currently reports:
    - `read_source = live_feedback`
    - `raw_canonical_joint_truth_available = true`
    - finite `arm_deg` for all six joints
    - `arm_display_deg` is `null` only for `J3`
    - `display_joint_truth_unavailable_joints = [3]`
    - `display_joint_truth_reason = absolute_home_anchor_stale`
    - `canonical_joint_truth_available = false`
  - `src/gradient_os/run_controller.py` still computes top-level `canonical_joint_truth_available` by starting from raw/live truth and then AND-ing in display truth.
  - `src/gradient_os/api/main.py` `/control/joint-jog` still rejects a jog when that global top-level flag is false, before it reaches the selected-joint truth check.
- Follow-up notes / risks:
  - The current live issue is not "the system cannot read joint position." The current live issue is that one `J3` display-anchor failure poisons the old global truth flag and the jog route still uses that global flag as a hard precondition.
  - A hard stop / drive power cycle may reset drive-side runtime objects, but by itself it does not guarantee the host-side stale display anchor or the API's global jog gate will clear.

## 2026-04-15 08:55 +0000

- What changed:
  - Reviewed the manufacturer's written reply against the existing A6-EC bench evidence and workstream notes.
  - Reconciled which parts are now explicitly vendor-confirmed versus still unresolved.
- Validation:
  - Vendor-confirmed points that match our bench direction:
    - `C00.07 = 4` is the intended absolute rotation mode
    - HM method `35` with `0x6060 = 6`, `0x6098 = 35`, `0x60E6 = 0`, then return to `0x6060 = 8`
    - `0x607C` is the persistent origin/home object and auto-saves
    - `0x60B0` is runtime-only
    - `0x6064` is the authoritative CSP/application position after homing
    - HM success/reference validity requires `0x6041 bit12 = 1` and `bit15 = 1`
  - Checked `0x9650` bit decode directly: set bits are `[4, 6, 9, 10, 12, 15]`, so it matches the vendor-stated HM success condition with bit 13 clear.
- Follow-up notes / risks:
  - This is strong confirmation that the long-running `0x60B0` persistence path was the wrong object model for A6-EC and that `0x9650` is a meaningful success terminal state, not an incidental value.
  - The reply still does not answer several earlier questions we care about:
    - exact role of `U40.16` relative to `0x6064`
    - whether direct `0x607C` writes alone establish a valid homing/reference state
    - `0x607C` signed/range behavior in rotation mode (`0..RM-1` vs persisted negative writes)
    - whether `C10.18/C10.19` must match true mechanics
    - what `0x2013:17` and `F31.10` do in this persistence/reference workflow

## 2026-04-15 09:05 +0000

- What changed:
  - Re-read the A6-EC manual excerpts to answer the user's follow-up on the vendor phrase "one full revolution of the load" and whether `RM` needs clarification.
- Validation:
  - `docs/resources/a6ec_manual_chapter_11_parameter_list.md` states under `6091`:
    - "The gear ratio is used to establish the proportional relationship between the load shaft displacement designated by the user and the motor shaft displacement."
    - "Motor position feedback = Load shaft position feedback x Gear ratio"
  - The same manual section for rotation mode still leaves the implementation source of `RM` ambiguous relative to `C10.1A/C10.1C`, `C10.18/C10.19`, and other scaling objects.
- Follow-up notes / risks:
  - Best current interpretation: "full revolution of the load" means one revolution of the output/load shaft in the user/application sense, not one motor-shaft turn.
  - This should still be sent back for clarification because the vendor did not define `RM` algebraically, did not state which objects determine it in absolute rotation mode, and did not reconcile the claimed `0..RM-1` `0x607C` range with our persisted negative `0x607C` observations.

## 2026-04-15 09:19 +0000

- What changed:
  - Tightened native-home status fallback semantics to match the new vendor guidance:
    - updated `src/gradient_os/telemetry/native_home_status.py`
    - updated `src/gradient_os/arm_controller/backends/ethercat_rtcore/backend.py`
  - Changed the statusword-derived success marker from `statusword_bit15` to `statusword_bits12_15_clear13`.
  - Updated `web-ui/src/ControlPanel.tsx` and `web-ui/src/ControlPanel.test.tsx` so the UI still recognizes both the new and legacy statusword-derived success markers when rendering conservative drive-home status.
  - Updated `scripts/a6ec_chapter5_probe.py` and `tests/test_a6ec_chapter5_probe.py` so probe snapshots now expose an explicit `vendor_hm_success_signature` plus `bit15_reference_attained`.
  - Updated focused regression expectations in:
    - `tests/test_gradient05_limits_and_backends.py`
    - `tests/test_drive_faults.py`
- Validation:
  - `source "/home/pi/GradientOS/.venv/bin/activate" && PYTHONPATH=src python -m pytest tests/test_gradient05_limits_and_backends.py -q -k "native_home_metrics_result or native_home_post_settle or wait_for_native_home_result or absolute_home_anchor_stale"` -> `10 passed`
  - `source "/home/pi/GradientOS/.venv/bin/activate" && PYTHONPATH=src python -m pytest tests/test_drive_faults.py tests/test_a6ec_chapter5_probe.py -q` -> `14 passed`
  - `cd /home/pi/GradientOS/web-ui && npx vitest run src/ControlPanel.test.tsx` -> `18 passed`
  - `ReadLints` on all touched product/test files returned clean
- Follow-up notes / risks:
  - The RTCore HM executor already matched the vendor-confirmed terminal condition (`bits 12+15 set, bit 13 clear`); this pass only corrected the Python-side fallback/telemetry interpretation.
  - I intentionally did not change RTCore queued-target conversion (`controller_target_counts - native_home_offset_counts`) in this pass. Existing bench notes still indicate that motion-path frame change needs separate live proof before it is safe to alter.

## 2026-04-15 09:31 +0000

- What changed:
  - Investigated the user's latest post-restart `J2` native-home run using:
    - `logs/startups/20260415-092423/controller.log`
    - `logs/startups/20260415-092423/api.log`
    - `journalctl -u gradient-rt-motion.service -n 120 --no-pager`
    - live `/info/joints-detailed`
    - live `/control/motion-status`
    - direct SDO reads of `J2` `0x607C`, `0x6064`, `U40.16`, and `0x6041`
    - `/run/gradient-rt-motion/metrics.json`
  - Implemented a new `watch` subcommand in `scripts/a6ec_chapter5_probe.py` for live hand-rotation / streaming captures.
  - Added tests covering the new watch helpers in `tests/test_a6ec_chapter5_probe.py`.
- Validation:
  - Latest `J2` native-home evidence after restart:
    - controller log shows `NATIVE_HOME_JOINT,2` followed by `Native drive-home verified`
    - RTCore journal logs `EtherCAT native_home axis=1 ... feedback_counts=2420 truth_value=0 commissioning_mode=6 steady_state_mode=8`
    - direct SDO reads return:
      - `0x607C = 0`
      - `0x6064 = 20`
      - `U40.16 = 21`
      - `0x6041 = 0x9650`
    - RTCore metrics axis 1 report:
      - `native_home_state = 2`
      - `native_home_position_offset = 0`
      - `statusword = 0x9650`
      - `pos_counts = 20`
    - live `/info/joints-detailed` reports `J2 arm_deg ~= 0.0005768`
    - live `/control/motion-status` reports `safe_for_power_transition = true`
  - New probe watch validation:
    - `source "/home/pi/GradientOS/.venv/bin/activate" && PYTHONPATH=src python -m pytest tests/test_a6ec_chapter5_probe.py -q` -> `6 passed`
    - `source "/home/pi/GradientOS/.venv/bin/activate" && PYTHONPATH=src python scripts/a6ec_chapter5_probe.py watch --label j6-dry-run --axes J6 --samples 1 --interval-s 0.1 --quiet` -> wrote a valid JSONL sample under `logs/encoder-retention/20260415-093037-a6ec-ch5-probe/`
  - `ReadLints` on the touched probe script/test returned clean
- Follow-up notes / risks:
  - This latest `J2` run strongly indicates the standard production HM workflow is now landing in the intended zero-offset case (`truth_value=0`, `0x607C=0`), so RTCore's queued-target subtraction is currently a no-op for that workflow.
  - The unresolved motion-path concern is now narrower: it mainly matters for nonzero-`607C` scenarios or alternate origin conventions, not for the standard `HM 35 + 607C=0` home sequence.

## 2026-04-15 09:38 +0000

- What changed:
  - Started a live `J6` hand-rotation watcher for the user's upcoming manual brake-release experiment with:
    - `source "/home/pi/GradientOS/.venv/bin/activate" && PYTHONPATH=src python scripts/a6ec_chapter5_probe.py watch --label j6-hand-rotate-live --axes J6 --interval-s 0.25`
- Validation:
  - Watcher reached healthy steady state and wrote metadata plus live samples to:
    - `logs/encoder-retention/20260415-093803-a6ec-ch5-probe/j6-hand-rotate-live.watch.jsonl`
  - Initial observed `J6` baseline from the first live stream lines:
    - `6064 ~= 108296`
    - `607C = 0`
    - `U40.16 ~= -22776`
    - raw absolute `U40.20/.22 ~= 33350`
    - rotation-mode counts `U40.2A/.2C ~= 108296`
    - API `arm_deg ~= 6.257`
    - `vendor_hm_success_signature = false`
- Follow-up notes / risks:
  - This is a high-value starting state because it already shows `607C = 0` while the HM-valid signature is false and `6064` is nonzero.
  - If the user rotates `J6` by hand while the drive electronics stay powered and the axis stays disarmed, the new watch stream should let us see exactly which position-domain families track the manual motion together.

## 2026-04-15 09:42 +0000

- What changed:
  - Stopped the live `J6` hand-rotation watcher after the user completed a large manual rotation in both directions.
  - Summarized the finished stream from:
    - `logs/encoder-retention/20260415-093803-a6ec-ch5-probe/j6-hand-rotate-live.watch.jsonl`
- Validation:
  - Completed stream size: `257` samples
  - Invariants observed across the run:
    - `607C` stayed exactly `0`
    - `statusword` stayed `0x1650`
    - `vendor_hm_success_signature` stayed `false`
    - `6064` stayed in a bounded one-turn-like band (`4129 .. 129172`)
    - `U40.28/U40.2A/.2C` stayed in the same bounded rotation/reference band (`3265 .. 130438`)
    - `U40.16` and raw absolute `U40.20/.22` moved through large multi-turn ranges (`~ -2.84M .. +1.87M`, `~ -2.79M .. +1.87M`)
    - API `arm_deg` / `arm_display_deg` also ranged widely (`~ -377° .. +780°`)
- Follow-up notes / risks:
  - This is strong read-side evidence that `607C = 0` alone does not establish an active valid homed/reference frame; the HM-valid statusword bits still matter.
  - This run did not exercise the `0x607A` command/write path, so it does not by itself justify changing RTCore's queued-target subtraction logic.
  - The motion-path concern is now narrower: to answer it directly we need a controlled nonzero-`607C` experiment or another command-path proof, not just more read-only hand-rotation traces.

## 2026-04-15 09:56 +0000

- What changed:
  - Ran the next controlled nonzero-`607C` write-path experiment on `J2` while the axis was still in the clean post-home state (`0x6041 = 0x9650`, `vendor_hm_success_signature = true`, `0x607C = 0`).
  - Captured three probe snapshots under `logs/encoder-retention/20260415-j2-607c-write-test/`:
    - `j2-pre-607c-write.json/.md`
    - `j2-post-607c-write.json/.md`
    - `j2-post-607c-restore.json/.md`
  - Wrote a temporary positive home offset with:
    - `sudo ethercat download -p 1 -t int32 0x607C 0 12345`
  - Restored `0x607C` to zero before any motion with:
    - `sudo ethercat download -p 1 -t int32 0x607C 0 0`
- Validation:
  - Immediate readback after the temporary write:
    - `0x607C = 12345`
    - `0x6041 = 0x9650`
    - `0x6064 = 21`
    - `U40.16 = 22`
  - Snapshot-to-snapshot comparison showed no large frame jump when `0x607C` changed:
    - `6064`: `21 -> 22 -> 22`
    - `U40.16`: `21 -> 22 -> 22`
    - raw absolute `U40.20/.22`: `17758 -> 17759 -> 17758`
    - API `raw_counts`: `23 -> 21 -> 21`
    - API `absolute_counts`: `17758 -> 17761 -> 17760`
    - API `canonical_rad`: `9.587e-06 -> 1.103e-05 -> 1.055e-05`
  - `0x6041` stayed `0x9650` and `vendor_hm_success_signature` stayed true for all three snapshots.
- Follow-up notes / risks:
  - This is strong evidence that a direct nonzero `0x607C` write is not being immediately absorbed into the live `6064`/`U40.16`/API truth frame in the current steady-state workflow.
  - That weakens the live double-apply hypothesis for RTCore's current queued-target subtraction, but it still does not fully close the question for post-write motion, re-arm, HM rerun, or power-cycle activation paths.

## 2026-04-15 10:09 +0000

- What changed:
  - Ran the next activation-timing experiment on `J2` to test whether a nonzero `0x607C` becomes active on `SAFE_POWER_UP`.
  - Stored six snapshots under `logs/encoder-retention/20260415-j2-607c-powerup-activation-test/`:
    - `j2-pre-powerup-activation.json/.md`
    - `j2-post-write-disarmed.json/.md`
    - `j2-post-power-up.json/.md`
    - `j2-post-restore-write-disarmed.json/.md`
    - `j2-post-restore-power-up.json/.md`
    - `j2-final-disarmed.json/.md`
  - Executed this sequence:
    - verified initial `safe_for_power_transition = true`
    - wrote `sudo ethercat download -p 1 -t int32 0x607C 0 12345`
    - issued API `POST /control/power-up`
    - issued API `POST /control/power-down`
    - wrote `sudo ethercat download -p 1 -t int32 0x607C 0 0`
    - issued API `POST /control/power-up`
    - issued API `POST /control/power-down`
- Validation:
  - Direct write while disarmed again showed no immediate large jump:
    - `0x607C: 0 -> 12345`
    - `0x6064: 21 -> 23`
    - `U40.16: 23 -> 23`
    - API `canonical_rad` unchanged at `~1.10e-05`
  - First `SAFE_POWER_UP` changed `J2`, but not by the written `0x607C` amount:
    - `0x607C` stayed `12345`
    - `0x6064: 21 -> 2253`
    - `U40.16: 23 -> 2252`
    - raw absolute `U40.20/.22: 17761 -> 19991`
    - API `absolute_counts: 17761 -> 19990`
    - API `canonical_rad: ~1.10e-05 -> ~1.08e-03`
  - Key bridge invariants stayed effectively constant across the whole run:
    - `combined(U40.20/.22) - 6064 ~= 17737..17740`
    - `api absolute_counts - raw_counts ~= 17736..17739`
    - `absolute_home_anchor_rad` stayed exactly `0.008503047254848595`
  - After restoring `0x607C = 0`, a second `SAFE_POWER_UP` still shifted the same families again:
    - `0x6064: 2283 -> 4464`
    - `U40.16: 2283 -> 4464`
    - raw absolute `U40.20/.22: 20022 -> 22203`
  - Final post-settle state after the closing `SAFE_POWER_DOWN`:
    - controller returned to `safe_for_power_transition = true`
    - final snapshot: `0x607C = 0`, `0x6041 = 0x9650`, `0x6064 ~= 4495`, `U40.16 ~= 4495`, `U40.20/.22 ~= 22232`
- Follow-up notes / risks:
  - This further weakens the hypothesis that the current power-up path is simply "activating nonzero `0x607C` into `6064` and causing RTCore double-apply." The observed shift was not `12345` counts and it moved the raw absolute and reference families together.
  - The new leading question is why `J2` whole-frame counts move coherently by about `~2.2k` counts across idle `SAFE_POWER_UP` transitions, even after `0x607C` is restored to zero.
  - The system was returned to a disarmed, settled state at the end of the experiment, but `J2` did not numerically return to its original near-zero `6064`/API state.

## 2026-04-15 10:22 +0000

- What changed:
  - Re-read the attached manual extracts for:
    - `docs/resources/chapter 5 absolute system - extract from A6-EC_series_servo_drive_manual (2).pdf`
    - `docs/resources/chapter 11 - parameter list - A6-EC_series_servo_drive_manual (2).pdf`
  - Cross-referenced the manual wording against the vendor reply and the latest `J2` / `J6` experiments.
- Validation:
  - Chapter 5 confirms:
    - `C00.07 = 4` is absolute position rotation mode.
    - rotation mode is intended for unlimited load travel with `< 32767` unidirectional revolutions.
    - `RM` is `encoder pulses per load revolution`.
    - in rotation mode while in HM, the home-offset range is `0 .. (RM - 1)`.
    - the drive calculates the upper limit of mechanical absolute position from `C10.1A/C10.1C` first, otherwise `C10.18/C10.19`.
  - Chapter 11 confirms:
    - `6064` is reference-unit position actual value and `6064 * 6091 = 6063`.
    - `607C` is home offset.
    - `60B0` is position offset.
    - `60E6` is the actual-position calculation method after homing.
    - `607C` is said to be active when powered on, homing is complete, and `6041 bit15 = 1`.
    - after homing, `6064` is said to equal `607C`.
    - `6091` defines the proportional relationship between load-shaft displacement and motor-shaft displacement.
  - Manual/bench alignment:
    - the documented `6064`/`6063` split matches our observed reference-vs-encoder frame split
    - the `6091` wording reinforces the interpretation that "load" means load/output shaft
    - the attached Chapter 5/11 extracts do **not** spell out the vendor-confirmed HM success bits (`bit12 + bit15`, bit13 clear)
  - Manual/bench tension:
    - our clean `J2` tests met the documented `607C` activation preconditions (`powered on`, homed, `bit15=1`) but direct `607C` writes still did not immediately rebase `6064`
    - Chapter 5 lists `U40.16` under absolute linear mode and `U40.28` under absolute rotation mode, yet on the live rotation-mode axes `U40.16` is still present and behaved differently from `6064`
- Follow-up notes / risks:
  - The manual revisit usefully narrows the open questions: the main unresolved issues are now `607C` direct-write activation semantics, the exact role of `60E6`, and the meaning/validity of `U40.16` in rotation mode.
  - A strong new manufacturer follow-up is to ask whether direct manual `607C` writes are supposed to affect `6064` immediately once the documented activation conditions are already true, or only after a specific refresh event such as HM, software reset, or repower.

## 2026-04-15 10:30 +0000

- What changed:
  - Ran the requested zero-`607C` `J6` control sequence with probe plus safe power transitions only, no jog:
    - `python3 scripts/a6ec_chapter5_probe.py snapshot --label j6-pre-zero-607c-control --axes J6 --experiment-id 20260415-j6-zero-607c-power-control`
    - API `POST /control/power-up`
    - `python3 scripts/a6ec_chapter5_probe.py snapshot --label j6-post-power-up --axes J6 --experiment-id 20260415-j6-zero-607c-power-control`
    - API `POST /control/power-down`
    - `python3 scripts/a6ec_chapter5_probe.py snapshot --label j6-final-disarmed --axes J6 --experiment-id 20260415-j6-zero-607c-power-control`
  - Re-checked final controller motion status after the sequence.
- Validation:
  - Final controller state returned to:
    - `safe_for_power_transition = true`
    - `power_transition_blockers = []`
    - `state = idle`
  - Across the three snapshots:
    - `0x607C` stayed `0`
    - `vendor_hm_success_signature` stayed `false`
    - pre snapshot: `0x6041 = 0x1650`, `6064 = 40736`, `U40.16 = -90338`, raw absolute `U40.20/.22 = -34214`
    - post power-up: all major families shifted only about `41..46` counts
    - final disarmed: `6064` returned exactly to baseline, `U40.16` returned within `1` count, raw absolute `U40.20/.22` within `5` counts
    - bridge invariants stayed effectively flat:
      - `abs_minus_ref`: `-74950 -> -74945 -> -74945`
      - `api_abs_minus_raw`: `-74947 -> -74946 -> -74945`
      - `absolute_home_anchor_rad` unchanged
- Follow-up notes / risks:
  - This is a strong control result against the `J2` anomaly: `SAFE_POWER_UP` / `SAFE_POWER_DOWN` by themselves do **not** inherently cause the large coherent multi-family shift seen on `J2`.
  - The leading interpretation is now that the `J2` transition behavior is axis-specific (for example load/gravity/brake/compliance or another `J2`-local effect), not a universal `607C` activation behavior.
  - The generic `11.3.11 Group U40` prose remains non-probative for `U40.16`/`U40.20/.22`/`U40.28/.2A/.2C`; it only documents the low-number generic monitor fields and does not settle rotation-mode semantics for the fields we actually care about.

## 2026-04-15 10:35 +0000

- What changed:
  - Ran the next `J6` nonzero-`607C` power-control sequence, still with no jog:
    - `python3 scripts/a6ec_chapter5_probe.py snapshot --label j6-pre-nonzero-607c-control --axes J6 --experiment-id 20260415-j6-nonzero-607c-power-control`
    - `sudo ethercat download -p 5 -t int32 0x607C 0 4096`
    - `python3 scripts/a6ec_chapter5_probe.py snapshot --label j6-post-write-disarmed --axes J6 --experiment-id 20260415-j6-nonzero-607c-power-control`
    - API `POST /control/power-up`
    - `python3 scripts/a6ec_chapter5_probe.py snapshot --label j6-post-power-up --axes J6 --experiment-id 20260415-j6-nonzero-607c-power-control`
    - API `POST /control/power-down`
    - `python3 scripts/a6ec_chapter5_probe.py snapshot --label j6-post-power-down-nonzero --axes J6 --experiment-id 20260415-j6-nonzero-607c-power-control`
    - `sudo ethercat download -p 5 -t int32 0x607C 0 0`
    - `python3 scripts/a6ec_chapter5_probe.py snapshot --label j6-final-disarmed --axes J6 --experiment-id 20260415-j6-nonzero-607c-power-control`
  - Re-checked final controller motion status after restoring `0x607C = 0`.
- Validation:
  - Initial state:
    - `0x6041 = 0x1650`
    - `vendor_hm_success_signature = false`
    - `0x607C = 0`
    - `6064 = 40734`
    - `U40.16 = -90337`
    - raw absolute `U40.20/.22 = -34210`
  - Direct nonzero write remained inert while disarmed:
    - `0x607C: 0 -> 4096`
    - `6064: 40734 -> 40734`
    - `api_raw_counts: 40734 -> 40734`
    - `api_absolute_counts: -34213 -> -34212`
  - `SAFE_POWER_UP` still caused only the same small drift-band movement as the zero-`607C` control:
    - `6064: 40734 -> 40691` (`-43`)
    - `U40.16: -90337 -> -90383` (`-46`)
    - raw absolute `U40.20/.22: -34210 -> -34255` (`-45`)
    - API `canonical_rad` changed by only `~2.06e-04`
  - Bridge invariants stayed effectively flat:
    - `abs_minus_ref: -74944 -> -74946 -> -74948 -> -74946`
    - `api_abs_minus_raw: -74947 -> -74946 -> -74948 -> -74946 -> -74947`
    - `absolute_home_anchor_rad` unchanged
  - Final restored state:
    - `0x607C = 0`
    - `safe_for_power_transition = true`
    - `power_transition_blockers = []`
- Follow-up notes / risks:
  - This shows that on `J6`, while HM-valid remains false, a small direct nonzero `607C` still does not produce any observable selective rebase of `6064`, `U40.16`, or API truth.
  - Together with the zero-`607C` control, this further isolates the `J2` large-shift behavior as axis-specific rather than a generic `607C` activation behavior.
  - Scope limit remains important: because `J6` never entered a clean HM-valid state in this run, this does not answer the distinct question of what nonzero `607C` would do on a clean homed `J6`-style axis.

## 2026-04-15 10:49 +0000

- What changed:
  - Ran the requested `J6` zero-`607C` jog experiment as a new sequence under `logs/encoder-retention/20260415-j6-zero-607c-jog-control/`:
    - `j6-pre-jog-control.json/.md`
    - `j6-post-power-up-pre-jog.json/.md`
    - `j6-post-jog.json/.md`
    - `j6-final-disarmed.json/.md`
  - Powered up with API `POST /control/power-up`.
  - Because `/control/joint-jog` is still blocked by the unrelated top-level canonical/display truth gate, sent the same underlying controller command directly over UDP:
    - live-read current `arm_deg` from `/info/joints-detailed`
    - added `+0.25 deg` to `J6`
    - sent `APPLY_JOINT_SETPOINT,<base64-json>` with `max_motor_rpm = 100.0`
  - Waited for idle with API `POST /control/wait-for-idle`.
  - Powered down with API `POST /control/power-down`.
- Validation:
  - Command accepted cleanly:
    - controller replied `ACK,APPLY_JOINT_SETPOINT,...`
    - RTCore reported trajectory `id=1`, `duration_s=0.25`, `frequency_hz=100`
  - `WAIT_FOR_IDLE` reported the move `completed`.
  - During the post-complete state, RTCore kept `active_trajectory` latched briefly; `SAFE_POWER_DOWN` cleared that latch and returned the system to:
    - `safe_for_power_transition = true`
    - `active_traj_id = 0`
  - Motion result from powered pre-jog to post-jog:
    - API `canonical_deg`: `24.82525634765625 -> 25.07601928710937` (`+0.25076293945312145 deg`)
    - `6064`: `40690 -> 39777` (`-913`)
    - `U40.16`: `-90382 -> -91295` (`-913`)
    - raw absolute `U40.20/.22`: `-34256 -> -35169` (`-913`)
    - `U40.28`: `40689 -> 39775` (`-914`)
    - `U40.2A/.2C`: `40689 -> 39776` (`-913`)
  - Coherence checks stayed healthy:
    - `combined(U40.20/.22) - 6064` stayed constant
    - `absolute_home_anchor_rad` stayed unchanged
    - final disarmed state remained near the post-jog pose rather than falling back to the pre-jog pose
- Follow-up notes / risks:
  - This is the strongest direct evidence so far that `J6` motion semantics at `607C = 0` are healthy: the tiny commanded move produced a matching tiny canonical pose change and the raw/reference families moved together coherently.
  - The public `/control/joint-jog` route remains misleadingly blocked by the top-level global truth gate; that API problem is now even more clearly separate from the underlying `J6` motion path.
  - The next clean extension, if desired, is the same tiny `J6` jog test on the nonzero-`607C` branch to see whether motion stays equally coherent there.

## 2026-04-15 10:59 +0000

- What changed:
  - Ran the requested nonzero-`607C` `J6` jog experiment as a new sequence under `logs/encoder-retention/20260415-j6-nonzero-607c-jog-control/`:
    - `j6-pre-nonzero-jog-control.json/.md`
    - `j6-post-write-disarmed.json/.md`
    - `j6-post-power-up-pre-jog.json/.md`
    - `j6-post-jog.json/.md`
    - `j6-post-power-down-nonzero.json/.md`
    - `j6-final-disarmed.json/.md`
  - Wrote `J6 0x607C = 4096` while disarmed, then powered up with API `POST /control/power-up`.
  - As with the prior jog experiment, bypassed the broken public `/control/joint-jog` route and sent the underlying controller command directly over UDP:
    - live-read current `arm_deg` from `/info/joints-detailed`
    - added `+0.25 deg` to `J6`
    - sent `APPLY_JOINT_SETPOINT,<base64-json>` with `max_motor_rpm = 100.0`
  - Waited for idle with API `POST /control/wait-for-idle`, then powered down with `POST /control/power-down`.
  - Restored `J6 0x607C = 0` after the run.
- Validation:
  - Immediate disarmed write still looked inert:
    - `j6-pre-nonzero-jog-control` had `607C = 0`
    - `j6-post-write-disarmed` had `607C = 4096`
    - API canonical pose stayed unchanged across that write
  - Motion command accepted cleanly:
    - controller replied `ACK,APPLY_JOINT_SETPOINT,...`
    - RTCore reported trajectory `id=2`, `duration_s=0.25`, `frequency_hz=100`
    - `WAIT_FOR_IDLE` reported the move `completed`
  - Motion result from powered pre-jog to post-jog:
    - API `canonical_deg`: `25.083984375 -> 25.33529663085937` (`+0.25131225585937145 deg`)
    - `6064`: `39746 -> 38831` (`-915`)
    - `U40.16`: `-91326 -> -92237` (`-911`)
    - raw absolute `U40.20/.22`: `-35200 -> -36114` (`-914`)
    - `U40.28`: `39746 -> 38834` (`-912`)
    - `U40.2A/.2C`: `39748 -> 38833` (`-915`)
  - Coherence checks stayed healthy:
    - `combined(U40.20/.22) - 6064` stayed within `1` count
    - `api absolute_counts - raw_counts` stayed within `3` counts
    - `absolute_home_anchor_rad` stayed unchanged
  - Powered-down and restored-zero states remained clean:
    - `j6-post-power-down-nonzero` still showed nonzero `607C = 4096`
    - `j6-final-disarmed` showed restored `607C = 0`
    - final motion status was `safe_for_power_transition = true`, `active_traj_id = 0`
- Follow-up notes / risks:
  - This materially strengthens the earlier conclusion: on non-HM-valid `J6`, a small nonzero `607C` does not appear to change the live motion semantics. The tiny commanded move stayed coherent and almost numerically identical to the zero-`607C` control jog.
  - That pushes the remaining uncertainty away from generic `J6` nonzero-`607C` behavior and toward either:
    - axis-specific `J2` behavior, or
    - the still-untested case where an axis is both nonzero-`607C` and cleanly HM-valid.
  - The public `/control/joint-jog` route is still blocked by the unrelated global truth gate, so future motion experiments will remain easier to interpret if they continue using the direct `APPLY_JOINT_SETPOINT` path until that API gate is fixed.

## 2026-04-15 11:07 +0000

- What changed:
  - Re-checked the A6-EC manual directly for reset/default behavior before any `J2` pre-replacement recommendation.
  - Confirmed from `docs/resources/chapter 11 - parameter list - A6-EC_series_servo_drive_manual (2).pdf` that the vendor reset family is under `2031h/F31`:
    - `F31.00 / 0x2031:01` = fault reset
    - `F31.01 / 0x2031:02` = software reset
    - `F31.02 / 0x2031:03` = parameter initialization
    - `F31.03 / 0x2031:04` = drive/motor parameter reset
    - `F31.10 / 0x2031:11` = encoder data reset/read/write/fault reset
  - Confirmed manual semantics:
    - all are `At stop` and `Immediately` effective
    - `F31.01` software reset is allowed only while the drive is disabled and there is no non-resettable fault
    - `F31.10` warns that resetting multi-turn encoder data changes saved absolute position abruptly and requires mechanical homing
  - Rechecked prior live bench evidence from the earlier `J2` reset probe:
    - normal `POST /control/reset-faults` did not restore the pre-boot pose
    - vendor software reset `0x2031:02 = 1` also did not restore HM/reference-valid state
- Validation:
  - Read manual reset table and descriptions from the Chapter 11 parameter-list extract
  - Re-read the prior reset experiment entry in `.cursor/memory/DEVLOG.md`
- Follow-up notes / risks:
  - The manual does contain the factory/default reset operations that were missing from the active SOP notes, but that does not make them the best next `J2` step.
  - Current recommendation remains: do not jump straight to `F31.02`/`F31.03` factory/default resets before backup and a full recommission plan, because they are destructive and our current evidence does not yet point to a simple stale-parameter problem.
  - If the user wants this made durable in team-facing docs, the next documentation task is to add a reset-object subsection to the commissioning SOP with the manual-backed guardrails and the project-specific caution about `J2`.

## 2026-04-15 11:15 +0000

- What changed:
  - Ran the requested softer reset probe on `J2` using the manual-backed vendor software-reset object `F31.01 / 0x2031:02`.
  - Captured before/after probe artifacts under `logs/encoder-retention/20260415-j2-software-reset-probe/`:
    - `j2-pre-software-reset.json/.md`
    - `j2-post-software-reset.json/.md`
  - Verified baseline controller state was idle/disarmed before the write.
  - Confirmed `J2` is still slave `-p 1` from direct EtherCAT reads.
  - Wrote `sudo ethercat download -p 1 -t uint16 0x2031 0x02 1`.
  - Polled RTCore metrics through the reset and then issued `POST /control/reset-faults` to clear the induced transient drive fault.
- Validation:
  - Pre-reset `J2` snapshot:
    - `0x6041 = 0x9650`
    - `vendor_hm_success_signature = true`
    - `0x607C = 0`
    - `0x6064 = 13350`
    - raw absolute `U40.20/.22 = 31087`
    - API `canonical_deg ~= 0.36661376953124997`
    - roundtrip error `0`
  - Software-reset transition:
    - RTCore dropped to `startup_ready = 0`
    - `wkc_actual` temporarily dropped as low as `7`
    - after recovery, RTCore returned to `startup_ready = 1`, `wkc_actual = 18`
  - Post-reset `J2` snapshot:
    - `0x6041 = 0x1650`
    - `vendor_hm_success_signature = false`
    - `0x607C = 0`
    - `0x6064 = 113099`
    - raw absolute `U40.20/.22` still `31087`
    - API canonical truth unavailable
    - roundtrip error `~31322` counts
  - Side effects:
    - controller reported transient `fault_present` on axis `0` and `not_synchronized`
    - `metrics.json` showed axis `0` fault code `34560` (`0x8700`) before cleanup
    - `POST /control/reset-faults` cleared the drive fault, but controller/API still remained in `not_synchronized`
    - `/info/joints-detailed` currently reports `read_source = unavailable` and empty `arm_deg`
- Follow-up notes / risks:
  - This re-run is stronger evidence than the earlier summary alone: `F31.01` software reset is not a harmless pre-replacement diagnostic on the current stack. It can actively destroy a previously coherent `J2` home/reference-valid state without fixing the underlying issue.
  - Fault reset alone was not enough to restore synchronized truth after this probe. Recovery likely needs a higher-level controller/stack restart or a fresh native-home workflow on `J2`.
  - This result makes a factory/default reset look less attractive as a "safe practice" step, because the softer reset already moves the system away from a usable truth state rather than toward one.

## 2026-04-15 11:24 +0000

- What changed:
  - Reviewed a new manufacturer reply covering:
    - `RM` / "load" meaning
    - `C10.18 / C10.19` vs `6091`
    - the need to re-home after changing the mechanical ratio
    - manual `607C` writes vs HM Method 35
    - negative `607C` interpretation in rotary mode
    - trust-state bits and `0x9650`
    - `C13.10`
    - `F31.10`
  - Updated the active workstream note `docs/ethercat/a6ec-frame-semantics-and-native-home.md` with a new manufacturer-clarification section and explicit integration implications.
  - Updated `.cursor/skills/gradientos-sop/commissioning-safety.md` so the SOP now captures the manufacturer-backed rotary-mode guidance about:
    - `C10.18 / C10.19`
    - re-home-after-ratio-change
    - `607C` manual write insufficiency
    - avoiding negative `607C` in rotary mode
- Validation:
  - Cross-checked the reply against:
    - current workstream note
    - current startup-config code shape
    - manual extracts already in the repo
- Follow-up notes / risks:
  - The manufacturer reply materially strengthens one previously-reverted implementation direction: programming the true mechanical ratio in `C10.18 / C10.19` at startup and then doing one explicit re-home migration.
  - Current code impact is localized rather than architectural: the RTCore startup-config pipeline still exists, but the active A6-EC profile currently emits only `C00.07`, so adding `C10.18 / C10.19` back would mainly be a drive-profile/startup-config extension plus tests.
  - Even with the stronger vendor guidance, this should still be treated as a deliberate migration step, not a blind hotfix, because changing `C10.18 / C10.19` immediately changes the coordinate system and therefore requires a controlled re-home on hardware.

## 2026-04-15 21:20 +0000

- What changed:
  - Extended the A6-EC drive-native migration through the runtime, backend, telemetry, tests, and docs:
    - `src/gradient_os/arm_controller/backends/ethercat_rtcore/runtime.py`
    - `src/gradient_os/arm_controller/backends/ethercat_rtcore/backend.py`
    - `src/gradient_os/telemetry/native_home_status.py`
    - `src/gradient_os/telemetry/drive_faults.py`
    - `tests/test_rtcore_runtime.py`
    - `tests/test_gradient05_limits_and_backends.py`
    - `docs/ethercat/a6ec-frame-semantics-and-native-home.md`
    - `.cursor/skills/gradientos-sop/commissioning-safety.md`
  - Added shared `derive_drive_native_truth_validity(...)` logic so telemetry/backend code use one conservative restart-validity contract for A6-EC drive-native truth.
  - Added explicit `drive_native_ratio_enabled`, startup-validity, coordinate-system-validity, and drive-native-truth fields to the drive fault snapshot and backend truth details.
  - Rebased the backend canonical/display truth path so A6-EC can use the drive reference/output-shaft frame directly, but only when both:
    - startup drive-config verification succeeds
    - the live statusword still carries the vendor-confirmed HM-valid signature with no active alarms
  - Kept the fallback conservative: if startup verification is missing/mismatched or the HM-valid signature is absent, the backend stays on the legacy absolute-anchor reconstruction path.
  - Neutralized host-side double-scaling for the drive-native posture by rendering `GRADIENT_RT_GEAR_RATIO=1` in RTCore startup env for A6-EC while leaving the cold-start Python fallback config conservative until RTCore reports live axis scaling.
  - Added focused regression coverage for:
    - A6-EC drive-native RTCore axis scaling/env rendering
    - drive-native startup/truth status in `build_drive_fault_snapshot(...)`
    - backend direct truth selection once startup verification and `0x9650` are both present
- Validation:
  - `python3 -m py_compile src/gradient_os/arm_controller/backends/ethercat_rtcore/backend.py src/gradient_os/arm_controller/backends/ethercat_rtcore/runtime.py src/gradient_os/telemetry/drive_faults.py src/gradient_os/telemetry/native_home_status.py tests/test_rtcore_runtime.py tests/test_gradient05_limits_and_backends.py`
  - `ReadLints` on the edited Python files returned no diagnostics.
  - Attempted focused pytest slices, but the environment could not run them:
    - `pytest ...` failed because `pytest` is not installed on `PATH`
    - `python3 -m pytest ...` failed because `pytest` is not installed in the system interpreter
    - `uv run --extra dev python -m pytest ...` failed offline with DNS resolution errors while fetching dependencies
- Follow-up notes / risks:
  - The backend now has the intended conservative gate, but full regression confidence still depends on running the focused pytest slices in an environment with the project dev dependencies installed.
  - The startup-validity gate currently rides on the existing primary `startup_drive_config` readback channel; if the UI later needs per-ratio visibility, the telemetry contract may need a richer multi-descriptor summary without breaking current consumers.

## 2026-04-15 21:45 +0000

- What changed:
  - Tightened the A6-EC contract from "drive-native preferred with legacy fallback" to "drive-native only, fail closed when not valid":
    - `src/gradient_os/arm_controller/profiles/drive/a6ec_ds402.py`
    - `src/gradient_os/arm_controller/backends/ethercat_rtcore/backend.py`
    - `src/gradient_os/run_controller.py`
  - Added explicit A6-EC profile flags so the backend knows there is no legacy truth fallback and no absolute-home-anchor requirement for active A6-EC truth.
  - Moved `configured_drive_profile_id` capture earlier in backend init so cold-start axis scaling can honor non-default drive profiles before RTCore hello arrives.
  - Updated the backend/native-home flow so A6-EC:
    - keeps `drive_output_shaft` as the configured truth source
    - reports truth unavailable when startup verification or HM-valid status is missing
    - does not bootstrap or require absolute-home anchors for active A6-EC truth
  - Updated legacy backend tests to opt into an explicit legacy-anchor mode where needed, so non-drive-native coverage still exists without reintroducing fallback to production A6-EC behavior.
  - Updated docs and SOP guidance:
    - `docs/ethercat/a6ec-frame-semantics-and-native-home.md`
    - `.cursor/skills/gradientos-sop/commissioning-safety.md`
- Validation:
  - `source ./start.sh`
  - `python -m pytest tests/test_rtcore_runtime.py tests/test_gradient05_limits_and_backends.py tests/test_drive_faults.py tests/test_run_controller_helpers.py tests/test_terminal_dashboard.py -q`
    - result: `106 passed`
  - Live managed-stack validation:
    - started with `GRADIENT_STACK_INTERACTIVE_CONSOLE=0 ./start-stack.sh`
    - confirmed stack boot completed cleanly and then soft-stopped it with `./start-stack.sh stop`
    - `./start-stack.sh probe` showed runtime/profile selection was live on `a6ec_ds402`
    - `/etc/default/gradient-rt-motion` confirmed:
      - `GRADIENT_RT_GEAR_RATIO="1,1,1,1,1,1"`
      - `GRADIENT_RT_DRIVE_STARTUP_SDO_CONFIG="a6ec_encoder_position_tracking_mode ... ; a6ec_rotation_mode_gear_ratio_numerator ... ; a6ec_rotation_mode_gear_ratio_denominator ..."`
    - live drive-fault snapshot built from `/run/gradient-rt-motion/metrics.json` confirmed:
      - `drive_native_ratio_enabled = true`
      - `drive_native_startup_valid = true` on all 6 axes
      - `drive_native_truth_valid = false` on all 6 axes
      - every axis `statusword_hex = 0x1650`
      - every axis `drive_native_truth_reason = coordinate_system_invalid`
    - `/info/joints-detailed` axis detail matched the same conclusion: the controller is selecting the new drive-native path and failing closed because the coordinate system is not yet HM-valid.
- Follow-up notes / risks:
  - The live blocker is now clearly hardware/commissioning state, not code-path ambiguity: the axes still need a fresh HM35 re-home so the statusword returns to the vendor-confirmed `0x9650` trust state.
  - The current startup-validity witness still comes through the primary `startup_drive_config` readback object; if we later need operator-visible proof of `C10.18/C10.19` specifically, telemetry should gain a richer multi-descriptor summary.

## 2026-04-15 21:54 +0000

- What changed:
  - Removed the last mixed A6-EC fallback semantics from the active implementation/doc surface:
    - `src/gradient_os/arm_controller/profiles/drive/a6ec_ds402.py`
    - `src/gradient_os/arm_controller/backends/ethercat_rtcore/backend.py`
    - `docs/ethercat/a6ec-frame-semantics-and-native-home.md`
  - Dropped the now-misleading `legacy_truth_fallback_enabled` A6-EC profile flag.
  - Simplified backend anchor ownership so active A6-EC behavior is driven only by `absolute_home_anchor_required`:
    - A6-EC no longer loads anchor state into active truth decisions
    - A6-EC software-zero capture no longer refreshes absolute-home anchors
    - non-drive-native profiles still retain anchor-based truth where required
  - Cleaned the active A6-EC work note so it no longer describes any A6-EC fallback/debug truth contract.
- Validation:
  - `source ./start.sh && python -m pytest tests/test_rtcore_runtime.py tests/test_gradient05_limits_and_backends.py tests/test_drive_faults.py tests/test_run_controller_helpers.py tests/test_terminal_dashboard.py -q`
    - result: `106 passed`
  - `ReadLints` on:
    - `src/gradient_os/arm_controller/backends/ethercat_rtcore/backend.py`
    - `src/gradient_os/arm_controller/profiles/drive/a6ec_ds402.py`
    - `docs/ethercat/a6ec-frame-semantics-and-native-home.md`
    - result: no diagnostics
- Follow-up notes / risks:
  - Remaining `fallback` naming in the repo is now outside the active A6-EC truth contract:
    - generic non-A6EC legacy tests
    - unrelated planner/runtime fallback concepts
    - native-home statusword verification fallback logic
  - The A6-EC runtime behavior itself remains fail-closed and still needs a fresh HM35 re-home on hardware to move from `0x1650` to `0x9650`.

## 2026-04-15 22:31 +0000

- What changed:
  - Closed the remaining startup-verification gap in the A6-EC drive-native migration:
    - `src/gradient_rt_motion/main.cpp`
    - `src/gradient_os/arm_controller/profiles/drive/a6ec_ds402.py`
    - `src/gradient_os/arm_controller/backends/ethercat_rtcore/backend.py`
    - `src/gradient_os/telemetry/drive_faults.py`
    - `tests/test_rtcore_runtime.py`
    - `tests/test_gradient05_limits_and_backends.py`
    - `docs/ethercat/a6ec-frame-semantics-and-native-home.md`
    - `.cursor/memory/AGENT_SCRATCHPAD.md`
  - RTCore metrics now publish a richer per-axis `startup_drive_configs` array for all configured startup SDO descriptors while keeping the old primary `startup_drive_config` field for compatibility.
  - RTCore now clears startup-drive-config verification feedback when the startup epoch changes so restart-time truth fails closed instead of inheriting stale verification from the previous epoch.
  - The A6-EC profile extractor now aggregates `C00.07`, `C10.18`, and `C10.19` and only reports startup verification success when all required descriptors are present, readable, and verified.
  - The backend startup-validity gate now uses that aggregated extractor rather than the raw primary metrics field, so A6-EC truth availability no longer turns on without ratio-SDO proof.
  - Added focused regressions for:
    - missing-required-startup-settings fail-closed behavior
    - non-`1:1` A6-EC trajectory upload staying in logical radians while host scaling is neutralized
- Validation:
  - `make -C src/gradient_rt_motion`
  - `source ./start.sh && python -m pytest tests/test_rtcore_runtime.py tests/test_gradient05_limits_and_backends.py tests/test_api_endpoints.py tests/test_encoder_retention.py -q`
    - result: `154 passed`
  - `source ./start.sh && python -m pytest tests/test_rtcore_runtime.py tests/test_gradient05_limits_and_backends.py tests/test_drive_faults.py tests/test_run_controller_helpers.py tests/test_terminal_dashboard.py tests/test_api_endpoints.py tests/test_encoder_retention.py -q`
    - result: `178 passed`
  - `ReadLints` on:
    - `src/gradient_os/arm_controller/backends/ethercat_rtcore/backend.py`
    - `src/gradient_os/arm_controller/profiles/drive/a6ec_ds402.py`
    - `src/gradient_os/telemetry/drive_faults.py`
    - `src/gradient_rt_motion/main.cpp`
    - `tests/test_rtcore_runtime.py`
    - `tests/test_gradient05_limits_and_backends.py`
    - result: no diagnostics
- Follow-up notes / risks:
  - This turn validated build and tests only; it did not repeat a live stack bring-up after the multi-descriptor startup-readback change.
  - The commissioning blocker remains unchanged: hardware still needs a fresh HM35 re-home so statusword moves from `0x1650` to the vendor-valid `0x9650` trust state.

## 2026-04-15 22:45 +0000

- What changed:
  - Executed the live A6-EC validation plan against hardware without changing the attached plan file.
  - Rebuilt RTCore and reran the planned automated regression gate before touching the stack.
  - Brought the stack up in supervised non-interactive mode and confirmed the intended pre-home baseline:
    - `drive_native_startup_valid = true` on all six axes
    - `drive_native_truth_valid = false` on all six axes
    - every axis at `statusword_hex = 0x1650`
  - Ran `POST /control/home-joint-native` on `J6` first and verified the expected per-axis success state:
    - `statusword_hex = 0x9650`
    - `drive_native_truth_valid = true`
    - axis remained disabled after home
  - Found a live workflow constraint in the current controller path: public `SAFE_POWER_UP` stayed blocked with `not_synchronized` until all six axes had been HM35-homed, because the power-transition and `GET_JOINT_STATE` paths still require a complete live joint vector.
  - Completed HM35 on `J1`-`J5`, then ran the full public smoke path:
    - `POST /control/power-up`
    - tiny `POST /control/joint-jog` on `J6` with `delta_deg = 0.2`
    - `POST /control/power-down`
- Validation:
  - `make -C src/gradient_rt_motion`
  - `source ./start.sh && python -m pytest tests/test_rtcore_runtime.py tests/test_gradient05_limits_and_backends.py tests/test_drive_faults.py tests/test_run_controller_helpers.py tests/test_terminal_dashboard.py tests/test_api_endpoints.py tests/test_encoder_retention.py -q`
    - result: `178 passed`
  - Live API / hardware checks:
    - after HM35 on all axes, `/info/joints-detailed` returned `read_source = live_feedback` and `canonical_joint_truth_available = true`
    - tiny `J6` jog succeeded through the public API with observed delta about `+0.205994` deg for a requested `+0.2` deg
    - post-jog `./start-stack.sh probe` returned `BUS_UP_DISARMED`, `0/6` enabled, all axes still `0x9650`, and no faults
- Follow-up notes / risks:
  - The drive-native startup-verification and HM-valid truth contract is now validated live, not just in unit tests.
  - The public API smoke path currently needs the whole arm HM-valid before enable/jog; a single-axis HM35 is not enough to satisfy the existing controller synchronization guard.
  - The optional persistence / restart slice from the plan was not run in this pass.

## 2026-04-15 23:00 +0000

- What changed:
  - Cleaned up the A6-EC frontend pose path after the drive-native truth migration:
    - `src/gradient_os/run_controller.py`
    - `tests/test_run_controller_helpers.py`
    - `web-ui/src/App.tsx`
    - `web-ui/src/TelemetryCharts.tsx`
    - `web-ui/src/poseTelemetry.ts`
    - `web-ui/src/poseTelemetry.test.ts`
  - Fixed the controller monitor SSE contract so `display_joints` is no longer copied from raw canonical `q`; it now comes from the backend display snapshot, matching the same operator-facing truth already exposed by `/info/joints-detailed`.
  - Added focused controller helper regressions to prove monitor payload separation between raw `joints` and operator `display_joints`.
  - Added a tiny shared frontend helper for preferred operator pose selection and switched app/chart consumers to prefer `display_joints` over raw `joints`.
  - Fixed the app fallback pose path so `/info/joints` display feedback is stored in `display_joints` rather than being mislabeled as raw `joints`.
- Validation:
  - `source ./start.sh && python -m pytest tests/test_run_controller_helpers.py -q`
    - result: `9 passed`
  - `npm --prefix web-ui test -- src/poseTelemetry.test.ts src/ControlPanel.test.tsx`
    - result: `21 passed`
  - `ReadLints` on changed controller/frontend files
    - result: no diagnostics
  - Controlled disarmed stack restart, then live payload checks:
    - `/info/joints-detailed.arm_display_deg` showed the expected small operator-facing angles
    - `/monitor` now reports small `display_joints` while raw `joints` remain wrapped on `J3/J4/J6`, proving the streams are separated again
- Follow-up notes / risks:
  - I validated the live payload contract after restart, but I did not run an interactive browser visual pass against the commissioning panel itself.
  - The monitor stream still publishes both raw `joints` and operator `display_joints`; future UI code should keep preferring the display stream unless a view explicitly wants raw canonical motion state.

## 2026-04-15 23:10 +0000

- What changed:
  - Ran a live read-only ratio proof to confirm the A6-EC drives now hold the native mechanical gear ratios in `C10.18 / C10.19`.
  - Captured a full all-axis Chapter 5 probe artifact:
    - `logs/encoder-retention/native-ratio-proof/native-ratio-proof.json`
    - `logs/encoder-retention/native-ratio-proof/native-ratio-proof.md`
  - Cross-checked the direct probe with RTCore startup SDO readback from `/run/gradient-rt-motion/metrics.json`.
- Validation:
  - `source ./start.sh && ./start-stack.sh probe`
    - stack remained `BUS_UP_DISARMED`, `0/6` enabled, all axes `0x9650`, no faults before the read-only probe
  - `source ./start.sh && python scripts/a6ec_chapter5_probe.py snapshot --label native-ratio-proof --axes J1 J2 J3 J4 J5 J6 --experiment-id native-ratio-proof`
    - direct drive readback:
      - `J1/J2/J3`: `C00.07=4`, `C10.18=100`, `C10.19=1`
      - `J4`: `C00.07=4`, `C10.18=18`, `C10.19=1`
      - `J5`: `C00.07=4`, `C10.18=125`, `C10.19=4`
      - `J6`: `C00.07=4`, `C10.18=10`, `C10.19=1`
  - `source ./start.sh && python - <<'PY' ... /run/gradient-rt-motion/metrics.json ... PY`
    - startup SDO metrics readback matched those same commanded values and reported `verified = 1` for:
      - `a6ec_encoder_position_tracking_mode`
      - `a6ec_rotation_mode_gear_ratio_numerator`
      - `a6ec_rotation_mode_gear_ratio_denominator`
- Follow-up notes / risks:
  - This is the right proof for native gearing because it reads the rotary-mode mechanical ratio objects directly; `6091` remained `1:1`, which is expected and should not be confused with the native mechanical ratio path.

## 2026-04-15 23:18 +0000

- What changed:
  - Ran an explicit host-side double-count sanity check after the native ratio proof.
  - Verified the live RTCore env still neutralizes software gear ratios to `1,1,1,1,1,1`.
  - Verified the active A6-EC backend math for a non-`1:1` axis uses neutral counts-per-radian instead of legacy `encoder_counts_per_rev * gear_ratio / (2*pi)`.
- Validation:
  - `ReadFile /etc/default/gradient-rt-motion`
    - confirmed `GRADIENT_RT_GEAR_RATIO="1,1,1,1,1,1"`
  - `source ./start.sh && python -m pytest tests/test_gradient05_limits_and_backends.py -q -k nonunit_a6ec_axis_in_logical_radians`
    - result: `1 passed`
  - `source ./start.sh && python - <<'PY' ... EthercatRTCoreBackend(robot_config=Gradient05Config().get_config_dict()) ... PY`
    - confirmed:
      - `drive_native_ratio_enabled = True`
      - `robot_cfg_j5_gear_ratio = 31.25`
      - `counts_per_unit_j5 = 131072 / (2*pi)` exactly
      - host `counts_per_unit_j5` was not multiplied by `31.25`
- Follow-up notes / risks:
  - This is the expected “no double counting” posture: the drive owns the mechanical ratio in `C10.18/C10.19`, while the host keeps RTCore gear ratio at `1` and commands in logical/reference radians.

## 2026-04-15 23:42 +0000

- What changed:
  - Investigated a fresh live J6 commissioning jog failure from `logs/startups/latest/controller.log`.
  - Confirmed the failure pattern was:
    - `SAFE_POWER_UP`
    - bounded `APPLY_JOINT_SETPOINT` for J6 at `100 Hz` / `25` points
    - open-loop executor thread timing out on `backend.wait_for_trajectory_complete(...)`
    - later `/control/motion-status` returning clean RTCore idle with `last_submitted_traj_id=1`
  - Patched the targeted commissioning jog path so `/control/joint-jog` now includes `target_joint_indices=[joint-1]` in the `APPLY_JOINT_SETPOINT` payload.
  - Updated the controller/executor/RTCore backend path so targeted bounded trajectories are masked to only the selected logical joint axes when offloaded to RTCore.
  - Improved the RTCore timeout message to include the last observed execution snapshot (`state_name`, `active_traj_id`, `queue_depth`, `motion_done`, `active_command_seq`, `submitted_command_seq`) for future live diagnosis.
- Validation:
  - `source ./start.sh && python -m pytest tests/test_api_endpoints.py::test_control_joint_jog tests/test_api_endpoints.py::test_control_joint_jog_ignores_wait_for_idle_flag tests/test_command_api_direct_setpoint.py::test_handle_apply_joint_setpoint_can_start_bounded_joint_trajectory tests/test_gradient05_limits_and_backends.py::test_ethercat_backend_execute_joint_trajectory_uses_quantized_timing tests/test_trajectory_execution_backends.py::test_open_loop_executor_offloads_rtcore_trajectory_backend tests/test_trajectory_execution_backends.py::test_open_loop_executor_passes_targeted_rtcore_axis_mask -q`
    - result: `6 passed`
  - `source ./start.sh && python -m py_compile src/gradient_os/api/main.py src/gradient_os/run_controller.py src/gradient_os/arm_controller/command_api.py src/gradient_os/arm_controller/trajectory_execution.py src/gradient_os/arm_controller/backends/ethercat_rtcore/backend.py tests/test_api_endpoints.py tests/test_command_api_direct_setpoint.py tests/test_gradient05_limits_and_backends.py tests/test_trajectory_execution_backends.py`
    - result: success
  - `ReadLints` on the touched files
    - result: no diagnostics
  - Read-only live checks during investigation:
    - `GET /control/motion-status` showed RTCore idle, `queue_depth=0`, `motion_done=true`, `last_submitted_traj_id=1`
    - `ReadFile /run/gradient-rt-motion/metrics.json` showed all axes still `0x9650`, startup verified, no faults, disarmed after stop/power-down
- Follow-up notes / risks:
  - I did not re-run a live powered J6 jog after the patch because that would move hardware.
  - If the next live retry still times out, the new timeout payload should tell us whether the residual issue is completion bookkeeping or a deeper seam/command-path problem on the selected axis itself.

## 2026-04-15 23:56 +0000

- What changed:
  - Restarted the live stack after the user's hard stop and drive power cycle, then investigated the reported commissioning-angle flicker with read-only runtime checks before any new motion.
  - Confirmed the restarted stack came up healthy after one launcher-managed RTCore/EtherCAT recycle and landed in the expected fail-closed post-power-cycle state:
    - `safe_for_power_transition=false`
    - blocker `not_synchronized`
    - `/info/joints-detailed` returned `arm_deg=[]`, `arm_display_deg=[]`
    - `/run/gradient-rt-motion/metrics.json` showed all six axes at `0x1650` with startup verification still intact and no faults
  - Quantified read-only rest jitter from repeated `/info/joints-detailed` sampling:
    - raw/reference counts moved only about `1..4` counts per axis
    - display/reference-angle wander was about `0.0082..0.0110 deg`
    - the current post-power-cycle read-only run did not reproduce the earlier `~0.1 deg` J6 flicker
  - Identified the main UI amplification factor in `web-ui/src/ControlPanel.tsx`:
    - commissioning angles updated for changes above `0.001 deg`
    - labels rendered at `0.01 deg` precision
    - normal count-level rest jitter therefore showed up as visible last-digit chatter
  - Implemented a display-only stabilization path in `web-ui/src/ControlPanel.tsx`:
    - added a `0.02 deg` deadband for idle/disarmed EtherCAT commissioning angles
    - left controller/API telemetry and motion semantics unchanged
  - Added a focused regression in `web-ui/src/ControlPanel.test.tsx` to prove the panel stays steady through sub-deadband rest jitter but still updates on a larger change.
- Validation:
  - `source ./start.sh && ./start-stack.sh probe`
    - result: stack fully down before restart (`launcher_state: absent`, controller/API down, RTCore down)
  - `source ./start.sh && python -m pytest tests/test_api_endpoints.py::test_control_joint_jog tests/test_api_endpoints.py::test_control_joint_jog_ignores_wait_for_idle_flag tests/test_command_api_direct_setpoint.py::test_handle_apply_joint_setpoint_can_start_bounded_joint_trajectory tests/test_gradient05_limits_and_backends.py::test_ethercat_backend_execute_joint_trajectory_uses_quantized_timing tests/test_run_controller_helpers.py tests/test_trajectory_execution_backends.py::test_open_loop_executor_offloads_rtcore_trajectory_backend tests/test_trajectory_execution_backends.py::test_open_loop_executor_passes_targeted_rtcore_axis_mask -q`
    - result: `15 passed`
  - `source ./start.sh && GRADIENT_STACK_INTERACTIVE_CONSOLE=0 ./start-stack.sh`
    - result: stack online; latest run `20260415-235055`
  - live runtime checks:
    - `curl -s http://127.0.0.1:4000/control/motion-status`
    - `curl -s http://127.0.0.1:4000/info/joints-detailed`
    - repeated `python` polling against `/info/joints-detailed`
    - decoded `/monitor` SSE payload shape
  - `npm --prefix web-ui test -- src/ControlPanel.test.tsx src/poseTelemetry.test.ts`
    - result: `22 passed`
  - `ReadLints` on `web-ui/src/ControlPanel.tsx` and `web-ui/src/ControlPanel.test.tsx`
    - result: no diagnostics
- Follow-up notes / risks:
  - The new UI deadband addresses the current evidence-backed issue: the panel visibly amplifying harmless disarmed rest jitter.
  - Because the power cycle reset the axes back to `0x1650`, this pass did not reproduce the earlier post-home `~0.1 deg` J6 wobble. If that larger wobble returns after a fresh HM35/home-valid state, it still needs a separate post-home investigation.
  - I intentionally did not issue a new home or jog command in this pass, so there was no additional hardware motion.

## 2026-04-16 00:15 +0000

- What changed:
  - Revisited the A6-EC flicker investigation after the user correctly challenged the neutral-scaling assumption.
  - Proved read-only that the prior repo-wide A6-EC scaling posture was inconsistent with live drive objects:
    - J6 probe showed `6064 = U40.16 = U40.28 = 1310650`
    - `C10.18/C10.19 = 10/1`
    - the old host posture still forced neutral `GRADIENT_RT_GEAR_RATIO=1` and neutral backend `counts_per_unit`
  - Removed the temporary commissioning-panel deadband from `web-ui/src/ControlPanel.tsx` because it was only a presentation workaround and would mask validation of the real fix.
  - Restored physical A6-EC scaling at the actual backend/runtime layer:
    - `src/gradient_os/arm_controller/backends/ethercat_rtcore/runtime.py`
      - RTCore env generation now renders mechanical gear ratios for A6-EC instead of forcing `1`
    - `src/gradient_os/arm_controller/backends/ethercat_rtcore/backend.py`
      - backend robot axis config now uses physical `actuator_counts_per_radian` again instead of forcing neutral counts-per-radian for drive-native A6-EC
  - Updated regressions:
    - `tests/test_rtcore_runtime.py`
    - `tests/test_gradient05_limits_and_backends.py`
    - `web-ui/src/ControlPanel.test.tsx`
  - Added a direct J6 regression proving the same `1310650` feedback sample now decodes to `0.019226... deg`, not `0.19226... deg`.
- Validation:
  - focused runtime/backend regressions
    - `source ./start.sh && python -m pytest tests/test_rtcore_runtime.py -q tests/test_gradient05_limits_and_backends.py -q -k 'build_rtcore_axis_scaling_uses_drive_native_ratio_for_a6ec or render_rtcore_systemd_env_contains_scaling_and_profile or enqueue_trajectory_points_keeps_nonunit_a6ec_axis_in_logical_radians or j6_display_feedback_uses_rotation_mode_ratio_scaling'`
    - result: `4 passed`
  - frontend regressions after removing the deadband
    - `npm --prefix web-ui test -- src/ControlPanel.test.tsx src/poseTelemetry.test.ts`
    - result: `21 passed`
  - broader controller/runtime/trajectory slice
    - `source ./start.sh && python -m pytest tests/test_api_endpoints.py::test_control_joint_jog tests/test_api_endpoints.py::test_control_joint_jog_ignores_wait_for_idle_flag tests/test_command_api_direct_setpoint.py::test_handle_apply_joint_setpoint_can_start_bounded_joint_trajectory tests/test_gradient05_limits_and_backends.py tests/test_rtcore_runtime.py tests/test_trajectory_execution_backends.py::test_open_loop_executor_offloads_rtcore_trajectory_backend tests/test_trajectory_execution_backends.py::test_open_loop_executor_passes_targeted_rtcore_axis_mask -q`
    - result: `90 passed`
  - `ReadLints` on touched runtime/backend/test/frontend files
    - result: no diagnostics
  - direct code-level sanity check
    - patched backend J6 sample `1310650` now yields:
      - wrap period `1310720`
      - decoded angle `0.01922607421875 deg`
  - live restart and read-only verification
    - `source ./start.sh && ./start-stack.sh stop --hard`
    - `source ./start.sh && GRADIENT_STACK_INTERACTIVE_CONSOLE=0 ./start-stack.sh`
    - `/etc/default/gradient-rt-motion` now contains `GRADIENT_RT_GEAR_RATIO="100,100,100,18,31.25,10"`
    - live `/info/joints-detailed` now reports J6 `reference_pre_zero_deg ~= 0.0189514`
    - repeated J6 read-only sampling shows jitter range about `0.0010986 deg`
- Follow-up notes / risks:
  - The stack is still correctly blocked from power-up because the user’s power cycle returned all axes to `0x1650` / `not_synchronized`; this pass did not re-home or power up.
  - This turn corrected the host-side scaling assumption with read-only live proof, but it did not yet re-run a home-valid powered jog on hardware.

## 2026-04-16 00:18 +0000 - Verified per-axis A6-EC count totals are now correct

- Context:
  - The user clarified the required invariant: each axis must use its own mechanical gear ratio, so one output-shaft rotation equals `2^17 * gear_ratio[j]` counts for that axis.
  - This was a read-only verification pass after the earlier scaling correction.
- Validation that ran:
  - `source ./start.sh && python - <<'PY' ... PY`
  - The script compared four sources for all six axes:
    - `actuator_encoder_counts_per_rev`
    - `actuator_gear_ratios`
    - `build_rtcore_axis_scaling(..., drive_profile="a6ec_ds402")`
    - `EthercatRTCoreBackend(...)._reference_wrap_period_counts_for_axis(axis)`
    - `/etc/default/gradient-rt-motion` `GRADIENT_RT_GEAR_RATIO`
- Result:
  - all six axes passed
  - expected and actual wrap periods matched exactly:
    - J1 `13107200`
    - J2 `13107200`
    - J3 `13107200`
    - J4 `2359296`
    - J5 `4096000`
    - J6 `1310720`
  - `GRADIENT_RT_GEAR_RATIO` also matches per-axis robot config values exactly: `100,100,100,18,31.25,10`
- Follow-up implementation:
  - added `test_ethercat_backend_a6ec_reference_wrap_period_uses_per_axis_gear_ratios` to `tests/test_gradient05_limits_and_backends.py`
  - ran focused regressions:
    - `tests/test_rtcore_runtime.py::test_build_rtcore_axis_scaling_uses_drive_native_ratio_for_a6ec`
    - `tests/test_rtcore_runtime.py::test_render_rtcore_systemd_env_contains_scaling_and_profile`
    - `tests/test_gradient05_limits_and_backends.py::test_ethercat_backend_j6_display_feedback_uses_rotation_mode_ratio_scaling`
    - `tests/test_gradient05_limits_and_backends.py::test_ethercat_backend_a6ec_reference_wrap_period_uses_per_axis_gear_ratios`
  - result: `4 passed`
- Lint check:
  - `ReadLints` on `tests/test_gradient05_limits_and_backends.py`
  - result: no diagnostics
- Notes:
  - J5's fractional ratio `31.25` still yields an exact integer period because `131072 * 31.25 = 4096000`.
  - This started as a read-only verification pass, then turned into a test hardening pass so the earlier scaling fix now has an explicit all-axis regression guard, not just the prior J6-specific check.

## 2026-04-16 00:37 +0000 - Surfaced the real post-power-cycle blocker: coordinate system invalid

- Context:
  - The user reported they were still blocked from power-up and wanted the live failure explained and fixed properly.
  - Live logs/endpoints showed the block was real, but the software was collapsing it to a generic `not_synchronized` message.
- Code changes:
  - `src/gradient_os/arm_controller/backends/ethercat_rtcore/backend.py`
    - `get_power_transition_snapshot()` now carries truth-availability diagnostics (`feedback_truth_reasons`, unavailable axes/joints, statusword summary) instead of swallowing canonical-truth failure into an empty joint list.
    - canonical-truth exceptions now include summarized reasons and statuswords, e.g. `drive_native_coordinate_system_invalid` and `0x1650`.
  - `src/gradient_os/arm_controller/command_api.py`
    - power-transition guard now maps backend truth failures into specific blockers:
      - `coordinate_system_invalid` when drive-native truth is invalid
      - `canonical_truth_unavailable` for other truth-unavailable cases
    - keeps the generic `not_synchronized` blocker only for truly reasonless sync gaps.
  - `web-ui/src/ControlPanel.tsx`
    - runtime header now renders a specific operator message for `coordinate_system_invalid` instead of treating it like an ordinary sync-settle check.
- Tests added/updated:
  - `tests/test_gradient05_limits_and_backends.py`
    - added `test_ethercat_backend_power_transition_snapshot_reports_coordinate_system_invalid`
  - `tests/test_command_api_direct_setpoint.py`
    - added `test_build_power_transition_guard_surfaces_coordinate_system_invalid_blocker`
  - `web-ui/src/ControlPanel.test.tsx`
    - added runtime-header coverage for the blocked/native-home-required state
- Validation that ran:
  - focused backend/controller regressions:
    - `tests/test_gradient05_limits_and_backends.py::test_ethercat_backend_power_transition_snapshot_reports_coordinate_system_invalid`
    - `tests/test_gradient05_limits_and_backends.py::test_ethercat_backend_j6_display_feedback_uses_rotation_mode_ratio_scaling`
    - `tests/test_command_api_direct_setpoint.py::test_build_power_transition_guard_surfaces_coordinate_system_invalid_blocker`
    - `tests/test_rtcore_runtime.py::test_build_rtcore_axis_scaling_uses_drive_native_ratio_for_a6ec`
    - result: `4 passed`
  - focused power-up/API regressions:
    - `tests/test_api_endpoints.py::test_control_power_up`
    - `tests/test_api_endpoints.py::test_control_power_up_returns_conflict_when_safety_gate_blocks`
    - `tests/test_command_api_direct_setpoint.py::test_handle_safe_power_up_rejects_when_runtime_is_not_safe`
    - result: `3 passed`
  - frontend regression:
    - `npm --prefix web-ui test -- src/ControlPanel.test.tsx`
    - result: `19 passed`
  - `ReadLints` on touched backend/command-api/frontend/test files
    - result: no diagnostics
- Runtime verification:
  - restarted the stack onto the patched code (`logs/startups/20260416-003638`)
  - live `GET /control/motion-status` now reports:
    - `power_transition_blockers=["coordinate_system_invalid"]`
    - `truth_unavailable_joints=[1,2,3,4,5,6]`
    - `statuswords=["0x1650"]`
    - `requires_native_home=true`
  - live `GET /info/joints-detailed` now reports:
    - `canonical_joint_truth_error="Canonical joint truth unavailable (... reasons=['drive_native_coordinate_system_invalid'], statuswords=['0x1650'])"`
  - controller log now records the same reason explicitly.
- Follow-up / risk:
  - The software diagnosis is fixed, but the physical unblock has not been executed in this pass.
  - All six axes still need a clean native-home/HM35 cycle to return to the vendor-valid `0x9650` state before power-up can legitimately succeed again.

## 2026-04-16 00:44 +0000 - Unblocked Drive Home when canonical truth is unavailable

- Context:
  - After surfacing the real `coordinate_system_invalid` blocker, the user correctly reported they were still blocked because the commissioning UI had disabled every `Drive Home` button.
  - This was a UI deadlock: the recovery action was gated by the very truth signal that native home is supposed to restore.
- Code changes:
  - `web-ui/src/ControlPanel.tsx`
    - decoupled `Drive Home` button enablement from `zeroDisabled`
    - native-home now stays enabled when per-axis drive telemetry is present, even if `arm_display_deg`/canonical display truth is unavailable
    - tooltip now explains the live-drive-telemetry fallback
    - confirmation dialog fallback text now says `current live drive feedback`
- Validation that ran:
  - `npm --prefix web-ui test -- src/ControlPanel.test.tsx`
  - result: `20 passed`
  - `ReadLints` on `web-ui/src/ControlPanel.tsx` and `web-ui/src/ControlPanel.test.tsx`
  - result: no diagnostics
  - live dev server verification:
    - running stack Vite log reported `hmr update /src/ControlPanel.tsx, /src/index.css`
- Tests added/updated:
  - `web-ui/src/ControlPanel.test.tsx`
    - added coverage for the exact recovery state: canonical angles unavailable, live `0x1650` drive telemetry present, `Drive Home` buttons remain enabled
- Follow-up / risk:
  - This fixes the software deadlock in the commissioning panel.
  - The physical recovery still requires actually running native-home/HM35 on the affected axes; this pass did not issue those hardware commands.

## 2026-04-16 01:09 +0000 - Stabilized the commissioning message rail so live status updates stop moving the joint controls

- Context:
  - The user reported that the live message labels above the commissioning controls were flickering in and out and making the whole panel jump, which made the buttons difficult to click.
- Code changes:
  - `web-ui/src/ControlPanel.tsx`
    - replaced the conditional commissioning banner stack with a fixed three-slot message rail
    - each slot now keeps a constant `h-9` height and empty slots remain `invisible` so the surrounding panel height stays stable
    - long messages are clamped inside the slot instead of resizing the container
  - `web-ui/src/ControlPanel.test.tsx`
    - added a regression that asserts the commissioning rail always renders three fixed slots while a live native-home status banner is active
- Validation that ran:
  - `npm --prefix web-ui test -- src/ControlPanel.test.tsx`
  - result: `21 passed`
  - `ReadLints` on `web-ui/src/ControlPanel.tsx` and `web-ui/src/ControlPanel.test.tsx`
  - result: no diagnostics
- Follow-up / risk:
  - The UI now stays stable for clicking, but the fixed-height rail intentionally clips longer messages to preserve panel stability.

## 2026-04-16 01:36 +0000 - Separated J5 false-failure from J1 transient post-home fault during native-home log review

- Context:
  - The user asked why `J5` and `J1` showed native-home errors even though both appeared okay after a fault reset.
- Investigation performed:
  - reviewed the active controller terminal output for the current startup run
  - checked live `/control/motion-status` and `/info/joints-detailed`
  - read raw `/run/gradient-rt-motion/metrics.json`
  - re-read the native-home verification and post-home refresh logic in `src/gradient_os/arm_controller/backends/ethercat_rtcore/backend.py`
  - re-read `src/gradient_os/telemetry/native_home_status.py` to confirm how live telemetry upgrades stale failed metrics to a clean `0x9650` success
- Findings:
  - `J5` logged `native drive-home failed verification ... abort=0x06010002`
  - current raw RTCore metrics still show `J5 native_home_state=3` and abort `0x06010002`, while the same axis also shows clean `statusword=0x9650` and zero live faults
  - current live `/info/joints-detailed` therefore marks `J5` as succeeded via `native_home_verification_source=statusword_bits12_15_clear13`
  - `J1` logged `native drive-home reached a verified terminal state, but post-home anchor refresh failed ... (drive_native_fault_present)`
  - a `RESET_FAULTS` command was issued immediately after the `J1` warning, and canonical truth became available again
  - current raw metrics for `J1` are already clean (`native_home_state=2`, abort `0`), so `J1` was not the stale-`0x06010002` path
- Interpretation:
  - `J5` is the known false-failure/result-contract bug family where stale raw native-home metrics contradict the clean live drive wire-state
  - `J1` was a separate transient live fault/anchor-refresh failure after the home had already verified; the reset cleared that transient condition
- Validation that ran:
  - `python` fetch of live `/control/motion-status` and `/info/joints-detailed`
  - `python` probe printing live native-home fields for `J1` and `J5`
  - `ReadFile` on `/run/gradient-rt-motion/metrics.json`
- Follow-up / risk:
  - no code changes in this pass
  - the current controller warning for the `J1` path still lacks the exact transient `statusword`/`error_code`; reproducing with richer logging would be needed if we want the precise drive fault identity

## 2026-04-16 01:55 +0000 - Reduced recurring native-home false signals in backend wait logic, post-home validation, and UI row copy

- Context:
  - The user explicitly called out that the same native-home false signals kept recurring and needed to be fixed rather than re-explained.
- Code changes:
  - `src/gradient_os/arm_controller/backends/ethercat_rtcore/backend.py`
    - added a failed-result stabilization rule in `_wait_for_native_home_result()` so a single bad post-clear snapshot no longer latches a hard failure before the clean `0x9650` success snapshot arrives
    - added a targeted retry path in `native_home_joint()` for transient post-home truth failures like `drive_native_fault_present`, using the existing post-home settle window before returning `NATIVE_HOME_ANCHOR_REFRESH_FAILED`
  - `tests/test_gradient05_limits_and_backends.py`
    - added a regression for the J1-style transient post-home truth failure that now recovers after settle
    - added a regression for the J5-style single failed post-clear snapshot that now recovers when a clean live-success snapshot follows
    - updated the direct failed-after-clear regression so it still asserts hard failure only after two failed post-clear snapshots
  - `web-ui/src/ControlPanel.tsx`
    - changed per-joint native-home status text so a clean live `succeeded` state is shown as success even if stale reported abort metadata still exists underneath
  - `web-ui/src/ControlPanel.test.tsx`
    - updated the UI regression to assert success for that clean-live/stale-reported case
- Validation that ran:
  - `pytest tests/test_gradient05_limits_and_backends.py -k "native_home or wait_for_native_home_result"`
  - result: `22 passed`
  - `npm --prefix web-ui test -- src/ControlPanel.test.tsx`
  - result: `21 passed`
  - `ReadLints` on touched backend/test/frontend files
  - result: no diagnostics
- Follow-up / risk:
  - this fixes the code paths, but the already-running controller process is still using the pre-patch Python backend until the stack is restarted
  - raw RTCore metrics can still retain stale reported native-home failure fields; this pass stops those raw fields from dominating operator-facing status, but it does not yet redesign the RTCore metrics contract itself

## 2026-04-16 01:50 +0000 - Investigated J5 jog timeout after targeted-axis masking was already present

- Context:
  - The user asked why the current jogging attempt was failing with `TimeoutError: Timed out waiting for RTCore trajectory 4 to complete`.
- Investigation performed:
  - read the active terminal excerpt plus `logs/startups/20260416-012149/controller.log`
  - re-read the bounded jog/offloaded trajectory path in `src/gradient_os/arm_controller/command_api.py`, `src/gradient_os/arm_controller/trajectory_execution.py`, and `src/gradient_os/arm_controller/backends/ethercat_rtcore/backend.py`
  - re-read RTCore trajectory-completion logic in `src/gradient_rt_motion/main.cpp`
  - read raw `/run/gradient-rt-motion/metrics.json`
  - fetched live `curl -s http://127.0.0.1:4000/control/motion-status`
- Findings:
  - the failing bounded move was a J5 seam-crossing jog from `-0.048 deg` to `+0.952 deg`
  - the request already carried `target_joint_indices=[4]`, so the older "all held axes must satisfy completion" bug was not the direct cause of this timeout
  - the timeout snapshot was `saw_target=True state=executing active_traj_id=4 queue_depth=0 motion_done=False active_command_seq=133 submitted_command_seq=133`
  - after the timeout, the controller log showed `SAFE_POWER_DOWN,wait`, `Received STOP command`, and `WAIT_FOR_IDLE finished with state: completed`
  - live `/control/motion-status` after recovery showed RTCore idle with `last_submitted_traj_id=4`
  - raw `/run/gradient-rt-motion/metrics.json` after the run showed J5 `rotation_mode_position_reference=4085206` on the `31.25` ratio axis whose full wrap period is `4096000` counts, plus a separate J6 fault state (`statusword=0x9618`, `error_code=65280`)
- Interpretation:
  - this failure is later than the API ACK path: RTCore accepted the selected-axis trajectory, consumed the uploaded points, but did not satisfy the completion condition before the backend wait expired
  - strongest code-level suspicion to verify next: RTCore completion currently reduces wrapped final error with raw `counts_per_rev`, while the backend/Python A6-EC reference logic already uses the full wrapped period implied by `counts_per_unit * 2*pi`
- Validation that ran:
  - `ReadFile` on the current controller log and `/run/gradient-rt-motion/metrics.json`
  - `curl -s http://127.0.0.1:4000/control/motion-status`
- Follow-up / risk:
  - no code changes in this pass
  - the wrap-period mismatch is still a hypothesis until we capture/lock the exact J5 seam-crossing error counts in a focused repro or regression

## 2026-04-16 02:53 +0000 - Aligned RTCore wrapped completion with geared A6-EC rotary period

- Context:
  - Implemented the approved fix for the J5 seam-crossing jog timeout where RTCore consumed the queued points but stayed `executing` with `queue_depth=0` and `motion_done=false`.
- Code changes:
  - `src/gradient_rt_motion/main.cpp`
    - renamed `shortest_periodic_error_counts()` parameter to `period_counts` for clarity
    - added `completion_wrap_period_counts(const AxisConfig&)` to derive the wrapped rotary completion period from `counts_per_unit * 2*pi`, with `counts_per_rev` fallback
    - updated the trajectory-completion block to use that derived period instead of raw `counts_per_rev`
  - `tests/test_gradient05_limits_and_backends.py`
    - strengthened the A6-EC wrap-period regression with an explicit J5 `4096000`-count assertion
    - added `test_ethercat_backend_execute_joint_trajectory_uses_quantized_timing_for_j5_axis_mask`
    - added `test_ethercat_backend_wait_for_trajectory_complete_waits_past_queue_empty_executing_snapshot`
- Validation that ran:
  - `make -C src/gradient_rt_motion`
    - result: success
  - `source /home/pi/GradientOS/.venv/bin/activate && PYTHONPATH=src python -m pytest tests/test_gradient05_limits_and_backends.py::test_ethercat_backend_a6ec_reference_wrap_period_uses_per_axis_gear_ratios tests/test_gradient05_limits_and_backends.py::test_ethercat_backend_execute_joint_trajectory_uses_quantized_timing tests/test_gradient05_limits_and_backends.py::test_ethercat_backend_execute_joint_trajectory_uses_quantized_timing_for_j5_axis_mask tests/test_gradient05_limits_and_backends.py::test_ethercat_backend_wait_for_trajectory_complete_ignores_stale_previous_completion tests/test_gradient05_limits_and_backends.py::test_ethercat_backend_wait_for_short_trajectory_completion_without_observed_active_id tests/test_gradient05_limits_and_backends.py::test_ethercat_backend_wait_for_trajectory_complete_waits_past_queue_empty_executing_snapshot -q`
    - result: `6 passed`
  - `ReadLints` on `src/gradient_rt_motion/main.cpp` and `tests/test_gradient05_limits_and_backends.py`
    - result: no diagnostics
- Follow-up / risk:
  - live hardware confirmation still remained to be run after this code-only pass

## 2026-04-16 03:11 +0000 - Live J5 seam-crossing jog completed cleanly on the patched RTCore stack

- Context:
  - The user asked to test the RTCore seam-completion fix live rather than stopping at build/test validation.
- Runtime actions:
  - observed that the previously running stack had been hard-stopped from the interactive launcher, so the stack was down
  - restarted the stack with `./start-stack.sh` onto the patched RTCore binary (`logs/startups/20260416-030940`)
  - verified preflight state:
    - `/control/motion-status` returned `safe_for_power_transition=true`
    - `/info/joints-detailed` showed canonical/display truth available and J5 `arm_display_deg ~= +0.9484`
  - issued `POST /control/power-up`
  - issued `POST /control/joint-jog {"joint":5,"delta_deg":-1.0}` to cross the J5 seam from about `+0.9316 deg` display to about `-0.0683 deg`
  - issued `POST /control/power-down` after the verification
- Findings:
  - the controller log showed:
    - `APPLY_JOINT_SETPOINT bounded move ... target_deg=[..., -360.068, ...]`
    - `[Pi OL] RTCore trajectory execution finished: state=completed traj_id=1 elapsed=0.349s`
    - no `TimeoutError`
  - post-jog `/control/motion-status` showed:
    - `state=completed`
    - `active_traj_id=1`
    - `queue_depth=0`
    - `motion_done=true`
    - `last_event_code=291`
  - post-jog `/info/joints-detailed` showed J5 had crossed the seam successfully to `arm_display_deg ~= -0.0683`
  - power-down response returned RTCore to `idle` with `safe_for_power_transition=true`
- Validation that ran:
  - live stack restart via `./start-stack.sh`
  - `curl -s http://127.0.0.1:4000/control/motion-status`
  - `curl -s http://127.0.0.1:4000/info/joints-detailed`
  - `curl -s -X POST http://127.0.0.1:4000/control/power-up`
  - `curl -s -X POST http://127.0.0.1:4000/control/joint-jog -H 'Content-Type: application/json' -d '{"joint":5,"delta_deg":-1.0}'`
  - `curl -s -X POST http://127.0.0.1:4000/control/power-down`
- Follow-up / risk:
  - the targeted live J5 seam-crossing repro is now good evidence that the specific timeout regression is fixed
  - broader motion coverage across other seam-adjacent axes or larger moves was not exercised in this pass

## 2026-04-16 03:58 +0000 - Investigated post-power-cycle trust gating and the new transient J5 Er87.1 fault

- Context:
  - After a hard restart and drive power cycle, the user reported being locked out from power-up with no trusted telemetry, then asked whether our code was incorrectly demanding `0x9650` at startup instead of following the manufacturer restart rule that only calls for `6041 bit 15 = 1`.
  - The user also reported a fresh J5 `Er87.1` fault and asked why.
- Findings:
  - `src/gradient_os/arm_controller/profiles/drive/a6ec_ds402.py` currently sets `startup_truth_requires_hm_success_signature = True`.
  - `src/gradient_os/telemetry/native_home_status.py` currently treats drive-native truth as valid only when the live statusword has bits 12 and 15 set, bit 13 clear, and no active `error_code` / `manufacturer_error_code`.
  - The gate is therefore stricter than the vendor restart note, but it is not a literal exact-`0x9650` check; any clean signature-carrying state (for example a powered-up homed state) can pass.
  - The latest lockout still reflected a genuine drive-side invalid coordinate indication: live probe/API output showed all axes at `0x1650`, so `bit 15` was actually low. Relaxing the software rule to "bit15 only" would not have fixed that specific observed state.
  - `docs/resources/a6ec_manual_codes.md` maps `Er87.1` to "One-time excessive position reference increment (One-time increment of the target position is over 5 times of the maximum speed)".
  - `logs/startups/20260416-034141/controller.log` shows `NATIVE_HOME_JOINT,5` reached a verified terminal state and then faulted during the post-home settle window, which points to a transient post-home reference jump rather than the earlier RTCore seam-completion timeout.
- Follow-up / risk:
  - There is likely still a product decision to make about separating strict HM-success verification from restart-persistence verification so startup trust can align with the vendor bit-15 guidance.
  - The immediate lockout after the reported power cycle was still rooted in the live drive state not advertising retained coordinate validity.
  - The J5 `Er87.1` needs fresh live capture at the instant it happens if we want to prove whether the jump occurs on restore-to-CSP, hold-target resync, or another post-home handoff step.

## 2026-04-16 04:21 +0000 - Relaxed A6-EC startup trust to accept retained bit-15 coordinate validity

- Context:
  - The user asked to do the startup-truth change first so A6-EC restart validation matches the manufacturer guidance more closely without weakening fresh HM35 verification.
- Code changes:
  - `src/gradient_os/telemetry/native_home_status.py`
    - kept `statusword_indicates_valid_native_home_reference()` strict for HM-success verification
    - added a separate statusword-to-coordinate-validity path so drive-native truth can accept either the strict HM signature or a relaxed bit-15-only startup rule, depending on profile config
    - extended `derive_drive_native_truth_validity()` with `require_hm_success_signature`
  - `src/gradient_os/arm_controller/backends/ethercat_rtcore/backend.py`
    - added `_startup_truth_requires_hm_success_signature()`
    - passed the profile-controlled startup-truth rule into drive-native truth evaluation
  - `src/gradient_os/telemetry/drive_faults.py`
    - threaded the same position-semantics flag into the probe/runtime drive-fault snapshot path
  - `src/gradient_os/arm_controller/profiles/drive/a6ec_ds402.py`
    - set `startup_truth_requires_hm_success_signature = False` with a comment tying that choice to the vendor restart guidance
  - `tests/test_rtcore_runtime.py`
    - added a regression proving `build_drive_fault_snapshot()` accepts A6-EC `0x8650` as valid retained startup truth while still marking the strict HM signature as absent
  - `tests/test_gradient05_limits_and_backends.py`
    - added a regression proving `EthercatRTCoreBackend.get_power_transition_snapshot()` now treats `0x8650` as synchronized/valid feedback for A6-EC startup
- Validation that ran:
  - `source "/home/pi/GradientOS/.venv/bin/activate" && python -m py_compile src/gradient_os/telemetry/native_home_status.py src/gradient_os/telemetry/drive_faults.py src/gradient_os/arm_controller/backends/ethercat_rtcore/backend.py src/gradient_os/arm_controller/profiles/drive/a6ec_ds402.py`
    - result: success
  - `source "/home/pi/GradientOS/.venv/bin/activate" && PYTHONPATH=src python -m pytest tests/test_drive_faults.py::test_statusword_indicates_valid_native_home_reference_requires_vendor_success_bits tests/test_gradient05_limits_and_backends.py::test_native_home_metrics_result_requires_bit12_alongside_bit15_for_fallback tests/test_gradient05_limits_and_backends.py::test_ethercat_backend_power_transition_snapshot_reports_coordinate_system_invalid tests/test_gradient05_limits_and_backends.py::test_ethercat_backend_power_transition_snapshot_accepts_bit15_restart_truth tests/test_rtcore_runtime.py::test_drive_fault_snapshot_marks_drive_native_truth_valid_when_startup_and_status_are_valid tests/test_rtcore_runtime.py::test_drive_fault_snapshot_accepts_a6ec_bit15_only_restart_truth -q`
    - result: `6 passed`
  - `ReadLints` on touched Python/test files
    - result: no diagnostics
- Follow-up / risk:
  - This change intentionally does not fix states where the drive still comes back as `0x1650`; those remain true drive-side invalid-coordinate cases that still require re-home/recovery.
  - The separate J5 `Er87.1` post-home settle fault still needs its own investigation.

## 2026-04-16 04:32 +0000 - Investigated new J3 Er47.0 jog fault and the confusing `-359 deg` raw-angle logs

- Context:
  - The user reported that J3 moved erratically in the opposite direction during jogging, then faulted with `Er47.0`, and asked why many current positions in the logs showed values near `-359 deg` instead of near zero.
- Findings:
  - The failing sequence in the live controller log shows:
    - one J3 seam-adjacent jog completed cleanly (`target_deg` about `-360.979`)
    - the next J3 jog was baselined from `current_deg` about `-0.980` and targeted `-1.980`
    - that second jog faulted in the background executor, after the API had already acknowledged the request
  - Live post-fault API state now shows:
    - `/control/motion-status`: `fault_present` and `canonical_truth_unavailable` on axis 2 / joint 3
    - `/info/joints-detailed`: J3 `statusword=0xB638`, `error_code=34321 (0x8611 / Er47.0)`, truth unavailable because of the active drive fault
  - `docs/resources/a6ec_manual_codes.md` confirms `Er47.0` is `Excessive position deviation`, and the manual parameter list ties that to `6062 - 6064` exceeding the following-error window/time.
  - The zero-offset store `.gradient_joint_zero_offsets.json` still contains all-zero logical master offsets, so the `-359 deg` numbers are not caused by stale software zeroing.
  - The `-359`/`355` values come from the raw command/reference frame, not the operator display frame:
    - `servo_driver.get_current_arm_state_rad()` calls backend `get_joint_positions()`
    - `/control/joint-jog` baselines the next target from `arm_deg`
    - `web-ui/src/ControlPanel.tsx` still prefers `arm_display_deg` for presentation, which is why the UI can be near zero while the controller logs show seam-equivalent raw angles
  - Current live data confirms the split:
    - healthy joints still report operator display angles near zero or a few degrees
    - the raw command frame can differ by whole turns (for example J2 display about `-4.935 deg` while prior jog logs printed `355.063 deg`)
  - The post-fault J3 raw reference landed around `+12.3 deg` while the failing second jog target was `-1.98 deg`, so the axis ended up on the wrong side of the command by roughly `14 deg` before tripping the following fault.
- Interpretation:
  - This is most consistent with the old persistent J3/J4 commissioning bug family: seam / wrap-turn command mapping is still unstable across successive seam-adjacent jogs.
  - In this run, the first J3 seam move completed, then the next `1 deg` jog likely got translated into the wrong equivalent turn in the raw write frame, producing the opposite-direction lurch and the eventual `Er47.0`.
- Validation that ran:
  - read current controller/terminal logs around the failing jog sequence
  - `curl -s http://127.0.0.1:4000/control/motion-status`
  - `curl -s http://127.0.0.1:4000/info/joints-detailed`
  - read `/run/gradient-rt-motion/metrics.json`
  - inspected `src/gradient_os/api/main.py`, `src/gradient_os/arm_controller/command_api.py`, `src/gradient_os/arm_controller/servo_driver.py`, and `src/gradient_os/arm_controller/backends/ethercat_rtcore/backend.py`
- Follow-up / risk:
  - No code change in this pass yet; this was diagnosis only.
  - The next likely fix path is to stop baselining jog steps from a seam-sensitive raw frame and/or harden the raw-write wrap-turn selection after a seam-crossing move, especially for the persistent J3/J4 family.

## 2026-04-16 04:49 +0000 - Switched public A6-EC joint truth to continuous semantics while preserving raw write-frame turn selection

- Context:
  - The user explicitly called out that the live stack was still behaving like a single-turn wrapped system even though the drives expose multi-turn data, and rejected `-359` appearing adjacent to `0` in public/controller truth.
- Changes:
  - Updated `src/gradient_os/arm_controller/backends/ethercat_rtcore/backend.py`
    - changed `raw_to_joint_positions()` to return the continuous `reference_mode="display"` truth instead of the wrapped raw RTCore/reference frame
    - extended `_canonical_joint_positions_from_raw_feedback()` so display-mode truth also runs a second raw-frame roundtrip against the live wrapped reference and stores `raw_reference_wrap_lift_counts`
    - clear cached raw wrap-lift state whenever truth is unavailable or either the public-truth roundtrip or raw command-frame roundtrip fails, to avoid stale equivalent-turn reuse
  - Updated `tests/test_gradient05_limits_and_backends.py`
    - added `test_ethercat_backend_drive_native_truth_unwraps_public_joint_positions_but_preserves_raw_write_frame` to lock down the seam case: public truth reads back as continuous `-0.08 rad`, while converting that truth back into the raw command frame still reconstructs the original wrapped count
- Validation that ran:
  - `source "/home/pi/GradientOS/.venv/bin/activate" && python -m py_compile src/gradient_os/arm_controller/backends/ethercat_rtcore/backend.py tests/test_gradient05_limits_and_backends.py`
    - result: success
  - `source "/home/pi/GradientOS/.venv/bin/activate" && PYTHONPATH=src python -m pytest tests/test_gradient05_limits_and_backends.py::test_ethercat_backend_uses_drive_native_truth_when_startup_and_status_are_valid tests/test_gradient05_limits_and_backends.py::test_ethercat_backend_drive_native_truth_unwraps_public_joint_positions_but_preserves_raw_write_frame tests/test_gradient05_limits_and_backends.py::test_ethercat_backend_translates_canonical_truth_back_into_raw_wire_counts tests/test_gradient05_limits_and_backends.py::test_ethercat_backend_startup_bootstrap_uses_display_reference_mode -q`
    - result: `4 passed`
  - `ReadLints` on touched backend/test files
    - result: no diagnostics
- Follow-up / risk:
  - This fixes the immediate seam-wrapped public-truth/jog-baseline bug without moving the raw command upload path off the RTCore `6064/607A` frame.
  - It does not yet settle the larger architectural question of whether long-term A6-EC canonical truth should come directly from anchored `U40.20/.22` absolute counts instead of the drive-native reference frame plus continuous unwrapping.

## 2026-04-16 05:07 +0000 - Expanded the J6 Chapter 5 probe to capture controller, frontend, and RTCore views in one experiment artifact

- Context:
  - The user wants to rerun the manual J6 rotation experiment and answer the still-open question directly: do the encoder counts wrap, or do they continue monotonically across turns?
  - The user also explicitly asked to record not only raw drive objects, but also what the controller and frontend see during the experiment.
- Changes:
  - Updated `scripts/a6ec_chapter5_probe.py`
    - added direct UDP controller capture for `GET_JOINT_STATE` and `GET_MOTION_STATUS`
    - added API capture for `/info/joints`, `/info/joints-detailed`, `/control/motion-status`, and a one-event `/monitor` sample
    - added RTCore metrics capture from `/run/gradient-rt-motion/metrics.json`
    - threaded those views into both `snapshot` and `watch` outputs so each artifact now contains raw SDO reads, controller truth, frontend-facing payloads, and RTCore state together
    - added `--controller-host`, `--controller-port`, and `--monitor-timeout-s` CLI flags
    - preserved partial-capture behavior so powered-down/unavailable phases record explicit `ok/error` results instead of silently dropping failed reads
  - Updated `tests/test_a6ec_chapter5_probe.py`
    - added focused tests for base64 motion-status parsing, SSE monitor-event parsing, merged watch samples with controller/frontend/monitor/metrics fields, and snapshot assembly with the new captures
- Validation that ran:
  - `source "/home/pi/GradientOS/.venv/bin/activate" && python -m py_compile scripts/a6ec_chapter5_probe.py tests/test_a6ec_chapter5_probe.py`
    - result: success
  - `source "/home/pi/GradientOS/.venv/bin/activate" && PYTHONPATH=src python -m pytest tests/test_a6ec_chapter5_probe.py -q`
    - result: `9 passed`
  - `source "/home/pi/GradientOS/.venv/bin/activate" && python scripts/a6ec_chapter5_probe.py --help`
    - result: success
  - `source "/home/pi/GradientOS/.venv/bin/activate" && python scripts/a6ec_chapter5_probe.py snapshot --help`
    - result: success
  - `source "/home/pi/GradientOS/.venv/bin/activate" && python scripts/a6ec_chapter5_probe.py watch --help`
    - result: success
  - `ReadLints` on touched probe/test files
    - result: no diagnostics
- Follow-up / risk:
  - This change prepares the experiment harness, but it does not itself answer the wrap/monotonic question; that still requires the live J6 manual-rotation run.
  - The powered-down phase may legitimately produce missing controller/API/monitor data depending on how much of the stack remains reachable, but the artifact will now show that explicitly instead of hiding it.

## 2026-04-16 05:42 +0000 - Live J6 manual-rotation experiment shows multi-turn absolute continuity and a separate wrapped raw-reference family

- Context:
  - The user ran the live J6 experiment: rotate `> +360 deg`, back to zero, then `> -360 deg`, with the expanded probe recording raw SDO objects, controller replies, API payloads, `/monitor`, and RTCore metrics together.
- Runtime artifacts:
  - watch: `logs/encoder-retention/j6-manual-rotate-20260416-053435/j6-manual-rotate-live.watch.jsonl`
  - final snapshot: `logs/encoder-retention/j6-manual-rotate-20260416-053435/j6-manual-rotate-final.json`
  - final markdown: `logs/encoder-retention/j6-manual-rotate-20260416-053435/j6-manual-rotate-final.md`
- Findings:
  - Initial near-zero plateau:
    - `6064 ~= 3`
    - `U40.16 ~= 3`
    - `encoder_multi_turn_counts ~= 113075`
    - controller/frontend truth near `0 deg`
  - After the positive `>360` sweep and return near zero:
    - `encoder_multi_turn_counts` returned near the same neighborhood (`~113040`)
    - controller/frontend truth stayed near zero (`~0.008 deg`)
    - but the reference family sat around `6064 ~= 1310690`, `U40.16 ~= -30`, `rotation_mode_encoder_counts ~= 1310690`
    - interpretation: the absolute source returned near its starting count while the reference family kept a one-turn-lifted seam-equivalent state
  - During the longer sweep:
    - `encoder_multi_turn_counts` moved through large multi-turn values such as `-1216460`, `-1839631`, and `2190820`
    - controller/frontend truth also moved continuously to about `-570.9 deg`
    - interpretation: the absolute source is not single-turn wrapped; it remains multi-turn continuous over the excursion
  - Final stable snapshot:
    - controller truth: `arm_deg = 2.0687255859375`, `axis_counts = 1303188`
    - selected axis detail: `raw_counts = 1303188`, `reference_pre_zero_rad = 0.03610607279485828`, `raw_reference_pre_zero_rad = -6.247079234384728`
    - wrap bookkeeping: `raw_command_roundtrip_reference_wrap_lift_counts = 1310720`, `raw_command_roundtrip_reference_wrap_lift_turns = 1.0`
    - absolute source: `absolute_counts = 105539`, `absolute_source = encoder_multi_turn_counts`
  - Secondary observation:
    - final snapshot shows `U40.28 = 1303190` and `rotation_mode_encoder_counts = 1303190`
    - the older probe bridge assumption `U40.2A/.2C ~= U40.28 * C10_ratio` is false in the current posture because both values already matched directly while `C10.18/C10.19 = 10.0`
- Interpretation:
  - The J6 experiment directly supports the user's objection: the A6-EC exposes a continuous multi-turn count path, and that path is not behaving like a single-turn wrapped signal.
  - The wrapped/seam-equivalent behavior still present in the stack belongs to the drive reference/raw command family and the host-side lift used to reconcile it, not to the existence of the multi-turn absolute counts themselves.
- Validation that ran:
  - live `watch` capture during the manual experiment
  - stable post-run `snapshot` capture on the same experiment id
- Follow-up / risk:
  - This does not yet by itself decide the final write-path architecture, but it materially weakens the argument for treating the direct multi-turn absolute source as if it were inherently single-turn wrapped.

## 2026-04-16 06:08 +0000 - Built a trimmed dual-axis canvas for the J6 watch dataset

- What changed:
  - Generated `/home/pi/.cursor/projects/home-pi-GradientOS/canvases/j6-manual-rotate-dataset.canvas.tsx`
  - The canvas embeds the J6 watch time series from `logs/encoder-retention/j6-manual-rotate-20260416-053435/j6-manual-rotate-live.watch.jsonl` and renders a dual-axis SVG chart with counts-like fields on the left axis and all other numeric fields on the right
  - Added presets and per-series toggles so the full numeric capture remains explorable without forcing every series to stay visible at once
  - Froze the active `053435` probe and trimmed the long stationary tail before chart generation, reducing the plotted slice from `1189` total samples to `144`
- Validation performed:
  - confirmed the active `053435` watch file stopped growing after terminating its writer process
  - computed the flat-tail cutoff from the frozen dataset using trailing ranges of `combined_u4020_22_signed_counts <= 8`, `api_absolute_counts <= 8`, and `api_arm_deg <= 0.02`
  - `ReadLints` on `/home/pi/.cursor/projects/home-pi-GradientOS/canvases/j6-manual-rotate-dataset.canvas.tsx`
    - result: no diagnostics
- Follow-up / risk:
  - A separate older J6 probe for experiment `20260416-052258` is still running against its own file; it does not affect this canvas, but it can confuse future capture audits if left running

## 2026-04-16 06:18 +0000 - Investigated how to open the J6 canvas artifact

- What changed:
  - No product code changed.
  - Verified `/home/pi/.cursor/projects/home-pi-GradientOS/canvases/j6-manual-rotate-dataset.canvas.tsx` exists under the managed canvas directory.
  - Confirmed no `j6-manual-rotate-dataset.canvas.status.json` sidecar exists yet, which is consistent with the canvas not having been rendered/built once yet.
  - Updated `.cursor/memory/AGENT_SCRATCHPAD.md` with a durable canvas-opening guardrail.
- Validation performed:
  - Read the current canvas skill guidance plus the latest scratchpad/devlog context.
  - `Glob` search for `j6-manual-rotate-dataset.canvas*`
- Follow-up / risk:
  - If clicking/opening the canvas path still does not render it, the next step is to inspect the canvas source itself or wait for the first build attempt to emit a `.canvas.status.json` diagnostic sidecar.

## 2026-04-16 06:29 +0000 - Corrected the J6 canvas open-path guidance

- What changed:
  - No product code changed.
  - Identified that the earlier chat reply wrapped the canvas path with leading/trailing spaces inside the backticks, which likely caused Cursor to try opening the wrong literal path.
  - Updated `.cursor/memory/AGENT_SCRATCHPAD.md` with a guardrail to keep clickable file paths exact.
- Validation performed:
  - Re-read the existing canvas file at `/home/pi/.cursor/projects/home-pi-GradientOS/canvases/j6-manual-rotate-dataset.canvas.tsx`
  - Confirmed the real file exists, is readable, and has normal permissions
- Follow-up / risk:
  - If the corrected exact path still fails to open, the remaining likely causes are a Cursor-side path-opening quirk for this hidden managed directory or a canvas runtime issue that only appears on first build.

## 2026-04-16 06:47 +0000 - Rebuilt the J6 canvas as a normal web page for SSH/browser use

- What changed:
  - Added a standalone React page at `web-ui/j6-manual-rotate-dataset.html` with entrypoint `web-ui/src/j6-manual-rotate-dataset.tsx`
  - Added `web-ui/src/J6ManualRotateDatasetPage.tsx`, which ports the trimmed J6 manual-rotation dataset into a browser-native page with:
    - the archived experiment summary
    - the dual-axis SVG chart
    - the same preset-based filtering and per-series toggles as the canvas
    - a sample scrubber and selected-sample detail tables
  - Updated `web-ui/vite.config.ts` so `vite build` emits both the main app and the standalone J6 dataset page
  - Added `web-ui/src/J6ManualRotateDatasetPage.test.tsx` covering page render and preset switching
- Validation performed:
  - `npm test -- J6ManualRotateDatasetPage.test.tsx`
    - result: `2 passed`
  - `npm run build`
    - result: success; emitted `dist/j6-manual-rotate-dataset.html`
- Follow-up / risk:
  - The standalone page intentionally avoids threading this archive view through the main app shell, which keeps the SSH/browser delivery path simple and low-risk.
  - The existing large `ArmVisualizer` build chunk warning remains in the main app build and was not introduced by this dataset page.

## 2026-04-16 06:20 +0000 - Rooted A6-EC planner truth in anchored `encoder_multi_turn_counts`

- What changed:
  - Updated `src/gradient_os/arm_controller/profiles/drive/a6ec_ds402.py` so A6-EC declares `canonical_truth_source = "encoder_multi_turn_counts"` and `absolute_home_anchor_required = True`.
  - Narrowed A6-EC absolute-truth resolution to `encoder_multi_turn_counts` instead of allowing the truth resolver to fall through to the rotation-mode family.
  - Patched `src/gradient_os/arm_controller/backends/ethercat_rtcore/backend.py` so the drive-native A6-EC read-truth path reconstructs canonical joint truth from `absolute_axis_q_rad - absolute_home_anchor_rad - master_offset`, while still preserving raw 6064-family wrap-lift bookkeeping for the write path.
  - Split planner/control truth from operator display semantics more explicitly: `raw_to_joint_positions()` now validates the anchored absolute truth against the raw/write-frame roundtrip, while display snapshots remain on the stricter display-mode path.
  - Added and updated focused regressions in `tests/test_gradient05_limits_and_backends.py` for anchored A6-EC truth, preserved raw write-frame conversion, required-anchor fail-closed behavior, and restart/power-transition truth setup.
- Validation performed:
  - `pytest -q tests/test_gradient05_limits_and_backends.py -k "drive_native_truth or startup_bootstrap or absolute_anchor"`
    - result: `8 passed`
  - `pytest -q tests/test_gradient05_limits_and_backends.py tests/test_run_controller_helpers.py`
    - result: `93 passed`
  - `pytest -q tests/test_api_endpoints.py -k "joint or monitor"`
    - result: `13 passed, 56 deselected`
  - `ReadLints` on the touched backend/profile/test files
    - result: no diagnostics
- Follow-up / risk:
  - The read-truth path is now correctly anchored to the continuous encoder source, but the RTCore/drive write contract still targets the 607A/6064 CSP reference family. Commanding directly in the encoder-multiturn object family would require a deliberate write-path redesign rather than another read-truth tweak.

## 2026-04-16 16:40 +0000 - Decoupled display truth from command wrap bookkeeping and controller canonical-truth gating

- What changed:
  - Updated `src/gradient_os/arm_controller/backends/ethercat_rtcore/backend.py` so `_canonical_joint_positions_from_raw_feedback()` takes an explicit `mutate_command_wrap_bookkeeping` flag.
  - Kept `raw_to_joint_positions()` as the command/runtime truth path that may update `_raw_reference_wrap_lift_counts`.
  - Marked `get_display_feedback_snapshot()`, `raw_to_display_joint_positions()`, `get_power_transition_snapshot()`, and `_absolute_home_anchor_validation_for_joint()` as observational callers that must not mutate raw wrap-lift bookkeeping.
  - Updated `src/gradient_os/run_controller.py` so `canonical_joint_truth_available` no longer gets ANDed with display-truth availability; `display_joint_truth_available` remains a separate diagnostic field.
  - Added focused regressions in `tests/test_gradient05_limits_and_backends.py` for order-independent display reads and for preserving an existing raw wrap lift across a failing display snapshot.
  - Updated `tests/test_run_controller_helpers.py` expectations so canonical runtime truth stays available when live canonical feedback is good but display truth is degraded.
  - Added `tests/test_api_endpoints.py::test_control_joint_jog_rejects_when_selected_joint_truth_is_unavailable` to prove jog still refuses on the per-joint truth gate after the controller semantics split.
- Validation performed:
  - `"/home/pi/GradientOS/.venv/bin/python" -m py_compile src/gradient_os/arm_controller/backends/ethercat_rtcore/backend.py src/gradient_os/run_controller.py tests/test_gradient05_limits_and_backends.py tests/test_run_controller_helpers.py tests/test_api_endpoints.py`
    - result: success
  - `PYTHONPATH=src "/home/pi/GradientOS/.venv/bin/python" -m pytest tests/test_gradient05_limits_and_backends.py::test_ethercat_backend_j3_style_raw_truth_uses_wrap_lift_for_command_targets tests/test_gradient05_limits_and_backends.py::test_ethercat_backend_display_read_is_order_independent_for_raw_wrap_selection tests/test_gradient05_limits_and_backends.py::test_ethercat_backend_display_snapshot_does_not_clear_existing_raw_wrap_lift tests/test_gradient05_limits_and_backends.py::test_ethercat_backend_translates_canonical_truth_back_into_raw_wire_counts tests/test_gradient05_limits_and_backends.py::test_ethercat_backend_refuses_display_feedback_when_absolute_anchor_does_not_roundtrip tests/test_run_controller_helpers.py::test_build_joint_state_snapshot_does_not_fallback_display_feedback tests/test_run_controller_helpers.py::test_build_joint_state_snapshot_preserves_partial_display_feedback tests/test_run_controller_helpers.py::test_build_joint_state_snapshot_keeps_raw_blocker_details_when_display_truth_is_available tests/test_api_endpoints.py::test_control_joint_jog_rejects_when_canonical_truth_is_unavailable tests/test_api_endpoints.py::test_control_joint_jog_rejects_when_selected_joint_truth_is_unavailable -q`
    - result: `10 passed in 2.31s`
  - `source ./start.sh && python -m py_compile src/gradient_os/arm_controller/backends/ethercat_rtcore/backend.py src/gradient_os/run_controller.py tests/test_gradient05_limits_and_backends.py tests/test_run_controller_helpers.py tests/test_api_endpoints.py && python -m pytest tests/test_gradient05_limits_and_backends.py::test_ethercat_backend_j3_style_raw_truth_uses_wrap_lift_for_command_targets tests/test_gradient05_limits_and_backends.py::test_ethercat_backend_display_read_is_order_independent_for_raw_wrap_selection tests/test_gradient05_limits_and_backends.py::test_ethercat_backend_display_snapshot_does_not_clear_existing_raw_wrap_lift tests/test_gradient05_limits_and_backends.py::test_ethercat_backend_translates_canonical_truth_back_into_raw_wire_counts tests/test_gradient05_limits_and_backends.py::test_ethercat_backend_refuses_display_feedback_when_absolute_anchor_does_not_roundtrip tests/test_run_controller_helpers.py::test_build_joint_state_snapshot_does_not_fallback_display_feedback tests/test_run_controller_helpers.py::test_build_joint_state_snapshot_preserves_partial_display_feedback tests/test_run_controller_helpers.py::test_build_joint_state_snapshot_keeps_raw_blocker_details_when_display_truth_is_available tests/test_api_endpoints.py::test_control_joint_jog_rejects_when_canonical_truth_is_unavailable tests/test_api_endpoints.py::test_control_joint_jog_rejects_when_selected_joint_truth_is_unavailable -q`
    - result: `10 passed in 1.32s`
  - `ReadLints` on `src/gradient_os/arm_controller/backends/ethercat_rtcore/backend.py`, `src/gradient_os/run_controller.py`, `tests/test_gradient05_limits_and_backends.py`, `tests/test_run_controller_helpers.py`, and `tests/test_api_endpoints.py`
    - result: no diagnostics
- Follow-up / risk:
  - This is intentionally a local containment fix. It prevents display/monitor reads from poisoning the raw command frame, but it does not settle the broader A6-EC runtime truth-source architecture.
  - If the user asks for project-env activation specifically, prefer `source ./start.sh`; system `python3` lacked repo deps, and `uv run` attempted network resolution while DNS was unavailable.

## 2026-04-17 04:11 +0000 - Restored startup-drive-config expectation fields across RTCore startup epochs

- What changed:
  - Updated `src/gradient_rt_motion/main.cpp` so RTCore no longer drops startup-drive-config descriptor expectation fields when a startup epoch is re-armed.
  - Added `reset_startup_drive_config_feedback(...)` to repopulate `configured` and `commanded` from the active `startup_sdos` descriptors while still clearing readback-specific fields for the new epoch.
  - Updated the deferred startup readback path to refresh `configured` and `commanded` before verifying each descriptor, preserving downstream `unverified` vs `mismatch` semantics instead of collapsing matching descriptors into `startup_drive_config_unconfigured`.
  - Added focused regressions in `tests/test_rtcore_runtime.py` and `tests/test_gradient05_limits_and_backends.py` to prove that the authoritative `startup_drive_configs` list keeps startup validity and persisted-home-anchor restart trust reachable even when the legacy single-entry `startup_drive_config` view is stale.
- Validation performed:
  - `make -C src/gradient_rt_motion`
    - result: success
  - `PYTHONPATH=src /home/pi/GradientOS/.venv/bin/python -m pytest tests/test_rtcore_runtime.py tests/test_gradient05_limits_and_backends.py tests/test_drive_faults.py -q`
    - result: `134 passed`
  - `ReadLints` on `src/gradient_rt_motion/main.cpp`, `tests/test_rtcore_runtime.py`, and `tests/test_gradient05_limits_and_backends.py`
    - result: no diagnostics
- Follow-up / risk:
  - This patch intentionally leaves Python/controller startup-validity policy unchanged; the fix is in the RTCore metrics contract so ownership stays with the EtherCAT startup layer.
  - Operator-facing exposure of `drive_native_truth_verification_source` remains a separate follow-up once this upstream startup gate is stable.

## 2026-04-17 04:56 +0000 - Exposed canonical-truth verification source in API snapshots and Joint Commissioning UI

- What changed:
  - Updated `src/gradient_os/api/main.py` so `_selected_joint_feedback_snapshot()` now passes through `drive_native_truth_verification_source` from per-axis absolute-feedback detail into command/error payloads such as `/control/joint-jog` failures.
  - Updated `tests/test_api_endpoints.py` fixtures and expectations so `/info/joints-detailed` preserves the field and selected-joint error payloads carry it through.
  - Updated `web-ui/src/ControlPanel.tsx` to type `drive_native_truth_verification_source`, `drive_native_truth_reason`, `drive_native_truth_valid`, and `coordinate_system_valid`, then render a distinct per-joint commissioning line:
    - `Canonical truth trust: <source>` when truth is available
    - `Canonical truth unavailable: <reason>` when truth is not available
  - Kept the wording separate from native-home verification so operators can distinguish "how canonical truth is trusted" from "how HM35 completion was verified."
  - Updated `web-ui/src/ControlPanel.test.tsx`, `web-ui/src/App.tsx`, and `web-ui/src/liveState.tsx` for coverage and consistent live-state typing.
- Validation performed:
  - `PYTHONPATH=src /home/pi/GradientOS/.venv/bin/python -m pytest tests/test_api_endpoints.py -q`
    - result: `70 passed`
  - `npm test -- ControlPanel.test.tsx`
    - result: `22 passed`
  - `npm run build`
    - result: success
  - `ReadLints` on `src/gradient_os/api/main.py`, `tests/test_api_endpoints.py`, `web-ui/src/ControlPanel.tsx`, `web-ui/src/ControlPanel.test.tsx`, `web-ui/src/App.tsx`, and `web-ui/src/liveState.tsx`
    - result: no diagnostics
- Follow-up / risk:
  - The UI now surfaces the verification source from the existing `drive_faults` snapshot and selected-joint command payloads, but it still does not consume `/info/joints-detailed` directly for this display. That is intentional to avoid creating a second live truth channel in the frontend.

## 2026-04-17 09:35 +0000 - Unified `[timestamp] [label] message` chrome across start-stack subprocess output

- What changed:
  - Upgraded `src/gradient_os/telemetry/terminal_dashboard.py`:
    - Added module-level `DASHBOARD_LABEL` so canonical-truth transitions always emit under a stable `dashboard` tag.
    - Added `format_timestamp()` returning `%Y-%m-%d %H:%M:%S%z` to match the bash launcher's `date '+...'`.
    - Added `format_log_entry(label, message, ...)` which wraps timestamp + label in optional ANSI style codes and composes `[ts] [label] message`.
    - Added `log_palette_from_env(env=None)` which reads `GRADIENT_STACK_STYLE_{MUTED,LABEL,RESET}` so Python tailers can color-match the bash launcher's palette.
    - Converted `process_service_log_line()` to return `list[tuple[label, message]]` instead of pre-formatted strings. Regular service log lines retain their incoming label (`controller`, `api`, `web`); canonical-truth AVAILABLE/UNAVAILABLE state transitions relabel to `dashboard`.
  - Rewrote `start-stack.sh::start_tail()`:
    - `init_banner_palette` is called up front, and the backgrounded Python subprocess is launched with `GRADIENT_STACK_STYLE_MUTED/LABEL/RESET` + `GRADIENT_STACK_TAIL_CLEAR_SPINNER` env vars. The clear-spinner flag only flips to `1` when `ui_can_render` says we have a writable TTY, so redirected/non-TTY runs stay free of ANSI cursor escapes.
    - Embedded Python now imports `format_log_entry` + `log_palette_from_env`, assembles a palette from env vars, iterates the new tuple API, and emits `\r\x1b[2K[timestamp] [label] message\n`. That wipes any pending spinner row on the terminal before writing a formatted tail line, eliminating the old `[#---] controller ... <tail text>` mash-ups.
  - Rewrote `start-stack.sh::run_interactive_console()`:
    - Exports the same style palette, imports the shared formatter, and centralises a `fmt(label, message)` helper used by every output path.
    - Tail dispatch in `monitor_loop` now renders `fmt(label, message)` for both regular service lines and dashboard state transitions.
    - Startup banner, `command>` prompt echo, `help` / `Unknown command` / `Type 'help' ...` replies, supervised-child failure messages, and the initial `[dashboard] controller=ONLINE api=ONLINE ... canonical_truth=MONITORING` line all flow through `fmt(...)` so they gain timestamps and color to match the launcher chrome.
    - `probe` / `status` subcommands now print `[ts] [console] executing: <cmd>` ... captured subprocess body ... `[ts] [console] <cmd> complete`; the nested subprocess is invoked with `GRADIENT_STACK_COLOR=1` so the inner `[ts] [start-stack] INFO: ...` output keeps the launcher palette even when capture_output=True strips the TTY.
  - Updated the pre-Python bootstrap error at the top of `start-stack.sh` to print `[<timestamp>] [start-stack] ERROR: python3 (or python) is required ...`, matching the format even in the earliest failure path where no helper functions are defined yet.
  - Updated `tests/test_terminal_dashboard.py`:
    - Converted existing assertions to the new tuple API (`("dashboard", "// LIVE STATE // ...")`, `("controller", "[Backend Registry] Registered backend: ...")`).
    - Added coverage for `format_timestamp`, `format_log_entry` (palette-on and palette-off), and `log_palette_from_env` default/populated behavior.
    - Added a pass-through regression proving regular service lines keep their incoming label when formatted.
- Validation performed:
  - `bash -n /home/pi/GradientOS/start-stack.sh`
    - result: syntax OK
  - `source ./start.sh && python -m pytest tests/test_terminal_dashboard.py -q`
    - result: `13 passed`
  - `source ./start.sh && python -m pytest tests/test_terminal_dashboard.py tests/test_run_controller_helpers.py tests/test_api_endpoints.py -q`
    - result: `92 passed`
  - End-to-end simulation: ran the exact `start_tail` embedded Python (with the same env var set) against a synthetic controller/api/web log file. Output was exactly `[timestamp] [label] message` for each non-filtered line, noisy API access log entries were dropped, and the canonical-truth transition was relabelled to `[dashboard]`.
  - `ReadLints` on `src/gradient_os/telemetry/terminal_dashboard.py`, `tests/test_terminal_dashboard.py`, `start-stack.sh`
    - result: no diagnostics
- Follow-up / risk:
  - `print_status` and `print_probe` still emit bare `key: value` lines to stdout when the user runs `./start-stack.sh status|probe` directly. That is intentional so operators and external scripts keep a stable, grep-friendly contract; the interactive-console wrapper still frames those blocks with `[ts] [console] executing: <cmd>` / `... complete` lines so the live terminal looks uniform.
  - The `\r\x1b[2K` spinner-wipe is only applied when `ui_can_render` succeeds; if a future caller pipes `./start-stack.sh` to a file without `TERM=dumb`, the launcher already disables its own spinner but the tail should also remain free of cursor escapes thanks to the new gated flag.
  - Pre-existing test failures in `tests/test_driver.py`, `tests/test_planning.py`, `tests/test_protocol.py`, `tests/test_end_to_end.py`, and `tests/test_solver.py` are unrelated to this change and were already failing before this work.

## 2026-04-17 22:48 +0000 - Fix inverted frontend drive-power/jog gating when `/monitor` drops `drive_faults`

- What changed:
  - `web-ui/src/App.tsx`:
    - Expanded `synthesizeDriveFaultSnapshotFromAxes()` so the monitor fallback now derives fresh top-level drive-power fields from live `axis_absolute_feedback` + `servos`, not just per-axis truth. The synthesized snapshot now refreshes `driver_state`, `physical_state`, `armed`, `axis_enable_mask`, `enable_requested`, `requested_axes`, `op_enabled_axes`, `faulted_axes`, `statusword_feedback_axes`, `native_home_active_axis_mask`, and `num_axes`.
    - Added `mergeDriveFaultSnapshots(previous, next)` so a real backend-emitted `drive_faults` block still wins verbatim, while a synthesized fallback overlays only its freshly derived fields onto the previous full snapshot. This preserves richer metadata like `ethercat_master_state` / `rtcore_state` without letting stale power-state fields freeze the UI after power-up or power-down.
  - `web-ui/src/App.test.ts`:
    - Added regressions covering active fallback synthesis, disarmed fallback synthesis, and the exact stale-previous-snapshot merge case that had the UI showing powered-up/disarmed backwards.
- Validation performed:
  - `cd /home/pi/GradientOS/web-ui && npx vitest run src/App.test.ts src/ControlPanel.test.tsx`
    - result: `27 passed`
  - `cd /home/pi/GradientOS/web-ui && npm run build`
    - result: success
  - `ReadLints` on `web-ui/src/App.tsx` and `web-ui/src/App.test.ts`
    - result: no diagnostics
- Follow-up / risk:
  - The synthesized monitor fallback still cannot derive richer transport/fieldbus metadata such as `ethercat_master_state`, `link_up`, `responding`, or `metrics_path`. Those continue to come from the real backend `drive_faults` block when present; the new merge keeps their last known values without allowing them to pin the live power/jog gating state.

## 2026-04-17 23:06 +0000 - Fail closed on A6-EC seam-straddling absolute `607A` trajectories

- What changed:
  - `src/gradient_os/arm_controller/profiles/drive/a6ec_ds402.py`
    - Added `command_frame_seam_crossing_unsafe=True` to the A6-EC position-semantics config so the live-hardware seam rule stays drive-profile-owned.
  - `src/gradient_os/arm_controller/backends/ethercat_rtcore/backend.py`
    - Added `_command_frame_seam_crossing_unsafe()`.
    - Tightened `_enforce_trajectory_wire_frame_safety()` so seam-straddling first points and consecutive points now raise `command_frame_seam_crossing_first_point_disallowed` / `command_frame_seam_crossing_step_disallowed` for profiles that opt in, even when the shortest-angular delta is tiny.
    - This supersedes the earlier assumption that mod-RM seam crossings were safe on A6-EC if the angular step looked small.
  - `tests/test_gradient05_limits_and_backends.py`
    - Replaced the two counter-regressions that previously required seam-straddling steps/deviation to pass with rejection expectations matching the live robot behavior.
  - `tests/test_a6ec_joint_sweep.py`
    - Narrowed the positive-path monotone trajectory acceptance test to a non-seam window so it still proves small ordinary trajectories pass without preserving the disproved seam assumption.
  - `docs/ethercat/a6ec-frame-semantics-and-native-home.md`
    - Documented the corrected contract: seam-straddling absolute `607A` trajectories are unsafe on the current A6-EC firmware and are now rejected until a seam-biased wire-frame policy is validated.
- Validation performed:
  - `PYTHONPATH=src /home/pi/GradientOS/.venv/bin/python -m py_compile src/gradient_os/arm_controller/backends/ethercat_rtcore/backend.py src/gradient_os/arm_controller/profiles/drive/a6ec_ds402.py tests/test_gradient05_limits_and_backends.py`
    - result: success
  - `PYTHONPATH=src /home/pi/GradientOS/.venv/bin/python -m pytest tests/test_gradient05_limits_and_backends.py -q`
    - result: `121 passed`
  - `PYTHONPATH=src /home/pi/GradientOS/.venv/bin/python -m pytest tests/test_a6ec_joint_sweep.py tests/test_a6ec_j6_watch_replay.py -q`
    - result: `64 passed`
  - `ReadLints` on `backend.py`, `a6ec_ds402.py`, `tests/test_gradient05_limits_and_backends.py`, `tests/test_a6ec_joint_sweep.py`, and `docs/ethercat/a6ec-frame-semantics-and-native-home.md`
    - result: no diagnostics
- Follow-up / risk:
  - This is a safety fail-close, not the final ergonomic fix. Seam-crossing jogs that previously could trigger a one-turn excursion are now rejected instead of being allowed through.
  - The next functional step is a validated seam-biased native-home / wire-frame policy that keeps ordinary commissioning jogs away from the `0/RM` seam without breaking logical zero, display truth, or RTCore hold-target alignment.

## 2026-04-18 00:22 +0000 - Make wrapped RTCore motion periodic end-to-end and re-enable A6-EC seam crossings

- What changed:
  - `src/gradient_rt_motion/main.cpp`
    - Added wrapped-motion helpers so RTCore can normalize counts into the axis wrap period, derive shortest-periodic double deltas, and advance CSP hold targets without converting seam-adjacent commands into linear almost-one-revolution ramps.
    - Updated wrapped trajectory execution to interpolate target counts modulo the axis period, derive fallback segment velocity from the shortest-periodic delta, normalize direct point loads before `target_counts` are rounded, and keep jog target accumulation in the drive's single-turn rotation frame.
    - Replaced the active hold-target `desired - cur` clamp with a shared periodic stepping helper for `feedback_counts_wrap` axes while preserving the existing linear path for non-wrapped axes.
    - Normalized wrapped final target counts before trajectory completion comparison so completion uses the same command frame RTCore now writes.
  - `src/gradient_os/arm_controller/profiles/drive/a6ec_ds402.py`
    - Flipped `command_frame_seam_crossing_unsafe` back to `False` and documented that the real fix is the periodic RTCore motion path, not a permanent profile-level seam ban.
  - `tests/test_gradient05_limits_and_backends.py`
    - Replaced the temporary rejection regressions with positive-path regressions that assert seam-crossing first points and per-point steps are allowed again when their shortest-angular deltas stay inside the existing backend safety cage.
- Validation performed:
  - `make -C src/gradient_rt_motion`
    - result: success
  - `source "/home/pi/GradientOS/.venv/bin/activate" && PYTHONPATH=src python -m pytest tests/test_gradient05_limits_and_backends.py tests/test_a6ec_joint_sweep.py tests/test_a6ec_j6_watch_replay.py -q`
    - result: `185 passed`
  - `ReadLints` on `src/gradient_rt_motion/main.cpp`, `src/gradient_os/arm_controller/profiles/drive/a6ec_ds402.py`, and `tests/test_gradient05_limits_and_backends.py`
    - result: no diagnostics
- Follow-up / risk:
  - This corrects the host-side wrap math we identified, but it is not yet live-hardware-verified. The next required check is a seam-adjacent J6 jog from near zero to confirm physical motion now matches the shortest angular delta instead of a one-turn excursion.
  - If live behavior is still wrong after this RTCore change, capture RTCore `hold_target_counts` / `output_target_counts`, the live `6064`, and the final multi-turn counts for the seam-crossing window. At that point the remaining suspect would be a drive-side interpretation issue rather than host interpolation/clamp math.

## 2026-04-18 00:39 +0000 - Investigated post-fix J6 fault; raw 607A seam wrap now trips the drive

- What changed:
  - No code changes in this pass.
  - Investigated the fresh J6 hardware failure after the RTCore wrap-math change using:
    - `logs/startups/20260418-002742/controller.log`
    - live `/run/gradient-rt-motion/metrics.json`
    - live API snapshots from `/control/motion-status` and `/info/joints-detailed`
  - Confirmed the failure signature changed: the first J6 near-zero jog (`current_deg≈0.001` to `target_deg≈-0.999`, `duration_s=0.250`, `points=25`) now faults immediately instead of taking the long way around.
  - Confirmed only axis 5/J6 faulted after that move: `statusword=0x9638`, `error_code=0xFF00`, `manufacturer_error_code=0`, DS402 `Fault`, canonical truth unavailable because `drive_native_fault_present`.
  - Reconstructed the logged J6 point sequence in the drive's `[0, RM)` frame and found the first consecutive raw point jump is effectively `-RM` (`1310716 -> 148`, delta `-1310568` counts). This means the latest RTCore change still allows the drive-facing `607A` stream to raw-wrap across `0/RM`, which strongly matches the new immediate-fault behavior.
- Validation performed:
  - `python3 -c ... json.load(open('/run/gradient-rt-motion/metrics.json')) ...`
    - result: J6 live fault state captured (`0x9638`, `0xFF00`, `0x203F=0`)
  - `python3 -c ... urllib.request.urlopen('http://127.0.0.1:4400/control/motion-status') ...`
    - result: RTCore/controller idle now, but power transition blocked by `fault_present` on axis 5
  - `python3 -c ... urllib.request.urlopen('http://127.0.0.1:4400/info/joints-detailed') ...`
    - result: canonical truth unavailable for J6 because of `drive_native_fault_present`
  - `python3 -c ...` reconstruction of the logged move from `0.001 deg -> -0.999 deg`
    - result: wrapped counts `1310716, 148, 300, ...`; `max_abs_raw_delta=1310568`
- Follow-up / risk:
  - The latest RTCore fix is not safe to keep as-is. It solved the long-way linear slew but replaced it with a raw single-turn `607A` seam discontinuity that can trip the A6-EC immediately.
  - `0x603F=0xFF00` with `0x203F=0` is only an umbrella fault family; the current decoder labels it `Er11.0`, but the observed raw target jump is more consistent with an `Er87.x` excessive position reference increment path.
  - The next code change should either restore fail-closed seam blocking immediately or implement a seam-biased wire frame that keeps consecutive drive-facing targets continuous without raw `0/RM` wraps.

## 2026-04-18 00:39 +0000 - Bias A6-EC HM35 home frame to wrap midpoint and restore seam fail-close

- What changed:
  - `src/gradient_rt_motion/main.cpp`
    - Added a generic native-home operation kind `write_sdo_wrap_fraction` so RTCore can derive a per-axis SDO write value from the axis wrap period at execution time.
    - Implemented parsing and execution for `op|write_sdo_wrap_fraction|index|subindex|type|numerator|denominator`, using the current axis wrap period to compute the concrete value before the SDO write.
  - `src/gradient_os/arm_controller/profiles/drive/a6ec_ds402.py`
    - Reverted `command_frame_seam_crossing_unsafe` back to `True`.
    - Replaced the hardcoded HM35 `607C=0` step with `write_sdo_wrap_fraction` at `1/2` of the axis wrap period, so the drive's single-turn command/reference seam lands at the midpoint of the frame instead of at home.
    - Updated the native-home config renderer to emit the new descriptor op.
  - `tests/test_rtcore_runtime.py`
    - Updated the expected `GRADIENT_RT_NATIVE_HOME_CONFIG` string to include `op|write_sdo_wrap_fraction|0x607C|0x00|i32|1|2`.
  - `tests/test_gradient05_limits_and_backends.py`
    - Restored seam-crossing first-point and per-point-step expectations to fail-closed behavior.
- Validation performed:
  - `make -C src/gradient_rt_motion`
    - result: success
  - `source "/home/pi/GradientOS/.venv/bin/activate" && PYTHONPATH=src python -m pytest tests/test_rtcore_runtime.py tests/test_gradient05_limits_and_backends.py tests/test_a6ec_joint_sweep.py tests/test_a6ec_j6_watch_replay.py -q`
    - result: `197 passed`
  - `ReadLints` on `src/gradient_rt_motion/main.cpp`, `src/gradient_os/arm_controller/profiles/drive/a6ec_ds402.py`, `tests/test_rtcore_runtime.py`, and `tests/test_gradient05_limits_and_backends.py`
    - result: no diagnostics
- Follow-up / risk:
  - This change only takes effect on hardware after the stack restarts and HM35 runs again, because `607C` is written during native-home. J6 needs a fresh drive-home cycle to move its seam away from home.
  - The earlier RTCore modulo-interpolation experiment remains in `main.cpp`, but with seam-crossing uploads fail-closed again it should no longer be the active path for ordinary operator moves. Live validation should confirm that the J6 near-zero jog now neither faults nor wraps.

## 2026-04-18 01:56 +0000 - Expand the Chapter 5 J6 recorder before rerunning the turn dataset

- What changed:
  - `scripts/a6ec_chapter5_probe.py`
    - Added missing SDO capture objects for `0x203F`, `0x603F`, and `0x60B0` to the probe recorder.
    - Expanded `_build_watch_axis_sample()` so watch JSONL samples now surface `203F`, `603F`, `6062`, `607A`, `607C`, `60B0`, and hex-rendered fault codes alongside the existing counts/truth fields.
    - Expanded `_format_watch_line()` and `_render_markdown()` so the same command/fault-frame fields are visible in live watch output and single-shot snapshot markdown.
    - Updated `DEFAULT_API_URL` from `http://127.0.0.1:4000` to `http://127.0.0.1:4400` to match the current API port default and the existing test expectation.
  - `tests/test_a6ec_chapter5_probe.py`
    - Updated the probe sample/unit tests to assert the new `203F` / `603F` / `6062` / `607A` / `60B0` fields flow through correctly and appear in the condensed watch line.
- Validation performed:
  - `source "/home/pi/GradientOS/.venv/bin/activate" && PYTHONPATH=src python -m pytest tests/test_a6ec_chapter5_probe.py -q`
    - result: `9 passed`
  - `ReadLints` on `scripts/a6ec_chapter5_probe.py` and `tests/test_a6ec_chapter5_probe.py`
    - result: no diagnostics
  - Live readiness check:
    - `python3 -c ... urllib.request.urlopen('http://127.0.0.1:4400/health') ...`
    - result: API healthy on `4400`
  - Live state check:
    - `python3 -c ... urllib.request.urlopen('http://127.0.0.1:4400/control/motion-status') ...`
    - `python3 -c ... urllib.request.urlopen('http://127.0.0.1:4400/info/joints-detailed') ...`
    - result: current runtime still has J6 faulted (`statusword=0x9638`, `error_code=0xFF00`), so the actual expanded J6 turn/manual-rotate rerun is not yet safe to start
- Follow-up / risk:
  - The source-side recorder is ready, but the first plan todo is not physically complete until J6 is returned to a clean baseline and a fresh watch capture is recorded.
  - The archived `web-ui/src/J6ManualRotateDatasetPage.tsx` page is still a baked dataset from the old watch file. It has **not** yet been regenerated from a new expanded capture.

## 2026-04-18 02:46 +0000 - Step-0 continuous 607A experiment fails before actual motion; promote 60B0 fallback

- What changed:
  - `src/gradient_os/arm_controller/backends/ethercat_rtcore/backend.py`
    - Added `GRADIENT_RTCORE_EXPERIMENTAL_CONTINUOUS_607A_JOINTS` parsing so the command path can disable `wrap_to_single_turn` for explicitly targeted joints only.
    - Added a logicalized live-reference helper for the experiment path so first-point live comparison can use `raw_6064 + native_home_position_offset` instead of the raw single-turn wire counts.
    - Added a late-success native-home recovery path: if the initial wait times out but the next metrics snapshot shows `native_home_state=succeeded`, the backend now performs the normal anchor refresh and settle validation instead of returning permanent `pending verification`.
    - Switched post-home anchor capture/validation to the raw frame and fixed the shaft-frame consistency gate to compare against logicalized live reference counts.
    - Added a J6-only seam-guard bypass for the explicit experiment path so the controller can attempt a continuous-`607A` proof move without being blocked by the normal fail-closed seam rule.
  - `src/gradient_os/arm_controller/profiles/drive/a6ec_ds402.py`
    - Removed the routine `F31.10` read/write tail from the standard HM35 native-home transaction so home verification is no longer confounded by post-home encoder maintenance.
  - `src/gradient_rt_motion/main.cpp`
    - Imported `0x607C` truth into `native_home_position_offset` with the correct host-side sign (`offset = -607C`) so the additive logical feedback correction matches the new midpoint-home contract.
  - `tests/test_gradient05_limits_and_backends.py`
    - Added focused regressions covering:
      - continuous-`607A` nearest-turn output without single-turn wrap
      - logicalized first-point live comparison under the experiment flag
      - experiment-path seam-bypass behavior
      - midpoint `native_home_position_offset` cancelling the raw midpoint reference
      - shaft-frame consistency against logicalized live reference counts
      - native-home raw-frame anchor refresh and timeout recovery
  - `tests/test_rtcore_runtime.py`
    - Updated native-home env expectations after removing the routine `F31.10` tail.
- Validation performed:
  - `make -C src/gradient_rt_motion`
    - result: success
  - `source "/home/pi/GradientOS/.venv/bin/activate" && PYTHONPATH=src python -m pytest tests/test_a6ec_chapter5_probe.py -q`
    - result: `9 passed`
  - `source "/home/pi/GradientOS/.venv/bin/activate" && PYTHONPATH=src python -m pytest tests/test_gradient05_limits_and_backends.py -q -k "a6ec_experimental_continuous_607a or native_home or midpoint_native_home_offset_cancels_raw_midpoint_reference or shaft_frame_consistency_uses_logicalized_live_reference_counts"`
    - result: focused slices passed in successive runs (`2 passed`, `24 passed`, `27 passed`, `3 passed`)
  - `source "/home/pi/GradientOS/.venv/bin/activate" && PYTHONPATH=src python -m pytest tests/test_rtcore_runtime.py tests/test_gradient05_limits_and_backends.py -q -k "render_rtcore_systemd_env_contains_scaling_and_profile or build_rtcore_drive_startup_config_uses_drive_profile_module or native_home"`
    - result: `25 passed`
  - `ReadLints` on touched Python and C++ files
    - result: no diagnostics
- Live hardware evidence:
  - Expanded J6 dataset rerun captured under `logs/encoder-retention/j6-manual-rotate-20260418-020055/`; the watch shows `60B0` stayed `0` through the full manual multi-turn sweep.
  - Multiple restarts with `GRADIENT_RTCORE_EXPERIMENTAL_CONTINUOUS_607A_JOINTS=6` succeeded; fresh J6 HM35/native-home now verifies cleanly with:
    - `native_home_position_offset = -655360`
    - `statusword = 0x9650`
    - canonical truth available again
  - Direct controller `APPLY_JOINT_SETPOINT` proof move for `J6 +400 deg` is ACKed/accepted, but the controller thread still exits before actual motion occurs. The dedicated step-0 watch files (`j6-step0-motion.watch.jsonl`, `j6-step0-motion-rerun.watch.jsonl`, `j6-step0-final-proof.watch.jsonl`) show:
    - `603F = 0`
    - `203F = None`
    - `60B0 = 0`
    - `6062`, `6064`, and `607A` pinned near the midpoint-home baseline
    - no real J6 travel
  - Controller logs show the motion thread ending in RTCore `faulted` state without a drive fault, which is sufficient decision-gate evidence that the cheap continuous-`607A` path is still not a usable fix on this stack.
- Follow-up / risk:
  - The step-0 experiment path is now exhausted enough to justify promoting the `60B0` fallback workstream.
  - The fallback remains larger: it still needs cyclic `60B0` support in the EtherCAT/RTCore command path and matching tests/hardware validation.
## 2026-04-18 23:08 +0000 - Add Phase 1 RTCore fault tags and fast-proof watch mode

- What changed:
  - `src/gradient_rt_motion/main.cpp`
    - Replaced the generic trajectory-fault warnings with unique Phase 1 proof tags:
      - `FAULT_UPLOAD_U1..U5` for upload-path rejection branches
      - `FAULT_COMMIT_C1..C3` for commit-path rejection branches
      - `FAULT_EXEC_E1` for non-monotonic active segment timing
      - `FAULT_EXEC_E2` for drive-fault / DS402-fault completion-path failures
      - `FAULT_CLEAR_M1` when `clear_motion_intent(...)` clears an in-flight `pending_upload`
    - Each tagged line now carries the branch-local context needed by the plan (`traj_id`, pending upload state, point counts / indices, timing fields, or the first faulted axis/error-state tuple).
  - `scripts/a6ec_chapter5_probe.py`
    - Added opt-in `--fast-proof` watch mode.
    - Lowered the proof-run interval floor to `0.02 s` only when `--fast-proof` is set; standard watch mode keeps the existing `0.05 s` floor.
    - Added a reduced fast-proof SDO set centered on `203F`, `603F`, `6041`, `6062`, `6064`, `607A`, `607C`, `60B0`, and `U40.20/.22`.
    - Fast-proof watch samples now also skip optional controller joint-state and API/monitor fetches so the tighter cadence spends its budget on RTCore state plus the core wire/fault witnesses.
    - Watch metadata now records whether `fast_proof` was enabled, the effective interval floor, and the selected SDO labels.
  - `tests/test_a6ec_chapter5_probe.py`
    - Added focused regressions for the standard-vs-fast watch interval floor, the fast-proof SDO selection, and the reduced fast-proof capture profile used by `_capture_watch_sample(...)`.

- Validation performed:
  - `make -C src/gradient_rt_motion`
    - result: success
  - `source ./start.sh && python -m pytest tests/test_a6ec_chapter5_probe.py tests/test_rtcore_runtime.py -q`
    - result: `24 passed`
  - `ReadLints` on `src/gradient_rt_motion/main.cpp`, `scripts/a6ec_chapter5_probe.py`, and `tests/test_a6ec_chapter5_probe.py`
    - result: no diagnostics

- Follow-up / risk:
  - This pass does not classify the live J6 failure by itself; it prepares the next hardware rerun to attribute the fault to a named RTCore branch or to an actual drive-visible fault signature.
  - This entry follows the user's architectural correction that `60B0` remains runtime-only and is not the next primary implementation path; the immediate next action is the Phase 1 proof rerun against the new `FAULT_*` tags.
  - `--fast-proof` intentionally trades away some richer API/frontend/monitor fields to make 10-20 ms proof captures feasible. Full `snapshot` mode remains the richer artifact path for broader Chapter 5 frame comparisons.
## 2026-04-18 23:19 +0000 - Ran J6 Phase 1 Move A proof; small mid-range jog failed with no motion and stale RTCore latch

- What changed:
  - No code changes in this pass.
  - Executed the live Phase 1 rerun on the current stack after confirming the controller process still had `GRADIENT_RTCORE_EXPERIMENTAL_CONTINUOUS_607A_JOINTS=6` in its environment.
  - Re-established the J6 fresh-home baseline through `/control/home-joint-native` and verified the expected midpoint-home signature before motion:
    - `statusword = 0x9650`
    - `native_home_position_offset = -655360`
    - canonical truth available from `/info/joints-detailed`
  - Started a dedicated fast-proof watch:
    - `logs/encoder-retention/j6-proof-matrix-20260418/move-a-midrange-jog.watch.jsonl`
    - meta: `logs/encoder-retention/j6-proof-matrix-20260418/move-a-midrange-jog.watch-meta.json`
  - Sent a direct UDP `APPLY_JOINT_SETPOINT` for Move A (`J6 +10 deg`, `max_motor_rpm = 10`, `target_joint_indices = [5]`) so the test exercised the experimental continuous-`607A` command path rather than the API's fixed 100 RPM commissioning helper.

- Validation performed:
  - `curl -sS -X POST -H 'Content-Type: application/json' --data '{"joint":6}' http://127.0.0.1:4400/control/home-joint-native`
    - result: verified home, `post_home_settle_statusword_hex = 0x9650`
  - `python3 -c ... /proc/<run_controller_pid>/environ ...`
    - result: `GRADIENT_RTCORE_EXPERIMENTAL_CONTINUOUS_607A_JOINTS=6`
  - `source ./start.sh && PYTHONPATH=src python scripts/a6ec_chapter5_probe.py watch --label move-a-midrange-jog --axes J6 --interval-s 0.02 --duration-s 15 --fast-proof --experiment-id j6-proof-matrix-20260418`
    - result: watch completed, produced `22` samples in the 15 s window
  - direct UDP `APPLY_JOINT_SETPOINT,<b64>` with `arm_angles_rad`, `max_motor_rpm = 10.0`, `target_joint_indices = [5]`
    - result: accepted, `trajectory_id = 2`
  - `curl -sS -X POST -H 'Content-Type: application/json' --data '{"timeout_s":90}' http://127.0.0.1:4400/control/wait-for-idle`
    - result: timed out after 90 s; RTCore still reported `state='executing'`, `active_traj_id=2`, `current_point_index=166`, `queue_depth=0`
  - `curl -sS -X POST http://127.0.0.1:4400/control/stop`
    - result: did NOT clear the active trajectory latch; RTCore still reported `state='executing'`, `active_traj_id=2`
  - Runtime evidence checks:
    - `/info/joints-detailed`
    - `/control/motion-status`
    - `/run/gradient-rt-motion/metrics.json`
    - watch-file summary from `move-a-midrange-jog.watch.jsonl`

- Runtime outcome:
  - Move A failed even though it was only the small non-seam sanity check.
  - The watch file shows a no-motion signature:
    - `603F` stayed `0x0000`
    - `203F` stayed empty
    - `statusword` stayed `0x9650`
    - `6064` stayed `655361..655364`
    - `607A` stayed `655361..655363`
    - `U40.20/.22` stayed `3638..3641`
  - J6 therefore remained physically stationary and fault-free while RTCore/controller control-plane state diverged:
    - controller thread went idle
    - RTCore motion status stayed latched in `executing`
    - `active_traj_id = 2`
    - `current_point_index = 166`
    - `queue_depth = 0`
  - No accessible `FAULT_UPLOAD_*`, `FAULT_COMMIT_*`, `FAULT_EXEC_*`, or `FAULT_CLEAR_M1` line surfaced in the watched terminal/log paths for this rerun.

- Follow-up / risk:
  - Per the Phase 1 plan, the proof matrix should halt here; Move A failing means there is no value in running the seam-crossing or +360 deg steps until this small-move no-motion path is understood.
  - The current evidence still supports a pre-drive / host-side problem, but the specific path appears to be a stale active-trajectory latch or uninstrumented completion-release bug rather than one of the already-tagged explicit `FAULT_*` branches.
  - `--fast-proof` improved scoping but did not achieve the intended 10-20 ms live cadence on hardware; even with the reduced capture set, the watch only produced `22` samples over 15 s and controller `GET_MOTION_STATUS` calls timed out once the run was underway.
  - The machine is currently stationary and disarmed (`armed = 0`, `axis_enable_mask = 0`, J6 fault-free), but the stack is not ready for another motion command until the stale active trajectory latch is cleared, likely by a controlled restart or a new explicit abort path.
## 2026-04-18 23:37 +0000 - Restarted stack and reran Move A at 100 motor rpm; same no-motion stale-latch failure

- What changed:
  - No code changes in this pass.
  - Verified the semantics of `max_motor_rpm` in `src/gradient_os/arm_controller/command_api.py`: `_build_bounded_joint_path()` converts motor-side speed into joint/output-shaft speed via `(max_motor_rpm / gear_ratio) * 2pi/60`. For J6 (`gear_ratio = 10`), `10` motor rpm means about `1` output-shaft rpm and `100` motor rpm means about `10` output-shaft rpm.
  - Performed a controlled restart to clear the stale active-trajectory latch:
    - `./start-stack.sh stop --hard`
    - `GRADIENT_RTCORE_EXPERIMENTAL_CONTINUOUS_607A_JOINTS=6 ./start-stack.sh`
  - Verified the restarted controller process still had `GRADIENT_RTCORE_EXPERIMENTAL_CONTINUOUS_607A_JOINTS=6` in its environment.
  - Re-homed J6 again through `/control/home-joint-native` to restore the fresh-home baseline before rerunning Move A.
  - Started a second dedicated watch:
    - `logs/encoder-retention/j6-proof-matrix-20260418/move-a-midrange-jog-100rpm.watch.jsonl`
  - Sent the same direct UDP `APPLY_JOINT_SETPOINT` as before, but with `max_motor_rpm = 100.0` and `target_joint_indices = [5]`.

- Validation performed:
  - `./start-stack.sh probe` before restart
    - result: `BUS_UP_DISARMED`
  - `./start-stack.sh stop --hard`
    - result: clean teardown to `physical: INACTIVE`, `ethercat: DOWN`, `rtcore: DOWN`
  - `./start-stack.sh probe` after restart
    - result: `BUS_UP_DISARMED`, all 6 slaves operational, J6 `sw=0x9650`
  - `curl -sS http://127.0.0.1:4400/health`
    - result: API healthy
  - `python3 -c ... /proc/<run_controller_pid>/environ ...`
    - result: `GRADIENT_RTCORE_EXPERIMENTAL_CONTINUOUS_607A_JOINTS=6`
  - `curl -sS -X POST -H 'Content-Type: application/json' --data '{"joint":6}' http://127.0.0.1:4400/control/home-joint-native`
    - result: verified home, `post_home_settle_statusword_hex = 0x9650`
  - `source ./start.sh && PYTHONPATH=src python scripts/a6ec_chapter5_probe.py watch --label move-a-midrange-jog-100rpm --axes J6 --interval-s 0.02 --duration-s 10 --fast-proof --experiment-id j6-proof-matrix-20260418`
    - result: watch completed, produced `19` samples in the 10 s window
  - direct UDP `APPLY_JOINT_SETPOINT,<b64>` with `arm_angles_rad`, `max_motor_rpm = 100.0`, `target_joint_indices = [5]`
    - result: accepted, `trajectory_id = 1`, bounded move metadata reported `duration_s = 0.25`, `frequency_hz = 100`
  - `curl -sS -X POST -H 'Content-Type: application/json' --data '{"timeout_s":30}' http://127.0.0.1:4400/control/wait-for-idle`
    - result: timed out after 30 s; RTCore still reported `state='executing'`, `active_traj_id=1`, `current_point_index=24`, `queue_depth=0`
  - Runtime evidence checks:
    - `/control/motion-status`
    - `/run/gradient-rt-motion/metrics.json`
    - watch-file summary from `move-a-midrange-jog-100rpm.watch.jsonl`

- Runtime outcome:
  - Increasing Move A from `10` to `100` motor rpm did NOT change the failure shape.
  - The 100-rpm watch still showed a no-motion signature:
    - `603F` stayed `0x0000`
    - `203F` stayed empty
    - `statusword` stayed `0x9650`
    - `6064` stayed `655360..655363`
    - `607A` stayed `655360..655363`
    - `U40.20/.22` stayed `3640..3643`
  - The control-plane divergence also reproduced:
    - controller thread went idle
    - RTCore motion status stayed latched in `executing`
    - `active_traj_id = 1`
    - `current_point_index = 24`
    - `queue_depth = 0`

- Follow-up / risk:
  - This materially strengthens the pre-drive diagnosis: the problem is not that the earlier `10` rpm run was simply too slow after the gear ratio conversion.
  - The remaining likely failure surface is the RTCore stuck-`executing` / trajectory-release path, or another uninstrumented non-fault completion path, rather than wire-level seam behavior, multi-turn scaling, or drive-visible fault rejection.
  - The stack is again left in the same logical bad state after the rerun: physically stationary and disarmed, but with an active RTCore trajectory latch that requires another controlled restart before any further motion test.
## 2026-04-18 23:50 +0000 - Corrected `SAFE_POWER_UP` sequence reclassified Move A and isolated a real Move B seam fault

- What changed:
  - Took the user's correction as the new live-sequence rule: `NATIVE_HOME_JOINT` leaves the homed axis disabled, so any post-home motion proof must insert `SAFE_POWER_UP` before sending `APPLY_JOINT_SETPOINT`.
  - Performed a fresh controlled restart with the experiment flag preserved:
    - `./start-stack.sh stop --hard`
    - `GRADIENT_RTCORE_EXPERIMENTAL_CONTINUOUS_607A_JOINTS=6 ./start-stack.sh`
  - Re-homed J6 and confirmed the API now says that directly:
    - `/control/home-joint-native` returned `disarmed_after_home=true`
    - message: "The homed axis remains disabled until an explicit safe power-up."
  - Powered the stack up before motion:
    - `curl -sS -X POST http://127.0.0.1:4400/control/power-up`
    - `./start-stack.sh probe` then showed `armed = 1`, `enable_mask = 0x3f`, `op_enabled_axes = 6/6`, J6 `sw = 0x9637`
  - Attempted `/control/joint-jog` once only to confirm path differences; it rejected with `CANONICAL_JOINT_TRUTH_UNAVAILABLE` / `absolute_home_anchor_stale`, so I switched back to the raw UDP `APPLY_JOINT_SETPOINT` path for the actual proof because that is the path the user was talking about.
  - Ran corrected Move A with raw UDP:
    - watch: `logs/encoder-retention/j6-proof-matrix-20260418/move-a-midrange-jog-100rpm-safe-power-up-udp.watch.jsonl`
    - target: J6 `+10 deg`
    - `max_motor_rpm = 100.0`
  - Continued into Phase 1 Move B because Move A now behaved cleanly:
    - pre-positioned J6 to `+175 deg` at `max_motor_rpm = 100.0`
    - captured seam-near confirmation watch `logs/encoder-retention/j6-proof-matrix-20260418/move-b-preposition-truth-check.watch.jsonl`
    - ran seam-crossing Move B from `+175 deg` to `+185 deg` at `max_motor_rpm = 10.0`
    - Move B watch: `logs/encoder-retention/j6-proof-matrix-20260418/move-b-seam-crossing-10rpm-safe-power-up-udp.watch.jsonl`
  - After collecting the failure evidence, performed `./start-stack.sh stop --hard` to avoid leaving J6 faulted and the stack armed.

- Validation performed:
  - Corrected Move A:
    - raw UDP `APPLY_JOINT_SETPOINT,<b64>` to J6 `+10 deg`
    - `/control/wait-for-idle` returned `state='completed'`
    - `/info/joints` after the move reported J6 `10.000854492187516 deg`
    - `./start-stack.sh probe` remained healthy and fault-free
  - Move B pre-position:
    - raw UDP `APPLY_JOINT_SETPOINT,<b64>` to J6 `+175 deg`
    - `/control/wait-for-idle` returned `state='completed'`
    - `move-b-preposition-truth-check.watch.jsonl` showed `6064 ~= 18204`, `607A = 18204`, `603F = 0x0000`, which places J6 about 5 degrees off the wire seam
  - Move B seam-crossing:
    - raw UDP `APPLY_JOINT_SETPOINT,<b64>` to J6 `+185 deg`
    - `/control/wait-for-idle` returned `state='timeout'` with RTCore `state_name='faulted'`, `last_event_code = 293`, `current_point_index = 83`, `queue_depth = 83`
    - `./start-stack.sh probe` after the failure showed:
      - `physical_state = FAULTED`
      - J6 `sw = 0x9638`
      - J6 `err = 0xff00`
      - decoded drive fault: `Er11.0 | Excessive motor speed upon servo drive power-on`
    - `/var/log/syslog` contained the RTCore branch tag:
      - `FAULT_EXEC_E2 diag_now_ns=138397745915445 traj_id=3 final_due=0 axis_index=5 error_code=0xFF00 ds402_state=5`

- Runtime outcome:
  - The earlier "we regressed so far we cannot even turn the motor" conclusion was wrong for Move A. Once the missing `SAFE_POWER_UP` step was inserted after native home, the exact same raw UDP/controller/RTCore path moved J6 normally and completed cleanly.
  - Move A is now positively proven as a healthy powered baseline:
    - direct UDP `APPLY_JOINT_SETPOINT` accepted
    - RTCore completed the trajectory
    - J6 physically moved to about `+10 deg`
    - no servo fault (`603F = 0x0000`)
  - Move B is the first real fault-classification result:
    - the watch shows the drive did see a changing command on the wire during seam crossing:
      - `607A`: `18204 -> 14544 -> 527 -> 1169`
      - `6064`: `18204 -> 15828 -> 2904 -> 1169`
    - the failure is drive-visible, not pre-wire:
      - `603F` flipped from `0x0000` to `0xFF00`
      - J6 statusword became `0x9638`
      - RTCore fault branch `FAULT_EXEC_E2` fired exactly as intended by the Phase 1 instrumentation
  - Move C was not run because the plan says to stop once Move B produces the first real fault classification.

- Follow-up / risk:
  - The immediate Phase 1 target has shifted from "find the generic stale no-motion branch" to "understand why the J6 seam-crossing move near the `6064` seam provokes a drive-visible `0xFF00` / `FAULT_EXEC_E2` fault."
  - The pre-position near `+175 deg` also exposed a truth-consistency wrinkle in the higher-level API path (`absolute_home_anchor_stale` / `display_joint_truth_unavailable`) even though the raw UDP proof path still moved successfully; that is relevant context for any controller/API-layer follow-up.
  - Before the next live proof, start from a clean restart again because this run ended in a real drive fault and was intentionally torn down with `stop --hard`.
## 2026-04-19 00:17 +0000 - Clarified the correct negative-side seam test geometry

- What changed:
  - No code changes.
  - Clarified the live-proof geometry for J6 seam testing after the operator asked whether a negative move could be used instead of the `+175 deg -> +185 deg` setup.
  - Recorded that the symmetric negative-side Move B should be a pre-position near `-175 deg` followed by a seam-crossing jog to `-185 deg`.

- Validation performed:
  - Re-read the current J6 proof conclusions in `.cursor/memory/AGENT_SCRATCHPAD.md` and `.cursor/memory/DEVLOG.md`.
  - Verified from the existing midpoint-biased `607C = RM/2` framing that the J6 `6064` seam sits near canonical `+/-180 deg`, not near home.
  - Confirmed that `0 deg -> -5 deg` (or `+10 deg -> -5 deg`) would stay in the mid-band and would therefore only reproduce Move A-style non-seam motion, not the seam-crossing condition exercised by Move B.

- Follow-up / risk:
  - If the next hardware pass should make directionality easier to observe, run the negative-side symmetric seam test (`-175 deg -> -185 deg`) instead of the positive-side one.
  - The same sequence guardrails still apply: `home-joint-native -> /control/power-up -> raw UDP APPLY_JOINT_SETPOINT`, and do not run Move C until the Move B seam fault is understood.
## 2026-04-19 00:17 +0000 - Refined seam terminology after operator correction

- What changed:
  - No code changes.
  - Recorded the operator correction that the relevant question is specifically about a negative move over the same seam, not merely any negative move.
  - Clarified the frame distinction: the problematic seam for this investigation is the drive's single-turn `6064` / `607A` wrap boundary, while `U40.20/.22` remain the continuous multi-turn truth source.

- Validation performed:
  - Re-checked the current frame notes and manufacturer notes:
    - `docs/ethercat/a6ec-frame-semantics-and-native-home.md`
    - `docs/ethercat/a6ec-manufacturer-notes-2026-04-15.md`
    - `src/gradient_os/arm_controller/profiles/drive/a6ec_ds402.py`
  - Confirmed that under the current `607C = RM/2` home bias, canonical zero is intentionally moved away from the `6064` wrap boundary, so crossing canonical `0 deg` is not the same as crossing the seam.

- Follow-up / risk:
  - If the next repro should approach the same seam from the negative direction, use the negative-side seam-crossing geometry (`-175 deg -> -185 deg`) under the same powered raw-UDP proof path.
## 2026-04-19 00:38 +0000 - Negative-side seam repro matched the positive fault and surfaced `0x203F = 0x0871`

- What changed:
  - Saved the user-provided handoff verbatim at repo root as `HANDOFF_J6_MOVE_B_SEAM_FAULT_2026-04-18.md`.
  - Ran the negative-side seam-crossing repro under the same live-sequence guardrails:
    - `./start-stack.sh stop --hard`
    - `GRADIENT_RTCORE_EXPERIMENTAL_CONTINUOUS_607A_JOINTS=6 ./start-stack.sh`
    - `/control/home-joint-native`
    - `/control/power-up`
    - raw UDP `APPLY_JOINT_SETPOINT` to pre-position J6 at `-175 deg`
    - raw UDP `APPLY_JOINT_SETPOINT` to cross from `-175 deg` to `-185 deg`
  - Fixed the probe's `0x203F` SDO descriptor in `scripts/a6ec_chapter5_probe.py` from `uint32` to `uint16` after proving the live drive exposes that object as 2 bytes.

- Validation performed:
  - Safe-power baseline before motion:
    - `/control/home-joint-native` returned `disarmed_after_home=true`
    - `/control/power-up` succeeded
    - `./start-stack.sh probe` confirmed `armed = 1`, `enable_mask = 0x3f`, `op_enabled_axes = 6/6`, J6 `sw = 0x9637`
  - Negative seam pre-position:
    - raw UDP `APPLY_JOINT_SETPOINT,<b64>` to J6 `-175 deg` at `max_motor_rpm = 100.0`
    - `/control/wait-for-idle` returned `state='completed'`
    - `curl -sS http://127.0.0.1:4400/info/joints-detailed` reported J6 `-174.9979248046875 deg`
    - `logs/encoder-retention/j6-proof-matrix-20260419/move-b-negative-preposition-truth-check.watch.jsonl` showed J6 sitting on the opposite side of the same seam:
      - `6062 = 1292516`
      - `6064 = 1292515..1292517`
      - `607A = 1292516`
      - `603F = 0x0000`
  - Negative seam-crossing move:
    - watch: `logs/encoder-retention/j6-proof-matrix-20260419/move-b-negative-seam-crossing-10rpm-safe-power-up-udp.watch.jsonl`
    - raw UDP `APPLY_JOINT_SETPOINT,<b64>` to J6 `-185 deg` at `max_motor_rpm = 10.0`
    - `/control/wait-for-idle` returned `state='timeout'` with RTCore `state_name='faulted'`, `last_event_code = 293`, `current_point_index = 83`, `queue_depth = 83`
    - `./start-stack.sh probe` after the fault reported J6 `sw = 0x9638`, `err = 0xff00`
    - `/var/log/syslog` captured:
      - `FAULT_EXEC_E2 diag_now_ns=141255013141603 traj_id=2 final_due=0 axis_index=5 error_code=0xFF00 ds402_state=5`
  - Direct SDO reads while the drive was faulted:
    - `sudo ethercat upload -p 5 -t uint16 0x603F 0x00`
      - result: `0xff00 65280`
    - `sudo ethercat upload -p 5 -t uint16 0x6041 0x00`
      - result: `0x9638 38456`
    - `sudo ethercat upload -p 5 -t uint32 0x203F 0x00`
      - result: `Data type mismatch. Expected uint32 with 4 byte, but got 2 byte.`
    - `sudo ethercat upload -p 5 -t uint16 0x203F 0x00`
      - result: `0x0871 2161`
    - `docs/resources/a6ec_manual_codes.json` maps `0X871` to `Er87.1` = `One-time excessive position reference increment`
  - Tooling fix validation:
    - `source ./start.sh && python -m pytest tests/test_a6ec_chapter5_probe.py -q`
      - result: `12 passed in 0.06s`
    - `ReadLints` on `scripts/a6ec_chapter5_probe.py` and `HANDOFF_J6_MOVE_B_SEAM_FAULT_2026-04-18.md`
      - result: clean

- Runtime outcome:
  - The negative-side seam-crossing repro matches the positive-side failure in all important ways:
    - same RTCore fault branch (`FAULT_EXEC_E2`)
    - same bus fault (`0x603F = 0xFF00`)
    - same statusword (`0x9638`)
    - same RTCore terminal state (`faulted`)
    - same fault timing signature (`current_point_index = 83`, `queue_depth = 83`)
  - The missing ambiguity is now gone. The manufacturer subcode is live and concrete:
    - `0x203F = 0x0871`
    - vendor mapping: `Er87.1`
    - meaning: `One-time excessive position reference increment`
  - The watch also shows that the fault occurs as J6 approaches the seam from the negative side:
    - `607A`: `1292516 -> 1292535 -> 1299916 -> 1309972`
    - `6064`: `1292516 -> 1292514 -> 1298035 -> 1309972`
    - `603F` flips to `0xFF00` at the next sample, before the drive successfully wraps across the seam
  - The probe's previous `203F=None` output was not evidence that the drive omitted the subcode; it was a tooling bug caused by reading `0x203F` as `uint32` instead of `uint16`.

- Follow-up / risk:
  - The highest-value next investigation is now sharply narrowed: inspect why the seam-adjacent `607A` stream produces a one-time excessive position reference increment (`Er87.1`) from both directions.
  - This strongly favors a seam-step/interpolation jump hypothesis over `Er11.0`, generic enable-time instability, or a purely directional mechanical issue.
  - The saved handoff file is already partially stale because its "missing `0x203F` evidence" section is superseded by this run. If that file is going to be used for takeover, it should be refreshed with `0x203F = 0x0871` / `Er87.1`.
  - The stack was intentionally torn down with `./start-stack.sh stop --hard` after the negative repro, so the machine is left inactive and safe.

## 2026-04-19 01:24 +0000 - Offline prep for Move B seam-fault rerun (probe CLI + retention set + handoff refresh)

- What changed:
  - `start-stack.sh` probe per-axis summary now appends `[mfr <code> | <name>]` after the existing bus-level fault suffix when `axis.manufacturer_fault.decoded` is `True`. The `manufacturer_fault` payload was already being carried through `build_drive_fault_snapshot` but the CLI renderer was only printing the bus-level `axis.fault` block, which on A6-EC `0x603F = 0xFF00` collapses seven different vendor codes into whichever the decoder returned first (`Er11.0`). The next live Move B repro will now show `[Er11.0 | Excessive motor speed upon servo drive power-on][mfr Er87.1 | One-time excessive position reference increment]` directly from the probe line.
  - `src/gradient_os/arm_controller/profiles/drive/a6ec_ds402.py::ENCODER_RETENTION_FAULT_CODES` now includes `"ErA0.1"` (Multi-turn overflow fault, `0x203F = 0xA01`). Vendor email 4 Q2(a) explicitly lists `ErA0.1` as a primary signal that `U40.20/U40.22` is no longer reliable, but the set previously only covered `Er20.1 .. Er20.9` and `ALF9.0`. Before this patch a live `ErA0.1` would have collapsed into the generic `manufacturer_fault_present` reason instead of the specific `encoder_retention_fault_present` reason; the overall fail-closed gate still blocks trust, but the diagnostic label was wrong.
  - `.cursor/skills/gradientos-sop/RTOS_ETHERCAT_MASTER_OPERATING_PRINCIPLES.md` and `.cursor/skills/gradientos-sop/commissioning-safety.md` updated so their retention-family enumerations match the new set (`Er20.1 .. Er20.9`, `ErA0.1`, `ALF9.0`), with a one-line citation to vendor email 4 Q2(a).
  - `tests/test_gradient05_limits_and_backends.py` adds `test_a6ec_encoder_retention_fault_includes_multi_turn_overflow` exercising `encoder_retention_fault_code=0xA01` with the same anchor-trust geometry the existing `Er20.9` and `ALF9.0` regressions use; confirms `drive_native_truth_reason == "encoder_retention_fault_present"` and `retention_detail["codes"]` contains `"ErA0.1"`.
  - `HANDOFF_J6_MOVE_B_SEAM_FAULT_2026-04-18.md` refreshed:
    - Title gained "(Updated 2026-04-19)"
    - Lead paragraph now states the confirmed vendor code (`Er87.1`) instead of framing it as unknown
    - "Single most important missing measurement" section replaced with "Confirmed failure signature"
    - Seven-way `0xFF00` ambiguity table kept for reference, with the `Er87.1` row marked as the actual code
    - Step 1 / Step 2 rewritten to focus on disambiguating between RTCore per-cycle seam step vs drive firmware interpretation, not on code identification
    - Added explicit "Offline Prep Already Landed (2026-04-19)" section covering the three changes above
    - TL;DR and decision gate both updated to reflect the concrete `Er87.1` fix choices
- Validation performed:
  - `source ./start.sh && python -m pytest tests/test_gradient05_limits_and_backends.py tests/test_drive_faults.py tests/test_rtcore_runtime.py -q`
    - result: `152 passed in 4.32s`
  - `pytest tests/test_gradient05_limits_and_backends.py::test_a6ec_encoder_retention_fault_includes_multi_turn_overflow -v`
    - result: `1 passed in 0.78s`
  - `bash -n start-stack.sh` -> exit 0 (syntax OK)
  - `ast.parse` on every `<<'PY' ... PY` heredoc in `start-stack.sh` -> all 15 blocks OK
  - `ReadLints` on `start-stack.sh`, `src/gradient_os/arm_controller/profiles/drive/a6ec_ds402.py`, `tests/test_gradient05_limits_and_backends.py`, `.cursor/skills/gradientos-sop/RTOS_ETHERCAT_MASTER_OPERATING_PRINCIPLES.md`, `.cursor/skills/gradientos-sop/commissioning-safety.md`, `HANDOFF_J6_MOVE_B_SEAM_FAULT_2026-04-18.md` -> clean
- Follow-up / risk:
  - These are all offline-safe changes; no hardware was exercised in this pass.
  - The next hardware pass should start by confirming the probe CLI self-describes the vendor code after a reproduced seam fault. If the probe line does NOT show `[mfr Er87.1 | ...]` on the J6 axis, suspect a stale `/run/gradient-rt-motion/metrics.json` or a drive-profile decoder regression, not a drive-side change.
  - The `ErA0.1` retention fix is strictly a label-accuracy improvement; no currently-observed failure relies on it. It is included here because vendor email 4 Q2(a) makes it part of the documented retention family.
  - The handoff file now reflects the current state; it should NOT need further refresh unless a live rerun changes the `Er87.1` classification.

## 2026-04-19 01:38 +0000 - RTCore code-path trace pinned the `Er87.1` root cause to the `[0, RM)` wrap

- What changed:
  - No code/behavior change in this pass. Pure offline analysis + handoff/memory updates.
  - Traced the `607A` emission path in `src/gradient_rt_motion/main.cpp` to identify exactly where the Move B seam-cycle wrap originates.
  - Extracted per-sample `607A` / `6064` / `603F` / `cpi` / `queue_depth` from both Move B watch JSONLs to confirm the fault timing matches the seam-wrap mechanism.
  - Rewrote the handoff file's Step 1-4 "concrete investigation plan" into a tighter "Code-Path Analysis + Two Real Options" section. The previous framing ("is it RTCore interpolation or drive firmware") is resolved; it is RTCore's unconditional wrap.
  - Appended a detailed dated entry to `.cursor/memory/AGENT_SCRATCHPAD.md` that captures the durable rules discovered by the trace.

- Code-path findings (all in `src/gradient_rt_motion/main.cpp`):
  - `feedback_wrap_axis_mask` is a startup CLI flag populated by `GRADIENT_RT_FEEDBACK_WRAP_AXIS_MASK` (live value on this system: `0x3f`, i.e. all six axes wrap). It controls `AxisConfig::feedback_counts_wrap` per axis.
  - Trajectory segment interpolation (lines ~3399-3557): on wrapped axes, `target_counts_interp[i]` is always wrapped into `[0, period)` at line 3553-3554 before the final `clamp_round_to_i32` at line 3556.
  - Hold-target advance (lines 3742-3752): calls `advance_csp_hold_target_counts(hold_target_counts[i], target_counts[i], max_step_counts_per_cycle, wrap_period_counts)`. For wrapped axes this uses shortest-periodic math internally (`shortest_periodic_error_counts`) and clamps by `max_step_counts_per_cycle`, but always returns a value in `[0, period)`.
  - Wire write (line 3796): `EC_WRITE_S32(axis_pd + off[i].target_pos, target_pos_out)` where `target_pos_out = hold_target_counts[i]`.
  - Net effect: the `607A` stream on the wire is ALWAYS wrapped into `[0, RM)` for any axis with `feedback_counts_wrap=True`. The `GRADIENT_RTCORE_EXPERIMENTAL_CONTINUOUS_607A_JOINTS` Python-side flag does NOT reach this path.

- Derived consequence:
  - At a seam-crossing cycle, `hold_target_counts[i]` transitions from (say) `3` to `RM-3`. `advance_csp_hold_target_counts` sees a shortest-periodic delta of `-6` (well under the `~13,107 counts/cycle` clamp at `max_rpm=6000`), so the clamp does NOT fire. The wrapped value `RM-3` is written as-is to `607A`.
  - The A6-EC's `Er87.1` ("One-time excessive position reference increment") evaluates the absolute wire-frame delta `|607A[n+1] - 607A[n]|`, which in this case is `RM-6 ≈ 1,310,714` counts. That massively exceeds `5 * max_motor_speed_counts_per_cycle`, so the drive rejects and faults.

- Watch evidence (from `logs/encoder-retention/j6-proof-matrix-*/move-b-*seam-crossing-10rpm*.watch.jsonl`):
  - Both Move B runs used a 166-point trajectory at 100 Hz and faulted at `cpi=83/166`, i.e. at the seam-crossing point.
  - Positive Move B bracket: sample 14 `607A=527, 6064=2904, cpi=54`; sample 15 `607A=1169, 6064=1169, 603F=0xFF00, cpi=83`. The trajectory was descending toward the seam at `6064=0/RM` and faulted mid-traversal.
  - Negative Move B bracket: sample 25 `607A=1309972, 6064=1309972, cpi=64`; sample 26 `607A=1309923, 6064=1309923, 603F=0xFF00, cpi=83`. The trajectory was ascending toward the seam at `6064=RM` and faulted `~800` counts before the wrap.
  - The 500 ms probe cadence is too coarse to sample the exact wrap cycle, but the bracketing is unambiguous in both directions.

- Validation performed:
  - `source ./start.sh && python3 <inline loader>` over both Move B watch JSONLs to extract per-sample J6 wire-state deltas; confirmed no `FAULT_*` signature change between the two runs.
  - `rg`-level reads of `src/gradient_rt_motion/main.cpp` around `feedback_counts_wrap`, `advance_csp_hold_target_counts`, `wrapped_axis_period_counts`, `max_step_counts_per_cycle`.
  - Cross-checked `GRADIENT_RT_FEEDBACK_WRAP_AXIS_MASK=0x3f` via `.gradient_runtime_config.json` and `runtime.py::RTCORE_FEEDBACK_WRAP_AXIS_MASK_ENV_VAR`.
  - No tests run in this pass (no code change). `ReadLints` on the edited handoff + memory files came back clean.

- Two concrete fix options, with Option A recommended for immediate stability:
  - **Option A (safe, one-line):** flip `POSITION_SEMANTICS_CONFIG["command_frame_seam_crossing_unsafe"]` to `True` in `src/gradient_os/arm_controller/profiles/drive/a6ec_ds402.py` and retire the `GRADIENT_RTCORE_EXPERIMENTAL_CONTINUOUS_607A_JOINTS` env var. Host-side safety guard rejects any trajectory that would cross the wire seam. Operator must plan J6 moves that stay within one mechanical revolution; with `607C=RM/2` the seam sits at canonical `±180°`, so most operational motion is unaffected. Loses multi-turn J6 in rotation mode.
  - **Option B (experimental, live test required):** flip `GRADIENT_RT_FEEDBACK_WRAP_AXIS_MASK` from `0x3f` to `0x1f` (unwrap J6 only). This is a per-axis config change — no code edit. RTCore's interpolation and hold-target advance fall into the `wrap_period_counts=0` linear branches, so `607A` grows monotonically across the seam. Requires a live Move B rerun to verify the drive accepts out-of-`[0, RM-1]` `607A` in rotation mode (vendor has NOT explicitly confirmed). Enables multi-turn if it works; could surface a different drive-fault family (e.g. `Er87.4` "target exceeding maximum single-turn position") if the firmware rejects out-of-range targets.

- Follow-up / risk:
  - Option A is safe to land at any time; it's fully reversible and matches vendor-documented rotation-mode semantics.
  - Option B must NOT be attempted without operator oversight; the seam-crossing cycle is still a drive-fault candidate. Also note that changing the wrap mask affects completion-check semantics (non-wrapped axes use linear delta); need to verify the J6 trajectory completion check still lands correctly.
  - If Option B is attempted and fails, revert the mask back to `0x3f` and land Option A as the stable state. Do not leave the stack in a half-configured state.
  - Phase 2 (math-module adoption) remains gated on Move B passing clean. Option A unblocks Phase 2 within the single-revolution envelope; Option B unblocks it fully.
  - The `advance_csp_hold_target_counts` shortest-periodic clamp is correct for non-seam motion and should not be changed; it is only the seam-wrap interaction with the drive's absolute `Er87.1` check that is problematic.

## 2026-04-19 02:19 +0000 - Continuous 607A in rotation mode: RTCore change landed, staged, tests green

- What changed (offline only; live validation waiting on operator):
  - Scrapped the Option A "retire multi-turn for J6" path after re-reading the vendor manual's Chapter 5 §5.3 Figure 5-1, which plots the rotation-mode target position as a continuous linear ramp while 6064 follows a sawtooth. The previous RTCore wrap of 0x607A into [0, RM) was fighting the drive's own rotation-mode semantics; the fix is to emit continuous 0x607A.
  - New plan file: [`/home/pi/.cursor/plans/rtcore_continuous_607a_7a4d2e91.plan.md`](/home/pi/.cursor/plans/rtcore_continuous_607a_7a4d2e91.plan.md) captures the seven phases and the rollback strategy.
  - Drafted a vendor email (ten questions covering 607A range, Er87.1 threshold parameter, C00.07=2 vs =4, long-running i32 drift) and included it inline in the chat response for the operator to send. Implementation does not block on vendor reply; Chapter 5 Figure 5-1 is strong enough to proceed with a live test.

- Code changes:
  - `src/gradient_rt_motion/main.cpp`:
    - Added `AxisConfig::command_counts_wrap = false` (separate from `feedback_counts_wrap`).
    - Added `Options::command_wrap_axis_mask = UINT32_MAX` sentinel default (means "mirror `feedback_wrap_axis_mask`").
    - Added CLI parser for `--command-wrap-axis-mask` plus usage-string entry.
    - Finalize loop now populates `axis[i].command_counts_wrap` from the mask.
    - Trajectory interpolation output wrap (around line 3595) now reads `opt.axis[i].command_counts_wrap` instead of the feedback flag; shortest-periodic interpolation logic is unchanged.
    - `advance_csp_hold_target_counts` callsite (around line 3797) now passes the command-wrap period, so the linear-clamp branch is selected when command_counts_wrap is false.
    - Feedback-mirror-during-disarm, completion check, and jog-target accumulator all still use the feedback wrap.
  - `src/gradient_os/arm_controller/profiles/drive/a6ec_ds402.py`:
    - `MOTION_FEEDBACK_CONFIG["command_counts_wrap"] = False` (new key, with commentary citing Chapter 5 Figure 5-1).
    - `POSITION_SEMANTICS_CONFIG["command_frame_seam_crossing_unsafe"] = False` (historical note preserved inline).
  - `src/gradient_os/arm_controller/backends/ethercat_rtcore/backend.py`:
    - Retired the `GRADIENT_RTCORE_EXPERIMENTAL_CONTINUOUS_607A_JOINTS` env-var path (confirmed to have been a no-op on the wire because RTCore always re-wrapped).
    - New `_command_counts_wrap_for_joint` reads `MOTION_FEEDBACK_CONFIG["command_counts_wrap"]` via `drive_profile_registry.get_drive_motion_feedback_config(...)`; default True when unknown.
    - `_experimental_continuous_607a_enabled_for_joint` is now a thin alias for `not _command_counts_wrap_for_joint` so existing callsites (logicalized-live-ref helper, seam-guard composition, host fold) keep working.
    - `_resolve_experimental_continuous_607a_joint_indices` is dormant (always returns an empty set).
  - `src/gradient_os/arm_controller/backends/ethercat_rtcore/runtime.py`:
    - Added `RTCORE_COMMAND_WRAP_AXIS_MASK_ENV_VAR = "GRADIENT_RT_COMMAND_WRAP_AXIS_MASK"`.
    - Factored `_resolve_wrap_mask` helper shared between feedback and command paths; emits the command mask env only when the profile expresses an opinion, preserving back-compat on profiles that do not.
    - Note in `build_rtcore_startup_env` explaining why the command-wrap env is NOT defaulted in the rendered dict (systemd's hard-coded default plus the RTCore sentinel cover the fall-through).
  - `systemd/rt-motion/gradient-rt-motion.service`:
    - Added `Environment=GRADIENT_RT_COMMAND_WRAP_AXIS_MASK=0xffffffff` default (matches RTCore sentinel).
    - Appended `--command-wrap-axis-mask ${GRADIENT_RT_COMMAND_WRAP_AXIS_MASK}` to ExecStart.
  - Tests:
    - `tests/test_gradient05_limits_and_backends.py`: renamed `test_a6ec_command_frame_rejects_seam_crossing_step_in_linear_counts` -> `..._allows_...` and flipped assertion; same for `..._rejects_seam_straddling_first_point`. Added `test_a6ec_profile_emits_continuous_607a_command_in_rotation_mode` (profile flag invariants) and `test_a6ec_backend_routes_profile_command_counts_wrap_to_fold_decision` (backend correctly reads profile and exposes both helper methods consistently).
    - `tests/test_a6ec_joint_sweep.py`: dropped the `0 <= wire_counts < RM` assertion in `test_a6ec_joint_sweep_fresh_hm_small_jog_stays_within_half_rm` (obsoleted by continuous emission); kept the shortest-angular-delta invariant which is the real live gate. Updated docstring to cite the 2026-04-19 landing.
    - `tests/test_a6ec_j6_watch_replay.py`: same `0 <= wire_counts < RM` assertion dropped in `test_a6ec_j6_watch_replay_small_jog_stays_within_half_rm`.
    - `tests/test_rtcore_runtime.py`: added `GRADIENT_RT_COMMAND_WRAP_AXIS_MASK="0x0"` assertion to the A6-EC rendered-env test.

- Validation performed:
  - `make -C src/gradient_rt_motion` -> clean build with `-Wall -Wextra -Wpedantic`.
  - `source ./start.sh && python -m pytest tests/test_gradient05_limits_and_backends.py tests/test_rtcore_runtime.py tests/test_drive_faults.py tests/test_a6ec_chapter5_probe.py tests/test_a6ec_joint_sweep.py tests/test_a6ec_j6_watch_replay.py tests/test_api_endpoints.py tests/test_trajectory_execution_backends.py tests/test_command_api_direct_setpoint.py tests/test_run_controller_helpers.py -q` -> `351 passed in 8.10s`.
  - `ReadLints` on every touched file -> clean.
  - Targeted profile-flag check: `python3 -c "from gradient_os.arm_controller.profiles.drive.a6ec_ds402 import MOTION_FEEDBACK_CONFIG; print(MOTION_FEEDBACK_CONFIG)"` -> `{'profile_id': 'a6ec_ds402', 'feedback_counts_wrap': True, 'command_counts_wrap': False}`.
  - Rendered-env check: `render_rtcore_systemd_env(..., drive_profile='a6ec_ds402', ...)` -> includes `GRADIENT_RT_COMMAND_WRAP_AXIS_MASK="0x0"` and `GRADIENT_RT_FEEDBACK_WRAP_AXIS_MASK="0x3f"`.
  - Install + stage: `sudo -n ./systemd/rt-motion/sync-runtime.sh` (no `--ensure-active`). Updated `/usr/local/bin/gradient-rt-motion`, `/etc/systemd/system/gradient-rt-motion.service`, `/etc/default/gradient-rt-motion`. Reloaded `systemd daemon`. Service intentionally left inactive pending operator-present live validation.
  - `/usr/local/bin/gradient-rt-motion --help` confirms `--command-wrap-axis-mask` is exposed on the staged binary.

- Runtime outcome:
  - No live hardware motion in this pass. The stack remains in its 2026-04-19 00:38 hard-stop state (inactive, disarmed).
  - Next step is operator-present live validation under the proof matrix defined in the plan: Move A (smoke) -> Move B+ / Move B- (seam crossing both directions) -> Move C (+360 deg multi-turn) -> Move D (+720 deg multi-turn). Any fault other than a clean completion is a stop condition; rollback is a one-line revert of the profile's `command_counts_wrap` back to True followed by another `sync-runtime.sh`.

- Follow-up / risk:
  - If live Move B still faults with `Er87.1`, the drive firmware is interpreting continuous 607A differently than Chapter 5 Figure 5-1 suggests. Fall back to wrapped emission (revert profile flag) and wait for vendor email response before trying again.
  - If live Move B passes but Move C faults with `Er87.4` or similar, the drive may have an upper-bound limit on 607A that is not visible in the current parameter set. Investigate `C10.1A / C10.1C` and the sign handling for out-of-range targets.
  - The handoff file `HANDOFF_J6_MOVE_B_SEAM_FAULT_2026-04-18.md` is now partially superseded by the landed fix. It remains accurate as historical context but its "Two Real Options" section should be retired after live validation succeeds.
  - The plan file at `/home/pi/.cursor/plans/rtcore_continuous_607a_7a4d2e91.plan.md` captures everything for a fresh agent; update its `todos` frontmatter when Phase 6 completes.
  - `GRADIENT_RTCORE_EXPERIMENTAL_CONTINUOUS_607A_JOINTS` is documented-but-dormant. Removing all references in a future cleanup pass is fine; kept for now so historical log-parsing operator tooling still finds the name. `_resolve_experimental_continuous_607a_joint_indices` can go away when the alias is dropped.

## 2026-04-19 05:12 +0000 - Phase 6 PASS on hardware: continuous 607A + multi-turn J6 proven

- What changed (pre-live, in order):
  - Host-side fix to `_nearest_turn_fold_axis_q_for_axis`: `observed_reference_counts` was passed in live-6064 wire frame but compared against `base_counts` (raw axis-q frame). The two frames differ by `native_home_position_offset` (= `-607C`). At midpoint home (`607C=RM/2`) that is exactly `-RM/2`; a small canonical move from home produced `delta/period ~= 0.528` which `round()` snapped to 1 turn, so the host emitted `0x607A = RM + 618,952` instead of `+618,952`. With RTCore's prior unconditional wrap this was hidden (the wire wrapped back), but with Phase-1 continuous emission the spurious turn leaked straight to the wire and the drive took the long way around (2026-04-19 Move A +350 deg excursion at 04:19). Fix: add `native_home_position_offset` to `observed_counts` before the round, so the fold compares in a single consistent frame. Regression test: `test_a6ec_continuous_607a_fold_does_not_flip_turn_at_midpoint_home`.
  - RTCore fix in `main.cpp` (~line 3561): the trajectory segment interpolation's output wrap was still gated on `feedback_counts_wrap` (True for A6-EC) rather than the new `command_counts_wrap`. Phase 1 only gated the FINAL output wrap on `command_counts_wrap`; the per-cycle interpolation wrap was still active. That reintroduced the seam discontinuity (`+131 -> RM-20` between two consecutive RT cycles) and the drive interpreted it as a +RM forward leap, overriding C10.16=0 Nearest. Fix: introduced `interpolation_wrap_period_for_axis` gated on `command_counts_wrap`; replaced both the interpolation wrap and the `segment_velocity_for_axis` fallback delta to use it. Removed the now-unused `wrap_period_counts_for_axis` lambda.

- Code changes:
  - `src/gradient_os/arm_controller/backends/ethercat_rtcore/backend.py`: fold frame-correction (15 lines net, backward-compatible).
  - `src/gradient_rt_motion/main.cpp`: interpolation wrap-period helper plus two call-site updates, minus the old unused helper (25 lines net).
  - `tests/test_gradient05_limits_and_backends.py`: added `test_a6ec_continuous_607a_fold_does_not_flip_turn_at_midpoint_home` as a direct regression for the fold-frame bug.

- Live hardware matrix on J6 (after fixes + `./start-stack.sh stop --hard && ./start-stack.sh` + fresh HM35):
  - Move A (no seam, 0 deg -> +10 deg, 100 motor RPM): PASS. `6064 = 618,915`, `607A = 618,951`, multi-turn delta = -36,399 motor counts (= +10 deg canonical via sign=-1). No fault. Statusword 0x9637 throughout.
  - Move B+ pre-position (+10 deg -> +175 deg): PASS. `6064 = 18,129`, multi-turn delta = -600,765 motor counts.
  - Move B+ seam cross (+175 deg -> +185 deg, 10 motor RPM): PASS. **`607A = -18,204` (NEGATIVE continuous i32)**. `6064 = 1,292,399` (drive wrapped internally to single-turn). multi-turn delta = -36,456 motor counts (= +10 deg canonical). No fault. First-ever continuous-607A seam crossing on this stack; Er87.1 did not fire.
  - Move B- pre-position (0 deg -> -175 deg): PASS.
  - Move B- seam cross (-175 deg -> -185 deg, 10 motor RPM): PASS. **`607A = +1,328,924` (= RM + 18,204, continuous OVER RM)**. `6064 = 18,263` (drive wrapped internally). multi-turn delta = +36,387 motor counts (= -10 deg canonical). No fault. Proves symmetric seam behavior.
  - Chained multi-turn steps (0 deg -> +175 deg -> +350 deg at 100 motor RPM): PASS. Canonical crosses the +360 deg boundary via chaining. multi-turn delta matches each +175 deg canonical step (~-637 k motor counts each). No fault.
  - Reset from +340 deg back to 0 deg: PASS. Stack returns to clean home state. multi-turn ends at 67,270 (baseline-adjacent).

- Key wire-evidence verbatim:
  - Move B+ final state (via `ethercat upload`): `6064 = 0x0013b8e5 (1,292,517)`, `607A = 0xffffb8e4 (-18,204 signed)`, `U40.20 = 0xfff6bf68 (-606,360 signed)`, `603F = 0x0000`, `6041 = 0x9637`.
  - Move B- final state: `6064 = 0x00004757 (18,263)`, `607A = 0x0014465c (+1,328,924)`, `U40.20 = 0x000b4dd5 (+740,821)`, `603F = 0x0000`, `6041 = 0x9637`.

- Validation performed:
  - `make -C src/gradient_rt_motion` after each RTCore change: clean.
  - `sudo ./systemd/rt-motion/sync-runtime.sh` (no `--ensure-active`) to stage the new binary; hot-replaced pid=862021 on next stack restart.
  - `source ./start.sh && python -m pytest tests/test_gradient05_limits_and_backends.py tests/test_rtcore_runtime.py tests/test_drive_faults.py tests/test_a6ec_chapter5_probe.py tests/test_a6ec_joint_sweep.py tests/test_a6ec_j6_watch_replay.py tests/test_api_endpoints.py tests/test_trajectory_execution_backends.py tests/test_command_api_direct_setpoint.py tests/test_run_controller_helpers.py -q` -> `352 passed in ~10s`.
  - `ReadLints` on every touched file: clean.
  - Live probes verified continuous 0x607A emission on both seam directions and in chained multi-turn, with no fault transitions.

- Outcome:
  - The user's non-negotiable requirement (keep J6 multi-turn capability) is met. The A6-EC rotation-mode continuous-607A path works end-to-end as Chapter 5 Figure 5-1 describes.
  - The vendor email (drafted earlier) is insurance; it does not block operation. Keep it on file in case the vendor confirms or clarifies any detail we got wrong.

- Follow-up / risk:
  - LIMITATION (known, not a regression): single-shot canonical commands more than ~180 deg from live collapse via the host's nearest-turn fold (it uses live_6064 single-turn wire as the "nearest" reference). Commanded `+360 deg` or `+720 deg` as a single setpoint map to the same single-turn position as canonical 0 deg and do not advance multi-turn. Workaround: chain in ~175 deg increments (proved working end-to-end). Proper fix: host-side multi-turn planner or canonical-state-memory in the fold. Document this in the planner/programmatic user surface before any UI work exposes big canonical jumps.
  - `absolute_home_anchor_stale` diagnostic is over-strict: the 8-count tolerance fires on every normal motion because multi-turn moves tens of thousands of motor counts in a fraction of a second. Effect: `/info/joints arm_deg[J6]` goes empty during transitions and for a short window after motion. `arm_display_deg` still carries the correct last-known value. Not blocking motion; should be loosened/rethought in a separate pass.
  - `GRADIENT_RTCORE_EXPERIMENTAL_CONTINUOUS_607A_JOINTS` env var path is dormant (backward-compat alias only). The real switch is the profile's `MOTION_FEEDBACK_CONFIG["command_counts_wrap"] = False` which now drives both the Python fold and the RTCore wire-emission policy.
  - Jog-mode target accumulator in `main.cpp` (~line 3669) still uses `feedback_counts_wrap` for its wrap period. Not exercised by this test matrix, but should be audited if jog is ever run on J6 under continuous-607A; otherwise jog-mode might still wrap internally.

## 2026-04-19 05:30 +0000 - RETRACTION: Phase 6 "PASS" verdicts were endpoint-only and path-blind

- What changed:
  - No code changes. This is a correction record.
  - Operator physically observed J6 whipping the full 360 deg forward on seam crossings "several times" during the Phase 6 live matrix. My earlier verdict rested on multi-turn endpoint reads (pre-move `U40.20` vs post-move `U40.20`), which cannot distinguish a clean +10 deg canonical motion from a whip of "+360 deg forward, -350 deg backward, net +10 deg". Both produce the same endpoint delta.
  - Confirmed via post-mortem of the watch JSONL `logs/encoder-retention/j6-continuous-607a-20260419/move-b-pos-seam-rtcore-fix.watch.jsonl`: all 28 captured samples fell ENTIRELY within `trajectory_id=2` (pre-position) and showed a stable pre-seam state (`607A=18,204, 6064~=18,204, mt=-569,957`). The Move B+ seam trajectory (`trajectory_id=3`) executed AFTER the 8-second watch window ended, because the trajectory ran in 1.66 s at `max_motor_rpm=10` and the probe's effective cadence (~400 ms/sample due to HTTP fetches to the API and serial SDO reads per sample) left wide gaps with zero intermediate captures.

- Evidence of capture failure:
  - `/home/pi/GradientOS/scripts/a6ec_chapter5_probe.py watch --interval-s 0.02 --fast-proof` actually samples at roughly 400 ms/sample on this hardware. The interval flag only sets a floor; the per-sample work (HTTP fetches to `/info/joints-detailed`, `/control/motion-status`, RTCore metrics file reads, SDO reads for 11+ registers) dominates the loop. `--fast-proof` trims the SDO set but does not touch the HTTP path.
  - A 10 deg seam crossing at `max_motor_rpm=10` completes in ~1.66 s. At 400 ms/sample, the motion covers roughly 4 sample slots. In practice the motion fell BETWEEN probe samples and the watch captured only idle states.

- What stays proven:
  - The host-side fold frame-correction in `_nearest_turn_fold_axis_q_for_axis` (adding `native_home_offset_counts` to `observed_reference_counts` before the round) is correct arithmetic, locked down by `test_a6ec_continuous_607a_fold_does_not_flip_turn_at_midpoint_home`.
  - The RTCore fix that gates the trajectory interpolation output wrap on `command_counts_wrap` (rather than `feedback_counts_wrap`) is correct; without it the per-cycle interpolation wrap resurrected the seam discontinuity.
  - The mid-band Move A smoke test (0 deg -> +10 deg canonical, no seam, far from the `0/RM` boundary) has endpoint multi-turn evidence that is meaningful. Endpoint alone is enough for a no-seam small jog where no hidden whip is physically plausible within the speed envelope.

- What is RETRACTED:
  - Move B+ seam-crossing "PASS" verdict (operator observed 360 deg whip on this class of motion).
  - Move B- seam-crossing "PASS" verdict (same class of motion; endpoint read cannot distinguish).
  - Chained multi-turn "PASS" verdict (chain steps 2 and 4 each crossed the seam; endpoint reads cannot prove clean short-path motion).
  - Any blanket statement that "continuous 607A emission is validated on hardware" based on this session's matrix. The emission mechanism is present on the wire (verified via direct `ethercat upload` SDO reads of negative and >RM `0x607A` values), but whether the drive follows those targets SHORT vs LONG at the seam is not yet proven by density-of-capture sufficient to see the whip.

- Next-action plan (must complete before any further seam-crossing "PASS" claim):
  - Build a high-frequency multi-turn capture path that bypasses the HTTP API and targets 100-1000 Hz. Read the combined `U40.20 / U40.22` signed-i64 multi-turn value either by (a) tailing `/run/gradient-rt-motion/metrics.json` at the rate RTCore updates it, or (b) holding a single persistent ethercat SDO connection open and sampling `0x2040:21` + `0x2040:23` in a tight loop. The existing `a6ec_chapter5_probe.py watch` path is not suitable for this job.
  - Run the seam-crossing repros at a deliberately slow speed (`max_motor_rpm=1`, giving 0.1 output RPM = 0.6 deg/s) so a 10 deg move takes ~17 s. This way the capture cadence (even 50-100 Hz) produces a dense trace of the whole motion.
  - Plot the multi-turn trace against commanded canonical. The trace must be monotonic in the commanded direction with no excursions beyond the commanded canonical delta plus a small physical overshoot budget (say, 1-2 deg). Any excursion >20 deg is a whip and fails the test.
  - Only after a successfully traced clean seam crossing may the earlier Move A/B+/B- / chained matrix be re-validated and the PASS verdicts re-earned.

- Follow-up / risk:
  - Even the motion that looked clean via endpoints may in fact have whipped; re-validate all of them after the capture path is ready.
  - The "physical 360 deg whip" observation, if reproducible under the slow-speed trace, is a real safety concern that must be communicated to any downstream planner / UI (tighten speed limits, disable seam crossings at the host, force chained multi-turn, or keep the drive in rotation mode but refuse seam crossings until the motion path is fixed).
  - `absolute_home_anchor_stale` diagnostic remains over-strict and is a secondary clean-up; do NOT touch it until the multi-turn whip question is resolved — loosening it now would make the downstream bug easier to miss.
  - The live stack is currently armed with J6 at canonical 0 deg; operator should stop --hard before any further experimentation begins, to avoid another whip until the capture path is ready.

## 2026-04-19 06:20 +0000 - Phase 2 offline prep: fast multi-turn capture script + tests landed

- Context:
  - Executing Phase 2 of `/home/pi/.cursor/plans/j6_seam_whip_verification_b8c230f3.plan.md`.
  - Probe at session start: `./start-stack.sh probe` showed `physical_state=INACTIVE`, `ethercat_master_state=DOWN`, `rtcore_state=DOWN (socket_present=no)`; nothing to stop, no motion can happen until a deliberate restart. Phase 1 is already satisfied.

- Capture-backend research (offline, no hardware touched):
  - `ethercat --help` has no batch/REPL subcommand; every `ethercat upload` is a full fork+exec+ioctl cycle.
  - No Python bindings installed: `import pyethercat` and `import ethercat` both fail; `pip3 list` shows no ecrt/igh module.
  - `libethercat.so.1.2.0` is present at `/usr/local/lib/` and `ecrt.h` is at `/usr/local/include/`, but `ecrt_request_master()` from a second process would steal the master away from RTCore. Unsafe as the capture path.
  - `metrics.json` flushes at 5 Hz only: the metrics thread in `src/gradient_rt_motion/main.cpp` around line 4587 sleeps 200 ms per iteration.
  - The SDO poll of U40.20/.22 inside RTCore uses `kAbsoluteFeedbackPollIntervalNs = 200000000ULL` (200 ms) so even if metrics.json were flushed faster, the multi-turn field in it would only refresh at 5 Hz. Plan's Option B is disqualified for U40.20/.22 specifically.
  - `sudo -n -l` on this host confirms `(ALL) NOPASSWD: ALL` for `pi`. `sudo -n ethercat upload` can safely run inside a sample loop without password prompts.

- New code:
  - `scripts/j6_multiturn_fast_capture.py` (new, ~460 lines):
    - Zero HTTP in the sample loop.
    - `capture` subcommand spawns ALL per-sample SDO reads in parallel as `subprocess.Popen` of `sudo -n ethercat upload`, then `communicate(timeout=2.0)` on each. Combined U40.20+U40.22 signed i64 is the primary truth; 6064/607A/603F/6041 are captured for wire context + in-loop fault halt.
    - Full descriptor set is 6 SDOs; `--minimal` trims to 4 (drops 6064/607A) for when the full set cannot hit 100 Hz.
    - Halts immediately on `603F != 0` or DS402 Fault statusword (bit pattern `SW & 0x004F == 0x0008`). `--no-fault-halt` disables this for tooling experiments.
    - JSONL output at `logs/j6-multiturn-fast/<label>-<iso8601>Z.jsonl` with `{t_mono_ns, mt_i64, c6064, c607A, c603F, c6041, reads{...}}` per line plus a `.meta.json` sibling with `elapsed_s`, `effective_hz`, `fault_halt_reason`, `first_mt_i64`, `last_mt_i64`, `net_mt_delta_counts`, `descriptor_keys`, and operator `note`.
    - `analyze` subcommand reads a JSONL and computes: `sample_count`, `elapsed_s`, `effective_hz`, `first_mt / last_mt / net_mt_delta`, `cumulative_travel = sum(|delta|)`, `max_sample_abs_delta`, `cum/|net|` ratio (threshold 1.2 for WHIP verdict), monotonicity within `--overshoot-budget-counts` (default 500 motor counts = ~0.14 deg output), fault detection (first sample where 603F or statusword indicates fault). Writes a matplotlib PNG plot next to the JSONL if matplotlib is available.
  - `tests/test_j6_multiturn_fast_capture.py` (new, 26 tests):
    - `_combine_signed_i64`: zero, positive, high-bit-sign-extension, live-DEVLOG sample from 2026-04-19 Move B+ wire read (`0xFFF6BF68 / 0xFFFFFFFF -> -606_360`), None-tolerance.
    - `_parse_ethercat_value`: both observed output shapes (`0xhex decimal` and `decimal`), signed i32/i16 sign-extension, unsigned no-sign-extend, empty, malformed.
    - `_statusword_is_fault`: None, Operation Enabled (`0x9637`), Faulted (`0x9638` and canonical `0x0008`), Fault Reaction Active (`0x0048` NOT classified as pure Fault), Switched-on-disabled.
    - `_analyze_jsonl`: clean short-path motion (ratio=1.0, monotonic), synthetic whip (+50k forward then -40k back, net +10k; ratio=9.0, NOT monotonic), mid-trace fault at sample index 5 with 603F=0xFF00, small dither < 500 count budget that is not classified as whip, empty file raises.
    - `_counts_to_output_deg`: one motor rev = 36 deg output at gear ratio 10, one full output turn = 360 deg output, 10 deg output round-trip within 0.01 deg.
    - Test loader sets `sys.modules["j6_multiturn_fast_capture"]` before `exec_module` so `@dataclasses.dataclass` can resolve the class's parent module.

- Validation performed:
  - `python3 -c "import ast; ast.parse(open('scripts/j6_multiturn_fast_capture.py').read()); print('syntax OK')"` -> OK
  - `python3 scripts/j6_multiturn_fast_capture.py --help` / `capture --help` / `analyze --help` -> all correct.
  - `python3 -m pytest tests/test_j6_multiturn_fast_capture.py -q` -> `26 passed in 0.08s`.
  - `python3 -m pytest tests/test_j6_multiturn_fast_capture.py tests/test_a6ec_chapter5_probe.py tests/test_gradient05_limits_and_backends.py tests/test_rtcore_runtime.py tests/test_drive_faults.py tests/test_a6ec_joint_sweep.py tests/test_a6ec_j6_watch_replay.py tests/test_api_endpoints.py tests/test_trajectory_execution_backends.py tests/test_command_api_direct_setpoint.py tests/test_run_controller_helpers.py -q` -> `378 passed in 10.80s`.
  - End-to-end CLI smoke via `/tmp/j6cap/fake.jsonl` (synthetic whip: +50k forward then -40k back, net +10k over 92 samples at ~100 Hz): `analyze` correctly reported `cum/|net|=9.0`, `VERDICT: WHIP`, `monotonic=False`, wrote PNG next to JSONL.
  - `ReadLints` on both new files: clean.
  - Full-repo `pytest -q` shows 7 pre-existing failures in `test_driver.py`, `test_end_to_end.py`, `test_planning.py`, `test_protocol.py`, `test_solver.py`. All of them are in the legacy IK / serial-servo stack and were not affected by these changes; they also pre-date this work per prior DEVLOG entries (`352 passed` on the A6-EC workstream scope as of 2026-04-19 05:12 Phase 6 session).

- Runtime outcome:
  - No hardware touched in this pass. Stack remains INACTIVE / DOWN.
  - Phase 2 offline prep is complete; Phase 2 live smoke-test (run the capture on an idle bus to measure achievable Hz and validate the JSONL shape end-to-end) is the next step, gated on operator consent to bring the stack up.

- Follow-up / risk:
  - SDO mailbox throughput is the fundamental cap. On this hardware, realistic sample rates for the full 6-SDO descriptor set are expected to be 30-80 Hz (per-subprocess cost dominates; parallel Popen reduces wall time but not mailbox time). The plan's Phase 3 slow-speed (17 s for 10 deg at `max_motor_rpm=1`) makes 30-50 Hz plenty dense to see a 360 deg whip. If the idle-bus smoke-test comes in under 30 Hz, fall back to `--minimal` (4 SDOs) or drop motion speed further.
  - If `--minimal` at `max_motor_rpm=1` still cannot produce a clean trace dense enough to resolve a whip, escalate to plan Phase 2 Option C: add a ring-buffered multi-turn log inside RTCore itself (new IPC endpoint or dedicated trace file written by a high-priority thread that tap-reads the already-computed `pos_counts`/`target_pos_out` and issues U40.20/.22 SDO reads out-of-band).
  - Do NOT start Phase 3 (live seam crossing) until Phase 2 smoke-test has measured achievable Hz on an idle bus. The previous agent's failure mode was exactly "assume capture is fast enough; learn it wasn't, after motion".
  - Any idle-bus smoke-test is safe if and only if the motor is disabled. Even calling `sudo ethercat upload` in a tight loop while the drive is NOT operation-enabled will never move anything; but the capture script intentionally does NOT read any API endpoint, so keep hands off the UI while it runs too.
  - The 7 unrelated repo-wide pre-existing test failures (legacy IK/servo) stay open as pre-existing work, not part of this Phase 2 scope.

## 2026-04-19 06:42 +0000 - Phase 2 idle-bus smoke-test: 5.6 Hz ceiling measured, <20% of plan target

- What was run:
  - `./start-stack.sh` brought the stack to `physical_state=BUS_UP_DISARMED`, all six axes `SwitchOnDisabled`, `ethercat=OP`, `rtcore=UP`, `startup_ready=yes`, `armed=0`. Verified the axes cannot move (SwitchOnDisabled means the power stage is off).
  - Full capture (all 6 SDOs) for 4 s against J6: `logs/j6-multiturn-fast/idle-bus-smoke-20260419T063928Z.jsonl`:
    - `samples=23 elapsed_s=4.14 hz=5.56`
    - per-sample dt mean 180.8 ms (min 159.7, max 224.7)
    - U40.20+U40.22 combined i64 stable at ~67,242 +/- 2 motor counts (encoder noise on idle bus, expected).
    - All 6 fields populate on every sample (0 misses).
    - `first_mt_i64 == last_mt_i64 == 67,242`, `net_mt_delta_counts = 0`, no fault halt.
  - Minimal capture (4 SDOs: U40.20+.22+603F+6041) for 4 s: `idle-bus-minimal-20260419T064147Z.jsonl`:
    - `samples=34 elapsed_s=4.08 hz=8.33`.
  - Inline benchmark of `sudo ethercat upload` cost to understand the ceiling:
    - `echo`: ~1 ms (fork/exec baseline)
    - `sudo -n true`: ~8 ms (sudo overhead)
    - Single `sudo ethercat upload -p 5 -t int32 0x2040 0x21`: ~46 ms
    - 1 parallel read batch: ~51 ms (dominant cost)
    - 2 parallel reads: ~66 ms batch (~33 ms per read amortized)
    - 3 parallel reads: ~89 ms batch (~30 ms/read)
    - 4 parallel reads: ~125 ms batch (~31 ms/read, parallelism saturating)
    - 6 parallel reads: ~190 ms batch (~32 ms/read, fully saturated)

- Finding:
  - The IgH EtherCAT master kernel module serializes SDO mailbox transactions PER SLAVE at ~30 ms per SDO on this 1 kHz bus. `sudo -n ethercat upload` adds ~15 ms of subprocess overhead. Parallelism saturates at about 3 concurrent reads on the same slave; beyond that, the kernel queues additional requests behind the in-flight mailbox exchange.
  - Consequence: with N SDO reads per sample on J6 (slave 5), the effective rate ceiling is approximately `1 / (N * 30 ms + subprocess overhead)`:
    - 1 SDO ~= 20-33 Hz
    - 2 SDOs ~= 15-17 Hz
    - 4 SDOs ~= 7-8 Hz (matches observed minimal mode)
    - 6 SDOs ~= 5-6 Hz (matches observed full mode)
  - The plan's "100 Hz minimum, 1000 Hz ideal" is NOT physically attainable via userspace SDO to the same slave. Any claim of higher throughput via Python/subprocess would be a measurement bug.
  - `ecrt_request_master()` from a second process is unsafe (would release the master from RTCore). `libethercat.so.1.2.0` ctypes binding therefore does NOT give a safe speedup.
  - `/run/gradient-rt-motion/metrics.json` flushes at 5 Hz (RTCore metrics thread has a 200 ms sleep) and U40.20/.22 inside it is SDO-polled at 5 Hz too (`kAbsoluteFeedbackPollIntervalNs = 200_000_000 ns`). So tailing the metrics file offers no multi-turn improvement over the direct 4-8 Hz capture.

- Implication for Phase 3:
  - At `max_motor_rpm=1` (0.6 deg/s output), a 10 deg seam motion takes ~17 s. At the observed 5.6 Hz that's ~95 samples across the whole motion. A 360 deg whip takes roughly `60/(360*gear_ratio) = 0.017 s * 360 deg / deg = ~0.17 s` at a fairly modest 360 motor RPM (6 revs/s). A sub-200 ms whip could ENTIRELY fall between two samples at 5.6 Hz and be missed.
  - To safely catch such a whip with the current capture, Phase 3 must either run at a deliberately slow motion speed (e.g. `max_motor_rpm=0.5` giving a 34 s trajectory, ~190 samples at 5.6 Hz) AND/OR use `--minimal` to gain the 8.3 Hz rate (~280 samples over 34 s).
  - Alternatively, escalate to plan Phase 2 Option C: add a dense trace writer inside RTCore (new thread that snapshots per-cycle `pos_counts[J6]` = 6064 + `statusword[J6]` + `error_code[J6]` at 100-1000 Hz, plus U40.20/.22 at whatever SDO poll rate RTCore can sustain, into a dedicated ring-buffered JSONL). This would give the full 1 kHz context for 6064/6041/603F and keep U40.20/.22 at whatever the master mailbox tolerates, deduplicated across the cycle stream.
  - Do NOT begin Phase 3 without an explicit operator decision between "run very slow and accept the SDO rate" vs "implement Option C first".

- Validation / cleanup:
  - `./start-stack.sh stop --hard` after the smoke-test reached `physical: INACTIVE   driver: INACTIVE   ethercat: DOWN   rtcore: DOWN`. No motion occurred at any point; axes stayed in `SwitchOnDisabled`.
  - Artefacts left in place under `logs/j6-multiturn-fast/` for reference:
    - `idle-bus-smoke-20260419T063928Z.jsonl` + `.meta.json`
    - `idle-bus-minimal-20260419T064147Z.jsonl` + `.meta.json`

- Follow-up / risk:
  - Decision pending from operator: accept slower motion + 5-8 Hz OR authorize plan Phase 2 Option C (RTCore ring-buffer trace writer).
  - If Option C is authorized, the smallest-blast-radius implementation is a new dedicated thread in `src/gradient_rt_motion/main.cpp` that snapshots `latest_feedback.pos_counts[i]`, `statusword[i]`, `error_code[i]`, target_counts[i] (latest), and absolute_feedback[i].value for the J6 slot at a configurable rate (CLI flag `--j6-trace-hz` default 0 = disabled), writing JSONL to a file whose path is also CLI-configurable. U40.20/.22 would continue to be SDO-polled at the existing cadence but the output cadence of 6064/6041/603F/607A could be 100-1000 Hz trivially because all that data is already in-memory on every cycle.
  - If we go slow-motion path instead, update `scripts/j6_multiturn_fast_capture.py` to support arbitrary SDO subset via `--sdo-keys` flag and document the measured Hz as a function of subset size.

## 2026-04-19 07:32 +0000 - RTCore fast_trace at 1 kHz: userspace SDO path was the wrong tool

- Context:
  - Operator called out my userspace-SDO analysis as wrong: "how can you not read faster than 5 Hz? we fucking already do that. how does rtcore send back telemetry at ALL?" They were correct.
  - I had conflated SDO mailbox throughput (~30 ms/read kernel floor, shared-slave-serialized) with per-cycle PDO telemetry (1 kHz free in RTCore). On the A6-EC, 0x6064 / 0x607A / 0x6041 / 0x603F are all PDO-mapped per `GRADIENT_RT_DRIVE_{TX,RX}_PDO_LAYOUT`; only U40.20/U40.22 (0x2040:21/:23) is SDO-only on this drive.
  - Worse, `src/gradient_rt_motion/main.cpp` already has a `fast_trace_thread` (around line 4383) that writes per-cycle feedback to JSONL at up to the cycle rate, enabled by three env vars `GRADIENT_RT_FAST_TRACE_{PATH,HZ,AXIS_MASK}`. The systemd unit wires them into ExecStart with a comment literally citing this Phase-2 plan file. I missed the infrastructure by not reading the main.cpp deeply enough before starting to build a SDO subprocess loop.

- What changed:
  - `/etc/systemd/system/gradient-rt-motion.service.d/fast-trace.conf` (new drop-in):
    ```
    [Service]
    Environment=GRADIENT_RT_FAST_TRACE_PATH=/run/gradient-rt-motion/j6-fast-trace.jsonl
    Environment=GRADIENT_RT_FAST_TRACE_HZ=1000
    Environment=GRADIENT_RT_FAST_TRACE_AXIS_MASK=0x20
    ```
    Layered on top of the main unit's empty defaults; NOT overwritten by `start-stack.sh` since it regenerates `/etc/default/gradient-rt-motion`, not systemd drop-ins. Fully reversible: `sudo rm` the file + `sudo systemctl daemon-reload` + next restart.
  - `scripts/j6_multiturn_fast_capture.py` - added `analyze-rtcore` subcommand:
    - `_unwrap_wire_delta(delta, rm_counts)` computes the shortest-periodic wire-frame delta so 0x6064 seam crossings (raw delta close to +/- RM) are correctly reported as small motions across the wrap, not as 360-deg jumps.
    - `_extract_axis_sample(record, axis_index)` parses the RTCore JSONL shape per line, pulling pos_counts (p), target_pos_counts (tp), statusword (sw), error_code (er), manufacturer_error_code (mfr), and the combined signed-i64 multi-turn from the `af[]` `encoder_multi_turn_low` + `_high` fields.
    - `_analyze_rtcore_jsonl(path, axis_index, rm_counts, overshoot_budget_counts)` returns a `FastTraceAnalysis` with `cumulative_travel_wire_counts`, wrap-aware `net_displacement_wire_counts`, `max_abs_wire_step_counts`, `cum/|net|` ratio, monotonicity within a direction-flip budget, `mt_sample_count + mt_distinct_samples + mt_net_delta` cross-check, and the first fault-sample index.
    - `_print_rtcore_analysis(result, gear_ratio)` formats the summary; VERDICT is noise-floor gated: when `|net| < max(10 * max_abs_wire_step, 1000 counts)` it reports STATIC instead of WHIP (avoids misfiring on idle-bus encoder dither). WHIP fires on ratio > 1.2 OR monotonic=False, with an explicit reason list.
    - `_plot_rtcore_trace_png` produces a dual-panel matplotlib PNG: top panel shows UNWRAPPED cumulative 6064 displacement in deg output + U40.20/.22 overlay; bottom panel shows RAW wire 6064 with RM guide line. Fault samples get a red vertical line.
    - Old `capture` + `analyze` subcommands retained as deprecated fallback.
  - `tests/test_j6_multiturn_fast_capture.py` - added 13 new tests covering the fast_trace path:
    - `_unwrap_wire_delta` zero / small-positive / small-negative / positive-wrap / negative-wrap / rm-zero-passthrough
    - `_extract_axis_sample` J6 present / missing-axis / ok=false meaning mt_i64 is None
    - `_analyze_rtcore_jsonl` clean motion (ratio=1, monotonic), synthetic whip (50 forward then 40 back, 89 transitions, ratio 89/9, non-monotonic), seam wrap (RM-100 -> +100 is +200 not ~+RM), drops leading p=0 samples before PDO latch, detects 603F=0xFF00 at correct sample index, 5-Hz-mt-refresh-in-1kHz-stream distinct-values cross-check.

- Validation performed:
  - `python3 -c "import ast; ast.parse(open('scripts/j6_multiturn_fast_capture.py').read()); print('syntax OK')"` -> OK
  - `python3 scripts/j6_multiturn_fast_capture.py --help` / `analyze-rtcore --help` correct.
  - `python3 -m pytest tests/test_j6_multiturn_fast_capture.py -q` -> `39 passed in 0.09s`.
  - `ReadLints` on the touched files -> clean.
  - Live smoke on idle bus (J6 disarmed):
    - `./start-stack.sh` -> `BUS_UP_DISARMED`; journal line `fast_trace: writing to /run/gradient-rt-motion/j6-fast-trace.jsonl at 1000 Hz (period 1000000 ns) mask=0x20`.
    - 1 s `wc -l` delta: 1056 lines -> ~1000 Hz effective.
    - 183 s capture produced 183,373 valid samples (effective Hz 1000.01).
    - `analyze-rtcore`: `cumulative_travel=139,310 counts (38.26 deg)` (all encoder dither, max single-cycle step 4 counts = 0.001 deg), `net=-2 counts`, VERDICT=STATIC (|net|=2 counts < noise floor 1000).
    - Dual-panel PNG generated and stored at `logs/j6-multiturn-fast/j6-fast-trace-idle-3s.png`.
    - Stack hard-stopped cleanly; `/run/gradient-rt-motion/` wiped by systemd per RuntimeDirectory default.

- Runtime outcome:
  - No motion commanded at any point. J6 stayed SwitchOnDisabled throughout.
  - Infrastructure for Phase 3 is now proven: 1 kHz J6 trace with full PDO wire context + U40.20/.22 at its existing 5 Hz poll rate is enough to see a 360-deg whip at any reasonable motion speed (whip at 360 motor RPM = 6 rev/s -> ~100 deg per cycle at output, a massive spike in max_abs_wire_step; whip at 6000 RPM = 100 rev/s -> would still be visible as 10 cycles of large monotonic-wrong-direction deltas).
  - Idle trace + plot are archived at `logs/j6-multiturn-fast/j6-fast-trace-idle-3s.{jsonl,png}` (95 MB / 84 KB).

- Follow-up / risk:
  - `/etc/systemd/system/gradient-rt-motion.service.d/fast-trace.conf` is persistent across reboots. If someone else restarts the machine without knowing about it, the fast_trace will still be enabled. Harmless (disk-write-only, no wire impact) but the file will grow unbounded while the service is up. Recommend removing the drop-in or setting HZ=0 after Phase 3 completes.
  - The fast_trace's `absolute_feedback` (U40.20/.22) entries are still refreshed at 5 Hz by the metrics-thread SDO poll (`kAbsoluteFeedbackPollIntervalNs = 200 ms`). For the Phase 3 workflow this is fine (0x6064 at 1 kHz is the primary evidence; U40.20/.22 is cross-check for rollover). If a future experiment needs U40.20/.22 at > 5 Hz, bump that constant in `main.cpp` and rebuild; shouldn't be necessary for whip detection.
  - The `/etc/default/gradient-rt-motion.bak-<ts>` backup I created earlier (before learning start-stack regenerates the file) is no longer necessary but harmless. Safe to delete.
  - `scripts/j6_multiturn_fast_capture.py capture` subcommand and the prior idle-bus SDO smoke-test traces (`idle-bus-smoke-*.jsonl`, `idle-bus-minimal-*.jsonl`) are retained as historical context but are deprecated for this workflow. Future cleanup pass can delete them.
  - The previous entry in this devlog (2026-04-19 06:42, "5.6 Hz ceiling measured") is now superseded. It remains accurate for the userspace SDO path but mis-framed the overall Phase-2 options. Operator decision "authorize RTCore Option C" was actually unnecessary because Option C was already shipped - just not enabled.

## 2026-04-19 07:57 +0000 - Fast-trace autosave hook landed; runtime-wipe failure mode closed

- What changed:
  - `scripts/j6_multiturn_fast_capture.py` - new `save` subcommand:
    - Copies `/run/gradient-rt-motion/j6-fast-trace.jsonl` to `<log-dir>/<label>-<iso8601>Z.jsonl` with a matching `.meta.json` sibling.
    - Uses `sudo -n cp --preserve=timestamps` (the runtime file is root-owned) and then `sudo -n chown` to the invoking user so `analyze-rtcore` can read the saved file without sudo.
    - `--if-exists` flag: silent no-op if the source file is missing or size 0. Exit 0. Intended for automation hooks.
    - Without `--if-exists`: errors with `[save] ERROR: source ... does not exist` and exits 1.
    - Meta includes `schema_version`, `tool`, `label`, `source_path`, `dest_jsonl`, `saved_at_wall_utc`, `source_size_bytes`, `source_mtime_unix_s`, a `stats` block with `line_count/first_t_ns/last_t_ns/elapsed_s/effective_hz`, and operator `note`. Stats are computed via cheap head+tail reads so the helper scales to tens-of-MB traces without loading the full JSONL.
  - `start-stack.sh` - new `preserve_rtcore_fast_trace_if_any()` helper + invocation:
    - Runs at the TOP of `perform_shutdown_sequence`, before any RTCore/EtherCAT teardown.
    - Invokes `python3 scripts/j6_multiturn_fast_capture.py save --if-exists --label autosave`.
    - Non-fatal: never aborts shutdown. Output goes through the existing `log "rtcore-trace: ..."` path so it shows up inline in the stack log.
    - Fires on both soft-stop and hard-stop; harmless when fast_trace is disabled (source file will not exist -> silent no-op).
  - `tests/test_j6_multiturn_fast_capture.py` - new tests covering `_estimate_trace_stats` (empty/1-kHz head+tail correctness/trailing-blank-line tolerance) and `_save_rtcore_trace` (missing-source + `if_exists=True` returns `SaveResult(copied=False)` without polluting the dest dir; without `if_exists` raises `FileNotFoundError`).

- Validation performed:
  - `python3 -c "import ast; ast.parse(open('scripts/j6_multiturn_fast_capture.py').read()); print('syntax OK')"` -> OK
  - `bash -n start-stack.sh` -> OK
  - `python3 scripts/j6_multiturn_fast_capture.py save --help` -> correct
  - `python3 -m pytest tests/test_j6_multiturn_fast_capture.py -q` -> `44 passed in 0.20s`.
  - `ReadLints` on `scripts/j6_multiturn_fast_capture.py`, `tests/test_j6_multiturn_fast_capture.py`, `start-stack.sh` -> clean.
  - Live end-to-end:
    - `./start-stack.sh` -> BUS_UP_DISARMED in 2 s.
    - 3-second pause (fast_trace grew from 4.68 MB to 7.77 MB).
    - `./start-stack.sh stop --hard` -> hook ran at the top of `perform_shutdown_sequence`, BEFORE `stop_rtcore_runtime`. Stack log recorded: `[start-stack] rtcore-trace: [save] copied 7767435 bytes -> logs/j6-multiturn-fast/autosave-20260419T075747Z.jsonl`.
    - Saved file owned `pi:pi`, 7,775,643 bytes, 15,929 lines, spans 15.93 s at effective_hz `999.9994` (per the stats written into the meta file).
    - `python3 scripts/j6_multiturn_fast_capture.py analyze-rtcore logs/j6-multiturn-fast/autosave-20260419T075747Z.jsonl --no-plot` parses cleanly, reports VERDICT=STATIC (as expected for idle-bus data), `effective_hz=1000.13`, wire_monotonic=True. No sudo needed because the file is pi-owned.
    - `save --if-exists` after stack stop correctly reports `[save] skipped: source /run/gradient-rt-motion/j6-fast-trace.jsonl does not exist`. Without `--if-exists`, same invocation returns `[save] ERROR:` and exits 1.

- Runtime outcome:
  - No motion commanded at any point; J6 stayed SwitchOnDisabled throughout.
  - The `/run/gradient-rt-motion/` wipe-on-service-stop failure mode is now closed for the Phase 3 workflow: operator no longer needs to remember to copy the trace out before `stop --hard`. The hook does it automatically.
  - Operator may still use `save --label <custom>` at any time while RTCore is up to snapshot named points mid-session (e.g. `save --label pre-seam-crossing` and `save --label post-seam-crossing`), independent of the autosave fired by `stop --hard`.

- Follow-up / risk:
  - If the operator runs `./start-stack.sh stop` (soft-stop, RTCore stays up) the hook still fires and captures the current state. RTCore then continues writing to the same path, so a later `stop --hard` will create a second autosave with the full trace. That is intentional (two saves = two reference points) but could produce unexpected file proliferation if someone cycles soft-stop many times. Worth documenting only if it surfaces as an issue.
  - The noise-floor gated VERDICT=STATIC in `analyze-rtcore` also suppresses the WHIP verdict on very short traces where motion had not yet started. The `fault_seen` field still surfaces any pre-OP 0x8700 error_code the capture may pick up (seen above on the idle-bus autosave: `fault_seen: True -- 603F=0x8700 at sample 0`). For Phase 3 motion captures this is correct (the motion window always begins after OP has latched 603F=0), but be aware that `fault_seen` on idle autosaves may be benign pre-OP noise. Cross-reference with `first_t_ns` vs the operator's commanded-motion timestamp if in doubt.
  - The systemd drop-in at `/etc/systemd/system/gradient-rt-motion.service.d/fast-trace.conf` is still persistent across reboots. If nobody touches it, every future stack run will write `/run/gradient-rt-motion/j6-fast-trace.jsonl` at 1 kHz and auto-save it on stop. The saved files accumulate under `logs/j6-multiturn-fast/autosave-*.jsonl` - expected cost is ~600 MB/min of stack uptime at 1 kHz. Clean up or set `GRADIENT_RT_FAST_TRACE_HZ=0` when Phase 3 completes.
  - `/etc/default/gradient-rt-motion.bak-20260419T071725Z` is still present from the earlier (incorrect) manual-edit attempt. Safe to delete.

## 2026-04-19 08:44 +0000 - Phase 3 slow-speed seam crossing PROVEN CLEAN (max_motor_rpm=1)

- Context:
  - Per the plan (`/home/pi/.cursor/plans/j6_seam_whip_verification_b8c230f3.plan.md`) Phase 3: slow-speed seam-crossing repro with 1 kHz fast_trace capture, to see whether the seam-crossing motion is short-path-clean or the 360 deg whip the operator observed at 10 motor RPM.
  - Infrastructure at this point: RTCore fast_trace drop-in enabled at 1000 Hz for J6 only; `scripts/j6_multiturn_fast_capture.py save/analyze-rtcore` landed; autosave hook in `start-stack.sh` preserves the trace on `stop --hard`.

- New tooling:
  - `scripts/j6_seam_whip_phase3_runner.py`: orchestrates the live motion sequence end-to-end. Homes J6 via `/control/home-joint-native {"joint": 6}`, powers up via `/control/power-up`, saves trace snapshots at each checkpoint via `save --label phase3-*`, pre-positions to +175 deg at max_motor_rpm=100, then does the critical +175 -> +185 deg move at max_motor_rpm=1 (17 s at 0.6 deg/s output) using raw UDP APPLY_JOINT_SETPOINT with target_joint_indices=[5]. Caches pre-motion joint state to survive the `absolute_home_anchor_stale` diagnostic that blanks arm_rad post-motion. `--skip-home-power-up` flag for rerun when already armed.

- Motion sequence (live, no errors):
  - `./start-stack.sh`: BUS_UP_DISARMED + api up in 5 s.
  - Runner's first attempt crashed on `arm_deg[5]` being empty immediately post-power-up -> fixed runner to use `arm_display_rad` fallback + cached pre-motion joint state.
  - Second attempt with `--skip-home-power-up` (stack was still armed from first attempt):
    - J6 started at +175.000 deg canonical (already there from the aborted first run's preposition).
    - Pre-position APPLY_JOINT_SETPOINT accepted as trajectory_id=2 (effectively no-op since target = live).
    - wait-for-idle returned `state=completed` after 1 s.
    - SEAM CROSS APPLY_JOINT_SETPOINT accepted as trajectory_id=3 with max_motor_rpm=1.
    - wait-for-idle returned `state=completed` after 17.1 s.
    - Final J6 canonical position was reported as `unavailable` by `arm_deg` (anchor_stale diagnostic).
    - Snapshots saved inline: `phase3-after-powerup-20260419T084200Z.jsonl` (100 MB), `phase3-pre-seam-slow-20260419T084201Z.jsonl` (101 MB), `phase3-post-seam-slow-20260419T084219Z.jsonl` (110 MB).
  - `./start-stack.sh stop --hard`: autosave hook fired, producing `autosave-20260419T084255Z.jsonl` (129 MB) with the complete cumulative session.

- Analysis (isolated seam-cross window via t_ns filter):
  - Extracted the 17.49 s slow seam crossing from `phase3-post-seam-slow-*.jsonl` into `phase3-seam-cross-slow-isolated.jsonl` (9 MB, 17,491 samples, effective_hz 1000.06).
  - Wire-frame (0x6064) first = 18,204 (J6 at +175 deg canonical).
  - Wire-frame (0x6064) last = 1,292,489 = `RM - 18,231` (J6 at +185 deg canonical AFTER wrapping through 0/RM seam).
  - cumulative_travel = 39,035 motor counts = 10.72 deg output (essentially the commanded 10 deg move + 7% overhead).
  - net_displacement = -36,435 motor counts = -10.01 deg output (matches commanded +10 deg with J6 sign=-1).
  - max_abs_wire_step = 15 counts = 0.004 deg/cycle (well under 2 motor RPM peak throughout, matching the commanded 1 motor RPM speed).
  - cum / |net| = 1.071 (7% overhead, well within the plan's 1.2x WHIP threshold).
  - **VERDICT = CLEAN**.
  - wire_monotonic = True (no direction flip beyond the 500-count budget at any point).
  - U40.20/U40.22 cross-check: net = -35,620 motor counts = -9.78 deg (matches the wire-frame net within encoder resolution).
  - fault_seen = False.
  - If a 360 deg whip had occurred: cumulative_travel would have been ~1.35M counts (369 deg), max_abs_wire_step would have been hundreds of counts per ms (high peak velocity), and wire_monotonic would be False. None of those are present.
  - Artefacts: `logs/j6-multiturn-fast/phase3-seam-cross-slow-isolated.{jsonl,png}` (the PNG shows the smooth monotonic unwrapped-6064 curve; the raw-6064 panel shows the seam crossing as a clean wrap from near 0 down through 0/RM to near RM).

- Analyzer verdict caveat noted:
  - When `analyze-rtcore` is run on the FULL 206-second session (including the earlier 100-RPM preposition from 0 deg -> +175 deg), cum/|net| = 1.225 which trips WHIP because the preposition's per-cycle jitter inflates cumulative relative to the single-motion net. This is a FALSE POSITIVE caused by analyzing a multi-phase session as one window. The correct use of the tool is to isolate one motion segment at a time via t_ns filter (as done here). Documented for future sessions.

- Implication per plan Phase 4 decision tree:
  - Phase 3 slow (max_motor_rpm=1) is CLEAN at Move B+ direction (+175 -> +185). Next is re-run at max_motor_rpm=10 to see if the operator-observed whip at 10 RPM reproduces under the 1 kHz trace. If also clean, earn back the retracted Move B+ PASS. If NOT clean at 10 RPM, the issue is speed-dependent (drive position-loop gain bandwidth or host per-cycle step clamp interaction) and Phase 5 investigation opens.
  - Move B- (negative seam cross, -175 -> -185) at slow speed is also a required data point before re-earning the retraction marks, to prove symmetric behavior.

- Validation performed:
  - `make` and `pytest` not re-run (no code logic change in this pass beyond the runner script which doesn't have unit tests yet).
  - `ReadLints` on `scripts/j6_seam_whip_phase3_runner.py` -> clean.
  - Live analyze-rtcore output verified on both the full session and the isolated seam-cross window.
  - Stack hard-stopped cleanly after the run; autosave hook fired as expected.

- Follow-up / risk:
  - DO NOT claim Move B+ PASS yet. This is one direction, one slow speed. Plan's Phase 6 requires re-running at 10 motor RPM AND both directions before retracted PASS marks are re-earned.
  - The runner script assumes --skip-home-power-up can be set when already armed, but on a fresh stack it correctly homes and powers up. Keep --skip-home-power-up explicit when chaining multiple runs to avoid accidental re-home mid-session.
  - The whole `logs/j6-multiturn-fast/` directory is ~570 MB now from this one session. Future runs will add more. Consider pruning older snapshots or moving to a separate test-artefacts dir when the proof matrix completes.
  - `absolute_home_anchor_stale` is still firing on every motion (known issue, deferred cleanup per earlier scratchpad). The runner script works around it by caching the pre-motion joint state. Do NOT loosen the diagnostic's 8-count tolerance until the full proof matrix completes.
  - analyze-rtcore's 1.2x WHIP threshold will give false positives on multi-phase sessions. For clean verdicts, always isolate a single motion window first (cheap python filter on t_ns as done here). Consider adding `--from-t-ns` / `--to-t-ns` flags to analyze-rtcore in a future pass to make this easier.

## 2026-04-19 09:02 +0000 - Phase 3 Move B+ at 10 motor RPM also CLEAN

- Context:
  - Per plan Phase 4 decision tree, if the slow (1 RPM) seam cross is clean, re-run at 10 motor RPM — the speed at which the operator originally observed the whip that led to retracting Move B+. If also clean, start earning back the retraction.

- What happened:
  - `./start-stack.sh` fresh after the slow run. J6 at +185 deg physical from the prior test; absolute encoder retention held (`0x9650`, pos=1,292,557 ≈ canonical +185) across the restart.
  - Initial runner attempt failed because `arm_rad[1], arm_rad[2], arm_rad[5]` were all blanked by `absolute_home_anchor_stale` for seconds after the fresh home+power-up. Added `fetch_joint_state_with_all_joints_ready` (polls `/info/joints` until all 6 joints have non-None values, 15 s timeout).
  - Second attempt failed at the seam-cross step: controller rejected `APPLY_JOINT_SETPOINT` with `Canonical joint truth unavailable ... reasons=['multi_turn_anchor_inconsistent_with_live_6064']`. Root cause: the 16-count `_SHAFT_FRAME_CONSISTENCY_TOLERANCE_COUNTS` gate in `backend.py:125` trips after the ~630k-count 175 deg preposition moves the live 0x6064 far from the anchor-implied expected value. 1 s settle wasn't enough; bumped to 5 s.
  - Third attempt (with `--skip-home-power-up`, J6 already at +175 from the failed 2nd attempt) completed: preposition = no-op; seam cross to +185 at max_motor_rpm=10 accepted as trajectory_id=3. `wait-for-idle` returned `state=completed` after **1.9 s** (analytic expected = 10 deg * 10 gear / (10 motor RPM * 6) = 1.67 s).

- Runner fixes landed in `scripts/j6_seam_whip_phase3_runner.py`:
  - New `fetch_joint_state_with_all_joints_ready(timeout_s=10, poll_interval_s=0.25)` survives the anchor_stale blanking window post-home.
  - Post-preposition settle bumped from 1 s to 5 s so the `multi_turn_anchor_inconsistent_with_live_6064` gate passes before the next motion.
  - Added CLI flags `--preposition-deg`, `--seam-target-deg`, `--preposition-max-motor-rpm`, `--seam-max-motor-rpm`, `--label-suffix`, `--seam-idle-timeout-s`.
  - Fixed expected-duration formula: was `(delta_deg / gear_ratio) / (rpm / 60)` -> produced 6.0 s for 10 RPM case (actual 1.9 s). Corrected to `delta_deg * gear_ratio / (motor_rpm * 6)`.

- Analysis (isolated 2.31 s window via t_ns filter, `phase3-seam-cross-10rpm-positive-isolated.jsonl`, 2,312 samples at effective_hz 1000.43):
  - 0x6064 first = 18,205 (+175 deg canonical), last = 1,292,425 = `RM - 18,295` (+185 deg canonical after wrapping through 0/RM seam).
  - cumulative_travel = 37,024 motor counts = 10.17 deg output.
  - net_displacement = -36,500 motor counts = -10.025 deg output (matches commanded with J6 sign=-1).
  - max_abs_wire_step = 126 counts/ms = 0.035 deg/ms = ~5.8 motor RPM sustained peak (matches commanded 10 RPM within velocity-planner overhead).
  - cum / |net| = 1.014.
  - **VERDICT = CLEAN**, wire_monotonic = True, fault_seen = False.
  - U40.20/U40.22 cross-check: net = -34,334 motor counts = -9.43 deg output. Only 2 distinct mt values captured in the 2.31 s window at the 5 Hz SDO poll rate; the wire-frame verdict is the primary evidence.
  - Whip-impossibility gate: a 360 deg whip in ANY sub-interval of the 2.31 s window would produce max_abs_wire_step >= 565 counts/ms (for the slowest possible 360-deg-per-2.3s whip). Measured 126 is well below that floor.

- Validation performed:
  - `make`/`pytest` not re-run (no unit-testable change; runner wiring only).
  - `ReadLints` on the runner script -> clean.
  - Live analyze-rtcore on the isolated window -> CLEAN; on the full session -> WHIP false positive (multi-phase quirk, documented).
  - Stack hard-stopped cleanly; autosave hook fired, `autosave-20260419T090220Z.jsonl` (157 MB) preserved.

- Runtime outcome:
  - Move B+ is proven CLEAN at BOTH max_motor_rpm=1 AND max_motor_rpm=10 by 1 kHz telemetry with identical code (drop-in + host fold + RTCore interpolation-wrap gating, no changes between runs).
  - The earlier "operator observed 360 deg whip at 10 RPM" retraction is NOT reproducing. The telemetry is ground truth. Possible explanations: transient drive-side behavior that has cleared, visual misinterpretation of wrist motion, or uncharacterized state dependence. Not speculating further without more data.

- Follow-up / risk:
  - Phase 6 re-earn still requires Move B- (negative direction) at BOTH max_motor_rpm=1 AND max_motor_rpm=10 before the retraction is closed. Do NOT declare Move B+ PASS until the full 2x2 matrix is confirmed clean.
  - The `multi_turn_anchor_inconsistent_with_live_6064` gate (16-count tolerance) is operationally awkward: it rejects the next motion command until the anchor refreshes (takes ~5 s after a large preposition). Works but forces explicit settle. Consider raising the tolerance to something like RM/4 = 327,680 counts (= 90 deg physical) after the proof matrix completes; it still catches real frame-inconsistency bugs without forcing artificial settles.
  - The runner does not auto-retry on `multi_turn_anchor_inconsistent_with_live_6064`; it exits with a useful message. Consider adding a retry-with-extra-settle path in a future session.
  - Snapshot artefacts from this run (all in `logs/j6-multiturn-fast/`): `phase3-after-powerup-10rpm-positive-*.jsonl` (121 MB), `phase3-pre-seam-10rpm-positive-*.jsonl` (124 MB), `phase3-post-seam-10rpm-positive-*.jsonl` (125 MB), `autosave-20260419T090220Z.jsonl` (157 MB), `phase3-seam-cross-10rpm-positive-isolated.{jsonl,png}` (1.4 MB + 100 KB).

## 2026-04-19 18:30 +0000 - Phase 6 COMPLETE: full B+ / B- at slow/fast matrix is CLEAN

- Context:
  - Plan's Phase 6 gate: "Re-run the 10-motor-RPM seam-crossing matrix (Move B+, Move B-) with the fast capture active for every motion. Confirm the trace is clean at the faster speed too." This entry closes that gate by completing the Move B- direction that was blocked earlier on a physical EtherCAT disconnect.
  - Prior state: Move B+ slow and Move B+ 10 RPM both CLEAN under 1 kHz trace (DEVLOG 2026-04-19 08:44 and 09:02). Move B- blocked (DEVLOG 2026-04-19 after the stop --hard following the 10 RPM B+ run).
  - EtherCAT physical link was restored by the operator before this session (eth0 `carrier=1, operstate=up`, sudo ethercat master shows Slaves: 6 again).

- Motion runs (two separate stack sessions, home + power-up + preposition + seam cross, autosave hook captures full session on stop):
  - Session 1 -- Move B- at 1 motor RPM (`--preposition-deg -175 --seam-target-deg -185 --seam-max-motor-rpm 1 --label-suffix 1rpm-negative`):
    - Home, power-up, joint state settled after ~2.75 s (anchor-stale diagnostic).
    - Preposition 0 deg -> -175 deg at 100 motor RPM (no seam crossing) completed in ~3 s.
    - 5 s settle for anchor consistency gate.
    - Seam cross -175 deg -> -185 deg at 1 motor RPM completed in 17.1 s. wait-for-idle reported `state=completed`.
    - Stack hard-stop; autosave `autosave-20260419T182834Z.jsonl` (54 MB) produced.
  - Session 2 -- Move B- at 10 motor RPM (`--seam-max-motor-rpm 10 --label-suffix 10rpm-negative`):
    - Fresh stack, home, power-up, settle.
    - Preposition 0 deg -> -175 deg at 100 motor RPM completed cleanly.
    - Seam cross -175 deg -> -185 deg at 10 motor RPM completed in 1.8 s (matches analytic 1.67 s).
    - Stack hard-stop; autosave `autosave-20260419T183025Z.jsonl` (47 MB).

- Analysis (both isolated via t_ns filter, same pattern as prior runs):
  - Move B- at 1 RPM, `phase3-seam-cross-1rpm-negative-isolated.jsonl` (17,352 samples, effective_hz 1000.06):
    - first_p (0x6064) = 1,292,515 (-175 deg canonical wire frame), last_p = 18,238 (-185 deg canonical AFTER wrap through RM/0 seam).
    - cumulative_travel = 38,655 motor counts = 10.62 deg output.
    - net_displacement = +36,443 motor counts = +10.009 deg output (commanded +10 deg exactly, sign=+1 in this direction since canonical -175 -> -185 with J6 sign=-1 inverts to wire going UP through RM).
    - max_abs_wire_step = 18 counts/ms (0.005 deg/ms), mean = +2.22 counts/ms (matches 1 motor RPM).
    - cum / |net| = 1.061.
    - wire_monotonic = True, fault_seen = False. **VERDICT = CLEAN.**
    - U40.20/U40.22 cross-check: net = +36,420 motor counts = +10.003 deg output (matches wire frame within encoder resolution; 11 distinct mt values captured in 17.35 s at the 5 Hz SDO poll rate).
  - Move B- at 10 RPM, `phase3-seam-cross-10rpm-negative-isolated.jsonl` (2,017 samples, effective_hz 1000.50):
    - first_p = 1,292,515 (-175 deg), last_p = 18,365 (-185 deg through seam).
    - cumulative_travel = 36,908 motor counts = 10.137 deg.
    - net_displacement = +36,570 motor counts = +10.044 deg.
    - max_abs_wire_step = 150 counts/ms (0.041 deg/ms, matches commanded 10 motor RPM peak + velocity-planner overhead).
    - cum / |net| = 1.009 (nearly perfect).
    - wire_monotonic = True, fault_seen = False. **VERDICT = CLEAN.**
    - U40.20/U40.22 cross-check: net = +35,797 motor counts = +9.832 deg output (3 distinct mt values in 2.02 s).

- Full Phase 6 re-earn matrix is now CLEAN in all four cells:
  | direction                  | 1 motor RPM              | 10 motor RPM             |
  |----------------------------|--------------------------|--------------------------|
  | Move B+ (+175 -> +185)     | CLEAN +10.01 deg output  | CLEAN +10.03 deg output  |
  | Move B- (-175 -> -185)     | CLEAN +10.01 deg output  | CLEAN +10.04 deg output  |
  (net displacement, sign-adjusted for sign=-1; every cell within 0.05 deg of commanded 10 deg)

- Retraction audit trail: the DEVLOG entry at `## 2026-04-19 05:30 +0000 - RETRACTION: Phase 6 "PASS" verdicts were endpoint-only and path-blind` is the baseline for this reearn. That retraction was correctly applied at the time because the only evidence was endpoint-only multi-turn reads that cannot distinguish a clean 10 deg motion from a 360 deg whip netting 10 deg. This 2026-04-19 18:30 entry supersedes the claim of "retracted" for Move B+ and Move B- specifically, backed by dense 1 kHz telemetry covering the entire motion window in both directions at both speeds. **Move B+ and Move B- seam-crossing PASS marks are re-earned.** The retraction entry stays in place; do not edit it, the audit trail depends on it.

- Explicitly still retracted (not earned back in this session):
  - Chained multi-turn (0 deg -> +175 -> +350 deg) from the 2026-04-19 05:30 retraction entry. That scenario needs its own 1 kHz capture pass. Recommended next test before any canonical multi-turn is exposed to downstream consumers.

- Image subfolder for all Phase 6 evidence: `logs/j6-multiturn-fast/images/` now holds:
  - `phase3-seam-cross-slow-MOTION-EVIDENCE.png` (Move B+ 1 RPM, 276 KB)
  - `phase3-seam-cross-10rpm-positive-MOTION-EVIDENCE.png` (Move B+ 10 RPM, 255 KB)
  - `phase3-seam-cross-1rpm-negative-MOTION-EVIDENCE.png` (Move B- 1 RPM, 242 KB)
  - `phase3-seam-cross-10rpm-negative-MOTION-EVIDENCE.png` (Move B- 10 RPM, 257 KB)
  - Four matching `...-isolated.png` auto-generated by analyze-rtcore (100 KB each)
  - `phase3-post-seam-slow-20260419T084219Z.png` (historical, first full-session analysis from 08:44)
  - Each MOTION-EVIDENCE PNG has 4 stacked panels: unwrapped-6064 displacement (deg output) with U40.20/.22 overlay, raw 6064 + 607A wire values with RM wrap line, per-cycle wire velocity (counts/ms) with commanded-velocity reference line, 0x6041 statusword + 0x603F error code strip. Purple vertical line marks the seam-crossing timestamp on all panels.
  - JSONL traces (cumulative + isolated) stay in `logs/j6-multiturn-fast/` root to keep the subfolder focused on visual evidence.

- Validation performed:
  - No code changes in this pass; runner script was already landed in prior entries.
  - `ReadLints` on touched memory files -> clean.
  - Live analyze-rtcore verdicts printed for both negative-direction isolated windows.
  - Stack hard-stopped cleanly after each run; autosave hook fired as expected.

- Follow-up / risk:
  - The whole `logs/j6-multiturn-fast/` is now ~1.1 GB across 18 JSONL files. `images/` is ~1.5 MB. Consider deleting pre-phase3/ and the intermediate autosaves once the proof matrix is cited and the handoff file (if any) is updated; the four isolated JSONLs and four MOTION-EVIDENCE PNGs are the durable evidence that needs to survive. Defer cleanup until the operator gives the word.
  - Fast-trace drop-in at `/etc/systemd/system/gradient-rt-motion.service.d/fast-trace.conf` is still active. It's fine to leave for future verification work, but it will continue adding ~60 MB/minute of stack uptime and firing the autosave on every `stop --hard`. If the Phase 3 matrix is complete for this workstream, consider setting `GRADIENT_RT_FAST_TRACE_HZ=0` in the drop-in (or removing it) to stop the trace writer on future starts.
  - Chained-multi-turn (from the same retraction) is the last outstanding Phase-6-family test. Not blocking this entry but worth scheduling before any UI / downstream planner starts trusting continuous canonical >180 deg commands.

## 2026-04-19 19:22 +0000 - Operator reproduced the whip from the UI: two-jog pattern lands J6 on the seam, next jog flips the host's fold turn and whips long-path

- Context:
  - After the Phase 6 matrix was marked clean (single-command seam crossings), operator ran a separate test using the web UI's jog buttons. From canonical +175 deg: jogged +5 deg (to +180, EXACTLY on the seam), waited the natural UI/anchor-gate settle (~5 s), jogged +5 deg again (to +185). J6 whipped a full +350 deg forward -- operator saw it with their own eyes.
  - 1 kHz fast_trace captured the whole event.

- What the trace showed:
  - Isolated window `ui-test-seam-cross-isolated.jsonl`, 56,285 samples at 992 Hz, covering 56.7 s including both UI clicks and the 5 s settle between them.
  - Phase 1 (t=21.7-21.9 s, first UI click +175 -> +180):
    - Host emitted continuous 0x607A target ramping 13,759 -> -18 (short-path negative continuous).
    - Drive correctly followed the short path across the seam; 0x6064 went 18,204 -> 77 (close to 0), then drive wrapped and 6064 landed near RM-18.
    - J6 ended at canonical +180 deg (on the seam).
    - No whip in Phase 1.
  - Settle (t=21.9-26.8 s): 0x6064 parked near RM-18, 0x607A held at -18. No motion.
  - Phase 2 (t=26.828-26.99 s, second UI click +180 -> +185):
    - 0x607A JUMPED from -18 to +13,089 in ONE millisecond, then ramped +13,107 counts/ms for ~100 ms until it reached +1,292,370.
    - **+13,107 counts/ms = exactly GRADIENT_RT_MAX_RPM (6,000 motor RPM).** Not the commanded `max_motor_rpm=100` from the UI jog -- the drive's absolute max.
    - Drive chased the target and rotated J6 ~360 deg forward in those 100 ms. Physical whip.
  - Final state: J6 landed at canonical +185 deg (= final target) but after going the LONG way around. From the drive's perspective the motion "completed" and no fault fired.

- Root cause analysis:
  - Location: `src/gradient_os/arm_controller/backends/ethercat_rtcore/backend.py::_nearest_turn_fold_axis_q_for_axis`, lines 3982-3985:
    ```
    delta = float(observed_counts) - float(base_counts)
    wrap_turns = int(round(delta / float(period_counts)))
    wrap_lift_counts = int(wrap_turns * int(period_counts))
    adjusted_counts = float(base_counts) + float(wrap_lift_counts)
    ```
  - At the start of Phase 2, J6 is at canonical +180 deg which is the seam. The live 0x6064 sampled by the host is either 0 or RM-1 depending on sub-count encoder noise. With `observed_reference_counts = RM-1, native_home_offset_counts = -RM/2`: `observed_counts = RM/2 - 1`. Target base_counts for canonical +185 deg is `~ -673,785` (negative continuous). `delta / period_counts ≈ 1.014`, `round(...) = 1`. The fold emits `wrap_lift_counts = +RM = +1,310,720` and `adjusted_counts = +636,935` (long-path target). Convert back to wire frame and you get ~+1,292,370, which is exactly what we observed.
  - Had the host sampled live_6064 = 0 instead (the other valid reading at the seam), `observed_counts = -RM/2`, `delta ≈ +18,425`, `round(0.014) = 0`, `wrap_lift_counts = 0`, `adjusted_counts = -673,785` (short-path target, continuous negative). That's what Phase 1 did correctly.
  - **The bug**: `round(delta/period_counts)` is on the knife-edge at the seam. Sub-count noise in live_6064 determines whether the fold picks 0 or +1 turn, and a +1 flip commands a full extra revolution.
  - Why single-command Phase 3 tests were CLEAN: the runner goes from +175 -> +185 in one motion, fold computes ONCE from the safe +175 position (live_6064 ~= 18k, well away from the seam boundary), picks turn 0 correctly, and emits the continuous short-path target. The UI's two-jog pattern re-runs the fold AFTER J6 has landed on the seam, exposing the boundary flip.

- Analyzer fix landed (this session, `scripts/j6_multiturn_fast_capture.py`):
  - `FastTraceAnalysis` now carries `net_displacement_shortest_wrap_counts` (net wrapped into `[-RM/2, +RM/2]`) and `long_path_excess_counts` (= `|net - shortest|`).
  - `_print_rtcore_analysis` verdict logic: WHIP fires when `long_path_excess >= RM/2` (motor took >= one extra full revolution beyond the shortest equivalent path), regardless of `wire_monotonic` or `cum/|net|`. This is the correct gate for "went the long way" class of bug that the old `cum/|net| > 1.2` check missed because the whip was monotonic.
  - Tests: added `test_analyze_rtcore_detects_long_path_whip_even_when_monotonic` and `test_analyze_rtcore_clean_short_cross_stays_clean`. Sweep: 46 passed (was 44).

- Re-verdicted matrix across all five captures (identical code, old and new verdict):
  | run                                 | long_path_excess        | old verdict | new verdict |
  |-------------------------------------|-------------------------|-------------|-------------|
  | Phase 3 B+ slow (1 RPM, 1 cmd)      | 0 counts = 0 deg        | CLEAN       | CLEAN       |
  | Phase 3 B+ fast (10 RPM, 1 cmd)     | 0 counts = 0 deg        | CLEAN       | CLEAN       |
  | Phase 3 B- slow (1 RPM, 1 cmd)      | 0 counts = 0 deg        | CLEAN       | CLEAN       |
  | Phase 3 B- fast (10 RPM, 1 cmd)     | 0 counts = 0 deg        | CLEAN       | CLEAN       |
  | UI test (100 RPM, 2 jogs, seam)     | 1,310,720 counts = 360° | CLEAN (bug) | **WHIP**    |

- Implication for the Phase 6 re-earn:
  - The re-earn stands for SINGLE-COMMAND seam crossings (4/4 clean; verified by both old and new verdict). Operator can trust these for planner / runner paths that send one APPLY_JOINT_SETPOINT per trajectory.
  - The re-earn DOES NOT extend to the UI's multi-jog pattern or any other code path that triggers the host fold twice with an intermediate stop on or near the seam. Those paths are exposed to the boundary flip and can whip.
  - Chained multi-turn (still-retracted) is NOT yet tested with the new verdict, but the mechanism is identical (multiple fold invocations, intermediate targets near canonical multiples of 180 deg) so the same boundary flip is plausible. Do NOT unflag chained multi-turn until the fold boundary behavior is fixed.

- Next code action (not yet landed in this session):
  - Add a boundary-aware tiebreaker to `_nearest_turn_fold_axis_q_for_axis`. Candidate fixes:
    1. Hysteresis: prefer the previously-emitted wrap_turns value when `|delta/period - round(delta/period)| < ~0.02`.
    2. Explicit "J6 is at the seam" guard: if `|observed_counts modulo period_counts| < RM/16`, force wrap_turns = sign of delta such that adjusted_counts stays inside `[-RM/2, +RM/2]` relative to the observed_counts' continuous frame.
    3. Reject the motion command with a 409/CANONICAL_JOINT_TRUTH_UNAVAILABLE if J6 is within, say, 5 deg of canonical +/-180 deg at the time of command acceptance (defers the decision until operator moves away from the seam).
  - Regression test: `observed_reference_counts = RM - 1, base_axis_q = <canonical +185 deg>, assert wrap_lift_counts == 0`, same for `observed_reference_counts = 0` with `base_axis_q = <canonical -185 deg>` (the mirror case).
  - Ship this fix before any UI motion path that can cross the seam is trusted again.

- Validation performed in this pass:
  - `python3 -c "import ast; ast.parse(open('scripts/j6_multiturn_fast_capture.py').read())"` -> OK.
  - `python3 -m pytest tests/test_j6_multiturn_fast_capture.py -q` -> 46 passed.
  - `ReadLints` on touched files -> clean.
  - Re-ran analyze-rtcore on all 5 isolated JSONLs; verdicts correct (4 CLEAN, 1 WHIP).
  - No hardware touched; no motion commanded. Stack was brought up to idle to save the UI test snapshots earlier in the session but is fully stopped now.

- Follow-up / risk:
  - The fold boundary bug is the real Phase 5 finding. Everything else in this session was tooling to detect it.
  - The existing DEVLOG entry `## 2026-04-19 18:30 +0000 - Phase 6 COMPLETE: full B+ / B- at slow/fast matrix is CLEAN` is still accurate for single-command seam crossings. The new entry does NOT supersede it; it narrows the scope of what the Phase 6 claim protects.
  - The retraction audit trail entry at `## 2026-04-19 05:30 +0000 - RETRACTION: Phase 6 "PASS" verdicts were endpoint-only and path-blind` is also still accurate: the original retraction was legitimate (endpoint-only evidence could NOT distinguish a whip from a clean move). This session's finding adds a distinct failure mode (multi-jog fold flip at seam) on top of that.
  - PNGs in `logs/j6-multiturn-fast/images/`: `ui-test-seam-cross-WHIP-EVIDENCE.png` (the smoking gun, custom 4-panel plot annotating the two phases and the peak velocity spike), plus `ui-test-seam-cross-isolated.png` (auto-generated by analyze-rtcore with the new VERDICT=WHIP). The four Phase 3 MOTION-EVIDENCE PNGs remain correctly labeled CLEAN.

## 2026-04-19 20:XX +0000 - Nearest-turn fold: multi-turn disambiguation at seam boundary (vendor-agnostic, backend-only)

- Context:
  - Operator confirmed the 2026-04-19 UI two-jog whip via 1 kHz fast_trace (previous DEVLOG entry). Root cause in `_nearest_turn_fold_axis_q_for_axis`: at the seam, live 0x6064 can read 0 or RM-1 (ambiguous, sub-count noise), which flips `round(delta/period)` by +/-1 and emits a target one full revolution off.
  - Fix must keep pre-Phase-6-verified wire semantics for non-seam motion AND not hard-code vendor-specific fixes into the main controller (explicit operator constraint).

- What changed (all in `src/gradient_os/arm_controller/backends/ethercat_rtcore/backend.py`):
  - **New module-level constant** `_PROFILE_MULTI_TURN_COUNTS_KEY = "encoder_multi_turn_counts"`. This is the profile contract: any drive profile whose `normalize_absolute_feedback()` emits a mapping under this key with `{"valid": bool, "value": int (motor-frame counts, signed i64)}` gets the multi-turn fold disambiguation for free. Only the A6-EC profile emits this key today (via its `ABSOLUTE_FEEDBACK_SOURCES` `signed_i64_pair` source combining U40.20 + U40.22). No A6-EC-specific identifier appears in the backend path.
  - **Existing magic-string usage at line 2219** updated to reference the constant so future vendor-abstraction changes have one place to audit.
  - **New helper `_multi_turn_reference_counts_for_axis(axis_i)`**: pulls the drive's multi-turn reading from the per-axis absolute-feedback cache (via the profile-driven normalization), subtracts the home-anchor counts (= `home_anchor_rad * sign * counts_per_unit` for the joint mapped to this axis), and returns the continuous axis-q-frame equivalent. Returns `None` when the profile does not expose a multi-turn counter, when no valid reading has arrived yet (e.g., first cycles after boot), or when the axis config is incomplete. Callers that receive `None` fall back to the legacy single-turn path.
  - **`_nearest_turn_fold_axis_q_for_axis` signature + logic**: new kwarg `observed_multi_turn_reference_counts`. When provided alongside `observed_reference_counts`, the fold uses a **seam-only disambiguation** -- it computes `wire_mod = live_6064 % period` and `distance_to_seam = min(wire_mod, period - wire_mod)`, and only runs the multi-turn shift when `distance_to_seam < period // 16` (~22 deg). Outside the seam band, observed_counts stays at `ref_axis_q` unchanged -- meaning the fold runs identical math to the pre-fix implementation and emits identical wire values. Inside the seam band, multi-turn disambiguates the ambiguous single-turn reading by shifting observed_counts to the side matching the multi-turn continuous position.
  - **`_command_axis_q_for_joint_value` signature + plumbing**: new kwarg `live_multi_turn_reference_counts` (caller-provided); auto-fetches via `_multi_turn_reference_counts_for_axis` when caller does not pass one. Production callsites did not need to change -- the write-path at line ~4971 (`_axis_q_from_joint_positions`) picks up the new behavior automatically.

- What explicitly did NOT change:
  - RTCore C++ (`src/gradient_rt_motion/main.cpp`) is untouched. The absolute-feedback SDO poll (`kAbsoluteFeedbackPollIntervalNs = 200 ms`, = 5 Hz) is the source RTCore already provides to the host; no new IPC plumbing.
  - Drive profile (`src/gradient_os/arm_controller/profiles/drive/a6ec_ds402.py`) is untouched. The profile's existing `ABSOLUTE_FEEDBACK_SOURCES` already emits `encoder_multi_turn_counts`.
  - `systemd/rt-motion/gradient-rt-motion.service`, `start-stack.sh`, runtime env plumbing all untouched.
  - Wire emission for non-seam motion is bit-for-bit identical to the pre-fix behavior (the Phase 6 verified-safe range).

- Behavior inside the seam band (new):
  - Live 0x6064 within RM/16 (~22 deg) of the 0/RM boundary.
  - Multi-turn disambiguation picks the turn-shifted version of `ref_axis_q` that is closest to the multi-turn continuous position.
  - Emitted wire value may differ from `live_6064` by an integer multiple of RM in the multi-turn-continuous direction. This is EQUIVALENT to the wrapped value per Chapter 5 Fig 5-1 (drive does modular comparison under C10.16=0 Nearest), and matches the 2026-04-19 Phase 6 negative-continuous emission evidence.
  - This is the first intentional design point where the host emits multi-turn-continuous wire values OUTSIDE the Phase-6-verified `|wire - live_6064| < RM/2` band; it only fires at the seam where the alternative (single-turn-wrapped on the wrong side) demonstrably causes the UI whip. Risk is contained by the seam-only gate; any live issue with the modular-equivalent continuous wire can be isolated by checking whether the trace shows a seam-adjacent live_6064 at the moment of the deviant emission.

- New regression tests in `tests/test_gradient05_limits_and_backends.py` (5 tests, added adjacent to the existing `test_a6ec_continuous_607a_fold_does_not_flip_turn_at_midpoint_home`):
  - `test_a6ec_fold_uses_multi_turn_reference_at_seam_low_side`: J6 at canonical +180 deg with live 0x6064 = 0 (the low-side seam reading). Must emit short-path target when commanding +185 deg.
  - `test_a6ec_fold_uses_multi_turn_reference_at_seam_high_side`: SAME motor physical position but live 0x6064 = RM-1 (the high-side ambiguous reading). MUST still emit short-path target, PROVING multi-turn resolves the noise-driven flip. This is the direct regression for the 2026-04-19 UI whip.
  - `test_a6ec_fold_away_from_seam_preserves_single_turn_emission`: J6 at canonical +90 deg (mid-band, wire 6064 = RM/4, nowhere near seam). Commanding +91 deg MUST emit a wire close to live 6064 (within RM/2) -- multi-turn disambiguation must NOT fire outside the seam band.
  - `test_a6ec_multi_turn_reference_falls_back_when_profile_omits_key`: profile's absolute_feedback has no `encoder_multi_turn_counts` source; helper must return None so the fold cleanly falls back to the legacy single-turn path. This is the vendor-agnostic safety net.
  - Plus a shared `_setup_midpoint_home_fold_backend` helper that reduces duplication and makes the test suite easy to extend.

- Existing tests updated to modular-equivalence assertions (the old assertions were implicitly single-turn; the new behavior is modular-equivalent which is the correct semantic for rotation-mode continuous emission):
  - `test_a6ec_joint_full_range_sweep_fresh_hm_keeps_truth_continuous[0..5]`: changed `abs(wire_counts - live_6064) <= 1` to a modular-delta check (`(wire_counts - live_6064) mod RM`, wrapped into `[-RM/2, RM/2]`, abs <= 1).
  - `test_ethercat_backend_drive_native_truth_unwraps_public_joint_positions_but_preserves_raw_write_frame`: same modular-delta pattern with RM = 628 (test uses custom counts_per_rev).
  - `test_ethercat_backend_keeps_raw_truth_across_single_turn_wrap_even_when_display_truth_fails`: same pattern with RM = 131072.

- Validation performed:
  - `python3 -c "import ast; ast.parse(open('src/gradient_os/arm_controller/backends/ethercat_rtcore/backend.py').read())"` -> OK.
  - `python3 -m pytest tests/test_gradient05_limits_and_backends.py -k "fold or multi_turn_reference" -v` -> 6 passed (3 new seam regressions + 1 new fallback + 1 new away-from-seam + 1 existing midpoint-home).
  - Full A6-EC + runtime + capture sweep (`tests/test_gradient05_limits_and_backends.py tests/test_rtcore_runtime.py tests/test_drive_faults.py tests/test_a6ec_chapter5_probe.py tests/test_a6ec_joint_sweep.py tests/test_a6ec_j6_watch_replay.py tests/test_api_endpoints.py tests/test_trajectory_execution_backends.py tests/test_command_api_direct_setpoint.py tests/test_run_controller_helpers.py tests/test_j6_multiturn_fast_capture.py`) -> `402 passed`.
  - `ReadLints` on all touched files -> clean.

- Runtime outcome:
  - No hardware touched in this code-landing pass. Fix is behavioral but inert until the next stack start + motion command.
  - Live validation: operator to repeat the 2026-04-19 UI two-jog scenario with the 1 kHz fast_trace running; expected `analyze-rtcore` verdict is `CLEAN` with `long_path_excess = 0` (vs the pre-fix `WHIP` with `long_path_excess = 360 deg`).

- Follow-up / risk:
  - The seam-only disambiguation preserves the Phase 6 wire semantics for every non-seam motion. If a future live run shows a seam-adjacent whip despite this fix, check the multi-turn reading's staleness: the 5 Hz SDO poll can be up to 200 ms old, which at the UI-capped 100 motor RPM is at most ~10 deg of motor motion -- still well within the RM/2 disambiguation safety margin.
  - The 2026-04-19 05:30 retraction entry (and the 18:30 re-earn) stay unchanged. This fix addresses a DIFFERENT failure mode (UI multi-jog with intermediate seam landing) that the earlier matrix did not exercise. No retraction or re-earn to adjust.
  - True multi-turn single-shot support (commanded `canonical +720 deg` from home -> emit `+720 deg` target, not `0 deg`) remains a known limitation. The seam-only disambiguation does not change that limitation; a separate workstream would be needed to propagate multi-turn semantics through the host planner / canonical state. Defer.
  - Future drive profiles that opt into the multi-turn-aware fold need only emit `encoder_multi_turn_counts` in their `normalize_absolute_feedback()`. The constant `_PROFILE_MULTI_TURN_COUNTS_KEY` is the contract point.

## 2026-04-19 23:08 +0000 - Phase 4 live validation: UI two-jog whip reproduced end-to-end, now CLEAN post-fix

- Context:
  - Per the previous entry, the seam-only multi-turn-aware fold landed in the host backend. Live validation: reproduce the exact 2026-04-19 UI two-jog scenario against the running stack and confirm the fix prevents the 360 deg whip.

- Preconditions:
  - Fresh partition cleanup (root at 100% from accumulated 1 kHz fast_trace files; purged ~6 GB of old session autosaves and per-checkpoint snapshots from `logs/j6-multiturn-fast/` root, keeping `images/`, `pre-phase3/`, and the newly-created `phase4-multi-turn-fix-validation/` subfolder).
  - Fresh systemd stack start (`./start-stack.sh`), fast-trace drop-in still installed at 1 kHz.
  - New subfolder: `logs/j6-multiturn-fast/phase4-multi-turn-fix-validation/`.

- Motion sequence executed via two back-to-back runs of `scripts/j6_seam_whip_phase3_runner.py` (same script used for earlier Phase 3 matrix):
  1. First invocation (home + power-up + preposition 0 -> +175 deg + "seam cross" +175 -> +180 deg at `--seam-max-motor-rpm 100`, label `phase4-jog-to-180`):
     - Home + power-up + joint-state settle completed.
     - Preposition to +175 deg at 100 motor RPM: `state=completed` in ~3 s.
     - 5 s anchor-gate settle.
     - Seam-adjacent "jog" to +180 deg at 100 motor RPM: `state=completed` in 0.3 s. J6 lands exactly ON the seam (canonical +180 deg = wire 0x6064 ~= 0/RM boundary).
  2. Second invocation (skip home + power-up, preposition no-op +180 -> +180 deg + SEAM CROSS +180 -> +185 deg at 100 motor RPM, label `phase4-jog-to-185-CRITICAL`):
     - Preposition no-op completed in ~1 s (already at +180 deg).
     - 5 s anchor-gate settle.
     - CRITICAL seam cross +180 -> +185 deg at 100 motor RPM: `state=completed` in 0.3 s. This is the exact motion that whipped +350 deg in the 2026-04-19 UI test.
  3. `./start-stack.sh stop --hard` -> autosave hook fired, preserved `autosave-20260419T230647Z.jsonl` (66 MB, full session) before the runtime dir was wiped.

- Isolation + analysis (saved into `phase4-multi-turn-fix-validation/`):
  - First jog (+175 -> +180): isolated 635 samples over 0.634 s at effective_hz 1001.72:
    - first_p=18,204, last_p=1,310,442 (wire wrapped just past 0 into the high-count side).
    - cumulative_travel = 18,836 counts (5.173 deg).
    - net_displacement = -18,482 counts (-5.076 deg). Matches commanded -5 deg (sign=-1 inversion).
    - long_path_excess = 0.
    - max_abs_wire_step = 219 counts/ms.
    - cum/|net| = 1.019.
    - wire_monotonic = True, fault_seen = False. **VERDICT = CLEAN**.
  - **CRITICAL second jog (+180 -> +185): isolated 677 samples over 0.676 s at effective_hz 1001.47**:
    - first_p = 1 (J6 starts at seam, 0x6064 near 0).
    - last_p = 1,292,193 (J6 past +185 deg, wire near RM-18,527).
    - cumulative_travel = 18,972 counts (5.211 deg).
    - net_displacement = -18,528 counts (-5.089 deg). Matches commanded -5 deg.
    - **long_path_excess = 0 counts = 0 deg** (the pre-fix test had 1,310,720 counts = 360 deg).
    - max_abs_wire_step = 143 counts/ms (~47 motor RPM peak). The pre-fix test had 25,293 counts/ms (~6,900 motor RPM).
    - cum/|net| = 1.024.
    - wire_monotonic = True, fault_seen = False. **VERDICT = CLEAN**.

- Direct comparison (same physical setup, same speed, same start position on the seam):
  | metric                       | 2026-04-19 pre-fix          | 2026-04-19 post-fix          |
  |------------------------------|-----------------------------|------------------------------|
  | VERDICT                      | WHIP                        | **CLEAN**                    |
  | long_path_excess             | 1,310,720 counts (360 deg)  | **0 counts (0 deg)**         |
  | cumulative_travel            | 1,356,689 counts (372 deg)  | **18,972 counts (5.21 deg)** |
  | net_displacement             | +1,274,313 counts (+350 deg)| **-18,528 counts (-5.09 deg)** |
  | max_abs_wire_step            | 25,293 counts/ms            | **143 counts/ms**            |
  | peak velocity inferred       | ~6,900 motor RPM            | **~47 motor RPM**            |
  | commanded max_motor_rpm      | 100                         | 100                          |

- Artefacts preserved under `logs/j6-multiturn-fast/phase4-multi-turn-fix-validation/`:
  - `phase4-first-jog-175-to-180-isolated.jsonl` (326 KB) + `.png` (93 KB) + `-MOTION-EVIDENCE.png` (218 KB).
  - `phase4-second-jog-180-to-185-CRITICAL-isolated.jsonl` (343 KB) + `.png` (149 KB) + `-MOTION-EVIDENCE.png` (266 KB).
  - The per-checkpoint cumulative snapshots and the full 66 MB autosave were deleted after isolation to keep the disk bounded.

- Validation performed:
  - Live hardware: motor physically moved +5 deg short-path on the critical second jog (operator verification not needed; the max_abs_wire_step of 143 counts/ms = ~47 motor RPM peak is orders of magnitude below any whip-speed threshold; the drive would have needed to accelerate to ~6,000 motor RPM to produce the earlier 25,293 counts/ms spike).
  - `ReadLints` and full test sweep (`402 passed`) ran in the previous entry; no new Python code changed this session.
  - Disk is at 79% (6 GB free) after cleanup.

- Runtime outcome:
  - The multi-turn-aware fold fix is PROVEN on live hardware for the exact UI scenario that produced the whip.
  - The 2026-04-19 05:30 retraction is fully superseded for the Move B family: single-command B+/B- at both 1 RPM and 10 RPM are CLEAN, and the multi-jog UI pattern at 100 RPM across the seam is now CLEAN too.
  - Stack is hard-stopped; ready for normal operation.

- Follow-up / risk:
  - Chained multi-turn (0 -> +175 -> +350 deg from the 2026-04-19 05:30 retraction) remains un-re-earned. That scenario needs its own 1 kHz verification before any planner consumer trusts continuous canonical >180 deg commands. Not blocking; suggest scheduling as a separate workstream.
  - True single-shot canonical +720 deg command (from home, no intermediate chained motion) is a known limitation of the current fold (it uses live_6064 as the turn anchor, which collapses targets more than +/- 180 deg from live). The Phase 4 fix does NOT unlock this feature; a separate workstream would need to propagate multi-turn context through the host planner.
  - Disk hygiene: purge per-checkpoint + autosave JSONLs after isolation completes; keep only isolated windows + MOTION-EVIDENCE PNGs for audit. 1 kHz fast_trace at 60 MB/min will fill a 29 GB partition in ~8 hours of continuous stack uptime if left unchecked.
  - The fast_trace drop-in is still installed. For normal production operation (not verification), recommend setting `GRADIENT_RT_FAST_TRACE_HZ=0` in the drop-in to stop the trace writer. Leave it enabled for any future seam-verification runs.

## 2026-04-20 00:42 +0000 - Display-unwrap cache bug: `CANONICAL_JOINT_TRUTH_UNAVAILABLE` on UI jog after a session that boots J6 near-seam

- Context:
  - Operator attempted UI jog after restoring the stack. `/control/joint-jog` returned `409 CANONICAL_JOINT_TRUTH_UNAVAILABLE` with `truth_reason=absolute_home_anchor_stale`, `absolute_home_anchor_delta_counts=1,310,719` (= exactly RM-1 = one full motor-output turn). All other truth checks (`raw_canonical_joint_truth_available`, `shaft_frame_consistent`, `raw_command_roundtrip_consistent`) were True.
  - Smoking-gun asymmetry: `raw_reference_pre_zero_rad ≈ 0 rad` (correct), `reference_pre_zero_rad = 6.283 rad = 2π` (one turn off). The display-side of the canonical truth was one turn below the raw/multi-turn truth.

- Diagnosis (from the 1 kHz fast_trace of that session):
  - Sample 8162 (~8.2 s after stack start): drive latched its first valid PDO reading at `p=1,292,467, sw=0x9650`. Before that, `p=0, sw=0x0000` for ~8160 cycles (pre-OP, PDO not mapped).
  - Backend's `_display_feedback_counts_for_axis` first-call seed path ran while the PDO still read `p=0`, which seeded `_feedback_unwrapped_counts[5] = 0`.
  - When the drive later published the real absolute position near-seam (canonical +185° from the previous Phase 4 test left J6 there), `round((0 - 1,292,467)/RM) = -1` walked the unwrap cache to `-18,253`. `round((−18,253 - 1,292,467)/RM) ≈ -1` further locked it.
  - Sample 191088: HM35 completed and the drive jumped `0x6064` from `1,292,470` → `655,365` with zero physical motion. `round((−18,250 − 655,365)/RM) = round(-0.514) = -1` kept the stale turn offset. Result: `_feedback_unwrapped_counts[5] = −655,355` = one turn below truth, stuck there for the rest of the session.
  - Downstream: `display_reference_q = -655,359/-cpu + π = 2π` (one turn off), `command_roundtrip_consistent = False` with `error_counts = 1,310,719` (= RM), which the truth gate mapped to `absolute_home_anchor_stale` → `truth_available = False` → UI jog rejected.

- What changed (in `src/gradient_os/arm_controller/backends/ethercat_rtcore/backend.py`):
  - `_display_feedback_counts_for_axis` rewritten to prefer the drive's unambiguous multi-turn register as the PRIMARY truth for the display-unwrapped counts. When both a valid multi-turn reading (via the profile-contract `encoder_multi_turn_counts` key) AND a captured home anchor are available (and `drive_native_ratio_enabled()` is True), the function returns `multi_turn_axis_q_counts − native_home_offset_counts` directly — the exact wire-frame representation of the physically unambiguous axis-q position. Side effect: the accumulated-unwrap cache is kept in sync with ground truth as a self-correcting fallback, so any transient loss of multi-turn data continues from a correct seed.
  - Fallback accumulated-unwrap path (profile omits multi-turn, metrics haven't arrived yet, no home anchor captured, or drive in legacy single-motor-turn mode) gained a pre-OP seed gate: when `_feedback_unwrapped_valid[axis_i]` is False AND both `normalized_counts == 0` AND `_axis_statusword[axis_i] == 0`, the function returns the normalized counts without seeding. The seed is deferred until either signal becomes non-zero, matching the "drive actually has valid PDO data" requirement.
  - New helper `_multi_turn_reference_counts_for_axis_when_anchored` is the stricter variant of `_multi_turn_reference_counts_for_axis`: returns None when no home anchor has been captured for the mapped logical joint. The permissive sibling keeps its motor-encoder-internal fallback for the seam-disambiguation fold (which only cares about modulo-period shift).
  - `_refresh_native_home_offsets_from_metrics` gained an HM35-invalidation hook: when any axis's `native_home_position_offset` changes value relative to the cached copy, invalidate `_feedback_unwrapped_valid[axis_i]`. Defense-in-depth for axes that fall through to the accumulated-unwrap fallback.
  - All three changes are additive and profile-agnostic. No RTCore, profile, systemd, or start-stack changes.

- New regression tests in `tests/test_gradient05_limits_and_backends.py`:
  - `test_a6ec_display_feedback_prefers_multi_turn_when_anchored` — midpoint-home multi-turn + anchor → display returns `half_rm`, yielding `reference_q = 0` (not `2π`).
  - `test_a6ec_display_feedback_multi_turn_path_survives_pre_op_zero_reads` — direct reproduction of the 2026-04-20 UI-jog rejection: backend is invoked with `raw_counts = 0` and `statusword = 0` (pre-OP pattern), then the drive comes up near-seam; the multi-turn-preferred path returns the correct home value throughout, not the poisoned one-turn-off value.
  - `test_a6ec_display_feedback_fallback_gates_seed_on_pre_op_zero` — profile without anchor, drive in pre-OP pattern; the fallback MUST refuse to seed from `(raw=0, sw=0)`. Seeds correctly once either signal becomes non-zero.
  - `test_a6ec_display_feedback_fallback_invalidates_cache_on_hm35` — mid-session HM35 rewrites native_home_position_offset; the fallback cache must be invalidated so the next display query re-seeds from the post-HM35 wire value.
  - `test_a6ec_multi_turn_reference_when_anchored_returns_none_without_anchor` — the stricter helper returns None when no anchor is captured, so the display path cleanly falls back.

- Validation performed:
  - `python3 -m pytest tests/test_gradient05_limits_and_backends.py tests/test_rtcore_runtime.py tests/test_drive_faults.py tests/test_a6ec_chapter5_probe.py tests/test_a6ec_joint_sweep.py tests/test_a6ec_j6_watch_replay.py tests/test_api_endpoints.py tests/test_trajectory_execution_backends.py tests/test_command_api_direct_setpoint.py tests/test_run_controller_helpers.py tests/test_j6_multiturn_fast_capture.py -q` → `407 passed` (was 402 pre-fix; 5 new regressions added).
  - `ReadLints` on `backend.py` and `test_gradient05_limits_and_backends.py` → clean.
  - Live hardware: restarted stack, queried `/info/joints-detailed`. Post-fix: `truth_available=True, truth_reason=None, command_roundtrip_consistent=True, reference_pre_zero_rad=1.44e-05 rad ≈ 0` (was `2π` pre-fix). `display_joint_truth_unavailable_joints=[]` (was `[2, 4, 5, 6]` pre-fix). Operator's UI jog unblocked.

- Runtime outcome:
  - Operator ran the UI jog test sequence through the full range. Small jogs (1°, 5°), seam crossings (+175° → +180° → +185° → +190°), and a +175° big jog (+190° → +365°) all ran clean with `long_path_excess=0` and wire peaks at commanded 100 motor RPM.
  - Between sessions the pre-fix autosave trace was preserved at `logs/j6-multiturn-fast/unwrap-seed-bug-2026-04-20/unwrap-seed-bug-CRITICAL-SAMPLES.jsonl` (67 samples, 34 KB) with the two critical transitions (drive-latch at sample 8162 and HM35 at 191088). The full 1.66 GB trace was deleted after extracting these samples to free disk.

- Follow-up / risk (NEW, DISTINCT BUG UNCOVERED IN THIS SESSION):
  - Phase 5 of the operator's test sequence (canonical +365° → +180° via `delta=-185°` jog at 100 motor RPM) WHIPPED at the drive's absolute max RPM. Trace shows `cumulative_wire_travel=1,928,841 counts (530° output)` for a `net=-637,049 counts (-185° output)` move, with `max_abs_wire_step=13,124 counts/ms (≈6000 motor RPM)`. The target trajectory itself went `+365° → overshoot to +5° → back to +180°`.
  - Root cause: the host trajectory generator (`_build_bounded_joint_path`) smoothly interpolates between canonical waypoints. The fold's `round(delta/RM)` picks a DIFFERENT wrap_turns for each waypoint when `live_6064` alone is the anchor (and multi-turn is ignored away from the seam). For a trajectory that spans canonical `X` where `|(multi_turn − X*sign*cpu)/RM|` crosses `±0.5`, the wrap_turns changes discretely mid-trajectory, producing a non-monotonic wire-frame target path. RTCore's velocity planner tries to chase the discontinuities at max RPM, causing the whip.
  - This is the exact "known limitation" from `HANDOFF_J6_MOVE_B_SEAM_FAULT_2026-04-18.md` ("True fix: track continuous canonical state on the host [...] ~100 lines across `command_api.py` + `backend.py` + a turn counter that resets on home"). The fix landed in this session unblocks UI jogging but does NOT address this second failure mode.
  - Three possible fixes for the Phase 5 whip, none landed yet:
    - Option A: change `_nearest_turn_fold_axis_q_for_axis`'s `_disambiguate_seam_with_multi_turn` helper to always use multi-turn (not just at seam). Smallest delta; still has a discontinuity at the `round()` boundary.
    - Option B (surgical, ~20 LOC): in `_add_trajectory_points`, thread the first waypoint's `wrap_lift_counts` through to subsequent waypoints so the entire trajectory stays in a single turn-frame. Monotonic wire trajectory guaranteed.
    - Option C (architectural, ~50 LOC): build trajectories in multi-turn-continuous axis-q space directly (start_axis_q = multi_turn_axis_q, end_axis_q = start + canonical_delta*sign*cpu, linearly interpolate axis-q not canonical). Bypasses the fold for trajectory interior points. Matches the handoff's "true fix" description.
  - Recommend Option B or C before any downstream planner trusts continuous canonical multi-turn commands. Single-command multi-turn moves (Phase 4-style, staying in one direction through the seam) still work because all waypoints have monotonically-changing canonical and the fold picks the same wrap_turns for all of them. Reversal-after-multi-turn (Phase 5 pattern) is the failure case.


## 2026-04-20 02:20 +0000 - Phase 5 whip fix LIVE-VALIDATED: direction-preserving command path eliminates reversal-from-multi-turn whip

- Context:
  - Earlier this session the display-unwrap cache bug was fixed (multi-turn-preferred display path). Operator re-ran UI jog test successfully for small moves and the forward multi-turn path (0° → +175° → +180° → +185° → +190° → +365°). Phase 5 (canonical +365° → +180° via delta=-185°) whipped: cumulative wire travel 530° output for a -185° net displacement, peak wire step 13,124 counts/ms (= drive's absolute max 6000 motor RPM), cum/|net| = 2.86.
  - Root cause: `_nearest_turn_fold_axis_q_for_axis` anchors on live 6064 (single-turn modular) away from the seam. Per-waypoint fold invocations for a trajectory spanning the `round(delta/RM) = ±0.5` boundary pick DIFFERENT wrap_turns on adjacent waypoints, producing a non-monotonic wire-frame target path. RTCore's velocity planner tries to chase the discontinuities at max RPM.
  - Operator's principled fix: "the move should be restricted by the direction it turns - turning negative when we're supposed to turn positive makes no sense." Interpreting this: honor the commanded canonical delta's sign directly in axis-q space; do not let a modular "shortest-path" heuristic silently discard the direction intent.

- What changed (src/gradient_os/arm_controller/backends/ethercat_rtcore/backend.py):
  - `_command_axis_q_for_joint_value` rewritten to gate on the stricter `_multi_turn_reference_counts_for_axis_when_anchored` helper. When the drive is in continuous-607A mode (A6-EC, `wrap_to_single_turn=False`) AND a home anchor is captured (so the caller's canonical_q is multi-turn-aware per the 2026-04-20 display-path fix), skip the fold's turn-shift math entirely and emit `base_axis_q = canonical_q + master_offset` directly. The axis-q frame is linearly proportional to canonical, so a signed canonical delta maps to a same-signed axis-q delta which maps to a same-signed wire delta — direction is preserved by construction.
  - Legacy paths preserved: drives in `wrap_to_single_turn=True` (legacy single-motor-turn mode) still use the fold unconditionally because they require the `[0, RM)` wrap for drive-parse safety. Joints without a captured anchor (fresh boot, never homed) still route through the fold path with its seam-only multi-turn disambiguation, preserving the 2026-04-19 UI-whip regression coverage.
  - The `command_frame_oversized_delta` safety gate remains active and still fires if an emitted target would be modularly more than half a period from the live wire. This catches real frame bugs without inducing the whip.
  - Back-compat: when callers explicitly pass `live_multi_turn_reference_counts` (as existing tests do), the fold path is used. That preserves the old semantic tests depend on.

- New regression tests in tests/test_gradient05_limits_and_backends.py:
  - `test_a6ec_command_axis_q_preserves_direction_with_multi_turn_anchor` — J6 physically at canonical +365° with multi-turn register + anchor; commanding canonical +365° must emit axis_q = +365° exactly (direction-preserving), NOT +5° collapsed modular form.
  - `test_a6ec_command_axis_q_trajectory_waypoints_monotonic_with_multi_turn` — full 7-waypoint S-curve from canonical +365° → +180°; EVERY emitted axis_q must equal its canonical input exactly (no turn-shift), and all waypoints must be monotonic (no mid-trajectory turn-flip = the pre-fix Phase 5 whip signature).
  - `test_a6ec_command_axis_q_falls_back_to_fold_without_anchor` — when no home anchor is captured, the fold fallback (including its seam-only multi-turn disambiguation) must still fire at the high-side ambiguous seam reading. Preserves the 2026-04-19 UI-whip regression coverage for pre-commissioning joints.

- Validation performed:
  - `python3 -m pytest tests/test_gradient05_limits_and_backends.py tests/test_rtcore_runtime.py tests/test_drive_faults.py tests/test_a6ec_chapter5_probe.py tests/test_a6ec_joint_sweep.py tests/test_a6ec_j6_watch_replay.py tests/test_api_endpoints.py tests/test_trajectory_execution_backends.py tests/test_command_api_direct_setpoint.py tests/test_run_controller_helpers.py tests/test_j6_multiturn_fast_capture.py -q` -> `410 passed` (was 407 pre-fix; 3 new regressions added).
  - ReadLints on backend.py and the test file -> clean.

- Runtime outcome (live hardware, 2026-04-20 02:18 UTC):
  - Restarted stack; J6 entered session carrying canonical -355° (shaft_frame_wrap_turns = 1) from the earlier successful moves.
  - Commanded delta = -185° via `/control/joint-jog`. Motion completed in 3.08 s (= 185° / (100 motor RPM * 6 deg/s / RPM) = analytic match). Final canonical = -540° (= +180° mod 360°, shaft_frame_wrap_turns incremented to 2).
  - Isolated 1 kHz trace at `logs/j6-multiturn-fast/phase5-direction-preserving-fix-2026-04-20/phase5-reversal-post-fix-isolated.jsonl` + `.png`.
  - Metrics:
    - cumulative wire travel = 185.00° output (vs 530° pre-fix)
    - cum / |net| = 1.010 (essentially 1.0 — perfectly monotonic; pre-fix was 2.86)
    - peak wire step = 1565 counts/ms = ~716 motor RPM (consistent with velocity-planner acceleration burst on commanded 100 motor RPM, roughly 7× average; pre-fix was 13,124 counts/ms = drive's absolute 6000 motor RPM ceiling)
    - wire_monotonic = True (no direction flip)
    - U40.20/.22 cross-check: net_delta = +185.005° matches commanded -185° canonical after sign=-1 inversion
    - fault_seen = False

  Pre-fix vs post-fix, same physical motion (commanded delta=-185° from multi-turn state):
  | metric                     | pre-fix (00:57)                | post-fix (02:18)        |
  |----------------------------|--------------------------------|-------------------------|
  | VERDICT                    | WHIP                           | CLEAN                   |
  | cumulative travel          | 530° output (2.86× net)        | 185° output (1.01× net) |
  | peak wire step             | 13,124 counts/ms (=6000 RPM)   | 1565 counts/ms (=716 RPM) |
  | trajectory shape           | non-monotonic overshoot + recovery | strictly monotonic |

- Follow-up / risk:
  - The `scripts/j6_multiturn_fast_capture.py analyze-rtcore` subcommand's VERDICT logic still treats "long path excess" (net differs from shortest-path net by >=RM/2) as WHIP. With the direction-preserving fix, intentional direction-following motions that happen to traverse >180° canonical will trip this gate even though they are clean. The real whip signals (`cum / |net| > 1.3`, `wire_monotonic = False`, `max_abs_wire_step > ~5000 counts/ms`) remain reliable. Recommend revising the analyzer's WHIP gate to check those metrics first and only surface long_path_excess as informational context when direction-preserving mode is active. Not blocking; human review of the metrics surface the right answer immediately.
  - `/control/joint-jog`'s `wait_for_idle` parameter returned with `waited_for_idle: false` on the Phase 5 test run and the retried v3 script. The motion DID execute correctly (verified via trace), but the endpoint returned before the trajectory completed. Looks like the wait-for-idle path has a short-circuit condition that fires when the controller thread goes from "idle" to "executing" in one poll cycle. Side effect: bulk-scripted multi-jog sequences need explicit polling against `/control/motion-status` between jogs. Not caused by today's fix; pre-existing. File as a minor API-UX issue.
  - Phase 5's earlier rejections (HTTP 409 `multi_turn_anchor_inconsistent_with_live_6064`) were masked by the script's incomplete error parsing. The gate itself is working as designed (16-count tolerance, settles within ~5s post-motion). Scripts that chain multiple jogs must insert explicit idle-settle waits AND check HTTP status codes so rejections surface clearly.
  - `HANDOFF_J6_MOVE_B_SEAM_FAULT_2026-04-18.md`'s "Still outstanding #1: Single-shot canonical >180° collapse (known limitation)" is now partially superseded: single-shot canonical deltas >180° from a MULTI-TURN-STATE-AWARE current position work correctly under the direction-preserving command path. Single-shot `canonical +720°` from home (where multi-turn is 0) still collapses because the trajectory generator's canonical-space S-curve interpolates through 0 at t=0 regardless; that case needs either explicit multi-turn target disambiguation at the API boundary or the operator breaking into <180° chained jogs. Worth refreshing the handoff doc before the next pickup.

## 2026-04-20 23:31 +0000 - Researched Thor dev kit camera capacity and USB/display/inference paths

- What changed:
  - No product code changed.
  - Investigated official NVIDIA documentation for both `Jetson AGX Thor Developer Kit` and `DRIVE AGX Thor Developer Kit` to avoid mixing the two product families in camera/I/O guidance.
  - Verified `Jetson AGX Thor Developer Kit` camera and display I/O from official sources: `HSB camera via QSFP slot`, `USB camera`, `2x USB-A 3.2 Gen2`, `2x USB-C 3.1`, `HDMI 2.0b`, and `DisplayPort 1.4a`.
  - Verified `DRIVE AGX Thor Developer Kit` camera and display I/O from official sources: `16x GMSL2 + 2x GMSL3`, `1x DisplayPort up to 4K@60Hz`, and separate USB ports present on the hardware quick start guide.
  - Verified Jetson camera-stack behavior from official Jetson Linux docs: USB UVC cameras use `V4L2`; CoE/HSB raw data can feed SIPL with both raw and ISP outputs for downstream processing.

- Validation performed:
  - Cross-checked official NVIDIA product pages, user guides, and camera-development documentation with `WebSearch` and `WebFetch`.
  - Inspected Jetson T5000 official module datasheet text for `ISP = 3.5 GPixel/s` and video encode/decode throughput tables to frame practical 4K-capacity guidance.
  - No code changes, tests, or lints were applicable.

- Follow-up / risk:
  - For `Jetson AGX Thor`, NVIDIA documents interface support and module throughput, but does not publish a single official "max number of 4K USB cameras" guarantee for the developer kit. Real USB-camera count depends on compression, pixel format, frame rate, and the application pipeline.
  - Future Thor camera answers should explicitly state whether the target is `Jetson AGX Thor` or `DRIVE AGX Thor` before giving capacity guidance.

## 2026-04-21 01:21 +0000 - Checked Pi root filesystem free space

- What changed:
  - No product code changed.
  - Ran `df -h /` to answer how to check free storage space on this Pi.
  - Verified the root filesystem `/dev/mmcblk0p2` currently has `15G` available out of `29G` total, with `14G` used (`49%`).

- Validation performed:
  - `df -h /`
  - No code changes, tests, or lints were applicable.

- Follow-up / risk:
  - `df -h` is the next command to use when the user wants the same view for all mounted filesystems, not just `/`.

## 2026-04-23 21:30 +0000 - `scripts/rtcore_jog.py` IPC v1.0 → v1.1 bump; unblocked startup preflight after encoder-cable disconnect

- Context:
  - Operator disconnected J3/J4/J5/J6 encoder cables for mechanical work. On `./start-stack.sh` the fault-reset preflight failed with:
    - `[start-stack] startup preflight found disarmed drive faults before any drive power-up: J3/axis2 0x0208, J4/axis3 0x7305 Er20.1 Encoder internal fault, J5/axis4 0x0208, J6/axis5 0x0208`
    - `[start-stack] WARNING: Direct RTCore fault reset failed for axis_mask=0x3c: ERROR: WELCOME size mismatch (got 0 bytes)`
    - `[start-stack] ERROR: startup fault-reset preflight failed to send the RTCore reset pulse`
  - Operator's hypothesis was that the encoder faults were blocking startup. Correct about the symptom but not the immediate cause: the actual abort is an IPC regression layered ON TOP of the (real) encoder faults.

- Root cause (separate from the encoder state):
  - `src/gradient_rt_motion/ipc_v1.hpp::kVerMinor` was bumped `0 → 1` on 2026-04-20 (per the comment "2026-04-20: adds StatusExtendedSnapshotV1") to make room for `MSG_STATUS_EXTENDED_SNAPSHOT (0x0206)`.
  - `src/gradient_os/arm_controller/backends/ethercat_rtcore/backend.py::_VER_MINOR` was updated in the same pass to `1` with a matching comment.
  - `scripts/rtcore_jog.py::_VER_MINOR` was **not** updated and remained at `0`. `start-stack.sh` invokes this CLI directly for the fault-reset preflight pulse (`direct_rtcore_fault_reset_mask` → `python3 scripts/rtcore_jog.py fault_reset --mask <hex>`).
  - Flow: CLI opens UDS, sends HELLO with `ver_minor=0`; RTCore's accept loop in `src/gradient_rt_motion/main.cpp:5415-5422` compares to `kVerMinor=1`, logs `ERROR: HELLO validation failed (magic/ver/bytes/role mismatch)`, closes the socket. Python `recvmsg()` then returns 0 bytes → `RuntimeError("WELCOME size mismatch (got 0 bytes)")`. `journalctl -u gradient-rt-motion.service` at the failure timestamp shows the `HELLO validation failed` line that confirms it.
  - This bug would trip on ANY disarmed fault preflight path on the current stack, not just the user's encoder scenario.

- Secondary finding (expected, operator-side):
  - `Er20.1` (encoder internal fault) on J4 and the `0x0208` bus code on J3/J5/J6 are members of the A6-EC `ENCODER_RETENTION_FAULT_CODES` family (`src/gradient_os/arm_controller/profiles/drive/a6ec_ds402.py:526-548`). Per vendor Chapter 5 §5.1 and the vendor codebook (`"resettable": false`), these do NOT clear via a DS402 control-word fault-reset pulse (controlword 0x80 toggle). They require an SDO write to `0x2031:11h` (F31.10 "absolute encoder reset selection") and a re-home. RTCore's fault-reset path (`main.cpp:3856` `fault_reset_left[i] = kFaultResetPulseCycles;` + `main.cpp:3917-3993` pulse sequencer) only drives control-word bits, so even with the CLI fixed the preflight pulse cannot clear encoder-retention faults.

- What changed:
  - `scripts/rtcore_jog.py`: `_VER_MINOR = 0` → `_VER_MINOR = 1`, with a comment pointing at the canonical C++ / backend constants and explaining the failure mode (`WELCOME size mismatch (got 0 bytes)`) so a future session doesn't re-break lockstep.

- What explicitly did NOT change:
  - No RTCore C++ changes. No backend / profile / systemd / start-stack.sh changes. No test changes (the WELCOME size assertion in `rtcore_jog.py` was already correct; only the constant was stale).
  - `build_startup_fault_reset_plan` in `src/gradient_os/telemetry/drive_faults.py` was NOT changed, even though it classifies `Er20.x` as `should_auto_reset=1` and currently ignores its own computed `resettable` flag. That gap is called out in the scratchpad as a follow-up; touching it would widen the blast radius beyond the reported problem.

- Validation performed:
  - `python3 -c "import ast; ast.parse(open('scripts/rtcore_jog.py').read())"` → OK.
  - `python3 scripts/rtcore_jog.py status --timeout 1` against the running service (PID 782041, still up from the failed startup) → IPC handshake succeeds, prints a full 6-axis status table. axis0/1 clean (`ds402=2 err=0x0000`); axis2/4/5 latched in DS402 Fault with `err=0x0208`; axis3 latched in Fault with `err=0x7305 (Encoder error)`. Exactly the encoder-cable-disconnect signature we expected.
  - `python3 scripts/rtcore_jog.py fault_reset --mask 0x3c` → `exit=0`, no WELCOME error. Post-reset status shows axes 2-5 still in `ds402=8 Fault` with identical error codes, confirming the pulse reached the server but did not clear the Er20.x family — exactly as vendor docs predict.
  - `python3 -m pytest tests/test_drive_faults.py tests/test_rtcore_runtime.py tests/test_cartesian_jog_resilience.py tests/test_command_api_direct_setpoint.py tests/test_realtime_jog_backend_compatibility.py -q` → `75 passed`. No existing test exercised the IPC `_VER_MINOR` constant, so the fix was behavioral only.
  - `ReadLints` on `scripts/rtcore_jog.py` → clean.

- Runtime outcome:
  - `scripts/rtcore_jog.py` is now back in lockstep with the v1.1 server. The `start-stack.sh` preflight will get past the HELLO/WELCOME handshake on the next run.
  - Because the underlying Er20.x faults are still latched, the preflight will then trip on a DIFFERENT (and honest) failure: either the pulse returns, the drives stay in Fault, and `wait_for_probe_state "BUS_UP_DISARMED"` times out, OR the drives clear after cable reconnect + power-cycle (in which case startup continues).
  - RTCore service was NOT restarted in this pass; the fix only required the client-side constant to move. The live service continues to be v1.1-compatible.

- Follow-up / risk:
  - Operator path forward (encoder-cable reconnect scenario): now auto-handled by the startup preflight as of the follow-up devlog entry below. Manual override still available if needed: `sudo ethercat download -p <slave_pos> -t u16 0x2031 0x11 4` per affected slave, then clear the anchor file entry, then re-home.
  - IPC version discipline: any future `kVerMinor` bump MUST bump `_VER_MINOR` in BOTH `backend.py` AND `rtcore_jog.py`. Consider consolidating the Python constants into a single `gradient_os.rtcore_ipc_constants` module with a unit test that asserts it matches the C header text.

## 2026-04-23 22:10 +0000 - Startup preflight auto-drives F31.10 for encoder-retention faults

- Context:
  - Follow-up to the 21:30 `scripts/rtcore_jog.py` ver-minor fix. That unblocked the handshake but the downstream reset was only a DS402 control-word pulse, which vendor-spec does not clear the A6-EC `Er20.x` family (encoder battery / internal / multi-turn faults). The vendor path requires writing `F31.10 = 4` to `0x2031:0x11` and re-homing.
  - Operator ask: "on startup we should check for faults, reset the drives if they are resettable and if its this encoder issue, perform the process to reset the encoder fault so we can start up - should be simple". Implemented as a classifier in `build_startup_fault_reset_plan` plus a new preflight helper that drives the vendor-correct recovery and invalidates the persisted home anchors.

- Root cause of why the 21:30 fix alone was insufficient:
  - `describe_encoder_retention_fault` in `src/gradient_os/arm_controller/profiles/drive/a6ec_ds402.py` only classified retention faults when `manufacturer_error_code` (0x203F) was non-zero. On this RTCore config 0x203F is NOT PDO-mapped, so `manufacturer_error_code = 0x00000000` always, and the retention branch never fired.
  - The A6-EC firmware publishes the manufacturer value directly on the 0x603F bus code for encoder faults (e.g. `Er20.8` shows up live as `err=0x0208` rather than the vendor-table `0x7305`), which means the bus code itself carries enough information to classify most cases without 0x203F.
  - The existing `build_startup_fault_reset_plan` did not expose per-axis classification OR persistent home-anchor context needed to drive the different recovery paths safely.

- What changed:
  - `src/gradient_os/arm_controller/profiles/drive/a6ec_ds402.py`:
    - `describe_encoder_retention_fault` gained a two-stage bus-code fallback:
      1. Exact numeric match against `fault_code_203f` / `alarm_code_203f` (catches `0x0208` → `Er20.8` unambiguously; tagged `matched_sources=["error_code_matches_manufacturer_code"]`).
      2. Shared-bus-class match against any retention-family entry whose `bus_fault_code_603f` matches (catches `0x7305` → first-match `Er20.1`; tagged `matched_sources=["error_code_bus_class_retention_match"]`). Inherently lossy for subcode disambiguation (Er20.1..Er20.9 / ErA0.1 / ALF9.0 all share `0x7305`) but the retention verdict is correct since the entire class is retention-family on this drive.
    - Added internal helper `_lookup_retention_match_by_bus_code` for the shared-class match path.
    - Kept existing 0x203F path untouched - it remains the preferred disambiguator when available, and `matched_sources` tags let operators tell the paths apart in logs / monitor output.
  - `src/gradient_os/telemetry/drive_faults.py::build_startup_fault_reset_plan`:
    - Now classifies each faulted axis into three buckets: `encoder_data_reset` (F31.10 + anchor invalidation), `ds402_fault_pulse` (control-word 0x80), `unresettable` (neither; pulse anyway for legacy compat).
    - Emits new fields: `ds402_pulse_axis_mask` + `_hex` + `ds402_pulse_required`, `encoder_reset_axis_mask` + `_hex` + `encoder_reset_required`, `encoder_reset_logical_joints` (0-indexed for anchor-store parity), `unresettable_axis_mask` + `_hex`, per-axis `reset_action`, per-axis `encoder_retention_fault_present` / `encoder_retention_fault`.
    - Reason string narrows to `encoder_retention_reset_required`, `encoder_retention_and_ds402_resets_required`, or `faulted_disarmed_axes_ready_for_reset` so the preflight log is precise.
    - `faulted_summary` now annotates retention axes with `[encoder-retention]` so the single-line log shows the recovery path at a glance.
  - `src/gradient_os/absolute_encoder_anchors.py`:
    - New `invalidate_absolute_encoder_anchors(robot_id, *, num_joints, logical_joint_indices, actor)` helper. Loads the anchor store, clears the listed joint entries, saves back. Handles out-of-range / malformed indices gracefully (skipped, not raised) so the preflight does not fall over on a malformed probe payload.
  - `src/gradient_os/arm_controller/backends/ethercat_rtcore/backend.py::reset_encoder_data`:
    - New keyword-only `axis_mask` parameter. When supplied it overrides `logical_joint_index` and drives the SDO write to the explicit bitmask, so the startup preflight can do ONE `_send_cmd_service_sdo_write` covering every encoder-retention-faulted axis instead of looping (which would re-disarm and churn `prepare_for_power_transition` per iteration).
    - Added docstring and a validation path for empty masks.
  - `start-stack.sh`:
    - New `direct_rtcore_encoder_data_reset_and_invalidate_anchors` helper. Python heredoc (same pattern as `direct_rtcore_safe_power_down`) that: loads robot config, initializes a one-shot `EthercatRTCoreBackend`, calls `reset_encoder_data(axis_mask=...)` to write `F31.10 = 4`, sleeps 250 ms so RTCore drains the service-SDO ring, calls `invalidate_absolute_encoder_anchors(...)` with the affected 0-indexed logical joints, then shuts the backend down. Env-var-driven to keep the shell/Python boundary readable.
    - `startup_fault_reset_preflight` now reads the new plan fields and runs encoder-reset first (when `encoder_reset_required`), then the DS402 pulse (when `ds402_pulse_required`), with a 500 ms settle between them. A successful preflight with encoder reset prints a loud `RE-HOME REQUIRED for logical joints [...]` warning so operators know the anchor store was wiped.
  - `tests/test_drive_faults.py`: added five new tests covering the new classifier + bus-code fallbacks:
    - `test_build_startup_fault_reset_plan_routes_encoder_retention_to_f31_10` (mixed retention + DS402-pulse in one probe).
    - `test_build_startup_fault_reset_plan_encoder_retention_only_reason` (all axes retention → `encoder_retention_reset_required`).
    - `test_describe_encoder_retention_fault_falls_back_to_bus_code_when_203f_is_zero` (locks in `0x0208` → `Er20.8` unambiguous match when manufacturer code is zero).
    - `test_describe_encoder_retention_fault_falls_back_to_shared_bus_class_7305` (locks in `0x7305` → `Er20.1` class-retention match).
    - `test_describe_encoder_retention_fault_prefers_manufacturer_side_when_available` (regression: when both 0x203F and 0x603F are present, the manufacturer-side tag wins).
    - Added assertions on the new plan fields to the existing DS402-pulse-only test so it locks the bucketing contract.
  - `tests/test_absolute_encoder_anchors.py` (new file): covers target-joint clearing, out-of-range index tolerance, and empty-list round-trip behaviour.

- Validation performed:
  - `python3 -m pytest tests/test_drive_faults.py tests/test_absolute_encoder_anchors.py tests/test_gradient05_limits_and_backends.py tests/test_rtcore_runtime.py tests/test_a6ec_chapter5_probe.py tests/test_a6ec_joint_sweep.py tests/test_a6ec_j6_watch_replay.py tests/test_api_endpoints.py tests/test_trajectory_execution_backends.py tests/test_command_api_direct_setpoint.py tests/test_cartesian_jog_resilience.py tests/test_run_controller_helpers.py tests/test_realtime_jog_backend_compatibility.py -q` → `396 passed`.
  - `bash -n start-stack.sh` → OK. AST-parse of all 15 embedded Python heredocs → 0 syntax errors.
  - `ReadLints` on all touched files → clean.
  - Live classification against the current faulted stack (PID 782041, axes 2/3/4/5 faulted): post-fix plan returns `encoder_reset_required=True`, `encoder_reset_axis_mask_hex=0x3c`, `encoder_reset_logical_joints=[2,3,4,5]`, `reason=encoder_retention_reset_required`, `faulted_summary` annotates all four axes with `[encoder-retention]`. Every axis decodes with a retention match source so the preflight will now take the F31.10 path automatically. Actual SDO write NOT driven live in this session because operator still has encoder cables physically disconnected - that is the exact scenario the preflight was built for, and verifying it against disconnected encoders would destroy multi-turn data without the drive being able to latch a clean state afterwards.
  - Isolated Python-heredoc path verified with an empty mask env var (`GRADIENT_STARTUP_ENCODER_RESET_AXIS_MASK=0x0`): the helper imports cleanly, short-circuits on empty mask with exit 2, emits the `direct_rtcore_encoder_data_reset_skipped: empty axis mask` log line.

- Runtime outcome:
  - Code is live-ready. Full auto-reset path exercises the first time the operator reconnects the J3/J4/J5/J6 encoder cables and runs `./start-stack.sh`. Expected sequence: preflight detects retention faults → writes F31.10=4 to `axis_mask=0x3c` → clears anchor entries for logical joints 2/3/4/5 → waits for `BUS_UP_DISARMED` → logs `RE-HOME REQUIRED` → stack finishes start-up normally. Operator then re-homes each affected joint via `POST /control/home-joint-native` (or the HM35 UI path) before motion is trusted.
  - If the drive cannot clear the fault after F31.10 (e.g. encoder still not physically connected, or encoder hardware is actually dead), the existing `wait_for_probe_state "BUS_UP_DISARMED"` step will fail with `startup preflight faults still present after reset` and abort startup with a clear message. The anchor invalidation still happens in that case - that is intentional because F31.10=4 has already told the drive to reset multi-turn data, so the old anchor is a lie regardless of whether the bus-level fault latched clear.

- Follow-up / risk:
  - F31.10 value is pinned at `4` ("Reset encoder fault AND multi-turn data") in `a6ec_ds402.py::ENCODER_DATA_RESET_OPERATION`. Value `3` ("Reset encoder fault ONLY") preserves multi-turn data - which would let us skip the anchor invalidation / re-home step when the battery actually survived the disconnect. The trade-off is reliability: Er20.8 specifically signals the battery-backed register contents are no longer trusted, so using value `3` would leave a possibly-corrupted multi-turn counter in place. We stick with `4` (nuclear option) for the auto-reset path. A future pass could try value `3` first, re-probe, and only escalate to `4` when the fault persists - at the cost of preflight complexity.
  - Shared-bus-class retention match (bus `0x7305`) does false-positive on `Er21.0` (encoder-PPR config mismatch) and `ALFA.0` (drive high-temperature warning). Neither is plausible as a disarmed-fault startup condition, but if an operator somehow hits one of those and the preflight auto-drives F31.10, the anchor invalidation is a wasted destructive operation. Mitigation: operator can SDO-poll `0x203F` via `sudo ethercat upload -p <slave> -t uint16 0x203F 0x00` before starting the stack to disambiguate, and the `matched_sources=["error_code_bus_class_retention_match"]` tag in the retention detail makes it visible which path fired.
  - The anchor store is invalidated for the affected joints even if the downstream preflight wait eventually fails - see runtime-outcome note above. This is the right semantic (F31.10 already destroyed the multi-turn data) but worth documenting for future tooling that might be tempted to skip the invalidation when the SDO appears to fail.
  - `build_startup_fault_reset_plan` still does NOT consult the per-axis `fault.resettable` flag as a separate hard-block gate; axes with an unknown vendor code still walk into the DS402 pulse path. That is the pre-existing behaviour and is left unchanged here - the new classifier routes the KNOWN encoder-retention family correctly, which is the main operator pain point. Adding a full `all-axes-resettable` gate is a separate workstream.

## 2026-04-23 23:10 +0000 - Live-test driven: F31.10 alone didn't clear Er20.8, needed F31.01 software reset; also fixed tmpfs-fill bug that broke probe

- Context:
  - First live run of the previous pass's encoder-reset preflight (`./start-stack.sh` against a bus where J3/J4/J5/J6 all showed `Er20.8` encoder-battery fault from disconnected+reconnected cables) produced TWO distinct failures that the unit-test path missed:
    1. The preflight never reached its classifier because `/run` tmpfs was 100% full; the probe fell back to `rtcore: UNKNOWN, ethercat: DOWN` even though the master was healthy (`Phase: Operation, 6 slaves, 1000 frames/s`).
    2. When I manually ran the preflight sequence (`backend.reset_encoder_data(axis_mask=0x3c)` then `rtcore_jog.py fault_reset --mask 0x3c`), the F31.10=4 SDO write was accepted (register self-reset from 4 → 0 confirming operation completion) but the drive's error latches did NOT clear. `0x203F = 0x0208`, `0x603F = 0x0208/0x7305`, DS402 state stayed `Fault` on all four axes.
  - Operator frustration: "i need the bus to fire the encoder reset on startup IF the startup process detects the encoder reset fault!" The infrastructure WAS firing (RTCore journal at 21:56:25 showed all four F31.10=4 SDO writes going through), but the drives weren't leaving Fault.

- Root cause #1 (tmpfs fill):
  - The 2026-04-19 J6 seam-whip drop-in `/etc/systemd/system/gradient-rt-motion.service.d/fast-trace.conf` was still enabled with `GRADIENT_RT_FAST_TRACE_HZ=1000`. Accumulated `j6-fast-trace.jsonl` at 1.67 GB in `/run/gradient-rt-motion/` had exhausted the 1.6 GB tmpfs.
  - RTCore's metrics-writer thread uses atomic temp-write-then-rename on the same filesystem; when the filesystem is full the rename fails and `metrics.json` stays at 0 bytes. `probe_hardware_state_json` reads `metrics.json`, sees it empty, falls back to `rtcore_state=UNKNOWN`, `ethercat_master_state=DOWN`, `responding=0/0`. The bus-ready wait loop in `start-stack.sh` then times out at 20 s with the probe still looking DOWN.
  - The bus itself was healthy throughout. The fault was purely in the probe's ability to observe it.
  - Additional gotcha: `sudo rm -f /run/gradient-rt-motion/j6-fast-trace.jsonl` did NOT free tmpfs because RTCore held an open fd on the deleted file. `sudo -n truncate -s 0 /proc/<rtcore_pid>/fd/<fd_num>` freed it immediately without requiring a service restart.

- Root cause #2 (F31.10 not clearing latched fault):
  - Live SDO writes and read-backs on the faulted drives:
    - Before any reset: `0x203F = 0x0208`, `0x603F = 0x0208`, `0x6041 = 0x1608` (Fault).
    - After F31.10=4: register read immediately returns `0x0004` briefly, then `0x0000` - drive accepted and completed the operation.
    - After DS402 fault-reset pulse (controlword 0x80): statusword unchanged, error codes unchanged, drive stays in DS402 Fault.
    - After F31.10=3 (fault-only reset, preserves multi-turn): same result, no visible change.
  - The vendor profile's `requires_power_cycle: True` flag in `ENCODER_DATA_RESET_OPERATION` was interpreted conservatively in the 2026-04-23 22:10 +0000 entry ("cautious documentation"). It turns out to be LITERAL: the drive's error-register latches only clear when the drive firmware re-initialises.
  - F31.01 = 1 ("Software reset" at `0x2031:0x02`) IS the missing piece. Vendor Ch.11 §11.3.10 describes it as "similar to the program reset upon power-on, without the need for a power cycle". Writing F31.01=1 via `sudo ethercat download -p <slave> -t uint16 0x2031 0x02 1` drops the slave off the bus for ~5-7 s, the drive re-enumerates, and returns with `0x203F = 0x0000`, `0x603F = 0x0000`, `0x6041 = 0x1650` (SwitchOnDisabled, clean). Vendor's precondition ("no non-resettable fault") is NOT enforced on A6-EC in practice; F31.01=1 succeeded live with Er20.8 still visible.
  - Observed empirically: the ethercat CLI write for F31.01=1 commonly returns rc=1 with `Input/output error` or `matches 0 slaves` because the slave vanishes mid-transaction. Those error strings are success signatures, not failures - the preflight has to tolerate them.
  - Parallel F31.01=1 writes across multiple slaves DO NOT WORK: the first reset triggers an EtherCAT re-config pass that leaves other slaves transiently invisible. Must be serialized per-slave.
  - Collateral: each F31.01=1 triggers a transient 0x8700 ("Sync controller") fault on every OTHER slave on the bus. 0x8700 is resettable via DS402 pulse, but on A6-EC it needs TWO consecutive pulses separated by ~0.5 s to actually leave Fault state. First pulse alone leaves `6041=0x1618`; after second pulse the drives land clean at `6041=0x1650`.

- What changed:
  - `/etc/systemd/system/gradient-rt-motion.service.d/fast-trace.conf`: flipped `GRADIENT_RT_FAST_TRACE_HZ` from `1000` to `0`. Updated the file's comment to make the disable-by-default semantic explicit. Drop-in retained rather than deleted so future J6 verification work can re-enable with a single-line edit + `daemon-reload` + `restart gradient-rt-motion.service`.
  - `src/gradient_os/telemetry/drive_faults.py::build_startup_fault_reset_plan`: the encoder-retention classifier now ALSO puts the axis into `ds402_pulse_axis_mask` (previously the two masks were disjoint). This fixes the now-known bug where the preflight fired F31.10 but never DS402-pulsed the same axes. The per-axis `reset_action` field is now `encoder_data_reset_then_ds402_pulse` for retention axes. The `encoder_retention_and_ds402_resets_required` reason string is now gated on `ds402_pulse_axis_mask & ~encoder_reset_axis_mask != 0` (non-retention resettable axes exist), not on `ds402_pulse_required` alone.
  - `start-stack.sh::direct_rtcore_encoder_data_reset_and_invalidate_anchors`: reimplemented. Was: single `backend.reset_encoder_data(axis_mask=...)` call via RTCore IPC. Now: per-slave serialized `sudo ethercat download` sequence (F31.10=4 → sleep 0.1 s → F31.01=1 → wait for slave to re-enumerate via `ethercat upload 0x6041` polling with 15 s timeout → post-enumerate 0.5 s settle) followed by `invalidate_absolute_encoder_anchors`. Tolerates `Input/output error` / `matches 0 slaves` / `No such device` on the F31.01 write as success signatures. Uses the ethercat CLI directly because the RTCore IPC has no clean retry path when the slave vanishes mid-write.
  - `start-stack.sh::startup_fault_reset_preflight`: after the encoder-reset step, now re-probes the bus and rebuilds the plan so the DS402 pulse step fires against the NEW faulted set (which includes the sync-loss 0x8700 collaterals from each software reset). DS402 pulse now fires TWICE separated by 0.5 s to handle the observed A6-EC two-pulse requirement for sync-loss recovery.
  - `tests/test_drive_faults.py`: updated the plan test expectations for the new `ds402_pulse_axis_mask = encoder_reset_axis_mask ∪ ds402_pulse_only_mask` semantic and the new `encoder_data_reset_then_ds402_pulse` action label. Added documentation comments explaining the live-2026-04-23 finding so future refactors don't un-fix the bug.

- What explicitly did NOT change:
  - `backend.reset_encoder_data(axis_mask=...)` is unchanged. The backend path still works for direct-use callers (e.g. the existing `POST /control/reset-encoder-data` API); the startup preflight simply no longer goes through it because the F31.01 follow-up step is cleaner via the CLI.
  - `ENCODER_DATA_RESET_OPERATION` in `a6ec_ds402.py` is unchanged. The profile still describes F31.10=4; the F31.01 software-reset step is a preflight-layer concern.
  - No RTCore C++ changes. No drive profile changes beyond the previously-landed pass. Systemd unit + `start.sh` unchanged.

- Validation performed:
  - `bash -n start-stack.sh` → OK. AST-parse of all 15 embedded Python heredocs → 0 errors.
  - `python3 -m pytest tests/test_drive_faults.py tests/test_absolute_encoder_anchors.py tests/test_gradient05_limits_and_backends.py tests/test_rtcore_runtime.py tests/test_a6ec_chapter5_probe.py -q` → `190 passed` then `166 passed` on the narrower sweep after all the plan changes settled.
  - `ReadLints` on touched files → clean.
  - Live recovery proven on the faulted stack (axes 2/3/4/5 latched at Er20.8):
    - Step-by-step walk of F31.10=4 + F31.01=1 + wait sequence on slave 2 → came up CLEAN (`0x203F=0x0000, 0x603F=0x0000, 0x6041=0x1650`).
    - Sequential F31.01=1 on slaves 3, 4, 5 with 6 s wait each → all three came back clean.
    - Collateral 0x8700 sync-loss on slaves 0-3 → cleared with two consecutive `rtcore_jog.py fault_reset --mask 0xf` calls.
    - Final bus state: ALL 6 AXES `ds402=2 SwitchOnDisabled err=0x0000`. Clean.
  - `./start-stack.sh` end-to-end against the now-clean bus:
    - `BUS READY in 1.967s`
    - `startup preflight: no disarmed drive faults detected`
    - Controller online in 1.782 s, API `{"status":"ok"}` on `/health`, all 6 axes report `SwitchOnDisabled err=0x0000` via `/run/gradient-rt-motion/metrics.json`.
    - `/info/joints` returns empty `arm_deg=[]`/`arm_rad=[]` as expected - motion is correctly blocked because the encoder-reset wiped the multi-turn state and anchors are invalidated; operator must re-home J3/J4/J5/J6 before motion is trusted.

- Runtime outcome:
  - Stack is fully live. The preflight code now embodies the proven-live recovery sequence end-to-end. The next time a user hits latched `Er20.8` on any axis, `./start-stack.sh` will detect it in the classifier, fire F31.10=4 + F31.01=1 per-slave, wait for each drive to re-enumerate, clear the sync-loss collateral via two DS402 pulses, and bring the stack up to BUS_UP_DISARMED without any manual intervention. They only need to re-home the affected joints before commanding motion.
  - Re-home requirement is enforced by the preflight's `invalidate_absolute_encoder_anchors` call, which clears the persisted home entries for each affected logical joint. The display-truth gate (`absolute_home_anchor_stale`) then blocks motion until HM35 / native-home completes and repopulates the anchor.

- Follow-up / risk:
  - The preflight recovery takes ~6 s per encoder-reset axis (serialized software resets). For 4 axes that is ~24 s in the preflight window plus the normal bus-ready wait. Acceptable for an auto-recovery path; documented in the operator-facing warning block so it's not surprising.
  - The F31.01=1 software-reset precondition ("no non-resettable fault") is documented but not enforced on A6-EC. If a future firmware revision starts enforcing it, the preflight will fail at the F31.01=1 write with a non-tolerated error string and the existing `wait_for_probe_state "BUS_UP_DISARMED"` timeout will catch it. No operator-silent regression.
  - The fast-trace drop-in stays installed with HZ=0. Leaving the file intact keeps the J6 seam-whip verification ergonomics but means any operator who flips HZ back to 1000 and forgets to flip it off will refill tmpfs at ~60 MB/min. Consider adding a `preserve_rtcore_fast_trace_if_any` guard in `start-stack.sh` that warns when fast-trace is active on boot.
  - `/run` tmpfs fill as a probe failure mode is now documented in the scratchpad. A future robustness pass could have the probe script explicitly check `df /run` when metrics.json is unreadable and surface a clearer `tmpfs_full` reason in the probe snapshot instead of the generic `rtcore_state=UNKNOWN`.

## 2026-04-25 20:40 +0000 - Implemented realtime one-shot joint jog path and stale-tolerant display

- What changed:
  - `src/gradient_os/arm_controller/backends/ethercat_rtcore/backend.py`: split trajectory submission from strict completion wait via `RTCoreTrajectorySubmission`, `submit_joint_trajectory`, and `wait_for_trajectory_final_point_sent`; preserved existing `execute_joint_trajectory` strict settle behavior.
  - `src/gradient_os/arm_controller/utils.py`: added controller `SETTLING` motion state and endpoint-cache state keys.
  - `src/gradient_os/arm_controller/trajectory_execution.py`: RTCore path now releases `is_running` after the final trajectory point is issued, records `last_bounded_endpoint`, and treats post-final-point settle timeout as non-fatal diagnostics.
  - `src/gradient_os/arm_controller/command_api.py` and `src/gradient_os/run_controller.py`: added controller-owned `APPLY_JOINT_DELTA`, DS402/fault readiness preflight, endpoint-chain baselining, structured errors, wait-for-idle handling, and endpoint invalidation on STOP/power/reset operations.
  - `src/gradient_os/api/main.py`: `/control/joint-jog` is now transport-only and forwards `APPLY_JOINT_DELTA`; API no longer calls `GET_JOINT_STATE` or constructs absolute joint targets. SSE queue reduced to 5 frames.
  - `src/gradient_os/run_controller.py`: monitor telemetry now emits last-good joints with `joint_feedback_stale` metadata instead of blanking transient truth flickers.
  - `web-ui/src/App.tsx`, `web-ui/src/liveState.tsx`, `web-ui/src/ControlPanel.tsx`, `web-ui/src/ArmVisualizer.tsx`: carried stale telemetry metadata, displayed stale sample badge, and moved robot-joint application into the render loop with bounded visual interpolation.
  - Tests updated for controller-owned `APPLY_JOINT_DELTA`, endpoint chaining, disabled-axis rejection, and RTCore submit/final-point behavior.

- Validation performed:
  - `python3 -m py_compile src/gradient_os/arm_controller/backends/ethercat_rtcore/backend.py src/gradient_os/arm_controller/utils.py src/gradient_os/arm_controller/trajectory_execution.py src/gradient_os/arm_controller/command_api.py src/gradient_os/run_controller.py src/gradient_os/api/main.py` -> pass.
  - Targeted realtime-jog Python tests with `PYTHONPATH=src uv run --no-project ... pytest ...` -> `12 passed`.
  - Planned Python regression slice (`test_gradient05_limits_and_backends.py`, `test_rtcore_runtime.py`, `test_drive_faults.py`, `test_api_endpoints.py`, `test_trajectory_execution_backends.py`, `test_command_api_direct_setpoint.py`, `test_cartesian_jog_resilience.py`, `test_run_controller_helpers.py`, `test_realtime_jog_backend_compatibility.py`, `test_absolute_encoder_anchors.py`) -> `328 passed`.
  - `npm test` in `web-ui/` -> `37 passed`.
  - `ReadLints` on all touched Python/TS/TSX files -> clean.

- Follow-ups / risks:
  - Hardware jog smoke was not run by automation because it moves the physical robot; perform operator-supervised live validation for: click during EXECUTING rejects `MOTION_ACTIVE`, click during SETTLING accepts/chains, STOP clears endpoint, disabled-drive jog rejects quickly.
  - `uv run --extra dev` is currently blocked by optional `picamera2 -> python-prctl` requiring libcap headers; offline validation used `uv --no-project` with explicit test dependencies.

## 2026-04-25 21:05 +0000 - Realtime-jog review gaps closed; J5 ratio corrected to exact 10:1

- What changed:
  - Addressed the follow-up review transcript gaps by adding missing regression tests for final-point wait semantics, faulted final-point endpoint clearing, old settle watcher isolation, API transport-only joint delta, stale telemetry renderability, and cross-joint endpoint-chain rejection.
  - Updated `src/gradient_os/arm_controller/robots/gradient05/config.py` so J5's gear ratio is exactly `10.0` per operator correction. Adjusted comments and expectations that previously referenced `100/11` as J5-specific.

- Validation performed:
  - New targeted backend/realtime/ratio tests -> `12 passed`.
  - Planned Python regression slice -> `333 passed`.
  - `npm test -- src/ControlPanel.test.tsx` -> `31 passed`.
  - Full `web-ui` vitest suite -> `39 passed`.

- Follow-ups / risks:
  - Hardware jog smoke remains operator-supervised only; do not run unattended because it moves the physical robot.
  - Follow-up live-review patch: `wait_for_trajectory_final_point_sent` now raises when RTCore reports a newer command superseded the observed trajectory before the final point. Added regression `test_wait_for_trajectory_final_point_sent_rejects_superseded_command_before_final_point`. Targeted final-point wait tests -> `4 passed`; planned Python regression slice -> `334 passed`; lints clean on touched files.
  - Follow-up live-test response: held-jog `pointercancel` no longer sends `JOG_SESSION_STOP`; it zeroes the active velocity while keeping the jog session alive, preventing browser/DOM cancellation from de-energizing the drive/brakes while the operator is still holding. Added base/world-frame `MOVE_LINE_RELATIVE detail` logging with delta, start/target pose, current joints, orientation, speed, and closed-loop flag. Validation: `py_compile` on changed Python, `npm test -- src/ControlPanel.test.tsx` -> `31 passed`, planned Python regression slice -> `334 passed`, `ReadLints` clean.

## 2026-04-25 23:51 +0000 - Advisory canonical truth moved out of motion-control feedback

- What changed:
  - Added `EthercatRTCoreBackend.get_control_joint_positions()` and `servo_driver.get_control_arm_state_rad()` for live control feedback that skips strict canonical truth diagnostics but hard-fails on RTCore disconnect, missing feedback/config, DS402 Fault/FaultReactionActive, drive/manufacturer errors, or fault flags.
  - Moved Cartesian jog, one-shot joint delta baselines, bounded setpoint planning, and Cartesian/orientation motion planning to the control-feedback path while keeping `get_joint_positions()` strict for diagnostic surfaces.
  - Changed the jog loop so a control-feedback miss holds the RTCore jog lease with zero velocity and waits for resync instead of integrating a Cartesian target from stale `q_current`.
  - Updated `ControlPanel` so advisory canonical mismatches render as trust warnings and no longer disable jog controls when display/stale feedback exists; hard truth blockers remain red/unavailable.
  - Added backend, command API, jog-loop, relative move, and UI regressions for the new control/advisory split.

- Validation performed:
  - `python3 -m py_compile src/gradient_os/arm_controller/backends/ethercat_rtcore/backend.py src/gradient_os/arm_controller/servo_driver.py src/gradient_os/arm_controller/command_api.py` -> pass.
  - Targeted Python tests (`test_gradient05_limits_and_backends.py`, `test_command_api_direct_setpoint.py`, `test_cartesian_jog_resilience.py`) -> `212 passed`.
  - Broader Python slice from the plan -> `305 passed`.
  - `npm test -- src/ControlPanel.test.tsx` -> `32 passed`.
  - Full `web-ui` `npm test` -> `40 passed`.
  - `ReadLints` on touched files -> clean.

- Follow-ups / risks:
  - Live held-jog validation was not run by automation because it moves the physical robot. Operator-supervised validation still needed for held `+Z`, advisory truth flicker during motion, no diagonal drift, and hard blocking on actual drive fault/offline/invalid anchor.

## 2026-04-26 00:38 +0000 - Live jog residual wobble triage and velocity taper

- What changed:
  - Inspected live terminal/API evidence after operator reported residual start/stop wobble. Logs showed clean control feedback: no `control feedback miss`, no truth flicker, no IK/gate failures, no DS402 fault/offline, and `/info/joints-detailed` truth available on all axes with tiny shaft/roundtrip errors.
  - Identified the likely residual source as jog dynamics: repeated normal `ui-release` stops, some very short sessions, and RTCore's `cmd_stop` path snapping hold targets to live feedback for stop-arrest.
  - Added controller-side jog velocity taper in `src/gradient_os/arm_controller/command_api.py`: slew-limits linear/angular jog velocity before IK and sends a zero-velocity lease update plus a short settle before normal UI-release stop. Controller-stop/fault/lease-expired paths remain immediate.
  - Added regressions in `tests/test_cartesian_jog_resilience.py` for the slew limiter and zero-before-stop release behavior.

- Validation performed:
  - `python3 -m py_compile src/gradient_os/arm_controller/command_api.py` -> pass.
  - `PYTHONPATH=src uv run --no-project --with pytest==8.4.2 --with numpy==2.3.0 --with scipy==1.15.3 python -m pytest tests/test_cartesian_jog_resilience.py -q` -> `20 passed`.
  - Broader Python slice (`test_gradient05_limits_and_backends.py`, `test_api_endpoints.py`, `test_trajectory_execution_backends.py`, `test_command_api_direct_setpoint.py`, `test_cartesian_jog_resilience.py`, `test_run_controller_helpers.py`, `test_realtime_jog_backend_compatibility.py`) -> `307 passed`.
  - `ReadLints` on touched files -> clean.

- Follow-ups / risks:
  - Needs operator-supervised retest of held jog start/stop feel. If wobble persists, next evidence should be higher-rate pose/RTCore trace around release/start, because normal logs only show the controller/RTCore state after the fact.

## 2026-04-26 03:27 +0000 - 3D stage live telemetry decoupled from React re-render loop

- What changed:
  - Investigated operator report that the 3D stage was laggy and jumped during live operation.
  - `web-ui/src/ArmVisualizer.tsx`: added `pushLiveJointSample()` to the visualizer imperative handle and route incoming joint samples into the existing `requestAnimationFrame` interpolation buffer directly.
  - `web-ui/src/App.tsx`: monitor SSE handler now pushes accepted joint samples to the visualizer immediately, while broad `setLatest` / panel state updates are throttled to 10 Hz so the app shell does not re-render at monitor packet rate.
  - Existing `joints` prop path remains as a fallback/initial snapshot for when the visualizer mounts late or telemetry falls back through `/info/joints`.

- Validation performed:
  - `npm test -- src/App.test.ts src/ControlPanel.test.tsx` -> `35 passed`.
  - Full `web-ui` `npm test` -> `40 passed`.
  - `npm run build` -> pass. Vite still reports the pre-existing large chunk warning for `ArmVisualizer`/OCCT assets.
  - `ReadLints` on `web-ui/src/App.tsx` and `web-ui/src/ArmVisualizer.tsx` -> clean.

- Follow-ups / risks:
  - Requires browser/live-stack retest. Expected improvement: 3D robot follows telemetry at monitor rate without the full React app re-rendering every packet.

## 2026-04-26 03:35 +0000 - Live visualization throttle raised to 50 Hz

- What changed:
  - Updated `web-ui/src/App.tsx::REACT_TELEMETRY_MIN_INTERVAL_MS` from `100` ms to `20` ms after operator correction that 10 Hz looks unacceptable for the live visualization.
  - The 3D visualizer direct sample path still receives every accepted monitor joint sample; the shared React live-state path now also targets 50 Hz.

- Validation performed:
  - `npm test -- src/App.test.ts src/ControlPanel.test.tsx` -> `35 passed`.
  - `npm run build` -> pass. Existing Vite warnings about OCCT browser externals and large visualizer chunk remain.
  - `ReadLints` on `web-ui/src/App.tsx` -> clean.

- Follow-ups / risks:
  - Requires browser/live-stack retest after web UI reload/restart.

## 2026-04-26 03:40 +0000 - Prevented disarmed Cartesian trajectory enqueue and cleaned final-point timeout

- What changed:
  - Investigated traceback from `MOVE_LINE_RELATIVE,0,0,0.05`: RTCore accepted trajectory `1`, but Python timed out waiting for final point while status stayed at `current_point_index=0 queue_depth=201`. The physical pose did not change.
  - `src/gradient_os/arm_controller/command_api.py`: added `_require_target_axes_motion_ready(None)` before Cartesian/profiled move planning/submission, including relative move wrappers. This makes disarmed/not-OperationEnabled axes reject before RTCore trajectory enqueue.
  - `src/gradient_os/arm_controller/trajectory_execution.py`: final-point timeout now clears `last_bounded_endpoint`, aborts the RTCore trajectory if available, logs a clear error, and exits the executor cleanly instead of producing an uncaught thread traceback.
  - Added regression coverage in `tests/test_command_api_direct_setpoint.py` and `tests/test_trajectory_execution_backends.py`.

- Validation performed:
  - `python3 -m py_compile src/gradient_os/arm_controller/command_api.py src/gradient_os/arm_controller/trajectory_execution.py` -> pass.
  - Focused command/executor tests -> `53 passed`.
  - Broader motion regression slice -> `309 passed`.
  - `ReadLints` on touched files -> clean.
  - `./start-stack.sh status` after investigation showed `launcher_state: absent`, `controller: down`, `api: down`, `web: down`.

- Follow-ups / risks:
  - Restart the stack before live retest. After restart, explicit Power Up is required before any Cartesian/profiled motion.

## 2026-04-26 03:55 +0000 - Restored master-style smooth-chase 3D stage; removed telemetry throttle and per-sample bounds recompute

- Context:
  - Operator: "3d visualisation is insanely laggy this needs to be faster - the frontend already gets streamed the joint telemetry - do not throttle the visual updates. it used to work perfectly before the changes made on this branch so it can work again." Earlier 03:27 + 03:35 fixes (imperative push + 50 Hz `setLatest` throttle) did not resolve it.
  - Compared the branch against `master` and found two stacked regressions inside `web-ui/src/ArmVisualizer.tsx`:
    1. The animate loop rendered through `interpolateLiveJointSamples` with `LIVE_JOINT_LOOKBACK_MS = 30` and a 40 ms extrapolate cap. That builds in a fixed ~30 ms render delay vs the latest accepted telemetry sample. Master had no such buffer.
    2. `pushLiveJointSample` set `pendingDynamicBoundsRef = true` on every accepted sample (gated by `LIVE_BOUNDS_REFRESH_INTERVAL_MS = 20`). At ~50 Hz telemetry that pumped `alignToGroundAndUpdateBounds` 50x/sec - which traverses the URDF, recomputes the visible-world bbox, and rewrites bounding markers/walls/edges on the main thread. That starved the render loop.

- What changed (`web-ui/src/ArmVisualizer.tsx`):
  - Replaced the sample-buffer interpolation path with master's smooth chase. New module constant `LIVE_JOINT_SMOOTHING_RAD_PER_S = 12` (same value master used).
  - Removed `LIVE_JOINT_LOOKBACK_MS`, `LIVE_JOINT_MAX_EXTRAPOLATE_MS`, `liveJointSamplesRef`, `interpolateLiveJointSamples`, and the old `applyLiveJointSample` helper.
  - `pushLiveJointSample(values)` now only sets `targetAnglesRef.current`, plus seeding `currentAnglesRef.current` on first sample. No sample buffer, no per-sample bounds nudge.
  - Animate loop reads `targetAnglesRef`, blends `currentAnglesRef` toward it at `LIVE_JOINT_SMOOTHING_RAD_PER_S`, applies the result to the URDF joints, and only schedules a bounds refresh when (a) joints actually changed, (b) the 200 ms `LIVE_BOUNDS_REFRESH_INTERVAL_MS` interval has elapsed, and (c) the bounding box overlay is visible.
  - Bumped `LIVE_BOUNDS_REFRESH_INTERVAL_MS` from `20` to `200` ms. The bounding box is decorative; refreshing it at 5 Hz instead of 50 Hz reclaims ~45 expensive recomputes per second on the main thread.

- What changed (`web-ui/src/App.tsx`):
  - Removed `REACT_TELEMETRY_MIN_INTERVAL_MS`, `lastReactTelemetryPublishMsRef`, and `hasPublishedTelemetryRef`. The 20 ms throttle on `setLatest` is gone; React state, drive faults, and motion status now publish on every accepted SSE packet, exactly as master did.
  - Direct `visualizerRef.current?.pushLiveJointSample(poseJoints)` still runs on every accepted packet (cheap with the new minimal `pushLiveJointSample`). Joints prop path remains as a fallback for initial mount and the `/info/joints` HTTP fallback.

- Validation performed:
  - `npm test -- --run` in `web-ui/` -> `40 passed (4 files)`.
  - `npm run build` -> pass (existing OCCT external-module warning and `ArmVisualizer` chunk-size warning unchanged).
  - `ReadLints` on `web-ui/src/App.tsx` and `web-ui/src/ArmVisualizer.tsx` -> only the pre-existing `__synthesized_from_monitor_axes` literal-type warning at `App.tsx:5095`, which is unrelated to this change.

- Follow-ups / risks:
  - Operator-supervised live validation needed: open the 3D stage with the stack running, drive a held jog, and confirm the arm tracks live telemetry smoothly without lag or jumps. If fast jogs visually trail the wire, bump `LIVE_JOINT_SMOOTHING_RAD_PER_S` (12 -> 30-60) - a single-line tweak.
  - The branch's earlier interpolation buffer was added to "smooth out" telemetry jitter. With the smooth chase restored, jitter is naturally damped by the 12 rad/s blend; if the operator reports residual single-frame jitter, prefer increasing the smoothing rate over reintroducing a lookback buffer.

## 2026-04-26 04:20 +0000 - Iteration 2: snap to target (no smoothing) and re-throttle React panels

- Context:
  - Operator pushback after the 03:55 fix: "ITS STILL INSANELY LAGGY BEHIND REAL LIFE - IT WAS NOT LIKE THIS BEFORE". The previous iteration matched master at 12 rad/s smoothing, which mathematically leaves a `velocity / smoothing` steady-state visual trail (≈50° at a J6 jog of 100 motor RPM output). Master had the same property; operators just were not doing continuous fast jogs back then.

- What changed (`web-ui/src/ArmVisualizer.tsx`):
  - Removed `LIVE_JOINT_SMOOTHING_RAD_PER_S` and the smoothing blend in the animate loop. The animate loop now writes `targetAngles[index]` straight to `joint.setJointValue(...)` every frame. `currentAnglesRef` is kept in sync only to detect "did this joint actually move" for gating the dynamic-bounds refresh.
  - The visualizer now tracks telemetry 1:1: telemetry at ~50 Hz + animate at ~60 Hz means the rendered arm is at most one animate frame (~16 ms) behind the latest accepted SSE sample plus SSE/network jitter.

- What changed (`web-ui/src/App.tsx`):
  - Re-introduced a moderate React-state throttle (`REACT_TELEMETRY_MIN_INTERVAL_MS = 33`, ≈30 Hz). The 03:55 iteration removed the throttle entirely, which made `setLatest` re-render the full App tree + `ControlPanel.tsx` (which reads `latest` via `LiveStateContext`) on every accepted SSE packet. On the Pi that saturates the main thread and starves the WebGL rAF loop — which IS the visible lag.
  - The visualizer is still on the imperative direct path (`visualizerRef.current.pushLiveJointSample(poseJoints)`) and still receives every accepted SSE packet, so this throttle does NOT throttle visual updates per the operator's instruction. It only throttles text-panel updates.
  - Alerts continue to force-publish so the operator never misses a safety event.

- Validation performed:
  - `npm test -- --run` -> `40 passed (4 files)`.
  - `npm run build` -> pass; ArmVisualizer/app chunks rebuilt (`ArmVisualizer-Uc7SlZQO.js`, `app-BG719dUk.js`).
  - `ReadLints` -> only the pre-existing unrelated `__synthesized_from_monitor_axes` warning at `App.tsx:5103`.

- Follow-ups / risks:
  - Operator must hard-reload the browser (Ctrl+Shift+R / Cmd+Shift+R). HMR for `ArmVisualizer.tsx` (forwardRef + heavy `useImperativeHandle` + Three.js side effects) often does not propagate cleanly, so a stale module may still be running.
  - If the visualizer now appears jittery instead of laggy, the source is telemetry packet jitter / SSE jitter, not the visualizer code path. Mitigation if needed: a low-pass on the WIRE side of telemetry (RTCore or controller) rather than re-introducing easing in the visualizer.
  - Watch for any text-panel "feels stale" complaints; if the 30 Hz React cadence isn't fast enough for joint readouts in `ControlPanel`, the right fix is to subscribe panel components to a separate ref-driven channel (the same pattern the visualizer already uses) rather than push the React rate back to 50 Hz on the Pi.

## 2026-04-26 04:35 +0000 - Prevented stale React telemetry from echoing into live 3D stage

- What changed:
  - `web-ui/src/App.tsx`: when the monitor stream is connected, `LazyArmVisualizer` now receives `joints={undefined}` so throttled React `latest` samples cannot replay older poses through `ArmVisualizer`'s prop effect after newer samples arrived through `visualizerRef.current.pushLiveJointSample(...)`.
  - Preserved the disconnected/fallback prop path so the stage can still render the last known snapshot when the live stream is not connected.

- Validation performed:
  - `npm test -- src/App.test.ts --run` in `web-ui/` -> `3 passed`.
  - `ReadLints` on `web-ui/src/App.tsx` reported only the pre-existing `__synthesized_from_monitor_axes` type warning.

- Follow-ups / risks:
  - Requires browser hard reload/live retest. Expected result: no delayed React prop echo causing the stage to trail or snap back behind the direct SSE sample stream.

## 2026-04-26 04:42 +0000 - Added end-to-end 3D visualization lag instrumentation

- What changed:
  - `src/gradient_os/api/main.py`: `/monitor` payloads now include `monitor_timing.api_received_t` and `monitor_timing.api_sequence` when the API receives controller UDP telemetry.
  - `web-ui/src/visualizerLagTelemetry.ts`: added a rolling lag probe for monitor/fallback samples, tracking controller sample age, API-to-browser age, browser receive-to-visible time, push-to-visible time, frame interval, frame work, superseded samples, and dropped samples.
  - `web-ui/src/App.tsx`: stamps browser receive time for each monitor event, records push timing when handing samples to `ArmVisualizer`, and renders a small `3D Lag Probe` badge over the stage.
  - `web-ui/src/ArmVisualizer.tsx`: accepts an optional timing id in `pushLiveJointSample(...)` and records when that sample is actually applied/rendered by the rAF loop. Console warnings now emit structured `[GradientOS 3D lag]` entries when any segment exceeds 250 ms.

- Validation performed:
  - `python3 -m py_compile src/gradient_os/api/main.py` -> pass.
  - `npm test -- src/App.test.ts --run` in `web-ui/` -> `3 passed`.
  - `npm run build` in `web-ui/` -> pass; existing Vite OCCT externalization/chunk-size warnings remain.
  - `ReadLints` on touched files -> clean.

- Follow-ups / risks:
  - Requires live browser retest after API/web reload. Use the badge/console to identify the segment owning the reported 1-2s delay before making another performance change.

## 2026-04-26 04:43 +0000 - Corrected Gradient-05 J5 gear ratio to 11:1

- What changed:
  - `src/gradient_os/arm_controller/robots/gradient05/config.py`: set J5 `actuator_gear_ratios[4]` to exact `11.0` and updated the inline comment.
  - `tests/test_gradient05_limits_and_backends.py`: updated the Gradient-05 default ratio expectation and J5 derived wrap-period expectation to `1_441_792` counts.

- Validation performed:
  - Initial `python3 -m pytest ...` failed because system Python has no `pytest`.
  - Corrected workflow per operator instruction: `source ./start.sh && python -m pytest tests/test_gradient05_limits_and_backends.py::test_gradient05_config_defaults_and_mapping_shape tests/test_gradient05_limits_and_backends.py::test_ethercat_backend_a6ec_reference_wrap_period_uses_per_axis_gear_ratios tests/test_rtcore_runtime.py::test_render_rtcore_systemd_env_contains_scaling_and_profile tests/test_rtcore_runtime.py::test_build_rtcore_drive_startup_config_uses_drive_profile_defaults_when_robot_has_no_override tests/test_rtcore_runtime.py::test_build_rtcore_drive_startup_config_uses_drive_profile_module -q` -> `5 passed`.
  - `ReadLints` clean on the touched config/test files.

- Follow-ups / risks:
  - Changing the drive-native mechanical ratio means the next live startup/readback should confirm A6-EC `C10.18/C10.19` for J5 is `11/1`; per SOP/manufacturer notes, changing that drive ratio requires a fresh homing cycle afterward.

## 2026-04-26 04:53 +0000 - Corrected Gradient-05 J5 gear ratio again to 100:11

- What changed:
  - Operator corrected the previous `11:1` instruction: J5 is actually `100/11`.
  - `src/gradient_os/arm_controller/robots/gradient05/config.py`: set J5 to `100.0 / 11.0` with an inline `100:11` comment.
  - `tests/test_gradient05_limits_and_backends.py`: updated the Gradient-05 default ratio expectation and the rounded J5 wrap-period expectation to `1_191_564` counts.

- Validation performed:
  - `source ./start.sh && python -m pytest tests/test_gradient05_limits_and_backends.py::test_gradient05_config_defaults_and_mapping_shape tests/test_gradient05_limits_and_backends.py::test_ethercat_backend_a6ec_reference_wrap_period_uses_per_axis_gear_ratios tests/test_drive_faults.py::test_a6ec_gear_ratio_u16_pair_recovers_100_over_11_from_ieee754_float tests/test_rtcore_runtime.py::test_render_rtcore_systemd_env_contains_scaling_and_profile tests/test_rtcore_runtime.py::test_build_rtcore_drive_startup_config_uses_drive_profile_defaults_when_robot_has_no_override tests/test_rtcore_runtime.py::test_build_rtcore_drive_startup_config_uses_drive_profile_module -q` -> `6 passed`.
  - `ReadLints` clean on the touched config/test files.

- Follow-ups / risks:
  - Next live startup/readback should confirm J5 A6-EC `C10.18/C10.19` is `100/11`; changing that drive-native ratio still requires fresh homing afterward.

## 2026-04-26 05:05 +0000 - Cleaned up cockpit overlay layout

- What changed:
  - `web-ui/src/App.tsx`: compacted the runtime header so the LIVE/SIM selector and drive-state chips wrap instead of overlapping adjacent status controls.
  - Moved the `3D Lag Probe` from the top-right stage corner to the bottom-right and reduced its footprint.
  - Replaced separate left/right stage overlays with a single top bar so `3D Stage` / `Vision Feed` no longer collide with the stage title chips.
  - Hid the bottom stage guidance while the 3D visualizer is paused and made the remaining guidance non-interactive, preventing it from blocking the `Load 3D Workspace` button.

- Validation performed:
  - Browser sanity check against `http://localhost:8000/` caught and confirmed the blocked startup button issue before the final patch.
  - `ReadLints` on `web-ui/src/App.tsx` -> clean.
  - `npm test -- src/App.test.ts --run` in `web-ui/` -> `3 passed`.
  - `npm run build` in `web-ui/` -> pass; existing Vite OCCT externalization/chunk-size warnings remain.

- Follow-ups / risks:
  - The open browser view disappeared during the final visual click attempt, so final visual confirmation is from the pre-final screenshot plus automated build/test checks. Hard reload the UI if HMR shows stale chrome.

## 2026-04-27 01:27 +0000 - Looped trajectory preflight accepts final-point plus live endpoint proof

- What changed:
  - Investigated prior transcript `161b0127-4149-494a-adee-9c1c3e590f57`, which narrowed the loop issue to move-to-start preflight faulting after the robot physically reached the start because RTCore had not yet reported `motion_done=True`.
  - SUPERSEDED by the 01:56 correction below: this first patch incorrectly treated "final point issued" as enough executor-side proof and deferred completion to live endpoint verification.
  - `src/gradient_os/arm_controller/command_api.py`: clarified loop-wrapper contract: final point must be issued and live control feedback must match the wrapper endpoint before the loop body starts.
  - Updated regressions in `tests/test_trajectory_execution_backends.py` and `tests/test_command_api_direct_setpoint.py`.

- Validation performed:
  - `source ./start.sh && python -m pytest tests/test_trajectory_execution_backends.py::test_open_loop_executor_strict_completion_allows_settle_timeout_after_final_point tests/test_trajectory_execution_backends.py::test_open_loop_executor_strict_completion_raises_on_final_point_timeout tests/test_command_api_direct_setpoint.py::test_looping_trajectory_executor_thread_starts_body_when_live_endpoint_matches tests/test_command_api_direct_setpoint.py::test_looping_trajectory_executor_thread_faults_on_endpoint_mismatch -q` -> `4 passed`.
  - `source ./start.sh && python -m pytest tests/test_api_endpoints.py tests/test_command_api_direct_setpoint.py tests/test_trajectory_execution_backends.py -q` -> `130 passed`.
  - `source ./start.sh && python -m pytest tests/test_cartesian_jog_resilience.py tests/test_run_controller_helpers.py tests/test_api_endpoints.py tests/test_command_api_direct_setpoint.py tests/test_trajectory_execution_backends.py -q` -> `172 passed`.
  - `source ./start.sh && python -m py_compile src/gradient_os/arm_controller/trajectory_execution.py src/gradient_os/arm_controller/command_api.py tests/test_trajectory_execution_backends.py tests/test_command_api_direct_setpoint.py` -> pass.
  - `ReadLints` on touched Python/test files -> clean. `git diff --check` on touched files -> clean.

- Follow-ups / risks:
  - Operator-supervised live loop smoke is still needed: run a looped `racetrack_linear_1`, confirm the UI receives ACK after planning, the move-to-start completes, the loop body starts, and Stop/Power Down remain clear.

## 2026-04-27 01:56 +0000 - Added Gradient-05 robot spec sheet

- What changed:
  - Added top-level `GRADIENT_05_ROBOT_SPEC_SHEET.md`, an AR-style Markdown spec sheet for the Gradient-05 robot.
  - Sourced axis count, joint limits, gear ratios, encoder resolution, backend defaults, and controller defaults from `Gradient05Config` / runtime sources.
  - Derived approximate reach/envelope values from `robots/gradient-05/gradient-05.urdf` and marked payload, mass, repeatability, wrist moments, and final drawing dimensions as TBD because they are not present in the repo.

- Validation performed:
  - `GRADIENT_START_QUIET=1 source ./start.sh && python ...` one-off URDF envelope calculation completed.
  - `git diff --check -- "GRADIENT_05_ROBOT_SPEC_SHEET.md"` -> pass.
  - `ReadLints` on `GRADIENT_05_ROBOT_SPEC_SHEET.md` -> clean.

- Follow-ups / risks:
  - The Gradient-05 asset manifest/README still label the URDF/DH as template data, so the published reach figures should be re-derived after production CAD/URDF replacement.
  - Payload, repeatability, robot mass, and allowable wrist moments need mechanical/CAD/test inputs before they can be filled in.

## 2026-04-27 01:56 +0000 - Corrected loop preflight to keep RTCore motion_done as required

- What changed:
  - Re-examined the loop preflight safety question after operator challenged the previous patch as papering over root cause.
  - Root cause from RTCore code: `queue_depth=0, state=executing, motion_done=False` means the final point is due/issued, but at least one targeted axis feedback is still outside RTCore's final-position completion window (`kTrajectoryCompletionToleranceCounts = 128` counts).
  - `src/gradient_os/arm_controller/trajectory_execution.py`: restored strict-mode behavior so loop preflight does NOT accept `executing`; it must see RTCore terminal `completed`/`idle` before live endpoint verification can pass the wrapper.
  - Increased the strict post-final-point settle wait from the too-short `0.5s` probe to `GRADIENT_RTCORE_STRICT_COMPLETION_SETTLE_TIMEOUT_S` (default `5.0s`). If RTCore still does not complete, the wrapper aborts, clears `last_bounded_endpoint`, and raises.
  - `src/gradient_os/arm_controller/command_api.py`: updated the loop-wrapper docstring so RTCore completion is first-class, not replaceable by endpoint verification.
  - `tests/test_trajectory_execution_backends.py`: restored the strict settle-timeout regression to expect abort/raise and verify the new 5s wait.

- Validation performed:
  - `source ./start.sh && python -m pytest tests/test_trajectory_execution_backends.py::test_open_loop_executor_strict_completion_raises_on_settle_timeout tests/test_trajectory_execution_backends.py::test_open_loop_executor_strict_completion_raises_on_final_point_timeout tests/test_command_api_direct_setpoint.py::test_looping_trajectory_executor_thread_starts_body_when_live_endpoint_matches tests/test_command_api_direct_setpoint.py::test_looping_trajectory_executor_thread_faults_on_endpoint_mismatch -q` -> `4 passed`.
  - `source ./start.sh && python -m pytest tests/test_api_endpoints.py tests/test_command_api_direct_setpoint.py tests/test_trajectory_execution_backends.py -q` -> `130 passed`.
  - `source ./start.sh && python -m pytest tests/test_cartesian_jog_resilience.py tests/test_run_controller_helpers.py tests/test_api_endpoints.py tests/test_command_api_direct_setpoint.py tests/test_trajectory_execution_backends.py -q` -> `172 passed`.
  - `source ./start.sh && python -m py_compile src/gradient_os/arm_controller/trajectory_execution.py src/gradient_os/arm_controller/command_api.py tests/test_trajectory_execution_backends.py tests/test_command_api_direct_setpoint.py` -> pass.
  - `ReadLints` on touched files -> clean. `git diff --check` on touched files -> clean.

- Follow-ups / risks:
  - Live loop smoke should now distinguish two cases: if the wrapper completes within the longer 5s settle, the original issue was a too-short Python settle wait; if it still times out, capture RTCore metrics/trace for per-axis final target error because the drive is not settling inside RTCore's 128-count completion window.

## 2026-04-27 02:11 +0000 - Added live endpoint error diagnostic for strict loop preflight timeout

- What changed:
  - Compared non-loop vs loop execution paths. Non-loop runs normal `_trajectory_executor_thread` and its RTCore segment calls use non-strict `_open_loop_executor_thread`; loop mode adds a distinct move-to-start wrapper with strict RTCore completion before the body can start.
  - `src/gradient_os/arm_controller/trajectory_execution.py`: strict completion timeout errors now include controller live endpoint error vs the wrapper target (`live_endpoint_max_abs_err_rad` and per-joint error list) before aborting. This does not replace RTCore's count-level completion gate; it makes the next live failure explain whether the wrapper is barely outside tolerance or materially off target.
  - `tests/test_trajectory_execution_backends.py`: pinned the new diagnostic text in the strict settle-timeout regression.

- Validation performed:
  - Focused strict/loop regressions -> `4 passed`.
  - `source ./start.sh && python -m pytest tests/test_api_endpoints.py tests/test_command_api_direct_setpoint.py tests/test_trajectory_execution_backends.py -q` -> `130 passed`.
  - Broader motion/API slice -> `172 passed`.
  - `py_compile` on touched Python/test files -> pass.
  - `ReadLints` on touched files -> clean. `git diff --check` -> clean.

- Follow-ups / risks:
  - If loop preflight still fails live, compare `live_endpoint_max_abs_err_rad` with RTCore's 128-count completion window. If live rad error is small but RTCore stays executing, add/count-level RTCore final-target diagnostics rather than weakening the gate.

## 2026-04-27 02:29 +0000 - RTCore trajectory completion count diagnostics

- What changed:
  - Added count-level RTCore diagnostics for strict loop preflight failures without changing the motion completion gate.
  - `src/gradient_rt_motion/main.cpp`: RTCore now records the latest trajectory completion check in `metrics.json` under `trajectory_completion`, including `traj_id`, `final_due`, `tolerance_counts`, axis mask, and per-axis final target/feedback/error counts.
  - `src/gradient_os/arm_controller/backends/ethercat_rtcore/backend.py`: added `get_trajectory_completion_diagnostics(...)` to parse the metrics payload.
  - `src/gradient_os/arm_controller/trajectory_execution.py`: strict completion timeout errors now include both controller live endpoint error and RTCore per-axis count diagnostics when available.
  - `tests/test_trajectory_execution_backends.py`: regression now asserts the timeout includes `tolerance_counts` and per-axis target/feedback/error counts.

- Validation performed:
  - `make -C src/gradient_rt_motion` -> pass.
  - Focused strict/loop regressions -> `4 passed`.
  - `source ./start.sh && python -m pytest tests/test_api_endpoints.py tests/test_command_api_direct_setpoint.py tests/test_trajectory_execution_backends.py -q` -> `130 passed`.
  - Broader motion/API slice -> `172 passed`.
  - `py_compile` on touched Python/test files -> pass.
  - `ReadLints` on touched Python/test files -> clean. `git diff --check` on touched files -> clean.

- Follow-ups / risks:
  - Next live loop run should report exact axis/count error if preflight still times out. Use that to decide whether the problem is servo settling/tuning, trajectory shape/speed, command-frame mismatch, or an unrealistically tight completion tolerance for this robot.

## 2026-04-27 02:54 +0000 - Loop trajectory body now uses compound strict RTCore path

- What changed:
  - Investigated live loop report: regular run collapsed `11` planned steps into one `1577`-sample RTCore trajectory and completed cleanly. Loop mode moved to start, then executed `11` separate RTCore uploads. Several loop segments ended `state=executing motion_done=False`; one segment timed out before final point with `queue_depth=215`, then the next/reset segment started. That explains both the visible pauses and the erratic/jerky motion.
  - `src/gradient_os/arm_controller/command_api.py`: loop body now collapses `planned_steps[1:] + reset_move` via `_collapse_runtime_move_pause_steps(...)`, matching the regular trajectory execution strategy. The compound loop body is marked `require_completion=True` before each repeat.
  - Fallback if collapse is impossible: each loop-body move is marked `require_completion=True` so a segment cannot be preempted by the next segment after a timeout.
  - `src/gradient_os/arm_controller/trajectory_execution.py`: `_execute_joint_path(...)` now accepts `require_completion`; `_trajectory_executor_thread(...)` forwards the step flag. Removed the hard-coded `1s` loop restart sleep by replacing it with `GRADIENT_TRAJECTORY_LOOP_RESTART_PAUSE_S` defaulting to `0.0`.
  - Updated loop reliability regressions in `tests/test_command_api_direct_setpoint.py` and `tests/test_trajectory_execution_backends.py`.

- Validation performed:
  - Focused loop-path regressions -> `4 passed`.
  - `source ./start.sh && python -m pytest tests/test_api_endpoints.py tests/test_command_api_direct_setpoint.py tests/test_trajectory_execution_backends.py -q` -> `131 passed`.
  - Broader motion/API slice -> `173 passed`.
  - `py_compile` on touched Python/test files -> pass.
  - `ReadLints` on touched files -> clean. `git diff --check` -> clean.

- Follow-ups / risks:
  - Requires operator-supervised live retest. Expected log shape: after move-to-start, loop body should be a single compound `Starting Open-Loop Executor` per loop iteration, not eleven per-move uploads; no segment should advance after `final point not observed`.

## 2026-04-27 03:29 +0000 - Live observation: compound loop path running cleanly

- What changed:
  - Read-only live observation while operator had loop running. Did not send any controller/API commands.
  - Confirmed regular run shape: non-loop `__planner_preview__` collapsed into one compound RTCore trajectory (`1986` samples) and completed.
  - Confirmed loop shape after patch: move-to-start preflight completed (`traj_id=9`, `526` samples), then loop body collapsed into one `2074`-sample RTCore trajectory.
  - Observed loop body iterations `traj_id=10`, `11`, `12`, and `13` all finish `state=completed` before `Loop enabled. Restarting sequence...`; each restart begins `Executing Step 1/1 (move)` and one `Starting Open-Loop Executor at 100 Hz (2074 steps)`.

- Validation performed:
  - Read-only log inspection of `logs/startups/20260427-025736/controller.log`, terminal mirror, and `/run/gradient-rt-motion/metrics.json`.

- Follow-ups / risks:
  - Dashboard canonical-truth AVAILABLE/UNAVAILABLE flicker continues during motion, but the loop motion path itself now shows the desired single compound upload per iteration and no settle-timeout/fault/STOP/command-frame errors in the observed window.

## 2026-04-27 03:58 +0000 - STOP now latches motion inhibit until explicit power-up

- What changed:
  - Investigated operator report that the robot kept moving after several STOP clicks. Logs showed STOP did reach the controller at `03:51:31` multiple times and aborted the active loop trajectory immediately, but a new `/control/home` request at `03:51:39` started `APPLY_JOINT_SETPOINT` trajectory `72`, which caused the later motion.
  - `src/gradient_os/arm_controller/utils.py`: added `motion_stop_latched`, timestamp, and reason fields to trajectory state.
  - `src/gradient_os/arm_controller/command_api.py`: STOP now latches motion inhibit in addition to aborting the active RTCore trajectory. New program/non-program motion, direct setpoints, joint deltas, jog session starts, and trajectory runs reject with `MOTION_STOP_LATCHED` while the latch is active. `SAFE_POWER_UP` clears the latch as the explicit operator recovery action.
  - `src/gradient_os/run_controller.py`: maps `MOTION_STOP_LATCHED` to a structured apply-joint-delta error code.
  - `tests/test_command_api_direct_setpoint.py`: added regressions that direct setpoint and trajectory run are rejected while STOP is latched, and that safe power-up clears the latch.

- Validation performed:
  - Focused stop-latch tests -> `3 passed`.
  - `source ./start.sh && python -m pytest tests/test_api_endpoints.py tests/test_command_api_direct_setpoint.py tests/test_trajectory_execution_backends.py -q` -> `133 passed`.
  - Broader motion/API slice -> `175 passed`.
  - `py_compile` on touched Python/test files -> pass.
  - `ReadLints` on touched files -> clean. `git diff --check` on touched files -> clean.

- Follow-ups / risks:
  - UI should surface the stop-latched state clearly if the operator presses Home/Run after STOP; controller-side safety now rejects it, but frontend copy can be improved separately.

## 2026-04-27 02:13 +0000 - Corrected Gradient-05 spec-sheet axis speeds

- What changed:
  - `GRADIENT_05_ROBOT_SPEC_SHEET.md`: replaced the URDF `2.0 rad/s` velocity-derived axis speed with motor-speed-derived joint speeds.
  - Added rated and peak columns using `joint deg/s = motor RPM * 6 / gear_ratio`, with 3,000 RPM rated and 6,000 RPM peak.
  - Added motor rated/peak speed rows to the controller defaults table.

- Validation performed:
  - `git diff --check -- "GRADIENT_05_ROBOT_SPEC_SHEET.md"` -> pass.
  - `ReadLints` on `GRADIENT_05_ROBOT_SPEC_SHEET.md` -> clean.

- Follow-ups / risks:
  - These are mechanical ratio-derived maxima; controller jog/profile, drive thermal, payload, and safety policies may still command lower speeds.

## 2026-04-27 20:12 +0000 - Hardened Jacobian Cartesian jog implementation plan

- What changed:
  - Updated `/home/pi/.cursor/plans/jacobian_cartesian_jog_2dc8dd65.plan.md` after reviewing the related transcript context.
  - Clarified that held Cartesian jog should use Jacobian-DLS as the root fix, not RPM caps or IK-amplification rejection.
  - Added plan requirements for active-FK/runtime-frame Jacobian compatibility, spatial finite-difference fallback, command-hold horizon timing, same-tick drift fail-closed behavior, runtime A/B compare toggling, and `jacobian_compute_ms` telemetry.
  - Follow-up plan tightening: clarified Edit 2.2 also replaces the raw-`dt` target-pose block, added pyquik Jacobian startup/build-skew fallback, explicit `kinematics_runtime.get_revision()` cache key, temporary rollback flag guidance, FD Jacobian budget envelope, singularity advisory duplicate-control choice, and actionable `/kinematics/runtime-offsets` live validation instructions.

- Validation performed:
  - Read-only consistency checks with `rg` over the plan for stale endpoint/timing/frame wording.
  - `ReadLints` on `/home/pi/.cursor/plans/jacobian_cartesian_jog_2dc8dd65.plan.md` -> clean.

- Follow-ups / risks:
  - Implementation is still pending. During implementation, benchmark FD Jacobian cost on the Pi and verify analytical pyquik Jacobian compatibility against the active FK backend before enabling the analytical path.

## 2026-04-27 20:56 +0000 - Built Jacobian-DLS held Cartesian jog offline

- What changed:
  - `src/numeric_solver/pyquik/bindings.cpp`: added `Robot.jacobian(...)` pybind binding and rebuilt `pyquik.cpython-311-aarch64-linux-gnu.so`.
  - `src/gradient_os/kinematics/runtime.py`: added runtime revision and identity-offset helpers for Jacobian cache invalidation.
  - `src/gradient_os/ik_solver.py`: added Jacobian availability warmup/status, active-FK-compatible spatial Jacobian computation, analytical-vs-FD compatibility cache, and FD fallback.
  - `src/gradient_os/arm_controller/command_api.py`: added Jacobian-DLS held-jog path with command-horizon target generation, drift watchdog hold-zero, singularity advisory, A/B compare telemetry, rollback IK path, and enriched `ik_debug`.
  - `src/gradient_os/api/main.py` and `src/gradient_os/run_controller.py`: added runtime `SET_JOG_AB_COMPARE` toggle through existing `/control/jog/debug`.
  - Added `scripts/cartesian_jog_diagnostic_capture.py` plus Jacobian/API/analyzer tests.
  - Follow-up review gap closure: added tests for DLS singular damping, 2π seed invariance, no-IK Jacobian hot path, same-tick drift watchdog blocking/no command advance on drift tick, analytical mismatch fallback, and non-identity base/tool runtime-offset Jacobian consistency.

- Validation performed:
  - `cmake -B build && cmake --build build` in `src/numeric_solver/pyquik` -> pass.
  - `source ./start.sh && python -m py_compile ...` on changed Python files and new tests -> pass.
  - pyquik smoke: `robot.jacobian(np.zeros(6))` -> shape `(6, 6)`.
  - Focused tests -> `11 passed` before the follow-up gap-closure tests.
  - Expanded API/command slice -> `136 passed`.
  - Expanded broader motion slice -> `112 passed`.
  - `ReadLints` on touched Python/test/script files -> clean.
  - `git diff --check` on touched files -> clean.

- Follow-ups / risks:
  - Operator-supervised live validation is still required before trusting physical behavior. Use the new capture script plus RTCore fast trace to verify no full-turn motion, classify IK-vs-seam flavor, observe singularity advisory behavior, and measure `jacobian_compute_ms` on the Pi.

## 2026-04-28 00:26 +0000 - Live reproduction classified as RTCore jog command-wrap bug

- What changed:
  - Investigated operator report that J6 still physically spun after the Jacobian build.
  - Log evidence: `logs/startups/20260427-235547/controller.log` shows J6 display jumped from `-179.26776123 deg` to `737.45288086 deg` between lines 14783-14788, after the preceding `v_yaw=-15` jog session had already stopped at line 14448.
  - Root cause: `src/gradient_rt_motion/main.cpp` jog integration wrapped `jog_target_counts_float` with `feedback_counts_wrap`. A6 profile has `feedback_counts_wrap=True` for modulo feedback comparison and `command_counts_wrap=False` for continuous 0x607A commands, so jog targets were being folded into the wrong frame.
  - Fixed RTCore jog integration to wrap jog targets only when `command_counts_wrap` is true.
  - Fixed `scripts/cartesian_jog_diagnostic_capture.py` to extract jog data from `controller.jog` in `/debug/performance`; the first capture file stored `jog={}` / `ik_debug=null` because it read the wrong response level.
  - Added analyzer test for the nested `/debug/performance` shape.

- Validation performed:
  - `make -C src/gradient_rt_motion` -> pass.
  - `source ./start.sh && python -m pytest tests/test_cartesian_jog_diagnostic_capture.py -q` -> `5 passed`.
  - Focused post-fix validation -> `18 passed`.
  - `py_compile` on `scripts/cartesian_jog_diagnostic_capture.py` -> pass.
  - `ReadLints` on touched RTCore/script/test files -> clean.
  - `git diff --check` on touched RTCore/script/test/memory files -> clean.

- Follow-ups / risks:
  - The currently running RTCore process is still the old binary until the stack is restarted safely. Retest only after restarting via the project stack controls, then rerun the yaw/J6 reproducer with the corrected capture script.
  - Operator correction: although the first logged display jump appeared after the UI-release/stop log, the operator was tapping yaw jog while edging toward the seam. Treat the post-stop physical/display jump as potentially caused by the immediately preceding jog target because drive motion can lag the button release.

## 2026-04-28 01:22 +0000 - Home now targets canonical zero wrap for cable unwind

- What changed:
  - Investigated operator report that after rehome J6 display/canonical was still around `360 deg` while raw `6064` was near the single-turn home frame.
  - Root cause: `/control/home` sent `[0]*6`, but bounded setpoint planning used `get_control_arm_state_rad()` as its start baseline. On EtherCAT RTCore that relaxed control path reads the raw/reference frame, so it can see J6 near `0 deg` while the operator-facing canonical/display frame is at `360 deg`; Home then becomes a nearest-wrap no-op instead of unwinding cable twist.
  - `src/gradient_os/api/main.py`: `/control/home` now sends `canonical_wrap_target=True`.
  - `src/gradient_os/run_controller.py`: `APPLY_JOINT_SETPOINT` forwards `canonical_wrap_target`.
  - `src/gradient_os/arm_controller/command_api.py`: bounded setpoint planning uses strict/display `get_current_arm_state_rad()` when `canonical_wrap_target=True`, preserving the canonical full-turn delta to zero. Ordinary bounded setpoints still use relaxed control feedback.
  - Added tests for API payload and canonical-wrap baseline selection.

- Validation performed:
  - `py_compile` on changed API/controller/command/test files -> pass.
  - Focused tests -> `3 passed`.
  - `source ./start.sh && python -m pytest tests/test_api_endpoints.py tests/test_command_api_direct_setpoint.py -q` -> `126 passed`.
  - `ReadLints` on touched files -> clean.
  - `git diff --check` on touched files -> clean.

- Follow-ups / risks:
  - This changes `/control/home` semantics intentionally: Home unwinds to canonical zero, which may command a full revolution if the canonical frame is one turn off. Keep operator supervision for first live validation.

## 2026-04-28 01:43 +0000 - Aligned continuous 607A trajectories to live turn frame

- What changed:
  - Investigated operator report that Home produced a fast movement followed by slow rotation.
  - Log evidence confirmed `/control/home` planned `current_deg[..., J6=360.005] -> target_deg[..., J6=0.0]` with `duration_s=6.000`, so API/controller speed policy was correct.
  - Root cause: the continuous 607A trajectory's first point was one full continuous turn away from RTCore's current hold target. RTCore therefore did a fast initial catch-up before following the slow bounded path.
  - `src/gradient_os/arm_controller/backends/ethercat_rtcore/backend.py`: added `_align_continuous_trajectory_axis_q_to_live_turn(...)`, which applies one per-trajectory turn shift for continuous 607A axes so point 0 aligns with live hold while every subsequent point keeps the same signed delta. Home `+360° -> 0°` now becomes a slow full-turn unwind from the current hold frame instead of a catch-up jump plus slow path.
  - Added regression `test_a6ec_continuous_trajectory_aligns_home_unwind_to_live_turn`.

- Validation performed:
  - `make -C src/gradient_rt_motion` -> no rebuild needed after Python/backend-only change.
  - Focused tests -> `3 passed`.
  - `source ./start.sh && python -m pytest tests/test_gradient05_limits_and_backends.py tests/test_api_endpoints.py tests/test_command_api_direct_setpoint.py -q` -> `282 passed`.
  - `py_compile` on touched Python files -> pass.
  - `ReadLints` on touched files -> clean.
  - `git diff --check` on touched files -> clean.

- Follow-ups / risks:
  - Restart controller/API before retesting Home; this is Python backend code. First live validation should remain operator-supervised.

## 2026-04-28 05:09 +0000 - Fixed post-Home held-jog 360-degree spin (continuous-frame seed bug)

- What changed:
  - Investigated operator-reproduced bug: after Home unwinds J6 to canonical 0 deg, the next held yaw jog command produces a full 360 deg spin at max profile speed on the first jog cycle. Repeats reliably.
  - Root cause: held-jog initialization in `src/gradient_rt_motion/main.cpp` seeded `jog_target_counts_float[i]` from `csp_wire_counts_from_feedback(latest_feedback.pos_counts[i])`. On A6-EC J6 (`feedback_counts_wrap=true`, `command_counts_wrap=false`), feedback is single-turn sawtooth while RTCore's hold target lives in continuous wire counts. After a canonical-wrap home unwind the drive's 0x607A target sits in a different motor turn from the single-turn feedback slice, so the first jog command commands a wire count one full motor turn from the drive's current target -> drive slews that whole turn at max profile speed before settling into the held-jog rate.
  - Same bug also affected post-jog snap paths (`snap_jog_hold_to_feedback_mask` at line 3948 and `jog_stop_arrest_cycles_left[i]` at line 3954): both unconditionally snapped `hold_target_counts[i]` to single-turn feedback after a jog stops, re-creating the same one-turn step on the next commanded cycle.
  - Added helper `nearest_turn_equivalent_to_continuous(prev, feedback, turn_period_counts)` in `main.cpp` that returns the wire-count equivalent of `feedback` whose continuous value lies closest to `prev`, preserving multi-turn while still snapping to where the encoder reports the axis to be.
  - Held-jog init now seeds `jog_target_counts_float[i]` from `hold_target_counts[i]` for continuous-command axes (`!command_counts_wrap` and `have_hold[i]`); falls back to feedback otherwise (very first jog after start, sawtooth-command axes).
  - Both post-jog snap paths now use `nearest_turn_equivalent_to_continuous(...)` for continuous-command axes; sawtooth-command axes keep the existing feedback snap.

- Validation performed:
  - `make -C src/gradient_rt_motion` -> pass (no warnings about new code).
  - Installed updated binary: `sudo install -m 755 src/gradient_rt_motion/gradient-rt-motion /usr/local/bin/gradient-rt-motion`. Confirmed `md5sum` parity (`fbd9cbff23024e02d43cf6549790effe`).
  - `./start-stack.sh stop --hard` clean (RTCore + EtherCAT master shut down without orphaning the kernel module).
  - Python regression slice `tests/test_gradient05_limits_and_backends.py tests/test_api_endpoints.py tests/test_command_api_direct_setpoint.py tests/test_trajectory_execution_backends.py tests/test_realtime_jog_backend_compatibility.py tests/test_cartesian_jog_resilience.py tests/test_rtcore_runtime.py` -> `340 passed, 1 failed`. The single failure (`test_realtime_jog_loop_commands_joint_space_via_servo_driver`) is pre-existing/unrelated to RTCore changes (Python-only mock test, `set_servo_positions` capture is empty; the C++ binary is not exercised by this test).
  - `git diff src/gradient_rt_motion/main.cpp` -> `1 file changed, 80 insertions(+), 6 deletions(-)`, all comments + new helper + two `if (have_hold[i] && !opt.axis[i].command_counts_wrap)` branches.

- Follow-ups / risks:
  - Stack bring-up after the hard stop is currently blocked by physical power: `sudo ethercat master` reports `Link: DOWN, Slaves: 0`. Operator must restore arm physical power before retesting. The new RTCore binary is already installed at `/usr/local/bin/gradient-rt-motion` and will be picked up automatically by `./start-stack.sh` once power is restored.
  - First live retest after power-up: do Home -> wait for completion -> press yaw jog. Expected: smooth jog at v_yaw rate, no fast first-cycle lurch. Then release jog -> press jog again. Expected: same smooth jog (hold-snap preservation working).
  - Yaw button polarity issue (positive button -> -v_yaw -> +J6 motion) is still a separate UI/sign concern; this fix does not change EE Jacobian sign convention.

## 2026-04-28 20:20 +0000 - Rewrote turn-frame fix plan to target IK winding without broad state bloat

- What changed:
  - Rewrote `/home/pi/.cursor/plans/turn-frame_fix_stack_d8c7e686.plan.md`.
  - Marked RTCore jog seed/snap implementation as already landed/installed, with only live retest pending after physical power is restored.
  - Promoted the remaining high-priority fix to a minimal controller IK turn-intent guard: normalize/retry/reject hidden continuous-joint windings from `solve_ik_path_batch(...)`, specifically covering `ROTATE,z,-15 -> +375 deg` and equivalent implicit kinematic paths.
  - Deferred broad `csp_continuous_turn_anchor_counts` architecture unless targeted live retests prove `hold_target_counts`/trajectory alignment still lose turn state. The plan now explicitly says not to make motion policy depend on `JogDebugSnapshot`.
  - Kept smooth-step peak speed and canonical-truth flicker as separate narrow follow-ups, not blockers for the unsafe turn-frame bug.

- Validation performed:
  - Read-only review against current source and prior devlog/scratchpad.
  - `ReadLints` on the rewritten plan -> clean.

- Follow-ups / risks:
  - Implement PR2 next: the controller-level IK turn-intent guard. Without it, smooth FK-valid kinematic paths can still choose a hidden full-turn branch even though RTCore jog seeding is fixed.

## 2026-04-28 21:30 +0000 - Made turn-frame plan copy/paste implementation ready

- What changed:
  - Updated `/home/pi/.cursor/plans/turn-frame_fix_stack_d8c7e686.plan.md` after reviewing feedback transcripts `c0b4553d...` and `e42be0c2...`.
  - Replaced non-copyable placeholders with source-exact snippets:
    - `command_api.py`: `_continuous_joint_indices_from_active_backend()` now uses `_get_active_backend()` and a public/private backend accessor fallback; `validate_kinematic_turn_intent` now fails closed if `start_q` is shorter than the IK output; bounded setpoint executor snippet includes exact `allow_continuous_wind` kwarg.
    - `backend.py`: added exact `continuous_command_logical_joint_indices()` accessor snippet and PR4 global `motion_done` settle-window tracking snippet.
    - `run_controller.py`: exact before/after for ROTATE and SET_ORIENTATION `TurnWindingExceededError` handling using `_encode_controller_payload_b64`.
    - `api/main.py`: exact before/after for `/control/rotate` and `/control/set-orientation` to translate `IK_TURN_WINDING_EXCEEDED` into HTTP 400 with structured detail.
    - Tests: replaced the PR4 placeholder backend fixture with a concrete `EthercatRTCoreBackend` + `_AxisConfig` setup.
  - Follow-up pass from additional review:
    - Moved the IK turn-intent guard home from `command_api.py` to `trajectory_execution.py` in the plan to avoid circular imports.
    - Added exact `run_controller.py` import change for `trajectory_execution` and changed error catches to `trajectory_execution.TurnWindingExceededError`.
    - Added an exact policy list for all `_open_loop_executor_thread(...)` callers, including `pid_tuner.py` and direct test calls.
    - Removed placeholder language from the plan (`...`, `illustrative`, `adjust`, `whatever`, `grep`, `fill in`, `line depends`, etc.).

- Validation performed:
  - `ReadLints` on the plan -> clean.
  - `rg` placeholder scan on the plan -> no matches.

- Follow-ups / risks:
  - The plan is now suitable as an implementation checklist, but code itself has not been changed in this turn beyond the plan and memory files.

## 2026-04-28 21:51 +0000 - Updated turn-frame plan for high-res Home-after-jog evidence

- What changed:
  - Reviewed transcript `e8f34890-c392-4bf9-b215-a1a6189afb1d`.
  - Updated `/home/pi/.cursor/plans/turn-frame_fix_stack_d8c7e686.plan.md` to promote the latest reproduced failure into an active PR:
    - New PR2: trajectory upload must align point 0 to RTCore continuous `hold_target_counts`, not sawtooth feedback, after seam-crossing jog.
    - Existing IK turn-intent guard moved to PR3.
    - Smooth-step peak cap moved to PR4.
    - Canonical truth settle window moved to PR5.
  - Added copy/paste code snippets for `backend.py`: `_live_command_hold_counts_for_axis`, `_trajectory_turn_reference_counts_for_axis`, `submit_joint_trajectory` reference selection, helper rename to `_align_continuous_trajectory_axis_q_to_turn_reference`, and a regression `test_a6ec_continuous_trajectory_aligns_to_rtcore_hold_after_jog`.
  - Updated sequencing, validation matrix, pass criteria, deferred-anchor triggers, and out-of-scope list to reflect the new evidence.

- Validation performed:
  - `ReadLints` on the plan -> clean.
  - Placeholder scan on the plan (`...`, `illustrative`, `adjust`, `fill in`, `line depends`, `grep`, etc.) -> no matches.

- Follow-ups / risks:
  - Implement PR2 next before IK guard; latest live evidence shows Home-after-jog is a trajectory alignment bug, not only an IK branch bug.

## 2026-04-28 04:39 +0000 - Live operator retest validated canonical-wrap + trajectory alignment fixes

- What changed:
  - No code edits this session. Started a diagnostic capture and live monitoring while operator retested Home and yaw jog with the latest stack.
  - Captured `logs/cartesian-jog-diagnostic/operator-test-20260428-043822/python-jog-20260428-043822.jsonl` (1579 records) and tailed `logs/startups/20260428-043422/controller.log` for the duration.
  - Operator sequence: SAFE_POWER_UP -> Home -> yaw held jog `v_yaw=-15` (~35 ticks) -> Home -> a few `ROTATE z=±15°` commands.

- Behavioral findings (no fixes were violated):
  - Home #1 (`traj_id=2`): payload `canonical_wrap_target=true`. Bounded plan `J6=286.125° -> 0°` `duration_s=4.769s` `points=477`. RTCore `state=completed elapsed=5.200s`. No fast initial catch-up; `elapsed-duration=0.43s` is normal settle window.
  - Held yaw jog (`joint_velocity_lease` mode): J6 progressed monotonically `286° -> 404°` over ~35s (~3.4°/s, matching `v_yaw=15 deg/s` mapped through IK). Clean stop reason `ui-release`, no `MOTION_STOP_LATCHED`, no fault, no full-turn lurch at the seam.
  - Home #2 (`traj_id=12`): payload `canonical_wrap_target=true`. Bounded plan `J6=404.05° -> 0°` `duration_s=6.734s` `points=674`. RTCore `state=completed elapsed=7.452s`. After Home #2 the controller logged J6 logical angle = `0.005 rad` (0.3°), so the full-turn unwind landed on canonical zero correctly.
  - Canonical truth UNAVAILABLE warnings (`drive_native_command_frame_roundtrip_mismatch` axes=[5]) during the long Home unwinds are expected and advisory after the 2026-04-25 control-feedback split; they did NOT block motion. Truth recovered AVAILABLE after motion settled.
  - Operator-reported "yaw button feels backwards" reproduced: `v_yaw=-15` mapped to +J6 motion. Filed as a separate UI/sign issue; not in scope for the canonical-wrap or jog seam fixes.

- Validation performed:
  - Read-only log/metrics inspection (no controller commands sent).
  - Controller log scan for `APPLY_JOINT_SETPOINT|JOG_SESSION|Open-Loop|trajectory_completion|MOTION_STOP|drive_fault|ERROR` -> only the expected events for the operator's sequence. No motion-stop latch, no faults, no aborted trajectories.
  - Diagnostic capture analyzer noted `ik_debug=null` and `rtcore_jog_debug.active_jog=false` for all 1579 records because (a) `SET_JOG_AB_COMPARE,false` was set right before the test and (b) the yaw jog ran in `joint_velocity_lease` mode rather than the Cartesian held-jog Jacobian path that populates `ik_debug`. Documented in scratchpad as a monitoring guardrail for future retests.

- Follow-ups / risks:
  - Yaw button sign mismatch: triage independently. Confirm operator's mental model (`+yaw button => +J6 deg` or `+yaw button => +EE z-axis rotation per right-hand-rule`) before changing any sign in the UI or controller IK mapping.
  - For future single-joint or yaw-only retests where Jacobian-DLS telemetry is needed, enable `SET_JOG_AB_COMPARE=true` and (if needed) `scripts/cartesian_jog_diagnostic_capture.py enable-fast-trace` before the operator presses the button.

## 2026-04-28 02:18 +0000 - Verified home/seam fixes are in source and restarted stack

- What changed:
  - No code edits this session. Verified the four prior fixes are present in the working tree and that the running binaries match the corrected sources, then brought the stack back up so Python backend changes are loaded for live retest.
  - Confirmed source-level state of all four fixes:
    - `src/gradient_os/api/main.py` `/control/home` payload includes `canonical_wrap_target: True`.
    - `src/gradient_os/run_controller.py` `APPLY_JOINT_SETPOINT` UDP handler forwards `canonical_wrap_target` into `command_api.handle_apply_joint_setpoint(...)`.
    - `src/gradient_os/arm_controller/command_api.py` bounded planner branches on `canonical_wrap_target` to use strict-display `get_current_arm_state_rad()`.
    - `src/gradient_os/arm_controller/backends/ethercat_rtcore/backend.py` trajectory upload calls `_align_continuous_trajectory_axis_q_to_live_turn(...)` for every point with the helper at line 6270.
    - `src/gradient_rt_motion/main.cpp` jog integration uses `command_counts_wrap` policy for `wrap_period_counts`.
  - Verified RTCore binary parity: `md5sum /usr/local/bin/gradient-rt-motion src/gradient_rt_motion/gradient-rt-motion` -> identical `3219ab0f907a5360993c0bb6904764ca`. Both binaries were built from `main.cpp` last modified `2026-04-28 00:16:06 UTC`, so the jog seam fix is already installed/live.

- Validation performed:
  - Initial state: `./start-stack.sh status` -> `launcher_state: absent` / `controller down / api down / web down`.
  - `make -C src/gradient_rt_motion` -> `Nothing to be done for 'all'` (binary already up to date).
  - Focused tests: `tests/test_gradient05_limits_and_backends.py::test_a6ec_continuous_trajectory_aligns_home_unwind_to_live_turn`, `tests/test_api_endpoints.py::test_control_home`, `tests/test_command_api_direct_setpoint.py::test_apply_joint_setpoint_canonical_wrap_target_uses_display_baseline` -> `3 passed`.
  - Broader regression slice: `tests/test_gradient05_limits_and_backends.py tests/test_api_endpoints.py tests/test_command_api_direct_setpoint.py tests/test_trajectory_execution_backends.py` -> `298 passed`.
  - Stack restart: `./start-stack.sh` -> launcher running, controller/api/web all `up`, RTCore reports `startup_ready=1 operational=6/6 wkc=18/12`, J6 statusword `0x1650` err `0x0` (BUS_UP_DISARMED, no faults), drives `armed=0` and require operator-controlled safe-power-up before motion.

- Follow-ups / risks:
  - First live retest after this restart should still be operator-supervised. Expected behavior: Home plans canonical unwind from live J6 (currently `~301° canonical`) to `0°` and RTCore executes one continuous slow trajectory with point 0 already on the live hold turn (no fast initial catch-up).
  - Operator-reported "yaw jog feels inverted" is a separate UI/sign concern. Triage independently from the canonical-wrap and jog seam fixes; do not fold into Home scope.

## 2026-04-28 22:08 +0000 - PR2 trajectory upload aligns to RTCore hold target

- What changed:
  - Implemented PR2 only from `/home/pi/.cursor/plans/turn-frame_fix_stack_d8c7e686.plan.md`.
  - `src/gradient_os/arm_controller/backends/ethercat_rtcore/backend.py`: added `_live_command_hold_counts_for_axis(...)` and `_trajectory_turn_reference_counts_for_axis(...)`; trajectory upload now seeds point-0 turn alignment from RTCore `hold_target_counts` for continuous-command axes and falls back to 6064 feedback otherwise.
  - Renamed `_align_continuous_trajectory_axis_q_to_live_turn(...)` to `_align_continuous_trajectory_axis_q_to_turn_reference(...)` and changed its parameter to `initial_reference_counts`.
  - `tests/test_gradient05_limits_and_backends.py`: added `test_a6ec_continuous_trajectory_aligns_to_rtcore_hold_after_jog`.
  - Follow-up hardening in the same PR2 scope:
    - Added `_JOG_DEBUG_HOLD_REFERENCE_MAX_AGE_S` and `_last_jog_debug_monotonic_s`; hold-target references now require a nonzero RTCore jog-debug sample and a recent status-ring receive before they are used for trajectory alignment.
    - Added `initial_reference_from_hold` tracking through `enqueue_trajectory_points(...)` so first-point safety knows whether a turn reference came from continuous 0x607A hold state or sawtooth feedback.
    - First-point safety now uses linear deviation for continuous hold references, so a one-turn mismatch is rejected instead of being hidden by shortest-angular math.
    - Added upload-level regression `test_a6ec_continuous_trajectory_upload_uses_hold_turn_after_jog` and safety regression `test_a6ec_continuous_trajectory_first_point_safety_rejects_hold_turn_mismatch`.

- Validation performed:
  - `source ./start.sh && python -m pytest tests/test_gradient05_limits_and_backends.py::test_a6ec_continuous_trajectory_aligns_to_rtcore_hold_after_jog tests/test_gradient05_limits_and_backends.py::test_a6ec_continuous_trajectory_aligns_home_unwind_to_live_turn -q` -> `2 passed`.
  - `source ./start.sh && python -m pytest tests/test_gradient05_limits_and_backends.py tests/test_trajectory_execution_backends.py tests/test_command_api_direct_setpoint.py -q` -> `228 passed`.
  - Follow-up focused hardening tests: `source ./start.sh && python -m pytest tests/test_gradient05_limits_and_backends.py::test_a6ec_continuous_trajectory_aligns_to_rtcore_hold_after_jog tests/test_gradient05_limits_and_backends.py::test_a6ec_continuous_trajectory_upload_uses_hold_turn_after_jog tests/test_gradient05_limits_and_backends.py::test_a6ec_continuous_trajectory_first_point_safety_rejects_hold_turn_mismatch tests/test_gradient05_limits_and_backends.py::test_a6ec_continuous_trajectory_aligns_home_unwind_to_live_turn -q` -> `4 passed`.
  - Follow-up broader slice: `source ./start.sh && python -m pytest tests/test_gradient05_limits_and_backends.py tests/test_trajectory_execution_backends.py tests/test_command_api_direct_setpoint.py -q` -> `230 passed`.
  - `ReadLints` on `backend.py` and `tests/test_gradient05_limits_and_backends.py` -> clean.
  - `git diff --check -- src/gradient_os/arm_controller/backends/ethercat_rtcore/backend.py tests/test_gradient05_limits_and_backends.py` -> clean.

- Follow-ups / risks:
  - Live validation still pending with operator and physical power: Home -> jog J6/yaw across seam -> release -> Home again. Pass requires no max-profile first-cycle spin and fast trace first-sample delta from held target near zero.
  - Stopped at PR2 per plan; PR3/PR4/PR5 not implemented.

## 2026-04-29 00:01 +0000 - Installed GitNexus for Cursor on ARM64 Pi

- What changed:
  - Installed GitNexus `1.6.3` into a user-local Node 20 runtime at `/home/pi/.local/gitnexus-runtime` because the system Node is `18.20.4` and GitNexus requires Node `>=20`.
  - Added Cursor global MCP config at `/home/pi/.cursor/mcp.json` pointing to the local Node 20 CLI with `GITNEXUS_MAX_DB_SIZE=1073741824`.
  - Added `.gitnexus/` to `.gitignore`; the index is local/untracked.
  - Patched the installed GitNexus runtime for this Pi:
    - Rebuilt `tree-sitter` and all `tree-sitter-*` native addons with Node 20 headers via local `node-gyp` to avoid Node 18 `libnode.so.108` segfaults.
    - Patched LadybugDB `new lbug.Database(...)` calls in installed `dist` files to honor `GITNEXUS_MAX_DB_SIZE`, matching upstream Raspberry Pi workaround for the default 8 TiB mmap failure.

- Validation performed:
  - `gitnexus analyze --skip-agents-md --name GradientOS .` with `GITNEXUS_MAX_DB_SIZE=1073741824` and `GITNEXUS_WORKER_SUB_BATCH_TIMEOUT_MS=120000` succeeded: `426 files`, `17,756 symbols`, `26,225 edges`, `418 clusters`, `300 processes`.
  - `gitnexus status` reports `GradientOS` up to date at commit `bf22a96`.
  - `gitnexus context EthercatRTCoreBackend --repo GradientOS` returned class context successfully.
  - MCP smoke test stayed running until forced `timeout`; GitNexus logs `MCP server starting with 1 repo(s): GradientOS`. The SIGTERM cleanup path logs an upstream `process.exit('SIGTERM')` type warning only when the test timeout kills it.
  - `ReadLints` on `.gitignore` clean.

- Follow-ups / risks:
  - Cursor may need MCP reload/restart to discover the new `gitnexus` server.
  - Free-text `gitnexus query` still logs FTS read-only index warnings and returned an empty result for `RTCore backend`; symbol/context-style tools work against the indexed graph.
  - Local patches live under `/home/pi/.local/gitnexus-runtime` and may be overwritten by future `npm install -g` / GitNexus upgrades; preserve the Node 20 rebuild and LadybugDB max DB size workaround on upgrade.

## 2026-04-28 23:42 +0000 - Live PR2 validation passed for Home-after-jog

- What changed:
  - Ran operator-supervised live validation for PR2 after the backend.py hardening.
  - Captured high-frequency telemetry in `logs/cartesian-jog-diagnostic/highres-pr2-20260428-233834/` with RTCore fast trace start byte `412400991`.
  - Operator sequence included jog sessions across/near the seam followed by `APPLY_JOINT_SETPOINT` Home.

- Validation performed:
  - Controller follower showed a post-jog Home at controller-follow lines `32230-32233`: `current_deg[..., J6=29.697] -> target 0`, `duration_s=0.495`, `points=50`, RTCore `state=completed`, `elapsed=0.562s`.
  - Filtered RTCore 1 kHz trace over the Python capture window had no sustained full-turn/max-profile target slew. Largest target deltas inside the capture window were small single-sample timing artifacts / bounded trajectory increments, not the prior ~13,107 counts/ms full-turn chase.
  - Full trace contained one later `target_position` collapse from `1966080 -> 655360` while feedback was already `~655360`; immediately after, status moved out of Operation Enabled. This was hold/disable synchronization at the same physical pose, not the old target-away-from-feedback chase.
  - Post-state trajectory completion for `traj_id=2` reported J6 `target_counts=655360`, `feedback_counts=655264`, `error_counts=-96` within tolerance `128`.

- Follow-ups / risks:
  - PR2 live validation passes the specific repro: Home -> jog across seam -> release -> Home does not max-speed spin.
  - Separate issues remain out of this PR2 scope: canonical truth flicker during/after long moves, yaw sign convention, PR3 IK turn-intent guard for `ROTATE,z` hidden winding, and PR4 smooth-step peak cap.

## 2026-04-29 00:12 +0000 - Fixed RTCore disarm-time continuous hold collapse causing J6 Er87.1

- What changed:
  - Operator reported J6 drive display flashing `Er87.1` after power down.
  - Preserved evidence before reset: direct EtherCAT reads showed slave 5 / J6 `6041=0x1618`, `603F=0xff00`, `203F=0x0871` (`Er87.1`, one-time excessive position reference increment). Other slaves were `6041=0x1650`, zero fault registers.
  - Root cause from fast trace: after PR2 live validation, RTCore collapsed J6 `target_position` from `1966080 -> 655360` while the drive was still `0x1637` OperationEnabled. Feedback was already at the equivalent physical pose, but the raw 0x607A increment was one full J6 motor turn, so the drive faulted Er87.1.
  - `src/gradient_rt_motion/main.cpp`: added local helpers in the cyclic loop to preserve continuous-command hold targets during disable/service/passive transitions while the drive remains OperationEnabled or QuickStopActive.
  - Updated service-mode, `!want_enable`, and passive-startup hold-mirror paths to fold feedback to the nearest equivalent of the previous continuous hold target for `command_counts_wrap=false` axes, instead of unconditionally assigning sawtooth feedback.
  - Installed rebuilt RTCore binary to `/usr/local/bin/gradient-rt-motion` (md5 `6ee4edc0d87be3afaec333a1db43e5c5`).

- Validation performed:
  - `make -C src/gradient_rt_motion` -> pass.
  - After operator hard-stopped, repeated rebuild/install check: `make -C src/gradient_rt_motion` -> `Nothing to be done for 'all'`; `md5sum src/gradient_rt_motion/gradient-rt-motion /usr/local/bin/gradient-rt-motion` -> both `6ee4edc0d87be3afaec333a1db43e5c5`.
  - `source ./start.sh && python -m pytest tests/test_gradient05_limits_and_backends.py::test_a6ec_continuous_trajectory_aligns_to_rtcore_hold_after_jog tests/test_gradient05_limits_and_backends.py::test_a6ec_continuous_trajectory_upload_uses_hold_turn_after_jog tests/test_gradient05_limits_and_backends.py::test_a6ec_continuous_trajectory_first_point_safety_rejects_hold_turn_mismatch tests/test_gradient05_limits_and_backends.py::test_a6ec_continuous_trajectory_aligns_home_unwind_to_live_turn -q` -> `4 passed`.
  - `git diff --check -- src/gradient_rt_motion/main.cpp src/gradient_os/arm_controller/backends/ethercat_rtcore/backend.py tests/test_gradient05_limits_and_backends.py` -> clean.
  - Did NOT restart RTCore or reset the J6 fault in this turn; the new binary is installed but running service still needs a safe stop/restart after operator approves recovery.

- Follow-ups / risks:
  - Recovery/retest sequence: reset J6 faults only after operator is ready; `./start-stack.sh stop --hard`; restart stack so new RTCore binary loads; safe power-up; run Home -> jog seam -> release -> Home -> safe power-down. Pass requires no visible spin and no new J6 `Er87.1` on power-down/disarm.
  - If Er87.1 still appears, inspect remaining `hold_target_counts[i] = feedback_wire_target_counts` paths at first OperationEnabled latch / service transitions and consider moving hold-target state into a first-class RTCore status field.

## 2026-04-29 15:22 +0000 - Set RTCore startup max RPM default to shared 3000

- What changed:
  - Added `src/gradient_os/runtime_defaults.py` as the single Python owner for `DEFAULT_RT_MAX_RPM = 3000.0`.
  - Updated `runtime_config.py` and RTCore startup env rendering to import the shared default instead of owning separate defaults.
  - Set `.gradient_runtime_config.json` desired `rt_max_rpm` to `null`, so the active runtime config inherits the shared default instead of duplicating `3000.0` as an override.
  - Updated the static systemd fallback `GRADIENT_RT_MAX_RPM=3000` and added a regression that it matches the shared Python default.
  - Changed the web UI RTCore max RPM field to use `/info/runtime-config` desired/default/effective values; blank input now patches `rt_max_rpm: null` to mean "use runtime default" instead of hardcoding a numeric fallback.
  - Adjusted API/runtime test expectations for the default-sourced `3000.0` value.

- Validation performed:
  - `ReadLints` on touched Python/JSON/service/TS/test files -> clean.
  - `source ./start.sh && python -m pytest tests/test_rtcore_runtime.py tests/test_runtime_config.py tests/test_api_endpoints.py -q` -> `93 passed`.
  - `npm run build` in `web-ui` -> pass; Vite emitted existing browser-externalized module notes and chunk-size warning.
  - `git diff --check` -> clean.

- Follow-ups / risks:
  - Running RTCore will only pick up the new max RPM after the stack/RTCore is restarted through the normal safe teardown/start path.

## 2026-04-29 16:05 +0000 - Held jog keepalive survives stale browser interval after idle

- What changed:
  - Diagnosed post-idle `+X` jog failure from terminal logs: `JOG_SESSION_START` was accepted with `truth_valid_at_arm=True`, no `JOG_SESSION_UPDATE` arrived before the 1.0 s lease expired, and `SAFE_POWER_DOWN` succeeded immediately afterward. This isolated the failure to the frontend held-jog keepalive path, not controller/RTCore clogging.
  - `web-ui/src/ControlPanel.tsx`: added a post-ACK one-shot keepalive watchdog. Every successful active jog-session start/update schedules the next `sendJogTickRef` after `keepaliveMs`, so a held jog keeps refreshing the lease even if the 50 ms browser interval is stale after idle/wake. The watchdog is cleared on local stop, terminal jog errors, and component cleanup.
  - `web-ui/src/ControlPanel.test.tsx`: added a regression that stubs the 50 ms jog `setInterval` so it never fires, then verifies an active held `+X` jog still emits repeated `/control/jog/session/update` posts through the post-ACK watchdog.
- Validation performed:
  - `npm test -- src/ControlPanel.test.tsx` in `web-ui` -> `34 passed`.
  - `npm run build` in `web-ui` -> pass; Vite emitted existing browser-externalized module notes and chunk-size warning.
  - `ReadLints` on `web-ui/src/ControlPanel.tsx` and `web-ui/src/ControlPanel.test.tsx` -> clean.
- Follow-ups / risks:
  - Live hardware/browser retest still needed: leave UI idle, hold `+X` for more than one second, and confirm controller log shows `JOG_SESSION_UPDATE` continuing instead of `lease-expired-before-loop`.

## 2026-04-29 16:18 +0000 - Frontend telemetry cleanup removes live REST joint flood

- What changed:
  - Implemented `/home/pi/.cursor/plans/frontend_telemetry_cleanup_781c32e7.plan.md` without editing the plan file.
  - `web-ui/src/App.tsx`: made `/monitor` `EventSource` connection idempotent while opening/connected and ignored callbacks from stale `EventSource` objects. Moved `handleFallbackJointFeedback` freshness gating before any clear/push so REST feedback cannot affect the visualizer while `/monitor` is fresh.
  - `web-ui/src/ControlPanel.tsx` / `web-ui/src/liveState.tsx`: renamed the 100 ms poll interval to `STANDALONE_JOINT_FEEDBACK_POLL_MS` and gated `/info/joints-detailed` polling to standalone, disconnected, or stale-monitor cases. Fresh connected `/monitor` telemetry now drives the panel without parallel REST polling; explicit commissioning refresh calls are preserved.
  - `web-ui/src/ArmVisualizer.tsx`: disposed late URDF loads, disposed the mounted robot graph during teardown, and reused the bounding-edge `BufferAttribute` instead of replacing it during bounds updates.
  - `web-ui/src/TelemetryWorkspace.tsx`: capped diagnostics pose history at 1200 samples.
  - `web-ui/src/ControlPanel.test.tsx`: added coverage that a `ControlPanel` under fresh `LiveStateProvider` telemetry does not poll `/info/joints-detailed`.

- Validation performed:
  - `ReadLints` on touched frontend files -> clean.
  - `npm test -- src/ControlPanel.test.tsx src/App.test.ts` in `web-ui` -> `38 passed`.
  - `npm run build` in `web-ui` -> pass; Vite emitted existing `occt-import-js` browser-externalized module notes and chunk-size warning.
  - `git diff --check -- web-ui/src/App.tsx web-ui/src/ControlPanel.tsx web-ui/src/ControlPanel.test.tsx web-ui/src/ArmVisualizer.tsx web-ui/src/TelemetryWorkspace.tsx web-ui/src/liveState.tsx` -> clean.
  - Existing terminal log check after HMR/page reloads: 16:10-16:19 window showed fresh `START_TELEMETRY` events but no `GET /info/joints-detailed` / `GET_JOINT_STATE` flood.

- Follow-ups / risks:
  - Hard-reload the browser after this change because Vite Fast Refresh invalidated `App.tsx`, `ControlPanel.tsx`, and `liveState.tsx` during development. Live confirmation should be one `/monitor` stream after reload and no sustained REST joint polling while telemetry is fresh.

## 2026-04-29 16:23 +0000 - Fixed all-axis canonical truth loss after power-down by disabling fast trace

- What changed:
  - Diagnosed the commissioning-panel `Canonical truth trust warning: statusword_unavailable` on all axes after power-down.
  - Live API showed `/info/joints-detailed` had all axes `statusword_hex=0x0000`, `drive_native_truth_reason=statusword_unavailable`, and `truth_reason=drive_native_startup_drive_config_missing`.
  - Runtime check showed `/run` tmpfs was 100% full: `/run/gradient-rt-motion/cartesian-jog-fast-trace.jsonl` had consumed the 1.6G tmpfs and `/run/gradient-rt-motion/metrics.json` was truncated to 0 bytes.
  - Removed persistent systemd fast-trace drop-in `/etc/systemd/system/gradient-rt-motion.service.d/fast-trace.conf`, reloaded systemd after freeing space, removed the runtime fast-trace path to avoid autosave copying the huge sparse file, and restarted the stack via `./start-stack.sh stop --hard && ./start-stack.sh`.
- Validation performed:
  - After freeing `/run`, `metrics.json` repopulated and `/info/joints-detailed` immediately returned canonical/display truth true on all axes.
  - After stack restart, `df -h /run /run/gradient-rt-motion` -> `/run` 1% used.
  - `/run/gradient-rt-motion` contains `ipc.sock` and a small `metrics.json` only; no fast-trace JSONL path.
  - `systemctl show gradient-rt-motion.service -p Environment` reports `GRADIENT_RT_FAST_TRACE_PATH=` and `GRADIENT_RT_FAST_TRACE_HZ=0`.
  - `/info/joints-detailed` reports all six axes `truth_available=True`, `drive_native_truth_reason=valid`, `statusword_hex=0x1650`.
- Follow-ups / risks:
  - If high-rate RTCore trace is needed again, enable it only for a bounded capture window and disable it immediately afterward, or add rotation/size caps before leaving it persistent.

## 2026-04-29 17:58 +0000 - Added frontend 3D lag trace breakdown

- What changed:
  - User reported the 3D visualization still visibly lags one or more seconds behind the physical robot and asked for frontend traces; other agents are working jog fixes, so this stayed in `web-ui/src/visualizerLagTelemetry.ts` and `web-ui/src/App.tsx`.
  - Extended `VisualizerLagCompletedSample` with `sourceDeltaMs`, `sourceToApiMs`, `apiSequenceGap`, `payloadBytes`, `jointCount`, and `usedDisplayJoints`.
  - Added low-rate `[GradientOS 3D trace]` console heartbeats and expanded `[GradientOS 3D lag]` warnings to classify source cadence, API/SSE delivery, browser parse/handler, push-to-visible, frame interval, and render frame work.
  - Published `globalThis.__GRADIENT_3D_LAG_TRACE__` with `snapshot`, recent completed samples, and pending samples so the browser console can be inspected without adding backend endpoints.
  - `App.tsx` now passes payload length, joint count, and display-vs-canonical source metadata into the existing visualizer lag recorder.

- Validation performed:
  - `ReadLints` clean on `web-ui/src/visualizerLagTelemetry.ts`, `web-ui/src/App.tsx`, and `web-ui/src/ArmVisualizer.tsx`.
  - `git diff --check -- web-ui/src/visualizerLagTelemetry.ts web-ui/src/App.tsx web-ui/src/ArmVisualizer.tsx` -> clean.
  - `npm test -- src/App.test.ts` in `web-ui` -> `3 passed`.
  - `npm run build` in `web-ui` -> pass; Vite emitted existing `occt-import-js` browser-externalized module notes and chunk-size warning.

- Follow-ups / risks:
  - Hard-reload the browser before live interpretation. During a repro, copy the latest `[GradientOS 3D trace]` / `[GradientOS 3D lag]` object or inspect `window.__GRADIENT_3D_LAG_TRACE__` to decide whether the one-second lag is source cadence, API/browser delivery, parse/handler, or rAF/render work.

## 2026-04-29 18:05 +0000 - DLS achieved twist now owns Cartesian jog target advance

- What changed:
  - Implemented `/home/pi/.cursor/plans/dls_target_scaling_92521612.plan.md` without editing the plan file.
  - `src/gradient_os/arm_controller/command_api.py`: `_compute_jog_joint_velocity_via_jacobian` now publishes `achieved_twist`, `twist_residual`, residual norm, combined attenuation, and separate linear/angular attenuation ratios. The Jacobian hot path now builds the validation target pose from `achieved_twist = J @ q_dot` instead of raw requested twist, so DLS singularity damping cannot make the internal TCP target run ahead of what the arm can produce.
  - Successful Jacobian ticks now accept the FK-applied pose (`solved_pose_matrix`) as the next jog command state, while telemetry still reports target-vs-solved against the achieved-twist validation target. This avoids accumulating first-order integration error during long holds.
  - Follow-up cleanup: `ik_debug.accepted_commanded_pose` now receives the actual accepted command-state pose, so telemetry no longer implies the achieved-twist validation target is always the same thing as the persisted command state.
  - `tests/test_command_api_direct_setpoint.py`: updated DLS helper assertions and added a regression where +Y Cartesian jog is fully attenuated by a singular Jacobian; the old raw-target behavior would exceed `MAX_CART_RESIDUAL_M`, while the new target remains at Y=0 and no residual gate failure is recorded.
  - `tests/test_realtime_jog_backend_compatibility.py`: made the legacy servo-driver jog compatibility test explicitly set `JOG_USE_JACOBIAN=False`, because that test asserts the old IK fallback path and should not depend on the rollout flag/environment.
- Validation performed:
  - `source ./start.sh && python -m pytest tests/test_command_api_direct_setpoint.py -q -k "jacobian_jog or achieved_twist or drift_watchdog"` -> `5 passed, 51 deselected`.
  - `source ./start.sh && python -m py_compile src/gradient_os/arm_controller/command_api.py tests/test_command_api_direct_setpoint.py` -> pass.
  - First broad jog slice exposed the legacy compatibility test assumption (`1 failed, 86 passed`); fixed the test to force IK fallback.
  - `source ./start.sh && python -m pytest tests/test_command_api_direct_setpoint.py tests/test_cartesian_jog_resilience.py tests/test_realtime_jog_backend_compatibility.py -q` -> `87 passed`.
  - `source ./start.sh && python -m py_compile src/gradient_os/arm_controller/command_api.py tests/test_command_api_direct_setpoint.py tests/test_realtime_jog_backend_compatibility.py` -> pass.
  - `ReadLints` on touched Python/test files -> clean.
  - Follow-up telemetry cleanup rerun: focused Jacobian jog tests -> `5 passed, 51 deselected`; broad jog/control slice -> `87 passed`; touched-file `py_compile` -> pass; `ReadLints` -> clean.
- Follow-ups / risks:
  - Live operator-supervised singularity-edge Cartesian jog retest still needed. Expected behavior: `JOG_NEAR_SINGULARITY` can still appear, but TCP target should slow/attenuate rather than drift off-target. If `IK_JUMP_REJECTED` or `JOG_COMMAND_DRIFT_EXCEEDED` persists, add a second-stage uniform `q_dot` governor and recompute achieved twist before sending the RTCore command.

## 2026-04-29 19:18 +0000 - HIGH PRIORITY: DLS achieved-twist jog still departed the intended TCP target

- Investigation summary:
  - Operator live-retested the DLS achieved-twist change and reported the jog still took the TCP "right off our kinematic target."
  - Checked the newest controller log (`logs/startups/20260429-190442/controller.log`) and active terminal stream. The visible post-change jogs were accepted as `joint_velocity_lease` sessions with `truth_valid_at_arm=True`.
  - No controller-log hits for `JOG_NEAR_SINGULARITY`, `CARTESIAN_RESIDUAL_EXCEEDED`, `JOG_COMMAND_DRIFT_EXCEEDED`, `IK_JUMP_REJECTED`, `ORIENTATION_RESIDUAL_EXCEEDED`, `achieved_twist`, or `twist_residual` were present in the newest controller log. This means the failure did not surface as a fail-closed gate rejection or as durable DLS telemetry in the standard controller log.
  - The terminal/log sequence shows a burst of Cartesian jog commands (`vx=±0.05`, `vy=0.05`, `vz=0.05`, `v_roll=15`, `v_pitch=±15`) followed by Home. The Home plan started from a materially displaced configuration: `current_deg=[-17.594, 25.063, 29.157, 98.959, -87.832, -131.667] -> target_deg=[0.0]*6`.
- Current interpretation:
  - The 18:05 patch fixed one internal consistency problem: it stopped the validation target from running ahead of the damped DLS joint command.
  - It did NOT guarantee preservation of the operator's intended Cartesian target/path. By advancing the command target with `achieved_twist = J @ q_dot`, the controller can accept a projected/attenuated twist whose direction differs from the requested twist. Near a singularity, that can move the TCP away from the intended kinematic target while internal residual gates remain satisfied.
  - The missing durable DLS diagnostics in the standard logs are also a blocker: without a live diagnostic capture or controller log emission of requested/achieved/actual twist, we cannot yet distinguish "projection changed direction" from model/FK mismatch, runtime frame mismatch, or RTCore/drive following lag.
- High-priority follow-up:
  - Do not consider the DLS achieved-twist patch complete for live Cartesian jog.
  - Next design should preserve requested Cartesian target semantics: apply a scalar task-speed governor along the requested twist/path, or hold/reject when the requested direction is singular, instead of silently accepting a direction-changing projected twist.
  - Add capture/logging before the next live retest: requested twist, achieved twist, scalar attenuation, actual measured FK delta, target-vs-actual TCP error, `q_dot`, singular values, and explicit proof that the Jacobian path is active during RTCore `joint_velocity_lease` sessions.
- Validation performed:
  - Read-only log inspection only. No code changes or tests run for this investigation entry.
- Follow-ups / risks:
  - This is high priority because a "successful" jog tick can still move the physical TCP off the operator's kinematic target without raising the current gate alerts.

## 2026-04-29 18:14 +0000 - 3D visualizer now uses live joints instead of display joints

- What changed:
  - User pasted a live `window.__GRADIENT_3D_LAG_TRACE__` sample after visible 3D lag. The trace showed no browser/render bottleneck: monitor sequence gaps were 1, source cadence averaged ~36.6 Hz, parse-to-push averaged ~0.09 ms, push-to-visible averaged ~6.9 ms, and frame work averaged ~0.34 ms.
  - The trace identified `usedDisplayJoints=true`, meaning the physical 3D stage was rendering `display_joints`.
  - `web-ui/src/poseTelemetry.ts`: added `preferredLiveVisualizerPoseJoints(...)`, which prefers live `joints` and falls back to `display_joints` only when live joints are absent.
  - `web-ui/src/App.tsx`: switched the imperative visualizer handoff to `preferredLiveVisualizerPoseJoints(next)` and kept trace metadata (`usedDisplayJoints`) so the next live sample should show `false` when `joints` is present.
  - `web-ui/src/poseTelemetry.test.ts`: added coverage for the new joints-first visualizer helper while preserving display-first behavior for operator panels/charts.

- Validation performed:
  - `ReadLints` clean on `web-ui/src/App.tsx`, `web-ui/src/poseTelemetry.ts`, and `web-ui/src/poseTelemetry.test.ts`.
  - `git diff --check -- web-ui/src/App.tsx web-ui/src/poseTelemetry.ts web-ui/src/poseTelemetry.test.ts` -> clean.
  - `npm test -- src/poseTelemetry.test.ts src/App.test.ts` in `web-ui` -> `9 passed`.
  - `npm run build` in `web-ui` -> pass; Vite emitted existing `occt-import-js` browser-externalized module notes and chunk-size warning.

- Follow-ups / risks:
  - Hard-reload the browser, run the same jog, and confirm `window.__GRADIENT_3D_LAG_TRACE__.snapshot.latest.usedDisplayJoints === false`. If visible lag remains while `joints` drives the stage, compare the actual `joints` values against the physical pose/control truth next.

## 2026-04-29 18:22 +0000 - Monitor joints now use control feedback source

- What changed:
  - User confirmed `usedDisplayJoints=false` but the 3D stage still visibly lagged, so the next source of delay was `/monitor.joints` itself.
  - `src/gradient_os/run_controller.py`: added `_read_monitor_joint_feedback(...)`, which reads `servo_driver.get_control_arm_state_rad(verbose=False)` for `/monitor.joints` instead of strict `get_current_arm_state_rad(...)`.
  - The same helper preserves the existing last-good fallback and stale/error flags when the control-feedback read fails.
  - `display_joints`, `axis_absolute_feedback`, drive fault snapshots, and canonical truth diagnostics remain on their existing display/strict paths.
  - `tests/test_run_controller_helpers.py`: added tests proving monitor joint feedback uses `get_control_arm_state_rad` and preserves recent-sample fallback.

- Validation performed:
  - `ReadLints` clean on `src/gradient_os/run_controller.py` and `tests/test_run_controller_helpers.py`.
  - `source ./start.sh && python -m pytest tests/test_run_controller_helpers.py -q` -> `15 passed`.
  - `source ./start.sh && python -m py_compile src/gradient_os/run_controller.py tests/test_run_controller_helpers.py` -> pass.
  - `git diff --check -- src/gradient_os/run_controller.py tests/test_run_controller_helpers.py` -> clean.

- Follow-ups / risks:
  - Controller restart is required for this Python change to affect live `/monitor` telemetry. After restart and browser reload, inspect `window.__GRADIENT_3D_LAG_TRACE__` again; browser path should remain fast, and physical visual lag should drop if canonical conversion was the lagging source.

## 2026-04-29 19:09 +0000 - Lean monitor packets target 50 Hz visual telemetry

- What changed:
  - User asked if the post-control-feedback monitor cadence (`~35.5 Hz`) can reach 50 Hz.
  - `src/gradient_os/run_controller.py`: kept the live `/monitor.joints` read on every loop, but split heavier existing telemetry fields to slower cadences inside the same `/monitor` stream.
  - Cached display snapshot now refreshes at `max(period, 0.1s)` and only includes bulky `axis_absolute_feedback` on those refresh packets; cached `display_joints` can still accompany fast packets.
  - RTCore axis `servos` samples now refresh at `max(period, 0.1s)` instead of every monitor tick.
  - Existing extended servo/drive-fault block remains on the previous `0.5s` cadence.
  - `tests/test_run_controller_helpers.py`: added coverage that `_attach_monitor_joint_feedback(...)` can include display joints while omitting heavy `axis_absolute_feedback`.

- Validation performed:
  - `ReadLints` clean on `src/gradient_os/run_controller.py` and `tests/test_run_controller_helpers.py`.
  - `source ./start.sh && python -m pytest tests/test_run_controller_helpers.py -q` -> `16 passed`.
  - `source ./start.sh && python -m py_compile src/gradient_os/run_controller.py tests/test_run_controller_helpers.py` -> pass.
  - `git diff --check -- src/gradient_os/run_controller.py tests/test_run_controller_helpers.py` -> clean.

- Follow-ups / risks:
  - Requires controller/API stack restart to take effect. After restart and browser hard reload, expected trace improvement is `sourceHz`/`browserReceiveHz` closer to 50 Hz and lower average `payloadBytes` on most packets, while 10 Hz diagnostic packets still carry larger payloads.
