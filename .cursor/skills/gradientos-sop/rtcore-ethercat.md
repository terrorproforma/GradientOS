# RTCore and EtherCAT

Primary SOP sections: `§4`, `§9-11`, `§13.1-13.4`

Use this file when changing `gradient-rt-motion`, EtherCAT host setup, drive bring-up, startup telemetry, or drive-fault handling.

## Core Rules

- RTCore owns the active EtherCAT master behavior; the controller talks to it through IPC only.
- Keep EtherCAT bring-up descriptor-driven, not vendor-hardcoded in RTCore.
- Startup readiness must reflect configured expectations, not optimistic observations.
- Bus-up does not mean armed; startup must settle into a safe disarmed state first.

## Drive-Family Integration

- Put slave identity, sync indices, PDO layouts, DC timing, and startup SDO defaults in the EtherCAT drive catalog and profile layer.
- Keep manufacturer-specific decode/labels in drive-profile code.
- Keep generic RTCore and generic Python telemetry vendor-neutral.
- If a future drive needs a different object map, add descriptors and profile data instead of adding vendor branches to `main.cpp`.
- Treat the configured PDO layout as safety-critical truth, not a convenience string.
- Never add a drive-specific object to a cyclic PDO descriptor unless the drive's actual assigned PDO map includes it.
- For A6-EC specifically, fixed TxPDO `0x1B02` does not include `0x203F`; forcing `manufacturer_err` into that layout shifts later offsets and corrupts cyclic feedback.
- When vendor-specific EtherCAT or commissioning semantics are unclear, consult the in-repo A6-EC references first: `docs/resources/A6-EC_series_servo_drive_manual.pdf`, `docs/resources/a6ec_manual_codes.json`, and `docs/resources/a6ec_manual_codes.md`.

## PDO Layout Guardrails

- One wrong entry in a PDO descriptor can keep EtherCAT apparently healthy while making RTCore read the wrong bytes.
- The dangerous signature is: slaves reach `OP`, but `statusword`, `pos_counts`, and DS402 state look implausible or flat-zero across axes.
- For A6-EC, inserting `0x203F` into cyclic `0x1B02` shifted `0x6041`, `0x6064`, and later fields by 4 bytes and made the stack falsely think the drives were `NotReady`.
- When debugging bring-up, verify both:
  - the configured PDO assignment (`0x1702/0x1B02` here)
  - the exact ordered layout inside that PDO, not just the PDO index names
- If manufacturer fault detail is needed but is not in the cyclic PDO, fetch it through a separate non-RT path instead of breaking the PDO layout.
- ESI `PdoMapping=t` is NOT a promise that the drive populates the subitem cyclically. The A6-EC advertises `U40.20 / U40.22` as PDO-mappable in its DT2040 datatype but empirically the firmware does NOT update those bytes when the subitems are included in a custom TxPDO — the drive accepts the mapping but sends zeros, even when other `0x6064`-family subitems in the same frame populate correctly. Verify wire-level population (not just SDO acceptance of the PDO assignment) before relying on any custom-mapped vendor subitem.

## Asymmetric-Rate Atomic Snapshots

When the controller needs to compare a PDO value (cyclic, fast) with an SDO value (mailbox, slow) — e.g., multi-turn consistency on A6-EC — RTCore owns pairing the two samples because only it has nanosecond access to both at the same moment.

Pattern (applied to the A6-EC `U40.20 / U40.22` vs `0x6064` comparison):

- `AbsoluteFeedbackAxis::paired_pos_counts` (in `src/gradient_rt_motion/main.cpp`) latches live `0x6064` immediately BEFORE each axis's multi-turn SDO read, alongside `paired_valid` and `paired_sample_time_ns`.
- The latch runs inside the `perform_absolute_feedback_refresh` loop specifically on the first multi-turn subindex encountered (`U40.21 / U40.23 / U40.2B / U40.2D`), not at end-of-loop, to keep skew at `~1-5 ms` instead of `~40-160 ms` of inter-field SDO transit.
- `append_absolute_feedback_json` publishes `paired_pos_counts` + `paired_sample_time_ns` alongside the other fields so the Python consumer picks them up with the existing `_AbsoluteFeedbackAxisMetrics` parser (no schema break).
- Python's canonical-truth gate reads `metrics.paired_pos_counts()` and uses that as the reference for the shaft-frame comparison; falls back to live-now `0x6064` only when `paired_valid = 0` (pre-paired window).
- See `RTOS_ETHERCAT_MASTER_OPERATING_PRINCIPLES.md §9.5A` for the full rationale and the diagnostic-surface contract.

When adding a similar cross-fieldbus invariant for a different drive family, do the pairing in RTCore, not in Python. IPC + metrics.json latency blur the sampling moment for any Python-side approach.

## Startup Verification

- Enforce and read back startup drive config through generic `startup_drive_config` telemetry.
- Keep the transport generic even when the underlying setting is profile-specific.
- Include human-readable labels when the profile can supply them.

## Fault Telemetry

- Carry both generic bus faults and manufacturer faults when the drive exposes both.
- Decode manufacturer faults in profile/backend-specific code, not generic OS layers.
- Reuse existing status and telemetry payloads rather than inventing a second fault channel.
- "Carry both" does not mean "force both into the cyclic PDO layout". Keep the acquisition path honest to the drive's real object map.

## Safe Enable and Power

- Synchronize targets with feedback before enabling.
- STOP and power transitions must not re-inject stale motion into RTCore.
- Preserve the no-sudden-move contract across restart, enable, and recovery flows.
- Drive-facing CSP hold/output/enable targets stay in the raw PDO wire frame (`0x6064` / `0x607A` counts).
- Target counts for A6-EC are chosen with a stateless per-write nearest-turn fold against live `0x6064` (`abs(target − live_6064) <= RM / 2`). Do not carry cached wrap-lift state between writes, and do not re-apply any persisted home offset twice.

## Commissioning Transaction Preconditions

- RTCore `MSG_CMD_NATIVE_HOME` is a two-stage transaction:
  - Stage A: clear `axis_enable_mask` / `armed` / motion intent, then poll each targeted axis's `statusword` until it has left `OperationEnabled` / `QuickStopActive` for several consecutive cycles within a bounded budget (current implementation: `~500 ms`, `3` stable cycles).
  - Stage B: run the existing descriptor-driven HM35 transaction unchanged.
- Stage-A timeout uses a synthesized abort code in the reserved `0xFxxxxxxx` RTCore-side range (currently `NATIVE_HOME_ABORT_DISARM_PRECONDITION_TIMEOUT = 0xF1000001`). Real CoE SDO aborts live in `0x05xxxxxx`–`0x08xxxxxx`; the ranges must not collide. New synthesized codes should follow the same convention.
- Controller-side, `prepare_for_power_transition` accepts `require_drive_disarmed=True` and `require_drive_disarmed_axis_mask=<mask>`. When set, the neutrality test is `motion_intent_cleared AND per_axis_drive_disarmed[axis]` for every targeted axis, sourced from the DS402 state of the live statusword snapshot. Any commissioning transaction that requires a disarmed drive (currently HM35) must use this stronger wait.

## Trajectory Upload Sanity

- The Python backend rejects trajectory uploads whose consecutive-point `607A` step exceeds `0.5 * RM` on any targeted axis (`command_frame_oversized_step`). This is a host-side frame-sanity fence that catches wrong-turn command math before the fieldbus sees it.
- RTCore still enforces `max_step_counts_per_cycle` per cycle. These gates are complementary: the host gate prevents an entire trajectory from being a wrong-turn error; RTCore's per-cycle clamp prevents runaway motion within a single legitimate trajectory.

## Commissioning and Bring-Up

- Distinguish slave discovery from motion readiness.
- Validate host NIC binding and RTCore readiness separately.
- Prefer focused bring-up checks over assuming the full stack implies fieldbus health.

## First Files

- `src/gradient_rt_motion/main.cpp`
- `src/gradient_rt_motion/ipc_v1.hpp`
- `src/gradient_os/arm_controller/ethercat_drive_catalog.py`
- `src/gradient_os/arm_controller/profiles/drive/`
- `src/gradient_os/arm_controller/backends/ethercat_rtcore/runtime.py`
- `src/gradient_os/telemetry/drive_faults.py`
- `docs/ethercat/bringup.md`
- `systemd/rt-motion/gradient-rt-motion.service`
