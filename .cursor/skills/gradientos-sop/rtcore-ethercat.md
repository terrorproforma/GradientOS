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
- For A6-EC with persisted native home in `0x607C`, drive-facing CSP hold/output/enable targets must stay in the raw PDO wire frame (`0x6064` / `0x607A` counts).
- Subtract `native_home_position_offset` only when converting queued controller/logical targets into raw CSP wire counts; do not subtract it again when mirroring live feedback into hold targets or seeding realtime jog accumulators.

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
