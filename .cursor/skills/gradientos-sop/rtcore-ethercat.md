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

## Startup Verification

- Enforce and read back startup drive config through generic `startup_drive_config` telemetry.
- Keep the transport generic even when the underlying setting is profile-specific.
- Include human-readable labels when the profile can supply them.

## Fault Telemetry

- Carry both generic bus faults and manufacturer faults when the drive exposes both.
- Decode manufacturer faults in profile/backend-specific code, not generic OS layers.
- Reuse existing status and telemetry payloads rather than inventing a second fault channel.

## Safe Enable and Power

- Synchronize targets with feedback before enabling.
- STOP and power transitions must not re-inject stale motion into RTCore.
- Preserve the no-sudden-move contract across restart, enable, and recovery flows.

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
