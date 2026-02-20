---
description: "Entry point for RTOS/EtherCAT work. Tells agents what to read and the non-negotiable architecture decisions."
alwaysApply: true
---

## Canonical docs (read first)

- High-level overview + AI to-do: `RTOS-ETHERCAT-PLAN/Plan overview & to-do.md`
- Full specification: `RTOS-ETHERCAT-PLAN/RTOS-ETHERCAT-plan.md`
- Living “unknowns” checklist: `RTOS-ETHERCAT-PLAN/Uncertainties.md` (keep this up to date during bring-up)
- IgH/EtherLab notes + upstream links: `docs/ethercat/igh.md`

## Required memory context (always)

- Scratchpad skill: `.cursor/skills/learning-scratchpad-loop/SKILL.md`
- Scratchpad template: `.cursor/skills/learning-scratchpad-loop/references/scratchpad-template.md`
- Scratchpad file: `.cursor/memory/AGENT_SCRATCHPAD.md`
- Devlog skill: `.cursor/skills/devlog-loop/SKILL.md`
- Devlog template: `.cursor/skills/devlog-loop/references/devlog-entry-template.md`
- Devlog file: `.cursor/memory/DEVLOG.md`

If instructions conflict, the full specification wins.

## Non-negotiable architecture (v1)

- **Host**: RevPi Connect 5 (Linux + PREEMPT_RT), treated as an appliance (freeze after validation).
- **EtherCAT master**: **IgH / EtherLab** (`ec_master` kernel module + `libecrt` user-space API).
- **Timing**: DC/SYNC0 required; target **1 kHz** cycle.
- **RTCore language**: **C++17 user-space daemon** named `gradient-rt-motion` linking `libecrt`.
- **Separation**:
  - Python = command plane (API/UI/planning/trajectory generation).
  - RTCore = motion plane (EtherCAT/DC/DS402/brakes/watchdogs).
  - Python must never be in the 1 kHz critical path.

## NIC assignment (locked)

- `uplink0` (front RJ45, currently `eth1`, has IP): `c8:3e:a7:14:1c:76`
- `ethercat0` (front RJ45, currently `eth0`, dedicated EtherCAT): `c8:3e:a7:14:1c:75`
- PiBridge NICs (not used for motion): `pileft` `c8:3e:a7:14:1c:77`, `piright` `c8:3e:a7:14:1c:78`

If you temporarily swap physical cables during diagnosis, remember:
- IgH binds to `MASTER0_DEVICE` from `/etc/ethercat.conf` (MAC or interface name).
- `ethercat master` showing `Active: no` is normal until RTCore (libecrt app) activates it.
- For one-off tests, you can bind IgH without editing `/etc/ethercat.conf` via:
  `sudo /usr/local/sbin/ethercatctl -c ~/GradientOS/scripts/ethercat/ethercat-eth0.conf start`

## Working style for small “fresh context” chunks

When implementing a chunk:
- State which **Phase** (from `Plan overview & to-do.md`) you’re implementing.
- Only touch the files relevant to that phase.
- Add/adjust systemd units/scripts in a minimal, testable way.
- Do not weaken safety: missing topology/DC/WKC must prevent arming.
