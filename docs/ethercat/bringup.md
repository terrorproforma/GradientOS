## RTOS/EtherCAT bring-up (RevPi appliance)

This is the practical bring-up checklist for the RTOS/EtherCAT path described in:
- `RTOS-ETHERCAT-PLAN/RTOS-ETHERCAT-plan.md` (authoritative)

### CPU / RT assumptions (RevPi Connect 5)

- 4 physical cores
- Target partitioning:
  - **CPU0–CPU1**: housekeeping + non-RT helper work
  - **CPU2–CPU3**: RT cyclic loop + EtherCAT IRQ threads

### Phase A/B: Host prerequisites (scripts in-repo)

From the repo root:

```bash
cd systemd/ethercat-host
./install.sh
```

Optional but recommended (adds CPU isolation parameters to `/boot/firmware/cmdline.txt`):

```bash
cd systemd/ethercat-host
sudo ./rtos-apply-cmdline.sh
sudo reboot
```

After reboot, verify:

```bash
./scripts/ethercat/diagnose_host.sh
```

### Kernel headers (required to build IgH modules)

IgH includes kernel modules, so you need headers for the **running** RT kernel:

```bash
uname -r
ls -l "/lib/modules/$(uname -r)/build"
```

If missing, install the matching headers package (distro/vendor specific). Common attempts:

```bash
sudo apt-get update
sudo apt-get install -y "linux-headers-$(uname -r)"
```

On RevPi images, the header package name may differ; use `apt-cache search` on your configured repos.

### Phase C: Install IgH (EtherLab) master

IgH deliverables needed:
- kernel modules: `ec_master`, `ec_generic` (generic NIC binding)
- user-space: `libecrt` + `ethercat` CLI

Recommended: use the repo script to build/install a pinned IgH version:

```bash
./scripts/ethercat/install_igh.sh
```

IgH background notes (where things install, meaning of `Active: no`, NIC binding tips):
- `docs/ethercat/igh.md`

Where IgH installs artifacts (host OS):
- `ethercat` CLI: `/usr/local/bin/ethercat`
- `ecrt.h`: `/usr/local/include/ecrt.h`
- `libecrt`/`libethercat`: `/usr/local/lib/libethercat.so` (+ pkg-config metadata)
- systemd unit (from IgH install): `/etc/systemd/system/ethercat.service`
- kernel modules (after `make modules_install`): `/lib/modules/$(uname -r)/ethercat/**/ec_*.ko.xz`

Repo-owned config/templates:
- EtherCAT master binding config template: `systemd/ethercat-host/ethercat.conf` → installed to `/etc/ethercat.conf`
- Override to force `ethercat.service` to use `/etc/ethercat.conf`:
  `systemd/ethercat-host/ethercat.service.d/10-gradient.conf`

Bind the master to the EtherCAT port by MAC in `/etc/ethercat.conf`:

```bash
sudo cat /etc/ethercat.conf
```

Expected:
- `MASTER0_DEVICE="c8:3e:a7:14:1c:76"`
- `DEVICE_MODULES="generic"`

After installation, verify:

```bash
sudo systemctl enable --now ethercat.service
ethercat slaves -v
ethercat pdos
ethercat master
```

### Monitoring / verification (post-reboot)

Quick host sanity checks:

```bash
./scripts/ethercat/diagnose_host.sh
```

Key signals:
- `/proc/cmdline` contains `isolcpus=2,3 nohz_full=2,3 rcu_nocbs=2,3 irqaffinity=0,1`
- `/sys/devices/system/cpu/isolated` shows `2-3` (or equivalent)
- `ethercat master` shows Link UP and (once slaves exist) Rx frames > 0 and frame loss near 0
  - Note: `Active: no` is normal here (means no libecrt application is running yet).

### No-motion comms validation (IgH master ↔ slave)

Prerequisites:
- Drive is connected RevPi `eth1` (driver `lan743x`) → drive `CN3 (IN)` and is powered.
- `ethercat.service` binds to `/etc/ethercat.conf` (MAC `c8:3e:a7:14:1c:76`).

Run:

```bash
sudo systemctl restart ethercat.service

# 1) Link-layer comms health
sudo ethercat master

# Expect: Slaves > 0, Rx frames > 0, Frame loss near 0

# 2) Slave identity + mailbox comms
sudo ethercat slaves -v

# 3) PDOs: what the drive supports vs what is currently selected
sudo ethercat pdos -p0  # shows SII/fixed sets (may not reflect the current selection)

# Confirm the *selected* RxPDO/TxPDO via CoE assignment objects:
sudo ethercat upload -p0 -t uint16 0x1c12 0x01  # expect 0x1702
sudo ethercat upload -p0 -t uint16 0x1c13 0x01  # expect 0x1b02

# If needed (e.g. after a drive power-cycle), select 0x1702/0x1B02 (must be PREOP):
# sudo ethercat download -p0 -t uint8  0x1c12 0x00 0
# sudo ethercat download -p0 -t uint16 0x1c12 0x01 0x1702
# sudo ethercat download -p0 -t uint8  0x1c12 0x00 1
# sudo ethercat download -p0 -t uint8  0x1c13 0x00 0
# sudo ethercat download -p0 -t uint16 0x1c13 0x01 0x1b02
# sudo ethercat download -p0 -t uint8  0x1c13 0x00 1

# 4) Optional: read common CoE identity strings (device name / hw / sw)
sudo ethercat upload -p0 -t string 0x1008 0x00
sudo ethercat upload -p0 -t string 0x1009 0x00
sudo ethercat upload -p0 -t string 0x100A 0x00
```

### A6-EC EtherCAT specifics (manual ch8–10)

- **Physical layer / ports**: `100BASE-TX`, full duplex, linear topology, RJ45*2 (`IN`, `OUT`). Use the drive’s EtherCAT **`IN`** for the uplink from the master. Use Cat5 shielded (or Cat6+) and keep <100 m between nodes.
- **Connector naming**: manual calls these EtherCAT ports **CN3 (IN)** and **CN4 (OUT)**. For a single-drive test, connect RevPi → **CN3 (IN)** and leave CN4 unused.
- **Protocol**: CoE with **IEC 61800-7 / CiA402** drive profile; supports PP/PV/PT/HM/CSP/CSV/CST.
- **EtherCAT state machine**: Init → Pre-Op → Safe-Op → Op (no skipping on the way up).
- **DC / SYNC0**: drive supports **DC sync only**. Sync cycle must be an integer multiple of **250 μs** (else `Er74.0`). Missing/incorrect sync config can show `Er74.1` / `Er74.2`.
- **Fixed PDOs used for most bring-up** (verify with `ethercat pdos`):
  - **RxPDO `0x1702` (Outputs)**: `6040` control word, `607A` target position, `60FF` target velocity, `6071` target torque, `6060` mode selection, `60B8` touch probe function, `607F` max speed.
  - **TxPDO `0x1B02` (Inputs)**: `603F` fault code, `6041` status word, `6064` position actual value, `6077` torque feedback, `6061` mode display, `60B9/60BA/60BC` touch probe status/edges, `60FD` DI status.
- **PDO assignment objects**: `0x1C12:01` selects the active RPDO (`0x1600` or `0x1701..0x1705`), and `0x1C13:01` selects the active TPDO (`0x1A00` or `0x1B01..0x1B04`).
- **PDO reconfiguration gotcha**: mapping edits are only allowed in **Pre-Operational** and **aren’t stored in EEPROM** (must be re-applied after a power cycle).
- **Drive-side faults that often point to comms/config issues**:
  - `Er31.0`: too many PDO mapping objects (>10)
  - `Er32.1/Er32.5`: XML/ESI issues (check XML version `U42.0B`, re-program)
  - `Er74.0/Er74.1/Er74.2`: DC/SYNC issues
  - `Er87.1/Er87.2`: target position steps too large (align target to feedback before enabling / switching mode; keep increments small)
- **DI/DO note**: `S-ON` and `ALM-RST` DI functions are **only active in non-bus control mode**; in EtherCAT mode rely on DS402 `6040` (enable/quickstop/fault reset). DI status is available via `60FD` in the TxPDOs above.
- **Brake wiring note**: manual shows motor brake control uses DO3 (`BK+ / BK-`) and an **external 24V brake supply**; the brake coil has **no polarity**. This CN1 wiring does **not** affect EtherCAT slave discovery.

### Current bring-up status (notes)

If `ethercat master` shows:
- Link: UP
- Slaves: 0
- Rx frames: 0
- Frame loss: 100%

…then the master is transmitting but **getting no replies**. Common causes:
- cable is plugged into the wrong RJ45 on the drive (not EtherCAT, or OUT vs IN)
- the drive is not in EtherCAT mode / EtherCAT interface not enabled
- the drive is not actually on the dedicated EtherCAT port
- the IgH master is **bound to the wrong NIC** for the current wiring (check `ethercat master` → `Main:` MAC)

Extra sanity checks:
- `ip -s link show dev ethercat0` (or `eth1` before NIC renaming takes effect) should show **RX packets > 0** once a slave is responding.
- `sudo ethtool -S ethercat0` (or `eth1` pre-rename) should show **RX frames > 0** and should *not* show TX errors climbing 1:1 with TX frames. If RX stays at 0 and TX carrier errors climb with each frame, suspect **cable/port/device not replying** (not a PDO/DS402 issue).

Suggested first checks:

```bash
sudo ethercat rescan
sudo ethercat slaves -v
sudo ethercat pdos
```

If you temporarily swapped which RevPi port goes to the drive, you can bind IgH to the other interface for
diagnosis without editing `/etc/ethercat.conf`:

```bash
sudo systemctl stop ethercat.service
sudo /usr/local/sbin/ethercatctl -c ~/GradientOS/scripts/ethercat/ethercat-eth1.conf start
sudo ethercat master
sudo ethercat slaves -v
```

### Phase D+: RTCore + Python integration

- RTCore source: `src/gradient_rt_motion/` (binary: `gradient-rt-motion`)
- Python backend proxy: `src/gradient_os/arm_controller/backends/ethercat_rtcore/`

For the current **two-drive bring-up**, run RTCore with 2 axes:

```bash
make -C src/gradient_rt_motion
# Run RTCore as root (EtherCAT + RT privileges). By default it makes the IPC
# socket group-owned by `pi` (mode 0660) so the controller can connect.
sudo ./src/gradient_rt_motion/gradient-rt-motion --num-axes 2
```

Axis scaling defaults (v1):
- `--counts-per-rev 131072` (or comma list per axis, e.g. `131072,131072`)
- `--gear-ratio 1.0` (or comma list per axis, e.g. `100,50`)
- `--sign +1` (or comma list per axis, e.g. `+1,-1`)
- `--axis-type rotary` (or comma list per axis, `rotary,linear`; rotary q=rad, linear q=m)
- `--lead-m-per-rev M` (only for linear axes; can be a comma list)
- `--max-rpm 100` (safety cap; clamps per-cycle position steps and programs `0x607F` max profile velocity; `0` disables clamping)

Override as needed during commissioning, e.g.:

```bash
sudo ./src/gradient_rt_motion/gradient-rt-motion --num-axes 2 --gear-ratio 100
```

Per-axis example (axis0 has 100:1, axis1 has 50:1 and inverted sign):

```bash
sudo ./src/gradient_rt_motion/gradient-rt-motion --num-axes 2 --gear-ratio 100,50 --sign +1,-1
```

The Python backend defaults to mapping those 2 RT axes to **joint 3 and joint 4**
(`GRADIENT_RTCORE_CONTROL_JOINTS="3,4"` is the explicit override).

#### Motion test (p0 only)

If you want to do an initial motion test on **only the first slave** (EtherCAT position `p0` / RTCore `axis0`),
set an auto-arm mask so the controller only enables that axis on connect:

```bash
# Only enable RTCore axis 0 (EtherCAT p0). Axis 1 (p1) remains disabled.
export GRADIENT_RTCORE_AUTO_ARM_MASK=0x1

# Optional: map RT axes -> logical joints (1-based). Example:
#   axis0 -> J5, axis1 -> J3
export GRADIENT_RTCORE_CONTROL_JOINTS="5,3"

# Start the GradientOS controller using the EtherCAT RTCore backend.
./run.sh --servo-backend ethercat_rtcore
```

Then use the UI or `scripts/udp_client.py` to send a **very small** joint move on the joint mapped to `axis0`
(start with ~0.01 rad and increase slowly). RTCore clamps speed to `--max-rpm` and ignores setpoints for disabled axes.

#### Sending motion commands (quickest path)

If the controller is running and you see:
- `[Controller] Listening for UDP packets on 0.0.0.0:3000`

…you can command motion by sending **6 joint angles (radians)** over UDP.

From the RevPi itself:

```bash
# Interactive client (type a 6-value comma list, e.g. "0,0,0,0,0.01,0")
./.venv/bin/python scripts/udp_client.py --pi-ip 127.0.0.1
```

Example: if you mapped `axis0 -> J5` and want a tiny move on that axis:

```bash
# j1,j2,j3,j4,j5,j6 (radians)
0,0,0,0,0.01,0
```

#### Capturing joint zero offsets

Once a joint is physically placed at the pose you want to treat as logical zero,
capture that offset through the controller API:

```bash
curl -X POST http://127.0.0.1:8000/control/zero-joint \
  -H "Content-Type: application/json" \
  -d '{"joint": 5}'
```

Notes:
- This stores a **logical joint master offset** in repo-local state, it does **not**
  write a vendor-specific zero into the EtherCAT drive EEPROM.
- Offsets persist in `.gradient_joint_zero_offsets.json` and are reloaded by the
  `ethercat_rtcore` backend on controller startup.
- Zero joints one at a time after axis mapping/sign/scaling are confirmed with
  `scripts/rtcore_jog.py` and before aggressive PID tuning or trajectory execution.

#### Drive-native home vs software zero

For A6-EC absolute encoder commissioning there are now two distinct operations:

- `POST /control/home-joint-native` tells RTCore to set the drive's native home by
  rewriting the A6-EC position offset (`0x60B0`) for that axis and saving it to the
  drive (`0x1010:01`).
- `POST /control/zero-joint` keeps the existing GradientOS software-zero workflow by
  storing a logical offset in `.gradient_joint_zero_offsets.json`.

Recommended order:

1. Verify axis mapping, sign, and scaling first.
2. Use `Drive Home` / `/control/home-joint-native` when you want the servo drive's
   own absolute reference updated.
3. Use `Zero Joint` / `/control/zero-joint` only when you want to redefine the
   robot's logical work zero without changing the drive's native reference.

Example native-home call:

```bash
curl -X POST http://127.0.0.1:8000/control/home-joint-native \
  -H "Content-Type: application/json" \
  -d '{"joint": 5}'
```

#### A6-EC absolute multi-turn mode verification

The active EtherCAT drive family is now selected through the runtime drive profile,
not the robot config. Manufacturer-specific RTCore load details such as slave
identity, PDO defaults, and startup-setting defaults live in the separate
EtherCAT drive catalog under `src/gradient_os/arm_controller/ethercat_drive_catalog.py`.

For the current `a6ec_ds402` profile, RTCore configures each A6-EC axis for the
battery-backed limited multi-turn absolute encoder mode on startup via `C00.07`
(`0x2000:08`, mode value `1`). RTCore also reads the value back and publishes
whether the commanded mode verified successfully.

To inspect the live startup verification snapshot:

```bash
curl http://127.0.0.1:8000/health/drive-faults
```

Look for:

- `startup_drive_config`
- `startup_drive_config.setting_key`
- `startup_drive_config.commanded`
- `startup_drive_config.readback`
- `startup_drive_config.verified`
- `manufacturer_error_code` / decoded `manufacturer_fault`

The generic `startup_drive_config` object is now the primary telemetry contract from
RTCore through the Python controller and UI.

These fields help catch startup-mode mismatches and battery/multi-turn encoder faults
such as `Er20.8` and `Er20.9` after power-up.

#### Encoder retention experiment logging

To verify that absolute multi-turn counts survive a full drive power cycle, capture a
snapshot before power-down and another after power-up. GradientOS writes the results
under `logs/encoder-retention/<experiment-id>/`.

API examples:

```bash
curl -X POST http://127.0.0.1:8000/control/encoder-retention/capture \
  -H "Content-Type: application/json" \
  -d '{"phase": "before_power_down"}'
```

```bash
curl -X POST http://127.0.0.1:8000/control/encoder-retention/capture \
  -H "Content-Type: application/json" \
  -d '{"phase": "after_power_up", "experiment_id": "20260405-absolute-check"}'
```

Each experiment directory contains:

- `before_power_down.json`
- `after_power_up.json`
- `comparison.json`
- `comparison.md`

The comparison highlights:

- raw encoder count mismatches,
- logical joint-angle mismatches,
- startup absolute-mode verification failures,
- active battery/multi-turn faults after power-up.

#### Direct RTCore jog tool (no controller)

For bring-up and slave mapping, you can talk directly to RTCore without the full controller:

```bash
# In one terminal (root): start RTCore
sudo ./src/gradient_rt_motion/gradient-rt-motion --num-axes 2 --max-rpm 100

# In another terminal (pi): use a single RTCore connection (RTCore is single-client today)
python3 scripts/rtcore_jog.py console --rate-hz 2

# Then type:
#   arm 0x1
#   jog 0 0.01
```

More details: `docs/ethercat/rtcore_jog.md`.

#### Live monitoring dashboard (Sampler)

RTCore writes a small metrics file at:
- `/run/gradient-rt-motion/metrics.json`

You can view a live dashboard (RTCore Hz + jitter, per-core CPU usage, timer jitter on an isolated CPU vs non-isolated CPU)
using the Sampler config in:
- `scripts/sampler/rtos_monitor.yml`

See: `scripts/sampler/README.md`.

#### Safe power-cycle validation (no motion commands)

For the RTCore drive-power safety contract, the commissioning validation should
use a no-motion enable/disable cycle and verify both the hardware probe and the
controller motion-status contract.

From the repo root:

```bash
# 1) Full shutdown: controller/API/web + RTCore + EtherCAT
./start-stack.sh stop --hard

# 2) Start controller + API only (leave web UI out of the test)
./start-stack.sh --headless

# 3) Confirm the stack comes back disarmed
./start-stack.sh probe
curl http://127.0.0.1:4000/control/motion-status

# Expect before power-up:
# - physical_state=BUS_UP_DISARMED
# - driver_state=DISARMED
# - all axes SwitchOnDisabled
# - /control/motion-status => state=idle, active_traj_id=0, queue_depth=0,
#   safe_for_power_transition=true

# 4) Explicitly enable drives
curl -X POST http://127.0.0.1:4000/control/power-up \
  -H "Content-Type: application/json" \
  -d '{}'

# 5) Verify enabled state
./start-stack.sh probe

# Expect after power-up:
# - physical_state=ACTIVE
# - armed=1
# - enable_mask=0x3f
# - op_enabled_axes=6/6
# - all drive error codes remain 0x0000

# 6) Send NO motion commands

# 7) Explicitly power down using the safe path
curl -X POST http://127.0.0.1:4000/control/power-down \
  -H "Content-Type: application/json" \
  -d '{"wait_for_idle": true}'

# 8) Verify the stack returns to a disarmed-safe state
./start-stack.sh probe
curl http://127.0.0.1:4000/control/motion-status

# Expect after power-down:
# - physical_state=BUS_UP_DISARMED
# - driver_state=DISARMED
# - op_enabled_axes=0/6
# - /control/motion-status => idle, active_traj_id=0,
#   safe_for_power_transition=true

# 9) Return the host to a fully inactive state when done
./start-stack.sh stop --hard
```

Important:

- Do not send `MOVE_*`, `ROTATE`, `SET_ORIENTATION`, `APPLY_JOINT_SETPOINT`, or jog commands during this validation.
- After the 2026-03-24 fix, RTCore-backed `STOP` intentionally skips the legacy
  "hold current position" write so safe power-down does not accidentally create
  a one-point RTCore trajectory and relatch `active_traj_id`.

