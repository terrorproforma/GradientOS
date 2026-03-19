## IgH EtherCAT Master (EtherLab) notes

This repo uses the **IgH / EtherLab EtherCAT Master** (kernel `ec_master` + user-space `libecrt`) as the
EtherCAT implementation for the RTCore daemon.

### Version + where it installs on the RevPi

- **IgH version**: `1.6.8` (built from source via `scripts/ethercat/install_igh.sh`)
- **CLI**: `/usr/local/bin/ethercat`
- **Service helper**: `/usr/local/sbin/ethercatctl`
- **C header**: `/usr/local/include/ecrt.h`
- **Library**: `/usr/local/lib/libethercat.so` (via pkg-config as `libethercat`)
- **systemd unit** (from IgH install): `/etc/systemd/system/ethercat.service`
- **Runtime config** (our canonical): `/etc/ethercat.conf`
  - We install a drop-in so `ethercat.service` uses `/etc/ethercat.conf`:
    `/etc/systemd/system/ethercat.service.d/10-gradient.conf`

Local “docs” installed by the IgH build are minimal on this image:
- **bash completion**: `/usr/local/share/bash-completion/completions/ethercat`
- No doxygen/man pages were found under `/usr/local/share/doc` or `/usr/share/man`.

### What `Active: no` means in `ethercat master`

`ethercat master` has two independent “running” concepts:

- **Kernel master thread**: the `ec_master` kernel module is loaded and has a device attached.
- **Active**: a user-space **libecrt application has activated the master** (e.g. RTCore).

So it is normal to see `Active: no` while doing **slave discovery** with the CLI.

### Binding the master to the correct NIC

`ethercatctl` reads `MASTER0_DEVICE` from the config file and accepts either:

- a **MAC address**, or
- an **interface name** (it resolves it to a MAC at runtime).

For one-off diagnostics (e.g. after physically swapping cables), use the repo’s temporary configs:

- `scripts/ethercat/ethercat-eth0.conf`
- `scripts/ethercat/ethercat-eth1.conf`

Example:

```bash
sudo systemctl stop ethercat.service
sudo /usr/local/sbin/ethercatctl -c ~/GradientOS/scripts/ethercat/ethercat-eth1.conf start
sudo ethercat master
sudo ethercat slaves -v
```

### Device driver notes (generic vs native)

IgH supports:

- `generic`: works with “any” NIC (captures frames via a raw socket)
- native `ec_*` drivers: can be lower overhead if a matching `ec_<driver>` exists for your NIC

On this RevPi the two RJ45 NICs use different Linux drivers (check with `ethtool -i ethX`):

- `eth0`: `macb`
- `eth1`: `lan743x`

Example:

```bash
$ sudo ethtool -i eth0
driver: macb
bus-info: 1f00100000.ethernet

$ sudo ethtool -i eth1
driver: lan743x
bus-info: 0001:03:00.0
```

IgH is configured to use `DEVICE_MODULES="generic"`. Earlier notes recorded a working setup on `macb`
(`eth0`) and a failing setup on `lan743x` (`eth1`), but current live validation on this same RevPi with the
present cable layout proved the slave chain is reachable on `eth1` (`lan743x`, MAC `c8:3e:a7:14:1c:76`):

- temporary `ethercat-eth1.conf` showed `Slaves: 6`, non-zero RX frames, and zero lost frames,
- the current host also uses `eth0` as its normal wired uplink,
- so the default master binding was updated to `eth1` for the current appliance wiring.

Conclusion: treat the older `eth0` note as historical, not universal. Trust the live probe for the current
machine/wiring.

### Upstream references (canonical)

- IgH EtherCAT Master upstream: [`https://gitlab.com/etherlab.org/ethercat`](https://gitlab.com/etherlab.org/ethercat)
- 1.6 FEATURES: [`https://gitlab.com/etherlab.org/ethercat/-/blob/stable-1.6/FEATURES.md`](https://gitlab.com/etherlab.org/ethercat/-/blob/stable-1.6/FEATURES.md)
- Device driver guidance: [`https://docs.etherlab.org/ethercat/1.6/doxygen/devicedrivers.html`](https://docs.etherlab.org/ethercat/1.6/doxygen/devicedrivers.html)
- Doxygen data structures index: [`https://docs.etherlab.org/ethercat/1.6/doxygen/annotated.html`](https://docs.etherlab.org/ethercat/1.6/doxygen/annotated.html)

