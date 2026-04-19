# Systemd Units

GradientOS ships helper units and scripts for running core services under systemd.
Both subdirectories include a `.service` file plus convenience scripts for
installing, removing, and managing the units.

- `controller/` – wraps `gradient-controller` via `run.sh`, intended for the
  robotic arm host (Raspberry Pi). Use:
  ```bash
  cd systemd/controller
  ./install.sh        # copies arm-controller.service into /etc/systemd/system
  ./status.sh         # inspect current state
  ./restart.sh        # restart the controller service
  ./stop.sh
  ./uninstall.sh
  ```
  Adjust the unit if you need custom environment variables (e.g. `SERIAL_PORT`).
  The controller unit now `Wants=` / `After=` `gradient-rt-motion.service`, so a
  normal controller service start will also pull in RTCore. RTCore in turn
  `Requires=` `ethercat.service`, giving a single service chain for the EtherCAT
  hardware stack once all units are installed.

- `api/` – runs `gradient-api` to expose the FastAPI REST/SSE proxy. Scripts are
  analogous:
  ```bash
  cd systemd/api
  ./install.sh
  ./status.sh
  ./restart.sh
  ./stop.sh
  ./uninstall.sh
  ```
  The service binds to `0.0.0.0:4400` by default and uses the project virtualenv.
  (Historic note: was `4000`; moved because Windows `iphlpsvc` grabs port 4000
  dynamically and breaks Cursor Remote-SSH port forwarding. Override with
  `GRADIENT_API_PORT` via the unit if needed.)
  Override environment variables inside the unit if you host the controller on a
  different machine.

- `rt-motion/` – runs the **RTCore** (`gradient-rt-motion`) daemon that will own
  EtherCAT + the 1kHz cyclic loop (RTOS/EtherCAT architecture work). Use:
  ```bash
  cd systemd/rt-motion
  ./install.sh
  ./status.sh
  ./restart.sh
  ./stop.sh
  ./uninstall.sh
  ```
  Note: the unit is wired to `Requires=ethercat.service` (IgH master). Until
  IgH is installed/configured on the host, this service will not start.

- `ethercat-host/` – host-level EtherCAT prerequisites (NIC renaming, unmanaged
  `ethercat0`, managed uplink profile, NIC tuning + IRQ pinning templates). Use:
  ```bash
  cd systemd/ethercat-host
  ./install.sh
  ```
  The single source of truth for which physical port is the EtherCAT port is
  `systemd/ethercat-host/port-layout.env`.
  `install.sh` renders `/etc/ethercat.conf`, the `.link` files, and the
  NetworkManager configs from that one file, including a generated
  `gradient-uplink.nmconnection` profile for the non-EtherCAT NIC, then enables
  the tuning units. A reboot is required for NIC renaming (`eth0/eth1` →
  `uplink0/ethercat0` according to the selected mapping).
  To cut over a live SSH session safely, start the helper **before** unplugging
  the current uplink cable. It waits for carrier on the computed non-EtherCAT
  NIC, then activates the staged managed uplink profile there:
  ```bash
  cd systemd/ethercat-host
  sudo bash ./activate-uplink.sh
  ```

- `wifi/` – optional Wi‑Fi keepalive (auto-reconnect). Useful on flaky networks
  during bring-up (does not touch the EtherCAT NIC). Use:
  ```bash
  cd systemd/wifi
  ./install.sh
  ```

After editing any service file, remember to re-run `sudo systemctl daemon-reload`
before restarting the unit.
