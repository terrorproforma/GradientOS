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
  The service binds to `0.0.0.0:8000` by default and uses the project virtualenv.
  Override environment variables inside the unit if you host the controller on a
  different machine.

After editing any service file, remember to re-run `sudo systemctl daemon-reload`
before restarting the unit.
