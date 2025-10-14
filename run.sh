#!/bin/bash

# Unified launcher for the Gradient Robotics arm controller.
# Enables both manual invocations and use under systemd units.

set -euo pipefail

# Ensure we're running from the repository root
cd "$(dirname "$0")"

REPO_ROOT="${PWD}"
VENV_BIN="${REPO_ROOT}/.venv/bin"

# Put project paths at the front so systemd/manual runs behave consistently
if [[ -d "${VENV_BIN}" ]]; then
  export PATH="${VENV_BIN}:${PATH}"
fi
export PYTHONPATH="${REPO_ROOT}/src:${PYTHONPATH:-}"

# Prefer uv if available so dependencies resolve via lockfile; fall back to Python.
if command -v uv >/dev/null 2>&1; then
  CONTROLLER_CMD=(uv run gradient-controller)
else
  CONTROLLER_CMD=(python -m gradient_os.run_controller)
fi

# Allow SERIAL_PORT env override when args omit it.
SERIAL_PORT_ENV="${SERIAL_PORT:-}"
if [[ -n "${SERIAL_PORT_ENV}" ]]; then
  wants_serial_override=true
  for arg in "$@"; do
    if [[ "${arg}" == "--serial-port" ]]; then
      wants_serial_override=false
      break
    fi
  done
  if "${wants_serial_override}"; then
    set -- "$@" --serial-port "${SERIAL_PORT_ENV}"
  fi
fi

echo "[gradient-robotics] Launching arm controller from ${REPO_ROOT}"
exec "${CONTROLLER_CMD[@]}" "$@"
