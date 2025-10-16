#!/bin/bash

# Convenience launcher for running the Gradient controller in simulator mode.
# Mirrors run.sh but appends --sim so all existing flags/env overrides still apply.

set -euo pipefail

cd "$(dirname "$0")"

REPO_ROOT="${PWD}"
VENV_BIN="${REPO_ROOT}/.venv/bin"

if [[ -d "${VENV_BIN}" ]]; then
  export PATH="${VENV_BIN}:${PATH}"
fi
export PYTHONPATH="${REPO_ROOT}/src:${PYTHONPATH:-}"

if command -v uv >/dev/null 2>&1; then
  CONTROLLER_CMD=(uv run gradient-controller)
else
  CONTROLLER_CMD=(python -m gradient_os.run_controller)
fi

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

echo "[gradient-robotics] Launching arm controller (simulator mode) from ${REPO_ROOT}"
exec "${CONTROLLER_CMD[@]}" "$@" --sim
