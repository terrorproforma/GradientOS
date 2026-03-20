#!/bin/bash

# Unified launcher for the Gradient Robotics arm controller.
# Enables both manual invocations and use under systemd units.

set -euo pipefail

# Ensure we're running from the repository root
cd "$(dirname "$0")"

REPO_ROOT="${PWD}"
VENV_BIN="${REPO_ROOT}/.venv/bin"
VENV_PY="${VENV_BIN}/python"
START_SH="${REPO_ROOT}/start.sh"

# Prefer the same bootstrap semantics as manual shells when env is not already active.
if [[ -z "${VIRTUAL_ENV:-}" || "${VIRTUAL_ENV}" != "${REPO_ROOT}/.venv" ]]; then
  if [[ -f "${START_SH}" ]]; then
    # shellcheck disable=SC1090
    if ! source "${START_SH}" >/dev/null 2>&1; then
      echo "[gradient-robotics] ERROR: Failed to bootstrap environment via ${START_SH}" >&2
      exit 1
    fi
  fi
fi

# Put project paths at the front so systemd/manual runs behave consistently
if [[ -d "${VENV_BIN}" ]]; then
  export PATH="${VENV_BIN}:${PATH}"
fi
export PYTHONPATH="${REPO_ROOT}/src:${PYTHONPATH:-}"
export GRADIENT_RTCORE_AUTO_ARM="${GRADIENT_RTCORE_AUTO_ARM:-0}"

if [[ ! -x "${VENV_PY}" ]]; then
  echo "[gradient-robotics] ERROR: Missing ${VENV_PY}" >&2
  echo "[gradient-robotics] Use the single repo virtualenv (.venv) used by start.sh/setup.sh." >&2
  exit 1
fi

if [[ -n "${VIRTUAL_ENV:-}" ]] && [[ "${VIRTUAL_ENV}" != "${REPO_ROOT}/.venv" ]]; then
  echo "[gradient-robotics] WARNING: Different virtualenv active (${VIRTUAL_ENV}). Using ${VENV_PY} instead." >&2
fi

CONTROLLER_CMD=("${VENV_PY}" -m gradient_os.run_controller)

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
