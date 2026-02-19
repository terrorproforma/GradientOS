#!/bin/bash

# Launcher for the GradientOS API service.

set -euo pipefail

# Ensure we're running from repository root.
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

# Prepend virtualenv bin if present for consistent PATH resolution.
if [[ -d "${VENV_BIN}" ]]; then
  export PATH="${VENV_BIN}:${PATH}"
fi
export PYTHONPATH="${REPO_ROOT}/src:${PYTHONPATH:-}"

if [[ ! -x "${VENV_PY}" ]]; then
  echo "[gradient-robotics] ERROR: Missing ${VENV_PY}" >&2
  echo "[gradient-robotics] Use the single repo virtualenv (.venv) used by start.sh/setup.sh." >&2
  exit 1
fi

if [[ -n "${VIRTUAL_ENV:-}" ]] && [[ "${VIRTUAL_ENV}" != "${REPO_ROOT}/.venv" ]]; then
  echo "[gradient-robotics] WARNING: Different virtualenv active (${VIRTUAL_ENV}). Using ${VENV_PY} instead." >&2
fi

API_CMD=("${VENV_PY}" -m gradient_os.api.main)

echo "[gradient-robotics] Launching API service from ${REPO_ROOT}"
exec "${API_CMD[@]}" "$@"
