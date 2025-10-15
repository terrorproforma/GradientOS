#!/bin/bash

# Launcher for the GradientOS API service.

set -euo pipefail

# Ensure we're running from repository root.
cd "$(dirname "$0")"

REPO_ROOT="${PWD}"
VENV_BIN="${REPO_ROOT}/.venv/bin"

# Prepend virtualenv bin if present for consistent PATH resolution.
if [[ -d "${VENV_BIN}" ]]; then
  export PATH="${VENV_BIN}:${PATH}"
fi
export PYTHONPATH="${REPO_ROOT}/src:${PYTHONPATH:-}"

# Prefer uv to honor the lockfile; fall back to python module execution.
if command -v uv >/dev/null 2>&1; then
  API_CMD=(uv run gradient-api)
else
  API_CMD=(python -m gradient_os.api.main)
fi

echo "[gradient-robotics] Launching API service from ${REPO_ROOT}"
exec "${API_CMD[@]}" "$@"
