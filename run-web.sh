#!/bin/bash

# Launcher for the GradientOS web UI development server.

set -euo pipefail

cd "$(dirname "$0")"

REPO_ROOT="${PWD}"
WEB_DIR="${REPO_ROOT}/web-ui"

if [[ ! -d "${WEB_DIR}" ]]; then
  echo "[gradient-robotics] web-ui directory not found at ${WEB_DIR}" >&2
  exit 1
fi

if ! command -v npm >/dev/null 2>&1; then
  echo "[gradient-robotics] npm not found on PATH. Install Node.js/npm to run the web UI." >&2
  exit 1
fi

echo "[gradient-robotics] Launching web UI dev server from ${WEB_DIR}"
cd "${WEB_DIR}"
exec npm run dev "$@"
