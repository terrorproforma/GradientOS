#!/usr/bin/env bash
# Install system-level dependencies required by GradientOS vision and UI components.
set -euo pipefail

APT_GET=$(command -v apt-get || true)
if [[ -z "$APT_GET" ]]; then
  echo "apt-get not found. This script currently supports Debian/Ubuntu systems." >&2
  exit 1
fi

if [[ $EUID -ne 0 ]]; then
  SUDO="sudo"
else
  SUDO=""
fi

packages=(
  python3-libcamera
  python3-kms++
  libgl1-mesa-glx
  libgl1-mesa-dri
  mesa-utils
  libcap-dev
  curl
)

set -x
$SUDO "$APT_GET" update
$SUDO "$APT_GET" install -y "${packages[@]}"

if ! command -v uv >/dev/null 2>&1; then
  echo "[setup] uv not detected; installing via astral.sh script..."
  curl -LsSf https://astral.sh/uv/install.sh | sh
  echo "[setup] uv installed. Ensure $HOME/.local/bin is on PATH (setup.sh does not modify shell rc files)."
fi
