#!/usr/bin/env bash
# Install system-level dependencies required by GradientOS vision and UI components.
set -euo pipefail

OS_ID=""
OS_ID_LIKE=""
if [[ -f /etc/os-release ]]; then
  # shellcheck disable=SC1091
  source /etc/os-release
  OS_ID=${ID:-}
  OS_ID_LIKE=${ID_LIKE:-}
fi

APT_GET=$(command -v apt-get || true)

should_install_apt="false"
if [[ "${OS_ID}" == "raspbian" || "${OS_ID_LIKE}" == *"raspbian"* || "${OS_ID_LIKE}" == *"rpi"* ]]; then
  should_install_apt="true"
fi

if [[ "${should_install_apt}" == "true" ]]; then
  if [[ -z "$APT_GET" ]]; then
    echo "[setup] Expected apt-get for Raspberry Pi OS but it was not found." >&2
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
  set +x
else
  uname_out=$(uname -s)
  case "${OS_ID}" in
    ubuntu)
      cat <<'EOF'
[setup] Detected Ubuntu. Please install system dependencies manually:
    sudo apt-get update
    sudo apt-get install python3-libcamera python3-kms++ libgl1-mesa-glx \
        libgl1-mesa-dri mesa-utils libcap-dev curl
EOF
      ;;
    "")
      if [[ "${uname_out}" == "Darwin" ]]; then
        cat <<'EOF'
[setup] Detected macOS. Please ensure you have the following packages/installations:
    brew install python libomp
    # libcamera is not available; vision components require a Pi.
EOF
      else
        cat <<EOF
[setup] Unsupported distribution (${uname_out}). Install equivalent packages manually.
EOF
      fi
      ;;
    *)
      cat <<EOF
[setup] '${OS_ID}' detected. Please install the libcamera/Mesa dependencies manually.
EOF
      ;;
  esac
fi


if ! command -v uv >/dev/null 2>&1; then
  echo "[setup] uv not detected; installing via astral.sh script..."
  curl -LsSf https://astral.sh/uv/install.sh | sh
  echo "[setup] uv installed. Ensure $HOME/.local/bin is on PATH (setup.sh does not modify shell rc files)."
fi

if ! command -v uv >/dev/null 2>&1; then
  echo "[setup] ERROR: uv still not found on PATH after installation attempt. Aborting." >&2
  exit 1
fi

if [[ ! -d .venv ]]; then
  echo "[setup] Creating virtual environment with uv venv .venv"
  uv venv .venv --prompt "Gradient OS" --system-site-packages
else
  echo "[setup] Virtual environment .venv already exists. Skipping creation."
fi

if [[ -z "${VIRTUAL_ENV:-}" ]]; then
  echo "[setup] Activating .venv to finalize installation"
  # shellcheck disable=SC1091
  source .venv/bin/activate
fi

echo "[setup] Installing project in editable mode (uv pip install -e .)"
uv pip install -e .
