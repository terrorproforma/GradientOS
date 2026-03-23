#!/usr/bin/env bash
set -euo pipefail

# --- Styling helpers ---------------------------------------------------------
if [[ -t 1 ]] && command -v tput >/dev/null 2>&1; then
  BOLD=$(tput bold); GREEN=$(tput setaf 2); YELLOW=$(tput setaf 3); RED=$(tput setaf 1); BLUE=$(tput setaf 6); RESET=$(tput sgr0)
else
  BOLD=""; GREEN=""; YELLOW=""; RED=""; BLUE=""; RESET=""
fi

say()    { printf "%s%s%s\n" "$BLUE" "$1" "$RESET"; }
warn()   { printf "%s%s%s\n" "$YELLOW" "$1" "$RESET"; }
fail()   { printf "%s%s%s\n" "$RED" "$1" "$RESET" >&2; exit 1; }
headline(){ printf "\n%s%s%s\n\n" "$BOLD" "$1" "$RESET"; }

usage() {
  cat <<'USAGE'
Usage: ./setup.sh [--quiet|-q] [--clean]

  --quiet, -q   Install only the core controller/API dependencies without prompts.
  --clean       Remove existing .venv and clear caches before installing. This is the recommended primary install path.
USAGE
}

ask_yes_no() {
  local prompt=$1 default=${2:-n} answer suffix
  suffix="y/N"
  [[ $default == [Yy] ]] && suffix="Y/n"
  while true; do
    read -rp "$prompt [$suffix] " answer || answer=""
    answer=${answer:-$default}
    answer=$(printf '%s' "$answer" | tr '[:upper:]' '[:lower:]')
    case $answer in
      y|yes) return 0;;
      n|no)  return 1;;
    esac
    warn "Please answer y or n."
  done
}

# --- Argument parsing --------------------------------------------------------
QUIET=false
CLEAN=false
while [[ $# -gt 0 ]]; do
  case $1 in
    -q|--quiet) QUIET=true; shift ;;
    --clean) CLEAN=true; shift ;;
    -h|--help) usage; exit 0 ;;
    *) fail "Unknown argument: $1" ;;
  esac
done

# --- Environment detection ---------------------------------------------------
OS_NAME=$(uname -s)
OS_ID=""; OS_ID_LIKE=""; OS_PRETTY=""
if [[ -f /etc/os-release ]]; then
  # shellcheck disable=SC1091
  source /etc/os-release || true
  OS_ID=${ID:-}; OS_ID_LIKE=${ID_LIKE:-}; OS_PRETTY=${PRETTY_NAME:-}
fi
IS_RASPI=false

# Check device-tree identifiers first (most reliable)
if [[ -r /proc/device-tree/model ]]; then
  if tr -d '\0' </proc/device-tree/model | grep -qi 'raspberry pi'; then
    IS_RASPI=true
  fi
fi
if ! $IS_RASPI && [[ -r /sys/firmware/devicetree/base/model ]]; then
  if tr -d '\0' </sys/firmware/devicetree/base/model | grep -qi 'raspberry pi'; then
    IS_RASPI=true
  fi
fi

# Fall back to os-release heuristics when device-tree missing (e.g. inside chroots)
if ! $IS_RASPI; then
  case ${OS_ID} in
    raspi*|raspbi*) IS_RASPI=true ;;
  esac
fi
if ! $IS_RASPI; then
  case ${OS_ID_LIKE} in
    *raspi*|*raspbian*|*raspios*) IS_RASPI=true ;;
  esac
fi
if ! $IS_RASPI && [[ -f /etc/os-release ]]; then
  if grep -qiE 'raspberry ?pi|raspbian|raspios' /etc/os-release; then
    IS_RASPI=true
  fi
fi

# --- Collect choices ---------------------------------------------------------
headline "GradientOS Setup"

EXTRA_SPEC="core"
SELECTED_DESC=("core (controller/API)")
want_cad=false
want_ui=false
want_vision=false
want_ai=false
want_datasets=false
want_dev=false
want_picamera=false
want_web_ui=false

if $QUIET; then
  say "Quiet mode enabled – installing core only."
else
  say "You'll be guided through optional components."
  echo
  if ask_yes_no "Install CAD STEP topology support (OpenCascade bindings)?" n; then
    want_cad=true
  fi
  if ask_yes_no "Install the Gradient UI (PySide6 desktop app)?" n; then
    want_ui=true
  fi
  if ask_yes_no "Install Gradient Vision tooling (CLI, telemetry, OpenCV)?" n; then
    want_vision=true
  fi
  if $IS_RASPI; then
    if ask_yes_no "Add Raspberry Pi camera support (picamera2)?" n; then
      want_picamera=true
      want_vision=true
    fi
  else
    say "Pi camera support only available on Raspberry Pi OS; skipping."
  fi
  if ask_yes_no "Install AI extras (Ultralytics + Torch)?" n; then
    want_ai=true
  fi
  if ask_yes_no "Install dataset tooling (LeRobot converters)?" n; then
    want_datasets=true
  fi
  if ask_yes_no "Install developer tooling (pytest, pre-commit)?" n; then
    want_dev=true
  fi
  if [[ -f web-ui/package.json ]]; then
    if ask_yes_no "Install web UI frontend dependencies (npm install)?" n; then
      want_web_ui=true
    fi
  else
    say "web-ui directory not found; skipping frontend install prompt."
  fi
fi

append_extra() {
  local extra=$1 desc=$2
  case ",$EXTRA_SPEC," in
    *",$extra,"*) return ;;
  esac
  EXTRA_SPEC+=",$extra"
  SELECTED_DESC+=("$desc")
}

if $want_cad; then
  append_extra cad "cad (STEP topology)"
fi
if $want_ui; then
  append_extra ui "ui (desktop app)"
fi
if $want_vision; then
  append_extra vision "vision (camera + telemetry)"
fi
if $want_picamera; then
  append_extra picamera "picamera (Pi CSI)"
fi
if $want_ai; then
  append_extra ai "ai (YOLO / Torch)"
fi
if $want_datasets; then
  append_extra datasets "datasets (LeRobot)"
fi
if $want_dev; then
  append_extra dev "dev (tests/tooling)"
fi
if $want_web_ui; then
  SELECTED_DESC+=("web-ui (npm install)")
fi

headline "Selected Components"
for desc in "${SELECTED_DESC[@]}"; do
  printf " - %s\n" "$desc"
done

# --- System dependencies ----------------------------------------------------
APT_PACKAGES=""
BREW_PACKAGES=""
add_pkg() {
  local pkg=$1
  local list=$2
  if [[ -z $list ]]; then
    printf '%s' "$pkg"
  elif ! printf '%s\n' "$list" | grep -Fxq "$pkg"; then
    printf '%s\n%s' "$list" "$pkg"
  else
    printf '%s' "$list"
  fi
}

if [[ $OS_NAME == "Linux" ]]; then
  # Core build dependencies required for native Python extensions (pybind11, etc.)
  for pkg in build-essential python3-dev curl; do
    APT_PACKAGES=$(add_pkg "$pkg" "$APT_PACKAGES")
  done
  if $IS_RASPI; then
    if $want_vision; then
      for pkg in libgl1-mesa-glx libgl1-mesa-dri mesa-utils libcap-dev python3-libcamera python3-kms++; do
        APT_PACKAGES=$(add_pkg "$pkg" "$APT_PACKAGES")
      done
    fi
    if $want_picamera; then
      APT_PACKAGES=$(add_pkg python3-picamera2 "$APT_PACKAGES")
    fi
  else
    if $want_vision; then
      for pkg in libgl1-mesa-glx libgl1-mesa-dri mesa-utils libcap-dev; do
        APT_PACKAGES=$(add_pkg "$pkg" "$APT_PACKAGES")
      done
    fi
  fi
  # Node.js for web UI
  if $want_web_ui; then
    for pkg in nodejs npm; do
      APT_PACKAGES=$(add_pkg "$pkg" "$APT_PACKAGES")
    done
  fi
elif [[ $OS_NAME == "Darwin" ]]; then
  if $want_vision; then
    BREW_PACKAGES=$(add_pkg libomp "$BREW_PACKAGES")
  fi
  if $want_ui; then
    BREW_PACKAGES=$(add_pkg qt "$BREW_PACKAGES")
  fi
  if $want_web_ui; then
    BREW_PACKAGES=$(add_pkg node "$BREW_PACKAGES")
  fi
else
  warn "No automated system dependency installation for $OS_NAME."
fi

if [[ -n $APT_PACKAGES ]]; then
  if command -v apt-get >/dev/null 2>&1; then
    headline "Installing apt packages"
    say "Packages: $APT_PACKAGES"
    SUDO=""
    [[ $EUID -ne 0 ]] && SUDO="sudo"
    $SUDO apt-get update
    $SUDO apt-get install -y $APT_PACKAGES
  else
    warn "apt-get not found; please install: $APT_PACKAGES"
  fi
fi

if [[ -n $BREW_PACKAGES ]]; then
  if command -v brew >/dev/null 2>&1; then
    headline "Installing Homebrew packages"
    say "Packages: $BREW_PACKAGES"
    brew install $BREW_PACKAGES
  else
    warn "Homebrew not detected; run: brew install $BREW_PACKAGES"
  fi
fi

# --- Ensure uv --------------------------------------------------------------
if ! command -v uv >/dev/null 2>&1; then
  headline "Installing uv"
  curl -LsSf https://astral.sh/uv/install.sh | sh
  say "uv installed. Ensure ${HOME}/.local/bin is on PATH."
fi
command -v uv >/dev/null 2>&1 || fail "uv still not on PATH after installation attempt."

# Optional clean: remove existing venv and clear uv cache to avoid stale build deps
if $CLEAN; then
  headline "Cleaning previous environment and caches"
  rm -rf .venv || true
  uv cache clean || true
fi

# --- Virtual environment ----------------------------------------------------
if [[ ! -d .venv ]]; then
  headline "Creating virtual environment (prefer Python 3.12)"
  # Prefer CPython 3.12 for wheel compatibility across platforms.
  # Fall back to 3.11, then 3.14, then let uv manage.
  if command -v python3.12 >/dev/null 2>&1; then
    uv venv .venv --python python3.12 --prompt "Gradient OS"
  elif command -v python3.11 >/dev/null 2>&1; then
    uv venv .venv --python python3.11 --prompt "Gradient OS"
  elif command -v python3.14 >/dev/null 2>&1; then
    uv venv .venv --python python3.14 --prompt "Gradient OS"
  else
    # uv will acquire a compatible interpreter (project supports 3.11–3.14)
    uv venv .venv --prompt "Gradient OS"
  fi
else
  say "Virtual environment .venv already exists; reusing."
fi

if [[ -z ${VIRTUAL_ENV:-} ]]; then
  # shellcheck disable=SC1091
  source .venv/bin/activate
fi

# --- Python dependencies ----------------------------------------------------
headline "Installing Python packages"

# Increase timeout for slow network connections (e.g. Raspberry Pi)
export UV_HTTP_TIMEOUT=300

# Install pip in venv so fallback mechanism works
uv pip install pip

# Try isolated build first; if packaging/metadata error occurs, fall back
set +e
uv pip install -e ".[${EXTRA_SPEC}]"
STATUS=$?
set -e
if [[ $STATUS -ne 0 ]]; then
  warn "Editable install failed; applying Jetson build backend workaround and retrying without isolation."
  # Prefer newer backend if available to align with latest packaging APIs
  if ! uv pip install -U "scikit-build-core==0.12.1" "packaging==25.0" "setuptools>=69" "wheel>=0.42"; then
    warn "uv failed to fetch scikit-build-core 0.12.1; retrying with pip"
    python -m pip install -U "scikit-build-core==0.12.1" "packaging==25.0" "setuptools>=69" "wheel>=0.42"
  fi
  uv pip install --no-build-isolation -e ".[${EXTRA_SPEC}]"
fi

if $want_web_ui; then
  headline "Installing web UI dependencies"
  if command -v npm >/dev/null 2>&1; then
    pushd web-ui >/dev/null
    npm install
    popd >/dev/null
    say "web UI dependencies installed."
  else
    warn "npm not found on PATH; install Node.js/npm and run 'npm install' inside web-ui/ manually."
  fi
fi