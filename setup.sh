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
Usage: ./setup.sh [--quiet|-q]

  --quiet, -q   Install only the core controller/API dependencies without prompts.
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
while [[ $# -gt 0 ]]; do
  case $1 in
    -q|--quiet) QUIET=true; shift ;;
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
want_ui=false
want_vision=false
want_ai=false
want_datasets=false
want_dev=false
want_picamera=false

if $QUIET; then
  say "Quiet mode enabled – installing core only."
else
  say "You'll be guided through optional components."
  echo
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
fi

append_extra() {
  local extra=$1 desc=$2
  case ",$EXTRA_SPEC," in
    *",$extra,"*) return ;;
  esac
  EXTRA_SPEC+=",$extra"
  SELECTED_DESC+=("$desc")
}

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
  if $IS_RASPI; then
    APT_PACKAGES=$(add_pkg curl "$APT_PACKAGES")
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
elif [[ $OS_NAME == "Darwin" ]]; then
  if $want_vision; then
    BREW_PACKAGES=$(add_pkg libomp "$BREW_PACKAGES")
  fi
  if $want_ui; then
    BREW_PACKAGES=$(add_pkg qt "$BREW_PACKAGES")
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

# --- Virtual environment ----------------------------------------------------
if [[ ! -d .venv ]]; then
  headline "Creating virtual environment"
  uv venv .venv --prompt "Gradient OS"
else
  say "Virtual environment .venv already exists; reusing."
fi

if [[ -z ${VIRTUAL_ENV:-} ]]; then
  # shellcheck disable=SC1091
  source .venv/bin/activate
fi

# --- Python dependencies ----------------------------------------------------
headline "Installing Python packages"
say "uv pip install -e .[$EXTRA_SPEC]"
uv pip install -e ".[${EXTRA_SPEC}]"

say "Setup complete. Activate with: source .venv/bin/activate"
