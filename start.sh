#!/bin/bash
# Activation script that includes system Python packages for camera libraries

# Ensure this script is sourced, not executed
if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
  echo "This script must be sourced so the environment persists in your current shell."
  echo "Run: source ./start.sh"
  exit 1
fi

START_SH_QUIET="${GRADIENT_START_QUIET:-0}"

start_sh_print() {
  if [[ "${START_SH_QUIET}" != "1" ]]; then
    printf '%s\n' "$*"
  fi
}

start_sh_print 'NOTE: start.sh is intended for manual/legacy setups.'
start_sh_print 'If you ran ./setup.sh, activation + install are already handled.'

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
VENV_ACTIVATE="${REPO_ROOT}/.venv/bin/activate"

if [[ ! -f "${VENV_ACTIVATE}" ]]; then
  echo "Missing ${VENV_ACTIVATE}"
  echo "Create the single repo virtualenv with ./setup.sh (or uv venv .venv --python python3.12)."
  return 1
fi

# Activate the single repo virtual environment
source "${VENV_ACTIVATE}"

if [[ "${START_SH_QUIET}" != "1" ]]; then
cat <<'BANNER'
>>> GradientOS environment bootstrap
>>> - for manual setups that create .venv without running ./setup.sh
>>> - activates the venv and injects camera/project paths
>>> - registers CLI aliases when console scripts are missing
BANNER
fi

# Compute venv site-packages path and place it FIRST in PYTHONPATH to avoid system packages shadowing venv wheels
VENV_SITE=$(python -c "import site; print([p for p in site.getsitepackages() if p.endswith('site-packages')][0])" 2>/dev/null)
export PYTHONPATH="$VENV_SITE:$(pwd)/src:/usr/lib/python3/dist-packages"

start_sh_print "Virtual environment activated with system camera libraries"
start_sh_print "PYTHONPATH includes: $PYTHONPATH"

# Register CLI aliases only if console scripts are missing
if ! command -v gradient-controller >/dev/null 2>&1; then
  alias gradient-vision='python -m gradient_os.vision'
  alias gradient-ui='python -m gradient_os.ui_start'
  alias gradient-controller='python -m gradient_os.run_controller'
  alias gradient-cli='python -m gradient_os.cli_controller'
  if [[ "${START_SH_QUIET}" != "1" ]]; then
    printf 'CLI aliases registered: %s
' 'gradient-vision gradient-ui gradient-controller gradient-cli'
  fi
else
  start_sh_print 'CLI console scripts already available; aliases not required.'
fi
