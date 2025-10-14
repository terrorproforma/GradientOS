#!/bin/bash
# Activation script that includes system Python packages for camera libraries

# Ensure this script is sourced, not executed
if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
  echo "This script must be sourced so the environment persists in your current shell."
  echo "Run: source ./start.sh"
  exit 1
fi

echo 'NOTE: start.sh is intended for manual/legacy setups.'
echo 'If you ran ./setup.sh, activation + install are already handled.'
# Activate the virtual environment
source .venv/bin/activate

cat <<'BANNER'
>>> GradientOS environment bootstrap
>>> - for manual setups that create .venv without running ./setup.sh
>>> - activates the venv and injects camera/project paths
>>> - registers CLI aliases when console scripts are missing
BANNER

# Compute venv site-packages path and place it FIRST in PYTHONPATH to avoid system packages shadowing venv wheels
VENV_SITE=$(python -c "import site; print([p for p in site.getsitepackages() if p.endswith('site-packages')][0])" 2>/dev/null)
export PYTHONPATH="$VENV_SITE:$(pwd)/src:/usr/lib/python3/dist-packages"

echo "Virtual environment activated with system camera libraries"
echo "PYTHONPATH includes: $PYTHONPATH"

# Register CLI aliases only if console scripts are missing
if ! command -v gradient-controller >/dev/null 2>&1; then
  alias gradient-vision='python -m gradient_os.vision'
  alias gradient-ui='python -m gradient_os.ui_start'
  alias gradient-controller='python -m gradient_os.run_controller'
  alias gradient-cli='python -m gradient_os.cli_controller'
  printf 'CLI aliases registered: %s
' 'gradient-vision gradient-ui gradient-controller gradient-cli'
else
  echo 'CLI console scripts already available; aliases not required.'
fi
