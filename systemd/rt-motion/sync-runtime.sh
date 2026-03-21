#!/bin/bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
SERVICE_NAME="gradient-rt-motion.service"
REPO_SERVICE_PATH="${SCRIPT_DIR}/${SERVICE_NAME}"
INSTALLED_SERVICE_PATH="/etc/systemd/system/${SERVICE_NAME}"
ENV_PATH="/etc/default/gradient-rt-motion"
REPO_BINARY_PATH="${REPO_ROOT}/src/gradient_rt_motion/gradient-rt-motion"
INSTALLED_BINARY_PATH="/usr/local/bin/gradient-rt-motion"
ENSURE_ACTIVE=0

log() {
  printf '[rt-motion sync] %s\n' "$*"
}

run_as_root() {
  if [[ "$(id -u)" -eq 0 ]]; then
    "$@"
  else
    sudo -n "$@"
  fi
}

render_runtime_env() {
  local python_bin="${REPO_ROOT}/.venv/bin/python"
  if [[ ! -x "${python_bin}" ]]; then
    python_bin="$(command -v python3 || command -v python)"
  fi
  PYTHONPATH="${REPO_ROOT}/src:${PYTHONPATH:-}" "${python_bin}" - "${REPO_ROOT}" <<'PY'
import io
import sys
from contextlib import redirect_stdout
from pathlib import Path

repo_root = Path(sys.argv[1])
sys.path.insert(0, str(repo_root / "src"))

with redirect_stdout(io.StringIO()):
    from gradient_os import runtime_config
    from gradient_os.arm_controller.robots import get_robot_config
    from gradient_os.arm_controller.backends.ethercat_rtcore.runtime import render_rtcore_systemd_env

desired_config = runtime_config.load_runtime_config()
desired = desired_config.get("desired", {}) if isinstance(desired_config, dict) else {}
desired_overrides = desired.get("overrides", {}) if isinstance(desired.get("overrides"), dict) else {}
allow_unsafe = runtime_config.resolve_allow_unsafe_overrides(
    cli_flag=False,
    desired_flag=bool(desired.get("allow_unsafe_overrides", False)),
)
resolved = runtime_config.resolve_effective_runtime(
    robot_name=str(desired.get("robot", "gradient05")),
    sim_mode=False,
    requested_ik_solver_backend=desired_overrides.get("ik_solver_backend"),
    requested_servo_backend=desired_overrides.get("servo_backend"),
    requested_drive_profile=desired_overrides.get("drive_profile"),
    requested_rt_max_rpm=desired_overrides.get("rt_max_rpm"),
    requested_active_tool_id=desired.get("active_tool_id"),
    allow_unsafe_overrides=allow_unsafe,
)
robot = get_robot_config(str(resolved.get("robot", {}).get("name", "gradient05")))
drive_profile = resolved.get("drive_profile", {}).get("configured_profile")
rtcore_max_rpm = resolved.get("rtcore", {}).get("configured_max_rpm")
print(
    render_rtcore_systemd_env(
        robot_config=robot.get_config_dict(),
        drive_profile=str(drive_profile).strip() or None,
        max_rpm=rtcore_max_rpm,
    ),
    end="",
)
PY
}

service_is_active() {
  run_as_root systemctl is-active --quiet "${SERVICE_NAME}" >/dev/null 2>&1
}

usage() {
  cat <<'EOF'
Usage:
  ./systemd/rt-motion/sync-runtime.sh [--ensure-active]

Options:
  --ensure-active   Start RTCore if inactive, or restart it after syncing changed files.
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --ensure-active)
      ENSURE_ACTIVE=1
      shift
      ;;
    --help|-h)
      usage
      exit 0
      ;;
    *)
      echo "Unknown argument: $1" >&2
      usage >&2
      exit 1
      ;;
  esac
done

if [[ ! -f "${REPO_SERVICE_PATH}" ]]; then
  echo "Missing repo service file: ${REPO_SERVICE_PATH}" >&2
  exit 1
fi

tmp_env="$(mktemp)"
trap 'rm -f "${tmp_env}"' EXIT
render_runtime_env > "${tmp_env}"

service_changed=0
env_changed=0
binary_changed=0

if [[ ! -x "${REPO_BINARY_PATH}" ]]; then
  echo "Missing built RTCore binary: ${REPO_BINARY_PATH}" >&2
  echo "Build it first with: make -C ${REPO_ROOT}/src/gradient_rt_motion" >&2
  exit 1
fi

if [[ ! -f "${INSTALLED_BINARY_PATH}" ]] || ! cmp -s "${REPO_BINARY_PATH}" "${INSTALLED_BINARY_PATH}"; then
  log "Updating installed RTCore binary at ${INSTALLED_BINARY_PATH}"
  run_as_root install -m 0755 "${REPO_BINARY_PATH}" "${INSTALLED_BINARY_PATH}"
  binary_changed=1
fi

if [[ ! -f "${INSTALLED_SERVICE_PATH}" ]] || ! cmp -s "${REPO_SERVICE_PATH}" "${INSTALLED_SERVICE_PATH}"; then
  log "Updating installed RTCore unit at ${INSTALLED_SERVICE_PATH}"
  run_as_root install -m 0644 "${REPO_SERVICE_PATH}" "${INSTALLED_SERVICE_PATH}"
  service_changed=1
fi

if [[ ! -f "${ENV_PATH}" ]] || ! cmp -s "${tmp_env}" "${ENV_PATH}"; then
  log "Updating RTCore runtime env at ${ENV_PATH}"
  run_as_root install -m 0644 "${tmp_env}" "${ENV_PATH}"
  env_changed=1
fi

if [[ "${service_changed}" -eq 1 ]]; then
  log "Reloading systemd daemon"
  run_as_root systemctl daemon-reload
fi

if [[ "${ENSURE_ACTIVE}" -eq 1 ]]; then
  if [[ "${service_changed}" -eq 1 || "${env_changed}" -eq 1 || "${binary_changed}" -eq 1 ]]; then
    run_as_root systemctl reset-failed "${SERVICE_NAME}" >/dev/null 2>&1 || true
  fi
  if service_is_active; then
    if [[ "${service_changed}" -eq 1 || "${env_changed}" -eq 1 || "${binary_changed}" -eq 1 ]]; then
      log "Restarting ${SERVICE_NAME} to apply updated RTCore scaling/profile config"
      run_as_root systemctl restart "${SERVICE_NAME}"
    else
      log "${SERVICE_NAME} already active; runtime config is current"
    fi
  else
    log "Starting ${SERVICE_NAME}"
    run_as_root systemctl start "${SERVICE_NAME}"
  fi
else
  if [[ "${service_changed}" -eq 0 && "${env_changed}" -eq 0 && "${binary_changed}" -eq 0 ]]; then
    log "RTCore unit/env already in sync"
  fi
fi
