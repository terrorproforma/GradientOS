#!/bin/bash

# Staged supervisor for the local GradientOS stack.
# Starts controller -> API -> web UI with readiness checks and durable logs.

set -euo pipefail

cd "$(dirname "$0")"

REPO_ROOT="${PWD}"
START_SH="${REPO_ROOT}/start.sh"
CONTROLLER_SCRIPT="${REPO_ROOT}/run.sh"
API_SCRIPT="${REPO_ROOT}/run-api.sh"
WEB_SCRIPT="${REPO_ROOT}/run-web.sh"
LOG_ROOT="${REPO_ROOT}/logs/startups"
STATE_DIR="${LOG_ROOT}/_runtime"
ACTIVE_STATE="${STATE_DIR}/active.env"

SYSTEM_PYTHON="$(command -v python3 || command -v python || true)"
if [[ -z "${SYSTEM_PYTHON}" ]]; then
  echo "[start-stack] ERROR: python3 (or python) is required for health probes." >&2
  exit 1
fi
PROJECT_PYTHON="${REPO_ROOT}/.venv/bin/python"
PROBE_PYTHON="${PROJECT_PYTHON}"
if [[ ! -x "${PROBE_PYTHON}" ]]; then
  PROBE_PYTHON="${SYSTEM_PYTHON}"
fi
RTCORE_SYNC_SCRIPT="${REPO_ROOT}/systemd/rt-motion/sync-runtime.sh"
TTY_DEVICE="$(tty 2>/dev/null || true)"

ACTION="start"
HEADLESS=0
HARD_STOP=0

RUN_ID=""
MODE="full"
LOG_DIR=""
LAUNCHER_LOG=""
MANIFEST_PATH=""

CONTROLLER_HOST="${GRADIENT_CONTROLLER_HOST:-127.0.0.1}"
CONTROLLER_PORT="${GRADIENT_CONTROLLER_PORT:-3000}"
API_PORT="${GRADIENT_API_PORT:-4000}"
WEB_PORT="${GRADIENT_WEB_PORT:-8000}"
RTCORE_METRICS_PATH="${GRADIENT_RTCORE_METRICS:-/run/gradient-rt-motion/metrics.json}"
RTCORE_SERVICE_NAME="gradient-rt-motion.service"
ETHERCAT_SERVICE_NAME="ethercat.service"

CONTROLLER_TIMEOUT_S="${GRADIENT_STACK_CONTROLLER_TIMEOUT_S:-25}"
API_TIMEOUT_S="${GRADIENT_STACK_API_TIMEOUT_S:-20}"
WEB_TIMEOUT_S="${GRADIENT_STACK_WEB_TIMEOUT_S:-20}"
PROBE_INTERVAL_S="${GRADIENT_STACK_PROBE_INTERVAL_S:-0.25}"
BUS_READY_TIMEOUT_S="${GRADIENT_STACK_BUS_READY_TIMEOUT_S:-20}"
STARTUP_FAULT_RESET_TIMEOUT_S="${GRADIENT_STACK_STARTUP_FAULT_RESET_TIMEOUT_S:-3}"
INTERACTIVE_CONSOLE_MODE="${GRADIENT_STACK_INTERACTIVE_CONSOLE:-auto}"

START_RESULT="not_started"
START_FAILURE=""
SHUTDOWN_REASON=""
MANAGE_CHILDREN=0
STOP_REQUESTED=0
SHUTDOWN_POWER_DOWN_ATTEMPTED=0

CONTROLLER_PID=""
API_PID=""
WEB_PID=""
TAIL_PIDS=()
CHILD_PIDS=()

CONTROLLER_LOG=""
API_LOG=""
WEB_LOG=""
STARTED_PID=""

usage() {
  cat <<'EOF'
Usage:
  ./start-stack.sh [--headless]
  ./start-stack.sh status
  ./start-stack.sh probe
  ./start-stack.sh stop [--hard]
  ./start-stack.sh --help

Options:
  --headless   Start only controller + API, skip the web UI.
  status       Show launcher state plus live controller/API/web probe results.
  probe        Show physical hardware state from RTCore metrics + live runtime probes.
  stop         Soft-stop the stack: de-energize drives, stop controller/API/web, leave RTCore + EtherCAT up.
  --hard       With stop: also stop RTCore and ethercat.service after the drives are disarmed.
  --help       Show this help text.

Environment:
  GRADIENT_CONTROLLER_HOST           Controller probe host (default: 127.0.0.1)
  GRADIENT_CONTROLLER_PORT           Controller probe UDP port (default: 3000)
  GRADIENT_API_PORT                  API HTTP port (default: 4000)
  GRADIENT_WEB_PORT                  Web UI HTTP port (default: 8000)
  GRADIENT_RTCORE_METRICS            RTCore metrics path (default: /run/gradient-rt-motion/metrics.json)
  GRADIENT_RTCORE_READY_TIMEOUT_S    Controller-side RTCore readiness timeout (default: 30 via start-stack.sh)
  GRADIENT_STACK_BUS_READY_TIMEOUT_S Launcher bus-ready timeout after controller comes up (default: 20)
  GRADIENT_STACK_INTERACTIVE_CONSOLE Interactive in-terminal command console: auto|0 (default: auto)
  GRADIENT_STACK_CONTROLLER_TIMEOUT_S  Controller readiness timeout (default: 25)
  GRADIENT_STACK_API_TIMEOUT_S         API readiness timeout (default: 20)
  GRADIENT_STACK_WEB_TIMEOUT_S         Web readiness timeout (default: 20)
  GRADIENT_STACK_PROBE_INTERVAL_S      Probe interval seconds (default: 0.25)
  GRADIENT_STACK_STARTUP_FAULT_RESET_TIMEOUT_S  Wait time after startup auto-reset before aborting (default: 3)
EOF
}

timestamp() {
  date '+%Y-%m-%d %H:%M:%S%z'
}

log() {
  printf '[%s] [start-stack] %s\n' "$(timestamp)" "$*"
}

warn() {
  printf '[%s] [start-stack] WARNING: %s\n' "$(timestamp)" "$*" >&2
}

error() {
  printf '[%s] [start-stack] ERROR: %s\n' "$(timestamp)" "$*" >&2
}

pid_is_live() {
  local pid="${1:-}"
  [[ -n "${pid}" ]] && kill -0 "${pid}" 2>/dev/null
}

pid_group_is_live() {
  local pgid="${1:-}"
  [[ -n "${pgid}" ]] && kill -0 -- "-${pgid}" 2>/dev/null
}

pid_group_id() {
  local pid="${1:-}"
  [[ -n "${pid}" ]] || return 1
  ps -o pgid= -p "${pid}" 2>/dev/null | tr -d '[:space:]'
}

collect_descendant_pids() {
  local root_pid="${1:-}"
  local child=""
  [[ -n "${root_pid}" ]] || return 0

  while read -r child; do
    [[ -n "${child}" ]] || continue
    printf '%s\n' "${child}"
    collect_descendant_pids "${child}"
  done < <(pgrep -P "${root_pid}" 2>/dev/null || true)
}

stop_pid_hierarchy() {
  local name="$1"
  local pid="$2"
  local grace_ticks="${3:-50}"
  local start_message="${4:-Stopping ${name} (pid=${pid})}"
  local timeout_message="${5:-${name} did not exit after SIGTERM; sending SIGKILL}"
  local pgid=""
  local child=""
  local live_pid=""
  local -a targets=()
  local -a survivors=()
  local -A seen=()

  if [[ -z "${pid}" ]]; then
    return 0
  fi

  pgid="$(pid_group_id "${pid}" || true)"

  if pid_is_live "${pid}"; then
    targets+=("${pid}")
    seen["${pid}"]=1
  fi

  while read -r child; do
    [[ -n "${child}" ]] || continue
    if [[ -n "${seen[${child}]:-}" ]]; then
      continue
    fi
    targets+=("${child}")
    seen["${child}"]=1
  done < <(collect_descendant_pids "${pid}")

  if (( ${#targets[@]} == 0 )) && [[ -z "${pgid}" || "${pgid}" != "${pid}" ]]; then
    return 0
  fi

  log "${start_message}"
  if [[ -n "${pgid}" && "${pgid}" == "${pid}" ]]; then
    kill -TERM -- "-${pgid}" 2>/dev/null || true
  fi
  if (( ${#targets[@]} > 0 )); then
    kill -TERM "${targets[@]}" 2>/dev/null || true
  fi

  local waited=0
  while [[ "${waited}" -lt "${grace_ticks}" ]]; do
    local still_live=0
    if [[ -n "${pgid}" && "${pgid}" == "${pid}" ]] && pid_group_is_live "${pgid}"; then
      still_live=1
    fi
    if [[ "${still_live}" -eq 0 ]]; then
      for live_pid in "${targets[@]}"; do
        if pid_is_live "${live_pid}"; then
          still_live=1
          break
        fi
      done
    fi
    if [[ "${still_live}" -eq 0 ]]; then
      return 0
    fi
    sleep 0.1
    waited=$((waited + 1))
  done

  warn "${timeout_message}"
  if [[ -n "${pgid}" && "${pgid}" == "${pid}" ]]; then
    kill -KILL -- "-${pgid}" 2>/dev/null || true
  fi

  for live_pid in "${targets[@]}"; do
    if pid_is_live "${live_pid}"; then
      survivors+=("${live_pid}")
    fi
  done
  if (( ${#survivors[@]} > 0 )); then
    kill -KILL "${survivors[@]}" 2>/dev/null || true
  fi
}

safe_source_state() {
  if [[ -f "${ACTIVE_STATE}" ]]; then
    # shellcheck disable=SC1090
    source "${ACTIVE_STATE}"
  fi
}

write_state() {
  mkdir -p "${STATE_DIR}"
  {
    printf 'LAUNCHER_PID=%q\n' "$$"
    printf 'RUN_ID=%q\n' "${RUN_ID}"
    printf 'MODE=%q\n' "${MODE}"
    printf 'LOG_DIR=%q\n' "${LOG_DIR}"
    printf 'MANIFEST_PATH=%q\n' "${MANIFEST_PATH}"
    printf 'START_RESULT=%q\n' "${START_RESULT}"
    printf 'START_FAILURE=%q\n' "${START_FAILURE}"
    printf 'SHUTDOWN_REASON=%q\n' "${SHUTDOWN_REASON}"
    printf 'CONTROLLER_PID=%q\n' "${CONTROLLER_PID}"
    printf 'API_PID=%q\n' "${API_PID}"
    printf 'WEB_PID=%q\n' "${WEB_PID}"
  } > "${ACTIVE_STATE}"
}

write_manifest() {
  [[ -n "${MANIFEST_PATH}" ]] || return 0

  export STACK_MANIFEST_PATH="${MANIFEST_PATH}"
  export STACK_REPO_ROOT="${REPO_ROOT}"
  export STACK_RUN_ID="${RUN_ID}"
  export STACK_MODE="${MODE}"
  export STACK_LOG_DIR="${LOG_DIR}"
  export STACK_RESULT="${START_RESULT}"
  export STACK_FAILURE="${START_FAILURE}"
  export STACK_SHUTDOWN_REASON="${SHUTDOWN_REASON}"
  export STACK_CONTROLLER_PID="${CONTROLLER_PID}"
  export STACK_API_PID="${API_PID}"
  export STACK_WEB_PID="${WEB_PID}"
  export STACK_CONTROLLER_HOST="${CONTROLLER_HOST}"
  export STACK_CONTROLLER_PORT="${CONTROLLER_PORT}"
  export STACK_API_PORT="${API_PORT}"
  export STACK_WEB_PORT="${WEB_PORT}"
  export STACK_HEADLESS="${HEADLESS}"
  export STACK_LAUNCHER_PID="$$"

  "${SYSTEM_PYTHON}" - <<'PY'
import json
import os
import subprocess
from pathlib import Path

def _int_or_none(value: str):
    value = (value or "").strip()
    return int(value) if value else None

def _git_output(args):
    try:
        return subprocess.check_output(args, cwd=os.environ["STACK_REPO_ROOT"], text=True).strip()
    except Exception:
        return None

payload = {
    "run_id": os.environ.get("STACK_RUN_ID", ""),
    "mode": os.environ.get("STACK_MODE", ""),
    "headless": os.environ.get("STACK_HEADLESS", "0") == "1",
    "repo_root": os.environ.get("STACK_REPO_ROOT", ""),
    "log_dir": os.environ.get("STACK_LOG_DIR", ""),
    "launcher_pid": _int_or_none(os.environ.get("STACK_LAUNCHER_PID", "")),
    "result": os.environ.get("STACK_RESULT", ""),
    "failure": os.environ.get("STACK_FAILURE", ""),
    "shutdown_reason": os.environ.get("STACK_SHUTDOWN_REASON", ""),
    "controller": {
        "host": os.environ.get("STACK_CONTROLLER_HOST", ""),
        "port": _int_or_none(os.environ.get("STACK_CONTROLLER_PORT", "")),
        "pid": _int_or_none(os.environ.get("STACK_CONTROLLER_PID", "")),
        "command": "./run.sh",
    },
    "api": {
        "port": _int_or_none(os.environ.get("STACK_API_PORT", "")),
        "pid": _int_or_none(os.environ.get("STACK_API_PID", "")),
        "command": "./run-api.sh",
    },
    "web": {
        "port": _int_or_none(os.environ.get("STACK_WEB_PORT", "")),
        "pid": _int_or_none(os.environ.get("STACK_WEB_PID", "")),
        "command": "./run-web.sh",
    },
    "git": {
        "branch": _git_output(["git", "rev-parse", "--abbrev-ref", "HEAD"]),
        "commit": _git_output(["git", "rev-parse", "HEAD"]),
    },
}

path = Path(os.environ["STACK_MANIFEST_PATH"])
path.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")
PY
}

service_label() {
  case "$1" in
    controller) printf 'controller' ;;
    api) printf 'api' ;;
    web) printf 'web' ;;
    *) printf '%s' "$1" ;;
  esac
}

start_tail() {
  local name="$1"
  local log_file="$2"
  local label
  label="$(service_label "${name}")"

  "${SYSTEM_PYTHON}" -u - "${label}" "${log_file}" <<'PY' &
import io
import os
import sys
import time

label = sys.argv[1]
path = sys.argv[2]

with open(path, "r", encoding="utf-8", errors="replace") as handle:
    while True:
        line = handle.readline()
        if line:
            sys.stdout.write(f"[{label}] {line}")
            sys.stdout.flush()
            continue
        time.sleep(0.1)
PY
  TAIL_PIDS+=("$!")
}

stop_tailers() {
  local pid
  for pid in "${TAIL_PIDS[@]:-}"; do
    if pid_is_live "${pid}"; then
      kill "${pid}" 2>/dev/null || true
    fi
  done
}

interactive_console_enabled() {
  case "${INTERACTIVE_CONSOLE_MODE,,}" in
    0|false|no|off)
      return 1
      ;;
  esac

  [[ -t 0 ]] || return 1
  [[ -n "${TTY_DEVICE}" && -r "${TTY_DEVICE}" && -w "${TTY_DEVICE}" ]] || return 1
  [[ "${TERM:-}" != "dumb" ]] || return 1
  return 0
}

stop_child_process() {
  local name="$1"
  local pid="$2"
  stop_pid_hierarchy "${name}" "${pid}" 50
}

send_controller_command_udp() {
  local command="$1"
  local timeout_s="${2:-1.5}"

  "${SYSTEM_PYTHON}" - "${CONTROLLER_HOST}" "${CONTROLLER_PORT}" "${command}" "${timeout_s}" <<'PY'
import socket
import sys
from contextlib import closing

host = sys.argv[1]
port = int(sys.argv[2])
command = sys.argv[3]
timeout_s = float(sys.argv[4])

with closing(socket.socket(socket.AF_INET, socket.SOCK_DGRAM)) as sock:
    sock.settimeout(timeout_s)
    try:
        sock.sendto(command.encode("utf-8"), (host, port))
        payload, _ = sock.recvfrom(4096)
    except Exception as exc:
        print(str(exc))
        raise SystemExit(1)

detail = payload.decode("utf-8", errors="replace").strip()
if not detail:
    print("empty controller reply")
    raise SystemExit(1)

print(detail)
PY
}

request_controller_stop_via_api() {
  if ! probe_api_health >/dev/null 2>&1; then
    return 1
  fi

  local detail=""
  if detail="$(curl -fsS --max-time 2 -X POST "http://127.0.0.1:${API_PORT}/control/stop" 2>&1)"; then
    log "Issued /control/stop before controller shutdown: ${detail}"
    return 0
  fi

  warn "Failed to issue /control/stop before controller shutdown: ${detail}"
  return 1
}

request_controller_power_down_via_api() {
  if ! probe_api_health >/dev/null 2>&1; then
    return 1
  fi

  local detail=""
  if detail="$(curl -fsS --max-time 5 -X POST "http://127.0.0.1:${API_PORT}/control/power-down" 2>&1)"; then
    log "Issued /control/power-down before controller shutdown: ${detail}"
    return 0
  fi

  warn "Failed to issue /control/power-down before controller shutdown: ${detail}"
  return 1
}

request_controller_power_down_via_udp() {
  local detail=""
  if detail="$(send_controller_command_udp "SAFE_POWER_DOWN" 3.0 2>&1)"; then
    log "Issued SAFE_POWER_DOWN over UDP before controller shutdown: ${detail}"
    return 0
  fi

  warn "Failed to issue SAFE_POWER_DOWN over UDP before controller shutdown: ${detail}"
  return 1
}

request_controller_safe_power_down() {
  if [[ "${SHUTDOWN_POWER_DOWN_ATTEMPTED}" -eq 1 ]]; then
    return 0
  fi
  SHUTDOWN_POWER_DOWN_ATTEMPTED=1
  request_controller_power_down_via_api && return 0
  request_controller_power_down_via_udp && return 0
  request_controller_stop_via_api || true
  return 1
}

stop_controller_process() {
  local pid="$1"
  stop_pid_hierarchy \
    "controller" \
    "${pid}" \
    30 \
    "Stopping controller gracefully with SIGTERM (pid=${pid})" \
    "controller did not exit after SIGTERM; sending SIGKILL"
}

cleanup() {
  local exit_code=$?
  trap - EXIT INT TERM

  if [[ "${MANAGE_CHILDREN}" -eq 1 ]]; then
    MANAGE_CHILDREN=0
    perform_shutdown_sequence
    stop_tailers
    rm -f "${ACTIVE_STATE}"
    write_manifest
  fi

  exit "${exit_code}"
}

handle_signal() {
  if [[ "${STOP_REQUESTED}" -eq 1 ]]; then
    return 0
  fi
  STOP_REQUESTED=1
  SHUTDOWN_REASON="signal"
  if [[ "${START_RESULT}" == "running" ]]; then
    START_RESULT="stopped"
  fi
  write_state
  write_manifest
  log "Signal received; stopping supervised stack."
  return 0
}

trap cleanup EXIT
trap handle_signal INT TERM

probe_controller() {
  "${SYSTEM_PYTHON}" - "${CONTROLLER_HOST}" "${CONTROLLER_PORT}" <<'PY'
import socket
import sys
from contextlib import closing

host = sys.argv[1]
port = int(sys.argv[2])

with closing(socket.socket(socket.AF_INET, socket.SOCK_DGRAM)) as sock:
    sock.settimeout(0.5)
    try:
        sock.sendto(b"GET_STATUS", (host, port))
        payload, _ = sock.recvfrom(4096)
    except Exception as exc:
        print(str(exc))
        raise SystemExit(1)

detail = payload.decode("utf-8", errors="replace").strip()
if not detail:
    print("empty status reply")
    raise SystemExit(1)

print(detail)
PY
}

probe_api_health() {
  curl -fsS --max-time 1 "http://127.0.0.1:${API_PORT}/health"
}

probe_api_runtime_config() {
  curl -fsS --max-time 2 "http://127.0.0.1:${API_PORT}/info/runtime-config"
}

probe_api_joints() {
  curl -fsS --max-time 2 "http://127.0.0.1:${API_PORT}/info/joints"
}

probe_api_pose() {
  curl -fsS --max-time 2 "http://127.0.0.1:${API_PORT}/info/pose"
}

probe_web() {
  curl -fsS --max-time 1 "http://127.0.0.1:${WEB_PORT}/"
}

probe_hardware_state_json() {
  "${PROBE_PYTHON}" - "${RTCORE_METRICS_PATH}" "${CONTROLLER_HOST}" "${CONTROLLER_PORT}" "${API_PORT}" "${REPO_ROOT}" <<'PY'
import json
import io
import socket
import sys
import urllib.error
import urllib.request
from contextlib import closing
from contextlib import redirect_stdout
from pathlib import Path

metrics_path = Path(sys.argv[1])
controller_host = sys.argv[2]
controller_port = int(sys.argv[3])
api_port = int(sys.argv[4])
repo_root = Path(sys.argv[5])
socket_path = metrics_path.with_name("ipc.sock")
sys.path.insert(0, str(repo_root / "src"))
try:
    with redirect_stdout(io.StringIO()):
        from gradient_os import runtime_config
        from gradient_os.arm_controller.backends import registry as backend_registry
        from gradient_os.arm_controller.robots import get_robot_config
        from gradient_os.telemetry.drive_faults import build_drive_fault_snapshot
except Exception:
    runtime_config = None
    backend_registry = None
    get_robot_config = None
    build_drive_fault_snapshot = None


def desired_runtime_fallback():
    if runtime_config is None:
        return {}, [], None
    try:
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
            requested_active_tool_id=desired.get("active_tool_id"),
            allow_unsafe_overrides=allow_unsafe,
        )
        axis_to_joint: list[int] = []
        robot_name = str(resolved.get("robot", {}).get("name", "")).strip()
        if robot_name and callable(get_robot_config):
            robot_cfg = get_robot_config(robot_name).get_config_dict()
            mapping = robot_cfg.get("logical_to_physical_map", {})
            if isinstance(mapping, dict):
                num_axes = int(robot_cfg.get("num_physical_actuators", 0))
                axis_to_joint = [-1] * max(0, num_axes)
                for logical_joint, physical_axes in mapping.items():
                    try:
                        logical_joint_idx = int(logical_joint)
                    except Exception:
                        continue
                    if not isinstance(physical_axes, list):
                        continue
                    for axis_idx_raw in physical_axes:
                        try:
                            axis_idx = int(axis_idx_raw)
                        except Exception:
                            continue
                        if 0 <= axis_idx < len(axis_to_joint):
                            axis_to_joint[axis_idx] = logical_joint_idx
        return resolved, axis_to_joint, None
    except Exception as exc:
        return {}, [], str(exc)


def api_json(path: str):
    url = f"http://127.0.0.1:{api_port}{path}"
    try:
        with urllib.request.urlopen(url, timeout=1.5) as resp:
            return json.loads(resp.read().decode("utf-8")), None
    except Exception as exc:
        return None, str(exc)


def controller_status():
    with closing(socket.socket(socket.AF_INET, socket.SOCK_DGRAM)) as sock:
        sock.settimeout(0.5)
        try:
            sock.sendto(b"GET_STATUS", (controller_host, controller_port))
            payload, _ = sock.recvfrom(4096)
        except Exception as exc:
            return False, str(exc)
    detail = payload.decode("utf-8", errors="replace").strip()
    return bool(detail), detail or "empty reply"


def load_metrics():
    try:
        data = json.loads(metrics_path.read_text(encoding="utf-8"))
        if isinstance(data, dict):
            return data, None
        return {}, "metrics file did not contain a JSON object"
    except FileNotFoundError:
        return {}, "metrics file does not exist"
    except Exception as exc:
        return {}, f"{exc.__class__.__name__}: {exc}"


controller_ok, controller_detail = controller_status()
api_health, api_health_err = api_json("/health")
runtime_cfg, runtime_cfg_err = api_json("/info/runtime-config")
metrics, metrics_err = load_metrics()
runtime_active = runtime_cfg.get("active") if isinstance(runtime_cfg, dict) else None
ik = runtime_active.get("ik_solver") if isinstance(runtime_active, dict) else {}
servo = runtime_active.get("servo_backend") if isinstance(runtime_active, dict) else {}
drive = runtime_active.get("drive_profile") if isinstance(runtime_active, dict) else {}
if not isinstance(ik, dict):
    ik = {}
if not isinstance(servo, dict):
    servo = {}
if not isinstance(drive, dict):
    drive = {}
runtime_servo_backend = str(servo.get("effective_backend", "")).strip() or None
runtime_drive_profile = str(drive.get("effective_profile", "")).strip() or None
runtime_drive_profile_configured = str(
    drive.get("configured_profile", drive.get("effective_profile", ""))
).strip() or None
runtime_drive_profile_live = str(drive.get("live_profile", "")).strip() or None
runtime_drive_profile_source = str(drive.get("source", "")).strip() or None
fallback_runtime, fallback_axis_to_joint, fallback_error = desired_runtime_fallback()
fallback_servo = (
    fallback_runtime.get("servo_backend", {}).get("effective_backend")
    if isinstance(fallback_runtime.get("servo_backend"), dict)
    else None
)
fallback_drive = (
    fallback_runtime.get("drive_profile", {}).get("configured_profile")
    if isinstance(fallback_runtime.get("drive_profile"), dict)
    else None
)
probe_servo_backend = runtime_servo_backend or (str(fallback_servo).strip() or None)
probe_drive_profile = runtime_drive_profile or runtime_drive_profile_configured or (str(fallback_drive).strip() or None)
probe_context_source = (
    "api_active_runtime"
    if runtime_servo_backend
    else ("desired_runtime_fallback" if probe_servo_backend else "unavailable")
)

if not metrics:
    payload = {
        "controller_udp_up": controller_ok,
        "controller_detail": controller_detail,
        "api_http_up": bool(api_health),
        "api_error": api_health_err,
        "runtime_config_available": bool(runtime_cfg),
        "runtime_config_error": runtime_cfg_err,
        "runtime_ik": ik.get("effective_backend"),
        "runtime_ik_source": ik.get("source"),
        "runtime_servo": runtime_servo_backend,
        "runtime_drive_profile": runtime_drive_profile,
        "runtime_drive_profile_configured": runtime_drive_profile_configured,
        "runtime_drive_profile_live": runtime_drive_profile_live,
        "runtime_drive_profile_source": runtime_drive_profile_source,
        "probe_servo_backend": probe_servo_backend,
        "probe_drive_profile": probe_drive_profile,
        "probe_context_source": probe_context_source,
        "probe_context_error": fallback_error,
        "drive_fault_reference": None,
        "metrics_available": False,
        "metrics_error": metrics_err,
        "rtcore_socket_present": socket_path.exists(),
        "rtcore_state": "DOWN" if not socket_path.exists() else "UNKNOWN",
        "ethercat_master_state": "DOWN",
        "driver_state": "INACTIVE",
        "physical_state": "INACTIVE" if not controller_ok and not api_health else "UNKNOWN",
        "armed": 0,
        "axis_enable_mask": 0,
        "op_enabled_axes": 0,
        "num_axes": 0,
        "link_up": 0,
        "responding": 0,
        "online": 0,
        "operational": 0,
        "startup_ready": 0,
        "wkc_actual": 0,
        "wkc_expected": 0,
        "master_al": 0,
        "axes": [],
        "metrics_path": str(metrics_path),
    }
    print(json.dumps(payload))
    raise SystemExit(0)
snapshot = {}
if callable(build_drive_fault_snapshot):
    try:
        snapshot = build_drive_fault_snapshot(
            metrics=metrics,
            servo_backend=probe_servo_backend,
            drive_profile=probe_drive_profile,
            configured_drive_profile=runtime_drive_profile_configured or (str(fallback_drive).strip() or None),
            live_drive_profile=runtime_drive_profile_live,
            axis_to_joint=fallback_axis_to_joint,
            socket_present=socket_path.exists(),
        )
    except Exception:
        snapshot = {}

payload = {
    "controller_udp_up": controller_ok,
    "controller_detail": controller_detail,
    "api_http_up": bool(api_health),
    "api_error": api_health_err,
    "runtime_config_available": bool(runtime_cfg),
    "runtime_config_error": runtime_cfg_err,
    "runtime_ik": ik.get("effective_backend"),
    "runtime_ik_source": ik.get("source"),
    "runtime_servo": runtime_servo_backend or None,
    "runtime_drive_profile": runtime_drive_profile,
    "runtime_drive_profile_configured": runtime_drive_profile_configured,
    "runtime_drive_profile_live": runtime_drive_profile_live,
    "runtime_drive_profile_source": runtime_drive_profile_source,
    "probe_servo_backend": probe_servo_backend,
    "probe_drive_profile": probe_drive_profile,
    "probe_context_source": probe_context_source,
    "probe_context_error": fallback_error,
    "runtime_restart_required": runtime_cfg.get("restart_required") if isinstance(runtime_cfg, dict) else None,
    "drive_fault_reference": snapshot.get("reference") if isinstance(snapshot, dict) else None,
    "metrics_available": True,
    "metrics_error": None,
    "rtcore_socket_present": socket_path.exists(),
    "metrics_path": str(metrics_path),
}
payload.update(snapshot if isinstance(snapshot, dict) else {})
print(json.dumps(payload))
PY
}

probe_hardware_state() {
  local payload
  if ! payload="$(probe_hardware_state_json 2>/dev/null)"; then
    error "hardware probe failed"
    return 1
  fi

  "${SYSTEM_PYTHON}" - "${payload}" <<'PY'
import json
import sys

data = json.loads(sys.argv[1])
print("hardware_probe:")
print(
    f"  controller_udp: {'up' if data.get('controller_udp_up') else 'down'} "
    f"({data.get('controller_detail')})"
)
print(f"  api_http: {'up' if data.get('api_http_up') else 'down'}")
if data.get("api_error"):
    print(f"    api_error={data['api_error']}")
if data.get("runtime_config_available"):
    print(
        "  runtime_config:"
        f" ik={data.get('runtime_ik', 'unknown')}"
        f" source={data.get('runtime_ik_source', 'unknown')}"
        f" servo={data.get('runtime_servo', 'unknown')}"
        f" drive_profile={data.get('runtime_drive_profile', 'unknown')}"
        f" drive_source={data.get('runtime_drive_profile_source', 'unknown')}"
        f" configured={data.get('runtime_drive_profile_configured', 'unknown')}"
        f" live={data.get('runtime_drive_profile_live') or 'unavailable'}"
        f" restart_required={data.get('runtime_restart_required')}"
    )
elif data.get("runtime_config_error"):
    print(f"  runtime_config: unavailable ({data['runtime_config_error']})")
if data.get("probe_context_source") and data.get("probe_context_source") != "api_active_runtime":
    print(
        "  probe_decode:"
        f" backend={data.get('probe_servo_backend') or 'unknown'}"
        f" drive_profile={data.get('probe_drive_profile') or 'unknown'}"
        f" source={data.get('probe_context_source')}"
    )
    if data.get("probe_context_error"):
        print(f"    probe_context_error={data['probe_context_error']}")

drive_fault_reference = data.get("drive_fault_reference") or data.get("reference")
if isinstance(drive_fault_reference, dict) and drive_fault_reference.get("available"):
    print(
        "  drive_fault_reference:"
        f" {drive_fault_reference.get('label') or drive_fault_reference.get('profile_id') or 'configured'}"
        " (only valid for the current servo backend)"
    )
elif data.get("probe_servo_backend") or data.get("runtime_servo"):
    print(
        "  drive_fault_reference:"
        f" raw_only (no vendor-specific decode applied for backend={data.get('probe_servo_backend') or data.get('runtime_servo')})"
    )
else:
    print("  drive_fault_reference: raw_only (active servo backend unavailable)")

print(
    f"  driver_state: {data.get('driver_state')}"
    f" (armed={data.get('armed')} enable_mask=0x{int(data.get('axis_enable_mask', 0)):x} "
    f"op_enabled_axes={data.get('op_enabled_axes')}/{data.get('num_axes')})"
)
print(
    "  ethercat_master_state:"
    f" {data.get('ethercat_master_state')}"
    f" link_up={data.get('link_up')}"
    f" responding={data.get('responding')}/{data.get('num_axes')}"
    f" online={data.get('online')}/{data.get('num_axes')}"
    f" operational={data.get('operational')}/{data.get('num_axes')}"
    f" wkc={data.get('wkc_actual')}/{data.get('wkc_expected')}"
    f" startup_ready={data.get('startup_ready')}"
    f" master_al=0x{int(data.get('master_al', 0)):x}"
)
print(
    f"  rtcore_state: {data.get('rtcore_state')} "
    f"(socket_present={1 if data.get('rtcore_socket_present') else 0})"
)
print(f"  physical_state: {data.get('physical_state')}")
print(f"  metrics_path: {data.get('metrics_path')}")
for axis in data.get("axes", []):
    fault = axis.get("fault") if isinstance(axis, dict) else None
    fault_suffix = ""
    if isinstance(fault, dict):
        details = []
        code = str(fault.get("code", "")).strip()
        name = str(fault.get("name", "")).strip()
        if code:
            details.append(code)
        if name:
            details.append(name)
        if fault.get("resettable") is True:
            details.append("resettable")
        if details:
            fault_suffix = f" [{' | '.join(details)}]"
    print(
        "  "
        f"{('J' + str(axis.get('logical_joint')) + '/axis' + str(axis.get('axis', axis.get('index')))) if axis.get('logical_joint') is not None else ('axis' + str(axis.get('axis', axis.get('index'))))}: ds402={axis.get('ds402_state')} "
        f"sw=0x{int(axis.get('statusword', 0)):04x} "
        f"err=0x{int(axis.get('error_code', 0)):04x}{fault_suffix} "
        f"slave_online={axis.get('slave_online')} "
        f"slave_operational={axis.get('slave_operational')} "
        f"slave_al={axis.get('slave_al_state_name')} "
        f"pos_counts={axis.get('pos_counts')}"
    )
PY
}

runtime_config_summary() {
  local payload="$1"
  "${SYSTEM_PYTHON}" - "${payload}" <<'PY'
import json
import sys

payload = json.loads(sys.argv[1])
active_error = payload.get("active_error")
restart_required = bool(payload.get("restart_required"))
active = payload.get("active") or {}
ik = active.get("ik_solver") if isinstance(active, dict) else {}
if not isinstance(ik, dict):
    ik = {}
effective = str(ik.get("effective_backend", "")).strip() or "unknown"
source = str(ik.get("source", "")).strip() or "unknown"

if active_error:
    print(f"active_error={active_error}")
    raise SystemExit(1)
if restart_required:
    print(
        f"restart_required=true effective_backend={effective} source={source}"
    )
    raise SystemExit(2)

print(
    f"restart_required=false effective_backend={effective} source={source}"
)
PY
}

probe_json_field() {
  local payload="$1"
  local field="$2"
  "${SYSTEM_PYTHON}" - "${payload}" "${field}" <<'PY'
import json
import sys

payload = json.loads(sys.argv[1])
field = sys.argv[2]
cur = payload
for token in field.split("."):
    if isinstance(cur, dict) and token in cur:
        cur = cur[token]
    else:
        cur = ""
        break

if isinstance(cur, bool):
    print("1" if cur else "0")
elif cur is None:
    print("")
else:
    print(cur)
PY
}

capture_probe_json() {
  probe_hardware_state_json 2>/dev/null
}

log_probe_snapshot() {
  local payload="$1"
  "${SYSTEM_PYTHON}" - "${payload}" <<'PY'
import json
import sys

data = json.loads(sys.argv[1])
print(
    "[start-stack] probe:"
    f" physical_state={data.get('physical_state')}"
    f" driver_state={data.get('driver_state')}"
    f" ethercat_master_state={data.get('ethercat_master_state')}"
    f" rtcore_state={data.get('rtcore_state')}"
    f" armed={data.get('armed')}"
    f" enable_mask=0x{int(data.get('axis_enable_mask', 0)):x}"
    f" op_enabled_axes={data.get('op_enabled_axes')}/{data.get('num_axes')}"
    f" wkc={data.get('wkc_actual')}/{data.get('wkc_expected')}"
)
PY
}

probe_is_soft_stop_safe() {
  local payload="$1"
  "${SYSTEM_PYTHON}" - "${payload}" <<'PY'
import json
import sys

data = json.loads(sys.argv[1])
physical_state = str(data.get("physical_state", "")).strip().upper()
armed = int(data.get("armed", 0) or 0)
enable_mask = int(data.get("axis_enable_mask", 0) or 0)
op_enabled_axes = int(data.get("op_enabled_axes", 0) or 0)

safe = (
    physical_state in {"BUS_UP_DISARMED", "INACTIVE"}
    or (
        physical_state == "FAULTED"
        and armed == 0
        and enable_mask == 0
        and op_enabled_axes == 0
    )
)
raise SystemExit(0 if safe else 1)
PY
}

describe_probe_soft_stop_state() {
  local payload="$1"
  "${SYSTEM_PYTHON}" - "${payload}" <<'PY'
import json
import sys

data = json.loads(sys.argv[1])
physical_state = str(data.get("physical_state", "")).strip().upper() or "UNKNOWN"
armed = int(data.get("armed", 0) or 0)
enable_mask = int(data.get("axis_enable_mask", 0) or 0)
op_enabled_axes = int(data.get("op_enabled_axes", 0) or 0)
driver_state = str(data.get("driver_state", "")).strip().upper() or "UNKNOWN"

if physical_state == "INACTIVE":
    print("inactive")
elif physical_state == "BUS_UP_DISARMED":
    print("bus up and disarmed")
elif (
    physical_state == "FAULTED"
    and armed == 0
    and enable_mask == 0
    and op_enabled_axes == 0
):
    print("faults latched, but robot is already electrically disarmed")
else:
    print(
        f"robot still active or unresolved "
        f"(physical_state={physical_state} driver_state={driver_state} "
        f"armed={armed} enable_mask=0x{enable_mask:x} op_enabled_axes={op_enabled_axes})"
    )
PY
}

wait_for_probe_state() {
  local desired_state="$1"
  local timeout_s="$2"
  local started_at
  started_at="$(date +%s)"

  while true; do
    local payload=""
    payload="$(capture_probe_json || true)"
    if [[ -n "${payload}" ]]; then
      local current_state=""
      current_state="$(probe_json_field "${payload}" "physical_state")"
      if [[ "${current_state}" == "${desired_state}" ]]; then
        log "hardware reached ${desired_state}"
        return 0
      fi
    fi

    local now
    now="$(date +%s)"
    if (( now - started_at >= timeout_s )); then
      return 1
    fi
    sleep "${PROBE_INTERVAL_S}"
  done
}

wait_for_probe_soft_stop_safe() {
  local timeout_s="$1"
  local started_at
  started_at="$(date +%s)"

  while true; do
    local payload=""
    payload="$(capture_probe_json || true)"
    if [[ -n "${payload}" ]] && probe_is_soft_stop_safe "${payload}"; then
      log "hardware reached safe soft-stop state: $(describe_probe_soft_stop_state "${payload}")"
      return 0
    fi

    local now
    now="$(date +%s)"
    if (( now - started_at >= timeout_s )); then
      return 1
    fi
    sleep "${PROBE_INTERVAL_S}"
  done
}

wait_for_bus_operational() {
  local started_at
  started_at="$(date +%s)"
  local last_detail=""

  while true; do
    local payload=""
    payload="$(capture_probe_json || true)"
    if [[ -n "${payload}" ]]; then
      local metrics_available=""
      local num_axes=""
      local responding=""
      local online=""
      local operational=""
      local startup_ready=""
      local wkc_actual=""

      metrics_available="$(probe_json_field "${payload}" "metrics_available")"
      num_axes="$(probe_json_field "${payload}" "num_axes")"
      responding="$(probe_json_field "${payload}" "responding")"
      online="$(probe_json_field "${payload}" "online")"
      operational="$(probe_json_field "${payload}" "operational")"
      startup_ready="$(probe_json_field "${payload}" "startup_ready")"
      wkc_actual="$(probe_json_field "${payload}" "wkc_actual")"

      if [[ "${metrics_available}" == "1" && -n "${num_axes}" && "${num_axes}" != "0" ]]; then
        if [[ "${responding}" == "${num_axes}" && "${online}" == "${num_axes}" && "${operational}" == "${num_axes}" && "${startup_ready}" == "1" ]]; then
          log "bus ready: responding=${responding}/${num_axes} online=${online}/${num_axes} operational=${operational}/${num_axes} wkc=${wkc_actual}"
          return 0
        fi
        local detail="responding=${responding}/${num_axes} online=${online}/${num_axes} operational=${operational}/${num_axes} startup_ready=${startup_ready} wkc=${wkc_actual}"
        if [[ "${detail}" != "${last_detail}" ]]; then
          log "Waiting for full bus readiness: ${detail}"
          last_detail="${detail}"
        fi
      fi
    fi

    local now
    now="$(date +%s)"
    if (( now - started_at >= BUS_READY_TIMEOUT_S )); then
      if [[ -n "${payload}" ]]; then
        log_probe_snapshot "${payload}"
      fi
      error "bus failed readiness within ${BUS_READY_TIMEOUT_S}s"
      return 1
    fi
    sleep "${PROBE_INTERVAL_S}"
  done
}

probe_startup_fault_reset_plan() {
  local payload="$1"
  PYTHONPATH="${REPO_ROOT}/src:${PYTHONPATH:-}" "${PROBE_PYTHON}" - "${payload}" <<'PY'
import io
import json
import sys
from contextlib import redirect_stdout

with redirect_stdout(io.StringIO()):
    from gradient_os.telemetry.drive_faults import build_startup_fault_reset_plan

payload = json.loads(sys.argv[1])
print(json.dumps(build_startup_fault_reset_plan(payload)))
PY
}

direct_rtcore_fault_reset_mask() {
  local axis_mask="$1"
  if [[ ! -x "${PROJECT_PYTHON}" ]]; then
    warn "direct RTCore fault reset unavailable: missing ${PROJECT_PYTHON}"
    return 1
  fi

  local detail=""
  if detail="$(
    PYTHONPATH="${REPO_ROOT}/src:${PYTHONPATH:-}" \
    "${PROJECT_PYTHON}" "${REPO_ROOT}/scripts/rtcore_jog.py" fault_reset --mask "${axis_mask}" 2>&1
  )"; then
    log "Issued direct RTCore fault reset: axis_mask=${axis_mask}"
    if [[ -n "${detail}" ]]; then
      while IFS= read -r line; do
        [[ -n "${line}" ]] || continue
        log "${line}"
      done <<< "${detail}"
    fi
    return 0
  fi

  warn "Direct RTCore fault reset failed for axis_mask=${axis_mask}: ${detail}"
  return 1
}

startup_fault_reset_preflight() {
  if ! wait_for_bus_operational; then
    return 1
  fi

  local payload=""
  payload="$(capture_probe_json || true)"
  if [[ -z "${payload}" ]]; then
    error "startup preflight could not capture RTCore probe data"
    return 1
  fi

  local plan=""
  if ! plan="$(probe_startup_fault_reset_plan "${payload}" 2>/dev/null)"; then
    log_probe_snapshot "${payload}"
    error "startup preflight could not build a fault-reset plan from the probe payload"
    return 1
  fi

  local should_auto_reset=""
  local blocks_startup=""
  local axis_mask_hex=""
  local fault_summary=""
  local reason=""
  should_auto_reset="$(probe_json_field "${plan}" "should_auto_reset")"
  blocks_startup="$(probe_json_field "${plan}" "blocks_startup")"
  axis_mask_hex="$(probe_json_field "${plan}" "faulted_axis_mask_hex")"
  fault_summary="$(probe_json_field "${plan}" "faulted_summary")"
  reason="$(probe_json_field "${plan}" "reason")"

  if [[ "${should_auto_reset}" == "1" ]]; then
    log "startup preflight found disarmed drive faults before any drive power-up: ${fault_summary:-axis_mask=${axis_mask_hex}}"
    if ! direct_rtcore_fault_reset_mask "${axis_mask_hex}"; then
      error "startup fault-reset preflight failed to send the RTCore reset pulse"
      return 1
    fi
    if ! wait_for_probe_state "BUS_UP_DISARMED" "${STARTUP_FAULT_RESET_TIMEOUT_S}"; then
      local post_reset_probe=""
      post_reset_probe="$(capture_probe_json || true)"
      if [[ -n "${post_reset_probe}" ]]; then
        log_probe_snapshot "${post_reset_probe}"
        local post_reset_plan=""
        post_reset_plan="$(probe_startup_fault_reset_plan "${post_reset_probe}" 2>/dev/null || true)"
        if [[ -n "${post_reset_plan}" ]]; then
          local remaining_faults=""
          remaining_faults="$(probe_json_field "${post_reset_plan}" "faulted_summary")"
          if [[ -n "${remaining_faults}" ]]; then
            warn "startup preflight faults still present after reset: ${remaining_faults}"
          fi
        fi
      fi
      error "startup fault-reset preflight did not clear the disarmed drive faults; refusing to start controller"
      return 1
    fi
    log "startup fault-reset preflight cleared the disarmed drive faults"
    return 0
  fi

  if [[ "${blocks_startup}" == "1" ]]; then
    log_probe_snapshot "${payload}"
    error "startup preflight found faulted hardware that is not safe to auto-reset (${reason}); refusing to start controller"
    return 1
  fi

  log "startup preflight: no disarmed drive faults detected"
  return 0
}

direct_rtcore_safe_power_down() {
  if [[ ! -x "${PROJECT_PYTHON}" ]]; then
    warn "direct RTCore power-down unavailable: missing ${PROJECT_PYTHON}"
    return 1
  fi

  local detail=""
  if detail="$(
    PYTHONPATH="${REPO_ROOT}/src:${PYTHONPATH:-}" \
    GRADIENT_RTCORE_AUTO_ARM=0 \
    "${PROJECT_PYTHON}" - <<'PY' 2>&1
from gradient_os import runtime_config
from gradient_os.arm_controller.robots import get_robot_config
from gradient_os.arm_controller.backends.ethercat_rtcore.backend import EthercatRTCoreBackend

cfg_name = runtime_config.load_runtime_config().get("desired", {}).get("robot")
robot_cfg = get_robot_config(str(cfg_name or "").strip()).get_config_dict()
backend = EthercatRTCoreBackend(robot_config=robot_cfg)
if not backend.initialize():
    raise SystemExit("failed to connect to RTCore IPC")
try:
    backend.safe_power_down()
    print("direct_rtcore_power_down_sent")
finally:
    backend.shutdown()
PY
  )"; then
    log "Issued direct RTCore power-down: ${detail}"
    return 0
  fi

  warn "Direct RTCore power-down failed: ${detail}"
  return 1
}

run_systemctl() {
  if [[ "$(id -u)" -eq 0 ]]; then
    systemctl "$@"
  else
    sudo -n systemctl "$@"
  fi
}

systemd_service_is_active() {
  run_systemctl is-active --quiet "$1" >/dev/null 2>&1
}

stop_systemd_service_if_active() {
  local service_name="$1"
  if ! systemd_service_is_active "${service_name}"; then
    return 1
  fi
  log "Stopping systemd service ${service_name}"
  if run_systemctl stop "${service_name}" >/dev/null 2>&1; then
    return 0
  fi
  warn "Failed to stop ${service_name} via systemd"
  return 1
}

discover_pid_by_pattern() {
  local pattern="$1"
  pgrep -f -n "${pattern}" 2>/dev/null || true
}

stop_external_process_by_pid() {
  local name="$1"
  local pid="$2"
  if ! pid_is_live "${pid}"; then
    return 1
  fi
  stop_pid_hierarchy \
    "${name} external" \
    "${pid}" \
    80 \
    "Stopping ${name} external pid=${pid}" \
    "${name} external pid ${pid} did not exit after SIGTERM; sending SIGKILL"
  return 0
}

stop_rtcore_runtime() {
  if stop_systemd_service_if_active "${RTCORE_SERVICE_NAME}"; then
    return 0
  fi
  local rtcore_pid=""
  rtcore_pid="$(discover_pid_by_pattern 'gradient-rt-motion')"
  if [[ -n "${rtcore_pid}" ]]; then
    stop_external_process_by_pid "rtcore" "${rtcore_pid}" || true
    return 0
  fi
  return 1
}

stop_ethercat_master_runtime() {
  stop_systemd_service_if_active "${ETHERCAT_SERVICE_NAME}" || true
}

perform_shutdown_sequence() {
  local initial_probe=""
  initial_probe="$(capture_probe_json || true)"
  if [[ -n "${initial_probe}" ]]; then
    log_probe_snapshot "${initial_probe}"
  fi

  stop_child_process "web" "${WEB_PID}"

  local current_state=""
  local hardware_safe_for_soft_stop=1
  if [[ -n "${initial_probe}" ]]; then
    current_state="$(probe_json_field "${initial_probe}" "physical_state")"
    if ! probe_is_soft_stop_safe "${initial_probe}"; then
      hardware_safe_for_soft_stop=0
    else
      log "soft-stop probe already safe: $(describe_probe_soft_stop_state "${initial_probe}")"
    fi
  else
    hardware_safe_for_soft_stop=0
  fi

  if [[ "${hardware_safe_for_soft_stop}" -eq 0 ]]; then
    request_controller_safe_power_down || true
    if ! wait_for_probe_soft_stop_safe 3; then
      stop_controller_process "${CONTROLLER_PID}"
      direct_rtcore_safe_power_down || true
      if ! wait_for_probe_soft_stop_safe 2; then
        local unresolved_probe=""
        unresolved_probe="$(capture_probe_json || true)"
        if [[ -n "${unresolved_probe}" ]]; then
          warn "soft-stop probe still unresolved: $(describe_probe_soft_stop_state "${unresolved_probe}")"
        else
          warn "soft-stop probe still unresolved and no probe data is available"
        fi
      fi
    fi
  fi

  stop_controller_process "${CONTROLLER_PID}"
  stop_child_process "api" "${API_PID}"

  local mid_probe=""
  mid_probe="$(capture_probe_json || true)"
  if [[ -n "${mid_probe}" ]]; then
    log_probe_snapshot "${mid_probe}"
    current_state="$(probe_json_field "${mid_probe}" "physical_state")"
  else
    current_state=""
  fi

  if [[ "${HARD_STOP}" -eq 1 ]]; then
    if [[ "${current_state}" == "ACTIVE" || "${current_state}" == "BUS_UP_DISARMED" || "${current_state}" == "FAULTED" || -z "${current_state}" ]]; then
      stop_rtcore_runtime || true
      sleep 1
    fi

    stop_ethercat_master_runtime
  else
    log "Soft stop complete: leaving RTCore and EtherCAT master up in the disarmed state."
  fi

  local final_probe=""
  final_probe="$(capture_probe_json || true)"
  if [[ -n "${final_probe}" ]]; then
    log_probe_snapshot "${final_probe}"
    if probe_is_soft_stop_safe "${final_probe}"; then
      log "soft-stop result: $(describe_probe_soft_stop_state "${final_probe}")"
    fi
  else
    log "final probe unavailable (RTCore metrics likely gone)"
  fi
}

wait_for_probe() {
  local name="$1"
  local timeout_s="$2"
  local probe_func="$3"

  local started_at
  started_at="$(date +%s)"
  local last_detail=""

  while true; do
    local detail=""
    if detail="$(${probe_func} 2>&1)"; then
      log "${name} ready: ${detail}"
      return 0
    fi

    if [[ "${detail}" != "${last_detail}" ]]; then
      log "Waiting for ${name}: ${detail}"
      last_detail="${detail}"
    fi

    local now
    now="$(date +%s)"
    if (( now - started_at >= timeout_s )); then
      error "${name} failed readiness within ${timeout_s}s: ${detail}"
      return 1
    fi

    sleep "${PROBE_INTERVAL_S}"
  done
}

wait_for_controller_readiness() {
  local started_at
  started_at="$(date +%s)"
  local last_detail=""

  while true; do
    if ! pid_is_live "${CONTROLLER_PID}"; then
      error "controller exited before readiness completed"
      return 1
    fi

    local detail=""
    if detail="$(probe_controller 2>&1)"; then
      log "controller ready: ${detail}"
      return 0
    fi

    if [[ "${detail}" != "${last_detail}" ]]; then
      log "Waiting for controller: ${detail}"
      last_detail="${detail}"
    fi

    local now
    now="$(date +%s)"
    if (( now - started_at >= CONTROLLER_TIMEOUT_S )); then
      error "controller failed readiness within ${CONTROLLER_TIMEOUT_S}s: ${detail}"
      return 1
    fi

    sleep "${PROBE_INTERVAL_S}"
  done
}

wait_for_api_readiness() {
  wait_for_probe "api health" "${API_TIMEOUT_S}" probe_api_health || return 1

  local runtime_payload
  if ! runtime_payload="$(probe_api_runtime_config 2>&1)"; then
    error "failed to fetch /info/runtime-config after API health check: ${runtime_payload}"
    return 1
  fi

  local runtime_summary
  if ! runtime_summary="$(runtime_config_summary "${runtime_payload}" 2>&1)"; then
    error "runtime-config sanity check failed: ${runtime_summary}"
    return 1
  fi
  log "runtime-config sanity: ${runtime_summary}"

  wait_for_probe "api joints" 10 probe_api_joints >/dev/null || return 1
  wait_for_probe "api pose" 10 probe_api_pose >/dev/null || return 1
  return 0
}

wait_for_web_readiness() {
  wait_for_probe "web UI" "${WEB_TIMEOUT_S}" probe_web >/dev/null
}

start_process() {
  local name="$1"
  local log_file="$2"
  shift 2

  : > "${log_file}"
  start_tail "${name}" "${log_file}"

  log "Starting ${name}: $*"
  if command -v setsid >/dev/null 2>&1; then
    if command -v stdbuf >/dev/null 2>&1; then
      setsid stdbuf -oL -eL "$@" >> "${log_file}" 2>&1 &
    else
      setsid "$@" >> "${log_file}" 2>&1 &
    fi
  elif command -v stdbuf >/dev/null 2>&1; then
    stdbuf -oL -eL "$@" >> "${log_file}" 2>&1 &
  else
    "$@" >> "${log_file}" 2>&1 &
  fi

  local pid=$!
  CHILD_PIDS+=("${pid}")
  log "${name} started with pid=${pid}, log=${log_file}"
  STARTED_PID="${pid}"
}

ensure_required_files() {
  local path
  if [[ ! -f "${START_SH}" ]]; then
    error "required file missing: ${START_SH}"
    return 1
  fi

  for path in "${CONTROLLER_SCRIPT}" "${API_SCRIPT}"; do
    if [[ ! -f "${path}" ]]; then
      error "required file missing: ${path}"
      return 1
    fi
    if [[ ! -x "${path}" ]]; then
      error "required launcher is not executable: ${path}"
      return 1
    fi
  done
  if [[ ! -f "${RTCORE_SYNC_SCRIPT}" ]]; then
    error "required file missing: ${RTCORE_SYNC_SCRIPT}"
    return 1
  fi
  if [[ ! -x "${RTCORE_SYNC_SCRIPT}" ]]; then
    error "required helper is not executable: ${RTCORE_SYNC_SCRIPT}"
    return 1
  fi
  if [[ "${HEADLESS}" -eq 0 ]]; then
    if [[ ! -f "${WEB_SCRIPT}" ]]; then
      error "required file missing: ${WEB_SCRIPT}"
      return 1
    fi
    if [[ ! -x "${WEB_SCRIPT}" ]]; then
      error "required launcher is not executable: ${WEB_SCRIPT}"
      return 1
    fi
  fi
  if ! command -v curl >/dev/null 2>&1; then
    error "curl is required for readiness probes"
    return 1
  fi
  return 0
}

bootstrap_environment() {
  if [[ ! -f "${START_SH}" ]]; then
    error "missing ${START_SH}"
    return 1
  fi

  log "Bootstrapping environment via ${START_SH}"
  # shellcheck disable=SC1090
  source "${START_SH}"

  export PYTHONUNBUFFERED=1
  export GRADIENT_RTCORE_READY_TIMEOUT_S="${GRADIENT_RTCORE_READY_TIMEOUT_S:-30}"
  return 0
}

ensure_rtcore_runtime_sync() {
  log "Ensuring RTCore unit/env match the selected robot scaling"
  local detail=""
  if detail="$("${RTCORE_SYNC_SCRIPT}" --ensure-active 2>&1)"; then
    while IFS= read -r line; do
      [[ -n "${line}" ]] || continue
      log "${line}"
    done <<< "${detail}"
    return 0
  fi
  while IFS= read -r line; do
    [[ -n "${line}" ]] || continue
    warn "${line}"
  done <<< "${detail}"
  error "RTCore runtime sync failed; refusing to start with potentially stale scaling."
  return 1
}

detect_external_services() {
  local found=()

  if probe_controller >/dev/null 2>&1; then
    found+=("controller@${CONTROLLER_HOST}:${CONTROLLER_PORT}")
  fi
  if probe_api_health >/dev/null 2>&1; then
    found+=("api@127.0.0.1:${API_PORT}")
  fi
  if [[ "${HEADLESS}" -eq 0 ]] && probe_web >/dev/null 2>&1; then
    found+=("web@127.0.0.1:${WEB_PORT}")
  fi

  if (( ${#found[@]} > 0 )); then
    error "existing live services detected: ${found[*]}"
    error "Refusing to start duplicates. Stop the manual stack first or use ./start-stack.sh status."
    return 1
  fi

  return 0
}

setup_logging() {
  mkdir -p "${LOG_ROOT}" "${STATE_DIR}"
  RUN_ID="$(date '+%Y%m%d-%H%M%S')"
  MODE="full"
  if [[ "${HEADLESS}" -eq 1 ]]; then
    MODE="headless"
  fi

  LOG_DIR="${LOG_ROOT}/${RUN_ID}"
  mkdir -p "${LOG_DIR}"
  ln -sfn "${LOG_DIR}" "${LOG_ROOT}/latest"

  LAUNCHER_LOG="${LOG_DIR}/launcher.log"
  MANIFEST_PATH="${LOG_DIR}/manifest.json"
  CONTROLLER_LOG="${LOG_DIR}/controller.log"
  API_LOG="${LOG_DIR}/api.log"
  WEB_LOG="${LOG_DIR}/web.log"

  exec > >(tee -a "${LAUNCHER_LOG}") 2>&1

  log "Run ID: ${RUN_ID}"
  log "Logs: ${LOG_DIR}"
}

print_status() {
  local launcher_status="absent"
  local state_log_dir=""
  local state_result=""
  local state_mode=""

  if [[ -f "${ACTIVE_STATE}" ]]; then
    safe_source_state
    state_log_dir="${LOG_DIR:-}"
    state_result="${START_RESULT:-}"
    state_mode="${MODE:-}"
    if pid_is_live "${LAUNCHER_PID:-}"; then
      launcher_status="running (pid=${LAUNCHER_PID})"
    else
      launcher_status="stale state (launcher pid ${LAUNCHER_PID:-unknown} not running)"
    fi
  fi

  echo "launcher_state: ${launcher_status}"
  if [[ -n "${state_mode}" ]]; then
    echo "launcher_mode: ${state_mode}"
  fi
  if [[ -n "${state_result}" ]]; then
    echo "launcher_result: ${state_result}"
  fi
  if [[ -n "${state_log_dir}" ]]; then
    echo "launcher_logs: ${state_log_dir}"
  fi

  local detail=""
  if detail="$(probe_controller 2>&1)"; then
    echo "controller: up (${detail})"
  else
    echo "controller: down (${detail})"
  fi

  if detail="$(probe_api_health 2>&1)"; then
    echo "api: up"
  else
    echo "api: down (${detail})"
  fi

  if detail="$(probe_web 2>&1)"; then
    echo "web: up"
  else
    echo "web: down (${detail})"
  fi

  if [[ -L "${LOG_ROOT}/latest" || -d "${LOG_ROOT}/latest" ]]; then
    echo "latest_logs: ${LOG_ROOT}/latest"
  fi
}

print_probe() {
  local launcher_status="absent"
  local state_log_dir=""
  if [[ -f "${ACTIVE_STATE}" ]]; then
    safe_source_state
    if pid_is_live "${LAUNCHER_PID:-}"; then
      launcher_status="running (pid=${LAUNCHER_PID})"
    else
      launcher_status="stale state (launcher pid ${LAUNCHER_PID:-unknown} not running)"
    fi
    state_log_dir="${LOG_DIR:-}"
  fi

  echo "launcher_state: ${launcher_status}"
  if [[ -n "${state_log_dir}" ]]; then
    echo "launcher_logs: ${state_log_dir}"
  fi
  if [[ -L "${LOG_ROOT}/latest" || -d "${LOG_ROOT}/latest" ]]; then
    echo "latest_logs: ${LOG_ROOT}/latest"
  fi
  probe_hardware_state
}

stop_managed_stack() {
  if [[ ! -f "${ACTIVE_STATE}" ]]; then
    warn "no active launcher state found at ${ACTIVE_STATE}"
    WEB_PID="$(discover_pid_by_pattern 'vite')"
    CONTROLLER_PID="$(discover_pid_by_pattern 'gradient_os.run_controller')"
    API_PID="$(discover_pid_by_pattern 'gradient_os.api.main|gradient-api')"
    perform_shutdown_sequence
    return 1
  fi

  safe_source_state

  if pid_is_live "${LAUNCHER_PID:-}"; then
    log "Stopping launcher pid=${LAUNCHER_PID}"
    kill "${LAUNCHER_PID}" 2>/dev/null || true

    local waited=0
    while pid_is_live "${LAUNCHER_PID}" && [[ "${waited}" -lt 100 ]]; do
      sleep 0.1
      waited=$((waited + 1))
    done

    if pid_is_live "${LAUNCHER_PID}"; then
      warn "launcher did not exit after SIGTERM; sending SIGKILL"
      kill -KILL "${LAUNCHER_PID}" 2>/dev/null || true
    fi
    rm -f "${ACTIVE_STATE}"
    return 0
  fi

  warn "launcher pid is not running; stopping any recorded child processes directly"
  if [[ -z "${WEB_PID:-}" ]]; then
    WEB_PID="$(discover_pid_by_pattern 'vite')"
  fi
  if [[ -z "${CONTROLLER_PID:-}" ]]; then
    CONTROLLER_PID="$(discover_pid_by_pattern 'gradient_os.run_controller')"
  fi
  if [[ -z "${API_PID:-}" ]]; then
    API_PID="$(discover_pid_by_pattern 'gradient_os.api.main|gradient-api')"
  fi
  perform_shutdown_sequence
  rm -f "${ACTIVE_STATE}"
}

print_failed_log_excerpt() {
  local name="$1"
  local file="$2"
  if [[ -f "${file}" ]]; then
    log "Last 40 lines from ${name} log (${file}):"
    tail -n 40 "${file}" || true
  fi
}

run_interactive_console() {
  "${SYSTEM_PYTHON}" - "${REPO_ROOT}" "${CONTROLLER_LOG}" "${API_LOG}" "${WEB_LOG}" "${HEADLESS}" "${CONTROLLER_PID:-}" "${API_PID:-}" "${WEB_PID:-}" "${TTY_DEVICE}" <<'PY'
import _thread
import os
import readline  # type: ignore
import subprocess
import sys
import threading
import time
from pathlib import Path

repo_root = Path(sys.argv[1])
controller_log = Path(sys.argv[2])
api_log = Path(sys.argv[3])
web_log = Path(sys.argv[4])
headless = sys.argv[5] == "1"
controller_pid = int(sys.argv[6]) if sys.argv[6].strip() else 0
api_pid = int(sys.argv[7]) if sys.argv[7].strip() else 0
web_pid = int(sys.argv[8]) if sys.argv[8].strip() else 0
tty_device = sys.argv[9]
script_path = repo_root / "start-stack.sh"
prompt_prefix = "command> "
help_text = "Commands: stop | stop --hard | probe | status | help | clear"

if tty_device and tty_device != "not a tty":
    tty_fd = os.open(tty_device, os.O_RDWR)
    try:
        os.dup2(tty_fd, 0)
        os.dup2(tty_fd, 1)
        os.dup2(tty_fd, 2)
    finally:
        if tty_fd > 2:
            os.close(tty_fd)


class TailReader:
    def __init__(self, label: str, path: Path):
        self.label = label
        self.path = path
        self.handle = None
        try:
            self.handle = path.open("r", encoding="utf-8", errors="replace")
            self.handle.seek(0, os.SEEK_END)
        except Exception:
            self.handle = None

    def poll(self) -> list[str]:
        if self.handle is None:
            return []
        out: list[str] = []
        while True:
            line = self.handle.readline()
            if not line:
                break
            out.append(f"[{self.label}] {line.rstrip()}")
        return out


def pid_alive(pid: int) -> bool:
    if pid <= 0:
        return False
    try:
        os.kill(pid, 0)
    except OSError:
        return False
    return True


def run_stack_command(args: list[str]) -> list[str]:
    env = os.environ.copy()
    env["GRADIENT_STACK_INTERACTIVE_CONSOLE"] = "0"
    proc = subprocess.run(
        [str(script_path), *args],
        cwd=str(repo_root),
        env=env,
        capture_output=True,
        text=True,
    )
    lines: list[str] = []
    if proc.stdout:
        lines.extend(proc.stdout.rstrip("\n").splitlines())
    if proc.stderr:
        lines.extend(proc.stderr.rstrip("\n").splitlines())
    if not lines:
        lines.append(f"(no output, exit={proc.returncode})")
    elif proc.returncode != 0:
        lines.append(f"(command exit={proc.returncode})")
    return lines


print_lock = threading.Lock()
input_active = threading.Event()
stop_evt = threading.Event()
child_failure_evt = threading.Event()
failed_children: list[str] = []


def safe_print_line(line: str) -> None:
    with print_lock:
        if input_active.is_set():
            buf = readline.get_line_buffer()
            sys.stdout.write("\r\033[K")
            sys.stdout.write(line + "\n")
            sys.stdout.write(prompt_prefix + buf)
            sys.stdout.flush()
        else:
            print(line, flush=True)


def safe_print_block(lines: list[str]) -> None:
    with print_lock:
        if input_active.is_set():
            buf = readline.get_line_buffer()
            sys.stdout.write("\r\033[K")
            for line in lines:
                sys.stdout.write(line + "\n")
            sys.stdout.write(prompt_prefix + buf)
            sys.stdout.flush()
        else:
            for line in lines:
                print(line, flush=True)


tails = [
    TailReader("controller", controller_log),
    TailReader("api", api_log),
]
if not headless:
    tails.append(TailReader("web", web_log))


def monitor_loop() -> None:
    while not stop_evt.is_set():
        for tail in tails:
            for line in tail.poll():
                safe_print_line(line)

        failed: list[str] = []
        if controller_pid and not pid_alive(controller_pid):
            failed.append("controller")
        if api_pid and not pid_alive(api_pid):
            failed.append("api")
        if not headless and web_pid and not pid_alive(web_pid):
            failed.append("web")
        if failed:
            failed_children[:] = failed
            child_failure_evt.set()
            safe_print_line(f"[start-stack] supervised process exited: {' '.join(failed)}")
            _thread.interrupt_main()
            return

        time.sleep(0.1)


safe_print_block(
    [
        "[start-stack] Interactive line console active.",
        "[start-stack] Commands: stop | stop --hard | probe | status | help | clear",
    ]
)

monitor_thread = threading.Thread(target=monitor_loop, daemon=True)
monitor_thread.start()

exit_code = 0
try:
    while True:
        input_active.set()
        try:
            line = input(prompt_prefix)
        except KeyboardInterrupt:
            if child_failure_evt.is_set():
                exit_code = 20
            else:
                exit_code = 10
            break
        finally:
            input_active.clear()

        command = line.strip()
        if not command:
            continue

        safe_print_line(f"[console] command> {command}")

        if command in {"stop", "quit", "exit"}:
            exit_code = 10
            break
        if command in {"stop --hard", "hard-stop", "hard stop", "hardstop"}:
            exit_code = 11
            break
        if command == "help":
            safe_print_line("[console] Commands: stop, stop --hard, probe, status, help, clear")
            continue
        if command == "clear":
            with print_lock:
                sys.stdout.write("\033[2J\033[H")
                sys.stdout.flush()
            continue
        if command == "probe":
            safe_print_block([f"[probe] {line}" for line in run_stack_command(["probe"])])
            continue
        if command == "status":
            safe_print_block([f"[status] {line}" for line in run_stack_command(["status"])])
            continue
        safe_print_line(f"[console] Unknown command: {command}")
        safe_print_line("[console] Type 'help' to list available commands.")
finally:
    stop_evt.set()
    monitor_thread.join(timeout=0.5)

raise SystemExit(exit_code)
PY
}

supervise_children_noninteractive() {
  log "Startup complete. Streaming logs from ${LOG_DIR}"
  log "Press Ctrl-C to stop the supervised stack."

  while true; do
    local wait_status=0
    wait -n "${CHILD_PIDS[@]}" || wait_status=$?

    if [[ "${STOP_REQUESTED}" -eq 1 ]]; then
      return 0
    fi

    local failed=()
    if [[ -n "${CONTROLLER_PID}" ]] && ! pid_is_live "${CONTROLLER_PID}"; then
      failed+=("controller")
    fi
    if [[ -n "${API_PID}" ]] && ! pid_is_live "${API_PID}"; then
      failed+=("api")
    fi
    if [[ "${HEADLESS}" -eq 0 && -n "${WEB_PID}" ]] && ! pid_is_live "${WEB_PID}"; then
      failed+=("web")
    fi

    if (( ${#failed[@]} == 0 )); then
      continue
    fi

    START_RESULT="failed"
    START_FAILURE="supervised process exited: ${failed[*]}"
    SHUTDOWN_REASON="child_exit"
    write_state
    write_manifest
    error "${START_FAILURE}"

    if [[ " ${failed[*]} " == *" controller "* ]]; then
      print_failed_log_excerpt "controller" "${CONTROLLER_LOG}"
    fi
    if [[ " ${failed[*]} " == *" api "* ]]; then
      print_failed_log_excerpt "api" "${API_LOG}"
    fi
    if [[ " ${failed[*]} " == *" web "* ]]; then
      print_failed_log_excerpt "web" "${WEB_LOG}"
    fi

    return 1
  done
}

supervise_children() {
  START_RESULT="running"
  write_state
  write_manifest

  if ! interactive_console_enabled; then
    supervise_children_noninteractive
    return $?
  fi

  log "Startup complete. Interactive console attached to this terminal."
  log "Type stop, stop --hard, probe, status, help, or press Ctrl-C."
  stop_tailers

  local console_rc=0
  run_interactive_console || console_rc=$?

  case "${console_rc}" in
    10)
      STOP_REQUESTED=1
      SHUTDOWN_REASON="console_stop"
      START_RESULT="stopped"
      write_state
      write_manifest
      log "Interactive console requested soft stop."
      return 0
      ;;
    11)
      STOP_REQUESTED=1
      HARD_STOP=1
      SHUTDOWN_REASON="console_hard_stop"
      START_RESULT="stopped"
      write_state
      write_manifest
      log "Interactive console requested hard stop."
      return 0
      ;;
    20)
      local failed=()
      if [[ -n "${CONTROLLER_PID}" ]] && ! pid_is_live "${CONTROLLER_PID}"; then
        failed+=("controller")
      fi
      if [[ -n "${API_PID}" ]] && ! pid_is_live "${API_PID}"; then
        failed+=("api")
      fi
      if [[ "${HEADLESS}" -eq 0 && -n "${WEB_PID}" ]] && ! pid_is_live "${WEB_PID}"; then
        failed+=("web")
      fi
      START_RESULT="failed"
      START_FAILURE="supervised process exited: ${failed[*]}"
      SHUTDOWN_REASON="child_exit"
      write_state
      write_manifest
      error "${START_FAILURE}"

      if [[ " ${failed[*]} " == *" controller "* ]]; then
        print_failed_log_excerpt "controller" "${CONTROLLER_LOG}"
      fi
      if [[ " ${failed[*]} " == *" api "* ]]; then
        print_failed_log_excerpt "api" "${API_LOG}"
      fi
      if [[ " ${failed[*]} " == *" web "* ]]; then
        print_failed_log_excerpt "web" "${WEB_LOG}"
      fi
      return 1
      ;;
    *)
      error "interactive console exited unexpectedly (code=${console_rc})"
      return 1
      ;;
  esac
}

parse_args() {
  while [[ $# -gt 0 ]]; do
    case "$1" in
      --headless)
        HEADLESS=1
        shift
        ;;
      --hard)
        HARD_STOP=1
        shift
        ;;
      status|probe|stop|start)
        ACTION="$1"
        shift
        ;;
      --help|-h)
        usage
        exit 0
        ;;
      *)
        error "unknown argument: $1"
        usage
        exit 1
        ;;
    esac
  done
}

start_stack() {
  if [[ -f "${ACTIVE_STATE}" ]]; then
    local existing_launcher_pid=""
    existing_launcher_pid="$(
      STATE_PATH="${ACTIVE_STATE}" bash -c '
        # shellcheck disable=SC1090
        source "${STATE_PATH}" >/dev/null 2>&1 || exit 0
        printf "%s" "${LAUNCHER_PID:-}"
      '
    )"
    if [[ -n "${existing_launcher_pid}" ]] && pid_is_live "${existing_launcher_pid}"; then
      error "another start-stack launcher is already running with pid=${existing_launcher_pid}"
      error "Use ./start-stack.sh status or ./start-stack.sh stop first."
      return 1
    fi
  fi

  setup_logging
  ensure_required_files
  detect_external_services
  bootstrap_environment
  ensure_rtcore_runtime_sync
  startup_fault_reset_preflight

  MANAGE_CHILDREN=1
  START_RESULT="starting"
  write_state
  write_manifest

  start_process "controller" "${CONTROLLER_LOG}" env GRADIENT_RTCORE_AUTO_ARM=0 "${CONTROLLER_SCRIPT}"
  CONTROLLER_PID="${STARTED_PID}"
  write_state
  write_manifest
  wait_for_controller_readiness
  wait_for_bus_operational

  start_process "api" "${API_LOG}" "${API_SCRIPT}"
  API_PID="${STARTED_PID}"
  write_state
  write_manifest
  wait_for_api_readiness

  if [[ "${HEADLESS}" -eq 0 ]]; then
    start_process "web" "${WEB_LOG}" "${WEB_SCRIPT}"
    WEB_PID="${STARTED_PID}"
    write_state
    write_manifest
    wait_for_web_readiness
  fi

  supervise_children
}

parse_args "$@"

if [[ "${HARD_STOP}" -eq 1 && "${ACTION}" != "stop" ]]; then
  error "--hard is only supported with the stop action"
  usage
  exit 1
fi

case "${ACTION}" in
  status)
    print_status
    ;;
  probe)
    print_probe
    ;;
  stop)
    stop_managed_stack
    ;;
  start)
    start_stack
    ;;
  *)
    error "unsupported action: ${ACTION}"
    exit 1
    ;;
esac
