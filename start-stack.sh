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
BUS_READY_PROGRESS_GRACE_S="${GRADIENT_STACK_BUS_PROGRESS_GRACE_S:-15}"
BUS_READY_MAX_TIMEOUT_S="${GRADIENT_STACK_BUS_MAX_TIMEOUT_S:-60}"
STARTUP_FAULT_RESET_TIMEOUT_S="${GRADIENT_STACK_STARTUP_FAULT_RESET_TIMEOUT_S:-3}"
INTERACTIVE_CONSOLE_MODE="${GRADIENT_STACK_INTERACTIVE_CONSOLE:-auto}"
GRADIENT_STACK_COLOR_MODE="${GRADIENT_STACK_COLOR:-auto}"

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
UI_STATUS_ACTIVE=0
UI_SPINNER_INDEX=0
UI_SPINNER_PID=""
BANNER_RESET=""
BANNER_BORDER=""
BANNER_TITLE_PRIMARY=""
BANNER_TITLE_SECONDARY=""
BANNER_LABEL=""
BANNER_VALUE=""
BANNER_MUTED=""
UI_DANGER=""
UI_WARN=""
UI_OK=""
UI_INFO=""
UI_CMD=""
UI_PANEL=""
BOOT_SUMMARY_ROBOT=""
BOOT_SUMMARY_IK=""
BOOT_SUMMARY_SERVO=""
BOOT_SUMMARY_DRIVE=""
BOOT_SUMMARY_TOOL=""
BOOT_SUMMARY_RT_MAX_RPM=""

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
  GRADIENT_STACK_BUS_PROGRESS_GRACE_S  Extra seconds to keep waiting after positive bus progress (default: 15)
  GRADIENT_STACK_BUS_MAX_TIMEOUT_S     Hard cap for bus readiness wait, including progress grace (default: 60)
  GRADIENT_STACK_STARTUP_FAULT_RESET_TIMEOUT_S  Wait time after startup auto-reset before aborting (default: 3)
  GRADIENT_STACK_COLOR                Banner color mode: auto|0|1 (default: auto)
EOF
}

timestamp() {
  date '+%Y-%m-%d %H:%M:%S%z'
}

now_ms() {
  if [[ -n "${EPOCHREALTIME:-}" ]]; then
    local sec="${EPOCHREALTIME%.*}"
    local frac="${EPOCHREALTIME#*.}"
    frac="${frac:0:3}"
    while [[ "${#frac}" -lt 3 ]]; do
      frac="${frac}0"
    done
    printf '%s\n' "$((10#${sec} * 1000 + 10#${frac}))"
    return 0
  fi
  "${SYSTEM_PYTHON}" - <<'PY'
import time
print(int(time.time() * 1000))
PY
}

format_duration_ms() {
  local total_ms="${1:-0}"
  if (( total_ms < 1000 )); then
    printf '%dms' "${total_ms}"
    return 0
  fi

  local seconds=$(( total_ms / 1000 ))
  local millis=$(( total_ms % 1000 ))
  if (( seconds < 60 )); then
    printf '%d.%03ds' "${seconds}" "${millis}"
    return 0
  fi

  local minutes=$(( seconds / 60 ))
  local rem_seconds=$(( seconds % 60 ))
  printf '%dm %02d.%03ds' "${minutes}" "${rem_seconds}" "${millis}"
}

init_banner_palette() {
  BANNER_RESET=""
  BANNER_BORDER=""
  BANNER_TITLE_PRIMARY=""
  BANNER_TITLE_SECONDARY=""
  BANNER_LABEL=""
  BANNER_VALUE=""
  BANNER_MUTED=""
  UI_DANGER=""
  UI_WARN=""
  UI_OK=""
  UI_INFO=""
  UI_CMD=""
  UI_PANEL=""

  local mode="${GRADIENT_STACK_COLOR_MODE,,}"
  local enable=0
  case "${mode}" in
    1|true|yes|on|always)
      enable=1
      ;;
    0|false|no|off|never)
      enable=0
      ;;
    *)
      if [[ -z "${NO_COLOR:-}" && -n "${TTY_DEVICE}" && "${TTY_DEVICE}" != "not a tty" && "${TERM:-}" != "dumb" ]]; then
        enable=1
      fi
      ;;
  esac

  if [[ "${enable}" -eq 1 ]]; then
    BANNER_RESET=$'\033[0m'
    BANNER_BORDER=$'\033[38;5;45m'
    BANNER_TITLE_PRIMARY=$'\033[1;38;5;226m'
    BANNER_TITLE_SECONDARY=$'\033[1;38;5;51m'
    BANNER_LABEL=$'\033[1;38;5;111m'
    BANNER_VALUE=$'\033[38;5;255m'
    BANNER_MUTED=$'\033[38;5;244m'
    UI_DANGER=$'\033[1;38;5;196m'
    UI_WARN=$'\033[1;38;5;226m'
    UI_OK=$'\033[1;38;5;82m'
    UI_INFO=$'\033[1;38;5;45m'
    UI_CMD=$'\033[1;38;5;118m'
    UI_PANEL=$'\033[38;5;240m'
  fi
}

ui_can_render() {
  [[ -n "${TTY_DEVICE}" && "${TTY_DEVICE}" != "not a tty" && -w "${TTY_DEVICE}" && "${TERM:-}" != "dumb" ]]
}

ui_next_spinner_frame() {
  local frames=("[#---]" "[-#--]" "[--#-]" "[---#]" "[--#-]" "[-#--]")
  local frame="${frames[$((UI_SPINNER_INDEX % ${#frames[@]}))]}"
  UI_SPINNER_INDEX=$((UI_SPINNER_INDEX + 1))
  printf '%s' "${frame}"
}

ui_status_clear() {
  if [[ -n "${UI_SPINNER_PID}" ]] && kill -0 "${UI_SPINNER_PID}" 2>/dev/null; then
    kill "${UI_SPINNER_PID}" 2>/dev/null || true
    wait "${UI_SPINNER_PID}" 2>/dev/null || true
  fi
  UI_SPINNER_PID=""
  if ui_can_render && [[ "${UI_STATUS_ACTIVE}" -eq 1 ]]; then
    printf '\r\033[2K' > "${TTY_DEVICE}"
    UI_STATUS_ACTIVE=0
  fi
}

ui_loading_begin() {
  local stage="$1"
  local detail="$2"
  local started_ms="${3:-}"
  local timeout_s="${4:-}"
  init_banner_palette
  ui_status_clear
  if ! ui_can_render; then
    return 0
  fi
  if [[ -z "${started_ms}" ]]; then
    started_ms="$(now_ms)"
  fi
  local tty_device="${TTY_DEVICE}"
  local warn_style="${UI_WARN}"
  local info_style="${UI_INFO}"
  local muted_style="${BANNER_MUTED}"
  local reset_style="${BANNER_RESET}"
  (
    trap 'exit 0' TERM INT
    local frames=("[#---]" "[-#--]" "[--#-]" "[---#]" "[--#-]" "[-#--]")
    local index=0
    while true; do
      local current_ms
      local elapsed_ms
      local elapsed_label
      local countdown=""
      current_ms="$(now_ms)"
      elapsed_ms=$(( current_ms - started_ms ))
      elapsed_label="$(format_duration_ms "${elapsed_ms}")"
      if [[ -n "${timeout_s}" ]]; then
        local timeout_ms=$(( timeout_s * 1000 ))
        local remaining_ms=$(( timeout_ms - elapsed_ms ))
        if (( remaining_ms < 0 )); then
          remaining_ms=0
        fi
        countdown="  ${timeout_s}s timeout :: left $(format_duration_ms "${remaining_ms}")"
      fi
      printf '\r\033[2K  %b%s%b %b%-18s%b %s  t+%s' \
        "${warn_style}" "${frames[$((index % ${#frames[@]}))]}" "${reset_style}" \
        "${info_style}" "${stage}" "${reset_style}" \
        "${detail}" "${elapsed_label}" > "${tty_device}"
      if [[ -n "${countdown}" ]]; then
        printf '  %b%s%b' "${muted_style}" "${countdown}" "${reset_style}" > "${tty_device}"
      fi
      index=$((index + 1))
      sleep 0.12
    done
  ) &
  UI_SPINNER_PID="$!"
  UI_STATUS_ACTIVE=1
}

ui_loading_status() {
  local stage="$1"
  local detail="$2"
  local started_ms="${3:-}"
  local timeout_s="${4:-}"
  init_banner_palette
  if ! ui_can_render; then
    return 0
  fi
  local frame=""
  local elapsed_suffix=""
  local timeout_suffix=""
  frame="$(ui_next_spinner_frame)"
  if [[ -n "${started_ms}" ]]; then
    local current_ms=""
    local elapsed_ms=0
    current_ms="$(now_ms)"
    elapsed_ms=$(( current_ms - started_ms ))
    elapsed_suffix="  t+$(format_duration_ms "${elapsed_ms}")"
    if [[ -n "${timeout_s}" ]]; then
      local timeout_ms=$(( timeout_s * 1000 ))
      local remaining_ms=$(( timeout_ms - elapsed_ms ))
      if (( remaining_ms < 0 )); then
        remaining_ms=0
      fi
      timeout_suffix="  ${timeout_s}s timeout :: left $(format_duration_ms "${remaining_ms}")"
    fi
  fi
  printf '\r\033[2K  %b%s%b %b%-18s%b %s' \
    "${UI_WARN}" "${frame}" "${BANNER_RESET}" \
    "${UI_INFO}" "${stage}" "${BANNER_RESET}" \
    "${detail}${elapsed_suffix}${timeout_suffix}" > "${TTY_DEVICE}"
  UI_STATUS_ACTIVE=1
}

run_with_loading_capture() {
  local __out_var="$1"
  local stage="$2"
  local detail="$3"
  local timeout_s="${4:-}"
  shift 3
  if [[ -n "${timeout_s}" ]]; then
    shift
  fi
  local output=""
  local rc=0
  local started_ms
  started_ms="$(now_ms)"
  ui_loading_begin "${stage}" "${detail}" "${started_ms}" "${timeout_s}"
  output="$("$@" 2>&1)" || rc=$?
  ui_status_clear
  printf -v "${__out_var}" '%s' "${output}"
  return "${rc}"
}

style_text() {
  init_banner_palette
  local style="$1"
  local text="$2"
  printf '%b%s%b' "${style}" "${text}" "${BANNER_RESET}"
}

print_log_line() {
  local level="$1"
  local accent="$2"
  local fd="$3"
  local message="$4"
  init_banner_palette
  ui_status_clear

  local ts=""
  local tag=""
  local level_prefix=""
  ts="$(style_text "${BANNER_MUTED}" "[$(timestamp)]")"
  tag="$(style_text "${UI_INFO}" "[start-stack]")"
  if [[ -n "${level}" ]]; then
    level_prefix=" $(style_text "${accent}" "${level}:")"
  fi

  if [[ "${fd}" == "2" ]]; then
    printf '%s %s%s %s\n' "${ts}" "${tag}" "${level_prefix}" "${message}" >&2
  else
    printf '%s %s%s %s\n' "${ts}" "${tag}" "${level_prefix}" "${message}"
  fi
}

log() {
  print_log_line "" "${UI_INFO}" "1" "$*"
}

info() {
  print_log_line "INFO" "${UI_INFO}" "1" "$*"
}

success() {
  print_log_line "SUCCESS" "${UI_OK}" "1" "$*"
}

warn() {
  print_log_line "WARNING" "${UI_WARN}" "2" "$*"
}

error() {
  print_log_line "ERROR" "${UI_DANGER}" "2" "$*"
}

style_danger() {
  style_text "${UI_DANGER}" "$1"
}

style_warn() {
  style_text "${UI_WARN}" "$1"
}

style_ok() {
  style_text "${UI_OK}" "$1"
}

style_info() {
  style_text "${UI_INFO}" "$1"
}

style_cmd() {
  style_text "${UI_CMD}" "$1"
}

style_probe_state() {
  local raw="${1:-unknown}"
  local state="${raw^^}"
  case "${state}" in
    OP|UP|READY|ONLINE|ACTIVE)
      style_ok "${raw}"
      ;;
    BUS_UP_DISARMED|DISARMED)
      style_info "${raw}"
      ;;
    UNKNOWN|INACTIVE|STARTING|PREOP|SAFEOP)
      style_warn "${raw}"
      ;;
    DOWN|FAULTED|FAILED|ERROR)
      style_danger "${raw}"
      ;;
    *)
      style_text "${BANNER_VALUE}" "${raw}"
      ;;
  esac
}

style_probe_ratio() {
  local actual="${1:-0}"
  local expected="${2:-0}"
  local rendered="${actual}/${expected}"
  if [[ "${expected}" =~ ^[0-9]+$ ]] && [[ "${actual}" =~ ^[0-9]+$ ]] && (( expected > 0 )); then
    if (( actual >= expected )); then
      style_ok "${rendered}"
    else
      style_warn "${rendered}"
    fi
    return 0
  fi
  style_text "${BANNER_VALUE}" "${rendered}"
}

style_probe_bool() {
  local raw="${1:-}"
  local normalized="${raw,,}"
  case "${normalized}" in
    1|true|yes)
      style_ok "yes"
      ;;
    0|false|no)
      style_danger "no"
      ;;
    *)
      style_warn "${raw:-unknown}"
      ;;
  esac
}

style_launcher_state() {
  local raw="${1:-unknown}"
  case "${raw}" in
    running*)
      style_ok "${raw}"
      ;;
    absent)
      style_warn "${raw}"
      ;;
    stale*)
      style_danger "${raw}"
      ;;
    *)
      style_text "${BANNER_VALUE}" "${raw}"
      ;;
  esac
}

style_ds402_state() {
  local raw="${1:-unknown}"
  local normalized="${raw^^}"
  normalized="${normalized//[^A-Z0-9]/}"
  case "${normalized}" in
    OPERATIONENABLED)
      style_ok "${raw}"
      ;;
    SWITCHONDISABLED|READYTOSWITCHON|SWITCHEDON)
      style_info "${raw}"
      ;;
    QUICKSTOPACTIVE|NOTREADYTOSWITCHON|UNKNOWN)
      style_warn "${raw}"
      ;;
    FAULT|FAULTREACTIONACTIVE)
      style_danger "${raw}"
      ;;
    *)
      style_text "${BANNER_VALUE}" "${raw}"
      ;;
  esac
}

style_probe_hex_code() {
  local raw="${1:-0x0000}"
  local normalized="${raw#0x}"
  normalized="${normalized#0X}"
  if [[ "${normalized}" =~ ^[0-9A-Fa-f]+$ ]]; then
    if (( 16#${normalized} == 0 )); then
      style_ok "${raw}"
    else
      style_danger "${raw}"
    fi
    return 0
  fi
  style_text "${BANNER_VALUE}" "${raw}"
}

probe_callout_accent() {
  init_banner_palette
  local raw="${1:-unknown}"
  local state="${raw^^}"
  case "${state}" in
    ACTIVE|OP|UP|READY|ONLINE)
      printf '%s' "${UI_OK}"
      ;;
    BUS_UP_DISARMED|DISARMED)
      printf '%s' "${UI_INFO}"
      ;;
    INACTIVE|STARTING|PREOP|SAFEOP|UNKNOWN)
      printf '%s' "${UI_WARN}"
      ;;
    DOWN|FAULTED|FAILED|ERROR)
      printf '%s' "${UI_DANGER}"
      ;;
    *)
      printf '%s' "${UI_INFO}"
      ;;
  esac
}

print_callout_block() {
  init_banner_palette
  ui_status_clear
  local accent="$1"
  local title="$2"
  shift 2

  printf '\n%b%s%b\n' "${UI_PANEL}" "  ----------------------------------------------------------------------" "${BANNER_RESET}"
  printf '  %b// %s //%b\n' "${accent}" "${title}" "${BANNER_RESET}"
  while [[ "$#" -gt 0 ]]; do
    printf '  %s\n' "$1"
    shift
  done
  printf '%b%s%b\n' "${UI_PANEL}" "  ----------------------------------------------------------------------" "${BANNER_RESET}"
}

probe_kv_line() {
  local label="$1"
  local value="$2"
  printf '  %-24s %s' "${label}" "${value}"
}

print_boot_success_block() {
  local mode_label="full stack"
  local web_status
  local control_hint
  if [[ "${HEADLESS}" -eq 1 ]]; then
    mode_label="headless"
    web_status="$(style_warn 'DISABLED (--headless)')"
  else
    web_status="$(style_ok "ONLINE at http://127.0.0.1:${WEB_PORT}")"
  fi
  control_hint="$(style_cmd 'probe') / $(style_cmd 'status') / $(style_cmd 'stop') / $(style_cmd 'stop --hard')"

  print_callout_block "${UI_OK}" "SYSTEM ONLINE" \
    "  status: $(style_ok 'GRADIENTOS BOOT COMPLETE')" \
    "  mode:   ${mode_label}" \
    "  robot:  ${BOOT_SUMMARY_ROBOT:-unknown}" \
    "  stack:  $(style_ok "${BOOT_SUMMARY_IK:-unknown}") / $(style_ok "${BOOT_SUMMARY_SERVO:-unknown}") / $(style_ok "${BOOT_SUMMARY_DRIVE:-unknown}")" \
    "  bus:    $(style_ok 'READY') and $(style_ok 'DISARMED')" \
    "  truth:  $(style_warn 'MONITORING')" \
    "  api:    $(style_ok "ONLINE at http://127.0.0.1:${API_PORT}")" \
    "  web:    ${web_status}" \
    "  logs:   ${LOG_DIR}" \
    "  ops:    ${control_hint}"
}

banner_stat_line() {
  local left_label="$1"
  local left_value="$2"
  local right_label="$3"
  local right_value="$4"
  printf '  %b%-11s%b %b%-20s%b %b%-11s%b %b%s%b\n' \
    "${BANNER_LABEL}" "${left_label}" "${BANNER_RESET}" \
    "${BANNER_VALUE}" "${left_value}" "${BANNER_RESET}" \
    "${BANNER_LABEL}" "${right_label}" "${BANNER_RESET}" \
    "${BANNER_VALUE}" "${right_value}" "${BANNER_RESET}"
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
try:
    with io.StringIO() as _sink:
        from contextlib import redirect_stdout
        with redirect_stdout(_sink):
            from gradient_os.telemetry.terminal_dashboard import (
                TerminalDashboardState,
                process_service_log_line,
            )
except Exception:
    TerminalDashboardState = None
    process_service_log_line = None

state = TerminalDashboardState() if TerminalDashboardState is not None else None

with open(path, "r", encoding="utf-8", errors="replace") as handle:
    while True:
        line = handle.readline()
        if line:
            if state is not None and callable(process_service_log_line):
                emitted = process_service_log_line(label, line, state)
                for output_line in emitted:
                    sys.stdout.write(output_line + "\n")
                    sys.stdout.flush()
            else:
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

probe_hardware_axis_rows() {
  local payload="$1"
  "${SYSTEM_PYTHON}" - "${payload}" <<'PY'
import json
import sys

data = json.loads(sys.argv[1])
for axis in data.get("axes", []):
    label = (
        f"J{axis.get('logical_joint')}/axis{axis.get('axis', axis.get('index'))}"
        if axis.get("logical_joint") is not None
        else f"axis{axis.get('axis', axis.get('index'))}"
    )
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
            fault_suffix = "[" + " | ".join(details) + "]"
    print(
        "\x1f".join(
            [
                label,
                str(axis.get("ds402_state") or "unknown"),
                f"0x{int(axis.get('statusword', 0) or 0):04x}",
                f"0x{int(axis.get('error_code', 0) or 0):04x}",
                fault_suffix,
                str(axis.get("slave_online")),
                str(axis.get("slave_operational")),
                str(axis.get("slave_al_state_name") or "UNKNOWN"),
                str(axis.get("pos_counts") if axis.get("pos_counts") is not None else "unknown"),
            ]
        )
    )
PY
}

probe_hardware_state() {
  local launcher_status="${1:-absent}"
  local state_log_dir="${2:-}"
  local payload
  if ! payload="$(probe_hardware_state_json 2>/dev/null)"; then
    error "hardware probe failed"
    return 1
  fi

  local latest_log_dir=""
  if [[ -L "${LOG_ROOT}/latest" || -d "${LOG_ROOT}/latest" ]]; then
    latest_log_dir="${LOG_ROOT}/latest"
  fi

  local controller_udp_up=""
  local controller_detail=""
  local api_http_up=""
  local api_error=""
  local runtime_config_available=""
  local runtime_config_error=""
  local runtime_ik=""
  local runtime_ik_source=""
  local runtime_servo=""
  local runtime_drive_profile=""
  local runtime_drive_profile_source=""
  local runtime_drive_profile_configured=""
  local runtime_drive_profile_live=""
  local runtime_restart_required=""
  local probe_context_source=""
  local probe_servo_backend=""
  local probe_drive_profile=""
  local probe_context_error=""
  local drive_fault_available=""
  local drive_fault_label=""
  local drive_fault_profile=""
  local reference_available=""
  local reference_label=""
  local reference_profile=""
  local driver_state=""
  local armed=""
  local axis_enable_mask=""
  local op_enabled_axes=""
  local num_axes=""
  local ethercat_master_state=""
  local link_up=""
  local responding=""
  local online=""
  local operational=""
  local wkc_actual=""
  local wkc_expected=""
  local startup_ready=""
  local master_al=""
  local rtcore_state=""
  local rtcore_socket_present=""
  local physical_state=""
  local metrics_path=""
  local metrics_startup_readback_enabled=""
  local metrics_native_home_refresh_enabled=""
  local metrics_absolute_feedback_poll_enabled=""
  local controller_state="down"
  local api_state="down"

  controller_udp_up="$(probe_json_field "${payload}" "controller_udp_up")"
  controller_detail="$(probe_json_field "${payload}" "controller_detail")"
  api_http_up="$(probe_json_field "${payload}" "api_http_up")"
  api_error="$(probe_json_field "${payload}" "api_error")"
  runtime_config_available="$(probe_json_field "${payload}" "runtime_config_available")"
  runtime_config_error="$(probe_json_field "${payload}" "runtime_config_error")"
  runtime_ik="$(probe_json_field "${payload}" "runtime_ik")"
  runtime_ik_source="$(probe_json_field "${payload}" "runtime_ik_source")"
  runtime_servo="$(probe_json_field "${payload}" "runtime_servo")"
  runtime_drive_profile="$(probe_json_field "${payload}" "runtime_drive_profile")"
  runtime_drive_profile_source="$(probe_json_field "${payload}" "runtime_drive_profile_source")"
  runtime_drive_profile_configured="$(probe_json_field "${payload}" "runtime_drive_profile_configured")"
  runtime_drive_profile_live="$(probe_json_field "${payload}" "runtime_drive_profile_live")"
  runtime_restart_required="$(probe_json_field "${payload}" "runtime_restart_required")"
  probe_context_source="$(probe_json_field "${payload}" "probe_context_source")"
  probe_servo_backend="$(probe_json_field "${payload}" "probe_servo_backend")"
  probe_drive_profile="$(probe_json_field "${payload}" "probe_drive_profile")"
  probe_context_error="$(probe_json_field "${payload}" "probe_context_error")"
  drive_fault_available="$(probe_json_field "${payload}" "drive_fault_reference.available")"
  drive_fault_label="$(probe_json_field "${payload}" "drive_fault_reference.label")"
  drive_fault_profile="$(probe_json_field "${payload}" "drive_fault_reference.profile_id")"
  reference_available="$(probe_json_field "${payload}" "reference.available")"
  reference_label="$(probe_json_field "${payload}" "reference.label")"
  reference_profile="$(probe_json_field "${payload}" "reference.profile_id")"
  driver_state="$(probe_json_field "${payload}" "driver_state")"
  armed="$(probe_json_field "${payload}" "armed")"
  axis_enable_mask="$(probe_json_field "${payload}" "axis_enable_mask")"
  op_enabled_axes="$(probe_json_field "${payload}" "op_enabled_axes")"
  num_axes="$(probe_json_field "${payload}" "num_axes")"
  ethercat_master_state="$(probe_json_field "${payload}" "ethercat_master_state")"
  link_up="$(probe_json_field "${payload}" "link_up")"
  responding="$(probe_json_field "${payload}" "responding")"
  online="$(probe_json_field "${payload}" "online")"
  operational="$(probe_json_field "${payload}" "operational")"
  wkc_actual="$(probe_json_field "${payload}" "wkc_actual")"
  wkc_expected="$(probe_json_field "${payload}" "wkc_expected")"
  startup_ready="$(probe_json_field "${payload}" "startup_ready")"
  master_al="$(probe_json_field "${payload}" "master_al")"
  rtcore_state="$(probe_json_field "${payload}" "rtcore_state")"
  rtcore_socket_present="$(probe_json_field "${payload}" "rtcore_socket_present")"
  physical_state="$(probe_json_field "${payload}" "physical_state")"
  metrics_path="$(probe_json_field "${payload}" "metrics_path")"
  metrics_startup_readback_enabled="$(probe_json_field "${payload}" "metrics_startup_readback_enabled")"
  metrics_native_home_refresh_enabled="$(probe_json_field "${payload}" "metrics_native_home_refresh_enabled")"
  metrics_absolute_feedback_poll_enabled="$(probe_json_field "${payload}" "metrics_absolute_feedback_poll_enabled")"

  if [[ "${controller_udp_up}" == "1" ]]; then
    controller_state="up"
  fi
  if [[ "${api_http_up}" == "1" ]]; then
    api_state="up"
  fi

  local controller_value=""
  local api_value=""
  local rtcore_value=""
  local runtime_primary=""
  local runtime_secondary=""
  local probe_decode_value=""
  local drive_fault_value=""
  local metrics_toggle_value=""
  local enable_mask_value=""
  local master_al_value=""
  local resolved_fault_reference=""

  controller_value="$(style_probe_state "${controller_state}")"
  if [[ -n "${controller_detail}" ]]; then
    controller_value="${controller_value} $(style_text "${BANNER_VALUE}" "(${controller_detail})")"
  fi
  api_value="$(style_probe_state "${api_state}")"
  rtcore_value="$(style_probe_state "${rtcore_state:-unknown}") (socket_present=$(style_probe_bool "${rtcore_socket_present}"))"
  enable_mask_value="$(style_cmd "0x$(printf '%x' "${axis_enable_mask:-0}")")"
  master_al_value="$(style_cmd "0x$(printf '%x' "${master_al:-0}")")"

  if [[ "${runtime_config_available}" == "1" ]]; then
    runtime_primary="ik=$(style_text "${BANNER_VALUE}" "${runtime_ik:-unknown}") source=$(style_text "${BANNER_VALUE}" "${runtime_ik_source:-unknown}") servo=$(style_text "${BANNER_VALUE}" "${runtime_servo:-unknown}")"
    runtime_secondary="drive_profile=$(style_text "${BANNER_VALUE}" "${runtime_drive_profile:-unknown}") drive_source=$(style_text "${BANNER_VALUE}" "${runtime_drive_profile_source:-unknown}") configured=$(style_text "${BANNER_VALUE}" "${runtime_drive_profile_configured:-unknown}") live=$(style_text "${BANNER_VALUE}" "${runtime_drive_profile_live:-unavailable}") restart_required=$(style_probe_bool "${runtime_restart_required}")"
  fi

  if [[ -n "${probe_context_source}" && "${probe_context_source}" != "api_active_runtime" ]]; then
    probe_decode_value="backend=$(style_text "${BANNER_VALUE}" "${probe_servo_backend:-unknown}") drive_profile=$(style_text "${BANNER_VALUE}" "${probe_drive_profile:-unknown}") source=$(style_text "${BANNER_VALUE}" "${probe_context_source}")"
  fi

  if [[ "${drive_fault_available}" == "1" || "${reference_available}" == "1" ]]; then
    resolved_fault_reference="${drive_fault_label:-${reference_label:-${drive_fault_profile:-${reference_profile:-configured}}}}"
    drive_fault_value="$(style_text "${BANNER_VALUE}" "${resolved_fault_reference}") (only valid for the current servo backend)"
  elif [[ -n "${probe_servo_backend}" || -n "${runtime_servo}" ]]; then
    drive_fault_value="$(style_warn "raw_only") (no vendor-specific decode applied for backend=$(style_text "${BANNER_VALUE}" "${probe_servo_backend:-${runtime_servo}}"))"
  else
    drive_fault_value="$(style_warn "raw_only") (active servo backend unavailable)"
  fi

  if [[ -n "${metrics_startup_readback_enabled}" || -n "${metrics_native_home_refresh_enabled}" || -n "${metrics_absolute_feedback_poll_enabled}" ]]; then
    metrics_toggle_value="startup_readback=$(style_probe_bool "${metrics_startup_readback_enabled}") native_home_refresh=$(style_probe_bool "${metrics_native_home_refresh_enabled}") absolute_feedback_poll=$(style_probe_bool "${metrics_absolute_feedback_poll_enabled}")"
  fi

  local -a overview_lines=()
  overview_lines+=("$(probe_kv_line "launcher_state:" "$(style_launcher_state "${launcher_status}")")")
  if [[ -n "${state_log_dir}" ]]; then
    overview_lines+=("$(probe_kv_line "launcher_logs:" "$(style_text "${BANNER_VALUE}" "${state_log_dir}")")")
  fi
  if [[ -n "${latest_log_dir}" ]]; then
    overview_lines+=("$(probe_kv_line "latest_logs:" "$(style_text "${BANNER_VALUE}" "${latest_log_dir}")")")
  fi
  overview_lines+=("$(probe_kv_line "physical_state:" "$(style_probe_state "${physical_state:-unknown}")")")
  overview_lines+=("$(probe_kv_line "driver_state:" "$(style_probe_state "${driver_state:-unknown}")")")
  overview_lines+=("$(probe_kv_line "ethercat_master_state:" "$(style_probe_state "${ethercat_master_state:-unknown}")")")
  overview_lines+=("$(probe_kv_line "rtcore_state:" "${rtcore_value}")")
  print_callout_block "$(probe_callout_accent "${physical_state}")" "PROBE OVERVIEW" "${overview_lines[@]}"

  local -a runtime_lines=()
  runtime_lines+=("$(probe_kv_line "controller_udp:" "${controller_value}")")
  runtime_lines+=("$(probe_kv_line "api_http:" "${api_value}")")
  if [[ -n "${api_error}" ]]; then
    runtime_lines+=("$(probe_kv_line "api_error:" "$(style_warn "${api_error}")")")
  fi
  if [[ "${runtime_config_available}" == "1" ]]; then
    runtime_lines+=("$(probe_kv_line "runtime_config:" "${runtime_primary}")")
    runtime_lines+=("$(probe_kv_line "" "${runtime_secondary}")")
  elif [[ -n "${runtime_config_error}" ]]; then
    runtime_lines+=("$(probe_kv_line "runtime_config:" "$(style_warn "unavailable (${runtime_config_error})")")")
  else
    runtime_lines+=("$(probe_kv_line "runtime_config:" "$(style_warn "unavailable")")")
  fi
  if [[ -n "${probe_decode_value}" ]]; then
    runtime_lines+=("$(probe_kv_line "probe_decode:" "${probe_decode_value}")")
    if [[ -n "${probe_context_error}" ]]; then
      runtime_lines+=("$(probe_kv_line "probe_context_error:" "$(style_warn "${probe_context_error}")")")
    fi
  fi
  runtime_lines+=("$(probe_kv_line "drive_fault_reference:" "${drive_fault_value}")")
  print_callout_block "${UI_INFO}" "RUNTIME PROBES" "${runtime_lines[@]}"

  local -a hardware_lines=()
  hardware_lines+=("$(probe_kv_line "armed:" "$(style_text "${BANNER_VALUE}" "${armed:-0}")")")
  hardware_lines+=("$(probe_kv_line "enable_mask:" "${enable_mask_value}")")
  hardware_lines+=("$(probe_kv_line "op_enabled_axes:" "$(style_probe_ratio "${op_enabled_axes:-0}" "${num_axes:-0}")")")
  hardware_lines+=("$(probe_kv_line "link_up:" "$(style_probe_bool "${link_up}")")")
  hardware_lines+=("$(probe_kv_line "responding:" "$(style_probe_ratio "${responding:-0}" "${num_axes:-0}")")")
  hardware_lines+=("$(probe_kv_line "online:" "$(style_probe_ratio "${online:-0}" "${num_axes:-0}")")")
  hardware_lines+=("$(probe_kv_line "operational:" "$(style_probe_ratio "${operational:-0}" "${num_axes:-0}")")")
  hardware_lines+=("$(probe_kv_line "wkc:" "$(style_probe_ratio "${wkc_actual:-0}" "${wkc_expected:-0}")")")
  hardware_lines+=("$(probe_kv_line "startup_ready:" "$(style_probe_bool "${startup_ready}")")")
  hardware_lines+=("$(probe_kv_line "master_al:" "${master_al_value}")")
  hardware_lines+=("$(probe_kv_line "metrics_path:" "$(style_text "${BANNER_VALUE}" "${metrics_path:-unavailable}")")")
  if [[ -n "${metrics_toggle_value}" ]]; then
    hardware_lines+=("$(probe_kv_line "rtcore_metrics_sdo:" "${metrics_toggle_value}")")
  fi
  print_callout_block "$(probe_callout_accent "${ethercat_master_state}")" "BUS METRICS" "${hardware_lines[@]}"

  local -a axis_lines=()
  local axis_count=0
  local axis_label=""
  local ds402_state=""
  local statusword=""
  local error_code=""
  local fault_suffix=""
  local slave_online=""
  local slave_operational=""
  local slave_al=""
  local pos_counts=""
  local axis_line=""
  while IFS=$'\x1f' read -r axis_label ds402_state statusword error_code fault_suffix slave_online slave_operational slave_al pos_counts; do
    [[ -n "${axis_label}" ]] || continue
    axis_count=$((axis_count + 1))
    axis_line="  $(style_text "${BANNER_LABEL}" "${axis_label}:") $(style_ds402_state "${ds402_state}")   sw=$(style_cmd "${statusword}")   err=$(style_probe_hex_code "${error_code}")"
    if [[ -n "${fault_suffix}" ]]; then
      axis_line="${axis_line} $(style_warn "${fault_suffix}")"
    fi
    axis_line="${axis_line}   online=$(style_probe_bool "${slave_online}")   op=$(style_probe_bool "${slave_operational}")   al=$(style_probe_state "${slave_al}")   pos=$(style_text "${BANNER_VALUE}" "${pos_counts}")"
    axis_lines+=("${axis_line}")
  done < <(probe_hardware_axis_rows "${payload}")
  if (( axis_count == 0 )); then
    axis_lines+=("  $(style_warn 'no axes reported in RTCore metrics')")
  fi
  print_callout_block "${UI_INFO}" "AXIS STATES" "${axis_lines[@]}"
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
  local physical_state=""
  local driver_state=""
  local ethercat_master_state=""
  local rtcore_state=""
  local armed=""
  local enable_mask=""
  local op_enabled_axes=""
  local num_axes=""
  local wkc_actual=""
  local wkc_expected=""

  physical_state="$(probe_json_field "${payload}" "physical_state")"
  driver_state="$(probe_json_field "${payload}" "driver_state")"
  ethercat_master_state="$(probe_json_field "${payload}" "ethercat_master_state")"
  rtcore_state="$(probe_json_field "${payload}" "rtcore_state")"
  armed="$(probe_json_field "${payload}" "armed")"
  enable_mask="$(probe_json_field "${payload}" "axis_enable_mask")"
  op_enabled_axes="$(probe_json_field "${payload}" "op_enabled_axes")"
  num_axes="$(probe_json_field "${payload}" "num_axes")"
  wkc_actual="$(probe_json_field "${payload}" "wkc_actual")"
  wkc_expected="$(probe_json_field "${payload}" "wkc_expected")"

  print_callout_block "${UI_INFO}" "PROBE SNAPSHOT" \
    "  physical: $(style_probe_state "${physical_state}")   driver: $(style_probe_state "${driver_state}")" \
    "  ethercat: $(style_probe_state "${ethercat_master_state}")   rtcore: $(style_probe_state "${rtcore_state}")" \
    "  armed:    $(style_text "${BANNER_VALUE}" "${armed}")   enable_mask: $(style_cmd "0x$(printf '%x' "${enable_mask:-0}")")   op_enabled: $(style_probe_ratio "${op_enabled_axes:-0}" "${num_axes:-0}")" \
    "  wkc:      $(style_probe_ratio "${wkc_actual:-0}" "${wkc_expected:-0}")"
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

probe_is_bus_ready() {
  local payload="$1"
  "${SYSTEM_PYTHON}" - "${payload}" <<'PY'
import json
import sys

data = json.loads(sys.argv[1])
metrics_available = int(data.get("metrics_available", 0) or 0)
num_axes = int(data.get("num_axes", 0) or 0)
responding = int(data.get("responding", 0) or 0)
online = int(data.get("online", 0) or 0)
operational = int(data.get("operational", 0) or 0)
startup_ready = int(data.get("startup_ready", 0) or 0)
ready = (
    metrics_available == 1
    and num_axes > 0
    and responding >= num_axes
    and online >= num_axes
    and operational >= num_axes
    and startup_ready == 1
)
raise SystemExit(0 if ready else 1)
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

capture_rtcore_recent_journal() {
  run_journalctl -u "${RTCORE_SERVICE_NAME}" -n 120 --no-pager 2>/dev/null || true
}

capture_fieldbus_failure_diagnostics() {
  local payload="${1:-}"
  local started_ms="${2:-0}"
  if [[ -z "${LOG_DIR}" ]]; then
    return 0
  fi

  local diag_dir="${LOG_DIR}/fieldbus-failure-diagnostics"
  mkdir -p "${diag_dir}"

  if [[ -n "${payload}" ]]; then
    printf '%s\n' "${payload}" > "${diag_dir}/probe.json"
  fi

  local since_arg=""
  if [[ "${started_ms}" =~ ^[0-9]+$ ]] && (( started_ms > 0 )); then
    since_arg="@$(( started_ms / 1000 ))"
  fi

  run_systemctl status "${RTCORE_SERVICE_NAME}" "${ETHERCAT_SERVICE_NAME}" --no-pager \
    > "${diag_dir}/systemd-status.txt" 2>&1 || true
  if [[ -n "${since_arg}" ]]; then
    run_journalctl -u "${RTCORE_SERVICE_NAME}" -u "${ETHERCAT_SERVICE_NAME}" -S "${since_arg}" --no-pager \
      > "${diag_dir}/unit-journal.txt" 2>&1 || true
    run_journalctl -k -S "${since_arg}" --no-pager \
      > "${diag_dir}/kernel-journal.txt" 2>&1 || true
  else
    run_journalctl -u "${RTCORE_SERVICE_NAME}" -u "${ETHERCAT_SERVICE_NAME}" -n 200 --no-pager \
      > "${diag_dir}/unit-journal.txt" 2>&1 || true
    run_journalctl -k -n 200 --no-pager \
      > "${diag_dir}/kernel-journal.txt" 2>&1 || true
  fi
  ps -eo pid,ppid,state,wchan:32,cmd | awk '
    NR == 1 { print; next }
    index($0, "gradient-rt-motion") || index($0, "gradient-rt-mot") || index($0, "metrics") || index($0, "ethercat") { print }
  ' > "${diag_dir}/processes.txt" 2>&1 || true
  lsmod | awk '$1 ~ /^ec_/' > "${diag_dir}/kernel-modules.txt" 2>&1 || true

  "${SYSTEM_PYTHON}" - "${diag_dir}" "${RUN_ID:-unknown}" <<'PY' > "${diag_dir}/summary.txt"
from pathlib import Path
import json
import re
import sys

diag_dir = Path(sys.argv[1])
run_id = sys.argv[2]

def read_text(name: str) -> str:
    path = diag_dir / name
    if not path.exists():
        return ""
    return path.read_text(encoding="utf-8", errors="replace")

probe_summary = "probe_unavailable"
probe_path = diag_dir / "probe.json"
if probe_path.exists():
    try:
        probe = json.loads(probe_path.read_text(encoding="utf-8", errors="replace"))
        probe_summary = (
            f"physical_state={probe.get('physical_state')} "
            f"driver_state={probe.get('driver_state')} "
            f"ethercat_master_state={probe.get('ethercat_master_state')} "
            f"rtcore_state={probe.get('rtcore_state')} "
            f"responding={probe.get('responding')}/{probe.get('num_axes')} "
            f"online={probe.get('online')}/{probe.get('num_axes')} "
            f"operational={probe.get('operational')}/{probe.get('num_axes')} "
            f"startup_ready={probe.get('startup_ready')} "
            f"wkc={probe.get('wkc_actual')}/{probe.get('wkc_expected')}"
        )
    except Exception:
        probe_summary = "probe_parse_failed"

unit_journal = read_text("unit-journal.txt")
kernel_journal = read_text("kernel-journal.txt")
systemd_status = read_text("systemd-status.txt")
processes = read_text("processes.txt")
modules = read_text("kernel-modules.txt")

summary = [
    f"run_id={run_id}",
    f"probe={probe_summary}",
]

if (
    "Failed to reserve master" in unit_journal
    or "ecrt_request_master(0) failed" in unit_journal
    or "Failed to reserve master" in systemd_status
    or "ecrt_request_master(0) failed" in systemd_status
):
    summary.append("likely_cause=rtcore_master_reservation_failed")
if "Master already in use!" in kernel_journal:
    summary.append("kernel_master_already_in_use=1")
if "left-over process" in unit_journal or "left-over process" in systemd_status:
    summary.append("systemd_leftover_rtcore_process=1")
if "ec_generic is in use" in unit_journal or "ec_generic is in use" in systemd_status:
    summary.append("ethercat_kernel_module_busy=1")
if re.search(r"task metrics:\d+ blocked for more than", kernel_journal):
    match = re.search(r"task (metrics:\d+) blocked for more than", kernel_journal)
    summary.append(f"hung_kernel_task={match.group(1) if match else 'metrics'}")
if "[gradient-rt-mot] <defunct>" in processes:
    summary.append("rtcore_zombie_marker_present=1")
if "ec_generic" in modules or "ec_master" in modules:
    summary.append("ethercat_modules_loaded=1")

if len(summary) == 2:
    summary.append("likely_cause=unknown")

for line in summary:
    print(line)
PY

  local summary_path="${diag_dir}/summary.txt"
  warn "Captured fieldbus failure diagnostics in ${diag_dir}"
  if [[ -f "${summary_path}" ]]; then
    while IFS= read -r line; do
      [[ -n "${line}" ]] || continue
      warn "fieldbus diagnostic: ${line}"
    done < "${summary_path}"
  fi
}

probe_rtcore_startup_recovery_plan() {
  local payload="$1"
  local recent_log="$2"
  local recovery_attempted="${3:-0}"
  PYTHONPATH="${REPO_ROOT}/src:${PYTHONPATH:-}" "${PROBE_PYTHON}" - "${payload}" "${recent_log}" "${recovery_attempted}" <<'PY'
import json
import sys

from gradient_os.telemetry.startup_recovery import build_rtcore_startup_recovery_plan

payload = json.loads(sys.argv[1])
recent_log = sys.argv[2]
recovery_attempted = bool(int(sys.argv[3]))
print(
    json.dumps(
        build_rtcore_startup_recovery_plan(
            payload,
            recent_log=recent_log,
            recovery_attempted=recovery_attempted,
        )
    )
)
PY
}

wait_for_probe_state() {
  local desired_state="$1"
  local timeout_s="$2"
  local started_at
  local started_ms
  started_at="$(date +%s)"
  started_ms="$(now_ms)"

  while true; do
    local payload=""
    payload="$(capture_probe_json || true)"
    if [[ -n "${payload}" ]]; then
      local current_state=""
      current_state="$(probe_json_field "${payload}" "physical_state")"
      if [[ "${current_state}" == "${desired_state}" ]]; then
        ui_status_clear
        success "hardware reached ${desired_state} in $(format_duration_ms $(( $(now_ms) - started_ms )))"
        return 0
      fi
      ui_loading_status "hardware" "Waiting for physical_state=${desired_state} (current=${current_state})" "${started_ms}" "${timeout_s}"
    fi

    local now
    now="$(date +%s)"
    if (( now - started_at >= timeout_s )); then
      ui_status_clear
      return 1
    fi
    sleep "${PROBE_INTERVAL_S}"
  done
}

wait_for_probe_soft_stop_safe() {
  local timeout_s="$1"
  local started_at
  local started_ms
  started_at="$(date +%s)"
  started_ms="$(now_ms)"

  while true; do
    local payload=""
    payload="$(capture_probe_json || true)"
    if [[ -n "${payload}" ]] && probe_is_soft_stop_safe "${payload}"; then
      ui_status_clear
      success "hardware reached safe soft-stop state in $(format_duration_ms $(( $(now_ms) - started_ms ))): $(describe_probe_soft_stop_state "${payload}")"
      return 0
    fi
    if [[ -n "${payload}" ]]; then
      ui_loading_status "soft stop" "$(describe_probe_soft_stop_state "${payload}")" "${started_ms}" "${timeout_s}"
    fi

    local now
    now="$(date +%s)"
    if (( now - started_at >= timeout_s )); then
      ui_status_clear
      return 1
    fi
    sleep "${PROBE_INTERVAL_S}"
  done
}

wait_for_bus_operational() {
  local started_at
  local started_ms
  local deadline_at
  local hard_deadline_at
  local best_progress_score=-1
  local extension_logged=0
  started_at="$(date +%s)"
  started_ms="$(now_ms)"
  deadline_at=$(( started_at + BUS_READY_TIMEOUT_S ))
  hard_deadline_at=$(( started_at + BUS_READY_MAX_TIMEOUT_S ))
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
      local link_up=""

      metrics_available="$(probe_json_field "${payload}" "metrics_available")"
      num_axes="$(probe_json_field "${payload}" "num_axes")"
      responding="$(probe_json_field "${payload}" "responding")"
      online="$(probe_json_field "${payload}" "online")"
      operational="$(probe_json_field "${payload}" "operational")"
      startup_ready="$(probe_json_field "${payload}" "startup_ready")"
      wkc_actual="$(probe_json_field "${payload}" "wkc_actual")"
      link_up="$(probe_json_field "${payload}" "link_up")"

      if [[ "${metrics_available}" == "1" && -n "${num_axes}" && "${num_axes}" != "0" ]]; then
        if [[ "${responding}" == "${num_axes}" && "${online}" == "${num_axes}" && "${operational}" == "${num_axes}" && "${startup_ready}" == "1" ]]; then
          ui_status_clear
          success "BUS READY in $(format_duration_ms $(( $(now_ms) - started_ms ))): responding=${responding}/${num_axes} online=${online}/${num_axes} operational=${operational}/${num_axes} wkc=${wkc_actual}"
          return 0
        fi
        local detail="responding=${responding}/${num_axes} online=${online}/${num_axes} operational=${operational}/${num_axes} startup_ready=${startup_ready} wkc=${wkc_actual}"
        local now_for_progress
        local progress_score
        local remaining_s
        now_for_progress="$(date +%s)"
        progress_score=$(( ${responding:-0} + ${online:-0} + ${operational:-0} + (${startup_ready:-0} * ${num_axes:-0}) + ${link_up:-0} ))
        if (( progress_score > best_progress_score )); then
          best_progress_score="${progress_score}"
          if (( progress_score > 0 )); then
            local candidate_deadline=$(( now_for_progress + BUS_READY_PROGRESS_GRACE_S ))
            if (( candidate_deadline > deadline_at )); then
              deadline_at="${candidate_deadline}"
              if (( deadline_at > hard_deadline_at )); then
                deadline_at="${hard_deadline_at}"
              fi
              if (( extension_logged == 0 )) || [[ "${detail}" != "${last_detail}" ]]; then
                info "Fieldbus progress detected; extending readiness window to $(format_duration_ms $(( (deadline_at - started_at) * 1000 ))) total."
                extension_logged=1
              fi
            fi
          fi
        fi
        remaining_s=$(( deadline_at - now_for_progress ))
        if (( remaining_s < 1 )); then
          remaining_s=1
        fi
        ui_loading_status "fieldbus" "${detail}" "${started_ms}" "${remaining_s}"
        if [[ "${detail}" != "${last_detail}" ]]; then
          log "Waiting for full bus readiness: ${detail}"
          last_detail="${detail}"
        fi
      fi
    fi

    local now
    now="$(date +%s)"
    if (( now >= deadline_at )); then
      ui_status_clear
      if [[ -n "${payload}" ]]; then
        log_probe_snapshot "${payload}"
      fi
      capture_fieldbus_failure_diagnostics "${payload}" "${started_ms}"
      error "bus failed readiness after $(format_duration_ms $(( $(now_ms) - started_ms ))); base_timeout=${BUS_READY_TIMEOUT_S}s progress_grace=${BUS_READY_PROGRESS_GRACE_S}s hard_cap=${BUS_READY_MAX_TIMEOUT_S}s"
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

run_journalctl() {
  if [[ "$(id -u)" -eq 0 ]]; then
    journalctl "$@"
  else
    sudo -n journalctl "$@"
  fi
}

systemd_service_is_active() {
  run_systemctl is-active --quiet "$1" >/dev/null 2>&1
}

systemd_service_state() {
  local service_name="$1"
  local state=""
  state="$(run_systemctl is-active "${service_name}" 2>/dev/null || true)"
  state="${state//$'\n'/}"
  state="${state//$'\r'/}"
  printf '%s\n' "${state:-unknown}"
}

wait_for_systemd_service_inactive() {
  local service_name="$1"
  local timeout_s="${2:-12}"
  local deadline=0
  local now=0
  local state=""

  deadline=$(( $(date +%s) + timeout_s ))
  while true; do
    state="$(systemd_service_state "${service_name}")"
    case "${state}" in
      inactive|failed|unknown)
        return 0
        ;;
    esac
    now="$(date +%s)"
    if (( now >= deadline )); then
      warn "${service_name} still ${state} after ${timeout_s}s"
      return 1
    fi
    sleep 0.2
  done
}

stop_systemd_service_if_active() {
  local service_name="$1"
  local started_ms
  started_ms="$(now_ms)"
  if ! systemd_service_is_active "${service_name}"; then
    return 1
  fi
  log "Stopping systemd service ${service_name}"
  local stop_output=""
  if run_with_loading_capture stop_output "service stop" "Stopping ${service_name} via systemd" 12 run_systemctl stop "${service_name}"; then
    if wait_for_systemd_service_inactive "${service_name}" 12; then
      success "${service_name} stopped in $(format_duration_ms $(( $(now_ms) - started_ms )))"
      return 0
    fi
    warn "Escalating ${service_name} stop via systemctl kill"
    run_systemctl kill --signal=SIGKILL "${service_name}" >/dev/null 2>&1 || true
    run_systemctl stop "${service_name}" >/dev/null 2>&1 || true
    if wait_for_systemd_service_inactive "${service_name}" 5; then
      success "${service_name} force-stopped in $(format_duration_ms $(( $(now_ms) - started_ms )))"
      return 0
    fi
  fi
  warn "Failed to stop ${service_name} via systemd after $(format_duration_ms $(( $(now_ms) - started_ms )))"
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
    fi

    stop_ethercat_master_runtime

    local rtcore_state=""
    local ethercat_state=""
    rtcore_state="$(systemd_service_state "${RTCORE_SERVICE_NAME}")"
    ethercat_state="$(systemd_service_state "${ETHERCAT_SERVICE_NAME}")"
    if [[ "${rtcore_state}" != "inactive" && "${rtcore_state}" != "failed" && "${rtcore_state}" != "unknown" ]]; then
      warn "Hard stop ended with ${RTCORE_SERVICE_NAME} still ${rtcore_state}"
    fi
    if [[ "${ethercat_state}" != "inactive" && "${ethercat_state}" != "failed" && "${ethercat_state}" != "unknown" ]]; then
      warn "Hard stop ended with ${ETHERCAT_SERVICE_NAME} still ${ethercat_state}"
    fi
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
  local started_ms
  started_at="$(date +%s)"
  started_ms="$(now_ms)"
  local last_detail=""

  while true; do
    local detail=""
    if detail="$(${probe_func} 2>&1)"; then
      ui_status_clear
      success "${name} ready in $(format_duration_ms $(( $(now_ms) - started_ms ))): ${detail}"
      return 0
    fi

    ui_loading_status "${name}" "${detail:-awaiting response}" "${started_ms}" "${timeout_s}"
    if [[ "${detail}" != "${last_detail}" ]]; then
      log "Waiting for ${name}: ${detail}"
      last_detail="${detail}"
    fi

    local now
    now="$(date +%s)"
    if (( now - started_at >= timeout_s )); then
      ui_status_clear
      error "${name} failed readiness within ${timeout_s}s: ${detail}"
      return 1
    fi

    sleep "${PROBE_INTERVAL_S}"
  done
}

wait_for_controller_readiness() {
  local started_at
  local started_ms
  started_at="$(date +%s)"
  started_ms="$(now_ms)"
  local last_detail=""

  while true; do
    if ! pid_is_live "${CONTROLLER_PID}"; then
      ui_status_clear
      error "controller exited before readiness completed"
      return 1
    fi

    local detail=""
    if detail="$(probe_controller 2>&1)"; then
      ui_status_clear
      success "controller online in $(format_duration_ms $(( $(now_ms) - started_ms ))): ${detail}"
      return 0
    fi

    ui_loading_status "controller" "${detail:-starting controller process}" "${started_ms}" "${CONTROLLER_TIMEOUT_S}"
    if [[ "${detail}" != "${last_detail}" ]]; then
      log "Waiting for controller: ${detail}"
      last_detail="${detail}"
    fi

    local now
    now="$(date +%s)"
    if (( now - started_at >= CONTROLLER_TIMEOUT_S )); then
      ui_status_clear
      error "controller failed readiness within ${CONTROLLER_TIMEOUT_S}s: ${detail}"
      return 1
    fi

    sleep "${PROBE_INTERVAL_S}"
  done
}

wait_for_api_readiness() {
  local started_ms
  started_ms="$(now_ms)"
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
  success "api runtime-config sanity after $(format_duration_ms $(( $(now_ms) - started_ms ))): ${runtime_summary}"

  wait_for_probe "api joints" 10 probe_api_joints >/dev/null || return 1
  local pose_detail=""
  if pose_detail="$(probe_api_pose 2>&1)"; then
    success "api pose ready: ${pose_detail}"
  else
    warn "API pose probe unavailable during startup; continuing to web bring-up: ${pose_detail}"
  fi
  success "$(style_ok 'API ONLINE') in $(format_duration_ms $(( $(now_ms) - started_ms ))) at http://127.0.0.1:${API_PORT}"
  return 0
}

wait_for_web_readiness() {
  local started_ms
  started_ms="$(now_ms)"
  wait_for_probe "web UI" "${WEB_TIMEOUT_S}" probe_web >/dev/null || return 1
  success "$(style_ok 'WEB UI ONLINE') in $(format_duration_ms $(( $(now_ms) - started_ms ))) at http://127.0.0.1:${WEB_PORT}"
  return 0
}

start_process() {
  local name="$1"
  local log_file="$2"
  local started_ms
  shift 2
  started_ms="$(now_ms)"

  : > "${log_file}"
  start_tail "${name}" "${log_file}"

  info "Launching ${name} process"
  ui_loading_status "launch ${name}" "Starting process and binding stdout to ${log_file}"
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
  ui_status_clear
  success "${name} process started in $(format_duration_ms $(( $(now_ms) - started_ms ))): pid=${pid}, log=${log_file}"
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
  local started_ms
  started_ms="$(now_ms)"
  if [[ ! -f "${START_SH}" ]]; then
    error "missing ${START_SH}"
    return 1
  fi

  info "Environment bootstrap: loading ${START_SH}"
  ui_loading_status "environment" "Activating .venv, project paths, and system camera libraries" "${started_ms}"
  local previous_start_quiet="${GRADIENT_START_QUIET:-}"
  export GRADIENT_START_QUIET=1
  # shellcheck disable=SC1090
  if ! source "${START_SH}"; then
    ui_status_clear
    if [[ -n "${previous_start_quiet}" ]]; then
      export GRADIENT_START_QUIET="${previous_start_quiet}"
    else
      unset GRADIENT_START_QUIET
    fi
    error "environment bootstrap failed via ${START_SH}"
    return 1
  fi
  if [[ -n "${previous_start_quiet}" ]]; then
    export GRADIENT_START_QUIET="${previous_start_quiet}"
  else
    unset GRADIENT_START_QUIET
  fi

  export PYTHONUNBUFFERED=1
  export GRADIENT_RTCORE_READY_TIMEOUT_S="${GRADIENT_RTCORE_READY_TIMEOUT_S:-30}"
  ui_status_clear

  local cli_status=""
  local elapsed_ms
  elapsed_ms=$(( $(now_ms) - started_ms ))
  if command -v gradient-controller >/dev/null 2>&1; then
    cli_status="$(style_ok 'CLI COMMANDS READY')"
  else
    cli_status="$(style_warn 'ALIASES READY')"
  fi
  print_callout_block "${UI_INFO}" "ENVIRONMENT READY" \
    "  status: $(style_ok '.VENV ACTIVE') and $(style_ok 'PYTHONPATH INJECTED')" \
    "  python: ${PROBE_PYTHON}" \
    "  cli:    ${cli_status}" \
    "  timing: $(style_ok "$(format_duration_ms "${elapsed_ms}")")" \
    "  repo:   ${REPO_ROOT}"
  return 0
}

render_runtime_banner_summary() {
  PYTHONPATH="${REPO_ROOT}/src:${PYTHONPATH:-}" "${PROBE_PYTHON}" - "${REPO_ROOT}" <<'PY'
import io
import sys
from contextlib import redirect_stdout
from pathlib import Path

repo_root = Path(sys.argv[1])
sys.path.insert(0, str(repo_root / "src"))

try:
    with redirect_stdout(io.StringIO()):
        from gradient_os import runtime_config
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
    robot_name = str(resolved.get("robot", {}).get("name", "unknown")).strip() or "unknown"
    ik_name = str(resolved.get("ik_solver", {}).get("effective_backend", "unknown")).strip() or "unknown"
    servo_name = str(resolved.get("servo_backend", {}).get("effective_backend", "unknown")).strip() or "unknown"
    drive_name = str(resolved.get("drive_profile", {}).get("configured_profile", "unknown")).strip() or "unknown"
    tool_name = str(resolved.get("robot", {}).get("active_tool_id", "none")).strip() or "none"
    max_rpm = resolved.get("rtcore", {}).get("configured_max_rpm")
    max_rpm_label = str(int(max_rpm)) if isinstance(max_rpm, (int, float)) and float(max_rpm).is_integer() else str(max_rpm or "default")
    print(f"robot={robot_name}")
    print(f"ik={ik_name}")
    print(f"servo={servo_name}")
    print(f"drive={drive_name}")
    print(f"tool={tool_name}")
    print(f"rt_max_rpm={max_rpm_label}")
except Exception as exc:
    print("robot=unknown")
    print("ik=unknown")
    print("servo=unknown")
    print("drive=unknown")
    print("tool=unknown")
    print("rt_max_rpm=unknown")
    print(f"runtime_error={exc}")
PY
}

print_start_banner() {
  init_banner_palette

  local runtime_summary=""
  runtime_summary="$(render_runtime_banner_summary 2>/dev/null || true)"

  local robot="unknown"
  local ik="unknown"
  local servo="unknown"
  local drive="unknown"
  local tool_id="unknown"
  local rt_max_rpm="unknown"
  local runtime_error=""

  if [[ -n "${runtime_summary}" ]]; then
    robot="$(printf '%s\n' "${runtime_summary}" | awk -F= '/^robot=/{print substr($0, index($0, "=")+1)}' | tail -n 1)"
    ik="$(printf '%s\n' "${runtime_summary}" | awk -F= '/^ik=/{print substr($0, index($0, "=")+1)}' | tail -n 1)"
    servo="$(printf '%s\n' "${runtime_summary}" | awk -F= '/^servo=/{print substr($0, index($0, "=")+1)}' | tail -n 1)"
    drive="$(printf '%s\n' "${runtime_summary}" | awk -F= '/^drive=/{print substr($0, index($0, "=")+1)}' | tail -n 1)"
    tool_id="$(printf '%s\n' "${runtime_summary}" | awk -F= '/^tool=/{print substr($0, index($0, "=")+1)}' | tail -n 1)"
    rt_max_rpm="$(printf '%s\n' "${runtime_summary}" | awk -F= '/^rt_max_rpm=/{print substr($0, index($0, "=")+1)}' | tail -n 1)"
    runtime_error="$(printf '%s\n' "${runtime_summary}" | awk -F= '/^runtime_error=/{print substr($0, index($0, "=")+1)}' | tail -n 1)"
  fi

  BOOT_SUMMARY_ROBOT="${robot}"
  BOOT_SUMMARY_IK="${ik}"
  BOOT_SUMMARY_SERVO="${servo}"
  BOOT_SUMMARY_DRIVE="${drive}"
  BOOT_SUMMARY_TOOL="${tool_id}"
  BOOT_SUMMARY_RT_MAX_RPM="${rt_max_rpm}"

  local mode_label="full stack"
  local web_label="http://127.0.0.1:${WEB_PORT}"
  if [[ "${HEADLESS}" -eq 1 ]]; then
    mode_label="headless"
    web_label="disabled (--headless)"
  fi

  printf '\n%b%s%b\n' "${BANNER_BORDER}" "========================================================================" "${BANNER_RESET}"
  printf '  %b%s%b\n' "${UI_WARN}" "// FOR INDUSTRIAL USE ONLY // LIVE STACK INITIALIZATION //" "${BANNER_RESET}"
  printf '%b%s%b\n' "${BANNER_TITLE_PRIMARY}" '    ____               _ _            _    ___  ____' "${BANNER_RESET}"
  printf '%b%s%b\n' "${BANNER_TITLE_PRIMARY}" '   / ___|_ __ __ _  __| (_) ___ _ __ | |_ / _ \/ ___|' "${BANNER_RESET}"
  printf '%b%s%b\n' "${BANNER_TITLE_SECONDARY}" "  | |  _| '__/ _\` |/ _\` | |/ _ \\ '_ \\| __| | | \\___ \\" "${BANNER_RESET}"
  printf '%b%s%b\n' "${BANNER_TITLE_SECONDARY}" '  | |_| | | | (_| | (_| | |  __/ | | | |_| |_| |___) |' "${BANNER_RESET}"
  printf '%b%s%b\n' "${BANNER_TITLE_SECONDARY}" '   \____|_|  \__,_|\__,_|_|\___|_| |_|\__|\___/|____/' "${BANNER_RESET}"
  printf '\n'
  printf '  %b%s%b\n' "${BANNER_MUTED}" "Gradient Industrial Robotics // Stack launcher" "${BANNER_RESET}"
  banner_stat_line "mode:" "${mode_label}" "run:" "${RUN_ID}"
  banner_stat_line "robot:" "${robot}" "tool:" "${tool_id}"
  banner_stat_line "ik:" "${ik}" "servo:" "${servo}"
  banner_stat_line "drive:" "${drive}" "rt max rpm:" "${rt_max_rpm}"
  printf '  %b%-11s%b %b%s%b\n' "${BANNER_LABEL}" "controller:" "${BANNER_RESET}" "${BANNER_VALUE}" "udp://${CONTROLLER_HOST}:${CONTROLLER_PORT}" "${BANNER_RESET}"
  printf '  %b%-11s%b %b%s%b\n' "${BANNER_LABEL}" "api:" "${BANNER_RESET}" "${BANNER_VALUE}" "http://127.0.0.1:${API_PORT}" "${BANNER_RESET}"
  printf '  %b%-11s%b %b%s%b\n' "${BANNER_LABEL}" "web:" "${BANNER_RESET}" "${BANNER_VALUE}" "${web_label}" "${BANNER_RESET}"
  printf '  %b%-11s%b %b%s%b\n' "${BANNER_LABEL}" "logs:" "${BANNER_RESET}" "${BANNER_VALUE}" "${LOG_DIR}" "${BANNER_RESET}"
  printf '  %b%s%b %s %b|%b %s %b|%b %s %b|%b %s\n' \
    "${BANNER_LABEL}" "commands:" "${BANNER_RESET}" \
    "$(style_cmd 'probe')" "${BANNER_MUTED}" "${BANNER_RESET}" \
    "$(style_cmd 'status')" "${BANNER_MUTED}" "${BANNER_RESET}" \
    "$(style_cmd 'stop')" "${BANNER_MUTED}" "${BANNER_RESET}" \
    "$(style_cmd 'stop --hard')"
  printf '%b%s%b\n' "${BANNER_BORDER}" "========================================================================" "${BANNER_RESET}"

  if [[ -n "${runtime_error}" ]]; then
    warn "banner runtime summary fallback: ${runtime_error}"
  fi
}

sync_rtcore_runtime_once() {
  local started_ms
  started_ms="$(now_ms)"
  info "RTCore sync: ensuring unit, binary, and runtime env match the selected robot scaling"
  local detail=""
  if run_with_loading_capture detail "rtcore sync" "Verifying service unit, binary install, and EtherCAT runtime env" 45 "${RTCORE_SYNC_SCRIPT}" --ensure-active; then
    ui_status_clear
    while IFS= read -r line; do
      [[ -n "${line}" ]] || continue
      info "${line}"
    done <<< "${detail}"
    local elapsed_ms
    elapsed_ms=$(( $(now_ms) - started_ms ))
    success "$(style_ok 'RTCORE SYNC COMPLETE') in $(format_duration_ms "${elapsed_ms}")"
    return 0
  fi
  ui_status_clear
  local elapsed_ms
  elapsed_ms=$(( $(now_ms) - started_ms ))
  print_callout_block "${UI_DANGER}" "RTCORE SYNC FAILURE" \
    "  status:  $(style_danger 'RTCORE SERVICE DID NOT START CLEANLY')" \
    "  timing:  $(style_warn "$(format_duration_ms "${elapsed_ms}")")" \
    "  inspect: $(style_cmd 'systemctl status gradient-rt-motion.service')" \
    "           $(style_cmd 'journalctl -xeu gradient-rt-motion.service')"
  while IFS= read -r line; do
    [[ -n "${line}" ]] || continue
    warn "rtcore sync detail: ${line}"
  done <<< "${detail}"
  error "RTCore runtime sync failed; refusing to start with potentially stale scaling."
  return 1
}

attempt_rtcore_startup_recovery_once() {
  local started_ms
  started_ms="$(now_ms)"
  log "Attempting $(style_warn 'HARD RTCORE/ETHERCAT RECYCLE') before controller launch"
  stop_rtcore_runtime || true
  stop_ethercat_master_runtime
  sleep 1
  sync_rtcore_runtime_once || return 1
  sleep 1
  success "recovery recycle finished in $(format_duration_ms $(( $(now_ms) - started_ms )))"
  return 0
}

ensure_rtcore_runtime_sync() {
  sync_rtcore_runtime_once || return 1

  local recovery_attempted=0
  while true; do
    local payload=""
    payload="$(capture_probe_json || true)"
    if [[ -z "${payload}" ]]; then
      return 0
    fi
    if probe_is_bus_ready "${payload}"; then
      return 0
    fi

    local recent_journal=""
    recent_journal="$(capture_rtcore_recent_journal)"
    local plan=""
    if ! plan="$(probe_rtcore_startup_recovery_plan "${payload}" "${recent_journal}" "${recovery_attempted}" 2>/dev/null)"; then
      return 0
    fi

    local should_recover=""
    local reboot_required=""
    local reason=""
    local detail=""
    local journal_signatures=""
    should_recover="$(probe_json_field "${plan}" "should_recover")"
    reboot_required="$(probe_json_field "${plan}" "reboot_required")"
    reason="$(probe_json_field "${plan}" "reason")"
    detail="$(probe_json_field "${plan}" "detail")"
    journal_signatures="$(probe_json_field "${plan}" "journal_signatures")"

    if [[ "${reboot_required}" == "1" ]]; then
      log_probe_snapshot "${payload}"
      if [[ -n "${journal_signatures}" ]]; then
        warn "RTCore startup recovery signatures: ${journal_signatures}"
      fi
      print_callout_block "${UI_DANGER}" "REBOOT REQUIRED" \
        "  detail: ${detail}" \
        "  status: $(style_danger 'STALE RTCORE OWNER STILL HOLDS ETHERCAT MASTER')" \
        "  action: $(style_danger 'REBOOT HOST')" \
        "  next:   after boot, rerun $(style_cmd './start-stack.sh') and use $(style_cmd 'probe') or $(style_cmd 'status') to confirm clean startup"
      error "${detail}"
      error "RTCore/EtherCAT ownership did not clear after one recycle attempt; $(style_danger 'REBOOT HOST') before retrying startup."
      return 1
    fi

    if [[ "${should_recover}" == "1" && "${recovery_attempted}" == "0" ]]; then
      log_probe_snapshot "${payload}"
      print_callout_block "${UI_WARN}" "STARTUP RECOVERY PLAN" \
        "  classification: $(style_warn "${reason}")" \
        "  detail:         ${detail}" \
        "  action:         $(style_warn 'ONE HARD RTCORE/ETHERCAT RECYCLE')" \
        "  if it still fails: expect $(style_danger 'REBOOT REQUIRED') rather than looping restarts"
      log "RTCore startup recovery classified $(style_warn "${reason}")"
      recovery_attempted=1
      attempt_rtcore_startup_recovery_once || return 1
      continue
    fi

    if [[ "${recovery_attempted}" == "1" && -n "${detail}" && "${reason}" != "healthy" && "${reason}" != "no_action" ]]; then
      log "RTCore startup status after one recycle attempt: ${detail}"
    fi
    return 0
  done
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

  probe_hardware_state "${launcher_status}" "${state_log_dir}"
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
    if [[ "${HARD_STOP}" -eq 1 ]]; then
      if [[ -z "${WEB_PID:-}" ]]; then
        WEB_PID="$(discover_pid_by_pattern 'vite')"
      fi
      if [[ -z "${CONTROLLER_PID:-}" ]]; then
        CONTROLLER_PID="$(discover_pid_by_pattern 'gradient_os.run_controller')"
      fi
      if [[ -z "${API_PID:-}" ]]; then
        API_PID="$(discover_pid_by_pattern 'gradient_os.api.main|gradient-api')"
      fi
      log "Hard stop requested by CLI; completing RTCore/EtherCAT teardown after launcher exit."
      perform_shutdown_sequence
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

try:
    from gradient_os.telemetry.terminal_dashboard import (
        TerminalDashboardState,
        process_service_log_line,
    )
except Exception:
    TerminalDashboardState = None
    process_service_log_line = None

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
            out.append(line.rstrip())
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
dashboard_state = TerminalDashboardState() if TerminalDashboardState is not None else None


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
                if dashboard_state is not None and callable(process_service_log_line):
                    for output_line in process_service_log_line(tail.label, line, dashboard_state):
                        safe_print_line(output_line)
                else:
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
        "[dashboard] controller=ONLINE api=ONLINE"
        + ("" if headless else " web=ONLINE")
        + " canonical_truth=MONITORING",
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
  log "Type $(style_cmd 'stop'), $(style_cmd 'stop --hard'), $(style_cmd 'probe'), $(style_cmd 'status'), $(style_cmd 'help'), or press Ctrl-C."
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
  local boot_started_ms
  boot_started_ms="$(now_ms)"
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
  print_start_banner
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

  success "$(style_ok 'STACK BOOT COMPLETE') in $(format_duration_ms $(( $(now_ms) - boot_started_ms )))"
  print_boot_success_block
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
