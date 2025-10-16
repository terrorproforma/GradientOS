#!/bin/bash

# Launcher for GradientOS vision tooling with platform-aware defaults.

set -euo pipefail

cd "$(dirname "$0")"

REPO_ROOT="${PWD}"
VENV_BIN="${REPO_ROOT}/.venv/bin"

if [[ -d "${VENV_BIN}" ]]; then
  export PATH="${VENV_BIN}:${PATH}"
fi
export PYTHONPATH="${REPO_ROOT}/src:${PYTHONPATH:-}"

UV_AVAILABLE=false
if command -v uv >/dev/null 2>&1; then
  UV_AVAILABLE=true
  VISION_BASE_CMD=(uv run gradient-vision)
else
  VISION_BASE_CMD=(python -m gradient_os.vision.cli)
fi

check_ai_with() {
  local cmd=("$@")
  if "${cmd[@]}" <<'PY' >/dev/null 2>&1; then
import importlib.util
mods = ("torch", "ultralytics")
missing = [m for m in mods if importlib.util.find_spec(m) is None]
raise SystemExit(0 if not missing else 1)
PY
    return 0
  fi
  return 1
}

AI_READY=false
if command -v python >/dev/null 2>&1 && check_ai_with python -; then
  AI_READY=true
elif command -v python3 >/dev/null 2>&1 && check_ai_with python3 -; then
  AI_READY=true
elif $UV_AVAILABLE && check_ai_with uv run python -; then
  AI_READY=true
fi

is_raspi=false
if [[ "$(uname -s)" == "Linux" ]]; then
  if [[ -f /proc/device-tree/model ]] && grep -qi "raspberry pi" /proc/device-tree/model 2>/dev/null; then
    is_raspi=true
  elif [[ -f /etc/os-release ]] && grep -qiE 'raspberry ?pi|raspbian|raspios' /etc/os-release; then
    is_raspi=true
  fi
fi

if [[ $# -gt 0 ]]; then
  USER_ARGS=("$@")
else
  USER_ARGS=()
fi
ORIGINAL_ARGC=$#

request_help=false
for arg in "${USER_ARGS[@]:-}"; do
  if [[ "$arg" == "-h" || "$arg" == "--help" ]]; then
    request_help=true
    break
  fi
done

if ! $request_help; then
  if [[ ${#USER_ARGS[@]} -eq 0 ]]; then
    USER_ARGS=(mjpeg)
    if $AI_READY; then
      USER_ARGS+=(ai)
    fi
  fi

  if ! "${is_raspi}"; then
    has_backend=false
    for ((i=0; i<${#USER_ARGS[@]}; i++)); do
      if [[ "${USER_ARGS[i]}" == "--backend" ]]; then
        has_backend=true
        break
      fi
    done
    if ! $has_backend; then
      tmp_args=()
      if [[ ${#USER_ARGS[@]} -gt 0 ]]; then
        tmp_args=("${USER_ARGS[@]:-}")
      fi
      USER_ARGS=(--backend usb)
      if [[ ${#tmp_args[@]} -gt 0 ]]; then
        USER_ARGS+=("${tmp_args[@]}")
      fi
    fi
  fi

  commands=(list init processing stream mjpeg)
  command_token=""
  for arg in "${USER_ARGS[@]:-}"; do
    case "$arg" in
      --*) continue ;;
      list|init|processing|stream|mjpeg)
        command_token="$arg"
        break
        ;;
    esac
  done

  if [[ -z $command_token ]]; then
    USER_ARGS+=(mjpeg)
    command_token="mjpeg"
  fi

  if ! "${is_raspi}"; then
    has_camera=false
    for ((i=0; i<${#USER_ARGS[@]}; i++)); do
      if [[ "${USER_ARGS[i]}" == "--camera" ]]; then
        has_camera=true
        break
      fi
    done
    if ! $has_camera && { [[ "$command_token" == "stream" ]] || [[ "$command_token" == "mjpeg" ]]; }; then
      insert_index=${#USER_ARGS[@]}
      for ((i=0; i<${#USER_ARGS[@]}; i++)); do
        case "${USER_ARGS[i]}" in
          ai|ai-seg|ai-pose|img-proc)
            insert_index=$i
            break
            ;;
        esac
      done
      new_args=()
      for ((i=0; i<insert_index; i++)); do
        new_args+=("${USER_ARGS[i]}")
      done
      new_args+=(--camera "${VISION_CAMERA:-0}")
      for ((i=insert_index; i<${#USER_ARGS[@]}; i++)); do
        new_args+=("${USER_ARGS[i]}")
      done
      USER_ARGS=("${new_args[@]}")
    fi
  fi
fi

if ! $request_help && [[ $ORIGINAL_ARGC -eq 0 ]]; then
  default_notice="gradient-vision mjpeg"
  if $AI_READY; then
    default_notice+=" ai"
  fi
  if ! "${is_raspi}"; then
    default_notice+=" --camera ${VISION_CAMERA:-0}"
  fi
  echo "[gradient-robotics] No arguments supplied; defaulting to '${default_notice}'. Pass '--help' for options." >&2
fi

echo "[gradient-robotics] Launching vision CLI from ${REPO_ROOT}"
exec "${VISION_BASE_CMD[@]}" "${USER_ARGS[@]}"
