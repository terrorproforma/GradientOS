#!/bin/bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
TIMEOUT_S=120

while [ "$#" -gt 0 ]; do
  case "$1" in
    --timeout)
      TIMEOUT_S="$2"
      shift 2
      ;;
    *)
      echo "ERROR: unknown arg: $1" >&2
      echo "Usage: sudo bash ./activate-uplink.sh [--timeout SECONDS]" >&2
      exit 2
      ;;
  esac
done

# shellcheck disable=SC1091
. "${SCRIPT_DIR}/port-layout.env"

case "${ETHERCAT_PORT}" in
  eth0)
    UPLINK_PORT="eth1"
    ;;
  eth1)
    UPLINK_PORT="eth0"
    ;;
  *)
    echo "ERROR: ETHERCAT_PORT must be eth0 or eth1 in ${SCRIPT_DIR}/port-layout.env" >&2
    exit 1
    ;;
esac

UPLINK_PROFILE="${UPLINK_NM_CONNECTION_ID:-Gradient Uplink}"

if ! command -v nmcli >/dev/null 2>&1; then
  echo "ERROR: nmcli is required for uplink activation." >&2
  exit 1
fi

if [ "${EUID}" -ne 0 ]; then
  echo "ERROR: run this script with sudo so it can control NetworkManager." >&2
  exit 1
fi

if ! [[ "${TIMEOUT_S}" =~ ^[0-9]+$ ]]; then
  echo "ERROR: --timeout must be a non-negative integer" >&2
  exit 2
fi

busctl_nmcli() {
  env DBUS_SYSTEM_BUS_ADDRESS=unix:path=/run/dbus/system_bus_socket nmcli "$@"
}

active_connection_on_device() {
  busctl_nmcli -t -f NAME,DEVICE connection show --active |
    awk -F: -v dev="$1" '$2 == dev {print $1; exit}'
}

device_has_carrier() {
  local dev="$1"
  local carrier_file="/sys/class/net/${dev}/carrier"
  if [ -r "${carrier_file}" ]; then
    [ "$(tr -d '\n' < "${carrier_file}")" = "1" ]
    return
  fi
  ip -o link show dev "${dev}" | awk '{print $0}' | grep -q "LOWER_UP"
}

wait_for_carrier() {
  local dev="$1"
  local timeout_s="$2"
  local waited=0
  while ! device_has_carrier "${dev}"; do
    if [ "${waited}" -ge "${timeout_s}" ]; then
      return 1
    fi
    sleep 1
    waited=$((waited + 1))
  done
}

echo "Preparing uplink cutover:"
echo "  EtherCAT port: ${ETHERCAT_PORT}"
echo "  Uplink port:   ${UPLINK_PORT}"
echo "  Uplink profile:${UPLINK_PROFILE}"
echo
echo "Start this script BEFORE unplugging the current uplink cable."
echo "It will keep the current ${ETHERCAT_PORT} session alone, wait for carrier on ${UPLINK_PORT},"
echo "then bring up the staged uplink profile on ${UPLINK_PORT}."
echo

# Load any staged profile changes without force-restarting the whole service.
busctl_nmcli connection reload >/dev/null 2>&1 || true

ACTIVE_ON_ETHERCAT="$(
  active_connection_on_device "${ETHERCAT_PORT}"
)"
ACTIVE_ON_UPLINK="$(
  active_connection_on_device "${UPLINK_PORT}"
)"

if [ -n "${ACTIVE_ON_ETHERCAT}" ]; then
  echo "Current active connection on ${ETHERCAT_PORT}: ${ACTIVE_ON_ETHERCAT}"
fi
if [ -n "${ACTIVE_ON_UPLINK}" ]; then
  echo "Current active connection on ${UPLINK_PORT}: ${ACTIVE_ON_UPLINK}"
fi

if device_has_carrier "${UPLINK_PORT}"; then
  echo "Carrier already present on ${UPLINK_PORT}; activating uplink now."
else
  echo "Waiting up to ${TIMEOUT_S}s for carrier on ${UPLINK_PORT}..."
  echo "Move the uplink cable to ${UPLINK_PORT} now."
  if ! wait_for_carrier "${UPLINK_PORT}" "${TIMEOUT_S}"; then
    echo "ERROR: no carrier detected on ${UPLINK_PORT} within ${TIMEOUT_S}s." >&2
    echo "Nothing was deactivated on ${ETHERCAT_PORT}." >&2
    exit 1
  fi
fi

busctl_nmcli connection up "${UPLINK_PROFILE}" ifname "${UPLINK_PORT}"

if [ -n "${ACTIVE_ON_ETHERCAT}" ]; then
  echo "Deactivating old connection on ${ETHERCAT_PORT}: ${ACTIVE_ON_ETHERCAT}"
  busctl_nmcli connection down "${ACTIVE_ON_ETHERCAT}" >/dev/null 2>&1 || true
fi

echo
nmcli device status
echo
ip -4 addr show dev "${UPLINK_PORT}" || true

echo
echo "If ${ETHERCAT_PORT} still shows up as a managed NetworkManager device,"
echo "reboot to apply the unmanaged-device policy before using it for EtherCAT."
