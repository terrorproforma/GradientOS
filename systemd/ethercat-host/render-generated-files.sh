#!/bin/bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CONFIG_FILE="${SCRIPT_DIR}/port-layout.env"
OUTPUT_DIR="${SCRIPT_DIR}"

while [ "$#" -gt 0 ]; do
  case "$1" in
    --config)
      CONFIG_FILE="$2"
      shift 2
      ;;
    --output-dir)
      OUTPUT_DIR="$2"
      shift 2
      ;;
    *)
      echo "Unknown arg: $1" >&2
      exit 2
      ;;
  esac
done

if [ ! -r "${CONFIG_FILE}" ]; then
  echo "Config file not readable: ${CONFIG_FILE}" >&2
  exit 1
fi

# shellcheck disable=SC1090
. "${CONFIG_FILE}"

case "${ETHERCAT_PORT:-}" in
  eth0)
    ETHERCAT_MAC="${ETH0_MAC:-}"
    UPLINK_PORT="eth1"
    UPLINK_MAC="${ETH1_MAC:-}"
    ;;
  eth1)
    ETHERCAT_MAC="${ETH1_MAC:-}"
    UPLINK_PORT="eth0"
    UPLINK_MAC="${ETH0_MAC:-}"
    ;;
  *)
    echo "ETHERCAT_PORT must be eth0 or eth1 in ${CONFIG_FILE}" >&2
    exit 1
    ;;
esac

: "${ETH0_MAC:?ETH0_MAC missing in ${CONFIG_FILE}}"
: "${ETH1_MAC:?ETH1_MAC missing in ${CONFIG_FILE}}"
: "${ETHERCAT_RENAMED_IFACE:?ETHERCAT_RENAMED_IFACE missing in ${CONFIG_FILE}}"
: "${UPLINK_RENAMED_IFACE:?UPLINK_RENAMED_IFACE missing in ${CONFIG_FILE}}"
: "${ETHERCAT_DEVICE_MODULES:?ETHERCAT_DEVICE_MODULES missing in ${CONFIG_FILE}}"

UPLINK_NM_CONNECTION_ID="${UPLINK_NM_CONNECTION_ID:-Gradient Uplink}"
UPLINK_AUTOCONNECT="${UPLINK_AUTOCONNECT:-true}"
UPLINK_IPV4_METHOD="${UPLINK_IPV4_METHOD:-manual}"
UPLINK_IPV4_ADDRESS="${UPLINK_IPV4_ADDRESS:-}"
UPLINK_IPV4_GATEWAY="${UPLINK_IPV4_GATEWAY:-}"
UPLINK_IPV4_DNS="${UPLINK_IPV4_DNS:-}"
UPLINK_IPV4_NEVER_DEFAULT="${UPLINK_IPV4_NEVER_DEFAULT:-true}"
UPLINK_IPV6_METHOD="${UPLINK_IPV6_METHOD:-auto}"
UPLINK_IPV6_NEVER_DEFAULT="${UPLINK_IPV6_NEVER_DEFAULT:-true}"

case "${UPLINK_IPV4_METHOD}" in
  auto|manual|disabled)
    ;;
  *)
    echo "UPLINK_IPV4_METHOD must be auto, manual, or disabled in ${CONFIG_FILE}" >&2
    exit 1
    ;;
esac

if [ "${UPLINK_IPV4_METHOD}" = "manual" ] && [ -z "${UPLINK_IPV4_ADDRESS}" ]; then
  echo "UPLINK_IPV4_ADDRESS must be set when UPLINK_IPV4_METHOD=manual in ${CONFIG_FILE}" >&2
  exit 1
fi

case "${UPLINK_IPV6_METHOD}" in
  auto|manual|disabled|ignore)
    ;;
  *)
    echo "UPLINK_IPV6_METHOD must be auto, manual, disabled, or ignore in ${CONFIG_FILE}" >&2
    exit 1
    ;;
esac

UPLINK_CONNECTION_UUID="$(python3 - "${UPLINK_MAC}" <<'PY'
import sys
import uuid

mac = sys.argv[1].strip().lower()
print(uuid.uuid5(uuid.NAMESPACE_DNS, f"gradient-uplink-{mac}"))
PY
)"

UPLINK_IPV4_DNS_KEYFILE=""
if [ -n "${UPLINK_IPV4_DNS}" ]; then
  UPLINK_IPV4_DNS_KEYFILE="$(printf '%s' "${UPLINK_IPV4_DNS}" | tr ',' ';')"
  case "${UPLINK_IPV4_DNS_KEYFILE}" in
    *';')
      ;;
    *)
      UPLINK_IPV4_DNS_KEYFILE="${UPLINK_IPV4_DNS_KEYFILE};"
      ;;
  esac
fi

mkdir -p "${OUTPUT_DIR}"

cat > "${OUTPUT_DIR}/ethercat.conf" <<EOF
# Generated from systemd/ethercat-host/port-layout.env.
# Edit ETHERCAT_PORT there, then re-run render-generated-files.sh/install.sh.
# Current role assignment:
# - EtherCAT port: ${ETHERCAT_PORT} (${ETHERCAT_MAC}) -> ${ETHERCAT_RENAMED_IFACE}
# - Uplink port: ${UPLINK_PORT} (${UPLINK_MAC}) -> ${UPLINK_RENAMED_IFACE}
MASTER0_DEVICE="${ETHERCAT_MAC}"
DEVICE_MODULES="${ETHERCAT_DEVICE_MODULES}"
EOF

cat > "${OUTPUT_DIR}/10-ethercat0.link" <<EOF
# Generated from systemd/ethercat-host/port-layout.env.
[Match]
MACAddress=${ETHERCAT_MAC}

[Link]
Name=${ETHERCAT_RENAMED_IFACE}
EOF

cat > "${OUTPUT_DIR}/10-uplink0.link" <<EOF
# Generated from systemd/ethercat-host/port-layout.env.
[Match]
MACAddress=${UPLINK_MAC}

[Link]
Name=${UPLINK_RENAMED_IFACE}
EOF

cat > "${OUTPUT_DIR}/10-unmanaged-ethercat.conf" <<EOF
# Generated from systemd/ethercat-host/port-layout.env.
[keyfile]
unmanaged-devices=interface-name:${ETHERCAT_PORT};interface-name:${ETHERCAT_RENAMED_IFACE}
EOF

{
  cat <<EOF
# Generated from systemd/ethercat-host/port-layout.env.
[connection]
id=${UPLINK_NM_CONNECTION_ID}
uuid=${UPLINK_CONNECTION_UUID}
type=802-3-ethernet
autoconnect=${UPLINK_AUTOCONNECT}

[802-3-ethernet]
mac-address=${UPLINK_MAC}

[ipv4]
method=${UPLINK_IPV4_METHOD}
EOF
  if [ "${UPLINK_IPV4_METHOD}" = "manual" ]; then
    echo "address1=${UPLINK_IPV4_ADDRESS}"
  fi
  if [ -n "${UPLINK_IPV4_GATEWAY}" ]; then
    echo "gateway=${UPLINK_IPV4_GATEWAY}"
  fi
  if [ -n "${UPLINK_IPV4_DNS_KEYFILE}" ]; then
    echo "dns=${UPLINK_IPV4_DNS_KEYFILE}"
  fi
  cat <<EOF
never-default=${UPLINK_IPV4_NEVER_DEFAULT}

[ipv6]
method=${UPLINK_IPV6_METHOD}
never-default=${UPLINK_IPV6_NEVER_DEFAULT}

[proxy]
EOF
} > "${OUTPUT_DIR}/gradient-uplink.nmconnection"

