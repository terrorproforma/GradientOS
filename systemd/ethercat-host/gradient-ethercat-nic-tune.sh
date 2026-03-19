#!/bin/sh
set -eu

# Tune EtherCAT NIC for determinism (offloads/EEE).
# This must run BEFORE ethercat.service / RTCore.

resolve_ethercat_iface() {
  DEVICE=""
  PREFERRED_IFACE=""
  if [ -r /etc/ethercat.conf ]; then
    DEVICE="$(awk -F'"' '/^MASTER0_DEVICE=/{print $2; exit}' /etc/ethercat.conf)"
  fi

  if [ -r /etc/gradient-ethercat-host.env ]; then
    # shellcheck disable=SC1091
    . /etc/gradient-ethercat-host.env
    case "${ETHERCAT_PORT:-}" in
      eth0)
        PREFERRED_IFACE="eth0"
        [ -n "${DEVICE}" ] || DEVICE="${ETH0_MAC:-}"
        ;;
      eth1)
        PREFERRED_IFACE="eth1"
        [ -n "${DEVICE}" ] || DEVICE="${ETH1_MAC:-}"
        ;;
    esac
  fi

  if [ -n "${DEVICE}" ] && ip link show "${DEVICE}" >/dev/null 2>&1; then
    printf '%s\n' "${DEVICE}"
    return 0
  fi

  if [ -n "${DEVICE}" ]; then
    IFACE_BY_MAC="$(ip -o link | awk -v want="${DEVICE}" '
      BEGIN { want = tolower(want) }
      {
        name = $2
        sub(/:$/, "", name)
        mac = ""
        for (i = 1; i <= NF; ++i) {
          if ($i == "link/ether") {
            mac = tolower($(i + 1))
            break
          }
        }
        if (mac == want) {
          print name
          exit
        }
      }
    ')"
    if [ -n "${IFACE_BY_MAC}" ]; then
      printf '%s\n' "${IFACE_BY_MAC}"
      return 0
    fi
  fi

  if ip link show "ethercat0" >/dev/null 2>&1; then
    printf '%s\n' "ethercat0"
    return 0
  fi
  if [ -n "${PREFERRED_IFACE}" ] && ip link show "${PREFERRED_IFACE}" >/dev/null 2>&1; then
    printf '%s\n' "${PREFERRED_IFACE}"
    return 0
  fi
  if [ "${PREFERRED_IFACE}" = "eth0" ] && ip link show "eth1" >/dev/null 2>&1; then
    printf '%s\n' "eth1"
    return 0
  fi
  if [ "${PREFERRED_IFACE}" = "eth1" ] && ip link show "eth0" >/dev/null 2>&1; then
    printf '%s\n' "eth0"
    return 0
  fi
  if [ -z "${PREFERRED_IFACE}" ]; then
    if ip link show "eth1" >/dev/null 2>&1; then
      printf '%s\n' "eth1"
      return 0
    fi
    if ip link show "eth0" >/dev/null 2>&1; then
      printf '%s\n' "eth0"
      return 0
    fi
  fi

  return 1
}

IFACE="$(resolve_ethercat_iface || true)"
[ -n "${IFACE}" ] || exit 0

ip link set "${IFACE}" up || true

# Disable common offloads that can add latency/jitter.
ethtool -K "${IFACE}" gro off gso off tso off lro off || true

# Disable Energy Efficient Ethernet (EEE).
ethtool --set-eee "${IFACE}" eee off || true

# Optional: only force speed/duplex if auto-negotiation proves unstable.
# ethtool -s "${IFACE}" speed 100 duplex full autoneg on || true

exit 0

