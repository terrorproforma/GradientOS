#!/bin/sh
set -eu

# Pin EtherCAT NIC IRQs to CPU2-CPU3 (0xC on a 4-core CPU).
#
# NOTE:
# - This script is intentionally conservative and best-effort.
# - IRQ thread priority raising is left as a follow-up once IRQ names are verified on-host.

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
MASK_HEX="c"

if [ ! -f /proc/interrupts ]; then
  exit 0
fi

# Prefer sysfs-backed device IRQ discovery because some drivers expose only the
# driver name in /proc/interrupts (for example `lan743x` instead of `eth1`).
if [ -d "/sys/class/net/${IFACE}/device/msi_irqs" ]; then
  IRQS="$(for p in /sys/class/net/${IFACE}/device/msi_irqs/*; do
    [ -e "${p}" ] || continue
    basename "${p}"
  done)"
else
  DRIVER="$(ethtool -i "${IFACE}" 2>/dev/null | awk -F': *' '/^driver:/{print $2; exit}')"
  IRQS="$(awk -v iface="${IFACE}" -v driver="${DRIVER}" '
    ($0 ~ iface) || (driver != "" && $0 ~ driver) {
      gsub(/:/, "", $1)
      print $1
    }
  ' /proc/interrupts)"
fi

for irq in ${IRQS}; do
  if [ -n "${irq}" ] && [ -d "/proc/irq/${irq}" ]; then
    printf '%s' "${MASK_HEX}" > "/proc/irq/${irq}/smp_affinity" || true
  fi
done

exit 0

