#!/bin/bash

set -euo pipefail

echo "=== GradientOS EtherCAT host diagnostics ==="
echo ""

echo "## Kernel"
uname -a || true
echo ""

echo "## CPU"
command -v lscpu >/dev/null 2>&1 && lscpu | sed -n '1,25p' || true
echo ""

echo "## Kernel cmdline"
cat /proc/cmdline || true
echo ""

echo "## RT CPU isolation (post-reboot verification)"
for p in /sys/devices/system/cpu/isolated /sys/devices/system/cpu/nohz_full /sys/devices/system/cpu/rcu_nocbs; do
  if [ -f "$p" ]; then
    echo "- $p: $(cat "$p")"
  fi
done

echo "- irqbalance:"
if command -v systemctl >/dev/null 2>&1; then
  systemctl is-enabled irqbalance.service 2>/dev/null || true
  systemctl is-active irqbalance.service 2>/dev/null || true
fi
echo ""

echo "## CPU governor (best-effort)"
for gov in /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor; do
  if [ -f "${gov}" ]; then
    echo "- ${gov}: $(cat "${gov}")"
  fi
done
echo ""

echo "## Boot cmdline file(s)"
for p in /boot/firmware/cmdline.txt /boot/cmdline.txt; do
  if [ -f "$p" ]; then
    echo "- $p:"
    cat "$p"
  fi
done
echo ""

echo "## NICs"
ip -brief link || true
echo ""

echo "## NetworkManager devices"
command -v nmcli >/dev/null 2>&1 && nmcli dev status || true
echo ""

echo "## EtherCAT prerequisites"
kver="$(uname -r)"
if [ -e "/lib/modules/$kver/build" ]; then
  echo "- kernel headers: present (/lib/modules/$kver/build)"
else
  echo "- kernel headers: MISSING (/lib/modules/$kver/build not found)"
fi

if command -v ethercat >/dev/null 2>&1; then
  echo "- ethercat CLI: present ($(command -v ethercat))"
else
  echo "- ethercat CLI: missing"
fi

echo "- IgH kernel modules:"
if command -v lsmod >/dev/null 2>&1; then
  lsmod | awk 'BEGIN{found=0} $1 ~ /^ec_/ {print "  - " $1; found=1} END{ if(!found) print "  - (none loaded)" }'
fi

if [ -f /etc/ethercat.conf ]; then
  echo "- /etc/ethercat.conf:"
  cat /etc/ethercat.conf
else
  echo "- /etc/ethercat.conf: missing"
fi

echo ""
echo "## systemd units (expected enabled)"
if command -v systemctl >/dev/null 2>&1; then
  for unit in \
    gradient-ethercat-nic-tune.service \
    gradient-irq-affinity.service \
    gradient-cpu-performance.service \
    ethercat.service \
    gradient-rt-motion.service; do
    echo "- ${unit}:"
    systemctl is-enabled "${unit}" 2>/dev/null || true
    systemctl is-active "${unit}" 2>/dev/null || true
  done
fi

echo ""
echo "## EtherCAT NIC IRQ affinity (best-effort)"
IFACE="${1:-}"
if [ -z "${IFACE}" ] && [ -r /etc/gradient-ethercat-host.env ]; then
  # shellcheck disable=SC1091
  . /etc/gradient-ethercat-host.env
  case "${ETHERCAT_PORT:-}" in
    eth0) IFACE="eth0" ;;
    eth1) IFACE="eth1" ;;
  esac
fi
IFACE="${IFACE:-ethercat0}"
if [ -f /proc/interrupts ]; then
  irqs="$(awk -v iface="${IFACE}" '$0 ~ iface { gsub(/:/,"",$1); print $1 }' /proc/interrupts)"
  if [ -n "${irqs}" ]; then
    for irq in ${irqs}; do
      echo "- IRQ ${irq} (${IFACE}):"
      [ -f "/proc/irq/${irq}/smp_affinity" ] && echo "  smp_affinity: $(cat "/proc/irq/${irq}/smp_affinity")"
      [ -f "/proc/irq/${irq}/smp_affinity_list" ] && echo "  smp_affinity_list: $(cat "/proc/irq/${irq}/smp_affinity_list")"
    done
  else
    echo "- No IRQ lines matched '${IFACE}' in /proc/interrupts (try: ./diagnose_host.sh eth1)"
  fi
fi

echo ""
echo "Done."

