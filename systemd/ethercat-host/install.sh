#!/bin/bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
TMP_DIR="$(mktemp -d)"
trap 'rm -rf "${TMP_DIR}"' EXIT

bash "${SCRIPT_DIR}/render-generated-files.sh" --output-dir "${TMP_DIR}"

# shellcheck disable=SC1091
. "${SCRIPT_DIR}/port-layout.env"

case "${ETHERCAT_PORT}" in
  eth0)
    ETHERCAT_MAC="${ETH0_MAC}"
    UPLINK_PORT="eth1"
    UPLINK_MAC="${ETH1_MAC}"
    ;;
  eth1)
    ETHERCAT_MAC="${ETH1_MAC}"
    UPLINK_PORT="eth0"
    UPLINK_MAC="${ETH0_MAC}"
    ;;
  *)
    echo "ERROR: ETHERCAT_PORT must be eth0 or eth1 in ${SCRIPT_DIR}/port-layout.env" >&2
    exit 1
    ;;
esac

echo "Installing EtherCAT host configuration (NIC + tuning templates)..."
echo "NOTE: This modifies /etc and may require a reboot for NIC renaming."

sudo mkdir -p /etc/systemd/network /etc/NetworkManager/conf.d /etc/NetworkManager/system-connections /etc/systemd/system /usr/local/sbin
sudo mkdir -p /etc/systemd/system/ethercat.service.d

echo "0) Installing single-source EtherCAT role config"
sudo install -m 0644 "${SCRIPT_DIR}/port-layout.env" /etc/gradient-ethercat-host.env

echo "1) Installing deterministic NIC naming (.link)"
sudo install -m 0644 "${TMP_DIR}/10-ethercat0.link" /etc/systemd/network/10-ethercat0.link
sudo install -m 0644 "${TMP_DIR}/10-uplink0.link" /etc/systemd/network/10-uplink0.link

echo "2) Marking ethercat0 unmanaged (NetworkManager)"
sudo install -m 0644 "${TMP_DIR}/10-unmanaged-ethercat.conf" /etc/NetworkManager/conf.d/10-unmanaged-ethercat.conf

echo "3) Installing managed uplink NetworkManager profile"
sudo install -m 0600 "${TMP_DIR}/gradient-uplink.nmconnection" /etc/NetworkManager/system-connections/gradient-uplink.nmconnection

echo "4) Installing NIC tune + IRQ affinity scripts"
sudo install -m 0755 "${SCRIPT_DIR}/gradient-ethercat-nic-tune.sh" /usr/local/sbin/gradient-ethercat-nic-tune.sh
sudo install -m 0755 "${SCRIPT_DIR}/gradient-irq-affinity.sh" /usr/local/sbin/gradient-irq-affinity.sh
sudo install -m 0755 "${SCRIPT_DIR}/gradient-cpu-performance.sh" /usr/local/sbin/gradient-cpu-performance.sh

echo "5) Installing systemd units for NIC tune + IRQ affinity + CPU governor"
sudo install -m 0644 "${SCRIPT_DIR}/gradient-ethercat-nic-tune.service" /etc/systemd/system/gradient-ethercat-nic-tune.service
sudo install -m 0644 "${SCRIPT_DIR}/gradient-irq-affinity.service" /etc/systemd/system/gradient-irq-affinity.service
sudo install -m 0644 "${SCRIPT_DIR}/gradient-cpu-performance.service" /etc/systemd/system/gradient-cpu-performance.service

echo "6) Installing /etc/ethercat.conf template (IgH master binds by MAC)"
sudo install -m 0644 "${TMP_DIR}/ethercat.conf" /etc/ethercat.conf

echo "7) Installing ethercat.service override to use /etc/ethercat.conf"
sudo install -m 0644 "${SCRIPT_DIR}/ethercat.service.d/10-gradient.conf" /etc/systemd/system/ethercat.service.d/10-gradient.conf

echo "Reloading systemd daemon..."
sudo systemctl daemon-reload

echo "Enabling NIC tune + IRQ affinity + CPU governor services..."
sudo systemctl enable gradient-ethercat-nic-tune.service
sudo systemctl enable gradient-irq-affinity.service
sudo systemctl enable gradient-cpu-performance.service

echo "Disabling irqbalance (if present)..."
sudo systemctl disable --now irqbalance.service 2>/dev/null || true

echo ""
echo "Done."
echo "Configured role assignment:"
echo "  - EtherCAT port: ${ETHERCAT_PORT} (${ETHERCAT_MAC})"
echo "  - Uplink port: ${UPLINK_PORT} (${UPLINK_MAC})"
echo "  - Uplink profile: ${UPLINK_NM_CONNECTION_ID:-Gradient Uplink}"
echo "  - If a legacy wired profile still uses the old uplink NIC, disable it before using both ports on the same subnet."
echo "Next steps:"
echo "  - (Optional but recommended) Apply RT CPU isolation params:"
echo "      sudo ${SCRIPT_DIR}/rtos-apply-cmdline.sh && reboot"
echo "  - To cut over a live SSH session, start this before moving the uplink cable:"
echo "      sudo bash ${SCRIPT_DIR}/activate-uplink.sh"
echo "  - Reboot to apply NIC renaming according to ${SCRIPT_DIR}/port-layout.env."
echo "  - Install IgH master (ethercat.service + libecrt + ethercat CLI)."

