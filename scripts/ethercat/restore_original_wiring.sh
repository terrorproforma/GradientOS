#!/bin/bash
set -euo pipefail

# Restore the currently configured appliance EtherCAT role assignment.
# The active port/MAC comes from /etc/ethercat.conf, which is now derived from
# /etc/gradient-ethercat-host.env installed by systemd/ethercat-host/install.sh.

echo "Stopping any running EtherCAT master modules..."
sudo /usr/local/sbin/ethercatctl -c /etc/ethercat.conf stop || true

echo "Starting ethercat.service (uses /etc/ethercat.conf)..."
sudo systemctl start ethercat.service

echo ""
echo "Verify:"
echo "  sudo ethercat master"
echo "  sudo ethercat slaves -v"

