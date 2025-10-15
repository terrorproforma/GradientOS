#!/bin/bash

set -euo pipefail

SERVICE_NAME="arm-controller.service"

echo "Gradient Robotics arm controller service status:"
sudo systemctl status "${SERVICE_NAME}" --no-pager

echo ""
echo "Recent logs:"
sudo journalctl -u "${SERVICE_NAME}" -n 20 --no-pager
