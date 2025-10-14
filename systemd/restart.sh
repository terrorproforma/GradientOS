#!/bin/bash

set -euo pipefail

SERVICE_NAME="arm-controller.service"

echo "Restarting Gradient Robotics arm controller service..."
sudo systemctl restart "${SERVICE_NAME}"

echo "Service status:"
sudo systemctl status "${SERVICE_NAME}" --no-pager

echo "Service restarted successfully!"
