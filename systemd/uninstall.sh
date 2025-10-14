#!/bin/bash

set -euo pipefail

SERVICE_NAME="arm-controller.service"
SERVICE_PATH="/etc/systemd/system/${SERVICE_NAME}"

echo "Uninstalling Gradient Robotics arm controller systemd service..."

echo "Stopping service..."
sudo systemctl stop "${SERVICE_NAME}" || true

echo "Disabling service..."
sudo systemctl disable "${SERVICE_NAME}"

echo "Removing service file..."
sudo rm -f "${SERVICE_PATH}"

echo "Reloading systemd daemon..."
sudo systemctl daemon-reload

echo "Service uninstalled successfully!"
