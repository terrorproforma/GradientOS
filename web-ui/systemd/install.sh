#!/bin/bash

set -euo pipefail

SERVICE_NAME="gradient-api.service"
SERVICE_PATH="/etc/systemd/system/${SERVICE_NAME}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

echo "Installing GradientOS HTTP API systemd service..."

sudo cp "${SCRIPT_DIR}/${SERVICE_NAME}" "${SERVICE_PATH}"

echo "Reloading systemd daemon..."
sudo systemctl daemon-reload

echo "Enabling service..."
sudo systemctl enable "${SERVICE_NAME}"

echo "Starting service..."
sudo systemctl start "${SERVICE_NAME}"

echo "Service status:"
sudo systemctl status "${SERVICE_NAME}" --no-pager

echo "Service installed successfully!"
