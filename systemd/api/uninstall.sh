#!/bin/bash

set -euo pipefail

SERVICE_NAME="gradient-api.service"
SERVICE_PATH="/etc/systemd/system/${SERVICE_NAME}"

if ! sudo systemctl list-unit-files | grep -q "${SERVICE_NAME}"; then
  echo "Service ${SERVICE_NAME} not installed. Nothing to do."
  exit 0
fi

echo "Stopping service..."
sudo systemctl stop "${SERVICE_NAME}" || true

echo "Disabling service..."
sudo systemctl disable "${SERVICE_NAME}" || true

if [[ -f "${SERVICE_PATH}" ]]; then
  echo "Removing service file..."
  sudo rm -f "${SERVICE_PATH}"
fi

echo "Reloading systemd daemon..."
sudo systemctl daemon-reload

echo "Service ${SERVICE_NAME} removed."
