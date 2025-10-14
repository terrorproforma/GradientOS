#!/bin/bash

set -euo pipefail

SERVICE_NAME="arm-controller.service"

echo "Stopping Gradient Robotics arm controller service..."
sudo systemctl stop "${SERVICE_NAME}"

echo "Service stopped successfully!"
