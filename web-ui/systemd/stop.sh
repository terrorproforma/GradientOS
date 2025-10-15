#!/bin/bash

set -euo pipefail

SERVICE_NAME="gradient-api.service"

echo "Stopping ${SERVICE_NAME}..."
sudo systemctl stop "${SERVICE_NAME}"
