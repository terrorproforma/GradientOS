#!/bin/bash

set -euo pipefail

SERVICE_NAME="gradient-api.service"

echo "Restarting ${SERVICE_NAME}..."
sudo systemctl restart "${SERVICE_NAME}"

sudo systemctl status "${SERVICE_NAME}" --no-pager
