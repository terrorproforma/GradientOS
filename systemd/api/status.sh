#!/bin/bash

set -euo pipefail

SERVICE_NAME="gradient-api.service"

sudo systemctl status "${SERVICE_NAME}" --no-pager
