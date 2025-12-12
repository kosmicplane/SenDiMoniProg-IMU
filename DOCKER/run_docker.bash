#!/bin/bash
set -euo pipefail

SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
COMPOSE_FILE="$SCRIPT_DIR/docker-compose.yml"

XAUTHORITY=${XAUTHORITY:-/tmp/.docker.xauth}
export DISPLAY=${DISPLAY:-:0}
export QT_X11_NO_MITSHM=1

# Ensure X11 access for root inside the container.
xhost +local:root || true

# Pre-create the Xauthority file so the mount always exists.
touch "$XAUTHORITY"

echo "Starting sendi-dev:local via docker compose..."
docker compose -f "$COMPOSE_FILE" up "$@"
