#!/bin/bash
set -euo pipefail

# Legacy helper kept for convenience. Prefer the docker compose workflow documented in README.
SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
COMPOSE_FILE="$SCRIPT_DIR/docker-compose.offline.yml"

XAUTHORITY=${XAUTHORITY:-/tmp/.docker.xauth}
export DISPLAY=${DISPLAY:-:0}
export QT_X11_NO_MITSHM=1

# Ensure X11 access for root inside the container.
xhost local:root || true

# Pre-create the Xauthority file so the mount always exists.
touch "$XAUTHORITY"

echo "Starting badkitten:offline via docker compose (offline run)..."
docker compose -f "$COMPOSE_FILE" up --remove-orphans "$@"
