#!/usr/bin/env bash
# Build the ADLabel dev image and start the development container.
# Idempotent: re-run any time. Docker's layer cache + `up -d` handle
# "rebuild only when needed" and "start only if not running".

set -euo pipefail

cd "$(dirname "$0")/.."

# Pass host UID/GID so files created inside the container are owned by
# the host user. Compose reads these from the environment.
export HOST_UID="$(id -u)"
export HOST_GID="$(id -g)"

echo "🚀 Starting ADLabel dev container..."
docker compose -f docker/docker-compose.yml up -d --build

echo ""
echo "✅ Container 'adlabel-dev' is up."
echo "   Enter it with: ./devtools/dev_into.sh"
