#!/usr/bin/env bash
# Start the ADLabel development container.
# By default this does not rebuild the image. Pass --build when the image needs
# to be rebuilt; Docker Compose will still create the image if it is missing.

set -euo pipefail

cd "$(dirname "$0")/.."

# Pass host UID/GID so files created inside the container are owned by
# the host user. Compose reads these from the environment.
export HOST_UID="$(id -u)"
export HOST_GID="$(id -g)"

COMPOSE_FILE="docker/docker-compose.yml"

BUILD_IMAGE=false
if [[ "${1:-}" == "--build" ]]; then
    BUILD_IMAGE=true
elif [[ -n "${1:-}" ]]; then
    echo "Usage: $0 [--build]"
    exit 1
fi

echo "🚀 Starting ADLabel dev container..."
echo ""

compose_args=(up -d)
if [[ "$BUILD_IMAGE" == "true" ]]; then
    compose_args+=(--build)
    echo "▶️  Building image and starting container..."
else
    echo "▶️  Starting container..."
    echo "   Docker Compose will recreate the container if docker-compose.yml changed."
    echo "   It will only build the image when the configured image is missing."
fi

docker compose -f "$COMPOSE_FILE" "${compose_args[@]}"

echo ""
echo "✅ Container 'adlabel-dev' is up."
echo "   Enter it with: ./devtools/dev_into.sh"
