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

IMAGE_NAME="adlabel-dev:0.1.0"
DOCKERFILE="docker/Dockerfile"
COMPOSE_FILE="docker/docker-compose.yml"

build_args=()
if ! docker image inspect "$IMAGE_NAME" >/dev/null 2>&1; then
    echo "🔨 Image '$IMAGE_NAME' not found; building it first..."
    build_args+=(--build)
else
    image_created="$(docker image inspect -f '{{.Created}}' "$IMAGE_NAME")"
    image_created_epoch="$(date -d "$image_created" +%s)"
    dockerfile_mtime_epoch="$(date -r "$DOCKERFILE" +%s)"

    if (( dockerfile_mtime_epoch > image_created_epoch )); then
        echo "🔨 $DOCKERFILE is newer than '$IMAGE_NAME'; rebuilding..."
        build_args+=(--build)
    else
        echo "✅ Image '$IMAGE_NAME' is up to date; starting without rebuild."
    fi
fi

echo "🚀 Starting ADLabel dev container..."
docker compose -f "$COMPOSE_FILE" up -d "${build_args[@]}"

echo ""
echo "✅ Container 'adlabel-dev' is up."
echo "   Enter it with: ./devtools/dev_into.sh"
