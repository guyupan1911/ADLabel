#!/usr/bin/env bash
# Attach an interactive shell to the running ADLabel dev container.
# Run ./devtools/dev_start.sh first if the container isn't running yet.

set -euo pipefail

cd "$(dirname "$0")/.."

if ! docker compose -f docker/docker-compose.yml ps | grep -q "Up"; then
    echo "❌ Container is not running"
    echo ""
    echo "💡 Please start the container first:"
    echo "   ./devtools/dev_start.sh"
    exit 1
fi

echo "🔗 Entering ADLabel dev container..."
exec docker compose -f docker/docker-compose.yml exec dev bash
