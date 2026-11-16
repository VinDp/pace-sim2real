#!/usr/bin/env bash
# Helper to build, start, enter and stop the PACE container.
set -euo pipefail

cd "$(dirname "${BASH_SOURCE[0]}")"
COMPOSE="docker compose -f docker-compose.yaml"

case "${1:-}" in
    build) $COMPOSE build ;;
    start)
        command -v xhost >/dev/null && xhost +local:root >/dev/null || true
        $COMPOSE up -d
        ;;
    enter) $COMPOSE exec pace bash ;;
    stop) $COMPOSE down ;;
    # Give files created by the container (data/, logs/) back to the host user
    fix-perms) $COMPOSE exec pace chown -R "$(id -u):$(id -g)" /workspace/pace-sim2real ;;
    *)
        echo "Usage: $0 {build|start|enter|stop|fix-perms}"
        exit 1
        ;;
esac
