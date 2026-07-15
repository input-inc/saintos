#!/usr/bin/env bash
#
# Wrapper around the e2e docker-compose that reaps the dangling image
# `docker compose ... --build` leaves behind on every rebuild.
#
# Each `--build` retags the `docker-e2e` image onto a fresh id and
# orphans the previous one as a <none> dangling image (~1.3-1.8GB). Run
# by hand a few dozen times and that's 10s of GB of garbage. This
# wrapper runs the exact same compose invocation the README documents,
# then removes only the orphaned images carrying the e2e compose service
# label — so it never touches unrelated dangling images on the host.
#
# Usage (from anywhere) — mirrors the README's modes:
#   firmware/simulation/docker/run-e2e.sh              # MODE=full (default)
#   MODE=fake  firmware/simulation/docker/run-e2e.sh   # Python fake firmware
#   MODE=smoke firmware/simulation/docker/run-e2e.sh   # sim bring-up only
#   MODE=shell firmware/simulation/docker/run-e2e.sh   # debug shell
#
# Any extra args are passed through to the compose command.
#
# The raw `docker compose -f docker-compose.e2e.yml up --build ...`
# commands in README.md still work — this just cleans up after itself.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
COMPOSE_FILE="$SCRIPT_DIR/docker-compose.e2e.yml"
export MODE="${MODE:-full}"

# Reap images orphaned by `--build`. Filter on the compose service label
# so only this stack's dangling images are removed — never anything else
# the developer has on the host.
reap_orphaned_e2e_images() {
    local ids
    ids="$(docker images \
        --filter=dangling=true \
        --filter=label=com.docker.compose.service=e2e \
        --format '{{.ID}}' | sort -u)"
    if [ -n "$ids" ]; then
        echo "[run-e2e] reaping $(printf '%s\n' "$ids" | wc -l | tr -d ' ') orphaned e2e image(s)"
        # shellcheck disable=SC2086
        docker image rm $ids >/dev/null 2>&1 || true
    fi
}
trap reap_orphaned_e2e_images EXIT

if [ "$MODE" = "shell" ]; then
    docker compose -f "$COMPOSE_FILE" run --rm --service-ports e2e "$@"
else
    docker compose -f "$COMPOSE_FILE" up \
        --build --abort-on-container-exit --exit-code-from e2e "$@"
fi
