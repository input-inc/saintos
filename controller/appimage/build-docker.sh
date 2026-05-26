#!/usr/bin/env bash
#
# Mac/local-host driver for the controller AppImage build.
#
# Builds the SAINT Controller .AppImage inside a linux/amd64 Docker
# container so the resulting binary runs on the Steam Deck (x86_64)
# regardless of the host machine's architecture. On Apple Silicon,
# the amd64 emulation runs through Rosetta if Docker Desktop has it
# enabled (Settings → General → "Use Rosetta for x86/amd64
# emulation"), significantly faster than QEMU for the cargo-heavy
# parts. Unlike the prior flatpak path, there's no bwrap-vs-Rosetta
# conflict because nothing in this pipeline tries to layer a
# kernel-level sandbox.
#
# Caches:
#   ~/.cache/saint-os/controller-appimage/{cargo,target,node_modules,npm-cache}
#       — persistent across builds, bind-mounted as /build. Override
#       the parent path with SAINT_CONTROLLER_CACHE.
#
# First clean build pulls the Rust + npm dependency trees into the
# cache; subsequent builds reuse them and finish in minutes.
#
# Usage:
#   controller/appimage/build-docker.sh                 # incremental build
#   controller/appimage/build-docker.sh --rebuild-image # re-run the Dockerfile
#   controller/appimage/build-docker.sh --clean         # wipe the cache
#   SAINT_CONTROLLER_CACHE=/path build-docker.sh        # override cache root
#
# Output:
#   server/resources/firmware/controller/saint_firmware_controller_<v>-local.<sha>.AppImage
#   server/resources/firmware/controller/info.json

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
CACHE_ROOT="${SAINT_CONTROLLER_CACHE:-${HOME}/.cache/saint-os/controller-appimage}"

IMAGE_TAG="saint-controller-appimage-builder:ubuntu24"
DOCKERFILE="$SCRIPT_DIR/Dockerfile"

REBUILD_IMAGE=0
DO_CLEAN=0

usage() {
    sed -n '2,30p' "$0" | sed 's/^# \{0,1\}//'
}

for arg in "$@"; do
    case "$arg" in
        --rebuild-image) REBUILD_IMAGE=1 ;;
        --clean)         DO_CLEAN=1 ;;
        -h|--help)       usage; exit 0 ;;
        *) echo "Unknown option: $arg" >&2; usage >&2; exit 2 ;;
    esac
done

command -v docker >/dev/null \
    || { echo "docker is required (Docker Desktop on Mac, docker-ce on Linux)" >&2; exit 1; }

if (( DO_CLEAN )); then
    echo "==> --clean: wiping $CACHE_ROOT"
    rm -rf "$CACHE_ROOT"
fi

mkdir -p "$CACHE_ROOT"/{cargo,target,node_modules,npm-cache}

if (( REBUILD_IMAGE )) || ! docker image inspect "$IMAGE_TAG" >/dev/null 2>&1; then
    echo "==> Building Docker image $IMAGE_TAG (linux/amd64)"
    docker buildx build \
        --platform=linux/amd64 \
        --load \
        --tag "$IMAGE_TAG" \
        --file "$DOCKERFILE" \
        "$SCRIPT_DIR"
fi

echo "==> Building controller AppImage in linux/amd64 container"
echo "    repo:  $REPO_ROOT"
echo "    cache: $CACHE_ROOT"

# Plain bind mounts. No --privileged, no --security-opt, no named
# volume — none of the flatpak workarounds are needed because no
# kernel-level sandboxing happens inside.
docker run --rm \
    --platform=linux/amd64 \
    --volume "$REPO_ROOT:/work" \
    --volume "$CACHE_ROOT:/build" \
    --env REPO_ROOT=/work \
    --env BUILD_DIR=/build \
    "$IMAGE_TAG"
