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
    # A rebuild retags $IMAGE_TAG onto a fresh image id and orphans the
    # previous one as a dangling <none> image (~1.9GB each). Capture the
    # old id so we can drop it after a successful rebuild — otherwise
    # repeated --rebuild-image runs pile up multi-GB garbage.
    prev_image_id="$(docker image inspect "$IMAGE_TAG" --format '{{.Id}}' 2>/dev/null || true)"
    docker buildx build \
        --platform=linux/amd64 \
        --load \
        --tag "$IMAGE_TAG" \
        --file "$DOCKERFILE" \
        "$SCRIPT_DIR"
    new_image_id="$(docker image inspect "$IMAGE_TAG" --format '{{.Id}}' 2>/dev/null || true)"
    if [ -n "$prev_image_id" ] && [ "$prev_image_id" != "$new_image_id" ]; then
        echo "==> Removing superseded builder image ${prev_image_id#sha256:}"
        docker image rm "$prev_image_id" >/dev/null 2>&1 || true
    fi
fi

echo "==> Building controller AppImage in linux/amd64 container"
echo "    repo:  $REPO_ROOT"
echo "    cache: $CACHE_ROOT"

# controller/node_modules must be the container-side symlink into the
# cache (build-bundle.sh creates it). A host-side `npm install` for
# local dev turns it into a real directory; reclaim it HERE, on the
# host, before the container starts. Doing the rm inside the container
# is unreliable on Docker Desktop for Mac: rm -rf over the virtiofs
# bind mount races directory-entry caching (and Finder's .DS_Store
# drops) and dies with "Directory not empty".
NODE_MODULES="$REPO_ROOT/controller/node_modules"
if [ -e "$NODE_MODULES" ] && [ ! -L "$NODE_MODULES" ]; then
    echo "==> Removing host-installed controller/node_modules (container will re-link it into the cache)"
    echo "    (re-run 'npm install' in controller/ if you need host-side npm tooling again)"
    rm -rf "$NODE_MODULES"
fi

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
