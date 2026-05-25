#!/usr/bin/env bash
#
# Mac/local-host driver for the controller flatpak build.
#
# Builds the SAINT Controller .flatpak inside a linux/amd64 Docker
# container so the resulting bundle installs cleanly on the Steam Deck
# regardless of the host machine's architecture. On Apple Silicon, the
# amd64 emulation runs through Rosetta if Docker Desktop has it enabled
# (Settings → General → "Use Rosetta for x86/amd64 emulation"), which
# is significantly faster than QEMU for the cargo-heavy parts.
#
# Caches live under ~/.cache/saint-os/controller-flatpak/ (override
# with SAINT_CONTROLLER_CACHE). First clean build pulls the GNOME
# runtime (~1 GB), cargo registry, and npm tarballs into the cache;
# subsequent builds reuse them and finish in minutes.
#
# Usage:
#   controller/flatpak/build-docker.sh                 # incremental build
#   controller/flatpak/build-docker.sh --rebuild-image # re-run the Dockerfile
#   controller/flatpak/build-docker.sh --clean         # wipe the persistent cache
#   SAINT_CONTROLLER_CACHE=/path build-docker.sh       # override cache root
#
# Output:
#   server/resources/firmware/controller/saint_firmware_controller_<version>-local.<sha>.flatpak
#   server/resources/firmware/controller/info.json

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
CACHE_ROOT="${SAINT_CONTROLLER_CACHE:-${HOME}/.cache/saint-os/controller-flatpak}"

IMAGE_TAG="saint-controller-flatpak-builder:ubuntu24"
DOCKERFILE="$SCRIPT_DIR/Dockerfile"

REBUILD_IMAGE=0
DO_CLEAN=0

usage() {
    sed -n '2,28p' "$0" | sed 's/^# \{0,1\}//'
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
    echo "==> --clean: wiping $CACHE_ROOT/persistent (keeps flatpak runtime cache)"
    rm -rf "$CACHE_ROOT/persistent" "$CACHE_ROOT/builder-state"
fi

mkdir -p \
    "$CACHE_ROOT/flatpak" \
    "$CACHE_ROOT/builder-state" \
    "$CACHE_ROOT/persistent/cargo" \
    "$CACHE_ROOT/persistent/target" \
    "$CACHE_ROOT/persistent/node_modules" \
    "$CACHE_ROOT/persistent/npm-cache"

if (( REBUILD_IMAGE )) || ! docker image inspect "$IMAGE_TAG" >/dev/null 2>&1; then
    echo "==> Building Docker image $IMAGE_TAG (linux/amd64)"
    docker buildx build \
        --platform=linux/amd64 \
        --load \
        --tag "$IMAGE_TAG" \
        --file "$DOCKERFILE" \
        "$SCRIPT_DIR"
fi

echo "==> Building controller flatpak in linux/amd64 container"
echo "    repo:   $REPO_ROOT"
echo "    cache:  $CACHE_ROOT"

# --privileged enables bubblewrap (flatpak-builder's internal sandbox).
# Granular caps would be --cap-add=SYS_ADMIN + --security-opt
# apparmor=unconfined, but --privileged is portable across Docker
# hosts (Docker Desktop, Colima, podman-machine, etc.) and the
# container is short-lived + throwaway.
docker run --rm \
    --platform=linux/amd64 \
    --privileged \
    --volume "$REPO_ROOT:/work" \
    --volume "$CACHE_ROOT/flatpak:/var/lib/flatpak" \
    --volume "$CACHE_ROOT/builder-state:/build/state" \
    --volume "$CACHE_ROOT/persistent:/build/persistent" \
    --env PERSISTENT_CACHE=/build/persistent \
    --env REPO_ROOT=/work \
    --env BUILD_DIR=/build \
    "$IMAGE_TAG"
