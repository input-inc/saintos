#!/usr/bin/env bash
#
# Headless SAINT Controller flatpak builder.
#
# Produces a .flatpak bundle from the manifest, stages it into the
# server's firmware resources tree, and regenerates info.json. Designed
# to be invoked from two places without modification:
#
#   1. Inside the linux/amd64 Docker container built from
#      controller/flatpak/Dockerfile, driven by build-docker.sh on a
#      developer machine (Apple Silicon Mac via Rosetta, Linux native,
#      whatever).
#   2. Directly on a GitHub Linux runner (amd64 native, no Docker),
#      driven by the flatpak-controller job in .github/workflows/dist.yml.
#
# Everything is parameterized via env vars so the same script handles
# both. Defaults assume the Docker layout; CI overrides.
#
#   REPO_ROOT             repo checkout root              (default /work)
#   BUILD_DIR             scratch dir for flatpak-builder (default /build)
#   PERSISTENT_CACHE      cargo + npm + node_modules dir  (default $BUILD_DIR/persistent)
#   FLATPAK_INSTALL_SCOPE --system or --user              (default --system)
#
# Output:
#   $REPO_ROOT/server/resources/firmware/controller/saint_firmware_controller_<version>-local.<sha>.flatpak
#   $REPO_ROOT/server/resources/firmware/controller/info.json
#
# This is the cross-host counterpart to controller/flatpak/build.sh
# (which is the on-Deck path — assumes a real desktop, installs to the
# user's flatpak, runs Steam artwork hooks). Both produce the same
# artifact when run with --bundle, just from different hosts.

set -euo pipefail

REPO_ROOT="${REPO_ROOT:-/work}"
BUILD_DIR="${BUILD_DIR:-/build}"
PERSISTENT_CACHE="${PERSISTENT_CACHE:-$BUILD_DIR/persistent}"
FLATPAK_INSTALL_SCOPE="${FLATPAK_INSTALL_SCOPE:---system}"

CONTROLLER_DIR="$REPO_ROOT/controller"
SOURCE_MANIFEST="$CONTROLLER_DIR/flatpak/com.saintos.Controller.yml"
GENERATED_MANIFEST="$BUILD_DIR/com.saintos.Controller.generated.yml"

mkdir -p \
    "$BUILD_DIR" \
    "$PERSISTENT_CACHE"/{cargo,target,node_modules,npm-cache}

# Materialize the manifest with the cache path baked in. The on-Deck
# build.sh does the same substitution against its own path; we mirror
# that so the manifest stays portable (no committed absolute paths).
sed "s|@PERSISTENT_CACHE@|$PERSISTENT_CACHE|g" "$SOURCE_MANIFEST" > "$GENERATED_MANIFEST"

echo "==> Configuring Flathub remote ($FLATPAK_INSTALL_SCOPE)"
flatpak $FLATPAK_INSTALL_SCOPE remote-add --if-not-exists \
    flathub https://dl.flathub.org/repo/flathub.flatpakrepo

echo "==> Installing runtimes (cached from previous runs if present)"
# Versions tracked in lockstep with controller/flatpak/build.sh's
# `prereqs` function. Bump together when the manifest's
# runtime-version changes.
flatpak $FLATPAK_INSTALL_SCOPE install -y --noninteractive flathub \
    org.gnome.Sdk//50 \
    org.gnome.Platform//50 \
    org.freedesktop.Sdk.Extension.rust-stable//25.08 \
    org.freedesktop.Sdk.Extension.node20//25.08

# --user vs --system on the builder side has to match the install
# scope chosen above — otherwise the runtimes flatpak-builder needs
# aren't visible to it.
BUILDER_SCOPE="${FLATPAK_INSTALL_SCOPE/--system/--user}"
if [ "$FLATPAK_INSTALL_SCOPE" = "--system" ]; then
    BUILDER_SCOPE=""  # flatpak-builder defaults to --system when run as root
fi

echo "==> Running flatpak-builder"
cd "$CONTROLLER_DIR"
# shellcheck disable=SC2086
flatpak-builder \
    $BUILDER_SCOPE \
    --force-clean \
    --install-deps-from=flathub \
    --state-dir="$BUILD_DIR/state" \
    --ccache \
    --repo="$BUILD_DIR/repo" \
    "$BUILD_DIR/output" \
    "$GENERATED_MANIFEST"

echo "==> Bundling into a portable .flatpak"
flatpak build-bundle \
    "$BUILD_DIR/repo" \
    "$BUILD_DIR/SAINT-Controller.flatpak" \
    com.saintos.Controller

# --- stage into server/resources/firmware/controller/ ----------------
#
# Match the saint_firmware_<type>_<version>-local.<sha>.<ext> naming
# established by firmware/rpi5/scripts/package.sh + the on-Deck
# build.sh's stage_to_server helper. The server's /api/firmware*
# endpoints just read info.json, so as long as the shape matches
# server/resources/firmware/rpi5/info.json, nothing on the server
# needs to know this happened.

VERSION_FILE="$CONTROLLER_DIR/VERSION"
BASE_VERSION=$(tr -d '[:space:]' < "$VERSION_FILE")
GIT_SHA=$(cd "$REPO_ROOT" && git rev-parse --short=7 HEAD 2>/dev/null || echo unknown)
VERSION="${BASE_VERSION}-local.${GIT_SHA}"
FILENAME="saint_firmware_controller_${VERSION}.flatpak"
DEST_DIR="$REPO_ROOT/server/resources/firmware/controller"

mkdir -p "$DEST_DIR"
cp "$BUILD_DIR/SAINT-Controller.flatpak" "$DEST_DIR/$FILENAME"

CHECKSUM=$(sha256sum "$DEST_DIR/$FILENAME" | awk '{print $1}')
SIZE=$(wc -c < "$DEST_DIR/$FILENAME" | tr -d ' ')
UPDATED=$(date -u +%Y-%m-%dT%H:%M:%SZ)

cat > "$DEST_DIR/info.json" <<EOF
{
    "type": "controller",
    "latest_version": "${VERSION}",
    "latest_package": "${FILENAME}",
    "latest_checksum": "${CHECKSUM}",
    "updated": "${UPDATED}",
    "packages": [
        {
            "version": "${VERSION}",
            "filename": "${FILENAME}",
            "checksum": "${CHECKSUM}",
            "size": ${SIZE}
        }
    ]
}
EOF

echo "==> Staged for OTA: $DEST_DIR/$FILENAME ($(du -h "$DEST_DIR/$FILENAME" | cut -f1))"
