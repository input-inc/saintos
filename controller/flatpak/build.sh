#!/usr/bin/env bash
#
# Build the SAINT Controller Flatpak and install it for the current user.
#
# Usage:
#   controller/flatpak/build.sh           # build + install (default)
#   controller/flatpak/build.sh --bundle  # also produce a portable .flatpak file
#   controller/flatpak/build.sh --prereqs # install runtime/SDK deps first
#
# Run this from anywhere; the script chdirs to controller/ so the
# manifest's `path: ../..` resolves correctly.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CONTROLLER_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
MANIFEST="$SCRIPT_DIR/com.saintos.Controller.yml"
APP_ID="com.saintos.Controller"
BUILD_DIR="$CONTROLLER_DIR/flatpak-build"
REPO_DIR="$CONTROLLER_DIR/flatpak-repo"

prereqs() {
    # Run once per host. Flathub is already on a stock Steam Deck;
    # the install commands are no-ops if the runtimes are already
    # present (Flatpak skips already-installed refs).
    echo "==> Ensuring Flathub remote is registered"
    flatpak remote-add --user --if-not-exists \
        flathub https://dl.flathub.org/repo/flathub.flatpakrepo

    echo "==> Installing required runtimes + SDK extensions"
    flatpak install --user --noninteractive --or-update flathub \
        org.flatpak.Builder \
        org.gnome.Sdk//47 \
        org.gnome.Platform//47 \
        org.freedesktop.Sdk.Extension.rust-stable//24.08 \
        org.freedesktop.Sdk.Extension.node20//24.08
}

build_and_install() {
    echo "==> Building + installing $APP_ID"
    cd "$CONTROLLER_DIR"
    flatpak-builder \
        --user \
        --install \
        --force-clean \
        --install-deps-from=flathub \
        "$BUILD_DIR" \
        "$MANIFEST"

    echo
    echo "Installed. Launch with:"
    echo "  flatpak run $APP_ID"
    echo
    echo "Or add it to Steam (Desktop Mode):"
    echo "  Steam → Games → Add a Non-Steam Game → Browse → /var/lib/flatpak/exports/bin/$APP_ID"
}

bundle() {
    echo "==> Building $APP_ID into a portable .flatpak"
    cd "$CONTROLLER_DIR"
    flatpak-builder \
        --user \
        --repo="$REPO_DIR" \
        --force-clean \
        --install-deps-from=flathub \
        "$BUILD_DIR" \
        "$MANIFEST"

    local out="$CONTROLLER_DIR/SAINT-Controller.flatpak"
    flatpak build-bundle "$REPO_DIR" "$out" "$APP_ID"
    echo
    echo "Bundle written to: $out"
    echo "Install on another machine with:"
    echo "  flatpak install --user $out"
}

case "${1:-}" in
    --prereqs)
        prereqs
        ;;
    --bundle)
        build_and_install
        bundle
        ;;
    "")
        build_and_install
        ;;
    *)
        echo "Unknown option: $1" >&2
        echo "Usage: $0 [--prereqs|--bundle]" >&2
        exit 2
        ;;
esac
