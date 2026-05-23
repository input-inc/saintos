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
#
# All caches / temp files / state directories are pinned under
# ~/saintos/controller/.flatpak-cache/ rather than the system defaults
# (/var/tmp, /tmp, /var/lib/flatpak-builder/...) — on SteamOS the
# /var and / partitions are tiny (230 MB and 5 GB) while /home is
# roomy (~47 GB), so the system defaults blow up on the second or
# third build. By forcing everything into /home explicitly, we
# never touch the small partitions.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CONTROLLER_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
MANIFEST="$SCRIPT_DIR/com.saintos.Controller.yml"
APP_ID="com.saintos.Controller"

# All flatpak-builder working state and scratch space live under here.
# Subdirectories:
#   build/         per-module build dir (was BUILD_DIR)
#   repo/          OSTree repo for bundling (was REPO_DIR)
#   state/         flatpak-builder cache: downloaded sources, build
#                  state, ccache. THIS is the big one — once seeded
#                  it's ~5–10 GB of cargo registry + npm tarballs.
#   ccache/        compiler cache
#   tmp/           TMPDIR for every subprocess we launch (cargo, npm,
#                  ostree, tar). Without redirecting this, anything
#                  that writes to /tmp (RAM-backed tmpfs) or /var/tmp
#                  (tiny SteamOS partition) eats those out of space
#                  long before /home would notice.
CACHE_ROOT="$CONTROLLER_DIR/.flatpak-cache"
BUILD_DIR="$CACHE_ROOT/build"
REPO_DIR="$CACHE_ROOT/repo"
STATE_DIR="$CACHE_ROOT/state"
CCACHE_DIR="$CACHE_ROOT/ccache"
TMPDIR_LOCAL="$CACHE_ROOT/tmp"

# Make all the cache dirs eagerly so subprocesses don't fail on
# ENOENT before we get a chance to write anything.
mkdir -p "$BUILD_DIR" "$REPO_DIR" "$STATE_DIR" "$CCACHE_DIR" "$TMPDIR_LOCAL"

# Export TMPDIR for any subprocess that honors it (cargo's tarball
# unpacks, npm's package extracts, ostree's transaction journal,
# generic tar/gzip). When flatpak-builder runs sandboxed (the
# org.flatpak.Builder Flatpak) we ALSO forward this into its sandbox
# via --env below — without that the in-sandbox processes still see
# the host's default /tmp.
export TMPDIR="$TMPDIR_LOCAL"

# flatpak-builder's `--ccache` flag enables the compile cache but has
# no companion `--ccache-dir` flag — the cache path is set entirely
# via the CCACHE_DIR env var. Forward this into the sandboxed flatpak
# run too (see FLATPAK_BUILDER below).
export CCACHE_DIR="$CCACHE_DIR"

# Resolve which `flatpak-builder` to use. On most distros it's a host
# package; on SteamOS (read-only root) it's installed as the Flathub
# Flatpak `org.flatpak.Builder` and invoked via `flatpak run`. Use
# the host binary if it exists, otherwise fall through to the Flatpak.
# Built as an array so the call-site can expand cleanly with args.
#
# When using the sandboxed version, forward TMPDIR into the sandbox
# and grant it filesystem access to the cache dirs — otherwise
# /run/build/... inside the sandbox can't see our /home paths.
if command -v flatpak-builder >/dev/null 2>&1; then
    FLATPAK_BUILDER=(flatpak-builder)
else
    FLATPAK_BUILDER=(
        flatpak run
        --filesystem="$CACHE_ROOT"
        --filesystem="$CONTROLLER_DIR"
        --filesystem="$(cd "$CONTROLLER_DIR/.." && pwd)"   # repo root for the source dir
        --env=TMPDIR="$TMPDIR"
        --env=CCACHE_DIR="$CCACHE_DIR"
        org.flatpak.Builder
    )
fi

# Common builder args shared between the install and bundle paths.
# --state-dir replaces flatpak-builder's default
# (~/.local/share/flatpak-builder/) which is fine on most hosts but
# we want a single tree we can wipe with one `rm -rf` if things go
# sideways. --ccache enables the compile cache; its path is set via
# the CCACHE_DIR env var (exported above, forwarded into the sandbox).
BUILDER_COMMON=(
    --user
    --force-clean
    --install-deps-from=flathub
    --state-dir="$STATE_DIR"
    --ccache
)

prereqs() {
    # Run once per host. Flathub is already on a stock Steam Deck;
    # the install commands are no-ops if the runtimes are already
    # present (Flatpak skips already-installed refs).
    echo "==> Ensuring Flathub remote is registered"
    flatpak remote-add --user --if-not-exists \
        flathub https://dl.flathub.org/repo/flathub.flatpakrepo

    echo "==> Installing required runtimes + SDK extensions"
    # Versions match the manifest's runtime-version. Bumped together
    # — GNOME runtimes EOL ~12 months after release. Check current
    # with `flatpak remote-info flathub org.gnome.Platform`.
    flatpak install --user --noninteractive --or-update flathub \
        org.flatpak.Builder \
        org.gnome.Sdk//50 \
        org.gnome.Platform//50 \
        org.freedesktop.Sdk.Extension.rust-stable//25.08 \
        org.freedesktop.Sdk.Extension.node20//25.08
}

build_and_install() {
    echo "==> Building + installing $APP_ID"
    echo "    cache root:  $CACHE_ROOT"
    echo "    TMPDIR:      $TMPDIR"
    cd "$CONTROLLER_DIR"
    "${FLATPAK_BUILDER[@]}" \
        "${BUILDER_COMMON[@]}" \
        --install \
        "$BUILD_DIR" \
        "$MANIFEST"

    echo
    echo "Installed. Launch with:"
    echo "  flatpak run $APP_ID"
    echo
    echo "Or add it to Steam (Desktop Mode):"
    echo "  Steam → Games → Add a Non-Steam Game → Browse →"
    echo "  $HOME/.local/share/flatpak/exports/bin/$APP_ID"
    echo "  (per-user install — system installs would be under /var/lib/flatpak/exports/bin/)"
}

bundle() {
    echo "==> Building $APP_ID into a portable .flatpak"
    cd "$CONTROLLER_DIR"
    "${FLATPAK_BUILDER[@]}" \
        "${BUILDER_COMMON[@]}" \
        --repo="$REPO_DIR" \
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
