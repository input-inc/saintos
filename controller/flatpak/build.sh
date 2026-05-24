#!/usr/bin/env bash
#
# Build the SAINT Controller Flatpak and install it for the current user.
#
# Builds are INCREMENTAL by default. The per-module build dir (which
# holds cargo's target/, node_modules, CARGO_HOME, npm cache) survives
# between runs via flatpak-builder's --keep-build-dirs flag, so a
# no-change rebuild is seconds and a small-edit rebuild is tens of
# seconds rather than the multi-minute full rebuild this used to be.
#
# Usage:
#   controller/flatpak/build.sh           # incremental build + install
#   controller/flatpak/build.sh --bundle  # also produce a portable .flatpak
#   controller/flatpak/build.sh --prereqs # install runtime/SDK deps first
#   controller/flatpak/build.sh --clean   # wipe per-module build dirs first
#                                         # (full cargo + npm rebuild, but
#                                         #  KEEPS source downloads + ccache)
#   controller/flatpak/build.sh --nuke    # wipe the entire cache root
#                                         # (re-downloads everything; last
#                                         #  resort for corrupted state)
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
SOURCE_MANIFEST="$SCRIPT_DIR/com.saintos.Controller.yml"
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
#   persistent/    Per-user host-side cache exposed to the build
#                  sandbox via --filesystem. Holds cargo registry +
#                  target/, node_modules, and the npm package cache —
#                  the things that need to survive between runs to
#                  make builds fast AND offline-capable. Without
#                  this, every invocation of flatpak-builder gets a
#                  fresh saint-controller-N build dir and every
#                  cache inside it starts empty.
CACHE_ROOT="$CONTROLLER_DIR/.flatpak-cache"
BUILD_DIR="$CACHE_ROOT/build"
REPO_DIR="$CACHE_ROOT/repo"
STATE_DIR="$CACHE_ROOT/state"
CCACHE_DIR="$CACHE_ROOT/ccache"
TMPDIR_LOCAL="$CACHE_ROOT/tmp"
PERSISTENT_CACHE="$CACHE_ROOT/persistent"

# The generated manifest is the source manifest with the per-user
# PERSISTENT_CACHE path substituted in. flatpak-builder doesn't expand
# env vars in YAML, so the path has to be a literal; we keep the
# committed manifest portable by templating it and emitting a
# .gitignored sibling at build time. It lives next to the source so
# `path: ../..` resolves the same way.
MANIFEST="$SCRIPT_DIR/.generated.com.saintos.Controller.yml"

# Make all the cache dirs eagerly so subprocesses don't fail on
# ENOENT before we get a chance to write anything. Persistent subdirs
# match the env vars + symlink targets in the manifest.
mkdir -p "$BUILD_DIR" "$REPO_DIR" "$STATE_DIR" "$CCACHE_DIR" "$TMPDIR_LOCAL" \
         "$PERSISTENT_CACHE/cargo" \
         "$PERSISTENT_CACHE/target" \
         "$PERSISTENT_CACHE/node_modules" \
         "$PERSISTENT_CACHE/npm-cache"

# Render the generated manifest fresh each run so the substituted path
# tracks whatever CACHE_ROOT resolves to (e.g. if the script moves).
sed "s|@PERSISTENT_CACHE@|$PERSISTENT_CACHE|g" "$SOURCE_MANIFEST" > "$MANIFEST"

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
        --filesystem="$PERSISTENT_CACHE"
        --filesystem="$CONTROLLER_DIR"
        --filesystem="$(cd "$CONTROLLER_DIR/.." && pwd)"   # repo root for the source dir
        --env=TMPDIR="$TMPDIR"
        --env=CCACHE_DIR="$CCACHE_DIR"
        org.flatpak.Builder
    )
fi

# Common builder args shared between the install and bundle paths.
#
# --state-dir replaces flatpak-builder's default
#   (~/.local/share/flatpak-builder/) which is fine on most hosts but
#   we want a single tree we can wipe with one `rm -rf` if things go
#   sideways.
# --ccache enables the C/C++ compile cache; its path is set via
#   the CCACHE_DIR env var (exported above, forwarded into the sandbox).
# --keep-build-dirs preserves per-module build dirs under
#   $STATE_DIR/build/ between invocations. THIS is what makes builds
#   incremental — without it, cargo's target/ and node_modules get
#   wiped after every successful run and the next build starts cold.
#   The manifest's `skip` list ensures the re-extracted source tree
#   doesn't overwrite those preserved artifacts. To force a cold
#   rebuild, use `--clean` (wipes per-module dirs) or `--nuke`
#   (wipes everything including the source download cache).
# --force-clean wipes the positional $BUILD_DIR (the staging output
#   passed to flatpak-builder, not the per-module build dir). Without
#   this, flatpak-builder refuses to run when $BUILD_DIR exists from
#   a prior invocation. Re-creating the staging tree is cheap; the
#   incremental work happens in $STATE_DIR/build/ where target/ lives.
BUILDER_COMMON=(
    --user
    --force-clean
    --install-deps-from=flathub
    --state-dir="$STATE_DIR"
    --ccache
    # No --keep-build-dirs: it preserves per-module build dirs after
    # the build but doesn't make flatpak-builder REUSE them on the
    # next invocation — each run still allocates a fresh
    # saint-controller-N (-1, -2, -3, ...), so the only thing
    # --keep-build-dirs accomplished was accumulating GB of stale
    # dirs on disk. Incremental caching is now handled by the
    # host-mounted persistent cache (see PERSISTENT_CACHE above plus
    # --filesystem in the generated manifest).
)

clean_build_dirs() {
    # Wipe compile artifacts so cargo + npm rebuild from scratch, but
    # KEEP downloaded sources (cargo registry, npm package tarballs)
    # so the rebuild can run offline. Use this when you've changed
    # toolchains, cargo flags, or suspect a stale binary.
    echo "==> --clean: wiping compile artifacts (full rebuild, keeps downloads)"
    echo "    removing per-module build dirs: $STATE_DIR/build"
    echo "    removing staging output:        $BUILD_DIR"
    echo "    removing cargo target:          $PERSISTENT_CACHE/target"
    echo "    removing node_modules:          $PERSISTENT_CACHE/node_modules"
    echo "    KEEPING cargo registry:         $PERSISTENT_CACHE/cargo"
    echo "    KEEPING npm package cache:      $PERSISTENT_CACHE/npm-cache"
    rm -rf "$STATE_DIR/build" "$BUILD_DIR" \
           "$PERSISTENT_CACHE/target" "$PERSISTENT_CACHE/node_modules"
    mkdir -p "$BUILD_DIR" \
             "$PERSISTENT_CACHE/target" "$PERSISTENT_CACHE/node_modules"
}

nuke_cache() {
    # Wipe EVERYTHING including downloaded sources. The next build
    # needs the network to re-fetch the cargo registry + npm
    # tarballs + flatpak runtime extensions + ccache. Last resort —
    # only use when --clean isn't enough.
    echo "==> --nuke: wiping entire flatpak cache root"
    echo "    removing: $CACHE_ROOT"
    rm -rf "$CACHE_ROOT"
    mkdir -p "$BUILD_DIR" "$REPO_DIR" "$STATE_DIR" "$CCACHE_DIR" "$TMPDIR_LOCAL" \
             "$PERSISTENT_CACHE/cargo" \
             "$PERSISTENT_CACHE/target" \
             "$PERSISTENT_CACHE/node_modules" \
             "$PERSISTENT_CACHE/npm-cache"
}

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

    # Auto-set Steam library artwork. Runs on the HOST shell (not in
    # the Flatpak sandbox) so it sees ~/.local/share/Steam/ without
    # needing finish-args permissions. Exits 1 with a friendly
    # message if the operator hasn't added the controller as a
    # Non-Steam Game yet — we treat that as a soft warning rather
    # than a build failure, since the install ITSELF was fine.
    echo
    echo "==> Configuring Steam library artwork (post-install hook)"
    if command -v python3 >/dev/null 2>&1; then
        if python3 "$SCRIPT_DIR/../scripts/set-steamdeck-artwork.py"; then
            :  # success message already printed by the script
        else
            echo "    (Artwork not installed yet. Add the controller as a"
            echo "     Non-Steam Game in Steam, then re-run:"
            echo "       flatpak run --command=saint-controller-artwork-setup $APP_ID)"
        fi
    else
        echo "    python3 not found on host — skipping. Re-run after"
        echo "    adding the controller to Steam:"
        echo "      flatpak run --command=saint-controller-artwork-setup $APP_ID"
    fi
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

usage() {
    cat <<EOF
Usage: $(basename "$0") [OPTIONS]

Build + install the SAINT Controller Flatpak. Incremental by default.

Options:
  --bundle    Also produce a portable .flatpak file after install.
  --prereqs   Install required Flatpak runtimes/SDK extensions, then exit.
  --clean     Wipe per-module build dirs before building. Forces a full
              cargo + npm rebuild but keeps source downloads + ccache.
              Use when you've changed Cargo flags / toolchain / suspect
              a stale artifact.
  --nuke      Wipe the entire cache root before building. Re-downloads
              everything. Use only when --clean isn't enough.
  -h, --help  Show this help and exit.

Flags can be combined, e.g. \`$(basename "$0") --clean --bundle\`.
EOF
}

# Multi-flag parser. Order-independent. --prereqs short-circuits to
# just prereq install. --nuke / --clean run before the build phase.
DO_BUNDLE=0
DO_PREREQS=0
DO_CLEAN=0
DO_NUKE=0

for arg in "$@"; do
    case "$arg" in
        --bundle)       DO_BUNDLE=1 ;;
        --prereqs)      DO_PREREQS=1 ;;
        --clean)        DO_CLEAN=1 ;;
        --nuke|--reset) DO_NUKE=1 ;;
        -h|--help)      usage; exit 0 ;;
        *)
            echo "Unknown option: $arg" >&2
            usage >&2
            exit 2
            ;;
    esac
done

if [ "$DO_PREREQS" = 1 ]; then
    prereqs
    exit 0
fi

if [ "$DO_NUKE" = 1 ]; then
    nuke_cache
elif [ "$DO_CLEAN" = 1 ]; then
    clean_build_dirs
fi

build_and_install

if [ "$DO_BUNDLE" = 1 ]; then
    bundle
fi
