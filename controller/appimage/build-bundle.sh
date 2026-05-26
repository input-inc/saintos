#!/usr/bin/env bash
#
# Headless SAINT Controller AppImage builder.
#
# Produces a .AppImage from the Tauri project, stages it into the
# server's firmware resources tree, and regenerates info.json.
# Designed to be invoked from two places without modification:
#
#   1. Inside the linux/amd64 Docker container built from
#      controller/appimage/Dockerfile, driven by build-docker.sh on a
#      developer machine (Apple Silicon Mac via Rosetta, Linux native,
#      whatever).
#   2. Directly on a GitHub Linux runner (amd64 native, no Docker),
#      driven by the appimage-controller job in .github/workflows/dist.yml.
#
# Everything is parameterized via env vars so the same script handles
# both. Defaults assume the Docker layout; CI overrides.
#
#   REPO_ROOT     repo checkout root              (default /work)
#   BUILD_DIR     persistent cache root           (default /build)
#                 holds cargo registry, target/, node_modules, npm cache
#
# Output:
#   $REPO_ROOT/server/resources/firmware/controller/saint_firmware_controller_<v>-local.<sha>.AppImage
#   $REPO_ROOT/server/resources/firmware/controller/info.json
#
# This replaces controller/flatpak/build-bundle.sh; see
# controller/docs/APPIMAGE_MIGRATION.md for the migration rationale.

set -euo pipefail

REPO_ROOT="${REPO_ROOT:-/work}"
BUILD_DIR="${BUILD_DIR:-/build}"
CONTROLLER_DIR="$REPO_ROOT/controller"

mkdir -p "$BUILD_DIR"/{cargo,target,node_modules,npm-cache}

# Anchor cargo + npm state in the persistent cache so iterations are
# fast. The Mac driver bind-mounts $BUILD_DIR to
# ~/.cache/saint-os/controller-appimage/; CI uses an action cache for
# the same paths.
export CARGO_HOME="$BUILD_DIR/cargo"
export CARGO_TARGET_DIR="$BUILD_DIR/target"
export npm_config_cache="$BUILD_DIR/npm-cache"

# Suppress interactive prompts from tools that don't realize they're
# running headless without a tty (Angular CLI's analytics consent in
# particular crashes with `force closed the prompt with 0 null`
# without these).
export CI=true
export NG_CLI_ANALYTICS=false

# linuxdeploy and its plugins are themselves AppImages, which mount
# via FUSE. Docker containers don't expose /dev/fuse by default, and
# enabling it would require --device + CAP_SYS_ADMIN (sliding back
# toward the flatpak --privileged territory we just escaped). This
# env var tells AppImage-based tools to extract themselves to a tmp
# dir and run from there instead. Standard pattern in CI envs
# without FUSE (GitHub Actions runners do the same).
export APPIMAGE_EXTRACT_AND_RUN=1

cd "$CONTROLLER_DIR"

# node_modules in the persistent cache, symlinked from the source dir
# so npm/tauri see it at the conventional location. The source dir is
# bind-mounted, so this symlink survives between runs. On the Mac
# side it appears as a broken link when the container isn't running —
# harmless; `npm install` from the Mac shell would just rewrite it
# as a regular dir if the developer ever runs it directly.
if [ ! -L node_modules ]; then
    rm -rf node_modules
    ln -s "$BUILD_DIR/node_modules" node_modules
fi

# npm ci is strict about package-lock.json matching package.json.
# When they're out of sync (someone edited package.json without
# regenerating the lockfile), fall back to npm install so this build
# still goes through, and surface a LOUD warning pointing at the fix.
if [ -f node_modules/.package-lock.json ] \
    && cmp -s package-lock.json node_modules/.package-lock.json; then
    echo "==> node_modules matches package-lock.json — skipping npm ci"
else
    if ! npm ci --no-audit --no-fund --prefer-offline; then
        echo
        echo "================================================================"
        echo "WARN  npm ci failed — package.json and package-lock.json are out"
        echo "      of sync. Falling back to 'npm install' so this build can"
        echo "      proceed."
        echo
        echo "      Permanent fix (run from the host, NOT inside the container):"
        echo "          cd controller && npm install"
        echo "          git add package-lock.json && git commit"
        echo "================================================================"
        echo
        npm install --no-audit --no-fund --prefer-offline
    fi
fi

# Compute the canonical SAINT version string early so we can export
# it to the Tauri build. build.rs in src-tauri reads SAINT_BUILD_VERSION
# and bakes it into the binary as a compile-time constant — that's
# what the runtime About + Software Update screens display. The same
# string is also used to name the .AppImage further down, so the
# embedded version and the on-disk filename can never disagree.
#
# Honor an inbound override (e.g. CI passing a tag-derived value);
# otherwise reconstruct the local-channel form from VERSION + git sha.
if [ -z "${SAINT_BUILD_VERSION:-}" ]; then
    _base_version=$(tr -d '[:space:]' < "$CONTROLLER_DIR/VERSION")
    _git_sha=$(cd "$REPO_ROOT" && git rev-parse --short=7 HEAD 2>/dev/null || echo unknown)
    export SAINT_BUILD_VERSION="${_base_version}-local.${_git_sha}"
fi
echo "==> Build version: $SAINT_BUILD_VERSION"

echo "==> Running tauri build --bundles appimage"
# We treat tauri's own bundling step as best-effort: it produces the
# AppDir we want (binary + icon + .desktop + Steam library art under
# bundle.resources), but its linuxdeploy invocation either:
#   - fails on Apple Silicon Docker Desktop (Rosetta refuses to exec
#     the AppImage tools because of their non-standard ABI byte), or
#   - succeeds on native Linux CI but produces an AppImage that's
#     missing webkit2gtk's helper binaries (linuxdeploy only walks
#     ldd; webkit spawns its helpers via execve from a path that ldd
#     can't see), which crashes on stock SteamOS the moment any HTTP
#     request fires.
# So we ignore tauri's exit code, take the AppDir, and bundle it
# ourselves using our own linuxdeploy + appimagetool with the
# webkit-helper bundling and LD_PRELOAD path-shim layered on top.
set +e
npx tauri build --bundles appimage
set -e

APPDIR=$(find "$CARGO_TARGET_DIR/release/bundle/appimage" \
    -maxdepth 1 -name '*.AppDir' -print -quit 2>/dev/null || true)
if [ -z "$APPDIR" ]; then
    echo "ERROR: tauri did not produce an AppDir" >&2
    exit 1
fi
if ! command -v linuxdeploy >/dev/null \
    || ! command -v appimagetool >/dev/null; then
    echo "ERROR: linuxdeploy + appimagetool not on PATH" >&2
    echo "       (Docker image bakes these in; CI installs them as a separate step)" >&2
    exit 1
fi

# --- Post-process the AppDir into a Deck-compatible AppImage --------
#
# 1. Bundle webkit2gtk's helper binaries. linuxdeploy walks `ldd` on
#    the main binary to find shared library deps and copies them;
#    that catches libwebkit2gtk-4.1.so.0 itself. What it MISSES are
#    webkit's split-process helper binaries (WebKitNetworkProcess,
#    WebKitWebProcess, WebKitGPUProcess, MiniBrowser, plus the
#    injected-bundle dir), which webkit spawns via execve from inside
#    its own library at the hardcoded build-time path
#    /usr/lib/x86_64-linux-gnu/webkit2gtk-4.1/. ldd can't see them.
#    Stock SteamOS doesn't have webkit2gtk-4.1 installed at all, so
#    that path is empty and the spawn fails. We copy the helpers
#    into the AppDir at the same relative path.
WEBKIT_LIBDIR=/usr/lib/x86_64-linux-gnu/webkit2gtk-4.1
if [ -d "$WEBKIT_LIBDIR" ]; then
    echo "==> Bundling webkit2gtk helper processes from $WEBKIT_LIBDIR"
    mkdir -p "$APPDIR$WEBKIT_LIBDIR"
    cp -a "$WEBKIT_LIBDIR/." "$APPDIR$WEBKIT_LIBDIR/"
else
    echo "WARN: $WEBKIT_LIBDIR not found; webkit will fail to spawn helpers" >&2
fi

# 2. Run linuxdeploy on the AppDir to populate /usr/lib with the
#    `ldd`-discovered .so files. No --output: we'll seal with
#    appimagetool after replacing AppRun. (Idempotent if libs are
#    already present, e.g. on a re-run.)
pushd "$CARGO_TARGET_DIR/release/bundle/appimage" >/dev/null
linuxdeploy --appdir "$APPDIR"

# 3. Install the LD_PRELOAD path-mapping shim that intercepts
#    execve / posix_spawn / open / stat calls and rewrites
#    /usr/lib/x86_64-linux-gnu/webkit2gtk-4.1/* (the path baked into
#    libwebkit2gtk-4.1.so.0 at distro build time) to the AppImage-
#    bundled equivalent. Webkit's WEBKIT_EXEC_PATH env-var override
#    is gated behind ENABLE(DEVELOPER_MODE) which distro builds turn
#    off, so an LD_PRELOAD shim is the only knob left.
SHIM_SRC="$CONTROLLER_DIR/appimage/path-shim.c"
SHIM_SO=""
if [ -f /opt/saint-shim/libpath-shim.so ]; then
    SHIM_SO=/opt/saint-shim/libpath-shim.so
else
    SHIM_SO="$BUILD_DIR/libpath-shim.so"
    if [ ! -f "$SHIM_SO" ] || [ "$SHIM_SRC" -nt "$SHIM_SO" ]; then
        echo "==> Compiling LD_PRELOAD path-shim"
        gcc -shared -fPIC -O2 -Wall -o "$SHIM_SO" "$SHIM_SRC" -ldl
    fi
fi
install -Dm644 "$SHIM_SO" "$APPDIR/usr/lib/libpath-shim.so"

# 4. Replace linuxdeploy's default AppRun (a compiled C binary that
#    doesn't know about LD_PRELOAD or webkit) with a shell wrapper.
cat > "$APPDIR/AppRun" <<'APPRUN'
#!/bin/bash
HERE="$(dirname "$(readlink -f "${0}")")"
export APPDIR="$HERE"
export LD_LIBRARY_PATH="$APPDIR/usr/lib:$APPDIR/usr/lib/x86_64-linux-gnu${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
export PATH="$APPDIR/usr/bin:$PATH"
# Intercepts execve/posix_spawn/open/stat calls inside libwebkit2gtk
# and rewrites /usr/lib/x86_64-linux-gnu/webkit2gtk-4.1/* references
# to point inside this AppImage's mount. Without this, webkit fails
# to spawn its WebKitNetworkProcess helper on systems that lack the
# Debian-multiarch path (Steam Deck / Arch). See path-shim.c.
export LD_PRELOAD="$APPDIR/usr/lib/libpath-shim.so${LD_PRELOAD:+:$LD_PRELOAD}"
export GIO_MODULE_DIR="$APPDIR/usr/lib/x86_64-linux-gnu/gio/modules"
exec "$APPDIR/usr/bin/saint-controller" "$@"
APPRUN
chmod +x "$APPDIR/AppRun"

# 5. Seal the modified AppDir into the final AppImage. Delete any
#    .AppImage tauri's own linuxdeploy may have created earlier (on
#    native CI runners) so the stage step below picks up ours.
find . -maxdepth 1 -name '*.AppImage' -delete
appimagetool "$APPDIR" SAINT_Controller-x86_64.AppImage
popd >/dev/null

# --- stage into server/resources/firmware/controller/ ---------------
#
# Match the saint_firmware_<type>_<version>-local.<sha>.<ext> naming
# established by firmware/rpi5/scripts/package.sh + the on-Deck
# build.sh's stage_to_server helper. The server's /api/firmware*
# endpoints just read info.json, so as long as the shape matches
# server/resources/firmware/rpi5/info.json, nothing on the server
# needs to know this happened.

# Reuse the same version string the binary baked in at compile time
# (see the SAINT_BUILD_VERSION export near the top of this script).
# That guarantees the AppImage filename and the embedded version
# match — which is the whole reason this got centralized.
VERSION="$SAINT_BUILD_VERSION"
FILENAME="saint_firmware_controller_${VERSION}.AppImage"
DEST_DIR="$REPO_ROOT/server/resources/firmware/controller"

# Tauri writes the AppImage with a version-stamped name. Glob for it
# rather than hardcode the version, since BASE_VERSION here is the
# repo VERSION file but Tauri uses src-tauri/Cargo.toml's version
# (kept in sync by controller/scripts/sync-version.js but worth not
# depending on that being perfect).
APPIMAGE_SRC=$(find "$CARGO_TARGET_DIR/release/bundle/appimage" \
    -maxdepth 1 -name '*.AppImage' -print -quit 2>/dev/null || true)
if [ -z "$APPIMAGE_SRC" ]; then
    echo "ERROR: tauri build did not produce an .AppImage in" >&2
    echo "       $CARGO_TARGET_DIR/release/bundle/appimage/" >&2
    exit 1
fi

mkdir -p "$DEST_DIR"
# Prune any prior .AppImage from the staging dir so the dist tarball
# ships exactly one — the most recently built. Two reasons:
#   - The dist tarball would otherwise grow ~80MB per stale build.
#   - The server's firmware-listing endpoint picks `latest_package`
#     from info.json, but any stray .AppImage left here would still
#     be servable via /api/firmware/controller/<filename> and confuse
#     the OTA flow if an operator pointed at it manually.
# Other firmware staging dirs (rp2040/teensy41/rpi5) keep their
# version-stamped artifacts on purpose — that's their convention.
find "$DEST_DIR" -maxdepth 1 -type f -name '*.AppImage' -delete
cp "$APPIMAGE_SRC" "$DEST_DIR/$FILENAME"

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
