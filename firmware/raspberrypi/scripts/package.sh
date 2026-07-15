#!/bin/bash
#
# Package Pi 5 firmware for distribution
#
# Creates a zip file that can be served by the SAINT.OS server
# for over-the-air updates to Pi 5 nodes.
#
# Usage: ./package.sh [version]
#

set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
FIRMWARE_DIR="$(dirname "$SCRIPT_DIR")"
OUTPUT_DIR="${FIRMWARE_DIR}/dist"

# Target Debian release for the Pi node. Used to pick the matching
# ROS2 source-built tree (and, transitively, the matching Python ABI):
#   bookworm → _ros2/ (libpython3.11, built by scripts/build-local-dist.sh)
#   trixie   → _ros2_trixie/ (libpython3.13, built by
#              firmware/raspberrypi/scripts/build-ros2-trixie.sh)
# --multi-target bundles BOTH bookworm and trixie ROS trees + deb caches
# into a single zip; install.sh on the Pi detects /etc/os-release at
# install time and picks the right subdir. This is the recommended
# default when both _ros2/ (bookworm) and _ros2_trixie/ are present:
# one artifact ships to any Pi regardless of OS, and the OTA flow on
# the server doesn't need to know the node's Debian release to pick
# the right URL.
#
# --target-release <name> keeps the legacy single-target shape (one
# OS per zip). Useful when only one target's source build is present
# on the dev box.
TARGET_RELEASE="bookworm"
MULTI_TARGET=0

# Parse flags then collapse remaining positional args back. Old callers
# passed just a version string (`package.sh 1.2.3`); preserve that.
ARGS=()
while [ $# -gt 0 ]; do
    case "$1" in
        --target-release)
            TARGET_RELEASE="$2"
            shift 2
            ;;
        --target-release=*)
            TARGET_RELEASE="${1#*=}"
            shift
            ;;
        --multi-target)
            MULTI_TARGET=1
            shift
            ;;
        *)
            ARGS+=("$1")
            shift
            ;;
    esac
done
set -- "${ARGS[@]+"${ARGS[@]}"}"

# Get version from argument or __init__.py
if [ -n "${1:-}" ]; then
    VERSION="$1"
else
    VERSION=$(grep '__version__' "${FIRMWARE_DIR}/saint_node/__init__.py" | sed 's/.*"\([^"]*\)".*/\1/')
fi

if [ -z "$VERSION" ]; then
    VERSION="0.0.0"
fi

echo "Packaging Pi 5 firmware version ${VERSION}"

# Create output directory and clear any stale artifacts. Otherwise
# zips from older builds (different version strings, pre-rename
# rpi5_* names, single-target shapes from before --multi-target)
# pile up here and then get cp -r'd into server/resources/firmware/
# /raspberrypi/, which in turn gets stuffed into the server's dist
# tarball — ballooning the installer by hundreds of MB.
mkdir -p "${OUTPUT_DIR}"
# Remove only files this script produces — info.json + the
# saint_firmware_raspberrypi_* family (both legacy .zip and current
# .tar.zst) — so an unrelated artifact someone dropped here by hand
# doesn't disappear.
find "${OUTPUT_DIR}" -maxdepth 1 -type f \
    \( -name 'saint_firmware_raspberrypi_*.zip' \
       -o -name 'saint_firmware_raspberrypi_*.tar.zst' \
       -o -name 'saint_firmware_rpi5_*.zip' \
       -o -name 'info.json' \) \
    -delete

# Create temporary directory for packaging
TEMP_DIR=$(mktemp -d)
PACKAGE_DIR="${TEMP_DIR}/saint_firmware_raspberrypi_${VERSION}"
mkdir -p "${PACKAGE_DIR}"

# Copy firmware files
echo "Copying firmware files..."
cp -r "${FIRMWARE_DIR}/saint_node" "${PACKAGE_DIR}/"
cp -r "${FIRMWARE_DIR}/config" "${PACKAGE_DIR}/" 2>/dev/null || true
cp -r "${FIRMWARE_DIR}/scripts" "${PACKAGE_DIR}/" 2>/dev/null || true

# Bundle operator-facing docs so the zip is self-explanatory. Previously
# an operator who downloaded just the zip got install.sh and a guess —
# the actual install guide lived in the repo at
# firmware/raspberrypi/docs/INSTALL.md and was invisible to anyone not
# checking out the source. INSTALL.md is the comprehensive walkthrough;
# README.md is the short orientation card.
mkdir -p "${PACKAGE_DIR}/docs"
[ -f "${FIRMWARE_DIR}/docs/INSTALL.md" ] && cp "${FIRMWARE_DIR}/docs/INSTALL.md" "${PACKAGE_DIR}/docs/INSTALL.md"
[ -f "${FIRMWARE_DIR}/README.md" ]      && cp "${FIRMWARE_DIR}/README.md"      "${PACKAGE_DIR}/README.md"

# Plymouth boot-splash theme (firmware/raspberrypi/assets/plymouth/).
# install.sh copies this to /usr/share/plymouth/themes/saint-os/ and
# runs plymouth-set-default-theme. The theme script references
# opensaint.png as the logo — single source of truth is
# firmware/raspberrypi/assets/opensaint.png (the canonical brand
# asset). Copy it into the theme dir at package time so the same
# image powers both the Pi boot splash and the SPA splash.
if [ -d "${FIRMWARE_DIR}/assets/plymouth" ]; then
    mkdir -p "${PACKAGE_DIR}/assets"
    cp -a "${FIRMWARE_DIR}/assets/plymouth" "${PACKAGE_DIR}/assets/plymouth"
    if [ -f "${FIRMWARE_DIR}/assets/opensaint.png" ]; then
        cp "${FIRMWARE_DIR}/assets/opensaint.png" \
           "${PACKAGE_DIR}/assets/plymouth/saint-os/opensaint.png"
        echo "Bundled Plymouth theme with opensaint.png logo"
    else
        echo "WARNING: Plymouth theme bundled WITHOUT opensaint.png — install.sh will"
        echo "  skip the theme install. Drop a PNG at"
        echo "  firmware/raspberrypi/assets/opensaint.png and re-package."
    fi
fi

# Remove __pycache__ directories
find "${PACKAGE_DIR}" -type d -name "__pycache__" -exec rm -rf {} + 2>/dev/null || true
find "${PACKAGE_DIR}" -name "*.pyc" -delete 2>/dev/null || true

# ---------------------------------------------------------------------------
# Optional offline-install payload (matches packaging/install.sh's contract):
#
#   ros2_install/   -- bundled ROS2 install tree, extracted to
#                      /opt/ros/<distro>/install/ on the target. Pi nodes
#                      typically have no internet so we'd rather ship the
#                      whole tree than try to fetch it from packages.ros.org
#                      at install time.
#   deps/           -- .debs + Packages index for runtime apt deps (gpiod,
#                      vlc, alsa-utils, plus everything ROS2 dynamically
#                      links against). The Pi installer wires this up as a
#                      local apt repo so `apt-get install` resolves
#                      offline.
#
# Sources for these:
#   $SAINT_RPI_ROS2_DIR   -- explicit override (caller-provided path)
#   _ros2/opt/ros/<distro>/install   -- the server build's cached output
#                                       (scripts/build-local-dist.sh)
#   $SAINT_RPI_DEBS_DIR   -- explicit override
#   _rpi_debs   /   _debs  -- the server build's deb cache (Pi-specific
#                             cache preferred if present)
#
# Either or both are optional. Without them, the install bundle is smaller
# and the installer falls back to online apt + ros-apt-source.

REPO_ROOT="$(cd "${FIRMWARE_DIR}/../.." && pwd)"

# Resolve which ROS2 distro the install tree targets. Default kilted —
# matches the project_ros_distro_plan migration target.
ROS_DISTRO="${SAINT_ROS_DISTRO:-kilted}"

# Resolve the ROS2 install tree path for a given Debian release. Hides
# the _ros2/ (bookworm legacy) vs _ros2_<release>/ (per-release) layout
# behind one helper. Echoes the absolute path; empty if no tree found.
ros2_src_for_release() {
    local rel="$1"
    local override="${SAINT_RPI_ROS2_DIR:-}"
    if [ -n "$override" ] && [ -d "$override" ]; then
        echo "$override"; return 0
    fi
    local specific="${REPO_ROOT}/_ros2_${rel}/opt/ros/${ROS_DISTRO}/install"
    if [ -d "$specific" ]; then
        echo "$specific"; return 0
    fi
    if [ "$rel" = "bookworm" ] && [ -d "${REPO_ROOT}/_ros2/opt/ros/${ROS_DISTRO}/install" ]; then
        # The server-side build (scripts/build-local-dist.sh) writes the
        # bookworm tree to _ros2/. Treat it as the bookworm-specific
        # tree even though it doesn't carry the _bookworm suffix.
        echo "${REPO_ROOT}/_ros2/opt/ros/${ROS_DISTRO}/install"; return 0
    fi
    return 1
}

# Resolve the runtime deb cache dir for a given Debian release.
debs_src_for_release() {
    local rel="$1"
    local override="${SAINT_RPI_DEBS_DIR:-}"
    if [ -n "$override" ] && [ -d "$override" ] && ls "$override"/*.deb >/dev/null 2>&1; then
        echo "$override"; return 0
    fi
    local specific="${REPO_ROOT}/_rpi_debs_${rel}"
    if [ -d "$specific" ] && ls "$specific"/*.deb >/dev/null 2>&1; then
        echo "$specific"; return 0
    fi
    if [ "$rel" = "bookworm" ] && [ -d "${REPO_ROOT}/_rpi_debs" ] && ls "${REPO_ROOT}/_rpi_debs"/*.deb >/dev/null 2>&1; then
        # Legacy: pre-multi-release _rpi_debs/ was assumed bookworm.
        echo "${REPO_ROOT}/_rpi_debs"; return 0
    fi
    return 1
}

# bundle-debs.sh builds the per-release runtime deb cache. We invoke it
# on demand so packaging never silently ships a bundle without deps/.
BUNDLE_DEBS_SCRIPT="${SCRIPT_DIR}/bundle-debs.sh"
# Set AUTO_FETCH_DEBS=0 to opt out (e.g. deliberately building an
# online-install bundle on a host with no docker/network).
AUTO_FETCH_DEBS="${AUTO_FETCH_DEBS:-1}"

# Resolve a release's deb cache, fetching it first if it isn't there.
# Echoes the cache path on success. Returns non-zero only when the cache
# is absent AND we couldn't build it (auto-fetch off, or bundle-debs.sh
# failed — e.g. no docker/network). All progress goes to stderr so the
# command substitution captures only the resolved path.
ensure_debs_for_release() {
    local rel="$1"
    local src
    if src=$(debs_src_for_release "$rel"); then
        echo "$src"; return 0
    fi
    if [ "$AUTO_FETCH_DEBS" != "1" ]; then
        return 1
    fi
    if [ ! -x "$BUNDLE_DEBS_SCRIPT" ]; then
        echo "Cannot auto-fetch debs: ${BUNDLE_DEBS_SCRIPT} missing" >&2
        return 1
    fi
    echo "No runtime deb cache for ${rel} — fetching it (bundle-debs.sh)…" >&2
    if DEBIAN_RELEASE="$rel" ROS_DISTRO="$ROS_DISTRO" "$BUNDLE_DEBS_SCRIPT" >&2; then
        if src=$(debs_src_for_release "$rel"); then
            echo "$src"; return 0
        fi
    fi
    echo "bundle-debs.sh did not produce a usable cache for ${rel}" >&2
    return 1
}

# Single-target layout: bundle tree+debs at PACKAGE_DIR/ros2_install/
# and PACKAGE_DIR/deps/ — what every existing install.sh knows about.
bundle_ros2() {
    local src
    src=$(ros2_src_for_release "$TARGET_RELEASE") || true
    if [ -z "$src" ]; then
        echo "No ROS2 install tree found for ${TARGET_RELEASE}."
        if [ "$TARGET_RELEASE" = "trixie" ]; then
            echo "  Run firmware/raspberrypi/scripts/build-ros2-trixie.sh first."
        else
            echo "  Run scripts/build-local-dist.sh first (or place a tree at"
            echo "  _ros2_${TARGET_RELEASE}/opt/ros/${ROS_DISTRO}/install)."
        fi
        echo "  Without a tree the bundle will rely on online ros-apt-source at install time."
        return
    fi
    echo "Bundling ROS2 install tree (${TARGET_RELEASE}) from ${src} → ros2_install/"
    mkdir -p "${PACKAGE_DIR}/ros2_install"
    cp -a "${src}/." "${PACKAGE_DIR}/ros2_install/"
}

# Multi-target layout: bundle each available release's tree+debs under
# release-suffixed subdirs (ros2_install/<release>/, deps/<release>/).
# install.sh picks one at install time based on /etc/os-release.
bundle_ros2_multi() {
    local found_any=0
    for rel in bookworm trixie; do
        local src
        src=$(ros2_src_for_release "$rel") || true
        if [ -n "$src" ]; then
            echo "Bundling ROS2 install tree (${rel}) from ${src} → ros2_install/${rel}/"
            mkdir -p "${PACKAGE_DIR}/ros2_install/${rel}"
            cp -a "${src}/." "${PACKAGE_DIR}/ros2_install/${rel}/"
            found_any=1
            BUNDLED_RELEASES+=("$rel")
        else
            echo "Skipping ${rel} ROS2 tree — not built on this dev box."
        fi
    done
    if [ "$found_any" -ne 1 ]; then
        echo "ERROR: --multi-target but no ROS2 trees found for any release."
        exit 1
    fi
}

# Single-target deb cache → PACKAGE_DIR/deps/.
bundle_debs() {
    local src=""
    local from_server_fallback=0
    if src=$(ensure_debs_for_release "$TARGET_RELEASE"); then
        :
    elif [ -d "${REPO_ROOT}/_debs" ] && ls "${REPO_ROOT}/_debs"/*.deb >/dev/null 2>&1; then
        # Last-ditch: the shared server deb cache. Only safe when the Pi
        # target matches the cache's DEBIAN_RELEASE.
        src="${REPO_ROOT}/_debs"
        from_server_fallback=1
    fi
    if [ -z "$src" ] || [ ! -d "$src" ]; then
        echo "No runtime deb cache found — bundle will rely on online apt at install time"
        echo "Run firmware/raspberrypi/scripts/bundle-debs.sh first to produce _rpi_debs_${TARGET_RELEASE}/"
        return
    fi
    if [ "$from_server_fallback" -eq 1 ]; then
        echo "Falling back to server deb cache (${src}) — Pi-specific debs (vlc, gpiod, ros) NOT included"
        echo "  Run firmware/raspberrypi/scripts/bundle-debs.sh first for a full offline bundle"
    else
        echo "Bundling runtime apt debs (${TARGET_RELEASE}) from ${src} → deps/"
    fi
    mkdir -p "${PACKAGE_DIR}/deps"
    cp -a "${src}/." "${PACKAGE_DIR}/deps/"
    echo "  ($(ls -1 "${PACKAGE_DIR}/deps"/*.deb 2>/dev/null | wc -l | tr -d ' ') .debs bundled)"
}

# Multi-target deb caches → PACKAGE_DIR/deps/<release>/.
bundle_debs_multi() {
    for rel in "${BUNDLED_RELEASES[@]}"; do
        local src
        if ! src=$(ensure_debs_for_release "$rel"); then
            echo "WARNING: ROS tree present for ${rel} but no _rpi_debs_${rel}/ cache and auto-fetch failed." >&2
            echo "  Ensure docker + network are available, or run DEBIAN_RELEASE=${rel} firmware/raspberrypi/scripts/bundle-debs.sh first." >&2
            continue
        fi
        echo "Bundling runtime apt debs (${rel}) from ${src} → deps/${rel}/"
        mkdir -p "${PACKAGE_DIR}/deps/${rel}"
        cp -a "${src}/." "${PACKAGE_DIR}/deps/${rel}/"
        local n
        n=$(ls -1 "${PACKAGE_DIR}/deps/${rel}"/*.deb 2>/dev/null | wc -l | tr -d ' ')
        echo "  (${n} .debs bundled)"
    done
}

BUNDLED_RELEASES=()
if [ "$MULTI_TARGET" -eq 1 ]; then
    bundle_ros2_multi
    bundle_debs_multi
else
    bundle_ros2
    bundle_debs
    BUNDLED_RELEASES=("$TARGET_RELEASE")
fi

# Manifest so install.sh knows what's in the bundle without globbing.
# `targets` enumerates every release whose ros2_install/<rel> + deps/<rel>
# subdirs are present (multi-target layout). For single-target bundles
# the array contains one entry and the legacy top-level ros2_install/
# and deps/ are also populated — install.sh handles both shapes.
TARGETS_JSON=$(printf '"%s"\n' "${BUNDLED_RELEASES[@]}" | paste -sd, -)
cat > "${PACKAGE_DIR}/manifest.json" << EOF
{
    "type": "raspberrypi",
    "version": "${VERSION}",
    "ros_distro": "${ROS_DISTRO}",
    "multi_target": $([ "$MULTI_TARGET" -eq 1 ] && echo true || echo false),
    "targets": [${TARGETS_JSON}],
    "target_release": "${TARGET_RELEASE}",
    "has_ros2_bundle": $([ -d "${PACKAGE_DIR}/ros2_install" ] && echo true || echo false),
    "has_deps_bundle": $([ -d "${PACKAGE_DIR}/deps" ] && echo true || echo false),
    "built_at": "$(date -u +%Y-%m-%dT%H:%M:%SZ)"
}
EOF

# Create package info
cat > "${PACKAGE_DIR}/package_info.json" << EOF
{
    "type": "raspberrypi",
    "version": "${VERSION}",
    "created": "$(date -u +%Y-%m-%dT%H:%M:%SZ)",
    "files": [
        "saint_node/__init__.py",
        "saint_node/node.py",
        "saint_node/gpio_control.py",
        "saint_node/config.py",
        "saint_node/updater.py"
    ]
}
EOF

# Package filename: multi-target bundles get the unsuffixed name (one
# universal package per version, picked at install time on the Pi).
# Single-target bundles get a release suffix for non-bookworm so
# bookworm/trixie packages don't overwrite each other on the dev box.
#
# .tar.zst chosen over .zip for two reasons: ~16% smaller (zstd
# crushes the duplicated ROS .so binaries + apt .debs much better
# than zip's DEFLATE), and the main server dist already uses .tar.zst
# (scripts/make-dist.sh), so the two artifacts now share one format.
# zstd decompresses faster than DEFLATE too — material on the Pi 5
# where the OTA extract used to dominate update time.
if [ "$MULTI_TARGET" -eq 1 ]; then
    PACKAGE_NAME="saint_firmware_raspberrypi_${VERSION}.tar.zst"
elif [ "$TARGET_RELEASE" = "bookworm" ]; then
    PACKAGE_NAME="saint_firmware_raspberrypi_${VERSION}.tar.zst"
else
    PACKAGE_NAME="saint_firmware_raspberrypi_${VERSION}_${TARGET_RELEASE}.tar.zst"
fi
PACKAGE_PATH="${OUTPUT_DIR}/${PACKAGE_NAME}"

echo "Creating package: ${PACKAGE_NAME}"
# Delete any prior archive at the same path first. tar + zstd are
# stream-based so they don't suffer the zip-r merge problem, but we
# still wipe to keep the "this build = exactly these bytes" invariant
# that the cleanup logic above assumes.
rm -f "${PACKAGE_PATH}"
cd "${TEMP_DIR}"
# Require zstd on PATH. Modern tar (1.32+, on every supported target
# including macOS Homebrew tar and Debian Bookworm/Trixie GNU tar)
# detects libzstd at build time and accepts --zstd directly; if a
# dev box has stripped-down tar the --use-compress-program path
# below covers it.
if ! command -v zstd >/dev/null 2>&1; then
    echo "ERROR: zstd not found on PATH. Install it (brew install zstd / apt-get install zstd) and retry."
    exit 1
fi
# Compression level 19: ~10% smaller than the default 3 on our
# payload (ROS install trees + .debs), at ~30s extra build time on
# Apple Silicon. The packager is build-time only so the trade is
# strongly in favor of size. The Pi's decompression speed is
# independent of compression level — same zstd-stream throughput
# either way. -T0 uses every dev-box core.
tar --use-compress-program='zstd -19 -T0' \
    -cf "${PACKAGE_PATH}" "saint_firmware_raspberrypi_${VERSION}"

# Calculate checksum
CHECKSUM=$(shasum -a 256 "${PACKAGE_PATH}" | cut -d' ' -f1)

# Create info.json for server
cat > "${OUTPUT_DIR}/info.json" << EOF
{
    "type": "raspberrypi",
    "latest_version": "${VERSION}",
    "latest_package": "${PACKAGE_NAME}",
    "latest_checksum": "${CHECKSUM}",
    "updated": "$(date -u +%Y-%m-%dT%H:%M:%SZ)",
    "packages": [
        {
            "version": "${VERSION}",
            "filename": "${PACKAGE_NAME}",
            "checksum": "${CHECKSUM}",
            "size": $(stat -f%z "${PACKAGE_PATH}" 2>/dev/null || stat -c%s "${PACKAGE_PATH}")
        }
    ]
}
EOF

# Clean up
rm -rf "${TEMP_DIR}"

echo ""
echo "Package created: ${PACKAGE_PATH}"
echo "Checksum: ${CHECKSUM}"
echo ""
echo "To deploy to server:"
echo "  cp ${PACKAGE_PATH} /path/to/server/resources/firmware/raspberrypi/"
echo "  cp ${OUTPUT_DIR}/info.json /path/to/server/resources/firmware/raspberrypi/"
