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
# Default: bookworm (preserves existing fleet behaviour).
TARGET_RELEASE="bookworm"

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

# Create output directory
mkdir -p "${OUTPUT_DIR}"

# Create temporary directory for packaging
TEMP_DIR=$(mktemp -d)
PACKAGE_DIR="${TEMP_DIR}/saint_firmware_raspberrypi_${VERSION}"
mkdir -p "${PACKAGE_DIR}"

# Copy firmware files
echo "Copying firmware files..."
cp -r "${FIRMWARE_DIR}/saint_node" "${PACKAGE_DIR}/"
cp -r "${FIRMWARE_DIR}/config" "${PACKAGE_DIR}/" 2>/dev/null || true
cp -r "${FIRMWARE_DIR}/scripts" "${PACKAGE_DIR}/" 2>/dev/null || true

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

bundle_ros2() {
    local src=""
    local release_specific="${REPO_ROOT}/_ros2_${TARGET_RELEASE}/opt/ros/${ROS_DISTRO}/install"
    local default_tree="${REPO_ROOT}/_ros2/opt/ros/${ROS_DISTRO}/install"
    if [ -n "${SAINT_RPI_ROS2_DIR:-}" ]; then
        src="${SAINT_RPI_ROS2_DIR}"
    elif [ -d "$release_specific" ]; then
        # Per-release source build present (e.g. _ros2_trixie/). Prefer
        # it — its Python ABI was chosen to match the Pi target.
        src="$release_specific"
        echo "Using ${TARGET_RELEASE}-specific ROS2 tree from ${src}"
    elif [ "$TARGET_RELEASE" = "bookworm" ] && [ -d "$default_tree" ]; then
        # Bookworm fallback: the server-side build at _ros2/ targets
        # Bookworm already (libpython3.11), so it's safe to use for
        # Bookworm Pi nodes without a release-specific subtree.
        src="$default_tree"
        echo "Using server _ros2/ tree (bookworm) from ${src}"
    fi
    if [ -z "$src" ] || [ ! -d "$src" ]; then
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
    echo "Bundling ROS2 install tree from ${src} → ros2_install/"
    mkdir -p "${PACKAGE_DIR}/ros2_install"
    cp -a "${src}/." "${PACKAGE_DIR}/ros2_install/"
}

bundle_debs() {
    local src=""
    local from_server_fallback=0
    # Release-specific deb cache (built by bundle-debs.sh with matching
    # DEBIAN_RELEASE) is the only safe pick when targeting Trixie —
    # _debs/ is built for the server's Bookworm and would 404 / mismatch
    # half the runtime libs.
    local release_specific="${REPO_ROOT}/_rpi_debs_${TARGET_RELEASE}"
    if [ -n "${SAINT_RPI_DEBS_DIR:-}" ]; then
        src="${SAINT_RPI_DEBS_DIR}"
    elif [ -d "$release_specific" ] && ls "$release_specific"/*.deb >/dev/null 2>&1; then
        src="$release_specific"
    elif [ -d "${REPO_ROOT}/_rpi_debs" ] && ls "${REPO_ROOT}/_rpi_debs"/*.deb >/dev/null 2>&1; then
        src="${REPO_ROOT}/_rpi_debs"
    elif [ -d "${REPO_ROOT}/_debs" ] && ls "${REPO_ROOT}/_debs"/*.deb >/dev/null 2>&1; then
        # Fall back to the shared server deb cache. Only safe when the
        # Pi target matches the cache's DEBIAN_RELEASE — operators on
        # mismatched releases should populate _rpi_debs instead via
        # firmware/raspberrypi/scripts/bundle-debs.sh.
        src="${REPO_ROOT}/_debs"
        from_server_fallback=1
    fi
    if [ -z "$src" ] || [ ! -d "$src" ]; then
        echo "No runtime deb cache found — bundle will rely on online apt at install time"
        echo "Run firmware/raspberrypi/scripts/bundle-debs.sh first to produce _rpi_debs/"
        return
    fi
    if [ $from_server_fallback -eq 1 ]; then
        echo "Falling back to server deb cache (${src}) — Pi-specific debs (vlc, gpiod, ros) NOT included"
        echo "  Run firmware/raspberrypi/scripts/bundle-debs.sh first for a full offline bundle"
    else
        echo "Bundling runtime apt debs from ${src} → deps/"
    fi
    mkdir -p "${PACKAGE_DIR}/deps"
    cp -a "${src}/." "${PACKAGE_DIR}/deps/"
    echo "  ($(ls -1 "${PACKAGE_DIR}/deps"/*.deb 2>/dev/null | wc -l | tr -d ' ') .debs bundled)"
}

bundle_ros2
bundle_debs

# Manifest so install.sh can read the ROS distro + target release
# without guessing. target_release lets install.sh refuse to drop a
# bookworm bundle onto a trixie Pi (the ABI mismatch we keep hitting).
cat > "${PACKAGE_DIR}/manifest.json" << EOF
{
    "type": "raspberrypi",
    "version": "${VERSION}",
    "ros_distro": "${ROS_DISTRO}",
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

# Create zip package
PACKAGE_NAME="saint_firmware_raspberrypi_${VERSION}.zip"
PACKAGE_PATH="${OUTPUT_DIR}/${PACKAGE_NAME}"

echo "Creating package: ${PACKAGE_NAME}"
# Delete any prior zip at the same path first. `zip -r` UPDATES an
# existing archive (adds + replaces files but doesn't remove paths that
# no longer exist in the source). A previous run with --target-release
# bookworm would leave python3.11/ entries that a subsequent trixie run
# can't strip, producing a zip with BOTH python3.11/ and python3.13/
# trees and confusing the Pi installer's ABI detection.
rm -f "${PACKAGE_PATH}"
cd "${TEMP_DIR}"
zip -r "${PACKAGE_PATH}" "saint_firmware_raspberrypi_${VERSION}"

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
