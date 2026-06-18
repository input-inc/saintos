#!/usr/bin/env bash
#
# Source-build a Trixie/python3.13-compatible ROS2 install tree for
# Raspberry Pi nodes running Pi OS 13 (Trixie). Wraps the existing
# scripts/build-ros2-in-container.sh with DEBIAN_RELEASE=trixie and
# extracts the resulting tarball to _ros2_trixie/opt/ros/<distro>/install/
# where firmware/raspberrypi/scripts/package.sh picks it up when
# packaging with --target-release trixie.
#
# Why this exists: OSRF does NOT publish ros-${ROS_DISTRO}-* .debs for
# Debian Trixie (only Ubuntu Noble has Kilted binaries; their trixie
# repo holds build tooling only). Pi nodes running Trixie can't apt-
# install ROS, so they need a source-built tree. The server-side
# _ros2/ tree targets Debian Bookworm (libpython3.11) and is ABI-
# incompatible with Trixie's libpython3.13.
#
# Pi nodes with Bookworm: use the existing _ros2/ tree (built by
# scripts/build-local-dist.sh — no changes needed).
# Pi nodes with Trixie:   run THIS script once, then re-run
# firmware/raspberrypi/scripts/package.sh --target-release trixie.
#
# Wall-clock cost: ~2-3 hours on Apple Silicon (native arm64), 4+
# hours on Intel via qemu emulation.

set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../../.." && pwd)"

# Pin to the same upstream refs build-local-dist.sh uses so micro-ROS
# protocol stays compatible across nodes regardless of which build
# produced the tree.
ROS_DISTRO="${ROS_DISTRO:-kilted}"
MICRO_ROS_REF="${MICRO_ROS_REF:-6.0.1}"
ROS2_REPOS_REF="${ROS2_REPOS_REF:-release-kilted-20250728}"
DEBIAN_RELEASE="trixie"

OUTPUT_PARENT="${REPO_ROOT}/_ros2_trixie/opt/ros/${ROS_DISTRO}"
OUTPUT_TREE="${OUTPUT_PARENT}/install"

if [[ -f "${OUTPUT_TREE}/setup.bash" ]]; then
    if [[ "${1:-}" == "--force" ]]; then
        echo "--force: rebuilding even though _ros2_trixie tree already exists"
    elif [[ ! -t 0 ]]; then
        echo "Existing _ros2_trixie tree at ${OUTPUT_TREE} — skipping rebuild (non-interactive)."
        echo "Pass --force to rebuild."
        exit 0
    else
        echo "Existing _ros2_trixie tree found at ${OUTPUT_TREE}"
        read -p "Re-run the source build anyway? (y/N) " -n 1 -r
        echo
        if [[ ! $REPLY =~ ^[Yy]$ ]]; then
            echo "Keeping existing tree. Done."
            exit 0
        fi
    fi
fi

echo "Building ROS2 ${ROS_DISTRO} for arm64 against Debian ${DEBIAN_RELEASE}."
echo "Expected wall-clock: 2-3 hours on Apple Silicon. Coffee."
echo ""

# Stage the build workspace inside the repo. Docker Desktop on macOS
# auto-shares the repo's path but doesn't reliably share /var/folders/
# (where mktemp lands), so a /var/folders bind mount silently appears
# empty inside the container — manifested as "/work/_ros2_src/ros2.repos
# does not exist" in vcs import. Use _ros2_trixie_build/ instead.
BUILD_WS="${REPO_ROOT}/_ros2_trixie_build"
# Wipe between runs so a partially-checked-out src tree (e.g. from a
# failed prior attempt) doesn't confuse vcs import.
rm -rf "$BUILD_WS"
mkdir -p "${BUILD_WS}/_ros2_src"
echo "Build workspace: $BUILD_WS"

echo "Fetching ros2.repos at ${ROS2_REPOS_REF}…"
curl -fsSL \
    "https://raw.githubusercontent.com/ros2/ros2/${ROS2_REPOS_REF}/ros2.repos" \
    -o "${BUILD_WS}/_ros2_src/ros2.repos"

# Sanity: confirm the file actually landed (catches a silent curl
# failure before we burn 2 hours in docker land).
[[ -s "${BUILD_WS}/_ros2_src/ros2.repos" ]] \
    || { echo "ERROR: ros2.repos download produced an empty file" >&2; exit 1; }

export ROS_DISTRO MICRO_ROS_REF DEBIAN_RELEASE
GITHUB_WORKSPACE="$BUILD_WS" bash "${REPO_ROOT}/scripts/build-ros2-in-container.sh"

if [[ ! -f "${BUILD_WS}/ros2-install.tar.gz" ]]; then
    echo "ERROR: build did not produce ros2-install.tar.gz" >&2
    exit 1
fi

echo ""
echo "Extracting to ${OUTPUT_TREE}…"
rm -rf "${OUTPUT_PARENT}"
mkdir -p "${OUTPUT_PARENT}"
tar -xzf "${BUILD_WS}/ros2-install.tar.gz" -C "${OUTPUT_PARENT}"

# Keep BUILD_WS around for incremental rebuilds (colcon's build/ +
# install/ + log/ are in there). It's gitignored by virtue of the
# _ros2* glob already in .gitignore.

if [[ ! -f "${OUTPUT_TREE}/setup.bash" ]]; then
    echo "ERROR: extracted tree missing setup.bash" >&2
    exit 1
fi

# Sanity-check the Python ABI in the resulting tree matches Trixie's 3.13.
# If the build container's apt resolved a different python3 than expected
# (e.g. trixie's default changed), catch it here rather than at install
# time on the Pi.
if [[ ! -d "${OUTPUT_TREE}/lib/python3.13" ]]; then
    echo "WARNING: tree built but lib/python3.13/ is missing — the Pi's python3 (3.13) won't load these binaries." >&2
    echo "Found Python dirs:" >&2
    find "${OUTPUT_TREE}/lib" -maxdepth 1 -name 'python*' -type d -exec basename {} \; >&2
    exit 1
fi

echo ""
echo "Done. Trixie ROS2 tree at: ${OUTPUT_TREE}"
echo ""
echo "Next:"
echo "  firmware/raspberrypi/scripts/bundle-debs.sh     # rebuild Trixie deb cache"
echo "  firmware/raspberrypi/scripts/package.sh --target-release trixie"
