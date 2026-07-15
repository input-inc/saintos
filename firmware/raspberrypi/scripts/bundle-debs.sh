#!/usr/bin/env bash
#
# Build the Pi-node-side .deb cache for offline install.
#
# Runs apt-get download inside a clean debian:<release>-slim container
# (so we don't taint the host's apt cache and so the .debs match the
# Pi's target Debian release exactly). The container:
#
#   1. Adds the OSRF apt source (so ros-${ROS_DISTRO}-* is resolvable).
#   2. Downloads every runtime deb listed in $RUNTIME_DEB_LIST plus
#      ros-${ROS_DISTRO}-ros-base and all of its transitive deps.
#   3. Builds a Packages.gz index so install.sh can use the directory
#      directly as a file:// apt source.
#
# Output lands at <repo-root>/_rpi_debs/. firmware/raspberrypi/scripts/
# package.sh picks it up automatically and copies it into the
# distribution bundle as deps/.
#
# Defaults: ARCH=arm64, DEBIAN_RELEASE=trixie, ROS_DISTRO=kilted —
# matches the current Pi 5 fleet. Override via env vars.
#
# Usage:
#   firmware/raspberrypi/scripts/bundle-debs.sh
#   DEBIAN_RELEASE=bookworm ROS_DISTRO=jazzy ARCH=arm64 ./bundle-debs.sh
#   ARCH=amd64 ./bundle-debs.sh                       # for x86 Pi-emu dev hosts
#

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../../.." && pwd)"

ARCH="${ARCH:-arm64}"
DEBIAN_RELEASE="${DEBIAN_RELEASE:-trixie}"
# ROS_DISTRO is kept for the sentinel hash + symmetry with the build
# scripts, but no ros-${ROS_DISTRO}-* .debs are fetched here (ROS lives
# in the source-built _ros2_${DEBIAN_RELEASE}/ tree instead).
ROS_DISTRO="${ROS_DISTRO:-kilted}"

# Per-release output dir so a single dev box can hold both a bookworm
# and a trixie deb cache without overwriting each other. package.sh
# prefers _rpi_debs_${TARGET_RELEASE} over the legacy _rpi_debs/.
OUTPUT_DIR="${REPO_ROOT}/_rpi_debs_${DEBIAN_RELEASE}"

# Pi-node runtime apt deps. ROS2 itself is NOT downloaded here — it's
# bundled as a source-built tree (_ros2/ for bookworm, _ros2_trixie/
# for trixie) since OSRF doesn't publish ros-${ROS_DISTRO}-* .debs for
# Debian Trixie. The .debs below cover what the ROS install tree's
# binaries dynamically link against PLUS the Pi-side runtime deps
# (gpiod, vlc, alsa). Keep this list in lockstep with
# firmware/raspberrypi/scripts/install.sh (Step 1 + Step 4).
#
# libpython is intentionally NOT listed by version — apt resolves the
# right libpython for the codename (3.11 on bookworm, 3.13 on trixie).
# Same story for libtinyxml2-* (-9 on bookworm, -11 on trixie): we list
# libtinyxml2-dev instead so apt pulls the matching runtime SO.
read -r -d '' RUNTIME_DEB_LIST <<'DEB_LIST' || true
python3
python3-pip
python3-venv
python3-yaml
python3-numpy
python3-lark
python3-empy
python3-packaging
python3-importlib-metadata
python3-argcomplete
python3-psutil
python3-typing-extensions
python3-bleak
libssl3
libyaml-0-2
libyaml-cpp-dev
libxml2
libacl1
libcurl4
libtinyxml2-dev
libconsole-bridge1.0
liblttng-ust1
liborocos-kdl1.5
libgpiod-dev
python3-libgpiod
vlc-bin
python3-vlc
alsa-utils
python3-alsaaudio
ca-certificates
curl
rsync
zstd
DEB_LIST

if [[ ! -d "$OUTPUT_DIR" ]]; then
    mkdir -p "$OUTPUT_DIR"
fi

echo "Bundling Pi-node debs:"
echo "  arch=${ARCH}  debian=${DEBIAN_RELEASE}  ros=${ROS_DISTRO}"
echo "  output=${OUTPUT_DIR}"

# Idempotent reuse: if the cache is fresh (same input set), bail.
SENTINEL="${OUTPUT_DIR}/.bundle-sha256"
INPUT_HASH=$(printf '%s\n%s\n%s\n%s' \
    "$ARCH" "$DEBIAN_RELEASE" "$ROS_DISTRO" "$RUNTIME_DEB_LIST" \
    | shasum -a 256 | awk '{print $1}' | cut -c1-12)
if [[ -f "$SENTINEL" && "$(cat "$SENTINEL" 2>/dev/null)" == "$INPUT_HASH" ]] \
   && ls "$OUTPUT_DIR"/*.deb >/dev/null 2>&1; then
    echo "Cache fresh (hash=${INPUT_HASH}, $(ls -1 "$OUTPUT_DIR"/*.deb | wc -l | tr -d ' ') debs) — re-run with --force to rebuild"
    if [[ "${1:-}" != "--force" ]]; then
        exit 0
    fi
fi

# Wipe and rebuild — easier than tracking which .debs the new input
# set would render obsolete.
find "$OUTPUT_DIR" -mindepth 1 -delete

docker run --rm -i \
    --platform "linux/${ARCH}" \
    -v "${OUTPUT_DIR}:/debs" \
    -e RUNTIME_DEB_LIST="${RUNTIME_DEB_LIST}" \
    "debian:${DEBIAN_RELEASE}-slim" \
    bash -eo pipefail <<'CONTAINER_EOF'

export DEBIAN_FRONTEND=noninteractive

# apt-utils provides apt-ftparchive (needed to build Packages.gz so the
# Pi can use file:///deps as an apt source).
apt-get update
apt-get install -y --no-install-recommends apt-utils

# ROS2 itself is shipped as a source-built tree (_ros2_${codename}/),
# not from apt — OSRF doesn't publish ros-${ROS_DISTRO}-* for Trixie,
# and the bookworm Pi flow already uses the server's source build.
# We only need the runtime apt deps here. Skipping the OSRF apt-source
# .deb avoids the 1.1.0/1.2.0 codename-coverage minefield entirely.

# --download-only --reinstall stages the .debs into the apt archives
# without actually configuring them. The container's filesystem stays
# uninstalled; we just want the binaries.
# shellcheck disable=SC2086
apt-get install -y --download-only --reinstall ${RUNTIME_DEB_LIST}

cp /var/cache/apt/archives/*.deb /debs/

# Build the Packages.gz index so the target's install.sh can use
# file:///debs as an apt source. apt-ftparchive ships with apt-utils.
cd /debs
apt-ftparchive packages . > Packages
gzip -kf Packages

echo "Bundled $(ls -1 *.deb | wc -l) .deb files, $(du -sh . | cut -f1) total"
CONTAINER_EOF

# Verify on the host that debs actually persisted before stamping the
# sentinel. The container reports its own count, but if the bind-mount
# write-back didn't land (or nothing downloaded), we'd otherwise leave an
# empty dir with a "fresh" sentinel — and the next build would treat that
# empty cache as complete and silently ship a bundle with no deps/.
deb_count=$(ls -1 "$OUTPUT_DIR"/*.deb 2>/dev/null | wc -l | tr -d ' ')
if [ "$deb_count" -eq 0 ]; then
    echo "ERROR: no .debs landed in ${OUTPUT_DIR} after the container run" >&2
    echo "  (docker bind-mount write-back failed, or nothing downloaded)" >&2
    exit 1
fi

printf '%s' "$INPUT_HASH" > "$SENTINEL"
echo ""
echo "Done. Next: re-run firmware/raspberrypi/scripts/package.sh to bundle"
echo "      these .debs into the Pi node firmware zip."
