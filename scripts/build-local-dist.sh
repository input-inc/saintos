#!/usr/bin/env bash
#
# Build a SAINT.OS dist tarball locally without GitHub Actions.
#
# Mirrors .github/workflows/dist.yml's package-server job, but caches the
# slow-to-fetch pieces (ROS2 install tree, bundled .deb cache) in
# ~/.cache/saint-os/ so iteration loops just rebuild the saint_os colcon
# install — typically 2-4 minutes per run after the first.
#
# Output: dist/saint-os_<version>_arm64_jazzy.tar.gz
#
# Usage:
#   scripts/build-local-dist.sh [options]
#
# Default behavior: builds RP2040 / Teensy / Pi5 node firmware AND the
# Steam Deck Controller .AppImage locally so the server tarball ships
# everything needed to OTA-update every connected target. Override with
# the firmware / controller skip flags below.
#
# Options:
#   --version VER            Override version string (default: <VERSION>-local.<sha7>)
#   --rebundle-debs          Re-download the bundled .deb cache (slow; usually unneeded)
#   --refetch-ros2           Force re-download of the bundled ROS2 install tree
#   --fetch-firmware         Pull the latest CI firmware + controller artifacts from
#                            GitHub via `gh` instead of building locally (requires
#                            `gh auth login`)
#   --skip-firmware-build    Use whatever firmware is already staged under
#                            server/resources/firmware/ — fastest server-only iteration
#   --skip-firmware          Don't stage any firmware OR controller (smallest tarball)
#   --skip-controller-build  Use whatever controller bundle is already staged; skip
#                            the linux/amd64 Docker build (saves the QEMU/Rosetta tax)
#   --skip-web-build         Skip `npm run build` — reuse whatever's already in
#                            server/web/dist/. Speeds up iteration when only
#                            Python or firmware changed.
#   --clean                  Remove the local build dirs (_ros2/, _debs/, install/) first
#   -h, --help               Show this help
#
# Requirements:
#   - docker (with linux/arm64 platform support — buildx + qemu on Intel)
#   - curl, tar, sha256sum / shasum
#   - python3
#   - node + npm (skipped only with --skip-web-build)
#   - gh (only with --fetch-firmware)

set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$REPO_ROOT"

ROS_DISTRO="kilted"
ARCH="arm64"
DEBIAN_RELEASE="bookworm"
# Upstream pins — must match the same-named env vars in
# .github/workflows/ros2-build.yml and .github/workflows/dist.yml or
# the local hash will diverge from CI's and we won't find the cached
# release. Bump deliberately (in all three files at once).
ROS2_REPOS_REF="${ROS2_REPOS_REF:-release-kilted-20250728}"
MICRO_ROS_REF="${MICRO_ROS_REF:-6.0.1}"
REPO_SLUG="${SAINT_REPO_SLUG:-input-inc/saintos}"

CACHE_ROOT="${SAINT_LOCAL_CACHE:-${HOME}/.cache/saint-os}"
mkdir -p "$CACHE_ROOT"

# --- options ----------------------------------------------------------------

VERSION=""
REBUNDLE_DEBS=0
REFETCH_ROS2=0
FETCH_FIRMWARE=0
SKIP_FIRMWARE=0
SKIP_FIRMWARE_BUILD=0
SKIP_CONTROLLER_BUILD=0
SKIP_WEB_BUILD=0
CLEAN=0

while (( "$#" )); do
    case "$1" in
        --version) VERSION="$2"; shift 2 ;;
        --rebundle-debs) REBUNDLE_DEBS=1; shift ;;
        --refetch-ros2) REFETCH_ROS2=1; shift ;;
        --fetch-firmware) FETCH_FIRMWARE=1; shift ;;
        --skip-firmware) SKIP_FIRMWARE=1; shift ;;
        --skip-firmware-build) SKIP_FIRMWARE_BUILD=1; shift ;;
        --skip-controller-build) SKIP_CONTROLLER_BUILD=1; shift ;;
        --skip-web-build) SKIP_WEB_BUILD=1; shift ;;
        --clean) CLEAN=1; shift ;;
        -h|--help) sed -n '2,45p' "$0"; exit 0 ;;
        *) echo "Unknown option: $1" >&2; exit 1 ;;
    esac
done

log() { printf '\033[1;36m[build]\033[0m %s\n' "$*"; }
warn() { printf '\033[1;33m[build]\033[0m %s\n' "$*" >&2; }
die() { printf '\033[1;31m[build]\033[0m %s\n' "$*" >&2; exit 1; }

# --- preflight --------------------------------------------------------------

command -v docker >/dev/null || die "docker is required"
command -v curl >/dev/null || die "curl is required"
command -v python3 >/dev/null || die "python3 is required"
if command -v sha256sum >/dev/null; then SHA256="sha256sum"; else SHA256="shasum -a 256"; fi

# Probe linux/arm64 emulation. Apple Silicon: native. Intel Mac + Linux x86: qemu.
if ! docker buildx ls >/dev/null 2>&1; then
    warn "docker buildx not available — linux/arm64 builds may fail"
fi

# --- version resolution -----------------------------------------------------

BASE_VERSION=$(tr -d '[:space:]' < server/VERSION)
GIT_SHA=$(git rev-parse --short=7 HEAD 2>/dev/null || echo unknown)
if [[ -z "$VERSION" ]]; then
    VERSION="${BASE_VERSION}-local.${GIT_SHA}"
fi
log "Building version: ${VERSION}"

# --- optional clean ---------------------------------------------------------

if (( CLEAN )); then
    log "Cleaning local build artifacts"
    # Top-level extract / colcon outputs / staged firmware
    rm -rf _ros2 _debs install build dist server/resources/firmware
    # Per-firmware build dirs (CMake out-of-source trees). Incremental
    # builds normally keep these between runs; --clean is the only path
    # that wipes them.
    rm -rf firmware/rp2040/build
    rm -rf firmware/teensy41/.pio
    rm -rf firmware/raspberrypi/dist
fi

# colcon `build/` and `install/` at the repo root persist between runs
# (the container bind-mounts `$PWD` to `/work`, so colcon's outputs land
# here on the host). That's why a server-only iteration after the first
# typically takes seconds — only the changed package gets recompiled.
# Use `--clean` above to start fresh.

# --- ROS2 install tree (cached) ---------------------------------------------
#
# Same hash logic as dist.yml — find the matching ros2-build release.

if [[ ! -d _ros2_src ]]; then
    mkdir -p _ros2_src
fi
( cd _ros2_src
  if [[ ! -f ros2.repos || $REFETCH_ROS2 -eq 1 ]]; then
      curl -fsSL \
        "https://raw.githubusercontent.com/ros2/ros2/${ROS2_REPOS_REF}/ros2.repos" \
        -o ros2.repos
  fi
  # Reuse the cached micro-ROS SHA when we already have one. git ls-remote
  # requires internet, and an offline build with warm caches should just
  # work — the cached ROS2 tarball was built against a known-good SHA so
  # the existing micro_ros.txt is fine for re-deriving the cache key.
  if [[ ! -f micro_ros.txt || $REFETCH_ROS2 -eq 1 ]]; then
      # Resolve MICRO_ROS_REF as either a tag or a branch (or raw SHA).
      # `git ls-remote` returns the matching ref's SHA; head -n1 picks
      # the first match if there's ambiguity (rare — tags and branches
      # don't usually share names).
      if MICRO_SHA=$(git ls-remote https://github.com/micro-ROS/micro_ros_setup.git \
            "refs/tags/${MICRO_ROS_REF}" "refs/heads/${MICRO_ROS_REF}" "${MICRO_ROS_REF}" \
            2>/dev/null | head -n1 | awk '{print $1}') \
            && [[ -n "$MICRO_SHA" ]]; then
          echo "micro_ros_setup_sha=${MICRO_SHA}" > micro_ros.txt
      else
          die "Cannot resolve micro-ROS SHA and no cached micro_ros.txt to fall back to. Connect to a network or run with --refetch-ros2 once online."
      fi
  fi
)
# Mirror the hash computation from .github/workflows/ros2-build.yml so the
# release tag we look up here matches what CI built. Includes the build
# script content so a pin change in scripts/build-ros2-in-container.sh
# (e.g. bumping Micro-XRCE-DDS-Agent past v2.4.3) produces a fresh
# ROS_TAG and stale local caches don't mask the upstream rebuild.
ROS_HASH=$(cat _ros2_src/ros2.repos _ros2_src/micro_ros.txt \
    "${REPO_ROOT}/scripts/build-ros2-in-container.sh" | $SHA256 | cut -c1-12)
ROS_TAG="ros2-${ROS_DISTRO}-${ARCH}-${ROS_HASH}"
ROS_CACHE_TARBALL="${CACHE_ROOT}/${ROS_TAG}.tar.gz"

# Local-iteration fast path: if we already have a working extracted
# install tree at _ros2/, trust it and skip the download/extract dance.
# This decouples local iteration from CI: the operator can edit
# build-ros2-in-container.sh (which the hash includes) without first
# having to push and re-run the ros2-build workflow to publish a tarball
# under the new hash. Pass --refetch-ros2 to override.
EXTRACTED_INSTALL="_ros2/opt/ros/${ROS_DISTRO}/install"
if (( REFETCH_ROS2 )); then
    log "Forcing ROS2 re-fetch (--refetch-ros2) — wiping ${EXTRACTED_INSTALL}"
    rm -rf "$EXTRACTED_INSTALL"
fi

# Tighter completeness check: require both shell entry points AND a
# canonical ament cmake config. setup.bash alone isn't enough — a
# partial/interrupted prior extract can leave just that file behind
# and silently break the colcon build later. Anything looking
# incomplete forces a full re-extract from the cached tarball.
EXTRACT_COMPLETE=1
for f in "${EXTRACTED_INSTALL}/setup.bash" \
         "${EXTRACTED_INSTALL}/local_setup.bash" \
         "${EXTRACTED_INSTALL}/share/ament_cmake/cmake/ament_cmakeConfig.cmake"; do
    [[ -f "$f" ]] || { EXTRACT_COMPLETE=0; break; }
done

if (( EXTRACT_COMPLETE )); then
    log "Using already-extracted ROS2 install tree at ${EXTRACTED_INSTALL} (skipping download)"
else
    if [[ ! -f "$ROS_CACHE_TARBALL" ]]; then
        log "Downloading ROS2 install tree: ${ROS_TAG}"
        curl -fL --progress-bar \
          "https://github.com/${REPO_SLUG}/releases/download/${ROS_TAG}/ros2-install.tar.gz" \
          -o "$ROS_CACHE_TARBALL" \
          || die "ROS2 release ${ROS_TAG} not found. Run the ros2-build workflow first, or re-extract a prior tarball under _ros2/opt/ros/${ROS_DISTRO}/install/."
    else
        log "Using cached ROS2 tarball ($(du -h "$ROS_CACHE_TARBALL" | cut -f1))"
    fi
    log "Extracting ROS2 install tree (wiping any partial prior extract)"
    rm -rf "_ros2/opt/ros/${ROS_DISTRO}"
    mkdir -p "_ros2/opt/ros/${ROS_DISTRO}"
    tar -xzf "$ROS_CACHE_TARBALL" -C "_ros2/opt/ros/${ROS_DISTRO}"
fi
[[ -f "${EXTRACTED_INSTALL}/setup.bash" ]] \
    || die "ROS2 setup.bash missing after extract"
[[ -f "${EXTRACTED_INSTALL}/local_setup.bash" ]] \
    || die "ROS2 local_setup.bash missing after extract — tarball is incomplete"

# --- firmware --------------------------------------------------------------
#
# The server tarball is the source of truth for node firmware — operators
# expect installing a new server build to ship the matching firmware.
# Default behavior here is therefore to *build* firmware locally before
# packaging. Override with --skip-firmware-build (use whatever's already
# staged), --fetch-firmware (pull from CI artifacts), or --skip-firmware
# (omit firmware entirely).

build_firmware_rp2040() {
    if [[ ! -x "firmware/rp2040/build.sh" ]]; then
        warn "RP2040 build script missing — skipping"
        return
    fi
    log "Building RP2040 hardware firmware (OTA bootloader ON, incremental)"
    # Build with the OTA bootloader enabled so the dist tarball contains
    # both the combined first-flash .uf2 and the body-only .bin the
    # bootloader fetches over HTTP.
    #
    # Incremental: keep build/ between runs so cmake + make only rebuild
    # what changed. `--clean` wipes it. The previous version did
    # `rm -rf build` every time on the theory that cmake config drift
    # could leave stale state — but cmake is robust to re-running on an
    # existing tree, and our config (-DSIMULATION + -DSAINT_OS_OTA_BOOTLOADER)
    # doesn't change run-to-run unless someone edits this script.
    ( cd firmware/rp2040 \
        && mkdir -p build && cd build \
        && cmake -DSIMULATION=OFF -DSAINT_OS_OTA_BOOTLOADER=ON .. > /dev/null \
        && make -j"$(sysctl -n hw.ncpu 2>/dev/null || nproc 2>/dev/null || echo 4)" \
               saint_node saint_ota_bootloader saint_node_combined ) \
        || { warn "RP2040 firmware build failed — leaving existing staged files"; return; }

    local fw_out=firmware/rp2040/build
    local fw_bl=firmware/rp2040/build/bootloader

    if [[ ! -f "${fw_out}/saint_node.uf2" ]]; then
        warn "RP2040 build produced no saint_node.uf2 — staged files unchanged"
        return
    fi
    mkdir -p server/resources/firmware/rp2040
    # App artifacts: .uf2 (legacy first-flash), .elf (debug), .bin (OTA fetch).
    find "${fw_out}" -maxdepth 1 -type f \
        \( -name 'saint_node.uf2' -o -name 'saint_node.elf' -o -name 'saint_node.bin' \) \
        -exec cp -v {} server/resources/firmware/rp2040/ \;
    # Combined .uf2 = bootloader + app, for first-time BOOTSEL flash.
    if [[ -f "${fw_out}/saint_node_combined.uf2" ]]; then
        cp -v "${fw_out}/saint_node_combined.uf2" \
            server/resources/firmware/rp2040/
    fi
    # Bootloader-only .uf2, for ad-hoc bootloader reflashing.
    if [[ -f "${fw_bl}/saint_ota_bootloader.uf2" ]]; then
        cp -v "${fw_bl}/saint_ota_bootloader.uf2" \
            server/resources/firmware/rp2040/
    fi
    # version.h carries FIRMWARE_VERSION_STRING / FIRMWARE_VERSION_FULL /
    # FIRMWARE_GIT_HASH / FIRMWARE_BUILD_TIMESTAMP. The server reads it
    # to display the build version + decide whether an update is needed.
    if [[ -f "${fw_out}/generated/version.h" ]]; then
        mkdir -p server/resources/firmware/rp2040/generated
        cp -v "${fw_out}/generated/version.h" \
            server/resources/firmware/rp2040/generated/version.h
    fi
}

build_firmware_teensy41() {
    if [[ ! -x "firmware/teensy41/build.sh" ]]; then
        warn "Teensy build script missing — skipping"
        return
    fi
    log "Building Teensy 4.1 hardware firmware (best-effort; needs PlatformIO + binutils)"
    if ! ( cd firmware/teensy41 && ./build.sh hw 2>&1 ); then
        warn "Teensy firmware build failed — leaving existing staged files (run 'brew install binutils' if missing)"
        return
    fi
    # stage_firmware.py (a SCons post-action in platformio.ini) already copies
    # firmware.hex + saint_node.bin into server/resources/firmware/teensy41/.
}

build_firmware_raspberrypi() {
    local pkg=firmware/raspberrypi/scripts/package.sh
    if [[ ! -x "$pkg" ]]; then
        warn "Pi5 package script missing — skipping"
        return
    fi
    log "Packaging Pi5 firmware"
    ( cd firmware/raspberrypi/scripts && ./package.sh "${VERSION}" ) \
        || { warn "Pi5 firmware package failed — leaving existing staged files"; return; }
    if [[ -d firmware/raspberrypi/dist ]]; then
        mkdir -p server/resources/firmware/raspberrypi
        cp -rv firmware/raspberrypi/dist/. server/resources/firmware/raspberrypi/
    fi
}

# Build the Steam Deck controller .AppImage via the linux/amd64 Docker
# wrapper. Slow on first run (Apple Silicon: Rosetta-emulated cargo
# build, ~15-25 min cold); incremental rebuilds are fast once the
# persistent cache warms. Failures here are non-fatal — we'd rather
# ship a dist tarball with the previous controller bundle than fail
# the whole build over an AppImage hiccup.
build_controller_appimage() {
    local driver=controller/appimage/build-docker.sh
    if [[ ! -x "$driver" ]]; then
        warn "Controller AppImage driver $driver missing — skipping"
        return
    fi
    log "Building controller AppImage (linux/amd64 Docker; Rosetta on Apple Silicon)"
    if ! "$driver" 2>&1; then
        warn "Controller AppImage build failed — leaving existing staged bundle (if any)"
        return
    fi
}

if (( SKIP_FIRMWARE )); then
    log "Skipping firmware + controller staging (--skip-firmware)"
    rm -rf server/resources/firmware/*
elif (( FETCH_FIRMWARE )); then
    command -v gh >/dev/null || die "--fetch-firmware needs the gh CLI"
    log "Fetching latest CI firmware + controller artifacts from main"
    rm -rf _fw && mkdir -p _fw
    # Pick the latest successful dist.yml run on main.
    RUN_ID=$(gh run list --workflow=dist.yml --branch=main \
        --status=success --limit=1 --json databaseId -q '.[0].databaseId')
    [[ -n "$RUN_ID" ]] || die "No successful dist.yml runs on main found"
    log "Using run ${RUN_ID}"
    gh run download "$RUN_ID" --dir _fw \
        --name firmware-rp2040 --name firmware-teensy41 --name firmware-raspberrypi \
        --name appimage-controller
    mkdir -p server/resources/firmware/{rp2040,teensy41,raspberrypi,controller}
    [[ -d _fw/firmware-rp2040 ]] && find _fw/firmware-rp2040 -type f \
        \( -name '*.uf2' -o -name '*.elf' \) \
        -exec cp {} server/resources/firmware/rp2040/ \;
    [[ -d _fw/firmware-teensy41 ]] && find _fw/firmware-teensy41 -type f \
        -name '*.hex' -exec cp {} server/resources/firmware/teensy41/ \;
    [[ -d _fw/firmware-raspberrypi ]] && cp -r _fw/firmware-raspberrypi/. server/resources/firmware/raspberrypi/
    # Controller artifact ships the .AppImage + info.json already shaped
    # for the firmware-list endpoint — just copy it over verbatim.
    [[ -d _fw/appimage-controller ]] && cp -r _fw/appimage-controller/. server/resources/firmware/controller/
elif (( SKIP_FIRMWARE_BUILD )); then
    log "Using existing server/resources/firmware/ (--skip-firmware-build)"
else
    log "Building firmware + controller locally so the server tarball contains the latest"
    build_firmware_rp2040
    build_firmware_teensy41
    build_firmware_raspberrypi
    if (( SKIP_CONTROLLER_BUILD )); then
        log "Skipping controller AppImage build (--skip-controller-build); using existing staged bundle if any"
    else
        build_controller_appimage
    fi
fi

# --- bundled apt deps (cached) ---------------------------------------------

DEB_CACHE_DIR="${CACHE_ROOT}/debs-${DEBIAN_RELEASE}-${ARCH}"

# The runtime deb list lives here as a heredoc so we can both (a) feed it
# to the container and (b) hash it to detect when it's changed. If you
# edit this list, the cache invalidates automatically on next run — no
# need for callers to remember --rebundle-debs.
read -r -d '' RUNTIME_DEB_LIST <<'DEB_LIST' || true
libssl3
libtinyxml2-9
libyaml-0-2
libxml2
libacl1
libcurl4
python3
python3-yaml
python3-numpy
python3-aiohttp
python3-websockets
python3-psutil
python3-packaging
python3-lark
python3-empy
python3-catkin-pkg
python3-importlib-metadata
python3-argcomplete
python3-bleak
libpython3.11
python3.11
network-manager
wireless-regdb
rfkill
iw
avahi-daemon
libnss-mdns
dnsmasq
zstd
DEB_LIST

DEB_LIST_HASH=$(printf '%s' "$RUNTIME_DEB_LIST" | shasum -a 256 | awk '{print $1}' | cut -c1-12)
CACHE_SENTINEL="$DEB_CACHE_DIR/.deb-list.sha256"

needs_rebundle=0
if [[ $REBUNDLE_DEBS -eq 1 ]]; then needs_rebundle=1; fi
if [[ ! -d "$DEB_CACHE_DIR" || -z "$(ls -A "$DEB_CACHE_DIR"/*.deb 2>/dev/null)" ]]; then
    needs_rebundle=1
fi
if [[ -d "$DEB_CACHE_DIR" && ( ! -f "$CACHE_SENTINEL" || "$(cat "$CACHE_SENTINEL" 2>/dev/null)" != "$DEB_LIST_HASH" ) ]]; then
    log "Deb list changed since last bundle; cache will be rebuilt"
    needs_rebundle=1
fi

if (( needs_rebundle )); then
    log "Bundling runtime apt deps in debian:${DEBIAN_RELEASE}-slim container (this takes a few minutes)"
    rm -rf "$DEB_CACHE_DIR"
    mkdir -p "$DEB_CACHE_DIR"
    # Pass the deb list to the container via stdin so a single source of
    # truth covers both the hash and the actual install command.
    docker run --rm -i --platform "linux/${ARCH}" \
      -v "${DEB_CACHE_DIR}:/debs" \
      -e RUNTIME_DEB_LIST="${RUNTIME_DEB_LIST}" \
      "debian:${DEBIAN_RELEASE}-slim" \
      bash -eo pipefail <<'CONTAINER_EOF'
export DEBIAN_FRONTEND=noninteractive
apt-get update

# shellcheck disable=SC2086
apt-get install -y --download-only --reinstall ${RUNTIME_DEB_LIST}

cp /var/cache/apt/archives/*.deb /debs/

apt-get install -y --no-install-recommends apt-utils
cd /debs
apt-ftparchive packages . > Packages
gzip -kf Packages
echo "Bundled $(ls -1 *.deb | wc -l) .deb files, $(du -sh . | cut -f1) total"
CONTAINER_EOF
    printf '%s' "$DEB_LIST_HASH" > "$CACHE_SENTINEL"
else
    log "Using cached .deb bundle ($(ls -1 "$DEB_CACHE_DIR"/*.deb | wc -l) packages, deb-list hash ${DEB_LIST_HASH})"
fi

# make-dist.sh expects _debs/ at repo root.
rm -rf _debs
mkdir -p _debs
cp -a "$DEB_CACHE_DIR"/. _debs/

# --- web frontend (Vite build) ---------------------------------------------
#
# Runs on the host. The output is platform-independent static assets, so
# there's no reason to do this inside the arm64 Debian container. setup.py
# globs server/web/dist/ into the colcon install tree, so this step has
# to complete before the colcon container runs below.

if (( SKIP_WEB_BUILD )); then
    if [[ -d server/web/dist && -f server/web/dist/index.html ]]; then
        log "Reusing existing server/web/dist/ (--skip-web-build)"
    else
        warn "--skip-web-build set but server/web/dist/ is empty —"
        warn "the tarball will ship without the Vue UI (legacy.html only)."
    fi
else
    command -v node >/dev/null || die "node is required (or pass --skip-web-build)"
    command -v npm  >/dev/null || die "npm is required (or pass --skip-web-build)"
    log "Building web/dist via Vite"
    ( cd server/web
      # Use `npm ci` when a lockfile exists for reproducible installs;
      # fall back to `npm install` on first build so the tree can be set up.
      if [[ -f package-lock.json ]]; then
          npm ci --no-audit --no-fund
      else
          npm install --no-audit --no-fund
      fi
      npm run build
    )
    [[ -f server/web/dist/index.html ]] \
        || die "Vite build did not produce server/web/dist/index.html"
    log "Web build size: $(du -sh server/web/dist | cut -f1)"
fi

# --- build saint_os against bundled ROS2 -----------------------------------

log "Building saint_os in debian:${DEBIAN_RELEASE} container (3-5 min)"
docker run --rm -i --platform "linux/${ARCH}" \
  -v "${PWD}:/work" \
  -w /work \
  -e ROS_DISTRO="${ROS_DISTRO}" \
  "debian:${DEBIAN_RELEASE}" \
  bash -eo pipefail <<'CONTAINER_EOF'
export DEBIAN_FRONTEND=noninteractive
apt-get update
apt-get install -y --no-install-recommends \
    ca-certificates curl gnupg \
    build-essential cmake git pkg-config \
    python3 python3-dev python3-pip \
    python3-numpy python3-yaml \
    libpython3.11 libssl3 libtinyxml2-dev libxml2-dev libyaml-dev
python3 -m pip install --break-system-packages -U \
    colcon-common-extensions \
    "empy==3.3.4" \
    lark
export LANG=C.UTF-8 LC_ALL=C.UTF-8

mkdir -p "/opt/ros/${ROS_DISTRO}"
cp -a "/work/_ros2/opt/ros/${ROS_DISTRO}/install" "/opt/ros/${ROS_DISTRO}/install"
source "/opt/ros/${ROS_DISTRO}/install/setup.bash"

# Fail loud if sourcing setup.bash didn't actually set the ament
# prefix — almost always a sign of a half-extracted install tree
# (e.g. the per-package local_setup hooks didn't run because the
# top-level local_setup.bash was missing). The outer script's
# EXTRACT_COMPLETE check should catch this earlier, but guard here
# too so the colcon build below dies with a useful message rather
# than the cryptic CMake "Could not find ament_cmake".
[[ -n "${AMENT_PREFIX_PATH:-}" ]] \
    || { echo "ERROR: AMENT_PREFIX_PATH unset after sourcing setup.bash — ROS install is broken" >&2; exit 1; }

# --base-paths server: scope colcon's package discovery to the one
# directory that actually contains the ROS package. Without it colcon
# crawls the whole tree (controller/node_modules, pico-sdk, _ros2/, …)
# and on macOS bind mounts the walk has been observed to silently
# truncate after large subtrees like pico-sdk, ending with
# "ignoring unknown package 'saint_os' in --packages-select" and an
# empty install/. Explicit base-paths is also ~30s faster on a cold
# discovery and keeps CI and local in sync.
colcon build \
    --base-paths server \
    --merge-install \
    --packages-select saint_os \
    --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF

# Fail loud if the build produced no library output — the dist tarball
# is useless without install/lib, but make-dist.sh has been seen to
# happily package an empty install/ tree.
[[ -d install/lib ]] \
    || { echo "ERROR: install/lib missing after colcon build" >&2; exit 1; }
ls install/setup.bash >/dev/null
echo "saint_os build size: $(du -sh install | cut -f1)"
CONTAINER_EOF

# --- assemble tarball -------------------------------------------------------

log "Assembling dist tarball"
scripts/make-dist.sh "${VERSION}" "${ARCH}" "${ROS_DISTRO}"

TARBALL=$(ls -t dist/saint-os_${VERSION}_${ARCH}_${ROS_DISTRO}.tar.zst 2>/dev/null | head -n1)
[[ -f "$TARBALL" ]] || die "make-dist.sh did not produce a tarball"

log "Done."
echo
echo "  Tarball:   ${TARBALL}"
echo "  Size:      $(du -h "$TARBALL" | cut -f1)"
echo "  SHA-256:   $($SHA256 "$TARBALL" | cut -d' ' -f1)"
echo
echo "  Copy to USB and use the dashboard's 'Install from USB' flow, or:"
echo "    scp ${TARBALL} pi@opensaint.local:/tmp/"
# tar -xaf auto-detects compression from the extension — works for both
# .tar.gz (legacy) and .tar.zst (current). Requires GNU tar 1.32+, which
# is on every supported target.
echo "    ssh pi@opensaint.local 'cd /tmp && tar -xaf $(basename "$TARBALL") && sudo $(basename "$TARBALL" .tar.zst)/install.sh'"
echo
