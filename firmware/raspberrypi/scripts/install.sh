#!/bin/bash
#
# SAINT.OS Raspberry Pi Node Installation Script
#
# Installs the SAINT node firmware on any supported Raspberry Pi
# (Pi 3 / Pi 4 / Pi 5). Run with sudo privileges:
#
#   sudo ./install.sh
#
# See firmware/raspberrypi/docs/INSTALL.md for the full guide.

set -e

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Script directory
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
FIRMWARE_DIR="$(dirname "$SCRIPT_DIR")"

echo -e "${GREEN}========================================${NC}"
echo -e "${GREEN}SAINT.OS Raspberry Pi Node Installer${NC}"
echo -e "${GREEN}========================================${NC}"
echo ""

# Check if running as root
if [ "$EUID" -ne 0 ]; then
    echo -e "${RED}Error: This script must be run as root (sudo)${NC}"
    exit 1
fi

# Detect device model. The firmware itself runs on Pi 3 / 4 / 5;
# the installer warns only on non-Pi hosts so a developer testing on
# a desktop x86 box still gets a clear signal.
if [ ! -f /proc/device-tree/model ]; then
    echo -e "${YELLOW}Warning: Cannot detect device model (not running on a Pi?)${NC}"
else
    # /proc/device-tree/model is null-terminated; trim the trailing
    # NUL via `tr` so bash doesn't emit the "ignored null byte" warning.
    MODEL=$(tr -d '\0' < /proc/device-tree/model)
    echo -e "Detected: ${GREEN}$MODEL${NC}"

    if [[ "$MODEL" != *"Raspberry Pi"* ]]; then
        echo -e "${YELLOW}Warning: Not a Raspberry Pi — installer untested here${NC}"
        read -p "Continue anyway? (y/N) " -n 1 -r
        echo
        if [[ ! $REPLY =~ ^[Yy]$ ]]; then
            exit 1
        fi
    fi
fi

# ---------------------------------------------------------------------------
# Offline bundle support — mirrors packaging/install.sh's contract.
#
#   ros2_install/  -- pre-built ROS2 install tree (extracted to
#                     /opt/ros/<distro>/install/). When present we skip the
#                     online ros-apt-source path entirely.
#   deps/          -- .debs + Packages index for runtime apt deps. When
#                     present we wire them up as a temporary file://
#                     apt source so the system-deps step satisfies from
#                     local files — Pi nodes typically have no internet.
#
# Both directories are optional. Without them the installer falls back to
# online apt + ros-apt-source (the prior behaviour).
# ---------------------------------------------------------------------------
# Detect this Pi's Debian release codename. Used to pick the right
# bundled ROS2 install tree + deb cache when the firmware bundle is
# multi-target (carries both bookworm and trixie payloads).
PI_RELEASE=$(. /etc/os-release && echo "${VERSION_CODENAME:-}")
if [ -z "$PI_RELEASE" ]; then
    echo -e "${YELLOW}Warning: /etc/os-release lacks VERSION_CODENAME — assuming bookworm.${NC}"
    PI_RELEASE="bookworm"
fi
echo "Pi Debian release: ${PI_RELEASE}"

# Pick the right deps/ dir from the bundle. Multi-target bundles ship
# deps/<release>/ subdirs; legacy single-target bundles ship a flat
# deps/. Same logic applies to ros2_install/ further down.
resolve_bundle_subdir() {
    local subkey="$1"   # "deps" or "ros2_install"
    local base="$SCRIPT_DIR/../${subkey}"
    if [ -d "${base}/${PI_RELEASE}" ]; then
        echo "${base}/${PI_RELEASE}"
        return 0
    fi
    if [ -d "$base" ]; then
        echo "$base"
        return 0
    fi
    return 1
}

LOCAL_APT_LIST="/etc/apt/sources.list.d/saint-rpi-local.list"
LOCAL_APT_CONF="/etc/apt/apt.conf.d/00saint-rpi-install"
LOCAL_REPO_ENABLED=0
APT_UPDATE_OPTS=()

setup_local_apt_repo() {
    local deps_dir
    deps_dir=$(resolve_bundle_subdir deps) || return
    if ! ls "$deps_dir"/*.deb >/dev/null 2>&1; then
        return
    fi
    # Resolve to an absolute path because the apt source line is
    # file://<abs>; relative paths would be opaque to _apt.
    deps_dir=$(cd "$deps_dir" && pwd)

    echo "Bundled .deb cache found at ${deps_dir} — configuring local apt repo"
    # trusted=yes accepts the unsigned local repo.
    echo "deb [trusted=yes] file://${deps_dir} ./" > "$LOCAL_APT_LIST"

    # apt drops to the _apt user when fetching, which can't traverse
    # /home/<user>/ when the tarball was unpacked there (mode 0700).
    # Disable the sandbox for the install run; cleaned up on exit.
    echo 'APT::Sandbox::User "root";' > "$LOCAL_APT_CONF"
    chmod 0644 "$LOCAL_APT_CONF"

    # Restrict apt-get update to the local source only — avoids hanging
    # on unreachable online mirrors on a Pi without internet.
    APT_UPDATE_OPTS=(
        -o "Dir::Etc::sourcelist=${LOCAL_APT_LIST}"
        -o "Dir::Etc::sourceparts=-"
        -o "APT::Get::List-Cleanup=0"
    )
    LOCAL_REPO_ENABLED=1
}

teardown_local_apt_repo() {
    if [ $LOCAL_REPO_ENABLED -eq 1 ]; then
        [ -f "$LOCAL_APT_LIST" ] && rm -f "$LOCAL_APT_LIST"
        [ -f "$LOCAL_APT_CONF" ] && rm -f "$LOCAL_APT_CONF"
    fi
}
trap teardown_local_apt_repo EXIT

setup_local_apt_repo

echo ""
echo "Step 1: Installing system dependencies..."
# Package names differ between Debian Bookworm (12) and Trixie (13):
#   libgpiod 1.x → libgpiod2 + python3-gpiod          (Bookworm)
#   libgpiod 2.x → libgpiod3 + python3-libgpiod       (Trixie)
# We detect availability via `apt-cache show` and install whichever
# matches this host. The `gpiod` PyPI package (installed in step 3)
# also pulls in working bindings, so a partial apt failure here
# isn't fatal — it just means we fall back to the pip-installed
# version of the Python binding.
if [ $LOCAL_REPO_ENABLED -eq 1 ]; then
    echo "Using bundled .deb repo (no internet required)"
    apt-get update "${APT_UPDATE_OPTS[@]}"
else
    apt-get update
fi

# Critical packages (always install).
CORE_PKGS=(python3-pip python3-venv alsa-utils)

# Pick the gpiod stack that this Debian release ships.
GPIOD_PKGS=()
pkg_available () {
    apt-cache show "$1" >/dev/null 2>&1
}
if pkg_available libgpiod3 && pkg_available python3-libgpiod; then
    GPIOD_PKGS+=(libgpiod3 python3-libgpiod)
    echo "Using Trixie gpiod stack: libgpiod3 + python3-libgpiod"
elif pkg_available libgpiod2 && pkg_available python3-gpiod; then
    GPIOD_PKGS+=(libgpiod2 python3-gpiod)
    echo "Using Bookworm gpiod stack: libgpiod2 + python3-gpiod"
else
    echo -e "${YELLOW}Warning: no system libgpiod / python3-gpiod available — falling back to the pip 'gpiod' package in step 3.${NC}"
fi

apt-get install -y "${CORE_PKGS[@]}" "${GPIOD_PKGS[@]}"

# Optional VLC stack — backs the built-in audio_player peripheral.
# If the apt repo doesn't ship these (some minimal Trixie spins drop
# them), the audio_player driver self-disables with a loud log line
# and the rest of the node keeps running. Install is best-effort:
# we don't fail the whole installer if a non-essential audio package
# is missing.
# Pi-side runtime only — no compile, so libvlc-dev (headers + dev .so
# symlink) is dead weight. Worse, the Pi's rpt apt repo ships
# libvlc5:+rpt-flavored while pure-Debian libvlc-dev pin-depends on the
# unsuffixed libvlc5, so apt refuses the install. python3-vlc is a
# ctypes wrapper — it loads libvlc5 (already installed via vlc-bin) at
# runtime, no headers required.
VLC_PKGS=(vlc-bin python3-vlc)
VLC_AVAILABLE=()
for pkg in "${VLC_PKGS[@]}"; do
    if pkg_available "$pkg"; then
        VLC_AVAILABLE+=("$pkg")
    fi
done
if [ ${#VLC_AVAILABLE[@]} -eq ${#VLC_PKGS[@]} ]; then
    apt-get install -y "${VLC_AVAILABLE[@]}"
else
    missing_vlc=$(comm -23 <(printf '%s\n' "${VLC_PKGS[@]}" | sort) <(printf '%s\n' "${VLC_AVAILABLE[@]}" | sort) | tr '\n' ' ')
    echo -e "${YELLOW}Warning: VLC stack not fully available in apt"
    echo -e "         (missing: ${missing_vlc})"
    if [ "$LOCAL_REPO_ENABLED" -eq 1 ]; then
        echo -e "         Offline install: these .debs aren't in the bundled deps/ repo."
        echo -e "         Re-run firmware/raspberrypi/scripts/bundle-debs.sh --force on the dev box,"
        echo -e "         then re-package — bundle-debs.sh's RUNTIME_DEB_LIST already lists all three."
    fi
    echo -e "         The audio_player peripheral will self-disable on first sync; everything else still works.${NC}"
fi

echo ""
echo "Step 1b: Installing ROS2 runtime libraries..."
# The source-built ROS install tree dlopens these at runtime — Trixie's
# default Pi image doesn't ship most of them. Without them the
# saint-node service starts, loads rclpy, and dies on the first dlopen
# (libtinyxml2.so.<N>, libconsole_bridge, etc.) with a misleading
# "RMW implementation not installed" error.
#
# libtinyxml2 and libpython are release-versioned (-9 vs -11 SO,
# 3.11 vs 3.13 Python ABI). Probe apt for whichever the local repo
# resolves rather than hardcoding the codename.
ROS_RUNTIME_PKGS=(
    libconsole-bridge1.0
    liblttng-ust1
    liborocos-kdl1.5
    libyaml-0-2
    libyaml-cpp0.8
    python3-yaml
    python3-numpy
    python3-lark
    python3-empy
    python3-packaging
    python3-importlib-metadata
    python3-argcomplete
    python3-psutil
    python3-typing-extensions
)
for cand in libtinyxml2-11 libtinyxml2-10 libtinyxml2-9; do
    if pkg_available "$cand"; then
        ROS_RUNTIME_PKGS+=("$cand")
        break
    fi
done
for cand in libpython3.13 libpython3.12 libpython3.11; do
    if pkg_available "$cand"; then
        ROS_RUNTIME_PKGS+=("$cand")
        break
    fi
done

# Filter to what's actually resolvable so a missing-from-bundle package
# doesn't take down the whole step. apt-get install -y fails the entire
# transaction on any unknown package; better to warn and continue with
# what we have, since the service will surface its own clearer error
# at startup if something it needs is absent.
ROS_RUNTIME_AVAILABLE=()
ROS_RUNTIME_MISSING=()
for pkg in "${ROS_RUNTIME_PKGS[@]}"; do
    if pkg_available "$pkg"; then
        ROS_RUNTIME_AVAILABLE+=("$pkg")
    else
        ROS_RUNTIME_MISSING+=("$pkg")
    fi
done
if [ ${#ROS_RUNTIME_MISSING[@]} -gt 0 ]; then
    echo -e "${YELLOW}Warning: ROS runtime libs missing from local repo: ${ROS_RUNTIME_MISSING[*]}${NC}"
    echo -e "${YELLOW}  Service may fail at startup. Re-run bundle-debs.sh --force on the dev box to refresh.${NC}"
fi
if [ ${#ROS_RUNTIME_AVAILABLE[@]} -gt 0 ]; then
    apt-get install -y --no-install-recommends "${ROS_RUNTIME_AVAILABLE[@]}"
fi

echo ""
echo "Step 2: Ensuring ROS2 Kilted is installed..."
# Pin to Kilted — same distro the server and other nodes use
# (project_ros_distro_plan memory: Jazzy → Kilted, now). Don't
# silently fall back to Jazzy or Humble: a mismatched distro
# across nodes leads to micro-ROS protocol drift that's hard to
# diagnose. If apt can't satisfy ros-kilted-ros-base on this host,
# surface that as an error and let the operator decide.
#
# OSRF doesn't ship ROS2 in Debian's default archive; the upstream
# `ros2-apt-source` package wires apt up to packages.ros.org for
# every supported distro+codename (trixie / bookworm / noble /
# jammy). We install that setup package first (idempotent), refresh
# apt, then install ros-kilted-ros-base normally.
ROS_DISTRO="kilted"
ROS_SETUP=""

# Wire up the OSRF apt source if it's not already configured. The
# setup is a tiny .deb that drops the GPG key + sources.list.d entry
# — see github.com/ros-infrastructure/ros-apt-source. Codename comes
# from /etc/os-release (trixie / bookworm / noble / jammy).
_setup_ros_apt_source() {
    # Already configured?
    if ls /etc/apt/sources.list.d/ros2*.list 2>/dev/null | grep -q . \
       || ls /etc/apt/sources.list.d/ros*.sources 2>/dev/null | grep -q .; then
        return 0
    fi

    # Need curl + ca-certs to fetch the .deb. Pre-installed on most
    # Pi OS / Ubuntu hosts, but safe to ensure.
    apt-get install -y curl ca-certificates

    # Codename detection (trixie / bookworm / etc).
    local codename
    codename=$(. /etc/os-release && echo "${UBUNTU_CODENAME:-${VERSION_CODENAME:-}}")
    if [ -z "$codename" ]; then
        echo -e "${RED}Could not detect OS codename from /etc/os-release${NC}"
        return 1
    fi

    # Fetch latest ros-apt-source release tag from GitHub. Falls
    # back to a pinned version if the API isn't reachable (offline
    # install, GitHub rate-limit, etc.).
    local apt_source_ver
    apt_source_ver=$(curl -fsSL https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest 2>/dev/null \
        | awk -F'"' '/"tag_name":/ {print $4; exit}')
    if [ -z "$apt_source_ver" ]; then
        # 1.2.0 is the first release with a Trixie .deb (Apr 2026).
        # Don't fall back to 1.1.0 — it lacks the trixie asset and the
        # download will 404, masking the real "no internet" failure.
        apt_source_ver="1.2.0"
        echo -e "${YELLOW}GitHub API unreachable — falling back to ros-apt-source ${apt_source_ver}${NC}"
    fi

    local deb_url="https://github.com/ros-infrastructure/ros-apt-source/releases/download/${apt_source_ver}/ros2-apt-source_${apt_source_ver}.${codename}_all.deb"
    local deb_path
    deb_path=$(mktemp --suffix=.deb)
    echo "Downloading ros2-apt-source for ${codename}…"
    if ! curl -fsSL -o "$deb_path" "$deb_url"; then
        echo -e "${RED}Failed to download ros2-apt-source for codename '${codename}' (${deb_url})${NC}"
        echo -e "${YELLOW}If your OS isn't supported by upstream releases, install ROS2 manually.${NC}"
        rm -f "$deb_path"
        return 1
    fi
    apt-get install -y "$deb_path"
    rm -f "$deb_path"

    # Refresh apt now that OSRF's source is live.
    apt-get update
}

# Resolve ROS distro from the bundled manifest if present (the
# packager writes it there). Falls back to the hardcoded default
# above; either way, ROS_DISTRO must match the bundled ros2_install
# tree.
if [ -f "$SCRIPT_DIR/../manifest.json" ]; then
    manifest_distro=$(python3 -c "import json,sys; print(json.load(open('$SCRIPT_DIR/../manifest.json')).get('ros_distro') or '')" 2>/dev/null || true)
    if [ -n "$manifest_distro" ]; then
        ROS_DISTRO="$manifest_distro"
    fi
fi

# ROS2 install layouts vary by how it got installed:
#   - apt-installed (ros-${ROS_DISTRO}-ros-base): setup.bash lives at
#       /opt/ros/${ROS_DISTRO}/setup.bash
#   - rsync'd from a build container's install tree (server's _ros2/):
#       /opt/ros/${ROS_DISTRO}/install/setup.bash      (build embeds
#       absolute paths into the file; the install/ subdir is required)
# We probe both, set ROS_SETUP to whichever exists, and pass that
# explicit path to the systemd service template at the end.
_find_ros_setup() {
    if [ -f "/opt/ros/${ROS_DISTRO}/install/setup.bash" ]; then
        echo "/opt/ros/${ROS_DISTRO}/install/setup.bash"
        return 0
    fi
    if [ -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]; then
        echo "/opt/ros/${ROS_DISTRO}/setup.bash"
        return 0
    fi
    return 1
}

# Verify that a ROS install at <prefix>/lib/python<X.Y>/site-packages
# matches the system's Python. The bundled tree carries C extensions
# tagged with the cpython ABI tag (e.g. cpython-311 / cpython-313)
# that they were compiled against — if the system's Python is a
# different minor version, `import rclpy` blows up with a misleading
# "_rclpy_pybind11 isn't present on the system" error. Catch this
# at install time instead of letting the systemd unit crash-loop.
#
# `prefix` is /opt/ros/<distro>/install or /opt/ros/<distro> — either
# layout has lib/python<X.Y>/site-packages directly under it.
_ros_install_python_ok() {
    local prefix=$1
    local sys_py
    sys_py=$(python3 -c "import sys; print(f'python{sys.version_info.major}.{sys.version_info.minor}')")
    if [ -d "${prefix}/lib/${sys_py}/site-packages/rclpy" ]; then
        return 0
    fi
    # Surface what we found vs what we need so the error is debuggable.
    local found
    found=$(find "${prefix}/lib" -maxdepth 1 -name 'python3.*' -type d 2>/dev/null \
            | xargs -I{} basename {} | paste -sd ',' -)
    echo -e "${YELLOW}ROS2 tree at ${prefix} is built for [${found:-unknown}] but this system runs ${sys_py}.${NC}" >&2
    echo -e "${YELLOW}Python C extensions in the bundle are ABI-tagged; mixing 3.11 with 3.13 fails at rclpy import.${NC}" >&2
    return 1
}

if ROS_SETUP=$(_find_ros_setup); then
    # An install already exists — but does it match this Pi's Python?
    # If not, wipe and reinstall so the bad bundle from a previous
    # cross-release attempt doesn't keep crashing the service.
    existing_prefix="${ROS_SETUP%/setup.bash}"
    if _ros_install_python_ok "$existing_prefix"; then
        echo -e "Found existing ROS2 ${GREEN}${ROS_DISTRO}${NC} at ${ROS_SETUP}"
    else
        echo -e "${YELLOW}Existing ROS2 install at ${existing_prefix} is the wrong Python ABI — wiping.${NC}"
        rm -rf "/opt/ros/${ROS_DISTRO}"
        ROS_SETUP=""
    fi
fi

if [ -z "${ROS_SETUP:-}" ]; then
    # Bundled tree: resolve release-specific subdir first
    # (ros2_install/<release>/ from multi-target bundles), then the
    # flat ros2_install/ from legacy single-target bundles. Whichever
    # exists must have the right Python ABI for this host — install
    # bombed out earlier if not.
    bundled_ros2=""
    if [ -d "$SCRIPT_DIR/../ros2_install/${PI_RELEASE}" ]; then
        bundled_ros2="$SCRIPT_DIR/../ros2_install/${PI_RELEASE}"
        echo "Multi-target bundle: using ros2_install/${PI_RELEASE}/ for this Pi"
    elif [ -d "$SCRIPT_DIR/../ros2_install" ] && [ -f "$SCRIPT_DIR/../ros2_install/setup.bash" ]; then
        bundled_ros2="$SCRIPT_DIR/../ros2_install"
    fi

    if [ -n "$bundled_ros2" ]; then
        # Mirrors packaging/install.sh's behaviour: rsync into
        # /opt/ros/<distro>/install/ (the path the build embedded into
        # setup.bash's absolute references).
        if ! _ros_install_python_ok "$bundled_ros2"; then
            echo -e "${YELLOW}Skipping bundled ros2_install/ and falling through to apt.${NC}"
            # Don't set ROS_SETUP; the apt branch below picks up the slack.
        else
            ROS_PARENT="/opt/ros/${ROS_DISTRO}"
            ROS_PREFIX="${ROS_PARENT}/install"
            echo -e "Installing ROS2 ${GREEN}${ROS_DISTRO}${NC} from bundled tree…"
            # If a previous install left ROS_PARENT in the apt (flat) layout
            # (setup.bash directly under /opt/ros/<distro>/), wipe before
            # syncing so the install/ subdir takes over cleanly.
            if [ -f "${ROS_PARENT}/setup.bash" ] && [ ! -d "$ROS_PREFIX" ]; then
                echo "Older ROS2 layout under ${ROS_PARENT} — wiping before sync"
                rm -rf "$ROS_PARENT"
            fi
            install -d "$ROS_PREFIX"
            rsync -a --delete "$bundled_ros2/" "$ROS_PREFIX/" \
                || { echo -e "${RED}Failed to copy bundled ROS2 tree${NC}"; exit 1; }
            if [ ! -f "$ROS_PREFIX/setup.bash" ]; then
                echo -e "${RED}Bundled ros2 tree at ${bundled_ros2} has no setup.bash${NC}"
                echo -e "${YELLOW}Make sure the bundle contains the install/ subtree directly,"
                echo -e "not a wrapping directory. The packager copies _ros2*/opt/ros/<distro>/install/ verbatim.${NC}"
                exit 1
            fi
            ROS_SETUP="$ROS_PREFIX/setup.bash"
        fi
    fi
fi

if [ -z "${ROS_SETUP:-}" ]; then
    # Apt path. Reached when there's no usable bundle (none present, or
    # bundled tree's Python ABI doesn't match this host). Resolves
    # ros-${ROS_DISTRO}-ros-base — preferentially from the bundled .deb
    # repo set up in setup_local_apt_repo, otherwise from the upstream
    # OSRF source. Either path produces Trixie-native (python3.13-correct)
    # binaries on a Trixie Pi.
    for other in jazzy humble iron; do
        if [ -f "/opt/ros/${other}/setup.bash" ] || [ -f "/opt/ros/${other}/install/setup.bash" ]; then
            echo -e "${YELLOW}Warning: ROS2 ${other} is installed at /opt/ros/${other} but the fleet runs ${ROS_DISTRO}.${NC}"
            echo -e "${YELLOW}The installer will add ${ROS_DISTRO} alongside it; consider removing the old distro after install.${NC}"
        fi
    done

    if ! pkg_available "ros-${ROS_DISTRO}-ros-base"; then
        # When the local repo is enabled we're in OFFLINE mode by
        # contract — the operator built a bundle precisely because the
        # Pi has no internet. Falling through to OSRF here just times
        # out on DNS and produces a misleading error. Fail clearly with
        # the exact remediation steps instead.
        if [ "$LOCAL_REPO_ENABLED" -eq 1 ]; then
            echo -e "${RED}ros-${ROS_DISTRO}-ros-base is missing from the bundled deps/ repo.${NC}" >&2
            echo "" >&2
            echo -e "${YELLOW}The bundled .deb cache was packaged WITHOUT ros-${ROS_DISTRO}-* debs.${NC}" >&2
            echo -e "${YELLOW}This usually means one of:${NC}" >&2
            echo -e "  1) firmware/raspberrypi/scripts/bundle-debs.sh wasn't run before package.sh —" >&2
            echo -e "     package.sh fell back to the server's _debs/ cache, which doesn't have ROS2." >&2
            echo -e "  2) bundle-debs.sh ran but its cache is stale (RUNTIME_DEB_LIST changed since)." >&2
            echo "" >&2
            echo -e "${YELLOW}Fix on the dev box, then re-package + re-install:${NC}" >&2
            echo -e "  firmware/raspberrypi/scripts/bundle-debs.sh --force" >&2
            echo -e "  firmware/raspberrypi/scripts/package.sh" >&2
            echo "" >&2
            echo -e "${YELLOW}Verify _rpi_debs/ contains ros-${ROS_DISTRO}-ros-base*.deb before re-packaging:${NC}" >&2
            echo -e "  ls _rpi_debs/ros-${ROS_DISTRO}-ros-base*.deb" >&2
            exit 1
        fi
        echo "No bundled ros2_install/ and ros-${ROS_DISTRO}-ros-base not in apt — configuring OSRF repo…"
        if ! _setup_ros_apt_source; then
            echo -e "${RED}Failed to set up OSRF apt repo.${NC}"
            echo -e "${YELLOW}This Pi probably has no internet. Re-build the firmware bundle with a"
            echo -e "deps/ payload that includes ros-${ROS_DISTRO}-ros-base — see firmware/raspberrypi/scripts/bundle-debs.sh.${NC}"
            exit 1
        fi
    fi

    if ! pkg_available "ros-${ROS_DISTRO}-ros-base"; then
        echo -e "${RED}ros-${ROS_DISTRO}-ros-base is not available for this OS codename.${NC}"
        echo -e "${YELLOW}Check https://docs.ros.org/en/${ROS_DISTRO}/Installation.html for supported releases.${NC}"
        exit 1
    fi

    echo -e "Installing ROS2 ${GREEN}${ROS_DISTRO}${NC} (ros-${ROS_DISTRO}-ros-base)…"
    apt-get install -y "ros-${ROS_DISTRO}-ros-base"
    ROS_SETUP=$(_find_ros_setup) || {
        echo -e "${RED}apt installed ros-${ROS_DISTRO}-ros-base but no setup.bash on disk${NC}"
        exit 1
    }
fi

# Final sanity check — the install must actually be importable. Catches
# any remaining ABI mismatch (e.g. apt path also picked up an
# unexpected variant) before we hand off to systemd.
if ! _ros_install_python_ok "${ROS_SETUP%/setup.bash}"; then
    echo -e "${RED}ROS2 install at ${ROS_SETUP%/setup.bash} doesn't match this system's Python ABI.${NC}"
    echo -e "${YELLOW}Reinstall on this Pi, or re-bundle with a ros2_install/ tree built for the right Debian release.${NC}"
    exit 1
fi

echo "Using ROS setup: ${ROS_SETUP}"

# Source ROS2 so subsequent steps see colcon / ros2 commands.
# `set +u` is precautionary — ROS2's setup.bash references unset
# env vars during normal operation. The script doesn't enable
# `set -u` by default, but a future tweak might; keep this so the
# installer survives that change.
set +u
# shellcheck disable=SC1090
source "$ROS_SETUP"

echo ""
echo "Step 3: Installing Python dependencies..."
# python-vlc is installed via apt (python3-vlc above) so this pip
# step covers only what apt doesn't ship.
pip3 install --break-system-packages pyyaml gpiod || pip3 install pyyaml gpiod

echo ""
echo "Step 4: Installing SAINT node package..."

# Create installation directory
INSTALL_DIR="/opt/saint-node"
mkdir -p "$INSTALL_DIR"

# Copy node package
cp -r "$FIRMWARE_DIR/saint_node" "$INSTALL_DIR/"

# Create symlink for Python module access
SITE_PACKAGES=$(python3 -c "import site; print(site.getsitepackages()[0])")
ln -sf "$INSTALL_DIR/saint_node" "$SITE_PACKAGES/saint_node" 2>/dev/null || \
    cp -r "$INSTALL_DIR/saint_node" "$SITE_PACKAGES/"

echo ""
echo "Step 5: Creating configuration + state directories..."
mkdir -p /etc/saint-node
chmod 755 /etc/saint-node
# Audio library: the built-in audio_player peripheral plays files
# from this folder. Default path matches the rpi5 board YAML's
# `library_path` param — operators drop .wav / .mp3 / .flac here.
mkdir -p /var/lib/saint-os/audio
chmod 755 /var/lib/saint-os/audio

echo ""
echo "Step 6: Installing systemd service..."
# Render the service template, substituting @ROS_DISTRO@ + @ROS_SETUP@
# with what Step 2 actually resolved. Same pattern as the server
# installer (packaging/install.sh). Using @ROS_SETUP@ — the full
# absolute path — instead of just the distro lets the service work
# regardless of whether the setup.bash is at /opt/ros/<distro>/ (apt)
# or /opt/ros/<distro>/install/ (bundled-tree).
sed \
    -e "s|@ROS_DISTRO@|${ROS_DISTRO}|g" \
    -e "s|@ROS_SETUP@|${ROS_SETUP}|g" \
    "$FIRMWARE_DIR/config/saint-node.service" > /etc/systemd/system/saint-node.service
chmod 644 /etc/systemd/system/saint-node.service

systemctl daemon-reload

# Enable boot-time start AND start (or restart, if already running)
# now. Matches the server installer's behaviour — no operator step
# needed to bring the node up post-install. Wrapped in
# reset-failed so a previously-crashed install doesn't block the
# restart with "Unit is in failed state."
systemctl reset-failed saint-node.service 2>/dev/null || true
systemctl enable saint-node.service
systemctl restart saint-node.service
# Brief settle pause so the status snapshot at the end of the
# installer reflects the actual post-start state (active vs
# failed) rather than the transient "activating" tick.
sleep 2
SAINT_NODE_ACTIVE=0
if systemctl is-active --quiet saint-node.service; then
    SAINT_NODE_ACTIVE=1
fi

echo ""
echo "Step 7: Enabling GPIO permissions..."

# Add user to gpio group if it exists
if getent group gpio > /dev/null; then
    usermod -aG gpio root
fi

# Create udev rule for GPIO access
cat > /etc/udev/rules.d/99-saint-gpio.rules << 'EOF'
# Allow GPIO access for SAINT node
SUBSYSTEM=="gpio", KERNEL=="gpiochip*", MODE="0660", GROUP="gpio"
EOF

udevadm control --reload-rules
udevadm trigger

echo ""
echo "Step 8: Installing SAINT.OS Plymouth boot splash (optional)..."
# The bundled theme lives at $SCRIPT_DIR/../assets/plymouth/saint-os/.
# We only install it when (a) the theme dir is present in the bundle
# AND (b) splash.png is non-empty — otherwise plymouth-set-default-theme
# leaves the screen blank during boot. Plymouth itself needs to be
# present too (apt package: plymouth); skip with a warning rather than
# failing the whole installer if the operator didn't preinstall it.
PLYMOUTH_THEME_SRC="$SCRIPT_DIR/../assets/plymouth/saint-os"
PLYMOUTH_THEME_DST="/usr/share/plymouth/themes/saint-os"
# Logo file: opensaint.png — bundled by package.sh from
# firmware/raspberrypi/assets/opensaint.png (the canonical brand asset
# that also drives the SPA splash). The Plymouth script references it
# by that exact filename.
if [ -d "$PLYMOUTH_THEME_SRC" ] \
   && [ -s "$PLYMOUTH_THEME_SRC/opensaint.png" ] \
   && command -v plymouth-set-default-theme >/dev/null 2>&1; then
    echo "Installing SAINT.OS Plymouth theme → ${PLYMOUTH_THEME_DST}"
    mkdir -p "$PLYMOUTH_THEME_DST"
    cp -a "$PLYMOUTH_THEME_SRC/." "$PLYMOUTH_THEME_DST/"
    # -R rebuilds the initramfs so the theme also covers the very
    # early boot phase (before systemd takes over). Without -R the
    # splash only kicks in mid-boot, which defeats the point.
    if plymouth-set-default-theme -R saint-os; then
        echo -e "${GREEN}SAINT.OS boot splash enabled.${NC}"
        echo "  (Reboot to see it. To revert: sudo plymouth-set-default-theme -R pix)"
    else
        echo -e "${YELLOW}plymouth-set-default-theme failed — splash files copied"
        echo -e "but the active theme wasn't changed. Run manually if you want it on.${NC}"
    fi
elif [ -d "$PLYMOUTH_THEME_SRC" ] && [ ! -s "$PLYMOUTH_THEME_SRC/opensaint.png" ]; then
    echo -e "${YELLOW}Skipping Plymouth theme — assets/plymouth/saint-os/opensaint.png is missing"
    echo -e "or empty. Add a PNG at firmware/raspberrypi/assets/opensaint.png and"
    echo -e "re-package to enable the SAINT.OS boot splash.${NC}"
elif [ -d "$PLYMOUTH_THEME_SRC" ]; then
    echo -e "${YELLOW}Skipping Plymouth theme — plymouth-set-default-theme not found."
    echo -e "Install plymouth + plymouth-themes apt packages first.${NC}"
fi

echo ""
echo -e "${GREEN}========================================${NC}"
echo -e "${GREEN}Installation Complete!${NC}"
echo -e "${GREEN}========================================${NC}"
echo ""
if [ "$SAINT_NODE_ACTIVE" -eq 1 ]; then
    echo -e "Service: ${GREEN}saint-node is active${NC} (and enabled on boot)"
    echo ""
    echo "The node should appear in the server's Unadopted Nodes panel within a"
    echo "few seconds. If it doesn't, follow up with:"
    echo "  journalctl -u saint-node -f"
else
    echo -e "Service: ${RED}saint-node failed to start${NC} (but is enabled on boot)"
    echo ""
    echo "See what went wrong:"
    echo "  systemctl status saint-node"
    echo "  journalctl -u saint-node -n 100"
fi
echo ""
echo "Configuration files are stored in: /etc/saint-node/"
echo "Audio library:                     /var/lib/saint-os/audio/"
echo ""
echo "See firmware/raspberrypi/docs/INSTALL.md for OTA + troubleshooting."
echo ""
