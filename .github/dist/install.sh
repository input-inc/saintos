#!/usr/bin/env bash
#
# SAINT.OS Server installer.
#
# Run as root on a fresh Ubuntu 22.04 (arm64) host or Raspberry Pi 4/5
# running Ubuntu Server 22.04. Reads manifest.json to determine the
# bundled ROS2 distribution.
#
# Usage:
#   sudo ./install.sh                 # install + enable + start
#   sudo ./install.sh --no-start      # install + enable, don't start
#   sudo ./install.sh --dry-run       # show what would happen
#

set -euo pipefail

PAYLOAD_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

PREFIX="/opt/saint-os"
CONFIG_DIR="/etc/saint-os"
STATE_DIR="/var/lib/saint-os"
LOG_DIR="/var/log/saint-os"
SERVICE_USER="saint"
SERVICE_NAME="saint-os"

NO_START=0
DRY_RUN=0
for arg in "$@"; do
    case "$arg" in
        --no-start) NO_START=1 ;;
        --dry-run)  DRY_RUN=1 ;;
        -h|--help)
            sed -n '2,12p' "$0"
            exit 0
            ;;
        *) echo "Unknown option: $arg" >&2; exit 1 ;;
    esac
done

run() {
    if (( DRY_RUN )); then
        echo "+ $*"
    else
        "$@"
    fi
}

log() { printf '\033[1;32m[install]\033[0m %s\n' "$*"; }
warn() { printf '\033[1;33m[install]\033[0m %s\n' "$*" >&2; }
die() { printf '\033[1;31m[install]\033[0m %s\n' "$*" >&2; exit 1; }

# --- Preflight ---------------------------------------------------------------

[[ $EUID -eq 0 ]] || die "Must run as root (use sudo)"
[[ -f "${PAYLOAD_DIR}/manifest.json" ]] || die "manifest.json not found next to install.sh"

# Manifest fields.
VERSION=$(python3 -c "import json,sys; print(json.load(open('${PAYLOAD_DIR}/manifest.json'))['version'])")
ARCH=$(python3 -c "import json,sys; print(json.load(open('${PAYLOAD_DIR}/manifest.json'))['arch'])")
ROS_DISTRO=$(python3 -c "import json,sys; print(json.load(open('${PAYLOAD_DIR}/manifest.json'))['ros_distro'])")

HOST_ARCH=$(dpkg --print-architecture)
if [[ "$HOST_ARCH" != "$ARCH" ]]; then
    die "Architecture mismatch: package is ${ARCH}, host is ${HOST_ARCH}"
fi

if ! grep -q '^ID=ubuntu' /etc/os-release; then
    warn "This installer targets Ubuntu. Detected: $(grep '^PRETTY_NAME' /etc/os-release | cut -d= -f2)"
fi

log "Installing SAINT.OS ${VERSION} (${ARCH}, ros-${ROS_DISTRO})"

# --- apt prereqs + ROS2 ------------------------------------------------------

ROS_SETUP="/opt/ros/${ROS_DISTRO}/setup.bash"

if [[ ! -f "$ROS_SETUP" ]]; then
    log "Installing ROS2 ${ROS_DISTRO}..."
    run apt-get update
    run apt-get install -y curl gnupg lsb-release software-properties-common
    run add-apt-repository -y universe

    if [[ ! -f /usr/share/keyrings/ros-archive-keyring.gpg ]]; then
        run bash -c "curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
            -o /usr/share/keyrings/ros-archive-keyring.gpg"
    fi

    CODENAME=$(. /etc/os-release && echo "$UBUNTU_CODENAME")
    LIST_FILE=/etc/apt/sources.list.d/ros2.list
    if [[ ! -f "$LIST_FILE" ]]; then
        run bash -c "echo 'deb [arch=${HOST_ARCH} signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
http://packages.ros.org/ros2/ubuntu ${CODENAME} main' > ${LIST_FILE}"
    fi

    run apt-get update
    run apt-get install -y "ros-${ROS_DISTRO}-ros-base"
else
    log "Found existing ROS2 ${ROS_DISTRO} at ${ROS_SETUP}"
fi

# Runtime Python deps via apt (avoids PEP 668 break-system-packages dance).
log "Installing Python runtime dependencies..."
run apt-get install -y \
    python3-aiohttp \
    python3-websockets \
    python3-yaml \
    python3-numpy \
    python3-psutil

# --- User + directories ------------------------------------------------------

if ! id -u "${SERVICE_USER}" >/dev/null 2>&1; then
    log "Creating system user ${SERVICE_USER}"
    run useradd --system --create-home --home-dir "${STATE_DIR}" \
        --shell /usr/sbin/nologin "${SERVICE_USER}"
fi

# Add to dialout/gpio if present (for RC receiver UART / GPIO).
for grp in dialout gpio; do
    if getent group "$grp" >/dev/null; then
        run usermod -aG "$grp" "${SERVICE_USER}"
    fi
done

run install -d -m 0755 -o "${SERVICE_USER}" -g "${SERVICE_USER}" \
    "${PREFIX}" "${CONFIG_DIR}" "${STATE_DIR}" "${LOG_DIR}"

# --- Copy payload ------------------------------------------------------------

log "Copying payload to ${PREFIX}"
run rm -rf "${PREFIX}/install"
run cp -a "${PAYLOAD_DIR}/ros_install" "${PREFIX}/install"

if [[ -d "${PAYLOAD_DIR}/firmware" ]]; then
    run rm -rf "${PREFIX}/firmware"
    run cp -a "${PAYLOAD_DIR}/firmware" "${PREFIX}/firmware"

    # http_server.py resolves firmware_root from the package's site-packages
    # parent. Link the bundled firmware into that location so the default
    # path-finding works. This is a workaround until firmware_root becomes
    # a server parameter.
    SITE_PKGS=$(find "${PREFIX}/install/lib" -maxdepth 2 -name site-packages -type d | head -n1)
    if [[ -n "$SITE_PKGS" ]]; then
        run install -d "${SITE_PKGS}/resources"
        run ln -sfn "${PREFIX}/firmware" "${SITE_PKGS}/resources/firmware"
    else
        warn "Could not locate site-packages under ${PREFIX}/install/lib — firmware OTA may not find files"
    fi
fi

# Default configs into /etc/saint-os on first install only.
if [[ -d "${PREFIX}/install/share/saint_os/config" ]]; then
    for cfg in "${PREFIX}/install/share/saint_os/config"/*.yaml; do
        [[ -e "$cfg" ]] || continue
        dest="${CONFIG_DIR}/$(basename "$cfg")"
        if [[ ! -e "$dest" ]]; then
            log "Seeding default config: ${dest}"
            run install -m 0644 -o "${SERVICE_USER}" -g "${SERVICE_USER}" "$cfg" "$dest"
        fi
    done
fi

run chown -R "${SERVICE_USER}:${SERVICE_USER}" "${PREFIX}"

# --- systemd unit ------------------------------------------------------------

UNIT_PATH="/etc/systemd/system/${SERVICE_NAME}.service"
log "Installing systemd unit: ${UNIT_PATH}"

UNIT_SRC="${PAYLOAD_DIR}/systemd/${SERVICE_NAME}.service"
[[ -f "$UNIT_SRC" ]] || die "Missing systemd unit at ${UNIT_SRC}"

if (( DRY_RUN )); then
    echo "+ render ${UNIT_SRC} -> ${UNIT_PATH} with ROS_DISTRO=${ROS_DISTRO}"
else
    sed \
        -e "s|@ROS_DISTRO@|${ROS_DISTRO}|g" \
        -e "s|@PREFIX@|${PREFIX}|g" \
        -e "s|@CONFIG_DIR@|${CONFIG_DIR}|g" \
        -e "s|@STATE_DIR@|${STATE_DIR}|g" \
        -e "s|@LOG_DIR@|${LOG_DIR}|g" \
        -e "s|@USER@|${SERVICE_USER}|g" \
        "$UNIT_SRC" > "$UNIT_PATH"
    chmod 0644 "$UNIT_PATH"
fi

run systemctl daemon-reload
run systemctl enable "${SERVICE_NAME}.service"

if (( NO_START )); then
    log "Skipping start (--no-start). Start manually with: systemctl start ${SERVICE_NAME}"
else
    log "Starting ${SERVICE_NAME}..."
    run systemctl restart "${SERVICE_NAME}.service"
fi

# --- Summary -----------------------------------------------------------------

cat <<EOF

  SAINT.OS ${VERSION} installed.

  Status:    systemctl status ${SERVICE_NAME}
  Logs:      journalctl -u ${SERVICE_NAME} -f
  Configs:   ${CONFIG_DIR}/
  Payload:   ${PREFIX}/
  Web UI:    http://\$(hostname -I | awk '{print \$1}')/

EOF
