#!/usr/bin/env bash
#
# SAINT.OS Server installer.
#
# Self-contained: ROS2 (jazzy + micro-ROS agent) is bundled in the payload
# and extracted to /opt/ros/<distro>/ — no apt repos to configure.
#
# Supported targets (arm64):
#   - Raspberry Pi OS Bookworm (Pi 4 / Pi 5)
#   - Ubuntu 24.04 noble
#
# Usage:
#   sudo ./install.sh                 # install + enable + start (+ WiFi AP)
#   sudo ./install.sh --no-start      # install + enable, don't start
#   sudo ./install.sh --no-wifi       # don't configure the WiFi access point
#   sudo ./install.sh --dry-run       # show what would happen
#
# WiFi AP overrides (set before invoking):
#   SAINT_WIFI_SSID    (default: OpenSAINT)
#   SAINT_WIFI_PASS    (default: ifeelalive)
#   SAINT_WIFI_COUNTRY (default: US)
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
NO_WIFI=0

# WiFi AP defaults — override via env vars before running install.sh.
WIFI_SSID="${SAINT_WIFI_SSID:-OpenSAINT}"
WIFI_PASS="${SAINT_WIFI_PASS:-ifeelalive}"
WIFI_COUNTRY="${SAINT_WIFI_COUNTRY:-US}"

for arg in "$@"; do
    case "$arg" in
        --no-start) NO_START=1 ;;
        --no-wifi)  NO_WIFI=1 ;;
        --dry-run)  DRY_RUN=1 ;;
        -h|--help)
            sed -n '2,18p' "$0"
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

# Detect target distro family. The bundled ROS2 was built against Debian
# Bookworm (libstdc++ from gcc-12, libpython3.11, glibc 2.36), which is
# binary-compatible with Pi OS Bookworm directly, and with Ubuntu 24.04
# noble after libpython3.11 is installed from universe.
. /etc/os-release
OS_ID="${ID:-unknown}"
OS_VERSION_ID="${VERSION_ID:-unknown}"
OS_PRETTY="${PRETTY_NAME:-${OS_ID} ${OS_VERSION_ID}}"
case "${OS_ID}" in
    debian|raspbian)
        OS_FAMILY=debian ;;
    ubuntu)
        OS_FAMILY=ubuntu ;;
    *)
        OS_FAMILY=unknown
        warn "Unsupported OS: ${OS_PRETTY} — proceeding, but YMMV"
        ;;
esac

log "Installing SAINT.OS ${VERSION} (${ARCH}, ros-${ROS_DISTRO}) on ${OS_PRETTY}"

# --- Runtime apt dependencies -----------------------------------------------

# Common deps that exist with the same name on both Debian Bookworm and
# Ubuntu noble.
COMMON_DEPS=(
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
)

# Python 3.11 specifically: the bundled ROS2 was built against libpython3.11.
# Bookworm has it as the default; Ubuntu noble (default 3.12) ships it in
# universe and apt installs it alongside.
PY311_DEPS=(libpython3.11 python3.11)

log "Installing runtime dependencies via apt"
run apt-get update
if [[ "${OS_FAMILY}" == "ubuntu" ]]; then
    run apt-get install -y software-properties-common
    run add-apt-repository -y universe
    run apt-get update
fi
run apt-get install -y "${COMMON_DEPS[@]}" "${PY311_DEPS[@]}"

# --- Extract bundled ROS2 ---------------------------------------------------

ROS_PREFIX="/opt/ros/${ROS_DISTRO}"
if [[ ! -d "${PAYLOAD_DIR}/ros2_install" ]]; then
    die "Bundled ROS2 not found at ${PAYLOAD_DIR}/ros2_install"
fi
log "Extracting bundled ROS2 to ${ROS_PREFIX}"
run install -d "${ROS_PREFIX}"
# Wipe any prior install to avoid stale files mixing with the new tree.
run rm -rf "${ROS_PREFIX:?}"/*
run cp -a "${PAYLOAD_DIR}/ros2_install/." "${ROS_PREFIX}/"

if [[ ! -f "${ROS_PREFIX}/setup.bash" ]]; then
    die "ROS2 setup.bash not found at ${ROS_PREFIX}/setup.bash after extract"
fi

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

log "Copying saint_os payload to ${PREFIX}"
run rm -rf "${PREFIX}/install"
run cp -a "${PAYLOAD_DIR}/saint_install" "${PREFIX}/install"

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

# --- WiFi access point -------------------------------------------------------

setup_wifi_ap() {
    if (( NO_WIFI )); then
        log "Skipping WiFi AP setup (--no-wifi)"
        return
    fi

    # Detect a wireless interface. If none, the host probably has no WiFi
    # (e.g. an x86 dev box) — skip without erroring.
    local wlan
    if command -v iw >/dev/null 2>&1; then
        wlan=$(iw dev 2>/dev/null | awk '$1=="Interface"{print $2; exit}')
    fi
    if [[ -z "${wlan:-}" ]]; then
        # Fallback: scan /sys for any wireless interface.
        for dev in /sys/class/net/*/wireless; do
            [[ -e "$dev" ]] || continue
            wlan=$(basename "$(dirname "$dev")")
            break
        done
    fi
    if [[ -z "${wlan:-}" ]]; then
        warn "No wireless interface detected — skipping WiFi AP setup"
        return
    fi
    log "Configuring WiFi AP on ${wlan} (SSID=${WIFI_SSID})"

    # Install NetworkManager + the regulatory DB (Pi WiFi refuses AP mode
    # without a recognized country) + rfkill (Pi often soft-blocks WiFi by
    # default until rfkill unblock is run).
    local nm_pkgs=(network-manager wireless-regdb rfkill iw)
    local need_install=0
    for pkg in "${nm_pkgs[@]}"; do
        dpkg -s "$pkg" >/dev/null 2>&1 || need_install=1
    done
    if (( need_install )); then
        log "Installing NetworkManager + WiFi support packages"
        run apt-get install -y "${nm_pkgs[@]}"
    fi
    run systemctl enable --now NetworkManager

    # Pi WiFi is frequently soft-blocked until explicitly unblocked.
    run rfkill unblock wifi || true

    # Ensure netplan delegates the wifi interface to NetworkManager. On
    # Ubuntu Server, systemd-networkd will otherwise hold wlan0 and NM won't
    # touch it. The 99-* prefix ensures we override anything cloud-init
    # generated.
    #
    # On Raspberry Pi OS (Bookworm+) netplan isn't installed — NetworkManager
    # manages interfaces directly — so /etc/netplan doesn't exist and we
    # skip this step entirely.
    if [[ -d /etc/netplan ]]; then
        local netplan_file=/etc/netplan/99-saint-wifi.yaml
        if [[ ! -f "$netplan_file" ]]; then
            if (( DRY_RUN )); then
                echo "+ write ${netplan_file} (renderer=NetworkManager for ${wlan})"
            else
                cat > "$netplan_file" <<NETPLAN
network:
  version: 2
  renderer: NetworkManager
  wifis:
    ${wlan}: {}
NETPLAN
                chmod 0600 "$netplan_file"
                netplan apply || warn "netplan apply reported an issue (continuing)"
            fi
        fi
    else
        log "No /etc/netplan — assuming NetworkManager manages ${wlan} directly"
    fi

    # Set regulatory domain — required by most adapters before AP mode works.
    run iw reg set "${WIFI_COUNTRY}" || warn "Could not set WiFi country to ${WIFI_COUNTRY}"

    # (Re)create the AP profile. Deleting first keeps this idempotent across
    # repeated installs / SSID changes.
    local conn=saint-os-ap
    run nmcli connection delete "$conn" || true
    run nmcli connection add type wifi ifname "$wlan" \
        con-name "$conn" autoconnect yes ssid "$WIFI_SSID"
    run nmcli connection modify "$conn" \
        802-11-wireless.mode ap \
        802-11-wireless.band bg \
        ipv4.method shared \
        wifi-sec.key-mgmt wpa-psk \
        wifi-sec.psk "$WIFI_PASS"

    if (( NO_START )); then
        log "WiFi AP profile created (autostarts on next boot)"
    else
        run nmcli connection up "$conn" \
            || warn "Failed to bring up AP now (will retry on boot)"
    fi
}

setup_wifi_ap

# --- Summary -----------------------------------------------------------------

if (( NO_WIFI )); then
    WIFI_SUMMARY="  WiFi AP:   (skipped)"
else
    WIFI_SUMMARY="  WiFi AP:   SSID=${WIFI_SSID}  password=${WIFI_PASS}"
fi

cat <<EOF

  SAINT.OS ${VERSION} installed.

  Status:    systemctl status ${SERVICE_NAME}
  Logs:      journalctl -u ${SERVICE_NAME} -f
  Configs:   ${CONFIG_DIR}/
  Payload:   ${PREFIX}/
  Web UI:    http://\$(hostname -I | awk '{print \$1}')/
${WIFI_SUMMARY}

EOF
