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

# mDNS hostname — clients on the SAINT-OS network reach the server at
# <hostname>.local. Override via env to use a different name.
SAINT_HOSTNAME="${SAINT_HOSTNAME:-opensaint}"

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
    # ROS2 Python runtime deps that ros2cli / ros2launch / rclpy import
    # eagerly. Names match on both Debian Bookworm and Ubuntu noble.
    python3-packaging
    python3-lark
    python3-empy
    python3-catkin-pkg
    python3-importlib-metadata
    # mDNS — so clients on the SAINT-OS AP can reach opensaint.local
    # without knowing the assigned IP. libnss-mdns lets the Pi itself
    # resolve .local hostnames (useful for any local tooling).
    avahi-daemon
    libnss-mdns
)

# Python 3.11 specifically: the bundled ROS2 was built against libpython3.11.
# Bookworm has it as the default; Ubuntu noble (default 3.12) ships it in
# universe and apt installs it alongside.
PY311_DEPS=(libpython3.11 python3.11)

# --- Local apt repo from bundled .debs (offline support) --------------------
#
# Air-gapped installs: the dist tarball ships a deps/ directory containing
# .deb files for every runtime dependency plus a Packages index. install.sh
# wires it up as a temporary local apt source so apt can satisfy deps
# without internet access. Used only on Debian/Pi OS hosts (where the
# bundled debs match the target ABI).

LOCAL_APT_LIST="/etc/apt/sources.list.d/saint-os-local.list"
LOCAL_REPO_ENABLED=0
APT_UPDATE_OPTS=()

setup_local_apt_repo() {
    if [[ ! -d "${PAYLOAD_DIR}/deps" ]]; then
        return
    fi
    if ! ls "${PAYLOAD_DIR}/deps"/*.deb >/dev/null 2>&1; then
        return
    fi
    case "${OS_ID}" in
        debian|raspbian) ;;
        *)
            log "Bundled debs present but OS is ${OS_ID} — using online apt instead"
            return
            ;;
    esac

    log "Configuring local apt repo at ${PAYLOAD_DIR}/deps"
    if (( DRY_RUN )); then
        echo "+ write ${LOCAL_APT_LIST}"
    else
        # trusted=yes accepts the unsigned local repo.
        echo "deb [trusted=yes] file://${PAYLOAD_DIR}/deps ./" > "${LOCAL_APT_LIST}"
    fi

    # Restrict apt-get update to ONLY the local source — avoids hanging on
    # unreachable online mirrors when the robot is offline.
    APT_UPDATE_OPTS=(
        -o "Dir::Etc::sourcelist=${LOCAL_APT_LIST}"
        -o "Dir::Etc::sourceparts=-"
        -o "APT::Get::List-Cleanup=0"
    )
    LOCAL_REPO_ENABLED=1
}

teardown_local_apt_repo() {
    if (( LOCAL_REPO_ENABLED )) && [[ -f "${LOCAL_APT_LIST}" ]]; then
        rm -f "${LOCAL_APT_LIST}"
    fi
}
trap teardown_local_apt_repo EXIT

setup_local_apt_repo

log "Installing runtime dependencies via apt"
if (( LOCAL_REPO_ENABLED )); then
    log "Using bundled .deb repo (no internet required)"
    run apt-get update "${APT_UPDATE_OPTS[@]}"
else
    run apt-get update
    if [[ "${OS_FAMILY}" == "ubuntu" ]]; then
        run apt-get install -y software-properties-common
        run add-apt-repository -y universe
        run apt-get update
    fi
fi
run apt-get install -y "${COMMON_DEPS[@]}" "${PY311_DEPS[@]}"

# --- Extract bundled ROS2 ---------------------------------------------------

# We deploy to /opt/ros/<distro>/install/ — same path the saint_os build
# container saw at compile time. colcon embeds absolute paths to ROS2's
# local_setup.bash inside saint_os's setup tree; if the deploy path
# doesn't match the build path, the systemd unit fails to source.
ROS_PARENT="/opt/ros/${ROS_DISTRO}"
ROS_PREFIX="${ROS_PARENT}/install"
if [[ ! -d "${PAYLOAD_DIR}/ros2_install" ]]; then
    die "Bundled ROS2 not found at ${PAYLOAD_DIR}/ros2_install"
fi
log "Extracting bundled ROS2 to ${ROS_PREFIX}"
# Wipe any prior install (including older layouts that put ROS2 directly
# under /opt/ros/<distro> without the install/ subdir).
run rm -rf "${ROS_PARENT:?}"
run install -d "${ROS_PREFIX}"
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

# Stop the existing service (if running) so we can replace files cleanly.
# Idempotent: systemctl stop on a non-running unit is a no-op.
if systemctl list-unit-files --quiet --type=service "${SERVICE_NAME}.service" >/dev/null 2>&1; then
    if systemctl is-active --quiet "${SERVICE_NAME}.service"; then
        log "Stopping running ${SERVICE_NAME} for update"
        run systemctl stop "${SERVICE_NAME}.service" || true
    fi
fi

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

# Persist version + manifest so the update manager can read the installed
# version cheaply (without parsing the ROS package.xml).
if (( DRY_RUN )); then
    echo "+ write ${PREFIX}/VERSION"
    echo "+ copy manifest.json -> ${PREFIX}/"
else
    echo "${VERSION}" > "${PREFIX}/VERSION"
    if [[ -f "${PAYLOAD_DIR}/manifest.json" ]]; then
        cp "${PAYLOAD_DIR}/manifest.json" "${PREFIX}/manifest.json"
    fi
fi

run chown -R "${SERVICE_USER}:${SERVICE_USER}" "${PREFIX}"

# --- Update wrapper + sudoers -----------------------------------------------
#
# Install the apply-update.sh wrapper so the web UI (running as the saint
# user) can apply downloaded updates without a full root shell. The
# sudoers rule grants NOPASSWD access to that single command and nothing
# else.

if [[ -f "${PAYLOAD_DIR}/apply-update.sh" ]]; then
    log "Installing update wrapper to ${PREFIX}/bin/apply-update.sh"
    run install -d -m 0755 "${PREFIX}/bin"
    run install -m 0755 -o root -g root \
        "${PAYLOAD_DIR}/apply-update.sh" "${PREFIX}/bin/apply-update.sh"

    log "Writing sudoers rule for update applier"
    SUDOERS_FILE="/etc/sudoers.d/saint-os-updater"
    if (( DRY_RUN )); then
        echo "+ write ${SUDOERS_FILE}"
    else
        cat > "${SUDOERS_FILE}" <<SUDOERS
# SAINT.OS web UI applies updates via this single wrapper.
${SERVICE_USER} ALL=(root) NOPASSWD: ${PREFIX}/bin/apply-update.sh
SUDOERS
        chmod 0440 "${SUDOERS_FILE}"
        # Sanity check syntax — broken sudoers can lock the host out.
        if command -v visudo >/dev/null 2>&1; then
            visudo -cf "${SUDOERS_FILE}" >/dev/null
        fi
    fi

    # Updates staging directory (writable by service user; tarballs land here).
    run install -d -m 0755 -o "${SERVICE_USER}" -g "${SERVICE_USER}" \
        "${STATE_DIR}/updates"
fi

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

# Clear any failed-state flag so a previously-crashed install doesn't
# block restart. Idempotent — no-op if the unit isn't in failed state.
run systemctl reset-failed "${SERVICE_NAME}.service" 2>/dev/null || true

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
        # Always rewrite so config changes (different interface name across
        # re-runs, etc.) actually take effect.
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

# --- mDNS hostname -----------------------------------------------------------
#
# Set a stable hostname and make sure avahi-daemon is running so clients
# joining the SAINT-OS AP can navigate to http://<hostname>.local
# regardless of the auto-assigned IP. macOS resolves .local via Bonjour
# natively; Linux clients need libnss-mdns (or avahi-resolve) installed.

setup_mdns() {
    log "Setting hostname to ${SAINT_HOSTNAME}"
    if (( DRY_RUN )); then
        echo "+ hostnamectl set-hostname ${SAINT_HOSTNAME}"
    else
        # Idempotent — hostnamectl is a no-op if the hostname is already set.
        hostnamectl set-hostname "${SAINT_HOSTNAME}" || \
            warn "hostnamectl failed; falling back to /etc/hostname"
        echo "${SAINT_HOSTNAME}" > /etc/hostname

        # Keep /etc/hosts in sync so local lookups of the hostname work
        # even when avahi isn't queried (e.g., sudo's reverse lookup).
        if ! grep -qE "^127\.0\.1\.1[[:space:]]+${SAINT_HOSTNAME}\b" /etc/hosts; then
            # Strip any prior 127.0.1.1 entry and append the new one.
            sed -i.saint-bak '/^127\.0\.1\.1[[:space:]]/d' /etc/hosts
            echo "127.0.1.1   ${SAINT_HOSTNAME}" >> /etc/hosts
        fi
    fi

    # Advertise SAINT.OS as a Bonjour HTTP service. Optional — pure cosmetic
    # for Bonjour browsers; .local resolution works without it.
    local svc_file=/etc/avahi/services/saint-os.service
    if (( DRY_RUN )); then
        echo "+ write ${svc_file}"
    else
        install -d /etc/avahi/services
        cat > "${svc_file}" <<AVAHI
<?xml version="1.0" standalone='no'?>
<!DOCTYPE service-group SYSTEM "avahi-service.dtd">
<service-group>
  <name replace-wildcards="yes">SAINT.OS on %h</name>
  <service>
    <type>_http._tcp</type>
    <port>80</port>
  </service>
</service-group>
AVAHI
        chmod 0644 "${svc_file}"
    fi

    run systemctl enable --now avahi-daemon || \
        warn "Could not start avahi-daemon — clients will need the IP address"
}

setup_mdns

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
  Web UI:    http://${SAINT_HOSTNAME}.local/   (or http://\$(hostname -I | awk '{print \$1}')/)
${WIFI_SUMMARY}

EOF
