#!/usr/bin/env bash
#
# SAINT.OS Server installer.
#
# Self-contained: ROS2 (kilted + micro-ROS agent) is bundled in the payload
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
NO_DHCP=0

# WiFi AP defaults — override via env vars before running install.sh.
WIFI_SSID="${SAINT_WIFI_SSID:-OpenSAINT}"
WIFI_PASS="${SAINT_WIFI_PASS:-ifeelalive}"
WIFI_COUNTRY="${SAINT_WIFI_COUNTRY:-US}"

# mDNS hostname — clients on the SAINT-OS network reach the server at
# <hostname>.local. Override via env to use a different name.
SAINT_HOSTNAME="${SAINT_HOSTNAME:-opensaint}"

# Internal-network DHCP defaults match server.yaml's network.internal block.
# Override via env to deploy on a different subnet.
SAINT_INTERNAL_IFACE="${SAINT_INTERNAL_IFACE:-eth0}"
SAINT_INTERNAL_IP="${SAINT_INTERNAL_IP:-192.168.10.1}"
SAINT_INTERNAL_CIDR="${SAINT_INTERNAL_CIDR:-24}"
SAINT_DHCP_RANGE_START="${SAINT_DHCP_RANGE_START:-192.168.10.10}"
SAINT_DHCP_RANGE_END="${SAINT_DHCP_RANGE_END:-192.168.10.254}"
SAINT_DHCP_LEASE="${SAINT_DHCP_LEASE:-12h}"

for arg in "$@"; do
    case "$arg" in
        --no-start) NO_START=1 ;;
        --no-wifi)  NO_WIFI=1 ;;
        --no-dhcp)  NO_DHCP=1 ;;
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
    python3-argcomplete
    # mDNS — so clients on the SAINT-OS AP can reach opensaint.local
    # without knowing the assigned IP. libnss-mdns lets the Pi itself
    # resolve .local hostnames (useful for any local tooling).
    avahi-daemon
    libnss-mdns
    # DHCP + DNS server for the internal node network (eth0). Microcontroller
    # nodes get IPs handed out from the configured range and can resolve
    # the server's hostname even without mDNS.
    dnsmasq
    # zstd: distribution tarballs since 2026-05 ship as .tar.zst (the
    # build step compresses ~3× faster than gzip and the archives are
    # slightly smaller). apply-update.sh uses `tar -xaf` which calls
    # zstd via the system path; without this package an OTA update of
    # a zstd-compressed tarball would fail with "Cannot exec zstd".
    # Pi OS Bookworm bundles it with dpkg already, but Ubuntu noble
    # and minimal Debian images don't — listing it here closes the gap.
    zstd
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
LOCAL_APT_CONF="/etc/apt/apt.conf.d/00saint-os-install"
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
        echo "+ write ${LOCAL_APT_CONF}"
    else
        # trusted=yes accepts the unsigned local repo.
        echo "deb [trusted=yes] file://${PAYLOAD_DIR}/deps ./" > "${LOCAL_APT_LIST}"

        # apt drops privileges to the _apt user when fetching, which
        # blows up when the payload lives under /home/<user>/ (mode
        # 0700 — _apt can't traverse in). Disable the sandbox just for
        # this install run; the conf file is removed on exit.
        echo 'APT::Sandbox::User "root";' > "${LOCAL_APT_CONF}"
        chmod 0644 "${LOCAL_APT_CONF}"
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
    if (( LOCAL_REPO_ENABLED )); then
        [[ -f "${LOCAL_APT_LIST}" ]] && rm -f "${LOCAL_APT_LIST}"
        [[ -f "${LOCAL_APT_CONF}" ]] && rm -f "${LOCAL_APT_CONF}"
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
# One-time migration: older layouts dropped ROS2 directly under
# /opt/ros/<distro> (no install/ subdir). If we see that — a setup.bash
# at ROS_PARENT but no install/ subdir — wipe ROS_PARENT first so the
# rsync below starts clean. Once everyone's on the install/ layout the
# wipe never fires.
if [[ -f "${ROS_PARENT}/setup.bash" && ! -d "${ROS_PREFIX}" ]]; then
    log "Older ROS2 layout under ${ROS_PARENT} — wiping before sync"
    run rm -rf "${ROS_PARENT:?}"
fi
run install -d "${ROS_PREFIX}"

# rsync skips files whose size + mtime match what we'd copy (tar/cp -a
# preserve mtime, so an unchanged tree statting-clean is the common
# case on upgrade). --delete removes files in the destination that
# aren't in the payload, matching the old "wipe and re-extract"
# semantics for version churn. Result: a no-op upgrade walks the tree
# without writing, instead of copying hundreds of MB.
log "Syncing bundled ROS2 to ${ROS_PREFIX}"
run rsync -a --delete "${PAYLOAD_DIR}/ros2_install/" "${ROS_PREFIX}/"

if [[ ! -f "${ROS_PREFIX}/setup.bash" ]]; then
    die "ROS2 setup.bash not found at ${ROS_PREFIX}/setup.bash after extract"
fi

# --- User + directories ------------------------------------------------------

if ! id -u "${SERVICE_USER}" >/dev/null 2>&1; then
    log "Creating system user ${SERVICE_USER}"
    run useradd --system --create-home --home-dir "${STATE_DIR}" \
        --shell /usr/sbin/nologin "${SERVICE_USER}"
fi

# Add to standard hardware-access groups so the service can:
#   dialout — RC receiver / SBUS UART
#   gpio    — direct GPIO from Python
#   video   — vcgencmd (CPU temp / throttle status via /dev/vchiq)
for grp in dialout gpio video; do
    if getent group "$grp" >/dev/null; then
        run usermod -aG "$grp" "${SERVICE_USER}"
    fi
done

run install -d -m 0755 -o "${SERVICE_USER}" -g "${SERVICE_USER}" \
    "${PREFIX}" "${CONFIG_DIR}" "${CONFIG_DIR}/nodes" "${STATE_DIR}" "${LOG_DIR}"

# Migrate runtime state from the previous install location. Earlier
# versions stored adopted-node YAML + system routing in
# /opt/saint-os/install/share/saint_os/config/, which is part of the
# install tree we wipe on every update — so they got nuked. If we find
# any leftover here and the new ${CONFIG_DIR} doesn't have them yet,
# move them over before the wipe.
if [[ -d "${PREFIX}/install/share/saint_os/config" ]]; then
    old_nodes_dir="${PREFIX}/install/share/saint_os/config/nodes"
    old_routing="${PREFIX}/install/share/saint_os/config/system_routing.yaml"
    if [[ -d "$old_nodes_dir" ]]; then
        for f in "$old_nodes_dir"/*.yaml; do
            [[ -e "$f" ]] || continue
            dest="${CONFIG_DIR}/nodes/$(basename "$f")"
            if [[ ! -e "$dest" ]]; then
                log "Migrating node config to persistent location: $(basename "$f")"
                run install -m 0644 -o "${SERVICE_USER}" -g "${SERVICE_USER}" "$f" "$dest"
            fi
        done
    fi
    if [[ -e "$old_routing" && ! -e "${CONFIG_DIR}/system_routing.yaml" ]]; then
        log "Migrating system_routing.yaml to persistent location"
        run install -m 0644 -o "${SERVICE_USER}" -g "${SERVICE_USER}" \
            "$old_routing" "${CONFIG_DIR}/system_routing.yaml"
    fi
fi

# Stop the existing service (if running) so we can replace files cleanly.
# Idempotent: systemctl stop on a non-running unit is a no-op.
if systemctl list-unit-files --quiet --type=service "${SERVICE_NAME}.service" >/dev/null 2>&1; then
    if systemctl is-active --quiet "${SERVICE_NAME}.service"; then
        log "Stopping running ${SERVICE_NAME} for update"
        run systemctl stop "${SERVICE_NAME}.service" || true
    fi
fi

# --- Copy payload ------------------------------------------------------------

log "Syncing saint_os payload to ${PREFIX}/install"
run install -d "${PREFIX}/install"
run rsync -a --delete "${PAYLOAD_DIR}/saint_install/" "${PREFIX}/install/"

if [[ -d "${PAYLOAD_DIR}/firmware" ]]; then
    log "Syncing firmware bundle to ${PREFIX}/firmware"
    run install -d "${PREFIX}/firmware"
    run rsync -a --delete "${PAYLOAD_DIR}/firmware/" "${PREFIX}/firmware/"

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

    # USB helper — scans/stages tarballs on mounted removable media.
    # Needed because udisks2 mounts USB sticks at 0700 paths only the
    # mounting user can read; the saint service needs root to traverse.
    if [[ -f "${PAYLOAD_DIR}/usb-helper.sh" ]]; then
        run install -m 0755 -o root -g root \
            "${PAYLOAD_DIR}/usb-helper.sh" "${PREFIX}/bin/usb-helper.sh"
    fi

    log "Writing sudoers rule for ${SERVICE_USER}"
    SUDOERS_FILE="/etc/sudoers.d/saint-os-updater"
    if (( DRY_RUN )); then
        echo "+ write ${SUDOERS_FILE}"
    else
        # The web terminal runs as ${SERVICE_USER} but the operator
        # needs to run privileged commands (systemctl, apt, edit /etc,
        # etc.) so we grant full NOPASSWD sudo. Effectively makes the
        # dashboard's WebSocket password the root-access password —
        # leave a comment so future readers understand the trade.
        cat > "${SUDOERS_FILE}" <<SUDOERS
# SAINT.OS service account. NOPASSWD sudo lets the web terminal and
# update flow run privileged commands. Anyone with the dashboard
# password can reach root through this rule — protect it accordingly.
${SERVICE_USER} ALL=(root) NOPASSWD: ALL
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

# Returns 0 (success) when the AP is LIVE and serving the expected
# SSID — meaning a re-run of install.sh should leave it alone.
#
# We used to compare ten nmcli stored property fields for exact
# string match (ssid/mode/band/powersave/keymgmt/psk/proto/pairwise/
# group/ipmethod). That broke too easily: nmcli normalizes those
# values inconsistently across versions (band can come back as "bg"
# or "BG", powersave as "2" or "default", PSK as raw or hashed), so
# a single field drifting would fail the fast-path and the function
# would fall through to `netplan apply` + `iw reg set` — both of
# which flap the radio and drop every connected client, including
# the operator's SSH session over WiFi.
#
# Live-state check is what actually matters: if NM reports the
# connection activated AND `iw dev` shows the interface in AP mode
# with our SSID, clients are connected and traffic is flowing. No
# matter what nmcli's stored representation looks like, there's
# nothing to fix.
#
# Trade-off: if someone changes WIFI_SSID/WIFI_PASS in env and
# re-runs install, the SSID change is still caught (drives the
# fast-path miss) but a pure password change is NOT. If you need
# to rotate the PSK, delete the saint-os-ap connection first or
# run with --no-wifi and reconfigure by hand.
_wifi_ap_already_matches() {
    local conn=$1

    # 1. Connection profile exists in NM.
    nmcli -g connection.id connection show "$conn" >/dev/null 2>&1 || return 1

    # 2. Connection is currently activated.
    local state
    state=$(nmcli -g GENERAL.STATE connection show "$conn" 2>/dev/null || true)
    [[ "$state" == "activated" ]] || return 1

    # 3. Find which wifi interface the activated connection is bound
    #    to. nmcli reports this in GENERAL.DEVICES for activated
    #    connections.
    local device
    device=$(nmcli -g GENERAL.DEVICES connection show "$conn" 2>/dev/null || true)
    [[ -n "$device" ]] || return 1

    # 4. Live radio state: interface in AP mode broadcasting our SSID.
    #    `iw dev <iface> info` output (varies slightly between
    #    versions) includes both `ssid <NAME>` and `type AP` lines.
    command -v iw >/dev/null 2>&1 || return 1
    local iw_info
    iw_info=$(iw dev "$device" info 2>/dev/null || true)
    [[ "$iw_info" == *"type AP"* ]]              || return 1
    [[ "$iw_info" == *"ssid ${WIFI_SSID}"* ]]    || return 1

    return 0
}

setup_wifi_ap() {
    if (( NO_WIFI )); then
        log "Skipping WiFi AP setup (--no-wifi)"
        return
    fi

    # Fast path: AP is already up exactly the way we'd configure it.
    # Skip ALL the setup steps below — including netplan apply and
    # iw reg set, which restart the WiFi interface and drop every
    # connected client even when the profile is unchanged. The
    # existing "profile differs" comparison further down already
    # avoids the nmcli teardown when settings match, but the early
    # disruptive steps run regardless. This guard cuts the whole
    # function off so a re-run of install.sh on a working rover
    # leaves WiFi (and its operator SSH session) completely alone.
    if _wifi_ap_already_matches saint-os-ap; then
        log "WiFi AP 'saint-os-ap' is up and matches desired config — leaving it untouched"
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

    # Idempotent reconfigure: if the existing profile already matches what
    # we'd write, don't touch it. Tearing down the AP just to re-add the
    # same settings kicks every connected client — including the operator's
    # SSH session over WiFi.
    #
    # Security: nmcli's default `key-mgmt=wpa-psk` accepts both WPA-TKIP
    # and WPA2-AES, which macOS / iOS now flag as "weak security." Pin to
    # WPA2-only (proto=rsn) with AES/CCMP for both pairwise and group
    # ciphers — same UX, no insecure-network warning.
    # AP credential policy (2026-05-25 change):
    #
    # SSID, password, band, and channel are *operator-managed* via the
    # dashboard's Wireless settings. Re-running install.sh must NOT
    # clobber operator-set values — otherwise every dist update would
    # silently revert the AP back to the env-var defaults, kick the
    # operator off, and force them to reconfigure. So:
    #
    #   - Profile missing → create from env-var seeds. First install.
    #   - Profile exists  → only ensure *structural* fields (mode=ap,
    #                       ipv4.method=shared, WPA2 cipher stack,
    #                       powersave off) match what we expect.
    #                       Leave SSID/PSK/band/channel alone.
    #
    # nmcli modify is idempotent and per-property — modifying e.g.
    # `mode ap` when it's already `ap` is a no-op, so this stays safe
    # to re-run.
    local conn=saint-os-ap
    local existed=0
    if nmcli -g connection.id connection show "$conn" >/dev/null 2>&1; then
        existed=1
    fi

    if (( existed == 0 )); then
        log "Creating WiFi AP profile '${conn}' from defaults"
        run nmcli connection add type wifi ifname "$wlan" \
            con-name "$conn" autoconnect yes ssid "$WIFI_SSID"
        # 802-11-wireless.powersave=2 disables Pi WiFi power-save on the AP.
        # The brcmfmac default leaves it on, which causes idle SSH sessions
        # over WiFi to lock up while the radio dozes between beacons.
        run nmcli connection modify "$conn" \
            802-11-wireless.mode ap \
            802-11-wireless.band bg \
            802-11-wireless.powersave 2 \
            ipv4.method shared \
            wifi-sec.key-mgmt wpa-psk \
            wifi-sec.proto rsn \
            wifi-sec.pairwise ccmp \
            wifi-sec.group ccmp \
            wifi-sec.psk "$WIFI_PASS"
    else
        log "WiFi AP profile '${conn}' already exists — preserving operator-set SSID/password/band/channel"
        # Touch only the fields that are structural / required for the
        # AP to function correctly. Notably absent: ssid, band,
        # channel, psk — those are managed via the dashboard.
        run nmcli connection modify "$conn" \
            802-11-wireless.mode ap \
            802-11-wireless.powersave 2 \
            ipv4.method shared \
            wifi-sec.key-mgmt wpa-psk \
            wifi-sec.proto rsn \
            wifi-sec.pairwise ccmp \
            wifi-sec.group ccmp
    fi

    if (( NO_START )); then
        log "WiFi AP profile ready (autostarts on next boot)"
    elif (( existed == 0 )); then
        # First-time bring-up after creation.
        run nmcli connection up "$conn" \
            || warn "Failed to bring up AP now (will retry on boot)"
    elif ! nmcli -g GENERAL.STATE connection show "$conn" 2>/dev/null | grep -q activated; then
        # Existing profile but currently inactive — bring it back up.
        run nmcli connection up "$conn" \
            || warn "Failed to bring up AP now (will retry on boot)"
    else
        # Existing AND active — leave it alone so a reinstall doesn't
        # kick the operator's own session. Any structural-field changes
        # we just made will take effect on next nmcli con up / reboot.
        log "WiFi AP already active — not touching it"
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

    # Advertise SAINT.OS as a Bonjour HTTP service. Avahi watches this
    # directory and re-publishes when files change — so we only write if
    # the content actually differs from what's already there.
    local svc_file=/etc/avahi/services/saint-os.service
    local svc_tmp; svc_tmp=$(mktemp)
    cat > "$svc_tmp" <<AVAHI
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

    if (( DRY_RUN )); then
        if [[ ! -f "$svc_file" ]] || ! cmp -s "$svc_tmp" "$svc_file"; then
            echo "+ write ${svc_file}"
        else
            echo "+ ${svc_file} unchanged"
        fi
        rm -f "$svc_tmp"
    elif [[ ! -f "$svc_file" ]] || ! cmp -s "$svc_tmp" "$svc_file"; then
        install -d /etc/avahi/services
        install -m 0644 "$svc_tmp" "$svc_file"
        rm -f "$svc_tmp"
    else
        rm -f "$svc_tmp"
    fi

    # `enable --now` is idempotent: it won't restart an already-running
    # daemon, just makes sure it's enabled + started.
    run systemctl enable --now avahi-daemon || \
        warn "Could not start avahi-daemon — clients will need the IP address"
}

setup_mdns

# --- Internal network DHCP + DNS --------------------------------------------
#
# Hand out IPs on eth0 so microcontroller node controllers join the network
# automatically. dnsmasq binds with bind-dynamic + interface=<iface> so it
# only owns the internal interface; NetworkManager's internal dnsmasq on
# wlan0 (from the AP's `shared` mode) is unaffected.

setup_internal_dhcp() {
    if (( NO_DHCP )); then
        log "Skipping DHCP server setup (--no-dhcp)"
        return
    fi

    if ! ip link show "${SAINT_INTERNAL_IFACE}" >/dev/null 2>&1; then
        warn "Interface ${SAINT_INTERNAL_IFACE} not present — skipping DHCP setup"
        return
    fi

    # Idempotent reconfigure: only touch the interface if the LIVE
    # state differs from what we want. The earlier version of this
    # check compared nmcli's stored properties (ipv4.method,
    # ipv4.addresses, connection.interface-name) for an exact string
    # match, which broke whenever nmcli normalized one of those
    # values differently between versions — need_apply ended up =1
    # on every re-install, the delete/add/up cycle bounced eth0, and
    # every node leased through this interface lost its DHCP grant.
    #
    # Live-state check is what actually matters: if the desired
    # CIDR is already assigned to the interface and the link is up,
    # the network is functional regardless of what nmcli's stored
    # representation looks like. Skip the bounce in that case.
    local conn=saint-os-internal
    local desired_addr="${SAINT_INTERNAL_IP}/${SAINT_INTERNAL_CIDR}"
    local need_apply=1

    # `ip -4 -o addr show dev eth0` prints something like:
    #   3: eth0    inet 192.168.10.1/24 brd 192.168.10.255 scope global eth0\       valid_lft forever preferred_lft forever
    # The 4th whitespace-separated field is the CIDR. Loop in case
    # the interface has multiple v4 addrs; match any of them against
    # the desired one.
    local have_desired=0
    while read -r live_addr; do
        if [[ "$live_addr" == "$desired_addr" ]]; then
            have_desired=1
            break
        fi
    done < <(ip -4 -o addr show dev "$SAINT_INTERNAL_IFACE" 2>/dev/null \
             | awk '{print $4}')

    local link_up=0
    if [[ "$(cat "/sys/class/net/${SAINT_INTERNAL_IFACE}/operstate" 2>/dev/null)" == "up" ]]; then
        link_up=1
    fi

    if (( have_desired )) && (( link_up )); then
        log "Internal interface ${SAINT_INTERNAL_IFACE} already at ${desired_addr} and up — not reconfiguring (leases preserved)"
        need_apply=0
    fi

    if command -v nmcli >/dev/null 2>&1; then
        if (( need_apply )); then
            if nmcli -g connection.id connection show "$conn" >/dev/null 2>&1; then
                log "Internal interface needs reconfigure (have='$(ip -4 -o addr show dev "$SAINT_INTERNAL_IFACE" 2>/dev/null | awk '{print $4}' | paste -sd, -)', want='${desired_addr}') — briefly drops node leases"
            else
                log "Configuring ${SAINT_INTERNAL_IFACE} with static IP ${desired_addr}"
            fi
            run nmcli connection delete "$conn" 2>/dev/null || true
            run nmcli connection add type ethernet ifname "${SAINT_INTERNAL_IFACE}" \
                con-name "$conn" autoconnect yes \
                ipv4.method manual \
                ipv4.addresses "${desired_addr}" \
                ipv6.method ignore
            if ! (( NO_START )); then
                run nmcli connection up "$conn" \
                    || warn "Failed to bring up internal interface now (will retry on boot)"
            fi
        elif ! nmcli -g connection.id connection show "$conn" >/dev/null 2>&1; then
            # Live state is good but no persistent nmcli record exists —
            # someone configured the IP by hand. Add the connection profile
            # WITHOUT bringing it up so reboots stick, while leaving the
            # current live link (and its leases) undisturbed.
            log "Adding persistent nmcli profile for ${SAINT_INTERNAL_IFACE} (current link left in place)"
            run nmcli connection add type ethernet ifname "${SAINT_INTERNAL_IFACE}" \
                con-name "$conn" autoconnect yes \
                ipv4.method manual \
                ipv4.addresses "${desired_addr}" \
                ipv6.method ignore
        fi
    else
        warn "nmcli not available — set ${SAINT_INTERNAL_IFACE}'s IP manually"
    fi

    # dnsmasq config — only write + restart if the contents actually changed.
    # The whole point of this block is "don't bounce running services when
    # nothing meaningful changed."
    local dnsmasq_conf=/etc/dnsmasq.d/saint-os.conf
    local dnsmasq_tmp; dnsmasq_tmp=$(mktemp)
    cat > "$dnsmasq_tmp" <<DNSMASQ
# SAINT.OS internal node network
# Generated by install.sh — edit and restart dnsmasq to apply changes.

interface=${SAINT_INTERNAL_IFACE}
bind-dynamic

# DHCP for microcontroller nodes joining the internal network.
dhcp-range=${SAINT_DHCP_RANGE_START},${SAINT_DHCP_RANGE_END},${SAINT_DHCP_LEASE}
dhcp-option=option:router,${SAINT_INTERNAL_IP}
dhcp-option=option:dns-server,${SAINT_INTERNAL_IP}

# DNS: pin the server's hostname so nodes without mDNS clients can still
# resolve it. Local '.local' queries are kept here (not forwarded upstream).
address=/${SAINT_HOSTNAME}.local/${SAINT_INTERNAL_IP}
address=/${SAINT_HOSTNAME}/${SAINT_INTERNAL_IP}
local=/local/

# Reasonable default cache size for a small fleet.
cache-size=512
DNSMASQ

    local dnsmasq_changed=1
    if [[ -f "$dnsmasq_conf" ]] && cmp -s "$dnsmasq_tmp" "$dnsmasq_conf"; then
        dnsmasq_changed=0
    fi

    if (( DRY_RUN )); then
        if (( dnsmasq_changed )); then
            echo "+ write ${dnsmasq_conf}"
        else
            echo "+ ${dnsmasq_conf} unchanged"
        fi
        rm -f "$dnsmasq_tmp"
    elif (( dnsmasq_changed )); then
        log "Updating dnsmasq config for ${SAINT_INTERNAL_IFACE}"
        install -m 0644 "$dnsmasq_tmp" "$dnsmasq_conf"
        rm -f "$dnsmasq_tmp"
    else
        log "dnsmasq config unchanged — not restarting service"
        rm -f "$dnsmasq_tmp"
    fi

    run systemctl enable dnsmasq
    if ! (( NO_START )); then
        if (( dnsmasq_changed )) || ! systemctl is-active --quiet dnsmasq; then
            run systemctl restart dnsmasq \
                || warn "dnsmasq failed to start — check 'journalctl -u dnsmasq'"
        fi
    fi
}

setup_internal_dhcp

# --- Summary -----------------------------------------------------------------

if (( NO_WIFI )); then
    WIFI_SUMMARY="  WiFi AP:   (skipped)"
else
    WIFI_SUMMARY="  WiFi AP:   SSID=${WIFI_SSID}  password=${WIFI_PASS}"
fi

if (( NO_DHCP )); then
    DHCP_SUMMARY="  DHCP:      (skipped)"
else
    DHCP_SUMMARY="  DHCP:      ${SAINT_INTERNAL_IFACE} ${SAINT_INTERNAL_IP}/${SAINT_INTERNAL_CIDR}  pool=${SAINT_DHCP_RANGE_START}-${SAINT_DHCP_RANGE_END}"
fi

cat <<EOF

  SAINT.OS ${VERSION} installed.

  Status:    systemctl status ${SERVICE_NAME}
  Logs:      journalctl -u ${SERVICE_NAME} -f
  Configs:   ${CONFIG_DIR}/
  Payload:   ${PREFIX}/
  Web UI:    http://${SAINT_HOSTNAME}.local/   (or http://\$(hostname -I | awk '{print \$1}')/)
${WIFI_SUMMARY}
${DHCP_SUMMARY}

EOF
