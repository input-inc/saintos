#!/usr/bin/env bash
#
# SAINT.OS update applier — runs as root via sudoers from the saint user.
#
# Validates a downloaded/USB-supplied dist tarball, then launches a
# transient systemd unit that extracts it and runs the bundled
# install.sh.
#
# Usage:
#   apply-update.sh <tarball_path>                 # launcher (server calls this)
#   apply-update.sh --worker <tarball_path> <log>  # transient-unit half (internal)
#
# TWO things the launcher must get right, both learned the hard way:
#
#  1. Don't block. The caller (update_manager) bounds us with a short
#     sudo timeout; extracting the ~1.2 GB payload takes ~50 s, so the
#     heavy work must run detached and the launcher must return in <1 s.
#
#  2. Escape this service's sandbox. saint-os.service runs with
#     ProtectSystem=full, which mounts /etc, /usr, /boot READ-ONLY for
#     the service AND everything it spawns — and sudo/setsid/nohup do
#     NOT leave that mount namespace. install.sh writes /etc/apt,
#     /etc/sudoers.d, etc., so a setsid-detached worker died with
#     "/etc/...: Read-only file system". `systemd-run` asks PID 1 to run
#     the worker as a fresh transient unit in systemd's own (unsandboxed)
#     namespace — writable /etc — and that unit also outlives the
#     saint-os restart install.sh triggers.
#
# The script is intentionally small: keep the privilege boundary surface
# minimal so the sudoers rule covers exactly one well-understood command.

set -euo pipefail

ALLOWED_PARENT="/var/lib/saint-os/updates"

# ── Worker mode (runs inside the transient systemd unit) ────────────────
if [[ "${1:-}" == "--worker" ]]; then
    TARBALL="${2:?worker needs a tarball path}"
    LOG="${3:?worker needs a log path}"
    # Everything from here streams to the log the dashboard tails live.
    exec >>"$LOG" 2>&1
    echo "=== SAINT.OS update $(date -u +%FT%TZ 2>/dev/null || date) ==="

    STAGING="$(mktemp -d /tmp/saint-os-update.XXXXXX)"
    trap 'rm -rf "$STAGING"' EXIT

    echo "Extracting $(basename "$TARBALL") ($(du -h "$TARBALL" | cut -f1)); this can take ~1 min..."
    # Warning suppression keeps the log readable: the tarball is built on
    # macOS via Docker, so it carries xattrs/fflags Linux tar doesn't
    # know ("Ignoring unknown extended header keyword"), and a lagging Pi
    # clock flags every file as "timestamp in the future". --checkpoint
    # prints a heartbeat every ~25 MB so the long extract isn't silent.
    tar --warning=no-timestamp --warning=no-unknown-keyword \
        --checkpoint=50000 --checkpoint-action="echo=  ... still extracting" \
        -xaf "$TARBALL" -C "$STAGING"
    echo "Extraction complete."

    PAYLOAD=$(find "$STAGING" -mindepth 1 -maxdepth 1 -type d | head -n1)
    [[ -d "$PAYLOAD" ]]               || { echo "ERROR: no payload dir in tarball"; exit 2; }
    [[ -f "$PAYLOAD/install.sh" ]]    || { echo "ERROR: install.sh missing from payload"; exit 2; }
    [[ -f "$PAYLOAD/manifest.json" ]] || { echo "ERROR: manifest.json missing from payload"; exit 2; }

    RUNTIME_PAYLOAD="${ALLOWED_PARENT}/staging.$$"
    rm -rf "$RUNTIME_PAYLOAD"
    mv "$PAYLOAD" "$RUNTIME_PAYLOAD"

    echo "Running install.sh..."
    rc=0
    ( cd "$RUNTIME_PAYLOAD" && ./install.sh ) || rc=$?
    rm -rf "$RUNTIME_PAYLOAD"
    echo "=== install.sh exited rc=${rc} ==="
    exit "$rc"
fi

# ── Launcher mode (invoked via sudo by the saint user) ──────────────────
TARBALL="${1:-}"
[[ -n "$TARBALL" ]] || { echo "usage: apply-update.sh <tarball>" >&2; exit 2; }
[[ -f "$TARBALL" ]] || { echo "tarball not found: $TARBALL" >&2; exit 2; }

# Only accept tarballs from a known staging area. Prevents the saint user
# from passing arbitrary file paths through sudo.
case "$(readlink -f "$TARBALL")" in
    "${ALLOWED_PARENT}"/*) ;;
    *)
        echo "tarball must live under ${ALLOWED_PARENT}/" >&2
        exit 2
        ;;
esac

command -v systemd-run >/dev/null 2>&1 \
    || { echo "systemd-run not found — cannot launch the installer" >&2; exit 2; }

mkdir -p /var/log/saint-os
LOG="/var/log/saint-os/update-$(date +%Y%m%d-%H%M%S).log"
: > "$LOG"   # create immediately so the dashboard's log poll finds it

# Run the worker as a transient unit: fresh (unsandboxed) namespace so
# install.sh can write /etc, and it survives the saint-os restart.
# --collect reaps the unit when it exits so it doesn't linger.
if ! systemd-run --collect \
        --unit="saint-os-update-$(date +%s)" \
        --description="SAINT.OS update install" \
        "$0" --worker "$TARBALL" "$LOG" >/dev/null 2>&1; then
    echo "ERROR: systemd-run failed to launch the installer" | tee -a "$LOG" >&2
    exit 1
fi

echo "Update install launched (transient unit); log: ${LOG}" >&2
exit 0
