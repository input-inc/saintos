#!/usr/bin/env bash
#
# SAINT.OS update applier — runs as root via sudoers from the saint user.
#
# Validates a downloaded/USB-supplied dist tarball, extracts it, and
# detaches the bundled install.sh so it survives the saint-os service
# restart triggered by the install itself.
#
# Usage:
#   apply-update.sh <tarball_path>
#
# The script is intentionally small: keep the privilege boundary surface
# minimal so the sudoers rule covers exactly one well-understood command.

set -euo pipefail

TARBALL="${1:-}"
[[ -n "$TARBALL" ]] || { echo "usage: apply-update.sh <tarball>" >&2; exit 2; }
[[ -f "$TARBALL" ]] || { echo "tarball not found: $TARBALL" >&2; exit 2; }

# Only accept tarballs from a known staging area. Prevents the saint user
# from passing arbitrary file paths through sudo.
ALLOWED_PARENT="/var/lib/saint-os/updates"
case "$(readlink -f "$TARBALL")" in
    "${ALLOWED_PARENT}"/*) ;;
    *)
        echo "tarball must live under ${ALLOWED_PARENT}/" >&2
        exit 2
        ;;
esac

STAGING="$(mktemp -d /tmp/saint-os-update.XXXXXX)"
trap 'rm -rf "${STAGING}"' EXIT

echo "Extracting ${TARBALL} -> ${STAGING}" >&2
tar -xzf "${TARBALL}" -C "${STAGING}"

# Locate the unpacked payload directory (single top-level dir per tarball).
PAYLOAD=$(find "${STAGING}" -mindepth 1 -maxdepth 1 -type d | head -n1)
[[ -d "$PAYLOAD" ]] || { echo "no payload dir in tarball" >&2; exit 2; }
[[ -f "${PAYLOAD}/install.sh" ]] || { echo "install.sh missing from payload" >&2; exit 2; }
[[ -f "${PAYLOAD}/manifest.json" ]] || { echo "manifest.json missing from payload" >&2; exit 2; }

# Move payload out of the cleanup trap so the detached install.sh can
# keep reading it after this script exits.
RUNTIME_PAYLOAD="${ALLOWED_PARENT}/staging.$$"
rm -rf "$RUNTIME_PAYLOAD"
mv "$PAYLOAD" "$RUNTIME_PAYLOAD"

echo "Launching install.sh in detached process" >&2

# Detach the install so it survives the saint-os service restart that
# install.sh itself triggers. setsid creates a new session; nohup keeps
# the process alive after the parent (this script) exits; redirecting
# stdio so the sudo invocation that called us can return.
LOG="/var/log/saint-os/update-$(date +%Y%m%d-%H%M%S).log"
mkdir -p /var/log/saint-os
setsid nohup bash -c "
    cd '${RUNTIME_PAYLOAD}'
    './install.sh' >> '${LOG}' 2>&1
    rm -rf '${RUNTIME_PAYLOAD}'
" </dev/null >>"${LOG}" 2>&1 &
disown

echo "Update install detached; log: ${LOG}" >&2
exit 0
