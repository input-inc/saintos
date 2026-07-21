#!/usr/bin/env bash
#
# Build the SAINT Controller AppImage and publish it to the SAINT.OS
# server so the Steam Deck can pull it via the in-app OTA update.
#
# What it does:
#   1. Builds the AppImage in Docker (linux/amd64) via build-docker.sh,
#      which stages it + info.json into
#      server/resources/firmware/controller/.
#   2. Copies both to the server's firmware-serving directory
#      (/opt/saint-os/firmware/controller by default), owned by `saint`.
#   3. Prunes the previous AppImage so exactly one is served.
#   4. Verifies the checksum survived the copy and that the server's
#      /api/firmware/controller endpoint reports the new version.
#
# The server dir is `saint`-owned, so the copy goes via /tmp + sudo. The
# SSH user needs passwordless sudo on the server (the standard operator
# account does).
#
# Usage:
#   controller/appimage/deploy-to-server.sh                 # build + deploy
#   controller/appimage/deploy-to-server.sh --no-build      # deploy last build
#   controller/appimage/deploy-to-server.sh --host pi.local # override host
#
# Env overrides:
#   SAINT_SERVER_HOST       server hostname (default: opensaint.local)
#   SAINT_SSH_USER          ssh user (default: your ssh default for the host)
#   SAINT_CONTROLLER_FW_DIR server firmware dir
#                           (default: /opt/saint-os/firmware/controller)

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
STAGE_DIR="$REPO_ROOT/server/resources/firmware/controller"

HOST="${SAINT_SERVER_HOST:-opensaint.local}"
SSH_USER="${SAINT_SSH_USER:-}"
DEST_DIR="${SAINT_CONTROLLER_FW_DIR:-/opt/saint-os/firmware/controller}"
SKIP_BUILD=0

while [ $# -gt 0 ]; do
    case "$1" in
        --no-build) SKIP_BUILD=1 ;;
        --host)     HOST="$2"; shift ;;
        --host=*)   HOST="${1#*=}" ;;
        --dir)      DEST_DIR="$2"; shift ;;
        --dir=*)    DEST_DIR="${1#*=}" ;;
        -h|--help)  sed -n '2,40p' "$0" | sed 's/^# \{0,1\}//'; exit 0 ;;
        *) echo "Unknown option: $1" >&2; exit 2 ;;
    esac
    shift
done

TARGET="$HOST"
[ -n "$SSH_USER" ] && TARGET="$SSH_USER@$HOST"

log() { printf '\033[1;36m==> %s\033[0m\n' "$*"; }
die() { printf '\033[1;31mERROR: %s\033[0m\n' "$*" >&2; exit 1; }

sha256() {
    if command -v shasum >/dev/null 2>&1; then shasum -a 256 "$1" | awk '{print $1}';
    else sha256sum "$1" | awk '{print $1}'; fi
}

# ── 1. Build ─────────────────────────────────────────────────────────
if [ "$SKIP_BUILD" -eq 0 ]; then
    log "Building controller AppImage (Docker, linux/amd64)…"
    "$SCRIPT_DIR/build-docker.sh"
else
    log "--no-build: using the already-staged AppImage"
fi

# ── 2. Locate staged artifacts ───────────────────────────────────────
APPIMAGE="$(find "$STAGE_DIR" -maxdepth 1 -type f -name '*.AppImage' -print -quit 2>/dev/null || true)"
[ -n "$APPIMAGE" ] || die "no .AppImage in $STAGE_DIR (run without --no-build first)"
INFO="$STAGE_DIR/info.json"
[ -f "$INFO" ] || die "no info.json in $STAGE_DIR"

NAME="$(basename "$APPIMAGE")"
VERSION="$(python3 -c "import json,sys;print(json.load(open(sys.argv[1]))['latest_version'])" "$INFO")"
LOCAL_SHA="$(sha256 "$APPIMAGE")"
log "Artifact: $NAME"
log "Version:  $VERSION"
log "SHA-256:  $LOCAL_SHA"

# ── 3. Upload to /tmp on the server ──────────────────────────────────
log "Uploading to $TARGET:/tmp …"
scp -o ConnectTimeout=10 "$APPIMAGE" "$INFO" "$TARGET:/tmp/" \
    || die "scp failed — check SSH access to $TARGET"

# ── 4. Install into the serving dir (sudo; saint-owned) + prune ──────
log "Installing into $DEST_DIR (via sudo) and pruning old AppImage(s)…"
REMOTE_SHA="$(
  ssh -o ConnectTimeout=10 "$TARGET" 'bash -s' -- "$NAME" "$DEST_DIR" <<'REMOTE'
set -e
NAME="$1"; DEST="$2"
sudo -n mkdir -p "$DEST"
sudo -n install -m 0755 -o saint -g saint "/tmp/$NAME" "$DEST/$NAME"
sudo -n install -m 0644 -o saint -g saint /tmp/info.json "$DEST/info.json"
# Keep only the just-installed AppImage.
sudo -n find "$DEST" -maxdepth 1 -type f -name '*.AppImage' ! -name "$NAME" -delete
rm -f "/tmp/$NAME" /tmp/info.json
sha256sum "$DEST/$NAME" | awk '{print $1}'
REMOTE
)" || die "remote install failed (passwordless sudo required on $TARGET)"

# ── 5. Verify checksum + OTA endpoint ────────────────────────────────
[ "$REMOTE_SHA" = "$LOCAL_SHA" ] \
    || die "checksum mismatch after copy (local $LOCAL_SHA != remote $REMOTE_SHA)"
log "Checksum verified on server."

log "Verifying OTA endpoint reports $VERSION …"
SERVED="$(
  ssh -o ConnectTimeout=10 "$TARGET" python3 - <<'PY' 2>/dev/null || true
import urllib.request as u, json
try:
    d = json.load(u.urlopen("http://localhost/api/firmware/controller", timeout=6))
    print(d.get("latest_version", ""))
except Exception:
    print("")
PY
)"
if [ "$SERVED" = "$VERSION" ]; then
    log "OTA endpoint serving $VERSION ✓"
else
    printf '\033[1;33mWARNING: OTA endpoint reports "%s" (expected "%s").\n' "$SERVED" "$VERSION"
    printf '         The file is installed in %s but the server may read a\n' "$DEST_DIR"
    printf '         different firmware dir — override with --dir / SAINT_CONTROLLER_FW_DIR.\033[0m\n'
fi

log "Done. Pull the update from the Steam Deck's in-app OTA."
