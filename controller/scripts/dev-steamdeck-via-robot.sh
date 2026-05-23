#!/usr/bin/env bash
#
# Sync the controller source to a Steam Deck that lives on the
# rover's WiFi network, from a dev machine that's on the rover's
# Ethernet network — i.e. the two endpoints can't see each other
# directly, but both can see the rover (opensaint.local).
#
# Strategy: SSH ProxyJump (`ssh -J <jump>`) tunnels every rsync /
# ssh connection through the rover. The rover resolves the Steam
# Deck's mDNS name on its WiFi interface (avahi-daemon, configured
# at install time), so we never have to know the Deck's IP.
#
# Usage:
#   controller/scripts/dev-steamdeck-via-rover.sh
#   controller/scripts/dev-steamdeck-via-rover.sh --rover pi@10.0.0.5
#   controller/scripts/dev-steamdeck-via-rover.sh --deck-host steamdeck-2.local
#
# Env / flag overrides (flags win):
#   ROVER=user@host          jump host (default: pi@opensaint.local)
#   DECK_USER=user           Steam Deck username (default: deck)
#   DECK_HOST=host           Steam Deck hostname on rover's WiFi
#                            (default: steamdeck.local, resolved via
#                            the rover's libnss-mdns on wlan0)
#   REMOTE_DIR=path          where on the Deck to land the files
#                            (default: ~/saintos/controller)
#
# Prereqs:
#   - SSH key from dev machine → rover (so the dev→rover hop is silent)
#   - SSH key from rover → Steam Deck (so the rover→deck hop is silent)
#     One-time setup from rover:
#       ssh-keygen -t ed25519 -N "" -f ~/.ssh/id_ed25519
#       ssh-copy-id deck@steamdeck.local
#   - `fswatch` on the dev machine (auto-installed via Homebrew on
#     macOS if missing; on Linux, `apt install inotify-tools` and
#     swap the watch loop)
#
# What this script does NOT do:
#   - Start a dev server on the Deck. The intended workflow with the
#     Flatpak-deployment model is: this script keeps the source in
#     sync, you ssh in and run `flatpak/build.sh` (or `tauri dev`)
#     when you want to rebuild. Mixing the file-sync loop with a
#     long-lived remote build process turned out to be fragile.

set -euo pipefail

# ---- Defaults + flag parsing -------------------------------------

ROVER="${ROVER:-pi@opensaint.local}"
DECK_USER="${DECK_USER:-deck}"
DECK_HOST="${DECK_HOST:-steamdeck.local}"
REMOTE_DIR="${REMOTE_DIR:-~/saintos/controller}"

while [[ $# -gt 0 ]]; do
    case "$1" in
        --rover)       ROVER="$2";       shift 2 ;;
        --deck-user)   DECK_USER="$2";   shift 2 ;;
        --deck-host)   DECK_HOST="$2";   shift 2 ;;
        --remote-dir)  REMOTE_DIR="$2";  shift 2 ;;
        -h|--help)
            sed -n '2,/^$/p' "$0"  # print the leading comment block
            exit 0
            ;;
        *)
            echo "Unknown option: $1" >&2
            echo "Try --help" >&2
            exit 2
            ;;
    esac
done

DECK="${DECK_USER}@${DECK_HOST}"
LOCAL_DIR="$(cd "$(dirname "$0")/.." && pwd)"

# ---- SSH options that route every hop through the rover ----------

# -J <jump> tells OpenSSH to open the connection through <jump>.
# The destination hostname (DECK_HOST) is resolved BY THE JUMP HOST,
# not the dev machine — which is exactly what we want, because the
# rover can resolve steamdeck.local via mDNS on wlan0 but the dev
# machine can't (different network segment).
#
# ControlMaster + ControlPersist multiplex the SSH connections so
# every sync doesn't re-do the rover handshake.
SSH_OPTS=(
    -J "$ROVER"
    -o ControlMaster=auto
    -o ControlPath="/tmp/ssh-saintdev-%C"
    -o ControlPersist=10m
    -o StrictHostKeyChecking=accept-new
    -o LogLevel=ERROR
)

ssh_cmd_string() {
    # rsync's `-e` flag wants a single shell-quoted string. Build it
    # so spaces in option args (none today, but possible) survive
    # the trip through ssh's argv parsing.
    local parts=("ssh")
    parts+=("${SSH_OPTS[@]}")
    printf '%s ' "${parts[@]}"
}

# ---- Sync ---------------------------------------------------------

echo "=== Steam Deck remote dev (via $ROVER) ==="
echo "Local:    $LOCAL_DIR"
echo "Remote:   $DECK:$REMOTE_DIR"
echo "Jump:     $ROVER"
echo

# Ensure the target directory exists. Don't `mkdir -p` everything
# under it — rsync handles the rest.
ssh "${SSH_OPTS[@]}" "$DECK" "mkdir -p $REMOTE_DIR"

sync_files() {
    echo "[$(date +%H:%M:%S)] syncing → $DECK:$REMOTE_DIR"
    # ADDITIVE ONLY — no --delete. Files only ever flow dev → Deck;
    # nothing on the Deck side gets removed. That keeps Deck-local
    # state intact: flatpak-builder caches, build artifacts, and
    # anything the operator manually left in the tree. Trade-off:
    # renaming a file on the dev machine leaves the old copy on the
    # Deck — rare in practice, and a manual `ssh ... rm` is the right
    # cleanup tool when it happens.
    #
    # Excludes:
    #   node_modules / target / dist / .angular  — heavy build outputs
    #   .git                                     — irrelevant on the Deck
    #   flatpak-build / flatpak-repo             — local build outputs
    #                                              that flatpak/build.sh
    #                                              produces on the Deck
    #   .flatpak-builder                         — flatpak-builder's
    #                                              own cache (SDK
    #                                              extensions etc).
    #                                              Surviving this is
    #                                              what makes Flatpak
    #                                              re-builds fast.
    rsync -avz \
        --exclude 'node_modules' \
        --exclude 'target' \
        --exclude 'dist' \
        --exclude '.angular' \
        --exclude '.git' \
        --exclude 'flatpak-build' \
        --exclude 'flatpak-repo' \
        --exclude '.flatpak-builder' \
        -e "$(ssh_cmd_string)" \
        "$LOCAL_DIR/" "$DECK:$REMOTE_DIR/"
    echo "[$(date +%H:%M:%S)] sync complete"
}

sync_files

# ---- Watch loop ---------------------------------------------------

# fswatch on macOS, inotifywait on Linux. The dev box this script
# runs from is almost always macOS, so fall back to a polite error
# on Linux rather than silently failing.
if command -v fswatch >/dev/null 2>&1; then
    WATCHER=(fswatch -o
        --exclude 'node_modules'
        --exclude 'target'
        --exclude 'dist'
        --exclude '.angular'
        --exclude '.git'
        --exclude 'flatpak-build'
        --exclude 'flatpak-repo'
        "$LOCAL_DIR/src"
        "$LOCAL_DIR/src-tauri/src"
        "$LOCAL_DIR/src-tauri/Cargo.toml"
        "$LOCAL_DIR/src-tauri/tauri.conf.json"
        "$LOCAL_DIR/package.json"
        "$LOCAL_DIR/angular.json"
        "$LOCAL_DIR/tailwind.config.js"
        "$LOCAL_DIR/flatpak"
    )
elif command -v inotifywait >/dev/null 2>&1; then
    echo "fswatch missing on Linux — install with: sudo apt install fswatch" >&2
    echo "Falling back to inotifywait..."
    WATCHER=(inotifywait -m -r -e modify,create,delete,move
        --exclude '(node_modules|target|dist|\.angular|\.git|flatpak-build|flatpak-repo)'
        "$LOCAL_DIR/src"
        "$LOCAL_DIR/src-tauri/src"
        "$LOCAL_DIR/src-tauri/Cargo.toml"
        "$LOCAL_DIR/src-tauri/tauri.conf.json"
        "$LOCAL_DIR/package.json"
        "$LOCAL_DIR/angular.json"
        "$LOCAL_DIR/tailwind.config.js"
        "$LOCAL_DIR/flatpak"
    )
else
    echo "Neither fswatch nor inotifywait found." >&2
    echo "  macOS:  brew install fswatch" >&2
    echo "  Linux:  sudo apt install fswatch  # or inotify-tools" >&2
    exit 1
fi

echo "Watching $LOCAL_DIR for changes (Ctrl-C to stop)…"
echo "On the Deck, rebuild the Flatpak when you want to test:"
echo "  ssh -J $ROVER $DECK '$REMOTE_DIR/flatpak/build.sh'"
echo

"${WATCHER[@]}" | while read -r _; do
    sync_files
done
