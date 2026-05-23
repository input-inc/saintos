#!/usr/bin/env bash
#
# Sync the controller source to a Steam Deck for development. Works
# in two topologies, selected by the presence of --robot / $ROBOT:
#
#   Direct (default):
#     Dev machine can reach the Deck directly. No jump host.
#     Example:
#       controller/scripts/dev-steamdeck.sh
#       controller/scripts/dev-steamdeck.sh --deck-host steamdeck-2.local
#
#   Via the robot (proxy-jump):
#     Dev machine is on the robot's Ethernet, Deck is on the robot's
#     WiFi — the two endpoints can't see each other but both can see
#     the robot. SSH ProxyJump (`ssh -J <robot>`) tunnels every rsync
#     and ssh through the robot; the robot resolves the Deck's mDNS
#     name on its WiFi interface, so we never have to know its IP.
#     Example:
#       controller/scripts/dev-steamdeck.sh --robot pi@opensaint.local
#       ROBOT=pi@opensaint.local controller/scripts/dev-steamdeck.sh
#
# Env / flag overrides (flags win):
#   ROBOT=user@host    Optional jump host. When set, every SSH/rsync
#                      hops through it via `ssh -J`. Unset → direct.
#   DECK_USER=user     Steam Deck username (default: deck)
#   DECK_HOST=host     Steam Deck hostname or IP (default: steamdeck.local).
#                      When ROBOT is set, this is resolved by the robot —
#                      so mDNS .local names work even when the dev
#                      machine can't see the Deck's network directly.
#   REMOTE_DIR=path    where on the Deck to land the files
#                      (default: ~/saintos/controller)
#
# Prereqs:
#   - SSH key from dev machine → Deck (or → robot, if via-robot):
#       ssh-copy-id deck@steamdeck.local
#       (and ssh-copy-id pi@opensaint.local if using --robot)
#   - When using --robot: SSH key from robot → Deck. One-time, from
#     the robot:
#       ssh-keygen -t ed25519 -N "" -f ~/.ssh/id_ed25519
#       ssh-copy-id deck@steamdeck.local
#   - `fswatch` on the dev machine. macOS: `brew install fswatch`.
#     Linux: `apt install fswatch` or `inotify-tools` (auto-detected).
#
# What this script does NOT do:
#   - Start a dev server on the Deck. The intended workflow with the
#     Flatpak-deployment model is: this script keeps the source in
#     sync, you ssh in and run `flatpak/build.sh` (or `tauri dev`)
#     when you want to rebuild. Mixing the file-sync loop with a
#     long-lived remote build process turned out to be fragile.

set -euo pipefail

# ---- Defaults + flag parsing -------------------------------------

ROBOT="${ROBOT:-}"
DECK_USER="${DECK_USER:-deck}"
DECK_HOST="${DECK_HOST:-steamdeck.local}"
REMOTE_DIR="${REMOTE_DIR:-~/saintos/controller}"

while [[ $# -gt 0 ]]; do
    case "$1" in
        --robot)       ROBOT="$2";       shift 2 ;;
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

# ---- SSH options -------------------------------------------------

# ControlMaster + ControlPersist multiplex SSH connections so every
# sync reuses an open socket instead of doing the full handshake.
# Worth ~hundreds of ms per sync, especially with a jump host where
# the handshake is two-stage.
#
# StrictHostKeyChecking=accept-new auto-adds first-seen host keys
# (so a fresh checkout doesn't prompt) but still refuses if the key
# CHANGES (so a MITM still trips the alarm).
SSH_OPTS=(
    -o ControlMaster=auto
    -o ControlPath="/tmp/ssh-saintdev-%C"
    -o ControlPersist=10m
    -o StrictHostKeyChecking=accept-new
    -o LogLevel=ERROR
)

# -J <jump> tells OpenSSH to open the connection through <jump>. The
# destination hostname (DECK_HOST) is resolved BY THE JUMP HOST, not
# the dev machine — which is exactly what we want when the dev box
# can't see the Deck's network directly but the jump host can. Only
# added when ROBOT is set; direct mode skips it entirely.
if [[ -n "$ROBOT" ]]; then
    SSH_OPTS+=(-J "$ROBOT")
fi

ssh_cmd_string() {
    # rsync's `-e` flag wants a single shell-quoted string. Build it
    # so spaces in option args (none today, but possible) survive
    # the trip through ssh's argv parsing.
    local parts=("ssh")
    parts+=("${SSH_OPTS[@]}")
    printf '%s ' "${parts[@]}"
}

# ---- Sync ---------------------------------------------------------

if [[ -n "$ROBOT" ]]; then
    echo "=== Steam Deck remote dev (via $ROBOT) ==="
else
    echo "=== Steam Deck remote dev (direct) ==="
fi
echo "Local:    $LOCAL_DIR"
echo "Remote:   $DECK:$REMOTE_DIR"
[[ -n "$ROBOT" ]] && echo "Jump:     $ROBOT"
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
        --exclude '.flatpak-cache' \
        --exclude 'SAINT-Controller.flatpak' \
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
        --exclude '.flatpak-cache'
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
        --exclude '(node_modules|target|dist|\.angular|\.git|\.flatpak-cache)'
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
if [[ -n "$ROBOT" ]]; then
    echo "On the Deck, rebuild the Flatpak when you want to test:"
    echo "  ssh -J $ROBOT $DECK '$REMOTE_DIR/flatpak/build.sh'"
else
    echo "On the Deck, rebuild the Flatpak when you want to test:"
    echo "  ssh $DECK '$REMOTE_DIR/flatpak/build.sh'"
fi
echo

"${WATCHER[@]}" | while read -r _; do
    sync_files
done
