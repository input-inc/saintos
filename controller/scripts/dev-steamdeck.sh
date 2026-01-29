#!/bin/bash

# Steam Deck Remote Development Script
# Usage: ./scripts/dev-steamdeck.sh [steamdeck-ip] [steamdeck-user]
#
# Prerequisites on Steam Deck:
#   1. Enable Developer Mode and SSH
#   2. Install Rust: curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh
#   3. Install Node.js: sudo pacman -S nodejs npm
#   4. Install build deps: sudo pacman -S webkit2gtk base-devel

STEAMDECK_IP="${1:-steamdeck.local}"
STEAMDECK_USER="${2:-deck}"
REMOTE_DIR="~/Projects/saint-controller"
LOCAL_DIR="$(cd "$(dirname "$0")/.." && pwd)"

echo "=== Steam Deck Remote Development ==="
echo "Local:  $LOCAL_DIR"
echo "Remote: $STEAMDECK_USER@$STEAMDECK_IP:$REMOTE_DIR"
echo ""

# Check if fswatch is installed (for file watching on macOS)
if ! command -v fswatch &> /dev/null; then
    echo "Installing fswatch for file watching..."
    brew install fswatch
fi

# Initial sync
sync_files() {
    echo "[$(date +%H:%M:%S)] Syncing files to Steam Deck..."
    rsync -avz --delete \
        --exclude 'node_modules' \
        --exclude 'target' \
        --exclude 'dist' \
        --exclude '.angular' \
        --exclude '.git' \
        "$LOCAL_DIR/" "$STEAMDECK_USER@$STEAMDECK_IP:$REMOTE_DIR/"
    echo "[$(date +%H:%M:%S)] Sync complete"
}

# Build and run on Steam Deck
build_and_run() {
    echo "[$(date +%H:%M:%S)] Building on Steam Deck..."
    ssh "$STEAMDECK_USER@$STEAMDECK_IP" "cd $REMOTE_DIR && npm install && npm run tauri:dev"
}

# Initial sync
sync_files

# Start the app on Steam Deck in background
echo ""
echo "Starting app on Steam Deck..."
echo "Press Ctrl+C to stop watching for changes"
echo ""

# Run build in background and capture PID
ssh "$STEAMDECK_USER@$STEAMDECK_IP" "cd $REMOTE_DIR && npm install"

# Start dev server on Steam Deck (in background via screen or tmux)
ssh "$STEAMDECK_USER@$STEAMDECK_IP" "cd $REMOTE_DIR && pkill -f 'tauri dev' 2>/dev/null; nohup npm run tauri:dev > /tmp/tauri-dev.log 2>&1 &"

echo "Dev server started on Steam Deck. Watching for changes..."
echo "View logs: ssh $STEAMDECK_USER@$STEAMDECK_IP 'tail -f /tmp/tauri-dev.log'"
echo ""

# Watch for file changes and sync
fswatch -o \
    --exclude 'node_modules' \
    --exclude 'target' \
    --exclude 'dist' \
    --exclude '.angular' \
    --exclude '.git' \
    "$LOCAL_DIR/src" \
    "$LOCAL_DIR/src-tauri/src" \
    "$LOCAL_DIR/src-tauri/Cargo.toml" \
    "$LOCAL_DIR/package.json" \
    "$LOCAL_DIR/angular.json" \
    "$LOCAL_DIR/tailwind.config.js" \
| while read -r; do
    sync_files
    # Tauri dev server should auto-reload on file changes
done
