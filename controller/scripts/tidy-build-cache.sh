#!/usr/bin/env bash
#
# Prune accumulated Rust build artifacts under controller/src-tauri/target.
#
# WHY THIS EXISTS: Cargo never garbage-collects target/debug/deps — every
# past dependency version and every dead incremental cache just piles up
# (we saw this hit 6.6 GB). `cargo clean` would fix it but nukes the
# whole cache, forcing a slow from-scratch rebuild next time. Instead we
# use cargo-sweep to remove only the STALE artifacts (unused for N days,
# or built by a toolchain that's no longer installed) while leaving the
# current working set intact — so the next build stays incremental.
#
# Usage:
#   ./tidy-build-cache.sh              # sweep stale artifacts (safe)
#   ./tidy-build-cache.sh --install    # install cargo-sweep first if missing
#   ./tidy-build-cache.sh --days 30    # change the staleness threshold
#   ./tidy-build-cache.sh --deep       # full `cargo clean` (forces rebuild)
#
# Wired into `npm run tidy` (with --install) and as a `posttauri:build`
# hook (best-effort, no install) so heavy local builds self-tidy. The
# script never exits non-zero on a missing tool, so build chains that
# call it can't fail because of it.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
TAURI_DIR="$(cd "$SCRIPT_DIR/../src-tauri" && pwd)"
TARGET_DIR="$TAURI_DIR/target"

DAYS="${TIDY_DAYS:-14}"
DEEP=0
INSTALL=0
while [ $# -gt 0 ]; do
    case "$1" in
        --deep)    DEEP=1 ;;
        --install) INSTALL=1 ;;
        --days)    shift; DAYS="${1:-14}" ;;
        --days=*)  DAYS="${1#*=}" ;;
        *) echo "unknown arg: $1" >&2 ;;
    esac
    shift
done

size_of() { du -sh "$1" 2>/dev/null | cut -f1; }

if [ ! -d "$TARGET_DIR" ]; then
    echo "Nothing to tidy — $TARGET_DIR doesn't exist yet."
    exit 0
fi

BEFORE="$(size_of "$TARGET_DIR")"
echo "==> target before: ${BEFORE:-?}  ($TARGET_DIR)"

# --deep: blunt reset. Reclaims everything but the next build is full.
if [ "$DEEP" = "1" ]; then
    echo "==> --deep: cargo clean (next build will be from scratch)"
    ( cd "$TAURI_DIR" && cargo clean )
    echo "==> target after:  $(size_of "$TARGET_DIR")"
    exit 0
fi

# Ensure cargo-sweep is available. Only auto-install when explicitly
# asked (--install) — `npm run tidy` passes it, the build hook does not,
# so a normal build never triggers a surprise `cargo install`.
if ! command -v cargo-sweep >/dev/null 2>&1; then
    if [ "$INSTALL" = "1" ]; then
        echo "==> cargo-sweep not found — installing (cargo install cargo-sweep)…"
        if ! cargo install cargo-sweep; then
            echo "WARN: cargo-sweep install failed; leaving cache untouched." >&2
            exit 0
        fi
    else
        echo "NOTE: cargo-sweep not installed — skipping prune."
        echo "      Run 'npm run tidy' (installs it) or 'cargo install cargo-sweep'."
        echo "      For an immediate full reset: $0 --deep"
        exit 0
    fi
fi

# Time-based prune covers deps/ AND incremental/: anything not touched in
# DAYS days goes. --installed additionally drops artifacts from toolchains
# that are no longer installed (e.g. after a `rustup update`).
echo "==> cargo sweep --time $DAYS"
( cd "$TAURI_DIR" && cargo sweep --time "$DAYS" ) || true
echo "==> cargo sweep --installed (drop dead-toolchain artifacts)"
( cd "$TAURI_DIR" && cargo sweep --installed ) || true

echo "==> target after:  $(size_of "$TARGET_DIR")"
