#!/usr/bin/env bash
#
# Run the controller's Rust test suite inside a Linux container.
#
# Why: parts of the controller (the Steam Deck HID parser in
# src/input/steamdeck_hid.rs) are #[cfg(target_os = "linux")], so they
# don't compile — and their tests don't run — on a macOS/Windows dev box.
# This builds the Linux toolchain image and runs `cargo test` in it, so
# those tests are exercised everywhere. CI calls the same script
# (.github/workflows/controller-tests.yml) so local and CI stay in lockstep.
#
# Usage (from anywhere):
#   controller/ci/run-linux-tests.sh                # full lib test suite
#   controller/ci/run-linux-tests.sh steamdeck_hid  # just the HID parser
#
# Any args are passed through as a cargo test filter.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
IMAGE="${SAINT_CTRL_TEST_IMAGE:-saint-ctrl-linux-test}"

echo "[run-linux-tests] building $IMAGE"
docker build -t "$IMAGE" -f "$SCRIPT_DIR/Dockerfile.linux-test" "$SCRIPT_DIR"

# tauri-build (invoked by build.rs during `cargo test`) requires the
# configured frontendDist (../dist) to exist. `cargo test` does NOT run
# the beforeBuildCommand that normally produces it, and a real frontend
# isn't needed to compile/run the Rust unit tests — so drop a minimal
# placeholder when none is present (e.g. a fresh CI checkout). A
# developer's already-built dist/ is left untouched.
DIST="$REPO_ROOT/controller/dist"
if [ ! -f "$DIST/index.html" ]; then
  echo "[run-linux-tests] no controller/dist — writing a placeholder for tauri-build"
  mkdir -p "$DIST"
  printf '<!doctype html><meta charset="utf-8"><title>test</title>\n' > "$DIST/index.html"
fi

# Named volumes persist the cargo registry + the Linux target dir across
# runs (fast re-runs locally; cold but correct in CI). CARGO_TARGET_DIR
# keeps the container's Linux artifacts out of the host's macOS target/.
echo "[run-linux-tests] cargo test --lib ${*:-(all)}"
exec docker run --rm \
  -v "$REPO_ROOT":/work -w /work/controller/src-tauri \
  -e CARGO_TARGET_DIR=/cargo-target \
  -v saint_cargo_registry:/root/.cargo/registry \
  -v saint_linux_target:/cargo-target \
  "$IMAGE" \
  cargo test --lib "$@" -- --nocapture
