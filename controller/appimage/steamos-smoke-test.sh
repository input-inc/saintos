#!/usr/bin/env bash
# Launch the controller AppImage on SteamOS's own userland and check that
# its webview actually comes up.
#
# Usage: steamos-smoke-test.sh <AppImage | AppDir> [--image steamos-rootfs:<ver>]
#                              [--seconds N] [--expect-failure]
#
# An unpacked AppDir works too (its AppRun is launched), for trying a
# change to the bundle without re-sealing it.
#
# The SteamOS image comes from steamos-rootfs.sh (the root filesystem of
# Valve's recovery image, imported into Docker), so the AppImage loads
# against the Deck's glibc / Mesa / GTK / libwayland exactly as it would
# on a Deck. SteamOS ships no virtual X server, so Xvfb runs in a sidecar
# container and shares its socket. What this does NOT cover: gamescope,
# Steam's launcher, the GPU.
#
# Pass = after --seconds the app is still running, WebKitNetworkProcess
# and WebKitWebProcess are running, the frontend has logged at least one
# line (its JavaScript ran and reached Rust over IPC), and the log has
# neither of the two failures found on SteamOS:
#   - "Failed to spawn child process ././/lib/.../WebKitNetworkProcess":
#     webkit couldn't find its helpers (what a user hit on a Deck).
#   - "Could not create default EGL display": the bundled libwayland
#     shadowed the one the host's Mesa needs; WebKitWebProcess aborted.
# --expect-failure inverts the verdict, to prove the test catches a
# known-broken build.
set -euo pipefail

APPIMAGE=""
IMAGE="steamos-rootfs:latest"
SECONDS_TO_RUN=30
EXPECT_FAILURE=0
while [[ $# -gt 0 ]]; do
    case "$1" in
        --image) IMAGE="$2"; shift 2 ;;
        --seconds) SECONDS_TO_RUN="$2"; shift 2 ;;
        --expect-failure) EXPECT_FAILURE=1; shift ;;
        -h|--help) sed -n '2,27p' "$0"; exit 0 ;;
        *) APPIMAGE="$1"; shift ;;
    esac
done
[[ -e "$APPIMAGE" ]] || { echo "usage: $0 <AppImage|AppDir> [--image IMG] [--seconds N] [--expect-failure]" >&2; exit 2; }

# Under $HOME off CI: Docker Desktop on macOS doesn't share $TMPDIR
# (/var/folders) or /tmp into containers, so /app would come up empty.
WORK_BASE="${RUNNER_TEMP:-$HOME/.cache/saint-os}"
mkdir -p "$WORK_BASE"
WORK=$(mktemp -d "$WORK_BASE/steamos-smoke.XXXXXX")
APPIMAGE_ABS=$(cd "$(dirname "$APPIMAGE")" && pwd)/$(basename "$APPIMAGE")
chmod 755 "$WORK"
if [[ -d "$APPIMAGE_ABS" ]]; then
    APP_MOUNT=(-v "$APPIMAGE_ABS:/app/appdir:ro")
    APP_CMD=/app/appdir/AppRun
else
    cp "$APPIMAGE_ABS" "$WORK/app.AppImage"
    chmod 755 "$WORK/app.AppImage"
    # Zero the "AI\x02" AppImage marker in the ELF header padding (bytes
    # 8-10) of our copy. A real x86 kernel ignores those bytes, but Rosetta
    # and qemu-user (a Mac's Docker) refuse the binary with "Exec format
    # error". The runtime never reads them, so this changes nothing on CI.
    printf '\0\0\0' | dd of="$WORK/app.AppImage" bs=1 seek=8 count=3 conv=notrunc status=none
    APP_MOUNT=()
    APP_CMD=/app/app.AppImage
fi
TAG="saint-smoke-$$"
X11_VOL="$TAG-x11"

cleanup() {
    docker rm -f "$TAG-app" "$TAG-xvfb" >/dev/null 2>&1 || true
    docker volume rm "$X11_VOL" >/dev/null 2>&1 || true
    rm -rf "$WORK"
}
trap cleanup EXIT

echo "==> SteamOS: $(docker run --rm --platform linux/amd64 "$IMAGE" sh -c '. /etc/os-release; echo "$PRETTY_NAME ${VERSION_ID:-} (build ${BUILD_ID:-?})"')"

docker volume create "$X11_VOL" >/dev/null
echo "==> Starting Xvfb sidecar"
docker run -d --platform linux/amd64 --name "$TAG-xvfb" -v "$X11_VOL:/tmp/.X11-unix" debian:trixie-slim bash -c '
    apt-get update -qq >/dev/null && DEBIAN_FRONTEND=noninteractive \
        apt-get install -y -qq --no-install-recommends xvfb >/dev/null &&
    chmod 1777 /tmp/.X11-unix &&
    exec Xvfb :99 -screen 0 1280x800x24 -nolisten tcp' >/dev/null
for _ in $(seq 1 120); do
    docker exec "$TAG-xvfb" test -S /tmp/.X11-unix/X99 2>/dev/null && break
    sleep 1
done
docker exec "$TAG-xvfb" test -S /tmp/.X11-unix/X99 \
    || { docker logs "$TAG-xvfb" >&2; echo "ERROR: Xvfb did not start" >&2; exit 1; }

# As the Deck's own user (deck = uid 1000), from a cwd outside the
# AppImage — the way Steam or a double-click launches it. No FUSE in a
# container, so the runtime extracts and runs instead of mounting; AppRun
# and the libraries it loads are the same either way.
echo "==> Launching $(basename "$APPIMAGE_ABS") as uid 1000 for ${SECONDS_TO_RUN}s"
docker run -d --platform linux/amd64 --name "$TAG-app" --user 1000:1000 \
    -v "$X11_VOL:/tmp/.X11-unix" -v "$WORK:/app" ${APP_MOUNT[@]+"${APP_MOUNT[@]}"} \
    -e DISPLAY=:99 -e HOME=/tmp/home -e XDG_RUNTIME_DIR=/tmp/xdg \
    -e APPIMAGE_EXTRACT_AND_RUN=1 -e TMPDIR=/tmp -e APP_CMD="$APP_CMD" -w /tmp \
    "$IMAGE" sh -c 'mkdir -p /tmp/home /tmp/xdg && chmod 700 /tmp/xdg && exec "$APP_CMD"' >/dev/null

sleep "$SECONDS_TO_RUN"

# argv[0] of every process in the app container. /proc needs no tools in
# the image; cmdline rather than exe because under Rosetta (a Mac's
# Docker) every exe reads /run/rosetta/rosetta.
PROCS=$(docker exec "$TAG-app" sh -c 'for p in /proc/[0-9]*; do tr "\0" "\n" < "$p/cmdline" | head -n1; done 2>/dev/null' 2>/dev/null || true)
RUNNING=$(docker inspect -f '{{.State.Running}}' "$TAG-app")
LOG="$WORK/app.log"
docker logs "$TAG-app" > "$LOG" 2>&1 || true

# (Each listing tolerates finding nothing: under pipefail an empty grep
# would end the script before the verdict.)
echo "==> Processes:"
{ printf '%s\n' "$PROCS" | grep -v '^$' || echo "(none — container not running)"; } | sort | uniq -c | sed 's/^/    /'
echo "==> App log (warnings / errors):"
{ grep -E 'ERROR|CRITICAL|WARN|spawn|WebKit|EGL' "$LOG" || true; } | cut -c1-240 | head -40 | sed 's/^/    /'

fail=()
[[ "$RUNNING" == "true" ]] || fail+=("the app exited (code $(docker inspect -f '{{.State.ExitCode}}' "$TAG-app"))")
grep -q 'Failed to spawn' "$LOG" && fail+=("webkit failed to spawn a helper process")
grep -q 'Could not create default EGL display' "$LOG" && fail+=("WebKitWebProcess could not initialise EGL")
grep -q '\]\[frontend\]\[' "$LOG" || fail+=("the frontend never logged (its JavaScript didn't run)")
grep -q 'WebKitNetworkProcess' <<<"$PROCS" || fail+=("WebKitNetworkProcess is not running")
grep -q 'WebKitWebProcess' <<<"$PROCS" || fail+=("WebKitWebProcess is not running")

if (( ${#fail[@]} )); then
    printf '==> Smoke test FAILED:\n'; printf '    - %s\n' "${fail[@]}"
    if (( EXPECT_FAILURE )); then echo "==> ...as expected (--expect-failure)"; exit 0; fi
    echo "==> Full app log:"; sed 's/^/    /' "$LOG" | tail -80
    exit 1
fi
echo "==> Smoke test PASSED: webview helpers up, frontend running, app still running"
if (( EXPECT_FAILURE )); then echo "ERROR: expected this build to fail" >&2; exit 1; fi
