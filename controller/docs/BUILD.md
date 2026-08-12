# SAINT.OS Controller — Build

Building the controller from source. Most people never need this — see
[`INSTALL.md`](INSTALL.md) for the install-first path. The repo-wide build index
is [`../../docs/BUILD.md`](../../docs/BUILD.md); this is the controller-specific
detail.

## How it ships

The controller is built into a single self-contained `.AppImage` by the
linux/amd64 Docker pipeline in `controller/appimage/`. The AppImage bundles its
own webkit2gtk-4.1, GTK, and libsoup, plus a small `LD_PRELOAD` shim that remaps
webkit's hardcoded helper-process path to the bundled equivalent — so it runs on
a stock SteamOS without any system dependencies.

## Building locally

The build runs in a linux/amd64 Docker container so the output is x86_64
regardless of the host. On Apple Silicon, Docker Desktop's Rosetta emulation
handles the architecture mismatch:

```bash
controller/appimage/build-docker.sh
```

First clean build is ~15–25 min (cargo deps + the Vue frontend). Subsequent
builds reuse the persistent cache at `~/.cache/saint-os/controller-appimage/`
(cargo registry, target/, node_modules, npm cache) and finish in minutes for
small edits.

Output:
- `server/resources/firmware/controller/saint_firmware_controller_<version>-local.<sha>.AppImage`
- `server/resources/firmware/controller/info.json` (matches the server's
  `/api/firmware/controller` endpoint schema)

CI builds the same artifact natively via `.github/workflows/dist.yml`'s
`appimage-controller` job — the same `controller/appimage/build-bundle.sh` runs
in both places, so behavior stays in lockstep.

## Developer mode (no AppImage)

For tight inner-loop development on a laptop you don't need to bundle. Install
the prereqs for your host OS and run `npm run tauri dev`.

### macOS

```bash
# Rust + Node
curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh
source ~/.cargo/env
brew install node

# Build & run
cd controller
npm install
npm run tauri dev
```

### Linux (Ubuntu / Debian)

```bash
sudo apt install build-essential curl wget pkg-config libssl-dev libudev-dev \
    libgtk-3-dev libwebkit2gtk-4.1-dev libayatana-appindicator3-dev librsvg2-dev \
    nodejs npm
curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh
source ~/.cargo/env

cd controller && npm install && npm run tauri dev
```

### Steam Deck developer mode

`npm run tauri dev` does work on the Deck if you really want it, but it requires
disabling read-only root and installing a Rust toolchain into the system — which
SteamOS wipes on every OS update. The AppImage flow is the supported path. The
native-dev recipe is preserved in `controller/scripts/setup-steamdeck.sh` for
the rare case you need it.

## Build commands

| Command | Effect |
|---|---|
| `controller/appimage/build-docker.sh` | Build the .AppImage in linux/amd64 Docker (incremental) |
| `controller/appimage/build-docker.sh --rebuild-image` | Re-run the Dockerfile (after deps change) |
| `controller/appimage/build-docker.sh --clean` | Wipe the persistent build cache and start fresh |
| `npm run tauri dev` | Native dev mode (hot reload, macOS / Linux desktops) |
| `npm run tauri build` | Native production build (host-OS bundle, not AppImage) |
| `npm run tidy` | Prune stale Rust build artifacts in `src-tauri/target` (installs cargo-sweep on first run) |

### Keeping the build cache tidy

Cargo never garbage-collects `src-tauri/target/debug/deps` or the `incremental/`
caches — every old dependency version and dead toolchain artifact just
accumulates (easily several GB). Two ways to reclaim it:

- `npm run tidy` — uses [`cargo-sweep`](https://github.com/holmgr/cargo-sweep)
  to remove artifacts unused for >14 days and those from uninstalled toolchains,
  while leaving the current working set so the next build stays incremental.
  `npm run tauri:build` runs this automatically afterward (best-effort; skipped
  if cargo-sweep isn't installed).
- `bash scripts/tidy-build-cache.sh --deep` — `cargo clean`: reclaims everything
  immediately, but the next build is from scratch.

The Docker dist build keeps its cache in a separate dir; reset it with
`build-docker.sh --clean`.

## Troubleshooting

### Build fails on Apple Silicon with `Exec format error`

Docker Desktop's Rosetta emulation has trouble exec'ing the AppImage runtime
stub used by linuxdeploy / appimagetool. The build pipeline already works around
this — the Dockerfile pre-installs patched copies with the AppImage magic byte
zeroed. If you see this error, you're likely running an old Docker image; pass
`--rebuild-image`.

### `npm ci` fails inside the AppImage build

`package.json` and `package-lock.json` are out of sync in the source tree. The
build falls back to `npm install` automatically so this run goes through, but
it'll prompt every build until you fix it. On the host (not inside the
container):

```bash
cd controller && npm install
git add package-lock.json && git commit
```
