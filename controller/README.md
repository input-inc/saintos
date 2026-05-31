# SAINT.OS Controller

A Tauri-based controller application for SAINT.OS that captures gamepad,
trigger, touch, gyro, and touchscreen input and forwards it to the SAINT.OS
server over WebSocket.

- **Frontend:** Angular 19 + TypeScript + Tailwind CSS
- **Backend:** Rust (Tauri 2.0)
- **Primary platform:** Steam Deck (SteamOS) — single-file AppImage, launched
  from Game Mode as a Non-Steam Game
- **Secondary platforms:** macOS / Linux / Windows (developer mode only)

## How it ships

The controller is built into a single self-contained `.AppImage` file by the
linux/amd64 Docker pipeline in `controller/appimage/`. The AppImage bundles
its own webkit2gtk-4.1, GTK, libsoup, and a small LD_PRELOAD shim that
remaps webkit's hardcoded helper-process path to the bundled equivalent
— see [docs/APPIMAGE_MIGRATION.md](docs/APPIMAGE_MIGRATION.md) for why
that shim exists and what it does.

End user (Deck operator) never touches a toolchain. They get a `.AppImage`
file from one of two places:

1. **Built locally**, via `controller/appimage/build-docker.sh` on a Mac
   or Linux dev machine, then `scp`'d to the Deck.
2. **OTA-fetched**, from a SAINT.OS server: the controller's Settings tab
   polls `/api/firmware/controller` for newer versions and atomically
   replaces the running AppImage in place (wherever the operator put it).

Either way, the operator-side install is the same atomic-file-replace
flow: write the file, `chmod +x`, drop a `.desktop` entry. No portal
calls, no package manager, no sandbox shell game.

## First-time install on a Steam Deck

### 1. Get the AppImage onto the Deck

From your dev machine (or wherever you have the built artifact):

```bash
ssh deck@steamdeck.local mkdir -p ~/Applications
scp server/resources/firmware/controller/saint_firmware_controller_*.AppImage \
    deck@steamdeck.local:~/Applications/SAINT-Controller.AppImage
```

`~/Applications/` is the conventional AppImage location on Linux and is
visible in Dolphin's home view, which makes manual swaps easy. SteamOS
doesn't create it by default — hence the `mkdir -p` above.

### 2. Mark it executable and try a launch

```bash
ssh deck@steamdeck.local
chmod +x ~/Applications/SAINT-Controller.AppImage
~/Applications/SAINT-Controller.AppImage
```

The controller window should come up. (GTK module warnings about
`canberra-gtk-module`, `colorreload-gtk-module`, and
`window-decorations-gtk-module` are non-fatal noise — those are KDE-side
modules GTK probes for and skips when missing.)

### 3. Add it to Steam as a Non-Steam Game

In Desktop Mode, **Steam → Games → Add a Non-Steam Game to My Library →
BROWSE…** and pick:

```
/home/deck/Applications/SAINT-Controller.AppImage
```

Rename the entry to **SAINT Controller** in the library so the
artwork-setup script in the next step finds it.

### 4. Set the Steam library artwork (optional)

`set-steamdeck-artwork.py` finds the Steam shortcut by name and writes
the bundled hero + capsule PNGs into Steam's grid dir.

```bash
~/Applications/SAINT-Controller.AppImage \
    --appimage-extract-and-run saint-controller-artwork-setup
```

Then restart Steam (`steam -shutdown && steam &`). The hero banner and
vertical capsule should appear on the library page. Pass
`--name-pattern "<Your Custom Name>"` if you used a different name.

### 5. Launch in Game Mode

Switch back to Game Mode. The controller appears under **Non-Steam →
SAINT Controller**. Press A.

On first launch, point it at the SAINT.OS server (default
`ws://opensaint.local/api/ws`, password `12345`) and accept the prompt.
The connection setting persists under `~/.config/saint-controller/`.

## Updating

### From the server's OTA flow (recommended)

The controller's **Settings** tab polls `/api/firmware/controller` on the
configured SAINT.OS server. When the server has a newer
`saint_firmware_controller_*.AppImage` than what's running, a banner
appears: **Update available — Install**. The flow downloads, verifies
SHA-256, then atomically replaces the running AppImage at its current
location (typically `~/Applications/SAINT-Controller.AppImage`). The
Steam shortcut keeps working — the file path doesn't change.

Operator-visible UX: click Install, wait for "Update installed. Please
manually relaunch the SAINT Controller", relaunch from the Steam tile.

### Manual replacement

`scp` a newer AppImage over the existing one. `chmod +x` it. Relaunch.

## Building locally

The build runs in a linux/amd64 Docker container so the output is x86_64
regardless of the host. On Apple Silicon, Docker Desktop's Rosetta
emulation handles the architecture mismatch:

```bash
controller/appimage/build-docker.sh
```

First clean build is ~15–25 min (cargo deps + Angular). Subsequent builds
reuse the persistent cache at `~/.cache/saint-os/controller-appimage/`
(cargo registry, target/, node_modules, npm cache) and finish in
minutes for small edits.

Output:
- `server/resources/firmware/controller/saint_firmware_controller_<version>-local.<sha>.AppImage`
- `server/resources/firmware/controller/info.json` (matches the server's
  `/api/firmware/controller` endpoint schema)

CI builds the same artifact natively via `.github/workflows/dist.yml`'s
`appimage-controller` job — same `controller/appimage/build-bundle.sh`
runs in both places, so behavior stays in lockstep.

## Developer mode (no AppImage)

For tight inner-loop development on a laptop you don't need to bundle.
Install the prereqs for your host OS and run `npm run tauri dev`.

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

`npm run tauri dev` does work on the Deck if you really want it, but it
requires disabling read-only root and installing a Rust toolchain into
the system — which SteamOS wipes on every OS update. The AppImage flow
is the supported path. The native-dev recipe is preserved in
`controller/scripts/setup-steamdeck.sh` for the rare case you need it.

## Build commands

| Command | Effect |
|---|---|
| `controller/appimage/build-docker.sh` | Build the .AppImage in linux/amd64 Docker (incremental) |
| `controller/appimage/build-docker.sh --rebuild-image` | Re-run the Dockerfile (after deps change) |
| `controller/appimage/build-docker.sh --clean` | Wipe the persistent build cache and start fresh |
| `npm run tauri dev` | Native dev mode (hot reload, macOS / Linux desktops) |
| `npm run tauri build` | Native production build (host-OS bundle, not AppImage) |

## Documentation

- [docs/BINDINGS_SYSTEM.md](docs/BINDINGS_SYSTEM.md) — input → action
  data model (binding profiles, preset panels, action types)
- [docs/SHEETS_BINDINGS.md](docs/SHEETS_BINDINGS.md) — how to bind a
  controller input to a routing-sheet WebSocket input on the server
  (the end-to-end "make my joystick drive this servo" walkthrough)

## Troubleshooting

### Build fails on Apple Silicon with `Exec format error`

Docker Desktop's Rosetta emulation has trouble exec'ing the AppImage
runtime stub used by linuxdeploy / appimagetool. The build pipeline
already works around this — the Dockerfile pre-installs patched copies
with the AppImage magic byte zeroed. If you see this error, you're
likely running an old Docker image; pass `--rebuild-image`.

### AppImage launches but the webview is blank

Check `~/Applications/SAINT-Controller.AppImage` is the actual current
build (compare with the server's `info.json` checksum). Tauri's release
binary embeds the production frontend, so a blank window usually means
the binary is stale relative to what the controller's UI expects from
the server.

### `npm ci` fails inside the AppImage build

`package.json` and `package-lock.json` are out of sync in the source
tree. The build falls back to `npm install` automatically so this run
goes through, but it'll prompt every build until you fix it. On the
host (not inside the container):

```bash
cd controller && npm install
git add package-lock.json && git commit
```

### Controller doesn't appear in Game Mode

Verify the Steam shortcut exists
(`grep SAINT ~/.local/share/Steam/userdata/*/config/shortcuts.vdf` —
the file is binary but the name will be readable). If it's missing,
re-do step 3 in Desktop Mode. Steam must be restarted after editing
shortcuts.
