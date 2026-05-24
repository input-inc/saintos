# SAINT.OS Controller

A Tauri-based controller application for SAINT.OS that captures gamepad,
trigger, touch, gyro, and touchscreen input and forwards it to the SAINT.OS
server over WebSocket.

- **Frontend:** Angular 19 + TypeScript + Tailwind CSS
- **Backend:** Rust (Tauri 2.0)
- **Primary platform:** Steam Deck (SteamOS) — Flatpak install, launched from
  Game Mode as a Non-Steam Game
- **Secondary platforms:** macOS / Linux / Windows (developer mode only)

## Steam Deck deployment (the canonical install)

Everything you need is in `controller/flatpak/`:

- `com.saintos.Controller.yml` — Flatpak manifest
- `build.sh` — wrapper that installs the runtime, builds the Flatpak, and
  pins all caches under `/home` so SteamOS's tiny `/var` partition doesn't
  fill up
- `com.saintos.Controller.desktop` / `.metainfo.xml` — desktop integration

You do **not** need to disable `steamos-readonly`, install pacman packages,
or set up a Rust toolchain on the host. The Flatpak sandbox brings its own
runtime, Rust extension, and Node extension; the only thing the Deck supplies
is Flatpak itself, which ships with SteamOS.

### 1. Switch to Desktop Mode

From Game Mode: **STEAM → Power → Switch to Desktop**. Open **Konsole** from
the taskbar / app launcher.

If you haven't already, set a sudo password — needed for `ssh` and for one
sudoers entry the build process adds later (none today, but useful in
general):

```bash
passwd
```

### 2. Get the source on the Deck

Pick whichever fits your workflow.

**Option A — clone directly on the Deck.** Simplest when the Deck has
internet:

```bash
mkdir -p ~/saintos
cd ~/saintos
git clone https://github.com/OpenSAINT/SaintOS.git .
cd source/controller
```

If you have an SSH key on the Deck already wired to GitHub, the
`git@github.com:OpenSAINT/SaintOS.git` form works too.

**Option B — sync from a dev machine.** Useful when you're iterating from a
laptop and don't want to commit-push-pull every change. From your dev
machine:

```bash
controller/scripts/dev-steamdeck.sh
```

That script keeps `~/saintos/controller/` on the Deck in sync (rsync over
SSH, additive — never deletes Deck-local files) and prints the command to
trigger a rebuild on the Deck side. Pass `--deck-host steamdeck-2.local` if
the Deck's mDNS name differs, or `--robot pi@opensaint.local` to bounce
through the SAINT.OS server when the Deck is reachable from the robot but
not directly from your laptop. See the top of `dev-steamdeck.sh` for the
full flag list.

### 3. One-time prereqs — Flatpak runtime + SDK extensions

Run this once per Deck. It needs internet for the initial download (~1 GB):

```bash
~/saintos/source/controller/flatpak/build.sh --prereqs
```

That installs (from the Flathub remote, all `--user`):

| Component | What it is |
|---|---|
| `org.flatpak.Builder` | Sandboxed `flatpak-builder` — SteamOS's root is read-only so we use the Flatpak version, not a pacman install |
| `org.gnome.Sdk//50` + `org.gnome.Platform//50` | Runtime that ships `webkit2gtk-4.1` — the webview Tauri renders into |
| `org.freedesktop.Sdk.Extension.rust-stable//25.08` | Rust toolchain inside the build sandbox |
| `org.freedesktop.Sdk.Extension.node20//25.08` | Node 20 for the Angular build |

Already-installed refs are no-ops, so re-running `--prereqs` is safe.

### 4. Build and install the Flatpak

```bash
~/saintos/source/controller/flatpak/build.sh
```

The first build takes ~10–15 minutes: cargo fetches and compiles the full
Tauri dep graph, npm fetches the Angular tree, and `ng build` + `tauri build`
produce the final binary. Subsequent rebuilds reuse the persistent cache
under `controller/.flatpak-cache/persistent/` (cargo registry, `target/`,
`node_modules`, npm cache) — they finish in tens of seconds for small edits.

When it succeeds, the Flatpak is installed at the per-user prefix:

```
~/.local/share/flatpak/exports/bin/com.saintos.Controller
```

You can launch it immediately from a shell:

```bash
flatpak run com.saintos.Controller
```

If you ever need to wipe state and rebuild from scratch:

```bash
controller/flatpak/build.sh --clean   # full cargo + npm rebuild, keeps downloads
controller/flatpak/build.sh --nuke    # also wipes the source / runtime download cache
```

Use `--bundle` to additionally write a portable `SAINT-Controller.flatpak`
file you can copy to another Deck and `flatpak install --user` from there.

### 5. Add it to Steam as a Non-Steam Game

In Desktop Mode, open Steam → **Games → Add a Non-Steam Game to My
Library → BROWSE…** Browse to:

```
/home/deck/.local/share/flatpak/exports/bin/com.saintos.Controller
```

Tick the box, click **Add Selected Programs**. Steam will name the entry
"com.saintos.Controller" by default; rename it to **SAINT Controller** in
the library so the artwork-setup script in the next step finds it.

### 6. Set the library artwork (optional but recommended)

The Flatpak ships hero + capsule art in `/app/share/com.saintos.Controller/art/`,
plus a helper that finds your assigned Steam app-id in `shortcuts.vdf` and
copies the art into Steam's grid dir.

`build.sh` runs the helper automatically after install, but the first time
through it'll print a warning ("Add the controller as a Non-Steam Game,
then re-run…") because the shortcut doesn't exist yet. After step 5, run:

```bash
flatpak run --command=saint-controller-artwork-setup com.saintos.Controller
```

Then restart Steam (or `steam -shutdown && steam &`). The hero banner and
vertical capsule should appear on the library page.

The helper matches by name; pass `--name-pattern "Your Custom Name"` if
you renamed the shortcut to something other than "SAINT Controller".

### 7. Launch in Game Mode

Switch back to Game Mode (taskbar → **Return to Gaming Mode** or the
desktop shortcut). The controller appears under **Non-Steam → SAINT
Controller**. Press A to launch.

On first launch, point it at the SAINT.OS server (default
`ws://opensaint.local/api/ws`, password `12345`) and accept the prompt —
the connection setting persists under `~/.var/app/com.saintos.Controller/
config/saint-controller/`.

## Updating the controller

```bash
cd ~/saintos/source
git pull
controller/flatpak/build.sh
```

The persistent cache makes incremental rebuilds fast. The same Steam
shortcut keeps working — the Flatpak install path doesn't change.

## Developer mode (no Flatpak)

For tight inner-loop development on a laptop you don't need the Flatpak
overhead. Install the prereqs for your host OS, then `npm run tauri dev`.

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
sudo apt install build-essential curl wget pkg-config libssl-dev \
    libgtk-3-dev libwebkit2gtk-4.1-dev libayatana-appindicator3-dev librsvg2-dev \
    nodejs npm
curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh
source ~/.cargo/env

cd controller && npm install && npm run tauri dev
```

### Steam Deck developer mode

`npm run tauri dev` does work on the Deck if you really want it, but it
requires disabling read-only root and installing a Rust toolchain into the
system — which SteamOS wipes on every OS update. The Flatpak workflow
above is the supported path. The native-dev recipe is preserved in
`controller/scripts/setup-steamdeck.sh` for the rare case you need it.

## Build commands

| Command | Effect |
|---|---|
| `controller/flatpak/build.sh` | Incremental Flatpak build + install (Steam Deck path) |
| `controller/flatpak/build.sh --bundle` | Above, plus produce a portable `.flatpak` file |
| `controller/flatpak/build.sh --clean` | Wipe compile artifacts but keep downloads |
| `controller/flatpak/build.sh --nuke` | Wipe everything — last resort |
| `npm run tauri dev` | Native dev mode (hot reload, for macOS / Linux desktops) |
| `npm run tauri build` | Native production build (Tauri-native bundle for the host OS) |

## Documentation

- [docs/BINDINGS_SYSTEM.md](docs/BINDINGS_SYSTEM.md) — input → action data
  model (binding profiles, preset panels, action types)
- [docs/SHEETS_BINDINGS.md](docs/SHEETS_BINDINGS.md) — how to bind a
  controller input to a routing-sheet WebSocket input on the server (the
  end-to-end "make my joystick drive this servo" walkthrough)

## Troubleshooting

### `build.sh --prereqs` can't reach Flathub

The Deck has to have internet for the first build only. Plug into
Ethernet via the dock, or join WiFi from Desktop Mode (`STEAM` icon →
network indicator). Subsequent builds run offline thanks to the cache.

### "no space left on device" during build

The Flatpak cache lives in `controller/.flatpak-cache/` under your home
dir (~10 GB once populated). On a Deck with a 64 GB internal drive, keep
an eye on `df -h ~`. If you're tight on space:

```bash
controller/flatpak/build.sh --nuke   # frees ~10 GB
flatpak uninstall --unused           # removes orphan runtimes
```

Avoid `/var` — SteamOS gives it only 230 MB.

### Controller doesn't appear in Game Mode

Verify the shortcut exists (`grep SAINT ~/.local/share/Steam/userdata/*/config/shortcuts.vdf`
— the file is binary but the name will be readable). If it's missing,
re-do step 5 in Desktop Mode. Steam must be restarted after editing
shortcuts.

### Webview is blank or shows "connection refused"

The release binary embeds the production frontend; a blank window usually
means the Tauri CLI's `beforeBuildCommand` didn't run (you built with
`cargo build` instead of `tauri build`). Use `controller/flatpak/build.sh`
— it runs the correct command.

### `npm ci` fails inside the Flatpak build

`package.json` and `package-lock.json` are out of sync in the source
tree. The build falls back to `npm install` automatically so this run
goes through, but it'll prompt every build until you fix it. On the
host (not the Deck):

```bash
cd controller && npm install
git add package-lock.json && git commit
```
