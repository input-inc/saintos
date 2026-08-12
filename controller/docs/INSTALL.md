# SAINT.OS Controller — Install

Installing the controller on a Steam Deck. The operator never touches a
toolchain — the controller ships as a single self-contained `.AppImage`, and the
install is an atomic file-replace: write the file, `chmod +x`, add a `.desktop`
entry. No portal calls, no package manager.

You get the `.AppImage` two ways:

- **OTA from a SAINT.OS server** — the controller's **Settings** tab polls
  `/api/firmware/controller` and self-updates in place.
- **Bundled file** — the server ships
  `saint_firmware_controller_*.AppImage`; copy it to the Deck.

> To build the AppImage yourself, see [`BUILD.md`](BUILD.md).

## First-time install on a Steam Deck

### 1. Get the AppImage onto the Deck

From your dev machine (or wherever you have the artifact — e.g. the server's
`server/resources/firmware/controller/`):

```bash
ssh deck@steamdeck.local mkdir -p ~/Applications
scp saint_firmware_controller_*.AppImage \
    deck@steamdeck.local:~/Applications/SAINT-Controller.AppImage
```

`~/Applications/` is the conventional AppImage location on Linux and is visible
in Dolphin's home view, which makes manual swaps easy. SteamOS doesn't create it
by default — hence the `mkdir -p` above.

### 2. Mark it executable and try a launch

```bash
ssh deck@steamdeck.local
chmod +x ~/Applications/SAINT-Controller.AppImage
~/Applications/SAINT-Controller.AppImage
```

The controller window should come up. (GTK module warnings about
`canberra-gtk-module`, `colorreload-gtk-module`, and
`window-decorations-gtk-module` are non-fatal noise — those are KDE-side modules
GTK probes for and skips when missing.)

### 3. Add it to Steam as a Non-Steam Game

In Desktop Mode, **Steam → Games → Add a Non-Steam Game to My Library →
BROWSE…** and pick:

```
/home/deck/Applications/SAINT-Controller.AppImage
```

Rename the entry to **SAINT Controller** in the library so the artwork-setup
script in the next step finds it.

### 4. Set the Steam library artwork (optional)

`set-steamdeck-artwork.py` finds the Steam shortcut by name and writes the
bundled hero + capsule PNGs into Steam's grid dir.

```bash
~/Applications/SAINT-Controller.AppImage \
    --appimage-extract-and-run saint-controller-artwork-setup
```

Then restart Steam (`steam -shutdown && steam &`). The hero banner and vertical
capsule should appear on the library page. Pass `--name-pattern "<Your Custom
Name>"` if you used a different name.

### 5. Launch in Game Mode

Switch back to Game Mode. The controller appears under **Non-Steam → SAINT
Controller**. Press A.

On first launch, point it at the SAINT.OS server (default
`ws://opensaint.local/api/ws`, password `12345`) and accept the prompt. The
connection setting persists under `~/.config/saint-controller/`.

## Updating

### From the server's OTA flow (recommended)

The **Settings** tab polls `/api/firmware/controller` on the configured server.
When the server has a newer `saint_firmware_controller_*.AppImage` than what's
running, a banner appears: **Update available — Install**. The flow downloads,
verifies SHA-256, then atomically replaces the running AppImage at its current
location (typically `~/Applications/SAINT-Controller.AppImage`). The Steam
shortcut keeps working — the file path doesn't change.

Operator-visible UX: click Install, wait for "Update installed. Please manually
relaunch the SAINT Controller", relaunch from the Steam tile.

### Manual replacement

`scp` a newer AppImage over the existing one. `chmod +x` it. Relaunch.

## Troubleshooting

### AppImage launches but the webview is blank

Check `~/Applications/SAINT-Controller.AppImage` is the actual current build
(compare with the server's `info.json` checksum). Tauri's release binary embeds
the production frontend, so a blank window usually means the binary is stale
relative to what the controller's UI expects from the server.

### Controller doesn't appear in Game Mode

Verify the Steam shortcut exists
(`grep SAINT ~/.local/share/Steam/userdata/*/config/shortcuts.vdf` — the file is
binary but the name will be readable). If it's missing, re-do step 3 in Desktop
Mode. Steam must be restarted after editing shortcuts.
