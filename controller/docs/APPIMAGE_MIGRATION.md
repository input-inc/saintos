# Migration plan: flatpak → AppImage

Drafted 2026-05-24 after exhausting the flatpak-in-Docker-on-Mac path. The flatpak build was solving the right problem (ship a Tauri app to read-only SteamOS without depending on the host's webkit2gtk version) but with the wrong tool given our build environment — flatpak-builder needs bwrap, bwrap needs seccomp, and bwrap's seccomp setup is incompatible with Rosetta's syscall translation on Apple Silicon Docker Desktop. AppImage gets us the same "ship everything we need bundled" guarantee with none of the fight.

## Why this is safe

- SteamOS Holo ships `gtk3 3.24.43-4` and `libsoup3 3.6.1-1` already — only `webkit2gtk-4.1` is missing. AppImage will bundle that plus its transitive deps.
- SteamOS glibc is 2.41; Ubuntu 24.04 build host is 2.39. Forward-compatible direction, so the existing base image works.
- AppImage has no sandbox, runs as the user. Everything the flatpak's `finish-args` were granting (network, X11, Wayland, audio, devices, xdg paths) is implicit.

## New build pipeline

Replace `controller/flatpak/` with `controller/appimage/`.

### `controller/appimage/Dockerfile`

```
FROM ubuntu:24.04
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends \
        rustc cargo nodejs npm \
        libwebkit2gtk-4.1-dev libsoup-3.0-dev libgtk-3-dev \
        libappindicator3-dev libssl-dev pkg-config build-essential \
        file fuse libfuse2 ca-certificates curl git \
    && rm -rf /var/lib/apt/lists/*
WORKDIR /work
CMD ["/work/controller/appimage/build-bundle.sh"]
```

No flatpak, no flatpak-builder, no ostree, no GNOME runtime download.

### `controller/appimage/build-bundle.sh`

Env-parameterized like the current flatpak version. Body:

```
cd /work/controller
npm ci --prefer-offline --no-audit --no-fund
npx tauri build --bundles appimage
# Stage the produced
# controller/src-tauri/target/release/bundle/appimage/SAINT-Controller_<v>_amd64.AppImage
# into server/resources/firmware/controller/saint_firmware_controller_<v>-local.<sha>.AppImage
# and regenerate info.json (same shape as today).
```

### `controller/appimage/build-docker.sh`

Mac driver. `docker run --platform=linux/amd64 --rm` with bind-mounts for `/work` (repo) and `~/.cache/saint-os/controller-appimage/` (cargo registry, target/, node_modules, npm-cache).

**Drop everything we added for the flatpak path:**

- `--privileged`
- `--security-opt systempaths=unconfined`
- `--security-opt seccomp=unconfined`
- `--security-opt apparmor=unconfined`
- The `/var/lib/flatpak` bind mount
- The `saint-controller-flatpak-build` named volume
- The compact_memory shim
- The manifest path generation (no manifest)

Plain bind mounts are fine here — no splice() between FUSE files, because `cargo build` writes to a single mount tree.

## Tauri config

`controller/src-tauri/tauri.conf.json` — set `bundle.targets: ["appimage"]`. Verify `bundle.icon` includes a square PNG ≥256px for the AppImage launcher.

If we want Steam-library artwork bundled inside the AppImage, add to `bundle.resources`:

```
controller/images/libraryCapsule.png
controller/images/libraryHero.png
```

They'll land at `usr/share/com.saintos.Controller/art/` inside the squashfs.

## Install code rewrite

`controller/src-tauri/src/commands.rs` — replace `install_controller_update`:

```rust
#[tauri::command]
async fn install_controller_update(bytes: Vec<u8>) -> Result<(), String> {
    let install_dir = dirs::data_local_dir().ok_or("no XDG_DATA_HOME")?
        .join("saint-controller");
    fs::create_dir_all(&install_dir).map_err(|e| e.to_string())?;

    let target = install_dir.join("SAINT-Controller.AppImage");
    let staging = install_dir.join("SAINT-Controller.AppImage.new");

    fs::write(&staging, &bytes).map_err(|e| e.to_string())?;
    fs::set_permissions(&staging, Permissions::from_mode(0o755))
        .map_err(|e| e.to_string())?;
    // Atomic rename. Safe over a busy executable on Linux —
    // the running process keeps the old inode; next launch
    // execs the new file.
    fs::rename(&staging, &target).map_err(|e| e.to_string())?;

    write_desktop_file(&install_dir, &target)?;
    Ok(())
}
```

`write_desktop_file` writes `~/.local/share/applications/saint-controller.desktop` with `Exec=<absolute AppImage path>`, then runs `update-desktop-database` if available. ~30 lines total.

**Drop:** the `flatpak-spawn --host flatpak install --user --reinstall` subprocess and the `--talk-name=org.freedesktop.Flatpak` permission grant. Both become moot.

## Server-side changes

- `server/saint_server/webserver/http_server.py:24` — `.flatpak` extension becomes `.AppImage` in the fallback firmware scanner. `FIRMWARE_TYPES` keeps `'controller'`.
- `server/resources/firmware/controller/info.json` — placeholder gets the new filename pattern. No schema change.

## Settings UI

`controller/src/app/features/settings/settings.component.ts` — only changes if the file extension shows in user-facing strings. Fetch/SHA-256-verify/invoke flow is identical.

## CI

`.github/workflows/dist.yml` — rename `flatpak-controller` job to `appimage-controller`. New steps:

```
- apt install -y rustc cargo nodejs npm libwebkit2gtk-4.1-dev ...
- (cd controller && npm ci --prefer-offline)
- (cd controller && npx tauri build --bundles appimage)
- upload artifact: SAINT-Controller_*.AppImage
```

Drop: flathub remote registration, GNOME runtime install, flatpak-builder cache, the whole bwrap configuration. Resulting job runs in ~10-15 min instead of ~30+.

`scripts/build-local-dist.sh` — rename `build_controller_flatpak()` → `build_controller_appimage()`. `--fetch-firmware` pulls the `appimage-controller` artifact. `--skip-controller-build` flag stays.

## Cleanup

After AppImage is verified working end-to-end on a real Deck:

**Delete:**
- `controller/flatpak/com.saintos.Controller.yml`
- `controller/flatpak/Dockerfile`
- `controller/flatpak/build.sh` (on-Deck path)
- `controller/flatpak/build-bundle.sh`
- `controller/flatpak/build-docker.sh`
- `controller/flatpak/RESUME.md`
- `controller/flatpak/.generated.com.saintos.Controller.yml` (gitignored leftover)
- `controller/flatpak/com.saintos.Controller.metainfo.xml` (only consumed by flatpak's appstream)
- Docker named volume `saint-controller-flatpak-build`
- `~/.cache/saint-os/controller-flatpak/` host cache dir

**Keep but move/repurpose:**
- `controller/flatpak/com.saintos.Controller.desktop` — Tauri generates one inside the AppImage; drop unless we want a stable host-side template.

**Update:**
- `.gitignore` — drop the `.generated.com.saintos.Controller.yml` entry, add anything needed for AppImage build output.

## Order of operations

1. **Build pipeline** (Dockerfile + 2 scripts + tauri.conf.json change) — no deletes yet, no server changes. Produces an `.AppImage` in `controller/src-tauri/target/release/bundle/appimage/`.
2. **Manually verify on Deck**: scp the file, `chmod +x`, launch. Confirms webkit2gtk + GTK + libsoup3 + the AppImage's bundle all work on stock SteamOS. **This is the gate.** If it fails to launch, we debug bundling before going further.
3. **Server side**: update `http_server.py` extension + `info.json` placeholder.
4. **Rust install command**: rewrite `install_controller_update`. Build a new controller version locally (still through the old flatpak install path on the deck, since we haven't deleted it yet) so the new binary has the new install command.
5. **Stage the new AppImage** through `build-bundle.sh` → drops into `server/resources/firmware/controller/`.
6. **Test OTA end-to-end**: install the previous version on a Deck, run server, trigger update from Settings, verify the new AppImage installs and the next launch picks it up.
7. **CI** workflow update + `build-local-dist.sh` rename.
8. **Cleanup**: delete flatpak files, cache dir, Docker volume.
9. **Update or delete `RESUME.md`** — the work it tracked is done.

## What this buys us

- **Build:** ~10-15 min Docker run. No bwrap, no GNOME runtime, no Rosetta-vs-seccomp.
- **Code:** net deletion. flatpak manifest gone (~280 lines YAML), flatpak install glue in Rust gone (~50 lines), CI flatpak setup gone (~80 lines).
- **Artifact:** one file, ~50-80 MB.
- **Install:** file write + chmod + atomic rename + desktop file. No portals, no subprocesses.
- **Failure modes:** regular Linux library loading. If something breaks, `ldd` tells you why.

## Theoretical risks worth knowing

- *webkit2gtk dlopen'd deps not auto-discovered.* `linuxdeploy` walks `ldd` for static link-time deps; some plugins are dlopened at runtime (GIO modules, gst plugins, ICU data). Tauri's AppImage pipeline handles most of these; if something's missing the symptom is a clear error on first launch, fix is adding a path to linuxdeploy's library list.
- *GTK theming on KDE.* `linuxdeploy-plugin-gtk` bundles a minimal theme stack; the app will look like Adwaita on KDE Plasma rather than themed to match. Cosmetic, not functional.
- *GPU/Mesa.* AppImage explicitly does NOT bundle `libGL`/`libEGL` — those need to match the host's kernel driver. linuxdeploy excludes them by default. The Deck has Mesa.
