# SAINT.OS Controller

A Tauri-based controller application for SAINT.OS that captures gamepad,
trigger, touch, gyro, and touchscreen input and forwards it to the SAINT.OS
server over WebSocket.

- **Frontend:** Vue 3 + TypeScript + Tailwind CSS
- **Backend:** Rust (Tauri 2.0)
- **Primary platform:** Steam Deck (SteamOS) — single-file AppImage, launched
  from Game Mode as a Non-Steam Game
- **Secondary platforms:** macOS / Linux / Windows (developer mode only)

## Overview

The controller ships as a single self-contained `.AppImage` — the operator
never touches a toolchain. They get the file from a SAINT.OS server (OTA, from
the Settings tab) or as the bundled artifact, and install it with an atomic
file-replace: write, `chmod +x`, add a `.desktop` entry.

## Install

Install on a Steam Deck via OTA from the server or the bundled AppImage — full
first-time setup (get the file on, add to Steam, artwork, Game Mode) and
updating are in **[`docs/INSTALL.md`](docs/INSTALL.md)**.

## Building from source

The AppImage builds in a linux/amd64 Docker container
(`controller/appimage/build-docker.sh`); a native dev loop is `npm run tauri
dev`. Full detail — local build, developer mode, build commands, cache tidy —
is in **[`docs/BUILD.md`](docs/BUILD.md)**. The repo-wide build index is
[`../docs/BUILD.md`](../docs/BUILD.md).

## Documentation

- [`docs/INSTALL.md`](docs/INSTALL.md) — install & update on a Steam Deck
- [`docs/BUILD.md`](docs/BUILD.md) — build the AppImage / native dev mode
- [`docs/BINDINGS_SYSTEM.md`](docs/BINDINGS_SYSTEM.md) — input → action data model (binding profiles, preset panels, action types)
- [`docs/SHEETS_BINDINGS.md`](docs/SHEETS_BINDINGS.md) — binding a controller input to a routing-sheet WebSocket input (the end-to-end "make my joystick drive this servo" walkthrough)
