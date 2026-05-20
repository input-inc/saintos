# Frontend migration to Vue 3 + Vite + Tailwind

Branch: `frontend-vue`.

## Current state

Scaffolding is in place. One real slice is ported (State tab); every other
route renders a `<PendingPort>` placeholder that links back to the legacy
vanilla UI at `/legacy.html`.

```
web/
├── package.json, vite.config.js, tailwind.config.js, postcss.config.js
├── index.html              ← Vite entry, mounts <App>
├── legacy.html             ← old vanilla UI, kept reachable during migration
├── src/
│   ├── main.js, App.vue, router.js, style.css
│   ├── stores/             ← ws, nodes, peripheralCatalog
│   ├── composables/        ← useWsTopic, useThrottledSend
│   ├── components/         ← NavSidebar, ConnectionBadge, PeripheralCard,
│   │                          channel/{ChannelSlider,Toggle,Color,Spec}
│   └── views/              ← Dashboard, Nodes, NodeDetail (+ node/*), and
│                              one stub per remaining top-level route
├── js/, css/               ← legacy vanilla assets, untouched
└── dist/                   ← Vite build output (gitignored)
```

## Dev loop

```sh
cd web/
npm install
SAINT_HOST=http://<pi-ip>:8080 npm run dev   # http://localhost:5173, /api proxied to the Pi
```

Vite's dev server proxies `/api/*` and `/api/ws` to the Pi, so the running
Pi keeps serving the backend while you iterate on UI on a dev machine.

## Cutover

1. `npm run build` — outputs to `web/dist/`.
2. Update the call site in `saint_server/server_node.py` (or wherever the
   web server is constructed) to pass `web_root=<repo>/web/dist` instead of
   `<repo>/web`.
3. Delete `web/legacy.html` and `web/js/`, `web/css/` once the new UI is
   fully fledged.

## Building a release tarball

The dist tarball is the install unit for the Pi. The Pi never sees `npm`
or `node` — only the static files in `web/dist/`. The npm build runs once
on the dev machine (or CI), output gets baked into the colcon install
tree via `setup.py`, then `make-dist.sh` rolls it up.

### Locally

```sh
# From the parent dir (source/), one shot:
scripts/build-local-dist.sh
# → dist/saint-os_<version>_arm64_jazzy.tar.gz

# Faster iteration when only Python/firmware changed (reuses web/dist/):
scripts/build-local-dist.sh --skip-web-build
```

The script's `--- web frontend (Vite build) ---` section runs
`npm ci && npm run build` in `saint_os/web/` on the host before the
arm64 Debian container builds saint_os.

### In CI

`.github/workflows/dist.yml` has the matching steps right after
checkout: `actions/setup-node@v4` (with npm cache keyed on
`saint_os/web/package-lock.json`) followed by `npm ci && npm run build`.
The lockfile must be committed — without it neither `npm ci` nor the
cache key works.

### Where it ends up on the Pi

`setup.py:get_data_files()` walks `web/dist/` recursively and adds every
file to `data_files`. After colcon install + tarball extraction on the
target, the assets land at:

```
/opt/saint-os/share/saint_os/web/
├── index.html, legacy.html, …      ← top-level html (incl. legacy during migration)
├── js/                              ← legacy vanilla js (during migration)
└── dist/
    ├── index.html                   ← Vue entry — point web_root here at cutover
    └── assets/<hashed>.{js,css}     ← Vite bundle output
```

### Producing a lockfile

The first `npm install` produces `package-lock.json`. Commit it — every
build after that uses `npm ci` (deterministic, fails on drift) instead
of `npm install`.

## Porting status

**All legacy functionality is now ported.** Below is what each Vue view
does. Where a view is intentionally simpler than the legacy it's
called out explicitly.

| Vue view | Legacy source | Notes |
| --- | --- | --- |
| `views/Dashboard.vue` | `js/app.js` (dashboard panel) | System status (CPU/mem/temp/uptime from `system_status` broadcast), node summary, recent activity log. |
| `views/Nodes.vue` | `js/app.js` (nodes list) | Adopted + unadopted sections; adoption modal with chip/board/role pickers; scan button. |
| `views/NodeDetail.vue` + `views/node/*.vue` | `js/app.js` + per-tab files | Six tabs with `NodeActions` sidebar (identify, restart, e-stop, firmware update, factory reset, unadopt, remove). |
| `views/node/Overview.vue` | inline in legacy.html | Identity + connection stat grids. |
| `views/node/Peripherals.vue` | `js/peripherals.js` | Add/edit/remove with shared `AppModal`, pin pickers from node capabilities, params per type. |
| `views/node/Live.vue` | `js/nodelive.js` | Per-peripheral channel readings. |
| `views/node/State.vue` | `js/statecontrols.js` | Channel-addressed sliders/toggles/color via `set_channel_value`. |
| `views/node/Logs.vue` | `js/nodelogs.js` | History + live tail, scroll-to-bottom, clear. |
| `views/node/Boards.vue` | `js/boards.js` | Lists registered boards and highlights the node's current one. The editor lives on Settings. |
| `views/Routes.vue` | `js/routing.js` | SVG graph editor with draggable peripheral/signal/widget nodes and bezier wires. Click a channel handle to start a wire, click another compatible handle to complete it. Position persisted to localStorage. Buttons to add signals/widgets and auto-layout. Route list below the canvas for raw view + delete. |
| `views/Widgets.vue` | `js/widgets.js` | Read-only widget cards with input → route summary. |
| `views/Inputs.vue` | `js/livelink.js` (summary) | LiveLink summary card. |
| `views/LiveLink.vue` | `js/livelink.js` | Detail with paused-able blend-shape bars. |
| `views/Control.vue` | `js/controlpage.js` + `js/pincontrol.js` | Cross-node jump-off (links to each node's State tab) + global E-Stop. |
| `views/Moods.vue` | `js/moods.js` | List moods, parse to show props, apply to head node. |
| `views/Updates.vue` | `js/updates.js` | Check / download / install / scan-USB. Installing overlay matches legacy. |
| `views/Logs.vue` | `js/app.js` (activity log) | System log buffer with text filter + per-level (info/warn/error/debug) checkboxes. |
| `views/Terminal.vue` | `js/terminal.js` | xterm.js + FitAddon, binary frame forwarding, resize/restart. |
| `views/Settings.vue` | `js/app.js` Settings + `js/boards.js` + `js/websocket.js` | Five sub-tabs: Server (this-browser connection + identity + WebSocket/Network/ROS + connected clients list with disconnect), Interface (temperature unit + theme), LiveLink (receiver settings + live status), Firmware (bundled builds list), Boards (full CRUD via `BoardEditorModal`). Dirty tracking + save/reset. |
| `components/LoginScreen.vue` | `legacy.html` login section | Shown when `ws.authRequired && !ws.authenticated`. Saves host/password to localStorage, calls `ws.authenticate()`. |

## Shared components, composables, stores

- `components/AppHeader.vue` — sticky header with gradient logo + connection pill + E-Stop
- `components/NavBar.vue` — horizontal `.nav-link` strip
- `components/AppModal.vue` — generic modal w/ slot for actions, ESC-to-close
- `components/LoginScreen.vue` — full-page auth gate when WS requires it
- `components/PeripheralCard.vue` + `components/channel/{ChannelSlider,Toggle,Color,Spec.js}.vue`
- `components/AdoptModal.vue` — chip/board/role pickers for unadopted nodes
- `components/NodeActions.vue` — sidebar on node detail with all the operator actions
- `components/FirmwareUpdateModal.vue` — per-node firmware update + force-build override
- `components/BoardEditorModal.vue` — YAML editor for operator-authored boards
- `composables/useWsTopic.js` — subscribe-on-mount, unsub-on-unmount, re-sub on reconnect
- `composables/useThrottledSend.js` — slider commit throttling
- `stores/ws.js` — full WS port, reactive connection state, send/management/control/router/subscribe/authenticate
- `stores/nodes.js` — adopted + unadopted nodes registry; scan(), fetchAll()
- `stores/peripheralCatalog.js` — cached peripheral type catalog
- `stores/settings.js` — server settings with dirty tracking + save/reset
- `stores/display.js` — operator display preferences (temperature unit) in localStorage
- `stores/activity.js` — capped activity log buffer fed by `activity` topic + manual `add()`

## Conventions to follow

- **`<script setup>` Composition API**, Vue 3 idiomatic.
- **Pinia stores** for anything that crosses component boundaries. Each store
  owns its fetching and reacts to WS broadcasts directly.
- **`useWsTopic(() => 'topic/key')`** for live subscriptions — never call
  `ws.subscribe` from a component directly. The composable handles cleanup.
- **Tailwind classes inline**, no `<style scoped>` unless absolutely needed.
  Shared utility classes (`.card`, `.btn`, `.stat-label`) live in `src/style.css`.
- **No globals.** No `window.saintWS`. Everything goes through `useWsStore()`.
- **No JSX, no TypeScript** for now. Keep the entry barrier low.
- **Tests**: none yet. Vitest is the natural choice when we add them — defer.

## Known design questions still open

None — see resolved list below.

Resolved:

- **Authentication UI** — `components/LoginScreen.vue` gates the app via
  `ws.authRequired && !ws.authenticated` in `App.vue`.
- **xterm.js for Terminal** — `@xterm/xterm` + `@xterm/addon-fit` are in
  `package.json`; `views/Terminal.vue` wires binary frames through
  `ws.on('binary', …)`, with `ResizeObserver`-driven `FitAddon`.
- **Routes endpoint shape** — `views/Routes.vue` ports `js/routing.js` as
  a single component; the SVG canvas, wire-drawing, and route list all
  live there.
- **Live reload on reconnect** — `stores/ws.js` tracks a `hadSession` flag;
  the *second* `ready` (i.e. after a reconnect) triggers `location.reload()`.
  The hash route survives the reload, so no explicit state save is needed.
