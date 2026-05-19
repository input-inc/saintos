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

## Porting picklist (priority order)

Each entry lists the new Vue file and the legacy source(s) it should match.
Pick one slice at a time, port end-to-end (data fetch + render + interactions),
verify it in the dev server, commit. Repeat.

### Tier 1 — operator-critical
- **Node detail / Live tab** → `src/views/node/Live.vue` ← `js/nodelive.js`
- **Nodes list (adoption flow)** → `src/views/Nodes.vue` ← parts of `js/app.js`
- **Node detail / Peripherals** → `src/views/node/Peripherals.vue` ← `js/peripherals.js`
- **Node detail / Logs** → `src/views/node/Logs.vue` ← `js/nodelogs.js`

### Tier 2 — system configuration
- **Routes** → `src/views/Routes.vue` ← `js/routing.js`
- **Widgets / dashboard editor** → `src/views/Widgets.vue` ← `js/widgets.js`
- **Boards (per-node board picker)** → `src/views/node/Boards.vue` ← `js/boards.js`

### Tier 3 — inputs / behaviors
- **Inputs (LiveLink status)** → `src/views/Inputs.vue` ← `js/livelink.js` (top-level)
- **LiveLink detail** → `src/views/LiveLink.vue` ← `js/livelink.js`
- **Moods** → `src/views/Moods.vue` ← `js/moods.js`

### Tier 4 — chrome / dev tools
- **Control page (global control)** → `src/views/Control.vue` ← `js/controlpage.js` + `js/pincontrol.js`
- **Terminal** → `src/views/Terminal.vue` ← `js/terminal.js` (xterm.js)
- **Firmware updates** → `src/views/Updates.vue` ← `js/updates.js`
- **System logs** → `src/views/Logs.vue` ← parts of `js/app.js`
- **Settings** → `src/views/Settings.vue` ← `js/websocket.js` connection settings

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

- **Authentication UI**: legacy `js/websocket.js` had auth-required handling.
  The store carries the state; we don't have a login screen yet.
- **xterm.js for Terminal**: needs to be added to dependencies (`xterm`,
  `@xterm/addon-fit`) and wrapped in a component that handles binary frames
  via `ws.on('binary', ...)`.
- **Live reload on reconnect**: legacy code force-reloads the page after a
  reconnect to pick up new server code. Vue's HMR makes this less necessary
  during dev; we may still want it in production.
- **Routes endpoint shape**: `js/routing.js` is the most complex legacy
  file. Worth a closer look before porting — the canvas/wire-drawing logic
  may want to stay as a single component rather than getting decomposed.
