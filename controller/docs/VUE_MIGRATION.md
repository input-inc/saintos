# Migration plan: Angular → Vue 3

Drafted 2026-05-25 after the native-`<select>` scaling limitation surfaced — webkit2gtk-4.1 renders form-control popups as separate GTK windows that ignore the webview's zoom level, and Angular's ecosystem for Tailwind-themed unstyled components is significantly thinner than Vue's (no official `@headlessui/angular`, only React+Vue versions). Migrating away from Angular also drops bundle size and the Angular CLI's compile overhead — both small wins on Deck class hardware where every MB and second matters.

## What changes

| Layer | Today | After |
|---|---|---|
| Frontend framework | Angular 19 (standalone components, signals) | Vue 3 (composition API, `<script setup>`) |
| Build tool | Angular CLI (webpack) | Vite |
| State / DI | Services via Angular's injector | Composables (`useConnection()`, `useBindings()`, …) — same conceptual shape, no constructor injection |
| Routing | `@angular/router` | `vue-router@4` |
| Templates | `@if`, `@for`, `[(ngModel)]`, `(click)`, `[class]` | `v-if`, `v-for`, `v-model`, `@click`, `:class` — near-1:1 mapping |
| Reactivity | `signal(x)` / `computed(() => …)` | `ref(x)` / `computed(() => …)` — same names, same mental model |
| Unstyled dropdown / popover primitives | none, would need Angular CDK Overlay | `@headlessui/vue` (official Tailwind Labs) |
| Tauri IPC (`invoke`, `listen`) | unchanged | unchanged — `@tauri-apps/api/core` is framework-agnostic |
| Tailwind classes | unchanged | unchanged |
| Rust side (`src-tauri/`) | unchanged | unchanged |

## What stays

- All Rust code in `controller/src-tauri/` (commands, protocol, bindings mapper, mDNS discovery, AppImage install) — frontend-agnostic
- `controller/appimage/` build pipeline — Vite produces a `dist/` exactly like Angular CLI did; `tauri build --bundles appimage` consumes whatever's in `bundle.frontendDist`
- `tauri.conf.json` — only `build.beforeBuildCommand` and `build.frontendDist` need new values
- `controller/VERSION`, `sync-version.js`
- Tailwind config + the icon font (`material-icons` CSS)
- All `.png` icons + Steam library art

## Migration strategy

**Parallel-track build-out, single-cutover swap.** The current Angular code stays *untouched* until the Vue rewrite is feature-complete. Steps:

1. **Scaffold the Vue app at `controller/src-vue/`** alongside the existing `controller/src/`. Add Vite config that targets the new dir. Add `npm run dev:vue` / `npm run build:vue` scripts that operate on the new code path, leaving `npm run start` / `npm run build` (Angular) untouched. Result: `dist:vue/` builds and runs, Angular keeps shipping in dev and CI.
2. **Port one service / composable at a time, leaf-first.** Order: `connection.service` → `discovery` plumbing → `input.service` → `keyboard.service` → `drag-scroll.service` → `bindings.service`. Each becomes a `useXxx()` composable. Tauri `invoke` / `listen` calls move over verbatim — no signature changes.
3. **Port components, simple-to-complex.** Order: `App.vue` shell + routing → Settings tab → Controller tab → Preset Panel → Bindings tab (most complex; substantial forms). The shell-first ordering means we can run the app early and incrementally fill it out.
4. **Drop in `@headlessui/vue` `Listbox`** for the `<select>` replacements during the Settings + Bindings ports — solves the original problem.
5. **Cutover.** Once `src-vue/` is feature-parity tested on a Deck: delete `src/`, rename `src-vue/` → `src/`, swap `tauri.conf.json`'s `beforeBuildCommand` + `frontendDist` paths, remove Angular dependencies from `package.json`, remove `angular.json` and Angular-only configs. One commit, clear boundary.

## What we'd add to `package.json`

```
vue                       ^3.5
vue-router                ^4.4
@vitejs/plugin-vue        ^5.1
vite                      ^5.4
@headlessui/vue           ^1.7
```

Approximate added size (gzipped): Vue runtime ~33KB, Vue Router ~6KB, Headless UI ~12KB ≈ 50KB. After removing Angular at cutover, *net* bundle change: down ~250-300KB.

## Risk + scope

- **The bindings UI is the biggest single chunk** — `bindings.component.ts` has ~1000+ lines of template and substantial form state. Probably half the total migration effort lives there.
- **Forms behave subtly differently.** Angular reactive forms vs Vue's `v-model` aren't 1:1 in validation semantics. Most of the controller uses template-driven `[(ngModel)]` which maps cleanly to `v-model`, but anywhere I find a `FormGroup` / `FormControl`, expect a rewrite.
- **Signals → refs is a syntactic swap**, but watch for places where `computed()` is used recursively in Angular — Vue's `computed` works the same but the dependency graph is tracked differently in edge cases.

## What I'd flag for review before cutover

- Bindings tab parity: every input type, every action type, every panel ID
- Preset panel: the muscle-memory-critical Steam Deck overlay
- Settings OTA flow: download → verify → install → relaunch
- E-Stop button + banner + outgoing-traffic gate
- mDNS discovery list refresh + autoconnect
- Virtual keyboard show/hide (sandbox quirks here)

## Order of operations

1. Save this plan. Get sign-off on approach.
2. Scaffold `src-vue/` + Vite + dev/build scripts. Verify `npm run dev:vue` opens a "Hello SAINT" stub via Tauri.
3. Compositables: connection, discovery, input, keyboard, drag-scroll, bindings.
4. App shell + routing + Settings tab (medium complexity, exercises composables).
5. Controller tab + Preset Panel.
6. Bindings tab.
7. Test on Deck via AppImage build. Iterate until parity.
8. Cutover commit: delete Angular, rename `src-vue/` → `src/`, update Tauri config.
9. Final dist build + deploy.

Step 2 (scaffold) is the smallest commit-able unit; everything else stacks on top.
