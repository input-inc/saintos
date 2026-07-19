/**
 * Server library composable — the controller's view of the animations
 * and poses saved on the SAINT.OS server.
 *
 * The preset panels marked `source: 'animations'` / `'poses'` /
 * `'sounds'` render from these lists (names + icons), and selecting an
 * item fires start_animation / apply_pose / play_sound (see
 * useBindings.triggerActiveItem).
 *
 * Data path (over the existing WS):
 *   connect → invoke('list_animations') / list_poses / list_sounds
 *           → 'library-animations' / 'library-poses' / 'library-sounds'
 *             events → refs.
 *
 * Module-scoped singleton (same pattern as useConnection/useBatteries)
 * so one fetch feeds every caller.
 */

import { computed, ref, watch } from 'vue';
import { invoke } from '@tauri-apps/api/core';
import { listen, type UnlistenFn } from '@tauri-apps/api/event';
import { useConnection } from './useConnection';

export interface LibraryItem {
    id: string;
    name: string;
    icon?: string;
    // Animations only: whether the animation loops. Lets the panel apply
    // tap-again-to-stop to loopers and badge them. Absent for poses/sounds.
    loop?: boolean;
    // Sounds carry a group name; the panel offers a group filter dropdown.
    // Empty/absent means ungrouped.
    group?: string;
}

// One currently-playing animation, from the server's 'animation_state'
// broadcast. Keyed by animation id in the `playing` map.
export interface PlayingAnimation {
    id: string;
    name: string;
    loop: boolean;
    t: number;         // elapsed seconds
    duration: number;  // total seconds (0 if unknown)
}

// ── Persisted cache ──────────────────────────────────────────────────
// The panels must open instantly with whatever we last knew, even on a
// cold launch or before the WS connects. We persist each list to
// localStorage and hydrate the refs at module load, then sync in the
// background over the WS.
const STORAGE_PREFIX = 'saint.library.';

function loadCached(key: string): LibraryItem[] | null {
    try {
        const raw = localStorage.getItem(STORAGE_PREFIX + key);
        if (!raw) return null;
        const parsed = JSON.parse(raw);
        return Array.isArray(parsed) ? (parsed as LibraryItem[]) : null;
    } catch (e) {
        console.error(`[useLibrary] read cache '${key}' failed:`, e);
        return null;
    }
}

function saveCached(key: string, items: LibraryItem[]): void {
    try {
        localStorage.setItem(STORAGE_PREFIX + key, JSON.stringify(items));
    } catch (e) {
        console.error(`[useLibrary] persist '${key}' failed:`, e);
    }
}

const cachedAnimations = loadCached('animations');
const cachedPoses = loadCached('poses');
const cachedSounds = loadCached('sounds');

const animationsRef = ref<LibraryItem[]>(cachedAnimations ?? []);
const posesRef = ref<LibraryItem[]>(cachedPoses ?? []);
const soundsRef = ref<LibraryItem[]>(cachedSounds ?? []);
// Whether we have a definitive list to show (cached or freshly fetched) —
// lets the UI tell "still loading" apart from "loaded but genuinely
// empty". A present cache counts as loaded so the panel renders items
// immediately instead of a spinner.
const animationsLoadedRef = ref(cachedAnimations !== null);
const posesLoadedRef = ref(cachedPoses !== null);
const soundsLoadedRef = ref(cachedSounds !== null);

// ── Animation playback state ─────────────────────────────────────────
// Which animations are currently playing, keyed by id, from the server's
// 'animation_state' broadcast (~1 Hz). Drives the "playing" badge and
// tap-again-to-stop. Optimistically updated on tap for instant feedback,
// then reconciled by the next broadcast. Not persisted — it's live state.
const playingRef = ref<Record<string, PlayingAnimation>>({});

function noteStarted(id: string, loop: boolean): void {
    playingRef.value = {
        ...playingRef.value,
        [id]: { id, name: id, loop, t: 0, duration: 0 },
    };
}

function noteStopped(id: string): void {
    if (playingRef.value[id]) {
        const next = { ...playingRef.value };
        delete next[id];
        playingRef.value = next;
    }
}

// ── Background-sync indicator ────────────────────────────────────────
// True while a refresh is in flight (drives the header spinner). A
// refresh isn't done when invoke() resolves — the lists arrive later via
// events — so we track outstanding responses and clear on the last one,
// with a timeout safety net so a dropped response can't wedge the
// spinner on forever.
const syncingRef = ref(false);
let pendingResponses = 0;
let syncTimeout: ReturnType<typeof setTimeout> | null = null;
const SYNC_TIMEOUT_MS = 8000;

function beginSync(): void {
    pendingResponses = 3; // animations + poses + sounds
    syncingRef.value = true;
    if (syncTimeout) clearTimeout(syncTimeout);
    syncTimeout = setTimeout(() => {
        pendingResponses = 0;
        syncingRef.value = false;
        syncTimeout = null;
    }, SYNC_TIMEOUT_MS);
}

function markResponse(): void {
    if (pendingResponses > 0) {
        pendingResponses--;
        if (pendingResponses === 0) {
            syncingRef.value = false;
            if (syncTimeout) {
                clearTimeout(syncTimeout);
                syncTimeout = null;
            }
        }
    }
}

// Module-scoped connection handle (singleton) so refresh() can skip work
// when offline instead of spinning against a dead socket.
const conn = useConnection();

let initialized = false;
const unlistenFns: UnlistenFn[] = [];

// The server summaries carry more than we render (group, duration, …);
// keep just what the panel needs and tolerate missing fields.
function toItems(raw: unknown): LibraryItem[] {
    if (!Array.isArray(raw)) return [];
    return raw
        .filter((x): x is Record<string, unknown> => !!x && typeof x === 'object')
        .map(x => ({
            id: String(x['id'] ?? ''),
            name: String(x['name'] ?? x['id'] ?? ''),
            icon: typeof x['icon'] === 'string' && x['icon'] ? x['icon'] : undefined,
            loop: typeof x['loop'] === 'boolean' ? x['loop'] : undefined,
            group: typeof x['group'] === 'string' && x['group'] ? x['group'] : undefined,
        }))
        .filter(i => i.id);
}

async function refresh(): Promise<void> {
    // Nothing to sync while offline — keep showing the cached lists rather
    // than kicking off invokes that will fail and leave the spinner stuck.
    if (!conn.isConnected.value) return;

    beginSync();
    await Promise.all([
        invoke('list_animations').catch(e =>
            console.error('[useLibrary] list_animations failed:', e)),
        invoke('list_poses').catch(e =>
            console.error('[useLibrary] list_poses failed:', e)),
        invoke('list_sounds').catch(e =>
            console.error('[useLibrary] list_sounds failed:', e)),
    ]);
}

async function ensureInit(): Promise<void> {
    if (initialized) return;
    initialized = true;

    unlistenFns.push(
        await listen<{ animations?: unknown }>('library-animations', event => {
            animationsRef.value = toItems(event.payload?.animations);
            animationsLoadedRef.value = true;
            saveCached('animations', animationsRef.value);
            markResponse();
        }),
    );
    unlistenFns.push(
        await listen<{ poses?: unknown }>('library-poses', event => {
            posesRef.value = toItems(event.payload?.poses);
            posesLoadedRef.value = true;
            saveCached('poses', posesRef.value);
            markResponse();
        }),
    );
    unlistenFns.push(
        await listen<{ sounds?: unknown }>('library-sounds', event => {
            soundsRef.value = toItems(event.payload?.sounds);
            soundsLoadedRef.value = true;
            saveCached('sounds', soundsRef.value);
            markResponse();
        }),
    );

    // Live animation playback state. Payload: { players: [{id, name,
    // duration, t, running, loop, …}] }. The server only reports active
    // players, so the map is simply "everything currently playing".
    unlistenFns.push(
        await listen<{ players?: unknown }>('animation-state', event => {
            const players = Array.isArray(event.payload?.players)
                ? event.payload!.players : [];
            const next: Record<string, PlayingAnimation> = {};
            for (const p of players) {
                if (!p || typeof p !== 'object') continue;
                const rec = p as Record<string, unknown>;
                const id = String(rec['id'] ?? '');
                if (!id) continue;
                next[id] = {
                    id,
                    name: String(rec['name'] ?? id),
                    loop: rec['loop'] === true,
                    t: typeof rec['t'] === 'number' ? rec['t'] : 0,
                    duration: typeof rec['duration'] === 'number' ? rec['duration'] : 0,
                };
            }
            playingRef.value = next;
        }),
    );

    // Fetch on connect, and re-fetch on every reconnect (a server restart
    // drops our view). We deliberately do NOT clear the lists on
    // disconnect: the persisted cache stays on screen so a panel opened
    // during a reconnect blip still renders instantly, and the background
    // sync (with the header spinner) refreshes it once we're back.
    watch(conn.isConnected, (connected) => {
        if (connected) {
            void refresh();
        } else {
            // No server → no authoritative playback state. Clear so a
            // stale "playing" badge doesn't linger across a disconnect.
            playingRef.value = {};
        }
    }, { immediate: true });
}

export function useLibrary() {
    void ensureInit();
    return {
        animations: computed(() => animationsRef.value),
        poses: computed(() => posesRef.value),
        sounds: computed(() => soundsRef.value),
        animationsLoaded: computed(() => animationsLoadedRef.value),
        posesLoaded: computed(() => posesLoadedRef.value),
        soundsLoaded: computed(() => soundsLoadedRef.value),
        // True while a background refresh is in flight — drives the header
        // sync spinner.
        syncing: computed(() => syncingRef.value),
        // Currently-playing animations, keyed by id (server-authoritative,
        // optimistically nudged on tap).
        playing: computed(() => playingRef.value),
        // Optimistic updates so the badge flips instantly on tap; the next
        // broadcast reconciles.
        noteStarted,
        noteStopped,
        refresh,
    };
}
