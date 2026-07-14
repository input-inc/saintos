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
}

const animationsRef = ref<LibraryItem[]>([]);
const posesRef = ref<LibraryItem[]>([]);
const soundsRef = ref<LibraryItem[]>([]);
// Whether a list response has been received since connect — lets the UI
// tell "still loading" apart from "loaded but genuinely empty".
const animationsLoadedRef = ref(false);
const posesLoadedRef = ref(false);
const soundsLoadedRef = ref(false);

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
        }))
        .filter(i => i.id);
}

async function refresh(): Promise<void> {
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
        }),
    );
    unlistenFns.push(
        await listen<{ poses?: unknown }>('library-poses', event => {
            posesRef.value = toItems(event.payload?.poses);
            posesLoadedRef.value = true;
        }),
    );
    unlistenFns.push(
        await listen<{ sounds?: unknown }>('library-sounds', event => {
            soundsRef.value = toItems(event.payload?.sounds);
            soundsLoadedRef.value = true;
        }),
    );

    // Fetch on connect, and re-fetch on every reconnect (a server
    // restart drops our view). Clear on disconnect so a stale list
    // doesn't linger in the panels.
    const conn = useConnection();
    watch(conn.isConnected, (connected) => {
        if (connected) {
            void refresh();
        } else {
            // Clear so a stale list doesn't linger, and reset loaded so
            // the UI shows "not connected" rather than "empty".
            animationsRef.value = [];
            posesRef.value = [];
            soundsRef.value = [];
            animationsLoadedRef.value = false;
            posesLoadedRef.value = false;
            soundsLoadedRef.value = false;
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
        refresh,
    };
}
