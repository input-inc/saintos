/**
 * Per-preset-panel display preferences — icon layout (horizontal vs
 * vertical) and column count (2–4). Set from the panel's display-options
 * modal, consumed by PresetPanel (rendering + row measurement) and
 * useBindings (navigation math). Persisted to localStorage, keyed by
 * panel id, so each board keeps its own layout across launches.
 *
 * Module-scoped singleton (same pattern as useConnection/useLibrary).
 */

import { computed, ref } from 'vue';

export type IconLayout = 'horizontal' | 'vertical';
/** Which edge the panel's source list sits against. */
export type SourceListSide = 'left' | 'right';

export interface PanelDisplayPrefs {
    layout: IconLayout;
    columns: number; // 2, 3, or 4
    // Side for the source list (the playlists of the panel's kind).
    // Left by default because that is where a list of sources is
    // conventionally read from; right exists because on a Deck held in
    // two hands, which thumb reaches the list matters more than
    // convention. The list hides itself when the panel's kind has no
    // playlists, so this only takes effect when there is one to place.
    sourceListSide: SourceListSide;
}

const DEFAULTS: PanelDisplayPrefs = {
    layout: 'horizontal', columns: 4, sourceListSide: 'left',
};
const STORAGE_KEY = 'saint.panelDisplayPrefs';

function load(): Record<string, PanelDisplayPrefs> {
    try {
        const raw = localStorage.getItem(STORAGE_KEY);
        if (!raw) return {};
        const parsed = JSON.parse(raw);
        return (parsed && typeof parsed === 'object') ? parsed : {};
    } catch (e) {
        console.error('[useDisplayPrefs] read failed:', e);
        return {};
    }
}

const prefsRef = ref<Record<string, PanelDisplayPrefs>>(load());

function save(): void {
    try {
        localStorage.setItem(STORAGE_KEY, JSON.stringify(prefsRef.value));
    } catch (e) {
        console.error('[useDisplayPrefs] persist failed:', e);
    }
}

function prefsFor(panelId: string): PanelDisplayPrefs {
    return { ...DEFAULTS, ...(prefsRef.value[panelId] ?? {}) };
}

function setPrefs(panelId: string, patch: Partial<PanelDisplayPrefs>): void {
    if (!panelId) return;
    const next = { ...prefsFor(panelId), ...patch };
    // Clamp columns to the supported 2–4 range.
    next.columns = Math.max(2, Math.min(4, Math.round(next.columns)));
    // Anything but 'right' means left -- a stored value from a future or
    // corrupted build must not leave the list unrendered.
    next.sourceListSide = next.sourceListSide === 'right' ? 'right' : 'left';
    prefsRef.value = { ...prefsRef.value, [panelId]: next };
    save();
}

export function useDisplayPrefs() {
    return {
        prefsFor,
        setPrefs,
        // Reactive map so callers can build computed(() => prefsFor(id)).
        all: computed(() => prefsRef.value),
    };
}
