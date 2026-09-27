/**
 * useBindings owns the binding-profile model and the panel-navigation
 * state machine (the UI-side mirror of the Rust mapper's panel nav).
 * We test the exported pure target helpers and the grid navigation /
 * show-hide-toggle logic. The Tauri bridge is mocked so importing the
 * module (which eagerly spins up useLibrary/useConnection) doesn't try
 * to talk to a backend.
 */
import { describe, it, expect } from 'vitest';
import { vi } from 'vitest';

vi.mock('@tauri-apps/api/core', () => ({ invoke: vi.fn(() => Promise.resolve(undefined)) }));
vi.mock('@tauri-apps/api/event', () => ({ listen: vi.fn(() => Promise.resolve(() => {})) }));
vi.mock('@tauri-apps/plugin-log', () => ({
    trace: vi.fn(), debug: vi.fn(), info: vi.fn(), warn: vi.fn(), error: vi.fn(),
}));

// The Sounds panel is server-backed (source: 'sounds'), so its grid is
// empty until the server library streams in. Mock useLibrary so the
// panel has 6 items — enough to exercise the grid-navigation math the
// way the old static presets did.
// Two sound playlists so the source-list tests have something to pick
// between, and one animations playlist to prove the kinds stay apart.
// `loud` deliberately lists its members in a different order than the
// library and names a member the library doesn't have.
// vi.hoisted: vi.mock factories are lifted above module-level consts,
// so a plain `const PLAYLISTS` here would be in the TDZ when the factory
// runs.
const PLAYLISTS = vi.hoisted(() => ([
    { id: 'pl_quiet', name: 'Quiet', kind: 'sounds', items: ['s1', 's3'] },
    { id: 'pl_loud', name: 'Loud', kind: 'sounds', items: ['s4', 's0', 'ghost'] },
    { id: 'pl_anim', name: 'Greetings', kind: 'animations', items: ['a1'] },
]));

vi.mock('../useLibrary', () => {
    const items = Array.from({ length: 6 }, (_, i) => ({ id: `s${i}`, name: `S${i}` }));
    const box = (v: unknown) => ({ value: v });
    return {
        useLibrary: () => ({
            animations: box([]), poses: box([]), sounds: box(items),
            playlists: box(PLAYLISTS),
            animationsLoaded: box(true), posesLoaded: box(true), soundsLoaded: box(true),
            refresh: () => Promise.resolve(),
        }),
    };
});

import {
    useBindings,
    targetDisplayName,
    isWsInputTarget,
    type ControlTarget,
} from '../useBindings';

describe('useBindings target helpers', () => {
    it('discriminates WS-input vs topic targets', () => {
        const ws: ControlTarget = { sheet_id: 's1', input_id: 'in1' };
        const topic: ControlTarget = { topic: '/tracks', channel: 'left' };
        expect(isWsInputTarget(ws)).toBe(true);
        expect(isWsInputTarget(topic)).toBe(false);
    });

    it('builds a display name (explicit name wins, else a derived label)', () => {
        expect(targetDisplayName({ sheet_id: 's1', input_id: 'in1' })).toBe('s1/in1');
        expect(targetDisplayName({ topic: '/tracks', channel: 'left' })).toBe('/tracks:left');
        expect(targetDisplayName({ topic: '/tracks', channel: 'left', name: 'Left Track' }))
            .toBe('Left Track');
    });
});

describe('useBindings panel navigation', () => {
    it('navigates the default "sounds" grid within bounds', () => {
        const b = useBindings();
        b.showPanel('sounds'); // 6 items (mocked library), 4 columns, 8 per page
        expect(b.activePanelState.value.activePanelId).toBe('sounds');
        expect(b.activePanelState.value.selectedIndex).toBe(0);

        b.navigatePanel('right');
        expect(b.activePanelState.value.selectedIndex).toBe(1);
        b.navigatePanel('left');
        expect(b.activePanelState.value.selectedIndex).toBe(0);
        b.navigatePanel('down'); // +columns
        expect(b.activePanelState.value.selectedIndex).toBe(4);
        b.navigatePanel('up');
        expect(b.activePanelState.value.selectedIndex).toBe(0);
    });

    it('does not move past the first or last item', () => {
        const b = useBindings();
        b.showPanel('sounds');
        b.navigatePanel('left'); // already at 0
        expect(b.activePanelState.value.selectedIndex).toBe(0);
        b.navigatePanel('up');   // already top row
        expect(b.activePanelState.value.selectedIndex).toBe(0);
    });

    it('show / hide / toggle drive the active panel id', () => {
        const b = useBindings();
        b.showPanel('sounds');
        expect(b.activePanelState.value.activePanelId).toBe('sounds');
        b.hidePanel();
        expect(b.activePanelState.value.activePanelId).toBeNull();
        b.togglePanel('sounds'); // closed → open
        expect(b.activePanelState.value.activePanelId).toBe('sounds');
        b.togglePanel('sounds'); // open → closed
        expect(b.activePanelState.value.activePanelId).toBeNull();
    });
});

// ─── Source list (playlists) ─────────────────────────────────────────
//
// Playlists replaced the single `group` string an item used to carry.
// The panel's source rail renders these; picking one filters the grid
// to its members AND puts them in that playlist's order, which is the
// behaviour a group-name filter could never have.

describe('panel source list', () => {
    it('offers only the playlists matching the panel\'s kind', () => {
        const b = useBindings();
        expect(b.groupsForPanel('sounds').map(p => p.id))
            .toEqual(['pl_quiet', 'pl_loud']);
        expect(b.groupsForPanel('animations').map(p => p.id)).toEqual(['pl_anim']);
    });

    it('has no playlists for a panel whose kind has none', () => {
        // This is what hides the rail entirely rather than showing a
        // lone "All" row.
        const b = useBindings();
        expect(b.groupsForPanel('poses')).toEqual([]);
    });

    it('opens showing everything by default', () => {
        const b = useBindings();
        b.showPanel('sounds');
        expect(b.activePanelSelectedGroup.value).toBe('All');
        expect(b.activePanelItems.value).toHaveLength(6);
    });

    it('filters the grid to the selected playlist', () => {
        const b = useBindings();
        b.showPanel('sounds');
        b.setActiveGroup('pl_quiet');
        expect(b.activePanelItems.value.map(i => i.id)).toEqual(['s1', 's3']);
    });

    it('orders the grid by the playlist, not the library', () => {
        // pl_loud lists s4 before s0; the library has s0 first. The
        // playlist wins -- that ordering is the operator's arrangement.
        const b = useBindings();
        b.showPanel('sounds');
        b.setActiveGroup('pl_loud');
        expect(b.activePanelItems.value.map(i => i.id)).toEqual(['s4', 's0']);
    });

    it('drops playlist members the library no longer has', () => {
        // pl_loud names 'ghost'; rendering it would be an empty tile
        // that does nothing when tapped.
        const b = useBindings();
        b.showPanel('sounds');
        b.setActiveGroup('pl_loud');
        expect(b.activePanelItems.value.map(i => i.id)).not.toContain('ghost');
    });

    it('falls back to everything for an unknown playlist id', () => {
        const b = useBindings();
        b.showPanel('sounds');
        b.setActiveGroup('pl_deleted');
        expect(b.activePanelItems.value).toHaveLength(6);
    });

    it('resets the selection when the source changes', () => {
        const b = useBindings();
        b.showPanel('sounds');
        b.navigatePanel('right');
        expect(b.activePanelState.value.selectedIndex).toBe(1);
        b.setActiveGroup('pl_quiet');
        expect(b.activePanelState.value.selectedIndex).toBe(0);
        expect(b.activePanelState.value.currentPage).toBe(0);
    });

    it('opens on a button\'s configured playlist id', () => {
        const b = useBindings();
        b.showPanel('sounds', 'pl_quiet');
        expect(b.activePanelSelectedGroup.value).toBe('pl_quiet');
        expect(b.activePanelItems.value.map(i => i.id)).toEqual(['s1', 's3']);
    });

    it('still honours a profile that saved a group NAME', () => {
        // Bindings written before playlists stored the group's name.
        // Resolving it by name keeps those buttons working instead of
        // silently opening on everything.
        const b = useBindings();
        b.showPanel('sounds', 'Quiet');
        expect(b.activePanelSelectedGroup.value).toBe('pl_quiet');
    });

    it('matches a saved name case-insensitively', () => {
        const b = useBindings();
        b.showPanel('sounds', 'loud');
        expect(b.activePanelSelectedGroup.value).toBe('pl_loud');
    });

    it('opens on everything when the saved source no longer exists', () => {
        // A stale binding should look unconfigured, not broken.
        const b = useBindings();
        b.showPanel('sounds', 'Deleted Playlist');
        expect(b.activePanelSelectedGroup.value).toBe('All');
        expect(b.activePanelItems.value).toHaveLength(6);
    });
});
