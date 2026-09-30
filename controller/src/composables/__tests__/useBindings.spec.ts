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
    BINDABLE_DIGITAL_INPUTS,
    DIGITAL_INPUTS,
    digitalInputLabel,
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

// ─── The digital-input catalog ───────────────────────────────────────
//
// The bindings editor used to keep its own hand-written list of buttons
// and it simply never mentioned the Steam Deck's back buttons — L4, L5,
// R4 and R5 were read over HID, carried through the mapper, and
// impossible to bind because no dropdown offered them. The catalog is
// now the single source and `DigitalInput` is derived from it, so a
// button cannot exist in the type without a label. These guard the
// contents against the Rust enum it mirrors.

describe('digital input catalog', () => {
    // src-tauri/src/bindings/config.rs DigitalInput, after serde renames.
    const RUST_NAMES = [
        'a', 'b', 'x', 'y', 'lb', 'rb',
        'd_pad_up', 'd_pad_down', 'd_pad_left', 'd_pad_right',
        'start', 'select', 'left_stick', 'right_stick',
        'l4', 'r4', 'l5', 'r5', 'steam',
    ];

    it('covers every input the Rust mapper can read', () => {
        expect([...DIGITAL_INPUTS].map(i => i.value).sort())
            .toEqual([...RUST_NAMES].sort());
    });

    it('offers the Steam Deck back buttons for binding', () => {
        const offered = BINDABLE_DIGITAL_INPUTS.map(i => i.value);
        for (const b of ['l4', 'r4', 'l5', 'r5']) expect(offered).toContain(b);
    });

    it('does not offer the Steam button', () => {
        // It opens the Steam overlay at the OS level; a binding on it
        // would fire an action and leave the overlay up.
        expect(BINDABLE_DIGITAL_INPUTS.map(i => i.value)).not.toContain('steam');
        expect(DIGITAL_INPUTS.map(i => i.value)).toContain('steam');
    });

    it('gives every input a label distinct from its key', () => {
        for (const i of DIGITAL_INPUTS) {
            expect(i.label.length).toBeGreaterThan(0);
            expect(i.label).not.toBe(i.value);
        }
    });

    it('has no duplicate values or labels', () => {
        const values = DIGITAL_INPUTS.map(i => i.value);
        const labels = DIGITAL_INPUTS.map(i => i.label);
        expect(new Set(values).size).toBe(values.length);
        expect(new Set(labels).size).toBe(labels.length);
    });

    it('labels the back buttons by their hardware markings', () => {
        expect(digitalInputLabel('l4')).toContain('L4');
        expect(digitalInputLabel('r5')).toContain('R5');
    });

    it('falls back to the raw key for something unknown', () => {
        // An older profile naming a button we no longer have must still
        // render as a row the operator can delete.
        expect(digitalInputLabel('paddle9')).toBe('paddle9');
    });
});

// ─── Boards ──────────────────────────────────────────────────────────
//
// The Bindings → Boards tab browses the server's boards instead of
// letting the operator invent panels. The three boards mirror what the
// server's Boards page manages; a panel with no server source has no
// items to show, so it is not a board.

describe('boards', () => {
    it('exposes the three server-backed boards', () => {
        const b = useBindings();
        expect(b.boards.value.map(x => x.id)).toEqual(['animations', 'poses', 'sounds']);
    });

    it('carries the board kind through for the item lookup', () => {
        const b = useBindings();
        expect(b.boards.value.map(x => x.kind)).toEqual(['animations', 'poses', 'sounds']);
    });

    it('lists a board\'s whole library under All', () => {
        const b = useBindings();
        expect(b.boardItems('sounds', 'All').map(i => i.id))
            .toEqual(['s0', 's1', 's2', 's3', 's4', 's5']);
    });

    it('filters and reorders a board by playlist', () => {
        // Same resolution the panel overlay uses, so what is browsed here
        // is what the panel will show.
        const b = useBindings();
        expect(b.boardItems('sounds', 'pl_loud').map(i => i.id)).toEqual(['s4', 's0']);
    });

    it('returns nothing for a board that is not in the profile', () => {
        const b = useBindings();
        expect(b.boardItems('nope', 'All')).toEqual([]);
    });

    it('offers only that board\'s playlists', () => {
        const b = useBindings();
        expect(b.groupsForPanel('sounds').map(p => p.id)).toEqual(['pl_quiet', 'pl_loud']);
        expect(b.groupsForPanel('animations').map(p => p.id)).toEqual(['pl_anim']);
    });

    it('no longer offers a way to create a panel', () => {
        // Panels are the server's. The button that used to make one here
        // produced an empty panel with no editor to fill it.
        const b = useBindings() as Record<string, unknown>;
        expect(b['addPresetPanel']).toBeUndefined();
    });

    it('firing a board item does not open a panel', () => {
        const b = useBindings();
        b.hidePanel();
        b.triggerBoardItem('sounds', 's1');
        expect(b.activePanelState.value.activePanelId).toBeNull();
    });

    it('remembers a fired item as that board\'s last selection', () => {
        // Browsing and firing from this tab should leave the panel
        // highlight where the operator last acted.
        const b = useBindings();
        b.triggerBoardItem('sounds', 's3');
        b.showPanel('sounds');
        const items = b.activePanelItems.value;
        expect(items[b.activePanelState.value.selectedIndex].id).toBe('s3');
    });
});

// ─── Activate Board Item ─────────────────────────────────────────────
//
// Replaces the old "Activate Preset", which fired a preset stored on a
// static panel — and static panels can no longer be created, so it had
// become unreachable. The binding now names a board and an item on it.
//
// The playlist the operator picked while choosing is remembered for the
// editor's benefit only: firing goes by item id, so renaming or deleting
// the playlist afterwards must not break the binding.

describe('activate_board_item', () => {
    it('fires the named item on the named board', () => {
        const b = useBindings();
        b.hidePanel();
        b.triggerBoardItem('sounds', 's2');
        // Recorded as that board's last selection — the same bookkeeping
        // selecting it in the panel overlay does.
        b.showPanel('sounds');
        const items = b.activePanelItems.value;
        expect(items[b.activePanelState.value.selectedIndex].id).toBe('s2');
    });

    it('fires an item that is in a playlist the binding never named', () => {
        // The stored playlist is picker scope, not a filter on firing.
        const b = useBindings();
        b.triggerBoardItem('sounds', 's4');
        b.showPanel('sounds');
        expect(b.activePanelItems.value[b.activePanelState.value.selectedIndex].id)
            .toBe('s4');
    });

    it('ignores an item on a board the profile does not have', () => {
        const b = useBindings();
        expect(() => b.triggerBoardItem('nope', 's0')).not.toThrow();
    });

    it('narrowing by playlist is what shortens the pickable list', () => {
        // What the editor's Item dropdown is built from.
        const b = useBindings();
        expect(b.boardItems('sounds', 'All')).toHaveLength(6);
        expect(b.boardItems('sounds', 'pl_quiet').map(i => i.id)).toEqual(['s1', 's3']);
    });
});
