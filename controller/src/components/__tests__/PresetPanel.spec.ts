/**
 * PresetPanel renders the active panel's items, highlights the selected
 * one, and dispatches selection/navigation back through useBindings.
 * We mock the three composables it consumes so we can drive panel state
 * directly and assert the render + the dispatch wiring (the bits that
 * break when someone refactors the panel UI).
 */
import { describe, it, expect, beforeEach, vi } from 'vitest';
import { mount } from '@vue/test-utils';

// Shared, hoisted handles so the mock factories and the tests see the
// same spies + reactive-ish data holders.
const m = vi.hoisted(() => ({
    trigger: vi.fn(),
    hide: vi.fn(),
    nav: vi.fn(),
    panel: {
        value: {
            id: 'sounds', name: 'Sounds', icon: 'volume_up', color: '#22c55e',
            columns: 4, itemsPerPage: 8, presets: [],
        } as any,
    },
    state: { value: { activePanelId: 'sounds', selectedIndex: 1, currentPage: 0, selectedGroup: 'All', keepOpen: false } },
    items: { value: [{ id: 'a', name: 'Alpha' }, { id: 'b', name: 'Bravo' }, { id: 'c', name: 'Charlie' }] as any[] },
    source: { value: null as string | null },
    connected: { value: true },
    animLoaded: { value: true },
    posesLoaded: { value: true },
    soundsLoaded: { value: true },
    groups: { value: [] as any[] },
    selectedGroup: { value: 'All' },
    setGroup: vi.fn(),
    itemsRaw: { value: [] as any[] },
    displayPrefs: { value: { layout: 'horizontal', columns: 4, sourceListSide: 'left' } },
    setPrefs: vi.fn(),
    playing: { value: {} as Record<string, unknown> },
    itemsPerPage: { value: 8 },
    setItemsPerPage: vi.fn(),
}));

vi.mock('../../composables/useBindings', () => ({
    // The panel imports this constant alongside the composable; the mock
    // has to supply it or the "All" row renders undefined.
    ALL_SOURCES: 'All',
    useBindings: () => ({
        activePanel: m.panel,
        activePanelState: m.state,
        activePanelItems: m.items,
        activePanelItemsRaw: m.itemsRaw,
        activePanelSource: m.source,
        activePanelGroups: m.groups,
        activePanelSelectedGroup: m.selectedGroup,
        setActiveGroup: m.setGroup,
        activePanelItemsPerPage: m.itemsPerPage,
        setPanelItemsPerPage: m.setItemsPerPage,
        triggerActiveItem: m.trigger,
        hidePanel: m.hide,
        navigatePanel: m.nav,
    }),
}));
vi.mock('../../composables/useConnection', () => ({
    useConnection: () => ({ isConnected: m.connected }),
}));
vi.mock('../../composables/useDisplayPrefs', () => ({
    useDisplayPrefs: () => ({
        prefsFor: () => m.displayPrefs.value,
        setPrefs: m.setPrefs,
    }),
}));
vi.mock('../../composables/useLibrary', () => ({
    useLibrary: () => ({
        animationsLoaded: m.animLoaded,
        posesLoaded: m.posesLoaded,
        soundsLoaded: m.soundsLoaded,
        playing: m.playing,
    }),
}));

import PresetPanel from '../PresetPanel.vue';

describe('PresetPanel', () => {
    beforeEach(() => {
        // Reset to the baseline single-page panel before each test.
        m.trigger.mockClear();
        m.hide.mockClear();
        m.nav.mockClear();
        m.panel.value = {
            id: 'sounds', name: 'Sounds', icon: 'volume_up', color: '#22c55e',
            columns: 4, itemsPerPage: 8, presets: [],
        };
        m.state.value = {
            activePanelId: 'sounds', selectedIndex: 1, currentPage: 0,
            selectedGroup: 'All', keepOpen: false,
        };
        m.items.value = [{ id: 'a', name: 'Alpha' }, { id: 'b', name: 'Bravo' }, { id: 'c', name: 'Charlie' }];
        m.itemsRaw.value = m.items.value;
        m.source.value = null;
        m.connected.value = true;
        m.groups.value = [];
        m.selectedGroup.value = 'All';
        m.setGroup.mockClear();
        m.displayPrefs.value = { layout: 'horizontal', columns: 4, sourceListSide: 'left' };
    });

    it('renders one button per visible item with its name', () => {
        const w = mount(PresetPanel);
        const items = w.findAll('.preset-item');
        expect(items).toHaveLength(3);
        expect(w.text()).toContain('Alpha');
        expect(w.text()).toContain('Bravo');
        expect(w.text()).toContain('Charlie');
    });

    it('highlights the selected item only', () => {
        const w = mount(PresetPanel); // selectedIndex = 1 → Bravo
        const items = w.findAll('.preset-item');
        expect(items[0].classes()).not.toContain('preset-selected');
        expect(items[1].classes()).toContain('preset-selected');
        expect(items[2].classes()).not.toContain('preset-selected');
    });

    it('dispatches trigger + hide when an item is clicked', async () => {
        const w = mount(PresetPanel);
        await w.findAll('.preset-item')[0].trigger('click'); // Alpha
        expect(m.trigger).toHaveBeenCalledWith('a');
        expect(m.hide).toHaveBeenCalledOnce();
    });

    it('keeps the panel open after selecting when keepOpen is set', async () => {
        m.state.value.keepOpen = true;
        const w = mount(PresetPanel);
        await w.findAll('.preset-item')[0].trigger('click'); // Alpha
        expect(m.trigger).toHaveBeenCalledWith('a');
        expect(m.hide).not.toHaveBeenCalled();
        m.state.value.keepOpen = false; // restore for other tests
    });

    it('shows a connect hint for a server-backed panel while disconnected', () => {
        m.source.value = 'animations';
        m.connected.value = false;
        m.items.value = []; // nothing loaded yet
        const w = mount(PresetPanel);
        expect(w.text()).toMatch(/Connect to the robot to load animations/i);
    });

    it('exposes pager controls and dispatches next_page across multiple pages', async () => {
        // 10 items / 8 per page = 2 pages → footer pager appears.
        m.items.value = Array.from({ length: 10 }, (_, i) => ({ id: `i${i}`, name: `Item ${i}` }));
        const w = mount(PresetPanel);
        const buttons = w.findAll('.panel-footer button');
        const next = buttons.find(b => b.text().includes('chevron_right'));
        expect(next).toBeTruthy();
        await next!.trigger('click');
        expect(m.nav).toHaveBeenCalledWith('next_page');
    });
});

// ─── Source list ─────────────────────────────────────────────────────
//
// The rail of playlists beside the grid. It replaced the group dropdown
// that used to sit in the app header, so these pin the two rules that
// dropdown didn't have: it hides itself when there is nothing to choose
// between, and which side it sits on is a per-panel preference.

describe('PresetPanel source list', () => {
    const PLAYLISTS = [
        { id: 'pl_quiet', name: 'Quiet', kind: 'sounds', items: ['a', 'b'] },
        { id: 'pl_loud', name: 'Loud', kind: 'sounds', items: ['c'] },
    ];

    // The hoisted holders are module-scoped and shared, and the other
    // describe's beforeEach doesn't reach in here -- reset explicitly or
    // one test's playlists leak into the next.
    beforeEach(() => {
        m.panel.value = {
            id: 'sounds', name: 'Sounds', icon: 'volume_up', color: '#22c55e',
            columns: 4, itemsPerPage: 8, presets: [],
        };
        m.state.value = {
            activePanelId: 'sounds', selectedIndex: 0, currentPage: 0,
            selectedGroup: 'All', keepOpen: false,
        };
        m.items.value = [
            { id: 'a', name: 'Alpha' }, { id: 'b', name: 'Bravo' }, { id: 'c', name: 'Charlie' },
        ];
        m.itemsRaw.value = m.items.value;
        m.groups.value = [];
        m.selectedGroup.value = 'All';
        m.setGroup.mockClear();
        m.displayPrefs.value = { layout: 'horizontal', columns: 4, sourceListSide: 'left' };
    });

    it('is not rendered when the panel has no playlists', () => {
        const w = mount(PresetPanel);
        expect(w.find('.source-list').exists()).toBe(false);
    });

    it('renders a row per playlist plus All', () => {
        m.groups.value = PLAYLISTS;
        const w = mount(PresetPanel);
        const rows = w.findAll('.source-row');
        expect(rows).toHaveLength(3);
        expect(rows[0].text()).toContain('All');
        expect(rows[1].text()).toContain('Quiet');
        expect(rows[2].text()).toContain('Loud');
    });

    it('counts what each source would actually show', () => {
        m.groups.value = PLAYLISTS;
        const w = mount(PresetPanel);
        const rows = w.findAll('.source-row');
        expect(rows[0].text()).toContain('3');   // All
        expect(rows[1].text()).toContain('2');   // Quiet: a, b
        expect(rows[2].text()).toContain('1');   // Loud: c
    });

    it('does not count playlist members the library no longer has', () => {
        m.groups.value = [{ id: 'pl_x', name: 'X', kind: 'sounds', items: ['a', 'ghost'] }];
        const w = mount(PresetPanel);
        expect(w.findAll('.source-row')[1].text()).toContain('1');
    });

    it('marks the selected source', () => {
        m.groups.value = PLAYLISTS;
        m.selectedGroup.value = 'pl_loud';
        const w = mount(PresetPanel);
        const rows = w.findAll('.source-row');
        expect(rows[0].classes()).not.toContain('source-selected');
        expect(rows[2].classes()).toContain('source-selected');
    });

    it('dispatches the selection back through useBindings', async () => {
        m.groups.value = PLAYLISTS;
        const w = mount(PresetPanel);
        await w.findAll('.source-row')[1].trigger('click');
        expect(m.setGroup).toHaveBeenCalledWith('pl_quiet');
    });

    it('sits on the left by default', () => {
        m.groups.value = PLAYLISTS;
        const w = mount(PresetPanel);
        expect(w.find('.source-list').classes()).toContain('order-0');
    });

    it('moves to the right when the panel prefers it', () => {
        m.groups.value = PLAYLISTS;
        m.displayPrefs.value = { layout: 'horizontal', columns: 4, sourceListSide: 'right' };
        const w = mount(PresetPanel);
        const rail = w.find('.source-list');
        expect(rail.classes()).toContain('order-2');
        expect(rail.classes()).not.toContain('order-0');
    });

    it('offers the side choice in display options only when a rail exists', async () => {
        const w = mount(PresetPanel);
        (w.vm as any).showDisplayModal = true;
        await w.vm.$nextTick();
        expect(w.text()).not.toContain('Source list');

        m.groups.value = PLAYLISTS;
        const w2 = mount(PresetPanel);
        (w2.vm as any).showDisplayModal = true;
        await w2.vm.$nextTick();
        expect(w2.text()).toContain('Source list');
    });
});
