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
    groups: { value: [] as string[] },
    selectedGroup: { value: 'All' },
    setGroup: vi.fn(),
    playing: { value: {} as Record<string, unknown> },
    itemsPerPage: { value: 8 },
    setItemsPerPage: vi.fn(),
}));

vi.mock('../../composables/useBindings', () => ({
    useBindings: () => ({
        activePanel: m.panel,
        activePanelState: m.state,
        activePanelItems: m.items,
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
        m.state.value = { activePanelId: 'sounds', selectedIndex: 1, currentPage: 0 };
        m.items.value = [{ id: 'a', name: 'Alpha' }, { id: 'b', name: 'Bravo' }, { id: 'c', name: 'Charlie' }];
        m.source.value = null;
        m.connected.value = true;
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
