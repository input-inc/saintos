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
vi.mock('../useLibrary', () => {
    const items = Array.from({ length: 6 }, (_, i) => ({ id: `s${i}`, name: `S${i}` }));
    const box = (v: unknown) => ({ value: v });
    return {
        useLibrary: () => ({
            animations: box([]), poses: box([]), sounds: box(items),
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
