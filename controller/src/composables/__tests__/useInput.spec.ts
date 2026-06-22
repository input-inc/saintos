/**
 * useInput listens for Rust-emitted `input-state` events and republishes
 * gamepad button press/release EDGES through onButtonEvent. The edge
 * detection is the testable bit (it drives every digital binding's
 * UI-side reaction). We mock the Tauri event bridge so we can capture
 * the registered `input-state` callback and feed it synthetic frames.
 */
import { describe, it, expect, vi } from 'vitest';

// Capture listeners the composable registers via listen().
const bridge = vi.hoisted(() => ({ listeners: new Map<string, (e: any) => void>() }));

vi.mock('@tauri-apps/api/event', () => ({
    listen: vi.fn((event: string, cb: (e: any) => void) => {
        bridge.listeners.set(event, cb);
        return Promise.resolve(() => bridge.listeners.delete(event));
    }),
}));
vi.mock('@tauri-apps/api/core', () => ({
    invoke: vi.fn(() => Promise.resolve(undefined)),
}));

import { useInput } from '../useInput';

function frame(buttons: Record<string, boolean>) {
    return {
        payload: {
            gamepad: {
                connected: true, name: 'test',
                leftStick: { x: 0, y: 0 }, rightStick: { x: 0, y: 0 },
                leftTrigger: 0, rightTrigger: 0, buttons,
            },
            gyro: { pitch: 0, roll: 0, yaw: 0 },
            leftTouchpad: { x: 0, y: 0, touched: false, clicked: false },
            rightTouchpad: { x: 0, y: 0, touched: false, clicked: false },
        },
    };
}

describe('useInput button edge detection', () => {
    it('emits a single event per press and release edge', async () => {
        const input = useInput();
        await vi.waitFor(() => expect(bridge.listeners.has('input-state')).toBe(true));
        const fire = bridge.listeners.get('input-state')!;

        const events: Array<{ button: string; pressed: boolean }> = [];
        const off = input.onButtonEvent(e => events.push(e));

        fire(frame({ A: true }));            // A pressed
        fire(frame({ A: true, B: true }));   // B pressed (A unchanged → no repeat)
        fire(frame({ A: true, B: true }));   // nothing changed → no events
        fire(frame({ A: false, B: true }));  // A released
        fire(frame({}));                     // B drops out → treated as released
        off();

        expect(events).toEqual([
            { button: 'A', pressed: true },
            { button: 'B', pressed: true },
            { button: 'A', pressed: false },
            { button: 'B', pressed: false },
        ]);
    });

    it('stops delivering to an unsubscribed listener', async () => {
        const input = useInput();
        await vi.waitFor(() => expect(bridge.listeners.has('input-state')).toBe(true));
        const fire = bridge.listeners.get('input-state')!;

        const seen: number[] = [];
        const off = input.onButtonEvent(() => seen.push(1));
        fire(frame({ X: true }));
        off();
        fire(frame({ X: false }));
        expect(seen).toHaveLength(1); // only the press, not the post-unsubscribe release
    });
});
