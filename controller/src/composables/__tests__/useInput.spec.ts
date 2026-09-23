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

/** A fresh payload, the way the Tauri bridge delivers one — every call
 *  builds new objects. useInput relies on that: it keeps the previous
 *  button map by reference rather than copying it. */
function frame(buttons: Record<string, boolean>, axes: Partial<{
    leftStickX: number; leftTrigger: number; pitch: number; padTouched: boolean;
}> = {}) {
    return {
        payload: {
            gamepad: {
                connected: true, name: 'test',
                leftStick: { x: axes.leftStickX ?? 0, y: 0 }, rightStick: { x: 0, y: 0 },
                leftTrigger: axes.leftTrigger ?? 0, rightTrigger: 0, buttons,
            },
            gyro: { pitch: axes.pitch ?? 0, roll: 0, yaw: 0 },
            leftTouchpad: { x: 0, y: 0, touched: axes.padTouched ?? false, clicked: false },
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

    // The state is held in a shallowRef: only the top-level replacement
    // is reactive, because the payload arrives fresh each event and
    // nothing mutates a nested field. These pin that the computed views
    // still see every replacement — a deep ref would too, so this is the
    // test that would catch someone "optimising" the ref into something
    // that doesn't invalidate.
    it('propagates a replaced payload to every computed view', async () => {
        const input = useInput();
        await vi.waitFor(() => expect(bridge.listeners.has('input-state')).toBe(true));
        const fire = bridge.listeners.get('input-state')!;

        fire(frame({}, { leftStickX: 0.5, leftTrigger: 0.25, pitch: 120, padTouched: true }));
        expect(input.gamepad.value.leftStick.x).toBe(0.5);
        expect(input.gamepad.value.leftTrigger).toBe(0.25);
        expect(input.gyro.value.pitch).toBe(120);
        expect(input.leftTouchpad.value.touched).toBe(true);
        expect(input.isGamepadConnected.value).toBe(true);

        // A second replacement must invalidate the computeds again, not
        // serve the first frame's cached values.
        fire(frame({}, { leftStickX: -0.25, pitch: -60 }));
        expect(input.gamepad.value.leftStick.x).toBe(-0.25);
        expect(input.gyro.value.pitch).toBe(-60);
        expect(input.leftTouchpad.value.touched).toBe(false);
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
