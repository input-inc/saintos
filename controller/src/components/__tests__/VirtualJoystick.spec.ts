/**
 * VirtualJoystick is the on-screen control-path UI: it turns pointer
 * drags into a normalized (x, y) in [-1, 1] that the operator can route
 * to a robot. These tests lock the pointer→normalized math, the
 * circular clamp, the return-to-center on release, and the external-
 * input noise floor.
 *
 * happy-dom doesn't lay out elements, so getBoundingClientRect() returns
 * an all-zero rect → the joystick centre sits at (0, 0). That makes the
 * pointer coordinates we pass map directly to offsets from centre, which
 * is exactly what we want for deterministic assertions. maxRadius for the
 * default 80px base / 40px knob is (80 - 40) / 2 = 20.
 */
import { describe, it, expect } from 'vitest';
import { mount } from '@vue/test-utils';
import VirtualJoystick from '../VirtualJoystick.vue';

type Pos = { x: number; y: number };

function lastEmit(wrapper: ReturnType<typeof mount>): Pos {
    const ev = wrapper.emitted('position-change');
    if (!ev || ev.length === 0) throw new Error('no position-change emitted');
    return ev[ev.length - 1][0] as Pos;
}

describe('VirtualJoystick', () => {
    it('emits normalized x on a horizontal drag', async () => {
        const w = mount(VirtualJoystick);
        await w.trigger('mousedown', { clientX: 20, clientY: 0 }); // full right at maxRadius
        const p = lastEmit(w);
        expect(p.x).toBeCloseTo(1, 5);
        expect(p.y).toBeCloseTo(0, 5);
    });

    it('inverts Y so dragging up is positive', async () => {
        const w = mount(VirtualJoystick);
        await w.trigger('mousedown', { clientX: 0, clientY: -20 }); // up
        const p = lastEmit(w);
        expect(p.x).toBeCloseTo(0, 5);
        expect(p.y).toBeCloseTo(1, 5);
    });

    it('clamps drags beyond the base radius to the unit circle', async () => {
        const w = mount(VirtualJoystick);
        await w.trigger('mousedown', { clientX: 100, clientY: 0 }); // way past edge
        const p = lastEmit(w);
        expect(p.x).toBeCloseTo(1, 5); // clamped, not 5.0
        expect(p.y).toBeCloseTo(0, 5);
    });

    it('clamps diagonal drags so magnitude never exceeds 1', async () => {
        const w = mount(VirtualJoystick);
        await w.trigger('mousedown', { clientX: 40, clientY: -40 });
        const p = lastEmit(w);
        expect(Math.hypot(p.x, p.y)).toBeLessThanOrEqual(1 + 1e-6);
        // 45° up-right → both components equal and positive.
        expect(p.x).toBeCloseTo(Math.SQRT1_2, 4);
        expect(p.y).toBeCloseTo(Math.SQRT1_2, 4);
    });

    it('returns to centre and emits zero on release', async () => {
        const w = mount(VirtualJoystick);
        await w.trigger('mousedown', { clientX: 20, clientY: 0 });
        // Release is tracked on a window-level mouseup listener.
        window.dispatchEvent(new Event('mouseup'));
        await w.vm.$nextTick();
        const p = lastEmit(w);
        expect(p.x).toBeCloseTo(0, 5);
        expect(p.y).toBeCloseTo(0, 5);
    });

    it('flags external input only above the 0.05 noise floor', () => {
        const live = mount(VirtualJoystick, {
            props: { externalPosition: { x: 0.5, y: 0.0 } },
        });
        expect(live.classes()).toContain('border-saint-success');

        const idle = mount(VirtualJoystick, {
            props: { externalPosition: { x: 0.01, y: -0.02 } }, // within noise floor
        });
        expect(idle.classes()).not.toContain('border-saint-success');
    });
});
