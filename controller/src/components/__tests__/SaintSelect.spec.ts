/**
 * SaintSelect is the app-wide dropdown (a headlessui Listbox wrapper).
 * The logic this component actually owns is the displayed label —
 * selected option's label, or the placeholder when nothing matches —
 * and forwarding v-model. headlessui's open/keyboard internals are
 * third-party and not re-tested here. We render the button (closed
 * state) and assert the label resolves correctly across prop changes.
 */
import { describe, it, expect } from 'vitest';
import { mount } from '@vue/test-utils';
import SaintSelect from '../SaintSelect.vue';

const OPTIONS = [
    { value: 1.0, label: '100%' },
    { value: 1.25, label: '125%' },
    { value: 1.5, label: '150%' },
];

describe('SaintSelect', () => {
    it('shows the label of the option matching modelValue', () => {
        const w = mount(SaintSelect, { props: { modelValue: 1.25, options: OPTIONS } });
        expect(w.text()).toContain('125%');
    });

    it('falls back to the placeholder when no option matches', () => {
        const w = mount(SaintSelect, {
            props: { modelValue: 99, options: OPTIONS, placeholder: 'Pick one' },
        });
        expect(w.text()).toContain('Pick one');
    });

    it('updates the displayed label when modelValue changes', async () => {
        const w = mount(SaintSelect, { props: { modelValue: 1.0, options: OPTIONS } });
        expect(w.text()).toContain('100%');
        await w.setProps({ modelValue: 1.5 });
        expect(w.text()).toContain('150%');
        expect(w.text()).not.toContain('100%');
    });

    it('renders without a label when no match and no placeholder', () => {
        const w = mount(SaintSelect, { props: { modelValue: 99, options: OPTIONS } });
        // No throw, and none of the option labels leak into the button.
        for (const o of OPTIONS) expect(w.text()).not.toContain(o.label);
    });
});
