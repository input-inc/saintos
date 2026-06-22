/**
 * useKeyboard is the on-screen-keyboard controller: enable/disable (with
 * localStorage persistence), show/hide, and submitValue writing back to
 * the focused field + notifying a callback. No Tauri — pure DOM, so it
 * runs straight in happy-dom.
 */
import { describe, it, expect, beforeEach } from 'vitest';
import { useKeyboard } from '../useKeyboard';

const STORAGE_KEY = 'saint-controller-osk-enabled';

describe('useKeyboard', () => {
    beforeEach(() => {
        // Known starting point — the composable is a module singleton.
        useKeyboard().setEnabled(true);
        useKeyboard().hide();
    });

    it('shows and hides when enabled', () => {
        const kb = useKeyboard();
        kb.show();
        expect(kb.isVisible.value).toBe(true);
        kb.hide();
        expect(kb.isVisible.value).toBe(false);
    });

    it('refuses to show while disabled, and hides if disabled mid-show', () => {
        const kb = useKeyboard();
        kb.show();
        expect(kb.isVisible.value).toBe(true);
        kb.setEnabled(false);             // disabling hides an open keyboard
        expect(kb.isVisible.value).toBe(false);
        kb.show();                        // and blocks re-showing
        expect(kb.isVisible.value).toBe(false);
    });

    it('persists the enabled flag to localStorage', () => {
        const kb = useKeyboard();
        kb.setEnabled(false);
        expect(localStorage.getItem(STORAGE_KEY)).toBe('0');
        kb.setEnabled(true);
        expect(localStorage.getItem(STORAGE_KEY)).toBe('1');
    });

    it('submitValue writes to the field, fires the callback, and hides', () => {
        const kb = useKeyboard();
        const input = document.createElement('input');
        input.type = 'text';
        document.body.appendChild(input);

        let submitted: string | null = null;
        kb.onSubmit(v => { submitted = v; });
        kb.show(input);
        kb.submitValue('hello world');

        expect(input.value).toBe('hello world');
        expect(submitted).toBe('hello world');
        expect(kb.isVisible.value).toBe(false);
        input.remove();
    });

    it('detects whether a text field is focused', () => {
        const kb = useKeyboard();
        const input = document.createElement('input');
        input.type = 'text';
        document.body.appendChild(input);
        input.focus();
        expect(kb.isTextFieldFocused()).toBe(true);
        input.blur();
        input.remove();
    });
});
