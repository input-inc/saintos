/**
 * Virtual-keyboard composable — Vue equivalent of the Angular
 * `KeyboardService`. Manages the in-app on-screen keyboard for
 * gamepad/touch text entry, with localStorage persistence of the
 * enabled/disabled preference.
 *
 * Default OFF: on Steam Deck Game Mode the Steam OSK (Steam + X) is
 * more discoverable and integrates with Steam Input. In Desktop Mode
 * the user typically has a physical or KDE virtual keyboard. The
 * settings panel can toggle this on via `setEnabled(true)`.
 */

import { computed, ref } from 'vue';

const STORAGE_KEY = 'saint-controller-osk-enabled';

function loadEnabled(): boolean {
    try {
        const raw = localStorage.getItem(STORAGE_KEY);
        if (raw === '1') return true;
        if (raw === '0') return false;
    } catch {
        // localStorage unavailable — fall through to default.
    }
    return false; // Default off: prefer platform OSK.
}

// ─── Module-level singleton state ────────────────────────────────────

const isVisibleRef = ref<boolean>(false);
const enabledRef = ref<boolean>(loadEnabled());

let activeInput: HTMLInputElement | HTMLTextAreaElement | null = null;
let onSubmitCallback: ((value: string) => void) | null = null;

let initialized = false;

function ensureInit(): void {
    if (initialized) return;
    initialized = true;
    setupGlobalListeners();
}

// ─── Public API ──────────────────────────────────────────────────────

/** Toggle the in-app keyboard on/off. Persists to localStorage so the
 *  next session starts in the same state. If you flip from on→off
 *  while a keyboard is open, the open instance is closed so the
 *  operator doesn't end up with a stuck overlay. */
function setEnabled(value: boolean): void {
    enabledRef.value = value;
    try {
        localStorage.setItem(STORAGE_KEY, value ? '1' : '0');
    } catch {
        // Ignore — localStorage in some sandboxed contexts can throw;
        // the in-memory ref still works for the session.
    }
    if (!value && isVisibleRef.value) {
        hide();
    }
}

/** Show the virtual keyboard for a specific input element. No-op
 *  when the in-app keyboard is disabled — the operator's relying on
 *  Steam OSK / KDE virtual keyboard / physical keyboard instead. */
function show(input?: HTMLInputElement | HTMLTextAreaElement): void {
    if (!enabledRef.value) return;
    if (input) {
        activeInput = input;
    }
    isVisibleRef.value = true;
}

function hide(): void {
    isVisibleRef.value = false;
    activeInput = null;
    onSubmitCallback = null;
}

function submitValue(value: string): void {
    if (activeInput) {
        activeInput.value = value;
        // Fire input + change so any v-model bindings (Vue's
        // equivalent of Angular's [(ngModel)]) detect the change.
        activeInput.dispatchEvent(new Event('input', { bubbles: true }));
        activeInput.dispatchEvent(new Event('change', { bubbles: true }));
    }
    if (onSubmitCallback) {
        onSubmitCallback(value);
    }
    hide();
}

function getCurrentInputValue(): string {
    return activeInput?.value ?? '';
}

function onSubmit(callback: (value: string) => void): void {
    onSubmitCallback = callback;
}

function isTextFieldFocused(): boolean {
    return isTextInput(document.activeElement as HTMLElement | null);
}

function isTextInput(element: HTMLElement | null): boolean {
    if (!element) return false;
    if (element.tagName === 'INPUT') {
        const inputType = (element as HTMLInputElement).type.toLowerCase();
        const textTypes = ['text', 'password', 'email', 'search', 'tel', 'url', 'number'];
        return textTypes.includes(inputType);
    }
    if (element.tagName === 'TEXTAREA') return true;
    if (element.isContentEditable) return true;
    return false;
}

function setupGlobalListeners(): void {
    // Focus listener stays registered unconditionally so a runtime
    // toggle-on takes effect on the next focus event without a
    // reload. The show() call early-returns when disabled.
    document.addEventListener('focusin', event => {
        const target = event.target as HTMLElement;
        if (!enabledRef.value) return;
        if (isTextInput(target)) {
            activeInput = target as HTMLInputElement | HTMLTextAreaElement;
            show();
        }
    });

    // We deliberately don't auto-hide on focusout — clicking a key on
    // the virtual keyboard component briefly removes focus from the
    // input, and we don't want that to dismiss the keyboard. The
    // keyboard component handles its own dismissal via a Done button.
    document.addEventListener('focusout', () => {
        // intentional no-op
    });
}

// ─── Composable export ───────────────────────────────────────────────

export function useKeyboard() {
    ensureInit();
    return {
        isVisible: computed(() => isVisibleRef.value),
        enabled: computed(() => enabledRef.value),

        setEnabled,
        show,
        hide,
        submitValue,
        getCurrentInputValue,
        onSubmit,
        isTextFieldFocused,
    };
}
