<!--
    On-screen keyboard overlay used when the operator focuses a text
    input and the platform OSK isn't carrying that case. The
    simple-keyboard library renders the key grid; we layer a manual
    pointerdown delegate on top because simple-keyboard's own touch
    handlers don't fire cleanly under gamescope + Flatpak in Steam
    Deck Game Mode (the original reason the manual delegate exists).
-->
<script setup lang="ts">
import { computed, nextTick, ref, watch, onBeforeUnmount } from 'vue';
import Keyboard from 'simple-keyboard';
import { useKeyboard } from '../composables/useKeyboard';

type KeyboardLayout = 'default' | 'shift' | 'symbols' | 'symbols-shift';

const keyboard = useKeyboard();
const containerEl = ref<HTMLDivElement | null>(null);

const inputValue = ref<string>('');
const currentLayout = ref<KeyboardLayout>('default');
let sk: Keyboard | null = null;

function initKeyboard(): void {
    if (sk || !containerEl.value) return;

    sk = new Keyboard(containerEl.value, {
        onChange: input => { inputValue.value = input; },
        onKeyPress: button => { handleKeyPress(button); },
        layout: {
            default: [
                '1 2 3 4 5 6 7 8 9 0',
                'q w e r t y u i o p',
                'a s d f g h j k l',
                '{shift} z x c v b n m {backspace}',
                '{symbols} {space} . {enter}',
            ],
            shift: [
                '1 2 3 4 5 6 7 8 9 0',
                'Q W E R T Y U I O P',
                'A S D F G H J K L',
                '{shift} Z X C V B N M {backspace}',
                '{symbols} {space} . {enter}',
            ],
            symbols: [
                '! @ # $ % ^ & * ( )',
                '- _ = + [ ] { } |',
                "; : ' \" , . < > ?",
                '{shift} / \\ ~ ` {backspace}',
                '{abc} {space} . {enter}',
            ],
            'symbols-shift': [
                '1 2 3 4 5 6 7 8 9 0',
                '€ £ ¥ © ® ™ § ¶ •',
                '° ± × ÷ ≠ ≈ ∞ µ',
                '{shift} … – — « » {backspace}',
                '{abc} {space} . {enter}',
            ],
        },
        display: {
            '{backspace}': '⌫',
            '{enter}': '↵',
            '{shift}': '⇧',
            '{space}': ' ',
            '{symbols}': '?123',
            '{abc}': 'ABC',
        },
        theme: 'simple-keyboard hg-theme-default',
        useButtonTag: true,
        disableButtonHold: true,
        // simple-keyboard's own touch/mouse handlers don't fire cleanly
        // under gamescope/Flatpak in Steam Deck Game Mode. We turn them
        // off and install our own pointerdown delegate below.
        useTouchEvents: false,
        useMouseEvents: false,
    });

    // Single pointerdown delegate on the wrapper. Catches every key
    // press regardless of input modality (touch / mouse / pen /
    // gamescope-synthesized) — that uniformity is the whole reason
    // pointer events exist on the Web platform.
    containerEl.value.addEventListener('pointerdown', (event: PointerEvent) => {
        const target = event.target as HTMLElement | null;
        if (!target) return;
        const btn = target.closest('[data-skbtn]') as HTMLElement | null;
        if (!btn) return;
        const key = btn.getAttribute('data-skbtn');
        if (!key) return;
        event.preventDefault();
        handleKeyPress(key);
    });

    sk.setInput(inputValue.value);
}

function destroyKeyboard(): void {
    if (sk) {
        sk.destroy();
        sk = null;
    }
}

function handleKeyPress(key: string): void {
    if (key === '{shift}')     { toggleShift();         return; }
    if (key === '{symbols}')   { setLayout('symbols');  return; }
    if (key === '{abc}')       { setLayout('default');  return; }
    if (key === '{enter}')     { onDone();              return; }
    if (key === '{backspace}') { onBackspace();         return; }
    if (key === '{space}')     { onSpace();             return; }

    // Regular character — append to the buffer.
    const next = inputValue.value + key;
    inputValue.value = next;
    sk?.setInput(next);

    // Auto-unshift after typing one letter in shift mode (mirrors
    // platform OSK conventions).
    if (currentLayout.value === 'shift') setLayout('default');
}

function toggleShift(): void {
    switch (currentLayout.value) {
        case 'default':       setLayout('shift'); break;
        case 'shift':         setLayout('default'); break;
        case 'symbols':       setLayout('symbols-shift'); break;
        case 'symbols-shift': setLayout('symbols'); break;
    }
}

function setLayout(layout: KeyboardLayout): void {
    currentLayout.value = layout;
    sk?.setOptions({ layoutName: layout });
}

function onBackspace(): void {
    if (inputValue.value.length > 0) {
        const next = inputValue.value.slice(0, -1);
        inputValue.value = next;
        sk?.setInput(next);
    }
}

function onSpace(): void {
    const next = inputValue.value + ' ';
    inputValue.value = next;
    sk?.setInput(next);
}

function onDone(): void  { keyboard.submitValue(inputValue.value); }
function onCancel(): void { keyboard.hide(); }
function onOverlayClick(_event: Event): void { onCancel(); }

// Show/hide lifecycle. When visibility flips on, seed the buffer with
// the current input's value and (re-)create simple-keyboard inside
// the freshly-mounted DOM. nextTick because the v-if branch needs to
// render before containerEl is non-null.
watch(() => keyboard.isVisible.value, async visible => {
    if (visible) {
        inputValue.value = keyboard.getCurrentInputValue();
        await nextTick();
        // Small delay so any pending mount/layout work settles first.
        setTimeout(() => initKeyboard(), 50);
    } else {
        destroyKeyboard();
    }
});

onBeforeUnmount(() => destroyKeyboard());
</script>

<template>
    <div v-if="keyboard.isVisible.value" class="keyboard-overlay" @click="onOverlayClick">
        <div class="keyboard-container" @click.stop>
            <div class="input-preview">
                <span class="preview-text">{{ inputValue }}</span>
                <span class="cursor">|</span>
            </div>

            <div ref="containerEl" class="keyboard-wrapper"></div>

            <div class="keyboard-toolbar">
                <button class="toolbar-btn" @click="onCancel">
                    <span class="material-icons">close</span>
                    Cancel
                </button>
                <div class="toolbar-spacer"></div>
                <button class="toolbar-btn" @click="onBackspace">
                    <span class="material-icons">backspace</span>
                </button>
                <button class="toolbar-btn" @click="onSpace">
                    <span class="material-icons">space_bar</span>
                    Space
                </button>
                <button class="toolbar-btn primary" @click="onDone">
                    <span class="material-icons">check</span>
                    Done
                </button>
            </div>
        </div>
    </div>
</template>

<style scoped>
.keyboard-overlay {
    position: fixed;
    inset: 0;
    background: rgba(15, 23, 42, 0.9);
    backdrop-filter: blur(4px);
    display: flex;
    align-items: flex-end;
    justify-content: center;
    z-index: 9999;
    padding-bottom: 1rem;
}

.keyboard-container {
    width: 100%;
    max-width: 900px;
    max-height: calc(100% - 1rem);
    background: #1e293b;
    border-radius: 1rem 1rem 0 0;
    border: 1px solid #334155;
    border-bottom: none;
    overflow-y: auto;
    box-shadow: 0 -4px 20px rgba(0, 0, 0, 0.5);
    display: flex;
    flex-direction: column;
}

.input-preview {
    padding: 1rem 1.5rem;
    background: #0f172a;
    border-bottom: 1px solid #334155;
    font-family: 'Inter', sans-serif;
    font-size: 1.25rem;
    color: #f1f5f9;
    min-height: 3.5rem;
    display: flex;
    align-items: center;
    flex-shrink: 0;
}

.preview-text {
    white-space: pre-wrap;
    word-break: break-all;
}

.cursor {
    animation: blink 1s infinite;
    color: #3b82f6;
    margin-left: 1px;
}

@keyframes blink {
    0%, 50% { opacity: 1; }
    51%, 100% { opacity: 0; }
}

.keyboard-wrapper {
    padding: 0.75rem;
    touch-action: manipulation;
    overflow-y: auto;
    min-height: 0;
    flex: 1;
}

.keyboard-toolbar {
    display: flex;
    align-items: center;
    gap: 0.5rem;
    padding: 0.75rem 1rem;
    background: #0f172a;
    border-top: 1px solid #334155;
    flex-shrink: 0;
}

.toolbar-spacer {
    flex: 1;
}

.toolbar-btn {
    display: flex;
    align-items: center;
    gap: 0.5rem;
    padding: 0.75rem 1.25rem;
    background: #334155;
    border: 1px solid #475569;
    border-radius: 0.5rem;
    color: #f1f5f9;
    font-family: 'Inter', sans-serif;
    font-size: 1rem;
    font-weight: 500;
    cursor: pointer;
    transition: all 0.15s ease;
}

.toolbar-btn:hover { background: #475569; }
.toolbar-btn:active {
    background: #3b82f6;
    transform: translateY(1px);
}
.toolbar-btn.primary {
    background: #3b82f6;
    border-color: #60a5fa;
}
.toolbar-btn.primary:hover { background: #2563eb; }
.toolbar-btn .material-icons { font-size: 1.25rem; }
</style>
