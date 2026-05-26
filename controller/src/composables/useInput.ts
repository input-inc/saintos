/**
 * Input composable — Vue equivalent of the Angular `InputService`.
 * Listens for `input-state` Tauri events (emitted by the Rust input
 * manager on every gamepad poll) and exposes the latest gamepad /
 * gyro / touchpad state as reactive refs.
 *
 * Also publishes button press/release edges through `onButtonEvent` —
 * components subscribe with a callback that fires once per state
 * change. This replaces the Angular RxJS `buttonEvents$` observable;
 * the API is simpler (callback-per-listener instead of an Observable
 * + Subscription) which fits Vue's lifecycle hooks.
 */

import { computed, ref } from 'vue';
import { invoke } from '@tauri-apps/api/core';
import { listen, type UnlistenFn } from '@tauri-apps/api/event';

export interface AxisState {
    x: number;
    y: number;
}

export interface GyroState {
    pitch: number;
    roll: number;
    yaw: number;
}

export interface TouchpadState {
    x: number;
    y: number;
    touched: boolean;
    clicked: boolean;
}

export interface GamepadState {
    connected: boolean;
    name: string;
    leftStick: AxisState;
    rightStick: AxisState;
    leftTrigger: number;
    rightTrigger: number;
    buttons: { [key: string]: boolean };
}

export interface InputState {
    gamepad: GamepadState;
    gyro: GyroState;
    leftTouchpad: TouchpadState;
    rightTouchpad: TouchpadState;
}

export interface ButtonEvent {
    button: string;
    pressed: boolean;
}

export type ButtonEventListener = (event: ButtonEvent) => void;

const DEFAULT_INPUT_STATE: InputState = {
    gamepad: {
        connected: false,
        name: '',
        leftStick: { x: 0, y: 0 },
        rightStick: { x: 0, y: 0 },
        leftTrigger: 0,
        rightTrigger: 0,
        buttons: {},
    },
    gyro: { pitch: 0, roll: 0, yaw: 0 },
    leftTouchpad: { x: 0, y: 0, touched: false, clicked: false },
    rightTouchpad: { x: 0, y: 0, touched: false, clicked: false },
};

// ─── Module-level singleton state ────────────────────────────────────

const inputStateRef = ref<InputState>(DEFAULT_INPUT_STATE);
let previousButtons: { [key: string]: boolean } = {};
const buttonListeners = new Set<ButtonEventListener>();

let initialized = false;
const unlistenFns: UnlistenFn[] = [];

async function ensureInit(): Promise<void> {
    if (initialized) return;
    initialized = true;

    unlistenFns.push(
        await listen<InputState>('input-state', event => {
            const state = event.payload;
            detectButtonChanges(state.gamepad.buttons);
            inputStateRef.value = state;
        }),
    );
}

function detectButtonChanges(currentButtons: { [key: string]: boolean }): void {
    // Pressed-edge (false → true) and released-edge (true → false) on
    // any button that appeared in either state.
    for (const [button, pressed] of Object.entries(currentButtons)) {
        const wasPressed = previousButtons[button] || false;
        if (pressed !== wasPressed) {
            emitButtonEvent({ button, pressed });
        }
    }
    // Buttons that disappeared from the state are treated as released.
    for (const [button, wasPressed] of Object.entries(previousButtons)) {
        if (wasPressed && !currentButtons[button]) {
            emitButtonEvent({ button, pressed: false });
        }
    }
    previousButtons = { ...currentButtons };
}

function emitButtonEvent(event: ButtonEvent): void {
    for (const fn of buttonListeners) {
        try { fn(event); }
        catch (err) { console.error('[useInput] button listener threw:', err); }
    }
}

/** Subscribe to button press/release edges. Returns an unsubscribe
 *  function — call it in `onBeforeUnmount` to release the listener.
 *  Components that just want the latest button state can read
 *  `gamepad().buttons` directly without subscribing.
 */
function onButtonEvent(listener: ButtonEventListener): () => void {
    buttonListeners.add(listener);
    return () => { buttonListeners.delete(listener); };
}

async function getInputState(): Promise<InputState> {
    try {
        return await invoke<InputState>('get_input_state');
    } catch {
        return DEFAULT_INPUT_STATE;
    }
}

// ─── Composable export ───────────────────────────────────────────────

export function useInput() {
    void ensureInit();

    return {
        gamepad: computed(() => inputStateRef.value.gamepad),
        gyro: computed(() => inputStateRef.value.gyro),
        leftTouchpad: computed(() => inputStateRef.value.leftTouchpad),
        rightTouchpad: computed(() => inputStateRef.value.rightTouchpad),
        isGamepadConnected: computed(() => inputStateRef.value.gamepad.connected),

        onButtonEvent,
        getInputState,
    };
}
