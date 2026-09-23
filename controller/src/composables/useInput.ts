/**
 * Input composable — Vue equivalent of the Angular `InputService`.
 * Listens for `input-state` Tauri events and exposes the latest
 * gamepad / gyro / touchpad state as reactive refs.
 *
 * The Rust side publishes these at a UI cadence (UI_EMIT_MS in lib.rs,
 * ~60 Hz) and only when the state actually changed — NOT at the input
 * sample rate. So an idle controller delivers roughly one event per
 * second, and a moving one at most ~60. Don't treat event arrival as a
 * clock; read the refs.
 *
 * Also publishes button press/release edges through `onButtonEvent` —
 * components subscribe with a callback that fires once per state
 * change. This replaces the Angular RxJS `buttonEvents$` observable;
 * the API is simpler (callback-per-listener instead of an Observable
 * + Subscription) which fits Vue's lifecycle hooks.
 */

import { computed, shallowRef } from 'vue';
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

// shallowRef, not ref: this is only ever REPLACED wholesale (the Tauri
// event hands over a freshly deserialized object) and every consumer
// reads it. Deep `ref` would walk the payload and wrap gamepad, the
// button map, gyro and both touchpads in reactive proxies on every
// single event — allocation and proxy setup buying reactivity that
// nothing uses, since no code mutates a nested field in place.
//
// It also stops Vue making DEFAULT_INPUT_STATE reactive: that's a
// module-level shared const, and deep reactivity on it is a footgun
// waiting for the first person who writes through it.
const inputStateRef = shallowRef<InputState>(DEFAULT_INPUT_STATE);
let previousButtons: { [key: string]: boolean } = {};
const buttonListeners = new Set<ButtonEventListener>();

let initialized = false;
let sawFirstEvent = false;
const unlistenFns: UnlistenFn[] = [];

async function ensureInit(): Promise<void> {
    if (initialized) return;
    initialized = true;

    unlistenFns.push(
        await listen<InputState>('input-state', event => {
            const state = event.payload;
            sawFirstEvent = true;
            detectButtonChanges(state.gamepad.buttons);
            inputStateRef.value = state;
        }),
    );

    // The Rust emitter is change-gated — an untouched controller
    // publishes only on a 1 s idle heartbeat. Without a bootstrap read
    // the UI would show DEFAULT_INPUT_STATE ("No gamepad detected")
    // until that heartbeat lands. Pull the current state once, and drop
    // it if a live event beat us to it so we can't overwrite fresher
    // data with the value we asked for before the listener attached.
    const initial = await getInputState();
    if (!sawFirstEvent) {
        detectButtonChanges(initial.gamepad.buttons);
        inputStateRef.value = initial;
    }
}

function detectButtonChanges(currentButtons: { [key: string]: boolean }): void {
    // Pressed-edge (false → true) and released-edge (true → false) on
    // any button that appeared in either state.
    //
    // `for...in` rather than Object.entries: entries() allocates an array
    // plus a two-element array per key, and this runs over ~24 buttons on
    // every input event. for...in walks the same keys and allocates
    // nothing.
    for (const button in currentButtons) {
        const pressed = currentButtons[button];
        const wasPressed = previousButtons[button] || false;
        if (pressed !== wasPressed) {
            emitButtonEvent({ button, pressed });
        }
    }
    // Buttons that disappeared entirely from the state are treated as
    // released. Guard on absence (`!(button in …)`), NOT falsiness — a
    // button present as `false` was already handled by the transition
    // loop above. The firmware emits the full button map every frame, so
    // `!currentButtons[button]` here would re-fire every release a second
    // time.
    for (const button in previousButtons) {
        if (previousButtons[button] && !(button in currentButtons)) {
            emitButtonEvent({ button, pressed: false });
        }
    }
    // Keep the reference, don't copy it. `currentButtons` came out of a
    // freshly deserialized event payload and nothing mutates it, so a
    // spread would allocate a duplicate of a map that is already
    // immutable in practice. If a caller ever starts writing into the
    // input state in place, this has to go back to a copy — which is the
    // same assumption shallowRef above relies on.
    previousButtons = currentButtons;
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
