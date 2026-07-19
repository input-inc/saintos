/**
 * Connection composable — Vue equivalent of the Angular
 * `ConnectionService`. Wraps the Rust-side WebSocket client +
 * authentication + the e-stop mirror.
 *
 * Module-scoped state makes `useConnection()` behave like Angular's
 * `providedIn: 'root'` — every caller gets the same reactive refs.
 * The `ensureInit()` pattern guards the Tauri event listeners so they
 * only register once regardless of how many components import this.
 *
 * The Rust side (`controller/src-tauri/src/protocol/`) is unchanged
 * from the Angular era. `invoke('connect', …)` and the
 * `connection-status` / `estop-state` events have the same shapes.
 */

import { computed, ref } from 'vue';
import { invoke } from '@tauri-apps/api/core';
import { listen, type UnlistenFn } from '@tauri-apps/api/event';

export enum ConnectionStatus {
    Disconnected = 'disconnected',
    Connecting = 'connecting',
    Authenticating = 'authenticating',
    Connected = 'connected',
    Error = 'error',
}

export interface ConnectionConfig {
    host: string;
    port: number;
    password: string;
}

export interface ConnectionState {
    status: ConnectionStatus;
    error?: string;
    lastConnected?: Date;
}

const CONFIG_KEY = 'saint-controller-config';

// ─── Module-level singleton state ────────────────────────────────────
//
// These refs are shared across every `useConnection()` call so multiple
// components see the same connection state. Equivalent to Angular's
// `@Injectable({ providedIn: 'root' })`.

const statusRef = ref<ConnectionStatus>(ConnectionStatus.Disconnected);
const errorRef = ref<string | undefined>(undefined);
/** Mirror of the server's system-wide e-stop latch. Driven by the
 *  `estop-state` event from Rust. Read by the app shell for the
 *  banner; the Rust side also gates outgoing streaming control by
 *  this same value so this ref is purely for visual feedback. */
const estopActiveRef = ref<boolean>(false);

/** Last measured WebSocket round-trip to the server, in ms (null until
 *  the first probe returns). Driven by the `connection-latency` event the
 *  Rust client emits every ~2s. Shown in the connection dropdown. */
const pingMsRef = ref<number | null>(null);

let initialized = false;
const unlistenFns: UnlistenFn[] = [];

async function ensureInit(): Promise<void> {
    if (initialized) return;
    initialized = true;

    unlistenFns.push(
        await listen<ConnectionState>('connection-status', event => {
            const state = event.payload;
            console.log('[useConnection] Connection status changed:', state.status, state.error || '');
            statusRef.value = state.status;
            errorRef.value = state.error;
            // Clear the local estop mirror on disconnect so the
            // banner doesn't linger after the WS drops. The
            // bootstrap on the next connect re-sets it from the
            // authoritative server-side value.
            if (state.status === ConnectionStatus.Disconnected) {
                estopActiveRef.value = false;
                pingMsRef.value = null;
            }
        }),
    );

    // Round-trip latency probe results from the Rust WS client (~2s cadence).
    unlistenFns.push(
        await listen<number>('connection-latency', event => {
            pingMsRef.value = event.payload;
        }),
    );

    // estop-state is emitted by the Rust side on both the
    // get_estop_state bootstrap response and the 'estop' topic
    // broadcast.
    unlistenFns.push(
        await listen<{ active: boolean }>('estop-state', event => {
            const active = !!event.payload.active;
            console.log('[useConnection] E-Stop state:', active ? 'ENGAGED' : 'released');
            estopActiveRef.value = active;
        }),
    );

    // Auto-connect on app boot if credentials are saved. The short
    // delay lets the Tauri JS bridge finish wiring up before we
    // try to invoke; without it, the first invoke can race the
    // bridge's ready signal on slower hosts.
    setTimeout(() => {
        void autoConnect();
    }, 500);
}

// ─── Pure helpers (don't depend on init) ─────────────────────────────

function getSavedConfig(): ConnectionConfig | null {
    const saved = localStorage.getItem(CONFIG_KEY);
    if (!saved) return null;
    try {
        const parsed = JSON.parse(saved);
        if (parsed.host && parsed.port) {
            return {
                host: parsed.host,
                port: parsed.port,
                password: parsed.password || '',
            };
        }
    } catch {
        // Ignore parse errors
    }
    return null;
}

async function autoConnect(): Promise<void> {
    const config = getSavedConfig();
    if (!config || !config.password) {
        console.log('[useConnection] No saved credentials for auto-connect');
        return;
    }
    console.log('[useConnection] Attempting auto-connect to', config.host);
    try {
        await connect(config);
    } catch (err) {
        console.error('[useConnection] Auto-connect failed:', err);
        // Auto-connect failure is non-fatal.
    }
}

async function connect(config: ConnectionConfig): Promise<void> {
    statusRef.value = ConnectionStatus.Connecting;
    errorRef.value = undefined;
    try {
        await invoke('connect', {
            host: config.host,
            port: config.port,
            password: config.password,
        });
    } catch (err) {
        statusRef.value = ConnectionStatus.Error;
        errorRef.value = String(err);
        throw err;
    }
}

async function disconnect(): Promise<void> {
    try {
        await invoke('disconnect');
        statusRef.value = ConnectionStatus.Disconnected;
        errorRef.value = undefined;
        pingMsRef.value = null;
    } catch (err) {
        console.error('[useConnection] Disconnect error:', err);
    }
}

/** Drop the current connection and immediately reconnect with the saved
 *  config — the "Reconnect" action in the connection dropdown. */
async function reconnect(): Promise<void> {
    await disconnect();
    const config = getSavedConfig();
    if (!config || !config.password) {
        console.warn('[useConnection] reconnect: no saved credentials');
        return;
    }
    await connect(config);
}

// ─── Server-side command invocations ─────────────────────────────────

/** Legacy command using node_id + pin_id. Deprecated; prefer the
 *  routing-graph WS-input or topic-channel paths. */
async function sendCommand(nodeId: string, pinId: number, value: unknown): Promise<void> {
    if (statusRef.value !== ConnectionStatus.Connected) {
        throw new Error('Not connected');
    }
    await invoke('send_command', { nodeId, pinId, value });
}

/** Push a single scalar onto a ROS topic channel. The server merges
 *  the field into a per-topic buffer and republishes. */
async function sendTopicChannelValue(topic: string, channel: string, value: unknown): Promise<void> {
    if (statusRef.value !== ConnectionStatus.Connected) {
        console.warn('[useConnection] sendTopicChannelValue called but not connected');
        throw new Error('Not connected');
    }
    try {
        await invoke('send_topic_channel_value', { topic, channel, value });
    } catch (err) {
        console.error('[useConnection] sendTopicChannelValue error:', err);
        throw err;
    }
}

async function discoverRoles(): Promise<void> {
    if (statusRef.value !== ConnectionStatus.Connected) {
        throw new Error('Not connected');
    }
    await invoke('discover_roles');
}

/** Response arrives on the `discovery-topic-channels` Tauri event. */
async function discoverTopicChannels(): Promise<void> {
    if (statusRef.value !== ConnectionStatus.Connected) {
        throw new Error('Not connected');
    }
    await invoke('discover_topic_channels');
}

/** Response arrives on the `discovery-ws-inputs` Tauri event. */
async function discoverWsInputs(): Promise<void> {
    if (statusRef.value !== ConnectionStatus.Connected) {
        throw new Error('Not connected');
    }
    await invoke('discover_ws_inputs');
}

async function discoverControllable(): Promise<void> {
    if (statusRef.value !== ConnectionStatus.Connected) {
        throw new Error('Not connected');
    }
    await invoke('discover_controllable');
}

async function emergencyStop(): Promise<void> {
    await invoke('emergency_stop');
}

/** Play a saved animation on the server by id. */
async function startAnimation(id: string): Promise<void> {
    await invoke('start_animation', { id });
}

/** Stop a currently-playing animation on the server by id. */
async function stopAnimation(id: string): Promise<void> {
    await invoke('stop_animation', { id });
}

/** Apply a saved pose on the server by id. */
async function applyPose(id: string): Promise<void> {
    await invoke('apply_pose', { id });
}

/** Play a saved soundboard sound on the server by id. */
async function playSound(id: string): Promise<void> {
    await invoke('play_sound', { id });
}

// ─── Composable export ───────────────────────────────────────────────

export function useConnection() {
    // Fire-and-forget init. Returns a Promise but we don't await — the
    // refs start at their default values and update as the events
    // arrive. Components don't need to know about the lifecycle.
    void ensureInit();

    return {
        // Reactive state
        status: computed(() => statusRef.value),
        error: computed(() => errorRef.value),
        isConnected: computed(() => statusRef.value === ConnectionStatus.Connected),
        estopActive: computed(() => estopActiveRef.value),
        pingMs: computed(() => pingMsRef.value),

        // Connection lifecycle
        connect,
        disconnect,
        reconnect,
        autoConnect,
        getSavedConfig,

        // Commands
        sendCommand,
        sendTopicChannelValue,
        emergencyStop,
        startAnimation,
        stopAnimation,
        applyPose,
        playSound,

        // Discovery requests (responses arrive via Tauri events
        // consumed by useDiscovery).
        discoverRoles,
        discoverTopicChannels,
        discoverWsInputs,
        discoverControllable,
    };
}
