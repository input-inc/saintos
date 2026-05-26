/**
 * Discovery composable — Vue equivalent of the Angular
 * `DiscoveryService`. Listens for the four `discovery-*` Tauri events
 * the Rust client emits when the server responds to a discovery
 * request, and exposes the parsed catalogs (controllable nodes, role
 * definitions, ROS topic channels, WS-input slots) as reactive refs.
 *
 * Same singleton pattern as useConnection — module-scoped refs, one
 * `ensureInit()` per app, every caller gets the same reactive state.
 *
 * Triggers a refresh automatically when the connection transitions to
 * Connected. Manual refresh via `refresh()` for explicit re-pull, and
 * `refreshWsInputs()` for the bindings editor's "I want the latest
 * sheet catalog" use case (the server doesn't push on sheet edits).
 */

import { computed, ref } from 'vue';
import { listen, type UnlistenFn } from '@tauri-apps/api/event';
import { useConnection, ConnectionStatus } from './useConnection';

export interface ControllableFunction {
    function: string;
    mode: string;  // 'pwm', 'servo', 'digital_out'
    gpio: number;  // For debugging only
}

export interface ControllableNode {
    role: string;
    node_id: string;
    display_name: string;
    online: boolean;
    functions: ControllableFunction[];
}

export interface RoleDefinition {
    name: string;
    display_name: string;
    description?: string;
    pins: Record<string, unknown>;
}

/** A single scalar channel inside a ROS topic message (flattened from
 *  the message's field tree by the server's introspection pass). */
export interface TopicChannel {
    field: string;
    label: string;
    type?: string;
}

export interface TopicChannelTopic {
    topic: string;
    state_type?: string;
    channels: TopicChannel[];
}

/** A WebSocket-input slot on a routing sheet. The new bindings picker
 *  lists these (per-sheet) instead of raw ROS topic channels — a
 *  gamepad axis writes into the slot, and the server-side routing
 *  graph evaluator picks the value up and fans it onward through math
 *  nodes / ROS outputs / peripheral channels. */
export interface WsInputSlot {
    sheet_id: string;
    sheet_label: string;
    input_id: string;
    label: string;
    /** "command" = real controller target (joystick → motor). "state"
     *  = echo of a migrated state-only ROS endpoint, exists only so
     *  downstream wires can tap the value. The binding picker filters
     *  to "command" since you can't meaningfully drive a sensor
     *  reading from a controller. */
    kind: 'command' | 'state';
}

// ─── Module-level singleton state ────────────────────────────────────

const controllableRef = ref<ControllableNode[]>([]);
const rolesRef = ref<RoleDefinition[]>([]);
const topicsRef = ref<TopicChannelTopic[]>([]);
const wsInputsRef = ref<WsInputSlot[]>([]);
const loadingRef = ref<boolean>(false);
const lastFetchedRef = ref<Date | null>(null);

let initialized = false;
const unlistenFns: UnlistenFn[] = [];

async function ensureInit(): Promise<void> {
    if (initialized) return;
    initialized = true;

    unlistenFns.push(
        await listen<{ controllable: ControllableNode[] }>('discovery-controllable', event => {
            const data = event.payload;
            console.log('[useDiscovery] discovery-controllable:', data);
            if (data && data.controllable) {
                controllableRef.value = data.controllable;
                lastFetchedRef.value = new Date();
            }
        }),
    );

    unlistenFns.push(
        await listen<{ roles: RoleDefinition[] }>('discovery-roles', event => {
            const data = event.payload;
            console.log('[useDiscovery] discovery-roles:', data);
            if (data && data.roles) {
                rolesRef.value = data.roles;
            }
        }),
    );

    unlistenFns.push(
        await listen<{ topics: TopicChannelTopic[] }>('discovery-topic-channels', event => {
            const data = event.payload;
            console.log('[useDiscovery] discovery-topic-channels:', data);
            if (data && Array.isArray(data.topics)) {
                topicsRef.value = data.topics;
            }
        }),
    );

    unlistenFns.push(
        await listen<{ ws_inputs: WsInputSlot[] }>('discovery-ws-inputs', event => {
            const data = event.payload;
            console.log('[useDiscovery] discovery-ws-inputs:', data);
            if (data && Array.isArray(data.ws_inputs)) {
                wsInputsRef.value = data.ws_inputs;
            }
        }),
    );

    // Auto-refresh on connect. We listen to the same connection-status
    // event the connection composable does — Tauri delivers to every
    // registered listener so this doesn't interfere.
    unlistenFns.push(
        await listen<{ status: string }>('connection-status', event => {
            const state = event.payload;
            if (state.status === 'connected') {
                console.log('[useDiscovery] Connected → scheduling refresh in 500ms');
                setTimeout(() => {
                    void refresh();
                }, 500);
            }
        }),
    );
}

// ─── Mutations / commands ────────────────────────────────────────────

async function refresh(): Promise<void> {
    const conn = useConnection();
    if (conn.status.value !== ConnectionStatus.Connected) {
        console.log('[useDiscovery] Not connected, skipping refresh');
        return;
    }

    loadingRef.value = true;
    try {
        await conn.discoverControllable();
        await conn.discoverRoles();
        // Topic/channel catalog — kept for legacy bindings authored
        // against the old picker; new bindings should use WS inputs.
        try { await conn.discoverTopicChannels(); }
        catch (err) { console.warn('[useDiscovery] discoverTopicChannels failed:', err); }
        // WS-input slots from routing sheets — the picker for new bindings.
        try { await conn.discoverWsInputs(); }
        catch (err) { console.warn('[useDiscovery] discoverWsInputs failed:', err); }

        // Fallback if the server doesn't answer within 3s.
        setTimeout(() => {
            if (controllableRef.value.length === 0) {
                console.warn('[useDiscovery] No data from server after 3s, falling back');
                setFallbackData();
            }
        }, 3000);
    } catch (err) {
        console.error('[useDiscovery] Refresh failed:', err);
        setFallbackData();
    } finally {
        loadingRef.value = false;
    }
}

/** Re-fetch just the WS-input slot catalog. Called when the bindings
 *  editor opens so the picker reflects the current server routing
 *  graph — the server only pushes this on explicit request, not on
 *  sheet edits, so without this the dropdown shows whatever was
 *  fetched at connect time. */
async function refreshWsInputs(): Promise<void> {
    const conn = useConnection();
    if (conn.status.value !== ConnectionStatus.Connected) return;
    try {
        await conn.discoverWsInputs();
    } catch (err) {
        console.warn('[useDiscovery] refreshWsInputs failed:', err);
    }
}

function setFallbackData(): void {
    const fallback: ControllableNode[] = [
        {
            role: 'head',
            node_id: 'head-node',
            display_name: 'Head Controller',
            online: true,
            functions: [
                { function: 'pan', mode: 'servo', gpio: 0 },
                { function: 'tilt', mode: 'servo', gpio: 1 },
                { function: 'eye_left_lr', mode: 'servo', gpio: 2 },
                { function: 'eye_left_ud', mode: 'servo', gpio: 3 },
                { function: 'eye_right_lr', mode: 'servo', gpio: 4 },
                { function: 'eye_right_ud', mode: 'servo', gpio: 5 },
                { function: 'eyelid_left', mode: 'servo', gpio: 6 },
                { function: 'eyelid_right', mode: 'servo', gpio: 7 },
            ],
        },
        {
            role: 'tracks',
            node_id: 'tracks-node',
            display_name: 'Track Drive',
            online: true,
            functions: [
                { function: 'linear_velocity', mode: 'pwm', gpio: 0 },
                { function: 'angular_velocity', mode: 'pwm', gpio: 1 },
            ],
        },
        {
            role: 'sound',
            node_id: 'sound-node',
            display_name: 'Sound System',
            online: true,
            functions: [
                { function: 'play', mode: 'digital_out', gpio: 0 },
                { function: 'volume', mode: 'pwm', gpio: 1 },
            ],
        },
    ];
    controllableRef.value = fallback;
    lastFetchedRef.value = new Date();
}

// ─── Helpers (pure, no init dependency) ──────────────────────────────

function getRoleDisplayName(role: string): string {
    const roleDef = rolesRef.value.find(r => r.name === role);
    return roleDef?.display_name || role;
}

function isValidTarget(role: string, functionName: string): boolean {
    return rolesRef.value.length > 0
        && controllableRef.value.some(node =>
            node.role === role && node.functions.some(fn => fn.function === functionName));
}

function getFunctionsForRole(role: string): string[] {
    const functions = new Set<string>();
    for (const node of controllableRef.value) {
        if (node.role === role) {
            for (const fn of node.functions) functions.add(fn.function);
        }
    }
    return Array.from(functions).sort();
}

function getChannelsForTopic(topic: string): TopicChannel[] {
    return topicsRef.value.find(t => t.topic === topic)?.channels ?? [];
}

function getWsInputsForSheet(sheetId: string): WsInputSlot[] {
    return wsInputsRef.value.filter(s => s.sheet_id === sheetId);
}

// ─── Composable export ───────────────────────────────────────────────

export function useDiscovery() {
    void ensureInit();

    return {
        // Reactive state (readonly via computed wrapper)
        controllable: computed(() => controllableRef.value),
        roles: computed(() => rolesRef.value),
        topics: computed(() => topicsRef.value),
        wsInputs: computed(() => wsInputsRef.value),
        loading: computed(() => loadingRef.value),
        lastFetched: computed(() => lastFetchedRef.value),

        // Derived state
        availableTopics: computed(() =>
            topicsRef.value.map(t => t.topic).sort()),
        wsInputSheets: computed(() => {
            const byId = new Map<string, string>();
            for (const s of wsInputsRef.value) {
                byId.set(s.sheet_id, s.sheet_label || s.sheet_id);
            }
            return Array.from(byId, ([id, label]) => ({ id, label }))
                .sort((a, b) => a.label.localeCompare(b.label));
        }),
        activeRoles: computed(() => {
            const roles = new Set<string>();
            for (const node of controllableRef.value) roles.add(node.role);
            return Array.from(roles).sort();
        }),
        allFunctions: computed(() => {
            const fns = new Set<string>();
            for (const node of controllableRef.value) {
                for (const fn of node.functions) fns.add(fn.function);
            }
            return Array.from(fns).sort();
        }),

        // Commands
        refresh,
        refreshWsInputs,

        // Lookups
        getRoleDisplayName,
        isValidTarget,
        getFunctionsForRole,
        getChannelsForTopic,
        getWsInputsForSheet,
    };
}
