/**
 * Battery telemetry composable.
 *
 * Mirrors the dashboard's BMSMonitor.vue, but the controller has no
 * saved routes telling it which peripheral feeds the widget. So instead
 * of route lookups we go topic-only: subscribe to `pin_state/<node>`
 * for every adopted node and sniff the frame for BMS channels. Any
 * peripheral whose channels include `soc`/`pack_voltage` is treated as a
 * battery pack and surfaced with the same field set BMSMonitor shows
 * (SOC, voltage, current, temp, protection bits, FET state).
 *
 * Data path (all over the existing WS):
 *   connect → invoke('get_adopted_nodes')
 *           → 'adopted-nodes' event → invoke('subscribe_topics', pin_state/<n>)
 *           → 'pin-state' events stream frames → grouped into packs.
 *
 * Module-scoped singleton (same pattern as useConnection) so every
 * caller shares one subscription and one reactive pack list.
 */

import { computed, ref, watch } from 'vue';
import { invoke } from '@tauri-apps/api/core';
import { listen, type UnlistenFn } from '@tauri-apps/api/event';
import { useConnection } from './useConnection';
import {
    createTelemetrySubscription,
    holdWhileInScope,
} from './useTelemetrySubscription';

// Channel ids emitted by the pathfinder_bms / jbd_bms firmware drivers.
// Presence of any STRONG id marks a peripheral as a battery pack.
const STRONG_BMS_CHANNELS = ['soc', 'pack_voltage'];

// JBD protection bit → label. Kept in sync with BMSMonitor.vue and the
// server-side jbd_bms.py decoder.
const PROTECTION_LABELS = [
    'cell overvoltage', 'cell undervoltage', 'pack overvoltage', 'pack undervoltage',
    'charge overtemp', 'charge undertemp', 'discharge overtemp', 'discharge undertemp',
    'charge overcurrent', 'discharge overcurrent', 'short circuit',
    'front-end IC error', 'MOS software lock',
];

export interface BmsPack {
    key: string;            // `${nodeId}/${peripheralId}` — stable v-for key
    nodeId: string;
    peripheralId: string;
    name: string;
    soc: number | null;
    voltage: number | null;
    current: number | null;
    temp: number | null;
    protection: number;
    fetStatus: number;
    chargeOn: boolean;
    dischargeOn: boolean;
    faults: string[];
}

interface Channel {
    peripheral_id: string;
    channel_id: string;
    value: number | null;
}

interface PinStateEvent {
    node: string;                 // "pin_state/<nodeId>"
    data: { channels?: Channel[] } | null;
}

interface AdoptedNodesEvent {
    nodes?: Array<{ node_id: string }>;
}

// ─── Module-level singleton state ────────────────────────────────────

// nodeId → latest channels array from that node's pin_state frame.
const framesRef = ref<Record<string, Channel[]>>({});

let initialized = false;
const unlistenFns: UnlistenFn[] = [];

// Ref-counted so the pin_state stream stops when no component is
// rendering batteries. Topics are whatever the adopted-node list turns
// out to be, so the subscription tracks them for us — see
// useTelemetrySubscription.
const subscription = createTelemetrySubscription('useBatteries', () => {
    void invoke('get_adopted_nodes').catch(err =>
        console.error('[useBatteries] get_adopted_nodes failed:', err));
});

function decodeProtection(bits: number): string[] {
    const b = bits | 0;
    const out: string[] = [];
    for (let i = 0; i < 16; i++) {
        if (b & (1 << i)) out.push(PROTECTION_LABELS[i] || `fault bit ${i}`);
    }
    return out;
}

function chanVal(channels: Channel[], peripheralId: string, channelId: string): number | null {
    for (const c of channels) {
        if (c.peripheral_id === peripheralId && c.channel_id === channelId
            && typeof c.value === 'number') {
            return c.value;
        }
    }
    return null;
}

async function ensureInit(): Promise<void> {
    if (initialized) return;
    initialized = true;

    unlistenFns.push(
        await listen<AdoptedNodesEvent>('adopted-nodes', event => {
            const nodes = event.payload?.nodes || [];
            const topics = nodes
                .map(n => n.node_id)
                .filter(Boolean)
                .map(id => `pin_state/${id}`);
            // add() is a no-op while nothing holds the subscription, so a
            // response that lands after the dashboard unmounted can't
            // re-open the stream behind our back.
            subscription.add(topics);
        }),
    );

    unlistenFns.push(
        await listen<PinStateEvent>('pin-state', event => {
            const { node, data } = event.payload;
            if (!node?.startsWith('pin_state/')) return;
            const nodeId = node.slice('pin_state/'.length);
            const channels = data?.channels;
            if (Array.isArray(channels)) {
                framesRef.value = { ...framesRef.value, [nodeId]: channels };
            }
        }),
    );

    // Re-discover battery sources whenever the link comes up: server-side
    // subscriptions live on the per-connection client object, so a
    // reconnect starts from nothing. Both sides are no-ops unless a
    // consumer is actually holding the subscription.
    const conn = useConnection();
    watch(conn.isConnected, (connected) => {
        if (connected) {
            subscription.refresh();
        } else {
            // The server already dropped our subscriptions with the
            // connection; forget the tracked set so the reconnect
            // re-subscribes cleanly rather than treating them as live.
            subscription.forget();
            framesRef.value = {};
        }
    }, { immediate: true });
}

// Build one pack per (node, peripheral) that looks like a BMS. Mirrors
// ConsoleBatteriesOverview's grouping, but discovery is purely from the
// channel ids present in the frame — no peripheral-type lookup.
const batteries = computed<BmsPack[]>(() => {
    const out: BmsPack[] = [];
    for (const [nodeId, channels] of Object.entries(framesRef.value)) {
        const bmsPeripherals = new Set<string>();
        for (const c of channels) {
            if (STRONG_BMS_CHANNELS.includes(c.channel_id)) bmsPeripherals.add(c.peripheral_id);
        }
        for (const pid of bmsPeripherals) {
            const protection = (chanVal(channels, pid, 'protection') ?? 0) | 0;
            const fetStatus = (chanVal(channels, pid, 'fet_status') ?? 0) | 0;
            out.push({
                key: `${nodeId}/${pid}`,
                nodeId,
                peripheralId: pid,
                name: pid,
                soc: chanVal(channels, pid, 'soc'),
                voltage: chanVal(channels, pid, 'pack_voltage'),
                current: chanVal(channels, pid, 'current'),
                temp: chanVal(channels, pid, 'temp_1'),
                protection,
                fetStatus,
                chargeOn: (fetStatus & 0x01) === 0x01,
                dischargeOn: (fetStatus & 0x02) === 0x02,
                faults: decodeProtection(protection),
            });
        }
    }
    // Stable order so cards don't reshuffle as frames arrive.
    return out.sort((a, b) => a.key.localeCompare(b.key));
});

export function useBatteries() {
    void ensureInit();
    // Hold the pin_state stream only while the calling component lives.
    holdWhileInScope('useBatteries', subscription);
    return { batteries };
}
