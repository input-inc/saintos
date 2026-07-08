/**
 * WiFi link-health + channel-management composable.
 *
 * Port of the dashboard's WiFi card + WifiChannelModal onto the
 * controller. Three data paths, all over the existing WS:
 *
 *  - Live signal: subscribe to `pin_state/host_controller` and sniff
 *    the `system_monitor` peripheral's wifi_* channels. The server
 *    publishes the strongest associated station's stats at 1 Hz —
 *    on a Pi AP that station is (almost always) this Deck, so the
 *    numbers ARE the operator's link quality.
 *  - Config + survey: request/response management actions
 *    (`wifi_get_config`, `wifi_survey`) → `wifi-config` /
 *    `wifi-survey` Tauri events.
 *  - Channel switch: `wifi_set_channel` ACKs first (`wifi-switching`
 *    event), then the AP restarts and the WS drops for ~5-10 s. The
 *    switching flag stays up until the reconnect completes, so the
 *    panel can show "AP restarting…" instead of a scary dead link.
 *
 * Module-scoped singleton (same pattern as useBatteries).
 */

import { computed, ref, watch } from 'vue';
import { invoke } from '@tauri-apps/api/core';
import { listen, type UnlistenFn } from '@tauri-apps/api/event';
import { useConnection } from './useConnection';

export interface WifiConfig {
    ssid: string | null;
    band: string | null;      // wire encoding: 'bg' (2.4 GHz) | 'a' (5 GHz)
    channel: number | null;
    iface: string | null;
}

export interface WifiLive {
    signalDbm: number | null;
    retryPct: number | null;   // percent 0-100 (wifi_stats.py pre-multiplies)
    noiseDbm: number | null;
    bitrateMbps: number | null;
    lastUpdate: number | null;  // Date.now() of last telemetry frame
}

export interface SurveyChannel {
    band: '2.4' | '5';
    channel: number;
    freq_mhz: number;
    ap_count: number;
    strongest_signal_dbm: number | null;
    is_dfs: boolean;
    is_current: boolean;
}

export interface SurveyState {
    loading: boolean;
    error: string | null;
    channels: SurveyChannel[];
    currentChannel: number | null;
    scannedAt: number | null;
}

export interface SwitchingState {
    active: boolean;
    detail: string | null;
}

// How long to wait for the survey response before declaring the scan
// dead. The server's iw scan takes ~10 s; leave generous headroom.
const SURVEY_TIMEOUT_MS = 30_000;

// After the switch ACK, give the server time to actually take the AP
// down (it delays ~0.4 s, then nmcli tears down) before we cycle our
// side of the connection.
const SWITCH_RECONNECT_DELAY_MS = 3_000;

// Failsafe cap on the "Restarting AP…" state — parity with the
// dashboard's WifiSwitchingOverlay. Covers DFS channels (60 s
// radar-listen) failing to come up, or reconnect never succeeding.
const SWITCH_FAILSAFE_MS = 60_000;

// ─── Module-level singleton state ────────────────────────────────────

const configRef = ref<WifiConfig>({ ssid: null, band: null, channel: null, iface: null });
const liveRef = ref<WifiLive>({
    signalDbm: null, retryPct: null, noiseDbm: null, bitrateMbps: null, lastUpdate: null,
});
const surveyRef = ref<SurveyState>({
    loading: false, error: null, channels: [], currentChannel: null, scannedAt: null,
});
const switchingRef = ref<SwitchingState>({ active: false, detail: null });

let initialized = false;
let surveyTimeout: ReturnType<typeof setTimeout> | null = null;
let switchTimers: Array<ReturnType<typeof setTimeout>> = [];
const unlistenFns: UnlistenFn[] = [];

function num(v: unknown): number | null {
    return typeof v === 'number' && isFinite(v) ? v : null;
}

function clearSwitchTimers(): void {
    for (const t of switchTimers) clearTimeout(t);
    switchTimers = [];
}

/**
 * Enter the "AP restarting" state and schedule its two exits.
 *
 * The exit can't be left to passive link detection: when the AP
 * bounces, our TCP socket dies SILENTLY — handle_connection in the
 * Rust client has no ping/keepalive, so read.next() simply never
 * yields and the client sits "Connected" on a dead socket
 * indefinitely (the bug where the panel showed "Restarting WiFi AP…"
 * forever). So:
 *
 *  1. After SWITCH_RECONNECT_DELAY_MS we force a reconnect cycle
 *     (disconnect + autoConnect). The Rust loop retries with backoff
 *     until the AP is back; the resulting isConnected transition
 *     clears the flag (see the watch in ensureInit).
 *  2. SWITCH_FAILSAFE_MS caps the state outright, mirroring the web
 *     overlay's 60 s timeout.
 */
function beginSwitching(detail: string | null): void {
    switchingRef.value = { active: true, detail };
    clearSwitchTimers();
    switchTimers.push(setTimeout(() => {
        if (!switchingRef.value.active) return;
        const conn = useConnection();
        void conn.disconnect()
            .then(() => conn.autoConnect())
            .catch(err => console.error('[useWifi] reconnect cycle failed:', err));
    }, SWITCH_RECONNECT_DELAY_MS));
    switchTimers.push(setTimeout(() => {
        switchingRef.value = { active: false, detail: null };
    }, SWITCH_FAILSAFE_MS));
}

async function ensureInit(): Promise<void> {
    if (initialized) return;
    initialized = true;

    // Live telemetry — reuse the pin-state pipeline the battery panel
    // set up; we just watch a different pseudo-node.
    unlistenFns.push(
        await listen<{ node: string; data: { channels?: Array<{ peripheral_id: string; channel_id: string; value: number | null }> } | null }>(
            'pin-state', event => {
                const { node, data } = event.payload;
                if (node !== 'pin_state/host_controller') return;
                const channels = data?.channels;
                if (!Array.isArray(channels)) return;
                const pick = (id: string): number | null => {
                    const c = channels.find(c =>
                        c.peripheral_id === 'system_monitor' && c.channel_id === id);
                    return num(c?.value);
                };
                liveRef.value = {
                    signalDbm: pick('wifi_signal'),
                    retryPct: pick('wifi_retry_pct'),
                    noiseDbm: pick('wifi_noise'),
                    bitrateMbps: pick('wifi_bitrate'),
                    lastUpdate: Date.now(),
                };
            }),
    );

    unlistenFns.push(
        await listen<{ ssid?: string; band?: string; channel?: number; iface?: string }>(
            'wifi-config', event => {
                const p = event.payload || {};
                configRef.value = {
                    ssid: typeof p.ssid === 'string' ? p.ssid : null,
                    band: typeof p.band === 'string' ? p.band : null,
                    channel: num(p.channel) || null,
                    iface: typeof p.iface === 'string' ? p.iface : null,
                };
            }),
    );

    unlistenFns.push(
        await listen<{ ok?: boolean; error?: string; current_channel?: number; channels?: SurveyChannel[] }>(
            'wifi-survey', event => {
                if (surveyTimeout) { clearTimeout(surveyTimeout); surveyTimeout = null; }
                const p = event.payload || {};
                const failed = p.ok === false;
                surveyRef.value = {
                    loading: false,
                    error: failed ? (p.error || 'Survey failed') : null,
                    channels: !failed && Array.isArray(p.channels) ? p.channels : [],
                    currentChannel: num(p.current_channel),
                    scannedAt: failed ? surveyRef.value.scannedAt : Date.now(),
                };
            }),
    );

    unlistenFns.push(
        await listen<{ band?: string; channel?: number; ssid?: string }>(
            'wifi-switching', event => {
                const p = event.payload || {};
                const detail = p.channel != null
                    ? `channel ${p.channel} (${p.band === 'a' ? '5' : '2.4'} GHz)`
                    : (p.ssid ? `SSID ${p.ssid}` : null);
                beginSwitching(detail);
            }),
    );

    const conn = useConnection();
    watch(conn.isConnected, (connected) => {
        if (connected) {
            // Reconnected — if we were mid channel-switch, the AP is
            // back: drop the overlay, cancel its timers, and re-read
            // the (new) config.
            clearSwitchTimers();
            switchingRef.value = { active: false, detail: null };
            void invoke('get_wifi_config').catch(err =>
                console.error('[useWifi] get_wifi_config failed:', err));
            void invoke('subscribe_topics', { topics: ['pin_state/host_controller'] })
                .catch(err => console.error('[useWifi] subscribe_topics failed:', err));
        } else {
            // Link down: live numbers are stale by definition. Keep the
            // last config (SSID/channel don't change on their own) and
            // keep any switching flag — this drop is the expected one.
            liveRef.value = {
                signalDbm: null, retryPct: null, noiseDbm: null,
                bitrateMbps: null, lastUpdate: null,
            };
        }
    }, { immediate: true });
}

/** Kick off a channel survey (~10 s server-side). */
async function runSurvey(): Promise<void> {
    if (surveyRef.value.loading) return;
    surveyRef.value = { ...surveyRef.value, loading: true, error: null };
    try {
        await invoke('wifi_survey');
    } catch (err) {
        surveyRef.value = { ...surveyRef.value, loading: false, error: String(err) };
        return;
    }
    surveyTimeout = setTimeout(() => {
        if (surveyRef.value.loading) {
            surveyRef.value = { ...surveyRef.value, loading: false, error: 'Scan timed out' };
        }
    }, SURVEY_TIMEOUT_MS);
}

/**
 * Switch the AP to a surveyed channel. Expect the connection to drop
 * ~5-10 s right after; the switching flag covers the gap.
 */
async function applyChannel(ch: SurveyChannel): Promise<void> {
    const wireBand = ch.band === '5' ? 'a' : 'bg';
    // Optimistic — the ACK (`wifi-switching`) usually lands, but if the
    // AP bounces before it does, the panel should already be covered.
    beginSwitching(`channel ${ch.channel} (${ch.band} GHz)`);
    try {
        await invoke('set_wifi_channel', { band: wireBand, channel: ch.channel });
    } catch (err) {
        clearSwitchTimers();
        switchingRef.value = { active: false, detail: null };
        throw err;
    }
}

// Mirror the dashboard's recommendation: fewest APs among non-current
// channels, non-DFS preferred on a tie, then quieter strongest-neighbor.
const bestPick = computed<SurveyChannel | null>(() => {
    const candidates = surveyRef.value.channels.filter(c => !c.is_current);
    if (!candidates.length) return null;
    const sorted = [...candidates].sort((a, b) =>
        (a.ap_count - b.ap_count)
        || (Number(a.is_dfs) - Number(b.is_dfs))
        || ((a.strongest_signal_dbm ?? -Infinity) - (b.strongest_signal_dbm ?? -Infinity)));
    return sorted[0];
});

export function useWifi() {
    void ensureInit();
    return {
        config: computed(() => configRef.value),
        live: computed(() => liveRef.value),
        survey: computed(() => surveyRef.value),
        switching: computed(() => switchingRef.value),
        bestPick,
        runSurvey,
        applyChannel,
    };
}
