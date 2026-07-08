/**
 * useWifi — simulated end-to-end tests for the channel-switch flow.
 *
 * The AP restart is the tricky part: the server ACKs, then the WS
 * drops SILENTLY (no ping/keepalive in the Rust client, so a dead
 * socket looks "Connected" forever). The composable therefore owns
 * its own exits from the "Restarting AP…" state:
 *   1. a forced reconnect cycle (disconnect + autoConnect) 3 s after
 *      the switch begins — the regression that shipped without this
 *      left the panel wedged on "Restarting WiFi AP…";
 *   2. a 60 s failsafe (parity with the dashboard overlay).
 *
 * Tauri IPC and useConnection are mocked; timers are simulated.
 */
import { describe, it, expect, vi, beforeEach, afterEach } from 'vitest';
import { computed, nextTick } from 'vue';
import type { SurveyChannel } from '../useWifi';

const h = vi.hoisted(() => {
    const { ref } = require('vue') as typeof import('vue');
    return {
        listeners: new Map<string, (event: { payload: unknown }) => void>(),
        invokeMock: vi.fn(async (_cmd: string, _args?: unknown) => undefined),
        connected: ref(false),
        disconnectMock: vi.fn(async () => undefined),
        autoConnectMock: vi.fn(async () => undefined),
    };
});

vi.mock('@tauri-apps/api/core', () => ({
    invoke: (cmd: string, args?: unknown) => h.invokeMock(cmd, args),
}));

vi.mock('@tauri-apps/api/event', () => ({
    listen: async (name: string, cb: (event: { payload: unknown }) => void) => {
        h.listeners.set(name, cb);
        return () => h.listeners.delete(name);
    },
}));

vi.mock('../useConnection', () => ({
    useConnection: () => ({
        isConnected: computed(() => h.connected.value),
        disconnect: h.disconnectMock,
        autoConnect: h.autoConnectMock,
    }),
}));

function fire(event: string, payload: unknown): void {
    const cb = h.listeners.get(event);
    if (!cb) throw new Error(`no listener registered for '${event}'`);
    cb({ payload });
}

/** Fresh module instance per test — useWifi is a module singleton. */
async function freshWifi() {
    vi.resetModules();
    const mod = await import('../useWifi');
    const wifi = mod.useWifi();
    // Drain ensureInit's listener registration + the immediate watch.
    for (let i = 0; i < 10; i++) await Promise.resolve();
    await nextTick();
    return wifi;
}

function surveyChannel(over: Partial<SurveyChannel> = {}): SurveyChannel {
    return {
        band: '2.4', channel: 6, freq_mhz: 2437, ap_count: 3,
        strongest_signal_dbm: -60, is_dfs: false, is_current: false,
        ...over,
    };
}

async function simulateReconnect(): Promise<void> {
    h.connected.value = false;
    await nextTick();
    h.connected.value = true;
    await nextTick();
}

describe('useWifi channel-switch flow', () => {
    beforeEach(() => {
        vi.useFakeTimers();
        h.listeners.clear();
        h.invokeMock.mockClear();
        h.invokeMock.mockResolvedValue(undefined);
        h.disconnectMock.mockClear();
        h.autoConnectMock.mockClear();
        h.connected.value = false;
    });

    afterEach(() => {
        vi.useRealTimers();
    });

    it('switch ACK enters the restarting state with a readable detail', async () => {
        const wifi = await freshWifi();
        fire('wifi-switching', { band: 'bg', channel: 6 });
        expect(wifi.switching.value.active).toBe(true);
        expect(wifi.switching.value.detail).toContain('channel 6 (2.4 GHz)');
    });

    it('forces a reconnect cycle 3 s after the switch begins', async () => {
        await freshWifi();
        fire('wifi-switching', { band: 'a', channel: 36 });
        expect(h.disconnectMock).not.toHaveBeenCalled();
        await vi.advanceTimersByTimeAsync(3_000);
        expect(h.disconnectMock).toHaveBeenCalledTimes(1);
        expect(h.autoConnectMock).toHaveBeenCalledTimes(1);
    });

    it('reconnect clears the restarting state and refetches config', async () => {
        const wifi = await freshWifi();
        fire('wifi-switching', { band: 'bg', channel: 11 });
        await vi.advanceTimersByTimeAsync(3_000);
        h.invokeMock.mockClear();

        await simulateReconnect();

        expect(wifi.switching.value.active).toBe(false);
        expect(h.invokeMock).toHaveBeenCalledWith('get_wifi_config', undefined);
        expect(h.invokeMock).toHaveBeenCalledWith('subscribe_topics',
            { topics: ['pin_state/host_controller'] });

        // The cancelled failsafe must not resurrect or re-cycle anything.
        h.disconnectMock.mockClear();
        await vi.advanceTimersByTimeAsync(120_000);
        expect(wifi.switching.value.active).toBe(false);
        expect(h.disconnectMock).not.toHaveBeenCalled();
    });

    it('failsafe clears the restarting state after 60 s if the link never returns', async () => {
        const wifi = await freshWifi();
        fire('wifi-switching', { band: 'bg', channel: 1 });
        await vi.advanceTimersByTimeAsync(59_000);
        expect(wifi.switching.value.active).toBe(true);
        await vi.advanceTimersByTimeAsync(1_100);
        expect(wifi.switching.value.active).toBe(false);
    });

    it('applyChannel translates UI band to the wire encoding', async () => {
        const wifi = await freshWifi();
        await wifi.applyChannel(surveyChannel({ band: '5', channel: 36 }));
        expect(h.invokeMock).toHaveBeenCalledWith('set_wifi_channel',
            { band: 'a', channel: 36 });
        expect(wifi.switching.value.active).toBe(true);

        await wifi.applyChannel(surveyChannel({ band: '2.4', channel: 6 }));
        expect(h.invokeMock).toHaveBeenCalledWith('set_wifi_channel',
            { band: 'bg', channel: 6 });
    });

    it('a failed switch command clears the state and cancels the cycle', async () => {
        const wifi = await freshWifi();
        h.invokeMock.mockImplementation(async (cmd: string) => {
            if (cmd === 'set_wifi_channel') throw new Error('Not connected');
        });
        await expect(wifi.applyChannel(surveyChannel())).rejects.toThrow();
        expect(wifi.switching.value.active).toBe(false);
        await vi.advanceTimersByTimeAsync(10_000);
        expect(h.disconnectMock).not.toHaveBeenCalled();
    });
});

describe('useWifi survey + telemetry', () => {
    beforeEach(() => {
        vi.useFakeTimers();
        h.listeners.clear();
        h.invokeMock.mockClear();
        h.invokeMock.mockResolvedValue(undefined);
        h.connected.value = false;
    });

    afterEach(() => {
        vi.useRealTimers();
    });

    it('survey lifecycle: loading → results', async () => {
        const wifi = await freshWifi();
        const p = wifi.runSurvey();
        expect(h.invokeMock).toHaveBeenCalledWith('wifi_survey', undefined);
        await p;
        expect(wifi.survey.value.loading).toBe(true);

        fire('wifi-survey', {
            ok: true, iface: 'wlan0', current_channel: 6, error: null,
            channels: [surveyChannel({ is_current: true }), surveyChannel({ channel: 11, ap_count: 0 })],
        });
        expect(wifi.survey.value.loading).toBe(false);
        expect(wifi.survey.value.error).toBeNull();
        expect(wifi.survey.value.channels).toHaveLength(2);
        expect(wifi.survey.value.currentChannel).toBe(6);
    });

    it('survey timeout surfaces an error instead of spinning forever', async () => {
        const wifi = await freshWifi();
        await wifi.runSurvey();
        await vi.advanceTimersByTimeAsync(30_100);
        expect(wifi.survey.value.loading).toBe(false);
        expect(wifi.survey.value.error).toBe('Scan timed out');
    });

    it('a failed survey reports the server error', async () => {
        const wifi = await freshWifi();
        await wifi.runSurvey();
        fire('wifi-survey', { ok: false, error: 'iw scan failed', current_channel: null, channels: [] });
        expect(wifi.survey.value.loading).toBe(false);
        expect(wifi.survey.value.error).toBe('iw scan failed');
    });

    it('bestPick prefers fewest APs, non-DFS on ties, never the current channel', async () => {
        const wifi = await freshWifi();
        fire('wifi-survey', {
            ok: true, current_channel: 6, error: null,
            channels: [
                surveyChannel({ channel: 6, ap_count: 0, is_current: true }),
                surveyChannel({ band: '5', channel: 52, ap_count: 1, is_dfs: true }),
                surveyChannel({ band: '5', channel: 36, ap_count: 1, is_dfs: false }),
                surveyChannel({ channel: 11, ap_count: 4 }),
            ],
        });
        expect(wifi.bestPick.value?.channel).toBe(36);
    });

    it('parses live telemetry from host_controller frames only', async () => {
        const wifi = await freshWifi();
        fire('pin-state', {
            node: 'pin_state/some_other_node',
            data: { channels: [{ peripheral_id: 'system_monitor', channel_id: 'wifi_signal', value: -40 }] },
        });
        expect(wifi.live.value.signalDbm).toBeNull();

        fire('pin-state', {
            node: 'pin_state/host_controller',
            data: {
                channels: [
                    { peripheral_id: 'system_monitor', channel_id: 'wifi_signal', value: -56 },
                    { peripheral_id: 'system_monitor', channel_id: 'wifi_retry_pct', value: 5.8 },
                    { peripheral_id: 'system_monitor', channel_id: 'wifi_noise', value: -95 },
                    { peripheral_id: 'system_monitor', channel_id: 'wifi_bitrate', value: 144.4 },
                ],
            },
        });
        expect(wifi.live.value.signalDbm).toBe(-56);
        expect(wifi.live.value.retryPct).toBeCloseTo(5.8);
        expect(wifi.live.value.noiseDbm).toBe(-95);
        expect(wifi.live.value.bitrateMbps).toBeCloseTo(144.4);
    });
});
