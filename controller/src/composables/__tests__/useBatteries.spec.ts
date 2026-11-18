// The dashboard lists the controller's own battery (a Steam Deck, or any
// Linux device the app runs on) alongside the robot's BMS packs. It's
// read from the OS through `get_local_battery`, polled only while a
// battery view is showing.
import { describe, it, expect, vi, beforeEach, afterEach } from 'vitest';
import { effectScope } from 'vue';

let local: unknown = null;
const invoke = vi.fn(async (cmd: string) => (cmd === 'get_local_battery' ? local : undefined));

vi.mock('@tauri-apps/api/core', () => ({ invoke: (cmd: string) => invoke(cmd) }));
vi.mock('@tauri-apps/api/event', () => ({ listen: vi.fn(async () => () => {}) }));

const localPolls = () => invoke.mock.calls.filter(c => c[0] === 'get_local_battery').length;

describe('the controller battery', () => {
    beforeEach(() => {
        vi.resetModules();
        vi.useFakeTimers();
        invoke.mockClear();
        local = null;
    });
    afterEach(() => vi.useRealTimers());

    async function mounted() {
        const { useBatteries } = await import('../useBatteries');
        const scope = effectScope();
        const api = scope.run(() => useBatteries())!;
        await vi.advanceTimersByTimeAsync(0);
        return { scope, batteries: api.batteries };
    }

    it('is listed first, under the device name', async () => {
        local = { device_name: 'Steam Deck', percent: 64, status: 'Discharging',
                  charging: false, voltage: 8.1, current: -1.4 };
        const { batteries } = await mounted();
        expect(batteries.value[0]).toMatchObject({
            kind: 'local', name: 'Steam Deck', soc: 64, status: 'Discharging', chargeOn: false,
        });
    });

    it('shows no card where there is no battery', async () => {
        const { batteries } = await mounted();
        expect(batteries.value).toEqual([]);
    });

    it('polls only while something is showing it', async () => {
        local = { device_name: 'Steam Deck', percent: 50, status: 'Charging',
                  charging: true, voltage: 8, current: 1 };
        const { scope } = await mounted();
        expect(localPolls()).toBe(1);
        await vi.advanceTimersByTimeAsync(15000);
        expect(localPolls()).toBe(2);
        scope.stop();   // dashboard unmounted
        await vi.advanceTimersByTimeAsync(60000);
        expect(localPolls()).toBe(2);
    });
});
