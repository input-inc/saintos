// The header's connection icon follows `connection-status` events from
// the Rust client. Events alone could leave it wrong: across a reconnect
// a retiring connection loop reported `disconnected` after the new one
// reported `connected`, and the icon stayed red over a working link.
// The composable now re-reads the real status every couple of seconds.
import { describe, it, expect, vi, beforeEach, afterEach } from 'vitest';

const listeners: Record<string, (e: { payload: unknown }) => void> = {};
let rustState: { status: string; error?: string } = { status: 'disconnected' };

vi.mock('@tauri-apps/api/event', () => ({
    listen: vi.fn(async (name: string, cb: (e: { payload: unknown }) => void) => {
        listeners[name] = cb;
        return () => {};
    }),
}));
vi.mock('@tauri-apps/api/core', () => ({
    invoke: vi.fn(async (cmd: string) => {
        if (cmd === 'get_connection_status') return rustState;
        return undefined;
    }),
}));

describe('connection status', () => {
    beforeEach(() => {
        // Fresh module per test: its state and re-check timer are
        // module-level singletons, and the timer must start on this
        // test's fake clock.
        vi.resetModules();
        rustState = { status: 'disconnected' };
        vi.useFakeTimers();
        localStorage.clear();   // no saved credentials: no auto-connect
    });
    afterEach(() => {
        vi.useRealTimers();
    });

    it('corrects a stale status from the real one', async () => {
        const { useConnection } = await import('../useConnection');
        const conn = useConnection();
        await vi.advanceTimersByTimeAsync(0);   // let init register listeners

        rustState = { status: 'connected' };
        listeners['connection-status']({ payload: { status: 'connected' } });
        expect(conn.isConnected.value).toBe(true);

        // A late event from a retired connection loop.
        listeners['connection-status']({ payload: { status: 'disconnected' } });
        expect(conn.isConnected.value).toBe(false);

        await vi.advanceTimersByTimeAsync(2100);
        expect(conn.isConnected.value).toBe(true);
    });

    it('shows a real disconnect even if its event was missed', async () => {
        const { useConnection } = await import('../useConnection');
        const conn = useConnection();
        await vi.advanceTimersByTimeAsync(0);
        rustState = { status: 'connected' };
        listeners['connection-status']({ payload: { status: 'connected' } });

        rustState = { status: 'error', error: 'Server not responding' };
        await vi.advanceTimersByTimeAsync(2100);
        expect(conn.isConnected.value).toBe(false);
        expect(conn.error.value).toBe('Server not responding');
    });
});
