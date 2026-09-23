import { afterEach, beforeEach, describe, expect, it, vi } from 'vitest';

const invoke = vi.fn();
vi.mock('@tauri-apps/api/core', () => ({ invoke: (...args: unknown[]) => invoke(...args) }));

// The module patches the global console on install and keeps
// module-level state, so each test needs a fresh copy of it.
async function freshModule() {
    vi.resetModules();
    return import('../useLogForwarding');
}

/** Let the module's async drain loop run to completion. */
async function flush(): Promise<void> {
    for (let i = 0; i < 10; i++) await Promise.resolve();
}

/** Payloads sent to the Rust logger so far. */
function sent() {
    return invoke.mock.calls
        .filter(c => c[0] === 'log_frontend')
        .map(c => c[1] as { level: string; context: string; message: string });
}

describe('useLogForwarding', () => {
    const METHODS = ['error', 'warn', 'info', 'log', 'debug'] as const;
    let saved: Record<string, unknown>;

    beforeEach(() => {
        invoke.mockReset();
        invoke.mockResolvedValue(undefined);
        // Every test installs the patch onto the REAL global console.
        // Restoring only the method a test spied on leaves the others
        // patched, and the next test's install then wraps the previous
        // wrapper — the stacked chain is what made these tests lie.
        saved = {};
        for (const m of METHODS) saved[m] = console[m];
    });

    afterEach(() => {
        for (const m of METHODS) {
            (console as unknown as Record<string, unknown>)[m] = saved[m];
        }
    });

    it('forwards console.error to the Rust logger', async () => {
        const { installLogForwarding } = await freshModule();
        installLogForwarding();

        console.error('boom');
        await flush();

        expect(sent()).toEqual([
            { level: 'error', context: 'ui', message: 'boom' },
        ]);
    });

    it('still calls the original console method', async () => {
        const { installLogForwarding } = await freshModule();
        const spy = vi.fn();
        // Before install: install is what snapshots the native methods.
        console.error = spy;
        installLogForwarding();

        console.error('boom');

        // Patching must be additive — devtools output is unchanged.
        expect(spy).toHaveBeenCalledWith('boom');
    });

    it('lifts a leading [tag] into the log context', async () => {
        const { installLogForwarding } = await freshModule();
        installLogForwarding();

        console.warn('[useBatteries] subscribe failed');
        await flush();

        expect(sent()[0]).toEqual({
            level: 'warn',
            context: 'useBatteries',
            message: 'subscribe failed',
        });
    });

    it('maps console.log to info and console.debug to debug', async () => {
        const { installLogForwarding } = await freshModule();
        installLogForwarding();

        console.log('a');
        console.debug('b');
        await flush();

        expect(sent().map(e => e.level)).toEqual(['info', 'debug']);
    });

    it('keeps an Error stack instead of stringifying it away', async () => {
        const { installLogForwarding } = await freshModule();
        installLogForwarding();

        const err = new Error('kaboom');
        console.error('[useInput] listener threw:', err);
        await flush();

        const msg = sent()[0].message;
        expect(msg).toContain('listener threw:');
        expect(msg).toContain('Error: kaboom');
        // A stack is the most useful part of a crash report.
        expect(msg).toContain('useLogForwarding.spec');
    });

    it('survives a circular object without throwing at the call site', async () => {
        const { installLogForwarding } = await freshModule();
        installLogForwarding();

        const circular: Record<string, unknown> = { name: 'node' };
        circular['self'] = circular;

        expect(() => console.error('state:', circular)).not.toThrow();
        await flush();
        expect(sent()[0].message).toContain('[Circular]');
    });

    it('does not recurse when the Rust logger is unreachable', async () => {
        const { installLogForwarding } = await freshModule();
        invoke.mockRejectedValue(new Error('no bridge'));
        const spy = vi.fn();
        console.warn = spy;
        installLogForwarding();

        console.error('boom');
        // Generous drain: the retry path sleeps between attempts.
        await vi.waitFor(() => expect(invoke).toHaveBeenCalled());

        // The internal "can't reach the logger" notice must use the
        // captured native console, never the patched one — otherwise it
        // re-enters the patch and recurses forever.
        expect(spy.mock.calls.length).toBeLessThanOrEqual(1);
    });

    it('reports a dropped-line count rather than hiding the gap', async () => {
        const { installLogForwarding } = await freshModule();
        // Block the drain so the queue can actually fill.
        let release: () => void = () => {};
        invoke.mockImplementation(
            () => new Promise<void>(resolve => { release = () => resolve(); }),
        );
        installLogForwarding();

        // One in flight + QUEUE_LIMIT (500) queued + overflow.
        for (let i = 0; i < 620; i++) console.error(`line ${i}`);

        invoke.mockResolvedValue(undefined);
        release();
        await vi.waitFor(() => {
            const gap = sent().find(e => e.context === 'logForwarding');
            expect(gap).toBeDefined();
            expect(gap!.message).toMatch(/log line\(s\) dropped/);
        });
    });

    it('logVueError records the component error and keeps console output', async () => {
        const { installLogForwarding, logVueError } = await freshModule();
        const spy = vi.fn();
        console.error = spy;
        installLogForwarding();

        logVueError(new Error('render failed'), 'render function');
        await flush();

        // Overriding app.config.errorHandler suppresses Vue's own
        // logging, so this has to print as well as forward.
        expect(spy).toHaveBeenCalled();
        const entry = sent()[0];
        expect(entry.context).toBe('vue');
        expect(entry.message).toContain('render failed');
        expect(entry.message).toContain('render function');
    });
});
