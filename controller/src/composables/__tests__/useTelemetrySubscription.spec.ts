/**
 * Ref-counted telemetry subscriptions. The property under test is that a
 * `pin_state` stream exists exactly while something consumes it —
 * previously it was subscribed once and never torn down, so visiting the
 * dashboard cost Wi-Fi airtime and webview wake-ups for the rest of the
 * session.
 */
import { afterEach, beforeEach, describe, expect, it, vi } from 'vitest';
import { effectScope } from 'vue';

const invoke = vi.fn();
vi.mock('@tauri-apps/api/core', () => ({ invoke: (...args: unknown[]) => invoke(...args) }));

// Capture the app-focus listener so tests can drive focus changes.
const bridge = vi.hoisted(() => ({ focus: [] as Array<(e: unknown) => void> }));
vi.mock('@tauri-apps/api/event', () => ({
    listen: vi.fn((event: string, cb: (e: unknown) => void) => {
        if (event === 'app-focus') bridge.focus.push(cb);
        return Promise.resolve(() => {});
    }),
}));

import {
    createTelemetrySubscription,
    holdWhileInScope,
} from '../useTelemetrySubscription';

/**
 * Fresh module instance. The focus gate keeps module-level state (the
 * registry of live subscriptions and the current focus), so tests that
 * drive focus must not share it — otherwise subscriptions from earlier
 * tests are still registered and react to the event too.
 */
async function freshModule() {
    vi.resetModules();
    bridge.focus.length = 0;
    return import('../useTelemetrySubscription');
}

/** Drive a window focus change through the Rust-emitted event. */
async function setFocus(focused: boolean): Promise<void> {
    // listen() resolves on a microtask; let registration land first.
    for (let i = 0; i < 5; i++) await Promise.resolve();
    for (const cb of bridge.focus) cb({ payload: focused });
}

/** Topic lists passed to a given command, in call order. */
function callsTo(command: string): string[][] {
    return invoke.mock.calls
        .filter(c => c[0] === command)
        .map(c => (c[1] as { topics: string[] }).topics);
}

describe('createTelemetrySubscription', () => {
    let warn: typeof console.warn;

    beforeEach(() => {
        invoke.mockReset();
        invoke.mockResolvedValue(undefined);
        warn = console.warn;
    });

    afterEach(() => {
        console.warn = warn;
    });

    it('subscribes on first acquire and unsubscribes on last release', async () => {
        const sub = createTelemetrySubscription('t', add => add(['pin_state/a']));

        sub.acquire();
        expect(callsTo('subscribe_topics')).toEqual([['pin_state/a']]);
        expect(sub.isActive()).toBe(true);

        sub.release();
        expect(callsTo('unsubscribe_topics')).toEqual([['pin_state/a']]);
        expect(sub.isActive()).toBe(false);
    });

    it('keeps the stream while any holder remains', () => {
        const sub = createTelemetrySubscription('t', add => add(['pin_state/a']));

        sub.acquire();
        sub.acquire();
        sub.release();
        expect(callsTo('unsubscribe_topics')).toEqual([]);
        expect(sub.isActive()).toBe(true);

        sub.release();
        expect(callsTo('unsubscribe_topics')).toEqual([['pin_state/a']]);
    });

    it('activates only once across overlapping holders', () => {
        const onActivate = vi.fn();
        const sub = createTelemetrySubscription('t', onActivate);
        sub.acquire();
        sub.acquire();
        sub.acquire();
        expect(onActivate).toHaveBeenCalledTimes(1);
    });

    /// Topics are discovered at runtime (one per adopted node), so release
    /// has to unsubscribe whatever was actually added — there is no static
    /// list to fall back on.
    it('unsubscribes exactly the topics that were added', () => {
        const sub = createTelemetrySubscription('t', () => {});
        sub.acquire();
        sub.add(['pin_state/n1', 'pin_state/n2']);
        sub.add(['pin_state/n3']);
        sub.release();

        expect(callsTo('unsubscribe_topics')).toEqual([
            ['pin_state/n1', 'pin_state/n2', 'pin_state/n3'],
        ]);
    });

    it('does not re-subscribe a topic it already holds', () => {
        const sub = createTelemetrySubscription('t', () => {});
        sub.acquire();
        sub.add(['pin_state/n1']);
        sub.add(['pin_state/n1', 'pin_state/n2']);

        // Only the genuinely new topic goes on the wire.
        expect(callsTo('subscribe_topics')).toEqual([
            ['pin_state/n1'],
            ['pin_state/n2'],
        ]);
    });

    /// The discovery response is async: `get_adopted_nodes` can land after
    /// the dashboard unmounted. If add() honoured that, the stream would
    /// silently re-open with nobody watching — the exact leak this exists
    /// to close.
    it('ignores topics added while nothing holds it', () => {
        const sub = createTelemetrySubscription('t', () => {});
        sub.add(['pin_state/late']);
        expect(callsTo('subscribe_topics')).toEqual([]);

        sub.acquire();
        sub.release();
        sub.add(['pin_state/later']);
        expect(callsTo('subscribe_topics')).toEqual([]);
        // Nothing tracked means nothing to unsubscribe.
        expect(callsTo('unsubscribe_topics')).toEqual([]);
    });

    it('release is idempotent and cannot drive the count negative', () => {
        const sub = createTelemetrySubscription('t', add => add(['pin_state/a']));
        sub.release();
        sub.release();
        expect(callsTo('unsubscribe_topics')).toEqual([]);

        // A stray release must not have left the count below zero, or the
        // next acquire wouldn't activate.
        sub.acquire();
        expect(callsTo('subscribe_topics')).toEqual([['pin_state/a']]);
    });

    /// On link loss the server drops subscriptions with the connection, so
    /// sending an unsubscribe is pointless — but the tracked set must be
    /// cleared or the reconnect would consider those topics already live.
    it('forget clears tracking without sending an unsubscribe', () => {
        const sub = createTelemetrySubscription('t', () => {});
        sub.acquire();
        sub.add(['pin_state/n1']);

        sub.forget();
        expect(callsTo('unsubscribe_topics')).toEqual([]);

        // Re-adding after forget must reach the wire again.
        sub.add(['pin_state/n1']);
        expect(callsTo('subscribe_topics')).toEqual([
            ['pin_state/n1'],
            ['pin_state/n1'],
        ]);
    });

    it('refresh re-runs discovery only while held', () => {
        const onActivate = vi.fn();
        const sub = createTelemetrySubscription('t', onActivate);

        sub.refresh();
        expect(onActivate).not.toHaveBeenCalled();

        sub.acquire();
        sub.refresh();
        expect(onActivate).toHaveBeenCalledTimes(2); // acquire + refresh
    });
});

describe('focus gating', () => {
    beforeEach(() => {
        invoke.mockReset();
        invoke.mockResolvedValue(undefined);
    });

    /// A hidden window renders nothing, so frames pushed to it are pure
    /// Wi-Fi airtime and webview wake-ups.
    it('drops topics off the wire while backgrounded and restores them', async () => {
        const { createTelemetrySubscription } = await freshModule();
        const sub = createTelemetrySubscription('t', add => add(['pin_state/a']));
        sub.acquire();
        expect(callsTo('subscribe_topics')).toEqual([['pin_state/a']]);

        await setFocus(false);
        expect(callsTo('unsubscribe_topics')).toEqual([['pin_state/a']]);

        await setFocus(true);
        expect(callsTo('subscribe_topics')).toEqual([
            ['pin_state/a'],
            ['pin_state/a'],
        ]);
    });

    /// Backgrounding must not look like a release: the dashboard is still
    /// mounted and still wants this data. If the holder count moved,
    /// refocus would read as a fresh acquire and the count would drift.
    it('does not change the holder count', async () => {
        const { createTelemetrySubscription } = await freshModule();
        const sub = createTelemetrySubscription('t', add => add(['pin_state/a']));
        sub.acquire();
        await setFocus(false);
        expect(sub.isActive()).toBe(true);
        await setFocus(true);
        expect(sub.isActive()).toBe(true);
    });

    /// Topics are discovered at runtime, so a response landing while
    /// hidden has to be remembered rather than either dropped (data lost
    /// on return) or sent (stream re-opened behind our back).
    it('parks topics discovered while hidden and subscribes them on return', async () => {
        const { createTelemetrySubscription } = await freshModule();
        const sub = createTelemetrySubscription('t', () => {});
        sub.acquire();
        await setFocus(false);

        sub.add(['pin_state/late']);
        expect(callsTo('subscribe_topics')).toEqual([]);

        await setFocus(true);
        expect(callsTo('subscribe_topics')).toEqual([['pin_state/late']]);
    });

    it('releasing while hidden still unsubscribes nothing twice', async () => {
        const { createTelemetrySubscription } = await freshModule();
        const sub = createTelemetrySubscription('t', add => add(['pin_state/a']));
        sub.acquire();
        await setFocus(false);
        // Already off the wire; release must not send a second unsubscribe.
        sub.release();
        expect(callsTo('unsubscribe_topics')).toEqual([['pin_state/a']]);
        await setFocus(true);
        // Nothing held any more, so nothing comes back.
        expect(callsTo('subscribe_topics')).toEqual([['pin_state/a']]);
    });
});

describe('holdWhileInScope', () => {
    beforeEach(() => {
        invoke.mockReset();
        invoke.mockResolvedValue(undefined);
    });

    it('releases when the component scope is disposed', () => {
        const sub = createTelemetrySubscription('t', add => add(['pin_state/a']));
        const scope = effectScope();

        scope.run(() => holdWhileInScope('t', sub));
        expect(sub.isActive()).toBe(true);

        // Unmount equivalent. This is why the app must not wrap views in
        // <KeepAlive>: a kept-alive view's scope is never disposed.
        scope.stop();
        expect(sub.isActive()).toBe(false);
        expect(callsTo('unsubscribe_topics')).toEqual([['pin_state/a']]);
    });

    it('warns but still subscribes outside a scope', () => {
        const spy = vi.fn();
        const original = console.warn;
        console.warn = spy;
        try {
            const sub = createTelemetrySubscription('t', add => add(['pin_state/a']));
            holdWhileInScope('t', sub);
            // Data must still flow — dropping the acquire would leave the
            // caller with a dead feed and no explanation.
            expect(sub.isActive()).toBe(true);
            expect(spy).toHaveBeenCalled();
        } finally {
            console.warn = original;
        }
    });

    it('two components share one subscription and the last one out closes it', () => {
        const sub = createTelemetrySubscription('t', add => add(['pin_state/a']));
        const a = effectScope();
        const b = effectScope();

        a.run(() => holdWhileInScope('t', sub));
        b.run(() => holdWhileInScope('t', sub));
        expect(callsTo('subscribe_topics')).toEqual([['pin_state/a']]);

        a.stop();
        expect(sub.isActive()).toBe(true);
        b.stop();
        expect(callsTo('unsubscribe_topics')).toEqual([['pin_state/a']]);
    });
});
