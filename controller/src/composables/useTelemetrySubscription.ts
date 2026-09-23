/**
 * Ref-counted `pin_state` telemetry subscriptions.
 *
 * Telemetry subscriptions used to be one-way. The dashboard subscribed
 * to `pin_state/<node>` for every adopted node (and `pin_state/
 * host_controller` for the WiFi panel) the first time it mounted, and
 * the server then pushed those frames for the rest of the session — on
 * every view, whether or not anything rendered them. Each frame cost
 * Wi-Fi airtime, an IPC hop into the webview, and a reactive update that
 * re-ran the consuming computed. Visiting the dashboard once was enough
 * to pay that forever.
 *
 * This wraps a subscription in a reference count so it exists only while
 * something is actually consuming it. Consumers acquire on setup and
 * release when their effect scope is disposed (see `holdWhileInScope`),
 * so the common case needs no lifecycle code in the component.
 *
 * Topics are tracked, not assumed: `add()` records what was actually
 * subscribed so release can unsubscribe exactly that set. The battery
 * panel's topics are discovered at runtime (one per adopted node), so
 * there is no static list to unsubscribe from.
 */

import { getCurrentScope, onScopeDispose } from 'vue';
import { invoke } from '@tauri-apps/api/core';
import { listen } from '@tauri-apps/api/event';

export interface TelemetrySubscription {
    /** Register a consumer. The first one activates the subscription. */
    acquire(): void;
    /** Drop a consumer. The last one unsubscribes. */
    release(): void;
    /** Whether any consumer currently holds this. */
    isActive(): boolean;
    /**
     * Subscribe to more topics. No-op while inactive — a frame that
     * arrives for a topic nobody holds is exactly what this exists to
     * prevent, so a late `adopted-nodes` event after release must not
     * quietly re-open the stream.
     */
    add(topics: string[]): void;
    /**
     * Forget the tracked topic set WITHOUT unsubscribing. For link loss:
     * subscriptions live on the server's per-connection client object, so
     * they are already gone — sending an unsubscribe would be pointless
     * and re-subscribing on reconnect needs a clean slate.
     */
    forget(): void;
    /** Re-run discovery, e.g. after a reconnect. No-op while inactive. */
    refresh(): void;
    /**
     * Start or stop actually receiving, without changing the holder
     * count. Driven by window focus; see the Focus gating section.
     */
    setStreaming(on: boolean): void;
}

// ─── Focus gating ────────────────────────────────────────────────────
//
// A backgrounded window renders nothing, so streaming telemetry into it
// is Wi-Fi airtime and webview wake-ups for frames no one sees. Rust
// emits `app-focus` on every window focus change (see lib.rs); every
// live subscription drops its topics while hidden and re-subscribes on
// return.
//
// This is separate from the reference count on purpose: a consumer still
// HOLDS the subscription while hidden — the dashboard is still mounted —
// it just doesn't want bytes for the moment. Conflating the two would
// make refocus look like a fresh acquire and lose the holder count.

const registry = new Set<TelemetrySubscription>();
let windowFocused = true;
let focusWatchStarted = false;

function startFocusWatch(): void {
    if (focusWatchStarted) return;
    focusWatchStarted = true;
    void listen<boolean>('app-focus', event => {
        const focused = !!event.payload;
        if (focused === windowFocused) return;
        windowFocused = focused;
        for (const sub of registry) sub.setStreaming(focused);
    }).catch(err =>
        console.error('[telemetry] app-focus listen failed:', err));
}

/**
 * @param label  Used only in log lines, so a leak is attributable.
 * @param onActivate  Called when the count goes 0 → 1, and by `refresh()`.
 *   Receives the `add` function; either call it directly with a fixed
 *   topic list, or kick off discovery and call it from the response
 *   handler.
 */
export function createTelemetrySubscription(
    label: string,
    onActivate: (add: (topics: string[]) => void) => void,
): TelemetrySubscription {
    let holders = 0;
    // What we have actually told the server we want. Release unsubscribes
    // precisely this, so a topic discovered at runtime can be undone.
    const subscribed = new Set<string>();
    // Topics a holder still wants but which are not on the wire right
    // now, because the window is backgrounded. Kept so refocus restores
    // exactly what was dropped — including runtime-discovered topics,
    // which no static list could reproduce.
    const parked = new Set<string>();

    function sendSubscribe(topics: string[]): void {
        if (topics.length === 0) return;
        void invoke('subscribe_topics', { topics }).catch(err =>
            console.error(`[${label}] subscribe_topics failed:`, err));
    }

    function sendUnsubscribe(topics: string[]): void {
        if (topics.length === 0) return;
        // The Rust command treats a down link as success, so this needs
        // no connection check of its own.
        void invoke('unsubscribe_topics', { topics }).catch(err =>
            console.error(`[${label}] unsubscribe_topics failed:`, err));
    }

    function add(topics: string[]): void {
        if (holders === 0) return;
        const fresh = topics.filter(
            topic => topic && !subscribed.has(topic) && !parked.has(topic));
        if (fresh.length === 0) return;
        if (!windowFocused) {
            // Remember the intent; don't put it on the wire until the
            // window is back. A discovery response that lands while
            // hidden must not re-open the stream.
            for (const topic of fresh) parked.add(topic);
            return;
        }
        for (const topic of fresh) subscribed.add(topic);
        sendSubscribe(fresh);
    }

    const sub: TelemetrySubscription = {
        acquire(): void {
            holders++;
            if (holders !== 1) return;
            registry.add(sub);
            startFocusWatch();
            onActivate(add);
        },

        release(): void {
            if (holders === 0) return;
            holders--;
            if (holders > 0) return;
            registry.delete(sub);
            const topics = [...subscribed];
            subscribed.clear();
            parked.clear();
            sendUnsubscribe(topics);
        },

        isActive(): boolean {
            return holders > 0;
        },

        add,

        forget(): void {
            subscribed.clear();
            parked.clear();
        },

        refresh(): void {
            if (holders === 0) return;
            onActivate(add);
        },

        setStreaming(on: boolean): void {
            // Holder count is untouched: the dashboard is still mounted
            // and still wants this data, it just can't render it now.
            if (holders === 0) return;
            if (on) {
                const restore = [...parked];
                parked.clear();
                for (const topic of restore) subscribed.add(topic);
                sendSubscribe(restore);
            } else {
                const drop = [...subscribed];
                subscribed.clear();
                for (const topic of drop) parked.add(topic);
                sendUnsubscribe(drop);
            }
        },
    };

    return sub;
}

/**
 * Hold `sub` for as long as the calling component lives.
 *
 * Uses the effect scope rather than `onMounted`/`onBeforeUnmount` so it
 * works from a composable called anywhere in setup. Called from outside a
 * scope there is nothing to release on, so it warns and holds for the
 * session — the data still flows (silently dropping the acquire would
 * leave a caller with a dead feed and no clue why), but the leak is
 * attributable instead of invisible.
 */
export function holdWhileInScope(label: string, sub: TelemetrySubscription): void {
    sub.acquire();
    if (getCurrentScope()) {
        onScopeDispose(() => sub.release());
    } else {
        console.warn(
            `[${label}] acquired telemetry outside a component scope; ` +
                'it will stay subscribed for the rest of the session',
        );
    }
}
