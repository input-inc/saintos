/**
 * Frontend → log-file forwarding.
 *
 * Everything the Vue layer reports used to stop at the devtools
 * console: 50-odd `console.error`/`warn` sites, plus uncaught
 * exceptions and rejected promises. None of it reached
 * `saint-controller.log`, so pulling the log off a Deck after a field
 * failure showed the Rust half of the story and nothing else — the
 * layer most likely to have produced a blank screen was invisible.
 *
 * This patches `console` and the global error hooks to also hand each
 * line to the Rust logger (`log_frontend`), which writes it to the same
 * file as everything else under the `frontend` target. The original
 * console methods are still called, so devtools behaves exactly as
 * before.
 *
 * Install once, as early as possible — see main.ts.
 */

import { invoke } from '@tauri-apps/api/core';

type Level = 'error' | 'warn' | 'info' | 'debug';

interface Entry {
    level: Level;
    context: string;
    message: string;
    /** Send attempts so far; see drain() for why this is per-entry. */
    attempts: number;
}

/** Console methods map onto the Rust logger's levels. `console.log` is
 *  this codebase's informational default, so it lands at info rather
 *  than debug. */
const LEVEL_OF: Record<string, Level> = {
    error: 'error',
    warn: 'warn',
    info: 'info',
    log: 'info',
    debug: 'debug',
};

/** Cap on unsent lines held in memory. A component erroring on every
 *  frame must not grow this without bound — better to drop and say so
 *  than to let the log queue become the memory leak that takes the app
 *  down. */
const QUEUE_LIMIT = 500;

/** Per-entry send attempts before giving up on it. The Tauri bridge can
 *  lag the first paint (main.ts has the same note about `invoke`
 *  racing bridge setup), and the lines logged during boot are exactly
 *  the ones worth keeping. */
const MAX_ATTEMPTS = 3;
const RETRY_DELAY_MS = 250;

// ─── Module state ────────────────────────────────────────────────────

// The unpatched console methods. Every internal diagnostic in this
// module must go through these, never through the patched console — a
// `console.error` inside the forwarding path would re-enter the patch
// and recurse until the stack blows.
//
// Re-captured by installLogForwarding() immediately before patching,
// rather than frozen at import: this module may be imported well before
// it is installed, and whatever is on `console` at install time is what
// we must keep calling.
const native = {
    error: console.error.bind(console),
    warn: console.warn.bind(console),
    info: console.info.bind(console),
    log: console.log.bind(console),
    debug: console.debug.bind(console),
};

function captureNative(): void {
    native.error = console.error.bind(console);
    native.warn = console.warn.bind(console);
    native.info = console.info.bind(console);
    native.log = console.log.bind(console);
    native.debug = console.debug.bind(console);
}

const queue: Entry[] = [];
let dropped = 0;
let draining = false;
let installed = false;
let unavailableWarned = false;

// ─── Formatting ──────────────────────────────────────────────────────

function safeJson(value: unknown): string {
    const seen = new WeakSet<object>();
    try {
        return (
            JSON.stringify(value, (_key, val) => {
                if (typeof val === 'object' && val !== null) {
                    if (seen.has(val as object)) return '[Circular]';
                    seen.add(val as object);
                }
                return val;
            }) ?? String(value)
        );
    } catch {
        return String(value);
    }
}

/** One console argument → a log-file-friendly string. Errors keep their
 *  stack: a stack is the single most useful thing in a crash report and
 *  `String(err)` throws it away. */
function formatValue(value: unknown): string {
    if (typeof value === 'string') return value;
    if (value instanceof Error) {
        return value.stack
            ? `${value.name}: ${value.message}\n${value.stack}`
            : `${value.name}: ${value.message}`;
    }
    if (value === undefined) return 'undefined';
    if (value === null) return 'null';
    if (typeof value === 'object') return safeJson(value);
    return String(value);
}

function formatArgs(args: unknown[]): string {
    return args.map(formatValue).join(' ');
}

/** This codebase prefixes console output with a `[source]` tag
 *  (`'[useBatteries] subscribe failed:'`). Lift that into the Rust
 *  command's `context` so the file gets one tidy tag instead of a
 *  bracket buried mid-line. */
function splitContext(message: string): { context: string; message: string } {
    const match = /^\[([^\]]{1,64})\]\s*/.exec(message);
    if (!match) return { context: 'ui', message };
    return { context: match[1], message: message.slice(match[0].length) };
}

// ─── Queue ───────────────────────────────────────────────────────────

function enqueue(level: Level, rawMessage: string): void {
    if (queue.length >= QUEUE_LIMIT) {
        // Drop the NEWEST, not the oldest. In a cascade the first error
        // is usually the cause and the rest are consequences, so the
        // head of the queue is the part worth keeping.
        dropped++;
        return;
    }
    const { context, message } = splitContext(rawMessage);
    queue.push({ level, context, message, attempts: 0 });
    void drain();
}

function sleep(ms: number): Promise<void> {
    return new Promise(resolve => setTimeout(resolve, ms));
}

async function drain(): Promise<void> {
    if (draining) return;
    draining = true;
    try {
        // `dropped > 0` is part of the loop condition, not a tail step
        // after it. Pushing the notice below the loop and calling
        // drain() again could never work: `draining` is still true at
        // that point, so the recursive call returned immediately and the
        // notice sat in the queue until the next unrelated log line.
        while (queue.length > 0 || dropped > 0) {
            if (queue.length === 0 && dropped > 0) {
                const count = dropped;
                dropped = 0;
                // Record the gap rather than hiding it — a log with a
                // silent hole is worse than one that admits the hole.
                queue.push({
                    level: 'warn',
                    context: 'logForwarding',
                    message: `${count} frontend log line(s) dropped: queue was full`,
                    attempts: 0,
                });
            }

            const entry = queue[0];
            try {
                await invoke('log_frontend', {
                    level: entry.level,
                    message: entry.message,
                    context: entry.context,
                });
                queue.shift();
            } catch (err) {
                entry.attempts++;
                // Retrying is for the boot race, where the bridge shows
                // up a moment later. Once we've concluded it is not
                // coming (plain `npm run dev` in a browser, no Tauri at
                // all), stop paying the retry delay per line — otherwise
                // every log line costs MAX_ATTEMPTS * RETRY_DELAY_MS and
                // the queue backs up into spurious drop notices.
                if (unavailableWarned || entry.attempts >= MAX_ATTEMPTS) {
                    queue.shift();
                    if (!unavailableWarned) {
                        unavailableWarned = true;
                        // native.* only — see the `native` comment.
                        native.warn(
                            '[logForwarding] cannot reach the Rust logger; ' +
                                'frontend lines stay console-only:',
                            err,
                        );
                    }
                } else {
                    // Leave it at the head and back off: most likely the
                    // bridge just isn't up yet.
                    await sleep(RETRY_DELAY_MS);
                }
            }
        }
    } finally {
        draining = false;
    }
}

// ─── Install ─────────────────────────────────────────────────────────

/** Report a Vue component error. Wired to `app.config.errorHandler` in
 *  main.ts — overriding that handler suppresses Vue's own console
 *  output, so this re-emits it through the native console as well. */
export function logVueError(err: unknown, info: string): void {
    native.error('[vue]', err, info);
    enqueue('error', `[vue] ${formatValue(err)} (${info})`);
}

/** Patch `console` and the global error hooks so the Vue layer's output
 *  reaches the same log file as the Rust side. Idempotent. */
export function installLogForwarding(): void {
    if (installed) return;
    installed = true;

    captureNative();

    for (const method of Object.keys(LEVEL_OF) as (keyof typeof native)[]) {
        const level = LEVEL_OF[method];
        const original = native[method];
        (console as unknown as Record<string, unknown>)[method] = (
            ...args: unknown[]
        ): void => {
            original(...args);
            try {
                enqueue(level, formatArgs(args));
            } catch {
                // Formatting must never break the call site that logged.
            }
        };
    }

    // Uncaught exceptions. This is the case the whole change exists for:
    // a throw during render leaves a blank screen, and before this the
    // log file said nothing about why.
    window.addEventListener('error', event => {
        const detail = event.error
            ? formatValue(event.error)
            : event.message || 'unknown error';
        const where = event.filename
            ? ` (${event.filename}:${event.lineno}:${event.colno})`
            : '';
        enqueue('error', `[window] Uncaught: ${detail}${where}`);
    });

    // A rejected promise nobody caught — a failed `invoke` in a
    // fire-and-forget `void somePromise()` lands here.
    window.addEventListener('unhandledrejection', event => {
        enqueue('error', `[window] Unhandled rejection: ${formatValue(event.reason)}`);
    });
}
