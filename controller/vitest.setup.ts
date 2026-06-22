/**
 * Vitest global setup.
 *
 * Node 26 ships an experimental global `localStorage` that is `undefined`
 * unless the process is launched with `--localstorage-file`. That global
 * shadows the one happy-dom would otherwise provide, so any code that
 * touches `localStorage` (e.g. useKeyboard's enabled-flag persistence)
 * sees `undefined`. Provide a small in-memory Storage so those code
 * paths actually run under test.
 */
function inMemoryStorage(): Storage {
    const map = new Map<string, string>();
    return {
        getItem: (k: string) => (map.has(k) ? map.get(k)! : null),
        setItem: (k: string, v: string) => { map.set(k, String(v)); },
        removeItem: (k: string) => { map.delete(k); },
        clear: () => { map.clear(); },
        key: (i: number) => Array.from(map.keys())[i] ?? null,
        get length() { return map.size; },
    } as Storage;
}

const existing = (globalThis as { localStorage?: Storage }).localStorage;
if (!existing || typeof existing.getItem !== 'function') {
    Object.defineProperty(globalThis, 'localStorage', {
        value: inMemoryStorage(),
        configurable: true,
        writable: true,
    });
}
