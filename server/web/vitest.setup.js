// Vitest global setup.
//
// Node 26 ships an experimental global `localStorage` that is `undefined`
// unless launched with `--localstorage-file`, and it shadows the one
// happy-dom would provide. Any store/composable that persists settings
// (display theme, ws connection settings, …) would then see `undefined`.
// Provide a small in-memory Storage so those paths run under test.
function inMemoryStorage () {
  const map = new Map()
  return {
    getItem: k => (map.has(k) ? map.get(k) : null),
    setItem: (k, v) => { map.set(k, String(v)) },
    removeItem: k => { map.delete(k) },
    clear: () => { map.clear() },
    key: i => Array.from(map.keys())[i] ?? null,
    get length () { return map.size },
  }
}

const existing = globalThis.localStorage
if (!existing || typeof existing.getItem !== 'function') {
  Object.defineProperty(globalThis, 'localStorage', {
    value: inMemoryStorage(),
    configurable: true,
    writable: true,
  })
}
