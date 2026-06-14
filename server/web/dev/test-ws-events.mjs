// Standalone sanity test for the WS store's event surface.
//
// Exercises the same chain the Histoire story uses:
//   1. Create Pinia + activate it
//   2. Instantiate the WS store
//   3. Register a 'state' handler via ws.on
//   4. Fire ws.emit('state', ...)
//   5. Assert the handler fired
//
// If this passes here, the WS store works fine and any failure in
// Histoire is environmental (HMR cache, iframe scoping, etc.). If
// it fails here, the WS store itself has the bug.

// Browser globals the store touches at construction time.
globalThis.location = { protocol: 'http:', host: 'localhost' }
globalThis.WebSocket = class {}

import { createApp } from 'vue'
import { createPinia, setActivePinia } from 'pinia'
import { useWsStore } from '../src/stores/ws.js'

const pinia = createPinia()
const app = createApp({})
app.use(pinia)
setActivePinia(pinia)

const ws = useWsStore()

let received = null
ws.on('state', (msg) => { received = msg })

console.log('ws.on / ws.emit identity check:')
console.log('  typeof ws.on   =', typeof ws.on)
console.log('  typeof ws.off  =', typeof ws.off)
console.log('  typeof ws.emit =', typeof ws.emit)

ws.emit('state', { node: 'pin_state/test_node', data: { channels: [{ peripheral_id: 'bms', channel_id: 'soc', value: 42 }] } })

if (!received) {
  console.error('FAIL: emit("state", ...) did NOT reach the handler')
  process.exit(1)
}
console.log('OK: handler fired with', received)

// Reactive Map test — does ws.topics.set trigger a watch?
import { watch } from 'vue'
let mapHits = 0
const topic = 'pin_state/another_test'
watch(() => ws.topics.get(topic), () => { mapHits++ })
ws.topics.set(topic, { channels: [{ peripheral_id: 'bms', channel_id: 'soc', value: 1 }] })
// give vue's flush queue a tick
await Promise.resolve()
console.log(`ws.topics.set after a 1ms tick: mapHits=${mapHits}`)
if (mapHits === 0) {
  console.warn('WARN: ws.topics.set did NOT trigger a watch on ws.topics.get — this is the Map-reactivity through Pinia issue we suspected.')
}

console.log('done')
