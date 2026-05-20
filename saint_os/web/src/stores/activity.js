import { defineStore } from 'pinia'
import { ref } from 'vue'
import { useWsStore } from './ws'

const MAX = 500

// Activity log buffer. Server emits log lines on the 'activity' topic;
// we also accept locally-emitted entries (`add()` from UI actions).
export const useActivityStore = defineStore('activity', () => {
  const ws = useWsStore()
  const entries = ref([])

  ws.on('state', (msg) => {
    if (msg?.node !== 'activity') return
    const e = msg.data
    if (!e) return
    entries.value.push(e)
    if (entries.value.length > MAX) entries.value.splice(0, entries.value.length - MAX)
  })

  ws.on('activity', (e) => {
    if (!e) return
    entries.value.push(e)
    if (entries.value.length > MAX) entries.value.splice(0, entries.value.length - MAX)
  })

  function add (text, level = 'info') {
    entries.value.push({ time: Date.now() / 1000, text, level })
    if (entries.value.length > MAX) entries.value.splice(0, entries.value.length - MAX)
  }

  function clear () { entries.value = [] }

  return { entries, add, clear }
})
