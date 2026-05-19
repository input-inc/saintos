import { computed, onMounted, onUnmounted, watch } from 'vue'
import { useWsStore } from '@/stores/ws'

// Subscribe a component to a server topic. Returns a computed ref that
// holds the latest `data` payload broadcast on that topic.
//
//   const pinState = useWsTopic(() => `pin_state/${nodeId.value}`)
//
// The composable handles (un)subscribe on mount/unmount and reactively
// re-subscribes when the topic key changes.
export function useWsTopic (topicGetter, rateHz = 10) {
  const ws = useWsStore()
  const key = typeof topicGetter === 'function' ? computed(topicGetter) : computed(() => topicGetter)
  let current = null

  async function attach (topic) {
    if (!topic || topic === current) return
    if (current) {
      try { await ws.unsubscribe([current]) } catch (_) {}
    }
    current = topic
    try { await ws.subscribe([topic], rateHz) } catch (e) {
      console.warn(`Subscribe to ${topic} failed:`, e)
    }
  }

  onMounted(() => attach(key.value))
  onUnmounted(() => {
    if (current) ws.unsubscribe([current]).catch(() => {})
    current = null
  })
  watch(key, (v) => attach(v))

  // Subscribe should auto-renew across reconnects.
  ws.on('ready', () => { if (key.value) ws.subscribe([key.value], rateHz).catch(() => {}) })

  return computed(() => key.value ? ws.topics.get(key.value) : null)
}
