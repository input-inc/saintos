import { onUnmounted } from 'vue'

// Throttle a send-on-change function. Coalesces a trailing call so the
// final value always reaches the server even when the user stops
// dragging right after a throttled tick.
export function useThrottledSend (sendFn, ms = 50) {
  let lastAt = 0
  let pendingValue = null
  let timer = null

  function fire (value) {
    pendingValue = null
    timer = null
    lastAt = Date.now()
    sendFn(value)
  }

  function call (value) {
    const now = Date.now()
    if (now - lastAt >= ms) {
      fire(value)
      return
    }
    pendingValue = value
    if (timer) return
    timer = setTimeout(() => fire(pendingValue), ms - (now - lastAt))
  }

  onUnmounted(() => { if (timer) clearTimeout(timer) })

  return call
}
