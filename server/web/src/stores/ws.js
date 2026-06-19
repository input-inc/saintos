import { defineStore } from 'pinia'
import { reactive, ref } from 'vue'

// WebSocket store — single shared connection, reactive state, and the
// command/subscribe surface every other store and component leans on.
//
// Wire protocol:
//   - Reconnect with exponential backoff up to maxReconnectAttempts.
//   - Request/response correlation via incrementing message id.
//   - Topic subscriptions emit `state` frames keyed by `{node, data}`.
//   - Binary frames are forwarded to "binary" listeners (terminal uses this).
export const useWsStore = defineStore('ws', () => {
  const url = `${location.protocol === 'https:' ? 'wss:' : 'ws:'}//${location.host}/api/ws`

  // Reactive connection state — components can `useWsStore().connected` etc.
  const connected      = ref(false)
  const authenticated  = ref(false)
  const authRequired   = ref(false)
  const serverName     = ref(null)
  const clientId       = ref(null)
  const reconnectAttempts = ref(0)

  // Latest `state` frame per topic, reactive. Components prefer the
  // `useWsTopic()` composable below — this map is the underlying store.
  const topics = reactive(new Map())

  // Non-reactive bookkeeping. Refs to the live socket and pending
  // request promises don't belong in reactive state.
  let ws = null
  let requestId = 0
  const pending = new Map()                  // id -> {resolve, reject, timer}
  const handlers = new Map()                 // type -> Set<fn>
  const maxReconnect = 10
  const reconnectBase = 1000
  let reconnectTimer = null
  // Auto-reload: once we've had a successful session, a *second* `ready`
  // means the server restarted — reload so the client picks up new code.
  // The hash route survives the reload, so no explicit state save needed.
  let hadSession = false

  // ── Lifecycle ────────────────────────────────────────────────────

  function connect () {
    if (ws && (ws.readyState === WebSocket.OPEN || ws.readyState === WebSocket.CONNECTING)) {
      return
    }
    try {
      ws = new WebSocket(url)
      ws.binaryType = 'arraybuffer'
      ws.onopen    = onOpen
      ws.onclose   = onClose
      ws.onerror   = onError
      ws.onmessage = onMessage
    } catch (e) {
      console.error('WebSocket connect failed:', e)
      scheduleReconnect()
    }
  }

  function disconnect () {
    if (reconnectTimer) clearTimeout(reconnectTimer)
    reconnectTimer = null
    reconnectAttempts.value = maxReconnect    // suppress auto-reconnect
    if (ws) { ws.close(); ws = null }
    connected.value = false
    authenticated.value = false
  }

  function onOpen () {
    connected.value = true
    reconnectAttempts.value = 0
  }

  function onClose (event) {
    connected.value = false
    authenticated.value = false
    emit('disconnected', { code: event.code, reason: event.reason })
    scheduleReconnect()
  }

  function onError (err) {
    console.error('WebSocket error:', err)
    emit('error', err)
  }

  function handleReady () {
    // Reload-on-reconnect is handled earlier in the 'connected'
    // handler (so the user logs into the fresh bundle ONCE, not twice
    // — see the comment there). By the time handleReady runs in a
    // post-reload flow, hadSession is already false (fresh module).
    hadSession = true
    emit('ready')
  }

  function scheduleReconnect () {
    if (reconnectAttempts.value >= maxReconnect) {
      emit('reconnect_failed')
      return
    }
    const delay = reconnectBase * Math.pow(2, reconnectAttempts.value)
    reconnectAttempts.value++
    reconnectTimer = setTimeout(connect, delay)
  }

  // ── Inbound message routing ──────────────────────────────────────

  function onMessage (event) {
    // Binary frames bypass JSON — terminal feature streams PTY output here.
    if (event.data instanceof ArrayBuffer) {
      emit('binary', event.data); return
    }
    if (typeof Blob !== 'undefined' && event.data instanceof Blob) {
      event.data.arrayBuffer().then(buf => emit('binary', buf))
        .catch(e => console.warn('Binary frame read failed:', e))
      return
    }

    let msg
    try { msg = JSON.parse(event.data) }
    catch (e) { console.error('Bad WS frame:', e); return }

    // Server hello — sets serverName / clientId and tells us whether auth is needed.
    if (msg.type === 'connected') {
      // If we had a working session in this page-load before, this is
      // a RECONNECT after the WS dropped. Most often that means the
      // server restarted (install of a new build, manual systemctl
      // restart, deploy). Reload here so the user logs into the FRESH
      // JS bundle once — the previous flow reloaded inside
      // handleReady() AFTER the user had already re-auth'd, which
      // forced them to type the password a second time on the
      // reloaded page. Doing it now means:
      //   - browser fetches the new index.html + asset hashes
      //   - LoginScreen on the fresh page pre-fills the password
      //     from localStorage (saint_ws_settings) so it's one click
      //     to get back in
      // Note: a transient network blip with no server change still
      // triggers this reload. It's wasteful but not broken — one
      // click later they're back in. Adding a server-side version
      // field to the hello would let us skip the reload when the
      // build hash hasn't changed; defer that until it actually
      // bites.
      if (hadSession) {
        console.log('Reconnected after session — reloading for fresh server code.')
        setTimeout(() => location.reload(), 100)
        return
      }
      serverName.value   = msg.server_name
      clientId.value     = msg.client_id
      authRequired.value = !!msg.auth_required
      if (!authRequired.value) {
        authenticated.value = true
        handleReady()
      } else {
        emit('auth_required', msg)
      }
      emit('connected', msg)
      return
    }

    // Auth result — flips authenticated, emits 'ready' on success.
    if (msg.type === 'auth_result') {
      if (msg.status === 'ok') {
        authenticated.value = true
        handleReady()
      }
      emit('auth_result', msg)
      return
    }

    // Topic broadcasts arrive as `{type: 'state', node, data}`. Stash
    // the latest payload by node-key and emit so on()-listeners (and
    // useWsTopic) can react.
    if (msg.type === 'state' && msg.node) {
      topics.set(msg.node, msg.data)
    }

    // Correlated request/response.
    if (msg.id && pending.has(msg.id)) {
      const { resolve, reject, timer } = pending.get(msg.id)
      clearTimeout(timer)
      pending.delete(msg.id)
      if (msg.status === 'ok') resolve(msg.data)
      else reject(new Error(msg.message || 'Request failed'))
      return
    }

    emit(msg.type, msg)
  }

  // ── Outbound: send / convenience wrappers ────────────────────────

  function send (type, action, params = {}) {
    if (!connected.value) return Promise.reject(new Error('Not connected'))
    if (authRequired.value && !authenticated.value) {
      return Promise.reject(new Error('Authentication required'))
    }
    const id = `req_${++requestId}`
    const message = { id, type, action, params }
    return new Promise((resolve, reject) => {
      const timer = setTimeout(() => {
        if (pending.has(id)) {
          pending.delete(id)
          reject(new Error('Request timeout'))
        }
      }, 30000)
      pending.set(id, { resolve, reject, timer })
      ws.send(JSON.stringify(message))
    })
  }

  // `creds` is either a plain password string (legacy LoginScreen path)
  // or an object — `{password}` or `{kiosk_token}` — so kiosk sessions
  // can authenticate without a human typing anything. See main.js for
  // the auto-auth-on-?kiosk_token entry.
  function authenticate (creds) {
    if (!ws) return Promise.reject(new Error('Not connected'))
    const body = (typeof creds === 'string' || creds == null)
      ? { password: creds }
      : creds
    return new Promise((resolve, reject) => {
      const handler = (msg) => {
        off('auth_result', handler)
        if (msg.status === 'ok') resolve(msg)
        else reject(new Error(msg.message || 'Auth failed'))
      }
      on('auth_result', handler)
      ws.send(JSON.stringify({ type: 'auth', action: 'login', ...body }))
      setTimeout(() => {
        off('auth_result', handler)
        reject(new Error('Auth timeout'))
      }, 10000)
    })
  }

  const management = (action, params = {}) => send('management', action, params)
  const command    = (target, action, params = {}) => send('command', action, { target, ...params })
  const control    = (action, params = {}) => send('control', action, params)
  const routerCmd  = (action, params = {}) => send('router', action, params)
  const subscribe   = (topicList, rateHz = 10) => send('subscribe', 'subscribe', { topics: topicList, rate_hz: rateHz })
  const unsubscribe = (topicList) => send('subscribe', 'unsubscribe', { topics: topicList })

  // ── Event handler registry ───────────────────────────────────────

  function on (type, handler) {
    if (!handlers.has(type)) handlers.set(type, new Set())
    handlers.get(type).add(handler)
  }
  function off (type, handler) {
    handlers.get(type)?.delete(handler)
  }
  function emit (type, data = null) {
    handlers.get(type)?.forEach(h => {
      try { h(data) } catch (e) { console.error(`Handler error for ${type}:`, e) }
    })
  }

  return {
    // reactive state
    connected, authenticated, authRequired, serverName, clientId, reconnectAttempts, topics,
    // lifecycle
    connect, disconnect,
    // command surface
    send, management, command, control, router: routerCmd, subscribe, unsubscribe, authenticate,
    // events. `emit` is exposed primarily for dev/story harnesses
    // (Histoire mocks) so they can synthesize 'state' frames without
    // monkey-patching `on/off` — reassigning setup-store methods on
    // a Pinia proxy is not reliable, and a missed handler registration
    // is silent. Production code paths never call emit directly; the
    // WebSocket onMessage handler is the only intended caller.
    on, off, emit,
  }
})
