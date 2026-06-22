// The ws store is the dashboard's entire network path: connect → server
// hello → (auth) → ready, request/response correlation by id, topic
// state fan-out, and exponential-backoff reconnect. We swap in a
// controllable mock WebSocket so we can drive every branch deterministically
// without a real server.
import { describe, it, expect, beforeEach, afterEach, vi } from 'vitest'
import { setActivePinia, createPinia } from 'pinia'
import { useWsStore } from '../ws'

let sockets = []

class MockWebSocket {
  static CONNECTING = 0
  static OPEN = 1
  static CLOSING = 2
  static CLOSED = 3
  constructor (url) {
    this.url = url
    this.readyState = MockWebSocket.CONNECTING
    this.binaryType = ''
    this.sent = []
    this.onopen = this.onclose = this.onerror = this.onmessage = null
    sockets.push(this)
  }
  send (data) { this.sent.push(data) }
  close () { this.readyState = MockWebSocket.CLOSED }

  // ── test drivers ──
  _open () { this.readyState = MockWebSocket.OPEN; this.onopen && this.onopen() }
  _recv (obj) {
    this.onmessage && this.onmessage({ data: typeof obj === 'string' ? obj : JSON.stringify(obj) })
  }
  _drop (code = 1006) { this.readyState = MockWebSocket.CLOSED; this.onclose && this.onclose({ code, reason: '' }) }
  lastSent () { return JSON.parse(this.sent[this.sent.length - 1]) }
}

function openStore () {
  const store = useWsStore()
  store.connect()
  const sock = sockets[sockets.length - 1]
  sock._open()
  return { store, sock }
}

describe('ws store', () => {
  beforeEach(() => {
    setActivePinia(createPinia())
    sockets = []
    vi.stubGlobal('WebSocket', MockWebSocket)
  })
  afterEach(() => {
    vi.unstubAllGlobals()
    vi.useRealTimers()
  })

  it('dials the /api/ws endpoint with a ws(s) scheme', () => {
    const store = useWsStore()
    store.connect()
    expect(sockets).toHaveLength(1)
    expect(sockets[0].url).toMatch(/^wss?:\/\/.*\/api\/ws$/)
  })

  it('marks connected on open and resets the reconnect counter', () => {
    const { store } = openStore()
    expect(store.connected).toBe(true)
    expect(store.reconnectAttempts).toBe(0)
  })

  it('goes ready without auth when the server says auth is not required', () => {
    const { store, sock } = openStore()
    const ready = vi.fn()
    store.on('ready', ready)
    sock._recv({ type: 'connected', server_name: 'pi', client_id: 'c1', auth_required: false })
    expect(store.authenticated).toBe(true)
    expect(store.serverName).toBe('pi')
    expect(store.clientId).toBe('c1')
    expect(ready).toHaveBeenCalledOnce()
  })

  it('requests auth when required and authenticates on a good result', () => {
    const { store, sock } = openStore()
    const needAuth = vi.fn()
    store.on('auth_required', needAuth)
    sock._recv({ type: 'connected', auth_required: true })
    expect(store.authRequired).toBe(true)
    expect(store.authenticated).toBe(false)
    expect(needAuth).toHaveBeenCalledOnce()

    sock._recv({ type: 'auth_result', status: 'ok' })
    expect(store.authenticated).toBe(true)
  })

  it('rejects sends while disconnected', async () => {
    const store = useWsStore()
    await expect(store.management('list_nodes')).rejects.toThrow(/not connected/i)
  })

  it('correlates a request to its response by id and resolves with data', async () => {
    const { store, sock } = openStore()
    const p = store.management('list_nodes', { foo: 1 })
    const frame = sock.lastSent()
    expect(frame.type).toBe('management')
    expect(frame.action).toBe('list_nodes')
    expect(frame.params).toEqual({ foo: 1 })
    expect(frame.id).toBeTruthy()

    sock._recv({ id: frame.id, status: 'ok', data: { nodes: ['a'] } })
    await expect(p).resolves.toEqual({ nodes: ['a'] })
  })

  it('rejects a request when the response status is not ok', async () => {
    const { store, sock } = openStore()
    const p = store.control('estop')
    const frame = sock.lastSent()
    sock._recv({ id: frame.id, status: 'error', message: 'boom' })
    await expect(p).rejects.toThrow('boom')
  })

  it('builds the command frame with target merged into params', async () => {
    const { store, sock } = openStore()
    store.command('node-7', 'set', { value: 0.5 })
    const frame = sock.lastSent()
    expect(frame.type).toBe('command')
    expect(frame.action).toBe('set')
    expect(frame.params).toEqual({ target: 'node-7', value: 0.5 })
  })

  it('stashes the latest state frame per topic in the reactive map', () => {
    const { store, sock } = openStore()
    sock._recv({ type: 'state', node: 'pin_state/n1', data: { soc: 82 } })
    expect(store.topics.get('pin_state/n1')).toEqual({ soc: 82 })
    sock._recv({ type: 'state', node: 'pin_state/n1', data: { soc: 81 } })
    expect(store.topics.get('pin_state/n1')).toEqual({ soc: 81 })
  })

  it('schedules an exponential-backoff reconnect on an unexpected drop', () => {
    vi.useFakeTimers()
    const { store, sock } = openStore()
    sock._drop()
    expect(store.connected).toBe(false)
    expect(store.reconnectAttempts).toBe(1) // first retry queued

    vi.advanceTimersByTime(1000) // reconnectBase * 2^0
    expect(sockets).toHaveLength(2) // connect() ran again
  })

  it('gives up after the max attempts with a reconnect_failed event', () => {
    const { store, sock } = openStore()
    const failed = vi.fn()
    store.on('reconnect_failed', failed)
    store.reconnectAttempts = 10 // already at the cap
    sock._drop()
    expect(failed).toHaveBeenCalledOnce()
  })

  it('disconnect suppresses auto-reconnect', () => {
    const { store, sock } = openStore()
    store.disconnect()
    expect(store.connected).toBe(false)
    expect(store.reconnectAttempts).toBe(10) // pinned to max → no retry
  })
})
