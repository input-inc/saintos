#!/usr/bin/env node
// Dev-only mock SAINT.OS WebSocket server.
//
// Lets the Vue frontend run without the real Pi backend:
//   1. `npm run mock`                      # this script, listens on 8081
//   2. `SAINT_HOST=http://localhost:8081 npm run dev`
//   3. browse http://localhost:5173
//
// Speaks the same JSON protocol as
// server/saint_server/webserver/websocket_handler.py — see the README
// in this directory for the action inventory + topic shapes.

import http from 'node:http'
import { randomUUID } from 'node:crypto'
import { WebSocketServer } from 'ws'

import * as st from './mock-state.js'
import {
  managementHandlers, handleCommand, handleRouter, handleControl, handleGeneric,
} from './mock-handlers.js'
import { handleHttp } from './mock-http.js'

const PORT = parseInt(process.env.MOCK_PORT || '8081', 10)

// ── HTTP server (landing page + WebSocket upgrade) ───────────────────

const server = http.createServer(async (req, res) => {
  if (req.url === '/' || req.url === '/index.html') {
    res.writeHead(200, { 'Content-Type': 'text/html; charset=utf-8' })
    res.end(LANDING_HTML)
    return
  }
  if (req.url === '/api/health' || req.url === '/api/ping') {
    res.writeHead(200, { 'Content-Type': 'application/json' })
    res.end(JSON.stringify({ ok: true, server: 'SAINT.OS mock', port: PORT }))
    return
  }
  // Animation builder HTTP surface — URDF upload/serve, Maestro import.
  // Lives in mock-http.js so the URDF/zip/multipart plumbing stays out
  // of this file's WebSocket flow.
  try {
    const handled = await handleHttp(req, res)
    if (handled) return
  } catch (e) {
    console.error('[mock] HTTP handler crashed:', e)
    if (!res.writableEnded) {
      res.writeHead(500, { 'Content-Type': 'application/json' })
      res.end(JSON.stringify({ error: e.message || 'Internal error' }))
    }
    return
  }
  res.writeHead(404, { 'Content-Type': 'text/plain' })
  res.end('Mock server: route not found.\n')
})

// WS endpoint matches the real server: /api/ws
const wss = new WebSocketServer({ noServer: true })

server.on('upgrade', (req, socket, head) => {
  if (req.url === '/api/ws') {
    wss.handleUpgrade(req, socket, head, (ws) => wss.emit('connection', ws, req))
  } else {
    socket.destroy()
  }
})

// ── Per-client lifecycle + dispatch ──────────────────────────────────

wss.on('connection', (ws) => {
  const clientId = randomUUID().slice(0, 8)
  const client = {
    id: clientId, ws,
    subscriptions: new Set(),
    connected_at: st.nowSec(),
  }
  st.clients.set(clientId, client)

  // Hello frame. auth_required=false so the Vue store auto-flips
  // `authenticated` and emits `ready`.
  sendJson(ws, {
    type: 'connected',
    client_id: clientId,
    server_name: 'SAINT.OS (Mock)',
    auth_required: false,
  })

  ws.on('message', (raw) => onMessage(client, raw))
  ws.on('close', () => st.clients.delete(clientId))
  ws.on('error', (e) => console.warn(`[mock] ws error for ${clientId}:`, e.message))
})

function onMessage (client, raw) {
  let msg
  try { msg = JSON.parse(raw.toString('utf8')) }
  catch { return sendJson(client.ws, { status: 'error', message: 'Invalid JSON' }) }

  // Auth: real server replies with type:auth_result. Mock has
  // auth_required=false but we still ack login so the UI's Login screen
  // works if it pops up (e.g. after `set_settings` adds a password).
  if (msg.type === 'auth') {
    sendJson(client.ws, { type: 'auth_result', status: 'ok', message: 'Mock auth ok' })
    return
  }

  if (!msg.id) return sendJson(client.ws, { status: 'error', message: 'Missing message id' })

  const ctx = {
    clientId: client.id,
    client,
    broadcast: (topic, data) => broadcast(topic, data),
    activity: (text, level = 'info') => broadcastActivity(text, level),
  }
  const params = msg.params || {}

  // Subscriptions are tracked per-client so broadcasts only land where
  // they were asked for — same semantics as the real handler.
  if (msg.type === 'subscribe') {
    const topics = params.topics || []
    if (msg.action === 'subscribe') {
      for (const t of topics) client.subscriptions.add(t)
    } else if (msg.action === 'unsubscribe') {
      for (const t of topics) client.subscriptions.delete(t)
    }
    return sendJson(client.ws, { id: msg.id, status: 'ok',
                                  data: { topics: [...client.subscriptions] } })
  }

  let result
  try {
    if (msg.type === 'management') {
      const h = managementHandlers[msg.action]
      result = h ? h(params, ctx) : { ok: false, message: `Unknown action: ${msg.action}` }
    } else if (msg.type === 'command') {
      result = handleCommand(msg.action, params, ctx)
    } else if (msg.type === 'router') {
      result = handleRouter(msg.action, params, ctx)
    } else if (msg.type === 'control') {
      result = handleControl(msg.action, params, ctx)
    } else {
      result = handleGeneric(msg.type, msg.action, params, ctx)
    }
  } catch (e) {
    console.error(`[mock] handler ${msg.type}/${msg.action} threw:`, e)
    result = { ok: false, message: e.message || 'Handler error' }
  }

  if (!result) result = { ok: true, data: {} }
  if (result.ok) {
    sendJson(client.ws, { id: msg.id, status: 'ok', data: result.data })
  } else {
    sendJson(client.ws, { id: msg.id, status: 'error', message: result.message })
  }
}

function sendJson (ws, obj) {
  if (ws.readyState !== ws.OPEN) return
  try { ws.send(JSON.stringify(obj)) }
  catch (e) { console.warn('[mock] send failed:', e.message) }
}

// ── Broadcast helpers ────────────────────────────────────────────────

function broadcast (topic, data) {
  const frame = JSON.stringify({ type: 'state', node: topic, data })
  for (const c of st.clients.values()) {
    if (c.ws.readyState !== c.ws.OPEN) continue
    if (!c.subscriptions.has(topic)) continue
    try { c.ws.send(frame) } catch (_) {}
  }
}

function broadcastActivity (text, level = 'info') {
  // The real server sends activity to ALL clients regardless of
  // subscriptions — mirror that here.
  const frame = JSON.stringify({ type: 'activity', text, level, timestamp: st.nowSec() })
  for (const c of st.clients.values()) {
    if (c.ws.readyState !== c.ws.OPEN) continue
    try { c.ws.send(frame) } catch (_) {}
  }
}

// ── Live-data simulation tick ────────────────────────────────────────
//
// Cadences are chosen to roughly match what the real server emits:
//   system_status      — every 2 s
//   pin_state/host_*   — every 1 s (host CPU/mem/temp + WiFi metrics)
//   pin_state/<node>   — every 250 ms per adopted node
//   routing_values     — every 500 ms (derived from pin_state)
//   activity           — random log entry every 5–15 s

// Track a baseline value per channel so wandering looks smooth instead
// of pure white noise. peripheralBase[node][periph][channel] = value.
const peripheralBase = {}
function jitter (key, mean, span) {
  const cur = peripheralBase[key] ?? mean
  // Mean-reverting random walk: pull 5% toward mean, add ±span% noise.
  const next = cur * 0.95 + mean * 0.05 + (Math.random() - 0.5) * span
  peripheralBase[key] = next
  return next
}

function tickSystemStatus () {
  st.live.hostMonitor.cpu_usage = clamp(jitter('cpu', 12, 4), 3, 40)
  st.live.hostMonitor.mem_usage = clamp(jitter('mem', 50, 2), 35, 70)
  st.live.hostMonitor.cpu_temp  = clamp(jitter('temp', 52, 2), 42, 75)
  // Occasional 75°C spikes to exercise the dashboard color thresholds.
  if (Math.random() < 0.05) st.live.hostMonitor.cpu_temp = 70 + Math.random() * 10
  st.live.hostMonitor.uptime   += 2

  st.live.hostMonitor.wifi_signal    = clamp(jitter('wifi_sig',  -60, 3),  -80, -40)
  st.live.hostMonitor.wifi_retry_pct = clamp(jitter('wifi_retry', 3,  2),   0,  15)
  st.live.hostMonitor.wifi_noise     = clamp(jitter('wifi_noise', -95, 1), -100, -85)
  st.live.hostMonitor.wifi_bitrate   = clamp(jitter('wifi_br',    90, 8),  60, 150)

  broadcast('system_status', {
    cpu_usage: round(st.live.hostMonitor.cpu_usage, 1),
    memory_usage: round(st.live.hostMonitor.mem_usage, 1),
    cpu_temp_c: round(st.live.hostMonitor.cpu_temp, 1),
    uptime_seconds: Math.floor(st.live.hostMonitor.uptime),
    server_name: 'SAINT.OS (Mock)',
    throttle: { raw: '0x0', status: 'ok', summary: 'No throttling',
                flags: [], descriptions: [] },
    adopted_node_count: st.adoptedNodes.length,
    unadopted_node_count: st.unadoptedNodes.length,
    websocket_client_count: st.clients.size,
  })

  // Also publish a fresh adopted_nodes/unadopted_nodes broadcast every
  // few ticks so nodes-store stays in sync without an explicit refresh.
  broadcast('adopted_nodes', { nodes: st.adoptedListSnapshot() })
  broadcast('unadopted_nodes', { nodes: st.unadoptedNodes })
}

function tickHostPinState () {
  // host_controller pin_state surfaces the system_monitor channels —
  // Dashboard.vue reads wifi_* from this topic, NOT from system_status.
  const hm = st.live.hostMonitor
  const channels = [
    { peripheral_id: 'system_monitor', channel_id: 'cpu_usage',     value: hm.cpu_usage,    last_updated: st.nowSec() },
    { peripheral_id: 'system_monitor', channel_id: 'cpu_temp',      value: hm.cpu_temp,     last_updated: st.nowSec() },
    { peripheral_id: 'system_monitor', channel_id: 'mem_usage',     value: hm.mem_usage,    last_updated: st.nowSec() },
    { peripheral_id: 'system_monitor', channel_id: 'uptime',        value: hm.uptime,       last_updated: st.nowSec() },
    { peripheral_id: 'system_monitor', channel_id: 'wifi_signal',   value: hm.wifi_signal,  last_updated: st.nowSec() },
    { peripheral_id: 'system_monitor', channel_id: 'wifi_retry_pct',value: hm.wifi_retry_pct, last_updated: st.nowSec() },
    { peripheral_id: 'system_monitor', channel_id: 'wifi_noise',    value: hm.wifi_noise,   last_updated: st.nowSec() },
    { peripheral_id: 'system_monitor', channel_id: 'wifi_bitrate',  value: hm.wifi_bitrate, last_updated: st.nowSec() },
  ]
  broadcast('pin_state/host_controller', {
    node_id: 'host_controller', pins: [], channels,
    last_feedback: st.nowSec(), stale: false,
  })
}

function tickNodePinStates () {
  // Per adopted node, walk its peripherals and synthesize channel
  // readings for any "in" direction channels. Stored in
  // live.peripheralValues so routing_values can derive consistent
  // sheet values from the same source.
  for (const node of st.adoptedNodes) {
    const pc = st.nodePeripherals[node.node_id]
    if (!pc) continue
    const channels = []
    const nodeValues = st.live.peripheralValues[node.node_id] || {}
    st.live.peripheralValues[node.node_id] = nodeValues

    for (const periph of pc.peripherals) {
      const ptype = st.peripheralCatalog.find(t => t.id === periph.type)
      if (!ptype) continue
      const periphValues = nodeValues[periph.id] || {}
      nodeValues[periph.id] = periphValues

      for (const ch of ptype.channels) {
        const key = `${node.node_id}/${periph.id}/${ch.id}`
        let value
        if (ch.dir === 'in') {
          // Synthesize plausible readings by channel id.
          value = synthesizeChannel(periph.type, ch.id, key)
        } else {
          // Echo back whatever the last write was (or 0).
          value = periphValues[ch.id] ?? 0
        }
        periphValues[ch.id] = value
        channels.push({ peripheral_id: periph.id, channel_id: ch.id,
                        value: round(value, 3), last_updated: st.nowSec() })
      }
    }
    broadcast(`pin_state/${node.node_id}`, {
      node_id: node.node_id, pins: [], channels,
      last_feedback: st.nowSec(), stale: false,
    })
  }
}

function synthesizeChannel (type, channelId, key) {
  if (type === 'roboclaw') {
    if (channelId === 'encoder') return jitter(key, 1500, 30)
    if (channelId === 'voltage') return jitter(key, 24.5, 0.1)
    if (channelId === 'current') return Math.abs(jitter(key, 3.5, 0.6))
    if (channelId === 'temp')    return jitter(key, 38, 0.5)
  }
  if (type === 'fas100') {
    if (channelId === 'amps')  return Math.abs(jitter(key, 5, 0.5))
    if (channelId === 'volts') return jitter(key, 12.4, 0.05)
    if (channelId === 'temp1') return jitter(key, 35, 0.5)
    if (channelId === 'temp2') return jitter(key, 33, 0.5)
  }
  if (type === 'button') return Math.random() < 0.02 ? 1 : 0
  if (type === 'analog_in') return jitter(key, 1.65, 0.05)
  return jitter(key, 0.5, 0.1)
}

function tickRoutingValues () {
  // routing_values mirrors the per-sheet evaluator output. The Vue
  // routes view reads sheets[<id>].peripherals[<node>/<peripheral>/<channel>]
  // among others. Derive what we can from the channel readings we just
  // published; widget values come from following wires.
  const sheetsOut = {}
  for (const [sheetId, sheet] of Object.entries(st.systemRouting.sheets)) {
    const peripherals = {}
    const widgets = {}
    const operators = {}
    const ws_inputs = {}
    const inputs = {}
    const outputs = {}

    // Peripheral readings: same value the pin_state broadcast published.
    for (const [nodeId, perips] of Object.entries(st.live.peripheralValues)) {
      for (const [pid, channels] of Object.entries(perips)) {
        for (const [chId, v] of Object.entries(channels)) {
          peripherals[`${nodeId}/${pid}/${chId}`] = round(v, 3)
        }
      }
    }
    // URDF-joint input values: each input with kind="urdf_joint"
    // pulls its current setpoint from the joint cache (populated by
    // active animation players).
    for (const inp of (sheet.inputs || [])) {
      if (inp.kind === 'urdf_joint') {
        const v = st.getUrdfJointValue(inp.joint)
        if (v != null) inputs[inp.id] = round(v, 3)
      }
    }
    // Widget input values: follow wires whose sink is "widget".
    for (const w of sheet.wires) {
      if (w.sink.kind !== 'widget') continue
      const [widgetId, inputId] = w.sink.parts
      const value = resolveEndpoint(w.source)
      if (value != null) widgets[`${widgetId}/${inputId}`] = round(value, 3)
    }
    // ws_inputs: keep last written value if we tracked one; default 0.
    for (const wi of sheet.ws_inputs) ws_inputs[wi.id] = 0
    // Operator outputs: just echo "the input" for scale/gain — good
    // enough for the value-pill animation on the canvas.
    for (const op of sheet.operators) operators[op.id] = 0

    sheetsOut[sheetId] = { inputs, ws_inputs, outputs, operators, widgets, peripherals }
  }
  broadcast('routing_values', { sheets: sheetsOut })
}

// Animation playback ticker. Runs at 30 Hz, advances `t` for each
// active player by the actual elapsed wall time (so pausing the
// scheduler doesn't drop frames), samples value tracks into
// st.live_animation_values, and stops the player when it crosses
// `duration` (or loops if anim.loop is set).
const ANIM_TICK_HZ = 30
let _animLastTick = Date.now()
function tickAnimations () {
  const now = Date.now()
  const dt = Math.max(0, (now - _animLastTick) / 1000.0)
  _animLastTick = now
  for (const player of [...st.animationPlayers.values()]) {
    if (player.paused || !player.running) continue
    const anim = st.animations.get(player.id)
    if (!anim) {
      st.animationPlayers.delete(player.id)
      continue
    }
    player.last_t = player.t
    player.t += dt
    if (anim.duration > 0 && player.t >= anim.duration) {
      if (anim.loop) {
        player.t = 0
        player.last_t = 0
      } else {
        // Land the final frame at duration then stop.
        for (const track of anim.value_tracks || []) {
          st.setUrdfJointValue(anim.id, track.id,
                               st.sampleCurve(track.curve, anim.duration))
        }
        st.animationPlayers.delete(player.id)
        continue
      }
    }
    for (const track of anim.value_tracks || []) {
      // Track id is the URDF joint name: any input with
      // kind="urdf_joint" and matching `joint` field reads this value
      // out of the cache.
      st.setUrdfJointValue(anim.id, track.id,
                           st.sampleCurve(track.curve, player.t))
    }
  }
}

// Resolve a wire endpoint to a live scalar — used for widget value pills.
function resolveEndpoint (ep) {
  if (ep.kind === 'peripheral') {
    const [nodeId, pid, chId] = ep.parts
    return st.live.peripheralValues?.[nodeId]?.[pid]?.[chId] ?? null
  }
  return null
}

function tickActivity () {
  const samples = [
    ['info', 'Heartbeat from controller'],
    ['info', 'Routing evaluator tick'],
    ['warn', 'Roboclaw retry — packet checksum'],
    ['info', 'Battery 24.5V nominal'],
    ['info', 'WiFi survey requested by dashboard'],
  ]
  const [level, text] = samples[Math.floor(Math.random() * samples.length)]
  broadcastActivity(text, level)
}

// ── Scheduling ───────────────────────────────────────────────────────

function schedule (fn, ms, label) {
  setInterval(() => {
    try { fn() } catch (e) { console.warn(`[mock] ${label} tick failed:`, e.message) }
  }, ms)
}

server.listen(PORT, () => {
  console.log(`[mock] SAINT.OS mock server listening on http://localhost:${PORT}`)
  console.log(`[mock] WebSocket: ws://localhost:${PORT}/api/ws`)
  console.log(`[mock] Point Vite at it:`)
  console.log(`[mock]   SAINT_HOST=http://localhost:${PORT} npm run dev`)
  schedule(tickSystemStatus,  2000, 'system_status')
  schedule(tickHostPinState,  1000, 'pin_state/host_controller')
  schedule(tickNodePinStates,  250, 'pin_state/<node>')
  schedule(tickAnimations,  1000 / ANIM_TICK_HZ, 'animation_playback')
  schedule(tickRoutingValues,  500, 'routing_values')
  // Activity is irregular — jitter between 5 and 15 s.
  ;(function activityLoop () {
    setTimeout(() => { tickActivity(); activityLoop() }, 5000 + Math.random() * 10000)
  })()
})

// ── Utility ──────────────────────────────────────────────────────────

function clamp (v, lo, hi) { return Math.max(lo, Math.min(hi, v)) }
function round (v, dp = 2) { const k = 10 ** dp; return Math.round(v * k) / k }

const LANDING_HTML = `<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8" />
  <title>SAINT.OS Mock Server</title>
  <style>
    body { font-family: ui-sans-serif, system-ui, sans-serif; background: #0f172a;
           color: #e2e8f0; margin: 0; min-height: 100vh; display: grid;
           place-items: center; }
    .card { max-width: 640px; padding: 32px; background: #1e293b;
            border: 1px solid #334155; border-radius: 12px; }
    h1 { margin-top: 0; color: #38bdf8; }
    code { background: #0f172a; padding: 2px 6px; border-radius: 4px;
           color: #f1f5f9; }
    pre { background: #0f172a; padding: 16px; border-radius: 8px;
          overflow-x: auto; }
    a { color: #38bdf8; }
  </style>
</head>
<body>
  <div class="card">
    <h1>SAINT.OS Mock Server</h1>
    <p>This is the dev-only mock backend for the Vue operator UI. It is
    not the real SAINT.OS server — there is no hardware behind these
    APIs and no persistence between runs.</p>
    <p>WebSocket endpoint: <code>/api/ws</code></p>
    <p>Run the Vue dev server in another terminal:</p>
    <pre>SAINT_HOST=http://localhost:${PORT} npm run dev</pre>
    <p>Then browse to <a href="http://localhost:5173">http://localhost:5173</a>.</p>
  </div>
</body>
</html>`
