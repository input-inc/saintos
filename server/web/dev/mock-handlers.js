// Management-action handlers for the dev mock WebSocket server.
//
// Each handler receives (params, ctx) and returns one of:
//   { ok: true, data }                    → wrapped into a success reply
//   { ok: false, message }                → wrapped into an error reply
//   undefined                             → also success, with data: {}
//
// Mutating handlers may emit broadcasts via ctx.broadcast(topic, data).

import * as st from './mock-state.js'

// ── Registry ─────────────────────────────────────────────────────────

export const managementHandlers = {
  // Node lifecycle ─────────────────────────────────────────────────
  list_adopted:        () => ok({ nodes: st.adoptedListSnapshot() }),
  list_unadopted:      () => ok({ nodes: st.unadoptedNodes }),
  list_clients:        (_, ctx) => ok({ clients: [...st.clients.values()].map(c => ({
    id: c.id, ip_address: '127.0.0.1', user_agent: 'mock-browser',
    client_name: null, connected_at: c.connected_at,
    authenticated: true, is_self: c.id === ctx.clientId,
  })) }),
  disconnect_client:   ({ client_id }, ctx) => {
    const c = st.clients.get(client_id)
    if (!c) return err('Client not found')
    if (client_id === ctx.clientId) return err('Cannot disconnect yourself')
    try { c.ws.close() } catch (_) {}
    return ok({ message: 'Client disconnected' })
  },
  scan_nodes:          () => ok({ scanning: true }),
  adopt_node:          ({ node_id, role, display_name, board_id, chip_family }, ctx) => {
    const idx = st.unadoptedNodes.findIndex(n => n.node_id === node_id)
    if (idx < 0) return err(`Node ${node_id} not found`)
    const u = st.unadoptedNodes.splice(idx, 1)[0]
    const adopted = {
      ...u,
      display_name: display_name || `${role} Node`,
      role,
      chip_family: chip_family || u.chip_family,
      board_id: board_id || (u.chip_family === 'rp2040' ? 'feather_rp2040' : 'teensy41_default'),
      state: 'ACTIVE', online: true, has_capabilities: true,
      peripheral_count: 0, peripheral_sync_status: 'synced',
      firmware_update_available: false, firmware_update_message: 'up to date',
      server_firmware_version: '0.5.0',
      server_firmware_build: '2026-05-25 14:30:00',
      server_firmware_hash: 'mock1234',
    }
    st.adoptedNodes.push(adopted)
    st.nodePeripherals[node_id] = { version: 0, sync_status: 'synced',
                                    last_synced: st.nowSec(), peripherals: [] }
    ctx.broadcast('adopted_nodes', { nodes: st.adoptedListSnapshot() })
    ctx.broadcast('unadopted_nodes', { nodes: st.unadoptedNodes })
    ctx.activity(`Node ${node_id} adopted as ${role}`, 'info')
    return ok({ success: true, message: 'Adopted', assigned_topic_prefix: `/saint/${role}`,
                board_id: adopted.board_id })
  },
  update_node:         (params, ctx) => {
    const n = st.findAdopted(params.node_id)
    if (!n) return err(`Node ${params.node_id} not found`)
    for (const k of ['role', 'display_name', 'board_id', 'chip_family']) {
      if (params[k] != null) n[k] = params[k]
    }
    ctx.broadcast('adopted_nodes', { nodes: st.adoptedListSnapshot() })
    ctx.broadcast('update', { node_id: n.node_id, kind: 'node_updated' })
    return ok({ success: true })
  },
  remove_node:         ({ node_id }, ctx) => {
    const idx = st.adoptedNodes.findIndex(n => n.node_id === node_id)
    if (idx < 0) return err(`Node ${node_id} not found`)
    st.adoptedNodes.splice(idx, 1)
    delete st.nodePeripherals[node_id]
    delete st.systemRouting.sheets[node_id]
    st.systemRouting.version++
    ctx.broadcast('adopted_nodes', { nodes: st.adoptedListSnapshot() })
    ctx.broadcast('system_routing', st.systemRouting)
    ctx.activity(`Node ${node_id} removed`, 'info')
    return ok({ success: true })
  },
  restart_node:        ({ node_id }) => ok({ message: `Restart sent to ${node_id} (mock)` }),
  identify_node:       ({ node_id }) => ok({ message: `Identifying ${node_id} (mock)` }),

  // Boards / chips / roles ─────────────────────────────────────────
  list_chips:          () => ok({ chips: st.chips }),
  list_boards:         ({ chip_family } = {}) => ok({
    boards: chip_family ? st.boards.filter(b => b.chip_family === chip_family) : st.boards,
  }),
  get_board_yaml:      ({ board_id }) => {
    const b = st.boards.find(x => x.board_id === board_id)
    if (!b) return err(`Unknown board '${board_id}'`)
    return ok({ board_id, yaml: `# Mock YAML for ${board_id}\nboard_id: ${b.board_id}\n` +
                                `display_name: ${b.display_name}\nchip_family: ${b.chip_family}\n` +
                                `builtin: ${b.builtin}\n` })
  },
  save_board:          ({ yaml }) => ok({ success: true, board_id: 'custom_board' }),
  delete_board:        ({ board_id }) => {
    const idx = st.boards.findIndex(b => b.board_id === board_id)
    if (idx < 0) return err('Board not found')
    if (st.boards[idx].builtin) return err('Cannot delete built-in board')
    st.boards.splice(idx, 1)
    return ok({ success: true })
  },
  get_roles:           () => ok({ roles: st.roles }),

  // Capabilities + per-node peripherals ────────────────────────────
  get_node_capabilities: ({ node_id }) => {
    const n = st.findAdopted(node_id) || st.findUnadopted(node_id)
    if (!n) return err('Node not found')
    return ok({
      node_id, chip_family: n.chip_family, board_id: n.board_id,
      pins: rangePins(n.chip_family),
      reserved_pins: [], uart_pairs: [{ tx: 0, rx: 1 }, { tx: 4, rx: 5 }],
      builtin_peripherals: [],
    })
  },
  get_peripheral_catalog: () => ok({
    peripheral_types: st.peripheralCatalog,
    widget_types: st.widgetCatalog,
    operator_types: st.operatorCatalog,
  }),
  // Flat enumeration of WS inputs across every sheet — powers the
  // trigger-keyframe target picker in the animation builder.
  list_ws_inputs: () => {
    const out = []
    for (const [sheetId, sheet] of Object.entries(st.systemRouting.sheets)) {
      for (const wsi of (sheet.ws_inputs || [])) {
        out.push({
          sheet_id: sheetId,
          sheet_label: sheetId,
          input_id: wsi.id,
          label: wsi.label || wsi.id,
          kind: wsi.kind || 'command',
        })
      }
    }
    return ok({ ws_inputs: out })
  },
  get_node_peripherals: ({ node_id }) => {
    const pc = st.nodePeripherals[node_id]
    if (!pc) return err(`Node ${node_id} not found`)
    return ok(pc)
  },
  save_node_peripheral: ({ node_id, peripheral }, ctx) => {
    const pc = st.nodePeripherals[node_id]
    if (!pc) return err(`Node ${node_id} not found`)
    if (!peripheral || typeof peripheral !== 'object') {
      return err('Missing peripheral payload (expected { peripheral: { type, label, pins, params } })')
    }
    if (!peripheral.type) return err('Peripheral type is required')
    const pid = peripheral.id ||
      `${peripheral.type}-${pc.peripherals.filter(p => p.type === peripheral.type).length + 1}`
    const idx = pc.peripherals.findIndex(p => p.id === pid)
    const merged = { id: pid, type: peripheral.type, label: peripheral.label || peripheral.type,
                     pins: peripheral.pins || {}, params: peripheral.params || {},
                     builtin: !!peripheral.builtin, log_enabled: !!peripheral.log_enabled }
    if (idx >= 0) pc.peripherals[idx] = merged
    else pc.peripherals.push(merged)
    pc.version++; pc.sync_status = 'synced'; pc.last_synced = st.nowSec()
    ctx.broadcast('update', { node_id, kind: 'peripherals' })
    return ok({ success: true, peripheral: merged, version: pc.version })
  },
  remove_node_peripheral: ({ node_id, peripheral_id }, ctx) => {
    const pc = st.nodePeripherals[node_id]
    if (!pc) return err('Node not found')
    const idx = pc.peripherals.findIndex(p => p.id === peripheral_id)
    if (idx < 0) return err(`Peripheral ${peripheral_id} not found`)
    pc.peripherals.splice(idx, 1)
    pc.version++
    ctx.broadcast('update', { node_id, kind: 'peripherals' })
    return ok({ success: true })
  },
  set_peripheral_log_enabled: ({ node_id, peripheral_id, enabled }) => {
    const p = st.nodePeripherals[node_id]?.peripherals.find(x => x.id === peripheral_id)
    if (!p) return err('Peripheral not found')
    p.log_enabled = !!enabled
    return ok({ success: true, log_enabled: p.log_enabled })
  },
  get_peripheral_history: ({ node_id, peripheral_id, channel_id, window = '30s' }) => {
    // Generate a fake sample series so the channel-history chart isn't empty.
    const seconds = window === '60s' ? 60 : 30
    const samples = []
    const now = st.nowSec()
    for (let i = seconds * 10; i > 0; i--) {
      samples.push({ ts: now - i * 0.1,
                     v: 0.5 + 0.4 * Math.sin(i / 7) + (Math.random() - 0.5) * 0.05 })
    }
    return ok({ node_id, peripheral_id, channel_id, window, samples })
  },
  sync_node_peripherals: ({ node_id }) => {
    const pc = st.nodePeripherals[node_id]
    if (!pc) return err('No configuration to sync')
    pc.sync_status = 'synced'; pc.last_synced = st.nowSec()
    return ok({ message: 'Sync simulated', success: true })
  },

  // Routing (system_routing graph) ─────────────────────────────────
  get_system_routing:  () => ok(st.systemRouting),
  add_routing_input:   ({ node_id, topic = '', field = '', joint = '',
                          kind = 'topic', label = '',
                          position = [40, 40] }, ctx) => {
    const sheet = ensureSheet(node_id)
    const id = nextId(sheet.inputs, 'in')
    // Default label: joint name for urdf_joint, else topic.field.
    const fallback = kind === 'urdf_joint'
      ? (joint || 'joint')
      : `${topic}${field ? '.' + field : ''}`
    sheet.inputs.push({
      id, topic, field, joint, kind,
      label: label || fallback, position,
    })
    bumpRouting(ctx)
    return ok({ id })
  },
  add_routing_ws_input: ({ node_id, label = '', position = [40, 40] }, ctx) => {
    const sheet = ensureSheet(node_id)
    const id = nextId(sheet.ws_inputs, 'wsin')
    sheet.ws_inputs.push({ id, label, position, kind: 'command' })
    bumpRouting(ctx)
    return ok({ id })
  },
  add_routing_output:  ({ node_id, topic, field = '', label = '', position = [600, 40] }, ctx) => {
    const sheet = ensureSheet(node_id)
    const id = nextId(sheet.outputs, 'out')
    sheet.outputs.push({ id, topic, field, label: label || `${topic}.${field}`, position })
    bumpRouting(ctx)
    return ok({ id })
  },
  add_routing_operator: ({ node_id, op, label = '', params = {}, defaults = {}, position = [320, 40] }, ctx) => {
    const sheet = ensureSheet(node_id)
    const id = nextId(sheet.operators, 'op')
    sheet.operators.push({ id, op, label, params, defaults, position })
    bumpRouting(ctx)
    return ok({ id })
  },
  add_routing_wire:    ({ node_id, source, sink }, ctx) => {
    const sheet = ensureSheet(node_id)
    // Last-writer-wins on sink — drop any existing wire feeding the same sink.
    sheet.wires = sheet.wires.filter(w =>
      !(w.sink.kind === sink.kind && JSON.stringify(w.sink.parts) === JSON.stringify(sink.parts)))
    const id = nextId(sheet.wires, 'w')
    sheet.wires.push({ id, source, sink })
    bumpRouting(ctx)
    return ok({ id })
  },
  remove_routing_wire: ({ node_id, wire_id }, ctx) => {
    const sheet = st.systemRouting.sheets[node_id]
    if (!sheet) return err('Sheet not found')
    const before = sheet.wires.length
    sheet.wires = sheet.wires.filter(w => w.id !== wire_id)
    if (sheet.wires.length === before) return err('Wire not found')
    bumpRouting(ctx)
    return ok({ success: true })
  },
  update_sheet_node:   ({ node_id, sheet_node_id, ...changes }, ctx) => {
    const sheet = st.systemRouting.sheets[node_id]
    if (!sheet) return err('Sheet not found')
    const target = findSheetNode(sheet, sheet_node_id)
    if (!target) return err('Sheet node not found')
    for (const k of ['label', 'params', 'defaults', 'position', 'topic', 'field', 'kind', 'joint']) {
      if (changes[k] !== undefined) target[k] = changes[k]
    }
    bumpRouting(ctx)
    return ok({ success: true })
  },
  remove_sheet_node:   ({ node_id, sheet_node_id }, ctx) => {
    const sheet = st.systemRouting.sheets[node_id]
    if (!sheet) return err('Sheet not found')
    for (const list of ['inputs', 'ws_inputs', 'outputs', 'operators', 'widgets']) {
      if (!Array.isArray(sheet[list])) continue
      const idx = sheet[list].findIndex(x => x.id === sheet_node_id)
      if (idx >= 0) {
        sheet[list].splice(idx, 1)
        sheet.wires = sheet.wires.filter(w =>
          !(w.source.parts?.[0] === sheet_node_id || w.sink.parts?.[0] === sheet_node_id))
        bumpRouting(ctx)
        return ok({ success: true })
      }
    }
    return err('Sheet node not found')
  },
  add_widget:          ({ node_id, type, label, position = [40, 40], params = {} }, ctx) => {
    const sheet = ensureSheet(node_id)
    // Widget IDs must be unique across all sheets (dashboard keys DOM
    // nodes off bare widget IDs). Pick the next free <type>-N globally.
    const allIds = new Set()
    for (const s of Object.values(st.systemRouting.sheets)) {
      for (const w of s.widgets) allIds.add(w.id)
    }
    let n = 1
    while (allIds.has(`${type}-${n}`)) n++
    const id = `${type}-${n}`
    sheet.widgets.push({ id, type, label: label || type, position, params })
    bumpRouting(ctx)
    return ok({ id })
  },

  // Settings ───────────────────────────────────────────────────────
  get_settings:        () => ok(st.settings),
  set_settings:        ({ settings }, ctx) => {
    if (!settings) return err('Missing settings data')
    deepMerge(st.settings, settings)
    ctx.activity('Server settings updated', 'info')
    return ok({ restart_required: true })
  },

  // Logs ───────────────────────────────────────────────────────────
  get_logs:            ({ limit = 100 } = {}) => ok({ logs: st.serverLogs.slice(-limit) }),
  get_node_logs:       ({ node_id }) => ok({ entries: st.nodeLogs[node_id] || [] }),
  clear_node_logs:     ({ node_id }) => { st.nodeLogs[node_id] = []; return ok({ success: true }) },

  // WiFi ───────────────────────────────────────────────────────────
  wifi_get_config:     () => ok(st.wifi),
  wifi_survey:         () => ok({
    ok: true, iface: 'wlan0', current_channel: st.wifi.channel,
    channels: [
      // A handful of plausible 5 GHz channels — sort order matches real
      // server (current first, then by ap_count asc, then signal asc).
      { band: '5', channel: 44, freq_mhz: 5220, ap_count: 0,
        strongest_signal_dbm: null, is_dfs: false, is_current: true },
      { band: '5', channel: 36, freq_mhz: 5180, ap_count: 1,
        strongest_signal_dbm: -78, is_dfs: false, is_current: false },
      { band: '5', channel: 149, freq_mhz: 5745, ap_count: 2,
        strongest_signal_dbm: -65, is_dfs: false, is_current: false },
      { band: '5', channel: 52, freq_mhz: 5260, ap_count: 0,
        strongest_signal_dbm: null, is_dfs: true, is_current: false },
      { band: '2', channel: 1, freq_mhz: 2412, ap_count: 4,
        strongest_signal_dbm: -55, is_dfs: false, is_current: false },
      { band: '2', channel: 11, freq_mhz: 2462, ap_count: 6,
        strongest_signal_dbm: -50, is_dfs: false, is_current: false },
    ],
  }),
  wifi_set_channel:    ({ band, channel }, ctx) => {
    st.wifi.band = band
    st.wifi.channel = parseInt(channel, 10)
    ctx.activity(`Switching AP to ${band} channel ${channel}`, 'warn')
    return ok({ switching: true, band, channel: st.wifi.channel })
  },
  wifi_set_credentials: ({ ssid, password }, ctx) => {
    if (typeof ssid !== 'string' || typeof password !== 'string')
      return err('ssid and password must be strings')
    if (ssid.length < 1 || ssid.length > 32) return err('SSID must be 1–32 chars')
    if (password.length < 8 || password.length > 63) return err('Password must be 8–63 chars')
    st.wifi.ssid = ssid
    st.wifi.password = password
    ctx.activity(`Updating AP credentials (SSID=${ssid})`, 'warn')
    return ok({ switching: true, ssid })
  },

  // Firmware ───────────────────────────────────────────────────────
  get_firmware_builds: () => ok(st.firmwareBuilds),
  update_firmware:     ({ node_id }) => ok({
    message: 'Firmware update initiated (mock)',
    from_version: '0.4.9', to_version: '0.5.0',
  }),
  force_firmware_update: ({ node_id, build_type }) => ok({
    message: 'Forced firmware update initiated (mock)',
    build_type, version: '0.5.0', success: true,
  }),

  // Update manager — used by Settings → Updates panel.
  'update.get_status': () => ok({
    state: 'idle', message: 'No update in progress', progress: 0,
    available: null, installed_version: '0.5.0', server_version: '0.5.0',
  }),
  'update.check_now':  () => ok({ checking: true }),

  // Animations ────────────────────────────────────────────────────
  list_animations: () => ok({
    animations: [...st.animations.values()].map(a => ({
      id: a.id, name: a.name, duration: a.duration, fps: a.fps, loop: !!a.loop,
      icon: a.icon || '', group: a.group || '',
      value_tracks: a.value_tracks?.length || 0,
      trigger_tracks: a.trigger_tracks?.length || 0,
      modified: a.modified || '',
    })),
  }),
  get_animation: ({ id }) => {
    const a = st.animations.get(id)
    return a ? ok({ animation: a }) : err('Not found')
  },
  save_animation: ({ animation }, ctx) => {
    if (!animation || typeof animation !== 'object') return err('Invalid animation payload')
    const id = st.slugify(animation.id || animation.name || 'untitled')
    const now = new Date().toISOString()
    const saved = {
      ...animation, id,
      created: animation.created || now,
      modified: now,
    }
    st.animations.set(id, saved)
    return ok({ success: true, animation: saved })
  },
  delete_animation: ({ id }, ctx) => {
    st.animations.delete(id)
    // Stop any active player. Animations now drive routing through
    // URDF-joint inputs keyed by joint name, so deleting an animation
    // doesn't invalidate sheet topology — the joint inputs just stop
    // receiving values until another animation drives the same joint.
    if (st.animationPlayers.has(id)) st.animationPlayers.delete(id)
    st.clearAnimationValues(id)
    st.systemRouting.version++
    ctx.broadcast('system_routing', st.systemRouting)
    return ok({ success: true })
  },
  start_animation: ({ id, loop }) => {
    const a = st.animations.get(id)
    if (!a) return err('Animation not found')
    if (loop != null) a.loop = !!loop
    st.animationPlayers.set(id, {
      id, t: 0, last_t: 0, running: true, paused: false,
      loop: !!a.loop, started_at: Date.now(),
    })
    return ok({ success: true })
  },
  stop_animation: ({ id }) => {
    if (!st.animationPlayers.delete(id)) return err('Animation not running')
    st.clearAnimationValues(id)
    return ok({ success: true })
  },
  pause_animation: ({ id }) => {
    const p = st.animationPlayers.get(id)
    if (!p) return err('Animation not running')
    p.paused = true
    return ok({ success: true })
  },
  resume_animation: ({ id }) => {
    const p = st.animationPlayers.get(id)
    if (!p) return err('Animation not running')
    p.paused = false
    return ok({ success: true })
  },
  seek_animation: ({ id, t }) => {
    const p = st.animationPlayers.get(id)
    if (!p) return err('Animation not running')
    p.t = Math.max(0, Number(t) || 0)
    return ok({ success: true })
  },
  animation_state: () => ok({
    players: [...st.animationPlayers.values()].map(p => {
      const a = st.animations.get(p.id)
      return {
        id: p.id, name: a?.name || p.id, duration: a?.duration || 0,
        t: p.t, running: p.running, paused: p.paused, loop: p.loop,
      }
    }),
  }),

  // Poses ──────────────────────────────────────────────────────────
  list_poses: () => ok({
    poses: [...st.poses.values()].map(p => ({
      id: p.id, name: p.name,
      icon: p.icon || '', group: p.group || '',
      description: p.description || '',
      setpoint_count: p.setpoints?.length || 0,
      modified: p.modified || '',
    })),
  }),
  get_pose: ({ id }) => {
    const p = st.poses.get(id)
    return p ? ok({ pose: p }) : err('Not found')
  },
  save_pose: ({ pose }) => {
    if (!pose || typeof pose !== 'object') return err('Invalid pose payload')
    const id = st.slugify(pose.id || pose.name || 'untitled')
    const now = new Date().toISOString()
    const saved = {
      ...pose, id,
      created: pose.created || now,
      modified: now,
    }
    st.poses.set(id, saved)
    return ok({ success: true, pose: saved })
  },
  delete_pose: ({ id }) => {
    return st.poses.delete(id) ? ok({ success: true }) : err('Pose not found')
  },
  apply_pose: ({ id }, ctx) => {
    const p = st.poses.get(id)
    if (!p) return err('Pose not found')
    // Mock can't actually write into the routing evaluator's ws_input
    // cache, but reporting success lines up with what the UI shows
    // when the real server applies a pose.
    const applied = (p.setpoints || []).length
    ctx.activity?.(`Applied pose ${p.name || id} (${applied} setpoint${applied === 1 ? '' : 's'})`, 'info')
    return ok({ success: true, applied, skipped: [] })
  },

}

// ── Helpers ──────────────────────────────────────────────────────────

function ok (data = {})       { return { ok: true,  data } }
function err (message)        { return { ok: false, message } }

function ensureSheet (node_id) {
  let s = st.systemRouting.sheets[node_id]
  if (!s) {
    s = { node_id, inputs: [], ws_inputs: [], outputs: [],
          operators: [], widgets: [], wires: [] }
    st.systemRouting.sheets[node_id] = s
  }
  return s
}

function nextId (arr, prefix) {
  let n = 1
  const existing = new Set(arr.map(x => x.id))
  while (existing.has(`${prefix}${n}`)) n++
  return `${prefix}${n}`
}

function findSheetNode (sheet, id) {
  for (const list of ['inputs', 'ws_inputs', 'outputs', 'operators', 'widgets']) {
    const found = sheet[list].find(x => x.id === id)
    if (found) return found
  }
  return null
}

function bumpRouting (ctx) {
  st.systemRouting.version++
  ctx.broadcast('system_routing', st.systemRouting)
}

function deepMerge (target, source) {
  for (const k of Object.keys(source)) {
    if (source[k] && typeof source[k] === 'object' && !Array.isArray(source[k])
        && target[k] && typeof target[k] === 'object') {
      deepMerge(target[k], source[k])
    } else {
      target[k] = source[k]
    }
  }
}

// Synthesize a minimal pins[] for the capabilities response — enough
// for the pin picker dropdown to render something.
function rangePins (chip_family) {
  const max = chip_family === 'rp2040' ? 29 : 41
  const caps = ['digital_in', 'digital_out', 'pwm']
  const out = []
  for (let g = 0; g <= max; g++) {
    out.push({ gpio: g, name: chip_family === 'rp2040' ? `GP${g}` : `D${g}`, capabilities: caps })
  }
  return out
}

// ── Command / router / control channels (small surface — most paths
//    just need a successful ack to keep the UI moving).

export function handleCommand (action, params, ctx) {
  if (action === 'estop') {
    const target = params?.target || params?.node_id || 'all'
    ctx.broadcast('estop', {
      global: target === 'all' || target === 'system',
      node_id: target !== 'all' && target !== 'system' ? target : undefined,
      ts: st.nowIso(),
    })
    ctx.activity(`E-stop ${target}`, 'warning')
    return ok({ engaged: true })
  }
  return ok({ acked: true, action })
}

export function handleRouter (action, params, ctx) {
  if (action === 'list_websocket_inputs') {
    // Flatten ws_inputs across every sheet. Includes sheet_label so the
    // animation/pose target pickers can render readable names rather
    // than raw node IDs.
    const ws_inputs = []
    for (const [sid, sheet] of Object.entries(st.systemRouting.sheets)) {
      const node = st.findAdopted(sid)
      const sheet_label = node?.display_name || sid
      for (const wi of sheet.ws_inputs) {
        ws_inputs.push({
          sheet_id: sid, sheet_label,
          input_id: wi.id, label: wi.label,
          kind: wi.kind || 'command',
        })
      }
    }
    return ok({ ws_inputs })
  }
  if (action === 'set_input') return ok({})
  return ok({ acked: true, action })
}

export function handleControl (action, params, ctx) {
  // Pin / channel writes — mutate live.peripheralValues so the
  // pin_state broadcast next tick echoes the desired value back.
  if (action === 'set_pin_value' || action === 'set_pin_values' ||
      action === 'set_channel_value') {
    return ok({ throttled: false, native_value: params?.value })
  }
  if (action === 'get_runtime_state') {
    const state = st.live.peripheralValues[params?.node_id] || {}
    return ok({ node_id: params?.node_id, channels: state })
  }
  return ok({ acked: true, action })
}

// 'send' action types beyond management/command/control we still want
// to ack (e.g. `ros`, `discovery`, `livelink`, `file`, terminal.*).
export function handleGeneric (type, action, params, ctx) {
  if (type === 'ros' && action === 'list_topic_channels') {
    return ok({ topics: [
      { topic: '/joy', channels: [
        { field: 'axes[0]', display: 'Axis 0' },
        { field: 'axes[1]', display: 'Axis 1' },
        { field: 'buttons[0]', display: 'Button 0' },
      ]},
      { topic: '/cmd_vel', channels: [
        { field: 'linear.x', display: 'Linear X' },
        { field: 'angular.z', display: 'Angular Z' },
      ]},
    ]})
  }
  if (type === 'management' && action?.startsWith('terminal.')) {
    return ok({})  // terminal is a no-op in the mock
  }
  return ok({ acked: true, type, action })
}
