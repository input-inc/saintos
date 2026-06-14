// In-memory fixture state for the dev mock WebSocket server.
//
// Shapes here mirror the real server's JSON wire format. The originals
// live in:
//   - server/saint_server/peripheral_model.py     (routing, catalogs)
//   - server/saint_server/webserver/state_manager.py (nodes, settings)
//   - server/saint_server/webserver/websocket_handler.py (action returns)
//
// Mutations are performed in place; broadcast helpers in mock-server.js
// snapshot via JSON.stringify on every send.

// ── Adopted / unadopted nodes ────────────────────────────────────────

export const adoptedNodes = [
  makeAdopted({
    node_id: 'rp2040_5857c7555f34',
    display_name: 'Roboclaw Right',
    chip_family: 'rp2040',
    board_id: 'feather_rp2040',
    role: 'controller',
    hardware_model: 'Adafruit Feather RP2040',
    ip_address: '192.168.4.21',
    mac_address: '58:57:c7:55:5f:34',
  }),
  makeAdopted({
    node_id: 'rp2040_48405f4f3d28',
    display_name: 'Roboclaw Left',
    chip_family: 'rp2040',
    board_id: 'feather_rp2040',
    role: 'controller',
    hardware_model: 'Adafruit Feather RP2040',
    ip_address: '192.168.4.22',
    mac_address: '48:40:5f:4f:3d:28',
  }),
]

export const unadoptedNodes = [
  {
    node_id: 'teensy41_a1b2c3d4',
    hardware_model: 'Teensy 4.1',
    mac_address: 'a1:b2:c3:d4:00:01',
    ip_address: '192.168.4.40',
    firmware_version: '0.5.0',
    bootloader_version: 'unknown',
    firmware_build: '2026-05-20 12:00:00',
    cpu_temp: 45.0, cpu_usage: 8.0, memory_usage: 35.0, uptime_seconds: 120,
    state: 'UNADOPTED', last_seen: nowSec(), online: true,
    chip_family: 'teensy41',
  },
]

function makeAdopted ({
  node_id, display_name, chip_family, board_id, role,
  hardware_model, ip_address, mac_address,
}) {
  return {
    node_id, display_name, role, hardware_model, ip_address, mac_address,
    firmware_version: '0.5.0',
    bootloader_version: '0.2.0',
    firmware_build: '2026-05-25 14:30:00',
    online: true,
    cpu_temp: 48.0,
    cpu_usage: 12.0,
    memory_usage: 38.0,
    uptime_seconds: 3600,
    state: 'ACTIVE',
    last_seen: nowSec(),
    has_capabilities: true,
    chip_family, board_id,
    // Filled in by getNodePeripheralsCount() at serialize-time.
    peripheral_count: 0,
    peripheral_sync_status: 'synced',
    firmware_update_available: false,
    firmware_update_message: 'Firmware is up to date',
    server_firmware_version: '0.5.0',
    server_firmware_build: '2026-05-25 14:30:00',
    server_firmware_hash: 'mock1234',
  }
}

// ── Per-node peripherals ─────────────────────────────────────────────
//
// `pins` keys are the GPIO names declared in the peripheral type's
// `pin_kind` plus any explicit `tx`/`rx` for UART devices.

export const nodePeripherals = {
  rp2040_5857c7555f34: {
    version: 3,
    sync_status: 'synced',
    last_synced: nowSec() - 60,
    peripherals: [
      {
        id: 'roboclaw-1', type: 'roboclaw', label: 'Drive Motor',
        pins: { tx: 0, rx: 1 },
        params: { address: 128, deadband: 0, max_current_ma: 30000,
                  estop_pin: 0, uart_swap: false, invert_direction: false },
        builtin: false, log_enabled: false,
      },
      {
        id: 'neopixel-1', type: 'neopixel', label: 'Status LED',
        pins: { data: 16 },
        params: { default_color: '#1e293b' },
        builtin: true, log_enabled: false,
      },
    ],
  },
  rp2040_48405f4f3d28: {
    version: 2,
    sync_status: 'synced',
    last_synced: nowSec() - 90,
    peripherals: [
      {
        id: 'roboclaw-1', type: 'roboclaw', label: 'Drive Motor',
        pins: { tx: 0, rx: 1 },
        params: { address: 128, deadband: 0, max_current_ma: 30000,
                  estop_pin: 0, uart_swap: false, invert_direction: false },
        builtin: false, log_enabled: false,
      },
      {
        id: 'button-1', type: 'button', label: 'Limit Switch',
        pins: { gpio: 6 },
        params: { pull_up: true, active_low: true, debounce_ms: 20 },
        builtin: false, log_enabled: false,
      },
    ],
  },
}

// ── Catalog (peripheral / widget / operator types) ───────────────────
//
// All three catalogs are populated at mock-server boot by spawning
// `python3 scripts/dump_catalog.py` against the canonical Python
// model (server/saint_server/peripheral_model.py). The arrays exported
// below are intentionally empty — the boot script splices the dumped
// types in-place so importers keep their live references.

export const peripheralCatalog = []
export const widgetCatalog = []

// Populated at mock-server boot from the Python catalog dump (see
// dev/mock-server.js's loadCatalogsFromPython()). Stays mutable so
// importers hold live array references — adding to OPERATOR_CATALOG
// in peripheral_model.py automatically flows here on next mock start.
export const operatorCatalog = []

// ── Chips, boards, roles ─────────────────────────────────────────────

export const chips = [
  { chip_family: 'rp2040', display_name: 'RP2040', description: 'Raspberry Pi RP2040 microcontroller',
    chip_id_value: 0x2040, gpio_count: 30, pin_capabilities: [], uart_pairs: [], adc_pins: [26, 27, 28] },
  { chip_family: 'teensy41', display_name: 'Teensy 4.1', description: 'NXP iMXRT1062 microcontroller',
    chip_id_value: 0x1062, gpio_count: 42, pin_capabilities: [], uart_pairs: [], adc_pins: [] },
]

export const boards = [
  { board_id: 'feather_rp2040', display_name: 'Adafruit Feather RP2040',
    chip_family: 'rp2040', builtin: true,
    available_pins: range(0, 29).map(g => ({ gpio: g, name: `GP${g}` })),
    reserved_pins: [], builtin_peripherals: [
      { id: 'neopixel-builtin', type: 'neopixel', label: 'Onboard NeoPixel',
        pins: { data: 16 }, params: {} },
    ] },
  { board_id: 'pico', display_name: 'Raspberry Pi Pico',
    chip_family: 'rp2040', builtin: true,
    available_pins: range(0, 29).map(g => ({ gpio: g, name: `GP${g}` })),
    reserved_pins: [], builtin_peripherals: [] },
  { board_id: 'teensy41_default', display_name: 'Teensy 4.1 (default)',
    chip_family: 'teensy41', builtin: true,
    available_pins: range(0, 41).map(g => ({ gpio: g, name: `D${g}` })),
    reserved_pins: [], builtin_peripherals: [] },
]

export const roles = [
  { role: 'controller', display_name: 'Controller',
    description: 'Routing-graph controller node with peripherals.',
    required_count: 0, optional_count: 0 },
  { role: 'sensor', display_name: 'Sensor',
    description: 'Read-only sensor reporting telemetry topics.',
    required_count: 0, optional_count: 0 },
  { role: 'unspecified', display_name: 'Unspecified',
    description: 'Generic node — pick a more specific role later.',
    required_count: 0, optional_count: 0 },
]

// ── Firmware builds ──────────────────────────────────────────────────

export const firmwareBuilds = {
  simulation: { available: true, version: '0.5.0', version_full: '0.5.0-sim',
                build_date: '2026-05-25 14:30:00', git_hash: 'simhash1' },
  hardware:   { available: true, version: '0.5.0', version_full: '0.5.0-hw',
                build_date: '2026-05-25 14:30:00', git_hash: 'hwhash01' },
  raspberrypi:       { available: true, version: '0.5.0', filename: 'saint-os_0.5.0_raspberrypi.zip',
                checksum: 'abcd1234', build_date: '2026-05-25 14:30:00', type: 'raspberrypi' },
  controller: { available: true, version: '0.5.0-local.mock',
                filename: 'saint_firmware_controller_0.5.0-local.mock.AppImage',
                checksum: 'mockmockmock', build_date: '2026-05-25T14:30:00Z', type: 'controller' },
}

// ── Settings ─────────────────────────────────────────────────────────

export const settings = {
  server: { name: 'SAINT.OS (Mock)' },
  websocket: { password: null, auth_timeout: 10.0 },
  network: { web_port: 80, websocket_port: null },
  ros_bridge: { throttle_ms: 50 },
  livelink: { enabled: true, port: 11111 },
  logging: { level: 'WARNING' },
}

// ── WiFi config ──────────────────────────────────────────────────────

export const wifi = {
  ok: true,
  ssid: 'SAINT-AP',
  password: 'mock-password-1234',
  band: 'a',          // 'a' = 5GHz, 'bg' = 2.4GHz (matches nmcli's coding)
  channel: 44,
  iface: 'wlan0',
}

// ── System routing — sheets keyed by sheet_id ────────────────────────

export const systemRouting = {
  version: 1,
  sheets: {
    rp2040_5857c7555f34: {
      node_id: 'rp2040_5857c7555f34',
      inputs: [],
      ws_inputs: [
        { id: 'wsin1', label: 'Drive cmd', position: [40, 120], kind: 'command' },
      ],
      outputs: [],
      operators: [
        { id: 'op1', op: 'scale', label: 'Scale drive', params: {},
          defaults: { factor: 1.0 }, position: [280, 120] },
      ],
      widgets: [],
      wires: [
        { id: 'w1',
          source: { kind: 'ws_input',  parts: ['wsin1'] },
          sink:   { kind: 'operator',  parts: ['op1', 'value'] } },
        { id: 'w2',
          source: { kind: 'operator',  parts: ['op1', 'out'] },
          sink:   { kind: 'peripheral',
                    parts: ['rp2040_5857c7555f34', 'roboclaw-1', 'motor'] } },
      ],
    },
    rp2040_48405f4f3d28: {
      node_id: 'rp2040_48405f4f3d28',
      inputs: [],
      ws_inputs: [
        { id: 'wsin1', label: 'Drive cmd', position: [40, 120], kind: 'command' },
      ],
      outputs: [],
      operators: [],
      widgets: [],
      wires: [
        { id: 'w1',
          source: { kind: 'ws_input',  parts: ['wsin1'] },
          sink:   { kind: 'peripheral',
                    parts: ['rp2040_48405f4f3d28', 'roboclaw-1', 'motor'] } },
      ],
    },
    _dashboard: {
      node_id: '_dashboard',
      inputs: [], ws_inputs: [], outputs: [], operators: [],
      widgets: [
        { id: 'roboclaw_monitor-1', type: 'roboclaw_monitor',
          label: 'Right Motor', position: [40, 40], params: {} },
        { id: 'roboclaw_monitor-2', type: 'roboclaw_monitor',
          label: 'Left Motor', position: [380, 40], params: {} },
      ],
      wires: [
        { id: 'w1',
          source: { kind: 'peripheral',
                    parts: ['rp2040_5857c7555f34', 'roboclaw-1', 'encoder'] },
          sink:   { kind: 'widget', parts: ['roboclaw_monitor-1', 'encoder'] } },
        { id: 'w2',
          source: { kind: 'peripheral',
                    parts: ['rp2040_5857c7555f34', 'roboclaw-1', 'voltage'] },
          sink:   { kind: 'widget', parts: ['roboclaw_monitor-1', 'voltage'] } },
      ],
    },
  },
}

// ── Live-data scratchpad — updated each broadcast tick ───────────────
//
// peripheralValues[node_id][peripheral_id][channel_id] = value.
// The broadcast loop walks adoptedNodes + nodePeripherals to populate.

export const live = {
  peripheralValues: {},     // {node_id: {peripheral_id: {channel_id: number}}}
  hostMonitor: {            // host_controller telemetry tracked separately
    cpu_usage: 12, cpu_temp: 50, mem_usage: 45, throttle: 0, uptime: 3600,
    wifi_signal: -58, wifi_retry_pct: 2.0, wifi_noise: -95, wifi_bitrate: 87,
  },
}

// ── Activity log + node logs ─────────────────────────────────────────

export const nodeLogs = {}   // node_id -> [{ts, level, message}]
export const serverLogs = [
  logEntry('info', 'Mock server started'),
  logEntry('info', 'Loaded 2 adopted nodes from fixture'),
  logEntry('warn', 'Mock mode — no real hardware attached'),
]

function logEntry (level, message) {
  return { timestamp: nowSec(), level, message }
}

// ── Subscription bookkeeping (per-client topic sets) ─────────────────

export const clients = new Map()   // client_id -> { ws, subscriptions: Set, ... }

// ── Animation builder fixtures (mock-only) ──────────────────────────
//
// Mirrors the real server's animation/pose stores. Persistence is
// memory-only — restart the mock and you lose everything.

export const animations = new Map()   // id → Animation JSON
export const poses      = new Map()   // id → Pose JSON

// Active playback state. Per id: { t, last_t, running, paused, loop, started_at }.
// The mock-server tick loop advances `t` and samples value tracks into
// live.animationValues so the routing canvas lights up.
export const animationPlayers = new Map()

// URDF model — single slot, replaced on every upload.
//
// Shape: { metadata: {…}, urdfBytes: Buffer, meshes: Map<string, Buffer> }
// `metadata` matches what the real server's URDFStore writes to
// metadata.json — original_filename, urdf_filename, sha256, uploaded_at,
// mesh_files (filenames only), link_count, joint_count.
export let urdfModel = null
export function setUrdfModel (model) { urdfModel = model }
export function getUrdfModel ()      { return urdfModel }

// URDF-joint cache — animation players write here per tick and every
// URDF-joint input on every sheet reads from here. Keyed by joint
// name so any animation whose value-track id matches the joint drives
// it (matches the real server's evaluator._urdf_joint_values).
//
// `live_animation_track_origins` tracks which animation owns which
// joint write so clearAnimationValues can zero out only that
// animation's contributions when a player stops.
export const live_urdf_joint_values = new Map()      // joint → value
const live_animation_track_origins   = new Map()      // animation_id → Set(joint)
export function setUrdfJointValue (animation_id, joint, value) {
  live_urdf_joint_values.set(joint, value)
  let owned = live_animation_track_origins.get(animation_id)
  if (!owned) {
    owned = new Set()
    live_animation_track_origins.set(animation_id, owned)
  }
  owned.add(joint)
}
export function getUrdfJointValue (joint) {
  return live_urdf_joint_values.get(joint)
}
export function clearAnimationValues (animation_id) {
  const owned = live_animation_track_origins.get(animation_id)
  if (!owned) return
  for (const joint of owned) live_urdf_joint_values.delete(joint)
  live_animation_track_origins.delete(animation_id)
}

// ── Curve sampling (mirrors saint_server/unreal/animation.py) ───────
//
// Interp encoding matches the Vue catalog in
// src/composables/easings.js:
//   0 CONSTANT   1 LINEAR   2 CUBIC (Hermite)
//   3-6   CSS basics (ease, ease-in, ease-out, ease-in-out)
//   10-39 easings.net palette (Sine / Quad / Cubic / Quart / Quint /
//         Expo / Circ / Back / Elastic / Bounce, each in/out/inOut)
//
// Keep this in sync with src/composables/easings.js — any divergence
// shows up as a preview-vs-playback mismatch.

const MOCK_EASING_BEZIERS = {
  1: [0,    0,   1,    1],
  3: [0.25, 0.1, 0.25, 1],
  4: [0.42, 0,   1,    1],
  5: [0,    0,   0.58, 1],
  6: [0.42, 0,   0.58, 1],
}

const _C1 = 1.70158, _C2 = _C1 * 1.525, _C3 = _C1 + 1
const _C4 = (2 * Math.PI) / 3, _C5 = (2 * Math.PI) / 4.5
const _N1 = 7.5625, _D1 = 2.75
function _bounceOut (x) {
  if (x < 1 / _D1)   return _N1 * x * x
  if (x < 2 / _D1)   return _N1 * (x -= 1.5 / _D1)   * x + 0.75
  if (x < 2.5 / _D1) return _N1 * (x -= 2.25 / _D1)  * x + 0.9375
  return                    _N1 * (x -= 2.625 / _D1) * x + 0.984375
}
function _easingsNet (code, t) {
  if (t <= 0) return 0
  if (t >= 1) return 1
  switch (code) {
    case 10: return 1 - Math.cos((t * Math.PI) / 2)
    case 11: return Math.sin((t * Math.PI) / 2)
    case 12: return -(Math.cos(Math.PI * t) - 1) / 2
    case 13: return t * t
    case 14: return 1 - (1 - t) * (1 - t)
    case 15: return t < 0.5 ? 2 * t * t : 1 - Math.pow(-2 * t + 2, 2) / 2
    case 16: return t * t * t
    case 17: return 1 - Math.pow(1 - t, 3)
    case 18: return t < 0.5 ? 4 * t * t * t : 1 - Math.pow(-2 * t + 2, 3) / 2
    case 19: return t * t * t * t
    case 20: return 1 - Math.pow(1 - t, 4)
    case 21: return t < 0.5 ? 8 * t ** 4 : 1 - Math.pow(-2 * t + 2, 4) / 2
    case 22: return t ** 5
    case 23: return 1 - Math.pow(1 - t, 5)
    case 24: return t < 0.5 ? 16 * t ** 5 : 1 - Math.pow(-2 * t + 2, 5) / 2
    case 25: return t === 0 ? 0 : Math.pow(2, 10 * t - 10)
    case 26: return t === 1 ? 1 : 1 - Math.pow(2, -10 * t)
    case 27: {
      if (t === 0) return 0
      if (t === 1) return 1
      return t < 0.5 ? Math.pow(2, 20 * t - 10) / 2
                     : (2 - Math.pow(2, -20 * t + 10)) / 2
    }
    case 28: return 1 - Math.sqrt(1 - t * t)
    case 29: return Math.sqrt(1 - (t - 1) ** 2)
    case 30: return t < 0.5
      ? (1 - Math.sqrt(1 - (2 * t) ** 2)) / 2
      : (Math.sqrt(1 - (-2 * t + 2) ** 2) + 1) / 2
    case 31: return _C3 * t ** 3 - _C1 * t * t
    case 32: return 1 + _C3 * Math.pow(t - 1, 3) + _C1 * Math.pow(t - 1, 2)
    case 33: return t < 0.5
      ? (Math.pow(2 * t, 2) * ((_C2 + 1) * 2 * t - _C2)) / 2
      : (Math.pow(2 * t - 2, 2) * ((_C2 + 1) * (t * 2 - 2) + _C2) + 2) / 2
    case 34: {
      if (t === 0) return 0
      if (t === 1) return 1
      return -Math.pow(2, 10 * t - 10) * Math.sin((t * 10 - 10.75) * _C4)
    }
    case 35: {
      if (t === 0) return 0
      if (t === 1) return 1
      return Math.pow(2, -10 * t) * Math.sin((t * 10 - 0.75) * _C4) + 1
    }
    case 36: {
      if (t === 0) return 0
      if (t === 1) return 1
      return t < 0.5
        ? -(Math.pow(2, 20 * t - 10) * Math.sin((20 * t - 11.125) * _C5)) / 2
        : (Math.pow(2, -20 * t + 10) * Math.sin((20 * t - 11.125) * _C5)) / 2 + 1
    }
    case 37: return 1 - _bounceOut(1 - t)
    case 38: return _bounceOut(t)
    case 39: return t < 0.5
      ? (1 - _bounceOut(1 - 2 * t)) / 2
      : (1 + _bounceOut(2 * t - 1)) / 2
    default: return t
  }
}
function cubicBezierAtTime (t, c1x, c1y, c2x, c2y) {
  if (t <= 0) return 0
  if (t >= 1) return 1
  let s = t
  for (let i = 0; i < 8; i++) {
    const one_s = 1 - s
    const x = 3 * one_s * one_s * s * c1x
            + 3 * one_s * s * s * c2x
            + s * s * s
    const dx = 3 * one_s * one_s * c1x
             + 6 * one_s * s * (c2x - c1x)
             + 3 * s * s * (1 - c2x)
    if (Math.abs(x - t) < 1e-5 || Math.abs(dx) < 1e-6) break
    s -= (x - t) / dx
    if (s < 0) s = 0
    if (s > 1) s = 1
  }
  const one_s = 1 - s
  return 3 * one_s * one_s * s * c1y
       + 3 * one_s * s * s * c2y
       + s * s * s
}
export function sampleCurve (curve, time) {
  const keys = curve?.keys || []
  if (!keys.length) return 0
  if (time <= keys[0].time) return keys[0].value
  if (time >= keys[keys.length - 1].time) return keys[keys.length - 1].value
  for (let i = 0; i < keys.length - 1; i++) {
    const k0 = keys[i], k1 = keys[i + 1]
    if (k0.time <= time && time <= k1.time) {
      const dt = k1.time - k0.time
      const t = dt === 0 ? 0 : (time - k0.time) / dt
      const interp = k0.interp ?? 1
      if (interp === 0) return k0.value
      if (interp === 2) {
        const t2 = t * t, t3 = t2 * t
        const h1 = 2 * t3 - 3 * t2 + 1
        const h2 = -2 * t3 + 3 * t2
        const h3 = t3 - 2 * t2 + t
        const h4 = t3 - t2
        return (h1 * k0.value + h2 * k1.value +
                h3 * (k0.leave_tangent || 0) * dt +
                h4 * (k1.arrive_tangent || 0) * dt)
      }
      const cp = MOCK_EASING_BEZIERS[interp]
      if (cp) {
        const y = cubicBezierAtTime(t, cp[0], cp[1], cp[2], cp[3])
        return k0.value + y * (k1.value - k0.value)
      }
      if (interp >= 10 && interp <= 39) {
        const y = _easingsNet(interp, t)
        return k0.value + y * (k1.value - k0.value)
      }
      return k0.value + t * (k1.value - k0.value)
    }
  }
  return keys[keys.length - 1].value
}

export function slugify (name) {
  return String(name || '').toLowerCase()
    .replace(/[^a-z0-9_-\s]+/g, '').trim()
    .replace(/\s+/g, '_')
    .replace(/_+/g, '_')
    .slice(0, 64) || 'untitled'
}

// ── Helpers ──────────────────────────────────────────────────────────

export function nowSec () { return Date.now() / 1000 }
export function nowIso () { return new Date().toISOString() }

function range (a, b) {
  const out = []
  for (let i = a; i <= b; i++) out.push(i)
  return out
}

// Look up an adopted node by id — null if unknown.
export function findAdopted (node_id) {
  return adoptedNodes.find(n => n.node_id === node_id) || null
}

export function findUnadopted (node_id) {
  return unadoptedNodes.find(n => n.node_id === node_id) || null
}

// Snapshot the adopted list with peripheral_count freshly counted from
// nodePeripherals so the UI's "N peripherals" badge stays in sync after
// save / remove.
export function adoptedListSnapshot () {
  return adoptedNodes.map(n => ({
    ...n,
    peripheral_count: nodePeripherals[n.node_id]?.peripherals?.length || 0,
    peripheral_sync_status: nodePeripherals[n.node_id]?.sync_status || 'unconfigured',
  }))
}
