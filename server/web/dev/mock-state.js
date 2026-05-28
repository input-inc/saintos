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

export const peripheralCatalog = [
  ptype('button', 'Button', 'A momentary switch.', 'gpio',
    [chan('pressed', 'Pressed', 'in', 'digital_in')],
    [{ id: 'pull_up', label: 'Pull-up', type: 'bool', default: true },
     { id: 'active_low', label: 'Active low', type: 'bool', default: true },
     { id: 'debounce_ms', label: 'Debounce (ms)', type: 'int', default: 20, min: 0, max: 500 }]),
  ptype('led', 'LED', 'A digital output on a GPIO pin.', 'gpio',
    [chan('on', 'On', 'out', 'digital_out')],
    [{ id: 'active_low', label: 'Active low', type: 'bool', default: false }]),
  ptype('analog_in', 'Analog Input', 'Voltage reading on an ADC pin.', 'adc',
    [chan('voltage', 'Voltage', 'in', 'analog')],
    [{ id: 'divider_ratio', label: 'Voltage divider ratio', type: 'float', default: 1.0 }]),
  ptype('neopixel', 'NeoPixel (RGB)', 'WS2812 RGB LED hardwired on the board.', 'builtin',
    [chan('color', 'Color (RGB)', 'out', 'rgb'),
     chan('brightness', 'Brightness', 'out', 'analog')],
    [{ id: 'default_color', label: 'Idle color', type: 'string', default: '#1e293b' }],
    true),
  ptype('roboclaw', 'RoboClaw Motor', 'RoboClaw Solo 60A motor controller.', 'uart',
    [chan('motor',   'Motor duty',    'out', 'analog'),
     chan('encoder', 'Encoder',       'in',  'analog'),
     chan('voltage', 'Bus voltage',   'in',  'analog'),
     chan('current', 'Motor current', 'in',  'analog'),
     chan('temp',    'Temperature',   'in',  'analog')],
    [{ id: 'address',  label: 'Address',  type: 'int', default: 128, min: 128, max: 135 },
     { id: 'deadband', label: 'Deadband', type: 'int', default: 0,   min: 0,   max: 127 },
     { id: 'max_current_ma', label: 'Max current (mA)', type: 'int', default: 0, min: 0, max: 60000 },
     { id: 'estop_pin', label: 'E-stop pin', type: 'gpio', default: 0, min: 0, max: 29 },
     { id: 'uart_swap', label: 'Use PIO UART', type: 'bool', default: false },
     { id: 'invert_direction', label: 'Invert direction', type: 'bool', default: false }]),
  ptype('fas100', 'FAS100', 'FrSky FAS100 ADV current/voltage/temperature sensor.', 'uart',
    [chan('amps',  'Current (A)', 'in', 'analog'),
     chan('volts', 'Voltage (V)', 'in', 'analog'),
     chan('temp1', 'Temp 1 (°C)', 'in', 'analog'),
     chan('temp2', 'Temp 2 (°C)', 'in', 'analog')],
    [{ id: 'poll_interval_ms', label: 'Poll interval (ms)', type: 'int', default: 50, min: 20, max: 1000 }]),
  ptype('system_monitor', 'System Monitor',
    'Host telemetry: CPU/memory/temp/throttle/uptime + WiFi link health.', 'builtin',
    [chan('cpu_usage', 'CPU usage (%)', 'in', 'analog'),
     chan('cpu_temp',  'CPU temp (°C)', 'in', 'analog'),
     chan('mem_usage', 'Memory usage (%)', 'in', 'analog'),
     chan('throttle',  'Throttled', 'in', 'digital_in'),
     chan('uptime',    'Uptime (s)', 'in', 'analog'),
     chan('wifi_signal',    'WiFi signal (dBm)', 'in', 'analog'),
     chan('wifi_retry_pct', 'WiFi tx retry (%)', 'in', 'analog'),
     chan('wifi_noise',     'WiFi noise (dBm)', 'in', 'analog'),
     chan('wifi_bitrate',   'WiFi tx bitrate (Mbps)', 'in', 'analog')],
    [],
    true),
]

export const widgetCatalog = [
  wtype('roboclaw_monitor', 'RoboClaw Motor Monitor',
    'RoboClaw telemetry: duty, encoder, voltage, current, temp.',
    [winp('motor', 'Motor Duty', 'analog'),
     winp('encoder', 'Encoder', 'analog'),
     winp('voltage', 'Bus Voltage', 'analog'),
     winp('current', 'Motor Current', 'analog'),
     winp('temp', 'Temperature', 'analog')]),
  wtype('battery_monitor', 'FAS100 Power Monitor',
    'FrSky FAS100 battery telemetry.',
    [winp('current', 'Current', 'analog'),
     winp('voltage', 'Voltage', 'analog'),
     winp('temp1', 'Temp 1', 'analog'),
     winp('temp2', 'Temp 2', 'analog')]),
  wtype('single_gauge', 'Single-value Gauge',
    'One numeric reading with a meter.',
    [winp('value', 'Value', 'analog')]),
  wtype('status_led_indicator', 'Status Indicator',
    'Reflects a digital signal as an on-screen LED.',
    [winp('state', 'State', 'digital_in')]),
  wtype('estop_indicator', 'E-Stop Indicator',
    'Lights up when an e-stop signal goes high.',
    [winp('triggered', 'Triggered', 'digital_in')]),
]

export const operatorCatalog = [
  otype('gain', 'Gain', 'out = value × gain',
    [oin('value', 'Value')],
    [{ id: 'gain', label: 'Gain', type: 'float', default: 1.0 }]),
  otype('scale', 'Scale', 'out = value × factor',
    [oin('value', 'Value'), oin('factor', 'Factor', 1.0)]),
  otype('add', 'Add', 'out = a + b', [oin('a', 'A'), oin('b', 'B')]),
  otype('multiply', 'Multiply', 'out = a × b',
    [oin('a', 'A', 1.0), oin('b', 'B', 1.0)]),
  otype('clamp', 'Clamp', 'Clamp value into [min, max].',
    [oin('value', 'Value'), oin('min', 'Min', -1), oin('max', 'Max', 1)]),
  otype('deadband', 'Deadband', 'Pass through if |value| > threshold.',
    [oin('value', 'Value'), oin('threshold', 'Threshold', 0.05)]),
  otype('invert', 'Invert', 'out = −value', [oin('value', 'Value')]),
]

function ptype (id, label, description, pin_kind, channels, params, builtin_only = false) {
  return { id, label, description, pin_kind, channels, params, builtin_only }
}
function chan (id, display, dir, cap) { return { id, display, dir, cap } }
function wtype (id, label, description, inputs) { return { id, label, description, inputs } }
function winp (id, display, cap) { return { id, display, cap } }
function otype (id, label, description, inputs, params = []) {
  return { id, label, description, inputs, params }
}
function oin (id, label, def = 0.0) { return { id, label, default: def } }

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
  rpi5:       { available: true, version: '0.5.0', filename: 'saint-os_0.5.0_rpi5.zip',
                checksum: 'abcd1234', build_date: '2026-05-25 14:30:00', type: 'rpi5' },
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
