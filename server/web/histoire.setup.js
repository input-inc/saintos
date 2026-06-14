// Histoire global setup — installs Pinia and stubs the WS / catalog
// stores every widget story leans on. Lives outside the production
// bundle (Histoire compiles to its own .histoire/ dir, never web/dist).
//
// What's mocked:
//   ws.subscribe/unsubscribe/management → no-op promises (real store
//     would Promise.reject('Not connected'); silent stub keeps story
//     load paths clean).
//   ws.on/off/emit → story-friendly registry replacing the closed-over
//     real one. Stories call ws.emit('state', ...) to drive widgets.
//   ws.topics → still the real reactive Map; stories `topics.set(...)`
//     to push synthetic pin_state frames.
//   catalog → loaded=true and widgetTypes/types seeded from the
//     server's DEFAULT_*_CATALOG mirrors (declared inline below — kept
//     in sync with peripheral_model.py by hand; small list).

import { createPinia } from 'pinia'
import { defineSetupVue3 } from '@histoire/plugin-vue'
import { useWsStore } from './src/stores/ws.js'
import { usePeripheralCatalog } from './src/stores/peripheralCatalog.js'
import './src/style.css'   // Tailwind + design tokens

// Inputs the widget components expect to find via catalog.widgetType().
const WIDGET_TYPE_FIXTURES = [
  {
    id: 'battery_monitor',
    label: 'FAS100 Power Monitor',
    description: 'FrSky FAS100 battery telemetry.',
    inputs: [
      { id: 'current', display: 'Current Draw',    type: 'analog' },
      { id: 'voltage', display: 'Battery Voltage', type: 'analog' },
      { id: 'temp1',   display: 'Temp 1',          type: 'analog' },
      { id: 'temp2',   display: 'Temp 2',          type: 'analog' },
    ],
  },
  {
    id: 'roboclaw_monitor',
    label: 'RoboClaw Motor Monitor',
    description: 'RoboClaw motor controller telemetry.',
    inputs: [
      { id: 'motor',   display: 'Motor Duty',    type: 'analog' },
      { id: 'encoder', display: 'Encoder',       type: 'analog' },
      { id: 'voltage', display: 'Bus Voltage',   type: 'analog' },
      { id: 'current', display: 'Motor Current', type: 'analog' },
      { id: 'temp',    display: 'Temperature',   type: 'analog' },
    ],
  },
  {
    id: 'bms_monitor',
    label: 'BMS Monitor',
    description: 'Pack-level battery summary.',
    inputs: [
      { id: 'soc',        display: 'State of Charge', type: 'analog' },
      { id: 'voltage',    display: 'Pack Voltage',    type: 'analog' },
      { id: 'current',    display: 'Pack Current',    type: 'analog' },
      { id: 'temp',       display: 'Temperature',     type: 'analog' },
      { id: 'protection', display: 'Protection Bits', type: 'analog' },
      { id: 'fet_status', display: 'FET Status',      type: 'analog' },
    ],
  },
]

function patchWsForStories (ws) {
  // Stub the real send-over-socket surface so widget onMounted hooks
  // don't reject loudly. The store's on/off/emit are left alone —
  // reassigning setup-store methods on a Pinia proxy isn't reliable,
  // so stories use the store's REAL handler registry via the now-
  // exposed `ws.emit` (see ws.js).
  ws.subscribe   = async () => ({ ok: true })
  ws.unsubscribe = async () => ({ ok: true })
  ws.management  = async () => ({})

  // Pretend we're connected — matches the post-handshake state widgets
  // expect when they call subscribe.
  ws.connected = true
}

export const setupVue3 = defineSetupVue3(({ app }) => {
  const pinia = createPinia()
  app.use(pinia)

  const ws = useWsStore(pinia)
  patchWsForStories(ws)

  const catalog = usePeripheralCatalog(pinia)
  catalog.widgetTypes = WIDGET_TYPE_FIXTURES
  catalog.types = []
  catalog.operatorTypes = []
  catalog.loaded = true
})
