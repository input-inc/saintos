// Story helpers — only used by *.story.vue files under Histoire.
//
// Generates and pushes synthetic pin_state frames into the WS store so
// widgets that subscribe to live peripherals see motion without a
// running backend.

import { useWsStore } from '@/stores/ws'

// Build a widget descriptor + routes wiring each declared input to a
// matching channel on a mock peripheral. Caller passes the input list
// so this helper stays peripheral-agnostic.
//
//   widget(id, type, inputs)
//     → { widget: {id, type, label}, routes: [...] }
//
// routes[].sink   = {kind:'widget',    parts: [widgetId, inputId]}
// routes[].source = {kind:'peripheral', parts: [nodeId, peripheralId, inputId]}
export function makeWidgetRouting (widgetId, widgetType, inputIds, opts = {}) {
  const {
    nodeId = 'story_node',
    peripheralId = 'story_peripheral',
    label = widgetId,
  } = opts
  const widget = { id: widgetId, type: widgetType, label }
  const routes = inputIds.map(inputId => ({
    sink:   { kind: 'widget',     parts: [widgetId, inputId] },
    source: { kind: 'peripheral', parts: [nodeId, peripheralId, inputId] },
  }))
  return { widget, routes, nodeId, peripheralId }
}

// Push one pin_state frame for `nodeId` containing the given
// channels map ({channel_id: value}). Also fires the `state` event so
// widgets that ingest sparkline samples on event (Fas100Monitor) tick.
export function pushFrame (nodeId, peripheralId, values) {
  const ws = useWsStore()
  const channels = []
  for (const [channel_id, value] of Object.entries(values)) {
    channels.push({ peripheral_id: peripheralId, channel_id, value, last_updated: Date.now() })
  }
  const frame = { node_id: nodeId, channels }
  const topic = `pin_state/${nodeId}`
  ws.topics.set(topic, frame)
  if (typeof ws.emit === 'function') ws.emit('state', { node: topic, data: frame })
}

// Drive a values-generator function on an interval until the story
// unmounts. Returns the stop function — call it from onUnmounted.
export function startTicker (nodeId, peripheralId, gen, intervalMs = 250) {
  let t = 0
  const id = setInterval(() => {
    pushFrame(nodeId, peripheralId, gen(t))
    t += 1
  }, intervalMs)
  // Push an initial frame so the widget has values to render immediately.
  pushFrame(nodeId, peripheralId, gen(0))
  return () => clearInterval(id)
}
