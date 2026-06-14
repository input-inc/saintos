<script setup>
// Story-only — bridges Histoire's per-Variant `state` (a reactive
// object kept in sync across the controls slot and the default slot)
// into the WS mock store the ConsoleBatteryView reads from. Watching
// `state` deeply re-pushes a fresh pin_state frame on every change,
// and a slow heartbeat keeps the view alive even when nothing has
// moved.

import { onMounted, onUnmounted, watch } from 'vue'
import ConsoleBatteryView from './ConsoleBatteryView.vue'
import KioskFrame from './_KioskFrame.vue'
import { pushFrame } from '@/components/widgets/storyHelpers.js'

const props = defineProps({
  state:   { type: Object, required: true },
  nodeId:  { type: String, required: true },
})
const PERIPHERAL_ID = 'bms'

function generateCells (count, nominalV, spreadMv) {
  const half = spreadMv / 2 / 1000
  const out = []
  for (let i = 0; i < count; i++) {
    let v = nominalV
    if (i === 0)             v = nominalV - half
    else if (i === count-1)  v = nominalV + half
    out.push(v)
  }
  return out
}

function makeFramePayload (s) {
  const cells = generateCells(s.cellCount, s.cellNominalV, s.cellSpreadMv)
  const protBits = (s.faults || []).reduce((acc, i) => acc | (1 << i), 0)
  const fet = (s.chgFet ? 0x01 : 0) | (s.dsgFet ? 0x02 : 0)
  const out = {
    soc:          s.soc,
    pack_voltage: cells.reduce((a, b) => a + b, 0),
    current:      s.current,
    temp_1:       s.temp1,
    temp_2:       s.temp2,
    protection:   protBits,
    fet_status:   fet,
    cell_count:   s.cellCount,
  }
  cells.forEach((v, i) => {
    out[`cell_${String(i + 1).padStart(2, '0')}`] = v
  })
  return out
}

function push () {
  pushFrame(props.nodeId, PERIPHERAL_ID, makeFramePayload(props.state))
}

let heartbeat = null
let stopWatch = null
onMounted(() => {
  push()                       // initial paint
  // Re-push on every state change so the view reflects the slider
  // immediately. Deep so changes to arrays (faults) trigger too.
  stopWatch = watch(() => props.state, push, { deep: true })
  // Slow heartbeat as a safety net in case a downstream consumer
  // (history sparklines, etc.) needs periodic emissions.
  heartbeat = setInterval(push, 1000)
})
onUnmounted(() => {
  if (heartbeat) clearInterval(heartbeat)
  if (stopWatch) stopWatch()
})
</script>

<template>
  <KioskFrame>
    <ConsoleBatteryView :node-id="nodeId" :peripheral-id="PERIPHERAL_ID" />
  </KioskFrame>
</template>
