<script setup>
import { computed, onMounted, onUnmounted, watch } from 'vue'
import ConsoleBatteriesOverview from './ConsoleBatteriesOverview.vue'
import KioskFrame from './_KioskFrame.vue'
import { pushFrame } from '@/components/widgets/storyHelpers.js'

// Wires Histoire's per-Variant bridged state into the WS mock store
// the overview reads from. One synthetic pin_state topic per pack —
// each pack lives on its own nodeId so the overview's per-node
// subscriptions exercise the real multi-pack code path.

const props = defineProps({
  state: { type: Object, required: true },
})

function nodeIdFor (i) { return `story_pack_${i}` }
function peripheralIdFor (i) { return `bms_${i}` }

const packsAsRouted = computed(() =>
  props.state.packs.map((p, i) => ({
    label: p.label, nodeId: nodeIdFor(i), peripheralId: peripheralIdFor(i),
  })))

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
function makePayload (p) {
  const cells = generateCells(p.cellCount, p.cellNominalV, p.cellSpreadMv)
  const protBits = (p.faults || []).reduce((acc, i) => acc | (1 << i), 0)
  const fet = (p.chgFet ? 0x01 : 0) | (p.dsgFet ? 0x02 : 0)
  const out = {
    soc: p.soc,
    pack_voltage: cells.reduce((a, b) => a + b, 0),
    current: p.current,
    temp_1: 32,
    temp_2: 30,
    protection: protBits,
    fet_status: fet,
    cell_count: p.cellCount,
  }
  cells.forEach((v, i) => {
    out[`cell_${String(i + 1).padStart(2, '0')}`] = v
  })
  return out
}

function pushAll () {
  props.state.packs.forEach((p, i) => {
    pushFrame(nodeIdFor(i), peripheralIdFor(i), makePayload(p))
  })
}

let heartbeat = null
let stopWatch = null
onMounted(() => {
  pushAll()
  stopWatch = watch(() => props.state, pushAll, { deep: true })
  heartbeat = setInterval(pushAll, 1000)
})
onUnmounted(() => {
  if (heartbeat) clearInterval(heartbeat)
  if (stopWatch) stopWatch()
})
</script>

<template>
  <KioskFrame>
    <ConsoleBatteriesOverview :packs="packsAsRouted" />
  </KioskFrame>
</template>
