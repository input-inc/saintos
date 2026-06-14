<script setup>
import { onMounted, onUnmounted, ref } from 'vue'
import BMSMonitor from './BMSMonitor.vue'
import { makeWidgetRouting, startTicker } from './storyHelpers.js'

// Three variants reflect the states an operator should be able to
// recognise at a glance on the dashboard.
const INPUTS = ['soc', 'voltage', 'current', 'temp', 'protection', 'fet_status']

// Healthy 4S Li-ion at gentle discharge.
const healthy = makeWidgetRouting('bms_healthy', 'bms_monitor', INPUTS, { peripheralId: 'bms_healthy_p' })

// Same pack, charge FET tripped open — operator sees ISOLATED chip.
const isolated = makeWidgetRouting('bms_isolated', 'bms_monitor', INPUTS, { peripheralId: 'bms_isolated_p' })

// Protection bits asserted: cell overvoltage + charge overtemp.
const faulted = makeWidgetRouting('bms_faulted', 'bms_monitor', INPUTS, { peripheralId: 'bms_faulted_p' })

const socTarget = ref(78)

let stopHealthy, stopIsolated, stopFaulted
onMounted(() => {
  stopHealthy = startTicker(healthy.nodeId, healthy.peripheralId, (t) => ({
    soc:        socTarget.value + Math.sin(t / 20) * 0.5,
    voltage:    14.8 + Math.sin(t / 10) * 0.05,
    current:    -3.5 + Math.sin(t / 8) * 0.3,    // negative = discharge
    temp:       32 + Math.sin(t / 30) * 1.2,
    protection: 0,
    fet_status: 0x03,                              // both FETs on
  }))
  stopIsolated = startTicker(isolated.nodeId, isolated.peripheralId, (t) => ({
    soc:        46,
    voltage:    14.3,
    current:    0.0,
    temp:       30,
    protection: 0,
    fet_status: 0x02,                              // discharge only — charge FET off
  }))
  stopFaulted = startTicker(faulted.nodeId, faulted.peripheralId, (t) => ({
    soc:        12,
    voltage:    16.9,
    current:    -0.1,
    temp:       58 + Math.sin(t / 5),
    protection: (1 << 0) | (1 << 4),               // cell overvoltage + charge overtemp
    fet_status: 0x00,                              // both FETs off
  }))
})
onUnmounted(() => { stopHealthy?.(); stopIsolated?.(); stopFaulted?.() })
</script>

<template>
  <Story title="Widgets / BMS Monitor" :layout="{ type: 'grid', width: 360 }">
    <Variant title="Healthy (interactive SOC)">
      <template #controls>
        <label class="text-xs text-fg-muted block">SOC: {{ socTarget }}%</label>
        <input type="range" min="0" max="100" v-model.number="socTarget" class="w-full" />
      </template>
      <div class="p-3 bg-bg">
        <BMSMonitor :widget="healthy.widget" :routes="healthy.routes" />
      </div>
    </Variant>

    <Variant title="Isolated (charge FET open)">
      <div class="p-3 bg-bg">
        <BMSMonitor :widget="isolated.widget" :routes="isolated.routes" />
      </div>
    </Variant>

    <Variant title="Faulted (overvoltage + overtemp)">
      <div class="p-3 bg-bg">
        <BMSMonitor :widget="faulted.widget" :routes="faulted.routes" />
      </div>
    </Variant>
  </Story>
</template>
