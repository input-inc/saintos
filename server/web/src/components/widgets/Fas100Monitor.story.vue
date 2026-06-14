<script setup>
import { onMounted, onUnmounted, ref } from 'vue'
import Fas100Monitor from './Fas100Monitor.vue'
import { makeWidgetRouting, startTicker } from './storyHelpers.js'

const INPUTS = ['current', 'voltage', 'temp1', 'temp2']

const idle = makeWidgetRouting('fas_idle', 'battery_monitor', INPUTS, { peripheralId: 'fas_idle_p' })
const active = makeWidgetRouting('fas_active', 'battery_monitor', INPUTS, { peripheralId: 'fas_active_p' })

const load = ref(25)

let stopIdle, stopActive
onMounted(() => {
  stopIdle = startTicker(idle.nodeId, idle.peripheralId, () => ({
    current: 0.4,
    voltage: 12.65,
    temp1:   24,
    temp2:   25,
  }))
  stopActive = startTicker(active.nodeId, active.peripheralId, (t) => ({
    current: load.value + Math.sin(t / 4) * 2,
    voltage: 12.2 - load.value * 0.02,
    temp1:   38 + load.value * 0.4 + Math.sin(t / 10),
    temp2:   36 + load.value * 0.35,
  }))
})
onUnmounted(() => { stopIdle?.(); stopActive?.() })
</script>

<template>
  <Story title="Widgets / FAS100 Monitor" :layout="{ type: 'grid', width: 360 }">
    <Variant title="Idle bus">
      <div class="p-3 bg-bg">
        <Fas100Monitor :widget="idle.widget" :routes="idle.routes" />
      </div>
    </Variant>

    <Variant title="Under load (interactive)">
      <template #controls>
        <label class="text-xs text-fg-muted block">Load draw: {{ load }} A</label>
        <input type="range" min="0" max="80" step="1" v-model.number="load" class="w-full" />
      </template>
      <div class="p-3 bg-bg">
        <Fas100Monitor :widget="active.widget" :routes="active.routes" />
      </div>
    </Variant>
  </Story>
</template>
