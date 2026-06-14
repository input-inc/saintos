<script setup>
import { onMounted, onUnmounted, ref } from 'vue'
import RoboClawMonitor from './RoboClawMonitor.vue'
import { makeWidgetRouting, startTicker } from './storyHelpers.js'

const INPUTS = ['motor', 'encoder', 'voltage', 'current', 'temp']

const idle = makeWidgetRouting('rc_idle', 'roboclaw_monitor', INPUTS, { peripheralId: 'rc_idle_p' })
const driving = makeWidgetRouting('rc_driving', 'roboclaw_monitor', INPUTS, { peripheralId: 'rc_driving_p' })

const duty = ref(0.55)
let encoderPos = 0

let stopIdle, stopDriving
onMounted(() => {
  stopIdle = startTicker(idle.nodeId, idle.peripheralId, () => ({
    motor: 0,
    encoder: 0,
    voltage: 24.2,
    current: 0.05,
    temp: 28,
  }))
  stopDriving = startTicker(driving.nodeId, driving.peripheralId, (t) => {
    // Encoder integrates commanded duty over time so the readout
    // looks like a real driven shaft, not a free-floating number.
    encoderPos += duty.value * 100
    return {
      motor: duty.value,
      encoder: Math.round(encoderPos),
      voltage: 24.0 - Math.abs(duty.value) * 0.6 + Math.sin(t / 6) * 0.05,
      current: Math.abs(duty.value) * 18 + Math.sin(t / 3) * 0.4,
      temp: 30 + Math.abs(duty.value) * 25,
    }
  })
})
onUnmounted(() => { stopIdle?.(); stopDriving?.() })
</script>

<template>
  <Story title="Widgets / RoboClaw Motor Monitor" :layout="{ type: 'grid', width: 380 }">
    <Variant title="Idle (zero duty)">
      <div class="p-3 bg-bg">
        <RoboClawMonitor :widget="idle.widget" :routes="idle.routes" />
      </div>
    </Variant>

    <Variant title="Driving (interactive duty)">
      <template #controls>
        <label class="text-xs text-fg-muted block">Duty: {{ duty.toFixed(2) }}</label>
        <input type="range" min="-1" max="1" step="0.05" v-model.number="duty" class="w-full" />
      </template>
      <div class="p-3 bg-bg">
        <RoboClawMonitor :widget="driving.widget" :routes="driving.routes" />
      </div>
    </Variant>
  </Story>
</template>
