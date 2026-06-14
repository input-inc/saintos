<script setup>
// Each Variant declares its own `initState` (a function returning the
// initial scenario). Histoire keeps that state in sync between the
// per-variant sandbox app (where the default slot mounts) and the
// outer Histoire UI (where the #controls slot renders) via its
// applyState / syncStateBundledAndExternal bridge. Both slots receive
// the same reactive `state` via scoped-slot props — moving a slider
// in the sidebar propagates through to the ticker inside the variant
// body.
//
// See @histoire/plugin-vue/src/client/app/RenderStory.ts for the
// bridge implementation. The pattern matters because each variant
// runs in its own Vue app via createApp(...) — script-setup state
// from this file lives in the OUTER app, not the per-variant
// sandbox, so direct references would not survive the boundary.
import BatteryScenarioControls from './_BatteryScenarioControls.vue'
import BatteryScenarioRunner from './_BatteryScenarioRunner.vue'

const PRESETS = {
  healthy:    { soc: 80, current: -3.5, temp1: 32, temp2: 30,
                cellCount: 4,  cellNominalV: 3.7,  cellSpreadMv:  20,
                chgFet: true,  dsgFet: true,  faults: [] },
  lowSoc:     { soc: 14, current: -1.2, temp1: 30, temp2: 28,
                cellCount: 4,  cellNominalV: 3.3,  cellSpreadMv:  18,
                chgFet: true,  dsgFet: true,  faults: [] },
  isolated:   { soc: 46, current:  0.0, temp1: 30, temp2: 29,
                cellCount: 8,  cellNominalV: 3.7,  cellSpreadMv:  30,
                chgFet: false, dsgFet: true,  faults: [] },
  imbalanced: { soc: 62, current: -2.0, temp1: 35, temp2: 34,
                cellCount: 12, cellNominalV: 3.75, cellSpreadMv: 180,
                chgFet: true,  dsgFet: true,  faults: [] },
  faulted:    { soc:  9, current: -0.1, temp1: 58, temp2: 55,
                cellCount: 6,  cellNominalV: 4.2,  cellSpreadMv: 320,
                chgFet: false, dsgFet: false, faults: [0, 4] },
}

const VARIANTS = [
  { key: 'healthy',    label: 'Healthy',                          nodeId: 'story_bms_healthy',    preset: PRESETS.healthy },
  { key: 'lowSoc',     label: 'Low SOC',                          nodeId: 'story_bms_low_soc',    preset: PRESETS.lowSoc },
  { key: 'isolated',   label: 'Isolated (CHG FET open)',          nodeId: 'story_bms_isolated',   preset: PRESETS.isolated },
  { key: 'imbalanced', label: 'Imbalanced (Δ 180 mV)',            nodeId: 'story_bms_imbalanced', preset: PRESETS.imbalanced },
  { key: 'faulted',    label: 'Faulted (overvoltage + overtemp)', nodeId: 'story_bms_faulted',    preset: PRESETS.faulted },
]
</script>

<template>
  <Story title="Console / ConsoleBatteryView" :layout="{ type: 'single' }">
    <Variant v-for="v in VARIANTS" :key="v.key" :title="v.label"
             :init-state="() => ({ ...v.preset, faults: [...v.preset.faults] })">
      <template #default="{ state }">
        <BatteryScenarioRunner :state="state" :node-id="v.nodeId" />
      </template>
      <template #controls="{ state }">
        <BatteryScenarioControls :scenario="state" />
      </template>
    </Variant>
  </Story>
</template>
