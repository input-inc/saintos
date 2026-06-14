<script setup>
// Multi-pack overview story. Each variant pre-seeds a fleet of fake
// packs (different mixes of healthy / faulted / low-SOC); the
// controls panel lets you toggle pack count + bend each pack's SOC.
//
// Tapping a pack tile triggers router.push to a pack-detail route —
// in Histoire the URL won't match so it'll noop, but the controller
// in the controls panel reports the last "navigation" so we can
// verify the wiring.
import BatteriesScenarioRunner from './_BatteriesScenarioRunner.vue'
import BatteriesScenarioControls from './_BatteriesScenarioControls.vue'

const PRESETS = {
  healthy: {
    label: 'Healthy fleet',
    packs: [
      { label: 'PACK A', soc: 76, current: -3.5, cellCount: 4, cellNominalV: 3.78, cellSpreadMv: 12, chgFet: true, dsgFet: true, faults: [] },
      { label: 'PACK B', soc: 82, current: -2.1, cellCount: 4, cellNominalV: 3.85, cellSpreadMv: 18, chgFet: true, dsgFet: true, faults: [] },
      { label: 'PACK C', soc: 71, current: -4.2, cellCount: 8, cellNominalV: 3.72, cellSpreadMv: 22, chgFet: true, dsgFet: true, faults: [] },
      { label: 'PACK D', soc: 79, current: -3.0, cellCount: 4, cellNominalV: 3.80, cellSpreadMv: 16, chgFet: true, dsgFet: true, faults: [] },
    ],
  },
  oneLow: {
    label: 'One pack low (operator should notice)',
    packs: [
      { label: 'PACK A', soc: 68, current: -3.5, cellCount: 4, cellNominalV: 3.74, cellSpreadMv: 15, chgFet: true, dsgFet: true, faults: [] },
      { label: 'PACK B', soc: 72, current: -3.5, cellCount: 4, cellNominalV: 3.76, cellSpreadMv: 18, chgFet: true, dsgFet: true, faults: [] },
      { label: 'PACK C', soc: 14, current: -1.0, cellCount: 8, cellNominalV: 3.32, cellSpreadMv: 22, chgFet: true, dsgFet: true, faults: [] },
      { label: 'PACK D', soc: 71, current: -3.5, cellCount: 4, cellNominalV: 3.75, cellSpreadMv: 14, chgFet: true, dsgFet: true, faults: [] },
    ],
  },
  faultedPack: {
    label: 'One pack faulted',
    packs: [
      { label: 'PACK A', soc: 68, current: -3.5, cellCount: 4, cellNominalV: 3.74, cellSpreadMv: 15, chgFet: true, dsgFet: true, faults: [] },
      { label: 'PACK B', soc: 9,  current: -0.1, cellCount: 6, cellNominalV: 4.18, cellSpreadMv: 240, chgFet: false, dsgFet: false, faults: [0, 4] },
      { label: 'PACK C', soc: 72, current: -3.5, cellCount: 8, cellNominalV: 3.76, cellSpreadMv: 22, chgFet: true, dsgFet: true, faults: [] },
      { label: 'PACK D', soc: 75, current: -3.5, cellCount: 4, cellNominalV: 3.78, cellSpreadMv: 16, chgFet: true, dsgFet: true, faults: [] },
    ],
  },
  twoPacks: {
    label: '2-pack rig',
    packs: [
      { label: 'PORT',  soc: 65, current: -3.0, cellCount: 4, cellNominalV: 3.72, cellSpreadMv: 16, chgFet: true, dsgFet: true, faults: [] },
      { label: 'STBD',  soc: 81, current: -2.5, cellCount: 4, cellNominalV: 3.82, cellSpreadMv: 14, chgFet: true, dsgFet: true, faults: [] },
    ],
  },
}

const VARIANTS = [
  { key: 'healthy',     label: PRESETS.healthy.label,     preset: PRESETS.healthy },
  { key: 'oneLow',      label: PRESETS.oneLow.label,      preset: PRESETS.oneLow },
  { key: 'faultedPack', label: PRESETS.faultedPack.label, preset: PRESETS.faultedPack },
  { key: 'twoPacks',    label: PRESETS.twoPacks.label,    preset: PRESETS.twoPacks },
]
</script>

<template>
  <Story title="Console / ConsoleBatteriesOverview" :layout="{ type: 'single' }">
    <Variant v-for="v in VARIANTS" :key="v.key" :title="v.label"
             :init-state="() => ({ packs: v.preset.packs.map(p => ({ ...p, faults: [...p.faults] })) })">
      <template #default="{ state }">
        <BatteriesScenarioRunner :state="state" />
      </template>
      <template #controls="{ state }">
        <BatteriesScenarioControls :state="state" />
      </template>
    </Variant>
  </Story>
</template>
