<script setup>
// Histoire-only — reusable control panel for each ConsoleBatteryView
// variant. Takes a `scenario` reactive object (see the story file for
// shape) and renders sliders / checkboxes that mutate it directly.
// Pinia-reactive props propagate without explicit emits.

defineProps({
  scenario: { type: Object, required: true },
})

// JBD protection-bit names — must stay in sync with PROTECTION_LABELS
// in ConsoleBatteryView / BMSCard / jbd_bms.py.
const PROTECTION_BITS = [
  'cell overvoltage',
  'cell undervoltage',
  'pack overvoltage',
  'pack undervoltage',
  'charge overtemp',
  'charge undertemp',
  'discharge overtemp',
  'discharge undertemp',
  'charge overcurrent',
  'discharge overcurrent',
  'short circuit',
  'IC error',
  'MOS lock',
]

function toggleFault (scenario, bit) {
  const i = scenario.faults.indexOf(bit)
  if (i >= 0) scenario.faults.splice(i, 1)
  else scenario.faults.push(bit)
}
</script>

<template>
  <div class="space-y-3 text-xs">
    <div>
      <label class="block mb-1">SOC: {{ scenario.soc.toFixed(0) }}%</label>
      <input type="range" min="0" max="100" step="1" v-model.number="scenario.soc" class="w-full" />
    </div>
    <div>
      <label class="block mb-1">Current: {{ scenario.current.toFixed(2) }} A
        <span class="text-fg-faint">({{ scenario.current >= 0 ? 'charging' : 'discharging' }})</span>
      </label>
      <input type="range" min="-30" max="30" step="0.1" v-model.number="scenario.current" class="w-full" />
    </div>
    <div class="grid grid-cols-2 gap-2">
      <div>
        <label class="block mb-1">Temp 1: {{ scenario.temp1.toFixed(0) }}°C</label>
        <input type="range" min="-10" max="80" step="1" v-model.number="scenario.temp1" class="w-full" />
      </div>
      <div>
        <label class="block mb-1">Temp 2: {{ scenario.temp2.toFixed(0) }}°C</label>
        <input type="range" min="-10" max="80" step="1" v-model.number="scenario.temp2" class="w-full" />
      </div>
    </div>
    <div class="grid grid-cols-2 gap-2">
      <div>
        <label class="block mb-1">Cells (S): {{ scenario.cellCount }}</label>
        <input type="range" min="1" max="16" step="1" v-model.number="scenario.cellCount" class="w-full" />
      </div>
      <div>
        <label class="block mb-1">Nominal V: {{ scenario.cellNominalV.toFixed(2) }}</label>
        <input type="range" min="2.5" max="4.3" step="0.01" v-model.number="scenario.cellNominalV" class="w-full" />
      </div>
    </div>
    <div>
      <label class="block mb-1">Cell spread: {{ scenario.cellSpreadMv }} mV
        <span v-if="scenario.cellSpreadMv > 50" class="text-amber-300">(imbalance)</span>
      </label>
      <input type="range" min="0" max="500" step="5" v-model.number="scenario.cellSpreadMv" class="w-full" />
    </div>
    <div class="flex items-center gap-4">
      <label class="flex items-center gap-1">
        <input type="checkbox" v-model="scenario.chgFet" /> CHG FET
      </label>
      <label class="flex items-center gap-1">
        <input type="checkbox" v-model="scenario.dsgFet" /> DSG FET
      </label>
    </div>
    <fieldset class="border border-line-strong rounded p-2">
      <legend class="px-1 text-fg-muted">Protection bits</legend>
      <div class="grid grid-cols-1 gap-0.5">
        <label v-for="(name, i) in PROTECTION_BITS" :key="i" class="flex items-center gap-1">
          <input type="checkbox"
                 :checked="scenario.faults.includes(i)"
                 @change="toggleFault(scenario, i)" />
          <span class="text-[11px]">{{ name }}</span>
        </label>
      </div>
    </fieldset>
  </div>
</template>
