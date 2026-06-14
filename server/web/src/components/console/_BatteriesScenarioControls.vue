<script setup>
// Histoire-only — per-variant controls for the multi-pack overview.
// Each pack has its own SOC + cell-spread + FET toggles; faults are
// edited as a comma-separated list of bit indices (less visually
// busy than 13 checkboxes per pack × 4 packs).

defineProps({
  state: { type: Object, required: true },
})

const PROTECTION_BITS = [
  'cell ov', 'cell uv', 'pack ov', 'pack uv',
  'chg ot',  'chg ut',  'dsg ot',  'dsg ut',
  'chg oc',  'dsg oc',  'short',   'ic err', 'mos lock',
]

function toggleFault (pack, bit) {
  const i = pack.faults.indexOf(bit)
  if (i >= 0) pack.faults.splice(i, 1)
  else pack.faults.push(bit)
}
</script>

<template>
  <div class="space-y-3 text-xs">
    <div v-for="(p, i) in state.packs" :key="i"
         class="border border-line-strong rounded p-2 space-y-2">
      <div class="font-bold text-fg-strong">{{ p.label || `Pack ${i + 1}` }}</div>

      <div>
        <label class="block mb-0.5">SOC: {{ p.soc.toFixed(0) }}%</label>
        <input type="range" min="0" max="100" step="1" v-model.number="p.soc" class="w-full" />
      </div>
      <div>
        <label class="block mb-0.5">Current: {{ p.current.toFixed(2) }} A</label>
        <input type="range" min="-30" max="30" step="0.1" v-model.number="p.current" class="w-full" />
      </div>
      <div class="grid grid-cols-2 gap-2">
        <div>
          <label class="block mb-0.5">Cells: {{ p.cellCount }}</label>
          <input type="range" min="1" max="16" step="1" v-model.number="p.cellCount" class="w-full" />
        </div>
        <div>
          <label class="block mb-0.5">Spread: {{ p.cellSpreadMv }}mV</label>
          <input type="range" min="0" max="500" step="5" v-model.number="p.cellSpreadMv" class="w-full" />
        </div>
      </div>
      <div class="flex items-center gap-3">
        <label class="flex items-center gap-1">
          <input type="checkbox" v-model="p.chgFet" /> CHG
        </label>
        <label class="flex items-center gap-1">
          <input type="checkbox" v-model="p.dsgFet" /> DSG
        </label>
      </div>
      <div>
        <div class="text-[10px] text-fg-muted mb-0.5">Faults (tap to toggle)</div>
        <div class="flex flex-wrap gap-1">
          <button v-for="(name, bit) in PROTECTION_BITS" :key="bit"
                  class="px-1.5 py-0.5 text-[10px] rounded border"
                  :class="p.faults.includes(bit)
                    ? 'border-red-500 bg-red-500/30 text-red-200'
                    : 'border-line-strong text-fg-muted'"
                  @click="toggleFault(p, bit)">
            {{ name }}
          </button>
        </div>
      </div>
    </div>
  </div>
</template>
