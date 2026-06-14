<script setup>
import { ref } from 'vue'
import ServoExtentsControl from './ServoExtentsControl.vue'

// Three variants:
//   • Default — the catalog's out-of-box numbers (1000/2000/1500/1500).
//   • Asymmetric — center off-midpoint to demonstrate that the dial
//     supports lopsided ranges (common when a servo mounts at an
//     angle that puts mechanical zero off the geometric midpoint).
//   • Reversed — start > end, simulating a servo mounted backwards.
const defaultValue    = ref({ start_us: 1000, end_us: 2000, center_us: 1500, home_us: 1500 })
const asymmetricValue = ref({ start_us:  900, end_us: 2200, center_us: 1700, home_us: 1100 })
const reversedValue   = ref({ start_us: 2100, end_us:  900, center_us: 1500, home_us: 1500 })
</script>

<template>
  <Story title="Peripherals / ServoExtentsControl" :layout="{ type: 'grid', width: 480 }">
    <Variant title="Default extents">
      <div class="card p-4">
        <ServoExtentsControl v-model="defaultValue" />
        <pre class="mt-3 text-xs text-fg-faint font-mono">{{ defaultValue }}</pre>
      </div>
    </Variant>

    <Variant title="Asymmetric (center off-midpoint)">
      <div class="card p-4">
        <ServoExtentsControl v-model="asymmetricValue" />
        <pre class="mt-3 text-xs text-fg-faint font-mono">{{ asymmetricValue }}</pre>
      </div>
    </Variant>

    <Variant title="Reversed (start > end)">
      <div class="card p-4">
        <ServoExtentsControl v-model="reversedValue" />
        <pre class="mt-3 text-xs text-fg-faint font-mono">{{ reversedValue }}</pre>
      </div>
    </Variant>
  </Story>
</template>
