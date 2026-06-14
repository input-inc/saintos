<script setup>
import { ref } from 'vue'
import ConsoleScreen from './ConsoleScreen.vue'
import KioskFrame from './_KioskFrame.vue'

const clickLog = ref([])
function log (label) { clickLog.value.unshift(`${new Date().toLocaleTimeString()} · ${label}`) }
</script>

<template>
  <Story title="Console / ConsoleScreen" :layout="{ type: 'single' }">
    <Variant title="Default chrome">
      <KioskFrame>
        <ConsoleScreen title="TITLE" subtitle="ONLINE" :buttons="[
          { label: 'Activate', onClick: () => log('Activate') },
          { label: 'System',   onClick: () => log('System')   },
        ]">
          <div class="space-y-6 text-3xl tracking-widest">
            <p class="led-text-plain">CONTENT GOES HERE.</p>
            <p class="led-text-dim">DIM (UNLIT) SECONDARY TEXT.</p>
            <p class="led-text-amber-plain">CAUTION ROW</p>
            <p class="led-text-green-plain">OK ROW</p>
          </div>
        </ConsoleScreen>
      </KioskFrame>
      <div class="mt-3 text-xs font-mono">
        <div v-for="(e, i) in clickLog.slice(0, 5)" :key="i">{{ e }}</div>
      </div>
    </Variant>

    <Variant title="Two-button override">
      <KioskFrame>
        <ConsoleScreen title="SETUP" subtitle="EDIT MODE" :buttons="[
          { label: 'Save',   onClick: () => log('Save')   },
          { label: 'Cancel', onClick: () => log('Cancel') },
        ]">
          <p class="led-text-plain text-3xl">
            The bar always renders exactly two equal-width buttons.
            Pass labels and handlers via :buttons; pass fewer and the
            defaults (Activate / System) backfill; pass more and the
            extras are silently dropped.
          </p>
        </ConsoleScreen>
      </KioskFrame>
    </Variant>

    <Variant title="Flat background (no dot-matrix grid)">
      <KioskFrame>
        <ConsoleScreen title="DENSE" :flat-background="true">
          <p class="led-text-plain text-2xl">For dense screens where the dot-matrix grid would create read noise.</p>
        </ConsoleScreen>
      </KioskFrame>
    </Variant>
  </Story>
</template>
