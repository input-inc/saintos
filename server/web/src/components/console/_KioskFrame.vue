<script setup>
// Story-only — responsive 1440×2560 kiosk frame.
//
// The outer wrapper takes 100 % of its container's width (clamped to
// the kiosk's native 1440 px) and locks its height to the kiosk's
// 9 : 16 aspect via `aspect-ratio`. The inner element always renders
// at the literal hardware pixel dimensions (1440 × 2560) so font
// sizes / paddings / line widths inside it are tuned for the real
// kiosk; a ResizeObserver computes the scale factor needed to fit
// that 1440-wide content into whatever width the outer ended up at,
// and applies it via `transform: scale(...)`.
//
// Use anywhere a Console-target story needs to be previewed — the
// preview matches the hardware ratio and stays accurate as the
// operator resizes Histoire's panels.

import { onMounted, onUnmounted, ref } from 'vue'

const KIOSK_W = 1440
const KIOSK_H = 2560

const outer = ref(null)
const scale = ref(1)

let ro = null
onMounted(() => {
  if (!outer.value) return
  // Initial read — ResizeObserver fires once immediately but having
  // a synchronous read here prevents a one-frame flash at the
  // default scale=1 (which would render the inner overflowing).
  scale.value = outer.value.clientWidth / KIOSK_W
  ro = new ResizeObserver((entries) => {
    for (const e of entries) {
      const w = e.contentRect.width
      if (w > 0) scale.value = w / KIOSK_W
    }
  })
  ro.observe(outer.value)
})
onUnmounted(() => { if (ro) ro.disconnect() })
</script>

<template>
  <div ref="outer"
       class="relative overflow-hidden bg-black w-full"
       style="aspect-ratio: 1440 / 2560; max-width: 1440px;">
    <div class="absolute top-0 left-0"
         :style="{
           width: KIOSK_W + 'px',
           height: KIOSK_H + 'px',
           transform: `scale(${scale})`,
           transformOrigin: 'top left',
         }">
      <slot />
    </div>
  </div>
</template>
