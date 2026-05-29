<script setup>
import { onBeforeUnmount, onMounted, ref } from 'vue'

// Compact icon picker. Shows the currently-selected material-icon as
// a clickable swatch; clicking opens a curated grid + a free-text
// "custom name" input so power users can type any icon from the
// material-icons set (https://fonts.google.com/icons).

const props = defineProps({
  modelValue: { type: String, default: '' },     // current icon name
  fallback:   { type: String, default: 'animation' },
})
const emit = defineEmits(['update:modelValue'])

// Curated set covering common robot-animation themes. Operators
// looking for something exotic can type it into the custom field.
const CURATED = [
  'animation', 'motion_photos_on', 'play_circle', 'replay',
  'accessibility', 'accessibility_new', 'self_improvement',
  'directions_walk', 'directions_run', 'sports_kabaddi',
  'waving_hand', 'pan_tool', 'thumb_up', 'thumb_down', 'handshake',
  'mood', 'sentiment_satisfied', 'sentiment_neutral', 'sentiment_dissatisfied',
  'emoji_emotions', 'celebration', 'cake',
  'precision_manufacturing', 'smart_toy', 'adb', 'memory',
  'bolt', 'flag', 'star', 'favorite', 'military_tech',
  'pets', 'nights_stay', 'wb_sunny',
  'music_note', 'queue_music', 'record_voice_over', 'mic',
  'visibility', 'visibility_off', 'volume_up',
]

const open = ref(false)
const customName = ref('')

function pick (name) {
  emit('update:modelValue', name)
  open.value = false
}

function applyCustom () {
  const trimmed = customName.value.trim()
  if (trimmed) pick(trimmed)
  customName.value = ''
}

function onDocClick (e) {
  if (!open.value) return
  if (!e.target.closest('.icon-picker')) open.value = false
}

onMounted(() => document.addEventListener('click', onDocClick))
onBeforeUnmount(() => document.removeEventListener('click', onDocClick))
</script>

<template>
  <div class="icon-picker relative">
    <button class="icon-swatch" type="button"
            :title="`Change icon — current: ${modelValue || fallback}`"
            @click.stop="open = !open">
      <span class="material-icons">{{ modelValue || fallback }}</span>
      <span class="material-icons icon-sm chev">expand_more</span>
    </button>
    <div v-if="open" class="icon-menu" @click.stop>
      <div class="icon-grid">
        <button v-for="name in CURATED" :key="name"
                :class="['icon-cell', name === modelValue ? 'is-active' : '']"
                :title="name"
                type="button"
                @click="pick(name)">
          <span class="material-icons">{{ name }}</span>
        </button>
      </div>
      <div class="border-t border-line/60 mt-2 pt-2">
        <label class="block text-[10px] uppercase tracking-wide text-fg-faint mb-1">Custom</label>
        <div class="flex gap-1">
          <input v-model="customName"
                 class="input-field flex-1 text-xs py-1"
                 placeholder="material-icon name (e.g. rocket_launch)"
                 @keydown.enter="applyCustom" />
          <button class="btn-sm bg-cyan-600 hover:bg-cyan-500 text-white"
                  type="button"
                  @click="applyCustom">Use</button>
        </div>
      </div>
    </div>
  </div>
</template>

<style scoped>
.icon-swatch {
  display: inline-flex; align-items: center; gap: 4px;
  padding: 4px 6px;
  background: var(--color-canvas);
  border: 1px solid var(--color-surface);
  border-radius: 6px;
  color: var(--color-fg);
  cursor: pointer;
  transition: border-color 0.1s;
}
.icon-swatch:hover { border-color: #0e7490; }
.icon-swatch .material-icons { font-size: 20px; }
.icon-swatch .chev { font-size: 14px; color: var(--color-fg-faint); }

.icon-menu {
  position: absolute;
  top: 100%;
  left: 0;
  margin-top: 4px;
  width: 280px;
  background: var(--color-canvas);
  border: 1px solid var(--color-surface);
  border-radius: 8px;
  box-shadow: 0 18px 50px -12px rgba(0,0,0,0.7);
  padding: 8px;
  z-index: 50;
}
.icon-grid {
  display: grid;
  grid-template-columns: repeat(7, 1fr);
  gap: 2px;
  max-height: 280px;
  overflow-y: auto;
}
.icon-cell {
  display: inline-flex; align-items: center; justify-content: center;
  width: 100%; aspect-ratio: 1;
  border-radius: 4px;
  color: var(--color-fg-muted);
  cursor: pointer;
  transition: background-color 0.1s, color 0.1s;
}
/* Hover paints the cell on a `surface` tone (slate-200 in light,
   slate-700 in dark) with `fg-strong` foreground — works in both
   themes. The previous version literally used `color: white` which
   left hovered icons invisible against the white menu panel in
   light mode. */
.icon-cell:hover { background: var(--color-surface); color: var(--color-fg-strong); }
.icon-cell.is-active {
  background: rgba(6, 182, 212, 0.2);
  color: var(--color-cyan-300);
}
.icon-cell .material-icons { font-size: 18px; }
</style>
