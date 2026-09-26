<script setup>
/**
 * A number input that lets the operator finish typing.
 *
 * The problem it replaces: the animation editor's numeric fields were
 * fully controlled — `:value="fmt2(x)"` plus an `@input` handler that
 * parsed, rounded and wrote straight back to the model. Because Vue
 * patches `el.value` whenever the bound string changes, every keystroke
 * rewrote the text under the caret. On a native `type="number"` that is
 * unusable: the element reports `value === ""` for any partially-typed
 * number ("1.", "-", "1e"), so typing the "." in "2.5" fed `Number("")`
 * → 0 → the field snapped to "0.00" mid-word. Decimals were literally
 * untypeable, and it read as the field "auto-completing" numbers.
 *
 * The fix is to stop re-rendering the text while it is being edited.
 * On focus we freeze the displayed string; from then until blur the
 * browser owns what is in the box and Vue's patch is a no-op (the bound
 * string never changes). Parseable intermediate states still stream out
 * as `update:modelValue`, so live preview — servo follows the number,
 * gizmo follows the joint — keeps working. Rounding and min/max
 * clamping happen once, on commit (blur, Enter, or the native change
 * event), which is also the only time the display is allowed to
 * reformat.
 *
 * Unparseable or empty input is simply not emitted: leaving a field
 * blank keeps the last good value rather than writing 0 or "" into the
 * animation.
 */
import { computed, ref } from 'vue'

const props = defineProps({
  modelValue: { type: [Number, String], default: 0 },
  // Decimal places for the *displayed* and committed value.
  // null = round nothing, show the number as-is.
  decimals: { type: Number, default: null },
  min: { type: [Number, String], default: null },
  max: { type: [Number, String], default: null },
  step: { type: [Number, String], default: 'any' },
})

const emit = defineEmits(['update:modelValue', 'commit'])

// Non-null only while the field has focus. Holding the string the box
// started with keeps the bound value stable across re-renders, which is
// what stops Vue from clobbering the caret.
const frozen = ref(null)

const display = computed(() => {
  if (frozen.value !== null) return frozen.value
  const v = Number(props.modelValue)
  if (!Number.isFinite(v)) return ''
  return props.decimals === null ? String(v) : v.toFixed(props.decimals)
})

function clamp (v) {
  const lo = props.min === null || props.min === '' ? null : Number(props.min)
  const hi = props.max === null || props.max === '' ? null : Number(props.max)
  if (lo !== null && Number.isFinite(lo)) v = Math.max(lo, v)
  if (hi !== null && Number.isFinite(hi)) v = Math.min(hi, v)
  return v
}

function onFocus (e) {
  frozen.value = e.target.value
}

function onInput (e) {
  // Track what the box holds so `display` keeps matching it — if these
  // ever diverge Vue will patch the DOM and eat the caret.
  frozen.value = e.target.value
  // "" is a partially-typed number, not a zero. Wait for the rest.
  if (e.target.value === '') return
  const v = Number(e.target.value)
  if (!Number.isFinite(v)) return
  // Live, un-rounded and un-clamped: the operator is still typing and
  // "0.0" on the way to "0.05" must not be snapped to a step boundary.
  emit('update:modelValue', v)
}

function onCommit (e) {
  const raw = e.target.value
  // Hand the display back to the model, reformatted.
  frozen.value = null
  if (raw === '') { emit('commit', Number(props.modelValue)); return }
  let v = Number(raw)
  if (!Number.isFinite(v)) { emit('commit', Number(props.modelValue)); return }
  if (props.decimals !== null) v = Number(v.toFixed(props.decimals))
  v = clamp(v)
  emit('update:modelValue', v)
  emit('commit', v)
}
</script>

<template>
  <input type="number"
         :step="step"
         :min="min === null ? undefined : min"
         :max="max === null ? undefined : max"
         :value="display"
         autocomplete="off"
         @focus="onFocus"
         @input="onInput"
         @change="onCommit"
         @blur="onCommit"
         @keydown.enter="e => e.target.blur()" />
</template>
