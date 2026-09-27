<script setup>
import { computed, inject, reactive, ref, watch } from 'vue'
import {
  INTERP_CUBIC, sampleCurve,
} from '@/composables/useCurveSampling'
import { EASING_CATEGORIES } from '@/composables/easings'
import { useAnimationsStore } from '@/stores/animations'
import IconPicker from './IconPicker.vue'
import NumberField from '@/components/NumberField.vue'

const animations = useAnimationsStore()

// Right-rail content. Shape switches on `selection.kind`:
//   - "keyframe" → time/value/interp editor for the selected key
//   - "track"    → metadata for the selected track
//   - "joint"    → manual-rotation slider + bind/keyframe controls
// Renders nothing if no selection is held.

const props = defineProps({
  animation: { type: Object, required: true },
  selection: { type: Object, default: () => ({ kind: null }) },
})

const emit = defineEmits([
  'dirty',
  'delete-keyframe',
  'delete-override-key',
  'select',
])

// Read-only 2-decimal formatting for inline text (e.g. the clip range
// label). The editable Time / Value fields use <NumberField>, which
// owns its own display formatting — it has to, because reformatting
// while the operator is mid-keystroke is exactly the bug it exists to
// prevent. See components/NumberField.vue.
function fmt2 (n) {
  const v = Number(n)
  return Number.isFinite(v) ? v.toFixed(2) : '0.00'
}

const viewerRef        = inject('urdf-viewer', null)
const playerPos        = inject('player-pos', ref(0))
const setKeyframeAtPH  = inject('set-keyframe-at-playhead', () => null)
// When the 3D viewer click resolves to more than one co-located joint
// (e.g. a shoulder with separate Pitch and Roll), this holds the full
// list so the operator can switch between them without re-clicking.
const jointAlternatives = inject('joint-alternatives', ref([]))
// WS-input + ROS-topic catalogs feed the trigger-keyframe target
// pickers. Lazily loaded the first time we surface a trigger key.
const triggerTargets = inject('trigger-targets', {
  wsInputs: ref([]), topics: ref([]), load: () => {},
})
// Live joint rotation pushed in by the 3D viewer's gizmo — when set,
// our slider tracks the gizmo in real time instead of staying pinned
// to the sampled track value.
const liveJointAngle   = inject('live-joint-angle', ref({ name: null, angle: 0 }))

// Selected trigger track + keyframe.
const selectedTriggerTrack = computed(() => {
  const s = props.selection
  if (s?.kind !== 'trigger-keyframe' && s?.kind !== 'trigger-track') return null
  return props.animation.trigger_tracks?.find(t => t.id === s.trackId) || null
})
const selectedTriggerKey = computed(() => {
  const s = props.selection
  if (s?.kind !== 'trigger-keyframe' || !selectedTriggerTrack.value) return null
  return selectedTriggerTrack.value.keyframes?.[s.kfIdx] || null
})
// Load the target catalog on demand so dev mode doesn't always pay
// the ws round-trip if the operator never opens a trigger.
watch(selectedTriggerKey, (kf) => {
  if (kf) triggerTargets.load?.()
}, { immediate: true })

const selectedTrack = computed(() => {
  const s = props.selection
  if (!s || (s.kind !== 'keyframe' && s.kind !== 'track')) return null
  return props.animation.value_tracks?.find(t => t.id === s.trackId) || null
})
const selectedKeyframe = computed(() => {
  const s = props.selection
  if (s?.kind !== 'keyframe' || !selectedTrack.value) return null
  return selectedTrack.value.curve?.keys?.[s.kfIdx] || null
})
// ── Per-joint override keys on a pose track ────────────────────────
//
// A pose track's disclosed joint rows carry the operator's own keys
// between the locked anchors. The anchors aren't editable here — the pose
// owns those times — so this editor only ever sees a user key.

const overrideTrack = computed(() => {
  const s = props.selection
  if (s?.kind !== 'override-keyframe') return null
  return props.animation.value_tracks?.find(t => t.id === s.trackId) || null
})
const overrideKeys = computed(() =>
  overrideTrack.value?.joint_overrides?.[props.selection?.joint]?.keys || null)
const selectedOverrideKey = computed(() => {
  const keys = overrideKeys.value
  const t = props.selection?.time
  if (!keys || t === undefined) return null
  return keys.find(k => Math.abs(k.time - t) < 1e-6) || null
})
// Keys only evaluate inside the pose track's clip, so time edits clamp
// to it rather than letting a key wander somewhere it does nothing.
const overrideClip = computed(() => {
  const keys = overrideTrack.value?.curve?.keys || []
  if (!keys.length) return null
  const times = keys.map(k => Number(k.time) || 0)
  return [Math.min(...times), Math.max(...times)]
})

function setOverrideTime (raw) {
  const key = selectedOverrideKey.value
  if (!key) return
  let t = Number(Number(raw).toFixed(3))
  if (!Number.isFinite(t)) return
  const clip = overrideClip.value
  if (clip) t = Math.max(clip[0], Math.min(clip[1], t))
  key.time = t
  overrideKeys.value.sort((a, b) => a.time - b.time)
  onChange()
  // Selection is keyed by time, so it has to follow the edit or the
  // panel would blank out mid-type.
  emit('select', { ...props.selection, time: t })
}

function deleteOverrideKey () {
  const s = props.selection
  if (s?.kind !== 'override-keyframe') return
  animations.snapshot({ force: true })
  emit('delete-override-key', { trackId: s.trackId, joint: s.joint, time: s.time })
  emit('select', null)
}

// Both ws_input tracks and URDF-joint tracks now carry −1..+1 control
// scalars, so the keyframe value editor uses the same range for either.
const selectedTrackIsWs = computed(() =>
  selectedTrack.value?.target_kind === 'ws_input')
const valueSliderRange = computed(() => selectedTrackIsWs.value ? 1 : JOINT_RANGE)

// Flatten the categorized catalog into <optgroup>s for the native
// <select>. Native grouping handles long lists better than a custom
// dropdown for a one-off picker.
const easingGroups = EASING_CATEGORIES

// Keystrokes in inputs coalesce into one undo step via the store's
// 500 ms snapshot window — typing a number doesn't pile up history.
function onChange () {
  animations.snapshot()
  emit('dirty')
}
function deleteKey () {
  const s = props.selection
  if (s?.kind !== 'keyframe') return
  animations.snapshot({ force: true })
  emit('delete-keyframe', s.trackId, s.kfIdx)
  emit('select', null)
}

// Channel list for the selected trigger topic — fed into the field
// dropdown so the operator picks a known field instead of typing
// one in.
const triggerTopicChannels = computed(() => {
  const topic = selectedTriggerKey.value?.target?.[0] || ''
  if (!topic) return []
  return triggerTargets.topics.value
    .find(t => t.topic === topic)?.channels || []
})

// Trigger value coercion: numeric strings round-trip as floats so
// the server-side animation player ends up dispatching ints/floats
// the way set_ws_input / set_topic_channel expect; anything else
// passes through as a string (works for topic-field targets that
// accept string types).
function coerceTriggerValue (raw) {
  const s = String(raw ?? '').trim()
  if (s === '') return ''
  const n = Number(s)
  return Number.isFinite(n) && s.match(/^-?\d+(?:\.\d+)?$/) ? n : s
}
function deleteTriggerKey () {
  const s = props.selection
  if (s?.kind !== 'trigger-keyframe' || !selectedTriggerTrack.value) return
  animations.snapshot({ force: true })
  selectedTriggerTrack.value.keyframes.splice(s.kfIdx, 1)
  emit('dirty')
  emit('select', null)
}

// ── Joint mode ──────────────────────────────────────────────────────

const jointName = computed(() =>
  props.selection?.kind === 'joint' ? props.selection.value : null
)

// When the 3D click resolved a multi-DOF cluster (shoulder, hip,
// wrist…), we render a full slider + number + keyframe button for
// EACH co-located joint so the operator can pose every DOF without
// re-clicking. Single-joint picks fall back to a one-card view.
const jointList = computed(() => {
  if (jointAlternatives.value.length > 1) return jointAlternatives.value
  return jointName.value ? [jointName.value] : []
})
function trackForJoint (name) {
  return props.animation.value_tracks?.find(t => t.id === name) || null
}

// Per-joint live angle. Reactive so each card's slider + number box
// can v-model independently and writes are reflected back when the
// 3D gizmo or the scrub-time curve sample wants to update them.
const jointAngles = reactive({})
function syncJointAngles () {
  for (const name of jointList.value) {
    const t = trackForJoint(name)
    jointAngles[name] = t
      ? Number(sampleCurve(t.curve, Number(playerPos.value) || 0).toFixed(3))
      : 0
  }
  // Drop angles for joints no longer in the cluster (e.g. clicking a
  // different body part).
  for (const k of Object.keys(jointAngles)) {
    if (!jointList.value.includes(k)) delete jointAngles[k]
  }
}
watch(jointList, syncJointAngles, { immediate: true })
watch(playerPos, syncJointAngles)
// Gizmo updates win over the sampled-track value — the operator is
// actively setting the angle by hand and the slider should follow.
watch(liveJointAngle, ({ name, angle }) => {
  if (name && name in jointAngles) jointAngles[name] = angle
}, { deep: true })

function onJointSlider (name) {
  const v = viewerRef?.value
  if (v?.setJointValue) v.setJointValue(name, jointAngles[name])
}

function setJointKeyframe (name) {
  animations.snapshot({ force: true })
  setKeyframeAtPH(name, jointAngles[name])
  emit('dirty')
}

function makeJointActive (name) {
  emit('select', { kind: 'joint', value: name })
}

// Joint control range. SaintOS drives URDF joints in the same −1..+1
// range as the servos (mapped onto each joint's URDF <limit>, home
// centered at 0 — see URDFViewer's denormJoint). So both the pose
// slider and the keyframe-value slider run −1..+1 regardless of joint
// type (revolute radians vs prismatic metres — the viewer converts).
const JOINT_RANGE = 1

function onIconChange (value) {
  if (!props.animation) return
  animations.snapshot()
  props.animation.icon = value
  emit('dirty')
}
</script>

<template>
  <div class="space-y-3 text-sm">
    <!-- Keyframe editor -->
    <template v-if="selection?.kind === 'keyframe' && selectedKeyframe">
      <div class="text-xs text-fg-muted">
        Track <span class="text-fg-strong">{{ selectedTrack?.name || selectedTrack?.id }}</span>
        · keyframe #{{ selection.kfIdx + 1 }}
      </div>

      <label class="block">
        <span class="block text-fg-muted text-xs mb-1">Time (s)</span>
        <NumberField class="input-field w-full"
                     step="0.01" :decimals="2" :min="0" :max="animation.duration"
                     :model-value="selectedKeyframe.time"
                     @update:model-value="v => { selectedKeyframe.time = v; onChange() }" />
      </label>

      <label class="block">
        <span class="block text-fg-muted text-xs mb-1">{{
          selectedTrackIsWs ? 'Value (−1 to 1 controller input)' : 'Value (drag to pose the joint)' }}</span>
        <div class="space-y-1">
          <input type="range" :min="-valueSliderRange" :max="valueSliderRange" step="0.005"
                 v-model.number="selectedKeyframe.value"
                 @input="onChange"
                 class="w-full accent-cyan-400" />
          <NumberField class="input-field w-full"
                       step="0.01" :decimals="2"
                       :model-value="selectedKeyframe.value"
                       @update:model-value="v => { selectedKeyframe.value = v; onChange() }" />
        </div>
      </label>

      <label class="block">
        <span class="block text-fg-muted text-xs mb-1">Interpolation</span>
        <select class="input-field w-full"
                v-model.number="selectedKeyframe.interp"
                @change="onChange">
          <optgroup v-for="g in easingGroups" :key="g.category" :label="g.category">
            <option v-for="o in g.items" :key="o.code" :value="o.code">{{ o.label }}</option>
          </optgroup>
        </select>
      </label>

      <div v-if="selectedKeyframe.interp === INTERP_CUBIC" class="grid grid-cols-2 gap-2">
        <label class="block">
          <span class="block text-fg-muted text-xs mb-1">Arrive tangent</span>
          <input type="number" step="0.01" class="input-field w-full"
                 v-model.number="selectedKeyframe.arrive_tangent" @input="onChange" />
        </label>
        <label class="block">
          <span class="block text-fg-muted text-xs mb-1">Leave tangent</span>
          <input type="number" step="0.01" class="input-field w-full"
                 v-model.number="selectedKeyframe.leave_tangent" @input="onChange" />
        </label>
      </div>

      <button class="btn-sm w-full bg-surface hover:bg-red-600 text-fg-strong hover:text-fg-strong justify-center"
              @click="deleteKey">
        <span class="material-icons icon-sm">delete</span>
        Delete keyframe
      </button>
    </template>

    <!-- Per-joint override key on a pose track's disclosed row -->
    <template v-else-if="selection?.kind === 'override-keyframe' && selectedOverrideKey">
      <div class="text-xs text-fg-muted">
        Pose <span class="text-fg-strong">{{ overrideTrack?.name || overrideTrack?.id }}</span>
        · joint <span class="text-fg-strong font-mono">{{ selection.joint }}</span>
      </div>
      <p class="text-[11px] text-fg-faint">
        Overrides this one joint inside the clip. The pose keeps driving every
        other joint it names; the locked anchors are set on the pose row.
      </p>

      <label class="block">
        <span class="block text-fg-muted text-xs mb-1">
          Time (s)<span v-if="overrideClip" class="text-fg-faint">
            — clip {{ fmt2(overrideClip[0]) }}–{{ fmt2(overrideClip[1]) }}</span>
        </span>
        <NumberField class="input-field w-full"
                     step="0.01" :decimals="3"
                     :min="overrideClip ? overrideClip[0] : 0"
                     :max="overrideClip ? overrideClip[1] : animation.duration"
                     :model-value="selectedOverrideKey.time"
                     @update:model-value="setOverrideTime" />
      </label>

      <label class="block">
        <span class="block text-fg-muted text-xs mb-1">Value (drag to pose the joint)</span>
        <div class="space-y-1">
          <input type="range" :min="-JOINT_RANGE" :max="JOINT_RANGE" step="0.005"
                 v-model.number="selectedOverrideKey.value"
                 @input="onChange"
                 class="w-full accent-amber-400" />
          <NumberField class="input-field w-full"
                       step="0.01" :decimals="2"
                       :model-value="selectedOverrideKey.value"
                       @update:model-value="v => { selectedOverrideKey.value = v; onChange() }" />
        </div>
      </label>

      <label class="block">
        <span class="block text-fg-muted text-xs mb-1">Interpolation</span>
        <select class="input-field w-full"
                v-model.number="selectedOverrideKey.interp"
                @change="onChange">
          <optgroup v-for="g in easingGroups" :key="g.category" :label="g.category">
            <option v-for="o in g.items" :key="o.code" :value="o.code">{{ o.label }}</option>
          </optgroup>
        </select>
      </label>

      <button class="btn-sm w-full bg-surface hover:bg-red-600 text-fg-strong hover:text-fg-strong justify-center"
              @click="deleteOverrideKey">
        <span class="material-icons icon-sm">delete</span>
        Delete override key
      </button>
    </template>

    <!-- Trigger keyframe editor -->
    <template v-else-if="selection?.kind === 'trigger-keyframe' && selectedTriggerKey">
      <div class="text-xs text-fg-muted">
        Trigger track <span class="text-fg-strong">{{ selectedTriggerTrack?.name || selectedTriggerTrack?.id }}</span>
        · keyframe #{{ selection.kfIdx + 1 }}
      </div>

      <label class="block">
        <span class="block text-fg-muted text-xs mb-1">Time (s)</span>
        <NumberField class="input-field w-full"
                     step="0.01" :decimals="2" :min="0" :max="animation.duration"
                     :model-value="selectedTriggerKey.time"
                     @update:model-value="v => { selectedTriggerKey.time = v; onChange() }" />
      </label>

      <label class="block">
        <span class="block text-fg-muted text-xs mb-1">Target</span>
        <select class="input-field w-full"
                v-model="selectedTriggerKey.target_kind"
                @change="e => { selectedTriggerKey.target = []; onChange() }">
          <option value="ws_input">Routing sheet input (WebSocket)</option>
          <option value="topic">ROS topic field</option>
        </select>
      </label>

      <template v-if="selectedTriggerKey.target_kind === 'ws_input'">
        <label class="block">
          <span class="block text-fg-muted text-xs mb-1">WS input</span>
          <select class="input-field w-full"
                  :value="`${selectedTriggerKey.target?.[0] || ''}::${selectedTriggerKey.target?.[1] || ''}`"
                  @change="e => {
                    const [sheet, input] = e.target.value.split('::')
                    selectedTriggerKey.target = [sheet || '', input || '']
                    onChange()
                  }">
            <option value="::" disabled>— pick a target —</option>
            <option v-for="w in triggerTargets.wsInputs.value" :key="`${w.sheet_id}::${w.input_id}`"
                    :value="`${w.sheet_id}::${w.input_id}`">
              {{ w.sheet_label || w.sheet_id }} → {{ w.label || w.input_id }}
            </option>
            <option v-if="!triggerTargets.wsInputs.value.length" value="" disabled>
              (no WS inputs — add some on a routing sheet first)
            </option>
          </select>
        </label>
      </template>
      <template v-else>
        <label class="block">
          <span class="block text-fg-muted text-xs mb-1">Topic</span>
          <select class="input-field w-full"
                  :value="selectedTriggerKey.target?.[0] || ''"
                  @change="e => {
                    selectedTriggerKey.target = [e.target.value, selectedTriggerKey.target?.[1] || '']
                    onChange()
                  }">
            <option value="" disabled>— pick a topic —</option>
            <option v-for="t in triggerTargets.topics.value" :key="t.topic" :value="t.topic">{{ t.topic }}</option>
          </select>
        </label>
        <label class="block">
          <span class="block text-fg-muted text-xs mb-1">Field</span>
          <select class="input-field w-full"
                  :value="selectedTriggerKey.target?.[1] || ''"
                  @change="e => {
                    selectedTriggerKey.target = [selectedTriggerKey.target?.[0] || '', e.target.value]
                    onChange()
                  }">
            <option value="" disabled>— pick a field —</option>
            <option v-for="c in triggerTopicChannels" :key="c.field" :value="c.field">
              {{ c.display || c.label || c.field }}
            </option>
          </select>
        </label>
      </template>

      <label class="block">
        <span class="block text-fg-muted text-xs mb-1">Value</span>
        <input class="input-field w-full" type="text"
               :value="String(selectedTriggerKey.value ?? '')"
               @input="e => { selectedTriggerKey.value = coerceTriggerValue(e.target.value); onChange() }"
               :placeholder="selectedTriggerKey.target_kind === 'ws_input' ? 'Number (e.g. 1) or string' : 'Number or string'" />
        <p class="text-xs text-fg-faint mt-1">
          Numbers send as floats; anything non-numeric is sent as a string.
        </p>
      </label>

      <label class="block">
        <span class="block text-fg-muted text-xs mb-1">Label (optional)</span>
        <input class="input-field w-full"
               v-model="selectedTriggerKey.label"
               placeholder="e.g. play-bark"
               @input="onChange" />
      </label>

      <button class="btn-sm w-full bg-surface hover:bg-red-600 text-fg-strong hover:text-fg-strong justify-center"
              @click="deleteTriggerKey">
        <span class="material-icons icon-sm">delete</span>
        Delete trigger keyframe
      </button>
    </template>

    <!-- Track metadata -->
    <template v-else-if="selection?.kind === 'track' && selectedTrack">
      <div class="text-xs text-fg-muted">Track properties</div>

      <label class="block">
        <span class="block text-fg-muted text-xs mb-1">Name</span>
        <input class="input-field w-full" v-model="selectedTrack.name" @input="onChange" />
      </label>

      <div class="text-xs text-fg-faint">
        Track id <span class="font-mono text-fg">{{ selectedTrack.id }}</span> ·
        {{ selectedTrack.curve?.keys?.length || 0 }} keyframes
      </div>
    </template>

    <!-- Joint mode — manual rotation + record. When the picked
         region is a multi-DOF cluster (shoulder, hip, …) we render
         one card per joint so each axis is controllable in place;
         the currently-active joint (the one carrying the 3D gizmo)
         is highlighted. Clicking a non-active joint's title moves
         the gizmo to it. -->
    <template v-else-if="selection?.kind === 'joint' && jointName">
      <div class="text-xs text-fg-muted">
        {{ jointList.length > 1 ? 'URDF joints at this location' : 'URDF joint' }}
      </div>

      <div v-for="name in jointList" :key="name"
           :class="['joint-card space-y-1',
                    name === jointName ? 'joint-card--active' : '']">
        <div class="flex items-center gap-2">
          <button class="font-mono text-fg-strong break-all text-left flex-1 truncate hover:text-cyan-300"
                  :title="name === jointName ? 'Active joint (gizmo attached)' : 'Click to attach gizmo here'"
                  @click="makeJointActive(name)">
            <span v-if="name === jointName" class="material-icons icon-sm text-cyan-400 align-middle mr-1">my_location</span>
            {{ name }}
          </button>
          <span class="text-fg text-xs tabular-nums shrink-0">{{ (jointAngles[name] ?? 0).toFixed(3) }}</span>
        </div>
        <input type="range" :min="-JOINT_RANGE" :max="JOINT_RANGE" step="0.005"
               v-model.number="jointAngles[name]"
               @input="onJointSlider(name)"
               class="w-full accent-cyan-400" />
        <NumberField class="input-field w-full"
                     step="0.01" :decimals="3"
                     :model-value="jointAngles[name]"
                     @update:model-value="v => { jointAngles[name] = v; onJointSlider(name) }" />
        <button class="btn-sm w-full bg-cyan-600 hover:bg-cyan-500 text-fg-strong justify-center"
                @click="setJointKeyframe(name)">
          <span class="material-icons icon-sm">fiber_manual_record</span>
          Set keyframe at {{ Number(playerPos).toFixed(2) }}s
        </button>
        <p class="text-xs text-fg-faint">
          <template v-if="trackForJoint(name)">
            Bound to track <span class="font-mono text-fg">{{ name }}</span>
            ({{ trackForJoint(name).curve?.keys?.length || 0 }} keys).
          </template>
          <template v-else>
            Setting a keyframe auto-binds this joint as a value track.
          </template>
        </p>
      </div>
    </template>

    <!-- Animation metadata — default view when nothing else is selected.
         Editing happens directly on the animation object via v-model;
         the focusin handler in Animations.vue snapshots before the
         first mutation in a typing burst. -->
    <template v-else>
      <div class="text-xs text-fg-muted">Animation</div>

      <div class="flex items-center gap-2">
        <IconPicker :model-value="animation.icon || ''"
                    fallback="animation"
                    @update:model-value="onIconChange" />
        <input class="input-field flex-1" placeholder="Name"
               v-model="animation.name" @input="onChange" />
      </div>

      <div class="grid grid-cols-2 gap-2">
        <label class="block">
          <span class="block text-fg-muted text-xs mb-1">Duration (s)</span>
          <NumberField class="input-field w-full"
                       step="0.1" :min="0"
                       :model-value="animation.duration"
                       @update:model-value="v => { animation.duration = v; onChange() }" />
        </label>
        <label class="block">
          <span class="block text-fg-muted text-xs mb-1">FPS</span>
          <NumberField class="input-field w-full"
                       step="1" :decimals="0" :min="1" :max="240"
                       :model-value="animation.fps"
                       @update:model-value="v => { animation.fps = v; onChange() }" />
        </label>
      </div>

      <label class="flex items-center gap-2 text-sm">
        <input type="checkbox" v-model="animation.loop" @change="onChange" />
        <span class="text-fg">Loop playback</span>
      </label>

      <p class="text-xs text-fg-faint italic mt-3">
        Click a keyframe in the timeline, a joint in the 3D preview, or a track's color swatch to drill in.
      </p>
    </template>
  </div>
</template>

<style scoped>
.joint-card {
  border: 1px solid var(--color-surface);
  border-radius: 0.375rem;
  padding: 0.5rem;
  background: rgba(15, 23, 42, 0.4);
}
.joint-card + .joint-card { margin-top: 0.5rem; }
.joint-card--active {
  border-color: #06b6d4;
  background: rgba(6, 182, 212, 0.08);
}
</style>
