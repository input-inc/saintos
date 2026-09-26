import { computed, ref, watch } from 'vue'
import { useWsStore } from '@/stores/ws'
import { useWsTopic } from '@/composables/useWsTopic'

// Live current reading for servo calibration.
//
// Dialing a servo's extents is a mechanical judgement — you want to see
// what it draws as it approaches the ends of its travel, so you can
// stop before it stalls against a hard limit. That reading almost never
// comes from the servo itself: on this rig the FAS100 current sensor
// sits on the Cradle Base while the Maestro is on the Head Node.
//
// The dashboard used to auto-pick the first current-reporting
// peripheral on the SAME node, so for Head Node servos it found nothing
// and the indicator stayed blank. This lets the operator choose the
// sensor, from anywhere in the fleet, and remembers the choice — a rig
// has one servo power rail and you calibrate against it all day.
//
// Which channels count as current readings is decided by the SERVER
// from the peripheral catalog (`list_current_sources`), not by matching
// channel-id spellings here. The dashboard's old private copy of that
// list had already drifted, missing the Servo 2040's `current_a`.

const STORAGE_KEY = 'saintos.currentSource'

export function sourceKey (s) {
  return s ? `${s.node_id}|${s.peripheral_id}|${s.channel_id}` : ''
}

export function useCurrentSource () {
  const ws = useWsStore()
  const sources = ref([])
  const selectedKey = ref(localStorage.getItem(STORAGE_KEY) || '')

  const selected = computed(
    () => sources.value.find(s => sourceKey(s) === selectedKey.value) || null)

  // Subscribe to whichever node owns the chosen sensor. useWsTopic
  // re-subscribes when the topic changes and no-ops on an empty one, so
  // switching sensors mid-session just works.
  const feed = useWsTopic(
    () => (selected.value ? `pin_state/${selected.value.node_id}` : ''), 5)

  const value = computed(() => {
    const s = selected.value
    if (!s) return null
    for (const ch of (feed.value?.channels || [])) {
      if (ch.peripheral_id === s.peripheral_id
          && ch.channel_id === s.channel_id
          && typeof ch.value === 'number') {
        return ch.value
      }
    }
    return null
  })

  // Absent from the payload is NOT zero amps — it means the sensor has
  // not reported since we subscribed (node offline, driver not polling).
  // Showing "0.00 A" there would read as "no load" and invite dialing a
  // servo straight into a stall.
  const stale = computed(() => selected.value != null && value.value == null)

  const label = computed(() => {
    const s = selected.value
    if (!s) return ''
    return `${s.node_name} · ${s.peripheral_label} · ${s.channel_label}`
  })

  async function refresh () {
    try {
      const r = await ws.management('list_current_sources', {})
      sources.value = r?.sources || []
    } catch (_) {
      sources.value = []
      return
    }
    // Drop a stored selection whose peripheral has since been removed,
    // so the picker can't sit on something that no longer exists.
    if (selectedKey.value
        && !sources.value.some(s => sourceKey(s) === selectedKey.value)) {
      selectedKey.value = ''
    }
    // One sensor in the fleet is the overwhelmingly common case — pick
    // it rather than making the operator choose from a list of one.
    if (!selectedKey.value && sources.value.length === 1) {
      selectedKey.value = sourceKey(sources.value[0])
    }
  }

  watch(selectedKey, (k) => {
    try {
      if (k) localStorage.setItem(STORAGE_KEY, k)
      else localStorage.removeItem(STORAGE_KEY)
    } catch (_) { /* private browsing / quota — selection just won't persist */ }
  })

  return { sources, selectedKey, selected, value, stale, label, refresh }
}
