import { defineStore } from 'pinia'
import { computed, ref } from 'vue'

// Robot model lifecycle — three files, mirroring how ROS packages a
// robot and publishes each on its own parameter:
//
//   robot.urdf     robot_description           structure, limits, geometry
//   robot.srdf     robot_description_semantic  groups + named poses
//   robot.rig.xml  robot_description_rig       controls (docs/RIG_SCHEMA.md)
//
// The URDF owns every name; the other two are annotation layers whose
// every reference resolves against it. That's why `warnings` matters
// here: an unresolved reference is a silent no-op, not a load error, so
// this UI is the only place an operator will ever find out that half a
// rig stopped working after a joint rename.
//
// Backed by /api/robot/* rather than the WebSocket, because these
// endpoints exist for the Tauri controller too and HTTP is what its
// webview can hit cross-origin via the CORS middleware.
export const useRobotModelStore = defineStore('robotModel', () => {
  const metadata = ref(null)     // null when no model installed
  const loading = ref(false)
  const uploading = ref(false)
  const error = ref('')

  const installed = computed(() => metadata.value?.installed === true)
  const hasSrdf = computed(() => !!metadata.value?.srdf_filename)
  const hasRig = computed(() => !!metadata.value?.rig_filename)
  const srdf = computed(() => metadata.value?.srdf || null)
  const rig = computed(() => metadata.value?.rig || null)
  const warnings = computed(() => metadata.value?.warnings || [])

  // Cache-busting query string — bumped after every upload/delete so
  // consumers reload instead of pulling the stale copy from the browser
  // cache. The URDF loader reads its URL directly so the suffix flows
  // through into mesh fetches as well.
  const cacheBust = ref(0)
  const urdfUrl = computed(() =>
    installed.value ? `/api/robot/urdf?v=${cacheBust.value}` : null
  )
  const srdfUrl = computed(() =>
    hasSrdf.value ? `/api/robot/srdf?v=${cacheBust.value}` : null
  )
  const rigUrl = computed(() =>
    hasRig.value ? `/api/robot/rig?v=${cacheBust.value}` : null
  )
  const meshesBase = computed(() =>
    installed.value ? `/api/robot/meshes/` : null
  )

  async function refresh () {
    loading.value = true
    error.value = ''
    try {
      const r = await fetch('/api/robot/metadata')
      if (!r.ok) throw new Error(`HTTP ${r.status}`)
      const data = await r.json()
      metadata.value = data?.installed ? data : null
    } catch (e) {
      error.value = e.message || String(e)
      metadata.value = null
    } finally {
      loading.value = false
    }
  }

  // One upload path for all three files — they differ only in endpoint.
  // Returns the parsed response so a caller can act on it immediately;
  // an SRDF upload comes back carrying its group_states, which is what
  // lets the settings tab offer to import them as poses without a
  // second round trip.
  async function uploadTo (path, file) {
    if (!file) return null
    uploading.value = true
    error.value = ''
    try {
      const fd = new FormData()
      fd.append('file', file, file.name)
      const r = await fetch(path, { method: 'POST', body: fd })
      const body = await r.json().catch(() => ({}))
      if (!r.ok) throw new Error(body?.error || `HTTP ${r.status}`)
      cacheBust.value++
      // The URDF endpoint returns the full describe() payload; the
      // companion endpoints nest it under `metadata`.
      const desc = body?.metadata || body
      if (desc?.installed !== undefined) {
        metadata.value = desc.installed ? desc : null
      } else {
        await refresh()
      }
      return body
    } catch (e) {
      error.value = e.message || String(e)
      throw e
    } finally {
      uploading.value = false
    }
  }

  const upload = (file) => uploadTo('/api/robot/urdf', file)
  const uploadSrdf = (file) => uploadTo('/api/robot/srdf', file)
  const uploadRig = (file) => uploadTo('/api/robot/rig', file)

  async function removeFrom (path) {
    error.value = ''
    try {
      const r = await fetch(path, { method: 'DELETE' })
      if (!r.ok) throw new Error(`HTTP ${r.status}`)
      cacheBust.value++
      await refresh()
    } catch (e) {
      error.value = e.message || String(e)
      throw e
    }
  }

  async function remove () {
    error.value = ''
    try {
      const r = await fetch('/api/robot/urdf', { method: 'DELETE' })
      if (!r.ok) throw new Error(`HTTP ${r.status}`)
      metadata.value = null
      cacheBust.value++
    } catch (e) {
      error.value = e.message || String(e)
      throw e
    }
  }

  const removeSrdf = () => removeFrom('/api/robot/srdf')
  const removeRig = () => removeFrom('/api/robot/rig')

  // ── queries ──────────────────────────────────────────────────────

  // Actuatable joints with their limits. Limits ride along because the
  // normalized −1..+1 the rest of the stack speaks is *defined* by them,
  // so any UI wanting to show native radians has to have them.
  const joints = ref([])
  async function loadJoints () {
    try {
      const r = await fetch('/api/robot/joints')
      if (!r.ok) throw new Error(`HTTP ${r.status}`)
      joints.value = (await r.json())?.joints || []
    } catch (e) {
      console.warn('loadJoints failed:', e)
      joints.value = []
    }
    return joints.value
  }

  // SRDF joint groups, already expanded to their joint lists.
  const groups = ref([])
  async function loadGroups () {
    try {
      const r = await fetch('/api/robot/groups')
      if (!r.ok) throw new Error(`HTTP ${r.status}`)
      groups.value = (await r.json())?.groups || []
    } catch (e) {
      console.warn('loadGroups failed:', e)
      groups.value = []
    }
    return groups.value
  }

  // Importable pose candidates. Each carries the native (radian/metre)
  // values as authored AND the normalized −1..+1 that would be stored,
  // plus any joints that didn't resolve.
  const groupStates = ref([])
  async function loadGroupStates () {
    try {
      const r = await fetch('/api/robot/group_states')
      if (!r.ok) throw new Error(`HTTP ${r.status}`)
      groupStates.value = (await r.json())?.group_states || []
    } catch (e) {
      console.warn('loadGroupStates failed:', e)
      groupStates.value = []
    }
    return groupStates.value
  }

  // Joint-limit lookup for converting between native and normalized.
  const jointLimits = computed(() => {
    const map = {}
    for (const j of joints.value) map[j.name] = [j.lower, j.upper]
    return map
  })

  return {
    metadata, loading, uploading, error,
    installed, hasSrdf, hasRig, srdf, rig, warnings,
    urdfUrl, srdfUrl, rigUrl, meshesBase, cacheBust,
    joints, groups, groupStates, jointLimits,
    refresh,
    upload, uploadSrdf, uploadRig,
    remove, removeSrdf, removeRig,
    loadJoints, loadGroups, loadGroupStates,
  }
})
