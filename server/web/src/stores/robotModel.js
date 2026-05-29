import { defineStore } from 'pinia'
import { computed, ref } from 'vue'

// Robot URDF + mesh model lifecycle. Backed by the server's
// /api/robot/* endpoints (not WebSocket — these endpoints exist for
// the Tauri controller too, and HTTP is what its webview can hit
// cross-origin via the CORS middleware).
export const useRobotModelStore = defineStore('robotModel', () => {
  const metadata = ref(null)     // null when no model installed
  const loading = ref(false)
  const uploading = ref(false)
  const error = ref('')

  const installed = computed(() => metadata.value?.installed === true)

  // Cache-busting query string — bumped after every upload/delete so
  // the URDF viewer reloads the model instead of pulling the stale
  // copy from the browser cache. The URDF loader reads this URL
  // directly so the suffix flows through into mesh fetches as well.
  const cacheBust = ref(0)
  const urdfUrl = computed(() =>
    installed.value ? `/api/robot/urdf?v=${cacheBust.value}` : null
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

  async function upload (file) {
    if (!file) return
    uploading.value = true
    error.value = ''
    try {
      const fd = new FormData()
      fd.append('file', file, file.name)
      const r = await fetch('/api/robot/urdf', { method: 'POST', body: fd })
      if (!r.ok) {
        const body = await r.json().catch(() => ({}))
        throw new Error(body?.error || `HTTP ${r.status}`)
      }
      const data = await r.json()
      metadata.value = data?.installed ? data : null
      cacheBust.value++
    } catch (e) {
      error.value = e.message || String(e)
      throw e
    } finally {
      uploading.value = false
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

  return {
    metadata, loading, uploading, error,
    installed, urdfUrl, meshesBase, cacheBust,
    refresh, upload, remove,
  }
})
