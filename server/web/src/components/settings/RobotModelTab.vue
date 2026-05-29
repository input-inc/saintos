<script setup>
import { computed, defineAsyncComponent, onMounted, ref } from 'vue'
import { useRobotModelStore } from '@/stores/robotModel'

// Lazy-loaded — three.js + urdf-loader together are ~800 KB raw and
// only matter once an operator opens this tab. Keeping them out of
// the Settings bundle saves ~200 KB gzipped on first paint.
const URDFViewer = defineAsyncComponent(
  () => import('@/components/animation/URDFViewer.vue')
)

const robot = useRobotModelStore()
const fileInput = ref(null)
const uploadHint = ref('')

const meta = computed(() => robot.metadata || null)

function pickFile () {
  fileInput.value?.click()
}

async function onFileChosen (e) {
  const file = e.target.files?.[0]
  e.target.value = ''  // allow re-uploading the same filename
  if (!file) return
  uploadHint.value = `Uploading ${file.name}…`
  try {
    await robot.upload(file)
    uploadHint.value = `Installed: ${file.name}`
  } catch (err) {
    uploadHint.value = `Upload failed: ${err.message || err}`
  }
}

async function deleteModel () {
  if (!confirm('Remove the installed robot model? Animations referencing it will still play but lose their viewport preview.')) return
  try {
    await robot.remove()
    uploadHint.value = 'Model removed.'
  } catch (err) {
    uploadHint.value = `Delete failed: ${err.message || err}`
  }
}

function fmtTimestamp (t) {
  if (!t) return '—'
  return new Date(t * 1000).toLocaleString()
}

onMounted(() => robot.refresh())
</script>

<template>
  <div class="space-y-6">
    <div>
      <h3 class="text-lg font-semibold text-fg-strong">Robot Model</h3>
      <p class="text-sm text-fg-muted mt-1">
        Upload a URDF (or a <code class="text-cyan-300 text-xs">.zip</code> containing
        the URDF + a <code class="text-cyan-300 text-xs">meshes/</code> subdir) to enable
        the 3D viewer in the Animation Builder. One model at a time — uploading replaces
        the previous one.
      </p>
    </div>

    <div class="flex items-center gap-3">
      <button class="btn-primary" :disabled="robot.uploading" @click="pickFile">
        <span class="material-icons icon-sm">upload_file</span>
        {{ meta ? 'Replace model' : 'Upload model' }}
      </button>
      <button
        v-if="meta"
        class="btn-secondary"
        :disabled="robot.uploading"
        @click="deleteModel"
      >
        <span class="material-icons icon-sm">delete</span>
        Remove
      </button>
      <input
        ref="fileInput"
        type="file"
        accept=".zip,.urdf,.xacro"
        class="hidden"
        @change="onFileChosen"
      />
      <span v-if="uploadHint" class="text-xs text-fg-muted">{{ uploadHint }}</span>
    </div>

    <div v-if="robot.error" class="p-3 bg-red-500/20 border border-red-500/40 rounded-lg text-sm text-red-300">
      {{ robot.error }}
    </div>

    <div v-if="meta" class="rounded-lg border border-line/50 bg-panel/30 p-4">
      <h4 class="text-sm font-semibold text-fg-strong mb-3">Installed model</h4>
      <dl class="grid grid-cols-2 gap-x-4 gap-y-2 text-sm">
        <dt class="text-fg-muted">Original file</dt>
        <dd class="text-fg-strong">{{ meta.original_filename }}</dd>
        <dt class="text-fg-muted">URDF</dt>
        <dd class="text-fg-strong">{{ meta.urdf_filename }}</dd>
        <dt class="text-fg-muted">Links / joints</dt>
        <dd class="text-fg-strong">{{ meta.link_count }} links · {{ meta.joint_count }} joints</dd>
        <dt class="text-fg-muted">Meshes bundled</dt>
        <dd class="text-fg-strong">{{ meta.mesh_files?.length || 0 }}</dd>
        <dt class="text-fg-muted">SHA-256</dt>
        <dd class="text-fg-strong font-mono text-xs">{{ meta.sha256?.slice(0, 16) }}…</dd>
        <dt class="text-fg-muted">Uploaded</dt>
        <dd class="text-fg-strong">{{ fmtTimestamp(meta.uploaded_at) }}</dd>
      </dl>
    </div>

    <div v-if="meta">
      <h4 class="text-sm font-semibold text-fg-strong mb-2">Preview</h4>
      <URDFViewer
        :urdf-url="robot.urdfUrl"
        :meshes-base="robot.meshesBase"
        height="420px"
      />
    </div>
  </div>
</template>
