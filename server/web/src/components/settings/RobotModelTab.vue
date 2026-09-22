<script setup>
import { computed, defineAsyncComponent, onMounted, ref } from 'vue'
import { useRobotModelStore } from '@/stores/robotModel'
import { usePosesStore } from '@/stores/poses'
import { useWsStore } from '@/stores/ws'

// Lazy-loaded — three.js + urdf-loader together are ~800 KB raw and
// only matter once an operator opens this tab. Keeping them out of
// the Settings bundle saves ~200 KB gzipped on first paint.
const URDFViewer = defineAsyncComponent(
  () => import('@/components/animation/URDFViewer.vue')
)
const ImportGroupStatesModal = defineAsyncComponent(
  () => import('@/components/animation/ImportGroupStatesModal.vue')
)

const robot = useRobotModelStore()
const poses = usePosesStore()
const ws = useWsStore()

const urdfInput = ref(null)
const srdfInput = ref(null)
const rigInput = ref(null)
const uploadHint = ref('')

const meta = computed(() => robot.metadata || null)

// ── group_state import ─────────────────────────────────────────────
//
// Offered automatically right after an SRDF lands, since that's the
// moment the operator is thinking about it. Reachable again from the
// SRDF card afterwards.
const showImport = ref(false)
const importStates = ref([])
const importing = ref(false)
const poseGroups = computed(() =>
  [...new Set((poses.list || []).map(p => p.group).filter(Boolean))].sort())

async function openImport () {
  // Go through the management channel rather than the HTTP endpoint:
  // this one annotates each candidate with whether a pose already
  // exists and whether it's been edited since import, which the plain
  // /api/robot/group_states response can't know.
  try {
    const r = await ws.management('list_group_states', {})
    importStates.value = r?.group_states || []
  } catch (e) {
    importStates.value = await robot.loadGroupStates()
  }
  showImport.value = true
}

async function doImport (payload) {
  importing.value = true
  try {
    const r = await ws.management('import_group_states', payload)
    const n = r?.imported?.length || 0
    const skipped = r?.skipped?.length || 0
    uploadHint.value = `Imported ${n} pose${n === 1 ? '' : 's'}`
      + (skipped ? ` · ${skipped} skipped` : '')
      + (r?.warnings?.length ? ` · ${r.warnings.join('; ')}` : '')
    await poses.reload()
    showImport.value = false
  } catch (e) {
    uploadHint.value = `Import failed: ${e.message || e}`
  } finally {
    importing.value = false
  }
}

// ── uploads ────────────────────────────────────────────────────────

async function onUrdfChosen (e) {
  const file = e.target.files?.[0]
  e.target.value = ''  // allow re-uploading the same filename
  if (!file) return
  uploadHint.value = `Uploading ${file.name}…`
  try {
    const body = await robot.upload(file)
    uploadHint.value = `Installed: ${file.name}`
    // A bundle may have carried an SRDF along with it — offer the
    // import in that case too, not only on a standalone SRDF upload.
    if (body?.srdf?.group_state_count) await openImport()
  } catch (err) {
    uploadHint.value = `Upload failed: ${err.message || err}`
  }
}

async function onSrdfChosen (e) {
  const file = e.target.files?.[0]
  e.target.value = ''
  if (!file) return
  uploadHint.value = `Uploading ${file.name}…`
  try {
    const body = await robot.uploadSrdf(file)
    const n = body?.summary?.group_state_count || 0
    uploadHint.value = `Installed: ${file.name}`
    if (n > 0) {
      // The upload response already carries the group_states, but go
      // through openImport() so the rows pick up their exists /
      // locally_edited annotations.
      await openImport()
    } else {
      uploadHint.value += ' — no group_state definitions found'
    }
  } catch (err) {
    uploadHint.value = `Upload failed: ${err.message || err}`
  }
}

async function onRigChosen (e) {
  const file = e.target.files?.[0]
  e.target.value = ''
  if (!file) return
  uploadHint.value = `Uploading ${file.name}…`
  try {
    const body = await robot.uploadRig(file)
    const n = body?.summary?.control_count || 0
    uploadHint.value = `Installed: ${file.name} — ${n} control${n === 1 ? '' : 's'}`
  } catch (err) {
    uploadHint.value = `Upload failed: ${err.message || err}`
  }
}

// ── deletes ────────────────────────────────────────────────────────

async function deleteModel () {
  if (!confirm('Remove the installed robot model? Animations referencing it will still play but lose their viewport preview.')) return
  try {
    await robot.remove()
    uploadHint.value = 'Model removed.'
  } catch (err) {
    uploadHint.value = `Delete failed: ${err.message || err}`
  }
}
async function deleteSrdf () {
  if (!confirm('Remove the SRDF? Poses already imported from it are kept — they live in the pose library now.')) return
  try {
    await robot.removeSrdf()
    uploadHint.value = 'SRDF removed.'
  } catch (err) {
    uploadHint.value = `Delete failed: ${err.message || err}`
  }
}
async function deleteRigFile () {
  if (!confirm('Remove the rig file? On-screen controls will disappear until another is uploaded.')) return
  try {
    await robot.removeRig()
    uploadHint.value = 'Rig file removed.'
  } catch (err) {
    uploadHint.value = `Delete failed: ${err.message || err}`
  }
}

function fmtTimestamp (t) {
  if (!t) return '—'
  return new Date(t * 1000).toLocaleString()
}

onMounted(async () => {
  await robot.refresh()
  robot.loadJoints()
  poses.reload()
})
</script>

<template>
  <div class="space-y-6">
    <div>
      <h3 class="text-lg font-semibold text-fg-strong">Robot Model</h3>
      <p class="text-sm text-fg-muted mt-1">
        Three files describe the robot. The
        <strong class="text-fg-strong">URDF</strong> owns every name — links,
        joints, limits, geometry. The
        <strong class="text-fg-strong">SRDF</strong> adds joint groups and
        named poses, and the <strong class="text-fg-strong">rig file</strong>
        adds on-screen controls. The last two are annotation layers: every
        name in them has to resolve against the URDF.
      </p>
    </div>

    <div v-if="robot.error"
         class="p-3 bg-red-500/20 border border-red-500/40 rounded-lg text-sm text-red-300">
      {{ robot.error }}
    </div>

    <!-- Unresolved cross-file references. Worth surfacing loudly: an -->
    <!-- SRDF or rig reference that doesn't resolve is a silent no-op, -->
    <!-- so this panel is the only place it ever shows up.             -->
    <div v-if="robot.warnings.length"
         class="rounded-lg border border-amber-500/40 bg-amber-500/10 p-3">
      <div class="flex items-center gap-2 text-sm text-amber-200 font-medium">
        <span class="material-icons icon-sm">warning</span>
        {{ robot.warnings.length }} unresolved reference{{ robot.warnings.length === 1 ? '' : 's' }}
      </div>
      <p class="text-xs text-amber-200/70 mt-1">
        These don't stop anything loading — they just silently do nothing at
        runtime, which is why they're listed here.
      </p>
      <ul class="mt-2 space-y-0.5 text-xs text-amber-100/90 font-mono max-h-40 overflow-y-auto">
        <li v-for="(w, i) in robot.warnings" :key="i">{{ w }}</li>
      </ul>
    </div>

    <!-- ── URDF ─────────────────────────────────────────────────── -->
    <div class="rounded-lg border border-line/50 bg-panel/30 p-4">
      <div class="flex items-start justify-between gap-3">
        <div>
          <h4 class="text-sm font-semibold text-fg-strong">
            URDF <span class="text-fg-faint font-normal font-mono text-xs">robot_description</span>
          </h4>
          <p class="text-xs text-fg-muted mt-1">
            A <code class="text-cyan-300">.urdf</code>, or a
            <code class="text-cyan-300">.zip</code> holding the URDF plus a
            <code class="text-cyan-300">meshes/</code> subdir and optionally the
            SRDF and rig file. One model at a time — uploading replaces the
            previous one, but keeps any SRDF and rig already installed.
          </p>
        </div>
        <div class="flex items-center gap-2 shrink-0">
          <button class="btn-primary" :disabled="robot.uploading"
                  @click="urdfInput?.click()">
            <span class="material-icons icon-sm">upload_file</span>
            {{ meta ? 'Replace' : 'Upload' }}
          </button>
          <button v-if="meta" class="btn-secondary" :disabled="robot.uploading"
                  @click="deleteModel">
            <span class="material-icons icon-sm">delete</span>
          </button>
        </div>
      </div>
      <input ref="urdfInput" type="file" accept=".zip,.urdf,.xacro"
             class="hidden" @change="onUrdfChosen" />

      <dl v-if="meta" class="grid grid-cols-2 gap-x-4 gap-y-2 text-sm mt-4">
        <dt class="text-fg-muted">Robot name</dt>
        <dd class="text-fg-strong font-mono">{{ meta.robot_name || '—' }}</dd>
        <dt class="text-fg-muted">File</dt>
        <dd class="text-fg-strong">{{ meta.urdf_filename }}</dd>
        <dt class="text-fg-muted">Links / joints</dt>
        <dd class="text-fg-strong">{{ meta.link_count }} links · {{ meta.joint_count }} joints</dd>
        <dt class="text-fg-muted">Actuatable joints</dt>
        <dd class="text-fg-strong">{{ robot.joints.length }}</dd>
        <dt class="text-fg-muted">Meshes bundled</dt>
        <dd class="text-fg-strong">{{ meta.mesh_files?.length || 0 }}</dd>
        <dt class="text-fg-muted">Uploaded</dt>
        <dd class="text-fg-strong">{{ fmtTimestamp(meta.uploaded_at) }}</dd>
      </dl>
    </div>

    <!-- ── SRDF ─────────────────────────────────────────────────── -->
    <div v-if="meta" class="rounded-lg border border-line/50 bg-panel/30 p-4">
      <div class="flex items-start justify-between gap-3">
        <div>
          <h4 class="text-sm font-semibold text-fg-strong">
            SRDF <span class="text-fg-faint font-normal font-mono text-xs">robot_description_semantic</span>
          </h4>
          <p class="text-xs text-fg-muted mt-1">
            MoveIt's semantic layer: joint groups and
            <code class="text-cyan-300">&lt;group_state&gt;</code> named poses.
            Group states import straight into the pose library.
          </p>
        </div>
        <div class="flex items-center gap-2 shrink-0">
          <button class="btn-primary" :disabled="robot.uploading"
                  @click="srdfInput?.click()">
            <span class="material-icons icon-sm">upload_file</span>
            {{ robot.hasSrdf ? 'Replace' : 'Upload' }}
          </button>
          <button v-if="robot.hasSrdf" class="btn-secondary"
                  :disabled="robot.uploading" @click="deleteSrdf">
            <span class="material-icons icon-sm">delete</span>
          </button>
        </div>
      </div>
      <input ref="srdfInput" type="file" accept=".srdf,.xml,.xacro"
             class="hidden" @change="onSrdfChosen" />

      <template v-if="robot.hasSrdf && robot.srdf">
        <dl class="grid grid-cols-2 gap-x-4 gap-y-2 text-sm mt-4">
          <dt class="text-fg-muted">File</dt>
          <dd class="text-fg-strong">{{ meta.srdf_filename }}</dd>
          <dt class="text-fg-muted">Declares robot</dt>
          <dd :class="robot.srdf.robot_name === meta.robot_name
                        ? 'text-fg-strong font-mono'
                        : 'text-amber-300 font-mono'">
            {{ robot.srdf.robot_name || '—' }}
            <span v-if="robot.srdf.robot_name && robot.srdf.robot_name !== meta.robot_name"
                  class="text-xs">(doesn't match the URDF)</span>
          </dd>
          <dt class="text-fg-muted">Groups</dt>
          <dd class="text-fg-strong">{{ robot.srdf.group_count }}</dd>
          <dt class="text-fg-muted">Named poses</dt>
          <dd class="text-fg-strong">{{ robot.srdf.group_state_count }}</dd>
          <dt class="text-fg-muted">Disabled collision pairs</dt>
          <dd class="text-fg-strong">{{ robot.srdf.disabled_collision_count }}</dd>
        </dl>
        <div v-if="robot.srdf.group_state_count" class="mt-3">
          <button class="btn-secondary" @click="openImport">
            <span class="material-icons icon-sm">library_add</span>
            Import group states as poses…
          </button>
        </div>
      </template>
      <p v-else class="text-xs text-fg-faint mt-3">No SRDF installed.</p>
    </div>

    <!-- ── Rig file ─────────────────────────────────────────────── -->
    <div v-if="meta" class="rounded-lg border border-line/50 bg-panel/30 p-4">
      <div class="flex items-start justify-between gap-3">
        <div>
          <h4 class="text-sm font-semibold text-fg-strong">
            Rig file <span class="text-fg-faint font-normal font-mono text-xs">robot_description_rig</span>
          </h4>
          <p class="text-xs text-fg-muted mt-1">
            On-screen controls — sliders that blend poses, XY pads for eye
            look, weighted joint drives for a head nod or tilt. Our own
            schema; see <code class="text-cyan-300">docs/RIG_SCHEMA.md</code>.
          </p>
        </div>
        <div class="flex items-center gap-2 shrink-0">
          <button class="btn-primary" :disabled="robot.uploading"
                  @click="rigInput?.click()">
            <span class="material-icons icon-sm">upload_file</span>
            {{ robot.hasRig ? 'Replace' : 'Upload' }}
          </button>
          <button v-if="robot.hasRig" class="btn-secondary"
                  :disabled="robot.uploading" @click="deleteRigFile">
            <span class="material-icons icon-sm">delete</span>
          </button>
        </div>
      </div>
      <input ref="rigInput" type="file" accept=".xml,.rig" class="hidden"
             @change="onRigChosen" />

      <template v-if="robot.hasRig && robot.rig">
        <dl class="grid grid-cols-2 gap-x-4 gap-y-2 text-sm mt-4">
          <dt class="text-fg-muted">File</dt>
          <dd class="text-fg-strong">{{ meta.rig_filename }}</dd>
          <dt class="text-fg-muted">Schema version</dt>
          <dd class="text-fg-strong font-mono">{{ robot.rig.version }}</dd>
          <dt class="text-fg-muted">Controls</dt>
          <dd class="text-fg-strong">{{ robot.rig.control_count }}</dd>
          <dt class="text-fg-muted">Limit policy</dt>
          <dd class="text-fg-strong font-mono">{{ robot.rig.clamp }}</dd>
          <dt class="text-fg-muted">Neutral pose</dt>
          <dd class="text-fg-strong font-mono">{{ robot.rig.neutral_pose || '(zeros)' }}</dd>
        </dl>
        <ul v-if="robot.rig.controls?.length"
            class="mt-3 flex flex-wrap gap-1.5">
          <li v-for="c in robot.rig.controls" :key="c.name"
              class="rounded bg-surface/60 border border-line/40 px-2 py-1 text-xs">
            <span class="text-fg-strong">{{ c.label || c.name }}</span>
            <span class="text-fg-faint ml-1 font-mono">{{ c.kind }}</span>
          </li>
        </ul>
      </template>
      <p v-else class="text-xs text-fg-faint mt-3">No rig file installed.</p>
    </div>

    <!-- ── Preview ──────────────────────────────────────────────── -->
    <div v-if="meta">
      <h4 class="text-sm font-semibold text-fg-strong mb-2">Preview</h4>
      <!-- Preview only — no collision machinery. With it on, this tab
           downloaded every <collision> mesh and ran the full hull/BVH/ACM
           build just to show the model, which is why the upload preview
           used to sit on "Loading robot model…" long after the mesh was
           visible. -->
      <URDFViewer
        :urdf-url="robot.urdfUrl"
        :meshes-base="robot.meshesBase"
        height="420px"
        :collision="false"
      />
    </div>

    <p v-if="uploadHint" class="text-xs text-fg-muted">{{ uploadHint }}</p>

    <ImportGroupStatesModal v-if="showImport"
                            :states="importStates"
                            :pose-groups="poseGroups"
                            :busy="importing"
                            @close="showImport = false"
                            @import="doImport" />
  </div>
</template>
