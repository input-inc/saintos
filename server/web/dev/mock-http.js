// HTTP-side handlers for the dev mock server. Implements the URDF
// upload/serve routes and Maestro Pololu .xml parsing — same surface
// as the real Python server's http_server.py, but backed by in-memory
// state and a Node-side curve/zip pipeline.

import { createHash } from 'node:crypto'
import JSZip from 'jszip'

import * as st from './mock-state.js'
import { bridge } from './mock-bridge.js'

const MAX_URDF_UPLOAD_BYTES = 64 * 1024 * 1024
const ALLOWED_MESH_EXT = new Set(['.stl', '.dae', '.obj', '.ply', '.glb', '.gltf'])

// ── Public dispatcher ───────────────────────────────────────────────

/**
 * Try to handle an HTTP request. Returns true if handled, false
 * otherwise (so the caller can fall back to its 404).
 */
export async function handleHttp (req, res) {
  const url = req.url || ''

  if (req.method === 'GET' && url === '/api/robot/metadata') {
    return sendJson(res, await robotMetadataPayload())
  }
  if (req.method === 'GET' && url.startsWith('/api/robot/urdf')) {
    return serveUrdf(res)
  }
  if (req.method === 'POST' && url === '/api/robot/urdf') {
    return await handleUrdfUpload(req, res)
  }
  if (req.method === 'DELETE' && url === '/api/robot/urdf') {
    st.setUrdfModel(null)
    return sendJson(res, { removed: true })
  }
  // ── SRDF + rig companions ──────────────────────────────────────
  // Written in place: an annotation layer install must not disturb the
  // URDF. Mirrors RobotModelStore.install_srdf / install_rig.
  if (req.method === 'GET' && url.startsWith('/api/robot/srdf')) {
    return serveCompanion(res, 'srdfBytes', 'No SRDF installed')
  }
  if (req.method === 'POST' && url === '/api/robot/srdf') {
    return await handleCompanionUpload(req, res, 'srdf')
  }
  if (req.method === 'DELETE' && url === '/api/robot/srdf') {
    return sendJson(res, { removed: st.clearSrdf() })
  }
  if (req.method === 'GET' && url.startsWith('/api/robot/rig')) {
    return serveCompanion(res, 'rigBytes', 'No rig file installed')
  }
  if (req.method === 'POST' && url === '/api/robot/rig') {
    return await handleCompanionUpload(req, res, 'rig')
  }
  if (req.method === 'DELETE' && url === '/api/robot/rig') {
    return sendJson(res, { removed: st.clearRig() })
  }
  if (req.method === 'GET' && url === '/api/robot/groups') {
    return sendJson(res, await bridge('groups', st.robotModelTexts()))
  }
  if (req.method === 'GET' && url === '/api/robot/group_states') {
    return sendJson(res, await bridge('group_states', {
      ...st.robotModelTexts(),
      existing_pose_ids: [...st.poses.keys()],
    }))
  }
  if (req.method === 'GET' && url === '/api/robot/joints') {
    return sendJson(res, { joints: await listUrdfJoints() })
  }
  if (req.method === 'GET' && url.startsWith('/api/robot/meshes/')) {
    return serveMesh(res, decodeURIComponent(url.slice('/api/robot/meshes/'.length)))
  }
  if (req.method === 'POST' && url === '/api/animations/import/maestro') {
    return await handleMaestroImport(req, res)
  }
  // Dev-only: poke a simulated switch_input so the interlock path is
  // observable without hardware. There is no equivalent on the real
  // server — a real sensor is asserted by moving the mechanism.
  //   curl -X POST 'localhost:8081/api/dev/switch?id=teensy41_lift01/limit-1&asserted=1'
  if (req.method === 'POST' && url.startsWith('/api/dev/switch')) {
    return handleDevSwitch(url, res)
  }
  // Firmware listing + download. Mirrors the real server's
  // /api/firmware surface (http_server.py) so the Settings → Firmware
  // download rows render and actually transfer a file.
  if (req.method === 'GET' && url === '/api/firmware') {
    return sendJson(res, {
      firmware_root: '/mock/resources/firmware',
      firmware_types: Object.entries(MOCK_FIRMWARE).map(([type, files]) => ({
        type,
        files: files.map(f => ({
          filename: f.filename,
          size: f.size,
          ext: f.filename.slice(f.filename.lastIndexOf('.')),
          url: `/api/firmware/${type}/${f.filename}`,
        })),
      })),
    })
  }
  if (req.method === 'GET' && url.startsWith('/api/firmware/')) {
    return serveMockFirmware(url, res)
  }
  return false
}

// Sizes mirror the real staged artifacts closely enough that the size
// column and the "large download" feel are representative.
const MOCK_FIRMWARE = {
  rp2040: [
    { filename: 'saint_node.uf2',          size: 545_259 },
    { filename: 'saint_node_combined.uf2', size: 663_552 },
    { filename: 'saint_ota_bootloader.uf2', size: 126_976 },
    { filename: 'saint_node.elf',          size: 2_243_216 },
    { filename: 'saint_node.bin',          size: 272_384 },
  ],
  teensy41: [
    { filename: 'firmware.hex',   size: 1_499_136 },
    { filename: 'saint_node.bin', size: 534_528 },
  ],
  raspberrypi: [
    { filename: 'saint_firmware_raspberrypi_1.1.0.tar.zst', size: 660_812_800 },
  ],
  controller: [
    { filename: 'saint_firmware_controller_0.5.0-local.ef592f5.AppImage', size: 92_557_312 },
  ],
}

// Serve a synthetic file of the right name and length. Content is filler
// — the point is that the browser's download path works end to end
// (headers, filename, progress), not that the bytes are a real image.
function serveMockFirmware (url, res) {
  const rest = decodeURIComponent(url.slice('/api/firmware/'.length))
  const slash = rest.indexOf('/')
  if (slash < 0) {
    const files = MOCK_FIRMWARE[rest]
    if (!files) return sendJson(res, { error: `Unknown firmware type: ${rest}` }, 404)
    return sendJson(res, { type: rest, files })
  }
  const type = rest.slice(0, slash)
  const filename = rest.slice(slash + 1)
  // Same traversal guard as the real handler.
  if (filename.includes('..') || filename.includes('/') || filename.includes('\\')) {
    res.writeHead(403); res.end('Forbidden'); return true
  }
  const entry = (MOCK_FIRMWARE[type] || []).find(f => f.filename === filename)
  if (!entry) { res.writeHead(404); res.end('Not Found'); return true }

  // Cap the synthetic payload: streaming a truthful 630 MB of filler for
  // the Pi bundle would tie up the dev loop for no benefit. The
  // Content-Length is the real size so the UI shows the true figure;
  // the transfer just ends early, which is fine for a mock.
  const CAP = 2 * 1024 * 1024
  const bytes = Math.min(entry.size, CAP)
  res.writeHead(200, {
    'Content-Type': 'application/octet-stream',
    'Content-Disposition': `attachment; filename="${filename}"`,
    'Content-Length': String(bytes),
    'X-Mock-Truncated': bytes < entry.size ? 'true' : 'false',
  })
  res.end(Buffer.alloc(bytes, 0x00))
  return true
}

function handleDevSwitch (url, res) {
  const q = new URL(url, 'http://localhost').searchParams
  const id = q.get('id') || ''
  const asserted = q.get('asserted') === '1' || q.get('asserted') === 'true'
  if (!id.includes('/')) {
    return sendJson(res, {
      error: 'id must be "<node_id>/<peripheral_id>"',
      example: '/api/dev/switch?id=teensy41_lift01/limit-1&asserted=1',
    }, 400)
  }
  st.live.switchAsserted[id] = asserted
  // Asserting latches on the next telemetry tick (the latch is what the
  // firmware does, so the mock does it there too, not here). Releasing
  // leaves the latch set — clearing it is the operator's job, via the
  // clear_latch command.
  return sendJson(res, {
    id, asserted,
    latched: !!st.live.switchLatched[id],
    note: asserted
      ? 'latches on next tick; interlock fires in firmware on real hardware'
      : 'released — latch stays set until cleared',
  })
}

// Actuatable joints in the installed URDF, WITH their limits. Skips
// fixed joints because they accept no setpoint — they're structural (a
// control anchor frame is a massless link on a fixed joint).
//
// Goes through the Python bridge rather than a regex: the limits are the
// point. Every consumer that shows a joint also needs them, because the
// normalized -1..+1 the whole stack speaks is *defined* by them. Falls
// back to the old regex scan if the bridge is unavailable, so the joint
// picker still populates on a machine without python3.
async function listUrdfJoints () {
  const m = st.getUrdfModel()
  if (!m) return []
  const viaBridge = await bridge('joints', { urdf: m.urdfBytes.toString('utf8') })
  if (!viaBridge.error && Array.isArray(viaBridge.joints)) return viaBridge.joints

  const text = m.urdfBytes.toString('utf-8')
  const out = []
  const re = /<joint\s+[^>]*\bname="([^"]+)"[^>]*\btype="([^"]+)"/gi
  let match
  while ((match = re.exec(text)) !== null) {
    if (match[2].toLowerCase() === 'fixed') continue
    out.push({ name: match[1], type: match[2] })
  }
  return out
}

// ── URDF lifecycle ──────────────────────────────────────────────────

async function robotMetadataPayload () {
  const m = st.getUrdfModel()
  if (!m) return { installed: false }
  // describe(): metadata plus each companion's parsed summary and every
  // unresolved cross-file reference. Those warnings matter more than
  // they look — an SRDF or rig reference that doesn't resolve is a
  // silent no-op, so this response is the only place it surfaces.
  const desc = await bridge('describe', {
    ...st.robotModelTexts(),
    pose_names: [...st.poses.keys()],
  })
  return {
    installed: true,
    ...m.metadata,
    srdf: desc.srdf ?? null,
    rig: desc.rig ?? null,
    robot_name: desc.robot_name ?? m.metadata.robot_name ?? '',
    warnings: desc.warnings || (desc.error ? [`bridge: ${desc.error}`] : []),
  }
}

function serveCompanion (res, field, missingMsg) {
  const m = st.getUrdfModel()
  const bytes = m?.[field]
  if (!bytes) {
    res.writeHead(404, { 'Content-Type': 'text/plain' })
    res.end(missingMsg)
    return true
  }
  res.writeHead(200, {
    'Content-Type': 'application/xml',
    'Cache-Control': 'no-cache, must-revalidate',
  })
  res.end(bytes)
  return true
}

/**
 * Install an SRDF or rig file alongside the existing URDF.
 *
 * The SRDF response carries its parsed group_states, which is what lets
 * the settings tab offer to import them as poses without a second round
 * trip.
 */
async function handleCompanionUpload (req, res, which) {
  try {
    if (!st.getUrdfModel()) {
      throw httpErr(400, which === 'srdf'
        ? 'no URDF installed — an SRDF is an annotation layer and every '
          + 'name in it is a dangling reference on its own'
        : "no URDF installed — a rig file's joints and links are dangling "
          + 'references on their own')
    }
    const part = await readSingleFilePart(req)
    if (!part) throw httpErr(400, 'Missing file field')
    const { buffer, filename } = part
    const text = buffer.toString('utf8')

    // Parse-check through the real implementation before storing, so a
    // broken file is refused rather than installed inert.
    const probe = await bridge(which === 'srdf' ? 'group_states' : 'rig',
      which === 'srdf'
        ? { urdf: st.robotModelTexts().urdf, srdf: text }
        : { ...st.robotModelTexts(), rig: text })
    if (probe.error) throw httpErr(400, probe.error)

    const sha = sha256Hex(buffer)
    const base = (filename.split(/[\\/]/).pop()
      || (which === 'srdf' ? 'robot.srdf' : 'robot.rig.xml'))
    if (which === 'srdf') st.setSrdf(buffer, base, sha)
    else st.setRig(buffer, base, sha)

    const metadata = await robotMetadataPayload()
    if (which === 'srdf') {
      return sendJson(res, {
        installed: true,
        srdf_filename: base,
        summary: metadata.srdf,
        warnings: (metadata.warnings || []).filter(w => w.startsWith('SRDF')),
        group_states: probe.group_states || [],
        metadata,
      })
    }
    return sendJson(res, {
      installed: true,
      rig_filename: base,
      summary: metadata.rig,
      warnings: (metadata.warnings || []).filter(w => w.startsWith('rig')),
      metadata,
    })
  } catch (e) {
    return sendJson(res, { error: e.message || String(e) }, e.status || 400)
  }
}

function serveUrdf (res) {
  const m = st.getUrdfModel()
  if (!m) {
    res.writeHead(404, { 'Content-Type': 'text/plain' })
    res.end('No URDF installed')
    return true
  }
  res.writeHead(200, {
    'Content-Type': 'application/xml',
    'Cache-Control': 'no-cache, must-revalidate',
  })
  res.end(m.urdfBytes)
  return true
}

function serveMesh (res, filename) {
  // Accept nested URDF-relative paths (Meshes/Foo/bar.stl) — matching the
  // real server's /api/robot/meshes/{filename:.+} route — while still
  // rejecting path traversal and absolute paths.
  if (!filename || filename.includes('..') || filename.includes('\\') || filename.startsWith('/')) {
    res.writeHead(403, { 'Content-Type': 'text/plain' })
    res.end('Forbidden')
    return true
  }
  const m = st.getUrdfModel()
  const buf = m?.meshes?.get(filename) || m?.meshes?.get(filename.split('/').pop())
  if (!buf) {
    res.writeHead(404, { 'Content-Type': 'text/plain' })
    res.end('Mesh not found')
    return true
  }
  res.writeHead(200, {
    'Content-Type': mimeForMesh(filename),
    'Cache-Control': 'no-cache, must-revalidate',
  })
  res.end(buf)
  return true
}

function mimeForMesh (filename) {
  const ext = filename.toLowerCase().split('.').pop()
  if (ext === 'stl')  return 'model/stl'
  if (ext === 'dae')  return 'model/vnd.collada+xml'
  if (ext === 'obj')  return 'model/obj'
  if (ext === 'gltf') return 'model/gltf+json'
  if (ext === 'glb')  return 'model/gltf-binary'
  return 'application/octet-stream'
}

async function handleUrdfUpload (req, res) {
  try {
    const part = await readSingleFilePart(req)
    if (!part) {
      return sendJson(res, { error: 'Missing file field' }, 400)
    }
    const { filename, buffer } = part
    const lower = filename.toLowerCase()
    let model
    if (lower.endsWith('.zip')) {
      model = await installFromZip(buffer, filename)
    } else {
      model = installFromUrdf(buffer, filename)
    }
    st.setUrdfModel(model)
    // describe()-shaped, like the real server: the client wants the
    // companion summaries and warnings in the same round trip.
    return sendJson(res, await robotMetadataPayload())
  } catch (e) {
    const code = e.status || 400
    return sendJson(res, { error: e.message || String(e) }, code)
  }
}

async function installFromZip (zipBytes, originalFilename) {
  if (!zipBytes?.length) throw httpErr(400, 'empty upload')
  let zip
  try {
    zip = await JSZip.loadAsync(zipBytes)
  } catch (e) {
    throw httpErr(400, `not a valid zip file: ${e.message || e}`)
  }

  let urdfEntry = null
  const meshEntries = []
  const xmlEntries = []
  zip.forEach((relPath, entry) => {
    if (entry.dir) return
    // Reject any entry whose path tries to escape the bundle root —
    // same defensive policy as the real RobotModelStore.
    if (relPath.startsWith('/') || relPath.split('/').includes('..')) return
    const base = relPath.split('/').pop()
    const lower = relPath.toLowerCase()
    // Collect every candidate description file for a CONTENT sniff
    // below. URDF and SRDF share the <robot> root element and are
    // indistinguishable by tag, so the extension is the least
    // trustworthy signal available.
    if (/\.(urdf|srdf|xacro|xml|rig)$/i.test(lower)) {
      xmlEntries.push({ entry, base, relPath })
    } else {
      const ext = '.' + (base.toLowerCase().split('.').pop() || '')
      if (ALLOWED_MESH_EXT.has(ext)) meshEntries.push({ entry, base, relPath })
    }
  })

  // Classify each candidate by content. Mirrors
  // RobotModelStore._classify_xml, via the same Python that implements
  // it, so a `robot.srdf.xacro` or a bare `model.xml` routes the same
  // way here as it would on the real server.
  let srdfBytes = null, srdfName = ''
  let rigBytes = null, rigName = ''
  for (const cand of xmlEntries) {
    const buf = Buffer.from(await cand.entry.async('uint8array'))
    const { kind } = await bridge('classify', {
      data: buf.toString('utf8'), filename: cand.base,
    })
    if (kind === 'rig' && !rigBytes) { rigBytes = buf; rigName = cand.base }
    else if (kind === 'srdf' && !srdfBytes) { srdfBytes = buf; srdfName = cand.base }
    else if (kind === 'urdf' && !urdfEntry) {
      urdfEntry = { ...cand, bytes: buf }
    }
  }

  if (!urdfEntry) {
    throw httpErr(400, 'zip contains no URDF at any level (looked for a '
      + '<robot> document with <link>/<joint> geometry)')
  }

  const urdfBytes = urdfEntry.bytes
    || Buffer.from(await urdfEntry.entry.async('uint8array'))
  const { linkCount, jointCount } = validateUrdf(urdfBytes)

  // Key meshes by their path RELATIVE TO THE URDF's directory — the same
  // URDF-relative path the `<mesh filename>` refs use and that the viewer
  // requests (e.g. "Meshes/EyeMechanism/eye.stl"). This mirrors the real
  // server's URDFStore, which preserves nested paths. The old
  // flatten-to-basename behavior 404'd every nested mesh — so only the
  // primitive-geometry links (johnny5's eye-lens cylinders) rendered —
  // and collided same-named files from different subfolders. A basename
  // fallback is kept for flat/legacy refs.
  const urdfDir = urdfEntry.relPath.includes('/')
    ? urdfEntry.relPath.slice(0, urdfEntry.relPath.lastIndexOf('/') + 1)
    : ''
  const meshes = new Map()
  const meshFiles = []
  for (const { entry, base, relPath } of meshEntries) {
    const buf = Buffer.from(await entry.async('uint8array'))
    const rel = relPath.startsWith(urdfDir) ? relPath.slice(urdfDir.length) : relPath
    meshes.set(rel, buf)
    meshFiles.push(rel)
    if (!meshes.has(base)) meshes.set(base, buf) // basename fallback (first wins)
  }

  return {
    metadata: {
      original_filename: originalFilename,
      urdf_filename: urdfEntry.base,
      sha256: sha256Hex(urdfBytes),
      uploaded_at: Date.now() / 1000,
      mesh_files: meshFiles.sort(),
      link_count: linkCount,
      joint_count: jointCount,
      srdf_filename: srdfName,
      srdf_sha256: srdfBytes ? sha256Hex(srdfBytes) : '',
      srdf_uploaded_at: srdfBytes ? Date.now() / 1000 : 0,
      rig_filename: rigName,
      rig_sha256: rigBytes ? sha256Hex(rigBytes) : '',
      rig_uploaded_at: rigBytes ? Date.now() / 1000 : 0,
    },
    urdfBytes,
    meshes,
    srdfBytes,
    rigBytes,
  }
}

function installFromUrdf (urdfBytes, originalFilename) {
  const { linkCount, jointCount } = validateUrdf(urdfBytes)
  const base = (originalFilename.split(/[\\/]/).pop() || 'robot.urdf')
  const urdfName = /\.(urdf|xacro)$/i.test(base) ? base : 'robot.urdf'
  return {
    metadata: {
      original_filename: originalFilename,
      urdf_filename: urdfName,
      sha256: sha256Hex(urdfBytes),
      uploaded_at: Date.now() / 1000,
      mesh_files: [],
      link_count: linkCount,
      joint_count: jointCount,
      srdf_filename: '', srdf_sha256: '', srdf_uploaded_at: 0,
      rig_filename: '', rig_sha256: '', rig_uploaded_at: 0,
    },
    urdfBytes,
    meshes: new Map(),
    srdfBytes: null,
    rigBytes: null,
  }
}

function validateUrdf (urdfBytes) {
  const text = urdfBytes.toString('utf8')
  // Just enough check to refuse non-URDFs: the document must contain
  // a <robot ...> opening tag. The real server parses with ElementTree;
  // for the dev mock we count tags via regex which is plenty for the
  // metadata panel.
  if (!/<robot\b/i.test(text)) {
    throw httpErr(400, 'URDF root element must be <robot>')
  }
  const linkCount  = (text.match(/<link\s[^>]*>|<link>\s*</gi) || []).length
  const jointCount = (text.match(/<joint\s[^>]*>/gi) || []).length
  return { linkCount, jointCount }
}

// ── Maestro import ──────────────────────────────────────────────────

async function handleMaestroImport (req, res) {
  try {
    const parts = await readAllParts(req)
    const file = parts.find(p => p.name === 'file')
    if (!file) return sendJson(res, { error: 'Missing file field' }, 400)
    const sequenceField = parts.find(p => p.name === 'sequence')
    const sequenceName = sequenceField?.buffer?.toString('utf8').trim() || null

    const xml = file.buffer.toString('utf8')
    const sequences = listSequences(xml)
    const result = parseMaestroXml(xml, sequenceName)

    return sendJson(res, {
      animation: result.animation,
      channels: result.channels,
      sequence_name: result.animation.name,
      frame_count: result.frame_count,
      sequences: sequenceName ? undefined : sequences,
    })
  } catch (e) {
    const code = e.status || 400
    return sendJson(res, { error: e.message || String(e) }, code)
  }
}

const QUS_PER_US = 4.0

function listSequences (xml) {
  // Maestro saves are small (<1MB) — a regex scan is faster to read
  // and ship than wiring up a real XML parser.
  const out = []
  const re = /<Sequence\b([^>]*)>/g
  let m
  while ((m = re.exec(xml)) !== null) {
    const nameMatch = m[1].match(/name="([^"]*)"/)
    if (nameMatch) out.push(nameMatch[1])
  }
  return out
}

function parseMaestroXml (xml, sequenceName) {
  const sequences = extractSequences(xml)
  if (!sequences.length) throw httpErr(400, 'XML contains no <Sequence> blocks')
  let chosen
  if (sequenceName) {
    const target = sequenceName.toLowerCase()
    chosen = sequences.find(s => (s.name || '').toLowerCase() === target)
    if (!chosen) throw httpErr(400, `No sequence named ${JSON.stringify(sequenceName)}`)
  } else {
    chosen = sequences[0]
  }

  const perChannel = new Map()    // index → [{time, value, interp, …}]
  const extremes = new Map()      // index → [min, max]
  let t = 0
  for (const frame of chosen.frames) {
    const positions = frame.positions
    for (let i = 0; i < positions.length; i++) {
      const qus = positions[i]
      if (!Number.isFinite(qus)) continue
      const pulseUs = qus / QUS_PER_US
      if (!perChannel.has(i)) perChannel.set(i, [])
      perChannel.get(i).push({
        time: t, value: pulseUs, interp: 1,
        arrive_tangent: 0, leave_tangent: 0,
      })
      const ext = extremes.get(i) || [pulseUs, pulseUs]
      if (pulseUs < ext[0]) ext[0] = pulseUs
      if (pulseUs > ext[1]) ext[1] = pulseUs
      extremes.set(i, ext)
    }
    t += frame.durationMs / 1000.0
  }

  const valueTracks = []
  const channels = []
  for (const idx of [...perChannel.keys()].sort((a, b) => a - b)) {
    valueTracks.push({
      id: `ch${idx}`,
      name: `Channel ${idx}`,
      curve: { name: `channel_${idx}`, keys: perChannel.get(idx) },
    })
    const ext = extremes.get(idx)
    channels.push({
      index: idx,
      min_value: ext[0],
      max_value: ext[1],
      keyframe_count: perChannel.get(idx).length,
    })
  }

  return {
    animation: {
      id: st.slugify(chosen.name || 'imported'),
      name: chosen.name || 'imported',
      duration: t,
      fps: 60,
      loop: false,
      value_tracks: valueTracks,
      trigger_tracks: [],
      created: '',
      modified: '',
    },
    channels,
    frame_count: chosen.frames.length,
  }
}

function extractSequences (xml) {
  // Each <Sequence> can contain <Frame Duration="…"><Positions>…</Positions></Frame>
  const result = []
  const seqRe = /<Sequence\b([^>]*)>([\s\S]*?)<\/Sequence>/g
  let m
  while ((m = seqRe.exec(xml)) !== null) {
    const nameMatch = m[1].match(/name="([^"]*)"/)
    const body = m[2]
    const frames = []
    const frameRe = /<Frame\b([^>]*)(?:\/>|>([\s\S]*?)<\/Frame>)/g
    let f
    while ((f = frameRe.exec(body)) !== null) {
      const attrs = f[1]
      const inner = f[2] || ''
      const durMatch = attrs.match(/Duration="([^"]*)"/)
      const durationMs = durMatch ? parseFloat(durMatch[1]) : 0
      let positionsText = ''
      const posChild = inner.match(/<Positions[^>]*>([\s\S]*?)<\/Positions>/)
      if (posChild) positionsText = posChild[1]
      if (!positionsText) {
        const posAttr = attrs.match(/Positions="([^"]*)"/)
        if (posAttr) positionsText = posAttr[1]
      }
      const positions = positionsText.trim().split(/\s+/)
        .map(s => parseFloat(s)).filter(n => Number.isFinite(n))
      frames.push({ durationMs, positions })
    }
    result.push({ name: nameMatch ? nameMatch[1] : '', frames })
  }
  return result
}

// ── Multipart parsing ───────────────────────────────────────────────
//
// Single-file uploads are the only shape the Vue front-end sends —
// FormData with one file part (URDF) plus an optional text part
// (Maestro sequence name). Hand-rolled to avoid a multipart dep just
// for the dev mock.

async function readBody (req) {
  if (req._body) return req._body
  const total = parseInt(req.headers['content-length'] || '0', 10)
  if (total > MAX_URDF_UPLOAD_BYTES) {
    throw httpErr(413, `Upload exceeds ${MAX_URDF_UPLOAD_BYTES} byte limit`)
  }
  const chunks = []
  let size = 0
  for await (const chunk of req) {
    chunks.push(chunk)
    size += chunk.length
    if (size > MAX_URDF_UPLOAD_BYTES) {
      throw httpErr(413, `Upload exceeds ${MAX_URDF_UPLOAD_BYTES} byte limit`)
    }
  }
  req._body = Buffer.concat(chunks)
  return req._body
}

async function readAllParts (req) {
  const ct = req.headers['content-type'] || ''
  const m = ct.match(/boundary=(?:"([^"]+)"|([^;]+))/i)
  if (!m) throw httpErr(400, 'Expected multipart/form-data')
  const boundary = '--' + (m[1] || m[2]).trim()
  const body = await readBody(req)
  return splitMultipart(body, boundary)
}

async function readSingleFilePart (req) {
  const parts = await readAllParts(req)
  return parts.find(p => p.filename) || null
}

function splitMultipart (body, boundary) {
  const parts = []
  const boundaryBuf = Buffer.from(boundary)
  const closingBuf = Buffer.from(boundary + '--')
  let offset = body.indexOf(boundaryBuf)
  if (offset < 0) return parts
  offset += boundaryBuf.length
  while (offset < body.length) {
    // Skip CRLF after the boundary line.
    if (body[offset] === 0x0d) offset += 2  // \r\n
    if (offset >= body.length) break
    // Locate end of headers (\r\n\r\n).
    const headerEnd = body.indexOf(Buffer.from('\r\n\r\n'), offset)
    if (headerEnd < 0) break
    const headerText = body.slice(offset, headerEnd).toString('utf8')
    const partStart = headerEnd + 4
    // Locate next boundary marker for the part body.
    let next = body.indexOf(boundaryBuf, partStart)
    if (next < 0) break
    // Trim the trailing CRLF that precedes the boundary.
    const partEnd = next - 2 >= partStart ? next - 2 : next
    const part = parsePartHeaders(headerText, body.slice(partStart, partEnd))
    parts.push(part)
    // Check for closing boundary --
    if (body.slice(next, next + closingBuf.length).equals(closingBuf)) break
    offset = next + boundaryBuf.length
  }
  return parts
}

function parsePartHeaders (headerText, bodyBuf) {
  const disposition = headerText.split(/\r?\n/)
    .find(l => /^content-disposition:/i.test(l)) || ''
  const nameMatch = disposition.match(/\bname="([^"]*)"/i)
  const filenameMatch = disposition.match(/\bfilename="([^"]*)"/i)
  return {
    name: nameMatch ? nameMatch[1] : '',
    filename: filenameMatch ? filenameMatch[1] : '',
    buffer: bodyBuf,
  }
}

// ── helpers ─────────────────────────────────────────────────────────

function sendJson (res, body, status = 200) {
  res.writeHead(status, { 'Content-Type': 'application/json' })
  res.end(JSON.stringify(body))
  return true
}

function sha256Hex (buf) {
  return createHash('sha256').update(buf).digest('hex')
}

function httpErr (status, message) {
  const e = new Error(message)
  e.status = status
  return e
}
