// HTTP-side handlers for the dev mock server. Implements the URDF
// upload/serve routes and Maestro Pololu .xml parsing — same surface
// as the real Python server's http_server.py, but backed by in-memory
// state and a Node-side curve/zip pipeline.

import { createHash } from 'node:crypto'
import JSZip from 'jszip'

import * as st from './mock-state.js'

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
    return sendJson(res, robotMetadataPayload())
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
  if (req.method === 'GET' && url === '/api/robot/joints') {
    return sendJson(res, { joints: listUrdfJoints() })
  }
  if (req.method === 'GET' && url.startsWith('/api/robot/meshes/')) {
    return serveMesh(res, decodeURIComponent(url.slice('/api/robot/meshes/'.length)))
  }
  if (req.method === 'POST' && url === '/api/animations/import/maestro') {
    return await handleMaestroImport(req, res)
  }
  return false
}

// Parse the installed URDF (if any) and return the list of actuatable
// joints. Skips fixed joints because they don't accept setpoints.
// Mirrors the real server's URDFStore.list_joints.
function listUrdfJoints () {
  const m = st.getUrdfModel()
  if (!m) return []
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

function robotMetadataPayload () {
  const m = st.getUrdfModel()
  if (!m) return { installed: false }
  return { installed: true, ...m.metadata }
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
  if (!filename || filename.includes('/') || filename.includes('\\') || filename.includes('..')) {
    res.writeHead(403, { 'Content-Type': 'text/plain' })
    res.end('Forbidden')
    return true
  }
  const m = st.getUrdfModel()
  const buf = m?.meshes?.get(filename)
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
    return sendJson(res, { installed: true, ...model.metadata })
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
  zip.forEach((relPath, entry) => {
    if (entry.dir) return
    // Reject any entry whose path tries to escape the bundle root —
    // same defensive policy as the real URDFStore.
    if (relPath.startsWith('/') || relPath.split('/').includes('..')) return
    const base = relPath.split('/').pop()
    const lower = relPath.toLowerCase()
    if ((lower.endsWith('.urdf') || lower.endsWith('.xacro')) && !urdfEntry) {
      urdfEntry = { entry, base }
    } else {
      const ext = '.' + (base.toLowerCase().split('.').pop() || '')
      if (ALLOWED_MESH_EXT.has(ext)) meshEntries.push({ entry, base })
    }
  })

  if (!urdfEntry) throw httpErr(400, 'zip contains no .urdf file at any level')

  const urdfBytes = Buffer.from(await urdfEntry.entry.async('uint8array'))
  const { linkCount, jointCount } = validateUrdf(urdfBytes)

  const meshes = new Map()
  for (const { entry, base } of meshEntries) {
    const buf = Buffer.from(await entry.async('uint8array'))
    meshes.set(base, buf)
  }

  return {
    metadata: {
      original_filename: originalFilename,
      urdf_filename: urdfEntry.base,
      sha256: sha256Hex(urdfBytes),
      uploaded_at: Date.now() / 1000,
      mesh_files: [...meshes.keys()].sort(),
      link_count: linkCount,
      joint_count: jointCount,
    },
    urdfBytes,
    meshes,
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
    },
    urdfBytes,
    meshes: new Map(),
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
