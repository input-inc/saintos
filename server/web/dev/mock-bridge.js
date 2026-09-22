// Robot-model operations, delegated to a long-lived Python worker.
//
// The mock parses SRDFs, validates cross-file references, and evaluates
// the control rig by talking to `server/scripts/robot_model_bridge.py`
// rather than reimplementing any of it in JS. Same rationale as the
// catalog dump in mock-server.js: a hand-maintained JS copy of the blend
// math, the clamp policy, and the mimic round-trip would silently drift,
// and then the mock would be demonstrating something the real server
// doesn't do.
//
// WHY A PERSISTENT WORKER, not spawnSync per call:
//
// The rig arithmetic is genuinely sub-millisecond, but Python interpreter
// startup plus imports is ~46 ms — and the rig panel fires on every
// slider `input` event, roughly 30 a second. Spawning per call (and the
// first version did it TWICE per request) put the mock about 2.7x behind
// realtime with an unbounded backlog, which surfaced as
// "Request timeout" once the queue passed the client's 30 s window.
//
// One worker, line-delimited JSON, requests correlated by id. Roughly
// 1 ms per call, and it never blocks the event loop.

import { spawn } from 'node:child_process'
import { fileURLToPath } from 'node:url'
import path from 'node:path'

const HERE = path.dirname(fileURLToPath(import.meta.url))
const SCRIPT = path.resolve(HERE, '..', '..', 'scripts', 'robot_model_bridge.py')
const PYTHON = process.env.SAINT_PYTHON || 'python3'

// A single request should never take anywhere near this; the cap exists
// so a wedged worker surfaces as an error instead of a hung UI.
const REQUEST_TIMEOUT_MS = 10000

let child = null
let buffer = ''
let seq = 0
const pending = new Map()
let warned = false
let disabled = false

function warnOnce (msg) {
  if (warned) return
  warned = true
  console.error(`[mock] robot_model_bridge unavailable: ${msg}`)
  console.error(`[mock] tried: ${PYTHON} ${SCRIPT} serve`)
  console.error('[mock] export SAINT_PYTHON if python3 is somewhere unusual')
  console.error('[mock] SRDF / rig features will be inert until fixed')
}

function rejectAll (err) {
  for (const p of pending.values()) {
    clearTimeout(p.timer)
    p.resolve({ error: err })
  }
  pending.clear()
}

function ensureWorker () {
  if (disabled) return null
  if (child && !child.killed && child.exitCode === null) return child

  try {
    child = spawn(PYTHON, [SCRIPT, 'serve'], {
      stdio: ['pipe', 'pipe', 'pipe'],
    })
  } catch (e) {
    warnOnce(e.message)
    disabled = true
    return null
  }

  buffer = ''
  child.stdout.setEncoding('utf8')
  child.stdout.on('data', (chunk) => {
    buffer += chunk
    // Line-delimited: one complete response per newline. A partial tail
    // stays in the buffer for the next chunk.
    let nl
    while ((nl = buffer.indexOf('\n')) >= 0) {
      const line = buffer.slice(0, nl)
      buffer = buffer.slice(nl + 1)
      if (!line.trim()) continue
      let msg
      try {
        msg = JSON.parse(line)
      } catch (e) {
        console.error(`[mock] bridge sent non-JSON: ${line.slice(0, 200)}`)
        continue
      }
      const p = pending.get(msg.id)
      if (!p) continue
      pending.delete(msg.id)
      clearTimeout(p.timer)
      p.resolve(msg.result ?? { error: 'bridge returned no result' })
    }
  })

  // stderr is the worker's own diagnostics (tracebacks from a bug in the
  // bridge itself). Surface rather than swallow — a silently broken
  // worker is exactly what this whole file is trying to avoid.
  child.stderr.setEncoding('utf8')
  child.stderr.on('data', (d) => {
    const t = String(d).trim()
    if (t) console.error(`[mock] bridge stderr: ${t}`)
  })

  child.on('error', (e) => {
    warnOnce(e.message)
    disabled = true
    rejectAll(e.message)
  })

  child.on('exit', (code, signal) => {
    // Don't mark disabled: a crash on one bad input shouldn't kill the
    // feature for the session. The next call respawns.
    if (code !== 0 && code !== null) {
      console.error(`[mock] bridge worker exited (code ${code}${signal ? ', ' + signal : ''}); will respawn`)
    }
    rejectAll('bridge worker exited')
    child = null
  })

  return child
}

/**
 * Run one bridge command. Resolves to the parsed result object, or an
 * object with an `error` key. Never rejects — the mock should degrade to
 * "no SRDF features" rather than 500 on every request.
 *
 * @returns {Promise<object>}
 */
export function bridge (command, payload = {}) {
  const worker = ensureWorker()
  if (!worker) return Promise.resolve({ error: 'bridge unavailable' })

  const id = ++seq
  return new Promise((resolve) => {
    const timer = setTimeout(() => {
      if (pending.delete(id)) {
        console.error(`[mock] bridge ${command} timed out after ${REQUEST_TIMEOUT_MS}ms`)
        resolve({ error: `${command} timed out` })
      }
    }, REQUEST_TIMEOUT_MS)
    pending.set(id, { resolve, timer })
    try {
      worker.stdin.write(JSON.stringify({ id, cmd: command, payload }) + '\n')
    } catch (e) {
      pending.delete(id)
      clearTimeout(timer)
      resolve({ error: e.message })
    }
  })
}

/** True if the worker answers a trivial request. */
export async function bridgeAvailable () {
  const r = await bridge('classify', {
    data: '<robot name="probe"><link name="a"/></robot>',
  })
  return r.kind === 'urdf'
}

/** Shut the worker down (used on mock-server exit). */
export function stopBridge () {
  if (child && !child.killed) child.kill()
  child = null
}
