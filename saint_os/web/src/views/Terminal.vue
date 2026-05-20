<script setup>
import { onMounted, onUnmounted, ref } from 'vue'
import { Terminal } from '@xterm/xterm'
import { FitAddon } from '@xterm/addon-fit'
import '@xterm/xterm/css/xterm.css'
import { useWsStore } from '@/stores/ws'

const ws = useWsStore()
const mount = ref(null)
const status = ref('Not connected')
let term = null
let fit = null
let binaryHandler = null
let exitHandler = null
let inputDispose = null
let resizeObs = null
let resizeTimer = null
let opened = false

async function open () {
  if (!mount.value) return
  if (!term) {
    term = new Terminal({
      cursorBlink: true, fontSize: 13,
      fontFamily: '"JetBrains Mono", "Menlo", "Consolas", monospace',
      scrollback: 5000,
      theme: { background: '#000000', foreground: '#e2e8f0', cursor: '#22d3ee', selectionBackground: '#334155' },
      convertEol: false,
    })
    fit = new FitAddon()
    term.loadAddon(fit)
    term.open(mount.value)
  }
  fit.fit()
  const { cols, rows } = term

  status.value = 'Starting shell…'
  try {
    await ws.management('terminal.open', { cols, rows })
  } catch (e) {
    status.value = `Failed: ${e.message}`
    term.writeln(`\r\n\x1b[31m[saint-os] ${e.message}\x1b[0m`)
    return
  }
  status.value = `Live  (${cols}×${rows})`

  binaryHandler = (buf) => term.write(new Uint8Array(buf))
  ws.on('binary', binaryHandler)
  exitHandler = (msg) => {
    if (msg?.node !== 'terminal') return
    term.writeln(`\r\n\x1b[33m[saint-os] shell exited (code=${msg.data?.code})\x1b[0m`)
    status.value = 'Shell exited'
    opened = false
  }
  ws.on('state', exitHandler)

  inputDispose = term.onData((data) => {
    ws.management('terminal.input', { data }).catch(() => {})
  })

  resizeObs = new ResizeObserver(() => {
    clearTimeout(resizeTimer)
    resizeTimer = setTimeout(() => {
      try { fit.fit() } catch (_) { return }
      const { cols, rows } = term
      ws.management('terminal.resize', { cols, rows }).catch(() => {})
      status.value = `Live  (${cols}×${rows})`
    }, 100)
  })
  resizeObs.observe(mount.value)

  opened = true
  term.focus()
}

async function close () {
  if (binaryHandler) { ws.off('binary', binaryHandler); binaryHandler = null }
  if (exitHandler)   { ws.off('state', exitHandler);     exitHandler = null }
  if (inputDispose)  { inputDispose.dispose();           inputDispose = null }
  if (resizeObs)     { resizeObs.disconnect();           resizeObs = null }
  clearTimeout(resizeTimer)
  if (opened) {
    try { await ws.management('terminal.close', {}) } catch (_) {}
  }
  opened = false
  status.value = 'Not connected'
}

async function restart () {
  await close()
  if (term) { term.clear(); term.reset() }
  await open()
}

onMounted(open)
onUnmounted(close)
</script>

<template>
  <section class="flex flex-col" style="height: calc(100vh - 12rem);">
    <div class="flex items-center justify-between mb-3">
      <h2 class="text-2xl font-bold text-white">Terminal</h2>
      <div class="flex items-center gap-3">
        <span class="text-xs text-slate-400 font-mono">{{ status }}</span>
        <button class="btn-secondary text-sm" @click="restart">
          <span class="material-icons icon-sm">restart_alt</span>
          Restart
        </button>
      </div>
    </div>
    <div ref="mount" class="flex-1 bg-black border border-slate-700 rounded-lg overflow-hidden" />
  </section>
</template>
