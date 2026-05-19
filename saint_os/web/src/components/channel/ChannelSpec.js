// Per-(type, channel) UI hints. Keep this table tiny — the goal is a
// natural range + label for the common writable channels. Everything
// else falls back to a sensible analog/digital default.
const overrides = {
  'roboclaw/motor':      { kind: 'slider', min: -1, max: 1,   step: 0.01, neutral: 0,   format: v => `${(v * 100).toFixed(0)}%`, hint: '-100% reverse · 0 stop · +100% forward' },
  'syren/motor':         { kind: 'slider', min: -1, max: 1,   step: 0.01, neutral: 0,   format: v => `${(v * 100).toFixed(0)}%`, hint: '-100% reverse · 0 stop · +100% forward' },
  'servo/angle':         { kind: 'slider', min: 0,  max: 180, step: 1,    neutral: 90,  format: v => `${v.toFixed(0)}°` },
  'led/on':              { kind: 'toggle' },
  'neopixel/color':      { kind: 'color',  unsupported: true, note: 'No firmware control path yet.' },
  'neopixel/brightness': { kind: 'slider', min: 0,  max: 1,   step: 0.01, neutral: 0.5, format: v => `${(v * 100).toFixed(0)}%`, unsupported: true, note: 'No firmware control path yet.' },
}

export function specFor (peripheralType, channel) {
  const key = `${peripheralType}/${channel.id}`
  if (overrides[key]) return overrides[key]
  if (channel.cap === 'digital_out') return { kind: 'toggle' }
  if (channel.cap === 'rgb')         return { kind: 'color', unsupported: true, note: 'No firmware control path yet.' }
  return { kind: 'slider', min: 0, max: 1, step: 0.01, neutral: 0, format: v => v.toFixed(2) }
}
