// Per-(type, channel) UI hints. Keep this table tiny — the goal is a
// natural range + label for the common writable channels. Everything
// else falls back to a sensible analog/digital default.
const overrides = {
  'roboclaw/motor':      { kind: 'slider', min: -1, max: 1,   step: 0.01, neutral: 0,   format: v => `${(v * 100).toFixed(0)}%`, hint: '-100% reverse · 0 stop · +100% forward' },
  'syren/motor':         { kind: 'slider', min: -1, max: 1,   step: 0.01, neutral: 0,   format: v => `${(v * 100).toFixed(0)}%`, hint: '-100% reverse · 0 stop · +100% forward' },
  'servo/angle':         { kind: 'slider', min: 0,  max: 180, step: 1,    neutral: 90,  format: v => `${v.toFixed(0)}°` },
  'led/on':              { kind: 'toggle' },
  // Firmware routes neopixel/color through pin_control.c's
  // apply_set_channel → led_set_override_color. The picker emits a
  // packed uint24 (0xRRGGBB) which the server forwards as a float —
  // atof() round-trips integers up to 2^24 exactly.
  'neopixel/color':      { kind: 'color',  hint: "Sticks until the node's connection state changes." },
  // Brightness-only override isn't wired server-side yet (the
  // firmware refuses it with a warn until color + brightness are
  // tracked independently). Keep the unsupported flag.
  'neopixel/brightness': { kind: 'slider', min: 0,  max: 1,   step: 0.01, neutral: 0.5, format: v => `${(v * 100).toFixed(0)}%`, unsupported: true, note: 'Set color first via the picker — brightness-only override not yet implemented.' },
}

export function specFor (peripheralType, channel) {
  const key = `${peripheralType}/${channel.id}`
  if (overrides[key]) return overrides[key]
  // Maestro servo channels are bipolar: the firmware maps the routed
  // signal −1 → min pulse, 0 → neutral, +1 → max pulse (see
  // drv_set_value in firmware/shared/src/maestro_driver.c). The 24
  // channels share one catalog entry with per-channel ids (chN), so
  // key off the type rather than enumerating every channel. Centered
  // on 0 so the slider rests at neutral and can reach both sides of
  // the servo's travel.
  if (peripheralType === 'maestro') {
    return { kind: 'slider', min: -1, max: 1, step: 0.01, neutral: 0,
             format: v => `${(v * 100).toFixed(0)}%`,
             hint: '−100% min · 0 center · +100% max' }
  }
  if (channel.cap === 'digital_out') return { kind: 'toggle' }
  if (channel.cap === 'rgb')         return { kind: 'color', unsupported: true, note: 'No firmware control path yet.' }
  return { kind: 'slider', min: 0, max: 1, step: 0.01, neutral: 0, format: v => v.toFixed(2) }
}
