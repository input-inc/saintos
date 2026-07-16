// Per-(type, channel) UI hints. Keep this table tiny — the goal is a
// natural range + label for the common writable channels. Everything
// else falls back to a sensible analog/digital default.
const overrides = {
  'roboclaw/motor':      { kind: 'slider', min: -1, max: 1,   step: 0.01, neutral: 0,   format: v => `${(v * 100).toFixed(0)}%`, hint: '-100% reverse · 0 stop · +100% forward' },
  'syren/motor':         { kind: 'slider', min: -1, max: 1,   step: 0.01, neutral: 0,   format: v => `${(v * 100).toFixed(0)}%`, hint: '-100% reverse · 0 stop · +100% forward' },
  // Native (direct-PWM) servo. Bipolar −1…+1 exactly like Maestro: the
  // firmware (pin_control_set_servo) maps −1 → start_us, 0 → center_us,
  // +1 → end_us using the extents stored in the node's flash. The old
  // 0–180° range sent raw degrees the firmware clamped to +1, so every
  // command slammed the servo to end_us and the extents never took —
  // keep this in lock-step with the maestro branch below.
  'servo/angle':         { kind: 'slider', min: -1, max: 1,   step: 0.01, neutral: 0,   format: v => Number(v).toFixed(2), hint: '−1 start · 0 center · +1 end' },
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
    // −1…+1 maps to the channel's configured [min, max] extents in the
    // firmware, so the slider already can't command outside the range.
    // Show the raw routed signal (−1.00 … 1.00), not a percentage.
    return { kind: 'slider', min: -1, max: 1, step: 0.01, neutral: 0,
             format: v => Number(v).toFixed(2) }
  }
  if (channel.cap === 'digital_out') return { kind: 'toggle' }
  if (channel.cap === 'rgb')         return { kind: 'color', unsupported: true, note: 'No firmware control path yet.' }
  return { kind: 'slider', min: 0, max: 1, step: 0.01, neutral: 0, format: v => v.toFixed(2) }
}
