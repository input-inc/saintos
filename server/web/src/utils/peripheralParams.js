/**
 * Peripheral save-payload helpers.
 *
 * Extracted from views/node/Peripherals.vue so the two pieces of logic
 * that can silently destroy operator work are testable in isolation:
 * what a save carries, and what a save would discard.
 */

/**
 * Build the `params` object to send for a peripheral save.
 *
 * Two rules, and they pull in opposite directions:
 *
 *  1. Declared params hidden by `visible_when` must be DROPPED, so a
 *     stale value (a MAC typed before switching transport back to UART)
 *     can't reach the firmware.
 *  2. Params the type does NOT declare must be KEPT. `params.channels`
 *     — per-channel labels, icons and extents — is deliberately not a
 *     PeripheralTypeParam (see the schema note in peripheral_model.py),
 *     so it is invisible to rule 1's loop.
 *
 * Building the payload from `type.params` alone honoured rule 1 and
 * violated rule 2: every peripheral save dropped `channels`, the
 * server's maestro_normalize_channels then saw no channels array and
 * regenerated defaults, and opening a Maestro, changing nothing and
 * pressing Save wiped every channel name and extent.
 *
 * @param {Array<{id: string}>} typeParams    the type's declared params
 * @param {Object} draftParams                 the modal's working copy
 * @param {(param: object) => boolean} isVisible  `visible_when` evaluator
 * @returns {Object} params to send
 */
export function buildSaveParams (typeParams, draftParams, isVisible) {
  const declared = new Set((typeParams || []).map(p => p.id))
  const out = {}
  // Rule 2 first: carry through anything the type doesn't describe.
  for (const [key, value] of Object.entries(draftParams || {})) {
    if (!declared.has(key)) out[key] = value
  }
  // Rule 1: declared params, minus the hidden ones.
  for (const param of (typeParams || [])) {
    if (isVisible(param)) out[param.id] = (draftParams || {})[param.id]
  }
  return out
}

/**
 * Extent defaults a channel inherits when it has none of its own.
 * Mirrors _maestro_default_channel in peripheral_model.py.
 */
export function channelFallbacks (peripheralParams) {
  const min = Number(peripheralParams?.min_pulse_us) || 1000
  const max = Number(peripheralParams?.max_pulse_us) || 2000
  return { min, max, mid: Math.floor((min + max) / 2) }
}

/**
 * Whether a channel holds anything an operator would miss.
 *
 * Errs toward "configured": a spurious confirmation prompt costs a
 * click, a silent wipe costs a re-bringup. "Ch <n>" is the default
 * label and carries no information the channel id doesn't, so it does
 * not count as named.
 */
export function channelHasConfig (ch, idx, fallbacks) {
  if (!ch || typeof ch !== 'object') return false

  const label = String(ch.label ?? '').trim()
  if (label && label !== `Ch ${idx}`) return true
  if (String(ch.icon ?? '').trim()) return true

  if (Number(ch.speed) || Number(ch.acceleration) || Number(ch.idle_disengage_ms)) {
    return true
  }

  const fb = fallbacks || { min: 1000, max: 2000, mid: 1500 }
  if (ch.min_pulse_us != null && Number(ch.min_pulse_us) !== fb.min) return true
  if (ch.max_pulse_us != null && Number(ch.max_pulse_us) !== fb.max) return true
  if (ch.neutral_us != null && Number(ch.neutral_us) !== fb.mid) return true
  if (ch.home_us != null && Number(ch.home_us) !== fb.mid) return true

  return false
}

/**
 * Configured channels a channel_count reduction would discard.
 *
 * The server keeps channels[0..count-1] and truncates the tail, so
 * growing the count is lossless (the new entries get defaults) and
 * shrinking it destroys the tail with no undo.
 *
 * @returns {Array<{idx: number, label: string}>} empty when nothing is at risk
 */
export function channelsLostByCountChange (storedChannels, nextCount, fallbacks) {
  if (!Array.isArray(storedChannels) || !storedChannels.length) return []
  // An explicit 0 must mean zero, not "unset" — `Number(x) || fallback`
  // would silently turn it into "no change" and suppress the warning.
  // Only a missing/unparseable count means we can't tell, and then the
  // safe answer is "nothing is being dropped".
  const parsed = Number(nextCount)
  const count = Number.isFinite(parsed) ? parsed : storedChannels.length
  if (count >= storedChannels.length) return []
  const lost = []
  for (let i = count; i < storedChannels.length; i++) {
    if (channelHasConfig(storedChannels[i], i, fallbacks)) {
      lost.push({ idx: i, label: storedChannels[i]?.label || `Ch ${i}` })
    }
  }
  return lost
}
