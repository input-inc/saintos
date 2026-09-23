// Lock-down tests for the peripheral save payload (2026-09).
//
// Regression: opening a Maestro's Edit modal, changing nothing, and
// pressing Save cleared every channel name and extent value.
//
// saveModal built its `params` payload by looping over the TYPE's
// declared params. `params.channels` — per-channel labels, icons and
// extents — is deliberately not a PeripheralTypeParam (see the schema
// note in peripheral_model.py), so it wasn't in that loop and was
// dropped from every save. The server's maestro_normalize_channels then
// found no channels array and regenerated defaults. A 24-channel rig
// lost 24 names and 96 extent values to one no-op Save.
import { describe, expect, it } from 'vitest'
import {
  buildSaveParams,
  channelFallbacks,
  channelHasConfig,
  channelsLostByCountChange,
} from '../peripheralParams'

const ALWAYS = () => true

describe('buildSaveParams', () => {
  it('preserves params the type does not declare (the regression)', () => {
    const typeParams = [{ id: 'channel_count' }, { id: 'min_pulse_us' }]
    const draft = {
      channel_count: 24,
      min_pulse_us: 900,
      // Not declared by the type — must survive.
      channels: [{ label: 'Pan', min_pulse_us: 1200 }],
    }
    const out = buildSaveParams(typeParams, draft, ALWAYS)
    expect(out.channels).toEqual([{ label: 'Pan', min_pulse_us: 1200 }])
    expect(out.channel_count).toBe(24)
  })

  it('still drops declared params hidden by visible_when', () => {
    const typeParams = [{ id: 'transport' }, { id: 'mac' }]
    const draft = { transport: 'uart', mac: 'AA:BB:CC:DD:EE:FF' }
    // `mac` only applies to a BLE transport; hidden means stale.
    const isVisible = p => p.id !== 'mac'
    const out = buildSaveParams(typeParams, draft, isVisible)
    expect(out).toEqual({ transport: 'uart' })
    expect('mac' in out).toBe(false)
  })

  it('a hidden declared param does not resurrect via the undeclared pass', () => {
    // The undeclared pass runs first, so a param that is declared AND
    // hidden must not be carried through by it.
    const typeParams = [{ id: 'mac' }]
    const draft = { mac: 'AA:BB:CC:DD:EE:FF', channels: [] }
    const out = buildSaveParams(typeParams, draft, () => false)
    expect('mac' in out).toBe(false)
    expect(out.channels).toEqual([])
  })

  it('tolerates a type with no params and an empty draft', () => {
    expect(buildSaveParams(null, null, ALWAYS)).toEqual({})
    expect(buildSaveParams([], { channels: [1] }, ALWAYS)).toEqual({ channels: [1] })
  })
})

describe('channelHasConfig', () => {
  const fb = channelFallbacks({ min_pulse_us: 1000, max_pulse_us: 2000 })

  it('treats the default "Ch <n>" label as unnamed', () => {
    // Otherwise every untouched channel triggers a data-loss warning and
    // the prompt becomes noise the operator learns to click through.
    expect(channelHasConfig({ label: 'Ch 7' }, 7, fb)).toBe(false)
    expect(channelHasConfig({ label: '' }, 7, fb)).toBe(false)
    expect(channelHasConfig({}, 7, fb)).toBe(false)
  })

  it('counts a real name, an icon, or motion defaults as configured', () => {
    expect(channelHasConfig({ label: 'Pan' }, 7, fb)).toBe(true)
    expect(channelHasConfig({ icon: 'rotate_right' }, 7, fb)).toBe(true)
    expect(channelHasConfig({ speed: 30 }, 7, fb)).toBe(true)
    expect(channelHasConfig({ acceleration: 5 }, 7, fb)).toBe(true)
    expect(channelHasConfig({ idle_disengage_ms: 2000 }, 7, fb)).toBe(true)
  })

  it('counts extents that differ from the inherited fallbacks', () => {
    expect(channelHasConfig({ min_pulse_us: 1200 }, 0, fb)).toBe(true)
    expect(channelHasConfig({ home_us: 1700 }, 0, fb)).toBe(true)
    // Matching the fallbacks is what a default channel looks like.
    expect(channelHasConfig(
      { min_pulse_us: 1000, max_pulse_us: 2000, neutral_us: 1500, home_us: 1500 },
      0, fb,
    )).toBe(false)
  })

  it('measures against the peripheral-level fallbacks, not hardcoded values', () => {
    // A Maestro whose Advanced section sets 900/2100 gives defaults of
    // 900/2100/1500 — a channel at 1000/2000 is then a real deviation.
    const wide = channelFallbacks({ min_pulse_us: 900, max_pulse_us: 2100 })
    expect(wide.mid).toBe(1500)
    expect(channelHasConfig({ min_pulse_us: 900, max_pulse_us: 2100 }, 0, wide)).toBe(false)
    expect(channelHasConfig({ min_pulse_us: 1000 }, 0, wide)).toBe(true)
  })

  it('is false for junk entries', () => {
    expect(channelHasConfig(null, 0, fb)).toBe(false)
    expect(channelHasConfig('nope', 0, fb)).toBe(false)
  })
})

describe('channelsLostByCountChange', () => {
  const fb = channelFallbacks({})
  const stored = [
    { label: 'Pan' },        // 0
    { label: 'Tilt' },       // 1
    { label: 'Ch 2' },       // 2 — untouched
    { label: 'Gripper' },    // 3
  ]

  it('reports the configured channels a reduction would discard', () => {
    expect(channelsLostByCountChange(stored, 2, fb)).toEqual([
      { idx: 3, label: 'Gripper' },
    ])
  })

  it('ignores untouched channels in the discarded range', () => {
    // Fixture where the ONLY discarded channel is an untouched one:
    // dropping "Ch 2" loses nothing worth interrupting a save for.
    const tail = [{ label: 'Pan' }, { label: 'Tilt' }, { label: 'Ch 2' }]
    expect(channelsLostByCountChange(tail, 2, fb)).toEqual([])
  })

  it('still warns when a configured channel sits past the new limit', () => {
    // Cutting 4 -> 3 drops ch3 ("Gripper"), which is configured.
    expect(channelsLostByCountChange(stored, 3, fb)).toEqual([
      { idx: 3, label: 'Gripper' },
    ])
  })

  it('treats growing the count as lossless', () => {
    // The server pads with defaults; nothing existing is touched.
    expect(channelsLostByCountChange(stored, 24, fb)).toEqual([])
    expect(channelsLostByCountChange(stored, 4, fb)).toEqual([])
  })

  it('reports every configured channel when cutting to the minimum', () => {
    expect(channelsLostByCountChange(stored, 0, fb).map(c => c.idx)).toEqual([0, 1, 3])
  })

  it('returns nothing when there is no stored channel array', () => {
    expect(channelsLostByCountChange(undefined, 6, fb)).toEqual([])
    expect(channelsLostByCountChange([], 6, fb)).toEqual([])
  })
})
