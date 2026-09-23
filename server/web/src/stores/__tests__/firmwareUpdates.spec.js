// The firmwareUpdates store is what makes an OTA visible. Every place
// that shows update messaging — the Nodes cards, the node Overview's
// firmware block — reads this one store, so "is this node updating"
// has a single answer and the UI cannot contradict itself.
//
// These pin the two behaviours that were wrong:
//   * a bulk update left the cards blank, because only the per-node
//     modal seeded a progress entry; the fleet-wide path did not, and
//     bootloader-driven nodes (RP2040 / Teensy) may never publish a
//     progress frame at all, so nothing ever appeared.
//   * `isUpdating` has to be a plain predicate: `isActive` returns a
//     fresh computed per call, which allocates one per node per render
//     when used from a v-for template.
import { describe, it, expect, beforeEach, vi } from 'vitest'
import { setActivePinia, createPinia } from 'pinia'

vi.mock('../ws', () => ({
  useWsStore: () => ({ subscribe: vi.fn().mockResolvedValue(undefined), on: vi.fn() }),
}))
vi.mock('../nodes', () => ({
  useNodesStore: () => ({ all: [{ node_id: 'n1', firmware_version: '1.0.0' }] }),
}))

import { useFirmwareUpdatesStore } from '../firmwareUpdates'

describe('firmwareUpdates store', () => {
  let store
  beforeEach(() => {
    setActivePinia(createPinia())
    store = useFirmwareUpdatesStore()
  })

  it('reports nothing updating before a start', () => {
    expect(store.isUpdating('n1')).toBe(false)
  })

  it('start() seeds progress before any frame arrives', async () => {
    // The whole point: bootloader targets may never publish a frame,
    // so the seed is the only thing the operator sees.
    await store.start('n1')
    expect(store.isUpdating('n1')).toBe(true)
    expect(store.entries.n1.stage).toBe('starting')
    expect(store.entries.n1.status).toBe('in_progress')
  })

  it('captures the starting version so completion can be detected', async () => {
    await store.start('n1')
    expect(store.entries.n1.startingVersion).toBe('1.0.0')
  })

  it('cancel() clears a seed, for a request that never reached the server', async () => {
    await store.start('n1')
    store.cancel('n1')
    expect(store.isUpdating('n1')).toBe(false)
  })

  it('a terminal status is not "updating"', async () => {
    await store.start('n1')
    store.entries.n1.status = 'complete'
    expect(store.isUpdating('n1')).toBe(false)
    store.entries.n1.status = 'failed'
    expect(store.isUpdating('n1')).toBe(false)
  })

  it('isUpdating allocates nothing — it is a predicate, not a computed', async () => {
    await store.start('n1')
    const a = store.isUpdating('n1')
    const b = store.isUpdating('n1')
    expect(a).toBe(true)
    expect(b).toBe(true)
    // A computed factory would return two distinct ref objects here.
    expect(typeof a).toBe('boolean')
  })

  it('tracks nodes independently', async () => {
    await store.start('n1')
    expect(store.isUpdating('n2')).toBe(false)
  })
})
