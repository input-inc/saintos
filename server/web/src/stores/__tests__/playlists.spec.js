// Playlists are the many-to-many replacement for the single `group`
// string that used to sit on each animation, pose and sound. The store
// is what the board view drives on every drag, so these pin the shapes
// the UI depends on:
//
//   * byKind keeps the three sidebar sections separate — an item can
//     only ever join a playlist of its own kind.
//   * itemsOf is the ORDER the main list renders in when a playlist is
//     selected. It is per-playlist, which is the whole point: the same
//     id sits at a different index in each list it belongs to.
//   * every mutation reloads, so the sidebar counts can't drift from
//     what the server actually stored.
import { describe, it, expect, beforeEach, vi } from 'vitest'
import { setActivePinia, createPinia } from 'pinia'

const management = vi.fn()
vi.mock('../ws', () => ({ useWsStore: () => ({ management }) }))

import { usePlaylistsStore } from '../playlists'

const GREETINGS = {
  id: 'anim_greetings', name: 'Greetings', kind: 'animations',
  items: ['wave', 'salute', 'bow'], count: 3, position: 1,
}
const IDLE = {
  id: 'anim_idle', name: 'Idle loops', kind: 'animations',
  items: ['bow', 'breathe'], count: 2, position: 2,
}
const ALERTS = {
  id: 'soun_alerts', name: 'Alerts', kind: 'sounds',
  items: ['beep'], count: 1, position: 1,
}
const ALL = [GREETINGS, IDLE, ALERTS]

/** Answer list_playlists with `rows`; everything else succeeds. */
function server (rows = ALL) {
  management.mockImplementation(async (action) => {
    if (action === 'list_playlists') return { playlists: rows }
    return { success: true }
  })
}

describe('playlists store', () => {
  let store
  beforeEach(() => {
    setActivePinia(createPinia())
    management.mockReset()
    server()
    store = usePlaylistsStore()
  })

  it('starts empty', () => {
    expect(store.list).toEqual([])
  })

  it('loads the playlists', async () => {
    await store.reload()
    expect(store.list).toHaveLength(3)
  })

  it('keeps the three sections separate', async () => {
    await store.reload()
    expect(store.byKind('animations').map(p => p.id))
      .toEqual(['anim_greetings', 'anim_idle'])
    expect(store.byKind('sounds').map(p => p.id)).toEqual(['soun_alerts'])
    expect(store.byKind('poses')).toEqual([])
  })

  it('exposes each playlist as its own ordering of members', async () => {
    await store.reload()
    // "bow" is last in Greetings and first in Idle -- one item, two
    // slots. A position field on the item could not express this.
    expect(store.itemsOf('anim_greetings').indexOf('bow')).toBe(2)
    expect(store.itemsOf('anim_idle').indexOf('bow')).toBe(0)
  })

  it('returns an empty order for an unknown playlist', async () => {
    await store.reload()
    expect(store.itemsOf('nope')).toEqual([])
    expect(store.get('nope')).toBeNull()
  })

  it('sends the right frame when adding an item at an index', async () => {
    await store.reload()
    await store.addItem('anim_idle', 'wave', 1)
    expect(management).toHaveBeenCalledWith('playlist_add_item', {
      playlist_id: 'anim_idle', item_id: 'wave', index: 1,
    })
  })

  it('omits the index when appending', async () => {
    await store.reload()
    await store.addItem('anim_idle', 'wave')
    expect(management).toHaveBeenCalledWith('playlist_add_item', {
      playlist_id: 'anim_idle', item_id: 'wave',
    })
  })

  it('reloads after a mutation so counts stay truthful', async () => {
    await store.reload()
    management.mockClear()
    await store.addItem('anim_idle', 'wave')
    expect(management.mock.calls.map(c => c[0]))
      .toEqual(['playlist_add_item', 'list_playlists'])
  })

  it('surfaces a rejected add instead of pretending it worked', async () => {
    await store.reload()
    management.mockImplementation(async (action) => {
      if (action === 'list_playlists') return { playlists: ALL }
      return { success: false, message: 'No animation with id "beep"' }
    })
    const ok = await store.addItem('anim_idle', 'beep')
    expect(ok).toBe(false)
    expect(store.error).toBe('No animation with id "beep"')
  })

  it('reports a transport failure rather than throwing at the caller', async () => {
    await store.reload()
    management.mockRejectedValue(new Error('socket closed'))
    const ok = await store.addItem('anim_idle', 'wave')
    expect(ok).toBe(false)
    expect(store.error).toBe('socket closed')
  })

  it('sends the new order when reordering members', async () => {
    await store.reload()
    await store.reorderItems('anim_greetings', ['bow', 'wave', 'salute'])
    expect(management).toHaveBeenCalledWith('reorder_playlist_items', {
      playlist_id: 'anim_greetings', ordered_ids: ['bow', 'wave', 'salute'],
    })
  })

  it('creates a playlist with no members', async () => {
    await store.create('Show opener', 'animations')
    expect(management).toHaveBeenCalledWith('save_playlist', {
      playlist: { id: '', name: 'Show opener', kind: 'animations', icon: '', items: [] },
    })
  })

  it('patches name without dropping the members', async () => {
    await store.reload()
    await store.patch('anim_greetings', { name: 'Hellos' })
    const sent = management.mock.calls
      .filter(c => c[0] === 'save_playlist').at(-1)[1].playlist
    expect(sent.name).toBe('Hellos')
    expect(sent.items).toEqual(['wave', 'salute', 'bow'])
  })

  it('does not patch an unknown playlist', async () => {
    await store.reload()
    management.mockClear()
    expect(await store.patch('nope', { name: 'x' })).toBeNull()
    expect(management).not.toHaveBeenCalled()
  })

  it('removes a member and a whole playlist through distinct actions', async () => {
    await store.reload()
    await store.removeItem('anim_greetings', 'bow')
    expect(management).toHaveBeenCalledWith('playlist_remove_item', {
      playlist_id: 'anim_greetings', item_id: 'bow',
    })
    await store.remove('anim_greetings')
    expect(management).toHaveBeenCalledWith('delete_playlist', { id: 'anim_greetings' })
  })

  it('reorders the sections themselves per kind', async () => {
    await store.reload()
    await store.reorder('animations', ['anim_idle', 'anim_greetings'])
    expect(management).toHaveBeenCalledWith('reorder_playlists', {
      kind: 'animations', ordered_ids: ['anim_idle', 'anim_greetings'],
    })
  })
})
