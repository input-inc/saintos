import { defineStore } from 'pinia'
import { computed, ref } from 'vue'
import { useWsStore } from './ws'

// Playlists: the many-to-many replacement for the single `group` string
// that used to live on each animation, pose and sound.
//
// A playlist holds one KIND of item ("animations" | "poses" | "sounds")
// and owns both its membership and its order. That is what lets the same
// animation sit third in "Greetings" and first in "Idle loops" — an
// ordering field on the item itself could only ever express one slot.
//
// Everything here writes through the server and reloads, matching the
// poses/sounds stores. Drag-and-drop fires a lot of these, so `move` and
// `reorderItems` are the two that carry the interaction cost; both are a
// single round trip.
export const usePlaylistsStore = defineStore('playlists', () => {
  const ws = useWsStore()
  const list = ref([])
  const loading = ref(false)
  const error = ref('')

  async function reload () {
    loading.value = true
    error.value = ''
    try {
      const r = await ws.management('list_playlists', {})
      list.value = r?.playlists || []
    } catch (e) {
      error.value = e.message || String(e)
    } finally {
      loading.value = false
    }
  }

  /** Playlists for one board section, already in sidebar order. */
  const byKind = computed(() => (kind) =>
    (list.value || []).filter(p => p.kind === kind))

  /** The playlist record for an id, or null. */
  function get (id) {
    return (list.value || []).find(p => p.id === id) || null
  }

  /** Ordered member ids of a playlist ([] if unknown). */
  function itemsOf (id) {
    return get(id)?.items || []
  }

  async function create (name, kind, icon = '') {
    error.value = ''
    try {
      const r = await ws.management('save_playlist', {
        playlist: { id: '', name, kind, icon, items: [] },
      })
      if (r?.success === false) {
        error.value = r.message || 'Create failed'
        return null
      }
      await reload()
      return r?.playlist || null
    } catch (e) {
      error.value = e.message || String(e)
      return null
    }
  }

  /** Patch name/icon on an existing playlist, keeping its members. */
  async function patch (id, fields) {
    const current = get(id)
    if (!current) return null
    error.value = ''
    try {
      const r = await ws.management('save_playlist', {
        playlist: { ...current, ...fields, id },
      })
      if (r?.success === false) {
        error.value = r.message || 'Save failed'
        return null
      }
      await reload()
      return r?.playlist || null
    } catch (e) {
      error.value = e.message || String(e)
      return null
    }
  }

  /** Delete the playlist. Its items are NOT deleted. */
  async function remove (id) {
    try {
      await ws.management('delete_playlist', { id })
      await reload()
    } catch (e) {
      error.value = e.message || String(e)
    }
  }

  /**
   * Put `itemId` into `playlistId` at `index` (append when null).
   * An item already in this playlist is MOVED, not duplicated — which
   * is what dragging a row within its own list means.
   */
  async function addItem (playlistId, itemId, index = null) {
    error.value = ''
    try {
      const r = await ws.management('playlist_add_item', {
        playlist_id: playlistId, item_id: itemId,
        ...(index === null ? {} : { index }),
      })
      if (r?.success === false) {
        error.value = r.message || 'Could not add to playlist'
        return false
      }
      await reload()
      return true
    } catch (e) {
      error.value = e.message || String(e)
      return false
    }
  }

  async function removeItem (playlistId, itemId) {
    error.value = ''
    try {
      await ws.management('playlist_remove_item', {
        playlist_id: playlistId, item_id: itemId,
      })
      await reload()
      return true
    } catch (e) {
      error.value = e.message || String(e)
      return false
    }
  }

  /** Reorder members within one playlist. */
  async function reorderItems (playlistId, orderedIds) {
    error.value = ''
    try {
      await ws.management('reorder_playlist_items', {
        playlist_id: playlistId, ordered_ids: orderedIds,
      })
      await reload()
    } catch (e) {
      error.value = e.message || String(e)
    }
  }

  /** Reorder the playlists themselves within a section. */
  async function reorder (kind, orderedIds) {
    error.value = ''
    try {
      await ws.management('reorder_playlists', { kind, ordered_ids: orderedIds })
      await reload()
    } catch (e) {
      error.value = e.message || String(e)
    }
  }

  return {
    list, loading, error, byKind,
    reload, get, itemsOf,
    create, patch, remove,
    addItem, removeItem, reorderItems, reorder,
  }
})
