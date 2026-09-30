// Board-item trigger keyframes on the timeline.
//
// A sound or nested animation has a real LENGTH, so it draws as a bar
// rather than a diamond: an operator placing a 4-second clip needs to
// see what it overlaps. The bar is not editable — it is the clip's own
// length — except for a LOOPING item, whose right edge drags to set how
// long it repeats. Cutting a one-shot short is a different feature and
// is deliberately not offered.
//
// Poses are absent here on purpose: a pose is a weighted value track
// with blending and per-joint overrides, which a one-shot fire cannot
// express. The + Board Item menu presents all three together anyway.
import { describe, it, expect, vi } from 'vitest'
import { mount } from '@vue/test-utils'

vi.mock('@/stores/animations', () => ({
  useAnimationsStore: () => ({ snapshot: vi.fn(), markDirty: vi.fn() }),
}))

import TimelineEditor from '@/components/animation/TimelineEditor.vue'

const SOUNDS = [
  { id: 'fanfare', name: 'Fanfare', duration: 3.0, loop: false },
  { id: 'siren', name: 'Siren', duration: 2.0, loop: true },
  { id: 'unmeasured', name: 'Unmeasured', duration: 0, loop: false },
]
const ANIMATIONS = [
  { id: 'wave', name: 'Wave hello', duration: 1.5, loop: false },
  { id: 'idle', name: 'Idle', duration: 4.0, loop: true },
]

function mountWith (keyframes, extra = {}) {
  return mount(TimelineEditor, {
    props: {
      animation: {
        id: 'a', name: 'A', duration: 10, fps: 60,
        value_tracks: [],
        trigger_tracks: [{ id: 'trig1', name: 'Board', keyframes }],
      },
      playerPos: 0,
      selection: { kind: null },
      poses: [], wsInputs: [], unboundJoints: [],
      poseJoints: {}, resolvedJoints: {},
      sounds: SOUNDS, animations: ANIMATIONS,
      ...extra,
    },
  })
}

const kf = (kind, id, time = 1, duration = 0) => ({
  time, target_kind: kind, target: [id], value: null, label: '', duration,
})

describe('board-item bars', () => {
  it('draws a sound as a bar the length of the clip', () => {
    const w = mountWith([kf('sound', 'fanfare', 1)])
    const bar = w.find('.board-item-bar')
    expect(bar.exists()).toBe(true)
    // 1s start, 3s long, on a 10s timeline.
    expect(bar.attributes('style')).toContain('left: 10%')
    expect(bar.attributes('style')).toContain('width: 30%')
  })

  it('draws an animation as a bar too', () => {
    const w = mountWith([kf('animation', 'wave', 2)])
    const bar = w.find('.board-item-bar')
    expect(bar.attributes('style')).toContain('left: 20%')
    expect(bar.attributes('style')).toContain('width: 15%')
  })

  it('tints sounds and animations differently', () => {
    const w = mountWith([kf('sound', 'fanfare'), kf('animation', 'wave', 5)])
    const classes = w.findAll('.board-item-bar').map(b => b.classes().join(' '))
    expect(classes.some(c => c.includes('kind-sound'))).toBe(true)
    expect(classes.some(c => c.includes('kind-animation'))).toBe(true)
  })

  it('names the item on the bar', () => {
    const w = mountWith([kf('sound', 'fanfare')])
    expect(w.find('.board-item-bar').text()).toContain('Fanfare')
  })

  it('falls back to a marker when the length is not measured', () => {
    // 0 means unknown, never zero-length — a zero-width bar would be
    // invisible and unclickable.
    const w = mountWith([kf('sound', 'unmeasured')])
    expect(w.find('.board-item-bar').exists()).toBe(false)
    expect(w.find('.trigger-keyframe').exists()).toBe(true)
  })

  it('falls back to a marker for an item the library no longer has', () => {
    const w = mountWith([kf('sound', 'deleted')])
    expect(w.find('.board-item-bar').exists()).toBe(false)
    expect(w.find('.trigger-keyframe').exists()).toBe(true)
  })

  it('leaves a plain trigger as a diamond', () => {
    const w = mountWith([
      { time: 1, target_kind: 'ws_input', target: ['sheet', 'in'], value: 1 },
    ])
    expect(w.find('.board-item-bar').exists()).toBe(false)
    expect(w.find('.trigger-keyframe').exists()).toBe(true)
  })
})

describe('loop handles', () => {
  it('offers a drag handle only on a looping item', () => {
    const looping = mountWith([kf('sound', 'siren')])
    expect(looping.find('.board-item-loop-handle').exists()).toBe(true)

    const oneShot = mountWith([kf('sound', 'fanfare')])
    expect(oneShot.find('.board-item-loop-handle').exists()).toBe(false)
  })

  it('uses the dragged length for a looping item', () => {
    // 6s dragged over a 2s clip, on a 10s timeline.
    const w = mountWith([kf('sound', 'siren', 1, 6)])
    expect(w.find('.board-item-bar').attributes('style')).toContain('width: 60%')
  })

  it('ignores a dragged length on a one-shot', () => {
    // Its length is the clip's own; honouring `duration` here would be
    // truncation, which is a different feature.
    const w = mountWith([kf('sound', 'fanfare', 1, 0.5)])
    expect(w.find('.board-item-bar').attributes('style')).toContain('width: 30%')
  })

  it('says in the tooltip which bars can be dragged', () => {
    const looping = mountWith([kf('animation', 'idle', 0, 5)])
    expect(looping.find('.board-item-bar').attributes('title'))
      .toContain('drag the right edge')

    const oneShot = mountWith([kf('animation', 'wave')])
    expect(oneShot.find('.board-item-bar').attributes('title'))
      .toContain('fixed by the clip')
  })

  it('tells the operator when a length is still unmeasured', () => {
    const w = mountWith([kf('sound', 'unmeasured')])
    expect(w.find('.trigger-keyframe').attributes('title'))
      .toContain('not measured yet')
  })
})

describe('+ Board Item menu', () => {
  const POSES = [{ id: 'happy', name: 'Happy', playlists: ['pl_face'], joint_count: 3 }]

  function openMenu (extra = {}) {
    const w = mountWith([], { poses: POSES, posePlaylists: { pl_face: 'Face' }, ...extra })
    const btn = w.findAll('button').find(b => b.text().includes('Board Item'))
    expect(btn, 'the + Board Item button must exist').toBeTruthy()
    return btn.trigger('click').then(() => w)
  }

  it('replaces the separate + Pose button', async () => {
    const w = await openMenu()
    const labels = w.findAll('button').map(b => b.text())
    expect(labels.some(t => t.includes('Board Item'))).toBe(true)
    expect(labels.some(t => t.trim() === 'add\nPose')).toBe(false)
  })

  it('keeps the raw + Trigger button', async () => {
    // A ROS topic field or peripheral command isn't on any board, and
    // existing animations use those.
    const w = await openMenu()
    expect(w.findAll('button').some(b => b.text().includes('Trigger'))).toBe(true)
  })

  it('offers all three boards', async () => {
    const w = await openMenu()
    const text = w.text()
    expect(text).toContain('Poses')
    expect(text).toContain('Sounds')
    expect(text).toContain('Animations')
  })

  it('shows lengths for sounds but not poses', async () => {
    const w = await openMenu()
    // Poses board is the default: no length column.
    expect(w.text()).toContain('Happy')
    const soundsTab = w.findAll('button').find(b => b.text().includes('Sounds'))
    await soundsTab.trigger('click')
    expect(w.text()).toContain('3.00s')
  })

  it('badges a looping item', async () => {
    const w = await openMenu()
    await w.findAll('button').find(b => b.text().includes('Sounds')).trigger('click')
    expect(w.text()).toContain('loop')
  })

  it('emits add-pose for a pose and add-board-item for a sound', async () => {
    const w = await openMenu()
    await w.findAll('button').find(b => b.text().includes('Happy')).trigger('click')
    expect(w.emitted('add-pose')).toBeTruthy()
    expect(w.emitted('add-board-item')).toBeFalsy()

    const w2 = await openMenu()
    await w2.findAll('button').find(b => b.text().includes('Sounds')).trigger('click')
    await w2.findAll('button').find(b => b.text().includes('Fanfare')).trigger('click')
    expect(w2.emitted('add-board-item')[0][0])
      .toMatchObject({ kind: 'sound', id: 'fanfare' })
  })
})
