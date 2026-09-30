// Disclosing a pose track lists the targets that pose drives.
//
// It used to list only URDF-JOINT setpoints. Poses authored in the
// dashboard's pose editor are made of routing-sheet (ws_input)
// setpoints — PoseLibrary.addSetpoint creates sheet_id/ws_input_id
// pairs, and PoseSetpoint.target_kind defaults to 'ws_input' — so every
// such pose disclosed nothing at all. The pose still applied correctly;
// only the disclosure looked broken, which is the worst way for it to
// fail.
import { describe, it, expect, vi } from 'vitest'
import { mount } from '@vue/test-utils'

vi.mock('@/stores/animations', () => ({
  useAnimationsStore: () => ({ snapshot: vi.fn(), markDirty: vi.fn() }),
}))

import TimelineEditor from '@/components/animation/TimelineEditor.vue'

const track = (id, poseId) => ({
  id, name: poseId, target_kind: 'pose', target: [poseId],
  curve: { name: poseId, keys: [
    { time: 0, value: 0, interp: 1 }, { time: 1, value: 1, interp: 1 }] },
})

function mountWith (props) {
  return mount(TimelineEditor, {
    props: {
      animation: { id: 'a', name: 'A', duration: 2, fps: 60,
                   value_tracks: [track('t1', 'happy')], trigger_tracks: [] },
      playerPos: 0.5,
      selection: { kind: null },
      poses: [], wsInputs: [], unboundJoints: [],
      poseJoints: {}, resolvedJoints: {},
      ...props,
    },
  })
}

async function expand (w) {
  const btn = w.findAll('button')
    .find(b => /arrow_right|arrow_drop_down/.test(b.text()))
  expect(btn, 'pose tracks must have a disclosure button').toBeTruthy()
  await btn.trigger('click')
  await w.vm.$nextTick()
  return w.findAll('.pose-child-row')
}

describe('pose track disclosure', () => {
  it('lists ws_input setpoints — the bug', async () => {
    const w = mountWith({
      poseJoints: { happy: {} },       // no joints: a UI-authored pose
      poseSetpoints: { happy: [
        { key: 'ws:face/brow', label: 'face/brow', joint: '', kind: 'ws_input', value: 0.8 },
        { key: 'ws:face/jaw', label: 'face/jaw', joint: '', kind: 'ws_input', value: 0.4 },
      ] },
    })
    const rows = await expand(w)
    expect(rows).toHaveLength(2)
    expect(w.text()).toContain('face/brow')
    expect(w.text()).toContain('face/jaw')
  })

  it('still lists joint setpoints', async () => {
    const w = mountWith({
      poseJoints: { happy: { brow_l: 0.8, mouth: 0.6 } },
      poseSetpoints: { happy: [
        { key: 'joint:brow_l', label: 'brow_l', joint: 'brow_l', kind: 'joint', value: 0.8 },
        { key: 'joint:mouth', label: 'mouth', joint: 'mouth', kind: 'joint', value: 0.6 },
      ] },
      resolvedJoints: { brow_l: 0.4, mouth: 0.3 },
    })
    const rows = await expand(w)
    expect(rows).toHaveLength(2)
    expect(w.text()).toContain('brow_l')
  })

  it('lists a pose that mixes both kinds', async () => {
    const w = mountWith({
      poseJoints: { happy: { brow_l: 0.8 } },
      poseSetpoints: { happy: [
        { key: 'joint:brow_l', label: 'brow_l', joint: 'brow_l', kind: 'joint', value: 0.8 },
        { key: 'ws:face/jaw', label: 'face/jaw', joint: '', kind: 'ws_input', value: 0.4 },
      ] },
      resolvedJoints: { brow_l: 0.4 },
    })
    expect(await expand(w)).toHaveLength(2)
  })

  it('shows the live value only for joints', async () => {
    // A ws_input setpoint isn't part of the joint frame, so there is no
    // resolved value to show beside its target.
    const w = mountWith({
      poseJoints: { happy: { brow_l: 0.8 } },
      poseSetpoints: { happy: [
        { key: 'joint:brow_l', label: 'brow_l', joint: 'brow_l', kind: 'joint', value: 0.8 },
        { key: 'ws:face/jaw', label: 'face/jaw', joint: '', kind: 'ws_input', value: 0.4 },
      ] },
      resolvedJoints: { brow_l: 0.4 },
    })
    await expand(w)
    expect(w.text()).toContain('0.80 → 0.40')   // joint: target → live
    expect(w.text()).toContain('0.40')          // ws: target alone
  })

  it('falls back to the joint map when no setpoint list is supplied', async () => {
    const w = mountWith({ poseJoints: { happy: { brow_l: 0.8, mouth: 0.6 } } })
    expect(await expand(w)).toHaveLength(2)
  })

  it('discloses nothing for a pose that no longer exists', async () => {
    const w = mountWith({ poseJoints: { happy: null }, poseSetpoints: { happy: null } })
    expect(await expand(w)).toHaveLength(0)
  })
})
