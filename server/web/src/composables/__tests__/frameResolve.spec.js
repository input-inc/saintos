// Mirror of server/test/test_frame_resolve.py.
//
// The client resolves frames itself to drive the 3D viewport at 60 fps,
// so there are two implementations of one rule. These fixtures and
// assertions are deliberately the same as the Python ones: if the two
// resolvers drift, one of these suites fails instead of the rig quietly
// looking different in the viewport than it behaves on the robot.
//
// The property under test is that TRACK ORDER MATTERS — pose tracks
// layer over each other and over joint tracks in list order.

import { describe, expect, it } from 'vitest'
import {
  effectiveOverrideKeys,
  frameToPreviewValues,
  referencedPoseIds,
  relaxedFrame,
  resolveFrame,
  trackSpan,
} from '../useFrameResolve'

// Poses are normalized −1..+1, as stored.
const POSES = {
  happy: { brow_l: 0.8, brow_r: 0.8, mouth: 0.6 },
  mad: { brow_l: -0.6, brow_r: -0.6 },
  mouth_open: { mouth: 1.0 },
}
const poseLookup = (id) => POSES[id] || null

// Compact track specs, matching the Python `anim()` helper:
//   ['pose', 'happy', [[t, w], …]]
//   ['joint', 'brow_l', [[t, v], …]]
//   ['ws', ['sheet', 'input'], [[t, v], …]]
function anim (...tracks) {
  return {
    id: 'a',
    name: 'a',
    duration: 1,
    value_tracks: tracks.map(([kind, target, keys, overrides], i) => {
      const curve = {
        name: `c${i}`,
        keys: keys.map(([time, value]) => ({ time, value, interp: 1 })),
      }
      if (kind === 'pose') {
        const track = { id: `pose${i}`, name: target, target_kind: 'pose', target: [target], curve }
        if (overrides) {
          track.joint_overrides = Object.fromEntries(
            Object.entries(overrides).map(([joint, ks]) => [joint, {
              name: joint,
              keys: ks.map(([time, value]) => ({ time, value, interp: 1 })),
            }]))
        }
        return track
      }
      if (kind === 'joint') {
        return { id: target, name: target, target_kind: 'urdf_joint', target: [], curve }
      }
      return { id: `ws${i}`, name: 'ws', target_kind: 'ws_input', target, curve }
    }),
  }
}

const opts = { poseLookup }

describe('resolveFrame — joint and ws tracks', () => {
  it('sets a joint from its own curve', () => {
    const { joints, wsValues } = resolveFrame(
      anim(['joint', 'brow_l', [[0, 0], [1, 1]]]), 0.5, opts)
    expect(joints.brow_l).toBeCloseTo(0.5)
    expect(wsValues).toEqual([])
  })

  it('keeps ws-input tracks out of the joint bucket entirely', () => {
    const { joints, wsValues } = resolveFrame(
      anim(['ws', ['sheet1', 'in1'], [[0, 0], [1, 1]]]), 0.25, opts)
    expect(joints).toEqual({})
    expect(wsValues).toEqual([{ target: ['sheet1', 'in1'], value: 0.25 }])
  })

  it('treats a track without target_kind as a joint', () => {
    // Every track authored before target_kind existed.
    const a = {
      value_tracks: [{ id: 'brow_l', curve: { keys: [{ time: 0, value: 0.4, interp: 1 }] } }],
    }
    expect(resolveFrame(a, 0, opts).joints.brow_l).toBeCloseTo(0.4)
  })
})

describe('resolveFrame — pose tracks', () => {
  it('applies the whole pose at full weight', () => {
    const { joints } = resolveFrame(anim(['pose', 'happy', [[0, 1]]]), 0, opts)
    expect(joints.brow_l).toBeCloseTo(0.8)
    expect(joints.brow_r).toBeCloseTo(0.8)
    expect(joints.mouth).toBeCloseTo(0.6)
  })

  it('blends from neutral at half weight', () => {
    const { joints } = resolveFrame(anim(['pose', 'happy', [[0, 0.5]]]), 0, opts)
    expect(joints.brow_l).toBeCloseTo(0.4)
    expect(joints.mouth).toBeCloseTo(0.3)
  })

  it('contributes nothing at zero weight rather than zeroing joints', () => {
    // So a pose fading out doesn't fight the track below it.
    const { joints } = resolveFrame(anim(['pose', 'happy', [[0, 0]]]), 0, opts)
    expect(joints).toEqual({})
  })

  it('clamps the weight to 0..1', () => {
    // The curve is a weight, not a joint value.
    expect(resolveFrame(anim(['pose', 'happy', [[0, 3]]]), 0, opts).joints.brow_l)
      .toBeCloseTo(0.8)
    expect(resolveFrame(anim(['pose', 'happy', [[0, -2]]]), 0, opts).joints)
      .toEqual({})
  })

  it('blends up from the neutral base', () => {
    const { joints } = resolveFrame(anim(['pose', 'happy', [[0, 0.5]]]), 0,
      { poseLookup, neutral: { brow_l: 0.2, mouth: 0 } })
    expect(joints.brow_l).toBeCloseTo(0.5)   // 0.2 → 0.8 at 50%
    expect(joints.mouth).toBeCloseTo(0.3)
  })

  it('skips a deleted pose without losing the rest of the frame', () => {
    const { joints } = resolveFrame(
      anim(['pose', 'deleted_pose', [[0, 1]]], ['joint', 'brow_l', [[0, 0.25]]]),
      0, opts)
    expect(joints).toEqual({ brow_l: 0.25 })
  })

  it('contributes nothing with no pose lookup wired', () => {
    expect(resolveFrame(anim(['pose', 'happy', [[0, 1]]]), 0, {}).joints)
      .toEqual({})
  })
})

describe('resolveFrame — ORDER MATTERS', () => {
  it('lets a later pose track win on shared joints', () => {
    const { joints } = resolveFrame(
      anim(['pose', 'happy', [[0, 1]]], ['pose', 'mad', [[0, 1]]]), 0, opts)
    expect(joints.brow_l).toBeCloseTo(-0.6)   // mad, on top
    // happy also set `mouth`, which mad says nothing about — it survives.
    expect(joints.mouth).toBeCloseTo(0.6)
  })

  it('reverses the winner when the order reverses', () => {
    // The pin for reorder-ability: same tracks, same weights, different
    // output purely because of list order.
    const forward = anim(['pose', 'happy', [[0, 1]]], ['pose', 'mad', [[0, 1]]])
    const reverse = anim(['pose', 'mad', [[0, 1]]], ['pose', 'happy', [[0, 1]]])
    expect(resolveFrame(forward, 0, opts).joints.brow_l).toBeCloseTo(-0.6)
    expect(resolveFrame(reverse, 0, opts).joints.brow_l).toBeCloseTo(0.8)
  })

  it('blends a partial-weight layer from the layer below', () => {
    // happy fully applied, then mad at half weight: brow_l goes from 0.8
    // halfway toward -0.6, landing on 0.1.
    const { joints } = resolveFrame(
      anim(['pose', 'happy', [[0, 1]]], ['pose', 'mad', [[0, 0.5]]]), 0, opts)
    expect(joints.brow_l).toBeCloseTo(0.1)
  })

  it('lets a joint track above a pose override it', () => {
    const { joints } = resolveFrame(
      anim(['pose', 'happy', [[0, 1]]], ['joint', 'brow_l', [[0, -1]]]), 0, opts)
    expect(joints.brow_l).toBeCloseTo(-1)
    expect(joints.brow_r).toBeCloseTo(0.8)
  })

  it('lets a pose override a joint track below it', () => {
    const { joints } = resolveFrame(
      anim(['joint', 'brow_l', [[0, -1]]], ['pose', 'happy', [[0, 1]]]), 0, opts)
    expect(joints.brow_l).toBeCloseTo(0.8)
  })

  it('blends a pose from an explicitly keyed joint below it', () => {
    const { joints } = resolveFrame(
      anim(['joint', 'mouth', [[0, 0]]], ['pose', 'mouth_open', [[0, 0.5]]]),
      0, opts)
    expect(joints.mouth).toBeCloseTo(0.5)   // 0.0 → 1.0 at 50%
  })

  it('composes three layers bottom-up', () => {
    const { joints } = resolveFrame(
      anim(['pose', 'happy', [[0, 1]]],
        ['pose', 'mouth_open', [[0, 1]]],
        ['joint', 'brow_r', [[0, 0]]]),
      0, opts)
    expect(joints.brow_l).toBeCloseTo(0.8)   // happy only
    expect(joints.mouth).toBeCloseTo(1.0)    // mouth_open over happy
    expect(joints.brow_r).toBeCloseTo(0)     // joint track on top
  })
})

// Mirrors the "CLIP semantics" block in test_frame_resolve.py.
describe('resolveFrame — pose tracks are CLIPS', () => {
  it('contributes inside its span', () => {
    const a = anim(['pose', 'happy', [[1, 1], [2, 1]]])
    expect(resolveFrame(a, 1.5, opts).joints.brow_l).toBeCloseTo(0.8)
  })

  it('contributes nothing before its span', () => {
    const a = anim(['pose', 'happy', [[1, 1], [2, 1]]])
    expect(resolveFrame(a, 0.5, opts).joints).toEqual({})
  })

  it('contributes nothing after its span', () => {
    // The crux: a joint track's curve holds forever, a pose clip doesn't.
    const a = anim(['pose', 'happy', [[1, 1], [2, 1]]])
    expect(resolveFrame(a, 3, opts).joints).toEqual({})
  })

  it('treats span endpoints as inclusive', () => {
    const a = anim(['pose', 'happy', [[1, 1], [2, 1]]])
    expect(resolveFrame(a, 1, opts).joints.brow_l).toBeCloseTo(0.8)
    expect(resolveFrame(a, 2, opts).joints.brow_l).toBeCloseTo(0.8)
  })

  it('lets a later clip take over regardless of list order', () => {
    const a = anim(['pose', 'happy', [[0, 1], [2, 1]]],
      ['pose', 'mad', [[2, 1], [4, 1]]])
    expect(resolveFrame(a, 1, opts).joints.brow_l).toBeCloseTo(0.8)
    expect(resolveFrame(a, 3, opts).joints.brow_l).toBeCloseTo(-0.6)
  })

  it('hands off the same way with the list order reversed', () => {
    const a = anim(['pose', 'mad', [[2, 1], [4, 1]]],
      ['pose', 'happy', [[0, 1], [2, 1]]])
    expect(resolveFrame(a, 1, opts).joints.brow_l).toBeCloseTo(0.8)
    expect(resolveFrame(a, 3, opts).joints.brow_l).toBeCloseTo(-0.6)
  })

  it('still uses list order where spans overlap', () => {
    // t=2 is inside BOTH spans, so position breaks the tie. This is the
    // one thing reordering is still for.
    const forward = anim(['pose', 'happy', [[0, 1], [3, 1]]],
      ['pose', 'mad', [[1, 1], [4, 1]]])
    const reverse = anim(['pose', 'mad', [[1, 1], [4, 1]]],
      ['pose', 'happy', [[0, 1], [3, 1]]])
    expect(resolveFrame(forward, 2, opts).joints.brow_l).toBeCloseTo(-0.6)
    expect(resolveFrame(reverse, 2, opts).joints.brow_l).toBeCloseTo(0.8)
  })

  it('reports a single-key pose as a zero-length span', () => {
    // Why "+ Pose" creates two keys and the timeline draws the span.
    const a = anim(['pose', 'happy', [[1, 1]]])
    expect(trackSpan(a.value_tracks[0])).toEqual([1, 1])
    expect(resolveFrame(a, 1, opts).joints.brow_l).toBeCloseTo(0.8)
    expect(resolveFrame(a, 1.1, opts).joints).toEqual({})
  })

  it('gives a keyless track no span', () => {
    const a = anim(['pose', 'happy', []])
    expect(trackSpan(a.value_tracks[0])).toBeNull()
    expect(resolveFrame(a, 0, opts).joints).toEqual({})
  })

  it('leaves joint tracks extrapolating outside their keys', () => {
    // Clip semantics are for POSE tracks only, or every existing
    // animation changes behaviour.
    const a = anim(['joint', 'brow_l', [[1, 0.5]]])
    expect(resolveFrame(a, 0, opts).joints.brow_l).toBeCloseTo(0.5)
    expect(resolveFrame(a, 3, opts).joints.brow_l).toBeCloseTo(0.5)
  })
})

// Mirrors the "per-joint overrides" block in test_frame_resolve.py.
describe('resolveFrame — per-joint overrides', () => {
  it('replaces only the joint it names', () => {
    const a = anim(['pose', 'happy', [[0, 1], [2, 1]], { mouth: [[1, -0.9]] }])
    const { joints } = resolveFrame(a, 1, opts)
    expect(joints.mouth).toBeCloseTo(-0.9)
    expect(joints.brow_l).toBeCloseTo(0.8)
  })

  it('is anchored to the pose at the parent keyframes', () => {
    // Or the locked anchors the UI draws would be a lie.
    const a = anim(['pose', 'happy', [[0, 1], [2, 1]], { mouth: [[1, -0.9]] }])
    expect(resolveFrame(a, 0, opts).joints.mouth).toBeCloseTo(0.6)
    expect(resolveFrame(a, 2, opts).joints.mouth).toBeCloseTo(0.6)
  })

  it('interpolates between an anchor and a user key', () => {
    const a = anim(['pose', 'happy', [[0, 1], [2, 1]], { mouth: [[1, 0]] }])
    expect(resolveFrame(a, 0.5, opts).joints.mouth).toBeCloseTo(0.3)
  })

  it('anchors follow the weight curve', () => {
    const a = anim(['pose', 'happy', [[0, 0.5], [2, 1]], { mouth: [[1, 0]] }])
    expect(resolveFrame(a, 0, opts).joints.mouth).toBeCloseTo(0.3)
  })

  it('applies even at zero weight, but stays clip-bounded', () => {
    const a = anim(['pose', 'happy', [[0, 0], [2, 0]], { mouth: [[1, 0.75]] }])
    expect(resolveFrame(a, 1, opts).joints.mouth).toBeCloseTo(0.75)
    expect(resolveFrame(a, 3, opts).joints.mouth).toBeUndefined()
  })

  it('ignores an override for a joint the pose does not name', () => {
    const a = anim(['pose', 'mad', [[0, 1], [2, 1]], { mouth: [[1, 0.5]] }])
    expect(resolveFrame(a, 1, opts).joints.mouth).toBeUndefined()
  })

  it('lets a user key at an anchor time win', () => {
    const a = anim(['pose', 'happy', [[0, 1], [2, 1]], { mouth: [[0, -1]] }])
    expect(resolveFrame(a, 0, opts).joints.mouth).toBeCloseTo(-1)
  })

  it('merges anchors and user keys in time order', () => {
    const a = anim(['pose', 'happy', [[0, 1], [2, 1]],
      { mouth: [[0.5, 0.1], [1.5, 0.2]] }])
    const keys = effectiveOverrideKeys(a.value_tracks[0], 'mouth', 0.6, 0)
    expect(keys.map(k => k.time)).toEqual([0, 0.5, 1.5, 2])
    expect(keys.map(k => Number(k.value.toFixed(3)))).toEqual([0.6, 0.1, 0.2, 0.6])
    // The UI relies on this flag to know which keys it must not move.
    expect(keys.map(k => !!k.locked)).toEqual([true, false, false, true])
  })
})

describe('relaxedFrame', () => {
  it('zeroes joints a pose was moving', () => {
    // Resolving at zero weight would strand them, because a pose at
    // weight 0 contributes nothing.
    const { joints, wsValues } = relaxedFrame(
      anim(['pose', 'happy', [[0, 1]]],
        ['joint', 'neck', [[0, 0.5]]],
        ['ws', ['s', 'i'], [[0, 0.9]]]),
      opts)
    expect(joints).toEqual({ brow_l: 0, brow_r: 0, mouth: 0, neck: 0 })
    expect(wsValues).toEqual([{ target: ['s', 'i'], value: 0 }])
  })

  it('settles to the neutral pose when there is one', () => {
    const { joints } = relaxedFrame(anim(['pose', 'happy', [[0, 1]]]),
      { poseLookup, neutral: { brow_l: 0.2 } })
    expect(joints.brow_l).toBeCloseTo(0.2)
    expect(joints.brow_r).toBeCloseTo(0)
  })
})

describe('introspection and wire shape', () => {
  it('lists referenced pose ids in order without duplicates', () => {
    expect(referencedPoseIds(anim(
      ['pose', 'happy', [[0, 1]]], ['joint', 'x', [[0, 1]]],
      ['pose', 'mad', [[0, 1]]], ['pose', 'happy', [[0, 1]]],
    ))).toEqual(['happy', 'mad'])
  })

  it('falls back to the track id for a pose id', () => {
    const a = {
      value_tracks: [{
        id: 'happy', target_kind: 'pose', target: [],
        curve: { keys: [{ time: 0, value: 1, interp: 1 }] },
      }],
    }
    expect(resolveFrame(a, 0, opts).joints.brow_l).toBeCloseTo(0.8)
  })

  it('resolves an empty animation to nothing', () => {
    expect(resolveFrame(anim(), 0, opts)).toEqual({ joints: {}, wsValues: [] })
  })

  it('flattens a frame into preview wire values', () => {
    // Pose tracks are already resolved to joints here, so the server
    // never needs a "pose" kind on its preview path.
    const frame = resolveFrame(
      anim(['pose', 'mouth_open', [[0, 1]]], ['ws', ['s', 'i'], [[0, 0.5]]]),
      0, opts)
    expect(frameToPreviewValues(frame)).toEqual([
      { target_kind: 'urdf_joint', id: 'mouth', value: 1 },
      { target_kind: 'ws_input', target: ['s', 'i'], value: 0.5 },
    ])
  })
})

// ─── neutral is the base a pose blends up from ───────────────────────
//
// The server player resolves with `neutral_source=rig_neutral`. The
// editor's Live Preview used to resolve with no neutral at all, so a
// pose track at any weight below 1 previewed a different joint value
// than it played. These pin the parameter that closes that gap; the
// mirror assertions live in server/test/test_frame_resolve.py.

describe('neutral as the pose base', () => {
  const poseTrack = {
    id: 'p1', name: 'happy', target_kind: 'pose', target: ['happy'],
    curve: { keys: [
      { time: 0, value: 0.5, interp: 1 },
      { time: 1, value: 0.5, interp: 1 },
    ] },
  }
  const anim = { duration: 1, value_tracks: [poseTrack] }
  const poseLookup = (id) => POSES[id] || null

  it('blends from zero when no neutral is given', () => {
    const { joints } = resolveFrame(anim, 0.5, { poseLookup })
    // 0 + (0.8 - 0) * 0.5
    expect(joints.brow_l).toBeCloseTo(0.4, 6)
  })

  it('blends from the neutral pose when one is given', () => {
    const neutral = { brow_l: 0.2 }
    const { joints } = resolveFrame(anim, 0.5, { poseLookup, neutral })
    // 0.2 + (0.8 - 0.2) * 0.5 -- NOT 0.4
    expect(joints.brow_l).toBeCloseTo(0.5, 6)
  })

  it('leaves joints the neutral does not name at zero', () => {
    const { joints } = resolveFrame(anim, 0.5, {
      poseLookup, neutral: { brow_l: 0.2 },
    })
    expect(joints.mouth).toBeCloseTo(0.3, 6)
  })

  it('agrees with a full-weight pose regardless of neutral', () => {
    // At weight 1 the base cancels out, which is why the gap only ever
    // showed on partially-weighted poses.
    const full = { ...anim, value_tracks: [{
      ...poseTrack,
      curve: { keys: [{ time: 0, value: 1, interp: 1 }, { time: 1, value: 1, interp: 1 }] },
    }] }
    const a = resolveFrame(full, 0.5, { poseLookup }).joints
    const b = resolveFrame(full, 0.5, { poseLookup, neutral: { brow_l: 0.2 } }).joints
    expect(a.brow_l).toBeCloseTo(b.brow_l, 6)
  })

  it('relaxes to neutral, not to zero', () => {
    const values = frameToPreviewValues(
      relaxedFrame(anim, { poseLookup, neutral: { brow_l: 0.2 } }))
    const brow = values.find(v => v.id === 'brow_l')
    expect(brow.value).toBeCloseTo(0.2, 6)
  })

  it('relaxes to zero when there is no neutral', () => {
    const values = frameToPreviewValues(relaxedFrame(anim, { poseLookup }))
    expect(values.find(v => v.id === 'brow_l').value).toBe(0)
  })
})
