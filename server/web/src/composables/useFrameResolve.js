// Resolve an animation's value tracks into one frame of setpoints.
//
// Mirror of server/saint_server/animation/frame.py. The client needs its
// own copy because it drives the 3D viewport at 60 fps and can't round
// trip to the server for every frame — but the two MUST agree, or the
// rig looks different in the viewport than it behaves on the robot.
// server/test/test_frame_resolve.py and ./__tests__/frameResolve.spec.js
// share one fixture table so a divergence fails a test instead.
//
// TRACK ORDER MATTERS. Value tracks resolve in list order, each layering
// over what came before:
//
//   * a `pose` track lerps the joints its pose names from the
//     accumulated value toward the pose's value, by the track's weight;
//   * a `urdf_joint` track hard-sets its one joint;
//   * a `ws_input` track writes to a separate bucket and never touches
//     joints at all.
//
// POSE TRACKS ARE CLIPS, not infinite layers. A pose track contributes
// only between its first and last keyframe; outside that span it
// contributes nothing, rather than holding its last value forever the way
// a joint track's curve does. That's what makes handing off between poses
// automatic — give "happy" 0-2 s and "mad" 2-4 s and mad takes over
// because happy's clip ENDED, not because it out-ranks it. Order only
// decides where two spans actually overlap.
//
// A pose track may also carry `joint_overrides[joint]` — a curve of
// absolute joint values that refines one joint inside the clip, anchored
// to the pose's own keyframe times and free in between.
//
// This is NOT how rig controls compose. Controls are simultaneous, so
// they sum as commutative deltas from neutral and reordering the sliders
// changes nothing. Two different problems that both look like "blending
// poses":
//
//   pose tracks   sequential in time   order matters   clip / override
//   rig controls  simultaneous         order is moot   additive deltas

import { sampleCurve } from './useCurveSampling'

// Tolerance for "is this user key at the same time as an anchor". One
// millisecond: finer than any keyframe an operator can place, coarse
// enough that float drift through a JSON round-trip can't split an
// anchor into two keys a hair apart.
const TIME_EPS = 1e-3

/**
 * `[firstKeyTime, lastKeyTime]` for a track, or null when it has no keys.
 * The clip extent of a pose track. A single key gives a zero-length span,
 * which is a real (if useless) state the timeline surfaces rather than
 * silently widening.
 */
export function trackSpan (track) {
  const keys = track?.curve?.keys || []
  if (!keys.length) return null
  let lo = Infinity, hi = -Infinity
  for (const k of keys) {
    const t = Number(k.time) || 0
    if (t < lo) lo = t
    if (t > hi) hi = t
  }
  return [lo, hi]
}

function inSpan (track, time) {
  const span = trackSpan(track)
  if (!span) return false
  return time >= span[0] - TIME_EPS && time <= span[1] + TIME_EPS
}

function overrideCurve (track, joint) {
  const c = track?.joint_overrides?.[joint]
  return c && (c.keys || []).length ? c : null
}

/**
 * Locked anchor keys for a joint's override curve — one per keyframe on
 * the pose track itself, valued at the pose's own contribution there
 * (`neutral + (target − neutral) · weight(T)`).
 *
 * Deliberately independent of the other tracks: an anchor whose value
 * depended on the surrounding layers would move when an unrelated track
 * was reordered, and the operator would have no way to reason about it.
 *
 * Exported because the timeline draws these as the locked keyframes.
 */
export function overrideAnchors (track, target, neutralValue) {
  return (track?.curve?.keys || []).map((k) => {
    const w = Math.max(0, Math.min(1, sampleCurve(track.curve, k.time)))
    return {
      time: Number(k.time) || 0,
      value: neutralValue + (target - neutralValue) * w,
      interp: k.interp ?? 1,
      locked: true,
    }
  })
}

/**
 * Anchors merged with the operator's keys, sorted by time. A user key at
 * (within epsilon of) an anchor time wins.
 */
export function effectiveOverrideKeys (track, joint, target, neutralValue) {
  const anchors = overrideAnchors(track, target, neutralValue)
  const curve = overrideCurve(track, joint)
  const user = (curve?.keys || []).map(k => ({ ...k, locked: false }))
  const merged = user.slice()
  for (const a of anchors) {
    if (!user.some(u => Math.abs((Number(u.time) || 0) - a.time) <= TIME_EPS)) {
      merged.push(a)
    }
  }
  merged.sort((x, y) => (Number(x.time) || 0) - (Number(y.time) || 0))
  return merged
}

// The canonical pose id for a pose track. Falls back to the track id so
// a hand-written or migrated track that put the pose id there still
// resolves rather than silently doing nothing.
export function poseIdOf (track) {
  const target = track?.target || []
  if (target.length && target[0]) return String(target[0])
  return String(track?.id || '')
}

/**
 * Sample every value track at `time` and layer them in order.
 *
 * @param {object} animation
 * @param {number} time
 * @param {object} opts
 * @param {(poseId: string) => (object|null)} opts.poseLookup
 *        pose id → { joint: normalized −1..+1 }
 * @param {object} opts.neutral
 *        Base a pose blends up from on joints nothing has touched yet —
 *        the rig's neutral pose, or {} for implicit zeros (the midpoint
 *        of each joint's travel).
 * @returns {{ joints: object, wsValues: Array }}
 *        `wsValues` entries are { target: [sheetId, inputId], value }.
 */
export function resolveFrame (animation, time, opts = {}) {
  const poseLookup = opts.poseLookup || (() => null)
  const neutral = opts.neutral || {}
  const joints = {}
  const wsValues = []

  for (const track of animation?.value_tracks || []) {
    const kind = track.target_kind || 'urdf_joint'
    const value = sampleCurve(track.curve, time)

    if (kind === 'ws_input') {
      const target = track.target || []
      if (target.length >= 2) {
        wsValues.push({ target: [target[0], target[1]], value })
      }
      continue
    }

    if (kind === 'pose') {
      const poseId = poseIdOf(track)
      if (!poseId) continue
      // CLIP semantics: outside its keyframe span a pose track
      // contributes nothing. This is what lets a later pose take over
      // from an earlier one automatically — the earlier clip has ended,
      // rather than holding forever and having to be out-ranked.
      if (!inSpan(track, time)) continue
      const poseJoints = poseLookup(poseId)
      if (!poseJoints) continue
      // Weight, not a joint value: a pose track's curve says "how much
      // of this pose", so it clamps to 0..1 whatever the operator drew.
      const weight = Math.max(0, Math.min(1, value))
      for (const [joint, targetValue] of Object.entries(poseJoints)) {
        const base = joint in neutral ? neutral[joint] : 0
        if (overrideCurve(track, joint)) {
          // An override REPLACES the layered value for this joint inside
          // the clip — refine one joint without unpicking the pose that
          // drives the rest. Not gated on weight: the override drives
          // the joint directly.
          joints[joint] = sampleCurve(
            { keys: effectiveOverrideKeys(track, joint, targetValue, base) },
            time)
          continue
        }
        if (weight <= 0) continue
        const current = joint in joints ? joints[joint] : base
        joints[joint] = current + (targetValue - current) * weight
      }
      continue
    }

    // urdf_joint (default) — the track id IS the joint name.
    joints[track.id] = value
  }

  return { joints, wsValues }
}

/**
 * The frame to apply when playback stops or Live Preview turns off.
 *
 * Every target the animation can touch, driven back to neutral. NOT the
 * same as resolving at zero weight: a pose track at weight 0 contributes
 * nothing, which would strand the joints it had been moving wherever the
 * last frame left them.
 */
export function relaxedFrame (animation, opts = {}) {
  const poseLookup = opts.poseLookup || (() => null)
  const neutral = opts.neutral || {}
  const joints = {}
  const wsValues = []

  for (const track of animation?.value_tracks || []) {
    const kind = track.target_kind || 'urdf_joint'
    if (kind === 'ws_input') {
      const target = track.target || []
      if (target.length >= 2) {
        wsValues.push({ target: [target[0], target[1]], value: 0 })
      }
    } else if (kind === 'pose') {
      const poseJoints = poseLookup(poseIdOf(track))
      if (!poseJoints) continue
      for (const joint of Object.keys(poseJoints)) {
        joints[joint] = neutral[joint] ?? 0
      }
    } else {
      joints[track.id] = neutral[track.id] ?? 0
    }
  }
  return { joints, wsValues }
}

/** Every pose id an animation's tracks reference, in order, deduped. */
export function referencedPoseIds (animation) {
  const out = []
  for (const track of animation?.value_tracks || []) {
    if ((track.target_kind || '') !== 'pose') continue
    const id = poseIdOf(track)
    if (id && !out.includes(id)) out.push(id)
  }
  return out
}

/**
 * Flatten a resolved frame into the wire shape `preview_animation_frame`
 * expects. Pose tracks are already resolved to joint values here, so the
 * server never needs a "pose" target kind on its preview path — one
 * resolution point per side, and the server's player uses its own.
 */
export function frameToPreviewValues (frame) {
  const out = []
  for (const [joint, value] of Object.entries(frame.joints || {})) {
    out.push({ target_kind: 'urdf_joint', id: joint, value })
  }
  for (const ws of frame.wsValues || []) {
    out.push({ target_kind: 'ws_input', target: ws.target, value: ws.value })
  }
  return out
}
