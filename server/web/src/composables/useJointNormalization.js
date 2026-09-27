/**
 * Control value (−1..+1) ↔ URDF joint value (radians / metres).
 *
 * SaintOS drives joints in a −1..+1 control range — the same range the
 * servos use — and the 3D viewport has to put that on screen. Only the
 * viewport uses this: the robot is driven by the normalized value
 * straight through the routing sheet, so a mistake here shows the
 * operator motion the hardware will not make.
 *
 * Extracted from URDFViewer.vue so the mapping can be tested. It was
 * unreachable inside the SFC, which is why the case below went unnoticed.
 *
 * ── The mapping ─────────────────────────────────────────────────────
 *
 * When the joint's travel straddles home (lower ≤ 0 ≤ upper) the map is
 * HOME-CENTERED, pinning control 0 to the URDF home θ=0:
 *
 *     −1 → lower        0 → home (θ=0)        +1 → upper
 *
 * Each side is scaled independently (θ = n·upper for n ≥ 0, n·|lower|
 * for n < 0) so asymmetric joints keep 0 at home, at the cost of a
 * different gain per side. One-sided joints (lower = 0, or upper = 0)
 * have no travel on one side, so control values there hold at home —
 * that is deliberate and unchanged.
 *
 * When the travel does NOT contain home — `<limit lower="-2.0"
 * upper="-0.5">`, a joint whose whole range sits on one side of zero —
 * home-centering is impossible: θ=0 is not a position the joint can
 * reach. The old code applied the home-centered formula anyway, and the
 * result was that the ENTIRE positive control range collapsed onto the
 * nearest limit. An animation with positive keyframes showed the joint
 * pinned at a negative angle with no motion, while the robot — which
 * never goes through this function — moved correctly. For those joints
 * we map the full control range across the full travel instead:
 *
 *     −1 → lower        0 → midpoint          +1 → upper
 *
 * Reversed limits (lower > upper) are swapped rather than trusted. Left
 * as authored they made `max(lo, min(hi, θ))` collapse to a constant, so
 * the joint froze; an author who writes them backwards means the range,
 * not a locked joint.
 */

/** `{lo, hi}` for a joint with finite limits, else null. */
export function jointLimits (joint) {
  const lim = joint?.limit
  const lo = lim ? Number(lim.lower) : NaN
  const hi = lim ? Number(lim.upper) : NaN
  if (!Number.isFinite(lo) || !Number.isFinite(hi)) return null
  return lo <= hi ? { lo, hi } : { lo: hi, hi: lo }
}

/** True when home (θ=0) is inside the joint's travel. */
function straddlesHome (L) {
  return L.lo <= 0 && L.hi >= 0
}

/** control (−1..+1) → joint value (radians / metres). */
export function denormJoint (joint, n) {
  const L = jointLimits(joint)
  if (!L) return n
  const c = Math.max(-1, Math.min(1, Number(n) || 0))
  let theta
  if (straddlesHome(L)) {
    theta = c >= 0 ? c * L.hi : c * Math.abs(L.lo)
  } else {
    // Home is unreachable — spread the control range over the travel.
    theta = L.lo + ((c + 1) / 2) * (L.hi - L.lo)
  }
  return Math.max(L.lo, Math.min(L.hi, theta))
}

/** joint value (radians / metres) → control (−1..+1). Inverse of the above. */
export function normJoint (joint, theta) {
  const L = jointLimits(joint)
  if (!L) return theta
  const t = Number(theta) || 0
  if (!straddlesHome(L)) {
    const span = L.hi - L.lo
    if (span === 0) return 0
    const c = ((t - L.lo) / span) * 2 - 1
    return Math.max(-1, Math.min(1, c))
  }
  if (t >= 0) return L.hi > 0 ? Math.min(1, t / L.hi) : 0
  return L.lo < 0 ? Math.max(-1, t / Math.abs(L.lo)) : 0
}
