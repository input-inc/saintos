"""Read-only URDF introspection: names, limits, and the kinematic tree.

The URDF owns every name in the stack. The SRDF and the rig file are
pure annotation layers whose every reference is a dangling name until
it resolves against this model, so parsing the URDF into a queryable
form is the prerequisite for validating either of them.

Three consumers, each needing something different:

  * **SRDF validation + chain expansion** — a ``<group>`` may name a
    ``<chain base_link= tip_link=>`` rather than list its joints, which
    only expands by walking child→parent links up the tree.
  * **Unit conversion.** SRDF ``<group_state>`` joint values are in
    URDF-native units (radians for revolute/continuous, metres for
    prismatic). Everything downstream of the routing graph — and
    ``URDFViewer.setJointValue`` in the web UI — speaks a normalized
    −1..+1. Converting between them requires each joint's ``<limit>``.
  * **Rig evaluation** — clamping a blended pose needs the same limits,
    and ``<mimic>`` couplings have to be applied after the blend so a
    mimicking joint isn't authored independently of its master.

Deliberately not a general URDF library: no geometry, no inertials, no
mesh resolution. Those stay in the store (which serves mesh bytes) and
in the web viewer (which renders them via urdf-loader).
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Set, Tuple
from xml.etree import ElementTree as ET


# Joint types that accept a setpoint. ``fixed`` joints are structural
# (they're how control anchor frames attach — a massless link on a
# fixed joint, the standard trick for a gizmo anchor) and never take a
# value. ``floating`` and ``planar`` are multi-DOF and can't be driven
# by a single scalar, so they're excluded from the actuatable set even
# though URDF permits them.
ACTUATABLE_TYPES = frozenset({"revolute", "continuous", "prismatic"})

# Fallback half-range for a ``continuous`` joint, which by definition
# has no ``<limit lower/upper>``. Normalizing against ±π keeps the
# −1..+1 convention meaningful; the rig evaluator handles wraparound
# separately (shortest-path interpolation) since lerping a continuous
# joint the long way round is the classic "one baffling 359° rotation"
# bug.
CONTINUOUS_HALF_RANGE = math.pi


@dataclass
class MimicSpec:
    """``q_this = multiplier * q_joint + offset`` — URDF's one real
    coupling primitive, honored by MoveIt and ros2_control.

    Parsed so the rig evaluator can apply it *after* blending: a
    mimicking joint has no independent authored value, and letting a
    pose set one directly would fight the coupling.
    """
    joint: str
    multiplier: float = 1.0
    offset: float = 0.0


@dataclass
class JointInfo:
    name: str
    type: str
    parent_link: str = ""
    child_link: str = ""
    # None for continuous joints and for joints whose <limit> is absent
    # (legal in URDF for continuous; a spec violation otherwise, but we
    # tolerate it rather than reject the model).
    lower: Optional[float] = None
    upper: Optional[float] = None
    velocity: Optional[float] = None
    effort: Optional[float] = None
    mimic: Optional[MimicSpec] = None

    @property
    def actuatable(self) -> bool:
        return self.type in ACTUATABLE_TYPES

    @property
    def has_limits(self) -> bool:
        return self.lower is not None and self.upper is not None

    def range(self) -> Tuple[float, float]:
        """Effective (lower, upper) in URDF-native units.

        Continuous joints — and limit-less joints of any type — fall
        back to ±π so normalization still has a defined range.
        """
        if self.has_limits and self.upper > self.lower:
            return (float(self.lower), float(self.upper))
        return (-CONTINUOUS_HALF_RANGE, CONTINUOUS_HALF_RANGE)

    def normalize(self, native: float) -> float:
        """URDF-native value (rad / m) → −1..+1.

        Maps the joint's range onto −1..+1 linearly. An asymmetric
        range maps its own zero to a non-zero normalized value, which
        is correct: normalized 0 means "the midpoint of travel", and
        that's the convention the rest of the stack already uses for
        peripheral channels.
        """
        lo, hi = self.range()
        span = hi - lo
        if span <= 0:
            return 0.0
        n = 2.0 * (float(native) - lo) / span - 1.0
        return max(-1.0, min(1.0, n))

    def denormalize(self, norm: float) -> float:
        """−1..+1 → URDF-native value (rad / m). Inverse of normalize."""
        lo, hi = self.range()
        n = max(-1.0, min(1.0, float(norm)))
        return lo + (n + 1.0) * 0.5 * (hi - lo)

    def clamp_native(self, native: float) -> float:
        lo, hi = self.range()
        return max(lo, min(hi, float(native)))


@dataclass
class UrdfModel:
    """Queryable view of a parsed URDF."""

    robot_name: str = ""
    links: List[str] = field(default_factory=list)
    joints: Dict[str, JointInfo] = field(default_factory=dict)

    # ── construction ────────────────────────────────────────────────

    @classmethod
    def parse(cls, urdf_bytes: bytes) -> "UrdfModel":
        """Parse URDF bytes. Raises ET.ParseError on malformed XML.

        Tolerant by design: a missing ``<limit>``, an unparseable
        numeric attribute, or a joint with no parent/child yields a
        partial JointInfo rather than an exception. A robot model that
        renders but has one odd joint is more useful to an operator
        than a hard failure, and :meth:`validate` surfaces the gaps.
        """
        root = ET.fromstring(urdf_bytes)
        model = cls(robot_name=root.get("name") or "")
        for link_el in root.findall("link"):
            name = link_el.get("name")
            if name:
                model.links.append(name)
        for joint_el in root.findall("joint"):
            info = _parse_joint(joint_el)
            if info is not None:
                model.joints[info.name] = info
        return model

    # ── queries ─────────────────────────────────────────────────────

    @property
    def link_set(self) -> Set[str]:
        return set(self.links)

    def actuatable_joints(self) -> List[JointInfo]:
        return [j for j in self.joints.values() if j.actuatable]

    def joint(self, name: str) -> Optional[JointInfo]:
        return self.joints.get(name)

    def normalize(self, joint: str, native: float) -> Optional[float]:
        """Convenience: native → normalized, or None for unknown joints.

        Returning None rather than falling back to a passthrough is
        deliberate — a silently unconverted radian value reaching a
        −1..+1 sink would drive a servo to its stop.
        """
        j = self.joints.get(joint)
        return None if j is None else j.normalize(native)

    def denormalize(self, joint: str, norm: float) -> Optional[float]:
        j = self.joints.get(joint)
        return None if j is None else j.denormalize(norm)

    def child_map(self) -> Dict[str, JointInfo]:
        """child_link → the joint that attaches it to its parent.

        The tree walks upward through this map. Each link has exactly
        one parent joint in a well-formed URDF (it's a strict tree — no
        closed loops, which is why animatronic four-bar linkages get
        modelled broken and closed in the rig evaluator instead).
        """
        return {j.child_link: j for j in self.joints.values() if j.child_link}

    def chain_joints(self, base_link: str, tip_link: str) -> List[str]:
        """Joints along the chain from ``base_link`` down to ``tip_link``.

        Walks tip→base through the child map and reverses, so the
        result reads base-first. Returns [] if the two links aren't
        connected in that direction — an SRDF group naming a bogus
        chain gets an empty expansion plus a validation warning, rather
        than a partial chain that would silently under-select joints.
        """
        if base_link == tip_link:
            return []
        parents = self.child_map()
        out: List[str] = []
        cursor = tip_link
        # Bound the walk by the joint count: a malformed URDF with a
        # cycle would otherwise spin here forever.
        for _ in range(len(self.joints) + 1):
            joint = parents.get(cursor)
            if joint is None:
                return []          # hit the root without finding base_link
            out.append(joint.name)
            cursor = joint.parent_link
            if cursor == base_link:
                out.reverse()
                return out
        return []

    def subtree_joints(self, root_link: str) -> List[str]:
        """Every joint at or below ``root_link``, breadth-first."""
        children: Dict[str, List[JointInfo]] = {}
        for j in self.joints.values():
            if j.parent_link:
                children.setdefault(j.parent_link, []).append(j)
        out: List[str] = []
        queue = [root_link]
        seen: Set[str] = set()
        while queue:
            link = queue.pop(0)
            if link in seen:
                continue
            seen.add(link)
            for j in children.get(link, []):
                out.append(j.name)
                if j.child_link:
                    queue.append(j.child_link)
        return out

    def validate(self) -> List[str]:
        """Structural warnings — never fatal, surfaced to the operator.

        Catches the cases that make downstream behavior confusing
        rather than broken: a revolute joint with no limits normalizes
        against a guessed ±π, and a mimic pointing at a missing joint
        silently never couples.
        """
        problems: List[str] = []
        if not self.robot_name:
            problems.append("<robot> has no name attribute; an SRDF cannot bind to it")
        link_set = self.link_set
        for j in self.joints.values():
            if j.parent_link and j.parent_link not in link_set:
                problems.append(
                    f"joint '{j.name}' references unknown parent link '{j.parent_link}'")
            if j.child_link and j.child_link not in link_set:
                problems.append(
                    f"joint '{j.name}' references unknown child link '{j.child_link}'")
            if j.type in ("revolute", "prismatic") and not j.has_limits:
                problems.append(
                    f"{j.type} joint '{j.name}' has no <limit>; "
                    f"normalizing against ±{CONTINUOUS_HALF_RANGE:.3f}")
            if j.mimic is not None and j.mimic.joint not in self.joints:
                problems.append(
                    f"joint '{j.name}' mimics unknown joint '{j.mimic.joint}'")
        return problems


def _parse_float(el: Optional[ET.Element], attr: str) -> Optional[float]:
    if el is None:
        return None
    raw = el.get(attr)
    if raw is None:
        return None
    try:
        return float(raw)
    except (TypeError, ValueError):
        return None


def _parse_joint(joint_el: ET.Element) -> Optional[JointInfo]:
    name = joint_el.get("name")
    if not name:
        return None
    limit_el = joint_el.find("limit")
    parent_el = joint_el.find("parent")
    child_el = joint_el.find("child")
    mimic_el = joint_el.find("mimic")

    mimic = None
    if mimic_el is not None and mimic_el.get("joint"):
        # `is None` rather than `or`: an explicit multiplier="0" is
        # legal and means "hold at the offset regardless of the master".
        # Defaulting that to 1.0 would silently couple a joint that was
        # authored to stay put.
        mult = _parse_float(mimic_el, "multiplier")
        off = _parse_float(mimic_el, "offset")
        mimic = MimicSpec(
            joint=mimic_el.get("joint") or "",
            multiplier=1.0 if mult is None else mult,
            offset=0.0 if off is None else off,
        )

    return JointInfo(
        name=name,
        type=joint_el.get("type") or "",
        parent_link=(parent_el.get("link") if parent_el is not None else "") or "",
        child_link=(child_el.get("link") if child_el is not None else "") or "",
        lower=_parse_float(limit_el, "lower"),
        upper=_parse_float(limit_el, "upper"),
        velocity=_parse_float(limit_el, "velocity"),
        effort=_parse_float(limit_el, "effort"),
        mimic=mimic,
    )
