"""The rig file: controls, curves, bindings, and widget hints.

Third file in the stack, and the only custom schema in it:

    robot.urdf       robot_description            structure, limits, geometry
    robot.srdf       robot_description_semantic   groups + named poses
    robot.rig.xml    robot_description_rig        controls  ← this module

It follows the SRDF's precedent rather than smuggling tags into
someone else's format: namespaced, versioned, its own file, its own
parameter. That trade was made deliberately — see docs/RIG_SCHEMA.md
for the reasoning and the full element reference.

Two rules hold the design together, and both are load-bearing:

**Everything except ``<widget>`` is contract.** ``<widget>`` carries
presentation hints only — shape, colour, which panel a slider lands in.
A scripted performance, a CLI, a test harness, or recorded playback
needs the bindings and has no use for a colour. Fusing the two is what
makes a rig unable to run without a viewport, so the evaluator in
``rig_eval.py`` never reads a Widget.

**Evaluation is declarative, never a graph.** Controls resolve in a
fixed order: pose blends, then direct joint drives, then solver-backed
bindings. There is no node graph, no expression language, and no
control-drives-another-control edge. Unreal's Rig Graph is a full
visual programming environment with its own execution model and
debugger; the moment arbitrary logic sits between a control and a
joint, this stops being a feature and becomes a language.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional, Tuple
from xml.etree import ElementTree as ET

from saint_server.animation.srdf import SRDF
from saint_server.animation.urdf_model import UrdfModel
from saint_server.unreal.animation import EASING_NAME_TO_CODE, easing_code


# XML namespace. A URN rather than an http:// URI on purpose: an XML
# namespace is only an identifier, never a fetched document, and a URN
# doesn't imply ownership of a domain that might lapse.
#
# Pinned at :1.0 and NOT bumped for minor versions. A namespace change
# means "different vocabulary, re-write your documents"; 1.1 only added
# optional widget attributes, so every 1.0 file is still valid. Bump this
# only on a breaking major.
RIG_NAMESPACE = "urn:saintos:rig:1.0"

# Schema version this parser implements. Files declare `version="1.1"`;
# a file declaring a newer MAJOR is rejected rather than
# best-effort-parsed, since an unknown control kind silently dropped is
# worse than a load error an operator can see. A newer MINOR loads fine,
# so 1.0 files keep working — 1.1 only ADDED widget geometry.
RIG_VERSION = "1.1"

# Control shapes drawable in the 3D view. Deliberately a closed set: a
# shape name that silently falls back to a sphere would leave an operator
# wondering why their arrow didn't appear.
#
# Chosen to cover the interaction vocabulary rather than to mirror
# Unreal's full library — a ring for rotation, an arrow for a single
# axis, a pad/plane for two, a sphere or box for a position, and a wedge
# for a bounded sweep.
WIDGET_SHAPES = frozenset({
    "sphere", "box", "circle", "ring", "cylinder",
    "cone", "arrow", "diamond", "torus", "wedge", "plane",
})

# Control kinds.
#   channel — one scalar. Renders as a slider. A scalar with no spatial
#             meaning must never get a 3D gizmo; that's strictly worse
#             than a slider, which is why Unreal keeps float curves in
#             the Anim panel rather than the viewport.
#   pad     — two scalars (x, y). Renders as an XY pad. The right shape
#             for eye look on a pan/tilt mechanism: direct, predictable,
#             and no solver in the loop.
#   spatial — a 3D point in an anchor frame. Renders as a draggable
#             gizmo and requires a solver-backed <gaze> binding.
CONTROL_KINDS = frozenset({"channel", "pad", "spatial"})

# Limit-clamping policy, applied after all controls compose.
#   clamp      — clamp each joint independently at its limit.
#   scale_back — if any joint would exceed, scale the whole frame's
#                contribution back proportionally so the pose saturates
#                as a unit.
# The difference shows at the extremes: with per-joint clamping a brow
# stops while the mouth keeps travelling, so the face *breaks* rather
# than saturating. scale_back is the default for that reason.
CLAMP_POLICIES = frozenset({"clamp", "scale_back"})

_AXIS_NAMES = ("x", "y")


class RigError(Exception):
    """Raised when a rig file fails to parse or declares a future major."""


# ── binding elements (contract) ─────────────────────────────────────


@dataclass
class Drive:
    """Drive a joint directly from a control's value.

    ``contribution = curve(|v|) * sign(v) * scale + offset``, in
    NORMALIZED joint space (−1..+1). See the module note in
    ``rig_eval.py`` on why the whole rig evaluates normalized.

    This is the head-nod primitive: no pose needs to exist, and
    ``scale`` reads as "fraction of this joint's travel at full
    deflection", which is directly authorable.
    """
    joint: str
    scale: float = 1.0
    offset: float = 0.0
    curve: int = 1                      # easing code; 1 == linear

    def to_dict(self) -> Dict[str, Any]:
        return {"joint": self.joint, "scale": self.scale,
                "offset": self.offset, "curve": self.curve}


@dataclass
class Target:
    """A named SRDF pose reached at control value ``at``.

    Targets make a control a 1D blend space. With ``at="-1"`` on "mad"
    and ``at="1"`` on "happy", one slider covers both and they can
    never be active at once — the same reason riggers use a single
    smile/frown slider instead of two.
    """
    pose: str
    at: float = 1.0
    curve: int = 1

    def to_dict(self) -> Dict[str, Any]:
        return {"pose": self.pose, "at": self.at, "curve": self.curve}


@dataclass
class Axis:
    """One axis of a ``pad`` control, with its own bindings."""
    name: str = "x"
    min: float = -1.0
    max: float = 1.0
    default: float = 0.0
    drives: List[Drive] = field(default_factory=list)
    targets: List[Target] = field(default_factory=list)

    def to_dict(self) -> Dict[str, Any]:
        return {
            "name": self.name, "min": self.min, "max": self.max,
            "default": self.default,
            "drives": [d.to_dict() for d in self.drives],
            "targets": [t.to_dict() for t in self.targets],
        }


@dataclass
class GazeFrame:
    """One frame participating in a gaze solve.

    ``axis`` is the frame-local forward direction — which way this link
    "points". Weight sets how hard this frame competes: strong on the
    eyes, weak on the head, and the head follows.
    """
    link: str
    axis: Tuple[float, float, float] = (0.0, 0.0, 1.0)
    weight: float = 1.0

    def to_dict(self) -> Dict[str, Any]:
        return {"link": self.link, "axis": list(self.axis), "weight": self.weight}


@dataclass
class Regularizer:
    """A posture task pulling joints back toward centre.

    This is what produces eyes-lead-head-follows without a single
    authored curve. The eyes snap to the target because their gaze task
    is strong; this task then complains they're off-centre, and the only
    way to satisfy both is for the head to rotate and bring them back to
    neutral. Two weights, not a curve per joint.
    """
    joints: List[str] = field(default_factory=list)
    weight: float = 0.05

    def to_dict(self) -> Dict[str, Any]:
        return {"joints": list(self.joints), "weight": self.weight}


@dataclass
class Gaze:
    """A look-at binding: point some frames at a target.

    Deliberately a 2-DOF residual per frame, not a 6-DOF pose goal. A
    full orientation target pins roll about the gaze axis, which forces
    an arbitrary head tilt as a side effect of asking the rig to look
    somewhere — in rigging terms, an aim constraint with a spurious
    up-vector.
    """
    frames: List[GazeFrame] = field(default_factory=list)
    regularizers: List[Regularizer] = field(default_factory=list)

    def joints(self) -> List[str]:
        out: List[str] = []
        for r in self.regularizers:
            for j in r.joints:
                if j not in out:
                    out.append(j)
        return out

    def to_dict(self) -> Dict[str, Any]:
        return {
            "frames": [f.to_dict() for f in self.frames],
            "regularizers": [r.to_dict() for r in self.regularizers],
        }


# ── presentation (never read by the evaluator) ──────────────────────


@dataclass
class Widget:
    """Presentation hints — how a control is drawn and grabbed.

    Cosmetic by contract: the evaluator never reads a Widget, so the rig
    still runs headless. But "cosmetic" is not the same as "vague". A
    control shape sits at a real place on the robot with a real
    orientation and a real drag axis, and that geometry is what turns an
    abstract scalar into something you can grab — the same trick
    Unreal's Control Rig uses when it parents a shape to a bone with an
    offset transform.

    ``offset`` and ``rotation`` are expressed in the ANCHOR LINK's frame,
    so a shape follows the joint it belongs to for free.

    ``axis`` is the drag direction for a channel control, in the widget's
    own (post-rotation) frame: pointer motion is projected onto that axis
    on screen and mapped to the control's min..max. That's what gives a
    scalar spatial meaning rather than leaving it a slider bolted into
    3D space.

    ``dofs`` mirrors ``visualization_msgs/InteractiveMarkerControl``'s
    ``interaction_mode`` vocabulary (move_3d, move_axis, rotate_axis, …)
    so a spatial control can be instantiated as a ROS interactive marker
    without a translation table, which makes rviz a free viewport.
    """
    kind: str = ""                       # slider | pad | gizmo
    shape: str = "sphere"
    # Per-axis, so a ring can be flattened or an arrow stretched. A
    # single number in the file expands to all three.
    scale: Tuple[float, float, float] = (0.05, 0.05, 0.05)
    color: Tuple[float, float, float, float] = (1.0, 0.7, 0.0, 0.8)
    # Placement within the anchor link's frame.
    offset: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    rotation: Tuple[float, float, float] = (0.0, 0.0, 0.0)   # roll pitch yaw
    # Drag direction for channel controls, in the widget's frame.
    axis: Tuple[float, float, float] = (1.0, 0.0, 0.0)
    dofs: str = "move_3d"
    invert_y: bool = False
    # Draw this control in the 3D view at all. A control that's only
    # ever driven by an animation doesn't need a handle cluttering the
    # viewport, but still wants its slider in the panel.
    visible: bool = True

    def to_dict(self) -> Dict[str, Any]:
        return {
            "kind": self.kind, "shape": self.shape,
            "scale": list(self.scale),
            "color": list(self.color),
            "offset": list(self.offset),
            "rotation": list(self.rotation),
            "axis": list(self.axis),
            "dofs": self.dofs,
            "invert_y": self.invert_y,
            "visible": self.visible,
        }


# ── controls ────────────────────────────────────────────────────────


@dataclass
class Control:
    name: str
    kind: str = "channel"
    label: str = ""
    group: str = ""                      # UI panel grouping
    order: int = 0                       # sort key within the group
    min: float = -1.0
    max: float = 1.0
    default: float = 0.0
    targets: List[Target] = field(default_factory=list)
    drives: List[Drive] = field(default_factory=list)
    axes: List[Axis] = field(default_factory=list)      # pad only
    gaze: Optional[Gaze] = None                          # spatial only
    anchor: str = ""                     # URDF link the gizmo lives in
    widget: Widget = field(default_factory=Widget)

    @property
    def display_label(self) -> str:
        return self.label or self.name

    def axis(self, name: str) -> Optional[Axis]:
        for a in self.axes:
            if a.name == name:
                return a
        return None

    def referenced_poses(self) -> List[str]:
        out = [t.pose for t in self.targets]
        for a in self.axes:
            out.extend(t.pose for t in a.targets)
        return out

    def referenced_joints(self) -> List[str]:
        out = [d.joint for d in self.drives]
        for a in self.axes:
            out.extend(d.joint for d in a.drives)
        if self.gaze is not None:
            out.extend(self.gaze.joints())
        return out

    def to_dict(self) -> Dict[str, Any]:
        return {
            "name": self.name, "kind": self.kind, "label": self.label,
            "group": self.group, "order": self.order,
            "min": self.min, "max": self.max, "default": self.default,
            "anchor": self.anchor,
            "targets": [t.to_dict() for t in self.targets],
            "drives": [d.to_dict() for d in self.drives],
            "axes": [a.to_dict() for a in self.axes],
            "gaze": self.gaze.to_dict() if self.gaze else None,
            "widget": self.widget.to_dict(),
        }


@dataclass
class RigSettings:
    clamp: str = "scale_back"
    # Named SRDF group_state used as the blend origin q₀. Without one,
    # neutral is all-zeros (the midpoint of every joint's travel).
    neutral_pose: str = ""

    def to_dict(self) -> Dict[str, Any]:
        return {"clamp": self.clamp, "neutral_pose": self.neutral_pose}


@dataclass
class Rig:
    version: str = RIG_VERSION
    robot: str = ""
    settings: RigSettings = field(default_factory=RigSettings)
    controls: List[Control] = field(default_factory=list)

    # ── construction ────────────────────────────────────────────────

    @classmethod
    def parse(cls, rig_bytes: bytes) -> "Rig":
        try:
            root = ET.fromstring(rig_bytes)
        except ET.ParseError as e:
            raise RigError(f"rig file is not valid XML: {e}") from e

        if _localname(root.tag) != "rig":
            raise RigError(
                f"rig root element must be <rig>, got <{_localname(root.tag)}>")

        version = (root.get("version") or RIG_VERSION).strip()
        _check_version(version)

        rig = cls(version=version, robot=root.get("robot") or "")

        settings_el = _find(root, "settings")
        if settings_el is not None:
            clamp = (settings_el.get("clamp") or "scale_back").strip()
            if clamp not in CLAMP_POLICIES:
                raise RigError(
                    f"<settings clamp=\"{clamp}\"> is not a known policy; "
                    f"expected one of {sorted(CLAMP_POLICIES)}")
            rig.settings = RigSettings(
                clamp=clamp,
                neutral_pose=settings_el.get("neutral_pose") or "",
            )

        for el in _findall(root, "control"):
            rig.controls.append(_parse_control(el))

        names = [c.name for c in rig.controls]
        dupes = {n for n in names if names.count(n) > 1}
        if dupes:
            raise RigError(
                f"duplicate control name(s): {', '.join(sorted(dupes))}")

        return rig

    # ── queries ─────────────────────────────────────────────────────

    def control(self, name: str) -> Optional[Control]:
        for c in self.controls:
            if c.name == name:
                return c
        return None

    def controls_by_group(self) -> List[Tuple[str, List[Control]]]:
        """Controls bucketed by ``group``, for the UI panel.

        Groups appear in first-seen order (so file order is authorable
        intent), and controls sort by ``order`` then label within each.
        """
        buckets: Dict[str, List[Control]] = {}
        order: List[str] = []
        for c in self.controls:
            key = c.group or ""
            if key not in buckets:
                buckets[key] = []
                order.append(key)
            buckets[key].append(c)
        return [
            (g, sorted(buckets[g], key=lambda c: (c.order, c.display_label)))
            for g in order
        ]

    def referenced_poses(self) -> List[str]:
        out: List[str] = []
        for c in self.controls:
            for p in c.referenced_poses():
                if p not in out:
                    out.append(p)
        return out

    def resolve_anchors(self, urdf: Optional[UrdfModel]) -> Dict[str, str]:
        """control name → the URDF link its shape hangs off.

        An explicit ``anchor`` always wins. Absent one, the anchor is
        DERIVED from what the control drives: the child link of the first
        joint it moves, or — for a pure pose-blend control with no direct
        drives — the child link of the first joint its poses name.

        Deriving matters more than it looks. Without it, every rig file
        would need an ``anchor`` on every control before anything appeared
        in the viewport, and a file written before this feature existed
        would silently draw nothing. With it, a control lands on the part
        of the robot it actually moves and an operator only writes an
        anchor when they want to override that.

        Controls with no resolvable link are omitted rather than defaulted
        to the robot root, where a cluster of unrelated handles would pile
        up on the base and look like a bug.
        """
        out: Dict[str, str] = {}
        if urdf is None:
            return out
        for c in self.controls:
            if c.anchor:
                out[c.name] = c.anchor
                continue
            link = self._derive_anchor(c, urdf)
            if link:
                out[c.name] = link
        return out

    def _derive_anchor(self, control: Control,
                       urdf: UrdfModel) -> Optional[str]:
        for joint_name in control.referenced_joints():
            joint = urdf.joint(joint_name)
            if joint is not None and joint.child_link:
                return joint.child_link
        # Pose-blend-only control: fall back to the first joint any of its
        # target poses touches. Needs the pose data, which this module
        # doesn't have, so callers wanting this precision pass
        # `pose_joints` to resolve_anchors_with_poses below.
        if control.gaze is not None:
            for frame in control.gaze.frames:
                if frame.link in urdf.link_set:
                    return frame.link
        return None

    def resolve_anchors_with_poses(
            self, urdf: Optional[UrdfModel],
            pose_joints: Optional[Dict[str, Dict[str, float]]] = None
    ) -> Dict[str, str]:
        """:meth:`resolve_anchors`, plus pose data so a control that only
        blends poses (no direct ``<drive>``) still lands somewhere sensible.

        Split out because ``rig.py`` has no business loading the pose
        library — the caller that already has it passes it in.
        """
        out = self.resolve_anchors(urdf)
        if urdf is None or not pose_joints:
            return out
        for c in self.controls:
            if c.name in out:
                continue
            for pose_name in c.referenced_poses():
                joints = pose_joints.get(pose_name) or {}
                for joint_name in sorted(joints):
                    joint = urdf.joint(joint_name)
                    if joint is not None and joint.child_link:
                        out[c.name] = joint.child_link
                        break
                if c.name in out:
                    break
        return out

    def validate(self, urdf: Optional[UrdfModel] = None,
                 srdf: Optional[SRDF] = None,
                 pose_names: Optional[List[str]] = None) -> List[str]:
        """Cross-check every reference against the URDF, SRDF, and poses.

        Same rationale as SRDF.validate: this file is almost entirely
        references, and an unresolved one is a silent no-op. A control
        whose drive names a renamed joint just stops doing anything, and
        nothing in any log says so.

        ``pose_names`` lets the caller supply the *pose library* rather
        than the SRDF, since a pose can be authored in the UI and saved
        without ever appearing in a group_state.
        """
        problems: List[str] = []

        if urdf is not None and self.robot and urdf.robot_name \
                and self.robot != urdf.robot_name:
            problems.append(
                f"rig <rig robot=\"{self.robot}\"> does not match URDF "
                f"<robot name=\"{urdf.robot_name}\">")

        known_poses = set(pose_names or [])
        if srdf is not None:
            known_poses |= {gs.name for gs in srdf.group_states}

        neutral = self.settings.neutral_pose
        if neutral and known_poses and neutral not in known_poses:
            problems.append(
                f"<settings neutral_pose=\"{neutral}\"> names no known pose")

        for c in self.controls:
            problems.extend(self._validate_control(c, urdf, known_poses))

        return problems

    def _validate_control(self, c: Control, urdf: Optional[UrdfModel],
                          known_poses: set) -> List[str]:
        problems: List[str] = []
        where = f"control '{c.name}'"

        if c.kind not in CONTROL_KINDS:
            problems.append(
                f"{where}: unknown kind '{c.kind}'; "
                f"expected one of {sorted(CONTROL_KINDS)}")

        # Shape rules per kind. These are the ones that produce a
        # control that renders but does nothing, which is the most
        # confusing possible outcome for an operator.
        if c.kind == "pad":
            if not c.axes:
                problems.append(f"{where}: kind='pad' needs <axis> children")
            for a in c.axes:
                if a.name not in _AXIS_NAMES:
                    problems.append(
                        f"{where}: axis name '{a.name}' must be one of "
                        f"{list(_AXIS_NAMES)}")
                if not a.drives and not a.targets:
                    problems.append(
                        f"{where}: axis '{a.name}' has no <drive> or <target>, "
                        f"so moving it does nothing")
        elif c.kind == "spatial":
            if c.gaze is None:
                problems.append(
                    f"{where}: kind='spatial' needs a <gaze> binding")
            if not c.anchor:
                problems.append(
                    f"{where}: kind='spatial' needs anchor=\"<link>\" naming "
                    f"the frame its target lives in")
        else:   # channel
            if not c.drives and not c.targets:
                problems.append(
                    f"{where}: no <drive> or <target>, so it does nothing")
            if c.axes:
                problems.append(
                    f"{where}: <axis> is only meaningful on kind='pad'")

        if c.min >= c.max:
            problems.append(f"{where}: min ({c.min}) must be below max ({c.max})")
        if not (c.min <= c.default <= c.max):
            problems.append(
                f"{where}: default ({c.default}) falls outside "
                f"min..max ({c.min}..{c.max})")

        # Pose references.
        if known_poses:
            for pose in c.referenced_poses():
                if pose not in known_poses:
                    problems.append(f"{where}: <target> names no known pose '{pose}'")

        # Joint + link references.
        if urdf is not None:
            for joint in c.referenced_joints():
                j = urdf.joint(joint)
                if j is None:
                    problems.append(f"{where}: unknown joint '{joint}'")
                elif not j.actuatable:
                    problems.append(
                        f"{where}: joint '{joint}' is {j.type or 'untyped'} "
                        f"and accepts no value")
            if c.anchor and c.anchor not in urdf.link_set:
                problems.append(f"{where}: anchor names unknown link '{c.anchor}'")
            if c.gaze is not None:
                for f in c.gaze.frames:
                    if f.link not in urdf.link_set:
                        problems.append(
                            f"{where}: <frame> names unknown link '{f.link}'")

        # Duplicate `at` stops make a blend segment zero-width, which
        # would divide by zero in the evaluator.
        ats = [t.at for t in c.targets]
        if len(set(ats)) != len(ats):
            problems.append(
                f"{where}: two <target> elements share the same at= value")

        return problems

    def to_dict(self) -> Dict[str, Any]:
        return {
            "version": self.version,
            "robot": self.robot,
            "settings": self.settings.to_dict(),
            "controls": [c.to_dict() for c in self.controls],
        }

    def summary(self) -> Dict[str, Any]:
        return {
            "version": self.version,
            "robot": self.robot,
            "control_count": len(self.controls),
            "clamp": self.settings.clamp,
            "neutral_pose": self.settings.neutral_pose,
            "controls": [
                {"name": c.name, "kind": c.kind, "label": c.display_label,
                 "group": c.group}
                for c in self.controls
            ],
        }


# ── parsing helpers ─────────────────────────────────────────────────


def _localname(tag: str) -> str:
    """Strip any ``{namespace}`` prefix ElementTree prepends.

    We accept both namespaced and bare documents. Requiring the xmlns
    would make every hand-authored rig file fail its first load for a
    reason the error message can't easily explain, and the namespace
    buys nothing at parse time — it matters for tools that *aren't*
    this one.
    """
    return tag.rsplit("}", 1)[-1] if "}" in tag else tag


def _find(parent: ET.Element, name: str) -> Optional[ET.Element]:
    for child in parent:
        if _localname(child.tag) == name:
            return child
    return None


def _findall(parent: ET.Element, name: str) -> List[ET.Element]:
    return [c for c in parent if _localname(c.tag) == name]


def _check_version(version: str) -> None:
    major = version.split(".", 1)[0].strip()
    want_major = RIG_VERSION.split(".", 1)[0]
    if not major.isdigit():
        raise RigError(f"rig version '{version}' is not a version number")
    if int(major) > int(want_major):
        raise RigError(
            f"rig file declares version {version}, but this server "
            f"implements {RIG_VERSION}. Refusing to guess — an unknown "
            f"control kind dropped in silence is worse than a load error.")


def _float_attr(el: ET.Element, name: str, default: float) -> float:
    raw = el.get(name)
    if raw is None:
        return default
    try:
        return float(raw)
    except (TypeError, ValueError):
        raise RigError(
            f"<{_localname(el.tag)} {name}=\"{raw}\"> is not a number")


def _int_attr(el: ET.Element, name: str, default: int) -> int:
    raw = el.get(name)
    if raw is None:
        return default
    try:
        return int(raw)
    except (TypeError, ValueError):
        raise RigError(
            f"<{_localname(el.tag)} {name}=\"{raw}\"> is not an integer")


def _curve_attr(el: ET.Element) -> int:
    raw = (el.get("curve") or "").strip()
    if not raw:
        return EASING_NAME_TO_CODE["linear"]
    if raw not in EASING_NAME_TO_CODE:
        code = easing_code(raw, default=-1)
        if code < 0:
            raise RigError(
                f"<{_localname(el.tag)} curve=\"{raw}\"> is not a known "
                f"easing name. See docs/RIG_SCHEMA.md for the catalog.")
        return code
    return EASING_NAME_TO_CODE[raw]


def _vec3_attr(el: ET.Element, name: str,
               default: Tuple[float, float, float]) -> Tuple[float, float, float]:
    raw = el.get(name)
    if not raw:
        return default
    parts = raw.replace(",", " ").split()
    if len(parts) != 3:
        raise RigError(
            f"<{_localname(el.tag)} {name}=\"{raw}\"> needs 3 numbers")
    try:
        return (float(parts[0]), float(parts[1]), float(parts[2]))
    except ValueError:
        raise RigError(
            f"<{_localname(el.tag)} {name}=\"{raw}\"> is not 3 numbers")


def _parse_drive(el: ET.Element) -> Drive:
    joint = el.get("joint")
    if not joint:
        raise RigError("<drive> needs a joint= attribute")
    return Drive(
        joint=joint,
        scale=_float_attr(el, "scale", 1.0),
        offset=_float_attr(el, "offset", 0.0),
        curve=_curve_attr(el),
    )


def _parse_target(el: ET.Element) -> Target:
    pose = el.get("pose")
    if not pose:
        raise RigError("<target> needs a pose= attribute")
    return Target(pose=pose, at=_float_attr(el, "at", 1.0),
                  curve=_curve_attr(el))


def _parse_axis(el: ET.Element) -> Axis:
    return Axis(
        name=(el.get("name") or "x").strip().lower(),
        min=_float_attr(el, "min", -1.0),
        max=_float_attr(el, "max", 1.0),
        default=_float_attr(el, "default", 0.0),
        drives=[_parse_drive(c) for c in _findall(el, "drive")],
        targets=[_parse_target(c) for c in _findall(el, "target")],
    )


def _parse_gaze(el: ET.Element) -> Gaze:
    frames = []
    for f in _findall(el, "frame"):
        link = f.get("link")
        if not link:
            raise RigError("<frame> needs a link= attribute")
        frames.append(GazeFrame(
            link=link,
            axis=_vec3_attr(f, "axis", (0.0, 0.0, 1.0)),
            weight=_float_attr(f, "weight", 1.0),
        ))
    regs = []
    for r in _findall(el, "regularize"):
        joints = (r.get("joints") or "").replace(",", " ").split()
        regs.append(Regularizer(
            joints=joints, weight=_float_attr(r, "weight", 0.05)))
    return Gaze(frames=frames, regularizers=regs)


def _numbers_attr(el: ET.Element, name: str, allowed: Tuple[int, ...]) -> Optional[list]:
    """Parse a whitespace/comma-separated numeric attribute.

    Returns None when absent so the caller can keep its own default.
    """
    raw = el.get(name)
    if raw is None or not raw.strip():
        return None
    parts = raw.replace(",", " ").split()
    if len(parts) not in allowed:
        want = " or ".join(str(a) for a in allowed)
        raise RigError(
            f"<{_localname(el.tag)} {name}=\"{raw}\"> needs {want} numbers")
    try:
        return [float(p) for p in parts]
    except ValueError:
        raise RigError(
            f"<{_localname(el.tag)} {name}=\"{raw}\"> is not numeric")


# Default shape per control kind. A control that says nothing about its
# appearance still gets something grabbable and appropriate: a ring reads
# as "rotate this" for a 1-DOF channel, a plane as "drag in 2D" for a
# pad, a sphere as "a point in space" for a spatial target.
_DEFAULT_SHAPE = {"channel": "ring", "pad": "plane", "spatial": "sphere"}


def _parse_widget(el: Optional[ET.Element], kind: str) -> Widget:
    # Defaults follow the control kind, so the common case needs no
    # <widget> element at all and still draws in the viewport.
    default_kind = {"channel": "slider", "pad": "pad", "spatial": "gizmo"}.get(
        kind, "slider")
    default_shape = _DEFAULT_SHAPE.get(kind, "sphere")
    if el is None:
        return Widget(kind=default_kind, shape=default_shape)

    color = _numbers_attr(el, "color", (3, 4))
    rgba = ((color[0], color[1], color[2],
             color[3] if len(color) == 4 else 1.0)
            if color else (1.0, 0.7, 0.0, 0.8))

    # A single number means a uniform scale; three means per-axis.
    scale_nums = _numbers_attr(el, "scale", (1, 3))
    if scale_nums is None:
        scale = (0.05, 0.05, 0.05)
    elif len(scale_nums) == 1:
        scale = (scale_nums[0],) * 3
    else:
        scale = (scale_nums[0], scale_nums[1], scale_nums[2])

    offset = _numbers_attr(el, "offset", (3,)) or [0.0, 0.0, 0.0]
    rotation = _numbers_attr(el, "rotation", (3,)) or [0.0, 0.0, 0.0]
    axis = _numbers_attr(el, "axis", (3,)) or [1.0, 0.0, 0.0]

    shape = (el.get("shape") or default_shape).strip()
    if shape not in WIDGET_SHAPES:
        raise RigError(
            f"<widget shape=\"{shape}\"> is not a known shape; expected one "
            f"of {', '.join(sorted(WIDGET_SHAPES))}")

    visible = (el.get("visible") or "true").strip().lower()
    return Widget(
        kind=(el.get("kind") or default_kind).strip(),
        shape=shape,
        scale=scale,
        color=rgba,
        offset=(offset[0], offset[1], offset[2]),
        rotation=(rotation[0], rotation[1], rotation[2]),
        axis=(axis[0], axis[1], axis[2]),
        dofs=(el.get("dofs") or "move_3d").strip(),
        invert_y=(el.get("invert_y") or "").strip().lower() in ("1", "true", "yes"),
        visible=visible not in ("0", "false", "no"),
    )


def _parse_control(el: ET.Element) -> Control:
    name = el.get("name")
    if not name:
        raise RigError("<control> needs a name= attribute")
    kind = (el.get("kind") or "channel").strip()
    gaze_el = _find(el, "gaze")
    return Control(
        name=name,
        kind=kind,
        label=el.get("label") or "",
        group=el.get("group") or "",
        order=_int_attr(el, "order", 0),
        min=_float_attr(el, "min", -1.0),
        max=_float_attr(el, "max", 1.0),
        default=_float_attr(el, "default", 0.0),
        anchor=el.get("anchor") or "",
        targets=[_parse_target(c) for c in _findall(el, "target")],
        drives=[_parse_drive(c) for c in _findall(el, "drive")],
        axes=[_parse_axis(c) for c in _findall(el, "axis")],
        gaze=_parse_gaze(gaze_el) if gaze_el is not None else None,
        widget=_parse_widget(_find(el, "widget"), kind),
    )


def looks_like_rig(xml_bytes: bytes) -> bool:
    """True if these bytes are a rig file.

    Unlike URDF/SRDF (which share a ``<robot>`` root and need a
    content sniff), the rig file has its own root element, so this is
    unambiguous.
    """
    try:
        root = ET.fromstring(xml_bytes)
    except ET.ParseError:
        return False
    return _localname(root.tag) == "rig"
