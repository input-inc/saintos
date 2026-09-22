"""SRDF (Semantic Robot Description Format) parser.

The SRDF is a pure annotation layer over a URDF — MoveIt's format,
conventionally published as the ``robot_description_semantic``
parameter alongside ``robot_description``. It defines nothing
structural and cannot stand alone: every ``name=`` must resolve to a
link or joint in the URDF, and its ``<robot name="...">`` has to match
the URDF's robot name or nothing binds.

What we read, and why:

  * ``<group_state>`` — **named poses**, the reason this module exists.
    A group_state is literally a named list of joint values, which is
    exactly our pose library in a format MoveIt already reads.
  * ``<group>`` — joint groups (face, eyes, arm). Scopes both rig
    controls and the group_states above. A group may enumerate joints
    and links directly, name a ``<chain>``, or include other groups, so
    resolving one to a joint list needs the URDF tree.
  * ``<passive_joint>`` — joints nothing actuates. Relevant here
    because a broken-loop joint (the animatronic four-bar case) is
    driven by the rig evaluator rather than commanded directly.
  * ``<virtual_joint>`` — the one structural element SRDF adds rather
    than annotates: it attaches the robot root to a world frame.
  * ``<disable_collisions>`` — link pairs to skip. Usually most of the
    file by line count, and generated rather than hand-written; we
    parse it so the collision work can consume an SRDF-supplied ACM
    instead of the whitelist in docs/COLLISION_AVOIDANCE.md D3.

Note that an SRDF and a URDF share the same ``<robot>`` root element,
so the two are indistinguishable by root tag — only the extension and
the contents tell you which you have. :func:`looks_like_srdf` is what
the upload path uses to tell them apart.
"""

from __future__ import annotations

from dataclasses import dataclass, field, asdict
from typing import Any, Dict, List, Optional, Tuple
from xml.etree import ElementTree as ET

from saint_server.animation.urdf_model import UrdfModel


class SRDFError(Exception):
    """Raised when an SRDF fails to parse or isn't an SRDF at all."""


# Elements that only ever appear in an SRDF, never a URDF. Used to
# disambiguate the shared <robot> root. A URDF's distinguishing
# children are <link>/<joint> with structural content; an SRDF's
# <link>/<joint> references (inside groups) carry no geometry.
_SRDF_ONLY_TAGS = frozenset({
    "group", "group_state", "end_effector", "virtual_joint",
    "passive_joint", "disable_collisions", "disable_default_collisions",
    "link_sphere_approximation",
})


@dataclass
class GroupState:
    """A named pose: joint values in URDF-native units.

    ``joint_values`` is joint name → radians (revolute/continuous) or
    metres (prismatic) — NOT the normalized −1..+1 the routing graph
    and the 3D viewer use. Conversion is the caller's job and needs
    the URDF's ``<limit>`` for each joint; see
    :meth:`GroupState.normalized_values`.

    Multi-DOF joints may carry several values in SRDF; we keep only the
    first, since every joint we can drive is single-DOF.
    """
    name: str
    group: str
    joint_values: Dict[str, float] = field(default_factory=dict)

    def normalized_values(self, urdf: UrdfModel) -> Tuple[Dict[str, float], List[str]]:
        """Convert to normalized −1..+1 against the URDF's limits.

        Returns ``(normalized, unresolved)`` — joints absent from the
        URDF land in ``unresolved`` and are dropped rather than passed
        through. A radian value leaking into a −1..+1 sink would
        command a servo straight to its stop, so silence is not an
        option here.
        """
        out: Dict[str, float] = {}
        unresolved: List[str] = []
        for joint, native in self.joint_values.items():
            n = urdf.normalize(joint, native)
            if n is None:
                unresolved.append(joint)
            else:
                out[joint] = n
        return out, unresolved

    def to_dict(self) -> Dict[str, Any]:
        return {
            "name": self.name,
            "group": self.group,
            "joint_values": dict(self.joint_values),
        }


@dataclass
class Group:
    """A named collection of joints, links, chains, and subgroups."""
    name: str
    joints: List[str] = field(default_factory=list)
    links: List[str] = field(default_factory=list)
    chains: List[Tuple[str, str]] = field(default_factory=list)   # (base, tip)
    subgroups: List[str] = field(default_factory=list)

    def to_dict(self) -> Dict[str, Any]:
        return {
            "name": self.name,
            "joints": list(self.joints),
            "links": list(self.links),
            "chains": [list(c) for c in self.chains],
            "subgroups": list(self.subgroups),
        }


@dataclass
class VirtualJoint:
    name: str
    type: str = "fixed"
    parent_frame: str = ""
    child_link: str = ""

    def to_dict(self) -> Dict[str, Any]:
        return asdict(self)


@dataclass
class EndEffector:
    name: str
    group: str = ""
    parent_link: str = ""
    parent_group: str = ""

    def to_dict(self) -> Dict[str, Any]:
        return asdict(self)


@dataclass
class SRDF:
    """A parsed SRDF. Every name here is a reference into a URDF."""

    robot_name: str = ""
    groups: List[Group] = field(default_factory=list)
    group_states: List[GroupState] = field(default_factory=list)
    passive_joints: List[str] = field(default_factory=list)
    virtual_joints: List[VirtualJoint] = field(default_factory=list)
    end_effectors: List[EndEffector] = field(default_factory=list)
    # (link1, link2, reason) — reason is often "Adjacent" or "Never"
    # when produced by MoveIt's Setup Assistant.
    disabled_collisions: List[Tuple[str, str, str]] = field(default_factory=list)

    # ── construction ────────────────────────────────────────────────

    @classmethod
    def parse(cls, srdf_bytes: bytes) -> "SRDF":
        try:
            root = ET.fromstring(srdf_bytes)
        except ET.ParseError as e:
            raise SRDFError(f"SRDF is not valid XML: {e}") from e
        if root.tag != "robot":
            raise SRDFError(
                f"SRDF root element must be <robot>, got <{root.tag}>")

        srdf = cls(robot_name=root.get("name") or "")

        for el in root.findall("group"):
            name = el.get("name")
            if not name:
                continue
            srdf.groups.append(Group(
                name=name,
                joints=[c.get("name") for c in el.findall("joint") if c.get("name")],
                links=[c.get("name") for c in el.findall("link") if c.get("name")],
                chains=[
                    (c.get("base_link") or "", c.get("tip_link") or "")
                    for c in el.findall("chain")
                ],
                subgroups=[c.get("name") for c in el.findall("group") if c.get("name")],
            ))

        for el in root.findall("group_state"):
            name = el.get("name")
            if not name:
                continue
            values: Dict[str, float] = {}
            for jel in el.findall("joint"):
                jname = jel.get("name")
                if not jname:
                    continue
                raw = jel.get("value")
                if raw is None:
                    continue
                # SRDF permits several whitespace-separated values for a
                # multi-DOF joint. Everything we can drive is 1-DOF, so
                # take the first and ignore the rest.
                first = raw.strip().split()
                if not first:
                    continue
                try:
                    values[jname] = float(first[0])
                except ValueError:
                    continue
            srdf.group_states.append(GroupState(
                name=name, group=el.get("group") or "", joint_values=values,
            ))

        for el in root.findall("passive_joint"):
            name = el.get("name")
            if name:
                srdf.passive_joints.append(name)

        for el in root.findall("virtual_joint"):
            name = el.get("name")
            if name:
                srdf.virtual_joints.append(VirtualJoint(
                    name=name,
                    type=el.get("type") or "fixed",
                    parent_frame=el.get("parent_frame") or "",
                    child_link=el.get("child_link") or "",
                ))

        for el in root.findall("end_effector"):
            name = el.get("name")
            if name:
                srdf.end_effectors.append(EndEffector(
                    name=name,
                    group=el.get("group") or "",
                    parent_link=el.get("parent_link") or "",
                    parent_group=el.get("parent_group") or "",
                ))

        # MoveIt emits <disable_collisions>; newer configs also use
        # <disable_default_collisions>. Both mean "skip this pair".
        for tag in ("disable_collisions", "disable_default_collisions"):
            for el in root.findall(tag):
                l1, l2 = el.get("link1"), el.get("link2")
                if l1 and l2:
                    srdf.disabled_collisions.append(
                        (l1, l2, el.get("reason") or ""))

        return srdf

    # ── queries ─────────────────────────────────────────────────────

    def group(self, name: str) -> Optional[Group]:
        for g in self.groups:
            if g.name == name:
                return g
        return None

    def group_state(self, name: str, group: str = "") -> Optional[GroupState]:
        """Find a group_state by name, optionally scoped to a group.

        SRDF only requires group_state names to be unique *within* a
        group, so two groups can both define "open". The group argument
        disambiguates; without it the first match wins.
        """
        for gs in self.group_states:
            if gs.name == name and (not group or gs.group == group):
                return gs
        return None

    def resolve_group_joints(self, name: str, urdf: UrdfModel,
                             _seen: Optional[set] = None) -> List[str]:
        """Expand a group to its full actuatable joint list.

        Handles all four SRDF ways of naming membership: explicit
        joints, links (contributing the joint that drives each link),
        chains (walked through the URDF tree), and subgroups
        (recursively). Order is preserved and duplicates collapse, so
        the result is stable enough to drive a UI list.

        Non-actuatable joints are filtered out — a group legitimately
        contains fixed joints (they're how anchor frames attach) but
        they accept no setpoint, so including them in a pose or a
        control binding would be meaningless.
        """
        seen_groups = _seen if _seen is not None else set()
        if name in seen_groups:
            return []                      # cyclic subgroup reference
        seen_groups.add(name)

        grp = self.group(name)
        if grp is None:
            return []

        ordered: List[str] = []
        seen: set = set()

        def add(joint_name: str) -> None:
            j = urdf.joint(joint_name)
            if j is None or not j.actuatable or joint_name in seen:
                return
            seen.add(joint_name)
            ordered.append(joint_name)

        for jn in grp.joints:
            add(jn)
        for link in grp.links:
            # A link names the joint that moves it — its parent joint.
            for j in urdf.joints.values():
                if j.child_link == link:
                    add(j.name)
        for base, tip in grp.chains:
            for jn in urdf.chain_joints(base, tip):
                add(jn)
        for sub in grp.subgroups:
            for jn in self.resolve_group_joints(sub, urdf, seen_groups):
                add(jn)

        return ordered

    def validate(self, urdf: UrdfModel) -> List[str]:
        """Cross-check every reference against the URDF.

        This is the check that matters most for this format: an SRDF is
        *all* references, so a name that doesn't resolve is a silent
        no-op rather than an error at load time. A group_state naming a
        renamed joint simply stops moving it, with nothing in any log.
        """
        problems: List[str] = []
        joint_names = set(urdf.joints)
        link_names = urdf.link_set
        group_names = {g.name for g in self.groups}

        if self.robot_name and urdf.robot_name and self.robot_name != urdf.robot_name:
            problems.append(
                f"SRDF <robot name=\"{self.robot_name}\"> does not match URDF "
                f"<robot name=\"{urdf.robot_name}\">; MoveIt will not bind them")

        for g in self.groups:
            for jn in g.joints:
                if jn not in joint_names:
                    problems.append(f"group '{g.name}': unknown joint '{jn}'")
            for ln in g.links:
                if ln not in link_names:
                    problems.append(f"group '{g.name}': unknown link '{ln}'")
            for base, tip in g.chains:
                if base not in link_names:
                    problems.append(f"group '{g.name}': unknown chain base_link '{base}'")
                elif tip not in link_names:
                    problems.append(f"group '{g.name}': unknown chain tip_link '{tip}'")
                elif not urdf.chain_joints(base, tip):
                    problems.append(
                        f"group '{g.name}': no path from '{base}' to '{tip}' "
                        f"(is tip_link a descendant of base_link?)")
            for sg in g.subgroups:
                if sg not in group_names:
                    problems.append(f"group '{g.name}': unknown subgroup '{sg}'")

        for gs in self.group_states:
            if gs.group and gs.group not in group_names:
                problems.append(
                    f"group_state '{gs.name}': unknown group '{gs.group}'")
            for jn in gs.joint_values:
                j = urdf.joint(jn)
                if j is None:
                    problems.append(
                        f"group_state '{gs.name}': unknown joint '{jn}'")
                elif not j.actuatable:
                    problems.append(
                        f"group_state '{gs.name}': joint '{jn}' is "
                        f"{j.type or 'untyped'} and accepts no value")

        for jn in self.passive_joints:
            if jn not in joint_names:
                problems.append(f"passive_joint: unknown joint '{jn}'")

        for vj in self.virtual_joints:
            if vj.child_link and vj.child_link not in link_names:
                problems.append(
                    f"virtual_joint '{vj.name}': unknown child_link '{vj.child_link}'")

        for ee in self.end_effectors:
            if ee.group and ee.group not in group_names:
                problems.append(f"end_effector '{ee.name}': unknown group '{ee.group}'")
            if ee.parent_link and ee.parent_link not in link_names:
                problems.append(
                    f"end_effector '{ee.name}': unknown parent_link '{ee.parent_link}'")

        return problems

    def summary(self) -> Dict[str, Any]:
        """Compact shape for metadata + the settings UI."""
        return {
            "robot_name": self.robot_name,
            "group_count": len(self.groups),
            "group_state_count": len(self.group_states),
            "passive_joint_count": len(self.passive_joints),
            "virtual_joint_count": len(self.virtual_joints),
            "disabled_collision_count": len(self.disabled_collisions),
            "groups": [g.name for g in self.groups],
            "group_states": [
                {"name": gs.name, "group": gs.group, "joint_count": len(gs.joint_values)}
                for gs in self.group_states
            ],
        }


def looks_like_srdf(xml_bytes: bytes) -> bool:
    """True if these bytes are an SRDF rather than a URDF.

    Both formats use ``<robot>`` as their root, so the root tag proves
    nothing. The discriminator is the presence of an SRDF-only child
    element. Checked before falling back to file extension, because a
    bundle may ship ``robot.srdf.xacro`` or an oddly-named file and the
    contents are more trustworthy than the name.
    """
    try:
        root = ET.fromstring(xml_bytes)
    except ET.ParseError:
        return False
    if root.tag != "robot":
        return False
    for child in root:
        if child.tag in _SRDF_ONLY_TAGS:
            return True
    return False
