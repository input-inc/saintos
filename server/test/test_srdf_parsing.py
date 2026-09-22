"""URDF introspection + SRDF parsing/validation.

Covers the three things that make this layer load-bearing:

  * **Unit conversion.** SRDF group_state values are URDF-native
    (radians / metres); every sink downstream speaks normalized −1..+1.
    A missed conversion drives a servo to its stop, so the round-trip
    and the asymmetric-range case are both pinned here.
  * **Reference resolution.** An SRDF is *all* references. A name that
    doesn't resolve is a silent no-op, not a load error — so validate()
    has to catch it, and these tests assert it does.
  * **Group expansion.** A group can name joints, links, chains, or
    subgroups; all four have to produce the same kind of joint list.
"""
from __future__ import annotations

import math

import pytest

from saint_server.animation.srdf import SRDF, SRDFError, looks_like_srdf
from saint_server.animation.urdf_model import (
    CONTINUOUS_HALF_RANGE,
    UrdfModel,
)


# A trimmed animatronic head: neck → head → eyes/brow, one fixed joint
# acting as a control anchor frame, and one mimic coupling.
URDF_XML = b"""<?xml version="1.0"?>
<robot name="johnny5">
  <link name="base"/>
  <link name="neck"/>
  <link name="head"/>
  <link name="eye_l"/>
  <link name="eye_r"/>
  <link name="brow_l"/>
  <link name="ctrl_lookat"/>

  <joint name="neck_yaw" type="revolute">
    <parent link="base"/><child link="neck"/>
    <limit lower="-1.57" upper="1.57" velocity="2" effort="10"/>
  </joint>
  <joint name="head_pitch" type="revolute">
    <parent link="neck"/><child link="head"/>
    <limit lower="-0.5" upper="0.7" velocity="2" effort="10"/>
  </joint>
  <joint name="eye_l_pan" type="revolute">
    <parent link="head"/><child link="eye_l"/>
    <limit lower="-0.4" upper="0.4" velocity="5" effort="1"/>
  </joint>
  <joint name="eye_r_pan" type="revolute">
    <parent link="head"/><child link="eye_r"/>
    <limit lower="-0.4" upper="0.4" velocity="5" effort="1"/>
    <mimic joint="eye_l_pan" multiplier="1.0" offset="0"/>
  </joint>
  <joint name="brow_l_lift" type="revolute">
    <parent link="head"/><child link="brow_l"/>
    <limit lower="0" upper="1.0" velocity="5" effort="1"/>
  </joint>
  <joint name="lookat_mount" type="fixed">
    <parent link="head"/><child link="ctrl_lookat"/>
  </joint>
</robot>
"""

SRDF_XML = b"""<?xml version="1.0"?>
<robot name="johnny5">
  <group name="head_grp"><chain base_link="base" tip_link="head"/></group>
  <group name="eyes">
    <joint name="eye_l_pan"/>
    <joint name="eye_r_pan"/>
  </group>
  <group name="brows"><link name="brow_l"/></group>
  <group name="face">
    <group name="eyes"/>
    <group name="brows"/>
  </group>

  <group_state name="neutral" group="face">
    <joint name="eye_l_pan" value="0"/>
    <joint name="brow_l_lift" value="0"/>
  </group_state>
  <group_state name="happy" group="face">
    <joint name="brow_l_lift" value="0.8"/>
    <joint name="eye_l_pan" value="0.1"/>
  </group_state>

  <passive_joint name="eye_r_pan"/>
  <virtual_joint name="world_joint" type="fixed"
                 parent_frame="world" child_link="base"/>
  <disable_collisions link1="eye_l" link2="brow_l" reason="Adjacent"/>
  <disable_collisions link1="eye_r" link2="brow_l" reason="Never"/>
</robot>
"""


@pytest.fixture
def urdf() -> UrdfModel:
    return UrdfModel.parse(URDF_XML)


@pytest.fixture
def srdf() -> SRDF:
    return SRDF.parse(SRDF_XML)


# ── URDF introspection ─────────────────────────────────────────────


def test_parses_names_and_tree(urdf):
    assert urdf.robot_name == "johnny5"
    assert len(urdf.links) == 7
    assert len(urdf.joints) == 6


def test_fixed_joints_are_not_actuatable(urdf):
    """A control anchor frame is a massless link on a fixed joint — it
    must never show up as something an operator can key."""
    names = {j.name for j in urdf.actuatable_joints()}
    assert "lookat_mount" not in names
    assert names == {"neck_yaw", "head_pitch", "eye_l_pan",
                     "eye_r_pan", "brow_l_lift"}


def test_chain_expansion_walks_up_the_tree(urdf):
    assert urdf.chain_joints("base", "head") == ["neck_yaw", "head_pitch"]


def test_chain_expansion_is_directional(urdf):
    """tip must be a descendant of base; the reverse is not a chain."""
    assert urdf.chain_joints("head", "base") == []


def test_chain_expansion_rejects_unconnected_links(urdf):
    assert urdf.chain_joints("eye_l", "brow_l") == []


def test_subtree_collects_descendants(urdf):
    assert set(urdf.subtree_joints("head")) == {
        "eye_l_pan", "eye_r_pan", "brow_l_lift", "lookat_mount"}


def test_mimic_is_parsed(urdf):
    m = urdf.joint("eye_r_pan").mimic
    assert m is not None
    assert (m.joint, m.multiplier, m.offset) == ("eye_l_pan", 1.0, 0.0)


def test_zero_multiplier_mimic_survives():
    """`multiplier="0"` means "hold at the offset". Defaulting it to 1.0
    would silently couple a joint authored to stay put."""
    xml = b"""<robot name="r">
      <link name="a"/><link name="b"/>
      <joint name="master" type="revolute"><parent link="a"/><child link="b"/>
        <limit lower="-1" upper="1"/></joint>
      <joint name="held" type="revolute"><parent link="a"/><child link="b"/>
        <limit lower="-1" upper="1"/>
        <mimic joint="master" multiplier="0" offset="0.25"/></joint>
    </robot>"""
    m = UrdfModel.parse(xml).joint("held").mimic
    assert m.multiplier == 0.0
    assert m.offset == 0.25


# ── unit conversion ────────────────────────────────────────────────


def test_normalize_maps_limits_to_plus_minus_one(urdf):
    j = urdf.joint("neck_yaw")
    assert j.normalize(-1.57) == pytest.approx(-1.0)
    assert j.normalize(1.57) == pytest.approx(1.0)
    assert j.normalize(0.0) == pytest.approx(0.0)


def test_normalize_on_asymmetric_range_puts_native_zero_off_centre(urdf):
    """head_pitch travels -0.5..0.7, so native 0 is NOT normalized 0 —
    normalized zero means the midpoint of travel. This is the convention
    the rest of the stack uses for peripheral channels, and getting it
    backwards is a subtle way to make every imported pose lean."""
    j = urdf.joint("head_pitch")
    assert j.normalize(-0.5) == pytest.approx(-1.0)
    assert j.normalize(0.7) == pytest.approx(1.0)
    assert j.normalize(0.1) == pytest.approx(0.0)      # midpoint of travel
    assert j.normalize(0.0) == pytest.approx(-1.0 / 6.0)


def test_normalize_denormalize_round_trips(urdf):
    for name in ("neck_yaw", "head_pitch", "brow_l_lift"):
        j = urdf.joint(name)
        lo, hi = j.range()
        for native in (lo, (lo + hi) / 2, hi, lo + (hi - lo) * 0.31):
            assert j.denormalize(j.normalize(native)) == pytest.approx(native)


def test_normalize_clamps_out_of_range(urdf):
    j = urdf.joint("brow_l_lift")          # 0 .. 1.0
    assert j.normalize(5.0) == pytest.approx(1.0)
    assert j.normalize(-5.0) == pytest.approx(-1.0)


def test_continuous_joint_falls_back_to_pi(urdf):
    xml = b"""<robot name="r">
      <link name="a"/><link name="b"/>
      <joint name="spin" type="continuous">
        <parent link="a"/><child link="b"/></joint>
    </robot>"""
    j = UrdfModel.parse(xml).joint("spin")
    assert j.range() == (-CONTINUOUS_HALF_RANGE, CONTINUOUS_HALF_RANGE)
    assert j.normalize(math.pi) == pytest.approx(1.0)


def test_normalize_returns_none_for_unknown_joint(urdf):
    """None, not a passthrough. An unconverted radian value reaching a
    −1..+1 sink would command a servo straight to its stop."""
    assert urdf.normalize("no_such_joint", 0.5) is None
    assert urdf.denormalize("no_such_joint", 0.5) is None


def test_urdf_validate_flags_missing_limits():
    xml = b"""<robot name="r">
      <link name="a"/><link name="b"/>
      <joint name="unlimited" type="revolute">
        <parent link="a"/><child link="b"/></joint>
    </robot>"""
    problems = UrdfModel.parse(xml).validate()
    assert any("unlimited" in p and "<limit>" in p for p in problems)


def test_urdf_validate_clean_model_is_silent(urdf):
    assert urdf.validate() == []


# ── SRDF / URDF discrimination ─────────────────────────────────────


def test_srdf_and_urdf_share_a_root_tag_so_contents_decide():
    """Both formats root at <robot>, so the tag proves nothing. The
    upload path relies on this discriminator to route a file."""
    assert looks_like_srdf(SRDF_XML) is True
    assert looks_like_srdf(URDF_XML) is False


def test_looks_like_srdf_rejects_junk():
    assert looks_like_srdf(b"not xml at all") is False
    assert looks_like_srdf(b"<sdf version='1.11'><model/></sdf>") is False


def test_parse_rejects_non_robot_root():
    with pytest.raises(SRDFError):
        SRDF.parse(b"<sdf version='1.11'><model name='x'/></sdf>")


def test_parse_rejects_malformed_xml():
    with pytest.raises(SRDFError):
        SRDF.parse(b"<robot name='x'><group>")


# ── SRDF content ───────────────────────────────────────────────────


def test_parses_groups_and_states(srdf):
    assert {g.name for g in srdf.groups} == {"head_grp", "eyes", "brows", "face"}
    assert {gs.name for gs in srdf.group_states} == {"neutral", "happy"}
    assert srdf.passive_joints == ["eye_r_pan"]
    assert len(srdf.disabled_collisions) == 2
    assert srdf.disabled_collisions[0] == ("eye_l", "brow_l", "Adjacent")
    assert srdf.virtual_joints[0].parent_frame == "world"


def test_group_state_values_stay_native(srdf):
    """Parsing must NOT convert. The values are radians here; conversion
    happens explicitly, with the URDF in hand."""
    happy = srdf.group_state("happy")
    assert happy.joint_values == {"brow_l_lift": 0.8, "eye_l_pan": 0.1}


def test_group_state_normalization(srdf, urdf):
    happy = srdf.group_state("happy")
    norm, unresolved = happy.normalized_values(urdf)
    assert unresolved == []
    # brow_l_lift travels 0..1.0, so 0.8 → 0.6 normalized.
    assert norm["brow_l_lift"] == pytest.approx(0.6)
    # eye_l_pan travels ±0.4, so 0.1 → 0.25.
    assert norm["eye_l_pan"] == pytest.approx(0.25)


def test_group_state_normalization_reports_unresolved(urdf):
    srdf = SRDF.parse(b"""<robot name="johnny5">
      <group_state name="ghosty" group="">
        <joint name="eye_l_pan" value="0.2"/>
        <joint name="renamed_away" value="0.9"/>
      </group_state></robot>""")
    norm, unresolved = srdf.group_state("ghosty").normalized_values(urdf)
    assert unresolved == ["renamed_away"]
    assert "renamed_away" not in norm      # dropped, not passed through
    assert "eye_l_pan" in norm


def test_multi_dof_group_state_value_takes_first(urdf):
    srdf = SRDF.parse(b"""<robot name="johnny5">
      <group_state name="multi" group="">
        <joint name="eye_l_pan" value="0.2 0.3 0.4"/>
      </group_state></robot>""")
    assert srdf.group_state("multi").joint_values == {"eye_l_pan": 0.2}


def test_group_state_lookup_can_scope_to_group():
    """SRDF only requires state names unique *within* a group, so two
    groups may both define "open"."""
    srdf = SRDF.parse(b"""<robot name="r">
      <group name="a"/><group name="b"/>
      <group_state name="open" group="a"><joint name="ja" value="1"/></group_state>
      <group_state name="open" group="b"><joint name="jb" value="2"/></group_state>
    </robot>""")
    assert srdf.group_state("open", "b").joint_values == {"jb": 2.0}
    assert srdf.group_state("open", "a").joint_values == {"ja": 1.0}


# ── group expansion ────────────────────────────────────────────────


def test_expand_group_from_explicit_joints(srdf, urdf):
    assert srdf.resolve_group_joints("eyes", urdf) == ["eye_l_pan", "eye_r_pan"]


def test_expand_group_from_chain(srdf, urdf):
    assert srdf.resolve_group_joints("head_grp", urdf) == ["neck_yaw", "head_pitch"]


def test_expand_group_from_link_uses_its_parent_joint(srdf, urdf):
    assert srdf.resolve_group_joints("brows", urdf) == ["brow_l_lift"]


def test_expand_group_recurses_into_subgroups(srdf, urdf):
    assert srdf.resolve_group_joints("face", urdf) == [
        "eye_l_pan", "eye_r_pan", "brow_l_lift"]


def test_expand_group_filters_non_actuatable(urdf):
    """A group legitimately contains fixed joints — they're how anchor
    frames attach — but they accept no setpoint."""
    srdf = SRDF.parse(b"""<robot name="johnny5"><group name="g">
      <joint name="lookat_mount"/><joint name="eye_l_pan"/>
    </group></robot>""")
    assert srdf.resolve_group_joints("g", urdf) == ["eye_l_pan"]


def test_expand_group_survives_cyclic_subgroups(urdf):
    srdf = SRDF.parse(b"""<robot name="johnny5">
      <group name="a"><group name="b"/><joint name="eye_l_pan"/></group>
      <group name="b"><group name="a"/><joint name="brow_l_lift"/></group>
    </robot>""")
    assert set(srdf.resolve_group_joints("a", urdf)) == {"eye_l_pan", "brow_l_lift"}


def test_expand_unknown_group_is_empty(srdf, urdf):
    assert srdf.resolve_group_joints("nonexistent", urdf) == []


# ── validation ─────────────────────────────────────────────────────


def test_validate_clean_pair_is_silent(srdf, urdf):
    assert srdf.validate(urdf) == []


def test_validate_flags_robot_name_mismatch(urdf):
    srdf = SRDF.parse(b'<robot name="some_other_robot"><group name="g"/></robot>')
    assert any("does not match" in p for p in srdf.validate(urdf))


def test_validate_flags_unknown_joint_in_group_state(urdf):
    srdf = SRDF.parse(b"""<robot name="johnny5"><group_state name="s" group="">
      <joint name="renamed_away" value="1.0"/></group_state></robot>""")
    problems = srdf.validate(urdf)
    assert any("renamed_away" in p for p in problems)


def test_validate_flags_group_state_on_fixed_joint(urdf):
    srdf = SRDF.parse(b"""<robot name="johnny5"><group_state name="s" group="">
      <joint name="lookat_mount" value="1.0"/></group_state></robot>""")
    assert any("accepts no value" in p for p in srdf.validate(urdf))


def test_validate_flags_unknown_group_and_link_and_subgroup(urdf):
    srdf = SRDF.parse(b"""<robot name="johnny5">
      <group name="g">
        <joint name="ghost_joint"/>
        <link name="ghost_link"/>
        <group name="ghost_group"/>
      </group>
      <group_state name="s" group="ghost_grp"><joint name="eye_l_pan" value="0"/></group_state>
      <passive_joint name="ghost_passive"/>
    </robot>""")
    problems = " | ".join(srdf.validate(urdf))
    for missing in ("ghost_joint", "ghost_link", "ghost_group",
                    "ghost_grp", "ghost_passive"):
        assert missing in problems


def test_validate_flags_disconnected_chain(urdf):
    srdf = SRDF.parse(
        b'<robot name="johnny5"><group name="g">'
        b'<chain base_link="eye_l" tip_link="brow_l"/></group></robot>')
    assert any("no path from" in p for p in srdf.validate(urdf))


def test_summary_shape(srdf):
    s = srdf.summary()
    assert s["robot_name"] == "johnny5"
    assert s["group_count"] == 4
    assert s["group_state_count"] == 2
    assert s["disabled_collision_count"] == 2
    assert {gs["name"] for gs in s["group_states"]} == {"neutral", "happy"}
