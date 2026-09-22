"""Rig file parsing, validation, and evaluation.

The evaluator is small but every part of it is a place a subtle error
hides silently: a blend that leans because deflection is measured from
the wrong origin, a clamp policy that breaks a face instead of
saturating it, a mimic applied in the wrong units. Each of those
produces plausible-looking motion, so they're pinned individually.
"""
from __future__ import annotations

import pytest

from saint_server.animation.rig import (
    CLAMP_POLICIES,
    RIG_VERSION,
    Rig,
    RigError,
    looks_like_rig,
)
from saint_server.animation.rig_eval import RigEvaluator
from saint_server.animation.srdf import SRDF
from saint_server.animation.urdf_model import UrdfModel


URDF_XML = b"""<?xml version="1.0"?>
<robot name="johnny5">
  <link name="base"/><link name="neck"/><link name="head"/>
  <link name="eye_l"/><link name="eye_r"/>
  <link name="brow_l"/><link name="brow_r"/>
  <link name="lid_l"/><link name="ctrl_lookat"/>

  <joint name="neck_yaw" type="revolute">
    <parent link="base"/><child link="neck"/>
    <limit lower="-1.0" upper="1.0"/></joint>
  <joint name="head_pitch" type="revolute">
    <parent link="neck"/><child link="head"/>
    <limit lower="-1.0" upper="1.0"/></joint>
  <joint name="eye_l_pan" type="revolute">
    <parent link="head"/><child link="eye_l"/>
    <limit lower="-0.4" upper="0.4"/></joint>
  <joint name="eye_l_tilt" type="revolute">
    <parent link="head"/><child link="eye_l"/>
    <limit lower="-0.3" upper="0.3"/></joint>
  <joint name="eye_r_pan" type="revolute">
    <parent link="head"/><child link="eye_r"/>
    <limit lower="-0.4" upper="0.4"/></joint>
  <joint name="brow_l_lift" type="revolute">
    <parent link="head"/><child link="brow_l"/>
    <limit lower="-1.0" upper="1.0"/></joint>
  <joint name="brow_r_lift" type="revolute">
    <parent link="head"/><child link="brow_r"/>
    <limit lower="-1.0" upper="1.0"/></joint>
  <!-- Different travel from its master on purpose: catches a mimic
       applied in normalized rather than native units. -->
  <joint name="lid_l_close" type="revolute">
    <parent link="head"/><child link="lid_l"/>
    <limit lower="-2.0" upper="2.0"/>
    <mimic joint="brow_l_lift" multiplier="0.5" offset="0"/></joint>
  <joint name="lookat_mount" type="fixed">
    <parent link="head"/><child link="ctrl_lookat"/></joint>
</robot>
"""

RIG_XML = b"""<?xml version="1.0"?>
<rig xmlns="urn:saintos:rig:1.0" version="1.0" robot="johnny5">
  <settings clamp="scale_back" neutral_pose="neutral"/>

  <control name="mood" kind="channel" label="Mood" group="Face" order="10"
           min="-1" max="1" default="0">
    <target pose="mad"   at="-1" curve="easeInOut"/>
    <target pose="happy" at="1"  curve="easeInOut"/>
  </control>

  <control name="head_nod" kind="channel" label="Nod" group="Head" order="20"
           min="-1" max="1" default="0">
    <drive joint="head_pitch" scale="1.0"  curve="linear"/>
    <drive joint="neck_yaw"   scale="0.25" curve="linear"/>
  </control>

  <control name="eye_look" kind="pad" label="Eye look" group="Head" order="30">
    <axis name="x" min="-1" max="1" default="0">
      <drive joint="eye_l_pan" scale="1.0"/>
      <drive joint="eye_r_pan" scale="1.0"/>
      <drive joint="neck_yaw"  scale="0.25"/>
    </axis>
    <axis name="y" min="-1" max="1" default="0">
      <drive joint="eye_l_tilt" scale="1.0"/>
    </axis>
    <widget kind="pad" invert_y="true"/>
  </control>

  <control name="look_at" kind="spatial" label="Look at" group="Head"
           anchor="ctrl_lookat">
    <gaze>
      <frame link="eye_l" axis="0 0 1" weight="1.0"/>
      <frame link="eye_r" axis="0 0 1" weight="1.0"/>
      <frame link="head"  axis="1 0 0" weight="0.2"/>
      <regularize joints="eye_l_pan eye_l_tilt eye_r_pan" weight="0.05"/>
    </gaze>
    <widget kind="gizmo" shape="sphere" scale="0.05"
            color="1 0.7 0 0.8" dofs="move_3d"/>
  </control>
</rig>
"""

# Poses are NORMALIZED −1..+1 — the conversion from SRDF radians
# happens once at the import boundary, not here.
POSES = {
    "neutral": {"brow_l_lift": 0.0, "brow_r_lift": 0.0},
    "happy":   {"brow_l_lift": 0.8, "brow_r_lift": 0.8},
    "mad":     {"brow_l_lift": -0.6, "brow_r_lift": -0.6},
}


@pytest.fixture
def urdf():
    return UrdfModel.parse(URDF_XML)


@pytest.fixture
def rig():
    return Rig.parse(RIG_XML)


@pytest.fixture
def ev(rig, urdf):
    return RigEvaluator(rig, urdf=urdf, poses=POSES)


# ── parsing ────────────────────────────────────────────────────────


def test_parses_controls(rig):
    assert [c.name for c in rig.controls] == [
        "mood", "head_nod", "eye_look", "look_at"]
    assert rig.robot == "johnny5"
    # The DECLARED version, not the parser's constant — a 1.0 file must
    # keep reporting 1.0 after the parser moves to 1.1, or nothing can
    # tell which schema a file was written against.
    assert rig.version == "1.0"
    assert rig.settings.clamp == "scale_back"
    assert rig.settings.neutral_pose == "neutral"


def test_namespace_is_optional():
    """Requiring xmlns would make every hand-authored file fail its
    first load for a reason the error can't easily explain."""
    bare = b"""<rig version="1.0"><control name="c" kind="channel">
      <drive joint="j"/></control></rig>"""
    assert [c.name for c in Rig.parse(bare).controls] == ["c"]


def test_curve_names_resolve_to_catalog_codes(rig):
    from saint_server.unreal.animation import CurveInterpolation
    mood = rig.control("mood")
    assert mood.targets[0].curve == int(CurveInterpolation.EASE_IN_OUT)
    # An omitted curve is linear, not "unset".
    assert rig.control("head_nod").drives[0].curve == int(CurveInterpolation.LINEAR)


def test_unknown_curve_name_is_an_error():
    with pytest.raises(RigError, match="not a known easing"):
        Rig.parse(b'<rig version="1.0"><control name="c"><drive joint="j" '
                  b'curve="easeInOutBanana"/></control></rig>')


def test_pad_axes_and_widget(rig):
    pad = rig.control("eye_look")
    assert [a.name for a in pad.axes] == ["x", "y"]
    assert [d.joint for d in pad.axis("x").drives] == [
        "eye_l_pan", "eye_r_pan", "neck_yaw"]
    assert pad.widget.invert_y is True


def test_gaze_binding_parsed(rig):
    gaze = rig.control("look_at").gaze
    assert [f.link for f in gaze.frames] == ["eye_l", "eye_r", "head"]
    assert gaze.frames[2].weight == pytest.approx(0.2)
    assert gaze.frames[0].axis == (0.0, 0.0, 1.0)
    assert gaze.regularizers[0].joints == ["eye_l_pan", "eye_l_tilt", "eye_r_pan"]


def test_widget_defaults_follow_control_kind():
    r = Rig.parse(b"""<rig version="1.0">
      <control name="a" kind="channel"><drive joint="j"/></control>
      <control name="b" kind="pad"><axis name="x"><drive joint="j"/></axis></control>
      <control name="c" kind="spatial" anchor="l"><gaze/></control>
    </rig>""")
    assert r.control("a").widget.kind == "slider"
    assert r.control("b").widget.kind == "pad"
    assert r.control("c").widget.kind == "gizmo"


def test_rejects_bad_root_and_duplicate_names():
    with pytest.raises(RigError, match="root element"):
        Rig.parse(b'<robot name="x"/>')
    with pytest.raises(RigError, match="duplicate control"):
        Rig.parse(b'<rig version="1.0"><control name="a"/>'
                  b'<control name="a"/></rig>')


def test_rejects_future_major_version():
    """An unknown control kind dropped in silence is worse than a load
    error an operator can actually see."""
    with pytest.raises(RigError, match="implements"):
        Rig.parse(b'<rig version="2.0"><control name="a"/></rig>')
    # Same major, higher minor is fine — additive changes stay loadable.
    assert Rig.parse(b'<rig version="1.7"><control name="a"><drive joint="j"/>'
                     b'</control></rig>').version == "1.7"


def test_rejects_unknown_clamp_policy():
    with pytest.raises(RigError, match="not a known policy"):
        Rig.parse(b'<rig version="1.0"><settings clamp="whatever"/></rig>')
    assert "scale_back" in CLAMP_POLICIES and "clamp" in CLAMP_POLICIES


def test_rejects_non_numeric_attributes():
    with pytest.raises(RigError, match="not a number"):
        Rig.parse(b'<rig version="1.0"><control name="a" min="lots">'
                  b'<drive joint="j"/></control></rig>')


def test_looks_like_rig_is_unambiguous():
    assert looks_like_rig(RIG_XML) is True
    assert looks_like_rig(b'<robot name="x"/>') is False
    assert looks_like_rig(b'not xml') is False


def test_controls_by_group_preserves_file_order_and_sorts_within(rig):
    groups = rig.controls_by_group()
    assert [g for g, _ in groups] == ["Face", "Head"]
    assert [c.name for c in groups[1][1]] == ["look_at", "head_nod", "eye_look"]


# ── validation ─────────────────────────────────────────────────────


def test_validate_clean_rig(rig, urdf):
    srdf = SRDF.parse(b'<robot name="johnny5"/>')
    assert rig.validate(urdf, srdf, pose_names=list(POSES)) == []


def test_validate_flags_unknown_joint_and_pose(urdf):
    r = Rig.parse(b"""<rig version="1.0" robot="johnny5">
      <control name="c" kind="channel">
        <drive joint="ghost_joint"/>
        <target pose="ghost_pose" at="1"/>
      </control></rig>""")
    problems = " | ".join(r.validate(urdf, pose_names=list(POSES)))
    assert "ghost_joint" in problems
    assert "ghost_pose" in problems


def test_validate_flags_drive_on_fixed_joint(urdf):
    r = Rig.parse(b'<rig version="1.0"><control name="c">'
                  b'<drive joint="lookat_mount"/></control></rig>')
    assert any("accepts no value" in p for p in r.validate(urdf))


def test_validate_flags_control_that_does_nothing(urdf):
    """A control that renders but has no binding is the most confusing
    possible outcome for an operator."""
    r = Rig.parse(b'<rig version="1.0"><control name="inert" kind="channel"/></rig>')
    assert any("does nothing" in p for p in r.validate(urdf))


def test_validate_flags_pad_without_axes_and_spatial_without_gaze(urdf):
    r = Rig.parse(b"""<rig version="1.0">
      <control name="p" kind="pad"/>
      <control name="s" kind="spatial"/>
    </rig>""")
    problems = " | ".join(r.validate(urdf))
    assert "needs <axis>" in problems
    assert "needs a <gaze> binding" in problems
    assert "needs anchor=" in problems


def test_validate_flags_default_outside_range(urdf):
    r = Rig.parse(b'<rig version="1.0"><control name="c" min="0" max="1" '
                  b'default="5"><drive joint="neck_yaw"/></control></rig>')
    assert any("falls outside" in p for p in r.validate(urdf))


def test_validate_flags_duplicate_target_stops(urdf):
    """Two targets at the same `at` make a zero-width blend segment."""
    r = Rig.parse(b'<rig version="1.0"><control name="c">'
                  b'<target pose="happy" at="1"/><target pose="mad" at="1"/>'
                  b'</control></rig>')
    assert any("same at=" in p for p in r.validate(urdf, pose_names=list(POSES)))


def test_validate_flags_robot_name_mismatch(urdf):
    r = Rig.parse(b'<rig version="1.0" robot="other"><control name="c">'
                  b'<drive joint="neck_yaw"/></control></rig>')
    assert any("does not match" in p for p in r.validate(urdf))


def test_validate_flags_unknown_neutral_pose(urdf):
    r = Rig.parse(b'<rig version="1.0"><settings neutral_pose="ghost"/></rig>')
    assert any("names no known pose" in p
               for p in r.validate(urdf, pose_names=list(POSES)))


# ── evaluation: rest state ─────────────────────────────────────────


def test_rest_state_is_neutral(ev):
    """A freshly loaded rig with no control values evaluates to neutral."""
    frame = ev.evaluate({})
    assert frame.joints["brow_l_lift"] == pytest.approx(0.0)
    assert frame.joints.get("head_pitch", 0.0) == pytest.approx(0.0)
    assert frame.clamped is False


def test_control_defaults_keys_pads_by_axis(ev):
    d = ev.control_defaults()
    assert d["mood"] == 0.0
    assert d["eye_look.x"] == 0.0 and d["eye_look.y"] == 0.0
    assert "eye_look" not in d


# ── evaluation: direct joint drives (head nod) ──────────────────────


def test_drive_scales_joints_independently(ev):
    frame = ev.evaluate({"head_nod": 1.0})
    assert frame.joints["head_pitch"] == pytest.approx(1.0)
    assert frame.joints["neck_yaw"] == pytest.approx(0.25)


def test_drive_is_signed_around_default(ev):
    frame = ev.evaluate({"head_nod": -1.0})
    assert frame.joints["head_pitch"] == pytest.approx(-1.0)
    assert frame.joints["neck_yaw"] == pytest.approx(-0.25)


def test_drive_is_proportional_mid_range(ev):
    frame = ev.evaluate({"head_nod": 0.5})
    assert frame.joints["head_pitch"] == pytest.approx(0.5)


def test_deflection_measured_from_default_not_from_min(urdf):
    """A 0..1 control resting at 0 must contribute nothing at rest. If
    deflection were measured from `min`, an asymmetric control would sit
    permanently half-applied — every pose built on it would lean."""
    r = Rig.parse(b'<rig version="1.0"><control name="c" min="0" max="1" '
                  b'default="0"><drive joint="neck_yaw" scale="1.0"/>'
                  b'</control></rig>')
    e = RigEvaluator(r, urdf=urdf)
    assert e.evaluate({"c": 0.0}).joints["neck_yaw"] == pytest.approx(0.0)
    assert e.evaluate({"c": 1.0}).joints["neck_yaw"] == pytest.approx(1.0)
    assert e.evaluate({"c": 0.5}).joints["neck_yaw"] == pytest.approx(0.5)


def test_asymmetric_control_reaches_full_scale_both_ways(urdf):
    r = Rig.parse(b'<rig version="1.0"><control name="c" min="-0.25" max="1" '
                  b'default="0"><drive joint="neck_yaw" scale="1.0"/>'
                  b'</control></rig>')
    e = RigEvaluator(r, urdf=urdf)
    assert e.evaluate({"c": 1.0}).joints["neck_yaw"] == pytest.approx(1.0)
    assert e.evaluate({"c": -0.25}).joints["neck_yaw"] == pytest.approx(-1.0)


def test_drive_curve_shapes_the_ramp(urdf):
    """Nonlinear response per joint is where the organic feel lives."""
    r = Rig.parse(b"""<rig version="1.0"><control name="c" min="-1" max="1" default="0">
      <drive joint="neck_yaw"   scale="1.0" curve="linear"/>
      <drive joint="head_pitch" scale="1.0" curve="easeInQuad"/>
    </control></rig>""")
    frame = RigEvaluator(r, urdf=urdf).evaluate({"c": 0.5})
    assert frame.joints["neck_yaw"] == pytest.approx(0.5)
    # easeInQuad(0.5) == 0.25 — the eased joint lags the linear one.
    assert frame.joints["head_pitch"] == pytest.approx(0.25)


# ── evaluation: pad controls (eye look) ────────────────────────────


def test_pad_axes_drive_independently(ev):
    frame = ev.evaluate({"eye_look.x": 1.0, "eye_look.y": -1.0})
    assert frame.joints["eye_l_pan"] == pytest.approx(1.0)
    assert frame.joints["eye_r_pan"] == pytest.approx(1.0)
    assert frame.joints["eye_l_tilt"] == pytest.approx(-1.0)
    # The x axis also feeds a little neck yaw, so the head follows.
    assert frame.joints["neck_yaw"] == pytest.approx(0.25)


def test_bare_pad_name_feeds_the_x_axis(ev):
    """Lets a 1D caller drive a pad without knowing it's 2D."""
    frame = ev.evaluate({"eye_look": 1.0})
    assert frame.joints["eye_l_pan"] == pytest.approx(1.0)
    assert frame.joints["eye_l_tilt"] == pytest.approx(0.0)


# ── evaluation: pose blending ──────────────────────────────────────


def test_pose_blend_reaches_the_target_at_its_stop(ev):
    frame = ev.evaluate({"mood": 1.0})
    assert frame.joints["brow_l_lift"] == pytest.approx(0.8)
    assert frame.joints["brow_r_lift"] == pytest.approx(0.8)


def test_pose_blend_reaches_the_opposite_target(ev):
    frame = ev.evaluate({"mood": -1.0})
    assert frame.joints["brow_l_lift"] == pytest.approx(-0.6)


def test_pose_blend_at_default_is_neutral(ev):
    assert ev.evaluate({"mood": 0.0}).joints["brow_l_lift"] == pytest.approx(0.0)


def test_pose_blend_midway_is_eased(ev):
    """The curve belongs to the target being approached, so each half of
    a ±1 slider can ease differently."""
    frame = ev.evaluate({"mood": 0.5})
    # easeInOut(0.5) == 0.5 exactly, so this lands on half of happy.
    assert frame.joints["brow_l_lift"] == pytest.approx(0.4)


def test_single_slider_cannot_activate_both_antagonists(ev):
    """happy at +1 and mad at −1 on one slider can never co-apply —
    the same reason riggers use one smile/frown slider instead of two."""
    positive = ev.evaluate({"mood": 0.5}).joints["brow_l_lift"]
    negative = ev.evaluate({"mood": -0.5}).joints["brow_l_lift"]
    assert positive > 0 and negative < 0


def test_pose_leaves_unnamed_joints_at_neutral(urdf):
    """A sparse pose means "don't care", not "centre it"."""
    poses = {"neutral": {"brow_l_lift": 0.5, "neck_yaw": 0.5},
             "wink": {"brow_l_lift": 1.0}}      # says nothing about neck_yaw
    r = Rig.parse(b'<rig version="1.0"><settings neutral_pose="neutral"/>'
                  b'<control name="c"><target pose="wink" at="1"/></control></rig>')
    frame = RigEvaluator(r, urdf=urdf, poses=poses).evaluate({"c": 1.0})
    assert frame.joints["brow_l_lift"] == pytest.approx(1.0)
    assert frame.joints["neck_yaw"] == pytest.approx(0.5)   # held, not zeroed


def test_blend_origin_is_the_neutral_pose_not_zero(urdf):
    poses = {"rest": {"brow_l_lift": 0.2}, "up": {"brow_l_lift": 1.0}}
    r = Rig.parse(b'<rig version="1.0"><settings neutral_pose="rest"/>'
                  b'<control name="c"><target pose="up" at="1"/></control></rig>')
    e = RigEvaluator(r, urdf=urdf, poses=poses)
    assert e.evaluate({"c": 0.0}).joints["brow_l_lift"] == pytest.approx(0.2)
    assert e.evaluate({"c": 0.5}).joints["brow_l_lift"] == pytest.approx(0.6)


# ── evaluation: composition ────────────────────────────────────────


def test_control_composition_is_commutative(urdf):
    """The whole reason for additive deltas: nobody should ever debug
    why the result changed when the UI reordered the sliders."""
    poses = {"neutral": {}, "a": {"brow_l_lift": 0.4},
             "b": {"brow_l_lift": 0.3}}
    forward = b"""<rig version="1.0"><settings neutral_pose="neutral"/>
      <control name="ca"><target pose="a" at="1"/></control>
      <control name="cb"><target pose="b" at="1"/></control></rig>"""
    reversed_ = b"""<rig version="1.0"><settings neutral_pose="neutral"/>
      <control name="cb"><target pose="b" at="1"/></control>
      <control name="ca"><target pose="a" at="1"/></control></rig>"""
    vals = {"ca": 1.0, "cb": 1.0}
    f1 = RigEvaluator(Rig.parse(forward), urdf=urdf, poses=poses).evaluate(vals)
    f2 = RigEvaluator(Rig.parse(reversed_), urdf=urdf, poses=poses).evaluate(vals)
    assert f1.joints["brow_l_lift"] == pytest.approx(f2.joints["brow_l_lift"])
    assert f1.joints["brow_l_lift"] == pytest.approx(0.7)


def test_contributions_attribute_each_joint_to_its_control(ev):
    """So that "the mouth is wrong" is answerable without guessing."""
    frame = ev.evaluate({"mood": 1.0, "head_nod": 1.0, "eye_look.x": 1.0})
    assert frame.contributions["mood"]["brow_l_lift"] == pytest.approx(0.8)
    assert frame.contributions["head_nod"]["head_pitch"] == pytest.approx(1.0)
    # neck_yaw is driven by two controls; each reports its own share.
    assert frame.contributions["head_nod"]["neck_yaw"] == pytest.approx(0.25)
    assert frame.contributions["eye_look"]["neck_yaw"] == pytest.approx(0.25)
    assert frame.joints["neck_yaw"] == pytest.approx(0.5)


# ── evaluation: clamp policy ───────────────────────────────────────


def test_scale_back_preserves_pose_shape(urdf):
    """Two controls summing past a limit scale back as a unit, so the
    pose saturates instead of breaking. brow wants 1.5, mouth wants
    0.75; scaling by 1/1.5 keeps their 2:1 ratio."""
    poses = {"neutral": {}, "big": {"brow_l_lift": 1.5, "brow_r_lift": 0.75}}
    r = Rig.parse(b'<rig version="1.0"><settings clamp="scale_back" '
                  b'neutral_pose="neutral"/><control name="c">'
                  b'<target pose="big" at="1"/></control></rig>')
    frame = RigEvaluator(r, urdf=urdf, poses=poses).evaluate({"c": 1.0})
    assert frame.clamped is True
    assert frame.scale_applied == pytest.approx(1 / 1.5)
    assert frame.joints["brow_l_lift"] == pytest.approx(1.0)
    assert frame.joints["brow_r_lift"] == pytest.approx(0.5)
    ratio = frame.joints["brow_l_lift"] / frame.joints["brow_r_lift"]
    assert ratio == pytest.approx(2.0)


def test_per_joint_clamp_breaks_the_ratio(urdf):
    """The contrast case: with clamp=clamp the brow pins while the other
    joint keeps travelling, so the shape is lost. Both policies exist
    because this is an authoring decision, not a detail."""
    poses = {"neutral": {}, "big": {"brow_l_lift": 1.5, "brow_r_lift": 0.75}}
    r = Rig.parse(b'<rig version="1.0"><settings clamp="clamp" '
                  b'neutral_pose="neutral"/><control name="c">'
                  b'<target pose="big" at="1"/></control></rig>')
    frame = RigEvaluator(r, urdf=urdf, poses=poses).evaluate({"c": 1.0})
    assert frame.clamped is True
    assert frame.joints["brow_l_lift"] == pytest.approx(1.0)
    assert frame.joints["brow_r_lift"] == pytest.approx(0.75)


def test_two_controls_at_full_can_exceed_and_scale_back(urdf):
    poses = {"neutral": {}, "a": {"brow_l_lift": 0.8},
             "b": {"brow_l_lift": 0.8}}
    r = Rig.parse(b"""<rig version="1.0"><settings clamp="scale_back"
        neutral_pose="neutral"/>
      <control name="ca"><target pose="a" at="1"/></control>
      <control name="cb"><target pose="b" at="1"/></control></rig>""")
    frame = RigEvaluator(r, urdf=urdf, poses=poses).evaluate(
        {"ca": 1.0, "cb": 1.0})
    assert frame.joints["brow_l_lift"] == pytest.approx(1.0)


def test_within_limits_leaves_frame_untouched(ev):
    frame = ev.evaluate({"mood": 1.0})
    assert frame.clamped is False
    assert frame.scale_applied == pytest.approx(1.0)


# ── evaluation: mimic ──────────────────────────────────────────────


def test_mimic_is_applied_in_native_units(ev):
    """lid_l_close mimics brow_l_lift at 0.5×, but travels ±2.0 against
    the brow's ±1.0. In native units: brow at 0.8 → 0.8 rad, lid →
    0.4 rad → 0.2 normalized. Applying the multiplier to the normalized
    value would have given 0.4 — double. This is exactly why mimic
    round-trips through the limits."""
    frame = ev.evaluate({"mood": 1.0})
    assert frame.joints["brow_l_lift"] == pytest.approx(0.8)
    assert frame.joints["lid_l_close"] == pytest.approx(0.2)


def test_mimic_overwrites_any_authored_value(urdf):
    """A mimicking joint has no independent value; letting a pose set
    one would just fight the coupling."""
    poses = {"neutral": {},
             "conflict": {"brow_l_lift": 0.8, "lid_l_close": -1.0}}
    r = Rig.parse(b'<rig version="1.0"><settings neutral_pose="neutral"/>'
                  b'<control name="c"><target pose="conflict" at="1"/>'
                  b'</control></rig>')
    frame = RigEvaluator(r, urdf=urdf, poses=poses).evaluate({"c": 1.0})
    assert frame.joints["lid_l_close"] == pytest.approx(0.2)


def test_mimic_offset_respected(urdf):
    xml = b"""<robot name="r">
      <link name="a"/><link name="b"/>
      <joint name="master" type="revolute"><parent link="a"/><child link="b"/>
        <limit lower="-1" upper="1"/></joint>
      <joint name="slave" type="revolute"><parent link="a"/><child link="b"/>
        <limit lower="-1" upper="1"/>
        <mimic joint="master" multiplier="1.0" offset="0.5"/></joint>
    </robot>"""
    u = UrdfModel.parse(xml)
    r = Rig.parse(b'<rig version="1.0"><control name="c">'
                  b'<drive joint="master" scale="0.5"/></control></rig>')
    frame = RigEvaluator(r, urdf=u).evaluate({"c": 1.0})
    assert frame.joints["master"] == pytest.approx(0.5)
    # native master 0.5 → slave 0.5*1.0 + 0.5 = 1.0 rad → normalized 1.0
    assert frame.joints["slave"] == pytest.approx(1.0)


# ── evaluation: unsupported bindings ───────────────────────────────


def test_gaze_control_is_reported_not_silently_ignored(ev):
    """The schema accepts it; the evaluator can't solve it yet. Reported
    STRUCTURED, because the viewport needs the control NAME to draw that
    handle as inert — an operator dragging a shape that does nothing is
    exactly the confusion this avoids."""
    frame = ev.evaluate({})
    skipped = {s["control"]: s["reason"] for s in frame.skipped}
    assert "look_at" in skipped
    assert "IK solver" in skipped["look_at"]


def test_driven_joints_covers_drives_and_poses(ev):
    driven = set(ev.driven_joints())
    assert {"head_pitch", "neck_yaw", "eye_l_pan", "eye_l_tilt"} <= driven
    # Reached only via a pose target, not a drive.
    assert "brow_l_lift" in driven


# ── docs / schema / parser agreement ───────────────────────────────
#
# Three artifacts describe this format: the prose spec, the XSD, and the
# parser. They drift silently — a doc example nobody runs is the classic
# way a schema and its documentation diverge — so the example in the docs
# is executed against both of the other two here.

import pathlib
import re
import shutil
import subprocess

_REPO = pathlib.Path(__file__).resolve().parents[2]
_DOC = _REPO / "docs" / "RIG_SCHEMA.md"
_XSD = _REPO / "server" / "resources" / "schema" / "rig-1.1.xsd"


def _doc_example() -> bytes:
    """The first complete rig document in docs/RIG_SCHEMA.md."""
    blocks = re.findall(r"```xml\n(.*?)```", _DOC.read_text(), re.S)
    for b in blocks:
        if b.lstrip().startswith("<?xml"):
            return b.strip().encode()
    raise AssertionError("no complete rig example found in docs/RIG_SCHEMA.md")


def test_doc_example_parses():
    rig = Rig.parse(_doc_example())
    assert [c.kind for c in rig.controls] == [
        "channel", "channel", "pad", "spatial"]
    assert rig.settings.clamp == "scale_back"


def test_doc_example_evaluates(urdf):
    """Not just parseable — the documented rig has to actually produce
    joint values, so the example can be copied and used as a starting
    point rather than being decorative."""
    rig = Rig.parse(_doc_example())
    # The doc example names a few joints this fixture URDF lacks
    # (neck_pitch, eye_r_tilt); evaluation is name-driven so that's fine.
    frame = RigEvaluator(rig, urdf=urdf, poses=POSES).evaluate(
        {"mood": 1.0, "head_nod": 0.5, "eye_look.x": 1.0})
    assert frame.joints["brow_l_lift"] == pytest.approx(0.8)
    assert frame.joints["eye_l_pan"] == pytest.approx(1.0)


@pytest.mark.skipif(shutil.which("xmllint") is None,
                    reason="xmllint not installed")
def test_doc_example_validates_against_the_xsd(tmp_path):
    path = tmp_path / "example.rig.xml"
    path.write_bytes(_doc_example())
    proc = subprocess.run(
        ["xmllint", "--noout", "--schema", str(_XSD), str(path)],
        capture_output=True, text=True)
    assert proc.returncode == 0, proc.stderr


@pytest.mark.skipif(shutil.which("xmllint") is None,
                    reason="xmllint not installed")
def test_xsd_accepts_order_free_bindings(tmp_path):
    """A <drive> after a <target> is valid; the parser collects by name,
    so the schema must not impose an order the parser doesn't."""
    path = tmp_path / "order.rig.xml"
    path.write_bytes(b"""<?xml version="1.0"?>
<rig xmlns="urn:saintos:rig:1.0" version="1.0">
  <control name="c" kind="channel">
    <target pose="happy" at="1"/>
    <drive joint="neck_yaw" scale="0.5"/>
    <target pose="mad" at="-1"/>
    <widget kind="slider"/>
  </control>
</rig>""")
    proc = subprocess.run(
        ["xmllint", "--noout", "--schema", str(_XSD), str(path)],
        capture_output=True, text=True)
    assert proc.returncode == 0, proc.stderr


# ── widget geometry (schema 1.1) ────────────────────────────────────
#
# A control shape sits at a real place on the robot with a real
# orientation and a real drag axis — that geometry is what turns an
# abstract scalar into something grabbable in the viewport.


def test_widget_geometry_parsed():
    r = Rig.parse(b"""<rig version="1.1"><control name="c" anchor="head">
      <drive joint="head_pitch"/>
      <widget shape="ring" scale="0.08 0.08 0.02" offset="0 0 0.1"
              rotation="1.5708 0 0" axis="0 1 0" color="0.1 0.2 0.3"
              dofs="rotate_axis" visible="true"/>
    </control></rig>""")
    w = r.control("c").widget
    assert w.shape == "ring"
    assert w.scale == pytest.approx((0.08, 0.08, 0.02))
    assert w.offset == pytest.approx((0.0, 0.0, 0.1))
    assert w.rotation[0] == pytest.approx(1.5708)
    assert w.axis == pytest.approx((0.0, 1.0, 0.0))
    # A 3-number colour fills alpha in rather than leaving it undefined.
    assert w.color == pytest.approx((0.1, 0.2, 0.3, 1.0))
    assert w.dofs == "rotate_axis"
    assert w.visible is True


def test_uniform_scale_expands_to_three_axes():
    r = Rig.parse(b'<rig version="1.1"><control name="c"><drive joint="j"/>'
                  b'<widget scale="0.2"/></control></rig>')
    assert r.control("c").widget.scale == pytest.approx((0.2, 0.2, 0.2))


def test_widget_shape_defaults_per_control_kind():
    """A control that says nothing about its appearance still gets
    something grabbable and appropriate."""
    r = Rig.parse(b"""<rig version="1.1">
      <control name="a" kind="channel"><drive joint="j"/></control>
      <control name="b" kind="pad"><axis name="x"><drive joint="j"/></axis></control>
      <control name="c" kind="spatial" anchor="l"><gaze/></control>
    </rig>""")
    assert r.control("a").widget.shape == "ring"     # 1-DOF reads as rotate
    assert r.control("b").widget.shape == "plane"    # 2-DOF reads as drag
    assert r.control("c").widget.shape == "sphere"   # a point in space


def test_unknown_shape_is_an_error():
    """Silently falling back to a sphere would leave an operator
    wondering why their arrow never appeared."""
    with pytest.raises(RigError, match="not a known shape"):
        Rig.parse(b'<rig version="1.1"><control name="c"><drive joint="j"/>'
                  b'<widget shape="dodecahedron"/></control></rig>')


def test_bad_vector_arity_is_an_error():
    with pytest.raises(RigError, match="needs 3 numbers"):
        Rig.parse(b'<rig version="1.1"><control name="c"><drive joint="j"/>'
                  b'<widget offset="0 1"/></control></rig>')
    with pytest.raises(RigError, match="needs 1 or 3 numbers"):
        Rig.parse(b'<rig version="1.1"><control name="c"><drive joint="j"/>'
                  b'<widget scale="1 2"/></control></rig>')


def test_widget_can_be_hidden_from_the_viewport():
    """A control only ever driven by an animation doesn't need a handle
    cluttering the view, but still wants its slider in the panel."""
    r = Rig.parse(b'<rig version="1.1"><control name="c"><drive joint="j"/>'
                  b'<widget visible="false"/></control></rig>')
    assert r.control("c").widget.visible is False


def test_widget_round_trips(rig):
    again = Rig.parse(_reserialize(rig))
    for name in ("mood", "eye_look", "look_at"):
        a, b = rig.control(name).widget, again.control(name).widget
        assert a.to_dict() == b.to_dict()


def _reserialize(rig_obj) -> bytes:
    """Re-emit a parsed rig as XML. Only the widget attributes we care
    about here — enough to prove they survive a round trip."""
    parts = [f'<rig version="{rig_obj.version}" robot="{rig_obj.robot}">']
    for c in rig_obj.controls:
        w = c.widget
        parts.append(
            f'<control name="{c.name}" kind="{c.kind}" min="{c.min}" '
            f'max="{c.max}" default="{c.default}" '
            f'anchor="{c.anchor}">')
        for d in c.drives:
            parts.append(f'<drive joint="{d.joint}" scale="{d.scale}"/>')
        for t in c.targets:
            parts.append(f'<target pose="{t.pose}" at="{t.at}"/>')
        for ax in c.axes:
            parts.append(f'<axis name="{ax.name}">')
            for d in ax.drives:
                parts.append(f'<drive joint="{d.joint}" scale="{d.scale}"/>')
            parts.append('</axis>')
        if c.gaze is not None:
            parts.append('<gaze/>')
        parts.append(
            f'<widget kind="{w.kind}" shape="{w.shape}" '
            f'scale="{" ".join(str(v) for v in w.scale)}" '
            f'color="{" ".join(str(v) for v in w.color)}" '
            f'offset="{" ".join(str(v) for v in w.offset)}" '
            f'rotation="{" ".join(str(v) for v in w.rotation)}" '
            f'axis="{" ".join(str(v) for v in w.axis)}" '
            f'dofs="{w.dofs}" invert_y="{str(w.invert_y).lower()}" '
            f'visible="{str(w.visible).lower()}"/>')
        parts.append('</control>')
    parts.append('</rig>')
    return "".join(parts).encode()


# ── anchor resolution ───────────────────────────────────────────────
#
# Without derivation, every rig file would need an explicit anchor on
# every control before anything appeared in the viewport, and a file
# written before widget geometry existed would silently draw nothing.


def test_explicit_anchor_wins(rig, urdf):
    assert rig.resolve_anchors(urdf)["look_at"] == "ctrl_lookat"


def test_anchor_derived_from_the_first_driven_joint(rig, urdf):
    """head_nod drives head_pitch first, whose child link is `head`."""
    assert rig.resolve_anchors(urdf)["head_nod"] == "head"


def test_pad_anchor_derived_from_its_axis_drives(rig, urdf):
    assert rig.resolve_anchors(urdf)["eye_look"] == "eye_l"


def test_pose_only_control_needs_pose_data_to_anchor(rig, urdf):
    """`mood` has no <drive> at all — only pose targets — so plain
    resolution can't place it and must omit rather than guess."""
    assert "mood" not in rig.resolve_anchors(urdf)
    with_poses = rig.resolve_anchors_with_poses(urdf, POSES)
    # happy/mad name brow_l_lift, whose child link is brow_l.
    assert with_poses["mood"] == "brow_l"


def test_unresolvable_control_is_omitted_not_defaulted(urdf):
    """Defaulting to the robot root would pile unrelated handles onto the
    base and read as a bug."""
    r = Rig.parse(b'<rig version="1.1"><control name="ghost">'
                  b'<drive joint="no_such_joint"/></control></rig>')
    assert r.resolve_anchors(urdf) == {}


def test_anchors_empty_without_a_urdf(rig):
    assert rig.resolve_anchors(None) == {}


# ── the shipped example files ──────────────────────────────────────
#
# server/resources/examples/ is what an operator copies to start from, so
# a broken example is a broken first experience. These also catch the
# XML-comment double-hyphen trap, which is easy to reintroduce when
# documenting a command line inside a comment.

_EXAMPLES = _REPO / "server" / "resources" / "examples"


def test_example_rig_parses_and_is_self_consistent():
    from saint_server.animation.srdf import SRDF as _SRDF
    rig = Rig.parse((_EXAMPLES / "example.rig.xml").read_bytes())
    srdf = _SRDF.parse((_EXAMPLES / "example.srdf").read_bytes())

    assert rig.robot == srdf.robot_name, \
        "the example rig and SRDF must name the same robot or nothing binds"
    # Every pose the rig blends toward has to exist in the SRDF, or the
    # example ships controls that do nothing.
    srdf_states = {gs.name for gs in srdf.group_states}
    assert set(rig.referenced_poses()) <= srdf_states
    assert rig.settings.neutral_pose in srdf_states


def test_every_example_control_resolves_an_anchor():
    """A control with no anchor draws NOTHING in the viewport and says
    nothing about why. That's the failure mode this catches — and it bit
    the example itself once, where `eye_look` derived onto the eye it
    controls instead of the head, so its offset landed in the wrong frame.
    """
    from saint_server.animation.srdf import SRDF as _SRDF
    rig = Rig.parse((_EXAMPLES / "example.rig.xml").read_bytes())
    urdf = UrdfModel.parse((_EXAMPLES / "example_head.urdf").read_bytes())
    srdf = _SRDF.parse((_EXAMPLES / "example.srdf").read_bytes())

    # Poses as the import would produce them: normalized, slug-keyed.
    poses = {}
    for gs in srdf.group_states:
        normalized, _ = gs.normalized_values(urdf)
        poses[gs.name] = normalized

    anchors = rig.resolve_anchors_with_poses(urdf, poses)
    missing = [c.name for c in rig.controls if c.name not in anchors]
    assert not missing, f"controls that would draw nothing: {missing}"
    # And every anchor has to be a real link, or the shape has no parent.
    for name, link in anchors.items():
        assert link in urdf.link_set, f"{name} anchored to unknown link {link}"


def test_example_pad_is_anchored_where_its_offset_expects():
    """Regression: without an explicit anchor this derived onto
    eye_l_pan_link, so the pad sat in the wrong place AND travelled with
    the eye it was supposed to aim."""
    rig = Rig.parse((_EXAMPLES / "example.rig.xml").read_bytes())
    urdf = UrdfModel.parse((_EXAMPLES / "example_head.urdf").read_bytes())
    assert rig.resolve_anchors(urdf)["eye_look"] == "head_link"


def test_example_rig_covers_the_documented_control_kinds():
    rig = Rig.parse((_EXAMPLES / "example.rig.xml").read_bytes())
    kinds = {c.kind for c in rig.controls}
    assert kinds == {"channel", "pad", "spatial"}
    # The features the request was actually about.
    assert rig.control("eye_look").kind == "pad"
    assert [d.joint for d in rig.control("head_nod").drives]
    assert [d.joint for d in rig.control("head_tilt").drives]


def test_example_srdf_group_expansion_needs_no_urdf_to_parse():
    from saint_server.animation.srdf import SRDF as _SRDF
    srdf = _SRDF.parse((_EXAMPLES / "example.srdf").read_bytes())
    assert {g.name for g in srdf.groups} >= {"eyes", "brows", "mouth", "face"}
    assert srdf.group_state("neutral") is not None
    assert srdf.passive_joints          # the broken-loop follower joints
    assert srdf.disabled_collisions


@pytest.mark.skipif(shutil.which("xmllint") is None,
                    reason="xmllint not installed")
def test_example_rig_validates_against_the_xsd():
    proc = subprocess.run(
        ["xmllint", "--noout", "--schema", str(_XSD),
         str(_EXAMPLES / "example.rig.xml")],
        capture_output=True, text=True)
    assert proc.returncode == 0, proc.stderr


@pytest.mark.skipif(shutil.which("xmllint") is None,
                    reason="xmllint not installed")
def test_xsd_rejects_a_bogus_curve_name(tmp_path):
    path = tmp_path / "bad.rig.xml"
    path.write_bytes(b"""<?xml version="1.0"?>
<rig xmlns="urn:saintos:rig:1.0" version="1.0">
  <control name="c"><drive joint="j" curve="easeInOutBanana"/></control>
</rig>""")
    proc = subprocess.run(
        ["xmllint", "--noout", "--schema", str(_XSD), str(path)],
        capture_output=True, text=True)
    assert proc.returncode != 0
