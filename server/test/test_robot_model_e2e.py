"""End-to-end walk through the three-file workflow.

Each layer has its own unit tests; this one proves they connect, in the
order an operator actually does things:

    upload bundle → import group_states as poses → build an animation
    with a pose track → resolve a frame → play it → drive the rig

The integration bugs this catches are the ones unit tests structurally
can't: a units conversion applied twice or not at all, a pose id that
doesn't survive slugification, a resolver that can't find the poses the
player cached.
"""
from __future__ import annotations

import asyncio
import io
import zipfile

import pytest

from saint_server.animation.frame import make_pose_lookup, resolve_frame
from saint_server.animation.models import Animation
from saint_server.animation.robot_model_store import RobotModelStore
from saint_server.animation.store import AnimationStore, PoseStore
from saint_server.webserver.state_manager import StateManager


# brow travels 0..2.0 rad, so a group_state's 1.5 rad must land on
# normalized 0.5 — a doubled or skipped conversion shows up immediately.
URDF_XML = b"""<?xml version='1.0'?>
<robot name="e2e_head">
  <link name="base"/><link name="neck"/><link name="head"/><link name="brow"/>
  <joint name="neck_yaw" type="revolute">
    <parent link="base"/><child link="neck"/>
    <limit lower="-1.0" upper="1.0"/></joint>
  <joint name="head_pitch" type="revolute">
    <parent link="neck"/><child link="head"/>
    <limit lower="-1.0" upper="1.0"/></joint>
  <joint name="brow_lift" type="revolute">
    <parent link="head"/><child link="brow"/>
    <limit lower="0" upper="2.0"/></joint>
</robot>
"""

SRDF_XML = b"""<?xml version='1.0'?>
<robot name="e2e_head">
  <group name="face"><joint name="brow_lift"/></group>
  <group_state name="neutral" group="face">
    <joint name="brow_lift" value="1.0"/>
  </group_state>
  <group_state name="Very Happy" group="face">
    <joint name="brow_lift" value="1.5"/>
  </group_state>
</robot>
"""

RIG_XML = b"""<?xml version='1.0'?>
<rig xmlns="urn:saintos:rig:1.0" version="1.0" robot="e2e_head">
  <settings clamp="scale_back" neutral_pose="neutral"/>
  <control name="mood" kind="channel" label="Mood" group="Face">
    <target pose="very_happy" at="1" curve="linear"/>
  </control>
  <control name="head_nod" kind="channel" label="Nod" group="Head">
    <drive joint="head_pitch" scale="1.0"/>
    <drive joint="neck_yaw" scale="0.25"/>
  </control>
</rig>
"""


class FakeEvaluator:
    def __init__(self):
        self.joint_values = {}
        self.frames = 0

    def apply_animation_frame(self, joint_values, ws_values):
        self.frames += 1
        self.joint_values.update(joint_values or {})
        return True

    def set_urdf_joint_value(self, joint, value):
        self.joint_values[joint] = value
        return True

    def set_ws_input(self, *_):
        return True


def bundle(*members) -> bytes:
    buf = io.BytesIO()
    with zipfile.ZipFile(buf, "w") as zf:
        for name, data in members:
            zf.writestr(name, data)
    return buf.getvalue()


@pytest.fixture
def sm(tmp_path):
    manager = StateManager(server_name="TEST", config_dir=str(tmp_path))
    manager.robot_store = RobotModelStore(config_dir=str(tmp_path))
    manager.pose_store = PoseStore(str(tmp_path))
    manager.animation_store = AnimationStore(str(tmp_path))
    manager._routing_evaluator = FakeEvaluator()
    return manager


def test_full_workflow(sm):
    # ── 1. One upload installs all three files ─────────────────────
    sm.robot_store.install_from_zip(
        bundle(("robot.urdf", URDF_XML),
               ("robot.srdf", SRDF_XML),
               ("robot.rig.xml", RIG_XML)),
        "head.zip")
    described = sm.robot_store.describe()
    assert described["installed"] is True
    assert described["srdf"]["group_state_count"] == 2
    assert described["rig"]["control_count"] == 2

    # The rig references pose "very_happy" — the SLUG of the SRDF's
    # "Very Happy" group_state. That resolves even before the import,
    # because the SRDF defines it and importing is what materialises it.
    # Flagging it as broken here would train the operator to ignore the
    # warning list.
    assert not [w for w in described["warnings"] if "very_happy" in w], \
        described["warnings"]

    # ── 2. Import the group_states as poses ────────────────────────
    result = sm.import_group_states()
    assert {p["name"] for p in result["imported"]} == {"neutral", "Very Happy"}

    # A human-readable group_state name becomes a slug id, and the rig
    # has to be able to find it under that slug.
    assert sm.pose_store.get("very_happy") is not None
    # PoseStore slugifies on lookup, so the authored spelling works too —
    # which is why validation accepts either form.
    assert sm.pose_store.get("Very Happy") is not None

    # Conversion happened exactly once: 1.5 rad on a 0..2.0 joint → 0.5.
    assert sm.pose_store.get("very_happy").joint_values()["brow_lift"] == \
        pytest.approx(0.5)
    # And the neutral pose: 1.0 rad is the midpoint of 0..2.0 → 0.0.
    assert sm.pose_store.get("neutral").joint_values()["brow_lift"] == \
        pytest.approx(0.0)

    # With the poses in place, the rig's reference now resolves.
    assert not [w for w in sm.robot_store.validate() if "very_happy" in w]

    # ── 3. The rig drives joints from control values ───────────────
    rig_frame = sm.evaluate_rig({"mood": 1.0, "head_nod": 0.5}, apply=True)
    assert rig_frame["joints"]["brow_lift"] == pytest.approx(0.5)
    assert rig_frame["joints"]["head_pitch"] == pytest.approx(0.5)
    assert rig_frame["joints"]["neck_yaw"] == pytest.approx(0.125)
    # One batched frame, not one call per joint.
    assert sm._routing_evaluator.frames == 1

    # ── 4. Build an animation that layers the pose ─────────────────
    # A pose track fading in over 1s, with a joint track ABOVE it that
    # pins neck_yaw — so the layering order is exercised, not just the
    # pose blend.
    saved = sm.save_animation({
        "id": "", "name": "nod hello", "duration": 1.0, "fps": 30,
        "value_tracks": [
            {"id": "pose.very_happy", "name": "Very Happy",
             "target_kind": "pose", "target": ["very_happy"],
             "curve": {"name": "w", "keys": [
                 {"time": 0.0, "value": 0.0, "interp": 1},
                 {"time": 1.0, "value": 1.0, "interp": 1}]}},
            {"id": "neck_yaw", "name": "neck_yaw",
             "target_kind": "urdf_joint", "target": [],
             "curve": {"name": "n", "keys": [
                 {"time": 0.0, "value": -1.0, "interp": 1}]}},
        ],
    })
    assert saved["success"] is True
    anim = sm.animation_store.get(saved["animation"]["id"])
    assert anim is not None

    # ── 5. Resolving a frame uses the imported pose ────────────────
    lookup = make_pose_lookup(sm.pose_store)
    neutral = sm.rig_neutral()

    at_start, _ = resolve_frame(anim, 0.0, lookup, neutral)
    # Weight 0 contributes nothing, so the brow sits at neutral...
    assert at_start.get("brow_lift", 0.0) == pytest.approx(0.0)
    # ...and the joint track pins neck_yaw regardless.
    assert at_start["neck_yaw"] == pytest.approx(-1.0)

    at_half, _ = resolve_frame(anim, 0.5, lookup, neutral)
    assert at_half["brow_lift"] == pytest.approx(0.25)   # 0.0 → 0.5 at 50%

    at_end, _ = resolve_frame(anim, 1.0, lookup, neutral)
    assert at_end["brow_lift"] == pytest.approx(0.5)
    assert at_end["neck_yaw"] == pytest.approx(-1.0)


def test_player_drives_a_pose_track(sm):
    """The player has to reach the pose library through its own cached
    lookup — a resolver that works in a test harness but can't find
    poses at playback time would be invisible until a live performance.
    """
    sm.robot_store.install_from_zip(
        bundle(("robot.urdf", URDF_XML), ("robot.srdf", SRDF_XML),
               ("robot.rig.xml", RIG_XML)),
        "head.zip")
    sm.import_group_states()

    from saint_server.animation.player import AnimationPlayer

    anim = Animation.from_dict({
        "id": "a", "name": "a", "duration": 0.05, "fps": 60,
        "value_tracks": [
            {"id": "pose.very_happy", "name": "Very Happy",
             "target_kind": "pose", "target": ["very_happy"],
             "curve": {"name": "w", "keys": [
                 {"time": 0.0, "value": 1.0, "interp": 1}]}},
        ],
    })
    ev = sm._routing_evaluator

    async def run():
        player = AnimationPlayer(
            anim,
            set_urdf_joint_value=ev.set_urdf_joint_value,
            set_ws_input=ev.set_ws_input,
            set_topic_channel=lambda *a, **k: {},
            estop_active=lambda: False,
            apply_frame=ev.apply_animation_frame,
            pose_lookup=make_pose_lookup(sm.pose_store),
            neutral=sm.rig_neutral(),
        )
        await player.start()
        # Let it run to completion (duration is 50 ms).
        for _ in range(50):
            await asyncio.sleep(0.01)
            if not player.is_running:
                break
        return player

    player = asyncio.new_event_loop().run_until_complete(run())
    assert player is not None
    assert ev.joint_values["brow_lift"] == pytest.approx(0.5)


def test_srdf_reupload_keeps_hand_tuned_poses(sm):
    """The workflow that would silently destroy work if it went wrong:
    import, tune by hand, then re-upload a revised SRDF."""
    sm.robot_store.install_from_zip(
        bundle(("robot.urdf", URDF_XML), ("robot.srdf", SRDF_XML)), "head.zip")
    sm.import_group_states()

    tuned = sm.pose_store.get("very_happy")
    tuned.setpoints[0].value = 0.9
    sm.pose_store.save(tuned)

    # Revised SRDF: same states, different authored value.
    revised = SRDF_XML.replace(b'value="1.5"', b'value="0.4"')
    sm.robot_store.install_srdf(revised, "robot.srdf")

    # Re-import without overwrite leaves the tuned value alone.
    result = sm.import_group_states()
    assert result["imported"] == []
    assert sm.pose_store.get("very_happy").joint_values()["brow_lift"] == \
        pytest.approx(0.9)

    # With overwrite, it re-derives from the new SRDF: 0.4 on a 0..2.0
    # joint → normalized -0.6.
    sm.import_group_states(overwrite=True)
    assert sm.pose_store.get("very_happy").joint_values()["brow_lift"] == \
        pytest.approx(-0.6)


def test_urdf_replacement_surfaces_broken_rig_references(sm):
    """Re-exporting geometry with renamed joints must not silently
    disable half the rig."""
    sm.robot_store.install_from_zip(
        bundle(("robot.urdf", URDF_XML), ("robot.srdf", SRDF_XML),
               ("robot.rig.xml", RIG_XML)), "head.zip")
    sm.import_group_states()
    assert not [w for w in sm.robot_store.validate() if "head_pitch" in w]

    renamed = URDF_XML.replace(b'"head_pitch"', b'"head_tilt"')
    sm.robot_store.install_from_urdf(renamed, "v2.urdf")

    # The rig survived the replacement...
    assert sm.robot_store.get_metadata().rig_filename == "robot.rig.xml"
    # ...and its now-dangling reference is reported, not swallowed.
    warnings = sm.robot_store.validate()
    assert any("head_pitch" in w for w in warnings), warnings
