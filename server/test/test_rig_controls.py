"""The control-rig endpoints: get_rig and evaluate_rig.

The rig evaluates on the SERVER and hands the resolved joint values back
so the client can drive its 3D viewport from the same evaluation that
drives the robot. A JS port would be a second implementation of the
blend math, the clamp policy, and the mimic round-trip — three places to
drift — and a local round-trip is a millisecond, which a slider drag
doesn't notice. These tests pin that contract: what comes back is enough
to render the viewport without recomputing anything.
"""
from __future__ import annotations

import io
import zipfile

import pytest

from saint_server.animation.robot_model_store import RobotModelStore
from saint_server.animation.store import PoseStore
from saint_server.animation.models import Pose, PoseSetpoint
from saint_server.webserver.state_manager import StateManager


URDF_XML = b"""<?xml version='1.0'?>
<robot name="rig_test">
  <link name="base"/><link name="neck"/><link name="head"/>
  <link name="eye_l"/><link name="eye_r"/><link name="brow"/><link name="lid"/>
  <joint name="neck_yaw" type="revolute">
    <parent link="base"/><child link="neck"/>
    <limit lower="-1" upper="1"/></joint>
  <joint name="head_pitch" type="revolute">
    <parent link="neck"/><child link="head"/>
    <limit lower="-1" upper="1"/></joint>
  <joint name="eye_l_pan" type="revolute">
    <parent link="head"/><child link="eye_l"/>
    <limit lower="-1" upper="1"/></joint>
  <joint name="eye_l_tilt" type="revolute">
    <parent link="head"/><child link="eye_l"/>
    <limit lower="-1" upper="1"/></joint>
  <joint name="brow_lift" type="revolute">
    <parent link="head"/><child link="brow"/>
    <limit lower="-1" upper="1"/></joint>
  <joint name="lid_close" type="revolute">
    <parent link="head"/><child link="lid"/>
    <limit lower="-2" upper="2"/>
    <mimic joint="brow_lift" multiplier="0.5" offset="0"/></joint>
</robot>
"""

RIG_XML = b"""<?xml version='1.0'?>
<rig xmlns="urn:saintos:rig:1.0" version="1.0" robot="rig_test">
  <settings clamp="scale_back" neutral_pose="neutral"/>

  <control name="mood" kind="channel" label="Mood" group="Face" order="10"
           min="-1" max="1" default="0">
    <target pose="mad" at="-1" curve="easeInOut"/>
    <target pose="happy" at="1" curve="easeInOut"/>
    <widget kind="slider"/>
  </control>

  <control name="head_nod" kind="channel" label="Nod" group="Head" order="20">
    <drive joint="head_pitch" scale="1.0"/>
    <drive joint="neck_yaw" scale="0.25"/>
  </control>

  <control name="eye_look" kind="pad" label="Eye look" group="Head" order="30">
    <axis name="x"><drive joint="eye_l_pan" scale="1.0"/></axis>
    <axis name="y"><drive joint="eye_l_tilt" scale="1.0"/></axis>
    <widget kind="pad" invert_y="true"/>
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
    manager._routing_evaluator = FakeEvaluator()
    manager.robot_store.install_from_zip(
        bundle(("robot.urdf", URDF_XML), ("robot.rig.xml", RIG_XML)), "b.zip")
    for name, joints in (
        ("neutral", {"brow_lift": 0.0}),
        ("happy", {"brow_lift": 0.8}),
        ("mad", {"brow_lift": -0.6}),
    ):
        manager.pose_store.save(Pose(
            id="", name=name, setpoints=[
                PoseSetpoint(target_kind="joint", joint=j, value=v)
                for j, v in joints.items()]))
    return manager


# ── get_rig ────────────────────────────────────────────────────────


def test_get_rig_returns_the_full_definition(sm):
    r = sm.get_rig()
    assert r["success"] is True
    assert [c["name"] for c in r["rig"]["controls"]] == [
        "mood", "head_nod", "eye_look"]
    assert r["rig"]["settings"]["clamp"] == "scale_back"


def test_get_rig_includes_widget_hints(sm):
    """The evaluator never reads a widget — presentation is the client's
    business — so they have to reach the client somehow."""
    controls = {c["name"]: c for c in sm.get_rig()["rig"]["controls"]}
    assert controls["eye_look"]["widget"]["kind"] == "pad"
    assert controls["eye_look"]["widget"]["invert_y"] is True


def test_get_rig_defaults_key_pads_by_axis(sm):
    """A pad takes two scalars, so one flat key can't address it."""
    defaults = sm.get_rig()["defaults"]
    assert defaults["mood"] == 0.0
    assert defaults["eye_look.x"] == 0.0
    assert defaults["eye_look.y"] == 0.0
    assert "eye_look" not in defaults


def test_get_rig_reports_validation_warnings(sm):
    bad = RIG_XML.replace(b'joint="head_pitch"', b'joint="ghost_joint"')
    sm.robot_store.install_rig(bad, "robot.rig.xml")
    assert any("ghost_joint" in w for w in sm.get_rig()["warnings"])


def test_get_rig_without_a_rig_is_not_an_error(sm):
    sm.robot_store.delete_rig()
    r = sm.get_rig()
    assert r["success"] is True and r["rig"] is None


def test_get_rig_without_a_store_fails_cleanly(sm):
    sm.robot_store = None
    assert sm.get_rig()["success"] is False


# ── evaluate_rig ───────────────────────────────────────────────────


def test_evaluate_returns_joint_values(sm):
    r = sm.evaluate_rig({"head_nod": 1.0})
    assert r["success"] is True
    assert r["joints"]["head_pitch"] == pytest.approx(1.0)
    assert r["joints"]["neck_yaw"] == pytest.approx(0.25)


def test_evaluate_at_defaults_is_the_rest_pose(sm):
    r = sm.evaluate_rig({})
    assert r["joints"].get("head_pitch", 0.0) == pytest.approx(0.0)
    assert r["clamped"] is False


def test_evaluate_blends_a_pose_target(sm):
    r = sm.evaluate_rig({"mood": 1.0})
    assert r["joints"]["brow_lift"] == pytest.approx(0.8)


def test_evaluate_pad_axes_addressed_separately(sm):
    r = sm.evaluate_rig({"eye_look.x": 1.0, "eye_look.y": -0.5})
    assert r["joints"]["eye_l_pan"] == pytest.approx(1.0)
    assert r["joints"]["eye_l_tilt"] == pytest.approx(-0.5)


def test_evaluate_applies_mimic_in_native_units(sm):
    """lid_close mimics brow_lift at 0.5x but travels ±2 against the
    brow's ±1, so brow 0.8 → lid 0.2 normalized. Doing the multiply in
    normalized space would give 0.4 — double."""
    r = sm.evaluate_rig({"mood": 1.0})
    assert r["joints"]["lid_close"] == pytest.approx(0.2)


def test_evaluate_reports_contributions_per_control(sm):
    """So the UI can answer "which slider moved this joint?" without
    re-running the evaluation."""
    r = sm.evaluate_rig({"mood": 1.0, "head_nod": 1.0})
    assert r["contributions"]["mood"]["brow_lift"] == pytest.approx(0.8)
    assert r["contributions"]["head_nod"]["head_pitch"] == pytest.approx(1.0)


def test_evaluate_reports_clamping(sm):
    """Two controls driving one joint past its limit must surface, or the
    operator can't tell why the slider stopped doing anything."""
    rig = RIG_XML.replace(
        b'<drive joint="neck_yaw" scale="0.25"/>',
        b'<drive joint="neck_yaw" scale="2.0"/>')
    sm.robot_store.install_rig(rig, "robot.rig.xml")
    r = sm.evaluate_rig({"head_nod": 1.0})
    assert r["clamped"] is True
    assert r["scale_applied"] < 1.0
    assert abs(r["joints"]["neck_yaw"]) <= 1.0 + 1e-9


def test_evaluate_does_not_touch_the_robot_without_apply(sm):
    sm.evaluate_rig({"head_nod": 1.0})
    assert sm._routing_evaluator.joint_values == {}


def test_evaluate_with_apply_drives_the_robot(sm):
    r = sm.evaluate_rig({"head_nod": 1.0}, apply=True)
    ev = sm._routing_evaluator
    assert r["applied"] == len(r["joints"])
    assert ev.joint_values["head_pitch"] == pytest.approx(1.0)


def test_apply_batches_into_one_frame(sm):
    """One sheet evaluation and one UI broadcast per slider tick, not one
    per joint — a rig control can easily touch a dozen joints."""
    sm.evaluate_rig({"mood": 1.0, "head_nod": 1.0, "eye_look.x": 1.0},
                    apply=True)
    assert sm._routing_evaluator.frames == 1


def test_evaluate_ignores_non_numeric_values(sm):
    r = sm.evaluate_rig({"head_nod": "quite a lot", "eye_look.x": 1.0})
    assert r["success"] is True
    assert r["joints"]["eye_l_pan"] == pytest.approx(1.0)
    assert r["joints"].get("head_pitch", 0.0) == pytest.approx(0.0)


def test_evaluate_ignores_unknown_control_names(sm):
    r = sm.evaluate_rig({"no_such_control": 1.0})
    assert r["success"] is True


def test_evaluate_without_a_rig_fails_cleanly(sm):
    sm.robot_store.delete_rig()
    r = sm.evaluate_rig({"head_nod": 1.0})
    assert r["success"] is False
    assert "No rig" in r["message"]


def test_gaze_control_is_reported_as_skipped(sm):
    """Declared in the schema, not evaluated yet. An operator dragging a
    gizmo that does nothing deserves a reason."""
    rig = RIG_XML.replace(b'</rig>', b'''
      <control name="look_at" kind="spatial" anchor="head">
        <gaze><frame link="eye_l" axis="0 0 1"/></gaze>
      </control></rig>''')
    sm.robot_store.install_rig(rig, "robot.rig.xml")
    r = sm.evaluate_rig({})
    assert [s["control"] for s in r["skipped"]] == ["look_at"]
    assert "IK solver" in r["skipped"][0]["reason"]


def test_neutral_pose_is_the_blend_origin(sm):
    """Not zeros: with neutral_pose naming a pose that offsets a joint,
    a control at rest has to sit at that offset."""
    sm.pose_store.save(Pose(id="neutral", name="neutral", setpoints=[
        PoseSetpoint(target_kind="joint", joint="brow_lift", value=0.3)]))
    r = sm.evaluate_rig({})
    assert r["joints"]["brow_lift"] == pytest.approx(0.3)
    # And a half-blend toward happy runs from there, not from 0.
    half = sm.evaluate_rig({"mood": 0.5})
    assert half["joints"]["brow_lift"] == pytest.approx(0.3 + (0.8 - 0.3) * 0.5)


class TestWebSocketDispatch:
    """The actions have to be reachable on the channel the UI calls.

    An action registered on the wrong channel — or misspelled — returns
    "Unknown action", which the frontend's catch swallows into an empty
    panel. That's happened before in this codebase
    (`list_websocket_inputs` lives on the router channel, not
    management), so the wiring gets its own test rather than being
    assumed from the fact that the state_manager method works.
    """

    @pytest.fixture
    def handler(self, sm):
        from saint_server.webserver.websocket_handler import WebSocketHandler
        return WebSocketHandler(sm)

    @staticmethod
    def call(handler, action, params=None):
        import asyncio
        return asyncio.get_event_loop().run_until_complete(
            handler._handle_management(None, action, params or {}))

    def test_get_rig_is_reachable(self, handler):
        r = self.call(handler, 'get_rig')
        assert r["status"] == "ok"
        assert r["data"]["rig"]["controls"]

    def test_evaluate_rig_is_reachable(self, handler):
        r = self.call(handler, 'evaluate_rig', {"values": {"head_nod": 1.0}})
        assert r["status"] == "ok"
        assert r["data"]["joints"]["head_pitch"] == pytest.approx(1.0)

    def test_evaluate_rig_rejects_a_non_object_values(self, handler):
        r = self.call(handler, 'evaluate_rig', {"values": [1, 2, 3]})
        assert r["status"] == "error"

    def test_list_group_states_is_reachable(self, handler):
        r = self.call(handler, 'list_group_states')
        assert r["status"] == "ok"
        assert "group_states" in r["data"]

    def test_import_group_states_is_reachable(self, handler):
        r = self.call(handler, 'import_group_states', {"names": []})
        assert r["status"] == "ok"

    def test_import_group_states_rejects_a_non_list_names(self, handler):
        r = self.call(handler, 'import_group_states', {"names": "happy"})
        assert r["status"] == "error"


class TestEvaluatorCache:
    """`evaluate_rig` runs on every slider input event — ~30/s while
    dragging. Rebuilding the evaluator parses the rig XML, parses the
    WHOLE URDF, and reads a JSON file per referenced pose, so doing it
    per call means re-parsing a several-hundred-link URDF thirty times a
    second on a Pi. Caching is required, not an optimization.

    Both directions matter: a cache that never hits is the performance
    bug, and a cache that never invalidates silently blends toward the
    old shape of a pose the operator just edited.
    """

    def test_repeated_evaluation_does_not_reparse(self, sm, monkeypatch):
        parses = []
        original = sm.robot_store.load_urdf
        monkeypatch.setattr(sm.robot_store, "load_urdf",
                            lambda: (parses.append(1), original())[1])

        for v in (0.0, 0.25, 0.5, 0.75, 1.0):
            sm.evaluate_rig({"mood": v})
        assert len(parses) == 1, f"URDF parsed {len(parses)}x for 5 evaluations"

    def test_saving_a_pose_invalidates(self, sm):
        """The pose is an input to the blend, so a stale cache keeps the
        control blending toward the pose's old shape."""
        before = sm.evaluate_rig({"mood": 1.0})["joints"]["brow_lift"]
        assert before == pytest.approx(0.8)

        sm.save_pose({"id": "happy", "name": "happy", "setpoints": [
            {"target_kind": "joint", "joint": "brow_lift", "value": 0.2}]})

        after = sm.evaluate_rig({"mood": 1.0})["joints"]["brow_lift"]
        assert after == pytest.approx(0.2)

    def test_deleting_a_pose_invalidates(self, sm):
        assert sm.evaluate_rig({"mood": 1.0})["joints"]["brow_lift"] == \
            pytest.approx(0.8)
        sm.delete_pose("happy")
        # With the pose gone the target contributes nothing, so the joint
        # falls back to neutral.
        assert sm.evaluate_rig({"mood": 1.0})["joints"].get("brow_lift", 0.0) == \
            pytest.approx(0.0)

    def test_replacing_the_rig_invalidates(self, sm):
        assert sm.evaluate_rig({"head_nod": 1.0})["joints"]["head_pitch"] == \
            pytest.approx(1.0)
        halved = RIG_XML.replace(
            b'<drive joint="head_pitch" scale="1.0"/>',
            b'<drive joint="head_pitch" scale="0.5"/>')
        sm.robot_store.install_rig(halved, "robot.rig.xml")
        assert sm.evaluate_rig({"head_nod": 1.0})["joints"]["head_pitch"] == \
            pytest.approx(0.5)

    def test_replacing_the_urdf_invalidates(self, sm):
        """Limits define the normalization, so a changed limit changes
        every value the evaluator produces."""
        assert sm.evaluate_rig({"head_nod": 1.0})["joints"]["neck_yaw"] == \
            pytest.approx(0.25)
        # Same rig, but eye_l_pan now has a mimic off neck_yaw — a joint
        # that wasn't in the output before.
        widened = URDF_XML.replace(
            b'<joint name="eye_l_pan" type="revolute">\n'
            b'    <parent link="head"/><child link="eye_l"/>\n'
            b'    <limit lower="-1" upper="1"/></joint>',
            b'<joint name="eye_l_pan" type="revolute">\n'
            b'    <parent link="head"/><child link="eye_l"/>\n'
            b'    <limit lower="-1" upper="1"/>\n'
            b'    <mimic joint="neck_yaw" multiplier="1.0" offset="0"/></joint>')
        assert widened != URDF_XML, "fixture replace did not apply"
        sm.robot_store.install_from_urdf(widened, "v2.urdf")
        frame = sm.evaluate_rig({"head_nod": 1.0})
        assert frame["joints"]["eye_l_pan"] == pytest.approx(0.25)


def test_only_referenced_poses_are_loaded(sm, monkeypatch):
    """Loading the whole library per call would read every pose file off
    disk on every slider tick."""
    reads = []
    original = sm.pose_store.get

    def spy(pose_id):
        reads.append(pose_id)
        return original(pose_id)

    monkeypatch.setattr(sm.pose_store, "get", spy)
    sm.pose_store.save(Pose(id="unrelated", name="unrelated", setpoints=[
        PoseSetpoint(target_kind="joint", joint="neck_yaw", value=1.0)]))
    reads.clear()
    sm.evaluate_rig({"mood": 1.0})
    assert "unrelated" not in reads
    assert set(reads) == {"happy", "mad", "neutral"}
