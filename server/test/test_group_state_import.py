"""Importing SRDF group_states as poses.

The conversion at the heart of this is the risky part: an SRDF
``<group_state>`` is authored in URDF-native units (radians, metres),
while the pose library and every sink downstream of the routing graph
speak a normalized −1..+1. A radian value that reaches a −1..+1 sink
unconverted commands a servo straight to its stop, so these tests pin
the conversion, and pin that unresolvable joints are *dropped and
reported* rather than passed through.

The other theme is not destroying an operator's work: poses imported
once and then hand-tuned must survive a re-uploaded SRDF unless the
operator explicitly asks for an overwrite.
"""
from __future__ import annotations

import io
import zipfile

import pytest

from saint_server.animation.models import Pose, PoseSetpoint
from saint_server.animation.robot_model_store import RobotModelStore
from saint_server.animation.store import PoseStore
from saint_server.webserver.state_manager import StateManager


# neck travels ±1.0 (so native == normalized, keeping arithmetic
# readable), brow travels 0..2.0 (so native 0.5 → normalized −0.5),
# jaw travels -0.5..0.7 (asymmetric: native 0 is NOT normalized 0).
URDF_XML = b"""<?xml version='1.0'?>
<robot name="rig_test">
  <link name="base"/><link name="neck"/><link name="brow"/><link name="jaw"/>
  <joint name="neck_yaw" type="revolute">
    <parent link="base"/><child link="neck"/>
    <limit lower="-1.0" upper="1.0"/></joint>
  <joint name="brow_lift" type="revolute">
    <parent link="neck"/><child link="brow"/>
    <limit lower="0" upper="2.0"/></joint>
  <joint name="jaw_open" type="revolute">
    <parent link="neck"/><child link="jaw"/>
    <limit lower="-0.5" upper="0.7"/></joint>
</robot>
"""

SRDF_XML = b"""<?xml version='1.0'?>
<robot name="rig_test">
  <group name="face">
    <joint name="brow_lift"/><joint name="jaw_open"/>
  </group>
  <group_state name="neutral" group="face">
    <joint name="brow_lift" value="0"/>
    <joint name="jaw_open" value="0.1"/>
  </group_state>
  <group_state name="happy" group="face">
    <joint name="brow_lift" value="1.5"/>
    <joint name="jaw_open" value="0.7"/>
  </group_state>
  <group_state name="mad" group="face">
    <joint name="brow_lift" value="0.5"/>
  </group_state>
</robot>
"""


def bundle(*members) -> bytes:
    buf = io.BytesIO()
    with zipfile.ZipFile(buf, "w") as zf:
        for name, data in members:
            zf.writestr(name, data)
    return buf.getvalue()


class PerSetpointEvaluator:
    """Only the per-setpoint API — no apply_animation_frame at all."""

    def __init__(self):
        self.joint_values = {}
        self.ws_values = {}

    def set_urdf_joint_value(self, joint, value):
        self.joint_values[joint] = value
        return True

    def set_ws_input(self, sheet_id, input_id, value):
        self.ws_values[(sheet_id, input_id)] = value
        return True


class FakeEvaluator(PerSetpointEvaluator):
    """Records what a pose apply fanned out, in both address spaces."""

    def __init__(self):
        super().__init__()
        self.batch_calls = 0

    def apply_animation_frame(self, joint_values, ws_values):
        self.batch_calls += 1
        self.joint_values.update(joint_values or {})
        self.ws_values.update(ws_values or {})
        return True


@pytest.fixture
def sm(tmp_path):
    """StateManager with a real robot store + pose store on tmp disk."""
    manager = StateManager(server_name="TEST", config_dir=str(tmp_path))
    manager.robot_store = RobotModelStore(config_dir=str(tmp_path))
    manager.pose_store = PoseStore(str(tmp_path))
    manager._routing_evaluator = FakeEvaluator()
    return manager


@pytest.fixture
def sm_loaded(sm):
    sm.robot_store.install_from_zip(
        bundle(("robot.urdf", URDF_XML), ("robot.srdf", SRDF_XML)), "b.zip")
    return sm


# ── the unit conversion ────────────────────────────────────────────


def test_import_converts_native_units_to_normalized(sm_loaded):
    """brow_lift travels 0..2.0, so the authored 1.5 rad is normalized
    0.5. Importing 1.5 verbatim would drive it 3× past its own limit."""
    result = sm_loaded.import_group_states(names=["happy"])
    assert result["success"] is True

    pose = sm_loaded.pose_store.get("happy")
    values = pose.joint_values()
    assert values["brow_lift"] == pytest.approx(0.5)
    # jaw_open travels -0.5..0.7; 0.7 is its upper stop → +1.0.
    assert values["jaw_open"] == pytest.approx(1.0)


def test_asymmetric_joint_native_zero_is_not_normalized_zero(sm_loaded):
    """jaw_open travels -0.5..0.7, so its midpoint of travel is 0.1.
    Getting this backwards makes every imported pose lean."""
    sm_loaded.import_group_states(names=["neutral"])
    values = sm_loaded.pose_store.get("neutral").joint_values()
    assert values["jaw_open"] == pytest.approx(0.0)      # native 0.1 == midpoint
    assert values["brow_lift"] == pytest.approx(-1.0)    # native 0 == lower stop


def test_import_stores_joint_addressed_setpoints(sm_loaded):
    """A group_state names joints, and there's no way to express that as
    WS inputs without a pre-existing routing binding per joint."""
    sm_loaded.import_group_states(names=["happy"])
    pose = sm_loaded.pose_store.get("happy")
    assert len(pose.setpoints) == 2
    for s in pose.setpoints:
        assert s.target_kind == "joint"
        assert s.joint and not s.sheet_id and not s.ws_input_id


def test_unresolved_joints_are_dropped_and_reported(sm):
    """Silence is not an option: an unconverted radian value is one hop
    from a servo."""
    srdf = SRDF_XML.replace(
        b'<joint name="brow_lift" value="1.5"/>',
        b'<joint name="brow_lift" value="1.5"/>'
        b'<joint name="renamed_away" value="9.9"/>')
    sm.robot_store.install_from_zip(
        bundle(("robot.urdf", URDF_XML), ("robot.srdf", srdf)), "b.zip")

    result = sm.import_group_states(names=["happy"])
    assert any("renamed_away" in w for w in result["warnings"])
    assert "renamed_away" not in sm.pose_store.get("happy").joint_values()


def test_group_state_with_nothing_resolvable_is_skipped(sm):
    srdf = b"""<robot name="rig_test">
      <group_state name="ghostly" group="">
        <joint name="nope_a" value="1"/><joint name="nope_b" value="2"/>
      </group_state></robot>"""
    sm.robot_store.install_from_zip(
        bundle(("robot.urdf", URDF_XML), ("robot.srdf", srdf)), "b.zip")
    result = sm.import_group_states()
    assert result["imported"] == []
    assert result["skipped"][0]["name"] == "ghostly"
    assert "no joints resolved" in result["skipped"][0]["reason"]


# ── selection + metadata ───────────────────────────────────────────


def test_import_all_when_no_names_given(sm_loaded):
    result = sm_loaded.import_group_states()
    assert {p["name"] for p in result["imported"]} == {"neutral", "happy", "mad"}


def test_import_selected_subset_only(sm_loaded):
    result = sm_loaded.import_group_states(names=["happy", "mad"])
    assert {p["name"] for p in result["imported"]} == {"happy", "mad"}
    assert sm_loaded.pose_store.get("neutral") is None


def test_imported_pose_records_provenance(sm_loaded):
    sm_loaded.import_group_states(names=["happy"], group="Face", icon="mood")
    pose = sm_loaded.pose_store.get("happy")
    assert pose.source == "srdf"
    assert pose.source_ref == "happy"
    assert pose.group == "Face"
    assert pose.icon == "mood"
    assert "group_state" in pose.description


def test_group_defaults_to_the_srdf_group(sm_loaded):
    sm_loaded.import_group_states(names=["happy"])
    assert sm_loaded.pose_store.get("happy").group == "face"


# ── not destroying operator work ───────────────────────────────────


def test_existing_pose_is_skipped_without_overwrite(sm_loaded):
    """An operator may have tuned an imported pose by hand; re-uploading
    the SRDF must not discard that."""
    sm_loaded.import_group_states(names=["happy"])
    tuned = sm_loaded.pose_store.get("happy")
    tuned.setpoints[0].value = 0.123
    sm_loaded.pose_store.save(tuned)

    result = sm_loaded.import_group_states(names=["happy"])
    assert result["imported"] == []
    assert result["skipped"][0]["reason"].startswith("pose 'happy' already exists")
    assert sm_loaded.pose_store.get("happy").setpoints[0].value == pytest.approx(0.123)


def test_overwrite_replaces_but_keeps_the_creation_stamp(sm_loaded):
    sm_loaded.import_group_states(names=["happy"])
    original_created = sm_loaded.pose_store.get("happy").created

    tuned = sm_loaded.pose_store.get("happy")
    tuned.setpoints[0].value = 0.123
    sm_loaded.pose_store.save(tuned)

    result = sm_loaded.import_group_states(names=["happy"], overwrite=True)
    assert len(result["imported"]) == 1
    pose = sm_loaded.pose_store.get("happy")
    assert pose.joint_values()["brow_lift"] == pytest.approx(0.5)   # re-derived
    assert pose.created == original_created      # a revision, not a new pose


def test_list_group_states_flags_existing_and_edited(sm_loaded):
    """The prompt needs both flags to avoid silently clobbering tweaks."""
    before = {g["name"]: g for g in sm_loaded.list_group_states()["group_states"]}
    assert before["happy"]["exists"] is False

    sm_loaded.import_group_states(names=["happy"])
    after = {g["name"]: g for g in sm_loaded.list_group_states()["group_states"]}
    assert after["happy"]["exists"] is True
    assert after["happy"]["pose_id"] == "happy"
    assert after["mad"]["exists"] is False


def test_list_group_states_carries_both_unit_systems(sm_loaded):
    states = {g["name"]: g for g in sm_loaded.list_group_states()["group_states"]}
    happy = states["happy"]
    assert happy["joint_values"]["brow_lift"] == pytest.approx(1.5)   # as authored
    assert happy["normalized"]["brow_lift"] == pytest.approx(0.5)     # as stored


# ── guardrails ─────────────────────────────────────────────────────


def test_import_without_a_urdf_fails_cleanly(sm):
    result = sm.import_group_states()
    assert result["success"] is False
    assert "No URDF" in result["message"]


def test_import_without_an_srdf_says_so(sm):
    sm.robot_store.install_from_urdf(URDF_XML, "robot.urdf")
    result = sm.import_group_states()
    assert result["success"] is False
    assert "SRDF" in result["message"]


def test_list_group_states_without_a_store_is_not_a_crash(sm):
    sm.robot_store = None
    result = sm.list_group_states()
    assert result["success"] is False
    assert result["group_states"] == []


# ── applying joint-addressed poses ─────────────────────────────────


def test_apply_pose_routes_joint_setpoints_to_the_joint_cache(sm_loaded):
    sm_loaded.import_group_states(names=["happy"])
    result = sm_loaded.apply_pose("happy")
    ev = sm_loaded._routing_evaluator
    assert result["success"] is True
    assert result["skipped"] == []
    assert ev.joint_values["brow_lift"] == pytest.approx(0.5)
    assert ev.joint_values["jaw_open"] == pytest.approx(1.0)
    assert ev.ws_values == {}


def test_apply_pose_batches_into_one_evaluator_call(sm_loaded):
    """One sheet evaluation and one UI broadcast for the whole pose,
    rather than one of each per setpoint."""
    sm_loaded.import_group_states(names=["happy"])
    sm_loaded.apply_pose("happy")
    assert sm_loaded._routing_evaluator.batch_calls == 1


def test_apply_pose_handles_both_address_spaces_together(sm_loaded):
    """A pose may mix joint setpoints with WS-input setpoints — nothing
    stops an operator adding a sound-trigger input to an imported pose."""
    sm_loaded.pose_store.save(Pose(
        id="mixed", name="mixed", setpoints=[
            PoseSetpoint(target_kind="joint", joint="neck_yaw", value=0.5),
            PoseSetpoint(sheet_id="sheet1", ws_input_id="in1", value=0.25),
        ]))
    result = sm_loaded.apply_pose("mixed")
    ev = sm_loaded._routing_evaluator
    assert result["applied"] == 2
    assert ev.joint_values["neck_yaw"] == pytest.approx(0.5)
    assert ev.ws_values[("sheet1", "in1")] == pytest.approx(0.25)


def test_apply_pose_falls_back_when_evaluator_lacks_the_batch_call(sm_loaded):
    sm_loaded._routing_evaluator = PerSetpointEvaluator()
    sm_loaded.import_group_states(names=["happy"])
    result = sm_loaded.apply_pose("happy")
    assert result["applied"] == 2
    assert sm_loaded._routing_evaluator.joint_values["brow_lift"] == pytest.approx(0.5)


def test_preview_setpoints_accepts_joint_targets(sm_loaded):
    result = sm_loaded.preview_setpoints([
        {"target_kind": "joint", "joint": "neck_yaw", "value": -0.75},
    ])
    assert result["applied"] == 1
    assert sm_loaded._routing_evaluator.joint_values["neck_yaw"] == pytest.approx(-0.75)


def test_setpoint_missing_its_address_is_skipped_not_applied(sm_loaded):
    result = sm_loaded.preview_setpoints([
        {"target_kind": "joint", "joint": "", "value": 1.0},
        {"target_kind": "ws_input", "sheet_id": "s", "ws_input_id": "", "value": 1.0},
    ])
    assert result["applied"] == 0
    assert len(result["skipped"]) == 2


# ── backward compatibility ─────────────────────────────────────────


def test_poses_saved_before_joint_setpoints_still_load(sm_loaded):
    """Every pose authored before this feature has no target_kind and
    must keep behaving as a WS-input pose."""
    legacy = {
        "id": "legacy", "name": "Legacy",
        "setpoints": [{"sheet_id": "sheetA", "ws_input_id": "inputB",
                       "value": 0.4}],
    }
    pose = Pose.from_dict(legacy)
    assert pose.setpoints[0].target_kind == "ws_input"
    assert pose.setpoints[0].is_joint is False
    assert pose.joint_values() == {}

    sm_loaded.pose_store.save(pose)
    sm_loaded.apply_pose("legacy")
    assert sm_loaded._routing_evaluator.ws_values[("sheetA", "inputB")] == \
        pytest.approx(0.4)


def test_setpoint_round_trips_through_dict():
    s = PoseSetpoint(target_kind="joint", joint="neck_yaw", value=-0.25)
    assert PoseSetpoint.from_dict(s.to_dict()) == s
    w = PoseSetpoint(sheet_id="s", ws_input_id="i", value=0.5)
    assert PoseSetpoint.from_dict(w.to_dict()) == w
