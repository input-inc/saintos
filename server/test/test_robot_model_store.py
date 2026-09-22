"""Robot model store: URDF + meshes, SRDF, and rig file.

Two sets of concerns here.

**Mesh path preservation (2026-08 regression).** The store used to
flatten every mesh to its basename on upload with last-write-wins. The
johnny5 head model ships meshes in subfolders where three basenames
repeat with DIFFERENT geometry (Meshes/SimpleMouth/static_97a3da.stl vs
Meshes/SimplifiedHead2/static_97a3da.stl, plus static_737373.stl and
static_e9e9eb.stl). After flattening, both URDF references resolved to
the one surviving file — that part rendered TWICE in the web preview and
its counterpart vanished. Those tests pin the fix: meshes install under
their URDF-relative paths, both colliding files survive with their own
bytes, serving resolves nested paths (with a basename fallback for
legacy flat installs), and traversal stays rejected.

**Three-file lifecycle.** URDF and SRDF share the ``<robot>`` root
element and are indistinguishable by tag, so classification is by
content — getting that wrong installs an SRDF as the robot's structure.
And the two lifecycles differ on purpose: a URDF install is a full
staged swap, while an SRDF or rig install writes in place and a URDF
replacement *carries companions forward* rather than deleting an
operator's rig because they re-exported the geometry.
"""
from __future__ import annotations

import io
import os
import sys
import zipfile

import pytest

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from saint_server.animation.robot_model_store import (
    RobotModelError,
    RobotModelStore,
)


URDF_XML = b"""<?xml version='1.0'?>
<robot name="johnny5_mini">
  <link name="base_link"/>
  <link name="head_link">
    <visual>
      <geometry><mesh filename="Meshes/SimpleMouth/static_97a3da.stl"/></geometry>
    </visual>
    <visual>
      <geometry><mesh filename="Meshes/SimplifiedHead2/static_97a3da.stl"/></geometry>
    </visual>
    <visual>
      <geometry><mesh filename="Meshes/EyeMechanism/eye_lens.stl"/></geometry>
    </visual>
  </link>
  <joint name="NeckTurn" type="revolute">
    <parent link="base_link"/>
    <child link="head_link"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1" upper="1" effort="1" velocity="1"/>
  </joint>
</robot>
"""

MOUTH_BYTES = b"MOUTH-GEOMETRY-BYTES"
HEAD_BYTES = b"HEAD-GEOMETRY-BYTES-DIFFERENT"
EYE_BYTES = b"EYE-LENS-BYTES"


def johnny5_zip() -> bytes:
    """Mimic the real bundle layout: URDF nested one level down, meshes
    in per-assembly subfolders, two of them sharing a basename."""
    buf = io.BytesIO()
    with zipfile.ZipFile(buf, "w") as zf:
        zf.writestr("Models/johnny5_head.urdf", URDF_XML)
        zf.writestr("Models/Meshes/SimpleMouth/static_97a3da.stl", MOUTH_BYTES)
        zf.writestr("Models/Meshes/SimplifiedHead2/static_97a3da.stl", HEAD_BYTES)
        zf.writestr("Models/Meshes/EyeMechanism/eye_lens.stl", EYE_BYTES)
    return buf.getvalue()


@pytest.fixture
def store(tmp_path):
    return RobotModelStore(config_dir=str(tmp_path))


class TestPathPreservingInstall:
    def test_colliding_basenames_both_survive_with_own_bytes(self, store):
        # THE johnny5 regression pin. Flattening made one of these
        # bytes win both references.
        store.install_from_zip(johnny5_zip(), "johnny5.zip")
        mouth = store.get_mesh_path("Meshes/SimpleMouth/static_97a3da.stl")
        head = store.get_mesh_path("Meshes/SimplifiedHead2/static_97a3da.stl")
        assert mouth and head and mouth != head
        assert open(mouth, "rb").read() == MOUTH_BYTES
        assert open(head, "rb").read() == HEAD_BYTES

    def test_metadata_lists_urdf_relative_paths(self, store):
        meta = store.install_from_zip(johnny5_zip(), "johnny5.zip")
        assert sorted(meta.mesh_files) == [
            "Meshes/EyeMechanism/eye_lens.stl",
            "Meshes/SimpleMouth/static_97a3da.stl",
            "Meshes/SimplifiedHead2/static_97a3da.stl",
        ]

    def test_paths_are_relative_to_the_urdf_not_the_zip_root(self, store):
        # The URDF lives at Models/johnny5_head.urdf and references
        # "Meshes/…" — the stored keys must match the URDF's view, with
        # the zip's leading "Models/" stripped.
        store.install_from_zip(johnny5_zip(), "johnny5.zip")
        assert store.get_mesh_path("Meshes/EyeMechanism/eye_lens.stl")
        assert store.get_mesh_path("Models/Meshes/EyeMechanism/eye_lens.stl") is None

    def test_urdf_still_parses_and_counts(self, store):
        meta = store.install_from_zip(johnny5_zip(), "johnny5.zip")
        assert meta.link_count == 2
        assert meta.joint_count == 1
        assert store.has_model()

    def test_flat_zip_installs_at_top_level(self, store):
        # URDF at zip root with a flat mesh next to it — rel path is
        # just the basename, same behavior as before.
        buf = io.BytesIO()
        with zipfile.ZipFile(buf, "w") as zf:
            zf.writestr("robot.urdf", URDF_XML)
            zf.writestr("wheel.stl", b"WHEEL")
        store.install_from_zip(buf.getvalue(), "flat.zip")
        p = store.get_mesh_path("wheel.stl")
        assert p and open(p, "rb").read() == b"WHEEL"


class TestServingResolution:
    def test_traversal_and_absolute_paths_rejected(self, store):
        store.install_from_zip(johnny5_zip(), "johnny5.zip")
        assert store.get_mesh_path("../metadata.json") is None
        assert store.get_mesh_path("Meshes/../../metadata.json") is None
        assert store.get_mesh_path("/etc/passwd") is None
        assert store.get_mesh_path("..\\metadata.json") is None
        assert store.get_mesh_path("") is None

    def test_legacy_flat_install_basename_fallback(self, store):
        # Models installed before path preservation have meshes flat
        # under meshes/. A nested request must still find them.
        os.makedirs(store.meshes_dir, exist_ok=True)
        with open(os.path.join(store.meshes_dir, "old_flat.stl"), "wb") as f:
            f.write(b"LEGACY")
        p = store.get_mesh_path("Meshes/Anything/old_flat.stl")
        assert p and open(p, "rb").read() == b"LEGACY"

    def test_basename_fallback_cannot_escape_meshes_dir(self, store):
        os.makedirs(store.robot_dir, exist_ok=True)
        with open(os.path.join(store.robot_dir, "metadata.json"), "w") as f:
            f.write("{}")
        # Even via the fallback, only meshes/ is reachable.
        assert store.get_mesh_path("Foo/metadata.json") is None


class TestBundleEdgeCases:
    def test_zip_without_urdf_rejected(self, store):
        buf = io.BytesIO()
        with zipfile.ZipFile(buf, "w") as zf:
            zf.writestr("only_mesh.stl", b"X")
        with pytest.raises(RobotModelError):
            store.install_from_zip(buf.getvalue(), "nourdf.zip")

    def test_disallowed_extensions_dropped(self, store):
        buf = io.BytesIO()
        with zipfile.ZipFile(buf, "w") as zf:
            zf.writestr("robot.urdf", URDF_XML)
            zf.writestr("Meshes/evil.exe", b"MZ")
            zf.writestr("Meshes/ok.stl", b"OK")
        meta = store.install_from_zip(buf.getvalue(), "mixed.zip")
        assert meta.mesh_files == ["Meshes/ok.stl"]
        assert store.get_mesh_path("Meshes/evil.exe") is None

    def test_mesh_outside_urdf_dir_kept_by_bundle_path(self, store):
        # A mesh that is a SIBLING of the URDF's directory can't be
        # referenced without "..", but it must not be dropped or allowed
        # to escape — it installs under its bundle-rooted path.
        buf = io.BytesIO()
        with zipfile.ZipFile(buf, "w") as zf:
            zf.writestr("Models/robot.urdf", URDF_XML)
            zf.writestr("SharedMeshes/common.stl", b"COMMON")
        meta = store.install_from_zip(buf.getvalue(), "sibling.zip")
        assert "SharedMeshes/common.stl" in meta.mesh_files
        p = store.get_mesh_path("SharedMeshes/common.stl")
        assert p and open(p, "rb").read() == b"COMMON"


# ── the SRDF + rig half ────────────────────────────────────────────

SRDF_XML = b"""<?xml version='1.0'?>
<robot name="johnny5_mini">
  <group name="neck"><joint name="NeckTurn"/></group>
  <group_state name="neutral" group="neck">
    <joint name="NeckTurn" value="0"/>
  </group_state>
  <group_state name="left" group="neck">
    <joint name="NeckTurn" value="-0.5"/>
  </group_state>
  <disable_collisions link1="base_link" link2="head_link" reason="Adjacent"/>
</robot>
"""

RIG_XML = b"""<?xml version='1.0'?>
<rig xmlns="urn:saintos:rig:1.0" version="1.0" robot="johnny5_mini">
  <settings clamp="scale_back" neutral_pose="neutral"/>
  <control name="head_turn" kind="channel" label="Head turn" group="Head">
    <drive joint="NeckTurn" scale="1.0" curve="easeInOut"/>
  </control>
</rig>
"""


def bundle(*members) -> bytes:
    buf = io.BytesIO()
    with zipfile.ZipFile(buf, "w") as zf:
        for name, data in members:
            zf.writestr(name, data)
    return buf.getvalue()


class TestBundleClassification:
    """URDF and SRDF both root at <robot>, so the root tag proves
    nothing and the filename is the least trustworthy signal available.
    Misclassifying here installs an SRDF as the robot's structure."""

    def test_bundle_with_all_three_files(self, store):
        store.install_from_zip(
            bundle(("robot.urdf", URDF_XML),
                   ("robot.srdf", SRDF_XML),
                   ("robot.rig.xml", RIG_XML),
                   ("Meshes/ok.stl", b"OK")),
            "full.zip")
        meta = store.get_metadata()
        assert meta.urdf_filename == "robot.urdf"
        assert meta.srdf_filename == "robot.srdf"
        assert meta.rig_filename == "robot.rig.xml"
        assert meta.robot_name == "johnny5_mini"

    def test_srdf_is_not_mistaken_for_the_urdf_when_it_sorts_first(self, store):
        """Named to sort ahead of the URDF and given a .urdf extension —
        only a content sniff gets this right."""
        store.install_from_zip(
            bundle(("aaa_semantic.urdf", SRDF_XML),
                   ("zzz_structure.urdf", URDF_XML)),
            "confusing.zip")
        meta = store.get_metadata()
        assert meta.urdf_filename == "zzz_structure.urdf"
        assert meta.srdf_filename == "aaa_semantic.urdf"
        assert store.load_urdf().joints          # real structure loaded

    def test_oddly_named_srdf_still_routes(self, store):
        store.install_from_zip(
            bundle(("robot.urdf", URDF_XML), ("semantics.xml", SRDF_XML)),
            "odd.zip")
        assert store.get_metadata().srdf_filename == "semantics.xml"

    def test_bundle_with_invalid_srdf_fails_the_whole_install(self, store):
        """Better to reject than to install a URDF next to an SRDF that
        will silently never bind."""
        with pytest.raises(RobotModelError, match="SRDF"):
            store.install_from_zip(
                bundle(("robot.urdf", URDF_XML),
                       ("robot.srdf", b"<robot><group>")),
                "broken.zip")

    def test_bundle_with_invalid_rig_fails_the_whole_install(self, store):
        with pytest.raises(RobotModelError, match="rig"):
            store.install_from_zip(
                bundle(("robot.urdf", URDF_XML),
                       ("robot.rig.xml", b'<rig version="9.0"/>')),
                "broken.zip")

    def test_unparseable_description_extension_is_refused(self, store):
        """A .srdf that doesn't parse used to classify as nothing and get
        dropped in silence, installing a URDF next to an SRDF that would
        never bind — exactly the failure you can't debug afterwards."""
        with pytest.raises(RobotModelError, match="not a parseable"):
            store.install_from_zip(
                bundle(("robot.urdf", URDF_XML),
                       ("robot.srdf", b"<robot><group>")),
                "broken.zip")

    def test_stray_generic_xml_is_ignored_not_fatal(self, store):
        """A bundle carrying an unrelated .xml is common enough that
        refusing it would be obnoxious — warn and move on."""
        store.install_from_zip(
            bundle(("robot.urdf", URDF_XML),
                   ("build_notes.xml", b"<notes>exported from Fusion</notes>")),
            "stray.zip")
        assert store.has_model()
        assert store.get_metadata().srdf_filename == ""


class TestCompanionInstall:
    def test_srdf_install_onto_existing_urdf(self, store):
        store.install_from_urdf(URDF_XML, "robot.urdf")
        result = store.install_srdf(SRDF_XML, "head.srdf")
        assert result["installed"] is True
        assert result["srdf_filename"] == "head.srdf"
        assert result["summary"]["group_state_count"] == 2
        assert store.get_srdf_path().endswith("head.srdf")
        # The URDF must survive — an SRDF install is not a replacement.
        assert store.get_urdf_path() is not None

    def test_srdf_install_without_a_urdf_is_refused(self, store):
        """Every name in an SRDF is a dangling reference on its own."""
        with pytest.raises(RobotModelError, match="no URDF installed"):
            store.install_srdf(SRDF_XML, "head.srdf")

    def test_srdf_uploaded_as_a_urdf_gets_a_useful_error(self, store):
        with pytest.raises(RobotModelError, match="looks like an SRDF"):
            store.install_from_urdf(SRDF_XML, "robot.urdf")

    def test_replacing_an_srdf_under_a_new_name_removes_the_old_file(self, store):
        """Otherwise the fallback scan can still find the stale one."""
        store.install_from_urdf(URDF_XML, "robot.urdf")
        store.install_srdf(SRDF_XML, "head.srdf")
        old = os.path.join(store.robot_dir, "head.srdf")
        assert os.path.isfile(old)
        store.install_srdf(SRDF_XML, "robot.srdf")
        assert not os.path.isfile(old)
        assert store.get_srdf_path().endswith("robot.srdf")

    def test_rig_install_and_summary(self, store):
        store.install_from_urdf(URDF_XML, "robot.urdf")
        result = store.install_rig(RIG_XML, "robot.rig.xml")
        assert result["summary"]["control_count"] == 1
        assert result["summary"]["clamp"] == "scale_back"
        assert store.load_rig().control("head_turn") is not None

    def test_rig_install_without_a_urdf_is_refused(self, store):
        with pytest.raises(RobotModelError, match="no URDF installed"):
            store.install_rig(RIG_XML, "robot.rig.xml")

    def test_invalid_companion_is_rejected_without_touching_disk(self, store):
        store.install_from_urdf(URDF_XML, "robot.urdf")
        store.install_srdf(SRDF_XML, "robot.srdf")
        with pytest.raises(RobotModelError):
            store.install_srdf(b"<robot><group>", "robot.srdf")
        # The good one is still there and still parses.
        assert len(store.load_srdf().group_states) == 2

    def test_delete_companions_leaves_the_urdf(self, store):
        store.install_from_zip(
            bundle(("robot.urdf", URDF_XML), ("robot.srdf", SRDF_XML),
                   ("robot.rig.xml", RIG_XML)), "full.zip")
        assert store.delete_srdf() is True
        assert store.delete_rig() is True
        assert store.get_srdf_path() is None
        assert store.get_rig_path() is None
        assert store.has_model()                 # URDF untouched
        # Idempotent — deleting what isn't there is not an error.
        assert store.delete_srdf() is False


class TestCompanionCarryOver:
    """A new URDF may invalidate an SRDF or rig — every name they carry
    is a reference — but deleting an operator's rig because they
    re-exported the geometry would be hostile."""

    def test_urdf_replacement_preserves_companions(self, store):
        store.install_from_urdf(URDF_XML, "robot.urdf")
        store.install_srdf(SRDF_XML, "robot.srdf")
        store.install_rig(RIG_XML, "robot.rig.xml")

        store.install_from_urdf(URDF_XML, "robot_v2.urdf")

        meta = store.get_metadata()
        assert meta.urdf_filename == "robot_v2.urdf"
        assert meta.srdf_filename == "robot.srdf"
        assert meta.rig_filename == "robot.rig.xml"
        assert len(store.load_srdf().group_states) == 2

    def test_carried_companions_are_revalidated_and_warn(self, store):
        """A rename that invalidates half a rig should surface as
        warnings, not silence."""
        store.install_from_urdf(URDF_XML, "robot.urdf")
        store.install_srdf(SRDF_XML, "robot.srdf")

        renamed = URDF_XML.replace(b'"NeckTurn"', b'"neck_turn"')
        store.install_from_urdf(renamed, "renamed.urdf")

        warnings = store.get_metadata().warnings
        assert any("NeckTurn" in w for w in warnings), warnings
        assert store.get_metadata().srdf_filename == "robot.srdf"

    def test_bundled_companion_wins_over_the_carried_one(self, store):
        store.install_from_urdf(URDF_XML, "robot.urdf")
        store.install_srdf(SRDF_XML, "old.srdf")

        newer = SRDF_XML.replace(b'name="left"', b'name="right"')
        store.install_from_zip(
            bundle(("robot.urdf", URDF_XML), ("fresh.srdf", newer)),
            "v2.zip")

        assert store.get_metadata().srdf_filename == "fresh.srdf"
        assert {gs.name for gs in store.load_srdf().group_states} == {
            "neutral", "right"}
        # The superseded file must not linger where a scan could find it.
        assert not os.path.isfile(os.path.join(store.robot_dir, "old.srdf"))


class TestQueries:
    def test_list_joints_carries_limits(self, store):
        """Every consumer that shows a joint also needs its limits — the
        normalized −1..+1 the stack speaks is defined by them."""
        store.install_from_urdf(URDF_XML, "robot.urdf")
        joints = store.list_joints()
        assert len(joints) == 1
        assert joints[0]["name"] == "NeckTurn"
        assert joints[0]["lower"] == -1.0 and joints[0]["upper"] == 1.0
        assert joints[0]["has_limits"] is True

    def test_list_groups_expands_to_joints(self, store):
        store.install_from_zip(
            bundle(("robot.urdf", URDF_XML), ("robot.srdf", SRDF_XML)), "b.zip")
        assert store.list_groups() == [{"name": "neck", "joints": ["NeckTurn"]}]

    def test_list_group_states_carries_native_and_normalized(self, store):
        """The import prompt needs the native values to show what the
        file says, the normalized ones to save, and the unresolved list
        to warn that a pose will be partial."""
        store.install_from_zip(
            bundle(("robot.urdf", URDF_XML), ("robot.srdf", SRDF_XML)), "b.zip")
        states = {s["name"]: s for s in store.list_group_states()}
        assert states["left"]["joint_values"] == {"NeckTurn": -0.5}   # native
        assert states["left"]["normalized"] == {"NeckTurn": -0.5}     # ±1 range
        assert states["left"]["unresolved"] == []
        assert states["neutral"]["normalized"] == {"NeckTurn": 0.0}

    def test_group_state_normalization_uses_the_urdf_limits(self, store):
        """A joint travelling 0..2 puts native 0.5 at normalized −0.5.
        Passing radians through unconverted would drive a servo to its
        stop, so this conversion is the whole point."""
        urdf = URDF_XML.replace(b'lower="-1" upper="1"', b'lower="0" upper="2"')
        srdf = SRDF_XML.replace(b'value="-0.5"', b'value="0.5"')
        store.install_from_zip(
            bundle(("robot.urdf", urdf), ("robot.srdf", srdf)), "b.zip")
        states = {s["name"]: s for s in store.list_group_states()}
        assert states["left"]["joint_values"] == {"NeckTurn": 0.5}
        assert states["left"]["normalized"] == {"NeckTurn": -0.5}

    def test_queries_are_empty_without_an_srdf(self, store):
        store.install_from_urdf(URDF_XML, "robot.urdf")
        assert store.list_groups() == []
        assert store.list_group_states() == []

    def test_validate_reports_unresolved_references_across_files(self, store):
        srdf = SRDF_XML.replace(b'"NeckTurn"', b'"ghost_joint"')
        store.install_from_zip(
            bundle(("robot.urdf", URDF_XML), ("robot.srdf", srdf)), "b.zip")
        problems = store.validate()
        assert any("SRDF" in p and "ghost_joint" in p for p in problems)

    def test_rig_pose_reference_matches_a_group_state_slug(self, store):
        """A rig names poses; importing an SRDF group_state called
        "Very Happy" produces a pose whose id is `very_happy`. Both
        spellings resolve at runtime (PoseStore slugifies on lookup), so
        validation has to accept both — otherwise it flags a reference
        that works perfectly well, and the operator learns to ignore the
        warning list."""
        srdf = b"""<robot name="johnny5_mini">
          <group name="neck"><joint name="NeckTurn"/></group>
          <group_state name="Very Happy" group="neck">
            <joint name="NeckTurn" value="0.5"/></group_state></robot>"""
        rig = b"""<rig version="1.0" robot="johnny5_mini">
          <control name="mood"><target pose="very_happy" at="1"/></control>
        </rig>"""
        store.install_from_zip(
            bundle(("robot.urdf", URDF_XML), ("robot.srdf", srdf),
                   ("robot.rig.xml", rig)), "b.zip")
        assert not [p for p in store.validate() if "very_happy" in p]
        # The authored spelling is equally valid.
        rig2 = rig.replace(b'pose="very_happy"', b'pose="Very Happy"')
        store.install_rig(rig2, "robot.rig.xml")
        assert not [p for p in store.validate() if "Happy" in p]

    def test_rig_pose_reference_to_a_real_nonexistent_pose_is_flagged(self, store):
        """The forgiving matching must not become no matching at all."""
        rig = b"""<rig version="1.0" robot="johnny5_mini">
          <control name="mood"><target pose="never_authored" at="1"/></control>
        </rig>"""
        store.install_from_zip(
            bundle(("robot.urdf", URDF_XML), ("robot.srdf", SRDF_XML),
                   ("robot.rig.xml", rig)), "b.zip")
        assert any("never_authored" in p for p in store.validate())

    def test_validate_accepts_ui_authored_pose_names(self, store):
        """A pose created in the UI never appears in a group_state, so the
        caller has to be able to supply the real library ids."""
        rig = b"""<rig version="1.0" robot="johnny5_mini">
          <control name="mood"><target pose="ui_made" at="1"/></control>
        </rig>"""
        store.install_from_zip(
            bundle(("robot.urdf", URDF_XML), ("robot.rig.xml", rig)), "b.zip")
        # Supplied the real library: the reference resolves.
        assert not [p for p in store.validate(pose_names=["ui_made"])
                    if "ui_made" in p]
        # Supplied a library that lacks it: flagged.
        assert any("ui_made" in p
                   for p in store.validate(pose_names=["something_else"]))

    def test_pose_references_are_unchecked_when_nothing_is_known(self, store):
        """With no SRDF and no pose list there's no ground truth, so pose
        references are skipped rather than all reported broken. Flagging
        every reference in that state would be pure noise."""
        rig = b"""<rig version="1.0" robot="johnny5_mini">
          <control name="mood"><target pose="anything" at="1"/></control>
        </rig>"""
        store.install_from_zip(
            bundle(("robot.urdf", URDF_XML), ("robot.rig.xml", rig)), "b.zip")
        assert not [p for p in store.validate() if "anything" in p]

    def test_describe_shape(self, store):
        store.install_from_zip(
            bundle(("robot.urdf", URDF_XML), ("robot.srdf", SRDF_XML),
                   ("robot.rig.xml", RIG_XML)), "full.zip")
        d = store.describe()
        assert d["installed"] is True
        assert d["robot_name"] == "johnny5_mini"
        assert d["srdf"]["group_state_count"] == 2
        assert d["rig"]["control_count"] == 1
        assert d["warnings"] == []

    def test_describe_when_nothing_installed(self, store):
        assert store.describe() == {"installed": False}

    def test_describe_reports_absent_companions_as_none(self, store):
        store.install_from_urdf(URDF_XML, "robot.urdf")
        d = store.describe()
        assert d["srdf"] is None and d["rig"] is None


class TestMetadataCompatibility:
    def test_metadata_predating_srdf_fields_still_loads(self, store):
        """Installs made before the three-file split have none of the new
        keys and must pick up defaults rather than raising."""
        from saint_server.animation.robot_model_store import RobotModelMetadata
        legacy = {
            "original_filename": "old.zip", "urdf_filename": "robot.urdf",
            "sha256": "abc", "uploaded_at": 1.0, "mesh_files": ["a.stl"],
            "link_count": 2, "joint_count": 1,
        }
        meta = RobotModelMetadata.from_dict(legacy)
        assert meta.urdf_filename == "robot.urdf"
        assert meta.srdf_filename == "" and meta.has_srdf is False
        assert meta.warnings == []

    def test_unknown_metadata_keys_are_dropped_not_fatal(self):
        """Metadata written by a newer server must not brick an older
        one."""
        from saint_server.animation.robot_model_store import RobotModelMetadata
        meta = RobotModelMetadata.from_dict(
            {"urdf_filename": "r.urdf", "future_field": {"nested": 1}})
        assert meta.urdf_filename == "r.urdf"

    def test_metadata_survives_a_round_trip(self, store):
        store.install_from_zip(
            bundle(("robot.urdf", URDF_XML), ("robot.srdf", SRDF_XML)), "b.zip")
        before = store.get_metadata()
        after = store.get_metadata()
        assert before.to_dict() == after.to_dict()
