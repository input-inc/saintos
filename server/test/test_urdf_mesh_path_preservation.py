"""Lock-down tests for URDF mesh path preservation (2026-08).

Regression: the store used to flatten every mesh to its basename on
upload with last-write-wins. The johnny5 head model ships meshes in
subfolders where three basenames repeat with DIFFERENT geometry
(Meshes/SimpleMouth/static_97a3da.stl vs
Meshes/SimplifiedHead2/static_97a3da.stl, plus static_737373.stl and
static_e9e9eb.stl). After flattening, both URDF references resolved to
the one surviving file — that part rendered TWICE in the web preview
and its counterpart vanished.

These tests pin the fix: meshes install under their URDF-relative
paths, both colliding files survive with their own bytes, serving
resolves nested paths (with a basename fallback for legacy flat
installs), and traversal stays rejected.
"""
from __future__ import annotations

import io
import os
import sys
import zipfile

import pytest

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from saint_server.animation.urdf_store import URDFStore, URDFStoreError


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
    return URDFStore(config_dir=str(tmp_path))


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
        with pytest.raises(URDFStoreError):
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
