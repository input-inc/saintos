"""URDF + mesh storage for the animation builder.

Operators upload a `.zip` bundle containing a `.urdf` file at the top
level and any referenced mesh sidecars (`.stl` / `.dae` / `.obj`)
under a `meshes/` subdirectory. The store unpacks them into
``{config_dir}/robot/``, where they outlive package installs (same
runtime-config convention as ``nodes/`` and ``system_routing.yaml``).

The HTTP layer (`http_server.py`) serves the URDF text and individual
mesh files. Both the server web UI and (in a follow-on phase) the
controller's Tauri webview fetch from the same canonical URLs.
"""

from __future__ import annotations

import hashlib
import io
import json
import os
import shutil
import time
import zipfile
from dataclasses import dataclass, asdict
from typing import Optional, List
from xml.etree import ElementTree as ET


# Mesh file extensions a URDF is allowed to reference. Anything else in
# the upload bundle is dropped on the floor so a stray `.exe` doesn't
# end up on disk in a place the webserver will happily serve it.
ALLOWED_MESH_EXTENSIONS = {".stl", ".dae", ".obj", ".ply", ".glb", ".gltf"}


@dataclass
class URDFMetadata:
    """Bookkeeping for the currently-installed robot model."""
    original_filename: str
    urdf_filename: str
    sha256: str
    uploaded_at: float            # unix timestamp
    mesh_files: List[str]         # filenames (no path), as found under meshes/
    link_count: int               # parsed from URDF for quick UI display
    joint_count: int

    def to_dict(self) -> dict:
        return asdict(self)


class URDFStoreError(Exception):
    """Raised when a URDF upload fails validation."""


class URDFStore:
    """Filesystem-backed store for the active robot URDF + meshes.

    One model at a time — replacing an upload wipes the previous tree
    in a single shutil.rmtree() so we don't accumulate stale meshes
    from prior bundles.
    """

    def __init__(self, config_dir: str, logger=None):
        self.config_dir = config_dir
        self.robot_dir = os.path.join(config_dir, "robot")
        self.meshes_dir = os.path.join(self.robot_dir, "meshes")
        self.metadata_path = os.path.join(self.robot_dir, "metadata.json")
        self.logger = logger

    # ── public API ──────────────────────────────────────────────────

    def has_model(self) -> bool:
        """True if a URDF + metadata are present on disk."""
        return os.path.isfile(self.metadata_path) and self._find_urdf() is not None

    def get_metadata(self) -> Optional[URDFMetadata]:
        if not os.path.isfile(self.metadata_path):
            return None
        try:
            with open(self.metadata_path, "r") as f:
                d = json.load(f)
            return URDFMetadata(**d)
        except Exception as e:
            self._log("warning", f"Failed to read URDF metadata: {e}")
            return None

    def get_urdf_path(self) -> Optional[str]:
        """Absolute path to the URDF on disk, or None."""
        return self._find_urdf()

    def get_mesh_path(self, filename: str) -> Optional[str]:
        """Resolve a mesh filename to an absolute path, or None.

        Rejects path-traversal attempts — only files directly under
        ``meshes/`` are reachable.
        """
        if not filename or "/" in filename or "\\" in filename or ".." in filename:
            return None
        candidate = os.path.join(self.meshes_dir, filename)
        candidate_norm = os.path.normpath(candidate)
        if not candidate_norm.startswith(os.path.normpath(self.meshes_dir) + os.sep):
            return None
        return candidate_norm if os.path.isfile(candidate_norm) else None

    def install_from_zip(self, zip_bytes: bytes, original_filename: str) -> URDFMetadata:
        """Replace any existing model with the contents of a zip bundle.

        Raises URDFStoreError if the bundle is missing a URDF or the
        URDF doesn't parse. Mesh files outside ALLOWED_MESH_EXTENSIONS
        are silently dropped (logged).
        """
        if not zip_bytes:
            raise URDFStoreError("empty upload")
        try:
            zf = zipfile.ZipFile(io.BytesIO(zip_bytes))
        except zipfile.BadZipFile as e:
            raise URDFStoreError(f"not a valid zip file: {e}") from e

        urdf_member = None
        mesh_members = []
        for member in zf.namelist():
            if member.endswith("/"):
                continue
            # Reject any entry whose path tries to escape the bundle root.
            if member.startswith("/") or ".." in member.split("/"):
                self._log("warning", f"Rejecting unsafe zip entry: {member}")
                continue
            lower = member.lower()
            base = os.path.basename(member)
            if lower.endswith(".urdf") or lower.endswith(".xacro"):
                # Prefer the first URDF found. Multiple .urdf files in
                # one bundle is unusual and we don't try to be clever.
                if urdf_member is None:
                    urdf_member = (member, base)
            else:
                ext = os.path.splitext(base)[1].lower()
                if ext in ALLOWED_MESH_EXTENSIONS:
                    mesh_members.append((member, base))

        if urdf_member is None:
            raise URDFStoreError("zip contains no .urdf file at any level")

        urdf_data = zf.read(urdf_member[0])
        link_count, joint_count = _validate_urdf(urdf_data)

        # Stage to a temp dir then atomically swap — protects the live
        # model if the operator uploads a broken bundle.
        tmp_dir = self.robot_dir + ".staging"
        if os.path.isdir(tmp_dir):
            shutil.rmtree(tmp_dir)
        os.makedirs(os.path.join(tmp_dir, "meshes"), exist_ok=True)

        urdf_target_name = urdf_member[1]
        urdf_target = os.path.join(tmp_dir, urdf_target_name)
        with open(urdf_target, "wb") as f:
            f.write(urdf_data)

        mesh_names: List[str] = []
        for member, base in mesh_members:
            # Strip any path prefix — meshes live in a flat dir.
            # Collisions across nested folders take the last write, which
            # is fine for URDFs that already use unique mesh filenames.
            target = os.path.join(tmp_dir, "meshes", base)
            with open(target, "wb") as f:
                f.write(zf.read(member))
            mesh_names.append(base)

        sha = hashlib.sha256(urdf_data).hexdigest()
        metadata = URDFMetadata(
            original_filename=original_filename,
            urdf_filename=urdf_target_name,
            sha256=sha,
            uploaded_at=time.time(),
            mesh_files=sorted(mesh_names),
            link_count=link_count,
            joint_count=joint_count,
        )
        with open(os.path.join(tmp_dir, "metadata.json"), "w") as f:
            json.dump(metadata.to_dict(), f, indent=2)

        # Swap in the new tree.
        if os.path.isdir(self.robot_dir):
            shutil.rmtree(self.robot_dir)
        os.rename(tmp_dir, self.robot_dir)

        self._log("info",
                  f"URDF installed: {urdf_target_name} "
                  f"({link_count} links, {joint_count} joints, "
                  f"{len(mesh_names)} meshes)")
        return metadata

    def install_from_urdf(self, urdf_bytes: bytes, original_filename: str) -> URDFMetadata:
        """Convenience: install a single .urdf file (no meshes).

        Useful for primitive-only robots and tests. For real robots the
        operator should upload a zip.
        """
        link_count, joint_count = _validate_urdf(urdf_bytes)

        urdf_target_name = os.path.basename(original_filename) or "robot.urdf"
        if not urdf_target_name.lower().endswith((".urdf", ".xacro")):
            urdf_target_name = "robot.urdf"

        tmp_dir = self.robot_dir + ".staging"
        if os.path.isdir(tmp_dir):
            shutil.rmtree(tmp_dir)
        os.makedirs(os.path.join(tmp_dir, "meshes"), exist_ok=True)
        with open(os.path.join(tmp_dir, urdf_target_name), "wb") as f:
            f.write(urdf_bytes)

        metadata = URDFMetadata(
            original_filename=original_filename,
            urdf_filename=urdf_target_name,
            sha256=hashlib.sha256(urdf_bytes).hexdigest(),
            uploaded_at=time.time(),
            mesh_files=[],
            link_count=link_count,
            joint_count=joint_count,
        )
        with open(os.path.join(tmp_dir, "metadata.json"), "w") as f:
            json.dump(metadata.to_dict(), f, indent=2)

        if os.path.isdir(self.robot_dir):
            shutil.rmtree(self.robot_dir)
        os.rename(tmp_dir, self.robot_dir)
        self._log("info", f"URDF installed (no meshes): {urdf_target_name}")
        return metadata

    def delete(self) -> bool:
        """Remove any installed URDF. Returns True if something was deleted."""
        if os.path.isdir(self.robot_dir):
            shutil.rmtree(self.robot_dir)
            self._log("info", "Robot model deleted")
            return True
        return False

    # ── internals ───────────────────────────────────────────────────

    def list_joints(self) -> list:
        """Return the list of joint names in the currently-installed
        URDF (or [] if no model is installed / the file is unreadable).

        The animation builder uses this to populate the URDF-joint
        picker when an operator adds a URDF-joint input on a sheet.
        Skips fixed joints since they don't accept setpoints — only
        actuatable types (revolute, continuous, prismatic, floating,
        planar) are useful as routing sources.
        """
        path = self.get_urdf_path()
        if not path:
            return []
        try:
            with open(path, "rb") as fh:
                root = ET.fromstring(fh.read())
        except (OSError, ET.ParseError) as e:
            self._log("warn", f"list_joints: failed to parse URDF: {e}")
            return []
        joints = []
        for j in root.findall("joint"):
            name = j.get("name") or ""
            jtype = j.get("type") or ""
            if not name or jtype == "fixed":
                continue
            joints.append({"name": name, "type": jtype})
        return joints

    def _find_urdf(self) -> Optional[str]:
        """Locate the URDF file on disk by reading metadata, or scanning."""
        meta = self.get_metadata()
        if meta:
            path = os.path.join(self.robot_dir, meta.urdf_filename)
            if os.path.isfile(path):
                return path
        # Fallback scan — the metadata file might be missing on hand-
        # edited installs.
        if os.path.isdir(self.robot_dir):
            for name in os.listdir(self.robot_dir):
                if name.lower().endswith((".urdf", ".xacro")):
                    return os.path.join(self.robot_dir, name)
        return None

    def _log(self, level: str, msg: str) -> None:
        if self.logger:
            from saint_server.log_level import log_at
            log_at(self.logger, level, msg)


def _validate_urdf(urdf_bytes: bytes) -> tuple[int, int]:
    """Parse the URDF and return (link_count, joint_count).

    Raises URDFStoreError if the XML is malformed or doesn't look like
    a URDF (no <robot> root).
    """
    try:
        root = ET.fromstring(urdf_bytes)
    except ET.ParseError as e:
        raise URDFStoreError(f"URDF is not valid XML: {e}") from e
    if root.tag != "robot":
        raise URDFStoreError(
            f"URDF root element must be <robot>, got <{root.tag}>"
        )
    link_count = sum(1 for _ in root.findall("link"))
    joint_count = sum(1 for _ in root.findall("joint"))
    return link_count, joint_count
