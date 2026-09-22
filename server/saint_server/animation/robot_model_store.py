"""Storage for the robot model: URDF + meshes, SRDF, and rig file.

Replaces the older URDF-only store. The URDF is unchanged in role — it
still owns every name and carries all the geometry — but it's now one of
three files, mirroring how the ROS ecosystem actually packages a robot:

    robot.urdf     robot_description           structure, limits, geometry
    robot.srdf     robot_description_semantic  groups + named poses
    robot.rig.xml  robot_description_rig       controls (ours; docs/RIG_SCHEMA.md)

Operators upload either a ``.zip`` bundle containing all three plus a
``meshes/`` subdirectory, or individual files. Everything unpacks into
``{config_dir}/robot/``, which outlives package installs — the same
runtime-config convention as ``nodes/`` and ``system_routing.yaml``.

**Lifecycles differ, deliberately.** Installing a URDF is a full
replacement: the whole tree is staged and swapped so a broken bundle
can't take out the live model. Installing an SRDF or a rig file writes
in place, because those are companion annotations and wiping the URDF to
accept one would be absurd. And a new URDF *preserves* any existing
SRDF/rig rather than deleting them, then re-validates and reports what
no longer resolves — a rename that invalidates half a rig should produce
warnings an operator can act on, not silent data loss.

The HTTP layer (``http_server.py``) serves each file and the individual
meshes. Both the server web UI and the controller's Tauri webview fetch
from the same canonical URLs.
"""

from __future__ import annotations

import hashlib
import io
import json
import os
import posixpath
import shutil
import time
import zipfile
from dataclasses import dataclass, asdict, field, fields
from typing import Any, Dict, List, Optional, Tuple
from xml.etree import ElementTree as ET

from saint_server.animation.rig import Rig, RigError, looks_like_rig
from saint_server.animation.srdf import SRDF, SRDFError, looks_like_srdf
from saint_server.animation.urdf_model import UrdfModel


# Mesh file extensions a URDF is allowed to reference. Anything else in
# the upload bundle is dropped on the floor so a stray `.exe` doesn't
# end up on disk in a place the webserver will happily serve it.
ALLOWED_MESH_EXTENSIONS = {".stl", ".dae", ".obj", ".ply", ".glb", ".gltf"}

# Extensions that could be any of the three description files. URDF and
# SRDF share the <robot> root element and are indistinguishable by tag,
# so classification is by content first (see _classify_xml) and by
# extension only as a tiebreak.
_XML_EXTENSIONS = {".urdf", ".srdf", ".xacro", ".xml", ".rig"}


class RobotModelError(Exception):
    """Raised when an upload fails validation."""


@dataclass
class RobotModelMetadata:
    """Bookkeeping for the currently-installed robot model."""

    original_filename: str = ""
    urdf_filename: str = ""
    sha256: str = ""
    uploaded_at: float = 0.0
    mesh_files: List[str] = field(default_factory=list)
    link_count: int = 0
    joint_count: int = 0

    # Parsed from the URDF's <robot name>. The SRDF and rig both have to
    # match it or nothing binds, so it's worth surfacing in the UI.
    robot_name: str = ""

    srdf_filename: str = ""
    srdf_sha256: str = ""
    srdf_uploaded_at: float = 0.0

    rig_filename: str = ""
    rig_sha256: str = ""
    rig_uploaded_at: float = 0.0

    # Unresolved-reference warnings from the last install, recomputed on
    # every write. These are never fatal: an SRDF naming a joint that was
    # renamed away still loads, it just silently stops moving it, which
    # is exactly why they need surfacing.
    warnings: List[str] = field(default_factory=list)

    def to_dict(self) -> dict:
        return asdict(self)

    @classmethod
    def from_dict(cls, d: Dict[str, Any]) -> "RobotModelMetadata":
        """Tolerant load.

        Installs predating the SRDF/rig fields simply lack them and pick
        up defaults. Unknown keys are dropped rather than raising, so
        metadata written by a newer server doesn't brick an older one.
        """
        known = {f.name for f in fields(cls)}
        return cls(**{k: v for k, v in (d or {}).items() if k in known})

    @property
    def has_srdf(self) -> bool:
        return bool(self.srdf_filename)

    @property
    def has_rig(self) -> bool:
        return bool(self.rig_filename)


class RobotModelStore:
    """Filesystem-backed store for the active robot model.

    One model at a time — replacing the URDF wipes the previous tree in a
    single shutil.rmtree() so we don't accumulate stale meshes from prior
    bundles.
    """

    def __init__(self, config_dir: str, logger=None):
        self.config_dir = config_dir
        self.robot_dir = os.path.join(config_dir, "robot")
        self.meshes_dir = os.path.join(self.robot_dir, "meshes")
        self.metadata_path = os.path.join(self.robot_dir, "metadata.json")
        self.logger = logger

    # ── presence + metadata ─────────────────────────────────────────

    def has_model(self) -> bool:
        """True if a URDF + metadata are present on disk."""
        return os.path.isfile(self.metadata_path) and self._find_urdf() is not None

    def get_metadata(self) -> Optional[RobotModelMetadata]:
        if not os.path.isfile(self.metadata_path):
            return None
        try:
            with open(self.metadata_path, "r") as f:
                return RobotModelMetadata.from_dict(json.load(f))
        except Exception as e:
            self._log("warning", f"Failed to read robot metadata: {e}")
            return None

    def _write_metadata(self, meta: RobotModelMetadata,
                        directory: Optional[str] = None) -> None:
        target = os.path.join(directory or self.robot_dir, "metadata.json")
        os.makedirs(os.path.dirname(target), exist_ok=True)
        tmp = target + ".tmp"
        with open(tmp, "w") as f:
            json.dump(meta.to_dict(), f, indent=2)
        os.replace(tmp, target)

    # ── file paths ──────────────────────────────────────────────────

    def get_urdf_path(self) -> Optional[str]:
        return self._find_urdf()

    def get_srdf_path(self) -> Optional[str]:
        return self._find_companion("srdf_filename", (".srdf",))

    def get_rig_path(self) -> Optional[str]:
        return self._find_companion("rig_filename", (".rig.xml", ".rig"))

    def get_mesh_path(self, filename: str) -> Optional[str]:
        """Resolve a URDF-relative mesh path to an absolute path, or None.

        Accepts nested paths ("Meshes/EyeMechanism/eye.stl") — meshes are
        stored under their URDF-relative directories so two files with
        the same basename in different folders stay distinct (johnny5
        regression: SimpleMouth/ and SimplifiedHead2/ both ship a
        static_97a3da.stl). Rejects path-traversal attempts — only files
        under ``meshes/`` are reachable.

        Falls back to a flat basename lookup for models installed before
        path preservation (they were flattened on upload).
        """
        if not filename:
            return None
        rel = filename.replace("\\", "/").lstrip("/")
        if not rel or any(part in ("..", "") for part in rel.split("/")):
            return None
        root = os.path.normpath(self.meshes_dir)
        candidate = os.path.normpath(os.path.join(root, *rel.split("/")))
        if candidate != root and not candidate.startswith(root + os.sep):
            return None
        if os.path.isfile(candidate):
            return candidate
        # Legacy flat install: same file, path prefix stripped.
        flat = os.path.normpath(os.path.join(root, posixpath.basename(rel)))
        if flat.startswith(root + os.sep) and os.path.isfile(flat):
            return flat
        return None

    # ── parsed models ───────────────────────────────────────────────

    def load_urdf(self) -> Optional[UrdfModel]:
        path = self.get_urdf_path()
        if not path:
            return None
        try:
            with open(path, "rb") as fh:
                return UrdfModel.parse(fh.read())
        except (OSError, ET.ParseError) as e:
            self._log("warn", f"load_urdf: {e}")
            return None

    def load_srdf(self) -> Optional[SRDF]:
        path = self.get_srdf_path()
        if not path:
            return None
        try:
            with open(path, "rb") as fh:
                return SRDF.parse(fh.read())
        except (OSError, SRDFError) as e:
            self._log("warn", f"load_srdf: {e}")
            return None

    def load_rig(self) -> Optional[Rig]:
        path = self.get_rig_path()
        if not path:
            return None
        try:
            with open(path, "rb") as fh:
                return Rig.parse(fh.read())
        except (OSError, RigError) as e:
            self._log("warn", f"load_rig: {e}")
            return None

    # ── queries for the UI ──────────────────────────────────────────

    def list_joints(self) -> List[Dict[str, Any]]:
        """Actuatable joints in the installed URDF, with their limits.

        Powers the joint picker in the routing UI's Add Input modal and
        the animation editor's + Joint dropdown. Skips fixed joints since
        they accept no setpoint — they're structural (a control anchor
        frame is a massless link on a fixed joint).

        Limits ride along because every consumer that shows a joint also
        needs them: the normalized −1..+1 the rest of the stack speaks is
        defined by them, so a UI that wants to display native radians has
        to have them to convert back.
        """
        urdf = self.load_urdf()
        if urdf is None:
            return []
        out = []
        for j in urdf.actuatable_joints():
            lower, upper = j.range()
            out.append({
                "name": j.name,
                "type": j.type,
                "lower": lower,
                "upper": upper,
                "has_limits": j.has_limits,
                "mimics": j.mimic.joint if j.mimic else "",
            })
        return out

    def list_groups(self) -> List[Dict[str, Any]]:
        """SRDF joint groups, expanded to their actuatable joint lists."""
        urdf, srdf = self.load_urdf(), self.load_srdf()
        if urdf is None or srdf is None:
            return []
        return [
            {"name": g.name,
             "joints": srdf.resolve_group_joints(g.name, urdf)}
            for g in srdf.groups
        ]

    def list_group_states(self) -> List[Dict[str, Any]]:
        """SRDF group_states as importable pose candidates.

        Each entry carries BOTH the native values (as authored) and the
        normalized −1..+1 the pose library stores, plus any joints that
        didn't resolve against the URDF. The import UI needs all three:
        the native values to show the operator what the file says, the
        normalized ones to actually save, and the unresolved list to warn
        that a pose will be partial.
        """
        urdf, srdf = self.load_urdf(), self.load_srdf()
        if urdf is None or srdf is None:
            return []
        out = []
        for gs in srdf.group_states:
            normalized, unresolved = gs.normalized_values(urdf)
            out.append({
                "name": gs.name,
                "group": gs.group,
                "joint_values": dict(gs.joint_values),   # native (rad/m)
                "normalized": normalized,                # −1..+1
                "unresolved": unresolved,
                "joint_count": len(gs.joint_values),
            })
        return out

    def validate(self, pose_names: Optional[List[str]] = None) -> List[str]:
        """Every unresolved reference across the installed model.

        The whole point: an SRDF and a rig file are almost entirely
        references into the URDF, and an unresolved one is a silent no-op
        rather than a load error. A group_state naming a renamed joint
        just stops moving it, with nothing in any log.

        ``pose_names`` should be the actual pose-library ids when the
        caller has them (StateManager does; this store doesn't own the
        pose store). Absent that, rig pose references are checked against
        the SRDF's group_states — under BOTH their authored names and
        their slugs, because importing "Very Happy" produces a pose whose
        id is ``very_happy`` and a rig may legitimately name either.
        """
        problems: List[str] = []
        urdf = self.load_urdf()
        if urdf is None:
            return ["no URDF installed"]
        problems.extend(urdf.validate())

        srdf = self.load_srdf()
        if srdf is not None:
            problems.extend(f"SRDF: {p}" for p in srdf.validate(urdf))

        rig = self.load_rig()
        if rig is not None:
            problems.extend(
                f"rig: {p}"
                for p in rig.validate(urdf, srdf,
                                      self._known_pose_names(srdf, pose_names)))
        return problems

    @staticmethod
    def _known_pose_names(srdf: Optional[SRDF],
                          pose_names: Optional[List[str]]) -> List[str]:
        """Every spelling a rig may use to name a pose.

        ``PoseStore`` slugifies on lookup, so ``get("Very Happy")`` finds
        ``very_happy.json`` — the runtime is forgiving about which form a
        rig file uses. Validation has to be equally forgiving, or it
        reports a reference as broken that resolves perfectly well at
        playback.
        """
        from saint_server.animation.store import slugify

        known: List[str] = list(pose_names or [])
        for name in list(known):
            slug = slugify(name)
            if slug and slug not in known:
                known.append(slug)
        if srdf is not None:
            for gs in srdf.group_states:
                for form in (gs.name, slugify(gs.name)):
                    if form and form not in known:
                        known.append(form)
        return known

    def describe(self, pose_names: Optional[List[str]] = None) -> Dict[str, Any]:
        """Full picture for the settings UI and the /api/robot/metadata
        response: metadata plus each companion file's parsed summary.

        Pass ``pose_names`` (the pose-library ids) when available so rig
        pose references are checked against what actually exists rather
        than against the SRDF alone — a pose authored in the UI never
        appears in a group_state.
        """
        meta = self.get_metadata()
        if meta is None or not self.has_model():
            return {"installed": False}

        out: Dict[str, Any] = {"installed": True, **meta.to_dict()}
        srdf = self.load_srdf()
        out["srdf"] = srdf.summary() if srdf else None
        rig = self.load_rig()
        out["rig"] = rig.summary() if rig else None
        out["warnings"] = self.validate(pose_names)
        return out

    # ── installing the URDF (full replacement) ──────────────────────

    def install_from_zip(self, zip_bytes: bytes,
                         original_filename: str) -> RobotModelMetadata:
        """Replace the model with the contents of a zip bundle.

        The bundle must contain a URDF; an SRDF and a rig file are
        optional. Mesh files outside ALLOWED_MESH_EXTENSIONS are
        silently dropped (logged).
        """
        if not zip_bytes:
            raise RobotModelError("empty upload")
        try:
            zf = zipfile.ZipFile(io.BytesIO(zip_bytes))
        except zipfile.BadZipFile as e:
            raise RobotModelError(f"not a valid zip file: {e}") from e

        urdf_member: Optional[Tuple[str, str]] = None
        srdf_member: Optional[Tuple[str, str]] = None
        rig_member: Optional[Tuple[str, str]] = None
        mesh_members: List[Tuple[str, str]] = []

        for member in zf.namelist():
            if member.endswith("/"):
                continue
            # Reject any entry whose path tries to escape the bundle root.
            if member.startswith("/") or ".." in member.split("/"):
                self._log("warning", f"Rejecting unsafe zip entry: {member}")
                continue
            base = os.path.basename(member)
            ext = os.path.splitext(base)[1].lower()

            if ext in _XML_EXTENSIONS or base.lower().endswith(".rig.xml"):
                # Content sniff, because URDF and SRDF are
                # indistinguishable by root tag and a bundle may name
                # either one oddly (robot.srdf.xacro, model.xml, …).
                try:
                    data = zf.read(member)
                except Exception as e:
                    self._log("warning", f"Unreadable zip entry {member}: {e}")
                    continue
                kind = _classify_xml(data, base)
                if kind == "":
                    # Unclassifiable. If the extension explicitly claims
                    # a description file, this is broken input and must
                    # not be quietly dropped — installing a URDF next to
                    # an SRDF that will silently never bind is exactly
                    # the failure mode we can't debug later. A generic
                    # `.xml` is more likely an unrelated stray, so it
                    # only warns.
                    if _claims_description_file(base):
                        raise RobotModelError(
                            f"'{member}' has a {ext} extension but is not a "
                            f"parseable URDF, SRDF, or rig file")
                    self._log("warning",
                              f"Ignoring '{member}': not a recognizable "
                              f"URDF, SRDF, or rig file")
                    continue
                if kind == "rig" and rig_member is None:
                    rig_member = (member, base)
                elif kind == "srdf" and srdf_member is None:
                    srdf_member = (member, base)
                elif kind == "urdf" and urdf_member is None:
                    # Prefer the first URDF found. Multiple .urdf files
                    # in one bundle is unusual and we don't try to be
                    # clever.
                    urdf_member = (member, base)
                continue

            if ext in ALLOWED_MESH_EXTENSIONS:
                mesh_members.append((member, base))

        if urdf_member is None:
            raise RobotModelError(
                "zip contains no URDF at any level (looked for a <robot> "
                "document with <link>/<joint> geometry)")

        urdf_data = zf.read(urdf_member[0])
        urdf = _parse_urdf_or_raise(urdf_data)

        # Stage to a temp dir then atomically swap — protects the live
        # model if the operator uploads a broken bundle.
        tmp_dir = self.robot_dir + ".staging"
        if os.path.isdir(tmp_dir):
            shutil.rmtree(tmp_dir)
        os.makedirs(os.path.join(tmp_dir, "meshes"), exist_ok=True)

        urdf_target_name = urdf_member[1]
        with open(os.path.join(tmp_dir, urdf_target_name), "wb") as f:
            f.write(urdf_data)

        mesh_names = self._stage_meshes(
            zf, mesh_members, posixpath.dirname(urdf_member[0]), tmp_dir)

        meta = RobotModelMetadata(
            original_filename=original_filename,
            urdf_filename=urdf_target_name,
            sha256=hashlib.sha256(urdf_data).hexdigest(),
            uploaded_at=time.time(),
            mesh_files=sorted(mesh_names),
            link_count=len(urdf.links),
            joint_count=len(urdf.joints),
            robot_name=urdf.robot_name,
        )

        # Companion files from the bundle, if present.
        if srdf_member is not None:
            data = zf.read(srdf_member[0])
            try:
                SRDF.parse(data)         # fail the whole install on bad XML
            except SRDFError as e:
                raise RobotModelError(f"bundled SRDF is invalid: {e}") from e
            with open(os.path.join(tmp_dir, srdf_member[1]), "wb") as f:
                f.write(data)
            meta.srdf_filename = srdf_member[1]
            meta.srdf_sha256 = hashlib.sha256(data).hexdigest()
            meta.srdf_uploaded_at = time.time()

        if rig_member is not None:
            data = zf.read(rig_member[0])
            try:
                Rig.parse(data)
            except RigError as e:
                raise RobotModelError(f"bundled rig file is invalid: {e}") from e
            with open(os.path.join(tmp_dir, rig_member[1]), "wb") as f:
                f.write(data)
            meta.rig_filename = rig_member[1]
            meta.rig_sha256 = hashlib.sha256(data).hexdigest()
            meta.rig_uploaded_at = time.time()

        self._carry_over_companions(meta, tmp_dir)
        self._write_metadata(meta, tmp_dir)
        self._swap_in(tmp_dir)

        # Recompute warnings now that everything is in its final place
        # (validation needs to read the companion files back).
        meta.warnings = self.validate()
        self._write_metadata(meta)

        self._log("info",
                  f"Robot model installed: {urdf_target_name} "
                  f"({meta.link_count} links, {meta.joint_count} joints, "
                  f"{len(mesh_names)} meshes"
                  f"{', SRDF' if meta.has_srdf else ''}"
                  f"{', rig' if meta.has_rig else ''})")
        if meta.warnings:
            self._log("warn",
                      f"Robot model installed with {len(meta.warnings)} "
                      f"unresolved reference(s); first: {meta.warnings[0]}")
        return meta

    def install_from_urdf(self, urdf_bytes: bytes,
                          original_filename: str) -> RobotModelMetadata:
        """Install a single URDF file (no meshes).

        Useful for primitive-only robots and tests. For real robots the
        operator should upload a zip.
        """
        urdf = _parse_urdf_or_raise(urdf_bytes)

        urdf_target_name = os.path.basename(original_filename) or "robot.urdf"
        if not urdf_target_name.lower().endswith((".urdf", ".xacro")):
            urdf_target_name = "robot.urdf"

        tmp_dir = self.robot_dir + ".staging"
        if os.path.isdir(tmp_dir):
            shutil.rmtree(tmp_dir)
        os.makedirs(os.path.join(tmp_dir, "meshes"), exist_ok=True)
        with open(os.path.join(tmp_dir, urdf_target_name), "wb") as f:
            f.write(urdf_bytes)

        meta = RobotModelMetadata(
            original_filename=original_filename,
            urdf_filename=urdf_target_name,
            sha256=hashlib.sha256(urdf_bytes).hexdigest(),
            uploaded_at=time.time(),
            mesh_files=[],
            link_count=len(urdf.links),
            joint_count=len(urdf.joints),
            robot_name=urdf.robot_name,
        )
        self._carry_over_companions(meta, tmp_dir)
        self._write_metadata(meta, tmp_dir)
        self._swap_in(tmp_dir)

        meta.warnings = self.validate()
        self._write_metadata(meta)
        self._log("info", f"URDF installed (no meshes): {urdf_target_name}")
        return meta

    # ── installing companions (in place) ────────────────────────────

    def install_srdf(self, srdf_bytes: bytes,
                     original_filename: str) -> Dict[str, Any]:
        """Install an SRDF alongside the existing URDF.

        Writes in place — an SRDF is an annotation layer, and staging a
        whole-tree swap to accept one would risk the URDF for no reason.

        Returns a dict carrying the refreshed metadata, the SRDF summary,
        validation warnings, and the group_states available for import as
        poses, so the UI can prompt in one round trip.
        """
        if not self.has_model():
            raise RobotModelError(
                "no URDF installed — an SRDF is an annotation layer and "
                "every name in it is a dangling reference on its own")
        try:
            srdf = SRDF.parse(srdf_bytes)
        except SRDFError as e:
            raise RobotModelError(str(e)) from e

        name = os.path.basename(original_filename) or "robot.srdf"
        if not name.lower().endswith((".srdf", ".xacro", ".xml")):
            name = "robot.srdf"
        name = _safe_basename(name)

        meta = self.get_metadata() or RobotModelMetadata()
        # Drop a previous SRDF under a different filename, or the old one
        # keeps being served by its own URL.
        self._remove_companion(meta.srdf_filename, name)

        with open(os.path.join(self.robot_dir, name), "wb") as f:
            f.write(srdf_bytes)

        meta.srdf_filename = name
        meta.srdf_sha256 = hashlib.sha256(srdf_bytes).hexdigest()
        meta.srdf_uploaded_at = time.time()
        meta.warnings = self.validate()
        self._write_metadata(meta)

        urdf = self.load_urdf()
        problems = srdf.validate(urdf) if urdf else []
        self._log("info",
                  f"SRDF installed: {name} "
                  f"({len(srdf.groups)} groups, "
                  f"{len(srdf.group_states)} group_states"
                  f"{f', {len(problems)} unresolved' if problems else ''})")

        return {
            "installed": True,
            "srdf_filename": name,
            "summary": srdf.summary(),
            "warnings": problems,
            "group_states": self.list_group_states(),
            "metadata": self.describe(),
        }

    def install_rig(self, rig_bytes: bytes,
                    original_filename: str) -> Dict[str, Any]:
        """Install a rig file alongside the existing URDF."""
        if not self.has_model():
            raise RobotModelError(
                "no URDF installed — a rig file's joints and links are "
                "dangling references on their own")
        try:
            rig = Rig.parse(rig_bytes)
        except RigError as e:
            raise RobotModelError(str(e)) from e

        name = os.path.basename(original_filename) or "robot.rig.xml"
        if not name.lower().endswith((".xml", ".rig")):
            name = "robot.rig.xml"
        name = _safe_basename(name)

        meta = self.get_metadata() or RobotModelMetadata()
        self._remove_companion(meta.rig_filename, name)

        with open(os.path.join(self.robot_dir, name), "wb") as f:
            f.write(rig_bytes)

        meta.rig_filename = name
        meta.rig_sha256 = hashlib.sha256(rig_bytes).hexdigest()
        meta.rig_uploaded_at = time.time()
        meta.warnings = self.validate()
        self._write_metadata(meta)

        urdf, srdf = self.load_urdf(), self.load_srdf()
        pose_names = [gs.name for gs in (srdf.group_states if srdf else [])]
        problems = rig.validate(urdf, srdf, pose_names)
        self._log("info",
                  f"Rig installed: {name} ({len(rig.controls)} controls"
                  f"{f', {len(problems)} unresolved' if problems else ''})")

        return {
            "installed": True,
            "rig_filename": name,
            "summary": rig.summary(),
            "warnings": problems,
            "metadata": self.describe(),
        }

    # ── deletion ────────────────────────────────────────────────────

    def delete(self) -> bool:
        """Remove the whole installed model. True if something went."""
        if os.path.isdir(self.robot_dir):
            shutil.rmtree(self.robot_dir)
            self._log("info", "Robot model deleted")
            return True
        return False

    def delete_srdf(self) -> bool:
        return self._delete_companion("srdf")

    def delete_rig(self) -> bool:
        return self._delete_companion("rig")

    def _delete_companion(self, which: str) -> bool:
        meta = self.get_metadata()
        if meta is None:
            return False
        attr = f"{which}_filename"
        name = getattr(meta, attr, "")
        if not name:
            return False
        path = os.path.join(self.robot_dir, name)
        if os.path.isfile(path):
            os.unlink(path)
        setattr(meta, attr, "")
        setattr(meta, f"{which}_sha256", "")
        setattr(meta, f"{which}_uploaded_at", 0.0)
        meta.warnings = self.validate()
        self._write_metadata(meta)
        self._log("info", f"{which.upper()} removed: {name}")
        return True

    # ── internals ───────────────────────────────────────────────────

    def _stage_meshes(self, zf: zipfile.ZipFile,
                      mesh_members: List[Tuple[str, str]],
                      urdf_dir: str, tmp_dir: str) -> List[str]:
        """Write each mesh under its path RELATIVE TO THE URDF.

        Mirrors how the URDF references it. The old behavior flattened to
        basename with last-write-wins, which silently collapsed distinct
        meshes sharing a filename across subfolders (johnny5's
        SimpleMouth/static_97a3da.stl vs
        SimplifiedHead2/static_97a3da.stl) — one geometry then rendered
        twice and the other vanished.
        """
        mesh_names: List[str] = []
        seen_targets: Dict[str, str] = {}
        for member, _base in mesh_members:
            rel = posixpath.relpath(member, urdf_dir) if urdf_dir else member
            if ".." in rel.split("/"):
                # Mesh outside the URDF's directory — a URDF can't
                # reference it without ".." (which we reject on the
                # serving side anyway). Keep it reachable by its
                # bundle-rooted path.
                rel = member.lstrip("/")
            key = rel.lower()   # macOS/Windows checkouts are case-insensitive
            if key in seen_targets:
                self._log("warning",
                          f"Bundle: mesh path collision — '{member}' and "
                          f"'{seen_targets[key]}' both install as '{rel}'; "
                          "keeping the last one. Rename one file in the "
                          "bundle to keep both.")
            seen_targets[key] = member
            target = os.path.join(tmp_dir, "meshes", *rel.split("/"))
            os.makedirs(os.path.dirname(target), exist_ok=True)
            with open(target, "wb") as f:
                f.write(zf.read(member))
            mesh_names.append(rel)
        return mesh_names

    def _carry_over_companions(self, meta: RobotModelMetadata,
                               tmp_dir: str) -> None:
        """Preserve an existing SRDF/rig across a URDF replacement.

        A new URDF may well invalidate them — every name they carry is a
        reference — but deleting an operator's rig because they re-exported
        the geometry would be hostile. They're copied forward and
        re-validated instead, so a rename surfaces as warnings the
        operator can act on rather than as silent data loss.

        Skipped for whichever companion the incoming bundle supplies, so
        a bundled file always wins over the carried-over one.
        """
        old = self.get_metadata()
        if old is None:
            return
        for which in ("srdf", "rig"):
            if getattr(meta, f"{which}_filename"):
                continue        # the bundle supplied one; it wins
            name = getattr(old, f"{which}_filename", "")
            if not name:
                continue
            src = os.path.join(self.robot_dir, name)
            if not os.path.isfile(src):
                continue
            shutil.copy2(src, os.path.join(tmp_dir, name))
            setattr(meta, f"{which}_filename", name)
            setattr(meta, f"{which}_sha256", getattr(old, f"{which}_sha256", ""))
            setattr(meta, f"{which}_uploaded_at",
                    getattr(old, f"{which}_uploaded_at", 0.0))
            self._log("info",
                      f"Carried existing {which.upper()} '{name}' across the "
                      f"URDF replacement; re-validating against the new model")

    def _swap_in(self, tmp_dir: str) -> None:
        if os.path.isdir(self.robot_dir):
            shutil.rmtree(self.robot_dir)
        os.rename(tmp_dir, self.robot_dir)

    def _remove_companion(self, old_name: str, new_name: str) -> None:
        """Delete a superseded companion whose filename differs.

        Without this, replacing ``head.srdf`` with ``robot.srdf`` leaves
        the old file on disk where the fallback scan can still find it.
        """
        if not old_name or old_name == new_name:
            return
        path = os.path.join(self.robot_dir, old_name)
        if os.path.isfile(path):
            os.unlink(path)

    def _find_urdf(self) -> Optional[str]:
        """Locate the URDF on disk via metadata, or by scanning."""
        meta = self.get_metadata()
        if meta and meta.urdf_filename:
            path = os.path.join(self.robot_dir, meta.urdf_filename)
            if os.path.isfile(path):
                return path
        # Fallback scan — the metadata file might be missing on
        # hand-edited installs. Content-sniff so we don't hand back an
        # SRDF that happens to sort first.
        if os.path.isdir(self.robot_dir):
            for name in sorted(os.listdir(self.robot_dir)):
                if not name.lower().endswith((".urdf", ".xacro")):
                    continue
                path = os.path.join(self.robot_dir, name)
                try:
                    with open(path, "rb") as fh:
                        if _classify_xml(fh.read(), name) == "urdf":
                            return path
                except OSError:
                    continue
        return None

    def _find_companion(self, meta_attr: str,
                        suffixes: Tuple[str, ...]) -> Optional[str]:
        meta = self.get_metadata()
        if meta:
            name = getattr(meta, meta_attr, "")
            if name:
                path = os.path.join(self.robot_dir, name)
                if os.path.isfile(path):
                    return path
        if os.path.isdir(self.robot_dir):
            for name in sorted(os.listdir(self.robot_dir)):
                if name.lower().endswith(suffixes):
                    return os.path.join(self.robot_dir, name)
        return None

    def _log(self, level: str, msg: str) -> None:
        if self.logger:
            from saint_server.log_level import log_at
            log_at(self.logger, level, msg)


# ── module helpers ──────────────────────────────────────────────────


def _claims_description_file(filename: str) -> bool:
    """True if the filename asserts it's one of the three description
    files, as opposed to being an incidental ``.xml``.

    Drives the difference between "this bundle is broken, refuse it" and
    "there's a stray XML in here, ignore it".
    """
    lowered = filename.lower()
    return lowered.endswith((".urdf", ".srdf", ".xacro", ".rig", ".rig.xml"))


def _safe_basename(name: str) -> str:
    """Strip anything that could escape robot_dir."""
    base = os.path.basename(name.replace("\\", "/"))
    return base if base not in ("", ".", "..") else "robot.xml"


def _classify_xml(data: bytes, filename: str) -> str:
    """Classify an XML upload as ``urdf``, ``srdf``, ``rig``, or ``""``.

    Content first, extension only as a tiebreak — URDF and SRDF share
    the ``<robot>`` root element and are indistinguishable by tag, so
    the filename is the least trustworthy signal available. A bundle
    shipping ``robot.srdf.xacro`` or a bare ``model.xml`` still routes
    correctly this way.
    """
    if looks_like_rig(data):
        return "rig"
    if looks_like_srdf(data):
        return "srdf"
    try:
        root = ET.fromstring(data)
    except ET.ParseError:
        return ""
    if root.tag != "robot":
        return ""
    # A <robot> with no SRDF-only children. If it carries structural
    # link/joint content it's a URDF; otherwise fall back to extension,
    # which covers a nearly-empty SRDF stub.
    if root.find("link") is not None or root.find("joint") is not None:
        return "urdf"
    lowered = filename.lower()
    if lowered.endswith(".srdf"):
        return "srdf"
    return "urdf"


def _parse_urdf_or_raise(urdf_bytes: bytes) -> UrdfModel:
    """Parse a URDF, raising RobotModelError with an operator-readable
    message on anything malformed.

    Gates on the root element only. An *unexpanded* xacro legitimately
    has zero direct ``<link>`` children — they're generated by macros at
    expansion time — so an empty model is not grounds for rejection. The
    one thing worth catching precisely is an SRDF uploaded as a URDF,
    since both root at ``<robot>`` and the operator gets no other clue.
    """
    try:
        root = ET.fromstring(urdf_bytes)
    except ET.ParseError as e:
        raise RobotModelError(f"URDF is not valid XML: {e}") from e

    if root.tag != "robot":
        raise RobotModelError(
            f"URDF root element must be <robot>, got <{root.tag}>")

    if looks_like_srdf(urdf_bytes):
        raise RobotModelError(
            "this looks like an SRDF, not a URDF — it carries <group> / "
            "<group_state> annotations but no structure. An SRDF "
            "supplements a URDF and can't stand in for one; upload it "
            "with the SRDF endpoint instead.")

    try:
        return UrdfModel.parse(urdf_bytes)
    except Exception as e:
        raise RobotModelError(f"URDF could not be parsed: {e}") from e
