#!/usr/bin/env python3
"""Robot-model operations for the JS mock server, as a JSON-in/JSON-out CLI.

Used by `web/dev/mock-http.js` and `mock-handlers.js` so the dev mock
parses SRDFs, validates cross-file references, and **evaluates the rig**
with the real Python implementation instead of a JS mirror. Same
rationale as `dump_catalog.py`: a hand-maintained JS copy of the blend
math, the clamp policy, and the mimic round-trip would silently drift,
and then the mock would be demonstrating the wrong thing.

Stateless by design. The mock owns the file bytes (it already does for
the URDF); this script is pure computation:

    echo '{"urdf": "<robot .../>"}' | robot_model_bridge.py joints

Every subcommand reads one JSON object on stdin and writes one on
stdout. Errors come back as `{"error": "..."}` with exit 0, so the mock
can surface the message rather than parsing a traceback off stderr.

Standalone: registers stub parent packages in sys.modules so the
submodules import without executing `saint_server/__init__.py`, which
pulls in rclpy. Runs on any machine with plain Python 3.
"""

from __future__ import annotations

import json
import os
import sys
import types


def _bootstrap_packages() -> None:
    """Make `saint_server.animation.*` importable without rclpy.

    `saint_server/__init__.py` imports server_node, which imports rclpy.
    Registering the parent packages by hand — with __path__ set but no
    __init__ executed — lets the submodules resolve each other normally
    while skipping that.
    """
    here = os.path.dirname(os.path.abspath(__file__))
    root = os.path.normpath(os.path.join(here, ".."))
    if root not in sys.path:
        sys.path.insert(0, root)

    for name, rel in (
        ("saint_server", "saint_server"),
        ("saint_server.animation", "saint_server/animation"),
        ("saint_server.unreal", "saint_server/unreal"),
    ):
        if name in sys.modules:
            continue
        pkg = types.ModuleType(name)
        pkg.__path__ = [os.path.join(root, rel)]
        sys.modules[name] = pkg


_bootstrap_packages()

from saint_server.animation.rig import Rig, RigError, looks_like_rig  # noqa: E402
from saint_server.animation.rig_eval import RigEvaluator  # noqa: E402
from saint_server.animation.srdf import SRDF, SRDFError, looks_like_srdf  # noqa: E402
from saint_server.animation.urdf_model import UrdfModel  # noqa: E402


# ── helpers ─────────────────────────────────────────────────────────


def _slugify(name: str) -> str:
    """Mirror of animation.store.slugify, inlined to keep this script's
    import surface to the four parser modules."""
    import re
    s = (name or "").strip().lower().replace(" ", "_")
    s = re.sub(r"[^a-z0-9_-]+", "", s)
    s = re.sub(r"_+", "_", s).strip("_-")
    return "untitled" if s in ("", ".", "..") else s[:64]


def _urdf(payload: dict):
    text = payload.get("urdf")
    if not text:
        return None
    return UrdfModel.parse(text.encode() if isinstance(text, str) else text)


def _srdf(payload: dict):
    text = payload.get("srdf")
    if not text:
        return None
    return SRDF.parse(text.encode() if isinstance(text, str) else text)


def _rig(payload: dict):
    text = payload.get("rig")
    if not text:
        return None
    return Rig.parse(text.encode() if isinstance(text, str) else text)


def _known_pose_names(srdf, pose_names) -> list:
    """Every spelling a rig may use to name a pose — matching
    RobotModelStore._known_pose_names, since the mock has to report the
    same warnings the real server would."""
    known = list(pose_names or [])
    for name in list(known):
        slug = _slugify(name)
        if slug and slug not in known:
            known.append(slug)
    if srdf is not None:
        for gs in srdf.group_states:
            for form in (gs.name, _slugify(gs.name)):
                if form and form not in known:
                    known.append(form)
    return known


def _validate(urdf, srdf, rig, pose_names) -> list:
    problems = []
    if urdf is None:
        return ["no URDF installed"]
    problems.extend(urdf.validate())
    if srdf is not None:
        problems.extend(f"SRDF: {p}" for p in srdf.validate(urdf))
    if rig is not None:
        problems.extend(
            f"rig: {p}"
            for p in rig.validate(urdf, srdf, _known_pose_names(srdf, pose_names)))
    return problems


# ── subcommands ─────────────────────────────────────────────────────


def cmd_classify(payload: dict) -> dict:
    """Route an upload. URDF and SRDF share the <robot> root, so content
    decides and the filename is only a tiebreak."""
    data = payload.get("data") or ""
    raw = data.encode() if isinstance(data, str) else data
    filename = (payload.get("filename") or "").lower()
    if looks_like_rig(raw):
        return {"kind": "rig"}
    if looks_like_srdf(raw):
        return {"kind": "srdf"}
    try:
        model = UrdfModel.parse(raw)
    except Exception as e:
        return {"kind": "", "error": str(e)}
    if model.links or model.joints:
        return {"kind": "urdf"}
    return {"kind": "srdf" if filename.endswith(".srdf") else "urdf"}


def cmd_joints(payload: dict) -> dict:
    urdf = _urdf(payload)
    if urdf is None:
        return {"joints": []}
    out = []
    for j in urdf.actuatable_joints():
        lower, upper = j.range()
        out.append({
            "name": j.name, "type": j.type,
            "lower": lower, "upper": upper,
            "has_limits": j.has_limits,
            "mimics": j.mimic.joint if j.mimic else "",
        })
    return {"joints": out}


def cmd_groups(payload: dict) -> dict:
    urdf, srdf = _urdf(payload), _srdf(payload)
    if urdf is None or srdf is None:
        return {"groups": []}
    return {"groups": [
        {"name": g.name, "joints": srdf.resolve_group_joints(g.name, urdf)}
        for g in srdf.groups
    ]}


def cmd_group_states(payload: dict) -> dict:
    """Importable pose candidates, carrying BOTH unit systems.

    The import dialog needs the native values to show what the file
    says, the normalized ones to save, and the unresolved list to warn
    that a pose will land partial.
    """
    urdf, srdf = _urdf(payload), _srdf(payload)
    if urdf is None or srdf is None:
        return {"group_states": []}
    existing = set(payload.get("existing_pose_ids") or [])
    out = []
    for gs in srdf.group_states:
        normalized, unresolved = gs.normalized_values(urdf)
        pose_id = _slugify(gs.name)
        out.append({
            "name": gs.name,
            "group": gs.group,
            "joint_values": dict(gs.joint_values),
            "normalized": normalized,
            "unresolved": unresolved,
            "joint_count": len(gs.joint_values),
            "pose_id": pose_id,
            "exists": pose_id in existing,
            "locally_edited": False,
        })
    return {"group_states": out}


def cmd_describe(payload: dict) -> dict:
    urdf = _urdf(payload)
    if urdf is None:
        return {"installed": False}
    srdf, rig = _srdf(payload), _rig(payload)
    return {
        "installed": True,
        "robot_name": urdf.robot_name,
        "link_count": len(urdf.links),
        "joint_count": len(urdf.joints),
        "srdf": srdf.summary() if srdf else None,
        "rig": rig.summary() if rig else None,
        "warnings": _validate(urdf, srdf, rig, payload.get("pose_names")),
    }


def cmd_rig(payload: dict) -> dict:
    rig = _rig(payload)
    if rig is None:
        return {"success": True, "rig": None}
    urdf, srdf = _urdf(payload), _srdf(payload)

    # Anchors: which link each control's 3D shape hangs off. Must match
    # what state_manager.get_rig returns, or the mock would place shapes
    # differently from the real server — the exact divergence this whole
    # bridge exists to prevent.
    #
    # `poses` is optional: without it, a control that only blends poses
    # (no <drive>) can't be placed. The SRDF's group_states are the same
    # data under their pre-slug names, so they're a usable fallback when
    # the caller didn't send the pose library.
    poses = payload.get("poses") or {}
    if not poses and srdf is not None and urdf is not None:
        for gs in srdf.group_states:
            normalized, _ = gs.normalized_values(urdf)
            if normalized:
                poses[gs.name] = normalized
                poses.setdefault(_slugify(gs.name), normalized)

    return {
        "success": True,
        "rig": rig.to_dict(),
        "defaults": RigEvaluator(rig, urdf=urdf).control_defaults(),
        "anchors": rig.resolve_anchors_with_poses(urdf, poses),
        "warnings": _validate(urdf, srdf, rig, payload.get("pose_names"))
        if urdf is not None else [],
    }


def cmd_evaluate_rig(payload: dict) -> dict:
    """Evaluate the rig at the given control values.

    `poses` maps pose name → {joint: normalized}; the mock passes only
    the poses the rig references, same as the real server does.
    """
    rig = _rig(payload)
    if rig is None:
        return {"success": False, "message": "No rig file installed"}
    urdf = _urdf(payload)
    poses = payload.get("poses") or {}
    values = {}
    for key, raw in (payload.get("values") or {}).items():
        try:
            values[str(key)] = float(raw)
        except (TypeError, ValueError):
            continue
    frame = RigEvaluator(rig, urdf=urdf, poses=poses).evaluate(values)
    return {"success": True, **frame.to_dict()}


def cmd_referenced_poses(payload: dict) -> dict:
    """Pose names a rig needs, so the mock knows which to send back in
    `evaluate_rig`."""
    rig = _rig(payload)
    if rig is None:
        return {"poses": [], "neutral": ""}
    return {"poses": rig.referenced_poses(),
            "neutral": rig.settings.neutral_pose}


COMMANDS = {
    "classify": cmd_classify,
    "joints": cmd_joints,
    "groups": cmd_groups,
    "group_states": cmd_group_states,
    "describe": cmd_describe,
    "rig": cmd_rig,
    "evaluate_rig": cmd_evaluate_rig,
    "referenced_poses": cmd_referenced_poses,
}


def _run(command: str, payload: dict) -> dict:
    if command not in COMMANDS:
        return {"error": f"unknown command: {command}"}
    try:
        return COMMANDS[command](payload)
    except (SRDFError, RigError) as e:
        return {"error": str(e)}
    except Exception as e:                       # noqa: BLE001
        return {"error": f"{type(e).__name__}: {e}"}


def serve() -> int:
    """Long-lived worker mode: one JSON request per line on stdin, one
    JSON response per line on stdout.

    Exists because the one-shot mode costs ~46 ms of interpreter startup
    and imports per call, and the caller (the dev mock's rig panel) fires
    on every slider input event — around 30 a second. Paying process
    startup per tick put the mock ~2.7x behind realtime, which surfaced
    as request timeouts once the backlog passed 30 s.

    Request:  {"id": 7, "cmd": "evaluate_rig", "payload": {...}}
    Response: {"id": 7, "result": {...}}

    Never exits on a bad request — a malformed line gets an error
    response so the caller stays in sync on ids. Terminates on EOF.
    """
    out = sys.stdout
    for line in sys.stdin:
        line = line.strip()
        if not line:
            continue
        req_id = None
        try:
            req = json.loads(line)
            req_id = req.get("id")
            result = _run(str(req.get("cmd") or ""), req.get("payload") or {})
        except json.JSONDecodeError as e:
            result = {"error": f"bad JSON: {e}"}
        except Exception as e:                   # noqa: BLE001
            result = {"error": f"{type(e).__name__}: {e}"}
        # One line per response, flushed — the caller reads line-delimited
        # and would otherwise block on a half-buffered reply.
        out.write(json.dumps({"id": req_id, "result": result}) + "\n")
        out.flush()
    return 0


def main() -> int:
    if len(sys.argv) < 2 or (sys.argv[1] not in COMMANDS
                             and sys.argv[1] != "serve"):
        sys.stderr.write(
            f"usage: {os.path.basename(sys.argv[0])} "
            f"<{'|'.join(sorted(COMMANDS))}|serve>  # JSON on stdin\n")
        return 2

    if sys.argv[1] == "serve":
        return serve()

    raw = sys.stdin.read()
    try:
        payload = json.loads(raw) if raw.strip() else {}
    except json.JSONDecodeError as e:
        json.dump({"error": f"bad JSON on stdin: {e}"}, sys.stdout)
        return 0

    json.dump(_run(sys.argv[1], payload), sys.stdout)
    return 0


if __name__ == "__main__":
    sys.exit(main())
