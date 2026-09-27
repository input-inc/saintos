"""Data classes for poses, animations, value tracks, and trigger tracks.

Wraps the existing curve infrastructure in
``saint_server/unreal/animation.py`` (CurveKey, AnimationCurve,
CurveInterpolation) so we don't reinvent keyframe interpolation —
that module's been waiting for a consumer since it was authored.
"""

from __future__ import annotations

import time
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional

from saint_server.unreal.animation import (
    AnimationCurve,
    CurveInterpolation,
    CurveKey,
)


# ── curve (de)serialization ─────────────────────────────────────────
#
# Shared by a track's own curve and by its per-joint override curves, so
# the two can't drift on tangent or interp handling.


def _curve_to_dict(curve: AnimationCurve) -> Dict[str, Any]:
    return {
        "name": curve.name,
        "keys": [
            {
                "time": k.time,
                "value": k.value,
                "interp": int(k.interp),
                "arrive_tangent": k.arrive_tangent,
                "leave_tangent": k.leave_tangent,
            }
            for k in curve.keys
        ],
    }


def _curve_from_dict(d: Dict[str, Any], default_name: str = "") -> AnimationCurve:
    keys = [
        CurveKey(
            time=float(k.get("time", 0.0)),
            value=float(k.get("value", 0.0)),
            interp=CurveInterpolation(int(k.get("interp", 1))),
            arrive_tangent=float(k.get("arrive_tangent", 0.0)),
            leave_tangent=float(k.get("leave_tangent", 0.0)),
        )
        for k in (d.get("keys") or [])
    ]
    # Keys arrive sorted from the editor, but a hand-edited file or a
    # retimed key can break that, and every consumer walks them assuming
    # ascending time.
    keys.sort(key=lambda k: k.time)
    return AnimationCurve(name=str(d.get("name", default_name)), keys=keys)


# ── value tracks ────────────────────────────────────────────────────


@dataclass
class ValueTrack:
    """One continuous-value channel on an animation timeline.

    The track's value at any time t comes from its underlying
    AnimationCurve. ``target_kind`` selects where the sampled value is
    pushed each tick — mirroring TriggerKeyframe's target model:

      * ``"urdf_joint"`` (default) — ``track.id`` is the URDF joint
        name; the value goes to ``set_urdf_joint_value(track.id, v)``.
        Backward-compatible with every track authored before sheet
        binding existed (those tracks have no ``target_kind`` and
        deserialize to this).
      * ``"ws_input"`` — ``target`` is ``[sheet_id, ws_input_id]``; the
        value goes to ``set_ws_input(...)`` — the same path poses and
        controller gamepad bindings use. This lets an animation drive a
        controller routing-sheet input directly, so animations can be
        authored with NO URDF at all.
    """
    id: str
    name: str
    curve: AnimationCurve
    target_kind: str = "urdf_joint"
    target: List[str] = field(default_factory=list)
    # ``pose`` tracks only. joint name → curve of ABSOLUTE joint values
    # that refines one joint inside the clip, leaving the rest of the pose
    # alone. Only the operator's own keys are stored; the locked anchors
    # at the pose track's keyframe times are derived at resolve time
    # (see frame.effective_override_keys), so retiming the pose moves its
    # anchors with it instead of stranding a stale copy.
    joint_overrides: Dict[str, AnimationCurve] = field(default_factory=dict)

    def value_at(self, t: float) -> float:
        return self.curve.get_value_at_time(t)

    def to_dict(self) -> Dict[str, Any]:
        out = {
            "id": self.id,
            "name": self.name,
            "target_kind": self.target_kind,
            "target": list(self.target),
            "curve": _curve_to_dict(self.curve),
        }
        # Omitted when empty: every track authored before per-joint
        # overrides existed stays byte-identical on re-save.
        if self.joint_overrides:
            out["joint_overrides"] = {
                joint: _curve_to_dict(curve)
                for joint, curve in self.joint_overrides.items()
                if curve.keys
            }
        return out

    @classmethod
    def from_dict(cls, d: Dict[str, Any]) -> "ValueTrack":
        overrides_d = d.get("joint_overrides") or {}
        overrides = {}
        for joint, curve_d in overrides_d.items():
            curve = _curve_from_dict(curve_d, default_name=str(joint))
            if curve.keys:
                overrides[str(joint)] = curve
        return cls(
            id=str(d["id"]),
            name=str(d.get("name", "")),
            target_kind=str(d.get("target_kind", "urdf_joint")),
            target=[str(p) for p in (d.get("target") or [])],
            curve=_curve_from_dict(d.get("curve") or {}),
            joint_overrides=overrides,
        )


# ── trigger tracks ──────────────────────────────────────────────────


@dataclass
class TriggerKeyframe:
    """A discrete event that fires once at ``time``.

    ``target_kind`` selects the dispatch path:
      * ``"ws_input"`` — target is [sheet_id, ws_input_id]; the
        animation player calls routing_evaluator.set_ws_input(...) with
        ``value`` cast to float.
      * ``"topic"`` — target is [endpoint_path, field]; the animation
        player calls ros_bridge.set_topic_channel(...) with ``value``
        cast to float.
      * ``"peripheral_command"`` — target is [node_id, peripheral_id];
        ``value`` is a dict ``{"command": str, "args": dict}`` (or just
        a string filename, which is desugared to
        ``{"command": "play_file", "args": {"filename": ...}}`` so
        operators authoring an audio cue can skip the wrapper).
        Dispatched through server_node.send_peripheral_command, which
        publishes the same JSON action as a websocket-issued
        peripheral_command — i.e. an animation-fired play_file is
        wire-indistinguishable from one a UI button sent.
    """
    time: float
    target_kind: str
    target: List[str]
    value: Any
    label: str = ""

    def to_dict(self) -> Dict[str, Any]:
        return {
            "time": self.time,
            "target_kind": self.target_kind,
            "target": list(self.target),
            "value": self.value,
            "label": self.label,
        }

    @classmethod
    def from_dict(cls, d: Dict[str, Any]) -> "TriggerKeyframe":
        return cls(
            time=float(d.get("time", 0.0)),
            target_kind=str(d.get("target_kind", "ws_input")),
            target=[str(p) for p in (d.get("target") or [])],
            value=d.get("value"),
            label=str(d.get("label", "")),
        )


@dataclass
class TriggerTrack:
    id: str
    name: str
    keyframes: List[TriggerKeyframe] = field(default_factory=list)

    def fires_in(self, t_prev: float, t_now: float) -> List[TriggerKeyframe]:
        """Keyframes whose ``time`` is in (t_prev, t_now]."""
        return [
            k for k in self.keyframes
            if t_prev < k.time <= t_now
        ]

    def to_dict(self) -> Dict[str, Any]:
        return {
            "id": self.id,
            "name": self.name,
            "keyframes": [k.to_dict() for k in self.keyframes],
        }

    @classmethod
    def from_dict(cls, d: Dict[str, Any]) -> "TriggerTrack":
        return cls(
            id=str(d["id"]),
            name=str(d.get("name", "")),
            keyframes=sorted(
                [TriggerKeyframe.from_dict(k) for k in d.get("keyframes", [])],
                key=lambda k: k.time,
            ),
        )


# ── animation container ─────────────────────────────────────────────


@dataclass
class Animation:
    id: str
    name: str
    duration: float = 0.0
    fps: int = 60
    loop: bool = False
    # Material icon name — surfaced in the sidebar so operators can
    # recognize "wave hello" vs "salute" at a glance. Defaults to the
    # generic "animation" icon when unset.
    icon: str = ""
    # Single-level group (operator-defined string). Empty means
    # ungrouped — appears under an "Ungrouped" section in the UI.
    group: str = ""
    value_tracks: List[ValueTrack] = field(default_factory=list)
    trigger_tracks: List[TriggerTrack] = field(default_factory=list)
    created: str = ""
    modified: str = ""

    def to_dict(self) -> Dict[str, Any]:
        return {
            "id": self.id,
            "name": self.name,
            "duration": self.duration,
            "fps": self.fps,
            "loop": self.loop,
            "icon": self.icon,
            "group": self.group,
            "value_tracks": [t.to_dict() for t in self.value_tracks],
            "trigger_tracks": [t.to_dict() for t in self.trigger_tracks],
            "created": self.created,
            "modified": self.modified,
        }

    @classmethod
    def from_dict(cls, d: Dict[str, Any]) -> "Animation":
        return cls(
            id=str(d["id"]),
            name=str(d.get("name", "")),
            duration=float(d.get("duration", 0.0)),
            fps=int(d.get("fps", 60)),
            loop=bool(d.get("loop", False)),
            icon=str(d.get("icon", "")),
            group=str(d.get("group", "")),
            value_tracks=[ValueTrack.from_dict(t) for t in d.get("value_tracks", [])],
            trigger_tracks=[TriggerTrack.from_dict(t) for t in d.get("trigger_tracks", [])],
            created=str(d.get("created", "")),
            modified=str(d.get("modified", "")),
        )

    def stamp(self) -> None:
        """Update the modified timestamp; set created on first save."""
        now = time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())
        if not self.created:
            self.created = now
        self.modified = now


# ── poses ───────────────────────────────────────────────────────────


@dataclass
class PoseSetpoint:
    """A single value to push into the routing graph when a pose applies.

    ``target_kind`` selects the address space, mirroring ValueTrack's
    target model:

      * ``"ws_input"`` (default) — ``(sheet_id, ws_input_id)``, the same
        convention the controller gamepad bindings use. Applying the
        pose fans out ``routing_evaluator.set_ws_input`` calls.
        Backward-compatible with every pose authored before joint
        setpoints existed (those have no ``target_kind`` and deserialize
        to this).
      * ``"joint"`` — ``joint`` is a URDF joint name and the value goes
        to ``set_urdf_joint_value``, the same path animation value
        tracks use. This is what SRDF ``<group_state>`` import produces:
        a group_state is literally a named list of joint values, and
        there's no way to express it as WS inputs without a pre-existing
        routing binding for every joint.

    ``value`` is always **normalized −1..+1**, never radians. SRDF
    group_states are authored in URDF-native units and converted exactly
    once, at import, against each joint's ``<limit>`` — see
    ``srdf.GroupState.normalized_values``. Storing radians here would
    put an unconverted value one hop from a servo.
    """
    sheet_id: str = ""
    ws_input_id: str = ""
    value: float = 0.0
    target_kind: str = "ws_input"
    joint: str = ""

    def to_dict(self) -> Dict[str, Any]:
        return {
            "sheet_id": self.sheet_id,
            "ws_input_id": self.ws_input_id,
            "value": self.value,
            "target_kind": self.target_kind,
            "joint": self.joint,
        }

    @classmethod
    def from_dict(cls, d: Dict[str, Any]) -> "PoseSetpoint":
        # sheet_id / ws_input_id are optional now: a joint setpoint has
        # neither. Poses saved before joint setpoints existed always
        # carry both and default to target_kind="ws_input".
        return cls(
            sheet_id=str(d.get("sheet_id") or ""),
            ws_input_id=str(d.get("ws_input_id") or ""),
            value=float(d.get("value", 0.0)),
            target_kind=str(d.get("target_kind") or "ws_input"),
            joint=str(d.get("joint") or ""),
        )

    @property
    def is_joint(self) -> bool:
        return self.target_kind == "joint"

    def address(self) -> str:
        """Human-readable target, for skip lists and log lines."""
        return self.joint if self.is_joint else f"{self.sheet_id}/{self.ws_input_id}"


@dataclass
class Pose:
    id: str
    name: str
    icon: str = ""        # material-icon name; sidebar/management UI uses it
    group: str = ""       # single-level group ("" → Ungrouped bucket)
    setpoints: List[PoseSetpoint] = field(default_factory=list)
    description: str = ""
    # Provenance. ``"srdf"`` marks a pose imported from an SRDF
    # ``<group_state>``, with ``source_ref`` holding the group_state name
    # it came from. Lets the UI badge imported poses and lets a re-import
    # recognize what it would be overwriting — an operator who edited an
    # imported pose in the UI should not silently lose that work when
    # they upload a revised SRDF.
    source: str = ""
    source_ref: str = ""
    created: str = ""
    modified: str = ""

    def to_dict(self) -> Dict[str, Any]:
        return {
            "id": self.id,
            "name": self.name,
            "icon": self.icon,
            "group": self.group,
            "description": self.description,
            "source": self.source,
            "source_ref": self.source_ref,
            "setpoints": [s.to_dict() for s in self.setpoints],
            "created": self.created,
            "modified": self.modified,
        }

    @classmethod
    def from_dict(cls, d: Dict[str, Any]) -> "Pose":
        return cls(
            id=str(d["id"]),
            name=str(d.get("name", "")),
            icon=str(d.get("icon", "")),
            group=str(d.get("group", "")),
            description=str(d.get("description", "")),
            source=str(d.get("source", "")),
            source_ref=str(d.get("source_ref", "")),
            setpoints=[PoseSetpoint.from_dict(s) for s in d.get("setpoints", [])],
            created=str(d.get("created", "")),
            modified=str(d.get("modified", "")),
        )

    def joint_values(self) -> Dict[str, float]:
        """Joint-addressed setpoints as ``{joint: normalized}``.

        The shape the rig evaluator wants for a pose target, and what an
        animation pose track blends. WS-input setpoints are excluded —
        they have no joint to name.
        """
        return {s.joint: s.value for s in self.setpoints
                if s.is_joint and s.joint}

    def stamp(self) -> None:
        now = time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())
        if not self.created:
            self.created = now
        self.modified = now


#: Ceiling for a soundboard entry's ``volume``. 1.0 is the clip's own
#: level (0 dB); above that VLC applies software gain, which is how a
#: quiet clip is brought up to match the rest of a board.
#:
#: 2.0 rather than "unbounded" because gain cannot invent headroom: a
#: clip already near full scale just clips harder the further past 1.0 it
#: goes, so a bigger number would only buy distortion. It matches what
#: VLC's own UI offers by default.
#:
#: MIRRORED in firmware/raspberrypi/saint_node/soundboard.py — the node
#: is a separate deployable and cannot import this. Change both.
SOUND_VOLUME_MAX = 2.0


@dataclass
class Sound:
    """A soundboard entry: an audio file that a specific node plays.

    Node-scoped — ``file_path`` is an absolute path on ``node_id``'s own
    storage (files are never shipped between nodes) and playback comes out
    of that node's ``output_device`` (blank → the node's default ALSA
    device). ``position`` gives the operator an explicit ordering within a
    group (poses/animations are alpha-sorted; sounds are drag-ordered).
    """
    id: str
    name: str
    icon: str = ""            # material-icon name; default "volume_up" in UI
    group: str = ""           # single-level group ("" → Ungrouped bucket)
    node_id: str = ""         # which node plays this sound
    file_path: str = ""       # absolute path on that node
    output_device: str = ""   # ALSA device id ("" → node default)
    # 0.0 … SOUND_VOLUME_MAX. 1.0 is the file's own level; above it the
    # player applies software gain so a quiet clip can sit level with a
    # loud one on the same board.
    volume: float = 1.0
    start_time: float = 0.0   # seek offset in seconds at play
    loop: bool = False
    loop_count: int = 0       # repeats when loop on; 0 → infinite
    position: int = 0         # explicit order within the list/group
    created: str = ""
    modified: str = ""

    def to_dict(self) -> Dict[str, Any]:
        return {
            "id": self.id,
            "name": self.name,
            "icon": self.icon,
            "group": self.group,
            "node_id": self.node_id,
            "file_path": self.file_path,
            "output_device": self.output_device,
            "volume": self.volume,
            "start_time": self.start_time,
            "loop": self.loop,
            "loop_count": self.loop_count,
            "position": self.position,
            "created": self.created,
            "modified": self.modified,
        }

    @classmethod
    def from_dict(cls, d: Dict[str, Any]) -> "Sound":
        return cls(
            id=str(d["id"]),
            name=str(d.get("name", "")),
            icon=str(d.get("icon", "")),
            group=str(d.get("group", "")),
            node_id=str(d.get("node_id", "")),
            file_path=str(d.get("file_path", "")),
            output_device=str(d.get("output_device", "")),
            # Clamped on the way in: this is the last point before the
            # value is persisted and handed to a media player, and a
            # hand-edited file or an old client should not be able to
            # push an arbitrary gain into it.
            volume=max(0.0, min(SOUND_VOLUME_MAX,
                                float(d.get("volume", 1.0)))),
            start_time=float(d.get("start_time", 0.0)),
            loop=bool(d.get("loop", False)),
            loop_count=int(d.get("loop_count", 0)),
            position=int(d.get("position", 0)),
            created=str(d.get("created", "")),
            modified=str(d.get("modified", "")),
        )

    def stamp(self) -> None:
        now = time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())
        if not self.created:
            self.created = now
        self.modified = now


# ── playlists ───────────────────────────────────────────────────────


#: The three board kinds a playlist can hold. A playlist is single-kind:
#: an animation playlist holds animation ids and nothing else. Keeps the
#: sidebar's Animations / Poses / Sounds sections intact and lets each
#: kind's row renderer stay as it is.
PLAYLIST_KINDS = ("animations", "poses", "sounds")


@dataclass
class Playlist:
    """An ordered, named set of board items of one kind.

    Replaces the single ``group`` string that used to live on Animation,
    Pose and Sound. Membership is many-to-many — the same animation can
    sit in "Greetings" and in "Show opener" — so it cannot live on the
    item as one value any more.

    Membership AND order both live here, on the playlist, rather than
    being split between an ``item.playlists`` list and an ``item.position``
    int. That is what makes an item's slot per-playlist: "Bow" can be
    third in Greetings and first in Idle loops at the same time, which a
    single position field on the item cannot express. It also means
    reordering is one write to one file instead of N writes across the
    members, and an item's membership is removed by deleting it from this
    list — there is no second place for it to linger.

    ``items`` is the ordered list of member ids. It is allowed to name an
    id that no longer exists (an item deleted out from under the
    playlist); readers filter those out rather than the store rewriting
    every playlist on every delete. :meth:`prune` exists for callers that
    do want to compact it.
    """
    id: str
    name: str
    kind: str                                          # PLAYLIST_KINDS
    icon: str = ""
    items: List[str] = field(default_factory=list)     # ordered member ids
    position: int = 0                                  # order in the sidebar
    created: str = ""
    modified: str = ""

    def to_dict(self) -> Dict[str, Any]:
        return {
            "id": self.id,
            "name": self.name,
            "kind": self.kind,
            "icon": self.icon,
            "items": list(self.items),
            "position": self.position,
            "created": self.created,
            "modified": self.modified,
        }

    @classmethod
    def from_dict(cls, d: Dict[str, Any]) -> "Playlist":
        kind = str(d.get("kind", ""))
        if kind not in PLAYLIST_KINDS:
            raise ValueError(f"Unknown playlist kind: {kind!r}")
        # De-dupe on read as well as on write: a playlist that acquired a
        # duplicate id through some other path must not render the same
        # row twice, and "already a member" checks elsewhere assume this.
        seen = set()
        items: List[str] = []
        for raw in d.get("items", []) or []:
            iid = str(raw)
            if iid and iid not in seen:
                seen.add(iid)
                items.append(iid)
        return cls(
            id=str(d["id"]),
            name=str(d.get("name", "")),
            kind=kind,
            icon=str(d.get("icon", "")),
            items=items,
            position=int(d.get("position", 0)),
            created=str(d.get("created", "")),
            modified=str(d.get("modified", "")),
        )

    def add(self, item_id: str, index: Optional[int] = None) -> bool:
        """Insert ``item_id`` at ``index`` (default: append).

        Returns True if the playlist changed. An item already present is
        MOVED to the new slot rather than duplicated — dragging a row
        that is already in this list is a reorder, which is what the
        operator means by it.
        """
        if not item_id:
            return False
        before = list(self.items)
        if item_id in self.items:
            self.items.remove(item_id)
        if index is None or index < 0 or index >= len(self.items):
            self.items.append(item_id)
        else:
            self.items.insert(index, item_id)
        return self.items != before

    def remove(self, item_id: str) -> bool:
        """Drop ``item_id``. Returns True if it was there."""
        if item_id in self.items:
            self.items.remove(item_id)
            return True
        return False

    def reorder(self, ordered_ids: List[str]) -> None:
        """Set the order from ``ordered_ids``.

        Ids not currently in the playlist are ignored, and members the
        caller left out keep their relative order at the end — so a
        partial list (the visible rows, say, with a filter applied)
        reorders what it names without silently dropping the rest.
        """
        wanted = [i for i in dict.fromkeys(ordered_ids) if i in self.items]
        rest = [i for i in self.items if i not in wanted]
        self.items = wanted + rest

    def prune(self, known_ids) -> bool:
        """Drop members that no longer exist. True if anything went."""
        known = set(known_ids)
        before = list(self.items)
        self.items = [i for i in self.items if i in known]
        return self.items != before

    def stamp(self) -> None:
        now = time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())
        if not self.created:
            self.created = now
        self.modified = now
