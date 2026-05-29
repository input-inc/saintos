"""Parse Pololu Maestro `.xml` save files into Animation skeletons.

Pololu's Maestro Control Center exports settings as XML. Within
``<Settings>`` there's a ``<Sequences>`` block; each ``<Sequence>``
holds an ordered list of ``<Frame>`` children with a duration (in
ms) and ``<Positions>`` per channel (in quarter-microseconds — the
Maestro's native pulse-width units).

We convert each Maestro channel into a separate ValueTrack and each
Frame into a keyframe per channel. Operators map Maestro channel
indices to human-readable track names in the UI before saving.

Returned animations are NOT yet persisted — the caller posts them
through the regular save_animation path after the operator has
named tracks and chosen scaling.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, List, Optional
from xml.etree import ElementTree as ET

from saint_server.unreal.animation import (
    AnimationCurve,
    CurveInterpolation,
    CurveKey,
)
from saint_server.animation.models import Animation, ValueTrack


# Pololu Maestro stores pulse width in quarter-microseconds: a 1500 µs
# neutral position is stored as 6000. Default mapping converts those
# units into pulse-microseconds; the operator can override the
# normalization later.
QUS_PER_US = 4.0


@dataclass
class MaestroChannel:
    """Summary of a discovered Maestro channel, ready for operator mapping."""
    index: int
    min_value: float
    max_value: float
    keyframe_count: int


@dataclass
class MaestroImportResult:
    """A parsed Maestro animation plus per-channel metadata for the mapping UI."""
    animation: Animation
    channels: List[MaestroChannel]
    sequence_name: str
    raw_frame_count: int


def parse_maestro_xml(xml_bytes: bytes,
                      sequence_name: Optional[str] = None
                      ) -> MaestroImportResult:
    """Parse a Maestro save XML and convert one ``<Sequence>`` to an Animation.

    If ``sequence_name`` is None, the first sequence in the file is
    used. ``sequence_name`` is matched case-insensitively against the
    Sequence ``name`` attribute.

    Raises ValueError if the file isn't a Maestro export, contains
    no sequences, or the requested sequence doesn't exist.
    """
    try:
        root = ET.fromstring(xml_bytes)
    except ET.ParseError as e:
        raise ValueError(f"Not valid XML: {e}") from e

    # The save file can root at <UscSettings> or <Settings> depending on
    # which Maestro version emitted it; accept either.
    sequences_parent = root.find(".//Sequences")
    if sequences_parent is None:
        raise ValueError("XML contains no <Sequences> block")
    sequence_nodes = sequences_parent.findall("Sequence")
    if not sequence_nodes:
        raise ValueError("<Sequences> block is empty")

    chosen = None
    if sequence_name:
        target = sequence_name.casefold()
        for s in sequence_nodes:
            if (s.attrib.get("name", "")).casefold() == target:
                chosen = s
                break
        if chosen is None:
            raise ValueError(f"No sequence named {sequence_name!r}")
    else:
        chosen = sequence_nodes[0]

    name = chosen.attrib.get("name", "imported")

    # Walk frames, accumulating per-channel keyframes.
    # Each <Frame> has a Duration (ms) and Positions text — a space-
    # separated list of channel positions in quarter-microseconds.
    per_channel: Dict[int, List[CurveKey]] = {}
    channel_extremes: Dict[int, List[float]] = {}   # [min, max]
    t = 0.0
    frames = chosen.findall("Frame")
    for frame in frames:
        duration_ms = float(frame.attrib.get("Duration", "0"))
        positions_text = (frame.findtext("Positions") or "").strip()
        # Backward-compat: some Maestro versions put Positions in an
        # attribute instead of a child element.
        if not positions_text:
            positions_text = frame.attrib.get("Positions", "").strip()
        if not positions_text:
            t += duration_ms / 1000.0
            continue
        parts = positions_text.split()
        for idx, raw in enumerate(parts):
            try:
                qus = float(raw)
            except ValueError:
                continue
            pulse_us = qus / QUS_PER_US
            per_channel.setdefault(idx, []).append(
                CurveKey(time=t, value=pulse_us,
                         interp=CurveInterpolation.LINEAR)
            )
            ext = channel_extremes.setdefault(idx, [pulse_us, pulse_us])
            if pulse_us < ext[0]: ext[0] = pulse_us
            if pulse_us > ext[1]: ext[1] = pulse_us
        t += duration_ms / 1000.0

    duration = t
    value_tracks: List[ValueTrack] = []
    channels: List[MaestroChannel] = []
    for idx in sorted(per_channel.keys()):
        track_id = f"ch{idx}"
        curve = AnimationCurve(name=f"channel_{idx}", keys=per_channel[idx])
        value_tracks.append(
            ValueTrack(id=track_id, name=f"Channel {idx}", curve=curve)
        )
        ext = channel_extremes[idx]
        channels.append(MaestroChannel(
            index=idx,
            min_value=ext[0],
            max_value=ext[1],
            keyframe_count=len(per_channel[idx]),
        ))

    # Slugify name → animation id.
    from saint_server.animation.store import slugify
    anim = Animation(
        id=slugify(name) or "imported",
        name=name,
        duration=duration,
        fps=60,
        value_tracks=value_tracks,
    )
    return MaestroImportResult(
        animation=anim,
        channels=channels,
        sequence_name=name,
        raw_frame_count=len(frames),
    )


def list_sequences(xml_bytes: bytes) -> List[str]:
    """Return the sequence names in a Maestro save file (for the picker)."""
    try:
        root = ET.fromstring(xml_bytes)
    except ET.ParseError as e:
        raise ValueError(f"Not valid XML: {e}") from e
    out = []
    for s in root.findall(".//Sequences/Sequence"):
        out.append(s.attrib.get("name", ""))
    return out
