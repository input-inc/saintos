"""Tests for the soundboard model + store.

Sounds are node-scoped audio entries the operator manages under Boards →
Sounds. Unlike poses/animations they carry an explicit ``position`` for
drag-ordering, so these tests cover:

  1. Model roundtrip: Sound.to_dict() → from_dict() preserves every
     field (node/file/device + play options + position).
  2. Store CRUD: save assigns a slug id, auto-positions new entries at
     the end of their group, list() returns sorted summaries, delete
     removes the file.
  3. Reorder: reorder(ordered_ids) rewrites positions so list() reflects
     the new order.
"""
from __future__ import annotations

import os
import sys

import pytest

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from saint_server.animation.models import SOUND_VOLUME_MAX, Sound
from saint_server.animation.store import SoundStore


# ── model ───────────────────────────────────────────────────────────


def test_sound_roundtrip_preserves_fields():
    s = Sound(
        id="fanfare",
        name="Fanfare",
        icon="celebration",
        group="Intros",
        node_id="raspberrypi_A1B2C3D4",
        file_path="/var/lib/saint-os/audio/fanfare.mp3",
        output_device="hw:1,0",
        volume=0.75,
        start_time=1.5,
        loop=True,
        loop_count=3,
        position=2,
    )
    back = Sound.from_dict(s.to_dict())
    assert back == s


def test_volume_above_one_is_kept(tmp_path):
    # A quiet clip can be boosted past its own level; that is the whole
    # point of allowing >100%.
    s = Sound.from_dict({"id": "quiet", "name": "Quiet", "volume": 1.6})
    assert s.volume == 1.6


def test_volume_is_clamped_to_the_ceiling():
    s = Sound.from_dict({"id": "loud", "name": "Loud", "volume": 12.0})
    assert s.volume == SOUND_VOLUME_MAX


def test_negative_volume_floors_at_mute():
    s = Sound.from_dict({"id": "neg", "name": "Neg", "volume": -1.0})
    assert s.volume == 0.0


def test_sound_from_dict_defaults():
    s = Sound.from_dict({"id": "beep", "name": "Beep"})
    assert s.volume == 1.0
    assert s.loop is False
    assert s.loop_count == 0
    assert s.output_device == ""
    assert s.position == 0


# ── store CRUD ──────────────────────────────────────────────────────


def test_save_assigns_slug_and_auto_position(tmp_path):
    store = SoundStore(str(tmp_path))
    a = store.save(Sound(id="", name="Hello World"))
    b = store.save(Sound(id="", name="Second"))
    assert a.id == "hello_world"
    # Auto-position runs over the whole library now: `position` orders
    # the flat "All Sounds" view. Order within a playlist belongs to the
    # playlist, which is why this is no longer per-group.
    assert a.position == 1
    assert b.position == 2
    c = store.save(Sound(id="", name="Other"))
    assert c.position == 3


def test_list_sorted_by_position_then_name(tmp_path):
    store = SoundStore(str(tmp_path))
    store.save(Sound(id="", name="Bravo"))
    store.save(Sound(id="", name="Alpha"))
    store.save(Sound(id="", name="Charlie"))
    names = [s["name"] for s in store.list()]
    # Insertion order, via auto-position -- NOT alphabetical, and no
    # longer bucketed by group.
    assert names == ["Bravo", "Alpha", "Charlie"]


def test_position_leads_the_sort_over_name(tmp_path):
    store = SoundStore(str(tmp_path))
    store.save(Sound(id="", name="Zulu", position=1))
    store.save(Sound(id="", name="Alpha", position=2))
    assert [s["name"] for s in store.list()] == ["Zulu", "Alpha"]


def test_get_and_delete(tmp_path):
    store = SoundStore(str(tmp_path))
    saved = store.save(Sound(id="", name="Zap", file_path="/tmp/zap.wav"))
    assert store.get(saved.id).file_path == "/tmp/zap.wav"
    assert store.delete(saved.id) is True
    assert store.get(saved.id) is None
    assert store.delete(saved.id) is False


# ── reorder ─────────────────────────────────────────────────────────


def test_reorder_rewrites_positions(tmp_path):
    store = SoundStore(str(tmp_path))
    one = store.save(Sound(id="", name="One", group="G"))
    two = store.save(Sound(id="", name="Two", group="G"))
    three = store.save(Sound(id="", name="Three", group="G"))
    # Reverse the order.
    store.reorder([three.id, two.id, one.id])
    order = [s["id"] for s in store.list()]
    assert order == [three.id, two.id, one.id]
    # Positions are 1..N in the new order.
    positions = {s["id"]: s["position"] for s in store.list()}
    assert positions[three.id] == 1
    assert positions[one.id] == 3


def test_reorder_skips_unknown_ids(tmp_path):
    store = SoundStore(str(tmp_path))
    a = store.save(Sound(id="", name="A", group="G"))
    # Unknown ids are ignored; known ones still get sequential positions.
    result = store.reorder(["ghost", a.id])
    assert any(s["id"] == a.id for s in result)
    assert store.get(a.id).position == 2


if __name__ == "__main__":
    sys.exit(pytest.main([__file__, "-v"]))


# ── duration backfill ───────────────────────────────────────────────
#
# save() only measures a clip when it is written, so a library saved
# before probing existed had no lengths at all and the animation
# timeline drew every sound as a start marker instead of a clip bar.


def _write_wav(path, seconds, rate=8000):
    import wave
    with wave.open(str(path), "wb") as w:
        w.setnchannels(1)
        w.setsampwidth(1)
        w.setframerate(rate)
        w.writeframes(b"\x80" * int(seconds * rate))


def _legacy_entry(store, sid, file_path, **extra):
    """An entry as written before duration probing: no `duration` key."""
    raw = {"id": sid, "name": sid, "node_id": "host_controller",
           "file_path": file_path, "modified": "2026-05-25T02:51:35Z"}
    raw.update(extra)
    store.store.write_raw(sid, raw)


def test_backfill_measures_entries_saved_without_a_duration(tmp_path):
    store = SoundStore(str(tmp_path))
    wav = tmp_path / "clip.wav"
    _write_wav(wav, 2.0)
    _legacy_entry(store, "clip", str(wav))

    assert store.backfill_durations() == 1
    entry = {s["id"]: s for s in store.list()}["clip"]
    assert entry["duration"] == pytest.approx(2.0, abs=0.01)


def test_backfill_does_not_count_as_an_operator_edit(tmp_path):
    store = SoundStore(str(tmp_path))
    wav = tmp_path / "clip.wav"
    _write_wav(wav, 1.0)
    _legacy_entry(store, "clip", str(wav))

    store.backfill_durations()
    assert store.store.read_raw("clip")["modified"] == "2026-05-25T02:51:35Z"


def test_backfill_leaves_measured_and_unmeasurable_entries_alone(tmp_path):
    store = SoundStore(str(tmp_path))
    wav = tmp_path / "clip.wav"
    _write_wav(wav, 3.0)
    # Already measured: kept even though the file now says otherwise.
    _legacy_entry(store, "known", str(wav), duration=9.5)
    # File isn't on this machine (e.g. it lives on another node).
    _legacy_entry(store, "elsewhere", "/nonexistent/clip.mp3")

    assert store.backfill_durations() == 0
    by_id = {s["id"]: s for s in store.list()}
    assert by_id["known"]["duration"] == 9.5
    assert by_id["elsewhere"]["duration"] == 0.0


def test_backfill_does_not_clobber_a_save_made_while_probing(tmp_path, monkeypatch):
    store = SoundStore(str(tmp_path))
    old, new = tmp_path / "old.wav", tmp_path / "new.wav"
    _write_wav(old, 1.0)
    _write_wav(new, 4.0)
    _legacy_entry(store, "clip", str(old))

    import saint_server.animation.store as store_mod
    real_probe = store_mod.probe_duration

    def probe_while_operator_repoints(path):
        # The operator re-points the sound mid-probe; their save measures
        # the new file itself.
        if path == str(old):
            snd = store.get("clip")
            snd.file_path = str(new)
            store.save(snd)
        return real_probe(path)

    monkeypatch.setattr(store_mod, "probe_duration", probe_while_operator_repoints)
    assert store.backfill_durations() == 0
    entry = store.store.read_raw("clip")
    assert entry["file_path"] == str(new)
    assert entry["duration"] == pytest.approx(4.0, abs=0.01)


# ── measure (called when a sound is added to a timeline) ────────────


def test_measure_fills_and_stores_a_missing_length(tmp_path):
    store = SoundStore(str(tmp_path))
    wav = tmp_path / "clip.wav"
    _write_wav(wav, 1.5)
    _legacy_entry(store, "clip", str(wav))

    assert store.measure("clip") == pytest.approx(1.5, abs=0.01)
    raw = store.store.read_raw("clip")
    assert raw["duration"] == pytest.approx(1.5, abs=0.01)
    assert raw["modified"] == "2026-05-25T02:51:35Z"   # a lookup, not an edit


def test_measure_returns_a_stored_length_without_probing(tmp_path, monkeypatch):
    store = SoundStore(str(tmp_path))
    _legacy_entry(store, "clip", "/nonexistent.mp3", duration=7.25)
    import saint_server.animation.store as store_mod
    monkeypatch.setattr(store_mod, "probe_duration",
                        lambda p: pytest.fail("should not probe"))
    assert store.measure("clip") == 7.25


def test_measure_unknown_sound_is_none(tmp_path):
    assert SoundStore(str(tmp_path)).measure("nope") is None


def test_keyframe_clip_length_survives_a_save():
    from saint_server.animation.models import TriggerKeyframe
    kf = TriggerKeyframe(time=2.0, target_kind="sound", target=["fanfare"],
                         value=None, clip_length=3.25)
    assert TriggerKeyframe.from_dict(kf.to_dict()).clip_length == 3.25
    # Older animations without the field load as "not recorded".
    legacy = {"time": 1.0, "target_kind": "sound", "target": ["x"]}
    assert TriggerKeyframe.from_dict(legacy).clip_length == 0.0
