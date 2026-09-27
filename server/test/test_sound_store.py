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
