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

from saint_server.animation.models import Sound
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
    a = store.save(Sound(id="", name="Hello World", group="G1"))
    b = store.save(Sound(id="", name="Second", group="G1"))
    assert a.id == "hello_world"
    # Auto-position: first in group → 1, second → 2.
    assert a.position == 1
    assert b.position == 2
    # New group starts its own position sequence.
    c = store.save(Sound(id="", name="Other", group="G2"))
    assert c.position == 1


def test_list_sorted_by_group_position_name(tmp_path):
    store = SoundStore(str(tmp_path))
    store.save(Sound(id="", name="Bravo", group="B"))
    store.save(Sound(id="", name="Alpha", group="A"))
    store.save(Sound(id="", name="Charlie", group="A"))
    names = [s["name"] for s in store.list()]
    # Group A (positions 1,2) before group B.
    assert names == ["Alpha", "Charlie", "Bravo"]


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
