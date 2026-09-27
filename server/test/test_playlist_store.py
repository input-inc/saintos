"""Tests for playlists — the many-to-many replacement for ``group``.

Board items (animations, poses, sounds) used to carry a single ``group``
string, so an item lived in exactly one bucket. Playlists lift that into
an ordered, named, many-to-many set: an animation can be in "Greetings"
and "Show opener" at once, at a different slot in each.

Covered here:

  1. Model: add/remove/reorder semantics, including the two that the
     drag-and-drop UI leans on — re-adding a member MOVES it rather than
     duplicating, and a partial reorder keeps the members it didn't name.
  2. Store CRUD: kind-namespaced slugs, auto-position at the end of the
     section, list() filtering and ordering, delete-playlist-keeps-items.
  3. Reverse index: memberships() answers "which playlists is this row
     in" for a whole kind at once.
  4. forget_item: deleting an item clears it out of every playlist, so a
     later item that slugs to the same id can't inherit memberships.
  5. Migration: legacy group strings become playlists exactly once, with
     the order the old UI displayed, and never re-runs.
"""
from __future__ import annotations

import os
import sys

import pytest

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from saint_server.animation.models import Playlist
from saint_server.animation.store import PlaylistStore


# ── model ───────────────────────────────────────────────────────────


def test_playlist_roundtrip_preserves_fields():
    pl = Playlist(id="anim_greetings", name="Greetings", kind="animations",
                  icon="waving_hand", items=["wave", "salute"], position=3)
    assert Playlist.from_dict(pl.to_dict()) == pl


def test_playlist_rejects_unknown_kind():
    with pytest.raises(ValueError):
        Playlist.from_dict({"id": "x", "name": "X", "kind": "widgets"})


def test_from_dict_dedupes_members():
    pl = Playlist.from_dict({"id": "x", "name": "X", "kind": "poses",
                             "items": ["a", "b", "a", "b", "c"]})
    assert pl.items == ["a", "b", "c"]


def test_add_appends_and_reports_change():
    pl = Playlist(id="x", name="X", kind="animations")
    assert pl.add("a") is True
    assert pl.add("b") is True
    assert pl.items == ["a", "b"]


def test_add_at_index_inserts():
    pl = Playlist(id="x", name="X", kind="animations", items=["a", "b", "c"])
    pl.add("d", 1)
    assert pl.items == ["a", "d", "b", "c"]


def test_re_adding_a_member_moves_it_rather_than_duplicating():
    # Dragging a row that is already in this playlist is a reorder --
    # the operator is placing it, not adding a second copy.
    pl = Playlist(id="x", name="X", kind="animations", items=["a", "b", "c"])
    assert pl.add("c", 0) is True
    assert pl.items == ["c", "a", "b"]


def test_re_adding_at_the_same_slot_is_not_a_change():
    pl = Playlist(id="x", name="X", kind="animations", items=["a", "b"])
    assert pl.add("b") is False


def test_remove_reports_whether_it_was_there():
    pl = Playlist(id="x", name="X", kind="animations", items=["a"])
    assert pl.remove("a") is True
    assert pl.remove("a") is False
    assert pl.items == []


def test_reorder_keeps_members_the_caller_left_out():
    # The UI may reorder a filtered view; unnamed members must survive.
    pl = Playlist(id="x", name="X", kind="sounds", items=["a", "b", "c", "d"])
    pl.reorder(["c", "a"])
    assert pl.items == ["c", "a", "b", "d"]


def test_reorder_ignores_ids_that_are_not_members():
    pl = Playlist(id="x", name="X", kind="sounds", items=["a", "b"])
    pl.reorder(["zzz", "b", "a"])
    assert pl.items == ["b", "a"]


def test_prune_drops_unknown_members():
    pl = Playlist(id="x", name="X", kind="poses", items=["a", "gone", "b"])
    assert pl.prune({"a", "b"}) is True
    assert pl.items == ["a", "b"]


# ── store ───────────────────────────────────────────────────────────


@pytest.fixture
def store(tmp_path):
    return PlaylistStore(str(tmp_path))


def test_save_assigns_kind_namespaced_slug(store):
    pl = store.save(Playlist(id="", name="Greetings", kind="animations"))
    assert pl.id == "anim_greetings"


def test_same_name_across_kinds_does_not_collide(store):
    a = store.save(Playlist(id="", name="Greetings", kind="animations"))
    s = store.save(Playlist(id="", name="Greetings", kind="sounds"))
    assert a.id != s.id
    assert {p["id"] for p in store.list()} == {a.id, s.id}


def test_same_name_within_a_kind_gets_a_suffix(store):
    a = store.save(Playlist(id="", name="Greetings", kind="animations"))
    b = store.save(Playlist(id="", name="Greetings", kind="animations"))
    assert a.id != b.id


def test_new_playlists_land_at_the_end_of_their_section(store):
    a = store.save(Playlist(id="", name="Alpha", kind="animations"))
    b = store.save(Playlist(id="", name="Beta", kind="animations"))
    assert (a.position, b.position) == (1, 2)


def test_list_filters_by_kind_and_sorts_by_position(store):
    store.save(Playlist(id="", name="Zulu", kind="animations", position=1))
    store.save(Playlist(id="", name="Alpha", kind="animations", position=2))
    store.save(Playlist(id="", name="Sounds one", kind="sounds"))
    names = [p["name"] for p in store.list("animations")]
    assert names == ["Zulu", "Alpha"]
    assert [p["name"] for p in store.list("sounds")] == ["Sounds one"]


def test_list_carries_items_and_count(store):
    pl = store.save(Playlist(id="", name="G", kind="poses", items=["a", "b"]))
    row = store.list("poses")[0]
    assert row["items"] == ["a", "b"]
    assert row["count"] == 2


def test_add_remove_and_reorder_items_persist(store):
    pl = store.save(Playlist(id="", name="G", kind="animations"))
    store.add_item(pl.id, "wave")
    store.add_item(pl.id, "salute")
    store.add_item(pl.id, "bow", 0)
    assert store.get(pl.id).items == ["bow", "wave", "salute"]

    store.reorder_items(pl.id, ["salute", "wave", "bow"])
    assert store.get(pl.id).items == ["salute", "wave", "bow"]

    store.remove_item(pl.id, "wave")
    assert store.get(pl.id).items == ["salute", "bow"]


def test_an_item_can_be_in_several_playlists_at_different_slots(store):
    # The whole point of the change.
    g = store.save(Playlist(id="", name="Greetings", kind="animations",
                            items=["wave", "salute", "bow"]))
    i = store.save(Playlist(id="", name="Idle", kind="animations",
                            items=["bow", "breathe"]))
    assert store.get(g.id).items.index("bow") == 2
    assert store.get(i.id).items.index("bow") == 0


def test_memberships_indexes_one_kind(store):
    g = store.save(Playlist(id="", name="Greetings", kind="animations",
                            items=["wave", "bow"]))
    i = store.save(Playlist(id="", name="Idle", kind="animations",
                            items=["bow"]))
    store.save(Playlist(id="", name="Noise", kind="sounds", items=["bow"]))
    idx = store.memberships("animations")
    assert idx["bow"] == [g.id, i.id]
    assert idx["wave"] == [g.id]


def test_deleting_a_playlist_leaves_its_members_alone(store):
    pl = store.save(Playlist(id="", name="G", kind="animations", items=["wave"]))
    other = store.save(Playlist(id="", name="H", kind="animations", items=["wave"]))
    assert store.delete(pl.id) is True
    assert store.get(pl.id) is None
    assert store.get(other.id).items == ["wave"]


def test_forget_item_clears_it_everywhere_in_its_kind(store):
    a = store.save(Playlist(id="", name="G", kind="animations", items=["wave", "bow"]))
    b = store.save(Playlist(id="", name="H", kind="animations", items=["bow"]))
    c = store.save(Playlist(id="", name="S", kind="sounds", items=["bow"]))
    assert store.forget_item("animations", "bow") == 2
    assert store.get(a.id).items == ["wave"]
    assert store.get(b.id).items == []
    # A sound that happens to share the id is a different namespace.
    assert store.get(c.id).items == ["bow"]


def test_reorder_rewrites_sidebar_positions(store):
    a = store.save(Playlist(id="", name="A", kind="animations"))
    b = store.save(Playlist(id="", name="B", kind="animations"))
    store.reorder("animations", [b.id, a.id])
    assert [p["name"] for p in store.list("animations")] == ["B", "A"]


# ── legacy migration ────────────────────────────────────────────────


LEGACY = {
    "animations": [
        {"id": "wave", "name": "Wave hello", "group": "Greetings"},
        {"id": "salute", "name": "Salute", "group": "Greetings"},
        {"id": "idle", "name": "Idle", "group": ""},
    ],
    "poses": [
        {"id": "stand", "name": "Stand", "group": "Base"},
    ],
    "sounds": [
        {"id": "s_b", "name": "B sound", "group": "Intros", "position": 1},
        {"id": "s_a", "name": "A sound", "group": "Intros", "position": 2},
    ],
}


def test_migration_creates_one_playlist_per_legacy_group(store):
    assert store.migrate_legacy_groups(LEGACY) == 3
    assert [p["name"] for p in store.list("animations")] == ["Greetings"]
    assert [p["name"] for p in store.list("poses")] == ["Base"]
    assert [p["name"] for p in store.list("sounds")] == ["Intros"]


def test_migration_keeps_the_order_the_old_ui_showed(store):
    store.migrate_legacy_groups(LEGACY)
    # Animations were alpha-sorted...
    anim = store.list("animations")[0]
    assert anim["items"] == ["salute", "wave"]
    # ...sounds by their explicit position, not by name.
    snd = store.list("sounds")[0]
    assert snd["items"] == ["s_b", "s_a"]


def test_migration_leaves_ungrouped_items_ungrouped(store):
    store.migrate_legacy_groups(LEGACY)
    assert "idle" not in store.memberships("animations")


def test_migration_runs_only_once(store):
    assert store.migrate_legacy_groups(LEGACY) == 3
    assert store.migrate_legacy_groups(LEGACY) == 0
    assert len(store.list()) == 3


def test_migration_does_not_resurrect_deleted_playlists(store):
    store.migrate_legacy_groups(LEGACY)
    for row in store.list():
        store.delete(row["id"])
    assert store.migrate_legacy_groups(LEGACY) == 0
    assert store.list() == []


def test_migration_on_an_ungrouped_library_still_marks_itself_done(store):
    only_ungrouped = {"animations": [{"id": "a", "name": "A", "group": ""}]}
    assert store.migrate_legacy_groups(only_ungrouped) == 0
    # Second call must be a no-op even though nothing was created --
    # otherwise grouping one item later would trigger a late migration.
    assert store.migrate_legacy_groups(LEGACY) == 0
