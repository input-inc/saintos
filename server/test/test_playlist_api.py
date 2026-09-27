"""StateManager-level tests for the playlist API.

test_playlist_store.py covers the model and the store on their own.
This file covers the parts that only exist once the stores are wired
together in the StateManager:

  1. Item summaries carry the playlists they belong to.
  2. Legacy `group` compatibility for the Steam Deck, which still reads
     one group name per sound.
  3. add_item validates the item against the playlist's kind.
  4. Deleting an item clears it out of every playlist.
  5. Migration runs on construction, so the very first list call already
     reflects playlists rather than a half-converted library.
"""
from __future__ import annotations

import json
import os
import sys

import pytest

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from saint_server.animation.models import Animation, Playlist, Pose, Sound
from saint_server.webserver.state_manager import StateManager


@pytest.fixture
def sm(tmp_path):
    return StateManager(config_dir=str(tmp_path))


def _seed_animation(sm, aid, name):
    sm.animation_store.save(Animation(id=aid, name=name))


def _seed_sound(sm, sid, name):
    sm.sound_store.save(Sound(id=sid, name=name, node_id="n1",
                              file_path=f"/audio/{sid}.wav"))


# ── membership annotation ───────────────────────────────────────────


def test_list_animations_carries_playlist_membership(sm):
    _seed_animation(sm, "wave", "Wave")
    _seed_animation(sm, "bow", "Bow")
    g = sm.playlist_store.save(Playlist(id="", name="Greetings",
                                        kind="animations", items=["wave"]))
    i = sm.playlist_store.save(Playlist(id="", name="Idle",
                                        kind="animations", items=["wave", "bow"]))
    rows = {r["id"]: r for r in sm.list_animations()}
    assert rows["wave"]["playlists"] == [g.id, i.id]
    assert rows["bow"]["playlists"] == [i.id]


def test_an_item_in_no_playlist_reports_an_empty_list(sm):
    _seed_animation(sm, "lonely", "Lonely")
    assert sm.list_animations()[0]["playlists"] == []


def test_poses_and_sounds_are_annotated_too(sm):
    sm.pose_store.save(Pose(id="stand", name="Stand"))
    _seed_sound(sm, "beep", "Beep")
    p = sm.playlist_store.save(Playlist(id="", name="Base", kind="poses",
                                        items=["stand"]))
    s = sm.playlist_store.save(Playlist(id="", name="Alerts", kind="sounds",
                                        items=["beep"]))
    assert sm.list_poses()[0]["playlists"] == [p.id]
    assert sm.list_sounds()[0]["playlists"] == [s.id]


# ── Steam Deck legacy `group` shim ──────────────────────────────────


def test_sounds_still_report_a_single_group_name_for_the_deck(sm):
    _seed_sound(sm, "beep", "Beep")
    sm.playlist_store.save(Playlist(id="", name="Alerts", kind="sounds",
                                    items=["beep"]))
    assert sm.list_sounds()[0]["group"] == "Alerts"


def test_deck_group_is_the_first_playlist_when_there_are_several(sm):
    _seed_sound(sm, "beep", "Beep")
    a = sm.playlist_store.save(Playlist(id="", name="Alerts", kind="sounds",
                                        items=["beep"], position=1))
    sm.playlist_store.save(Playlist(id="", name="Zed", kind="sounds",
                                    items=["beep"], position=2))
    assert sm.list_sounds()[0]["group"] == "Alerts"


def test_deck_group_is_blank_for_an_unplaylisted_sound(sm):
    _seed_sound(sm, "beep", "Beep")
    assert sm.list_sounds()[0]["group"] == ""


# ── add / remove / reorder through the manager ──────────────────────


def test_add_item_appends_and_is_idempotent_on_slot(sm):
    _seed_animation(sm, "wave", "Wave")
    pl = sm.playlist_store.save(Playlist(id="", name="G", kind="animations"))
    assert sm.playlist_add_item(pl.id, "wave")["success"] is True
    res = sm.playlist_add_item(pl.id, "wave")
    assert res["playlist"]["items"] == ["wave"]


def test_add_item_rejects_an_item_of_the_wrong_kind(sm):
    _seed_sound(sm, "beep", "Beep")
    pl = sm.playlist_store.save(Playlist(id="", name="G", kind="animations"))
    res = sm.playlist_add_item(pl.id, "beep")
    assert res["success"] is False
    assert "no animation with id" in res["message"].lower()


def test_add_item_rejects_an_unknown_playlist(sm):
    _seed_animation(sm, "wave", "Wave")
    assert sm.playlist_add_item("nope", "wave")["success"] is False


def test_add_item_at_index_places_the_row(sm):
    for aid in ("a", "b", "c"):
        _seed_animation(sm, aid, aid.upper())
    pl = sm.playlist_store.save(Playlist(id="", name="G", kind="animations",
                                         items=["a", "b"]))
    res = sm.playlist_add_item(pl.id, "c", 0)
    assert res["playlist"]["items"] == ["c", "a", "b"]


def test_remove_item_and_reorder(sm):
    for aid in ("a", "b", "c"):
        _seed_animation(sm, aid, aid.upper())
    pl = sm.playlist_store.save(Playlist(id="", name="G", kind="animations",
                                         items=["a", "b", "c"]))
    assert sm.playlist_remove_item(pl.id, "b")["playlist"]["items"] == ["a", "c"]
    res = sm.reorder_playlist_items(pl.id, ["c", "a"])
    assert res["playlist"]["items"] == ["c", "a"]


def test_reorder_playlists_rejects_an_unknown_kind(sm):
    assert sm.reorder_playlists("widgets", [])["success"] is False


# ── deletion cleanup ────────────────────────────────────────────────


def test_deleting_an_animation_clears_it_from_playlists(sm):
    _seed_animation(sm, "wave", "Wave")
    pl = sm.playlist_store.save(Playlist(id="", name="G", kind="animations",
                                         items=["wave"]))
    assert sm.delete_animation("wave")["success"] is True
    assert sm.playlist_store.get(pl.id).items == []


def test_deleting_a_sound_clears_it_from_playlists(sm):
    _seed_sound(sm, "beep", "Beep")
    pl = sm.playlist_store.save(Playlist(id="", name="G", kind="sounds",
                                         items=["beep"]))
    assert sm.delete_sound("beep")["success"] is True
    assert sm.playlist_store.get(pl.id).items == []


def test_deleting_a_playlist_keeps_its_items(sm):
    _seed_animation(sm, "wave", "Wave")
    pl = sm.playlist_store.save(Playlist(id="", name="G", kind="animations",
                                         items=["wave"]))
    assert sm.delete_playlist(pl.id)["success"] is True
    assert [r["id"] for r in sm.list_animations()] == ["wave"]


# ── migration on construction ───────────────────────────────────────


def test_a_legacy_library_is_migrated_before_the_first_list_call(tmp_path):
    # Write animation files carrying the old single `group` string, the
    # way a pre-playlist install has them on disk.
    anim_dir = tmp_path / "animations"
    anim_dir.mkdir()
    for aid, group in (("wave", "Greetings"), ("bow", "Greetings"),
                       ("idle", "")):
        (anim_dir / f"{aid}.json").write_text(json.dumps({
            "id": aid, "name": aid.title(), "group": group,
        }))

    sm = StateManager(config_dir=str(tmp_path))
    playlists = sm.list_playlists("animations")
    assert [p["name"] for p in playlists] == ["Greetings"]
    assert sorted(playlists[0]["items"]) == ["bow", "wave"]

    rows = {r["id"]: r for r in sm.list_animations()}
    assert rows["wave"]["playlists"] == [playlists[0]["id"]]
    assert rows["idle"]["playlists"] == []


def test_migration_does_not_re_run_for_a_second_manager(tmp_path):
    anim_dir = tmp_path / "animations"
    anim_dir.mkdir()
    (anim_dir / "wave.json").write_text(json.dumps({
        "id": "wave", "name": "Wave", "group": "Greetings"}))

    first = StateManager(config_dir=str(tmp_path))
    pl_id = first.list_playlists("animations")[0]["id"]
    first.playlist_remove_item(pl_id, "wave")

    # A restart must not re-add the member from the stale `group` field.
    second = StateManager(config_dir=str(tmp_path))
    assert second.list_playlists("animations")[0]["items"] == []
