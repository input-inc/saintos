"""
Tests for the in-process host soundboard module (server-side twin of the
Pi firmware soundboard). VLC is faked so this runs on any host.

Covers: directory listing (dirs-first + audio flagging), device
enumeration always exposing a default, and SoundboardPlayer applying the
per-entry options (device, volume, start-time, loop/loop_count) plus
stop-and-replace.
"""
import os
import sys
import types

import pytest

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))


class _FakeMedia:
    def __init__(self, path):
        self.path = path
        self.options = []

    def add_option(self, opt):
        self.options.append(opt)


class _FakePlayer:
    def __init__(self):
        self.calls = []
        self.media = None
        self.released = False

    def set_media(self, m):
        self.media = m

    def audio_set_volume(self, v):
        self.calls.append(("volume", v))

    def play(self):
        self.calls.append(("play",))
        return 0

    def stop(self):
        self.calls.append(("stop",))

    def release(self):
        self.released = True


class _FakeInstance:
    def __init__(self, args):
        self.args = list(args)

    def media_player_new(self):
        return _FakePlayer()

    def media_new(self, path):
        return _FakeMedia(path)


@pytest.fixture
def sb(monkeypatch):
    fake = types.SimpleNamespace(Instance=lambda *a, **k: _FakeInstance(a))
    monkeypatch.setitem(sys.modules, "vlc", fake)
    import importlib
    from saint_server.host_peripherals import soundboard as sb_mod
    importlib.reload(sb_mod)
    return sb_mod


# ── list_directory ──────────────────────────────────────────────────


def test_list_directory_lists_dirs_and_flags_audio(sb, tmp_path):
    (tmp_path / "sub").mkdir()
    (tmp_path / "clip.wav").write_bytes(b"\0\0")
    (tmp_path / "notes.txt").write_text("x")
    res = sb.list_directory(str(tmp_path))
    assert res["status"] == "ok"
    by_name = {e["name"]: e for e in res["entries"]}
    assert by_name["sub"]["is_dir"] is True
    assert by_name["clip.wav"]["is_audio"] is True
    assert by_name["notes.txt"]["is_audio"] is False
    assert res["entries"][0]["name"] == "sub"   # dirs first


def test_list_directory_bad_path(sb):
    assert sb.list_directory("/definitely/not/real/xyz")["status"] == "error"


def test_list_audio_devices_always_has_default(sb):
    res = sb.list_audio_devices()
    assert res["status"] == "ok"
    assert any(d["id"] == "default" and d["is_default"] for d in res["devices"])


# ── SoundboardPlayer ────────────────────────────────────────────────


def test_play_missing_file_errors(sb):
    ok, _ = sb.SoundboardPlayer().play("/no/such/file.wav")
    assert ok is False


def test_play_applies_options(sb, tmp_path):
    clip = tmp_path / "clip.wav"
    clip.write_bytes(b"\0\0")
    player = sb.SoundboardPlayer()
    ok, _ = player.play(str(clip), device="hw:1,0", volume=0.5,
                        start_time_s=2.0, loop=True, loop_count=3)
    assert ok is True
    inst = player._instances["hw:1,0"]
    assert any("--alsa-audio-device=hw:1,0" in a for a in inst.args)
    assert ("volume", 50) in player._player.calls
    opts = player._player.media.options
    assert any(o.startswith("start-time=2") for o in opts)
    assert "input-repeat=2" in opts     # loop_count 3 → 2 repeats


def test_play_infinite_loop_uses_sentinel(sb, tmp_path):
    clip = tmp_path / "loop.wav"
    clip.write_bytes(b"\0\0")
    player = sb.SoundboardPlayer()
    player.play(str(clip), loop=True, loop_count=0)
    assert f"input-repeat={sb._INFINITE_REPEAT}" in player._player.media.options


def test_play_stops_previous(sb, tmp_path):
    clip = tmp_path / "a.wav"
    clip.write_bytes(b"\0\0")
    player = sb.SoundboardPlayer()
    player.play(str(clip))
    first = player._player
    player.play(str(clip))
    assert ("stop",) in first.calls and first.released is True


if __name__ == "__main__":
    sys.exit(pytest.main([__file__, "-v"]))
