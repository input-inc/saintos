"""Audio duration probing for soundboard clips.

The animation timeline draws a sound board-item as a bar the length of
the clip, so a Sound needs a real duration. Nothing carried one before:
a Sound had volume, start offset and loop options but no length.

The probe is layered — stdlib for uncompressed, then ffprobe, then
python-vlc — because a server may have any combination of those. The
contract every layer shares is that failure returns 0.0, meaning
"unknown", which the timeline must render as a marker rather than a
zero-length clip.
"""
from __future__ import annotations

import os
import struct
import sys
import wave

import pytest

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from saint_server.animation.audio_probe import probe_duration
from saint_server.animation.models import Sound
from saint_server.animation.store import SoundStore


def _write_wav(path, seconds=1.0, rate=8000):
    """A real, readable WAV of a known length."""
    frames = int(rate * seconds)
    with wave.open(str(path), "wb") as w:
        w.setnchannels(1)
        w.setsampwidth(2)
        w.setframerate(rate)
        w.writeframes(struct.pack("<h", 0) * frames)
    return str(path)


# ── the probe ───────────────────────────────────────────────────────


def test_measures_a_wav_exactly(tmp_path):
    path = _write_wav(tmp_path / "clip.wav", seconds=1.5)
    assert probe_duration(path) == pytest.approx(1.5, abs=0.01)


def test_measures_a_short_wav(tmp_path):
    path = _write_wav(tmp_path / "blip.wav", seconds=0.25)
    assert probe_duration(path) == pytest.approx(0.25, abs=0.01)


def test_a_missing_file_is_unknown_not_an_error(tmp_path):
    assert probe_duration(str(tmp_path / "nope.wav")) == 0.0


def test_an_empty_path_is_unknown(tmp_path):
    assert probe_duration("") == 0.0


def test_a_directory_is_unknown(tmp_path):
    assert probe_duration(str(tmp_path)) == 0.0


def test_garbage_is_unknown_rather_than_raising(tmp_path):
    junk = tmp_path / "notaudio.wav"
    junk.write_bytes(b"this is not audio at all, not even slightly")
    # ffprobe/vlc may also be asked; all three must decline quietly.
    assert probe_duration(str(junk)) == 0.0


# ── the store wires it in ───────────────────────────────────────────


def test_saving_a_sound_measures_its_clip(tmp_path):
    clip = _write_wav(tmp_path / "fanfare.wav", seconds=2.0)
    store = SoundStore(str(tmp_path / "cfg"))
    saved = store.save(Sound(id="", name="Fanfare", file_path=clip))
    assert saved.duration == pytest.approx(2.0, abs=0.01)
    assert store.list()[0]["duration"] == pytest.approx(2.0, abs=0.01)


def test_a_sound_with_no_file_has_an_unknown_duration(tmp_path):
    store = SoundStore(str(tmp_path / "cfg"))
    saved = store.save(Sound(id="", name="Empty"))
    assert saved.duration == 0.0


def test_an_ordinary_edit_does_not_re_measure(tmp_path):
    # Renaming or changing volume must not re-read the file; the stored
    # duration is kept.
    clip = _write_wav(tmp_path / "a.wav", seconds=1.0)
    store = SoundStore(str(tmp_path / "cfg"))
    saved = store.save(Sound(id="", name="A", file_path=clip))
    os.remove(clip)                     # the probe would now fail
    saved.volume = 0.5
    again = store.save(saved)
    assert again.duration == pytest.approx(1.0, abs=0.01)


def test_pointing_a_sound_at_a_different_file_re_measures(tmp_path):
    short = _write_wav(tmp_path / "short.wav", seconds=0.5)
    long = _write_wav(tmp_path / "long.wav", seconds=3.0)
    store = SoundStore(str(tmp_path / "cfg"))
    saved = store.save(Sound(id="", name="Clip", file_path=short))
    assert saved.duration == pytest.approx(0.5, abs=0.01)
    saved.file_path = long
    again = store.save(saved)
    assert again.duration == pytest.approx(3.0, abs=0.01)


def test_reprobe_picks_up_a_replaced_file(tmp_path):
    # Same path, different audio — only an explicit reprobe catches it.
    clip = tmp_path / "swap.wav"
    _write_wav(clip, seconds=1.0)
    store = SoundStore(str(tmp_path / "cfg"))
    saved = store.save(Sound(id="", name="Swap", file_path=str(clip)))
    assert saved.duration == pytest.approx(1.0, abs=0.01)

    _write_wav(clip, seconds=4.0)
    assert store.save(saved).duration == pytest.approx(1.0, abs=0.01)  # unchanged
    assert store.reprobe(saved.id).duration == pytest.approx(4.0, abs=0.01)


def test_reprobe_of_an_unknown_sound_is_none(tmp_path):
    assert SoundStore(str(tmp_path)).reprobe("nope") is None


# ── model round-trip ────────────────────────────────────────────────


def test_duration_survives_a_round_trip():
    s = Sound(id="x", name="X", duration=2.75)
    assert Sound.from_dict(s.to_dict()).duration == 2.75


def test_a_negative_duration_reads_as_unknown():
    assert Sound.from_dict({"id": "x", "name": "X", "duration": -3}).duration == 0.0


def test_a_missing_duration_reads_as_unknown():
    assert Sound.from_dict({"id": "x", "name": "X"}).duration == 0.0
