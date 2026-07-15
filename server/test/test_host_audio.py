"""
Tests for the in-process host audio drivers (audio_mixer / audio_player)
that let the operator control the server host's system volume from the
web UI / Steam Deck. ALSA is faked so this runs anywhere.
"""
import os
import sys
import types

import pytest

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))


class _FakeMixer:
    def __init__(self, control="Master", cardindex=0, channels=2):
        self.control = control
        self.cardindex = cardindex
        self.vol = [80] * channels
        self.muted = [0] * channels
        self.calls = []

    def getvolume(self):
        return list(self.vol)

    def setvolume(self, pct, ch=None):
        self.calls.append(("setvolume", pct, ch))
        if ch is None:
            self.vol = [pct] * max(1, len(self.vol))
        else:
            while len(self.vol) <= ch:
                self.vol.append(0)
            self.vol[ch] = pct

    def getmute(self):
        return list(self.muted)

    def setmute(self, state):
        self.calls.append(("setmute", state))
        self.muted = [state] * max(1, len(self.muted))

    def close(self):
        self.calls.append(("close",))


@pytest.fixture
def audio(monkeypatch):
    created = {}

    def _mixer(control="Master", cardindex=0):
        m = _FakeMixer(control, cardindex)
        created["last"] = m
        return m

    fake = types.SimpleNamespace(Mixer=_mixer)
    monkeypatch.setitem(sys.modules, "alsaaudio", fake)
    import importlib
    from saint_server.host_peripherals import audio as audio_mod
    importlib.reload(audio_mod)
    audio_mod._created = created  # expose for assertions
    return audio_mod


def _mixer_driver(audio, params=None):
    events = []
    drv = audio.HostAudioMixerDriver(
        "host_mixer", params or {},
        state_cb=lambda pid, ch, v: events.append((ch, v)))
    return drv, events


# ── audio_mixer ─────────────────────────────────────────────────────


def test_mixer_start_applies_default_volume_and_telemetry(audio):
    drv, events = _mixer_driver(audio, {"default_volume": 0.6})
    drv.start()
    assert drv.is_connected
    m = audio._created["last"]
    # default 0.6, centered → both channels 60%
    assert ("setvolume", 60, 0) in m.calls and ("setvolume", 60, 1) in m.calls
    # telemetry emitted: left/right/muted
    chans = {ch for ch, _ in events}
    assert {"left", "right", "muted"} <= chans


def test_mixer_volume_channel_write(audio):
    drv, events = _mixer_driver(audio, {"default_volume": 0.8})
    drv.start()
    m = audio._created["last"]
    m.calls.clear()
    drv.set_channel("volume", 0.5)
    assert ("setvolume", 50, 0) in m.calls and ("setvolume", 50, 1) in m.calls


def test_mixer_balance_pans(audio):
    drv, _ = _mixer_driver(audio, {"default_volume": 1.0})
    drv.start()
    m = audio._created["last"]
    m.calls.clear()
    drv.set_channel("balance", 1.0)   # full right → left attenuated to 0
    assert ("setvolume", 0, 0) in m.calls    # left channel 0
    assert ("setvolume", 100, 1) in m.calls  # right channel full


def test_mixer_mute(audio):
    drv, _ = _mixer_driver(audio)
    drv.start()
    m = audio._created["last"]
    m.calls.clear()
    drv.set_channel("mute", 1.0)
    assert ("setmute", 1) in m.calls
    drv.set_channel("mute", 0.0)
    assert ("setmute", 0) in m.calls


def test_mixer_unavailable_is_graceful(audio, monkeypatch):
    monkeypatch.setattr(audio, "_ALSA_AVAILABLE", False)
    drv, _ = _mixer_driver(audio)
    drv.start()          # must not raise
    assert not drv.is_connected


# ── audio_player path containment (no VLC needed) ───────────────────


def test_player_resolve_rejects_escape(audio, tmp_path):
    lib = tmp_path / "audio"
    lib.mkdir()
    (lib / "ok.wav").write_bytes(b"\0")
    (tmp_path / "secret.wav").write_bytes(b"\0")
    drv = audio.HostAudioPlayerDriver(
        "host_player", {"library_path": str(lib)}, state_cb=lambda *a: None)
    assert drv._resolve("ok.wav") == str(lib / "ok.wav")
    assert drv._resolve("../secret.wav") is None   # escape blocked
    assert drv._resolve("missing.wav") is None      # not present


if __name__ == "__main__":
    sys.exit(pytest.main([__file__, "-v"]))
