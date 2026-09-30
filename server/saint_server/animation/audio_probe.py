"""Read an audio file's duration.

The animation timeline draws a sound board-item as a bar the length of
the clip, so it needs a real duration. Nothing else in SAINT.OS knew one:
a Sound carried volume, start offset and loop options but no length.

Soundboard files live on the server's own filesystem, so this reads them
directly rather than asking a node. Three backends, cheapest first, each
optional — a host with none of them still runs, it just reports 0:

  1. stdlib ``wave`` / ``aifc`` / ``sunau`` — exact for uncompressed
     audio, no dependency and no subprocess. Covers the .wav files that
     make up most soundboards.
  2. ``ffprobe`` — handles everything else (mp3, ogg, flac, m4a, opus)
     and is the usual companion to a VLC install.
  3. ``python-vlc`` — already an optional runtime dependency for
     playback, so a host that can play a file can usually measure it.

0.0 means "unknown", never "zero length". Callers must treat it as
absent rather than drawing a zero-width bar.
"""

from __future__ import annotations

import contextlib
import json
import os
import subprocess
from typing import Optional

# Bound every backend. A malformed or enormous file must not wedge a
# save; an unknown duration is a recoverable outcome and a hung web
# request is not.
PROBE_TIMEOUT_S = 10.0


def _probe_stdlib(path: str) -> Optional[float]:
    """Uncompressed formats, via the standard library."""
    import aifc
    import sunau
    import wave

    for module, mode in ((wave, "rb"), (aifc, "rb"), (sunau, "r")):
        try:
            with contextlib.closing(module.open(path, mode)) as handle:
                rate = handle.getframerate()
                frames = handle.getnframes()
                if rate > 0 and frames > 0:
                    return frames / float(rate)
        except Exception:
            continue
    return None


def _probe_ffprobe(path: str) -> Optional[float]:
    try:
        out = subprocess.run(
            ["ffprobe", "-v", "error", "-show_entries", "format=duration",
             "-of", "json", path],
            capture_output=True, text=True, timeout=PROBE_TIMEOUT_S)
    except (OSError, subprocess.SubprocessError):
        return None
    if out.returncode != 0:
        return None
    try:
        value = json.loads(out.stdout)["format"]["duration"]
        seconds = float(value)
    except (ValueError, KeyError, TypeError):
        return None
    return seconds if seconds > 0 else None


def _probe_vlc(path: str) -> Optional[float]:
    try:
        import vlc  # type: ignore
    except Exception:
        return None
    try:
        # A parse-only instance: no audio output is opened, so probing
        # can never make a sound or grab the operator's output device.
        inst = vlc.Instance("--no-video", "--no-audio", "--quiet")
        media = inst.media_new_path(path)
        media.parse_with_options(vlc.MediaParseFlag.local, int(PROBE_TIMEOUT_S * 1000))
        # parse_with_options is asynchronous; get_duration returns -1
        # until it lands, so poll rather than trusting the first read.
        deadline = PROBE_TIMEOUT_S
        step = 0.05
        ms = media.get_duration()
        while ms <= 0 and deadline > 0:
            import time
            time.sleep(step)
            deadline -= step
            ms = media.get_duration()
        return (ms / 1000.0) if ms > 0 else None
    except Exception:
        return None


def probe_duration(path: str) -> float:
    """Seconds of audio at ``path``; 0.0 when it can't be determined.

    Never raises: a probe failure degrades to "unknown", which the
    timeline renders as a marker rather than a bar.
    """
    if not path or not os.path.isfile(path):
        return 0.0
    for backend in (_probe_stdlib, _probe_ffprobe, _probe_vlc):
        try:
            seconds = backend(path)
        except Exception:
            seconds = None
        if seconds and seconds > 0:
            return round(float(seconds), 3)
    return 0.0
