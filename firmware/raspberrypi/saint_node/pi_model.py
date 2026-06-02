"""
Runtime detection of the Raspberry Pi model the node is running on.

The saint-node firmware ships as a single Python package that runs on
Pi 3, Pi 4, and Pi 5. Most of the code is model-agnostic — libgpiod
handles the GPIO chip name differences, ALSA handles audio device
differences. What still needs to be model-aware:

  - The ``hw`` field in the announcement message (so the operator sees
    "Raspberry Pi 4 Model B" vs "Raspberry Pi 5" in the node list, not
    a generic string).
  - Future Pi-specific feature flags (e.g. "this Pi has a 3.5mm audio
    jack" — true on Pi 3/4, false on Pi 5).

Detection reads ``/proc/device-tree/model``, which Linux populates from
the device tree the bootloader hands to the kernel. The file's content
is a single null-terminated string like ``Raspberry Pi 5 Model B Rev
1.0``. We strip the trailing null + whitespace and pattern-match.

The host fallback (model file missing, or content unrecognized) returns
``"Raspberry Pi"`` — the firmware still runs, the operator just sees a
less-specific label.
"""

from __future__ import annotations

import os
from typing import Optional


# Path the Pi's Linux kernel exposes the device-tree model on. Pi
# bootloaders populate this from `/etc/firmware`. Constant rather than
# inlined so tests can monkeypatch a temp file.
DEVICE_TREE_MODEL_PATH = "/proc/device-tree/model"


def read_device_tree_model(path: str = DEVICE_TREE_MODEL_PATH
                           ) -> Optional[str]:
    """Read the raw model string the kernel exposes. Returns ``None``
    when the file doesn't exist (we're not on a Pi, or the developer
    is testing on a Mac/x86 Linux box)."""
    try:
        with open(path, "rb") as f:
            raw = f.read()
    except FileNotFoundError:
        return None
    except OSError:
        return None
    # Strip null terminator + trailing whitespace.
    return raw.rstrip(b"\x00").rstrip().decode("utf-8", errors="replace")


def detect_pi_model(path: str = DEVICE_TREE_MODEL_PATH) -> str:
    """Return a human-readable Pi model string for the ``hw`` field of
    the node announcement message. Falls back to a generic ``"Raspberry
    Pi"`` when detection fails so the dashboard still gets *something*
    rather than an empty string."""
    raw = read_device_tree_model(path)
    if not raw:
        return "Raspberry Pi"
    if "Raspberry Pi" in raw:
        # Trim the "Rev 1.0" suffix the bootloader sometimes appends —
        # board revisions are noise for the dashboard.
        cleaned = raw
        if " Rev " in cleaned:
            cleaned = cleaned.split(" Rev ", 1)[0].rstrip()
        return cleaned
    # The kernel populates this on every Pi we care about; if the
    # string doesn't include "Raspberry Pi", we're not on one — but
    # the user may still be running the firmware deliberately (e.g. a
    # Compute Module 5 board, an Orange Pi clone). Pass it through.
    return raw


def detect_pi_generation(path: str = DEVICE_TREE_MODEL_PATH) -> Optional[int]:
    """Return ``3``, ``4``, ``5`` (etc.) for the major Pi generation.
    Returns ``None`` when detection fails. Useful for feature gates —
    e.g. "Pi 5 lacks a 3.5mm audio jack; default ALSA device differs"."""
    raw = read_device_tree_model(path)
    if not raw:
        return None
    # Look for "Pi <digit>" so the regex doesn't false-positive on
    # version-style suffixes like "Rev 1.0".
    for token in raw.split():
        if token.isdigit():
            try:
                n = int(token)
            except ValueError:
                continue
            if 1 <= n <= 99:    # bounded so a board rev doesn't slip through
                return n
    return None
