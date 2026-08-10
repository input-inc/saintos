"""Regression budget: Pimoroni Servo 2040 config JSON must fit XRCE-DDS limits.

Same class of protection as test_maestro_wire_size_budget.py — the
firmware receives peripheral config over a single ROS2 String on
/saint/nodes/<id>/config, bounded by:

  - Wire MTU per UDP frame:     512
  - XRCE-DDS reassembly cap:    MTU × MAX_HISTORY ≈ 2048
  - Firmware config_buffer:     4096

The Servo 2040 ships an 18-entry per-servo `channels` array (each with
start/end/center/home extents). If every field were emitted in full it
would rival the Maestro's ~3500-byte worst case, so
pimoroni_slim_channels_for_wire diffs each channel against its default
and emits `{}` for all-default channels. This test fails CI if that slim
logic regresses or a new field pushes the payload past the caps.
"""
from __future__ import annotations

import json
import os
import sys

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from saint_server.peripheral_model import (
    pimoroni_normalize_channels,
    pimoroni_slim_channels_for_wire,
)

XRCE_SINGLE_FRAME_MTU     = 512
XRCE_REASSEMBLY_CAP_BYTES = 4 * 512   # 2048


def _wire_size(peripheral_dict):
    payload = {"action": "configure", "version": 1, "peripherals": [peripheral_dict]}
    return len(json.dumps(payload, separators=(",", ":")))


def _peripheral(params):
    return {
        "id": "servo2040-1",
        "type": "pimoroni_servo2040",
        "pins": {},
        "params": pimoroni_slim_channels_for_wire(params),
    }


def _baseline_params():
    p = {
        "sda_pin": 2,
        "scl_pin": 3,
        "led_brightness": 255,
    }
    pimoroni_normalize_channels(p)
    return p


def test_default_board_fits_single_xrce_frame():
    """A freshly added Servo 2040 with default extents on all 18 channels
    must fit a single 512-byte XRCE frame — no fragmentation."""
    n = _wire_size(_peripheral(_baseline_params()))
    assert n <= XRCE_SINGLE_FRAME_MTU, (
        f"All-default Servo 2040 must fit {XRCE_SINGLE_FRAME_MTU}-byte XRCE "
        f"frame; got {n}. Check pimoroni_slim_channels_for_wire."
    )


def test_few_customized_channels_under_kilobyte():
    """Typical workflow: a handful of tuned servos stays well under 1 KB."""
    p = _baseline_params()
    p["channels"][1].update(label="Pan",  home_us=1700)
    p["channels"][2].update(label="Tilt", start_us=1100)
    p["channels"][5].update(label="Iris", end_us=1900)
    n = _wire_size(_peripheral(p))
    assert n < 1024, f"3 customized channels must stay < 1024 bytes; got {n}."


def test_full_customization_under_xrce_cap():
    """Every one of the 18 servos with a fully tuned envelope + home must
    still fit under the XRCE reassembly cap — the load-bearing assertion."""
    p = _baseline_params()
    for i in range(18):
        ch = p["channels"][i]
        ch["label"]     = f"Right Top Flap Rotation {i}"  # display-only, off the wire
        ch["start_us"]  = 950 + i * 3
        ch["end_us"]    = 2050 + i * 3
        ch["center_us"] = 1500 + i
        ch["home_us"]   = 1400 + i * 10
    n = _wire_size(_peripheral(p))
    assert n <= XRCE_REASSEMBLY_CAP_BYTES, (
        f"18 fully-customized channels must fit the XRCE reassembly cap "
        f"({XRCE_REASSEMBLY_CAP_BYTES} bytes); got {n}. Tighten the slim diff "
        f"or split the config push."
    )


def test_labels_stay_off_the_wire():
    """Per-channel display labels/icons must never reach the firmware —
    they're not in _PIMORONI_CHANNEL_KEYS, so slim must drop them even
    when set on every channel."""
    p = _baseline_params()
    for ch in p["channels"]:
        ch["label"] = "A very long operator label for this servo channel"
        ch["icon"]  = "open_with"
    encoded = json.dumps(_peripheral(p))
    assert "operator label" not in encoded, (
        f"Channel labels leaked onto the wire: {encoded[:300]}"
    )
    n = _wire_size(_peripheral(p))
    assert n <= XRCE_SINGLE_FRAME_MTU, (
        f"All-default extents (labels aside) must still fit a single frame; got {n}."
    )
