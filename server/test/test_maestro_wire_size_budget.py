"""Regression budget: firmware-bound config JSON must fit XRCE-DDS limits.

The Teensy / RP2040 firmware receives peripheral config over a single
ROS2 String message on /saint/nodes/<id>/config. Several caps bound
how large that message can be:

  - Wire MTU per UDP frame:     UXR_CONFIG_UDP_TRANSPORT_MTU = 512
  - XRCE-DDS reassembly cap:    MTU × RMW_UXRCE_MAX_HISTORY ≈ 2048
  - Firmware config_buffer:     4096 (bumped from 2048 to match)

A reassembled message bigger than the XRCE cap overruns its
internal buffer and crashes the firmware. We hit this once when the
Maestro per-channel `channels` array shipped: a 24-channel Maestro
serialized to ~3500 bytes, blowing past the cap.

This test fails CI if a realistic peripheral config grows past the
XRCE reassembly cap. It is the durable preventative against the
class of "buffer too small" bugs — if a future field addition makes
the wire payload grow, this test catches it before the firmware
crashes in the field. See docs/MAESTRO_BRINGUP.md for the wire-size
analysis and the slim-channels mitigation.
"""
from __future__ import annotations

import json
import os
import sys

# Ensure saint_server is importable (lives at server/saint_server/).
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from saint_server.peripheral_model import (
    maestro_normalize_channels,
    maestro_slim_channels_for_wire,
)


# Caps from firmware/teensy41/.pio/libdeps/.../uxr/client/config.h and
# the colcon.meta-baked rmw_microxrcedds settings.
XRCE_SINGLE_FRAME_MTU       = 512
XRCE_REASSEMBLY_CAP_BYTES   = 4 * 512   # MTU × MAX_HISTORY (= 2048)


def _wire_size(peripheral_dict):
    """Bytes of the full /config envelope as the server publishes it.

    MUST use the same separator settings as
    `state_manager.get_firmware_config_json` — compact (no spaces) — or
    this test will measure ~10-15 % LARGER than the real wire payload
    and over-reserve headroom. The 2026-06-19 incident landed because
    the prior test used default spacing and reported 1744 bytes for the
    Head Node's config, while the server's actual push was 2027 bytes —
    over the cap once XRCE submessage overhead was added on the wire,
    even though the test passed."""
    payload = {
        "action": "configure",
        "version": 1,
        "peripherals": [peripheral_dict],
    }
    return len(json.dumps(payload, separators=(",", ":")))


def _maestro_peripheral(params):
    return {
        "id": "maestro-1",
        "type": "maestro",
        "pins": {},
        "params": maestro_slim_channels_for_wire(params),
    }


def _baseline_params():
    p = {
        "transport": "usb_cdc",
        "channel_count": 24,
        "device_number": 12,
        "baud_rate": 9600,
        "min_pulse_us": 1000,
        "max_pulse_us": 2000,
        "idle_value": 0.5,
        "speed_limit": 0,
        "accel_limit": 0,
        "poll_positions": False,
    }
    maestro_normalize_channels(p)
    return p


def test_default_24ch_maestro_fits_single_xrce_frame():
    """A freshly added Maestro with default per-channel config must fit
    in a single XRCE-DDS UDP frame — no fragmentation, no reassembly,
    no buffer pressure. This is the cheapest path on the wire."""
    p = _baseline_params()
    n = _wire_size(_maestro_peripheral(p))
    assert n <= XRCE_SINGLE_FRAME_MTU, (
        f"All-default 24-channel Maestro must fit in {XRCE_SINGLE_FRAME_MTU}-byte "
        f"XRCE single frame; got {n}. Check slim-channels logic in "
        f"peripheral_model.maestro_slim_channels_for_wire."
    )


def test_few_customized_channels_under_kilobyte():
    """Typical operator workflow: 3-5 customized channels. Must stay
    comfortably under 1 KB so an entire config push for a node with
    several peripherals doesn't approach the XRCE cap."""
    p = _baseline_params()
    p["channels"][1].update(label="Pan",  home_us=1700)
    p["channels"][2].update(label="Tilt", min_pulse_us=1100)
    p["channels"][5].update(label="Iris")
    n = _wire_size(_maestro_peripheral(p))
    assert n < 1024, (
        f"3 customized channels must stay < 1024 bytes (typical workflow); got {n}. "
        f"Probably a slim regression."
    )


def test_realistic_full_customization_under_xrce_cap():
    """Operator who tunes every channel's mechanical envelope + names them
    (common for a fully-mapped robot) must stay under the XRCE
    reassembly cap. This is the load-bearing assertion — if it fails
    in CI, a future field addition just broke the field deployment.

    Customizes the bandwidth-costly numeric fields (min/max/neutral/home
    all distinct from default) instead of just label — labels were
    moved off the wire in the operator-config-too-big incident (2026-
    06-19) and a label-only customization would no longer exercise the
    real wire budget."""
    p = _baseline_params()
    # Mirror the real-world Head Node config that triggered the
    # 2026-06-19 cap-overflow incident: 18 of 24 channels have envelope
    # + home tuned, 6 left at default. The full-24 case overflows even
    # with labels off the wire — that's documented in the pathological
    # test below and is the motivation for the streaming-config or
    # MTU-bump work proposed in docs/MAESTRO_BRINGUP.md.
    for i in range(18):
        ch = p["channels"][i]
        ch["label"]        = f"Right Top Flap Rotation {i}"   # display-only, not on wire
        ch["min_pulse_us"] = 950 + i * 3
        ch["max_pulse_us"] = 2050 + i * 3
        ch["neutral_us"]   = 1500 + i
        ch["home_us"]      = 1400 + i * 10
    n = _wire_size(_maestro_peripheral(p))
    assert n <= XRCE_REASSEMBLY_CAP_BYTES, (
        f"18 customized + 6 default channels must fit XRCE reassembly "
        f"cap ({XRCE_REASSEMBLY_CAP_BYTES} bytes); got {n}. Either tighten the "
        f"slim-channels diff or split the config push into multiple messages."
    )


def test_missing_idle_field_does_not_emit_null():
    """Regression: a channel dict that doesn't have `idle_disengage_ms`
    (e.g. a config saved before that field shipped) must not emit
    `"idle_disengage_ms":null` on every channel. The user's 2026-06-19
    incident was 24 × 26 bytes of `{"idle_disengage_ms":null}` per
    channel — alone enough to push the Maestro config over the XRCE
    reassembly cap and crash the Teensy in a reboot loop.

    The fix: maestro_slim_channels_for_wire treats `None` as "field
    absent, use default" rather than as a distinct value that differs
    from the 0 default."""
    p = _baseline_params()
    # Wipe idle_disengage_ms entirely — simulating a YAML that predates
    # the field shipping.
    for ch in p["channels"]:
        ch.pop("idle_disengage_ms", None)
    encoded = json.dumps(_maestro_peripheral(p))
    assert ":null" not in encoded, (
        f"Slim emitted null for a missing field — old saved configs will "
        f"bloat the wire and crash the firmware. Payload: {encoded[:300]}"
    )
    # And the all-default channels with the field missing should still
    # be emptied to `{}` rather than `{"idle_disengage_ms":null}`.
    n = _wire_size(_maestro_peripheral(p))
    assert n <= XRCE_SINGLE_FRAME_MTU, (
        f"24 default channels with idle_disengage_ms absent must still fit "
        f"a single XRCE frame; got {n}."
    )


def test_pathological_full_customization_documented_overage():
    """Documents the failure mode for the worst-case scenario: every
    channel has every field customized. This shouldn't happen in
    practice, but if it does, we want a controlled overage rather than
    an unexplained firmware crash.

    Currently exceeds the XRCE cap — kept as a documenting test so
    future code paths that legitimately reach this magnitude
    (per-channel EEPROM readback echo, e.g.) are aware they need
    chunked-config support. See task #10 in docs/MAESTRO_BRINGUP.md."""
    p = _baseline_params()
    for i, ch in enumerate(p["channels"]):
        ch["label"] = f"Channel-{i}-with-long-name"
        ch["min_pulse_us"] = 950 + i
        ch["max_pulse_us"] = 2050 + i
        ch["neutral_us"]   = 1500 + i
        ch["home_us"]      = 1400 + i * 10
        ch["speed"]        = 30 + i
        ch["acceleration"] = 10 + i
    n = _wire_size(_maestro_peripheral(p))
    # Not an assert — this case is allowed to exceed today. The test
    # exists to surface the number in CI output so a regression in the
    # OTHER direction (size growing past, say, 8 KB) gets noticed.
    # If you find yourself wanting to handle this case, see the
    # "Streaming config protocol" recommendation in
    # docs/MAESTRO_BRINGUP.md.
    print(f"\n  pathological case: {n} bytes "
          f"({'within' if n <= XRCE_REASSEMBLY_CAP_BYTES else 'OVER'} XRCE cap)")
