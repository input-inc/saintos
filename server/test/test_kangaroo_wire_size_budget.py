"""Regression budget: Kangaroo config JSON must fit XRCE-DDS limits.

Same class of protection as test_maestro_wire_size_budget.py and
test_pimoroni_servo2040_wire_size_budget.py. The firmware receives
peripheral config over a single ROS2 String on /saint/nodes/<id>/config,
bounded by:

  - Wire MTU per UDP frame:     512
  - XRCE-DDS reassembly cap:    MTU x MAX_HISTORY ~= 2048

Unlike the Maestro and Servo 2040, the Kangaroo has no per-channel array
— its params are flat, so a single peripheral is small. The pressure
here is COUNT: KANGAROO_MAX_UNITS is 8, so a node can legitimately carry
eight Kangaroo peripherals in one config push, and the linear-actuator
mode added five params to each. This test fails CI if that combination
outgrows the reassembly cap.

See docs/KANGAROO_BRINGUP.md for the wire-size constraints.
"""
from __future__ import annotations

import json
import os
import re
import sys

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from saint_server.peripheral_model import (
    DEFAULT_CATALOG,
    kangaroo_slim_params_for_wire,
)
from saint_server.webserver.state_manager import _FIRMWARE_CHANNEL_MAP

XRCE_SINGLE_FRAME_MTU     = 512
XRCE_REASSEMBLY_CAP_BYTES = 4 * 512   # 2048

# Mirrors KANGAROO_MAX_UNITS in firmware/shared/include/kangaroo_protocol.h.
KANGAROO_MAX_UNITS = 8


def _wire_size(peripheral_dicts):
    payload = {"action": "configure", "version": 1, "peripherals": peripheral_dicts}
    return len(json.dumps(payload, separators=(",", ":")))


def _all_params_set(index=0):
    """Every catalog param at a non-default, worst-case-length value —
    what the operator's saved config looks like with nothing left
    defaulted."""
    return {
        "address": 128 + index,
        "channel": "1",
        "protocol": "packet",
        "home_on_start": False,
        "max_position": 536870911,
        "max_speed": 536870911,
        "baud": 115200,
        "motion_mode": "linear",
        "jog_power_pct": 100,
        "home_position": -536870911,
        "power_on_enabled": True,
        "power_on_position": 536870911,
    }


def _peripheral(index=0):
    """Shaped exactly like what state_manager pushes — slimming included,
    so the test measures what the node actually receives."""
    return {
        "id": f"kangaroo-{index + 1}",
        "type": "kangaroo",
        "pins": {"tx": 0, "rx": 1},
        "params": kangaroo_slim_params_for_wire(_all_params_set(index)),
    }


def test_server_only_params_stay_off_the_wire():
    """motion_mode, home/power-on positions are acted on server-side. They
    must never reach the node — they'd only burn the XRCE budget."""
    encoded = json.dumps(_peripheral())
    for key in ("motion_mode", "home_position",
                "power_on_enabled", "power_on_position"):
        assert key not in encoded, f"'{key}' leaked onto the wire: {encoded}"

    # ...but the open-loop power cap MUST reach the firmware: it bounds
    # how hard an untuned axis is driven with no feedback and no travel
    # limits, and that limit belongs in the firmware.
    assert "jog_power_pct" in encoded, (
        "jog_power_pct must stay on the wire — the firmware owns the "
        "open-loop power cap"
    )


def test_channel_stride_matches_firmware_header():
    """The virtual-GPIO stride in state_manager._FIRMWARE_CHANNEL_MAP and
    KANGAROO_CHANNELS_PER_UNIT in the C header are two declarations of one
    wire contract. If they drift, unit 1's channels silently decode as
    unit 0's — wrong numbers on screen, no error anywhere. Read the header
    and compare rather than trusting a comment.
    """
    header = os.path.join(
        os.path.dirname(__file__), "..", "..",
        "firmware", "shared", "include", "kangaroo_protocol.h")
    with open(header) as fh:
        text = fh.read()

    m = re.search(r"#define\s+KANGAROO_CHANNELS_PER_UNIT\s+(\d+)", text)
    assert m, "KANGAROO_CHANNELS_PER_UNIT not found in kangaroo_protocol.h"
    firmware_stride = int(m.group(1))

    base, mapping = _FIRMWARE_CHANNEL_MAP["kangaroo_motion"]
    assert base == 364, "virtual GPIO base drifted from the header"

    # Derive the server's stride from where unit 1's first channel lands.
    target_position_indices = sorted(
        idx for idx, name in mapping.items() if name == "target_position")
    server_stride = target_position_indices[1] - target_position_indices[0]

    assert server_stride == firmware_stride, (
        f"channel stride mismatch: firmware says {firmware_stride} per unit, "
        f"state_manager._FIRMWARE_CHANNEL_MAP uses {server_stride}. Update "
        f"the map in state_manager.py to match kangaroo_protocol.h."
    )

    # And every catalog channel must be reachable through the map, or the
    # Live tab shows a permanently blank field.
    catalog_ids = {c.id for c in DEFAULT_CATALOG["kangaroo"].channels}
    mapped_ids = set(mapping.values())
    missing = catalog_ids - mapped_ids
    assert not missing, (
        f"catalog channels with no virtual-GPIO mapping: {sorted(missing)}. "
        f"They will never receive a value."
    )


def test_catalog_exposes_linear_tune_params():
    """The teach-tune workflow is unreachable without these, and they must
    stay gated on motion_mode so rotational configs are unaffected."""
    params = {p.id: p for p in DEFAULT_CATALOG["kangaroo"].params}
    for pid in ("motion_mode", "jog_power_pct", "home_position",
                "power_on_enabled", "power_on_position"):
        assert pid in params, f"kangaroo catalog is missing '{pid}'"

    assert params["motion_mode"].default == "rotational", (
        "motion_mode must default to rotational so existing saved configs "
        "keep today's behavior"
    )
    assert params["power_on_enabled"].default is False, (
        "power-on motion must be opt-in — with absolute pot feedback the "
        "actuator should not move at startup"
    )
    for pid in ("jog_power_pct", "home_position", "power_on_enabled"):
        assert params[pid].visible_when == {"motion_mode": "linear"}, (
            f"'{pid}' must be gated on motion_mode=linear"
        )


def test_single_kangaroo_fits_one_frame():
    """One fully-specified linear Kangaroo must not fragment."""
    n = _wire_size([_peripheral()])
    assert n <= XRCE_SINGLE_FRAME_MTU, (
        f"A single fully-configured Kangaroo must fit the "
        f"{XRCE_SINGLE_FRAME_MTU}-byte XRCE frame; got {n}."
    )


def test_max_units_fit_reassembly_cap():
    """The load-bearing assertion: eight Kangaroos — the firmware's
    KANGAROO_MAX_UNITS — each with every linear param set, must still fit
    the XRCE reassembly cap. Past it the firmware crashes rather than
    rejecting the push, so this is a hard ceiling, not a guideline."""
    peripherals = [_peripheral(i) for i in range(KANGAROO_MAX_UNITS)]
    n = _wire_size(peripherals)
    assert n <= XRCE_REASSEMBLY_CAP_BYTES, (
        f"{KANGAROO_MAX_UNITS} fully-configured Kangaroos must fit the XRCE "
        f"reassembly cap ({XRCE_REASSEMBLY_CAP_BYTES} bytes); got {n}. Add a "
        f"slim-for-wire diff like pimoroni_slim_channels_for_wire, or drop "
        f"defaulted params from the push."
    )


def test_headroom_for_one_more_param():
    """Tripwire for the next person adding a wire-bound Kangaroo param.

    This is DELIBERATELY close to the line: at eight fully-customized
    units the payload sits around 1.8 KB against a 2048-byte cap, leaving
    roughly 25 bytes per unit. That is under one average param
    (`"jog_power_pct":100,` alone is 20 bytes), so the next field added to
    the wire will very likely need `kangaroo_slim_params_for_wire` to
    start dropping default-valued params too — not just server-only ones.

    Fail here, loudly, rather than on hardware where an over-cap config
    push crashes the node instead of being rejected.
    """
    peripherals = [_peripheral(i) for i in range(KANGAROO_MAX_UNITS)]
    n = _wire_size(peripherals)
    per_unit_headroom = (XRCE_REASSEMBLY_CAP_BYTES - n) / KANGAROO_MAX_UNITS
    assert per_unit_headroom >= 20, (
        f"Only {per_unit_headroom:.0f} bytes/unit of headroom left under the "
        f"{XRCE_REASSEMBLY_CAP_BYTES}-byte cap (payload {n}). Extend "
        f"kangaroo_slim_params_for_wire to drop default-valued params before "
        f"adding another field to the wire."
    )
