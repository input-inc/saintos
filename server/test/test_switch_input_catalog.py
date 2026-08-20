"""switch_input catalog contract + wire normalization.

The generic switch/sensor input (docs/SENSOR_INPUTS.md) spans three
declarations of one contract — the C header, the catalog entry, and the
config push. These tests pin the parts a refactor could silently drift.
"""
from __future__ import annotations

import os
import re
import sys

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from saint_server.peripheral_model import (
    DEFAULT_CATALOG,
    SWITCH_INPUT_MAX_TARGETS,
    switch_input_params_for_wire,
)

HEADER = os.path.join(
    os.path.dirname(__file__), "..", "..",
    "firmware", "shared", "include", "switch_input_protocol.h")


def _header_define(name):
    with open(HEADER) as fh:
        text = fh.read()
    m = re.search(r"#define\s+%s\s+(\d+)" % re.escape(name), text)
    assert m, f"{name} not found in switch_input_protocol.h"
    return int(m.group(1))


def test_catalog_channels_match_firmware_sub_indices():
    """The catalog channel order must match the SWITCH_SUB_* indices, or
    the Live tab labels one reading with another's name."""
    expected = ["state", "latched", "voltage", "trip_count"]
    actual = [c.id for c in DEFAULT_CATALOG["switch_input"].channels]
    assert actual == expected, (
        f"catalog channel order {actual} doesn't match the SWITCH_SUB_* "
        f"order {expected} in switch_input_protocol.h"
    )
    assert _header_define("SWITCH_INPUT_CHANNELS_PER_UNIT") == len(expected)
    for i, cid in enumerate(expected):
        assert _header_define(f"SWITCH_SUB_{cid.upper()}") == i


def test_max_targets_matches_firmware():
    """Truncating to a different number than the firmware stores would
    let the UI claim targets that were never actually armed."""
    assert SWITCH_INPUT_MAX_TARGETS == _header_define("SWITCH_INPUT_MAX_TARGETS")


def test_trip_action_values_match_firmware():
    params = {p.id: p for p in DEFAULT_CATALOG["switch_input"].params}
    values = [c["value"] for c in params["on_trip"].choices]
    assert values == [
        _header_define("SWITCH_TRIP_NONE"),
        _header_define("SWITCH_TRIP_STOP_TARGETS"),
        _header_define("SWITCH_TRIP_ESTOP_NODE"),
    ]


def test_safe_defaults():
    """A freshly added switch must not stop anything until the operator
    says so, and must latch so a brief pulse isn't lost."""
    params = {p.id: p for p in DEFAULT_CATALOG["switch_input"].params}
    assert params["on_trip"].default == 0, "must default to report-only"
    assert params["latch"].default is True
    assert params["active_low"].default is True, (
        "normally-closed is the common limit-switch wiring, and it makes a "
        "cut cable read as tripped"
    )


def test_targets_param_is_a_picker_not_free_text():
    """A typo'd interlock target fails silently at exactly the wrong
    moment, so this must not be a freeform string field."""
    params = {p.id: p for p in DEFAULT_CATALOG["switch_input"].params}
    assert params["targets"].type == "motion_peripherals"


def test_motion_types_are_flagged():
    """The target picker filters on commands_motion, so anything that
    drives a motor/servo/actuator has to carry it — and nothing that
    doesn't should."""
    motion = {k for k, v in DEFAULT_CATALOG.items() if v.commands_motion}
    assert motion == {
        "servo", "pwm", "roboclaw", "syren", "maestro",
        "pimoroni_servo2040", "tic", "tmc2208", "kangaroo",
    }
    for quiet in ("led", "neopixel", "mono_led", "audio_player",
                  "pathfinder_bms", "switch_input", "button", "analog_in"):
        assert not DEFAULT_CATALOG[quiet].commands_motion, (
            f"'{quiet}' is not motion and must not appear in the interlock "
            f"target picker")


def test_interlock_support_matches_firmware_reality():
    """supports_interlock gates whether a target is SELECTABLE, so it has
    to track which drivers actually implement the per-instance estop verb.
    Anything flagged here without `.command` in its driver would produce an
    interlock that logs 'could NOT stop' instead of stopping.

    Grepping the driver source is crude, but the alternative is a flag
    that drifts from the firmware silently.
    """
    fw = os.path.join(os.path.dirname(__file__), "..", "..",
                      "firmware", "shared", "src")
    for tid, ptype in DEFAULT_CATALOG.items():
        if not ptype.supports_interlock:
            continue
        driver = os.path.join(fw, f"{tid}_driver.c")
        assert os.path.exists(driver), (
            f"'{tid}' claims supports_interlock but has no shared driver")
        src = open(driver).read()
        assert ".command" in src, (
            f"'{tid}' claims supports_interlock but its driver registers no "
            f"command handler")
        assert '"estop"' in src, (
            f"'{tid}' claims supports_interlock but its driver handles no "
            f"'estop' verb")

    # servo/pwm are pin_control modes, not registered drivers — they can
    # never receive a peripheral_command, so they must never be flagged.
    for tid in ("servo", "pwm"):
        assert not DEFAULT_CATALOG[tid].supports_interlock, (
            f"'{tid}' is a pin_control mode, not a peripheral driver — it "
            f"cannot receive a peripheral_command")


def test_targets_string_becomes_a_list():
    out = switch_input_params_for_wire({"targets": "kangaroo-1, roboclaw-2"})
    assert out["targets"] == ["kangaroo-1", "roboclaw-2"]


def test_direction_suffix_survives_to_the_wire():
    """The ':<dir>' suffix is what lets the axis retreat off the switch.
    Strip it and every interlock silently becomes a full stop."""
    out = switch_input_params_for_wire(
        {"targets": "kangaroo-1:+, roboclaw-2:-, maestro-3"})
    assert out["targets"] == ["kangaroo-1:+", "roboclaw-2:-", "maestro-3"]


def test_block_direction_codes_match_firmware():
    """The UI writes '+'/'-' and the firmware decodes to SWITCH_BLOCK_*.
    An unannotated target must decode to BOTH — over-blocking is
    recoverable, a wrong direction drives further into the switch."""
    assert _header_define("SWITCH_BLOCK_BOTH") == 0
    assert _header_define("SWITCH_BLOCK_POSITIVE") == 1
    assert _header_define("SWITCH_BLOCK_NEGATIVE") == 2


def test_targets_blank_entries_dropped():
    out = switch_input_params_for_wire({"targets": " , kangaroo-1 ,,  "})
    assert out["targets"] == ["kangaroo-1"]


def test_targets_truncated_to_firmware_capacity():
    """Never put more on the wire than the firmware will arm — otherwise
    the UI shows targets that silently aren't protected."""
    many = ",".join(f"p{i}" for i in range(10))
    out = switch_input_params_for_wire({"targets": many})
    assert len(out["targets"]) == SWITCH_INPUT_MAX_TARGETS


def test_params_for_wire_does_not_mutate_input():
    src = {"targets": "a,b", "debounce_ms": 5}
    switch_input_params_for_wire(src)
    assert src["targets"] == "a,b", "must not mutate the stored config"


def test_analog_params_hidden_when_digital():
    params = {p.id: p for p in DEFAULT_CATALOG["switch_input"].params}
    for pid in ("threshold_mv", "hysteresis_mv"):
        assert params[pid].visible_when == {"sense_analog": True}
    assert params["pull_up"].visible_when == {"sense_analog": False}
    assert params["targets"].visible_when == {"on_trip": 1}
