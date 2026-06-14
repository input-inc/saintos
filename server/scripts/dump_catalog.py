#!/usr/bin/env python3
"""Dump the peripheral / widget / operator catalogs to stdout as JSON.

Used by the JS mock server (`web/dev/mock-server.js`) at boot so the
mock returns the same catalog the real Python server does — no hand-
maintained mirror in JS to drift out of sync.

Standalone by design: imports `peripheral_model.py` directly via
importlib instead of going through `saint_server.__init__`, so this
runs on any machine with plain Python 3, no rclpy install required.
"""

from __future__ import annotations

import importlib.util
import json
import os
import sys


def _load_peripheral_model():
    here = os.path.dirname(os.path.abspath(__file__))
    src = os.path.normpath(os.path.join(here, "..", "saint_server", "peripheral_model.py"))
    spec = importlib.util.spec_from_file_location("peripheral_model", src)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"could not load peripheral_model.py at {src}")
    m = importlib.util.module_from_spec(spec)
    # Register in sys.modules BEFORE exec — Python 3.9's @dataclass
    # processor walks sys.modules[cls.__module__].__dict__ during
    # ClassVar resolution and KeyErrors if the module isn't there yet.
    sys.modules["peripheral_model"] = m
    spec.loader.exec_module(m)
    return m


def main() -> int:
    m = _load_peripheral_model()
    out = {
        "peripheral_types": [t.to_dict() for t in m.DEFAULT_CATALOG.values()],
        "widget_types":     [t.to_dict() for t in m.DEFAULT_WIDGET_CATALOG.values()],
        "operator_types":   [t.to_dict() for t in m.OPERATOR_CATALOG.values()],
    }
    json.dump(out, sys.stdout)
    sys.stdout.write("\n")
    return 0


if __name__ == "__main__":
    sys.exit(main())
