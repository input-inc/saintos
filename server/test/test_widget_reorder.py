"""Dashboard widget ordering — persisted so it survives a server restart.

`dashboard_order` is a flat sequence across every sheet, not a per-sheet
index, because the dashboard flattens widgets from all sheets: ordering
one sheet's list would leave the cross-sheet order at the mercy of dict
iteration.
"""
from __future__ import annotations

import os
import sys

import pytest

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from saint_server.peripheral_model import SystemRouting, WidgetInstance


def _routing(*specs):
    """Build a routing graph: each spec is (sheet_id, [widget_id, ...])."""
    r = SystemRouting()
    for sheet_id, ids in specs:
        sheet = r.get_sheet(sheet_id)
        for i, wid in enumerate(ids):
            sheet.widgets.append(WidgetInstance(
                id=wid, type="single_gauge", label=wid, dashboard_order=i))
    return r


def _order(routing):
    """Widgets across all sheets, in dashboard order."""
    all_w = [w for s in routing.sheets.values() for w in s.widgets]
    return [w.id for w in sorted(all_w, key=lambda w: w.dashboard_order)]


# ── Model ────────────────────────────────────────────────────────────

def test_order_survives_serialization():
    """The whole point: it has to come back after a restart, which means
    surviving the round trip through system_routing.yaml."""
    w = WidgetInstance(id="w1", type="single_gauge", label="Gauge",
                       position=(40, 80), dashboard_order=7)
    back = WidgetInstance.from_dict(w.to_dict())
    assert back.dashboard_order == 7
    # Canvas position must round-trip independently — they're two
    # different layouts of the same widget.
    assert back.position == (40, 80)


def test_legacy_widget_dict_defaults_to_zero():
    """Configs written before ordering existed have no such key. They must
    all tie at 0 so a stable sort preserves their previous order rather
    than shuffling on upgrade."""
    legacy = {"id": "w1", "type": "single_gauge", "label": "Gauge"}
    assert WidgetInstance.from_dict(legacy).dashboard_order == 0


def test_non_numeric_order_falls_back_to_zero():
    """A hand-edited yaml shouldn't crash the load."""
    bad = {"id": "w1", "type": "single_gauge", "label": "G",
           "dashboard_order": "third"}
    assert WidgetInstance.from_dict(bad).dashboard_order == 0


# ── reorder_widgets ──────────────────────────────────────────────────

@pytest.fixture
def sm(tmp_path):
    """A StateManager with persistence pointed at a temp dir."""
    pytest.importorskip("yaml")
    from saint_server.webserver.state_manager import StateManager
    m = StateManager.__new__(StateManager)          # skip __init__'s ROS wiring
    m.config_dir = str(tmp_path)
    m.system_routing_path = str(tmp_path / "system_routing.yaml")
    m.logger = None
    m._routing_evaluator = None

    class _State:
        pass
    m.state = _State()
    m.state.system_routing = _routing(
        ("_dashboard", ["w1", "w2"]),
        ("node-a", ["w3"]),
    )
    return m


def test_reorder_across_sheets(sm):
    """Ordering has to work across sheet boundaries — the dashboard shows
    them in one grid."""
    r = sm.reorder_widgets(["w3", "w1", "w2"])
    assert r["success"] is True
    assert _order(sm.state.system_routing) == ["w3", "w1", "w2"]


def test_reorder_persists_to_disk(sm):
    """The requirement is restart-survival, so the write must actually
    happen — not just the in-memory mutation."""
    import yaml
    sm.reorder_widgets(["w3", "w2", "w1"])
    with open(sm.system_routing_path) as f:
        saved = yaml.safe_load(f)
    orders = {
        w["id"]: w["dashboard_order"]
        for s in saved["sheets"].values()
        for w in s["widgets"]
    }
    assert orders == {"w3": 0, "w2": 1, "w1": 2}


def test_reload_from_disk_restores_order(sm):
    """End to end: reorder, read the file back as a fresh graph, and the
    order is the one the operator chose."""
    import yaml
    sm.reorder_widgets(["w2", "w3", "w1"])
    with open(sm.system_routing_path) as f:
        reloaded = SystemRouting.from_dict(yaml.safe_load(f))
    assert _order(reloaded) == ["w2", "w3", "w1"]


def test_unlisted_widgets_sort_after_not_before(sm):
    """A client with a stale widget list shouldn't reshuffle cards it
    never knew about to the front."""
    r = sm.reorder_widgets(["w3"])
    assert r["success"] is True
    order = _order(sm.state.system_routing)
    assert order[0] == "w3"
    # w1/w2 keep their existing relative order behind it.
    assert order[1:] == ["w1", "w2"]


def test_unknown_id_is_rejected_without_mutating(sm):
    before = _order(sm.state.system_routing)
    r = sm.reorder_widgets(["w1", "ghost-9"])
    assert r["success"] is False
    assert "ghost-9" in r["message"]
    assert _order(sm.state.system_routing) == before, "rejected call must not reorder"


def test_non_list_rejected(sm):
    assert sm.reorder_widgets("w1,w2")["success"] is False


def test_duplicate_ids_use_first_position(sm):
    """A duplicated id in the payload shouldn't produce two widgets
    claiming the same slot."""
    r = sm.reorder_widgets(["w2", "w1", "w2"])
    assert r["success"] is True
    order = _order(sm.state.system_routing)
    assert order.index("w2") < order.index("w1")
    assert len(order) == len(set(order))
