"""Deterministic parser + validator tests for the WiFi admin code.

These cover the input-formats-from-tools-we-don't-control side of the
WiFi feature: `iw dev … station dump`, `iw dev … scan`, and
/proc/net/wireless. If a future Debian/Pi update changes the column
ordering or strings, these tests catch the regression before an
operator sees stale telemetry on the dashboard.

What's deliberately NOT here:
  * The live `subprocess.run` calls — they need root + real WiFi
    hardware. Worth keeping a wifi-hardware smoke script under
    server/scripts/ later, but it's not a unit test.
  * The asyncio executor flow in websocket_handler. Integration, not
    leaf logic.
"""

from __future__ import annotations

import io
from unittest.mock import patch

import pytest

from saint_server import wifi_stats
from saint_server import wifi_admin


# ── wifi_stats: iw station dump ─────────────────────────────────────

# Two stations associated. Station 01 has higher signal_avg, so it
# should win _pick_active_station. The fixture intentionally mixes
# spacing (tabs after the colon, multiple spaces) because real iw
# output is irregular and the parser shouldn't care.
_STATION_DUMP = """\
Station aa:bb:cc:dd:ee:01 (on wlan0)
\tinactive time:\t120 ms
\trx packets:\t152034
\ttx packets:\t81204
\ttx retries:\t5023
\ttx failed:\t12
\tsignal:  \t-58 dBm
\tsignal avg:\t-56 dBm
\ttx bitrate:\t144.4 MBit/s
Station aa:bb:cc:dd:ee:02 (on wlan0)
\tinactive time:\t5 ms
\trx packets:\t8001
\ttx packets:\t3204
\ttx retries:\t44
\tsignal:  \t-72 dBm
\tsignal avg:\t-71 dBm
\ttx bitrate:\t72.2 MBit/s
"""


def test_station_dump_parses_both_stations():
    stations = wifi_stats._parse_station_dump(_STATION_DUMP)
    assert len(stations) == 2
    assert stations[0]["mac"] == "aa:bb:cc:dd:ee:01"
    assert stations[0]["signal_avg_dbm"] == -56
    assert stations[0]["tx_bitrate_mbps"] == 144.4
    assert stations[1]["mac"] == "aa:bb:cc:dd:ee:02"


def test_station_dump_empty_yields_empty_list():
    assert wifi_stats._parse_station_dump("") == []


def test_pick_active_station_prefers_strongest_signal_avg():
    stations = wifi_stats._parse_station_dump(_STATION_DUMP)
    active = wifi_stats._pick_active_station(stations)
    assert active["mac"] == "aa:bb:cc:dd:ee:01"


def test_pick_active_station_returns_none_when_no_stations():
    assert wifi_stats._pick_active_station([]) is None


def test_pick_active_station_falls_back_to_instant_signal():
    # No signal_avg on either station — picker should fall back to signal
    stations = [
        {"mac": "x", "signal_dbm": -80},
        {"mac": "y", "signal_dbm": -50},
    ]
    assert wifi_stats._pick_active_station(stations)["mac"] == "y"


def test_retry_pct_math_in_collect_path():
    """The retry_pct calculation lives inside collect(); we exercise it
    by mocking the subprocess + /proc reads. Validates the math, not
    the iw invocation."""
    with patch.object(wifi_stats, "_run") as run, \
         patch.object(wifi_stats, "_read_noise_floor", return_value=-95.0), \
         patch.object(wifi_stats, "detect_wifi_interface", return_value="wlan0"):
        run.return_value = _STATION_DUMP
        snap = wifi_stats.collect()
        # 5023 / (81204 + 5023) * 100 ≈ 5.82
        assert snap.retry_pct is not None
        assert 5.5 < snap.retry_pct < 6.0
        assert snap.signal_dbm == -56
        assert snap.bitrate_mbps == 144.4
        assert snap.noise_dbm == -95.0
        assert snap.client_count == 2


def test_collect_returns_empty_when_no_iface():
    with patch.object(wifi_stats, "detect_wifi_interface", return_value=None):
        snap = wifi_stats.collect()
        assert snap.signal_dbm is None
        assert snap.client_count == 0


# ── wifi_stats: /proc/net/wireless ─────────────────────────────────

_PROC_NET_WIRELESS = """\
Inter-| sta-|   Quality        |   Discarded packets               | Missed | WE
 face | tus | link level noise |  nwid  crypt   frag  retry   misc | beacon | 22
 wlan0: 0000   53.   -57.  -89.    0      0      0      0      0        0
"""


def test_noise_floor_parses_from_proc():
    with patch("builtins.open", return_value=io.StringIO(_PROC_NET_WIRELESS)):
        assert wifi_stats._read_noise_floor("wlan0") == -89.0


def test_noise_floor_returns_none_when_iface_missing():
    fixture = (
        "Inter-| sta-|   Quality        |   Discarded packets        \n"
        " face | tus | link level noise |  nwid  crypt              \n"
        " wlan1: 0000   53.   -57.  -89.    0     0\n"
    )
    with patch("builtins.open", return_value=io.StringIO(fixture)):
        assert wifi_stats._read_noise_floor("wlan0") is None


def test_noise_floor_returns_none_when_proc_unreadable():
    def oserror(*args, **kwargs):
        raise OSError("permission denied")
    with patch("builtins.open", side_effect=oserror):
        assert wifi_stats._read_noise_floor("wlan0") is None


# ── wifi_admin: iw scan parser ──────────────────────────────────────

_SCAN_OUTPUT = """\
BSS aa:bb:cc:dd:ee:01(on wlan0)
\tTSF: 12345 usec
\tfreq: 2412
\tcapability: ESS Privacy (0x0411)
\tsignal: -42.00 dBm
\tSSID: NeighborA
BSS aa:bb:cc:dd:ee:02(on wlan0)
\tfreq: 2412
\tsignal: -67.00 dBm
\tSSID: NeighborB
BSS aa:bb:cc:dd:ee:03(on wlan0)
\tfreq: 2437
\tsignal: -55.00 dBm
\tSSID: ShopWiFi
BSS aa:bb:cc:dd:ee:04(on wlan0)
\tfreq: 5180
\tsignal: -71.00 dBm
\tSSID: Office5G
BSS aa:bb:cc:dd:ee:05(on wlan0)
\tfreq: 2412
"""


def test_scan_groups_by_frequency():
    by_freq = wifi_admin._parse_scan(_SCAN_OUTPUT)
    # ch1 has three APs — including one with no signal line
    assert len(by_freq[2412]) == 3
    assert -42.0 in by_freq[2412]
    assert -67.0 in by_freq[2412]
    # The third AP on 2412 had no `signal:` line; parser inserts -100
    # placeholder so the count is still accurate.
    assert -100.0 in by_freq[2412]
    assert by_freq[2437] == [-55.0]
    assert by_freq[5180] == [-71.0]


def test_scan_empty_input():
    assert wifi_admin._parse_scan("") == {}


# ── wifi_admin: freq → (band, channel) ──────────────────────────────

@pytest.mark.parametrize("freq,expected", [
    (2412, ("2.4", 1)),
    (2437, ("2.4", 6)),
    (2462, ("2.4", 11)),
    (2484, ("2.4", 14)),     # Japan-only channel
    (5180, ("5", 36)),
    (5260, ("5", 52)),       # DFS
    (5825, ("5", 165)),
])
def test_freq_to_channel_mapping(freq, expected):
    assert wifi_admin._freq_to_channel(freq) == expected


@pytest.mark.parametrize("freq", [2400, 2500, 5000, 6000, 60000])
def test_freq_to_channel_returns_none_for_unsupported(freq):
    assert wifi_admin._freq_to_channel(freq) is None


# ── wifi_admin: credential validation ───────────────────────────────

def test_credentials_valid():
    assert wifi_admin._validate_credentials("OpenSAINT", "validpass8") is None


def test_credentials_ssid_too_short():
    err = wifi_admin._validate_credentials("", "validpass8")
    assert err is not None and "SSID" in err


def test_credentials_ssid_too_long():
    err = wifi_admin._validate_credentials("x" * 33, "validpass8")
    assert err is not None and "SSID" in err


def test_credentials_password_too_short():
    err = wifi_admin._validate_credentials("OK", "short")
    assert err is not None and "Password" in err


def test_credentials_password_too_long():
    err = wifi_admin._validate_credentials("OK", "x" * 64)
    assert err is not None and "Password" in err


def test_credentials_rejects_non_ascii_ssid():
    err = wifi_admin._validate_credentials("café", "validpass8")
    assert err is not None and "SSID" in err and "ASCII" in err


def test_credentials_rejects_non_ascii_password():
    err = wifi_admin._validate_credentials("OK", "passéword123")
    assert err is not None and "Password" in err and "ASCII" in err


# ── wifi_admin: survey() sort order ─────────────────────────────────

def test_survey_orders_current_first_then_least_busy(monkeypatch):
    """survey() must put the current channel at the top (so the
    operator's reference point is obvious), then sort the rest by
    AP count ascending, then by strongest signal (quieter first).
    This is the contract the modal UI depends on."""
    monkeypatch.setattr(wifi_admin, "detect_wifi_interface",
                        lambda: "wlan0")
    monkeypatch.setattr(wifi_admin, "_run",
                        lambda cmd, **kw: _SCAN_OUTPUT
                        if "scan" in cmd else "wiphy 0\nchannel 6 (2437 MHz)")
    monkeypatch.setattr(wifi_admin, "_list_supported_frequencies",
                        lambda iface: [2412, 2437, 2462, 5180, 5825])
    monkeypatch.setattr(wifi_admin, "_get_current_channel",
                        lambda iface: 6)

    result = wifi_admin.survey()
    assert result["ok"] is True
    rows = result["channels"]
    assert len(rows) == 5

    # Row 0 must be the current channel.
    assert rows[0]["is_current"] is True
    assert rows[0]["channel"] == 6

    # Subsequent rows: AP-count ascending. ch11 (5180 has 1 AP, 5825 has 0
    # APs, 2462 has 0 APs) — quiet channels float to the top, busy at
    # the bottom.
    remainder = rows[1:]
    ap_counts = [r["ap_count"] for r in remainder]
    assert ap_counts == sorted(ap_counts), \
        f"non-current rows not sorted by AP count: {ap_counts}"

    # The 2.4 GHz channels and 5 GHz channels both appear (not band-filtered).
    bands = {r["band"] for r in rows}
    assert bands == {"2.4", "5"}


def test_survey_returns_error_when_no_iface(monkeypatch):
    monkeypatch.setattr(wifi_admin, "detect_wifi_interface", lambda: None)
    result = wifi_admin.survey()
    assert result["ok"] is False
    assert "interface" in (result.get("error") or "").lower()
