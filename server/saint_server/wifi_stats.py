"""WiFi link health metrics for the host_controller's system_monitor.

Surfaces four numbers the operator cares about when the control link
gets flaky:

  * signal      — strongest associated client's average signal (dBm).
                  When the Pi is in AP mode (the default install
                  layout), the "active link" is the strongest station
                  — overwhelmingly the operator's Steam Deck. dBm
                  scale: -50 = excellent, -70 = marginal, -80 = bad.
  * retry_pct   — tx_retries / (tx_retries + tx_packets) × 100 for
                  that same station. Sustained >5 % means the AP is
                  retransmitting a lot, which usually points at
                  interference or a marginal physical link rather
                  than packet loss higher up the stack.
  * noise       — noise floor from /proc/net/wireless on the AP
                  interface (dBm). Slow-moving; a jump from -95 to
                  -80 = a microwave or another AP nearby on this
                  channel.
  * bitrate     — current PHY tx rate (Mbps) to that station. Rate-
                  adaptation drops this first when the link
                  deteriorates, before retries spike.

Everything returns None if the host has no WiFi interface or `iw`
isn't on PATH — happens on dev boxes and ethernet-only setups. The
caller filters None out before publishing channel values so widgets
just show "no data" instead of bogus zeros.
"""

from __future__ import annotations

import re
import subprocess
from dataclasses import dataclass
from typing import List, Optional


@dataclass
class WifiSnapshot:
    """One pass of telemetry. Any field can be None on a system that
    doesn't expose it (or when the AP has no associated clients yet)."""
    signal_dbm: Optional[float] = None
    retry_pct: Optional[float] = None
    noise_dbm: Optional[float] = None
    bitrate_mbps: Optional[float] = None
    # Number of currently-associated stations; useful as a sanity
    # check (signal/retry are derived from the strongest station, so
    # client_count = 0 means signal/retry will be None too).
    client_count: int = 0


def _run(cmd: List[str], timeout: float = 1.0) -> Optional[str]:
    """Run a short shell command with a hard timeout. Returns stdout
    on success, None on any failure — never raises. We swallow errors
    aggressively here because the broadcast loop calls this every
    second and a failed `iw` invocation must not stall it."""
    try:
        proc = subprocess.run(
            cmd, capture_output=True, text=True, timeout=timeout,
        )
        if proc.returncode != 0:
            return None
        return proc.stdout
    except (FileNotFoundError, subprocess.TimeoutExpired, OSError):
        return None


def detect_wifi_interface() -> Optional[str]:
    """Pick the AP interface — the one this script should pull stats
    from. Strategy:

      1. `iw dev` lists every wireless interface with its mode. Prefer
         one in "AP" mode (matches the install.sh AP setup). Fall back
         to the first interface listed if no AP is found, so a Pi
         configured as a client also reports something.
      2. If `iw dev` isn't available at all, return None — the caller
         will skip WiFi entirely.
    """
    out = _run(["iw", "dev"])
    if not out:
        return None

    # `iw dev` blocks are introduced by "Interface <name>" lines.
    # We scan each block, recording its name and mode.
    current_iface: Optional[str] = None
    first_iface: Optional[str] = None
    ap_iface: Optional[str] = None
    for line in out.splitlines():
        stripped = line.strip()
        if stripped.startswith("Interface "):
            current_iface = stripped.split(None, 1)[1]
            if first_iface is None:
                first_iface = current_iface
        elif stripped.startswith("type ") and current_iface:
            if stripped.split(None, 1)[1].strip() == "AP":
                ap_iface = current_iface
    return ap_iface or first_iface


def _parse_station_dump(text: str) -> List[dict]:
    """Parse the output of `iw dev <iface> station dump` into a list of
    per-station dicts. Each block starts with a `Station <mac>` header
    and contains `key: value` lines (with units we strip).

    We only extract the fields the snapshot uses. Unknown lines are
    ignored so the parser stays robust against `iw` adding new fields.
    """
    stations: List[dict] = []
    cur: Optional[dict] = None

    # Match the leading number from a value string like "-52 dBm" or
    # "144.4 MBit/s". This dodges unit-suffix variations across iw
    # versions (MBit/s vs Mbps, etc.) without writing a parser per
    # field.
    NUM_RE = re.compile(r"-?\d+(?:\.\d+)?")

    def num(s: str) -> Optional[float]:
        m = NUM_RE.search(s)
        return float(m.group(0)) if m else None

    for raw in text.splitlines():
        line = raw.rstrip()
        if line.startswith("Station "):
            if cur is not None:
                stations.append(cur)
            cur = {"mac": line.split()[1]}
            continue
        if cur is None:
            continue
        if ":" not in line:
            continue
        key, _, val = line.strip().partition(":")
        key = key.strip().lower()
        val = val.strip()
        if key == "signal":
            cur["signal_dbm"] = num(val)
        elif key == "signal avg":
            cur["signal_avg_dbm"] = num(val)
        elif key == "tx bitrate":
            cur["tx_bitrate_mbps"] = num(val)
        elif key == "tx packets":
            cur["tx_packets"] = num(val)
        elif key == "tx retries":
            cur["tx_retries"] = num(val)
        elif key == "tx failed":
            cur["tx_failed"] = num(val)
    if cur is not None:
        stations.append(cur)
    return stations


def _pick_active_station(stations: List[dict]) -> Optional[dict]:
    """The 'active link' from the operator's perspective is the strongest
    client (highest signal_avg). On a typical deployment that's the
    Steam Deck. If avg isn't available (some iw builds skip it),
    fall back to instant signal."""
    if not stations:
        return None

    def key(st: dict) -> float:
        # Higher dBm = stronger. None sentinels go to the bottom.
        return float(st.get("signal_avg_dbm")
                     or st.get("signal_dbm")
                     or -200.0)

    return max(stations, key=key)


def _read_noise_floor(iface: str) -> Optional[float]:
    """Pull the noise floor (dBm) from /proc/net/wireless.

    The file format is two header lines followed by one line per
    interface:

      Inter-| sta-|   Quality        |   Discarded packets         …
       face | tus | link level noise | nwid  crypt   frag  retry  …
       wlan0:  0   53.   -57.  -95.    0     0       0      0     …

    We just split on whitespace; column 4 is the noise value. The
    trailing '.' on each number is a iw-historical quirk — strip it.
    """
    try:
        with open("/proc/net/wireless", "r") as f:
            for line in f:
                if iface in line and ":" in line:
                    parts = line.replace(":", " ").split()
                    # parts[0]=iface, status, quality, level, noise
                    if len(parts) >= 5:
                        noise_str = parts[4].rstrip(".")
                        try:
                            return float(noise_str)
                        except ValueError:
                            return None
    except OSError:
        return None
    return None


def collect(iface: Optional[str] = None) -> WifiSnapshot:
    """Single-shot collection. Safe to call from the 1 Hz broadcast
    loop — total cost is two short subprocess invocations and one
    pseudo-file read."""
    if iface is None:
        iface = detect_wifi_interface()
    if not iface:
        return WifiSnapshot()

    snap = WifiSnapshot()
    snap.noise_dbm = _read_noise_floor(iface)

    dump = _run(["iw", "dev", iface, "station", "dump"])
    if dump is None:
        return snap
    stations = _parse_station_dump(dump)
    snap.client_count = len(stations)
    st = _pick_active_station(stations)
    if st is None:
        return snap

    snap.signal_dbm = st.get("signal_avg_dbm") or st.get("signal_dbm")
    snap.bitrate_mbps = st.get("tx_bitrate_mbps")

    tx_pkts = st.get("tx_packets")
    tx_retries = st.get("tx_retries")
    if tx_pkts is not None and tx_retries is not None:
        denom = tx_pkts + tx_retries
        if denom > 0:
            snap.retry_pct = (tx_retries / denom) * 100.0
    return snap
