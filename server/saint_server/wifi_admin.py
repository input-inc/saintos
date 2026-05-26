"""WiFi AP administration — survey nearby channels and switch the AP.

This is the "Find Better Channel" backend. The Pi 4's brcmfmac driver
does NOT support `channel_switch` (verified via iw phy info — the
netlink command isn't in the supported-commands list), so a smooth
802.11h CSA isn't possible on this hardware. The realistic path is:

  1. Run `iw dev <iface> scan` to enumerate visible APs and tally
     them per channel (gives us a noise-by-occupancy proxy without
     having to flip the radio off-channel for survey dumps).
  2. Present the result to the operator.
  3. When they pick a channel, modify the saint-os-ap NM connection
     and restart it via nmcli. ~5–10 s outage; all clients drop and
     reconnect. The operator controls the timing by deciding when
     to press the button.

If you ever move to hardware that exposes `channel_switch` (USB
adapter with mac80211 — MT76 / iwlwifi / RTL8852BE), revisit:
hostapd_cli chan_switch is far less disruptive but requires the AP
to be managed by hostapd directly, not by NetworkManager.
"""

from __future__ import annotations

import re
import subprocess
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

from saint_server.wifi_stats import detect_wifi_interface, _run


# NetworkManager connection name for the AP, written by install.sh.
# Hard-coded because there's exactly one AP profile and the dashboard
# only manages that one.
AP_CONNECTION_NAME = "saint-os-ap"


# Channels supported in the US/EU/JP regulatory domains as the
# operator is likely to encounter. The Pi 4's iw phy output (from the
# diagnostics we ran) confirms exactly which freqs are enabled and
# which are DFS — we cross-reference here so the modal can mark them.
# DFS channels require radar detection and have an initial CAC silence
# period; they're listed but flagged so the operator knows the trade.
_DFS_5GHZ_CHANNELS = {
    52, 56, 60, 64, 100, 104, 108, 112, 116, 120, 124, 128,
    132, 136, 140, 144,
}


@dataclass
class ChannelInfo:
    """One row in the survey result. The UI table renders these directly."""
    band: str            # "2.4" or "5"
    channel: int
    freq_mhz: int
    ap_count: int = 0
    # Strongest competitor on this channel (dBm). Higher = more noise
    # from nearby APs; useful as a secondary tiebreaker when AP count
    # is the same.
    strongest_signal_dbm: Optional[float] = None
    is_dfs: bool = False
    is_current: bool = False

    def to_dict(self) -> Dict:
        return {
            "band": self.band,
            "channel": self.channel,
            "freq_mhz": self.freq_mhz,
            "ap_count": self.ap_count,
            "strongest_signal_dbm": self.strongest_signal_dbm,
            "is_dfs": self.is_dfs,
            "is_current": self.is_current,
        }


def _freq_to_channel(freq_mhz: int) -> Optional[Tuple[str, int]]:
    """Map a frequency to its (band, channel-number) tuple. Returns
    None for frequencies outside the bands we care about (e.g. 6 GHz
    or 60 GHz hardware that the Pi 4 doesn't support but might appear
    in a scan if some exotic device is nearby)."""
    if 2412 <= freq_mhz <= 2472:
        return ("2.4", (freq_mhz - 2407) // 5)
    if freq_mhz == 2484:
        return ("2.4", 14)
    if 5170 <= freq_mhz <= 5825:
        return ("5", (freq_mhz - 5000) // 5)
    return None


def _get_current_channel(iface: str) -> Optional[int]:
    """Parse `iw dev <iface> info` for the active channel number.
    Used to mark the current row in the modal so the operator knows
    where they are. None if the AP isn't currently up."""
    out = _run(["iw", "dev", iface, "info"])
    if not out:
        return None
    # Looks like: "channel 6 (2437 MHz), width: 20 MHz, ..."
    m = re.search(r"channel\s+(\d+)", out)
    return int(m.group(1)) if m else None


def _parse_scan(text: str) -> Dict[int, List[float]]:
    """Group scan results by frequency. Returns {freq_mhz: [signal_dbm, ...]}.

    `iw scan` output has one block per BSS, each starting with
    "BSS <mac>(on <iface>)" and including `freq:` and `signal:` lines.
    We don't care about SSIDs or anything else for the channel-busy
    summary — just count APs and capture their RSSI per frequency.
    """
    by_freq: Dict[int, List[float]] = {}
    cur_freq: Optional[int] = None
    cur_signal: Optional[float] = None

    def flush():
        nonlocal cur_freq, cur_signal
        if cur_freq is not None:
            by_freq.setdefault(cur_freq, [])
            if cur_signal is not None:
                by_freq[cur_freq].append(cur_signal)
            else:
                # Some APs come through with no signal line (rare);
                # still count them as one occupant.
                by_freq[cur_freq].append(-100.0)
        cur_freq = None
        cur_signal = None

    for raw in text.splitlines():
        line = raw.rstrip()
        if line.startswith("BSS "):
            flush()
            continue
        stripped = line.strip()
        if stripped.startswith("freq:"):
            try:
                cur_freq = int(stripped.split(":", 1)[1].strip())
            except (ValueError, IndexError):
                cur_freq = None
        elif stripped.startswith("signal:"):
            m = re.search(r"-?\d+(?:\.\d+)?", stripped)
            if m:
                try:
                    cur_signal = float(m.group(0))
                except ValueError:
                    cur_signal = None
    flush()
    return by_freq


def _list_supported_frequencies(iface: str) -> List[int]:
    """Pull the list of frequencies this radio actually supports + has
    enabled in the current regulatory domain, from `iw phy phyN info`.
    Frequencies marked "(disabled)" are skipped — they're either out
    of regulatory bounds or hardware-blocked. We render only what the
    operator can actually pick."""
    # iw dev <iface> info tells us which phy this interface lives on;
    # iw phy <phyN> info enumerates frequencies.
    info = _run(["iw", "dev", iface, "info"])
    phy = "phy0"
    if info:
        m = re.search(r"wiphy\s+(\d+)", info)
        if m:
            phy = f"phy{m.group(1)}"
    phy_info = _run(["iw", "phy", phy, "info"])
    if not phy_info:
        # Fall back to the common 2.4 GHz channels 1-11 so the modal
        # isn't empty when iw is broken in some unexpected way.
        return [2412 + 5 * i for i in range(11)]

    freqs: List[int] = []
    for line in phy_info.splitlines():
        m = re.search(r"\*\s+(\d+)\s+MHz\s+\[\d+\]", line)
        if m and "disabled" not in line:
            freqs.append(int(m.group(1)))
    return freqs


def survey() -> Dict:
    """Scan once, return per-channel summary for the modal.

    `iw dev <iface> scan` pauses beacons briefly (typically <5 s on
    brcmfmac) and may cause one missed beacon for clients — usually
    not enough to drop them but worth noting. The operator is going
    to disconnect everyone anyway as soon as they apply, so the cost
    is acceptable.

    Returns:
        {
            "ok": bool,
            "iface": str | None,
            "current_channel": int | None,
            "channels": [ChannelInfo.to_dict(), ...],
            "error": str | None,
        }
    """
    iface = detect_wifi_interface()
    if not iface:
        return {"ok": False, "iface": None, "current_channel": None,
                "channels": [], "error": "No wireless interface found"}

    # iw scan needs root to put the interface into scan mode while
    # it's in AP mode. The service user has NOPASSWD sudo (install.sh
    # writes the sudoers rule for it).
    #
    # log_failures=True so the "see server log" hint in the
    # operator-facing error is actually backed by a log entry. Common
    # cause on the Pi 4: brcmfmac firmware doesn't allow scan while
    # the radio is operating as an AP — the journal line will look
    # like "command failed: Device or resource busy (-16)".
    scan_out = _run(
        ["sudo", "-n", "iw", "dev", iface, "scan"],
        timeout=10.0,
        log_failures=True,
    )
    if scan_out is None:
        return {"ok": False, "iface": iface, "current_channel": None,
                "channels": [], "error":
                    f"iw scan on {iface} failed — see server log"}

    by_freq = _parse_scan(scan_out)
    supported = _list_supported_frequencies(iface)
    current_ch = _get_current_channel(iface)

    rows: List[ChannelInfo] = []
    for freq in supported:
        mapped = _freq_to_channel(freq)
        if mapped is None:
            continue
        band, channel = mapped
        signals = by_freq.get(freq, [])
        info = ChannelInfo(
            band=band,
            channel=channel,
            freq_mhz=freq,
            ap_count=len(signals),
            strongest_signal_dbm=max(signals) if signals else None,
            is_dfs=(band == "5" and channel in _DFS_5GHZ_CHANNELS),
            is_current=(channel == current_ch),
        )
        rows.append(info)

    # Sort: current channel first (so the operator's reference point
    # is obvious), then by ap_count ascending (least-busy first),
    # then by strongest signal ascending (quieter is better).
    rows.sort(key=lambda r: (
        not r.is_current,                          # current → top
        r.ap_count,
        (r.strongest_signal_dbm if r.strongest_signal_dbm is not None else -200),
        r.band,                                    # 2.4 before 5 on ties
        r.channel,
    ))

    return {
        "ok": True,
        "iface": iface,
        "current_channel": current_ch,
        "channels": [r.to_dict() for r in rows],
        "error": None,
    }


def get_credentials() -> Dict:
    """Read the current AP SSID, password, band, and channel from NM.

    NM is the single source of truth — no YAML mirror. That avoids the
    drift problem where install.sh + dashboard + manual nmcli edits
    would each write to a different store. The password is returned
    in cleartext when readable (nmcli -s --show-secrets); the UI is
    responsible for masking it on display.
    """
    conn = AP_CONNECTION_NAME

    def get_field(name: str, show_secret: bool = False) -> Optional[str]:
        cmd = ["sudo", "-n", "nmcli"]
        if show_secret:
            cmd.append("-s")
        cmd += ["-g", name, "connection", "show", conn]
        out = _run(cmd, timeout=3.0)
        return out.strip() if out is not None else None

    ssid = get_field("802-11-wireless.ssid")
    band = get_field("802-11-wireless.band")
    # nmcli emits "--" for unset channel (auto/ACS), not 0
    channel_raw = get_field("802-11-wireless.channel")
    password = get_field("802-11-wireless-security.psk", show_secret=True)
    iface = get_field("connection.interface-name")

    try:
        channel = int(channel_raw) if channel_raw and channel_raw != "--" else 0
    except ValueError:
        channel = 0

    return {
        "ok": ssid is not None,
        "ssid": ssid,
        "password": password,
        "band": band,
        "channel": channel,
        "iface": iface,
    }


# WPA2-PSK valid range per the standard. Below 8 or above 63 chars
# nmcli will reject the connection bring-up; we mirror the check here
# so the UI can give a clean error instead of a cryptic nmcli failure.
def _validate_credentials(ssid: str, password: str) -> Optional[str]:
    if not isinstance(ssid, str) or not (1 <= len(ssid) <= 32):
        return "SSID must be 1-32 characters"
    # Restrict to printable ASCII so nmcli quoting can't be confused
    # by control chars / shell metacharacters. The WiFi spec technically
    # allows broader Unicode SSIDs but client compatibility is patchy
    # and the operator surely doesn't need them here.
    if not all(0x20 <= ord(c) < 0x7F for c in ssid):
        return "SSID must contain only printable ASCII characters"
    if not isinstance(password, str) or not (8 <= len(password) <= 63):
        return "Password must be 8-63 characters (WPA2-PSK requirement)"
    if not all(0x20 <= ord(c) < 0x7F for c in password):
        return "Password must contain only printable ASCII characters"
    return None


def set_credentials(ssid: str, password: str) -> Dict:
    """Update the AP's SSID + password. Restarts the AP — same outage
    cost as a channel switch. Caller is responsible for the same
    background-task / disconnect-warning UX as apply_channel."""
    err = _validate_credentials(ssid, password)
    if err:
        return {"ok": False, "error": err}

    cmd_mod = [
        "sudo", "-n", "nmcli", "connection", "modify", AP_CONNECTION_NAME,
        "802-11-wireless.ssid", ssid,
        "wifi-sec.key-mgmt", "wpa-psk",
        "wifi-sec.psk", password,
    ]
    proc = subprocess.run(cmd_mod, capture_output=True, text=True, timeout=10)
    if proc.returncode != 0:
        return {"ok": False, "error":
                f"nmcli modify failed: {proc.stderr.strip() or proc.stdout.strip()}"}

    cmd_up = ["sudo", "-n", "nmcli", "connection", "up", AP_CONNECTION_NAME]
    proc = subprocess.run(cmd_up, capture_output=True, text=True, timeout=60)
    if proc.returncode != 0:
        return {"ok": False, "error":
                f"nmcli up failed: {proc.stderr.strip() or proc.stdout.strip()}"}

    return {"ok": True, "error": None}


def apply_channel(band: str, channel: int) -> Dict:
    """Switch the AP to (band, channel). Synchronous — returns after
    nmcli completes. Callers that need the AP to come back up before
    proceeding should run this from a background task; the WebSocket
    handler does that so its response reaches the client BEFORE the
    AP actually goes down.

    Band must be 'bg' (2.4 GHz) or 'a' (5 GHz) — NetworkManager's
    spelling. Caller is responsible for translating from the UI's
    "2.4"/"5" labels.
    """
    if band not in ("bg", "a"):
        return {"ok": False, "error": f"Invalid band {band!r} (expected 'bg' or 'a')"}
    if not isinstance(channel, int) or channel < 1 or channel > 200:
        return {"ok": False, "error": f"Invalid channel {channel!r}"}

    cmd_mod = [
        "sudo", "-n", "nmcli", "connection", "modify", AP_CONNECTION_NAME,
        "802-11-wireless.band", band,
        "802-11-wireless.channel", str(channel),
    ]
    proc = subprocess.run(cmd_mod, capture_output=True, text=True, timeout=10)
    if proc.returncode != 0:
        return {"ok": False, "error":
                f"nmcli modify failed: {proc.stderr.strip() or proc.stdout.strip()}"}

    # Bring the connection up. This restarts hostapd, kicks all
    # clients off briefly, and rebinds with the new channel. Returns
    # only after the AP is back active (or fails). 30 s is generous —
    # typical bring-up is 5–10 s; DFS channels can take longer due to
    # the CAC silence period.
    cmd_up = ["sudo", "-n", "nmcli", "connection", "up", AP_CONNECTION_NAME]
    proc = subprocess.run(cmd_up, capture_output=True, text=True, timeout=60)
    if proc.returncode != 0:
        return {"ok": False, "error":
                f"nmcli up failed: {proc.stderr.strip() or proc.stdout.strip()}"}

    return {"ok": True, "error": None}
