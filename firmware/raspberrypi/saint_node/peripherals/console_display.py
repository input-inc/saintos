"""
SAINT.OS Pi node — console_display driver.

Configures the Pi as an HDMI kiosk that loads a Console view URL from
the SAINT.OS server in Chromium fullscreen. No I/O channels — this is
a config-only peripheral whose effect is entirely filesystem-side:

  - writes a launcher shell script (~/.config/saint/console-kiosk/launch.sh)
  - writes an XDG autostart entry (~/.config/autostart/saint-console-kiosk.desktop)
    if params.autostart is True
  - signals any running Chromium kiosk to relaunch with the new URL

The kiosk URL is built from operator params (server_url + view + target
node/peripheral) plus the server's kiosk_token (injected by
state_manager.get_firmware_config_json under the private `_kiosk_token`
param key). The token is what lets the browser session bypass the
operator-password gate without prompting.

This driver intentionally does NOT install Chromium, X/Wayland, or any
session manager — it assumes the Pi was provisioned with a desktop
environment already. The launch script is callable on its own
(`bash ~/.config/saint/console-kiosk/launch.sh`) so a Pi running pure
headless without autostart can still be brought up manually.
"""

from __future__ import annotations

import os
import shlex
import shutil
import signal
import stat
import subprocess
from pathlib import Path
from typing import Any, Dict, Optional
from urllib.parse import quote

from .base import PeripheralDriver


# Virtual GPIO range. console_display has no logical I/O channels but
# the manager's overlap detection still needs a unique slot — pick a
# range well clear of the other Pi-side peripherals (audio_player at
# 400, system_monitor at 600-ish in the firmware-side mapping).
CONSOLE_DISPLAY_VIRTUAL_GPIO_BASE = 500
CONSOLE_DISPLAY_CHANNEL_COUNT     = 0    # truly no channels

# System-wide filesystem locations. The saint-node service runs as
# root (see firmware/raspberrypi/config/saint-node.service), but the
# desktop auto-login session that launches Chromium runs as the
# regular Pi user (pi / hackman / etc.). Writing kiosk files to
# /root/.config/* (the natural `~` expansion under root) is invisible
# to that session — autostart never fires. Use /etc/xdg/autostart/
# (XDG-spec system-wide autostart) and /etc/saint-node/console-kiosk/
# so any desktop user picks them up at session start.
#
# The constants are read by tests via monkey-patching, so keep the
# names stable.
KIOSK_DIR           = "/etc/saint-node/console-kiosk"
LAUNCH_SCRIPT_PATH  = KIOSK_DIR + "/launch.sh"
URL_STATE_PATH      = KIOSK_DIR + "/url"
AUTOSTART_PATH      = "/etc/xdg/autostart/saint-console-kiosk.desktop"


class ConsoleDisplayDriver(PeripheralDriver):
    """Pi-side console_display driver. See module docstring."""

    TYPE_ID = "console_display"
    MODE_STRING = "console_display"
    VIRTUAL_GPIO_BASE = CONSOLE_DISPLAY_VIRTUAL_GPIO_BASE
    # The base class's overlap check uses total_channel_count() which
    # multiplies these. A 0 channel-count peripheral claims an empty
    # range — fine for overlap detection.
    CHANNELS_PER_INSTANCE = max(1, CONSOLE_DISPLAY_CHANNEL_COUNT)  # see _ranges note
    MAX_INSTANCES = 1
    SUB_CHANNEL_NAMES: list = []

    # ── PeripheralDriver overrides ────────────────────────────────

    def apply_config(self, instance_id: int, pins: Dict[str, int],
                     params: Dict[str, Any]) -> bool:
        inst = self._get_or_create_instance(instance_id)
        inst.pins = dict(pins or {})
        inst.params = dict(params or {})

        url = build_kiosk_url(inst.params)
        if url is None:
            # build_kiosk_url only returns None for the per-pack
            # "battery" view when target_* params are missing. The
            # default "batteries" overview view requires no targeting,
            # so a fresh adoption with defaults still produces a URL.
            self._log("warn",
                f"console_display#{instance_id}: per-pack 'battery' "
                f"view selected but target_node_id/target_peripheral_id "
                f"are empty — skipping kiosk file write")
            inst.connected = False
            return False

        try:
            launch_path = Path(LAUNCH_SCRIPT_PATH)
            url_path    = Path(URL_STATE_PATH)
            autostart_path = Path(AUTOSTART_PATH)

            launch_path.parent.mkdir(parents=True, exist_ok=True)
            launch_path.write_text(self._render_launch_script(inst.params))
            # World-readable + executable so the desktop user's session
            # (running as pi / hackman / whoever auto-logs in) can run
            # it from /etc/xdg/autostart's Exec= line.
            launch_path.chmod(0o755)

            url_path.write_text(url + "\n")
            # World-readable so the launch script (running as the
            # desktop user) can read the URL written by the saint-node
            # service (running as root).
            url_path.chmod(0o644)

            if inst.params.get("autostart", True):
                autostart_path.parent.mkdir(parents=True, exist_ok=True)
                autostart_path.write_text(self._render_autostart_desktop(launch_path))
                autostart_path.chmod(0o644)
            else:
                # If autostart was previously enabled but the operator
                # turned it off, remove the entry so the kiosk doesn't
                # silently keep coming back on next desktop login.
                if autostart_path.exists():
                    try: autostart_path.unlink()
                    except OSError: pass

            self._log("info",
                f"console_display#{instance_id}: wrote {launch_path} "
                f"+ {autostart_path if inst.params.get('autostart', True) else '(autostart off)'} "
                f"URL={url}")
        except OSError as e:
            self._log("error",
                f"console_display#{instance_id}: failed to write kiosk "
                f"files: {e}")
            inst.connected = False
            return False

        # Relaunch any already-running Chromium kiosk so the operator
        # doesn't have to log out / log back in to pick up a new URL
        # or rotation. Best-effort across users; if Chromium isn't
        # running yet (typical first-adoption case), the autostart
        # entry handles it on next desktop login.
        self._relaunch_kiosk(launch_path)

        inst.connected = True
        return True

    def set_value(self, instance_id: int, sub_channel: int,
                  value: float) -> bool:
        # No channels — there is nothing to drive at runtime.
        return False

    def get_value(self, instance_id: int, sub_channel: int) -> Optional[float]:
        return None

    # ── Rendering helpers ─────────────────────────────────────────

    def _render_launch_script(self, params: Dict[str, Any]) -> str:
        rotation = int(params.get("rotation", 0) or 0)
        # Chromium flag list — kiosk-mode best practice for an HDMI
        # appliance. --kiosk hides chrome, --noerrdialogs suppresses
        # the "Restore session?" bubble after a crash, --disable-pinch
        # blocks unwanted zoom on touch displays, --incognito stops
        # local profile bloat on every relaunch.
        flags = " ".join([
            "--kiosk",
            "--noerrdialogs",
            "--disable-infobars",
            "--disable-pinch",
            "--overscroll-history-navigation=0",
            "--no-first-run",
            "--incognito",
            "--check-for-update-interval=31536000",
        ])
        # Try chromium-browser (Raspberry Pi OS), then chromium
        # (Debian-flavored fallback), then chrome. The script picks
        # the first one present at launch time.
        rotation_block = ""
        if rotation in (90, 180, 270):
            xrandr_dir = {90: "right", 180: "inverted", 270: "left"}[rotation]
            rotation_block = (
                "# Apply HDMI rotation, best-effort: xrandr first (X), "
                "wlr-randr second (Wayland).\n"
                f'xrandr --output "$(xrandr | awk \'/ connected/ {{print $1; exit}}\')" --rotate {xrandr_dir} 2>/dev/null || true\n'
                f'wlr-randr --output "$(wlr-randr | awk \'/^[^ ]/ {{print $1; exit}}\')" --transform {rotation} 2>/dev/null || true\n'
            )
        # Read URL from the state file at launch time (NOT baked into
        # the script). This lets the saint-node service update the
        # URL by rewriting the state file alone — no script rewrite,
        # no rights drama between root and the desktop user.
        url_file_q = shlex.quote(URL_STATE_PATH)
        return (
            "#!/usr/bin/env bash\n"
            "# AUTO-GENERATED by saint_node console_display driver — do not edit by hand.\n"
            "# Re-applies on every peripheral sync from the SAINT.OS server.\n"
            "set -u\n"
            f"URL=$(cat {url_file_q} 2>/dev/null | head -1)\n"
            "if [ -z \"$URL\" ]; then\n"
            f"    echo \"saint-console-kiosk: no URL in {URL_STATE_PATH}\" >&2\n"
            "    exit 1\n"
            "fi\n"
            f"{rotation_block}"
            "for B in chromium-browser chromium google-chrome chrome; do\n"
            "    if command -v \"$B\" >/dev/null 2>&1; then\n"
            f"        exec \"$B\" {flags} \"$URL\"\n"
            "    fi\n"
            "done\n"
            "echo 'saint-console-kiosk: no Chromium binary found on PATH' >&2\n"
            "exit 1\n"
        )

    def _render_autostart_desktop(self, launch_path: Path) -> str:
        return (
            "[Desktop Entry]\n"
            "Type=Application\n"
            "Name=SAINT.OS Console Kiosk\n"
            "Comment=Auto-launched by the console_display peripheral.\n"
            f"Exec=bash {shlex.quote(str(launch_path))}\n"
            "X-GNOME-Autostart-enabled=true\n"
            "Terminal=false\n"
        )

    # ── Hot relaunch ──────────────────────────────────────────────

    def _relaunch_kiosk(self, launch_path: Path) -> None:
        """Best-effort: kill any running Chromium kiosk so the desktop
        user's session can respawn it with the new URL. No-op if no
        Chromium is running yet (typical first-adoption case — the
        autostart entry handles it on next desktop session start).

        We don't try to start Chromium ourselves: the saint-node service
        runs as root with no X/Wayland display, while Chromium needs the
        desktop user's DISPLAY/DBUS context. The kill-only path lets
        whatever already-running session manager (gdm, lightdm,
        labwc, etc.) respawn from /etc/xdg/autostart on its own
        cadence. If the session never auto-respawns, the user can
        log out + back in once to pick up the new URL.
        """
        if not shutil.which("pgrep"):
            return
        # Search ALL users — saint-node runs as root but Chromium runs
        # under the desktop user. No -u filter; just match anything
        # with --kiosk in its argv.
        try:
            res = subprocess.run(
                ["pgrep", "-f", "--kiosk"],
                check=False, capture_output=True, text=True, timeout=3)
        except (OSError, subprocess.SubprocessError):
            return
        pids = [int(p) for p in res.stdout.split() if p.isdigit()]
        for pid in pids:
            try: os.kill(pid, signal.SIGTERM)
            except OSError: pass
        if pids:
            self._log("info",
                f"console_display: signalled {len(pids)} existing kiosk "
                f"chromium process(es); session will respawn via autostart")


# ── Pure helpers (testable without filesystem) ────────────────────

def build_kiosk_url(params: Dict[str, Any]) -> Optional[str]:
    """Construct the kiosk URL the Pi browser should load.

    Returns None if a required field is missing for the chosen view.
    For the default "batteries" overview only `server_url` is needed;
    the view auto-discovers all BMS peripherals from the server.
    For the per-pack "battery" view both `target_node_id` and
    `target_peripheral_id` are required (otherwise the URL can't
    route to a specific pack).

    Examples:
        http://saint.local:8080/#/console/batteries?kiosk_token=...
        http://saint.local:8080/#/console/battery/controller-1/bms-1?kiosk_token=...

    The token is sourced from the private `_kiosk_token` param the
    server's state_manager injects during the config push. Operator
    UI never shows it.
    """
    server_url = (params.get("server_url") or "").strip().rstrip("/")
    if not server_url:
        return None
    # Default view: the multi-pack overview. The operator can opt into
    # the per-pack detail view by setting view="battery" + the target
    # IDs, but for "just show me batteries" — the common case — the
    # overview handles it without operator config.
    view = (params.get("view") or "batteries").strip()

    if view == "battery":
        node_id = (params.get("target_node_id") or "").strip()
        peripheral_id = (params.get("target_peripheral_id") or "").strip()
        if not node_id or not peripheral_id:
            return None
        hash_part = f"#/console/battery/{quote(node_id, safe='')}/{quote(peripheral_id, safe='')}"
    else:
        # batteries overview — no per-pack targeting. The Vue view
        # discovers adopted BMSes via the management WS.
        hash_part = "#/console/batteries"

    # Place ?kiosk_token=... BEFORE the # fragment so it lands in
    # window.location.search (which is everything between ? and #).
    # If we put it after the #, the token ends up inside the hash and
    # main.js's URLSearchParams(window.location.search) returns null,
    # and the kiosk falls through to the LoginScreen. Vue Router's
    # in-hash query parsing handles in-hash queries independently of
    # this outer query, so the two coexist without conflict.
    token = params.get("_kiosk_token")
    query = ""
    if token:
        query = "?kiosk_token=" + quote(str(token), safe="")
    return f"{server_url}/{query}{hash_part}"
