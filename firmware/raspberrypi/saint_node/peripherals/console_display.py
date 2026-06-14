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

# Filesystem locations under $HOME. Kept here so tests can monkey-patch
# them without touching the operator's real config dir.
KIOSK_DIR_REL       = ".config/saint/console-kiosk"
LAUNCH_SCRIPT_REL   = KIOSK_DIR_REL + "/launch.sh"
URL_STATE_REL       = KIOSK_DIR_REL + "/url"
AUTOSTART_REL       = ".config/autostart/saint-console-kiosk.desktop"


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
            self._log("warn",
                f"console_display#{instance_id}: missing required params; "
                f"skipping kiosk file write")
            inst.connected = False
            return False

        try:
            home = Path(os.path.expanduser("~"))
            kiosk_dir = home / KIOSK_DIR_REL
            kiosk_dir.mkdir(parents=True, exist_ok=True)

            launch_path = home / LAUNCH_SCRIPT_REL
            launch_path.write_text(self._render_launch_script(url, inst.params))
            launch_path.chmod(launch_path.stat().st_mode | stat.S_IEXEC | stat.S_IXGRP | stat.S_IXOTH)

            url_path = home / URL_STATE_REL
            url_path.write_text(url + "\n")

            autostart_path = home / AUTOSTART_REL
            if inst.params.get("autostart", True):
                autostart_path.parent.mkdir(parents=True, exist_ok=True)
                autostart_path.write_text(self._render_autostart_desktop(launch_path))
            else:
                # If autostart was previously enabled but the operator
                # turned it off, remove the entry so the kiosk doesn't
                # silently keep coming back.
                if autostart_path.exists():
                    try: autostart_path.unlink()
                    except OSError: pass

            self._log("info",
                f"console_display#{instance_id}: wrote {launch_path} "
                f"(autostart={'on' if inst.params.get('autostart', True) else 'off'})")
        except OSError as e:
            self._log("error",
                f"console_display#{instance_id}: failed to write kiosk "
                f"files: {e}")
            inst.connected = False
            return False

        # Relaunch any already-running Chromium kiosk so the operator
        # doesn't have to log out / log back in to pick up a new URL
        # or rotation. Best-effort; failure here doesn't roll back the
        # filesystem write.
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

    def _render_launch_script(self, url: str, params: Dict[str, Any]) -> str:
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
        return (
            "#!/usr/bin/env bash\n"
            "# AUTO-GENERATED by saint_node console_display driver — do not edit by hand.\n"
            "# Re-applies on every peripheral sync from the SAINT.OS server.\n"
            "set -u\n"
            f"URL={shlex.quote(url)}\n"
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
        """Kill any running Chromium kiosk owned by this user, then
        respawn via the new launch script. No-op if no Chromium is
        running (operator hasn't logged into the desktop yet — the
        autostart entry will handle it on next login).
        """
        if not shutil.which("pgrep"):
            return
        try:
            res = subprocess.run(
                ["pgrep", "-u", str(os.getuid()), "-f", "--kiosk"],
                check=False, capture_output=True, text=True, timeout=3)
        except (OSError, subprocess.SubprocessError):
            return
        pids = [int(p) for p in res.stdout.split() if p.isdigit()]
        for pid in pids:
            try: os.kill(pid, signal.SIGTERM)
            except OSError: pass
        if not pids:
            # Operator may not be logged into the desktop yet — that's
            # the normal first-boot case. Autostart entry handles it.
            return
        # Give Chromium a moment to tear down, then respawn detached.
        try:
            subprocess.Popen(
                ["bash", str(launch_path)],
                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                start_new_session=True)
        except OSError as e:
            self._log("warn", f"console_display: respawn failed: {e}")


# ── Pure helpers (testable without filesystem) ────────────────────

def build_kiosk_url(params: Dict[str, Any]) -> Optional[str]:
    """Construct the kiosk URL the Pi browser should load.

    Returns None if any required field is missing — the driver
    surfaces a warn and skips the write. Otherwise produces something
    like:

        http://saint.local:8080/#/console/battery/controller-1/bms-1?kiosk_token=...

    The token is sourced from the private `_kiosk_token` param the
    server's state_manager injects during the config push. Operator
    UI never shows it.
    """
    server_url = (params.get("server_url") or "").strip().rstrip("/")
    view = (params.get("view") or "").strip()
    node_id = (params.get("target_node_id") or "").strip()
    peripheral_id = (params.get("target_peripheral_id") or "").strip()
    if not server_url or not view or not node_id or not peripheral_id:
        return None
    path = f"#/console/{view}/{quote(node_id, safe='')}/{quote(peripheral_id, safe='')}"
    token = params.get("_kiosk_token")
    query = ""
    if token:
        query = "?kiosk_token=" + quote(str(token), safe="")
    return f"{server_url}/{path}{query}"
