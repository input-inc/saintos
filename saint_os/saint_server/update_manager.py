"""SAINT.OS update manager.

Handles three concerns:

1. GitHub release polling. On startup and on demand, queries the public
   releases endpoint and compares the latest tag to the installed VERSION.
2. USB tarball scanning. Walks /media/* and /mnt/* for valid dist tarballs
   that are newer than the installed version.
3. Apply path. Downloads (or copies, for USB) a tarball into the staging
   directory and invokes the privileged apply-update.sh wrapper.

State is exposed as a dataclass that the WebSocket handler serializes to
clients. Callers can subscribe to state changes via add_listener().
"""

from __future__ import annotations

import asyncio
import json
import logging
import os
import re
import shutil
import subprocess
import time
import urllib.error
import urllib.request
from dataclasses import asdict, dataclass, field
from pathlib import Path
from typing import Callable, List, Optional

# packaging.version is installed by the dist tarball's apt deps
# (python3-packaging). Import lazily so unit tests on dev machines don't
# require it just to import this module.
try:
    from packaging.version import InvalidVersion, Version
except ImportError:  # pragma: no cover - dev fallback
    Version = None
    InvalidVersion = Exception

GITHUB_REPO = os.environ.get("SAINT_UPDATE_REPO", "input-inc/saintos")
GITHUB_API_TIMEOUT_S = 5.0
USB_SCAN_ROOTS = ("/media", "/mnt")
INSTALL_PREFIX = Path(os.environ.get("SAINT_INSTALL_PREFIX", "/opt/saint-os"))
UPDATE_STAGING = Path(os.environ.get("SAINT_UPDATE_STAGING", "/var/lib/saint-os/updates"))
APPLY_WRAPPER = INSTALL_PREFIX / "bin" / "apply-update.sh"

# Tarballs published by the dist workflow look like:
#   saint-os_0.5.1_arm64_jazzy.tar.gz
#   saint-os_v0.5.1_arm64_jazzy.tar.gz
TARBALL_RE = re.compile(
    r"^saint-os_v?(?P<version>[0-9]+\.[0-9]+\.[0-9]+(?:[-+][^_]+)?)"
    r"_(?P<arch>[a-z0-9]+)_(?P<ros>[a-z0-9]+)\.tar\.gz$"
)


@dataclass
class ReleaseInfo:
    """A newer release we've discovered (from GitHub or USB)."""
    version: str
    source: str  # 'github' or 'usb'
    asset_url: Optional[str] = None      # download URL (github only)
    asset_size: Optional[int] = None     # bytes
    asset_name: Optional[str] = None     # tarball filename
    html_url: Optional[str] = None       # release page (github only)
    notes: Optional[str] = None          # release body / changelog (github only)
    local_path: Optional[str] = None     # filesystem path (usb only)
    published_at: Optional[str] = None


@dataclass
class UpdateState:
    """Snapshot of the updater for the UI."""
    status: str = "unknown"
    # status values:
    #   unknown          - no check has run yet
    #   checking         - GitHub check in flight
    #   up_to_date       - latest checked, no newer version
    #   available        - newer version on GitHub
    #   no_network       - couldn't reach GitHub
    #   downloading      - pulling tarball
    #   downloaded       - tarball staged, ready to install
    #   installing       - apply-update.sh launched
    #   error            - last operation failed

    installed_version: str = "unknown"
    last_check_at: Optional[float] = None
    last_error: Optional[str] = None

    # When status == 'available' / 'downloaded' / 'installing', this holds
    # the candidate release info.
    github_release: Optional[ReleaseInfo] = None

    # USB scan results. Always populated independently of status.
    usb_releases: List[ReleaseInfo] = field(default_factory=list)
    last_usb_scan_at: Optional[float] = None

    # Download progress (when status == 'downloading').
    download_total: Optional[int] = None
    download_received: int = 0

    # Staged tarball ready for install (set in 'downloaded' state).
    staged_tarball: Optional[str] = None

    def to_dict(self):
        d = asdict(self)
        # asdict serializes dataclasses recursively; nothing else to do.
        return d


def _read_installed_version() -> str:
    """Best-effort read of the currently installed version."""
    version_file = INSTALL_PREFIX / "VERSION"
    if version_file.is_file():
        try:
            return version_file.read_text().strip() or "unknown"
        except OSError:
            pass

    # Fallback: parse manifest.json the installer wrote next to PREFIX.
    manifest = INSTALL_PREFIX / "manifest.json"
    if manifest.is_file():
        try:
            return json.loads(manifest.read_text()).get("version", "unknown")
        except (OSError, json.JSONDecodeError):
            pass

    return "unknown"


def _version_newer(candidate: str, installed: str) -> bool:
    """Return True if `candidate` is strictly newer than `installed`.

    Falls back to string comparison when packaging isn't importable.
    Treats 'unknown' as oldest (anything is newer).
    """
    if installed == "unknown":
        return True
    if Version is None:  # pragma: no cover - dev fallback
        return candidate != installed
    try:
        return Version(candidate) > Version(installed)
    except InvalidVersion:
        return False


class UpdateManager:
    """Owns the update lifecycle. Single instance per server."""

    def __init__(self, logger: Optional[logging.Logger] = None):
        self.logger = logger or logging.getLogger(__name__)
        self.state = UpdateState(installed_version=_read_installed_version())
        self._listeners: List[Callable[[UpdateState], None]] = []
        self._lock = asyncio.Lock()
        # Reusable HTTP request for the API check; GitHub recommends UA.
        self._api_headers = {
            "Accept": "application/vnd.github+json",
            "User-Agent": f"saint-os-update-manager/{self.state.installed_version}",
        }

    # ── Listener fanout ──────────────────────────────────────────────

    def add_listener(self, fn: Callable[[UpdateState], None]):
        self._listeners.append(fn)

    def _notify(self):
        for fn in self._listeners:
            try:
                fn(self.state)
            except Exception:
                self.logger.exception("update listener raised")

    def _set_status(self, status: str, **changes):
        self.state.status = status
        for k, v in changes.items():
            setattr(self.state, k, v)
        self._notify()

    # ── GitHub check ─────────────────────────────────────────────────

    async def check_github(self) -> UpdateState:
        """Query GitHub for the latest release.

        Returns the resulting state. Non-blocking timeout of 5s — short
        enough that an offline robot's startup isn't delayed noticeably.
        """
        async with self._lock:
            self._set_status("checking")
            try:
                release = await asyncio.get_event_loop().run_in_executor(
                    None, self._fetch_latest_release
                )
            except (urllib.error.URLError, TimeoutError, OSError) as e:
                self.logger.info("GitHub release check failed: %s", e)
                self._set_status(
                    "no_network",
                    last_check_at=time.time(),
                    last_error=str(e),
                    github_release=None,
                )
                return self.state
            except Exception as e:  # noqa: BLE001
                self.logger.exception("GitHub release check raised")
                self._set_status(
                    "error",
                    last_check_at=time.time(),
                    last_error=str(e),
                )
                return self.state

            self.state.last_check_at = time.time()
            self.state.last_error = None

            if release is None:
                # No releases at all — treat as up to date.
                self._set_status("up_to_date", github_release=None)
                return self.state

            if _version_newer(release.version, self.state.installed_version):
                self._set_status("available", github_release=release)
            else:
                self._set_status("up_to_date", github_release=None)
            return self.state

    def _fetch_latest_release(self) -> Optional[ReleaseInfo]:
        """Synchronous HTTP call — run via run_in_executor."""
        url = f"https://api.github.com/repos/{GITHUB_REPO}/releases/latest"
        req = urllib.request.Request(url, headers=self._api_headers)
        with urllib.request.urlopen(req, timeout=GITHUB_API_TIMEOUT_S) as resp:
            data = json.loads(resp.read().decode("utf-8"))

        tag = data.get("tag_name", "")
        version = tag.lstrip("v")
        if not version:
            return None

        # Pick the tarball asset matching the install layout.
        asset_url = None
        asset_size = None
        asset_name = None
        for asset in data.get("assets", []):
            name = asset.get("name", "")
            if TARBALL_RE.match(name):
                asset_url = asset.get("browser_download_url")
                asset_size = asset.get("size")
                asset_name = name
                break

        return ReleaseInfo(
            version=version,
            source="github",
            asset_url=asset_url,
            asset_size=asset_size,
            asset_name=asset_name,
            html_url=data.get("html_url"),
            notes=data.get("body"),
            published_at=data.get("published_at"),
        )

    # ── USB scan ─────────────────────────────────────────────────────

    async def scan_usb(self) -> UpdateState:
        """Walk mount points for valid newer tarballs."""
        async with self._lock:
            results = await asyncio.get_event_loop().run_in_executor(
                None, self._scan_usb_blocking
            )
            self.state.usb_releases = results
            self.state.last_usb_scan_at = time.time()
            self._notify()
            return self.state

    def _scan_usb_blocking(self) -> List[ReleaseInfo]:
        found: List[ReleaseInfo] = []
        for root in USB_SCAN_ROOTS:
            if not os.path.isdir(root):
                continue
            for entry in self._walk_mounts(root):
                m = TARBALL_RE.match(os.path.basename(entry))
                if not m:
                    continue
                version = m.group("version")
                if not _version_newer(version, self.state.installed_version):
                    continue
                try:
                    size = os.path.getsize(entry)
                except OSError:
                    continue
                found.append(ReleaseInfo(
                    version=version,
                    source="usb",
                    asset_name=os.path.basename(entry),
                    asset_size=size,
                    local_path=entry,
                ))
        # De-dup by version, prefer larger files (likely complete).
        by_version: dict = {}
        for r in found:
            existing = by_version.get(r.version)
            if existing is None or (r.asset_size or 0) > (existing.asset_size or 0):
                by_version[r.version] = r
        return sorted(by_version.values(), key=lambda r: r.version, reverse=True)

    @staticmethod
    def _walk_mounts(root: str):
        """Yield tarball paths up to 3 levels deep — covers typical
        auto-mount layouts (/media/<user>/<label>/file or /mnt/<label>/file)
        without scanning huge filesystems."""
        try:
            entries = os.scandir(root)
        except OSError:
            return
        for level1 in entries:
            if not level1.is_dir(follow_symlinks=False):
                continue
            yield from UpdateManager._scan_dir(level1.path, depth=2)

    @staticmethod
    def _scan_dir(path: str, depth: int):
        if depth <= 0:
            return
        try:
            entries = os.scandir(path)
        except OSError:
            return
        for entry in entries:
            try:
                if entry.is_file(follow_symlinks=False) and entry.name.endswith(".tar.gz"):
                    yield entry.path
                elif entry.is_dir(follow_symlinks=False):
                    yield from UpdateManager._scan_dir(entry.path, depth - 1)
            except OSError:
                continue

    # ── Download ─────────────────────────────────────────────────────

    async def download_github_release(self) -> UpdateState:
        """Download the cached GitHub release's tarball into the staging dir."""
        if self.state.github_release is None or not self.state.github_release.asset_url:
            raise RuntimeError("No GitHub release available to download")

        release = self.state.github_release
        dest_dir = UPDATE_STAGING
        dest_dir.mkdir(parents=True, exist_ok=True)
        dest = dest_dir / (release.asset_name or f"saint-os_{release.version}.tar.gz")

        async with self._lock:
            self._set_status(
                "downloading",
                download_total=release.asset_size,
                download_received=0,
                staged_tarball=None,
            )

        try:
            await asyncio.get_event_loop().run_in_executor(
                None, self._download_to, release.asset_url, dest
            )
        except Exception as e:  # noqa: BLE001
            self.logger.exception("download failed")
            self._set_status("error", last_error=f"download failed: {e}")
            return self.state

        self._set_status(
            "downloaded",
            staged_tarball=str(dest),
            download_received=os.path.getsize(dest),
        )
        return self.state

    def _download_to(self, url: str, dest: Path):
        """Streaming download with progress updates."""
        req = urllib.request.Request(url, headers=self._api_headers)
        with urllib.request.urlopen(req, timeout=30) as resp:
            total = int(resp.headers.get("Content-Length", 0)) or self.state.download_total
            self.state.download_total = total
            received = 0
            tmp = dest.with_suffix(dest.suffix + ".part")
            with open(tmp, "wb") as f:
                while True:
                    chunk = resp.read(65536)
                    if not chunk:
                        break
                    f.write(chunk)
                    received += len(chunk)
                    self.state.download_received = received
                    # Notify on every ~5% of progress, or every chunk if no total.
                    if total and received % max(1, total // 20) < 65536:
                        self._notify()
            tmp.replace(dest)

    async def stage_usb_release(self, version: str) -> UpdateState:
        """Copy a USB-detected tarball into the staging directory."""
        match = next(
            (r for r in self.state.usb_releases if r.version == version),
            None,
        )
        if match is None or not match.local_path:
            raise RuntimeError(f"USB release {version} not in scan results")
        dest_dir = UPDATE_STAGING
        dest_dir.mkdir(parents=True, exist_ok=True)
        dest = dest_dir / os.path.basename(match.local_path)

        async with self._lock:
            self._set_status("downloading", download_total=match.asset_size, download_received=0)

        try:
            await asyncio.get_event_loop().run_in_executor(
                None, shutil.copy2, match.local_path, dest
            )
        except OSError as e:
            self._set_status("error", last_error=f"USB copy failed: {e}")
            return self.state

        self._set_status(
            "downloaded",
            staged_tarball=str(dest),
            download_received=os.path.getsize(dest),
            github_release=ReleaseInfo(
                version=match.version,
                source="usb",
                asset_name=match.asset_name,
                asset_size=match.asset_size,
                local_path=str(dest),
            ),
        )
        return self.state

    # ── Install ──────────────────────────────────────────────────────

    async def install_staged(self) -> UpdateState:
        """Invoke the privileged apply-update wrapper. Detaches the install
        so it survives the saint-os service restart that install.sh triggers."""
        if not self.state.staged_tarball:
            raise RuntimeError("No staged tarball to install")

        tarball = self.state.staged_tarball
        if not APPLY_WRAPPER.exists():
            self._set_status(
                "error",
                last_error=f"apply wrapper missing at {APPLY_WRAPPER}",
            )
            return self.state

        self._set_status("installing")

        try:
            # Non-blocking — apply-update.sh returns immediately after
            # detaching the install.
            await asyncio.get_event_loop().run_in_executor(
                None, self._run_apply_wrapper, tarball
            )
        except subprocess.CalledProcessError as e:
            self._set_status(
                "error",
                last_error=f"apply-update.sh failed (rc={e.returncode}): {e.stderr or e.stdout}",
            )
            return self.state

        # Status stays at 'installing' — the WebSocket will disconnect as
        # the server shuts down for the restart. Reconnect will pick up
        # the new VERSION on next boot.
        return self.state

    @staticmethod
    def _run_apply_wrapper(tarball: str):
        subprocess.run(
            ["sudo", "-n", str(APPLY_WRAPPER), tarball],
            check=True,
            capture_output=True,
            text=True,
            timeout=30,
        )
