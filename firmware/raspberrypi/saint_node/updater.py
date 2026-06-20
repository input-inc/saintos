#!/usr/bin/env python3
"""
SAINT.OS Raspberry Pi Firmware Updater

Handles over-the-air firmware updates for Raspberry Pi nodes (Pi 3 /
Pi 4 / Pi 5). The firmware package is model-independent, so the same
update artifact reaches every adopted Pi regardless of generation.
The update process:
1. Receive update notification via ROS2 control topic
2. Download update package from server via HTTP
3. Validate package checksum
4. Extract to staging directory
5. Replace current firmware files
6. Restart the service
"""

import hashlib
import json
import os
import shutil
import subprocess
import tarfile
import tempfile
import time
import urllib.request
import zipfile
from pathlib import Path
from typing import Optional, Callable, Dict, Any

from . import __version__


class FirmwareUpdater:
    """Handles firmware self-updates for Raspberry Pi nodes."""

    # Where the firmware is installed. MUST match what scripts/install.sh
    # used (/opt/saint-node/saint_node/) — that path is the symlink
    # target in Python's site-packages, so anything else means OTA
    # writes to a path Python never imports from. Before today's fix
    # the updater used /opt/saint/firmware/raspberrypi/ which the
    # symlink didn't point at, so updates appeared to succeed but the
    # running code never changed.
    DEFAULT_INSTALL_DIR = Path("/opt/saint-node")

    # Backup directory for rollback. Sibling of the install dir so the
    # rollback path is a simple swap.
    BACKUP_DIR = Path("/opt/saint-node.backup")

    # Staging directory for new firmware
    STAGING_DIR = Path("/tmp/saint_firmware_staging")

    # Service name for restart
    SERVICE_NAME = "saint-node.service"

    def __init__(self, logger=None, install_dir: Optional[Path] = None):
        """
        Initialize the firmware updater.

        Args:
            logger: ROS2 logger instance (optional)
            install_dir: Override default install directory
        """
        self._logger = logger
        self._install_dir = install_dir or self.DEFAULT_INSTALL_DIR
        self._current_version = __version__
        self._update_in_progress = False
        self._progress_callback: Optional[Callable[[str, int], None]] = None
        # Terminal-result callback (success/failure + message). Needed
        # because restart_service() kills this process before
        # perform_update can return on the success path — so we publish
        # the "complete" result through here, just before the restart,
        # rather than relying on a return value the caller never sees.
        self._result_callback: Optional[Callable[[bool, str], None]] = None
        # Last (stage, percent) tuple we reported. Used to suppress
        # per-block duplicate emissions during the download (urllib's
        # urlretrieve calls the progress hook every 8 KB; on a 350 MB
        # transfer that's ~43k invocations, most of which carry the
        # same integer percent value as the prior call. Without
        # de-duplication we saturate the /update_progress topic
        # — DDS BEST_EFFORT depth=10 silently drops 99 % of frames
        # and the UI's progress bar appears frozen).
        self._last_progress_key: tuple = (None, None)
        # Most recent error from a step (download/extract/validate/install).
        # perform_update reads this so the operator-facing error message
        # includes the specific reason ("tar --zstd extract failed:
        # cannot open archive") instead of a generic "Extraction failed".
        # Without it, /log + the OTA progress bar only carry the stage
        # name; the actual error sits in the local journal where the
        # operator can't see it.
        self._last_error: Optional[str] = None

    def set_progress_callback(self, callback: Callable[[str, int], None]):
        """Set callback for progress updates: callback(stage, percent)."""
        self._progress_callback = callback

    def set_result_callback(self, callback: Callable[[bool, str], None]):
        """Set callback for the terminal result: callback(success, message)."""
        self._result_callback = callback

    def _report_result(self, success: bool, message: str):
        """Emit the terminal update result, if a callback is wired."""
        if self._result_callback:
            try:
                self._result_callback(success, message)
            except Exception as e:
                self._log("warn", f"Result callback failed: {e}")

    def _log(self, level: str, msg: str):
        """Log a message.

        Each severity MUST be dispatched from its own distinct source
        line. rclpy's RcutilsLogger memoizes severity per call-site
        (file+function+line); routing every level through a single
        ``getattr(self._logger, level)(msg)`` line makes them all share
        one call-site, so the first INFO call locks that line to INFO
        and the next ERROR call raises
        ``ValueError: Logger severity cannot be changed between calls.``
        That exception used to fire from inside the updater's own
        error-reporting path during an OTA — masking the real failure,
        killing the update thread, and leaving the node running the old
        code with no restart and no operator-visible error. Keep the
        branches separate.
        """
        if not self._logger:
            print(f"[{level.upper()}] {msg}")
            return
        if level == "error":
            self._logger.error(msg)
        elif level in ("warn", "warning"):
            self._logger.warning(msg)
        elif level == "debug":
            self._logger.debug(msg)
        else:
            self._logger.info(msg)

    def _report_progress(self, stage: str, percent: int):
        """Report progress to callback if set.

        De-duplicated against the last (stage, percent) tuple we
        reported. The progress hook in download_update is called per
        8-KB block (urllib's default), so on a 350 MB download we'd
        otherwise fire ~43,000 callbacks — most carrying the same
        integer-percent value as the prior call. The /update_progress
        ROS topic is BEST_EFFORT with depth=10; at that emission rate
        the subscriber drops nearly everything and the UI looks
        frozen until the download finishes. Filtering to one event per
        true (stage, percent) change reduces it to ~100 events per
        stage — which DDS delivers reliably and the UI renders smoothly.
        """
        key = (stage, percent)
        if key == self._last_progress_key:
            return
        self._last_progress_key = key
        if self._progress_callback:
            self._progress_callback(stage, percent)
        self._log("info", f"Update progress: {stage} ({percent}%)")

    @property
    def current_version(self) -> str:
        """Get current firmware version."""
        return self._current_version

    def check_update_available(self, new_version: str) -> bool:
        """
        Check if an update is available.

        Args:
            new_version: Version string to compare against

        Returns:
            True if new_version is newer than current
        """
        try:
            current_parts = [int(x) for x in self._current_version.split('.')]
            new_parts = [int(x) for x in new_version.split('.')]

            # Pad to same length
            while len(current_parts) < len(new_parts):
                current_parts.append(0)
            while len(new_parts) < len(current_parts):
                new_parts.append(0)

            return new_parts > current_parts
        except ValueError:
            # If parsing fails, allow update
            return True

    def download_update(self, url: str, expected_checksum: Optional[str] = None) -> Optional[Path]:
        """
        Download update package from URL.

        Args:
            url: URL to download from
            expected_checksum: SHA256 checksum to verify (optional)

        Returns:
            Path to downloaded file, or None on failure
        """
        self._report_progress("downloading", 0)

        try:
            # Create staging directory
            self.STAGING_DIR.mkdir(parents=True, exist_ok=True)

            # Determine filename from URL. Accept every format
            # extract_update() knows how to handle. The 'firmware_update'
            # default with .tar.zst extension matches what package.sh
            # emits today — older servers serving .zip / .tar.gz still
            # work because extract_update() dispatches on the suffix
            # rather than assuming one format.
            filename = url.split('/')[-1]
            if not filename.endswith(('.zip', '.tar.gz', '.tgz', '.tar.zst', '.tzst')):
                filename = "firmware_update.tar.zst"

            download_path = self.STAGING_DIR / filename

            # Stream the download in chunks so we can report real
            # progress. urlretrieve's reporthook only yields a usable
            # percent when the server sends Content-Length; a chunked or
            # gzip-encoded response leaves total_size <= 0 and the bar
            # never moves (the reported symptom). We request identity
            # encoding to preserve Content-Length, read the header
            # ourselves, and fall back to a byte counter when it's
            # genuinely absent so the operator always sees activity.
            req = urllib.request.Request(
                url, headers={"Accept-Encoding": "identity"})
            with urllib.request.urlopen(req, timeout=30) as resp, \
                    open(download_path, "wb") as out:
                try:
                    total = int(resp.headers.get("Content-Length") or 0)
                except (TypeError, ValueError):
                    total = 0
                read = 0
                last = -1
                while True:
                    chunk = resp.read(64 * 1024)
                    if not chunk:
                        break
                    out.write(chunk)
                    read += len(chunk)
                    if total > 0:
                        pct = min(100, int(read * 100 / total))
                        if pct != last:
                            last = pct
                            self._report_progress("downloading", pct)
                    else:
                        # No Content-Length — report MB downloaded so the
                        # log still shows movement (percent stays 0).
                        mb = read // (1024 * 1024)
                        if mb != last:
                            last = mb
                            self._log("info",
                                      f"Downloading firmware: {mb} MB")

            self._report_progress("downloading", 100)

            # Verify checksum if provided
            if expected_checksum:
                self._report_progress("verifying", 0)
                actual_checksum = self._calculate_checksum(download_path)

                if actual_checksum != expected_checksum:
                    self._log("error", f"Checksum mismatch: expected {expected_checksum}, got {actual_checksum}")
                    download_path.unlink()
                    return None

                self._report_progress("verifying", 100)
                self._log("info", "Checksum verified")

            return download_path

        except Exception as e:
            self._log("error", f"Download failed: {e}")
            return None

    def _calculate_checksum(self, file_path: Path) -> str:
        """Calculate SHA256 checksum of a file."""
        sha256 = hashlib.sha256()
        with open(file_path, 'rb') as f:
            for chunk in iter(lambda: f.read(8192), b''):
                sha256.update(chunk)
        return sha256.hexdigest()

    def extract_update(self, package_path: Path) -> Optional[Path]:
        """
        Extract update package to staging directory.

        Args:
            package_path: Path to downloaded package

        Returns:
            Path to extracted directory, or None on failure
        """
        self._report_progress("extracting", 0)

        try:
            extract_dir = self.STAGING_DIR / "extracted"
            if extract_dir.exists():
                shutil.rmtree(extract_dir)
            extract_dir.mkdir(parents=True)

            # Extract based on file type. Three formats accepted so
            # an old server pushing a .zip to a new Pi (or vice versa)
            # still works.
            #
            # .tar.zst — current packager output. Python's stdlib
            # tarfile didn't get native zstd support until 3.14;
            # 3.13 (Trixie's default) needs an external decompressor.
            # We shell out to `tar --zstd` rather than pip-install
            # zstandard so the OTA path doesn't depend on a python
            # package that the install bundle's deb cache may not
            # carry. `tar` ships with libzstd on Bookworm / Trixie
            # so --zstd works without a separate zstd binary, but if
            # libzstd isn't linked in tar falls back to spawning the
            # `zstd` CLI which IS in our bundle-debs RUNTIME_DEB_LIST.
            if package_path.suffix == '.zip':
                with zipfile.ZipFile(package_path, 'r') as zf:
                    zf.extractall(extract_dir)
            elif package_path.name.endswith(('.tar.gz', '.tgz')):
                with tarfile.open(package_path, 'r:gz') as tf:
                    tf.extractall(extract_dir)
            elif package_path.name.endswith(('.tar.zst', '.tzst')):
                import subprocess
                proc = subprocess.run(
                    ['tar', '--zstd', '-xf', str(package_path),
                     '-C', str(extract_dir)],
                    capture_output=True, text=True)
                if proc.returncode != 0:
                    err = (f"tar --zstd extract failed (rc={proc.returncode}): "
                           f"{proc.stderr.strip()[:200]}")
                    self._log("error", err)
                    self._last_error = err
                    return None
            else:
                err = f"Unsupported package format: {package_path.suffix}"
                self._log("error", err)
                self._last_error = err
                return None

            self._report_progress("extracting", 100)

            # Locate the directory that CONTAINS a saint_node/ subpackage —
            # that's what install_update copies to the install dir and what
            # validate_update walks with rel paths like "saint_node/node.py".
            # Two archive shapes ship today:
            #   (a) saint_node/...                       (zip --no-prefix)
            #   (b) saint_firmware_raspberrypi_<v>/saint_node/...
            #       (the layout package.sh emits)
            # Walk extract_dir + one level of subdirs looking for the
            # first match. Previous logic returned the saint_node dir
            # itself in case (a), which silently broke validate.
            candidates = [extract_dir]
            for child in extract_dir.iterdir():
                if child.is_dir():
                    candidates.append(child)
            for parent in candidates:
                init_file = parent / "saint_node" / "__init__.py"
                if init_file.exists():
                    return parent

            err = ("Could not find saint_node/ package inside the extracted "
                   "bundle — package layout looks wrong")
            self._log("error", err)
            self._last_error = err
            return None

        except Exception as e:
            err = f"Extraction failed: {e}"
            self._log("error", err)
            self._last_error = err
            return None

    def validate_update(self, extracted_dir: Path) -> bool:
        """
        Validate extracted update package.

        Args:
            extracted_dir: Path to extracted firmware

        Returns:
            True if valid
        """
        self._report_progress("validating", 0)

        required_files = [
            "saint_node/__init__.py",
            "saint_node/node.py",
            "saint_node/gpio_control.py",
            "saint_node/config.py",
        ]

        for i, rel_path in enumerate(required_files):
            file_path = extracted_dir / rel_path
            if not file_path.exists():
                err = f"Missing required file in bundle: {rel_path}"
                self._log("error", err)
                self._last_error = err
                return False
            percent = int((i + 1) / len(required_files) * 100)
            self._report_progress("validating", percent)

        # Check version in new firmware
        try:
            init_path = extracted_dir / "saint_node" / "__init__.py"
            with open(init_path, 'r') as f:
                content = f.read()
                if '__version__' not in content:
                    self._log("warn", "New firmware missing version string")
        except Exception as e:
            self._log("warn", f"Could not verify version: {e}")

        self._report_progress("validating", 100)
        return True

    def create_backup(self) -> bool:
        """
        Create backup of the current ``saint_node/`` package for rollback.

        Only the swappable bit (the Python package) gets backed up —
        the same scope install_update() rewrites. If the OTA fails,
        rollback() restores this one directory and the service is
        good as new. Backing up /opt/saint-node/ in full was wasteful
        in the legacy code (would copy GBs of mistakenly-placed
        ros2_install/, deps/, etc.) AND pointless: those files don't
        change across OTAs.

        Returns:
            True if successful
        """
        self._report_progress("backup", 0)

        try:
            src = self._install_dir / "saint_node"
            if not src.exists():
                self._log("info", "No existing saint_node/ to backup")
                return True

            # Remove old backup
            if self.BACKUP_DIR.exists():
                shutil.rmtree(self.BACKUP_DIR)
            self.BACKUP_DIR.mkdir(parents=True)

            # symlinks=True copies any symlink AS a symlink rather than
            # recursing into its target. Without it a self-referential
            # link (e.g. the saint_node/saint_node -> saint_node loop a
            # buggy `ln -sf` in install.sh used to leave behind) sends
            # copytree into infinite recursion until ELOOP — which is
            # exactly the "Too many levels of symbolic links" failure
            # that bricked the OTA at the backup stage.
            shutil.copytree(src, self.BACKUP_DIR / "saint_node", symlinks=True)

            self._report_progress("backup", 100)
            self._log("info", f"Backup created at {self.BACKUP_DIR}")
            return True

        except Exception as e:
            err = f"Backup failed: {e}"
            self._log("error", err)
            self._last_error = err
            return False

    def install_update(self, extracted_dir: Path) -> bool:
        """
        Install extracted firmware to install directory.

        Only the ``saint_node/`` Python package is swapped. The bundled
        ``ros2_install/`` (~275 MB), ``deps/`` (~295 MB), ``scripts/``,
        ``config/``, etc. are install-time-only artifacts the operator
        already laid down via scripts/install.sh — there's no reason
        to re-copy them on an OTA, AND copying them is actively
        harmful: a Pi 5 SD card at ~10 MB/s would spend 60+ s blocking
        on the copytree() call here, with no progress emission in the
        meantime, so the operator-side bar appears frozen and they
        give up before the install finishes. Restricting to
        saint_node/ drops the install phase to <1 s of file ops.

        Args:
            extracted_dir: Path to extracted firmware bundle root
                (contains saint_node/, scripts/, deps/, etc.)

        Returns:
            True if successful
        """
        self._report_progress("installing", 0)

        try:
            src_saint_node = extracted_dir / "saint_node"
            if not src_saint_node.is_dir():
                err = f"Installation source missing saint_node/: {src_saint_node}"
                self._log("error", err)
                self._last_error = err
                return False

            dst_saint_node = self._install_dir / "saint_node"

            # Ensure /opt/saint-node exists; scripts/install.sh created
            # it on first install, but a bare-cold OTA shouldn't depend
            # on that having happened.
            self._install_dir.mkdir(parents=True, exist_ok=True)

            self._report_progress("installing", 30)

            # Atomic swap: drop the old saint_node/ subdir, copy the
            # new one in. Python's site-packages symlink (set up by
            # install.sh) points at /opt/saint-node/saint_node and is
            # untouched, so on next service start the new code is
            # imported via the same path.
            # rmtree unlinks symlinks without following them, so this
            # also clears any stale self-referential link the old
            # install.sh `ln -sf` bug may have left under the install
            # dir. copytree(symlinks=True) keeps us loop-safe even if a
            # future bundle ever ships a symlink.
            if dst_saint_node.exists():
                shutil.rmtree(dst_saint_node)
            shutil.copytree(src_saint_node, dst_saint_node, symlinks=True)

            self._report_progress("installing", 80)

            # Set permissions on the swapped tree only — no point
            # rglob'ing /opt/saint-node/ wholesale, which would now
            # also walk anything operators dropped under there.
            for py_file in dst_saint_node.rglob("*.py"):
                py_file.chmod(0o644)

            self._report_progress("installing", 100)
            self._log("info", f"Firmware installed to {dst_saint_node}")
            return True

        except Exception as e:
            err = f"Installation failed: {e}"
            self._log("error", err)
            self._last_error = err
            return False

    def rollback(self) -> bool:
        """
        Rollback the saint_node/ package to the pre-OTA backup.

        Scoped to match install_update() / create_backup() — only
        the swappable Python package is restored. The rest of
        /opt/saint-node/ wasn't touched in the first place.

        Returns:
            True if successful
        """
        self._log("warn", "Rolling back to previous firmware")

        try:
            backup_pkg = self.BACKUP_DIR / "saint_node"
            if not backup_pkg.exists():
                self._log("error",
                    f"No backup available for rollback at {backup_pkg}")
                return False

            dst = self._install_dir / "saint_node"
            if dst.exists():
                shutil.rmtree(dst)
            shutil.copytree(backup_pkg, dst, symlinks=True)

            self._log("info", "Rollback complete")
            return True

        except Exception as e:
            self._log("error", f"Rollback failed: {e}")
            return False

    def cleanup(self):
        """Clean up staging directory."""
        try:
            if self.STAGING_DIR.exists():
                shutil.rmtree(self.STAGING_DIR)
        except Exception as e:
            self._log("warn", f"Cleanup failed: {e}")

    def restart_service(self) -> bool:
        """
        Restart the saint-node service.

        Returns:
            True if successful
        """
        self._report_progress("restarting", 0)

        try:
            # Check if running as systemd service
            result = subprocess.run(
                ["systemctl", "is-active", self.SERVICE_NAME],
                capture_output=True,
                text=True
            )

            if result.returncode == 0:
                # Service is running, restart it. saint-node.service
                # runs as User=root so no `sudo` prefix is needed (and
                # minimal images may not ship sudo at all). --no-block
                # so we don't wait on the systemd reply that will never
                # come — restart kills this very process before reply.
                self._log("info", f"Restarting {self.SERVICE_NAME}")
                subprocess.run(
                    ["systemctl", "restart", "--no-block", self.SERVICE_NAME],
                    check=True
                )
                self._report_progress("restarting", 100)
                return True
            else:
                # Not running as service, just exit (supervisor will restart)
                self._log("info", "Not running as service, exiting for restart")
                self._report_progress("restarting", 100)
                os._exit(0)

        except subprocess.CalledProcessError as e:
            self._log("error", f"Service restart failed: {e}")
            return False
        except Exception as e:
            self._log("error", f"Restart failed: {e}")
            return False

    def perform_update(self, url: str, checksum: Optional[str] = None,
                       new_version: Optional[str] = None) -> Dict[str, Any]:
        """
        Perform complete firmware update process.

        Args:
            url: URL to download update package from
            checksum: Expected SHA256 checksum (optional)
            new_version: New version string for logging (optional)

        Returns:
            Dict with 'success' bool and 'message' string
        """
        if self._update_in_progress:
            return {"success": False, "message": "Update already in progress"}

        self._update_in_progress = True

        # Reset for this run — otherwise a stale error from a previous
        # failed attempt would bleed into the next result message.
        self._last_error = None

        try:
            self._log("info", f"Starting firmware update from {url}")
            if new_version:
                self._log("info", f"Updating from {self._current_version} to {new_version}")

            def _err(default: str) -> str:
                return self._last_error or default

            # Download
            package_path = self.download_update(url, checksum)
            if not package_path:
                return {"success": False, "message": _err("Download failed")}

            # Extract
            extracted_dir = self.extract_update(package_path)
            if not extracted_dir:
                return {"success": False, "message": _err("Extraction failed")}

            # Validate
            if not self.validate_update(extracted_dir):
                return {"success": False, "message": _err("Validation failed")}

            # Backup current
            if not self.create_backup():
                return {"success": False, "message": _err("Backup failed")}

            # Install
            if not self.install_update(extracted_dir):
                self.rollback()
                return {"success": False,
                        "message": _err("Installation failed") + " (rolled back)"}

            # Cleanup
            self.cleanup()

            # Publish the success result NOW. restart_service() kills
            # this process before perform_update returns, so this is the
            # only chance to tell the operator the update applied —
            # otherwise the UI hangs on the last progress frame ("Backing
            # up…") forever. Then sleep briefly so the RELIABLE
            # /update_progress + /log frames (this result, plus the
            # installing/restarting frames just queued) actually transmit
            # before systemctl pulls the rug.
            done_msg = (f"Updated to {new_version}" if new_version
                        else "Update complete")
            self._log("info", f"Firmware update successful: {done_msg}; restarting…")
            self._report_progress("restarting", 100)
            self._report_result(True, done_msg)
            time.sleep(2)

            # Restart (this does not return on success)
            self.restart_service()

            return {"success": True, "message": "Update complete, restarting"}

        except Exception as e:
            self._log("error", f"Update failed: {e}")
            self.rollback()
            return {"success": False, "message": f"Update failed: {e}"}

        finally:
            self._update_in_progress = False
