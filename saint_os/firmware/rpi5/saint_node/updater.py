#!/usr/bin/env python3
"""
SAINT.OS Raspberry Pi 5 Firmware Updater

Handles over-the-air firmware updates for Pi 5 nodes.
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
import urllib.request
import zipfile
from pathlib import Path
from typing import Optional, Callable, Dict, Any

from . import __version__


class FirmwareUpdater:
    """Handles firmware self-updates for Pi 5 nodes."""

    # Where the firmware is installed
    DEFAULT_INSTALL_DIR = Path("/opt/saint/firmware/rpi5")

    # Backup directory for rollback
    BACKUP_DIR = Path("/opt/saint/firmware/rpi5.backup")

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

    def set_progress_callback(self, callback: Callable[[str, int], None]):
        """Set callback for progress updates: callback(stage, percent)."""
        self._progress_callback = callback

    def _log(self, level: str, msg: str):
        """Log a message."""
        if self._logger:
            getattr(self._logger, level)(msg)
        else:
            print(f"[{level.upper()}] {msg}")

    def _report_progress(self, stage: str, percent: int):
        """Report progress to callback if set."""
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

            # Determine filename from URL
            filename = url.split('/')[-1]
            if not filename.endswith(('.zip', '.tar.gz', '.tgz')):
                filename = "firmware_update.zip"

            download_path = self.STAGING_DIR / filename

            # Download with progress
            def progress_hook(block_num, block_size, total_size):
                if total_size > 0:
                    percent = min(100, int(block_num * block_size * 100 / total_size))
                    self._report_progress("downloading", percent)

            urllib.request.urlretrieve(url, download_path, progress_hook)

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

            # Extract based on file type
            if package_path.suffix == '.zip':
                with zipfile.ZipFile(package_path, 'r') as zf:
                    zf.extractall(extract_dir)
            elif package_path.name.endswith(('.tar.gz', '.tgz')):
                with tarfile.open(package_path, 'r:gz') as tf:
                    tf.extractall(extract_dir)
            else:
                self._log("error", f"Unsupported package format: {package_path.suffix}")
                return None

            self._report_progress("extracting", 100)

            # Find the saint_node directory
            # It might be at root or in a subdirectory
            for candidate in [extract_dir / "saint_node", extract_dir]:
                if (candidate / "__init__.py").exists():
                    return candidate

            # Check one level deep
            for subdir in extract_dir.iterdir():
                if subdir.is_dir():
                    candidate = subdir / "saint_node"
                    if candidate.exists() and (candidate / "__init__.py").exists():
                        return subdir
                    if (subdir / "__init__.py").exists():
                        return subdir.parent

            self._log("error", "Could not find saint_node package in update")
            return None

        except Exception as e:
            self._log("error", f"Extraction failed: {e}")
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
                self._log("error", f"Missing required file: {rel_path}")
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
        Create backup of current firmware for rollback.

        Returns:
            True if successful
        """
        self._report_progress("backup", 0)

        try:
            if not self._install_dir.exists():
                self._log("info", "No existing installation to backup")
                return True

            # Remove old backup
            if self.BACKUP_DIR.exists():
                shutil.rmtree(self.BACKUP_DIR)

            # Copy current installation
            shutil.copytree(self._install_dir, self.BACKUP_DIR)

            self._report_progress("backup", 100)
            self._log("info", f"Backup created at {self.BACKUP_DIR}")
            return True

        except Exception as e:
            self._log("error", f"Backup failed: {e}")
            return False

    def install_update(self, extracted_dir: Path) -> bool:
        """
        Install extracted firmware to install directory.

        Args:
            extracted_dir: Path to extracted firmware

        Returns:
            True if successful
        """
        self._report_progress("installing", 0)

        try:
            # Ensure install directory parent exists
            self._install_dir.parent.mkdir(parents=True, exist_ok=True)

            # Remove current installation
            if self._install_dir.exists():
                shutil.rmtree(self._install_dir)

            self._report_progress("installing", 30)

            # Copy new firmware
            shutil.copytree(extracted_dir, self._install_dir)

            self._report_progress("installing", 80)

            # Set permissions
            for py_file in self._install_dir.rglob("*.py"):
                py_file.chmod(0o644)

            # Make scripts executable
            scripts_dir = self._install_dir / "scripts"
            if scripts_dir.exists():
                for script in scripts_dir.iterdir():
                    script.chmod(0o755)

            self._report_progress("installing", 100)
            self._log("info", f"Firmware installed to {self._install_dir}")
            return True

        except Exception as e:
            self._log("error", f"Installation failed: {e}")
            return False

    def rollback(self) -> bool:
        """
        Rollback to previous firmware version.

        Returns:
            True if successful
        """
        self._log("warn", "Rolling back to previous firmware")

        try:
            if not self.BACKUP_DIR.exists():
                self._log("error", "No backup available for rollback")
                return False

            # Remove failed installation
            if self._install_dir.exists():
                shutil.rmtree(self._install_dir)

            # Restore backup
            shutil.copytree(self.BACKUP_DIR, self._install_dir)

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
                # Service is running, restart it
                self._log("info", f"Restarting {self.SERVICE_NAME}")
                subprocess.run(
                    ["sudo", "systemctl", "restart", self.SERVICE_NAME],
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

        try:
            self._log("info", f"Starting firmware update from {url}")
            if new_version:
                self._log("info", f"Updating from {self._current_version} to {new_version}")

            # Download
            package_path = self.download_update(url, checksum)
            if not package_path:
                return {"success": False, "message": "Download failed"}

            # Extract
            extracted_dir = self.extract_update(package_path)
            if not extracted_dir:
                return {"success": False, "message": "Extraction failed"}

            # Validate
            if not self.validate_update(extracted_dir):
                return {"success": False, "message": "Validation failed"}

            # Backup current
            if not self.create_backup():
                return {"success": False, "message": "Backup failed"}

            # Install
            if not self.install_update(extracted_dir):
                self.rollback()
                return {"success": False, "message": "Installation failed, rolled back"}

            # Cleanup
            self.cleanup()

            self._log("info", "Firmware update successful, restarting...")

            # Restart (this may not return)
            self.restart_service()

            return {"success": True, "message": "Update complete, restarting"}

        except Exception as e:
            self._log("error", f"Update failed: {e}")
            self.rollback()
            return {"success": False, "message": f"Update failed: {e}"}

        finally:
            self._update_in_progress = False
