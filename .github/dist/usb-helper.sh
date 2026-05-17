#!/usr/bin/env bash
#
# SAINT.OS USB helper — runs as root via sudoers from the saint user.
#
# udisks2 auto-mounts removable media at /media/<user>/<label>/ with
# mode 0700, so the saint user (running the web server) can't traverse
# into it. This helper does the privileged work the update manager
# needs: enumerate tarballs on mounted USB media, and copy a chosen
# tarball into the staging directory.
#
# Subcommands:
#   scan
#       Print "<size_bytes>\t<absolute_path>" per discovered tarball
#       matching saint-os_*_*.tar.gz on any mounted removable filesystem
#       under /media, /mnt, or /run/media. Searches up to 4 directory
#       levels deep so e.g. /media/<user>/<label>/<file> is reachable.
#
#   stage <tarball_path>
#       Copy a tarball into /var/lib/saint-os/updates/ owned by the
#       saint user. Prints the destination path on stdout.
#
# Keep this script small — the sudoers rule allows any invocation, so
# the surface area is the privilege boundary.

set -euo pipefail

STAGING_DIR=/var/lib/saint-os/updates
SERVICE_USER=saint

# Filesystem types we treat as "removable media." System mounts (ext4 on
# /, tmpfs, proc, etc.) are excluded so we don't waste time scanning
# unrelated trees.
USB_FSTYPES_RE='^(vfat|exfat|exfat-fuse|ntfs|ntfs3|fuseblk|ext2|ext3|ext4|hfs|hfsplus)$'

# Mount-point roots that signal "this was auto-mounted as user media."
USB_TARGET_RE='^(/media/|/mnt/|/run/media/)'

scan() {
    # findmnt with -rn = raw, no header. -o picks the columns we need.
    # We deliberately don't restrict to removable: ext4 USB sticks won't
    # set REMOVABLE properly via every mounter. Filter by mount point
    # prefix + fstype instead.
    findmnt -rn -o TARGET,FSTYPE | while IFS=$' \t' read -r target fstype rest; do
        [[ -z "$target" || -z "$fstype" ]] && continue
        [[ "$fstype" =~ $USB_FSTYPES_RE ]] || continue
        [[ "$target" =~ $USB_TARGET_RE ]] || continue

        # Up to 4 levels of nesting catches /media/<user>/<label>/<file>
        # and /run/media/<user>/<label>/updates/<file>.
        find "$target" -maxdepth 4 -type f \
            -name 'saint-os_*_*.tar.gz' \
            -printf '%s\t%p\n' 2>/dev/null
    done
}

stage() {
    local src="$1"
    [[ -f "$src" ]] || { echo "source not found: $src" >&2; exit 2; }

    # Constrain inputs to the mount roots we scan from. Prevents the
    # saint user from passing arbitrary file paths through sudo.
    case "$(readlink -f "$src")" in
        /media/*|/mnt/*|/run/media/*) ;;
        *) echo "source not under /media, /mnt, or /run/media: $src" >&2; exit 2 ;;
    esac

    install -d -m 0755 -o "$SERVICE_USER" -g "$SERVICE_USER" "$STAGING_DIR"
    local dest="${STAGING_DIR}/$(basename "$src")"
    install -m 0644 -o "$SERVICE_USER" -g "$SERVICE_USER" "$src" "$dest"
    echo "$dest"
}

case "${1:-}" in
    scan)  scan ;;
    stage) shift; stage "${1:?stage requires a path}" ;;
    *)
        echo "usage: usb-helper.sh {scan|stage <path>}" >&2
        exit 2
        ;;
esac
