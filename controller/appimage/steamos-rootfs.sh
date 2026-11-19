#!/usr/bin/env bash
# Turn Valve's SteamOS recovery image into a Docker image of SteamOS's
# root filesystem, for steamos-smoke-test.sh.
#
# Usage: steamos-rootfs.sh <steamdeck-*-repair-*.img[.bz2]> <out.tar>
#        steamos-rootfs.sh --resolve        # print the latest image URL
#
# The recovery image is a full disk image (GPT: esp, efi-A/B, rootfs-A/B,
# var-A/B, home). We pull the rootfs-A partition out and copy its files
# with `btrfs restore`, which reads the filesystem without mounting it —
# no loop devices or privileged containers, so it runs the same on CI and
# on a Mac's Docker. The result is a tarball for `docker import`.
set -euo pipefail

LATEST_URL="https://steamdeck-images.steamos.cloud/recovery/steamdeck-repair-latest.img.bz2"

if [[ "${1:-}" == "--resolve" ]]; then
    # The "latest" link redirects to a versioned file name, e.g.
    # steamdeck-oobe-repair-20260707.10-3.8.14.img.bz2 — that name is the
    # cache key.
    curl -fsIL -o /dev/null -w '%{url_effective}\n' "$LATEST_URL"
    exit 0
fi

IMG="${1:?usage: $0 <recovery.img[.bz2]> <out.tar>}"
OUT="${2:?usage: $0 <recovery.img[.bz2]> <out.tar>}"
IMG_DIR=$(cd "$(dirname "$IMG")" && pwd)
OUT_DIR=$(mkdir -p "$(dirname "$OUT")" && cd "$(dirname "$OUT")" && pwd)

# Everything runs in a Debian container as root, so the restored files
# keep SteamOS's ownership and modes (and the host needs only Docker).
docker run --rm -i --platform linux/amd64 \
    -v "$IMG_DIR:/in" -v "$OUT_DIR:/out" \
    -e IMG_NAME="$(basename "$IMG")" -e OUT_NAME="$(basename "$OUT")" \
    debian:trixie-slim bash -euo pipefail <<'CONTAINER'
export DEBIAN_FRONTEND=noninteractive
apt-get update -qq >/dev/null
apt-get install -y -qq --no-install-recommends btrfs-progs fdisk lbzip2 jq >/dev/null

cd /tmp
if [[ "$IMG_NAME" == *.bz2 ]]; then
    echo "==> Decompressing $IMG_NAME"
    lbzip2 -dc "/in/$IMG_NAME" > disk.img
else
    ln -s "/in/$IMG_NAME" disk.img
fi

echo "==> Partitions:"
sfdisk -J disk.img | jq -r '.partitiontable.partitions[] | "    \(.name // "-")  start=\(.start) size=\(.size)"'
read -r START SIZE < <(sfdisk -J disk.img \
    | jq -r '.partitiontable.partitions[] | select(.name == "rootfs-A") | "\(.start) \(.size)"')
[[ -n "${START:-}" ]] || { echo "ERROR: no rootfs-A partition in $IMG_NAME" >&2; exit 1; }

echo "==> Extracting rootfs-A ($(( SIZE * 512 / 1048576 )) MiB)"
dd if=disk.img of=rootfs.btrfs bs=4M iflag=skip_bytes,count_bytes \
    skip=$(( START * 512 )) count=$(( SIZE * 512 )) status=none
rm -f disk.img
mkdir rootfs
# -m owner/mode/times, -S symlinks, -i ignore errors on
# individual files rather than abort the whole restore. No -x: the only
# xattrs are btrfs.compression, which other filesystems reject loudly.
btrfs restore -m -S -i rootfs.btrfs rootfs/ >/dev/null
rm -f rootfs.btrfs

[[ -f rootfs/etc/os-release ]] || { echo "ERROR: restored tree has no /etc/os-release" >&2; exit 1; }
. rootfs/etc/os-release
echo "==> Restored ${PRETTY_NAME:-SteamOS} ${VERSION_ID:-} (build ${BUILD_ID:-?}), $(du -sh rootfs | cut -f1)"
tar -C rootfs --numeric-owner -cf "/out/$OUT_NAME" .
CONTAINER
echo "==> Wrote $OUT"
