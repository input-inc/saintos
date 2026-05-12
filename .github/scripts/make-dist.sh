#!/usr/bin/env bash
#
# Assemble a SAINT.OS server distribution tarball.
#
# Expects (relative to repo root) that `colcon build --merge-install` has
# already produced ./install/, and that node firmware artifacts have been
# staged under saint_os/resources/firmware/{rp2040,teensy41,rpi5}/.
#
# Usage: make-dist.sh <version> <arch> <ros_distro>
# Output: dist/saint-os_<version>_<arch>_<ros_distro>.tar.gz
#

set -euo pipefail

VERSION="${1:?version required}"
ARCH="${2:?arch required}"
ROS_DISTRO="${3:?ros distro required}"

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$REPO_ROOT"

if [[ ! -d install ]]; then
    echo "ERROR: ./install not found — run colcon build first" >&2
    exit 1
fi

PKG_NAME="saint-os_${VERSION}_${ARCH}_${ROS_DISTRO}"
STAGING="$(mktemp -d)"
PKG_DIR="${STAGING}/${PKG_NAME}"
mkdir -p "${PKG_DIR}"

trap 'rm -rf "${STAGING}"' EXIT

echo "Staging payload to ${PKG_DIR}..."

# Colcon install tree — this carries Python code, message bindings, configs,
# launch files, web/, and the ROS2 setup.bash for the package.
cp -a install "${PKG_DIR}/ros_install"

# Bundled node firmware for the OTA endpoint.
if [[ -d saint_os/resources/firmware ]]; then
    mkdir -p "${PKG_DIR}/firmware"
    cp -a saint_os/resources/firmware/. "${PKG_DIR}/firmware/"
fi

# Installer + systemd unit template.
cp .github/dist/install.sh "${PKG_DIR}/install.sh"
chmod +x "${PKG_DIR}/install.sh"
mkdir -p "${PKG_DIR}/systemd"
cp .github/dist/saint-os.service "${PKG_DIR}/systemd/saint-os.service"

# Manifest. Checksums are computed over the staged tree (deterministic-ish).
GIT_SHA="${GITHUB_SHA:-$(git rev-parse HEAD 2>/dev/null || echo unknown)}"
BUILT_AT="$(date -u +%Y-%m-%dT%H:%M:%SZ)"

python3 - "${PKG_DIR}" "${VERSION}" "${ARCH}" "${ROS_DISTRO}" "${GIT_SHA}" "${BUILT_AT}" <<'PY'
import hashlib, json, os, sys

pkg_dir, version, arch, ros, git_sha, built_at = sys.argv[1:7]

def sha256(path):
    h = hashlib.sha256()
    with open(path, 'rb') as f:
        for chunk in iter(lambda: f.read(65536), b''):
            h.update(chunk)
    return h.hexdigest()

firmware = {}
fw_root = os.path.join(pkg_dir, 'firmware')
if os.path.isdir(fw_root):
    for kind in sorted(os.listdir(fw_root)):
        kdir = os.path.join(fw_root, kind)
        if not os.path.isdir(kdir):
            continue
        files = []
        for name in sorted(os.listdir(kdir)):
            fp = os.path.join(kdir, name)
            if os.path.isfile(fp):
                files.append({
                    'filename': name,
                    'size': os.path.getsize(fp),
                    'sha256': sha256(fp),
                })
        if files:
            firmware[kind] = files

manifest = {
    'name': 'saint-os',
    'version': version,
    'arch': arch,
    'ros_distro': ros,
    'git_sha': git_sha,
    'built_at': built_at,
    'firmware': firmware,
}

with open(os.path.join(pkg_dir, 'manifest.json'), 'w') as f:
    json.dump(manifest, f, indent=2)
    f.write('\n')
PY

# Create tarball.
mkdir -p dist
TARBALL="${REPO_ROOT}/dist/${PKG_NAME}.tar.gz"
( cd "${STAGING}" && tar czf "${TARBALL}" "${PKG_NAME}" )

SIZE=$(stat -f%z "${TARBALL}" 2>/dev/null || stat -c%s "${TARBALL}")
SHA=$(shasum -a 256 "${TARBALL}" | cut -d' ' -f1)

echo ""
echo "Built: ${TARBALL}"
echo "  size:   ${SIZE} bytes"
echo "  sha256: ${SHA}"
