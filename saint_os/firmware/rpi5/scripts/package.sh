#!/bin/bash
#
# Package Pi 5 firmware for distribution
#
# Creates a zip file that can be served by the SAINT.OS server
# for over-the-air updates to Pi 5 nodes.
#
# Usage: ./package.sh [version]
#

set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
FIRMWARE_DIR="$(dirname "$SCRIPT_DIR")"
OUTPUT_DIR="${FIRMWARE_DIR}/dist"

# Get version from argument or __init__.py
if [ -n "$1" ]; then
    VERSION="$1"
else
    VERSION=$(grep '__version__' "${FIRMWARE_DIR}/saint_node/__init__.py" | sed 's/.*"\([^"]*\)".*/\1/')
fi

if [ -z "$VERSION" ]; then
    VERSION="0.0.0"
fi

echo "Packaging Pi 5 firmware version ${VERSION}"

# Create output directory
mkdir -p "${OUTPUT_DIR}"

# Create temporary directory for packaging
TEMP_DIR=$(mktemp -d)
PACKAGE_DIR="${TEMP_DIR}/saint_firmware_rpi5_${VERSION}"
mkdir -p "${PACKAGE_DIR}"

# Copy firmware files
echo "Copying firmware files..."
cp -r "${FIRMWARE_DIR}/saint_node" "${PACKAGE_DIR}/"
cp -r "${FIRMWARE_DIR}/config" "${PACKAGE_DIR}/" 2>/dev/null || true
cp -r "${FIRMWARE_DIR}/scripts" "${PACKAGE_DIR}/" 2>/dev/null || true

# Remove __pycache__ directories
find "${PACKAGE_DIR}" -type d -name "__pycache__" -exec rm -rf {} + 2>/dev/null || true
find "${PACKAGE_DIR}" -name "*.pyc" -delete 2>/dev/null || true

# Create package info
cat > "${PACKAGE_DIR}/package_info.json" << EOF
{
    "type": "rpi5",
    "version": "${VERSION}",
    "created": "$(date -u +%Y-%m-%dT%H:%M:%SZ)",
    "files": [
        "saint_node/__init__.py",
        "saint_node/node.py",
        "saint_node/gpio_control.py",
        "saint_node/config.py",
        "saint_node/updater.py"
    ]
}
EOF

# Create zip package
PACKAGE_NAME="saint_firmware_rpi5_${VERSION}.zip"
PACKAGE_PATH="${OUTPUT_DIR}/${PACKAGE_NAME}"

echo "Creating package: ${PACKAGE_NAME}"
cd "${TEMP_DIR}"
zip -r "${PACKAGE_PATH}" "saint_firmware_rpi5_${VERSION}"

# Calculate checksum
CHECKSUM=$(shasum -a 256 "${PACKAGE_PATH}" | cut -d' ' -f1)

# Create info.json for server
cat > "${OUTPUT_DIR}/info.json" << EOF
{
    "type": "rpi5",
    "latest_version": "${VERSION}",
    "latest_package": "${PACKAGE_NAME}",
    "latest_checksum": "${CHECKSUM}",
    "updated": "$(date -u +%Y-%m-%dT%H:%M:%SZ)",
    "packages": [
        {
            "version": "${VERSION}",
            "filename": "${PACKAGE_NAME}",
            "checksum": "${CHECKSUM}",
            "size": $(stat -f%z "${PACKAGE_PATH}" 2>/dev/null || stat -c%s "${PACKAGE_PATH}")
        }
    ]
}
EOF

# Clean up
rm -rf "${TEMP_DIR}"

echo ""
echo "Package created: ${PACKAGE_PATH}"
echo "Checksum: ${CHECKSUM}"
echo ""
echo "To deploy to server:"
echo "  cp ${PACKAGE_PATH} /path/to/saint_os/resources/firmware/rpi5/"
echo "  cp ${OUTPUT_DIR}/info.json /path/to/saint_os/resources/firmware/rpi5/"
