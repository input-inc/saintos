#!/bin/bash
#
# SAINT.OS Raspberry Pi 5 Node Uninstallation Script
#
# Usage: sudo ./uninstall.sh
#

set -e

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

echo -e "${YELLOW}========================================${NC}"
echo -e "${YELLOW}SAINT.OS Raspberry Pi 5 Node Uninstaller${NC}"
echo -e "${YELLOW}========================================${NC}"
echo ""

if [ "$EUID" -ne 0 ]; then
    echo -e "${RED}Error: This script must be run as root (sudo)${NC}"
    exit 1
fi

read -p "This will remove the SAINT node. Continue? (y/N) " -n 1 -r
echo
if [[ ! $REPLY =~ ^[Yy]$ ]]; then
    exit 0
fi

echo ""
echo "Step 1: Stopping service..."
systemctl stop saint-node 2>/dev/null || true
systemctl disable saint-node 2>/dev/null || true

echo "Step 2: Removing systemd service..."
rm -f /etc/systemd/system/saint-node.service
systemctl daemon-reload

echo "Step 3: Removing installation directory..."
rm -rf /opt/saint-node

echo "Step 4: Removing Python module symlink..."
SITE_PACKAGES=$(python3 -c "import site; print(site.getsitepackages()[0])" 2>/dev/null || echo "")
if [ -n "$SITE_PACKAGES" ]; then
    rm -rf "$SITE_PACKAGES/saint_node"
fi

echo "Step 5: Removing udev rules..."
rm -f /etc/udev/rules.d/99-saint-gpio.rules
udevadm control --reload-rules 2>/dev/null || true

read -p "Remove configuration files in /etc/saint-node? (y/N) " -n 1 -r
echo
if [[ $REPLY =~ ^[Yy]$ ]]; then
    rm -rf /etc/saint-node
    echo "Configuration removed."
else
    echo "Configuration preserved in /etc/saint-node/"
fi

echo ""
echo -e "${GREEN}Uninstallation complete!${NC}"
