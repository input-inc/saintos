#!/bin/bash
#
# SAINT.OS Raspberry Pi Node Installation Script
#
# Installs the SAINT node firmware on any supported Raspberry Pi
# (Pi 3 / Pi 4 / Pi 5). Run with sudo privileges:
#
#   sudo ./install.sh
#
# See firmware/raspberrypi/docs/INSTALL.md for the full guide.

set -e

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Script directory
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
FIRMWARE_DIR="$(dirname "$SCRIPT_DIR")"

echo -e "${GREEN}========================================${NC}"
echo -e "${GREEN}SAINT.OS Raspberry Pi Node Installer${NC}"
echo -e "${GREEN}========================================${NC}"
echo ""

# Check if running as root
if [ "$EUID" -ne 0 ]; then
    echo -e "${RED}Error: This script must be run as root (sudo)${NC}"
    exit 1
fi

# Detect device model. The firmware itself runs on Pi 3 / 4 / 5;
# the installer warns only on non-Pi hosts so a developer testing on
# a desktop x86 box still gets a clear signal.
if [ ! -f /proc/device-tree/model ]; then
    echo -e "${YELLOW}Warning: Cannot detect device model (not running on a Pi?)${NC}"
else
    MODEL=$(cat /proc/device-tree/model)
    echo -e "Detected: ${GREEN}$MODEL${NC}"

    if [[ "$MODEL" != *"Raspberry Pi"* ]]; then
        echo -e "${YELLOW}Warning: Not a Raspberry Pi — installer untested here${NC}"
        read -p "Continue anyway? (y/N) " -n 1 -r
        echo
        if [[ ! $REPLY =~ ^[Yy]$ ]]; then
            exit 1
        fi
    fi
fi

echo ""
echo "Step 1: Installing system dependencies..."
# libvlc-dev + vlc-bin: backs the built-in audio_player peripheral.
#   python-vlc (installed in step 3) wraps libvlc; without these
#   system packages the audio_player driver disables itself with a
#   loud error and the rest of the node keeps running.
# libgpiod2 / python3-gpiod: GPIO control (works on Pi 3/4 gpiochip0
#   and Pi 5 RP1 gpiochip4 alike).
apt-get update
apt-get install -y \
    python3-pip python3-venv \
    python3-gpiod libgpiod2 \
    vlc-bin libvlc-dev python3-vlc \
    alsa-utils

echo ""
echo "Step 2: Checking for ROS2 installation..."
ROS_SETUP=""
if [ -f "/opt/ros/jazzy/setup.bash" ]; then
    ROS_SETUP="/opt/ros/jazzy/setup.bash"
    echo -e "Found ROS2 ${GREEN}Jazzy${NC}"
elif [ -f "/opt/ros/humble/setup.bash" ]; then
    ROS_SETUP="/opt/ros/humble/setup.bash"
    echo -e "Found ROS2 ${GREEN}Humble${NC}"
else
    echo -e "${RED}Error: ROS2 not found!${NC}"
    echo "Please install ROS2 Jazzy or Humble first:"
    echo "  https://docs.ros.org/en/jazzy/Installation.html"
    exit 1
fi

# Source ROS2
source "$ROS_SETUP"

echo ""
echo "Step 3: Installing Python dependencies..."
# python-vlc is installed via apt (python3-vlc above) so this pip
# step covers only what apt doesn't ship.
pip3 install --break-system-packages pyyaml gpiod || pip3 install pyyaml gpiod

echo ""
echo "Step 4: Installing SAINT node package..."

# Create installation directory
INSTALL_DIR="/opt/saint-node"
mkdir -p "$INSTALL_DIR"

# Copy node package
cp -r "$FIRMWARE_DIR/saint_node" "$INSTALL_DIR/"

# Create symlink for Python module access
SITE_PACKAGES=$(python3 -c "import site; print(site.getsitepackages()[0])")
ln -sf "$INSTALL_DIR/saint_node" "$SITE_PACKAGES/saint_node" 2>/dev/null || \
    cp -r "$INSTALL_DIR/saint_node" "$SITE_PACKAGES/"

echo ""
echo "Step 5: Creating configuration + state directories..."
mkdir -p /etc/saint-node
chmod 755 /etc/saint-node
# Audio library: the built-in audio_player peripheral plays files
# from this folder. Default path matches the rpi5 board YAML's
# `library_path` param — operators drop .wav / .mp3 / .flac here.
mkdir -p /var/lib/saint-os/audio
chmod 755 /var/lib/saint-os/audio

echo ""
echo "Step 6: Installing systemd service..."
cp "$FIRMWARE_DIR/config/saint-node.service" /etc/systemd/system/
chmod 644 /etc/systemd/system/saint-node.service

# Update service file with correct ROS setup path
sed -i "s|/opt/ros/jazzy/setup.bash|$ROS_SETUP|g" /etc/systemd/system/saint-node.service

systemctl daemon-reload

echo ""
echo "Step 7: Enabling GPIO permissions..."

# Add user to gpio group if it exists
if getent group gpio > /dev/null; then
    usermod -aG gpio root
fi

# Create udev rule for GPIO access
cat > /etc/udev/rules.d/99-saint-gpio.rules << 'EOF'
# Allow GPIO access for SAINT node
SUBSYSTEM=="gpio", KERNEL=="gpiochip*", MODE="0660", GROUP="gpio"
EOF

udevadm control --reload-rules
udevadm trigger

echo ""
echo -e "${GREEN}========================================${NC}"
echo -e "${GREEN}Installation Complete!${NC}"
echo -e "${GREEN}========================================${NC}"
echo ""
echo "To start the SAINT node service:"
echo "  sudo systemctl start saint-node"
echo ""
echo "To enable auto-start on boot:"
echo "  sudo systemctl enable saint-node"
echo ""
echo "To view logs:"
echo "  journalctl -u saint-node -f"
echo ""
echo "Configuration files are stored in: /etc/saint-node/"
echo "Audio library:                     /var/lib/saint-os/audio/"
echo ""
echo "See firmware/raspberrypi/docs/INSTALL.md for OTA + troubleshooting."
echo ""
