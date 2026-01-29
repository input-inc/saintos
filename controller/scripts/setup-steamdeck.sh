#!/bin/bash

# Steam Deck Setup Script
# Run this ON the Steam Deck to install development dependencies
#
# Usage:
#   scp scripts/setup-steamdeck.sh deck@steamdeck.local:~/
#   ssh deck@steamdeck.local 'chmod +x setup-steamdeck.sh && ./setup-steamdeck.sh'

echo "=== Steam Deck Development Environment Setup ==="
echo ""

# Disable read-only filesystem temporarily
echo "Disabling read-only filesystem..."
sudo steamos-readonly disable

# Initialize pacman keyring if needed
sudo pacman-key --init
sudo pacman-key --populate archlinux

# Update package database
echo "Updating package database..."
sudo pacman -Sy

# Install build dependencies
echo "Installing build dependencies..."
sudo pacman -S --noconfirm --needed \
    base-devel \
    webkit2gtk \
    gtk3 \
    libappindicator-gtk3 \
    librsvg \
    libvips \
    nodejs \
    npm \
    openssl \
    appmenu-gtk-module

# Install Rust if not present
if ! command -v rustc &> /dev/null; then
    echo "Installing Rust..."
    curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh -s -- -y
    source "$HOME/.cargo/env"
else
    echo "Rust already installed: $(rustc --version)"
fi

# Create projects directory
mkdir -p ~/Projects

# Re-enable read-only filesystem
echo "Re-enabling read-only filesystem..."
sudo steamos-readonly enable

echo ""
echo "=== Setup Complete ==="
echo ""
echo "Make sure to run: source ~/.cargo/env"
echo "Then you can use: npm run tauri:dev"
