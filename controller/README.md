# SAINT.OS Controller

A Tauri-based controller application for SAINT.OS that captures gamepad, trigger, touch, gyro, and touchscreen input and sends control messages to the SAINT.OS server via WebSocket.

## Stack

- **Frontend**: Angular 19 + TypeScript + Tailwind CSS
- **Backend**: Rust (Tauri 2.0)
- **Target Platforms**: Steam Deck (SteamOS/Linux), macOS, Windows

## Development Setup

### macOS

1. **Install prerequisites:**
   ```bash
   # Install Rust
   curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh
   source ~/.cargo/env

   # Install Node.js (via Homebrew)
   brew install node
   ```

2. **Install dependencies and run:**
   ```bash
   cd controller
   npm install
   npm run tauri dev
   ```

### Steam Deck (SteamOS)

Steam Deck requires additional setup due to its read-only filesystem.

1. **Switch to Desktop Mode:**
   - Press the Steam button
   - Select **Power** → **Switch to Desktop**
   - Open **Konsole** from the application launcher

2. **Set a password (if not already set):**
   ```bash
   passwd
   ```

3. **Disable read-only filesystem:**
   ```bash
   sudo steamos-readonly disable
   ```

4. **Initialize pacman keyring:**
   ```bash
   sudo pacman-key --init
   sudo pacman-key --populate archlinux
   sudo pacman-key --populate holo
   ```

5. **Install development dependencies:**
   ```bash
   sudo pacman -S --noconfirm \
     base-devel \
     glibc \
     linux-api-headers \
     webkit2gtk-4.1 \
     gtk3 \
     glib2 \
     pango \
     gdk-pixbuf2 \
     cairo \
     atk \
     libayatana-appindicator \
     librsvg \
     openssl \
     nodejs \
     npm \
     git \
     pkg-config \
     pcre2 \
     sysprof
   ```

6. **Set PKG_CONFIG_PATH (add to ~/.bashrc for persistence):**
   ```bash
   echo 'export PKG_CONFIG_PATH=/usr/lib/pkgconfig' >> ~/.bashrc
   source ~/.bashrc
   ```

7. **Install Rust:**
   ```bash
   curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh
   source ~/.cargo/env
   ```

8. **Clone and run:**
   ```bash
   mkdir -p ~/Projects
   cd ~/Projects
   git clone git@github.com:input-inc/saintos.git
   cd saintos/controller
   npm install
   npm run tauri dev
   ```

9. **(Optional) Re-enable read-only filesystem:**
   ```bash
   sudo steamos-readonly enable
   ```

## Cross-Development Workflow

For developing on Mac and testing on Steam Deck:

### On Mac (development):
```bash
# Make changes, test locally
npm run tauri dev

# Commit and push when ready
git add -A && git commit -m "Your message" && git push
```

### On Steam Deck (testing):
```bash
cd ~/Projects/saintos/controller

# Pull latest changes
git pull

# Run dev mode
npm run tauri dev

# Or build a release
npm run tauri build
```

### Quick Sync Script (Steam Deck)

Create `~/sync-and-run.sh`:
```bash
#!/bin/bash
cd ~/Projects/saintos/controller
git pull
npm run tauri dev
```

Then run: `~/sync-and-run.sh`

## Build Commands

```bash
# Development (with hot reload)
npm run tauri dev

# Production build (current platform)
npm run tauri build
```

## Troubleshooting

### Steam Deck: "Package X was not found"

Ensure PKG_CONFIG_PATH is set:
```bash
export PKG_CONFIG_PATH=/usr/lib/pkgconfig
```

Verify pkg-config can find the library:
```bash
pkg-config --modversion glib-2.0
```

### Steam Deck: "could not get file information for usr/include/..."

The filesystem is still read-only. Run:
```bash
sudo steamos-readonly disable
```

Then reinstall the packages.

### Steam Deck: "stdint.h: No such file or directory"

Missing C library headers:
```bash
sudo pacman -S --noconfirm glibc linux-api-headers
```

### Steam Deck: Keyring errors with pacman

Initialize the keyring:
```bash
sudo pacman-key --init
sudo pacman-key --populate archlinux
sudo pacman-key --populate holo
```
