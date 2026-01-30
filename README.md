# SAINT.OS

**System for Articulated Intelligence and Navigation Tasks**

SAINT.OS is a ROS2-based operating system for track-based mobile robots featuring dual manipulator arms and an articulated head system. Designed to run on Raspberry Pi hardware with a distributed node architecture.

## Features

- **Distributed Architecture**: Central server with multiple peripheral nodes (head, arms, tracks, console)
- **Multiple Input Sources**: WebSocket, Unreal Engine LiveLink, iOS/Android face tracking, RC controllers
- **Unified Firmware**: Single firmware image with role-based node adoption
- **Real-time Control**: Low-latency input routing with configurable bindings
- **OTA Updates**: Over-the-air firmware updates for all nodes
- **Steam Deck Support**: Native controller app with gyro, touchpad, and back button support

## Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                         SAINT.OS Server                          │
│  ┌──────────────┐  ┌──────────────┐  ┌────────────────────────┐ │
│  │   WebSocket  │  │   LiveLink   │  │   Input/Output Router  │ │
│  │    Server    │  │   Receiver   │  │   (configurable maps)  │ │
│  └──────────────┘  └──────────────┘  └────────────────────────┘ │
└─────────────────────────────────────────────────────────────────┘
                              │ ROS2
        ┌─────────────────────┼─────────────────────┐
        ▼                     ▼                     ▼
   ┌─────────┐           ┌─────────┐           ┌─────────┐
   │  Head   │           │  Arms   │           │ Tracks  │
   │  Node   │           │  Node   │           │  Node   │
   └─────────┘           └─────────┘           └─────────┘
```

## Components

### Server (`saint_os/`)

The central coordinator running on Raspberry Pi 4/5:

- ROS2 node management and topic bridging
- WebSocket API for external control
- LiveLink receiver for Unreal Engine and face tracking
- Web administration interface
- Firmware repository and OTA distribution

### Controller App (`controller/`)

Cross-platform Tauri application for remote control:

- **Frontend**: Angular 19 + TypeScript + Tailwind CSS
- **Backend**: Rust with native input handling
- **Platforms**: Steam Deck (SteamOS), macOS, Windows, Linux
- **Features**: Gamepad input, gyro control, touchpad support, customizable bindings

### Firmware (`saint_os/firmware/`)

Unified firmware for peripheral nodes:

- Supports Raspberry Pi Pico W, ESP32-S3, and Raspberry Pi 4/5
- micro-ROS integration for ROS2 communication
- Role-based adoption (head, arms, tracks, console)
- GPIO control and sensor interfaces

## Getting Started

### Prerequisites

- Ubuntu 22.04/24.04 (server) or macOS/Windows (development)
- ROS2 Humble or Iron
- Node.js 18+ and npm (for controller app)
- Rust toolchain (for controller app)

### Server Installation

```bash
# Clone the repository
git clone https://github.com/your-org/saintos.git
cd saintos

# Install ROS2 dependencies
rosdep install --from-paths saint_os --ignore-src -y

# Build
cd saint_os
colcon build

# Source and run
source install/setup.bash
ros2 launch saint_os server.launch.py
```

### Controller App

```bash
cd controller

# Install dependencies
npm install

# Development
npm run tauri dev

# Build for production
npm run tauri build
```

See [INSTALL.md](INSTALL.md) for detailed installation instructions.

## Documentation

- [SAINT_OS_SPEC.md](SAINT_OS_SPEC.md) - Full system specification
- [HARDWARE.md](HARDWARE.md) - Hardware requirements and supported platforms
- [INSTALL.md](INSTALL.md) - Installation guide

## Hardware Support

### Server

| Platform | Status |
|----------|--------|
| Raspberry Pi 5 | Recommended |
| Raspberry Pi 4 | Supported |
| x86_64 Linux | Development |
| macOS (ARM) | Development |

### Nodes

| Platform | Status |
|----------|--------|
| Raspberry Pi Pico W | Recommended |
| ESP32-S3 | Supported |
| Raspberry Pi 4/5 | Supported |

### Controller App

| Platform | Status |
|----------|--------|
| Steam Deck | Primary target |
| Linux | Supported |
| macOS | Supported |
| Windows | Supported |

## License

[Add license information]

## Contributing

[Add contribution guidelines]
