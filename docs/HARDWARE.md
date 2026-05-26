# SAINT.OS Hardware Requirements

This document specifies the minimum and recommended hardware for running SAINT.OS.

## Server (Main Controller)

The SAINT.OS server runs the central coordinator, web interface, firmware repository, and micro-ROS agent. It requires a full Linux environment with ROS2 support.

### Minimum Requirements

| Component | Minimum | Recommended |
|-----------|---------|-------------|
| **Board** | Raspberry Pi 4 (2GB) | Raspberry Pi 5 (4GB+) |
| **RAM** | 2GB | 4GB+ |
| **Storage** | 16GB microSD | 32GB+ microSD or NVMe |
| **OS** | Ubuntu 22.04 Server (64-bit) | Ubuntu 24.04 Server (64-bit) |
| **Network** | Ethernet (required) | Gigabit Ethernet |

### Supported Server Platforms

| Platform | Status | Notes |
|----------|--------|-------|
| Raspberry Pi 5 | Recommended | Best performance, native NVMe support |
| Raspberry Pi 4 | Supported | 4GB model recommended |
| Raspberry Pi 4 (2GB) | Minimum | May struggle with many nodes |
| x86_64 Linux | Supported | For development/testing |
| macOS (Apple Silicon) | Development only | Via RoboStack/Conda |

### Server Responsibilities

- ROS2 node coordination
- micro-ROS agent (bridges microcontroller nodes to ROS2 network)
- Web administration interface (HTTP + WebSocket)
- Firmware repository and OTA updates
- Input processing (LiveLink, RC receiver, WebSocket)
- Route management and topic bridging

---

## Nodes (Peripheral Controllers)

Nodes handle specific robot functions (head, arms, tracks, console). SAINT.OS supports two types of nodes:

### Option A: Microcontroller Nodes (Recommended for most uses)

Lightweight, low-power nodes using micro-ROS over UDP.

#### Supported Microcontrollers

| Board | Status | RAM | Flash | Notes |
|-------|--------|-----|-------|-------|
| Adafruit Feather RP2040 | Supported | 264KB | 8MB | Requires Ethernet FeatherWing |
| Raspberry Pi Pico W | Planned | 264KB | 2MB | WiFi only (higher latency) |
| Raspberry Pi Pico | Supported | 264KB | 2MB | Requires ethernet adapter |
| ESP32-S3 | Planned | 512KB | 8MB+ | WiFi or ethernet |
| Teensy 4.1 | Planned | 1MB | 8MB | Native ethernet |

#### Ethernet Adapters (for boards without native ethernet)

| Adapter | Chip | Interface | Notes |
|---------|------|-----------|-------|
| Adafruit Ethernet FeatherWing | W5500 | SPI | For Feather boards |
| W5500-EVB-Pico | W5500 | SPI | Pico form factor with ethernet |
| Generic W5500 module | W5500 | SPI | Universal, requires wiring |

#### Microcontroller Node Architecture

```
┌─────────────────────────────────────────────────────────┐
│                    SAINT.OS Server                       │
│                    (Raspberry Pi 5)                      │
│  ┌─────────────┐  ┌─────────────┐  ┌─────────────────┐  │
│  │ saint_server│  │ micro-ROS   │  │ Web Interface   │  │
│  │ (ROS2 node) │  │ Agent       │  │ (HTTP/WS)       │  │
│  └──────┬──────┘  └──────┬──────┘  └─────────────────┘  │
│         │                │                               │
│         └───────┬────────┘                               │
│                 │ ROS2 DDS                               │
└─────────────────┼───────────────────────────────────────┘
                  │
        ┌─────────┴─────────┐ UDP (micro-ROS)
        │                   │
        ▼                   ▼
┌───────────────┐   ┌───────────────┐
│  Head Node    │   │  Tracks Node  │
│  (RP2040 +    │   │  (RP2040 +    │
│   Ethernet)   │   │   Ethernet)   │
└───────────────┘   └───────────────┘
```

### Option B: Full Linux Nodes (For complex processing)

For nodes requiring camera processing, ML inference, or high compute.

| Board | Status | Notes |
|-------|--------|-------|
| Raspberry Pi 4 (2GB+) | Supported | Full ROS2, no agent needed |
| Raspberry Pi 5 | Supported | Best for camera/ML workloads |
| Raspberry Pi Zero 2 W | Limited | Low RAM, WiFi only |

---

## Network Architecture

SAINT.OS uses a dual-network architecture:

### Internal Network (Nodes ↔ Server)

| Requirement | Specification |
|-------------|---------------|
| **Type** | Wired Ethernet (required for nodes) |
| **Speed** | 100Mbps minimum |
| **Topology** | Star (switch) or direct |
| **IP Range** | 192.168.10.0/24 (configurable) |
| **DHCP** | Server provides DHCP, or static IPs |

### External Network (Admin/Clients)

| Requirement | Specification |
|-------------|---------------|
| **Type** | WiFi or secondary Ethernet |
| **Purpose** | Web admin, LiveLink, external API |
| **Security** | Isolated from internal node network |

### Recommended Network Hardware

- **Switch**: Any managed or unmanaged gigabit switch
- **Cables**: Cat5e or Cat6 ethernet cables
- **WiFi**: Built-in Pi WiFi or USB adapter for external access

---

## micro-ROS Agent

The micro-ROS agent bridges microcontroller nodes (using Micro XRCE-DDS) to the full ROS2 network (using FastDDS/CycloneDDS). This is required because:

1. Microcontrollers cannot run full DDS (requires too much RAM/CPU)
2. micro-ROS uses a lightweight protocol (XRCE-DDS) optimized for constrained devices
3. The agent translates between XRCE-DDS ↔ full DDS

### Agent Deployment

The micro-ROS agent is **embedded in the saint_server** process:
- No separate process to manage
- Automatically starts with the server
- Listens on UDP port 8888 for micro-ROS clients
- Transparently bridges nodes to the ROS2 network

```
# The server handles everything - no separate agent needed
ros2 run saint_os saint_server
```

### Resource Usage

The embedded agent adds minimal overhead:
- ~10MB additional RAM
- Negligible CPU usage (event-driven)
- One UDP socket per transport

---

## Power Requirements

### Server

| Board | Idle | Load | Recommended PSU |
|-------|------|------|-----------------|
| Pi 4 | 3W | 6W | 5V 3A USB-C |
| Pi 5 | 4W | 10W | 5V 5A USB-C (official) |

### Microcontroller Nodes

| Board | Idle | Active | Power Source |
|-------|------|--------|--------------|
| Feather RP2040 + Ethernet | 150mA | 250mA | USB-C or LiPo |
| Pico + W5500 | 100mA | 200mA | USB or 5V input |

---

## Bill of Materials (Example Setup)

### Minimal Setup (Server + 1 Node)

| Item | Qty | Notes |
|------|-----|-------|
| Raspberry Pi 5 (4GB) | 1 | Server |
| Pi 5 power supply | 1 | 27W official PSU |
| microSD card (32GB+) | 1 | For server OS |
| Adafruit Feather RP2040 | 1 | Node |
| Adafruit Ethernet FeatherWing | 1 | Node ethernet |
| Ethernet switch (5-port) | 1 | Network |
| Ethernet cables | 2 | Server + node |
| USB-C cables | 2 | Power for node |

### Full Robot Setup (Server + 4 Nodes)

| Item | Qty | Notes |
|------|-----|-------|
| Raspberry Pi 5 (4GB) | 1 | Server |
| Pi 5 power supply | 1 | 27W official PSU |
| microSD card (32GB+) | 1 | For server OS |
| Adafruit Feather RP2040 | 4 | Head, Arms L, Arms R, Tracks |
| Adafruit Ethernet FeatherWing | 4 | Node ethernet |
| Ethernet switch (8-port) | 1 | Network |
| Ethernet cables | 5 | Server + 4 nodes |
| USB-C cables | 4 | Power for nodes |
| 5V power distribution | 1 | For robot power |

---

## Development Setup

For development and testing on a desktop/laptop:

| Component | Requirement |
|-----------|-------------|
| **OS** | Ubuntu 22.04/24.04, macOS (via RoboStack) |
| **ROS2** | Humble or newer |
| **RAM** | 8GB+ recommended |
| **Network** | Ethernet adapter (for node testing) |

See [INSTALL.md](INSTALL.md) for development environment setup.
