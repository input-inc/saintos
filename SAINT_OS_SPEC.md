# SAINT.OS Project Specification

## Overview

SAINT.OS (System for Articulated Intelligence and Navigation Tasks) is a ROS2-based operating system for a track-based mobile robot platform featuring dual manipulator arms and an articulated head system. The system is designed to run entirely on Raspberry Pi hardware with a distributed node architecture.

---

## Hardware Architecture

### Platform
- **Locomotion**: Dual track drive system
- **Manipulation**: 2 articulated arms
- **Perception**: Articulated head with sensors
- **Compute**: Raspberry Pi cluster

### Compute Nodes
| Node | Hardware | Function |
|------|----------|----------|
| Main Server | Raspberry Pi | Central coordinator, firmware storage, WebSocket gateway |
| Head Node | Raspberry Pi | Head articulation, sensors, cameras |
| Arms Node | Raspberry Pi | Dual arm control and feedback |
| Tracks Node | Raspberry Pi | Track motor control and odometry |
| Console Node | Raspberry Pi | User interface, status display |

---

## Software Architecture

### ROS2 Distribution
- **Target**: ROS2 Humble (LTS) or Iron
- **DDS**: Default FastDDS or CycloneDDS for Pi optimization

### Build Architecture

The SAINT.OS system uses a **unified build** approach:
- **Server software** and **generic node software** are compiled together
- A single node binary contains all role logic (head, arms, tracks, console)
- Nodes boot into an **unadopted state** until assigned a role
- Role assignment persists until software or hardware reset

```
┌─────────────────────────────────────────────────────────┐
│                    BUILD OUTPUT                          │
├─────────────────────────────────────────────────────────┤
│  saint_server        - Server binary + management UI    │
│  saint_node          - Generic node binary (all roles)  │
│  firmware/           - Packaged node firmware image     │
└─────────────────────────────────────────────────────────┘
```

### System Topology

```
                          EXTERNAL INPUTS (WiFi / RF / Wired)
┌─────────────────┐  ┌─────────────────┐  ┌─────────────────┐  ┌─────────────────┐
│ External Control│  │  Unreal Engine  │  │ LiveLink Face   │  │  RC Controller  │
│ (WebSocket App) │  │ (LiveLink)      │  │ (iOS/Android)   │  │  (RF Receiver)  │
└────────┬────────┘  └────────┬────────┘  └────────┬────────┘  └────────┬────────┘
         │ WebSocket          │ LiveLink           │ LiveLink           │ PWM/PPM/
         │ (WiFi)             │ (WiFi)             │ (WiFi)             │ SBUS/IBUS
         │                    └──────────┬─────────┘                    │ (GPIO)
         │                               │                              │
         ▼                               ▼                              ▼
┌────────────────────────────────────────────────────────────────────────────────┐
│                              MAIN SERVER (Pi)                                   │
│  ┌───────────┐  ┌───────────┐  ┌───────────┐  ┌───────────┐  ┌───────────────┐ │
│  │ Firmware  │  │ WebSocket │  │ LiveLink  │  │    RC     │  │  ROS2 Bridge  │ │
│  │ Repository│  │ Server    │  │ Receiver  │  │ Receiver  │  │ (cmd↔topics)  │ │
│  └───────────┘  └─────┬─────┘  └─────┬─────┘  └─────┬─────┘  └───────────────┘ │
│                       │              │              │                          │
│                       ▼              ▼              ▼                          │
│  ┌─────────────────────────────────────────────────────────────────────────┐   │
│  │                       Input/Output Router                                │   │
│  │  - Map any input source to any node output                              │   │
│  │  - Sources: WebSocket, LiveLink, RC Controller, Autonomous              │   │
│  │  - Configurable via WebSocket/Web UI                                    │   │
│  │  - Blend/prioritize multiple input sources                              │   │
│  └─────────────────────────────────────────────────────────────────────────┘   │
│  ┌─────────────────────────────────────────────────────────────────────────┐   │
│  │                       Node Management Interface                          │   │
│  │  - View unadopted nodes        - Assign roles                           │   │
│  │  - Monitor GPIO status         - Reset nodes                            │   │
│  └─────────────────────────────────────────────────────────────────────────┘   │
└────────────────────────────────────────────────────────────────────────────────┘
           │ ROS2 Topics (Internal Ethernet)
           ▼
    ┌──────┴──────┬──────────────┬──────────────┐
    ▼             ▼              ▼              ▼
┌───────┐   ┌─────────┐   ┌──────────┐   ┌─────────┐
│ Node  │   │  Node   │   │  Node    │   │  Node   │
│(Head) │   │ (Arms)  │   │ (Tracks) │   │(Console)│
└───────┘   └─────────┘   └──────────┘   └─────────┘
     ▲           ▲              ▲              ▲
     └───────────┴──────────────┴──────────────┘
              Same generic firmware
              Role assigned via adoption
```

---

## Generic Node Architecture

### Overview

All nodes run identical firmware containing logic for every possible role. This provides:
- **Single firmware image** to maintain and deploy
- **Hot-swappable hardware** - replace a failed Pi, adopt it as the same role
- **Flexible configuration** - repurpose nodes without reflashing
- **Simplified updates** - one firmware update applies to all nodes

### Node States

```
┌─────────────┐
│   BOOT      │
└──────┬──────┘
       ▼
┌─────────────┐     ┌─────────────────────────────────────┐
│  UPDATING   │◄────│ Firmware update available from server│
└──────┬──────┘     └─────────────────────────────────────┘
       ▼
┌─────────────┐
│  UNADOPTED  │◄──── Advertises on /saint/nodes/unadopted
└──────┬──────┘      Publishes GPIO status
       │
       │ Role assigned via management interface
       ▼
┌─────────────┐
│  ADOPTING   │◄──── Downloads role configuration
└──────┬──────┘      Initializes role-specific hardware
       ▼
┌─────────────┐
│   ACTIVE    │◄──── Operating in assigned role
└──────┬──────┘      Publishes to role-specific topics
       │
       │ Software reset command OR hardware reset button
       ▼
┌─────────────┐
│   BOOT      │      Returns to unadopted state
└─────────────┘
```

### Role Modules

The generic node firmware includes all role modules compiled in:

```
saint_node binary
├── core/
│   ├── boot_manager      - Startup, update check, state machine
│   ├── gpio_monitor      - Hardware abstraction, GPIO reporting
│   ├── adoption_client   - Server communication for adoption
│   └── firmware_client   - Firmware update client
├── roles/
│   ├── head_role         - Head articulation logic
│   ├── arms_role         - Arm control logic
│   ├── tracks_role       - Track drive logic
│   └── console_role      - Console interface logic
└── drivers/
    ├── servo_driver      - PWM servo control
    ├── motor_driver      - DC/stepper motor control
    ├── encoder_driver    - Rotary encoder input
    ├── camera_driver     - Camera capture
    └── display_driver    - LCD/OLED output
```

---

## Node Adoption System

### Unadopted Node Behavior

When a node boots without an assigned role:

1. **Announces presence** on `/saint/nodes/unadopted` topic
2. **Reports hardware info**:
   - Unique node ID (MAC address or serial)
   - Hardware revision
   - Firmware version
   - Available GPIO pins and their current state
   - Connected peripherals detected
3. **Listens** for adoption commands from server
4. **Responds** to GPIO probe requests for diagnostics

### GPIO Status Reporting

Unadopted nodes continuously publish GPIO information:

```
# NodeGPIOStatus.msg
string node_id                    # Unique identifier
string hardware_rev               # Hardware revision
string firmware_version           # Current firmware version
GPIOPin[] pins                    # Array of GPIO pin states
PeripheralInfo[] peripherals      # Detected peripherals
float32 cpu_temp                  # CPU temperature
float32 cpu_usage                 # CPU usage percentage
float32 memory_usage              # Memory usage percentage
uint32 uptime_seconds             # Seconds since boot
```

```
# GPIOPin.msg
uint8 pin_number
string pin_name                   # BCM name (e.g., "GPIO17")
string direction                  # "input", "output", "pwm", "i2c", "spi", "uart"
string current_state              # "high", "low", "pwm_50", etc.
bool in_use                       # Currently claimed by a driver
string used_by                    # Driver name if in_use
```

```
# PeripheralInfo.msg
string type                       # "i2c", "spi", "usb", "camera"
string address                    # Bus address or port
string description                # Detected device description
bool responsive                   # Device responding
```

### Adoption Process

**Step 1: Discovery**
- Management interface subscribes to `/saint/nodes/unadopted`
- Displays list of available nodes with their GPIO status

**Step 2: Role Assignment**
- Administrator selects a node and assigns a role
- Server calls `/saint/node/{node_id}/adopt` service

**Step 3: Configuration Download**
- Node receives role assignment
- Downloads role-specific configuration from server
- Configuration includes GPIO pin mappings for the role

**Step 4: Role Activation**
- Node transitions to ADOPTING state
- Initializes hardware drivers for assigned role
- Verifies all required peripherals are present
- Transitions to ACTIVE state
- Begins publishing on role-specific topics

**Step 5: Persistence**
- Node stores assigned role in local storage
- On reboot, node checks for stored role
- If role exists and valid, skips unadopted state
- Role persists until explicit reset

### Adoption Services

**On Server:**
```
# AdoptNode.srv
string node_id
string role                       # "head", "arms", "tracks", "console"
string instance                   # Optional: "left", "right" for arms
---
bool success
string message
string assigned_topic_prefix      # e.g., "/saint/head" or "/saint/arms/left"
```

```
# ResetNode.srv
string node_id
bool factory_reset                # Also clear firmware (re-download on boot)
---
bool success
string message
```

**On Node:**
```
# Service: /saint/node/{node_id}/adopt
# Accepts adoption request from server

# Service: /saint/node/{node_id}/reset
# Triggers return to unadopted state

# Service: /saint/node/{node_id}/gpio_probe
# Returns detailed GPIO diagnostics
```

---

## Web Administration Interface

### Overview

The main server hosts a **web-based administration interface** served over HTTP. This provides a complete management dashboard for the SAINT.OS robot, accessible from any device with a web browser on the WiFi network.

### Web Server

| Setting | Value |
|---------|-------|
| URL | `http://saint-server.local/` or `http://<server-ip>/` |
| Port | 80 (HTTP) |
| Technology | aiohttp serving static files + WebSocket API |
| Authentication | Optional token-based auth (configurable) |

### Site Structure

```
┌─────────────────────────────────────────────────────────────────────────┐
│  SAINT.OS Administration                              [Status: Online]  │
├─────────────────────────────────────────────────────────────────────────┤
│                                                                          │
│  ┌──────────┐ ┌──────────┐ ┌──────────┐ ┌──────────┐ ┌──────────┐      │
│  │Dashboard │ │  Nodes   │ │  Routes  │ │  Inputs  │ │ Settings │      │
│  └──────────┘ └──────────┘ └──────────┘ └──────────┘ └──────────┘      │
│                                                                          │
└─────────────────────────────────────────────────────────────────────────┘
```

### Page Descriptions

#### 1. Dashboard (`/`)

**Main activity view showing real-time system status.**

```
┌─────────────────────────────────────────────────────────────────────────┐
│  Dashboard                                                               │
├─────────────────────────────────────────────────────────────────────────┤
│                                                                          │
│  SYSTEM STATUS                          ACTIVITY LOG                    │
│  ┌────────────────────────────┐        ┌─────────────────────────────┐ │
│  │ Server: ● Online           │        │ 14:32:01 Head node adopted  │ │
│  │ Uptime: 3d 14h 22m         │        │ 14:31:45 LiveLink connected │ │
│  │ CPU: 23%  Memory: 45%      │        │ 14:30:12 RC signal acquired │ │
│  │                            │        │ 14:28:33 Tracks calibrated  │ │
│  │ Network:                   │        │ 14:25:01 System started     │ │
│  │  Internal: 192.168.10.1    │        │ ...                         │ │
│  │  External: 192.168.1.50    │        │                             │ │
│  └────────────────────────────┘        └─────────────────────────────┘ │
│                                                                          │
│  NODES                                  INPUTS                          │
│  ┌────────────────────────────┐        ┌─────────────────────────────┐ │
│  │ ● Head      Online  24°C   │        │ ● WebSocket    2 clients    │ │
│  │ ● Arms L    Online  31°C   │        │ ● LiveLink     FaceCapture  │ │
│  │ ● Arms R    Online  29°C   │        │ ● RC Receiver  8ch SBUS     │ │
│  │ ● Tracks    Online  35°C   │        │ ○ Autonomous   Disabled     │ │
│  │ ○ Console   Offline        │        │                             │ │
│  │ ? Unknown   Unadopted      │        │                             │ │
│  └────────────────────────────┘        └─────────────────────────────┘ │
│                                                                          │
│  QUICK ACTIONS                                                          │
│  [Adopt New Node]  [E-Stop All]  [View Logs]  [System Restart]         │
│                                                                          │
└─────────────────────────────────────────────────────────────────────────┘
```

**Features:**
- Real-time system health metrics
- Live activity log with filtering
- Node status summary with alerts
- Active input sources overview
- Quick action buttons

#### 2. Nodes (`/nodes`)

**Node management, adoption, and monitoring.**

```
┌─────────────────────────────────────────────────────────────────────────┐
│  Nodes                                              [+ Scan for Nodes]  │
├─────────────────────────────────────────────────────────────────────────┤
│                                                                          │
│  ADOPTED NODES                                                          │
│  ┌────────────────────────────────────────────────────────────────────┐ │
│  │ Node             Role      Status    IP             Actions        │ │
│  ├────────────────────────────────────────────────────────────────────┤ │
│  │ pi-a1b2c3d4     Head      ● Online  192.168.10.11  [View] [Reset] │ │
│  │ pi-e5f6g7h8     Arms L    ● Online  192.168.10.12  [View] [Reset] │ │
│  │ pi-i9j0k1l2     Arms R    ● Online  192.168.10.13  [View] [Reset] │ │
│  │ pi-m3n4o5p6     Tracks    ● Online  192.168.10.14  [View] [Reset] │ │
│  │ pi-q7r8s9t0     Console   ○ Offline 192.168.10.15  [View] [Reset] │ │
│  └────────────────────────────────────────────────────────────────────┘ │
│                                                                          │
│  UNADOPTED NODES                                                        │
│  ┌────────────────────────────────────────────────────────────────────┐ │
│  │ Node             Hardware     Firmware    Detected     Actions     │ │
│  ├────────────────────────────────────────────────────────────────────┤ │
│  │ pi-u1v2w3x4     Pi 4 4GB     v1.2.0      2 min ago    [Adopt]     │ │
│  │ pi-y5z6a7b8     Pi 4 2GB     v1.1.0      5 min ago    [Adopt]     │ │
│  └────────────────────────────────────────────────────────────────────┘ │
│                                                                          │
└─────────────────────────────────────────────────────────────────────────┘
```

**Subpages:**

##### Node Details (`/nodes/{node_id}`)

```
┌─────────────────────────────────────────────────────────────────────────┐
│  Node: pi-a1b2c3d4 (Head)                          [Reset] [Unadopt]   │
├─────────────────────────────────────────────────────────────────────────┤
│                                                                          │
│  HARDWARE INFO                          CURRENT STATE                   │
│  ┌────────────────────────────┐        ┌─────────────────────────────┐ │
│  │ Model: Raspberry Pi 4B     │        │ Pan:   45.2°                │ │
│  │ RAM: 4GB                   │        │ Tilt: -12.8°                │ │
│  │ Serial: a1b2c3d4           │        │ Roll:   0.0°                │ │
│  │ MAC: dc:a6:32:xx:xx:xx     │        │ Camera: Active              │ │
│  │ Firmware: v1.2.0           │        │                             │ │
│  │ Config: v1.2.0             │        │ Last command: 0.2s ago      │ │
│  └────────────────────────────┘        └─────────────────────────────┘ │
│                                                                          │
│  SYSTEM METRICS                         GPIO STATUS                     │
│  ┌────────────────────────────┐        ┌─────────────────────────────┐ │
│  │ CPU: ████████░░ 78%        │        │ GPIO12 (PWM) → Pan Servo    │ │
│  │ Mem: ████░░░░░░ 42%        │        │ GPIO13 (PWM) → Tilt Servo   │ │
│  │ Temp: 52°C                 │        │ GPIO18 (PWM) → Roll Servo   │ │
│  │ Uptime: 14h 32m            │        │ GPIO4  (I2C) → IMU          │ │
│  │ ROS2: Connected            │        │ CSI    (CAM) → Pi Camera    │ │
│  └────────────────────────────┘        └─────────────────────────────┘ │
│                                                                          │
│  RECENT ACTIVITY                                                        │
│  ┌────────────────────────────────────────────────────────────────────┐ │
│  │ 14:32:15  Received head/cmd: pan=45, tilt=-13                     │ │
│  │ 14:32:14  Published head/state                                     │ │
│  │ 14:32:13  Received head/cmd: pan=44, tilt=-12                     │ │
│  │ ...                                                                │ │
│  └────────────────────────────────────────────────────────────────────┘ │
│                                                                          │
└─────────────────────────────────────────────────────────────────────────┘
```

##### Adopt Node (`/nodes/adopt/{node_id}`)

```
┌─────────────────────────────────────────────────────────────────────────┐
│  Adopt Node: pi-u1v2w3x4                                    [Cancel]   │
├─────────────────────────────────────────────────────────────────────────┤
│                                                                          │
│  NODE INFORMATION                       GPIO DETECTED                   │
│  ┌────────────────────────────┐        ┌─────────────────────────────┐ │
│  │ Model: Raspberry Pi 4B     │        │     3V3 [ ][ ] 5V           │ │
│  │ RAM: 4GB                   │        │   GPIO2 [ ][ ] 5V           │ │
│  │ Firmware: v1.2.0           │        │   GPIO3 [ ][ ] GND          │ │
│  │ IP: 192.168.10.20          │        │   GPIO4 [I][ ] GPIO14       │ │
│  │ First seen: 2 min ago      │        │     GND [ ][ ] GPIO15       │ │
│  │                            │        │  GPIO17 [ ][P] GPIO18       │ │
│  │ Peripherals detected:      │        │  GPIO27 [ ][ ] GND          │ │
│  │  - I2C: 0x68 (MPU6050?)    │        │  GPIO22 [ ][P] GPIO23       │ │
│  │  - Camera: Pi Camera v2    │        │     ... more pins ...       │ │
│  │  - PWM: 3 servos?          │        │                             │ │
│  └────────────────────────────┘        │  [I]=I2C [P]=PWM [S]=SPI    │ │
│                                         └─────────────────────────────┘ │
│                                                                          │
│  SELECT ROLE                                                            │
│  ┌────────────────────────────────────────────────────────────────────┐ │
│  │  ○ Head       - Pan/tilt/roll articulation, camera                │ │
│  │  ○ Arms Left  - Left arm joint control, gripper                   │ │
│  │  ○ Arms Right - Right arm joint control, gripper                  │ │
│  │  ○ Tracks     - Differential drive, odometry                      │ │
│  │  ○ Console    - Display, user input                               │ │
│  └────────────────────────────────────────────────────────────────────┘ │
│                                                                          │
│  DISPLAY NAME: [Head Unit_________________]                             │
│                                                                          │
│                                            [Cancel]  [Adopt as Head]   │
│                                                                          │
└─────────────────────────────────────────────────────────────────────────┘
```

#### 3. Routes (`/routes`)

**Input/Output mapping configuration.**

```
┌─────────────────────────────────────────────────────────────────────────┐
│  Routes                                  [+ New Route]  [Load Preset]  │
├─────────────────────────────────────────────────────────────────────────┤
│                                                                          │
│  ACTIVE ROUTES                                                          │
│  ┌────────────────────────────────────────────────────────────────────┐ │
│  │ Route              Input        Output     Priority  Status        │ │
│  ├────────────────────────────────────────────────────────────────────┤ │
│  │ rc_drive          RC Ch1-2     Tracks     300       ● Active      │ │
│  │ rc_head           RC Ch3-4     Head       300       ● Active      │ │
│  │ facecap_head      LiveLink     Head       100       ● Active      │ │
│  │ websocket_ctrl    WebSocket    All        200       ● Active      │ │
│  │ estop_override    RC Ch6       Tracks     1000      ○ Standby     │ │
│  └────────────────────────────────────────────────────────────────────┘ │
│                                                                          │
│  [Edit] [Disable] [Delete] buttons appear on row hover                  │
│                                                                          │
│  ROUTE FLOW VISUALIZATION                                               │
│  ┌────────────────────────────────────────────────────────────────────┐ │
│  │                                                                     │ │
│  │   INPUTS              ROUTER                OUTPUTS                │ │
│  │   ┌─────────┐                              ┌─────────┐             │ │
│  │   │ RC      │────┐    ┌──────────┐    ┌───│ Head    │             │ │
│  │   │ Ch 1-6  │    ├───►│ Priority │────┤   └─────────┘             │ │
│  │   └─────────┘    │    │ Blending │    │   ┌─────────┐             │ │
│  │   ┌─────────┐    │    └──────────┘    ├───│ Arms L  │             │ │
│  │   │LiveLink │────┤                    │   └─────────┘             │ │
│  │   │FaceCap  │    │                    │   ┌─────────┐             │ │
│  │   └─────────┘    │                    ├───│ Arms R  │             │ │
│  │   ┌─────────┐    │                    │   └─────────┘             │ │
│  │   │WebSocket│────┘                    │   ┌─────────┐             │ │
│  │   │ Cmds    │                         └───│ Tracks  │             │ │
│  │   └─────────┘                             └─────────┘             │ │
│  │                                                                     │ │
│  └────────────────────────────────────────────────────────────────────┘ │
│                                                                          │
│  PRESETS                                                                │
│  ┌────────────────────────────────────────────────────────────────────┐ │
│  │ [RC Manual]  [Face Capture]  [UE Animation]  [WebSocket Only]      │ │
│  │                                                                     │ │
│  │ [Save Current as Preset...]                                        │ │
│  └────────────────────────────────────────────────────────────────────┘ │
│                                                                          │
└─────────────────────────────────────────────────────────────────────────┘
```

##### Edit Route (`/routes/edit/{route_id}`)

```
┌─────────────────────────────────────────────────────────────────────────┐
│  Edit Route: rc_drive                               [Delete] [Save]    │
├─────────────────────────────────────────────────────────────────────────┤
│                                                                          │
│  BASIC SETTINGS                                                         │
│  ┌────────────────────────────────────────────────────────────────────┐ │
│  │ Name:     [rc_drive_______________]                                │ │
│  │ Priority: [300_] (higher = overrides lower)                        │ │
│  │ Enabled:  [✓]                                                      │ │
│  └────────────────────────────────────────────────────────────────────┘ │
│                                                                          │
│  INPUT SOURCE                            OUTPUT TARGET                  │
│  ┌─────────────────────────┐            ┌─────────────────────────┐    │
│  │ Source: [RC Controller▼]│            │ Target: [Tracks      ▼]│    │
│  │ Protocol: SBUS          │            │                        │    │
│  └─────────────────────────┘            └─────────────────────────┘    │
│                                                                          │
│  CHANNEL MAPPINGS                                        [+ Add Mapping]│
│  ┌────────────────────────────────────────────────────────────────────┐ │
│  │ From           To          Scale   Deadzone  Curve      Actions   │ │
│  ├────────────────────────────────────────────────────────────────────┤ │
│  │ [Channel 2▼]   [linear▼]   [1.0]   [0.05]    [expo 0.3] [×]       │ │
│  │ [Channel 1▼]   [angular▼]  [1.0]   [0.05]    [linear  ] [×]       │ │
│  └────────────────────────────────────────────────────────────────────┘ │
│                                                                          │
│  LIVE PREVIEW                                                           │
│  ┌────────────────────────────────────────────────────────────────────┐ │
│  │ Input: Ch2 = 0.75   →   Output: linear = 0.75                     │ │
│  │ Input: Ch1 = -0.20  →   Output: angular = -0.20                   │ │
│  └────────────────────────────────────────────────────────────────────┘ │
│                                                                          │
│                                                    [Cancel]  [Save]    │
│                                                                          │
└─────────────────────────────────────────────────────────────────────────┘
```

#### 4. Inputs (`/inputs`)

**Monitor and configure all input sources.**

```
┌─────────────────────────────────────────────────────────────────────────┐
│  Inputs                                                                 │
├─────────────────────────────────────────────────────────────────────────┤
│                                                                          │
│  ┌──────────┐ ┌──────────┐ ┌──────────┐ ┌──────────┐                   │
│  │WebSocket │ │ LiveLink │ │    RC    │ │Autonomous│                   │
│  └──────────┘ └──────────┘ └──────────┘ └──────────┘                   │
│                                                                          │
├─────────────────────────────────────────────────────────────────────────┤
│  WEBSOCKET CLIENTS                                                      │
│  ┌────────────────────────────────────────────────────────────────────┐ │
│  │ Client ID          IP Address       Connected    Messages/sec      │ │
│  ├────────────────────────────────────────────────────────────────────┤ │
│  │ controller_001     192.168.1.100    2h 15m       12.3              │ │
│  │ admin_browser      192.168.1.105    0h 05m       0.5               │ │
│  └────────────────────────────────────────────────────────────────────┘ │
│                                                                          │
├─────────────────────────────────────────────────────────────────────────┤
│  LIVELINK SOURCES                                        [Refresh]     │
│  ┌────────────────────────────────────────────────────────────────────┐ │
│  │ Subject          Type       Source IP       FPS    Properties      │ │
│  ├────────────────────────────────────────────────────────────────────┤ │
│  │ FaceCapture      Face       192.168.1.50    60     52 blendshapes  │ │
│  │ UE_Character     Animation  192.168.1.100   30     65 bones        │ │
│  └────────────────────────────────────────────────────────────────────┘ │
│                                                                          │
│  LiveLink Subject Details (click to expand):                            │
│  ┌────────────────────────────────────────────────────────────────────┐ │
│  │ FaceCapture - Live Values:                                         │ │
│  │  headYaw: ████████░░ 0.42    jawOpen: ██░░░░░░░░ 0.12              │ │
│  │  headPitch: ███░░░░░░░ -0.15  browUp: ████░░░░░░ 0.25              │ │
│  │  headRoll: █░░░░░░░░░ 0.02   eyeBlink: ░░░░░░░░░░ 0.00             │ │
│  └────────────────────────────────────────────────────────────────────┘ │
│                                                                          │
├─────────────────────────────────────────────────────────────────────────┤
│  RC RECEIVER                               [Configure] [Calibrate]     │
│  ┌────────────────────────────────────────────────────────────────────┐ │
│  │ Status: ● Connected    Protocol: SBUS    Signal: Strong (-45dBm)  │ │
│  │ Failsafe: Off          Frame Rate: 50 Hz                           │ │
│  │                                                                     │ │
│  │ Channels:                                                          │ │
│  │  CH1 Steering:  ████████░░░░░░░░░░░░ 0.42                         │ │
│  │  CH2 Throttle:  ██████████░░░░░░░░░░ 0.00 (center)                │ │
│  │  CH3 Head Tilt: ████████████░░░░░░░░ 0.25                         │ │
│  │  CH4 Head Pan:  ██████████░░░░░░░░░░ 0.00 (center)                │ │
│  │  CH5 Gripper:   ████████████████████ 1.00 (switch ON)             │ │
│  │  CH6 E-Stop:    ░░░░░░░░░░░░░░░░░░░░ 0.00 (switch OFF)            │ │
│  └────────────────────────────────────────────────────────────────────┘ │
│                                                                          │
└─────────────────────────────────────────────────────────────────────────┘
```

#### 5. Settings (`/settings`)

**System configuration and maintenance.**

```
┌─────────────────────────────────────────────────────────────────────────┐
│  Settings                                                               │
├─────────────────────────────────────────────────────────────────────────┤
│                                                                          │
│  SYSTEM                                                                 │
│  ┌────────────────────────────────────────────────────────────────────┐ │
│  │ Server Name:     [SAINT-01_______________]                         │ │
│  │ Hostname:        [saint-server____________]                        │ │
│  │                                                                     │ │
│  │ Internal Network: 192.168.10.0/24 (eth0)                           │ │
│  │ External Network: DHCP (wlan0) - Current: 192.168.1.50             │ │
│  └────────────────────────────────────────────────────────────────────┘ │
│                                                                          │
│  FIRMWARE                                                               │
│  ┌────────────────────────────────────────────────────────────────────┐ │
│  │ Current Version: v1.2.0                                            │ │
│  │ Last Updated: 2026-01-20                                           │ │
│  │                                                                     │ │
│  │ [Check for Updates]  [Upload Firmware]  [View Changelog]           │ │
│  └────────────────────────────────────────────────────────────────────┘ │
│                                                                          │
│  LIVELINK                                                               │
│  ┌────────────────────────────────────────────────────────────────────┐ │
│  │ Enabled: [✓]                                                       │ │
│  │ Discovery Port: [54321]    Data Port: [54322]                      │ │
│  │ Broadcast Name: [SAINT-01_____________]                            │ │
│  └────────────────────────────────────────────────────────────────────┘ │
│                                                                          │
│  RC RECEIVER                                                            │
│  ┌────────────────────────────────────────────────────────────────────┐ │
│  │ Enabled: [✓]                                                       │ │
│  │ Protocol: [SBUS          ▼]                                        │ │
│  │ Failsafe Action: [Hold      ▼]                                     │ │
│  │ Signal Timeout: [500] ms                                           │ │
│  │                                                                     │ │
│  │ [Configure Channels]  [Run Calibration]                            │ │
│  └────────────────────────────────────────────────────────────────────┘ │
│                                                                          │
│  AUTHENTICATION                                                         │
│  ┌────────────────────────────────────────────────────────────────────┐ │
│  │ Require Auth: [✓]                                                  │ │
│  │ API Token: [••••••••••••••••]  [Regenerate]  [Copy]               │ │
│  └────────────────────────────────────────────────────────────────────┘ │
│                                                                          │
│  MAINTENANCE                                                            │
│  ┌────────────────────────────────────────────────────────────────────┐ │
│  │ [View System Logs]  [Download Diagnostics]  [Factory Reset]        │ │
│  │ [Restart Server]    [Shutdown System]                              │ │
│  └────────────────────────────────────────────────────────────────────┘ │
│                                                                          │
└─────────────────────────────────────────────────────────────────────────┘
```

#### 6. Logs (`/logs`)

**System-wide activity and event logs.**

```
┌─────────────────────────────────────────────────────────────────────────┐
│  System Logs                                     [Download] [Clear]    │
├─────────────────────────────────────────────────────────────────────────┤
│                                                                          │
│  FILTERS                                                                │
│  ┌────────────────────────────────────────────────────────────────────┐ │
│  │ Level: [All▼]  Source: [All▼]  Node: [All▼]  Search: [________]   │ │
│  └────────────────────────────────────────────────────────────────────┘ │
│                                                                          │
│  ┌────────────────────────────────────────────────────────────────────┐ │
│  │ Time         Level   Source      Message                           │ │
│  ├────────────────────────────────────────────────────────────────────┤ │
│  │ 14:32:15.123 INFO    Router      Route rc_drive processed input   │ │
│  │ 14:32:15.001 DEBUG   LiveLink    Frame 12345 from FaceCapture     │ │
│  │ 14:32:14.892 INFO    Head        State published: pan=45.2        │ │
│  │ 14:32:01.445 INFO    Adoption    Node pi-a1b2c3d4 adopted as Head │ │
│  │ 14:31:45.221 INFO    LiveLink    Subject FaceCapture connected    │ │
│  │ 14:30:12.003 INFO    RC          Signal acquired, SBUS 8ch        │ │
│  │ 14:28:33.712 WARN    Tracks      High motor temperature: 52°C     │ │
│  │ 14:25:01.000 INFO    System      SAINT.OS v1.2.0 started          │ │
│  │ ...                                                                │ │
│  └────────────────────────────────────────────────────────────────────┘ │
│                                                                          │
│  [< Prev]  Page 1 of 234  [Next >]                    [Auto-refresh ✓] │
│                                                                          │
└─────────────────────────────────────────────────────────────────────────┘
```

### Web Interface Technology

| Component | Technology |
|-----------|------------|
| Serving | aiohttp static file serving |
| Frontend | Vanilla JS + minimal framework (Alpine.js or similar) |
| Styling | CSS (custom or Tailwind) |
| Real-time | WebSocket for live updates |
| Charts | Lightweight charting (Chart.js or similar) |

### API Endpoints

The web interface communicates with the server via:

| Endpoint | Protocol | Purpose |
|----------|----------|---------|
| `/` | HTTP | Static web UI files |
| `/api/ws` | WebSocket | All commands and real-time updates |
| `/api/firmware` | HTTP | Firmware upload/download |
| `/api/logs` | HTTP | Log file streaming |

### WebSocket Management Commands

```json
{
  "type": "management",
  "action": "list_unadopted",
  "params": {}
}
```

```json
{
  "type": "management",
  "action": "get_gpio_status",
  "params": {
    "node_id": "saint-node-a1b2c3"
  }
}
```

```json
{
  "type": "management",
  "action": "adopt_node",
  "params": {
    "node_id": "saint-node-a1b2c3",
    "role": "head",
    "display_name": "Head Unit"
  }
}
```

```json
{
  "type": "management",
  "action": "reset_node",
  "params": {
    "node_id": "saint-node-a1b2c3",
    "factory_reset": false
  }
}
```

```json
{
  "type": "management",
  "action": "get_activity_log",
  "params": {
    "limit": 100,
    "offset": 0,
    "level": "INFO",
    "source": null
  }
}
```

---

## Unreal LiveLink Integration

### Overview

The SAINT.OS server advertises itself as an **Unreal Engine LiveLink destination**, allowing:
- Unreal Engine projects to drive robot animation in real-time
- LiveLink-compatible face capture apps (Live Link Face, etc.) to animate the robot's head
- Multiple simultaneous LiveLink sources with configurable routing
- Blending between different input sources

### LiveLink Protocol

LiveLink uses a message-based protocol over TCP/UDP:
- **Discovery**: Server broadcasts presence via UDP multicast
- **Connection**: Clients connect via TCP for reliable frame data
- **Data**: Streaming animation frames at configurable rates

```
┌─────────────────────┐         ┌─────────────────────┐
│   Unreal Engine     │         │   Live Link Face    │
│   (Animation BP)    │         │   (iOS App)         │
└──────────┬──────────┘         └──────────┬──────────┘
           │                               │
           │ LiveLink Frame Data           │ LiveLink Frame Data
           │ (Skeleton/Transform)          │ (BlendShapes)
           │                               │
           ▼                               ▼
┌──────────────────────────────────────────────────────────┐
│              SAINT.OS LiveLink Receiver                   │
│  ┌────────────────────────────────────────────────────┐  │
│  │              Subject Registry                       │  │
│  │  - "UE_Character" (Skeleton)                       │  │
│  │  - "FaceCapture" (BlendShapes)                     │  │
│  │  - "VR_Controller_L" (Transform)                   │  │
│  └────────────────────────────────────────────────────┘  │
│                          │                                │
│                          ▼                                │
│  ┌────────────────────────────────────────────────────┐  │
│  │              Input/Output Router                    │  │
│  │  Maps LiveLink subjects → Robot outputs            │  │
│  └────────────────────────────────────────────────────┘  │
└──────────────────────────────────────────────────────────┘
```

### Supported LiveLink Subject Types

| Subject Type | Data Format | Typical Source | Robot Mapping |
|--------------|-------------|----------------|---------------|
| **Animation** | Bone transforms | Unreal Animation BP | Arm joints, full body |
| **Transform** | Position/Rotation | VR controllers, tracked objects | Head, individual joints |
| **Face** | BlendShape weights (ARKit) | Live Link Face app | Head expression, eye gaze |
| **Camera** | Transform + FOV | Virtual camera | Head tracking |

### LiveLink Subject Data

```
# LiveLinkFrame.msg (internal)
string subject_name               # e.g., "FaceCapture", "UE_Character"
string subject_type               # "animation", "transform", "face", "camera"
uint64 frame_number
float64 timestamp
string[] property_names           # BlendShape names or bone names
float32[] property_values         # Corresponding values
geometry_msgs/Transform transform # For transform-based subjects
```

### Face Capture Mapping

ARKit blend shapes from Live Link Face can drive head articulation:

```yaml
# Example: Face capture → Head movement mapping
face_to_head_mapping:
  # Eye gaze → head micro-movements
  eyeLookUpLeft: { target: head.tilt, scale: 0.1 }
  eyeLookDownLeft: { target: head.tilt, scale: -0.1 }
  eyeLookInLeft: { target: head.pan, scale: -0.05 }
  eyeLookOutLeft: { target: head.pan, scale: 0.05 }

  # Head rotation (if supported by capture app)
  headYaw: { target: head.pan, scale: 1.0 }
  headPitch: { target: head.tilt, scale: 1.0 }
  headRoll: { target: head.roll, scale: 1.0 }

  # Jaw → expression or secondary actuator
  jawOpen: { target: head.jaw, scale: 1.0 }
```

---

## RC Controller Integration

### Overview

The server supports an **optional RC (Radio Control) receiver** connected directly to the server's GPIO pins. This allows traditional hobby RC transmitters to control the robot, providing a familiar interface for manual operation and a failsafe control method.

### Supported Protocols

| Protocol | Description | Connection | Channels |
|----------|-------------|------------|----------|
| **PWM** | Individual PWM signal per channel | 1 GPIO per channel | 1-8 typical |
| **PPM** | Combined pulse stream | 1 GPIO (any) | Up to 8 |
| **SBUS** | Futaba serial protocol | UART (inverted) | Up to 16 |
| **IBUS** | FlySky serial protocol | UART | Up to 14 |
| **CRSF** | Crossfire/ELRS protocol | UART | Up to 16 |

### Hardware Connection

```
┌─────────────────────────────────────────────────────────────┐
│                     MAIN SERVER (Pi)                         │
│                                                              │
│   GPIO Header                                                │
│   ┌──────────────────────────────────────────────────────┐  │
│   │  ┌─────┐                                              │  │
│   │  │ 3.3V├──────────────────┐                          │  │
│   │  ├─────┤                  │                          │  │
│   │  │ GND ├───────────────┐  │                          │  │
│   │  ├─────┤               │  │                          │  │
│   │  │GPIO15├─── UART RX ──┼──┼─── SBUS/IBUS/CRSF        │  │
│   │  ├─────┤               │  │                          │  │
│   │  │GPIO18├─── PPM ──────┼──┼─── PPM Sum Signal        │  │
│   │  ├─────┤               │  │                          │  │
│   │  │GPIO12├─── PWM CH1 ──┼──┼─┐                        │  │
│   │  │GPIO13├─── PWM CH2 ──┼──┼─┤ Individual PWM         │  │
│   │  │GPIO19├─── PWM CH3 ──┼──┼─┤ Channels               │  │
│   │  │GPIO26├─── PWM CH4 ──┼──┼─┘                        │  │
│   │  └─────┘               │  │                          │  │
│   └────────────────────────┼──┼──────────────────────────┘  │
│                            │  │                              │
└────────────────────────────┼──┼──────────────────────────────┘
                             │  │
                             ▼  ▼
                    ┌─────────────────┐
                    │  RC Receiver    │
                    │  (SBUS/PPM/PWM) │
                    │                 │
                    │  ◄── RF ───     │
                    └─────────────────┘
                             ▲
                             │ 2.4GHz / 900MHz
                    ┌─────────────────┐
                    │  RC Transmitter │
                    │  (Handheld)     │
                    └─────────────────┘
```

### RC Channel Data

Each RC channel provides a value typically in the range 1000-2000μs (PWM) or 0-2047 (SBUS):

```
# RCChannelState.msg
uint8 channel_number              # 1-16
uint16 raw_value                  # Raw receiver value
float32 normalized                # -1.0 to 1.0 (centered) or 0.0 to 1.0
bool failsafe                     # True if signal lost
uint64 last_update_ns             # Timestamp of last valid signal
```

```
# RCReceiverState.msg
string protocol                   # "pwm", "ppm", "sbus", "ibus", "crsf"
bool connected                    # Receiver communication active
bool failsafe                     # Transmitter signal lost
uint8 channel_count               # Number of active channels
RCChannelState[] channels         # All channel states
float32 rssi                      # Signal strength (if available)
uint32 frame_rate_hz              # Update rate
```

### Channel Mapping Examples

RC channels can be mapped to any robot output via the router:

```yaml
# Example: RC transmitter → robot control
routes:
  - id: "rc_drive"
    enabled: true
    priority: 300                 # Higher than LiveLink/WebSocket for direct control
    input:
      source: "rc"
      protocol: "sbus"
    output:
      target: "tracks"
    mapping:
      - from: "channel_2"         # Right stick vertical (throttle)
        to: "linear"
        scale: 1.0
        deadzone: 0.05
        curve: "exponential"      # Optional: response curve
        expo: 0.3
      - from: "channel_1"         # Right stick horizontal (steering)
        to: "angular"
        scale: 1.0
        deadzone: 0.05

  - id: "rc_head"
    enabled: true
    priority: 300
    input:
      source: "rc"
    output:
      target: "head"
    mapping:
      - from: "channel_4"         # Left stick horizontal
        to: "pan"
        scale: 180.0              # Map to degrees
        deadzone: 0.05
      - from: "channel_3"         # Left stick vertical
        to: "tilt"
        scale: 90.0
        deadzone: 0.05

  - id: "rc_arm_gripper"
    enabled: true
    priority: 300
    input:
      source: "rc"
    output:
      target: "arms.left"
    mapping:
      - from: "channel_5"         # 2-position switch
        to: "gripper"
        mode: "switch"            # Binary open/close
        threshold: 0.5

  - id: "rc_estop"
    enabled: true
    priority: 1000                # Highest priority
    input:
      source: "rc"
    output:
      target: "tracks"
    mapping:
      - from: "channel_6"         # Emergency switch
        to: "estop"
        mode: "switch"
        inverted: true            # Low = stop
```

### RC Configuration

```yaml
# config/rc_receiver.yaml
rc_receiver:
  enabled: true
  protocol: "sbus"                # Primary protocol

  # UART settings (for SBUS/IBUS/CRSF)
  uart:
    device: "/dev/ttyAMA0"
    baud_rate: 100000             # SBUS baud rate
    inverted: true                # SBUS requires inversion

  # GPIO settings (for PWM/PPM)
  gpio:
    ppm_pin: 18
    pwm_pins: [12, 13, 19, 26]

  # Failsafe behavior
  failsafe:
    timeout_ms: 500               # Signal loss timeout
    action: "hold"                # "hold", "neutral", "estop"

  # Channel configuration
  channels:
    - number: 1
      name: "steering"
      min: 1000
      max: 2000
      center: 1500
      reversed: false
    - number: 2
      name: "throttle"
      min: 1000
      max: 2000
      center: 1500
      reversed: false
    # ... additional channels
```

### Failsafe Behavior

When RC signal is lost:

| Action | Behavior |
|--------|----------|
| `hold` | Maintain last known values (default) |
| `neutral` | Return all channels to center/neutral |
| `estop` | Trigger emergency stop on all motion |
| `passthrough` | Allow other input sources to take over |

### WebSocket RC Commands

```json
{
  "type": "rc",
  "action": "get_status",
  "params": {}
}
```

**Response:**
```json
{
  "type": "response",
  "status": "ok",
  "data": {
    "enabled": true,
    "connected": true,
    "protocol": "sbus",
    "failsafe": false,
    "rssi": -65,
    "frame_rate_hz": 50,
    "channels": [
      {"channel": 1, "raw": 1500, "normalized": 0.0, "name": "steering"},
      {"channel": 2, "raw": 1000, "normalized": -1.0, "name": "throttle"},
      {"channel": 3, "raw": 1500, "normalized": 0.0, "name": "head_tilt"},
      // ...
    ]
  }
}
```

```json
{
  "type": "rc",
  "action": "set_channel_config",
  "params": {
    "channel": 1,
    "name": "steering",
    "min": 1000,
    "max": 2000,
    "center": 1500,
    "reversed": false
  }
}
```

```json
{
  "type": "rc",
  "action": "set_enabled",
  "params": {
    "enabled": true
  }
}
```

---

## Input/Output Router

### Overview

The Input/Output Router is the central system for mapping input sources (LiveLink, WebSocket, RC controller, internal logic) to robot outputs (node topics). All routing is configurable via the web interface or WebSocket API.

### Input Sources

| Source Type | Description | Priority (default) |
|-------------|-------------|-------------------|
| `rc` | RC controller receiver | 300 |
| `websocket` | Direct commands from controller app | 200 |
| `livelink` | Unreal LiveLink animation data | 100 |
| `autonomous` | Internal AI/behavior system | 50 |
| `safety` | Emergency stop, limits | 1000 (highest) |

### Output Targets

| Target | Topic | Controllable Properties |
|--------|-------|------------------------|
| `head` | `/saint/head/cmd` | pan, tilt, roll, jaw, eyes |
| `arms.left` | `/saint/arms/left/cmd` | joint positions, gripper |
| `arms.right` | `/saint/arms/right/cmd` | joint positions, gripper |
| `tracks` | `/saint/tracks/cmd_vel` | linear, angular velocity |

### Routing Rules

Routes define how inputs map to outputs with optional transformations:

```yaml
# Example routing configuration
routes:
  - id: "facecap_to_head"
    enabled: true
    priority: 100
    input:
      source: "livelink"
      subject: "FaceCapture"
      type: "face"
    output:
      target: "head"
    mapping:
      - from: "headYaw"
        to: "pan"
        scale: 1.0
        offset: 0.0
        clamp: [-180, 180]
      - from: "headPitch"
        to: "tilt"
        scale: 1.0
        offset: 0.0
        clamp: [-90, 90]
      - from: "headRoll"
        to: "roll"
        scale: 1.0
        offset: 0.0
        clamp: [-45, 45]

  - id: "ue_character_to_arms"
    enabled: true
    priority: 100
    input:
      source: "livelink"
      subject: "UE_Character"
      type: "animation"
    output:
      target: "arms.left"
    mapping:
      - from: "LeftArm.shoulder"
        to: "joint_0"
        scale: 1.0
      - from: "LeftArm.elbow"
        to: "joint_1"
        scale: 1.0
      # ... additional joint mappings

  - id: "websocket_direct"
    enabled: true
    priority: 200
    input:
      source: "websocket"
      command_types: ["move", "home"]
    output:
      target: "*"  # Any target specified in command
    mapping: "passthrough"
```

### Priority and Blending

When multiple inputs target the same output:

1. **Priority Override**: Higher priority completely overrides lower
2. **Blending**: Optional smooth blending between sources
3. **Timeout**: Sources timeout after configurable period of no data

```yaml
blending:
  mode: "priority"          # "priority", "blend", "additive"
  blend_time_ms: 200        # Smooth transition time
  source_timeout_ms: 500    # Consider source inactive after this
```

### Router State Machine

```
┌─────────────┐
│   IDLE      │◄──── No active inputs
└──────┬──────┘
       │ Input received
       ▼
┌─────────────┐
│  ROUTING    │◄──── Processing inputs, applying mappings
└──────┬──────┘
       │
       ├─── Single source ──► Direct output
       │
       └─── Multiple sources
                │
                ▼
       ┌─────────────┐
       │  BLENDING   │◄──── Combining based on priority/mode
       └──────┬──────┘
              │
              ▼
         Output to node topic
```

### WebSocket Router Commands

**List Routes:**
```json
{
  "type": "router",
  "action": "list_routes",
  "params": {}
}
```

**Create/Update Route:**
```json
{
  "type": "router",
  "action": "set_route",
  "params": {
    "route": {
      "id": "my_custom_route",
      "enabled": true,
      "priority": 150,
      "input": {
        "source": "livelink",
        "subject": "MyCharacter",
        "type": "animation"
      },
      "output": {
        "target": "head"
      },
      "mapping": [
        {"from": "Head", "to": "pan", "scale": 1.0}
      ]
    }
  }
}
```

**Delete Route:**
```json
{
  "type": "router",
  "action": "delete_route",
  "params": {
    "route_id": "my_custom_route"
  }
}
```

**Enable/Disable Route:**
```json
{
  "type": "router",
  "action": "set_route_enabled",
  "params": {
    "route_id": "facecap_to_head",
    "enabled": false
  }
}
```

**List LiveLink Sources:**
```json
{
  "type": "router",
  "action": "list_livelink_sources",
  "params": {}
}
```

**Response:**
```json
{
  "type": "response",
  "status": "ok",
  "data": {
    "sources": [
      {
        "subject_name": "FaceCapture",
        "subject_type": "face",
        "source_ip": "192.168.1.50",
        "last_frame": 1705312800.123,
        "frame_rate": 60.0,
        "properties": ["headYaw", "headPitch", "headRoll", "jawOpen", "..."]
      },
      {
        "subject_name": "UE_Character",
        "subject_type": "animation",
        "source_ip": "192.168.1.100",
        "last_frame": 1705312800.125,
        "frame_rate": 30.0,
        "properties": ["Root", "Spine", "Head", "LeftArm.shoulder", "..."]
      }
    ]
  }
}
```

### Web Interface: Router Configuration

The management web interface includes a visual router configuration:

1. **Source Panel**: Shows connected LiveLink sources with live data preview
2. **Route Builder**: Drag-and-drop interface to create mappings
3. **Output Preview**: Real-time visualization of output values
4. **Preset Library**: Save/load routing configurations

```
┌─────────────────────────────────────────────────────────────────────┐
│  Router Configuration                                                │
├─────────────────────────────────────────────────────────────────────┤
│                                                                      │
│  SOURCES                    ROUTES                      OUTPUTS     │
│  ┌──────────────┐          ┌──────────────┐          ┌───────────┐ │
│  │ FaceCapture  │───┐      │ Route 1      │     ┌───►│ Head      │ │
│  │ [CONNECTED]  │   │      │ Face→Head    │─────┤    │ pan: 45°  │ │
│  │ 60 fps       │   └─────►│ Priority:100 │     │    │ tilt: -10°│ │
│  └──────────────┘          └──────────────┘     │    └───────────┘ │
│                                                  │                   │
│  ┌──────────────┐          ┌──────────────┐     │    ┌───────────┐ │
│  │ UE_Character │───┐      │ Route 2      │     │    │ Left Arm  │ │
│  │ [CONNECTED]  │   │      │ Anim→Arms    │─────┼───►│ j0: 30°   │ │
│  │ 30 fps       │   └─────►│ Priority:100 │     │    │ j1: -45°  │ │
│  └──────────────┘          └──────────────┘     │    └───────────┘ │
│                                                  │                   │
│  ┌──────────────┐          ┌──────────────┐     │    ┌───────────┐ │
│  │ WebSocket    │──────────│ Direct Ctrl  │─────┴───►│ Tracks    │ │
│  │ [2 clients]  │          │ Priority:200 │          │ lin: 0.0  │ │
│  └──────────────┘          └──────────────┘          └───────────┘ │
│                                                                      │
│  [+ Add Route]  [Save Preset]  [Load Preset]                        │
└─────────────────────────────────────────────────────────────────────┘
```

---

## Node Specifications

### Main Server Node

**Responsibilities:**
- Firmware storage and distribution (single generic image)
- Node adoption and management
- **LiveLink receiver and subject registry**
- **Input/Output routing and blending**
- WebSocket server for external controller connections
- Command routing between WebSocket and ROS2 topics
- System health monitoring
- Logging and diagnostics

**Services:**
- `/saint/firmware/check` - Check for firmware updates
- `/saint/firmware/download` - Download firmware package
- `/saint/nodes/adopt` - Adopt an unadopted node
- `/saint/nodes/reset` - Reset a node to unadopted state
- `/saint/system/status` - Get overall system status
- `/saint/system/shutdown` - Initiate system shutdown

**Subscribed Topics:**
- `/saint/nodes/unadopted` - Unadopted node announcements
- `/saint/nodes/+/status` - Status from all nodes

### Head Node

**Topic**: `/saint/head`

**Capabilities:**
- Pan/tilt/roll articulation
- Camera streaming
- Sensor data (proximity, ambient light, etc.)
- Expression/display control (if equipped)

**Published Topics:**
- `/saint/head/state` - Current head position and status
- `/saint/head/camera/image_raw` - Camera feed
- `/saint/head/sensors` - Sensor readings

**Subscribed Topics:**
- `/saint/head/cmd` - Head movement commands

**Message Types:**
```
# HeadState.msg
float32 pan      # -180 to 180 degrees
float32 tilt     # -90 to 90 degrees
float32 roll     # -45 to 45 degrees
bool camera_active
```

```
# HeadCommand.msg
float32 pan
float32 tilt
float32 roll
float32 speed    # 0.0 to 1.0
```

### Arms Node

**Topic**: `/saint/arms`

**Capabilities:**
- Dual arm kinematics (left/right)
- Gripper control
- Force/torque feedback
- Collision detection

**Published Topics:**
- `/saint/arms/left/state` - Left arm joint states
- `/saint/arms/right/state` - Right arm joint states
- `/saint/arms/left/gripper/state` - Left gripper state
- `/saint/arms/right/gripper/state` - Right gripper state

**Subscribed Topics:**
- `/saint/arms/left/cmd` - Left arm commands
- `/saint/arms/right/cmd` - Right arm commands
- `/saint/arms/left/gripper/cmd` - Left gripper commands
- `/saint/arms/right/gripper/cmd` - Right gripper commands

**Message Types:**
```
# ArmState.msg
float32[] joint_positions    # Array of joint angles
float32[] joint_velocities
float32[] joint_efforts
bool in_motion
bool error_state
string error_message
```

```
# ArmCommand.msg
float32[] target_positions
float32 speed_factor         # 0.0 to 1.0
bool immediate               # Override current motion
```

```
# GripperCommand.msg
float32 position             # 0.0 (closed) to 1.0 (open)
float32 force                # Grip force limit
```

### Tracks Node

**Topic**: `/saint/tracks`

**Capabilities:**
- Differential track drive
- Odometry calculation
- Motor current monitoring
- Emergency stop

**Published Topics:**
- `/saint/tracks/odom` - Odometry data (nav_msgs/Odometry)
- `/saint/tracks/state` - Track system state
- `/saint/tracks/motor_current` - Motor current readings

**Subscribed Topics:**
- `/saint/tracks/cmd_vel` - Velocity commands (geometry_msgs/Twist)
- `/saint/tracks/estop` - Emergency stop trigger

**Message Types:**
```
# TrackState.msg
float32 left_velocity
float32 right_velocity
float32 left_current
float32 right_current
bool estop_active
bool motors_enabled
```

### Console Node

**Topic**: `/saint/console`

**Capabilities:**
- Local user interface
- Status display
- Manual override controls
- Diagnostic output

**Published Topics:**
- `/saint/console/input` - User input events
- `/saint/console/override` - Manual control overrides

**Subscribed Topics:**
- `/saint/console/display` - Display content commands
- `/saint/console/alerts` - Alert messages

---

## Firmware Update System

### Single Firmware Model

Since all nodes run the same generic firmware, the update system is simplified:
- **One firmware image** for all nodes regardless of assigned role
- Server stores the current firmware version
- Nodes check for updates on boot, before checking adoption status
- Role-specific configurations are separate from firmware

### Update Protocol

1. **Node Startup Sequence:**
   ```
   Node Boot
       │
       ▼
   Initialize Core Hardware
       │
       ▼
   Connect to ROS2 Network
       │
       ▼
   Call /saint/firmware/check service
       │
       ├─── No Update ─────────────────────┐
       │                                    │
       └─── Update Available                │
                │                           │
                ▼                           │
           Call /saint/firmware/download    │
                │                           │
                ▼                           │
           Verify Checksum                  │
                │                           │
                ▼                           │
           Apply Update                     │
                │                           │
                ▼                           │
           Reboot Node                      │
                                            │
       ┌────────────────────────────────────┘
       ▼
   Check Local Role Storage
       │
       ├─── No Role Stored ──► Enter UNADOPTED state
       │                       Publish to /saint/nodes/unadopted
       │                       Await adoption
       │
       └─── Role Found
                │
                ▼
           Validate Role Config with Server
                │
                ├─── Config Valid ──► Enter ACTIVE state
                │                     Start role operation
                │
                └─── Config Invalid/Outdated
                          │
                          ▼
                     Download Updated Config
                          │
                          ▼
                     Enter ACTIVE state
   ```

2. **Firmware Package Structure:**
   ```
   firmware/
   ├── manifest.json              # Version info, checksums
   ├── current/
   │   ├── saint_node.img         # Generic node firmware image
   │   ├── saint_node.img.sha256  # Checksum
   │   └── changelog.md           # Version changelog
   ├── archive/
   │   ├── v1.1.0/
   │   └── v1.0.0/
   └── configs/                   # Role-specific configurations
       ├── head.yaml
       ├── arms_left.yaml
       ├── arms_right.yaml
       ├── tracks.yaml
       └── console.yaml
   ```

3. **Manifest Format:**
   ```json
   {
     "firmware": {
       "version": "1.2.0",
       "release_date": "2026-01-15",
       "checksum_sha256": "abc123...",
       "min_hardware_rev": "1.0",
       "changelog": "Added smooth motion interpolation, improved GPIO reporting",
       "size_bytes": 52428800
     },
     "configs": {
       "head": {"version": "1.2.0", "checksum": "..."},
       "arms_left": {"version": "1.2.0", "checksum": "..."},
       "arms_right": {"version": "1.2.0", "checksum": "..."},
       "tracks": {"version": "1.1.0", "checksum": "..."},
       "console": {"version": "1.2.0", "checksum": "..."}
     },
     "supported_roles": ["head", "arms_left", "arms_right", "tracks", "console"]
   }
   ```

4. **Role Configuration Format:**
   ```yaml
   # head.yaml
   role: head
   version: "1.2.0"

   gpio_mapping:
     pan_servo:
       pin: 12
       type: pwm
       frequency: 50
     tilt_servo:
       pin: 13
       type: pwm
       frequency: 50
     roll_servo:
       pin: 18
       type: pwm
       frequency: 50

   i2c_devices:
     - bus: 1
       address: 0x68
       driver: mpu6050
       purpose: imu

   camera:
     enabled: true
     resolution: [640, 480]
     framerate: 30

   limits:
     pan: [-180, 180]
     tilt: [-90, 90]
     roll: [-45, 45]
     max_speed: 1.0
   ```

---

## WebSocket API

### Connection
- **URL**: `ws://<server-ip>:9090/saint`
- **Protocol**: JSON-RPC 2.0 style messages

### Authentication
```json
{
  "type": "auth",
  "token": "<api_token>",
  "client_id": "controller_001"
}
```

### Message Format

**Command (Client → Server):**
```json
{
  "id": "uuid-string",
  "type": "command",
  "target": "head",
  "action": "move",
  "params": {
    "pan": 45.0,
    "tilt": -15.0,
    "speed": 0.5
  }
}
```

**Response (Server → Client):**
```json
{
  "id": "uuid-string",
  "type": "response",
  "status": "ok",
  "data": {}
}
```

**State Update (Server → Client):**
```json
{
  "type": "state",
  "node": "head",
  "timestamp": 1705312800.123,
  "data": {
    "pan": 44.8,
    "tilt": -14.9,
    "roll": 0.0,
    "camera_active": true
  }
}
```

### Subscription Model
```json
{
  "type": "subscribe",
  "topics": ["head", "arms", "tracks"],
  "rate_hz": 10
}
```

```json
{
  "type": "unsubscribe",
  "topics": ["arms"]
}
```

### Available Commands

**Robot Control Commands:**
| Target | Action | Parameters |
|--------|--------|------------|
| head | move | pan, tilt, roll, speed |
| head | home | - |
| head | camera | enable/disable |
| arms | move | arm (left/right), positions[], speed |
| arms | home | arm (left/right/both) |
| arms | gripper | arm, position, force |
| tracks | drive | linear, angular |
| tracks | stop | - |
| tracks | estop | enable/disable |
| system | status | - |
| system | shutdown | - |
| system | reboot | node (optional) |

**Node Management Commands:**
| Target | Action | Parameters |
|--------|--------|------------|
| management | list_unadopted | - |
| management | list_adopted | - |
| management | get_gpio_status | node_id |
| management | adopt_node | node_id, role, instance (optional) |
| management | reset_node | node_id, factory_reset (bool) |
| management | get_node_info | node_id |
| management | set_node_name | node_id, display_name |
| management | firmware_status | - |
| management | trigger_update | node_id (optional, all if omitted) |

**Router Commands:**
| Target | Action | Parameters |
|--------|--------|------------|
| router | list_routes | - |
| router | set_route | route (object) |
| router | delete_route | route_id |
| router | set_route_enabled | route_id, enabled (bool) |
| router | list_livelink_sources | - |
| router | get_livelink_subject | subject_name |
| router | list_presets | - |
| router | load_preset | preset_name |
| router | save_preset | preset_name, routes (optional, current if omitted) |
| router | delete_preset | preset_name |

**RC Controller Commands:**
| Target | Action | Parameters |
|--------|--------|------------|
| rc | get_status | - |
| rc | set_enabled | enabled (bool) |
| rc | get_channel_config | channel (optional, all if omitted) |
| rc | set_channel_config | channel, name, min, max, center, reversed |
| rc | calibrate_channel | channel, position ("min", "max", "center") |
| rc | set_failsafe | action ("hold", "neutral", "estop", "passthrough") |
| rc | get_protocol | - |
| rc | set_protocol | protocol ("pwm", "ppm", "sbus", "ibus", "crsf") |

---

## Network Configuration

### Network Architecture

The robot has two network interfaces:
1. **Internal Ethernet**: Dedicated wired network connecting server and all nodes
2. **External WiFi**: Connection to external systems (admin interface, client apps, LiveLink sources)

```
    EXTERNAL SYSTEMS (WiFi Network)
    ═══════════════════════════════════════════════════════════════════

    ┌─────────────────┐   ┌─────────────────┐   ┌─────────────────┐
    │  Admin Web UI   │   │  Client App     │   │ Unreal Engine / │
    │  (Browser)      │   │  (Controller)   │   │ LiveLink Face   │
    │  192.168.1.x    │   │  192.168.1.x    │   │  192.168.1.x    │
    └────────┬────────┘   └────────┬────────┘   └────────┬────────┘
             │ HTTP/WS             │ WebSocket           │ LiveLink
             └─────────────────────┴─────────────────────┘
                                   │
                                   │ WiFi (192.168.1.0/24)
                                   │
    ═══════════════════════════════╪═══════════════════════════════════
                                   │
    ROBOT CHASSIS                  │
    ┌──────────────────────────────┴──────────────────────────────────┐
    │                              ▼                                   │
    │                     ┌────────────────┐                          │
    │                     │  WiFi Adapter  │                          │
    │                     │ (wlan0)        │                          │
    │                     └───────┬────────┘                          │
    │                             │                                    │
    │   ┌─────────────────────────┴─────────────────────────┐         │
    │   │              MAIN SERVER (Pi)                      │         │
    │   │  ┌──────────┐  ┌──────────┐  ┌──────────────────┐ │         │
    │   │  │ WebSocket│  │ LiveLink │  │ Management       │ │         │
    │   │  │ Server   │  │ Receiver │  │ Web Server       │ │         │
    │   │  └──────────┘  └──────────┘  └──────────────────┘ │         │
    │   │  ┌─────────────────────────────────────────────┐  │         │
    │   │  │         Input/Output Router                 │  │         │
    │   │  └─────────────────────────────────────────────┘  │         │
    │   │  ┌──────────┐  ┌──────────┐  ┌──────────────────┐ │         │
    │   │  │ Firmware │  │ Adoption │  │ ROS2 Node        │ │         │
    │   │  │ Manager  │  │ Manager  │  │ (topics/services)│ │         │
    │   │  └──────────┘  └──────────┘  └──────────────────┘ │         │
    │   │              eth0: 192.168.10.1                   │         │
    │   └─────────────────────────┬─────────────────────────┘         │
    │                             │                                    │
    │                    Internal Ethernet                             │
    │                    192.168.10.0/24                               │
    │                             │                                    │
    │            ┌────────────────┼────────────────┐                   │
    │            │                │                │                   │
    │   ┌────────┴───────┐ ┌─────┴──────┐ ┌──────┴───────┐            │
    │   │   Head Node    │ │ Arms Node  │ │ Tracks Node  │ ...        │
    │   │ 192.168.10.11  │ │192.168.10.12│ │192.168.10.13│            │
    │   └────────────────┘ └────────────┘ └──────────────┘            │
    │                                                                  │
    └──────────────────────────────────────────────────────────────────┘
```

### Internal Network (Ethernet)

- **Purpose**: Low-latency, reliable communication between server and nodes
- **Medium**: Wired Ethernet (Cat5e/Cat6)
- **Topology**: Star topology via switch or direct connections
- **Traffic**: ROS2 DDS, firmware updates, node adoption

### External Network (WiFi)

- **Purpose**: Remote access for administration and control
- **Medium**: WiFi (2.4GHz or 5GHz)
- **Clients**:
  - Web browsers accessing management UI
  - Client controller applications (WebSocket)
  - Unreal Engine (LiveLink)
  - LiveLink Face capture apps (iOS/Android)

### Network Configuration

| Parameter | Value | Notes |
|-----------|-------|-------|
| Internal Subnet | 192.168.10.0/24 | Dedicated robot network |
| Server IP | 192.168.10.1 | Static, DHCP server |
| Node IPs | 192.168.10.10-254 | DHCP assigned by server |
| External Interface | Configurable | WiFi or secondary Ethernet |

### Default Ports
| Service | Port | Protocol | Network |
|---------|------|----------|---------|
| ROS2 DDS Discovery | 7400 | UDP | Internal |
| ROS2 DDS Data | 7401-7500 | UDP | Internal |
| Firmware HTTP | 8080 | TCP | Internal |
| WebSocket Server | 9090 | TCP | External |
| LiveLink Discovery | 54321 | UDP (multicast) | External |
| LiveLink Data | 54322 | TCP | External |
| Management Web UI | 80 | HTTP | External |

### Node Discovery
- Server runs DHCP on internal network (dnsmasq or similar)
- Nodes receive IP via DHCP on boot
- ROS2 automatic discovery via DDS on internal network
- Main server hostname: `saint-server.local` (mDNS)
- Nodes register with server after DHCP lease

### LiveLink Network Setup
- Server broadcasts LiveLink availability via UDP multicast on **external** network
- Unreal Engine / LiveLink apps discover server automatically
- TCP connection established for reliable frame data
- Multiple simultaneous LiveLink sources supported
- LiveLink traffic bridged to internal ROS2 topics via router

---

## Directory Structure

```
saint_os/
├── CMakeLists.txt                    # Unified build configuration
├── package.xml
├── setup.py
│
├── config/
│   ├── server.yaml                   # Server configuration
│   ├── livelink.yaml                 # LiveLink receiver settings
│   ├── rc_receiver.yaml              # RC controller receiver settings
│   ├── routes/                       # Router configuration presets
│   │   ├── default.yaml              # Default routing rules
│   │   ├── facecap_head.yaml         # Face capture → head preset
│   │   ├── ue_fullbody.yaml          # Unreal full body animation preset
│   │   ├── rc_manual.yaml            # RC transmitter manual control preset
│   │   └── manual_override.yaml      # WebSocket-only control preset
│   └── roles/                        # Role-specific configs (deployed to nodes)
│       ├── head.yaml
│       ├── arms_left.yaml
│       ├── arms_right.yaml
│       ├── tracks.yaml
│       └── console.yaml
│
├── firmware/                         # Built firmware output (generated)
│   ├── manifest.json
│   ├── current/
│   │   └── saint_node.img
│   └── configs/
│
├── launch/
│   ├── server.launch.py              # Launch main server
│   ├── node_sim.launch.py            # Launch simulated node for testing
│   └── full_system_sim.launch.py     # Launch full simulated system
│
├── msg/
│   ├── NodeGPIOStatus.msg            # GPIO status for unadopted nodes
│   ├── GPIOPin.msg                   # Individual GPIO pin state
│   ├── PeripheralInfo.msg            # Detected peripheral info
│   ├── NodeAnnouncement.msg          # Unadopted node announcement
│   ├── LiveLinkFrame.msg             # Incoming LiveLink animation frame
│   ├── LiveLinkSubject.msg           # LiveLink subject info
│   ├── RCReceiverState.msg           # RC receiver status and all channels
│   ├── RCChannelState.msg            # Individual RC channel state
│   ├── RouteStatus.msg               # Current routing state
│   ├── HeadState.msg
│   ├── HeadCommand.msg
│   ├── ArmState.msg
│   ├── ArmCommand.msg
│   ├── GripperCommand.msg
│   ├── GripperState.msg
│   ├── TrackState.msg
│   ├── ConsoleInput.msg
│   └── SystemStatus.msg
│
├── srv/
│   ├── FirmwareCheck.srv             # Check for firmware updates
│   ├── FirmwareDownload.srv          # Download firmware
│   ├── AdoptNode.srv                 # Adopt an unadopted node
│   ├── ResetNode.srv                 # Reset node to unadopted state
│   ├── GPIOProbe.srv                 # Probe GPIO on a node
│   └── SystemCommand.srv             # System-wide commands
│
├── saint_server/                     # Server package
│   ├── __init__.py
│   ├── server_node.py                # Main server ROS2 node
│   ├── firmware_manager.py           # Firmware storage and distribution
│   ├── adoption_manager.py           # Node adoption state machine
│   ├── websocket_server.py           # WebSocket gateway
│   ├── management_api.py             # Management interface handlers
│   ├── livelink/                     # Unreal LiveLink integration
│   │   ├── __init__.py
│   │   ├── receiver.py               # LiveLink protocol receiver
│   │   ├── subject_registry.py       # Track connected LiveLink subjects
│   │   ├── frame_parser.py           # Parse animation/face/transform frames
│   │   └── discovery.py              # UDP multicast discovery broadcast
│   ├── rc_receiver/                  # RC controller receiver (optional)
│   │   ├── __init__.py
│   │   ├── rc_manager.py             # RC receiver manager and state
│   │   ├── protocols/                # Protocol implementations
│   │   │   ├── __init__.py
│   │   │   ├── pwm_reader.py         # PWM signal reader (pigpio)
│   │   │   ├── ppm_reader.py         # PPM sum signal decoder
│   │   │   ├── sbus_reader.py        # Futaba SBUS protocol
│   │   │   ├── ibus_reader.py        # FlySky IBUS protocol
│   │   │   └── crsf_reader.py        # Crossfire/ELRS protocol
│   │   ├── calibration.py            # Channel calibration utilities
│   │   └── failsafe.py               # Failsafe behavior handling
│   └── router/                       # Input/Output routing system
│       ├── __init__.py
│       ├── router_core.py            # Main routing engine
│       ├── route_config.py           # Route definition and storage
│       ├── input_sources.py          # Input source abstraction
│       ├── output_targets.py         # Output target abstraction
│       ├── blending.py               # Priority/blend logic
│       └── presets.py                # Save/load routing presets
│
├── saint_node/                       # Generic node package (compiled to firmware)
│   ├── __init__.py
│   ├── node_main.py                  # Node entry point and state machine
│   ├── core/
│   │   ├── __init__.py
│   │   ├── boot_manager.py           # Boot sequence and update check
│   │   ├── gpio_monitor.py           # GPIO abstraction and reporting
│   │   ├── adoption_client.py        # Handles adoption from server
│   │   ├── firmware_client.py        # Firmware update client
│   │   └── role_loader.py            # Dynamic role activation
│   ├── roles/
│   │   ├── __init__.py
│   │   ├── base_role.py              # Abstract base class for roles
│   │   ├── head_role.py              # Head articulation implementation
│   │   ├── arms_role.py              # Arm control implementation
│   │   ├── tracks_role.py            # Track drive implementation
│   │   └── console_role.py           # Console interface implementation
│   └── drivers/
│       ├── __init__.py
│       ├── servo_driver.py           # PWM servo control
│       ├── motor_driver.py           # DC/stepper motor control
│       ├── encoder_driver.py         # Rotary encoder input
│       ├── camera_driver.py          # Camera capture (Pi Camera)
│       ├── display_driver.py         # LCD/OLED output
│       └── gpio_hal.py               # GPIO hardware abstraction layer
│
├── saint_common/                     # Shared code between server and node
│   ├── __init__.py
│   ├── constants.py                  # Shared constants and enums
│   ├── protocol.py                   # Message format definitions
│   └── utils.py                      # Common utilities
│
├── scripts/
│   ├── build_firmware.py             # Package node into firmware image
│   ├── deploy_server.sh              # Deploy server to Pi
│   ├── flash_node.sh                 # Flash firmware to node Pi
│   └── simulate_node.py              # Run node in simulation mode
│
├── web/                              # Management web interface
│   ├── index.html
│   ├── css/
│   ├── js/
│   └── assets/
│
└── test/
    ├── test_server.py
    ├── test_node_core.py
    ├── test_adoption.py
    ├── test_roles.py
    ├── test_gpio.py
    └── test_websocket.py
```

### Build Process

```
┌─────────────────────────────────────────────────────────────┐
│                     colcon build                             │
└───────────────────────────┬─────────────────────────────────┘
                            │
            ┌───────────────┴───────────────┐
            ▼                               ▼
┌───────────────────────┐       ┌───────────────────────────┐
│   saint_server        │       │   saint_node              │
│   (install/saint_*)   │       │   (install/saint_*)       │
└───────────────────────┘       └─────────────┬─────────────┘
                                              │
                                              ▼
                                ┌───────────────────────────┐
                                │  scripts/build_firmware.py │
                                │  - Package node + deps     │
                                │  - Create bootable image   │
                                │  - Generate checksums      │
                                └─────────────┬─────────────┘
                                              │
                                              ▼
                                ┌───────────────────────────┐
                                │  firmware/current/        │
                                │  └── saint_node.img       │
                                └───────────────────────────┘
```

---

## Node Identity and Persistence

### Node Identification

Each node is uniquely identified by:
- **Primary**: Raspberry Pi serial number (`/proc/cpuinfo`)
- **Secondary**: MAC address of primary network interface
- **Display name**: User-assigned friendly name (e.g., "Head Unit", "Left Arm")

```
# NodeIdentity stored locally on each node
node_id: "pi-serial-a1b2c3d4"
mac_address: "dc:a6:32:xx:xx:xx"
display_name: "Head Unit"
first_seen: "2026-01-20T14:30:00Z"
```

### Role Persistence

Assigned roles are stored locally on the node:

```
# /var/lib/saint/role.yaml
assigned_role: "head"
assigned_at: "2026-01-20T15:00:00Z"
assigned_by: "admin"
config_version: "1.2.0"
server_address: "192.168.1.100"
```

### Reset Behavior

| Reset Type | Trigger | Effect |
|------------|---------|--------|
| Soft Reset | WebSocket/ROS2 command | Clears role, returns to unadopted |
| Hard Reset | Physical button hold (5s) | Clears role, returns to unadopted |
| Factory Reset | WebSocket command + flag | Clears role AND forces firmware re-download |
| Power Cycle | Power off/on | Retains role, resumes operation |
| Reboot | Software reboot | Retains role, resumes operation |

---

## Development Phases

### Phase 1: Foundation
- [ ] ROS2 workspace and package setup
- [ ] Message and service definitions
- [ ] Common utilities and constants
- [ ] Basic build system configuration

### Phase 2: Server Core
- [ ] Server node skeleton with ROS2 integration
- [ ] Firmware storage and manifest management
- [ ] WebSocket server with basic commands
- [ ] Node discovery (subscribe to unadopted announcements)

### Phase 3: Generic Node Core
- [ ] Node state machine (boot → unadopted → adopting → active)
- [ ] GPIO hardware abstraction layer
- [ ] GPIO status reporting
- [ ] Firmware update client
- [ ] Adoption client

### Phase 4: Adoption System
- [ ] Server-side adoption manager
- [ ] Node-side adoption handling
- [ ] Role configuration download and validation
- [ ] Role persistence and resume on reboot
- [ ] Reset functionality (soft/hard/factory)

### Phase 5: Role Implementations
- [ ] Base role abstract class
- [ ] Head role (servo control, camera)
- [ ] Arms role (multi-joint control, grippers)
- [ ] Tracks role (differential drive, odometry)
- [ ] Console role (display, input)

### Phase 6: Management Interface
- [ ] Web UI for node management
- [ ] GPIO visualization
- [ ] Role assignment interface
- [ ] System status dashboard

### Phase 7: Hardware Integration
- [ ] Firmware image build script
- [ ] Raspberry Pi deployment and flashing
- [ ] Real hardware driver testing
- [ ] Performance optimization for Pi

### Phase 8: Controller Integration
- [ ] WebSocket client library (Python)
- [ ] Example controller application
- [ ] API documentation
- [ ] Integration testing

---

## Dependencies

### ROS2 Packages
- `rclpy` - ROS2 Python client library
- `std_msgs` - Standard message types
- `geometry_msgs` - Geometry message types
- `nav_msgs` - Navigation messages (odometry)
- `sensor_msgs` - Sensor messages (camera, IMU)

### Python Packages (Server)
- `websockets` - WebSocket server implementation
- `aiohttp` - Async HTTP for firmware serving
- `pyyaml` - Configuration parsing
- `numpy` - Numerical operations

### Python Packages (Node)
- `RPi.GPIO` - Raspberry Pi GPIO control
- `pigpio` - Advanced GPIO (PWM, hardware timing)
- `smbus2` - I2C communication
- `spidev` - SPI communication
- `picamera2` - Pi Camera interface
- `pyyaml` - Configuration parsing

### System Requirements

**Server:**
- Raspberry Pi 4 (4GB+ RAM recommended)
- Ubuntu 22.04 Server (64-bit) or Raspberry Pi OS (64-bit)
- ROS2 Humble Hawksbill
- 32GB+ SD card (for firmware storage)

**Nodes:**
- Raspberry Pi 4 (2GB+ RAM)
- Ubuntu 22.04 Server (64-bit) or Raspberry Pi OS (64-bit)
- ROS2 Humble Hawksbill
- 16GB+ SD card

### Web Interface
- Static HTML/CSS/JS (no server-side rendering)
- Served directly by aiohttp on the server
- No additional dependencies required

---

## Revision History

| Version | Date | Author | Changes |
|---------|------|--------|---------|
| 0.1.0 | 2026-01-23 | - | Initial specification |
| 0.2.0 | 2026-01-23 | - | Redesigned to generic node architecture with adoption system. Single firmware for all nodes, role assignment via management interface, GPIO reporting for unadopted nodes. |
| 0.3.0 | 2026-01-23 | - | Added Unreal LiveLink integration as animation input source. Added Input/Output Router for configurable mapping between LiveLink/WebSocket inputs and robot outputs. Defined dual-network architecture: internal Ethernet for nodes, external WiFi for admin/client/LiveLink access. |
| 0.4.0 | 2026-01-23 | - | Added optional RC controller receiver support. Supports PWM, PPM, SBUS, IBUS, and CRSF protocols. RC channels mappable to robot outputs via administration interface. Added failsafe behavior configuration. |
| 0.5.0 | 2026-01-23 | - | Expanded Web Administration Interface specification. Detailed all admin pages: Dashboard (activity/status), Nodes (adoption/monitoring), Routes (mapping config), Inputs (source monitoring), Settings, and Logs. Added wireframes and UI flows. |
