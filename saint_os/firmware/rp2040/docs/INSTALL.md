# SAINT.OS Node Firmware Installation

## Adafruit Feather RP2040 + Ethernet FeatherWing

This guide covers building and installing the SAINT.OS node firmware on an Adafruit Feather RP2040 with the Ethernet FeatherWing adapter.

The firmware uses **micro-ROS** to communicate with the SAINT.OS server as a native ROS2 node over UDP.

### Hardware Requirements

- [Adafruit Feather RP2040](https://www.adafruit.com/product/4884)
- [Adafruit Ethernet FeatherWing](https://www.adafruit.com/product/3201) (W5500-based)
- Feather stacking headers or direct soldering
- Ethernet cable
- USB-C cable for programming

### Hardware Assembly

1. **Solder headers** to the Feather RP2040 if not pre-assembled
2. **Stack the Ethernet FeatherWing** on top of the Feather RP2040
3. The FeatherWing uses the following pins:
   - **SPI**: SCK (GPIO18), MOSI (GPIO19), MISO (GPIO20)
   - **CS**: GPIO10
   - **Reset**: GPIO11

---

## Building the Firmware

### Prerequisites

Install the following on your development machine:

1. **Pico SDK**
   ```bash
   cd ~
   git clone https://github.com/raspberrypi/pico-sdk.git
   cd pico-sdk
   git submodule update --init
   export PICO_SDK_PATH=~/pico-sdk
   ```

2. **ARM Toolchain**
   ```bash
   # Ubuntu/Debian
   sudo apt install cmake gcc-arm-none-eabi libnewlib-arm-none-eabi

   # macOS
   brew install cmake armmbed/formulae/arm-none-eabi-gcc
   ```

3. **micro-ROS for Pico**
   ```bash
   cd ~
   git clone https://github.com/micro-ROS/micro_ros_raspberrypi_pico_sdk.git
   export MICRO_ROS_PATH=~/micro_ros_raspberrypi_pico_sdk
   ```

4. **WIZnet ioLibrary** (for W5500 Ethernet and DHCP)

   The ioLibrary is included as a git submodule. Initialize it after cloning:
   ```bash
   cd saint_os/firmware/rp2040
   git submodule update --init lib/ioLibrary_Driver
   ```

5. **SAINT.OS Messages**

   The firmware needs the SAINT.OS message definitions. Build them first:
   ```bash
   cd /path/to/SaintOS/source
   colcon build --packages-select saint_os
   ```

### Build Steps

The firmware has two build modes:
- **Hardware** (`SIMULATION=OFF`) - For real RP2040 boards with W5500 Ethernet, USB serial output
- **Simulation** (`SIMULATION=ON`) - For Renode simulation with UDP transport, UART serial output

#### Building for Hardware (Real RP2040)

```bash
cd saint_os/firmware/rp2040

# Create hardware build directory
mkdir -p build_hardware && cd build_hardware

# Configure for hardware
# The node will automatically discover the SAINT server via UDP broadcast
cmake .. \
  -DPICO_SDK_PATH=$PICO_SDK_PATH \
  -DSIMULATION=OFF

# Build
make -j4
```

Output: `build_hardware/saint_node.uf2`

**Automatic Server Discovery:**

The firmware automatically discovers the SAINT server on the local network:
1. Node broadcasts a discovery request to UDP port 8889
2. SAINT server responds with its IP address and agent port
3. Node connects to the discovered server

No hardcoded IP addresses required!

**Optional Fallback Configuration:**

If DHCP fails, the node uses a static IP fallback. You can customize these at build time:

```bash
cmake .. -DSIMULATION=OFF \
  -DNODE_IP="192.168.1.200" \
  -DGATEWAY_IP="192.168.1.1"
```

| Option | Default | Description |
|--------|---------|-------------|
| `NODE_IP` | 192.168.1.200 | Static IP if DHCP fails |
| `GATEWAY_IP` | 192.168.1.1 | Gateway if DHCP fails |

#### Building for Simulation (Renode)

```bash
cd saint_os/firmware/rp2040

# Create simulation build directory
mkdir -p build_sim && cd build_sim

# Configure for simulation
cmake .. \
  -DPICO_SDK_PATH=$PICO_SDK_PATH \
  -DSIMULATION=ON

# Build
make -j4
```

Output: `build_sim/saint_node.elf` (used by Renode)

#### Build Directory Convention

| Directory | Mode | Serial Output | Network | Use Case |
|-----------|------|---------------|---------|----------|
| `build_hardware/` | SIMULATION=OFF | USB | W5500 Ethernet | Real hardware |
| `build_sim/` | SIMULATION=ON | UART0 | UDP Bridge | Renode simulation |

**Note:** Avoid using a generic `build/` directory - always use `build_hardware/` or `build_sim/` to prevent confusion.

#### Output Files

- `saint_node.uf2` - Drag-and-drop firmware file (hardware only)
- `saint_node.elf` - ELF binary with debug symbols
- `saint_node.bin` - Raw binary

---

## Flashing the Firmware

### Method 1: UF2 Drag-and-Drop (Recommended)

1. **Enter bootloader mode**:
   - Hold the **BOOTSEL** button on the Feather
   - Press and release the **RESET** button
   - Release **BOOTSEL** after the USB drive appears

2. A drive named `RPI-RP2` will appear on your computer

3. Drag `build_hardware/saint_node.uf2` onto the `RPI-RP2` drive

4. The board will automatically reboot with the new firmware

### Method 2: Picotool

```bash
# Install picotool
sudo apt install picotool  # or brew install picotool

# Flash (board must be in bootloader mode)
picotool load build_hardware/saint_node.uf2
picotool reboot
```

### Method 3: OpenOCD (for debugging)

```bash
openocd -f interface/cmsis-dap.cfg -f target/rp2040.cfg \
  -c "program build_hardware/saint_node.elf verify reset exit"
```

---

## Configuration

The firmware uses compile-time configuration. Edit `include/saint_node.h` before building to change:

### Network Settings

```c
// Default server IP (micro-ROS agent)
// Can be changed at runtime via adoption
g_node.server_ip[0] = 192;
g_node.server_ip[1] = 168;
g_node.server_ip[2] = 1;
g_node.server_ip[3] = 10;
g_node.server_port = 8888;

// DHCP vs Static IP
g_node.use_dhcp = true;  // Set false for static
```

### Hardware Pins

The pin definitions in `include/saint_node.h` match the Feather RP2040 + Ethernet FeatherWing:

```c
// SPI pins for W5500 Ethernet
#define ETH_PIN_SCK     18
#define ETH_PIN_MOSI    19
#define ETH_PIN_MISO    20
#define ETH_PIN_CS      10
#define ETH_PIN_RST     11

// Available GPIO for peripherals
#define GPIO_A0         26  // ADC0
#define GPIO_A1         27  // ADC1
#define GPIO_D5         5
#define GPIO_D6         6
// ... etc
```

---

## Server Setup

The SAINT.OS server must run the **micro-ROS agent** to communicate with microcontroller nodes.

### Using the Launch File (Recommended)

```bash
# Start server with micro-ROS agent
ros2 launch saint_os saint_server.launch.py

# Or with custom settings
ros2 launch saint_os saint_server.launch.py \
  server_name:=SAINT-01 \
  web_port:=80 \
  agent_port:=8888
```

### Running Agent Separately

```bash
# Terminal 1: Start server
ros2 run saint_os saint_server

# Terminal 2: Start micro-ROS agent
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888
```

### Installing micro-ROS Agent

```bash
# Ubuntu with ROS2 Humble
sudo apt install ros-humble-micro-ros-agent

# Or build from source
mkdir -p ~/microros_ws/src
cd ~/microros_ws/src
git clone -b humble https://github.com/micro-ROS/micro-ROS-Agent.git
cd ~/microros_ws
colcon build
source install/setup.bash
```

---

## Testing the Node

### Hardware Testing

1. **Connect hardware**
   - Ethernet cable from FeatherWing to network
   - USB cable for power (and serial debugging)

2. **Monitor USB serial output** (hardware build uses USB for stdio)
   ```bash
   # macOS
   screen /dev/tty.usbmodem* 115200

   # Linux
   screen /dev/ttyACM0 115200

   # Or use minicom
   minicom -D /dev/ttyACM0 -b 115200
   ```

### Simulation Testing

For simulation testing with Renode, see [SIMULATION.md](./SIMULATION.md).

Quick start:
```bash
# Start Renode with the simulation script
cd saint_os/firmware/rp2040/simulation/renode_rp2040
renode run_saint_node.resc

# In Renode console:
start
```

The simulation build uses UART0 for serial output, which Renode displays in an analyzer window.

3. **Expected output**
   ```
   ========================================
   SAINT.OS Node Firmware v1.0.0
   Hardware: Adafruit Feather RP2040 + Ethernet FeatherWing
   ========================================
   Node ID: rp2040-a1b2c3d4e5f6
   Initializing W5500...
   W5500 detected (PHYCFGR=0xBF)
   MAC: 02:A1:B2:C3:D4:E5
   Connecting to network...
   IP: 192.168.1.100
   Initializing micro-ROS...
   Created ROS2 node: saint_node_rp2040-a1b2c3d4e5f6
   micro-ROS initialized successfully
   Node ready. Waiting for adoption...
   ========================================
   ```

4. **Verify in ROS2**
   ```bash
   # List nodes
   ros2 node list
   # Should show: /saint_node_rp2040-a1b2c3d4e5f6

   # List topics
   ros2 topic list
   # Should show: /saint/nodes/unadopted

   # Monitor announcements
   ros2 topic echo /saint/nodes/unadopted
   ```

5. **Check web interface**
   - Open http://localhost:80 (or your server IP)
   - The node should appear under "Unadopted Nodes"
   - Click "Adopt" and select a role

---

## LED Status Indicators

The onboard NeoPixel indicates node state:

| Color | Pattern | Meaning |
|-------|---------|---------|
| Blue | Solid | Booting / Initializing |
| Yellow | Pulsing | Connecting to network |
| Orange | Pulsing | Unadopted, announcing |
| White | Flashing | Being adopted |
| Green | Solid | Adopted and active |
| Red | Pulsing | Error state |

---

## Troubleshooting

### No ethernet link
- Check SPI wiring between Feather and FeatherWing
- Verify ethernet cable is connected
- Check that link LEDs on FeatherWing are lit

### DHCP fails
- Verify DHCP server on network
- Check subnet configuration
- Try static IP instead

### micro-ROS agent not connecting
- Verify agent is running: `ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888 -v6`
- Check firewall allows UDP port 8888
- Verify node and server are on same network
- Check IP address configuration

### Node not appearing in web interface
- Verify server is running
- Check ROS2 topics: `ros2 topic echo /saint/nodes/unadopted`
- Check agent logs for connection attempts

### Serial console not working
- **Verify you're using the hardware build** - The hardware build (`build_hardware/`) outputs to USB serial. The simulation build (`build_sim/`) outputs to UART0 and won't show anything over USB.
- Try different USB cable (some are charge-only)
- Check device is enumerated: `ls /dev/tty*`
- On macOS: `ls /dev/tty.usbmodem*`
- Wait 2 seconds after boot - the firmware delays startup for USB enumeration

---

## Pin Reference

### Feather RP2040 Pinout

```
                    USB-C
              ┌───────────────┐
       RESET ─┤ RST       BAT ├─ Battery+
         3V3 ─┤ 3V3       GND ├─ Ground
        AREF ─┤ AREF       EN ├─ Enable
   ADC0  A0  ─┤ A0   ┌─┐  USB ├─ USB 5V
   ADC1  A1  ─┤ A1   │ │  D13 ├─ GPIO13 (LED)
   ADC2  A2  ─┤ A2   │ │  D12 ├─ GPIO12
   ADC3  A3  ─┤ A3   │ │  D11 ├─ GPIO11 (ETH_RST) ←
         D24 ─┤ D24  │ │  D10 ├─ GPIO10 (ETH_CS)  ←
         D25 ─┤ D25  └─┘   D9 ├─ GPIO9
         SCK ─┤ SCK        D6 ├─ GPIO6            (ETH) →
        MOSI ─┤ MO         D5 ├─ GPIO5            (ETH) →
        MISO ─┤ MI         TX ├─ GPIO0 (UART TX)  (ETH) →
          RX ─┤ RX         RX ├─ GPIO1 (UART RX)
              └───────────────┘

← = Used by Ethernet FeatherWing
→ = Reserved for SPI (ETH)
```

**Reserved for Ethernet FeatherWing:**
- SCK (GPIO18) - SPI Clock
- MOSI (GPIO19) - SPI Data Out
- MISO (GPIO20) - SPI Data In
- D10 (GPIO10) - Chip Select
- D11 (GPIO11) - Reset

**Available for role-specific hardware:**
- A0-A3 (GPIO26-29) - Analog inputs / Digital I/O
- D5, D6, D9, D12, D13 - Digital I/O / PWM
- D24, D25 - Digital I/O
- TX/RX (GPIO0/1) - UART

---

## Next Steps

After the node appears in the SAINT.OS web interface:

1. **Adopt the node** - Select a role (head, arms, tracks, console)
2. **Wire peripherals** - Connect servos, sensors, motors as needed
3. **Configure GPIO** - Map pins to role functions via web interface

See role-specific documentation:
- [Head Node Setup](./HEAD_NODE.md) - Servos for pan/tilt/roll
- [Arms Node Setup](./ARMS_NODE.md) - Joint servos and gripper
- [Tracks Node Setup](./TRACKS_NODE.md) - Motor drivers and encoders
- [Console Node Setup](./CONSOLE_NODE.md) - Display and buttons
