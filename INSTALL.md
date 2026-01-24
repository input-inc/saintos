# SAINT.OS Installation Guide

This guide covers building SAINT.OS from source on macOS, Linux, and Windows.

## Table of Contents

- [Prerequisites](#prerequisites)
  - [All Platforms](#all-platforms)
  - [macOS](#macos)
  - [Linux (Ubuntu/Debian)](#linux-ubuntudebian)
  - [Windows](#windows)
- [Building SAINT.OS](#building-saintos)
  - [Quick Start](#quick-start)
  - [Build Options](#build-options)
- [Running the Server](#running-the-server)
- [Development Setup](#development-setup)
- [Troubleshooting](#troubleshooting)

---

## Prerequisites

### All Platforms

- **Python 3.10+** (Python 3.11 or 3.12 recommended)
- **ROS2 Humble Hawksbill** (LTS) or later (Iron, Jazzy)
- **colcon** build tools
- **Git**

### macOS

ROS2 does not have official binary packages for macOS. There are three main approaches:

- **Option A: RoboStack with Conda** (Recommended - easiest for Apple Silicon)
- **Option B: Build from source** (Official method, more complex)
- **Option C: Docker or VM** (Run Ubuntu with ROS2)

#### Prerequisites

```bash
# Install Homebrew (if not already installed)
/bin/bash -c "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/HEAD/install.sh)"

# Install essential tools
brew install cmake git wget
```

---

#### Option A: RoboStack with Conda (Recommended)

[RoboStack](https://robostack.github.io/) provides pre-built ROS2 packages via Conda, including support for Apple Silicon (M1/M2/M3).

##### 1. Install Miniforge (Conda for ARM64)

```bash
# Download Miniforge for macOS ARM64
wget https://github.com/conda-forge/miniforge/releases/latest/download/Miniforge3-MacOSX-arm64.sh

# Install
bash Miniforge3-MacOSX-arm64.sh

# Restart terminal or source
source ~/.zshrc  # or ~/.bashrc
```

##### 2. Create ROS2 Environment

```bash
# Create a new conda environment for ROS2
conda create -n ros2_env python=3.11 -y
conda activate ros2_env

# Add RoboStack channels
conda config --env --add channels conda-forge
conda config --env --add channels robostack-staging

# Install ROS2 Humble
conda install ros-humble-desktop -y

# Install colcon build tools
conda install colcon-common-extensions -y
```

##### 3. Activate and Verify

```bash
# Activate the environment (do this each time)
conda activate ros2_env

# Verify installation
ros2 --version
colcon --help
```

**Note:** Always activate the conda environment before building or running SAINT.OS:
```bash
conda activate ros2_env
```

---

#### Option B: Build from Source

For the latest packages or if RoboStack doesn't meet your needs, build ROS2 from source.

##### 1. Install Dependencies

```bash
# Install build dependencies
brew install asio assimp bison bullet cmake console_bridge cppcheck \
  cunit eigen freetype graphviz opencv openssl orocos-kdl pcre poco \
  pyqt@5 python@3.11 qt@5 sip spdlog tinyxml tinyxml2

# Install Python dependencies
pip3 install -U \
  argcomplete catkin_pkg colcon-common-extensions coverage \
  cryptography empy flake8 flake8-blind-except flake8-builtins \
  flake8-class-newline flake8-comprehensions flake8-deprecated \
  flake8-docstrings flake8-import-order flake8-quotes \
  importlib-metadata lark-parser mock mypy netifaces nose \
  pep8 pydocstyle pyparsing pytest pytest-mock pytest-cov \
  pytest-repeat pytest-rerunfailures pytest-runner rosdep \
  setuptools vcstool numpy lxml
```

##### 2. Create Workspace and Download Source

```bash
mkdir -p ~/ros2_humble/src
cd ~/ros2_humble

# Download ROS2 source
vcs import --input https://raw.githubusercontent.com/ros2/ros2/humble/ros2.repos src
```

##### 3. Install Dependencies via rosdep

```bash
sudo rosdep init  # Only if not already done
rosdep update
rosdep install --from-paths src --ignore-src -y --skip-keys "fastcdr rti-connext-dds-6.0.1 urdfdom_headers"
```

##### 4. Build

```bash
cd ~/ros2_humble
colcon build --symlink-install --cmake-args -DBUILD_TESTING=OFF
```

##### 5. Source the Workspace

```bash
# Add to your shell profile (~/.zshrc or ~/.bashrc)
echo "source ~/ros2_humble/install/setup.bash" >> ~/.zshrc
source ~/.zshrc
```

See the [official ROS2 macOS documentation](https://docs.ros.org/en/humble/Installation/Alternatives/macOS-Development-Setup.html) for detailed instructions.

---

#### Option C: Docker or Virtual Machine

If native installation is problematic, run Ubuntu with ROS2 in a container or VM.

##### Docker

```bash
# Install Docker Desktop for Mac
brew install --cask docker

# Run ROS2 Humble container
docker run -it --rm \
  -v $(pwd):/workspace \
  -w /workspace \
  osrf/ros:humble-desktop \
  bash
```

##### UTM Virtual Machine

1. Download [UTM](https://mac.getutm.app/) (free virtualization for Mac)
2. Install Ubuntu 22.04 ARM64
3. Follow the Linux installation instructions below

---

#### Verify Installation (All Options)

```bash
# For Conda: activate environment first
conda activate ros2_env

# Verify ROS2
ros2 --version
ros2 topic list

# Verify colcon
colcon --help
```

---

### Linux (Ubuntu/Debian)

#### 1. Install Python 3.10+

Ubuntu 22.04 comes with Python 3.10. For older versions:

```bash
sudo apt update
sudo apt install python3.10 python3.10-venv python3-pip
```

#### 2. Install ROS2 Humble

```bash
# Set locale
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# Add ROS2 apt repository
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS2
sudo apt update
sudo apt install ros-humble-desktop
```

#### 3. Install colcon and Build Tools

```bash
sudo apt install python3-colcon-common-extensions python3-rosdep
```

#### 4. Initialize rosdep

```bash
sudo rosdep init
rosdep update
```

#### 5. Add ROS2 to Shell

```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

#### 6. Verify Installation

```bash
ros2 --version
colcon --help
```

---

### Windows

#### 1. Install Python 3.10+

Download and install from https://www.python.org/downloads/

**Important:** Check "Add Python to PATH" during installation.

#### 2. Install Visual Studio Build Tools

Download and install Visual Studio 2019 or 2022 with:
- "Desktop development with C++"
- Windows 10/11 SDK

#### 3. Install ROS2 Humble

Download the ROS2 Humble binary from:
https://github.com/ros2/ros2/releases

Extract to `C:\opt\ros\humble` (or your preferred location)

Alternative: Use Chocolatey

```powershell
# Install Chocolatey (if not already installed)
Set-ExecutionPolicy Bypass -Scope Process -Force
[System.Net.ServicePointManager]::SecurityProtocol = [System.Net.ServicePointManager]::SecurityProtocol -bor 3072
iex ((New-Object System.Net.WebClient).DownloadString('https://community.chocolatey.org/install.ps1'))

# Install ROS2
choco install ros-humble-desktop
```

#### 4. Install colcon

```powershell
pip install colcon-common-extensions
```

#### 5. Verify Installation

Open a new command prompt:

```cmd
call C:\opt\ros\humble\setup.bat
ros2 --version
colcon --help
```

---

## Building SAINT.OS

### Quick Start

#### macOS (with Conda/RoboStack)

```bash
# Clone the repository
git clone https://github.com/OpenSAINT/SaintOS.git
cd SaintOS/source

# Activate your ROS2 Conda environment
conda activate ros2_env

# Build using the build script
./saint_os/scripts/build.sh

# Or using the Python script
python3 saint_os/scripts/build.py
```

#### Linux

```bash
# Clone the repository
git clone https://github.com/OpenSAINT/SaintOS.git
cd SaintOS/source

# Source ROS2
source /opt/ros/humble/setup.bash

# Build using the build script
./saint_os/scripts/build.sh

# Or using the Python script
python3 saint_os/scripts/build.py
```

#### Windows

```powershell
# Clone the repository
git clone https://github.com/OpenSAINT/SaintOS.git
cd SaintOS\source

# Source ROS2
call C:\opt\ros\humble\setup.bat

# Build using PowerShell script
.\saint_os\scripts\build.ps1

# Or using batch file
.\saint_os\scripts\build.bat

# Or using Python script
python saint_os\scripts\build.py
```

### Build Options

All build scripts support the following options:

| Option | Description |
|--------|-------------|
| `--clean` | Clean build artifacts before building |
| `--release` | Build in Release mode (default) |
| `--debug` | Build in Debug mode |
| `--test` | Run tests after building |
| `-j, --jobs N` | Number of parallel build jobs |
| `--install-deps` | Install Python dependencies |
| `--dev` | Include development dependencies |
| `--help` | Show help message |

#### Examples

```bash
# Clean build with tests
./saint_os/scripts/build.sh --clean --test

# Debug build with 4 parallel jobs
./saint_os/scripts/build.sh --debug -j 4

# Install dependencies and build
./saint_os/scripts/build.sh --install-deps --dev
```

### Manual Build (Using colcon directly)

If you prefer to build manually:

```bash
# Navigate to workspace (parent of saint_os)
cd SaintOS/source

# Source ROS2
source /opt/ros/humble/setup.bash

# Install Python dependencies
pip install -e saint_os/.[dev]

# Build with colcon
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

# Source the built workspace
source install/setup.bash

# Run tests (optional)
colcon test
colcon test-result --verbose
```

---

## Running the Server

After building, you can run the SAINT.OS server:

```bash
# Source the workspace
source install/setup.bash

# Run the server
ros2 run saint_os saint_server

# Or with custom parameters
ros2 run saint_os saint_server --ros-args \
    -p server_name:=SAINT-01 \
    -p web_port:=80 \
    -p websocket_port:=9090
```

### Using Launch Files

```bash
# Launch the server with default configuration
ros2 launch saint_os server.launch.py

# Launch with custom config
ros2 launch saint_os server.launch.py config:=/path/to/config.yaml
```

---

## Development Setup

### Setting Up a Development Environment

```bash
# Clone the repository
git clone https://github.com/OpenSAINT/SaintOS.git
cd SaintOS/source

# Create a Python virtual environment (optional but recommended)
python3 -m venv venv
source venv/bin/activate  # Linux/macOS
# or: venv\Scripts\activate  # Windows

# Install development dependencies
pip install -e saint_os/.[dev]

# Build in debug mode
./saint_os/scripts/build.sh --debug --install-deps --dev
```

### Running Tests

```bash
# Run all tests
./saint_os/scripts/build.sh --test

# Or manually
source install/setup.bash
colcon test
colcon test-result --verbose

# Run specific tests with pytest
pytest saint_os/test/test_server.py -v
```

### Code Style

The project uses:
- **black** for code formatting
- **isort** for import sorting
- **flake8** for linting
- **mypy** for type checking

```bash
# Format code
black saint_os/
isort saint_os/

# Lint
flake8 saint_os/
mypy saint_os/
```

---

## Troubleshooting

### Common Issues

#### "ROS2 not found"

Make sure ROS2 is sourced before building:

```bash
source /opt/ros/humble/setup.bash
```

Add to your shell profile (`~/.bashrc` or `~/.zshrc`) for convenience.

#### "colcon: command not found"

Install colcon:

```bash
pip3 install colcon-common-extensions
```

#### Build fails with missing dependencies

Install ROS2 dependencies:

```bash
# Linux
sudo apt install python3-rosdep
rosdep install --from-paths saint_os --ignore-src -r -y

# macOS
pip3 install rosdep
rosdep install --from-paths saint_os --ignore-src -r -y
```

#### Python package import errors

Make sure to source the workspace after building:

```bash
source install/setup.bash
```

#### Windows: "execution of scripts is disabled"

Enable PowerShell script execution:

```powershell
Set-ExecutionPolicy -ExecutionPolicy RemoteSigned -Scope CurrentUser
```

#### macOS Apple Silicon: Architecture issues

If you encounter architecture-related issues on M1/M2 Macs:

```bash
# Ensure you're using native ARM Python
arch -arm64 python3 -m pip install <package>

# Or run the build under Rosetta if needed
arch -x86_64 ./saint_os/scripts/build.sh
```

### Getting Help

- Check the [SAINT.OS Specification](SAINT_OS_SPEC.md) for architecture details
- Open an issue on GitHub: https://github.com/OpenSAINT/SaintOS/issues
- ROS2 Documentation: https://docs.ros.org/en/humble/

---

## Next Steps

After successful installation:

1. Review the [SAINT_OS_SPEC.md](SAINT_OS_SPEC.md) for system architecture
2. Configure the server in `saint_os/config/server.yaml`
3. Set up your network (internal Ethernet + external WiFi)
4. Access the web administration interface at `http://<server-ip>/`

---

## Version Information

| Component | Version |
|-----------|---------|
| SAINT.OS | 0.5.0 |
| ROS2 | Humble (recommended) |
| Python | 3.10+ |
