#!/usr/bin/env bash
#
# Source-build ROS2 (ros_base subset) + micro_ros_agent inside a Debian
# Bookworm container, producing ros2-install.tar.gz in the workspace root.
#
# Why Debian Bookworm: building against Bookworm's glibc 2.36 / libpython3.11
# yields binaries that run natively on Raspberry Pi OS Bookworm and on
# Ubuntu 24.04 noble (with libpython3.11 installed via apt).
#
# Expected env vars:
#   ROS_DISTRO       - e.g. "jazzy"
#   MICRO_ROS_REF    - micro_ros_setup branch, e.g. "jazzy"
#   DEBIAN_RELEASE   - container tag, e.g. "bookworm"
#
# Output: $GITHUB_WORKSPACE/ros2-install.tar.gz

set -eo pipefail

: "${ROS_DISTRO:?must set ROS_DISTRO}"
: "${MICRO_ROS_REF:?must set MICRO_ROS_REF}"
: "${DEBIAN_RELEASE:?must set DEBIAN_RELEASE}"

WS="${GITHUB_WORKSPACE:-$PWD}"
echo "Workspace: $WS"
echo "Building ROS2 ${ROS_DISTRO} for arm64 against Debian ${DEBIAN_RELEASE}"

# The actual build runs as root inside the container. We mount the workspace
# in so the resulting tarball lands back on the runner FS.
# -i forwards our heredoc as stdin to bash inside the container. Without it,
# docker run discards stdin and the container's bash exits immediately with
# nothing to do — producing a "success" run with no actual work performed.
docker run --rm -i --platform linux/arm64 \
    -v "${WS}:/work" \
    -w /work \
    -e ROS_DISTRO \
    -e MICRO_ROS_REF \
    "debian:${DEBIAN_RELEASE}" \
    bash -eo pipefail <<'CONTAINER_EOF'

export DEBIAN_FRONTEND=noninteractive

# --- Container prereqs ------------------------------------------------------

apt-get update
apt-get install -y --no-install-recommends \
    ca-certificates curl gnupg wget \
    build-essential cmake git pkg-config \
    python3 python3-dev python3-pip python3-venv \
    python3-flake8 python3-numpy python3-yaml python3-pytest \
    libacl1-dev liblog4cxx-dev libcurl4-openssl-dev libssl-dev \
    libtinyxml2-dev libxml2-dev libyaml-dev libeigen3-dev \
    libasio-dev libtinyxml-dev libcunit1-dev libbenchmark-dev \
    uncrustify cppcheck

# Use C.UTF-8 — built into glibc, no locale generation needed.
export LANG=C.UTF-8 LC_ALL=C.UTF-8

# ROS2 build tooling — these aren't in Debian's apt repos (only Ubuntu's
# via packages.ros.org), so we follow the ROS2 source-install procedure for
# unsupported distros and install them from PyPI. --break-system-packages
# is fine in a throwaway build container; never use it on the target.
# EmPy is pinned to 3.x — Jazzy is incompatible with EmPy 4.
python3 -m pip install --break-system-packages -U \
    colcon-common-extensions \
    rosdep \
    vcstool \
    catkin_pkg \
    "empy==3.3.4" \
    lark \
    netifaces \
    pyparsing \
    setuptools

# rosdep needs to be initialized to map keys to apt packages on bookworm.
# bookworm is "Tier 3" so we'll see warnings about unsupported deps — they
# don't block the build of ros_base since we provide the system libs above.
rosdep init || true
rosdep update --rosdistro="${ROS_DISTRO}"

# --- Stage ROS2 sources -----------------------------------------------------

ROS_ROOT=/opt/ros/${ROS_DISTRO}
mkdir -p "${ROS_ROOT}/src"
cd "${ROS_ROOT}"

echo ">>> Importing ros2.repos..."
vcs import --input /work/_ros2_src/ros2.repos src

echo ">>> Cloning micro_ros_setup..."
git clone --depth 1 -b "${MICRO_ROS_REF}" \
    https://github.com/micro-ROS/micro_ros_setup.git \
    src/micro_ros_setup

# --- Install transitive rosdeps for what we just imported -------------------

echo ">>> Installing rosdeps..."
# --os debian:bookworm forces rosdep to map keys for our actual platform,
# even though it's Tier 3. Skip keys it doesn't know — we've installed the
# essential system libs above.
rosdep install --from-paths src --ignore-src -y \
    --rosdistro="${ROS_DISTRO}" \
    --os=debian:${DEBIAN_RELEASE:-bookworm} \
    --skip-keys="fastcdr rti-connext-dds-6.0.1 urdfdom_headers" \
    || true

# --- Phase 1: build ros_base (and only what it transitively needs) ----------

echo ">>> Building ROS2 ros_base..."
colcon build \
    --merge-install \
    --packages-up-to ros_base \
    --cmake-args -DCMAKE_BUILD_TYPE=Release \
                 -DBUILD_TESTING=OFF \
    --event-handlers console_cohesion+ \
    --executor sequential

source install/setup.bash

# --- Phase 2: build micro_ros_setup tool ------------------------------------

echo ">>> Building micro_ros_setup..."
colcon build \
    --merge-install \
    --packages-select micro_ros_setup \
    --cmake-args -DCMAKE_BUILD_TYPE=Release \
                 -DBUILD_TESTING=OFF

source install/setup.bash

# --- Phase 3: create + build the micro-ROS agent ----------------------------

echo ">>> Creating micro-ROS agent workspace..."
ros2 run micro_ros_setup create_agent_ws.sh

echo ">>> Building micro-ROS agent..."
ros2 run micro_ros_setup build_agent.sh

# --- Tar up the result for the runner --------------------------------------

cd "${ROS_ROOT}"
echo ">>> Packaging install/ tree..."
tar --owner=0 --group=0 -czf /work/ros2-install.tar.gz install/

SIZE=$(stat -c%s /work/ros2-install.tar.gz)
echo ">>> Done. ros2-install.tar.gz = ${SIZE} bytes"

CONTAINER_EOF

echo "Build script finished. Output: ${WS}/ros2-install.tar.gz"
ls -lh "${WS}/ros2-install.tar.gz"
