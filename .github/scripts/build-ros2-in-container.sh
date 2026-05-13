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
    python3-pytest-cov python3-pytest-mock python3-pytest-repeat \
    python3-pytest-rerunfailures python3-pytest-runner python3-pytest-timeout \
    python3-mypy \
    uncrustify cppcheck \
    libacl1-dev \
    libasio-dev \
    libbenchmark-dev \
    libbullet-dev \
    libcunit1-dev \
    libcurl4-openssl-dev \
    libeigen3-dev \
    libffi-dev \
    liblog4cxx-dev \
    liblttng-ust-dev \
    liblz4-dev \
    libpcre3-dev \
    libsqlite3-dev \
    libssl-dev \
    libtinyxml-dev \
    libtinyxml2-dev \
    libxml2-dev \
    libxslt1-dev \
    libyaml-dev \
    libzstd-dev \
    uuid-dev

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

# ros2/variants holds the ros_base / ros_core / desktop meta-packages.
# It's NOT included in Jazzy's ros2.repos (was in Humble), but it still
# exists upstream — we need it for `colcon build --packages-up-to ros_base`.
echo ">>> Cloning ros2/variants (provides ros_base meta-package)..."
git clone --depth 1 -b "${ROS_DISTRO}" \
    https://github.com/ros2/variants.git \
    src/ros2/variants

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

# --- Phase 3: micro-ROS agent -----------------------------------------------
#
# We replicate the work create_agent_ws.sh would do — vcs-import the agent
# repos, then build — but bypass its built-in rosdep call. Its rosdep call
# hardcodes --os=ubuntu:noble (we patched that earlier, but rosdep on Debian
# still can't resolve every Ubuntu-only key in rosdistro), and we don't
# actually need it: every transitive dep of micro_ros_agent is either a
# ROS package we source-built in Phase 1 (already in /opt/ros/jazzy/install/)
# or a system lib we apt-installed at container setup.

PREFIX=$(ros2 pkg prefix micro_ros_setup)
TARGETDIR=src
mkdir -p "${TARGETDIR}"

AGENT_REPOS="${PREFIX}/config/agent_uros_packages.repos"
if [[ ! -f "${AGENT_REPOS}" ]]; then
    AGENT_REPOS=$(find "${ROS_ROOT}/install" -name 'agent_uros_packages.repos' -type f -print -quit)
fi

# Strip drive_base (vision demo, brings in image_pipeline) and any
# Connext-specific repos (Ubuntu-only, proprietary) from the imports.
echo ">>> Pruning unwanted repos from ${AGENT_REPOS}"
python3 - "${AGENT_REPOS}" <<'PY'
import sys, yaml
path = sys.argv[1]
with open(path) as f:
    data = yaml.safe_load(f) or {}
repos = data.get('repositories', {}) or {}
DROP = ('drive_base', 'rti_connext')
removed = [k for k in list(repos) if any(p in k.lower() for p in DROP)]
for k in removed:
    repos.pop(k, None)
data['repositories'] = repos
with open(path, 'w') as f:
    yaml.safe_dump(data, f, default_flow_style=False, sort_keys=False)
print(f"    stripped: {removed if removed else '(nothing)'}")
PY

# create_ws.sh handles only the vcs imports (filtered ros2.repos + the user
# repos). It doesn't run rosdep — that's create_agent_ws.sh's later step
# which we're skipping.
echo ">>> Importing agent workspace repos..."
ros2 run micro_ros_setup create_ws.sh \
    "${TARGETDIR}" \
    "${PREFIX}/config/agent_ros2_packages.txt" \
    "${AGENT_REPOS}"

# Workspace inventory — useful when debugging unexpected deps.
echo ">>> Agent workspace package.xml inventory:"
find "${TARGETDIR}" -name package.xml -type f | sort | sed 's|^|    |'

# Belt-and-suspenders: drop any source dirs for packages we don't want
# colcon to compile, even if they snuck past the repos-file filter.
for pkg in rti_connext_dds_cmake_module drive_base; do
    find "${TARGETDIR}" -type d \( -name "${pkg}" -o -path "*/${pkg}" \) -prune \
        -exec rm -rf {} + 2>/dev/null || true
done

# Skip rosdep entirely. Reasoning above. If a transitive dep IS missing,
# the colcon build below will fail with a useful CMake-level error — much
# clearer than rosdep's "no Debian mapping" cryptic failures.

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
