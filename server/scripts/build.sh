#!/bin/bash
#
# SAINT.OS Build Script for Linux/macOS
#

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PACKAGE_DIR="$(dirname "$SCRIPT_DIR")"
WORKSPACE_DIR="$(dirname "$PACKAGE_DIR")"

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo_info() {
    echo -e "${GREEN}[INFO]${NC} $1"
}

echo_warn() {
    echo -e "${YELLOW}[WARN]${NC} $1"
}

echo_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# Detect OS
detect_os() {
    if [[ "$OSTYPE" == "darwin"* ]]; then
        echo "macos"
    elif [[ "$OSTYPE" == "linux-gnu"* ]]; then
        echo "linux"
    else
        echo "unknown"
    fi
}

# Find ROS2 setup script
find_ros2_setup() {
    # Check if ROS2 is already sourced (e.g., via Conda/RoboStack)
    if [[ -n "$ROS_DISTRO" ]] && command -v ros2 &> /dev/null; then
        echo "ALREADY_SOURCED"
        return 0
    fi

    # Check Conda environment (RoboStack)
    if [[ -n "$CONDA_PREFIX" ]]; then
        local conda_setup="$CONDA_PREFIX/setup.bash"
        if [[ -f "$conda_setup" ]]; then
            echo "$conda_setup"
            return 0
        fi
        # RoboStack may not have setup.bash, check if ros2 is available
        if command -v ros2 &> /dev/null; then
            echo "ALREADY_SOURCED"
            return 0
        fi
    fi

    # Standard ROS2 installation paths
    local ros2_paths=(
        "/opt/ros/jazzy/setup.bash"
        "/opt/ros/iron/setup.bash"
        "/opt/ros/humble/setup.bash"
        "$HOME/ros2_humble/install/setup.bash"
        "$HOME/ros2_humble/setup.bash"
    )

    for path in "${ros2_paths[@]}"; do
        if [[ -f "$path" ]]; then
            echo "$path"
            return 0
        fi
    done

    # Check ROS_DISTRO environment variable
    if [[ -n "$ROS_DISTRO" ]] && [[ -f "/opt/ros/$ROS_DISTRO/setup.bash" ]]; then
        echo "/opt/ros/$ROS_DISTRO/setup.bash"
        return 0
    fi

    return 1
}

# Check dependencies
check_dependencies() {
    echo_info "Checking dependencies..."

    # Check Python
    if ! command -v python3 &> /dev/null; then
        echo_error "Python 3 not found"
        exit 1
    fi

    PYTHON_VERSION=$(python3 -c 'import sys; print(f"{sys.version_info.major}.{sys.version_info.minor}")')
    echo_info "  Python: $PYTHON_VERSION"

    # Check for Conda environment
    if [[ -n "$CONDA_PREFIX" ]]; then
        echo_info "  Conda env: $CONDA_DEFAULT_ENV"
    fi

    # Check ROS2
    ROS2_SETUP=$(find_ros2_setup)
    if [[ -z "$ROS2_SETUP" ]]; then
        echo_error "ROS2 installation not found"
        echo_error ""
        echo_error "Please install ROS2 using one of these methods:"
        echo_error "  macOS:  Use RoboStack with Conda (see INSTALL.md)"
        echo_error "  Linux:  sudo apt install ros-humble-desktop"
        echo_error "  All:    Build from source"
        echo_error ""
        echo_error "If using Conda/RoboStack, activate the environment first:"
        echo_error "  conda activate ros2_env"
        exit 1
    fi

    # Source ROS2 if needed
    if [[ "$ROS2_SETUP" == "ALREADY_SOURCED" ]]; then
        echo_info "  ROS2: already sourced (ROS_DISTRO=$ROS_DISTRO)"
    else
        echo_info "  ROS2 setup: $ROS2_SETUP"
        source "$ROS2_SETUP"
    fi

    # Check colcon
    if ! command -v colcon &> /dev/null; then
        echo_error "colcon not found"
        echo_error "Install with: pip install colcon-common-extensions"
        exit 1
    fi
    echo_info "  colcon: found"

    echo "$ROS2_SETUP"
}

# Clean build artifacts
clean_build() {
    echo_info "Cleaning build artifacts..."
    cd "$WORKSPACE_DIR"
    rm -rf build install log
    echo_info "Clean complete"
}

# Build the package
build_package() {
    local build_type="${1:-Release}"
    local parallel_jobs="$2"

    echo_info "Building SAINT.OS ($build_type)..."
    cd "$WORKSPACE_DIR"

    local colcon_args=(
        "build"
        "--symlink-install"
        "--cmake-args" "-DCMAKE_BUILD_TYPE=$build_type"
    )

    if [[ -n "$parallel_jobs" ]]; then
        colcon_args+=("--parallel-workers" "$parallel_jobs")
    fi

    colcon "${colcon_args[@]}"

    echo_info "Build complete"
}

# Run tests
run_tests() {
    echo_info "Running tests..."
    cd "$WORKSPACE_DIR"

    colcon test --pytest-args -v
    colcon test-result --verbose

    echo_info "Tests complete"
}

# Install Python dependencies
install_python_deps() {
    local dev_mode="$1"
    echo_info "Installing Python dependencies..."
    cd "$PACKAGE_DIR"

    if [[ "$dev_mode" == "true" ]]; then
        pip3 install -e ".[dev]"
    else
        pip3 install -e .
    fi

    echo_info "Python dependencies installed"
}

# Print usage
usage() {
    echo "Usage: $0 [OPTIONS]"
    echo ""
    echo "Options:"
    echo "  --clean         Clean build artifacts before building"
    echo "  --release       Build in Release mode (default)"
    echo "  --debug         Build in Debug mode"
    echo "  --test          Run tests after building"
    echo "  -j, --jobs N    Number of parallel jobs"
    echo "  --install-deps  Install Python dependencies"
    echo "  --dev           Install development dependencies"
    echo "  --help          Show this help message"
}

# Main
main() {
    local clean=false
    local build_type="Release"
    local run_test=false
    local parallel_jobs=""
    local install_deps=false
    local dev_mode=false

    # Parse arguments
    while [[ $# -gt 0 ]]; do
        case $1 in
            --clean)
                clean=true
                shift
                ;;
            --release)
                build_type="Release"
                shift
                ;;
            --debug)
                build_type="Debug"
                shift
                ;;
            --test)
                run_test=true
                shift
                ;;
            -j|--jobs)
                parallel_jobs="$2"
                shift 2
                ;;
            --install-deps)
                install_deps=true
                shift
                ;;
            --dev)
                dev_mode=true
                shift
                ;;
            --help)
                usage
                exit 0
                ;;
            *)
                echo_error "Unknown option: $1"
                usage
                exit 1
                ;;
        esac
    done

    echo "========================================"
    echo "SAINT.OS Build Script"
    echo "========================================"
    echo "Platform: $(detect_os)"
    echo "Workspace: $WORKSPACE_DIR"
    echo ""

    # Check dependencies and get ROS2 setup path
    ROS2_SETUP=$(check_dependencies)
    echo ""

    # Source ROS2 if needed (skip if already sourced via Conda/RoboStack)
    if [[ "$ROS2_SETUP" != "ALREADY_SOURCED" ]]; then
        source "$ROS2_SETUP"
    fi

    # Install Python deps if requested
    if [[ "$install_deps" == "true" ]]; then
        install_python_deps "$dev_mode"
        echo ""
    fi

    # Clean if requested
    if [[ "$clean" == "true" ]]; then
        clean_build
        echo ""
    fi

    # Build
    build_package "$build_type" "$parallel_jobs"
    echo ""

    # Test if requested
    if [[ "$run_test" == "true" ]]; then
        run_tests
        echo ""
    fi

    echo "========================================"
    echo_info "Build successful!"
    echo ""
    echo "To use the built package, source the setup script:"
    echo "  source $WORKSPACE_DIR/install/setup.bash"
    echo ""
}

main "$@"
