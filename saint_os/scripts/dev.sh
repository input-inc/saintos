#!/bin/zsh
#
# SAINT.OS Development Server
#
# Usage (MUST be sourced, not executed):
#   source scripts/dev.sh          # Build and run server
#   source scripts/dev.sh --watch  # Build, run server, and watch for changes
#
# Or use the alias (add to ~/.zshrc):
#   alias saint-dev='source /path/to/saint_os/scripts/dev.sh'
#

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color


# Get the directory where this script lives
# ${(%):-%x} gives the path to the sourced script in zsh
SCRIPT_DIR="${0:A:h}"
if [[ "$SCRIPT_DIR" == *"zsh"* || "$SCRIPT_DIR" == "-"* || ! -d "$SCRIPT_DIR/saint_server" ]]; then
    # Fallback: assume we're in saint_os directory or its subdirectory
    if [[ -d "saint_server" ]]; then
        SAINT_OS_DIR="$(pwd)"
    elif [[ -d "../saint_server" ]]; then
        SAINT_OS_DIR="$(cd .. && pwd)"
    else
        echo "${RED}Error: Could not determine saint_os directory${NC}"
        echo "Please run from the saint_os directory"
        return 1
    fi
else
    SAINT_OS_DIR="$(dirname "$SCRIPT_DIR")"
fi

# Parse arguments
_SAINT_WATCH_MODE=false
for arg in "$@"; do
    case $arg in
        --watch|-w)
            _SAINT_WATCH_MODE=true
            ;;
    esac
done

echo "${BLUE}========================================${NC}"
echo "${BLUE}  SAINT.OS Development Server${NC}"
echo "${BLUE}========================================${NC}"
echo ""

# Change to saint_os directory
cd "$SAINT_OS_DIR"
echo "${GREEN}Working directory:${NC} $SAINT_OS_DIR"
echo ""

# Activate conda environment if needed
CURRENT_ENV=$(basename "$CONDA_PREFIX" 2>/dev/null || echo "")

if [[ "$CURRENT_ENV" != "ros2_env" ]]; then
    echo "${YELLOW}Activating ros2_env conda environment...${NC}"
    conda activate ros2_env
    if [[ $? -ne 0 ]]; then
        echo "${RED}Error: Failed to activate ros2_env${NC}"
        return 1
    fi
    echo "${GREEN}Activated:${NC} ros2_env"
else
    echo "${GREEN}Conda environment:${NC} ros2_env"
fi

# Save conda bin path before workspace sourcing can disrupt it
_SAINT_CONDA_BIN="$CONDA_PREFIX/bin"
export PATH="$_SAINT_CONDA_BIN:$PATH"
echo ""

# Build the workspace
echo "${YELLOW}Building workspace...${NC}"
"$_SAINT_CONDA_BIN/colcon" build --symlink-install
if [[ $? -ne 0 ]]; then
    echo "${RED}Build failed!${NC}"
    return 1
fi
echo "${GREEN}Build complete!${NC}"
echo ""

# Source the workspace
echo "${YELLOW}Sourcing workspace...${NC}"
source install/setup.zsh

# Re-ensure conda bin is on PATH (workspace sourcing can disrupt it)
export PATH="$_SAINT_CONDA_BIN:$PATH"

echo "${GREEN}Workspace sourced!${NC}"
echo ""

# Print micro-ROS agent instructions
echo "${BLUE}========================================${NC}"
echo "${BLUE}  micro-ROS Agent (run in separate terminal)${NC}"
echo "${BLUE}========================================${NC}"
echo ""

# Check for local Micro-XRCE-DDS-Agent build in common locations
_SAINT_AGENT_PATH=""
_SAINT_AGENT_SEARCH_PATHS=(
    "$HOME/Sources/Micro-XRCE-DDS-Agent/build/MicroXRCEAgent"
    "$HOME/Micro-XRCE-DDS-Agent/build/MicroXRCEAgent"
    "$SAINT_OS_DIR/build/Micro-XRCE-DDS-Agent/MicroXRCEAgent"
    "/usr/local/bin/MicroXRCEAgent"
)

for path in "${_SAINT_AGENT_SEARCH_PATHS[@]}"; do
    if [[ -f "$path" ]]; then
        _SAINT_AGENT_PATH="$path"
        break
    fi
done

if [[ -n "$_SAINT_AGENT_PATH" ]]; then
    echo "${GREEN}Local agent found!${NC} Run with:"
    echo ""
    echo "  ${YELLOW}$_SAINT_AGENT_PATH udp4 -p 8888${NC}"
    echo ""
else
    echo "Option 1: ${GREEN}Docker${NC} (easiest)"
    echo ""
    echo "  ${YELLOW}docker run -it --rm -p 8888:8888/udp microros/micro-ros-agent:humble udp4 --port 8888 -v4${NC}"
    echo ""
    echo "Option 2: ${GREEN}Build locally${NC}"
    echo ""
    echo "  git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git"
    echo "  cd Micro-XRCE-DDS-Agent && mkdir build && cd build"
    echo "  cmake .. && make"
    echo "  ./MicroXRCEAgent udp4 -p 8888"
    echo ""
fi

echo "${BLUE}========================================${NC}"
echo ""

# Run the server (with optional watch mode)
if $_SAINT_WATCH_MODE; then
    echo "${YELLOW}Watch mode enabled - server will restart on file changes${NC}"
    echo "Watching: saint_server/, saint_common/, web/"
    echo "Press Ctrl+C to stop"
    echo ""

    # Track server PID for cleanup
    _SAINT_SERVER_PID=""

    # Cleanup function for Ctrl+C
    _saint_cleanup() {
        echo ''
        echo "${YELLOW}Stopping server...${NC}"
        if [[ -n "$_SAINT_SERVER_PID" ]]; then
            kill $_SAINT_SERVER_PID 2>/dev/null
            wait $_SAINT_SERVER_PID 2>/dev/null
        fi
        # Kill any lingering fswatch
        pkill -f "fswatch.*saint_server" 2>/dev/null
        echo "${GREEN}Server stopped.${NC}"
        # Return instead of exit since we're sourcing
        return 0
    }
    trap _saint_cleanup INT TERM

    # Loop to restart server when changes detected
    while true; do
        echo "${GREEN}[$(date +%H:%M:%S)] Starting SAINT.OS server...${NC}"
        echo ""

        # Start the server in background
        PATH="$_SAINT_CONDA_BIN:$PATH" "$_SAINT_CONDA_BIN/ros2" launch saint_os saint_server.launch.py &
        _SAINT_SERVER_PID=$!

        # Wait for file changes (fswatch blocks until a change is detected)
        # -1 means exit after first event batch
        fswatch -1 "$SAINT_OS_DIR/saint_server" "$SAINT_OS_DIR/saint_common" "$SAINT_OS_DIR/web" >/dev/null 2>&1
        _FSWATCH_EXIT=$?

        # Check if fswatch was interrupted (Ctrl+C)
        if [[ $_FSWATCH_EXIT -ne 0 ]]; then
            break
        fi

        echo ""
        echo "${YELLOW}[$(date +%H:%M:%S)] Changes detected, restarting server...${NC}"

        # Kill the server
        kill $_SAINT_SERVER_PID 2>/dev/null
        wait $_SAINT_SERVER_PID 2>/dev/null

        # Rebuild
        echo "${YELLOW}Rebuilding...${NC}"
        "$_SAINT_CONDA_BIN/colcon" build --symlink-install 2>&1 | grep -E "(Starting|Finished|Failed|error:|warning:)" || true
        echo "${GREEN}[$(date +%H:%M:%S)] Rebuild complete${NC}"
        echo ""
    done
else
    # Run the server normally (no watch)
    echo "${GREEN}Starting SAINT.OS server...${NC}"
    echo ""
    # Ensure PATH includes conda bin for subprocesses (python3, etc.)
    PATH="$_SAINT_CONDA_BIN:$PATH" "$_SAINT_CONDA_BIN/ros2" launch saint_os saint_server.launch.py
fi

# Clean up variables
unset _SAINT_WATCH_MODE _SAINT_AGENT_PATH _SAINT_AGENT_SEARCH_PATHS _SAINT_FSWATCH_PID _SAINT_CONDA_BIN
