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

# CRITICAL: Save the original system PATH at the VERY START before anything can corrupt it
# This ensures we have access to basic commands like env, dirname, etc.
if [[ -z "$_SAINT_SYSTEM_PATH" ]]; then
    # First run - save the current PATH as the system baseline
    _SAINT_SYSTEM_PATH="/usr/local/bin:/usr/bin:/bin:/usr/sbin:/sbin:$PATH"
fi
# Always ensure basic system paths are available
export PATH="/usr/local/bin:/usr/bin:/bin:/usr/sbin:/sbin:$PATH"

# Use absolute paths for critical commands (PATH can be corrupted by ROS2 workspace sourcing)
DATE=/bin/date
SLEEP=/bin/sleep
FIND=/usr/bin/find
TOUCH=/usr/bin/touch
MKTEMP=/usr/bin/mktemp
GREP=/usr/bin/grep
HEAD=/usr/bin/head
LSOF=/usr/sbin/lsof
KILL=/bin/kill
XARGS=/usr/bin/xargs

# ===========================================================================
# Cleanup function - kill any lingering processes from previous runs
# ===========================================================================
_saint_kill_previous() {
    local killed=false

    # Kill any previous saint_server processes
    local pids=$(pgrep -f "saint_server" 2>/dev/null)
    if [[ -n "$pids" ]]; then
        echo "${YELLOW}Stopping previous saint_server processes...${NC}"
        echo "$pids" | $XARGS $KILL 2>/dev/null
        killed=true
    fi

    # Kill any previous server_node.py processes
    pids=$(pgrep -f "server_node.py" 2>/dev/null)
    if [[ -n "$pids" ]]; then
        echo "${YELLOW}Stopping previous server_node processes...${NC}"
        echo "$pids" | $XARGS $KILL 2>/dev/null
        killed=true
    fi

    # Kill any lingering fswatch processes for saint_os
    pids=$(pgrep -f "fswatch.*saint" 2>/dev/null)
    if [[ -n "$pids" ]]; then
        echo "${YELLOW}Stopping previous fswatch processes...${NC}"
        echo "$pids" | $XARGS $KILL 2>/dev/null
        killed=true
    fi

    # Kill any ros2 launch processes for saint_os
    pids=$(pgrep -f "ros2.*launch.*saint" 2>/dev/null)
    if [[ -n "$pids" ]]; then
        echo "${YELLOW}Stopping previous ros2 launch processes...${NC}"
        echo "$pids" | $XARGS $KILL 2>/dev/null
        killed=true
    fi

    # Kill processes holding our ports (80 for web, 11111 for LiveLink)
    for port in 80 11111; do
        pids=$($LSOF -ti :$port 2>/dev/null)
        if [[ -n "$pids" ]]; then
            echo "${YELLOW}Killing processes holding port $port...${NC}"
            echo "$pids" | $XARGS $KILL 2>/dev/null
            killed=true
        fi
    done

    # Give processes time to release ports
    if $killed; then
        $SLEEP 1
        # Force kill any remaining processes
        pgrep -f "saint_server" 2>/dev/null | $XARGS $KILL -9 2>/dev/null
        pgrep -f "server_node.py" 2>/dev/null | $XARGS $KILL -9 2>/dev/null
        # Force kill anything still on ports
        for port in 80 11111; do
            $LSOF -ti :$port 2>/dev/null | $XARGS $KILL -9 2>/dev/null
        done
        $SLEEP 0.5
        echo "${GREEN}Previous processes stopped.${NC}"
        echo ""
    fi
}

# Run cleanup before starting
_saint_kill_previous


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

# Save the FULL PATH before workspace sourcing can disrupt it
_SAINT_CONDA_BIN="$CONDA_PREFIX/bin"
_SAINT_SAVED_PATH="$PATH"
echo ""

# Build the workspace
echo "${YELLOW}Building workspace...${NC}"
PATH="$_SAINT_CONDA_BIN:$PATH" "$_SAINT_CONDA_BIN/colcon" build --symlink-install
if [[ $? -ne 0 ]]; then
    echo "${RED}Build failed!${NC}"
    return 1
fi
echo "${GREEN}Build complete!${NC}"
echo ""

# Source the workspace
echo "${YELLOW}Sourcing workspace...${NC}"
source install/setup.zsh

# CRITICAL: Restore the full PATH - workspace sourcing can completely break it
export PATH="$_SAINT_SAVED_PATH"

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

# Flag to prevent double cleanup
_SAINT_CLEANUP_DONE=false

# Define cleanup function at top level so it's always available
_saint_dev_cleanup() {
    # Prevent running cleanup twice
    if $_SAINT_CLEANUP_DONE; then
        return 0
    fi
    _SAINT_CLEANUP_DONE=true

    echo ''
    echo "${YELLOW}Stopping server...${NC}"

    # Remove marker file to signal polling loop to exit
    if [[ -n "$_SAINT_MARKER_FILE" ]]; then
        /bin/rm -f "$_SAINT_MARKER_FILE" 2>/dev/null
    fi

    # Kill the tracked server PID if set
    if [[ -n "$_SAINT_SERVER_PID" ]]; then
        /bin/kill $_SAINT_SERVER_PID 2>/dev/null
        wait $_SAINT_SERVER_PID 2>/dev/null
    fi

    # Kill any child processes (server_node, webserver, etc.)
    pkill -f "saint_server" 2>/dev/null
    pkill -f "server_node.py" 2>/dev/null

    # Give time for graceful shutdown
    /bin/sleep 0.5

    # Force kill any remaining
    pkill -9 -f "saint_server" 2>/dev/null
    pkill -9 -f "server_node.py" 2>/dev/null

    # Kill anything holding our ports
    /usr/sbin/lsof -ti :80 2>/dev/null | /usr/bin/xargs /bin/kill -9 2>/dev/null
    /usr/sbin/lsof -ti :11111 2>/dev/null | /usr/bin/xargs /bin/kill -9 2>/dev/null

    echo "${GREEN}Server stopped.${NC}"

    # Reset the trap
    trap - INT TERM

    return 0
}

# Final cleanup - call this at the very end to unset everything
_saint_final_cleanup() {
    # Unset all variables and functions
    unset _SAINT_WATCH_MODE _SAINT_AGENT_PATH _SAINT_AGENT_SEARCH_PATHS _SAINT_CONDA_BIN _SAINT_SERVER_PID _SAINT_SAVED_PATH _SAINT_MARKER_FILE _SAINT_CHANGES_DETECTED _SAINT_CLEANUP_DONE _SAINT_SYSTEM_PATH DATE SLEEP FIND TOUCH MKTEMP GREP HEAD LSOF KILL XARGS 2>/dev/null
    unset -f _saint_kill_previous _saint_dev_cleanup _saint_final_cleanup 2>/dev/null
}

# Set trap to use our cleanup function
trap _saint_dev_cleanup INT TERM

# Run the server (with optional watch mode)
if $_SAINT_WATCH_MODE; then
    echo "${YELLOW}Watch mode enabled - server will restart on file changes${NC}"
    echo "Watching: saint_server/, saint_common/, web/ (polling every 2s)"
    echo "Press Ctrl+C to stop"
    echo ""

    # Track server PID for cleanup
    _SAINT_SERVER_PID=""

    # Create a marker file for change detection
    _SAINT_MARKER_FILE=$($MKTEMP)
    $TOUCH "$_SAINT_MARKER_FILE"

    # Loop to restart server when changes detected
    while true; do
        echo "${GREEN}[$(/bin/date +%H:%M:%S)] Starting SAINT.OS server...${NC}"
        echo ""

        # Start the server in background
        PATH="$_SAINT_CONDA_BIN:$PATH" "$_SAINT_CONDA_BIN/ros2" launch saint_os saint_server.launch.py &
        _SAINT_SERVER_PID=$!

        # Poll for file changes (check every 2 seconds)
        _SAINT_CHANGES_DETECTED=false
        while true; do
            /bin/sleep 2

            # Check if we should exit (cleanup removes marker file)
            if [[ ! -f "$_SAINT_MARKER_FILE" ]]; then
                break 2  # Break out of both loops
            fi

            # Check if server process died unexpectedly
            if ! kill -0 $_SAINT_SERVER_PID 2>/dev/null; then
                echo ""
                echo "${RED}[$(/bin/date +%H:%M:%S)] Server process died unexpectedly${NC}"
                _SAINT_CHANGES_DETECTED=true
                break
            fi

            # Check for modified files using find -newer (exclude server_config.yaml to prevent restart on settings save)
            if $FIND "$SAINT_OS_DIR/saint_server" "$SAINT_OS_DIR/saint_common" "$SAINT_OS_DIR/web" \
                -type f \( -name "*.py" -o -name "*.js" -o -name "*.html" -o -name "*.css" -o -name "*.yaml" \) \
                ! -name "server_config.yaml" \
                -newer "$_SAINT_MARKER_FILE" 2>/dev/null | $HEAD -1 | $GREP -q .; then
                _SAINT_CHANGES_DETECTED=true
                break
            fi
        done

        # Check if we should exit (cleanup removes marker file)
        if [[ ! -f "$_SAINT_MARKER_FILE" ]]; then
            break
        fi

        if $_SAINT_CHANGES_DETECTED; then
            echo ""
            echo "${YELLOW}[$(/bin/date +%H:%M:%S)] Changes detected, restarting server...${NC}"

            # Kill the main server process
            $KILL $_SAINT_SERVER_PID 2>/dev/null
            wait $_SAINT_SERVER_PID 2>/dev/null

            # Kill all child processes that hold ports (webserver, LiveLink, etc.)
            pkill -f "saint_server" 2>/dev/null
            pkill -f "server_node.py" 2>/dev/null
            $SLEEP 0.5

            # Force kill any remaining
            pkill -9 -f "saint_server" 2>/dev/null
            pkill -9 -f "server_node.py" 2>/dev/null

            # Kill anything holding our ports
            $LSOF -ti :80 2>/dev/null | $XARGS $KILL -9 2>/dev/null
            $LSOF -ti :11111 2>/dev/null | $XARGS $KILL -9 2>/dev/null
            $SLEEP 0.5

            # Update marker file timestamp
            $TOUCH "$_SAINT_MARKER_FILE"

            # Rebuild
            echo "${YELLOW}Rebuilding...${NC}"
            PATH="$_SAINT_CONDA_BIN:$PATH" "$_SAINT_CONDA_BIN/colcon" build --symlink-install 2>&1 | $GREP -E "(Starting|Finished|Failed|error:|warning:)" || true
            echo "${GREEN}[$(/bin/date +%H:%M:%S)] Rebuild complete${NC}"
            echo ""
        fi
    done

    # Clean up marker file
    /bin/rm -f "$_SAINT_MARKER_FILE" 2>/dev/null

    # Clean up after loop exits (safe to call multiple times)
    _saint_dev_cleanup
    _saint_final_cleanup
else
    # Run the server normally (no watch)
    echo "${GREEN}Starting SAINT.OS server...${NC}"
    echo ""

    # Ensure PATH includes conda bin for subprocesses (python3, etc.)
    PATH="$_SAINT_CONDA_BIN:$PATH" "$_SAINT_CONDA_BIN/ros2" launch saint_os saint_server.launch.py

    # Clean up after server exits normally (safe to call multiple times)
    _saint_dev_cleanup
    _saint_final_cleanup
fi
