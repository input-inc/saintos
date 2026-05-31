#!/bin/bash
# SAINT.OS e2e container entrypoint.
#
# Starts the micro-ROS agent + the saint_server, waits for both to be
# ready, then runs the e2e harness. Exits with the harness's status.
#
# Logs from each subprocess go to /tmp/<name>.log so an operator can
# `docker compose logs` or `docker exec` to debug a failed run.

set -euo pipefail

ROS_DISTRO="${ROS_DISTRO:-jazzy}"
AGENT_PORT="${AGENT_PORT:-8888}"
WS_URL="${WS_URL:-ws://localhost:9090}"
PASSWORD="${PASSWORD:-12345}"

# Mode controls what to run. Useful for debugging the container without
# always running the full e2e flow.
#   full   — agent + server + harness (default)
#   smoke  — agent + (no server) + harness --no-server (sim-only check)
#   shell  — agent + server, then drop into bash (manual probing)
MODE="${MODE:-full}"

mkdir -p /tmp/saint-os

# ── Source ROS2 ────────────────────────────────────────────────────────
# Order matters: distro setup → micro-ROS agent overlay → saint_os overlay.
# Each layer prepends to AMENT/CMAKE/LD paths; saint_os last means its
# ros2 run hits work the same way they do in production.
# ROS2's setup.bash references AMENT_TRACE_SETUP_FILES without a guard,
# which trips `set -u` from `set -euo pipefail` above. Drop the nounset
# bit just for the source block and put it back afterwards.
set +u
# shellcheck source=/dev/null
source "/opt/ros/${ROS_DISTRO}/setup.bash"
# shellcheck source=/dev/null
source /uros_ws/install/setup.bash
# shellcheck source=/dev/null
source /work/install/setup.bash
set -u

# ── Start agent ────────────────────────────────────────────────────────
echo "[entrypoint] starting micro-ROS agent on UDP ${AGENT_PORT}"
ros2 run micro_ros_agent micro_ros_agent udp4 --port "${AGENT_PORT}" \
    > /tmp/agent.log 2>&1 &
AGENT_PID=$!

# Wait for the agent to bind its UDP socket. Capping at 30 s because
# the agent's startup is fast — if it's not up by then, something is
# fundamentally wrong.
for _ in $(seq 1 30); do
    if lsof -iUDP:"${AGENT_PORT}" -P 2>/dev/null | grep -q LISTEN.*ros2 \
       || lsof -iUDP:"${AGENT_PORT}" -P 2>/dev/null | grep -q micro; then
        echo "[entrypoint] agent ready"
        break
    fi
    if ! kill -0 "${AGENT_PID}" 2>/dev/null; then
        echo "[entrypoint] agent died during startup — log:" >&2
        cat /tmp/agent.log >&2
        exit 3
    fi
    sleep 1
done

# ── Optionally start server ────────────────────────────────────────────
SERVER_PID=""
if [[ "${MODE}" != "smoke" ]]; then
    echo "[entrypoint] starting saint_server"
    # SAINT_LOG_DIR is set in the image; the server writes per-node
    # activity logs there for the harness to inspect on failure.
    # Entry-point name comes from server/setup.py:console_scripts.
    ros2 run saint_os saint_server > /tmp/server.log 2>&1 &
    SERVER_PID=$!

    # Wait for the HTTP/WebSocket port (80) to accept connections.
    # SAINT.OS bundles WS upgrades on the same port the static UI
    # serves from — no separate 9090.
    for _ in $(seq 1 60); do
        if nc -z localhost 80 2>/dev/null; then
            echo "[entrypoint] server HTTP/WebSocket ready on :80"
            break
        fi
        if ! kill -0 "${SERVER_PID}" 2>/dev/null; then
            echo "[entrypoint] server died during startup — log:" >&2
            tail -40 /tmp/server.log >&2
            exit 3
        fi
        sleep 1
    done
fi

# ── Run the requested mode ─────────────────────────────────────────────
trap '[[ -n "${SERVER_PID}" ]] && kill ${SERVER_PID} 2>/dev/null; kill ${AGENT_PID} 2>/dev/null; true' EXIT

case "${MODE}" in
    full)
        echo "[entrypoint] running full e2e harness against ${WS_URL}"
        exec python3 /work/firmware/simulation/test_sync_recovery.py \
            --ws-url "${WS_URL}" --password "${PASSWORD}"
        ;;
    smoke)
        echo "[entrypoint] running --no-server smoke test"
        exec python3 /work/firmware/simulation/test_sync_recovery.py --no-server
        ;;
    shell)
        echo "[entrypoint] agent + server up. Dropping into bash."
        echo "  agent log:  tail -F /tmp/agent.log"
        echo "  server log: tail -F /tmp/server.log"
        echo "  harness:    python3 /work/firmware/simulation/test_sync_recovery.py"
        exec bash
        ;;
    *)
        echo "[entrypoint] unknown MODE=${MODE}; expected full|smoke|shell" >&2
        exit 2
        ;;
esac
