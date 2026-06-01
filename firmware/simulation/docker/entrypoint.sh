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
#   fake   — agent + server + Python fake-firmware + harness (DEFAULT;
#            recommended path until the Renode RP2040 bootrom/XIP
#            modelling gap is fixed and the real sim firmware boots)
#   full   — agent + server + Renode-sim firmware + harness
#            (currently blocked on Renode 1.15.3 / 1.16 XIP issues)
#   smoke  — agent + (no server) + harness --no-server (sim-only check)
#   shell  — agent + server, then drop into bash (manual probing)
MODE="${MODE:-fake}"

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
# The agent was linked against libfastrtps.so.2.14.5 / libfastcdr.so.2.2.5
# (versions baked into microros/micro-ros-agent:jazzy). ros:jazzy-ros-base's
# apt feeds now ship .6 / .7, which subtly break the SHM transport's
# stack-frame layout — the agent then aborts with `*** stack smashing
# detected ***` on the first received packet. We stash the matched libs
# under /opt/uros_libs and prepend that to LD_LIBRARY_PATH.
#
# Why we invoke the binary directly instead of via `ros2 run`: ros2 CLI
# loads extra typesupport libs (librmw_dds_common__rosidl_typesupport_*)
# from /opt/ros/jazzy/lib that were built against 2.2.7 and require a
# fastcdr symbol that doesn't exist in 2.2.5 — leading to a symbol
# lookup error on the first serialization. The agent itself doesn't
# need those libs, so bypassing `ros2 run` keeps the lib scope tight.
echo "[entrypoint] starting micro-ROS agent on UDP ${AGENT_PORT}"
( export LD_LIBRARY_PATH="/opt/uros_libs:/uros_ws/install/micro_ros_agent/lib:/uros_ws/install/micro_ros_msgs/lib:/opt/ros/jazzy/lib"; \
  exec /uros_ws/install/micro_ros_agent/lib/micro_ros_agent/micro_ros_agent \
      udp4 --port "${AGENT_PORT}" > /tmp/agent.log 2>&1 ) &
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

# ── Optionally start the Python fake firmware ─────────────────────────
# In `fake` mode the harness runs against a Python ROS2 node that
# speaks the same protocol as the real firmware. Lives in the same
# container as the agent + server so no cross-container DDS discovery
# is needed. Lets us validate the server-side chain (Sync → DDS →
# subscriber → log echo) without depending on the Renode sim.
FAKE_FW_PID=""
FAKE_FW_NODE_ID="${FAKE_FW_NODE_ID:-rp2040_fakefw}"
if [[ "${MODE}" == "fake" ]]; then
    echo "[entrypoint] starting Python fake firmware (node_id=${FAKE_FW_NODE_ID})"
    python3 /work/firmware/simulation/docker/fake_firmware.py \
        --node-id "${FAKE_FW_NODE_ID}" > /tmp/fake_firmware.log 2>&1 &
    FAKE_FW_PID=$!
    # Wait a moment for the publisher to come up and DDS to discover.
    sleep 2
    if ! kill -0 "${FAKE_FW_PID}" 2>/dev/null; then
        echo "[entrypoint] fake firmware died during startup — log:" >&2
        tail -40 /tmp/fake_firmware.log >&2
        exit 3
    fi
    echo "[entrypoint] fake firmware ready"
fi

# ── Run the requested mode ─────────────────────────────────────────────
trap '
    # Preserve /tmp logs to the host-mounted logs dir so an operator can
    # diagnose a failed run after the container has exited.
    if [[ -d /work/firmware/simulation/logs ]]; then
        for f in agent server fake_firmware; do
            [[ -f "/tmp/${f}.log" ]] && cp "/tmp/${f}.log" \
                "/work/firmware/simulation/logs/${f}.log" 2>/dev/null || true
        done
    fi
    [[ -n "${FAKE_FW_PID}" ]] && kill ${FAKE_FW_PID} 2>/dev/null
    [[ -n "${SERVER_PID}" ]] && kill ${SERVER_PID} 2>/dev/null
    kill ${AGENT_PID} 2>/dev/null
    true
' EXIT

case "${MODE}" in
    fake)
        echo "[entrypoint] running e2e harness vs fake firmware against ${WS_URL}"
        python3 /work/firmware/simulation/test_sync_recovery.py \
            --ws-url "${WS_URL}" --password "${PASSWORD}" \
            --fake-firmware --node-id "${FAKE_FW_NODE_ID}"
        ;;
    full)
        echo "[entrypoint] running full Renode e2e harness against ${WS_URL}"
        python3 /work/firmware/simulation/test_sync_recovery.py \
            --ws-url "${WS_URL}" --password "${PASSWORD}"
        ;;
    smoke)
        echo "[entrypoint] running --no-server smoke test"
        python3 /work/firmware/simulation/test_sync_recovery.py --no-server
        ;;
    shell)
        echo "[entrypoint] agent + server up. Dropping into bash."
        echo "  agent log:        tail -F /tmp/agent.log"
        echo "  server log:       tail -F /tmp/server.log"
        echo "  fake firmware log:tail -F /tmp/fake_firmware.log"
        echo "  harness:          python3 /work/firmware/simulation/test_sync_recovery.py --fake-firmware --node-id ${FAKE_FW_NODE_ID}"
        exec bash
        ;;
    *)
        echo "[entrypoint] unknown MODE=${MODE}; expected fake|full|smoke|shell" >&2
        exit 2
        ;;
esac
