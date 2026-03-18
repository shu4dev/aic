#!/usr/bin/env bash
# CheatCode_sampling.sh
# Runs CheatCode policy N times, one results folder per run.
# Fixes:
#   - Proper intra-container process cleanup via docker exec
#   - Container restart between runs for a guaranteed clean slate
#   - Smart engine readiness detection (Zenoh port + log signal)
#     instead of fixed sleep

set -euo pipefail

# ===========================================================================
# Config — all overridable via environment variables
# ===========================================================================
WORKSPACE="${AIC_WORKSPACE:-$HOME/ws_aic/src/aic}"
BASE_RESULTS="${AIC_BASE_RESULTS:-$HOME/ws_aic/src/aic/aic_results}"
NUM_RUNS="${NUM_RUNS:-10}"
CONTAINER_NAME="${AIC_CONTAINER:-aic_eval}"
POLICY="${AIC_POLICY:-aic_example_policies.ros.CheatCode}"

# Timeouts (seconds)
ENGINE_READY_TIMEOUT="${ENGINE_READY_TIMEOUT:-120}"   # max wait for engine ready
CONTAINER_RESTART_WAIT="${CONTAINER_RESTART_WAIT:-8}" # wait after docker restart
BETWEEN_RUN_WAIT="${BETWEEN_RUN_WAIT:-5}"             # brief pause between runs

# Zenoh router port (inside container, forwarded to host via distrobox)
ZENOH_PORT="${ZENOH_PORT:-7447}"

# Ready signal emitted by aic_engine when it is waiting for aic_model
ENGINE_READY_PATTERN="No node with name 'aic_model' found\|Waiting for aic_model\|aic_engine.*initialized\|Retrying"

# Track background engine PID so cleanup always knows what to kill
ENGINE_PID=""

# ===========================================================================
# Helpers
# ===========================================================================
banner() {
    echo ""
    echo "================================================"
    echo "  $*"
    echo "================================================"
}

ts() {
    echo "[$(date +%T)]"
}

# ---------------------------------------------------------------------------
# kill_container_procs
# Hard-kills all simulation-related processes INSIDE the container.
# This is the critical fix — killing the host-side distrobox wrapper PID does
# NOT kill Zenoh / Gazebo / ROS processes in the container's PID namespace.
# ---------------------------------------------------------------------------
kill_container_procs() {
    echo "$(ts) Killing simulation processes inside container..."
    docker exec "$CONTAINER_NAME" \
        bash -c 'pkill -9 -f "entrypoint|zenoh|zenohd|gz|rviz2|ros2|aic_engine|ruby" 2>/dev/null; true' \
        2>/dev/null || true
    sleep 2
}

# ---------------------------------------------------------------------------
# reset_container
# Restarts the container — the only guaranteed way to flush:
#   - Zenoh routing tables and port bindings
#   - ROS 2 shared memory and discovery state
#   - Any zombie processes that survived pkill
# ---------------------------------------------------------------------------
reset_container() {
    echo "$(ts) Restarting container for clean state..."
    docker restart "$CONTAINER_NAME" >/dev/null 2>&1 || true
    sleep "$CONTAINER_RESTART_WAIT"
    echo "$(ts) Container restarted."
}

# ---------------------------------------------------------------------------
# teardown_run
# Called after each run (and on any exit/signal) to cleanly stop everything.
# ---------------------------------------------------------------------------
teardown_run() {
    # 1. Kill the host-side distrobox wrapper process group
    if [ -n "$ENGINE_PID" ]; then
        echo "$(ts) Stopping engine (host wrapper PID: $ENGINE_PID)..."
        local pgid
        pgid=$(ps -o pgid= -p "$ENGINE_PID" 2>/dev/null | tr -d ' ') || true
        if [ -n "$pgid" ]; then
            kill -- -"$pgid" 2>/dev/null || true
        fi
        wait "$ENGINE_PID" 2>/dev/null || true
        ENGINE_PID=""
    fi

    # 2. Kill everything still alive inside the container
    kill_container_procs
}

# ---------------------------------------------------------------------------
# cleanup — trap handler for EXIT / SIGINT / SIGTERM
# ---------------------------------------------------------------------------
cleanup() {
    echo ""
    echo "$(ts) Signal caught — cleaning up..."
    teardown_run
    echo "$(ts) Cleanup complete."
}

trap cleanup EXIT SIGINT SIGTERM

# ===========================================================================
# wait_for_engine_ready
#
# Three-stage readiness check — replaces the fixed sleep:
#
#   Stage 1 — Zenoh port (comes up first, fast to poll)
#             Confirms the middleware router is accepting connections.
#             A port conflict from a prior run is caught here at second ~1
#             instead of silently failing 30 s later.
#
#   Stage 2 — Engine log signal (semantic confirmation)
#             The aic_engine prints a known message exactly when it is ready
#             to receive the aic_model node. This is the authoritative signal.
#             Catches crashes during startup that a port check cannot detect.
#
#   If either stage times out, the function returns 1 so the run loop can
#   mark the run as FAILED and reset rather than launching the policy against
#   a dead engine.
# ===========================================================================
wait_for_engine_ready() {
    local log_file="$1"
    local timeout="${ENGINE_READY_TIMEOUT}"
    local elapsed=0

    # ------------------------------------------------------------------
    # Stage 1: Zenoh port
    # ------------------------------------------------------------------
    echo "$(ts) [Stage 1/2] Waiting for Zenoh router on port ${ZENOH_PORT}..."

    until nc -z localhost "$ZENOH_PORT" 2>/dev/null; do
        if [ "$elapsed" -ge "$timeout" ]; then
            echo "$(ts) [ERROR] Timed out waiting for Zenoh port after ${elapsed}s."
            return 1
        fi
        sleep 1
        elapsed=$((elapsed + 1))
    done

    echo "$(ts) [Stage 1/2] Zenoh router up after ${elapsed}s."

    # ------------------------------------------------------------------
    # Stage 2: Engine log ready signal
    # ------------------------------------------------------------------
    echo "$(ts) [Stage 2/2] Waiting for engine ready log signal..."

    until grep -qE "$ENGINE_READY_PATTERN" "$log_file" 2>/dev/null; do
        if [ "$elapsed" -ge "$timeout" ]; then
            echo "$(ts) [ERROR] Timed out waiting for engine ready signal after ${elapsed}s."
            return 1
        fi
        sleep 1
        elapsed=$((elapsed + 1))
    done

    echo "$(ts) [Stage 2/2] Engine ready after ${elapsed}s total."
    return 0
}

# ===========================================================================
# Sanity checks
# ===========================================================================
if [ ! -d "$WORKSPACE" ]; then
    echo "[ERROR] Workspace not found: $WORKSPACE"
    exit 1
fi

for cmd in pixi distrobox docker nc; do
    if ! command -v "$cmd" &>/dev/null; then
        echo "[ERROR] Required command not found: '$cmd'"
        [ "$cmd" = "nc" ] && echo "       Install netcat: sudo apt install netcat-openbsd"
        exit 1
    fi
done

mkdir -p "$BASE_RESULTS"
export DBX_CONTAINER_MANAGER=docker

# ===========================================================================
# Summary file
# ===========================================================================
SUMMARY="$BASE_RESULTS/summary.txt"
{
    echo "CheatCode ${NUM_RUNS}-Run Summary"
    echo "Policy    : $POLICY"
    echo "Workspace : $WORKSPACE"
    echo "Started   : $(date)"
    echo "------------------------------------------------------------"
    printf "%-5s  %-38s  %-10s  %s\n" "Run" "Folder" "Status" "Engine ready (s)"
    echo "------------------------------------------------------------"
} > "$SUMMARY"

# ===========================================================================
# One-time setup (idempotent)
# ===========================================================================
banner "Pulling aic_eval image..."
docker pull ghcr.io/intrinsic-dev/aic/aic_eval:latest

banner "Creating distrobox container (skipped if already exists)..."
distrobox create -r --nvidia \
    -i ghcr.io/intrinsic-dev/aic/aic_eval:latest \
    "$CONTAINER_NAME" || true

# ===========================================================================
# Pre-loop reset
# Handles leftover state from ANY previous interrupted session.
# This is the fix for "the next time I run it, it won't work".
# ===========================================================================
banner "Pre-run cleanup (clearing any leftover state from prior sessions)..."
kill_container_procs
reset_container

# ===========================================================================
# Main run loop
# ===========================================================================
banner "Starting $NUM_RUNS runs of $POLICY"
echo "Workspace   : $WORKSPACE"
echo "Results base: $BASE_RESULTS"
echo "Ready timeout: ${ENGINE_READY_TIMEOUT}s per run"

for i in $(seq 1 "$NUM_RUNS"); do
    TIMESTAMP=$(date +%Y%m%d_%H%M%S)
    RUN_LABEL="run_$(printf '%02d' "$i")"
    RUN_DIR="$BASE_RESULTS/${RUN_LABEL}_${TIMESTAMP}"
    mkdir -p "$RUN_DIR"

    ENGINE_LOG="$RUN_DIR/engine.log"
    POLICY_LOG="$RUN_DIR/policy.log"
    RUN_STATUS="SUCCESS"
    ENGINE_READY_SECS="N/A"

    banner "Run $i / $NUM_RUNS"
    echo "Results dir : $RUN_DIR"

    # ------------------------------------------------------------------
    # Start the engine inside the container (background)
    # ------------------------------------------------------------------
    distrobox enter -r "$CONTAINER_NAME" -- bash -c \
        "export AIC_RESULTS_DIR='$RUN_DIR' && /entrypoint.sh ground_truth:=true start_aic_engine:=true" \
        >"$ENGINE_LOG" 2>&1 &
    ENGINE_PID=$!
    echo "$(ts) Engine started (host wrapper PID: $ENGINE_PID)"

    # ------------------------------------------------------------------
    # Wait for the engine to be truly ready (smart — no fixed sleep)
    # ------------------------------------------------------------------
    START_WAIT=$SECONDS
    if ! wait_for_engine_ready "$ENGINE_LOG"; then
        echo "$(ts) [ERROR] Engine did not become ready — skipping policy launch."
        RUN_STATUS="ENGINE_FAILED"
    else
        ENGINE_READY_SECS=$((SECONDS - START_WAIT))

        # ----------------------------------------------------------------
        # Launch the policy (outside the container, via pixi)
        # ----------------------------------------------------------------
        echo "$(ts) Launching policy..."
        (
            cd "$WORKSPACE"
            pixi run ros2 run aic_model aic_model \
                --ros-args \
                -p use_sim_time:=true \
                -p policy:="$POLICY"
        ) >"$POLICY_LOG" 2>&1 || RUN_STATUS="POLICY_FAILED"

        echo "$(ts) Policy exited — $RUN_STATUS"
    fi

    # ------------------------------------------------------------------
    # Tear down this run before resetting for the next
    # ------------------------------------------------------------------
    teardown_run

    echo "$(ts) Run $i complete."
    echo "  Engine log  : $ENGINE_LOG"
    echo "  Policy log  : $POLICY_LOG"
    echo "  Results     : $RUN_DIR"
    echo "  Status      : $RUN_STATUS"
    echo "  Engine ready: ${ENGINE_READY_SECS}s"

    printf "%-5s  %-38s  %-10s  %s\n" \
        "$i" "${RUN_LABEL}_${TIMESTAMP}" "$RUN_STATUS" "${ENGINE_READY_SECS}s" \
        >> "$SUMMARY"

    # ------------------------------------------------------------------
    # Full container restart between runs — flushes Zenoh, ROS shared
    # memory, and any remaining zombie processes before the next run
    # ------------------------------------------------------------------
    if [ "$i" -lt "$NUM_RUNS" ]; then
        banner "Resetting container before run $((i + 1))..."
        reset_container
        sleep "$BETWEEN_RUN_WAIT"
    fi
done

# ===========================================================================
# Final summary
# ===========================================================================
{
    echo "------------------------------------------------------------"
    echo "Completed: $(date)"
} >> "$SUMMARY"

banner "All $NUM_RUNS runs complete!"
echo ""
cat "$SUMMARY"
echo ""
echo "Full results: $BASE_RESULTS"