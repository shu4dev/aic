#!/usr/bin/env bash
set -euo pipefail

WORKSPACE="${AIC_WORKSPACE:-$HOME/ws_aic/src/aic}"
BASE_RESULTS="${AIC_BASE_RESULTS:-$HOME/ws_aic/src/aic/aic_results}"
NUM_RUNS="${NUM_RUNS:-10}"
CONTAINER_NAME="${AIC_CONTAINER:-aic_eval}"
POLICY="${AIC_POLICY:-aic_example_policies.ros.CheatCode}"

ENGINE_READY_TIMEOUT="${ENGINE_READY_TIMEOUT:-120}"
ENGINE_DONE_TIMEOUT="${ENGINE_DONE_TIMEOUT:-900}"
BETWEEN_RUN_WAIT="${BETWEEN_RUN_WAIT:-5}"
CONTAINER_RESTART_WAIT="${CONTAINER_RESTART_WAIT:-8}"
ZENOH_PORT="${ZENOH_PORT:-7447}"

# BRE alternation (\|) — do NOT add -E flag to grep or this pattern breaks
ENGINE_READY_PATTERN="No node with name 'aic_model' found\|aic_engine initialized\|Waiting for aic_model\|Starting AIC Engine Run"
ENGINE_DONE_PATTERN="All Trials Processed\|process has finished cleanly"

ENGINE_PID=""
POLICY_PID=""

banner() {
    echo ""
    echo "================================================"
    echo "  $*"
    echo "================================================"
}

ts() { echo "[$(date +%T)]"; }

strip_ansi() { sed 's/\x1b\[[0-9;]*[mGKHF]//g'; }

kill_container_procs() {
    echo "$(ts) Killing simulation processes inside container..."
    timeout 15 docker exec "$CONTAINER_NAME" \
        bash -c 'pkill -9 -f "entrypoint|zenoh|zenohd|gz|rviz2|ros2|aic_engine|ruby" 2>/dev/null; true' \
        2>/dev/null || true
    sleep 2
}

reset_container() {
    echo "$(ts) Restarting container for clean state..."
    docker restart "$CONTAINER_NAME" >/dev/null 2>&1 || true
    sleep "$CONTAINER_RESTART_WAIT"
    echo "$(ts) Container restarted."
}

# ---------------------------------------------------------------------------
# teardown_run
#
# Kills the policy process first (pixi run often hangs after on_shutdown
# because ros2 run waits indefinitely for the middleware to clean up).
# Then kills the engine docker exec wrapper.
# Then kills any remaining processes inside the container.
# ---------------------------------------------------------------------------
teardown_run() {
    if [ -n "$POLICY_PID" ]; then
        echo "$(ts) Stopping policy (PID: $POLICY_PID)..."
        kill -- "$POLICY_PID" 2>/dev/null || true
        POLICY_PID=""
    fi

    if [ -n "$ENGINE_PID" ]; then
        echo "$(ts) Stopping engine (PID: $ENGINE_PID)..."
        kill -- "$ENGINE_PID" 2>/dev/null || true
        ENGINE_PID=""
    fi

    sleep 3
    kill_container_procs
}

cleanup() {
    echo ""
    echo "$(ts) Signal caught — cleaning up..."
    teardown_run
    echo "$(ts) Cleanup complete."
}

trap cleanup EXIT SIGINT SIGTERM

# ---------------------------------------------------------------------------
# wait_for_engine_ready <log_file>
# Stage 1: Zenoh port up (fastest signal).
# Stage 2: Engine log shows it is waiting for aic_model (authoritative).
# ---------------------------------------------------------------------------
wait_for_engine_ready() {
    local log_file="$1"
    local timeout="$ENGINE_READY_TIMEOUT"
    local elapsed=0

    echo "$(ts) [Stage 1/2] Waiting for Zenoh router on port ${ZENOH_PORT}..."
    until nc -z localhost "$ZENOH_PORT" 2>/dev/null; do
        if [ "$elapsed" -ge "$timeout" ]; then
            echo "$(ts) [ERROR] Timed out waiting for Zenoh port after ${elapsed}s."
            return 1
        fi
        sleep 1; elapsed=$((elapsed + 1))
    done
    echo "$(ts) [Stage 1/2] Zenoh router up after ${elapsed}s."

    echo "$(ts) [Stage 2/2] Waiting for engine ready log signal..."
    until strip_ansi < "$log_file" 2>/dev/null | grep -q "$ENGINE_READY_PATTERN"; do
        if [ "$elapsed" -ge "$timeout" ]; then
            echo "$(ts) [ERROR] Timed out waiting for engine ready signal after ${elapsed}s."
            echo "$(ts) Last 10 lines of engine log:"
            strip_ansi < "$log_file" 2>/dev/null | tail -10 || true
            return 1
        fi
        sleep 1; elapsed=$((elapsed + 1))
    done
    echo "$(ts) [Stage 2/2] Engine ready after ${elapsed}s total."
    return 0
}

# ---------------------------------------------------------------------------
# wait_for_engine_done <log_file>
#
# Polls the engine log for the completion signal emitted after all trials
# finish. Using the log as the done signal (rather than waiting on the
# docker exec PID) because the ROS 2 launch system keeps gz/rviz2 alive
# after aic_engine exits, so the docker exec process never naturally exits.
# ---------------------------------------------------------------------------
wait_for_engine_done() {
    local log_file="$1"
    local timeout="$ENGINE_DONE_TIMEOUT"
    local elapsed=0

    echo "$(ts) Waiting for all trials to complete..."
    until strip_ansi < "$log_file" 2>/dev/null | grep -q "$ENGINE_DONE_PATTERN"; do
        if [ "$elapsed" -ge "$timeout" ]; then
            echo "$(ts) [ERROR] Engine did not complete trials within ${timeout}s."
            return 1
        fi
        sleep 2; elapsed=$((elapsed + 2))
    done
    echo "$(ts) All trials completed after ${elapsed}s."
    return 0
}

if [ ! -d "$WORKSPACE" ]; then
    echo "[ERROR] Workspace not found: $WORKSPACE"
    exit 1
fi

for cmd in pixi docker nc; do
    if ! command -v "$cmd" &>/dev/null; then
        echo "[ERROR] Required command not found: '$cmd'"
        [ "$cmd" = "nc" ] && echo "       Install with: sudo apt install netcat-openbsd"
        exit 1
    fi
done

mkdir -p "$BASE_RESULTS"
export DBX_CONTAINER_MANAGER=docker

SUMMARY="$BASE_RESULTS/summary.txt"
{
    echo "CheatCode ${NUM_RUNS}-Run Summary"
    echo "Policy    : $POLICY"
    echo "Workspace : $WORKSPACE"
    echo "Started   : $(date)"
    echo "------------------------------------------------------------"
    printf "%-5s  %-38s  %-14s  %s\n" "Run" "Folder" "Status" "Engine ready (s)"
    echo "------------------------------------------------------------"
} > "$SUMMARY"

banner "Pulling aic_eval image..."
docker pull ghcr.io/intrinsic-dev/aic/aic_eval:latest

banner "Creating distrobox container (skipped if already exists)..."
distrobox create -r --nvidia \
    -i ghcr.io/intrinsic-dev/aic/aic_eval:latest \
    "$CONTAINER_NAME" || true

banner "Pre-run cleanup (clearing any leftover state)..."
kill_container_procs
reset_container

banner "Warming up distrobox entry..."
distrobox enter -r "$CONTAINER_NAME" -- bash -c "echo 'Container ready'" \
    >/dev/null 2>&1
echo "$(ts) Distrobox entry ready."

banner "Starting $NUM_RUNS runs of $POLICY"
echo "Workspace     : $WORKSPACE"
echo "Results base  : $BASE_RESULTS"
echo "Ready timeout : ${ENGINE_READY_TIMEOUT}s per run"
echo "Done timeout  : ${ENGINE_DONE_TIMEOUT}s per run"

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

    # -------------------------------------------------------------------------
    # Launch engine via docker exec with DISPLAY forwarded so Gazebo/RViz
    # windows appear on the host desktop.
    # docker exec is used instead of distrobox enter because distrobox enter
    # attempts TTY/session setup that blocks when used as a background job
    # after the first run dirtied the session state.
    # -------------------------------------------------------------------------
    docker exec \
        -e AIC_RESULTS_DIR="$RUN_DIR" \
        -e DISPLAY="${DISPLAY:-:0}" \
        "$CONTAINER_NAME" \
        bash -c "/entrypoint.sh ground_truth:=true start_aic_engine:=true" \
        >"$ENGINE_LOG" 2>&1 &
    ENGINE_PID=$!
    echo "$(ts) Engine started (PID: $ENGINE_PID)"

    START_WAIT=$SECONDS
    if ! wait_for_engine_ready "$ENGINE_LOG"; then
        echo "$(ts) [ERROR] Engine did not become ready — skipping policy launch."
        RUN_STATUS="ENGINE_FAILED"
    else
        ENGINE_READY_SECS=$((SECONDS - START_WAIT))

        # ---------------------------------------------------------------------
        # Launch policy in the BACKGROUND.
        #
        # pixi run / ros2 run hangs after the node calls on_shutdown because
        # the ROS 2 middleware (Zenoh) holds open connections that keep the
        # executor alive. Running in the background lets us kill it explicitly
        # once the engine signals all trials are done, rather than waiting
        # forever for pixi run to exit on its own.
        # ---------------------------------------------------------------------
        echo "$(ts) Launching policy..."
        (
            cd "$WORKSPACE"
            pixi run ros2 run aic_model aic_model \
                --ros-args \
                -p use_sim_time:=true \
                -p policy:="$POLICY"
        ) >"$POLICY_LOG" 2>&1 &
        POLICY_PID=$!
        echo "$(ts) Policy started (PID: $POLICY_PID)"

        # Wait for engine to finish all trials, then kill the policy
        if wait_for_engine_done "$ENGINE_LOG"; then
            echo "$(ts) Engine done — stopping policy..."
            sleep 2
            kill -- "$POLICY_PID" 2>/dev/null || true
            wait "$POLICY_PID" 2>/dev/null || true
            POLICY_PID=""
        else
            RUN_STATUS="ENGINE_TIMEOUT"
        fi
    fi

    teardown_run

    echo "$(ts) Run $i complete."
    echo "  Engine log  : $ENGINE_LOG"
    echo "  Policy log  : $POLICY_LOG"
    echo "  Results     : $RUN_DIR"
    echo "  Status      : $RUN_STATUS"
    echo "  Engine ready: ${ENGINE_READY_SECS}s"

    printf "%-5s  %-38s  %-14s  %s\n" \
        "$i" "${RUN_LABEL}_${TIMESTAMP}" "$RUN_STATUS" "${ENGINE_READY_SECS}s" \
        >> "$SUMMARY"

    if [ "$i" -lt "$NUM_RUNS" ]; then
        banner "Cleaning up before run $((i + 1))..."
        kill_container_procs
        sleep "$BETWEEN_RUN_WAIT"
    fi
done

{
    echo "------------------------------------------------------------"
    echo "Completed: $(date)"
} >> "$SUMMARY"

banner "All $NUM_RUNS runs complete!"
echo ""
cat "$SUMMARY"
echo ""
echo "Full results: $BASE_RESULTS"