#!/usr/bin/env bash
set -euo pipefail

WORKSPACE="${AIC_WORKSPACE:-$HOME/ws_aic/src/aic}"
BASE_RESULTS="${AIC_BASE_RESULTS:-$HOME/ws_aic/src/aic/aic_results}"
NUM_RUNS="${NUM_RUNS:-10}"
CONTAINER_NAME="${AIC_CONTAINER:-aic_eval}"
POLICY="${AIC_POLICY:-aic_example_policies.ros.CheatCode}"

ENGINE_READY_TIMEOUT="${ENGINE_READY_TIMEOUT:-120}"
CONTAINER_RESTART_WAIT="${CONTAINER_RESTART_WAIT:-8}"
BETWEEN_RUN_WAIT="${BETWEEN_RUN_WAIT:-5}"
ZENOH_PORT="${ZENOH_PORT:-7447}"

# BRE alternation (\|) — do NOT add -E flag to grep or this pattern breaks
ENGINE_READY_PATTERN="No node with name 'aic_model' found\|aic_engine initialized\|Waiting for aic_model\|Starting AIC Engine Run"

ENGINE_PID=""

banner() {
    echo ""
    echo "================================================"
    echo "  $*"
    echo "================================================"
}

ts() { echo "[$(date +%T)]"; }

strip_ansi() { sed 's/\x1b\[[0-9;]*[mGKHF]//g'; }

# ---------------------------------------------------------------------------
# Kill all simulation processes inside the container via docker exec.
# This frees Zenoh port bindings and ROS 2 shared memory without needing
# a full container restart, which would break the distrobox session state.
# ---------------------------------------------------------------------------
kill_container_procs() {
    echo "$(ts) Killing simulation processes inside container..."
    docker exec "$CONTAINER_NAME" \
        bash -c 'pkill -9 -f "entrypoint|zenoh|zenohd|gz|rviz2|ros2|aic_engine|ruby" 2>/dev/null; true' \
        2>/dev/null || true
    sleep 2
}

# ---------------------------------------------------------------------------
# Full container restart — only used ONCE before the main loop.
# docker restart breaks the distrobox session layer if called between runs,
# causing the next distrobox enter to hang. Between runs, kill_container_procs
# is sufficient since killing the processes frees all shared state.
# ---------------------------------------------------------------------------
reset_container() {
    echo "$(ts) Restarting container for clean state..."
    docker restart "$CONTAINER_NAME" >/dev/null 2>&1 || true
    sleep "$CONTAINER_RESTART_WAIT"
    echo "$(ts) Container restarted."
}

teardown_run() {
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
#
# Stage 1: Poll Zenoh port with nc — comes up first, fastest signal.
#          A port conflict from a prior run is caught here at ~1s.
# Stage 2: Poll engine log for the exact string aic_engine prints when
#          waiting for aic_model. Uses BRE grep (no -E) so \| works as OR.
#          Log is ANSI-stripped before matching to avoid color code interference.
#
# Why no -E: grep -qE with \| treats \| as a literal backslash-pipe in ERE
# mode (it is NOT an alternation operator in ERE). This caused every run to
# time out at Stage 2 even though the log contained the ready signal.
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
        sleep 1
        elapsed=$((elapsed + 1))
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
        sleep 1
        elapsed=$((elapsed + 1))
    done
    echo "$(ts) [Stage 2/2] Engine ready after ${elapsed}s total."
    return 0
}

if [ ! -d "$WORKSPACE" ]; then
    echo "[ERROR] Workspace not found: $WORKSPACE"
    exit 1
fi

for cmd in pixi distrobox docker nc; do
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

# ---------------------------------------------------------------------------
# Pre-loop: guarantee a clean container state before any runs begin.
# This handles leftover state from any previously interrupted session.
# docker restart is ONLY done here — not between runs (see teardown comment).
# ---------------------------------------------------------------------------
banner "Pre-run cleanup (clearing any leftover state)..."
kill_container_procs
reset_container

# ---------------------------------------------------------------------------
# Warmup distrobox entry: the very first distrobox enter is significantly
# slower because distrobox initialises user namespaces, mounts home dirs,
# and syncs rc files. Doing this once here as a blocking no-op ensures all
# subsequent entries in the main loop start at normal speed.
# ---------------------------------------------------------------------------
banner "Warming up distrobox entry (first entry is always slow)..."
distrobox enter -r "$CONTAINER_NAME" -- bash -c "echo 'Container ready'" \
    >/dev/null 2>&1
echo "$(ts) Distrobox entry ready."

banner "Starting $NUM_RUNS runs of $POLICY"
echo "Workspace     : $WORKSPACE"
echo "Results base  : $BASE_RESULTS"
echo "Ready timeout : ${ENGINE_READY_TIMEOUT}s per run"

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

    distrobox enter -r "$CONTAINER_NAME" -- bash -c \
        "export AIC_RESULTS_DIR='$RUN_DIR' && /entrypoint.sh ground_truth:=true start_aic_engine:=true" \
        >"$ENGINE_LOG" 2>&1 &
    ENGINE_PID=$!
    echo "$(ts) Engine started (host wrapper PID: $ENGINE_PID)"

    START_WAIT=$SECONDS
    if ! wait_for_engine_ready "$ENGINE_LOG"; then
        echo "$(ts) [ERROR] Engine did not become ready — skipping policy launch."
        RUN_STATUS="ENGINE_FAILED"
    else
        ENGINE_READY_SECS=$((SECONDS - START_WAIT))

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

    # -------------------------------------------------------------------------
    # Between runs: kill all simulation processes inside the container to free
    # Zenoh port bindings and ROS 2 shared memory. Do NOT call reset_container
    # (docker restart) here — it breaks distrobox's session layer and causes
    # the next distrobox enter to hang indefinitely.
    # -------------------------------------------------------------------------
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