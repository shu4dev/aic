#!/usr/bin/env bash
set -euo pipefail

WORKSPACE="${AIC_WORKSPACE:-$HOME/ws_aic/src/aic}"
BASE_RESULTS="${AIC_BASE_RESULTS:-$HOME/ws_aic/src/aic/aic_results}"
NUM_RUNS="${NUM_RUNS:-10}"
CONTAINER_NAME="${AIC_CONTAINER:-aic_eval}"
POLICY="${AIC_POLICY:-aic_example_policies.ros.CheatCode}"

ENGINE_READY_TIMEOUT="${ENGINE_READY_TIMEOUT:-300}"
RUN_COMPLETE_TIMEOUT="${RUN_COMPLETE_TIMEOUT:-400}"
CONTAINER_RESTART_WAIT="${CONTAINER_RESTART_WAIT:-20}"
BETWEEN_RUN_WAIT="${BETWEEN_RUN_WAIT:-5}"
ZENOH_PORT="${ZENOH_PORT:-7447}"

DISPLAY="${DISPLAY:-:0}"
XAUTHORITY="${XAUTHORITY:-$HOME/.Xauthority}"

# BRE alternation — do NOT use grep -E or this pattern breaks
ENGINE_READY_PATTERN="No node with name 'aic_model' found\|aic_engine initialized\|Waiting for aic_model\|Starting AIC Engine Run"

# Must match only [aic_engine-N] — other nodes also emit "finished cleanly" early
RUN_COMPLETE_PATTERN="\[aic_engine-[0-9]*\]: process has finished cleanly"

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

# Only called once before the loop — NOT between runs (breaks distrobox session)
reset_container() {
    echo "$(ts) Restarting container for clean state..."
    docker restart "$CONTAINER_NAME" >/dev/null 2>&1 || true
    sleep "$CONTAINER_RESTART_WAIT"
    echo "$(ts) Container restarted."
}

teardown_run() {
    if [ -n "$POLICY_PID" ]; then
        echo "$(ts) Stopping policy (host PID: $POLICY_PID)..."
        kill "$POLICY_PID" 2>/dev/null || true
        wait "$POLICY_PID" 2>/dev/null || true
        POLICY_PID=""
    fi

    # Guard against killing our own process group
    if [ -n "$ENGINE_PID" ]; then
        echo "$(ts) Stopping engine (host wrapper PID: $ENGINE_PID)..."
        local pgid script_pgid
        pgid=$(ps -o pgid= -p "$ENGINE_PID" 2>/dev/null | tr -d ' ') || true
        script_pgid=$(ps -o pgid= -p $$ 2>/dev/null | tr -d ' ') || true
        if [ -n "$pgid" ] && [ "$pgid" != "$script_pgid" ]; then
            kill -- -"$pgid" 2>/dev/null || true
        elif [ -n "$ENGINE_PID" ]; then
            kill "$ENGINE_PID" 2>/dev/null || true
        fi
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

wait_for_run_complete() {
    local log_file="$1"
    local timeout="$RUN_COMPLETE_TIMEOUT"
    local elapsed=0

    echo "$(ts) Waiting for aic_engine to finish all trials (timeout: ${timeout}s)..."
    until strip_ansi < "$log_file" 2>/dev/null | grep -q "$RUN_COMPLETE_PATTERN"; do
        if [ "$elapsed" -ge "$timeout" ]; then
            echo "$(ts) [ERROR] Run timed out after ${elapsed}s — aic_engine never signalled completion."
            echo "$(ts) Last 10 lines of engine log:"
            strip_ansi < "$log_file" 2>/dev/null | tail -10 || true
            return 1
        fi
        sleep 2
        elapsed=$((elapsed + 2))
    done
    echo "$(ts) aic_engine finished all trials after ${elapsed}s."
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

banner "Pre-run cleanup (clearing any leftover state)..."
kill_container_procs
reset_container

# Two warmup passes to absorb cold-start distrobox init cost
banner "Warming up distrobox entry..."
distrobox enter -r "$CONTAINER_NAME" -- bash -c "echo 'Warmup pass 1 complete'" \
    >/dev/null 2>&1
sleep 5
distrobox enter -r "$CONTAINER_NAME" -- bash -c "echo 'Warmup pass 2 complete'" \
    >/dev/null 2>&1
echo "$(ts) Distrobox entry fully warmed up."

banner "Starting $NUM_RUNS runs of $POLICY"
echo "Workspace     : $WORKSPACE"
echo "Results base  : $BASE_RESULTS"
echo "Ready timeout : ${ENGINE_READY_TIMEOUT}s per run"
echo "Run timeout   : ${RUN_COMPLETE_TIMEOUT}s per run"

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
        "export DISPLAY='${DISPLAY}' \
         && export XAUTHORITY='${XAUTHORITY}' \
         && export AIC_RESULTS_DIR='$RUN_DIR' \
         && /entrypoint.sh ground_truth:=true start_aic_engine:=true" \
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
        ) >"$POLICY_LOG" 2>&1 &
        POLICY_PID=$!
        echo "$(ts) Policy started (host PID: $POLICY_PID)"

        if ! wait_for_run_complete "$ENGINE_LOG"; then
            RUN_STATUS="RUN_TIMEOUT"
        fi

        echo "$(ts) Run complete ($RUN_STATUS) — stopping policy..."

        if [ -n "$POLICY_PID" ]; then
            kill "$POLICY_PID" 2>/dev/null || true
            wait "$POLICY_PID" 2>/dev/null || true
            POLICY_PID=""
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