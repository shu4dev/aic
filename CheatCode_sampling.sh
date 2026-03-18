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
# Kill all simulation processes inside the container.
# Uses timeout to prevent docker exec from hanging if the container runtime
# is in a transitional state after the host-side wrapper was killed.
# ---------------------------------------------------------------------------
kill_container_procs() {
    echo "$(ts) Killing simulation processes inside container..."
    timeout 15 docker exec "$CONTAINER_NAME" \
        bash -c 'pkill -9 -f "entrypoint|zenoh|zenohd|gz|rviz2|ros2|aic_engine|ruby" 2>/dev/null; true' \
        2>/dev/null || true
    sleep 2
}

# ---------------------------------------------------------------------------
# Full container restart — only used ONCE before the main loop to clear state
# from any prior interrupted session. NOT called between runs because it breaks
# the distrobox session layer, causing the next distrobox enter to hang.
# ---------------------------------------------------------------------------
reset_container() {
    echo "$(ts) Restarting container for clean state..."
    docker restart "$CONTAINER_NAME" >/dev/null 2>&1 || true
    sleep "$CONTAINER_RESTART_WAIT"
    echo "$(ts) Container restarted."
}

# ---------------------------------------------------------------------------
# teardown_run: stops engine and container processes after each run.
#
# Why no `wait` after kill:
#   We send SIGKILL to the entire process group, so the process is immediately
#   dead. Calling `wait` on an already-dead process can still block in bash if
#   the PID has already been reaped by a prior wait or by the shell's async
#   job control. Skipping `wait` is safe here — the shell will reap zombies
#   automatically at its next opportunity.
#
# Why sleep before docker exec:
#   After killing the host-side distrobox wrapper, the container runtime needs
#   a moment to settle. Issuing `docker exec` immediately against a container
#   whose primary connection just received SIGKILL can cause docker exec to
#   hang indefinitely. A 3-second pause + 15-second timeout on the exec itself
#   prevents this from blocking the loop.
# ---------------------------------------------------------------------------
teardown_run() {
    if [ -n "$ENGINE_PID" ]; then
        echo "$(ts) Stopping engine (host wrapper PID: $ENGINE_PID)..."
        local pgid
        pgid=$(ps -o pgid= -p "$ENGINE_PID" 2>/dev/null | tr -d ' ') || true
        if [ -n "$pgid" ]; then
            kill -- -"$pgid" 2>/dev/null || true
        fi
        ENGINE_PID=""
    fi
    # Let the container runtime settle before issuing docker exec
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
#
# Stage 1: Poll Zenoh port with nc — fastest signal, catches port conflicts early.
# Stage 2: Poll engine log for aic_engine's ready string using BRE grep.
#
# Uses grep -q (BRE) not grep -qE (ERE) because \| is OR in BRE but a literal
# backslash-pipe in ERE — using -E caused every run to time out at Stage 2.
# Log is ANSI-stripped before matching to avoid interference from color codes.
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

# Pre-loop: guarantee clean state from any prior interrupted session.
# docker restart is ONLY called here — not between runs (see teardown comment).
banner "Pre-run cleanup (clearing any leftover state)..."
kill_container_procs
reset_container

# Warmup: the very first distrobox enter is slow (namespace init, home mounts).
# Absorb that cost here so run 1's engine startup isn't penalized.
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

    # NOTE: Added 'setsid' here to run the engine in its own process group
    setsid distrobox enter -r "$CONTAINER_NAME" -- bash -c \
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

    # Between runs: kill container processes only — NO docker restart.
    # pkill frees Zenoh port and ROS 2 shared memory without touching the
    # distrobox session layer that docker restart would break.
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