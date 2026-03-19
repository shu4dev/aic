#!/usr/bin/env bash
set -euo pipefail

WORKSPACE="${AIC_WORKSPACE:-$HOME/ws_aic/src/aic}"
BASE_RESULTS="${AIC_BASE_RESULTS:-$HOME/ws_aic/src/aic/aic_results}"
NUM_RUNS="${NUM_RUNS:-10}"
CONTAINER_NAME="${AIC_CONTAINER:-aic_eval}"
POLICY="${AIC_POLICY:-aic_example_policies.ros.CheatCode}"

ENGINE_READY_TIMEOUT="${ENGINE_READY_TIMEOUT:-300}"
RUN_COMPLETE_TIMEOUT="${RUN_COMPLETE_TIMEOUT:-600}"
# Increased from 8 to 20 to absorb cold-start Docker/distrobox init on fresh boot
CONTAINER_RESTART_WAIT="${CONTAINER_RESTART_WAIT:-20}"
BETWEEN_RUN_WAIT="${BETWEEN_RUN_WAIT:-5}"
ZENOH_PORT="${ZENOH_PORT:-7447}"

# Display forwarding — captured from the launching terminal (e.g. DCV session).
# DCV typically sets DISPLAY=:1; a plain desktop is usually :0.
# Override via env var if needed: DISPLAY=:2 ./CheatCode_sampling.sh
DISPLAY="${DISPLAY:-:0}"
XAUTHORITY="${XAUTHORITY:-$HOME/.Xauthority}"

# BRE alternation (\|) — do NOT add -E flag to grep or this pattern breaks
ENGINE_READY_PATTERN="No node with name 'aic_model' found\|aic_engine initialized\|Waiting for aic_model\|Starting AIC Engine Run"

# ---------------------------------------------------------------------------
# IMPORTANT: Pattern MUST match only [aic_engine-N] specifically.
#
# Other nodes (create-5, spawner-10) also emit "process has finished cleanly"
# within seconds of startup — long before aic_engine finishes all trials.
# A broad pattern causes wait_for_run_complete to return 0 immediately,
# the policy gets killed before executing, and result files are empty.
# ---------------------------------------------------------------------------
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
# teardown_run: stops policy, engine, and container processes after each run.
#
# Policy is killed first (lighter weight), then the engine wrapper PGID,
# then container procs are purged via docker exec.
#
# PGID self-kill guard:
#   `kill -- -"$pgid"` sends SIGTERM to the entire process group. Because
#   bash runs without job control (no `set -m`), background jobs inherit the
#   script's own PGID. If the engine PGID matches the script's own PGID,
#   we would kill ourselves and fire the SIGTERM trap mid-run, causing
#   teardown_run to be called twice and leaving ENGINE_PID un-cleared.
#   The guard compares engine PGID against the script's own PGID ($$ PGID)
#   and falls back to killing just the engine PID directly when they match.
#
# Why no `wait` after engine kill:
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
    # Kill policy first (lighter weight — just a ros2 run process)
    if [ -n "$POLICY_PID" ]; then
        echo "$(ts) Stopping policy (host PID: $POLICY_PID)..."
        kill "$POLICY_PID" 2>/dev/null || true
        wait "$POLICY_PID" 2>/dev/null || true
        POLICY_PID=""
    fi

    # Kill engine wrapper — guard against killing our own process group
    if [ -n "$ENGINE_PID" ]; then
        echo "$(ts) Stopping engine (host wrapper PID: $ENGINE_PID)..."
        local pgid script_pgid
        pgid=$(ps -o pgid= -p "$ENGINE_PID" 2>/dev/null | tr -d ' ') || true
        script_pgid=$(ps -o pgid= -p $$ 2>/dev/null | tr -d ' ') || true
        if [ -n "$pgid" ] && [ "$pgid" != "$script_pgid" ]; then
            # Safe: engine has its own process group — kill the whole group
            kill -- -"$pgid" 2>/dev/null || true
        elif [ -n "$ENGINE_PID" ]; then
            # PGID matches script or couldn't be determined — kill only the
            # engine wrapper PID directly to avoid self-termination
            kill "$ENGINE_PID" 2>/dev/null || true
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

# ---------------------------------------------------------------------------
# wait_for_run_complete <log_file>
#
# Polls the engine log until aic_engine node specifically reports it has
# finished cleanly. Pattern is scoped to [aic_engine-N] to avoid false
# positives from other nodes (create-5, spawner-10) that also emit
# "process has finished cleanly" within seconds of startup.
#
# Neither the engine wrapper nor the policy self-terminate — the log is the
# only reliable signal that all trials are done.
#
# Returns 0 on success, 1 on timeout.
# ---------------------------------------------------------------------------
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

# Pre-loop: guarantee clean state from any prior interrupted session.
# docker restart is ONLY called here — not between runs (see teardown comment).
banner "Pre-run cleanup (clearing any leftover state)..."
kill_container_procs
reset_container

# ---------------------------------------------------------------------------
# Warmup: absorb the cold-start cost of the first distrobox entry.
#
# Two passes are used intentionally:
#   Pass 1 — triggers namespace init, home mount setup, and group resolution
#             that distrobox defers until the first `enter` after a restart.
#   Pass 2 — confirms the session layer is fully settled and subsequent
#             `distrobox enter` calls will be fast (~1-2s overhead).
# Without this, run 1 on a fresh system boot can fail ENGINE_FAILED because
# the container init overhead pushes the first entrypoint.sh call past the
# Zenoh port timeout window.
# ---------------------------------------------------------------------------
banner "Warming up distrobox entry (two passes for cold-start safety)..."
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

    # Start engine in the background — it never self-terminates.
    # DISPLAY and XAUTHORITY are forwarded explicitly so Gazebo and RViz
    # can open windows in the DCV (or local desktop) session even though
    # this process is launched as a background job with redirected output.
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

        # Start policy in the background — ros2 run never self-terminates
        # after a lifecycle shutdown, so it must be killed explicitly below.
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

        # Block here until aic_engine specifically prints its completion line.
        # Pattern is scoped to [aic_engine-N] — see RUN_COMPLETE_PATTERN note
        # at the top of this script for why a broad pattern causes empty results.
        if ! wait_for_run_complete "$ENGINE_LOG"; then
            RUN_STATUS="RUN_TIMEOUT"
        fi

        echo "$(ts) Run complete ($RUN_STATUS) — stopping policy..."

        # Kill policy explicitly now that the run is done
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