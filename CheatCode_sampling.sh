#!/usr/bin/env bash
# run_cheatcode_10x.sh — Runs CheatCode policy 10 times, one folder per run.

set -euo pipefail

# ---------------------------------------------------------------------------
# Config
# ---------------------------------------------------------------------------
WORKSPACE="${AIC_WORKSPACE:-$HOME/ws_aic/src/aic}"
BASE_RESULTS="${AIC_BASE_RESULTS:-$HOME/ws_aic/src/aic/aic_results}"
NUM_RUNS="${NUM_RUNS:-10}"
ENGINE_WAIT_SECS="${ENGINE_WAIT_SECS:-20}"
POLICY="aic_example_policies.ros.CheatCode"
ENGINE_PID=""

# ---------------------------------------------------------------------------
# Cleanup — kills engine process group on exit, Ctrl+C, or error
# ---------------------------------------------------------------------------
cleanup() {
    echo ""
    echo "[$(date +%T)] Cleaning up..."
    if [ -n "$ENGINE_PID" ]; then
        kill -- -$(ps -o pgid= -p "$ENGINE_PID" | tr -d ' ') 2>/dev/null || true
        wait "$ENGINE_PID" 2>/dev/null || true
    fi
    echo "[$(date +%T)] Cleanup done."
}

trap cleanup EXIT SIGINT SIGTERM

# ---------------------------------------------------------------------------
# Sanity checks
# ---------------------------------------------------------------------------
if [ ! -d "$WORKSPACE" ]; then
    echo "[ERROR] Workspace not found: $WORKSPACE"
    exit 1
fi
if ! command -v pixi &>/dev/null; then
    echo "[ERROR] 'pixi' not found in PATH."
    exit 1
fi
if ! command -v distrobox &>/dev/null; then
    echo "[ERROR] 'distrobox' not found in PATH."
    exit 1
fi

mkdir -p "$BASE_RESULTS"

# ---------------------------------------------------------------------------
# Summary file
# ---------------------------------------------------------------------------
SUMMARY="$BASE_RESULTS/summary.txt"
{
    echo "CheatCode $NUM_RUNS-Run Summary"
    echo "Policy : $POLICY"
    echo "Started: $(date)"
    echo "------------------------------------------------------------"
    printf "%-5s  %-36s  %s\n" "Run" "Folder" "Status"
    echo "------------------------------------------------------------"
} > "$SUMMARY"

banner() {
    echo ""
    echo "================================================"
    echo "  $*"
    echo "================================================"
}

# ---------------------------------------------------------------------------
# Setup
# ---------------------------------------------------------------------------
export DBX_CONTAINER_MANAGER=docker

banner "Pulling aic_eval image..."
docker pull ghcr.io/intrinsic-dev/aic/aic_eval:latest

banner "Creating distrobox container (skipped if already exists)..."
distrobox create -r --nvidia -i ghcr.io/intrinsic-dev/aic/aic_eval:latest aic_eval || true

# ---------------------------------------------------------------------------
# Warmup run — boots the full engine once then shuts down cleanly
# so subsequent runs start fast.
# ---------------------------------------------------------------------------
banner "Warming up container..."
distrobox enter -r aic_eval -- bash -c \
    "/entrypoint.sh ground_truth:=true start_aic_engine:=true" \
    >/dev/null 2>&1 &
ENGINE_PID=$!

echo "[$(date +%T)] Warmup PID: $ENGINE_PID — waiting ${ENGINE_WAIT_SECS}s..."
sleep "$ENGINE_WAIT_SECS"

echo "[$(date +%T)] Shutting down warmup..."
kill -- -$(ps -o pgid= -p "$ENGINE_PID" | tr -d ' ') 2>/dev/null || true
wait "$ENGINE_PID" || true
ENGINE_PID=""

echo "[$(date +%T)] Warmup complete. Sleeping 10s before official runs..."
sleep 10

# ---------------------------------------------------------------------------
# Main loop
# ---------------------------------------------------------------------------
banner "Starting $NUM_RUNS runs of $POLICY"
echo "Workspace   : $WORKSPACE"
echo "Results base: $BASE_RESULTS"

for i in $(seq 1 "$NUM_RUNS"); do
    TIMESTAMP=$(date +%Y%m%d_%H%M%S)
    RUN_LABEL="run_$(printf '%02d' "$i")"
    RUN_DIR="$BASE_RESULTS/${RUN_LABEL}_${TIMESTAMP}"
    mkdir -p "$RUN_DIR"

    ENGINE_LOG="$RUN_DIR/engine.log"
    POLICY_LOG="$RUN_DIR/policy.log"

    banner "Run $i / $NUM_RUNS"
    echo "Results dir : $RUN_DIR"

    # Terminal 1 equivalent
    distrobox enter -r aic_eval -- bash -c \
        "export AIC_RESULTS_DIR='$RUN_DIR' && /entrypoint.sh ground_truth:=true start_aic_engine:=true" \
        >"$ENGINE_LOG" 2>&1 &
    ENGINE_PID=$!
    echo "[$(date +%T)] Engine PID: $ENGINE_PID"

    echo "[$(date +%T)] Waiting ${ENGINE_WAIT_SECS}s for engine to initialize..."
    sleep "$ENGINE_WAIT_SECS"

    # Terminal 2 equivalent
    echo "[$(date +%T)] Launching policy..."
    RUN_STATUS="SUCCESS"
    (
        cd "$WORKSPACE"
        pixi run ros2 run aic_model aic_model \
            --ros-args \
            -p use_sim_time:=true \
            -p policy:="$POLICY"
    ) >"$POLICY_LOG" 2>&1 || RUN_STATUS="FAILED"

    echo "[$(date +%T)] Policy exited — $RUN_STATUS"

    echo "[$(date +%T)] Shutting down engine..."
    kill -- -$(ps -o pgid= -p "$ENGINE_PID" | tr -d ' ') 2>/dev/null || true
    wait "$ENGINE_PID" || true
    ENGINE_PID=""

    echo "[$(date +%T)] Run $i complete."
    echo "  Engine log : $ENGINE_LOG"
    echo "  Policy log : $POLICY_LOG"
    echo "  Results    : $RUN_DIR"

    printf "%-5s  %-36s  %s\n" "$i" "${RUN_LABEL}_${TIMESTAMP}" "$RUN_STATUS" >> "$SUMMARY"

    if [ "$i" -lt "$NUM_RUNS" ]; then
        echo "[$(date +%T)] Sleeping 10s before next run..."
        sleep 10
    fi
done

# ---------------------------------------------------------------------------
# Final summary
# ---------------------------------------------------------------------------
echo "Completed: $(date)" >> "$SUMMARY"

banner "All $NUM_RUNS runs complete!"
echo ""
cat "$SUMMARY"
echo ""
echo "Full results: $BASE_RESULTS"