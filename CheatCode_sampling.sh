#!/usr/bin/env bash
# =============================================================================
# run_cheatcode_10x.sh
#
# Automates the standard two-terminal workflow for 10 runs of CheatCode:
#
#   [Terminal 1 equivalent - background]
#     distrobox enter -r aic_eval -- bash -c "
#       export AIC_RESULTS_DIR=<run_dir> &&
#       /entrypoint.sh ground_truth:=true start_aic_engine:=true"
#
#   [Terminal 2 equivalent - foreground, after engine is ready]
#     cd ~/ws_aic/src/aic
#     pixi run ros2 run aic_model aic_model \
#       --ros-args -p use_sim_time:=true \
#       -p policy:=aic_example_policies.ros.CheatCode
#
# Each run stores results in its own folder:
#   ~/aic_results/run_01_YYYYMMDD_HHMMSS/
#   ~/aic_results/run_02_YYYYMMDD_HHMMSS/
#   ...
#
# PREREQUISITES:
#   - Docker + Distrobox installed and the aic_eval image already pulled:
#       docker pull ghcr.io/intrinsic-dev/aic/aic_eval:latest
#       distrobox create -r --nvidia \
#         -i ghcr.io/intrinsic-dev/aic/aic_eval:latest aic_eval
#   - Pixi installed and workspace set up at ~/ws_aic/src/aic
#
# USAGE:
#   chmod +x run_cheatcode_10x.sh
#   ./run_cheatcode_10x.sh
#
# OPTIONAL OVERRIDES (set as env vars before running):
#   AIC_WORKSPACE      path to aic repo        (default: ~/ws_aic/src/aic)
#   AIC_BASE_RESULTS   base results folder     (default: ~/aic_results)
#   NUM_RUNS           number of runs          (default: 10)
#   ENGINE_WAIT_SECS   seconds to wait for the (default: 20)
#                      engine to be ready before
#                      launching the policy
# =============================================================================

set -euo pipefail

# ---------------------------------------------------------------------------
# Config
# ---------------------------------------------------------------------------
WORKSPACE="${AIC_WORKSPACE:-$HOME/ws_aic/src/aic}"
BASE_RESULTS="${AIC_BASE_RESULTS:-$HOME/aic_results}"
NUM_RUNS="${NUM_RUNS:-10}"
ENGINE_WAIT_SECS="${ENGINE_WAIT_SECS:-20}"
POLICY="aic_example_policies.ros.CheatCode"

# ---------------------------------------------------------------------------
# Sanity checks
# ---------------------------------------------------------------------------
if [ ! -d "$WORKSPACE" ]; then
    echo "[ERROR] Workspace not found: $WORKSPACE"
    echo "        Set AIC_WORKSPACE or clone the repo to ~/ws_aic/src/aic"
    exit 1
fi
if ! command -v pixi &>/dev/null; then
    echo "[ERROR] 'pixi' not found in PATH. Please install Pixi first."
    exit 1
fi
if ! command -v distrobox &>/dev/null; then
    echo "[ERROR] 'distrobox' not found in PATH. Please install Distrobox first."
    exit 1
fi

mkdir -p "$BASE_RESULTS"

# ---------------------------------------------------------------------------
# Summary file
# ---------------------------------------------------------------------------
SUMMARY="$BASE_RESULTS/summary.txt"
{
    echo "CheatCode 10-Run Summary"
    echo "Policy : $POLICY"
    echo "Started: $(date)"
    echo "------------------------------------------------------------"
    printf "%-5s  %-36s  %s\n" "Run" "Folder" "Status"
    echo "------------------------------------------------------------"
} > "$SUMMARY"

# ---------------------------------------------------------------------------
# Helper
# ---------------------------------------------------------------------------
banner() {
    echo ""
    echo "================================================"
    echo "  $*"
    echo "================================================"
}

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

    # ------------------------------------------------------------------
    # TERMINAL 1 equivalent:
    # Launch the eval container + aic_engine in the background.
    # AIC_RESULTS_DIR is passed into the container so the engine writes
    # results into this run's dedicated folder.
    # ------------------------------------------------------------------
    echo "[$(date +%T)] Starting eval container + engine in background..."
    export DBX_CONTAINER_MANAGER=docker
    docker pull ghcr.io/intrinsic-dev/aic/aic_eval:latest
    distrobox create -r --nvidia -i ghcr.io/intrinsic-dev/aic/aic_eval:latest aic_eval
    distrobox enter -r aic_eval -- bash -c \
        "export AIC_RESULTS_DIR='$RUN_DIR' && /entrypoint.sh ground_truth:=true start_aic_engine:=true" \
        >"$ENGINE_LOG" 2>&1 &
    ENGINE_PID=$!
    echo "[$(date +%T)] Engine PID: $ENGINE_PID"

    # ------------------------------------------------------------------
    # Wait for the engine to be ready.
    # The engine prints "No node with name 'aic_model' found. Retrying..."
    # while waiting — give it a fixed head start before connecting.
    # Tune ENGINE_WAIT_SECS if your machine is slower/faster.
    # ------------------------------------------------------------------
    echo "[$(date +%T)] Waiting ${ENGINE_WAIT_SECS}s for engine to initialize..."
    sleep "$ENGINE_WAIT_SECS"

    # ------------------------------------------------------------------
    # TERMINAL 2 equivalent:
    # Run the policy. It connects to the engine, completes all trials,
    # then exits on its own.
    # ------------------------------------------------------------------
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

    # ------------------------------------------------------------------
    # Wait for the engine background process to finish writing results
    # and shut down cleanly before starting the next run.
    # ------------------------------------------------------------------
    echo "[$(date +%T)] Waiting for engine to shut down..."
    wait "$ENGINE_PID" || true   # engine may exit non-zero; that is OK

    echo "[$(date +%T)] Run $i complete."
    echo "  Engine log : $ENGINE_LOG"
    echo "  Policy log : $POLICY_LOG"
    echo "  Results    : $RUN_DIR"

    # Append to summary table
    printf "%-5s  %-36s  %s\n" \
        "$i" "${RUN_LABEL}_${TIMESTAMP}" "$RUN_STATUS" >> "$SUMMARY"

    # Short pause so Gazebo/ROS state fully tears down before next run
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