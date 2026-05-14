#!/bin/bash
# Orchestrate N concurrent run_parallel.sh workers on one Lambda box.
#
# What this does:
#   1. Splits aic_engine/config/train.yaml into NUM_WORKERS parts via
#      scripts/split_trials.py (always re-runs so part count matches N).
#   2. Launches NUM_WORKERS copies of scripts/run_parallel.sh in the
#      background, each pinned to its own:
#        - PART_FILE       → train_part_<i>.yaml
#        - PART_INDEX      → i  (drives NAME, RESULTS, ROUTER_PORT defaults)
#        - WORKER_CPUS     → ⌊HOST_CPUS / N⌋ so workers don't oversubscribe
#        - WORKER_MEM      → ⌊HOST_MEM_GB / N⌋g
#      Worker 0 keeps --network host on port 7447; workers ≥1 use bridge
#      networking with --publish 127.0.0.1:744(7+i):7447 so all N routers
#      coexist. GPU is shared via --gpus all (Docker doesn't split a single
#      A100 between containers).
#   3. Waits for every worker to finish, then runs scripts/merge_scores.py.
#
# Why bridge instead of host networking for workers ≥1:
#   The image entrypoint (docker/aic_eval/zenoh_config_router.sh) hardcodes
#   the in-container Zenoh listen endpoint to tcp/[::]:7447 and rebuilds
#   ZENOH_CONFIG_OVERRIDE from scratch, so we can't override the port via
#   `docker run -e`. Bridge + --publish maps a distinct host port to the
#   container's hardcoded 7447 instead.
#
# Expected speed-up on 1× A100 / 30 vCPU / 200 GB:
#   GPU contention means N=4 workers do NOT run 4× faster than N=1. Headless
#   Gazebo + a small ACT policy each take ~3-5 GB VRAM and a slice of the
#   A100's SMs, so 3-4 in parallel usually yields ~2-2.5× wallclock speed-up
#   versus serial. Worth measuring on your actual workload.
#
# Usage:
#   bash scripts/run_parallel_multi.sh
#   NUM_WORKERS=2 bash scripts/run_parallel_multi.sh
#   NUM_WORKERS=4 POLICY=aic_example_policies.ros.WaveArm \
#       bash scripts/run_parallel_multi.sh
#
# Environment overrides (all optional):
#   NUM_WORKERS      worker count               (default: 4)
#   INPUT_CONFIG     trial config to split      (default: aic_engine/config/train.yaml)
#   SPLIT_DIR        where part files are written
#                                               (default: /home/ubuntu/aic-data/aic_split_configs)
#   RESULTS_BASE     parent dir for per-worker aic_results_<i>/
#                                               (default: /home/ubuntu/aic-data)
#   MERGED_OUTPUT    final merged scores YAML   (default: $RESULTS_BASE/merged_score.yaml)
#   STAGGER_SECS     seconds to wait between worker launches so CPU/disk init
#                    storms don't overlap. Default: 5.
#                    NOTE: real concurrent multi-worker execution on one host
#                    still needs Phase 2 work — distrobox containers default to
#                    --network host, so two of them both binding Zenoh on 7447
#                    will collide. Until each worker container is set up with
#                    its own Zenoh router port (separate plan), NUM_WORKERS>1
#                    will not actually run in parallel.
#   HOST_CPUS        cores available to split   (default: 30)
#   HOST_MEM_GB      GB of RAM available to split (default: 192)
#   WORKER_CPUS      per-worker --cpus override (default: ⌊HOST_CPUS/N⌋)
#   WORKER_MEM       per-worker --memory override (default: ⌊HOST_MEM_GB/N⌋g)
#   Plus every env var run_parallel.sh accepts (POLICY, GROUND_TRUTH, IMAGE,
#   AIC_IMAGE_SCALE, etc.) — forwarded to each worker unchanged.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

NUM_WORKERS="${NUM_WORKERS:-4}"
INPUT_CONFIG="${INPUT_CONFIG:-$REPO_ROOT/aic_engine/config/train.yaml}"
SPLIT_DIR="${SPLIT_DIR:-/home/ubuntu/aic-data/aic_split_configs}"
RESULTS_BASE="${RESULTS_BASE:-/home/ubuntu/aic-data}"
MERGED_OUTPUT="${MERGED_OUTPUT:-$RESULTS_BASE/merged_score.yaml}"
STAGGER_SECS="${STAGGER_SECS:-5}"
HOST_CPUS="${HOST_CPUS:-30}"
HOST_MEM_GB="${HOST_MEM_GB:-192}"

if ! [[ "$NUM_WORKERS" =~ ^[0-9]+$ ]] || [ "$NUM_WORKERS" -lt 1 ]; then
    echo "error: NUM_WORKERS must be a positive integer (got '$NUM_WORKERS')" >&2
    exit 1
fi

# Per-worker budgets (integer division — floor). Callers can override with
# explicit WORKER_CPUS / WORKER_MEM env vars; otherwise we derive from
# HOST_CPUS / HOST_MEM_GB divided by N.
WORKER_CPUS_DEFAULT=$((HOST_CPUS / NUM_WORKERS))
WORKER_MEM_DEFAULT="$((HOST_MEM_GB / NUM_WORKERS))g"
export WORKER_CPUS="${WORKER_CPUS:-$WORKER_CPUS_DEFAULT}"
export WORKER_MEM="${WORKER_MEM:-$WORKER_MEM_DEFAULT}"

if [ ! -f "$INPUT_CONFIG" ]; then
    echo "error: INPUT_CONFIG not found at $INPUT_CONFIG" >&2
    exit 1
fi

mkdir -p "$RESULTS_BASE" "$SPLIT_DIR"

echo "=== Multi-worker AIC eval ==="
echo "Num workers:   $NUM_WORKERS"
echo "Input config:  $INPUT_CONFIG"
echo "Split dir:     $SPLIT_DIR"
echo "Results base:  $RESULTS_BASE (per-worker: aic_results_<i>/)"
echo "Per-worker:    --cpus=$WORKER_CPUS --memory=$WORKER_MEM"
echo "Stagger:       ${STAGGER_SECS}s between launches"
echo ""

# --- Stage 1: split train.yaml into N parts -------------------------------

echo "[orchestrator] Splitting trials..."
pixi run python "$SCRIPT_DIR/split_trials.py" \
    --input "$INPUT_CONFIG" \
    --output-dir "$SPLIT_DIR" \
    --num-parts "$NUM_WORKERS"

# Verify every part file landed where we expect.
for i in $(seq 0 $((NUM_WORKERS - 1))); do
    pf="$SPLIT_DIR/train_part_${i}.yaml"
    if [ ! -f "$pf" ]; then
        echo "error: split_trials.py did not produce $pf" >&2
        exit 1
    fi
done

# --- Stage 2: fan out N workers -------------------------------------------

PIDS=()
NAMES=()

cleanup() {
    echo ""
    echo "[orchestrator] Cleaning up..."
    # Signal each backgrounded run_parallel.sh — its own EXIT trap kills the
    # in-container launch and the host-side policy/relay. Distrobox containers
    # themselves are long-lived (created by scripts/setup_workers.sh) and must
    # NOT be removed here.
    for pid in "${PIDS[@]}"; do
        kill "$pid" 2>/dev/null || true
    done
    # Belt-and-suspenders: directly kill any lingering /entrypoint.sh inside
    # each worker container, in case a child shell died before its trap fired.
    for name in "${NAMES[@]}"; do
        sudo docker exec "$name" pkill -TERM -f /entrypoint.sh >/dev/null 2>&1 || true
    done
}
trap cleanup EXIT INT TERM

for i in $(seq 0 $((NUM_WORKERS - 1))); do
    part_file="$SPLIT_DIR/train_part_${i}.yaml"
    worker_results="$RESULTS_BASE/aic_results_${i}"
    mkdir -p "$worker_results"
    runner_log="$worker_results/runner.log"

    # Stagger to avoid all N workers hitting Gazebo init / GPU at once.
    if [ "$i" -gt 0 ]; then
        sleep "$STAGGER_SECS"
    fi

    echo "[orchestrator] Launching worker $i (part $part_file, log $runner_log)"
    (
        PART_FILE="$part_file" \
        RESULTS="$RESULTS_BASE" \
        bash "$SCRIPT_DIR/run_parallel.sh"
    ) > "$runner_log" 2>&1 &
    PIDS+=("$!")
    NAMES+=("aic_worker_${i}")
done

echo ""
echo "[orchestrator] All $NUM_WORKERS workers launched. Waiting..."
echo "             Tail any worker with: tail -f $RESULTS_BASE/aic_results_<i>/runner.log"
echo ""

# --- Stage 3: wait for everyone -------------------------------------------

failed=0
for idx in "${!PIDS[@]}"; do
    pid="${PIDS[$idx]}"
    if wait "$pid"; then
        echo "[orchestrator] worker $idx done (PID $pid)"
    else
        rc=$?
        echo "[orchestrator] worker $idx FAILED (PID $pid, exit $rc) — see $RESULTS_BASE/aic_results_${idx}/runner.log" >&2
        failed=$((failed + 1))
    fi
done

# Clear the trap so we don't double-kill on normal exit. Cleanup of any
# remaining containers is already handled inside each child's own trap.
trap - EXIT INT TERM

# --- Stage 4: merge scores ------------------------------------------------

echo ""
if [ "$failed" -gt 0 ]; then
    echo "[orchestrator] $failed worker(s) failed — merging surviving score.yaml files anyway." >&2
fi

echo "[orchestrator] Merging per-worker scores → $MERGED_OUTPUT"
pixi run python "$SCRIPT_DIR/merge_scores.py" \
    --results-root "$RESULTS_BASE" \
    --output "$MERGED_OUTPUT"

echo ""
echo "=== Multi-worker run finished ==="
if [ "$failed" -gt 0 ]; then
    echo "  ($failed of $NUM_WORKERS worker(s) failed — merged_score.yaml may be incomplete)"
    exit 1
fi
