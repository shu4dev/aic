#!/bin/bash
# Run N aic_eval containers in parallel, each evaluating a chunk of trials
# produced by scripts/split_trials.py.
#
# Designed for a Lambda Cloud GPU instance prepared per
# docs/lambda_headless_setup.md — i.e., the host has matching
# libnvidia-gl-*-server packages installed and NVIDIA Container Toolkit
# is configured.
#
# Default sizing targets a 1× A100 40 GB / 30 vCPU / 200 GB RAM Lambda
# instance: 6 workers × (5 CPUs, 32 GB RAM) = 30 CPU / 192 GB,
# leaving ~8 GB host headroom. A100 40 GB VRAM is non-binding (~1-2 GB
# per headless gz-sim worker). Override N on the CLI and per-worker
# limits via WORKER_CPUS / WORKER_MEM (see below).
#
# Each worker:
#   * runs as its own Docker container with its own network namespace
#     (default bridge), so port 7447 inside one container does not collide
#     with port 7447 inside another;
#   * mounts a unique split config at /tmp/aic_config.yaml, a unique
#     host results dir at /root/aic_results, and the host-side
#     image_relay_node.py at /tmp/image_relay_node.py;
#   * launches image_relay_node (downscale + JPEG re-encode of camera
#     topics → ~160× smaller MCAP bags; see Step 8 of
#     docs/lambda_headless_setup.md) and the policy via `docker exec`
#     after the eval's Zenoh router comes up, so eval + relay + policy
#     all share the container's loopback.
#
# Workers self-terminate after their last trial because we pass
# `shutdown_on_aic_engine_exit:=true` to /entrypoint.sh.
#
# Usage:
#   # 1. Split the trial config (match --num-parts to N below):
#   pixi run python scripts/split_trials.py \
#       --input aic_engine/config/train.yaml \
#       --output-dir ~/aic_split_configs \
#       --num-parts 6
#
#   # 2. Run the workers (defaults to 6 on the A100 40 GB instance):
#   bash scripts/run_parallel.sh 6
#
#   # 3. Merge the per-worker score files:
#   pixi run python scripts/merge_scores.py --results-root ~
#
# Environment overrides (all optional):
#   IMAGE          eval Docker image       (default: ghcr.io/shu4dev/aic-eval:v1)
#   CONFIGS_DIR    where the splits live   (default: $HOME/aic_split_configs)
#   POLICY         policy module path      (default: aic_example_policies.ros.SCCheatCode)
#   RESULTS_BASE   per-worker results root (default: $HOME/aic_results_<i>)
#   RECORDING_DIR  optional MCAP root      (if set, mounts to /root/aic_recordings)
#   AIC_IMAGE_SCALE  passed into the policy environment (default: unset)
#   GROUND_TRUTH   ground_truth launch arg (default: true — required for recording)
#   WORKER_CPUS    --cpus per container    (default: 5      → 6×5 = 30 vCPU)
#   WORKER_MEM     --memory per container  (default: 32g    → 6×32 = 192 GB)
#   IMAGE_RELAY    launch image_relay_node inside each worker
#                                          (default: 1; set to 0 to skip)
#   RELAY_NODE     host path to image_relay_node.py
#                                          (default: alongside this script)
#
# Notes:
#   * The default IMAGE is a private GHCR repo. Log in once before running:
#       echo "$GHCR_TOKEN" | sudo docker login ghcr.io -u shu4dev --password-stdin
#     (matches Step 4 of docs/lambda_headless_setup.md). Otherwise the first
#     `docker run` fails with "pull access denied" / "manifest unknown".
#   * `--gpus all` + NVIDIA_DRIVER_CAPABILITIES=all is required for Ogre2 to
#     find the NVIDIA EGL ICD. See docs/lambda_headless_setup.md if you hit
#     "OpenGL 3.3 is not supported" or "Found Num EGL Devices: 0".
#   * image_relay runs inside each container (not on the host as in Step 8 of
#     the doc) because each worker has its own Zenoh router on its own
#     container-local loopback — host-side relay can only see one worker.
#     Container-side launch shares the eval router's UTS/IPC namespace, so
#     Zenoh shared-memory works and ZENOH_CONFIG_OVERRIDE is not needed.
#   * Use `sudo docker logs -f aic_worker_<i>` to tail any worker live.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

N="${1:-6}"
IMAGE="${IMAGE:-ghcr.io/shu4dev/aic-eval:v1}"
CONFIGS_DIR="${CONFIGS_DIR:-$HOME/aic_split_configs}"
POLICY="${POLICY:-aic_example_policies.ros.SCCheatCode}"
GROUND_TRUTH="${GROUND_TRUTH:-true}"
PREFIX="${PREFIX:-train_part}"
WORKER_CPUS="${WORKER_CPUS:-5}"
WORKER_MEM="${WORKER_MEM:-32g}"
IMAGE_RELAY="${IMAGE_RELAY:-1}"
RELAY_NODE="${RELAY_NODE:-$SCRIPT_DIR/image_relay_node.py}"

if [ "$IMAGE_RELAY" = "1" ] && [ ! -f "$RELAY_NODE" ]; then
    echo "error: image_relay_node not found at $RELAY_NODE — set RELAY_NODE or IMAGE_RELAY=0" >&2
    exit 1
fi

DOCKER="sudo docker"
if ${DOCKER} info >/dev/null 2>&1; then :; else
    echo "error: cannot run '${DOCKER} info' — is Docker running and do you have permission?" >&2
    exit 1
fi

if [ ! -d "$CONFIGS_DIR" ]; then
    echo "error: $CONFIGS_DIR does not exist — run split_trials.py first" >&2
    exit 1
fi

# Verify the image is locally present or pullable. Catches "not logged in
# to ghcr.io" up front with a clear message instead of letting the first
# `docker run` in Stage 1 fail halfway through with "pull access denied".
if ! ${DOCKER} image inspect "$IMAGE" >/dev/null 2>&1; then
    echo "Pulling $IMAGE ..."
    if ! ${DOCKER} pull "$IMAGE"; then
        echo "" >&2
        echo "error: cannot pull $IMAGE." >&2
        echo "       If this is a private GHCR repo, log in first:" >&2
        echo "       echo \"\$GHCR_TOKEN\" | ${DOCKER} login ghcr.io -u shu4dev --password-stdin" >&2
        echo "       (see Step 4 of docs/lambda_headless_setup.md)" >&2
        exit 1
    fi
fi

echo "=== Parallel AIC eval ==="
echo "Workers:     $N"
echo "Image:       $IMAGE"
echo "Splits dir:  $CONFIGS_DIR"
echo "Policy:      $POLICY"
echo "ground_truth: $GROUND_TRUTH"
echo "Per-worker:  --cpus=$WORKER_CPUS --memory=$WORKER_MEM"
echo "Image relay: $([ "$IMAGE_RELAY" = "1" ] && echo "on ($RELAY_NODE)" || echo "off")"
echo ""

# --- Stage 1: start N detached eval containers ---
worker_names=()
for i in $(seq 0 $((N-1))); do
    NAME="aic_worker_$i"
    RESULTS="${RESULTS_BASE:-$HOME/aic_results}_$i"
    CONFIG_HOST="$CONFIGS_DIR/${PREFIX}_$i.yaml"

    if [ ! -f "$CONFIG_HOST" ]; then
        echo "error: split config missing: $CONFIG_HOST" >&2
        exit 1
    fi

    mkdir -p "$RESULTS"
    ${DOCKER} rm -f "$NAME" >/dev/null 2>&1 || true

    recording_args=()
    if [ -n "${RECORDING_DIR:-}" ]; then
        REC_HOST="${RECORDING_DIR}_$i"
        mkdir -p "$REC_HOST"
        recording_args=(-v "$REC_HOST:/root/aic_recordings" -e "AIC_RECORDING_DIR=/root/aic_recordings")
    fi

    image_scale_args=()
    if [ -n "${AIC_IMAGE_SCALE:-}" ]; then
        image_scale_args=(-e "AIC_IMAGE_SCALE=$AIC_IMAGE_SCALE")
    fi

    relay_mount_args=()
    if [ "$IMAGE_RELAY" = "1" ]; then
        relay_mount_args=(-v "$RELAY_NODE:/tmp/image_relay_node.py:ro")
    fi

    ${DOCKER} run -d --rm \
        --name "$NAME" \
        --gpus all \
        --cpus "$WORKER_CPUS" \
        --memory "$WORKER_MEM" \
        --env NVIDIA_DRIVER_CAPABILITIES=all \
        --env NVIDIA_VISIBLE_DEVICES=all \
        --env __EGL_VENDOR_LIBRARY_FILENAMES=/usr/share/glvnd/egl_vendor.d/10_nvidia.json \
        --user root \
        -v "$RESULTS:/root/aic_results" \
        -v "$CONFIG_HOST:/tmp/aic_config.yaml:ro" \
        -e AIC_RESULTS_DIR=/root/aic_results \
        "${recording_args[@]}" \
        "${image_scale_args[@]}" \
        "${relay_mount_args[@]}" \
        "$IMAGE" \
        aic_engine_config_file:=/tmp/aic_config.yaml \
        ground_truth:="$GROUND_TRUTH" \
        gazebo_gui:=false launch_rviz:=false \
        start_aic_engine:=true \
        shutdown_on_aic_engine_exit:=true \
        >/dev/null

    echo "[$NAME] started — config=$(basename $CONFIG_HOST) results=$RESULTS"
    worker_names+=("$NAME")
done

# --- Stage 2: wait for each worker's Zenoh router (port 7447, inside the container) ---
for NAME in "${worker_names[@]}"; do
    echo -n "[$NAME] waiting for Zenoh "
    for _ in $(seq 1 90); do
        if ${DOCKER} exec "$NAME" bash -c '
            command -v nc >/dev/null && nc -z 127.0.0.1 7447 ||
            (echo > /dev/tcp/127.0.0.1/7447) 2>/dev/null
        ' 2>/dev/null; then
            echo "✓"
            break
        fi
        echo -n "."
        sleep 2
    done
done

# --- Stage 3a: launch image_relay_node inside each container ---
# Must come up before the policy so /<cam>/image_small is being
# published by the time the eval engine starts recording (per Step 8 of
# docs/lambda_headless_setup.md). Container-local: each worker has its
# own Zenoh router on its own loopback, so one relay per container.
if [ "$IMAGE_RELAY" = "1" ]; then
    for NAME in "${worker_names[@]}"; do
        ${DOCKER} exec -d "$NAME" bash -c "
            source /ws_aic/install/setup.bash
            export RMW_IMPLEMENTATION=rmw_zenoh_cpp
            unset DISPLAY
            python3 /tmp/image_relay_node.py \
                >> /root/aic_results/image_relay.log 2>&1
        "
        echo "[$NAME] image_relay_node launched"
    done
fi

# --- Stage 3b: launch the policy inside each container ---
for NAME in "${worker_names[@]}"; do
    ${DOCKER} exec -d "$NAME" bash -c "
        source /ws_aic/install/setup.bash
        export RMW_IMPLEMENTATION=rmw_zenoh_cpp
        unset DISPLAY
        ros2 run aic_model aic_model --ros-args \
            -p use_sim_time:=true \
            -p policy:=$POLICY \
            >> /root/aic_results/policy.log 2>&1
    "
    echo "[$NAME] policy ($POLICY) launched"
done

# --- Stage 4: wait for every container to self-terminate ---
echo ""
echo "All workers running. Waiting for them to finish..."
for NAME in "${worker_names[@]}"; do
    ${DOCKER} wait "$NAME" >/dev/null
    echo "[$NAME] done"
done

# --- Summary ---
echo ""
echo "=== All workers finished ==="
ls -lh "${RESULTS_BASE:-$HOME/aic_results}"_*/score.yaml 2>/dev/null || \
    echo "(no score.yaml files found — check per-worker logs)"
echo ""
echo "Next: pixi run python scripts/merge_scores.py --results-root \$HOME"
