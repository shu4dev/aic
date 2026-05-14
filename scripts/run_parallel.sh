#!/bin/bash
# Single-worker AIC eval launcher that mirrors the working 3-terminal Lambda
# manual flow:
#   T1: aic_eval container — start_aic_engine:=true gazebo_gui:=false
#       launch_rviz:=false (with shutdown_on_aic_engine_exit:=true added so
#       this script can wait + clean up automatically).
#   T2: host policy via `pixi run ros2 run aic_model …`.
#   T3: host image_relay via system ROS (`source /opt/ros/kilted/setup.bash`).
#
# Designed for the Lambda 1× A100 40 GB / 30 vCPU / 200 GB RAM instance:
# the container gets --cpus 30 --memory 192g (≈8 GB host headroom) and uses
# --network host so the host-side policy + relay can reach the container's
# Zenoh router on 127.0.0.1:7447. This locks the script to one worker per
# host — port 7447 is a single fixed slot under host networking.
#
# Usage:
#   bash scripts/run_parallel.sh
#
#   # With overrides:
#   POLICY=aic_example_policies.ros.WaveArm GROUND_TRUTH=false \
#       bash scripts/run_parallel.sh
#
# Environment overrides (all optional):
#   IMAGE            eval Docker image    (default: ghcr.io/shu4dev/aic-eval:v1)
#   POLICY           policy module path   (default: aic_example_policies.ros.SCCheatCode)
#   GROUND_TRUTH     ground_truth arg     (default: true)
#   RESULTS          host results dir     (default: $HOME/aic_results)
#   RECORDING_DIR    optional MCAP root   (mounted at /root/aic_recordings)
#   AIC_IMAGE_SCALE  passed to container env (default: unset)
#   NAME             container name       (default: aic_worker)
#   WORKER_CPUS      --cpus per container (default: 30)
#   WORKER_MEM       --memory per container (default: 192g)
#   RELAY_NODE       host path to image_relay_node.py
#                                         (default: alongside this script)
#   ROS_SETUP        system ROS setup     (default: /opt/ros/kilted/setup.bash)
#
# Notes:
#   * The default IMAGE is a private GHCR repo. Log in once:
#       echo "$GHCR_TOKEN" | sudo docker login ghcr.io -u shu4dev --password-stdin
#     (see Step 4 of docs/lambda_headless_setup.md).
#   * --gpus all + NVIDIA_DRIVER_CAPABILITIES=all is required for Ogre2 / EGL.
#   * `pixi_env_setup.sh` already exports RMW_IMPLEMENTATION=rmw_zenoh_cpp and
#     ZENOH_CONFIG_OVERRIDE=transport/shared_memory/enabled=false, so `pixi run`
#     gets those for free (matches T2 in the manual flow — no explicit exports
#     needed there). The host image_relay does need explicit RMW/Zenoh exports
#     because it sources /opt/ros/kilted, not pixi — matches T3.
#   * Tail logs at $RESULTS/{policy.log,image_relay.log}, or
#     `sudo docker logs -f $NAME` for the eval engine.
#   * Ctrl-C is safe: the EXIT trap kills the host policy/relay and force-rms
#     the container.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

IMAGE="${IMAGE:-ghcr.io/shu4dev/aic-eval:v1}"
POLICY="${POLICY:-aic_example_policies.ros.SCCheatCode}"
GROUND_TRUTH="${GROUND_TRUTH:-true}"
RESULTS="${RESULTS:-$HOME/aic_results}"
NAME="${NAME:-aic_worker}"
WORKER_CPUS="${WORKER_CPUS:-30}"
WORKER_MEM="${WORKER_MEM:-192g}"
RELAY_NODE="${RELAY_NODE:-$SCRIPT_DIR/image_relay_node.py}"
ROS_SETUP="${ROS_SETUP:-/opt/ros/kilted/setup.bash}"

DOCKER="sudo docker"

# --- Preflight -------------------------------------------------------------

if ! ${DOCKER} info >/dev/null 2>&1; then
    echo "error: cannot run '${DOCKER} info' — is Docker running and do you have permission?" >&2
    exit 1
fi

if [ ! -f "$RELAY_NODE" ]; then
    echo "error: image_relay_node not found at $RELAY_NODE — set RELAY_NODE" >&2
    exit 1
fi

if [ ! -f "$ROS_SETUP" ]; then
    echo "error: ROS setup not found at $ROS_SETUP — set ROS_SETUP" >&2
    exit 1
fi

if ! command -v pixi >/dev/null 2>&1; then
    echo "error: 'pixi' not found on PATH — needed for the host policy launch (T2)" >&2
    exit 1
fi

# Port 7447 must be free on the host, otherwise --network host launches a
# container whose Zenoh router silently fails to bind and the wait loop hangs.
if ss -ltn 2>/dev/null | awk '{print $4}' | grep -qE '[:.]7447$'; then
    echo "error: something is already listening on 127.0.0.1:7447 — free it first." >&2
    echo "       (Likely a leftover run; try: ${DOCKER} rm -f $NAME)" >&2
    exit 1
fi

# Verify the image is locally present or pullable. Catches "not logged in to
# ghcr.io" up front with a clear message instead of letting `docker run` fail
# halfway through with "pull access denied".
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

mkdir -p "$RESULTS"

echo "=== Single-worker AIC eval ==="
echo "Image:        $IMAGE"
echo "Policy:       $POLICY (host pixi)"
echo "ground_truth: $GROUND_TRUTH"
echo "Results:      $RESULTS"
echo "Container:    $NAME (--network host, --cpus=$WORKER_CPUS --memory=$WORKER_MEM)"
echo "Relay:        $RELAY_NODE (host, ROS @ $ROS_SETUP)"
echo ""

# --- Stage 1: launch eval container (mirrors T1) --------------------------

${DOCKER} rm -f "$NAME" >/dev/null 2>&1 || true

recording_args=()
if [ -n "${RECORDING_DIR:-}" ]; then
    mkdir -p "$RECORDING_DIR"
    recording_args=(-v "$RECORDING_DIR:/root/aic_recordings" -e "AIC_RECORDING_DIR=/root/aic_recordings")
fi

image_scale_args=()
if [ -n "${AIC_IMAGE_SCALE:-}" ]; then
    image_scale_args=(-e "AIC_IMAGE_SCALE=$AIC_IMAGE_SCALE")
fi

${DOCKER} run -d --rm \
    --name "$NAME" \
    --network host \
    --gpus all \
    --cpus "$WORKER_CPUS" \
    --memory "$WORKER_MEM" \
    --env NVIDIA_DRIVER_CAPABILITIES=all \
    --env NVIDIA_VISIBLE_DEVICES=all \
    --env __EGL_VENDOR_LIBRARY_FILENAMES=/usr/share/glvnd/egl_vendor.d/10_nvidia.json \
    --user root \
    -v "$RESULTS:/root/aic_results" \
    -e AIC_RESULTS_DIR=/root/aic_results \
    "${recording_args[@]}" \
    "${image_scale_args[@]}" \
    "$IMAGE" \
    ground_truth:="$GROUND_TRUTH" \
    gazebo_gui:=false launch_rviz:=false \
    start_aic_engine:=true \
    shutdown_on_aic_engine_exit:=true \
    >/dev/null

echo "[$NAME] started"

# --- Cleanup trap ---------------------------------------------------------

RELAY_PID=""
POLICY_PID=""

cleanup() {
    if [ -n "$POLICY_PID" ]; then kill "$POLICY_PID" 2>/dev/null || true; fi
    if [ -n "$RELAY_PID" ];  then kill "$RELAY_PID"  2>/dev/null || true; fi
    ${DOCKER} rm -f "$NAME" >/dev/null 2>&1 || true
}
trap cleanup EXIT INT TERM

# --- Stage 2: wait for Zenoh on host 127.0.0.1:7447 -----------------------

echo -n "[$NAME] waiting for Zenoh "
zenoh_up=0
for _ in $(seq 1 90); do
    if (echo > /dev/tcp/127.0.0.1/7447) 2>/dev/null; then
        echo "✓"
        zenoh_up=1
        break
    fi
    echo -n "."
    sleep 2
done

if [ "$zenoh_up" = "0" ]; then
    echo ""
    echo "error: Zenoh router on 127.0.0.1:7447 never came up — check '${DOCKER} logs $NAME'" >&2
    exit 1
fi

# --- Stage 3a: launch image_relay on host (mirrors T3) --------------------

RELAY_LOG="$RESULTS/image_relay.log"
(
    # shellcheck disable=SC1090
    source "$ROS_SETUP"
    export RMW_IMPLEMENTATION=rmw_zenoh_cpp
    export ZENOH_CONFIG_OVERRIDE="transport/shared_memory/enabled=false"
    exec python3 "$RELAY_NODE"
) >> "$RELAY_LOG" 2>&1 &
RELAY_PID=$!
echo "[$NAME] image_relay launched (PID $RELAY_PID, log $RELAY_LOG)"

# --- Stage 3b: launch policy on host (mirrors T2) -------------------------

POLICY_LOG="$RESULTS/policy.log"
(
    cd "$REPO_ROOT"
    exec pixi run ros2 run aic_model aic_model --ros-args \
        -p use_sim_time:=true \
        -p policy:="$POLICY"
) >> "$POLICY_LOG" 2>&1 &
POLICY_PID=$!
echo "[$NAME] policy launched: $POLICY (PID $POLICY_PID, log $POLICY_LOG)"

# --- Stage 4: wait for the eval container to self-terminate ---------------

echo ""
echo "Waiting for $NAME to finish..."
${DOCKER} wait "$NAME" >/dev/null
echo "[$NAME] done"

# --- Summary --------------------------------------------------------------

echo ""
echo "=== Run finished ==="
if [ -f "$RESULTS/score.yaml" ]; then
    ls -lh "$RESULTS/score.yaml"
else
    echo "(no score.yaml in $RESULTS — check $POLICY_LOG, $RELAY_LOG, and '${DOCKER} logs $NAME')"
fi
