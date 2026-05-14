#!/bin/bash
# AIC eval worker launcher — runs one trial-set end-to-end by mirroring the
# working 3-terminal Lambda manual flow. By default a single worker takes the
# whole box; with PART_INDEX≥1 / ROUTER_PORT≠7447 it shares the box with other
# concurrent workers (see scripts/run_parallel_multi.sh).
#
# The 3 stages map to the original manual terminals:
#   T1: aic_eval container — start_aic_engine:=true gazebo_gui:=false
#       launch_rviz:=false (with shutdown_on_aic_engine_exit:=true added so
#       this script can wait + clean up automatically).
#   T2: host policy via `pixi run ros2 run aic_model …`.
#   T3: host image_relay via system ROS (`source /opt/ros/kilted/setup.bash`).
#
# Designed for the Lambda 1× A100 40 GB / 30 vCPU / 200 GB RAM instance.
# Networking:
#   * Worker 0 (the default) uses --network host so the host-side policy +
#     relay reach the container's Zenoh router on 127.0.0.1:7447 — same as
#     the original single-worker flow.
#   * Workers ≥1 (PART_INDEX ≥ 1, or ROUTER_PORT set explicitly to something
#     other than 7447) use bridge networking with --publish 127.0.0.1:<port>:7447
#     so multiple containers can coexist on one Lambda box. The host-side
#     sessions are pointed at the per-worker port via ZENOH_CONFIG_OVERRIDE.
#     Use scripts/run_parallel_multi.sh to orchestrate N concurrent workers.
#
# Usage:
#   bash scripts/run_parallel.sh
#
#   # With overrides:
#   POLICY=aic_example_policies.ros.WaveArm GROUND_TRUTH=false \
#       bash scripts/run_parallel.sh
#
#   # Trial-split parallel workflow (one host per part):
#   #   1. On any host, split train.yaml into N self-contained part configs:
#   #        pixi run python scripts/split_trials.py \
#   #            --input aic_engine/config/train.yaml \
#   #            --output-dir /home/ubuntu/aic-data/aic_split_configs \
#   #            --num-parts 4
#   #   2. On worker i (i in 0..N-1), run:
#   #        PART_FILE=/home/ubuntu/aic-data/aic_split_configs/train_part_${i}.yaml \
#   #            bash scripts/run_parallel.sh
#   #      → results land in /home/ubuntu/aic-data/aic_results_${i}/score.yaml
#   #   3. After all parts finish, merge the per-part scores:
#   #        pixi run python scripts/merge_scores.py \
#   #            --results-root /home/ubuntu/aic-data \
#   #            --output /home/ubuntu/aic-data/merged_score.yaml
#
# Environment overrides (all optional):
#   IMAGE            eval Docker image    (default: ghcr.io/shu4dev/aic-eval:v1)
#   POLICY           policy module path   (default: aic_example_policies.ros.SCCheatCode)
#   GROUND_TRUTH     ground_truth arg     (default: true)
#   PART_FILE        host path to a train_part_<i>.yaml produced by
#                    scripts/split_trials.py. When set, mounted into the
#                    container at /root/aic_part.yaml and passed via
#                    aic_engine_config_file:=... so the engine only runs that
#                    slice of trials. Default: unset (image's bundled config).
#   PART_INDEX       integer suffix appended to NAME ("aic_worker_<i>") and
#                    RESULTS ("<dir>/aic_results_<i>") so concurrent workers
#                    sharing mounted storage don't clobber each other. Auto-
#                    derived from PART_FILE's trailing _<int> when unset.
#   RESULTS          host results dir     (default: /home/ubuntu/aic-data — mounted FS;
#                                          suffixed with /aic_results_<i> when PART_INDEX is set)
#   RECORDING_DIR    optional MCAP root   (mounted at /root/aic_recordings)
#   AIC_IMAGE_SCALE  passed to container env (default: unset)
#   NAME             container name       (default: aic_worker; suffixed with _<i> when PART_INDEX is set)
#   ROUTER_PORT      Zenoh router host port (default: 7447, or 7447+PART_INDEX
#                                          when PART_INDEX is set). 7447 keeps
#                                          --network host; anything else uses
#                                          bridge + --publish 127.0.0.1:<port>:7447
#                                          so multiple workers can share one host.
#   WORKER_CPUS      --cpus per container (default: 30; for N-way parallel,
#                                          set to ⌊30/N⌋ so the box isn't oversubscribed)
#   WORKER_MEM       --memory per container (default: 192g; for N-way parallel,
#                                          set to ⌊192/N⌋g for the same reason)
#   RELAY_NODE       host path to image_relay_node.py
#                                         (default: alongside this script)
#   RELAY_VENV       Python venv that image_relay_node.py needs activated
#                    (for cv2/numpy). Sourced after ROS_SETUP so the venv's
#                    python3 wins in PATH.
#                                         (default: $HOME/ws_aic/src/aic/.venv)
#   ROS_SETUP        system ROS setup     (default: /opt/ros/kilted/setup.bash)
#
# Notes:
#   * The default IMAGE is a private GHCR repo. Log in once:
#       echo "$GHCR_TOKEN" | sudo docker login ghcr.io -u shu4dev --password-stdin
#     (see Step 4 of docs/lambda_headless_setup.md).
#   * --gpus all + NVIDIA_DRIVER_CAPABILITIES=all is required for Ogre2 / EGL.
#   * `pixi_env_setup.sh` exports RMW_IMPLEMENTATION=rmw_zenoh_cpp and uses
#     ZENOH_CONFIG_OVERRIDE="${ZENOH_CONFIG_OVERRIDE:-...}" — the `:-` form
#     preserves an already-exported value, which is why this script's per-worker
#     ZENOH_CONFIG_OVERRIDE (set before `pixi run`) survives into the policy
#     env. The host image_relay sources /opt/ros/kilted (not pixi), so it needs
#     the same RMW + Zenoh exports done explicitly — matches T3.
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
PART_FILE="${PART_FILE:-}"
PART_INDEX="${PART_INDEX:-}"
ROUTER_PORT="${ROUTER_PORT:-}"
WORKER_CPUS="${WORKER_CPUS:-30}"
WORKER_MEM="${WORKER_MEM:-192g}"
RELAY_NODE="${RELAY_NODE:-$SCRIPT_DIR/image_relay_node.py}"
RELAY_VENV="${RELAY_VENV:-$HOME/ws_aic/src/aic/.venv}"
ROS_SETUP="${ROS_SETUP:-/opt/ros/kilted/setup.bash}"

# Auto-derive PART_INDEX from PART_FILE basename (e.g. train_part_3.yaml → 3)
# so callers only need to set PART_FILE.
if [ -n "$PART_FILE" ] && [ -z "$PART_INDEX" ]; then
    _part_base="$(basename "$PART_FILE" .yaml)"
    PART_INDEX="${_part_base##*_}"
fi

# Auto-derive ROUTER_PORT from PART_INDEX so concurrent workers on the same host
# get distinct Zenoh listen ports: worker 0 → 7447, worker 1 → 7448, …
# Worker 0 stays on --network host (current behavior); workers ≥1 switch to
# bridge networking with --publish 127.0.0.1:<port>:7447 because the image's
# entrypoint (docker/aic_eval/zenoh_config_router.sh) hardcodes the in-container
# listen endpoint to 7447 and can't be overridden via -e at run time.
if [ -z "$ROUTER_PORT" ]; then
    if [ -n "$PART_INDEX" ]; then
        ROUTER_PORT=$((7447 + PART_INDEX))
    else
        ROUTER_PORT=7447
    fi
fi

# Suffix NAME / RESULTS when running a trial split so concurrent workers
# sharing the mounted FS land in distinct dirs and the merge_scores.py glob
# (aic_results_*/score.yaml) picks each one up.
_NAME_BASE="${NAME:-aic_worker}"
_RESULTS_BASE="${RESULTS:-/home/ubuntu/aic-data}"
if [ -n "$PART_INDEX" ]; then
    NAME="${_NAME_BASE}_${PART_INDEX}"
    RESULTS="${_RESULTS_BASE}/aic_results_${PART_INDEX}"
else
    NAME="$_NAME_BASE"
    RESULTS="$_RESULTS_BASE"
fi

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

# image_relay_node imports cv2/numpy from a venv at ws_aic/src/aic/.venv; the
# activate script must exist or the relay subshell will run with the system
# python and crash on import.
if [ ! -f "$RELAY_VENV/bin/activate" ]; then
    echo "error: relay venv activate script not found at $RELAY_VENV/bin/activate — set RELAY_VENV" >&2
    exit 1
fi

if [ -n "$PART_FILE" ] && [ ! -f "$PART_FILE" ]; then
    echo "error: PART_FILE not found at $PART_FILE" >&2
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

# The chosen ROUTER_PORT must be free on the host. For worker 0 (port 7447 with
# --network host) the container binds directly; for workers ≥1 the bridge port
# mapping needs the host slot free. Either way, a port in use means a stale
# leftover or an overlapping worker.
if ss -ltn 2>/dev/null | awk '{print $4}' | grep -qE "[:.]${ROUTER_PORT}\$"; then
    echo "error: something is already listening on 127.0.0.1:${ROUTER_PORT} — free it first." >&2
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

echo "=== AIC eval worker ==="
echo "Image:        $IMAGE"
echo "Policy:       $POLICY (host pixi)"
echo "ground_truth: $GROUND_TRUTH"
if [ -n "$PART_FILE" ]; then
    echo "Part file:    $PART_FILE (PART_INDEX=$PART_INDEX)"
fi
echo "Results:      $RESULTS"
if [ "$ROUTER_PORT" = "7447" ]; then
    echo "Container:    $NAME (--network host, Zenoh @ 127.0.0.1:7447, --cpus=$WORKER_CPUS --memory=$WORKER_MEM)"
else
    echo "Container:    $NAME (bridge -p 127.0.0.1:${ROUTER_PORT}:7447, --cpus=$WORKER_CPUS --memory=$WORKER_MEM)"
fi
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

part_mount_args=()
part_launch_args=()
if [ -n "$PART_FILE" ]; then
    part_mount_args=(-v "$PART_FILE:/root/aic_part.yaml:ro")
    part_launch_args=(aic_engine_config_file:=/root/aic_part.yaml)
fi

# Worker 0 keeps --network host so single-worker invocations behave exactly as
# before. Workers ≥1 use bridge networking with --publish to expose the
# container's internal Zenoh router (still hardcoded to 7447 inside) on a
# distinct host port, so multiple workers can coexist on one Lambda box.
network_args=()
if [ "$ROUTER_PORT" = "7447" ]; then
    network_args=(--network host)
else
    network_args=(--publish "127.0.0.1:${ROUTER_PORT}:7447")
fi

${DOCKER} run -d --rm \
    --name "$NAME" \
    "${network_args[@]}" \
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
    "${part_mount_args[@]}" \
    "$IMAGE" \
    "${part_launch_args[@]}" \
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

# --- Stage 2: wait for Zenoh on host 127.0.0.1:${ROUTER_PORT} -------------

echo -n "[$NAME] waiting for Zenoh @ 127.0.0.1:${ROUTER_PORT} "
zenoh_up=0
for _ in $(seq 1 90); do
    if (echo > "/dev/tcp/127.0.0.1/${ROUTER_PORT}") 2>/dev/null; then
        echo "✓"
        zenoh_up=1
        break
    fi
    echo -n "."
    sleep 2
done

if [ "$zenoh_up" = "0" ]; then
    echo ""
    echo "error: Zenoh router on 127.0.0.1:${ROUTER_PORT} never came up — check '${DOCKER} logs $NAME'" >&2
    exit 1
fi

# Host-side ZENOH_CONFIG_OVERRIDE pointing the session at this worker's router.
# Worker 0 uses the default 7447 + multicast off (existing behavior); workers
# ≥1 need an explicit connect endpoint because they're not on the default port
# the base aic_zenoh_config.json5 specifies (tcp/localhost:7447). Multicast is
# already off in that base config but we re-assert it so a stray gossip packet
# can't pull this session into another worker's fabric.
if [ "$ROUTER_PORT" = "7447" ]; then
    HOST_ZENOH_OVERRIDE="transport/shared_memory/enabled=false"
else
    HOST_ZENOH_OVERRIDE='connect/endpoints=["tcp/localhost:'"${ROUTER_PORT}"'"];scouting/multicast/enabled=false;transport/shared_memory/enabled=false'
fi

# --- Stage 3a: launch image_relay on host (mirrors T3) --------------------

RELAY_LOG="$RESULTS/image_relay.log"
(
    # shellcheck disable=SC1090
    source "$ROS_SETUP"
    # Activate the ws_aic venv AFTER ROS setup so $RELAY_VENV/bin/python3 wins
    # in PATH while PYTHONPATH still contains the ROS python dirs (rclpy lives
    # there, cv2/numpy live in the venv).
    # shellcheck disable=SC1091
    source "$RELAY_VENV/bin/activate"
    export RMW_IMPLEMENTATION=rmw_zenoh_cpp
    export ZENOH_CONFIG_OVERRIDE="$HOST_ZENOH_OVERRIDE"
    exec python3 "$RELAY_NODE"
) >> "$RELAY_LOG" 2>&1 &
RELAY_PID=$!
echo "[$NAME] image_relay launched (PID $RELAY_PID, log $RELAY_LOG)"

# --- Stage 3b: launch policy on host (mirrors T2) -------------------------

POLICY_LOG="$RESULTS/policy.log"
(
    cd "$REPO_ROOT"
    # pixi_env_setup.sh uses `${ZENOH_CONFIG_OVERRIDE:-default}`, so exporting
    # here before `pixi run` makes our worker-specific endpoint stick.
    export ZENOH_CONFIG_OVERRIDE="$HOST_ZENOH_OVERRIDE"
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
