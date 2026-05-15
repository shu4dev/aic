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
#   * Every worker container is created by scripts/setup_workers.sh as a
#     long-lived distrobox container on --network host (distrobox default).
#   * scripts/setup_workers.sh bind-mounts a per-worker /entrypoint.sh that
#     pins the in-container Zenoh router to a distinct host port: worker i
#     listens on 7447+i. That's what lets N>1 coexist under --network host.
#   * Worker 0 (the default) uses port 7447 — identical wire behavior to the
#     original single-worker manual flow.
#   * Workers ≥1 (PART_INDEX ≥ 1, or ROUTER_PORT set explicitly) connect their
#     host-side policy + relay to localhost:$ROUTER_PORT via the per-worker
#     ZENOH_CONFIG_OVERRIDE this script sets just below.
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
#                                          suffixed with /aic_results_<i> when PART_INDEX is set).
#                                          aic_engine writes both score.yaml AND
#                                          per-trial bags (bag_trial_<id>_<ts>/)
#                                          into this dir via AIC_RESULTS_DIR.
#   AIC_IMAGE_SCALE  passed to container env (default: unset)
#   NAME             container name       (default: aic_worker; suffixed with _<i> when PART_INDEX is set)
#   ROUTER_PORT      Zenoh router host port (default: 7447, or 7447+PART_INDEX
#                                          when PART_INDEX is set). Must match
#                                          the port baked into the worker's
#                                          per-container /entrypoint.sh by
#                                          scripts/setup_workers.sh.
#   WORKER_CPUS      --cpus per container (default: 30; for N-way parallel,
#                                          set to ⌊30/N⌋ so the box isn't oversubscribed)
#   WORKER_MEM       --memory per container (default: 192g; for N-way parallel,
#                                          set to ⌊192/N⌋g for the same reason)
#   RELAY_NODE       host path to image_relay_node.py
#                                         (default: alongside this script)
#   RELAY_VENV       Python venv that image_relay_node.py needs activated
#                    (for cv2/numpy). Sourced after ROS_SETUP so the venv's
#                    python3 wins in PATH.
#                                         (default: $HOME/.venv)
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
#
# Bag collection:
#   This script's main output (besides score.yaml) is the per-trial MCAP bags
#   that aic_engine writes for downstream LeRobot conversion. Each trial in the
#   active config produces a directory of the shape:
#
#       $RESULTS/bag_trial_<id>_<timestamp>/<file>.mcap
#
#   In single-worker mode that's /home/ubuntu/aic-data/bag_trial_*/. In multi-
#   worker mode it's /home/ubuntu/aic-data/aic_results_<i>/bag_trial_*/. Both
#   land on the mounted aic-data filesystem so the host disk doesn't fill up.
#
#   To convert collected bags to LeRobot training format:
#       pixi run python scripts/mcap_to_lerobot.py <bag_dir_or_glob> ...
#   See scripts/mcap_to_lerobot.py --help for trimming / resolution options.

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
RELAY_VENV="${RELAY_VENV:-$HOME/.venv}"
ROS_SETUP="${ROS_SETUP:-/opt/ros/kilted/setup.bash}"

# Auto-derive PART_INDEX from PART_FILE basename (e.g. train_part_3.yaml → 3)
# so callers only need to set PART_FILE.
if [ -n "$PART_FILE" ] && [ -z "$PART_INDEX" ]; then
    _part_base="$(basename "$PART_FILE" .yaml)"
    PART_INDEX="${_part_base##*_}"
fi

# Default PART_INDEX to 0 if still unset. setup_workers.sh always names workers
# aic_worker_0 .. aic_worker_{N-1} (even for N=1), so the single-worker default
# must target aic_worker_0, not bare "aic_worker".
PART_INDEX="${PART_INDEX:-0}"

# Auto-derive ROUTER_PORT from PART_INDEX so concurrent workers on the same host
# get distinct Zenoh listen ports: worker 0 → 7447, worker 1 → 7448, …
# Must match the port baked into the worker's bind-mounted /entrypoint.sh by
# scripts/setup_workers.sh — that's what makes N>1 actually run on one host
# under distrobox's --network host default. The image's stock /entrypoint.sh
# hardcodes 7447 and can't be overridden via env, so the per-worker entrypoint
# bind-mount is how we vary the port.
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
# RESULTS_NO_SUFFIX=1 disables the auto-append of aic_results_<i> so callers
# (e.g. the chunked orchestrator) can place each chunk in its own pre-built
# subdir without an extra suffix level. NAME must also be explicit then.
if [ -n "${RESULTS_NO_SUFFIX:-}" ]; then
    NAME="${NAME:?RESULTS_NO_SUFFIX requires NAME to be set}"
    RESULTS="$_RESULTS_BASE"
elif [ -n "$PART_INDEX" ]; then
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

# image_relay_node imports cv2/numpy from a venv at $HOME/.venv; the activate
# script must exist or the relay subshell will run with the system python and
# crash on import.
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

# Zombie cleanup from prior failed runs. If a previous invocation got SIGKILLed
# (or a parent pixi run leaked a child), the leftover aic_model process keeps
# announcing itself on the host Zenoh fabric — aic_engine then fails the first
# trial with "More than one node with name 'aic_model' found". Same story for a
# leftover image_relay. Belt-and-suspenders: pkill them by command pattern so
# we start from a clean fabric.
#
# SKIP this in multi-worker mode: run_parallel_multi.sh runs N copies of this
# script concurrently, all sharing the host. A blanket `pkill -f aic_model`
# fired by worker_i kills the host-side aic_model of workers 0..i-1, leaving
# only the last-launched worker actually doing trials — a stealthy multi-
# worker regression. The orchestrator's own cleanup trap is responsible for
# cross-worker teardown; per-worker zombie cleanup is only valid for solo runs.
if [ -z "${MULTI_WORKER:-}" ]; then
    pkill -f 'aic_model'                  2>/dev/null || true
    pkill -f "$RELAY_NODE"                2>/dev/null || true
    # Give the OS a moment to actually reap them.
    sleep 1
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

# The container must already exist as a distrobox-managed container — that is
# how the host's libnvidia-gl-*-server gets bind-mounted in (Ogre2 needs it).
# Plain `docker run --gpus all` does not replicate distrobox's NVIDIA library
# mounts, so gz_server segfaults at Ogre2 init with "unable to find OpenGL 3+
# Rendering Subsystem". See docs/lambda_headless_setup.md.
if ! ${DOCKER} container inspect "$NAME" >/dev/null 2>&1; then
    echo "error: container '$NAME' does not exist." >&2
    echo "       Create it once via:  bash scripts/setup_workers.sh" >&2
    if [ -n "$PART_INDEX" ]; then
        echo "       (You're using PART_INDEX=$PART_INDEX, so make sure setup_workers.sh" >&2
        echo "        was run with NUM_WORKERS=$((PART_INDEX + 1)) or higher.)" >&2
    fi
    exit 1
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
echo "Container:    $NAME (distrobox, Zenoh @ 127.0.0.1:${ROUTER_PORT})"
echo "Relay:        $RELAY_NODE (host, ROS @ $ROS_SETUP)"
echo ""

# --- Stage 1: launch eval inside the distrobox container (mirrors T1) ----
#
# Container model: scripts/setup_workers.sh creates a long-lived distrobox
# container named "$NAME" with --nvidia (bind-mounts host NVIDIA GL/EGL libs
# so Ogre2 can resolve RenderSystem_GL3Plus) and a bind-mount of
# /home/ubuntu/aic-data → /home/ubuntu/aic-data at the same path inside the
# container. We just start it and `docker exec` the launch — the GL libs are
# already in place, no per-run `docker run` needed.

# Extra env to pass through to the in-container launch process. AIC_RESULTS_DIR
# uses the same host path because the bind-mount is identity-pathed.
exec_env_args=(
    -e "AIC_RESULTS_DIR=$RESULTS"
    -e "__EGL_VENDOR_LIBRARY_FILENAMES=/usr/share/glvnd/egl_vendor.d/10_nvidia.json"
)
if [ -n "${AIC_IMAGE_SCALE:-}" ]; then
    exec_env_args+=(-e "AIC_IMAGE_SCALE=$AIC_IMAGE_SCALE")
fi

# Part-file is on the mounted FS, which is bind-mounted at the same path inside
# the container — pass its host path directly as the launch arg.
part_launch_args=()
if [ -n "$PART_FILE" ]; then
    part_launch_args=("aic_engine_config_file:=$PART_FILE")
fi

# Make sure the container is up. `docker start` is a no-op if it's already
# running, otherwise it boots distrobox's init and returns when ready.
${DOCKER} start "$NAME" >/dev/null
echo "[$NAME] container started"

# Run the actual launch via `docker exec` in the background, streaming the
# launch's stdout/stderr straight into engine.log. We `wait` on this PID in
# Stage 4 instead of `docker wait`-ing the whole container — the container
# itself persists across runs so it can be re-used by the next invocation.
# Note: NO `-u root`. distrobox sets the container up around the `ubuntu`
# user (group memberships for /dev/dri/*, bashrc env, etc.); running as root
# bypasses that and Ogre2 fails with `unable to find OpenGL 3+ Rendering
# Subsystem` even though the bind-mounted libs are physically present.
ENGINE_LOG="$RESULTS/engine.log"
${DOCKER} exec \
    "${exec_env_args[@]}" \
    "$NAME" \
    /entrypoint.sh \
    "${part_launch_args[@]}" \
    ground_truth:="$GROUND_TRUTH" \
    gazebo_gui:=false launch_rviz:=false \
    start_aic_engine:=true \
    shutdown_on_aic_engine_exit:=true \
    > "$ENGINE_LOG" 2>&1 &
LAUNCH_PID=$!
echo "[$NAME] launch exec started (PID $LAUNCH_PID, log $ENGINE_LOG)"

# --- Cleanup trap ---------------------------------------------------------

RELAY_PID=""
POLICY_PID=""

cleanup() {
    if [ -n "$POLICY_PID" ]; then kill "$POLICY_PID" 2>/dev/null || true; fi
    if [ -n "$RELAY_PID" ];  then kill "$RELAY_PID"  2>/dev/null || true; fi
    if [ -n "${LAUNCH_PID:-}" ]; then kill "$LAUNCH_PID" 2>/dev/null || true; fi
    # Kill the in-container launch tree too — `kill $LAUNCH_PID` only closes our
    # docker-exec stream and doesn't always cascade to the process inside.
    # Don't `docker rm -f $NAME` — the container persists across runs.
    ${DOCKER} exec "$NAME" pkill -TERM -f /entrypoint.sh >/dev/null 2>&1 || true
    ${DOCKER} exec "$NAME" pkill -TERM -f ros2          >/dev/null 2>&1 || true
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
# Pin connect/endpoints to the per-worker port and disable multicast scouting
# so the host policy/relay can't multicast-discover another worker's router
# (the host Zenoh library defaults to multicast ON, unlike the JSON5 inside
# the container). Gossip stays enabled so the host policy's queryables
# (aic_model lifecycle services) are advertised through the router to the
# in-container aic_engine.
HOST_ZENOH_OVERRIDE='connect/endpoints=["tcp/localhost:'"${ROUTER_PORT}"'"]'
HOST_ZENOH_OVERRIDE+=';scouting/multicast/enabled=false'
HOST_ZENOH_OVERRIDE+=';transport/shared_memory/enabled=false'

# --- Stage 3a: launch image_relay on host (mirrors T3) --------------------

RELAY_LOG="$RESULTS/image_relay.log"
(
    # /opt/ros/kilted/setup.bash references $AMENT_TRACE_SETUP_FILES and other
    # vars without ${VAR:-} defaults, which trips the parent script's `set -u`.
    # The venv activate script is also third-party. Opt out of nounset for this
    # subshell only; errexit + pipefail still catch real failures.
    set +u
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
    # Defensive: pixi run sources its own activate script internally; opt out of
    # nounset for symmetry with the relay subshell, in case pixi's setup ever
    # grows an unguarded variable reference.
    set +u
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

# --- Stage 4: wait for the in-container launch to self-terminate ----------
#
# The container persists across runs; we wait on the docker-exec PID instead.
# shutdown_on_aic_engine_exit:=true makes /entrypoint.sh return once aic_engine
# is done, so this wait reliably terminates.

echo ""
echo "Waiting for $NAME launch to finish..."
echo "  (Tail aic_engine's view live: tail -f $ENGINE_LOG)"

# Ogre/EGL plugin-load failure is a *silent* killer: gz_server dies, but
# ros2 launch and the controller_manager spawner keep retrying forever, so
# the docker exec never returns and the orchestrator's per-chunk retry
# can't fire. Poll engine.log in the background; if we see the Ogre
# plugin-load failure signature, kill the launch fast so the retry loop
# can recreate the container.
#
# Communicate via a flag file rather than the exit code: when the watchdog
# kills `docker exec`, it propagates SIGTERM to /entrypoint.sh, which has
# `trap "kill -SIGINT ..." EXIT` and exits cleanly (return 0). The wait
# then returns 0 and the orchestrator's `&& break` mistakes it for success.
# A flag file lets us reliably signal "this attempt was a watchdog kill"
# regardless of how the killed process happens to exit.
WATCH_FIRED="${ENGINE_LOG}.watchdog_fired"
rm -f "$WATCH_FIRED"
(
    OGRE_RE='Unable to load Ogre Plugin|unable to find OpenGL 3\+ Rendering Subsystem'
    HEAP_RE='corrupted size vs\. prev_size'
    while kill -0 "$LAUNCH_PID" 2>/dev/null; do
        if [ -f "$ENGINE_LOG" ] && grep -qE "$OGRE_RE|$HEAP_RE" "$ENGINE_LOG"; then
            echo "[$NAME] WATCHDOG: detected gz init failure — killing launch so retry can fire" >&2
            touch "$WATCH_FIRED"
            kill "$LAUNCH_PID" 2>/dev/null || true
            ${DOCKER} exec "$NAME" pkill -TERM -f /entrypoint.sh >/dev/null 2>&1 || true
            ${DOCKER} exec "$NAME" pkill -TERM -f ros2 >/dev/null 2>&1 || true
            exit 0
        fi
        sleep 5
    done
) &
OGRE_WATCH_PID=$!

# Capture the launch exit code without letting `set -e` abort. Propagating it
# at the end lets run_parallel_multi.sh's per-worker retry loop detect crashes
# (gz_server SIGSEGV / heap corruption → /entrypoint.sh non-zero) and restart.
LAUNCH_EXIT=0
wait "$LAUNCH_PID" || LAUNCH_EXIT=$?
kill "$OGRE_WATCH_PID" 2>/dev/null || true
echo "[$NAME] launch done (exit $LAUNCH_EXIT)"

# If the watchdog fired, force non-zero exit so retry triggers regardless
# of how docker exec happened to return.
if [ -f "$WATCH_FIRED" ]; then
    rm -f "$WATCH_FIRED"
    LAUNCH_EXIT=42  # sentinel: watchdog killed for gz init failure
fi

# --- Summary --------------------------------------------------------------

echo ""
echo "=== Run finished ==="
if [ -f "$RESULTS/score.yaml" ]; then
    ls -lh "$RESULTS/score.yaml"
else
    echo "(no score.yaml in $RESULTS — check $POLICY_LOG, $RELAY_LOG, $ENGINE_LOG)"
fi

exit "$LAUNCH_EXIT"
