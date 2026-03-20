#!/usr/bin/env bash
set -euo pipefail

WORKSPACE="${AIC_WORKSPACE:-$HOME/ws_aic/src/aic}"
BASE_RESULTS="${AIC_BASE_RESULTS:-$HOME/ws_aic/src/aic/aic_results}"
NUM_RUNS="${NUM_RUNS:-10}"
CONTAINER_NAME="${AIC_CONTAINER:-aic_eval}"
POLICY="${AIC_POLICY:-aic_example_policies.ros.CheatCode}"

ENGINE_READY_TIMEOUT="${ENGINE_READY_TIMEOUT:-300}"
RUN_COMPLETE_TIMEOUT="${RUN_COMPLETE_TIMEOUT:-540}"
CONTAINER_RESTART_WAIT="${CONTAINER_RESTART_WAIT:-20}"
BETWEEN_RUN_WAIT="${BETWEEN_RUN_WAIT:-5}"
ZENOH_PORT="${ZENOH_PORT:-7447}"

DISPLAY="${DISPLAY:-:0}"
XAUTHORITY="${XAUTHORITY:-$HOME/.Xauthority}"

RANDOM_SCENE="${RANDOM_SCENE:-true}"

NOMINAL_SCENE=(
    "spawn_task_board:=true"
    "task_board_x:=0.300"
    "task_board_y:=-0.100"
    "task_board_z:=1.200"
    "task_board_roll:=0.000"
    "task_board_pitch:=0.000"
    "task_board_yaw:=0.785"
    "sfp_mount_rail_0_present:=true"
    "sfp_mount_rail_0_translation:=-0.080"
    "sfp_mount_rail_1_present:=false"
    "sfp_mount_rail_1_translation:=0.000"
    "sc_mount_rail_0_present:=true"
    "sc_mount_rail_0_translation:=-0.090"
    "sc_mount_rail_1_present:=false"
    "sc_mount_rail_1_translation:=0.000"
    "nic_card_mount_0_present:=true"
    "nic_card_mount_0_translation:=0.005"
    "nic_card_mount_1_present:=false"
    "nic_card_mount_1_translation:=0.000"
    "nic_card_mount_2_present:=false"
    "nic_card_mount_2_translation:=0.000"
    "sc_port_0_present:=true"
    "sc_port_0_translation:=-0.040"
    "spawn_cable:=true"
    "cable_type:=sfp_sc_cable"
    "attach_cable_to_gripper:=true"
)

ENGINE_READY_PATTERN="No node with name 'aic_model' found\|aic_engine initialized\|Waiting for aic_model\|Starting AIC Engine Run"
RUN_COMPLETE_PATTERN="\[aic_engine-[0-9]*\]: process has finished cleanly"

ENGINE_PID=""
POLICY_PID=""

rand_float_3d() {
    local lo=$1 hi=$2
    local range=$(( hi - lo + 1 ))
    local raw=$(( lo + RANDOM % range ))
    if [ "$raw" -lt 0 ]; then
        local abs=$(( -raw ))
        printf -- "-%d.%03d" $(( abs / 1000 )) $(( abs % 1000 ))
    else
        printf "%d.%03d" $(( raw / 1000 )) $(( raw % 1000 ))
    fi
}

rand_bool() {
    [ $(( RANDOM % 2 )) -eq 0 ] && echo "true" || echo "false"
}

rand_choice() {
    local n=$#
    local idx=$(( RANDOM % n + 1 ))
    eval echo "\${$idx}"
}

generate_scene_params() {
    local tb_x tb_y tb_z tb_yaw
    tb_x=$(rand_float_3d  200  450)
    tb_y=$(rand_float_3d -200   50)
    tb_z=$(rand_float_3d 1100 1350)
    tb_yaw=$(rand_float_3d   0 1571)

    local cable_type
    cable_type=$(rand_choice sfp_sc_cable sfp_sc_cable_reversed)

    local sfp0_present sfp0_trans sfp1_present sfp1_trans
    sfp0_present=$(rand_bool)
    sfp0_trans=$(rand_float_3d -120 80)
    sfp1_present=$(rand_bool)
    sfp1_trans=$(rand_float_3d -120 80)
    if [ "$sfp0_present" = "false" ] && [ "$sfp1_present" = "false" ]; then
        sfp0_present="true"
        sfp0_trans=$(rand_float_3d -120 80)
    fi

    local sc0_present sc0_trans sc1_present sc1_trans
    sc0_present=$(rand_bool)
    sc0_trans=$(rand_float_3d -120 80)
    sc1_present=$(rand_bool)
    sc1_trans=$(rand_float_3d -120 80)

    local nic0_present nic0_trans nic1_present nic1_trans nic2_present nic2_trans
    nic0_present=$(rand_bool)
    nic0_trans=$(rand_float_3d -80 80)
    nic1_present=$(rand_bool)
    nic1_trans=$(rand_float_3d -80 80)
    nic2_present=$(rand_bool)
    nic2_trans=$(rand_float_3d -80 80)

    local sc_port0_present sc_port0_trans
    sc_port0_present=$(rand_bool)
    sc_port0_trans=$(rand_float_3d -80 60)

    if [ "$sc0_present"      = "false" ] && \
       [ "$sc1_present"      = "false" ] && \
       [ "$nic0_present"     = "false" ] && \
       [ "$nic1_present"     = "false" ] && \
       [ "$nic2_present"     = "false" ] && \
       [ "$sc_port0_present" = "false" ]; then
        if [ $(( RANDOM % 2 )) -eq 0 ]; then
            sc0_present="true"
            sc0_trans=$(rand_float_3d -120 80)
        else
            sc_port0_present="true"
            sc_port0_trans=$(rand_float_3d -80 60)
        fi
    fi

    SCENE_PARAMS=(
        "spawn_task_board:=true"
        "task_board_x:=${tb_x}"
        "task_board_y:=${tb_y}"
        "task_board_z:=${tb_z}"
        "task_board_roll:=0.000"
        "task_board_pitch:=0.000"
        "task_board_yaw:=${tb_yaw}"
        "sfp_mount_rail_0_present:=${sfp0_present}"
        "sfp_mount_rail_0_translation:=${sfp0_trans}"
        "sfp_mount_rail_1_present:=${sfp1_present}"
        "sfp_mount_rail_1_translation:=${sfp1_trans}"
        "sc_mount_rail_0_present:=${sc0_present}"
        "sc_mount_rail_0_translation:=${sc0_trans}"
        "sc_mount_rail_1_present:=${sc1_present}"
        "sc_mount_rail_1_translation:=${sc1_trans}"
        "nic_card_mount_0_present:=${nic0_present}"
        "nic_card_mount_0_translation:=${nic0_trans}"
        "nic_card_mount_1_present:=${nic1_present}"
        "nic_card_mount_1_translation:=${nic1_trans}"
        "nic_card_mount_2_present:=${nic2_present}"
        "nic_card_mount_2_translation:=${nic2_trans}"
        "sc_port_0_present:=${sc_port0_present}"
        "sc_port_0_translation:=${sc_port0_trans}"
        "spawn_cable:=true"
        "cable_type:=${cable_type}"
        "attach_cable_to_gripper:=true"
    )
}

save_scene_metadata() {
    local run_dir="$1"
    local seed="$2"
    shift 2
    local meta_file="$run_dir/scene_params.txt"
    {
        echo "# SEED=${seed}"
        echo "# Generated: $(date)"
        echo ""
        for param in "$@"; do
            echo "$param"
        done
    } > "$meta_file"
    echo "$(ts) Scene params → $meta_file"
}

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

# Pre-flight checks
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
    echo "CheatCode ${NUM_RUNS}-Run Summary (Random Scene Sampling)"
    echo "Policy        : $POLICY"
    echo "Random scene  : $RANDOM_SCENE"
    echo "Workspace     : $WORKSPACE"
    echo "Started       : $(date)"
    echo "--------------------------------------------------------------------"
    printf "%-5s  %-38s  %-16s  %-8s  %s\n" \
        "Run" "Folder" "Status" "Ready(s)" "Cable type"
    echo "--------------------------------------------------------------------"
} > "$SUMMARY"

banner "Pulling aic_eval image..."
docker pull ghcr.io/intrinsic-dev/aic/aic_eval:latest

banner "Creating distrobox container (skipped if already exists)..."
distrobox create -r --nvidia \
    -i ghcr.io/intrinsic-dev/aic/aic_eval:latest \
    "$CONTAINER_NAME" || true

banner "Pre-run cleanup..."
kill_container_procs
reset_container

banner "Warming up distrobox entry..."
distrobox enter -r "$CONTAINER_NAME" -- bash -c "echo 'Warmup pass 1 complete'" \
    >/dev/null 2>&1
sleep 5
distrobox enter -r "$CONTAINER_NAME" -- bash -c "echo 'Warmup pass 2 complete'" \
    >/dev/null 2>&1
echo "$(ts) Distrobox entry fully warmed up."

banner "Starting $NUM_RUNS runs of $POLICY  [RANDOM_SCENE=${RANDOM_SCENE}]"
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

    RUN_SEED=$(( $(date +%N) % 32767 + i * 1337 ))
    RANDOM=$RUN_SEED

    SCENE_PARAMS=()
    if [ "${RANDOM_SCENE}" = "true" ]; then
        generate_scene_params
        save_scene_metadata "$RUN_DIR" "$RUN_SEED" "${SCENE_PARAMS[@]}"
    else
        SCENE_PARAMS=("${NOMINAL_SCENE[@]}")
        save_scene_metadata "$RUN_DIR" "nominal" "${SCENE_PARAMS[@]}"
    fi

    CABLE_TYPE="unknown"
    for p in "${SCENE_PARAMS[@]}"; do
        [[ "$p" == cable_type:=* ]] && CABLE_TYPE="${p#cable_type:=}" && break
    done

    echo "$(ts) Scene params for this run:"
    for p in "${SCENE_PARAMS[@]}"; do echo "         $p"; done

    banner "Resetting container for clean world state..."
    kill_container_procs
    reset_container
    
    # FIX: Remove stale world SDF to prevent double task board spawn
    echo "$(ts) Clearing stale world SDF from previous run..."
    distrobox enter -r "$CONTAINER_NAME" -- bash -c "rm -f /tmp/aic.sdf"

    SCENE_PARAM_STR="${SCENE_PARAMS[*]}"

    distrobox enter -r "$CONTAINER_NAME" -- bash -c \
        "export DISPLAY='${DISPLAY}' \
         && export XAUTHORITY='${XAUTHORITY}' \
         && export AIC_RESULTS_DIR='$RUN_DIR' \
         && /entrypoint.sh \
                ${SCENE_PARAM_STR} \
                ground_truth:=true \
                start_aic_engine:=true" \
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
    echo "  Cable type  : $CABLE_TYPE"

    printf "%-5s  %-38s  %-16s  %-8s  %s\n" \
        "$i" "${RUN_LABEL}_${TIMESTAMP}" "$RUN_STATUS" \
        "${ENGINE_READY_SECS}s" "$CABLE_TYPE" \
        >> "$SUMMARY"

    if [ "$i" -lt "$NUM_RUNS" ]; then
        banner "Cleaning up before run $((i + 1))..."
        kill_container_procs
        sleep "$BETWEEN_RUN_WAIT"
    fi
done

{
    echo "--------------------------------------------------------------------"
    echo "Completed: $(date)"
} >> "$SUMMARY"

banner "All $NUM_RUNS runs complete!"
echo ""
cat "$SUMMARY"
echo ""
echo "Full results: $BASE_RESULTS"