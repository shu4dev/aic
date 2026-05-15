#!/bin/bash
# Train ACT policy on Lambda Cloud (A100 40 GB / 30 vCPU / 200 GB RAM).
#
# Adapted from scripts/train.sh — keeps the same hyperparameters and
# val-loss sidecar architecture, but with paths and batch size tuned for
# Lambda's filesystem layout and the A100's smaller (vs 80 GB) VRAM.
#
# Usage:
#   bash data_checl/aic/scripts/train_lambda.sh
#   bash data_checl/aic/scripts/train_lambda.sh --steps=50000 --batch_size=96
#
# Environment overrides (all optional):
#   DATASET_DIR    LeRobot dataset path     (default: ~/aic_lerobot_dataset)
#   OUTPUT_ROOT    Where checkpoints land   (default: /home/ubuntu/aic-data/outputs/train — mounted FS)
#   TRAIN_STEPS    Training steps           (default: 100000)
#   BATCH_SIZE     Per-step batch           (default: 64 — A100 40 GB safe)
#   SAVE_FREQ      Checkpoint frequency     (default: 5000)
#   NUM_WORKERS    DataLoader workers       (default: 16)
#   CHUNK_SIZE     Action chunk            (default: 40 = 2 s @ 20 Hz)
#   OPTIMIZER_LR   Learning rate            (default: 1e-5)
#   TRAIN_TAG      Short tag in run name   (default: dataset basename)
#   WANDB_PROJECT  Project name             (default: aic-act)
#   WANDB_ENTITY   Wandb team/user          (default: your login)
#   VAL_LOSS       Run val-loss sidecar    (default: 1, set 0 to disable)
#
# Inspecting the run:
#   - Live console:       this terminal (or `tail -f` on the log paths below)
#   - WandB dashboard:    URL is printed once training writes its first step
#   - GPU utilization:    `nvidia-smi -l 2` in another shell
#   - Disk:               `df -h ~` (dataset + checkpoints share root)
#   - Val loss (sidecar): plotted in wandb under val/* keys, also in
#                         <output_dir>/val_loss.jsonl

set -e

# ── Paths ────────────────────────────────────────────────────────────────
REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
DATASET_DIR="${DATASET_DIR:-${HOME}/aic/aic_lerobot_dataset}"
OUTPUT_ROOT="${OUTPUT_ROOT:-/home/ubuntu/aic-data/outputs/train}"

# Prefer ~/.venv (provisioned by scripts/lambda_bootstrap.sh from
# scripts/requirements.txt), then fall back to pixi env, then system python.
# We keep the legacy variable name PIXI_PYTHON to avoid touching every call
# site below, but it now points at whichever interpreter actually has
# lerobot-train installed.
VENV_DIR="${VENV_DIR:-${HOME}/.venv}"
if [[ -x "${VENV_DIR}/bin/lerobot-train" ]]; then
    # shellcheck disable=SC1091
    source "${VENV_DIR}/bin/activate"
    PIXI_PYTHON="${VENV_DIR}/bin/python3"
    LEROBOT_TRAIN="${VENV_DIR}/bin/lerobot-train"
    echo "Using ~/.venv at ${VENV_DIR}"
elif [[ -x "${REPO_ROOT}/.pixi/envs/default/bin/lerobot-train" ]]; then
    PIXI_PYTHON="${REPO_ROOT}/.pixi/envs/default/bin/python3"
    LEROBOT_TRAIN="${REPO_ROOT}/.pixi/envs/default/bin/lerobot-train"
    echo "Using pixi env at ${REPO_ROOT}/.pixi/envs/default"
elif command -v lerobot-train >/dev/null 2>&1; then
    PIXI_PYTHON="$(command -v python3)"
    LEROBOT_TRAIN="$(command -v lerobot-train)"
    echo "Using system python: ${PIXI_PYTHON}"
else
    echo "ERROR: lerobot-train not found. Run scripts/lambda_bootstrap.sh"
    echo "(which installs scripts/requirements.txt into ~/.venv), or:"
    echo "  pip install --user lerobot==0.5.1"
    exit 1
fi

# ── Hyperparameters (matching scripts/train.sh) ─────────────────────────
TRAIN_STEPS="${TRAIN_STEPS:-100000}"
BATCH_SIZE="${BATCH_SIZE:-64}"          # 40 GB A100 safe; bump to 96/128 if no OOM
SAVE_FREQ="${SAVE_FREQ:-5000}"
NUM_WORKERS="${NUM_WORKERS:-16}"        # 30 vCPU - 4 (sidecar) - OS = ~24 avail
CHUNK_SIZE="${CHUNK_SIZE:-40}"          # 2 s lookahead at 20 Hz; ACT default 100 is overkill
OPTIMIZER_LR="${OPTIMIZER_LR:-1e-5}"

# ── Naming ──────────────────────────────────────────────────────────────
DATASET_NAME="$(basename "${DATASET_DIR}")"
TRAIN_TAG="${TRAIN_TAG:-${DATASET_NAME}}"
TIMESTAMP="$(date +%Y%m%d_%H%M%S)"
JOB_NAME="${TIMESTAMP}_${TRAIN_TAG}"
TRAIN_OUTPUT="${OUTPUT_ROOT}/${JOB_NAME}"

# repo_id is just for wandb run naming; not pushed to hub
REPO_ID="$(basename "$(dirname "${DATASET_DIR}")")/${DATASET_NAME}"

if [[ ! -d "${DATASET_DIR}" ]]; then
    echo "ERROR: Dataset not found at ${DATASET_DIR}"
    echo "Run mcap_to_lerobot_trimmed.py first, or set DATASET_DIR=..."
    exit 1
fi

# ── Video backend (A100 = Ampere SM 8.0, torchcodec works) ─────────────
# Override with VIDEO_BACKEND=pyav if torchcodec hits end-of-episode edge cases.
VIDEO_BACKEND="${VIDEO_BACKEND:-torchcodec}"
FRAME_CACHE_ENV=""

if [[ "${VIDEO_BACKEND}" == "pyav" ]] || ! "${PIXI_PYTHON}" -c "import torchcodec" 2>/dev/null; then
    echo "Using PyAV + frame cache (torchcodec unavailable or forced)"
    VIDEO_BACKEND="pyav"
    CACHE_PATH="${DATASET_DIR}/frame_cache_meta.json"
    [[ -f "${CACHE_PATH}" ]] || CACHE_PATH="${DATASET_DIR}/frame_cache.pt"
    if [[ ! -f "${CACHE_PATH}" ]]; then
        echo "ERROR: PyAV path needs a frame cache at ${DATASET_DIR}"
        echo "Build it via: ${PIXI_PYTHON} ${REPO_ROOT}/scripts/build_frame_cache.py --dataset-dir ${DATASET_DIR}"
        exit 1
    fi
    FRAME_CACHE_ENV="LEROBOT_FRAME_CACHE=${CACHE_PATH}"
    echo "Frame cache: ${CACHE_PATH}"
else
    echo "Using torchcodec (A100, no frame cache needed)"
fi

# ── Val-episode split (read from dataset's meta/val_episodes.json) ─────
VAL_EPISODES_FILE="${DATASET_DIR}/meta/val_episodes.json"
EPISODES_FLAG=()
VAL_LOSS="${VAL_LOSS:-1}"
N_TRAIN=0
N_VAL=0
if [[ "${VAL_LOSS}" == "1" && -f "${VAL_EPISODES_FILE}" ]]; then
    TRAIN_EPISODES=$("${PIXI_PYTHON}" -c "
import json, sys
v = json.load(open('${VAL_EPISODES_FILE}'))
val_set = set(v['val_episodes'])
train = [e for e in range(v['n_total']) if e not in val_set]
sys.stdout.write('[' + ','.join(str(e) for e in train) + ']')
")
    EPISODES_FLAG=(--dataset.episodes="${TRAIN_EPISODES}")
    N_TRAIN=$(echo "$TRAIN_EPISODES" | tr ',' '\n' | wc -l)
    N_VAL=$("${PIXI_PYTHON}" -c "import json; print(len(json.load(open('${VAL_EPISODES_FILE}'))['val_episodes']))")
fi

# ── Pre-flight summary ──────────────────────────────────────────────────
START_TIME=$(date +%s)
START_HUMAN=$(date '+%Y-%m-%d %H:%M:%S')

mkdir -p "${OUTPUT_ROOT}"
LEROBOT_TRAIN_LOG="${OUTPUT_ROOT}/${JOB_NAME}.lerobot_train.log"

echo ""
echo "════════════════════════════════════════════════════════════════════"
echo "  ACT Training on Lambda (A100 40 GB)"
echo "════════════════════════════════════════════════════════════════════"
echo "  Started:       ${START_HUMAN}"
echo "  Job name:      ${JOB_NAME}"
echo "  Dataset:       ${DATASET_DIR}"
echo "  Train/Val:     ${N_TRAIN} / ${N_VAL} episodes"
echo "  Output dir:    ${TRAIN_OUTPUT}"
echo "  Training log:  ${LEROBOT_TRAIN_LOG}"
echo "  Backend:       ${VIDEO_BACKEND}"
echo "  Steps:         ${TRAIN_STEPS}"
echo "  Batch size:    ${BATCH_SIZE}"
echo "  Workers:       ${NUM_WORKERS}"
echo "  Chunk size:    ${CHUNK_SIZE} (${CHUNK_SIZE} steps @ 20 Hz = $(awk "BEGIN{print ${CHUNK_SIZE}/20}") s)"
echo "  LR:            ${OPTIMIZER_LR}"
echo "  WandB:         enabled (URL appears after first log_freq=200 steps)"
echo "════════════════════════════════════════════════════════════════════"
echo ""

# ── Launch trainer (FP32; BF16 breaks ACT's VAE KL — see scripts/train.sh note) ─
env ${FRAME_CACHE_ENV} "${LEROBOT_TRAIN}" \
    --dataset.repo_id="${REPO_ID}" \
    --dataset.root="${DATASET_DIR}" \
    --policy.type=act \
    --policy.chunk_size="${CHUNK_SIZE}" \
    --policy.n_action_steps="${CHUNK_SIZE}" \
    --output_dir="${TRAIN_OUTPUT}" \
    --job_name="${JOB_NAME}" \
    --policy.device=cuda \
    --steps="${TRAIN_STEPS}" \
    --batch_size="${BATCH_SIZE}" \
    --num_workers="${NUM_WORKERS}" \
    --save_checkpoint=true \
    --save_freq="${SAVE_FREQ}" \
    --log_freq=200 \
    --wandb.enable=true \
    --wandb.project="${WANDB_PROJECT:-aic-act}" \
    ${WANDB_ENTITY:+--wandb.entity="${WANDB_ENTITY}"} \
    --policy.push_to_hub=false \
    --dataset.video_backend="${VIDEO_BACKEND}" \
    --dataset.image_transforms.enable="${IMAGE_TRANSFORMS_ENABLE:-true}" \
    --use_policy_training_preset=false \
    --optimizer.type=adamw \
    --optimizer.lr="${OPTIMIZER_LR}" \
    --policy.optimizer_lr="${OPTIMIZER_LR}" \
    --policy.optimizer_lr_backbone="${OPTIMIZER_LR}" \
    --optimizer.weight_decay=0.0001 \
    --scheduler.type=diffuser \
    --scheduler.name=constant_with_warmup \
    --scheduler.num_warmup_steps=500 \
    "${EPISODES_FLAG[@]}" \
    "$@" > >(tee "${LEROBOT_TRAIN_LOG}") 2>&1 &
TRAIN_PID=$!

# ── Spawn val-loss sidecar (concurrent on same GPU) ─────────────────────
SIDECAR_PID=""
if [[ "${VAL_LOSS}" == "1" && -f "${VAL_EPISODES_FILE}" ]]; then
    # Wait up to 5 min for the wandb run URL to appear in the log.
    WANDB_RUN_ID=""
    for _ in $(seq 1 60); do
        if [[ -f "${LEROBOT_TRAIN_LOG}" ]]; then
            WANDB_RUN_ID=$(grep -oE 'wandb\.ai/[^ ]+/runs/[a-z0-9]+' "${LEROBOT_TRAIN_LOG}" \
                | head -1 | awk -F/ '{print $NF}')
            [[ -n "${WANDB_RUN_ID}" ]] && break
        fi
        sleep 5
    done
    if [[ -d "${TRAIN_OUTPUT}" ]] && [[ -f "${REPO_ROOT}/scripts/val_loss_sidecar.py" ]]; then
        nohup env ${FRAME_CACHE_ENV} "${PIXI_PYTHON}" "${REPO_ROOT}/scripts/val_loss_sidecar.py" \
            "${TRAIN_OUTPUT}" \
            --num-workers 4 \
            ${WANDB_RUN_ID:+--wandb-run-id "${WANDB_RUN_ID}"} \
            > "${TRAIN_OUTPUT}/val_loss_sidecar.log" 2>&1 &
        SIDECAR_PID=$!
        echo ""
        echo "Val-loss sidecar PID ${SIDECAR_PID}"
        [[ -n "${WANDB_RUN_ID}" ]] && echo "  WandB run: https://wandb.ai/.../runs/${WANDB_RUN_ID}"
        echo "  Log: ${TRAIN_OUTPUT}/val_loss_sidecar.log"
        echo ""
    fi
fi

# Reap sidecar on exit (success or failure).
trap 'kill ${SIDECAR_PID} 2>/dev/null || true' EXIT
wait ${TRAIN_PID}
TRAIN_RC=$?

# ── Post-run summary with elapsed time ──────────────────────────────────
END_TIME=$(date +%s)
ELAPSED=$((END_TIME - START_TIME))
H=$((ELAPSED / 3600))
M=$(((ELAPSED % 3600) / 60))
S=$((ELAPSED % 60))

echo ""
echo "════════════════════════════════════════════════════════════════════"
echo "  Training complete (exit ${TRAIN_RC})"
echo "════════════════════════════════════════════════════════════════════"
echo "  Started:    ${START_HUMAN}"
echo "  Ended:      $(date '+%Y-%m-%d %H:%M:%S')"
echo "  Elapsed:    ${H}h ${M}m ${S}s"
echo "  Per-step:   $(awk "BEGIN{printf \"%.3f\", ${ELAPSED}/${TRAIN_STEPS}}") s/step"
echo "  Checkpoint: ${TRAIN_OUTPUT}/checkpoints/"
echo "  Train log:  ${LEROBOT_TRAIN_LOG}"
[[ -f "${TRAIN_OUTPUT}/val_loss.jsonl" ]] && echo "  Val loss:   ${TRAIN_OUTPUT}/val_loss.jsonl"
echo ""
echo "  Latest checkpoint:"
ls -1d "${TRAIN_OUTPUT}/checkpoints/"*/ 2>/dev/null | sort -V | tail -1 || echo "    (none)"
echo "════════════════════════════════════════════════════════════════════"
exit "${TRAIN_RC}"
