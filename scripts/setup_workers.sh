#!/bin/bash
# One-time setup for run_parallel.sh / run_parallel_multi.sh on Lambda.
#
# Creates N persistent distrobox-managed containers (aic_worker_0 .. aic_worker_{N-1})
# from ghcr.io/shu4dev/aic-eval:v1. Distrobox's --nvidia flag bind-mounts the
# host's libnvidia-gl-580-server libraries into the container, which is what
# Ogre2 / gz-rendering-ogre2 needs to find the OpenGL 3+ RenderSystem. Plain
# `docker run --gpus all` does NOT do those bind-mounts comprehensively — the
# symptom is `unable to find OpenGL 3+ Rendering Subsystem` followed by
# component_container exit -11. See docs/lambda_headless_setup.md for the
# original 3-terminal manual flow this matches.
#
# This script is idempotent: re-running it is a no-op if the workers already
# exist. Use it on a fresh Lambda box, or after `distrobox rm` to recreate.
#
# Prerequisites (done once per Lambda box — see docs/lambda_headless_setup.md):
#   - sudo apt install -y libnvidia-gl-580-server  (matching the kernel module)
#   - sudo apt install -y distrobox
#   - docker login ghcr.io  (to pull the private aic-eval image)
#
# Usage:
#   bash scripts/setup_workers.sh                  # creates aic_worker_0
#   NUM_WORKERS=4 bash scripts/setup_workers.sh    # creates aic_worker_0..3
#
# Environment overrides (all optional):
#   NUM_WORKERS      how many worker containers to create  (default: 1)
#   IMAGE            container image                       (default: ghcr.io/shu4dev/aic-eval:v1)
#   AIC_DATA_DIR     host mounted-FS to expose to workers  (default: /home/ubuntu/aic-data)
#   NAME_PREFIX      worker container name prefix          (default: aic_worker)

set -euo pipefail

NUM_WORKERS="${NUM_WORKERS:-1}"
IMAGE="${IMAGE:-ghcr.io/shu4dev/aic-eval:v1}"
AIC_DATA_DIR="${AIC_DATA_DIR:-/home/ubuntu/aic-data}"
NAME_PREFIX="${NAME_PREFIX:-aic_worker}"

if ! command -v distrobox >/dev/null 2>&1; then
    echo "error: distrobox not found on PATH — run 'sudo apt install -y distrobox' first" >&2
    exit 1
fi

if ! command -v docker >/dev/null 2>&1; then
    echo "error: docker not found on PATH" >&2
    exit 1
fi

export DBX_CONTAINER_MANAGER=docker

# Pre-pull the image once so each distrobox create doesn't redo it.
if ! sudo docker image inspect "$IMAGE" >/dev/null 2>&1; then
    echo "Pulling $IMAGE ..."
    sudo docker pull "$IMAGE"
fi

# Make sure the mounted-FS dir exists so distrobox doesn't refuse the bind.
mkdir -p "$AIC_DATA_DIR"

echo "=== Setting up $NUM_WORKERS distrobox worker container(s) ==="
echo "Image:      $IMAGE"
echo "Mount:      $AIC_DATA_DIR  (visible inside container at the same path)"
echo "Name prefix: $NAME_PREFIX"
echo ""

for i in $(seq 0 $((NUM_WORKERS - 1))); do
    name="${NAME_PREFIX}_${i}"
    if sudo docker container inspect "$name" >/dev/null 2>&1; then
        echo "[$name] already exists — skipping"
        continue
    fi

    echo "[$name] creating via distrobox --nvidia ..."
    # --root      run with sudo / root-owned container
    # --nvidia    bind-mount host NVIDIA GL/EGL libs (the critical bit for Ogre2)
    # --no-entry  don't drop the user into a shell at the end
    # --additional-flags lets us pass arbitrary `docker create` flags. We bind
    # the mounted-FS so aic_engine's output ($AIC_RESULTS_DIR) lands on the host
    # at the same path it sees inside.
    distrobox create \
        --root \
        --nvidia \
        --no-entry \
        --name "$name" \
        --image "$IMAGE" \
        --additional-flags "-v ${AIC_DATA_DIR}:${AIC_DATA_DIR}"

    # distrobox lazily starts containers on first `enter`. Pre-start now so the
    # first run of run_parallel.sh doesn't pay that cost.
    sudo docker start "$name" >/dev/null
done

echo ""
echo "=== Done ==="
echo "Containers:"
for i in $(seq 0 $((NUM_WORKERS - 1))); do
    name="${NAME_PREFIX}_${i}"
    state="$(sudo docker inspect -f '{{.State.Status}}' "$name" 2>/dev/null || echo missing)"
    echo "  $name  ($state)"
done

cat <<MSG

Next steps:
  - Verify NVIDIA EGL works inside one of them. The env var matters: libglvnd
    needs __EGL_VENDOR_LIBRARY_FILENAMES set to actually pick NVIDIA's ICD,
    otherwise eglinfo silently falls back to llvmpipe even though the libs and
    /dev/nvidia* are all present:
      sudo docker exec \\
          -e __EGL_VENDOR_LIBRARY_FILENAMES=/usr/share/glvnd/egl_vendor.d/10_nvidia.json \\
          ${NAME_PREFIX}_0 eglinfo 2>&1 | grep -iE 'vendor|renderer|nvidia' | head -5
    Expected output starts with "EGL vendor string: NVIDIA" and includes a line
    like "OpenGL core profile renderer: NVIDIA …".

  - Run an eval worker (run_parallel.sh sets the EGL env automatically):
      bash scripts/run_parallel.sh                      # uses ${NAME_PREFIX}_0
  - Tear down (if you ever want to recreate):
      sudo docker rm -f ${NAME_PREFIX}_0
MSG
