#!/bin/bash
# Lambda Cloud cloud-init bootstrap for an AIC eval worker box.
#
# Lambda passes this script as `user_data` to the instance, and cloud-init
# runs it as root on first boot. By the time we're done, the box has:
#   * distrobox + docker installed
#   * the aic repo cloned into ~ubuntu/ws_aic/src/aic
#   * pixi installed + the env materialized
#   * ~/.venv set up for image_relay
#   * 4 distrobox workers pre-created (aic_worker_0..3)
#   * a /home/ubuntu/.bootstrap_done marker file once everything is ready
#
# The fleet driver (scripts/lambda_fleet.py) polls for that marker before
# considering a box ready to receive work.
#
# Slow path (~25-30 min): does a full pixi install from scratch.
# Fast path: if you attach a Lambda persistent filesystem with a pre-built
# ~/ws_aic and ~/.pixi/envs/default, this bootstrap just mounts it and
# skips the heavy installs. Add the mount logic at the {{MOUNT_SHARED_FS}}
# marker below when you have one set up.
#
# Required substitution from the driver:
#   {{GHCR_TOKEN}}  GitHub Container Registry PAT (read:packages scope).
#                   Needed to pull ghcr.io/shu4dev/aic-eval:v1. Substitute
#                   in via Python before passing as user_data.

set -e
exec > >(tee -a /var/log/aic-bootstrap.log) 2>&1
echo "=== AIC bootstrap starting at $(date -Iseconds) ==="

GHCR_TOKEN="{{GHCR_TOKEN}}"

# --- 0. Precondition: Lambda Stack 24.04 (Ubuntu 24.04 Noble) -------------
# ROS 2 Kilted Kaiju (May 2025 release) is published only for Ubuntu 24.04
# Noble. There are no `ros-kilted-*` packages in the jammy apt source, so
# on a 22.04 (or older) image, step 1b's `apt install ros-kilted-ros-base`
# would silently fail with "Unable to locate package" — but only AFTER the
# ~20-min pixi install has already burned. Fail fast here in 1 second.
#
# The intended fleet base image is Lambda Stack 24.04, which is the default
# for gpu_1x_a100 + similar types on Lambda Cloud. Pin it explicitly via
# the launch payload's `image` field (in scripts/lambda_fleet.py:cmd_launch)
# if you're paranoid about drift in Lambda's defaults.
DETECTED_CODENAME="$(. /etc/os-release && echo "${UBUNTU_CODENAME:-}")"
if [ "$DETECTED_CODENAME" != "noble" ]; then
    echo "" >&2
    echo "FATAL: this bootstrap requires Ubuntu 24.04 Noble (Lambda Stack 24.04)." >&2
    echo "Detected: '${DETECTED_CODENAME:-unknown}' — ROS 2 Kilted Kaiju has no" >&2
    echo "apt packages for this codename. Re-launch the instance with the" >&2
    echo "Lambda Stack 24.04 image." >&2
    exit 1
fi
echo "Precondition OK: Ubuntu 24.04 Noble detected."

# --- 1. System packages ---------------------------------------------------
# Single apt block up front so the heavy `pixi install` later doesn't run
# for 20 min before discovering distrobox or libnvidia-gl-*-server is
# missing. Includes:
#   distrobox        : container runner used by scripts/setup_workers.sh
#   git              : cloning the repo
#   python3.12 + -venv + -dev :
#                      `python3.12 -m venv ~/.venv` needs -venv; -dev is for
#                      any C extensions in scripts/requirements.txt. We pin
#                      3.12 explicitly so the venv interpreter is stable even
#                      if Ubuntu's default python3 shifts in a future point
#                      release.
#   ffmpeg           : required downstream by scripts/mcap_to_lerobot.py
#                      for h264 encoding into the LeRobot dataset
#   jq, curl, rsync, ca-certificates : misc fleet utilities
apt-get update -qq
apt-get install -y -qq \
    distrobox \
    git \
    python3.12 \
    python3.12-venv \
    python3.12-dev \
    ffmpeg \
    jq \
    curl \
    rsync \
    ca-certificates

# Docker is preinstalled on Lambda's Ubuntu image, but make sure the user
# is in the docker group so they can talk to the daemon without sudo (we
# still `sudo docker` everywhere for safety, but this avoids surprises).
usermod -aG docker ubuntu || true

# --- 1a. NVIDIA server-channel GL libs (driver-matched) -------------------
# This is the most fragile prerequisite and must land BEFORE setup_workers.sh
# (which calls `distrobox create --nvidia` — distrobox snapshots the host's
# libEGL_nvidia.so / libGLX_nvidia.so AT CREATE TIME, so installing them
# later won't help). The package must be:
#   1. The `-server` channel — Lambda's `-server-open` kernel rejects bare
#      `libnvidia-gl-${MAJOR}` (silently enumerates 0 EGL devices, Ogre2
#      then aborts with "unable to find OpenGL 3+ Rendering Subsystem").
#   2. Major-version-matched to the loaded kernel module — pulled from
#      /proc/driver/nvidia/version (NVRM version line).
# See docs/lambda_headless_setup.md Step 1 for the full rationale + the
# 580.105.08-0lambda0.24.04.1 pin if you want an exact reproducible build.
if [ ! -r /proc/driver/nvidia/version ]; then
    echo "FATAL: /proc/driver/nvidia/version not readable — this box has" >&2
    echo "no NVIDIA driver loaded. Wrong instance type?" >&2
    exit 1
fi
NV_MAJOR="$(awk '/NVRM version:/ {for (i=1;i<=NF;i++) if ($i ~ /^[0-9]+\./) {split($i, a, "."); print a[1]; exit}}' /proc/driver/nvidia/version)"
if [ -z "$NV_MAJOR" ]; then
    echo "FATAL: could not parse NVIDIA driver major from /proc/driver/nvidia/version" >&2
    cat /proc/driver/nvidia/version >&2
    exit 1
fi
echo "Installing libnvidia-gl-${NV_MAJOR}-server (matched to loaded driver)..."
apt-get install -y -qq "libnvidia-gl-${NV_MAJOR}-server"
apt-mark hold "libnvidia-gl-${NV_MAJOR}-server"
# Sanity-check the EGL ICD config landed — setup_workers.sh bind-mounts
# this file into every worker; missing it = silent Mesa llvmpipe fallback.
if [ ! -f /usr/share/glvnd/egl_vendor.d/10_nvidia.json ]; then
    echo "FATAL: 10_nvidia.json missing after libnvidia-gl-${NV_MAJOR}-server install" >&2
    exit 1
fi

# --- 1b. System ROS Kilted (for the host-side image_relay) ---------------
# scripts/image_relay_node.py sources /opt/ros/kilted/setup.bash THEN
# activates ~/.venv (cv2/numpy come from the venv, rclpy+sensor_msgs from
# system ROS). The pixi env covers the policy but NOT the relay — relay
# would have to mix pixi + venv in one process which doesn't work.
# Add the ROS 2 apt repo + key, then the four packages the relay needs.
echo "Setting up ROS Kilted apt repo..."
curl -fsSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o /usr/share/keyrings/ros-archive-keyring.gpg
UBUNTU_CODENAME="$(. /etc/os-release && echo "$UBUNTU_CODENAME")"
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu ${UBUNTU_CODENAME} main" \
    > /etc/apt/sources.list.d/ros2.list
apt-get update -qq
apt-get install -y -qq \
    ros-kilted-ros-base \
    ros-kilted-rclpy \
    ros-kilted-sensor-msgs \
    ros-kilted-rmw-zenoh-cpp

# --- 2. Switch to ubuntu user for everything else -------------------------
# Heredoc with 'EOSU' (quoted) prevents the outer shell from expanding
# variables; this script body runs unchanged in the ubuntu shell.
sudo -u ubuntu -i bash <<EOSU
set -e
cd /home/ubuntu

# --- 2a. Wire the shared aic-data FS into /home/ubuntu/aic-data ----------
# Lambda mounts persistent filesystems at /lambda/nfs/<fs-name>/ automatically
# when --file-system is passed at launch. All collection scripts in this repo
# hardcode /home/ubuntu/aic-data, so we symlink the shared mount over there.
# Symlink (not bind mount) so it survives docker rm of any worker container
# that has it bind-mounted, and so the same path resolves inside the worker
# and on the host. Per-box scoping (box_<index>/) at the next directory level
# keeps multiple instances from colliding on a single shared FS.
if [ -d /lambda/nfs/aic-data ]; then
    if [ -d /home/ubuntu/aic-data ] && [ ! -L /home/ubuntu/aic-data ]; then
        # First-boot local dir is empty; remove only if so. Refuse to silently
        # discard any work that may have landed there on prior manual setup.
        rmdir /home/ubuntu/aic-data 2>/dev/null || \
            { echo "FATAL: /home/ubuntu/aic-data is a non-empty local dir; refusing to overwrite" >&2; exit 1; }
    fi
    ln -sfn /lambda/nfs/aic-data /home/ubuntu/aic-data
    echo "Mounted shared FS: /home/ubuntu/aic-data -> /lambda/nfs/aic-data"

    # Capacity pre-flight: surface free space so a near-full FS doesn't
    # silently fail \`StartRecording()\` mid-run after burning hours of compute.
    # Mcap median is ~1.3 GB/trial; 200 GB = ~150 trials of headroom. Soft
    # warning only — top-up runs on a near-full FS are a valid use case.
    free_kb="\$(df -P /lambda/nfs/aic-data | awk 'NR==2 {print \$4}')"
    free_gb=\$((free_kb / 1024 / 1024))
    echo "Shared FS free space: \${free_gb} GB on /lambda/nfs/aic-data"
    if [ "\$free_gb" -lt 200 ]; then
        echo "warning: shared FS has only \${free_gb} GB free — at ~1.3 GB/trial," >&2
        echo "         this leaves room for roughly \$((free_gb * 100 / 130)) trials." >&2
        echo "         Run \\\`du -sh /lambda/nfs/aic-data/box_*\\\` to find what's using it." >&2
    fi
else
    echo "warning: /lambda/nfs/aic-data not present; /home/ubuntu/aic-data will be" >&2
    echo "         per-box local SSD (0.5 TiB, ~390 trials at 1.3 GB/trial)." >&2
    echo "         If you passed --file-system aic-data, check the Lambda mount" >&2
    echo "         succeeded (otherwise box outputs won't be shared)." >&2
fi

# --- 3. Clone the aic repo -----------------------------------------------
if [ ! -d ws_aic/src/aic ]; then
    mkdir -p ws_aic/src
    git clone --depth 1 https://github.com/shu4dev/aic.git ws_aic/src/aic
fi

# --- 4. Install pixi + materialize env -----------------------------------
if [ ! -x \$HOME/.pixi/bin/pixi ]; then
    curl -fsSL https://pixi.sh/install.sh | bash
fi
export PATH="\$HOME/.pixi/bin:\$PATH"

cd \$HOME/ws_aic/src/aic
# This is the slow step — ~15-20 min. Provisions the entire ROS Kilted env
# + every local package (aic_model, aic_example_policies, …).
pixi self-update --version 0.67.2
pixi install

# --- 5. ~/.venv (Python 3.12) shared by every scripts/*.py invocation -----
# Built explicitly from python3.12 (not bare \`python3\`) so the venv interpreter
# is pinned even if Ubuntu later defaults python3 to a newer minor. Every
# shell wrapper in scripts/ that invokes a python script (run_parallel.sh,
# run_parallel_multi.sh, train_lambda.sh, ...) must \`source ~/.venv/bin/activate\`
# before exec'ing python — that contract is what lets us drop the prior
# pixi-vs-system auto-detect dance.
if [ ! -d \$HOME/.venv ]; then
    python3.12 -m venv \$HOME/.venv
fi
\$HOME/.venv/bin/pip install --quiet --upgrade pip
\$HOME/.venv/bin/pip install --quiet -r \$HOME/ws_aic/src/aic/scripts/requirements.txt

# --- 6. Pull the eval image + create distrobox workers --------------------
if [ -n "${GHCR_TOKEN}" ]; then
    echo "${GHCR_TOKEN}" | sudo docker login ghcr.io -u shu4dev --password-stdin
fi
sudo docker pull ghcr.io/shu4dev/aic-eval:v1

# Pre-warm just one worker container as a smoke test that distrobox +
# --nvidia bind-mounts are wired up. The actual collection runtime
# (scripts/run_parallel_multi.sh) re-invokes setup_workers.sh with whatever
# NUM_WORKERS the orchestrator picks, rm+recreating containers between
# chunks, so the count here only sets the floor — it does not constrain
# the fleet. Keep it at 1 so the bootstrap stays fast on every box and
# the smoke test exercises the same path as the default fleet config.
NUM_WORKERS=1 bash scripts/setup_workers.sh

EOSU

# --- 7. Sanity checks -----------------------------------------------------
# Each check must pass before we mark the box as ready. The fleet driver's
# `wait` polls for .bootstrap_done — if any prereq is half-installed we'd
# rather hang the wait than have `run` blow up an hour later in production.
echo "Running post-install sanity checks..."
fail=0

# nvidia-smi: NVIDIA userspace works.
if ! nvidia-smi >/dev/null 2>&1; then
    echo "  ✗ nvidia-smi failed"; fail=1
else echo "  ✓ nvidia-smi"; fi

# EGL ICD: 10_nvidia.json present + Mesa fallback also present (both
# needed by libglvnd; missing 10_nvidia.json = llvmpipe fallback).
if [ ! -f /usr/share/glvnd/egl_vendor.d/10_nvidia.json ]; then
    echo "  ✗ 10_nvidia.json missing"; fail=1
else echo "  ✓ 10_nvidia.json present"; fi

# Docker --gpus all works (proves NVIDIA Container Toolkit is wired up).
# Use the same cuda base image as the eval container's host CUDA needs.
if ! sudo docker run --rm --gpus all nvidia/cuda:12.4.0-base-ubuntu22.04 \
       nvidia-smi >/dev/null 2>&1; then
    echo "  ✗ docker --gpus all smoke failed"; fail=1
else echo "  ✓ docker --gpus all"; fi

# System ROS available (image_relay sources this).
if [ ! -f /opt/ros/kilted/setup.bash ]; then
    echo "  ✗ /opt/ros/kilted/setup.bash missing"; fail=1
else echo "  ✓ system ROS Kilted"; fi

# rclpy + sensor_msgs import under system ROS.
if ! sudo -u ubuntu bash -c \
        'source /opt/ros/kilted/setup.bash && python3 -c "import rclpy, sensor_msgs.msg"' \
        >/dev/null 2>&1; then
    echo "  ✗ rclpy/sensor_msgs import failed"; fail=1
else echo "  ✓ rclpy/sensor_msgs import"; fi

# Host venv on Python 3.12 with the scripts/requirements.txt set installed.
# cv2 + numpy are the image_relay imports; mcap + yaml cover the conversion
# and split/merge scripts that scripts/run_parallel_multi.sh invokes.
if ! sudo -u ubuntu bash -c \
        'source $HOME/.venv/bin/activate && python3 -c "import sys; assert sys.version_info[:2] == (3, 12), sys.version"' \
        >/dev/null 2>&1; then
    echo "  ✗ venv is not Python 3.12"; fail=1
else echo "  ✓ venv on Python 3.12"; fi
if ! sudo -u ubuntu bash -c \
        'source $HOME/.venv/bin/activate && python3 -c "import cv2, numpy, mcap, yaml"' \
        >/dev/null 2>&1; then
    echo "  ✗ venv requirements.txt import failed"; fail=1
else echo "  ✓ venv requirements.txt installed"; fi

# aic_eval image is local + worker containers exist.
if ! sudo docker image inspect ghcr.io/shu4dev/aic-eval:v1 >/dev/null 2>&1; then
    echo "  ✗ aic-eval image not pulled"; fail=1
else echo "  ✓ aic-eval image"; fi
if ! sudo docker container inspect aic_worker_0 >/dev/null 2>&1; then
    echo "  ✗ aic_worker_0 not created"; fail=1
else echo "  ✓ workers created"; fi

if [ "$fail" -ne 0 ]; then
    echo "=== Bootstrap FAILED at $(date -Iseconds) — not marking ready ==="
    exit 1
fi

# --- 8. Mark ready --------------------------------------------------------
# The fleet driver polls for this file via ssh to decide a box is workable.
sudo -u ubuntu touch /home/ubuntu/.bootstrap_done
echo "=== AIC bootstrap done at $(date -Iseconds) ==="
