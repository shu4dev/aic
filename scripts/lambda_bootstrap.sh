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
#   distrobox        : container runner (NUM_WORKERS=4 setup_workers.sh)
#   git              : cloning the repo
#   python3-venv     : `python3 -m venv ~/.venv` needs this; Ubuntu Server
#                      doesn't ship it with the base python3 package
#   ffmpeg           : required downstream by scripts/mcap_to_lerobot.py
#                      for h264 encoding into the LeRobot dataset
#   jq, curl, rsync, ca-certificates : misc fleet utilities
apt-get update -qq
apt-get install -y -qq \
    distrobox \
    git \
    python3-venv \
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

# {{MOUNT_SHARED_FS}}
# If you've attached a Lambda persistent filesystem with the repo + pixi env
# pre-built, mount it here and `exit 0` to skip the rest. Lambda mounts
# attached filesystems at /lambda/nfs/<fs-name>/ automatically. The
# downside of an attached FS is concurrent writes — multiple boxes
# sharing one FS is fine for read-only configs but not for outputs.

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
pixi install

# --- 5. ~/.venv for image_relay's cv2/numpy needs -------------------------
if [ ! -d \$HOME/.venv ]; then
    python3 -m venv \$HOME/.venv
    \$HOME/.venv/bin/pip install --quiet --upgrade pip
    \$HOME/.venv/bin/pip install --quiet opencv-python-headless numpy mcap mcap-ros2-support
fi

# --- 6. Pull the eval image + create distrobox workers --------------------
if [ -n "${GHCR_TOKEN}" ]; then
    echo "${GHCR_TOKEN}" | sudo docker login ghcr.io -u shu4dev --password-stdin
fi
sudo docker pull ghcr.io/shu4dev/aic-eval:v1

# Match the original Lambda 1×A100 sizing assumed by run_parallel_multi.sh.
NUM_WORKERS=4 bash scripts/setup_workers.sh

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

# Host venv with cv2 + numpy (the image_relay imports).
if ! sudo -u ubuntu bash -c \
        'source $HOME/.venv/bin/activate && python3 -c "import cv2, numpy"' \
        >/dev/null 2>&1; then
    echo "  ✗ venv cv2/numpy import failed"; fail=1
else echo "  ✓ venv cv2/numpy"; fi

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
