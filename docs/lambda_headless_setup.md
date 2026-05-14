# Headless Setup on Lambda Cloud (Fresh Instance)

This guide walks you through configuring a **fresh Lambda Cloud GPU instance** end-to-end so that the AIC evaluation container (`aic_eval`) runs the Gazebo simulation headlessly with GPU-accelerated rendering on the NVIDIA driver.

Lambda's default driver packages ship the compute-only userspace (`libnvidia-compute-*`), and the kernel module they ship is from the `-server-open` channel. Two consequences:

1. The container's `gz-sim-sensors-system` (which uses Ogre2) can't find NVIDIA GL/EGL libs — gzserver segfaults.
2. If you naively install `libnvidia-gl-*` from the regular channel, you'll get a version mismatch with the loaded kernel module and NVIDIA EGL silently enumerates zero devices.

The steps below avoid both pitfalls.

> [!NOTE]
> Tested on Ubuntu 24.04 Lambda instances with an NVIDIA A10 GPU and driver `580.105.08` (kernel module `nvidia-dkms-580-server-open`). Adapt the `580` to whatever your instance's driver major matches.

---

## Step 0 — Confirm the loaded driver

Once the instance is up, check what's actually loaded:

```bash
cat /proc/driver/nvidia/version
nvidia-smi
dpkg -l | grep -E 'nvidia-(dkms|driver|kernel-common)'
```

Note three values: the **NVRM version** (loaded kernel module), the **driver version** reported by `nvidia-smi`, and the **channel** of the dkms package (`-server-open`, `-server`, or unversioned). All userspace libraries you install must match the kernel module **version** and **channel**.

For driver `580.105.08` with `nvidia-dkms-580-server-open`, the matching userspace package is `libnvidia-gl-580-server`.

---

## Step 1 — Install matching NVIDIA graphics libraries on the host

Distrobox bind-mounts the host's NVIDIA libraries into the container; if they're not installed on the host, the container can't render on the GPU.

```bash
# Install the GL/EGL/GLX libs from the channel that matches your kernel module
sudo apt update
sudo apt install -y libnvidia-gl-580-server

# Pin so a subsequent `apt upgrade` can't drift the version
sudo apt-mark hold libnvidia-gl-580-server

# Verify version matches /proc/driver/nvidia/version exactly
ls /usr/lib/x86_64-linux-gnu/libGLX_nvidia.so.*
ls /usr/lib/x86_64-linux-gnu/libEGL_nvidia.so.*
```

> [!WARNING]
> Do **not** install `libnvidia-gl-580` (without the `-server` suffix) on a `-server-open` kernel. It will pull a newer version and NVIDIA's EGL will refuse to enumerate devices because of the mismatch. The symptom is `Found Num EGL Devices: 0` in `~/.gz/rendering/ogre2.log` even though `nvidia-smi` works.

If `apt-cache madison libnvidia-gl-580-server` shows multiple versions, pin to the exact one matching your kernel:

```bash
sudo apt install -y libnvidia-gl-580-server=580.105.08-0lambda0.24.04.1
```

---

## Step 2 — Install Docker, NVIDIA Container Toolkit, Distrobox, Pixi

Follow the standard setup from [getting_started.md](./getting_started.md), abbreviated here:

```bash
# Docker
curl -fsSL https://get.docker.com | sh
sudo usermod -aG docker $USER
newgrp docker

# NVIDIA Container Toolkit
distribution=$(. /etc/os-release; echo $ID$VERSION_ID)
curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey | sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg
curl -fsSL https://nvidia.github.io/libnvidia-container/$distribution/libnvidia-container.list | \
  sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' | \
  sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list
sudo apt update && sudo apt install -y nvidia-container-toolkit
sudo nvidia-ctk runtime configure --runtime=docker
sudo systemctl restart docker

# Distrobox
sudo apt install -y distrobox

# Pixi
curl -fsSL https://pixi.sh/install.sh | sh
# restart your shell or `source ~/.bashrc`
```

Smoke-test GPU passthrough in plain Docker:

```bash
docker run --rm --gpus all nvidia/cuda:12.4.0-base-ubuntu22.04 nvidia-smi
```

You should see the A10 listed.

---

## Step 3 — Clone the workspace and `pixi install`

```bash
mkdir -p ~/ws_aic/src && cd ~/ws_aic/src
git clone https://github.com/intrinsic-dev/aic
cd aic
pixi install
```

---

## Step 4 — Create the distrobox container with **graphics** capabilities enabled

The critical flag here is `NVIDIA_DRIVER_CAPABILITIES=all` — without it, the container gets compute but not graphics.

```bash
export DBX_CONTAINER_MANAGER=docker

docker pull ghcr.io/intrinsic-dev/aic/aic_eval:latest

distrobox create -r --nvidia \
  --additional-flags "--env NVIDIA_DRIVER_CAPABILITIES=all --env NVIDIA_VISIBLE_DEVICES=all" \
  -i ghcr.io/intrinsic-dev/aic/aic_eval:latest aic_eval
```

---

## Step 5 — Prepare the container (root-side, one time)

The image runs as the `ubuntu` user without sudo, and is missing a couple of OpenGL diagnostic tools. Set both up via a root shell:

```bash
# from the host
docker exec -it -u root aic_eval bash
```

Inside the root shell:

```bash
apt update
apt install -y mesa-utils mesa-utils-extra      # provides glxinfo + eglinfo for diagnostics

# Grant ubuntu access to GPU/DRI device nodes
usermod -aG video,render ubuntu

# Optional: passwordless sudo from the normal distrobox shell
echo 'ubuntu ALL=(ALL) NOPASSWD:ALL' > /etc/sudoers.d/ubuntu
chmod 440 /etc/sudoers.d/ubuntu

exit
```

---

## Step 6 — Verify NVIDIA EGL works inside the container

```bash
distrobox enter -r aic_eval

# Confirm GPU compute is visible
nvidia-smi

# Confirm NVIDIA EGL libraries are present (bind-mounted from host)
find / -name 'libGLX_nvidia.so.*' 2>/dev/null
find / -name 'libEGL_nvidia.so.*' 2>/dev/null
ls /usr/share/glvnd/egl_vendor.d/
# expect: 10_nvidia.json AND 50_mesa.json

# Confirm NVIDIA EGL enumerates the GPU
eglinfo 2>&1 | grep -iE 'vendor|renderer|nvidia' | head -20
```

You **must** see a line like:

```
OpenGL renderer string: NVIDIA A10/PCIe/SSE2
OpenGL core profile version: 4.6.0 NVIDIA 580.105.08
```

If `eglinfo` shows only Mesa/llvmpipe entries, revisit Step 1 — the userspace lib version doesn't match the kernel module. Check `/proc/driver/nvidia/version` and `ls /usr/lib/x86_64-linux-gnu/libGLX_nvidia.so.*` and ensure both report identical version strings.

> [!NOTE]
> `glxinfo` will always report `llvmpipe` in a headless container with no real X server. That is expected — Gazebo uses EGL, not GLX. Don't be misled by glxinfo output.

---

## Step 7 — Launch the evaluation environment headlessly

Inside the container:

```bash
# Make sure no X display is set — Ogre2 must take the EGL path
unset DISPLAY

# Explicitly select the NVIDIA EGL ICD (belt and suspenders)
export __EGL_VENDOR_LIBRARY_FILENAMES=/usr/share/glvnd/egl_vendor.d/10_nvidia.json

/entrypoint.sh ground_truth:=false start_aic_engine:=true gazebo_gui:=false launch_rviz:=false
```

Key launch arguments for headless:

| Argument | Value | Reason |
|---|---|---|
| `gazebo_gui` | `false` | No display server available |
| `launch_rviz` | `false` | Same |
| `start_aic_engine` | `true` | Run the scoring engine |
| `ground_truth` | `false` | Match the evaluation configuration |

In a second terminal, confirm gzserver is actually using the GPU:

```bash
watch -n1 nvidia-smi
```

You should see a `component_container` / `ruby` / `gz sim server` process under **Processes** with non-zero GPU memory usage.

---

## Step 8 — Install ROS 2 Kilted with Zenoh on the host (recommended)

You can talk to the eval container's Zenoh router from **any** ROS 2 Kilted process — it does not have to live inside a pixi env or inside the container. Installing the ROS 2 packages natively on the Lambda host is faster than `pixi run` and lets you run host-side utilities (the policy, the image-relay node, `ros2 topic` debugging, etc.) without any container plumbing.

The package list at `packages.ros.org` already includes Kilted on Ubuntu 24.04 (Noble), so the install is two `apt` commands:

```bash
# Add the ROS 2 apt repo (skip if already added by a previous step)
sudo apt update && sudo apt install -y software-properties-common curl
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | \
    sudo tee /etc/apt/sources.list.d/ros2.list >/dev/null
sudo apt update

# The two packages you actually need:
sudo apt install -y ros-kilted-rmw-zenoh-cpp
# Pulls in ros-kilted-zenoh-cpp-vendor automatically.
```

Then in **every shell that needs to talk to the eval container**:

```bash
source /opt/ros/kilted/setup.bash
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
export ZENOH_CONFIG_OVERRIDE='transport/shared_memory/enabled=false'
```

Why `transport/shared_memory/enabled=false`: shared memory only works between processes in the same UTS/IPC namespace. The eval router lives in a container and the host process doesn't, so we force Zenoh to use TCP — the router is already listening on `tcp/[::]:7447` so this just falls through to the same socket distrobox exposes via host networking.

Put the three exports in `~/.bashrc` (or a sourced file like `~/.aic_env`) once you confirm they work — every subsequent SSH session inherits them.

### Quick sanity check
With the eval container running per Step 7, in a new SSH session:

```bash
source /opt/ros/kilted/setup.bash
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
export ZENOH_CONFIG_OVERRIDE='transport/shared_memory/enabled=false'

ros2 topic list                      # should show /joint_states, /center_camera/image, etc.
ros2 topic hz /clock                 # should report ~1000 Hz (sim clock)
```

If `ros2 topic list` is empty, the host can't reach the container's Zenoh router — check that `nc -z 127.0.0.1 7447` succeeds on the host and `distrobox create` was run without an explicit `--network` override.

---

## Step 9 — Run an example policy

Two paths, pick one:

**Native (preferred — needs Step 8 installed):**
```bash
source /opt/ros/kilted/setup.bash
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
export ZENOH_CONFIG_OVERRIDE='transport/shared_memory/enabled=false'

cd ~/ws_aic/src/aic
python3 -m aic_model.aic_model --ros-args \
    -p use_sim_time:=true \
    -p policy:=aic_example_policies.ros.WaveArm
# Or, if you've sourced the local colcon build:
#   ros2 run aic_model aic_model --ros-args -p use_sim_time:=true -p policy:=...
```

**Pixi (fallback — slower; useful if you don't want to apt-install anything):**
```bash
cd ~/ws_aic/src/aic
pixi run ros2 run aic_model aic_model --ros-args -p use_sim_time:=true -p policy:=aic_example_policies.ros.WaveArm
```

Either way, you should see the eval terminal log Trial 1/3, Trial 2/3, Trial 3/3 with scores, and results written to `~/aic_results/`.

### Host-side utilities you can now run without `pixi run`

The same native ROS 2 environment makes these one-liners work without pixi or `docker exec`:

```bash
# Downscale + JPEG-encode camera images to shrink MCAP bags ~160x
python3 scripts/image_relay_node.py

# Inspect a live topic
ros2 topic echo /scoring/insertion_event

# Record any topic to MCAP
ros2 bag record -s mcap /joint_states /fts_broadcaster/wrench
```

These all reach the eval container's Zenoh router transparently because `RMW_IMPLEMENTATION=rmw_zenoh_cpp` + `transport/shared_memory/enabled=false` is set in the shell.

---

## Troubleshooting Quick Reference

| Symptom | Likely Cause | Fix |
|---|---|---|
| `ros_gz_container ... exit code -11` and no other errors | Ogre2 can't find a GL context | Steps 1, 4, 7 — make sure `DISPLAY` is **unset** and NVIDIA EGL libs are loaded |
| `Found Num EGL Devices: 0` in `~/.gz/rendering/ogre2.log` | Driver/userspace version mismatch | Step 1 — install the `-server` channel package matching `/proc/driver/nvidia/version` |
| `GLXBadFBConfig` in launch output | `DISPLAY` is set, Ogre2 took the GLX path on Xvfb | `unset DISPLAY` (do **not** use Xvfb) |
| `Unable to load Ogre Plugin[...RenderSystem_GL3Plus]` | Same as above; check `~/.gz/rendering/ogre2.log` for the real reason | Inspect the log; it almost always names the actual cause |
| `libEGL warning: failed to open /dev/dri/card*` | Permission noise from Mesa probing DRI nodes | Step 5 — add user to `video` and `render` groups |
| `nvidia-smi` fails inside container | `--nvidia` flag missing on `distrobox create` | Step 4 — recreate the container |
| `eglinfo` shows only Mesa | NVIDIA GL libs not in container | Step 1 (host install) and Step 4 (`NVIDIA_DRIVER_CAPABILITIES=all`) |
| Host `ros2 topic list` is empty while container shows topics fine | Zenoh shared-memory enabled in host shell (default) | Step 8 — `export ZENOH_CONFIG_OVERRIDE='transport/shared_memory/enabled=false'` |
| Host `ros2 ...` hangs forever | `RMW_IMPLEMENTATION` not set or wrong | Step 8 — `export RMW_IMPLEMENTATION=rmw_zenoh_cpp` before sourcing |
| `ros2: command not found` after install | Forgot to source the overlay | `source /opt/ros/kilted/setup.bash` in this shell |

## Diagnostics Cheat Sheet

```bash
# On the host
cat /proc/driver/nvidia/version
ls /usr/lib/x86_64-linux-gnu/libGLX_nvidia.so.*
dpkg -l | grep -E 'nvidia-(dkms|gl)'

# Inside the container
nvidia-smi
find / -name 'libEGL_nvidia.so.*' 2>/dev/null
ls /usr/share/glvnd/egl_vendor.d/
eglinfo 2>&1 | grep -iE 'NVIDIA|renderer'
cat ~/.gz/rendering/ogre2.log | tail -60
```

Versions must match across the host's kernel module, the host's userspace libs, and the bind-mounted libs in the container — all three should report the same value (e.g., `580.105.08`).
