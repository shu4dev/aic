# Lambda Cloud Fleet Workflow

This guide walks through running a parallel AIC trial-collection job across **N Lambda Cloud GPU instances** end-to-end: launch, bootstrap, distribute, collect, terminate. It is the multi-host counterpart to [`lambda_headless_setup.md`](./lambda_headless_setup.md) (which covers a single hand-configured box).

When to use this guide instead of single-box:

- You need 1500+ trials and the single-box wall-clock (~20 hr) is too long.
- You're comfortable with a one-command workflow that handles provisioning, bootstrapping, distribution, and teardown.
- You're paying per-minute on Lambda and want a hard guarantee that boxes get terminated when collection is done.

The pipeline behind this guide lives in three files at the repo root:

| File | Role |
|---|---|
| `scripts/lambda_bootstrap.sh` | Cloud-init script that runs on every fresh instance — installs every prereq, builds the pixi env, pulls the eval image, creates 4 distrobox workers. |
| `scripts/lambda_fleet.py` | Python driver — talks to the Lambda Cloud API for `launch`, `wait`, `run`, `status`, `poll`, `collect`, `terminate`. |
| `Makefile.lambda` | One-command wrappers around the Python driver (`make -f Makefile.lambda all …`). |

> [!IMPORTANT]
> **Base image must be Lambda Stack 24.04 (Ubuntu 24.04 Noble).** ROS 2 Kilted Kaiju is published only for Noble — there are no `ros-kilted-*` packages for `jammy` or earlier. `lambda_bootstrap.sh` hard-fails in ~1 second on any other base, so you'll catch this immediately; just make sure the launch payload requests Lambda Stack 24.04 (the default for `gpu_1x_a100`).

---

## Prerequisites

Before you begin, you need:

1. **Lambda Cloud account** with billing set up (`https://cloud.lambdalabs.com/`).
2. **A Lambda API key** — create one at `https://cloud.lambdalabs.com/api-keys`. Export it:
   ```bash
   export LAMBDA_API_KEY='<your-key>'
   ```
3. **An SSH key registered with Lambda** — under Account → SSH keys. Note the name (e.g. `my-laptop`). The Python driver uses this name in launch payloads. Export it:
   ```bash
   export SSH_KEY='my-laptop'
   ```
   The matching private key must be in your local `~/.ssh/` so `ssh ubuntu@<box-ip>` works.
4. **A GitHub Container Registry PAT** with `read:packages` scope — the bootstrap uses it to pull `ghcr.io/shu4dev/aic-eval:v1` on each box. Export it:
   ```bash
   export GHCR_TOKEN='<your-PAT>'
   ```
5. **Local pixi env** — the Python driver runs `pixi run python scripts/split_trials.py` to shard the config. From the repo root:
   ```bash
   pixi install
   ```

---

## Step 1 — Sanity-check API access and instance availability

Before paying for anything, confirm your key works and the right instance type has capacity:

```bash
make -f Makefile.lambda types
```

You should see a list like:

```
gpu_1x_a100                    $1.29/hr  ['us-east-1', 'us-west-1']
gpu_1x_a10                     $0.75/hr  ['us-east-1']
...
```

If you see nothing, A100 capacity is exhausted in every region right now — wait or pick a different type (e.g. `gpu_1x_a10` for a cheaper smoke test). If you get an authentication error, re-check `$LAMBDA_API_KEY`.

---

## Step 2 — Single-box smoke test

**Do this first**, before spending on a 4- or 8-box run. It validates the bootstrap script end-to-end on a real Lambda instance for ~$1.30 of compute.

```bash
make -f Makefile.lambda launch BOXES=1
make -f Makefile.lambda wait      # ~25-30 min on a cold box
```

`wait` polls the API until the instance is `active` with an IP, then polls via SSH until `/home/ubuntu/.bootstrap_done` appears. The marker is only created if every sanity check passes (see [Troubleshooting](#troubleshooting) for the full list).

If `wait` times out, ssh into the box and inspect the bootstrap log:

```bash
ssh ubuntu@<box-ip>
sudo tail -200 /var/log/aic-bootstrap.log
```

The bootstrap prints `✓` / `✗` per sanity check at the end. The most common failure is the `FATAL: this bootstrap requires Ubuntu 24.04 Noble` line — means Lambda gave you a non-Noble image (rare; usually means you picked an image override that wasn't Lambda Stack 24.04).

**Always terminate after the smoke test:**

```bash
make -f Makefile.lambda terminate
```

---

## Step 3 — Pre-split your trial config (optional but recommended)

By default, `make run` calls `split_trials.py` automatically inside the driver. If you want to inspect the shards first (or rebuild a previous fleet's input), do it manually:

```bash
pixi run python scripts/split_trials.py \
    --input aic_engine/config/train.yaml \
    --output-dir /tmp/fleet_shards \
    --num-parts 4
```

You'll get `train_part_{0,1,2,3}.yaml` in `/tmp/fleet_shards/`. Each shard is independent — there's no shared state across shards at runtime.

---

## Step 4 — Launch the production fleet

For 1500 trials at ~50% success rate, 4 boxes × N=4 workers gets you 1500 raw trials in ~9 hours. 8 boxes halves that. Pick `BOXES` accordingly.

```bash
make -f Makefile.lambda launch BOXES=4
make -f Makefile.lambda wait
```

After `wait` returns, the fleet state is persisted at `~/.lambda_fleet/fleet.json`. Subsequent commands re-read it; no need to remember instance IDs yourself.

> [!CAUTION]
> If you `Ctrl-C` between `launch` and `terminate`, **instances keep running and billing keeps accruing**. Always run `make terminate` at the end. The state file is your record — even if you lose the shell, `make -f Makefile.lambda terminate` from any machine with the same `LAMBDA_API_KEY` and the state file will reach them.

---

## Step 5 — Kick off collection on every box

```bash
make -f Makefile.lambda run CONFIG=aic_engine/config/train.yaml
```

This splits the input N ways, `scp`s one shard per box, then `ssh`s in and starts `run_parallel_multi.sh` under `nohup` so the run survives the SSH disconnect. The Python driver returns immediately — collection runs on the boxes, not your laptop.

To verify it actually started:

```bash
make -f Makefile.lambda status
```

You should see something like:

```
  box 0 <ip>:    12 bags  [running]
  box 1 <ip>:     8 bags  [running]
  box 2 <ip>:    10 bags  [running]
  box 3 <ip>:    11 bags  [running]
```

---

## Step 6 — Wait for completion

`make poll` blocks until every box's `run_parallel_multi.sh` process exits. This is hours; use a terminal multiplexer (`tmux`, `screen`) or just leave it running.

```bash
make -f Makefile.lambda poll
```

`poll` re-prints `status`-style output every two minutes by default. When all boxes show `[exited]` it returns.

If you'd rather check in periodically yourself instead of blocking, use `status` from any other shell — it's stateless and doesn't interfere with the running collection.

---

## Step 7 — Collect results

```bash
make -f Makefile.lambda collect
```

This rsyncs every box's `/home/ubuntu/aic-data/` to `./fleet_results/box_<i>/` on your local machine. Per-box layout:

```
./fleet_results/box_0/
├── aic_results_0/
│   ├── chunk_*/                 # only if CHUNK_TRIALS > 0
│   │   ├── bag_trial_*/         # MCAP recordings
│   │   ├── scoring.yaml         # per-trial tier scores
│   │   ├── engine.log
│   │   └── policy.log
│   └── score.yaml               # final summary (if engine completed)
├── aic_results_1/
└── ...
```

Each `bag_trial_*/*.mcap` is what you feed to `scripts/mcap_to_lerobot.py` to build a LeRobot dataset for training.

---

## Step 8 — Terminate (always)

```bash
make -f Makefile.lambda terminate
```

This is idempotent — safe to run twice, safe to run if no fleet exists. After it returns, the state file is removed and the Lambda Cloud bill stops accruing for these instances.

> [!IMPORTANT]
> Verify in the Lambda Cloud web console (Instances tab) that no boxes are running. The Python driver also prints "terminated" on success; if you don't see that line, something went wrong and you need to terminate manually via the web UI.

---

## One-shot end-to-end

If you trust the pipeline (and have done the smoke test in Step 2), the `all` target chains steps 4-8:

```bash
make -f Makefile.lambda all BOXES=4 CONFIG=aic_engine/config/train.yaml
```

It does `launch → wait → run → poll → collect → terminate`, with `|| true` between `poll` and `collect → terminate` so a poll timeout doesn't skip the bill-stopping terminate.

---

## Knobs / overrides

All defaults are in `Makefile.lambda`; override on the command line.

| Variable | Default | Meaning |
|---|---|---|
| `BOXES` | 4 | Number of Lambda instances |
| `WORKERS_PER_BOX` | 4 | `NUM_WORKERS` passed to `run_parallel_multi.sh` (per-box parallel workers) |
| `POLICY` | `aic_example_policies.ros.SCCheatCode` | Policy module the participant `aic_model` loads |
| `CONFIG` | `aic_engine/config/train.yaml` | Trial config to split + distribute |
| `INSTANCE_TYPE` | `gpu_1x_a100` | Lambda instance type (see `make types`) |
| `REGION` | `us-east-1` | Lambda region |
| `SSH_KEY` | `$(USER)` | Name of the SSH key already registered with Lambda |
| `RESULTS_DIR` | `./fleet_results` | Where `collect` lands data |

Example: cheaper smoke run on an A10:

```bash
make -f Makefile.lambda launch BOXES=1 INSTANCE_TYPE=gpu_1x_a10
```

---

## Cost expectations

| Fleet | Wall-clock for 1500 trials | Lambda compute cost (A100 @ $1.29/hr) |
|---|---|---|
| 1 box × N=4 | ~9 hr | ~$12 |
| 4 boxes × N=4 | ~4-5 hr (collection) + ~25 min (bootstrap × 4 in parallel) | ~$26 |
| 8 boxes × N=4 | ~2-3 hr (collection) + ~25 min (bootstrap × 8 in parallel) | ~$30 |

The total compute-hours stays roughly flat across fleet sizes (more boxes = same work, less time). What scales is wall-clock. Pick `BOXES` based on how soon you need the data.

---

## Troubleshooting

### `make wait` times out

SSH in and check the bootstrap log:

```bash
ssh ubuntu@<box-ip>
sudo tail -200 /var/log/aic-bootstrap.log
```

The bootstrap script prints `✓` / `✗` per sanity check before deciding whether to `touch .bootstrap_done`. Common failure lines:

- `FATAL: this bootstrap requires Ubuntu 24.04 Noble` — wrong base image. Lambda Stack 24.04 is the only supported base. Terminate and re-launch.
- `FATAL: /proc/driver/nvidia/version not readable` — instance type has no NVIDIA driver. Wrong type — use `gpu_1x_*`.
- `✗ docker --gpus all smoke failed` — NVIDIA Container Toolkit drift. Rare on Lambda Stack; usually means the image was changed manually.
- `✗ rclpy/sensor_msgs import failed` — system ROS 2 Kilted apt install failed (likely a transient apt mirror issue). Re-launch.

### Some boxes finish, others hang

Per-box `run_parallel_multi.sh` has its own watchdog (`scripts/run_parallel.sh:425-440`). If one box's gz_server hits the heap-corruption bug we documented elsewhere, its worker auto-retries; if all 4 workers on one box go down, that box's `nohup` process exits and `make status` shows it as `[exited]` with whatever bag count it reached. The other boxes are unaffected.

If a box is stuck without producing new bags for >15 min, SSH in and inspect `/home/ubuntu/run.log` (the stdout of `run_parallel_multi.sh`).

### Successful trials are ~50% of total

That's the expected rate for this workload (per the chunked-restart analysis we did). Filter post-hoc:

```bash
# Trials with valid insertion event = successful demonstrations
find ./fleet_results -name 'bag_trial_*.mcap' -print0 | \
    xargs -0 -I{} sh -c \
    'python -c "from mcap_ros2.reader import read_ros2_messages; \
      assert any(read_ros2_messages(\"{}\", topics=[\"/scoring/insertion_event\"]))" \
      2>/dev/null && echo "{}"'
```

Feed only those to `scripts/mcap_to_lerobot.py`.

### Lambda billed me for an instance I forgot

If `~/.lambda_fleet/fleet.json` still exists, run `make terminate` — it'll catch any stragglers. If the file is gone, go to the Lambda Cloud web console (Instances tab) and terminate manually.

---

## What happens behind the scenes

For context — you don't need to touch any of this — here's the flow:

1. **`launch`** posts to `POST /instance-operations/launch` with `quantity=$BOXES` and `user_data` set to the contents of `lambda_bootstrap.sh` (with `{{GHCR_TOKEN}}` substituted from your env). Instance IDs persist to `~/.lambda_fleet/fleet.json` immediately.
2. **Lambda's cloud-init** runs `lambda_bootstrap.sh` as root on each box's first boot. The script: precondition-checks Noble, installs apt prereqs (including driver-matched `libnvidia-gl-${MAJOR}-server`), sets up the ROS Kilted apt repo, clones the repo, runs `pixi install`, builds `~/.venv`, pulls the eval image, runs `setup_workers.sh` to create 4 distrobox workers, runs 8 post-install sanity assertions, and only then `touch /home/ubuntu/.bootstrap_done`.
3. **`wait`** polls `GET /instances` for `status=active` + IP, then polls each box via `ssh test -f ~/.bootstrap_done`. A failed assertion in step 2 means the marker never appears and `wait` hits its timeout, which is intentional — better than `run` blowing up later on a half-working box.
4. **`run`** calls `pixi run python scripts/split_trials.py` locally to shard the config, `scp`s one shard per box, then `ssh`s in and does `nohup bash scripts/run_parallel_multi.sh > ~/run.log 2>&1 &`. The driver returns instantly; collection runs detached on the boxes.
5. **`status` / `poll`** ssh in and run `find /home/ubuntu/aic-data -name 'bag_trial_*' | wc -l` plus `pgrep -f run_parallel_multi.sh` to know "X bags collected, alive/dead".
6. **`collect`** rsyncs each box's `/home/ubuntu/aic-data/` to local `./fleet_results/box_<i>/`.
7. **`terminate`** posts to `POST /instance-operations/terminate` with the stored IDs, then deletes the state file.

If you want to read the actual API responses or change behavior, the entire driver is ~310 lines in `scripts/lambda_fleet.py`.
