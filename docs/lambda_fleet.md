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
> **Base image must be Lambda Stack 24.04 (Ubuntu 24.04 Noble).** ROS 2 Kilted Kaiju is published only for Noble — there are no `ros-kilted-*` packages for `jammy` or earlier. `Makefile.lambda` now **pins this explicitly** via `IMAGE_FAMILY=lambda-stack-24-04` (Lambda's API default has drifted to `jammy` on some region/type combinations, which is why we don't rely on it). `lambda_bootstrap.sh` also hard-fails in ~1 second on any non-Noble base, as a defense-in-depth check. Run `make -f Makefile.lambda images` if you ever need to discover the correct family slug for a new region.

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

Optionally, confirm the image family the fleet pins to is actually offered in your region:

```bash
make -f Makefile.lambda images
```

The default `IMAGE_FAMILY=lambda-stack-24-04` should appear. If you don't see it, pick the closest 24.04 family from the listing and pass `IMAGE_FAMILY=<slug>` to subsequent `make launch` calls.

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

For 1500 trials at the current per-trial success rate, fleet sizing is `BOXES` ≈ ceil(total_trials / trials_per_box_per_run). With the defaults (`WORKERS_PER_BOX=1`, `CHUNK_TRIALS=10`), expect each box to chew through ~25-35 trials per hour wall-clock once boot-up settles. 4 boxes covers ~100 trials/hour; 8 boxes covers ~200/hour. Pick `BOXES` based on how soon you need the data and how many parallel A100s your account is willing to charge for.

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

**Per-box topology (current defaults):** `WORKERS_PER_BOX=1`, `CHUNK_TRIALS=10`, `MAX_CHUNK_RETRIES=3`. Each box runs **one** worker container that processes its whole shard **sequentially**, in 10-trial chunks. The container is `docker rm -f`'d and recreated between chunks (idempotent recreate via `setup_workers.sh`) — this prevents the gz/Ogre/Zenoh heap-corruption observed after ~70 cumulative trials in a single long-lived container. A failed chunk is retried up to 3 times. If you'd rather trade reliability for wall-clock, bump `WORKERS_PER_BOX` (parallel workers per box, each independently chunked) or raise `CHUNK_TRIALS` for fewer recycles.

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

This rsyncs each box's **scoped subtree** (`/home/ubuntu/aic-data/box_<i>/`) to `./fleet_results/box_<i>/`. The per-box scoping is what lets every instance share the `aic-data` Lambda persistent filesystem without colliding — see the [Shared-FS scoping](#shared-fs-scoping) section below for the full layout. With the default `WORKERS_PER_BOX=1`, there's exactly one `aic_results_0/` per box; if you bumped it, you'll see `aic_results_{0..N-1}/`. Per-worker, chunking always produces `chunk_<c>/` subdirs (default `CHUNK_TRIALS=10`):

```
./fleet_results/box_0/
├── aic_results_0/
│   ├── chunk_0/
│   │   ├── bag_trial_*/         # MCAP recordings (10 trials at most)
│   │   ├── scoring.yaml         # per-trial tier scores for this chunk
│   │   ├── engine.log
│   │   └── policy.log
│   ├── chunk_1/
│   ├── chunk_2/
│   └── ...
└── merged_score.yaml            # this box's chunk-merge output
```

To merge across the whole fleet after `collect`:
```bash
~/.venv/bin/python3 scripts/merge_scores.py \
    --results-root ./fleet_results \
    --pattern 'box_*/aic_results_*/chunk_*/scoring.yaml' \
    --output ./fleet_results/merged_score.yaml
```

Each `bag_trial_*/*.mcap` is what you feed to `scripts/mcap_to_lerobot.py` to build a LeRobot dataset for training.

## Shared-FS scoping

The fleet attaches one Lambda persistent filesystem (`--file-system aic-data`, default name `aic-data`) to **every** box. Lambda mounts it at `/lambda/nfs/aic-data/` and the bootstrap symlinks that to `/home/ubuntu/aic-data` so the collection scripts (which all hardcode the latter path) write into the shared FS automatically.

To prevent collisions when N instances write concurrently, every output path is scoped by a `BOX_PREFIX` (= `BOX_INDEX` 0..N-1 passed by the fleet driver, falling back to `$(hostname)` for direct shell users). The on-FS layout while a run is in flight:

```
/home/ubuntu/aic-data/                       # symlinked to /lambda/nfs/aic-data/
├── aic_split_configs/
│   ├── box_0/                                # this box's input shard split N ways
│   ├── box_1/
│   └── ...
├── worker_entrypoints/
│   ├── box_0/entrypoint_0.sh                 # per-worker Zenoh-port template
│   ├── box_1/entrypoint_0.sh
│   └── ...
├── box_0/                                    # this box's results
│   ├── aic_results_0/chunk_0/...
│   ├── aic_results_0/chunk_1/...
│   └── merged_score.yaml
├── box_1/
└── ...
```

Direct consequence for `status` / `poll`: the fleet driver scopes `find` to `/home/ubuntu/aic-data/box_<idx>/` per ssh call, so each row of `make status` shows only that box's bag count even though every box can see every other box's outputs over NFS.

### Capacity planning

At a median of **1.3 GB / trial** (measured on the existing recorded trials in the repo — worst case ~3 GB for trials that run the full 180 s task timeout), the shared filesystem must be sized for the **total fleet output**, not per box:

| Target trials | Median footprint (1.3 GB) | Worst case (3 GB) |
|---|---|---|
| 500 | 0.65 TiB | 1.5 TiB |
| 1,000 | 1.3 TiB | 3 TiB |
| 2,000 | 2.6 TiB | 6 TiB |

The bootstrap surfaces free space at boot:
```
ssh ubuntu@<ip> 'grep "Shared FS free space" /var/log/aic-bootstrap.log'
# Shared FS free space: 5120 GB on /lambda/nfs/aic-data
```

It also prints a soft warning if free space at boot is under 200 GB (~150 trials of headroom). The warning is non-fatal — top-up runs on a near-full FS are a valid use case — but it shows up in the bootstrap log for the record.

While a run is in flight, `make status` and `make poll` both print the current shared-FS free GB so you can watch headroom shrink:
```
$ make -f Makefile.lambda status
  box 0 1.2.3.4:    47 bags  [running]
  box 1 1.2.3.5:    52 bags  [running]
  ...
shared FS free: 4870 GB  (~3746 trials @ 1.3 GB)
```

If you see free GB approaching zero mid-run, `make terminate` early — `make collect` then rsyncs whatever completed chunks landed before the fill-up. The bag writer in `aic_engine` returns false from `StartRecording()` on disk-full, so trials become "failed" rather than corrupt; you won't lose chunks that had already landed.

### Local SSD usage (the 0.5 TiB number)

The Lambda instance root disk (`/home/ubuntu/` and below, excluding the `/home/ubuntu/aic-data` symlink) only holds:

- OS + Lambda Stack: ~30 GB
- The aic-eval container image + worker container layers: ~5-10 GB
- The cloned repo + pixi env (`.pixi/envs/default`): ~10-15 GB
- The host-side `~/.venv` and ROS Kilted apt packages: ~5 GB

Total: ~50-60 GB used out of 500 GB. The local SSD is **not** a concern as long as `/home/ubuntu/aic-data` is the shared-FS symlink (verified by the bootstrap and by the "Mounted shared FS" line in `/var/log/aic-bootstrap.log`). If that symlink fails to land, the fallback warning at boot tells you, and you have ~390 trials of headroom on local SSD before fill — enough to abort and re-attach the FS.

To disable chunking and revert to one long-lived container per worker (with a single `score.yaml` at the top of each `aic_results_<i>/`), pass `CHUNK_TRIALS=0` to `make run`.

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
| `WORKERS_PER_BOX` | 1 | `NUM_WORKERS` passed to `run_parallel_multi.sh`. **Default 1 = fully serial per instance** (one container processes the whole shard, chunking and recycling between chunks). Bump for parallel workers per box, each independently chunked. |
| `CHUNK_TRIALS` | 10 | Per-worker trials before its container is `docker rm`'d + recreated. Mitigation for heap corruption seen past ~70 cumulative trials in one container. Pass 0 to disable chunking. |
| `MAX_CHUNK_RETRIES` | 3 | How many times `run_parallel_multi.sh` retries a failed chunk (recreate + rerun) before giving up. |
| `POLICY` | `aic_example_policies.ros.SCCheatCode` | Policy module the participant `aic_model` loads |
| `CONFIG` | `aic_engine/config/train.yaml` | Trial config to split + distribute |
| `INSTANCE_TYPE` | `gpu_1x_a100_sxm4` | Lambda instance type (see `make types`) |
| `REGION` | `us-west-2` | Lambda region |
| `IMAGE_FAMILY` | `lambda-stack-24-04` | Lambda image family slug. Pinned because the API default has drifted to jammy on some region/type combos. Run `make images` to list options. |
| `SSH_KEY` | `$(USER)` | Name of the SSH key already registered with Lambda |
| `RESULTS_DIR` | `./fleet_results` | Where `collect` lands data |

Example: cheaper smoke run on an A10:

```bash
make -f Makefile.lambda launch BOXES=1 INSTANCE_TYPE=gpu_1x_a10
```

Example: prioritize wall-clock over reliability — 4 parallel workers per box, larger chunks:

```bash
make -f Makefile.lambda run WORKERS_PER_BOX=4 CHUNK_TRIALS=20
```

---

## Cost expectations

These rules of thumb were taken before chunking + serial-per-box became the default; treat them as upper-bound throughput for `WORKERS_PER_BOX=4`. With the new defaults (`WORKERS_PER_BOX=1`, chunked) wall-clock per box is roughly **2-2.5× longer** because workers no longer run in parallel inside a box, but the per-trial success rate is higher (no heap-corruption tail). The crossover usually favors the new defaults — fewer wasted trials means fewer total boxes-hours to hit a target number of *successful* demonstrations.

| Fleet | Wall-clock for 1500 raw trials, `WORKERS_PER_BOX=4` | Lambda compute cost (A100 @ $1.29/hr) |
|---|---|---|
| 1 box × N=4 | ~9 hr | ~$12 |
| 4 boxes × N=4 | ~4-5 hr (collection) + ~25 min (bootstrap × 4 in parallel) | ~$26 |
| 8 boxes × N=4 | ~2-3 hr (collection) + ~25 min (bootstrap × 8 in parallel) | ~$30 |

The total compute-hours stays roughly flat across fleet sizes (more boxes = same work, less time). What scales is wall-clock. Pick `BOXES` based on how soon you need the data, and `WORKERS_PER_BOX` based on whether you're optimizing for reliability (1, the default) or pure throughput (3-4, at the cost of more chunk failures).

---

## Troubleshooting

### `make wait` times out

SSH in and check the bootstrap log:

```bash
ssh ubuntu@<box-ip>
sudo tail -200 /var/log/aic-bootstrap.log
```

The bootstrap script prints `✓` / `✗` per sanity check before deciding whether to `touch .bootstrap_done`. Common failure lines:

- `FATAL: this bootstrap requires Ubuntu 24.04 Noble` — should be rare now that `IMAGE_FAMILY=lambda-stack-24-04` is pinned at launch. If you see it, your `--image-family` override (or `IMAGE_FAMILY=` make-var) resolved to a non-Noble image. Run `make images` to list valid families and relaunch with the correct slug.
- `FATAL: /proc/driver/nvidia/version not readable` — instance type has no NVIDIA driver. Wrong type — use `gpu_1x_*`.
- `✗ docker --gpus all smoke failed` — NVIDIA Container Toolkit drift. Rare on Lambda Stack; usually means the image was changed manually.
- `✗ rclpy/sensor_msgs import failed` — system ROS 2 Kilted apt install failed (likely a transient apt mirror issue). Re-launch.
- `✗ venv requirements.txt import failed` — `~/.venv` is built from `scripts/requirements.txt` on Python 3.12. A package failed to install (usually transient pip/network); ssh in and rerun `~/.venv/bin/pip install -r ~/ws_aic/src/aic/scripts/requirements.txt` to see the real error.
- `Error response from daemon: Get "https://ghcr.io/v2/": denied` — the `GHCR_TOKEN` placeholder wasn't substituted, or your PAT lacks `read:packages`. Confirm `echo $GHCR_TOKEN` is non-empty *before* `make launch` (it's baked into the user_data client-side), regenerate the PAT if needed, then relaunch.

### Some boxes finish, others hang

Per-box `run_parallel_multi.sh` has its own retry loop around each chunk (default `MAX_CHUNK_RETRIES=3` — recreate the container + rerun the chunk's 10 trials). If a chunk burns through all three retries, it's skipped and the next chunk starts; the box's `nohup` process keeps going. If a box runs out of work entirely (all chunks attempted), `make status` shows it as `[exited]`. The other boxes are unaffected.

If a box is stuck without producing new bags for >15 min, SSH in and inspect `/home/ubuntu/run.log` (the stdout of `run_parallel_multi.sh`). Look for repeated `[worker N chunk M attempt 3] failed` — that's the retry budget being exhausted on a specific chunk. Raising `MAX_CHUNK_RETRIES` on the next run rarely helps unless the failure is transient; usually it indicates a problem with that specific trial config or an apt mirror flake during container recreate.

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

1. **`launch`** posts to `POST /instance-operations/launch` with `quantity=$BOXES`, `image={"family": "$IMAGE_FAMILY"}` (default `lambda-stack-24-04`), `file_system_names=["aic-data"]` (default), and `user_data` set to the contents of `lambda_bootstrap.sh` (with `{{GHCR_TOKEN}}` substituted from your env). Instance IDs persist to `~/.lambda_fleet/fleet.json` immediately.
2. **Lambda's cloud-init** runs `lambda_bootstrap.sh` as root on each box's first boot. The script: precondition-checks Noble, installs apt prereqs (including driver-matched `libnvidia-gl-${MAJOR}-server` and `python3.12` + `python3.12-venv`), sets up the ROS Kilted apt repo, clones the repo, **symlinks `/home/ubuntu/aic-data` → `/lambda/nfs/aic-data` so all collection writes land on the shared persistent FS**, runs `pixi install`, builds `~/.venv` from `scripts/requirements.txt` on Python 3.12 (shared by every `scripts/*.py` invocation), pulls the eval image, runs `setup_workers.sh` to pre-warm one distrobox worker as a smoke test, runs post-install sanity assertions, and only then `touch /home/ubuntu/.bootstrap_done`.
3. **`wait`** polls `GET /instances` for `status=active` + IP, then polls each box via `ssh test -f ~/.bootstrap_done`. A failed assertion in step 2 means the marker never appears and `wait` hits its timeout, which is intentional — better than `run` blowing up later on a half-working box.
4. **`run`** uses `~/.venv/bin/python3 scripts/split_trials.py` locally to shard the config, `scp`s one shard per box, then `ssh`s in with `BOX_INDEX={idx} CHUNK_TRIALS=10 MAX_CHUNK_RETRIES=3 NUM_WORKERS=1 nohup bash scripts/run_parallel_multi.sh > ~/run.log 2>&1 &`. `BOX_INDEX` is what scopes every output path under `aic-data/box_<idx>/` so concurrent boxes don't collide. `run_parallel_multi.sh` further chunks each worker's shard, recreating the container between chunks. The driver returns instantly; collection runs detached on the boxes.
5. **`status` / `poll`** ssh in and run `find /home/ubuntu/aic-data/box_<idx> -name 'bag_trial_*' | wc -l` plus `pgrep -f run_parallel_multi.sh` to know "X bags collected, alive/dead" for **just this box's slice** of the shared FS.
6. **`collect`** rsyncs each box's `/home/ubuntu/aic-data/box_<idx>/` to local `./fleet_results/box_<i>/` (per-box scope only — every box can see every other box's outputs over NFS, but we pull each subtree from its owning box to keep the layout one-to-one with the fleet state).
7. **`terminate`** posts to `POST /instance-operations/terminate` with the stored IDs, then deletes the state file.

The driver lives in `scripts/lambda_fleet.py` — read it if you want to change behavior or inspect the API responses directly.
