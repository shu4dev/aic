# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Repository

This is the **AI for Industry Challenge (AIC)** toolkit — a ROS 2 Kilted workspace plus tooling for the cable-insertion challenge. The repo lives at `~/ws_aic/src/aic/`; results land in `~/aic_results/` (or `~/aic-data/` on the multi-worker setup). The remote of the fork in use here is `shu4dev/aic`; upstream is `intrinsic-dev/aic`.

## Environment & Build

Everything builds through **Pixi** (not raw colcon). The Pixi env is the one and only Python interpreter — `pixi shell` enters it; commands prefixed `(aic) $` in docs must run inside the env, `$` outside.

```bash
# Initial install (creates .pixi/ with every dep, including ROS Kilted via robostack)
cd ~/ws_aic/src/aic && pixi install

# Run anything in the pixi env without entering a sub-shell
pixi run ros2 <args...>

# After editing a local pixi-managed package (aic_model, aic_example_policies, …):
pixi reinstall <package>   # Pixi does NOT auto-pick up source changes
```

`pixi_env_setup.sh` activates with `RMW_IMPLEMENTATION=rmw_zenoh_cpp` and `ZENOH_CONFIG_OVERRIDE` defaulting to disabling shared-memory transport. Any caller that sets `ZENOH_CONFIG_OVERRIDE` *before* `pixi run` wins (the script uses `:-` defaults) — that's the mechanism `scripts/run_parallel.sh` uses to point each worker at a different router.

## Running the System

Standard 2-process flow:

```bash
# Terminal 1 (eval container) — brings up Gazebo + aic_engine + Zenoh router
distrobox enter -r aic_eval -- /entrypoint.sh ground_truth:=false start_aic_engine:=true

# Terminal 2 (host) — launches a policy that aic_engine waits for
pixi run ros2 run aic_model aic_model --ros-args \
    -p use_sim_time:=true \
    -p policy:=aic_example_policies.ros.WaveArm
```

`aic_engine` discovers `aic_model` within ~30 s, then drives trials via the YAML config in `aic_engine/config/` (default: container's bundled sample). It exits when the run completes if `shutdown_on_aic_engine_exit:=true`; otherwise it waits.

Custom configs are passed with `aic_engine_config_file:=/abs/path/to.yaml`. See `aic_engine/README.md` for the full parameter list and `aic_engine/config/sample_config.yaml` for trial structure.

## Tests

There is no top-level `pytest`/`colcon test` job; `.github/workflows/build.yml` only does a build with `skip-tests: true`. Manual lifecycle-node smoke tests live in `aic_model/test/` and are invoked directly:

```bash
pixi run python aic_model/test/cycle_through_lifecycle.py
pixi run python aic_model/test/create_and_cancel_task.py
```

## Lint & Style

CI in `.github/workflows/style.yml` enforces:
- **Python**: `black --check .` (no extra args). `isort` profile is `black` (`.isort.cfg`).
- **C/C++**: `clang-format` v19, Google style baseline (`.clang-format`), applied to `*.h,*.hpp,*.cc,*.cpp`.

There is no lint-runner wrapper — invoke `black .` / `clang-format -i` directly. `pyrightconfig.json` only type-checks `aic_utils/lerobot_robot_aic`.

## Architecture

### Two-component split

The challenge is intentionally divided so participants only ship one of the two halves:

1. **Evaluation component (provided)** — orchestrates trials, runs Gazebo, controls the robot, scores results. Distributed as the `aic_eval` Docker image; source for the same code is in this repo.
2. **Participant model (you ship)** — a single ROS 2 *lifecycle* node named `aic_model` that subscribes to fused `Observation` messages and publishes motion commands.

The two halves talk only over ROS 2 / Zenoh — never via shared memory. The eval container starts the Zenoh router; the host (or model container) connects to it.

### Engine state machine (`aic_engine/`)

`aic_engine` is the central orchestrator. Per trial it walks: **Model Ready → Endpoints Ready → Simulator Ready → Scoring Ready → Task Started → Task Completed**, then cleans up and advances. It validates that the participant node is an actual ROS 2 lifecycle node with proper services, starts unconfigured, doesn't move the robot until activated, and rejects goals before activation. Timeouts for every transition are exposed as ROS params.

### Policy framework (`aic_model/`, `aic_example_policies/`)

`aic_model` is a generic lifecycle node that **dynamically imports a Python module named by the `policy:` parameter** and instantiates a class with the same final dotted name. E.g. `policy:=aic_example_policies.ros.WaveArm` → imports `aic_example_policies.ros.WaveArm`, instantiates `WaveArm` (which subclasses `aic_model.policy.Policy`), and calls `insert_cable()` when the `InsertCable` action fires.

Policies receive three callbacks:
- `get_observation()` — most recent fused `Observation` (cameras × 3, joint states, wrist wrench, controller state, target/actual TCP pose)
- `move_robot(motion_update=..., joint_motion_update=...)` — Cartesian admittance or joint-space command (mutually exclusive)
- `send_feedback(str)` — publishes as `InsertCable` action feedback

Heavy ML imports (torch, lerobot, cv2) should be **deferred to `__init__`** of the policy class so `AicModel.__init__` returns fast enough for `aic_engine`'s discovery + configure handshake. See `aic_example_policies/ros/RunACT.py` for the pattern.

### Sensor/control plumbing

- `aic_adapter/` — fuses raw sensor topics into the 20 Hz `Observation` stream consumed by the policy.
- `aic_controller/` — Cartesian admittance + joint controllers built on `ros2_control`; consumes `MotionUpdate` / `JointMotionUpdate` from the model.
- `aic_interfaces/` — split into `aic_control_interfaces`, `aic_model_interfaces`, `aic_engine_interfaces`, `aic_task_interfaces` (one ROS msg/srv/action package per logical layer).
- `aic_bringup/` — `aic_gz_bringup.launch.py` is the master launch that the eval container's `/entrypoint.sh` ultimately runs.
- `aic_scoring/` — computes the three scoring tiers per trial; writes `score.yaml` to `$AIC_RESULTS_DIR`.

### Pixi multi-package layout (non-standard)

Unlike a stock colcon workspace, every locally-built ROS package is *also* declared as a path dependency in the root `pixi.toml` (`ros-kilted-aic-model = { path = "aic_model" }` …) **and** each package has its own `pixi.toml` listing its local sibling deps under `[package.host-dependencies]` and `[package.build-dependencies]`. Both lists must stay in sync — adding a new package or a new local dep requires editing both. Pixi prefixes `ros-<distro>-` and turns underscores into hyphens. See `docs/policy.md#dependency-management`.

## Parallel Evaluation (Lambda Cloud)

`scripts/run_parallel.sh` and `scripts/run_parallel_multi.sh` are the workflow for harvesting trials at scale on Lambda boxes. Mental model:

1. `scripts/setup_workers.sh` creates N persistent distrobox containers `aic_worker_0..aic_worker_{N-1}` and bind-mounts a per-worker `/entrypoint.sh` that pins Zenoh router to port `7447 + i`. Distrobox `--nvidia` is required (Ogre2 needs host NVIDIA GL libs bind-mounted — plain `docker run --gpus all` is not sufficient). See `docs/lambda_headless_setup.md` for the underlying NVIDIA driver/`libnvidia-gl-*-server` constraints.
2. `scripts/split_trials.py` splits a config into N round-robin parts under `~/aic-data/aic_split_configs/`.
3. `scripts/run_parallel_multi.sh` launches N copies of `run_parallel.sh` in parallel, each pinned to one part + worker container. Worker 0 uses `--network host` port 7447; workers ≥1 use bridge networking + `--publish 127.0.0.1:744{7+i}:7447`.
4. Each worker runs three concurrent stages: (T1) eval engine inside container, (T2) host-side `pixi run ros2 run aic_model …`, (T3) host `image_relay_node.py` (needs `$HOME/.venv` for cv2/numpy — sources ROS *first* then activates venv so venv `python3` wins).
5. `scripts/merge_scores.py` concatenates `aic_results_*/score.yaml` into one report.

Per-trial MCAP bags land in `$RESULTS/bag_trial_<id>_<ts>/` and feed `scripts/mcap_to_lerobot.py` (→ LeRobot v2.1 dataset) → `scripts/train_lambda.sh` (→ ACT training via `lerobot-train`, tuned for A100 40 GB).

## Submission

The submission target is a Docker image built from `docker/aic_model/Dockerfile` (template) or `docker/my_policy/Dockerfile`. It must:
- run a lifecycle node named `aic_model` on the eval Zenoh router (configured via `AIC_ROUTER_ADDR`)
- accept `RMW_IMPLEMENTATION=rmw_zenoh_cpp` and the `ZENOH_CONFIG_OVERRIDE` injected by the entrypoint
- respect the lifecycle protocol (`docs/challenge_rules.md`)

`docker/docker-compose.yaml` shows how the two containers wire together for local end-to-end testing.

## Useful File Pointers

- Policy base class & callbacks: `aic_model/aic_model/policy.py`
- Dynamic policy loader (the lifecycle node): `aic_model/aic_model/aic_model.py`
- Trial config schema (commented): `aic_engine/config/sample_config.yaml`
- Engine README (params, env vars, lifecycle states): `aic_engine/README.md`
- Lambda-specific NVIDIA / headless gotchas: `docs/lambda_headless_setup.md`
- Multi-worker design rationale (why bridge net, why per-worker entrypoint mounts): comment headers of `scripts/run_parallel.sh` and `scripts/setup_workers.sh`
