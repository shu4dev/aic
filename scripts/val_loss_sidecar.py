#!/usr/bin/env python3
"""
val_loss_sidecar.py — Live validation loss tracking for lerobot-train.

Watches <train_output>/checkpoints/ for newly written step-N directories.
For each new checkpoint, loads the policy and runs a forward pass over
the held-out val episodes (defined in <dataset_root>/meta/val_episodes.json),
producing a single mean-loss number per checkpoint. Logs to:

  - <train_output>/val_loss.jsonl  (one JSON line per checkpoint)
  - WandB (attached to the same run as training, if --wandb-run-id given)

Designed to run concurrently with lerobot-train on the same GPU. Adds
~30 s to each checkpoint save (during which training pauses on GPU
contention), so net training slowdown is ~1 %.

Spawned automatically by scripts/train_lambda.sh when the dataset has a
meta/val_episodes.json (i.e. when the converter was run with
--val-fraction > 0). Safe to start manually too:

    python val_loss_sidecar.py <train_output_dir> [--wandb-run-id ID]

The sidecar is idempotent — restarting it skips checkpoints already in the
JSONL. Stop with Ctrl-C; it cleans up the wandb attachment on exit.
"""

import argparse
import json
import os
import sys
import time
import traceback
from pathlib import Path

import torch
from torch.utils.data import DataLoader

# lerobot reorganized its package layout between 0.4.x and 0.5.x. Try both.
try:
    from lerobot.common.datasets.lerobot_dataset import LeRobotDataset
    from lerobot.common.policies.act.modeling_act import ACTPolicy
except ImportError:
    try:
        from lerobot.datasets.lerobot_dataset import LeRobotDataset
        from lerobot.policies.act.modeling_act import ACTPolicy
    except ImportError as e:
        sys.stderr.write(
            f"Cannot import lerobot APIs: {e}\n"
            "Install lerobot in the active Python env (see requirements.txt).\n"
        )
        sys.exit(1)


def find_train_config(train_output: Path) -> Path | None:
    """Locate train_config.json. Older lerobot wrote it at the output root;
    0.5.x writes it inside each checkpoint dir. Return the first one found,
    preferring the output-root copy."""
    root_cfg = train_output / "train_config.json"
    if root_cfg.exists():
        return root_cfg
    ckpt_dir = train_output / "checkpoints"
    if ckpt_dir.exists():
        candidates = sorted(ckpt_dir.glob("*/pretrained_model/train_config.json"))
        if candidates:
            return candidates[0]
    return None


def load_train_config(train_output: Path) -> dict:
    cfg_path = find_train_config(train_output)
    if cfg_path is None:
        raise FileNotFoundError(
            f"No train_config.json under {train_output} (checked root and "
            f"checkpoints/*/pretrained_model/)"
        )
    return json.load(open(cfg_path))


def find_dataset_root(cfg: dict) -> Path:
    ds_cfg = cfg.get("dataset", {})
    root = ds_cfg.get("root") or ds_cfg.get("dataset_root")
    if not root:
        raise KeyError("No dataset.root in train_config.json")
    return Path(root)


def build_delta_timestamps(cfg: dict, dataset_root: Path) -> dict[str, list[float]]:
    """Replicate lerobot-train's dataset delta_timestamps so the val loader
    yields the same chunked tensors training sees. Without this, batch[ACTION]
    is shape [B, action_dim] and the ACT chunk loss errors out at broadcast."""
    info = json.load(open(dataset_root / "meta" / "info.json"))
    fps = info["fps"]
    policy_cfg = cfg.get("policy", {})
    chunk_size = int(policy_cfg.get("chunk_size", 100))
    # ACT consumes only the current observation; targets are the next
    # `chunk_size` actions starting at t.
    return {"action": [i / fps for i in range(chunk_size)]}


def load_val_episodes(dataset_root: Path) -> tuple[list[int], int]:
    val_path = dataset_root / "meta" / "val_episodes.json"
    if not val_path.exists():
        raise FileNotFoundError(
            f"No {val_path} — re-run converter with --val-fraction > 0"
        )
    data = json.load(open(val_path))
    return data["val_episodes"], data.get("n_total", -1)


def extract_loss(output) -> float:
    """Pull the scalar loss from whatever the policy returned."""
    if isinstance(output, dict):
        if "loss" in output:
            return float(output["loss"].item() if torch.is_tensor(output["loss"])
                         else output["loss"])
        for k in ("total_loss", "l1_loss", "action_loss"):
            if k in output:
                return float(output[k].item() if torch.is_tensor(output[k]) else output[k])
    if hasattr(output, "loss"):
        return float(output.loss.item() if torch.is_tensor(output.loss) else output.loss)
    if torch.is_tensor(output):
        return float(output.item())
    raise TypeError(f"Cannot extract loss from {type(output).__name__}: {output}")


def compute_val_loss(policy, val_loader, device) -> float:
    policy.eval()
    losses: list[float] = []
    with torch.no_grad():
        for batch in val_loader:
            batch = {
                k: (v.to(device, non_blocking=True) if torch.is_tensor(v) else v)
                for k, v in batch.items()
            }
            try:
                output = policy(batch)
                losses.append(extract_loss(output))
            except torch.cuda.OutOfMemoryError:
                # Free what we can and keep going. The sidecar competes for VRAM
                # with training; an occasional skipped batch is acceptable.
                torch.cuda.empty_cache()
                print("  WARN: CUDA OOM on val batch — skipped", flush=True)
                continue
    return sum(losses) / len(losses) if losses else float("nan")


def step_from_ckpt_name(name: str):
    """Checkpoint dirs are named by step number, possibly zero-padded."""
    try:
        return int(name)
    except ValueError:
        return None


def main():
    p = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    p.add_argument("train_output", type=Path,
                   help="Directory created by lerobot-train (contains checkpoints/)")
    p.add_argument("--num-workers", type=int, default=4)
    p.add_argument("--batch-size", type=int, default=32)
    p.add_argument("--poll-interval", type=float, default=10.0,
                   help="Seconds between checkpoint-directory scans")
    p.add_argument("--config-timeout", type=float, default=3600.0,
                   help="Seconds to wait for train_config.json before giving up "
                        "(dataset init can take 10-20 min with PyAV on large datasets)")
    p.add_argument("--wandb-run-id", default=None,
                   help="Attach to this wandb run (same one as training)")
    p.add_argument("--wandb-project", default=os.environ.get("WANDB_PROJECT", "aic-act"))
    p.add_argument("--wandb-entity", default=os.environ.get("WANDB_ENTITY"))
    args = p.parse_args()

    train_output = args.train_output.resolve()
    print(f"Sidecar watching: {train_output}", flush=True)

    # Wait for train_config.json. Older lerobot wrote it at the output root after
    # dataset init; 0.5.x only writes it inside each saved checkpoint dir, so we
    # accept either location. PyAV + a large dataset can take 10-20 min before
    # the first checkpoint, so default wait is 1 h (override via --config-timeout).
    deadline = time.time() + args.config_timeout
    waited = 0
    cfg_path = find_train_config(train_output)
    while cfg_path is None and time.time() < deadline:
        time.sleep(5)
        waited += 5
        if waited % 60 == 0:
            print(f"  still waiting for train_config.json "
                  f"({waited // 60} min elapsed, "
                  f"{int(args.config_timeout) // 60} min timeout). Searched "
                  f"{train_output}/train_config.json and "
                  f"{train_output}/checkpoints/*/pretrained_model/train_config.json",
                  flush=True)
        cfg_path = find_train_config(train_output)
    if cfg_path is None:
        sys.stderr.write(
            f"Timeout: no train_config.json under {train_output} after "
            f"{int(args.config_timeout) // 60} min — is training running? "
            f"Bump --config-timeout if dataset init is slow.\n"
        )
        sys.exit(1)
    print(f"Found config:  {cfg_path}", flush=True)

    cfg = load_train_config(train_output)
    dataset_root = find_dataset_root(cfg)
    val_episodes, n_total = load_val_episodes(dataset_root)
    delta_timestamps = build_delta_timestamps(cfg, dataset_root)
    print(f"Dataset:       {dataset_root}", flush=True)
    print(f"Val episodes:  {len(val_episodes)} of {n_total}", flush=True)
    print(f"Action chunk:  {len(delta_timestamps['action'])} steps "
          f"(delta_timestamps for 'action')", flush=True)

    device = "cuda" if torch.cuda.is_available() else "cpu"
    print(f"Device:        {device}", flush=True)

    print("Loading val dataset...", flush=True)
    val_ds = LeRobotDataset(
        repo_id=dataset_root.name,
        root=str(dataset_root),
        episodes=val_episodes,
        delta_timestamps=delta_timestamps,
    )
    val_loader = DataLoader(
        val_ds,
        batch_size=args.batch_size,
        num_workers=args.num_workers,
        shuffle=False,
        pin_memory=(device == "cuda"),
    )
    print(f"Val frames:    {len(val_ds)}", flush=True)

    wandb_run = None
    if args.wandb_run_id:
        try:
            import wandb
            kwargs = {"id": args.wandb_run_id, "resume": "must"}
            if args.wandb_project:
                kwargs["project"] = args.wandb_project
            if args.wandb_entity:
                kwargs["entity"] = args.wandb_entity
            wandb_run = wandb.init(**kwargs)
            print(f"WandB attached to run {args.wandb_run_id}", flush=True)
        except Exception as e:
            print(f"WandB attach failed (logging to JSONL only): {e}", flush=True)

    jsonl_path = train_output / "val_loss.jsonl"
    checkpoints_dir = train_output / "checkpoints"
    seen: set[str] = set()

    # Resume from prior JSONL entries — restarting the sidecar doesn't re-eval.
    if jsonl_path.exists():
        for line in open(jsonl_path):
            try:
                seen.add(str(json.loads(line)["step"]))
            except (json.JSONDecodeError, KeyError):
                pass
        if seen:
            print(f"Resumed: {len(seen)} checkpoints already in JSONL", flush=True)

    print(f"Output:        {jsonl_path}", flush=True)
    print("Watching for new checkpoints (Ctrl-C to stop)...", flush=True)

    try:
        while True:
            if checkpoints_dir.exists():
                for ckpt in sorted(checkpoints_dir.iterdir()):
                    if not ckpt.is_dir():
                        continue
                    step = step_from_ckpt_name(ckpt.name)
                    if step is None or str(step) in seen:
                        continue
                    model_dir = ckpt / "pretrained_model"
                    if not model_dir.exists():
                        continue  # checkpoint still being written by trainer

                    print(f"[step {step}] loading checkpoint...", flush=True)
                    try:
                        policy = ACTPolicy.from_pretrained(str(model_dir)).to(device)
                        t0 = time.time()
                        val_loss = compute_val_loss(policy, val_loader, device)
                        dt = time.time() - t0
                        del policy
                        if device == "cuda":
                            torch.cuda.empty_cache()

                        record = {
                            "step": step,
                            "val_loss": val_loss,
                            "n_val_episodes": len(val_episodes),
                            "n_val_frames": len(val_ds),
                            "elapsed_s": round(dt, 2),
                        }
                        with open(jsonl_path, "a") as f:
                            f.write(json.dumps(record) + "\n")

                        if wandb_run is not None:
                            wandb_run.log({"val/loss": val_loss}, step=step)

                        print(f"[step {step}] val_loss={val_loss:.5f}  ({dt:.1f}s)",
                              flush=True)
                        seen.add(str(step))
                    except Exception as e:
                        print(f"[step {step}] ERROR: {e}", flush=True)
                        traceback.print_exc()
                        # Mark as failed so we don't retry forever.
                        seen.add(str(step))
            time.sleep(args.poll_interval)
    except KeyboardInterrupt:
        print("\nSidecar stopped by user.", flush=True)
    finally:
        if wandb_run is not None:
            wandb_run.finish()


if __name__ == "__main__":
    main()
