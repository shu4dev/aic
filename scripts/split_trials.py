#!/usr/bin/env python3
"""
Split an aic_engine config (e.g. aic_engine/config/train.yaml) into N chunks
so they can be evaluated in parallel by N independent eval containers.

Each output file is a complete, self-contained aic_engine config: same
`scoring:` and `task_board_limits:` blocks as the input, with a partition of
the `trials:` dict. Trials are distributed round-robin (i % N) so any
ordering bias in the source (difficulty, task type) is spread evenly.

Usage:
    pixi run python scripts/split_trials.py \\
        --input aic_engine/config/train.yaml \\
        --output-dir ~/aic_split_configs \\
        --num-parts 4

    # → ~/aic_split_configs/train_part_0.yaml ... train_part_3.yaml

Pair with scripts/run_parallel.sh, which expects the part files at
$HOME/aic_split_configs/train_part_<i>.yaml by default.
"""

import argparse
import pathlib
import sys

import yaml


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[1])
    ap.add_argument(
        "--input", required=True,
        help="path to the source aic_engine config (must contain a 'trials' key)",
    )
    ap.add_argument(
        "--output-dir", required=True,
        help="directory where train_part_<i>.yaml files will be written",
    )
    ap.add_argument(
        "-n", "--num-parts", type=int, required=True,
        help="number of chunks to produce",
    )
    ap.add_argument(
        "--prefix", default="train_part",
        help="output filename prefix (default: train_part)",
    )
    args = ap.parse_args()

    if args.num_parts < 1:
        print("--num-parts must be >= 1", file=sys.stderr)
        return 1

    src = pathlib.Path(args.input)
    cfg = yaml.safe_load(src.read_text())
    if "trials" not in cfg or not isinstance(cfg["trials"], dict):
        print(f"{src} has no 'trials' dict", file=sys.stderr)
        return 1

    trial_items = list(cfg["trials"].items())
    if not trial_items:
        print(f"{src} has zero trials", file=sys.stderr)
        return 1

    out_dir = pathlib.Path(args.output_dir).expanduser()
    out_dir.mkdir(parents=True, exist_ok=True)

    chunks: list[list] = [[] for _ in range(args.num_parts)]
    for i, item in enumerate(trial_items):
        chunks[i % args.num_parts].append(item)

    common = {k: v for k, v in cfg.items() if k != "trials"}
    for i, chunk in enumerate(chunks):
        part = dict(common)
        part["trials"] = dict(chunk)
        out_path = out_dir / f"{args.prefix}_{i}.yaml"
        out_path.write_text(yaml.safe_dump(part, sort_keys=False))
        print(f"{out_path} → {len(chunk)} trial(s)")

    total = sum(len(c) for c in chunks)
    print(f"Split {total} trial(s) from {src} into {args.num_parts} part(s) "
          f"in {out_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
