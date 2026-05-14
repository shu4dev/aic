#!/usr/bin/env python3
"""
Merge per-worker score.yaml files written by aic_engine into a single
combined report.

After scripts/run_parallel.sh finishes, each worker has written its own
$HOME/aic_results_<i>/score.yaml. This script collects them, concatenates
the per-trial breakdowns (trial ids are unique across workers because they
came from a single split), and writes one combined `merged_score.yaml`.

Usage:
    pixi run python scripts/merge_scores.py \\
        --results-root ~ \\
        --output ~/merged_score.yaml

    # The results-root contains the aic_results_*/score.yaml subdirs.
"""

import argparse
import glob
import pathlib
import sys
from typing import Any

import yaml


def _merge_breakdowns(files: list[pathlib.Path]) -> dict[str, Any]:
    """Concatenate the trial-keyed breakdowns from every worker.

    aic_engine writes a `breakdown:` key whose value is a dict mapping
    trial_id → {tier_1, tier_2, tier_3}. Some forks/versions flatten the
    structure (no `breakdown:` wrapper, trial ids at the top level), so we
    handle both shapes.
    """
    merged: dict[str, Any] = {}
    for f in files:
        data = yaml.safe_load(f.read_text()) or {}
        per_trial = data.get("breakdown", data) if isinstance(data, dict) else {}
        if not isinstance(per_trial, dict):
            print(f"warning: {f} has unexpected shape; skipping", file=sys.stderr)
            continue
        for trial_id, score in per_trial.items():
            if trial_id in merged:
                print(f"warning: duplicate trial '{trial_id}' (from {f}); "
                      f"keeping the first occurrence", file=sys.stderr)
                continue
            merged[trial_id] = score
    return merged


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[1])
    ap.add_argument(
        "--results-root", default=str(pathlib.Path.home()),
        help="parent dir containing aic_results_*/score.yaml subdirs "
             "(default: $HOME)",
    )
    ap.add_argument(
        "--pattern", default="aic_results_*/score.yaml",
        help="glob pattern relative to --results-root (default: "
             "aic_results_*/score.yaml)",
    )
    ap.add_argument(
        "--output", default="merged_score.yaml",
        help="path to write the merged YAML (default: ./merged_score.yaml)",
    )
    args = ap.parse_args()

    root = pathlib.Path(args.results_root).expanduser()
    files = sorted(pathlib.Path(p) for p in glob.glob(str(root / args.pattern)))
    if not files:
        print(f"no score.yaml files found under {root}/{args.pattern}",
              file=sys.stderr)
        return 1

    print(f"Merging {len(files)} file(s):")
    for f in files:
        print(f"  {f}")

    breakdown = _merge_breakdowns(files)
    out_path = pathlib.Path(args.output).expanduser()
    out_path.write_text(yaml.safe_dump(
        {"breakdown": breakdown, "trial_count": len(breakdown)},
        sort_keys=False,
    ))
    print(f"\nMerged {len(breakdown)} trial(s) → {out_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
