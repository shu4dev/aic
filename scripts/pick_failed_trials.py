#!/usr/bin/env python3
"""Pick trials from a scoring YAML whose tier_3 is not a clean success.

Usage:
    python pick_failed_trials.py <scoring.yaml>
    python pick_failed_trials.py <scoring.yaml> <config.yaml> <output.yaml>

The first form just lists the failed trials.
The second form additionally writes <output.yaml>: a copy of <config.yaml>
with its `trials:` mapping filtered down to only the failed ones.
"""
import argparse
import sys
import yaml

SUCCESS_SCORE = 75
SUCCESS_MSG = "Cable insertion successful."


def trial_sort_key(name: str) -> int:
    try:
        return int(name.split("_", 1)[1])
    except (IndexError, ValueError):
        return -1


def failed_trial_names(scoring: dict) -> list[str]:
    trials = {k: v for k, v in scoring.items() if k.startswith("trial_")}
    failed = []
    for name in sorted(trials, key=trial_sort_key):
        tier3 = (trials[name] or {}).get("tier_3") or {}
        if tier3.get("score") != SUCCESS_SCORE or tier3.get("message", "") != SUCCESS_MSG:
            failed.append(name)
    return failed


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("scoring_yaml", help="Path to *_scoring.yaml")
    parser.add_argument("config_yaml", nargs="?", help="Path to robot config YAML (e.g. train_50.yaml)")
    parser.add_argument("output_yaml", nargs="?", help="Path to write filtered config YAML")
    args = parser.parse_args()

    if (args.config_yaml is None) != (args.output_yaml is None):
        parser.error("config_yaml and output_yaml must be given together")

    with open(args.scoring_yaml) as f:
        scoring = yaml.safe_load(f)

    scoring_trials = {k: v for k, v in scoring.items() if k.startswith("trial_")}
    failed = failed_trial_names(scoring)

    for name in failed:
        tier3 = (scoring_trials[name] or {}).get("tier_3") or {}
        print(f"{name}: tier_3.score={tier3.get('score')}  message={tier3.get('message', '')!r}")
    print(f"\n{len(failed)} of {len(scoring_trials)} trials failed tier 3")

    if args.config_yaml is None:
        return 0

    with open(args.config_yaml) as f:
        config = yaml.safe_load(f)

    if "trials" not in config or not isinstance(config["trials"], dict):
        print(f"error: {args.config_yaml} has no top-level 'trials' mapping", file=sys.stderr)
        return 1

    original_trials = config["trials"]
    missing = [n for n in failed if n not in original_trials]
    if missing:
        print(f"warning: failed trials missing from config: {missing}", file=sys.stderr)

    filtered = {n: original_trials[n] for n in failed if n in original_trials}
    config["trials"] = filtered

    with open(args.output_yaml, "w") as f:
        yaml.safe_dump(config, f, sort_keys=False, default_flow_style=False)

    print(f"\nWrote {len(filtered)} failed trials to {args.output_yaml}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
