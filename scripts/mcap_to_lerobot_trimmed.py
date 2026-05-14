#!/usr/bin/env python3
"""
mcap_to_lerobot with automatic episode trimming at the first /scoring/insertion_event.

Each bag's recording covers ~30 sim-seconds, but the actual cable-insertion
trajectory ends earlier — at the moment SCCheatCode publishes the first
/scoring/insertion_event. Everything after that is post-success motion
(retreat / regrasp / etc.) that we do NOT want ACT to imitate.

This wrapper:
  1. Scans every input bag for the earliest /scoring/insertion_event log_time.
  2. Writes a slice CSV with keep_side=before so the existing converter drops
     post-success frames.
  3. Bags with zero events (failed trials) are skipped by default.
  4. Invokes scripts/mcap_to_lerobot.py with --slice-csv pointing at the CSV.

All other arguments are passed through unchanged.

Usage:
    python3 mcap_to_lerobot_trimmed.py \\
        --mcap-dirs ~/bag/train_50/bag_trial_*/ \\
        --config-yaml aic_engine/config/train.yaml \\
        --output-dir ~/aic_lerobot_dataset \\
        --fps 20 --workers 20 --action-mode cartesian_pose
"""

import argparse
import csv
import os
import subprocess
import sys
from concurrent.futures import ProcessPoolExecutor, as_completed
from pathlib import Path
from typing import Optional

from mcap.reader import make_reader


EVENT_TOPIC = "/scoring/insertion_event"
CONVERTER_SCRIPT = Path(__file__).parent / "mcap_to_lerobot.py"


def find_first_event_log_time_ns(mcap_path: Path) -> Optional[int]:
    """Return log_time (ns) of the earliest /scoring/insertion_event, or None.

    Reads only event messages — fast even on multi-GiB bags.
    """
    earliest = None
    with open(mcap_path, "rb") as f:
        reader = make_reader(f)
        for schema, channel, message in reader.iter_messages(topics=[EVENT_TOPIC]):
            if earliest is None or message.log_time < earliest:
                earliest = message.log_time
    return earliest


def find_mcap_in_dir(bag_dir: Path) -> Optional[Path]:
    files = sorted(bag_dir.glob("*.mcap"))
    return files[0] if files else None


def scan_one(bag_dir_str: str) -> tuple[str, Optional[int], Optional[str]]:
    """Return (bag_dir, first_event_log_time_ns, drop_reason).

    drop_reason is None when the bag is keepable.
    """
    bd = Path(bag_dir_str.rstrip("/"))
    mp = find_mcap_in_dir(bd)
    if mp is None:
        return (str(bd), None, "no .mcap file")
    t = find_first_event_log_time_ns(mp)
    if t is None:
        return (str(bd), None, "no insertion event (failed trial)")
    return (str(bd), t, None)


def build_event_slice_csv(
    bag_dirs: list[str], csv_path: Path, workers: int, keep_failed: bool
) -> tuple[int, int]:
    """Scan bags in parallel and write a slice CSV.

    Returns (n_kept, n_dropped).
    """
    results = []
    if workers > 1 and len(bag_dirs) > 1:
        with ProcessPoolExecutor(max_workers=workers) as pool:
            futures = {pool.submit(scan_one, bd): bd for bd in bag_dirs}
            for fut in as_completed(futures):
                results.append(fut.result())
    else:
        results = [scan_one(bd) for bd in bag_dirs]

    results.sort(key=lambda r: r[0])
    n_kept = 0
    n_dropped = 0
    fieldnames = ["bag_path", "slice_log_time_ns", "keep_side", "drop_reason"]
    with open(csv_path, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=fieldnames)
        w.writeheader()
        for bag_dir, t, drop in results:
            row = {"bag_path": bag_dir, "slice_log_time_ns": "", "keep_side": "before",
                   "drop_reason": ""}
            if drop is not None:
                if keep_failed:
                    row["drop_reason"] = ""
                    print(f"  KEEP (no trim): {Path(bag_dir).name} — {drop}")
                    n_kept += 1
                else:
                    row["drop_reason"] = drop
                    print(f"  SKIP: {Path(bag_dir).name} — {drop}")
                    n_dropped += 1
            else:
                row["slice_log_time_ns"] = str(t)
                print(f"  TRIM: {Path(bag_dir).name} at log_time={t}")
                n_kept += 1
            w.writerow(row)
    return n_kept, n_dropped


def main():
    p = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    p.add_argument("--mcap-dirs", nargs="+", required=True)
    p.add_argument("--output-dir", required=True)
    p.add_argument("--scan-workers", type=int,
                   default=max(1, (os.cpu_count() or 2) // 2),
                   help="Parallel workers for the event-scan pass (default: cpu/2)")
    p.add_argument("--keep-failed", action="store_true",
                   help="Include trials with no insertion event (no slicing applied "
                        "to them). Off by default — failed trials are skipped.")
    p.add_argument("--slice-csv-out", default=None,
                   help="Where to write the generated slice CSV. Default: "
                        "<output_dir>/event_slices.csv")
    # Pass-through args (forwarded verbatim to mcap_to_lerobot.py).
    p.add_argument("--repo-id", default=None)
    p.add_argument("--config-yaml", default=None)
    p.add_argument("--tasks", nargs="+", default=None)
    p.add_argument("--scoring-dir", default=None)
    p.add_argument("--action-mode",
                   choices=["cartesian", "cartesian_pose", "joint"],
                   default=None)
    p.add_argument("--obs-state-mode",
                   choices=["full", "joints", "joints_port", "joints_pose",
                            "joints_pose_wrench"],
                   default=None)
    p.add_argument("--image-scale", type=float, default=None)
    p.add_argument("--fps", type=int, default=None)
    p.add_argument("--workers", type=int, default=None,
                   help="Converter workers (forwarded to mcap_to_lerobot.py)")
    p.add_argument("--val-fraction", type=float, default=None)
    args = p.parse_args()

    if not CONVERTER_SCRIPT.exists():
        print(f"Error: cannot find {CONVERTER_SCRIPT}")
        sys.exit(1)

    out_dir = Path(args.output_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    slice_csv = Path(args.slice_csv_out) if args.slice_csv_out \
        else out_dir / "event_slices.csv"

    print("=" * 70)
    print("  Step 1: scanning bags for /scoring/insertion_event")
    print("=" * 70)
    print(f"  Bags:        {len(args.mcap_dirs)}")
    print(f"  Scan workers: {args.scan_workers}")
    print(f"  CSV output:  {slice_csv}\n")

    n_kept, n_dropped = build_event_slice_csv(
        args.mcap_dirs, slice_csv, args.scan_workers, args.keep_failed,
    )
    print(f"\n  Kept: {n_kept}   Dropped: {n_dropped}\n")

    if n_kept == 0:
        print("Error: no bags survived the scan.")
        sys.exit(1)

    print("=" * 70)
    print("  Step 2: running mcap_to_lerobot.py with auto-generated slice CSV")
    print("=" * 70)

    cmd = [
        sys.executable, str(CONVERTER_SCRIPT),
        "--mcap-dirs", *args.mcap_dirs,
        "--output-dir", args.output_dir,
        "--slice-csv", str(slice_csv),
    ]
    pass_through = {
        "--repo-id": args.repo_id,
        "--config-yaml": args.config_yaml,
        "--scoring-dir": args.scoring_dir,
        "--action-mode": args.action_mode,
        "--obs-state-mode": args.obs_state_mode,
        "--image-scale": args.image_scale,
        "--fps": args.fps,
        "--workers": args.workers,
        "--val-fraction": args.val_fraction,
    }
    for flag, val in pass_through.items():
        if val is not None:
            cmd.extend([flag, str(val)])
    if args.tasks is not None:
        cmd.extend(["--tasks", *args.tasks])

    print("  cmd:", " ".join(cmd), "\n")
    rc = subprocess.call(cmd)
    sys.exit(rc)


if __name__ == "__main__":
    main()
