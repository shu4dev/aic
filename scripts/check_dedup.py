#!/usr/bin/env python3
"""
Check whether duplicate-publisher messages share exact header.stamps.

If aic_engine leaks publishers across trials (trial N has N copies of every
publisher), the converter's dedup-by-header.stamp only works when all copies
emit the same stamp at each tick. This script verifies that.

For each bag and key topic, prints:
    - total messages
    - unique header.stamps
    - multiplicity histogram (how many times each stamp appears)

Healthy pattern for a trial-N bag: unique ≈ total/N, histogram dominated by {N}.
Broken pattern: unique ≈ total — duplicates jittered, dedup won't catch them.

Usage:
    python3 check_dedup.py <bag_dir_or_results_root> [...]
"""

import argparse
import os
from collections import Counter
from multiprocessing import Pool
from pathlib import Path

from mcap_ros2.reader import read_ros2_messages


TOPICS = [
    "/joint_states",
    "/center_camera/image_small",
    "/fts_broadcaster/wrench",
    "/aic_controller/pose_commands",
]


def find_mcaps(roots):
    found = []
    for r in roots:
        p = Path(r)
        if p.is_file() and p.suffix == ".mcap":
            found.append(p)
        elif p.is_dir():
            found.extend(sorted(p.rglob("*.mcap")))
    return found


def stamp_stats(mcap_path: Path, topic: str):
    stamps = []
    for m in read_ros2_messages(str(mcap_path), topics=[topic]):
        h = m.ros_msg.header.stamp
        stamps.append((h.sec, h.nanosec))
    if not stamps:
        return None
    c = Counter(stamps)
    multiplicities = Counter(c.values())
    return {
        "total": len(stamps),
        "unique": len(c),
        "max_mult": max(c.values()),
        "mult_hist": multiplicities.most_common(5),
    }


def analyze(mcap_path: Path) -> str:
    lines = [f"\n{mcap_path.name}"]
    for topic in TOPICS:
        s = stamp_stats(mcap_path, topic)
        if s is None:
            lines.append(f"  {topic:<40} (no messages)")
            continue
        ratio = s["total"] / s["unique"] if s["unique"] else 0
        verdict = "OK" if ratio < 1.05 or s["max_mult"] == int(round(ratio)) else "JITTER"
        lines.append(f"  {topic:<40} total={s['total']:>7} unique={s['unique']:>7} "
                     f"ratio={ratio:>5.2f}× max_mult={s['max_mult']:>3}  [{verdict}]")
        lines.append(f"    multiplicity histogram: {s['mult_hist']}")
    return "\n".join(lines)


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("paths", nargs="+")
    p.add_argument("-j", "--workers", type=int, default=max(1, (os.cpu_count() or 2) // 2))
    args = p.parse_args()

    mcaps = find_mcaps(args.paths)
    if not mcaps:
        print("No .mcap files found.")
        return

    if args.workers == 1 or len(mcaps) == 1:
        for mp in mcaps:
            print(analyze(mp))
    else:
        with Pool(args.workers) as pool:
            for block in pool.imap_unordered(analyze, mcaps):
                print(block, flush=True)


if __name__ == "__main__":
    main()
