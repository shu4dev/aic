#!/usr/bin/env python3
"""
Print every /scoring/insertion_event message in each bag, with payload and
inter-event timing. Use this to figure out what insertion_event actually marks
(attempt start, attempt complete, success only, etc.) before writing
episode-split logic.

For each bag, prints one line per event:
    [idx] wall_t=<s> sim_t=<s>  dt_from_prev=<s>  payload=<string>

Usage:
    python3 inspect_events.py <bag_dir_or_results_root> [...]
"""

import argparse
import os
from multiprocessing import Pool
from pathlib import Path

from mcap.reader import make_reader
from mcap_ros2.decoder import DecoderFactory


EVENT_TOPIC = "/scoring/insertion_event"


def find_mcaps(roots):
    found = []
    for r in roots:
        p = Path(r)
        if p.is_file() and p.suffix == ".mcap":
            found.append(p)
        elif p.is_dir():
            found.extend(sorted(p.rglob("*.mcap")))
    return found


def analyze(mcap_path: Path) -> str:
    lines = [f"\n{mcap_path.name}"]
    prev_wall = None
    idx = 0
    with open(mcap_path, "rb") as f:
        reader = make_reader(f, decoder_factories=[DecoderFactory()])
        for schema, channel, message, decoded in reader.iter_decoded_messages(
            topics=[EVENT_TOPIC]
        ):
            wall_t = message.log_time / 1e9
            dt = (wall_t - prev_wall) if prev_wall is not None else 0.0
            payload = getattr(decoded, "data", repr(decoded))
            lines.append(f"  [{idx:>3}] wall_t={wall_t:.3f}  "
                         f"dt_from_prev={dt:>7.3f}s  payload={payload!r}")
            prev_wall = wall_t
            idx += 1
    if idx == 0:
        lines.append("  (no insertion_event messages — likely a failed trial)")
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
