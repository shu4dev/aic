#!/usr/bin/env python3
"""
Report per-bag RTF (real-time factor) and key sim-rate sanity checks.

Usage:
    python3 check_rtf.py <bag_dir_or_results_root> [...]

Accepts either individual bag directories (containing one .mcap) or a parent
directory containing many bag_trial_*/ subdirectories.

RTF = sim_duration / wall_duration. sim_duration comes from header.stamp
spans on /joint_states (which ticks once per controller cycle = 500 sim-Hz);
wall_duration comes from MCAP log_time span. Camera sim-rate should be 20 Hz
on every bag — anything else means the publisher itself is misconfigured.
"""

import sys
from pathlib import Path

from mcap.reader import make_reader
from mcap_ros2.reader import read_ros2_messages


JS_TOPIC = "/joint_states"
CAM_TOPIC = "/center_camera/image_small"
EXPECTED_JS_SIM_HZ = 500.0
EXPECTED_CAM_SIM_HZ = 20.0


def header_stamps(mcap_path: Path, topic: str) -> list[float]:
    out = []
    for m in read_ros2_messages(str(mcap_path), topics=[topic]):
        h = m.ros_msg.header.stamp
        out.append(h.sec + h.nanosec * 1e-9)
    return out


def wall_duration_s(mcap_path: Path) -> float:
    with open(mcap_path, "rb") as f:
        stats = make_reader(f).get_summary().statistics
        return (stats.message_end_time - stats.message_start_time) / 1e9


def find_mcaps(roots: list[str]) -> list[Path]:
    found = []
    for r in roots:
        p = Path(r)
        if p.is_file() and p.suffix == ".mcap":
            found.append(p)
        elif p.is_dir():
            found.extend(sorted(p.rglob("*.mcap")))
    return found


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        sys.exit(1)

    mcaps = find_mcaps(sys.argv[1:])
    if not mcaps:
        print("No .mcap files found.")
        sys.exit(1)

    print(f"{'bag':<60} {'wall_s':>8} {'sim_s':>8} {'RTF':>6} "
          f"{'js_Hz':>8} {'cam_Hz':>8}")
    print("-" * 102)

    for mcap_path in mcaps:
        wall = wall_duration_s(mcap_path)
        js = header_stamps(mcap_path, JS_TOPIC)
        cam = header_stamps(mcap_path, CAM_TOPIC)

        if len(js) < 2 or wall <= 0:
            print(f"{mcap_path.name:<60} insufficient data")
            continue

        sim = js[-1] - js[0]
        rtf = sim / wall
        js_hz = (len(js) - 1) / sim
        cam_hz = (len(cam) - 1) / (cam[-1] - cam[0]) if len(cam) >= 2 else float("nan")

        js_flag = "" if abs(js_hz - EXPECTED_JS_SIM_HZ) < 5 else "  !js"
        cam_flag = "" if abs(cam_hz - EXPECTED_CAM_SIM_HZ) < 0.5 else "  !cam"

        print(f"{mcap_path.name:<60} {wall:8.2f} {sim:8.2f} {rtf:6.2f} "
              f"{js_hz:8.1f} {cam_hz:8.2f}{js_flag}{cam_flag}")


if __name__ == "__main__":
    main()
