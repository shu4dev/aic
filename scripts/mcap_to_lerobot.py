#!/usr/bin/env python3
"""
mcap_to_lerobot.py — Convert AIC ROS 2 .mcap bags to LeRobot v2.1 dataset.
See CONVERTER_README.md for full documentation.

Requires:
    pip install mcap mcap-ros2-support numpy opencv-python pyarrow pyyaml
    + ffmpeg on PATH (recommended)
"""

import argparse
import bisect
import json
import os
import random
import re
import subprocess
import sys
import time
from concurrent.futures import ProcessPoolExecutor, as_completed
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Optional

import cv2
import numpy as np
import pyarrow as pa
import pyarrow.parquet as pq
import yaml

from mcap.reader import make_reader
from mcap_ros2.decoder import DecoderFactory


# ═══════════════════════════════════════════════════════════════════════════════
#  CONFIGURATION
# ═══════════════════════════════════════════════════════════════════════════════

TOPIC_JOINT_STATES = "/joint_states"
TOPIC_CONTROLLER_STATE = "/aic_controller/controller_state"
TOPIC_POSE_COMMANDS = "/aic_controller/pose_commands"
TOPIC_JOINT_COMMANDS = "/aic_controller/joint_commands"
TOPIC_LEFT_IMAGE = "/left_camera/image"
TOPIC_CENTER_IMAGE = "/center_camera/image"
TOPIC_RIGHT_IMAGE = "/right_camera/image"
TOPIC_LEFT_IMAGE_SMALL = "/left_camera/image_small"
TOPIC_CENTER_IMAGE_SMALL = "/center_camera/image_small"
TOPIC_RIGHT_IMAGE_SMALL = "/right_camera/image_small"
TOPIC_FTS_WRENCH = "/fts_broadcaster/wrench"
TOPIC_TF = "/tf"
TOPIC_TF_STATIC = "/tf_static"
TOPIC_SCORING_TF = "/scoring/tf"

ARM_JOINT_NAMES = [
    "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
    "wrist_1_joint", "wrist_2_joint", "wrist_3_joint",
]

CAMERA_WIDTH, CAMERA_HEIGHT, CAMERA_FPS = 1152, 1024, 20

# Wrench smoothing — must match runtime defaults in
# aic_example_policies.utils.wrench_smoother. The same env vars (WRENCH_*)
# control both, so an experiment can override them in one place and have
# CheatCode collection, dataset build, and ACT eval all agree.
WRENCH_SMOOTH_WINDOW_S = float(os.environ.get("WRENCH_SMOOTH_WINDOW_S", "0.10"))
WRENCH_SMOOTH_MODE = os.environ.get("WRENCH_SMOOTH_MODE", "median")


def _aggregate_axis(values: list[float], mode: str) -> float:
    """Reduce a single axis to one scalar.

    Mirrors aic_example_policies.utils.wrench_smoother._aggregate. Kept inline
    here to avoid a Python-package dependency on the ROS-side aic_example_policies
    package from the offline conversion script.
    """
    if mode == "median":
        s = sorted(values)
        n = len(s)
        if n % 2:
            return s[n // 2]
        return 0.5 * (s[n // 2 - 1] + s[n // 2])
    if mode == "mean":
        return sum(values) / len(values)
    if mode == "max":
        return max(values)
    if mode == "min":
        return min(values)
    raise ValueError(f"unknown smoothing mode: {mode!r}")


def smooth_wrench_at(
    fts_ts: list[float],
    fts_tuples: list[tuple[float, float, float, float, float, float]],
    t: float,
    window_s: float = WRENCH_SMOOTH_WINDOW_S,
    mode: str = WRENCH_SMOOTH_MODE,
) -> list[float]:
    """Per-frame causal wrench smoothing for the dataset build.

    Selects samples whose timestamp ∈ (t - window_s, t] and per-axis aggregates
    them. Matches WrenchSmoother.smoothed() in the runtime path so the model
    sees the same distribution at training and eval. Falls back to nearest
    sample if the window is empty (covers episode-boundary frames).
    """
    if not fts_tuples:
        return [0.0] * 6
    lo = bisect.bisect_right(fts_ts, t - window_s)
    hi = bisect.bisect_right(fts_ts, t)
    window = fts_tuples[lo:hi]
    if not window:
        # Window empty — use the single nearest sample (matches the old
        # behavior at episode boundaries).
        idx = bisect.bisect_left(fts_ts, t)
        if idx == len(fts_ts):
            idx -= 1
        elif idx > 0 and (t - fts_ts[idx - 1]) <= (fts_ts[idx] - t):
            idx -= 1
        return list(fts_tuples[idx])
    return [_aggregate_axis([s[axis] for s in window], mode) for axis in range(6)]

# Raw full-res topics (old bags) and compressed downscaled topics (new bags)
CAMERA_TOPICS_RAW = {
    "left_camera": TOPIC_LEFT_IMAGE,
    "center_camera": TOPIC_CENTER_IMAGE,
    "right_camera": TOPIC_RIGHT_IMAGE,
}
CAMERA_TOPICS_SMALL = {
    "left_camera": TOPIC_LEFT_IMAGE_SMALL,
    "center_camera": TOPIC_CENTER_IMAGE_SMALL,
    "right_camera": TOPIC_RIGHT_IMAGE_SMALL,
}

# Scan both sets during Pass 1 so auto-detection works on old and new bags.
# /tf, /tf_static, /scoring/tf are read separately by resolve_port_entrance_baselink
# — do NOT include them here or streaming_convert_episode will deserialize every
# TF message (millions per bag).
NEEDED_TOPICS = {
    TOPIC_JOINT_STATES, TOPIC_CONTROLLER_STATE,
    TOPIC_POSE_COMMANDS, TOPIC_JOINT_COMMANDS, TOPIC_FTS_WRENCH,
    TOPIC_LEFT_IMAGE, TOPIC_CENTER_IMAGE, TOPIC_RIGHT_IMAGE,
    TOPIC_LEFT_IMAGE_SMALL, TOPIC_CENTER_IMAGE_SMALL, TOPIC_RIGHT_IMAGE_SMALL,
}


# ═══════════════════════════════════════════════════════════════════════════════
#  UTILITIES
# ═══════════════════════════════════════════════════════════════════════════════

def find_mcap_file(bag_path: str) -> str:
    p = Path(bag_path)
    if p.is_file() and p.suffix == ".mcap":
        return str(p)
    if p.is_dir():
        mcaps = sorted(p.glob("*.mcap"), key=lambda x: x.stat().st_size, reverse=True)
        if mcaps:
            return str(mcaps[0])
    raise FileNotFoundError(f"No .mcap file found in {bag_path}")


def has_ffmpeg() -> bool:
    try:
        subprocess.run(["ffmpeg", "-version"], capture_output=True, check=True)
        return True
    except (FileNotFoundError, subprocess.CalledProcessError):
        return False


_FFMPEG_AVAILABLE = None
def ffmpeg_available() -> bool:
    global _FFMPEG_AVAILABLE
    if _FFMPEG_AVAILABLE is None:
        _FFMPEG_AVAILABLE = has_ffmpeg()
    return _FFMPEG_AVAILABLE


def decode_ros2_image(msg) -> Optional[np.ndarray]:
    """Decode sensor_msgs/Image → BGR numpy array."""
    enc = msg.encoding
    h, w = msg.height, msg.width
    raw = bytes(msg.data)
    try:
        if enc in ("rgb8", "RGB8"):
            return cv2.cvtColor(np.frombuffer(raw, np.uint8).reshape(h, w, 3), cv2.COLOR_RGB2BGR)
        elif enc in ("bgr8", "BGR8"):
            return np.frombuffer(raw, np.uint8).reshape(h, w, 3).copy()
        elif enc in ("rgba8", "RGBA8"):
            return cv2.cvtColor(np.frombuffer(raw, np.uint8).reshape(h, w, 4), cv2.COLOR_RGBA2BGR)
        elif enc in ("bgra8", "BGRA8"):
            return cv2.cvtColor(np.frombuffer(raw, np.uint8).reshape(h, w, 4), cv2.COLOR_BGRA2BGR)
        elif enc in ("mono8", "MONO8"):
            return cv2.cvtColor(np.frombuffer(raw, np.uint8).reshape(h, w), cv2.COLOR_GRAY2BGR)
        else:
            return np.frombuffer(raw, np.uint8).reshape(h, w, 3).copy()
    except (ValueError, cv2.error):
        return None


def decode_compressed_image(msg) -> Optional[np.ndarray]:
    """Decode sensor_msgs/CompressedImage (JPEG) → BGR numpy array."""
    try:
        buf = np.frombuffer(bytes(msg.data), np.uint8)
        return cv2.imdecode(buf, cv2.IMREAD_COLOR)
    except Exception:
        return None


# ═══════════════════════════════════════════════════════════════════════════════
#  PASS 1: LIGHTWEIGHT TIMESTAMP INDEX (fast scan — no image decoding)
# ═══════════════════════════════════════════════════════════════════════════════

@dataclass
class TimestampIndex:
    """Timestamps (in seconds) per topic, built without decoding heavy messages.

    `topics` carries `log_time` (wall-clock recorder time) for every topic
    — used downstream by streaming_convert_episode for nearest-neighbor
    matching of action / proprio messages, where it doesn't matter much
    because those streams are dense (~1 kHz+ sim Hz).

    `camera_pairs` carries (header.stamp, log_time) tuples for camera
    topics only. The camera is the cadence-determining stream:
    compute_reference_timestamps subsamples on the sim-time
    `header.stamp` axis (collapsing the publisher's duplicate-burst
    artefact, which produces multiple wire messages with the same
    `header.stamp`) and returns the corresponding `log_time` values so
    streaming_convert_episode's existing log_time-based matching keeps
    working unchanged.
    """
    topics: dict[str, list[float]] = field(default_factory=lambda: {t: [] for t in NEEDED_TOPICS})
    camera_pairs: dict[str, list[tuple[float, float]]] = field(default_factory=dict)
    total_messages: int = 0
    compressed: bool = False  # True when camera topics are CompressedImage (image_small)


# Camera message wire layout (sensor_msgs/Image and sensor_msgs/CompressedImage):
#   4 bytes CDR header
#   std_msgs/Header { Time stamp { int32 sec, uint32 nanosec }, string frame_id }
# We only read the first 12 bytes after the CDR header — sec + nanosec — to
# avoid decoding the full message payload during the lightweight first pass.
def _parse_header_stamp_seconds(data: bytes) -> float:
    sec = int.from_bytes(data[4:8], "little", signed=True)
    nsec = int.from_bytes(data[8:12], "little")
    return sec + nsec * 1e-9


def build_timestamp_index(mcap_path: str) -> TimestampIndex:
    """
    Fast first pass: only reads message headers to collect timestamps.
    Uses iter_messages (not iter_decoded_messages) so images are NOT decoded.
    Auto-detects whether bags contain raw Image or CompressedImage camera topics.

    For camera topics, also extracts `header.stamp` (sim time, parsed from
    the first 12 bytes of payload) and pairs it with `log_time`.
    compute_reference_timestamps then subsamples on the sim-time axis but
    returns log_time values so streaming-pass matching is unchanged.
    """
    idx = TimestampIndex()
    camera_topics = set(CAMERA_TOPICS_SMALL.values()) | set(CAMERA_TOPICS_RAW.values())
    for cam_topic in camera_topics:
        idx.camera_pairs[cam_topic] = []

    with open(mcap_path, "rb") as f:
        reader = make_reader(f)
        for schema, channel, message in reader.iter_messages(topics=NEEDED_TOPICS):
            log_t = message.log_time * 1e-9
            if channel.topic in idx.topics:
                idx.topics[channel.topic].append(log_t)
                idx.total_messages += 1
            if channel.topic in camera_topics:
                hs = _parse_header_stamp_seconds(message.data)
                idx.camera_pairs[channel.topic].append((hs, log_t))

    for topic in idx.topics:
        idx.topics[topic].sort()
    # Sort camera pairs by header.stamp (then log_time as tiebreaker for
    # duplicate-bursts so we deterministically pick the first wire copy).
    for topic in idx.camera_pairs:
        idx.camera_pairs[topic].sort()

    # Auto-detect: prefer image_small (CompressedImage) if present
    idx.compressed = any(
        len(idx.topics[t]) > 0 for t in CAMERA_TOPICS_SMALL.values()
    )

    return idx


def compute_reference_timestamps(idx: TimestampIndex, target_fps: int) -> list[float]:
    """
    Pick reference timestamps from the center camera (or densest camera),
    subsampled to target_fps. Returns sim-time `header.stamp` values
    (the streaming pass also matches by header.stamp, keeping the whole
    pipeline in sim time).

    Subsampling on sim-time collapses the publisher's duplicate-burst
    artefact (multiple wire messages with identical `header.stamp`) into
    a single ref entry: the first appearance wins, the rest fail the
    target_dt threshold. The resulting cadence is sim-time-uniform
    regardless of host load or RTF.
    """
    cam_topics = CAMERA_TOPICS_SMALL if idx.compressed else CAMERA_TOPICS_RAW
    ref_topic = cam_topics["center_camera"]
    pairs = idx.camera_pairs.get(ref_topic, [])

    # Fallback to whichever camera has the most frames
    if not pairs:
        for cam_topic in [cam_topics["left_camera"], cam_topics["right_camera"]]:
            cand = idx.camera_pairs.get(cam_topic, [])
            if len(cand) > len(pairs):
                pairs = cand

    if not pairs:
        return []

    # camera_pairs are sorted by (header.stamp, log_time); take the
    # header.stamp axis for cadence subsampling.
    sim_ts = [hs for (hs, _) in pairs]

    target_dt = 1.0 / target_fps
    subsampled = [sim_ts[0]]
    for t in sim_ts[1:]:
        if t - subsampled[-1] >= target_dt * 0.9:
            subsampled.append(t)

    return subsampled


# ═══════════════════════════════════════════════════════════════════════════════
#  PASS 2: STREAMING DECODE + WRITE
# ═══════════════════════════════════════════════════════════════════════════════

def binary_search_nearest(sorted_ts: list[float], target: float) -> int:
    """Return index of the timestamp nearest to target."""
    if not sorted_ts:
        return -1
    lo, hi = 0, len(sorted_ts) - 1
    while lo < hi:
        mid = (lo + hi) // 2
        if sorted_ts[mid] < target:
            lo = mid + 1
        else:
            hi = mid
    best = lo
    if lo > 0 and abs(sorted_ts[lo - 1] - target) < abs(sorted_ts[lo] - target):
        best = lo - 1
    return best


def _q_to_R_np(q):
    """Quaternion (w, x, y, z) → 3×3 rotation matrix."""
    w, x, y, z = q
    return np.array([
        [1 - 2*(y*y + z*z),     2*(x*y - z*w),     2*(x*z + y*w)],
        [    2*(x*y + z*w), 1 - 2*(x*x + z*z),     2*(y*z - x*w)],
        [    2*(x*z - y*w),     2*(y*z + x*w), 1 - 2*(x*x + y*y)],
    ])


def _q_mul(q1, q2):
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    return np.array([
        w1*w2 - x1*x2 - y1*y2 - z1*z2,
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2,
    ])


def _compose_pose(parent_t, parent_q, child_t, child_q):
    """parent ∘ child (returns world-frame pose of child given parent's world pose)."""
    R = _q_to_R_np(parent_q)
    t_out = parent_t + R @ child_t
    q_out = _q_mul(parent_q, child_q)
    return t_out, q_out


def resolve_port_entrance_baselink(mcap_path: str) -> Optional[list]:
    """Walk /scoring/tf + /tf + /tf_static to find the port_entrance in base_link.
    Returns [x, y, z] or None if frames missing.

    The port_entrance is a STATIC frame (task_board is fixed during a trial),
    so we collect the latest transform for each (parent, child) pair across
    the bag and compose.
    """
    try:
        from mcap_ros2.reader import read_ros2_messages
    except Exception:
        return None

    tf_latest: dict[tuple[str, str], tuple[np.ndarray, np.ndarray]] = {}

    # /tf_static is tiny — reads in ~0s. Read fully.
    for m in read_ros2_messages(mcap_path, topics=[TOPIC_TF_STATIC]):
        msg = m.ros_msg
        if not hasattr(msg, 'transforms'):
            continue
        for tr in msg.transforms:
            key = (tr.header.frame_id, tr.child_frame_id)
            t = np.array([tr.transform.translation.x,
                          tr.transform.translation.y,
                          tr.transform.translation.z])
            q = np.array([tr.transform.rotation.w,
                          tr.transform.rotation.x,
                          tr.transform.rotation.y,
                          tr.transform.rotation.z])
            tf_latest[key] = (t, q)

    # /scoring/tf has the static task_board-related frames we need. In
    # collection bags, `aic_world->task_board` and the port_entrance frames are
    # first published ~2.4s in (not at t=0 like cable_0). Reading from t=0 is
    # slow (~9s) because /scoring/tf is dense with cable_0 links for the first
    # 2.4s. Skip ahead: find first /scoring/tf ts, then start reading 2s in
    # and break as soon as we have the task_board + module + entrance frames.
    first_ts = None
    for m in read_ros2_messages(mcap_path, topics=[TOPIC_SCORING_TF]):
        first_ts = m.log_time_ns
        break
    kwargs = {"topics": [TOPIC_SCORING_TF]}
    if first_ts is not None:
        kwargs["start_time"] = first_ts + 2_000_000_000  # skip cable-only prefix
        kwargs["end_time"] = first_ts + 5_000_000_000
    got_taskboard = False
    got_module = False
    got_entrance = False
    for m in read_ros2_messages(mcap_path, **kwargs):
        msg = m.ros_msg
        if not hasattr(msg, 'transforms'):
            continue
        for tr in msg.transforms:
            key = (tr.header.frame_id, tr.child_frame_id)
            # Don't overwrite /tf_static entries (they're authoritative for static frames)
            if key in tf_latest:
                continue
            t = np.array([tr.transform.translation.x,
                          tr.transform.translation.y,
                          tr.transform.translation.z])
            q = np.array([tr.transform.rotation.w,
                          tr.transform.rotation.x,
                          tr.transform.rotation.y,
                          tr.transform.rotation.z])
            tf_latest[key] = (t, q)
            if key[0] == "aic_world" and key[1] == "task_board":
                got_taskboard = True
            elif key[0] == "task_board" and key[1].startswith("task_board/") \
                    and key[1] != "task_board/task_board_base_link":
                got_module = True
            elif key[1].endswith("_link_entrance"):
                got_entrance = True
        if got_taskboard and got_module and got_entrance:
            break

    def lookup(parent, child):
        return tf_latest.get((parent, child))

    # In this sim, TF tree is:
    #   world -> aic_world -> task_board -> task_board/<module> -> task_board/<module>/<port>_link_entrance
    #   world -> tabletop -> base_link (robot)
    # So we ground both poses in 'world'.
    aic_in_world = lookup("world", "aic_world")
    tabletop_in_world = lookup("world", "tabletop")
    base_in_tabletop = lookup("tabletop", "base_link")
    if aic_in_world is None or tabletop_in_world is None or base_in_tabletop is None:
        return None
    base_in_world = _compose_pose(*tabletop_in_world, *base_in_tabletop)

    # port_entrance_in_world = aic_in_world ∘ task_board_in_aic ∘ module_in_tb ∘ entrance_in_module
    tb_in_aic = lookup("aic_world", "task_board")
    if tb_in_aic is None:
        return None
    tb_in_world = _compose_pose(*aic_in_world, *tb_in_aic)

    port_in_world = None
    # Module children are named e.g. "task_board/nic_card_mount_3", port_entrance
    # children are named e.g. "task_board/nic_card_mount_3/sfp_port_0_link_entrance".
    for (parent, child), (t, q) in tf_latest.items():
        if parent.startswith("task_board/") and child.endswith("_link_entrance"):
            # parent IS the full module path (e.g. "task_board/nic_card_mount_3")
            module_tf = lookup("task_board", parent)
            if module_tf is None:
                continue
            module_in_world = _compose_pose(*tb_in_world, *module_tf)
            port_in_world = _compose_pose(*module_in_world, t, q)
            break
    if port_in_world is None:
        return None

    # Transform port into base_link: base_in_world^{-1} ∘ port_in_world (translation only)
    bt, bq = base_in_world
    base_R = _q_to_R_np(bq)
    p_in_base = base_R.T @ (port_in_world[0] - bt)
    return p_in_base.tolist()


def extract_observation_state(joint_msg, controller_msg, mode: str = "full",
                              port_entrance_baselink: Optional[list] = None) -> list[float]:
    """Extract observation state vector.

    Modes:
      - "full" (default): 25D proprio = tcp_pose (7) + tcp_velocity (6) +
        tcp_error (6) + joint_positions (6). Wrench is appended separately
        in the caller, yielding 31D total.
      - "joints": 6D joint positions only (matches LeHome approach).
      - "joints_port": 9D = 6D joints + 3D port_entrance position in base_link.
        Used to test whether giving the model an explicit goal (ground truth
        port pose, as would come from a perception module) closes the ~10cm
        xy error gap we see with image-only observations.
      - "joints_pose": 15D = 6D joints + 9D current TCP pose (xyz + 6D
        rotation matrix, Zhou et al. 2019). Pairs with the
        cartesian_pose action mode so state and action live in the same
        representation — simpler mapping, no implicit FK required.
      - "joints_pose_wrench": 15D joints_pose + 6D wrench = 21D. Wrench
        is appended by the caller (same pathway as "full"). Target state
        for Exp-20a — adds force-torque feedback for contact-rich insertion.
    """
    if mode in ("joints", "joints_port", "joints_pose", "joints_pose_wrench"):
        state: list[float] = []
        if joint_msg is not None:
            if hasattr(joint_msg, 'name') and joint_msg.name:
                name_to_pos = dict(zip(joint_msg.name, joint_msg.position))
                state = [name_to_pos.get(jn, 0.0) for jn in ARM_JOINT_NAMES]
            else:
                state = list(joint_msg.position)[:6]
        else:
            state = [0.0] * 6
        if mode == "joints_port":
            if port_entrance_baselink is not None and len(port_entrance_baselink) >= 3:
                state.extend(port_entrance_baselink[:3])
            else:
                state.extend([0.0, 0.0, 0.0])
        elif mode in ("joints_pose", "joints_pose_wrench"):
            # Current TCP pose from controller_state → 9D (xyz + 6D rotation)
            if controller_msg is not None:
                p = controller_msg.tcp_pose.position
                o = controller_msg.tcp_pose.orientation
                state.extend(pose_to_xyz_rot6d(
                    np.array([p.x, p.y, p.z, o.x, o.y, o.z, o.w])))
            else:
                state.extend([0.0] * 9)
        return state

    # "full" mode
    state = []
    if controller_msg is not None:
        p = controller_msg.tcp_pose.position
        o = controller_msg.tcp_pose.orientation
        state.extend([p.x, p.y, p.z, o.x, o.y, o.z, o.w])
        v = controller_msg.tcp_velocity
        state.extend([v.linear.x, v.linear.y, v.linear.z,
                       v.angular.x, v.angular.y, v.angular.z])
        state.extend(list(controller_msg.tcp_error[:6]))
    else:
        state.extend([0.0] * 19)

    if joint_msg is not None:
        if hasattr(joint_msg, 'name') and joint_msg.name:
            name_to_pos = dict(zip(joint_msg.name, joint_msg.position))
            for jn in ARM_JOINT_NAMES:
                state.append(name_to_pos.get(jn, 0.0))
        else:
            state.extend(list(joint_msg.position)[:6])
    else:
        state.extend([0.0] * 6)
    return state


def extract_pose_cartesian(msg) -> Optional[np.ndarray]:
    """Extract absolute TCP pose [x, y, z, qx, qy, qz, qw] from a MODE_POSITION MotionUpdate.
    Returns None if the message is velocity-mode or None."""
    if msg is None:
        return None
    mode = 0
    if hasattr(msg, 'trajectory_generation_mode') and hasattr(msg.trajectory_generation_mode, 'mode'):
        mode = msg.trajectory_generation_mode.mode
    if mode == 1:
        return None  # velocity mode — no absolute pose
    # mode=2 (position) or unknown: extract pose
    p = msg.pose.position
    o = msg.pose.orientation
    return np.array([p.x, p.y, p.z, o.x, o.y, o.z, o.w], dtype=np.float64)


def pose_to_xyz_rot6d(pose_xyzq: np.ndarray) -> list[float]:
    """Convert [x,y,z, qx,qy,qz,qw] → 9D [x,y,z, r1x,r1y,r1z, r2x,r2y,r2z].

    The 6D rotation is the first two columns of the rotation matrix
    (Zhou et al. 2019, "On the Continuity of Rotation Representations").
    This representation is continuous (no quaternion double-cover, no
    Euler singularities) and is inverted at inference by Gram-Schmidt.
    """
    x, y, z, qx, qy, qz, qw = pose_xyzq
    # Rotation matrix from quaternion
    xx, yy, zz = qx*qx, qy*qy, qz*qz
    xy, xz, yz = qx*qy, qx*qz, qy*qz
    wx, wy, wz = qw*qx, qw*qy, qw*qz
    # First column = R[:, 0]
    r1 = [1 - 2*(yy + zz), 2*(xy + wz), 2*(xz - wy)]
    # Second column = R[:, 1]
    r2 = [2*(xy - wz), 1 - 2*(xx + zz), 2*(yz + wx)]
    return [float(x), float(y), float(z),
            float(r1[0]), float(r1[1]), float(r1[2]),
            float(r2[0]), float(r2[1]), float(r2[2])]


def interpolate_pose(pose_a: np.ndarray, pose_b: np.ndarray, alpha: float) -> np.ndarray:
    """Linearly interpolate position + SLERP quaternion between two poses.

    Each pose is [x, y, z, qx, qy, qz, qw].  alpha=0 returns pose_a, alpha=1 returns pose_b.
    """
    p = (1 - alpha) * pose_a[:3] + alpha * pose_b[:3]

    # Quaternion SLERP (xyzw layout)
    qa, qb = pose_a[3:7].copy(), pose_b[3:7].copy()
    dot = float(np.dot(qa, qb))
    if dot < 0:
        qb = -qb
        dot = -dot
    dot = min(dot, 1.0)
    if dot > 0.9995:
        q = (1 - alpha) * qa + alpha * qb
    else:
        theta = np.arccos(dot)
        sin_theta = np.sin(theta)
        q = (np.sin((1 - alpha) * theta) / sin_theta) * qa + (np.sin(alpha * theta) / sin_theta) * qb
    q /= np.linalg.norm(q)
    return np.concatenate([p, q])


def build_interpolated_poses(
    action_msgs: list, action_ts: list[float], ref_timestamps: list[float]
) -> list[Optional[np.ndarray]]:
    """Pre-compute interpolated poses at each reference timestamp.

    Instead of snapping to the nearest pose command (which creates zero-action
    frames when commands arrive at lower frequency than the target FPS),
    linearly interpolate between consecutive pose commands.  This produces
    smooth, non-zero deltas at every frame.
    """
    # Extract all valid poses with timestamps
    pose_data: list[tuple[float, np.ndarray]] = []
    for t, msg in zip(action_ts, [m for _, m in action_msgs]):
        pose = extract_pose_cartesian(msg)
        if pose is not None:
            pose_data.append((t, pose))
    if not pose_data:
        return [None] * len(ref_timestamps)

    poses_t = [x[0] for x in pose_data]
    poses_v = [x[1] for x in pose_data]

    result: list[Optional[np.ndarray]] = []
    for t in ref_timestamps:
        if t <= poses_t[0]:
            result.append(poses_v[0].copy())
        elif t >= poses_t[-1]:
            result.append(poses_v[-1].copy())
        else:
            # Find bracketing indices
            idx = bisect.bisect_right(poses_t, t) - 1
            idx = max(0, min(idx, len(poses_t) - 2))
            t0, t1 = poses_t[idx], poses_t[idx + 1]
            dt = t1 - t0
            alpha = (t - t0) / dt if dt > 1e-9 else 0.0
            alpha = max(0.0, min(1.0, alpha))
            result.append(interpolate_pose(poses_v[idx], poses_v[idx + 1], alpha))
    return result


def extract_joint_positions(joint_msg) -> list[float]:
    """Extract 6D joint positions from a JointState message in canonical order."""
    if joint_msg is None:
        return [0.0] * 6
    name_to_pos = dict(zip(joint_msg.name, joint_msg.position))
    return [name_to_pos.get(jn, 0.0) for jn in ARM_JOINT_NAMES]


def pose_delta_to_twist(prev: np.ndarray, curr: np.ndarray) -> list[float]:
    """Compute 6D delta twist [dx, dy, dz, dax, day, daz] from two absolute poses.

    Translation delta is simply (curr - prev) for xyz.
    Rotation delta is expressed as a rotation vector (axis × angle) derived from
    the quaternion difference q_delta = curr_q * inv(prev_q).  This matches the
    6D Cartesian twist format used by the original RunACT / MODE_VELOCITY controller
    and avoids all quaternion normalization issues during training.
    """
    # Position delta
    dp = curr[:3] - prev[:3]

    # Quaternion difference: q_delta = curr_q * conj(prev_q)
    # Quaternion layout: [qx, qy, qz, qw]
    qc = curr[3:7]  # [qx, qy, qz, qw]
    qp = prev[3:7]
    # Conjugate of prev: [-qx, -qy, -qz, qw]
    qp_conj = np.array([-qp[0], -qp[1], -qp[2], qp[3]])
    # Hamilton product: curr * conj(prev)  (xyzw layout)
    ax, ay, az, aw = qc
    bx, by, bz, bw = qp_conj
    dqx = aw*bx + ax*bw + ay*bz - az*by
    dqy = aw*by - ax*bz + ay*bw + az*bx
    dqz = aw*bz + ax*by - ay*bx + az*bw
    dqw = aw*bw - ax*bx - ay*by - az*bz

    # Rotation vector (axis-angle): v = 2 * arccos(|dqw|) * [dqx, dqy, dqz] / sin(...)
    dqw = np.clip(dqw, -1.0, 1.0)
    half_angle = np.arccos(abs(dqw))
    sin_half = np.sin(half_angle)
    if sin_half < 1e-8:
        rotvec = np.zeros(3)
    else:
        sign = 1.0 if dqw >= 0 else -1.0
        rotvec = sign * 2.0 * half_angle * np.array([dqx, dqy, dqz]) / sin_half

    return dp.tolist() + rotvec.tolist()


def extract_action_joint(msg) -> list[float]:
    if msg is None:
        return [0.0] * 7
    vels = list(msg.target_state.velocities)
    vels = vels[:6] if len(vels) >= 6 else vels + [0.0] * (6 - len(vels))
    return vels + [0.0]  # pad to 7-dim for consistency


class FfmpegVideoWriter:
    """Write raw BGR frames to an MP4 via an ffmpeg subprocess pipe."""

    def __init__(self, path: str, width: int, height: int, fps: int):
        self.path = path
        self.width = width
        self.height = height
        # Capture ffmpeg stderr to a sibling log so we can diagnose
        # BrokenPipeError mid-write (was silently discarded before).
        self._stderr_log_path = path + ".ffmpeg_stderr.log"
        self._stderr_log = open(self._stderr_log_path, "w")
        self.proc = subprocess.Popen(
            [
                "ffmpeg", "-y",
                "-f", "rawvideo",
                "-vcodec", "rawvideo",
                "-pix_fmt", "bgr24",
                "-s", f"{width}x{height}",
                "-r", str(fps),
                "-i", "-",
                "-c:v", "libx264",
                "-preset", "ultrafast",   # fastest encoding
                "-crf", "23",
                "-pix_fmt", "yuv420p",
                "-an",
                path,
            ],
            stdin=subprocess.PIPE,
            stdout=subprocess.DEVNULL,
            stderr=self._stderr_log,
        )
        self.frame_count = 0

    def write(self, frame: np.ndarray):
        try:
            self.proc.stdin.write(frame.tobytes())
        except BrokenPipeError:
            # ffmpeg subprocess died — surface the actual reason
            self._stderr_log.flush()
            try:
                with open(self._stderr_log_path) as f:
                    err_tail = f.read()[-2000:]
            except Exception:
                err_tail = "(could not read ffmpeg stderr)"
            raise RuntimeError(
                f"ffmpeg pipe broken at frame {self.frame_count} writing "
                f"{self.path}; ffmpeg stderr tail:\n{err_tail}"
            )
        self.frame_count += 1

    def close(self):
        self.proc.stdin.close()
        self.proc.wait()


class OpenCVVideoWriter:
    """Fallback writer when ffmpeg is not available."""

    def __init__(self, path: str, width: int, height: int, fps: int):
        fourcc = cv2.VideoWriter_fourcc(*"mp4v")
        self.writer = cv2.VideoWriter(path, fourcc, fps, (width, height))
        self.frame_count = 0

    def write(self, frame: np.ndarray):
        self.writer.write(frame)
        self.frame_count += 1

    def close(self):
        self.writer.release()


def make_video_writer(path: str, width: int, height: int, fps: int):
    if ffmpeg_available():
        return FfmpegVideoWriter(path, width, height, fps)
    return OpenCVVideoWriter(path, width, height, fps)


def streaming_convert_episode(
    mcap_path: str,
    ref_timestamps: list[float],
    action_mode: str,
    image_scale: float,
    fps: int,
    video_paths: dict[str, str],
    camera_topics: Optional[dict] = None,
    compressed: bool = False,
    obs_state_mode: str = "full",
    port_entrance_baselink: Optional[list] = None,
) -> tuple[list[dict], int]:
    """
    Single streaming pass: reads the MCAP once, decodes only needed messages,
    and pipes camera frames directly to ffmpeg writers.

    Supports both raw sensor_msgs/Image (compressed=False) and
    sensor_msgs/CompressedImage (compressed=True, already at target resolution).

    Returns (parquet_rows, total_frame_count).
    """
    if not ref_timestamps:
        return [], 0

    if camera_topics is None:
        camera_topics = CAMERA_TOPICS_RAW

    t0_episode = ref_timestamps[0]
    target_dt = 1.0 / fps
    img_h = int(CAMERA_HEIGHT * image_scale)
    img_w = int(CAMERA_WIDTH * image_scale)
    black_frame = np.zeros((img_h, img_w, 3), dtype=np.uint8)

    tolerance = target_dt * 0.5

    topic_to_cam = {v: k for k, v in camera_topics.items()}
    action_topic = (TOPIC_POSE_COMMANDS
                    if action_mode in ("cartesian", "cartesian_pose")
                    else TOPIC_JOINT_COMMANDS)
    image_topics = set(camera_topics.values())
    non_image_topics = NEEDED_TOPICS - set(CAMERA_TOPICS_RAW.values()) - set(CAMERA_TOPICS_SMALL.values())

    # ── Collect non-image data (small, fits in RAM easily) ────────────────
    # Timestamps are pulled from each message's `header.stamp` (sim time),
    # not the recorder's `log_time` (wall). Keeps ref-matching, action
    # interpolation, and slicing all in a single sim-time domain — under
    # host load or RTF != 1, log_time diverges from sim time and breaks
    # the train/eval temporal correspondence.
    def _hdr_stamp(msg) -> float:
        s = msg.header.stamp
        return s.sec + s.nanosec * 1e-9

    joint_msgs: list[tuple[float, Any]] = []
    ctrl_msgs: list[tuple[float, Any]] = []
    action_msgs: list[tuple[float, Any]] = []
    fts_msgs: list[tuple[float, Any]] = []

    # Per-topic seen-stamp sets. AIC bags from accumulated-publisher trials
    # contain N identical copies of every header-stamped message (one per
    # leaked publisher). Without dedup the *_msgs lists scale with N×count
    # and OOM workers on big bags; with dedup they scale with unique stamps.
    # Stamp tuples are (sec, nanosec) for exact-equality dedup matching the
    # converter's downstream behavior (which also keys off header.stamp).
    seen_js: set[tuple[int, int]] = set()
    seen_ctrl: set[tuple[int, int]] = set()
    seen_action: set[tuple[int, int]] = set()
    seen_fts: set[tuple[int, int]] = set()

    def _stamp_key(msg) -> tuple[int, int]:
        s = msg.header.stamp
        return (s.sec, s.nanosec)

    with open(mcap_path, "rb") as f:
        reader = make_reader(f, decoder_factories=[DecoderFactory()])
        for schema, channel, message, decoded_msg in reader.iter_decoded_messages(
            topics=non_image_topics
        ):
            topic = channel.topic
            # Filter to only-needed topics BEFORE extracting header.stamp.
            # `non_image_topics` widens to all command topics, including the
            # un-selected one (e.g. joint_commands when action_mode is
            # cartesian_pose). JointMotionUpdate has no header field, so an
            # unconditional _hdr_stamp would crash on those.
            if topic == TOPIC_JOINT_STATES:
                key = _stamp_key(decoded_msg)
                if key in seen_js:
                    continue
                seen_js.add(key)
                joint_msgs.append((_hdr_stamp(decoded_msg), decoded_msg))
            elif topic == TOPIC_CONTROLLER_STATE:
                key = _stamp_key(decoded_msg)
                if key in seen_ctrl:
                    continue
                seen_ctrl.add(key)
                ctrl_msgs.append((_hdr_stamp(decoded_msg), decoded_msg))
            elif topic == action_topic:
                # Only dedup when action_topic is pose_commands (has header).
                # joint_commands path uses JointMotionUpdate which has no
                # header field — leave the existing behavior untouched.
                if action_topic == TOPIC_POSE_COMMANDS:
                    key = _stamp_key(decoded_msg)
                    if key in seen_action:
                        continue
                    seen_action.add(key)
                    action_msgs.append((_hdr_stamp(decoded_msg), decoded_msg))
                else:
                    action_msgs.append((_hdr_stamp(decoded_msg), decoded_msg))
            elif topic == TOPIC_FTS_WRENCH:
                key = _stamp_key(decoded_msg)
                if key in seen_fts:
                    continue
                seen_fts.add(key)
                fts_msgs.append((_hdr_stamp(decoded_msg), decoded_msg))

    joint_msgs.sort(key=lambda x: x[0])
    ctrl_msgs.sort(key=lambda x: x[0])
    action_msgs.sort(key=lambda x: x[0])
    fts_msgs.sort(key=lambda x: x[0])

    joint_ts = [x[0] for x in joint_msgs]
    ctrl_ts = [x[0] for x in ctrl_msgs]
    action_ts = [x[0] for x in action_msgs]
    fts_ts = [x[0] for x in fts_msgs]
    # Pre-extract wrench tuples once; smooth_at_frame slices into this with
    # bisect so the smoothing pass is O(log N + window_size) per frame.
    fts_tuples: list[tuple[float, float, float, float, float, float]] = [
        (m.wrench.force.x, m.wrench.force.y, m.wrench.force.z,
         m.wrench.torque.x, m.wrench.torque.y, m.wrench.torque.z)
        for _, m in fts_msgs
    ]

    # ── Open video writers (before decode so we can stream-write frames) ─────
    writers: dict[str, Any] = {}
    for cam_name, vpath in video_paths.items():
        writers[cam_name] = make_video_writer(vpath, img_w, img_h, fps)

    # ── Stream images + write frames as they become ready ────────────────────
    # Instead of buffering all ~1800 frames per camera in image_at_ref (1-2GB
    # per worker), we flush completed frames to the video writer immediately and
    # free the memory. A frame at ref index ri is "ready" for writing once every
    # camera has advanced past ri (i.e. min(ref_idx_per_cam.values()) > ri).
    cam_latest: dict[str, Optional[np.ndarray]] = {c: None for c in camera_topics}
    # Sparse dict: only holds frames not yet flushed
    pending: dict[str, dict[int, Optional[np.ndarray]]] = {c: {} for c in camera_topics}
    ref_idx_per_cam: dict[str, int] = {c: 0 for c in camera_topics}
    last_flushed = -1
    rows: list[dict] = []
    prev_pose: Optional[np.ndarray] = None

    # Pre-compute interpolated poses at every 20Hz ref timestamp to avoid
    # zero-action frames when pose commands arrive at lower frequency.
    if action_mode in ("cartesian", "cartesian_pose") and ref_timestamps and action_msgs:
        interp_poses = build_interpolated_poses(action_msgs, action_ts, ref_timestamps)
    else:
        interp_poses = [None] * len(ref_timestamps)

    def _decode_and_resize(msg):
        if compressed:
            return decode_compressed_image(msg)
        img = decode_ros2_image(msg)
        if img is not None and image_scale != 1.0:
            img = cv2.resize(img, (img_w, img_h), interpolation=cv2.INTER_AREA)
        return img

    def _flush_ready():
        """Write and free all frames that all cameras have confirmed."""
        nonlocal last_flushed, prev_pose
        flush_up_to = min(ref_idx_per_cam.values())  # exclusive: all cams past here
        for fi in range(last_flushed + 1, flush_up_to):
            t = ref_timestamps[fi]
            # Compute row (state/action data)
            ji = binary_search_nearest(joint_ts, t)
            ci = binary_search_nearest(ctrl_ts, t)
            ai = binary_search_nearest(action_ts, t)
            joint_msg  = joint_msgs[ji][1]  if ji  >= 0 else None
            ctrl_msg   = ctrl_msgs[ci][1]   if ci  >= 0 else None
            action_msg = action_msgs[ai][1] if ai  >= 0 else None
            obs_state = extract_observation_state(
                joint_msg, ctrl_msg, mode=obs_state_mode,
                port_entrance_baselink=port_entrance_baselink,
            )
            # Append wrench in "full" and "joints_pose_wrench" modes. Other joints-based
            # modes stay strictly proprio. Smooth across (t - window, t] —
            # same window/mode the runtime smoother uses, so train and eval
            # see the same wrench distribution.
            if obs_state_mode in ("full", "joints_pose_wrench"):
                wrench = smooth_wrench_at(fts_ts, fts_tuples, t)
            else:
                wrench = []
            if action_mode == "cartesian":
                curr_pose = interp_poses[fi] if fi < len(interp_poses) else None
                if curr_pose is None or prev_pose is None:
                    action = [0.0] * 6
                else:
                    action = pose_delta_to_twist(prev_pose, curr_pose)
                prev_pose = curr_pose if curr_pose is not None else prev_pose
            elif action_mode == "cartesian_pose":
                # 9D action: [x, y, z, r1x, r1y, r1z, r2x, r2y, r2z].
                # Loss acts directly on tip pose, so gradient reflects
                # task-relevant precision (unlike joint-angle L1).
                curr_pose = interp_poses[fi] if fi < len(interp_poses) else None
                if curr_pose is None:
                    # No valid pose this frame; repeat previous if available
                    curr_pose = prev_pose
                if curr_pose is None:
                    action = [0.0] * 9
                else:
                    action = pose_to_xyz_rot6d(curr_pose)
                prev_pose = curr_pose if curr_pose is not None else prev_pose
            else:
                # Joint mode: action = next timestep's joint positions.
                # Look ahead one frame for the target joint state.
                next_fi = fi + 1
                if next_fi < len(ref_timestamps):
                    next_ji = binary_search_nearest(joint_ts, ref_timestamps[next_fi])
                    next_joint_msg = joint_msgs[next_ji][1] if next_ji >= 0 else None
                    action = extract_joint_positions(next_joint_msg)
                else:
                    # Last frame: repeat current joint positions (hold position)
                    action = extract_joint_positions(joint_msg)
            rows.append({
                "timestamp": t - t0_episode,
                "frame_index": fi,
                "observation.state": obs_state + wrench,
                "action": action,
            })
            # Write and immediately free frames (O(1) memory per frame)
            for cam_name in camera_topics:
                frame = pending[cam_name].pop(fi, None)
                writers[cam_name].write(frame if frame is not None else black_frame)
        last_flushed = flush_up_to - 1

    with open(mcap_path, "rb") as f:
        reader = make_reader(f, decoder_factories=[DecoderFactory()])
        for schema, channel, message, decoded_msg in reader.iter_decoded_messages(
            topics=image_topics
        ):
            # Sim time from msg header (matches ref_timestamps domain).
            # Messages with duplicate header.stamps (publisher bursts) all
            # see the same `t`; the matching block below commits one frame
            # per ref index and the rest harmlessly refresh cam_latest.
            t = _hdr_stamp(decoded_msg)
            cam_name = topic_to_cam.get(channel.topic)
            if cam_name is None:
                continue

            ri = ref_idx_per_cam[cam_name]

            # Advance past any ref timestamps this image is too late for,
            # filling skipped slots with the last known frame for this camera
            while ri < len(ref_timestamps) and ref_timestamps[ri] < t - tolerance:
                if ri not in pending[cam_name]:
                    pending[cam_name][ri] = cam_latest[cam_name]
                ri += 1
            ref_idx_per_cam[cam_name] = ri

            # Decode every camera message and keep cam_latest fresh. The
            # tolerance check used to gate decoding, but L/R cameras have a
            # systematic ~14–27 ms log_time offset from center; with a 10 ms
            # tolerance their messages NEVER matched a center-derived
            # ref_timestamp and cam_latest stayed frozen on the first frame
            # for the entire bag (every L/R video was a single still image).
            # Latest-known semantics fix that with no real cost — decoding
            # every kept message instead of one per ref bin still runs at
            # camera-rate, not ref-rate.
            img = _decode_and_resize(decoded_msg)
            cam_latest[cam_name] = img

            # If this message lines up with the current ref timestamp,
            # commit it and advance.
            if ri < len(ref_timestamps) and abs(t - ref_timestamps[ri]) < tolerance:
                pending[cam_name][ri] = img
                ref_idx_per_cam[cam_name] = ri + 1

            _flush_ready()

    # Finalize: advance all cameras to end, filling remaining slots
    for cam_name in camera_topics:
        ri = ref_idx_per_cam[cam_name]
        while ri < len(ref_timestamps):
            if ri not in pending[cam_name]:
                pending[cam_name][ri] = cam_latest[cam_name]
            ri += 1
        ref_idx_per_cam[cam_name] = len(ref_timestamps)
    _flush_ready()  # flush the last batch

    for w in writers.values():
        w.close()

    return rows, len(rows)


# ═══════════════════════════════════════════════════════════════════════════════
#  LEROBOT DATASET WRITER
# ═══════════════════════════════════════════════════════════════════════════════

def create_dataset_structure(output_dir: str) -> dict[str, Path]:
    base = Path(output_dir)
    dirs = {"root": base, "data": base / "data" / "chunk-000", "meta": base / "meta"}
    for cam_name in CAMERA_TOPICS_RAW:  # cam names are the same for raw and small
        # v3.0 layout: videos/{camera}/chunk-000/
        dirs[f"video_{cam_name}"] = (
            base / "videos" / f"observation.images.{cam_name}" / "chunk-000"
        )
    for d in dirs.values():
        d.mkdir(parents=True, exist_ok=True)
    return dirs


def write_episode_parquet(rows, episode_idx, task_index, parquet_dir, global_frame_offset: int = 0):
    """Write one episode's rows to a v3.0 parquet file (no embedded images)."""
    if not rows:
        return 0

    timestamps = []
    frame_indices = []
    episode_indices = []
    global_indices = []
    task_indices = []
    states = []
    actions = []

    for i, row in enumerate(rows):
        timestamps.append(float(i) / 20.0)   # clean i/fps timestamps
        frame_indices.append(i)
        episode_indices.append(episode_idx)
        global_indices.append(global_frame_offset + i)
        task_indices.append(task_index)
        states.append([float(v) for v in row["observation.state"]])
        actions.append([float(v) for v in row["action"]])

    columns = {
        "timestamp": pa.array(timestamps, type=pa.float32()),
        "frame_index": pa.array(frame_indices, type=pa.int64()),
        "episode_index": pa.array(episode_indices, type=pa.int64()),
        "index": pa.array(global_indices, type=pa.int64()),
        "task_index": pa.array(task_indices, type=pa.int64()),
        "observation.state": pa.array(states, type=pa.list_(pa.float32())),
        "action": pa.array(actions, type=pa.list_(pa.float32())),
    }

    table = pa.table(columns)
    out_path = parquet_dir / f"file-{episode_idx:06d}.parquet"
    pq.write_table(table, str(out_path))
    return len(rows)


def compute_stats(all_states, all_actions, camera_keys=None):
    stats = {}
    if all_states:
        s = np.array(all_states, dtype=np.float64)
        stats["observation.state"] = {
            "mean": s.mean(0).tolist(), "std": s.std(0).tolist(),
            "min": s.min(0).tolist(), "max": s.max(0).tolist(),
        }
    if all_actions:
        a = np.array(all_actions, dtype=np.float64)
        stats["action"] = {
            "mean": a.mean(0).tolist(), "std": a.std(0).tolist(),
            "min": a.min(0).tolist(), "max": a.max(0).tolist(),
        }
    # lerobot's factory.make_dataset() populates ImageNet stats by indexing
    # `dataset.meta.stats[key][stats_type] = ...` for every camera key. The
    # outer key must already exist or the assignment KeyErrors. We have to
    # pre-populate non-empty stat arrays — `cast_stats_to_numpy` flattens
    # the dict and silently DROPS empty leaves, so empty stub dicts here
    # come back missing after `load_stats`. Use ImageNet defaults as the
    # placeholder; lerobot overwrites them anyway when use_imagenet_stats
    # is True (the default), so the values don't actually matter.
    if camera_keys:
        for k in camera_keys:
            stats.setdefault(k, {
                "mean": [[[0.485]], [[0.456]], [[0.406]]],
                "std":  [[[0.229]], [[0.224]], [[0.225]]],
                "min":  [[[0.0]],   [[0.0]],   [[0.0]]],
                "max":  [[[1.0]],   [[1.0]],   [[1.0]]],
            })
    return stats


def write_metadata(meta_dir, repo_id, unique_tasks, episode_task_indices,
                   action_mode, fps, image_scale, episode_lengths, stats, total_frames,
                   episode_provenance: Optional[list[dict]] = None,
                   obs_state_mode: str = "full"):
    """Write LeRobot v3.0 metadata: info.json, tasks.parquet, episodes parquet, stats.json."""
    import pandas as pd

    meta_dir = Path(meta_dir)
    meta_dir.mkdir(parents=True, exist_ok=True)

    img_h = int(CAMERA_HEIGHT * image_scale)
    img_w = int(CAMERA_WIDTH * image_scale)

    if obs_state_mode == "joints":
        state_names = list(ARM_JOINT_NAMES)
    elif obs_state_mode == "joints_port":
        state_names = list(ARM_JOINT_NAMES) + [
            "port_entrance.position.x",
            "port_entrance.position.y",
            "port_entrance.position.z",
        ]
    elif obs_state_mode == "joints_pose":
        state_names = list(ARM_JOINT_NAMES) + [
            "tcp.position.x", "tcp.position.y", "tcp.position.z",
            "tcp.rot6d.0", "tcp.rot6d.1", "tcp.rot6d.2",
            "tcp.rot6d.3", "tcp.rot6d.4", "tcp.rot6d.5",
        ]
    elif obs_state_mode == "joints_pose_wrench":
        state_names = list(ARM_JOINT_NAMES) + [
            "tcp.position.x", "tcp.position.y", "tcp.position.z",
            "tcp.rot6d.0", "tcp.rot6d.1", "tcp.rot6d.2",
            "tcp.rot6d.3", "tcp.rot6d.4", "tcp.rot6d.5",
            "wrench.force.x", "wrench.force.y", "wrench.force.z",
            "wrench.torque.x", "wrench.torque.y", "wrench.torque.z",
        ]
    else:
        state_names = [
            "tcp_pose.position.x", "tcp_pose.position.y", "tcp_pose.position.z",
            "tcp_pose.orientation.x", "tcp_pose.orientation.y",
            "tcp_pose.orientation.z", "tcp_pose.orientation.w",
            "tcp_velocity.linear.x", "tcp_velocity.linear.y", "tcp_velocity.linear.z",
            "tcp_velocity.angular.x", "tcp_velocity.angular.y", "tcp_velocity.angular.z",
            "tcp_error.x", "tcp_error.y", "tcp_error.z",
            "tcp_error.rx", "tcp_error.ry", "tcp_error.rz",
            "joint_positions.0", "joint_positions.1", "joint_positions.2",
            "joint_positions.3", "joint_positions.4", "joint_positions.5",
            "wrench.force.x", "wrench.force.y", "wrench.force.z",
            "wrench.torque.x", "wrench.torque.y", "wrench.torque.z",
        ]
    if action_mode == "cartesian":
        action_names = ["delta.linear.x", "delta.linear.y", "delta.linear.z",
                        "delta.angular.x", "delta.angular.y", "delta.angular.z"]
    elif action_mode == "cartesian_pose":
        action_names = ["tcp.position.x", "tcp.position.y", "tcp.position.z",
                        "tcp.rot6d.0", "tcp.rot6d.1", "tcp.rot6d.2",
                        "tcp.rot6d.3", "tcp.rot6d.4", "tcp.rot6d.5"]
    else:
        action_names = ARM_JOINT_NAMES

    cam_feature = lambda: {
        "dtype": "video", "shape": [img_h, img_w, 3],
        "names": ["height", "width", "channels"],
        "video_info": {
            "video.fps": fps, "video.height": img_h, "video.width": img_w,
            "video.channels": 3, "video.codec": "h264",
            "video.pix_fmt": "yuv420p", "has_audio": False,
        },
    }

    # ── info.json (v3.0) ──────────────────────────────────────────────────────
    info = {
        "codebase_version": "v3.0",
        "robot_type": "ur5e_aic",
        "total_episodes": len(episode_lengths),
        "total_frames": total_frames,
        "total_tasks": len(unique_tasks),
        "chunks_size": 1000,
        "data_files_size_in_mb": 100,
        "video_files_size_in_mb": 200,
        "fps": fps,
        "splits": {"train": f"0:{len(episode_lengths)}"},
        "data_path": "data/chunk-{chunk_index:03d}/file-{file_index:06d}.parquet",
        "video_path": "videos/{video_key}/chunk-{chunk_index:03d}/file-{file_index:06d}.mp4",
        "features": {
            "observation.state": {
                "dtype": "float32", "shape": [len(state_names)], "names": state_names,
            },
            "action": {
                "dtype": "float32", "shape": [len(action_names)], "names": action_names,
            },
            "observation.images.left_camera": cam_feature(),
            "observation.images.center_camera": cam_feature(),
            "observation.images.right_camera": cam_feature(),
            "timestamp": {"dtype": "float32", "shape": [1], "names": None},
            "frame_index": {"dtype": "int64", "shape": [1], "names": None},
            "episode_index": {"dtype": "int64", "shape": [1], "names": None},
            "index": {"dtype": "int64", "shape": [1], "names": None},
            "task_index": {"dtype": "int64", "shape": [1], "names": None},
        },
    }
    with open(meta_dir / "info.json", "w") as f:
        json.dump(info, f, indent=2)

    # ── meta/tasks.parquet ────────────────────────────────────────────────────
    # Pandas DataFrame with string index (task text) and task_index column.
    # lerobot's load_tasks() reads this and uses df.index as task strings.
    df_tasks = pd.DataFrame(
        {"task_index": list(range(len(unique_tasks)))},
        index=unique_tasks,
    )
    df_tasks.to_parquet(str(meta_dir / "tasks.parquet"))

    # ── meta/episodes/chunk-000/file-000.parquet ──────────────────────────────
    # One row per episode. With 1 file per episode: file_index = episode_index.
    video_keys = [f"observation.images.{cam}" for cam in CAMERA_TOPICS_RAW]
    ep_rows = []
    cumulative = 0
    for ep_idx, length in enumerate(episode_lengths):
        task_str = unique_tasks[episode_task_indices[ep_idx]]
        row = {
            "episode_index": ep_idx,
            "tasks": [task_str],
            "length": length,
            "data/chunk_index": 0,
            "data/file_index": ep_idx,
            "dataset_from_index": cumulative,
            "dataset_to_index": cumulative + length,
            "meta/episodes/chunk_index": 0,
            "meta/episodes/file_index": 0,
        }
        for vk in video_keys:
            row[f"videos/{vk}/chunk_index"] = 0
            row[f"videos/{vk}/file_index"] = ep_idx
            row[f"videos/{vk}/from_timestamp"] = 0.0
            row[f"videos/{vk}/to_timestamp"] = float(length) / fps
        cumulative += length
        ep_rows.append(row)

    ep_df = pd.DataFrame(ep_rows)

    # Add per-episode provenance (source bag, tier_3 score, config YAML) if available
    if episode_provenance and len(episode_provenance) == len(ep_df):
        ep_df["source_bag"] = [p["source_bag"] for p in episode_provenance]
        ep_df["tier_3_score"] = [p["tier_3_score"] for p in episode_provenance]
        ep_df["config_yaml"] = [p["config_yaml"] for p in episode_provenance]
    elif episode_provenance:
        print(f"  WARNING: provenance length ({len(episode_provenance)}) != episodes ({len(ep_df)}), skipping provenance")

    ep_path = meta_dir / "episodes" / "chunk-000" / "file-000.parquet"
    ep_path.parent.mkdir(parents=True, exist_ok=True)
    ep_df.to_parquet(str(ep_path), index=False)

    # ── meta/stats.json ───────────────────────────────────────────────────────
    with open(meta_dir / "stats.json", "w") as f:
        json.dump(stats, f, indent=2)


# ═══════════════════════════════════════════════════════════════════════════════
#  SINGLE-EPISODE WORKER (for parallel processing)
# ═══════════════════════════════════════════════════════════════════════════════

def process_one_episode(args_tuple):
    """Process a single .mcap file end-to-end. Runs in a separate process.

    Catches per-bag exceptions so that one bad bag (or a transient error)
    doesn't poison ProcessPoolExecutor — which would otherwise silently
    fail every other episode the worker had queued.
    """
    (mcap_path, ep_idx, action_mode, obs_state_mode, image_scale, fps, output_dir,
     task_index, task_string, slice_info) = args_tuple
    # slice_info is (slice_log_time_ns or None, keep_side ∈ {"after","before"}).
    slice_log_time_ns, slice_keep_side = slice_info
    timings = {}

    try:
        # Pass 1: timestamp index (fast — no decoding)
        t0 = time.time()
        ts_index = build_timestamp_index(mcap_path)
        timings["pass1_index"] = time.time() - t0

        camera_topics = CAMERA_TOPICS_SMALL if ts_index.compressed else CAMERA_TOPICS_RAW
        ref_timestamps = compute_reference_timestamps(ts_index, fps)
        if not ref_timestamps:
            print(f"  [ep {ep_idx}] no ref_timestamps from {Path(mcap_path).parent.name}")
            return (ep_idx, [], 0, timings)

        # If a slice point was provided, drop reference timestamps before it.
        # ref_timestamps are now camera header.stamps (sim time) — see
        # compute_reference_timestamps. slice_log_time_ns from the slice
        # CSV is in the bag's wall-clock log_time domain, so translate
        # by finding the camera_pair whose log_time is closest to the
        # slice point and using that pair's header.stamp as the
        # sim-time threshold. Falls back to log_time comparison if no
        # camera pairs are available (very old bags).
        if slice_log_time_ns is not None:
            slice_log_t = slice_log_time_ns / 1e9
            # Find sim-time equivalent of slice_log_t via the densest camera.
            cam_topics = CAMERA_TOPICS_SMALL if ts_index.compressed else CAMERA_TOPICS_RAW
            slice_sim_t = slice_log_t  # fallback if camera_pairs missing
            for cam_topic in (cam_topics["center_camera"],
                              cam_topics["left_camera"],
                              cam_topics["right_camera"]):
                pairs = ts_index.camera_pairs.get(cam_topic, [])
                if not pairs:
                    continue
                # camera_pairs are sorted by (header.stamp, log_time);
                # pick the pair with log_time closest to slice_log_t.
                best = min(pairs, key=lambda p: abs(p[1] - slice_log_t))
                slice_sim_t = best[0]
                break
            if slice_keep_side == "before":
                kept = [t for t in ref_timestamps if t < slice_sim_t]
                drop_label = "post-anchor"
                keep_label = "pre-anchor"
            else:
                kept = [t for t in ref_timestamps if t >= slice_sim_t]
                drop_label = "pre-handoff"
                keep_label = "insertion"
            n_dropped = len(ref_timestamps) - len(kept)
            if not kept:
                print(f"  [ep {ep_idx}] slice point past end of bag — skipping")
                return (ep_idx, [], 0, timings)
            print(f"  [ep {ep_idx}] sliced ({slice_keep_side}): dropped {n_dropped} "
                  f"{drop_label} frames, kept {len(kept)} {keep_label} frames")
            ref_timestamps = kept

        # Prepare video output paths (v3.0 layout: videos/{cam}/chunk-000/file-XXXXXX.mp4)
        video_paths = {}
        for cam_name in camera_topics:
            vdir = Path(output_dir) / "videos" / f"observation.images.{cam_name}" / "chunk-000"
            vdir.mkdir(parents=True, exist_ok=True)
            video_paths[cam_name] = str(vdir / f"file-{ep_idx:06d}.mp4")

        # Pre-compute port_entrance in base_link for joints_port mode (static per bag)
        port_entrance_baselink = None
        if obs_state_mode == "joints_port":
            port_entrance_baselink = resolve_port_entrance_baselink(mcap_path)
            if port_entrance_baselink is None:
                print(f"  [ep {ep_idx}] WARNING: couldn't resolve port_entrance in base_link; using zeros")

        # Pass 2: streaming decode + write
        t0 = time.time()
        rows, n_frames = streaming_convert_episode(
            mcap_path, ref_timestamps, action_mode, image_scale, fps, video_paths,
            camera_topics=camera_topics, compressed=ts_index.compressed,
            obs_state_mode=obs_state_mode,
            port_entrance_baselink=port_entrance_baselink,
        )
        timings["pass2_convert"] = time.time() - t0

        # Parquet is written by convert() after all episodes complete (needs global_frame_offset)
        return (ep_idx, rows, n_frames, timings)
    except Exception as e:
        import traceback
        print(f"  [ep {ep_idx}] EXCEPTION on {Path(mcap_path).parent.name}: "
              f"{type(e).__name__}: {e}")
        traceback.print_exc()
        return (ep_idx, [], 0, timings)


# ═══════════════════════════════════════════════════════════════════════════════
#  AUTO TASK GENERATION FROM AIC CONFIG
# ═══════════════════════════════════════════════════════════════════════════════

def extract_trial_number(bag_path: str) -> Optional[int]:
    """Extract trial number from bag directory name like 'bag_trial_3_20260324_070458_762'."""
    m = re.search(r'bag_trial_(\d+)', str(bag_path))
    return int(m.group(1)) if m else None


def generate_task_from_config(config: dict, trial_num: int) -> str:
    """Generate a task description from the AIC config for a given trial number."""
    trial_key = f"trial_{trial_num}"
    trial = config.get("trials", {}).get(trial_key)
    if trial is None:
        return f"Insert cable (trial {trial_num})"

    task = trial.get("tasks", {}).get("task_1", {})
    plug_type = task.get("plug_type", "unknown")
    port_name = task.get("port_name", "unknown_port")
    target_module = task.get("target_module_name", "unknown_module")

    plug_name = "SFP module" if plug_type == "sfp" else "SC plug"

    scene = trial.get("scene", {}).get("task_board", {})

    if "nic_card_mount" in target_module:
        card_idx = target_module.split("_")[-1]
        mount_rail = "unknown_rail"
        for i in range(5):
            rail = scene.get(f"nic_rail_{i}", {})
            if rail.get("entity_present") and rail.get("entity_name") == f"nic_card_{card_idx}":
                mount_rail = f"nic_rail_{i}"
                break
        return f"Insert the grasped {plug_name} into {port_name} on the NIC card mounted on {mount_rail}"

    elif "sc_port" in target_module:
        port_idx = target_module.split("_")[-1]
        mount_rail = f"sc_rail_{port_idx}"
        return f"Insert the grasped {plug_name} into {port_name} on {target_module} mounted on {mount_rail}"

    return f"Insert the grasped {plug_name} into {port_name} on {target_module}"


def build_episode_provenance(bag_dirs: list[str], scoring_dir: str) -> list[dict]:
    """Look up per-bag tier_3 scores and config YAMLs from scoring files.

    Uses the same timestamp-bisect logic as convert.sh Step 1b to match each
    bag to the most recent scoring YAML that started before it. Returns a list
    of dicts with keys: source_bag, tier_3_score, config_yaml (one per bag_dir).
    """
    from datetime import datetime
    from bisect import bisect_right
    import glob as _glob

    # Build sorted list of (batch_datetime, trial_scores, config_yaml_name)
    batches = []
    for sf in sorted(_glob.glob(os.path.join(scoring_dir, "*_scoring.yaml"))):
        m = re.search(r'(\d{8}_\d{6})_scoring', sf)
        if not m:
            continue
        ts = datetime.strptime(m.group(1), "%Y%m%d_%H%M%S")
        with open(sf) as f:
            data = yaml.safe_load(f) or {}
        scores = {
            k: v.get("tier_3", {}).get("score", 0) if isinstance(v, dict) else 0
            for k, v in data.items() if k.startswith("trial_")
        }
        config_name = os.path.basename(sf).replace("_scoring.yaml", ".yaml")
        batches.append((ts, scores, config_name))
    batch_times = [b[0] for b in batches]

    provenance = []
    for bag_dir in bag_dirs:
        bag_name = os.path.basename(bag_dir.rstrip("/"))
        m = re.search(r'bag_trial_(\d+)_(\d{8}_\d{6})', bag_name)
        if not m:
            provenance.append({"source_bag": bag_name, "tier_3_score": None, "config_yaml": None})
            continue
        trial_num = int(m.group(1))
        bag_ts = datetime.strptime(m.group(2), "%Y%m%d_%H%M%S")
        idx = bisect_right(batch_times, bag_ts) - 1
        if idx < 0:
            provenance.append({"source_bag": bag_name, "tier_3_score": None, "config_yaml": None})
            continue
        score = batches[idx][1].get(f"trial_{trial_num}", None)
        config_yaml = batches[idx][2]
        provenance.append({"source_bag": bag_name, "tier_3_score": score, "config_yaml": config_yaml})
    return provenance


def load_tasks_from_config(config_path: str, bag_dirs: list[str]) -> list[str]:
    """Load AIC config YAML and generate task descriptions for each bag directory."""
    with open(config_path) as f:
        config = yaml.safe_load(f)

    tasks = []
    for bag_dir in bag_dirs:
        trial_num = extract_trial_number(bag_dir)
        if trial_num is not None:
            task = generate_task_from_config(config, trial_num)
            tasks.append(task)
            print(f"  Trial {trial_num}: {task}")
        else:
            tasks.append("Insert cable into port")
            print(f"  Warning: Could not extract trial number from '{bag_dir}'")

    return tasks


# ═══════════════════════════════════════════════════════════════════════════════
#  MAIN
# ═══════════════════════════════════════════════════════════════════════════════

def convert(args):
    print("=" * 70)
    print("  MCAP → LeRobot v3.0  |  FAST converter for AIC")
    print("=" * 70)

    mcap_paths = []
    bag_dirs = []
    for d in args.mcap_dirs:
        try:
            mcap_paths.append(find_mcap_file(d))
            bag_dirs.append(d)
        except FileNotFoundError as e:
            print(f"  Skipping: {e}")

    if not mcap_paths:
        print("Error: No .mcap files found!")
        sys.exit(1)

    # Optionally load slice CSV — keyed by basename of bag dir.
    # Value is (slice_log_time_ns, keep_side) where keep_side ∈ {"after","before"}.
    # "before" tells the slicer to keep frames preceding the anchor (used for
    # the approach-policy dataset); "after" is the original behavior used for
    # the insertion-specialist dataset.
    slice_by_bag = {}  # basename -> (slice_log_time_ns, keep_side)
    if args.slice_csv:
        import csv as _csv
        n_csv = 0
        with open(args.slice_csv) as _f:
            for row in _csv.DictReader(_f):
                if row.get("drop_reason"):
                    continue
                bp = row.get("bag_path", "").rstrip("/")
                if not bp or not row.get("slice_log_time_ns"):
                    continue
                keep_side = row.get("keep_side", "after") or "after"
                slice_by_bag[Path(bp).name] = (
                    int(row["slice_log_time_ns"]), keep_side
                )
                n_csv += 1
        print(f"\n  Slice CSV:  {args.slice_csv}  ({n_csv} bags with valid slice)")
        # Compatibility shim — older code used a flat dict by-basename.
        slice_log_time_by_bag = {k: v[0] for k, v in slice_by_bag.items()}
        # Filter inputs to only those present in the slice CSV. We keep both
        # mcap_paths and bag_dirs aligned, AND args.tasks if user passed it
        # (so per-bag task strings stay matched after the filter).
        keep_idx = [i for i, bd in enumerate(bag_dirs)
                    if Path(bd.rstrip("/")).name in slice_log_time_by_bag]
        n_orig = len(mcap_paths)
        mcap_paths = [mcap_paths[i] for i in keep_idx]
        bag_dirs = [bag_dirs[i] for i in keep_idx]
        if args.tasks:
            # args.tasks was indexed against args.mcap_dirs; map keep_idx through
            # the (mcap_path-survived → original index) chain via bag_dirs alignment.
            # Simpler: rebuild by matching basename → original task list.
            orig_basename_to_task = {Path(d.rstrip("/")).name: t
                                      for d, t in zip(args.mcap_dirs, args.tasks)}
            args.tasks = [orig_basename_to_task[Path(d.rstrip("/")).name]
                          for d in bag_dirs]
        print(f"  After slice filter: {len(mcap_paths)} / {n_orig} bags retained")
        if not mcap_paths:
            print("Error: No bags left after slice filter")
            sys.exit(1)

    # Generate task strings: from --config-yaml or --tasks
    if args.config_yaml:
        print(f"\n  Loading tasks from: {args.config_yaml}")
        task_strings = load_tasks_from_config(args.config_yaml, bag_dirs)
    elif args.tasks:
        if len(args.tasks) != len(bag_dirs):
            print(f"Error: --tasks ({len(args.tasks)}) must match --mcap-dirs ({len(bag_dirs)})")
            sys.exit(1)
        task_strings = args.tasks
    else:
        print("Error: Provide either --config-yaml or --tasks")
        sys.exit(1)

    # Build task index: deduplicate task strings while preserving order
    unique_tasks = list(dict.fromkeys(task_strings))  # ordered dedup
    task_to_index = {t: i for i, t in enumerate(unique_tasks)}
    episode_task_indices = [task_to_index[t] for t in task_strings]

    use_ffmpeg = ffmpeg_available()
    print(f"\n  Files:       {len(mcap_paths)}")
    print(f"  Tasks:       {len(unique_tasks)} unique")
    for ti, t in enumerate(unique_tasks):
        count = sum(1 for x in episode_task_indices if x == ti)
        print(f"    [{ti}] {t}  ({count} episodes)")
    print(f"  Action mode: {args.action_mode}")
    print(f"  Image scale: {args.image_scale}  →  "
          f"{int(CAMERA_WIDTH * args.image_scale)}×{int(CAMERA_HEIGHT * args.image_scale)}")
    print(f"  Target FPS:  {args.fps}")
    print(f"  Workers:     {args.workers}")
    print(f"  FFmpeg:      {'yes (direct pipe, ultrafast preset)' if use_ffmpeg else 'NO — install ffmpeg for 3-5x speedup!'}")
    print(f"  Output:      {args.output_dir}\n")

    create_dataset_structure(args.output_dir)

    def _slice_for(bd: str):
        info = slice_by_bag.get(Path(bd.rstrip("/")).name) if args.slice_csv else None
        return info if info is not None else (None, "after")

    work_items = [
        (mp, idx, args.action_mode, args.obs_state_mode, args.image_scale, args.fps, args.output_dir,
         episode_task_indices[idx], task_strings[idx],
         _slice_for(bag_dirs[idx]))
        for idx, mp in enumerate(mcap_paths)
    ]

    all_states, all_actions = [], []
    episode_lengths = []
    total_frames = 0
    overall_t0 = time.time()

    parquet_dir = Path(args.output_dir) / "data" / "chunk-000"
    parquet_dir.mkdir(parents=True, exist_ok=True)

    if args.workers > 1 and len(mcap_paths) > 1:
        with ProcessPoolExecutor(max_workers=args.workers) as pool:
            futures = {pool.submit(process_one_episode, item): item[1]
                       for item in work_items}
            results = {}
            for future in as_completed(futures):
                ep_key = futures[future]
                try:
                    results[ep_key] = future.result()
                except Exception as e:
                    print(f"  Episode {ep_key} FAILED: {e}")
                    results[ep_key] = (ep_key, [], 0, {})

            global_frame_offset = 0
            for ep_idx in range(len(mcap_paths)):
                _, rows, n_frames, timings = results[ep_idx]
                _print_episode_summary(ep_idx, mcap_paths[ep_idx], n_frames, timings)
                t0 = time.time()
                write_episode_parquet(rows, ep_idx, episode_task_indices[ep_idx],
                                      parquet_dir, global_frame_offset)
                timings["parquet"] = time.time() - t0
                for row in rows:
                    all_states.append(row["observation.state"])
                    all_actions.append(row["action"])
                episode_lengths.append(n_frames)
                total_frames += n_frames
                global_frame_offset += n_frames
    else:
        global_frame_offset = 0
        for item in work_items:
            ep_idx, rows, n_frames, timings = process_one_episode(item)
            _print_episode_summary(ep_idx, mcap_paths[ep_idx], n_frames, timings)
            t0 = time.time()
            write_episode_parquet(rows, ep_idx, episode_task_indices[ep_idx],
                                  parquet_dir, global_frame_offset)
            timings["parquet"] = time.time() - t0
            for row in rows:
                all_states.append(row["observation.state"])
                all_actions.append(row["action"])
            episode_lengths.append(n_frames)
            total_frames += n_frames
            global_frame_offset += n_frames

    overall_dt = time.time() - overall_t0

    # Build per-episode provenance (source bag, tier_3 score, config YAML)
    episode_provenance = None
    if args.scoring_dir:
        all_provenance = build_episode_provenance(bag_dirs, args.scoring_dir)
        # Filter to only kept episodes (those with n_frames > 0)
        episode_provenance = [all_provenance[i] for i, n in enumerate(episode_lengths) if n > 0]
        scored = sum(1 for p in episode_provenance if p["tier_3_score"] is not None)
        print(f"  Provenance: {scored}/{len(episode_provenance)} episodes with tier_3 scores")

    print("\n─── Computing statistics & writing metadata ───")
    camera_topics = CAMERA_TOPICS_SMALL  # collection bags only ever ship CompressedImage
    camera_keys = [f"observation.images.{c}" for c in camera_topics]
    stats = compute_stats(all_states, all_actions, camera_keys=camera_keys)
    write_metadata(
        Path(args.output_dir) / "meta", args.repo_id, unique_tasks, episode_task_indices,
        args.action_mode, args.fps, args.image_scale, episode_lengths, stats, total_frames,
        episode_provenance=episode_provenance,
        obs_state_mode=args.obs_state_mode,
    )

    # ── val episode split ────────────────────────────────────────────────────
    # Last N episodes by index → held-out validation set. Written here so
    # downstream tools (scripts/train.sh, scripts/val_loss_sidecar.py)
    # have a single canonical source for the split.
    #
    # Only count episodes that actually wrote a parquet file (n_frames > 0).
    # Failed bags (mcap corruption, slice dropped all frames, empty bags)
    # still occupy a slot in episode_lengths with n_frames=0 but have no
    # file on disk. Naively using len(episode_lengths) as n_total would
    # produce val IDs that reference non-existent files (footgun seen in
    # Exp-25 fulltask conversion: val_episodes [849-893] but dataset only
    # had 720 valid episode files, so sidecar couldn't load the val set).
    val_fraction = float(getattr(args, "val_fraction", 0) or 0)
    if val_fraction > 0 and len(episode_lengths) > 0:
        valid_indices = [i for i, n in enumerate(episode_lengths) if n > 0]
        n_total = len(valid_indices)
        if n_total > 0:
            n_val = max(1, int(round(n_total * val_fraction)))
            # Use POST-filter (sequential) indices so they reference
            # the actual parquet files written by LeRobot
            # (episode_000000.parquet ... episode_NNNNNN.parquet —
            # numbered 0..n_total-1 in valid-bag order, not their
            # original input position). Earlier code used
            # `valid_indices[-n_val:]` which returned PRE-filter
            # positions like 1000+ for a 910-episode dataset; the val
            # files don't exist at those paths and val_loss_sidecar
            # crashed with "no data". See feedback memory
            # `val_episodes_offbyone`.
            #
            # Use a random shuffle (fixed seed) instead of "last by
            # index" because the input bag order can encode source
            # bias (e.g., when --mcap-dirs lists archive bags first
            # then current-collection bags, "last n_val" comes
            # entirely from the current-collection source and the
            # val set never sees the archive distribution). Random
            # shuffle gives unbiased coverage across whatever sources
            # were combined, while a fixed seed keeps the split
            # reproducible across re-runs of the same dataset.
            rng = random.Random(42)
            shuffled = list(range(n_total))
            rng.shuffle(shuffled)
            val_episodes = sorted(shuffled[-n_val:])
            val_payload = {
                "val_episodes": val_episodes,
                "n_total": n_total,
                "fraction": val_fraction,
                "rule": "random_shuffle_seed42",
            }
            val_path = Path(args.output_dir) / "meta" / "val_episodes.json"
            with open(val_path, "w") as f:
                json.dump(val_payload, f, indent=2)
            print(f"  Val split:    {len(val_episodes)}/{n_total} valid episodes "
                  f"({val_fraction * 100:.1f}%, indices "
                  f"{val_episodes[0]}..{val_episodes[-1]}) → {val_path.name}")

    total_gb = sum(Path(p).stat().st_size for p in mcap_paths) / 1e9
    print(f"\n{'=' * 70}")
    print(f"  Done in {overall_dt:.1f}s  ({total_gb:.1f} GB input)")
    print(f"  Episodes:     {len(episode_lengths)}")
    print(f"  Total frames: {total_frames}")
    print(f"  Throughput:   {total_gb / max(overall_dt, 0.01) * 1000:.0f} MB/s")
    print(f"  Output:       {args.output_dir}")
    print(f"{'=' * 70}\n")


def _print_episode_summary(ep_idx, mcap_path, n_frames, timings):
    name = Path(mcap_path).name
    p1 = timings.get("pass1_index", 0)
    p2 = timings.get("pass2_convert", 0)
    pq = timings.get("parquet", 0)
    total = p1 + p2 + pq
    print(f"  Ep {ep_idx:3d} | {name:40s} | {n_frames:5d} frames | "
          f"index {p1:.1f}s + convert {p2:.1f}s + pq {pq:.1f}s = {total:.1f}s")


def main():
    parser = argparse.ArgumentParser(
        description="FAST converter: AIC .mcap ROS 2 bags → LeRobot v2.1 dataset",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="See CONVERTER_README.md for full documentation and examples.",
    )
    parser.add_argument("--mcap-dirs", nargs="+", required=True)
    parser.add_argument("--output-dir", required=True)
    parser.add_argument("--repo-id", default="local/aic-cable-insertion")
    parser.add_argument("--config-yaml", default=None,
                        help="AIC config YAML (e.g. 10_trials_config.yaml) — auto-generates task descriptions")
    parser.add_argument("--tasks", nargs="+", default=None,
                        help="Manual task descriptions (one per --mcap-dirs entry). Alternative to --config-yaml")
    parser.add_argument("--scoring-dir", default=None,
                        help="Directory with *_scoring.yaml files for per-episode provenance")
    parser.add_argument("--action-mode",
        choices=["cartesian", "cartesian_pose", "joint"], default="cartesian_pose",
        help="cartesian_pose = 9D absolute TCP pose (xyz + 6D rotation, "
             "Zhou et al. 2019) — current SOTA, default; "
             "cartesian = 6D twist deltas (velocity, legacy); "
             "joint = 6D next-joint-positions.")
    parser.add_argument("--obs-state-mode",
        choices=["full", "joints", "joints_port", "joints_pose", "joints_pose_wrench"],
        default="joints_pose_wrench",
        help="State vector: 'joints_pose_wrench' = 21D (Exp-23 SOTA, default); "
             "'full' = 31D proprio+wrench; 'joints' = 6D joints only; "
             "'joints_port' = 9D (joints + port_entrance xyz in base_link, diagnostic); "
             "'joints_pose' = 15D (joints + TCP pose 6D rot)")
    parser.add_argument("--image-scale", type=float, default=0.25)
    parser.add_argument("--fps", type=int, default=20)
    parser.add_argument("--workers", type=int, default=1,
                        help="Parallel workers (set to CPU core count)")
    parser.add_argument("--slice-csv", default=None,
                        help="CSV from scripts/slice_for_specialist.py. When set, "
                             "each episode is truncated to start at the slice point "
                             "(approach→insertion handoff). Bags absent from the CSV "
                             "or with a non-empty drop_reason are skipped entirely.")
    parser.add_argument("--val-fraction", type=float, default=0.05,
                        help="Fraction of episodes (last by index) to mark as the "
                             "validation set. Written to <output_dir>/meta/"
                             "val_episodes.json for downstream consumption by "
                             "scripts/train.sh (excludes from training) and "
                             "scripts/val_loss_sidecar.py (computes per-checkpoint "
                             "val loss). Set to 0 to disable.")

    args = parser.parse_args()
    convert(args)


if __name__ == "__main__":
    main()