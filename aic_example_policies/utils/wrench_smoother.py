"""Shared wrench smoothing for cheatcode collection, ACT eval, and dataset build.

Why this exists. The aic_adapter aggregates wrench into the per-frame Observation
by picking the single nearest /fts_broadcaster/wrench sample to each camera
timestamp. That gives a single, jittery scalar per ~50ms frame. CheatCode's
descent control and ACT's state vector both end up consuming that one jittery
sample. Worse, the three call sites (CheatCode collection-time control,
mcap_to_lerobot dataset build, RunACT eval) are independently free to smooth or
not, which silently drifts train/eval distributions.

This module concentrates the smoothing in one place. Two interfaces share the
same algorithm:

  WrenchSmoother(node) — runtime: subscribes to /fts_broadcaster/wrench, keeps
    a timestamped ring buffer in the callback, and exposes smoothed(ref_time,
    window_s, mode) and peak_magnitude(...). For CheatCode and RunACT.

  smooth_messages(messages, ref_time, window_s, mode) — offline: pure function
    over a list of (timestamp_s, msg) tuples. For mcap_to_lerobot, where we
    iterate over bag messages directly.

Both implementations apply the *same* window selection and aggregation rule, so
mcap_to_lerobot's training-time smoothing is bit-equivalent to RunACT's eval-time
smoothing (modulo float order).
"""

from __future__ import annotations

import math
import os
from collections import deque
from typing import Iterable, Optional, Tuple

# Defaults. 100ms = two 20Hz control-tick windows. Causal: window is
# (ref_time - window_s, ref_time], so eval can match the offline build
# (offline knows ref_time and has all earlier samples). Picked over 50ms
# because the /fts_broadcaster/wrench publish rate is ~40Hz in sim, so a
# 50ms window only contains 2 samples — median is "pick one of two." 100ms
# gives ~4 samples, enough for a real per-axis median while only adding
# ~50ms of state lag.
DEFAULT_WINDOW_S = float(os.environ.get("WRENCH_SMOOTH_WINDOW_S", "0.10"))
DEFAULT_MODE = os.environ.get("WRENCH_SMOOTH_MODE", "median")
DEFAULT_TOPIC = os.environ.get("WRENCH_TOPIC", "/fts_broadcaster/wrench")
DEFAULT_BUFFER_LEN = 200  # ~2s at 100Hz; oldest samples evicted automatically


def _aggregate(values: list[float], mode: str) -> float:
    """Reduce a list of scalar samples to one. Robust to single-spike noise."""
    if not values:
        return 0.0
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


def _wrench_to_tuple(msg) -> Tuple[float, float, float, float, float, float]:
    """WrenchStamped → (fx, fy, fz, tx, ty, tz)."""
    w = msg.wrench
    return (
        w.force.x, w.force.y, w.force.z,
        w.torque.x, w.torque.y, w.torque.z,
    )


def _samples_in_window(
    timestamped: Iterable[Tuple[float, Tuple[float, ...]]],
    ref_time: float,
    window_s: float,
) -> list[Tuple[float, ...]]:
    """Return wrench tuples whose timestamp ∈ (ref_time - window_s, ref_time].

    Causal: never includes samples after ref_time. If no samples fall in the
    window, returns []; callers fall back to nearest-available.
    """
    lo = ref_time - window_s
    return [w for t, w in timestamped if lo < t <= ref_time]


def smooth_messages(
    messages: list[Tuple[float, object]],
    ref_time: float,
    window_s: float = DEFAULT_WINDOW_S,
    mode: str = DEFAULT_MODE,
) -> Optional[Tuple[float, float, float, float, float, float]]:
    """Offline smoothing for mcap_to_lerobot.

    Args:
        messages: list of (timestamp_s, WrenchStamped-msg) sorted by timestamp.
        ref_time: target frame timestamp (sec).
        window_s: causal window width.
        mode: per-axis aggregation ('median' | 'mean' | 'max' | 'min').

    Returns:
        6-tuple (fx, fy, fz, tx, ty, tz) or None if `messages` is empty.

    Notes:
        - When the window is empty (no messages in (ref_time - window_s, ref_time]),
          falls back to the single nearest message, so partial coverage at episode
          boundaries doesn't drop the frame.
        - Per-axis aggregation: each of the 6 wrench channels is reduced
          independently. For 'median' this is the per-axis median, not the
          median wrench-vector — fine when channels are roughly independent.
    """
    if not messages:
        return None

    # Build (ts, 6-tuple) list once. Caller could pre-build this.
    typed: list[Tuple[float, Tuple[float, ...]]] = [
        (t, _wrench_to_tuple(m)) for t, m in messages
    ]

    in_window = _samples_in_window(typed, ref_time, window_s)
    if not in_window:
        # No samples in causal window — fall back to nearest message
        # (could be before or after; usually before unless ref_time is at start).
        nearest = min(typed, key=lambda tw: abs(tw[0] - ref_time))
        return nearest[1]

    out = []
    for axis in range(6):
        out.append(_aggregate([s[axis] for s in in_window], mode))
    return tuple(out)


class WrenchSmoother:
    """Runtime wrench smoother for CheatCode and RunACT.

    Subscribes to /fts_broadcaster/wrench (fast — 100Hz+ in sim), maintains a
    timestamped ring buffer, and exposes smoothed(...) / peak_magnitude(...).

    Use this *instead of* polling get_observation() within a tight loop —
    Observation is rebuilt only on each camera frame (~20Hz), so polling it
    multiple times in a 50ms window returns duplicates.

    Example:
        self._wrench = WrenchSmoother(parent_node)
        ...
        # Per-axis median wrench at this control tick
        fx, fy, fz, tx, ty, tz = self._wrench.smoothed(
            self.time_now().nanoseconds * 1e-9, window_s=0.05
        )
        # Peak |F| (force magnitude) for spike detection
        peak_f = self._wrench.peak_magnitude(now_s, window_s=0.05)
    """

    def __init__(
        self,
        node,
        topic: str = DEFAULT_TOPIC,
        buffer_len: int = DEFAULT_BUFFER_LEN,
        callback_group=None,
    ):
        from geometry_msgs.msg import WrenchStamped
        self._node = node
        self._buf: deque[Tuple[float, Tuple[float, ...]]] = deque(maxlen=buffer_len)
        # SensorDataQoS-equivalent: best-effort, KEEP_LAST/depth=5. Wrench is
        # high-rate state — losing one is fine; blocking the publisher isn't.
        from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )
        # If callback_group is provided, the wrench callback runs in its own
        # (typically ReentrantCallbackGroup) pool — independent of the policy
        # main loop. Without this, the high-rate wrench updates queue up
        # while the descent loop holds the default group's mutex, leading to
        # stale force-feedback readings in median_magnitude/peak_magnitude.
        kwargs = {"callback_group": callback_group} if callback_group else {}
        self._sub = node.create_subscription(
            WrenchStamped, topic, self._on_wrench, qos, **kwargs,
        )
        self._n_received = 0

    def _on_wrench(self, msg) -> None:
        # Use header.stamp (sim time) so consumers can match against
        # self.time_now() / lookup_transform Time() — both sim-time-aware.
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self._buf.append((t, _wrench_to_tuple(msg)))
        self._n_received += 1

    def n_received(self) -> int:
        return self._n_received

    def latest(self) -> Optional[Tuple[float, float, float, float, float, float]]:
        if not self._buf:
            return None
        return self._buf[-1][1]

    def smoothed(
        self,
        ref_time: float,
        window_s: float = DEFAULT_WINDOW_S,
        mode: str = DEFAULT_MODE,
    ) -> Optional[Tuple[float, float, float, float, float, float]]:
        """Per-axis aggregation of wrench samples in (ref_time - window_s, ref_time].

        Returns None if the buffer is empty. If samples exist but none fall in
        the window, falls back to the most recent buffered sample (callers
        usually treat that as "use latest, log a warning").
        """
        # Snapshot the deque before reading. _on_wrench runs on a separate
        # callback group thread and may append() while we iterate, otherwise
        # producing "deque mutated during iteration" RuntimeError.
        snapshot = list(self._buf)
        if not snapshot:
            return None
        in_window = _samples_in_window(snapshot, ref_time, window_s)
        if not in_window:
            return snapshot[-1][1]
        out = []
        for axis in range(6):
            out.append(_aggregate([s[axis] for s in in_window], mode))
        return tuple(out)

    def peak_magnitude(
        self,
        ref_time: float,
        window_s: float = DEFAULT_WINDOW_S,
    ) -> float:
        """Max |F| (force magnitude) across samples in the causal window.

        Used by CheatCode descent for spike-triggered backoff: a single
        in-window sample with high magnitude flips backoff, even if the
        window's median is quiet.
        """
        snapshot = list(self._buf)
        if not snapshot:
            return 0.0
        in_window = _samples_in_window(snapshot, ref_time, window_s)
        if not in_window:
            in_window = [snapshot[-1][1]]
        return max(
            math.sqrt(s[0] ** 2 + s[1] ** 2 + s[2] ** 2) for s in in_window
        )

    def median_magnitude(
        self,
        ref_time: float,
        window_s: float = DEFAULT_WINDOW_S,
    ) -> float:
        """Median |F| over the window. CheatCode taper region uses this."""
        snapshot = list(self._buf)
        if not snapshot:
            return 0.0
        in_window = _samples_in_window(snapshot, ref_time, window_s)
        if not in_window:
            in_window = [snapshot[-1][1]]
        mags = sorted(
            math.sqrt(s[0] ** 2 + s[1] ** 2 + s[2] ** 2) for s in in_window
        )
        n = len(mags)
        if n % 2:
            return mags[n // 2]
        return 0.5 * (mags[n // 2 - 1] + mags[n // 2])

    def n_in_window(self, ref_time: float, window_s: float = DEFAULT_WINDOW_S) -> int:
        return len(_samples_in_window(list(self._buf), ref_time, window_s))
